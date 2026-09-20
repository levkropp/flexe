/* Replay ESP-IDF 5.3.2's stock ESP32-S3 touch-v2 driver through RTC/SENS
 * MMIO and its RTC-core ISR. Host samples emulate the board-side electrode. */
#include "elf_symbols.h"
#include "flexe_session.h"
#include "jit.h"
#include "memory.h"
#include "peripherals.h"
#include "target.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define TOUCH_DONE UINT32_C(0x544F5543)
#define MAX_CYCLES UINT64_C(3000000000)
#define BATCH 10000u

volatile int emu_app_running = 1;

typedef struct {
    char data[32768];
    size_t length;
    bool overflow;
} uart_capture_t;

static void capture_uart(void *ctx, uint8_t byte)
{
    uart_capture_t *capture = ctx;
    if (capture->length + 1u >= sizeof(capture->data)) {
        capture->overflow = true;
        return;
    }
    capture->data[capture->length++] = (char)byte;
    capture->data[capture->length] = '\0';
}

int main(int argc, char **argv)
{
    int argi = 1;
    bool disable_jit = false;
    if (argi < argc && strcmp(argv[argi], "--no-jit") == 0) {
        disable_jit = true;
        argi++;
    }
    if (argc - argi != 3) {
        fprintf(stderr,
                "usage: %s [--no-jit] FIRMWARE.bin FIRMWARE.elf ROM.elf\n",
                argv[0]);
        return 2;
    }

    elf_symbols_t *symbols = elf_symbols_load(argv[argi + 1]);
    uint32_t stage_address = 0u;
    uint32_t result_address = 0u;
    if (!symbols ||
        elf_symbols_find(symbols, "flexe_touch_stage", &stage_address) != 0 ||
        elf_symbols_find(symbols, "flexe_touch_result", &result_address) != 0) {
        fprintf(stderr, "error: touch fixture marker symbols are missing\n");
        elf_symbols_destroy(symbols);
        return 2;
    }

    uart_capture_t uart = {0};
    flexe_session_config_t config = {
        .bin_path = argv[argi],
        .elf_path = argv[argi + 1],
        .rom_elf_path = argv[argi + 2],
        .native_freertos = 1,
        .disable_jit = disable_jit,
        .target = FLEXE_TARGET_ESP32S3,
        .unhandled_audit = 1,
        .uart_cb = capture_uart,
        .uart_ctx = &uart,
    };
    flexe_session_t *session = flexe_session_create(&config);
    if (!session) {
        elf_symbols_destroy(symbols);
        return 2;
    }

    esp32_periph_t *periph = flexe_session_periph(session);
    xtensa_cpu_t *cpu0 = flexe_session_cpu(session, 0);
    xtensa_cpu_t *cpu1 = flexe_session_cpu(session, 1);
    xtensa_mem_t *mem = flexe_session_mem(session);
    periph_touch_set_value(periph, 4, 1000u);

    bool active_injected = false;
    bool inactive_injected = false;
    uint32_t stage = 0u;
    while (cpu0->cycle_count < MAX_CYCLES) {
        stage = mem_read32(mem, stage_address);
        if (stage == 1u && !active_injected) {
            active_injected = true;
            periph_touch_set_value(periph, 4, 1400u);
        }
        if (stage == 2u && !inactive_injected) {
            inactive_injected = true;
            periph_touch_set_value(periph, 4, 1050u);
        }
        if ((stage == TOUCH_DONE && strstr(uart.data, "TOUCH_DONE")) ||
            (stage & UINT32_C(0xFFFF0000)) == UINT32_C(0xBAD00000) ||
            strstr(uart.data, "TOUCH_FAIL") || uart.overflow ||
            !cpu0->running || (cpu1 && cpu1->debug_break))
            break;
        if (flexe_session_run_core(session, 0, BATCH) < 0) break;
        flexe_session_post_batch(session, BATCH);
    }

    stage = mem_read32(mem, stage_address);
    uint32_t result[12];
    for (unsigned i = 0u; i < 12u; i++)
        result[i] = mem_read32(mem, result_address + i * 4u);
    uint64_t jit_instructions = 0u;
    jit_state_t *jit = flexe_session_jit(session);
    if (jit) jit_instructions = jit_get_stats(jit)->insns_jitted;
    unsigned unhandled = (unsigned)periph_unhandled_count(periph);
    uint64_t scans = periph_touch_scan_count(periph);
    uint32_t model_active = periph_touch_active_mask(periph);
    bool running = periph_touch_running(periph);
    bool ok = stage == TOUCH_DONE && active_injected && inactive_injected &&
              result[0] == 1u && result[1] == 1u &&
              result[3] == (1u << 4) && result[4] == 1400u &&
              result[6] == 0u && result[7] == 1050u &&
              result[8] == 1000u && result[9] == 1000u &&
              result[10] == 0u && result[11] == 0u &&
              scans >= 3u && model_active == 0u && !running &&
              unhandled == 0u && strstr(uart.data, "TOUCH_DONE") &&
              (disable_jit || jit_instructions != 0u);

    printf("%s: ESP-IDF S3 touch-v2 engine=%s stage=0x%08X "
           "active=%u inactive=%u baseline=%u/%u raws=%u/%u "
           "status=%u/%u teardown=%u scans=%llu model_active=%u "
           "running=%u unhandled=%u cycles=%llu "
           "jit_insns=%llu\n",
           ok ? "PASS" : "FAIL", disable_jit ? "interp" : "jit", stage,
           result[0], result[1], result[8], result[9], result[4], result[7],
           result[3], result[6], result[10],
           (unsigned long long)scans, model_active, running ? 1u : 0u,
           unhandled,
           (unsigned long long)cpu0->cycle_count,
           (unsigned long long)jit_instructions);
    if (!ok) {
        fprintf(stderr, "failure=0x%08X UART tail: %s\n", result[11],
                uart.data + (uart.length > 1000u ? uart.length - 1000u : 0u));
        for (size_t i = 0u; i < periph_unhandled_audit_count(periph); i++) {
            periph_unhandled_site_t site;
            if (!periph_unhandled_audit_get(periph, i, &site)) break;
            fprintf(stderr,
                    "unsupported %s addr=0x%08X pc=0x%08X value=0x%08X "
                    "core=%u count=%llu\n",
                    site.write ? "write" : "read", site.address, site.pc,
                    site.first_value, site.core,
                    (unsigned long long)site.count);
        }
    }

    flexe_session_destroy(session);
    elf_symbols_destroy(symbols);
    return ok ? 0 : 1;
}
