/* Replay ESP-IDF 5.3.2's stock continuous-ADC driver through S3 APB_SARADC,
 * trigger-8 GDMA, its ISR callback, and the driver's FreeRTOS ring buffer.
 * The host changes raw board samples between two complete conversion frames;
 * no guest function or firmware address is replaced. */
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

#define ADC_CONTINUOUS_DONE UINT32_C(0x41444344)
#define FRAME_BYTES 64u
#define FRAME_SAMPLES (FRAME_BYTES / sizeof(uint32_t))
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

static uint32_t expected_hash(uint16_t channel2, uint16_t channel3)
{
    uint32_t hash = UINT32_C(2166136261);
    for (unsigned index = 0u; index < FRAME_SAMPLES; index++) {
        unsigned channel = (index & 1u) ? 3u : 2u;
        uint32_t raw = (index & 1u) ? channel3 : channel2;
        uint32_t packed = raw | ((uint32_t)channel << 13u);
        hash = (hash ^ packed) * UINT32_C(16777619);
    }
    return hash;
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
        elf_symbols_find(symbols, "flexe_adc_continuous_stage",
                         &stage_address) != 0 ||
        elf_symbols_find(symbols, "flexe_adc_continuous_result",
                         &result_address) != 0) {
        fprintf(stderr,
                "error: continuous-ADC fixture marker symbols are missing\n");
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
    periph_set_adc_value(periph, 2, UINT16_C(0x123));
    periph_set_adc_value(periph, 3, UINT16_C(0xABC));

    bool injection_attempted = false;
    size_t injected = 0u;
    uint32_t stage = 0u;
    while (cpu0->cycle_count < MAX_CYCLES) {
        stage = mem_read32(mem, stage_address);
        if (stage == 3u && !injection_attempted) {
            injection_attempted = true;
            periph_set_adc_value(periph, 2, UINT16_C(0x456));
            periph_set_adc_value(periph, 3, UINT16_C(0x789));
            injected = periph_adc_continuous_inject(periph);
        }
        if ((stage == ADC_CONTINUOUS_DONE &&
             strstr(uart.data, "ADC_CONTINUOUS_DONE") != NULL) ||
            (stage & UINT32_C(0xFFFF0000)) == UINT32_C(0xBAD00000) ||
            strstr(uart.data, "ADC_CONTINUOUS_FAIL") != NULL ||
            uart.overflow || !cpu0->running ||
            (cpu1 && cpu1->debug_break))
            break;
        if (flexe_session_run_core(session, 0, BATCH) < 0) break;
        flexe_session_post_batch(session, BATCH);
    }

    stage = mem_read32(mem, stage_address);
    uint32_t callbacks = mem_read32(mem, result_address);
    uint32_t first_hash = mem_read32(mem, result_address + 4u);
    uint32_t second_hash = mem_read32(mem, result_address + 8u);
    uint32_t failure = mem_read32(mem, result_address + 36u);
    uint32_t expected_first = expected_hash(
        UINT16_C(0x123), UINT16_C(0xABC));
    uint32_t expected_second = expected_hash(
        UINT16_C(0x456), UINT16_C(0x789));
    uint64_t frames = periph_adc_continuous_frame_count(periph);
    uint64_t samples = periph_adc_continuous_sample_count(periph);
    bool active = periph_adc_continuous_active(periph);
    uint64_t jit_instructions = 0u;
    jit_state_t *jit = flexe_session_jit(session);
    if (jit) jit_instructions = jit_get_stats(jit)->insns_jitted;
    unsigned unhandled = (unsigned)periph_unhandled_count(periph);
    bool ok = stage == ADC_CONTINUOUS_DONE && callbacks == 2u &&
              first_hash == expected_first &&
              second_hash == expected_second && injection_attempted &&
              injected == FRAME_BYTES && frames == 2u && samples == 32u &&
              !active && unhandled == 0u &&
              strstr(uart.data, "ADC_CONTINUOUS_DONE") != NULL &&
              (disable_jit || jit_instructions != 0u);

    printf("%s: ESP-IDF S3 ADC continuous engine=%s stage=0x%08X "
           "callbacks=%u hashes=%08X,%08X inject=%zu frames=%llu "
           "samples=%llu active=%u unhandled=%u cycles=%llu "
           "jit_insns=%llu\n",
           ok ? "PASS" : "FAIL", disable_jit ? "interp" : "jit", stage,
           callbacks, first_hash, second_hash, injected,
           (unsigned long long)frames, (unsigned long long)samples, active,
           unhandled, (unsigned long long)cpu0->cycle_count,
           (unsigned long long)jit_instructions);
    if (!ok)
        fprintf(stderr,
                "failure=0x%08X expected_hashes=%08X,%08X "
                "injection_attempted=%u UART tail: %s\n",
                failure, expected_first, expected_second,
                injection_attempted,
                uart.data + (uart.length > 800u ? uart.length - 800u : 0u));

    flexe_session_destroy(session);
    elf_symbols_destroy(symbols);
    return ok ? 0 : 1;
}
