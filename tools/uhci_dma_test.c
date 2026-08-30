/* Run dual classic ESP32 UHCI controllers through a compiled Xtensa fixture
 * using Espressif's public register/lldesc ABI and genuine guest ISR path. */
#include "elf_symbols.h"
#include "flexe_session.h"
#include "memory.h"
#include "peripherals.h"
#include "rom_stubs.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define SUCCESS_MARKER 0x55484349u
#define MAX_CYCLES     120000000ull
#define RESULT_COUNT   16u
#define DMA_BYTES      16u

volatile int emu_app_running = 1;

typedef struct {
    size_t matched;
    size_t total;
    bool pattern_seen;
} uart_capture_t;

static void capture_uart(void *opaque, uint8_t byte) {
    uart_capture_t *capture = opaque;
    uint8_t expected = (uint8_t)(0x5Au ^ capture->matched);
    capture->total++;
    if (byte == expected) {
        capture->matched++;
        if (capture->matched == DMA_BYTES)
            capture->pattern_seen = true;
    } else {
        capture->matched = byte == 0x5Au ? 1u : 0u;
    }
}

int main(int argc, char **argv) {
    int argi = 1;
    int disable_jit = 0;
    if (argi < argc && strcmp(argv[argi], "--no-jit") == 0) {
        disable_jit = 1;
        argi++;
    }
    if (argc - argi != 2) {
        fprintf(stderr,
                "usage: %s [--no-jit] FIRMWARE.bin FIRMWARE.elf\n",
                argv[0]);
        return 2;
    }

    elf_symbols_t *symbols = elf_symbols_load(argv[argi + 1]);
    uint32_t stage_addr = 0;
    uint32_t result_addr = 0;
    if (!symbols ||
        elf_symbols_find(symbols, "flexe_uhci_stage", &stage_addr) != 0 ||
        elf_symbols_find(symbols, "flexe_uhci_result", &result_addr) != 0) {
        fprintf(stderr, "error: fixture marker symbols are missing\n");
        elf_symbols_destroy(symbols);
        return 2;
    }

    flexe_session_config_t config = {
        .bin_path = argv[argi],
        .elf_path = argv[argi + 1],
        .disable_jit = disable_jit,
    };
    flexe_session_t *session = flexe_session_create(&config);
    if (!session) {
        elf_symbols_destroy(symbols);
        return 2;
    }

    esp32_periph_t *periph = flexe_session_periph(session);
    uart_capture_t capture = {0};
    periph_set_uart_callback_num(periph, 0, capture_uart, &capture);
    xtensa_cpu_t *cpu = flexe_session_cpu(session, 0);
    xtensa_mem_t *mem = flexe_session_mem(session);
    uint32_t stage = 0;
    uint32_t last_stage = UINT32_MAX;
    size_t injected = 0;
    while (cpu->cycle_count < MAX_CYCLES) {
        stage = mem_read32(mem, stage_addr);
        if (stage != last_stage) {
            fprintf(stderr,
                    "[uhci-fixture] engine=%s stage=0x%08X cycles=%llu "
                    "pc=0x%08X\n",
                    flexe_session_jit(session) ? "jit" : "interp", stage,
                    (unsigned long long)cpu->cycle_count, cpu->pc);
            last_stage = stage;
        }
        if (stage == 2u && injected == 0u) {
            uint8_t input[DMA_BYTES];
            for (size_t index = 0; index < sizeof(input); ++index)
                input[index] = (uint8_t)(0xA0u ^ index);
            injected = periph_uart_rx_inject_num(
                periph, 0, input, sizeof(input));
        }
        if (stage == SUCCESS_MARKER ||
            (stage & 0xFFF00000u) == 0xBAD00000u)
            break;
        (void)flexe_session_run_core(session, 0, 10000);
        flexe_session_post_batch(session, 10000);
    }

    stage = mem_read32(mem, stage_addr);
    uint32_t result[RESULT_COUNT];
    for (unsigned index = 0; index < RESULT_COUNT; ++index)
        result[index] = mem_read32(mem, result_addr + index * 4u);
    int unhandled = periph_unhandled_count(periph);
    int unregistered = rom_stubs_unregistered_count(
        flexe_session_rom(session));

    printf("engine=%s stage=0x%08X result=",
           flexe_session_jit(session) ? "jit" : "interp", stage);
    for (unsigned index = 0; index < RESULT_COUNT; ++index)
        printf("%s0x%08X", index ? "/" : "", result[index]);
    printf(" tx=%zu/%zu/%d rx=%zu/%zu unhandled=%d unregistered=%d "
           "cycles=%llu\n",
           capture.matched, capture.total, capture.pattern_seen,
           injected, periph_uart_rx_pending_num(periph, 0), unhandled,
           unregistered, (unsigned long long)cpu->cycle_count);

    const uint32_t expected_irq =
        (1u << 4) | (1u << 5) | (1u << 7) | (1u << 8) | (1u << 13);
    int ok = stage == SUCCESS_MARKER && result[0] == 0u &&
             result[1] >= 1u && result[2] >= 1u &&
             (result[3] & expected_irq) == expected_irq &&
             result[4] == (DMA_BYTES | (1u << 16)) && result[5] == 0u &&
             result[6] != 0u && result[7] != 0u &&
             result[6] == result[8] && result[7] == result[9] &&
             (result[10] & ((1u << 22) | (1u << 19) | (1u << 9))) ==
                 ((1u << 22) | (1u << 19) | (1u << 9)) &&
             result[11] == (1u << 6) &&
             (result[12] & (1u << 31)) != 0u &&
             (result[13] & (1u << 31)) != 0u &&
             result[14] == 0xA5u && result[15] == 0u &&
             capture.pattern_seen && injected == DMA_BYTES &&
             periph_uart_rx_pending_num(periph, 0) == 0u &&
             unhandled == 0 && unregistered == 0;

    flexe_session_destroy(session);
    elf_symbols_destroy(symbols);
    return ok ? 0 : 1;
}
