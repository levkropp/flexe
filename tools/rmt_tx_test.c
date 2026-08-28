/* Run an unmodified ESP-IDF legacy RMT driver write through Flexe. The
 * firmware deliberately exceeds one hardware RAM block so the real ISR must
 * refill both halves before TX_END releases its callback. */
#include "elf_symbols.h"
#include "flexe_session.h"
#include "memory.h"
#include "peripherals.h"
#include "rom_stubs.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define SUCCESS_MARKER 0x524D54A0u
#define MAX_CYCLES     50000000ull
#define ITEM_COUNT     96u

volatile int emu_app_running = 1;

typedef struct {
    uint32_t items[ITEM_COUNT];
    size_t count;
    unsigned chunks;
    unsigned finishes;
    uint32_t tick_hz;
    uint32_t carrier_hz;
} rmt_capture_t;

static void capture_rmt(void *opaque, int channel, const uint32_t *items,
                        size_t count, uint32_t tick_hz,
                        uint32_t carrier_hz, bool finished) {
    rmt_capture_t *capture = opaque;
    if (channel != 0) return;
    capture->chunks++;
    capture->tick_hz = tick_hz;
    capture->carrier_hz = carrier_hz;
    if (finished) capture->finishes++;
    for (size_t i = 0; i < count && capture->count < ITEM_COUNT; i++)
        capture->items[capture->count++] = items[i];
}

static uint32_t expected_item(size_t index) {
    uint32_t duration0 = 5u + (uint32_t)(index % 7u);
    uint32_t duration1 = 9u + (uint32_t)(index % 5u);
    uint32_t level0 = (index & 1u) ? 1u << 15 : 0u;
    uint32_t level1 = (index & 1u) ? 0u : 1u << 31;
    return duration0 | level0 | (duration1 << 16) | level1;
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
    const char *firmware_path = argv[argi];
    const char *elf_path = argv[argi + 1];

    elf_symbols_t *symbols = elf_symbols_load(elf_path);
    uint32_t stage_addr = 0;
    uint32_t result_addr = 0;
    if (!symbols ||
        elf_symbols_find(symbols, "flexe_rmt_stage", &stage_addr) != 0 ||
        elf_symbols_find(symbols, "flexe_rmt_result", &result_addr) != 0) {
        fprintf(stderr, "error: fixture marker symbols are missing\n");
        elf_symbols_destroy(symbols);
        return 2;
    }

    flexe_session_config_t config = {
        .bin_path = firmware_path,
        .elf_path = elf_path,
        .disable_jit = disable_jit,
    };
    flexe_session_t *session = flexe_session_create(&config);
    if (!session) {
        elf_symbols_destroy(symbols);
        return 2;
    }

    rmt_capture_t capture = {0};
    esp32_periph_t *periph = flexe_session_periph(session);
    if (periph_set_rmt_tx_callback(periph, 0, capture_rmt, &capture) != 0) {
        fprintf(stderr, "error: could not attach virtual RMT endpoint\n");
        flexe_session_destroy(session);
        elf_symbols_destroy(symbols);
        return 2;
    }

    xtensa_cpu_t *cpu = flexe_session_cpu(session, 0);
    xtensa_mem_t *mem = flexe_session_mem(session);
    uint32_t stage = 0;
    uint32_t last_stage = UINT32_MAX;
    while (cpu->cycle_count < MAX_CYCLES) {
        stage = mem_read32(mem, stage_addr);
        if (stage != last_stage) {
            fprintf(stderr, "[rmt-fixture] stage=0x%08X cycles=%llu pc=0x%08X\n",
                    stage, (unsigned long long)cpu->cycle_count, cpu->pc);
            last_stage = stage;
        }
        if (stage == SUCCESS_MARKER ||
            (stage & 0xFFF00000u) == 0xBAD00000u)
            break;
        (void)flexe_session_run_core(session, 0, 10000);
        flexe_session_post_batch(session, 10000);
    }

    stage = mem_read32(mem, stage_addr);
    uint32_t results[5];
    for (unsigned i = 0; i < 5; i++)
        results[i] = mem_read32(mem, result_addr + i * 4u);
    if (stage != SUCCESS_MARKER) {
        fprintf(stderr,
                "[rmt-fixture] conf0=0x%08X conf1=0x%08X "
                "status=0x%08X addr=%u raw=0x%08X st=0x%08X "
                "ena=0x%08X limit=%u apb=0x%08X mem0=0x%08X "
                "ccount=%u cpu_mhz=%u\n",
                mem_read32(mem, 0x3FF56020u),
                mem_read32(mem, 0x3FF56024u),
                mem_read32(mem, 0x3FF56060u),
                mem_read32(mem, 0x3FF56080u),
                mem_read32(mem, 0x3FF560A0u),
                mem_read32(mem, 0x3FF560A4u),
                mem_read32(mem, 0x3FF560A8u),
                mem_read32(mem, 0x3FF560D0u),
                mem_read32(mem, 0x3FF560F0u),
                mem_read32(mem, 0x3FF56800u), cpu->ccount,
                mem_read32(mem, 0x3FFE01E0u));
        for (int i = 0;; i++) {
            const char *name = NULL;
            uint32_t addr = 0;
            uint32_t calls = 0;
            if (rom_stubs_get_stats(flexe_session_rom(session), i, &name,
                                    &addr, &calls) != 0)
                break;
            if ((name && strcmp(name, "vTaskDelay") == 0) ||
                addr == 0x4008A2C8u)
                fprintf(stderr,
                        "[rmt-fixture] vTaskDelay=0x%08X calls=%u\n",
                        addr, calls);
        }
    }
    bool pattern = capture.count == ITEM_COUNT;
    for (size_t i = 0; pattern && i < ITEM_COUNT; i++)
        pattern = capture.items[i] == expected_item(i);
    int unhandled = periph_unhandled_count(periph);
    int unregistered = rom_stubs_unregistered_count(
        flexe_session_rom(session));

    printf("engine=%s stage=0x%08X result=%u/%u/%u/%u/%u "
           "chunks=%u items=%zu pattern=%d finished=%u "
           "tick_hz=%u carrier_hz=%u unhandled=%d unregistered=%d "
           "cycles=%llu\n",
           flexe_session_jit(session) ? "jit" : "interp",
           stage, results[0], results[1], results[2], results[3], results[4],
           capture.chunks, capture.count, pattern, capture.finishes,
           capture.tick_hz, capture.carrier_hz, unhandled, unregistered,
           (unsigned long long)cpu->cycle_count);

    int ok = stage == SUCCESS_MARKER && results[0] == 0 &&
             results[1] == 0 && results[2] == 0 && results[3] == 1 &&
             results[4] == 0 && capture.chunks >= 3 && pattern &&
             capture.finishes == 1 && capture.tick_hz == 1000000u &&
             capture.carrier_hz == 0 && unhandled == 0 && unregistered == 0;

    flexe_session_destroy(session);
    elf_symbols_destroy(symbols);
    return ok ? 0 : 1;
}
