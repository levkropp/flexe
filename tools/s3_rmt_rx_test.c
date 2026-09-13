/* Exercise the stock Arduino-ESP32 3.x RX API and ESP-IDF RMT ISR with an
 * externally injected, already-decoded pulse frame. */
#include "elf_symbols.h"
#include "flexe_session.h"
#include "memory.h"
#include "peripherals.h"

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define SUCCESS_MARKER 0x01A7C0DEu
#define MAX_CYCLES     3000000000ull
#define RMT_RX_CONF4   0x60016034u

volatile int emu_app_running = 1;

int main(int argc, char **argv)
{
    int argi = 1;
    int disable_jit = 0;
    if (argi < argc && strcmp(argv[argi], "--no-jit") == 0) {
        disable_jit = 1;
        argi++;
    }
    if (argc - argi != 3) {
        fprintf(stderr,
                "usage: %s [--no-jit] FIRMWARE.merged.bin FIRMWARE.elf ROM.elf\n",
                argv[0]);
        return 2;
    }
    elf_symbols_t *symbols = elf_symbols_load(argv[argi + 1]);
    uint32_t stage_addr = 0u;
    uint32_t result_addr = 0u;
    if (!symbols ||
        elf_symbols_find(symbols, "flexe_rmt_rx_stage", &stage_addr) != 0 ||
        elf_symbols_find(symbols, "flexe_rmt_rx_result", &result_addr) != 0) {
        fprintf(stderr, "error: fixture marker symbols are missing\n");
        elf_symbols_destroy(symbols);
        return 2;
    }
    flexe_session_config_t config = {
        .bin_path = argv[argi],
        .elf_path = argv[argi + 1],
        .rom_elf_path = argv[argi + 2],
        .disable_jit = disable_jit,
        .native_freertos = 1,
        .target = FLEXE_TARGET_ESP32S3,
        .unhandled_audit = 1,
    };
    flexe_session_t *session = flexe_session_create(&config);
    if (!session) {
        elf_symbols_destroy(symbols);
        return 2;
    }
    xtensa_cpu_t *cpu = flexe_session_cpu(session, 0);
    xtensa_mem_t *mem = flexe_session_mem(session);
    esp32_periph_t *periph = flexe_session_periph(session);
    const uint32_t expected[] = {
        10u | (1u << 15) | (12u << 16),
        8u | (1u << 31) | (9u << 16),
    };
    uint32_t stage = 0u;
    int injected = 0;
    while (cpu->cycle_count < MAX_CYCLES) {
        stage = mem_read32(mem, stage_addr);
        if (stage == 1u && !injected &&
            (mem_read32(mem, RMT_RX_CONF4) & 1u)) {
            size_t accepted = periph_rmt_rx_inject(periph, 4,
                                                   expected, 2u);
            if (accepted != 2u) {
                fprintf(stderr, "RX injection accepted %zu/2 symbols\n",
                        accepted);
                break;
            }
            injected = 1;
        }
        if (stage == SUCCESS_MARKER || (stage & 0xFFF00000u) == 0xBAD00000u)
            break;
        (void)flexe_session_run_core(session, 0, 10000);
        flexe_session_post_batch(session, 10000);
    }
    stage = mem_read32(mem, stage_addr);
    uint32_t count = mem_read32(mem, result_addr);
    uint32_t first = mem_read32(mem, result_addr + 4u);
    uint32_t second = mem_read32(mem, result_addr + 8u);
    uint64_t cycles = cpu->cycle_count;
    unsigned unhandled = periph_unhandled_count(periph);
    unsigned rmt_unhandled = 0u;
    for (size_t i = 0u; i < periph_unhandled_audit_count(periph); i++) {
        periph_unhandled_site_t site;
        if (periph_unhandled_audit_get(periph, i, &site) &&
            site.address >= 0x60016000u && site.address < 0x60017000u) {
            rmt_unhandled++;
            fprintf(stderr, "[s3-rmt-rx] unsupported RMT %c 0x%08X "
                    "pc=0x%08X value=0x%08X count=%llu\n",
                    site.write ? 'W' : 'R', site.address, site.pc,
                    site.first_value, (unsigned long long)site.count);
        }
    }
    int ok = injected && stage == SUCCESS_MARKER && count == 2u &&
             first == expected[0] && second == expected[1] &&
             rmt_unhandled == 0u;
    fprintf(stderr,
            "[s3-rmt-rx] stage=0x%08X injected=%d count=%u "
            "symbols=%08X,%08X cycles=%llu unhandled=%u "
            "rmt_unhandled_sites=%u\n",
            stage, injected, count, first, second,
            (unsigned long long)cycles, unhandled, rmt_unhandled);
    flexe_session_destroy(session);
    elf_symbols_destroy(symbols);
    return ok ? 0 : 1;
}
