/* Run all four classic ESP32 general-purpose timers through the public
 * ESP-IDF legacy driver, including its genuine per-timer ISR callbacks. */
#include "elf_symbols.h"
#include "flexe_session.h"
#include "memory.h"
#include "peripherals.h"
#include "rom_stubs.h"

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define SUCCESS_MARKER 0x54494D47u
#define MAX_CYCLES     300000000ull
#define RESULT_COUNT   32u

volatile int emu_app_running = 1;

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
        elf_symbols_find(symbols, "flexe_timer_group_stage", &stage_addr) != 0 ||
        elf_symbols_find(symbols, "flexe_timer_group_result", &result_addr) != 0) {
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

    xtensa_cpu_t *cpu = flexe_session_cpu(session, 0);
    xtensa_mem_t *mem = flexe_session_mem(session);
    uint32_t stage = 0;
    uint32_t last_stage = UINT32_MAX;
    while (cpu->cycle_count < MAX_CYCLES) {
        stage = mem_read32(mem, stage_addr);
        if (stage != last_stage) {
            fprintf(stderr,
                    "[timer-group-fixture] stage=0x%08X cycles=%llu "
                    "pc=0x%08X\n",
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
    uint32_t result[RESULT_COUNT];
    for (unsigned i = 0; i < RESULT_COUNT; i++)
        result[i] = mem_read32(mem, result_addr + i * 4u);
    int unhandled = periph_unhandled_count(flexe_session_periph(session));
    int unregistered = rom_stubs_unregistered_count(
        flexe_session_rom(session));

    if (stage != SUCCESS_MARKER) {
        fprintf(stderr,
                "[timer-group-fixture] tg0 cfg=0x%08X/0x%08X "
                "count=0x%08X:%08X raw/st/ena=0x%X/0x%X/0x%X\n",
                mem_read32(mem, 0x3FF5F000u),
                mem_read32(mem, 0x3FF5F024u),
                mem_read32(mem, 0x3FF5F008u),
                mem_read32(mem, 0x3FF5F004u),
                mem_read32(mem, 0x3FF5F09Cu),
                mem_read32(mem, 0x3FF5F0A0u),
                mem_read32(mem, 0x3FF5F098u));
        fprintf(stderr,
                "[timer-group-fixture] tg1 cfg=0x%08X/0x%08X "
                "raw/st/ena=0x%X/0x%X/0x%X\n",
                mem_read32(mem, 0x3FF60000u),
                mem_read32(mem, 0x3FF60024u),
                mem_read32(mem, 0x3FF6009Cu),
                mem_read32(mem, 0x3FF600A0u),
                mem_read32(mem, 0x3FF60098u));
    }

    printf("engine=%s stage=0x%08X result=",
           flexe_session_jit(session) ? "jit" : "interp", stage);
    for (unsigned i = 0; i < RESULT_COUNT; i++)
        printf("%s%u", i ? "/" : "", result[i]);
    printf(" unhandled=%d unregistered=%d cycles=%llu\n",
           unhandled, unregistered,
           (unsigned long long)cpu->cycle_count);

    int api_ok = 1;
    for (unsigned i = 0; i < 20u; i++)
        if (result[i] != 0u) api_ok = 0;
    int callbacks_ok = result[20] >= 10u && result[21] == 4u &&
                       result[22] >= 10u && result[23] >= 5u;
    int behavior_ok = result[24] == result[25] && result[26] == 0u &&
                      (result[27] & 0xFFFFu) == 80u &&
                      ((result[27] >> 16) & 1u) == 0u &&
                      ((result[27] >> 20) & 1u) == 1u &&
                      ((result[27] >> 24) & 1u) == 0u &&
                      result[28] == 10000u && result[29] == 0x42u &&
                      result[30] == 0x12345678u;
    int ok = stage == SUCCESS_MARKER && api_ok && callbacks_ok &&
             behavior_ok && result[31] == 0u && unhandled == 0 &&
             unregistered == 0;

    flexe_session_destroy(session);
    elf_symbols_destroy(symbols);
    return ok ? 0 : 1;
}
