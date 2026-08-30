/* Run both classic ESP32 legacy FRC timers through Espressif's public
 * register definitions and genuine guest interrupt handlers. */
#include "elf_symbols.h"
#include "flexe_session.h"
#include "memory.h"
#include "peripherals.h"
#include "rom_stubs.h"

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define SUCCESS_MARKER 0x46524332u
#define MAX_CYCLES     300000000ull
#define RESULT_COUNT   24u

#define FRC_BASE       0x3FF47000u
#define FRC_TIMER(n)   (FRC_BASE + (n) * 0x20u)

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
        elf_symbols_find(symbols, "flexe_frc_timer_stage", &stage_addr) != 0 ||
        elf_symbols_find(symbols, "flexe_frc_timer_result", &result_addr) != 0) {
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
                    "[frc-timer-fixture] stage=0x%08X cycles=%llu "
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
                "[frc-timer-fixture] frc1 load/count/cfg=0x%08X/"
                "0x%08X/0x%08X int=0x%08X\n",
                mem_read32(mem, FRC_TIMER(0) + 0x00u),
                mem_read32(mem, FRC_TIMER(0) + 0x04u),
                mem_read32(mem, FRC_TIMER(0) + 0x08u),
                mem_read32(mem, FRC_TIMER(0) + 0x0Cu));
        fprintf(stderr,
                "[frc-timer-fixture] frc2 load/count/cfg=0x%08X/"
                "0x%08X/0x%08X int/alarm=0x%08X/0x%08X\n",
                mem_read32(mem, FRC_TIMER(1) + 0x00u),
                mem_read32(mem, FRC_TIMER(1) + 0x04u),
                mem_read32(mem, FRC_TIMER(1) + 0x08u),
                mem_read32(mem, FRC_TIMER(1) + 0x0Cu),
                mem_read32(mem, FRC_TIMER(1) + 0x10u));
    }

    printf("engine=%s stage=0x%08X result=",
           flexe_session_jit(session) ? "jit" : "interp", stage);
    for (unsigned i = 0; i < RESULT_COUNT; i++)
        printf("%s%u", i ? "/" : "", result[i]);
    printf(" unhandled=%d unregistered=%d cycles=%llu\n",
           unhandled, unregistered,
           (unsigned long long)cpu->cycle_count);

    const uint32_t frc_config_mask = 0x1CFu;
    int setup_ok = result[0] == 0u && result[1] == 0u &&
                   result[2] == 3u && result[3] == 80000u;
    int callbacks_ok = result[6] >= 5u && result[7] >= 5u &&
                       result[8] >= 0x1000u && result[8] <= 0x20000u &&
                       result[9] != 0x1000u && result[20] >= 78000u &&
                       result[20] <= 80000u;
    int prescalers_ok = (result[10] & frc_config_mask) == 0x85u &&
                        result[11] > result[12] &&
                        result[13] == result[14] &&
                        (result[15] & frc_config_mask) == 0x88u &&
                        result[17] > result[16];
    int stopped_ok = (result[18] & 0x180u) == 0u &&
                     (result[19] & 0x180u) == 0u;
    int ok = stage == SUCCESS_MARKER && setup_ok && callbacks_ok &&
             prescalers_ok && stopped_ok && result[23] == 0u &&
             unhandled == 0 && unregistered == 0;

    flexe_session_destroy(session);
    elf_symbols_destroy(symbols);
    return ok ? 0 : 1;
}
