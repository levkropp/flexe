/* Hold the emulator's three clocks to one timeline.
 *
 * The guest measures how far CCOUNT and esp_timer_get_time() each move across
 * four different kinds of wait; this side requires the ratio between them to be
 * the CPU frequency in every one. The guest cannot make that check itself in
 * any meaningful way -- it would be comparing two numbers it has no independent
 * reference for -- and every existing gate watches only elapsed time, which is
 * the clock the fast-forward paths already get right.
 *
 * What this catches is a stub that models a wait by crediting elapsed guest
 * time without moving CCOUNT. The delay still returns after the right interval,
 * so the firmware looks fine, but CCOMPARE is scheduled against CCOUNT: on a
 * firmware running its own FreeRTOS the tick quietly runs slow, and every
 * timeout measured against it runs long.
 */
#include "elf_symbols.h"
#include "flexe_session.h"
#include "memory.h"
#include "peripherals.h"
#include "rom_stubs.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define SUCCESS_MARKER 0xC10CC0DEu
#define MAX_CYCLES     4000000000ull
#define RESULT_COUNT   16u

/* Each phase asks for 50 ms. The ratio is allowed 12%: the fast-forward paths
 * land on batch and event boundaries rather than exactly on the requested
 * interval, and the spin phase pays for however much of a batch is left when
 * its deadline passes. A clock that is not advancing at all reads as 0, and a
 * stub crediting only one of the two is out by far more than this. */
#define RATIO_TOLERANCE_PCT 12

static const char *PHASE_NAME[4] = {
    "spin", "rom_delay", "arduino_delay", "task_delay"
};

volatile int emu_app_running = 1;

int main(int argc, char **argv) {
    int argi = 1;
    int disable_jit = 0;
    if (argi < argc && strcmp(argv[argi], "--no-jit") == 0) {
        disable_jit = 1;
        argi++;
    }
    if (argc - argi != 2) {
        fprintf(stderr, "usage: %s [--no-jit] FIRMWARE.bin FIRMWARE.elf\n",
                argv[0]);
        return 2;
    }

    elf_symbols_t *symbols = elf_symbols_load(argv[argi + 1]);
    uint32_t stage_addr = 0, result_addr = 0;
    if (!symbols ||
        elf_symbols_find(symbols, "flexe_clock_stage", &stage_addr) != 0 ||
        elf_symbols_find(symbols, "flexe_clock_result", &result_addr) != 0) {
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
    uint32_t stage = 0, last_stage = UINT32_MAX;
    bool budget_stop = true;
    while (cpu->cycle_count < MAX_CYCLES) {
        stage = mem_read32(mem, stage_addr);
        if (stage != last_stage) {
            fprintf(stderr, "[clock-fixture] stage=0x%08X cycles=%llu\n",
                    stage, (unsigned long long)cpu->cycle_count);
            last_stage = stage;
        }
        if (stage == SUCCESS_MARKER || (stage & 0xFFF00000u) == 0xBAD00000u) {
            budget_stop = false;
            break;
        }
        (void)flexe_session_run_core(session, 0, 10000);
        flexe_session_post_batch(session, 10000);
    }

    stage = mem_read32(mem, stage_addr);
    uint32_t r[RESULT_COUNT];
    for (unsigned i = 0; i < RESULT_COUNT; i++)
        r[i] = mem_read32(mem, result_addr + i * 4u);

    /* A run that simply ran out of budget looks exactly like a hang unless it
     * says so. */
    if (budget_stop)
        fprintf(stderr, "[clock-fixture] exited on cycle budget (%llu) at "
                        "pc=0x%08X\n", MAX_CYCLES, cpu->pc);

    int unhandled = periph_unhandled_count(flexe_session_periph(session));
    int unregistered = rom_stubs_unregistered_count(flexe_session_rom(session));
    uint32_t mhz = r[0];

    printf("engine=%s stage=0x%08X cpu=%uMHz",
           flexe_session_jit(session) ? "jit" : "interp", stage, mhz);

    bool ratios_ok = mhz >= 10u && mhz <= 240u;
    for (unsigned p = 0; p < 4; p++) {
        uint32_t cyc = r[2 + p * 2], us = r[3 + p * 2];
        /* Reported as a frequency so a failure names the number that is wrong,
         * rather than leaving a ratio to be worked out from two counters. */
        unsigned observed = us ? (unsigned)((uint64_t)cyc / us) : 0u;
        printf(" %s=%uc/%uus(%uMHz)", PHASE_NAME[p], cyc, us, observed);
        if (!us || !cyc) { ratios_ok = false; continue; }
        unsigned lo = mhz * (100u - RATIO_TOLERANCE_PCT) / 100u;
        unsigned hi = mhz * (100u + RATIO_TOLERANCE_PCT) / 100u;
        if (observed < lo || observed > hi) ratios_ok = false;
    }
    printf(" unhandled=%d unregistered=%d\n", unhandled, unregistered);

    int ok = stage == SUCCESS_MARKER && ratios_ok &&
             unhandled == 0 && unregistered == 0;
    if (!ok)
        fprintf(stderr, "[clock-fixture] stage_ok=%d ratios_ok=%d "
                        "(CCOUNT must track guest time within %d%% in every "
                        "phase)\n",
                stage == SUCCESS_MARKER, ratios_ok, RATIO_TOLERANCE_PCT);

    flexe_session_destroy(session);
    elf_symbols_destroy(symbols);
    return ok ? 0 : 1;
}
