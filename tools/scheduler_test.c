/* Periodic delays, a send blocking on a full queue, recursive mutexes and
 * suspend/resume.
 *
 * The fixture advances a stage per feature, so a hang identifies the feature
 * rather than just the run. Timing is checked as well as status throughout: a
 * fixed-rate loop that does not block returns the right number of iterations
 * instantly, and a send woken by its own timeout looks identical to one woken
 * by the receive unless the clock is consulted.
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

#define SUCCESS_MARKER 0x5CED0C0Bu
#define MAX_CYCLES     8000000000ull
#define RESULT_COUNT   16u

#define PERIOD_US   20000u
#define PERIODS     10u
#define DRAIN_US    50000u
#define IMMEDIATE_US 5000u

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
        elf_symbols_find(symbols, "flexe_sched_stage", &stage_addr) != 0 ||
        elf_symbols_find(symbols, "flexe_sched_result", &result_addr) != 0) {
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
    while (cpu->cycle_count < MAX_CYCLES) {
        stage = mem_read32(mem, stage_addr);
        if (stage != last_stage) {
            fprintf(stderr, "[scheduler] stage=0x%08X cycles=%llu\n",
                    stage, (unsigned long long)cpu->cycle_count);
            last_stage = stage;
        }
        if (stage == SUCCESS_MARKER || (stage & 0xFFF00000u) == 0xBAD00000u)
            break;
        (void)flexe_session_run_core(session, 0, 10000);
        flexe_session_post_batch(session, 10000);
    }

    stage = mem_read32(mem, stage_addr);
    uint32_t r[RESULT_COUNT];
    for (unsigned i = 0; i < RESULT_COUNT; i++)
        r[i] = mem_read32(mem, result_addr + i * 4u);
    if (stage != SUCCESS_MARKER) {
        char dump[4096];
        if (freertos_stubs_dump_tasks(flexe_session_frt(session), dump,
                                      sizeof dump) > 0)
            fprintf(stderr, "[scheduler] tasks:\n%s", dump);
        fprintf(stderr, "[scheduler] pc=0x%08X cycles=%llu\n", cpu->pc,
                (unsigned long long)cpu->cycle_count);
    }
    int unhandled = periph_unhandled_count(flexe_session_periph(session));
    int unregistered = rom_stubs_unregistered_count(flexe_session_rom(session));

    printf("engine=%s stage=0x%08X periodic=%uus fullsend=%u sendok=%u@%uus "
           "rmutex=%uus worker=%u/%u/%u/%u unhandled=%d unregistered=%d\n",
           flexe_session_jit(session) ? "jit" : "interp", stage,
           r[0], r[1], r[2], r[3], r[4], r[5], r[6], r[7], r[8],
           unhandled, unregistered);

    /* Ten periods of 20 ms, give or take a tick at each end. */
    uint32_t want_periodic = PERIODS * PERIOD_US;
    bool periodic_ok = r[0] + PERIOD_US >= want_periodic &&
                       r[0] <= want_periodic + PERIOD_US;
    /* A send into a full queue with no timeout fails; one with a timeout is
     * woken by the drain, not by its own expiry. */
    bool fullsend_ok = r[1] == 0u && r[2] == 1u &&
                       r[3] >= DRAIN_US / 2u && r[3] <= DRAIN_US * 2u;
    /* Two nested takes of a recursive mutex must not block at all. */
    bool rmutex_ok = r[4] < IMMEDIATE_US;
    /* The worker counts, stops dead while suspended, and counts again. */
    bool worker_ok = r[5] > 0u && r[7] == r[6] && r[8] > r[7];

    int ok = stage == SUCCESS_MARKER && periodic_ok && fullsend_ok &&
             rmutex_ok && worker_ok && unhandled == 0 && unregistered == 0;
    if (!ok)
        fprintf(stderr, "[scheduler] periodic=%d fullsend=%d rmutex=%d "
                        "worker=%d (want periodic=%uus)\n",
                periodic_ok, fullsend_ok, rmutex_ok, worker_ok, want_periodic);

    flexe_session_destroy(session);
    elf_symbols_destroy(symbols);
    return ok ? 0 : 1;
}
