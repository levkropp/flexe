/* Drive FreeRTOS software timers from guest code and check when the callbacks
 * land.
 *
 * Unlike the rest of the FreeRTOS surface, xTimerCreate and friends are not
 * hooked, so this exercises the guest's own timer service running on top of
 * Flexe's stubbed scheduler and queues. The guest checks the timer IDs and
 * that a stopped timer stays stopped; this side checks the counts and the
 * inter-callback gaps against what the periods say they should be.
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

#define SUCCESS_MARKER 0x71E20C0Bu
#define MAX_CYCLES     6000000000ull
#define RESULT_COUNT   12u

#define PERIOD_MS    20u
#define ONESHOT_MS   70u
#define RUN_MS       300u
#define NEWPERIOD_MS 40u
#define NEWRUN_MS    200u

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
        elf_symbols_find(symbols, "flexe_frt_timer_stage", &stage_addr) != 0 ||
        elf_symbols_find(symbols, "flexe_frt_timer_result", &result_addr) != 0) {
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
            fprintf(stderr, "[frt-timer] stage=0x%08X cycles=%llu\n",
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
            fprintf(stderr, "[frt-timer] tasks:\n%s", dump);
        fprintf(stderr, "[frt-timer] pc=0x%08X cycles=%llu\n", cpu->pc,
                (unsigned long long)cpu->cycle_count);
    }
    int unhandled = periph_unhandled_count(flexe_session_periph(session));
    int unregistered = rom_stubs_unregistered_count(flexe_session_rom(session));

    printf("engine=%s stage=0x%08X reload=%u oneshot=%u gap=%u..%u span=%uus "
           "oneshot_at=%uus active=%u/%u ids=%u newperiod=%u newgap=%u "
           "unhandled=%d unregistered=%d\n",
           flexe_session_jit(session) ? "jit" : "interp", stage,
           r[0], r[2], r[3], r[4], r[5], r[8], r[6], r[7], r[9], r[10], r[11],
           unhandled, unregistered);

    /* A tick-resolution timer can be a tick early or late at each end. */
    unsigned want_reload = RUN_MS / PERIOD_MS;
    unsigned want_new = NEWRUN_MS / NEWPERIOD_MS;
    bool count_ok = r[0] + 2u >= want_reload && r[0] <= want_reload + 2u &&
                    r[10] + 1u >= want_new && r[10] <= want_new + 1u;
    bool gap_ok = r[3] >= PERIOD_MS * 500u && r[4] <= PERIOD_MS * 2000u;
    bool oneshot_ok = r[2] == 1u && r[8] >= ONESHOT_MS * 1000u &&
                      r[8] <= (ONESHOT_MS + PERIOD_MS) * 1000u;
    /* The period change has to actually take effect: at the new period no gap
     * may still be the old one. */
    bool newperiod_ok = r[11] >= NEWPERIOD_MS * 500u &&
                        r[11] <= NEWPERIOD_MS * 2000u;
    /* A fired one-shot is inactive; an auto-reload left running is active. */
    bool active_ok = r[6] == 0u && r[7] == 1u;

    int ok = stage == SUCCESS_MARKER && count_ok && gap_ok && oneshot_ok &&
             newperiod_ok && active_ok && r[9] == 1u && r[0] == r[1] &&
             unhandled == 0 && unregistered == 0;
    if (!ok)
        fprintf(stderr, "[frt-timer] count=%d gap=%d oneshot=%d newperiod=%d "
                        "active=%d (want reload=%u new=%u)\n",
                count_ok, gap_ok, oneshot_ok, newperiod_ok, active_ok,
                want_reload, want_new);

    flexe_session_destroy(session);
    elf_symbols_destroy(symbols);
    return ok ? 0 : 1;
}
