/* Drive the public esp_timer API from guest code and check the callbacks land
 * at the right virtual time.
 *
 * The guest checks what only it can see -- the argument each callback got, and
 * that a stopped timer stays stopped. This side checks the numbers the guest
 * reported against what the wall-clock of emulated time says they should be,
 * because a dispatcher can be self-consistently wrong: fire every callback,
 * pass every guest-side assertion, and still run the firmware's notion of time
 * at the wrong rate.
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

#define SUCCESS_MARKER 0x71ED0C0Bu
#define MAX_CYCLES     4000000000ull
#define RESULT_COUNT   12u

#define PERIOD_US   10000u
#define ONESHOT_US  45000u
#define RUN_MS      200u
#define RESTART_MS  50u

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
        elf_symbols_find(symbols, "flexe_timer_stage", &stage_addr) != 0 ||
        elf_symbols_find(symbols, "flexe_timer_result", &result_addr) != 0) {
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
            fprintf(stderr, "[timer-fixture] stage=0x%08X cycles=%llu\n",
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
            fprintf(stderr, "[timer-fixture] tasks:\n%s", dump);
        fprintf(stderr, "[timer-fixture] pc=0x%08X cycles=%llu\n", cpu->pc,
                (unsigned long long)cpu->cycle_count);
    }
    int unhandled = periph_unhandled_count(flexe_session_periph(session));
    int unregistered = rom_stubs_unregistered_count(flexe_session_rom(session));

    /* How many periodic callbacks a correct dispatcher owes us over the run
     * window, and over the window after the restart. One either side covers
     * where the first tick lands relative to the delay() boundary. */
    unsigned want_run = (RUN_MS * 1000u) / PERIOD_US;
    unsigned want_restart = (RESTART_MS * 1000u) / PERIOD_US;

    printf("engine=%s stage=0x%08X periodic=%u oneshot=%u gap=%u..%u "
           "span=%uus oneshot_at=%uus stop_at=%uus restart=%u args=%u "
           "total=%uus unhandled=%d unregistered=%d\n",
           flexe_session_jit(session) ? "jit" : "interp", stage,
           r[0], r[2], r[3], r[4], r[5], r[6], r[7], r[9], r[8], r[10],
           unhandled, unregistered);

    bool count_ok = r[0] + 1u >= want_run && r[0] <= want_run + 1u &&
                    r[9] + 1u >= want_restart && r[9] <= want_restart + 1u;
    /* Each gap is one period, give or take the granularity of the batch the
     * dispatcher runs in. A burst (gap_min ~ 0) or a stall (gap_max several
     * periods) is what this catches, and neither shows up in the count. */
    bool gap_ok = r[3] >= PERIOD_US / 2u && r[4] <= PERIOD_US * 2u;
    /* The one-shot has a single deadline and no excuse to be early. */
    bool oneshot_ok = r[2] == 1u && r[6] >= ONESHOT_US &&
                      r[6] <= ONESHOT_US + PERIOD_US;
    /* The span between the first and last periodic callback has to match the
     * number of gaps in it, which is the drift check the per-gap bounds cannot
     * make: a timer 5% slow passes every individual gap and fails here. */
    unsigned want_span = (r[0] > 1u) ? (r[0] - 1u) * PERIOD_US : 0u;
    bool span_ok = r[0] > 1u && r[5] + PERIOD_US >= want_span &&
                   r[5] <= want_span + PERIOD_US;

    int ok = stage == SUCCESS_MARKER && count_ok && gap_ok && oneshot_ok &&
             span_ok && r[8] == 1u && r[0] == r[1] &&
             unhandled == 0 && unregistered == 0;
    if (!ok)
        fprintf(stderr, "[timer-fixture] count=%d gap=%d oneshot=%d span=%d "
                        "(want run=%u restart=%u span=%uus)\n",
                count_ok, gap_ok, oneshot_ok, span_ok, want_run, want_restart,
                want_span);

    flexe_session_destroy(session);
    elf_symbols_destroy(symbols);
    return ok ? 0 : 1;
}
