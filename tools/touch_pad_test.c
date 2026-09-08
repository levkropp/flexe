/* Drive the capacitive touch controller from the host side.
 *
 * The guest checks what only it can see -- that a read reflects the injected
 * count, that the counts fall on touch, and that its registered callback runs.
 * This side owns the pad values and checks the numbers the guest reported are
 * the ones that were injected, which a self-consistent-but-wrong model would
 * otherwise pass: a controller that always answers zero satisfies "below
 * threshold" forever and never distinguishes pressed from released.
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

#define SUCCESS_MARKER 0x70A0C4EDu
#define MAX_CYCLES     4000000000ull
#define RESULT_COUNT   8u

#define TOUCH_PAD      0
#define RELEASED_VALUE 1000u
#define PRESSED_VALUE  100u

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
    uint32_t stage_addr = 0, ack_addr = 0, result_addr = 0;
    if (!symbols ||
        elf_symbols_find(symbols, "flexe_touch_stage", &stage_addr) != 0 ||
        elf_symbols_find(symbols, "flexe_touch_ack", &ack_addr) != 0 ||
        elf_symbols_find(symbols, "flexe_touch_result", &result_addr) != 0) {
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
    esp32_periph_t *periph = flexe_session_periph(session);

    uint32_t stage = 0, last_stage = UINT32_MAX;
    bool budget_stop = true;
    while (cpu->cycle_count < MAX_CYCLES) {
        stage = mem_read32(mem, stage_addr);
        if (stage != last_stage) {
            fprintf(stderr, "[touch-fixture] stage=0x%08X cycles=%llu\n",
                    stage, (unsigned long long)cpu->cycle_count);
            last_stage = stage;
            /* Inject at the stage boundary rather than every batch: a pad that
             * is rewritten constantly would hide a model that only ever
             * reports its most recent write. */
            switch (stage) {
            case 1: case 3:
                periph_touch_set_value(periph, TOUCH_PAD, RELEASED_VALUE);
                break;
            case 2: case 4:
                periph_touch_set_value(periph, TOUCH_PAD, PRESSED_VALUE);
                break;
            default: break;
            }
            if (stage >= 1u && stage <= 4u)
                mem_write32(mem, ack_addr, stage);
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

    if (budget_stop)
        fprintf(stderr, "[touch-fixture] exited on cycle budget (%llu) at "
                        "pc=0x%08X\n", MAX_CYCLES, cpu->pc);

    int unhandled = periph_unhandled_count(flexe_session_periph(session));
    int unregistered = rom_stubs_unregistered_count(flexe_session_rom(session));

    printf("engine=%s stage=0x%08X released=%u pressed=%u isr_hits=%u "
           "final=%u unhandled=%d unregistered=%d\n",
           flexe_session_jit(session) ? "jit" : "interp", stage,
           r[0], r[1], r[2], r[3], unhandled, unregistered);

    /* Arduino 3 uses the IDF 5.5 software IIR filter, so a read after a host
     * transition is an intermediate filtered value rather than the raw count.
     * Require the released benchmark and the threshold crossing instead of
     * pretending both snapshots must equal the injected endpoints. */
    bool values_ok = r[0] == RELEASED_VALUE && r[1] < 500u && r[1] < r[0];
    bool isr_ok = r[2] >= 1u;

    int ok = stage == SUCCESS_MARKER && values_ok && isr_ok &&
             unhandled == 0 && unregistered == 0;
    if (!ok)
        fprintf(stderr, "[touch-fixture] stage_ok=%d values_ok=%d isr_ok=%d "
                        "(want released=%u pressed=%u)\n",
                stage == SUCCESS_MARKER, values_ok, isr_ok,
                RELEASED_VALUE, PRESSED_VALUE);
    flexe_session_destroy(session);
    elf_symbols_destroy(symbols);
    return ok ? 0 : 1;
}
