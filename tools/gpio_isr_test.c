/* Edge-triggered GPIO interrupts through the real ESP-IDF ISR service.
 *
 * attachInterrupt() installs a shared GPIO interrupt via
 * gpio_install_isr_service() and dispatches per-pin handlers from it. The
 * peripheral gates that touch GPIO drive it as a level input and poll, so
 * that dispatch path was never exercised -- and a CYD's touch controller
 * signals with PENIRQ, so it matters on the target hardware.
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

#define SUCCESS_MARKER 0x9109150Bu
#define MAX_CYCLES     400000000ull
#define RESULT_COUNT   8u
#define PIN_RISING     4
#define PIN_FALLING    16
#define PIN_CHANGE     17
#define EDGE_PAIRS     5

volatile int emu_app_running = 1;

static void run_batch(flexe_session_t *s, int cycles) {
    (void)flexe_session_run_core(s, 0, cycles);
    flexe_session_post_batch(s, cycles);
}

int main(int argc, char **argv) {
    int argi = 1, disable_jit = 0;
    if (argi < argc && strcmp(argv[argi], "--no-jit") == 0) { disable_jit = 1; argi++; }
    if (argc - argi != 2) {
        fprintf(stderr, "usage: %s [--no-jit] FIRMWARE.bin FIRMWARE.elf\n", argv[0]);
        return 2;
    }

    elf_symbols_t *symbols = elf_symbols_load(argv[argi + 1]);
    uint32_t stage_addr = 0, result_addr = 0;
    if (!symbols ||
        elf_symbols_find(symbols, "flexe_gpio_stage", &stage_addr) != 0 ||
        elf_symbols_find(symbols, "flexe_gpio_result", &result_addr) != 0) {
        fprintf(stderr, "error: fixture marker symbols are missing\n");
        elf_symbols_destroy(symbols);
        return 2;
    }

    flexe_session_config_t config = {
        .bin_path = argv[argi], .elf_path = argv[argi + 1],
        .disable_jit = disable_jit,
    };
    flexe_session_t *session = flexe_session_create(&config);
    if (!session) { elf_symbols_destroy(symbols); return 2; }

    xtensa_cpu_t *cpu = flexe_session_cpu(session, 0);
    xtensa_mem_t *mem = flexe_session_mem(session);
    esp32_periph_t *periph = flexe_session_periph(session);

    /* Idle low so the first transition is a genuine rising edge. */
    periph_gpio_set_input(periph, PIN_RISING, 0);
    periph_gpio_set_input(periph, PIN_FALLING, 0);
    periph_gpio_set_input(periph, PIN_CHANGE, 0);

    uint32_t stage = 0, last_stage = UINT32_MAX;
    bool driven = false, detached = false;
    while (cpu->cycle_count < MAX_CYCLES) {
        stage = mem_read32(mem, stage_addr);
        if (stage != last_stage) {
            fprintf(stderr, "[gpio-fixture] stage=0x%08X cycles=%llu\n",
                    stage, (unsigned long long)cpu->cycle_count);
            last_stage = stage;
        }
        if (stage == SUCCESS_MARKER || (stage & 0xFFF00000u) == 0xBAD00000u)
            break;

        if (stage == 1u && !driven) {
            driven = true;
            for (int i = 0; i < EDGE_PAIRS; i++) {
                periph_gpio_set_input(periph, PIN_RISING, 1);
                periph_gpio_set_input(periph, PIN_FALLING, 1);
                periph_gpio_set_input(periph, PIN_CHANGE, 1);
                run_batch(session, 20000);
                periph_gpio_set_input(periph, PIN_RISING, 0);
                periph_gpio_set_input(periph, PIN_FALLING, 0);
                periph_gpio_set_input(periph, PIN_CHANGE, 0);
                run_batch(session, 20000);
            }
            /* Leave PIN_CHANGE high so its last observed level is known. */
            periph_gpio_set_input(periph, PIN_CHANGE, 1);
            run_batch(session, 20000);
            mem_write32(mem, stage_addr, 2u);
        } else if (stage == 3u && !detached) {
            detached = true;
            /* More edges after detachInterrupt(): the count must not move. */
            for (int i = 0; i < EDGE_PAIRS; i++) {
                periph_gpio_set_input(periph, PIN_RISING, 1);
                run_batch(session, 20000);
                periph_gpio_set_input(periph, PIN_RISING, 0);
                run_batch(session, 20000);
            }
            mem_write32(mem, stage_addr, 4u);
        }
        run_batch(session, 10000);
    }

    stage = mem_read32(mem, stage_addr);
    uint32_t r[RESULT_COUNT];
    for (unsigned i = 0; i < RESULT_COUNT; i++)
        r[i] = mem_read32(mem, result_addr + i * 4u);
    int unhandled = periph_unhandled_count(periph);
    int unregistered = rom_stubs_unregistered_count(flexe_session_rom(session));

    printf("engine=%s stage=0x%08X rising=%u falling=%u change=%u level=%u "
           "at_detach=%u after_detach=%u unhandled=%d unregistered=%d\n",
           flexe_session_jit(session) ? "jit" : "interp", stage,
           r[0], r[1], r[2], r[3], r[4], r[5], unhandled, unregistered);

    /* CHANGE sees both edges of every pair plus the final rise. */
    int ok = stage == SUCCESS_MARKER &&
             r[0] == EDGE_PAIRS && r[1] == EDGE_PAIRS &&
             r[2] == EDGE_PAIRS * 2u + 1u && r[3] == 1u &&
             r[4] == EDGE_PAIRS && r[5] == r[4] &&
             unhandled == 0 && unregistered == 0;

    flexe_session_destroy(session);
    elf_symbols_destroy(symbols);
    return ok ? 0 : 1;
}
