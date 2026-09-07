/* Check RTC GPIO against the pad the rest of the machine sees.
 *
 * The guest cross-checks each direction through the other API; this side owns
 * the pad itself. Both halves are needed: the guest cannot tell a correct model
 * from one that keeps its own private copy of RTC_GPIO_OUT and answers
 * digitalRead() from the same place, and the host cannot tell whether the guest
 * driver got as far as issuing the write.
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

#define SUCCESS_MARKER 0x27C6D10Au
#define MAX_CYCLES     4000000000ull
#define RESULT_COUNT   8u

#define OUT_PIN 25
#define IN_PIN  26

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
        elf_symbols_find(symbols, "flexe_rtcio_stage", &stage_addr) != 0 ||
        elf_symbols_find(symbols, "flexe_rtcio_result", &result_addr) != 0) {
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

    /* What the pad read while the guest said it was driving it. */
    int pad_high = -1, pad_low = -1;
    uint32_t stage = 0, last_stage = UINT32_MAX;
    bool budget_stop = true;

    while (cpu->cycle_count < MAX_CYCLES) {
        stage = mem_read32(mem, stage_addr);
        if (stage != last_stage) {
            fprintf(stderr, "[rtcio-fixture] stage=0x%08X cycles=%llu\n",
                    stage, (unsigned long long)cpu->cycle_count);
            last_stage = stage;
            switch (stage) {
            case 1: pad_high = periph_gpio_pin_level(periph, OUT_PIN); break;
            case 2: pad_low  = periph_gpio_pin_level(periph, OUT_PIN);
                break;
            case 3: periph_gpio_set_input(periph, IN_PIN, 1); break;
            case 4: periph_gpio_set_input(periph, IN_PIN, 0); break;
            default: break;
            }
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
        fprintf(stderr, "[rtcio-fixture] exited on cycle budget (%llu) at "
                        "pc=0x%08X\n", MAX_CYCLES, cpu->pc);

    int unhandled = periph_unhandled_count(flexe_session_periph(session));
    int unregistered = rom_stubs_unregistered_count(flexe_session_rom(session));

    printf("engine=%s stage=0x%08X pad_high=%d pad_low=%d "
           "rtc_in_high=%u rtc_in_low=%u "
           "unhandled=%d unregistered=%d\n",
           flexe_session_jit(session) ? "jit" : "interp", stage,
           pad_high, pad_low, r[2], r[3],
           unhandled, unregistered);

    /* Driving from the RTC domain has to move the real pad. The host is the
     * only side that can check this: rtc_gpio_init() switches the pad to the
     * RTC mux and disconnects the digital input path, so GPIO_IN is not a
     * defined view of it and the guest has nothing to compare against. */
    bool out_ok = pad_high == 1 && pad_low == 0;
    /* And a level the host puts on the pad has to be visible from the RTC
     * domain. */
    bool in_ok = r[2] == 1u && r[3] == 0u;

    int ok = stage == SUCCESS_MARKER && out_ok && in_ok &&
             unhandled == 0 && unregistered == 0;
    if (!ok)
        fprintf(stderr, "[rtcio-fixture] stage_ok=%d out_ok=%d in_ok=%d\n",
                stage == SUCCESS_MARKER, out_ok, in_ok);

    flexe_session_destroy(session);
    elf_symbols_destroy(symbols);
    return ok ? 0 : 1;
}
