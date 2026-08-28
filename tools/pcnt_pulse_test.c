/* Run the public ESP-IDF PCNT driver and its real shared ISR service through
 * Flexe while host GPIO transitions feed the hardware pulse inputs. */
#include "elf_symbols.h"
#include "flexe_session.h"
#include "memory.h"
#include "peripherals.h"
#include "rom_stubs.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define SUCCESS_MARKER 0xC01A7E00u
#define MAX_CYCLES     200000000ull
#define RESULT_COUNT   20u

volatile int emu_app_running = 1;

static void run_batch(flexe_session_t *session, int cycles) {
    (void)flexe_session_run_core(session, 0, cycles);
    flexe_session_post_batch(session, cycles);
}

static void qualify_level(flexe_session_t *session, esp32_periph_t *periph,
                          int gpio, int level) {
    periph_gpio_set_input(periph, gpio, level);
    run_batch(session, 1000);
}

static void qualified_pulse(flexe_session_t *session,
                            esp32_periph_t *periph, int gpio) {
    qualify_level(session, periph, gpio, 1);
    qualify_level(session, periph, gpio, 0);
}

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
    uint32_t command_addr = 0;
    uint32_t result_addr = 0;
    if (!symbols ||
        elf_symbols_find(symbols, "flexe_pcnt_stage", &stage_addr) != 0 ||
        elf_symbols_find(symbols, "flexe_pcnt_command", &command_addr) != 0 ||
        elf_symbols_find(symbols, "flexe_pcnt_result", &result_addr) != 0) {
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
    uint32_t stage = 0;
    uint32_t last_stage = UINT32_MAX;
    bool handled[6] = {false};
    while (cpu->cycle_count < MAX_CYCLES) {
        stage = mem_read32(mem, stage_addr);
        if (stage != last_stage) {
            fprintf(stderr,
                    "[pcnt-fixture] stage=0x%08X cycles=%llu pc=0x%08X\n",
                    stage, (unsigned long long)cpu->cycle_count, cpu->pc);
            last_stage = stage;
        }
        if (stage == SUCCESS_MARKER ||
            (stage & 0xFFF00000u) == 0xBAD00000u)
            break;

        if (stage == 2u && !handled[2]) {
            handled[2] = true;
            /* A zero-time high/low glitch must be rejected by the driver's
             * ten-APB-cycle filter. */
            periph_gpio_set_input(periph, 18, 1);
            periph_gpio_set_input(periph, 18, 0);
            run_batch(session, 1000);
            qualified_pulse(session, periph, 22); /* unit7/channel1 */
            for (int i = 0; i < 3; i++)
                qualified_pulse(session, periph, 18);
            mem_write32(mem, command_addr, 1u);
        } else if (stage == 3u && !handled[3]) {
            handled[3] = true;
            qualify_level(session, periph, 19, 1); /* reverse direction */
            for (int i = 0; i < 2; i++)
                qualified_pulse(session, periph, 18);
            mem_write32(mem, command_addr, 2u);
        } else if (stage == 4u && !handled[4]) {
            handled[4] = true;
            qualified_pulse(session, periph, 18); /* paused: no count */
            mem_write32(mem, command_addr, 3u);
        } else if (stage == 5u && !handled[5]) {
            handled[5] = true;
            qualify_level(session, periph, 19, 0);
            for (int i = 0; i < 5; i++)
                qualified_pulse(session, periph, 18);
            mem_write32(mem, command_addr, 4u);
        }

        run_batch(session, 10000);
    }

    stage = mem_read32(mem, stage_addr);
    uint32_t results[RESULT_COUNT];
    for (unsigned i = 0; i < RESULT_COUNT; i++)
        results[i] = mem_read32(mem, result_addr + i * 4u);
    int unhandled = periph_unhandled_count(periph);
    int unregistered = rom_stubs_unregistered_count(
        flexe_session_rom(session));

    if (stage != SUCCESS_MARKER) {
        fprintf(stderr,
                "[pcnt-fixture] conf0=0x%08X conf1=0x%08X conf2=0x%08X "
                "count=%u raw=0x%08X st=0x%08X ena=0x%08X "
                "status=0x%08X ctrl=0x%08X\n",
                mem_read32(mem, 0x3FF57000u),
                mem_read32(mem, 0x3FF57004u),
                mem_read32(mem, 0x3FF57008u),
                mem_read32(mem, 0x3FF57060u),
                mem_read32(mem, 0x3FF57080u),
                mem_read32(mem, 0x3FF57084u),
                mem_read32(mem, 0x3FF57088u),
                mem_read32(mem, 0x3FF57090u),
                mem_read32(mem, 0x3FF570B0u));
        char task_dump[4096];
        if (freertos_stubs_dump_tasks(flexe_session_frt(session), task_dump,
                                      sizeof(task_dump)) > 0)
            fprintf(stderr, "[pcnt-fixture] tasks:\n%s", task_dump);
    }

    printf("engine=%s stage=0x%08X result=",
           flexe_session_jit(session) ? "jit" : "interp", stage);
    for (unsigned i = 0; i < RESULT_COUNT; i++)
        printf("%s%u", i ? "/" : "", results[i]);
    printf(" handled=%d/%d/%d/%d unhandled=%d unregistered=%d "
           "cycles=%llu\n",
           handled[2], handled[3], handled[4], handled[5],
           unhandled, unregistered,
           (unsigned long long)cpu->cycle_count);

    bool setup_ok = true;
    for (unsigned i = 0; i <= 10u; i++) {
        uint32_t expected = i == 5u ? 10u : 0u;
        if (results[i] != expected) setup_ok = false;
    }
    bool behavior_ok = results[11] == 3u && results[12] == 1u &&
                       (results[13] & (1u << 3)) != 0 &&
                       results[14] == 1u && results[15] == 1u &&
                       results[16] == 0u && results[17] == 3u &&
                       results[18] == 1u && results[19] == 0u;
    int ok = stage == SUCCESS_MARKER && setup_ok && behavior_ok &&
             handled[2] && handled[3] && handled[4] && handled[5] &&
             unhandled == 0 && unregistered == 0;

    flexe_session_destroy(session);
    elf_symbols_destroy(symbols);
    return ok ? 0 : 1;
}
