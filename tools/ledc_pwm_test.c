/* Run an unmodified ESP-IDF LEDC driver configuration and blocking fade
 * through Flexe, including the production driver's fade ISR. */
#include "elf_symbols.h"
#include "flexe_session.h"
#include "memory.h"
#include "peripherals.h"
#include "rom_stubs.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define SUCCESS_MARKER 0x1EDCC0DEu
#define MAX_CYCLES     100000000ull
#define RESULT_COUNT   13u

volatile int emu_app_running = 1;

typedef struct {
    unsigned events;
    bool active;
    bool stopped;
    bool saw_64;
    bool saw_192;
    bool saw_96;
    bool valid;
    uint32_t frequency_hz;
    uint32_t duty;
    uint32_t duty_max;
} ledc_capture_t;

static void capture_ledc(void *opaque, int speed_mode, int channel, int gpio,
                         uint32_t frequency_hz, uint32_t duty,
                         uint32_t duty_max, bool enabled, bool inverted) {
    ledc_capture_t *capture = opaque;
    capture->events++;
    if (!enabled && !capture->active) return;
    if (speed_mode != 0 || channel != 0 || gpio != 21 || inverted)
        capture->valid = false;
    capture->frequency_hz = frequency_hz;
    capture->duty = duty;
    capture->duty_max = duty_max;
    if (enabled) {
        capture->active = true;
        capture->valid = capture->valid && frequency_hz == 5000u &&
                         duty_max == 255u;
        if (duty == 64u) capture->saw_64 = true;
        if (duty == 192u) capture->saw_192 = true;
        if (duty == 96u) capture->saw_96 = true;
    } else if (capture->active) {
        capture->stopped = true;
    }
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
    uint32_t result_addr = 0;
    if (!symbols ||
        elf_symbols_find(symbols, "flexe_ledc_stage", &stage_addr) != 0 ||
        elf_symbols_find(symbols, "flexe_ledc_result", &result_addr) != 0) {
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

    ledc_capture_t capture = {.valid = true};
    esp32_periph_t *periph = flexe_session_periph(session);
    if (periph_set_ledc_output_callback(periph, 0, 0, capture_ledc,
                                        &capture) != 0) {
        fprintf(stderr, "error: could not attach virtual LEDC endpoint\n");
        flexe_session_destroy(session);
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
                    "[ledc-fixture] stage=0x%08X cycles=%llu pc=0x%08X\n",
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
    uint32_t results[RESULT_COUNT];
    for (unsigned i = 0; i < RESULT_COUNT; i++)
        results[i] = mem_read32(mem, result_addr + i * 4u);
    int unhandled = periph_unhandled_count(periph);
    int unregistered = rom_stubs_unregistered_count(
        flexe_session_rom(session));

    if (stage != SUCCESS_MARKER) {
        fprintf(stderr,
                "[ledc-fixture] conf0=0x%08X hpoint=0x%08X "
                "duty=0x%08X conf1=0x%08X duty_r=0x%08X "
                "timer=0x%08X count=%u raw=0x%08X st=0x%08X "
                "ena=0x%08X\n",
                mem_read32(mem, 0x3FF59000u),
                mem_read32(mem, 0x3FF59004u),
                mem_read32(mem, 0x3FF59008u),
                mem_read32(mem, 0x3FF5900Cu),
                mem_read32(mem, 0x3FF59010u),
                mem_read32(mem, 0x3FF59140u),
                mem_read32(mem, 0x3FF59144u),
                mem_read32(mem, 0x3FF59180u),
                mem_read32(mem, 0x3FF59184u),
                mem_read32(mem, 0x3FF59188u));
        fprintf(stderr,
                "[ledc-fixture] cpu0 pc=0x%08X run=%d halt=%d int=0x%08X "
                "ena=0x%08X ccount=%u next=%u\n",
                cpu->pc, cpu->running, cpu->halted, cpu->interrupt,
                cpu->intenable, cpu->ccount, cpu->next_timer_event);
        for (int i = 0;; i++) {
            const char *name = NULL;
            uint32_t addr = 0;
            uint32_t calls = 0;
            if (rom_stubs_get_stats(flexe_session_rom(session), i, &name,
                                    &addr, &calls) != 0)
                break;
            if (name && (strstr(name, "Queue") || strstr(name, "Semaphore")))
                fprintf(stderr, "[ledc-fixture] %s=0x%08X calls=%u\n",
                        name, addr, calls);
        }
        char task_dump[4096];
        if (freertos_stubs_dump_tasks(flexe_session_frt(session), task_dump,
                                      sizeof(task_dump)) > 0)
            fprintf(stderr, "[ledc-fixture] tasks:\n%s", task_dump);
    }

    printf("engine=%s stage=0x%08X result=",
           flexe_session_jit(session) ? "jit" : "interp", stage);
    for (unsigned i = 0; i < RESULT_COUNT; i++)
        printf("%s%u", i ? "/" : "", results[i]);
    printf(" events=%u active=%d stopped=%d duties=%d/%d/%d "
           "output=%u/%u/%u valid=%d unhandled=%d unregistered=%d "
           "cycles=%llu\n",
           capture.events, capture.active, capture.stopped,
           capture.saw_64, capture.saw_192, capture.saw_96,
           capture.frequency_hz, capture.duty, capture.duty_max,
           capture.valid, unhandled, unregistered,
           (unsigned long long)cpu->cycle_count);

    bool results_ok = results[0] == 0 && results[1] == 0 &&
                      results[2] == 5000 && results[3] == 64 &&
                      results[4] == 0 && results[5] == 0 &&
                      results[6] == 192 && results[7] == 0 &&
                      results[8] == 0 && results[9] == 0 &&
                      results[10] == 96 && results[11] == 0 &&
                      results[12] == 0;
    int ok = stage == SUCCESS_MARKER && results_ok && capture.valid &&
             capture.active && capture.stopped && capture.saw_64 &&
             capture.saw_192 && capture.saw_96 && unhandled == 0 &&
             unregistered == 0;

    flexe_session_destroy(session);
    elf_symbols_destroy(symbols);
    return ok ? 0 : 1;
}
