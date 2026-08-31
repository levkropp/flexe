/* Run a compiled Arduino/ESP-IDF sigma-delta fixture through Flexe's public
 * session API and validate the external pulse-density boundary. */
#include "elf_symbols.h"
#include "flexe_session.h"
#include "memory.h"
#include "peripherals.h"
#include "rom_stubs.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define SUCCESS_MARKER 0x5349474Du
#define MAX_CYCLES     30000000ull

volatile int emu_app_running = 1;

typedef struct {
    unsigned count;
    bool channel0_ready;
    bool channel7_initial;
    bool channel7_final;
} sigmadelta_capture_t;

static void capture_sigmadelta(void *opaque, int channel, int gpio,
                               uint32_t frequency_hz, int8_t duty,
                               bool enabled, bool inverted) {
    sigmadelta_capture_t *capture = opaque;
    capture->count++;
    if (channel == 0 && gpio == 18 && frequency_hz == 312500u &&
        duty == 64 && enabled && !inverted) {
        capture->channel0_ready = true;
    }
    if (channel == 7 && gpio == 23 && frequency_hz == 78125u &&
        duty == -64 && enabled && !inverted) {
        capture->channel7_initial = true;
    }
    if (channel == 7 && gpio == 22 && frequency_hz == 39062u &&
        duty == -32 && enabled && !inverted) {
        capture->channel7_final = true;
    }
}

static int run_fixture(const char *bin_path, const char *elf_path,
                       bool disable_jit) {
    elf_symbols_t *symbols = elf_symbols_load(elf_path);
    uint32_t stage_addr = 0u;
    uint32_t result_addr = 0u;
    if (!symbols ||
        elf_symbols_find(symbols, "flexe_sigmadelta_stage",
                         &stage_addr) != 0 ||
        elf_symbols_find(symbols, "flexe_sigmadelta_result",
                         &result_addr) != 0) {
        fprintf(stderr, "error: fixture marker symbols are missing\n");
        elf_symbols_destroy(symbols);
        return 2;
    }

    flexe_session_config_t config = {
        .bin_path = bin_path,
        .elf_path = elf_path,
        .disable_jit = disable_jit,
    };
    flexe_session_t *session = flexe_session_create(&config);
    if (!session) {
        elf_symbols_destroy(symbols);
        return 2;
    }

    esp32_periph_t *periph = flexe_session_periph(session);
    sigmadelta_capture_t capture = {0};
    if (periph_set_sigmadelta_output_callback(
            periph, 0, capture_sigmadelta, &capture) != 0 ||
        periph_set_sigmadelta_output_callback(
            periph, 7, capture_sigmadelta, &capture) != 0) {
        flexe_session_destroy(session);
        elf_symbols_destroy(symbols);
        return 2;
    }

    xtensa_cpu_t *cpu0 = flexe_session_cpu(session, 0);
    xtensa_cpu_t *cpu1 = flexe_session_cpu(session, 1);
    xtensa_mem_t *mem = flexe_session_mem(session);
    uint32_t stage = 0u;
    while (cpu0->cycle_count < MAX_CYCLES) {
        stage = mem_read32(mem, stage_addr);
        if (stage == SUCCESS_MARKER ||
            (stage & 0xFFF00000u) == 0xBAD00000u)
            break;
        (void)flexe_session_run_core(session, 0, 10000);
        flexe_session_post_batch(session, 10000);
    }

    stage = mem_read32(mem, stage_addr);
    uint32_t results[16];
    for (unsigned index = 0; index < 16u; index++)
        results[index] = mem_read32(mem, result_addr + index * 4u);

    /* The channel-0 fixture leaves signed duty +64 and prescale zero routed
     * to GPIO18. Sample exactly one complete 256-tick PDM period. */
    uint32_t cpu_mhz = mem_read32(mem, 0x3FFE01E0u);
    uint32_t sample_cycles = cpu_mhz >= 80u ? cpu_mhz / 80u : 0u;
    uint32_t base_ccount = cpu0->ccount;
    if (cpu1 && cpu1->ccount > base_ccount) base_ccount = cpu1->ccount;
    unsigned high_samples = 0u;
    if (sample_cycles != 0u) {
        for (unsigned sample = 1u; sample <= 256u; sample++) {
            cpu0->ccount = base_ccount + sample * sample_cycles;
            high_samples +=
                (unsigned)periph_gpio_pin_level(periph, 18);
        }
    }

    int unhandled = periph_unhandled_count(periph);
    int unregistered = rom_stubs_unregistered_count(
        flexe_session_rom(session));
    const char *engine = disable_jit ? "interp" : "jit";
    printf("engine=%s stage=0x%08X api=%u/%u "
           "regs=%04X/%04X routes=%u/%u enable=%08X "
           "callbacks=%u/%d/%d/%d density=%u/256 "
           "unhandled=%d unregistered=%d cycles=%llu\n",
           engine, stage, results[0], results[1], results[2], results[9],
           results[3], results[10], results[11], capture.count,
           capture.channel0_ready, capture.channel7_initial,
           capture.channel7_final, high_samples, unhandled, unregistered,
           (unsigned long long)cpu0->cycle_count);

    int ok = stage == SUCCESS_MARKER && results[0] == 312500u &&
             results[1] == 192u && results[2] == 0x00000040u &&
             results[3] == 100u && results[4] == 0u &&
             results[5] == 0x000003C0u && results[6] == 0u &&
             results[7] == 0u && results[8] == 0u &&
             results[9] == 0x000007E0u && results[10] == 107u &&
             (results[11] & ((1u << 18u) | (1u << 22u))) ==
                 ((1u << 18u) | (1u << 22u)) &&
             results[12] == 0x01506190u && results[13] == 1u &&
             capture.channel0_ready && capture.channel7_initial &&
             capture.channel7_final && high_samples == 192u &&
             unhandled == 0 && unregistered == 0;

    flexe_session_destroy(session);
    elf_symbols_destroy(symbols);
    return ok ? 0 : 1;
}

int main(int argc, char **argv) {
    bool disable_jit = false;
    int arg = 1;
    if (argc > 1 && strcmp(argv[1], "--no-jit") == 0) {
        disable_jit = true;
        arg++;
    }
    if (argc - arg != 2) {
        fprintf(stderr, "usage: %s [--no-jit] FIRMWARE.bin FIRMWARE.elf\n",
                argv[0]);
        return 2;
    }
    return run_fixture(argv[arg], argv[arg + 1], disable_jit);
}
