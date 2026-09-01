/* Run the compiled Arduino analog/DAC fixture through the public Flexe
 * session API. Arduino CLI builds the firmware separately so the emulator's
 * normal build stays independent of a particular ESP32 core release. */
#include "elf_symbols.h"
#include "flexe_session.h"
#include "memory.h"
#include "peripherals.h"
#include "rom_stubs.h"
#include "sandbox_events.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define SUCCESS_MARKER 0xADC0DAC0u
#define MAX_CYCLES     30000000ull

volatile int emu_app_running = 1;

typedef struct {
    unsigned count;
    bool dac1_enabled_value;
    bool dac1_disabled_value;
    bool dac2_enabled_value;
} dac_capture_t;

static void capture_dac(const sbx_event_t *event, void *opaque) {
    if (event->kind != SBX_EV_DAC_OUT)
        return;
    dac_capture_t *capture = opaque;
    capture->count++;
    if (event->dac_out.channel == 0 && event->dac_out.enabled &&
        event->dac_out.value == 0x35)
        capture->dac1_enabled_value = true;
    if (event->dac_out.channel == 0 && !event->dac_out.enabled &&
        event->dac_out.value == 0x35)
        capture->dac1_disabled_value = true;
    if (event->dac_out.channel == 1 && event->dac_out.enabled &&
        event->dac_out.value == 0xCA)
        capture->dac2_enabled_value = true;
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

    elf_symbols_t *symbols = elf_symbols_load(argv[arg + 1]);
    uint32_t stage_addr = 0;
    uint32_t result_addr = 0;
    if (!symbols ||
        elf_symbols_find(symbols, "flexe_analog_stage", &stage_addr) != 0 ||
        elf_symbols_find(symbols, "flexe_analog_result", &result_addr) != 0) {
        fprintf(stderr, "error: fixture marker symbols are missing\n");
        elf_symbols_destroy(symbols);
        return 2;
    }

    flexe_session_config_t config = {
        .bin_path = argv[arg],
        .elf_path = argv[arg + 1],
        .disable_jit = disable_jit,
    };
    flexe_session_t *session = flexe_session_create(&config);
    if (!session) {
        elf_symbols_destroy(symbols);
        return 2;
    }

    esp32_periph_t *periph = flexe_session_periph(session);
    periph_set_adc_value(periph, 6, 0x0A55u);
    periph_set_adc_value(periph, 10, 0x05AAu);
    dac_capture_t capture = {0};
    sbx_events_set_sink(capture_dac, &capture);

    xtensa_cpu_t *cpu = flexe_session_cpu(session, 0);
    xtensa_mem_t *mem = flexe_session_mem(session);
    uint32_t stage = 0;
    while (cpu->cycle_count < MAX_CYCLES) {
        stage = mem_read32(mem, stage_addr);
        if (stage == SUCCESS_MARKER ||
            (stage & 0xFFF00000u) == 0xBAD00000u)
            break;
        (void)flexe_session_run_core(session, 0, 10000);
        flexe_session_post_batch(session, 10000);
    }

    stage = mem_read32(mem, stage_addr);
    uint32_t results[4];
    for (unsigned i = 0; i < 4; ++i)
        results[i] = mem_read32(mem, result_addr + i * 4u);
    int dac1_enabled = periph_dac_enabled(periph, 0);
    int dac2_enabled = periph_dac_enabled(periph, 1);
    uint8_t dac1_value = periph_dac_value(periph, 0);
    uint8_t dac2_value = periph_dac_value(periph, 1);
    int unhandled = periph_unhandled_count(periph);
    int unregistered = rom_stubs_unregistered_count(
        flexe_session_rom(session));

    printf("engine=%s stage=0x%08X adc=%u/%u/%u/%u "
           "dac=%d:0x%02X/%d:0x%02X events=%u/%d/%d/%d "
           "unhandled=%d unregistered=%d cycles=%llu\n",
           disable_jit ? "interp" : "jit", stage,
           results[0], results[1], results[2], results[3],
           dac1_enabled, dac1_value, dac2_enabled, dac2_value,
           capture.count, capture.dac1_enabled_value,
           capture.dac1_disabled_value, capture.dac2_enabled_value,
           unhandled, unregistered, (unsigned long long)cpu->cycle_count);

    int ok = stage == SUCCESS_MARKER &&
             results[0] == 0x0A55u && results[1] == 0x05AAu &&
             results[2] == 0x0055u && results[3] == 0x01AAu &&
             dac1_enabled == 0 && dac1_value == 0x35u &&
             dac2_enabled == 1 && dac2_value == 0xCAu &&
             capture.dac1_enabled_value && capture.dac1_disabled_value &&
             capture.dac2_enabled_value && unhandled == 0 &&
             unregistered == 0;

    sbx_events_set_sink(NULL, NULL);
    flexe_session_destroy(session);
    elf_symbols_destroy(symbols);
    return ok ? 0 : 1;
}
