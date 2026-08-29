/* Run the public ESP-IDF MCPWM legacy driver through Flexe. The host observes
 * both motor-control units and drives genuine GPIO-matrix sync, fault, and
 * capture inputs, including the driver's real capture ISR callback. */
#include "elf_symbols.h"
#include "flexe_session.h"
#include "memory.h"
#include "peripherals.h"
#include "rom_stubs.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define SUCCESS_MARKER 0x4D435057u
#define MAX_CYCLES     250000000ull
#define RESULT_COUNT   32u

volatile int emu_app_running = 1;

typedef struct {
    int expected_unit;
    int expected_operator;
    int expected_generator;
    int expected_gpio;
    unsigned events;
    bool valid;
    bool active;
    bool stopped;
    bool saw_initial;
    bool saw_updated;
    bool saw_fault;
    bool saw_fault_release;
    periph_mcpwm_output_info_t info;
} mcpwm_host_capture_t;

static void capture_mcpwm(void *opaque, int unit, int operator_index,
                          int generator,
                          const periph_mcpwm_output_info_t *info) {
    mcpwm_host_capture_t *capture = opaque;
    capture->events++;
    capture->info = *info;
    if (!info->enabled && !capture->active) return;
    if (unit != capture->expected_unit ||
        operator_index != capture->expected_operator ||
        generator != capture->expected_generator ||
        info->gpio != capture->expected_gpio || info->inverted)
        capture->valid = false;
    if (info->enabled) {
        capture->active = true;
        if (unit == 0 && operator_index == 0 && generator == 0 &&
            info->frequency_hz == 20000u && info->period_ticks == 50u &&
            info->compare_ticks == 12u &&
            info->rising_delay_ticks == 9u &&
            info->falling_delay_ticks == 13u &&
            info->deadtime_clock_hz == 10000000u &&
            info->carrier_hz == 312500u &&
            info->carrier_duty_eighths == 5u)
            capture->saw_initial = true;
        if (unit == 0 && operator_index == 0 && generator == 0 &&
            info->frequency_hz == 10000u && info->period_ticks == 100u &&
            info->compare_ticks == 75u)
            capture->saw_updated = true;
        if (info->fault_active && info->forced_level == 0)
            capture->saw_fault = true;
        if (capture->saw_fault && !info->fault_active)
            capture->saw_fault_release = true;
    } else if (capture->active) {
        capture->stopped = true;
    }
}

static void run_batch(flexe_session_t *session, int cycles) {
    (void)flexe_session_run_core(session, 0, cycles);
    flexe_session_post_batch(session, cycles);
}

static bool results_are_valid(const uint32_t *result) {
    for (unsigned i = 0; i <= 7u; i++) {
        if (result[i] != 0u) return false;
    }
    if (result[8] != 20000u || result[9] != 2400u ||
        result[10] != 1000u || result[11] != 6000u)
        return false;
    for (unsigned i = 12; i <= 17u; i++) {
        if (result[i] != 0u) return false;
    }
    if (result[18] != 2u || result[19] != 1u ||
        result[21] <= result[20])
        return false;
    for (unsigned i = 22; i <= 26u; i++) {
        if (result[i] != 0u) return false;
    }
    return result[27] == 10000u && result[28] == 7500u &&
           result[29] == 0u && result[30] == 0u;
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
        elf_symbols_find(symbols, "flexe_mcpwm_stage", &stage_addr) != 0 ||
        elf_symbols_find(symbols, "flexe_mcpwm_command", &command_addr) != 0 ||
        elf_symbols_find(symbols, "flexe_mcpwm_result", &result_addr) != 0) {
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
    mcpwm_host_capture_t primary_a = {
        .expected_unit = 0, .expected_operator = 0,
        .expected_generator = 0, .expected_gpio = 18, .valid = true,
    };
    mcpwm_host_capture_t primary_b = {
        .expected_unit = 0, .expected_operator = 0,
        .expected_generator = 1, .expected_gpio = 19, .valid = true,
    };
    mcpwm_host_capture_t upper_b = {
        .expected_unit = 1, .expected_operator = 2,
        .expected_generator = 1, .expected_gpio = 23, .valid = true,
    };
    esp32_periph_t *periph = flexe_session_periph(session);
    if (periph_set_mcpwm_output_callback(
            periph, 0, 0, 0, capture_mcpwm, &primary_a) != 0 ||
        periph_set_mcpwm_output_callback(
            periph, 0, 0, 1, capture_mcpwm, &primary_b) != 0 ||
        periph_set_mcpwm_output_callback(
            periph, 1, 2, 1, capture_mcpwm, &upper_b) != 0) {
        fprintf(stderr, "error: could not attach virtual MCPWM endpoints\n");
        flexe_session_destroy(session);
        elf_symbols_destroy(symbols);
        return 2;
    }

    xtensa_cpu_t *cpu = flexe_session_cpu(session, 0);
    xtensa_mem_t *mem = flexe_session_mem(session);
    uint32_t stage = 0;
    uint32_t last_stage = UINT32_MAX;
    bool handled[6] = {false};
    bool sync_ok = false;
    bool force_high_ok = false;
    bool force_low_ok = false;
    bool fault_level_ok = false;
    bool fault_release_ok = false;

    while (cpu->cycle_count < MAX_CYCLES) {
        stage = mem_read32(mem, stage_addr);
        if (stage != last_stage) {
            fprintf(stderr,
                    "[mcpwm-fixture] stage=0x%08X cycles=%llu pc=0x%08X\n",
                    stage, (unsigned long long)cpu->cycle_count, cpu->pc);
            last_stage = stage;
        }
        if (stage == SUCCESS_MARKER ||
            (stage & 0xFFF00000u) == 0xBAD00000u)
            break;

        if (stage == 2u && !handled[2]) {
            handled[2] = true;

            /* GPIO SYNC0 reloads timer0 to 500/1000 of its 50-tick peak. */
            periph_gpio_set_input(periph, 32, 1);
            sync_ok = (mem_read32(mem, 0x3FF5E010u) & 0xFFFFu) == 25u;
            periph_gpio_set_input(periph, 32, 0);

            /* Both edges pass through the genuine driver ISR and callback. */
            run_batch(session, 2000);
            periph_gpio_set_input(periph, 34, 1);
            run_batch(session, 2000);
            periph_gpio_set_input(periph, 34, 0);
            run_batch(session, 2000);
            mem_write32(mem, command_addr, 1u);
        } else if (stage == 3u && !handled[3]) {
            handled[3] = true;
            (void)mem_read32(mem, 0x3FF5E010u);
            force_high_ok = periph_gpio_pin_level(periph, 18) == 1;
            mem_write32(mem, command_addr, 2u);
        } else if (stage == 4u && !handled[4]) {
            handled[4] = true;
            (void)mem_read32(mem, 0x3FF5E010u);
            force_low_ok = periph_gpio_pin_level(periph, 18) == 0;
            mem_write32(mem, command_addr, 3u);
        } else if (stage == 5u && !handled[5]) {
            handled[5] = true;
            /* Catch up the shadow compare update before checking the endpoint. */
            (void)mem_read32(mem, 0x3FF5E010u);

            periph_gpio_set_input(periph, 35, 1);
            fault_level_ok = periph_gpio_pin_level(periph, 18) == 0 &&
                             primary_a.info.fault_active;
            periph_gpio_set_input(periph, 35, 0);
            fault_release_ok = !primary_a.info.fault_active;
            mem_write32(mem, command_addr, 4u);
        }

        run_batch(session, 10000);
    }

    stage = mem_read32(mem, stage_addr);
    /* Catch up stop-at-TEZ commands issued immediately before success. */
    (void)mem_read32(mem, 0x3FF5E010u);
    (void)mem_read32(mem, 0x3FF6C030u);

    uint32_t result[RESULT_COUNT];
    for (unsigned i = 0; i < RESULT_COUNT; i++)
        result[i] = mem_read32(mem, result_addr + i * 4u);
    int unhandled = periph_unhandled_count(periph);
    int unregistered = rom_stubs_unregistered_count(
        flexe_session_rom(session));

    if (stage != SUCCESS_MARKER) {
        xtensa_cpu_t *cpu1 = flexe_session_cpu(session, 1);
        fprintf(stderr,
                "[mcpwm-fixture] cpu0 pc=0x%08X run=%d halt=%d exc=%d "
                "cause=%u vaddr=0x%08X epc1=0x%08X depc=0x%08X\n",
                cpu->pc, cpu->running, cpu->halted, cpu->exception,
                cpu->exccause, cpu->excvaddr, cpu->epc[0], cpu->depc);
        fprintf(stderr,
                "[mcpwm-fixture] cpu1 pc=0x%08X run=%d halt=%d exc=%d "
                "cause=%u vaddr=0x%08X epc1=0x%08X depc=0x%08X\n",
                cpu1->pc, cpu1->running, cpu1->halted, cpu1->exception,
                cpu1->exccause, cpu1->excvaddr, cpu1->epc[0], cpu1->depc);
        fprintf(stderr,
                "[mcpwm-fixture] unit0 timer=0x%08X/%u op=0x%08X/"
                "0x%08X/0x%08X fault=0x%08X cap=0x%08X/%u/0x%08X "
                "irq=0x%08X/0x%08X/0x%08X\n",
                mem_read32(mem, 0x3FF5E008u),
                mem_read32(mem, 0x3FF5E010u),
                mem_read32(mem, 0x3FF5E040u),
                mem_read32(mem, 0x3FF5E050u),
                mem_read32(mem, 0x3FF5E064u),
                mem_read32(mem, 0x3FF5E0E4u),
                mem_read32(mem, 0x3FF5E0F0u),
                mem_read32(mem, 0x3FF5E0FCu),
                mem_read32(mem, 0x3FF5E108u),
                mem_read32(mem, 0x3FF5E114u),
                mem_read32(mem, 0x3FF5E118u),
                mem_read32(mem, 0x3FF5E110u));
        char task_dump[4096];
        if (freertos_stubs_dump_tasks(flexe_session_frt(session), task_dump,
                                      sizeof(task_dump)) > 0)
            fprintf(stderr, "[mcpwm-fixture] tasks:\n%s", task_dump);
    }

    printf("engine=%s stage=0x%08X result=",
           flexe_session_jit(session) ? "jit" : "interp", stage);
    for (unsigned i = 0; i < RESULT_COUNT; i++)
        printf("%s%u", i ? "/" : "", result[i]);
    printf(" events=%u/%u/%u active=%d/%d/%d stopped=%d/%d/%d "
           "initial=%d updated=%d fault=%d/%d levels=%d/%d/%d/%d "
           "unhandled=%d unregistered=%d cycles=%llu\n",
           primary_a.events, primary_b.events, upper_b.events,
           primary_a.active, primary_b.active, upper_b.active,
           primary_a.stopped, primary_b.stopped, upper_b.stopped,
           primary_a.saw_initial, primary_a.saw_updated,
           primary_a.saw_fault, primary_a.saw_fault_release,
           sync_ok, force_high_ok, force_low_ok,
           fault_level_ok && fault_release_ok,
           unhandled, unregistered,
           (unsigned long long)cpu->cycle_count);

    bool endpoints_ok = primary_a.valid && primary_b.valid && upper_b.valid &&
                        primary_a.active && primary_b.active && upper_b.active &&
                        primary_a.saw_initial && primary_a.saw_updated &&
                        primary_a.saw_fault && primary_a.saw_fault_release;
    int ok = stage == SUCCESS_MARKER && results_are_valid(result) &&
             endpoints_ok && handled[2] && handled[3] && handled[4] &&
             handled[5] && sync_ok && force_high_ok && force_low_ok &&
             fault_level_ok && fault_release_ok && unhandled == 0 &&
             unregistered == 0;

    flexe_session_destroy(session);
    elf_symbols_destroy(symbols);
    return ok ? 0 : 1;
}
