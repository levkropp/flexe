/* Drive FreeRTOS event groups from guest code and check both what each wait
 * returned and how long it blocked.
 *
 * The bits themselves are the easy half: a broken implementation can set and
 * read them correctly while every blocking wait is wrong. So each wait is
 * bounded here in time as well -- a wait that should return immediately must
 * not consume its timeout, and a wait that should time out must not return
 * early.
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

#define SUCCESS_MARKER 0xE7600C0Bu
#define MAX_CYCLES     6000000000ull
#define RESULT_COUNT   12u

#define BIT_A 0x1u
#define BIT_B 0x2u
#define BIT_C 0x4u

#define TIMEOUT_US  120000u
#define SETTER_US   80000u
/* A wait that should not block at all still costs a scheduler round trip. */
#define IMMEDIATE_US 5000u

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
        elf_symbols_find(symbols, "flexe_eg_stage", &stage_addr) != 0 ||
        elf_symbols_find(symbols, "flexe_eg_result", &result_addr) != 0) {
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
            fprintf(stderr, "[event-group] stage=0x%08X cycles=%llu\n",
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
            fprintf(stderr, "[event-group] tasks:\n%s", dump);
        fprintf(stderr, "[event-group] pc=0x%08X cycles=%llu\n", cpu->pc,
                (unsigned long long)cpu->cycle_count);
    }
    int unhandled = periph_unhandled_count(flexe_session_periph(session));
    int unregistered = rom_stubs_unregistered_count(flexe_session_rom(session));

    printf("engine=%s stage=0x%08X bits=%02X/%02X preset=%02X@%uus "
           "timeout=%02X@%uus woken=%02X@%uus partial=%02X@%uus "
           "all=%02X@%uus unhandled=%d unregistered=%d\n",
           flexe_session_jit(session) ? "jit" : "interp", stage,
           r[0], r[1], r[2], r[3], r[4], r[5], r[6], r[7], r[8], r[9],
           r[10], r[11], unhandled, unregistered);

    /* An already-set bit returns without blocking. */
    bool preset_ok = (r[2] & BIT_A) && r[3] < IMMEDIATE_US;
    /* Nothing set: block for the whole timeout, return without the bit. */
    bool timeout_ok = !(r[4] & BIT_C) && r[5] >= TIMEOUT_US &&
                      r[5] <= TIMEOUT_US * 2u;
    /* Woken by another task partway through a much longer timeout. Both
     * bounds matter: returning at once would mean the wait never blocked,
     * and running to the timeout would mean the set never woke it. */
    bool woken_ok = (r[6] & BIT_B) && r[7] >= SETTER_US / 2u &&
                    r[7] <= SETTER_US * 2u;
    /* Wait-for-all with only one bit set must time out, not match early. */
    bool partial_ok = (r[8] & (BIT_A | BIT_C)) != (BIT_A | BIT_C) &&
                      r[9] >= TIMEOUT_US && r[9] <= TIMEOUT_US * 2u;
    /* Both bits set: same wait returns at once. */
    bool all_ok = (r[10] & (BIT_A | BIT_C)) == (BIT_A | BIT_C) &&
                  r[11] < IMMEDIATE_US;

    int ok = stage == SUCCESS_MARKER && r[0] == (BIT_A | BIT_C) &&
             r[1] == BIT_A && preset_ok && timeout_ok && woken_ok &&
             partial_ok && all_ok && unhandled == 0 && unregistered == 0;
    if (!ok)
        fprintf(stderr, "[event-group] preset=%d timeout=%d woken=%d "
                        "partial=%d all=%d\n",
                preset_ok, timeout_ok, woken_ok, partial_ok, all_ok);

    flexe_session_destroy(session);
    elf_symbols_destroy(symbols);
    return ok ? 0 : 1;
}
