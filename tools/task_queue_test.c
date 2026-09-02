/* Task-to-task queue and semaphore handoff.
 *
 * The existing queue coverage all has a peripheral ISR filling the queue,
 * which Flexe services through its own path. This drives the plain case: one
 * guest task sends, another is blocked waiting, and the send has to wake it.
 *
 * Each wait is bounded in time as well as checked for its result. A receive
 * that reports the right status only after sleeping through its entire
 * timeout has still lost the handoff, and time is the only thing that says so.
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

#define SUCCESS_MARKER 0x9EE00C0Bu
#define MAX_CYCLES     6000000000ull
#define RESULT_COUNT   16u

#define SEND_DELAY_US    50000u
#define SHORT_TIMEOUT_US 100000u
#define IMMEDIATE_US     5000u

#define VAL_A 0xC0DE0001u
#define VAL_B 0xC0DE0002u

volatile int emu_app_running = 1;

/* A wake that came from the producer lands near its delay; one that came from
 * the timeout expiring does not. */
static bool woken_by_producer(uint32_t elapsed_us) {
    return elapsed_us >= SEND_DELAY_US / 2u && elapsed_us <= SEND_DELAY_US * 2u;
}

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
        elf_symbols_find(symbols, "flexe_q_stage", &stage_addr) != 0 ||
        elf_symbols_find(symbols, "flexe_q_result", &result_addr) != 0) {
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
            fprintf(stderr, "[task-queue] stage=0x%08X cycles=%llu\n",
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
            fprintf(stderr, "[task-queue] tasks:\n%s", dump);
        fprintf(stderr, "[task-queue] pc=0x%08X cycles=%llu\n", cpu->pc,
                (unsigned long long)cpu->cycle_count);
    }
    int unhandled = periph_unhandled_count(flexe_session_periph(session));
    int unregistered = rom_stubs_unregistered_count(flexe_session_rom(session));

    printf("engine=%s stage=0x%08X timed=%u/%08X@%uus max=%u/%08X@%uus "
           "sem=%u@%uus expire=%u/%08X@%uus ready=%u/%u/%08X@%uus/%u "
           "unhandled=%d unregistered=%d\n",
           flexe_session_jit(session) ? "jit" : "interp", stage,
           r[0], r[1], r[2], r[3], r[4], r[5], r[6], r[7], r[8], r[9], r[10],
           r[11], r[12], r[13], r[14], r[15], unhandled, unregistered);

    /* (a) finite timeout, woken by the producer's send */
    bool timed_ok = r[0] == 1u && r[1] == VAL_A && woken_by_producer(r[2]);
    /* (b) portMAX_DELAY, same handoff */
    bool max_ok = r[3] == 1u && r[4] == VAL_B && woken_by_producer(r[5]);
    /* (c) semaphore given by another task */
    bool sem_ok = r[6] == 1u && woken_by_producer(r[7]);
    /* (d) genuine timeout: fails, blocks the whole time, leaves the buffer */
    bool expire_ok = r[8] == 0u && r[9] == 0xFFFFFFFFu &&
                     r[10] >= SHORT_TIMEOUT_US && r[10] <= SHORT_TIMEOUT_US * 2u;
    /* (e) already-queued item returns at once, and the count tracks it */
    bool ready_ok = r[11] == 1u && r[12] == 1u && r[13] == 0xABCD1234u &&
                    r[14] < IMMEDIATE_US && r[15] == 0u;

    int ok = stage == SUCCESS_MARKER && timed_ok && max_ok && sem_ok &&
             expire_ok && ready_ok && unhandled == 0 && unregistered == 0;
    if (!ok)
        fprintf(stderr, "[task-queue] timed=%d max=%d sem=%d expire=%d "
                        "ready=%d\n",
                timed_ok, max_ok, sem_ok, expire_ok, ready_ok);

    flexe_session_destroy(session);
    elf_symbols_destroy(symbols);
    return ok ? 0 : 1;
}
