/* Reproducible compute benchmark.
 *
 * Runs the bench_compute fixture for a fixed number of *rounds* rather than a
 * fixed cycle budget, so every run and every engine performs identical work.
 * Reports emulated MIPS and the real-time factor against a 240 MHz ESP32, and
 * prints a checksum the interpreter and the JIT must agree on -- which makes
 * this a JIT correctness check on real compiler output as well as a timer.
 */
#include "elf_symbols.h"
#include "flexe_session.h"
#include "jit.h"
#include "memory.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#define BATCH_CYCLES   200000
#define KERNEL_COUNT   4u
#define ESP32_HZ       240000000.0

volatile int emu_app_running = 1;

static double monotonic_seconds(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec + (double)ts.tv_nsec / 1e9;
}

static void usage(const char *argv0) {
    fprintf(stderr,
            "usage: %s [--no-jit] [--verify] [--rounds N] [--max-cycles N] "
            "FIRMWARE.bin FIRMWARE.elf\n", argv0);
}

int main(int argc, char **argv) {
    int argi = 1;
    int disable_jit = 0;
    int verify = 0;
    unsigned long rounds_target = 8;
    unsigned long long max_cycles = 20000000000ull;

    while (argi < argc && strncmp(argv[argi], "--", 2) == 0) {
        if (strcmp(argv[argi], "--no-jit") == 0) {
            disable_jit = 1;
            argi++;
        } else if (strcmp(argv[argi], "--rounds") == 0 && argi + 1 < argc) {
            rounds_target = strtoul(argv[argi + 1], NULL, 0);
            argi += 2;
        } else if (strcmp(argv[argi], "--verify") == 0) {
            verify = 1;
            argi++;
        } else if (strcmp(argv[argi], "--max-cycles") == 0 && argi + 1 < argc) {
            max_cycles = strtoull(argv[argi + 1], NULL, 0);
            argi += 2;
        } else {
            usage(argv[0]);
            return 2;
        }
    }
    if (argc - argi != 2 || rounds_target == 0) {
        usage(argv[0]);
        return 2;
    }

    elf_symbols_t *symbols = elf_symbols_load(argv[argi + 1]);
    uint32_t stage_addr = 0, rounds_addr = 0, sum_addr = 0, kernel_addr = 0;
    uint32_t limit_addr = 0;
    if (!symbols ||
        elf_symbols_find(symbols, "flexe_bench_stage", &stage_addr) != 0 ||
        elf_symbols_find(symbols, "flexe_bench_rounds", &rounds_addr) != 0 ||
        elf_symbols_find(symbols, "flexe_bench_checksum", &sum_addr) != 0 ||
        elf_symbols_find(symbols, "flexe_bench_kernel", &kernel_addr) != 0 ||
        elf_symbols_find(symbols, "flexe_bench_limit", &limit_addr) != 0) {
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

    if (verify) jit_set_verify(flexe_session_jit(session), true);

    xtensa_cpu_t *cpu = flexe_session_cpu(session, 0);
    xtensa_mem_t *mem = flexe_session_mem(session);

    /* Boot to the first completed round before starting the clock, so image
     * load, ROM-stub setup and JIT warm-up are excluded from the measurement
     * and every reported round is steady-state. */
    while (mem_read32(mem, rounds_addr) < 1u && cpu->cycle_count < max_cycles) {
        (void)flexe_session_run_core(session, 0, BATCH_CYCLES);
        flexe_session_post_batch(session, BATCH_CYCLES);
    }
    if (mem_read32(mem, rounds_addr) < 1u) {
        fprintf(stderr, "error: fixture never completed a round "
                        "(stage=%u cycles=%llu)\n",
                mem_read32(mem, stage_addr),
                (unsigned long long)cpu->cycle_count);
        flexe_session_destroy(session);
        elf_symbols_destroy(symbols);
        return 1;
    }

    uint64_t start_cycles = cpu->cycle_count;
    /* The fixture's loop() runs on core 1 (Arduino pins loopTask there), so
     * throughput has to count both cores' retired instructions. cycle_count
     * cannot be used for this: post_batch publishes the maximum of the two,
     * which is a shared timeline, not a sum of work. */
    xtensa_cpu_t *cpu1 = flexe_session_cpu(session, 1);
    uint64_t start_insns = xtensa_retired_insns(cpu) +
                           (cpu1 ? xtensa_retired_insns(cpu1) : 0);
    uint32_t start_rounds = mem_read32(mem, rounds_addr);
    uint32_t stop_rounds = start_rounds + (uint32_t)rounds_target;
    /* Let the fixture stop itself exactly on the target round. A host batch
     * spans several rounds, so stopping on the host side alone would leave
     * the two engines on different rounds with incomparable checksums. */
    mem_write32(mem, limit_addr, stop_rounds);
    double start_wall = monotonic_seconds();

    while (mem_read32(mem, rounds_addr) < stop_rounds &&
           cpu->cycle_count < max_cycles) {
        (void)flexe_session_run_core(session, 0, BATCH_CYCLES);
        flexe_session_post_batch(session, BATCH_CYCLES);
    }

    double wall = monotonic_seconds() - start_wall;
    uint64_t cycles = cpu->cycle_count - start_cycles;
    /* MIPS is instructions per second, so it must exclude the simulated time
     * the core spent halted -- otherwise a workload that sleeps reports
     * throughput it never achieved. The two differ little here by design,
     * but reporting both makes that visible rather than assumed. */
    uint64_t insns = xtensa_retired_insns(cpu) +
                     (cpu1 ? xtensa_retired_insns(cpu1) : 0) - start_insns;
    uint32_t done = mem_read32(mem, rounds_addr) - start_rounds;
    uint32_t checksum = mem_read32(mem, sum_addr);

    double mips = wall > 0.0 ? (double)insns / 1e6 / wall : 0.0;
    double realtime = wall > 0.0 ? (double)cycles / ESP32_HZ / wall : 0.0;

    printf("engine=%s rounds=%u cycles=%llu insns=%llu wall=%.3f mips=%.1f "
           "realtime=%.2fx checksum=0x%08X kernel=",
           flexe_session_jit(session) ? "jit" : "interp",
           done, (unsigned long long)cycles,
           (unsigned long long)insns, wall, mips, realtime,
           checksum);
    for (unsigned i = 0; i < KERNEL_COUNT; i++)
        printf("%s0x%08X", i ? "/" : "", mem_read32(mem, kernel_addr + i * 4u));
    printf("\n");

    if (verify) jit_verify_summary(flexe_session_jit(session));

    int ok = done == (uint32_t)rounds_target && checksum != 0u && wall > 0.0;
    if (!ok)
        fprintf(stderr, "error: benchmark did not reach %lu rounds "
                        "(cycle budget %llu exhausted?)\n",
                rounds_target, (unsigned long long)max_cycles);

    flexe_session_destroy(session);
    elf_symbols_destroy(symbols);
    return ok ? 0 : 1;
}
