/*
 * tools/test_aot.c — proof-of-concept test harness for the AOT
 * recompiler in tools/aot_recompile.py.
 *
 * Build:
 *   clang -O2 -I/tmp tools/test_aot.c -ldl -o /tmp/test_aot
 *
 * Run:
 *   /tmp/test_aot /tmp/firmware.aot.dylib
 *
 * What it does:
 *   1. dlopen()s the AOT dylib.
 *   2. Looks up aot_func_400e3314 (s_compare_reserved_regions). This
 *      function is: entry; l32i.n a2,a2,0; l32i.n a8,a3,0; sub a2,a2,a8;
 *      retw.n — i.e. *a2 - *a3.
 *   3. Sets up a fake xtensa_cpu_t with backing memory containing two
 *      known values, calls the AOT function, and checks the returned
 *      AR[2] equals the expected difference. Also diffs PC.
 *   4. Benchmarks the AOT function vs a hand-written reference
 *      "interpreter" that does the same work via an opcode switch.
 *      Reports the wall-clock speedup.
 */

#include <dlfcn.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "aot_runtime.h"

typedef uint32_t (*aot_fn_t)(xtensa_cpu_t *cpu);

/* Reference "interpreter" for the exact same 4 Xtensa instructions.
 * This is what flexe's xtensa_exec.c switch-based decoder effectively
 * does per instruction, minus the global decode dispatch overhead.
 * It's a FAVOURABLE baseline for the interpreter — real flexe has to
 * decode bytes every instruction, which is slower than this. */
enum { OP_ENTRY, OP_L32I_N, OP_SUB, OP_RETW_N };
struct ref_insn {
    int op;
    int r, s, t;
    int32_t imm;
};

static void ref_interp(xtensa_cpu_t *cpu, const struct ref_insn *prog,
                       int nprog) {
    for (int i = 0; i < nprog; i++) {
        const struct ref_insn *in = &prog[i];
        switch (in->op) {
        case OP_ENTRY:
            cpu->ar[((cpu->windowbase * 4) + in->s) & 63] -= (uint32_t)in->imm;
            break;
        case OP_L32I_N: {
            uint32_t addr = cpu->ar[((cpu->windowbase * 4) + in->s) & 63]
                            + (uint32_t)in->imm;
            uint32_t off = addr - cpu->mem_base;
            uint32_t v = 0;
            if (off + 4 <= cpu->mem_size) {
                memcpy(&v, cpu->mem + off, 4);
            }
            cpu->ar[((cpu->windowbase * 4) + in->t) & 63] = v;
            break;
        }
        case OP_SUB:
            cpu->ar[((cpu->windowbase * 4) + in->r) & 63] =
                cpu->ar[((cpu->windowbase * 4) + in->s) & 63] -
                cpu->ar[((cpu->windowbase * 4) + in->t) & 63];
            break;
        case OP_RETW_N:
            cpu->pc = cpu->ar[((cpu->windowbase * 4) + 0) & 63];
            cpu->_pc_written = 1;
            return;
        }
    }
}

static double now_s(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec + (double)ts.tv_nsec * 1e-9;
}

int main(int argc, char **argv) {
    const char *dylib = argc > 1 ? argv[1] : "/tmp/firmware.aot.dylib";
    void *h = dlopen(dylib, RTLD_NOW);
    if (!h) {
        fprintf(stderr, "dlopen(%s) failed: %s\n", dylib, dlerror());
        return 1;
    }
    const char *sym = "aot_func_400e3314";
    aot_fn_t fn = (aot_fn_t)dlsym(h, sym);
    if (!fn) {
        fprintf(stderr, "dlsym(%s) failed: %s\n", sym, dlerror());
        return 1;
    }

    /* Set up a fake CPU with 64 KiB of memory starting at 0x3ffb0000.
     * Store two 32-bit values at offset 0 and offset 64. */
    uint8_t *mem = calloc(1, 65536);
    const uint32_t BASE = 0x3ffb0000u;
    uint32_t a = 0xdeadbeefu;
    uint32_t b = 0x00000042u;
    memcpy(mem + 0,  &a, 4);
    memcpy(mem + 64, &b, 4);

    xtensa_cpu_t cpu = {0};
    cpu.mem = mem;
    cpu.mem_base = BASE;
    cpu.mem_size = 65536;
    cpu.windowbase = 0;
    cpu.ar[0] = 0x400e3320; /* return address (retw.n reads a0) */
    cpu.ar[1] = 0x3ffb8000; /* sp */
    cpu.ar[2] = BASE + 0;   /* arg0 ptr */
    cpu.ar[3] = BASE + 64;  /* arg1 ptr */
    cpu.pc = 0x400e3314;

    /* ---- correctness ---- */
    xtensa_cpu_t cpu_aot = cpu;
    (void)fn(&cpu_aot);
    uint32_t got = cpu_aot.ar[2];
    uint32_t want = a - b; /* = 0xdeadbead */
    printf("correctness:\n");
    printf("  AR[2]  got=0x%08x want=0x%08x %s\n", got, want,
           got == want ? "OK" : "MISMATCH");
    printf("  PC     got=0x%08x want=0x%08x %s\n", cpu_aot.pc, cpu.ar[0],
           cpu_aot.pc == cpu.ar[0] ? "OK" : "MISMATCH");

    /* Compare against reference interpreter */
    xtensa_cpu_t cpu_ref = cpu;
    struct ref_insn prog[] = {
        { OP_ENTRY,  0, 1, 0, 32 },
        { OP_L32I_N, 0, 2, 2, 0  },
        { OP_L32I_N, 0, 3, 8, 0  },
        { OP_SUB,    2, 2, 8, 0  },
        { OP_RETW_N, 0, 0, 0, 0  },
    };
    ref_interp(&cpu_ref, prog, 5);
    int same = (cpu_ref.ar[2] == cpu_aot.ar[2]) &&
               (cpu_ref.pc    == cpu_aot.pc);
    printf("  ref vs aot: %s (AR[2]=0x%08x PC=0x%08x)\n",
           same ? "OK" : "MISMATCH", cpu_ref.ar[2], cpu_ref.pc);
    if (!same || got != want) {
        fprintf(stderr, "FAIL\n");
        return 2;
    }

    /* ---- benchmark ---- */
    /* Do a big loop of (N / nprog) function calls, reset cpu state each
     * time so results are comparable. We reset only the three regs the
     * function mutates so reset overhead stays minimal. */
    const uint64_t ITERS = 50000000ULL;

    double t0 = now_s();
    uint32_t acc_aot = 0;
    for (uint64_t i = 0; i < ITERS; i++) {
        cpu_aot.ar[1] = 0x3ffb8000;
        cpu_aot.ar[2] = BASE + 0;
        cpu_aot.ar[3] = BASE + 64;
        cpu_aot.pc = 0x400e3314;
        fn(&cpu_aot);
        acc_aot ^= cpu_aot.ar[2];
    }
    double t_aot = now_s() - t0;

    t0 = now_s();
    uint32_t acc_ref = 0;
    for (uint64_t i = 0; i < ITERS; i++) {
        cpu_ref.ar[1] = 0x3ffb8000;
        cpu_ref.ar[2] = BASE + 0;
        cpu_ref.ar[3] = BASE + 64;
        cpu_ref.pc = 0x400e3314;
        ref_interp(&cpu_ref, prog, 5);
        acc_ref ^= cpu_ref.ar[2];
    }
    double t_ref = now_s() - t0;

    printf("\nbenchmark (%llu iterations x 5 Xtensa insns):\n",
           (unsigned long long)ITERS);
    printf("  ref interp: %.3f s (%.1f Minsn/s)  [acc=0x%08x]\n",
           t_ref, (ITERS * 5.0 / 1e6) / t_ref, acc_ref);
    printf("  aot dylib : %.3f s (%.1f Minsn/s)  [acc=0x%08x]\n",
           t_aot, (ITERS * 5.0 / 1e6) / t_aot, acc_aot);
    printf("  speedup   : %.2fx\n", t_ref / t_aot);

    dlclose(h);
    free(mem);
    return 0;
}
