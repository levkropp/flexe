/* JIT compiler tests: per-instruction differential testing vs interpreter.
 * Each test assembles a small block, runs via interpreter, runs via JIT,
 * and compares all state. */

#ifndef _MSC_VER

#include "jit.h"
#include "peripherals.h"
#include "savestate.h"

/* ===== Helpers ===== */

/* Assemble and run a sequence via interpreter, return state snapshot */
static void run_interp(xtensa_cpu_t *cpu, int n) {
    for (int i = 0; i < n; i++)
        xtensa_step(cpu);
}

/* Compare two CPU states (registers and key fields), report differences */
static int compare_state(const xtensa_cpu_t *a, const xtensa_cpu_t *b,
                         const char *test_name) {
    int diffs = 0;

    /* Check all 64 physical registers */
    for (int i = 0; i < 64; i++) {
        if (a->ar[i] != b->ar[i]) {
            fprintf(stderr, "  DIFF %s: ar[%d] interp=0x%08X jit=0x%08X\n",
                    test_name, i, a->ar[i], b->ar[i]);
            diffs++;
        }
    }

    if (a->pc != b->pc) {
        fprintf(stderr, "  DIFF %s: pc interp=0x%08X jit=0x%08X\n",
                test_name, a->pc, b->pc);
        diffs++;
    }
    if (a->sar != b->sar) {
        fprintf(stderr, "  DIFF %s: sar interp=%u jit=%u\n",
                test_name, a->sar, b->sar);
        diffs++;
    }
    if (a->ps != b->ps) {
        fprintf(stderr, "  DIFF %s: ps interp=0x%08X jit=0x%08X\n",
                test_name, a->ps, b->ps);
        diffs++;
    }
    if (a->lbeg != b->lbeg) {
        fprintf(stderr, "  DIFF %s: lbeg interp=0x%08X jit=0x%08X\n",
                test_name, a->lbeg, b->lbeg);
        diffs++;
    }
    if (a->lend != b->lend) {
        fprintf(stderr, "  DIFF %s: lend interp=0x%08X jit=0x%08X\n",
                test_name, a->lend, b->lend);
        diffs++;
    }
    if (a->lcount != b->lcount) {
        fprintf(stderr, "  DIFF %s: lcount interp=%u jit=%u\n",
                test_name, a->lcount, b->lcount);
        diffs++;
    }
    if (a->br != b->br) {
        fprintf(stderr, "  DIFF %s: br interp=0x%04X jit=0x%04X\n",
                test_name, a->br, b->br);
        diffs++;
    }
    if (a->windowbase != b->windowbase) {
        fprintf(stderr, "  DIFF %s: windowbase interp=%u jit=%u\n",
                test_name, a->windowbase, b->windowbase);
        diffs++;
    }
    if (a->windowstart != b->windowstart) {
        fprintf(stderr, "  DIFF %s: windowstart interp=0x%04X jit=0x%04X\n",
                test_name, a->windowstart, b->windowstart);
        diffs++;
    }
    for (int i = 0; i < 16; i++) {
        if (a->window_callsize[i] != b->window_callsize[i]) {
            fprintf(stderr,
                    "  DIFF %s: window_callsize[%d] interp=%u jit=%u\n",
                    test_name, i, a->window_callsize[i],
                    b->window_callsize[i]);
            diffs++;
        }
    }

#define COMPARE_U32(field) \
    do { \
        if (a->field != b->field) { \
            fprintf(stderr, "  DIFF %s: " #field \
                    " interp=0x%08X jit=0x%08X\n", \
                    test_name, a->field, b->field); \
            diffs++; \
        } \
    } while (0)
    COMPARE_U32(expstate);
    COMPARE_U32(threadptr);
    COMPARE_U32(fcr);
    COMPARE_U32(fsr);
    COMPARE_U32(f64r_lo);
    COMPARE_U32(f64r_hi);
    COMPARE_U32(f64s);
    COMPARE_U32(cpenable);
    COMPARE_U32(scompare1);
    COMPARE_U32(litbase);
    COMPARE_U32(acclo);
    COMPARE_U32(acchi);
    COMPARE_U32(mr[0]);
    COMPARE_U32(mr[1]);
    COMPARE_U32(mr[2]);
    COMPARE_U32(mr[3]);
#undef COMPARE_U32

    return diffs;
}

/* Run a block: first via interpreter, then via JIT, compare results */
static void test_block_differential(xtensa_cpu_t *template_cpu, int num_insns,
                                    const char *test_name) {
    /* Give isolated non-terminating instructions an explicit fallthrough and
     * exercise register allocation across more than one instruction. */
    uint32_t pad_pc = template_cpu->pc;
    for (int i = 0; i < num_insns; i++) {
        uint32_t insn;
        int ilen = xtensa_fetch(template_cpu, pad_pc, &insn);
        ASSERT_TRUE(ilen == 2 || ilen == 3);
        pad_pc += (uint32_t)ilen;
    }
    int padded_insns = num_insns;
    while (padded_insns < 2) {
        put_insn2(template_cpu, pad_pc, narrow(0xD, 15, 0, 3));
        pad_pc += 2;
        padded_insns++;
    }

    /* Interpreter run */
    xtensa_cpu_t interp_cpu;
    memcpy(&interp_cpu, template_cpu, sizeof(xtensa_cpu_t));
    run_interp(&interp_cpu, padded_insns);

    /* JIT run */
    xtensa_cpu_t jit_cpu;
    memcpy(&jit_cpu, template_cpu, sizeof(xtensa_cpu_t));
    jit_state_t *jit = jit_init();
    ASSERT_TRUE(jit != NULL);

    /* Force immediate compilation (set hot counter high) */
    uint32_t pc = jit_cpu.pc;
    for (int i = 0; i < JIT_HOT_THRESHOLD; i++)
        jit_get_block(jit, &jit_cpu, pc);

    jit_block_fn fn = jit_get_block(jit, &jit_cpu, pc);
    if (fn) {
        int ran = fn(&jit_cpu);
        jit_cpu.ccount += (uint32_t)ran;
        jit_cpu.cycle_count += (uint64_t)ran;
    } else {
        fprintf(stderr, "  FAIL %s: JIT refused padded differential block\n", test_name);
        jit_destroy(jit);
        test_failures++;
        return;
    }

    int diffs = compare_state(&interp_cpu, &jit_cpu, test_name);
    if (diffs == 0) {
        test_passes++;
    } else {
        test_failures += diffs;
    }

    jit_destroy(jit);
}

/* Differential path for blocks that deliberately side-exit before a complex
 * instruction. xtensa_run() must execute the native prefix and then resume in
 * the interpreter without exposing the split to its caller. */
static void test_run_differential(xtensa_cpu_t *template_cpu, int num_insns,
                                  const char *test_name) {
    xtensa_cpu_t interp_cpu;
    memcpy(&interp_cpu, template_cpu, sizeof(interp_cpu));
    interp_cpu.running = true;
    run_interp(&interp_cpu, num_insns);

    xtensa_cpu_t jit_cpu;
    memcpy(&jit_cpu, template_cpu, sizeof(jit_cpu));
    jit_cpu.running = true;
    jit_state_t *jit = jit_init();
    ASSERT_TRUE(jit != NULL);
    jit_install_hook(jit, &jit_cpu);

    uint32_t pc = jit_cpu.pc;
    for (int i = 0; i < JIT_HOT_THRESHOLD; i++)
        (void)jit_get_block(jit, &jit_cpu, pc);
    ASSERT_TRUE(jit_get_block(jit, &jit_cpu, pc) != NULL);

    jit_cpu._pc_written = true;
    int ran = xtensa_run(&jit_cpu, num_insns);
    if (ran != num_insns) {
        fprintf(stderr,
                "  DIAG %s: ran=%d pc=0x%08X running=%d halted=%d "
                "exception=%d cause=%u _pc_written=%d\n",
                test_name, ran, jit_cpu.pc, jit_cpu.running, jit_cpu.halted,
                jit_cpu.exception, jit_cpu.exccause, jit_cpu._pc_written);
    }
    ASSERT_EQ(ran, num_insns);

    int diffs = compare_state(&interp_cpu, &jit_cpu, test_name);
    if (diffs == 0)
        test_passes++;
    else
        test_failures += diffs;

    jit_destroy(jit);
}

/* ===== Individual instruction tests ===== */

TEST(test_jit_nop) {
    xtensa_cpu_t cpu;
    setup(&cpu);
    /* NOP.N at BASE, NOP.N at BASE+2 */
    put_insn2(&cpu, BASE,     narrow(0xD, 15, 0, 3)); /* NOP.N */
    put_insn2(&cpu, BASE + 2, narrow(0xD, 15, 0, 3)); /* NOP.N */
    test_block_differential(&cpu, 2, "nop");
    teardown(&cpu);
}

TEST(test_jit_movi) {
    xtensa_cpu_t cpu;
    setup(&cpu);
    /* MOVI a2, 42 — op0=2, r=0xA, s=2, imm8=42 */
    /* MOVI: r=0xA, s=reg, imm12 = s:imm8 → for val 42: s=0, imm8=42 */
    uint32_t insn = (0xA << 12) | (0 << 8) | (2 << 4) | 2;
    /* op0=2: bits 0-3. t=2: bits 4-7. s=0: bits 8-11. r=0xA: bits 12-15. imm8=42: bits 16-23 */
    insn = 0x2A0020u | (42u << 16);
    /* Let me construct properly: op0=2, t=2, s=0, r=0xA, imm8=42 */
    /* byte0 = (t << 4) | op0 = (2 << 4) | 2 = 0x22 */
    /* byte1 = (r << 4) | s  = (0xA << 4) | 0 = 0xA0 */
    /* byte2 = imm8 = 42 = 0x2A */
    insn = 0x22 | (0xA0 << 8) | (42u << 16);
    put_insn3(&cpu, BASE, insn);
    test_block_differential(&cpu, 1, "movi_42");
    teardown(&cpu);
}

TEST(test_jit_add) {
    xtensa_cpu_t cpu;
    setup(&cpu);
    ar_write(&cpu, 3, 100);
    ar_write(&cpu, 4, 200);
    /* ADD a2, a3, a4: op0=0, op1=0, op2=8, r=2, s=3, t=4 */
    put_insn3(&cpu, BASE, rrr(8, 0, 2, 3, 4));
    test_block_differential(&cpu, 1, "add");
    teardown(&cpu);
}

TEST(test_jit_sub) {
    xtensa_cpu_t cpu;
    setup(&cpu);
    ar_write(&cpu, 3, 500);
    ar_write(&cpu, 4, 200);
    /* SUB a2, a3, a4: op0=0, op1=0, op2=12, r=2, s=3, t=4 */
    put_insn3(&cpu, BASE, rrr(12, 0, 2, 3, 4));
    test_block_differential(&cpu, 1, "sub");
    teardown(&cpu);
}

TEST(test_jit_and) {
    xtensa_cpu_t cpu;
    setup(&cpu);
    ar_write(&cpu, 3, 0xFF00FF00);
    ar_write(&cpu, 4, 0x0F0F0F0F);
    put_insn3(&cpu, BASE, rrr(1, 0, 2, 3, 4));
    test_block_differential(&cpu, 1, "and");
    teardown(&cpu);
}

TEST(test_jit_or) {
    xtensa_cpu_t cpu;
    setup(&cpu);
    ar_write(&cpu, 3, 0xF0F0F0F0);
    ar_write(&cpu, 4, 0x0F0F0F0F);
    put_insn3(&cpu, BASE, rrr(2, 0, 2, 3, 4));
    test_block_differential(&cpu, 1, "or");
    teardown(&cpu);
}

TEST(test_jit_xor) {
    xtensa_cpu_t cpu;
    setup(&cpu);
    ar_write(&cpu, 3, 0xAAAAAAAA);
    ar_write(&cpu, 4, 0x55555555);
    put_insn3(&cpu, BASE, rrr(3, 0, 2, 3, 4));
    test_block_differential(&cpu, 1, "xor");
    teardown(&cpu);
}

TEST(test_jit_addi) {
    xtensa_cpu_t cpu;
    setup(&cpu);
    ar_write(&cpu, 3, 1000);
    /* ADDI a2, a3, -5: op0=2, r=0xC, s=3, t=2, imm8=sext(-5)=0xFB */
    uint32_t insn = 0x22 | (0xC3 << 8) | (0xFBu << 16);
    put_insn3(&cpu, BASE, insn);
    test_block_differential(&cpu, 1, "addi");
    teardown(&cpu);
}

TEST(test_jit_slli) {
    xtensa_cpu_t cpu;
    setup(&cpu);
    ar_write(&cpu, 3, 1);
    /* SLLI a2, a3, 4: op0=0, op1=1, op2=0, r=2, s=3, t=(32-4)=28 */
    /* SLLI: sa = 32 - (((op2&1)<<4)|t), so t=28, op2=0 → sa=32-28=4 */
    put_insn3(&cpu, BASE, rrr(0, 1, 2, 3, 28));
    test_block_differential(&cpu, 1, "slli");
    teardown(&cpu);
}

TEST(test_jit_srai) {
    xtensa_cpu_t cpu;
    setup(&cpu);
    ar_write(&cpu, 4, 0x80000000);
    /* SRAI a2, a4, 16: op0=0, op1=1, op2=2, r=2, s=16, t=4 */
    /* Actually s holds low 4 bits of shift, op2 bit holds bit 4 */
    /* sa = ((op2&1)<<4) | s. For sa=16: op2=3 (bit 0 set), s=0 → sa=16 */
    /* Wait: op2=2 → op2&1=0, s=16 won't fit. op2=3 → op2&1=1, s=0 → sa=16 */
    put_insn3(&cpu, BASE, rrr(3, 1, 2, 0, 4));
    test_block_differential(&cpu, 1, "srai");
    teardown(&cpu);
}

TEST(test_jit_extui) {
    xtensa_cpu_t cpu;
    setup(&cpu);
    ar_write(&cpu, 4, 0xDEADBEEF);
    /* EXTUI a2, a4, 8, 8: shift=8, maskwidth=8 → op2=7 */
    /* op1=4|5 (bit 0 = shift[4]). shift=8, so op1[0]=0 → op1=4, s=8 */
    /* EXTUI: op0=0, op1=4, op2=7, r=2, s=8, t=4 */
    put_insn3(&cpu, BASE, rrr(7, 4, 2, 8, 4));
    test_block_differential(&cpu, 1, "extui");
    teardown(&cpu);
}

/* The SAR-driven shifts were covered against the interpreter in test_shift.c
 * but never against the JIT, and SRC shipped miscompiled: the emitter built
 * the 64-bit (as:at) concatenation and then clobbered its high half with a
 * 32-bit `or eax, ebx`, which zero-extends. SRC silently degenerated to
 * `at >> SAR`, so every compiler-emitted rotate -- GCC lowers `(x << n) |
 * (x >> (32 - n))` to SSAI+SRC -- produced a wrong result once its block got
 * hot. Sweep the whole family, and sweep SAR: SAR 0 and 32 are the boundaries
 * where the high or low half alone is the answer, so a broken funnel still
 * looks right there. */
static void jit_src_case(uint32_t sar, uint32_t as, uint32_t at,
                         const char *name) {
    xtensa_cpu_t cpu;
    setup(&cpu);
    ar_write(&cpu, 4, as);
    ar_write(&cpu, 5, at);
    cpu.sar = sar;
    put_insn3(&cpu, BASE, rrr(8, 1, 3, 4, 5));  /* SRC a3, a4, a5 */
    test_block_differential(&cpu, 1, name);
    teardown(&cpu);
}

TEST(test_jit_src_funnel) {
    jit_src_case(0,  0xAAAAAAAAu, 0x55555555u, "src_sar0");
    jit_src_case(8,  0xAA0000FFu, 0x00BB0000u, "src_sar8");
    jit_src_case(16, 0x12345678u, 0x9ABCDEF0u, "src_sar16");
    jit_src_case(31, 0x12345678u, 0x9ABCDEF0u, "src_sar31");
    jit_src_case(32, 0x12345678u, 0x9ABCDEF0u, "src_sar32");
    /* The exact rotate the bench_compute mix kernel performs, and the value
     * it converged on: a broken funnel returns 0x00000007 and then stays
     * there forever, because 7 is a fixed point of `x >> 27`. */
    jit_src_case(27, 0x3D2154BFu, 0x3D2154BFu, "src_rotate27");
}

TEST(test_jit_sll_srl_sra) {
    static const uint32_t sars[] = {0, 1, 8, 16, 31, 32};
    for (unsigned i = 0; i < sizeof(sars) / sizeof(sars[0]); i++) {
        xtensa_cpu_t cpu;
        setup(&cpu);
        ar_write(&cpu, 4, 0x87654321u);
        cpu.sar = sars[i];
        put_insn3(&cpu, BASE, rrr(10, 1, 3, 4, 0)); /* SLL a3, a4 */
        test_block_differential(&cpu, 1, "sll");
        teardown(&cpu);

        setup(&cpu);
        ar_write(&cpu, 5, 0x87654321u);
        cpu.sar = sars[i];
        put_insn3(&cpu, BASE, rrr(9, 1, 3, 0, 5));  /* SRL a3, a5 */
        test_block_differential(&cpu, 1, "srl");
        teardown(&cpu);

        setup(&cpu);
        ar_write(&cpu, 5, 0x87654321u);
        cpu.sar = sars[i];
        put_insn3(&cpu, BASE, rrr(11, 1, 3, 0, 5)); /* SRA a3, a5 */
        test_block_differential(&cpu, 1, "sra");
        teardown(&cpu);
    }
}

TEST(test_jit_nsa_nsau) {
    xtensa_cpu_t cpu;
    setup(&cpu);
    ar_write(&cpu, 2, 0u);
    ar_write(&cpu, 3, 0x80000000u);
    ar_write(&cpu, 8, 0x00010000u);
    ar_write(&cpu, 9, 0xFFFEFFFFu);

    /* ST1/r=14,15 write t from source s. Cover both special zero results and
     * equal-magnitude positive/negative values in one compilable block. */
    put_insn3(&cpu, BASE,      rrr(4, 0, 14, 2, 4)); /* NSA  a4, a2 */
    put_insn3(&cpu, BASE + 3u, rrr(4, 0, 15, 2, 5)); /* NSAU a5, a2 */
    put_insn3(&cpu, BASE + 6u, rrr(4, 0, 14, 9, 6)); /* NSA  a6, a9 */
    put_insn3(&cpu, BASE + 9u, rrr(4, 0, 15, 8, 7)); /* NSAU a7, a8 */
    test_block_differential(&cpu, 4, "nsa_nsau");
    teardown(&cpu);
}

TEST(test_jit_mull) {
    xtensa_cpu_t cpu;
    setup(&cpu);
    ar_write(&cpu, 3, 7);
    ar_write(&cpu, 4, 6);
    /* MULL a2, a3, a4: op0=0, op1=2, op2=8, r=2, s=3, t=4 */
    put_insn3(&cpu, BASE, rrr(8, 2, 2, 3, 4));
    test_block_differential(&cpu, 1, "mull");
    teardown(&cpu);
}

TEST(test_jit_mulsh) {
    xtensa_cpu_t cpu;
    setup(&cpu);
    ar_write(&cpu, 3, 0x80000000u);
    ar_write(&cpu, 4, 2u);
    /* MULSH a2, a3, a4: high half of INT32_MIN * 2 is 0xffffffff. */
    put_insn3(&cpu, BASE, rrr(11, 2, 2, 3, 4));
    test_block_differential(&cpu, 1, "mulsh");
    teardown(&cpu);
}

TEST(test_jit_neg) {
    xtensa_cpu_t cpu;
    setup(&cpu);
    ar_write(&cpu, 4, 42);
    /* NEG a2, a4: op0=0, op1=0, op2=6, r=2, s=0, t=4 */
    put_insn3(&cpu, BASE, rrr(6, 0, 2, 0, 4));
    test_block_differential(&cpu, 1, "neg");
    teardown(&cpu);
}

TEST(test_jit_mov_n) {
    xtensa_cpu_t cpu;
    setup(&cpu);
    ar_write(&cpu, 3, 0x12345678);
    /* MOV.N a2, a3: op0=0xD, r=0, s=3, t=2 */
    put_insn2(&cpu, BASE, narrow(0xD, 0, 3, 2));
    test_block_differential(&cpu, 1, "mov.n");
    teardown(&cpu);
}

TEST(test_jit_add_n) {
    xtensa_cpu_t cpu;
    setup(&cpu);
    ar_write(&cpu, 3, 100);
    ar_write(&cpu, 4, 200);
    /* ADD.N a2, a3, a4: op0=0xA, r=2, s=3, t=4 */
    put_insn2(&cpu, BASE, narrow(0xA, 4, 3, 2));
    test_block_differential(&cpu, 1, "add.n");
    teardown(&cpu);
}

TEST(test_jit_addi_n) {
    xtensa_cpu_t cpu;
    setup(&cpu);
    ar_write(&cpu, 3, 100);
    /* ADDI.N a2, a3, 5: op0=0xB, r=2, s=3, t=5 */
    put_insn2(&cpu, BASE, narrow(0xB, 5, 3, 2));
    test_block_differential(&cpu, 1, "addi.n");
    teardown(&cpu);
}

TEST(test_jit_addi_n_minus1) {
    xtensa_cpu_t cpu;
    setup(&cpu);
    ar_write(&cpu, 3, 100);
    /* ADDI.N a2, a3, -1: t=0 means -1 */
    put_insn2(&cpu, BASE, narrow(0xB, 0, 3, 2));
    test_block_differential(&cpu, 1, "addi.n.-1");
    teardown(&cpu);
}

TEST(test_jit_l32i_s32i) {
    xtensa_cpu_t cpu;
    setup(&cpu);
    /* Write a value to memory, load it via L32I */
    uint32_t data_addr = 0x3FFE0000;
    mem_write32(cpu.mem, data_addr, 0xCAFEBABE);
    ar_write(&cpu, 3, data_addr);
    /* L32I a2, a3, 0: op0=2, r=2, s=3, t=2, imm8=0 */
    uint32_t insn = 0x22 | (0x23 << 8) | (0x00 << 16);
    put_insn3(&cpu, BASE, insn);
    test_block_differential(&cpu, 1, "l32i");
    teardown(&cpu);
}

TEST(test_jit_s32i) {
    xtensa_cpu_t cpu;
    setup(&cpu);
    uint32_t data_addr = 0x3FFE0000;
    ar_write(&cpu, 3, data_addr);
    ar_write(&cpu, 2, 0xDEADBEEF);
    /* S32I a2, a3, 0: op0=2, r=6, s=3, t=2, imm8=0 */
    uint32_t insn = 0x22 | (0x63 << 8) | (0x00 << 16);
    put_insn3(&cpu, BASE, insn);
    test_block_differential(&cpu, 1, "s32i");
    /* Also verify memory */
    xtensa_cpu_t verify;
    memcpy(&verify, &cpu, sizeof(cpu));
    xtensa_step(&verify);
    ASSERT_EQ(mem_read32(verify.mem, data_addr), 0xDEADBEEF);
    teardown(&cpu);
}

/* Byte and half-word memory ops had no JIT coverage at all, and S8I shipped
 * broken because of it: emit_rex() drops a bare 0x40 prefix as an
 * optimisation, but for an 8-bit register operand that prefix is exactly what
 * distinguishes BPL from CH. The store therefore wrote bits 8-15 of RCX --
 * the page index the address lowering had just computed into it -- instead of
 * the guest value, so every S8I stored a byte of its own target address.
 *
 * compare_state() only looks at registers, so these cases compare memory
 * explicitly; a store bug is invisible otherwise. The two engines run on
 * separate CPUs with separate memories so neither can observe the other. */
static void ldst_program(xtensa_cpu_t *cpu, unsigned subop, int treg,
                         int sreg, uint32_t addr, uint32_t treg_val) {
    setup(cpu);
    mem_write32(cpu->mem, (addr & ~3u), 0xA5A5A5A5u);
    mem_write32(cpu->mem, (addr & ~3u) + 4u, 0x5A5A5A5Au);
    ar_write(cpu, sreg, addr);
    ar_write(cpu, treg, treg_val);
    /* RRI8: byte0 = (t << 4) | op0(2), byte1 = (subop << 4) | s, byte2 = 0 */
    put_insn3(cpu, BASE, (uint32_t)(((unsigned)treg << 4) | 2u) |
                         ((uint32_t)(((subop & 0xFu) << 4) |
                                     ((unsigned)sreg & 0xFu)) << 8));
    /* Keep the memory differential case multi-instruction so it also checks
     * register allocation across adjacent operations. */
    put_insn2(cpu, BASE + 3u, narrow(0xD, 15, 0, 3));
    put_insn2(cpu, BASE + 5u, narrow(0xD, 15, 0, 3));
    put_insn2(cpu, BASE + 7u, narrow(0xD, 15, 0, 3));
}

static void ldst_check(unsigned subop, int treg, int sreg, uint32_t addr,
                       uint32_t treg_val, const char *name) {
    xtensa_cpu_t ic, jc;
    ldst_program(&ic, subop, treg, sreg, addr, treg_val);
    run_interp(&ic, 4);

    ldst_program(&jc, subop, treg, sreg, addr, treg_val);
    jit_state_t *jit = jit_init();
    ASSERT_TRUE(jit != NULL);
    for (int i = 0; i < JIT_HOT_THRESHOLD; i++)
        (void)jit_get_block(jit, &jc, BASE);
    jit_block_fn fn = jit_get_block(jit, &jc, BASE);
    if (!fn) {
        fprintf(stderr, "  FAIL %s: JIT refused the block\n", name);
        test_failures++;
        jit_destroy(jit);
        teardown(&ic);
        teardown(&jc);
        return;
    }
    (void)fn(&jc);

    int diffs = compare_state(&ic, &jc, name);
    for (unsigned w = 0; w < 2u; w++) {
        uint32_t a = (addr & ~3u) + w * 4u;
        uint32_t iv = mem_read32(ic.mem, a), jv = mem_read32(jc.mem, a);
        if (iv != jv) {
            fprintf(stderr, "  DIFF %s: mem[0x%08X] interp=0x%08X jit=0x%08X\n",
                    name, a, iv, jv);
            diffs++;
        }
    }
    if (diffs == 0) test_passes++; else test_failures += diffs;

    jit_destroy(jit);
    teardown(&ic);
    teardown(&jc);
}

TEST(test_jit_byte_halfword_memory_ops) {
    /* Cover mapped guest registers (a2-a6 live in host registers) and spilled
     * ones (a7+ live in ar[]), and every byte offset within a word. */
    static const int tregs[] = {2, 3, 4, 5, 6, 7, 12};
    for (unsigned i = 0; i < sizeof(tregs) / sizeof(tregs[0]); i++) {
        int t = tregs[i];
        for (unsigned off = 0; off < 4u; off++) {
            uint32_t addr = 0x3FFE0000u + off;
            ldst_check(0x4, t, 9, addr, 0x1234567Bu, "s8i");
            ldst_check(0x0, t, 9, addr, 0xDEADBEEFu, "l8ui");
        }
        for (unsigned off = 0; off < 4u; off += 2u) {
            uint32_t addr = 0x3FFE0000u + off;
            ldst_check(0x5, t, 9, addr, 0x1234F0E1u, "s16i");
            ldst_check(0x1, t, 9, addr, 0xDEADBEEFu, "l16ui");
            ldst_check(0x9, t, 9, addr, 0xDEADBEEFu, "l16si");
        }
    }
}

TEST(test_jit_multi_insn_block) {
    xtensa_cpu_t cpu;
    setup(&cpu);
    ar_write(&cpu, 3, 10);
    ar_write(&cpu, 4, 20);
    /* Block: ADD a2, a3, a4; ADD a5, a2, a3; ADD a6, a5, a4 */
    put_insn3(&cpu, BASE,     rrr(8, 0, 2, 3, 4));  /* ADD a2, a3, a4 = 30 */
    put_insn3(&cpu, BASE + 3, rrr(8, 0, 5, 2, 3));  /* ADD a5, a2, a3 = 40 */
    put_insn3(&cpu, BASE + 6, rrr(8, 0, 6, 5, 4));  /* ADD a6, a5, a4 = 60 */
    test_block_differential(&cpu, 3, "multi_add");
    teardown(&cpu);
}

TEST(test_jit_moveqz) {
    xtensa_cpu_t cpu;
    setup(&cpu);
    ar_write(&cpu, 2, 0x11111111); /* will be overwritten */
    ar_write(&cpu, 3, 0x22222222); /* source */
    ar_write(&cpu, 4, 0);          /* condition: zero → move */
    /* MOVEQZ a2, a3, a4: op0=0, op1=3, op2=8, r=2, s=3, t=4 */
    put_insn3(&cpu, BASE, rrr(8, 3, 2, 3, 4));
    test_block_differential(&cpu, 1, "moveqz_taken");
    teardown(&cpu);
}

TEST(test_jit_moveqz_not_taken) {
    xtensa_cpu_t cpu;
    setup(&cpu);
    ar_write(&cpu, 2, 0x11111111);
    ar_write(&cpu, 3, 0x22222222);
    ar_write(&cpu, 4, 1);          /* condition: non-zero → no move */
    put_insn3(&cpu, BASE, rrr(8, 3, 2, 3, 4));
    test_block_differential(&cpu, 1, "moveqz_not_taken");
    teardown(&cpu);
}

TEST(test_jit_min_max) {
    xtensa_cpu_t cpu;
    setup(&cpu);
    ar_write(&cpu, 3, 100);
    ar_write(&cpu, 4, 200);
    /* MIN a2, a3, a4: op0=0, op1=3, op2=4, r=2, s=3, t=4 */
    put_insn3(&cpu, BASE, rrr(4, 3, 2, 3, 4));
    test_block_differential(&cpu, 1, "min");
    teardown(&cpu);
}

TEST(test_jit_addx2) {
    xtensa_cpu_t cpu;
    setup(&cpu);
    ar_write(&cpu, 3, 10);
    ar_write(&cpu, 4, 5);
    put_insn3(&cpu, BASE, rrr(9, 0, 2, 3, 4)); /* ADDX2 a2, a3, a4 = 25 */
    test_block_differential(&cpu, 1, "addx2");
    teardown(&cpu);
}

TEST(test_jit_srli) {
    xtensa_cpu_t cpu;
    setup(&cpu);
    ar_write(&cpu, 4, 0xFF);
    /* SRLI a2, a4, 4: op0=0, op1=1, op2=4, r=2, s=4, t=4 */
    put_insn3(&cpu, BASE, rrr(4, 1, 2, 4, 4));
    test_block_differential(&cpu, 1, "srli");
    teardown(&cpu);
}

TEST(test_jit_rsil) {
    xtensa_cpu_t cpu;
    setup(&cpu);
    cpu.ps = 0x00040023; /* INTLEVEL=3 */
    /* RSIL a2, 5: op0=0, op1=0, op2=0, r=6, s=5, t=2 */
    put_insn3(&cpu, BASE, rrr(0, 0, 6, 5, 2));
    test_block_differential(&cpu, 1, "rsil");
    teardown(&cpu);
}

TEST(test_jit_init_destroy) {
    jit_state_t *jit = jit_init();
    ASSERT_TRUE(jit != NULL);
    const jit_stats_t *stats = jit_get_stats(jit);
    ASSERT_EQ(stats->blocks_compiled, 0);
    jit_destroy(jit);
}

TEST(test_branch_target_ring_is_disabled_without_a_jit_consumer) {
    xtensa_cpu_t cpu;
    setup(&cpu);
    put_insn2(&cpu, BASE, narrow(0xD, 15, 0, 3)); /* NOP.N */

    /* The initial PC is a dispatch boundary, but an interpreter-only CPU has
     * no consumer for the branch-target discovery ring. */
    cpu.running = true;
    cpu._pc_written = true;
    ASSERT_FALSE(cpu.record_branch_targets);
    ASSERT_EQ(xtensa_run(&cpu, 1), 1);
    ASSERT_EQ(cpu.br_ring_idx, 0u);

    /* Installing a JIT turns discovery on and records that same boundary. */
    cpu.pc = BASE;
    cpu._pc_written = true;
    jit_state_t *jit = jit_init();
    ASSERT_TRUE(jit != NULL);
    jit_install_hook(jit, &cpu);
    ASSERT_TRUE(cpu.record_branch_targets);
    ASSERT_EQ(xtensa_run(&cpu, 1), 1);
    ASSERT_EQ(cpu.br_ring_idx, 1u);
    ASSERT_EQ(cpu.br_ring[0], BASE);

    jit_destroy(jit);
    teardown(&cpu);
}

TEST(test_jit_fallback_stops_at_debug_break_boundary) {
    xtensa_cpu_t cpu;
    setup(&cpu);
    put_insn2(&cpu, BASE, narrow(0xD, 15, 0, 3)); /* NOP.N */
    put_insn3(&cpu, BASE + 2u, rrr(0, 0, 4, 0, 0)); /* BREAK 0, 0 */
    put_insn3(&cpu, BASE + 5u, rri8(0xA, 0, 3, 42)); /* MOVI a3, 42 */
    cpu.running = true;

    jit_state_t *jit = jit_init();
    ASSERT_TRUE(jit != NULL);
    jit_install_hook(jit, &cpu);
    for (int i = 0; i < JIT_HOT_THRESHOLD; i++)
        (void)jit_get_block(jit, &cpu, BASE);
    ASSERT_TRUE(jit_get_block(jit, &cpu, BASE) != NULL);

    ASSERT_EQ(jit_run(jit, &cpu, 10), 2);
    ASSERT_TRUE(cpu.debug_break);
    ASSERT_EQ(cpu.debugcause, XT_DEBUGCAUSE_BREAK);
    ASSERT_EQ(cpu.dbg_prev_pc, BASE + 2u);
    ASSERT_EQ(cpu.pc, BASE + 5u);
    ASSERT_EQ(ar_read(&cpu, 3), 0u);
    ASSERT_EQ(jit_run(jit, &cpu, 10), 0);

    jit_destroy(jit);
    teardown(&cpu);
}

TEST(test_jit_verify_toggle_recompiles_blocks) {
    xtensa_cpu_t cpu;
    setup(&cpu);
    for (unsigned i = 0; i < 4; i++)
        put_insn2(&cpu, BASE + i * 2u, narrow(0xD, 15, 0, 3));

    jit_state_t *jit = jit_init();
    ASSERT_TRUE(jit != NULL);
    for (int i = 0; i < JIT_HOT_THRESHOLD; i++)
        (void)jit_get_block(jit, &cpu, BASE);
    ASSERT_TRUE(jit_get_block(jit, &cpu, BASE) != NULL);

    uint64_t flushes = jit_get_stats(jit)->cache_flushes;
    jit_set_verify(jit, true);
    ASSERT_TRUE(jit_verify_enabled(jit));
    ASSERT_EQ64(jit_get_stats(jit)->cache_flushes, flushes + 1u);
    ASSERT_TRUE(jit_get_block(jit, &cpu, BASE) == NULL);
    ASSERT_EQ64(jit_verify_mismatch_count(jit), 0u);

    /* Setting the already-active mode is not another cache transition. */
    jit_set_verify(jit, true);
    ASSERT_EQ64(jit_get_stats(jit)->cache_flushes, flushes + 1u);

    jit_set_verify(jit, false);
    ASSERT_FALSE(jit_verify_enabled(jit));
    ASSERT_EQ64(jit_get_stats(jit)->cache_flushes, flushes + 2u);

    jit_destroy(jit);
    teardown(&cpu);
}

TEST(test_jit_verify_keeps_cross_block_chains_disabled) {
    xtensa_cpu_t cpu;
    setup(&cpu);
    const uint32_t target = BASE + 0x40u;

    put_insn2(&cpu, BASE,      narrow(0xD, 15, 0, 3));
    put_insn2(&cpu, BASE + 2u, narrow(0xD, 15, 0, 3));
    put_insn2(&cpu, BASE + 4u, narrow(0xD, 15, 0, 3));
    int32_t joff = (int32_t)target - (int32_t)(BASE + 6u + 3u) - 1;
    put_insn3(&cpu, BASE + 6u,
              (((uint32_t)joff & 0x3FFFFu) << 6) | 6u);

    put_insn2(&cpu, target,      narrow(0xD, 15, 0, 3));
    put_insn2(&cpu, target + 2u, narrow(0xD, 15, 0, 3));
    put_insn2(&cpu, target + 4u, narrow(0xD, 15, 0, 3));
    put_insn2(&cpu, target + 6u, narrow(0xD, 15, 0, 0)); /* RET.N */
    ar_write(&cpu, 0, BASE + 0x100u);

    jit_state_t *jit = jit_init();
    ASSERT_TRUE(jit != NULL);
    jit_set_verify(jit, true);
    for (int i = 0; i < JIT_HOT_THRESHOLD; i++)
        (void)jit_get_block(jit, &cpu, target);
    ASSERT_TRUE(jit_get_block(jit, &cpu, target) != NULL);
    for (int i = 0; i < JIT_HOT_THRESHOLD; i++)
        (void)jit_get_block(jit, &cpu, BASE);
    jit_block_fn fn = jit_get_block(jit, &cpu, BASE);
    ASSERT_TRUE(fn != NULL);

    cpu.pc = BASE;
    ASSERT_EQ(fn(&cpu), 4);
    ASSERT_EQ(cpu.pc, target);
    ASSERT_EQ64(jit_get_stats(jit)->chains_patched, 0u);

    jit_destroy(jit);
    teardown(&cpu);
}

TEST(test_jit_hot_threshold) {
    xtensa_cpu_t cpu;
    setup(&cpu);
    /* Two instructions are the minimum profitable standalone block. */
    put_insn2(&cpu, BASE,     narrow(0xD, 15, 0, 3)); /* NOP.N */
    put_insn2(&cpu, BASE + 2, narrow(0xD, 15, 0, 3)); /* NOP.N */

    jit_state_t *jit = jit_init();
    ASSERT_TRUE(jit != NULL);

    /* Should not compile before threshold */
    for (int i = 0; i < JIT_HOT_THRESHOLD - 1; i++) {
        jit_block_fn fn = jit_get_block(jit, &cpu, BASE);
        ASSERT_TRUE(fn == NULL);
    }

    /* Should compile at threshold */
    jit_block_fn fn = jit_get_block(jit, &cpu, BASE);
    ASSERT_TRUE(fn != NULL);

    ASSERT_EQ(jit_get_stats(jit)->blocks_compiled, 1);
    jit_destroy(jit);
    teardown(&cpu);
}

TEST(test_jit_hash_collision_uses_free_way) {
    xtensa_cpu_t cpu;
    setup(&cpu);

    /* These two (PC, windowbase=0, loop-variant=0) keys intentionally map to
     * the same set under jit_mix(). Both must coexist: an empty second way
     * takes precedence over evicting the compiled block in the first way. */
    const uint32_t first = BASE + 0x560u;
    const uint32_t second = BASE + 0x1080u;
    put_insn2(&cpu, first,      narrow(0xD, 15, 0, 3)); /* NOP.N */
    put_insn2(&cpu, first + 2u, narrow(0xD, 15, 0, 0)); /* RET.N */
    put_insn2(&cpu, second,      narrow(0xD, 15, 0, 3));
    put_insn2(&cpu, second + 2u, narrow(0xD, 15, 0, 0));

    jit_state_t *jit = jit_init();
    ASSERT_TRUE(jit != NULL);
    for (int i = 0; i < JIT_HOT_THRESHOLD; i++)
        (void)jit_get_block(jit, &cpu, first);
    jit_block_fn first_fn = jit_get_block(jit, &cpu, first);
    ASSERT_TRUE(first_fn != NULL);

    for (int i = 0; i < JIT_HOT_THRESHOLD; i++)
        (void)jit_get_block(jit, &cpu, second);
    ASSERT_TRUE(jit_get_block(jit, &cpu, second) != NULL);

    ASSERT_TRUE(jit_get_block(jit, &cpu, first) == first_fn);
    ASSERT_EQ(jit_get_stats(jit)->blocks_compiled, 2u);

    jit_destroy(jit);
    teardown(&cpu);
}

TEST(test_jit_one_instruction_straight_line_compiles_when_hot) {
    xtensa_cpu_t cpu;
    setup(&cpu);
    put_insn2(&cpu, BASE,     narrow(0xD, 15, 0, 3)); /* NOP.N */
    put_insn2(&cpu, BASE + 2, narrow(0xD, 15, 0, 2)); /* ILL.N ends scan */

    jit_state_t *jit = jit_init();
    ASSERT_TRUE(jit != NULL);
    for (int i = 0; i < JIT_HOT_THRESHOLD - 1; i++)
        ASSERT_TRUE(jit_get_block(jit, &cpu, BASE) == NULL);
    ASSERT_TRUE(jit_get_block(jit, &cpu, BASE) != NULL);
    ASSERT_EQ(jit_get_stats(jit)->blocks_compiled, 1u);

    jit_destroy(jit);
    teardown(&cpu);
}

TEST(test_jit_single_instruction_chain_target_is_native) {
    xtensa_cpu_t cpu;
    setup(&cpu);
    const uint32_t target = BASE;
    const uint32_t source = BASE + 0x40u;
    const uint32_t return_pc = BASE + 0x100u;

    /* Compile a backward predecessor before the lone RET becomes hot. Static
     * descent must still compile and chain the one-instruction target without
     * waiting for a separate dispatcher threshold. */
    put_insn2(&cpu, target, narrow(0xD, 15, 0, 0)); /* RET.N */
    put_insn2(&cpu, source,      narrow(0xD, 15, 0, 3));
    put_insn2(&cpu, source + 2u, narrow(0xD, 15, 0, 3));
    put_insn2(&cpu, source + 4u, narrow(0xD, 15, 0, 3));
    int32_t joff = (int32_t)target - (int32_t)(source + 6u + 3u) - 1;
    put_insn3(&cpu, source + 6u,
              (((uint32_t)joff & 0x3FFFFu) << 6) | 6u);
    ar_write(&cpu, 0, return_pc);

    jit_state_t *jit = jit_init();
    ASSERT_TRUE(jit != NULL);
    for (int i = 0; i < JIT_HOT_THRESHOLD; i++)
        (void)jit_get_block(jit, &cpu, source);
    jit_block_fn fn = jit_get_block(jit, &cpu, source);
    ASSERT_TRUE(fn != NULL);
    ASSERT_TRUE(jit_get_block(jit, &cpu, target) != NULL);
    ASSERT_EQ(jit_get_stats(jit)->blocks_compiled, 2u);
    ASSERT_EQ(jit_get_stats(jit)->chains_patched, 1u);

    cpu.pc = source;
    ASSERT_EQ(fn(&cpu), 5);
    ASSERT_EQ(cpu.pc, return_pc);

    jit_destroy(jit);
    teardown(&cpu);
}

TEST(test_jit_precompiled_fallthrough_chain_is_not_overwritten) {
    xtensa_cpu_t cpu;
    setup(&cpu);
    const uint32_t target = BASE;
    const uint32_t source = BASE + 0x40u;
    const uint32_t return_pc = BASE + 0x100u;

    put_insn2(&cpu, target, narrow(0xD, 15, 0, 0)); /* RET.N */
    put_insn2(&cpu, source,      narrow(0xD, 15, 0, 3));
    put_insn2(&cpu, source + 2u, narrow(0xD, 15, 0, 3));
    put_insn2(&cpu, source + 4u, narrow(0xD, 15, 0, 3));
    int32_t joff = (int32_t)target - (int32_t)(source + 6u + 3u) - 1;
    put_insn3(&cpu, source + 6u,
              (((uint32_t)joff & 0x3FFFFu) << 6) | 6u);
    ar_write(&cpu, 0, return_pc);

    jit_state_t *jit = jit_init();
    ASSERT_TRUE(jit != NULL);
    for (int i = 0; i < JIT_HOT_THRESHOLD; i++)
        (void)jit_get_block(jit, &cpu, target);
    ASSERT_TRUE(jit_get_block(jit, &cpu, target) != NULL);
    for (int i = 0; i < JIT_HOT_THRESHOLD; i++)
        (void)jit_get_block(jit, &cpu, source);
    jit_block_fn source_fn = jit_get_block(jit, &cpu, source);
    ASSERT_TRUE(source_fn != NULL);
    ASSERT_EQ(jit_get_stats(jit)->chains_patched, 1u);

    cpu.pc = source;
    ASSERT_EQ(source_fn(&cpu), 5);
    ASSERT_EQ(cpu.pc, return_pc);

    jit_destroy(jit);
    teardown(&cpu);
}

TEST(test_jit_static_chain_stops_at_timer_deadline) {
    xtensa_cpu_t cpu;
    setup(&cpu);
    const uint32_t target = BASE;
    const uint32_t source = BASE + 0x40u;

    put_insn2(&cpu, target, narrow(0xD, 15, 0, 3)); /* NOP.N */
    put_insn2(&cpu, target + 2u, narrow(0xD, 15, 0, 0)); /* RET.N */
    for (unsigned i = 0; i < 3u; i++)
        put_insn2(&cpu, source + i * 2u,
                  narrow(0xD, 15, 0, 3)); /* NOP.N */
    int32_t joff = (int32_t)target - (int32_t)(source + 6u + 3u) - 1;
    put_insn3(&cpu, source + 6u,
              (((uint32_t)joff & 0x3FFFFu) << 6) | 6u);

    jit_state_t *jit = jit_init();
    ASSERT_TRUE(jit != NULL);
    for (int i = 0; i < JIT_HOT_THRESHOLD; i++)
        (void)jit_get_block(jit, &cpu, target);
    ASSERT_TRUE(jit_get_block(jit, &cpu, target) != NULL);
    for (int i = 0; i < JIT_HOT_THRESHOLD; i++)
        (void)jit_get_block(jit, &cpu, source);
    jit_block_fn source_fn = jit_get_block(jit, &cpu, source);
    ASSERT_TRUE(source_fn != NULL);

    /* The source reaches the timer exactly at its static successor. The
     * successor must not execute before xtensa_step_impl() fires the event. */
    cpu.pc = source;
    cpu.ccount = 100u;
    cpu.next_timer_event = 104u;
    cpu.jit_chain_limit = 4u; /* dispatcher-computed event horizon */
    ASSERT_EQ(source_fn(&cpu), 4);
    ASSERT_EQ(cpu.pc, target);

    jit_destroy(jit);
    teardown(&cpu);
}

TEST(test_jit_precompiled_side_exit_chain_is_not_overwritten) {
    xtensa_cpu_t cpu;
    setup(&cpu);
    const uint32_t target = BASE;
    const uint32_t source = BASE + 0x40u;
    const uint32_t branch_pc = source + 6u;
    const uint32_t return_pc = BASE + 0x100u;

    put_insn2(&cpu, target, narrow(0xD, 15, 0, 0)); /* RET.N */
    put_insn2(&cpu, source,      narrow(0xD, 15, 0, 3));
    put_insn2(&cpu, source + 2u, narrow(0xD, 15, 0, 3));
    put_insn2(&cpu, source + 4u, narrow(0xD, 15, 0, 3));
    int32_t boff = (int32_t)target - (int32_t)(branch_pc + 3u) - 1;
    put_insn3(&cpu, branch_pc,
              (((uint32_t)boff & 0xFFFu) << 12) |
              (3u << 8) | (1u << 4) | 6u); /* BEQZ a3, target */
    put_insn2(&cpu, branch_pc + 3u, narrow(0xD, 15, 0, 2)); /* ILL.N */
    ar_write(&cpu, 0, return_pc);
    ar_write(&cpu, 3, 0u);

    jit_state_t *jit = jit_init();
    ASSERT_TRUE(jit != NULL);
    for (int i = 0; i < JIT_HOT_THRESHOLD; i++)
        (void)jit_get_block(jit, &cpu, target);
    ASSERT_TRUE(jit_get_block(jit, &cpu, target) != NULL);
    for (int i = 0; i < JIT_HOT_THRESHOLD; i++)
        (void)jit_get_block(jit, &cpu, source);
    jit_block_fn source_fn = jit_get_block(jit, &cpu, source);
    ASSERT_TRUE(source_fn != NULL);
    ASSERT_EQ(jit_get_stats(jit)->chains_patched, 1u);

    cpu.pc = source;
    ASSERT_EQ(source_fn(&cpu), 5);
    ASSERT_EQ(cpu.pc, return_pc);

    jit_destroy(jit);
    teardown(&cpu);
}

TEST(test_jit_short_backedge_loop_is_native) {
    xtensa_cpu_t cpu;
    setup(&cpu);

    /* The production ESP-IDF IPC wait loop has this shape: one load and a
     * backward BEQZ.  Its fallthrough RET makes the trace only three guest
     * instructions long, but the taken path can self-chain very profitably. */
    put_insn2(&cpu, BASE, narrow(0x8, 0, 2, 3)); /* L32I.N a3, a2, 0 */
    uint32_t imm12 = (uint32_t)-6 & 0xFFFu;
    put_insn3(&cpu, BASE + 2,
              (imm12 << 12) | (3u << 8) | (1u << 4) | 6u); /* BEQZ a3, BASE */
    put_insn2(&cpu, BASE + 5, narrow(0xD, 15, 0, 0));       /* RET.N */
    ar_write(&cpu, 2, BASE + 0x1000);
    mem_write32(cpu.mem, BASE + 0x1000, 0);

    jit_state_t *jit = jit_init();
    ASSERT_TRUE(jit != NULL);
    jit_install_hook(jit, &cpu);
    for (int i = 0; i < JIT_HOT_THRESHOLD; i++)
        (void)jit_get_block(jit, &cpu, BASE);
    ASSERT_TRUE(jit_get_block(jit, &cpu, BASE) != NULL);

    cpu.running = true;
    cpu._pc_written = true;
    int ran = xtensa_run(&cpu, 800);
    const jit_stats_t *stats = jit_get_stats(jit);
    ASSERT_TRUE(ran >= 800);
    ASSERT_TRUE(stats->blocks_executed >= 1);
    ASSERT_TRUE(stats->insns_jitted >= 400);
    ASSERT_EQ(cpu.pc, BASE);

    jit_destroy(jit);
    teardown(&cpu);
}

TEST(test_jit_stale_loop_past_lend_does_not_truncate_block) {
    xtensa_cpu_t cpu;
    setup(&cpu);

    /* FreeRTOS context frames may restore a nonzero LCOUNT after execution
     * has already moved beyond that loop. The cache key treats this PC as an
     * ordinary block; the scanner must make the identical range check rather
     * than truncating it after one instruction at the stale LEND. */
    put_insn2(&cpu, BASE,     narrow(0xD, 15, 0, 3)); /* NOP.N */
    put_insn2(&cpu, BASE + 2, narrow(0xD, 15, 0, 3)); /* NOP.N */
    put_insn2(&cpu, BASE + 4, narrow(0xD, 15, 0, 3)); /* NOP.N */
    put_insn2(&cpu, BASE + 6, narrow(0xD, 15, 0, 3)); /* NOP.N */
    cpu.lbeg = BASE - 0x100;
    cpu.lend = BASE - 0x20;
    cpu.lcount = UINT32_MAX;

    jit_state_t *jit = jit_init();
    ASSERT_TRUE(jit != NULL);
    for (int i = 0; i < JIT_HOT_THRESHOLD; i++)
        (void)jit_get_block(jit, &cpu, BASE);
    jit_block_fn fn = jit_get_block(jit, &cpu, BASE);
    ASSERT_TRUE(fn != NULL);
    if (fn) {
        int ran = fn(&cpu);
        ASSERT_EQ(ran, 4);
        ASSERT_EQ(cpu.pc, BASE + 8);
    }

    jit_destroy(jit);
    teardown(&cpu);
}

TEST(test_jit_contended_spinlock_returns_to_scheduler) {
    xtensa_cpu_t cpu;
    setup(&cpu);

    /* Four-instruction CAS retry loop, matching ESP-IDF's spinlock shape.
     * The deterministic scheduler cannot run the lock-owning core until this
     * core returns, so native execution must yield at the failed CAS itself. */
    put_insn3(&cpu, BASE, 0x00E432u); /* S32C1I a3, a4, 0 */
    put_insn2(&cpu, BASE + 3, narrow(0xD, 15, 0, 3)); /* NOP.N */
    put_insn2(&cpu, BASE + 5, narrow(0xD, 15, 0, 3)); /* NOP.N */
    uint32_t off18 = (uint32_t)-11 & 0x3FFFFu;
    put_insn3(&cpu, BASE + 7, (off18 << 6) | 6u); /* J BASE */
    ar_write(&cpu, 3, XTENSA_SPINLOCK_OWNER_CORE0);
    ar_write(&cpu, 4, DATA_BASE);
    cpu.scompare1 = XTENSA_SPINLOCK_FREE;
    mem_write32(cpu.mem, DATA_BASE, XTENSA_SPINLOCK_OWNER_CORE1);

    jit_state_t *jit = jit_init();
    ASSERT_TRUE(jit != NULL);
    jit_install_hook(jit, &cpu);
    for (int i = 0; i < JIT_HOT_THRESHOLD; i++)
        (void)jit_get_block(jit, &cpu, BASE);
    ASSERT_TRUE(jit_get_block(jit, &cpu, BASE) != NULL);

    cpu.running = true;
    cpu._pc_written = true;
    const int budget = 5000;
    int ran = jit_run(jit, &cpu, budget);
    ASSERT_EQ(ran, 1);
    ASSERT_EQ(cpu.pc, BASE + 3u);
    ASSERT_TRUE(cpu.core_handoff);
    ASSERT_EQ(mem_read32(cpu.mem, DATA_BASE), XTENSA_SPINLOCK_OWNER_CORE1);
    ASSERT_TRUE(jit_get_stats(jit)->insns_jitted > 0);

    jit_destroy(jit);
    teardown(&cpu);
}

/* A chained JIT run must report every block it executed, not just the last
 * one, and must stay bounded by JIT_CHAIN_CAP.  The guest-insn accumulator
 * is the sole driver of ccount/cycle_count in JIT mode (jit_pc_hook adds the
 * returned count), so under-reporting stalls virtual time and starves timers,
 * FreeRTOS preemption and the -c budget; a cap check that never fires hangs
 * outright on any self-chaining loop. */
TEST(test_jit_chained_run_accounts_every_block) {
    xtensa_cpu_t cpu;
    setup(&cpu);

    /* Four-instruction self-chaining loop. a4 counts iterations independently
     * of the JIT's own accounting; a3 stays zero so the backward BEQZ is
     * always taken and the block chains to itself. */
    put_insn2(&cpu, BASE,     narrow(0xB, 4, 4, 1));  /* ADDI.N a4, a4, 1 */
    put_insn2(&cpu, BASE + 2, narrow(0xD, 15, 0, 3)); /* NOP.N */
    put_insn2(&cpu, BASE + 4, narrow(0xD, 15, 0, 3)); /* NOP.N */
    uint32_t imm12 = (uint32_t)-10 & 0xFFFu;
    put_insn3(&cpu, BASE + 6,
              (imm12 << 12) | (3u << 8) | (1u << 4) | 6u); /* BEQZ a3, BASE */
    ar_write(&cpu, 3, 0);
    ar_write(&cpu, 4, 0);

    jit_state_t *jit = jit_init();
    ASSERT_TRUE(jit != NULL);
    jit_install_hook(jit, &cpu);
    for (int i = 0; i < JIT_HOT_THRESHOLD; i++)
        (void)jit_get_block(jit, &cpu, BASE);
    ASSERT_TRUE(jit_get_block(jit, &cpu, BASE) != NULL);

    cpu.running = true;
    cpu._pc_written = true;
    const int budget = 4000;
    uint64_t cycles_before = cpu.cycle_count;
    int ran = xtensa_run(&cpu, budget);
    uint64_t cycles_after = cpu.cycle_count;

    /* Each iteration is exactly four guest instructions. */
    uint32_t iterations = ar_read(&cpu, 4);
    ASSERT_TRUE(iterations > 0);
    ASSERT_EQ((uint32_t)ran, iterations * 4);

    /* Virtual time must advance by exactly what the run reports. */
    ASSERT_EQ64(cycles_after - cycles_before, (uint64_t)ran);

    /* The cap must bound the run: an infinite guest loop whose chain never
     * re-checks the budget never returns here. Overshoot is limited to the
     * final block, which the cap can only observe at its exit. */
    ASSERT_TRUE(ran >= budget);
    ASSERT_TRUE(ran <= budget + JIT_CHAIN_CAP + JIT_MAX_BLOCK_INSNS);

    /* Guard against the case degenerating into interpreted stepping, which
     * would not exercise chained accounting at all: one dispatch has to
     * cover many iterations. */
    const jit_stats_t *stats = jit_get_stats(jit);
    ASSERT_TRUE(stats->insns_jitted >= (uint64_t)budget);
    ASSERT_TRUE(stats->blocks_executed < iterations);

    jit_destroy(jit);
    teardown(&cpu);
}

/* A zero-overhead loop's back-edge is a taken branch and must dispatch like
 * one.  The interpreter rewrites PC to LBEG without executing a branch, so if
 * that path does not mark the PC as written, pc_hook dispatch is skipped and a
 * block compiled at LBEG can never be entered — every compiler-emitted LOOP
 * body then runs interpreted no matter how hot it gets. */
TEST(test_jit_loop_backedge_dispatches_native_body) {
    xtensa_cpu_t cpu;
    setup(&cpu);

    /* LOOP a3, lend ; body ; (lend) — op0=6, t=7 (n=3,m=1), r=8 selects LOOP. */
    const uint32_t lbeg = BASE + 3u;
    const uint32_t lend = lbeg + 10u;
    put_insn3(&cpu, BASE, (uint32_t)0x76u | ((uint32_t)((8 << 4) | 3) << 8) |
                          ((lend - (BASE + 4u)) << 16));
    put_insn2(&cpu, lbeg,      narrow(0xB, 4, 4, 1));  /* ADDI.N a4, a4, 1 */
    put_insn2(&cpu, lbeg + 2u, narrow(0xD, 15, 0, 3)); /* NOP.N */
    put_insn2(&cpu, lbeg + 4u, narrow(0xD, 15, 0, 3)); /* NOP.N */
    put_insn2(&cpu, lbeg + 6u, narrow(0xD, 15, 0, 3)); /* NOP.N */
    put_insn2(&cpu, lbeg + 8u, narrow(0xD, 15, 0, 3)); /* NOP.N */
    put_insn2(&cpu, lend,      narrow(0xD, 15, 0, 0)); /* RET.N */
    ar_write(&cpu, 3, 200);
    ar_write(&cpu, 4, 0);

    jit_state_t *jit = jit_init();
    ASSERT_TRUE(jit != NULL);
    jit_install_hook(jit, &cpu);

    /* Execute LOOP itself so LBEG/LEND/LCOUNT are live; the scanner needs
     * LEND to bound the body block. */
    cpu.running = true;
    cpu._pc_written = true;
    xtensa_step(&cpu);
    ASSERT_EQ(cpu.pc, lbeg);
    ASSERT_EQ(cpu.lbeg, lbeg);
    ASSERT_EQ(cpu.lend, lend);
    ASSERT_TRUE(cpu.lcount > 0);

    /* Interpret a few iterations first, so the run below resumes mid-body and
     * the next arrival at LBEG comes from the interpreter's back-edge rather
     * than from a JIT dispatch. */
    for (int i = 0; i < 7; i++)
        xtensa_step(&cpu);
    ASSERT_TRUE(cpu.pc > lbeg && cpu.pc <= lend);

    for (int i = 0; i < JIT_HOT_THRESHOLD; i++)
        (void)jit_get_block(jit, &cpu, lbeg);
    ASSERT_TRUE(jit_get_block(jit, &cpu, lbeg) != NULL);

    /* No pending branch: only the loop back-edge can hand control to the
     * compiled body from here. */
    cpu._pc_written = false;
    uint32_t a4_before = ar_read(&cpu, 4);
    uint32_t lcount_before = cpu.lcount;
    (void)xtensa_run(&cpu, 400);

    const jit_stats_t *stats = jit_get_stats(jit);
    ASSERT_TRUE(stats->insns_jitted > 100);

    /* The body counts iterations in a4, so the guest-visible effect must
     * match the number of back-edges actually taken. A run that stops with
     * the PC sitting on LBEG has taken the edge but not yet re-entered the
     * body, which is one increment short. */
    uint32_t taken = lcount_before - cpu.lcount;
    uint32_t pending = cpu.pc == cpu.lbeg ? 1u : 0u;
    ASSERT_EQ(ar_read(&cpu, 4) - a4_before, taken - pending);

    /* The back-edge is closed in native code, so one dispatch covers many
     * iterations. Requiring a high insns-per-dispatch ratio is what
     * distinguishes native self-looping from returning to jit_pc_hook once
     * per iteration -- the whole point of compiling the edge. */
    ASSERT_TRUE(stats->blocks_executed > 0);
    ASSERT_TRUE(stats->insns_jitted > stats->blocks_executed * 20);

    jit_destroy(jit);
    teardown(&cpu);
}

/* A cold LOOP setup must not strand its first body iteration in the
 * interpreter. A one-iteration loop has no back-edge at all, making it an
 * exact regression test for the private fallthrough dispatch boundary. */
TEST(test_jit_loop_fallthrough_dispatches_first_body) {
    xtensa_cpu_t cpu;
    setup(&cpu);

    const uint32_t lbeg = BASE + 3u;
    const uint32_t lend = lbeg + 2u;
    const uint32_t done = lend + 7u;
    put_insn3(&cpu, BASE, (uint32_t)0x76u | ((uint32_t)((8 << 4) | 3) << 8) |
                          ((lend - (BASE + 4u)) << 16));
    put_insn2(&cpu, lbeg, narrow(0xB, 4, 4, 1));  /* ADDI.N a4, a4, 1 */
    /* J done: target = instruction PC + sign_extend(offset18) + 4. */
    uint32_t jump_offset = (done - (lend + 4u)) & 0x3FFFFu;
    put_insn3(&cpu, lend, (jump_offset << 6) | 6u);
    put_insn2(&cpu, done, narrow(0xD, 15, 0, 3)); /* NOP.N */
    ar_write(&cpu, 3, 1);  /* exactly one loop iteration */
    ar_write(&cpu, 4, 0);

    jit_state_t *jit = jit_init();
    ASSERT_TRUE(jit != NULL);
    jit_install_hook(jit, &cpu);

    cpu.running = true;
    cpu._pc_written = true;
    xtensa_step(&cpu);  /* interpreted LOOP */
    ASSERT_EQ(cpu.pc, lbeg);
    ASSERT_EQ(cpu.lcount, 0u);
    ASSERT_TRUE(cpu.jit_fallthrough_dispatch);

    /* The one-iteration form has LCOUNT=0, so compile the same non-loop
     * variant jit_pc_hook will look up when it consumes the marker. */
    for (int i = 0; i < JIT_HOT_THRESHOLD; i++)
        (void)jit_get_block(jit, &cpu, lbeg);
    ASSERT_TRUE(jit_get_block(jit, &cpu, lbeg) != NULL);

    uint64_t jitted_before = jit_get_stats(jit)->insns_jitted;
    ASSERT_EQ(xtensa_run(&cpu, 2), 2);
    ASSERT_EQ(ar_read(&cpu, 4), 1u);
    ASSERT_EQ(cpu.pc, done);
    ASSERT_EQ64(jit_get_stats(jit)->insns_jitted - jitted_before, 2u);
    ASSERT_FALSE(cpu.jit_fallthrough_dispatch);

    jit_destroy(jit);
    teardown(&cpu);
}

static uint32_t jit_loop_setup_insn(unsigned kind, unsigned source,
                                    uint32_t pc, uint32_t lend)
{
    uint32_t offset = lend - (pc + 4u);
    return 0x76u | (((kind << 4) | source) << 8) | (offset << 16);
}

TEST(test_jit_compiles_loop_setup_family) {
    const uint32_t lend = BASE + 0x23u;
    struct {
        unsigned kind;
        uint32_t count;
        uint32_t initial_lcount;
        uint32_t expected_pc;
        uint32_t expected_lcount;
        const char *name;
    } cases[] = {
        { 8, 4u,          99u, BASE + 3u, 3u, "loop" },
        { 9, 1u,          99u, BASE + 3u, 0u, "loopnez_enter" },
        { 9, 0u,          99u, lend,       99u, "loopnez_skip" },
        { 10, 2u,         99u, BASE + 3u, 1u, "loopgtz_enter" },
        { 10, 0u,         99u, lend,       99u, "loopgtz_zero" },
        { 10, UINT32_MAX, 99u, lend,       99u, "loopgtz_negative" },
    };

    for (size_t i = 0; i < sizeof(cases) / sizeof(cases[0]); i++) {
        xtensa_cpu_t cpu;
        setup(&cpu);
        put_insn3(&cpu, BASE,
                  jit_loop_setup_insn(cases[i].kind, 3u, BASE, lend));
        ar_write(&cpu, 3, cases[i].count);
        cpu.lcount = cases[i].initial_lcount;
        test_run_differential(&cpu, 1, cases[i].name);
        cpu.running = true;
        ASSERT_EQ(xtensa_step(&cpu), 0);
        ASSERT_EQ(cpu.pc, cases[i].expected_pc);
        ASSERT_EQ(cpu.lbeg, BASE + 3u);
        ASSERT_EQ(cpu.lend, lend);
        ASSERT_EQ(cpu.lcount, cases[i].expected_lcount);
        teardown(&cpu);
    }
}

TEST(test_jit_loopnez_uses_dirty_prefix_value) {
    xtensa_cpu_t cpu;
    setup(&cpu);
    const uint32_t loop_pc = BASE + 2u;
    const uint32_t lend = BASE + 0x25u;
    put_insn2(&cpu, BASE, narrow(0xB, 3, 3, 1)); /* ADDI.N a3,a3,1 */
    put_insn3(&cpu, loop_pc,
              jit_loop_setup_insn(9u, 3u, loop_pc, lend));
    ar_write(&cpu, 3, UINT32_MAX); /* prefix produces zero, so skip */
    cpu.lcount = 0xA5A5u;
    test_run_differential(&cpu, 2, "loopnez_dirty_prefix_skip");
    teardown(&cpu);
}

TEST(test_jit_loop_setup_clears_architectural_edge) {
    xtensa_cpu_t cpu;
    setup(&cpu);
    const uint32_t lend = BASE + 0x23u;
    put_insn3(&cpu, BASE, jit_loop_setup_insn(8u, 3u, BASE, lend));
    ar_write(&cpu, 3, 2u);

    jit_state_t *jit = jit_init();
    ASSERT_TRUE(jit != NULL);
    if (!jit) {
        teardown(&cpu);
        return;
    }
    jit_install_hook(jit, &cpu);
    for (int i = 0; i < JIT_HOT_THRESHOLD; i++)
        (void)jit_get_block(jit, &cpu, BASE);
    ASSERT_TRUE(jit_get_block(jit, &cpu, BASE) != NULL);

    cpu.running = true;
    cpu._pc_written = true;
    ASSERT_EQ(xtensa_step(&cpu), 0);
    ASSERT_EQ(cpu.pc, BASE + 3u);
    ASSERT_FALSE(cpu._pc_written);
    ASSERT_TRUE(cpu.jit_fallthrough_dispatch);

    jit_destroy(jit);
    teardown(&cpu);
}

static uint32_t jit_movsp_insn(unsigned destination, unsigned source) {
    return rrr(0, 0, 1, (int)source, (int)destination);
}

TEST(test_jit_movsp_architectural_move) {
    xtensa_cpu_t cpu;
    setup(&cpu);
    cpu.real_window_vectors = true;
    cpu.ps = 1u << 18; /* WOE, outside an exception */
    cpu.windowbase = 4u;
    cpu.windowstart = (1u << 2) | (1u << 4); /* call8 caller is live */
    ar_write(&cpu, 2, DATA_BASE + 0x800u);
    put_insn3(&cpu, BASE, jit_movsp_insn(1u, 2u));
    put_insn2(&cpu, BASE + 3u, narrow(0xD, 15, 0, 3));
    test_block_differential(&cpu, 2, "movsp_architectural_move");
    teardown(&cpu);
}

TEST(test_jit_movsp_alloca_falls_back_at_opcode) {
    xtensa_cpu_t expected, actual;
    setup(&expected);
    expected.real_window_vectors = true;
    expected.vecbase = BASE;
    expected.ps = (1u << 18) | (1u << 5); /* WOE + user mode */
    expected.windowbase = 4u;
    expected.windowstart = 1u << 4; /* every possible caller is spilled */
    ar_write(&expected, 1, DATA_BASE + 0x900u);
    ar_write(&expected, 2, DATA_BASE + 0xA00u);
    put_insn3(&expected, BASE, jit_movsp_insn(1u, 2u));
    memcpy(&actual, &expected, sizeof(actual));

    ASSERT_EQ(xtensa_step(&expected), 0);

    jit_state_t *jit = jit_init();
    ASSERT_TRUE(jit != NULL);
    if (!jit) {
        teardown(&expected);
        return;
    }
    jit_install_hook(jit, &actual);
    for (int i = 0; i < JIT_HOT_THRESHOLD; i++)
        (void)jit_get_block(jit, &actual, BASE);
    ASSERT_TRUE(jit_get_block(jit, &actual, BASE) != NULL);
    actual._pc_written = true;
    ASSERT_EQ(xtensa_step(&actual), 0);

    ASSERT_EQ(compare_state(&expected, &actual, "movsp_alloca_fallback"), 0);
    ASSERT_EQ(actual.exccause, EXCCAUSE_ALLOCA);
    ASSERT_EQ(actual.epc[0], BASE);
    ASSERT_EQ(ar_read(&actual, 1), DATA_BASE + 0x900u);

    jit_destroy(jit);
    teardown(&expected);
}

TEST(test_jit_movsp_legacy_mode_falls_back) {
    xtensa_cpu_t cpu;
    setup(&cpu);
    cpu.real_window_vectors = false;
    cpu.ps = (1u << 18) | (1u << 4);
    ar_write(&cpu, 1, DATA_BASE + 0xB00u);
    ar_write(&cpu, 2, DATA_BASE + 0xB00u);
    put_insn3(&cpu, BASE, jit_movsp_insn(1u, 2u));
    test_run_differential(&cpu, 1, "movsp_legacy_fallback");
    teardown(&cpu);
}

/* The same flush obligation, but with the branch *before* the registers it
 * has to write back -- which is the case the obvious test misses.
 *
 * A side exit's flush used to be emitted inline at the branch, covering the
 * registers dirty at that point in the block. In a block that keeps its
 * registers resident across the loop back-edge that is not enough: a register
 * written further down the body is already dirty from the *previous*
 * iteration and lives only in a host register, so an exit taken before it is
 * written in program order still has to write it back. Leaving it out loses
 * every iteration's work but the first, silently -- the loop still terminates
 * and the exit still goes to the right place.
 *
 * This is what a real Marauder block did: a counted search loop whose early
 * out came first and whose shift-and-count came after. */
TEST(test_jit_self_loop_early_exit_flushes_later_writes) {
    xtensa_cpu_t cpu;
    setup(&cpu);

    /* LOOP a3, lend
     *   lbeg: ADDI.N a9, a9, -1     countdown, dirty before the branch
     *         BEQZ.N a9, out        the early out
     *         ADDI.N a10, a10, 1    written only after it
     *         NOP.N ; NOP.N
     *   lend: NOP.N
     *   out:  RET.N                                                     */
    const uint32_t loop_pc = BASE + 2u;
    const uint32_t lbeg = loop_pc + 3u;
    const uint32_t lend = lbeg + 10u;
    const uint32_t out  = lbeg + 12u;

    put_insn2(&cpu, BASE, narrow(0xD, 15, 0, 3));
    put_insn3(&cpu, loop_pc, (uint32_t)0x76u |
              ((uint32_t)((8 << 4) | 3) << 8) |
              ((lend - (loop_pc + 4u)) << 16));
    put_insn2(&cpu, lbeg,      narrow(0xB, 9, 9, 0));    /* ADDI.N a9,a9,-1 */
    put_insn2(&cpu, lbeg + 2u, narrow(0xC, 6, 9, 8));    /* BEQZ.N a9, out  */
    put_insn2(&cpu, lbeg + 4u, narrow(0xB, 10, 10, 1));  /* ADDI.N a10,a10,1 */
    put_insn2(&cpu, lbeg + 6u, narrow(0xD, 15, 0, 3));
    put_insn2(&cpu, lbeg + 8u, narrow(0xD, 15, 0, 3));
    put_insn2(&cpu, lend,      narrow(0xD, 15, 0, 3));
    put_insn2(&cpu, out,       narrow(0xD, 15, 0, 0));   /* RET.N */

    ar_write(&cpu, 3, 500);
    ar_write(&cpu, 9, 5);
    ar_write(&cpu, 10, 0);

    jit_state_t *jit = jit_init();
    ASSERT_TRUE(jit != NULL);
    jit_install_hook(jit, &cpu);

    cpu.pc = loop_pc;
    cpu.running = true;
    cpu._pc_written = true;
    xtensa_step(&cpu);                       /* execute LOOP */
    ASSERT_EQ(cpu.lbeg, lbeg);
    for (int i = 0; i <= JIT_HOT_THRESHOLD; i++)
        (void)jit_get_block(jit, &cpu, lbeg);
    ASSERT_TRUE(jit_get_block(jit, &cpu, lbeg) != NULL);

    /* Restart the countdown and let the block run it to the early out. */
    ar_write(&cpu, 9, 5);
    ar_write(&cpu, 10, 0);
    cpu.pc = lbeg;
    cpu.lcount = 400;
    cpu._pc_written = true;
    (void)xtensa_run(&cpu, 400);

    /* Four iterations ran the increment, the fifth took the early out. */
    ASSERT_EQ(ar_read(&cpu, 9), 0u);
    ASSERT_EQ(ar_read(&cpu, 10), 4u);
    ASSERT_EQ(cpu.ar[(cpu.windowbase * 4 + 10) & 63], 4u);

    jit_destroy(jit);
    teardown(&cpu);
}

/* CALLn writes its return address straight into ar[], bypassing the register
 * allocator. The slot is guest a0, a4, a8 or a12 -- n is encoded 0-3, so it is
 * inside the current window, not past a15 -- which means it can be a register
 * the allocator is holding a modified copy of. The flush at the block exit
 * then puts that stale copy back over the return address, and the callee
 * returns into whatever the guest last had in a8.
 *
 * This is what a windowed CALL8 does in real firmware, and it is the calling
 * convention Arduino and IDF code is built with.
 */
TEST(test_jit_call8_return_address_survives_the_exit_flush) {
    xtensa_cpu_t cpu;
    setup(&cpu);

    const uint32_t call_pc = BASE + 6u;
    const uint32_t next_pc = call_pc + 3u;
    const uint32_t target  = BASE + 0x40u;
    /* CALL8: op0=5, n=2 (bits 5:4), offset18 in bits 23:6. */
    const uint32_t off18 = ((target >> 2) - (call_pc >> 2) - 1u) & 0x3FFFFu;

    put_insn2(&cpu, BASE,      narrow(0xB, 8, 8, 1));   /* ADDI.N a8, a8, 1 */
    put_insn2(&cpu, BASE + 2u, narrow(0xD, 15, 0, 3));  /* NOP.N */
    put_insn2(&cpu, BASE + 4u, narrow(0xD, 15, 0, 3));  /* NOP.N */
    put_insn3(&cpu, call_pc, 5u | (2u << 4) | (off18 << 6));
    put_insn2(&cpu, target,    narrow(0xD, 15, 0, 0));  /* RET.N */

    ar_write(&cpu, 8, 0x3FFC6034u);   /* a8 holds a pointer, as it would */

    jit_state_t *jit = jit_init();
    ASSERT_TRUE(jit != NULL);
    jit_install_hook(jit, &cpu);

    cpu.pc = BASE;
    cpu.running = true;
    for (int i = 0; i <= JIT_HOT_THRESHOLD; i++) {
        cpu.pc = BASE;
        (void)jit_get_block(jit, &cpu, BASE);
    }
    jit_block_fn fn = jit_get_block(jit, &cpu, BASE);
    ASSERT_TRUE(fn != NULL);

    cpu.pc = BASE;
    cpu._pc_written = true;
    ar_write(&cpu, 8, 0x3FFC6034u);
    (void)xtensa_run(&cpu, 4);

    /* The block modified a8 and then called through it. What has to be in a8
     * is the return address the CALL wrote, not the value the allocator was
     * carrying. */
    uint32_t want = 0x80000000u | (next_pc & 0x3FFFFFFFu);
    ASSERT_EQ(ar_read(&cpu, 8), want);
    ASSERT_EQ(cpu.pc, target);

    jit_destroy(jit);
    teardown(&cpu);
}

/* A live window beyond the highest register a block touches is irrelevant.
 * The old all-three-windows guard returned every such block to the
 * interpreter even though the architectural operand check would let it run. */
TEST(test_jit_window_guard_checks_only_touched_windows) {
    xtensa_cpu_t cpu;
    setup(&cpu);
    cpu.windowbase = 1;
    cpu.windowstart = 1u << 1;
    ar_write(&cpu, 4, 10u);
    for (unsigned i = 0; i < 4; i++)
        put_insn2(&cpu, BASE + i * 2u, narrow(0xB, 4, 4, 1));

    jit_state_t *jit = jit_init();
    ASSERT_TRUE(jit != NULL);
    jit_install_hook(jit, &cpu);
    for (int i = 0; i <= JIT_HOT_THRESHOLD; i++)
        (void)jit_get_block(jit, &cpu, BASE);
    jit_block_fn fn = jit_get_block(jit, &cpu, BASE);
    ASSERT_TRUE(fn != NULL);

    cpu.real_window_vectors = true;
    cpu.ps = 1u << 18; /* WOE, not EXCM */
    /* a4 reaches only WB+1 (bit 2); WB+2 (bit 3) may remain live. */
    cpu.windowstart = (1u << 1) | (1u << 3);
    cpu.pc = BASE;
    ASSERT_EQ(fn(&cpu), 4);
    ASSERT_EQ(cpu.pc, BASE + 8u);
    ASSERT_EQ(ar_read(&cpu, 4), 14u);

    jit_destroy(jit);
    teardown(&cpu);
}

TEST(test_jit_head_window_collision_raises_guest_vector) {
    xtensa_cpu_t expected, actual;
    setup(&expected);
    expected.real_window_vectors = true;
    expected.vecbase = BASE;
    expected.ps = 1u << 18; /* WOE, outside an exception */
    expected.windowbase = 1u;
    /* MOV.N a9,a3 reaches WB+2. The nearest collision is WB+1, whose
     * following live window selects the Overflow8 vector. */
    expected.windowstart = (1u << 1) | (1u << 2) | (1u << 4);
    ar_write(&expected, 3, 0x12345678u);
    put_insn2(&expected, BASE, narrow(0xD, 0, 3, 9));
    memcpy(&actual, &expected, sizeof(actual));

    ASSERT_EQ(xtensa_step(&expected), 0);

    jit_state_t *jit = jit_init();
    ASSERT_TRUE(jit != NULL);
    if (!jit) {
        teardown(&expected);
        return;
    }
    for (int i = 0; i < JIT_HOT_THRESHOLD; i++)
        (void)jit_get_block(jit, &actual, BASE);
    jit_block_fn fn = jit_get_block(jit, &actual, BASE);
    ASSERT_TRUE(fn != NULL);
    ASSERT_EQ(fn(&actual), 1);

    ASSERT_EQ(compare_state(&expected, &actual,
                            "head_window_overflow_vector"), 0);
    ASSERT_EQ(actual.pc, BASE + VECOFS_WINDOW_OVERFLOW8);
    ASSERT_EQ(actual.epc[0], BASE);
    ASSERT_EQ(actual.windowbase, 2u);
    ASSERT_EQ(XT_PS_OWB(actual.ps), 1u);
    ASSERT_TRUE(XT_PS_EXCM(actual.ps));

    jit_destroy(jit);
    teardown(&expected);
}

/* WSR PS can enable window exceptions in the middle of a block. When a later
 * high-register operand collides, the entry guard must not rely only on the
 * old PS value; it returns to the interpreter so the precise access faults. */
TEST(test_jit_window_guard_handles_mid_block_woe_enable) {
    xtensa_cpu_t cpu;
    setup(&cpu);
    cpu.windowbase = 1;
    cpu.windowstart = 1u << 1;
    ar_write(&cpu, 2, 1u << 18); /* new PS: WOE, not EXCM */
    put_insn3(&cpu, BASE,
              rrr(1, 3, XT_SR_PS >> 4, XT_SR_PS & 15, 2));
    put_insn2(&cpu, BASE + 3u, narrow(0xB, 4, 4, 1));
    put_insn2(&cpu, BASE + 5u, narrow(0xB, 4, 4, 1));
    put_insn2(&cpu, BASE + 7u, narrow(0xB, 4, 4, 1));

    jit_state_t *jit = jit_init();
    ASSERT_TRUE(jit != NULL);
    jit_install_hook(jit, &cpu);
    for (int i = 0; i <= JIT_HOT_THRESHOLD; i++)
        (void)jit_get_block(jit, &cpu, BASE);
    jit_block_fn fn = jit_get_block(jit, &cpu, BASE);
    ASSERT_TRUE(fn != NULL);

    cpu.real_window_vectors = true;
    cpu.vecbase = BASE;
    cpu.ps = 0; /* WOE becomes enabled by the first instruction. */
    cpu.windowstart = (1u << 1) | (1u << 2) | (1u << 3);
    cpu.pc = BASE;
    ASSERT_EQ(fn(&cpu), 0);
    ASSERT_EQ(cpu.ps, 0u);

    cpu._pc_written = true;
    cpu.running = true;
    (void)xtensa_step(&cpu); /* WSR PS */
    (void)xtensa_step(&cpu); /* colliding ADDI.N a4 */
    ASSERT_EQ(cpu.epc[0], BASE + 3u);
    ASSERT_EQ(cpu.pc, BASE + VECOFS_WINDOW_OVERFLOW4);
    ASSERT_EQ(cpu.windowbase, 2u);

    jit_destroy(jit);
    teardown(&cpu);
}

/* A native CALLX8 must not write its return address into a live aliased
 * window.  Native code cannot take WindowOverflow directly, so the block
 * guard hands the collision to xtensa_step(), which faults before CALLX8 has
 * any side effects. */
TEST(test_jit_window_collision_falls_back_before_callx8) {
    xtensa_cpu_t cpu;
    setup(&cpu);

    const uint32_t call_pc = BASE + 6u;
    const uint32_t target = BASE + 0x200u;
    /* CALLX8 a2: op0/op1/op2/r=0, m=3, n=2, s=2. */
    const uint32_t callx8 = (2u << 8) | (14u << 4);
    put_insn2(&cpu, BASE,      narrow(0xD, 15, 0, 3));
    put_insn2(&cpu, BASE + 2u, narrow(0xD, 15, 0, 3));
    put_insn2(&cpu, BASE + 4u, narrow(0xD, 15, 0, 3));
    put_insn3(&cpu, call_pc, callx8);

    cpu.windowbase = 1;
    cpu.windowstart = 1u << 1;
    ar_write(&cpu, 2, target);

    jit_state_t *jit = jit_init();
    ASSERT_TRUE(jit != NULL);
    jit_install_hook(jit, &cpu);
    for (int i = 0; i <= JIT_HOT_THRESHOLD; i++)
        (void)jit_get_block(jit, &cpu, BASE);
    ASSERT_TRUE(jit_get_block(jit, &cpu, BASE) != NULL);

    cpu.real_window_vectors = true;
    cpu.vecbase = BASE;
    cpu.ps = 1u << 18; /* WOE, not EXCM */
    cpu.windowstart = (1u << 1) | (1u << 3) | (1u << 5);
    cpu.ar[3 * 4] = 0xDEADBEEFu;
    cpu.pc = BASE;
    cpu._pc_written = true;
    cpu.running = true;

    for (int i = 0; i < 4; i++)
        (void)xtensa_step(&cpu);

    ASSERT_EQ(cpu.pc, BASE + VECOFS_WINDOW_OVERFLOW8);
    ASSERT_EQ(cpu.epc[0], call_pc);
    ASSERT_EQ(cpu.windowbase, 3u);
    ASSERT_EQ(cpu.ar[3 * 4], 0xDEADBEEFu);

    jit_destroy(jit);
    teardown(&cpu);
}

/* The native back-edge no longer re-reads LBEG/LEND every iteration when the
 * body provably cannot write them, so the check that the live loop is the one
 * the block was compiled for happens once, on entry. That check is what stops
 * a block reached under a *different* loop from running its back-edge against
 * the wrong bounds -- ending the body early, every iteration, and silently
 * skipping the instructions between the two LENDs.
 *
 * Two loops share a head here and differ only in where they end. The inner
 * one's body updates a4; the outer one's body updates a4 and a5. Entering the
 * inner block with the outer loop live has to hand back to the interpreter,
 * which is visible as the two counters staying equal. */
TEST(test_jit_self_loop_block_declines_a_different_loop) {
    xtensa_cpu_t cpu;
    setup(&cpu);

    /* LOOP sits at BASE+2 so LBEG (+3) lands in the next 4-byte word: the
     * block hash indexes on pc >> 2, and a loop head sharing a word with its
     * own LBEG evicts it. */
    const uint32_t loop_pc = BASE + 2u;
    const uint32_t lbeg  = loop_pc + 3u;
    const uint32_t lend1 = lbeg + 10u;      /* inner: 5 instructions */
    const uint32_t lend2 = lbeg + 12u;      /* outer: 6 instructions */

    put_insn2(&cpu, BASE, narrow(0xD, 15, 0, 3));            /* NOP.N */
    put_insn3(&cpu, loop_pc, (uint32_t)0x76u |
              ((uint32_t)((8 << 4) | 3) << 8) |
              ((lend1 - (loop_pc + 4u)) << 16));             /* LOOP a3 */
    put_insn2(&cpu, lbeg,      narrow(0xB, 4, 4, 1));        /* ADDI.N a4,a4,1 */
    put_insn2(&cpu, lbeg + 2u, narrow(0xD, 15, 0, 3));
    put_insn2(&cpu, lbeg + 4u, narrow(0xD, 15, 0, 3));
    put_insn2(&cpu, lbeg + 6u, narrow(0xD, 15, 0, 3));
    put_insn2(&cpu, lbeg + 8u, narrow(0xD, 15, 0, 3));
    put_insn2(&cpu, lend1,     narrow(0xB, 5, 5, 1));        /* ADDI.N a5,a5,1 */
    put_insn2(&cpu, lend2,     narrow(0xD, 15, 0, 0));       /* RET.N */
    ar_write(&cpu, 3, 500);
    ar_write(&cpu, 4, 0);
    ar_write(&cpu, 5, 0);

    jit_state_t *jit = jit_init();
    ASSERT_TRUE(jit != NULL);
    jit_install_hook(jit, &cpu);

    /* Compile the block for the inner loop. */
    cpu.pc = loop_pc;
    cpu.running = true;
    cpu._pc_written = true;
    xtensa_step(&cpu);
    ASSERT_EQ(cpu.lbeg, lbeg);
    ASSERT_EQ(cpu.lend, lend1);
    for (int i = 0; i <= JIT_HOT_THRESHOLD; i++)
        (void)jit_get_block(jit, &cpu, lbeg);
    ASSERT_TRUE(jit_get_block(jit, &cpu, lbeg) != NULL);

    /* Now arrive at the same head with the outer loop live. Its LEND is past
     * the block's end, so the dispatcher's own bound check does not decline
     * it -- only the block's entry check can. */
    ar_write(&cpu, 4, 0);
    ar_write(&cpu, 5, 0);
    cpu.pc     = lbeg;
    cpu.lbeg   = lbeg;
    cpu.lend   = lend2;
    cpu.lcount = 20;
    cpu.jit_loop_exit = 0;
    cpu._pc_written = true;
    (void)xtensa_run(&cpu, 400);

    /* Both counters live in the outer body, so they move together. A block
     * that ran its own back-edge here would end each iteration at lend1 and
     * never reach the a5 increment. */
    ASSERT_TRUE(ar_read(&cpu, 4) > 0u);
    ASSERT_EQ(ar_read(&cpu, 4), ar_read(&cpu, 5));

    jit_destroy(jit);
    teardown(&cpu);
}

/* A block that closes its own loop back-edge keeps its allocated guest
 * registers resident in host registers for the whole loop instead of writing
 * them back and reloading them every iteration. That makes every path *out*
 * of the body responsible for the write-back, and a conditional branch
 * leaving the loop mid-body is the one that is easy to get wrong: the
 * registers it must flush are the ones several iterations of arithmetic have
 * accumulated, and nothing else in the run would notice them being stale.
 *
 * The counters here are read back out of cpu->ar, so a missing flush shows up
 * as the guest-visible values lagging the work actually done. */
TEST(test_jit_self_loop_side_exit_flushes_resident_registers) {
    xtensa_cpu_t cpu;
    setup(&cpu);

    /* LOOP a3, lend
     *   lbeg: ADDI.N a4, a4, 1     accumulate
     *         ADDI.N a5, a5, 1     accumulate
     *         ADDI.N a6, a6, -1    countdown
     *         BEQZ.N a6, out       leave the loop from inside the body
     *         NOP.N
     *   lend: NOP.N
     *   out:  RET.N
     * a3 is far larger than the countdown, so the loop is left by the branch
     * and not by running out of iterations. */
    const uint32_t lbeg = BASE + 3u;
    const uint32_t lend = lbeg + 8u;
    const uint32_t out  = lend + 2u;
    put_insn3(&cpu, BASE, (uint32_t)0x76u | ((uint32_t)((8 << 4) | 3) << 8) |
                          ((lend - (BASE + 4u)) << 16));
    put_insn2(&cpu, lbeg,      narrow(0xB, 4, 4, 1));   /* ADDI.N a4, a4, 1 */
    put_insn2(&cpu, lbeg + 2u, narrow(0xB, 5, 5, 1));   /* ADDI.N a5, a5, 1 */
    put_insn2(&cpu, lbeg + 4u, narrow(0xB, 6, 6, 0));   /* ADDI.N a6, a6, -1 */
    /* BEQZ.N a6, out: t_hi = 2, imm6 = target - (next_pc + 2) */
    {
        uint32_t next_pc = lbeg + 8u;
        int imm6 = (int)(out - (next_pc + 2u));
        put_insn2(&cpu, lbeg + 6u,
                  narrow(0xC, imm6 & 0xF, 6, (2 << 2) | ((imm6 >> 4) & 3)));
    }
    put_insn2(&cpu, lend,      narrow(0xD, 15, 0, 3));  /* NOP.N */
    put_insn2(&cpu, out,       narrow(0xD, 15, 0, 0));  /* RET.N */
    ar_write(&cpu, 3, 5000);
    ar_write(&cpu, 4, 0);
    ar_write(&cpu, 5, 0);
    ar_write(&cpu, 6, 40);

    jit_state_t *jit = jit_init();
    ASSERT_TRUE(jit != NULL);
    jit_install_hook(jit, &cpu);

    cpu.running = true;
    cpu._pc_written = true;
    xtensa_step(&cpu);                       /* execute LOOP */
    ASSERT_EQ(cpu.pc, lbeg);
    ASSERT_EQ(cpu.lbeg, lbeg);

    /* Interpret into the body so the next arrival at LBEG is the
     * interpreter's back-edge, then compile the body block. */
    for (int i = 0; i < 6; i++)
        xtensa_step(&cpu);
    for (int i = 0; i <= JIT_HOT_THRESHOLD; i++)
        (void)jit_get_block(jit, &cpu, lbeg);
    ASSERT_TRUE(jit_get_block(jit, &cpu, lbeg) != NULL);

    cpu._pc_written = false;
    (void)xtensa_run(&cpu, 4000);

    /* The branch fired, so the loop was left from inside the body. */
    ASSERT_EQ(ar_read(&cpu, 6), 0u);
    /* Both accumulators counted once per iteration, including the one the
     * branch left from -- a4 and a5 are incremented before the test. */
    ASSERT_EQ(ar_read(&cpu, 4), 40u);
    ASSERT_EQ(ar_read(&cpu, 5), 40u);
    /* And the values are in the guest register file, not only in the host
     * registers the block was holding them in. */
    ASSERT_EQ(cpu.ar[(cpu.windowbase * 4 + 4) & 63], 40u);
    ASSERT_EQ(cpu.ar[(cpu.windowbase * 4 + 6) & 63], 0u);

    jit_destroy(jit);
    teardown(&cpu);
}

/* Compilation is driven from PCs the CPU is not sitting at: jit_run() samples
 * the branch-target ring and LBEG, and jit_compile_now() descends into branch
 * targets. For all of those the live loop registers belong to whatever loop
 * the CPU is in, not to the candidate PC. Classifying the candidate on LEND
 * alone calls a PC that sits *before* LBEG "in-loop" and files an unbounded
 * block in the in-loop slot -- the very slot that PC needs once its own loop
 * runs. The runtime bound check then declines that block every iteration, and
 * because a decline follows a hash *hit*, jit_get_block() never runs and the
 * correctly bounded block never compiles: the body interprets forever.
 *
 * This is not hypothetical. It cost the compute benchmark 18%: three of its
 * four hot loop bodies ran interpreted, at 2.7M declines per run, while the
 * JIT reported a healthy block count and no fallbacks. */
TEST(test_jit_loop_body_sampled_under_another_loop_still_compiles) {
    xtensa_cpu_t cpu;
    setup(&cpu);

    /* LOOP a3, lend ; five-instruction body ; RET.N at lend. */
    const uint32_t lbeg = BASE + 3u;
    const uint32_t lend = lbeg + 10u;
    put_insn3(&cpu, BASE, (uint32_t)0x76u | ((uint32_t)((8 << 4) | 3) << 8) |
                          ((lend - (BASE + 4u)) << 16));
    put_insn2(&cpu, lbeg,      narrow(0xB, 4, 4, 1));  /* ADDI.N a4, a4, 1 */
    put_insn2(&cpu, lbeg + 2u, narrow(0xD, 15, 0, 3)); /* NOP.N */
    put_insn2(&cpu, lbeg + 4u, narrow(0xD, 15, 0, 3)); /* NOP.N */
    put_insn2(&cpu, lbeg + 6u, narrow(0xD, 15, 0, 3)); /* NOP.N */
    put_insn2(&cpu, lbeg + 8u, narrow(0xD, 15, 0, 3)); /* NOP.N */
    put_insn2(&cpu, lend,      narrow(0xD, 15, 0, 0)); /* RET.N */
    ar_write(&cpu, 3, 400);
    ar_write(&cpu, 4, 0);

    jit_state_t *jit = jit_init();
    ASSERT_TRUE(jit != NULL);
    jit_install_hook(jit, &cpu);

    cpu.running = true;
    cpu._pc_written = true;
    xtensa_step(&cpu);                      /* execute LOOP: registers live */
    ASSERT_EQ(cpu.lbeg, lbeg);
    ASSERT_EQ(cpu.lend, lend);

    /* Now stand in for the sampler: a *different* loop, further along, is
     * live when this body's PC comes up as a compilation candidate. Its LEND
     * is ahead of lbeg, which is all the old classification looked at, but
     * its LBEG is well past it -- so lbeg is plainly not inside it. */
    cpu.lbeg   = lend + 4u;
    cpu.lend   = lend + 24u;
    cpu.lcount = 50u;
    for (int i = 0; i <= JIT_HOT_THRESHOLD; i++)
        (void)jit_get_block(jit, &cpu, lbeg);

    /* Back to the body's own loop, resuming mid-body so the next arrival at
     * LBEG is the interpreter's back-edge rather than a JIT dispatch. */
    cpu.lbeg   = lbeg;
    cpu.lend   = lend;
    cpu.lcount = 400u;
    cpu.pc     = lbeg + 4u;
    for (int i = 0; i <= JIT_HOT_THRESHOLD; i++)
        (void)jit_get_block(jit, &cpu, lbeg);

    cpu._pc_written = false;
    uint32_t a4_before = ar_read(&cpu, 4);
    uint32_t lcount_before = cpu.lcount;
    (void)xtensa_run(&cpu, 400);

    const jit_stats_t *stats = jit_get_stats(jit);
    /* The discriminating assertion. A poisoned in-loop slot shows up here and
     * nowhere else in the stats: blocks still compile, nothing falls back,
     * and the body simply never runs native. */
    ASSERT_EQ(stats->loop_bound_rejects, 0u);
    ASSERT_TRUE(stats->insns_jitted > 100);

    /* And it ran correctly: the body counts iterations in a4, so the
     * guest-visible effect must match the back-edges actually taken. */
    uint32_t taken = lcount_before - cpu.lcount;
    uint32_t pending = cpu.pc == cpu.lbeg ? 1u : 0u;
    ASSERT_EQ(ar_read(&cpu, 4) - a4_before, taken - pending);

    jit_destroy(jit);
    teardown(&cpu);
}

/* jit_scan_block() can only bound a block at LEND using the loop registers
 * as they stand when it compiles, so a block first compiled outside a loop is
 * scanned straight through what later becomes LEND. Re-entering that block
 * once the loop is live would execute the instruction at LEND -- which
 * belongs *after* the body -- instead of taking the back-edge, and would
 * leave LCOUNT undecremented. Real firmware does reach the same PC both ways:
 * this is what made two Marauder CYD board builds hang. */
TEST(test_jit_block_compiled_outside_a_loop_is_not_reused_inside_one) {
    xtensa_cpu_t cpu;
    setup(&cpu);

    /* LOOP sits at BASE+2 so that LBEG (always the next instruction, three
     * bytes on) lands in a different word. The block hash indexes on pc >> 2,
     * so a loop head and its LBEG in the same word share a slot and evict
     * each other -- harmless in production, but it would hide what this test
     * is checking. */
    const uint32_t loop_pc = BASE + 2u;
    const uint32_t lbeg = loop_pc + 3u;
    const uint32_t lend = lbeg + 10u;
    /* Body: five ADDI.N a4, a4, 1 */
    for (unsigned i = 0; i < 5u; i++)
        put_insn2(&cpu, lbeg + i * 2u, narrow(0xB, 4, 4, 1));
    /* At LEND, and so outside the body: ADDI.N a5, a5, 1. A block that runs
     * past LEND is exactly what bumps a5 while the loop is still going. */
    put_insn2(&cpu, lend, narrow(0xB, 5, 5, 1));
    put_insn2(&cpu, lend + 2u, narrow(0xB, 5, 5, 1));
    put_insn2(&cpu, lend + 4u, narrow(0xD, 15, 0, 0)); /* RET.N */
    put_insn2(&cpu, BASE, narrow(0xD, 15, 0, 3));      /* NOP.N */
    /* LOOP a3, lend */
    put_insn3(&cpu, loop_pc, (uint32_t)0x76u |
                             ((uint32_t)((8 << 4) | 3) << 8) |
                             ((lend - (loop_pc + 4u)) << 16));

    jit_state_t *jit = jit_init();
    ASSERT_TRUE(jit != NULL);
    jit_install_hook(jit, &cpu);

    /* Compile the body's PC while no loop is active: nothing bounds the scan,
     * so this block runs off the end of what will become the body. */
    ASSERT_EQ(cpu.lcount, 0u);
    for (int i = 0; i < JIT_HOT_THRESHOLD + 1; i++)
        (void)jit_get_block(jit, &cpu, lbeg);
    jit_block_fn outside = jit_get_block(jit, &cpu, lbeg);
    ASSERT_TRUE(outside != NULL);

    /* Now enter the loop for real. */
    ar_write(&cpu, 3, 100);
    ar_write(&cpu, 4, 0);
    ar_write(&cpu, 5, 0);
    cpu.pc = loop_pc;
    cpu.running = true;
    cpu._pc_written = true;
    xtensa_step(&cpu);
    ASSERT_EQ(cpu.pc, lbeg);
    ASSERT_EQ(cpu.lend, lend);
    ASSERT_TRUE(cpu.lcount > 0);

    /* The same PC inside the loop must not resolve to the block compiled
     * outside it. It gets its own, LEND-bounded compilation. */
    for (int i = 0; i < JIT_HOT_THRESHOLD + 1; i++)
        (void)jit_get_block(jit, &cpu, lbeg);
    jit_block_fn inside = jit_get_block(jit, &cpu, lbeg);
    ASSERT_TRUE(inside != NULL);
    ASSERT_TRUE(inside != outside);

    (void)xtensa_run(&cpu, 200);

    /* Still mid-loop, so the instruction at LEND must not have run yet. */
    ASSERT_TRUE(cpu.lcount > 0);
    ASSERT_EQ(ar_read(&cpu, 5), 0u);
    /* The body bumps a4 once per instruction, so a4 pins down both how many
     * back-edges were taken and how far into the current iteration the run
     * stopped -- an off-by-one in either shows up here. */
    ASSERT_TRUE(cpu.pc >= lbeg && cpu.pc < lend);
    ASSERT_EQ(ar_read(&cpu, 4), 5u * (99u - cpu.lcount) + (cpu.pc - lbeg) / 2u);

    jit_destroy(jit);
    teardown(&cpu);
}

/* cycle_count is elapsed simulated time and advances while the core is
 * halted in WAITI, because timers and task wakeups are scheduled against it.
 * Throughput must not be measured in those units: a firmware that mostly
 * sleeps would report a MIPS figure it never achieved. idle_cycles records
 * the sleeping part so the two can be told apart. */
TEST(test_waiti_time_is_not_counted_as_retired_instructions) {
    xtensa_cpu_t cpu;
    setup(&cpu);
    /* WAITI 0 */
    put_insn3(&cpu, BASE, 0x007000u);
    put_insn2(&cpu, BASE + 3u, narrow(0xD, 15, 0, 3));
    cpu.running = true;
    cpu._pc_written = true;

    uint64_t insns_before = xtensa_retired_insns(&cpu);
    xtensa_step(&cpu);                      /* retires WAITI, then halts */
    ASSERT_TRUE(cpu.halted);
    ASSERT_EQ64(xtensa_retired_insns(&cpu) - insns_before, 1u);

    uint64_t cycles_before = cpu.cycle_count;
    insns_before = xtensa_retired_insns(&cpu);
    int ran = xtensa_run(&cpu, 5000);

    /* Simulated time advanced across the whole batch... */
    ASSERT_EQ64(cpu.cycle_count - cycles_before, (uint64_t)ran);
    ASSERT_TRUE(ran > 0);
    /* ...while nothing retired, because the core never left WAITI. */
    ASSERT_TRUE(cpu.halted);
    ASSERT_EQ64(xtensa_retired_insns(&cpu) - insns_before, 0u);

    teardown(&cpu);
}

/* ===== Encoding-space differential sweep =====
 *
 * SRC and every byte store shipped miscompiled because the JIT differential
 * tests covered a couple of dozen hand-picked instructions and neither was
 * among them. Enumerating the encoding space instead cannot drift from what
 * the emitter actually claims to handle: the candidate goes after four NOPs,
 * and the block's own reported instruction count says whether the JIT
 * compiled it or stopped short. Anything it declined is the interpreter's
 * problem and is skipped.
 *
 * Only straight-line cases are compared. If the interpreter did not land on
 * the instruction after the candidate -- a branch, a call, a halt, a trap --
 * the two engines are not executing comparable work and the case is skipped
 * rather than guessed at.
 */
#define FUZZ_SCRATCH 0x3FFE0000u
#define FUZZ_PAD     4            /* NOP.N slots before the candidate */

static void fuzz_program(xtensa_cpu_t *cpu, uint32_t insn, int ilen,
                         unsigned seed) {
    setup(cpu);
    for (unsigned i = 0; i < FUZZ_PAD; i++)
        put_insn2(cpu, BASE + i * 2u, narrow(0xD, 15, 0, 3));   /* NOP.N */
    if (ilen == 2) put_insn2(cpu, BASE + FUZZ_PAD * 2u, (uint16_t)insn);
    else           put_insn3(cpu, BASE + FUZZ_PAD * 2u, insn);
    /* Trailing NOPs rather than a RET: a terminator would be part of the
     * block, and its target is a scratch pointer rather than code. */
    for (unsigned i = 0; i < 4u; i++)
        put_insn2(cpu, BASE + FUZZ_PAD * 2u + (unsigned)ilen + i * 2u,
                  narrow(0xD, 15, 0, 3));

    /* Two register profiles. Profile 0 makes every register a distinct valid
     * scratch pointer, so whichever field an encoding treats as a base
     * address stays inside mapped RAM -- that is the only way memory
     * instructions can be exercised at all. Profile 1 uses the values that
     * break arithmetic instead (zero, +/-1, the signed extremes), which
     * profile 0 cannot reach because every value there is a small positive
     * address; without it nothing distinguishes a signed from an unsigned
     * multiply or shift. */
    static const uint32_t extremes[16] = {
        0x00000000u, 0x00000001u, 0xFFFFFFFFu, 0x80000000u,
        0x7FFFFFFFu, 0x00008000u, 0xFFFF8000u, 0x0000FFFFu,
        0xFFFF0000u, 0x55555555u, 0xAAAAAAAAu, 0x00000020u,
        0x0000001Fu, 0xDEADBEEFu, 0x00000002u, 0xFFFFFFFEu,
    };
    for (int i = 0; i < 16; i++)
        ar_write(cpu, i, (seed & 1u)
                 ? extremes[(i + (int)(seed >> 1)) & 15]
                 : FUZZ_SCRATCH + 0x400u + (uint32_t)i * 0x54u);
    cpu->br = (uint16_t)(seed * 0x9E37u);
    for (uint32_t off = 0; off < 0x800u; off += 4u)
        mem_write32(cpu->mem, FUZZ_SCRATCH + off,
                    0x9E3779B9u * (off + seed) + 0x12345678u);
    cpu->sar = (seed >> 1) % 33u;
    cpu->windowbase = 0;
    cpu->running = true;
    cpu->_pc_written = true;
}

static int fuzz_mem_differs(xtensa_cpu_t *a, xtensa_cpu_t *b, uint32_t pc,
                            const char *name) {
    int diffs = 0;
    for (uint32_t off = 0; off < 0x800u; off += 4u) {
        uint32_t addr = FUZZ_SCRATCH + off;
        uint32_t va = mem_read32(a->mem, addr), vb = mem_read32(b->mem, addr);
        if (va != vb) {
            fprintf(stderr, "  DIFF %s (insn @%08X): mem[%08X] interp=%08X "
                    "jit=%08X\n", name, pc, addr, va, vb);
            if (++diffs > 4) break;
        }
    }
    return diffs;
}

/* Returns 1 if the case was actually compared, 0 if skipped. */
static int fuzz_case(uint32_t insn, int ilen, unsigned seed) {
    xtensa_cpu_t ic, jc;

    /* The native block runs first because only its return value says how many
     * guest instructions it covered -- it may stop short of the candidate, or
     * run past it into the trailing NOPs. The interpreted reference then
     * replays exactly that many, so the two are always comparing the same
     * instruction sequence. */
    fuzz_program(&jc, insn, ilen, seed);
    jit_state_t *jit = jit_init();
    if (!jit) { teardown(&jc); return 0; }
    for (int i = 0; i < JIT_HOT_THRESHOLD + 1; i++)
        (void)jit_get_block(jit, &jc, BASE);
    jit_block_fn fn = jit_get_block(jit, &jc, BASE);
    if (!fn) { jit_destroy(jit); teardown(&jc); return 0; }

    int ran = fn(&jc);
    /* The block stopped before the candidate: the JIT declined to compile it
     * and the interpreter owns it. */
    if (ran < (int)FUZZ_PAD + 1) {
        jit_destroy(jit); teardown(&jc); return 0;
    }

    fuzz_program(&ic, insn, ilen, seed);
    run_interp(&ic, ran);
    /* Trapped, halted or rotated the window: not comparable work. */
    if (ic.exception || ic.halted || !ic.running || ic.windowbase != 0 ||
        jc.exception) {
        jit_destroy(jit); teardown(&ic); teardown(&jc); return 0;
    }

    char name[32];
    snprintf(name, sizeof name, "fuzz:%06X/%d/%u", insn & 0xFFFFFF, ilen,
             seed & 1u);
    int diffs = compare_state(&ic, &jc, name);
    /* Memory is only comparable under profile 0. Profile 1's registers are
     * not addresses, so anything that stores lands somewhere arbitrary and
     * identical in both engines -- but not worth asserting on. */
    if ((seed & 1u) == 0u)
        diffs += fuzz_mem_differs(&ic, &jc, insn, name);
    if (diffs == 0) test_passes++; else test_failures += diffs;

    jit_destroy(jit);
    teardown(&ic);
    teardown(&jc);
    return 1;
}

TEST(test_jit_encoding_sweep_matches_interpreter) {
    /* Operand triples chosen to cross the register allocator's boundaries:
     * a1-a6 live in host registers and a0/a7-a15 are spilled, so a wrong host
     * register only shows up when both kinds are in play. Same-register forms
     * are included because an emitter that clobbers its source before reading
     * it is only wrong when they coincide -- and RBP/RSI/RDI destinations are
     * where the byte-store REX bug lived. */
    static const int rst[][3] = {
        {2, 3, 4}, {5, 5, 6}, {3, 3, 3}, {7, 2, 9},
        {0, 1, 15}, {15, 0, 1}, {1, 14, 6}, {12, 12, 2},
        {6, 10, 11}, {10, 6, 0},
    };
    const unsigned ntriples = (unsigned)(sizeof(rst) / sizeof(rst[0]));
    int compared = 0;
    unsigned seed = 1u;

    /* 24-bit formats. For op0=0 the op1/op2 loops walk the QRST sub-opcode
     * space; for op0=2 (RRI8) the same bits are the 8-bit offset, so the
     * sweep covers load/store displacements as a side effect. op0 1 (L32R)
     * and 5-7 (calls and branches) are PC-relative and left out: they would
     * be dropped by the straight-line filter anyway. op0 3 compiles nothing
     * today, and is listed so that it starts being covered if it ever does. */
    static const unsigned op0s[] = {0u, 2u, 3u, 4u};
    for (unsigned oi = 0; oi < sizeof(op0s) / sizeof(op0s[0]); oi++) {
        for (unsigned op1 = 0; op1 < 16u; op1++) {
            for (unsigned op2 = 0; op2 < 16u; op2++) {
                for (unsigned k = 0; k < ntriples; k++) {
                    uint32_t insn = op0s[oi] |
                                    ((uint32_t)rst[k][2] << 4) |
                                    ((uint32_t)rst[k][1] << 8) |
                                    ((uint32_t)rst[k][0] << 12) |
                                    (op1 << 16) | (op2 << 20);
                    compared += fuzz_case(insn, 3, seed * 2u);
                    compared += fuzz_case(insn, 3, seed * 2u + 1u);
                    seed++;
                }
            }
        }
    }

    /* 16-bit density formats. */
    for (unsigned op0 = 8u; op0 <= 15u; op0++)
        for (unsigned r = 0; r < 16u; r++)
            for (unsigned k = 0; k < ntriples; k++) {
                uint32_t insn = op0 | ((uint32_t)rst[k][2] << 4) |
                                ((uint32_t)rst[k][1] << 8) | (r << 12);
                compared += fuzz_case(insn, 2, seed * 2u);
                compared += fuzz_case(insn, 2, seed * 2u + 1u);
                seed++;
            }

    /* The sweep is only meaningful if it actually reached the emitter. */
    ASSERT_TRUE(compared > 200);
    fprintf(stderr, "  [sweep] %d encodings compiled and compared\n", compared);
}

/* insn_count is separate state, so a checkpoint has to carry it. Restoring
 * only cycle_count would leave a resumed run reporting far more idle than it
 * had, since retired work would restart from zero against a cycle count that
 * did not. */
TEST(test_savestate_round_trips_retired_instruction_count) {
    char path[] = "/tmp/flexe-savestate-test-XXXXXX";
    int fd = mkstemp(path);
    ASSERT_TRUE(fd >= 0);
    close(fd);

    xtensa_cpu_t saved;
    setup(&saved);
    saved.cycle_count = 1234567u;
    saved.insn_count = 987654u;
    saved.expstate = 0xE6E6E6E6u;
    saved.threadptr = 0x3FFB7A80u;
    saved.fcr = 0x0000001Fu;
    saved.fsr = 0x00000078u;
    saved.f64r_lo = 0x01234567u;
    saved.f64r_hi = 0x89ABCDEFu;
    saved.f64s = 0x13579BDFu;
    mem_write32(saved.mem, 0x3FF801FCu, 0xF45A7A57u);
    mem_write32(saved.mem, 0x500001FCu, 0x5100A11Eu);
    ASSERT_EQ64(xtensa_retired_insns(&saved), 987654u);
    ASSERT_EQ(savestate_save(&saved, NULL, path, "unit-test"), 0);

    xtensa_cpu_t restored;
    setup(&restored);
    ASSERT_EQ64(restored.insn_count, 0u);
    ASSERT_EQ(savestate_restore(&restored, NULL, path), 0);
    ASSERT_EQ64(restored.cycle_count, 1234567u);
    ASSERT_EQ64(xtensa_retired_insns(&restored), 987654u);
    ASSERT_EQ(restored.expstate, 0xE6E6E6E6u);
    ASSERT_EQ(restored.threadptr, 0x3FFB7A80u);
    ASSERT_EQ(restored.fcr, 0x0000001Fu);
    ASSERT_EQ(restored.fsr, 0x00000078u);
    ASSERT_EQ(restored.f64r_lo, 0x01234567u);
    ASSERT_EQ(restored.f64r_hi, 0x89ABCDEFu);
    ASSERT_EQ(restored.f64s, 0x13579BDFu);
    ASSERT_EQ(mem_read32(restored.mem, 0x3FF801FCu), 0xF45A7A57u);
    ASSERT_EQ(mem_read32(restored.mem, 0x500001FCu), 0x5100A11Eu);

    unlink(path);
    teardown(&saved);
    teardown(&restored);
}

TEST(test_jit_flush) {
    jit_state_t *jit = jit_init();
    ASSERT_TRUE(jit != NULL);
    jit_flush(jit);
    ASSERT_EQ(jit_get_stats(jit)->cache_flushes, 1);
    jit_destroy(jit);
}

TEST(test_jit_flash_mmu_remap_flushes_upper_window) {
    xtensa_cpu_t cpu;
    setup(&cpu);
    esp32_periph_t *periph = periph_create(cpu.mem);
    periph_attach_cpus(periph, &cpu, NULL);
    const uint32_t pc = 0x40400000u;
    const uint32_t mmu_entry = 128u;
    uint16_t nop_n = narrow(0xD, 15, 0, 3);
    for (uint32_t i = 0; i < 4u; i++) {
        memcpy(cpu.mem->flash_insn + 0x30000u + i * 2u,
               &nop_n, sizeof(nop_n));
        memcpy(cpu.mem->flash_insn + 0x40000u + i * 2u,
               &nop_n, sizeof(nop_n));
    }
    mem_write32(cpu.mem, 0x3FF10000u + mmu_entry * 4u, 3u);

    jit_state_t *jit = jit_init();
    ASSERT_TRUE(jit != NULL);
    jit_install_hook(jit, &cpu);
    cpu.pc = pc;
    for (int i = 0; i < JIT_HOT_THRESHOLD; i++)
        (void)jit_get_block(jit, &cpu, pc);
    ASSERT_TRUE(jit_get_block(jit, &cpu, pc) != NULL);
    ASSERT_EQ(jit_get_stats(jit)->blocks_compiled, 1u);

    /* Remapping this executable page invalidates both the old native block
     * and its hook bit. The replacement page must become independently hot. */
    mem_write32(cpu.mem, 0x3FF10000u + mmu_entry * 4u, 4u);
    ASSERT_EQ(jit_get_stats(jit)->cache_flushes, 1u);
    ASSERT_TRUE(jit_get_block(jit, &cpu, pc) == NULL);
    for (int i = 0; i < JIT_HOT_THRESHOLD; i++)
        (void)jit_get_block(jit, &cpu, pc);
    ASSERT_TRUE(jit_get_block(jit, &cpu, pc) != NULL);
    ASSERT_EQ(jit_get_stats(jit)->blocks_compiled, 2u);

    cpu.pc = pc;
    cpu.running = true;
    cpu._pc_written = true;
    ASSERT_EQ(xtensa_run(&cpu, 1), 4u);
    ASSERT_EQ(cpu.pc, pc + 8u);
    ASSERT_TRUE(jit_get_stats(jit)->blocks_executed >= 1u);

    jit_destroy(jit);
    cpu.code_invalidate = NULL;
    cpu.code_invalidate_ctx = NULL;
    periph_destroy(periph);
    teardown(&cpu);
}

TEST(test_jit_xtensa_run_counts_guest_instructions) {
    xtensa_cpu_t cpu;
    setup(&cpu);
    put_insn2(&cpu, BASE,     narrow(0xD, 15, 0, 3));
    put_insn2(&cpu, BASE + 2, narrow(0xD, 15, 0, 3));
    put_insn2(&cpu, BASE + 4, narrow(0xD, 15, 0, 3));
    put_insn2(&cpu, BASE + 6, narrow(0xD, 15, 0, 3));

    jit_state_t *jit = jit_init();
    ASSERT_TRUE(jit != NULL);
    jit_install_hook(jit, &cpu);
    for (int i = 0; i < JIT_HOT_THRESHOLD; i++)
        (void)jit_get_block(jit, &cpu, BASE);

    cpu._pc_written = true;
    uint32_t ccount_before = cpu.ccount;
    uint64_t cycles_before = cpu.cycle_count;
    int ran = xtensa_run(&cpu, 1);
    ASSERT_EQ(ran, 4);
    ASSERT_EQ(cpu.ccount - ccount_before, 4);
    ASSERT_EQ64(cpu.cycle_count - cycles_before, 4);

    jit_destroy(jit);
    teardown(&cpu);
}

static int jit_time_jump_hook(xtensa_cpu_t *cpu, uint32_t pc, void *ctx) {
    (void)ctx;
    if (pc != 0x40001000u) return 0;
    cpu->ccount += 240000000u; /* delay(1000) at the ESP32's 240 MHz. */
    cpu->pc = BASE;
    return 1;
}

TEST(test_jit_run_does_not_count_delay_ccount_as_instructions) {
    xtensa_cpu_t cpu;
    setup(&cpu);
    for (unsigned i = 0; i < 4; i++)
        put_insn2(&cpu, BASE + i * 2u, narrow(0xD, 15, 0, 3));

    cpu.pc = 0x40001000u;
    cpu.running = true;
    cpu._pc_written = true;
    cpu.pc_hook = jit_time_jump_hook;

    jit_state_t *jit = jit_init();
    ASSERT_TRUE(jit != NULL);
    jit_install_hook(jit, &cpu);
    /* This synthetic hook has no ROM-stub bitmap entry.  Probe every control
     * transfer for the test, as a symbol-registered production stub would be
     * present in the merged bitmap. */
    cpu.pc_hook_bitmap = NULL;

    uint32_t ccount_before = cpu.ccount;
    uint64_t cycles_before = cpu.cycle_count;
    int ran = jit_run(jit, &cpu, 4);
    ASSERT_EQ(ran, 4);
    ASSERT_EQ(cpu.ccount - ccount_before, 240000004u);
    ASSERT_EQ64(cpu.cycle_count - cycles_before, 4u);
    ASSERT_EQ64(jit_get_stats(jit)->insns_interp, 4u);

    jit_destroy(jit);
    teardown(&cpu);
}

TEST(test_jit_rsr_wsr_sar) {
    xtensa_cpu_t cpu;
    setup(&cpu);
    ar_write(&cpu, 2, 0xFEDCBA7Fu);
    /* sr occupies bits 15:8. SAR=3 therefore encodes r=0, s=3. */
    put_insn3(&cpu, BASE, rrr(1, 3, 0, 3, 2));  /* WSR SAR, a2 */
    put_insn3(&cpu, BASE + 3, rrr(0, 3, 0, 3, 5));  /* RSR a5, SAR */
    test_block_differential(&cpu, 2, "rsr_wsr_sar");
    teardown(&cpu);
}

TEST(test_jit_rsr_wsr_mac16_state) {
    xtensa_cpu_t cpu;
    setup(&cpu);
    static const uint8_t sr[] = {
        XT_SR_ACCLO, XT_SR_ACCHI,
        XT_SR_MR0, XT_SR_MR1, XT_SR_MR2, XT_SR_MR3,
    };
    static const uint32_t value[] = {
        0x11223344u, 0xA5A5FE7Bu,
        0x01020304u, 0x55667788u, 0x89ABCDEFu, 0xDEADBEEFu,
    };

    /* These are ordinary architectural context-save fields. Exercise every
     * write followed by its matching read in one compiled block; ACCHI also
     * proves that the JIT applies the interpreter's eight-bit write mask. */
    uint32_t pc = BASE;
    for (unsigned i = 0; i < sizeof(sr) / sizeof(sr[0]); i++) {
        ar_write(&cpu, 2 + (int)i, value[i]);
        put_insn3(&cpu, pc,
                  rrr(1, 3, sr[i] >> 4, sr[i] & 15, 2 + (int)i));
        pc += 3u;
    }
    for (unsigned i = 0; i < sizeof(sr) / sizeof(sr[0]); i++) {
        put_insn3(&cpu, pc,
                  rrr(0, 3, sr[i] >> 4, sr[i] & 15, 8 + (int)i));
        pc += 3u;
    }
    test_block_differential(&cpu, 12, "rsr_wsr_mac16_state");
    teardown(&cpu);
}

TEST(test_jit_wsr_br_applies_architectural_mask) {
    xtensa_cpu_t cpu;
    setup(&cpu);
    ar_write(&cpu, 2, 0xDEADBEEFu);
    put_insn3(&cpu, BASE,
              rrr(1, 3, XT_SR_BR >> 4, XT_SR_BR & 15, 2));
    put_insn3(&cpu, BASE + 3u,
              rrr(0, 3, XT_SR_BR >> 4, XT_SR_BR & 15, 5));
    test_block_differential(&cpu, 2, "wsr_br_mask");
    teardown(&cpu);
}

TEST(test_jit_rsr_wsr_cpenable) {
    xtensa_cpu_t cpu;
    setup(&cpu);
    cpu.cpenable = 1u;
    ar_write(&cpu, 2, 0xA5A55A5Au);
    put_insn3(&cpu, BASE,
              rrr(1, 3, XT_SR_CPENABLE >> 4,
                  XT_SR_CPENABLE & 15, 2));  /* WSR CPENABLE, a2 */
    put_insn3(&cpu, BASE + 3u,
              rrr(0, 3, XT_SR_CPENABLE >> 4,
                  XT_SR_CPENABLE & 15, 5));  /* RSR a5, CPENABLE */
    test_run_differential(&cpu, 2, "rsr_wsr_cpenable");
    teardown(&cpu);
}

TEST(test_jit_wsr_windowstart_terminates_at_new_guard_context) {
    xtensa_cpu_t cpu;
    setup(&cpu);
    cpu.real_window_vectors = true;
    cpu.windowstart = 1u;
    ar_write(&cpu, 2, 0xABCD4321u);
    put_insn3(&cpu, BASE,
              rrr(1, 3, XT_SR_WINDOWSTART >> 4,
                  XT_SR_WINDOWSTART & 15, 2));
    put_insn2(&cpu, BASE + 3u, narrow(0xD, 15, 0, 3)); /* NOP.N */
    test_run_differential(&cpu, 2, "wsr_windowstart_boundary");
    teardown(&cpu);
}

TEST(test_jit_wsr_windowbase_flushes_old_mapping_before_dispatch) {
    xtensa_cpu_t cpu;
    setup(&cpu);
    cpu.real_window_vectors = true;
    cpu.windowbase = 0;
    ar_write(&cpu, 2, 2u);
    /* Keep the WSR source dirty in the native register allocator. */
    put_insn2(&cpu, BASE, narrow(0xB, 2, 2, 1)); /* ADDI.N a2, a2, 1 */
    put_insn3(&cpu, BASE + 2u,
              rrr(1, 3, XT_SR_WINDOWBASE >> 4,
                  XT_SR_WINDOWBASE & 15, 2));
    put_insn2(&cpu, BASE + 5u, narrow(0xD, 15, 0, 3)); /* NOP.N */
    test_run_differential(&cpu, 3, "wsr_windowbase_boundary");
    teardown(&cpu);
}

TEST(test_jit_wsr_windowbase_chains_under_runtime_destination) {
    xtensa_cpu_t cpu;
    setup(&cpu);
    const uint32_t target = BASE + 5u;

    cpu.real_window_vectors = true;
    cpu.windowbase = 0u;
    ar_write(&cpu, 2, 0u);
    /* Keep the WSR source dirty so this covers both the old-window flush and
     * the dynamic chain selector. */
    put_insn2(&cpu, BASE, narrow(0xB, 2, 2, 1)); /* ADDI.N a2, a2, 1 */
    put_insn3(&cpu, BASE + 2u,
              rrr(1, 3, XT_SR_WINDOWBASE >> 4,
                  XT_SR_WINDOWBASE & 15, 2));
    for (unsigned i = 0; i < 4u; i++)
        put_insn2(&cpu, target + i * 2u,
                  narrow(0xD, 15, 0, 3)); /* NOP.N */
    put_insn2(&cpu, target + 8u,
              narrow(0xD, 15, 0, 2)); /* ILL.N ends the target block */

    jit_state_t *jit = jit_init();
    ASSERT_TRUE(jit != NULL);
    jit_install_hook(jit, &cpu);

    /* Every leaf has its own physical-register mapping and patchable chain
     * site. Compile all sixteen so the test exercises the complete selector,
     * including its wraparound value. */
    for (uint32_t dest_wb = 0u; dest_wb < 16u; dest_wb++) {
        cpu.windowbase = dest_wb;
        for (int i = 0; i < JIT_HOT_THRESHOLD; i++)
            (void)jit_get_block(jit, &cpu, target);
        ASSERT_TRUE(jit_get_block(jit, &cpu, target) != NULL);
    }

    cpu.windowbase = 0u;
    for (int i = 0; i < JIT_HOT_THRESHOLD; i++)
        (void)jit_get_block(jit, &cpu, BASE);
    ASSERT_TRUE(jit_get_block(jit, &cpu, BASE) != NULL);
    ASSERT_TRUE(jit_get_stats(jit)->chains_patched > 0u);

    for (uint32_t dest_wb = 0u; dest_wb < 16u; dest_wb++) {
        cpu.windowbase = 0u;
        ar_write(&cpu, 2, dest_wb - 1u); /* ADDI.N produces dest_wb */
        uint64_t hooks_before = jit_get_stats(jit)->hook_calls;
        cpu.pc = BASE;
        cpu._pc_written = true;
        cpu.running = true;
        ASSERT_EQ(xtensa_run(&cpu, 6), 6);
        ASSERT_EQ(cpu.windowbase, dest_wb);
        ASSERT_EQ(cpu.pc, target + 8u);
        ASSERT_EQ64(jit_get_stats(jit)->hook_calls - hooks_before, 1u);
    }

    jit_destroy(jit);
    teardown(&cpu);
}

TEST(test_jit_rsr_prid_wsr_ps) {
    xtensa_cpu_t cpu;
    setup(&cpu);
    cpu.prid = XTENSA_SPINLOCK_OWNER_CORE1;
    cpu.ps = 0x00060003u;
    ar_write(&cpu, 6, 0x00060000u);
    /* The exact pair used by ESP-IDF spinlock acquire/release: identify the
     * core through PRID, then restore the saved PS after the critical region. */
    put_insn3(&cpu, BASE,
              rrr(0, 3, XT_SR_PRID >> 4, XT_SR_PRID & 15, 5));
    put_insn3(&cpu, BASE + 3,
              rrr(1, 3, XT_SR_PS >> 4, XT_SR_PS & 15, 6));
    test_block_differential(&cpu, 2, "rsr_prid_wsr_ps");
    teardown(&cpu);
}

TEST(test_jit_rsr_ccount_observes_instruction_position) {
    xtensa_cpu_t cpu;
    setup(&cpu);
    cpu.ccount = 1000u;

    put_insn2(&cpu, BASE,      narrow(0xD, 15, 0, 3)); /* NOP.N */
    put_insn2(&cpu, BASE + 2u, narrow(0xD, 15, 0, 3)); /* NOP.N */
    put_insn3(&cpu, BASE + 4u,
              rrr(0, 3, XT_SR_CCOUNT >> 4, XT_SR_CCOUNT & 15, 5));
    put_insn2(&cpu, BASE + 7u, narrow(0xD, 15, 0, 3)); /* NOP.N */
    test_block_differential(&cpu, 4, "rsr_ccount_position");
    teardown(&cpu);
}

TEST(test_jit_wsr_ps_rearms_irq_check) {
    xtensa_cpu_t cpu;
    setup(&cpu);
    ar_write(&cpu, 6, 0x00060000u);
    put_insn3(&cpu, BASE,
              rrr(1, 3, XT_SR_PS >> 4, XT_SR_PS & 15, 6));
    put_insn2(&cpu, BASE + 3, narrow(0xD, 15, 0, 3));
    put_insn2(&cpu, BASE + 5, narrow(0xD, 15, 0, 3));
    put_insn2(&cpu, BASE + 7, narrow(0xD, 15, 0, 3));

    jit_state_t *jit = jit_init();
    ASSERT_TRUE(jit != NULL);
    for (int i = 0; i < JIT_HOT_THRESHOLD; i++)
        (void)jit_get_block(jit, &cpu, BASE);
    jit_block_fn fn = jit_get_block(jit, &cpu, BASE);
    ASSERT_TRUE(fn != NULL);
    ASSERT_EQ(fn(&cpu), 4);
    ASSERT_EQ(cpu.ps, 0x00060000u);
    ASSERT_TRUE(cpu.irq_check);

    jit_destroy(jit);
    teardown(&cpu);
}

TEST(test_jit_wsr_ps_exits_before_pending_irq) {
    xtensa_cpu_t cpu;
    setup(&cpu);
    cpu.ps = 3u;
    cpu.interrupt = 1u << 6;
    cpu.intenable = 1u << 6;
    ar_write(&cpu, 6, 0u);
    put_insn3(&cpu, BASE,
              rrr(1, 3, XT_SR_PS >> 4, XT_SR_PS & 15, 6));
    put_insn2(&cpu, BASE + 3, narrow(0xD, 15, 0, 3));
    put_insn2(&cpu, BASE + 5, narrow(0xD, 15, 0, 3));
    put_insn2(&cpu, BASE + 7, narrow(0xD, 15, 0, 3));

    jit_state_t *jit = jit_init();
    ASSERT_TRUE(jit != NULL);
    for (int i = 0; i < JIT_HOT_THRESHOLD; i++)
        (void)jit_get_block(jit, &cpu, BASE);
    jit_block_fn fn = jit_get_block(jit, &cpu, BASE);
    ASSERT_TRUE(fn != NULL);
    ASSERT_EQ(fn(&cpu), 1);
    ASSERT_EQ(cpu.pc, BASE + 3);
    ASSERT_TRUE(cpu.irq_check);

    jit_destroy(jit);
    teardown(&cpu);
}

TEST(test_jit_rur_wur_user_registers) {
    xtensa_cpu_t cpu;
    setup(&cpu);
    cpu.f64r_lo = 0xD00DFEEDu;
    ar_write(&cpu, 5, 0x3FFB7A80u);
    ar_write(&cpu, 10, 0xA10A10A1u);
    /* WUR THREADPTR,a5: UR=(r<<4)|s. */
    put_insn3(&cpu, BASE, rrr(15, 3,
                              XT_UR_THREADPTR >> 4,
                              XT_UR_THREADPTR & 15, 5));
    /* RUR a3,THREADPTR and the exact ESP-IDF rur.f64r_lo a4 shape. */
    put_insn3(&cpu, BASE + 3, rrr(14, 3, 3,
                                  XT_UR_THREADPTR >> 4,
                                  XT_UR_THREADPTR & 15));
    put_insn3(&cpu, BASE + 6, rrr(14, 3, 4,
                                  XT_UR_F64R_LO >> 4,
                                  XT_UR_F64R_LO & 15));
    test_block_differential(&cpu, 3, "rur_wur_user_registers");
    teardown(&cpu);
}

/* Keep these builders local to the JIT suite: test_main.c includes every
 * test source into one translation unit, and test_window.c has equivalents. */
static uint32_t jit_calln_insn(int nn, int32_t offset) {
    uint32_t off18 = (uint32_t)offset & 0x3FFFF;
    return (off18 << 6) | ((nn & 3) << 4) | 5;
}

static uint32_t jit_callx_insn(int nn, int source) {
    return ((uint32_t)source << 8) |
           ((uint32_t)((3 << 2) | (nn & 3)) << 4);
}

static uint32_t jit_entry_insn(int s, uint32_t framesize) {
    uint32_t imm12 = (framesize >> 3) & 0xFFF;
    return (imm12 << 12) | ((uint32_t)s << 8) | (3u << 4) | 6u;
}

static uint32_t jit_rotw_insn(int amount) {
    return rrr(4, 0, 8, 0, amount & 15);
}

static uint32_t jit_l32e_insn(int t, int s, int byte_offset) {
    int r = (byte_offset + 64) >> 2;
    return rrr(0, 9, r & 15, s, t);
}

static uint32_t jit_s32e_insn(int t, int s, int byte_offset) {
    int r = (byte_offset + 64) >> 2;
    return rrr(4, 9, r & 15, s, t);
}

static uint32_t jit_rfwo_insn(void) {
    return rrr(0, 0, 3, 4, 0);
}

static uint32_t jit_rfwu_insn(void) {
    return rrr(0, 0, 3, 5, 0);
}

TEST(test_jit_entry_dispatches_compiled_callee_body) {
    xtensa_cpu_t cpu;
    setup(&cpu);
    cpu.ps = 2u << 16;  /* CALLINC=2 */
    cpu.windowbase = 0;
    cpu.windowstart = 1u;
    ar_write(&cpu, 1, DATA_BASE + 0x400u);
    put_insn3(&cpu, BASE, jit_entry_insn(1, 32));
    for (unsigned i = 0; i < 4; i++)
        put_insn2(&cpu, BASE + 3u + i * 2u,
                  narrow(0xD, 15, 0, 3));  /* NOP.N */

    jit_state_t *jit = jit_init();
    ASSERT_TRUE(jit != NULL);
    jit_install_hook(jit, &cpu);

    /* Compile the body for the window ENTRY will select, then return to the
     * caller state so execution still has to cross the interpreted ENTRY. */
    cpu.windowbase = 2;
    for (int i = 0; i < JIT_HOT_THRESHOLD; i++)
        (void)jit_get_block(jit, &cpu, BASE + 3);
    ASSERT_TRUE(jit_get_block(jit, &cpu, BASE + 3) != NULL);
    cpu.windowbase = 0;

    cpu.running = true;
    cpu._pc_written = true;
    ASSERT_EQ(xtensa_run(&cpu, 5), 5);
    ASSERT_EQ(cpu.windowbase, 2);
    ASSERT_EQ64(jit_get_stats(jit)->insns_jitted, 4u);

    jit_destroy(jit);
    teardown(&cpu);
}

TEST(test_jit_entry_chains_to_runtime_window) {
    xtensa_cpu_t cpu;
    setup(&cpu);
    cpu.ps = 0;
    cpu.windowbase = 14;
    cpu.windowstart = 1u << 14;
    ar_write(&cpu, 1, DATA_BASE + 0x400u);
    put_insn3(&cpu, BASE, jit_entry_insn(1, 32));
    for (unsigned i = 0; i < 4; i++)
        put_insn2(&cpu, BASE + 3u + i * 2u,
                  narrow(0xD, 15, 0, 3));  /* NOP.N */

    jit_state_t *jit = jit_init();
    ASSERT_TRUE(jit != NULL);
    jit_install_hook(jit, &cpu);

    /* Precompile the body under all four windows ENTRY can select, including
     * wraparound, then compile ENTRY under its caller window. */
    for (uint32_t ci = 0; ci < 4; ci++) {
        cpu.windowbase = (14u + ci) & 15u;
        for (int i = 0; i < JIT_HOT_THRESHOLD; i++)
            (void)jit_get_block(jit, &cpu, BASE + 3u);
        ASSERT_TRUE(jit_get_block(jit, &cpu, BASE + 3u) != NULL);
    }
    cpu.windowbase = 14;
    for (int i = 0; i < JIT_HOT_THRESHOLD; i++)
        (void)jit_get_block(jit, &cpu, BASE);
    ASSERT_TRUE(jit_get_block(jit, &cpu, BASE) != NULL);

    for (uint32_t ci = 0; ci < 4; ci++) {
        cpu.ps = ci << 16;
        cpu.windowbase = 14;
        cpu.windowstart = 1u << 14;
        ar_write(&cpu, 1, DATA_BASE + 0x400u);
        cpu.pc = BASE;
        cpu._pc_written = true;
        cpu.running = true;
        uint64_t hooks_before = jit_get_stats(jit)->hook_calls;
        uint64_t insns_before = jit_get_stats(jit)->insns_jitted;
        ASSERT_EQ(xtensa_run(&cpu, 5), 5);
        ASSERT_EQ(cpu.windowbase, (14u + ci) & 15u);
        ASSERT_EQ(cpu.pc, BASE + 11u);
        ASSERT_EQ64(jit_get_stats(jit)->insns_jitted - insns_before, 5u);
        ASSERT_EQ64(jit_get_stats(jit)->hook_calls - hooks_before, 1u);
    }

    jit_destroy(jit);
    teardown(&cpu);
}

TEST(test_jit_rotw_flushes_old_mapping_and_wraps) {
    xtensa_cpu_t cpu;
    setup(&cpu);
    cpu.real_window_vectors = true;
    cpu.ps = 1u << 18; /* WOE */
    cpu.windowbase = 14;
    cpu.windowstart = 1u << 14;
    ar_write(&cpu, 2, 41u);
    put_insn2(&cpu, BASE, narrow(0xB, 2, 2, 1)); /* ADDI.N a2, a2, 1 */
    put_insn3(&cpu, BASE + 2u, jit_rotw_insn(3));
    test_block_differential(&cpu, 2, "rotw_native_wrap");
    teardown(&cpu);
}

TEST(test_jit_rotw_legacy_without_woe_is_native) {
    xtensa_cpu_t cpu;
    setup(&cpu);
    cpu.real_window_vectors = false;
    cpu.ps = 0;
    cpu.windowbase = 1;
    cpu.windowstart = 1u << 1;
    put_insn2(&cpu, BASE, narrow(0xD, 15, 0, 3)); /* NOP.N */
    put_insn3(&cpu, BASE + 2u, jit_rotw_insn(-2));
    test_block_differential(&cpu, 2, "rotw_legacy_woe_off");
    teardown(&cpu);
}

TEST(test_jit_rotw_legacy_woe_falls_back) {
    xtensa_cpu_t cpu;
    setup(&cpu);
    cpu.real_window_vectors = false;
    cpu.ps = 1u << 18; /* legacy WOE requires synth_overflow_check() */
    cpu.windowbase = 2;
    cpu.windowstart = 1u << 2;
    ar_write(&cpu, 2, 9u);
    put_insn2(&cpu, BASE, narrow(0xB, 2, 2, 1)); /* dirty native prefix */
    put_insn3(&cpu, BASE + 2u, jit_rotw_insn(-1));

    jit_state_t *jit = jit_init();
    ASSERT_TRUE(jit != NULL);
    for (int i = 0; i < JIT_HOT_THRESHOLD; i++)
        (void)jit_get_block(jit, &cpu, BASE);
    jit_block_fn fn = jit_get_block(jit, &cpu, BASE);
    ASSERT_TRUE(fn != NULL);

    ASSERT_EQ(fn(&cpu), 1);
    ASSERT_EQ(cpu.pc, BASE + 2u);
    ASSERT_EQ(cpu.windowbase, 2u);
    ASSERT_EQ(cpu.ar[2 * 4 + 2], 10u);

    jit_destroy(jit);
    teardown(&cpu);
}

TEST(test_jit_rotw_chains_under_destination_window) {
    xtensa_cpu_t cpu;
    setup(&cpu);
    cpu.real_window_vectors = true;
    cpu.ps = 1u << 18;
    cpu.windowbase = 0;
    cpu.windowstart = 1u;
    put_insn3(&cpu, BASE, jit_rotw_insn(1));
    for (unsigned i = 0; i < 4; i++)
        put_insn2(&cpu, BASE + 3u + i * 2u,
                  narrow(0xD, 15, 0, 3)); /* NOP.N */

    jit_state_t *jit = jit_init();
    ASSERT_TRUE(jit != NULL);
    jit_install_hook(jit, &cpu);

    cpu.windowbase = 1;
    for (int i = 0; i < JIT_HOT_THRESHOLD; i++)
        (void)jit_get_block(jit, &cpu, BASE + 3u);
    ASSERT_TRUE(jit_get_block(jit, &cpu, BASE + 3u) != NULL);

    cpu.windowbase = 0;
    for (int i = 0; i < JIT_HOT_THRESHOLD; i++)
        (void)jit_get_block(jit, &cpu, BASE);
    ASSERT_TRUE(jit_get_block(jit, &cpu, BASE) != NULL);
    ASSERT_TRUE(jit_get_stats(jit)->chains_patched > 0);

    uint64_t hooks_before = jit_get_stats(jit)->hook_calls;
    cpu.pc = BASE;
    cpu._pc_written = true;
    cpu.running = true;
    ASSERT_EQ(xtensa_run(&cpu, 5), 5);
    ASSERT_EQ(cpu.windowbase, 1u);
    ASSERT_EQ(cpu.pc, BASE + 11u);
    ASSERT_EQ64(jit_get_stats(jit)->hook_calls - hooks_before, 1u);

    jit_destroy(jit);
    teardown(&cpu);
}

typedef struct {
    unsigned calls;
    uint32_t last_pc;
} jit_entry_spy_t;

typedef struct {
    uint32_t exact_pc;
    unsigned queries;
} jit_exact_hook_t;

static bool jit_exact_hook_contains(uint32_t pc, void *ctx) {
    jit_exact_hook_t *hook = ctx;
    hook->queries++;
    return pc == hook->exact_pc;
}

TEST(test_jit_exact_hook_query_distinguishes_bitmap_collisions) {
    xtensa_cpu_t cpu;
    setup(&cpu);

    const uint32_t false_positive_pc = BASE;
    const uint32_t exact_hook_pc = BASE + 0x100u;
    put_insn2(&cpu, false_positive_pc, narrow(0xD, 15, 0, 3));
    put_insn2(&cpu, false_positive_pc + 2u, narrow(0xD, 15, 0, 3));
    put_insn2(&cpu, exact_hook_pc, narrow(0xD, 15, 0, 3));

    uint64_t *hook_bitmap = calloc(HOOK_BITMAP_WORDS, sizeof(*hook_bitmap));
    ASSERT_TRUE(hook_bitmap != NULL);
    if (!hook_bitmap) {
        teardown(&cpu);
        return;
    }
    uint32_t false_bit = (false_positive_pc >> 2) & (HOOK_BITMAP_BITS - 1);
    uint32_t exact_bit = (exact_hook_pc >> 2) & (HOOK_BITMAP_BITS - 1);
    hook_bitmap[false_bit / 64] |= 1ULL << (false_bit & 63);
    hook_bitmap[exact_bit / 64] |= 1ULL << (exact_bit & 63);

    jit_exact_hook_t exact = {.exact_pc = exact_hook_pc};
    cpu.pc_hook_bitmap = hook_bitmap;
    cpu.pc_hook_contains = jit_exact_hook_contains;
    cpu.pc_hook_contains_ctx = &exact;

    jit_state_t *jit = jit_init();
    ASSERT_TRUE(jit != NULL);
    if (!jit) {
        free(hook_bitmap);
        teardown(&cpu);
        return;
    }
    jit_install_hook(jit, &cpu);

    for (int i = 0; i < JIT_HOT_THRESHOLD; i++)
        (void)jit_get_block(jit, &cpu, false_positive_pc);
    ASSERT_TRUE(jit_get_block(jit, &cpu, false_positive_pc) != NULL);

    for (int i = 0; i < JIT_HOT_THRESHOLD; i++)
        (void)jit_get_block(jit, &cpu, exact_hook_pc);
    ASSERT_TRUE(jit_get_block(jit, &cpu, exact_hook_pc) == NULL);
    ASSERT_TRUE(exact.queries > 0);

    jit_destroy(jit);
    free(hook_bitmap);
    teardown(&cpu);
}

static int jit_entry_spy_hook(xtensa_cpu_t *cpu, uint32_t pc, void *ctx) {
    (void)cpu;
    jit_entry_spy_t *spy = ctx;
    if (pc == BASE || pc == BASE + 3u) {
        spy->calls++;
        spy->last_pc = pc;
    }
    return 0;  /* Observe the entry, then execute the guest instruction. */
}

TEST(test_jit_entry_fallthrough_does_not_repeat_original_hook) {
    xtensa_cpu_t cpu;
    setup(&cpu);
    cpu.ps = 2u << 16;  /* CALLINC=2 */
    cpu.windowbase = 0;
    cpu.windowstart = 1u;
    ar_write(&cpu, 1, DATA_BASE + 0x400u);
    put_insn3(&cpu, BASE, jit_entry_insn(1, 32));
    put_insn2(&cpu, BASE + 3u, narrow(0xD, 15, 0, 3));  /* NOP.N */

    uint64_t *hook_bitmap = calloc(HOOK_BITMAP_WORDS, sizeof(*hook_bitmap));
    ASSERT_TRUE(hook_bitmap != NULL);
    if (!hook_bitmap) {
        teardown(&cpu);
        return;
    }
    uint32_t bit = (BASE >> 2) & (HOOK_BITMAP_BITS - 1);
    hook_bitmap[bit / 64] |= 1ULL << (bit & 63);

    jit_entry_spy_t spy = {0};
    cpu.pc_hook = jit_entry_spy_hook;
    cpu.pc_hook_ctx = &spy;
    cpu.pc_hook_bitmap = hook_bitmap;

    jit_state_t *jit = jit_init();
    ASSERT_TRUE(jit != NULL);
    if (!jit) {
        free(hook_bitmap);
        teardown(&cpu);
        return;
    }
    jit_install_hook(jit, &cpu);

    cpu.running = true;
    cpu._pc_written = true;
    ASSERT_EQ(xtensa_run(&cpu, 2), 2);
    ASSERT_EQ(spy.calls, 1);
    ASSERT_EQ(spy.last_pc, BASE);
    ASSERT_FALSE(cpu.jit_fallthrough_dispatch);

    jit_destroy(jit);
    free(hook_bitmap);
    teardown(&cpu);
}

static uint32_t jit_retw_insn(void) {
    return rrr(0, 0, 0, 0, (2 << 2) | 1);
}

static uint16_t jit_retw_n_insn(void) {
    return narrow(0xD, 15, 0, 1);
}

static void jit_put_three_nops(xtensa_cpu_t *cpu) {
    put_insn2(cpu, BASE,     narrow(0xD, 15, 0, 3));
    put_insn2(cpu, BASE + 2, narrow(0xD, 15, 0, 3));
    put_insn2(cpu, BASE + 4, narrow(0xD, 15, 0, 3));
}

TEST(test_jit_call4_windowed) {
    xtensa_cpu_t cpu;
    setup(&cpu);
    cpu.ps = (1u << 18) | (1u << 4); /* WOE + EXCM */
    cpu.windowbase = 2;
    cpu.windowstart = 1u << 2;
    jit_put_three_nops(&cpu);
    put_insn3(&cpu, BASE + 6, jit_calln_insn(1, 0));
    test_block_differential(&cpu, 4, "call4_windowed");
    teardown(&cpu);
}

TEST(test_jit_call0_full_return_address) {
    xtensa_cpu_t cpu;
    setup(&cpu);
    cpu.ps = (1u << 18) | (1u << 4) | (2u << 16);
    cpu.windowbase = 2;
    cpu.windowstart = 1u << 2;
    jit_put_three_nops(&cpu);
    put_insn3(&cpu, BASE + 6, jit_calln_insn(0, 0));
    test_block_differential(&cpu, 4, "call0_full_return_address");
    teardown(&cpu);
}

static void jit_callx_chain_case(int nn, int source) {
    xtensa_cpu_t cpu;
    setup(&cpu);
    const uint32_t target = BASE + 0x80u;

    put_insn3(&cpu, BASE, jit_callx_insn(nn, source));
    for (unsigned i = 0; i < 4u; i++)
        put_insn2(&cpu, target + i * 2u,
                  narrow(0xD, 15, 0, 3)); /* NOP.N */
    put_insn2(&cpu, target + 8u,
              narrow(0xD, 15, 0, 2));     /* ILL.N ends the block */

    jit_state_t *jit = jit_init();
    ASSERT_TRUE(jit != NULL);
    jit_install_hook(jit, &cpu);

    cpu.windowbase = 2u;
    for (int i = 0; i < JIT_HOT_THRESHOLD; i++)
        (void)jit_get_block(jit, &cpu, target);
    ASSERT_TRUE(jit_get_block(jit, &cpu, target) != NULL);
    for (int i = 0; i < JIT_HOT_THRESHOLD; i++)
        (void)jit_get_block(jit, &cpu, BASE);
    ASSERT_TRUE(jit_get_block(jit, &cpu, BASE) != NULL);

    cpu.ps = (1u << 18) | (1u << 4); /* WOE + EXCM */
    cpu.windowbase = 2u;
    cpu.windowstart = 1u << 2;
    ar_write(&cpu, source, target);
    cpu.pc = BASE;
    cpu._pc_written = true;
    cpu.running = true;

    uint64_t hooks_before = jit_get_stats(jit)->hook_calls;
    uint64_t insns_before = jit_get_stats(jit)->insns_jitted;
    ASSERT_EQ(xtensa_run(&cpu, 5), 5);
    ASSERT_EQ(cpu.pc, target + 8u);
    ASSERT_EQ(cpu.windowbase, 2u);
    ASSERT_EQ(XT_PS_CALLINC(cpu.ps), (uint32_t)nn);
    ASSERT_EQ(ar_read(&cpu, nn * 4),
              nn == 0 ? BASE + 3u
                      : ((uint32_t)nn << 30) |
                        ((BASE + 3u) & 0x3FFFFFFFu));
    ASSERT_EQ64(jit_get_stats(jit)->insns_jitted - insns_before, 5u);
    ASSERT_EQ64(jit_get_stats(jit)->hook_calls - hooks_before, 1u);

    jit_destroy(jit);
    teardown(&cpu);
}

TEST(test_jit_callx_chains_to_runtime_callee) {
    /* CALLX0 covers the call0 ABI; CALLX8 also makes the source and link
     * register alias, the ordinary windowed function-pointer call shape. */
    jit_callx_chain_case(0, 2);
    jit_callx_chain_case(2, 8);
}

TEST(test_jit_dynamic_call_stops_at_timer_deadline) {
    xtensa_cpu_t cpu;
    setup(&cpu);
    const uint32_t target = BASE + 0x80u;

    put_insn3(&cpu, BASE, jit_callx_insn(2, 8));
    put_insn2(&cpu, target, narrow(0xD, 15, 0, 3)); /* NOP.N */
    put_insn2(&cpu, target + 2u,
              narrow(0xD, 15, 0, 2)); /* ILL.N ends the block */

    jit_state_t *jit = jit_init();
    ASSERT_TRUE(jit != NULL);
    cpu.windowbase = 2u;
    for (int i = 0; i < JIT_HOT_THRESHOLD; i++)
        (void)jit_get_block(jit, &cpu, target);
    ASSERT_TRUE(jit_get_block(jit, &cpu, target) != NULL);
    for (int i = 0; i < JIT_HOT_THRESHOLD; i++)
        (void)jit_get_block(jit, &cpu, BASE);
    jit_block_fn source_fn = jit_get_block(jit, &cpu, BASE);
    ASSERT_TRUE(source_fn != NULL);

    cpu.ps = (1u << 18) | (1u << 4); /* WOE + EXCM */
    cpu.windowbase = 2u;
    cpu.windowstart = 1u << 2;
    ar_write(&cpu, 8, target);
    cpu.pc = BASE;
    cpu.ccount = 100u;
    cpu.next_timer_event = 101u;
    cpu.jit_chain_limit = 1u; /* dispatcher-computed event horizon */

    /* CALLX itself reaches the deadline. Its dynamically resolved callee must
     * remain at the published target until the dispatcher services the timer. */
    ASSERT_EQ(source_fn(&cpu), 1);
    ASSERT_EQ(cpu.pc, target);
    ASSERT_EQ(cpu.windowbase, 2u);

    jit_destroy(jit);
    teardown(&cpu);
}

TEST(test_jit_windowed_indirect_call_round_trip) {
    xtensa_cpu_t cpu;
    setup(&cpu);
    const uint32_t callee = BASE + 0x80u;
    const uint32_t body = callee + 3u;
    const uint32_t continuation = BASE + 3u;

    /* Exercise the complete windowed indirect-call path as three separately
     * compiled blocks: CALLX8 selects the callee at runtime, ENTRY rotates to
     * its register window, and RETW resolves the runtime continuation back in
     * the caller. This is the architectural cycle used by C++ virtual calls
     * and function pointers, independent of any particular firmware image. */
    put_insn3(&cpu, BASE, jit_callx_insn(2, 8));
    for (unsigned i = 0; i < 4u; i++)
        put_insn2(&cpu, continuation + i * 2u,
                  narrow(0xD, 15, 0, 3)); /* NOP.N */
    put_insn2(&cpu, continuation + 8u,
              narrow(0xD, 15, 0, 2));     /* ILL.N ends the block */

    put_insn3(&cpu, callee, jit_entry_insn(1, 32));
    put_insn2(&cpu, body, narrow(0xB, 2, 2, 1)); /* ADDI.N a2, a2, 1 */
    put_insn2(&cpu, body + 2u, jit_retw_n_insn());

    jit_state_t *jit = jit_init();
    ASSERT_TRUE(jit != NULL);
    jit_install_hook(jit, &cpu);

    /* Compile every destination under the window in which it executes, then
     * restore the caller state before entering the native chain. */
    cpu.windowbase = 2u;
    for (int i = 0; i < JIT_HOT_THRESHOLD; i++)
        (void)jit_get_block(jit, &cpu, continuation);
    ASSERT_TRUE(jit_get_block(jit, &cpu, continuation) != NULL);

    cpu.windowbase = 4u;
    for (int i = 0; i < JIT_HOT_THRESHOLD; i++)
        (void)jit_get_block(jit, &cpu, body);
    ASSERT_TRUE(jit_get_block(jit, &cpu, body) != NULL);

    cpu.windowbase = 2u;
    for (int i = 0; i < JIT_HOT_THRESHOLD; i++)
        (void)jit_get_block(jit, &cpu, callee);
    ASSERT_TRUE(jit_get_block(jit, &cpu, callee) != NULL);
    for (int i = 0; i < JIT_HOT_THRESHOLD; i++)
        (void)jit_get_block(jit, &cpu, BASE);
    ASSERT_TRUE(jit_get_block(jit, &cpu, BASE) != NULL);

    cpu.ps = (1u << 18) | (1u << 4); /* WOE + EXCM */
    cpu.windowbase = 2u;
    cpu.windowstart = 1u << 2;
    ar_write(&cpu, 1, DATA_BASE + 0x800u);
    ar_write(&cpu, 2, 41u);
    ar_write(&cpu, 8, callee); /* CALLX8 target aliases its link register. */
    cpu.ar[4u * 4u + 2u] = 41u;
    cpu.pc = BASE;
    cpu._pc_written = true;
    cpu.running = true;

    uint64_t hooks_before = jit_get_stats(jit)->hook_calls;
    uint64_t insns_before = jit_get_stats(jit)->insns_jitted;
    ASSERT_EQ(xtensa_run(&cpu, 8), 8);
    ASSERT_EQ(cpu.pc, continuation + 8u);
    ASSERT_EQ(cpu.windowbase, 2u);
    ASSERT_EQ(cpu.windowstart, 1u << 2);
    ASSERT_EQ(ar_read(&cpu, 2), 41u);
    ASSERT_EQ(cpu.ar[4u * 4u + 2u], 42u);
    ASSERT_EQ64(jit_get_stats(jit)->insns_jitted - insns_before, 8u);
    ASSERT_EQ64(jit_get_stats(jit)->hook_calls - hooks_before, 1u);

    jit_destroy(jit);
    teardown(&cpu);
}

TEST(test_jit_entry_windowed) {
    xtensa_cpu_t cpu;
    setup(&cpu);
    cpu.ps = (1u << 18) | (1u << 4) | (1u << 16); /* WOE + EXCM + CALLINC=1 */
    cpu.windowbase = 2;
    cpu.windowstart = 1u << 2;
    ar_write(&cpu, 1, BASE + 0x2000);
    ar_write(&cpu, 4, (1u << 30) | ((BASE + 0x80) & 0x3FFFFFFFu));
    jit_put_three_nops(&cpu);
    put_insn3(&cpu, BASE + 6, jit_entry_insn(1, 32));
    test_block_differential(&cpu, 4, "entry_windowed");
    teardown(&cpu);
}

/* ENTRY's collision prefix comes from runtime CALLINC, not the value seen
 * when the block was compiled. A live window beyond that prefix is an
 * unrelated caller frame and must not force the prologue back through the
 * interpreter once architectural window vectors are active. */
TEST(test_jit_entry_ignores_unrelated_live_window) {
    xtensa_cpu_t cpu;
    setup(&cpu);
    cpu.real_window_vectors = true;
    cpu.vecbase = BASE;
    cpu.windowbase = 2;
    cpu.windowstart = 1u << 2;
    cpu.ps = (1u << 18) | (2u << 16); /* Compile while CALLINC=2. */
    ar_write(&cpu, 1, BASE + 0x2000);
    jit_put_three_nops(&cpu);
    put_insn3(&cpu, BASE + 6, jit_entry_insn(1, 32));

    jit_state_t *jit = jit_init();
    ASSERT_TRUE(jit != NULL);
    for (int i = 0; i < JIT_HOT_THRESHOLD; i++)
        (void)jit_get_block(jit, &cpu, BASE);
    jit_block_fn fn = jit_get_block(jit, &cpu, BASE);
    ASSERT_TRUE(fn != NULL);

    /* Runtime CALLINC=1 reaches only bit 3. Bit 4 may stay live. */
    cpu.ps = (1u << 18) | (1u << 16);
    cpu.windowstart = (1u << 2) | (1u << 4);
    cpu.pc = BASE;
    ASSERT_EQ(fn(&cpu), 4);
    ASSERT_EQ(cpu.pc, BASE + 9u);
    ASSERT_EQ(cpu.windowbase, 3u);
    ASSERT_EQ(cpu.windowstart, (1u << 2) | (1u << 3) | (1u << 4));
    ASSERT_EQ(ar_read(&cpu, 1), BASE + 0x2000u - 32u);

    /* Reuse that same compiled block with a larger CALLINC. The now-reached
     * bit 5 is a real collision, so compiled ENTRY must take the architectural
     * transition without modifying its destination window first. */
    cpu.ps = (1u << 18) | (3u << 16);
    cpu.windowbase = 2;
    cpu.windowstart = (1u << 2) | (1u << 5);
    cpu.pc = BASE;
    ar_write(&cpu, 1, BASE + 0x2000);
    ASSERT_EQ(fn(&cpu), 4);
    ASSERT_EQ(cpu.pc, BASE + VECOFS_WINDOW_OVERFLOW12);
    ASSERT_EQ(cpu.epc[0], BASE + 6u);
    ASSERT_EQ(cpu.windowbase, 5u);
    ASSERT_EQ(XT_PS_OWB(cpu.ps), 2u);
    ASSERT_TRUE(XT_PS_EXCM(cpu.ps));
    ASSERT_EQ(cpu.windowstart, (1u << 2) | (1u << 5));

    /* The pre-vector compatibility path intentionally spills farther ahead
     * than architectural ENTRY. Keep routing that state through the exact C
     * implementation rather than silently changing startup behavior. */
    cpu.real_window_vectors = false;
    cpu.ps = (1u << 18) | (1u << 16);
    cpu.windowbase = 2;
    cpu.windowstart = (1u << 2) | (1u << 4);
    cpu.pc = BASE;
    ASSERT_EQ(fn(&cpu), 3);
    ASSERT_EQ(cpu.pc, BASE + 6u);
    ASSERT_EQ(cpu.windowbase, 2u);
    ASSERT_EQ(cpu.windowstart, (1u << 2) | (1u << 4));

    jit_destroy(jit);
    teardown(&cpu);
}

TEST(test_jit_retw_windowed) {
    xtensa_cpu_t cpu;
    setup(&cpu);
    cpu.ps = (1u << 18) | (1u << 4); /* WOE + EXCM */
    cpu.windowbase = 3;
    cpu.windowstart = (1u << 2) | (1u << 3);
    ar_write(&cpu, 0, (1u << 30) | ((BASE + 0x80) & 0x3FFFFFFFu));
    jit_put_three_nops(&cpu);
    put_insn3(&cpu, BASE + 6, jit_retw_insn());
    test_block_differential(&cpu, 4, "retw_windowed");
    teardown(&cpu);
}

TEST(test_jit_retw_n_windowed) {
    xtensa_cpu_t cpu;
    setup(&cpu);
    cpu.ps = (1u << 18) | (1u << 4); /* WOE + EXCM */
    cpu.windowbase = 3;
    cpu.windowstart = (1u << 2) | (1u << 3);
    ar_write(&cpu, 0, (1u << 30) | ((BASE + 0x80) & 0x3FFFFFFFu));
    jit_put_three_nops(&cpu);
    put_insn2(&cpu, BASE + 6, jit_retw_n_insn());
    test_block_differential(&cpu, 4, "retw_n_windowed");
    teardown(&cpu);
}

TEST(test_jit_retw_chains_to_runtime_caller) {
    xtensa_cpu_t cpu;
    setup(&cpu);
    const uint32_t caller = BASE + 0x80u;

    /* A RETW target is carried in the callee's a0 rather than in the opcode.
     * Compile the continuation under its caller window first, then prove the
     * callee can resolve and enter it without a second C hook dispatch. */
    put_insn3(&cpu, BASE, jit_retw_insn());
    for (unsigned i = 0; i < 4u; i++)
        put_insn2(&cpu, caller + i * 2u,
                  narrow(0xD, 15, 0, 3)); /* NOP.N */
    put_insn2(&cpu, caller + 8u,
              narrow(0xD, 15, 0, 2));     /* ILL.N ends the block */

    jit_state_t *jit = jit_init();
    ASSERT_TRUE(jit != NULL);
    jit_install_hook(jit, &cpu);

    cpu.windowbase = 2u;
    for (int i = 0; i < JIT_HOT_THRESHOLD; i++)
        (void)jit_get_block(jit, &cpu, caller);
    ASSERT_TRUE(jit_get_block(jit, &cpu, caller) != NULL);

    cpu.windowbase = 3u;
    for (int i = 0; i < JIT_HOT_THRESHOLD; i++)
        (void)jit_get_block(jit, &cpu, BASE);
    ASSERT_TRUE(jit_get_block(jit, &cpu, BASE) != NULL);

    cpu.ps = (1u << 18) | (1u << 4); /* WOE + EXCM */
    cpu.windowbase = 3u;
    cpu.windowstart = (1u << 2) | (1u << 3);
    ar_write(&cpu, 0,
             (1u << 30) | (caller & 0x3FFFFFFFu)); /* call4 caller */
    cpu.pc = BASE;
    cpu._pc_written = true;
    cpu.running = true;

    uint64_t hooks_before = jit_get_stats(jit)->hook_calls;
    uint64_t insns_before = jit_get_stats(jit)->insns_jitted;
    ASSERT_EQ(xtensa_run(&cpu, 5), 5);
    ASSERT_EQ(cpu.pc, caller + 8u);
    ASSERT_EQ(cpu.windowbase, 2u);
    ASSERT_EQ(cpu.windowstart, 1u << 2);
    ASSERT_EQ64(jit_get_stats(jit)->insns_jitted - insns_before, 5u);
    ASSERT_EQ64(jit_get_stats(jit)->hook_calls - hooks_before, 1u);

    jit_destroy(jit);
    teardown(&cpu);
}

TEST(test_jit_retw_does_not_chain_into_live_loop) {
    xtensa_cpu_t cpu;
    setup(&cpu);
    const uint32_t caller = BASE + 0x80u;
    const uint32_t lend = caller + 8u;

    put_insn3(&cpu, BASE, jit_retw_insn());
    for (unsigned i = 0; i < 4u; i++)
        put_insn2(&cpu, caller + i * 2u,
                  narrow(0xD, 15, 0, 3)); /* NOP.N */

    jit_state_t *jit = jit_init();
    ASSERT_TRUE(jit != NULL);

    /* Compile a loop-bounded target with a valid chain entry. RETW itself is
     * outside the loop, so its block is the ordinary unbounded variant; the
     * runtime resolver is responsible for rejecting the incompatible target. */
    cpu.lbeg = caller;
    cpu.lend = lend;
    cpu.lcount = 3u;
    cpu.windowbase = 2u;
    for (int i = 0; i < JIT_HOT_THRESHOLD; i++)
        (void)jit_get_block(jit, &cpu, caller);
    ASSERT_TRUE(jit_get_block(jit, &cpu, caller) != NULL);

    cpu.windowbase = 3u;
    for (int i = 0; i < JIT_HOT_THRESHOLD; i++)
        (void)jit_get_block(jit, &cpu, BASE);
    jit_block_fn retw = jit_get_block(jit, &cpu, BASE);
    ASSERT_TRUE(retw != NULL);

    cpu.ps = (1u << 18) | (1u << 4); /* WOE + EXCM */
    cpu.windowbase = 3u;
    cpu.windowstart = (1u << 2) | (1u << 3);
    ar_write(&cpu, 0,
             (1u << 30) | (caller & 0x3FFFFFFFu)); /* call4 caller */
    cpu.pc = BASE;

    ASSERT_EQ(retw(&cpu), 1);
    ASSERT_EQ(cpu.pc, caller);
    ASSERT_EQ(cpu.windowbase, 2u);
    ASSERT_EQ(cpu.lcount, 3u);

    jit_destroy(jit);
    teardown(&cpu);
}

TEST(test_jit_retw_underflow_raises_guest_vector) {
    xtensa_cpu_t expected, actual;
    setup(&expected);
    expected.real_window_vectors = true;
    expected.vecbase = BASE;
    expected.ps = 1u << 18; /* WOE, outside an exception */
    expected.windowbase = 3u;
    expected.windowstart = 1u << 3; /* call4 caller at WB=2 is spilled */
    expected.window_callsize[3] = 1u;
    ar_write(&expected, 0,
             (1u << 30) | ((BASE + 0x200u) & 0x3FFFFFFFu));
    ar_write(&expected, 1, DATA_BASE + 0xC00u);
    put_insn3(&expected, BASE, jit_retw_insn());
    memcpy(&actual, &expected, sizeof(actual));

    ASSERT_EQ(xtensa_step(&expected), 0);

    jit_state_t *jit = jit_init();
    ASSERT_TRUE(jit != NULL);
    if (!jit) {
        teardown(&expected);
        return;
    }
    for (int i = 0; i < JIT_HOT_THRESHOLD; i++)
        (void)jit_get_block(jit, &actual, BASE);
    jit_block_fn fn = jit_get_block(jit, &actual, BASE);
    ASSERT_TRUE(fn != NULL);
    ASSERT_EQ(fn(&actual), 1);

    ASSERT_EQ(compare_state(&expected, &actual, "retw_underflow_vector"), 0);
    ASSERT_EQ(actual.pc, BASE + VECOFS_WINDOW_UNDERFLOW4);
    ASSERT_EQ(actual.epc[0], BASE);
    ASSERT_EQ(actual.windowbase, 2u);
    ASSERT_EQ(XT_PS_OWB(actual.ps), 3u);
    ASSERT_TRUE(XT_PS_EXCM(actual.ps));

    jit_destroy(jit);
    teardown(&expected);
}

TEST(test_jit_retw_tail_call_fallback) {
    xtensa_cpu_t cpu;
    setup(&cpu);
    cpu.ps = (1u << 18) | (1u << 4); /* WOE + EXCM */
    cpu.windowbase = 3;
    cpu.windowstart = (1u << 2) | (1u << 3);
    cpu.window_callsize[3] = 1;
    ar_write(&cpu, 0, (BASE + 0x80) & 0x3FFFFFFFu); /* no callsize bits */
    jit_put_three_nops(&cpu);
    put_insn2(&cpu, BASE + 6, narrow(0xD, 15, 0, 3));
    put_insn3(&cpu, BASE + 8, jit_retw_insn());
    test_run_differential(&cpu, 5, "retw_tail_call_fallback");
    teardown(&cpu);
}

TEST(test_jit_entry_overflow_fallback) {
    xtensa_cpu_t cpu;
    setup(&cpu);
    cpu.ps = (1u << 18) | (1u << 4) | (1u << 16); /* WOE + EXCM + CALLINC=1 */
    cpu.windowbase = 2;
    cpu.windowstart = (1u << 2) | (1u << 3);
    ar_write(&cpu, 1, BASE + 0x2000);
    /* Values belonging to the endangered physical window. */
    cpu.ar[3 * 4 + 0] = (1u << 30) | ((BASE + 0x100) & 0x3FFFFFFFu);
    cpu.ar[3 * 4 + 1] = BASE + 0x2100;
    cpu.ar[3 * 4 + 2] = 0xA5A5A5A5u;
    cpu.ar[3 * 4 + 3] = 0x5A5A5A5Au;
    jit_put_three_nops(&cpu);
    put_insn2(&cpu, BASE + 6, narrow(0xD, 15, 0, 3));
    put_insn3(&cpu, BASE + 8, jit_entry_insn(1, 32));
    test_run_differential(&cpu, 5, "entry_overflow_fallback");
    teardown(&cpu);
}

TEST(test_jit_window_overflow_vector_is_native) {
    xtensa_cpu_t cpu;
    setup(&cpu);
    const uint32_t stack = DATA_BASE + 0x800u;
    const uint32_t target = BASE + 0x180u;
    static const uint32_t values[4] = {
        0x10203040u, 0x55667788u, 0x89ABCDEFu, 0xDEADC0DEu,
    };

    cpu.ps = (1u << 18) | (1u << 4) | (1u << 8); /* WOE, EXCM, OWB=1 */
    cpu.windowbase = 3;
    cpu.windowstart = (1u << 1) | (1u << 3);
    cpu.epc[0] = target;
    ar_write(&cpu, 8, stack);
    for (int i = 0; i < 4; i++) {
        ar_write(&cpu, i, values[i]);
        mem_write32(cpu.mem, stack - 64u + (uint32_t)i * 4u, 0);
        put_insn3(&cpu, BASE + (uint32_t)i * 3u,
                  jit_s32e_insn(i, 8, -64 + i * 4));
    }
    put_insn3(&cpu, BASE + 12u, jit_rfwo_insn());

    jit_state_t *jit = jit_init();
    ASSERT_TRUE(jit != NULL);
    for (int i = 0; i < JIT_HOT_THRESHOLD; i++)
        (void)jit_get_block(jit, &cpu, BASE);
    jit_block_fn fn = jit_get_block(jit, &cpu, BASE);
    ASSERT_TRUE(fn != NULL);
    ASSERT_EQ(fn(&cpu), 5);

    for (int i = 0; i < 4; i++)
        ASSERT_EQ(mem_read32(cpu.mem,
                            stack - 64u + (uint32_t)i * 4u), values[i]);
    ASSERT_EQ(cpu.ps, (1u << 18) | (1u << 8));
    ASSERT_EQ(cpu.windowbase, 1);
    ASSERT_EQ(cpu.windowstart, 1u << 1);
    ASSERT_EQ(cpu.pc, target);
    ASSERT_TRUE(cpu._pc_written);
    ASSERT_TRUE(cpu.irq_check);

    jit_destroy(jit);
    teardown(&cpu);
}

TEST(test_jit_window_underflow_vector_is_native) {
    xtensa_cpu_t cpu;
    setup(&cpu);
    const uint32_t stack = DATA_BASE + 0xA00u;
    const uint32_t target = BASE + 0x1C0u;
    static const uint32_t values[4] = {
        0xCAFEBABEu, 0x0BADF00Du, 0x13579BDFu, 0x2468ACE0u,
    };

    cpu.ps = (1u << 18) | (1u << 4) | (2u << 8); /* WOE, EXCM, OWB=2 */
    cpu.windowbase = 5;
    cpu.windowstart = 1u << 2;
    cpu.epc[0] = target;
    ar_write(&cpu, 9, stack);
    for (int i = 0; i < 4; i++) {
        mem_write32(cpu.mem, stack - 64u + (uint32_t)i * 4u, values[i]);
        put_insn3(&cpu, BASE + (uint32_t)i * 3u,
                  jit_l32e_insn(i, 9, -64 + i * 4));
    }
    put_insn3(&cpu, BASE + 12u, jit_rfwu_insn());

    jit_state_t *jit = jit_init();
    ASSERT_TRUE(jit != NULL);
    for (int i = 0; i < JIT_HOT_THRESHOLD; i++)
        (void)jit_get_block(jit, &cpu, BASE);
    jit_block_fn fn = jit_get_block(jit, &cpu, BASE);
    ASSERT_TRUE(fn != NULL);
    ASSERT_EQ(fn(&cpu), 5);

    for (int i = 0; i < 4; i++)
        ASSERT_EQ(cpu.ar[5 * 4 + i], values[i]);
    ASSERT_EQ(cpu.ps, (1u << 18) | (2u << 8));
    ASSERT_EQ(cpu.windowbase, 2);
    ASSERT_EQ(cpu.windowstart, (1u << 2) | (1u << 5));
    ASSERT_EQ(cpu.pc, target);
    ASSERT_TRUE(cpu._pc_written);
    ASSERT_TRUE(cpu.irq_check);

    jit_destroy(jit);
    teardown(&cpu);
}

/* ===== Test suite runner ===== */

static void run_jit_tests(void) {
    TEST_SUITE("jit");
    RUN_TEST(test_jit_init_destroy);
    RUN_TEST(test_branch_target_ring_is_disabled_without_a_jit_consumer);
    RUN_TEST(test_jit_fallback_stops_at_debug_break_boundary);
    RUN_TEST(test_jit_verify_toggle_recompiles_blocks);
    RUN_TEST(test_jit_verify_keeps_cross_block_chains_disabled);
    RUN_TEST(test_jit_hot_threshold);
    RUN_TEST(test_jit_hash_collision_uses_free_way);
    RUN_TEST(test_jit_one_instruction_straight_line_compiles_when_hot);
    RUN_TEST(test_jit_single_instruction_chain_target_is_native);
    RUN_TEST(test_jit_precompiled_fallthrough_chain_is_not_overwritten);
    RUN_TEST(test_jit_static_chain_stops_at_timer_deadline);
    RUN_TEST(test_jit_precompiled_side_exit_chain_is_not_overwritten);
    RUN_TEST(test_jit_short_backedge_loop_is_native);
    RUN_TEST(test_jit_stale_loop_past_lend_does_not_truncate_block);
    RUN_TEST(test_jit_contended_spinlock_returns_to_scheduler);
    RUN_TEST(test_jit_chained_run_accounts_every_block);
    RUN_TEST(test_jit_loop_backedge_dispatches_native_body);
    RUN_TEST(test_jit_loop_fallthrough_dispatches_first_body);
    RUN_TEST(test_jit_compiles_loop_setup_family);
    RUN_TEST(test_jit_loopnez_uses_dirty_prefix_value);
    RUN_TEST(test_jit_loop_setup_clears_architectural_edge);
    RUN_TEST(test_jit_movsp_architectural_move);
    RUN_TEST(test_jit_movsp_alloca_falls_back_at_opcode);
    RUN_TEST(test_jit_movsp_legacy_mode_falls_back);
    RUN_TEST(test_jit_loop_body_sampled_under_another_loop_still_compiles);
    RUN_TEST(test_jit_self_loop_side_exit_flushes_resident_registers);
    RUN_TEST(test_jit_self_loop_block_declines_a_different_loop);
    RUN_TEST(test_jit_call8_return_address_survives_the_exit_flush);
    RUN_TEST(test_jit_window_guard_checks_only_touched_windows);
    RUN_TEST(test_jit_head_window_collision_raises_guest_vector);
    RUN_TEST(test_jit_window_guard_handles_mid_block_woe_enable);
    RUN_TEST(test_jit_window_collision_falls_back_before_callx8);
    RUN_TEST(test_jit_self_loop_early_exit_flushes_later_writes);
    RUN_TEST(test_jit_block_compiled_outside_a_loop_is_not_reused_inside_one);
    RUN_TEST(test_waiti_time_is_not_counted_as_retired_instructions);
    RUN_TEST(test_jit_encoding_sweep_matches_interpreter);
    RUN_TEST(test_savestate_round_trips_retired_instruction_count);
    RUN_TEST(test_jit_flush);
    RUN_TEST(test_jit_flash_mmu_remap_flushes_upper_window);
    RUN_TEST(test_jit_xtensa_run_counts_guest_instructions);
    RUN_TEST(test_jit_run_does_not_count_delay_ccount_as_instructions);
    RUN_TEST(test_jit_nop);
    RUN_TEST(test_jit_movi);
    RUN_TEST(test_jit_add);
    RUN_TEST(test_jit_sub);
    RUN_TEST(test_jit_and);
    RUN_TEST(test_jit_or);
    RUN_TEST(test_jit_xor);
    RUN_TEST(test_jit_addi);
    RUN_TEST(test_jit_slli);
    RUN_TEST(test_jit_srai);
    RUN_TEST(test_jit_extui);
    RUN_TEST(test_jit_srli);
    RUN_TEST(test_jit_src_funnel);
    RUN_TEST(test_jit_sll_srl_sra);
    RUN_TEST(test_jit_nsa_nsau);
    RUN_TEST(test_jit_mull);
    RUN_TEST(test_jit_mulsh);
    RUN_TEST(test_jit_neg);
    RUN_TEST(test_jit_mov_n);
    RUN_TEST(test_jit_add_n);
    RUN_TEST(test_jit_addi_n);
    RUN_TEST(test_jit_addi_n_minus1);
    RUN_TEST(test_jit_l32i_s32i);
    RUN_TEST(test_jit_s32i);
    RUN_TEST(test_jit_byte_halfword_memory_ops);
    RUN_TEST(test_jit_multi_insn_block);
    RUN_TEST(test_jit_moveqz);
    RUN_TEST(test_jit_moveqz_not_taken);
    RUN_TEST(test_jit_min_max);
    RUN_TEST(test_jit_addx2);
    RUN_TEST(test_jit_rsil);
    RUN_TEST(test_jit_rsr_wsr_sar);
    RUN_TEST(test_jit_rsr_wsr_mac16_state);
    RUN_TEST(test_jit_wsr_br_applies_architectural_mask);
    RUN_TEST(test_jit_rsr_wsr_cpenable);
    RUN_TEST(test_jit_wsr_windowstart_terminates_at_new_guard_context);
    RUN_TEST(test_jit_wsr_windowbase_flushes_old_mapping_before_dispatch);
    RUN_TEST(test_jit_wsr_windowbase_chains_under_runtime_destination);
    RUN_TEST(test_jit_rsr_prid_wsr_ps);
    RUN_TEST(test_jit_rsr_ccount_observes_instruction_position);
    RUN_TEST(test_jit_wsr_ps_rearms_irq_check);
    RUN_TEST(test_jit_wsr_ps_exits_before_pending_irq);
    RUN_TEST(test_jit_rur_wur_user_registers);
    RUN_TEST(test_jit_entry_dispatches_compiled_callee_body);
    RUN_TEST(test_jit_entry_chains_to_runtime_window);
    RUN_TEST(test_jit_rotw_flushes_old_mapping_and_wraps);
    RUN_TEST(test_jit_rotw_legacy_without_woe_is_native);
    RUN_TEST(test_jit_rotw_legacy_woe_falls_back);
    RUN_TEST(test_jit_rotw_chains_under_destination_window);
    RUN_TEST(test_jit_entry_fallthrough_does_not_repeat_original_hook);
    RUN_TEST(test_jit_exact_hook_query_distinguishes_bitmap_collisions);
    RUN_TEST(test_jit_call4_windowed);
    RUN_TEST(test_jit_call0_full_return_address);
    RUN_TEST(test_jit_callx_chains_to_runtime_callee);
    RUN_TEST(test_jit_dynamic_call_stops_at_timer_deadline);
    RUN_TEST(test_jit_windowed_indirect_call_round_trip);
    RUN_TEST(test_jit_entry_windowed);
    RUN_TEST(test_jit_entry_ignores_unrelated_live_window);
    RUN_TEST(test_jit_retw_windowed);
    RUN_TEST(test_jit_retw_n_windowed);
    RUN_TEST(test_jit_retw_chains_to_runtime_caller);
    RUN_TEST(test_jit_retw_does_not_chain_into_live_loop);
    RUN_TEST(test_jit_retw_underflow_raises_guest_vector);
    RUN_TEST(test_jit_retw_tail_call_fallback);
    RUN_TEST(test_jit_entry_overflow_fallback);
    RUN_TEST(test_jit_window_overflow_vector_is_native);
    RUN_TEST(test_jit_window_underflow_vector_is_native);
}

#else /* _MSC_VER */

static void run_jit_tests(void) {
    TEST_SUITE("jit (disabled on MSVC)");
}

#endif /* _MSC_VER */
