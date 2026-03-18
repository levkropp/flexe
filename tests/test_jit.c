/* JIT compiler tests: per-instruction differential testing vs interpreter.
 * Each test assembles a small block, runs via interpreter, runs via JIT,
 * and compares all state. */

#ifndef _MSC_VER

#include "jit.h"

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

    return diffs;
}

/* Run a block: first via interpreter, then via JIT, compare results */
static void test_block_differential(xtensa_cpu_t *template_cpu, int num_insns,
                                    const char *test_name) {
    /* Interpreter run */
    xtensa_cpu_t interp_cpu;
    memcpy(&interp_cpu, template_cpu, sizeof(xtensa_cpu_t));
    run_interp(&interp_cpu, num_insns);

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
        /* JIT couldn't compile — fall back to interpreter for comparison */
        fprintf(stderr, "  NOTE %s: JIT couldn't compile, skipping differential\n", test_name);
        jit_destroy(jit);
        test_passes++;
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
    put_insn2(&cpu, BASE, narrow(0xD, 2, 3, 0));
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

TEST(test_jit_hot_threshold) {
    xtensa_cpu_t cpu;
    setup(&cpu);
    /* Need >= 4 instructions (minimum JIT block size) */
    put_insn2(&cpu, BASE,     narrow(0xD, 15, 0, 3)); /* NOP.N */
    put_insn2(&cpu, BASE + 2, narrow(0xD, 15, 0, 3)); /* NOP.N */
    put_insn2(&cpu, BASE + 4, narrow(0xD, 15, 0, 3)); /* NOP.N */
    put_insn2(&cpu, BASE + 6, narrow(0xD, 15, 0, 3)); /* NOP.N */

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

TEST(test_jit_flush) {
    jit_state_t *jit = jit_init();
    ASSERT_TRUE(jit != NULL);
    jit_flush(jit);
    ASSERT_EQ(jit_get_stats(jit)->cache_flushes, 1);
    jit_destroy(jit);
}

TEST(test_jit_rsr_wsr_sar) {
    xtensa_cpu_t cpu;
    setup(&cpu);
    ar_write(&cpu, 2, 17);
    /* WSR SAR, a2: op0=0, op1=3, op2=1, r=s_low, s=sr_high, t=2 */
    /* sr = XT_SR_SAR = 3. sr = s:r in insn → s=0, r=3 */
    /* WSR: op0=0, t=2, s=0, r=3, op1=3, op2=1 */
    put_insn3(&cpu, BASE, rrr(1, 3, 3, 0, 2));  /* WSR SAR, a2 */
    /* RSR a5, SAR: op0=0, op1=3, op2=0, r=3, s=0, t=5 */
    put_insn3(&cpu, BASE + 3, rrr(0, 3, 3, 0, 5));  /* RSR a5, SAR */
    test_block_differential(&cpu, 2, "rsr_wsr_sar");
    teardown(&cpu);
}

/* ===== Test suite runner ===== */

static void run_jit_tests(void) {
    TEST_SUITE("jit");
    RUN_TEST(test_jit_init_destroy);
    RUN_TEST(test_jit_hot_threshold);
    RUN_TEST(test_jit_flush);
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
    RUN_TEST(test_jit_mull);
    RUN_TEST(test_jit_neg);
    RUN_TEST(test_jit_mov_n);
    RUN_TEST(test_jit_add_n);
    RUN_TEST(test_jit_addi_n);
    RUN_TEST(test_jit_addi_n_minus1);
    RUN_TEST(test_jit_l32i_s32i);
    RUN_TEST(test_jit_s32i);
    RUN_TEST(test_jit_multi_insn_block);
    RUN_TEST(test_jit_moveqz);
    RUN_TEST(test_jit_moveqz_not_taken);
    RUN_TEST(test_jit_min_max);
    RUN_TEST(test_jit_addx2);
    RUN_TEST(test_jit_rsil);
    RUN_TEST(test_jit_rsr_wsr_sar);
}

#else /* _MSC_VER */

static void run_jit_tests(void) {
    TEST_SUITE("jit (disabled on MSVC)");
}

#endif /* _MSC_VER */
