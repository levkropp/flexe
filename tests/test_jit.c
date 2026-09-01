/* JIT compiler tests: per-instruction differential testing vs interpreter.
 * Each test assembles a small block, runs via interpreter, runs via JIT,
 * and compares all state. */

#ifndef _MSC_VER

#include "jit.h"
#include "peripherals.h"

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

    return diffs;
}

/* Run a block: first via interpreter, then via JIT, compare results */
static void test_block_differential(xtensa_cpu_t *template_cpu, int num_insns,
                                    const char *test_name) {
    /* The production JIT deliberately rejects blocks shorter than four
     * instructions because dispatch overhead would dominate.  Most of
     * these differential cases isolate one instruction, so pad their
     * straight-line tail with NOP.N instead of silently treating a refused
     * compilation as a passing test. */
    uint32_t pad_pc = template_cpu->pc;
    for (int i = 0; i < num_insns; i++) {
        uint32_t insn;
        int ilen = xtensa_fetch(template_cpu, pad_pc, &insn);
        ASSERT_TRUE(ilen == 2 || ilen == 3);
        pad_pc += (uint32_t)ilen;
    }
    int padded_insns = num_insns;
    while (padded_insns < 4) {
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
    put_insn2(cpu, BASE + 3u, narrow(0xD, 15, 0, 3)); /* pad to the JIT's */
    put_insn2(cpu, BASE + 5u, narrow(0xD, 15, 0, 3)); /* four-instruction */
    put_insn2(cpu, BASE + 7u, narrow(0xD, 15, 0, 3)); /* profitability floor */
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

    /* LOOP a3, lend ; body ; (lend) — op0=6, t=7 (n=3,m=1), r=8 selects LOOP.
     * Body is five instructions so it clears the four-instruction floor. */
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
    ar_write(&cpu, 2, 17);
    /* sr occupies bits 15:8. SAR=3 therefore encodes r=0, s=3. */
    put_insn3(&cpu, BASE, rrr(1, 3, 0, 3, 2));  /* WSR SAR, a2 */
    put_insn3(&cpu, BASE + 3, rrr(0, 3, 0, 3, 5));  /* RSR a5, SAR */
    test_block_differential(&cpu, 2, "rsr_wsr_sar");
    teardown(&cpu);
}

/* Keep these builders local to the JIT suite: test_main.c includes every
 * test source into one translation unit, and test_window.c has equivalents. */
static uint32_t jit_calln_insn(int nn, int32_t offset) {
    uint32_t off18 = (uint32_t)offset & 0x3FFFF;
    return (off18 << 6) | ((nn & 3) << 4) | 5;
}

static uint32_t jit_entry_insn(int s, uint32_t framesize) {
    uint32_t imm12 = (framesize >> 3) & 0xFFF;
    return (imm12 << 12) | ((uint32_t)s << 8) | (3u << 4) | 6u;
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

/* ===== Test suite runner ===== */

static void run_jit_tests(void) {
    TEST_SUITE("jit");
    RUN_TEST(test_jit_init_destroy);
    RUN_TEST(test_jit_hot_threshold);
    RUN_TEST(test_jit_short_backedge_loop_is_native);
    RUN_TEST(test_jit_chained_run_accounts_every_block);
    RUN_TEST(test_jit_loop_backedge_dispatches_native_body);
    RUN_TEST(test_jit_block_compiled_outside_a_loop_is_not_reused_inside_one);
    RUN_TEST(test_waiti_time_is_not_counted_as_retired_instructions);
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
    RUN_TEST(test_jit_mull);
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
    RUN_TEST(test_jit_call4_windowed);
    RUN_TEST(test_jit_call0_full_return_address);
    RUN_TEST(test_jit_entry_windowed);
    RUN_TEST(test_jit_retw_windowed);
    RUN_TEST(test_jit_retw_n_windowed);
    RUN_TEST(test_jit_retw_tail_call_fallback);
    RUN_TEST(test_jit_entry_overflow_fallback);
}

#else /* _MSC_VER */

static void run_jit_tests(void) {
    TEST_SUITE("jit (disabled on MSVC)");
}

#endif /* _MSC_VER */
