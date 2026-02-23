/*
 * Tests for load/store instruction execution (M3).
 */
#include "test_helpers.h"
#include <string.h>

/* RRI8 builder (same as test_alu.c) */
static uint32_t ls_rri8(int subop, int s, int t, int imm8) {
    return (uint32_t)(((imm8 & 0xFF) << 16) | (subop << 12) | (s << 8) | (t << 4) | 2);
}

/* Data region base (SRAM data bus) */
#define DATA_BASE 0x3FFB0000u

/* ===== L32I / S32I round-trip ===== */

TEST(ls_s32i_l32i_basic) {
    xtensa_cpu_t cpu; setup(&cpu);
    /* S32I a3, a4, 0 */
    put_insn3(&cpu, BASE, ls_rri8(0x6, 4, 3, 0));
    ar_write(&cpu, 3, 0xDEADBEEF);
    ar_write(&cpu, 4, DATA_BASE);
    xtensa_step(&cpu);
    ASSERT_EQ(cpu.pc, BASE + 3);
    /* L32I a5, a4, 0 */
    put_insn3(&cpu, BASE + 3, ls_rri8(0x2, 4, 5, 0));
    xtensa_step(&cpu);
    ASSERT_EQ(ar_read(&cpu, 5), 0xDEADBEEF);
    /* Base register unchanged */
    ASSERT_EQ(ar_read(&cpu, 4), DATA_BASE);
    teardown(&cpu);
}

TEST(ls_s32i_l32i_offset) {
    xtensa_cpu_t cpu; setup(&cpu);
    /* S32I a3, a4, offset=4 (imm8=1, addr = base + 1*4) */
    put_insn3(&cpu, BASE, ls_rri8(0x6, 4, 3, 1));
    ar_write(&cpu, 3, 0xCAFEBABE);
    ar_write(&cpu, 4, DATA_BASE);
    xtensa_step(&cpu);
    /* L32I a5, a4, offset=4 */
    put_insn3(&cpu, BASE + 3, ls_rri8(0x2, 4, 5, 1));
    xtensa_step(&cpu);
    ASSERT_EQ(ar_read(&cpu, 5), 0xCAFEBABE);
    teardown(&cpu);
}

TEST(ls_s32i_l32i_max_offset) {
    xtensa_cpu_t cpu; setup(&cpu);
    /* S32I a3, a4, offset=252*4=1008 (imm8=252) */
    put_insn3(&cpu, BASE, ls_rri8(0x6, 4, 3, 252));
    ar_write(&cpu, 3, 0x12345678);
    ar_write(&cpu, 4, DATA_BASE);
    xtensa_step(&cpu);
    put_insn3(&cpu, BASE + 3, ls_rri8(0x2, 4, 5, 252));
    xtensa_step(&cpu);
    ASSERT_EQ(ar_read(&cpu, 5), 0x12345678);
    teardown(&cpu);
}

/* ===== L8UI / S8I ===== */

TEST(ls_s8i_l8ui_basic) {
    xtensa_cpu_t cpu; setup(&cpu);
    /* S8I a3, a4, 0 */
    put_insn3(&cpu, BASE, ls_rri8(0x4, 4, 3, 0));
    ar_write(&cpu, 3, 0xAB);
    ar_write(&cpu, 4, DATA_BASE);
    xtensa_step(&cpu);
    /* L8UI a5, a4, 0 */
    put_insn3(&cpu, BASE + 3, ls_rri8(0x0, 4, 5, 0));
    xtensa_step(&cpu);
    ASSERT_EQ(ar_read(&cpu, 5), 0xAB);
    teardown(&cpu);
}

TEST(ls_s8i_only_one_byte) {
    xtensa_cpu_t cpu; setup(&cpu);
    /* Write 0 to two consecutive bytes first */
    mem_write8(cpu.mem, DATA_BASE, 0);
    mem_write8(cpu.mem, DATA_BASE + 1, 0);
    /* S8I a3, a4, 0 — should only write 1 byte */
    put_insn3(&cpu, BASE, ls_rri8(0x4, 4, 3, 0));
    ar_write(&cpu, 3, 0xFF);
    ar_write(&cpu, 4, DATA_BASE);
    xtensa_step(&cpu);
    ASSERT_EQ(mem_read8(cpu.mem, DATA_BASE), 0xFF);
    ASSERT_EQ(mem_read8(cpu.mem, DATA_BASE + 1), 0); /* adjacent unaffected */
    teardown(&cpu);
}

TEST(ls_l8ui_offset) {
    xtensa_cpu_t cpu; setup(&cpu);
    mem_write8(cpu.mem, DATA_BASE + 10, 0x42);
    put_insn3(&cpu, BASE, ls_rri8(0x0, 4, 3, 10));
    ar_write(&cpu, 4, DATA_BASE);
    xtensa_step(&cpu);
    ASSERT_EQ(ar_read(&cpu, 3), 0x42);
    teardown(&cpu);
}

TEST(ls_l8ui_zero_extends) {
    xtensa_cpu_t cpu; setup(&cpu);
    mem_write8(cpu.mem, DATA_BASE, 0xFF);
    put_insn3(&cpu, BASE, ls_rri8(0x0, 4, 3, 0));
    ar_write(&cpu, 4, DATA_BASE);
    xtensa_step(&cpu);
    ASSERT_EQ(ar_read(&cpu, 3), 0xFF); /* zero-extended, not sign-extended */
    teardown(&cpu);
}

/* ===== L16UI / L16SI / S16I ===== */

TEST(ls_s16i_l16ui_basic) {
    xtensa_cpu_t cpu; setup(&cpu);
    /* S16I a3, a4, 0 */
    put_insn3(&cpu, BASE, ls_rri8(0x5, 4, 3, 0));
    ar_write(&cpu, 3, 0x1234);
    ar_write(&cpu, 4, DATA_BASE);
    xtensa_step(&cpu);
    /* L16UI a5, a4, 0 */
    put_insn3(&cpu, BASE + 3, ls_rri8(0x1, 4, 5, 0));
    xtensa_step(&cpu);
    ASSERT_EQ(ar_read(&cpu, 5), 0x1234);
    teardown(&cpu);
}

TEST(ls_l16ui_zero_extends) {
    xtensa_cpu_t cpu; setup(&cpu);
    mem_write16(cpu.mem, DATA_BASE, 0x8000);
    put_insn3(&cpu, BASE, ls_rri8(0x1, 4, 3, 0));
    ar_write(&cpu, 4, DATA_BASE);
    xtensa_step(&cpu);
    ASSERT_EQ(ar_read(&cpu, 3), 0x8000); /* zero-extended */
    teardown(&cpu);
}

TEST(ls_l16si_sign_extends) {
    xtensa_cpu_t cpu; setup(&cpu);
    mem_write16(cpu.mem, DATA_BASE, 0x8000);
    /* L16SI a3, a4, 0 */
    put_insn3(&cpu, BASE, ls_rri8(0x9, 4, 3, 0));
    ar_write(&cpu, 4, DATA_BASE);
    xtensa_step(&cpu);
    ASSERT_EQ(ar_read(&cpu, 3), 0xFFFF8000); /* sign-extended */
    teardown(&cpu);
}

TEST(ls_l16si_positive) {
    xtensa_cpu_t cpu; setup(&cpu);
    mem_write16(cpu.mem, DATA_BASE, 0x7FFF);
    put_insn3(&cpu, BASE, ls_rri8(0x9, 4, 3, 0));
    ar_write(&cpu, 4, DATA_BASE);
    xtensa_step(&cpu);
    ASSERT_EQ(ar_read(&cpu, 3), 0x7FFF); /* positive, no sign extension */
    teardown(&cpu);
}

TEST(ls_s16i_l16ui_offset) {
    xtensa_cpu_t cpu; setup(&cpu);
    /* S16I a3, a4, offset=2 (imm8=1, addr = base + 1*2) */
    put_insn3(&cpu, BASE, ls_rri8(0x5, 4, 3, 1));
    ar_write(&cpu, 3, 0xBEEF);
    ar_write(&cpu, 4, DATA_BASE);
    xtensa_step(&cpu);
    put_insn3(&cpu, BASE + 3, ls_rri8(0x1, 4, 5, 1));
    xtensa_step(&cpu);
    ASSERT_EQ(ar_read(&cpu, 5), 0xBEEF);
    teardown(&cpu);
}

/* ===== L32I.N / S32I.N ===== */

TEST(ls_l32i_n_s32i_n_basic) {
    xtensa_cpu_t cpu; setup(&cpu);
    /* S32I.N a3, a4, 0 (r=0) */
    put_insn2(&cpu, BASE, narrow(0x9, 0, 4, 3));
    ar_write(&cpu, 3, 0xDEADBEEF);
    ar_write(&cpu, 4, DATA_BASE);
    xtensa_step(&cpu);
    ASSERT_EQ(cpu.pc, BASE + 2);
    /* L32I.N a5, a4, 0 (r=0) */
    put_insn2(&cpu, BASE + 2, narrow(0x8, 0, 4, 5));
    xtensa_step(&cpu);
    ASSERT_EQ(ar_read(&cpu, 5), 0xDEADBEEF);
    teardown(&cpu);
}

TEST(ls_l32i_n_max_offset) {
    xtensa_cpu_t cpu; setup(&cpu);
    /* S32I.N a3, a4, r=15 → offset 60 */
    put_insn2(&cpu, BASE, narrow(0x9, 15, 4, 3));
    ar_write(&cpu, 3, 0xFACEFEED);
    ar_write(&cpu, 4, DATA_BASE);
    xtensa_step(&cpu);
    put_insn2(&cpu, BASE + 2, narrow(0x8, 15, 4, 5));
    xtensa_step(&cpu);
    ASSERT_EQ(ar_read(&cpu, 5), 0xFACEFEED);
    teardown(&cpu);
}

/* ===== L32R ===== */

TEST(ls_l32r_basic) {
    xtensa_cpu_t cpu; setup(&cpu);
    /* Place literal value at a word-aligned address before the instruction.
     * L32R is op0=1, RI16 format.
     * target = (next_pc & ~3) + (0xFFFC0000 | (imm16 << 2))
     *
     * We want to load from BASE-4 (the word just before our instruction).
     * next_pc = BASE + 3
     * (BASE+3) & ~3 = BASE (BASE is 0x40080000, word-aligned)
     * offset = (BASE-4) - BASE = -4
     * 0xFFFC0000 | (imm16 << 2) = -4
     * imm16 << 2 must contribute the low 18 bits of -4 = 0x3FFFC
     * imm16 = 0x3FFFC >> 2 = 0xFFFF
     */
    mem_write32(cpu.mem, BASE - 4, 0xBAADF00D);
    /* L32R a3, [literal]: op0=1, t=3, imm16=0xFFFF */
    uint32_t insn = 1 | (3 << 4) | (0xFFFF << 8);
    put_insn3(&cpu, BASE, insn);
    xtensa_step(&cpu);
    ASSERT_EQ(ar_read(&cpu, 3), 0xBAADF00D);
    ASSERT_EQ(cpu.pc, BASE + 3);
    teardown(&cpu);
}

TEST(ls_l32r_farther_back) {
    xtensa_cpu_t cpu; setup(&cpu);
    /* Load from BASE-256 (64 words back).
     * offset = -256
     * 0xFFFC0000 | (imm16 << 2) = 0xFFFFFF00
     * imm16 << 2 = 0xFFFFFF00 & 0x3FFFF = 0x3FF00
     * imm16 = 0x3FF00 >> 2 = 0xFFC0
     */
    mem_write32(cpu.mem, BASE - 256, 0x11223344);
    uint32_t insn = 1 | (5 << 4) | (0xFFC0 << 8);
    put_insn3(&cpu, BASE, insn);
    xtensa_step(&cpu);
    ASSERT_EQ(ar_read(&cpu, 5), 0x11223344);
    teardown(&cpu);
}

/* ===== S32C1I (compare-and-swap) ===== */

TEST(ls_s32c1i_match) {
    xtensa_cpu_t cpu; setup(&cpu);
    /* Set memory to 0xAAAAAAAA */
    mem_write32(cpu.mem, DATA_BASE, 0xAAAAAAAA);
    /* SCOMPARE1 = 0xAAAAAAAA (matches) */
    cpu.scompare1 = 0xAAAAAAAA;
    /* S32C1I a3, a4, 0: try to swap in ar[3] */
    put_insn3(&cpu, BASE, ls_rri8(0xE, 4, 3, 0));
    ar_write(&cpu, 3, 0xBBBBBBBB); /* new value */
    ar_write(&cpu, 4, DATA_BASE);
    xtensa_step(&cpu);
    /* ar[t] gets old value */
    ASSERT_EQ(ar_read(&cpu, 3), 0xAAAAAAAA);
    /* Memory updated */
    ASSERT_EQ(mem_read32(cpu.mem, DATA_BASE), 0xBBBBBBBB);
    teardown(&cpu);
}

TEST(ls_s32c1i_no_match) {
    xtensa_cpu_t cpu; setup(&cpu);
    mem_write32(cpu.mem, DATA_BASE, 0xAAAAAAAA);
    cpu.scompare1 = 0x11111111; /* doesn't match */
    put_insn3(&cpu, BASE, ls_rri8(0xE, 4, 3, 0));
    ar_write(&cpu, 3, 0xBBBBBBBB);
    ar_write(&cpu, 4, DATA_BASE);
    xtensa_step(&cpu);
    /* ar[t] gets old value */
    ASSERT_EQ(ar_read(&cpu, 3), 0xAAAAAAAA);
    /* Memory NOT updated */
    ASSERT_EQ(mem_read32(cpu.mem, DATA_BASE), 0xAAAAAAAA);
    teardown(&cpu);
}

/* ===== L32AI / S32RI (memory ordering no-ops) ===== */

TEST(ls_l32ai_like_l32i) {
    xtensa_cpu_t cpu; setup(&cpu);
    mem_write32(cpu.mem, DATA_BASE, 0x55555555);
    put_insn3(&cpu, BASE, ls_rri8(0xB, 4, 3, 0));
    ar_write(&cpu, 4, DATA_BASE);
    xtensa_step(&cpu);
    ASSERT_EQ(ar_read(&cpu, 3), 0x55555555);
    teardown(&cpu);
}

TEST(ls_s32ri_like_s32i) {
    xtensa_cpu_t cpu; setup(&cpu);
    put_insn3(&cpu, BASE, ls_rri8(0xF, 4, 3, 0));
    ar_write(&cpu, 3, 0x66666666);
    ar_write(&cpu, 4, DATA_BASE);
    xtensa_step(&cpu);
    ASSERT_EQ(mem_read32(cpu.mem, DATA_BASE), 0x66666666);
    teardown(&cpu);
}

/* ===== Cache ops ===== */

TEST(ls_cache_noop) {
    xtensa_cpu_t cpu; setup(&cpu);
    /* CACHE op: r=7, should be a no-op, PC advances */
    put_insn3(&cpu, BASE, ls_rri8(0x7, 4, 3, 0));
    ar_write(&cpu, 4, DATA_BASE);
    xtensa_step(&cpu);
    ASSERT_EQ(cpu.pc, BASE + 3);
    ASSERT_FALSE(cpu.exception);
    teardown(&cpu);
}

void run_loadstore_tests(void) {
    TEST_SUITE("Load/Store Execution");

    RUN_TEST(ls_s32i_l32i_basic);
    RUN_TEST(ls_s32i_l32i_offset);
    RUN_TEST(ls_s32i_l32i_max_offset);
    RUN_TEST(ls_s8i_l8ui_basic);
    RUN_TEST(ls_s8i_only_one_byte);
    RUN_TEST(ls_l8ui_offset);
    RUN_TEST(ls_l8ui_zero_extends);
    RUN_TEST(ls_s16i_l16ui_basic);
    RUN_TEST(ls_l16ui_zero_extends);
    RUN_TEST(ls_l16si_sign_extends);
    RUN_TEST(ls_l16si_positive);
    RUN_TEST(ls_s16i_l16ui_offset);
    RUN_TEST(ls_l32i_n_s32i_n_basic);
    RUN_TEST(ls_l32i_n_max_offset);
    RUN_TEST(ls_l32r_basic);
    RUN_TEST(ls_l32r_farther_back);
    RUN_TEST(ls_s32c1i_match);
    RUN_TEST(ls_s32c1i_no_match);
    RUN_TEST(ls_l32ai_like_l32i);
    RUN_TEST(ls_s32ri_like_s32i);
    RUN_TEST(ls_cache_noop);
}
