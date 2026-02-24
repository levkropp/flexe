/*
 * Tests for ALU instruction execution.
 * Each test places an instruction at BASE, sets registers, calls xtensa_step(),
 * and checks the result register and PC advancement.
 */
#include "test_helpers.h"
#include <string.h>

/* Build RRI8 instruction: op0=2, r=subop, s, t, imm8 */
static uint32_t rri8(int subop, int s, int t, int imm8) {
    return (uint32_t)(((imm8 & 0xFF) << 16) | (subop << 12) | (s << 8) | (t << 4) | 2);
}

/* ===== ADD ===== */

TEST(exec_add_basic) {
    xtensa_cpu_t cpu; setup(&cpu);
    /* ADD a3, a4, a5 */
    put_insn3(&cpu, BASE, rrr(8, 0, 3, 4, 5));
    ar_write(&cpu, 4, 100);
    ar_write(&cpu, 5, 200);
    xtensa_step(&cpu);
    ASSERT_EQ(ar_read(&cpu, 3), 300);
    ASSERT_EQ(cpu.pc, BASE + 3);
    teardown(&cpu);
}

TEST(exec_add_overflow) {
    xtensa_cpu_t cpu; setup(&cpu);
    put_insn3(&cpu, BASE, rrr(8, 0, 3, 4, 5));
    ar_write(&cpu, 4, 0xFFFFFFFF);
    ar_write(&cpu, 5, 2);
    xtensa_step(&cpu);
    ASSERT_EQ(ar_read(&cpu, 3), 1); /* wraps */
    teardown(&cpu);
}

TEST(exec_add_zero) {
    xtensa_cpu_t cpu; setup(&cpu);
    put_insn3(&cpu, BASE, rrr(8, 0, 3, 4, 5));
    ar_write(&cpu, 4, 0);
    ar_write(&cpu, 5, 0);
    xtensa_step(&cpu);
    ASSERT_EQ(ar_read(&cpu, 3), 0);
    teardown(&cpu);
}

/* ===== SUB ===== */

TEST(exec_sub_basic) {
    xtensa_cpu_t cpu; setup(&cpu);
    put_insn3(&cpu, BASE, rrr(12, 0, 3, 4, 5));
    ar_write(&cpu, 4, 300);
    ar_write(&cpu, 5, 100);
    xtensa_step(&cpu);
    ASSERT_EQ(ar_read(&cpu, 3), 200);
    teardown(&cpu);
}

TEST(exec_sub_underflow) {
    xtensa_cpu_t cpu; setup(&cpu);
    put_insn3(&cpu, BASE, rrr(12, 0, 3, 4, 5));
    ar_write(&cpu, 4, 0);
    ar_write(&cpu, 5, 1);
    xtensa_step(&cpu);
    ASSERT_EQ(ar_read(&cpu, 3), 0xFFFFFFFF);
    teardown(&cpu);
}

TEST(exec_sub_self) {
    xtensa_cpu_t cpu; setup(&cpu);
    /* SUB a3, a4, a4 */
    put_insn3(&cpu, BASE, rrr(12, 0, 3, 4, 4));
    ar_write(&cpu, 4, 42);
    xtensa_step(&cpu);
    ASSERT_EQ(ar_read(&cpu, 3), 0);
    teardown(&cpu);
}

/* ===== ADDX2/4/8 ===== */

TEST(exec_addx2) {
    xtensa_cpu_t cpu; setup(&cpu);
    put_insn3(&cpu, BASE, rrr(9, 0, 3, 4, 5));
    ar_write(&cpu, 4, 10);
    ar_write(&cpu, 5, 5);
    xtensa_step(&cpu);
    ASSERT_EQ(ar_read(&cpu, 3), 10 * 2 + 5);
    teardown(&cpu);
}

TEST(exec_addx4) {
    xtensa_cpu_t cpu; setup(&cpu);
    put_insn3(&cpu, BASE, rrr(10, 0, 3, 4, 5));
    ar_write(&cpu, 4, 10);
    ar_write(&cpu, 5, 5);
    xtensa_step(&cpu);
    ASSERT_EQ(ar_read(&cpu, 3), 10 * 4 + 5);
    teardown(&cpu);
}

TEST(exec_addx8) {
    xtensa_cpu_t cpu; setup(&cpu);
    put_insn3(&cpu, BASE, rrr(11, 0, 3, 4, 5));
    ar_write(&cpu, 4, 10);
    ar_write(&cpu, 5, 5);
    xtensa_step(&cpu);
    ASSERT_EQ(ar_read(&cpu, 3), 10 * 8 + 5);
    teardown(&cpu);
}

/* ===== SUBX2/4/8 ===== */

TEST(exec_subx2) {
    xtensa_cpu_t cpu; setup(&cpu);
    put_insn3(&cpu, BASE, rrr(13, 0, 3, 4, 5));
    ar_write(&cpu, 4, 10);
    ar_write(&cpu, 5, 5);
    xtensa_step(&cpu);
    ASSERT_EQ(ar_read(&cpu, 3), 10 * 2 - 5);
    teardown(&cpu);
}

TEST(exec_subx4) {
    xtensa_cpu_t cpu; setup(&cpu);
    put_insn3(&cpu, BASE, rrr(14, 0, 3, 4, 5));
    ar_write(&cpu, 4, 10);
    ar_write(&cpu, 5, 5);
    xtensa_step(&cpu);
    ASSERT_EQ(ar_read(&cpu, 3), 10 * 4 - 5);
    teardown(&cpu);
}

/* ===== NEG, ABS ===== */

TEST(exec_neg_positive) {
    xtensa_cpu_t cpu; setup(&cpu);
    /* NEG a3, a5: op2=6, op1=0, r=3, s=0, t=5 */
    put_insn3(&cpu, BASE, rrr(6, 0, 3, 0, 5));
    ar_write(&cpu, 5, 42);
    xtensa_step(&cpu);
    ASSERT_EQ(ar_read(&cpu, 3), (uint32_t)-42);
    teardown(&cpu);
}

TEST(exec_neg_negative) {
    xtensa_cpu_t cpu; setup(&cpu);
    put_insn3(&cpu, BASE, rrr(6, 0, 3, 0, 5));
    ar_write(&cpu, 5, (uint32_t)-42);
    xtensa_step(&cpu);
    ASSERT_EQ(ar_read(&cpu, 3), 42);
    teardown(&cpu);
}

TEST(exec_neg_zero) {
    xtensa_cpu_t cpu; setup(&cpu);
    put_insn3(&cpu, BASE, rrr(6, 0, 3, 0, 5));
    ar_write(&cpu, 5, 0);
    xtensa_step(&cpu);
    ASSERT_EQ(ar_read(&cpu, 3), 0);
    teardown(&cpu);
}

TEST(exec_abs_positive) {
    xtensa_cpu_t cpu; setup(&cpu);
    /* ABS a3, a5: op2=6, op1=0, r=3, s=1, t=5 */
    put_insn3(&cpu, BASE, rrr(6, 0, 3, 1, 5));
    ar_write(&cpu, 5, 42);
    xtensa_step(&cpu);
    ASSERT_EQ(ar_read(&cpu, 3), 42);
    teardown(&cpu);
}

TEST(exec_abs_negative) {
    xtensa_cpu_t cpu; setup(&cpu);
    put_insn3(&cpu, BASE, rrr(6, 0, 3, 1, 5));
    ar_write(&cpu, 5, (uint32_t)-42);
    xtensa_step(&cpu);
    ASSERT_EQ(ar_read(&cpu, 3), 42);
    teardown(&cpu);
}

TEST(exec_abs_intmin) {
    xtensa_cpu_t cpu; setup(&cpu);
    put_insn3(&cpu, BASE, rrr(6, 0, 3, 1, 5));
    ar_write(&cpu, 5, 0x80000000);
    xtensa_step(&cpu);
    /* INT_MIN: -(-2^31) overflows to -2^31 in 2's complement */
    ASSERT_EQ(ar_read(&cpu, 3), 0x80000000);
    teardown(&cpu);
}

/* ===== AND, OR, XOR ===== */

TEST(exec_and) {
    xtensa_cpu_t cpu; setup(&cpu);
    put_insn3(&cpu, BASE, rrr(1, 0, 3, 4, 5));
    ar_write(&cpu, 4, 0xFF00FF00);
    ar_write(&cpu, 5, 0x0F0F0F0F);
    xtensa_step(&cpu);
    ASSERT_EQ(ar_read(&cpu, 3), 0x0F000F00);
    teardown(&cpu);
}

TEST(exec_or) {
    xtensa_cpu_t cpu; setup(&cpu);
    put_insn3(&cpu, BASE, rrr(2, 0, 3, 4, 5));
    ar_write(&cpu, 4, 0xFF00FF00);
    ar_write(&cpu, 5, 0x0F0F0F0F);
    xtensa_step(&cpu);
    ASSERT_EQ(ar_read(&cpu, 3), 0xFF0FFF0F);
    teardown(&cpu);
}

TEST(exec_xor) {
    xtensa_cpu_t cpu; setup(&cpu);
    put_insn3(&cpu, BASE, rrr(3, 0, 3, 4, 5));
    ar_write(&cpu, 4, 0xFF00FF00);
    ar_write(&cpu, 5, 0x0F0F0F0F);
    xtensa_step(&cpu);
    ASSERT_EQ(ar_read(&cpu, 3), 0xF00FF00F);
    teardown(&cpu);
}

TEST(exec_and_with_zero) {
    xtensa_cpu_t cpu; setup(&cpu);
    put_insn3(&cpu, BASE, rrr(1, 0, 3, 4, 5));
    ar_write(&cpu, 4, 0x12345678);
    ar_write(&cpu, 5, 0);
    xtensa_step(&cpu);
    ASSERT_EQ(ar_read(&cpu, 3), 0);
    teardown(&cpu);
}

TEST(exec_or_with_allones) {
    xtensa_cpu_t cpu; setup(&cpu);
    put_insn3(&cpu, BASE, rrr(2, 0, 3, 4, 5));
    ar_write(&cpu, 4, 0x12345678);
    ar_write(&cpu, 5, 0xFFFFFFFF);
    xtensa_step(&cpu);
    ASSERT_EQ(ar_read(&cpu, 3), 0xFFFFFFFF);
    teardown(&cpu);
}

/* ===== ADDI ===== */

TEST(exec_addi_positive) {
    xtensa_cpu_t cpu; setup(&cpu);
    /* ADDI a3, a4, 10: op0=2, r=0xC, s=4, t=3, imm8=10 */
    put_insn3(&cpu, BASE, rri8(0xC, 4, 3, 10));
    ar_write(&cpu, 4, 100);
    xtensa_step(&cpu);
    ASSERT_EQ(ar_read(&cpu, 3), 110);
    teardown(&cpu);
}

TEST(exec_addi_negative) {
    xtensa_cpu_t cpu; setup(&cpu);
    /* ADDI a3, a4, -5: imm8 = 0xFB (-5 as signed byte) */
    put_insn3(&cpu, BASE, rri8(0xC, 4, 3, 0xFB));
    ar_write(&cpu, 4, 100);
    xtensa_step(&cpu);
    ASSERT_EQ(ar_read(&cpu, 3), 95);
    teardown(&cpu);
}

TEST(exec_addi_zero) {
    xtensa_cpu_t cpu; setup(&cpu);
    put_insn3(&cpu, BASE, rri8(0xC, 4, 3, 0));
    ar_write(&cpu, 4, 42);
    xtensa_step(&cpu);
    ASSERT_EQ(ar_read(&cpu, 3), 42);
    teardown(&cpu);
}

/* ===== ADDMI ===== */

TEST(exec_addmi_basic) {
    xtensa_cpu_t cpu; setup(&cpu);
    /* ADDMI a3, a4, 256: r=0xD, imm8=1 (1*256=256) */
    put_insn3(&cpu, BASE, rri8(0xD, 4, 3, 1));
    ar_write(&cpu, 4, 100);
    xtensa_step(&cpu);
    ASSERT_EQ(ar_read(&cpu, 3), 356);
    teardown(&cpu);
}

TEST(exec_addmi_negative) {
    xtensa_cpu_t cpu; setup(&cpu);
    /* ADDMI a3, a4, -256: imm8=0xFF (-1*256=-256) */
    put_insn3(&cpu, BASE, rri8(0xD, 4, 3, 0xFF));
    ar_write(&cpu, 4, 1000);
    xtensa_step(&cpu);
    ASSERT_EQ(ar_read(&cpu, 3), 744);
    teardown(&cpu);
}

/* ===== MOVI ===== */

TEST(exec_movi_positive) {
    xtensa_cpu_t cpu; setup(&cpu);
    /* MOVI a3, 100: r=0xA, s=imm[11:8]=0, t=3, imm8=100 */
    put_insn3(&cpu, BASE, rri8(0xA, 0, 3, 100));
    xtensa_step(&cpu);
    ASSERT_EQ(ar_read(&cpu, 3), 100);
    teardown(&cpu);
}

TEST(exec_movi_negative) {
    xtensa_cpu_t cpu; setup(&cpu);
    /* MOVI a3, -1: imm12 = 0xFFF, s=0xF, imm8=0xFF */
    put_insn3(&cpu, BASE, rri8(0xA, 0xF, 3, 0xFF));
    xtensa_step(&cpu);
    ASSERT_EQ(ar_read(&cpu, 3), 0xFFFFFFFF);
    teardown(&cpu);
}

/* ===== ADD.N ===== */

TEST(exec_add_n) {
    xtensa_cpu_t cpu; setup(&cpu);
    put_insn2(&cpu, BASE, narrow(0xA, 3, 4, 5));
    ar_write(&cpu, 4, 100);
    ar_write(&cpu, 5, 200);
    xtensa_step(&cpu);
    ASSERT_EQ(ar_read(&cpu, 3), 300);
    ASSERT_EQ(cpu.pc, BASE + 2);
    teardown(&cpu);
}

/* ===== ADDI.N ===== */

TEST(exec_addi_n_basic) {
    xtensa_cpu_t cpu; setup(&cpu);
    /* ADDI.N a3, a4, 5: op0=0xB, r=3, s=4, t=5 */
    put_insn2(&cpu, BASE, narrow(0xB, 3, 4, 5));
    ar_write(&cpu, 4, 100);
    xtensa_step(&cpu);
    ASSERT_EQ(ar_read(&cpu, 3), 105);
    teardown(&cpu);
}

TEST(exec_addi_n_minus1) {
    xtensa_cpu_t cpu; setup(&cpu);
    /* ADDI.N a3, a4, -1: t=0 maps to -1 */
    put_insn2(&cpu, BASE, narrow(0xB, 3, 4, 0));
    ar_write(&cpu, 4, 100);
    xtensa_step(&cpu);
    ASSERT_EQ(ar_read(&cpu, 3), 99);
    teardown(&cpu);
}

/* ===== MOVI.N ===== */

TEST(exec_movi_n_basic) {
    xtensa_cpu_t cpu; setup(&cpu);
    /* MOVI.N a5, 10: op0=0xC, r=imm[3:0]=10, s=5, t=(0<<3)|imm[6:4]=0 */
    put_insn2(&cpu, BASE, narrow(0xC, 10, 5, 0));
    xtensa_step(&cpu);
    ASSERT_EQ(ar_read(&cpu, 5), 10);
    teardown(&cpu);
}

TEST(exec_movi_n_90) {
    xtensa_cpu_t cpu; setup(&cpu);
    /* MOVI.N a5, 90: imm7=90=0b1011010, r=0xA, t[2:0]=5, t[3]=0
       This tests the 64-95 range which is positive, NOT sign-extended */
    put_insn2(&cpu, BASE, narrow(0xC, 0xA, 5, 5));
    xtensa_step(&cpu);
    ASSERT_EQ(ar_read(&cpu, 5), 90u);
    teardown(&cpu);
}

TEST(exec_movi_n_negative) {
    xtensa_cpu_t cpu; setup(&cpu);
    /* MOVI.N a5, -32: imm7=96=0b1100000, r=0, t[2:0]=6, t[3]=0 */
    put_insn2(&cpu, BASE, narrow(0xC, 0, 5, 6));
    xtensa_step(&cpu);
    ASSERT_EQ(ar_read(&cpu, 5), (uint32_t)-32);
    teardown(&cpu);
}

/* ===== NOP ===== */

TEST(exec_nop) {
    xtensa_cpu_t cpu; setup(&cpu);
    put_insn3(&cpu, BASE, 0x002F00); /* NOP */
    ar_write(&cpu, 3, 42);
    xtensa_step(&cpu);
    ASSERT_EQ(ar_read(&cpu, 3), 42); /* unchanged */
    ASSERT_EQ(cpu.pc, BASE + 3);
    teardown(&cpu);
}

TEST(exec_nop_n) {
    xtensa_cpu_t cpu; setup(&cpu);
    put_insn2(&cpu, BASE, 0xF03D); /* NOP.N */
    ar_write(&cpu, 3, 42);
    xtensa_step(&cpu);
    ASSERT_EQ(ar_read(&cpu, 3), 42);
    ASSERT_EQ(cpu.pc, BASE + 2);
    teardown(&cpu);
}

/* ===== ILL / BREAK ===== */

TEST(exec_ill) {
    xtensa_cpu_t cpu; setup(&cpu);
    /* ILL: all zeros */
    put_insn3(&cpu, BASE, 0x000000);
    xtensa_step(&cpu);
    ASSERT_TRUE(cpu.exception);
    teardown(&cpu);
}

TEST(exec_break) {
    xtensa_cpu_t cpu; setup(&cpu);
    /* BREAK s, t: op2=0, op1=0, r=4, s and t encode params
       Actually BREAK is r=4 in the SYNC group? No.
       Let me use the correct encoding: op2=0, r=4(BREAK), s=0, t=0 */
    put_insn3(&cpu, BASE, rrr(0, 0, 4, 0, 0));
    xtensa_step(&cpu);
    ASSERT_TRUE(cpu.debug_break);
    teardown(&cpu);
}

/* ===== SALT / SALTU ===== */

TEST(exec_salt_less) {
    xtensa_cpu_t cpu; setup(&cpu);
    /* SALT a3, a4, a5: op2=6, op1=2 */
    put_insn3(&cpu, BASE, rrr(6, 2, 3, 4, 5));
    ar_write(&cpu, 4, (uint32_t)-10);
    ar_write(&cpu, 5, 5);
    xtensa_step(&cpu);
    ASSERT_EQ(ar_read(&cpu, 3), 1); /* -10 < 5 signed */
    teardown(&cpu);
}

TEST(exec_salt_equal) {
    xtensa_cpu_t cpu; setup(&cpu);
    put_insn3(&cpu, BASE, rrr(6, 2, 3, 4, 5));
    ar_write(&cpu, 4, 5);
    ar_write(&cpu, 5, 5);
    xtensa_step(&cpu);
    ASSERT_EQ(ar_read(&cpu, 3), 0); /* not strictly less */
    teardown(&cpu);
}

TEST(exec_saltu_unsigned) {
    xtensa_cpu_t cpu; setup(&cpu);
    /* SALTU: op2=7, op1=2 */
    put_insn3(&cpu, BASE, rrr(7, 2, 3, 4, 5));
    ar_write(&cpu, 4, 5);
    ar_write(&cpu, 5, 0xFFFFFFFF); /* large unsigned */
    xtensa_step(&cpu);
    ASSERT_EQ(ar_read(&cpu, 3), 1); /* 5 < 0xFFFFFFFF unsigned */
    teardown(&cpu);
}

/* ===== MULL / MULUH / MULSH ===== */

TEST(exec_mull_basic) {
    xtensa_cpu_t cpu; setup(&cpu);
    put_insn3(&cpu, BASE, rrr(8, 2, 3, 4, 5));
    ar_write(&cpu, 4, 100);
    ar_write(&cpu, 5, 200);
    xtensa_step(&cpu);
    ASSERT_EQ(ar_read(&cpu, 3), 20000);
    teardown(&cpu);
}

TEST(exec_mull_overflow) {
    xtensa_cpu_t cpu; setup(&cpu);
    put_insn3(&cpu, BASE, rrr(8, 2, 3, 4, 5));
    ar_write(&cpu, 4, 0x10000);
    ar_write(&cpu, 5, 0x10000);
    xtensa_step(&cpu);
    ASSERT_EQ(ar_read(&cpu, 3), 0); /* low 32 bits of 0x100000000 */
    teardown(&cpu);
}

TEST(exec_muluh) {
    xtensa_cpu_t cpu; setup(&cpu);
    /* MULUH: op2=10, op1=2 */
    put_insn3(&cpu, BASE, rrr(10, 2, 3, 4, 5));
    ar_write(&cpu, 4, 0x10000);
    ar_write(&cpu, 5, 0x10000);
    xtensa_step(&cpu);
    ASSERT_EQ(ar_read(&cpu, 3), 1); /* high 32 bits of 0x100000000 */
    teardown(&cpu);
}

/* ===== SEXT ===== */

TEST(exec_sext_8bit) {
    xtensa_cpu_t cpu; setup(&cpu);
    /* SEXT a3, a4, 7: sign-extend from bit 7 (8 bits). op2=2, op1=3, r=3, s=4, t=0 (t+8=8) */
    /* Wait: t=0 means bit (0+7)=7, so 8-bit sign-extend */
    put_insn3(&cpu, BASE, rrr(2, 3, 3, 4, 0));
    ar_write(&cpu, 4, 0x000000FF); /* 255 unsigned = -1 in 8-bit signed */
    xtensa_step(&cpu);
    ASSERT_EQ(ar_read(&cpu, 3), 0xFFFFFFFF);
    teardown(&cpu);
}

TEST(exec_sext_8bit_positive) {
    xtensa_cpu_t cpu; setup(&cpu);
    put_insn3(&cpu, BASE, rrr(2, 3, 3, 4, 0));
    ar_write(&cpu, 4, 0x0000007F); /* 127, positive in 8-bit */
    xtensa_step(&cpu);
    ASSERT_EQ(ar_read(&cpu, 3), 0x0000007F);
    teardown(&cpu);
}

TEST(exec_sext_16bit) {
    xtensa_cpu_t cpu; setup(&cpu);
    /* SEXT a3, a4, 15: t=8 → bit (8+7)=15 → 16-bit sign-extend */
    put_insn3(&cpu, BASE, rrr(2, 3, 3, 4, 8));
    ar_write(&cpu, 4, 0x0000FFFF); /* -1 in 16-bit */
    xtensa_step(&cpu);
    ASSERT_EQ(ar_read(&cpu, 3), 0xFFFFFFFF);
    teardown(&cpu);
}

/* ===== CLAMPS ===== */

TEST(exec_clamps_in_range) {
    xtensa_cpu_t cpu; setup(&cpu);
    /* CLAMPS a3, a4, 7: clamp to -(2^7)..+(2^7-1) = -128..127 */
    /* op2=3, op1=3, r=3, s=4, t=0 → bits=0+7=7 */
    put_insn3(&cpu, BASE, rrr(3, 3, 3, 4, 0));
    ar_write(&cpu, 4, 50);
    xtensa_step(&cpu);
    ASSERT_EQ(ar_read(&cpu, 3), 50);
    teardown(&cpu);
}

TEST(exec_clamps_above) {
    xtensa_cpu_t cpu; setup(&cpu);
    put_insn3(&cpu, BASE, rrr(3, 3, 3, 4, 0));
    ar_write(&cpu, 4, 200);
    xtensa_step(&cpu);
    ASSERT_EQ(ar_read(&cpu, 3), 127); /* clamped to 2^7-1 */
    teardown(&cpu);
}

TEST(exec_clamps_below) {
    xtensa_cpu_t cpu; setup(&cpu);
    put_insn3(&cpu, BASE, rrr(3, 3, 3, 4, 0));
    ar_write(&cpu, 4, (uint32_t)-200);
    xtensa_step(&cpu);
    ASSERT_EQ(ar_read(&cpu, 3), (uint32_t)-128); /* clamped to -(2^7) */
    teardown(&cpu);
}

/* ===== MIN / MAX / MINU / MAXU ===== */

TEST(exec_min) {
    xtensa_cpu_t cpu; setup(&cpu);
    put_insn3(&cpu, BASE, rrr(4, 3, 3, 4, 5));
    ar_write(&cpu, 4, 10);
    ar_write(&cpu, 5, 20);
    xtensa_step(&cpu);
    ASSERT_EQ(ar_read(&cpu, 3), 10);
    teardown(&cpu);
}

TEST(exec_min_signed) {
    xtensa_cpu_t cpu; setup(&cpu);
    put_insn3(&cpu, BASE, rrr(4, 3, 3, 4, 5));
    ar_write(&cpu, 4, (uint32_t)-5);
    ar_write(&cpu, 5, 5);
    xtensa_step(&cpu);
    ASSERT_EQ(ar_read(&cpu, 3), (uint32_t)-5);
    teardown(&cpu);
}

TEST(exec_max) {
    xtensa_cpu_t cpu; setup(&cpu);
    put_insn3(&cpu, BASE, rrr(5, 3, 3, 4, 5));
    ar_write(&cpu, 4, 10);
    ar_write(&cpu, 5, 20);
    xtensa_step(&cpu);
    ASSERT_EQ(ar_read(&cpu, 3), 20);
    teardown(&cpu);
}

TEST(exec_minu) {
    xtensa_cpu_t cpu; setup(&cpu);
    put_insn3(&cpu, BASE, rrr(6, 3, 3, 4, 5));
    ar_write(&cpu, 4, 0xFFFFFFFF); /* large unsigned */
    ar_write(&cpu, 5, 5);
    xtensa_step(&cpu);
    ASSERT_EQ(ar_read(&cpu, 3), 5);
    teardown(&cpu);
}

TEST(exec_maxu) {
    xtensa_cpu_t cpu; setup(&cpu);
    put_insn3(&cpu, BASE, rrr(7, 3, 3, 4, 5));
    ar_write(&cpu, 4, 0xFFFFFFFF);
    ar_write(&cpu, 5, 5);
    xtensa_step(&cpu);
    ASSERT_EQ(ar_read(&cpu, 3), 0xFFFFFFFF);
    teardown(&cpu);
}

/* ===== QUOU / QUOS / REMU / REMS ===== */

TEST(exec_quou) {
    xtensa_cpu_t cpu; setup(&cpu);
    put_insn3(&cpu, BASE, rrr(12, 2, 3, 4, 5));
    ar_write(&cpu, 4, 100);
    ar_write(&cpu, 5, 7);
    xtensa_step(&cpu);
    ASSERT_EQ(ar_read(&cpu, 3), 14); /* 100/7 = 14 */
    teardown(&cpu);
}

TEST(exec_remu) {
    xtensa_cpu_t cpu; setup(&cpu);
    put_insn3(&cpu, BASE, rrr(14, 2, 3, 4, 5));
    ar_write(&cpu, 4, 100);
    ar_write(&cpu, 5, 7);
    xtensa_step(&cpu);
    ASSERT_EQ(ar_read(&cpu, 3), 2); /* 100%7 = 2 */
    teardown(&cpu);
}

TEST(exec_quos) {
    xtensa_cpu_t cpu; setup(&cpu);
    put_insn3(&cpu, BASE, rrr(13, 2, 3, 4, 5));
    ar_write(&cpu, 4, (uint32_t)-100);
    ar_write(&cpu, 5, 7);
    xtensa_step(&cpu);
    ASSERT_EQ(ar_read(&cpu, 3), (uint32_t)-14);
    teardown(&cpu);
}

TEST(exec_div_by_zero) {
    xtensa_cpu_t cpu; setup(&cpu);
    put_insn3(&cpu, BASE, rrr(12, 2, 3, 4, 5));
    ar_write(&cpu, 4, 100);
    ar_write(&cpu, 5, 0);
    xtensa_step(&cpu);
    ASSERT_TRUE(cpu.exception);
    teardown(&cpu);
}

/* ===== MUL16U / MUL16S ===== */

TEST(exec_mul16u) {
    xtensa_cpu_t cpu; setup(&cpu);
    /* MUL16U: op2=12, op1=1 */
    put_insn3(&cpu, BASE, rrr(12, 1, 3, 4, 5));
    ar_write(&cpu, 4, 0x12340100);
    ar_write(&cpu, 5, 0x56780200);
    xtensa_step(&cpu);
    ASSERT_EQ(ar_read(&cpu, 3), 0x100 * 0x200); /* low 16-bits only */
    teardown(&cpu);
}

TEST(exec_mul16s) {
    xtensa_cpu_t cpu; setup(&cpu);
    /* MUL16S: op2=13, op1=1 */
    put_insn3(&cpu, BASE, rrr(13, 1, 3, 4, 5));
    ar_write(&cpu, 4, 0x0000FFFF); /* -1 as signed 16-bit */
    ar_write(&cpu, 5, 0x00000005);
    xtensa_step(&cpu);
    ASSERT_EQ(ar_read(&cpu, 3), (uint32_t)-5);
    teardown(&cpu);
}

void run_alu_tests(void) {
    TEST_SUITE("ALU Execution");

    RUN_TEST(exec_add_basic);
    RUN_TEST(exec_add_overflow);
    RUN_TEST(exec_add_zero);
    RUN_TEST(exec_sub_basic);
    RUN_TEST(exec_sub_underflow);
    RUN_TEST(exec_sub_self);
    RUN_TEST(exec_addx2);
    RUN_TEST(exec_addx4);
    RUN_TEST(exec_addx8);
    RUN_TEST(exec_subx2);
    RUN_TEST(exec_subx4);
    RUN_TEST(exec_neg_positive);
    RUN_TEST(exec_neg_negative);
    RUN_TEST(exec_neg_zero);
    RUN_TEST(exec_abs_positive);
    RUN_TEST(exec_abs_negative);
    RUN_TEST(exec_abs_intmin);
    RUN_TEST(exec_and);
    RUN_TEST(exec_or);
    RUN_TEST(exec_xor);
    RUN_TEST(exec_and_with_zero);
    RUN_TEST(exec_or_with_allones);
    RUN_TEST(exec_addi_positive);
    RUN_TEST(exec_addi_negative);
    RUN_TEST(exec_addi_zero);
    RUN_TEST(exec_addmi_basic);
    RUN_TEST(exec_addmi_negative);
    RUN_TEST(exec_movi_positive);
    RUN_TEST(exec_movi_negative);
    RUN_TEST(exec_add_n);
    RUN_TEST(exec_addi_n_basic);
    RUN_TEST(exec_addi_n_minus1);
    RUN_TEST(exec_movi_n_basic);
    RUN_TEST(exec_movi_n_90);
    RUN_TEST(exec_movi_n_negative);
    RUN_TEST(exec_nop);
    RUN_TEST(exec_nop_n);
    RUN_TEST(exec_ill);
    RUN_TEST(exec_break);
    RUN_TEST(exec_salt_less);
    RUN_TEST(exec_salt_equal);
    RUN_TEST(exec_saltu_unsigned);
    RUN_TEST(exec_mull_basic);
    RUN_TEST(exec_mull_overflow);
    RUN_TEST(exec_muluh);
    RUN_TEST(exec_sext_8bit);
    RUN_TEST(exec_sext_8bit_positive);
    RUN_TEST(exec_sext_16bit);
    RUN_TEST(exec_clamps_in_range);
    RUN_TEST(exec_clamps_above);
    RUN_TEST(exec_clamps_below);
    RUN_TEST(exec_min);
    RUN_TEST(exec_min_signed);
    RUN_TEST(exec_max);
    RUN_TEST(exec_minu);
    RUN_TEST(exec_maxu);
    RUN_TEST(exec_quou);
    RUN_TEST(exec_remu);
    RUN_TEST(exec_quos);
    RUN_TEST(exec_div_by_zero);
    RUN_TEST(exec_mul16u);
    RUN_TEST(exec_mul16s);
}
