/*
 * Tests for shift instruction execution.
 */
#include "test_helpers.h"
#include <string.h>

/* ===== SLLI ===== */

TEST(exec_slli_1) {
    xtensa_cpu_t cpu; setup(&cpu);
    /* SLLI a3, a4, 1: op2=0, op1=1, r=3, s=4, t=1 */
    put_insn3(&cpu, BASE, rrr(0, 1, 3, 4, 1));
    ar_write(&cpu, 4, 0x12345678);
    xtensa_step(&cpu);
    ASSERT_EQ(ar_read(&cpu, 3), 0x2468ACF0);
    teardown(&cpu);
}

TEST(exec_slli_31) {
    xtensa_cpu_t cpu; setup(&cpu);
    /* SLLI a3, a4, 31: op2=1(sh[4]=1), t=15(sh[3:0]=15), sh=16+15=31 */
    put_insn3(&cpu, BASE, rrr(1, 1, 3, 4, 15));
    ar_write(&cpu, 4, 1);
    xtensa_step(&cpu);
    ASSERT_EQ(ar_read(&cpu, 3), 0x80000000);
    teardown(&cpu);
}

TEST(exec_slli_0) {
    xtensa_cpu_t cpu; setup(&cpu);
    /* SLLI a3, a4, 0: op2=0, t=0 */
    put_insn3(&cpu, BASE, rrr(0, 1, 3, 4, 0));
    ar_write(&cpu, 4, 0xDEADBEEF);
    xtensa_step(&cpu);
    ASSERT_EQ(ar_read(&cpu, 3), 0xDEADBEEF);
    teardown(&cpu);
}

/* ===== SRAI ===== */

TEST(exec_srai_positive) {
    xtensa_cpu_t cpu; setup(&cpu);
    /* SRAI a3, a5, 4: op2=2, op1=1, r=3, s=4(sh[3:0]), t=5(src)
       sh = (op2&1)<<4 | s = 0<<4 | 4 = 4 */
    put_insn3(&cpu, BASE, rrr(2, 1, 3, 4, 5));
    ar_write(&cpu, 5, 0x00001000);
    xtensa_step(&cpu);
    ASSERT_EQ(ar_read(&cpu, 3), 0x00000100);
    teardown(&cpu);
}

TEST(exec_srai_negative) {
    xtensa_cpu_t cpu; setup(&cpu);
    put_insn3(&cpu, BASE, rrr(2, 1, 3, 1, 5));
    ar_write(&cpu, 5, 0x80000000); /* -2^31 */
    xtensa_step(&cpu);
    /* shift right 1 with sign extension: 0xC0000000 */
    ASSERT_EQ(ar_read(&cpu, 3), 0xC0000000);
    teardown(&cpu);
}

TEST(exec_srai_31) {
    xtensa_cpu_t cpu; setup(&cpu);
    /* SRAI a3, a5, 31: op2=3(sh[4]=1), s=15(sh[3:0]), sh=16+15=31 */
    put_insn3(&cpu, BASE, rrr(3, 1, 3, 15, 5));
    ar_write(&cpu, 5, 0x80000000); /* negative */
    xtensa_step(&cpu);
    ASSERT_EQ(ar_read(&cpu, 3), 0xFFFFFFFF); /* all ones from sign extension */
    teardown(&cpu);
}

/* ===== SRLI ===== */

TEST(exec_srli_1) {
    xtensa_cpu_t cpu; setup(&cpu);
    /* SRLI a3, a5, 1: op2=4, op1=1, r=3, s=1(shift), t=5(src) */
    put_insn3(&cpu, BASE, rrr(4, 1, 3, 1, 5));
    ar_write(&cpu, 5, 0x80000002);
    xtensa_step(&cpu);
    ASSERT_EQ(ar_read(&cpu, 3), 0x40000001); /* logical right shift, no sign ext */
    teardown(&cpu);
}

TEST(exec_srli_0) {
    xtensa_cpu_t cpu; setup(&cpu);
    put_insn3(&cpu, BASE, rrr(4, 1, 3, 0, 5));
    ar_write(&cpu, 5, 0xDEADBEEF);
    xtensa_step(&cpu);
    ASSERT_EQ(ar_read(&cpu, 3), 0xDEADBEEF);
    teardown(&cpu);
}

/* ===== SLL (shift by SAR) ===== */

TEST(exec_sll_sar1) {
    xtensa_cpu_t cpu; setup(&cpu);
    /* SLL a3, a4: op2=10, op1=1, r=3, s=4, t=0 (must be 0) */
    /* SLL: ar[r] = ar[s] << (32-SAR) */
    put_insn3(&cpu, BASE, rrr(10, 1, 3, 4, 0));
    ar_write(&cpu, 4, 0x00000001);
    cpu.sar = 31; /* shift = 32-31 = 1 */
    xtensa_step(&cpu);
    ASSERT_EQ(ar_read(&cpu, 3), 0x00000002);
    teardown(&cpu);
}

TEST(exec_sll_sar32) {
    xtensa_cpu_t cpu; setup(&cpu);
    put_insn3(&cpu, BASE, rrr(10, 1, 3, 4, 0));
    ar_write(&cpu, 4, 0xFFFFFFFF);
    cpu.sar = 0; /* shift = 32-0 = 32 → result 0 */
    xtensa_step(&cpu);
    ASSERT_EQ(ar_read(&cpu, 3), 0);
    teardown(&cpu);
}

/* ===== SRL (logical right by SAR) ===== */

TEST(exec_srl_sar0) {
    xtensa_cpu_t cpu; setup(&cpu);
    /* SRL a3, a5: op2=9, op1=1, r=3, s=0(must be 0), t=5 */
    put_insn3(&cpu, BASE, rrr(9, 1, 3, 0, 5));
    ar_write(&cpu, 5, 0xDEADBEEF);
    cpu.sar = 0;
    xtensa_step(&cpu);
    ASSERT_EQ(ar_read(&cpu, 3), 0xDEADBEEF);
    teardown(&cpu);
}

TEST(exec_srl_sar16) {
    xtensa_cpu_t cpu; setup(&cpu);
    put_insn3(&cpu, BASE, rrr(9, 1, 3, 0, 5));
    ar_write(&cpu, 5, 0x12340000);
    cpu.sar = 16;
    xtensa_step(&cpu);
    ASSERT_EQ(ar_read(&cpu, 3), 0x00001234);
    teardown(&cpu);
}

/* ===== SRA (arithmetic right by SAR) ===== */

TEST(exec_sra_positive) {
    xtensa_cpu_t cpu; setup(&cpu);
    /* SRA a3, a5: op2=11, op1=1, r=3, s=0(must be 0), t=5 */
    put_insn3(&cpu, BASE, rrr(11, 1, 3, 0, 5));
    ar_write(&cpu, 5, 0x40000000);
    cpu.sar = 1;
    xtensa_step(&cpu);
    ASSERT_EQ(ar_read(&cpu, 3), 0x20000000);
    teardown(&cpu);
}

TEST(exec_sra_negative) {
    xtensa_cpu_t cpu; setup(&cpu);
    put_insn3(&cpu, BASE, rrr(11, 1, 3, 0, 5));
    ar_write(&cpu, 5, 0x80000000);
    cpu.sar = 1;
    xtensa_step(&cpu);
    ASSERT_EQ(ar_read(&cpu, 3), 0xC0000000);
    teardown(&cpu);
}

/* ===== SRC (funnel shift) ===== */

TEST(exec_src_sar0) {
    xtensa_cpu_t cpu; setup(&cpu);
    /* SRC a3, a4, a5: op2=8, op1=1, r=3, s=4, t=5 */
    /* SRC: ar[r] = (ar[s] << (32-SAR)) | (ar[t] >> SAR) */
    /* SAR=0: result = ar[t] */
    put_insn3(&cpu, BASE, rrr(8, 1, 3, 4, 5));
    ar_write(&cpu, 4, 0xAAAAAAAA);
    ar_write(&cpu, 5, 0x55555555);
    cpu.sar = 0;
    xtensa_step(&cpu);
    ASSERT_EQ(ar_read(&cpu, 3), 0x55555555);
    teardown(&cpu);
}

TEST(exec_src_sar16) {
    xtensa_cpu_t cpu; setup(&cpu);
    put_insn3(&cpu, BASE, rrr(8, 1, 3, 4, 5));
    ar_write(&cpu, 4, 0x12345678);
    ar_write(&cpu, 5, 0x9ABCDEF0);
    cpu.sar = 16;
    xtensa_step(&cpu);
    /* (0x12345678 << 16) | (0x9ABCDEF0 >> 16) = 0x56789ABC */
    ASSERT_EQ(ar_read(&cpu, 3), 0x56789ABC);
    teardown(&cpu);
}

TEST(exec_src_sar8) {
    xtensa_cpu_t cpu; setup(&cpu);
    put_insn3(&cpu, BASE, rrr(8, 1, 3, 4, 5));
    ar_write(&cpu, 4, 0xAA000000);
    ar_write(&cpu, 5, 0x00BB0000);
    cpu.sar = 8;
    xtensa_step(&cpu);
    /* (0xAA000000 << 24) | (0x00BB0000 >> 8) = 0x0000BB00 */
    ASSERT_EQ(ar_read(&cpu, 3), 0x0000BB00);
    teardown(&cpu);
}

/* ===== EXTUI ===== */

TEST(exec_extui_bit0) {
    xtensa_cpu_t cpu; setup(&cpu);
    /* EXTUI a3, a5, 0, 1: shift=0, mask=1 bit
       op2=0(mask-1), op1=4(010+sh[4]=0), r=3, s=0(sh[3:0]), t=5 */
    put_insn3(&cpu, BASE, rrr(0, 4, 3, 0, 5));
    ar_write(&cpu, 5, 0xFFFFFFFF);
    xtensa_step(&cpu);
    ASSERT_EQ(ar_read(&cpu, 3), 1);
    teardown(&cpu);
}

TEST(exec_extui_byte1) {
    xtensa_cpu_t cpu; setup(&cpu);
    /* EXTUI a3, a5, 8, 8: shift=8, mask=8 bits
       op2=7(8-1), op1=4, r=3, s=8, t=5 */
    put_insn3(&cpu, BASE, rrr(7, 4, 3, 8, 5));
    ar_write(&cpu, 5, 0x12345678);
    xtensa_step(&cpu);
    ASSERT_EQ(ar_read(&cpu, 3), 0x56);
    teardown(&cpu);
}

TEST(exec_extui_high_byte) {
    xtensa_cpu_t cpu; setup(&cpu);
    /* EXTUI a3, a5, 24, 8: shift=24, mask=8 bits
       sh=24: sh[4]=1→op1=5, sh[3:0]=8→s=8, op2=7, r=3, t=5 */
    put_insn3(&cpu, BASE, rrr(7, 5, 3, 8, 5));
    ar_write(&cpu, 5, 0x12345678);
    xtensa_step(&cpu);
    ASSERT_EQ(ar_read(&cpu, 3), 0x12);
    teardown(&cpu);
}

/* ===== SSR / SSL / SSAI ===== */

TEST(exec_ssr_then_srl) {
    xtensa_cpu_t cpu; setup(&cpu);
    /* SSR a4: op2=4, op1=0, r=0, s=4, t=0 → SAR = ar[s] & 31 */
    put_insn3(&cpu, BASE, rrr(4, 0, 0, 4, 0));
    /* SRL a3, a5: op2=9, op1=1, r=3, s=0, t=5 */
    put_insn3(&cpu, BASE + 3, rrr(9, 1, 3, 0, 5));
    ar_write(&cpu, 4, 8);
    ar_write(&cpu, 5, 0x00001200);
    xtensa_step(&cpu); /* SSR */
    ASSERT_EQ(cpu.sar, 8);
    xtensa_step(&cpu); /* SRL */
    ASSERT_EQ(ar_read(&cpu, 3), 0x12);
    teardown(&cpu);
}

TEST(exec_ssl_then_sll) {
    xtensa_cpu_t cpu; setup(&cpu);
    /* SSL a4: op2=4, op1=0, r=1, s=4, t=0 → SAR = 32 - (ar[s] & 31) */
    put_insn3(&cpu, BASE, rrr(4, 0, 1, 4, 0));
    /* SLL a3, a5: op2=10, op1=1, r=3, s=5, t=0 → ar[r] = ar[s] << (32-SAR) */
    put_insn3(&cpu, BASE + 3, rrr(10, 1, 3, 5, 0));
    ar_write(&cpu, 4, 4); /* SSL: SAR = 32-4 = 28 */
    ar_write(&cpu, 5, 1);
    xtensa_step(&cpu); /* SSL */
    ASSERT_EQ(cpu.sar, 28);
    xtensa_step(&cpu); /* SLL: shift = 32-28 = 4 */
    ASSERT_EQ(ar_read(&cpu, 3), 16); /* 1 << 4 */
    teardown(&cpu);
}

TEST(exec_ssai) {
    xtensa_cpu_t cpu; setup(&cpu);
    /* SSAI 15: op2=4, op1=0, r=4, s=15, t=0 → SAR = (s | ((t&1)<<4)) = 15 */
    put_insn3(&cpu, BASE, rrr(4, 0, 4, 15, 0));
    xtensa_step(&cpu);
    ASSERT_EQ(cpu.sar, 15);
    teardown(&cpu);
}

TEST(exec_ssai_large) {
    xtensa_cpu_t cpu; setup(&cpu);
    /* SSAI 20: s=4, t=1 → SAR = 4 | (1<<4) = 20 */
    put_insn3(&cpu, BASE, rrr(4, 0, 4, 4, 1));
    xtensa_step(&cpu);
    ASSERT_EQ(cpu.sar, 20);
    teardown(&cpu);
}

void run_shift_tests(void) {
    TEST_SUITE("Shift Execution");

    RUN_TEST(exec_slli_1);
    RUN_TEST(exec_slli_31);
    RUN_TEST(exec_slli_0);
    RUN_TEST(exec_srai_positive);
    RUN_TEST(exec_srai_negative);
    RUN_TEST(exec_srai_31);
    RUN_TEST(exec_srli_1);
    RUN_TEST(exec_srli_0);
    RUN_TEST(exec_sll_sar1);
    RUN_TEST(exec_sll_sar32);
    RUN_TEST(exec_srl_sar0);
    RUN_TEST(exec_srl_sar16);
    RUN_TEST(exec_sra_positive);
    RUN_TEST(exec_sra_negative);
    RUN_TEST(exec_src_sar0);
    RUN_TEST(exec_src_sar16);
    RUN_TEST(exec_src_sar8);
    RUN_TEST(exec_extui_bit0);
    RUN_TEST(exec_extui_byte1);
    RUN_TEST(exec_extui_high_byte);
    RUN_TEST(exec_ssr_then_srl);
    RUN_TEST(exec_ssl_then_sll);
    RUN_TEST(exec_ssai);
    RUN_TEST(exec_ssai_large);
}
