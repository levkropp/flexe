/*
 * Tests for MAC16 multiply-accumulate instructions.
 */
#include "test_helpers.h"
#include <string.h>

/*
 * MAC16 encoding: op0=4
 * byte0 = [t:op0] = (t<<4)|4
 * byte1 = [r:s]   = (r<<4)|s
 * byte2 = [op2:op1]
 */
static uint32_t mac16_insn(int op2, int op1, int r, int s, int t) {
    return (uint32_t)((op2 << 20) | (op1 << 16) | (r << 12) | (s << 8) | (t << 4) | 4);
}

/* ===== MUL.AA.LL ===== */

TEST(exec_mul_aa_ll) {
    xtensa_cpu_t cpu; setup(&cpu);
    /* MUL.AA.LL as=a2, at=a3: op1[3:2]=0(MUL), op1[1:0]=0(LL), op2[3:2]=0(AA)
       op1=0, op2=0, r=0, s=2, t=3 */
    put_insn3(&cpu, BASE, mac16_insn(0, 0, 0, 2, 3));
    ar_write(&cpu, 2, 0x00030005); /* low16 = 5 */
    ar_write(&cpu, 3, 0x00070003); /* low16 = 3 */
    xtensa_step(&cpu);
    ASSERT_EQ(cpu.acclo, 15); /* 5 * 3 = 15 */
    ASSERT_EQ(cpu.acchi, 0);
    teardown(&cpu);
}

/* ===== MUL.AA.HH ===== */

TEST(exec_mul_aa_hh) {
    xtensa_cpu_t cpu; setup(&cpu);
    /* MUL.AA.HH: op1=3(HH), op2=0(AA) */
    put_insn3(&cpu, BASE, mac16_insn(0, 3, 0, 2, 3));
    ar_write(&cpu, 2, 0x00040000); /* high16 = 4 */
    ar_write(&cpu, 3, 0x00060000); /* high16 = 6 */
    xtensa_step(&cpu);
    ASSERT_EQ(cpu.acclo, 24); /* 4 * 6 = 24 */
    ASSERT_EQ(cpu.acchi, 0);
    teardown(&cpu);
}

/* ===== MULA.AA.LL ===== */

TEST(exec_mula_aa_ll) {
    xtensa_cpu_t cpu; setup(&cpu);
    /* MULA.AA.LL: op1[3:2]=1(MULA), op1[1:0]=0(LL), op2=0(AA)
       op1=4, op2=0 */
    put_insn3(&cpu, BASE, mac16_insn(0, 4, 0, 2, 3));
    cpu.acclo = 100;
    cpu.acchi = 0;
    ar_write(&cpu, 2, 10);
    ar_write(&cpu, 3, 5);
    xtensa_step(&cpu);
    ASSERT_EQ(cpu.acclo, 150); /* 100 + 10*5 = 150 */
    teardown(&cpu);
}

/* ===== MULS.AA.LL ===== */

TEST(exec_muls_aa_ll) {
    xtensa_cpu_t cpu; setup(&cpu);
    /* MULS.AA.LL: op1[3:2]=2(MULS), op1[1:0]=0(LL), op2=0(AA)
       op1=8, op2=0 */
    put_insn3(&cpu, BASE, mac16_insn(0, 8, 0, 2, 3));
    cpu.acclo = 100;
    cpu.acchi = 0;
    ar_write(&cpu, 2, 10);
    ar_write(&cpu, 3, 5);
    xtensa_step(&cpu);
    ASSERT_EQ(cpu.acclo, 50); /* 100 - 10*5 = 50 */
    teardown(&cpu);
}

/* ===== MUL.DA.LL ===== */

TEST(exec_mul_da_ll) {
    xtensa_cpu_t cpu; setup(&cpu);
    /* MUL.DA.LL: op1=0(MUL,LL), op2[3:2]=2(DA) → op2=8
       src1=mr[s/2], src2=ar[t] */
    put_insn3(&cpu, BASE, mac16_insn(8, 0, 0, 2, 3));
    cpu.mr[1] = 7; /* mr[2/2] = mr[1], low16=7 */
    ar_write(&cpu, 3, 6); /* low16=6 */
    xtensa_step(&cpu);
    ASSERT_EQ(cpu.acclo, 42); /* 7 * 6 = 42 */
    teardown(&cpu);
}

/* ===== MUL.DD.LL ===== */

TEST(exec_mul_dd_ll) {
    xtensa_cpu_t cpu; setup(&cpu);
    /* MUL.DD.LL: op1=0(MUL,LL), op2[3:2]=3(DD) → op2=12 */
    put_insn3(&cpu, BASE, mac16_insn(12, 0, 0, 2, 2));
    cpu.mr[1] = 8;  /* mr[2/2] = mr[1] */
    cpu.mr[1] = 8;  /* mr[2/2] = mr[1], low16=8 */
    xtensa_step(&cpu);
    ASSERT_EQ(cpu.acclo, 64); /* 8 * 8 = 64 */
    teardown(&cpu);
}

/* ===== LDDEC ===== */

TEST(exec_lddec_basic) {
    xtensa_cpu_t cpu; setup(&cpu);
    /* LDDEC mr[0], a4: op2=4, op1=0, r=0 (mr[0/4]=mr[0]), s=4, t=0 */
    put_insn3(&cpu, BASE, mac16_insn(4, 0, 0, 4, 0));
    uint32_t addr = BASE + 0x100;
    ar_write(&cpu, 4, addr);
    mem_write32(cpu.mem, addr, 0xDEADBEEF);
    xtensa_step(&cpu);
    ASSERT_EQ(cpu.mr[0], 0xDEADBEEF);
    ASSERT_EQ(ar_read(&cpu, 4), addr - 4);
    teardown(&cpu);
}

/* ===== LDINC ===== */

TEST(exec_ldinc_basic) {
    xtensa_cpu_t cpu; setup(&cpu);
    /* LDINC mr[0], a4: op2=5, op1=0 */
    put_insn3(&cpu, BASE, mac16_insn(5, 0, 0, 4, 0));
    uint32_t addr = BASE + 0x100;
    ar_write(&cpu, 4, addr);
    mem_write32(cpu.mem, addr, 0xCAFEBABE);
    xtensa_step(&cpu);
    ASSERT_EQ(cpu.mr[0], 0xCAFEBABE);
    ASSERT_EQ(ar_read(&cpu, 4), addr + 4);
    teardown(&cpu);
}

/* ===== 40-bit accumulator ===== */

TEST(exec_acc_overflow_40bit) {
    xtensa_cpu_t cpu; setup(&cpu);
    /* Set accumulator near 40-bit boundary, then add */
    cpu.acclo = 0xFFFFFFF0;
    cpu.acchi = 0x7F; /* large positive 40-bit value */
    /* MULA.AA.LL with small product */
    put_insn3(&cpu, BASE, mac16_insn(0, 4, 0, 2, 3));
    ar_write(&cpu, 2, 1);
    ar_write(&cpu, 3, 0x20); /* product = 32 */
    xtensa_step(&cpu);
    /* acclo should wrap: 0xFFFFFFF0 + 32 = 0x00000010, acchi should become 0x80 */
    ASSERT_EQ(cpu.acclo, 0x00000010);
    ASSERT_EQ(cpu.acchi, 0x80);
    teardown(&cpu);
}

/* ===== Negative multiplication ===== */

TEST(exec_mul_negative) {
    xtensa_cpu_t cpu; setup(&cpu);
    /* MUL.AA.LL with signed negative halves */
    put_insn3(&cpu, BASE, mac16_insn(0, 0, 0, 2, 3));
    ar_write(&cpu, 2, 0xFFFE); /* low16 = -2 as int16 */
    ar_write(&cpu, 3, 0x0003); /* low16 = 3 */
    xtensa_step(&cpu);
    /* -2 * 3 = -6 → acclo=0xFFFFFFFA, acchi=0xFF */
    ASSERT_EQ(cpu.acclo, 0xFFFFFFFA);
    ASSERT_EQ(cpu.acchi, 0xFF);
    teardown(&cpu);
}

/* ===== UMUL unsigned ===== */

TEST(exec_umul_unsigned) {
    xtensa_cpu_t cpu; setup(&cpu);
    /* UMUL.AA.LL: op1[3:2]=3(UMUL), op1[1:0]=0(LL), op2=0(AA) → op1=12 */
    put_insn3(&cpu, BASE, mac16_insn(0, 12, 0, 2, 3));
    ar_write(&cpu, 2, 0xFFFE); /* low16 as unsigned = 65534 */
    ar_write(&cpu, 3, 0x0002); /* low16 = 2 */
    xtensa_step(&cpu);
    /* 65534 * 2 = 131068 = 0x0001FFFC */
    ASSERT_EQ(cpu.acclo, 0x0001FFFC);
    ASSERT_EQ(cpu.acchi, 0);
    teardown(&cpu);
}

void run_mac16_tests(void) {
    TEST_SUITE("MAC16 Instructions");
    RUN_TEST(exec_mul_aa_ll);
    RUN_TEST(exec_mul_aa_hh);
    RUN_TEST(exec_mula_aa_ll);
    RUN_TEST(exec_muls_aa_ll);
    RUN_TEST(exec_mul_da_ll);
    RUN_TEST(exec_mul_dd_ll);
    RUN_TEST(exec_lddec_basic);
    RUN_TEST(exec_ldinc_basic);
    RUN_TEST(exec_acc_overflow_40bit);
    RUN_TEST(exec_mul_negative);
    RUN_TEST(exec_umul_unsigned);
}
