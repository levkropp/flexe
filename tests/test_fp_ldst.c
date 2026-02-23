/*
 * Tests for FP load/store and user register (FCR/FSR) instructions.
 */
#include "test_helpers.h"
#include <string.h>

/*
 * LSCI encoding: op0=3
 * byte0 = [t:op0] = (t<<4)|3
 * byte1 = [r:s]   = (r<<4)|s
 * byte2 = imm8
 */
static uint32_t lsci(int subop, int s, int t, int imm8) {
    return (uint32_t)(((imm8 & 0xFF) << 16) | (subop << 12) | (s << 8) | (t << 4) | 3);
}

/* ===== LSI ===== */

TEST(exec_lsi_basic) {
    xtensa_cpu_t cpu; setup(&cpu);
    /* LSI f2, a4, 8: lr=0, ls=4, lt=2, imm8=2 (offset=2*4=8) */
    put_insn3(&cpu, BASE, lsci(0, 4, 2, 2));
    uint32_t addr = BASE + 0x100;
    ar_write(&cpu, 4, addr);
    float val = 3.14f;
    uint32_t bits;
    memcpy(&bits, &val, 4);
    mem_write32(cpu.mem, addr + 8, bits);
    xtensa_step(&cpu);
    uint32_t result;
    memcpy(&result, &cpu.fr[2], 4);
    ASSERT_EQ(result, bits);
    teardown(&cpu);
}

/* ===== SSI ===== */

TEST(exec_ssi_basic) {
    xtensa_cpu_t cpu; setup(&cpu);
    /* SSI f2, a4, 0: lr=4, ls=4, lt=2, imm8=0 */
    put_insn3(&cpu, BASE, lsci(4, 4, 2, 0));
    uint32_t addr = BASE + 0x100;
    ar_write(&cpu, 4, addr);
    float val = 2.718f;
    uint32_t bits;
    memcpy(&bits, &val, 4);
    memcpy(&cpu.fr[2], &bits, 4);
    xtensa_step(&cpu);
    ASSERT_EQ(mem_read32(cpu.mem, addr), bits);
    teardown(&cpu);
}

/* ===== LSI/SSI roundtrip ===== */

TEST(exec_lsi_ssi_roundtrip) {
    xtensa_cpu_t cpu; setup(&cpu);
    uint32_t addr = BASE + 0x200;
    ar_write(&cpu, 4, addr);

    /* Store a known float */
    float val = -1.5f;
    uint32_t bits;
    memcpy(&bits, &val, 4);
    memcpy(&cpu.fr[3], &bits, 4);

    /* SSI f3, a4, 0 */
    put_insn3(&cpu, BASE, lsci(4, 4, 3, 0));
    xtensa_step(&cpu);

    /* Clear f3 */
    memset(&cpu.fr[3], 0, 4);

    /* LSI f3, a4, 0 */
    cpu.pc = BASE;
    put_insn3(&cpu, BASE, lsci(0, 4, 3, 0));
    xtensa_step(&cpu);

    uint32_t result;
    memcpy(&result, &cpu.fr[3], 4);
    ASSERT_EQ(result, bits);
    teardown(&cpu);
}

/* ===== LSIU (post-update) ===== */

TEST(exec_lsiu_post_update) {
    xtensa_cpu_t cpu; setup(&cpu);
    /* LSIU f1, a5, 12: lr=8, ls=5, lt=1, imm8=3 (offset=3*4=12) */
    put_insn3(&cpu, BASE, lsci(8, 5, 1, 3));
    uint32_t addr = BASE + 0x100;
    ar_write(&cpu, 5, addr);
    uint32_t bits = 0x41200000; /* 10.0f */
    mem_write32(cpu.mem, addr + 12, bits);
    xtensa_step(&cpu);
    uint32_t result;
    memcpy(&result, &cpu.fr[1], 4);
    ASSERT_EQ(result, bits);
    ASSERT_EQ(ar_read(&cpu, 5), addr + 12); /* base updated */
    teardown(&cpu);
}

/* ===== SSIU (post-update) ===== */

TEST(exec_ssiu_post_update) {
    xtensa_cpu_t cpu; setup(&cpu);
    /* SSIU f1, a5, 4: lr=12, ls=5, lt=1, imm8=1 (offset=1*4=4) */
    put_insn3(&cpu, BASE, lsci(12, 5, 1, 1));
    uint32_t addr = BASE + 0x100;
    ar_write(&cpu, 5, addr);
    uint32_t bits = 0x40A00000; /* 5.0f */
    memcpy(&cpu.fr[1], &bits, 4);
    xtensa_step(&cpu);
    ASSERT_EQ(mem_read32(cpu.mem, addr + 4), bits);
    ASSERT_EQ(ar_read(&cpu, 5), addr + 4); /* base updated */
    teardown(&cpu);
}

/* ===== RUR/WUR for FCR/FSR ===== */

/*
 * RUR at: op2=14, op1=3, ur = (s<<4)|r
 * FCR=232: s=14, r=8 → ur=(14<<4)|8 = 232
 * dest = t
 */
static uint32_t rur_insn(int ur, int dest_t) {
    int s = (ur >> 4) & 0xF;
    int r = ur & 0xF;
    return rrr(14, 3, r, s, dest_t);
}

/*
 * WUR at: op2=15, op1=3, ur = (s<<4)|r
 * src = t
 */
static uint32_t wur_insn(int ur, int src_t) {
    int s = (ur >> 4) & 0xF;
    int r = ur & 0xF;
    return rrr(15, 3, r, s, src_t);
}

TEST(exec_rur_fcr) {
    xtensa_cpu_t cpu; setup(&cpu);
    cpu.fcr = 0x0000001F;
    /* RUR a3, FCR(232): dest=a3 → t=3 */
    put_insn3(&cpu, BASE, rur_insn(232, 3));
    xtensa_step(&cpu);
    ASSERT_EQ(ar_read(&cpu, 3), 0x0000001F);
    teardown(&cpu);
}

TEST(exec_wur_fcr) {
    xtensa_cpu_t cpu; setup(&cpu);
    ar_write(&cpu, 3, 0x0000000F);
    /* WUR FCR(232), a3: src=a3 → t=3 */
    put_insn3(&cpu, BASE, wur_insn(232, 3));
    xtensa_step(&cpu);
    ASSERT_EQ(cpu.fcr, 0x0000000F);
    teardown(&cpu);
}

TEST(exec_wur_fsr_rur_fsr) {
    xtensa_cpu_t cpu; setup(&cpu);
    ar_write(&cpu, 5, 0x00000078);

    /* WUR FSR(233), a5 */
    put_insn3(&cpu, BASE, wur_insn(233, 5));
    xtensa_step(&cpu);
    ASSERT_EQ(cpu.fsr, 0x00000078);

    /* RUR a6, FSR(233) */
    cpu.pc = BASE;
    put_insn3(&cpu, BASE, rur_insn(233, 6));
    xtensa_step(&cpu);
    ASSERT_EQ(ar_read(&cpu, 6), 0x00000078);
    teardown(&cpu);
}

void run_fp_ldst_tests(void) {
    TEST_SUITE("FP Load/Store & User Registers");
    RUN_TEST(exec_lsi_basic);
    RUN_TEST(exec_ssi_basic);
    RUN_TEST(exec_lsi_ssi_roundtrip);
    RUN_TEST(exec_lsiu_post_update);
    RUN_TEST(exec_ssiu_post_update);
    RUN_TEST(exec_rur_fcr);
    RUN_TEST(exec_wur_fcr);
    RUN_TEST(exec_wur_fsr_rur_fsr);
}
