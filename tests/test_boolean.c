/*
 * Tests for boolean register operations, aggregation, and conditional moves.
 */
#include "test_helpers.h"
#include <string.h>

/* ===== ANDB ===== */

TEST(exec_andb_true) {
    xtensa_cpu_t cpu; setup(&cpu);
    /* ANDB b2, b3, b4: op2=0, op1=2, r=2, s=3, t=4 */
    put_insn3(&cpu, BASE, rrr(0, 2, 2, 3, 4));
    cpu.br = (1u << 3) | (1u << 4); /* b3=1, b4=1 */
    xtensa_step(&cpu);
    ASSERT_EQ((cpu.br >> 2) & 1, 1); /* b2 = 1&1 = 1 */
    teardown(&cpu);
}

TEST(exec_andb_false) {
    xtensa_cpu_t cpu; setup(&cpu);
    put_insn3(&cpu, BASE, rrr(0, 2, 2, 3, 4));
    cpu.br = (1u << 3); /* b3=1, b4=0 */
    xtensa_step(&cpu);
    ASSERT_EQ((cpu.br >> 2) & 1, 0); /* b2 = 1&0 = 0 */
    teardown(&cpu);
}

/* ===== ANDBC ===== */

TEST(exec_andbc_basic) {
    xtensa_cpu_t cpu; setup(&cpu);
    /* ANDBC b2, b3, b4: op2=1, op1=2 */
    put_insn3(&cpu, BASE, rrr(1, 2, 2, 3, 4));
    cpu.br = (1u << 3); /* b3=1, b4=0 */
    xtensa_step(&cpu);
    ASSERT_EQ((cpu.br >> 2) & 1, 1); /* b2 = 1 & ~0 = 1 */
    teardown(&cpu);
}

TEST(exec_andbc_both_set) {
    xtensa_cpu_t cpu; setup(&cpu);
    put_insn3(&cpu, BASE, rrr(1, 2, 2, 3, 4));
    cpu.br = (1u << 3) | (1u << 4); /* b3=1, b4=1 */
    xtensa_step(&cpu);
    ASSERT_EQ((cpu.br >> 2) & 1, 0); /* b2 = 1 & ~1 = 0 */
    teardown(&cpu);
}

/* ===== ORB ===== */

TEST(exec_orb_basic) {
    xtensa_cpu_t cpu; setup(&cpu);
    /* ORB b2, b3, b4: op2=2, op1=2 */
    put_insn3(&cpu, BASE, rrr(2, 2, 2, 3, 4));
    cpu.br = (1u << 4); /* b3=0, b4=1 */
    xtensa_step(&cpu);
    ASSERT_EQ((cpu.br >> 2) & 1, 1); /* b2 = 0|1 = 1 */
    teardown(&cpu);
}

TEST(exec_orb_both_zero) {
    xtensa_cpu_t cpu; setup(&cpu);
    put_insn3(&cpu, BASE, rrr(2, 2, 2, 3, 4));
    cpu.br = 0; /* b3=0, b4=0 */
    xtensa_step(&cpu);
    ASSERT_EQ((cpu.br >> 2) & 1, 0); /* b2 = 0|0 = 0 */
    teardown(&cpu);
}

/* ===== ORBC ===== */

TEST(exec_orbc_basic) {
    xtensa_cpu_t cpu; setup(&cpu);
    /* ORBC b2, b3, b4: op2=3, op1=2 */
    put_insn3(&cpu, BASE, rrr(3, 2, 2, 3, 4));
    cpu.br = 0; /* b3=0, b4=0 */
    xtensa_step(&cpu);
    ASSERT_EQ((cpu.br >> 2) & 1, 1); /* b2 = 0 | ~0 = 1 */
    teardown(&cpu);
}

/* ===== XORB ===== */

TEST(exec_xorb_same) {
    xtensa_cpu_t cpu; setup(&cpu);
    /* XORB b2, b3, b4: op2=4, op1=2 */
    put_insn3(&cpu, BASE, rrr(4, 2, 2, 3, 4));
    cpu.br = (1u << 3) | (1u << 4); /* b3=1, b4=1 */
    xtensa_step(&cpu);
    ASSERT_EQ((cpu.br >> 2) & 1, 0); /* b2 = 1^1 = 0 */
    teardown(&cpu);
}

TEST(exec_xorb_diff) {
    xtensa_cpu_t cpu; setup(&cpu);
    put_insn3(&cpu, BASE, rrr(4, 2, 2, 3, 4));
    cpu.br = (1u << 3); /* b3=1, b4=0 */
    xtensa_step(&cpu);
    ASSERT_EQ((cpu.br >> 2) & 1, 1); /* b2 = 1^0 = 1 */
    teardown(&cpu);
}

/* ===== ANY4 ===== */

TEST(exec_any4_some) {
    xtensa_cpu_t cpu; setup(&cpu);
    /* ANY4 bt, bs: op0=0, op1=0, op2=0, r=8, s=0, t=5 */
    put_insn3(&cpu, BASE, rrr(0, 0, 8, 0, 5));
    cpu.br = (1u << 2); /* b0=0, b1=0, b2=1, b3=0 → some set */
    xtensa_step(&cpu);
    ASSERT_EQ((cpu.br >> 5) & 1, 1); /* b5 = any of b0..b3 = 1 */
    teardown(&cpu);
}

TEST(exec_any4_none) {
    xtensa_cpu_t cpu; setup(&cpu);
    put_insn3(&cpu, BASE, rrr(0, 0, 8, 0, 5));
    cpu.br = 0; /* b0..b3 all 0 */
    xtensa_step(&cpu);
    ASSERT_EQ((cpu.br >> 5) & 1, 0);
    teardown(&cpu);
}

/* ===== ALL4 ===== */

TEST(exec_all4_all) {
    xtensa_cpu_t cpu; setup(&cpu);
    /* ALL4 bt, bs: r=9 */
    put_insn3(&cpu, BASE, rrr(0, 0, 9, 0, 5));
    cpu.br = 0xF; /* b0..b3 all set */
    xtensa_step(&cpu);
    ASSERT_EQ((cpu.br >> 5) & 1, 1);
    teardown(&cpu);
}

TEST(exec_all4_partial) {
    xtensa_cpu_t cpu; setup(&cpu);
    put_insn3(&cpu, BASE, rrr(0, 0, 9, 0, 5));
    cpu.br = 0x7; /* b0..b2 set, b3 clear */
    xtensa_step(&cpu);
    ASSERT_EQ((cpu.br >> 5) & 1, 0);
    teardown(&cpu);
}

/* ===== ANY8 ===== */

TEST(exec_any8_basic) {
    xtensa_cpu_t cpu; setup(&cpu);
    /* ANY8 bt, bs: r=10 */
    put_insn3(&cpu, BASE, rrr(0, 0, 10, 0, 9));
    cpu.br = (1u << 5); /* b5 set */
    xtensa_step(&cpu);
    ASSERT_EQ((cpu.br >> 9) & 1, 1);
    teardown(&cpu);
}

TEST(exec_any8_none) {
    xtensa_cpu_t cpu; setup(&cpu);
    put_insn3(&cpu, BASE, rrr(0, 0, 10, 0, 9));
    cpu.br = 0;
    xtensa_step(&cpu);
    ASSERT_EQ((cpu.br >> 9) & 1, 0);
    teardown(&cpu);
}

/* ===== ALL8 ===== */

TEST(exec_all8_basic) {
    xtensa_cpu_t cpu; setup(&cpu);
    /* ALL8 bt, bs: r=11 */
    put_insn3(&cpu, BASE, rrr(0, 0, 11, 0, 9));
    cpu.br = 0xFF; /* b0..b7 all set */
    xtensa_step(&cpu);
    ASSERT_EQ((cpu.br >> 9) & 1, 1);
    teardown(&cpu);
}

TEST(exec_all8_partial) {
    xtensa_cpu_t cpu; setup(&cpu);
    put_insn3(&cpu, BASE, rrr(0, 0, 11, 0, 9));
    cpu.br = 0xFE; /* b0 clear */
    xtensa_step(&cpu);
    ASSERT_EQ((cpu.br >> 9) & 1, 0);
    teardown(&cpu);
}

/* ===== MOVF ===== */

TEST(exec_movf_taken) {
    xtensa_cpu_t cpu; setup(&cpu);
    /* MOVF a3, a4, b5: op2=12, op1=3, r=3, s=4, t=5 */
    put_insn3(&cpu, BASE, rrr(12, 3, 3, 4, 5));
    ar_write(&cpu, 3, 99);
    ar_write(&cpu, 4, 42);
    cpu.br = 0; /* b5=0 → condition true for MOVF */
    xtensa_step(&cpu);
    ASSERT_EQ(ar_read(&cpu, 3), 42);
    teardown(&cpu);
}

TEST(exec_movf_not_taken) {
    xtensa_cpu_t cpu; setup(&cpu);
    put_insn3(&cpu, BASE, rrr(12, 3, 3, 4, 5));
    ar_write(&cpu, 3, 99);
    ar_write(&cpu, 4, 42);
    cpu.br = (1u << 5); /* b5=1 → no move */
    xtensa_step(&cpu);
    ASSERT_EQ(ar_read(&cpu, 3), 99);
    teardown(&cpu);
}

/* ===== MOVT ===== */

TEST(exec_movt_taken) {
    xtensa_cpu_t cpu; setup(&cpu);
    /* MOVT a3, a4, b5: op2=13, op1=3, r=3, s=4, t=5 */
    put_insn3(&cpu, BASE, rrr(13, 3, 3, 4, 5));
    ar_write(&cpu, 3, 99);
    ar_write(&cpu, 4, 42);
    cpu.br = (1u << 5); /* b5=1 → condition true for MOVT */
    xtensa_step(&cpu);
    ASSERT_EQ(ar_read(&cpu, 3), 42);
    teardown(&cpu);
}

TEST(exec_movt_not_taken) {
    xtensa_cpu_t cpu; setup(&cpu);
    put_insn3(&cpu, BASE, rrr(13, 3, 3, 4, 5));
    ar_write(&cpu, 3, 99);
    ar_write(&cpu, 4, 42);
    cpu.br = 0; /* b5=0 → no move */
    xtensa_step(&cpu);
    ASSERT_EQ(ar_read(&cpu, 3), 99);
    teardown(&cpu);
}

/* ===== RER/WER stubs ===== */

TEST(exec_rer_stub) {
    xtensa_cpu_t cpu; setup(&cpu);
    /* RER a3, a4: ST1 r=6, s=4, t=3 → op2=4, op1=0, r=6, s=4, t=3 */
    put_insn3(&cpu, BASE, rrr(4, 0, 6, 4, 3));
    ar_write(&cpu, 4, 0x1234);
    ar_write(&cpu, 3, 0xDEAD);
    xtensa_step(&cpu);
    ASSERT_EQ(ar_read(&cpu, 3), 0); /* stub returns 0 */
    teardown(&cpu);
}

TEST(exec_wer_stub) {
    xtensa_cpu_t cpu; setup(&cpu);
    /* WER a3, a4: ST1 r=7, s=4, t=3 */
    put_insn3(&cpu, BASE, rrr(4, 0, 7, 4, 3));
    ar_write(&cpu, 4, 0x1234);
    ar_write(&cpu, 3, 0xBEEF);
    xtensa_step(&cpu);
    ASSERT_EQ(cpu.pc, BASE + 3); /* just advances */
    teardown(&cpu);
}

void run_boolean_tests(void) {
    TEST_SUITE("Boolean Register Operations");
    RUN_TEST(exec_andb_true);
    RUN_TEST(exec_andb_false);
    RUN_TEST(exec_andbc_basic);
    RUN_TEST(exec_andbc_both_set);
    RUN_TEST(exec_orb_basic);
    RUN_TEST(exec_orb_both_zero);
    RUN_TEST(exec_orbc_basic);
    RUN_TEST(exec_xorb_same);
    RUN_TEST(exec_xorb_diff);
    RUN_TEST(exec_any4_some);
    RUN_TEST(exec_any4_none);
    RUN_TEST(exec_all4_all);
    RUN_TEST(exec_all4_partial);
    RUN_TEST(exec_any8_basic);
    RUN_TEST(exec_any8_none);
    RUN_TEST(exec_all8_basic);
    RUN_TEST(exec_all8_partial);
    RUN_TEST(exec_movf_taken);
    RUN_TEST(exec_movf_not_taken);
    RUN_TEST(exec_movt_taken);
    RUN_TEST(exec_movt_not_taken);
    RUN_TEST(exec_rer_stub);
    RUN_TEST(exec_wer_stub);
}
