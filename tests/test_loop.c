/*
 * Tests for zero-overhead loop instructions (M4).
 */
#include "test_helpers.h"
#include <string.h>

/* Build LOOP/LOOPNEZ/LOOPGTZ instruction: op0=6, n=3, m=1, r=8/9/10, s, imm8 */
static uint32_t loop_insn(int r, int s, int imm8) {
    return (uint32_t)(((imm8 & 0xFF) << 16) | (r << 12) | (s << 8) | (1 << 6) | (3 << 4) | 6);
}

/* Build RRI8 instruction */
static uint32_t loop_rri8(int subop, int s, int t, int imm8) {
    return (uint32_t)(((imm8 & 0xFF) << 16) | (subop << 12) | (s << 8) | (t << 4) | 2);
}

/* ===== LOOP ===== */

TEST(loop_basic) {
    xtensa_cpu_t cpu; setup(&cpu);
    /* Set up a loop that increments a3 five times.
     * LOOP a4, end   (a4 = 5)
     * ADDI a3, a3, 1 (loop body)
     * end:           (LEND points here)
     *
     * LOOP instruction at BASE.
     * Loop body = ADDI at BASE+3.
     * LEND = target = BASE + 3 + offset + 1 = ?
     * We want LEND = BASE+6 (right after the ADDI).
     * target = cpu->pc + offset + 1, where cpu->pc = BASE+3 at exec time.
     * BASE+6 = BASE+3 + offset + 1 -> offset = 2
     */
    put_insn3(&cpu, BASE, loop_insn(8, 4, 2));
    /* ADDI a3, a3, 1 */
    put_insn3(&cpu, BASE + 3, loop_rri8(0xC, 3, 3, 1));

    ar_write(&cpu, 3, 0);
    ar_write(&cpu, 4, 5);
    /* Execute LOOP setup */
    xtensa_step(&cpu);
    ASSERT_EQ(cpu.lbeg, BASE + 3);
    ASSERT_EQ(cpu.lend, BASE + 6);
    ASSERT_EQ(cpu.lcount, 4); /* 5 - 1 */

    /* Execute loop body 5 times */
    for (int i = 0; i < 5; i++)
        xtensa_step(&cpu);

    ASSERT_EQ(ar_read(&cpu, 3), 5);
    ASSERT_EQ(cpu.lcount, 0);
    ASSERT_EQ(cpu.pc, BASE + 6); /* exited loop */
    teardown(&cpu);
}

TEST(loop_count_one) {
    xtensa_cpu_t cpu; setup(&cpu);
    put_insn3(&cpu, BASE, loop_insn(8, 4, 2));
    put_insn3(&cpu, BASE + 3, loop_rri8(0xC, 3, 3, 1));

    ar_write(&cpu, 3, 0);
    ar_write(&cpu, 4, 1); /* Loop count = 1 */
    xtensa_step(&cpu);
    ASSERT_EQ(cpu.lcount, 0);

    xtensa_step(&cpu); /* Execute body once */
    ASSERT_EQ(ar_read(&cpu, 3), 1);
    ASSERT_EQ(cpu.pc, BASE + 6); /* exited */
    teardown(&cpu);
}

/* ===== LOOPNEZ ===== */

TEST(loopnez_nonzero) {
    xtensa_cpu_t cpu; setup(&cpu);
    put_insn3(&cpu, BASE, loop_insn(9, 4, 2));
    put_insn3(&cpu, BASE + 3, loop_rri8(0xC, 3, 3, 1));

    ar_write(&cpu, 3, 0);
    ar_write(&cpu, 4, 3);
    xtensa_step(&cpu);
    ASSERT_EQ(cpu.lcount, 2);
    ASSERT_EQ(cpu.pc, BASE + 3); /* enters loop */

    for (int i = 0; i < 3; i++)
        xtensa_step(&cpu);

    ASSERT_EQ(ar_read(&cpu, 3), 3);
    teardown(&cpu);
}

TEST(loopnez_zero_skips) {
    xtensa_cpu_t cpu; setup(&cpu);
    put_insn3(&cpu, BASE, loop_insn(9, 4, 2));
    put_insn3(&cpu, BASE + 3, loop_rri8(0xC, 3, 3, 1));

    ar_write(&cpu, 3, 0);
    ar_write(&cpu, 4, 0); /* zero -> skip */
    xtensa_step(&cpu);
    ASSERT_EQ(cpu.pc, BASE + 6); /* jumped past loop */
    ASSERT_EQ(ar_read(&cpu, 3), 0); /* body not executed */
    teardown(&cpu);
}

/* ===== LOOPGTZ ===== */

TEST(loopgtz_positive) {
    xtensa_cpu_t cpu; setup(&cpu);
    put_insn3(&cpu, BASE, loop_insn(10, 4, 2));
    put_insn3(&cpu, BASE + 3, loop_rri8(0xC, 3, 3, 1));

    ar_write(&cpu, 3, 0);
    ar_write(&cpu, 4, 2);
    xtensa_step(&cpu);
    ASSERT_EQ(cpu.lcount, 1);
    ASSERT_EQ(cpu.pc, BASE + 3);

    for (int i = 0; i < 2; i++)
        xtensa_step(&cpu);

    ASSERT_EQ(ar_read(&cpu, 3), 2);
    teardown(&cpu);
}

TEST(loopgtz_zero_skips) {
    xtensa_cpu_t cpu; setup(&cpu);
    put_insn3(&cpu, BASE, loop_insn(10, 4, 2));
    ar_write(&cpu, 4, 0);
    xtensa_step(&cpu);
    ASSERT_EQ(cpu.pc, BASE + 6);
    teardown(&cpu);
}

TEST(loopgtz_negative_skips) {
    xtensa_cpu_t cpu; setup(&cpu);
    put_insn3(&cpu, BASE, loop_insn(10, 4, 2));
    ar_write(&cpu, 4, (uint32_t)-1);
    xtensa_step(&cpu);
    ASSERT_EQ(cpu.pc, BASE + 6);
    teardown(&cpu);
}

void run_loop_tests(void) {
    TEST_SUITE("Loop Execution");

    RUN_TEST(loop_basic);
    RUN_TEST(loop_count_one);
    RUN_TEST(loopnez_nonzero);
    RUN_TEST(loopnez_zero_skips);
    RUN_TEST(loopgtz_positive);
    RUN_TEST(loopgtz_zero_skips);
    RUN_TEST(loopgtz_negative_skips);
}
