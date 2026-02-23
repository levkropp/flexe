/*
 * Integration tests: multi-instruction programs (M4).
 */
#include "test_helpers.h"
#include <string.h>

/* Build RRI8 instruction */
static uint32_t intg_rri8(int subop, int s, int t, int imm8) {
    return (uint32_t)(((imm8 & 0xFF) << 16) | (subop << 12) | (s << 8) | (t << 4) | 2);
}

/* Build LOOP instruction: op0=6, n=3, m=1, r=8, s, imm8 */
static uint32_t intg_loop(int s, int imm8) {
    return (uint32_t)(((imm8 & 0xFF) << 16) | (8 << 12) | (s << 8) | (1 << 6) | (3 << 4) | 6);
}

/*
 * Fibonacci: compute fib(10) = 55
 *
 * Algorithm:
 *   a3 = 0  (fib_prev)
 *   a4 = 1  (fib_curr)
 *   a5 = 9  (count: after N iterations, a4 = fib(N+1))
 *   loop a5:
 *     a6 = a3 + a4
 *     a3 = a4
 *     a4 = a6
 *   result in a4
 *
 * Instructions:
 *   BASE+0:  MOVI a3, 0       (rri8: r=0xA, s=0, t=3, imm8=0)
 *   BASE+3:  MOVI a4, 1       (rri8: r=0xA, s=0, t=4, imm8=1)
 *   BASE+6:  MOVI a5, 9       (rri8: r=0xA, s=0, t=5, imm8=9)
 *   BASE+9:  LOOP a5, +6      (loop body is 3 instructions = 9 bytes, offset such that lend = BASE+18)
 *                              lend = (BASE+12) + offset + 1, want BASE+18, so offset = 5
 *   BASE+12: ADD a6, a3, a4   (rrr: op2=8, op1=0, r=6, s=3, t=4)
 *   BASE+15: MOV.N a3, a4     (narrow: op0=0xD, r=0, s=4, t=3)
 *   BASE+17: MOV.N a4, a6     (narrow: op0=0xD, r=0, s=6, t=4)
 *   BASE+19: (after loop)     — lend should be BASE+19
 *
 * Wait, the loop body has mixed 3-byte and 2-byte instructions.
 * ADD is 3 bytes (BASE+12 to BASE+14), MOV.N is 2 bytes (BASE+15 to BASE+16),
 * MOV.N is 2 bytes (BASE+17 to BASE+18). So lend = BASE+19.
 *
 * LOOP at BASE+9: cpu->pc = BASE+12 at exec time.
 * lend = BASE+12 + offset + 1, want BASE+19, so offset = 6.
 */
TEST(fibonacci_10) {
    xtensa_cpu_t cpu; setup(&cpu);
    cpu.running = true;

    /* MOVI a3, 0 */
    put_insn3(&cpu, BASE + 0, intg_rri8(0xA, 0, 3, 0));
    /* MOVI a4, 1 */
    put_insn3(&cpu, BASE + 3, intg_rri8(0xA, 0, 4, 1));
    /* MOVI a5, 9 */
    put_insn3(&cpu, BASE + 6, intg_rri8(0xA, 0, 5, 9));
    /* LOOP a5, offset=6 */
    put_insn3(&cpu, BASE + 9, intg_loop(5, 6));
    /* ADD a6, a3, a4 */
    put_insn3(&cpu, BASE + 12, rrr(8, 0, 6, 3, 4));
    /* MOV.N a3, a4 */
    put_insn2(&cpu, BASE + 15, narrow(0xD, 0, 4, 3));
    /* MOV.N a4, a6 */
    put_insn2(&cpu, BASE + 17, narrow(0xD, 0, 6, 4));

    /* Run: 3 setup + 1 loop + 9 * 3 body = 31 steps, then ILL at exit */
    int steps = xtensa_run(&cpu, 50);
    (void)steps;

    ASSERT_EQ(ar_read(&cpu, 4), 55); /* fib(10) = 55 */
    ASSERT_EQ(cpu.pc, BASE + 22);    /* after loop + ILL at exit */
    teardown(&cpu);
}

/*
 * Counter loop using branch:
 *   a3 = 0  (counter)
 *   a4 = 10 (limit)
 *   loop:
 *     ADDI a3, a3, 1
 *     BNE a3, a4, loop
 *   result: a3 = 10
 *
 * Instructions:
 *   BASE+0:  MOVI a3, 0
 *   BASE+3:  MOVI a4, 10
 *   BASE+6:  ADDI a3, a3, 1     (loop body)
 *   BASE+9:  BNE a3, a4, -7     (back to BASE+6: target = BASE + (-7) + 4 = BASE - 3? no)
 *
 * Branch offset: target = original_pc + offset + 4
 * original_pc of BNE = BASE+9. target = BASE+6.
 * BASE+6 = BASE+9 + offset + 4 -> offset = -7
 * imm8 for -7 signed = 0xF9
 */
TEST(counter_loop) {
    xtensa_cpu_t cpu; setup(&cpu);
    cpu.running = true;

    /* MOVI a3, 0 */
    put_insn3(&cpu, BASE + 0, intg_rri8(0xA, 0, 3, 0));
    /* MOVI a4, 10 */
    put_insn3(&cpu, BASE + 3, intg_rri8(0xA, 0, 4, 10));
    /* ADDI a3, a3, 1 */
    put_insn3(&cpu, BASE + 6, intg_rri8(0xC, 3, 3, 1));
    /* BNE a3, a4, -7: op0=7, r=9(BNE), s=3, t=4, imm8=0xF9(-7) */
    uint32_t bne = (uint32_t)(((0xF9) << 16) | (9 << 12) | (3 << 8) | (4 << 4) | 7);
    put_insn3(&cpu, BASE + 9, bne);

    /* Run: 2 setup + 10 * 2 body = 22 steps */
    int steps = xtensa_run(&cpu, 30);
    (void)steps;

    ASSERT_EQ(ar_read(&cpu, 3), 10);
    ASSERT_EQ(cpu.pc, BASE + 15); /* fell through after last BNE + ILL at exit */
    teardown(&cpu);
}

/*
 * Sum 1..5 using LOOP:
 *   a3 = 0  (sum)
 *   a4 = 1  (current value to add)
 *   a5 = 5  (loop count)
 *   loop a5:
 *     ADD a3, a3, a4
 *     ADDI a4, a4, 1
 *   result: a3 = 1+2+3+4+5 = 15
 *
 * Layout:
 *   BASE+0:  MOVI a3, 0
 *   BASE+3:  MOVI a4, 1
 *   BASE+6:  MOVI a5, 5
 *   BASE+9:  LOOP a5, offset=2  -> lend = BASE+12+2+1 = BASE+15
 *   BASE+12: ADD a3, a3, a4
 *   BASE+15: ...after loop. Wait, that's wrong. We need 2 instructions in the body.
 *
 * Let me recalculate:
 *   BASE+9:  LOOP a5, offset=5  -> lend = BASE+12+5+1 = BASE+18
 *   BASE+12: ADD a3, a3, a4    (3 bytes)
 *   BASE+15: ADDI a4, a4, 1   (3 bytes)
 *   BASE+18: (lend, after loop)
 */
TEST(sum_1_to_5) {
    xtensa_cpu_t cpu; setup(&cpu);
    cpu.running = true;

    put_insn3(&cpu, BASE + 0, intg_rri8(0xA, 0, 3, 0));  /* MOVI a3, 0 */
    put_insn3(&cpu, BASE + 3, intg_rri8(0xA, 0, 4, 1));  /* MOVI a4, 1 */
    put_insn3(&cpu, BASE + 6, intg_rri8(0xA, 0, 5, 5));  /* MOVI a5, 5 */
    put_insn3(&cpu, BASE + 9, intg_loop(5, 5));           /* LOOP a5, +5 */
    put_insn3(&cpu, BASE + 12, rrr(8, 0, 3, 3, 4));       /* ADD a3, a3, a4 */
    put_insn3(&cpu, BASE + 15, intg_rri8(0xC, 4, 4, 1));  /* ADDI a4, a4, 1 */

    xtensa_run(&cpu, 50);

    ASSERT_EQ(ar_read(&cpu, 3), 15); /* 1+2+3+4+5 */
    ASSERT_EQ(cpu.pc, BASE + 21); /* after loop + ILL at exit */
    teardown(&cpu);
}

void run_integration_tests(void) {
    TEST_SUITE("Integration Programs");

    RUN_TEST(fibonacci_10);
    RUN_TEST(counter_loop);
    RUN_TEST(sum_1_to_5);
}
