/*
 * Tests for move and special register instruction execution.
 */
#include "test_helpers.h"
#include <string.h>

/* ===== MOVEQZ ===== */

TEST(exec_moveqz_true) {
    xtensa_cpu_t cpu; setup(&cpu);
    /* MOVEQZ a3, a4, a5: op2=8, op1=3 */
    put_insn3(&cpu, BASE, rrr(8, 3, 3, 4, 5));
    ar_write(&cpu, 3, 0);
    ar_write(&cpu, 4, 42);
    ar_write(&cpu, 5, 0); /* condition: t==0 → move */
    xtensa_step(&cpu);
    ASSERT_EQ(ar_read(&cpu, 3), 42);
    teardown(&cpu);
}

TEST(exec_moveqz_false) {
    xtensa_cpu_t cpu; setup(&cpu);
    put_insn3(&cpu, BASE, rrr(8, 3, 3, 4, 5));
    ar_write(&cpu, 3, 99);
    ar_write(&cpu, 4, 42);
    ar_write(&cpu, 5, 1); /* condition: t!=0 → no move */
    xtensa_step(&cpu);
    ASSERT_EQ(ar_read(&cpu, 3), 99); /* unchanged */
    teardown(&cpu);
}

/* ===== MOVNEZ ===== */

TEST(exec_movnez_true) {
    xtensa_cpu_t cpu; setup(&cpu);
    put_insn3(&cpu, BASE, rrr(9, 3, 3, 4, 5));
    ar_write(&cpu, 3, 0);
    ar_write(&cpu, 4, 42);
    ar_write(&cpu, 5, 7); /* condition: t!=0 → move */
    xtensa_step(&cpu);
    ASSERT_EQ(ar_read(&cpu, 3), 42);
    teardown(&cpu);
}

TEST(exec_movnez_false) {
    xtensa_cpu_t cpu; setup(&cpu);
    put_insn3(&cpu, BASE, rrr(9, 3, 3, 4, 5));
    ar_write(&cpu, 3, 99);
    ar_write(&cpu, 4, 42);
    ar_write(&cpu, 5, 0); /* condition: t==0 → no move */
    xtensa_step(&cpu);
    ASSERT_EQ(ar_read(&cpu, 3), 99);
    teardown(&cpu);
}

/* ===== MOVLTZ ===== */

TEST(exec_movltz_true) {
    xtensa_cpu_t cpu; setup(&cpu);
    put_insn3(&cpu, BASE, rrr(10, 3, 3, 4, 5));
    ar_write(&cpu, 3, 0);
    ar_write(&cpu, 4, 42);
    ar_write(&cpu, 5, (uint32_t)-1); /* condition: t<0 → move */
    xtensa_step(&cpu);
    ASSERT_EQ(ar_read(&cpu, 3), 42);
    teardown(&cpu);
}

TEST(exec_movltz_false_zero) {
    xtensa_cpu_t cpu; setup(&cpu);
    put_insn3(&cpu, BASE, rrr(10, 3, 3, 4, 5));
    ar_write(&cpu, 3, 99);
    ar_write(&cpu, 4, 42);
    ar_write(&cpu, 5, 0); /* condition: t==0, not <0 → no move */
    xtensa_step(&cpu);
    ASSERT_EQ(ar_read(&cpu, 3), 99);
    teardown(&cpu);
}

TEST(exec_movltz_false_positive) {
    xtensa_cpu_t cpu; setup(&cpu);
    put_insn3(&cpu, BASE, rrr(10, 3, 3, 4, 5));
    ar_write(&cpu, 3, 99);
    ar_write(&cpu, 4, 42);
    ar_write(&cpu, 5, 5); /* condition: t>0 → no move */
    xtensa_step(&cpu);
    ASSERT_EQ(ar_read(&cpu, 3), 99);
    teardown(&cpu);
}

/* ===== MOVGEZ ===== */

TEST(exec_movgez_true_positive) {
    xtensa_cpu_t cpu; setup(&cpu);
    put_insn3(&cpu, BASE, rrr(11, 3, 3, 4, 5));
    ar_write(&cpu, 3, 0);
    ar_write(&cpu, 4, 42);
    ar_write(&cpu, 5, 5); /* condition: t>0 → move */
    xtensa_step(&cpu);
    ASSERT_EQ(ar_read(&cpu, 3), 42);
    teardown(&cpu);
}

TEST(exec_movgez_true_zero) {
    xtensa_cpu_t cpu; setup(&cpu);
    put_insn3(&cpu, BASE, rrr(11, 3, 3, 4, 5));
    ar_write(&cpu, 3, 0);
    ar_write(&cpu, 4, 42);
    ar_write(&cpu, 5, 0); /* condition: t==0 → move (>=0) */
    xtensa_step(&cpu);
    ASSERT_EQ(ar_read(&cpu, 3), 42);
    teardown(&cpu);
}

TEST(exec_movgez_false) {
    xtensa_cpu_t cpu; setup(&cpu);
    put_insn3(&cpu, BASE, rrr(11, 3, 3, 4, 5));
    ar_write(&cpu, 3, 99);
    ar_write(&cpu, 4, 42);
    ar_write(&cpu, 5, (uint32_t)-1); /* condition: t<0 → no move */
    xtensa_step(&cpu);
    ASSERT_EQ(ar_read(&cpu, 3), 99);
    teardown(&cpu);
}

/* ===== MOV.N ===== */

TEST(exec_mov_n) {
    xtensa_cpu_t cpu; setup(&cpu);
    /* MOV.N a3, a4: op0=0xD, r=0, s=4, t=3 */
    put_insn2(&cpu, BASE, narrow(0xD, 0, 4, 3));
    ar_write(&cpu, 4, 0xDEADBEEF);
    xtensa_step(&cpu);
    ASSERT_EQ(ar_read(&cpu, 3), 0xDEADBEEF);
    ASSERT_EQ(cpu.pc, BASE + 2);
    teardown(&cpu);
}

/* ===== RET.N ===== */

TEST(exec_ret_n) {
    xtensa_cpu_t cpu; setup(&cpu);
    put_insn2(&cpu, BASE, 0xF00D); /* RET.N */
    ar_write(&cpu, 0, 0x40090000);
    xtensa_step(&cpu);
    ASSERT_EQ(cpu.pc, 0x40090000);
    teardown(&cpu);
}

/* ===== RSR / WSR / XSR ===== */

TEST(exec_rsr_sar) {
    xtensa_cpu_t cpu; setup(&cpu);
    /* RSR a3, SAR: op2=0, op1=3, SR=3 (SAR)
       SR_NUM = s||r = (s<<4)|r. SAR=3 → s=0, r=3
       insn = (0<<20)|(3<<16)|(3<<12)|(0<<8)|(3<<4)|0 = 0x033030 */
    /* Wait: t is the dest register for RSR. RSR at, sr.
       op2=0, op1=3, r=sr_low, s=sr_high, t=dest
       For SAR(sr=3): r=3, s=0 → SR_NUM = (s<<4)|r... no.
       Actually XT_SR_NUM(i) = (i >> 8) & 0xFF = s<<4|r... wait:
       bits 15:8 = (r << 4) | s. Hmm no.
       bits 15:12 = r, bits 11:8 = s.
       XT_SR_NUM = ((insn >> 8) & 0xFF) = (r << 4) | s.
       For SAR = 3: (r << 4) | s = 3. So r=0, s=3 OR r=0, s=3.
       r=0, s=3 → SR_NUM = (0<<4)|3 = 3. ✓ */
    put_insn3(&cpu, BASE, rrr(0, 3, 0, 3, 5));
    cpu.sar = 17;
    xtensa_step(&cpu);
    ASSERT_EQ(ar_read(&cpu, 5), 17);
    teardown(&cpu);
}

TEST(exec_wsr_sar) {
    xtensa_cpu_t cpu; setup(&cpu);
    /* WSR SAR, a5: op2=1, op1=3, r=0, s=3, t=5 */
    put_insn3(&cpu, BASE, rrr(1, 3, 0, 3, 5));
    ar_write(&cpu, 5, 25);
    xtensa_step(&cpu);
    ASSERT_EQ(cpu.sar, 25);
    teardown(&cpu);
}

TEST(exec_xsr_sar) {
    xtensa_cpu_t cpu; setup(&cpu);
    /* XSR a5, SAR: op2=6, op1=1, r=0, s=3, t=5
       Wait: XSR is in RST1 (op1=1), op2=6.
       SR_NUM = (r<<4)|s = (0<<4)|3 = 3. But that's wrong.
       Actually: XT_SR_NUM = ((insn >> 8) & 0xFF)
       insn bits 15:8 = (r<<4)|s = (0<<4)|3 = 0x03. So SR=3=SAR. ✓ */
    put_insn3(&cpu, BASE, rrr(6, 1, 0, 3, 5));
    ar_write(&cpu, 5, 20);
    cpu.sar = 10;
    xtensa_step(&cpu);
    ASSERT_EQ(ar_read(&cpu, 5), 10); /* old SAR value */
    ASSERT_EQ(cpu.sar, 20);          /* new SAR from old a5 */
    teardown(&cpu);
}

/* ===== RSR/WSR for other registers ===== */

TEST(exec_rsr_ps) {
    xtensa_cpu_t cpu; setup(&cpu);
    /* RSR a5, PS: SR=230. 230 = 0xE6. r=6, s=0xE.
       SR_NUM = (r<<4)|s = (6<<4)|0xE = 0x6E = 110... that's not 230.
       Wait: XT_SR_NUM = (insn>>8) & 0xFF = byte at bits 15:8 = r<<4|s.
       230 = 0xE6. So byte = 0xE6 → r = 0xE, s = 0x6.
       op2=0, op1=3, r=0xE, s=6, t=5 */
    put_insn3(&cpu, BASE, rrr(0, 3, 0xE, 6, 5));
    cpu.ps = 0x00040010;
    xtensa_step(&cpu);
    ASSERT_EQ(ar_read(&cpu, 5), 0x00040010);
    teardown(&cpu);
}

TEST(exec_wsr_ps) {
    xtensa_cpu_t cpu; setup(&cpu);
    /* WSR PS, a5: op2=1, op1=3, r=0xE, s=6, t=5 */
    put_insn3(&cpu, BASE, rrr(1, 3, 0xE, 6, 5));
    ar_write(&cpu, 5, 0x00060020);
    xtensa_step(&cpu);
    ASSERT_EQ(cpu.ps, 0x00060020);
    teardown(&cpu);
}

void run_move_tests(void) {
    TEST_SUITE("Move/SR Execution");

    RUN_TEST(exec_moveqz_true);
    RUN_TEST(exec_moveqz_false);
    RUN_TEST(exec_movnez_true);
    RUN_TEST(exec_movnez_false);
    RUN_TEST(exec_movltz_true);
    RUN_TEST(exec_movltz_false_zero);
    RUN_TEST(exec_movltz_false_positive);
    RUN_TEST(exec_movgez_true_positive);
    RUN_TEST(exec_movgez_true_zero);
    RUN_TEST(exec_movgez_false);
    RUN_TEST(exec_mov_n);
    RUN_TEST(exec_ret_n);
    RUN_TEST(exec_rsr_sar);
    RUN_TEST(exec_wsr_sar);
    RUN_TEST(exec_xsr_sar);
    RUN_TEST(exec_rsr_ps);
    RUN_TEST(exec_wsr_ps);
}
