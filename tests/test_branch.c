/*
 * Tests for branch and control flow instruction execution (M4).
 */
#include "test_helpers.h"
#include <string.h>

/* Build op0=7 branch: op0=7, r=subop, s, t, imm8 */
static uint32_t branch_rri8(int r, int s, int t, int imm8) {
    return (uint32_t)(((imm8 & 0xFF) << 16) | (r << 12) | (s << 8) | (t << 4) | 7);
}

/* Build J instruction: op0=6, n=0, offset18 */
static uint32_t j_insn(int32_t offset) {
    uint32_t off18 = (uint32_t)offset & 0x3FFFF;
    /* J format: offset18[17:0] at bits 23:6, n=0 at bits 5:4, op0=6 at bits 3:0 */
    return (off18 << 6) | (0 << 4) | 6;
}

/* Build CALL0 instruction: op0=5, n=0, offset18 */
static uint32_t call0_insn(int32_t offset) {
    uint32_t off18 = (uint32_t)offset & 0x3FFFF;
    return (off18 << 6) | (0 << 4) | 5;
}

/* Build BRI12 instruction: op0=6, n=1, m, s, imm12 */
static uint32_t bri12(int m, int s, int32_t imm12) {
    uint32_t uimm12 = (uint32_t)imm12 & 0xFFF;
    /* BRI12 format: imm12[11:0] at bits 23:12, s at bits 11:8, m at bits 7:6, n=1 at bits 5:4, op0=6 */
    return (uimm12 << 12) | (s << 8) | (m << 6) | (1 << 4) | 6;
}

/* Build BRI8 (BI0) instruction: op0=6, n=2, m, s, r, imm8 */
static uint32_t bri8_bi0(int m, int s, int r, int imm8) {
    /* BRI8 format: imm8[7:0] at bits 23:16, r at bits 15:12, s at bits 11:8, m at bits 7:6, n=2 at bits 5:4, op0=6 */
    return (uint32_t)(((imm8 & 0xFF) << 16) | (r << 12) | (s << 8) | (m << 6) | (2 << 4) | 6);
}

/* Build BRI8 (BI1) instruction: op0=6, n=3, m, s, r, imm8 */
static uint32_t bri8_bi1(int m, int s, int r, int imm8) {
    return (uint32_t)(((imm8 & 0xFF) << 16) | (r << 12) | (s << 8) | (m << 6) | (3 << 4) | 6);
}

/* ===== J (unconditional jump) ===== */

TEST(br_j_forward) {
    xtensa_cpu_t cpu; setup(&cpu);
    /* J +8: offset such that target = BASE + 3 + offset + 1 = BASE + 12
     * offset = 8. But J offset is original_pc-relative: target = original_pc + offset + 4
     * original_pc = BASE, target = BASE + offset + 4 = BASE + 12
     * offset = 8
     */
    put_insn3(&cpu, BASE, j_insn(8));
    xtensa_step(&cpu);
    ASSERT_EQ(cpu.pc, BASE + 12);
    teardown(&cpu);
}

TEST(br_j_backward) {
    xtensa_cpu_t cpu; setup(&cpu);
    /* J -4: target = BASE + (-4) + 4 = BASE */
    put_insn3(&cpu, BASE, j_insn(-4));
    xtensa_step(&cpu);
    ASSERT_EQ(cpu.pc, BASE);
    teardown(&cpu);
}

/* ===== CALL0 ===== */

TEST(br_call0_basic) {
    xtensa_cpu_t cpu; setup(&cpu);
    /* CALL0 to a target.
     * target = (((original_pc >> 2) + offset + 1) << 2)
     * original_pc = BASE = 0x40080000
     * original_pc >> 2 = 0x10020000
     * We want target = BASE + 16 = 0x40080010
     * target >> 2 = 0x10020004
     * offset + 1 = 0x10020004 - 0x10020000 = 4
     * offset = 3
     */
    put_insn3(&cpu, BASE, call0_insn(3));
    xtensa_step(&cpu);
    ASSERT_EQ(cpu.pc, BASE + 16);
    /* Return address in a0 = next instruction = BASE + 3 */
    ASSERT_EQ(ar_read(&cpu, 0), BASE + 3);
    teardown(&cpu);
}

TEST(br_call0_ret_roundtrip) {
    xtensa_cpu_t cpu; setup(&cpu);
    /* CALL0 at BASE, target = BASE+16 */
    put_insn3(&cpu, BASE, call0_insn(3));
    /* NOP at BASE+3 (for return target) */
    put_insn3(&cpu, BASE + 3, 0x002F00);
    /* RET at target (BASE+16): op0=0, op1=0, op2=0, r=0, m=2, n=0 */
    /* RET encoding: 0x000080 actually... let me build it properly */
    /* RET: SNM0 m=2, n=0 -> (m<<6)|(n<<4) = 0x80, r=0, s=0, t=0, op2=0, op1=0 */
    put_insn3(&cpu, BASE + 16, rrr(0, 0, 0, 0, 0) | 0x80);
    xtensa_step(&cpu); /* CALL0 */
    ASSERT_EQ(cpu.pc, BASE + 16);
    xtensa_step(&cpu); /* RET */
    ASSERT_EQ(cpu.pc, BASE + 3);
    teardown(&cpu);
}

/* ===== CALLX0 (bug fix verification) ===== */

TEST(br_callx0_ret_addr) {
    xtensa_cpu_t cpu; setup(&cpu);
    /* CALLX0 a5: SNM0, m=3, n=0, s=5 */
    uint32_t callx0 = rrr(0, 0, 0, 5, 0) | (3 << 6) | (0 << 4);
    put_insn3(&cpu, BASE, callx0);
    ar_write(&cpu, 5, BASE + 100);
    xtensa_step(&cpu);
    /* Return address should be BASE+3 (next instruction), not BASE+6 */
    ASSERT_EQ(ar_read(&cpu, 0), BASE + 3);
    ASSERT_EQ(cpu.pc, BASE + 100);
    teardown(&cpu);
}

/* ===== BEQ / BNE ===== */

TEST(br_beq_taken) {
    xtensa_cpu_t cpu; setup(&cpu);
    /* BEQ a4, a5, +8: r=1, s=4, t=5, offset=8 -> target = BASE + 8 + 4 = BASE + 12 */
    put_insn3(&cpu, BASE, branch_rri8(1, 4, 5, 8));
    ar_write(&cpu, 4, 42);
    ar_write(&cpu, 5, 42);
    xtensa_step(&cpu);
    ASSERT_EQ(cpu.pc, BASE + 12);
    teardown(&cpu);
}

TEST(br_beq_not_taken) {
    xtensa_cpu_t cpu; setup(&cpu);
    put_insn3(&cpu, BASE, branch_rri8(1, 4, 5, 8));
    ar_write(&cpu, 4, 42);
    ar_write(&cpu, 5, 43);
    xtensa_step(&cpu);
    ASSERT_EQ(cpu.pc, BASE + 3); /* falls through */
    teardown(&cpu);
}

TEST(br_bne_taken) {
    xtensa_cpu_t cpu; setup(&cpu);
    put_insn3(&cpu, BASE, branch_rri8(9, 4, 5, 8));
    ar_write(&cpu, 4, 1);
    ar_write(&cpu, 5, 2);
    xtensa_step(&cpu);
    ASSERT_EQ(cpu.pc, BASE + 12);
    teardown(&cpu);
}

TEST(br_bne_not_taken) {
    xtensa_cpu_t cpu; setup(&cpu);
    put_insn3(&cpu, BASE, branch_rri8(9, 4, 5, 8));
    ar_write(&cpu, 4, 7);
    ar_write(&cpu, 5, 7);
    xtensa_step(&cpu);
    ASSERT_EQ(cpu.pc, BASE + 3);
    teardown(&cpu);
}

/* ===== BLT / BGE (signed) ===== */

TEST(br_blt_taken) {
    xtensa_cpu_t cpu; setup(&cpu);
    put_insn3(&cpu, BASE, branch_rri8(2, 4, 5, 8));
    ar_write(&cpu, 4, (uint32_t)-10); /* -10 < 5 */
    ar_write(&cpu, 5, 5);
    xtensa_step(&cpu);
    ASSERT_EQ(cpu.pc, BASE + 12);
    teardown(&cpu);
}

TEST(br_blt_not_taken_equal) {
    xtensa_cpu_t cpu; setup(&cpu);
    put_insn3(&cpu, BASE, branch_rri8(2, 4, 5, 8));
    ar_write(&cpu, 4, 5);
    ar_write(&cpu, 5, 5);
    xtensa_step(&cpu);
    ASSERT_EQ(cpu.pc, BASE + 3);
    teardown(&cpu);
}

TEST(br_bge_taken) {
    xtensa_cpu_t cpu; setup(&cpu);
    put_insn3(&cpu, BASE, branch_rri8(10, 4, 5, 8));
    ar_write(&cpu, 4, 10);
    ar_write(&cpu, 5, 5);
    xtensa_step(&cpu);
    ASSERT_EQ(cpu.pc, BASE + 12);
    teardown(&cpu);
}

TEST(br_bge_taken_equal) {
    xtensa_cpu_t cpu; setup(&cpu);
    put_insn3(&cpu, BASE, branch_rri8(10, 4, 5, 8));
    ar_write(&cpu, 4, 5);
    ar_write(&cpu, 5, 5);
    xtensa_step(&cpu);
    ASSERT_EQ(cpu.pc, BASE + 12);
    teardown(&cpu);
}

/* ===== BLTU / BGEU (unsigned) ===== */

TEST(br_bltu_taken) {
    xtensa_cpu_t cpu; setup(&cpu);
    put_insn3(&cpu, BASE, branch_rri8(3, 4, 5, 8));
    ar_write(&cpu, 4, 5);
    ar_write(&cpu, 5, 0xFFFFFFFF);
    xtensa_step(&cpu);
    ASSERT_EQ(cpu.pc, BASE + 12);
    teardown(&cpu);
}

TEST(br_bgeu_taken) {
    xtensa_cpu_t cpu; setup(&cpu);
    put_insn3(&cpu, BASE, branch_rri8(11, 4, 5, 8));
    ar_write(&cpu, 4, 0xFFFFFFFF);
    ar_write(&cpu, 5, 5);
    xtensa_step(&cpu);
    ASSERT_EQ(cpu.pc, BASE + 12);
    teardown(&cpu);
}

/* ===== BEQZ / BNEZ / BLTZ / BGEZ (BRI12) ===== */

TEST(br_beqz_taken) {
    xtensa_cpu_t cpu; setup(&cpu);
    put_insn3(&cpu, BASE, bri12(0, 4, 8));
    ar_write(&cpu, 4, 0);
    xtensa_step(&cpu);
    ASSERT_EQ(cpu.pc, BASE + 12); /* BASE + 8 + 4 */
    teardown(&cpu);
}

TEST(br_beqz_not_taken) {
    xtensa_cpu_t cpu; setup(&cpu);
    put_insn3(&cpu, BASE, bri12(0, 4, 8));
    ar_write(&cpu, 4, 1);
    xtensa_step(&cpu);
    ASSERT_EQ(cpu.pc, BASE + 3);
    teardown(&cpu);
}

TEST(br_bnez_taken) {
    xtensa_cpu_t cpu; setup(&cpu);
    put_insn3(&cpu, BASE, bri12(1, 4, 8));
    ar_write(&cpu, 4, 42);
    xtensa_step(&cpu);
    ASSERT_EQ(cpu.pc, BASE + 12);
    teardown(&cpu);
}

TEST(br_bnez_not_taken) {
    xtensa_cpu_t cpu; setup(&cpu);
    put_insn3(&cpu, BASE, bri12(1, 4, 8));
    ar_write(&cpu, 4, 0);
    xtensa_step(&cpu);
    ASSERT_EQ(cpu.pc, BASE + 3);
    teardown(&cpu);
}

TEST(br_bltz_taken) {
    xtensa_cpu_t cpu; setup(&cpu);
    put_insn3(&cpu, BASE, bri12(2, 4, 8));
    ar_write(&cpu, 4, (uint32_t)-1);
    xtensa_step(&cpu);
    ASSERT_EQ(cpu.pc, BASE + 12);
    teardown(&cpu);
}

TEST(br_bltz_not_taken) {
    xtensa_cpu_t cpu; setup(&cpu);
    put_insn3(&cpu, BASE, bri12(2, 4, 8));
    ar_write(&cpu, 4, 0);
    xtensa_step(&cpu);
    ASSERT_EQ(cpu.pc, BASE + 3);
    teardown(&cpu);
}

TEST(br_bgez_taken) {
    xtensa_cpu_t cpu; setup(&cpu);
    put_insn3(&cpu, BASE, bri12(3, 4, 8));
    ar_write(&cpu, 4, 0);
    xtensa_step(&cpu);
    ASSERT_EQ(cpu.pc, BASE + 12);
    teardown(&cpu);
}

TEST(br_bgez_not_taken) {
    xtensa_cpu_t cpu; setup(&cpu);
    put_insn3(&cpu, BASE, bri12(3, 4, 8));
    ar_write(&cpu, 4, (uint32_t)-1);
    xtensa_step(&cpu);
    ASSERT_EQ(cpu.pc, BASE + 3);
    teardown(&cpu);
}

/* ===== BEQI / BNEI (BRI8 with b4const) ===== */

TEST(br_beqi_taken) {
    xtensa_cpu_t cpu; setup(&cpu);
    /* BEQI a4, b4const[r], offset: m=0, n=2, r selects constant.
     * b4const[3] = 3. So BEQI a4, 3, +8 */
    put_insn3(&cpu, BASE, bri8_bi0(0, 4, 3, 8));
    ar_write(&cpu, 4, 3);
    xtensa_step(&cpu);
    ASSERT_EQ(cpu.pc, BASE + 12);
    teardown(&cpu);
}

TEST(br_beqi_not_taken) {
    xtensa_cpu_t cpu; setup(&cpu);
    put_insn3(&cpu, BASE, bri8_bi0(0, 4, 3, 8));
    ar_write(&cpu, 4, 4);
    xtensa_step(&cpu);
    ASSERT_EQ(cpu.pc, BASE + 3);
    teardown(&cpu);
}

TEST(br_bnei_taken) {
    xtensa_cpu_t cpu; setup(&cpu);
    /* b4const[0] = -1 */
    put_insn3(&cpu, BASE, bri8_bi0(1, 4, 0, 8));
    ar_write(&cpu, 4, 5);
    xtensa_step(&cpu);
    ASSERT_EQ(cpu.pc, BASE + 12);
    teardown(&cpu);
}

TEST(br_blti_taken) {
    xtensa_cpu_t cpu; setup(&cpu);
    /* BLTI: m=2. b4const[5]=5. ar[s]=3 < 5 -> taken */
    put_insn3(&cpu, BASE, bri8_bi0(2, 4, 5, 8));
    ar_write(&cpu, 4, 3);
    xtensa_step(&cpu);
    ASSERT_EQ(cpu.pc, BASE + 12);
    teardown(&cpu);
}

TEST(br_bgei_taken) {
    xtensa_cpu_t cpu; setup(&cpu);
    /* BGEI: m=3. b4const[5]=5. ar[s]=5 >= 5 -> taken */
    put_insn3(&cpu, BASE, bri8_bi0(3, 4, 5, 8));
    ar_write(&cpu, 4, 5);
    xtensa_step(&cpu);
    ASSERT_EQ(cpu.pc, BASE + 12);
    teardown(&cpu);
}

/* ===== BLTUI / BGEUI (BRI8 with b4constu) ===== */

TEST(br_bltui_taken) {
    xtensa_cpu_t cpu; setup(&cpu);
    /* BLTUI: m=2, n=3. b4constu[4]=4. ar[s]=3 < 4 -> taken */
    put_insn3(&cpu, BASE, bri8_bi1(2, 4, 4, 8));
    ar_write(&cpu, 4, 3);
    xtensa_step(&cpu);
    ASSERT_EQ(cpu.pc, BASE + 12);
    teardown(&cpu);
}

TEST(br_bgeui_taken) {
    xtensa_cpu_t cpu; setup(&cpu);
    /* BGEUI: m=3, n=3. b4constu[4]=4. ar[s]=4 >= 4 -> taken */
    put_insn3(&cpu, BASE, bri8_bi1(3, 4, 4, 8));
    ar_write(&cpu, 4, 4);
    xtensa_step(&cpu);
    ASSERT_EQ(cpu.pc, BASE + 12);
    teardown(&cpu);
}

/* ===== BNONE / BALL / BANY / BNALL ===== */

TEST(br_bnone_taken) {
    xtensa_cpu_t cpu; setup(&cpu);
    put_insn3(&cpu, BASE, branch_rri8(0, 4, 5, 8));
    ar_write(&cpu, 4, 0xF0);
    ar_write(&cpu, 5, 0x0F);
    xtensa_step(&cpu);
    ASSERT_EQ(cpu.pc, BASE + 12); /* (0xF0 & 0x0F) == 0 */
    teardown(&cpu);
}

TEST(br_ball_taken) {
    xtensa_cpu_t cpu; setup(&cpu);
    put_insn3(&cpu, BASE, branch_rri8(4, 4, 5, 8));
    ar_write(&cpu, 4, 0xFF);
    ar_write(&cpu, 5, 0x0F);
    xtensa_step(&cpu);
    ASSERT_EQ(cpu.pc, BASE + 12); /* all bits of 0x0F are set in 0xFF */
    teardown(&cpu);
}

TEST(br_bany_taken) {
    xtensa_cpu_t cpu; setup(&cpu);
    put_insn3(&cpu, BASE, branch_rri8(8, 4, 5, 8));
    ar_write(&cpu, 4, 0x10);
    ar_write(&cpu, 5, 0x10);
    xtensa_step(&cpu);
    ASSERT_EQ(cpu.pc, BASE + 12); /* (0x10 & 0x10) != 0 */
    teardown(&cpu);
}

TEST(br_bnall_taken) {
    xtensa_cpu_t cpu; setup(&cpu);
    put_insn3(&cpu, BASE, branch_rri8(12, 4, 5, 8));
    ar_write(&cpu, 4, 0x0E); /* missing bit 0 */
    ar_write(&cpu, 5, 0x0F);
    xtensa_step(&cpu);
    ASSERT_EQ(cpu.pc, BASE + 12); /* not all bits of 0x0F are set in 0x0E */
    teardown(&cpu);
}

/* ===== BBC / BBS / BBCI / BBSI ===== */

TEST(br_bbc_taken) {
    xtensa_cpu_t cpu; setup(&cpu);
    put_insn3(&cpu, BASE, branch_rri8(5, 4, 5, 8));
    ar_write(&cpu, 4, 0xFFFFFFFE); /* bit 0 clear */
    ar_write(&cpu, 5, 0); /* test bit 0 */
    xtensa_step(&cpu);
    ASSERT_EQ(cpu.pc, BASE + 12);
    teardown(&cpu);
}

TEST(br_bbs_taken) {
    xtensa_cpu_t cpu; setup(&cpu);
    put_insn3(&cpu, BASE, branch_rri8(13, 4, 5, 8));
    ar_write(&cpu, 4, 0x80); /* bit 7 set */
    ar_write(&cpu, 5, 7); /* test bit 7 */
    xtensa_step(&cpu);
    ASSERT_EQ(cpu.pc, BASE + 12);
    teardown(&cpu);
}

TEST(br_bbci_taken) {
    xtensa_cpu_t cpu; setup(&cpu);
    /* BBCI: r=6, bit = t | ((r&1)<<4) = t (since r=6, r&1=0) */
    /* Test bit 3 of a4. t=3, r=6. */
    put_insn3(&cpu, BASE, branch_rri8(6, 4, 3, 8));
    ar_write(&cpu, 4, 0xFFFFFFF7); /* bit 3 clear */
    xtensa_step(&cpu);
    ASSERT_EQ(cpu.pc, BASE + 12);
    teardown(&cpu);
}

TEST(br_bbci_high_r7_taken) {
    xtensa_cpu_t cpu; setup(&cpu);
    /* BBCI/BBSI encoding: r[3] = BBCI(0)/BBSI(1), r[0] = low(0)/high(1)
     * r=6:  BBCI low  (bits 0-15)
     * r=7:  BBCI high (bits 16-31)
     * r=14: BBSI low  (bits 0-15)
     * r=15: BBSI high (bits 16-31)
     */
    /* BBCI with r=7: bit = t | 16 = 19. r=7, t=3, s=4 */
    put_insn3(&cpu, BASE, branch_rri8(7, 4, 3, 8));
    ar_write(&cpu, 4, 0xFFF7FFFF); /* bit 19 clear */
    xtensa_step(&cpu);
    ASSERT_EQ(cpu.pc, BASE + 12); /* taken: bit 19 clear */
    teardown(&cpu);
}

TEST(br_bbci_high_r7_not_taken) {
    xtensa_cpu_t cpu; setup(&cpu);
    /* BBCI with r=7: bit = t | 16 = 19 */
    put_insn3(&cpu, BASE, branch_rri8(7, 4, 3, 8));
    ar_write(&cpu, 4, 0x00080000); /* bit 19 set */
    xtensa_step(&cpu);
    ASSERT_EQ(cpu.pc, BASE + 3); /* not taken: bit 19 set */
    teardown(&cpu);
}

TEST(br_bbsi_low_r14_taken) {
    xtensa_cpu_t cpu; setup(&cpu);
    /* BBSI with r=14: bit = t | 0 = 4. r=14, t=4, s=4 */
    put_insn3(&cpu, BASE, branch_rri8(14, 4, 4, 8));
    ar_write(&cpu, 4, 0x00000010); /* bit 4 set */
    xtensa_step(&cpu);
    ASSERT_EQ(cpu.pc, BASE + 12); /* taken: bit 4 set */
    teardown(&cpu);
}

TEST(br_bbsi_low_r14_not_taken) {
    xtensa_cpu_t cpu; setup(&cpu);
    /* BBSI with r=14: bit = t = 4 */
    put_insn3(&cpu, BASE, branch_rri8(14, 4, 4, 8));
    ar_write(&cpu, 4, 0xFFFFFFEF); /* bit 4 clear */
    xtensa_step(&cpu);
    ASSERT_EQ(cpu.pc, BASE + 3); /* not taken: bit 4 clear */
    teardown(&cpu);
}

TEST(br_bbsi_high_taken) {
    xtensa_cpu_t cpu; setup(&cpu);
    /* BBSI with r=15: bit = t | 16. Test bit 20: r=15, t=4 -> bit = 4 | 16 = 20 */
    put_insn3(&cpu, BASE, branch_rri8(15, 4, 4, 8));
    ar_write(&cpu, 4, 0x00100000); /* bit 20 set */
    xtensa_step(&cpu);
    ASSERT_EQ(cpu.pc, BASE + 12);
    teardown(&cpu);
}

/* ===== BF / BT (boolean branches) ===== */

TEST(br_bf_taken) {
    xtensa_cpu_t cpu; setup(&cpu);
    /* BF b3, +8: n=3, m=1, r=0, s=3 */
    put_insn3(&cpu, BASE, bri8_bi1(1, 3, 0, 8));
    cpu.br = 0; /* b3 is 0 -> taken */
    xtensa_step(&cpu);
    ASSERT_EQ(cpu.pc, BASE + 12);
    teardown(&cpu);
}

TEST(br_bf_not_taken) {
    xtensa_cpu_t cpu; setup(&cpu);
    put_insn3(&cpu, BASE, bri8_bi1(1, 3, 0, 8));
    cpu.br = (1 << 3); /* b3 is 1 -> not taken */
    xtensa_step(&cpu);
    ASSERT_EQ(cpu.pc, BASE + 3);
    teardown(&cpu);
}

TEST(br_bt_taken) {
    xtensa_cpu_t cpu; setup(&cpu);
    /* BT b5, +8: n=3, m=1, r=1, s=5 */
    put_insn3(&cpu, BASE, bri8_bi1(1, 5, 1, 8));
    cpu.br = (1 << 5); /* b5 is 1 -> taken */
    xtensa_step(&cpu);
    ASSERT_EQ(cpu.pc, BASE + 12);
    teardown(&cpu);
}

/* ===== BEQZ.N / BNEZ.N ===== */

TEST(br_beqz_n_taken) {
    xtensa_cpu_t cpu; setup(&cpu);
    /* BEQZ.N a4, +4: op0=0xC, t_hi=2 (t[3:2]=10), t[1:0]=imm6[5:4]=0, r=imm6[3:0]=4
     * t = (1 << 3) | (0 << 2) | 0 = 8. r = 4.
     * imm6 = ((t&3) << 4) | r = (0 << 4) | 4 = 4
     * target = cpu->pc + 4 + 2 = BASE + 2 + 4 + 2 = BASE + 8
     * Wait, actually for narrow: cpu->pc = BASE + 2 already at exec time.
     * target = (BASE+2) + 4 + 2 = BASE + 8
     */
    put_insn2(&cpu, BASE, narrow(0xC, 4, 4, 8));
    ar_write(&cpu, 4, 0);
    xtensa_step(&cpu);
    ASSERT_EQ(cpu.pc, BASE + 8);
    teardown(&cpu);
}

TEST(br_beqz_n_not_taken) {
    xtensa_cpu_t cpu; setup(&cpu);
    put_insn2(&cpu, BASE, narrow(0xC, 4, 4, 8));
    ar_write(&cpu, 4, 1);
    xtensa_step(&cpu);
    ASSERT_EQ(cpu.pc, BASE + 2);
    teardown(&cpu);
}

TEST(br_bnez_n_taken) {
    xtensa_cpu_t cpu; setup(&cpu);
    /* BNEZ.N a4, +4: t_hi=3 (t[3:2]=11), t[1:0]=0, r=4
     * t = (1 << 3) | (1 << 2) | 0 = 12. */
    put_insn2(&cpu, BASE, narrow(0xC, 4, 4, 12));
    ar_write(&cpu, 4, 1);
    xtensa_step(&cpu);
    ASSERT_EQ(cpu.pc, BASE + 8);
    teardown(&cpu);
}

TEST(br_bnez_n_not_taken) {
    xtensa_cpu_t cpu; setup(&cpu);
    put_insn2(&cpu, BASE, narrow(0xC, 4, 4, 12));
    ar_write(&cpu, 4, 0);
    xtensa_step(&cpu);
    ASSERT_EQ(cpu.pc, BASE + 2);
    teardown(&cpu);
}

/* ===== Backward branch ===== */

TEST(br_beq_backward) {
    xtensa_cpu_t cpu; setup(&cpu);
    /* BEQ a4, a5, -4: target = BASE + (-4) + 4 = BASE */
    put_insn3(&cpu, BASE, branch_rri8(1, 4, 5, (uint8_t)-4));
    ar_write(&cpu, 4, 7);
    ar_write(&cpu, 5, 7);
    xtensa_step(&cpu);
    ASSERT_EQ(cpu.pc, BASE);
    teardown(&cpu);
}

void run_branch_tests(void) {
    TEST_SUITE("Branch/Control Flow Execution");

    RUN_TEST(br_j_forward);
    RUN_TEST(br_j_backward);
    RUN_TEST(br_call0_basic);
    RUN_TEST(br_call0_ret_roundtrip);
    RUN_TEST(br_callx0_ret_addr);
    RUN_TEST(br_beq_taken);
    RUN_TEST(br_beq_not_taken);
    RUN_TEST(br_bne_taken);
    RUN_TEST(br_bne_not_taken);
    RUN_TEST(br_blt_taken);
    RUN_TEST(br_blt_not_taken_equal);
    RUN_TEST(br_bge_taken);
    RUN_TEST(br_bge_taken_equal);
    RUN_TEST(br_bltu_taken);
    RUN_TEST(br_bgeu_taken);
    RUN_TEST(br_beqz_taken);
    RUN_TEST(br_beqz_not_taken);
    RUN_TEST(br_bnez_taken);
    RUN_TEST(br_bnez_not_taken);
    RUN_TEST(br_bltz_taken);
    RUN_TEST(br_bltz_not_taken);
    RUN_TEST(br_bgez_taken);
    RUN_TEST(br_bgez_not_taken);
    RUN_TEST(br_beqi_taken);
    RUN_TEST(br_beqi_not_taken);
    RUN_TEST(br_bnei_taken);
    RUN_TEST(br_blti_taken);
    RUN_TEST(br_bgei_taken);
    RUN_TEST(br_bltui_taken);
    RUN_TEST(br_bgeui_taken);
    RUN_TEST(br_bnone_taken);
    RUN_TEST(br_ball_taken);
    RUN_TEST(br_bany_taken);
    RUN_TEST(br_bnall_taken);
    RUN_TEST(br_bbc_taken);
    RUN_TEST(br_bbs_taken);
    RUN_TEST(br_bbci_taken);
    RUN_TEST(br_bbci_high_r7_taken);
    RUN_TEST(br_bbci_high_r7_not_taken);
    RUN_TEST(br_bbsi_low_r14_taken);
    RUN_TEST(br_bbsi_low_r14_not_taken);
    RUN_TEST(br_bbsi_high_taken);
    RUN_TEST(br_bf_taken);
    RUN_TEST(br_bf_not_taken);
    RUN_TEST(br_bt_taken);
    RUN_TEST(br_beqz_n_taken);
    RUN_TEST(br_beqz_n_not_taken);
    RUN_TEST(br_bnez_n_taken);
    RUN_TEST(br_bnez_n_not_taken);
    RUN_TEST(br_beq_backward);
}
