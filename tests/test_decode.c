/*
 * Tests for instruction decode and disassembly.
 * Each test places known instruction bytes into memory and verifies
 * that xtensa_disasm() produces the expected output.
 */
#include "test_helpers.h"
#include <string.h>

/* Helper: disasm and check result contains expected substring */
static int check_disasm(xtensa_cpu_t *cpu, uint32_t addr, const char *expected, int expect_len) {
    char buf[128];
    int len = xtensa_disasm(cpu, addr, buf, sizeof(buf));
    int pass = 1;

    if (len != expect_len) {
        fprintf(stderr, "    length: got %d, expected %d\n", len, expect_len);
        pass = 0;
    }
    if (strstr(buf, expected) == NULL) {
        fprintf(stderr, "    output: \"%s\", expected to contain \"%s\"\n", buf, expected);
        pass = 0;
    }
    return pass;
}

/* BASE defined in test_helpers.h */

/* ===== RRR Format Tests ===== */

TEST(test_nop) {
    xtensa_cpu_t cpu;
    xtensa_cpu_init(&cpu);
    cpu.mem = mem_create();
    /* NOP: op2=0, op1=0, r=2, s=0, t=15(=SYNC0,s=15->nop? No.)
       NOP encoding: op2=0000, op1=0000, r=0010, s=0000, t=1111, op0=0000
       Wait. From opcode tables:
       RST0 -> op2=0 -> ST0 -> r=2 -> SYNC -> t=0 -> SYNC0 -> s=15 -> NOP
       So: op2=0, op1=0, r=2, s=15, t=0, op0=0
       insn = (0<<20)|(0<<16)|(2<<12)|(15<<8)|(0<<4)|0 = 0x002F00 */
    put_insn3(&cpu, BASE, 0x002F00);
    ASSERT_TRUE(check_disasm(&cpu, BASE, "nop", 3));
    mem_destroy(cpu.mem);
}

TEST(test_add) {
    xtensa_cpu_t cpu;
    xtensa_cpu_init(&cpu);
    cpu.mem = mem_create();
    /* ADD a3, a4, a5: op2=0, op1=0, op2_full=8, r=3, s=4, t=5, op0=0
       RST0, op2=8 -> ADD
       insn = (8<<20)|(0<<16)|(3<<12)|(4<<8)|(5<<4)|0 = 0x803450 */
    put_insn3(&cpu, BASE, 0x803450);
    ASSERT_TRUE(check_disasm(&cpu, BASE, "add\ta3, a4, a5", 3));
    mem_destroy(cpu.mem);
}

TEST(test_sub) {
    xtensa_cpu_t cpu;
    xtensa_cpu_init(&cpu);
    cpu.mem = mem_create();
    /* SUB a6, a7, a8: op2=12, op1=0, r=6, s=7, t=8, op0=0
       insn = (12<<20)|(0<<16)|(6<<12)|(7<<8)|(8<<4)|0 = 0xC06780 */
    put_insn3(&cpu, BASE, 0xC06780);
    ASSERT_TRUE(check_disasm(&cpu, BASE, "sub\ta6, a7, a8", 3));
    mem_destroy(cpu.mem);
}

TEST(test_and_or_xor) {
    xtensa_cpu_t cpu;
    xtensa_cpu_init(&cpu);
    cpu.mem = mem_create();
    /* AND a1, a2, a3: op2=1, op1=0, r=1, s=2, t=3, op0=0
       insn = (1<<20)|(0<<16)|(1<<12)|(2<<8)|(3<<4)|0 = 0x101230 */
    put_insn3(&cpu, BASE, 0x101230);
    ASSERT_TRUE(check_disasm(&cpu, BASE, "and\ta1, a2, a3", 3));

    /* OR a4, a5, a6: op2=2 */
    put_insn3(&cpu, BASE, 0x204560);
    ASSERT_TRUE(check_disasm(&cpu, BASE, "or\ta4, a5, a6", 3));

    /* XOR a7, a8, a9: op2=3 */
    put_insn3(&cpu, BASE, 0x307890);
    ASSERT_TRUE(check_disasm(&cpu, BASE, "xor\ta7, a8, a9", 3));
    mem_destroy(cpu.mem);
}

TEST(test_neg_abs) {
    xtensa_cpu_t cpu;
    xtensa_cpu_init(&cpu);
    cpu.mem = mem_create();
    /* NEG a3, a5: op2=6, op1=0, r=3, s=0, t=5, op0=0
       insn = (6<<20)|(0<<16)|(3<<12)|(0<<8)|(5<<4)|0 = 0x603050 */
    put_insn3(&cpu, BASE, 0x603050);
    ASSERT_TRUE(check_disasm(&cpu, BASE, "neg\ta3, a5", 3));

    /* ABS a2, a4: op2=6, op1=0, r=2, s=1, t=4, op0=0
       insn = (6<<20)|(0<<16)|(2<<12)|(1<<8)|(4<<4)|0 = 0x602140 */
    put_insn3(&cpu, BASE, 0x602140);
    ASSERT_TRUE(check_disasm(&cpu, BASE, "abs\ta2, a4", 3));
    mem_destroy(cpu.mem);
}

/* ===== Shift Tests ===== */
/* SLLI encoding will be verified against real objdump output */

/* ===== RRI8 Format Tests ===== */

TEST(test_addi) {
    xtensa_cpu_t cpu;
    xtensa_cpu_init(&cpu);
    cpu.mem = mem_create();
    /* ADDI a3, a4, 10: r=12 (ADDI), s=4, t=3, imm8=10, op0=2
       insn = (10<<16)|(12<<12)|(4<<8)|(3<<4)|2 = 0x0AC432 */
    put_insn3(&cpu, BASE, 0x0AC432);
    ASSERT_TRUE(check_disasm(&cpu, BASE, "addi\ta3, a4, 10", 3));
    mem_destroy(cpu.mem);
}

TEST(test_addi_negative) {
    xtensa_cpu_t cpu;
    xtensa_cpu_init(&cpu);
    cpu.mem = mem_create();
    /* ADDI a5, a6, -1: r=12, s=6, t=5, imm8=0xFF (-1 as signed), op0=2
       insn = (0xFF<<16)|(12<<12)|(6<<8)|(5<<4)|2 = 0xFFC652 */
    put_insn3(&cpu, BASE, 0xFFC652);
    ASSERT_TRUE(check_disasm(&cpu, BASE, "addi\ta5, a6, -1", 3));
    mem_destroy(cpu.mem);
}

TEST(test_movi) {
    xtensa_cpu_t cpu;
    xtensa_cpu_init(&cpu);
    cpu.mem = mem_create();
    /* MOVI a3, 100: r=10 (0xA), s=imm[11:8]=0, t=3, imm8=100(0x64), op0=2
       12-bit imm = (0 << 8) | 100 = 100
       insn = (0x64<<16)|(0xA<<12)|(0<<8)|(3<<4)|2 = 0x64A032 */
    put_insn3(&cpu, BASE, 0x64A032);
    ASSERT_TRUE(check_disasm(&cpu, BASE, "movi\ta3, 100", 3));
    mem_destroy(cpu.mem);
}

TEST(test_movi_negative) {
    xtensa_cpu_t cpu;
    xtensa_cpu_init(&cpu);
    cpu.mem = mem_create();
    /* MOVI a2, -1: 12-bit imm = 0xFFF = -1 signed
       imm[11:8]=0xF=s, imm[7:0]=0xFF=imm8
       r=10, t=2, op0=2
       insn = (0xFF<<16)|(0xA<<12)|(0xF<<8)|(2<<4)|2 = 0xFFAF22 */
    put_insn3(&cpu, BASE, 0xFFAF22);
    ASSERT_TRUE(check_disasm(&cpu, BASE, "movi\ta2, -1", 3));
    mem_destroy(cpu.mem);
}

/* ===== Load/Store Tests ===== */

TEST(test_l32i) {
    xtensa_cpu_t cpu;
    xtensa_cpu_init(&cpu);
    cpu.mem = mem_create();
    /* L32I a3, a1, 16: r=2 (L32I), s=1, t=3, imm8=4 (16/4=4), op0=2
       insn = (4<<16)|(2<<12)|(1<<8)|(3<<4)|2 = 0x042132 */
    put_insn3(&cpu, BASE, 0x042132);
    ASSERT_TRUE(check_disasm(&cpu, BASE, "l32i\ta3, a1, 16", 3));
    mem_destroy(cpu.mem);
}

TEST(test_s32i) {
    xtensa_cpu_t cpu;
    xtensa_cpu_init(&cpu);
    cpu.mem = mem_create();
    /* S32I a5, a1, 8: r=6 (S32I), s=1, t=5, imm8=2 (8/4=2), op0=2
       insn = (2<<16)|(6<<12)|(1<<8)|(5<<4)|2 = 0x026152 */
    put_insn3(&cpu, BASE, 0x026152);
    ASSERT_TRUE(check_disasm(&cpu, BASE, "s32i\ta5, a1, 8", 3));
    mem_destroy(cpu.mem);
}

/* ===== RSR/WSR/XSR Tests ===== */

TEST(test_rsr) {
    xtensa_cpu_t cpu;
    xtensa_cpu_init(&cpu);
    cpu.mem = mem_create();
    /* RSR a3, SAR: op2=0, op1=3, sr=3 (SAR), t=3, op0=0
       sr = s || r = 0x03 -> s=0, r=3
       insn = (0<<20)|(3<<16)|(3<<12)|(0<<8)|(3<<4)|0 = 0x030030

       Wait: RSR encoding from Xtensa.pdf page 18:
       op2=0000, op1=0011, sr[7:0], t, op0=0000
       sr = bits 15:8 = (r << 4) | s ... no, sr is 8 bits at positions 15:8.
       In the RRR format: bits 15:12=r, 11:8=s. So sr = (r << 4) | s.
       Wait no, looking at the RSR format (page 10/657):
       RSR format: op2[4] | op1[4] | sr[8] | t[4] | op0[4]
       sr is bits 15:8, which spans both r and s fields: sr = (r << 4) | s.
       But XT_SR_NUM extracts it as: ((insn >> 8) & 0xFF) = bits 15:8.

       For SAR (sr=3): sr = 0x03, so r=0, s=3.
       insn = (0<<20)|(3<<16)|(0<<12)|(3<<8)|(3<<4)|0 = 0x030330 */
    put_insn3(&cpu, BASE, 0x030330);
    ASSERT_TRUE(check_disasm(&cpu, BASE, "rsr\ta3, sar", 3));
    mem_destroy(cpu.mem);
}

TEST(test_wsr) {
    xtensa_cpu_t cpu;
    xtensa_cpu_init(&cpu);
    cpu.mem = mem_create();
    /* WSR a5, PS: op2=1, op1=3, sr=230 (0xE6), t=5, op0=0
       sr=0xE6: r = 0xE, s = 0x6
       insn = (1<<20)|(3<<16)|(0xE<<12)|(0x6<<8)|(5<<4)|0 = 0x13E650 */
    put_insn3(&cpu, BASE, 0x13E650);
    ASSERT_TRUE(check_disasm(&cpu, BASE, "wsr\ta5, ps", 3));
    mem_destroy(cpu.mem);
}

/* ===== Branch Tests ===== */

TEST(test_beq) {
    xtensa_cpu_t cpu;
    xtensa_cpu_init(&cpu);
    cpu.mem = mem_create();
    /* BEQ a2, a3, target: op0=7, r=1 (BEQ), s=2, t=3, imm8=offset
       target = PC + sext(imm8) + 4
       For offset = 0x10, target = BASE + 0x10 + 4 = BASE + 0x14
       insn = (0x10<<16)|(1<<12)|(2<<8)|(3<<4)|7 = 0x101237 */
    put_insn3(&cpu, BASE, 0x101237);
    ASSERT_TRUE(check_disasm(&cpu, BASE, "beq\ta2, a3", 3));
    mem_destroy(cpu.mem);
}

TEST(test_bnez) {
    xtensa_cpu_t cpu;
    xtensa_cpu_init(&cpu);
    cpu.mem = mem_create();
    /* BNEZ a4, target: BRI12 format, op0=6, n=1, m=1, s=4
       imm12 = offset (say 0x20)
       target = PC + sext(imm12) + 4
       Encoding: imm[11:0] | s | m | n | op0
       insn = (0x20 << 12) | (4 << 8) | (1 << 6) | (1 << 4) | 6
       = 0x020000 | 0x000400 | 0x000040 | 0x000010 | 0x000006
       = 0x020456 */
    put_insn3(&cpu, BASE, 0x020456);
    ASSERT_TRUE(check_disasm(&cpu, BASE, "bnez\ta4", 3));
    mem_destroy(cpu.mem);
}

/* ===== Call/Jump Tests ===== */

TEST(test_ret) {
    xtensa_cpu_t cpu;
    xtensa_cpu_init(&cpu);
    cpu.mem = mem_create();
    /* RET: op2=0, op1=0, r=0, s=0, m=2, n=0, op0=0
       CALLX format: bits = op2|op1|r|s|m|n|op0
       In the actual encoding: m=10, n=00 for JR group, then n=00 for RET
       insn = (0<<20)|(0<<16)|(0<<12)|(0<<8)|(2<<6)|(0<<4)|0
       = 0x000080 */
    put_insn3(&cpu, BASE, 0x000080);
    ASSERT_TRUE(check_disasm(&cpu, BASE, "ret", 3));
    mem_destroy(cpu.mem);
}

TEST(test_call0) {
    xtensa_cpu_t cpu;
    xtensa_cpu_init(&cpu);
    cpu.mem = mem_create();
    /* CALL0: CALL format, op0=5, n=0, offset18
       offset = 0x10 (target = ((PC>>2) + 0x10 + 1) << 2)
       insn = (0x10 << 6) | (0 << 4) | 5 = 0x000405 */
    put_insn3(&cpu, BASE, 0x000405);
    ASSERT_TRUE(check_disasm(&cpu, BASE, "call0", 3));
    mem_destroy(cpu.mem);
}

TEST(test_j) {
    xtensa_cpu_t cpu;
    xtensa_cpu_init(&cpu);
    cpu.mem = mem_create();
    /* J: op0=6, n=0, offset18
       For a forward jump of 0x100:
       target = PC + sext(offset) + 4
       offset = 0x100 - 4 = 0xFC
       insn = (0xFC << 6) | (0 << 4) | 6 = 0x003F06 */
    put_insn3(&cpu, BASE, 0x003F06);
    ASSERT_TRUE(check_disasm(&cpu, BASE, "j\t", 3));
    mem_destroy(cpu.mem);
}

/* ===== Narrow Instruction Tests ===== */

TEST(test_add_n) {
    xtensa_cpu_t cpu;
    xtensa_cpu_init(&cpu);
    cpu.mem = mem_create();
    /* ADD.N a2, a3, a4: op0=0xA, t=4, s=3, r=2
       insn16 = (2<<12)|(3<<8)|(4<<4)|0xA = 0x234A */
    put_insn2(&cpu, BASE, 0x234A);
    ASSERT_TRUE(check_disasm(&cpu, BASE, "add.n\ta2, a3, a4", 2));
    mem_destroy(cpu.mem);
}

TEST(test_addi_n) {
    xtensa_cpu_t cpu;
    xtensa_cpu_init(&cpu);
    cpu.mem = mem_create();
    /* ADDI.N a3, a4, 5: op0=0xB(1011), t=5(imm), s=4, r=3
       insn16 = (3<<12)|(4<<8)|(5<<4)|0xB = 0x345B */
    put_insn2(&cpu, BASE, 0x345B);
    ASSERT_TRUE(check_disasm(&cpu, BASE, "addi.n\ta3, a4, 5", 2));
    mem_destroy(cpu.mem);
}

TEST(test_addi_n_minus1) {
    xtensa_cpu_t cpu;
    xtensa_cpu_init(&cpu);
    cpu.mem = mem_create();
    /* ADDI.N a3, a4, -1: when imm(t)=0, actual value is -1
       insn16 = (3<<12)|(4<<8)|(0<<4)|0xB = 0x340B */
    put_insn2(&cpu, BASE, 0x340B);
    ASSERT_TRUE(check_disasm(&cpu, BASE, "addi.n\ta3, a4, -1", 2));
    mem_destroy(cpu.mem);
}

TEST(test_l32i_n) {
    xtensa_cpu_t cpu;
    xtensa_cpu_init(&cpu);
    cpu.mem = mem_create();
    /* L32I.N a2, a1, 8: op0=8, t=2, s=1, r=2(imm=2, offset=8)
       insn16 = (2<<12)|(1<<8)|(2<<4)|8 = 0x2128 */
    put_insn2(&cpu, BASE, 0x2128);
    ASSERT_TRUE(check_disasm(&cpu, BASE, "l32i.n\ta2, a1, 8", 2));
    mem_destroy(cpu.mem);
}

TEST(test_s32i_n) {
    xtensa_cpu_t cpu;
    xtensa_cpu_init(&cpu);
    cpu.mem = mem_create();
    /* S32I.N a5, a1, 0: op0=9, t=5, s=1, r=0
       insn16 = (0<<12)|(1<<8)|(5<<4)|9 = 0x0159 */
    put_insn2(&cpu, BASE, 0x0159);
    ASSERT_TRUE(check_disasm(&cpu, BASE, "s32i.n\ta5, a1, 0", 2));
    mem_destroy(cpu.mem);
}

TEST(test_mov_n) {
    xtensa_cpu_t cpu;
    xtensa_cpu_init(&cpu);
    cpu.mem = mem_create();
    /* MOV.N a3, a4: op0=0xD, t=3, s=4, r=0
       insn16 = (0<<12)|(4<<8)|(3<<4)|0xD = 0x043D */
    put_insn2(&cpu, BASE, 0x043D);
    ASSERT_TRUE(check_disasm(&cpu, BASE, "mov.n\ta3, a4", 2));
    mem_destroy(cpu.mem);
}

TEST(test_ret_n) {
    xtensa_cpu_t cpu;
    xtensa_cpu_init(&cpu);
    cpu.mem = mem_create();
    /* RET.N: op0=0xD, r=0xF, t=0, s=0
       insn16 = (0xF<<12)|(0<<8)|(0<<4)|0xD = 0xF00D */
    put_insn2(&cpu, BASE, 0xF00D);
    ASSERT_TRUE(check_disasm(&cpu, BASE, "ret.n", 2));
    mem_destroy(cpu.mem);
}

TEST(test_nop_n) {
    xtensa_cpu_t cpu;
    xtensa_cpu_init(&cpu);
    cpu.mem = mem_create();
    /* NOP.N: op0=0xD, r=0xF, t=3, s=0
       insn16 = (0xF<<12)|(0<<8)|(3<<4)|0xD = 0xF03D */
    put_insn2(&cpu, BASE, 0xF03D);
    ASSERT_TRUE(check_disasm(&cpu, BASE, "nop.n", 2));
    mem_destroy(cpu.mem);
}

TEST(test_movi_n) {
    xtensa_cpu_t cpu;
    xtensa_cpu_init(&cpu);
    cpu.mem = mem_create();
    /* MOVI.N a5, 10: RI7 format, op0=0xC
       imm7[3:0] = r = 10 (0xA), imm7[6:4] = t[2:0] = 0, i = t[3] = 0
       imm7 = (0 << 4) | 10 = 10
       s = 5 (dest register)
       t = (i << 3) | imm7[6:4] = 0
       insn16 = (r<<12)|(s<<8)|(t<<4)|0xC = (0xA<<12)|(5<<8)|(0<<4)|0xC = 0xA50C */
    put_insn2(&cpu, BASE, 0xA50C);
    ASSERT_TRUE(check_disasm(&cpu, BASE, "movi.n\ta5, 10", 2));
    mem_destroy(cpu.mem);
}

/* ===== Shift Tests ===== */

TEST(test_slli) {
    xtensa_cpu_t cpu;
    xtensa_cpu_init(&cpu);
    cpu.mem = mem_create();
    /* SLLI a3, a4, 5: op0=0, op1=1, op2=0(sh[4]=0), r=3(dest), s=4(src), t=5(sh[3:0])
       sh = (op2[0]<<4)|t = (0<<4)|5 = 5
       insn = (0<<20)|(1<<16)|(3<<12)|(4<<8)|(5<<4)|0 = 0x013450 */
    put_insn3(&cpu, BASE, 0x013450);
    ASSERT_TRUE(check_disasm(&cpu, BASE, "slli\ta3, a4, 5", 3));
    mem_destroy(cpu.mem);
}

TEST(test_slli_large) {
    xtensa_cpu_t cpu;
    xtensa_cpu_init(&cpu);
    cpu.mem = mem_create();
    /* SLLI a3, a4, 20: sh=20, sh[4]=1 → op2=1, sh[3:0]=4 → t=4
       insn = (1<<20)|(1<<16)|(3<<12)|(4<<8)|(4<<4)|0 = 0x113440 */
    put_insn3(&cpu, BASE, 0x113440);
    ASSERT_TRUE(check_disasm(&cpu, BASE, "slli\ta3, a4, 20", 3));
    mem_destroy(cpu.mem);
}

TEST(test_srai) {
    xtensa_cpu_t cpu;
    xtensa_cpu_init(&cpu);
    cpu.mem = mem_create();
    /* SRAI a3, a4, 7: op0=0, op1=1, op2=2(sh[4]=0), r=3(dest), s=7(sh[3:0]), t=4(src)
       sh = (op2[0]<<4)|s = (0<<4)|7 = 7
       insn = (2<<20)|(1<<16)|(3<<12)|(7<<8)|(4<<4)|0 = 0x213740 */
    put_insn3(&cpu, BASE, 0x213740);
    ASSERT_TRUE(check_disasm(&cpu, BASE, "srai\ta3, a4, 7", 3));
    mem_destroy(cpu.mem);
}

TEST(test_srai_large) {
    xtensa_cpu_t cpu;
    xtensa_cpu_init(&cpu);
    cpu.mem = mem_create();
    /* SRAI a3, a4, 25: sh=25, sh[4]=1 → op2=3, sh[3:0]=9 → s=9
       insn = (3<<20)|(1<<16)|(3<<12)|(9<<8)|(4<<4)|0 = 0x313940 */
    put_insn3(&cpu, BASE, 0x313940);
    ASSERT_TRUE(check_disasm(&cpu, BASE, "srai\ta3, a4, 25", 3));
    mem_destroy(cpu.mem);
}

TEST(test_srli) {
    xtensa_cpu_t cpu;
    xtensa_cpu_init(&cpu);
    cpu.mem = mem_create();
    /* SRLI a3, a4, 8: op0=0, op1=1, op2=4, r=3(dest), s=8(shift), t=4(src)
       insn = (4<<20)|(1<<16)|(3<<12)|(8<<8)|(4<<4)|0 = 0x413840 */
    put_insn3(&cpu, BASE, 0x413840);
    ASSERT_TRUE(check_disasm(&cpu, BASE, "srli\ta3, a4, 8", 3));
    mem_destroy(cpu.mem);
}

TEST(test_extui) {
    xtensa_cpu_t cpu;
    xtensa_cpu_init(&cpu);
    cpu.mem = mem_create();
    /* EXTUI a3, a4, 8, 4: shift=8, maskwidth=4
       op2=imm[3:0]=maskwidth-1=3, op1=010sh[4]=0100(sh[4]=0), r=3(dest), s=sh[3:0]=8, t=4(src)
       insn = (3<<20)|(4<<16)|(3<<12)|(8<<8)|(4<<4)|0 = 0x343840 */
    put_insn3(&cpu, BASE, 0x343840);
    ASSERT_TRUE(check_disasm(&cpu, BASE, "extui\ta3, a4, 8, 4", 3));
    mem_destroy(cpu.mem);
}

TEST(test_extui_large_shift) {
    xtensa_cpu_t cpu;
    xtensa_cpu_init(&cpu);
    cpu.mem = mem_create();
    /* EXTUI a3, a4, 24, 8: shift=24, maskwidth=8
       sh[4]=1 → op1=0101=5, sh[3:0]=8 → s=8, op2=maskwidth-1=7, r=3, t=4
       insn = (7<<20)|(5<<16)|(3<<12)|(8<<8)|(4<<4)|0 = 0x753840 */
    put_insn3(&cpu, BASE, 0x753840);
    ASSERT_TRUE(check_disasm(&cpu, BASE, "extui\ta3, a4, 24, 8", 3));
    mem_destroy(cpu.mem);
}

TEST(test_movi_n_negative) {
    xtensa_cpu_t cpu;
    xtensa_cpu_init(&cpu);
    cpu.mem = mem_create();
    /* MOVI.N a5, -32: RI7 format, op0=0xC
       imm7 is sign-extended: -32 in 7-bit = 0x60 = 0b1100000
       imm7[3:0] = r = 0, imm7[6:4] = t[2:0] = 6, i = t[3] = 0 (always 0 for MOVI.N)
       t = (0<<3)|6 = 6, s=5, r=0
       insn16 = (0<<12)|(5<<8)|(6<<4)|0xC = 0x056C */
    put_insn2(&cpu, BASE, 0x056C);
    ASSERT_TRUE(check_disasm(&cpu, BASE, "movi.n\ta5, -32", 2));
    mem_destroy(cpu.mem);
}

/* ===== Multiply/Divide Tests ===== */

TEST(test_mull) {
    xtensa_cpu_t cpu;
    xtensa_cpu_init(&cpu);
    cpu.mem = mem_create();
    /* MULL a3, a4, a5: op0=0, op1=2, op2=8, r=3, s=4, t=5
       insn = (8<<20)|(2<<16)|(3<<12)|(4<<8)|(5<<4)|0 = 0x823450 */
    put_insn3(&cpu, BASE, 0x823450);
    ASSERT_TRUE(check_disasm(&cpu, BASE, "mull\ta3, a4, a5", 3));
    mem_destroy(cpu.mem);
}

TEST(test_quou) {
    xtensa_cpu_t cpu;
    xtensa_cpu_init(&cpu);
    cpu.mem = mem_create();
    /* QUOU a3, a4, a5: op0=0, op1=2, op2=12, r=3, s=4, t=5
       insn = (12<<20)|(2<<16)|(3<<12)|(4<<8)|(5<<4)|0 = 0xC23450 */
    put_insn3(&cpu, BASE, 0xC23450);
    ASSERT_TRUE(check_disasm(&cpu, BASE, "quou\ta3, a4, a5", 3));
    mem_destroy(cpu.mem);
}

/* ===== Misc Ops Tests ===== */

TEST(test_min_max) {
    xtensa_cpu_t cpu;
    xtensa_cpu_init(&cpu);
    cpu.mem = mem_create();
    /* MIN a3, a4, a5: op0=0, op1=3, op2=4, r=3, s=4, t=5
       insn = (4<<20)|(3<<16)|(3<<12)|(4<<8)|(5<<4)|0 = 0x433450 */
    put_insn3(&cpu, BASE, 0x433450);
    ASSERT_TRUE(check_disasm(&cpu, BASE, "min\ta3, a4, a5", 3));
    mem_destroy(cpu.mem);
}

TEST(test_moveqz) {
    xtensa_cpu_t cpu;
    xtensa_cpu_init(&cpu);
    cpu.mem = mem_create();
    /* MOVEQZ a3, a4, a5: op0=0, op1=3, op2=8, r=3, s=4, t=5
       insn = (8<<20)|(3<<16)|(3<<12)|(4<<8)|(5<<4)|0 = 0x833450 */
    put_insn3(&cpu, BASE, 0x833450);
    ASSERT_TRUE(check_disasm(&cpu, BASE, "moveqz\ta3, a4, a5", 3));
    mem_destroy(cpu.mem);
}

/* ===== L32R Test ===== */

TEST(test_l32r) {
    xtensa_cpu_t cpu;
    xtensa_cpu_init(&cpu);
    cpu.mem = mem_create();
    /* L32R a3, [literal]: op0=1, t=3, imm16
       The literal is at a negative offset from ((PC+3)&~3).
       With imm16 = 0xFFFF: offset = 0xFFFC0000 | (0xFFFF << 2) = 0xFFFFFFFC
       target = ((BASE+3)&~3) + 0xFFFFFFFC = BASE + 4 - 4 = BASE
       insn = (0xFFFF << 8) | (3 << 4) | 1 = 0xFFFF31 */
    put_insn3(&cpu, BASE, 0xFFFF31);
    ASSERT_TRUE(check_disasm(&cpu, BASE, "l32r\ta3", 3));
    mem_destroy(cpu.mem);
}

/* ===== Entry Test ===== */

TEST(test_entry) {
    xtensa_cpu_t cpu;
    xtensa_cpu_init(&cpu);
    cpu.mem = mem_create();
    /* ENTRY a1, 32: BRI12 format, op0=6, n=3, m=0, s=1
       framesize = imm12 << 3, so imm12 = 32/8 = 4
       insn = (4 << 12) | (1 << 8) | (0 << 6) | (3 << 4) | 6
            = 0x004000 | 0x000100 | 0x000000 | 0x000030 | 0x000006
            = 0x004136 */
    put_insn3(&cpu, BASE, 0x004136);
    ASSERT_TRUE(check_disasm(&cpu, BASE, "entry\ta1, 32", 3));
    mem_destroy(cpu.mem);
}

void run_decode_tests(void) {
    TEST_SUITE("Instruction Decode");

    RUN_TEST(test_nop);
    RUN_TEST(test_add);
    RUN_TEST(test_sub);
    RUN_TEST(test_and_or_xor);
    RUN_TEST(test_neg_abs);
    RUN_TEST(test_slli);
    RUN_TEST(test_slli_large);
    RUN_TEST(test_srai);
    RUN_TEST(test_srai_large);
    RUN_TEST(test_srli);
    RUN_TEST(test_extui);
    RUN_TEST(test_extui_large_shift);
    RUN_TEST(test_addi);
    RUN_TEST(test_addi_negative);
    RUN_TEST(test_movi);
    RUN_TEST(test_movi_negative);
    RUN_TEST(test_l32i);
    RUN_TEST(test_s32i);
    RUN_TEST(test_rsr);
    RUN_TEST(test_wsr);
    RUN_TEST(test_beq);
    RUN_TEST(test_bnez);
    RUN_TEST(test_ret);
    RUN_TEST(test_call0);
    RUN_TEST(test_j);
    RUN_TEST(test_add_n);
    RUN_TEST(test_addi_n);
    RUN_TEST(test_addi_n_minus1);
    RUN_TEST(test_l32i_n);
    RUN_TEST(test_s32i_n);
    RUN_TEST(test_mov_n);
    RUN_TEST(test_ret_n);
    RUN_TEST(test_nop_n);
    RUN_TEST(test_movi_n);
    RUN_TEST(test_movi_n_negative);
    RUN_TEST(test_mull);
    RUN_TEST(test_quou);
    RUN_TEST(test_min_max);
    RUN_TEST(test_moveqz);
    RUN_TEST(test_l32r);
    RUN_TEST(test_entry);
}
