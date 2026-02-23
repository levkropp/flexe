/*
 * Tests for FP arithmetic, comparisons, conversions, conditional moves,
 * indexed loads/stores, and div/sqrt helper instructions.
 */
#include "test_helpers.h"
#include <string.h>
#include <math.h>

/* Helper: build RRR encoding for FP ops (op0=0) */
/* rrr() already defined in test_helpers.h: rrr(op2, op1, r, s, t) */

/* float bit helpers */
static uint32_t f2b(float f) { uint32_t b; memcpy(&b, &f, 4); return b; }
static float b2f(uint32_t b) { float f; memcpy(&f, &b, 4); return f; }

static void set_fr(xtensa_cpu_t *cpu, int n, float val) {
    memcpy(&cpu->fr[n], &val, 4);
}

static uint32_t get_fr_bits(xtensa_cpu_t *cpu, int n) {
    uint32_t b; memcpy(&b, &cpu->fr[n], 4); return b;
}

/* ===== Basic Arithmetic ===== */

TEST(exec_add_s_basic) {
    xtensa_cpu_t cpu; setup(&cpu);
    set_fr(&cpu, 1, 1.5f);
    set_fr(&cpu, 2, 2.5f);
    /* ADD.S f3, f1, f2: op2=0, op1=10, r=3, s=1, t=2 */
    put_insn3(&cpu, BASE, rrr(0, 10, 3, 1, 2));
    xtensa_step(&cpu);
    ASSERT_EQ(get_fr_bits(&cpu, 3), f2b(4.0f));
    teardown(&cpu);
}

TEST(exec_sub_s_basic) {
    xtensa_cpu_t cpu; setup(&cpu);
    set_fr(&cpu, 1, 5.0f);
    set_fr(&cpu, 2, 3.0f);
    /* SUB.S f3, f1, f2: op2=1, op1=10 */
    put_insn3(&cpu, BASE, rrr(1, 10, 3, 1, 2));
    xtensa_step(&cpu);
    ASSERT_EQ(get_fr_bits(&cpu, 3), f2b(2.0f));
    teardown(&cpu);
}

TEST(exec_mul_s_basic) {
    xtensa_cpu_t cpu; setup(&cpu);
    set_fr(&cpu, 1, 3.0f);
    set_fr(&cpu, 2, 4.0f);
    /* MUL.S f3, f1, f2: op2=2, op1=10 */
    put_insn3(&cpu, BASE, rrr(2, 10, 3, 1, 2));
    xtensa_step(&cpu);
    ASSERT_EQ(get_fr_bits(&cpu, 3), f2b(12.0f));
    teardown(&cpu);
}

TEST(exec_madd_s_basic) {
    xtensa_cpu_t cpu; setup(&cpu);
    set_fr(&cpu, 3, 1.0f);  /* accumulator */
    set_fr(&cpu, 1, 2.0f);
    set_fr(&cpu, 2, 3.0f);
    /* MADD.S f3, f1, f2: op2=4, op1=10 → f3 = 1.0 + 2.0*3.0 = 7.0 */
    put_insn3(&cpu, BASE, rrr(4, 10, 3, 1, 2));
    xtensa_step(&cpu);
    ASSERT_EQ(get_fr_bits(&cpu, 3), f2b(7.0f));
    teardown(&cpu);
}

TEST(exec_msub_s_basic) {
    xtensa_cpu_t cpu; setup(&cpu);
    set_fr(&cpu, 3, 10.0f);
    set_fr(&cpu, 1, 2.0f);
    set_fr(&cpu, 2, 3.0f);
    /* MSUB.S f3, f1, f2: op2=5, op1=10 → f3 = 10.0 - 2.0*3.0 = 4.0 */
    put_insn3(&cpu, BASE, rrr(5, 10, 3, 1, 2));
    xtensa_step(&cpu);
    ASSERT_EQ(get_fr_bits(&cpu, 3), f2b(4.0f));
    teardown(&cpu);
}

TEST(exec_add_s_negative) {
    xtensa_cpu_t cpu; setup(&cpu);
    set_fr(&cpu, 1, -2.5f);
    set_fr(&cpu, 2, 1.0f);
    put_insn3(&cpu, BASE, rrr(0, 10, 3, 1, 2));
    xtensa_step(&cpu);
    ASSERT_EQ(get_fr_bits(&cpu, 3), f2b(-1.5f));
    teardown(&cpu);
}

TEST(exec_mul_s_zero) {
    xtensa_cpu_t cpu; setup(&cpu);
    set_fr(&cpu, 1, 42.0f);
    set_fr(&cpu, 2, 0.0f);
    put_insn3(&cpu, BASE, rrr(2, 10, 3, 1, 2));
    xtensa_step(&cpu);
    ASSERT_EQ(get_fr_bits(&cpu, 3), f2b(0.0f));
    teardown(&cpu);
}

/* ===== Data Movement ===== */

TEST(exec_mov_s) {
    xtensa_cpu_t cpu; setup(&cpu);
    set_fr(&cpu, 1, 3.14f);
    /* MOV.S f3, f1: op2=15, op1=10, r=3, s=1, t=0 */
    put_insn3(&cpu, BASE, rrr(15, 10, 3, 1, 0));
    xtensa_step(&cpu);
    ASSERT_EQ(get_fr_bits(&cpu, 3), f2b(3.14f));
    teardown(&cpu);
}

TEST(exec_abs_s_positive) {
    xtensa_cpu_t cpu; setup(&cpu);
    set_fr(&cpu, 1, 5.0f);
    /* ABS.S f3, f1: op2=15, op1=10, r=3, s=1, t=1 */
    put_insn3(&cpu, BASE, rrr(15, 10, 3, 1, 1));
    xtensa_step(&cpu);
    ASSERT_EQ(get_fr_bits(&cpu, 3), f2b(5.0f));
    teardown(&cpu);
}

TEST(exec_abs_s_negative) {
    xtensa_cpu_t cpu; setup(&cpu);
    set_fr(&cpu, 1, -7.0f);
    put_insn3(&cpu, BASE, rrr(15, 10, 3, 1, 1));
    xtensa_step(&cpu);
    ASSERT_EQ(get_fr_bits(&cpu, 3), f2b(7.0f));
    teardown(&cpu);
}

TEST(exec_neg_s) {
    xtensa_cpu_t cpu; setup(&cpu);
    set_fr(&cpu, 1, 4.0f);
    /* NEG.S f3, f1: op2=15, op1=10, r=3, s=1, t=6 */
    put_insn3(&cpu, BASE, rrr(15, 10, 3, 1, 6));
    xtensa_step(&cpu);
    ASSERT_EQ(get_fr_bits(&cpu, 3), f2b(-4.0f));
    teardown(&cpu);
}

TEST(exec_rfr_basic) {
    xtensa_cpu_t cpu; setup(&cpu);
    set_fr(&cpu, 2, 1.5f);
    /* RFR a3, f2: op2=15, op1=10, r=3, s=2, t=4 */
    put_insn3(&cpu, BASE, rrr(15, 10, 3, 2, 4));
    xtensa_step(&cpu);
    ASSERT_EQ(ar_read(&cpu, 3), f2b(1.5f));
    teardown(&cpu);
}

TEST(exec_wfr_basic) {
    xtensa_cpu_t cpu; setup(&cpu);
    ar_write(&cpu, 2, f2b(2.5f));
    /* WFR f3, a2: op2=15, op1=10, r=3, s=2, t=5 */
    put_insn3(&cpu, BASE, rrr(15, 10, 3, 2, 5));
    xtensa_step(&cpu);
    ASSERT_EQ(get_fr_bits(&cpu, 3), f2b(2.5f));
    teardown(&cpu);
}

TEST(exec_wfr_rfr_roundtrip) {
    xtensa_cpu_t cpu; setup(&cpu);
    uint32_t bits = 0xDEADBEEF;
    ar_write(&cpu, 4, bits);
    /* WFR f5, a4 */
    put_insn3(&cpu, BASE, rrr(15, 10, 5, 4, 5));
    xtensa_step(&cpu);
    /* RFR a6, f5 */
    cpu.pc = BASE;
    put_insn3(&cpu, BASE, rrr(15, 10, 6, 5, 4));
    xtensa_step(&cpu);
    ASSERT_EQ(ar_read(&cpu, 6), bits);
    teardown(&cpu);
}

/* ===== Conversions ===== */

TEST(exec_float_s_basic) {
    xtensa_cpu_t cpu; setup(&cpu);
    ar_write(&cpu, 2, 42);
    /* FLOAT.S f3, a2, 0: op2=12, op1=10, r=3, s=2, t=0 */
    put_insn3(&cpu, BASE, rrr(12, 10, 3, 2, 0));
    xtensa_step(&cpu);
    ASSERT_EQ(get_fr_bits(&cpu, 3), f2b(42.0f));
    teardown(&cpu);
}

TEST(exec_float_s_negative) {
    xtensa_cpu_t cpu; setup(&cpu);
    ar_write(&cpu, 2, (uint32_t)(int32_t)-7);
    /* FLOAT.S f3, a2, 0 */
    put_insn3(&cpu, BASE, rrr(12, 10, 3, 2, 0));
    xtensa_step(&cpu);
    ASSERT_EQ(get_fr_bits(&cpu, 3), f2b(-7.0f));
    teardown(&cpu);
}

TEST(exec_float_s_scale) {
    xtensa_cpu_t cpu; setup(&cpu);
    ar_write(&cpu, 2, 100);
    /* FLOAT.S f3, a2, 2: scale by 2^(-2) → 100 * 0.25 = 25.0 */
    put_insn3(&cpu, BASE, rrr(12, 10, 3, 2, 2));
    xtensa_step(&cpu);
    ASSERT_EQ(get_fr_bits(&cpu, 3), f2b(25.0f));
    teardown(&cpu);
}

TEST(exec_trunc_s_basic) {
    xtensa_cpu_t cpu; setup(&cpu);
    set_fr(&cpu, 2, 3.7f);
    /* TRUNC.S a4, f2, 0: op2=9, op1=10, r=0, s=2, t=4 */
    put_insn3(&cpu, BASE, rrr(9, 10, 0, 2, 4));
    xtensa_step(&cpu);
    ASSERT_EQ(ar_read(&cpu, 4), 3);
    teardown(&cpu);
}

TEST(exec_trunc_s_negative) {
    xtensa_cpu_t cpu; setup(&cpu);
    set_fr(&cpu, 2, -3.7f);
    put_insn3(&cpu, BASE, rrr(9, 10, 0, 2, 4));
    xtensa_step(&cpu);
    ASSERT_EQ(ar_read(&cpu, 4), (uint32_t)(int32_t)-3);
    teardown(&cpu);
}

TEST(exec_round_s) {
    xtensa_cpu_t cpu; setup(&cpu);
    set_fr(&cpu, 2, 3.5f);
    /* ROUND.S a4, f2, 0: op2=8, op1=10, r=0, s=2, t=4 */
    put_insn3(&cpu, BASE, rrr(8, 10, 0, 2, 4));
    xtensa_step(&cpu);
    ASSERT_EQ(ar_read(&cpu, 4), 4);
    teardown(&cpu);
}

TEST(exec_floor_s) {
    xtensa_cpu_t cpu; setup(&cpu);
    set_fr(&cpu, 2, 3.7f);
    /* FLOOR.S a4, f2, 0: op2=10, op1=10, r=0, s=2, t=4 */
    put_insn3(&cpu, BASE, rrr(10, 10, 0, 2, 4));
    xtensa_step(&cpu);
    ASSERT_EQ(ar_read(&cpu, 4), 3);
    teardown(&cpu);
}

TEST(exec_ceil_s) {
    xtensa_cpu_t cpu; setup(&cpu);
    set_fr(&cpu, 2, 3.2f);
    /* CEIL.S a4, f2, 0: op2=11, op1=10, r=0, s=2, t=4 */
    put_insn3(&cpu, BASE, rrr(11, 10, 0, 2, 4));
    xtensa_step(&cpu);
    ASSERT_EQ(ar_read(&cpu, 4), 4);
    teardown(&cpu);
}

TEST(exec_ufloat_s) {
    xtensa_cpu_t cpu; setup(&cpu);
    ar_write(&cpu, 2, 100);
    /* UFLOAT.S f3, a2, 0: op2=13, op1=10, r=3, s=2, t=0 */
    put_insn3(&cpu, BASE, rrr(13, 10, 3, 2, 0));
    xtensa_step(&cpu);
    ASSERT_EQ(get_fr_bits(&cpu, 3), f2b(100.0f));
    teardown(&cpu);
}

TEST(exec_utrunc_s) {
    xtensa_cpu_t cpu; setup(&cpu);
    set_fr(&cpu, 2, 100.5f);
    /* UTRUNC.S a4, f2, 0: op2=14, op1=10, r=0, s=2, t=4 */
    put_insn3(&cpu, BASE, rrr(14, 10, 0, 2, 4));
    xtensa_step(&cpu);
    ASSERT_EQ(ar_read(&cpu, 4), 100);
    teardown(&cpu);
}

/* ===== Comparisons ===== */

TEST(exec_oeq_s_equal) {
    xtensa_cpu_t cpu; setup(&cpu);
    set_fr(&cpu, 1, 2.0f);
    set_fr(&cpu, 2, 2.0f);
    cpu.br = 0;
    /* OEQ.S b3, f1, f2: op2=2, op1=11, r=3, s=1, t=2 */
    put_insn3(&cpu, BASE, rrr(2, 11, 3, 1, 2));
    xtensa_step(&cpu);
    ASSERT_EQ((cpu.br >> 3) & 1, 1);
    teardown(&cpu);
}

TEST(exec_oeq_s_notequal) {
    xtensa_cpu_t cpu; setup(&cpu);
    set_fr(&cpu, 1, 2.0f);
    set_fr(&cpu, 2, 3.0f);
    cpu.br = 0xFF;
    put_insn3(&cpu, BASE, rrr(2, 11, 3, 1, 2));
    xtensa_step(&cpu);
    ASSERT_EQ((cpu.br >> 3) & 1, 0);
    teardown(&cpu);
}

TEST(exec_olt_s_less) {
    xtensa_cpu_t cpu; setup(&cpu);
    set_fr(&cpu, 1, 1.0f);
    set_fr(&cpu, 2, 2.0f);
    cpu.br = 0;
    /* OLT.S b0, f1, f2: op2=4, op1=11 */
    put_insn3(&cpu, BASE, rrr(4, 11, 0, 1, 2));
    xtensa_step(&cpu);
    ASSERT_EQ(cpu.br & 1, 1);
    teardown(&cpu);
}

TEST(exec_ole_s_equal) {
    xtensa_cpu_t cpu; setup(&cpu);
    set_fr(&cpu, 1, 1.0f);
    set_fr(&cpu, 2, 1.0f);
    cpu.br = 0;
    /* OLE.S b0, f1, f2: op2=6, op1=11 */
    put_insn3(&cpu, BASE, rrr(6, 11, 0, 1, 2));
    xtensa_step(&cpu);
    ASSERT_EQ(cpu.br & 1, 1);
    teardown(&cpu);
}

TEST(exec_un_s_nan) {
    xtensa_cpu_t cpu; setup(&cpu);
    set_fr(&cpu, 1, b2f(0x7FC00000)); /* NaN */
    set_fr(&cpu, 2, 1.0f);
    cpu.br = 0;
    /* UN.S b0, f1, f2: op2=1, op1=11 */
    put_insn3(&cpu, BASE, rrr(1, 11, 0, 1, 2));
    xtensa_step(&cpu);
    ASSERT_EQ(cpu.br & 1, 1);
    teardown(&cpu);
}

TEST(exec_un_s_normal) {
    xtensa_cpu_t cpu; setup(&cpu);
    set_fr(&cpu, 1, 1.0f);
    set_fr(&cpu, 2, 2.0f);
    cpu.br = 0xFF;
    put_insn3(&cpu, BASE, rrr(1, 11, 0, 1, 2));
    xtensa_step(&cpu);
    ASSERT_EQ(cpu.br & 1, 0);
    teardown(&cpu);
}

/* ===== Conditional FP Moves ===== */

TEST(exec_moveqz_s_taken) {
    xtensa_cpu_t cpu; setup(&cpu);
    set_fr(&cpu, 1, 5.0f);
    set_fr(&cpu, 3, 0.0f);
    ar_write(&cpu, 4, 0); /* condition: ar[t] == 0 */
    /* MOVEQZ.S f3, f1, a4: op2=8, op1=11, r=3, s=1, t=4 */
    put_insn3(&cpu, BASE, rrr(8, 11, 3, 1, 4));
    xtensa_step(&cpu);
    ASSERT_EQ(get_fr_bits(&cpu, 3), f2b(5.0f));
    teardown(&cpu);
}

TEST(exec_moveqz_s_not) {
    xtensa_cpu_t cpu; setup(&cpu);
    set_fr(&cpu, 1, 5.0f);
    set_fr(&cpu, 3, 0.0f);
    ar_write(&cpu, 4, 1); /* condition: ar[t] != 0, no move */
    put_insn3(&cpu, BASE, rrr(8, 11, 3, 1, 4));
    xtensa_step(&cpu);
    ASSERT_EQ(get_fr_bits(&cpu, 3), f2b(0.0f));
    teardown(&cpu);
}

TEST(exec_movnez_s) {
    xtensa_cpu_t cpu; setup(&cpu);
    set_fr(&cpu, 1, 5.0f);
    set_fr(&cpu, 3, 0.0f);
    ar_write(&cpu, 4, 1);
    /* MOVNEZ.S f3, f1, a4: op2=9, op1=11 */
    put_insn3(&cpu, BASE, rrr(9, 11, 3, 1, 4));
    xtensa_step(&cpu);
    ASSERT_EQ(get_fr_bits(&cpu, 3), f2b(5.0f));
    teardown(&cpu);
}

TEST(exec_movltz_s) {
    xtensa_cpu_t cpu; setup(&cpu);
    set_fr(&cpu, 1, 5.0f);
    set_fr(&cpu, 3, 0.0f);
    ar_write(&cpu, 4, (uint32_t)(int32_t)-1);
    /* MOVLTZ.S f3, f1, a4: op2=10, op1=11 */
    put_insn3(&cpu, BASE, rrr(10, 11, 3, 1, 4));
    xtensa_step(&cpu);
    ASSERT_EQ(get_fr_bits(&cpu, 3), f2b(5.0f));
    teardown(&cpu);
}

TEST(exec_movgez_s) {
    xtensa_cpu_t cpu; setup(&cpu);
    set_fr(&cpu, 1, 5.0f);
    set_fr(&cpu, 3, 0.0f);
    ar_write(&cpu, 4, 0);
    /* MOVGEZ.S f3, f1, a4: op2=11, op1=11 */
    put_insn3(&cpu, BASE, rrr(11, 11, 3, 1, 4));
    xtensa_step(&cpu);
    ASSERT_EQ(get_fr_bits(&cpu, 3), f2b(5.0f));
    teardown(&cpu);
}

TEST(exec_movf_s_taken) {
    xtensa_cpu_t cpu; setup(&cpu);
    set_fr(&cpu, 1, 5.0f);
    set_fr(&cpu, 3, 0.0f);
    cpu.br = 0; /* b2 = 0 → MOVF taken */
    /* MOVF.S f3, f1, b2: op2=12, op1=11, r=3, s=1, t=2 */
    put_insn3(&cpu, BASE, rrr(12, 11, 3, 1, 2));
    xtensa_step(&cpu);
    ASSERT_EQ(get_fr_bits(&cpu, 3), f2b(5.0f));
    teardown(&cpu);
}

TEST(exec_movt_s_taken) {
    xtensa_cpu_t cpu; setup(&cpu);
    set_fr(&cpu, 1, 5.0f);
    set_fr(&cpu, 3, 0.0f);
    cpu.br = (1u << 2); /* b2 = 1 → MOVT taken */
    /* MOVT.S f3, f1, b2: op2=13, op1=11, r=3, s=1, t=2 */
    put_insn3(&cpu, BASE, rrr(13, 11, 3, 1, 2));
    xtensa_step(&cpu);
    ASSERT_EQ(get_fr_bits(&cpu, 3), f2b(5.0f));
    teardown(&cpu);
}

/* ===== Indexed FP Loads/Stores ===== */

TEST(exec_lsx_basic) {
    xtensa_cpu_t cpu; setup(&cpu);
    uint32_t addr = BASE + 0x200;
    ar_write(&cpu, 4, addr);
    ar_write(&cpu, 5, 8); /* offset */
    float val = 6.28f;
    mem_write32(cpu.mem, addr + 8, f2b(val));
    /* LSX f3, a4, a5: op2=0, op1=8, r=3, s=4, t=5 */
    put_insn3(&cpu, BASE, rrr(0, 8, 3, 4, 5));
    xtensa_step(&cpu);
    ASSERT_EQ(get_fr_bits(&cpu, 3), f2b(6.28f));
    teardown(&cpu);
}

TEST(exec_ssx_basic) {
    xtensa_cpu_t cpu; setup(&cpu);
    uint32_t addr = BASE + 0x200;
    ar_write(&cpu, 4, addr);
    ar_write(&cpu, 5, 12);
    set_fr(&cpu, 3, 9.81f);
    /* SSX f3, a4, a5: op2=4, op1=8, r=3, s=4, t=5 */
    put_insn3(&cpu, BASE, rrr(4, 8, 3, 4, 5));
    xtensa_step(&cpu);
    ASSERT_EQ(mem_read32(cpu.mem, addr + 12), f2b(9.81f));
    teardown(&cpu);
}

TEST(exec_lsxp_post_update) {
    xtensa_cpu_t cpu; setup(&cpu);
    uint32_t addr = BASE + 0x200;
    ar_write(&cpu, 4, addr);
    ar_write(&cpu, 5, 16); /* post-increment */
    mem_write32(cpu.mem, addr, f2b(1.23f));
    /* LSXP f3, a4, a5: op2=1, op1=8, r=3, s=4, t=5 */
    put_insn3(&cpu, BASE, rrr(1, 8, 3, 4, 5));
    xtensa_step(&cpu);
    ASSERT_EQ(get_fr_bits(&cpu, 3), f2b(1.23f));
    ASSERT_EQ(ar_read(&cpu, 4), addr + 16);
    teardown(&cpu);
}

TEST(exec_ssxp_post_update) {
    xtensa_cpu_t cpu; setup(&cpu);
    uint32_t addr = BASE + 0x200;
    ar_write(&cpu, 4, addr);
    ar_write(&cpu, 5, 8);
    set_fr(&cpu, 3, 4.56f);
    /* SSXP f3, a4, a5: op2=5, op1=8, r=3, s=4, t=5 */
    put_insn3(&cpu, BASE, rrr(5, 8, 3, 4, 5));
    xtensa_step(&cpu);
    ASSERT_EQ(mem_read32(cpu.mem, addr), f2b(4.56f));
    ASSERT_EQ(ar_read(&cpu, 4), addr + 8);
    teardown(&cpu);
}

/* ===== Div/Sqrt helpers (basic validation via full sequence) ===== */

TEST(exec_recip0_s) {
    xtensa_cpu_t cpu; setup(&cpu);
    set_fr(&cpu, 2, 4.0f);
    /* RECIP0.S f3, f2: op2=15, op1=10, r=3, s=2, t=8 */
    put_insn3(&cpu, BASE, rrr(15, 10, 3, 2, 8));
    xtensa_step(&cpu);
    ASSERT_EQ(get_fr_bits(&cpu, 3), f2b(0.25f));
    teardown(&cpu);
}

TEST(exec_sqrt0_s) {
    xtensa_cpu_t cpu; setup(&cpu);
    set_fr(&cpu, 2, 4.0f);
    /* SQRT0.S f3, f2: op2=15, op1=10, r=3, s=2, t=9 */
    put_insn3(&cpu, BASE, rrr(15, 10, 3, 2, 9));
    xtensa_step(&cpu);
    ASSERT_EQ(get_fr_bits(&cpu, 3), f2b(2.0f));
    teardown(&cpu);
}

TEST(exec_const_s) {
    xtensa_cpu_t cpu; setup(&cpu);
    /* CONST.S f3, 1: should load 1.0f. op2=15, op1=10, r=3, s=1, t=3 */
    put_insn3(&cpu, BASE, rrr(15, 10, 3, 1, 3));
    xtensa_step(&cpu);
    ASSERT_EQ(get_fr_bits(&cpu, 3), f2b(1.0f));
    teardown(&cpu);
}

/* ===== Disassembly Spot Checks ===== */

TEST(disasm_add_s) {
    xtensa_cpu_t cpu; setup(&cpu);
    put_insn3(&cpu, BASE, rrr(0, 10, 3, 1, 2));
    char buf[64];
    xtensa_disasm(&cpu, BASE, buf, sizeof(buf));
    ASSERT_TRUE(strstr(buf, "add.s") != NULL);
    teardown(&cpu);
}

TEST(disasm_oeq_s) {
    xtensa_cpu_t cpu; setup(&cpu);
    put_insn3(&cpu, BASE, rrr(2, 11, 3, 1, 2));
    char buf[64];
    xtensa_disasm(&cpu, BASE, buf, sizeof(buf));
    ASSERT_TRUE(strstr(buf, "oeq.s") != NULL);
    teardown(&cpu);
}

TEST(disasm_lsx) {
    xtensa_cpu_t cpu; setup(&cpu);
    put_insn3(&cpu, BASE, rrr(0, 8, 3, 4, 5));
    char buf[64];
    xtensa_disasm(&cpu, BASE, buf, sizeof(buf));
    ASSERT_TRUE(strstr(buf, "lsx") != NULL);
    teardown(&cpu);
}

void run_fp_arith_tests(void) {
    TEST_SUITE("FP Arithmetic & Comparisons");
    /* Arithmetic */
    RUN_TEST(exec_add_s_basic);
    RUN_TEST(exec_sub_s_basic);
    RUN_TEST(exec_mul_s_basic);
    RUN_TEST(exec_madd_s_basic);
    RUN_TEST(exec_msub_s_basic);
    RUN_TEST(exec_add_s_negative);
    RUN_TEST(exec_mul_s_zero);
    /* Data movement */
    RUN_TEST(exec_mov_s);
    RUN_TEST(exec_abs_s_positive);
    RUN_TEST(exec_abs_s_negative);
    RUN_TEST(exec_neg_s);
    RUN_TEST(exec_rfr_basic);
    RUN_TEST(exec_wfr_basic);
    RUN_TEST(exec_wfr_rfr_roundtrip);
    /* Conversions */
    RUN_TEST(exec_float_s_basic);
    RUN_TEST(exec_float_s_negative);
    RUN_TEST(exec_float_s_scale);
    RUN_TEST(exec_trunc_s_basic);
    RUN_TEST(exec_trunc_s_negative);
    RUN_TEST(exec_round_s);
    RUN_TEST(exec_floor_s);
    RUN_TEST(exec_ceil_s);
    RUN_TEST(exec_ufloat_s);
    RUN_TEST(exec_utrunc_s);
    /* Comparisons */
    RUN_TEST(exec_oeq_s_equal);
    RUN_TEST(exec_oeq_s_notequal);
    RUN_TEST(exec_olt_s_less);
    RUN_TEST(exec_ole_s_equal);
    RUN_TEST(exec_un_s_nan);
    RUN_TEST(exec_un_s_normal);
    /* Conditional moves */
    RUN_TEST(exec_moveqz_s_taken);
    RUN_TEST(exec_moveqz_s_not);
    RUN_TEST(exec_movnez_s);
    RUN_TEST(exec_movltz_s);
    RUN_TEST(exec_movgez_s);
    RUN_TEST(exec_movf_s_taken);
    RUN_TEST(exec_movt_s_taken);
    /* Indexed loads/stores */
    RUN_TEST(exec_lsx_basic);
    RUN_TEST(exec_ssx_basic);
    RUN_TEST(exec_lsxp_post_update);
    RUN_TEST(exec_ssxp_post_update);
    /* Div/sqrt helpers */
    RUN_TEST(exec_recip0_s);
    RUN_TEST(exec_sqrt0_s);
    RUN_TEST(exec_const_s);
    /* Disassembly */
    RUN_TEST(disasm_add_s);
    RUN_TEST(disasm_oeq_s);
    RUN_TEST(disasm_lsx);
}
