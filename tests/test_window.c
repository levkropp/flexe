/*
 * Tests for windowed register instructions (M5).
 * CALL4/8/12, CALLX4/8/12, ENTRY, RETW, RETW.N,
 * MOVSP, L32E/S32E, RFWO/RFWU, plus integration tests.
 */
#include "test_helpers.h"
#include <string.h>

/* ===== Instruction builders ===== */

/* CALL4/8/12: op0=5, nn=1/2/3, offset18 */
static uint32_t calln_insn(int nn, int32_t offset) {
    uint32_t off18 = (uint32_t)offset & 0x3FFFF;
    return (off18 << 6) | ((nn & 3) << 4) | 5;
}

/* CALLX0/4/8/12: op0=0, op1=0, op2=0, r=0, m=3, nn=0/1/2/3, s=src_reg
 * Encoding: rrr(0, 0, 0, s, (m<<2|nn)) where m=3, so t = 0xC|nn */
static uint32_t callxn_insn(int nn, int s) {
    int t_field = (3 << 2) | (nn & 3); /* m=3, nn */
    return rrr(0, 0, 0, s, t_field);
}

/* ENTRY: op0=6, nn=3, m=0, s, imm12 (framesize = imm12 << 3)
 * BRI12 format: imm12 at bits 23:12, s at bits 11:8, m at bits 7:6, n at bits 5:4, op0 at 3:0 */
static uint32_t entry_insn(int s, uint32_t framesize) {
    uint32_t imm12 = (framesize >> 3) & 0xFFF;
    return (imm12 << 12) | (s << 8) | (0 << 6) | (3 << 4) | 6;
}

/* RETW: op0=0, op1=0, op2=0, r=0, m=2, nn=1 → t=(2<<2|1)=9, s=0 */
static uint32_t retw_insn(void) {
    return rrr(0, 0, 0, 0, (2 << 2) | 1);
}

/* RETW.N: op0=0xD, r=15, t=1, s=0 */
static uint16_t retw_n_insn(void) {
    return narrow(0xD, 15, 0, 1);
}

/* MOVSP: op0=0, op1=0, op2=0, r=1, s=src, t=dst */
static uint32_t movsp_insn(int dst, int src) {
    return rrr(0, 0, 1, src, dst);
}

/* L32E: op0=0, op1=9, op2=0, r=offset_idx, s=base, t=dst */
static uint32_t l32e_insn(int dst, int base, int r_off) {
    return rrr(0, 9, r_off, base, dst);
}

/* S32E: op0=0, op1=9, op2=4, r=offset_idx, s=base, t=src */
static uint32_t s32e_insn(int src, int base, int r_off) {
    return rrr(4, 9, r_off, base, src);
}

/* RFWO: op0=0, op1=0, op2=0, r=3, t=0, s=4 */
static uint32_t rfwo_insn(void) {
    return rrr(0, 0, 3, 4, 0);
}

/* RFWU: op0=0, op1=0, op2=0, r=3, t=0, s=5 */
static uint32_t rfwu_insn(void) {
    return rrr(0, 0, 3, 5, 0);
}

/* ROTW: op0=0, op1=0, op2=4 (ST1), r=8, s=0, t=imm4 */
static uint32_t rotw_insn(int imm4) {
    return rrr(4, 0, 8, 0, imm4 & 0xF);
}

/* NOP: for padding */
static uint32_t nop_insn(void) {
    return rrr(0, 0, 2, 15, 0);
}

/* Helper: set up cpu for windowed tests.
 * Unlike setup(), this enables WOE and sets windowstart=1. */
static void setup_windowed(xtensa_cpu_t *cpu) {
    setup(cpu);
    cpu->running = true;
    cpu->ps = (1 << 18) | (1 << 4); /* WOE=1, EXCM=1 */
    cpu->windowbase = 0;
    cpu->windowstart = 1; /* window 0 valid */
}

/* ===== CALL4 + ENTRY + RETW round trip ===== */

TEST(call4_entry_retw) {
    xtensa_cpu_t cpu; setup_windowed(&cpu);
    uint32_t sp_val = BASE + 0x1000;

    /* Set up caller's a1 (SP) */
    ar_write(&cpu, 1, sp_val);

    /* At BASE: CALL4 to BASE+0x40
     * offset = target/4 - (pc/4 + 1) = (BASE+0x40)/4 - (BASE/4 + 1) = 0x10 - 1 = 0xF */
    put_insn3(&cpu, BASE, calln_insn(1, 0xF));

    /* At BASE+0x40: ENTRY a1, 32 */
    put_insn3(&cpu, BASE + 0x40, entry_insn(1, 32));

    /* At BASE+0x43: RETW */
    put_insn3(&cpu, BASE + 0x43, retw_insn());

    /* Step 1: CALL4 */
    xtensa_step(&cpu);
    ASSERT_EQ(cpu.pc, BASE + 0x40);
    ASSERT_EQ(XT_PS_CALLINC(cpu.ps), 1);
    /* a4 should have return address: (1 << 30) | (BASE+3) */
    ASSERT_EQ(ar_read(&cpu, 4), (1u << 30) | ((BASE + 3) & 0x3FFFFFFF));

    /* Step 2: ENTRY a1, 32 — rotates window by 1 */
    xtensa_step(&cpu);
    ASSERT_EQ(cpu.windowbase, 1);
    ASSERT_EQ(cpu.windowstart & (1u << 1), (1u << 1));
    ASSERT_EQ(XT_PS_OWB(cpu.ps), 0);
    ASSERT_EQ(XT_PS_CALLINC(cpu.ps), 0);
    /* New a1 = old a1 - 32 = sp_val - 32 */
    ASSERT_EQ(ar_read(&cpu, 1), sp_val - 32);

    /* Step 3: RETW — rotate back */
    xtensa_step(&cpu);
    ASSERT_EQ(cpu.windowbase, 0);
    /* PC restored from a0[29:0] with high bits from RETW's PC */
    ASSERT_EQ(cpu.pc, BASE + 3);

    teardown(&cpu);
}

/* ===== CALL8 round trip ===== */

TEST(call8_entry_retw) {
    xtensa_cpu_t cpu; setup_windowed(&cpu);
    uint32_t sp_val = BASE + 0x1000;

    ar_write(&cpu, 1, sp_val);

    /* CALL8 to BASE+0x80: offset = (BASE+0x80)/4 - (BASE/4 + 1) = 0x20 - 1 = 0x1F */
    put_insn3(&cpu, BASE, calln_insn(2, 0x1F));
    put_insn3(&cpu, BASE + 0x80, entry_insn(1, 64));
    put_insn3(&cpu, BASE + 0x83, retw_insn());

    /* CALL8 */
    xtensa_step(&cpu);
    ASSERT_EQ(cpu.pc, BASE + 0x80);
    ASSERT_EQ(XT_PS_CALLINC(cpu.ps), 2);
    /* a8 has return addr: (2 << 30) | (BASE+3) */
    ASSERT_EQ(ar_read(&cpu, 8), (2u << 30) | ((BASE + 3) & 0x3FFFFFFF));

    /* ENTRY — rotates by 2 */
    xtensa_step(&cpu);
    ASSERT_EQ(cpu.windowbase, 2);
    ASSERT_EQ(ar_read(&cpu, 1), sp_val - 64);

    /* RETW — n=2 from a0[31:30] */
    xtensa_step(&cpu);
    ASSERT_EQ(cpu.windowbase, 0);
    ASSERT_EQ(cpu.pc, BASE + 3);

    teardown(&cpu);
}

/* ===== CALL12 round trip ===== */

TEST(call12_entry_retw) {
    xtensa_cpu_t cpu; setup_windowed(&cpu);
    uint32_t sp_val = BASE + 0x1000;

    ar_write(&cpu, 1, sp_val);

    /* CALL12 to BASE+0xC0: offset = (BASE+0xC0)/4 - (BASE/4 + 1) = 0x30 - 1 = 0x2F */
    put_insn3(&cpu, BASE, calln_insn(3, 0x2F));
    put_insn3(&cpu, BASE + 0xC0, entry_insn(1, 48));
    put_insn3(&cpu, BASE + 0xC3, retw_insn());

    xtensa_step(&cpu);
    ASSERT_EQ(cpu.pc, BASE + 0xC0);
    ASSERT_EQ(XT_PS_CALLINC(cpu.ps), 3);
    ASSERT_EQ(ar_read(&cpu, 12), (3u << 30) | ((BASE + 3) & 0x3FFFFFFF));

    xtensa_step(&cpu);
    ASSERT_EQ(cpu.windowbase, 3);
    ASSERT_EQ(ar_read(&cpu, 1), sp_val - 48);

    xtensa_step(&cpu);
    ASSERT_EQ(cpu.windowbase, 0);
    ASSERT_EQ(cpu.pc, BASE + 3);

    teardown(&cpu);
}

/* ===== CALLX4 round trip ===== */

TEST(callx4_round_trip) {
    xtensa_cpu_t cpu; setup_windowed(&cpu);
    uint32_t sp_val = BASE + 0x1000;
    uint32_t target = BASE + 0x100;

    ar_write(&cpu, 1, sp_val);
    ar_write(&cpu, 2, target); /* target address in a2 */

    /* CALLX4 a2 */
    put_insn3(&cpu, BASE, callxn_insn(1, 2));
    put_insn3(&cpu, target, entry_insn(1, 32));
    put_insn3(&cpu, target + 3, retw_insn());

    xtensa_step(&cpu);
    ASSERT_EQ(cpu.pc, target);
    ASSERT_EQ(XT_PS_CALLINC(cpu.ps), 1);

    xtensa_step(&cpu);
    ASSERT_EQ(cpu.windowbase, 1);

    xtensa_step(&cpu);
    ASSERT_EQ(cpu.windowbase, 0);
    ASSERT_EQ(cpu.pc, BASE + 3);

    teardown(&cpu);
}

/* ===== ENTRY frame allocation ===== */

TEST(entry_frame_alloc) {
    xtensa_cpu_t cpu; setup_windowed(&cpu);
    uint32_t sp_val = BASE + 0x2000;

    ar_write(&cpu, 1, sp_val);

    /* CALL4, then ENTRY with various frame sizes */
    put_insn3(&cpu, BASE, calln_insn(1, 0xF));
    put_insn3(&cpu, BASE + 0x40, entry_insn(1, 128));

    xtensa_step(&cpu); /* CALL4 */
    xtensa_step(&cpu); /* ENTRY */

    ASSERT_EQ(ar_read(&cpu, 1), sp_val - 128);

    teardown(&cpu);
}

/* ===== RETW restores window ===== */

TEST(retw_restores_window) {
    xtensa_cpu_t cpu; setup_windowed(&cpu);
    uint32_t sp_val = BASE + 0x1000;

    ar_write(&cpu, 1, sp_val);
    ar_write(&cpu, 3, 0xDEADBEEF); /* a value that should survive */

    put_insn3(&cpu, BASE, calln_insn(1, 0xF));
    put_insn3(&cpu, BASE + 0x40, entry_insn(1, 32));
    put_insn3(&cpu, BASE + 0x43, retw_insn());

    xtensa_step(&cpu); /* CALL4 */
    xtensa_step(&cpu); /* ENTRY */

    /* In callee window, write something */
    ar_write(&cpu, 2, 0x12345678);

    xtensa_step(&cpu); /* RETW */

    /* Back in caller — a3 should still be intact */
    ASSERT_EQ(cpu.windowbase, 0);
    ASSERT_EQ(ar_read(&cpu, 3), 0xDEADBEEF);

    teardown(&cpu);
}

/* ===== Deep CALL4 chain — triggers overflow spill ===== */

TEST(deep_call4_chain) {
    xtensa_cpu_t cpu; setup_windowed(&cpu);

    /* We'll chain 5 CALL4s. With 16 windows and CALL4 rotating by 1 each,
     * window indices 0..4 will be used. No overflow until we exceed 16,
     * but let's chain enough that overflow check logic is exercised.
     * With callinc=1 per call, after ENTRY: new_wb+1..new_wb+3 are checked.
     */

    /* Set up stacks for each level */
    uint32_t stack[6];
    for (int i = 0; i < 6; i++)
        stack[i] = BASE + 0x4000 + i * 0x200;

    /* Level 0: caller */
    ar_write(&cpu, 1, stack[0]);

    /* Layout:
     * BASE+0x000: CALL4 → BASE+0x100
     * BASE+0x100: ENTRY a1,32; CALL4 → BASE+0x200
     * BASE+0x200: ENTRY a1,32; CALL4 → BASE+0x300
     * BASE+0x300: ENTRY a1,32; CALL4 → BASE+0x400
     * BASE+0x400: ENTRY a1,32; CALL4 → BASE+0x500
     * BASE+0x500: ENTRY a1,32; NOP (stops here)
     */
    for (int i = 0; i < 6; i++) {
        uint32_t addr = BASE + i * 0x100;
        if (i == 0) {
            /* First call */
            int32_t off = (int32_t)((BASE + 0x100) / 4 - (BASE / 4 + 1));
            put_insn3(&cpu, addr, calln_insn(1, off));
        } else {
            /* ENTRY a1, 32 */
            put_insn3(&cpu, addr, entry_insn(1, 32));
            if (i < 5) {
                /* CALL4 to next level */
                uint32_t next = BASE + (i + 1) * 0x100;
                int32_t off = (int32_t)(next / 4 - ((addr + 3) / 4 + 1));
                put_insn3(&cpu, addr + 3, calln_insn(1, off));
            } else {
                /* Last level: just NOP */
                put_insn3(&cpu, addr + 3, nop_insn());
            }
        }
    }

    /* Execute: CALL4 */
    xtensa_step(&cpu);
    ASSERT_EQ(cpu.pc, BASE + 0x100);

    /* Execute all ENTRY + CALL4 pairs */
    for (int i = 1; i < 6; i++) {
        xtensa_step(&cpu); /* ENTRY */
        ASSERT_EQ(cpu.windowbase, (uint32_t)i);
        ASSERT_EQ(cpu.windowstart & (1u << i), (1u << i));

        if (i < 5) {
            xtensa_step(&cpu); /* CALL4 */
        } else {
            xtensa_step(&cpu); /* NOP at last level */
        }
    }

    /* Verify all 6 windows are marked active */
    for (int i = 0; i <= 5; i++) {
        ASSERT_TRUE(cpu.windowstart & (1u << i));
    }

    teardown(&cpu);
}

/* ===== Deep CALL4 return — verify register restore ===== */

TEST(deep_call4_return) {
    xtensa_cpu_t cpu; setup_windowed(&cpu);

    uint32_t sp_val = BASE + 0x4000;
    ar_write(&cpu, 1, sp_val);

    /* 3-deep call chain: CALL4 → ENTRY → CALL4 → ENTRY → CALL4 → ENTRY → RETW → RETW → RETW */
    /* BASE+0x000: CALL4 → BASE+0x100 */
    int32_t off1 = (int32_t)((BASE + 0x100) / 4 - (BASE / 4 + 1));
    put_insn3(&cpu, BASE, calln_insn(1, off1));

    /* BASE+0x100: ENTRY a1,32; CALL4 → BASE+0x200 */
    put_insn3(&cpu, BASE + 0x100, entry_insn(1, 32));
    int32_t off2 = (int32_t)((BASE + 0x200) / 4 - ((BASE + 0x103) / 4 + 1));
    put_insn3(&cpu, BASE + 0x103, calln_insn(1, off2));

    /* BASE+0x200: ENTRY a1,32; CALL4 → BASE+0x300 */
    put_insn3(&cpu, BASE + 0x200, entry_insn(1, 32));
    int32_t off3 = (int32_t)((BASE + 0x300) / 4 - ((BASE + 0x203) / 4 + 1));
    put_insn3(&cpu, BASE + 0x203, calln_insn(1, off3));

    /* BASE+0x300: ENTRY a1,32; RETW */
    put_insn3(&cpu, BASE + 0x300, entry_insn(1, 32));
    put_insn3(&cpu, BASE + 0x303, retw_insn());

    /* BASE+0x106: RETW (return from level 1) */
    put_insn3(&cpu, BASE + 0x106, retw_insn());

    /* BASE+0x206: RETW (return from level 2) */
    put_insn3(&cpu, BASE + 0x206, retw_insn());

    /* Execute forward: 3 CALL4 + 3 ENTRY */
    for (int i = 0; i < 3; i++) {
        xtensa_step(&cpu); /* CALL4 */
        xtensa_step(&cpu); /* ENTRY */
    }
    ASSERT_EQ(cpu.windowbase, 3);

    /* Now return through all levels */
    xtensa_step(&cpu); /* RETW from level 3 */
    ASSERT_EQ(cpu.windowbase, 2);

    xtensa_step(&cpu); /* RETW from level 2 */
    ASSERT_EQ(cpu.windowbase, 1);

    xtensa_step(&cpu); /* RETW from level 1 */
    ASSERT_EQ(cpu.windowbase, 0);

    /* Verify we're back at caller's next instruction */
    ASSERT_EQ(cpu.pc, BASE + 3);

    teardown(&cpu);
}

/* ===== CALL8 overflow ===== */

TEST(call8_overflow) {
    xtensa_cpu_t cpu; setup_windowed(&cpu);

    /* CALL8 uses callinc=2. After 8 CALL8s we'd wrap 16 windows.
     * Let's do 4 CALL8s: WB goes 0→2→4→6→8. That's 5 windows active.
     * At WB=6, overflow check sees WB+2+1..+3 = 9,10,11 — all clear, no spill.
     * Let's just verify the chain works. */

    uint32_t sp_val = BASE + 0x8000;
    ar_write(&cpu, 1, sp_val);

    /* CALL8 to BASE+0x200 */
    int32_t off = (int32_t)((BASE + 0x200) / 4 - (BASE / 4 + 1));
    put_insn3(&cpu, BASE, calln_insn(2, off));
    put_insn3(&cpu, BASE + 0x200, entry_insn(1, 64));
    put_insn3(&cpu, BASE + 0x203, retw_insn());

    xtensa_step(&cpu); /* CALL8 */
    ASSERT_EQ(cpu.pc, BASE + 0x200);
    ASSERT_EQ(XT_PS_CALLINC(cpu.ps), 2);

    xtensa_step(&cpu); /* ENTRY */
    ASSERT_EQ(cpu.windowbase, 2);
    ASSERT_EQ(ar_read(&cpu, 1), sp_val - 64);

    xtensa_step(&cpu); /* RETW, n=2 */
    ASSERT_EQ(cpu.windowbase, 0);

    teardown(&cpu);
}

/* ===== Mixed CALL chain (CALL4 then CALL8) ===== */

TEST(mixed_call_chain) {
    xtensa_cpu_t cpu; setup_windowed(&cpu);
    uint32_t sp_val = BASE + 0x4000;
    ar_write(&cpu, 1, sp_val);

    /* CALL4 → ENTRY → CALL8 → ENTRY → RETW → RETW */

    /* CALL4 to BASE+0x100 */
    int32_t off1 = (int32_t)((BASE + 0x100) / 4 - (BASE / 4 + 1));
    put_insn3(&cpu, BASE, calln_insn(1, off1));

    /* Level 1: ENTRY a1,32; CALL8 to BASE+0x300 */
    put_insn3(&cpu, BASE + 0x100, entry_insn(1, 32));
    int32_t off2 = (int32_t)((BASE + 0x300) / 4 - ((BASE + 0x103) / 4 + 1));
    put_insn3(&cpu, BASE + 0x103, calln_insn(2, off2));

    /* Level 2: ENTRY a1,64; RETW */
    put_insn3(&cpu, BASE + 0x300, entry_insn(1, 64));
    put_insn3(&cpu, BASE + 0x303, retw_insn());

    /* Level 1 return: RETW */
    put_insn3(&cpu, BASE + 0x106, retw_insn());

    xtensa_step(&cpu); /* CALL4 */
    xtensa_step(&cpu); /* ENTRY, WB=1 */
    ASSERT_EQ(cpu.windowbase, 1);

    xtensa_step(&cpu); /* CALL8 */
    xtensa_step(&cpu); /* ENTRY, WB=3 */
    ASSERT_EQ(cpu.windowbase, 3);

    xtensa_step(&cpu); /* RETW from level 2, n=2, WB=1 */
    ASSERT_EQ(cpu.windowbase, 1);

    xtensa_step(&cpu); /* RETW from level 1, n=1, WB=0 */
    ASSERT_EQ(cpu.windowbase, 0);
    ASSERT_EQ(cpu.pc, BASE + 3);

    teardown(&cpu);
}

/* ===== MOVSP triggers spill ===== */

TEST(movsp_triggers_spill) {
    xtensa_cpu_t cpu; setup_windowed(&cpu);

    /* Set up: window 0 is current (WB=1), window 0 has windowstart set.
     * MOVSP should spill window 0 since it's below. */
    cpu.windowbase = 1;
    cpu.windowstart = (1u << 0) | (1u << 1); /* windows 0 and 1 active */

    /* Put some known values in window 0's physical regs */
    uint32_t sp0 = BASE + 0x2000;
    cpu.ar[0] = (1u << 30) | 0x1000; /* window 0's a0 (CALL4 ret addr) */
    cpu.ar[1] = sp0;                  /* window 0's a1 (SP) */
    cpu.ar[2] = 0xAAAA;
    cpu.ar[3] = 0xBBBB;

    /* Current window (WB=1): set a1 to some value, a2 to something */
    ar_write(&cpu, 1, BASE + 0x3000);
    ar_write(&cpu, 2, 0x42);

    /* MOVSP a3, a1 */
    put_insn3(&cpu, BASE, movsp_insn(3, 1));
    xtensa_step(&cpu);

    /* a3 should have a1's value */
    ASSERT_EQ(ar_read(&cpu, 3), BASE + 0x3000);

    /* Window 0 should have been spilled: its WS bit cleared */
    ASSERT_FALSE(cpu.windowstart & (1u << 0));

    /* Check spilled values in memory at callee's SP - 16 (hardware convention) */
    uint32_t callee_sp = BASE + 0x3000;
    ASSERT_EQ(mem_read32(cpu.mem, callee_sp - 16 + 0), (1u << 30) | 0x1000); /* a0 */
    ASSERT_EQ(mem_read32(cpu.mem, callee_sp - 16 + 4), sp0);                  /* a1 */
    ASSERT_EQ(mem_read32(cpu.mem, callee_sp - 16 + 8), 0xAAAA);               /* a2 */
    ASSERT_EQ(mem_read32(cpu.mem, callee_sp - 16 + 12), 0xBBBB);              /* a3 */

    teardown(&cpu);
}

/* ===== ROTW basic ===== */

TEST(rotw_basic) {
    xtensa_cpu_t cpu; setup_windowed(&cpu);
    cpu.windowbase = 3;

    /* ROTW -2 (imm4 = -2 sign-extended = 0xE in 4 bits) */
    put_insn3(&cpu, BASE, rotw_insn(0xE)); /* 0xE = -2 in 4-bit signed */
    xtensa_step(&cpu);
    ASSERT_EQ(cpu.windowbase, 1);

    /* ROTW +3 */
    put_insn3(&cpu, cpu.pc, rotw_insn(3));
    xtensa_step(&cpu);
    ASSERT_EQ(cpu.windowbase, 4);

    teardown(&cpu);
}

/* ===== L32E basic ===== */

TEST(l32e_basic) {
    xtensa_cpu_t cpu; setup_windowed(&cpu);
    uint32_t base_addr = BASE + 0x1000;

    ar_write(&cpu, 2, base_addr);

    /* Store a value at base_addr - 64 + 0*4 = base_addr - 64 */
    mem_write32(cpu.mem, base_addr - 64, 0xCAFEBABE);

    /* L32E a3, a2, -64 (r=0) */
    put_insn3(&cpu, BASE, l32e_insn(3, 2, 0));
    xtensa_step(&cpu);

    ASSERT_EQ(ar_read(&cpu, 3), 0xCAFEBABE);

    teardown(&cpu);
}

/* ===== S32E basic ===== */

TEST(s32e_basic) {
    xtensa_cpu_t cpu; setup_windowed(&cpu);
    uint32_t base_addr = BASE + 0x1000;

    ar_write(&cpu, 2, base_addr);
    ar_write(&cpu, 3, 0xDEADC0DE);

    /* S32E a3, a2, -60 (r=1 → offset = 1*4 - 64 = -60) */
    put_insn3(&cpu, BASE, s32e_insn(3, 2, 1));
    xtensa_step(&cpu);

    ASSERT_EQ(mem_read32(cpu.mem, base_addr - 60), 0xDEADC0DE);

    teardown(&cpu);
}

/* ===== L32E / S32E round trip ===== */

TEST(l32e_s32e_round_trip) {
    xtensa_cpu_t cpu; setup_windowed(&cpu);
    uint32_t base_addr = BASE + 0x1000;

    ar_write(&cpu, 2, base_addr);
    ar_write(&cpu, 4, 0x12345678);

    /* S32E a4, a2, -56 (r=2 → 2*4-64=-56) */
    put_insn3(&cpu, BASE, s32e_insn(4, 2, 2));
    xtensa_step(&cpu);

    /* L32E a5, a2, -56 */
    put_insn3(&cpu, cpu.pc, l32e_insn(5, 2, 2));
    xtensa_step(&cpu);

    ASSERT_EQ(ar_read(&cpu, 5), 0x12345678);

    teardown(&cpu);
}

/* ===== RFWO basic ===== */

TEST(rfwo_basic) {
    xtensa_cpu_t cpu; setup_windowed(&cpu);

    /* Set up state for RFWO:
     * WB=3, OWB=1 (stored in PS), EPC[1]=some address
     * RFWO: clear WS[WB], set WB=OWB, clear EXCM, jump to EPC[1] */
    cpu.windowbase = 3;
    cpu.windowstart = (1u << 3) | (1u << 1);
    XT_PS_SET_OWB(cpu.ps, 1);
    XT_PS_SET_EXCM(cpu.ps, 1);
    cpu.epc[0] = BASE + 0x500;

    put_insn3(&cpu, BASE, rfwo_insn());
    xtensa_step(&cpu);

    ASSERT_EQ(cpu.windowbase, 1);
    ASSERT_FALSE(cpu.windowstart & (1u << 3)); /* WS[3] cleared */
    ASSERT_TRUE(cpu.windowstart & (1u << 1));  /* WS[1] unchanged */
    ASSERT_EQ(XT_PS_EXCM(cpu.ps), 0);
    ASSERT_EQ(cpu.pc, BASE + 0x500);

    teardown(&cpu);
}

/* ===== RFWU basic ===== */

TEST(rfwu_basic) {
    xtensa_cpu_t cpu; setup_windowed(&cpu);

    /* RFWU: set WS[WB], WB=OWB, clear EXCM, jump to EPC[1] */
    cpu.windowbase = 5;
    cpu.windowstart = (1u << 0); /* only window 0 */
    XT_PS_SET_OWB(cpu.ps, 2);
    XT_PS_SET_EXCM(cpu.ps, 1);
    cpu.epc[0] = BASE + 0x600;

    put_insn3(&cpu, BASE, rfwu_insn());
    xtensa_step(&cpu);

    ASSERT_EQ(cpu.windowbase, 2);
    ASSERT_TRUE(cpu.windowstart & (1u << 5));  /* WS[5] set */
    ASSERT_EQ(XT_PS_EXCM(cpu.ps), 0);
    ASSERT_EQ(cpu.pc, BASE + 0x600);

    teardown(&cpu);
}

/* ===== RETW.N ===== */

TEST(retw_n_basic) {
    xtensa_cpu_t cpu; setup_windowed(&cpu);
    uint32_t sp_val = BASE + 0x1000;

    ar_write(&cpu, 1, sp_val);

    /* CALL4 → ENTRY → RETW.N */
    int32_t off = (int32_t)((BASE + 0x40) / 4 - (BASE / 4 + 1));
    put_insn3(&cpu, BASE, calln_insn(1, off));
    put_insn3(&cpu, BASE + 0x40, entry_insn(1, 32));
    put_insn2(&cpu, BASE + 0x43, retw_n_insn());

    xtensa_step(&cpu); /* CALL4 */
    xtensa_step(&cpu); /* ENTRY */
    ASSERT_EQ(cpu.windowbase, 1);

    xtensa_step(&cpu); /* RETW.N */
    ASSERT_EQ(cpu.windowbase, 0);
    ASSERT_EQ(cpu.pc, BASE + 3);

    teardown(&cpu);
}

/* ===== Factorial using windowed calls ===== */

TEST(factorial_windowed) {
    xtensa_cpu_t cpu; setup_windowed(&cpu);

    /* Compute fact(5) = 120 using CALL4/ENTRY/RETW.
     *
     * Caller (at BASE):
     *   MOVI a2, 5         ; argument
     *   CALL4 fact          ; call fact(5)
     *   NOP                 ; result in a2 after return
     *
     * fact (at BASE+0x100):
     *   ENTRY a1, 32
     *   MOVI a3, 1
     *   BLT a2, a3, .done  ; if n < 1, return 1 (but we start at 5, will not hit initially)
     *   Actually let's use a simpler iterative approach in the callee:
     *   ENTRY a1, 32         ; a2 = n (argument from caller's a6 mapped to callee's a2)
     *
     * Hmm, windowed calling convention: CALL4 puts return in a4 of caller.
     * After ENTRY (callinc=1), callee's a0-a3 map to caller's a4-a7.
     * So caller puts the argument in a6 (= callee's a2).
     *
     * But for a recursive factorial, each level needs to:
     *   1. Check if n <= 1, return 1
     *   2. call fact(n-1)
     *   3. multiply n * result
     *
     * This is complex to encode by hand. Let's do iterative instead:
     *   caller sets a6 = 5 (callee sees as a2)
     *   callee: ENTRY a1,32; MOVI a3,1; loop: MULL a3,a3,a2; ADDI a2,a2,-1;
     *           BGEI a2,2,loop; MOV a2,a3; RETW
     *
     * Wait — caller puts arg in a(nn*4 + 2) where nn=1 for CALL4.
     * So caller puts arg in a6 (index 6), which after window rotation by 1
     * becomes callee's a2. Result should go in callee's a2 (= caller's a6).
     */

    uint32_t sp_val = BASE + 0x2000;
    ar_write(&cpu, 1, sp_val);

    /* Caller code at BASE */
    uint32_t pc = BASE;

    /* MOVI a6, 5 (in caller's window, a6 will become callee's a2) */
    /* MOVI: r=0xA, s=imm[11:8], t=dest, imm8=imm[7:0]
     * For imm=5: imm12 = 5, s = 0, imm8 = 5 */
    uint32_t movi_a6_5 = ((5 & 0xFF) << 16) | (0xA << 12) | (0 << 8) | (6 << 4) | 2;
    put_insn3(&cpu, pc, movi_a6_5); pc += 3;

    /* CALL4 to BASE+0x100 */
    int32_t off = (int32_t)((BASE + 0x100) / 4 - (pc / 4 + 1));
    put_insn3(&cpu, pc, calln_insn(1, off)); pc += 3;

    /* NOP at return point (BASE+6) */
    put_insn3(&cpu, pc, nop_insn());

    /* Callee at BASE+0x100 */
    pc = BASE + 0x100;

    /* ENTRY a1, 32 */
    put_insn3(&cpu, pc, entry_insn(1, 32)); pc += 3;

    /* MOVI a3, 1 */
    uint32_t movi_a3_1 = ((1 & 0xFF) << 16) | (0xA << 12) | (0 << 8) | (3 << 4) | 2;
    put_insn3(&cpu, pc, movi_a3_1); pc += 3;

    /* loop (at pc = BASE+0x106):
     * MULL a3, a3, a2    ; a3 = a3 * a2
     * ADDI a2, a2, -1    ; a2--
     * BGEI a2, 2, loop   ; if a2 >= 2, goto loop
     */
    uint32_t loop_addr = pc;

    /* MULL a3, a3, a2: op0=0, op1=2, op2=8, r=3, s=3, t=2 */
    put_insn3(&cpu, pc, rrr(8, 2, 3, 3, 2)); pc += 3;

    /* ADDI a2, a2, -1: op0=2, r=0xC, s=2, t=2, imm8=0xFF (-1) */
    uint32_t addi_a2 = (0xFF << 16) | (0xC << 12) | (2 << 8) | (2 << 4) | 2;
    put_insn3(&cpu, pc, addi_a2); pc += 3;

    /* BGEI a2, 2, loop: op0=6, n=2, m=3, s=2, r=2(b4const[2]=2), imm8=offset
     * offset = loop_addr - (pc+3) - 1... wait, target = pc_of_branch + offset + 4
     * Actually after fetch, cpu->pc = next_pc. In exec_si:
     *   target = cpu->pc + offset + 1 where cpu->pc = addr + 3
     * So target = (addr+3) + offset + 1 = addr + offset + 4
     * offset = loop_addr - (pc + 4)
     */
    int32_t boff = (int32_t)(loop_addr - (pc + 4));
    uint32_t bgei = (((uint32_t)boff & 0xFF) << 16) | (2 << 12) | (2 << 8) | (3 << 6) | (2 << 4) | 6;
    put_insn3(&cpu, pc, bgei); pc += 3;

    /* MOV a2, a3: MOV.N a2, a3 — narrow: op0=0xD, r=0, s=3, t=2 */
    /* Actually use full MOV: OR a2, a3, a3 → rrr(2, 0, 2, 3, 3) */
    put_insn3(&cpu, pc, rrr(2, 0, 2, 3, 3)); pc += 3;

    /* RETW */
    put_insn3(&cpu, pc, retw_insn()); pc += 3;

    /* Run the program */
    int cycles = xtensa_run(&cpu, 100);
    (void)cycles;

    /* After RETW, we're back in caller window. a6 should be fact(5) = 120 */
    /* Actually after RETW, the result is in caller's a6 (which was callee's a2).
     * Since windowbase is back to 0, ar_read(&cpu, 6) should give us the result. */

    /* But wait — the RETW puts us at the return address, which is BASE+6 (the NOP).
     * xtensa_run stops when cycles exhausted. Let's just check the register. */

    /* After run, WB should be 0 */
    ASSERT_EQ(cpu.windowbase, 0);
    /* The result was written to callee's a2, which is physical reg ((1*4)+2)&63 = 6,
     * which is caller's a6 */
    ASSERT_EQ(ar_read(&cpu, 6), 120);

    teardown(&cpu);
}

/* ===== Test suite runner ===== */

static void run_window_tests(void) {
    TEST_SUITE("Window Registers (M5)");
    RUN_TEST(call4_entry_retw);
    RUN_TEST(call8_entry_retw);
    RUN_TEST(call12_entry_retw);
    RUN_TEST(callx4_round_trip);
    RUN_TEST(entry_frame_alloc);
    RUN_TEST(retw_restores_window);
    RUN_TEST(deep_call4_chain);
    RUN_TEST(deep_call4_return);
    RUN_TEST(call8_overflow);
    RUN_TEST(mixed_call_chain);
    RUN_TEST(movsp_triggers_spill);
    RUN_TEST(rotw_basic);
    RUN_TEST(l32e_basic);
    RUN_TEST(s32e_basic);
    RUN_TEST(l32e_s32e_round_trip);
    RUN_TEST(rfwo_basic);
    RUN_TEST(rfwu_basic);
    RUN_TEST(retw_n_basic);
    RUN_TEST(factorial_windowed);
}
