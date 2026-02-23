#include "xtensa.h"
#include "memory.h"
#include <string.h>
#include <stdio.h>
#include <math.h>

void xtensa_cpu_init(xtensa_cpu_t *cpu) {
    memset(cpu, 0, sizeof(*cpu));
    for (int i = 0; i < 32; i++)
        cpu->int_level[i] = 1;
    /* ESP32 timer interrupt level defaults */
    cpu->int_level[6]  = 1;   /* CCOMPARE0 → level 1 */
    cpu->int_level[15] = 3;   /* CCOMPARE1 → level 3 */
    cpu->int_level[16] = 5;   /* CCOMPARE2 → level 5 */
}

void xtensa_cpu_reset(xtensa_cpu_t *cpu) {
    xtensa_cpu_init(cpu);

    /* ESP32 reset vector */
    cpu->pc = 0x40000400;

    /* PS: WOE=1, EXCM=1, INTLEVEL=15 */
    cpu->ps = (1 << 18)    /* WOE */
            | (1 << 4)     /* EXCM */
            | 0xF;         /* INTLEVEL = 15 */

    /* Window registers */
    cpu->windowbase = 0;
    cpu->windowstart = 1;   /* Window 0 is valid */

    /* SAR undefined, set to 0 */
    cpu->sar = 0;
    cpu->lcount = 0;
    cpu->ccount = 0;

    /* ESP32 defaults */
    cpu->vecbase = 0x40000000;
    cpu->prid = 0xCDCD;        /* PRO_CPU */
    cpu->cpenable = 0;
    cpu->atomctl = 0x28;
    cpu->configid0 = 0;
    cpu->configid1 = 0;

    cpu->running = true;
}

int xtensa_fetch(const xtensa_cpu_t *cpu, uint32_t addr, uint32_t *insn_out) {
    const uint8_t *ptr = mem_get_ptr(cpu->mem, addr);
    if (!ptr) return 0;
    int op0 = ptr[0] & 0xF;
    if (op0 >= 8) {
        *insn_out = ptr[0] | ((uint32_t)ptr[1] << 8);
        return 2;
    } else {
        *insn_out = ptr[0] | ((uint32_t)ptr[1] << 8) | ((uint32_t)ptr[2] << 16);
        return 3;
    }
}

/* ===== Special Register Access ===== */

uint32_t sr_read(const xtensa_cpu_t *cpu, int sr) {
    switch (sr) {
    case XT_SR_LBEG:        return cpu->lbeg;
    case XT_SR_LEND:        return cpu->lend;
    case XT_SR_LCOUNT:      return cpu->lcount;
    case XT_SR_SAR:         return cpu->sar;
    case XT_SR_BR:          return cpu->br;
    case XT_SR_LITBASE:     return cpu->litbase;
    case XT_SR_SCOMPARE1:   return cpu->scompare1;
    case XT_SR_ACCLO:       return cpu->acclo;
    case XT_SR_ACCHI:       return cpu->acchi;
    case XT_SR_MR0:         return cpu->mr[0];
    case XT_SR_MR1:         return cpu->mr[1];
    case XT_SR_MR2:         return cpu->mr[2];
    case XT_SR_MR3:         return cpu->mr[3];
    case XT_SR_WINDOWBASE:  return cpu->windowbase;
    case XT_SR_WINDOWSTART: return cpu->windowstart;
    case XT_SR_IBREAKENABLE:return cpu->ibreakenable;
    case XT_SR_MEMCTL:      return cpu->memctl;
    case XT_SR_ATOMCTL:     return cpu->atomctl;
    case XT_SR_IBREAKA0:    return cpu->ibreaka[0];
    case XT_SR_IBREAKA1:    return cpu->ibreaka[1];
    case XT_SR_DBREAKA0:    return cpu->dbreaka[0];
    case XT_SR_DBREAKA1:    return cpu->dbreaka[1];
    case XT_SR_DBREAKC0:    return cpu->dbreakc[0];
    case XT_SR_DBREAKC1:    return cpu->dbreakc[1];
    case XT_SR_CONFIGID0:   return cpu->configid0;
    case XT_SR_EPC1:        return cpu->epc[0];
    case XT_SR_EPC2:        return cpu->epc[1];
    case XT_SR_EPC3:        return cpu->epc[2];
    case XT_SR_EPC4:        return cpu->epc[3];
    case XT_SR_EPC5:        return cpu->epc[4];
    case XT_SR_EPC6:        return cpu->epc[5];
    case XT_SR_EPC7:        return cpu->epc[6];
    case XT_SR_DEPC:        return cpu->depc;
    case XT_SR_EPS2:        return cpu->eps[1];
    case XT_SR_EPS3:        return cpu->eps[2];
    case XT_SR_EPS4:        return cpu->eps[3];
    case XT_SR_EPS5:        return cpu->eps[4];
    case XT_SR_EPS6:        return cpu->eps[5];
    case XT_SR_EPS7:        return cpu->eps[6];
    case XT_SR_CONFIGID1:   return cpu->configid1;
    case XT_SR_EXCSAVE1:    return cpu->excsave[0];
    case XT_SR_EXCSAVE2:    return cpu->excsave[1];
    case XT_SR_EXCSAVE3:    return cpu->excsave[2];
    case XT_SR_EXCSAVE4:    return cpu->excsave[3];
    case XT_SR_EXCSAVE5:    return cpu->excsave[4];
    case XT_SR_EXCSAVE6:    return cpu->excsave[5];
    case XT_SR_EXCSAVE7:    return cpu->excsave[6];
    case XT_SR_CPENABLE:    return cpu->cpenable;
    case XT_SR_INTSET:   return cpu->interrupt;
    case XT_SR_INTENABLE:   return cpu->intenable;
    case XT_SR_PS:          return cpu->ps;
    case XT_SR_VECBASE:     return cpu->vecbase;
    case XT_SR_EXCCAUSE:    return cpu->exccause;
    case XT_SR_DEBUGCAUSE:  return cpu->debugcause;
    case XT_SR_CCOUNT:      return cpu->ccount;
    case XT_SR_PRID:        return cpu->prid;
    case XT_SR_ICOUNT:      return cpu->icount;
    case XT_SR_ICOUNTLEVEL: return cpu->icountlevel;
    case XT_SR_EXCVADDR:    return cpu->excvaddr;
    case XT_SR_CCOMPARE0:   return cpu->ccompare[0];
    case XT_SR_CCOMPARE1:   return cpu->ccompare[1];
    case XT_SR_CCOMPARE2:   return cpu->ccompare[2];
    case XT_SR_MISC0:       return cpu->misc[0];
    case XT_SR_MISC1:       return cpu->misc[1];
    case XT_SR_MISC2:       return cpu->misc[2];
    case XT_SR_MISC3:       return cpu->misc[3];
    default:                return 0;
    }
}

void sr_write(xtensa_cpu_t *cpu, int sr, uint32_t val) {
    switch (sr) {
    case XT_SR_LBEG:        cpu->lbeg = val; break;
    case XT_SR_LEND:        cpu->lend = val; break;
    case XT_SR_LCOUNT:      cpu->lcount = val; break;
    case XT_SR_SAR:         cpu->sar = val & 0x3F; break;
    case XT_SR_BR:          cpu->br = val & 0xFFFF; break;
    case XT_SR_LITBASE:     cpu->litbase = val; break;
    case XT_SR_SCOMPARE1:   cpu->scompare1 = val; break;
    case XT_SR_ACCLO:       cpu->acclo = val; break;
    case XT_SR_ACCHI:       cpu->acchi = val & 0xFF; break;
    case XT_SR_MR0:         cpu->mr[0] = val; break;
    case XT_SR_MR1:         cpu->mr[1] = val; break;
    case XT_SR_MR2:         cpu->mr[2] = val; break;
    case XT_SR_MR3:         cpu->mr[3] = val; break;
    case XT_SR_WINDOWBASE:  cpu->windowbase = val & 0xF; break;
    case XT_SR_WINDOWSTART: cpu->windowstart = val & 0xFFFF; break;
    case XT_SR_IBREAKENABLE:cpu->ibreakenable = val; break;
    case XT_SR_MEMCTL:      cpu->memctl = val; break;
    case XT_SR_ATOMCTL:     cpu->atomctl = val; break;
    case XT_SR_IBREAKA0:    cpu->ibreaka[0] = val; break;
    case XT_SR_IBREAKA1:    cpu->ibreaka[1] = val; break;
    case XT_SR_DBREAKA0:    cpu->dbreaka[0] = val; break;
    case XT_SR_DBREAKA1:    cpu->dbreaka[1] = val; break;
    case XT_SR_DBREAKC0:    cpu->dbreakc[0] = val; break;
    case XT_SR_DBREAKC1:    cpu->dbreakc[1] = val; break;
    case XT_SR_EPC1:        cpu->epc[0] = val; break;
    case XT_SR_EPC2:        cpu->epc[1] = val; break;
    case XT_SR_EPC3:        cpu->epc[2] = val; break;
    case XT_SR_EPC4:        cpu->epc[3] = val; break;
    case XT_SR_EPC5:        cpu->epc[4] = val; break;
    case XT_SR_EPC6:        cpu->epc[5] = val; break;
    case XT_SR_EPC7:        cpu->epc[6] = val; break;
    case XT_SR_DEPC:        cpu->depc = val; break;
    case XT_SR_EPS2:        cpu->eps[1] = val; break;
    case XT_SR_EPS3:        cpu->eps[2] = val; break;
    case XT_SR_EPS4:        cpu->eps[3] = val; break;
    case XT_SR_EPS5:        cpu->eps[4] = val; break;
    case XT_SR_EPS6:        cpu->eps[5] = val; break;
    case XT_SR_EPS7:        cpu->eps[6] = val; break;
    case XT_SR_EXCSAVE1:    cpu->excsave[0] = val; break;
    case XT_SR_EXCSAVE2:    cpu->excsave[1] = val; break;
    case XT_SR_EXCSAVE3:    cpu->excsave[2] = val; break;
    case XT_SR_EXCSAVE4:    cpu->excsave[3] = val; break;
    case XT_SR_EXCSAVE5:    cpu->excsave[4] = val; break;
    case XT_SR_EXCSAVE6:    cpu->excsave[5] = val; break;
    case XT_SR_EXCSAVE7:    cpu->excsave[6] = val; break;
    case XT_SR_CPENABLE:    cpu->cpenable = val; break;
    case XT_SR_INTSET:   cpu->interrupt |= val; break;
    case XT_SR_INTCLEAR: cpu->interrupt &= ~val; break;
    case XT_SR_INTENABLE:   cpu->intenable = val; break;
    case XT_SR_PS:          cpu->ps = val; break;
    case XT_SR_VECBASE:     cpu->vecbase = val; break;
    case XT_SR_EXCCAUSE:    cpu->exccause = val; break;
    case XT_SR_DEBUGCAUSE:  cpu->debugcause = val; break;
    case XT_SR_CCOUNT:      cpu->ccount = val; break;
    case XT_SR_ICOUNT:      cpu->icount = val; break;
    case XT_SR_ICOUNTLEVEL: cpu->icountlevel = val; break;
    case XT_SR_EXCVADDR:    cpu->excvaddr = val; break;
    case XT_SR_CCOMPARE0:   cpu->ccompare[0] = val; cpu->interrupt &= ~(1u << 6); break;
    case XT_SR_CCOMPARE1:   cpu->ccompare[1] = val; cpu->interrupt &= ~(1u << 15); break;
    case XT_SR_CCOMPARE2:   cpu->ccompare[2] = val; cpu->interrupt &= ~(1u << 16); break;
    case XT_SR_MISC0:       cpu->misc[0] = val; break;
    case XT_SR_MISC1:       cpu->misc[1] = val; break;
    case XT_SR_MISC2:       cpu->misc[2] = val; break;
    case XT_SR_MISC3:       cpu->misc[3] = val; break;
    default: break; /* ignore writes to unknown/read-only SRs */
    }
}

/* ===== Exception/Interrupt Dispatch ===== */

void xtensa_raise_exception(xtensa_cpu_t *cpu, int cause, uint32_t fault_pc, uint32_t vaddr) {
    uint32_t vec;
    if (XT_PS_EXCM(cpu->ps)) {
        /* Double exception */
        cpu->depc = fault_pc;
        cpu->exccause = cause;
        cpu->excvaddr = vaddr;
        vec = cpu->vecbase + VECOFS_DOUBLE_EXC;
    } else {
        cpu->epc[0] = fault_pc;   /* EPC1 */
        cpu->exccause = cause;
        cpu->excvaddr = vaddr;
        /* Save UM state before setting EXCM */
        uint32_t user_mode = XT_PS_UM(cpu->ps);
        XT_PS_SET_EXCM(cpu->ps, 1);
        vec = cpu->vecbase + (user_mode ? VECOFS_USER_EXC : VECOFS_KERNEL_EXC);
    }
    if (!mem_get_ptr(cpu->mem, vec)) {
        cpu->exception = true;
        cpu->running = false;
        return;
    }
    cpu->pc = vec;
}

#define EXCMLEVEL 1  /* ESP32 config */

void xtensa_check_interrupts(xtensa_cpu_t *cpu) {
    uint32_t pending = cpu->interrupt & cpu->intenable;
    if (!pending) return;

    int eff_level = XT_PS_INTLEVEL(cpu->ps);
    if (XT_PS_EXCM(cpu->ps) && EXCMLEVEL > eff_level)
        eff_level = EXCMLEVEL;

    /* Find highest-level pending interrupt */
    int best_level = 0;
    for (int i = 0; i < 32; i++) {
        if (!(pending & (1u << i))) continue;
        int lvl = cpu->int_level[i];
        if (lvl > eff_level && lvl > best_level)
            best_level = lvl;
    }
    if (best_level == 0) return;

    if (best_level == 1) {
        /* Level-1: dispatched as exception */
        xtensa_raise_exception(cpu, EXCCAUSE_LEVEL1_INT, cpu->pc, 0);
    } else {
        /* High-priority (levels 2-7) */
        int idx = best_level - 1;
        cpu->epc[idx] = cpu->pc;
        cpu->eps[idx] = cpu->ps;
        XT_PS_SET_INTLEVEL(cpu->ps, best_level);
        XT_PS_SET_EXCM(cpu->ps, 1);

        static const uint32_t vecofs[] = {
            0, 0, VECOFS_LEVEL2_INT, VECOFS_LEVEL3_INT,
            VECOFS_LEVEL4_INT, VECOFS_LEVEL5_INT,
            VECOFS_DEBUG_EXC, VECOFS_NMI
        };
        uint32_t vec = cpu->vecbase + vecofs[best_level];
        if (!mem_get_ptr(cpu->mem, vec)) {
            cpu->exception = true;
            cpu->running = false;
            return;
        }
        cpu->pc = vec;
    }
}

/* ===== Instruction Execution ===== */

/* ===== Windowed Register Helpers ===== */

/*
 * Read a register from a specific physical window (not the current windowbase).
 */
static inline uint32_t phys_read(const xtensa_cpu_t *cpu, int widx, int reg) {
    return cpu->ar[((widx * 4) + reg) & 63];
}

/*
 * Write a register in a specific physical window.
 */
static inline void phys_write(xtensa_cpu_t *cpu, int widx, int reg, uint32_t val) {
    cpu->ar[((widx * 4) + reg) & 63] = val;
}

/*
 * Spill (save) registers of window widx to its stack frame.
 * The window's a0 encodes the call size in bits [31:30].
 * Always saves a0-a3 at SP-16; for CALL8+ also a4-a7 at SP-32;
 * for CALL12 also a8-a11 at SP-48.
 */
static void synth_spill_window(xtensa_cpu_t *cpu, int widx) {
    uint32_t sp = phys_read(cpu, widx, 1);  /* a1 = stack pointer */
    uint32_t a0 = phys_read(cpu, widx, 0);
    int callsize = (a0 >> 30) & 3;

    /* Always save base 4 regs: a0-a3 at SP-16 */
    for (int i = 0; i < 4; i++)
        mem_write32(cpu->mem, sp - 16 + i * 4, phys_read(cpu, widx, i));

    if (callsize >= 2) {
        /* CALL8+: also save a4-a7 at SP-32 */
        for (int i = 0; i < 4; i++)
            mem_write32(cpu->mem, sp - 32 + i * 4, phys_read(cpu, widx, 4 + i));
    }
    if (callsize == 3) {
        /* CALL12: also save a8-a11 at SP-48 */
        for (int i = 0; i < 4; i++)
            mem_write32(cpu->mem, sp - 48 + i * 4, phys_read(cpu, widx, 8 + i));
    }

    /* Clear windowstart bit for this window */
    cpu->windowstart &= ~(1u << (widx & 0xF));
}

/*
 * Overflow check: called during ENTRY to spill any live windows
 * that would be overwritten by the new callinc rotation.
 */
static void synth_overflow_check(xtensa_cpu_t *cpu, int callinc) {
    int new_wb = (cpu->windowbase + callinc) & 0xF;
    for (int i = 1; i <= 3; i++) {
        int w = (new_wb + i) & 0xF;
        if (cpu->windowstart & (1u << w))
            synth_spill_window(cpu, w);
    }
}

/*
 * Underflow fill: called during RETW when the caller's windowstart
 * bit is clear (registers were spilled and need restoration).
 */
static void synth_underflow_fill(xtensa_cpu_t *cpu, int ret_wb) {
    uint32_t sp = phys_read(cpu, ret_wb, 1);

    /* Restore a0-a3 from SP-16 */
    for (int i = 0; i < 4; i++)
        phys_write(cpu, ret_wb, i, mem_read32(cpu->mem, sp - 16 + i * 4));

    /* Check restored a0 for call size to determine extra regs */
    uint32_t a0 = phys_read(cpu, ret_wb, 0);
    int callsize = (a0 >> 30) & 3;

    if (callsize >= 2) {
        for (int i = 0; i < 4; i++)
            phys_write(cpu, ret_wb, 4 + i, mem_read32(cpu->mem, sp - 32 + i * 4));
    }
    if (callsize == 3) {
        for (int i = 0; i < 4; i++)
            phys_write(cpu, ret_wb, 8 + i, mem_read32(cpu->mem, sp - 48 + i * 4));
    }

    /* Set windowstart bit */
    cpu->windowstart |= (1u << (ret_wb & 0xF));
}

/*
 * RETW: shared helper for both RETW (24-bit) and RETW.N (16-bit).
 * ISA: n = AR[0][31:30], nextPC = PC[31:30] | AR[0][29:0]
 *   if WS[WB-n] set → normal: clear WS[owb], WB -= n
 *   if WS[WB-n] clear → underflow fill, then WB -= n
 */
static void exec_retw(xtensa_cpu_t *cpu) {
    uint32_t a0 = ar_read(cpu, 0);
    int n = (a0 >> 30) & 3;
    if (n == 0) n = 4;  /* n=0 encoding not used; safety fallback */

    uint32_t next_pc = (cpu->pc & 0xC0000000) | (a0 & 0x3FFFFFFF);

    int owb = cpu->windowbase;
    int ret_wb = (owb - n) & 0xF;

    if (!(cpu->windowstart & (1u << ret_wb))) {
        /* Caller's window was spilled — fill it back */
        synth_underflow_fill(cpu, ret_wb);
    }

    /* Clear current window's WS bit */
    cpu->windowstart &= ~(1u << owb);

    /* Rotate back */
    cpu->windowbase = ret_wb;

    cpu->pc = next_pc;
}

/* ===== Floating-Point Helpers ===== */

static inline uint32_t float_to_bits(float f) {
    uint32_t b; memcpy(&b, &f, 4); return b;
}
static inline float bits_to_float(uint32_t b) {
    float f; memcpy(&f, &b, 4); return f;
}

/* CONST.S lookup table (ISA Table 7-3, reciprocal estimation constants) */
static const uint32_t fp_const_table[16] = {
    0x00000000, /* 0: +0.0 */
    0x3F800000, /* 1: 1.0 */
    0x40000000, /* 2: 2.0 */
    0x3F000000, /* 3: 0.5 */
    0x00000000, /* 4: +0.0 (reserved) */
    0x00000000, /* 5: +0.0 (reserved) */
    0x00000000, /* 6: +0.0 (reserved) */
    0x00000000, /* 7: +0.0 (reserved) */
    0x00000000, /* 8: +0.0 (reserved) */
    0x00000000, /* 9: +0.0 (reserved) */
    0x00000000, /* 10: +0.0 (reserved) */
    0x00000000, /* 11: +0.0 (reserved) */
    0x00000000, /* 12: +0.0 (reserved) */
    0x00000000, /* 13: +0.0 (reserved) */
    0x00000000, /* 14: +0.0 (reserved) */
    0x00000000, /* 15: +0.0 (reserved) */
};

/* Execute FP0 (op0=0, op1=10): arithmetic, conversions, FP1OP */
static void exec_fp0(xtensa_cpu_t *cpu, uint32_t insn) {
    int op2 = XT_OP2(insn);
    int r = XT_R(insn);
    int s = XT_S(insn);
    int t = XT_T(insn);

    switch (op2) {
    case 0: /* ADD.S */
        cpu->fr[r] = cpu->fr[s] + cpu->fr[t];
        break;
    case 1: /* SUB.S */
        cpu->fr[r] = cpu->fr[s] - cpu->fr[t];
        break;
    case 2: /* MUL.S */
        cpu->fr[r] = cpu->fr[s] * cpu->fr[t];
        break;
    case 4: /* MADD.S */
        cpu->fr[r] = cpu->fr[r] + (cpu->fr[s] * cpu->fr[t]);
        break;
    case 5: /* MSUB.S */
        cpu->fr[r] = cpu->fr[r] - (cpu->fr[s] * cpu->fr[t]);
        break;
    case 6: /* MADDN.S (same as MSUB.S, forces round-to-nearest) */
        cpu->fr[r] = cpu->fr[r] - (cpu->fr[s] * cpu->fr[t]);
        break;
    case 7: { /* DIVN.S: fr[r] = fr[s] * fr[t] */
        cpu->fr[r] = cpu->fr[s] * cpu->fr[t];
    } break;
    case 8: { /* ROUND.S: ar[t] = (int32_t)roundf(fr[s] * 2^r) */
        float val = cpu->fr[s];
        if (r) val = val * (float)(1u << r);
        ar_write(cpu, t, (uint32_t)(int32_t)roundf(val));
    } break;
    case 9: { /* TRUNC.S: ar[t] = (int32_t)truncf(fr[s] * 2^r) */
        float val = cpu->fr[s];
        if (r) val = val * (float)(1u << r);
        ar_write(cpu, t, (uint32_t)(int32_t)truncf(val));
    } break;
    case 10: { /* FLOOR.S: ar[t] = (int32_t)floorf(fr[s] * 2^r) */
        float val = cpu->fr[s];
        if (r) val = val * (float)(1u << r);
        ar_write(cpu, t, (uint32_t)(int32_t)floorf(val));
    } break;
    case 11: { /* CEIL.S: ar[t] = (int32_t)ceilf(fr[s] * 2^r) */
        float val = cpu->fr[s];
        if (r) val = val * (float)(1u << r);
        ar_write(cpu, t, (uint32_t)(int32_t)ceilf(val));
    } break;
    case 12: { /* FLOAT.S: fr[r] = (float)(int32_t)ar[s] * 2^(-t) */
        float val = (float)(int32_t)ar_read(cpu, s);
        if (t) val = val / (float)(1u << t);
        cpu->fr[r] = val;
    } break;
    case 13: { /* UFLOAT.S: fr[r] = (float)(uint32_t)ar[s] * 2^(-t) */
        float val = (float)ar_read(cpu, s);
        if (t) val = val / (float)(1u << t);
        cpu->fr[r] = val;
    } break;
    case 14: { /* UTRUNC.S: ar[t] = (uint32_t)(fr[s] * 2^r) */
        float val = cpu->fr[s];
        if (r) val = val * (float)(1u << r);
        ar_write(cpu, t, (uint32_t)val);
    } break;
    case 15: /* FP1OP: sub-dispatch on t */
        switch (t) {
        case 0: /* MOV.S */
            cpu->fr[r] = cpu->fr[s];
            break;
        case 1: /* ABS.S */
            cpu->fr[r] = fabsf(cpu->fr[s]);
            break;
        case 3: /* CONST.S */
            cpu->fr[r] = bits_to_float(fp_const_table[s]);
            break;
        case 4: { /* RFR: ar[r] = fr[s] as bits */
            ar_write(cpu, r, float_to_bits(cpu->fr[s]));
        } break;
        case 5: /* WFR: fr[r] = ar[s] as bits */
            cpu->fr[r] = bits_to_float(ar_read(cpu, s));
            break;
        case 6: /* NEG.S */
            cpu->fr[r] = -cpu->fr[s];
            break;
        case 7: { /* DIV0.S: initial reciprocal approx */
            /* Produce approximate 1/fr[s] using host division */
            float fs = cpu->fr[s];
            if (fs == 0.0f)
                cpu->fr[r] = bits_to_float(0x7F800000); /* +inf */
            else
                cpu->fr[r] = 1.0f / fs;
        } break;
        case 8: { /* RECIP0.S: reciprocal initial approximation */
            float fs = cpu->fr[s];
            if (fs == 0.0f)
                cpu->fr[r] = bits_to_float(0x7F800000);
            else
                cpu->fr[r] = 1.0f / fs;
        } break;
        case 9: { /* SQRT0.S: square root initial approximation */
            float fs = cpu->fr[s];
            cpu->fr[r] = sqrtf(fs);
        } break;
        case 10: { /* RSQRT0.S: reciprocal square root initial */
            float fs = cpu->fr[s];
            if (fs <= 0.0f)
                cpu->fr[r] = bits_to_float(0x7F800000);
            else
                cpu->fr[r] = 1.0f / sqrtf(fs);
        } break;
        case 11: { /* NEXP01.S: force exponent to 127 (range [1.0, 2.0)) */
            uint32_t bits = float_to_bits(cpu->fr[s]);
            bits = (bits & 0x807FFFFFu) | (127u << 23);
            cpu->fr[r] = bits_to_float(bits);
        } break;
        case 12: { /* MKSADJ.S: make sqrt exponent adjustment */
            uint32_t bits = float_to_bits(cpu->fr[s]);
            int exp = (int)((bits >> 23) & 0xFF);
            int adj = 253 - exp; /* for sqrt: (253 - exp) */
            if ((exp & 1) == 0) adj--; /* even exponent */
            uint32_t result = ((uint32_t)(adj & 0xFF) << 23);
            if (bits & 0x80000000u) result |= 0x80000000u;
            cpu->fr[r] = bits_to_float(result);
        } break;
        case 13: { /* MKDADJ.S: make div exponent adjustment */
            uint32_t bits = float_to_bits(cpu->fr[s]);
            int exp = (int)((bits >> 23) & 0xFF);
            int adj = 253 - exp;
            uint32_t result = ((uint32_t)(adj & 0xFF) << 23);
            if (bits & 0x80000000u) result |= 0x80000000u;
            cpu->fr[r] = bits_to_float(result);
        } break;
        case 14: { /* ADDEXP.S: add exponent of fr[s] to fr[r], XOR signs */
            uint32_t rbits = float_to_bits(cpu->fr[r]);
            uint32_t sbits = float_to_bits(cpu->fr[s]);
            int rexp = (int)((rbits >> 23) & 0xFF);
            int sexp = (int)((sbits >> 23) & 0xFF);
            int newexp = rexp + sexp - 127;
            if (newexp < 0) newexp = 0;
            if (newexp > 255) newexp = 255;
            rbits = (rbits & 0x807FFFFFu) | ((uint32_t)(newexp & 0xFF) << 23);
            rbits ^= (sbits & 0x80000000u);
            cpu->fr[r] = bits_to_float(rbits);
        } break;
        case 15: { /* ADDEXPM.S: add exponent from mantissa bits of fr[s] */
            uint32_t rbits = float_to_bits(cpu->fr[r]);
            uint32_t sbits = float_to_bits(cpu->fr[s]);
            int rexp = (int)((rbits >> 23) & 0xFF);
            int mexp = (int)((sbits >> 14) & 0xFF);
            int newexp = rexp + mexp - 127;
            if (newexp < 0) newexp = 0;
            if (newexp > 255) newexp = 255;
            rbits = (rbits & 0x807FFFFFu) | ((uint32_t)(newexp & 0xFF) << 23);
            rbits ^= ((sbits & (1u << 22)) << 9); /* XOR sign with mantissa bit 22 */
            cpu->fr[r] = bits_to_float(rbits);
        } break;
        default: break;
        }
        break;
    default: break;
    }
}

/* Execute FP1 (op0=0, op1=11): comparisons, conditional FP moves */
static void exec_fp1(xtensa_cpu_t *cpu, uint32_t insn) {
    int op2 = XT_OP2(insn);
    int r = XT_R(insn);
    int s = XT_S(insn);
    int t = XT_T(insn);

    switch (op2) {
    case 1: { /* UN.S */
        int result = isnan(cpu->fr[s]) || isnan(cpu->fr[t]);
        cpu->br = (cpu->br & ~(1u << r)) | ((uint32_t)(result != 0) << r);
    } break;
    case 2: { /* OEQ.S */
        int result = !isnan(cpu->fr[s]) && !isnan(cpu->fr[t]) && cpu->fr[s] == cpu->fr[t];
        cpu->br = (cpu->br & ~(1u << r)) | ((uint32_t)(result != 0) << r);
    } break;
    case 3: { /* UEQ.S */
        int result = isnan(cpu->fr[s]) || isnan(cpu->fr[t]) || cpu->fr[s] == cpu->fr[t];
        cpu->br = (cpu->br & ~(1u << r)) | ((uint32_t)(result != 0) << r);
    } break;
    case 4: { /* OLT.S */
        int result = !isnan(cpu->fr[s]) && !isnan(cpu->fr[t]) && cpu->fr[s] < cpu->fr[t];
        cpu->br = (cpu->br & ~(1u << r)) | ((uint32_t)(result != 0) << r);
    } break;
    case 5: { /* ULT.S */
        int result = isnan(cpu->fr[s]) || isnan(cpu->fr[t]) || cpu->fr[s] < cpu->fr[t];
        cpu->br = (cpu->br & ~(1u << r)) | ((uint32_t)(result != 0) << r);
    } break;
    case 6: { /* OLE.S */
        int result = !isnan(cpu->fr[s]) && !isnan(cpu->fr[t]) && cpu->fr[s] <= cpu->fr[t];
        cpu->br = (cpu->br & ~(1u << r)) | ((uint32_t)(result != 0) << r);
    } break;
    case 7: { /* ULE.S */
        int result = isnan(cpu->fr[s]) || isnan(cpu->fr[t]) || cpu->fr[s] <= cpu->fr[t];
        cpu->br = (cpu->br & ~(1u << r)) | ((uint32_t)(result != 0) << r);
    } break;
    case 8: /* MOVEQZ.S */
        if (ar_read(cpu, t) == 0) cpu->fr[r] = cpu->fr[s];
        break;
    case 9: /* MOVNEZ.S */
        if (ar_read(cpu, t) != 0) cpu->fr[r] = cpu->fr[s];
        break;
    case 10: /* MOVLTZ.S */
        if ((int32_t)ar_read(cpu, t) < 0) cpu->fr[r] = cpu->fr[s];
        break;
    case 11: /* MOVGEZ.S */
        if ((int32_t)ar_read(cpu, t) >= 0) cpu->fr[r] = cpu->fr[s];
        break;
    case 12: /* MOVF.S */
        if (!((cpu->br >> t) & 1)) cpu->fr[r] = cpu->fr[s];
        break;
    case 13: /* MOVT.S */
        if ((cpu->br >> t) & 1) cpu->fr[r] = cpu->fr[s];
        break;
    default: break;
    }
}

/* Execute op0=0 (QRST) - the main RRR instruction group */
static void exec_qrst(xtensa_cpu_t *cpu, uint32_t insn) {
    int op1 = XT_OP1(insn);
    int op2 = XT_OP2(insn);
    int r = XT_R(insn);
    int s = XT_S(insn);
    int t = XT_T(insn);

    switch (op1) {
    case 0: /* RST0 */
        switch (op2) {
        case 0: /* ST0: specials */
            switch (r) {
            case 0: /* SNM0 */
                if (s == 0 && t == 0) {
                    /* ILL */
                    xtensa_raise_exception(cpu, EXCCAUSE_ILLEGAL, cpu->pc - 3, 0);
                    return;
                } else {
                    int m = XT_M(insn);
                    int nn = XT_N(insn);
                    if (m == 2 && nn == 0) {
                        /* RET: pc = a0 */
                        cpu->pc = ar_read(cpu, 0);
                        return; /* skip default pc advance */
                    } else if (m == 2 && nn == 1) {
                        /* RETW: windowed return */
                        exec_retw(cpu);
                        return;
                    } else if (m == 2 && nn == 2) {
                        /* JX: pc = ar[s] */
                        cpu->pc = ar_read(cpu, s);
                        return;
                    } else if (m == 3) {
                        /* CALLX0/4/8/12 */
                        uint32_t target = ar_read(cpu, s);
                        if (nn > 0) {
                            XT_PS_SET_CALLINC(cpu->ps, nn);
                            ar_write(cpu, nn * 4, ((uint32_t)nn << 30) | (cpu->pc & 0x3FFFFFFF));
                        } else {
                            ar_write(cpu, 0, cpu->pc);
                        }
                        cpu->pc = target;
                        return;
                    }
                    /* BREAK, etc. */
                    if (m == 0 && nn != 0) {
                        /* Some other SNM0 encoding */
                    }
                }
                break;
            case 1: { /* MOVSP */
                for (int i = 1; i <= 3; i++) {
                    int w = (cpu->windowbase - i) & 15;
                    if (cpu->windowstart & (1u << w))
                        synth_spill_window(cpu, w);
                }
                ar_write(cpu, t, ar_read(cpu, s));
            } break;
            case 2: /* SYNC group */
                /* NOP, ISYNC, RSYNC, ESYNC, DSYNC, EXTW, MEMW, EXCW */
                /* All no-ops for emulation purposes */
                break;
            case 3: /* RFEI group */
                switch (t) {
                case 0: /* RFET: RFE, RFWO, RFWU */
                    switch (s) {
                    case 0: /* RFE */
                        XT_PS_SET_EXCM(cpu->ps, 0);
                        cpu->pc = cpu->epc[0];
                        return;
                    case 4: /* RFWO */
                        XT_PS_SET_EXCM(cpu->ps, 0);
                        cpu->windowstart &= ~(1u << cpu->windowbase);
                        cpu->windowbase = XT_PS_OWB(cpu->ps);
                        cpu->pc = cpu->epc[0];
                        return;
                    case 5: /* RFWU */
                        XT_PS_SET_EXCM(cpu->ps, 0);
                        cpu->windowstart |= (1u << cpu->windowbase);
                        cpu->windowbase = XT_PS_OWB(cpu->ps);
                        cpu->pc = cpu->epc[0];
                        return;
                    default: break;
                    }
                    break;
                case 1: /* RFI */
                    if (s >= 1 && s <= 7) {
                        cpu->ps = cpu->eps[s - 1];
                        cpu->pc = cpu->epc[s - 1];
                    }
                    return;
                default: break;
                }
                break;
            case 4: /* BREAK */
                cpu->debug_break = true;
                break;
            case 5: /* SYSCALL - raise exception */
                xtensa_raise_exception(cpu, EXCCAUSE_SYSCALL, cpu->pc - 3, 0);
                return;
            case 6: /* RSIL - read/set interrupt level */
                ar_write(cpu, t, cpu->ps);
                cpu->ps = (cpu->ps & ~0xF) | (s & 0xF);
                break;
            case 7: /* WAITI */
                XT_PS_SET_INTLEVEL(cpu->ps, s);
                cpu->halted = true;
                break;
            case 8: /* ANY4: bt = bs|bs+1|bs+2|bs+3 */
                { int val = (cpu->br >> s) & 0xF;
                  cpu->br = (cpu->br & ~(1u << t)) | ((val ? 1u : 0u) << t);
                } break;
            case 9: /* ALL4: bt = bs&bs+1&bs+2&bs+3 */
                { int val = (cpu->br >> s) & 0xF;
                  cpu->br = (cpu->br & ~(1u << t)) | (((val == 0xF) ? 1u : 0u) << t);
                } break;
            case 10: /* ANY8: bt = any of bs..bs+7 */
                { int val = (cpu->br >> s) & 0xFF;
                  cpu->br = (cpu->br & ~(1u << t)) | ((val ? 1u : 0u) << t);
                } break;
            case 11: /* ALL8: bt = all of bs..bs+7 */
                { int val = (cpu->br >> s) & 0xFF;
                  cpu->br = (cpu->br & ~(1u << t)) | (((val == 0xFF) ? 1u : 0u) << t);
                } break;
            default: break;
            }
            break;
        case 1: /* AND */
            ar_write(cpu, r, ar_read(cpu, s) & ar_read(cpu, t));
            break;
        case 2: /* OR */
            ar_write(cpu, r, ar_read(cpu, s) | ar_read(cpu, t));
            break;
        case 3: /* XOR */
            ar_write(cpu, r, ar_read(cpu, s) ^ ar_read(cpu, t));
            break;
        case 4: /* ST1: shift-amount setup */
            switch (r) {
            case 0: /* SSR: SAR = ar[s] & 31 */
                cpu->sar = ar_read(cpu, s) & 0x1F;
                break;
            case 1: /* SSL: SAR = 32 - (ar[s] & 31) */
                cpu->sar = 32 - (ar_read(cpu, s) & 0x1F);
                break;
            case 2: /* SSA8L: SAR = (ar[s] & 3) * 8 */
                cpu->sar = (ar_read(cpu, s) & 3) * 8;
                break;
            case 3: /* SSA8B: SAR = 32 - (ar[s] & 3) * 8 */
                cpu->sar = 32 - (ar_read(cpu, s) & 3) * 8;
                break;
            case 4: /* SSAI: SAR = immediate (s | (t<<4))&31 ... actually just the 5-bit field */
                /* SSAI: SAR = (s | ((t & 1) << 4)) */
                cpu->sar = (s | ((t & 1) << 4));
                break;
            case 8: /* ROTW - rotate window */
                /* Simplified: just adjust windowbase */
                cpu->windowbase = (cpu->windowbase + (int32_t)sign_extend(t, 4)) & 0xF;
                break;
            case 14: /* NSA: normalized shift amount */
                { uint32_t val = ar_read(cpu, s);
                  int n = 0;
                  if ((int32_t)val < 0) val = ~val;
                  if (val == 0) { n = 31; }
                  else { while (!(val & 0x80000000)) { val <<= 1; n++; } }
                  ar_write(cpu, t, (uint32_t)n);
                } break;
            case 15: /* NSAU: normalized shift amount unsigned */
                { uint32_t val = ar_read(cpu, s);
                  int n = 0;
                  if (val == 0) { n = 32; }
                  else { while (!(val & 0x80000000)) { val <<= 1; n++; } }
                  ar_write(cpu, t, (uint32_t)n);
                } break;
            case 6: /* RER: ar[t] = external_reg[ar[s]] */
                ar_write(cpu, t, 0);  /* stub: return 0 */
                break;
            case 7: /* WER: external_reg[ar[s]] = ar[t] */
                break; /* stub: ignore */
            default: break;
            }
            break;
        case 5: /* TLB ops - stub */
            break;
        case 6: /* RT0 */
            switch (s) {
            case 0: /* NEG */
                ar_write(cpu, r, (uint32_t)(-(int32_t)ar_read(cpu, t)));
                break;
            case 1: /* ABS */
                { int32_t val = (int32_t)ar_read(cpu, t);
                  ar_write(cpu, r, (uint32_t)(val < 0 ? -val : val));
                } break;
            default: break;
            }
            break;
        case 7: /* reserved */
            break;
        case 8:  /* ADD */
            ar_write(cpu, r, ar_read(cpu, s) + ar_read(cpu, t));
            break;
        case 9:  /* ADDX2 */
            ar_write(cpu, r, (ar_read(cpu, s) << 1) + ar_read(cpu, t));
            break;
        case 10: /* ADDX4 */
            ar_write(cpu, r, (ar_read(cpu, s) << 2) + ar_read(cpu, t));
            break;
        case 11: /* ADDX8 */
            ar_write(cpu, r, (ar_read(cpu, s) << 3) + ar_read(cpu, t));
            break;
        case 12: /* SUB */
            ar_write(cpu, r, ar_read(cpu, s) - ar_read(cpu, t));
            break;
        case 13: /* SUBX2 */
            ar_write(cpu, r, (ar_read(cpu, s) << 1) - ar_read(cpu, t));
            break;
        case 14: /* SUBX4 */
            ar_write(cpu, r, (ar_read(cpu, s) << 2) - ar_read(cpu, t));
            break;
        case 15: /* SUBX8 */
            ar_write(cpu, r, (ar_read(cpu, s) << 3) - ar_read(cpu, t));
            break;
        }
        break;

    case 1: /* RST1 */
        switch (op2) {
        case 0: case 1: /* SLLI */
            { int sa = ((op2 & 1) << 4) | t;
              ar_write(cpu, r, ar_read(cpu, s) << sa);
            } break;
        case 2: case 3: /* SRAI */
            { int sa = ((op2 & 1) << 4) | s;
              ar_write(cpu, r, (uint32_t)((int32_t)ar_read(cpu, t) >> sa));
            } break;
        case 4: /* SRLI */
            ar_write(cpu, r, ar_read(cpu, t) >> s);
            break;
        case 6: /* XSR */
            { int sr_num = XT_SR_NUM(insn);
              uint32_t tmp = ar_read(cpu, t);
              ar_write(cpu, t, sr_read(cpu, sr_num));
              sr_write(cpu, sr_num, tmp);
            } break;
        case 8: /* SRC - funnel shift */
            { uint32_t sa = cpu->sar & 0x1F;
              if (sa == 0)
                  ar_write(cpu, r, ar_read(cpu, t));
              else
                  ar_write(cpu, r, (ar_read(cpu, s) << (32 - sa)) | (ar_read(cpu, t) >> sa));
            } break;
        case 9: /* SRL */
            { uint32_t sa = cpu->sar & 0x1F;
              ar_write(cpu, r, ar_read(cpu, t) >> sa);
            } break;
        case 10: /* SLL */
            { uint32_t sa = cpu->sar & 0x1F;
              uint32_t shift = 32 - sa;
              ar_write(cpu, r, shift >= 32 ? 0 : (ar_read(cpu, s) << shift));
            } break;
        case 11: /* SRA */
            { uint32_t sa = cpu->sar & 0x1F;
              ar_write(cpu, r, (uint32_t)((int32_t)ar_read(cpu, t) >> sa));
            } break;
        case 12: /* MUL16U */
            ar_write(cpu, r, (ar_read(cpu, s) & 0xFFFF) * (ar_read(cpu, t) & 0xFFFF));
            break;
        case 13: /* MUL16S */
            { int32_t a = (int32_t)(int16_t)(ar_read(cpu, s) & 0xFFFF);
              int32_t b = (int32_t)(int16_t)(ar_read(cpu, t) & 0xFFFF);
              ar_write(cpu, r, (uint32_t)(a * b));
            } break;
        default: break;
        }
        break;

    case 2: /* RST2 */
        switch (op2) {
        case 0: /* ANDB: br[r] = br[s] AND br[t] */
            { int val = ((cpu->br >> s) & 1) & ((cpu->br >> t) & 1);
              cpu->br = (cpu->br & ~(1u << r)) | ((uint32_t)val << r);
            } break;
        case 1: /* ANDBC: br[r] = br[s] AND NOT br[t] */
            { int val = ((cpu->br >> s) & 1) & (~(cpu->br >> t) & 1);
              cpu->br = (cpu->br & ~(1u << r)) | ((uint32_t)val << r);
            } break;
        case 2: /* ORB: br[r] = br[s] OR br[t] */
            { int val = ((cpu->br >> s) & 1) | ((cpu->br >> t) & 1);
              cpu->br = (cpu->br & ~(1u << r)) | ((uint32_t)val << r);
            } break;
        case 3: /* ORBC: br[r] = br[s] OR NOT br[t] */
            { int val = ((cpu->br >> s) & 1) | (~(cpu->br >> t) & 1);
              cpu->br = (cpu->br & ~(1u << r)) | ((uint32_t)val << r);
            } break;
        case 4: /* XORB: br[r] = br[s] XOR br[t] */
            { int val = ((cpu->br >> s) & 1) ^ ((cpu->br >> t) & 1);
              cpu->br = (cpu->br & ~(1u << r)) | ((uint32_t)val << r);
            } break;
        case 6: /* SALT */
            ar_write(cpu, r, (int32_t)ar_read(cpu, s) < (int32_t)ar_read(cpu, t) ? 1 : 0);
            break;
        case 7: /* SALTU */
            ar_write(cpu, r, ar_read(cpu, s) < ar_read(cpu, t) ? 1 : 0);
            break;
        case 8: /* MULL */
            ar_write(cpu, r, ar_read(cpu, s) * ar_read(cpu, t));
            break;
        case 10: /* MULUH */
            { uint64_t res = (uint64_t)ar_read(cpu, s) * (uint64_t)ar_read(cpu, t);
              ar_write(cpu, r, (uint32_t)(res >> 32));
            } break;
        case 11: /* MULSH */
            { int64_t res = (int64_t)(int32_t)ar_read(cpu, s) * (int64_t)(int32_t)ar_read(cpu, t);
              ar_write(cpu, r, (uint32_t)((uint64_t)res >> 32));
            } break;
        case 12: /* QUOU */
            { uint32_t divisor = ar_read(cpu, t);
              if (divisor == 0) { xtensa_raise_exception(cpu, EXCCAUSE_DIVIDE_BY_ZERO, cpu->pc - 3, 0); return; }
              ar_write(cpu, r, ar_read(cpu, s) / divisor);
            } break;
        case 13: /* QUOS */
            { int32_t divisor = (int32_t)ar_read(cpu, t);
              if (divisor == 0) { xtensa_raise_exception(cpu, EXCCAUSE_DIVIDE_BY_ZERO, cpu->pc - 3, 0); return; }
              int32_t dividend = (int32_t)ar_read(cpu, s);
              /* Handle INT_MIN / -1 overflow */
              if (dividend == (int32_t)0x80000000 && divisor == -1)
                  ar_write(cpu, r, 0x80000000);
              else
                  ar_write(cpu, r, (uint32_t)(dividend / divisor));
            } break;
        case 14: /* REMU */
            { uint32_t divisor = ar_read(cpu, t);
              if (divisor == 0) { xtensa_raise_exception(cpu, EXCCAUSE_DIVIDE_BY_ZERO, cpu->pc - 3, 0); return; }
              ar_write(cpu, r, ar_read(cpu, s) % divisor);
            } break;
        case 15: /* REMS */
            { int32_t divisor = (int32_t)ar_read(cpu, t);
              if (divisor == 0) { xtensa_raise_exception(cpu, EXCCAUSE_DIVIDE_BY_ZERO, cpu->pc - 3, 0); return; }
              int32_t dividend = (int32_t)ar_read(cpu, s);
              if (dividend == (int32_t)0x80000000 && divisor == -1)
                  ar_write(cpu, r, 0);
              else
                  ar_write(cpu, r, (uint32_t)(dividend % divisor));
            } break;
        default: break;
        }
        break;

    case 3: /* RST3 */
        switch (op2) {
        case 0: /* RSR */
            ar_write(cpu, t, sr_read(cpu, XT_SR_NUM(insn)));
            break;
        case 1: /* WSR */
            sr_write(cpu, XT_SR_NUM(insn), ar_read(cpu, t));
            break;
        case 2: /* SEXT - sign extend from bit position (t+7) */
            { int bits = t + 8; /* 8..23 */
              int32_t val = sign_extend(ar_read(cpu, s), bits);
              ar_write(cpu, r, (uint32_t)val);
            } break;
        case 3: /* CLAMPS - clamp to signed range -(2^(t+7)) .. (2^(t+7)-1) */
            { int bits = t + 7; /* 7..22 */
              int32_t val = (int32_t)ar_read(cpu, s);
              int32_t hi = (1 << bits) - 1;
              int32_t lo = -(1 << bits);
              if (val > hi) val = hi;
              else if (val < lo) val = lo;
              ar_write(cpu, r, (uint32_t)val);
            } break;
        case 4: /* MIN */
            { int32_t a = (int32_t)ar_read(cpu, s);
              int32_t b = (int32_t)ar_read(cpu, t);
              ar_write(cpu, r, (uint32_t)(a < b ? a : b));
            } break;
        case 5: /* MAX */
            { int32_t a = (int32_t)ar_read(cpu, s);
              int32_t b = (int32_t)ar_read(cpu, t);
              ar_write(cpu, r, (uint32_t)(a > b ? a : b));
            } break;
        case 6: /* MINU */
            { uint32_t a = ar_read(cpu, s);
              uint32_t b = ar_read(cpu, t);
              ar_write(cpu, r, a < b ? a : b);
            } break;
        case 7: /* MAXU */
            { uint32_t a = ar_read(cpu, s);
              uint32_t b = ar_read(cpu, t);
              ar_write(cpu, r, a > b ? a : b);
            } break;
        case 8: /* MOVEQZ */
            if (ar_read(cpu, t) == 0)
                ar_write(cpu, r, ar_read(cpu, s));
            break;
        case 9: /* MOVNEZ */
            if (ar_read(cpu, t) != 0)
                ar_write(cpu, r, ar_read(cpu, s));
            break;
        case 10: /* MOVLTZ */
            if ((int32_t)ar_read(cpu, t) < 0)
                ar_write(cpu, r, ar_read(cpu, s));
            break;
        case 11: /* MOVGEZ */
            if ((int32_t)ar_read(cpu, t) >= 0)
                ar_write(cpu, r, ar_read(cpu, s));
            break;
        case 12: /* MOVF: if (!bt) ar[r] = ar[s] */
            if (!((cpu->br >> t) & 1))
                ar_write(cpu, r, ar_read(cpu, s));
            break;
        case 13: /* MOVT: if (bt) ar[r] = ar[s] */
            if ((cpu->br >> t) & 1)
                ar_write(cpu, r, ar_read(cpu, s));
            break;
        case 14: /* RUR */
            { int ur = (s << 4) | r;
              switch (ur) {
              case 232: ar_write(cpu, t, cpu->fcr); break;
              case 233: ar_write(cpu, t, cpu->fsr); break;
              default:  ar_write(cpu, t, 0); break;
              }
            } break;
        case 15: /* WUR */
            { int ur = (s << 4) | r;
              switch (ur) {
              case 232: cpu->fcr = ar_read(cpu, t); break;
              case 233: cpu->fsr = ar_read(cpu, t); break;
              default: break;
              }
            } break;
        default: break;
        }
        break;

    case 4: case 5: /* EXTUI */
        { int shift = s | ((op1 & 1) << 4);
          uint32_t mask = (1u << (op2 + 1)) - 1;
          ar_write(cpu, r, (ar_read(cpu, t) >> shift) & mask);
        } break;

    case 8: /* LSCX: indexed FP loads/stores */
        switch (op2) {
        case 0: { /* LSX: fr[r] = mem32[ar[s] + ar[t]] */
            uint32_t addr = ar_read(cpu, s) + ar_read(cpu, t);
            uint32_t tmp = mem_read32(cpu->mem, addr);
            memcpy(&cpu->fr[r], &tmp, 4);
        } break;
        case 1: { /* LSXP: fr[r] = mem32[ar[s]]; ar[s] += ar[t] */
            uint32_t base = ar_read(cpu, s);
            uint32_t tmp = mem_read32(cpu->mem, base);
            memcpy(&cpu->fr[r], &tmp, 4);
            ar_write(cpu, s, base + ar_read(cpu, t));
        } break;
        case 4: { /* SSX: mem32[ar[s] + ar[t]] = fr[r] */
            uint32_t addr = ar_read(cpu, s) + ar_read(cpu, t);
            uint32_t tmp; memcpy(&tmp, &cpu->fr[r], 4);
            mem_write32(cpu->mem, addr, tmp);
        } break;
        case 5: { /* SSXP: mem32[ar[s]] = fr[r]; ar[s] += ar[t] */
            uint32_t base = ar_read(cpu, s);
            uint32_t tmp; memcpy(&tmp, &cpu->fr[r], 4);
            mem_write32(cpu->mem, base, tmp);
            ar_write(cpu, s, base + ar_read(cpu, t));
        } break;
        default: break;
        }
        break;

    case 9: /* LSC4: L32E, S32E */
        switch (op2) {
        case 0: { /* L32E */
            uint32_t addr = ar_read(cpu, s) + (uint32_t)((int32_t)(r << 2) - 64);
            ar_write(cpu, t, mem_read32(cpu->mem, addr));
        } break;
        case 4: { /* S32E */
            uint32_t addr = ar_read(cpu, s) + (uint32_t)((int32_t)(r << 2) - 64);
            mem_write32(cpu->mem, addr, ar_read(cpu, t));
        } break;
        default: break;
        }
        break;

    case 10: /* FP0: FP arithmetic, conversions */
        exec_fp0(cpu, insn);
        break;

    case 11: /* FP1: FP comparisons, conditional moves */
        exec_fp1(cpu, insn);
        break;

    default:
        /* Unimplemented op1 groups */
        break;
    }
}

/* Execute op0=2 (LSAI) - loads, stores, and ALU immediates */
static void exec_lsai(xtensa_cpu_t *cpu, uint32_t insn) {
    int r = XT_R(insn);
    int s = XT_S(insn);
    int t = XT_T(insn);
    int imm8 = XT_IMM8(insn);

    switch (r) {
    case 0x0: /* L8UI */
        { uint32_t addr = ar_read(cpu, s) + (uint32_t)imm8;
          ar_write(cpu, t, mem_read8(cpu->mem, addr));
        } break;
    case 0x1: /* L16UI */
        { uint32_t addr = ar_read(cpu, s) + (uint32_t)(imm8 << 1);
          ar_write(cpu, t, mem_read16(cpu->mem, addr));
        } break;
    case 0x2: /* L32I */
        { uint32_t addr = ar_read(cpu, s) + (uint32_t)(imm8 << 2);
          ar_write(cpu, t, mem_read32(cpu->mem, addr));
        } break;
    case 0x4: /* S8I */
        { uint32_t addr = ar_read(cpu, s) + (uint32_t)imm8;
          mem_write8(cpu->mem, addr, (uint8_t)ar_read(cpu, t));
        } break;
    case 0x5: /* S16I */
        { uint32_t addr = ar_read(cpu, s) + (uint32_t)(imm8 << 1);
          mem_write16(cpu->mem, addr, (uint16_t)ar_read(cpu, t));
        } break;
    case 0x6: /* S32I */
        { uint32_t addr = ar_read(cpu, s) + (uint32_t)(imm8 << 2);
          mem_write32(cpu->mem, addr, ar_read(cpu, t));
        } break;
    case 0x7: /* CACHE ops (DPFR, DPFW, DHWB, etc.) - no-op */
        break;
    case 0x9: /* L16SI */
        { uint32_t addr = ar_read(cpu, s) + (uint32_t)(imm8 << 1);
          ar_write(cpu, t, (uint32_t)sign_extend(mem_read16(cpu->mem, addr), 16));
        } break;
    case 0xB: /* L32AI (acquire semantics = no-op in emulator) */
        { uint32_t addr = ar_read(cpu, s) + (uint32_t)(imm8 << 2);
          ar_write(cpu, t, mem_read32(cpu->mem, addr));
        } break;
    case 0xE: /* S32C1I (conditional store) */
        { uint32_t addr = ar_read(cpu, s) + (uint32_t)(imm8 << 2);
          uint32_t old = mem_read32(cpu->mem, addr);
          if (old == cpu->scompare1)
              mem_write32(cpu->mem, addr, ar_read(cpu, t));
          ar_write(cpu, t, old);
        } break;
    case 0xF: /* S32RI (release semantics = no-op in emulator) */
        { uint32_t addr = ar_read(cpu, s) + (uint32_t)(imm8 << 2);
          mem_write32(cpu->mem, addr, ar_read(cpu, t));
        } break;

    case 0xA: /* MOVI */
        { int32_t imm12 = sign_extend(((uint32_t)s << 8) | (uint32_t)imm8, 12);
          ar_write(cpu, t, (uint32_t)imm12);
        } break;

    case 0xC: /* ADDI */
        { int32_t simm8 = sign_extend(imm8, 8);
          ar_write(cpu, t, ar_read(cpu, s) + (uint32_t)simm8);
        } break;

    case 0xD: /* ADDMI */
        { int32_t simm8 = sign_extend(imm8, 8);
          ar_write(cpu, t, ar_read(cpu, s) + (uint32_t)(simm8 << 8));
        } break;

    default:
        break;
    }
}

/* Execute narrow (16-bit) instructions */
static void exec_narrow(xtensa_cpu_t *cpu, uint32_t insn) {
    int op0 = XT_OP0(insn);
    int t = XT_T(insn);
    int s = XT_S(insn);
    int r = XT_R(insn);

    switch (op0) {
    case 0x8: /* L32I.N */
        ar_write(cpu, t, mem_read32(cpu->mem, ar_read(cpu, s) + (uint32_t)(r << 2)));
        break;
    case 0x9: /* S32I.N */
        mem_write32(cpu->mem, ar_read(cpu, s) + (uint32_t)(r << 2), ar_read(cpu, t));
        break;

    case 0xA: /* ADD.N */
        ar_write(cpu, r, ar_read(cpu, s) + ar_read(cpu, t));
        break;

    case 0xB: /* ADDI.N */
        { int imm = (t == 0) ? -1 : t;
          ar_write(cpu, r, ar_read(cpu, s) + (uint32_t)(int32_t)imm);
        } break;

    case 0xC: /* ST2: MOVI.N / BEQZ.N / BNEZ.N */
        { int t_hi = (t >> 2) & 3;
          if (t_hi < 2) {
              /* MOVI.N */
              int imm7 = ((t & 7) << 4) | r;
              int32_t val = sign_extend(imm7, 7);
              ar_write(cpu, s, (uint32_t)val);
          } else if (t_hi == 2) {
              /* BEQZ.N */
              int imm6 = ((t & 3) << 4) | r;
              if (ar_read(cpu, s) == 0)
                  cpu->pc = cpu->pc + (uint32_t)imm6 + 2;
          } else {
              /* BNEZ.N */
              int imm6 = ((t & 3) << 4) | r;
              if (ar_read(cpu, s) != 0)
                  cpu->pc = cpu->pc + (uint32_t)imm6 + 2;
          }
        } break;

    case 0xD: /* ST3 */
        switch (r) {
        case 0: /* MOV.N */
            ar_write(cpu, t, ar_read(cpu, s));
            break;
        case 15: /* ST3 r=15 subgroup */
            switch (t) {
            case 0: /* RET.N */
                cpu->pc = ar_read(cpu, 0);
                return; /* skip default pc advance */
            case 1: /* RETW.N */
                exec_retw(cpu);
                return;
            case 2: /* BREAK.N */
                cpu->debug_break = true;
                break;
            case 3: /* NOP.N */
                break;
            case 6: /* ILL.N */
                xtensa_raise_exception(cpu, EXCCAUSE_ILLEGAL, cpu->pc - 2, 0);
                return;
            default: break;
            }
            break;
        default: break;
        }
        break;

    default: break;
    }
}

/* B4const / B4constu lookup tables for immediate branches */
static const int32_t b4const[16] = {
    -1, 1, 2, 3, 4, 5, 6, 7, 8, 10, 12, 16, 32, 64, 128, 256
};
static const uint32_t b4constu[16] = {
    32768, 65536, 2, 3, 4, 5, 6, 7, 8, 10, 12, 16, 32, 64, 128, 256
};

/* Execute op0=5 (CALLN) - PC-relative calls */
static void exec_calln(xtensa_cpu_t *cpu, uint32_t insn) {
    int nn = XT_N(insn);
    int32_t offset = sign_extend(XT_OFFSET18(insn), 18);
    /* target[31:2] = (original_pc[31:2] + offset + 1), target[1:0] = 00 */
    uint32_t original_pc = cpu->pc - 3;
    uint32_t target = (((original_pc >> 2) + (uint32_t)offset + 1) << 2);

    if (nn > 0) {
        /* Windowed call: PS.CALLINC = nn, AR[nn*4] = nn || nextPC[29:0] */
        XT_PS_SET_CALLINC(cpu->ps, nn);
        ar_write(cpu, nn * 4, ((uint32_t)nn << 30) | (cpu->pc & 0x3FFFFFFF));
    } else {
        /* CALL0: return address = next instruction */
        ar_write(cpu, 0, cpu->pc);
    }
    cpu->pc = target;
}

/* Execute op0=6 (SI) - J, BRI12, BRI8, LOOP, ENTRY */
static void exec_si(xtensa_cpu_t *cpu, uint32_t insn) {
    int nn = XT_N(insn);
    int m = XT_M(insn);
    int s = XT_S(insn);

    switch (nn) {
    case 0: /* J - unconditional jump */
        { int32_t offset = sign_extend(XT_OFFSET18(insn), 18);
          cpu->pc = cpu->pc + (uint32_t)offset + 1;
        } break;

    case 1: /* BZ - BRI12 zero-compare branches */
        { int32_t imm12 = sign_extend(XT_IMM12(insn), 12);
          uint32_t target = cpu->pc + (uint32_t)imm12 + 1;
          int32_t val = (int32_t)ar_read(cpu, s);
          switch (m) {
          case 0: if (val == 0) cpu->pc = target; break;  /* BEQZ */
          case 1: if (val != 0) cpu->pc = target; break;  /* BNEZ */
          case 2: if (val < 0)  cpu->pc = target; break;  /* BLTZ */
          case 3: if (val >= 0) cpu->pc = target; break;  /* BGEZ */
          }
        } break;

    case 2: /* BI0 - BRI8 immediate-compare branches */
        { int imm8 = XT_IMM8(insn);
          int r = XT_R(insn);
          int32_t offset8 = sign_extend(imm8, 8);
          uint32_t target = cpu->pc + (uint32_t)offset8 + 1;
          int32_t val = (int32_t)ar_read(cpu, s);
          switch (m) {
          case 0: if (val == b4const[r]) cpu->pc = target; break;  /* BEQI */
          case 1: if (val != b4const[r]) cpu->pc = target; break;  /* BNEI */
          case 2: if (val < b4const[r])  cpu->pc = target; break;  /* BLTI */
          case 3: if (val >= b4const[r]) cpu->pc = target; break;  /* BGEI */
          }
        } break;

    case 3: /* BI1 */
        { int imm8 = XT_IMM8(insn);
          int r = XT_R(insn);
          switch (m) {
          case 0: { /* ENTRY */
              int callinc = XT_PS_CALLINC(cpu->ps);
              uint32_t imm12 = XT_IMM12(insn);
              uint32_t frame_size = imm12 << 3;

              synth_overflow_check(cpu, callinc);

              /* AR[callinc*4 | s&3] = AR[s] - framesize (before rotation) */
              int new_reg = (callinc << 2) | (s & 3);
              ar_write(cpu, new_reg, ar_read(cpu, s) - frame_size);

              uint32_t owb = cpu->windowbase;
              cpu->windowbase = (owb + callinc) & 0xF;
              cpu->windowstart |= (1u << cpu->windowbase);
              XT_PS_SET_OWB(cpu->ps, owb);
              XT_PS_SET_CALLINC(cpu->ps, 0);
          } break;
          case 1: /* B1: BF, BT, LOOP, LOOPNEZ, LOOPGTZ */
              { int32_t offset8 = sign_extend(imm8, 8);
                uint32_t target = cpu->pc + (uint32_t)offset8 + 1;
                switch (r) {
                case 0: /* BF */
                    if (!(cpu->br & (1u << s)))
                        cpu->pc = target;
                    break;
                case 1: /* BT */
                    if (cpu->br & (1u << s))
                        cpu->pc = target;
                    break;
                case 8: /* LOOP */
                    cpu->lend = target;
                    cpu->lbeg = cpu->pc;
                    cpu->lcount = ar_read(cpu, s) - 1;
                    break;
                case 9: /* LOOPNEZ */
                    cpu->lend = target;
                    cpu->lbeg = cpu->pc;
                    if (ar_read(cpu, s) == 0) {
                        cpu->pc = target;
                    } else {
                        cpu->lcount = ar_read(cpu, s) - 1;
                    }
                    break;
                case 10: /* LOOPGTZ */
                    cpu->lend = target;
                    cpu->lbeg = cpu->pc;
                    if ((int32_t)ar_read(cpu, s) <= 0) {
                        cpu->pc = target;
                    } else {
                        cpu->lcount = ar_read(cpu, s) - 1;
                    }
                    break;
                default: break;
                }
              } break;
          case 2: /* BLTUI */
              { int32_t offset8 = sign_extend(imm8, 8);
                uint32_t target = cpu->pc + (uint32_t)offset8 + 1;
                if (ar_read(cpu, s) < b4constu[r])
                    cpu->pc = target;
              } break;
          case 3: /* BGEUI */
              { int32_t offset8 = sign_extend(imm8, 8);
                uint32_t target = cpu->pc + (uint32_t)offset8 + 1;
                if (ar_read(cpu, s) >= b4constu[r])
                    cpu->pc = target;
              } break;
          }
        } break;
    }
}

/* Execute op0=7 (B) - RRI8 conditional branches */
static void exec_b(xtensa_cpu_t *cpu, uint32_t insn) {
    int r = XT_R(insn);
    int s = XT_S(insn);
    int t = XT_T(insn);
    int imm8 = XT_IMM8(insn);
    int32_t offset = sign_extend(imm8, 8);
    uint32_t target = cpu->pc + (uint32_t)offset + 1;
    uint32_t vs = ar_read(cpu, s);
    uint32_t vt = ar_read(cpu, t);

    int taken = 0;
    switch (r) {
    case 0:  taken = (vs & vt) == 0; break;                       /* BNONE */
    case 1:  taken = vs == vt; break;                              /* BEQ */
    case 2:  taken = (int32_t)vs < (int32_t)vt; break;            /* BLT */
    case 3:  taken = vs < vt; break;                               /* BLTU */
    case 4:  taken = (~vs & vt) == 0; break;                       /* BALL */
    case 5:  taken = !(vs & (1u << (vt & 31))); break;             /* BBC */
    case 6: case 14: /* BBCI */
        { int bit = t | ((r & 1) << 4);
          taken = !(vs & (1u << bit));
        } break;
    case 7: case 15: /* BBSI */
        { int bit = t | ((r & 1) << 4);
          taken = (vs & (1u << bit)) != 0;
        } break;
    case 8:  taken = (vs & vt) != 0; break;                       /* BANY */
    case 9:  taken = vs != vt; break;                              /* BNE */
    case 10: taken = (int32_t)vs >= (int32_t)vt; break;           /* BGE */
    case 11: taken = vs >= vt; break;                              /* BGEU */
    case 12: taken = (~vs & vt) != 0; break;                       /* BNALL */
    case 13: taken = (vs & (1u << (vt & 31))) != 0; break;         /* BBS */
    }

    if (taken)
        cpu->pc = target;
}

/* ===== MAC16 Helpers ===== */

static inline int32_t mac16_half(uint32_t val, int hi) {
    return hi ? (int16_t)(val >> 16) : (int16_t)(val & 0xFFFF);
}

static inline int64_t mac16_get_acc(const xtensa_cpu_t *cpu) {
    return ((int64_t)(int8_t)cpu->acchi << 32) | (uint64_t)cpu->acclo;
}

static inline void mac16_set_acc(xtensa_cpu_t *cpu, int64_t val) {
    cpu->acclo = (uint32_t)val;
    cpu->acchi = (uint32_t)((val >> 32) & 0xFF);
}

static void exec_mac16(xtensa_cpu_t *cpu, uint32_t insn) {
    int op1 = XT_OP1(insn);
    int op2 = XT_OP2(insn);
    int r = XT_R(insn);
    int s = XT_S(insn);
    int t = XT_T(insn);

    /* LDDEC / LDINC: op2=4,5 with op1=0 */
    if (op2 == 4 && (op1 & 0xC) == 0) {
        /* LDDEC: mr[r/4] = mem32[as]; as -= 4 */
        uint32_t addr = ar_read(cpu, s);
        cpu->mr[r >> 2] = mem_read32(cpu->mem, addr);
        ar_write(cpu, s, addr - 4);
        return;
    }
    if (op2 == 5 && (op1 & 0xC) == 0) {
        /* LDINC: mr[r/4] = mem32[as]; as += 4 */
        uint32_t addr = ar_read(cpu, s);
        cpu->mr[r >> 2] = mem_read32(cpu->mem, addr);
        ar_write(cpu, s, addr + 4);
        return;
    }

    /* Get source registers based on op2[3:2] */
    uint32_t src1, src2;
    int reg_mode = (op2 >> 2) & 3;
    switch (reg_mode) {
    case 0: /* AA */ src1 = ar_read(cpu, s); src2 = ar_read(cpu, t); break;
    case 1: /* AD */ src1 = ar_read(cpu, s); src2 = cpu->mr[t >> 1]; break;
    case 2: /* DA */ src1 = cpu->mr[s >> 1]; src2 = ar_read(cpu, t); break;
    case 3: /* DD */ src1 = cpu->mr[s >> 1]; src2 = cpu->mr[t >> 1]; break;
    default: return;
    }

    /* Get half-select from op1[1:0] */
    int sel = op1 & 3;
    int32_t h1 = mac16_half(src1, sel >> 1);
    int32_t h2 = mac16_half(src2, sel & 1);

    /* Operation from op1[3:2] */
    int op = (op1 >> 2) & 3;
    int64_t acc = mac16_get_acc(cpu);
    int64_t product;

    if (op == 3) {
        /* UMUL: unsigned */
        product = (int64_t)((uint32_t)(uint16_t)h1 * (uint32_t)(uint16_t)h2);
        acc = product;
    } else {
        product = (int64_t)h1 * (int64_t)h2;
        switch (op) {
        case 0: acc = product; break;    /* MUL */
        case 1: acc += product; break;   /* MULA */
        case 2: acc -= product; break;   /* MULS */
        }
    }
    mac16_set_acc(cpu, acc);

    /* Combined load for op2=8-11: MULA.xx.yy.LDDEC/LDINC */
    if ((op2 & 0xC) == 8) {
        uint32_t addr = ar_read(cpu, s);
        cpu->mr[r >> 2] = mem_read32(cpu->mem, addr);
        if (op2 & 1)
            ar_write(cpu, s, addr + 4); /* LDINC */
        else
            ar_write(cpu, s, addr - 4); /* LDDEC */
    }
}

/* ===== Main step function ===== */

int xtensa_step(xtensa_cpu_t *cpu) {
    uint32_t insn;
    if (cpu->halted) {
        cpu->ccount++;
        cpu->cycle_count++;
        /* Timer check while halted */
        if (cpu->ccount == cpu->ccompare[0]) cpu->interrupt |= (1u << 6);
        if (cpu->ccount == cpu->ccompare[1]) cpu->interrupt |= (1u << 15);
        if (cpu->ccount == cpu->ccompare[2]) cpu->interrupt |= (1u << 16);
        uint32_t pending = cpu->interrupt & cpu->intenable;
        if (pending) {
            cpu->halted = false;
            xtensa_check_interrupts(cpu);
        }
        return cpu->exception ? -1 : 0;
    }

    /* PC hook: intercept execution at specific addresses (e.g. ROM stubs) */
    if (cpu->pc_hook && cpu->pc_hook(cpu, cpu->pc, cpu->pc_hook_ctx)) {
        cpu->ccount++;
        cpu->cycle_count++;
        if (cpu->ccount == cpu->ccompare[0]) cpu->interrupt |= (1u << 6);
        if (cpu->ccount == cpu->ccompare[1]) cpu->interrupt |= (1u << 15);
        if (cpu->ccount == cpu->ccompare[2]) cpu->interrupt |= (1u << 16);
        xtensa_check_interrupts(cpu);
        return cpu->exception ? -1 : 0;
    }

    int ilen = xtensa_fetch(cpu, cpu->pc, &insn);
    if (ilen == 0) {
        xtensa_raise_exception(cpu, EXCCAUSE_IFETCH_ERROR, cpu->pc, 0);
        if (cpu->exception) return -1;
        return 0;
    }

    int op0 = XT_OP0(insn);
    uint32_t next_pc = cpu->pc + (uint32_t)ilen;

    if (ilen == 2) {
        cpu->pc = next_pc; /* set before exec so RET can override */
        exec_narrow(cpu, insn);
    } else {
        cpu->pc = next_pc;
        switch (op0) {
        case 0: exec_qrst(cpu, insn); break;
        case 1: /* L32R */
            { int lt = XT_T(insn);
              uint16_t imm16 = (uint16_t)XT_IMM16(insn);
              uint32_t target = (cpu->pc & ~3u) + (0xFFFC0000u | ((uint32_t)imm16 << 2));
              ar_write(cpu, lt, mem_read32(cpu->mem, target));
            } break;
        case 2: exec_lsai(cpu, insn); break;
        case 3: /* LSCI - FP loads/stores */
            { int lr = XT_R(insn);
              int ls = XT_S(insn);
              int lt = XT_T(insn);
              int limm8 = XT_IMM8(insn);
              uint32_t base = ar_read(cpu, ls);
              uint32_t offset = (uint32_t)(limm8 << 2);
              switch (lr) {
              case 0: /* LSI: ft = mem32[as + imm8*4] */
                  { uint32_t tmp = mem_read32(cpu->mem, base + offset);
                    memcpy(&cpu->fr[lt], &tmp, 4);
                  } break;
              case 4: /* SSI: mem32[as + imm8*4] = ft */
                  { uint32_t tmp; memcpy(&tmp, &cpu->fr[lt], 4);
                    mem_write32(cpu->mem, base + offset, tmp);
                  } break;
              case 8: /* LSIU: ft = mem32[as + imm8*4]; as += imm8*4 */
                  { uint32_t tmp = mem_read32(cpu->mem, base + offset);
                    memcpy(&cpu->fr[lt], &tmp, 4);
                    ar_write(cpu, ls, base + offset);
                  } break;
              case 12: /* SSIU: mem32[as + imm8*4] = ft; as += imm8*4 */
                  { uint32_t tmp; memcpy(&tmp, &cpu->fr[lt], 4);
                    mem_write32(cpu->mem, base + offset, tmp);
                    ar_write(cpu, ls, base + offset);
                  } break;
              default: break;
              }
            } break;
        case 4: exec_mac16(cpu, insn); break;
        case 5: exec_calln(cpu, insn); break;
        case 6: exec_si(cpu, insn); break;
        case 7: exec_b(cpu, insn); break;
        default: break;
        }
    }

    /* Zero-overhead loop: loop back when PC reaches LEND */
    if (cpu->lcount > 0 && cpu->pc == cpu->lend) {
        cpu->lcount--;
        cpu->pc = cpu->lbeg;
    }

    cpu->ccount++;
    cpu->cycle_count++;

    /* Timer interrupts */
    if (cpu->ccount == cpu->ccompare[0]) cpu->interrupt |= (1u << 6);
    if (cpu->ccount == cpu->ccompare[1]) cpu->interrupt |= (1u << 15);
    if (cpu->ccount == cpu->ccompare[2]) cpu->interrupt |= (1u << 16);

    /* Interrupt dispatch */
    xtensa_check_interrupts(cpu);

    return cpu->exception ? -1 : 0;
}

int xtensa_run(xtensa_cpu_t *cpu, int max_cycles) {
    int i;
    for (i = 0; i < max_cycles && cpu->running && !cpu->exception; i++) {
        if (xtensa_step(cpu) != 0)
            break;
    }
    return i;
}

/* xtensa_disasm() is in xtensa_disasm.c */
