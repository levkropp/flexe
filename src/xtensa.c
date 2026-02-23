#include "xtensa.h"
#include "memory.h"
#include <string.h>
#include <stdio.h>

void xtensa_cpu_init(xtensa_cpu_t *cpu) {
    memset(cpu, 0, sizeof(*cpu));
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
    case XT_SR_INTSET:   cpu->interrupt = val; break;
    case XT_SR_INTENABLE:   cpu->intenable = val; break;
    case XT_SR_PS:          cpu->ps = val; break;
    case XT_SR_VECBASE:     cpu->vecbase = val; break;
    case XT_SR_EXCCAUSE:    cpu->exccause = val; break;
    case XT_SR_DEBUGCAUSE:  cpu->debugcause = val; break;
    case XT_SR_CCOUNT:      cpu->ccount = val; break;
    case XT_SR_ICOUNT:      cpu->icount = val; break;
    case XT_SR_ICOUNTLEVEL: cpu->icountlevel = val; break;
    case XT_SR_EXCVADDR:    cpu->excvaddr = val; break;
    case XT_SR_CCOMPARE0:   cpu->ccompare[0] = val; break;
    case XT_SR_CCOMPARE1:   cpu->ccompare[1] = val; break;
    case XT_SR_CCOMPARE2:   cpu->ccompare[2] = val; break;
    case XT_SR_MISC0:       cpu->misc[0] = val; break;
    case XT_SR_MISC1:       cpu->misc[1] = val; break;
    case XT_SR_MISC2:       cpu->misc[2] = val; break;
    case XT_SR_MISC3:       cpu->misc[3] = val; break;
    default: break; /* ignore writes to unknown/read-only SRs */
    }
}

/* ===== Instruction Execution ===== */

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
                    cpu->exception = true;
                } else {
                    int m = XT_M(insn);
                    int nn = XT_N(insn);
                    if (m == 2 && nn == 0) {
                        /* RET: pc = a0 */
                        cpu->pc = ar_read(cpu, 0);
                        return; /* skip default pc advance */
                    } else if (m == 2 && nn == 2) {
                        /* JX: pc = ar[s] */
                        cpu->pc = ar_read(cpu, s);
                        return;
                    } else if (m == 3) {
                        /* CALLX0/4/8/12 */
                        uint32_t target = ar_read(cpu, s);
                        if (nn > 0) {
                            /* windowed call - simplified for now */
                            ar_write(cpu, nn * 4, (cpu->pc & 0xC0000000) | (((cpu->pc >> 2) + 3) << 0));
                            /* TODO: proper window rotation */
                        } else {
                            ar_write(cpu, 0, cpu->pc + 3);
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
            case 1: /* MOVSP */
                ar_write(cpu, t, ar_read(cpu, s));
                break;
            case 2: /* SYNC group */
                /* NOP, ISYNC, RSYNC, ESYNC, DSYNC, EXTW, MEMW, EXCW */
                /* All no-ops for emulation purposes */
                break;
            case 3: /* RFE and friends - return from exception */
                /* Stub: will implement in exception milestone */
                break;
            case 4: /* BREAK */
                cpu->debug_break = true;
                break;
            case 5: /* SYSCALL - raise exception */
                cpu->exception = true;
                break;
            case 6: /* RSIL - read/set interrupt level */
                ar_write(cpu, t, cpu->ps);
                cpu->ps = (cpu->ps & ~0xF) | (s & 0xF);
                break;
            case 7: /* WAITI - simplified: just a NOP */
                break;
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
              if (divisor == 0) { cpu->exception = true; break; }
              ar_write(cpu, r, ar_read(cpu, s) / divisor);
            } break;
        case 13: /* QUOS */
            { int32_t divisor = (int32_t)ar_read(cpu, t);
              if (divisor == 0) { cpu->exception = true; break; }
              int32_t dividend = (int32_t)ar_read(cpu, s);
              /* Handle INT_MIN / -1 overflow */
              if (dividend == (int32_t)0x80000000 && divisor == -1)
                  ar_write(cpu, r, 0x80000000);
              else
                  ar_write(cpu, r, (uint32_t)(dividend / divisor));
            } break;
        case 14: /* REMU */
            { uint32_t divisor = ar_read(cpu, t);
              if (divisor == 0) { cpu->exception = true; break; }
              ar_write(cpu, r, ar_read(cpu, s) % divisor);
            } break;
        case 15: /* REMS */
            { int32_t divisor = (int32_t)ar_read(cpu, t);
              if (divisor == 0) { cpu->exception = true; break; }
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
        case 14: /* RUR */
            ar_write(cpu, r, 0); /* stub */
            break;
        case 15: /* WUR */
            /* stub */
            break;
        default: break;
        }
        break;

    case 4: case 5: /* EXTUI */
        { int shift = s | ((op1 & 1) << 4);
          uint32_t mask = (1u << (op2 + 1)) - 1;
          ar_write(cpu, r, (ar_read(cpu, t) >> shift) & mask);
        } break;

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
          } else {
              /* BEQZ.N / BNEZ.N: stub for M4 (branches) */
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
            case 1: /* RETW.N - simplified: same as RET for now */
                cpu->pc = ar_read(cpu, 0);
                return;
            case 2: /* BREAK.N */
                cpu->debug_break = true;
                break;
            case 3: /* NOP.N */
                break;
            case 6: /* ILL.N */
                cpu->exception = true;
                break;
            default: break;
            }
            break;
        default: break;
        }
        break;

    default: break;
    }
}

/* ===== Main step function ===== */

int xtensa_step(xtensa_cpu_t *cpu) {
    uint32_t insn;
    int ilen = xtensa_fetch(cpu, cpu->pc, &insn);
    if (ilen == 0) {
        cpu->exception = true;
        return -1;
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
        default: break;
        }
    }

    cpu->ccount++;
    cpu->cycle_count++;
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
