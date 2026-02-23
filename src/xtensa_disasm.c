/*
 * Xtensa LX6 Disassembler
 * Complete instruction decode for all ESP32-relevant instructions.
 */
#include "xtensa.h"
#include "memory.h"
#include <stdio.h>
#include <string.h>

/* B4const / B4constu lookup tables for immediate branches */
static const int32_t b4const[16] = {
    -1, 1, 2, 3, 4, 5, 6, 7, 8, 10, 12, 16, 32, 64, 128, 256
};
static const uint32_t b4constu[16] = {
    32768, 65536, 2, 3, 4, 5, 6, 7, 8, 10, 12, 16, 32, 64, 128, 256
};

/* Special register name lookup */
static const char *sr_name(int sr) {
    switch (sr) {
    case 0:   return "lbeg";
    case 1:   return "lend";
    case 2:   return "lcount";
    case 3:   return "sar";
    case 4:   return "br";
    case 5:   return "litbase";
    case 12:  return "scompare1";
    case 16:  return "acclo";
    case 17:  return "acchi";
    case 32:  return "m0";
    case 33:  return "m1";
    case 34:  return "m2";
    case 35:  return "m3";
    case 72:  return "windowbase";
    case 73:  return "windowstart";
    case 83:  return "ptevaddr";
    case 90:  return "rasid";
    case 91:  return "itlbcfg";
    case 92:  return "dtlbcfg";
    case 96:  return "ibreakenable";
    case 97:  return "memctl";
    case 99:  return "atomctl";
    case 104: return "ddr";
    case 128: return "ibreaka0";
    case 129: return "ibreaka1";
    case 144: return "dbreaka0";
    case 145: return "dbreaka1";
    case 160: return "dbreakc0";
    case 161: return "dbreakc1";
    case 176: return "configid0";
    case 177: return "epc1";
    case 178: return "epc2";
    case 179: return "epc3";
    case 180: return "epc4";
    case 181: return "epc5";
    case 182: return "epc6";
    case 183: return "epc7";
    case 192: return "depc";
    case 194: return "eps2";
    case 195: return "eps3";
    case 196: return "eps4";
    case 197: return "eps5";
    case 198: return "eps6";
    case 199: return "eps7";
    case 208: return "configid1";
    case 209: return "excsave1";
    case 210: return "excsave2";
    case 211: return "excsave3";
    case 212: return "excsave4";
    case 213: return "excsave5";
    case 214: return "excsave6";
    case 215: return "excsave7";
    case 224: return "cpenable";
    case 226: return "intset";
    case 227: return "intclear";
    case 228: return "intenable";
    case 230: return "ps";
    case 231: return "vecbase";
    case 232: return "exccause";
    case 233: return "debugcause";
    case 234: return "ccount";
    case 235: return "prid";
    case 236: return "icount";
    case 237: return "icountlevel";
    case 238: return "excvaddr";
    case 240: return "ccompare0";
    case 241: return "ccompare1";
    case 242: return "ccompare2";
    case 244: return "misc0";
    case 245: return "misc1";
    case 246: return "misc2";
    case 247: return "misc3";
    default:  return NULL;
    }
}

/* Helper: append formatted string to buffer */
#define EMIT(fmt, ...) do { \
    n = snprintf(p, rem, fmt, ##__VA_ARGS__); \
    if (n > 0) { p += n; rem -= n; } \
} while(0)

/*
 * Disassemble op0=0 (QRST) - RRR and related formats
 * This is the most complex group with nested dispatch.
 */
static int disasm_qrst(uint32_t insn, uint32_t pc, char *buf, int bufsize) {
    char *p = buf;
    int rem = bufsize, n;
    int op1 = XT_OP1(insn);
    int op2 = XT_OP2(insn);
    int r = XT_R(insn);
    int s = XT_S(insn);
    int t = XT_T(insn);

    switch (op1) {
    case 0: /* RST0 */
        switch (op2) {
        case 0: /* ST0 */
            switch (r) {
            case 0: /* SNM0 */
                { int m = XT_M(insn);
                  int nn = XT_N(insn);
                  if (m == 0 && nn == 0) {
                      if (s == 0 && t == 0) EMIT("ill");
                      else EMIT("ill?");
                  } else if (m == 2) { /* JR */
                      switch (nn) {
                      case 0: EMIT("ret"); break;
                      case 1: EMIT("retw"); break;
                      case 2: EMIT("jx\ta%d", s); break;
                      default: EMIT("??jr.n=%d", nn); break;
                      }
                  } else if (m == 3) { /* CALLX */
                      switch (nn) {
                      case 0: EMIT("callx0\ta%d", s); break;
                      case 1: EMIT("callx4\ta%d", s); break;
                      case 2: EMIT("callx8\ta%d", s); break;
                      case 3: EMIT("callx12\ta%d", s); break;
                      }
                  } else {
                      EMIT("??snm0 m=%d n=%d", m, nn);
                  }
                } break;
            case 1: EMIT("movsp\ta%d, a%d", t, s); break;
            case 2: /* SYNC */
                switch (t) {
                case 0: /* SYNC0 */
                    switch (s) {
                    case 0: EMIT("isync"); break;
                    case 1: EMIT("rsync"); break;
                    case 2: EMIT("esync"); break;
                    case 3: EMIT("dsync"); break;
                    case 8: EMIT("excw"); break;
                    case 12: EMIT("memw"); break;
                    case 13: EMIT("extw"); break;
                    case 15: EMIT("nop"); break;
                    default: EMIT("??sync0 s=%d", s); break;
                    }
                    break;
                default: EMIT("??sync t=%d", t); break;
                }
                break;
            case 3: /* RFEI */
                switch (t) {
                case 0: /* RFET */
                    switch (s) {
                    case 0: EMIT("rfe"); break;
                    case 1: EMIT("rfue"); break;
                    case 2: EMIT("rfde"); break;
                    case 4: EMIT("rfwo"); break;
                    case 5: EMIT("rfwu"); break;
                    default: EMIT("??rfet s=%d", s); break;
                    }
                    break;
                case 1: /* RFI */
                    EMIT("rfi\t%d", s);
                    break;
                case 2: /* RFME/CLREX */
                    if (s == 0) EMIT("rfme");
                    else if (s == 1) EMIT("clrex");
                    else EMIT("??rfm s=%d", s);
                    break;
                default: EMIT("??rfei t=%d", t); break;
                }
                break;
            case 4: EMIT("break\t%d, %d", s, t); break;
            case 5: /* SYSCALL/SIMCALL/HALT */
                if (t == 0) {
                    if (s == 0) EMIT("syscall");
                    else if (s == 1) EMIT("simcall");
                    else EMIT("??sysim s=%d", s);
                } else {
                    EMIT("??st0.r5 t=%d", t);
                }
                break;
            case 6: EMIT("rsil\ta%d, %d", t, s); break;
            case 7: /* WAITI */
                if (t == 0) EMIT("waiti\t%d", s);
                else EMIT("??waiti t=%d", t);
                break;
            case 8: /* ANY4 */
                EMIT("any4\tb%d, b%d", t, s); break;
            case 9: /* ALL4 */
                EMIT("all4\tb%d, b%d", t, s); break;
            case 10: /* ANY8 */
                EMIT("any8\tb%d, b%d", t, s); break;
            case 11: /* ALL8 */
                EMIT("all8\tb%d, b%d", t, s); break;
            default: EMIT("??st0 r=%d", r); break;
            }
            break;
        case 1: EMIT("and\ta%d, a%d, a%d", r, s, t); break;
        case 2: EMIT("or\ta%d, a%d, a%d", r, s, t); break;
        case 3: EMIT("xor\ta%d, a%d, a%d", r, s, t); break;
        case 4: /* ST1 */
            switch (r) {
            case 0: /* SSR */
                if (t == 0) EMIT("ssr\ta%d", s);
                else EMIT("??ssr t=%d", t);
                break;
            case 1: /* SSL */
                if (t == 0) EMIT("ssl\ta%d", s);
                else EMIT("??ssl t=%d", t);
                break;
            case 2: /* SSA8L */
                if (t == 0) EMIT("ssa8l\ta%d", s);
                else EMIT("??ssa8l t=%d", t);
                break;
            case 3: /* SSA8B */
                if (t == 0) EMIT("ssa8b\ta%d", s);
                else EMIT("??ssa8b t=%d", t);
                break;
            case 4: /* SSAI */
                { int sa = (s | ((t & 1) << 4));
                  EMIT("ssai\t%d", sa);
                } break;
            case 6: /* RER */
                EMIT("rer\ta%d, a%d", t, s); break;
            case 7: /* WER */
                EMIT("wer\ta%d, a%d", t, s); break;
            case 8: /* ROTW */
                EMIT("rotw\t%d", sign_extend(t, 4)); break;
            case 14: /* NSA */
                EMIT("nsa\ta%d, a%d", t, s); break;
            case 15: /* NSAU */
                EMIT("nsau\ta%d, a%d", t, s); break;
            default: EMIT("??st1 r=%d", r); break;
            }
            break;
        case 5: /* TLB */
            switch (r) {
            case 3: EMIT("ritlb0\ta%d, a%d", t, s); break;
            case 4: EMIT("iitlb\ta%d", s); break;
            case 5: EMIT("pitlb\ta%d, a%d", t, s); break;
            case 6: EMIT("witlb\ta%d, a%d", t, s); break;
            case 7: EMIT("ritlb1\ta%d, a%d", t, s); break;
            case 11: EMIT("rdtlb0\ta%d, a%d", t, s); break;
            case 12: EMIT("idtlb\ta%d", s); break;
            case 13: EMIT("pdtlb\ta%d, a%d", t, s); break;
            case 14: EMIT("wdtlb\ta%d, a%d", t, s); break;
            case 15: EMIT("rdtlb1\ta%d, a%d", t, s); break;
            default: EMIT("??tlb r=%d", r); break;
            }
            break;
        case 6: /* RT0 - NEG, ABS */
            switch (s) {
            case 0: EMIT("neg\ta%d, a%d", r, t); break;
            case 1: EMIT("abs\ta%d, a%d", r, t); break;
            default: EMIT("??rt0 s=%d", s); break;
            }
            break;
        case 8: EMIT("add\ta%d, a%d, a%d", r, s, t); break;
        case 9: EMIT("addx2\ta%d, a%d, a%d", r, s, t); break;
        case 10: EMIT("addx4\ta%d, a%d, a%d", r, s, t); break;
        case 11: EMIT("addx8\ta%d, a%d, a%d", r, s, t); break;
        case 12: EMIT("sub\ta%d, a%d, a%d", r, s, t); break;
        case 13: EMIT("subx2\ta%d, a%d, a%d", r, s, t); break;
        case 14: EMIT("subx4\ta%d, a%d, a%d", r, s, t); break;
        case 15: EMIT("subx8\ta%d, a%d, a%d", r, s, t); break;
        default: EMIT("??rst0 op2=%d", op2); break;
        }
        break;

    case 1: /* RST1 */
        switch (op2) {
        case 0: case 1: /* SLLI */
            { /* Xtensa.pdf: op2=000sh[4], r=dest, s=src, t=sh[3..0] */
              int sa = ((op2 & 1) << 4) | t;
              EMIT("slli\ta%d, a%d, %d", r, s, sa);
            } break;
        case 2: case 3: /* SRAI */
            { /* Xtensa.pdf: op2=001sh[4], r=dest, s=sh[3..0], t=src */
              int sa = ((op2 & 1) << 4) | s;
              EMIT("srai\ta%d, a%d, %d", r, t, sa);
            } break;
        case 4: EMIT("srli\ta%d, a%d, %d", r, t, s); break;
        case 5: /* reserved */ EMIT("??rst1.5"); break;
        case 6: /* XSR */
            { int sr = XT_SR_NUM(insn);
              const char *name = sr_name(sr);
              if (name)
                  EMIT("xsr\ta%d, %s", t, name);
              else
                  EMIT("xsr\ta%d, %d", t, sr);
            } break;
        case 7: /* ACCER - RER/WER already in ST1. This is reserved or IMP */
            EMIT("??rst1.7"); break;
        case 8: EMIT("src\ta%d, a%d, a%d", r, s, t); break;
        case 9: /* SRL */
            if (s == 0) EMIT("srl\ta%d, a%d", r, t);
            else EMIT("??srl s=%d", s);
            break;
        case 10: /* SLL */
            if (t == 0) EMIT("sll\ta%d, a%d", r, s);
            else EMIT("??sll t=%d", t);
            break;
        case 11: /* SRA */
            if (s == 0) EMIT("sra\ta%d, a%d", r, t);
            else EMIT("??sra s=%d", s);
            break;
        case 12: EMIT("mul16u\ta%d, a%d, a%d", r, s, t); break;
        case 13: EMIT("mul16s\ta%d, a%d, a%d", r, s, t); break;
        case 15: /* IMP - implementation-specific */
            /* L32E, S32E are in LSCX (op1=2), not here */
            EMIT("??imp"); break;
        default: EMIT("??rst1 op2=%d", op2); break;
        }
        break;

    case 2: /* RST2 - Boolean, multiply, divide */
        switch (op2) {
        case 0: EMIT("andb\tb%d, b%d, b%d", r, s, t); break;
        case 1: EMIT("andbc\tb%d, b%d, b%d", r, s, t); break;
        case 2: EMIT("orb\tb%d, b%d, b%d", r, s, t); break;
        case 3: EMIT("orbc\tb%d, b%d, b%d", r, s, t); break;
        case 4: EMIT("xorb\tb%d, b%d, b%d", r, s, t); break;
        case 6: EMIT("salt\ta%d, a%d, a%d", r, s, t); break;
        case 7: EMIT("saltu\ta%d, a%d, a%d", r, s, t); break;
        case 8: EMIT("mull\ta%d, a%d, a%d", r, s, t); break;
        case 10: EMIT("muluh\ta%d, a%d, a%d", r, s, t); break;
        case 11: EMIT("mulsh\ta%d, a%d, a%d", r, s, t); break;
        case 12: EMIT("quou\ta%d, a%d, a%d", r, s, t); break;
        case 13: EMIT("quos\ta%d, a%d, a%d", r, s, t); break;
        case 14: EMIT("remu\ta%d, a%d, a%d", r, s, t); break;
        case 15: EMIT("rems\ta%d, a%d, a%d", r, s, t); break;
        default: EMIT("??rst2 op2=%d", op2); break;
        }
        break;

    case 3: /* RST3 - RSR, WSR, conditional moves, misc ops */
        switch (op2) {
        case 0: /* RSR */
            { int sr = XT_SR_NUM(insn);
              const char *name = sr_name(sr);
              if (name)
                  EMIT("rsr\ta%d, %s", t, name);
              else
                  EMIT("rsr\ta%d, %d", t, sr);
            } break;
        case 1: /* WSR */
            { int sr = XT_SR_NUM(insn);
              const char *name = sr_name(sr);
              if (name)
                  EMIT("wsr\ta%d, %s", t, name);
              else
                  EMIT("wsr\ta%d, %d", t, sr);
            } break;
        case 2: /* SEXT */
            EMIT("sext\ta%d, a%d, %d", r, s, t + 7); break;
        case 3: /* CLAMPS */
            EMIT("clamps\ta%d, a%d, %d", r, s, t + 7); break;
        case 4: EMIT("min\ta%d, a%d, a%d", r, s, t); break;
        case 5: EMIT("max\ta%d, a%d, a%d", r, s, t); break;
        case 6: EMIT("minu\ta%d, a%d, a%d", r, s, t); break;
        case 7: EMIT("maxu\ta%d, a%d, a%d", r, s, t); break;
        case 8: EMIT("moveqz\ta%d, a%d, a%d", r, s, t); break;
        case 9: EMIT("movnez\ta%d, a%d, a%d", r, s, t); break;
        case 10: EMIT("movltz\ta%d, a%d, a%d", r, s, t); break;
        case 11: EMIT("movgez\ta%d, a%d, a%d", r, s, t); break;
        case 12: EMIT("movf\ta%d, a%d, b%d", r, s, t); break;
        case 13: EMIT("movt\ta%d, a%d, b%d", r, s, t); break;
        case 14: /* RUR */
            { int ur = ((s << 4) | r);
              EMIT("rur\ta%d, %d", t, ur);
            } break;
        case 15: /* WUR */
            { int ur = ((s << 4) | r);
              EMIT("wur\ta%d, %d", t, ur);
            } break;
        }
        break;

    case 4: /* EXTUI - RRI4 format */
    case 5:
        { /* Xtensa.pdf: op2=imm[3..0], op1=010sh[4], r=dest, s=sh[3..0], t=src */
          int shift = (s | ((op1 & 1) << 4));
          int mask = op2 + 1; /* mask width 1..16 */
          EMIT("extui\ta%d, a%d, %d, %d", r, t, shift, mask);
        } break;

    case 8: /* LSCX - indexed FP loads/stores */
        switch (op2) {
        case 0: EMIT("lsx\tf%d, a%d, a%d", r, s, t); break;
        case 1: EMIT("lsxp\tf%d, a%d, a%d", r, s, t); break;
        case 4: EMIT("ssx\tf%d, a%d, a%d", r, s, t); break;
        case 5: EMIT("ssxp\tf%d, a%d, a%d", r, s, t); break;
        default: EMIT("??lscx op2=%d", op2); break;
        }
        break;

    case 9: /* LSC4 - L32E/S32E */
        switch (op2) {
        case 0: EMIT("l32e\ta%d, a%d, %d", t, s, (r << 2) - 64); break;
        case 4: EMIT("s32e\ta%d, a%d, %d", t, s, (r << 2) - 64); break;
        default: EMIT("??lsc4 op2=%d", op2); break;
        }
        break;

    case 10: /* FP0 - FP arithmetic, conversions */
        switch (op2) {
        case 0: EMIT("add.s\tf%d, f%d, f%d", r, s, t); break;
        case 1: EMIT("sub.s\tf%d, f%d, f%d", r, s, t); break;
        case 2: EMIT("mul.s\tf%d, f%d, f%d", r, s, t); break;
        case 4: EMIT("madd.s\tf%d, f%d, f%d", r, s, t); break;
        case 5: EMIT("msub.s\tf%d, f%d, f%d", r, s, t); break;
        case 6: EMIT("maddn.s\tf%d, f%d, f%d", r, s, t); break;
        case 7: EMIT("divn.s\tf%d, f%d, f%d", r, s, t); break;
        case 8: EMIT("round.s\ta%d, f%d, %d", t, s, r); break;
        case 9: EMIT("trunc.s\ta%d, f%d, %d", t, s, r); break;
        case 10: EMIT("floor.s\ta%d, f%d, %d", t, s, r); break;
        case 11: EMIT("ceil.s\ta%d, f%d, %d", t, s, r); break;
        case 12: EMIT("float.s\tf%d, a%d, %d", r, s, t); break;
        case 13: EMIT("ufloat.s\tf%d, a%d, %d", r, s, t); break;
        case 14: EMIT("utrunc.s\ta%d, f%d, %d", t, s, r); break;
        case 15: /* FP1OP */
            switch (t) {
            case 0: EMIT("mov.s\tf%d, f%d", r, s); break;
            case 1: EMIT("abs.s\tf%d, f%d", r, s); break;
            case 3: EMIT("const.s\tf%d, %d", r, s); break;
            case 4: EMIT("rfr\ta%d, f%d", r, s); break;
            case 5: EMIT("wfr\tf%d, a%d", r, s); break;
            case 6: EMIT("neg.s\tf%d, f%d", r, s); break;
            case 7: EMIT("div0.s\tf%d, f%d", r, s); break;
            case 8: EMIT("recip0.s\tf%d, f%d", r, s); break;
            case 9: EMIT("sqrt0.s\tf%d, f%d", r, s); break;
            case 10: EMIT("rsqrt0.s\tf%d, f%d", r, s); break;
            case 11: EMIT("nexp01.s\tf%d, f%d", r, s); break;
            case 12: EMIT("mksadj.s\tf%d, f%d", r, s); break;
            case 13: EMIT("mkdadj.s\tf%d, f%d", r, s); break;
            case 14: EMIT("addexp.s\tf%d, f%d", r, s); break;
            case 15: EMIT("addexpm.s\tf%d, f%d", r, s); break;
            default: EMIT("??fp1op t=%d", t); break;
            }
            break;
        default: EMIT("??fp0 op2=%d", op2); break;
        }
        break;

    case 11: /* FP1 - FP comparisons, conditional moves */
        switch (op2) {
        case 1: EMIT("un.s\tb%d, f%d, f%d", r, s, t); break;
        case 2: EMIT("oeq.s\tb%d, f%d, f%d", r, s, t); break;
        case 3: EMIT("ueq.s\tb%d, f%d, f%d", r, s, t); break;
        case 4: EMIT("olt.s\tb%d, f%d, f%d", r, s, t); break;
        case 5: EMIT("ult.s\tb%d, f%d, f%d", r, s, t); break;
        case 6: EMIT("ole.s\tb%d, f%d, f%d", r, s, t); break;
        case 7: EMIT("ule.s\tb%d, f%d, f%d", r, s, t); break;
        case 8: EMIT("moveqz.s\tf%d, f%d, a%d", r, s, t); break;
        case 9: EMIT("movnez.s\tf%d, f%d, a%d", r, s, t); break;
        case 10: EMIT("movltz.s\tf%d, f%d, a%d", r, s, t); break;
        case 11: EMIT("movgez.s\tf%d, f%d, a%d", r, s, t); break;
        case 12: EMIT("movf.s\tf%d, f%d, b%d", r, s, t); break;
        case 13: EMIT("movt.s\tf%d, f%d, b%d", r, s, t); break;
        default: EMIT("??fp1 op2=%d", op2); break;
        }
        break;

    default:
        EMIT("??qrst op1=%d", op1);
        break;
    }

    return 3;
}

/*
 * Disassemble op0=2 (LSAI) - Loads, stores, ADDI, MOVI
 */
static int disasm_lsai(uint32_t insn, uint32_t pc, char *buf, int bufsize) {
    char *p = buf;
    int rem = bufsize, n;
    int r = XT_R(insn);
    int s = XT_S(insn);
    int t = XT_T(insn);
    int imm8 = XT_IMM8(insn);

    switch (r) {
    case 0: /* L8UI */
        EMIT("l8ui\ta%d, a%d, %d", t, s, imm8); break;
    case 1: /* L16UI */
        EMIT("l16ui\ta%d, a%d, %d", t, s, imm8 << 1); break;
    case 2: /* L32I */
        EMIT("l32i\ta%d, a%d, %d", t, s, imm8 << 2); break;
    case 4: /* S8I */
        EMIT("s8i\ta%d, a%d, %d", t, s, imm8); break;
    case 5: /* S16I */
        EMIT("s16i\ta%d, a%d, %d", t, s, imm8 << 1); break;
    case 6: /* S32I */
        EMIT("s32i\ta%d, a%d, %d", t, s, imm8 << 2); break;
    case 7: /* CACHE - DPFR, DPFW, etc. */
        switch (t) {
        case 0: EMIT("dpfr\ta%d, %d", s, imm8 << 2); break;
        case 1: EMIT("dpfw\ta%d, %d", s, imm8 << 2); break;
        case 2: EMIT("dpfro\ta%d, %d", s, imm8 << 2); break;
        case 3: EMIT("dpfwo\ta%d, %d", s, imm8 << 2); break;
        case 4: EMIT("dhwb\ta%d, %d", s, imm8 << 2); break;
        case 5: EMIT("dhwbi\ta%d, %d", s, imm8 << 2); break;
        case 6: EMIT("dhi\ta%d, %d", s, imm8 << 2); break;
        case 7: EMIT("dii\ta%d, %d", s, imm8 << 2); break;
        case 12: EMIT("ipf\ta%d, %d", s, imm8 << 2); break;
        case 14: EMIT("ihi\ta%d, %d", s, imm8 << 2); break;
        case 15: EMIT("iii\ta%d, %d", s, imm8 << 2); break;
        default: EMIT("??cache t=%d", t); break;
        }
        break;
    case 9: /* L16SI */
        EMIT("l16si\ta%d, a%d, %d", t, s, imm8 << 1); break;
    case 10: /* MOVI */
        { /* 12-bit immediate: imm = sign_ext(imm8 | (s << 8), 12)
             Wait - MOVI encoding: r=1010, imm is imm[11..8]=s, imm[7..0]=imm8
             Actually looking at Xtensa.pdf page 16: MOVI: r=1010, imm[11..8]=t(!!!)
             No: the dest register is t, and the imm12 = s_field << 8 | imm8.
             Hmm, let me re-check. Xtensa.pdf encoding table says:
             imm[7..0]=imm8, 1010=r, imm[11..8]=s... but that uses s as part of imm.

             Actually from the research doc: MOVI: AR[t] = sign_ext_12(imm8 || r)
             No that's wrong too. Let me look at the actual encoding:

             From Xtensa.pdf page 16, MOVI row:
             imm[7..0]  1010  imm[11..8]  t  0010
             So: r=1010 (fixed), imm8=high byte, s=imm[11:8], t=dest register

             Full 12-bit immediate = (imm8 << 4) | s... no.

             Actually the format is RRI8: imm8[7:0] r[3:0] s[3:0] t[3:0] op0[3:0]
             For MOVI: r=0xA (1010), the 12-bit imm is { imm8[7:0], s[3:0] }
             Wait no, for MOVI the s field IS part of the immediate!

             From research doc: MOVI: imm12 = r[3:0] || imm8[7:0] ... no, r=1010 is fixed.

             Let me re-read: "AR[t] = sign_ext(imm) where imm = imm[11..8] || imm[7..0]"
             and looking at the encoding: imm[7..0] is bits 23:16, imm[11..8] is bits 11:8 = s field.

             So: imm12 = (s << 8) | imm8, sign-extended from bit 11.
          */
          int32_t imm12 = sign_extend((uint32_t)((s << 8) | imm8), 12);
          EMIT("movi\ta%d, %d", t, imm12);
        } break;
    case 11: /* L32AI - load acquire */
        EMIT("l32ai\ta%d, a%d, %d", t, s, imm8 << 2); break;
    case 12: /* ADDI */
        EMIT("addi\ta%d, a%d, %d", t, s, (int8_t)imm8); break;
    case 13: /* ADDMI */
        EMIT("addmi\ta%d, a%d, %d", t, s, (int8_t)imm8 * 256); break;
    case 14: /* S32C1I */
        EMIT("s32c1i\ta%d, a%d, %d", t, s, imm8 << 2); break;
    case 15: /* S32RI - store release */
        EMIT("s32ri\ta%d, a%d, %d", t, s, imm8 << 2); break;
    default:
        EMIT("??lsai r=%d", r);
        break;
    }
    return 3;
}

/*
 * Disassemble op0=5 (CALLN) - CALL format
 */
static int disasm_calln(uint32_t insn, uint32_t pc, char *buf, int bufsize) {
    char *p = buf;
    int rem = bufsize, n;
    int nn = XT_N(insn);
    int32_t offset = sign_extend(XT_OFFSET18(insn), 18);
    uint32_t target = ((pc >> 2) + offset + 1) << 2;

    switch (nn) {
    case 0: EMIT("call0\t0x%x", target); break;
    case 1: EMIT("call4\t0x%x", target); break;
    case 2: EMIT("call8\t0x%x", target); break;
    case 3: EMIT("call12\t0x%x", target); break;
    }
    return 3;
}

/*
 * Disassemble op0=6 (SI) - BRI12, BRI8, CALL formats
 * n field selects sub-group
 */
static int disasm_si(uint32_t insn, uint32_t pc, char *buf, int bufsize) {
    char *p = buf;
    int rem = bufsize, n;
    int nn = XT_N(insn);
    int m = XT_M(insn);
    int s = XT_S(insn);

    switch (nn) {
    case 0: /* J */
        { int32_t offset = sign_extend(XT_OFFSET18(insn), 18);
          uint32_t target = pc + offset + 4;
          EMIT("j\t0x%x", target);
        } break;
    case 1: /* BZ - BRI12 format */
        { int32_t imm12 = sign_extend(XT_IMM12(insn), 12);
          uint32_t target = pc + imm12 + 4;
          switch (m) {
          case 0: EMIT("beqz\ta%d, 0x%x", s, target); break;
          case 1: EMIT("bnez\ta%d, 0x%x", s, target); break;
          case 2: EMIT("bltz\ta%d, 0x%x", s, target); break;
          case 3: EMIT("bgez\ta%d, 0x%x", s, target); break;
          }
        } break;
    case 2: /* BI0 - BRI8 format */
        { int imm8 = XT_IMM8(insn);
          int r = XT_R(insn);
          int32_t offset8 = sign_extend(imm8, 8);
          uint32_t target = pc + offset8 + 4;
          switch (m) {
          case 0: EMIT("beqi\ta%d, %d, 0x%x", s, b4const[r], target); break;
          case 1: EMIT("bnei\ta%d, %d, 0x%x", s, b4const[r], target); break;
          case 2: EMIT("blti\ta%d, %d, 0x%x", s, b4const[r], target); break;
          case 3: EMIT("bgei\ta%d, %d, 0x%x", s, b4const[r], target); break;
          }
        } break;
    case 3: /* BI1 - BRI8/BRI12 format */
        { int imm8 = XT_IMM8(insn);
          int r = XT_R(insn);
          switch (m) {
          case 0: /* ENTRY */
              { /* BRI12 format: imm12 is the frame size / 8 */
                int imm12 = XT_IMM12(insn);
                int framesize = imm12 << 3;
                EMIT("entry\ta%d, %d", s, framesize);
              } break;
          case 1: /* B1 - BF/BT, LOOP */
              { int32_t offset8 = sign_extend(imm8, 8);
                uint32_t target = pc + offset8 + 4;
                switch (r) {
                case 0: EMIT("bf\tb%d, 0x%x", s, target); break;
                case 1: EMIT("bt\tb%d, 0x%x", s, target); break;
                case 8: EMIT("loop\ta%d, 0x%x", s, target); break;
                case 9: EMIT("loopnez\ta%d, 0x%x", s, target); break;
                case 10: EMIT("loopgtz\ta%d, 0x%x", s, target); break;
                default: EMIT("??b1 r=%d", r); break;
                }
              } break;
          case 2: /* BLTUI */
              { int32_t offset8 = sign_extend(imm8, 8);
                uint32_t target = pc + offset8 + 4;
                EMIT("bltui\ta%d, %u, 0x%x", s, b4constu[r], target);
              } break;
          case 3: /* BGEUI */
              { int32_t offset8 = sign_extend(imm8, 8);
                uint32_t target = pc + offset8 + 4;
                EMIT("bgeui\ta%d, %u, 0x%x", s, b4constu[r], target);
              } break;
          }
        } break;
    }
    return 3;
}

/*
 * Disassemble op0=7 (B) - RRI8 conditional branches
 */
static int disasm_b(uint32_t insn, uint32_t pc, char *buf, int bufsize) {
    char *p = buf;
    int rem = bufsize, n;
    int r = XT_R(insn);
    int s = XT_S(insn);
    int t = XT_T(insn);
    int imm8 = XT_IMM8(insn);
    int32_t offset = sign_extend(imm8, 8);
    uint32_t target = pc + offset + 4;

    switch (r) {
    case 0: EMIT("bnone\ta%d, a%d, 0x%x", s, t, target); break;
    case 1: EMIT("beq\ta%d, a%d, 0x%x", s, t, target); break;
    case 2: EMIT("blt\ta%d, a%d, 0x%x", s, t, target); break;
    case 3: EMIT("bltu\ta%d, a%d, 0x%x", s, t, target); break;
    case 4: EMIT("ball\ta%d, a%d, 0x%x", s, t, target); break;
    case 5: EMIT("bbc\ta%d, a%d, 0x%x", s, t, target); break;
    case 6: case 7:
        /* BBCI / BBSI: bit number = (r-6)*16 + t => bit = t | ((r&1)<<4) */
        { int bit = t | ((r & 1) << 4);
          if (r == 6) EMIT("bbci\ta%d, %d, 0x%x", s, bit, target);
          else        EMIT("bbsi\ta%d, %d, 0x%x", s, bit, target);
        } break;
    case 8: EMIT("bany\ta%d, a%d, 0x%x", s, t, target); break;
    case 9: EMIT("bne\ta%d, a%d, 0x%x", s, t, target); break;
    case 10: EMIT("bge\ta%d, a%d, 0x%x", s, t, target); break;
    case 11: EMIT("bgeu\ta%d, a%d, 0x%x", s, t, target); break;
    case 12: EMIT("bnall\ta%d, a%d, 0x%x", s, t, target); break;
    case 13: EMIT("bbs\ta%d, a%d, 0x%x", s, t, target); break;
    case 14: case 15:
        { int bit = t | ((r & 1) << 4);
          if (r == 14) EMIT("bbci\ta%d, %d, 0x%x", s, bit, target);
          else         EMIT("bbsi\ta%d, %d, 0x%x", s, bit, target);
        } break;
    }
    return 3;
}

/*
 * Disassemble 16-bit (narrow) instructions
 */
static int disasm_narrow(uint16_t insn, uint32_t pc, char *buf, int bufsize) {
    char *p = buf;
    int rem = bufsize, n;
    int op0 = XT_OP0_N(insn);
    int r = XT_R_N(insn);
    int s = XT_S_N(insn);
    int t = XT_T_N(insn);

    switch (op0) {
    case 8: /* L32I.N */
        EMIT("l32i.n\ta%d, a%d, %d", t, s, r << 2);
        break;
    case 9: /* S32I.N */
        EMIT("s32i.n\ta%d, a%d, %d", t, s, r << 2);
        break;
    case 10: /* ADD.N */
        EMIT("add.n\ta%d, a%d, a%d", r, s, t);
        break;
    case 11: /* ADDI.N */
        { int imm = t;
          if (imm == 0) imm = -1;
          EMIT("addi.n\ta%d, a%d, %d", r, s, imm);
        } break;
    case 12: /* ST2 */
        switch (t & 3) { /* i and z bits: t[7]=i, t[6]=z, but for 16-bit: bits 7,6 */
            /* Actually for RI6 and RI7 formats, we need to look at different bits.
               ST2 decodes on t field (bits 7:4 of the 16-bit insn):
               t[1:0] = bits 5:4
               t=00xx -> MOVI.N (RI7 format)
               t=10xx -> BEQZ.N (RI6 format)
               t=11xx -> BNEZ.N (RI6 format) */
        default: break;
        }
        /* Re-decode ST2 properly */
        { int t_hi = (t >> 2) & 3; /* bits 7:6 */
          if (t_hi < 2) { /* i=0 (bit 7=0): MOVI.N */
              /* MOVI.N - RI7 format */
              /* imm7 = r[3:0] || 0 || (t[1:0] as imm[6:4])
                 Wait: from the format diagram (page 658):
                 RI7 little-endian: imm7[3:0] | s | i | imm7[6:4] | op0
                 So: imm7[3:0] = r field (bits 15:12)
                     i = bit 7
                     imm7[6:4] = bits 6:4 = t[2:0]

                 Full: imm7 = (r << 4) | ... no.
                 Actually from Xtensa.pdf page 22:
                 MOVI.N: imm[3..0] | s | 0imm[6..4] | 1100
                 In our 16-bit LE word:
                   bits 15:12 = imm[3:0] = r field
                   bits 11:8 = s
                   bit 7 = i (sign/high bit) = 0 for MOVI.N
                   bits 6:4 = imm[6:4]
                   bits 3:0 = op0 = 1100

                 So imm7 = (r << 3) | (t & 7)... hmm, that's only 7 bits if r is 4 bits.
                 Actually: imm7[6:4] = t[2:0], imm7[3:0] = r[3:0]
                 So imm7 = (t[2:0] << 4) | r[3:0] ... but that's also 7 bits.
                 Wait: i=0 means it's MOVI.N. The 7-bit immediate:
                 imm = (r[3:0] << 0) placed in [3:0] and (t[2:0]) in [6:4]
                 No wait, from the big-endian format: imm7[3:0] is in bits 12:15 (r field in LE),
                 and imm7[6:4] is in bits 4:6 (lower part of t field in LE).

                 Let me just use the standard decode: for MOVI.N
                 The 7-bit value = (r << 0) ... Let me look at the actual encoding one more time.

                 RI7 (LE): bits 15:12=imm7[3:0], bits 11:8=s, bit 7=i, bits 6:4=imm7[6:4], bits 3:0=op0

                 So: imm7_lo = r (bits 15:12) = imm7[3:0]
                     imm7_hi = (insn >> 4) & 7 = t[2:0] = imm7[6:4]
                     i = (insn >> 7) & 1 = t[3]

                 imm7 = (imm7_hi << 4) | imm7_lo

                 Then the full immediate for MOVI.N is sign_extend(imm7, 7) if i indicates sign.

                 Actually from the research doc:
                 "imm7 = r[3:0] || 0 || imm[6:4]" ... hmm that doesn't match.

                 Let me just look at Xtensa.pdf page 22 one more time:
                 MOVI.N: AR[s] = sign_extend(imm)
                 And the encoding shows: imm[3..0] s 0imm[6..4] 1100

                 The "0" before imm[6..4] is the 'i' bit = 0 (distinguishing from BEQZ.N/BNEZ.N).

                 So imm7[6:4] = t & 0x7 (bits 6:4), imm7[3:0] = r (bits 15:12)
                 Full 7-bit value = ((t & 7) << 4) | r
                 Sign extend from bit 6: range -32..95
              */
              int imm7 = ((t & 7) << 4) | r;
              int32_t val = sign_extend(imm7, 7);
              EMIT("movi.n\ta%d, %d", s, val);
          } else if (t_hi == 2) {
              /* BEQZ.N - RI6 format */
              /* RI6 (LE): bits 15:12=imm6[3:0], bits 11:8=s, bit 7=i, bit 6=z, bits 5:4=imm6[5:4], bits 3:0=op0
                 For BEQZ.N: i=1, z=0
                 imm6[5:4] = (insn >> 4) & 3, imm6[3:0] = r
                 imm6 = ((t & 3) << 4) | r  -- where t&3 = bits 5:4
                 But t is bits 7:4, so t&3 = bits 5:4. i = bit 7 = t>>3, z = bit 6 = (t>>2)&1

                 Actually: imm6[5:4] = bits 5:4, imm6[3:0] = bits 15:12
                 imm6 = (((insn >> 4) & 3) << 4) | r
                 This is unsigned offset (0..63), target = PC + imm6 + 4
              */
              int imm6 = ((t & 3) << 4) | r;
              uint32_t target = pc + imm6 + 4;
              EMIT("beqz.n\ta%d, 0x%x", s, target);
          } else if (t_hi == 3) {
              /* BNEZ.N */
              int imm6 = ((t & 3) << 4) | r;
              uint32_t target = pc + imm6 + 4;
              EMIT("bnez.n\ta%d, 0x%x", s, target);
          } else {
              EMIT("??st2 t=%d", t);
          }
        }
        break;
    case 13: /* ST3 */
        switch (r) {
        case 0: /* MOV.N */
            EMIT("mov.n\ta%d, a%d", t, s);
            break;
        case 15: /* S3 */
            switch (t) {
            case 0:
                if (s == 0) EMIT("ret.n");
                else EMIT("??ret.n s=%d", s);
                break;
            case 1:
                if (s == 0) EMIT("retw.n");
                else EMIT("??retw.n s=%d", s);
                break;
            case 2:
                if (s == 0) EMIT("break.n\t0");
                else EMIT("break.n\t%d", s);
                break;
            case 3:
                if (s == 0) EMIT("nop.n");
                else EMIT("??nop.n s=%d", s);
                break;
            case 6:
                /* ILL.N */
                if (s == 0) EMIT("ill.n");
                else EMIT("??ill.n s=%d", s);
                break;
            default:
                EMIT("??s3 t=%d", t);
                break;
            }
            break;
        default:
            EMIT("??st3 r=%d", r);
            break;
        }
        break;
    default:
        EMIT("??narrow op0=%d", op0);
        break;
    }
    return 2;
}

/*
 * Main disassembly entry point.
 * Fetches instruction bytes from memory, decodes, writes assembly text to buf.
 * Returns instruction length in bytes (2 or 3).
 */
int xtensa_disasm(const xtensa_cpu_t *cpu, uint32_t addr, char *buf, int bufsize) {
    if (!cpu || !cpu->mem || bufsize < 1) {
        if (buf && bufsize > 0) buf[0] = '\0';
        return 3;
    }

    /* Fetch bytes */
    uint32_t insn;
    int ilen = xtensa_fetch(cpu, addr, &insn);
    if (ilen == 0) {
        snprintf(buf, bufsize, "??[unmapped 0x%08x]", addr);
        return 3;
    }

    int op0 = XT_OP0(insn);

    /* 16-bit narrow instruction? */
    if (ilen == 2) {
        return disasm_narrow((uint16_t)insn, addr, buf, bufsize);
    }

    char *p = buf;
    int rem = bufsize, n;

    switch (op0) {
    case 0: /* QRST */
        return disasm_qrst(insn, addr, buf, bufsize);

    case 1: /* L32R - RI16 format */
        { int t = XT_T(insn);
          int32_t imm16 = (int32_t)(int16_t)XT_IMM16(insn);
          /* L32R offset = (1^14 || imm16 || 0^2) - always negative
             vAddr = ((PC+3) & ~3) + (0xFFFC0000 | (imm16 << 2)) */
          uint32_t offset = 0xFFFC0000u | ((uint32_t)(uint16_t)imm16 << 2);
          uint32_t target = ((addr + 3) & ~3u) + offset;
          EMIT("l32r\ta%d, 0x%x", t, target);
        }
        return 3;

    case 2: /* LSAI */
        return disasm_lsai(insn, addr, buf, bufsize);

    case 3: /* LSCI - FP loads/stores */
        { int r = XT_R(insn);
          int s = XT_S(insn);
          int t = XT_T(insn);
          int imm8 = XT_IMM8(insn);
          switch (r) {
          case 0: EMIT("lsi\tf%d, a%d, %d", t, s, imm8 << 2); break;
          case 4: EMIT("ssi\tf%d, a%d, %d", t, s, imm8 << 2); break;
          case 8: EMIT("lsiu\tf%d, a%d, %d", t, s, imm8 << 2); break;
          case 12: EMIT("ssiu\tf%d, a%d, %d", t, s, imm8 << 2); break;
          default: EMIT("??lsci r=%d", r); break;
          }
        }
        return 3;

    case 4: /* MAC16 */
        { int op1 = XT_OP1(insn);
          int op2 = XT_OP2(insn);
          int r = XT_R(insn);
          int s = XT_S(insn);
          (void)XT_T(insn); /* t not used directly in MAC16 disasm */
          /* MAC16 has complex encoding. For disasm, just show the raw fields. */
          /* The qq suffix comes from op1[1:0]: 00=LL, 01=HL, 10=LH, 11=HH */
          static const char *qq[4] = {"ll", "hl", "lh", "hh"};
          const char *q = qq[op1 & 3];
          /* op2[1:0] selects the operation type */
          switch (op2) {
          case 0: /* MACID group */
          case 1:
          case 2:
          case 3:
              /* Simplified: show as generic MAC16 */
              EMIT("mac16\top2=%d, op1=%d", op2, op1);
              break;
          case 4: /* MACI */
              if (op1 == 0) EMIT("lddec\tm%d, a%d", r >> 2, s);
              else EMIT("mac16.4\top1=%d", op1);
              break;
          case 5: /* MACC */
              if (op1 == 0) EMIT("ldinc\tm%d, a%d", r >> 2, s);
              else EMIT("mac16.5\top1=%d", op1);
              break;
          case 8: /* MULA.DA.*.LDDEC */
              EMIT("mula.da.%s.lddec\tm%d, a%d", q, r >> 2, s);
              break;
          case 9: /* MULA.DA.*.LDINC */
              EMIT("mula.da.%s.ldinc\tm%d, a%d", q, r >> 2, s);
              break;
          case 10: /* MULA.DD.*.LDDEC */
              EMIT("mula.dd.%s.lddec\tm%d, a%d", q, r >> 2, s);
              break;
          case 11: /* MULA.DD.*.LDINC */
              EMIT("mula.dd.%s.ldinc\tm%d, a%d", q, r >> 2, s);
              break;
          default:
              /* Simple MAC16 ops */
              { const char *ops[] = {"mul", "mula", "muls", "umul"};
                const char *src[] = {"aa", "ad", "da", "dd"};
                int src_sel = (op2 >> 2) & 3;
                int op_sel = (op1 >> 2) & 3;
                if (src_sel < 4 && op_sel < 4)
                    EMIT("%s.%s.%s", ops[op_sel], src[src_sel], q);
                else
                    EMIT("mac16\t0x%06x", insn);
              }
              break;
          }
        }
        return 3;

    case 5: /* CALLN */
        return disasm_calln(insn, addr, buf, bufsize);

    case 6: /* SI */
        return disasm_si(insn, addr, buf, bufsize);

    case 7: /* B */
        return disasm_b(insn, addr, buf, bufsize);

    default:
        EMIT("??op0=%d", op0);
        return 3;
    }
}
