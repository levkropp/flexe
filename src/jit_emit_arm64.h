#ifndef JIT_EMIT_ARM64_H
#define JIT_EMIT_ARM64_H

/*
 * ARM64 (AArch64) machine code emitter for the Xtensa JIT compiler.
 * Drop-in replacement for jit_emit_x64.h: every public function name
 * matches, and the `RAX..R15` enum indices line up with the x64 file
 * so src/jit.c can use the same register-constant names unchanged.
 *
 * Register name map (x64 name -> ARM64 X-register number):
 *
 *   RAX = X0   first scratch / function return
 *   RCX = X1   scratch
 *   RDX = X2   scratch
 *   RBX = X3   scratch
 *   RSP = X4   (NOT ARM64 SP; the real SP is encoding 31. Index 4 is a
 *               placeholder so the enum positions match the x64 file.
 *               jit.c never uses RSP for addressing on ARM64.)
 *   RBP = X5   scratch (similarly placeholder; real frame uses X29)
 *   RSI = X6   scratch
 *   RDI = X7   scratch
 *   R8  = X19  callee-saved (matches x86 callee-saved semantics)
 *   R9  = X20  callee-saved
 *   R10 = X21  callee-saved
 *   R11 = X22  callee-saved
 *   R12 = X23  callee-saved
 *   R13 = X24  callee-saved
 *   R14 = X25  callee-saved (cpu->mem cache)
 *   R15 = X26  callee-saved (xtensa_cpu_t *cpu)
 *
 * ARM64 instructions are always 4 bytes. All encoding goes through
 * emit32() of the encoded instruction word.  Encoding references are
 * from the "Arm Architecture Reference Manual, A-profile" (Arm ARM).
 */

#include <stdint.h>
#include <string.h>
#include <stddef.h>

/* Enum indices must match jit_emit_x64.h exactly so jit.c builds
 * with identical register constants. The value is the ARM64 X-reg
 * number we want the emitter to use for that name. */
enum {
    RAX = 0,  /* X0  */
    RCX = 1,  /* X1  */
    RDX = 2,  /* X2  */
    RBX = 3,  /* X3  */
    RSP = 4,  /* X4  placeholder; ARM64 SP is encoding 31, unused here */
    RBP = 5,  /* X5  placeholder; real FP would be X29 */
    RSI = 6,  /* X6  */
    RDI = 7,  /* X7  */
    R8  = 19, /* X19 callee-saved */
    R9  = 20, /* X20 */
    R10 = 21, /* X21 */
    R11 = 22, /* X22 */
    R12 = 23, /* X23 */
    R13 = 24, /* X24 */
    R14 = 25, /* X25 cpu->mem */
    R15 = 26  /* X26 xtensa_cpu_t *cpu */
};

/* Scratch register used internally by the emitter for synthesized
 * sequences (large-immediate materialization, far branches, indirect
 * calls). X9 is a caller-saved temp reserved for codegen; jit.c's
 * register allocator never picks it. */
#define ARM64_SCRATCH 9
/* Second scratch (X10) for two-temp sequences (e.g. store-imm far-disp).
 * Neither X9 nor X10 appear in the RAX..R15 compat enum (0-7, 19-26),
 * so jit.c body code can never hold live values in them. */
#define ARM64_SCRATCH2 10
#define ARM64_SP_ENC  31  /* encoding used by LDR/STR when base==SP */

/* Emitter context — identical layout to the x64 version. */
typedef struct {
    uint8_t *buf;       /* Start of code buffer */
    uint8_t *ptr;       /* Current write position */
    uint8_t *end;       /* End of buffer */
} emit_t;

static inline void emit_init(emit_t *e, uint8_t *buf, size_t size) {
    e->buf = buf;
    e->ptr = buf;
    e->end = buf + size;
}

static inline size_t emit_size(const emit_t *e) {
    return (size_t)(e->ptr - e->buf);
}

static inline int emit_ok(const emit_t *e) {
    return e->ptr < e->end;
}

/* Raw byte emitters. On ARM64 only emit32() is normally used, but
 * the others are kept so the API matches jit_emit_x64.h. */
static inline void emit8(emit_t *e, uint8_t b) {
    if (e->ptr < e->end) *e->ptr++ = b;
}

static inline void emit16(emit_t *e, uint16_t w) {
    if (e->ptr + 2 <= e->end) {
        memcpy(e->ptr, &w, 2);
        e->ptr += 2;
    }
}

static inline void emit32(emit_t *e, uint32_t d) {
    if (e->ptr + 4 <= e->end) {
        memcpy(e->ptr, &d, 4);
        e->ptr += 4;
    }
}

static inline void emit64(emit_t *e, uint64_t q) {
    if (e->ptr + 8 <= e->end) {
        memcpy(e->ptr, &q, 8);
        e->ptr += 8;
    }
}

/* ===== MOV / constant materialization ===== */

/* MOVZ Xd, #imm16, LSL #sh — Arm ARM C6.2.207 */
static inline void emit_arm64_movz(emit_t *e, int rd, uint16_t imm, int sh) {
    emit32(e, 0xD2800000u | ((uint32_t)(sh / 16) << 21)
             | ((uint32_t)imm << 5) | (uint32_t)(rd & 31));
}

/* MOVK Xd, #imm16, LSL #sh — Arm ARM C6.2.205 */
static inline void emit_arm64_movk(emit_t *e, int rd, uint16_t imm, int sh) {
    emit32(e, 0xF2800000u | ((uint32_t)(sh / 16) << 21)
             | ((uint32_t)imm << 5) | (uint32_t)(rd & 31));
}

/* mov reg, imm64: MOVZ + up to three MOVKs. */
static inline void emit_mov_reg_imm64(emit_t *e, int reg, uint64_t imm) {
    emit_arm64_movz(e, reg, (uint16_t)(imm & 0xFFFF), 0);
    if (((imm >> 16) & 0xFFFF) != 0)
        emit_arm64_movk(e, reg, (uint16_t)((imm >> 16) & 0xFFFF), 16);
    if (((imm >> 32) & 0xFFFF) != 0)
        emit_arm64_movk(e, reg, (uint16_t)((imm >> 32) & 0xFFFF), 32);
    if (((imm >> 48) & 0xFFFF) != 0)
        emit_arm64_movk(e, reg, (uint16_t)((imm >> 48) & 0xFFFF), 48);
}

/* mov reg32, imm32: two 16-bit chunks on the W-view (upper 32 bits
 * are zeroed automatically by the W-form). We always emit MOVZ of
 * the low half then MOVK of the high half if non-zero. */
static inline void emit_mov_reg_imm32(emit_t *e, int reg, uint32_t imm) {
    /* 32-bit MOVZ: 0x52800000; MOVK: 0x72800000 */
    emit32(e, 0x52800000u | ((uint32_t)(imm & 0xFFFF) << 5)
             | (uint32_t)(reg & 31));
    if ((imm >> 16) != 0) {
        emit32(e, 0x72800000u | (1u << 21)
                 | ((uint32_t)((imm >> 16) & 0xFFFF) << 5)
                 | (uint32_t)(reg & 31));
    }
}

/* mov Xd, Xn — alias of ORR Xd, XZR, Xn. Arm ARM C6.2.204. */
static inline void emit_mov_reg_reg(emit_t *e, int dst, int src) {
    emit32(e, 0xAA0003E0u | ((uint32_t)(src & 31) << 16)
             | (uint32_t)(dst & 31));
}

/* mov Wd, Wn — 32-bit form, zero-extends. */
static inline void emit_mov_reg32_reg32(emit_t *e, int dst, int src) {
    emit32(e, 0x2A0003E0u | ((uint32_t)(src & 31) << 16)
             | (uint32_t)(dst & 31));
}

/* ===== Memory loads/stores =====
 *
 * Preferred form is the 12-bit unsigned-scaled immediate (LDR/STR
 * (immediate)).  Range:
 *    32-bit access: 0 .. 16380 in steps of 4
 *    64-bit access: 0 .. 32760 in steps of 8
 *     8-bit access: 0 .. 4095
 *    16-bit access: 0 .. 8190 in steps of 2
 *
 * For displacements outside that range OR negative, fall back to
 * LDUR/STUR (unscaled) with a signed 9-bit immediate (-256..+255).
 * If even that does not fit, synthesize: materialize disp in the
 * scratch and use the register-offset form.
 */

/* LDUR/STUR encoding helpers. imm9 is a signed 9-bit value. */
static inline void emit_arm64_ldur32(emit_t *e, int rt, int rn, int imm9) {
    emit32(e, 0xB8400000u | (((uint32_t)imm9 & 0x1FFu) << 12)
             | ((uint32_t)(rn & 31) << 5) | (uint32_t)(rt & 31));
}
static inline void emit_arm64_stur32(emit_t *e, int rt, int rn, int imm9) {
    emit32(e, 0xB8000000u | (((uint32_t)imm9 & 0x1FFu) << 12)
             | ((uint32_t)(rn & 31) << 5) | (uint32_t)(rt & 31));
}
static inline void emit_arm64_ldur64(emit_t *e, int rt, int rn, int imm9) {
    emit32(e, 0xF8400000u | (((uint32_t)imm9 & 0x1FFu) << 12)
             | ((uint32_t)(rn & 31) << 5) | (uint32_t)(rt & 31));
}
static inline void emit_arm64_stur64(emit_t *e, int rt, int rn, int imm9) {
    emit32(e, 0xF8000000u | (((uint32_t)imm9 & 0x1FFu) << 12)
             | ((uint32_t)(rn & 31) << 5) | (uint32_t)(rt & 31));
}
static inline void emit_arm64_ldurb(emit_t *e, int rt, int rn, int imm9) {
    emit32(e, 0x38400000u | (((uint32_t)imm9 & 0x1FFu) << 12)
             | ((uint32_t)(rn & 31) << 5) | (uint32_t)(rt & 31));
}
static inline void emit_arm64_ldurh(emit_t *e, int rt, int rn, int imm9) {
    emit32(e, 0x78400000u | (((uint32_t)imm9 & 0x1FFu) << 12)
             | ((uint32_t)(rn & 31) << 5) | (uint32_t)(rt & 31));
}
static inline void emit_arm64_ldursh32(emit_t *e, int rt, int rn, int imm9) {
    /* LDURSH Wt — opc=11 for 32-bit sign-extended halfword load */
    emit32(e, 0x78C00000u | (((uint32_t)imm9 & 0x1FFu) << 12)
             | ((uint32_t)(rn & 31) << 5) | (uint32_t)(rt & 31));
}
static inline void emit_arm64_sturb(emit_t *e, int rt, int rn, int imm9) {
    emit32(e, 0x38000000u | (((uint32_t)imm9 & 0x1FFu) << 12)
             | ((uint32_t)(rn & 31) << 5) | (uint32_t)(rt & 31));
}
static inline void emit_arm64_sturh(emit_t *e, int rt, int rn, int imm9) {
    emit32(e, 0x78000000u | (((uint32_t)imm9 & 0x1FFu) << 12)
             | ((uint32_t)(rn & 31) << 5) | (uint32_t)(rt & 31));
}

/* LDR Wt,  [Xn, #imm]  Arm ARM C6.2.130 (32-bit unsigned offset) */
static inline void emit_load32_disp(emit_t *e, int dst, int base, int32_t disp) {
    if (disp >= 0 && disp <= 16380 && (disp & 3) == 0) {
        uint32_t u = (uint32_t)(disp >> 2) & 0xFFFu;
        emit32(e, 0xB9400000u | (u << 10)
                 | ((uint32_t)(base & 31) << 5) | (uint32_t)(dst & 31));
    } else if (disp >= -256 && disp <= 255) {
        emit_arm64_ldur32(e, dst, base, disp);
    } else {
        /* LDR Wt, [Xn, Xm] — materialize offset in scratch, reg-offset form */
        emit_mov_reg_imm64(e, ARM64_SCRATCH, (uint64_t)(int64_t)disp);
        /* LDR Wt, [Xn, Xm, LSL #0] 0xB8606800 | Xm<<16 | Xn<<5 | Wt */
        emit32(e, 0xB8606800u | ((uint32_t)ARM64_SCRATCH << 16)
                 | ((uint32_t)(base & 31) << 5) | (uint32_t)(dst & 31));
    }
}

/* STR Wt, [Xn, #imm]  Arm ARM C6.2.273 */
static inline void emit_store32_disp(emit_t *e, int src, int base, int32_t disp) {
    if (disp >= 0 && disp <= 16380 && (disp & 3) == 0) {
        uint32_t u = (uint32_t)(disp >> 2) & 0xFFFu;
        emit32(e, 0xB9000000u | (u << 10)
                 | ((uint32_t)(base & 31) << 5) | (uint32_t)(src & 31));
    } else if (disp >= -256 && disp <= 255) {
        emit_arm64_stur32(e, src, base, disp);
    } else {
        emit_mov_reg_imm64(e, ARM64_SCRATCH, (uint64_t)(int64_t)disp);
        emit32(e, 0xB8206800u | ((uint32_t)ARM64_SCRATCH << 16)
                 | ((uint32_t)(base & 31) << 5) | (uint32_t)(src & 31));
    }
}

/* LDR Xt, [Xn, #imm] — 64-bit unsigned offset */
static inline void emit_load64_disp(emit_t *e, int dst, int base, int32_t disp) {
    if (disp >= 0 && disp <= 32760 && (disp & 7) == 0) {
        uint32_t u = (uint32_t)(disp >> 3) & 0xFFFu;
        emit32(e, 0xF9400000u | (u << 10)
                 | ((uint32_t)(base & 31) << 5) | (uint32_t)(dst & 31));
    } else if (disp >= -256 && disp <= 255) {
        emit_arm64_ldur64(e, dst, base, disp);
    } else {
        emit_mov_reg_imm64(e, ARM64_SCRATCH, (uint64_t)(int64_t)disp);
        /* LDR Xt, [Xn, Xm, LSL #0] — 0xF8606800 */
        emit32(e, 0xF8606800u | ((uint32_t)ARM64_SCRATCH << 16)
                 | ((uint32_t)(base & 31) << 5) | (uint32_t)(dst & 31));
    }
}

/* STR Xt, [Xn, #imm] */
static inline void emit_store64_disp(emit_t *e, int src, int base, int32_t disp) {
    if (disp >= 0 && disp <= 32760 && (disp & 7) == 0) {
        uint32_t u = (uint32_t)(disp >> 3) & 0xFFFu;
        emit32(e, 0xF9000000u | (u << 10)
                 | ((uint32_t)(base & 31) << 5) | (uint32_t)(src & 31));
    } else if (disp >= -256 && disp <= 255) {
        emit_arm64_stur64(e, src, base, disp);
    } else {
        emit_mov_reg_imm64(e, ARM64_SCRATCH, (uint64_t)(int64_t)disp);
        emit32(e, 0xF8206800u | ((uint32_t)ARM64_SCRATCH << 16)
                 | ((uint32_t)(base & 31) << 5) | (uint32_t)(src & 31));
    }
}

/* LDRB Wt, [Xn, #imm]  Arm ARM C6.2.133 (zero-extends to 32) */
static inline void emit_load8u_disp(emit_t *e, int dst, int base, int32_t disp) {
    if (disp >= 0 && disp <= 4095) {
        emit32(e, 0x39400000u | ((uint32_t)disp << 10)
                 | ((uint32_t)(base & 31) << 5) | (uint32_t)(dst & 31));
    } else if (disp >= -256 && disp <= 255) {
        emit_arm64_ldurb(e, dst, base, disp);
    } else {
        emit_mov_reg_imm64(e, ARM64_SCRATCH, (uint64_t)(int64_t)disp);
        emit32(e, 0x38606800u | ((uint32_t)ARM64_SCRATCH << 16)
                 | ((uint32_t)(base & 31) << 5) | (uint32_t)(dst & 31));
    }
}

/* LDRH Wt, [Xn, #imm]  Arm ARM C6.2.137 */
static inline void emit_load16u_disp(emit_t *e, int dst, int base, int32_t disp) {
    if (disp >= 0 && disp <= 8190 && (disp & 1) == 0) {
        uint32_t u = (uint32_t)(disp >> 1) & 0xFFFu;
        emit32(e, 0x79400000u | (u << 10)
                 | ((uint32_t)(base & 31) << 5) | (uint32_t)(dst & 31));
    } else if (disp >= -256 && disp <= 255) {
        emit_arm64_ldurh(e, dst, base, disp);
    } else {
        emit_mov_reg_imm64(e, ARM64_SCRATCH, (uint64_t)(int64_t)disp);
        emit32(e, 0x78606800u | ((uint32_t)ARM64_SCRATCH << 16)
                 | ((uint32_t)(base & 31) << 5) | (uint32_t)(dst & 31));
    }
}

/* LDRSH Wt, [Xn, #imm]  Arm ARM C6.2.142 — 32-bit sign-extend halfword */
static inline void emit_load16s_disp(emit_t *e, int dst, int base, int32_t disp) {
    if (disp >= 0 && disp <= 8190 && (disp & 1) == 0) {
        uint32_t u = (uint32_t)(disp >> 1) & 0xFFFu;
        emit32(e, 0x79C00000u | (u << 10)
                 | ((uint32_t)(base & 31) << 5) | (uint32_t)(dst & 31));
    } else if (disp >= -256 && disp <= 255) {
        emit_arm64_ldursh32(e, dst, base, disp);
    } else {
        emit_mov_reg_imm64(e, ARM64_SCRATCH, (uint64_t)(int64_t)disp);
        /* LDRSH Wt, [Xn, Xm, LSL #0] — 0x78E06800 */
        emit32(e, 0x78E06800u | ((uint32_t)ARM64_SCRATCH << 16)
                 | ((uint32_t)(base & 31) << 5) | (uint32_t)(dst & 31));
    }
}

/* STRB Wt, [Xn, #imm] */
static inline void emit_store8_disp(emit_t *e, int src, int base, int32_t disp) {
    if (disp >= 0 && disp <= 4095) {
        emit32(e, 0x39000000u | ((uint32_t)disp << 10)
                 | ((uint32_t)(base & 31) << 5) | (uint32_t)(src & 31));
    } else if (disp >= -256 && disp <= 255) {
        emit_arm64_sturb(e, src, base, disp);
    } else {
        emit_mov_reg_imm64(e, ARM64_SCRATCH, (uint64_t)(int64_t)disp);
        emit32(e, 0x38206800u | ((uint32_t)ARM64_SCRATCH << 16)
                 | ((uint32_t)(base & 31) << 5) | (uint32_t)(src & 31));
    }
}

/* STRH Wt, [Xn, #imm] */
static inline void emit_store16_disp(emit_t *e, int src, int base, int32_t disp) {
    if (disp >= 0 && disp <= 8190 && (disp & 1) == 0) {
        uint32_t u = (uint32_t)(disp >> 1) & 0xFFFu;
        emit32(e, 0x79000000u | (u << 10)
                 | ((uint32_t)(base & 31) << 5) | (uint32_t)(src & 31));
    } else if (disp >= -256 && disp <= 255) {
        emit_arm64_sturh(e, src, base, disp);
    } else {
        emit_mov_reg_imm64(e, ARM64_SCRATCH, (uint64_t)(int64_t)disp);
        emit32(e, 0x78206800u | ((uint32_t)ARM64_SCRATCH << 16)
                 | ((uint32_t)(base & 31) << 5) | (uint32_t)(src & 31));
    }
}

/* store imm32 at [base+disp]: materialize then store. Cannot route
 * through emit_store32_disp: its far-disp path uses ARM64_SCRATCH for
 * the displacement, which would clobber the value held there. */
static inline void emit_store32_disp_imm(emit_t *e, int base, int32_t disp, uint32_t imm) {
    emit_mov_reg_imm32(e, ARM64_SCRATCH, imm);   /* W9 = value */
    if (disp >= 0 && disp <= 16380 && (disp & 3) == 0) {
        uint32_t u = (uint32_t)(disp >> 2) & 0xFFFu;
        emit32(e, 0xB9000000u | (u << 10)
                 | ((uint32_t)(base & 31) << 5) | (uint32_t)ARM64_SCRATCH);
    } else if (disp >= -256 && disp <= 255) {
        emit_arm64_stur32(e, ARM64_SCRATCH, base, disp);
    } else {
        emit_mov_reg_imm64(e, ARM64_SCRATCH2, (uint64_t)(int64_t)disp);
        /* STR W9, [Xbase, X10] */
        emit32(e, 0xB8206800u | ((uint32_t)ARM64_SCRATCH2 << 16)
                 | ((uint32_t)(base & 31) << 5) | (uint32_t)ARM64_SCRATCH);
    }
}

/* ===== ALU reg-reg (32-bit W-form) =====
 *
 * All three-register ALU ops use the shifted-register form with
 * shift=LSL, amount=0. Arm ARM C4.1.67.
 */

/* ADD Wd, Wn, Wm — Arm ARM C6.2.4 */
static inline void emit_add_reg32(emit_t *e, int dst, int src) {
    emit32(e, 0x0B000000u | ((uint32_t)(src & 31) << 16)
             | ((uint32_t)(dst & 31) << 5) | (uint32_t)(dst & 31));
}

/* SUB Wd, Wn, Wm — Arm ARM C6.2.303 */
static inline void emit_sub_reg32(emit_t *e, int dst, int src) {
    emit32(e, 0x4B000000u | ((uint32_t)(src & 31) << 16)
             | ((uint32_t)(dst & 31) << 5) | (uint32_t)(dst & 31));
}

/* AND Wd, Wn, Wm — Arm ARM C6.2.12 */
static inline void emit_and_reg32(emit_t *e, int dst, int src) {
    emit32(e, 0x0A000000u | ((uint32_t)(src & 31) << 16)
             | ((uint32_t)(dst & 31) << 5) | (uint32_t)(dst & 31));
}

/* ORR Wd, Wn, Wm — Arm ARM C6.2.239 */
static inline void emit_or_reg32(emit_t *e, int dst, int src) {
    emit32(e, 0x2A000000u | ((uint32_t)(src & 31) << 16)
             | ((uint32_t)(dst & 31) << 5) | (uint32_t)(dst & 31));
}

/* EOR Wd, Wn, Wm — Arm ARM C6.2.75 */
static inline void emit_xor_reg32(emit_t *e, int dst, int src) {
    emit32(e, 0x4A000000u | ((uint32_t)(src & 31) << 16)
             | ((uint32_t)(dst & 31) << 5) | (uint32_t)(dst & 31));
}

/* MUL Wd, Wn, Wm — alias of MADD Wd, Wn, Wm, WZR. Arm ARM C6.2.196 */
static inline void emit_imul_reg32(emit_t *e, int dst, int src) {
    emit32(e, 0x1B007C00u | ((uint32_t)(src & 31) << 16)
             | ((uint32_t)(dst & 31) << 5) | (uint32_t)(dst & 31));
}

/* NEG Wd, Wn — alias of SUB Wd, WZR, Wn. */
static inline void emit_neg_reg32(emit_t *e, int reg) {
    emit32(e, 0x4B0003E0u | ((uint32_t)(reg & 31) << 16)
             | (uint32_t)(reg & 31));
}

/* CMP Wn, Wm — alias of SUBS WZR, Wn, Wm. Arm ARM C6.2.58 */
static inline void emit_cmp_reg32(emit_t *e, int a, int b) {
    emit32(e, 0x6B00001Fu | ((uint32_t)(b & 31) << 16)
             | ((uint32_t)(a & 31) << 5));
}

/* TST Wn, Wm — alias of ANDS WZR, Wn, Wm. Arm ARM C6.2.344 */
static inline void emit_test_reg32(emit_t *e, int a, int b) {
    emit32(e, 0x6A00001Fu | ((uint32_t)(b & 31) << 16)
             | ((uint32_t)(a & 31) << 5));
}

/* ===== ALU reg,imm =====
 *
 * ARM64 has no generic "ADD Wd, Wn, #imm32". The immediate form is
 * 12-bit (optionally shifted left 12). Anything bigger must be
 * materialized through the scratch register then folded with the
 * reg-reg form.
 */

static inline void emit_arm64_add_imm12(emit_t *e, int is64, int rd, int rn,
                                        uint32_t imm12, int sh) {
    uint32_t base = is64 ? 0x91000000u : 0x11000000u;
    emit32(e, base | ((uint32_t)(sh ? 1 : 0) << 22) | ((imm12 & 0xFFFu) << 10)
             | ((uint32_t)(rn & 31) << 5) | (uint32_t)(rd & 31));
}

static inline void emit_arm64_sub_imm12(emit_t *e, int is64, int rd, int rn,
                                        uint32_t imm12, int sh) {
    uint32_t base = is64 ? 0xD1000000u : 0x51000000u;
    emit32(e, base | ((uint32_t)(sh ? 1 : 0) << 22) | ((imm12 & 0xFFFu) << 10)
             | ((uint32_t)(rn & 31) << 5) | (uint32_t)(rd & 31));
}

/* add Wd, Wd, #imm32  (signed, via scratch for large values) */
static inline void emit_add_reg32_imm32(emit_t *e, int reg, int32_t imm) {
    if (imm >= 0 && imm < 4096) {
        emit_arm64_add_imm12(e, 0, reg, reg, (uint32_t)imm, 0);
    } else if (imm < 0 && imm > -4096) {
        emit_arm64_sub_imm12(e, 0, reg, reg, (uint32_t)(-imm), 0);
    } else {
        emit_mov_reg_imm32(e, ARM64_SCRATCH, (uint32_t)imm);
        emit_add_reg32(e, reg, ARM64_SCRATCH);
    }
}

static inline void emit_sub_reg32_imm32(emit_t *e, int reg, int32_t imm) {
    if (imm >= 0 && imm < 4096) {
        emit_arm64_sub_imm12(e, 0, reg, reg, (uint32_t)imm, 0);
    } else if (imm < 0 && imm > -4096) {
        emit_arm64_add_imm12(e, 0, reg, reg, (uint32_t)(-imm), 0);
    } else {
        emit_mov_reg_imm32(e, ARM64_SCRATCH, (uint32_t)imm);
        emit_sub_reg32(e, reg, ARM64_SCRATCH);
    }
}

/* and Wd, Wd, #imm — no general immediate-bitmask encoder here.
 * Materialize into the scratch and AND the two registers. */
static inline void emit_and_reg32_imm32(emit_t *e, int reg, int32_t imm) {
    emit_mov_reg_imm32(e, ARM64_SCRATCH, (uint32_t)imm);
    emit_and_reg32(e, reg, ARM64_SCRATCH);
}

/* cmp Wn, #imm32 — uses 12-bit imm form when possible, else scratch. */
static inline void emit_cmp_reg32_imm32(emit_t *e, int reg, int32_t imm) {
    if (imm >= 0 && imm < 4096) {
        /* SUBS WZR, Wn, #imm12 — 0x7100001F */
        emit32(e, 0x7100001Fu | ((uint32_t)imm << 10)
                 | ((uint32_t)(reg & 31) << 5));
    } else if (imm < 0 && imm > -4096) {
        /* CMN alias: ADDS WZR, Wn, #-imm12 — 0x3100001F */
        emit32(e, 0x3100001Fu | ((uint32_t)(-imm) << 10)
                 | ((uint32_t)(reg & 31) << 5));
    } else {
        emit_mov_reg_imm32(e, ARM64_SCRATCH, (uint32_t)imm);
        emit_cmp_reg32(e, reg, ARM64_SCRATCH);
    }
}

/* add Xd, Xd, #imm32 (sign-extended) — used to bump 64-bit pointers. */
static inline void emit_add_reg64_imm32(emit_t *e, int reg, int32_t imm) {
    if (imm >= 0 && imm < 4096) {
        emit_arm64_add_imm12(e, 1, reg, reg, (uint32_t)imm, 0);
    } else if (imm < 0 && imm > -4096) {
        emit_arm64_sub_imm12(e, 1, reg, reg, (uint32_t)(-imm), 0);
    } else {
        emit_mov_reg_imm64(e, ARM64_SCRATCH, (uint64_t)(int64_t)imm);
        /* ADD Xd, Xd, Xm — 0x8B000000 */
        emit32(e, 0x8B000000u | ((uint32_t)ARM64_SCRATCH << 16)
                 | ((uint32_t)(reg & 31) << 5) | (uint32_t)(reg & 31));
    }
}

static inline void emit_sub_reg64_imm32(emit_t *e, int reg, int32_t imm) {
    if (imm >= 0 && imm < 4096) {
        emit_arm64_sub_imm12(e, 1, reg, reg, (uint32_t)imm, 0);
    } else if (imm < 0 && imm > -4096) {
        emit_arm64_add_imm12(e, 1, reg, reg, (uint32_t)(-imm), 0);
    } else {
        emit_mov_reg_imm64(e, ARM64_SCRATCH, (uint64_t)(int64_t)imm);
        /* SUB Xd, Xd, Xm — 0xCB000000 */
        emit32(e, 0xCB000000u | ((uint32_t)ARM64_SCRATCH << 16)
                 | ((uint32_t)(reg & 31) << 5) | (uint32_t)(reg & 31));
    }
}

/* ===== Shifts =====
 *
 * Immediate shifts on Wn are UBFM/SBFM aliases. The shift amount must
 * be in [0,31] for 32-bit forms and [0,63] for 64-bit forms.
 * Register-indexed shifts (LSLV/LSRV/ASRV) take the amount from a
 * general register. For parity with the x64 "_cl" entry points —
 * which expect the count to already be in RCX (index 1 == W1/X1) —
 * the ARM64 emitter uses W1 as the shift-count source.
 */

/* LSL Wd, Wn, #sh — alias of UBFM Wd, Wn, #(32-sh), #(31-sh). C6.2.177 */
static inline void emit_shl_reg32_imm(emit_t *e, int reg, uint8_t sh) {
    sh &= 31;
    uint32_t immr = (uint32_t)((32 - sh) & 31);
    uint32_t imms = (uint32_t)(31 - sh);
    emit32(e, 0x53000000u | (immr << 16) | (imms << 10)
             | ((uint32_t)(reg & 31) << 5) | (uint32_t)(reg & 31));
}

/* LSR Wd, Wn, #sh — alias of UBFM Wd, Wn, #sh, #31. */
static inline void emit_shr_reg32_imm(emit_t *e, int reg, uint8_t sh) {
    sh &= 31;
    emit32(e, 0x53007C00u | ((uint32_t)sh << 16)
             | ((uint32_t)(reg & 31) << 5) | (uint32_t)(reg & 31));
}

/* ASR Wd, Wn, #sh — alias of SBFM Wd, Wn, #sh, #31. */
static inline void emit_sar_reg32_imm(emit_t *e, int reg, uint8_t sh) {
    sh &= 31;
    emit32(e, 0x13007C00u | ((uint32_t)sh << 16)
             | ((uint32_t)(reg & 31) << 5) | (uint32_t)(reg & 31));
}

/* LSR Xd, Xn, #sh */
static inline void emit_shr_reg64_imm(emit_t *e, int reg, uint8_t sh) {
    sh &= 63;
    emit32(e, 0xD340FC00u | ((uint32_t)sh << 16)
             | ((uint32_t)(reg & 31) << 5) | (uint32_t)(reg & 31));
}

/* LSL Xd, Xn, #sh  — UBFM Xd, Xn, #(64-sh), #(63-sh). C6.2.177 */
static inline void emit_shl_reg64_imm(emit_t *e, int reg, uint8_t sh) {
    sh &= 63;
    uint32_t immr = (uint32_t)((64 - sh) & 63);
    uint32_t imms = (uint32_t)(63 - sh);
    emit32(e, 0xD3400000u | (immr << 16) | (imms << 10)
             | ((uint32_t)(reg & 31) << 5) | (uint32_t)(reg & 31));
}

/* LSLV Wd, Wn, Wm — Arm ARM C6.2.179. W1 holds the count. */
static inline void emit_shl_reg32_cl(emit_t *e, int reg) {
    emit32(e, 0x1AC02000u | ((uint32_t)1 << 16)
             | ((uint32_t)(reg & 31) << 5) | (uint32_t)(reg & 31));
}

/* LSRV Wd, Wn, Wm — Arm ARM C6.2.182 */
static inline void emit_shr_reg32_cl(emit_t *e, int reg) {
    emit32(e, 0x1AC02400u | ((uint32_t)1 << 16)
             | ((uint32_t)(reg & 31) << 5) | (uint32_t)(reg & 31));
}

/* ASRV Wd, Wn, Wm — Arm ARM C6.2.25 */
static inline void emit_sar_reg32_cl(emit_t *e, int reg) {
    emit32(e, 0x1AC02800u | ((uint32_t)1 << 16)
             | ((uint32_t)(reg & 31) << 5) | (uint32_t)(reg & 31));
}

/* LSRV Xd, Xn, Xm */
static inline void emit_shr_reg64_cl(emit_t *e, int reg) {
    emit32(e, 0x9AC02400u | ((uint32_t)1 << 16)
             | ((uint32_t)(reg & 31) << 5) | (uint32_t)(reg & 31));
}

/* ===== Jumps / branches =====
 *
 * x64 condition-code macros — reused by jit.c — must still be
 * available but they'll be remapped to ARM64 cond codes inside
 * emit_jcc_rel32. We intentionally redefine them to ARM64 values
 * so jit.c's `emit_jcc_rel32(e, CC_E)` etc. does the right thing.
 */
#undef CC_O
#undef CC_NO
#undef CC_B
#undef CC_AE
#undef CC_E
#undef CC_NE
#undef CC_BE
#undef CC_A
#undef CC_S
#undef CC_NS
#undef CC_L
#undef CC_GE
#undef CC_LE
#undef CC_G

/* ARM64 condition field encodings (Arm ARM C1.2.4). */
#define CC_O   0x6  /* VS — overflow set */
#define CC_NO  0x7  /* VC */
#define CC_B   0x3  /* LO — unsigned < */
#define CC_AE  0x2  /* HS — unsigned >= */
#define CC_E   0x0  /* EQ */
#define CC_NE  0x1  /* NE */
#define CC_BE  0x9  /* LS — unsigned <= */
#define CC_A   0x8  /* HI — unsigned > */
#define CC_S   0x4  /* MI — negative */
#define CC_NS  0x5  /* PL */
#define CC_L   0xB  /* LT — signed < */
#define CC_GE  0xA  /* signed >= */
#define CC_LE  0xD  /* signed <= */
#define CC_G   0xC  /* signed > */

/*
 * Branch range strategy
 * ---------------------
 * ARM64 B.cond has a ±1 MB range, which is smaller than flexe's 32 MB
 * code cache, so block chaining across the whole cache cannot use
 * B.cond directly. We use the 26-bit unconditional B (±128 MB) as the
 * patchable branch for both emit_jmp_rel32 and emit_jcc_rel32:
 *
 *   emit_jmp_rel32: one B instruction (4 bytes). Patch site = that word.
 *
 *   emit_jcc_rel32: two instructions (8 bytes):
 *       B.<inverse-cond> +8      // skip the far jump if cond false
 *       B   <patchable>          // taken path
 *   The returned patch site is the offset of the second word (the B).
 *
 * emit_patch_rel32() rewrites whatever word the site points at, using
 * the B encoding. 128 MB is comfortably larger than the 32 MB cache.
 */

/* B <imm26> — Arm ARM C6.2.32. imm26 is word-scaled signed. */
static inline uint32_t arm64_encode_b(int32_t word_off) {
    return 0x14000000u | ((uint32_t)word_off & 0x03FFFFFFu);
}

static inline int emit_jmp_rel32(emit_t *e) {
    int patch_offset = (int)(e->ptr - e->buf);
    emit32(e, arm64_encode_b(0)); /* placeholder: branch to self */
    return patch_offset;
}

static inline int emit_jcc_rel32(emit_t *e, uint8_t cc) {
    /* B.<!cc> +8 ; B <patch> — invert the condition by flipping bit 0. */
    uint32_t inv = (uint32_t)(cc ^ 1) & 0xFu;
    /* B.cond imm19 : 0x54000000 | (imm19 << 5) | cond. imm19 is word-
     * scaled; +8 means skip one word (past the following B), so imm19=2. */
    emit32(e, 0x54000000u | ((uint32_t)2 << 5) | inv);
    int patch_offset = (int)(e->ptr - e->buf);
    emit32(e, arm64_encode_b(0));
    return patch_offset;
}

/* Patch a previously-returned site so the B at that offset targets
 * the current e->ptr. The site always points at a 4-byte B word. */
static inline void emit_patch_rel32(emit_t *e, int patch_offset) {
    uint8_t *site = e->buf + patch_offset;
    int64_t byte_delta = (int64_t)(e->ptr - site);
    int32_t word_off = (int32_t)(byte_delta / 4);
    uint32_t insn = arm64_encode_b(word_off);
    memcpy(site, &insn, 4);
}

/* ===== Call / return ===== */

/* BLR Xn — Arm ARM C6.2.36 */
static inline void emit_arm64_blr(emit_t *e, int rn) {
    emit32(e, 0xD63F0000u | ((uint32_t)(rn & 31) << 5));
}

/* Call an absolute 64-bit target: materialize into scratch, BLR. */
static inline void emit_call_imm(emit_t *e, void *target) {
    emit_mov_reg_imm64(e, ARM64_SCRATCH, (uint64_t)(uintptr_t)target);
    emit_arm64_blr(e, ARM64_SCRATCH);
}

/* RET — Arm ARM C6.2.256 (uses X30/LR by default). */
static inline void emit_ret(emit_t *e) {
    emit32(e, 0xD65F03C0u);
}

/*
 * push / pop on ARM64 use SP with a pre/post-index of 16 so the
 * stack stays 16-byte aligned. We model single-register push/pop as
 * STR [SP, #-16]! / LDR [SP], #16. This wastes 8 bytes per push but
 * keeps the API identical to the x64 helpers.
 */

/* STR Xt, [SP, #-16]! — pre-index. Arm ARM C6.2.275. */
static inline void emit_push_reg64(emit_t *e, int reg) {
    /* 0xF8 0x0F 0x8F Ee: F81F8FE0 | Rt */
    emit32(e, 0xF81F8FE0u | (uint32_t)(reg & 31));
}

/* LDR Xt, [SP], #16 — post-index. */
static inline void emit_pop_reg64(emit_t *e, int reg) {
    emit32(e, 0xF84087E0u | (uint32_t)(reg & 31));
}

/* x64-compat aliases (jit.c refers to both names). */
static inline void emit_push(emit_t *e, int reg) { emit_push_reg64(e, reg); }
static inline void emit_pop (emit_t *e, int reg) { emit_pop_reg64 (e, reg); }

/* ===== Misc helpers mirrored from jit_emit_x64.h ===== */

/* NOP — Arm ARM C6.2.213 */
static inline void emit_nop(emit_t *e) {
    emit32(e, 0xD503201Fu);
}

/* cmov cc, dst, src — ARM64 CSEL Wd, Wsrc, Wdst, cc */
static inline void emit_cmov_reg32(emit_t *e, uint8_t cc, int dst, int src) {
    emit32(e, 0x1A800000u | ((uint32_t)(src & 31) << 16)
             | ((uint32_t)(cc & 0xFu) << 12)
             | ((uint32_t)(dst & 31) << 5) | (uint32_t)(dst & 31));
}

/* movsx Wd, Wn (16->32) — SXTH alias of SBFM Wd, Wn, #0, #15 */
static inline void emit_movsx_reg32_reg16(emit_t *e, int dst, int src) {
    emit32(e, 0x13003C00u | ((uint32_t)(src & 31) << 5) | (uint32_t)(dst & 31));
}

/* call register indirect */
static inline void emit_call_reg(emit_t *e, int reg) {
    emit_arm64_blr(e, reg);
}

/* lea dst, [base + disp]  == ADD Xd, Xn, #disp (with scratch fallback). */
static inline void emit_lea(emit_t *e, int dst, int base, int32_t disp) {
    if (disp >= 0 && disp < 4096) {
        emit_arm64_add_imm12(e, 1, dst, base, (uint32_t)disp, 0);
    } else if (disp < 0 && disp > -4096) {
        emit_arm64_sub_imm12(e, 1, dst, base, (uint32_t)(-disp), 0);
    } else {
        emit_mov_reg_imm64(e, ARM64_SCRATCH, (uint64_t)(int64_t)disp);
        emit32(e, 0x8B000000u | ((uint32_t)ARM64_SCRATCH << 16)
                 | ((uint32_t)(base & 31) << 5) | (uint32_t)(dst & 31));
    }
}

/* cmp Wn, [Xb + disp]  — load then compare via scratch. */
static inline void emit_cmp32_mem(emit_t *e, int reg, int base, int32_t disp) {
    emit_load32_disp(e, ARM64_SCRATCH, base, disp);
    emit_cmp_reg32(e, reg, ARM64_SCRATCH);
}

/* or Wd, Wd, #imm32 — via scratch */
static inline void emit_or_reg32_imm32(emit_t *e, int reg, int32_t imm) {
    emit_mov_reg_imm32(e, ARM64_SCRATCH, (uint32_t)imm);
    emit_or_reg32(e, reg, ARM64_SCRATCH);
}

/* MVN Wd, Wn — alias of ORN Wd, WZR, Wn */
static inline void emit_not_reg32(emit_t *e, int reg) {
    emit32(e, 0x2A2003E0u | ((uint32_t)(reg & 31) << 16)
             | (uint32_t)(reg & 31));
}

/* popcnt — ARM64 has no scalar popcount; the NEON sequence (FMOV/CNT/
 * ADDV) is overkill for jit.c's only use: POPCNT(windowstart) where the
 * value is ≤ 16 bits. SWAR in W registers instead. Result in dst. */
static inline void emit_popcnt(emit_t *e, int dst, int src) {
    if (dst != src) emit_mov_reg32_reg32(e, dst, src);
    /* dst = dst - ((dst >> 1) & 0x5555) */
    emit_mov_reg32_reg32(e, ARM64_SCRATCH, dst);
    emit_shr_reg32_imm(e, ARM64_SCRATCH, 1);
    emit_and_reg32_imm32(e, ARM64_SCRATCH, 0x5555);
    emit_sub_reg32(e, dst, ARM64_SCRATCH);
    /* dst = (dst & 0x3333) + ((dst >> 2) & 0x3333) */
    emit_mov_reg32_reg32(e, ARM64_SCRATCH, dst);
    emit_shr_reg32_imm(e, ARM64_SCRATCH, 2);
    emit_and_reg32_imm32(e, ARM64_SCRATCH, 0x3333);
    emit_and_reg32_imm32(e, dst, 0x3333);
    emit_add_reg32(e, dst, ARM64_SCRATCH);
    /* nibbles now hold 0-4; dst = dst + (dst>>4); dst = dst + (dst>>8); & 0xF */
    emit_mov_reg32_reg32(e, ARM64_SCRATCH, dst);
    emit_shr_reg32_imm(e, ARM64_SCRATCH, 4);
    emit_add_reg32(e, dst, ARM64_SCRATCH);
    emit_mov_reg32_reg32(e, ARM64_SCRATCH, dst);
    emit_shr_reg32_imm(e, ARM64_SCRATCH, 8);
    emit_add_reg32(e, dst, ARM64_SCRATCH);
    emit_and_reg32_imm32(e, dst, 0xF);
}

/* bt reg, bit_reg — sets Z=1 iff bit `bit_reg` of `reg` is CLEAR.
 * Sequence: W9 = 1 << (bit_reg & 31); TST Wreg, W9.
 * LSLV masks the shift count to 31, matching x86 BT's modulo behavior.
 * Callers that want "branch if bit clear" use CC_E; "branch if bit set"
 * use CC_NE. (The x64 version sets CF=bit and uses CC_AE/CC_B instead.) */
static inline void emit_bt_reg_reg(emit_t *e, int reg, int bit_reg) {
    /* MOV W9, #1 */
    emit32(e, 0x52800029u);
    /* LSLV W9, W9, Wbit */
    emit32(e, 0x1AC02000u | ((uint32_t)(bit_reg & 31) << 16)
             | (9u << 5) | 9u);
    /* TST Wreg, W9 — ANDS WZR, Wreg, W9 */
    emit32(e, 0x6A00001Fu | (9u << 16) | ((uint32_t)(reg & 31) << 5));
}

/* TST Xn, Xm (64-bit) — ANDS XZR, Xn, Xm. For pointer null checks. */
static inline void emit_test_reg64(emit_t *e, int a, int b) {
    emit32(e, 0xEA00001Fu | ((uint32_t)(b & 31) << 16)
             | ((uint32_t)(a & 31) << 5));
}

/* TST Wn, #imm32 — materialize mask in W9, then TST. Sets Z=1 iff
 * (Wn & imm) == 0, same flag semantics as x86 TEST reg, imm. */
static inline void emit_test_reg32_imm32(emit_t *e, int reg, uint32_t imm) {
    emit_mov_reg_imm32(e, ARM64_SCRATCH, imm);
    emit32(e, 0x6A00001Fu | ((uint32_t)ARM64_SCRATCH << 16)
             | ((uint32_t)(reg & 31) << 5));
}

/* W9 = 1 << (bit_reg & 31) — bit-mask materialization for BBC/BBS. */
static inline void emit_bt_mask(emit_t *e, int bit_reg) {
    emit32(e, 0x52800029u); /* MOV W9, #1 */
    emit32(e, 0x1AC02000u | ((uint32_t)(bit_reg & 31) << 16)
             | (9u << 5) | 9u); /* LSLV W9, W9, Wbit */
}

/* CSET Wd, cc — Wd = cc ? 1 : 0. Alias of CSINC Wd, WZR, WZR, !cc. */
static inline void emit_cset_reg32(emit_t *e, uint8_t cc, int dst) {
    emit32(e, 0x1A9F07E0u | ((uint32_t)((cc ^ 1) & 0xF) << 12)
             | (uint32_t)(dst & 31));
}

/* ORR Xd, Xd, Xm (64-bit OR) */
static inline void emit_or_reg64(emit_t *e, int dst, int src) {
    emit32(e, 0xAA000000u | ((uint32_t)(src & 31) << 16)
             | ((uint32_t)(dst & 31) << 5) | (uint32_t)(dst & 31));
}

/* ASR Xd, Xn, #sh — alias of SBFM Xd, Xn, #sh, #63. */
static inline void emit_sar_reg64_imm(emit_t *e, int reg, uint8_t sh) {
    sh &= 63;
    emit32(e, 0x93407C00u | ((uint32_t)sh << 16)
             | ((uint32_t)(reg & 31) << 5) | (uint32_t)(reg & 31));
}

/* (uint32)a * (uint32)b >> 32 — UMULL Xd, Wa, Wb; LSR Xd, #32. */
static inline void emit_umulh32(emit_t *e, int dst, int a, int b) {
    emit32(e, 0x9BA07C00u | ((uint32_t)(b & 31) << 16)
             | ((uint32_t)(a & 31) << 5) | (uint32_t)(dst & 31));
    emit_shr_reg64_imm(e, dst, 32);
}

/* (int32)a * (int32)b >> 32 — SMULL Xd, Wa, Wb; ASR Xd, #32. */
static inline void emit_smulh32(emit_t *e, int dst, int a, int b) {
    emit32(e, 0x9B207C00u | ((uint32_t)(b & 31) << 16)
             | ((uint32_t)(a & 31) << 5) | (uint32_t)(dst & 31));
    emit_sar_reg64_imm(e, dst, 32);
}

/* 64-bit load from a table of pointers: dst = *(base + disp + idx32*8).
 * idx32 is a 32-bit register (upper bits guaranteed zero by W ops). */
static inline void emit_load64_index(emit_t *e, int dst, int base,
                                     int idx32, int32_t disp) {
    if (disp >= 0 && disp < 4096) {
        emit_arm64_add_imm12(e, 1, ARM64_SCRATCH, base, (uint32_t)disp, 0);
    } else {
        emit_mov_reg_imm64(e, ARM64_SCRATCH, (uint64_t)(int64_t)disp);
        emit32(e, 0x8B000000u | ((uint32_t)ARM64_SCRATCH << 16)
                 | ((uint32_t)(base & 31) << 5) | (uint32_t)ARM64_SCRATCH);
    }
    /* LDR Xt, [Xscratch, Widx, UXTW #3] — 0xF8605800 */
    emit32(e, 0xF8605800u | ((uint32_t)(idx32 & 31) << 16)
             | ((uint32_t)ARM64_SCRATCH << 5) | (uint32_t)(dst & 31));
}

/* Guest-width loads/stores at [base64 + off64] (register offset, no shift) */
static inline void emit_load32_rof(emit_t *e, int dst, int base, int idx) {
    emit32(e, 0xB8606800u | ((uint32_t)(idx & 31) << 16)
             | ((uint32_t)(base & 31) << 5) | (uint32_t)(dst & 31));
}
static inline void emit_store32_rof(emit_t *e, int src, int base, int idx) {
    emit32(e, 0xB8206800u | ((uint32_t)(idx & 31) << 16)
             | ((uint32_t)(base & 31) << 5) | (uint32_t)(src & 31));
}
static inline void emit_load8u_rof(emit_t *e, int dst, int base, int idx) {
    emit32(e, 0x38606800u | ((uint32_t)(idx & 31) << 16)
             | ((uint32_t)(base & 31) << 5) | (uint32_t)(dst & 31));
}
static inline void emit_store8_rof(emit_t *e, int src, int base, int idx) {
    emit32(e, 0x38206800u | ((uint32_t)(idx & 31) << 16)
             | ((uint32_t)(base & 31) << 5) | (uint32_t)(src & 31));
}
static inline void emit_load16u_rof(emit_t *e, int dst, int base, int idx) {
    emit32(e, 0x78606800u | ((uint32_t)(idx & 31) << 16)
             | ((uint32_t)(base & 31) << 5) | (uint32_t)(dst & 31));
}
static inline void emit_load16s_rof(emit_t *e, int dst, int base, int idx) {
    /* LDRSH Wt, [Xn, Xm] — sign-extend 16→32 */
    emit32(e, 0x78A06800u | ((uint32_t)(idx & 31) << 16)
             | ((uint32_t)(base & 31) << 5) | (uint32_t)(dst & 31));
}
static inline void emit_store16_rof(emit_t *e, int src, int base, int idx) {
    emit32(e, 0x78206800u | ((uint32_t)(idx & 31) << 16)
             | ((uint32_t)(base & 31) << 5) | (uint32_t)(src & 31));
}

/* or [base + disp], src  — load, or, store. */
static inline void emit_or_mem32_reg(emit_t *e, int base, int32_t disp, int src) {
    emit_load32_disp(e, ARM64_SCRATCH, base, disp);
    emit32(e, 0x2A000000u | ((uint32_t)(src & 31) << 16)
             | ((uint32_t)ARM64_SCRATCH << 5) | (uint32_t)ARM64_SCRATCH);
    emit_store32_disp(e, ARM64_SCRATCH, base, disp);
}

/* and [base + disp], src */
static inline void emit_and_mem32_reg(emit_t *e, int base, int32_t disp, int src) {
    emit_load32_disp(e, ARM64_SCRATCH, base, disp);
    emit32(e, 0x0A000000u | ((uint32_t)(src & 31) << 16)
             | ((uint32_t)ARM64_SCRATCH << 5) | (uint32_t)ARM64_SCRATCH);
    emit_store32_disp(e, ARM64_SCRATCH, base, disp);
}

/* Direct jmp to an absolute target pointer. */
static inline void emit_jmp_rel32_to(emit_t *e, uint8_t *target) {
    int64_t byte_delta = (int64_t)(target - e->ptr);
    int32_t word_off = (int32_t)(byte_delta / 4);
    emit32(e, arm64_encode_b(word_off));
}

/* SIB-style helpers from the x64 header.  ARM64 has LDR Wt, [Xn, Xm,
 * LSL #2]; we hardcode scale=2 (×4) to match emit_load32_sib's x64
 * behavior. Only used by a couple of jit.c table-lookup paths. */
static inline void emit_load32_sib(emit_t *e, int dst, int base, int index, int32_t disp) {
    if (disp != 0) {
        /* Fold disp into a temporary: scratch = base + disp */
        emit_mov_reg_imm64(e, ARM64_SCRATCH, (uint64_t)(int64_t)disp);
        emit32(e, 0x8B000000u | ((uint32_t)ARM64_SCRATCH << 16)
                 | ((uint32_t)(base & 31) << 5) | (uint32_t)ARM64_SCRATCH);
        base = ARM64_SCRATCH;
    }
    /* LDR Wt, [Xn, Xm, LSL #2]  — 0xB8607800 | Xm<<16 | Xn<<5 | Wt */
    emit32(e, 0xB8607800u | ((uint32_t)(index & 31) << 16)
             | ((uint32_t)(base & 31) << 5) | (uint32_t)(dst & 31));
}

static inline void emit_store32_sib(emit_t *e, int src, int base, int index, int32_t disp) {
    if (disp != 0) {
        emit_mov_reg_imm64(e, ARM64_SCRATCH, (uint64_t)(int64_t)disp);
        emit32(e, 0x8B000000u | ((uint32_t)ARM64_SCRATCH << 16)
                 | ((uint32_t)(base & 31) << 5) | (uint32_t)ARM64_SCRATCH);
        base = ARM64_SCRATCH;
    }
    /* STR Wt, [Xn, Xm, LSL #2]  — 0xB8207800 */
    emit32(e, 0xB8207800u | ((uint32_t)(index & 31) << 16)
             | ((uint32_t)(base & 31) << 5) | (uint32_t)(src & 31));
}

/* ===== Optional self-test =====
 *
 * Compile a .c file that #defines ARM64_EMIT_SELFTEST before including
 * this header, and run it. Proves that at least one emitted instruction
 * executes natively on the host.
 */
#ifdef ARM64_EMIT_SELFTEST
#include <stdio.h>
#include <stdlib.h>
#include <sys/mman.h>
#ifdef __APPLE__
#include <pthread.h>
#include <libkern/OSCacheControl.h>
#endif

static int arm64_emit_selftest(void) {
    size_t sz = 4096;
#ifdef __APPLE__
    void *page = mmap(NULL, sz, PROT_READ | PROT_WRITE | PROT_EXEC,
                      MAP_PRIVATE | MAP_ANON | MAP_JIT, -1, 0);
#else
    void *page = mmap(NULL, sz, PROT_READ | PROT_WRITE | PROT_EXEC,
                      MAP_PRIVATE | MAP_ANON, -1, 0);
#endif
    if (page == MAP_FAILED) { perror("mmap"); return 1; }

#ifdef __APPLE__
    pthread_jit_write_protect_np(0);
#endif

    emit_t e;
    emit_init(&e, (uint8_t *)page, sz);

    /* mov w0, #42 ; ret */
    emit_mov_reg_imm32(&e, RAX /* X0 */, 42);
    emit_ret(&e);

    /* Second program: add a small routine exercising more ops. */
    uint8_t *prog2 = e.ptr;
    emit_mov_reg_imm32(&e, RAX, 100);
    emit_mov_reg_imm32(&e, RCX, 23);
    emit_sub_reg32(&e, RAX, RCX);          /* w0 = 77 */
    emit_add_reg32_imm32(&e, RAX, 5);      /* w0 = 82 */
    emit_shl_reg32_imm(&e, RAX, 1);        /* w0 = 164 */
    emit_ret(&e);

    /* Third program: conditional branch sanity. cmp w0,#0 + bne +4
     * skipping a mov w0,#0 so we return 7 regardless of start. */
    uint8_t *prog3 = e.ptr;
    emit_mov_reg_imm32(&e, RAX, 7);
    emit_cmp_reg32_imm32(&e, RAX, 0);
    int site = emit_jcc_rel32(&e, CC_NE);
    emit_mov_reg_imm32(&e, RAX, 99);
    emit_patch_rel32(&e, site);
    emit_ret(&e);

#ifdef __APPLE__
    pthread_jit_write_protect_np(1);
    sys_icache_invalidate(page, sz);
#endif

    int (*fn1)(void) = (int (*)(void))page;
    int (*fn2)(void) = (int (*)(void))prog2;
    int (*fn3)(void) = (int (*)(void))prog3;

    int r1 = fn1();
    int r2 = fn2();
    int r3 = fn3();
    printf("arm64 selftest: fn1=%d (want 42), fn2=%d (want 164), fn3=%d (want 7)\n",
           r1, r2, r3);
    return (r1 == 42 && r2 == 164 && r3 == 7) ? 0 : 1;
}

int main(void) { return arm64_emit_selftest(); }
#endif /* ARM64_EMIT_SELFTEST */

#endif /* JIT_EMIT_ARM64_H */
