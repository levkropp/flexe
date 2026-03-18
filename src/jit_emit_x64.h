#ifndef JIT_EMIT_X64_H
#define JIT_EMIT_X64_H

/*
 * x86-64 machine code emitter for the Xtensa JIT compiler.
 * All functions are static inline for maximum performance.
 *
 * Register allocation:
 *   r15  = xtensa_cpu_t *cpu  (callee-saved, set in prologue)
 *   r14  = cpu->mem           (callee-saved, set in prologue)
 *   r13d = windowbase * 4     (callee-saved, constant per block)
 *   rbx, rbp, r12 = available scratch (callee-saved)
 *   rax, rcx, rdx, rsi, rdi = scratch / C call args
 */

#include <stdint.h>
#include <string.h>
#include <stddef.h>

/* x86-64 register encodings */
enum {
    RAX = 0, RCX = 1, RDX = 2, RBX = 3,
    RSP = 4, RBP = 5, RSI = 6, RDI = 7,
    R8  = 8, R9  = 9, R10 = 10, R11 = 11,
    R12 = 12, R13 = 13, R14 = 14, R15 = 15
};

/* Emitter context */
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

/* Raw byte emitters */
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

/* REX prefix generation */
static inline uint8_t rex(int w, int r, int x, int b) {
    return (uint8_t)(0x40 | (w << 3) | (r << 2) | (x << 1) | b);
}

static inline void emit_rex(emit_t *e, int w, int reg, int rm) {
    uint8_t r = rex(w, (reg >> 3) & 1, 0, (rm >> 3) & 1);
    if (r != 0x40 || w) emit8(e, r);
}

static inline void emit_rex_w(emit_t *e, int reg, int rm) {
    emit8(e, rex(1, (reg >> 3) & 1, 0, (rm >> 3) & 1));
}

/* ModRM byte */
static inline uint8_t modrm(int mod, int reg, int rm) {
    return (uint8_t)((mod << 6) | ((reg & 7) << 3) | (rm & 7));
}

/* SIB byte */
static inline uint8_t sib(int scale, int index, int base) {
    return (uint8_t)((scale << 6) | ((index & 7) << 3) | (base & 7));
}

/* ===== MOV instructions ===== */

/* mov reg64, imm64 (movabs) */
static inline void emit_mov_reg_imm64(emit_t *e, int reg, uint64_t imm) {
    emit8(e, rex(1, 0, 0, (reg >> 3) & 1));
    emit8(e, (uint8_t)(0xB8 | (reg & 7)));
    emit64(e, imm);
}

/* mov reg32, imm32 */
static inline void emit_mov_reg_imm32(emit_t *e, int reg, uint32_t imm) {
    if (reg >= 8) emit8(e, rex(0, 0, 0, 1));
    emit8(e, (uint8_t)(0xB8 | (reg & 7)));
    emit32(e, imm);
}

/* mov reg64, reg64 */
static inline void emit_mov_reg_reg(emit_t *e, int dst, int src) {
    emit_rex_w(e, src, dst);
    emit8(e, 0x89);
    emit8(e, modrm(3, src, dst));
}

/* mov reg32, reg32 */
static inline void emit_mov_reg32_reg32(emit_t *e, int dst, int src) {
    emit_rex(e, 0, src, dst);
    emit8(e, 0x89);
    emit8(e, modrm(3, src, dst));
}

/* mov reg32, [base64 + disp32] */
static inline void emit_load32_disp(emit_t *e, int dst, int base, int32_t disp) {
    emit_rex(e, 0, dst, base);
    emit8(e, 0x8B);
    if ((base & 7) == RSP) {
        /* Need SIB byte for RSP-based addressing */
        emit8(e, modrm(2, dst, RSP));
        emit8(e, sib(0, RSP, RSP));
    } else {
        emit8(e, modrm(2, dst, base));
    }
    emit32(e, (uint32_t)disp);
}

/* mov [base64 + disp32], reg32 */
static inline void emit_store32_disp(emit_t *e, int src, int base, int32_t disp) {
    emit_rex(e, 0, src, base);
    emit8(e, 0x89);
    if ((base & 7) == RSP) {
        emit8(e, modrm(2, src, RSP));
        emit8(e, sib(0, RSP, RSP));
    } else {
        emit8(e, modrm(2, src, base));
    }
    emit32(e, (uint32_t)disp);
}

/* mov reg64, [base64 + disp32] (64-bit load) */
static inline void emit_load64_disp(emit_t *e, int dst, int base, int32_t disp) {
    emit_rex_w(e, dst, base);
    emit8(e, 0x8B);
    if ((base & 7) == RSP) {
        emit8(e, modrm(2, dst, RSP));
        emit8(e, sib(0, RSP, RSP));
    } else {
        emit8(e, modrm(2, dst, base));
    }
    emit32(e, (uint32_t)disp);
}

/* mov [base64 + disp32], reg64 (64-bit store) */
static inline void emit_store64_disp(emit_t *e, int src, int base, int32_t disp) {
    emit_rex_w(e, src, base);
    emit8(e, 0x89);
    if ((base & 7) == RSP) {
        emit8(e, modrm(2, src, RSP));
        emit8(e, sib(0, RSP, RSP));
    } else {
        emit8(e, modrm(2, src, base));
    }
    emit32(e, (uint32_t)disp);
}

/* movzx reg32, byte [base64 + disp32] */
static inline void emit_load8u_disp(emit_t *e, int dst, int base, int32_t disp) {
    emit_rex(e, 0, dst, base);
    emit8(e, 0x0F);
    emit8(e, 0xB6);
    if ((base & 7) == RSP) {
        emit8(e, modrm(2, dst, RSP));
        emit8(e, sib(0, RSP, RSP));
    } else {
        emit8(e, modrm(2, dst, base));
    }
    emit32(e, (uint32_t)disp);
}

/* movzx reg32, word [base64 + disp32] */
static inline void emit_load16u_disp(emit_t *e, int dst, int base, int32_t disp) {
    emit_rex(e, 0, dst, base);
    emit8(e, 0x0F);
    emit8(e, 0xB7);
    if ((base & 7) == RSP) {
        emit8(e, modrm(2, dst, RSP));
        emit8(e, sib(0, RSP, RSP));
    } else {
        emit8(e, modrm(2, dst, base));
    }
    emit32(e, (uint32_t)disp);
}

/* movsx reg32, word [base64 + disp32] */
static inline void emit_load16s_disp(emit_t *e, int dst, int base, int32_t disp) {
    emit_rex(e, 0, dst, base);
    emit8(e, 0x0F);
    emit8(e, 0xBF);
    if ((base & 7) == RSP) {
        emit8(e, modrm(2, dst, RSP));
        emit8(e, sib(0, RSP, RSP));
    } else {
        emit8(e, modrm(2, dst, base));
    }
    emit32(e, (uint32_t)disp);
}

/* mov byte [base64 + disp32], reg8 */
static inline void emit_store8_disp(emit_t *e, int src, int base, int32_t disp) {
    /* Need REX if src >= 4 (SPL, BPL, SIL, DIL) or if high regs */
    emit_rex(e, 0, src, base);
    emit8(e, 0x88);
    if ((base & 7) == RSP) {
        emit8(e, modrm(2, src, RSP));
        emit8(e, sib(0, RSP, RSP));
    } else {
        emit8(e, modrm(2, src, base));
    }
    emit32(e, (uint32_t)disp);
}

/* mov word [base64 + disp32], reg16 */
static inline void emit_store16_disp(emit_t *e, int src, int base, int32_t disp) {
    emit8(e, 0x66);  /* operand size override */
    emit_rex(e, 0, src, base);
    emit8(e, 0x89);
    if ((base & 7) == RSP) {
        emit8(e, modrm(2, src, RSP));
        emit8(e, sib(0, RSP, RSP));
    } else {
        emit8(e, modrm(2, src, base));
    }
    emit32(e, (uint32_t)disp);
}

/* ===== ALU reg,reg (32-bit) ===== */

/* add reg32, reg32 */
static inline void emit_add_reg32(emit_t *e, int dst, int src) {
    emit_rex(e, 0, src, dst);
    emit8(e, 0x01);
    emit8(e, modrm(3, src, dst));
}

/* sub reg32, reg32 */
static inline void emit_sub_reg32(emit_t *e, int dst, int src) {
    emit_rex(e, 0, src, dst);
    emit8(e, 0x29);
    emit8(e, modrm(3, src, dst));
}

/* and reg32, reg32 */
static inline void emit_and_reg32(emit_t *e, int dst, int src) {
    emit_rex(e, 0, src, dst);
    emit8(e, 0x21);
    emit8(e, modrm(3, src, dst));
}

/* or reg32, reg32 */
static inline void emit_or_reg32(emit_t *e, int dst, int src) {
    emit_rex(e, 0, src, dst);
    emit8(e, 0x09);
    emit8(e, modrm(3, src, dst));
}

/* xor reg32, reg32 */
static inline void emit_xor_reg32(emit_t *e, int dst, int src) {
    emit_rex(e, 0, src, dst);
    emit8(e, 0x31);
    emit8(e, modrm(3, src, dst));
}

/* imul reg32, reg32 */
static inline void emit_imul_reg32(emit_t *e, int dst, int src) {
    emit_rex(e, 0, dst, src);
    emit8(e, 0x0F);
    emit8(e, 0xAF);
    emit8(e, modrm(3, dst, src));
}

/* cmp reg32, reg32 */
static inline void emit_cmp_reg32(emit_t *e, int a, int b) {
    emit_rex(e, 0, b, a);
    emit8(e, 0x39);
    emit8(e, modrm(3, b, a));
}

/* test reg32, reg32 */
static inline void emit_test_reg32(emit_t *e, int a, int b) {
    emit_rex(e, 0, b, a);
    emit8(e, 0x85);
    emit8(e, modrm(3, b, a));
}

/* ===== ALU reg,imm32 ===== */

/* add reg32, imm32 */
static inline void emit_add_reg32_imm32(emit_t *e, int reg, int32_t imm) {
    emit_rex(e, 0, 0, reg);
    emit8(e, 0x81);
    emit8(e, modrm(3, 0, reg));
    emit32(e, (uint32_t)imm);
}

/* sub reg32, imm32 */
static inline void emit_sub_reg32_imm32(emit_t *e, int reg, int32_t imm) {
    emit_rex(e, 0, 0, reg);
    emit8(e, 0x81);
    emit8(e, modrm(3, 5, reg));
    emit32(e, (uint32_t)imm);
}

/* and reg32, imm32 */
static inline void emit_and_reg32_imm32(emit_t *e, int reg, int32_t imm) {
    emit_rex(e, 0, 0, reg);
    emit8(e, 0x81);
    emit8(e, modrm(3, 4, reg));
    emit32(e, (uint32_t)imm);
}

/* cmp reg32, imm32 */
static inline void emit_cmp_reg32_imm32(emit_t *e, int reg, int32_t imm) {
    emit_rex(e, 0, 0, reg);
    emit8(e, 0x81);
    emit8(e, modrm(3, 7, reg));
    emit32(e, (uint32_t)imm);
}

/* add reg64, imm32 (sign-extended) */
static inline void emit_add_reg64_imm32(emit_t *e, int reg, int32_t imm) {
    emit_rex_w(e, 0, reg);
    emit8(e, 0x81);
    emit8(e, modrm(3, 0, reg));
    emit32(e, (uint32_t)imm);
}

/* sub reg64, imm32 (sign-extended) */
static inline void emit_sub_reg64_imm32(emit_t *e, int reg, int32_t imm) {
    emit_rex_w(e, 0, reg);
    emit8(e, 0x81);
    emit8(e, modrm(3, 5, reg));
    emit32(e, (uint32_t)imm);
}

/* ===== Shifts ===== */

/* shl reg32, imm8 */
static inline void emit_shl_reg32_imm(emit_t *e, int reg, uint8_t imm) {
    emit_rex(e, 0, 0, reg);
    emit8(e, 0xC1);
    emit8(e, modrm(3, 4, reg));
    emit8(e, imm);
}

/* shr reg32, imm8 */
static inline void emit_shr_reg32_imm(emit_t *e, int reg, uint8_t imm) {
    emit_rex(e, 0, 0, reg);
    emit8(e, 0xC1);
    emit8(e, modrm(3, 5, reg));
    emit8(e, imm);
}

/* sar reg32, imm8 */
static inline void emit_sar_reg32_imm(emit_t *e, int reg, uint8_t imm) {
    emit_rex(e, 0, 0, reg);
    emit8(e, 0xC1);
    emit8(e, modrm(3, 7, reg));
    emit8(e, imm);
}

/* shr reg64, imm8 */
static inline void emit_shr_reg64_imm(emit_t *e, int reg, uint8_t imm) {
    emit_rex_w(e, 0, reg);
    emit8(e, 0xC1);
    emit8(e, modrm(3, 5, reg));
    emit8(e, imm);
}

/* shl reg64, imm8 */
static inline void emit_shl_reg64_imm(emit_t *e, int reg, uint8_t imm) {
    emit_rex_w(e, 0, reg);
    emit8(e, 0xC1);
    emit8(e, modrm(3, 4, reg));
    emit8(e, imm);
}

/* shl reg32, cl */
static inline void emit_shl_reg32_cl(emit_t *e, int reg) {
    emit_rex(e, 0, 0, reg);
    emit8(e, 0xD3);
    emit8(e, modrm(3, 4, reg));
}

/* shr reg32, cl */
static inline void emit_shr_reg32_cl(emit_t *e, int reg) {
    emit_rex(e, 0, 0, reg);
    emit8(e, 0xD3);
    emit8(e, modrm(3, 5, reg));
}

/* sar reg32, cl */
static inline void emit_sar_reg32_cl(emit_t *e, int reg) {
    emit_rex(e, 0, 0, reg);
    emit8(e, 0xD3);
    emit8(e, modrm(3, 7, reg));
}

/* shr reg64, cl */
static inline void emit_shr_reg64_cl(emit_t *e, int reg) {
    emit_rex_w(e, 0, reg);
    emit8(e, 0xD3);
    emit8(e, modrm(3, 5, reg));
}

/* ===== NEG ===== */

/* neg reg32 */
static inline void emit_neg_reg32(emit_t *e, int reg) {
    emit_rex(e, 0, 0, reg);
    emit8(e, 0xF7);
    emit8(e, modrm(3, 3, reg));
}

/* ===== Jumps and branches ===== */

/* jmp rel32 — returns offset of the rel32 for patching */
static inline int emit_jmp_rel32(emit_t *e) {
    emit8(e, 0xE9);
    int patch_offset = (int)(e->ptr - e->buf);
    emit32(e, 0); /* placeholder */
    return patch_offset;
}

/* jcc rel32 — conditional jump. cc = 0x4 (E), 0x5 (NE), 0xC (L), etc. */
static inline int emit_jcc_rel32(emit_t *e, uint8_t cc) {
    emit8(e, 0x0F);
    emit8(e, (uint8_t)(0x80 | cc));
    int patch_offset = (int)(e->ptr - e->buf);
    emit32(e, 0);
    return patch_offset;
}

/* Patch a rel32 at the given offset to jump to current position */
static inline void emit_patch_rel32(emit_t *e, int patch_offset) {
    int32_t rel = (int32_t)(e->ptr - (e->buf + patch_offset + 4));
    memcpy(e->buf + patch_offset, &rel, 4);
}

/* Condition codes for jcc */
#define CC_O   0x0
#define CC_NO  0x1
#define CC_B   0x2   /* unsigned < */
#define CC_AE  0x3   /* unsigned >= */
#define CC_E   0x4   /* == */
#define CC_NE  0x5   /* != */
#define CC_BE  0x6   /* unsigned <= */
#define CC_A   0x7   /* unsigned > */
#define CC_S   0x8   /* sign */
#define CC_NS  0x9
#define CC_L   0xC   /* signed < */
#define CC_GE  0xD   /* signed >= */
#define CC_LE  0xE   /* signed <= */
#define CC_G   0xF   /* signed > */

/* ===== Stack / call ===== */

/* push reg64 */
static inline void emit_push(emit_t *e, int reg) {
    if (reg >= 8) emit8(e, rex(0, 0, 0, 1));
    emit8(e, (uint8_t)(0x50 | (reg & 7)));
}

/* pop reg64 */
static inline void emit_pop(emit_t *e, int reg) {
    if (reg >= 8) emit8(e, rex(0, 0, 0, 1));
    emit8(e, (uint8_t)(0x58 | (reg & 7)));
}

/* ret */
static inline void emit_ret(emit_t *e) {
    emit8(e, 0xC3);
}

/* call reg64 */
static inline void emit_call_reg(emit_t *e, int reg) {
    if (reg >= 8) emit8(e, rex(0, 0, 0, 1));
    emit8(e, 0xFF);
    emit8(e, modrm(3, 2, reg));
}

/* lea reg64, [base + disp32] */
static inline void emit_lea(emit_t *e, int dst, int base, int32_t disp) {
    emit_rex_w(e, dst, base);
    emit8(e, 0x8D);
    if ((base & 7) == RSP) {
        emit8(e, modrm(2, dst, RSP));
        emit8(e, sib(0, RSP, RSP));
    } else {
        emit8(e, modrm(2, dst, base));
    }
    emit32(e, (uint32_t)disp);
}

/* ===== Misc ===== */

/* nop */
static inline void emit_nop(emit_t *e) {
    emit8(e, 0x90);
}

/* cmove reg32, reg32 (conditional move if equal) */
static inline void emit_cmov_reg32(emit_t *e, uint8_t cc, int dst, int src) {
    emit_rex(e, 0, dst, src);
    emit8(e, 0x0F);
    emit8(e, (uint8_t)(0x40 | cc));
    emit8(e, modrm(3, dst, src));
}

/* mov [base64 + disp32], imm32 */
static inline void emit_store32_disp_imm(emit_t *e, int base, int32_t disp, uint32_t imm) {
    emit_rex(e, 0, 0, base);
    emit8(e, 0xC7);
    if ((base & 7) == RSP) {
        emit8(e, modrm(2, 0, RSP));
        emit8(e, sib(0, RSP, RSP));
    } else {
        emit8(e, modrm(2, 0, base));
    }
    emit32(e, (uint32_t)disp);
    emit32(e, imm);
}

/* movsx eax, ax (sign extend 16→32) — movsxd variant */
static inline void emit_movsx_reg32_reg16(emit_t *e, int dst, int src) {
    emit_rex(e, 0, dst, src);
    emit8(e, 0x0F);
    emit8(e, 0xBF);
    emit8(e, modrm(3, dst, src));
}

/* ===== Memory access with SIB: [base + index*scale + disp] ===== */

/* mov reg32, [base + index*4 + disp32] */
static inline void emit_load32_sib(emit_t *e, int dst, int base, int index, int32_t disp) {
    emit_rex(e, 0, dst, base | ((index >> 3) << 1));
    /* Actually need proper REX with X bit for index */
    /* Redo: REX.W=0, REX.R=dst>>3, REX.X=index>>3, REX.B=base>>3 */
    e->ptr--; /* back up the incorrect rex */
    uint8_t r = (uint8_t)(0x40 | ((dst >> 3) << 2) | ((index >> 3) << 1) | ((base >> 3)));
    if (r != 0x40) emit8(e, r);
    emit8(e, 0x8B);
    emit8(e, modrm(2, dst, 4)); /* mod=10, rm=100 (SIB) */
    emit8(e, sib(2, index, base)); /* scale=4, index, base */
    emit32(e, (uint32_t)disp);
}

/* mov [base + index*4 + disp32], reg32 */
static inline void emit_store32_sib(emit_t *e, int src, int base, int index, int32_t disp) {
    uint8_t r = (uint8_t)(0x40 | ((src >> 3) << 2) | ((index >> 3) << 1) | ((base >> 3)));
    if (r != 0x40) emit8(e, r);
    emit8(e, 0x89);
    emit8(e, modrm(2, src, 4));
    emit8(e, sib(2, index, base));
    emit32(e, (uint32_t)disp);
}

#endif /* JIT_EMIT_X64_H */
