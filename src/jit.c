#ifdef _MSC_VER
/* JIT is x86-64 only, disable on MSVC for now */
#else

#include "jit.h"
#include "jit_emit_x64.h"
#include "memory.h"
#include "rom_stubs.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <sys/mman.h>

/* ===== CPU struct field offsets (computed from xtensa_cpu_t layout) ===== */
/* These must match the struct in xtensa.h exactly */
#define CPU_OFF_AR          offsetof(xtensa_cpu_t, ar)
#define CPU_OFF_PC          offsetof(xtensa_cpu_t, pc)
#define CPU_OFF_CCOUNT      offsetof(xtensa_cpu_t, ccount)
#define CPU_OFF_NEXT_TIMER  offsetof(xtensa_cpu_t, next_timer_event)
#define CPU_OFF_WINDOWBASE  offsetof(xtensa_cpu_t, windowbase)
#define CPU_OFF_PS          offsetof(xtensa_cpu_t, ps)
#define CPU_OFF_SAR         offsetof(xtensa_cpu_t, sar)
#define CPU_OFF_LBEG        offsetof(xtensa_cpu_t, lbeg)
#define CPU_OFF_LEND        offsetof(xtensa_cpu_t, lend)
#define CPU_OFF_LCOUNT      offsetof(xtensa_cpu_t, lcount)
#define CPU_OFF_INTENABLE   offsetof(xtensa_cpu_t, intenable)
#define CPU_OFF_INTERRUPT   offsetof(xtensa_cpu_t, interrupt)
#define CPU_OFF_BR          offsetof(xtensa_cpu_t, br)
#define CPU_OFF_RUNNING     offsetof(xtensa_cpu_t, running)
#define CPU_OFF_HALTED      offsetof(xtensa_cpu_t, halted)
#define CPU_OFF_EXCEPTION   offsetof(xtensa_cpu_t, exception)
#define CPU_OFF_PC_WRITTEN  offsetof(xtensa_cpu_t, _pc_written)
#define CPU_OFF_IRQ_CHECK   offsetof(xtensa_cpu_t, irq_check)
#define CPU_OFF_CYCLE_COUNT offsetof(xtensa_cpu_t, cycle_count)
#define CPU_OFF_MEM         offsetof(xtensa_cpu_t, mem)
#define CPU_OFF_PC_HOOK     offsetof(xtensa_cpu_t, pc_hook)
#define CPU_OFF_PC_HOOK_CTX offsetof(xtensa_cpu_t, pc_hook_ctx)
#define CPU_OFF_HOOK_BITMAP offsetof(xtensa_cpu_t, pc_hook_bitmap)
#define CPU_OFF_PREDECODE   offsetof(xtensa_cpu_t, predecode)
#define CPU_OFF_WINDOWSTART offsetof(xtensa_cpu_t, windowstart)
#define CPU_OFF_MISC        offsetof(xtensa_cpu_t, misc)
#define CPU_OFF_EPC         offsetof(xtensa_cpu_t, epc)
#define CPU_OFF_EXCSAVE     offsetof(xtensa_cpu_t, excsave)
#define CPU_OFF_SCOMPARE1   offsetof(xtensa_cpu_t, scompare1)
#define CPU_OFF_LITBASE     offsetof(xtensa_cpu_t, litbase)
#define CPU_OFF_ACCLO       offsetof(xtensa_cpu_t, acclo)
#define CPU_OFF_ACCHI       offsetof(xtensa_cpu_t, acchi)
#define CPU_OFF_MR          offsetof(xtensa_cpu_t, mr)
#define CPU_OFF_CCOMPARE    offsetof(xtensa_cpu_t, ccompare)
#define CPU_OFF_VECBASE     offsetof(xtensa_cpu_t, vecbase)
#define CPU_OFF_EXCCAUSE    offsetof(xtensa_cpu_t, exccause)
#define CPU_OFF_EXCVADDR    offsetof(xtensa_cpu_t, excvaddr)

/* Memory struct offsets */
#define MEM_OFF_PAGE_TABLE  offsetof(xtensa_mem_t, page_table)

/* JIT state */
struct jit_state {
    /* Code cache */
    uint8_t    *code_cache;      /* mmap'd executable region */
    size_t      code_size;       /* Current usage */
    size_t      code_capacity;   /* Total capacity */

    /* Block hash table */
    jit_block_t hash[JIT_HASH_SIZE];

    /* Statistics */
    jit_stats_t stats;

    /* Verification mode */
    bool verify;

    /* Hook chaining: JIT installs itself as a pc_hook, forwarding
     * non-JIT addresses to the original (ROM stubs) hook. */
    xtensa_pc_hook_fn original_hook;
    void             *original_hook_ctx;

    /* JIT-specific hook bitmap: set bits for compiled block PCs.
     * Merged with the ROM stub bitmap for the interpreter's fast path. */
    uint64_t jit_bitmap[HOOK_BITMAP_WORDS];

    /* Block chaining */
    uint8_t      *epilogue_stub;       /* shared pop/ret stub in code cache */
    uint8_t      *last_chain_entry;    /* chain entry of last compiled block */
    chain_slot_t  chain_slots[MAX_CHAIN_SLOTS];
    int           chain_slot_count;
};

/* ===== Hash table operations ===== */

/* Hash key combines PC and windowbase so each (pc, wb) pair gets its own slot.
 * This eliminates the runtime windowbase guard — blocks are compiled for a
 * specific windowbase and only found when that windowbase is active. */
static inline uint32_t jit_hash_key(uint32_t pc, uint32_t wb) {
    return ((pc >> 2) ^ (wb * 2654435761u)) & JIT_HASH_MASK;
}

/* Combined tag for collision detection: pack wb into unused high bits of PC.
 * ESP32 PCs are 0x4000xxxx-0x404xxxxx, so bits 31:27 = 01000. We can pack
 * wb (0-15) into bits 31:28 safely by XORing. */
static inline uint32_t jit_make_tag(uint32_t pc, uint32_t wb) {
    return pc ^ (wb << 28);
}

static jit_block_t *jit_lookup(jit_state_t *jit, uint32_t pc, uint32_t wb) {
    uint32_t idx = jit_hash_key(pc, wb);
    uint32_t tag = jit_make_tag(pc, wb);
    jit_block_t *b = &jit->hash[idx];
    if (b->code && b->pc == tag)
        return b;
    return NULL;
}

static jit_block_t *jit_get_or_create(jit_state_t *jit, uint32_t pc, uint32_t wb) {
    uint32_t idx = jit_hash_key(pc, wb);
    uint32_t tag = jit_make_tag(pc, wb);
    jit_block_t *b = &jit->hash[idx];
    if (b->pc != tag) {
        /* Empty slot or collision with different PC/wb — reset */
        b->pc = tag;
        b->code = NULL;
        b->exec_count = 0;
        b->guest_insns = 0;
        b->flags = 0;
    }
    return b;
}

/* ===== Instruction fetch for block scanning ===== */

static int jit_fetch(xtensa_cpu_t *cpu, uint32_t addr, uint32_t *insn_out) {
    /* Use predecode table if available */
#if PREDECODE_SIZE > 0
    if (cpu->predecode) {
        uint32_t off = addr - PREDECODE_BASE;
        if (off < PREDECODE_SIZE) {
            uint32_t packed = cpu->predecode[off];
            if (packed) {
                *insn_out = PREDECODE_INSN(packed);
                return (int)PREDECODE_ILEN(packed);
            }
        }
    }
#endif
    /* Fall back to direct memory fetch */
    uint8_t *page = cpu->mem->page_table[addr >> 12];
    if (!page) return 0;
    const uint8_t *ptr = page + (addr & 0xFFF);
    if (ptr[0] & 0x8) {
        *insn_out = ptr[0] | ((uint32_t)ptr[1] << 8);
        return 2;
    } else {
        *insn_out = ptr[0] | ((uint32_t)ptr[1] << 8) | ((uint32_t)ptr[2] << 16);
        return 3;
    }
}

/* ===== Block scanning: determine block boundaries ===== */

typedef struct {
    uint32_t insns[JIT_MAX_BLOCK_INSNS];
    uint32_t pcs[JIT_MAX_BLOCK_INSNS];
    int      ilens[JIT_MAX_BLOCK_INSNS];
    int      count;
    uint32_t end_pc;   /* PC after last instruction */
} jit_scan_t;

/* Check if a PC is a stub hook address */
static int is_hook_addr(xtensa_cpu_t *cpu, uint32_t pc) {
    if (!cpu->pc_hook_bitmap) return 0;
    return rom_stubs_hook_bitmap_test(cpu->pc_hook_bitmap, pc);
}

/* Classify instruction: can it be JIT-compiled?
 * Returns: 0 = compilable, 1 = block terminator (branch), 2 = fallback (end block) */
static int classify_for_jit(uint32_t insn, int ilen) {
    if (ilen == 2) {
        int op0 = insn & 0xF;
        switch (op0) {
        case 0x8:  /* L32I.N */
        case 0x9:  /* S32I.N */
        case 0xA:  /* ADD.N */
        case 0xB:  /* ADDI.N */
            return 0;
        case 0xC: {
            int t_hi = ((insn >> 4) & 0xF) >> 2;
            if (t_hi < 2) return 0;  /* MOVI.N */
            return 1;  /* BEQZ.N / BNEZ.N — block terminators */
        }
        case 0xD: {
            int r = (insn >> 12) & 0xF;
            if (r == 0) return 0;  /* MOV.N */
            if (r == 15) {
                int t = (insn >> 4) & 0xF;
                if (t == 0) return 1;  /* RET.N — terminator */
                if (t == 1) return 1;  /* RETW.N — block terminator */
                if (t == 3) return 0;  /* NOP.N */
                return 2;  /* BREAK.N, ILL.N — fallback */
            }
            return 2;  /* unknown narrow */
        }
        default: return 2;
        }
    }

    /* 24-bit instruction */
    int op0 = insn & 0xF;
    switch (op0) {
    case 0: { /* QRST */
        int op1 = (insn >> 16) & 0xF;
        int op2 = (insn >> 20) & 0xF;
        int r = (insn >> 12) & 0xF;
        switch (op1) {
        case 0: /* RST0 */
            switch (op2) {
            case 0: { /* ST0 specials */
                int m = (insn >> 6) & 3;
                int nn = (insn >> 4) & 3;
                if (r == 0) {
                    if (m == 2 && nn == 0) return 1;  /* RET */
                    if (m == 2 && nn == 1) return 1;  /* RETW — block terminator */
                    if (m == 2 && nn == 2) return 1;  /* JX — terminator */
                    if (m == 3) return 2;  /* CALLX — fallback */
                    return 2;
                }
                if (r == 1) return 2;  /* MOVSP — complex */
                if (r == 2) return 0;  /* SYNC group (NOP, etc.) */
                if (r == 3) return 2;  /* RFEI group */
                if (r == 4) return 2;  /* BREAK */
                if (r == 5) return 2;  /* SYSCALL */
                if (r == 6) return 0;  /* RSIL */
                if (r == 7) return 2;  /* WAITI */
                return 0; /* ANY4/ALL4/ANY8/ALL8 */
            }
            case 1: return 0;  /* AND */
            case 2: return 0;  /* OR */
            case 3: return 0;  /* XOR */
            case 4: { /* ST1: shift-amount setup */
                if (r <= 4 || r == 14 || r == 15) return 0; /* SSR/SSL/SSA8L/SSA8B/SSAI/NSA/NSAU */
                if (r == 8) return 2; /* ROTW */
                return 2;
            }
            case 5: return 2;  /* TLB */
            case 6: return 0;  /* RT0: NEG, ABS */
            case 8:  return 0;  /* ADD */
            case 9:  return 0;  /* ADDX2 */
            case 10: return 0;  /* ADDX4 */
            case 11: return 0;  /* ADDX8 */
            case 12: return 0;  /* SUB */
            case 13: return 0;  /* SUBX2 */
            case 14: return 0;  /* SUBX4 */
            case 15: return 0;  /* SUBX8 */
            default: return 2;
            }
        case 1: /* RST1 */
            switch (op2) {
            case 0: case 1: return 0;  /* SLLI */
            case 2: case 3: return 0;  /* SRAI */
            case 4: return 0;  /* SRLI */
            case 6: return 2;  /* XSR — may have side effects */
            case 8: return 0;  /* SRC */
            case 9: return 0;  /* SRL */
            case 10: return 0; /* SLL */
            case 11: return 0; /* SRA */
            case 12: return 0; /* MUL16U */
            case 13: return 0; /* MUL16S */
            default: return 2;
            }
        case 2: /* RST2 */
            switch (op2) {
            case 8: return 0;  /* MULL */
            case 6: return 0;  /* SALT */
            case 7: return 0;  /* SALTU */
            case 10: return 0; /* MULUH */
            case 11: return 0; /* MULSH */
            default: return 2;
            }
        case 3: /* RST3 */
            switch (op2) {
            case 0: return 0;  /* RSR */
            case 1: { /* WSR — some have side effects */
                int sr = ((insn >> 8) & 0xFF);
                switch (sr) {
                case XT_SR_SAR: case XT_SR_LBEG: case XT_SR_LEND:
                case XT_SR_LCOUNT: case XT_SR_BR: case XT_SR_SCOMPARE1:
                case XT_SR_MISC0: case XT_SR_MISC1: case XT_SR_MISC2: case XT_SR_MISC3:
                case XT_SR_EPC1: case XT_SR_EPC2: case XT_SR_EPC3:
                case XT_SR_EPC4: case XT_SR_EPC5: case XT_SR_EPC6: case XT_SR_EPC7:
                case XT_SR_EXCSAVE1: case XT_SR_EXCSAVE2: case XT_SR_EXCSAVE3:
                case XT_SR_EXCSAVE4: case XT_SR_EXCSAVE5: case XT_SR_EXCSAVE6:
                case XT_SR_EXCSAVE7:
                case XT_SR_EPS2: case XT_SR_EPS3: case XT_SR_EPS4:
                case XT_SR_EPS5: case XT_SR_EPS6: case XT_SR_EPS7:
                case XT_SR_VECBASE: case XT_SR_EXCCAUSE: case XT_SR_EXCVADDR:
                case XT_SR_DEPC:
                    return 0;  /* Safe SRs */
                default:
                    return 2;  /* CCOUNT, CCOMPARE, INTENABLE, PS, etc — side effects */
                }
            }
            case 2: return 0;  /* SEXT */
            case 3: return 0;  /* CLAMPS */
            case 4: case 5: case 6: case 7: return 0;  /* MIN/MAX/MINU/MAXU */
            case 8: case 9: case 10: case 11: return 0;  /* MOVEQZ/MOVNEZ/MOVLTZ/MOVGEZ */
            case 12: case 13: return 0;  /* MOVF/MOVT */
            case 14: case 15: return 0;  /* RUR/WUR */
            default: return 2;
            }
        case 4: case 5: return 0;  /* EXTUI */
        default: return 2;  /* FP, MAC16, LSCX, etc */
        }
    }
    case 1: return 0;  /* L32R */
    case 2: { /* LSAI */
        int r = (insn >> 12) & 0xF;
        switch (r) {
        case 0: case 1: case 2: return 0;  /* L8UI, L16UI, L32I */
        case 4: case 5: case 6: return 0;  /* S8I, S16I, S32I */
        case 7: return 0;  /* CACHE ops (no-op) */
        case 9: return 0;  /* L16SI */
        case 0xA: return 0;  /* MOVI */
        case 0xB: return 0;  /* L32AI */
        case 0xC: return 0;  /* ADDI */
        case 0xD: return 0;  /* ADDMI */
        case 0xE: return 2;  /* S32C1I — complex */
        case 0xF: return 0;  /* S32RI */
        default: return 2;
        }
    }
    case 5: return 1;  /* CALLN — block terminator (compiled) */
    case 6: { /* SI: J, BZ, BI0, BI1 */
        int nn = (insn >> 4) & 3;
        if (nn == 0) return 1;  /* J — terminator */
        if (nn == 1) return 1;  /* BZ (BEQZ/BNEZ/BLTZ/BGEZ) — terminator */
        if (nn == 2) return 1;  /* BI0 (BEQI/BNEI/BLTI/BGEI) — terminator */
        /* nn == 3: BI1 */
        int m = (insn >> 6) & 3;
        if (m == 0) return 1;  /* ENTRY — block terminator (compiled) */
        if (m == 1) {
            int r = (insn >> 12) & 0xF;
            if (r == 0 || r == 1) return 1;  /* BF/BT — terminator */
            if (r >= 8 && r <= 10) return 2; /* LOOP/LOOPNEZ/LOOPGTZ — fallback */
            return 2;
        }
        return 1;  /* BLTUI/BGEUI — terminator */
    }
    case 7: return 1;  /* B — all conditional branches are terminators */
    default: return 2;  /* FP loads, MAC16, etc */
    }
}

/* Scan a basic block starting at pc */
static void jit_scan_block(jit_state_t *jit, xtensa_cpu_t *cpu, uint32_t pc,
                           jit_scan_t *scan) {
    (void)jit;
    scan->count = 0;
    uint32_t cur_pc = pc;
    uint32_t page_end = (pc & ~0xFFFu) + 0x1000;

    /* If a zero-overhead loop is active, stop the block at lend so the
     * loop-back check in jit_run fires between blocks. */
    uint32_t lend = (cpu->lcount > 0) ? cpu->lend : 0;

    for (int i = 0; i < JIT_MAX_BLOCK_INSNS; i++) {
        /* Stop at page boundary */
        if (cur_pc >= page_end) break;

        /* Stop at hook addresses (stubs) */
        if (is_hook_addr(cpu, cur_pc)) break;

        uint32_t insn;
        int ilen = jit_fetch(cpu, cur_pc, &insn);
        if (ilen == 0) break;

        int cls = classify_for_jit(insn, ilen);

        scan->insns[i] = insn;
        scan->pcs[i] = cur_pc;
        scan->ilens[i] = ilen;
        scan->count = i + 1;
        cur_pc += (uint32_t)ilen;

        /* Stop at loop end boundary: the instruction AT lend is the last
         * one before the loop-back fires, so include it then stop. */
        if (lend && cur_pc >= lend) break;

        if (cls == 1) {
            /* Block terminator (branch) — include it, then stop */
            break;
        }
        if (cls == 2) {
            /* Fallback instruction — don't include it, back up */
            scan->count = i;
            cur_pc -= (uint32_t)ilen;
            break;
        }
    }
    scan->end_pc = cur_pc;
}


/* ===== Code generation ===== */

/* CPU register R15, MEM pointer R14 */
#define REG_CPU  R15
#define REG_MEM  R14

/* Compute the offset into cpu->ar[] for guest register n,
 * given windowbase*4 is a compile-time constant per block.
 * Returns byte offset from cpu base. */
static inline int32_t ar_offset(int wb4, int n) {
    return (int32_t)(CPU_OFF_AR + ((uint32_t)(wb4 + n) & 63) * 4);
}

/* ===== Register Allocation ===== */

/* Number of guest registers allocated to host regs.
 * a1-a2 → R12-R13 (callee-saved, no save/restore needed around C calls).
 * a3-a6 → R8-R11 (caller-saved, push/pop around slow-path C calls in mem emitters). */
#define RA_COUNT 6

static const int8_t RA_MAP[16] = {
    -1,  /* a0: spilled (modified by CALL/RETW) */
    R12, /* a1: stack pointer — callee-saved */
    R13, /* a2: arg/return  — callee-saved */
    R8,  /* a3: arg — caller-saved, save around C calls */
    R9,  /* a4: arg — caller-saved */
    R10, /* a5: arg — caller-saved */
    R11, /* a6: arg — caller-saved */
    -1, -1, -1, -1, -1, -1, -1, -1, -1  /* a7-a15: spilled */
};

typedef struct {
    uint8_t dirty;    /* bit i set = a(i+1) was written, deferred store */
    uint8_t loaded;   /* bit i set = a(i+1) is live in its host reg */
} regalloc_t;

/* Load guest ar[n] into dst_x86. Uses host reg if allocated. */
static void ra_load_ar(emit_t *e, regalloc_t *ra, int dst_x86, int wb4, int n) {
    if (n >= 0 && n < 16 && RA_MAP[n] >= 0) {
        int host = RA_MAP[n];
        int bit = n - 1;
        if (!(ra->loaded & (1u << bit))) {
            /* Load from memory into host reg */
            emit_load32_disp(e, host, REG_CPU, ar_offset(wb4, n));
            ra->loaded |= (uint8_t)(1u << bit);
        }
        if (dst_x86 != host) {
            emit_mov_reg32_reg32(e, dst_x86, host);
        }
    } else {
        /* Spilled — direct memory load */
        emit_load32_disp(e, dst_x86, REG_CPU, ar_offset(wb4, n));
    }
}

/* Store x86 reg into guest ar[n]. Defers write if allocated. */
static void ra_store_ar(emit_t *e, regalloc_t *ra, int src_x86, int wb4, int n) {
    if (n >= 0 && n < 16 && RA_MAP[n] >= 0) {
        int host = RA_MAP[n];
        int bit = n - 1;
        if (src_x86 != host) {
            emit_mov_reg32_reg32(e, host, src_x86);
        }
        ra->dirty |= (uint8_t)(1u << bit);
        ra->loaded |= (uint8_t)(1u << bit);
    } else {
        /* Spilled — write-through immediately */
        emit_store32_disp(e, src_x86, REG_CPU, ar_offset(wb4, n));
    }
}

/* Flush all dirty allocated regs to memory. Called at block exits.
 * Dirty bits persist for the full block compilation (reset via regalloc_t ra = {0,0}
 * at each new block). Both branch exits emit the same stores — the second is
 * redundant but correct. Cost: ≤2 extra mov instructions at one exit per block. */
static void ra_flush(emit_t *e, regalloc_t *ra, int wb4) {
    for (int n = 1; n <= RA_COUNT; n++) {
        int bit = n - 1;
        if (ra->dirty & (1u << bit)) {
            emit_store32_disp(e, RA_MAP[n], REG_CPU, ar_offset(wb4, n));
        }
    }
    /* NOTE: do NOT clear ra->dirty here. Multiple block exits (both sides of a
     * branch) need to emit stores independently. Dirty bits reset at block start. */
}

/* Unconditionally flush ALL allocated regs (ignores dirty tracking).
 * Used on fallback paths where the compile-time dirty bits are unreliable. */
static void ra_flush_all(emit_t *e, int wb4) {
    for (int n = 1; n <= RA_COUNT; n++) {
        emit_store32_disp(e, RA_MAP[n], REG_CPU, ar_offset(wb4, n));
    }
}

/* Pre-load allocated regs from memory. Called at block entry. */
static void ra_preload(emit_t *e, regalloc_t *ra, int wb4) {
    for (int n = 1; n <= RA_COUNT; n++) {
        emit_load32_disp(e, RA_MAP[n], REG_CPU, ar_offset(wb4, n));
    }
    ra->loaded = (uint8_t)((1u << RA_COUNT) - 1);
    ra->dirty = 0;
}

/* (emit_load_ar_direct / emit_store_ar_direct removed — use emit_load32_disp directly) */

/* Emit: load a CPU field (32-bit) into x86 reg */
static void emit_load_cpu32(emit_t *e, int dst, int32_t offset) {
    emit_load32_disp(e, dst, REG_CPU, offset);
}

/* Emit: store x86 reg (32-bit) into a CPU field */
static void emit_store_cpu32(emit_t *e, int src, int32_t offset) {
    emit_store32_disp(e, src, REG_CPU, offset);
}

/* Emit: store immediate (32-bit) into a CPU field */
static void emit_store_cpu32_imm(emit_t *e, int32_t offset, uint32_t imm) {
    emit_store32_disp_imm(e, REG_CPU, offset, imm);
}

/* Emit memory read32 inlined fast path:
 * page = mem->page_table[(addr >> 12)]
 * if (page) result = *(uint32_t*)(page + (addr & 0xFFF))
 * else call mem_read32_slow
 *
 * addr_reg: register containing guest address (preserved)
 * dst_reg: register to receive loaded value
 * Uses RAX, RCX, RDX as scratch
 */
static void emit_mem_read32(emit_t *e, int addr_reg, int dst_reg) {
    /* ecx = addr >> 12 */
    emit_mov_reg32_reg32(e, RCX, addr_reg);
    emit_shr_reg32_imm(e, RCX, 12);

    /* rax = mem->page_table[ecx] — page_table is at MEM_OFF_PAGE_TABLE in mem struct */
    /* rax = [r14 + rcx*8 + MEM_OFF_PAGE_TABLE] */
    /* Manual encoding: REX.W=1, REX.R=0, REX.X=0, REX.B=1 (r14) */
    emit8(e, rex(1, 0, 0, (R14 >> 3) & 1));  /* REX.W + B for r14 */
    emit8(e, 0x8B);  /* MOV r64, [...]  */
    emit8(e, modrm(2, RAX, 4));  /* mod=10, reg=rax, rm=SIB */
    emit8(e, sib(3, RCX, R14 & 7));  /* scale=8, index=rcx, base=r14 */
    emit32(e, (uint32_t)MEM_OFF_PAGE_TABLE);

    /* test rax, rax */
    emit8(e, rex(1, 0, 0, 0));
    emit8(e, 0x85);
    emit8(e, modrm(3, RAX, RAX));

    /* jz slow_path */
    int slow_patch = emit_jcc_rel32(e, CC_E);

    /* Fast path: edx = addr & 0xFFF */
    emit_mov_reg32_reg32(e, RDX, addr_reg);
    emit_and_reg32_imm32(e, RDX, 0xFFF);

    /* dst = [rax + rdx] (32-bit load) */
    /* Using movsxd trick: mov dst32, [rax + rdx] */
    emit_rex(e, 0, dst_reg, RAX);
    emit8(e, 0x8B);
    emit8(e, modrm(0, dst_reg, 4));  /* SIB follows */
    emit8(e, sib(0, RDX, RAX));      /* scale=1, index=rdx, base=rax */

    int done_patch = emit_jmp_rel32(e);

    /* Slow path: call mem_read32_slow(mem, addr) */
    emit_patch_rel32(e, slow_patch);
    /* Save caller-saved allocated regs around C call */
    emit_push(e, R8); emit_push(e, R9); emit_push(e, R10); emit_push(e, R11);
    /* rdi = mem (r14), esi = addr */
    emit_mov_reg_reg(e, RDI, REG_MEM);
    emit_mov_reg32_reg32(e, RSI, addr_reg);
    emit_mov_reg_imm64(e, RAX, (uint64_t)(uintptr_t)mem_read32_slow);
    emit_call_reg(e, RAX);
    emit_pop(e, R11); emit_pop(e, R10); emit_pop(e, R9); emit_pop(e, R8);
    /* Result is in eax, move to dst if needed */
    if (dst_reg != RAX) {
        emit_mov_reg32_reg32(e, dst_reg, RAX);
    }

    emit_patch_rel32(e, done_patch);
}

/* Emit memory write32 inlined fast path.
 * addr_reg: register containing guest address
 * val_reg: register containing value to store
 * Uses RAX, RCX, RDX as scratch. addr_reg and val_reg must not be RAX/RCX/RDX.
 */
static void emit_mem_write32(emit_t *e, int addr_reg, int val_reg) {
    /* ecx = addr >> 12 */
    emit_mov_reg32_reg32(e, RCX, addr_reg);
    emit_shr_reg32_imm(e, RCX, 12);

    /* rax = mem->page_table[ecx] */
    emit8(e, rex(1, 0, 0, (R14 >> 3) & 1));
    emit8(e, 0x8B);
    emit8(e, modrm(2, RAX, 4));
    emit8(e, sib(3, RCX, R14 & 7));
    emit32(e, (uint32_t)MEM_OFF_PAGE_TABLE);

    /* test rax, rax */
    emit8(e, rex(1, 0, 0, 0));
    emit8(e, 0x85);
    emit8(e, modrm(3, RAX, RAX));

    int slow_patch = emit_jcc_rel32(e, CC_E);

    /* Fast path: edx = addr & 0xFFF */
    emit_mov_reg32_reg32(e, RDX, addr_reg);
    emit_and_reg32_imm32(e, RDX, 0xFFF);

    /* [rax + rdx] = val_reg (32-bit store) */
    emit_rex(e, 0, val_reg, RAX);
    emit8(e, 0x89);
    emit8(e, modrm(0, val_reg, 4));
    emit8(e, sib(0, RDX, RAX));

    int done_patch = emit_jmp_rel32(e);

    /* Slow path: call mem_write32_slow(mem, addr, val) */
    emit_patch_rel32(e, slow_patch);
    emit_push(e, R8); emit_push(e, R9); emit_push(e, R10); emit_push(e, R11);
    emit_mov_reg_reg(e, RDI, REG_MEM);
    emit_mov_reg32_reg32(e, RSI, addr_reg);
    emit_mov_reg32_reg32(e, RDX, val_reg);
    emit_mov_reg_imm64(e, RAX, (uint64_t)(uintptr_t)mem_write32_slow);
    emit_call_reg(e, RAX);
    emit_pop(e, R11); emit_pop(e, R10); emit_pop(e, R9); emit_pop(e, R8);

    emit_patch_rel32(e, done_patch);
}

/* Emit memory read8u inlined fast path */
static void emit_mem_read8u(emit_t *e, int addr_reg, int dst_reg) {
    emit_mov_reg32_reg32(e, RCX, addr_reg);
    emit_shr_reg32_imm(e, RCX, 12);
    emit8(e, rex(1, 0, 0, (R14 >> 3) & 1));
    emit8(e, 0x8B);
    emit8(e, modrm(2, RAX, 4));
    emit8(e, sib(3, RCX, R14 & 7));
    emit32(e, (uint32_t)MEM_OFF_PAGE_TABLE);
    emit8(e, rex(1, 0, 0, 0)); emit8(e, 0x85); emit8(e, modrm(3, RAX, RAX));
    int slow_patch = emit_jcc_rel32(e, CC_E);
    emit_mov_reg32_reg32(e, RDX, addr_reg);
    emit_and_reg32_imm32(e, RDX, 0xFFF);
    /* movzx dst32, byte [rax + rdx] */
    emit_rex(e, 0, dst_reg, RAX);
    emit8(e, 0x0F); emit8(e, 0xB6);
    emit8(e, modrm(0, dst_reg, 4));
    emit8(e, sib(0, RDX, RAX));
    int done_patch = emit_jmp_rel32(e);
    emit_patch_rel32(e, slow_patch);
    emit_push(e, R8); emit_push(e, R9); emit_push(e, R10); emit_push(e, R11);
    emit_mov_reg_reg(e, RDI, REG_MEM);
    emit_mov_reg32_reg32(e, RSI, addr_reg);
    emit_mov_reg_imm64(e, RAX, (uint64_t)(uintptr_t)mem_read8_slow);
    emit_call_reg(e, RAX);
    emit_pop(e, R11); emit_pop(e, R10); emit_pop(e, R9); emit_pop(e, R8);
    if (dst_reg != RAX) emit_mov_reg32_reg32(e, dst_reg, RAX);
    emit_patch_rel32(e, done_patch);
}

/* Emit memory read16u inlined fast path */
static void emit_mem_read16u(emit_t *e, int addr_reg, int dst_reg) {
    emit_mov_reg32_reg32(e, RCX, addr_reg);
    emit_shr_reg32_imm(e, RCX, 12);
    emit8(e, rex(1, 0, 0, (R14 >> 3) & 1));
    emit8(e, 0x8B);
    emit8(e, modrm(2, RAX, 4));
    emit8(e, sib(3, RCX, R14 & 7));
    emit32(e, (uint32_t)MEM_OFF_PAGE_TABLE);
    emit8(e, rex(1, 0, 0, 0)); emit8(e, 0x85); emit8(e, modrm(3, RAX, RAX));
    int slow_patch = emit_jcc_rel32(e, CC_E);
    emit_mov_reg32_reg32(e, RDX, addr_reg);
    emit_and_reg32_imm32(e, RDX, 0xFFF);
    /* movzx dst32, word [rax + rdx] */
    emit_rex(e, 0, dst_reg, RAX);
    emit8(e, 0x0F); emit8(e, 0xB7);
    emit8(e, modrm(0, dst_reg, 4));
    emit8(e, sib(0, RDX, RAX));
    int done_patch = emit_jmp_rel32(e);
    emit_patch_rel32(e, slow_patch);
    emit_push(e, R8); emit_push(e, R9); emit_push(e, R10); emit_push(e, R11);
    emit_mov_reg_reg(e, RDI, REG_MEM);
    emit_mov_reg32_reg32(e, RSI, addr_reg);
    emit_mov_reg_imm64(e, RAX, (uint64_t)(uintptr_t)mem_read16_slow);
    emit_call_reg(e, RAX);
    emit_pop(e, R11); emit_pop(e, R10); emit_pop(e, R9); emit_pop(e, R8);
    if (dst_reg != RAX) emit_mov_reg32_reg32(e, dst_reg, RAX);
    emit_patch_rel32(e, done_patch);
}

/* Emit memory read16s (signed) */
static void emit_mem_read16s(emit_t *e, int addr_reg, int dst_reg) {
    emit_mov_reg32_reg32(e, RCX, addr_reg);
    emit_shr_reg32_imm(e, RCX, 12);
    emit8(e, rex(1, 0, 0, (R14 >> 3) & 1));
    emit8(e, 0x8B);
    emit8(e, modrm(2, RAX, 4));
    emit8(e, sib(3, RCX, R14 & 7));
    emit32(e, (uint32_t)MEM_OFF_PAGE_TABLE);
    emit8(e, rex(1, 0, 0, 0)); emit8(e, 0x85); emit8(e, modrm(3, RAX, RAX));
    int slow_patch = emit_jcc_rel32(e, CC_E);
    emit_mov_reg32_reg32(e, RDX, addr_reg);
    emit_and_reg32_imm32(e, RDX, 0xFFF);
    /* movsx dst32, word [rax + rdx] */
    emit_rex(e, 0, dst_reg, RAX);
    emit8(e, 0x0F); emit8(e, 0xBF);
    emit8(e, modrm(0, dst_reg, 4));
    emit8(e, sib(0, RDX, RAX));
    int done_patch = emit_jmp_rel32(e);
    emit_patch_rel32(e, slow_patch);
    emit_push(e, R8); emit_push(e, R9); emit_push(e, R10); emit_push(e, R11);
    emit_mov_reg_reg(e, RDI, REG_MEM);
    emit_mov_reg32_reg32(e, RSI, addr_reg);
    emit_mov_reg_imm64(e, RAX, (uint64_t)(uintptr_t)mem_read16_slow);
    emit_call_reg(e, RAX);
    emit_pop(e, R11); emit_pop(e, R10); emit_pop(e, R9); emit_pop(e, R8);
    /* Sign extend from 16 bits */
    emit_movsx_reg32_reg16(e, dst_reg != RAX ? dst_reg : RAX, RAX);
    if (dst_reg != RAX) { /* already done */ }
    emit_patch_rel32(e, done_patch);
}

/* Emit memory write8 */
static void emit_mem_write8(emit_t *e, int addr_reg, int val_reg) {
    emit_mov_reg32_reg32(e, RCX, addr_reg);
    emit_shr_reg32_imm(e, RCX, 12);
    emit8(e, rex(1, 0, 0, (R14 >> 3) & 1));
    emit8(e, 0x8B);
    emit8(e, modrm(2, RAX, 4));
    emit8(e, sib(3, RCX, R14 & 7));
    emit32(e, (uint32_t)MEM_OFF_PAGE_TABLE);
    emit8(e, rex(1, 0, 0, 0)); emit8(e, 0x85); emit8(e, modrm(3, RAX, RAX));
    int slow_patch = emit_jcc_rel32(e, CC_E);
    emit_mov_reg32_reg32(e, RDX, addr_reg);
    emit_and_reg32_imm32(e, RDX, 0xFFF);
    /* mov byte [rax + rdx], val_reg_low8 */
    emit_rex(e, 0, val_reg, RAX);
    emit8(e, 0x88);
    emit8(e, modrm(0, val_reg, 4));
    emit8(e, sib(0, RDX, RAX));
    int done_patch = emit_jmp_rel32(e);
    emit_patch_rel32(e, slow_patch);
    emit_push(e, R8); emit_push(e, R9); emit_push(e, R10); emit_push(e, R11);
    emit_mov_reg_reg(e, RDI, REG_MEM);
    emit_mov_reg32_reg32(e, RSI, addr_reg);
    emit_mov_reg32_reg32(e, RDX, val_reg);
    emit_mov_reg_imm64(e, RAX, (uint64_t)(uintptr_t)mem_write8_slow);
    emit_call_reg(e, RAX);
    emit_pop(e, R11); emit_pop(e, R10); emit_pop(e, R9); emit_pop(e, R8);
    emit_patch_rel32(e, done_patch);
}

/* Emit memory write16 */
static void emit_mem_write16(emit_t *e, int addr_reg, int val_reg) {
    emit_mov_reg32_reg32(e, RCX, addr_reg);
    emit_shr_reg32_imm(e, RCX, 12);
    emit8(e, rex(1, 0, 0, (R14 >> 3) & 1));
    emit8(e, 0x8B);
    emit8(e, modrm(2, RAX, 4));
    emit8(e, sib(3, RCX, R14 & 7));
    emit32(e, (uint32_t)MEM_OFF_PAGE_TABLE);
    emit8(e, rex(1, 0, 0, 0)); emit8(e, 0x85); emit8(e, modrm(3, RAX, RAX));
    int slow_patch = emit_jcc_rel32(e, CC_E);
    emit_mov_reg32_reg32(e, RDX, addr_reg);
    emit_and_reg32_imm32(e, RDX, 0xFFF);
    /* mov word [rax + rdx], val_reg_low16 */
    emit8(e, 0x66);
    emit_rex(e, 0, val_reg, RAX);
    emit8(e, 0x89);
    emit8(e, modrm(0, val_reg, 4));
    emit8(e, sib(0, RDX, RAX));
    int done_patch = emit_jmp_rel32(e);
    emit_patch_rel32(e, slow_patch);
    emit_push(e, R8); emit_push(e, R9); emit_push(e, R10); emit_push(e, R11);
    emit_mov_reg_reg(e, RDI, REG_MEM);
    emit_mov_reg32_reg32(e, RSI, addr_reg);
    emit_mov_reg32_reg32(e, RDX, val_reg);
    emit_mov_reg_imm64(e, RAX, (uint64_t)(uintptr_t)mem_write16_slow);
    emit_call_reg(e, RAX);
    emit_pop(e, R11); emit_pop(e, R10); emit_pop(e, R9); emit_pop(e, R8);
    emit_patch_rel32(e, done_patch);
}


/* ===== Per-instruction compilation ===== */

/* Forward declarations for helpers used in instruction compilation */
static void emit_block_exit_ra(emit_t *e, regalloc_t *ra, int wb4,
                               uint32_t exit_pc, int insn_count,
                               jit_state_t *jit);
static void emit_jmp_to_epilogue(emit_t *e, jit_state_t *jit);

/* Compile a single instruction. Returns 1 on success, 0 if we should abort the block. */
static int jit_compile_insn(emit_t *e, int wb4, uint32_t insn, int ilen,
                            uint32_t pc, uint32_t next_pc, int insn_idx,
                            regalloc_t *ra, jit_state_t *jit) {
    if (ilen == 2) {
        /* Narrow instructions */
        int op0 = insn & 0xF;
        int t = (insn >> 4) & 0xF;
        int s = (insn >> 8) & 0xF;
        int r = (insn >> 12) & 0xF;

        switch (op0) {
        case 0x8: { /* L32I.N: at = mem32[as + r*4] */
            ra_load_ar(e, ra,RSI, wb4, s);
            emit_add_reg32_imm32(e, RSI, r << 2);
            emit_mem_read32(e, RSI, RBX);
            ra_store_ar(e, ra,RBX, wb4, t);
            return 1;
        }
        case 0x9: { /* S32I.N: mem32[as + r*4] = at */
            ra_load_ar(e, ra,RSI, wb4, s);
            emit_add_reg32_imm32(e, RSI, r << 2);
            ra_load_ar(e, ra,RBP, wb4, t);
            emit_mem_write32(e, RSI, RBP);
            return 1;
        }
        case 0xA: { /* ADD.N: ar = as + at */
            ra_load_ar(e, ra,RAX, wb4, s);
            ra_load_ar(e, ra,RBX, wb4, t);
            emit_add_reg32(e, RAX, RBX);
            ra_store_ar(e, ra,RAX, wb4, r);
            return 1;
        }
        case 0xB: { /* ADDI.N: ar = as + (t==0 ? -1 : t) */
            int32_t imm = (t == 0) ? -1 : t;
            ra_load_ar(e, ra,RAX, wb4, s);
            emit_add_reg32_imm32(e, RAX, imm);
            ra_store_ar(e, ra,RAX, wb4, r);
            return 1;
        }
        case 0xC: { /* ST2: MOVI.N / BEQZ.N / BNEZ.N */
            int t_hi = t >> 2;
            if (t_hi < 2) {
                /* MOVI.N */
                int imm7 = ((t & 7) << 4) | r;
                int32_t val = (imm7 >= 96) ? (imm7 - 128) : imm7;
                emit_mov_reg_imm32(e, RAX, (uint32_t)val);
                ra_store_ar(e, ra,RAX, wb4, s);
                return 1;
            }
            /* BEQZ.N / BNEZ.N — block terminators */
            int imm6 = ((t & 3) << 4) | r;
            uint32_t target = next_pc + (uint32_t)imm6 + 2; /* +2 per ISA */
            ra_load_ar(e, ra,RAX, wb4, s);
            emit_test_reg32(e, RAX, RAX);
            if (t_hi == 2) {
                /* BEQZ.N: taken if as == 0 */
                int taken_patch = emit_jcc_rel32(e, CC_E);
                /* Not taken: exit to next_pc */
                emit_block_exit_ra(e, ra, wb4, next_pc, insn_idx + 1, jit);
                emit_patch_rel32(e, taken_patch);
                emit_block_exit_ra(e, ra, wb4, target, insn_idx + 1, jit);
            } else {
                /* BNEZ.N: taken if as != 0 */
                int taken_patch = emit_jcc_rel32(e, CC_NE);
                emit_block_exit_ra(e, ra, wb4, next_pc, insn_idx + 1, jit);
                emit_patch_rel32(e, taken_patch);
                emit_block_exit_ra(e, ra, wb4, target, insn_idx + 1, jit);
            }
            return 1;
        }
        case 0xD: { /* ST3 */
            if (r == 0) {
                /* MOV.N: at = as */
                ra_load_ar(e, ra,RAX, wb4, s);
                ra_store_ar(e, ra,RAX, wb4, t);
                return 1;
            }
            if (r == 15) {
                if (t == 0) {
                    /* RET.N: pc = a0 */
                    ra_load_ar(e, ra,RAX, wb4, 0);
                    emit_store_cpu32(e, RAX, (int32_t)CPU_OFF_PC);
                    /* Set _pc_written = true */
                    emit_store32_disp_imm(e, REG_CPU, (int32_t)CPU_OFF_PC_WRITTEN, 1);
                    emit_block_exit_ra(e, ra, wb4, 0 /* will use cpu->pc */, insn_idx + 1, jit);
                    return 1;
                }
                if (t == 1) {
                    /* RETW.N — same as RETW but ilen=2 */
                    goto compile_retw;
                }
                if (t == 3) {
                    /* NOP.N */
                    return 1;
                }
            }
            return 0;
        }
        default: return 0;
        }
    }

    /* 24-bit instructions */
    int op0 = XT_OP0(insn);
    int t = XT_T(insn);
    int s = XT_S(insn);
    int r = XT_R(insn);
    int op1 = XT_OP1(insn);
    int op2 = XT_OP2(insn);
    int imm8 = XT_IMM8(insn);

    switch (op0) {
    case 0: { /* QRST */
        switch (op1) {
        case 0: { /* RST0 */
            switch (op2) {
            case 0: { /* ST0 specials */
                if (r == 2) return 1; /* NOP/SYNC — no-op */
                if (r == 6) {
                    /* RSIL: at = PS; PS.INTLEVEL = s */
                    emit_load_cpu32(e, RAX, (int32_t)CPU_OFF_PS);
                    ra_store_ar(e, ra,RAX, wb4, t);
                    /* PS = (PS & ~0xF) | (s & 0xF) */
                    emit_and_reg32_imm32(e, RAX, ~0xF);
                    emit_add_reg32_imm32(e, RAX, s & 0xF);
                    emit_store_cpu32(e, RAX, (int32_t)CPU_OFF_PS);
                    return 1;
                }
                /* RET: pc = a0 */
                int m = (insn >> 6) & 3;
                int nn = (insn >> 4) & 3;
                if (r == 0 && m == 2 && nn == 0) {
                    ra_load_ar(e, ra,RAX, wb4, 0);
                    emit_store_cpu32(e, RAX, (int32_t)CPU_OFF_PC);
                    emit_store32_disp_imm(e, REG_CPU, (int32_t)CPU_OFF_PC_WRITTEN, 1);
                    emit_block_exit_ra(e, ra, wb4, 0, insn_idx + 1, jit);
                    return 1;
                }
                /* RETW: window return (r==0, m==2, nn==1) */
                if (r == 0 && m == 2 && nn == 1) {
                    goto compile_retw;
                }
                /* JX: pc = as */
                if (r == 0 && m == 2 && nn == 2) {
                    ra_load_ar(e, ra, RAX, wb4, s);
                    emit_store_cpu32(e, RAX, (int32_t)CPU_OFF_PC);
                    emit_store32_disp_imm(e, REG_CPU, (int32_t)CPU_OFF_PC_WRITTEN, 1);
                    emit_block_exit_ra(e, ra, wb4, 0, insn_idx + 1, jit);
                    return 1;
                }
                /* Boolean ops */
                if (r >= 8 && r <= 11) {
                    /* ANY4/ALL4/ANY8/ALL8 */
                    return 0; /* skip for now */
                }
                return 0;
            }
            case 1: { /* AND: ar = as & at */
                ra_load_ar(e, ra,RAX, wb4, s);
                ra_load_ar(e, ra,RBX, wb4, t);
                emit_and_reg32(e, RAX, RBX);
                ra_store_ar(e, ra,RAX, wb4, r);
                return 1;
            }
            case 2: { /* OR: ar = as | at */
                ra_load_ar(e, ra,RAX, wb4, s);
                ra_load_ar(e, ra,RBX, wb4, t);
                emit_or_reg32(e, RAX, RBX);
                ra_store_ar(e, ra,RAX, wb4, r);
                return 1;
            }
            case 3: { /* XOR: ar = as ^ at */
                ra_load_ar(e, ra,RAX, wb4, s);
                ra_load_ar(e, ra,RBX, wb4, t);
                emit_xor_reg32(e, RAX, RBX);
                ra_store_ar(e, ra,RAX, wb4, r);
                return 1;
            }
            case 4: { /* ST1: shift setup */
                switch (r) {
                case 0: /* SSR: SAR = as & 31 */
                    ra_load_ar(e, ra,RAX, wb4, s);
                    emit_and_reg32_imm32(e, RAX, 0x1F);
                    emit_store_cpu32(e, RAX, (int32_t)CPU_OFF_SAR);
                    return 1;
                case 1: /* SSL: SAR = 32 - (as & 31) */
                    ra_load_ar(e, ra,RAX, wb4, s);
                    emit_and_reg32_imm32(e, RAX, 0x1F);
                    emit_neg_reg32(e, RAX);
                    emit_add_reg32_imm32(e, RAX, 32);
                    emit_store_cpu32(e, RAX, (int32_t)CPU_OFF_SAR);
                    return 1;
                case 2: /* SSA8L: SAR = (as & 3) * 8 */
                    ra_load_ar(e, ra,RAX, wb4, s);
                    emit_and_reg32_imm32(e, RAX, 3);
                    emit_shl_reg32_imm(e, RAX, 3);
                    emit_store_cpu32(e, RAX, (int32_t)CPU_OFF_SAR);
                    return 1;
                case 3: /* SSA8B: SAR = 32 - (as & 3) * 8 */
                    ra_load_ar(e, ra,RAX, wb4, s);
                    emit_and_reg32_imm32(e, RAX, 3);
                    emit_shl_reg32_imm(e, RAX, 3);
                    emit_neg_reg32(e, RAX);
                    emit_add_reg32_imm32(e, RAX, 32);
                    emit_store_cpu32(e, RAX, (int32_t)CPU_OFF_SAR);
                    return 1;
                case 4: /* SSAI: SAR = imm5 */
                    emit_mov_reg_imm32(e, RAX, (uint32_t)(s | ((t & 1) << 4)));
                    emit_store_cpu32(e, RAX, (int32_t)CPU_OFF_SAR);
                    return 1;
                case 14: { /* NSA */
                    ra_load_ar(e, ra,RAX, wb4, s);
                    /* Full NSA emulation: normalize signed value */
                    /* Emit a call to a small helper that computes NSA */
                    /* For now, use a C helper call */
                    return 0; /* skip for now */
                }
                case 15: { /* NSAU */
                    return 0; /* skip for now */
                }
                default: return 0;
                }
            }
            case 6: { /* RT0 */
                if (s == 0) {
                    /* NEG: ar = -at */
                    ra_load_ar(e, ra,RAX, wb4, t);
                    emit_neg_reg32(e, RAX);
                    ra_store_ar(e, ra,RAX, wb4, r);
                    return 1;
                }
                if (s == 1) {
                    /* ABS: ar = |at| */
                    ra_load_ar(e, ra,RAX, wb4, t);
                    emit_mov_reg32_reg32(e, RBX, RAX);
                    emit_sar_reg32_imm(e, RBX, 31);
                    emit_xor_reg32(e, RAX, RBX);
                    emit_sub_reg32(e, RAX, RBX);
                    ra_store_ar(e, ra,RAX, wb4, r);
                    return 1;
                }
                return 0;
            }
            case 8: { /* ADD: ar = as + at */
                ra_load_ar(e, ra,RAX, wb4, s);
                ra_load_ar(e, ra,RBX, wb4, t);
                emit_add_reg32(e, RAX, RBX);
                ra_store_ar(e, ra,RAX, wb4, r);
                return 1;
            }
            case 9: { /* ADDX2: ar = (as << 1) + at */
                ra_load_ar(e, ra,RAX, wb4, s);
                emit_shl_reg32_imm(e, RAX, 1);
                ra_load_ar(e, ra,RBX, wb4, t);
                emit_add_reg32(e, RAX, RBX);
                ra_store_ar(e, ra,RAX, wb4, r);
                return 1;
            }
            case 10: { /* ADDX4: ar = (as << 2) + at */
                ra_load_ar(e, ra,RAX, wb4, s);
                emit_shl_reg32_imm(e, RAX, 2);
                ra_load_ar(e, ra,RBX, wb4, t);
                emit_add_reg32(e, RAX, RBX);
                ra_store_ar(e, ra,RAX, wb4, r);
                return 1;
            }
            case 11: { /* ADDX8: ar = (as << 3) + at */
                ra_load_ar(e, ra,RAX, wb4, s);
                emit_shl_reg32_imm(e, RAX, 3);
                ra_load_ar(e, ra,RBX, wb4, t);
                emit_add_reg32(e, RAX, RBX);
                ra_store_ar(e, ra,RAX, wb4, r);
                return 1;
            }
            case 12: { /* SUB: ar = as - at */
                ra_load_ar(e, ra,RAX, wb4, s);
                ra_load_ar(e, ra,RBX, wb4, t);
                emit_sub_reg32(e, RAX, RBX);
                ra_store_ar(e, ra,RAX, wb4, r);
                return 1;
            }
            case 13: { /* SUBX2: ar = (as << 1) - at */
                ra_load_ar(e, ra,RAX, wb4, s);
                emit_shl_reg32_imm(e, RAX, 1);
                ra_load_ar(e, ra,RBX, wb4, t);
                emit_sub_reg32(e, RAX, RBX);
                ra_store_ar(e, ra,RAX, wb4, r);
                return 1;
            }
            case 14: { /* SUBX4 */
                ra_load_ar(e, ra,RAX, wb4, s);
                emit_shl_reg32_imm(e, RAX, 2);
                ra_load_ar(e, ra,RBX, wb4, t);
                emit_sub_reg32(e, RAX, RBX);
                ra_store_ar(e, ra,RAX, wb4, r);
                return 1;
            }
            case 15: { /* SUBX8 */
                ra_load_ar(e, ra,RAX, wb4, s);
                emit_shl_reg32_imm(e, RAX, 3);
                ra_load_ar(e, ra,RBX, wb4, t);
                emit_sub_reg32(e, RAX, RBX);
                ra_store_ar(e, ra,RAX, wb4, r);
                return 1;
            }
            default: return 0;
            }
        } /* end RST0 */

        case 1: { /* RST1: shifts */
            switch (op2) {
            case 0: case 1: { /* SLLI: ar = as << sa (sa = 32 - ((op2&1)<<4 | t)) */
                int sa = 32 - (((op2 & 1) << 4) | t);
                if (sa >= 32) {
                    emit_mov_reg_imm32(e, RAX, 0);
                } else {
                    ra_load_ar(e, ra,RAX, wb4, s);
                    if (sa > 0) emit_shl_reg32_imm(e, RAX, (uint8_t)sa);
                }
                ra_store_ar(e, ra,RAX, wb4, r);
                return 1;
            }
            case 2: case 3: { /* SRAI: ar = (int32)at >> sa */
                int sa = ((op2 & 1) << 4) | s;
                ra_load_ar(e, ra,RAX, wb4, t);
                if (sa > 0) emit_sar_reg32_imm(e, RAX, (uint8_t)sa);
                ra_store_ar(e, ra,RAX, wb4, r);
                return 1;
            }
            case 4: { /* SRLI: ar = at >> s */
                ra_load_ar(e, ra,RAX, wb4, t);
                if (s > 0) emit_shr_reg32_imm(e, RAX, (uint8_t)s);
                ra_store_ar(e, ra,RAX, wb4, r);
                return 1;
            }
            case 8: { /* SRC: ar = (as:at) >> SAR (SAR 0-32, 6 bits) */
                /* Load SAR into CL */
                emit_load_cpu32(e, RCX, (int32_t)CPU_OFF_SAR);
                emit_and_reg32_imm32(e, RCX, 0x3F);
                /* Build 64-bit concat in RAX */
                ra_load_ar(e, ra,RAX, wb4, s);
                emit_shl_reg64_imm(e, RAX, 32);  /* high 32 */
                ra_load_ar(e, ra,RBX, wb4, t);
                /* Zero-extend RBX to 64-bit and OR */
                emit_or_reg32(e, RAX, RBX);  /* This only ORs low 32 into RAX */
                /* Actually need: RAX = (as << 32) | at
                 * The 32-bit OR won't work for 64-bit - use OR r64 */
                /* Redo: rax already has as << 32. rbx has at (32-bit, zero-extended).
                 * We need: or rax, rbx (64-bit) */
                emit8(e, rex(1, 0, 0, 0)); emit8(e, 0x09); emit8(e, modrm(3, RBX, RAX));
                /* Now shift right by CL */
                emit_shr_reg64_cl(e, RAX);
                /* Store low 32 bits */
                ra_store_ar(e, ra,RAX, wb4, r);
                return 1;
            }
            case 9: { /* SRL: ar = at >> SAR (logical, SAR 0-32) */
                emit_load_cpu32(e, RCX, (int32_t)CPU_OFF_SAR);
                emit_and_reg32_imm32(e, RCX, 0x3F);
                ra_load_ar(e, ra,RAX, wb4, t);
                /* if SAR >= 32, result is 0 */
                emit_cmp_reg32_imm32(e, RCX, 32);
                int big_patch = emit_jcc_rel32(e, CC_AE);
                emit_shr_reg32_cl(e, RAX);
                int done_patch = emit_jmp_rel32(e);
                emit_patch_rel32(e, big_patch);
                emit_mov_reg_imm32(e, RAX, 0);
                emit_patch_rel32(e, done_patch);
                ra_store_ar(e, ra,RAX, wb4, r);
                return 1;
            }
            case 10: { /* SLL: ar = as << (32 - SAR) equivalent to (as:0) >> SAR */
                emit_load_cpu32(e, RCX, (int32_t)CPU_OFF_SAR);
                emit_and_reg32_imm32(e, RCX, 0x3F);
                ra_load_ar(e, ra,RAX, wb4, s);
                emit_shl_reg64_imm(e, RAX, 32);
                emit_shr_reg64_cl(e, RAX);
                ra_store_ar(e, ra,RAX, wb4, r);
                return 1;
            }
            case 11: { /* SRA: ar = (int32)at >> SAR (arithmetic) */
                emit_load_cpu32(e, RCX, (int32_t)CPU_OFF_SAR);
                emit_and_reg32_imm32(e, RCX, 0x3F);
                ra_load_ar(e, ra,RAX, wb4, t);
                emit_cmp_reg32_imm32(e, RCX, 32);
                int big_patch = emit_jcc_rel32(e, CC_AE);
                emit_sar_reg32_cl(e, RAX);
                int done_patch = emit_jmp_rel32(e);
                emit_patch_rel32(e, big_patch);
                emit_sar_reg32_imm(e, RAX, 31);
                emit_patch_rel32(e, done_patch);
                ra_store_ar(e, ra,RAX, wb4, r);
                return 1;
            }
            case 12: { /* MUL16U: ar = (as & 0xFFFF) * (at & 0xFFFF) */
                ra_load_ar(e, ra,RAX, wb4, s);
                emit_and_reg32_imm32(e, RAX, 0xFFFF);
                ra_load_ar(e, ra,RBX, wb4, t);
                emit_and_reg32_imm32(e, RBX, 0xFFFF);
                emit_imul_reg32(e, RAX, RBX);
                ra_store_ar(e, ra,RAX, wb4, r);
                return 1;
            }
            case 13: { /* MUL16S: ar = sext16(as) * sext16(at) */
                ra_load_ar(e, ra,RAX, wb4, s);
                emit_movsx_reg32_reg16(e, RAX, RAX);
                ra_load_ar(e, ra,RBX, wb4, t);
                emit_movsx_reg32_reg16(e, RBX, RBX);
                emit_imul_reg32(e, RAX, RBX);
                ra_store_ar(e, ra,RAX, wb4, r);
                return 1;
            }
            default: return 0;
            }
        } /* end RST1 */

        case 2: { /* RST2 */
            switch (op2) {
            case 6: { /* SALT: ar = (int32)as < (int32)at ? 1 : 0 */
                ra_load_ar(e, ra,RAX, wb4, s);
                ra_load_ar(e, ra,RBX, wb4, t);
                emit_cmp_reg32(e, RAX, RBX);
                /* setl al; movzx eax, al */
                emit8(e, 0x0F); emit8(e, 0x9C); emit8(e, modrm(3, 0, RAX)); /* setl al */
                emit8(e, 0x0F); emit8(e, 0xB6); emit8(e, modrm(3, RAX, RAX)); /* movzx eax, al */
                ra_store_ar(e, ra,RAX, wb4, r);
                return 1;
            }
            case 7: { /* SALTU: ar = as < at ? 1 : 0 (unsigned) */
                ra_load_ar(e, ra,RAX, wb4, s);
                ra_load_ar(e, ra,RBX, wb4, t);
                emit_cmp_reg32(e, RAX, RBX);
                emit8(e, 0x0F); emit8(e, 0x92); emit8(e, modrm(3, 0, RAX)); /* setb al */
                emit8(e, 0x0F); emit8(e, 0xB6); emit8(e, modrm(3, RAX, RAX));
                ra_store_ar(e, ra,RAX, wb4, r);
                return 1;
            }
            case 8: { /* MULL: ar = as * at */
                ra_load_ar(e, ra,RAX, wb4, s);
                ra_load_ar(e, ra,RBX, wb4, t);
                emit_imul_reg32(e, RAX, RBX);
                ra_store_ar(e, ra,RAX, wb4, r);
                return 1;
            }
            case 10: { /* MULUH: ar = (uint64)(as) * (uint64)(at) >> 32 */
                ra_load_ar(e, ra,RAX, wb4, s);
                ra_load_ar(e, ra,RBX, wb4, t);
                /* Use 64-bit multiply: need to zero-extend both to 64-bit */
                /* mov eax, eax already zero-extends in 64-bit mode */
                emit8(e, rex(1, RAX, 0, RBX)); emit8(e, 0x0F); emit8(e, 0xAF);
                emit8(e, modrm(3, RAX, RBX)); /* imul rax, rbx */
                emit_shr_reg64_imm(e, RAX, 32);
                ra_store_ar(e, ra,RAX, wb4, r);
                return 1;
            }
            case 11: { /* MULSH: ar = (int64)(int32)as * (int64)(int32)at >> 32 */
                ra_load_ar(e, ra,RAX, wb4, s);
                /* movsxd rax, eax */
                emit8(e, 0x48); emit8(e, 0x63); emit8(e, modrm(3, RAX, RAX));
                ra_load_ar(e, ra,RBX, wb4, t);
                emit8(e, 0x48); emit8(e, 0x63); emit8(e, modrm(3, RBX, RBX));
                emit8(e, rex(1, RAX, 0, RBX)); emit8(e, 0x0F); emit8(e, 0xAF);
                emit8(e, modrm(3, RAX, RBX));
                emit_shr_reg64_imm(e, RAX, 32);
                ra_store_ar(e, ra,RAX, wb4, r);
                return 1;
            }
            default: return 0;
            }
        } /* end RST2 */

        case 3: { /* RST3 */
            switch (op2) {
            case 0: { /* RSR: at = SR[sr] */
                int sr_num = XT_SR_NUM(insn);
                int32_t off = -1;
                switch (sr_num) {
                case XT_SR_SAR:      off = CPU_OFF_SAR; break;
                case XT_SR_LBEG:     off = CPU_OFF_LBEG; break;
                case XT_SR_LEND:     off = CPU_OFF_LEND; break;
                case XT_SR_LCOUNT:   off = CPU_OFF_LCOUNT; break;
                case XT_SR_BR:       off = CPU_OFF_BR; break;
                case XT_SR_PS:       off = CPU_OFF_PS; break;
                case XT_SR_WINDOWBASE: off = CPU_OFF_WINDOWBASE; break;
                case XT_SR_WINDOWSTART: off = CPU_OFF_WINDOWSTART; break;
                case XT_SR_INTENABLE: off = CPU_OFF_INTENABLE; break;
                case XT_SR_INTSET:   off = CPU_OFF_INTERRUPT; break;
                case XT_SR_CCOUNT:   off = CPU_OFF_CCOUNT; break;
                case XT_SR_VECBASE:  off = CPU_OFF_VECBASE; break;
                case XT_SR_EXCCAUSE: off = CPU_OFF_EXCCAUSE; break;
                case XT_SR_EXCVADDR: off = CPU_OFF_EXCVADDR; break;
                case XT_SR_SCOMPARE1: off = CPU_OFF_SCOMPARE1; break;
                case XT_SR_MISC0: off = (int32_t)(CPU_OFF_MISC + 0); break;
                case XT_SR_MISC1: off = (int32_t)(CPU_OFF_MISC + 4); break;
                case XT_SR_MISC2: off = (int32_t)(CPU_OFF_MISC + 8); break;
                case XT_SR_MISC3: off = (int32_t)(CPU_OFF_MISC + 12); break;
                case XT_SR_EPC1: case XT_SR_EPC2: case XT_SR_EPC3:
                case XT_SR_EPC4: case XT_SR_EPC5: case XT_SR_EPC6: case XT_SR_EPC7:
                    off = (int32_t)(CPU_OFF_EPC + (sr_num - XT_SR_EPC1) * 4); break;
                case XT_SR_EPS2: case XT_SR_EPS3: case XT_SR_EPS4:
                case XT_SR_EPS5: case XT_SR_EPS6: case XT_SR_EPS7:
                    off = (int32_t)(offsetof(xtensa_cpu_t, eps) + (sr_num - XT_SR_EPS2 + 1) * 4); break;
                case XT_SR_EXCSAVE1: case XT_SR_EXCSAVE2: case XT_SR_EXCSAVE3:
                case XT_SR_EXCSAVE4: case XT_SR_EXCSAVE5: case XT_SR_EXCSAVE6:
                case XT_SR_EXCSAVE7:
                    off = (int32_t)(CPU_OFF_EXCSAVE + (sr_num - XT_SR_EXCSAVE1) * 4); break;
                case XT_SR_CCOMPARE0: case XT_SR_CCOMPARE1: case XT_SR_CCOMPARE2:
                    off = (int32_t)(CPU_OFF_CCOMPARE + (sr_num - XT_SR_CCOMPARE0) * 4); break;
                case XT_SR_ACCLO:   off = CPU_OFF_ACCLO; break;
                case XT_SR_ACCHI:   off = CPU_OFF_ACCHI; break;
                case XT_SR_MR0: case XT_SR_MR1: case XT_SR_MR2: case XT_SR_MR3:
                    off = (int32_t)(CPU_OFF_MR + (sr_num - XT_SR_MR0) * 4); break;
                case XT_SR_LITBASE: off = CPU_OFF_LITBASE; break;
                case XT_SR_DEPC:    off = (int32_t)offsetof(xtensa_cpu_t, depc); break;
                default: return 0; /* Unknown SR: fall back */
                }
                emit_load_cpu32(e, RAX, off);
                ra_store_ar(e, ra,RAX, wb4, t);
                return 1;
            }
            case 1: { /* WSR: SR[sr] = at */
                int sr_num = XT_SR_NUM(insn);
                int32_t off = -1;
                switch (sr_num) {
                case XT_SR_SAR:      off = CPU_OFF_SAR; break;
                case XT_SR_LBEG:     off = CPU_OFF_LBEG; break;
                case XT_SR_LEND:     off = CPU_OFF_LEND; break;
                case XT_SR_LCOUNT:   off = CPU_OFF_LCOUNT; break;
                case XT_SR_BR:       off = CPU_OFF_BR; break;
                case XT_SR_SCOMPARE1: off = CPU_OFF_SCOMPARE1; break;
                case XT_SR_MISC0: off = (int32_t)(CPU_OFF_MISC + 0); break;
                case XT_SR_MISC1: off = (int32_t)(CPU_OFF_MISC + 4); break;
                case XT_SR_MISC2: off = (int32_t)(CPU_OFF_MISC + 8); break;
                case XT_SR_MISC3: off = (int32_t)(CPU_OFF_MISC + 12); break;
                case XT_SR_EPC1: case XT_SR_EPC2: case XT_SR_EPC3:
                case XT_SR_EPC4: case XT_SR_EPC5: case XT_SR_EPC6: case XT_SR_EPC7:
                    off = (int32_t)(CPU_OFF_EPC + (sr_num - XT_SR_EPC1) * 4); break;
                case XT_SR_EPS2: case XT_SR_EPS3: case XT_SR_EPS4:
                case XT_SR_EPS5: case XT_SR_EPS6: case XT_SR_EPS7:
                    off = (int32_t)(offsetof(xtensa_cpu_t, eps) + (sr_num - XT_SR_EPS2 + 1) * 4); break;
                case XT_SR_EXCSAVE1: case XT_SR_EXCSAVE2: case XT_SR_EXCSAVE3:
                case XT_SR_EXCSAVE4: case XT_SR_EXCSAVE5: case XT_SR_EXCSAVE6:
                case XT_SR_EXCSAVE7:
                    off = (int32_t)(CPU_OFF_EXCSAVE + (sr_num - XT_SR_EXCSAVE1) * 4); break;
                case XT_SR_VECBASE:  off = CPU_OFF_VECBASE; break;
                case XT_SR_EXCCAUSE: off = CPU_OFF_EXCCAUSE; break;
                case XT_SR_EXCVADDR: off = CPU_OFF_EXCVADDR; break;
                case XT_SR_DEPC:     off = (int32_t)offsetof(xtensa_cpu_t, depc); break;
                default: return 0;
                }
                ra_load_ar(e, ra,RAX, wb4, t);
                emit_store_cpu32(e, RAX, off);
                return 1;
            }
            case 2: { /* SEXT: ar = sign_extend(as, t+8) */
                int bits = t + 8;
                ra_load_ar(e, ra,RAX, wb4, s);
                /* Shift left then arithmetic shift right to sign extend */
                int shift = 32 - bits;
                if (shift > 0) {
                    emit_shl_reg32_imm(e, RAX, (uint8_t)shift);
                    emit_sar_reg32_imm(e, RAX, (uint8_t)shift);
                }
                ra_store_ar(e, ra,RAX, wb4, r);
                return 1;
            }
            case 3: { /* CLAMPS: clamp signed to -(2^(t+7)) .. (2^(t+7)-1) */
                int bits = t + 7;
                int32_t hi = (1 << bits) - 1;
                int32_t lo = -(1 << bits);
                ra_load_ar(e, ra,RAX, wb4, s);
                /* if (eax > hi) eax = hi; else if (eax < lo) eax = lo; */
                emit_cmp_reg32_imm32(e, RAX, hi);
                int gt_patch = emit_jcc_rel32(e, CC_G);
                emit_cmp_reg32_imm32(e, RAX, lo);
                int lt_patch = emit_jcc_rel32(e, CC_L);
                int done_patch = emit_jmp_rel32(e);
                emit_patch_rel32(e, gt_patch);
                emit_mov_reg_imm32(e, RAX, (uint32_t)hi);
                int done_patch2 = emit_jmp_rel32(e);
                emit_patch_rel32(e, lt_patch);
                emit_mov_reg_imm32(e, RAX, (uint32_t)lo);
                emit_patch_rel32(e, done_patch);
                emit_patch_rel32(e, done_patch2);
                ra_store_ar(e, ra,RAX, wb4, r);
                return 1;
            }
            case 4: { /* MIN (signed) */
                ra_load_ar(e, ra,RAX, wb4, s);
                ra_load_ar(e, ra,RBX, wb4, t);
                emit_cmp_reg32(e, RAX, RBX);
                emit_cmov_reg32(e, CC_G, RAX, RBX);
                ra_store_ar(e, ra,RAX, wb4, r);
                return 1;
            }
            case 5: { /* MAX (signed) */
                ra_load_ar(e, ra,RAX, wb4, s);
                ra_load_ar(e, ra,RBX, wb4, t);
                emit_cmp_reg32(e, RAX, RBX);
                emit_cmov_reg32(e, CC_L, RAX, RBX);
                ra_store_ar(e, ra,RAX, wb4, r);
                return 1;
            }
            case 6: { /* MINU (unsigned) */
                ra_load_ar(e, ra,RAX, wb4, s);
                ra_load_ar(e, ra,RBX, wb4, t);
                emit_cmp_reg32(e, RAX, RBX);
                emit_cmov_reg32(e, CC_A, RAX, RBX);
                ra_store_ar(e, ra,RAX, wb4, r);
                return 1;
            }
            case 7: { /* MAXU (unsigned) */
                ra_load_ar(e, ra,RAX, wb4, s);
                ra_load_ar(e, ra,RBX, wb4, t);
                emit_cmp_reg32(e, RAX, RBX);
                emit_cmov_reg32(e, CC_B, RAX, RBX);
                ra_store_ar(e, ra,RAX, wb4, r);
                return 1;
            }
            case 8: { /* MOVEQZ: if (at == 0) ar = as */
                ra_load_ar(e, ra,RBX, wb4, t);
                emit_test_reg32(e, RBX, RBX);
                int skip_patch = emit_jcc_rel32(e, CC_NE);
                ra_load_ar(e, ra,RAX, wb4, s);
                ra_store_ar(e, ra,RAX, wb4, r);
                emit_patch_rel32(e, skip_patch);
                return 1;
            }
            case 9: { /* MOVNEZ: if (at != 0) ar = as */
                ra_load_ar(e, ra,RBX, wb4, t);
                emit_test_reg32(e, RBX, RBX);
                int skip_patch = emit_jcc_rel32(e, CC_E);
                ra_load_ar(e, ra,RAX, wb4, s);
                ra_store_ar(e, ra,RAX, wb4, r);
                emit_patch_rel32(e, skip_patch);
                return 1;
            }
            case 10: { /* MOVLTZ: if ((int32)at < 0) ar = as */
                ra_load_ar(e, ra,RBX, wb4, t);
                emit_test_reg32(e, RBX, RBX);
                int skip_patch = emit_jcc_rel32(e, CC_NS);
                ra_load_ar(e, ra,RAX, wb4, s);
                ra_store_ar(e, ra,RAX, wb4, r);
                emit_patch_rel32(e, skip_patch);
                return 1;
            }
            case 11: { /* MOVGEZ: if ((int32)at >= 0) ar = as */
                ra_load_ar(e, ra,RBX, wb4, t);
                emit_test_reg32(e, RBX, RBX);
                int skip_patch = emit_jcc_rel32(e, CC_S);
                ra_load_ar(e, ra,RAX, wb4, s);
                ra_store_ar(e, ra,RAX, wb4, r);
                emit_patch_rel32(e, skip_patch);
                return 1;
            }
            case 12: { /* MOVF: if (!bt) ar = as */
                emit_load_cpu32(e, RBX, (int32_t)CPU_OFF_BR);
                emit8(e, 0xF6); emit8(e, modrm(3, 0, RBX)); emit8(e, (uint8_t)(1 << t)); /* test bl, imm8 */
                int skip_patch = emit_jcc_rel32(e, CC_NE);
                ra_load_ar(e, ra,RAX, wb4, s);
                ra_store_ar(e, ra,RAX, wb4, r);
                emit_patch_rel32(e, skip_patch);
                return 1;
            }
            case 13: { /* MOVT: if (bt) ar = as */
                emit_load_cpu32(e, RBX, (int32_t)CPU_OFF_BR);
                emit8(e, 0xF6); emit8(e, modrm(3, 0, RBX)); emit8(e, (uint8_t)(1 << t));
                int skip_patch = emit_jcc_rel32(e, CC_E);
                ra_load_ar(e, ra,RAX, wb4, s);
                ra_store_ar(e, ra,RAX, wb4, r);
                emit_patch_rel32(e, skip_patch);
                return 1;
            }
            case 14: { /* RUR */
                int ur = (s << 4) | r;
                if (ur == 232)      emit_load_cpu32(e, RAX, (int32_t)offsetof(xtensa_cpu_t, fcr));
                else if (ur == 233) emit_load_cpu32(e, RAX, (int32_t)offsetof(xtensa_cpu_t, fsr));
                else                emit_mov_reg_imm32(e, RAX, 0);
                ra_store_ar(e, ra,RAX, wb4, t);
                return 1;
            }
            case 15: { /* WUR */
                int ur = (s << 4) | r;
                ra_load_ar(e, ra,RAX, wb4, t);
                if (ur == 232)      emit_store_cpu32(e, RAX, (int32_t)offsetof(xtensa_cpu_t, fcr));
                else if (ur == 233) emit_store_cpu32(e, RAX, (int32_t)offsetof(xtensa_cpu_t, fsr));
                return 1;
            }
            default: return 0;
            }
        } /* end RST3 */

        case 4: case 5: { /* EXTUI: ar = (at >> shift) & mask */
            int shift = s | ((op1 & 1) << 4);
            uint32_t mask = (1u << (op2 + 1)) - 1;
            ra_load_ar(e, ra,RAX, wb4, t);
            if (shift > 0) emit_shr_reg32_imm(e, RAX, (uint8_t)shift);
            emit_and_reg32_imm32(e, RAX, (int32_t)mask);
            ra_store_ar(e, ra,RAX, wb4, r);
            return 1;
        }

        default: return 0;
        }
    } /* end QRST */

    case 1: { /* L32R: at = mem32[pc_aligned + sext(imm16 << 2)] */
        uint16_t imm16 = (uint16_t)XT_IMM16(insn);
        uint32_t target = (next_pc & ~3u) + (0xFFFC0000u | ((uint32_t)imm16 << 2));
        /* Load the literal value from guest memory */
        emit_mov_reg_imm32(e, RSI, target);
        emit_mem_read32(e, RSI, RBX);
        ra_store_ar(e, ra,RBX, wb4, t);
        return 1;
    }

    case 2: { /* LSAI: loads, stores, immediates */
        switch (r) {
        case 0x0: { /* L8UI */
            ra_load_ar(e, ra,RSI, wb4, s);
            emit_add_reg32_imm32(e, RSI, imm8);
            emit_mem_read8u(e, RSI, RBX);
            ra_store_ar(e, ra,RBX, wb4, t);
            return 1;
        }
        case 0x1: { /* L16UI */
            ra_load_ar(e, ra,RSI, wb4, s);
            emit_add_reg32_imm32(e, RSI, imm8 << 1);
            emit_mem_read16u(e, RSI, RBX);
            ra_store_ar(e, ra,RBX, wb4, t);
            return 1;
        }
        case 0x2: { /* L32I */
            ra_load_ar(e, ra,RSI, wb4, s);
            emit_add_reg32_imm32(e, RSI, imm8 << 2);
            emit_mem_read32(e, RSI, RBX);
            ra_store_ar(e, ra,RBX, wb4, t);
            return 1;
        }
        case 0x4: { /* S8I */
            ra_load_ar(e, ra,RSI, wb4, s);
            emit_add_reg32_imm32(e, RSI, imm8);
            ra_load_ar(e, ra,RBP, wb4, t);
            emit_mem_write8(e, RSI, RBP);
            return 1;
        }
        case 0x5: { /* S16I */
            ra_load_ar(e, ra,RSI, wb4, s);
            emit_add_reg32_imm32(e, RSI, imm8 << 1);
            ra_load_ar(e, ra,RBP, wb4, t);
            emit_mem_write16(e, RSI, RBP);
            return 1;
        }
        case 0x6: { /* S32I */
            ra_load_ar(e, ra,RSI, wb4, s);
            emit_add_reg32_imm32(e, RSI, imm8 << 2);
            ra_load_ar(e, ra,RBP, wb4, t);
            emit_mem_write32(e, RSI, RBP);
            return 1;
        }
        case 0x7: /* Cache ops — no-op */
            return 1;
        case 0x9: { /* L16SI */
            ra_load_ar(e, ra,RSI, wb4, s);
            emit_add_reg32_imm32(e, RSI, imm8 << 1);
            emit_mem_read16s(e, RSI, RBX);
            ra_store_ar(e, ra,RBX, wb4, t);
            return 1;
        }
        case 0xA: { /* MOVI: at = sext12(s:imm8) */
            int32_t imm12 = sign_extend(((uint32_t)s << 8) | (uint32_t)imm8, 12);
            emit_mov_reg_imm32(e, RAX, (uint32_t)imm12);
            ra_store_ar(e, ra,RAX, wb4, t);
            return 1;
        }
        case 0xB: { /* L32AI (acquire = no-op, same as L32I) */
            ra_load_ar(e, ra,RSI, wb4, s);
            emit_add_reg32_imm32(e, RSI, imm8 << 2);
            emit_mem_read32(e, RSI, RBX);
            ra_store_ar(e, ra,RBX, wb4, t);
            return 1;
        }
        case 0xC: { /* ADDI: at = as + sext8(imm8) */
            int32_t simm8 = sign_extend(imm8, 8);
            ra_load_ar(e, ra,RAX, wb4, s);
            emit_add_reg32_imm32(e, RAX, simm8);
            ra_store_ar(e, ra,RAX, wb4, t);
            return 1;
        }
        case 0xD: { /* ADDMI: at = as + sext8(imm8) << 8 */
            int32_t simm8 = sign_extend(imm8, 8);
            ra_load_ar(e, ra,RAX, wb4, s);
            emit_add_reg32_imm32(e, RAX, simm8 << 8);
            ra_store_ar(e, ra,RAX, wb4, t);
            return 1;
        }
        case 0xF: { /* S32RI (release = no-op, same as S32I) */
            ra_load_ar(e, ra,RSI, wb4, s);
            emit_add_reg32_imm32(e, RSI, imm8 << 2);
            ra_load_ar(e, ra,RBP, wb4, t);
            emit_mem_write32(e, RSI, RBP);
            return 1;
        }
        default: return 0;
        }
    } /* end LSAI */

    case 5: { /* CALLN: CALL4/CALL8/CALL12 */
        int call_nn = XT_N(insn);
        int32_t call_off = sign_extend(XT_OFFSET18(insn), 18);
        uint32_t call_target = (((pc >> 2) + (uint32_t)call_off + 1) << 2);
        uint32_t ret_addr = ((uint32_t)call_nn << 30) | (next_pc & 0x3FFFFFFFu);
        int32_t ret_ar_off = (int32_t)(CPU_OFF_AR + (((uint32_t)(wb4 + call_nn*4)) & 63) * 4);

        /* 1. PS.CALLINC = nn */
        emit_load_cpu32(e, RAX, (int32_t)CPU_OFF_PS);
        emit_and_reg32_imm32(e, RAX, (int32_t)(~(3u << 16)));
        emit_or_reg32_imm32(e, RAX, (int32_t)((uint32_t)call_nn << 16));
        emit_store_cpu32(e, RAX, (int32_t)CPU_OFF_PS);

        /* 2. Write return address to callee's a0 slot (bypasses regalloc — cross-window) */
        emit_store32_disp_imm(e, REG_CPU, ret_ar_off, ret_addr);

        /* 3. Block exit to callee */
        emit_block_exit_ra(e, ra, wb4, call_target, insn_idx + 1, jit);
        return 1;
    }

    case 6: { /* SI: J, BZ, BI0, BI1 */
        int nn = (insn >> 4) & 3;
        int m = (insn >> 6) & 3;

        if (nn == 0) {
            /* J: unconditional jump */
            int32_t offset = sign_extend(XT_OFFSET18(insn), 18);
            uint32_t target = next_pc + (uint32_t)offset + 1; /* +1 per ISA */
            emit_block_exit_ra(e, ra, wb4, target, insn_idx + 1, jit);
            return 1;
        }

        if (nn == 1) {
            /* BZ: BEQZ/BNEZ/BLTZ/BGEZ */
            int32_t imm12 = sign_extend(XT_IMM12(insn), 12);
            uint32_t target = next_pc + (uint32_t)imm12 + 1; /* +1 per ISA */
            ra_load_ar(e, ra,RAX, wb4, s);
            emit_test_reg32(e, RAX, RAX);
            uint8_t cc;
            switch (m) {
            case 0: cc = CC_E; break;   /* BEQZ */
            case 1: cc = CC_NE; break;  /* BNEZ */
            case 2: cc = CC_S; break;   /* BLTZ (sign flag) */
            case 3: cc = CC_NS; break;  /* BGEZ */
            default: return 0;
            }
            int taken_patch = emit_jcc_rel32(e, cc);
            emit_block_exit_ra(e, ra, wb4, next_pc, insn_idx + 1, jit);
            emit_patch_rel32(e, taken_patch);
            emit_block_exit_ra(e, ra, wb4, target, insn_idx + 1, jit);
            return 1;
        }

        if (nn == 2) {
            /* BI0: BEQI/BNEI/BLTI/BGEI with B4CONST table */
            static const int32_t b4c[16] = {
                -1, 1, 2, 3, 4, 5, 6, 7, 8, 10, 12, 16, 32, 64, 128, 256
            };
            int lr = XT_R(insn);
            int32_t offset8 = sign_extend(imm8, 8);
            uint32_t target = next_pc + (uint32_t)offset8 + 1; /* +1 per ISA */
            ra_load_ar(e, ra,RAX, wb4, s);
            emit_cmp_reg32_imm32(e, RAX, b4c[lr]);
            uint8_t cc;
            switch (m) {
            case 0: cc = CC_E; break;   /* BEQI */
            case 1: cc = CC_NE; break;  /* BNEI */
            case 2: cc = CC_L; break;   /* BLTI */
            case 3: cc = CC_GE; break;  /* BGEI */
            default: return 0;
            }
            int taken_patch = emit_jcc_rel32(e, cc);
            emit_block_exit_ra(e, ra, wb4, next_pc, insn_idx + 1, jit);
            emit_patch_rel32(e, taken_patch);
            emit_block_exit_ra(e, ra, wb4, target, insn_idx + 1, jit);
            return 1;
        }

        if (nn == 3) {
            /* BI1: ENTRY, BF/BT, LOOP, BLTUI/BGEUI */
            if (m == 1) {
                int lr = XT_R(insn);
                if (lr == 0 || lr == 1) {
                    /* BF / BT */
                    int32_t offset8 = sign_extend(imm8, 8);
                    uint32_t target = next_pc + (uint32_t)offset8 + 1; /* +1 per ISA */
                    emit_load_cpu32(e, RBX, (int32_t)CPU_OFF_BR);
                    emit8(e, 0xF6); emit8(e, modrm(3, 0, RBX)); emit8(e, (uint8_t)(1 << s));
                    if (lr == 0) {
                        /* BF: taken if bit NOT set */
                        int taken_patch = emit_jcc_rel32(e, CC_E);
                        emit_block_exit_ra(e, ra, wb4, next_pc, insn_idx + 1, jit);
                        emit_patch_rel32(e, taken_patch);
                        emit_block_exit_ra(e, ra, wb4, target, insn_idx + 1, jit);
                    } else {
                        /* BT: taken if bit set */
                        int taken_patch = emit_jcc_rel32(e, CC_NE);
                        emit_block_exit_ra(e, ra, wb4, next_pc, insn_idx + 1, jit);
                        emit_patch_rel32(e, taken_patch);
                        emit_block_exit_ra(e, ra, wb4, target, insn_idx + 1, jit);
                    }
                    return 1;
                }
                return 0; /* LOOP */
            }
            if (m == 2 || m == 3) {
                /* BLTUI / BGEUI */
                static const uint32_t b4cu[16] = {
                    32768, 65536, 2, 3, 4, 5, 6, 7, 8, 10, 12, 16, 32, 64, 128, 256
                };
                int lr = XT_R(insn);
                int32_t offset8 = sign_extend(imm8, 8);
                uint32_t target = next_pc + (uint32_t)offset8 + 1; /* +1 per ISA */
                ra_load_ar(e, ra,RAX, wb4, s);
                emit_cmp_reg32_imm32(e, RAX, (int32_t)b4cu[lr]);
                uint8_t cc = (m == 2) ? CC_B : CC_AE;
                int taken_patch = emit_jcc_rel32(e, cc);
                emit_block_exit_ra(e, ra, wb4, next_pc, insn_idx + 1, jit);
                emit_patch_rel32(e, taken_patch);
                emit_block_exit_ra(e, ra, wb4, target, insn_idx + 1, jit);
                return 1;
            }
            if (m == 0) {
                /* ENTRY: callee prologue — rotate window, set SP, guard overflow */
                int entry_s = XT_S(insn);
                uint32_t frame_size = (uint32_t)XT_IMM12(insn) << 3;

                /* Overflow guard: if POPCNT(windowstart) >= 14, fallback */
                emit_load_cpu32(e, RDX, (int32_t)CPU_OFF_WINDOWSTART);
                emit_popcnt(e, RDX, RDX);
                emit_cmp_reg32_imm32(e, RDX, 14);
                int overflow_fb = emit_jcc_rel32(e, CC_AE);

                /* Load PS → RAX; extract CALLINC → RCX */
                emit_load_cpu32(e, RAX, (int32_t)CPU_OFF_PS);
                emit_mov_reg32_reg32(e, RCX, RAX);
                emit_shr_reg32_imm(e, RCX, 16);
                emit_and_reg32_imm32(e, RCX, 3);

                /* Save old_wb → RBX */
                emit_load_cpu32(e, RBX, (int32_t)CPU_OFF_WINDOWBASE);

                /* FLUSH DIRTY REGS before windowbase changes */
                ra_flush(e, ra, wb4);

                /* Compute new SP: load current as (caller's ar[s]), subtract frame_size */
                /* entry_s is in caller's window (current wb4) */
                emit_load32_disp(e, RSI, REG_CPU, ar_offset(wb4, entry_s));
                emit_sub_reg32_imm32(e, RSI, (int32_t)frame_size);

                /* Compute new_wb = (old_wb + callinc) & 15 */
                emit_mov_reg32_reg32(e, RDX, RBX);
                emit_add_reg32(e, RDX, RCX);
                emit_and_reg32_imm32(e, RDX, 15);

                /* Write new SP to ar[(new_wb*4+1) & 63] */
                /* Compute index: new_wb*4+1 = RDX*4+1 */
                emit_mov_reg32_reg32(e, RDI, RDX);
                emit_shl_reg32_imm(e, RDI, 2);
                emit_add_reg32_imm32(e, RDI, 1);
                emit_and_reg32_imm32(e, RDI, 63);
                /* Store: cpu->ar[edi] = esi */
                emit_store32_sib(e, RSI, REG_CPU, RDI, (int32_t)CPU_OFF_AR);

                /* Store new_wb → WINDOWBASE */
                emit_store_cpu32(e, RDX, (int32_t)CPU_OFF_WINDOWBASE);

                /* Set WS[new_wb]: mov eax,1; shl eax,cl; or [ws], eax */
                emit_mov_reg_imm32(e, RAX, 1);
                emit_mov_reg32_reg32(e, RCX, RDX);  /* cl = new_wb */
                emit_shl_reg32_cl(e, RAX);
                emit_or_mem32_reg(e, REG_CPU, (int32_t)CPU_OFF_WINDOWSTART, RAX);

                /* Update PS: OWB=old_wb, CALLINC=0 */
                emit_load_cpu32(e, RAX, (int32_t)CPU_OFF_PS);
                emit_and_reg32_imm32(e, RAX, (int32_t)(~((3u << 16) | (0xFu << 8))));
                emit_mov_reg32_reg32(e, RCX, RBX);  /* old_wb */
                emit_shl_reg32_imm(e, RCX, 8);
                emit_or_reg32(e, RAX, RCX);
                emit_store_cpu32(e, RAX, (int32_t)CPU_OFF_PS);

                /* Exit to pc+3 (ENTRY is always 3 bytes) — no dirty flush needed (done above) */
                emit_store_cpu32_imm(e, (int32_t)CPU_OFF_PC, pc + 3);
                emit_mov_reg_imm32(e, RAX, (uint32_t)(insn_idx + 1));
                /* Record chain slot for the target */
                if (jit && jit->chain_slot_count < MAX_CHAIN_SLOTS) {
                    /* new_wb was computed at compile time from callinc;
                     * but callinc is only known at runtime. We can't chain ENTRY
                     * because the target wb depends on runtime CALLINC.
                     * Just jump to epilogue without a chain slot. */
                }
                emit_jmp_to_epilogue(e, jit);

                /* Overflow fallback: interpreter handles it */
                emit_patch_rel32(e, overflow_fb);
                ra_flush_all(e, wb4);
                emit_store_cpu32_imm(e, (int32_t)CPU_OFF_PC, pc);
                emit_mov_reg_imm32(e, RAX, 0);
                emit_jmp_to_epilogue(e, jit);
                return 1;
            }
        }
        return 0;
    } /* end SI */

    case 7: { /* B: RRI8 conditional branches */
        int32_t offset = sign_extend(imm8, 8);
        uint32_t target = next_pc + (uint32_t)offset + 1; /* +1 per ISA */
        ra_load_ar(e, ra,RAX, wb4, s);
        ra_load_ar(e, ra,RBX, wb4, t);

        uint8_t cc;
        int is_bit_test = 0;
        switch (r) {
        case 0: /* BNONE: (as & at) == 0 */
            emit_test_reg32(e, RAX, RBX);
            cc = CC_E; break;
        case 1: /* BEQ */
            emit_cmp_reg32(e, RAX, RBX);
            cc = CC_E; break;
        case 2: /* BLT (signed) */
            emit_cmp_reg32(e, RAX, RBX);
            cc = CC_L; break;
        case 3: /* BLTU (unsigned) */
            emit_cmp_reg32(e, RAX, RBX);
            cc = CC_B; break;
        case 4: /* BALL: (~as & at) == 0 */
            emit_mov_reg32_reg32(e, RCX, RAX);
            /* not ecx */
            emit_rex(e, 0, 0, RCX);
            emit8(e, 0xF7); emit8(e, modrm(3, 2, RCX));
            emit_test_reg32(e, RCX, RBX);
            cc = CC_E; break;
        case 5: /* BBC: !(as & (1 << (at & 31))) */
            emit_mov_reg32_reg32(e, RCX, RBX);
            emit_and_reg32_imm32(e, RCX, 31);
            /* bt eax, ecx */
            emit8(e, 0x0F); emit8(e, 0xA3); emit8(e, modrm(3, RCX, RAX));
            cc = CC_AE; /* CF=0 means bit clear — jnc/jae */
            is_bit_test = 1; break;
        case 6: case 7: { /* BBCI */
            int bit = t | ((r & 1) << 4);
            /* bt eax, imm8 */
            emit8(e, 0x0F); emit8(e, 0xBA); emit8(e, modrm(3, 4, RAX)); emit8(e, (uint8_t)bit);
            cc = CC_AE; /* CF=0 */
            is_bit_test = 1; break;
        }
        case 8: /* BANY: (as & at) != 0 */
            emit_test_reg32(e, RAX, RBX);
            cc = CC_NE; break;
        case 9: /* BNE */
            emit_cmp_reg32(e, RAX, RBX);
            cc = CC_NE; break;
        case 10: /* BGE (signed) */
            emit_cmp_reg32(e, RAX, RBX);
            cc = CC_GE; break;
        case 11: /* BGEU (unsigned) */
            emit_cmp_reg32(e, RAX, RBX);
            cc = CC_AE; break;
        case 12: /* BNALL: (~as & at) != 0 */
            emit_mov_reg32_reg32(e, RCX, RAX);
            emit_rex(e, 0, 0, RCX);
            emit8(e, 0xF7); emit8(e, modrm(3, 2, RCX));
            emit_test_reg32(e, RCX, RBX);
            cc = CC_NE; break;
        case 13: /* BBS: (as & (1 << (at & 31))) != 0 */
            emit_mov_reg32_reg32(e, RCX, RBX);
            emit_and_reg32_imm32(e, RCX, 31);
            emit8(e, 0x0F); emit8(e, 0xA3); emit8(e, modrm(3, RCX, RAX));
            cc = CC_B; /* CF=1 means bit set — jc/jb */
            is_bit_test = 1; break;
        case 14: case 15: { /* BBSI */
            int bit = t | ((r & 1) << 4);
            emit8(e, 0x0F); emit8(e, 0xBA); emit8(e, modrm(3, 4, RAX)); emit8(e, (uint8_t)bit);
            cc = CC_B; /* CF=1 */
            is_bit_test = 1; break;
        }
        default: return 0;
        }
        (void)is_bit_test;

        int taken_patch = emit_jcc_rel32(e, cc);
        emit_block_exit_ra(e, ra, wb4, next_pc, insn_idx + 1, jit);
        emit_patch_rel32(e, taken_patch);
        emit_block_exit_ra(e, ra, wb4, target, insn_idx + 1, jit);
        return 1;
    } /* end B */

    default:
        return 0;
    }

    /* RETW / RETW.N handler — reached via goto from RETW and RETW.N cases */
    if (0) {
compile_retw: ;
        /* 1. Load a0 directly (spilled, not in regalloc) */
        emit_load32_disp(e, RAX, REG_CPU, ar_offset(wb4, 0));

        /* 2. Extract nn = a0[31:30] */
        emit_mov_reg32_reg32(e, RCX, RAX);
        emit_shr_reg32_imm(e, RCX, 30);
        emit_and_reg32_imm32(e, RCX, 3);

        /* 3. Compute return_pc = (a0 & 0x3FFFFFFF) */
        emit_and_reg32_imm32(e, RAX, 0x3FFFFFFF);
        /* Add high bits from current PC */
        emit_or_reg32_imm32(e, RAX, (int32_t)(pc & 0xC0000000u));
        emit_mov_reg32_reg32(e, RSI, RAX);  /* RSI = return_pc */

        /* 4. Compute ret_wb = (windowbase - nn) & 15 */
        emit_load_cpu32(e, RDX, (int32_t)CPU_OFF_WINDOWBASE);
        emit_sub_reg32(e, RDX, RCX);
        emit_and_reg32_imm32(e, RDX, 15);  /* RDX = ret_wb */

        /* 5. Underflow guard: check WS[ret_wb], fallback if clear */
        emit_load_cpu32(e, RBX, (int32_t)CPU_OFF_WINDOWSTART);
        emit_bt_reg_reg(e, RBX, RDX);  /* test bit ret_wb of WS */
        int fill_fb = emit_jcc_rel32(e, CC_AE);  /* CF=0 → bit clear → need fill */

        /* 6. Flush dirty regs BEFORE window rotation */
        ra_flush(e, ra, wb4);

        /* 7. Clear WS[current_wb] */
        emit_load_cpu32(e, RCX, (int32_t)CPU_OFF_WINDOWBASE);
        emit_mov_reg_imm32(e, RAX, 1);
        emit_shl_reg32_cl(e, RAX);
        emit_not_reg32(e, RAX);
        emit_and_reg32(e, RBX, RAX);
        emit_store_cpu32(e, RBX, (int32_t)CPU_OFF_WINDOWSTART);

        /* 8. Store ret_wb → WINDOWBASE */
        emit_store_cpu32(e, RDX, (int32_t)CPU_OFF_WINDOWBASE);

        /* 9. Update PS: OWB = ret_wb, CALLINC = 0 */
        emit_load_cpu32(e, RAX, (int32_t)CPU_OFF_PS);
        emit_and_reg32_imm32(e, RAX, (int32_t)(~((3u << 16) | (0xFu << 8))));
        emit_mov_reg32_reg32(e, RCX, RDX);  /* ret_wb */
        emit_shl_reg32_imm(e, RCX, 8);
        emit_or_reg32(e, RAX, RCX);
        emit_store_cpu32(e, RAX, (int32_t)CPU_OFF_PS);

        /* 10. Store return_pc → cpu->pc, exit with insn_count */
        emit_store_cpu32(e, RSI, (int32_t)CPU_OFF_PC);
        emit_store32_disp_imm(e, REG_CPU, (int32_t)CPU_OFF_PC_WRITTEN, 1);
        emit_mov_reg_imm32(e, RAX, (uint32_t)(insn_idx + 1));
        /* No chain slot — dynamic target */
        emit_jmp_to_epilogue(e, jit);

        /* Fill fallback: interpreter handles it */
        emit_patch_rel32(e, fill_fb);
        /* Unconditionally flush all regs (compile-time dirty bits already cleared
         * by the main path's ra_flush, but at runtime this path is taken instead) */
        ra_flush_all(e, wb4);
        emit_store_cpu32_imm(e, (int32_t)CPU_OFF_PC, pc);
        emit_mov_reg_imm32(e, RAX, 0);
        emit_jmp_to_epilogue(e, jit);
        return 1;
    }

    return 0; /* unreachable */
}

/* Jump to shared epilogue stub */
static void emit_jmp_to_epilogue(emit_t *e, jit_state_t *jit) {
    emit_jmp_rel32_to(e, jit->epilogue_stub);
}

/* Emit the block exit sequence WITH register allocation:
 * 1. Flush dirty regs
 * 2. Store exit PC (if known)
 * 3. Set return value
 * 4. Record chain slot (if static target) and jmp to epilogue */
static void emit_block_exit_ra(emit_t *e, regalloc_t *ra, int wb4,
                               uint32_t exit_pc, int insn_count,
                               jit_state_t *jit) {
    /* Flush dirty regs to memory */
    ra_flush(e, ra, wb4);

    if (exit_pc != 0) {
        emit_store_cpu32_imm(e, (int32_t)CPU_OFF_PC, exit_pc);
    }
    emit_mov_reg_imm32(e, RAX, (uint32_t)insn_count);

    /* Record chain slot for static targets */
    if (exit_pc != 0 && jit && jit->chain_slot_count < MAX_CHAIN_SLOTS) {
        chain_slot_t *slot = &jit->chain_slots[jit->chain_slot_count++];
        slot->target_pc = exit_pc;
        slot->target_wb = (uint32_t)(wb4 / 4);
        slot->jmp_site = e->ptr;  /* points to the 0xE9 byte */
    }

    emit_jmp_to_epilogue(e, jit);
}

/* (Legacy emit_block_exit removed — all exits go through emit_block_exit_ra) */

/* Chain newly compiled block: patch any pending chain slots that target this (pc, wb) */
static void jit_chain_new_block(jit_state_t *jit, uint32_t pc, uint32_t wb, uint8_t *entry_ptr) {
    for (int i = 0; i < jit->chain_slot_count; i++) {
        chain_slot_t *slot = &jit->chain_slots[i];
        if (slot->target_pc == pc && slot->target_wb == wb) {
            /* Patch the jmp rel32: the jmp_site points to the 0xE9 byte */
            uint8_t *jmp = slot->jmp_site;
            int32_t rel = (int32_t)(entry_ptr - (jmp + 5));
            memcpy(jmp + 1, &rel, 4);
            /* Mark inactive */
            slot->target_pc = 0;
            jit->stats.chains_patched++;
        }
    }
}

/* Compile a block and return the function pointer */
static jit_block_fn jit_compile_block(jit_state_t *jit, xtensa_cpu_t *cpu,
                                      uint32_t pc, jit_scan_t *scan) {
    /* Check code cache space (worst case: ~512 bytes per guest instruction for ENTRY/RETW) */
    size_t needed = (size_t)scan->count * 512 + 512;
    if (jit->code_size + needed > jit->code_capacity) {
        jit_flush(jit);
    }

    uint8_t *code_start = jit->code_cache + jit->code_size;
    emit_t e;
    emit_init(&e, code_start, jit->code_capacity - jit->code_size);

    /* Prologue: save callee-saved registers.
     * 6 pushes + return address = 7 slots = 56 bytes → RSP % 16 = 8.
     * sub rsp,8 to realign to 16 before any C calls (mem_read/write slow paths). */
    emit_push(&e, RBX);
    emit_push(&e, RBP);
    emit_push(&e, R15);
    emit_push(&e, R14);
    emit_push(&e, R13);
    emit_push(&e, R12);
    emit_sub_reg64_imm32(&e, RSP, 8);

    /* rdi = cpu pointer (System V ABI first arg) */
    emit_mov_reg_reg(&e, REG_CPU, RDI);  /* r15 = cpu */

    /* Load mem pointer */
    emit_load64_disp(&e, REG_MEM, REG_CPU, (int32_t)CPU_OFF_MEM);

    /* Windowbase * 4 — compile-time constant per block */
    int wb4 = (int)(cpu->windowbase * 4);

    /* Chain entry point: chained blocks jump here (stack already has
     * callee-saved regs, R15=cpu, R14=mem). Timer check and preload follow. */
    uint8_t *chain_entry = e.ptr;

    /* Timer check: if ccount >= next_timer_event, return 0 (defer to jit_run) */
    emit_load_cpu32(&e, RAX, (int32_t)CPU_OFF_CCOUNT);
    emit_cmp32_mem(&e, RAX, REG_CPU, (int32_t)CPU_OFF_NEXT_TIMER);
    int timer_ok = emit_jcc_rel32(&e, CC_B);
    emit_mov_reg_imm32(&e, RAX, 0);
    emit_jmp_to_epilogue(&e, jit);
    emit_patch_rel32(&e, timer_ok);

    /* Initialize register allocator and pre-load allocated regs */
    regalloc_t ra = {0, 0};
    ra_preload(&e, &ra, wb4);

    /* Compile each instruction */
    int last_compiled = 0;
    for (int i = 0; i < scan->count; i++) {
        uint32_t next_pc = (i + 1 < scan->count) ? scan->pcs[i + 1] : scan->end_pc;
        int ok = jit_compile_insn(&e, wb4, scan->insns[i], scan->ilens[i],
                                  scan->pcs[i], next_pc, i, &ra, jit);
        if (!ok) {
            if (i == 0) return NULL;
            /* End block before this instruction */
            emit_block_exit_ra(&e, &ra, wb4, scan->pcs[i], i, jit);
            last_compiled = i;
            break;
        }
        last_compiled = i + 1;
    }

    /* If last instruction wasn't a terminator, add fallthrough exit */
    if (last_compiled == scan->count) {
        int last_cls = classify_for_jit(scan->insns[scan->count - 1], scan->ilens[scan->count - 1]);
        if (last_cls == 0) {
            emit_block_exit_ra(&e, &ra, wb4, scan->end_pc, scan->count, jit);
        }
    }

    if (!emit_ok(&e)) return NULL;

    jit->code_size += emit_size(&e);
    jit->stats.blocks_compiled++;
    jit->last_chain_entry = chain_entry;

    /* Patch any pending chain slots that target this block.
     * Chain slots jump to chain_entry (past prologue), not code_start. */
    jit_chain_new_block(jit, pc, cpu->windowbase, chain_entry);

    /* Also patch our own chain slots that target already-compiled blocks */
    for (int i = 0; i < jit->chain_slot_count; i++) {
        chain_slot_t *slot = &jit->chain_slots[i];
        if (slot->target_pc == 0) continue;
        jit_block_t *target_b = jit_lookup(jit, slot->target_pc, slot->target_wb);
        if (target_b && target_b->chain_entry) {
            uint8_t *jmp = slot->jmp_site;
            int32_t rel = (int32_t)((uint8_t *)target_b->chain_entry - (jmp + 5));
            memcpy(jmp + 1, &rel, 4);
            slot->target_pc = 0;
            jit->stats.chains_patched++;
        }
    }

    return (jit_block_fn)code_start;
}


/* ===== Public API ===== */

jit_state_t *jit_init(void) {
    jit_state_t *jit = calloc(1, sizeof(jit_state_t));
    if (!jit) return NULL;

    /* mmap executable code cache */
    jit->code_cache = mmap(NULL, JIT_CODE_CACHE_SIZE,
                           PROT_READ | PROT_WRITE | PROT_EXEC,
                           MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
    if (jit->code_cache == MAP_FAILED) {
        free(jit);
        return NULL;
    }
    jit->code_capacity = JIT_CODE_CACHE_SIZE;
    jit->code_size = 0;

    /* Emit shared epilogue stub at the start of the code cache.
     * Matches prologue: 6 pushes + sub rsp,8 → add rsp,8 + 6 pops + ret. */
    emit_t stub_e;
    emit_init(&stub_e, jit->code_cache, 64);
    jit->epilogue_stub = stub_e.ptr;
    emit_add_reg64_imm32(&stub_e, RSP, 8);
    emit_pop(&stub_e, R12);
    emit_pop(&stub_e, R13);
    emit_pop(&stub_e, R14);
    emit_pop(&stub_e, R15);
    emit_pop(&stub_e, RBP);
    emit_pop(&stub_e, RBX);
    emit_ret(&stub_e);
    jit->code_size = emit_size(&stub_e);
    jit->chain_slot_count = 0;

    fprintf(stderr, "[JIT] Initialized: %u MB code cache, %u-entry hash table, epilogue at +0\n",
            JIT_CODE_CACHE_SIZE / (1024 * 1024), JIT_HASH_SIZE);

    return jit;
}

/* Set a bit in the JIT bitmap for a compiled block PC */
static void jit_bitmap_set(jit_state_t *jit, uint32_t pc) {
    uint32_t idx = (pc >> 2) & (HOOK_BITMAP_BITS - 1);
    jit->jit_bitmap[idx / 64] |= (1ULL << (idx & 63));
}

/* JIT pc_hook: called by the interpreter for every bitmap-marked PC.
 * Direct hash lookup (no JIT bitmap check) — O(1) with one memory access.
 * Flow: check JIT hash → if hit, run block; if miss, forward to ROM stubs. */
static int jit_pc_hook(xtensa_cpu_t *cpu, uint32_t pc, void *ctx) {
    jit_state_t *jit = ctx;

    /* Skip JIT lookup for ROM range (0x40000000-0x4006FFFF) — always stubs.
     * This avoids a hash table access for the ~35M stub calls per 100M cycles. */
    if (__builtin_expect(pc < 0x40070000u, 0)) {
        if (jit->original_hook)
            return jit->original_hook(cpu, pc, jit->original_hook_ctx);
        return 0;
    }

    /* Direct hash table probe for firmware-space PCs */
    uint32_t wb = cpu->windowbase;
    uint32_t hidx = ((pc >> 2) ^ (wb * 2654435761u)) & JIT_HASH_MASK;
    uint32_t tag = pc ^ (wb << 28);
    jit_block_t *b = &jit->hash[hidx];

    jit_block_fn fn = NULL;
    if (__builtin_expect(b->code != NULL && b->pc == tag, 1)) {
        fn = (jit_block_fn)b->code;
    } else {
        /* Hash miss — try hot-counting and compilation */
        fn = jit_get_block(jit, cpu, pc);
    }

    if (fn) {
        int block_insns = fn(cpu);

        if (block_insns > 0) {
            /* Apply zero-overhead loop */
            if (cpu->lcount > 0 && cpu->pc == cpu->lend) {
                cpu->lcount--;
                cpu->pc = cpu->lbeg;
            }

            /* Advance ccount by block_insns - 1 (the interpreter adds 1) */
            cpu->ccount += (uint32_t)(block_insns - 1);
            cpu->cycle_count += (uint64_t)(block_insns - 1);

            jit->stats.blocks_executed++;
            jit->stats.insns_jitted += (uint64_t)block_insns;

            return 1; /* Handled */
        }
    }

    /* Forward to original hook (ROM stubs) */
    if (jit->original_hook) {
        return jit->original_hook(cpu, pc, jit->original_hook_ctx);
    }

    return 0;
}

/* Install JIT as a pc_hook, chaining with the existing hook */
void jit_install_hook(jit_state_t *jit, xtensa_cpu_t *cpu) {
    /* Save the original hook */
    jit->original_hook = cpu->pc_hook;
    jit->original_hook_ctx = cpu->pc_hook_ctx;

    /* Install JIT hook */
    cpu->pc_hook = jit_pc_hook;
    cpu->pc_hook_ctx = jit;

    /* The bitmap stays the same — JIT bits are added on top of ROM stub bits.
     * The interpreter's bitmap test will fire for both stubs and JIT blocks. */
}

void jit_destroy(jit_state_t *jit) {
    if (!jit) return;
    if (jit->code_cache && jit->code_cache != MAP_FAILED)
        munmap(jit->code_cache, jit->code_capacity);
    free(jit);
}

void jit_flush(jit_state_t *jit) {
    if (!jit) return;
    memset(jit->hash, 0, sizeof(jit->hash));
    /* Reset code cache but preserve epilogue stub. Re-emit it to be safe. */
    emit_t stub_e;
    emit_init(&stub_e, jit->code_cache, 64);
    jit->epilogue_stub = stub_e.ptr;
    emit_add_reg64_imm32(&stub_e, RSP, 8);
    emit_pop(&stub_e, R12);
    emit_pop(&stub_e, R13);
    emit_pop(&stub_e, R14);
    emit_pop(&stub_e, R15);
    emit_pop(&stub_e, RBP);
    emit_pop(&stub_e, RBX);
    emit_ret(&stub_e);
    jit->code_size = emit_size(&stub_e);
    jit->chain_slot_count = 0;
    jit->stats.cache_flushes++;
}

jit_block_fn jit_get_block(jit_state_t *jit, xtensa_cpu_t *cpu, uint32_t pc) {
    uint32_t wb = cpu->windowbase;
    jit_block_t *b = jit_lookup(jit, pc, wb);
    if (b && b->code)
        return (jit_block_fn)b->code;

    /* Get or create entry */
    b = jit_get_or_create(jit, pc, wb);
    b->exec_count++;

    if (b->exec_count < JIT_HOT_THRESHOLD)
        return NULL;  /* Not hot yet */

    /* Scan the block */
    jit_scan_t scan;
    jit_scan_block(jit, cpu, pc, &scan);

    if (scan.count < 4)
        return NULL;  /* Block too small — prologue/epilogue overhead dominates */

    /* Compile */
    jit_block_fn fn = jit_compile_block(jit, cpu, pc, &scan);
    if (fn) {
        b->code = (void *)fn;
        b->chain_entry = (void *)jit->last_chain_entry;
        b->guest_insns = (uint16_t)scan.count;

        /* Set JIT bitmap bit so the interpreter's hook fires for this PC.
         * Also set the bit in the CPU's hook bitmap (ROM bitmap) so the
         * interpreter's fast-path bitmap test catches it. */
        jit_bitmap_set(jit, pc);
        if (cpu->pc_hook_bitmap) {
            /* Const-cast: we need to modify the shared bitmap */
            uint64_t *bm = (uint64_t *)(uintptr_t)cpu->pc_hook_bitmap;
            uint32_t idx = (pc >> 2) & (HOOK_BITMAP_BITS - 1);
            bm[idx / 64] |= (1ULL << (idx & 63));
        }
    }

    return fn;
}

/* Main JIT execution loop.
 * With jit_install_hook, the interpreter's pc_hook dispatches compiled JIT
 * blocks transparently via the bitmap. jit_run uses 1000-instruction
 * interpreter batches (no cold-code regression). After each batch, it
 * hot-counts the current PC to trigger compilation. Once compiled, the
 * bitmap ensures the hook fires on every subsequent visit. */
__attribute__((hot))
int jit_run(jit_state_t *jit, xtensa_cpu_t *cpu, int max_cycles) {
    uint32_t ccount_start = cpu->ccount;
    uint64_t jit_insns_before = jit->stats.insns_jitted;

    while (__builtin_expect(cpu->running, 1) &&
           __builtin_expect(!cpu->breakpoint_hit, 1)) {

        int32_t done = (int32_t)(cpu->ccount - ccount_start);
        if (done >= max_cycles) break;
        int remaining = max_cycles - done;

        if (__builtin_expect(cpu->halted, 0)) {
            xtensa_run(cpu, 1);
            continue;
        }

        int batch = remaining < 1000 ? remaining : 1000;
        int ran = xtensa_run(cpu, batch);

        /* Hot-counting: check if current PC is a JIT candidate.
         * Triggers compilation for frequently-visited firmware PCs.
         * Once compiled, the bitmap ensures the hook dispatches directly. */
        uint32_t pc = cpu->pc;
        if (__builtin_expect(pc >= 0x40070000u && pc < 0x40500000u, 1)) {
            jit_get_block(jit, cpu, pc);
        }

        if (__builtin_expect(ran < batch, 0)) {
            if (!cpu->running || cpu->halted || cpu->breakpoint_hit) break;
        }
    }

    uint32_t total_done = cpu->ccount - ccount_start;
    /* Derive interp insns: total ccount advance minus JIT insns added by hook.
     * Each hook execution: interpreter counts 1 step, hook adds block_insns-1
     * to ccount. So ccount_delta = interp_steps + sum(block_insns_i - 1).
     * And insns_jitted = sum(block_insns_i). Therefore:
     * interp_steps = ccount_delta - (insns_jitted_delta - blocks_executed_delta) */
    uint64_t jit_insns_delta = jit->stats.insns_jitted - jit_insns_before;
    if (total_done > jit_insns_delta) {
        jit->stats.insns_interp += (uint64_t)total_done - jit_insns_delta;
    }
    return (int)total_done;
}

const jit_stats_t *jit_get_stats(const jit_state_t *jit) {
    return &jit->stats;
}

void jit_set_verify(jit_state_t *jit, bool enable) {
    if (jit) jit->verify = enable;
}

void jit_print_stats(const jit_state_t *jit) {
    const jit_stats_t *s = &jit->stats;
    /* insns_jitted is tracked by the hook; interp insns derived from total */
    uint64_t total = s->insns_jitted + s->insns_interp;
    /* If insns_interp wasn't tracked (hook-based mode), show what we have */
    fprintf(stderr, "\n[JIT Statistics]\n");
    fprintf(stderr, "  Blocks compiled: %llu\n", (unsigned long long)s->blocks_compiled);
    fprintf(stderr, "  Blocks executed: %llu\n", (unsigned long long)s->blocks_executed);
    fprintf(stderr, "  Insns JIT:       %llu (%.1f%%)\n",
            (unsigned long long)s->insns_jitted,
            total > 0 ? 100.0 * (double)s->insns_jitted / (double)total : 0.0);
    fprintf(stderr, "  Insns interp:    %llu (%.1f%%)\n",
            (unsigned long long)s->insns_interp,
            total > 0 ? 100.0 * (double)s->insns_interp / (double)total : 0.0);
    fprintf(stderr, "  Fallbacks:       %llu\n", (unsigned long long)s->fallbacks);
    fprintf(stderr, "  Cache flushes:   %llu\n", (unsigned long long)s->cache_flushes);
    fprintf(stderr, "  Chains patched:  %llu\n", (unsigned long long)s->chains_patched);
    fprintf(stderr, "  Code cache:      %zu / %zu KB\n",
            jit->code_size / 1024, jit->code_capacity / 1024);
}

#endif /* !_MSC_VER */
