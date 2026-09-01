#ifdef _MSC_VER
/* JIT is x86-64/ARM64 only, disable on MSVC for now */
#else

#include "jit.h"
#if defined(__aarch64__)
#  include "jit_emit_arm64.h"
#  define JIT_ARCH_ARM64 1
#else
#  include "jit_emit_x64.h"
#  define JIT_ARCH_X64 1
#endif
#include "memory.h"
#include "rom_stubs.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <sys/mman.h>
#if defined(__APPLE__) && defined(__aarch64__)
#  include <pthread.h>
#  include <libkern/OSCacheControl.h>
#  define JIT_NEEDS_WX 1
#endif

/* ===== W^X + icache management (Apple Silicon) =====
 * macOS on ARM64 enforces W^X on JIT pages: write mode and execute mode
 * are toggled per-thread via pthread_jit_write_protect_np(), and the
 * icache must be invalidated after any code mutation (block emission,
 * chain patching). On x86 these are no-ops. */
static inline void jit_wx_write_begin(void) {
#if defined(JIT_NEEDS_WX)
    pthread_jit_write_protect_np(0);
#endif
}
static inline void jit_wx_write_end(void *start, size_t len) {
#if defined(JIT_NEEDS_WX)
    pthread_jit_write_protect_np(1);
    sys_icache_invalidate(start, len);
#else
    (void)start; (void)len;
#endif
}

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
#define CPU_OFF_LOOP_EXIT   offsetof(xtensa_cpu_t, jit_loop_exit)
#define CPU_OFF_IRQ_CHECK   offsetof(xtensa_cpu_t, irq_check)
#define CPU_OFF_CYCLE_COUNT offsetof(xtensa_cpu_t, cycle_count)
#define CPU_OFF_MEM         offsetof(xtensa_cpu_t, mem)
#define CPU_OFF_PC_HOOK     offsetof(xtensa_cpu_t, pc_hook)
#define CPU_OFF_PC_HOOK_CTX offsetof(xtensa_cpu_t, pc_hook_ctx)
#define CPU_OFF_HOOK_BITMAP offsetof(xtensa_cpu_t, pc_hook_bitmap)
#define CPU_OFF_PREDECODE   offsetof(xtensa_cpu_t, predecode)
#define CPU_OFF_WINDOWSTART offsetof(xtensa_cpu_t, windowstart)
#define CPU_OFF_WINDOW_CALLSIZE offsetof(xtensa_cpu_t, window_callsize)
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

    /* Differential verification mode: every native block is re-run through
     * the interpreter and the two results compared. verify_active guards
     * against the re-run dispatching back into the JIT. */
    bool verify;
    bool verify_active;
    uint64_t verify_blocks;
    uint64_t verify_mismatches;
    uint64_t verify_skipped;

    /* Hook chaining: JIT installs itself as a pc_hook, forwarding
     * non-JIT addresses to the original (ROM stubs) hook. */
    xtensa_pc_hook_fn original_hook;
    void             *original_hook_ctx;

    /* JIT-specific hook bitmap: set bits for compiled block PCs.
     * Merged with the ROM stub bitmap for the interpreter's fast path. */
    uint64_t jit_bitmap[HOOK_BITMAP_WORDS];

    /* Block chaining: pending jmp sites keyed by target (pc, wb) in a hash
     * table — O(1) record and O(1) resolve, replacing the old linear scan
     * of a 131K-entry slot array that made compilation quadratic. */
    uint8_t      *epilogue_stub;       /* shared pop/ret stub in code cache */
    uint8_t      *last_chain_entry;    /* chain entry of last compiled block */
    chain_pending_t pend[JIT_HASH_SIZE];

    /* Static branch targets recorded during the current compile (for the
     * descent loop in jit_compile_now). Reset per compile; single-threaded. */
    uint32_t      (*dt)[2];
    int            dt_count;

    /* Hook bitmap management: at install we swap cpu->pc_hook_bitmap for a
     * JIT-owned merged copy (ROM stub bits | JIT block bits). orig_bitmap
     * keeps the pristine ROM-only bitmap so jit_scan_block can stop at real
     * stub hooks without being confused by JIT bits of already-compiled PCs. */
    const uint64_t *orig_bitmap;
    uint64_t       *merged_bitmap;

    /* Set when the code cache fills: no new blocks are compiled, but all
     * previously compiled blocks and chains keep executing. */
    bool          compile_disabled;

    /* Flash-MMU writes can occur inside a native block's memory helper. Do
     * not overwrite the code cache until that block has returned to C. */
    unsigned      execution_depth;
    bool          invalidate_pending;

    /* FLEXE_JIT_NOCHAIN debug: skip chain recording (all exits via epilogue) */
    int           no_chain;
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

/* NOTE: jit_scan_block() bounds a block at LEND using the *runtime* loop
 * registers, while blocks are keyed only by (pc, windowbase). A block first
 * compiled at LBEG outside a loop is therefore scanned past LEND, and reusing
 * it once the loop is live would run the body out of the loop. Keying the
 * cache on the compile-time LEND as well was tried and reverted: a PC entered
 * both inside and outside a loop then recompiles on every alternation, and
 * the resulting thrash starved guest_call8()'s step budget badly enough to
 * break the stock-ROM BLE path. In practice jit_run() only samples LBEG while
 * lcount > 0, so the bound is right on the path that matters. */
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
    /* Block compilation is cold relative to execution; use the canonical
     * fetch so page-boundary and unmapped-page behavior cannot diverge. */
    return xtensa_fetch(cpu, addr, insn_out);
}

/* ===== Block scanning: determine block boundaries ===== */

typedef struct {
    uint32_t insns[JIT_MAX_BLOCK_INSNS];
    uint32_t pcs[JIT_MAX_BLOCK_INSNS];
    int      ilens[JIT_MAX_BLOCK_INSNS];
    int      count;
    uint32_t end_pc;   /* PC after last instruction */
    bool     ends_at_lend; /* Truncated at LEND: fall-through is a loop back-edge */
} jit_scan_t;

/* Check if a PC is a ROM stub hook address (NOT a JIT block bit) */
static int is_hook_addr(jit_state_t *jit, xtensa_cpu_t *cpu, uint32_t pc) {
    const uint64_t *bm = jit->orig_bitmap ? jit->orig_bitmap
                                          : (const uint64_t *)cpu->pc_hook_bitmap;
    if (!bm) return 0;
    uint32_t idx = (pc >> 2) & (HOOK_BITMAP_BITS - 1);
    return (bm[idx / 64] >> (idx & 63)) & 1;
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
            return 3;  /* BEQZ.N / BNEZ.N — conditional, side-exit */
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
        if (nn == 1) return 3;  /* BZ (BEQZ/BNEZ/BLTZ/BGEZ) — conditional */
        if (nn == 2) return 3;  /* BI0 (BEQI/BNEI/BLTI/BGEI) — conditional */
        /* nn == 3: BI1 */
        int m = (insn >> 6) & 3;
        if (m == 0) return 1;  /* ENTRY — block terminator (compiled) */
        if (m == 1) {
            int r = (insn >> 12) & 0xF;
            if (r == 0 || r == 1) return 3;  /* BF/BT — conditional */
            if (r >= 8 && r <= 10) return 2; /* LOOP/LOOPNEZ/LOOPGTZ — fallback */
            return 2;
        }
        return 3;  /* BLTUI/BGEUI — conditional */
    }
    case 7: return 3;  /* B — conditional branches get side-exits */
    default: return 2;  /* FP loads, MAC16, etc */
    }
}

/* Scan a basic block starting at pc */
static void jit_scan_block(jit_state_t *jit, xtensa_cpu_t *cpu, uint32_t pc,
                           jit_scan_t *scan) {
    scan->count = 0;
    scan->ends_at_lend = false;
    uint32_t cur_pc = pc;
    uint32_t page_end = (pc & ~0xFFFu) + 0x1000;

    /* If a zero-overhead loop is active, stop the block at lend so the
     * loop-back check in jit_run fires between blocks. */
    uint32_t lend = (cpu->lcount > 0) ? cpu->lend : 0;

    for (int i = 0; i < JIT_MAX_BLOCK_INSNS; i++) {
        /* Stop at page boundary */
        if (cur_pc >= page_end) break;

        /* Stop at hook addresses (stubs) */
        if (is_hook_addr(jit, cpu, cur_pc)) break;

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
        if (lend && cur_pc >= lend) { scan->ends_at_lend = true; break; }

        if (cls == 1) {
            /* Block terminator (call/ret/jmp) — include it, then stop */
            break;
        }
        if (cls == 2) {
            /* Fallback instruction — don't include it, back up */
            scan->count = i;
            cur_pc -= (uint32_t)ilen;
            break;
        }
        /* cls 0 (straight-line) and cls 3 (conditional branch with
         * side-exit) both continue the trace. */
    }
    scan->end_pc = cur_pc;
}

/* Decode the fixed target of a control-flow instruction.  Dynamic returns
 * and calls are intentionally excluded: this is used only to recognize
 * short, native-chainable loops whose dispatch savings outweigh the normal
 * four-instruction JIT profitability floor. */
static int jit_static_branch_target(uint32_t insn, int ilen, uint32_t pc,
                                    uint32_t *target_out) {
    if (ilen != 3)
        return 0;  /* narrow conditional branches are forward-only */

    uint32_t next_pc = pc + 3;
    int op0 = XT_OP0(insn);
    if (op0 == 6) {
        int nn = XT_N(insn);
        int m = XT_M(insn);
        if (nn == 0) {
            *target_out = next_pc + (uint32_t)sign_extend(XT_OFFSET18(insn), 18) + 1;
            return 1;
        }
        if (nn == 1) {
            *target_out = next_pc + (uint32_t)sign_extend(XT_IMM12(insn), 12) + 1;
            return 1;
        }
        if (nn == 2) {
            *target_out = next_pc + (uint32_t)sign_extend(XT_IMM8(insn), 8) + 1;
            return 1;
        }
        if (nn == 3) {
            int r = XT_R(insn);
            if ((m == 1 && (r == 0 || r == 1)) || m == 2 || m == 3) {
                *target_out = next_pc + (uint32_t)sign_extend(XT_IMM8(insn), 8) + 1;
                return 1;
            }
        }
        return 0;
    }
    if (op0 == 7) {
        *target_out = next_pc + (uint32_t)sign_extend(XT_IMM8(insn), 8) + 1;
        return 1;
    }
    return 0;
}

static int jit_short_block_has_backedge(const jit_scan_t *scan,
                                        uint32_t block_pc) {
    for (int i = 0; i < scan->count; i++) {
        uint32_t target;
        if (jit_static_branch_target(scan->insns[i], scan->ilens[i],
                                     scan->pcs[i], &target) &&
            target <= block_pc)
            return 1;
    }
    return 0;
}


/* ===== Code generation ===== */

/* CPU register R15, MEM pointer R14 */
#define REG_CPU  R15
#define REG_MEM  R14

/* Guest-insn accumulator. Every block exit ADDS its count into the
 * accumulator; the epilogue returns the total to jit_pc_hook. It must
 * survive the whole block body (all instruction emitters clobber RAX..RDI
 * plus the guest-mapped R8..R13).
 *
 * ARM64: X27 — callee-saved, outside the RAX..R15 compat enum, saved in
 *        the block prologue/epilogue. Body code never touches it.
 * x86:   no free register exists (R8-R13 hold guest a1-a6 via RA_MAP,
 *        R14/R15 hold mem/cpu, and RAX..RDI plus RBP are body scratch), so
 *        the total lives in cpu->jit_acc. x86 adds to and compares against
 *        memory directly, so a block exit still costs one instruction, as
 *        in the ARM64 register form; the epilogue loads it into EAX. */
#ifdef JIT_ARCH_ARM64
#define REG_ACC  27  /* X27 */
static inline void emit_acc_zero(emit_t *e) { emit_mov_reg_imm32(e, REG_ACC, 0); }
static inline void emit_acc_add(emit_t *e, int n) {
    if (n) emit_add_reg32_imm32(e, REG_ACC, n);
}
/* cap check: returns jcc site that CONTINUES the block when under cap */
static inline int emit_acc_cap_jcc(emit_t *e) {
    emit_cmp_reg32_imm32(e, REG_ACC, JIT_CHAIN_CAP);
    return emit_jcc_rel32(e, CC_B);
}
#else
#define CPU_OFF_JIT_ACC  offsetof(xtensa_cpu_t, jit_acc)
static inline void emit_acc_zero(emit_t *e) {
    emit_store32_disp_imm(e, REG_CPU, (int32_t)CPU_OFF_JIT_ACC, 0);
}
static inline void emit_acc_add(emit_t *e, int n) {
    if (n) emit_add32_disp_imm(e, REG_CPU, (int32_t)CPU_OFF_JIT_ACC, (uint32_t)n);
}
static inline int emit_acc_cap_jcc(emit_t *e) {
    emit_cmp32_disp_imm(e, REG_CPU, (int32_t)CPU_OFF_JIT_ACC, JIT_CHAIN_CAP);
    return emit_jcc_rel32(e, CC_B);
}
#endif

/* Bit-test branch conditions. x86 BT sets CF=bit, so "bit clear" branches
 * use CC_AE (CF=0) and "bit set" uses CC_B. The ARM64 lowering uses TST,
 * which sets Z=1 when the masked bit is clear. */
#ifdef JIT_ARCH_ARM64
#define JIT_CC_BIT_CLEAR  CC_E
#define JIT_CC_BIT_SET    CC_NE
#else
#define JIT_CC_BIT_CLEAR  CC_AE
#define JIT_CC_BIT_SET    CC_B
#endif

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

/* (ra_preload removed — regs are loaded lazily on first use) */

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
/* Shared per-arch fragments for the mem fast paths:
 *  - emit_pt_load:       dst64 = mem->page_table[idx32]  (idx32 preserved)
 *  - emit_pt_null_jz:    branch-to-slow-path site if dst64 == NULL
 *  - emit_guest_ld/st:   access [page_ptr(RAX) + offset(RDX)] at guest width
 *  - emit_call_slow_*:   call the C slow-path helper with ABI-correct args
 * On ARM64 the allocated guest regs live in callee-saved X19-X24, so no
 * register saving is needed around the C call (SP stays 16-aligned). */
#ifdef JIT_ARCH_ARM64
static void emit_pt_load(emit_t *e, int idx32) {
    emit_load64_index(e, RAX, REG_MEM, idx32, MEM_OFF_PAGE_TABLE);
}
static int emit_pt_null_jz(emit_t *e) {
    emit_test_reg64(e, RAX, RAX);
    return emit_jcc_rel32(e, CC_E);
}
static void emit_call_slow2(emit_t *e, void *fn, int addr_reg) {
    if (addr_reg != RCX) emit_mov_reg32_reg32(e, RCX, addr_reg); /* W1 = addr */
    emit_mov_reg_reg(e, RAX, REG_MEM);                          /* X0 = mem  */
    emit_mov_reg_imm64(e, ARM64_SCRATCH, (uint64_t)(uintptr_t)fn);
    emit_call_reg(e, ARM64_SCRATCH);
}
static void emit_call_slow3(emit_t *e, void *fn, int addr_reg, int val_reg) {
    emit_mov_reg32_reg32(e, ARM64_SCRATCH, val_reg);            /* W9 = val  */
    if (addr_reg != RCX) emit_mov_reg32_reg32(e, RCX, addr_reg); /* W1 = addr */
    emit_mov_reg32_reg32(e, RDX, ARM64_SCRATCH);                /* W2 = val  */
    emit_mov_reg_reg(e, RAX, REG_MEM);                          /* X0 = mem  */
    emit_mov_reg_imm64(e, ARM64_SCRATCH, (uint64_t)(uintptr_t)fn);
    emit_call_reg(e, ARM64_SCRATCH);
}
#else
static void emit_pt_load(emit_t *e, int idx32) {
    /* rax = [r14 + idx*8 + MEM_OFF_PAGE_TABLE] */
    emit8(e, rex(1, 0, 0, (R14 >> 3) & 1));  /* REX.W + B for r14 */
    emit8(e, 0x8B);  /* MOV r64, [...]  */
    emit8(e, modrm(2, RAX, 4));  /* mod=10, reg=rax, rm=SIB */
    emit8(e, sib(3, idx32, R14 & 7));  /* scale=8, index=idx, base=r14 */
    emit32(e, (uint32_t)MEM_OFF_PAGE_TABLE);
}
static int emit_pt_null_jz(emit_t *e) {
    /* test rax, rax */
    emit8(e, rex(1, 0, 0, 0));
    emit8(e, 0x85);
    emit8(e, modrm(3, RAX, RAX));
    return emit_jcc_rel32(e, CC_E);
}
static void emit_call_slow2(emit_t *e, void *fn, int addr_reg) {
    /* Save caller-saved allocated regs around C call */
    emit_push(e, R8); emit_push(e, R9); emit_push(e, R10); emit_push(e, R11);
    emit_mov_reg_reg(e, RDI, REG_MEM);
    emit_mov_reg32_reg32(e, RSI, addr_reg);
    emit_mov_reg_imm64(e, RAX, (uint64_t)(uintptr_t)fn);
    emit_call_reg(e, RAX);
    emit_pop(e, R11); emit_pop(e, R10); emit_pop(e, R9); emit_pop(e, R8);
}
static void emit_call_slow3(emit_t *e, void *fn, int addr_reg, int val_reg) {
    emit_push(e, R8); emit_push(e, R9); emit_push(e, R10); emit_push(e, R11);
    emit_mov_reg_reg(e, RDI, REG_MEM);
    emit_mov_reg32_reg32(e, RSI, addr_reg);
    emit_mov_reg32_reg32(e, RDX, val_reg);
    emit_mov_reg_imm64(e, RAX, (uint64_t)(uintptr_t)fn);
    emit_call_reg(e, RAX);
    emit_pop(e, R11); emit_pop(e, R10); emit_pop(e, R9); emit_pop(e, R8);
}
#endif

static void emit_mem_read32(emit_t *e, int addr_reg, int dst_reg) {
    /* ecx = addr >> 12 */
    emit_mov_reg32_reg32(e, RCX, addr_reg);
    emit_shr_reg32_imm(e, RCX, 12);

    /* rax = mem->page_table[ecx] */
    emit_pt_load(e, RCX);

    int slow_patch = emit_pt_null_jz(e);

    /* Fast path: edx = addr & 0xFFF */
    emit_mov_reg32_reg32(e, RDX, addr_reg);
    emit_and_reg32_imm32(e, RDX, 0xFFF);

    /* dst = [rax + rdx] (32-bit load) */
#ifdef JIT_ARCH_ARM64
    emit_load32_rof(e, dst_reg, RAX, RDX);
#else
    emit_rex(e, 0, dst_reg, RAX);
    emit8(e, 0x8B);
    emit8(e, modrm(0, dst_reg, 4));  /* SIB follows */
    emit8(e, sib(0, RDX, RAX));      /* scale=1, index=rdx, base=rax */
#endif

    int done_patch = emit_jmp_rel32(e);

    /* Slow path: call mem_read32_slow(mem, addr) */
    emit_patch_rel32(e, slow_patch);
    emit_call_slow2(e, (void *)(uintptr_t)mem_read32_slow, addr_reg);
    /* Result is in W0/eax, move to dst if needed */
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
static void emit_mem_write32(emit_t *e, int addr_reg, int val_reg,
                             jit_state_t *jit) {
    /* Verification needs the journal to see this store, and the inline
     * page-table path bypasses it. Give up the fast path for the
     * duration -- a verification run has already given up speed. */
    if (jit && jit->verify) {
        emit_call_slow3(e, (void *)(uintptr_t)mem_write32_journaled, addr_reg, val_reg);
        return;
    }
    /* ecx = addr >> 12 */
    emit_mov_reg32_reg32(e, RCX, addr_reg);
    emit_shr_reg32_imm(e, RCX, 12);

    /* rax = mem->page_table[ecx] */
    emit_pt_load(e, RCX);

    int slow_patch = emit_pt_null_jz(e);

    /* Fast path: edx = addr & 0xFFF */
    emit_mov_reg32_reg32(e, RDX, addr_reg);
    emit_and_reg32_imm32(e, RDX, 0xFFF);

    /* [rax + rdx] = val_reg (32-bit store) */
#ifdef JIT_ARCH_ARM64
    emit_store32_rof(e, val_reg, RAX, RDX);
#else
    emit_rex(e, 0, val_reg, RAX);
    emit8(e, 0x89);
    emit8(e, modrm(0, val_reg, 4));
    emit8(e, sib(0, RDX, RAX));
#endif

    int done_patch = emit_jmp_rel32(e);

    /* Slow path: call mem_write32_slow(mem, addr, val) */
    emit_patch_rel32(e, slow_patch);
    emit_call_slow3(e, (void *)(uintptr_t)mem_write32_slow, addr_reg, val_reg);

    emit_patch_rel32(e, done_patch);
}

/* Emit memory read8u inlined fast path */
static void emit_mem_read8u(emit_t *e, int addr_reg, int dst_reg) {
    emit_mov_reg32_reg32(e, RCX, addr_reg);
    emit_shr_reg32_imm(e, RCX, 12);
    emit_pt_load(e, RCX);
    int slow_patch = emit_pt_null_jz(e);
    emit_mov_reg32_reg32(e, RDX, addr_reg);
    emit_and_reg32_imm32(e, RDX, 0xFFF);
    /* movzx dst32, byte [rax + rdx] */
#ifdef JIT_ARCH_ARM64
    emit_load8u_rof(e, dst_reg, RAX, RDX);
#else
    emit_rex(e, 0, dst_reg, RAX);
    emit8(e, 0x0F); emit8(e, 0xB6);
    emit8(e, modrm(0, dst_reg, 4));
    emit8(e, sib(0, RDX, RAX));
#endif
    int done_patch = emit_jmp_rel32(e);
    emit_patch_rel32(e, slow_patch);
    emit_call_slow2(e, (void *)(uintptr_t)mem_read8_slow, addr_reg);
    if (dst_reg != RAX) emit_mov_reg32_reg32(e, dst_reg, RAX);
    emit_patch_rel32(e, done_patch);
}

/* Emit memory read16u inlined fast path */
static void emit_mem_read16u(emit_t *e, int addr_reg, int dst_reg) {
    emit_mov_reg32_reg32(e, RCX, addr_reg);
    emit_shr_reg32_imm(e, RCX, 12);
    emit_pt_load(e, RCX);
    int slow_patch = emit_pt_null_jz(e);
    emit_mov_reg32_reg32(e, RDX, addr_reg);
    emit_and_reg32_imm32(e, RDX, 0xFFF);
    /* movzx dst32, word [rax + rdx] */
#ifdef JIT_ARCH_ARM64
    emit_load16u_rof(e, dst_reg, RAX, RDX);
#else
    emit_rex(e, 0, dst_reg, RAX);
    emit8(e, 0x0F); emit8(e, 0xB7);
    emit8(e, modrm(0, dst_reg, 4));
    emit8(e, sib(0, RDX, RAX));
#endif
    int done_patch = emit_jmp_rel32(e);
    emit_patch_rel32(e, slow_patch);
    emit_call_slow2(e, (void *)(uintptr_t)mem_read16_slow, addr_reg);
    if (dst_reg != RAX) emit_mov_reg32_reg32(e, dst_reg, RAX);
    emit_patch_rel32(e, done_patch);
}

/* Emit memory read16s (signed) */
static void emit_mem_read16s(emit_t *e, int addr_reg, int dst_reg) {
    emit_mov_reg32_reg32(e, RCX, addr_reg);
    emit_shr_reg32_imm(e, RCX, 12);
    emit_pt_load(e, RCX);
    int slow_patch = emit_pt_null_jz(e);
    emit_mov_reg32_reg32(e, RDX, addr_reg);
    emit_and_reg32_imm32(e, RDX, 0xFFF);
    /* movsx dst32, word [rax + rdx] */
#ifdef JIT_ARCH_ARM64
    emit_load16s_rof(e, dst_reg, RAX, RDX);
#else
    emit_rex(e, 0, dst_reg, RAX);
    emit8(e, 0x0F); emit8(e, 0xBF);
    emit8(e, modrm(0, dst_reg, 4));
    emit8(e, sib(0, RDX, RAX));
#endif
    int done_patch = emit_jmp_rel32(e);
    emit_patch_rel32(e, slow_patch);
    emit_call_slow2(e, (void *)(uintptr_t)mem_read16_slow, addr_reg);
    /* Sign extend from 16 bits */
    emit_movsx_reg32_reg16(e, dst_reg != RAX ? dst_reg : RAX, RAX);
    emit_patch_rel32(e, done_patch);
}

/* Emit memory write8 */
static void emit_mem_write8(emit_t *e, int addr_reg, int val_reg,
                             jit_state_t *jit) {
    /* Verification needs the journal to see this store, and the inline
     * page-table path bypasses it. Give up the fast path for the
     * duration -- a verification run has already given up speed. */
    if (jit && jit->verify) {
        emit_call_slow3(e, (void *)(uintptr_t)mem_write8_journaled, addr_reg, val_reg);
        return;
    }
    emit_mov_reg32_reg32(e, RCX, addr_reg);
    emit_shr_reg32_imm(e, RCX, 12);
    emit_pt_load(e, RCX);
    int slow_patch = emit_pt_null_jz(e);
    emit_mov_reg32_reg32(e, RDX, addr_reg);
    emit_and_reg32_imm32(e, RDX, 0xFFF);
    /* mov byte [rax + rdx], val_reg_low8 */
#ifdef JIT_ARCH_ARM64
    emit_store8_rof(e, val_reg, RAX, RDX);
#else
    /* val_reg is RBP here, so the REX prefix is mandatory: without it the
     * reg field names CH rather than BPL, and CH currently holds bits 8-15
     * of the page index -- the store lands, with the wrong byte. */
    emit_rex8(e, val_reg, RAX);
    emit8(e, 0x88);
    emit8(e, modrm(0, val_reg, 4));
    emit8(e, sib(0, RDX, RAX));
#endif
    int done_patch = emit_jmp_rel32(e);
    emit_patch_rel32(e, slow_patch);
    emit_call_slow3(e, (void *)(uintptr_t)mem_write8_slow, addr_reg, val_reg);
    emit_patch_rel32(e, done_patch);
}

/* Emit memory write16 */
static void emit_mem_write16(emit_t *e, int addr_reg, int val_reg,
                             jit_state_t *jit) {
    /* Verification needs the journal to see this store, and the inline
     * page-table path bypasses it. Give up the fast path for the
     * duration -- a verification run has already given up speed. */
    if (jit && jit->verify) {
        emit_call_slow3(e, (void *)(uintptr_t)mem_write16_journaled, addr_reg, val_reg);
        return;
    }
    emit_mov_reg32_reg32(e, RCX, addr_reg);
    emit_shr_reg32_imm(e, RCX, 12);
    emit_pt_load(e, RCX);
    int slow_patch = emit_pt_null_jz(e);
    emit_mov_reg32_reg32(e, RDX, addr_reg);
    emit_and_reg32_imm32(e, RDX, 0xFFF);
    /* mov word [rax + rdx], val_reg_low16 */
#ifdef JIT_ARCH_ARM64
    emit_store16_rof(e, val_reg, RAX, RDX);
#else
    emit8(e, 0x66);
    emit_rex(e, 0, val_reg, RAX);
    emit8(e, 0x89);
    emit8(e, modrm(0, val_reg, 4));
    emit8(e, sib(0, RDX, RAX));
#endif
    int done_patch = emit_jmp_rel32(e);
    emit_patch_rel32(e, slow_patch);
    emit_call_slow3(e, (void *)(uintptr_t)mem_write16_slow, addr_reg, val_reg);
    emit_patch_rel32(e, done_patch);
}


/* ===== Per-instruction compilation ===== */

/* Forward declarations for helpers used in instruction compilation */
static void emit_block_exit_ra(emit_t *e, regalloc_t *ra, int wb4,
                               uint32_t exit_pc, int insn_count,
                               jit_state_t *jit, bool loop_end_exit);
static void emit_jmp_to_epilogue(emit_t *e, jit_state_t *jit);
static void jit_chain_record(jit_state_t *jit, uint32_t target_pc,
                             uint32_t target_wb, uint8_t *jmp_site);

/* Side exit: a conditional branch in the middle of a trace. The taken
 * path jumps to a stub emitted after the block body; the fall-through
 * path keeps compiling. ar[] is flushed inline BEFORE the jcc so the
 * stub needs no flush (see jit_add_side_exit). */
typedef struct {
    int      patch_site;  /* emit-buffer offset of the jcc B instruction */
    uint32_t target_pc;   /* guest PC the taken path exits to */
    int      insn_count;  /* guest insns executed including the branch */
    uint32_t target_wb;   /* windowbase for the chain slot */
} side_exit_t;

/* Record a conditional branch's taken path as a deferred side exit.
 * Emits: flush dirty regs (ar[] correct at branch point), then the
 * conditional jump to the not-yet-emitted stub. Fall-through continues. */
static void jit_add_side_exit(emit_t *e, regalloc_t *ra, int wb4, int cc,
                              uint32_t target_pc, int insn_count,
                              side_exit_t *sx, int *sx_count, jit_state_t *jit) {
    (void)jit;
    ra_flush(e, ra, wb4);
    int patch = emit_jcc_rel32(e, (uint8_t)cc);
    sx[*sx_count].patch_site = patch;
    sx[*sx_count].target_pc  = target_pc;
    sx[*sx_count].insn_count = insn_count;
    sx[*sx_count].target_wb  = (uint32_t)(wb4 / 4);
    (*sx_count)++;
}

/* Compile a single instruction. Returns 1 on success, 0 if we should abort the block.
 * Conditional branches record deferred side exits in sx[] (fall-through
 * keeps compiling); terminators emit a full exit and end the block. */
static int jit_compile_insn(emit_t *e, xtensa_cpu_t *cpu, int wb4, uint32_t insn,
                            int ilen, uint32_t pc, uint32_t next_pc, int insn_idx,
                            regalloc_t *ra, jit_state_t *jit,
                            side_exit_t *sx, int *sx_count) {
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
            emit_mem_write32(e, RSI, RBP, jit);
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
            /* BEQZ.N / BNEZ.N — conditional, side-exit */
            int imm6 = ((t & 3) << 4) | r;
            uint32_t target = next_pc + (uint32_t)imm6 + 2; /* +2 per ISA */
            ra_load_ar(e, ra,RAX, wb4, s);
            emit_test_reg32(e, RAX, RAX);
            /* BEQZ.N: taken if as == 0; BNEZ.N: taken if as != 0 */
            jit_add_side_exit(e, ra, wb4, t_hi == 2 ? CC_E : CC_NE,
                              target, insn_idx + 1, sx, sx_count, jit);
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
                    emit_block_exit_ra(e, ra, wb4, 0 /* will use cpu->pc */, insn_idx + 1, jit, false);
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
                    emit_block_exit_ra(e, ra, wb4, 0, insn_idx + 1, jit, false);
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
                    emit_block_exit_ra(e, ra, wb4, 0, insn_idx + 1, jit, false);
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
                /* Build the 64-bit concatenation (as:at) in RAX. Both halves
                 * are loaded with 32-bit moves, which zero-extend, so a
                 * single 64-bit OR is enough -- and is the *only* thing that
                 * works here. A 32-bit `or eax, ebx` would zero RAX's upper
                 * half and silently turn SRC into a plain `at >> SAR`. */
                ra_load_ar(e, ra,RAX, wb4, s);
                emit_shl_reg64_imm(e, RAX, 32);  /* high 32 */
                ra_load_ar(e, ra,RBX, wb4, t);
#ifdef JIT_ARCH_ARM64
                emit_or_reg64(e, RAX, RBX);
#else
                emit8(e, rex(1, 0, 0, 0)); emit8(e, 0x09); emit8(e, modrm(3, RBX, RAX));
#endif
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
#ifdef JIT_ARCH_ARM64
                emit_cset_reg32(e, CC_L, RAX);  /* rax = (as < at) ? 1 : 0 */
#else
                /* setl al; movzx eax, al */
                emit8(e, 0x0F); emit8(e, 0x9C); emit8(e, modrm(3, 0, RAX)); /* setl al */
                emit8(e, 0x0F); emit8(e, 0xB6); emit8(e, modrm(3, RAX, RAX)); /* movzx eax, al */
#endif
                ra_store_ar(e, ra,RAX, wb4, r);
                return 1;
            }
            case 7: { /* SALTU: ar = as < at ? 1 : 0 (unsigned) */
                ra_load_ar(e, ra,RAX, wb4, s);
                ra_load_ar(e, ra,RBX, wb4, t);
                emit_cmp_reg32(e, RAX, RBX);
#ifdef JIT_ARCH_ARM64
                emit_cset_reg32(e, CC_B, RAX);  /* rax = (as <u at) ? 1 : 0 */
#else
                emit8(e, 0x0F); emit8(e, 0x92); emit8(e, modrm(3, 0, RAX)); /* setb al */
                emit8(e, 0x0F); emit8(e, 0xB6); emit8(e, modrm(3, RAX, RAX));
#endif
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
#ifdef JIT_ARCH_ARM64
                /* 32-bit loads already zero-extended; UMULL + LSR #32 */
                emit_umulh32(e, RAX, RAX, RBX);
#else
                /* mov eax, eax already zero-extends in 64-bit mode */
                emit8(e, rex(1, RAX, 0, RBX)); emit8(e, 0x0F); emit8(e, 0xAF);
                emit8(e, modrm(3, RAX, RBX)); /* imul rax, rbx */
                emit_shr_reg64_imm(e, RAX, 32);
#endif
                ra_store_ar(e, ra,RAX, wb4, r);
                return 1;
            }
            case 11: { /* MULSH: ar = (int64)(int32)as * (int64)(int32)at >> 32 */
                ra_load_ar(e, ra,RAX, wb4, s);
#ifdef JIT_ARCH_ARM64
                ra_load_ar(e, ra,RBX, wb4, t);
                /* SMULL sign-extends W inputs itself */
                emit_smulh32(e, RAX, RAX, RBX);
#else
                /* movsxd rax, eax */
                emit8(e, 0x48); emit8(e, 0x63); emit8(e, modrm(3, RAX, RAX));
                ra_load_ar(e, ra,RBX, wb4, t);
                emit8(e, 0x48); emit8(e, 0x63); emit8(e, modrm(3, RBX, RBX));
                emit8(e, rex(1, RAX, 0, RBX)); emit8(e, 0x0F); emit8(e, 0xAF);
                emit8(e, modrm(3, RAX, RBX));
                emit_shr_reg64_imm(e, RAX, 32);
#endif
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
                /* Always give the register allocator one deterministic
                 * destination value.  Branching around ra_store_ar() leaves
                 * its compile-time dirty mapping live on the untaken path. */
                ra_load_ar(e, ra,RAX, wb4, r);
                ra_load_ar(e, ra,RBX, wb4, s);
                ra_load_ar(e, ra,RCX, wb4, t);
                emit_test_reg32(e, RCX, RCX);
                emit_cmov_reg32(e, CC_E, RAX, RBX);
                ra_store_ar(e, ra,RAX, wb4, r);
                return 1;
            }
            case 9: { /* MOVNEZ: if (at != 0) ar = as */
                ra_load_ar(e, ra,RAX, wb4, r);
                ra_load_ar(e, ra,RBX, wb4, s);
                ra_load_ar(e, ra,RCX, wb4, t);
                emit_test_reg32(e, RCX, RCX);
                emit_cmov_reg32(e, CC_NE, RAX, RBX);
                ra_store_ar(e, ra,RAX, wb4, r);
                return 1;
            }
            case 10: { /* MOVLTZ: if ((int32)at < 0) ar = as */
                ra_load_ar(e, ra,RAX, wb4, r);
                ra_load_ar(e, ra,RBX, wb4, s);
                ra_load_ar(e, ra,RCX, wb4, t);
                emit_test_reg32(e, RCX, RCX);
                emit_cmov_reg32(e, CC_S, RAX, RBX);
                ra_store_ar(e, ra,RAX, wb4, r);
                return 1;
            }
            case 11: { /* MOVGEZ: if ((int32)at >= 0) ar = as */
                ra_load_ar(e, ra,RAX, wb4, r);
                ra_load_ar(e, ra,RBX, wb4, s);
                ra_load_ar(e, ra,RCX, wb4, t);
                emit_test_reg32(e, RCX, RCX);
                emit_cmov_reg32(e, CC_NS, RAX, RBX);
                ra_store_ar(e, ra,RAX, wb4, r);
                return 1;
            }
            case 12: { /* MOVF: if (!bt) ar = as */
                emit_load_cpu32(e, RBX, (int32_t)CPU_OFF_BR);
                emit_test_reg32_imm32(e, RBX, (uint32_t)(1 << t)); /* Z=1 iff bit clear */
                int skip_patch = emit_jcc_rel32(e, CC_NE);
                ra_load_ar(e, ra,RAX, wb4, s);
                ra_store_ar(e, ra,RAX, wb4, r);
                emit_patch_rel32(e, skip_patch);
                return 1;
            }
            case 13: { /* MOVT: if (bt) ar = as */
                emit_load_cpu32(e, RBX, (int32_t)CPU_OFF_BR);
                emit_test_reg32_imm32(e, RBX, (uint32_t)(1 << t));
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
        /* Constant-fold literal pools in ROM or the instruction-flash buses.
         * Flash-MMU and SPI writes flush translated code before these bytes
         * can change. Internal IRAM and DROM stay as runtime loads. */
        if ((target >= ESP32_INSN_ADDR_LOW && target < 0x40060000u) ||
            (target >= ESP32_FLASH_INSN_ADDR_LOW &&
             target < ESP32_INSN_ADDR_HIGH)) {
            uint32_t val = mem_read32(cpu->mem, target);
            emit_mov_reg_imm32(e, RBX, val);
            ra_store_ar(e, ra,RBX, wb4, t);
            return 1;
        }
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
            emit_mem_write8(e, RSI, RBP, jit);
            return 1;
        }
        case 0x5: { /* S16I */
            ra_load_ar(e, ra,RSI, wb4, s);
            emit_add_reg32_imm32(e, RSI, imm8 << 1);
            ra_load_ar(e, ra,RBP, wb4, t);
            emit_mem_write16(e, RSI, RBP, jit);
            return 1;
        }
        case 0x6: { /* S32I */
            ra_load_ar(e, ra,RSI, wb4, s);
            emit_add_reg32_imm32(e, RSI, imm8 << 2);
            ra_load_ar(e, ra,RBP, wb4, t);
            emit_mem_write32(e, RSI, RBP, jit);
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
            emit_add_reg32_imm32(e, RAX, simm8 * 256);
            ra_store_ar(e, ra,RAX, wb4, t);
            return 1;
        }
        case 0xF: { /* S32RI (release = no-op, same as S32I) */
            ra_load_ar(e, ra,RSI, wb4, s);
            emit_add_reg32_imm32(e, RSI, imm8 << 2);
            ra_load_ar(e, ra,RBP, wb4, t);
            emit_mem_write32(e, RSI, RBP, jit);
            return 1;
        }
        default: return 0;
        }
    } /* end LSAI */

    case 5: { /* CALLN: CALL0/CALL4/CALL8/CALL12 */
        int call_nn = XT_N(insn);
        int32_t call_off = sign_extend(XT_OFFSET18(insn), 18);
        uint32_t call_target = (((pc >> 2) + (uint32_t)call_off + 1) << 2);
        uint32_t ret_addr = call_nn == 0
                          ? next_pc
                          : ((uint32_t)call_nn << 30) |
                            (next_pc & 0x3FFFFFFFu);
        int32_t ret_ar_off = (int32_t)(CPU_OFF_AR + (((uint32_t)(wb4 + call_nn*4)) & 63) * 4);

        /* Windowed calls select the ENTRY rotation. CALL0 neither rotates nor
         * changes CALLINC, and stores an ordinary full-width return address. */
        if (call_nn != 0) {
            emit_load_cpu32(e, RAX, (int32_t)CPU_OFF_PS);
            emit_and_reg32_imm32(e, RAX, (int32_t)(~(3u << 16)));
            emit_or_reg32_imm32(e, RAX,
                                (int32_t)((uint32_t)call_nn << 16));
            emit_store_cpu32(e, RAX, (int32_t)CPU_OFF_PS);
        }

        /* Write return address to a0/current or the callee's future a0 slot. */
        emit_store32_disp_imm(e, REG_CPU, ret_ar_off, ret_addr);

        /* Block exit to callee. */
        emit_block_exit_ra(e, ra, wb4, call_target, insn_idx + 1, jit, false);
        return 1;
    }

    case 6: { /* SI: J, BZ, BI0, BI1 */
        int nn = (insn >> 4) & 3;
        int m = (insn >> 6) & 3;

        if (nn == 0) {
            /* J: unconditional jump */
            int32_t offset = sign_extend(XT_OFFSET18(insn), 18);
            uint32_t target = next_pc + (uint32_t)offset + 1; /* +1 per ISA */
            emit_block_exit_ra(e, ra, wb4, target, insn_idx + 1, jit, false);
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
            jit_add_side_exit(e, ra, wb4, cc, target, insn_idx + 1,
                              sx, sx_count, jit);
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
            jit_add_side_exit(e, ra, wb4, cc, target, insn_idx + 1,
                              sx, sx_count, jit);
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
                    emit_test_reg32_imm32(e, RBX, (uint32_t)(1 << s));
                    /* BF: taken if bit NOT set (Z=1); BT: taken if set */
                    jit_add_side_exit(e, ra, wb4, lr == 0 ? CC_E : CC_NE,
                                      target, insn_idx + 1, sx, sx_count, jit);
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
                jit_add_side_exit(e, ra, wb4, cc, target, insn_idx + 1,
                                  sx, sx_count, jit);
                return 1;
            }
            if (m == 0) {
                /* ENTRY: callee prologue — rotate window, set SP, guard overflow */
                int entry_s = XT_S(insn);
                uint32_t frame_size = (uint32_t)XT_IMM12(insn) << 3;

                /* ENTRY's interpreter path spills every live window that the
                 * new frame could overlap.  The native fast path is safe when
                 * only the current window is live; defer more complex window
                 * rings to that exact spill implementation. */
                emit_load_cpu32(e, RDX, (int32_t)CPU_OFF_WINDOWSTART);
                emit_and_reg32_imm32(e, RDX,
                                     (int32_t)~(1u << ((unsigned)wb4 >> 2)));
                emit_test_reg32(e, RDX, RDX);
                int overflow_fb = emit_jcc_rel32(e, CC_NE);

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

                /* ENTRY records how far this frame rotated. RETW needs this
                 * when a tail-call frame has an a0 whose high bits are zero. */
                emit_store8_index(e, RCX, REG_CPU, RDX,
                                  (int32_t)CPU_OFF_WINDOW_CALLSIZE);

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
                emit_store32_disp_imm(e, REG_CPU, (int32_t)CPU_OFF_PC_WRITTEN, 1);
                emit_acc_add(e, insn_idx + 1);
                /* No chain slot: the target wb depends on runtime CALLINC. */
                emit_jmp_to_epilogue(e, jit);

                /* Overflow fallback: interpreter handles it */
                emit_patch_rel32(e, overflow_fb);
                /* Dirty bits deliberately survive the main-path ra_flush(),
                 * so this emits the same valid writeback without storing
                 * host registers that were never loaded on the fallback. */
                ra_flush(e, ra, wb4);
                emit_store_cpu32_imm(e, (int32_t)CPU_OFF_PC, pc);
                emit_acc_add(e, insn_idx);  /* ENTRY itself didn't run */
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
            emit_not_reg32(e, RCX);
            emit_test_reg32(e, RCX, RBX);
            cc = CC_E; break;
        case 5: /* BBC: !(as & (1 << (at & 31))) */
            emit_mov_reg32_reg32(e, RCX, RBX);
            emit_and_reg32_imm32(e, RCX, 31);
#ifdef JIT_ARCH_ARM64
            /* W9 = 1 << bit; TST as, W9 → Z=1 iff bit clear */
            emit_bt_mask(e, RCX);
            emit_test_reg32(e, RAX, ARM64_SCRATCH);
#else
            /* bt eax, ecx */
            emit8(e, 0x0F); emit8(e, 0xA3); emit8(e, modrm(3, RCX, RAX));
#endif
            cc = JIT_CC_BIT_CLEAR;
            is_bit_test = 1; break;
        case 6: case 7: { /* BBCI */
            int bit = t | ((r & 1) << 4);
#ifdef JIT_ARCH_ARM64
            emit_test_reg32_imm32(e, RAX, 1u << bit);
#else
            /* bt eax, imm8 */
            emit8(e, 0x0F); emit8(e, 0xBA); emit8(e, modrm(3, 4, RAX)); emit8(e, (uint8_t)bit);
#endif
            cc = JIT_CC_BIT_CLEAR;
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
            emit_not_reg32(e, RCX);
            emit_test_reg32(e, RCX, RBX);
            cc = CC_NE; break;
        case 13: /* BBS: (as & (1 << (at & 31))) != 0 */
            emit_mov_reg32_reg32(e, RCX, RBX);
            emit_and_reg32_imm32(e, RCX, 31);
#ifdef JIT_ARCH_ARM64
            emit_bt_mask(e, RCX);
            emit_test_reg32(e, RAX, ARM64_SCRATCH);
#else
            emit8(e, 0x0F); emit8(e, 0xA3); emit8(e, modrm(3, RCX, RAX));
#endif
            cc = JIT_CC_BIT_SET;
            is_bit_test = 1; break;
        case 14: case 15: { /* BBSI */
            int bit = t | ((r & 1) << 4);
#ifdef JIT_ARCH_ARM64
            emit_test_reg32_imm32(e, RAX, 1u << bit);
#else
            emit8(e, 0x0F); emit8(e, 0xBA); emit8(e, modrm(3, 4, RAX)); emit8(e, (uint8_t)bit);
#endif
            cc = JIT_CC_BIT_SET;
            is_bit_test = 1; break;
        }
        default: return 0;
        }
        (void)is_bit_test;

        jit_add_side_exit(e, ra, wb4, cc, target, insn_idx + 1,
                          sx, sx_count, jit);
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

        /* Tail-called frames do not encode their creation callsize in a0.
         * The interpreter resolves those through window_callsize[]; leave
         * this uncommon case to it instead of rotating by zero. */
        emit_test_reg32(e, RCX, RCX);
        int callsize_fb = emit_jcc_rel32(e, CC_E);

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
        int fill_fb = emit_jcc_rel32(e, JIT_CC_BIT_CLEAR);  /* bit clear → need fill */

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

        /* 9. Store return_pc → cpu->pc, exit with insn_count. RETW does
         * not change PS.OWB; that field belongs to exception/window traps. */
        emit_store_cpu32(e, RSI, (int32_t)CPU_OFF_PC);
        emit_store32_disp_imm(e, REG_CPU, (int32_t)CPU_OFF_PC_WRITTEN, 1);
        emit_acc_add(e, insn_idx + 1);
        /* No chain slot — dynamic target */
        emit_jmp_to_epilogue(e, jit);

        /* Fill fallback: interpreter handles it */
        emit_patch_rel32(e, fill_fb);
        emit_patch_rel32(e, callsize_fb);
        /* Dirty bits survive the main-path ra_flush(), allowing this side exit
         * to write back exactly the guest registers touched by the prefix. */
        ra_flush(e, ra, wb4);
        emit_store_cpu32_imm(e, (int32_t)CPU_OFF_PC, pc);
        emit_acc_add(e, insn_idx);  /* RETW itself didn't run */
        emit_jmp_to_epilogue(e, jit);
        return 1;
    }

    return 0; /* unreachable */
}

/* Jump to shared epilogue stub */
static void emit_jmp_to_epilogue(emit_t *e, jit_state_t *jit) {
    emit_jmp_rel32_to(e, jit->epilogue_stub);
}

/* Emit a side-exit stub body (no ra flush — done inline at the branch).
 * Accumulates insn_count into RAX (chained runs accumulate) and sets
 * _pc_written so the interpreter's hook gate re-dispatches compiled
 * code on the very next step. */
static void emit_side_exit_body(emit_t *e, const side_exit_t *sx,
                                jit_state_t *jit) {
    emit_patch_rel32(e, sx->patch_site);
    emit_store_cpu32_imm(e, (int32_t)CPU_OFF_PC, sx->target_pc);
    emit_store32_disp_imm(e, REG_CPU, (int32_t)CPU_OFF_PC_WRITTEN, 1);
    emit_acc_add(e, sx->insn_count);

    jit_chain_record(jit, sx->target_pc, sx->target_wb, e->ptr);
    emit_jmp_to_epilogue(e, jit);
}

/* Emit the block exit sequence WITH register allocation:
 * 1. Flush dirty regs
 * 2. Store exit PC (if known) + mark _pc_written
 * 3. Accumulate return value (chained runs keep a running total in RAX)
 * 4. Record chain slot (if static target) and jmp to epilogue */
/* Close a zero-overhead loop's back-edge in native code.
 *
 * A block covering a whole LOOP body otherwise pays a full dispatcher
 * round-trip per iteration: return to jit_pc_hook, decrement LCOUNT there,
 * re-hash the PC and call back in. Loop bodies compilers emit are typically a
 * handful of instructions, so that dispatch dominates the work. Jump straight
 * back to the block's own chain entry instead, which re-checks the chain cap
 * so timers, preemption and the batch budget stay live.
 *
 * Only correct when the block starts at LBEG: a block covering just the tail
 * of a body must still go through the hook, because its entry is not where
 * the loop resumes.
 *
 * LBEG and LEND are re-read and compared every iteration rather than trusted
 * from compile time. The block cache is keyed on (pc, windowbase) and not on
 * the loop context, and the body may itself contain a WSR to LBEG/LEND, so
 * the compile-time loop is not guaranteed to be the live one. Either check
 * failing simply falls out to the hook, which handles the general case.
 */
/* Compare a 32-bit field of the CPU struct against an immediate. x86 can do
 * that against memory directly; ARM64 has to load it into a register first,
 * so callers must not rely on RAX surviving. */
static inline void emit_cmp_cpu32_imm(emit_t *e, int32_t off, uint32_t imm) {
#ifdef JIT_ARCH_ARM64
    emit_load_cpu32(e, RAX, off);
    emit_cmp_reg32_imm32(e, RAX, imm);
#else
    emit_cmp32_disp_imm(e, REG_CPU, off, imm);
#endif
}

/* Whether a block gets the native back-edge. Kept in one place so the
 * verifier can bound its reference run the same way the block behaves. */
static bool jit_block_self_loops(const xtensa_cpu_t *cpu,
                                 const jit_scan_t *scan, uint32_t pc) {
    return scan->ends_at_lend && pc == cpu->lbeg;
}

static void emit_loop_backedge_exit(emit_t *e, regalloc_t *ra, int wb4,
                                    uint32_t lbeg, uint32_t lend,
                                    int insn_count, jit_state_t *jit,
                                    uint8_t *chain_entry) {
    ra_flush(e, ra, wb4);

    emit_load_cpu32(e, RCX, (int32_t)CPU_OFF_LCOUNT);
    emit_cmp_reg32_imm32(e, RCX, 0);
    int done = emit_jcc_rel32(e, CC_E);
    emit_cmp_cpu32_imm(e, (int32_t)CPU_OFF_LEND, lend);
    int other_lend = emit_jcc_rel32(e, CC_NE);
    emit_cmp_cpu32_imm(e, (int32_t)CPU_OFF_LBEG, lbeg);
    int other_lbeg = emit_jcc_rel32(e, CC_NE);

    emit_add_reg32_imm32(e, RCX, (uint32_t)-1);
    emit_store_cpu32(e, RCX, (int32_t)CPU_OFF_LCOUNT);
    emit_store_cpu32_imm(e, (int32_t)CPU_OFF_PC, lbeg);
    emit_store32_disp_imm(e, REG_CPU, (int32_t)CPU_OFF_PC_WRITTEN, 1);
    emit_acc_add(e, insn_count);
    emit_jmp_rel32_to(e, chain_entry);

    /* Loop finished, or this is not the loop the block was compiled for:
     * exit exactly as a plain loop-end block would, and let jit_pc_hook
     * decide what the arrival at LEND means. */
    emit_patch_rel32(e, done);
    emit_patch_rel32(e, other_lend);
    emit_patch_rel32(e, other_lbeg);
    emit_store_cpu32_imm(e, (int32_t)CPU_OFF_PC, lend);
    emit_store32_disp_imm(e, REG_CPU, (int32_t)CPU_OFF_PC_WRITTEN, 1);
    emit_store32_disp_imm(e, REG_CPU, (int32_t)CPU_OFF_LOOP_EXIT, 1);
    emit_acc_add(e, insn_count);
    emit_jmp_to_epilogue(e, jit);
}

static void emit_block_exit_ra(emit_t *e, regalloc_t *ra, int wb4,
                               uint32_t exit_pc, int insn_count,
                               jit_state_t *jit, bool loop_end_exit) {
    /* Flush dirty regs to memory */
    ra_flush(e, ra, wb4);

    if (exit_pc != 0) {
        emit_store_cpu32_imm(e, (int32_t)CPU_OFF_PC, exit_pc);
        emit_store32_disp_imm(e, REG_CPU, (int32_t)CPU_OFF_PC_WRITTEN, 1);
        /* Flag a fall-through onto LEND so jit_pc_hook can tell a genuine
         * back-edge from a side exit that merely branches there. This must
         * not ride on _pc_written: clearing that would also close the hook
         * gate for any ROM or firmware stub sitting at the exit PC. */
        if (loop_end_exit)
            emit_store32_disp_imm(e, REG_CPU, (int32_t)CPU_OFF_LOOP_EXIT, 1);
    }
    emit_acc_add(e, insn_count);

    /* Record chain slot for static targets. Never chain a loop-body exit:
     * a native jump straight into a block compiled at LEND would bypass
     * jit_pc_hook entirely, and with it the lcount decrement and the branch
     * back to LBEG -- the loop would run once and fall out. */
    if (exit_pc != 0 && jit && !loop_end_exit)
        jit_chain_record(jit, exit_pc, (uint32_t)(wb4 / 4), e->ptr);

    emit_jmp_to_epilogue(e, jit);
}

/* (Legacy emit_block_exit removed — all exits go through emit_block_exit_ra) */

/* Patch a recorded chain-slot jump site to target `target`.
 * x86: jmp_site points at the 0xE9 opcode byte; rel32 follows.
 * ARM64: jmp_site points at a 4-byte B instruction; rewrite the imm26.
 * Caller holds the W^X write window; icache is invalidated here. */
static void jit_patch_chain_site(uint8_t *jmp_site, uint8_t *target) {
#ifdef JIT_ARCH_ARM64
    int64_t byte_delta = (int64_t)(target - jmp_site);
    uint32_t insn = arm64_encode_b((int32_t)(byte_delta / 4));
    memcpy(jmp_site, &insn, 4);
#  if defined(JIT_NEEDS_WX)
    sys_icache_invalidate(jmp_site, 4);
#  endif
#else
    int32_t rel = (int32_t)(target - (jmp_site + 5));
    memcpy(jmp_site + 1, &rel, 4);
#endif
}

/* Chain newly compiled block: patch any pending sites that target (pc, wb) */
static void jit_chain_new_block(jit_state_t *jit, uint32_t pc, uint32_t wb, uint8_t *entry_ptr) {
    uint32_t idx = jit_hash_key(pc, wb);
    uint32_t tag = jit_make_tag(pc, wb);
    chain_pending_t *p = &jit->pend[idx];
    if (p->tag != tag) return;
    for (uint32_t i = 0; i < p->n; i++) {
        jit_patch_chain_site(p->site[i], entry_ptr);
        jit->stats.chains_patched++;
    }
    p->tag = 0;
    p->n = 0;
}

/* Record a block exit's jump site for later chaining.
 * If the target is already compiled, patch immediately (we're inside the
 * compile-time W^X window). Otherwise pend it for jit_chain_new_block.
 * Also appends to jit->dt[] when set (descent target collection). */
static void jit_chain_record(jit_state_t *jit, uint32_t target_pc,
                             uint32_t target_wb, uint8_t *jmp_site) {
    /* Not chained under verification. A chain jumps directly between blocks,
     * so it steps over PCs at which the interpreted reference run still
     * dispatches a ROM stub -- the two stop executing comparable work and
     * every chain-cap exit reports a spurious mismatch. Block-level
     * verification stays exact; that chaining preserves results is covered
     * instead by the stock-ROM gates and by bench-compute.sh, which requires
     * the two engines to agree on a checksum over ~910M cycles.
     *
     * The native loop back-edge is deliberately *not* suppressed: it stays
     * inside one block, and fn()'s return value says exactly how many guest
     * instructions it covered, so the reference run can replay it. */
    if (jit->verify) return;
    if (jit->no_chain) return;
    if (jit->dt && jit->dt_count < JIT_MAX_BLOCK_INSNS * 2) {
        jit->dt[jit->dt_count][0] = target_pc;
        jit->dt[jit->dt_count][1] = target_wb;
        jit->dt_count++;
    }

    jit_block_t *tb = jit_lookup(jit, target_pc, target_wb);
    if (tb && tb->chain_entry) {
        jit_patch_chain_site(jmp_site, (uint8_t *)tb->chain_entry);
        jit->stats.chains_patched++;
        return;
    }

    uint32_t idx = jit_hash_key(target_pc, target_wb);
    uint32_t tag = jit_make_tag(target_pc, target_wb);
    chain_pending_t *p = &jit->pend[idx];
    if (p->tag != tag) { p->tag = tag; p->n = 0; }
    if (p->n < CHAIN_PENDING_MAX)
        p->site[p->n++] = jmp_site;
    /* else: full — chain opportunity dropped, exit still works via epilogue */
}

/* Compile a block and return the function pointer */
static jit_block_fn jit_compile_block(jit_state_t *jit, xtensa_cpu_t *cpu,
                                      uint32_t pc, jit_scan_t *scan) {
    static int no_chain = -1;
    if (__builtin_expect(no_chain < 0, 0))
        no_chain = getenv("FLEXE_JIT_NOCHAIN") != NULL;
    jit->no_chain = no_chain;

    /* Check code cache space (worst case: ~512 bytes per guest instruction for ENTRY/RETW) */
    size_t needed = (size_t)scan->count * 512 + 512;
    if (jit->code_size + needed > jit->code_capacity) {
        /* Cache full: stop compiling new blocks. Everything already in the
         * cache (blocks, chains) keeps working; unpatched exits just return
         * to the dispatcher. Flushing instead would thrash hot blocks. */
        jit->compile_disabled = true;
        return NULL;
    }

    uint8_t *code_start = jit->code_cache + jit->code_size;
    emit_t e;
    jit_wx_write_begin();
    emit_init(&e, code_start, jit->code_capacity - jit->code_size);

    /* Prologue: save callee-saved registers. */
#ifdef JIT_ARCH_ARM64
    /* stp x19,x20 / x21,x22 / x23,x24 / x25,x26 / x27,x28 / x29,x30 —
     * 6 pairs, 96 bytes, keeps SP 16-byte aligned for C calls in slow
     * paths. X29/X30 saved because BLR in slow paths clobbers LR;
     * X27 is the guest-insn accumulator (REG_ACC). */
    emit32(&e, 0xA9BF53F3u);  /* stp x19, x20, [sp, #-16]! */
    emit32(&e, 0xA9BF5BF5u);  /* stp x21, x22, [sp, #-16]! */
    emit32(&e, 0xA9BF63F7u);  /* stp x23, x24, [sp, #-16]! */
    emit32(&e, 0xA9BF6BF9u);  /* stp x25, x26, [sp, #-16]! */
    emit32(&e, 0xA9BF73FBu);  /* stp x27, x28, [sp, #-16]! */
    emit32(&e, 0xA9BF7BFDu);  /* stp x29, x30, [sp, #-16]! */
    /* x0 = cpu pointer (AAPCS64 first arg) */
    emit_mov_reg_reg(&e, REG_CPU, RAX);  /* x26 = cpu */
#else
    /* 6 pushes + return address = 7 slots = 56 bytes → RSP % 16 = 8.
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
#endif

    /* Load mem pointer */
    emit_load64_disp(&e, REG_MEM, REG_CPU, (int32_t)CPU_OFF_MEM);

    /* C entry: zero the guest-insn accumulator. Chained blocks arrive
     * with REG_ACC already accumulated from predecessor blocks. */
    emit_acc_zero(&e);

    /* Windowbase * 4 — compile-time constant per block */
    int wb4 = (int)(cpu->windowbase * 4);

    /* Timer check (C entry only): if ccount >= next_timer_event, defer to
     * jit_run. Chained entries skip this — the JIT_CHAIN_CAP bounds how
     * late a timer can fire (≤400 guest cycles), and the epilogue hands
     * back to the interpreter which fires it. */
    emit_load_cpu32(&e, RCX, (int32_t)CPU_OFF_CCOUNT);
    emit_cmp32_mem(&e, RCX, REG_CPU, (int32_t)CPU_OFF_NEXT_TIMER);
    int timer_ok = emit_jcc_rel32(&e, CC_B);
    emit_jmp_to_epilogue(&e, jit);
    emit_patch_rel32(&e, timer_ok);

    /* Chain entry point: chained blocks jump here (stack already has
     * callee-saved regs, REG_CPU=cpu, REG_MEM=mem, RAX=accumulated). */
    uint8_t *chain_entry = e.ptr;

    /* Chain-run cap: break out to the dispatcher every JIT_CHAIN_CAP
     * guest insns so timers, preemption and batch limits stay live even
     * inside self-chaining loops. REG_ACC accumulates the run total. */
    int cap_ok = emit_acc_cap_jcc(&e);
    emit_jmp_to_epilogue(&e, jit);
    emit_patch_rel32(&e, cap_ok);

    /* Register allocator: lazy load — regs are loaded from ar[] on first
     * use, so blocks only pay for the guest regs they actually touch. */
    regalloc_t ra = {0, 0};

    /* Deferred side exits for conditional branches */
    side_exit_t sx[JIT_MAX_BLOCK_INSNS];
    int sx_count = 0;

    /* Compile each instruction */
    int last_compiled = 0;
    for (int i = 0; i < scan->count; i++) {
        uint32_t next_pc = (i + 1 < scan->count) ? scan->pcs[i + 1] : scan->end_pc;
        int ok = jit_compile_insn(&e, cpu, wb4, scan->insns[i], scan->ilens[i],
                                  scan->pcs[i], next_pc, i, &ra, jit,
                                  sx, &sx_count);
        if (!ok) {
            if (i == 0) { jit_wx_write_end(code_start, 0); return NULL; }
            /* End block before this instruction */
            emit_block_exit_ra(&e, &ra, wb4, scan->pcs[i], i, jit, false);
            last_compiled = i;
            break;
        }
        last_compiled = i + 1;
    }

    /* If last instruction wasn't a terminator, add fallthrough exit.
     * cls 3 (conditional branch) also falls through to scan->end_pc. */
    if (last_compiled == scan->count) {
        int last_cls = classify_for_jit(scan->insns[scan->count - 1], scan->ilens[scan->count - 1]);
        if (last_cls == 0 || last_cls == 3) {
            if (jit_block_self_loops(cpu, scan, pc))
                emit_loop_backedge_exit(&e, &ra, wb4, cpu->lbeg, cpu->lend,
                                        scan->count, jit, chain_entry);
            else
                emit_block_exit_ra(&e, &ra, wb4, scan->end_pc, scan->count,
                                   jit, scan->ends_at_lend);
        }
    }

    /* Emit the deferred side-exit stubs (targets of the in-body jcc's) */
    for (int k = 0; k < sx_count; k++) {
        if (sx[k].target_pc == pc) {
            /* Self-target (loop back-edge to this block's own head): the
             * exit pc never changes and _pc_written is already set from
             * the interpreted branch that first got us here, so the full
             * stub's stores are dead work. Emit just the accounting + jump. */
            emit_patch_rel32(&e, sx[k].patch_site);
            emit_acc_add(&e, sx[k].insn_count);
            jit_chain_record(jit, sx[k].target_pc, sx[k].target_wb, e.ptr);
            emit_jmp_to_epilogue(&e, jit);
        } else {
            emit_side_exit_body(&e, &sx[k], jit);
        }
    }

    if (!emit_ok(&e)) { jit_wx_write_end(code_start, 0); return NULL; }

    jit->code_size += emit_size(&e);
    jit->stats.blocks_compiled++;
    jit->last_chain_entry = chain_entry;

    static int dbg_jit = -1;
    if (__builtin_expect(dbg_jit < 0, 0))
        dbg_jit = getenv("FLEXE_JIT_DEBUG") != NULL;
    if (__builtin_expect(dbg_jit, 0)) {
        fprintf(stderr, "[JIT] compile pc=0x%08X wb=%d insns=%d size=%zu host=%p cache=%p\n",
                pc, cpu->windowbase, scan->count, emit_size(&e),
                (void *)code_start, (void *)jit->code_cache);
        for (size_t bi = 0; bi < emit_size(&e); bi += 4)
            fprintf(stderr, "      +%03zx: %02x%02x%02x%02x\n", bi,
                    code_start[bi+3], code_start[bi+2], code_start[bi+1], code_start[bi]);
    }

    /* Patch any pending chain sites that target this block.
     * Chain sites jump to chain_entry (past prologue), not code_start.
     * NOTE: still inside the W^X write window — chain patches mutate
     * previously-emitted code in the cache. */
    jit_chain_new_block(jit, pc, cpu->windowbase, chain_entry);

    jit_wx_write_end(code_start, emit_size(&e));
    return (jit_block_fn)code_start;
}


/* ===== Public API ===== */

/* Emit the shared epilogue stub: restore callee-saved regs saved by the
 * block prologue, then return to the C caller (jit_pc_hook). Return value
 * (block insn count) is already in RAX/X0. */
static void jit_emit_epilogue_stub(emit_t *e) {
#ifdef JIT_ARCH_ARM64
    /* Return value: guest-insn accumulator X27 → W0 (before restore). */
    emit_mov_reg32_reg32(e, RAX, REG_ACC);
    emit32(e, 0xA8C17BFDu);  /* ldp x29, x30, [sp], #16 */
    emit32(e, 0xA8C173FBu);  /* ldp x27, x28, [sp], #16 */
    emit32(e, 0xA8C16BF9u);  /* ldp x25, x26, [sp], #16 */
    emit32(e, 0xA8C163F7u);  /* ldp x23, x24, [sp], #16 */
    emit32(e, 0xA8C15BF5u);  /* ldp x21, x22, [sp], #16 */
    emit32(e, 0xA8C153F3u);  /* ldp x19, x20, [sp], #16 */
    emit_ret(e);
#else
    /* Return value: guest-insn accumulator cpu->jit_acc → EAX. Must happen
     * while REG_CPU is still live, i.e. before the pops below. */
    emit_load_cpu32(e, RAX, (int32_t)CPU_OFF_JIT_ACC);
    /* Matches prologue: 6 pushes + sub rsp,8 → add rsp,8 + 6 pops + ret. */
    emit_add_reg64_imm32(e, RSP, 8);
    emit_pop(e, R12);
    emit_pop(e, R13);
    emit_pop(e, R14);
    emit_pop(e, R15);
    emit_pop(e, RBP);
    emit_pop(e, RBX);
    emit_ret(e);
#endif
}

jit_state_t *jit_init(void) {
    jit_state_t *jit = calloc(1, sizeof(jit_state_t));
    if (!jit) return NULL;

    /* mmap executable code cache */
#if defined(JIT_NEEDS_WX)
    jit->code_cache = mmap(NULL, JIT_CODE_CACHE_SIZE,
                           PROT_READ | PROT_WRITE | PROT_EXEC,
                           MAP_PRIVATE | MAP_ANONYMOUS | MAP_JIT, -1, 0);
#else
    jit->code_cache = mmap(NULL, JIT_CODE_CACHE_SIZE,
                           PROT_READ | PROT_WRITE | PROT_EXEC,
                           MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
#endif
    if (jit->code_cache == MAP_FAILED) {
        free(jit);
        return NULL;
    }
    jit->code_capacity = JIT_CODE_CACHE_SIZE;
    jit->code_size = 0;

    /* Emit shared epilogue stub at the start of the code cache. */
    jit_wx_write_begin();
    emit_t stub_e;
    emit_init(&stub_e, jit->code_cache, 64);
    jit->epilogue_stub = stub_e.ptr;
    jit_emit_epilogue_stub(&stub_e);
    jit->code_size = emit_size(&stub_e);
    jit_wx_write_end(jit->code_cache, jit->code_size);

    static int dbg_init = -1;
    if (dbg_init < 0) dbg_init = getenv("FLEXE_JIT_DEBUG") != NULL;
    if (dbg_init)
        fprintf(stderr, "[JIT] Initialized: %u MB code cache, %u-entry hash table, epilogue at +0\n",
                JIT_CODE_CACHE_SIZE / (1024 * 1024), JIT_HASH_SIZE);

    return jit;
}

/* Set a bit in the JIT bitmap for a compiled block PC */
static void jit_bitmap_set(jit_state_t *jit, uint32_t pc) {
    uint32_t idx = (pc >> 2) & (HOOK_BITMAP_BITS - 1);
    jit->jit_bitmap[idx / 64] |= (1ULL << (idx & 63));
}


/* ===== Differential block verification =====
 *
 * Runs a compiled block, undoes everything it did, re-runs the identical
 * guest instructions through the interpreter, and compares. The interpreter
 * is the reference, so execution continues from *its* state -- a wrong block
 * is reported rather than allowed to corrupt the run, which means a firmware
 * image can be taken all the way to completion under verification and every
 * bad block along the way gets named.
 *
 * Only architectural state is compared. ccount and cycle_count are excluded
 * on purpose: the two engines account for a block's cost differently by
 * design (the hook adds block_insns - 1 for a native block), and that is not
 * a miscompile.
 */
#define MEM_JOURNAL_MAX_COMPARE 1024

typedef struct {
    uint32_t ar[64];
    uint32_t pc, sar, ps, lbeg, lend, lcount, windowbase, windowstart;
    uint16_t br;
} jit_arch_state_t;

static void jit_arch_capture(const xtensa_cpu_t *cpu, jit_arch_state_t *st) {
    memcpy(st->ar, cpu->ar, sizeof st->ar);
    st->pc = cpu->pc;             st->sar = cpu->sar;
    st->ps = cpu->ps;             st->lbeg = cpu->lbeg;
    st->lend = cpu->lend;         st->lcount = cpu->lcount;
    st->windowbase = cpu->windowbase;
    st->windowstart = cpu->windowstart;
    st->br = cpu->br;
}

static int jit_arch_report(const jit_arch_state_t *ref,
                           const jit_arch_state_t *got, uint32_t pc) {
    int diffs = 0;
    for (int i = 0; i < 64; i++) {
        if (ref->ar[i] != got->ar[i]) {
            fprintf(stderr, "[jit-verify] block %08X: ar[%d] interp=%08X "
                    "jit=%08X\n", pc, i, ref->ar[i], got->ar[i]);
            diffs++;
        }
    }
#define JV_CMP(field, fmt) \
    if (ref->field != got->field) { \
        fprintf(stderr, "[jit-verify] block %08X: " #field \
                " interp=" fmt " jit=" fmt "\n", pc, ref->field, got->field); \
        diffs++; \
    }
    JV_CMP(pc, "%08X") JV_CMP(sar, "%u") JV_CMP(ps, "%08X")
    JV_CMP(lbeg, "%08X") JV_CMP(lend, "%08X") JV_CMP(lcount, "%u")
    JV_CMP(windowbase, "%u") JV_CMP(windowstart, "%04X")
#undef JV_CMP
    if (ref->br != got->br) {
        fprintf(stderr, "[jit-verify] block %08X: br interp=%04X jit=%04X\n",
                pc, ref->br, got->br);
        diffs++;
    }
    return diffs;
}

/* Apply the post-block bookkeeping the fast path performs, so the state the
 * verifier compares is the state the rest of the emulator would see. */
static void jit_apply_block_exit(xtensa_cpu_t *cpu) {
    uint32_t loop_exit = cpu->jit_loop_exit;
    cpu->jit_loop_exit = 0;
    if (loop_exit && cpu->lcount > 0 && cpu->pc == cpu->lend) {
        cpu->lcount--;
        cpu->pc = cpu->lbeg;
        cpu->_pc_written = 1;
    }
}

/* The native block runs *first* and is then undone, which is only possible
 * because verification compiles stores through mem_write*_journaled(). That
 * ordering is what lets a chained run be checked at all: one call to fn() may
 * cover many blocks, and only its return value says how many guest
 * instructions that was -- which the interpreted reference then replays
 * exactly. */
static int jit_run_block_verified(jit_state_t *jit, xtensa_cpu_t *cpu,
                                  uint32_t pc, jit_block_fn fn) {
    xtensa_cpu_t before = *cpu;

    mem_journal_begin();
    int n_jit = fn(cpu);
    if (n_jit > 0) jit_apply_block_exit(cpu);
    int unsafe = g_mem_journal_unsafe;
    int nwrites = g_mem_journal_count;
    g_mem_journal_en = 0;

    if (n_jit <= 0) {
        mem_journal_end();
        return n_jit;
    }
    /* MMIO in the window: the block cannot be undone, because reading a
     * device register to save it is itself a side effect. Keep the native
     * result and count a skip. */
    if (unsafe || nwrites > MEM_JOURNAL_MAX_COMPARE) {
        jit->verify_skipped++;
        mem_journal_end();
        return n_jit;
    }

    jit_arch_state_t jit_state;
    jit_arch_capture(cpu, &jit_state);

    /* What the block left at every address it touched, and what was there
     * before, so an address only the interpreter writes can be compared too. */
    uint32_t waddr[MEM_JOURNAL_MAX_COMPARE];
    uint32_t wjit[MEM_JOURNAL_MAX_COMPARE];
    uint32_t wold[MEM_JOURNAL_MAX_COMPARE];
    int wn = nwrites;
    for (int i = 0; i < wn; i++) {
        waddr[i] = g_mem_journal[i].addr;
        wold[i] = g_mem_journal[i].old;
        wjit[i] = mem_read32(cpu->mem, waddr[i]);
    }
    mem_journal_rollback(cpu->mem);
    mem_journal_end();
    *cpu = before;

    /* Reference run: the same guest instructions, interpreted. Journalled as
     * well, so a store the block *failed* to make is caught too. */
    mem_journal_begin();
    jit->verify_active = true;
    for (int i = 0; i < n_jit && cpu->running; i++)
        xtensa_step(cpu);
    jit->verify_active = false;
    int iwrites = g_mem_journal_count;
    int ionly[MEM_JOURNAL_MAX_COMPARE];
    int in = iwrites < MEM_JOURNAL_MAX_COMPARE ? iwrites
                                               : MEM_JOURNAL_MAX_COMPARE;
    for (int i = 0; i < in; i++)
        ionly[i] = i;
    g_mem_journal_en = 0;

    /* An interrupt or exception taken by the reference run is not a
     * miscompile: a native block defers interrupts to its exit, while
     * xtensa_step() checks after every instruction. */
    if (cpu->exception || (XT_PS_EXCM(cpu->ps) && !XT_PS_EXCM(before.ps))) {
        jit->verify_skipped++;
        mem_journal_end();
        cpu->ccount = before.ccount;
        cpu->cycle_count = before.cycle_count;
        return n_jit;
    }

    jit_arch_state_t interp_state;
    jit_arch_capture(cpu, &interp_state);

    jit->verify_blocks++;
    int diffs = jit_arch_report(&interp_state, &jit_state, pc);
    for (int i = 0; i < wn; i++) {
        uint32_t got = mem_read32(cpu->mem, waddr[i]);
        if (got != wjit[i]) {
            fprintf(stderr, "[jit-verify] block %08X: mem[%08X] interp=%08X "
                    "jit=%08X\n", pc, waddr[i], got, wjit[i]);
            diffs++;
        }
    }
    /* Addresses only the reference touched: the block left them untouched, so
     * its value is whatever was there before. */
    for (int k = 0; k < in; k++) {
        uint32_t addr = g_mem_journal[ionly[k]].addr;
        bool seen = false;
        for (int i = 0; i < wn; i++)
            if (waddr[i] == addr) { seen = true; break; }
        if (seen) continue;
        uint32_t got = mem_read32(cpu->mem, addr);
        uint32_t was = g_mem_journal[ionly[k]].old;
        if (got != was) {
            fprintf(stderr, "[jit-verify] block %08X: mem[%08X] interp=%08X "
                    "jit=%08X (block never wrote it)\n", pc, addr, got, was);
            diffs++;
        }
    }
    mem_journal_end();
    if (diffs > 0) jit->verify_mismatches++;

    /* Continue from the reference state: it is by definition correct, so a
     * verification run of a whole firmware image stays on the rails and
     * reports every bad block instead of derailing at the first. Cycle
     * accounting is left to the caller, which adds n_jit - 1 on top of the
     * step that dispatched here. */
    (void)wold;
    cpu->ccount = before.ccount;
    cpu->cycle_count = before.cycle_count;
    return n_jit;
}

/* JIT pc_hook: called by the interpreter for every bitmap-marked PC.
 * Direct hash lookup (no JIT bitmap check) — O(1) with one memory access.
 * Flow: check JIT hash → if hit, run block; if miss, forward to ROM stubs. */
static int jit_pc_hook(xtensa_cpu_t *cpu, uint32_t pc, void *ctx) {
    jit_state_t *jit = ctx;

    /* Inside a verification re-run: the interpreter is the reference, so it
     * must not dispatch back into the code being checked. ROM stubs stay
     * live -- they are part of the semantics being compared. */
    if (__builtin_expect(jit->verify_active, 0)) {
        if (jit->original_hook)
            return jit->original_hook(cpu, pc, jit->original_hook_ctx);
        return 0;
    }

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
        jit->execution_depth++;
        int block_insns = __builtin_expect(jit->verify, 0)
                        ? jit_run_block_verified(jit, cpu, pc, fn)
                        : fn(cpu);
        jit->execution_depth--;
        if (jit->invalidate_pending)
            jit_flush(jit);

        if (block_insns > 0) {
            /* Apply the zero-overhead loop back-edge. Only a fall-through
             * onto LEND closes the loop; a side exit that branches there is
             * a break out of it and must not consume an iteration.
             * jit_run_block_verified() has already done this for its own
             * comparison, so skip it there. */
            if (!jit->verify) jit_apply_block_exit(cpu);

            /* Advance ccount by block_insns - 1 (the interpreter adds 1) */
            cpu->ccount += (uint32_t)(block_insns - 1);
            cpu->cycle_count += (uint64_t)(block_insns - 1);

            jit->stats.blocks_executed++;
            jit->stats.insns_jitted += (uint64_t)block_insns;

            /* Grow the chain: hot-count the block's exit target. After a
             * few visits it compiles and this block's exit gets patched
             * to jump straight into it. Without this, compilation only
             * triggers on jit_run's 1-per-1000 batch sampling and chains
             * never form. */
            uint32_t npc = cpu->pc;
            if (__builtin_expect(npc >= ESP32_FIRMWARE_INSN_ADDR_LOW &&
                                 npc < ESP32_INSN_ADDR_HIGH, 1))
                jit_get_block(jit, cpu, npc);

            return block_insns; /* Handled; expose exact guest work */
        }
    }

    /* Forward to original hook (ROM stubs) */
    if (jit->original_hook) {
        return jit->original_hook(cpu, pc, jit->original_hook_ctx);
    }

    return 0;
}

static void jit_code_invalidate(void *ctx, uint32_t addr, size_t len) {
    (void)addr;
    (void)len;
    jit_flush(ctx);
}

/* Install JIT as a pc_hook, chaining with the existing hook */
void jit_install_hook(jit_state_t *jit, xtensa_cpu_t *cpu) {
    /* Save the original hook */
    jit->original_hook = cpu->pc_hook;
    jit->original_hook_ctx = cpu->pc_hook_ctx;

    /* Swap in the merged hook bitmap (ROM bits | JIT bits). Done once —
     * both cores share firmware code, so one merged copy serves all. */
    if (!jit->merged_bitmap) {
        jit->orig_bitmap = (const uint64_t *)cpu->pc_hook_bitmap;
        jit->merged_bitmap = malloc(HOOK_BITMAP_WORDS * sizeof(uint64_t));
        if (jit->merged_bitmap) {
            if (jit->orig_bitmap)
                memcpy(jit->merged_bitmap, jit->orig_bitmap,
                       HOOK_BITMAP_WORDS * sizeof(uint64_t));
            else
                memset(jit->merged_bitmap, 0, HOOK_BITMAP_WORDS * sizeof(uint64_t));
        }
    }
    if (jit->merged_bitmap)
        cpu->pc_hook_bitmap = jit->merged_bitmap;

    /* Install JIT hook */
    cpu->pc_hook = jit_pc_hook;
    cpu->pc_hook_ctx = jit;
    cpu->accelerated_blocks = true;
    cpu->code_invalidate = jit_code_invalidate;
    cpu->code_invalidate_ctx = jit;

    /* The bitmap stays the same — JIT bits are added on top of ROM stub bits.
     * The interpreter's bitmap test will fire for both stubs and JIT blocks. */
}

void jit_destroy(jit_state_t *jit) {
    if (!jit) return;
    if (jit->code_cache && jit->code_cache != MAP_FAILED)
        munmap(jit->code_cache, jit->code_capacity);
    free(jit->merged_bitmap);
    free(jit);
}

void jit_flush(jit_state_t *jit) {
    if (!jit) return;
    if (jit->execution_depth != 0) {
        jit->invalidate_pending = true;
        return;
    }
    memset(jit->hash, 0, sizeof(jit->hash));
    memset(jit->pend, 0, sizeof(jit->pend));
    memset(jit->jit_bitmap, 0, sizeof(jit->jit_bitmap));
    if (jit->merged_bitmap) {
        if (jit->orig_bitmap)
            memcpy(jit->merged_bitmap, jit->orig_bitmap,
                   HOOK_BITMAP_WORDS * sizeof(uint64_t));
        else
            memset(jit->merged_bitmap, 0,
                   HOOK_BITMAP_WORDS * sizeof(uint64_t));
    }
    jit->last_chain_entry = NULL;
    jit->dt = NULL;
    jit->dt_count = 0;
    jit->compile_disabled = false;
    jit->invalidate_pending = false;
    /* Reset code cache but preserve epilogue stub. Re-emit it to be safe. */
    jit_wx_write_begin();
    emit_t stub_e;
    emit_init(&stub_e, jit->code_cache, 64);
    jit->epilogue_stub = stub_e.ptr;
    jit_emit_epilogue_stub(&stub_e);
    jit->code_size = emit_size(&stub_e);
    jit_wx_write_end(jit->code_cache, jit->code_size);
    jit->stats.cache_flushes++;
}

/* Force-compile (pc, wb) if not already compiled, then recursively
 * compile its static branch targets up to JIT_DESCEND_DEPTH levels.
 *
 * Why: the interpreter only fires the hook at control-flow targets whose
 * bitmap bit is set. A block compiled from batch sampling usually sits in
 * the MIDDLE of a hot region — straight-line execution never reaches it,
 * so without descent the compiled block is dead code. Following static
 * exit targets (in particular back-edges) compiles the loop head, whose
 * bit IS hit on every iteration. */
#define JIT_DESCEND_DEPTH 1
static void jit_compile_now(jit_state_t *jit, xtensa_cpu_t *cpu,
                            uint32_t pc, uint32_t wb, int depth) {
    if (depth > JIT_DESCEND_DEPTH || jit->compile_disabled) return;
    /* Cache-pressure adaptive: once the cache is 75% full, only genuinely
     * hot code may compile (the last quarter is reserved for it). */
    if (jit->code_size > (jit->code_capacity * 3) / 4 && depth > 0) return;
    jit_block_t *b = jit_lookup(jit, pc, wb);
    if (b && b->code) return;

    b = jit_get_or_create(jit, pc, wb);

    jit_scan_t scan;
    jit_scan_block(jit, cpu, pc, &scan);
    if (scan.count < 4 && !jit_short_block_has_backedge(&scan, pc))
        return;  /* short straight-line block: dispatch overhead dominates */

    uint32_t saved_wb = cpu->windowbase;
    cpu->windowbase = wb;

    /* Collect static branch targets for the descent loop. Save/restore the
     * parent's collection state — recursive descent calls reuse the field. */
    uint32_t dt[JIT_MAX_BLOCK_INSNS * 2][2];
    uint32_t (*saved_dt)[2] = jit->dt;
    int saved_dt_count = jit->dt_count;
    jit->dt = dt;
    jit->dt_count = 0;

    jit_block_fn fn = jit_compile_block(jit, cpu, pc, &scan);
    int dt_count = jit->dt_count;
    jit->dt = saved_dt;
    jit->dt_count = saved_dt_count;
    cpu->windowbase = saved_wb;
    if (!fn) return;

    /* FLEXE_COMPILEDBG lists every block as it compiles. Diffing that list
     * between a passing and a failing run is what localises a miscompile to
     * a block: an identical list means the fault is value-dependent inside a
     * block that was already there, a longer one names the new suspect. */
    {
        static int dbg = -1;
        if (dbg < 0) dbg = getenv("FLEXE_COMPILEDBG") != NULL;
        if (dbg)
            fprintf(stderr, "[compile] pc=%08X wb=%u insns=%d end=%08X "
                    "lend_block=%d lcount=%u lbeg=%08X lend=%08X\n",
                    pc, wb, scan.count, scan.end_pc, (int)scan.ends_at_lend,
                    cpu->lcount, cpu->lbeg, cpu->lend);
    }
    b->code = (void *)fn;
    b->chain_entry = (void *)jit->last_chain_entry;
    b->guest_insns = (uint16_t)scan.count;

    /* Set JIT bitmap bit so the interpreter's hook fires for this PC */
    jit_bitmap_set(jit, pc);
    if (cpu->pc_hook_bitmap) {
        uint64_t *bm = (uint64_t *)(uintptr_t)cpu->pc_hook_bitmap;
        uint32_t idx = (pc >> 2) & (HOOK_BITMAP_BITS - 1);
        bm[idx / 64] |= (1ULL << (idx & 63));
    }

    /* Descend into static branch targets recorded during this compile.
     * Backward edges only: those are loop back-edges, whose targets (loop
     * heads) are the PCs the interpreter's hook actually hits each
     * iteration. Forward paths get compiled lazily via exit hot-counting. */
    for (int i = 0; i < dt_count; i++) {
        if (dt[i][0] < pc)
            jit_compile_now(jit, cpu, dt[i][0], dt[i][1], depth + 1);
    }
}

jit_block_fn jit_get_block(jit_state_t *jit, xtensa_cpu_t *cpu, uint32_t pc) {
    uint32_t wb = cpu->windowbase;
    jit_block_t *b = jit_lookup(jit, pc, wb);
    if (b && b->code)
        return (jit_block_fn)b->code;

    /* Get or create entry */
    b = jit_get_or_create(jit, pc, wb);
    b->exec_count++;

    /* Cache-pressure adaptive threshold: the last quarter of the cache is
     * reserved for code that is hot for real. */
    uint32_t threshold = JIT_HOT_THRESHOLD;
    if (jit->code_size > (jit->code_capacity * 3) / 4)
        threshold = 64;
    if (b->exec_count < threshold)
        return NULL;  /* Not hot yet */

    jit_compile_now(jit, cpu, pc, wb, 0);
    return (b->code) ? (jit_block_fn)b->code : NULL;
}

/* Main JIT execution loop.
 * With jit_install_hook, the interpreter's pc_hook dispatches compiled JIT
 * blocks transparently via the bitmap. The native chain cap and interpreter
 * timer checks already bound latency, so consume the caller's whole batch;
 * subdividing it into 1000-instruction slices caused ten times more sampling
 * and compilation work in mostly-cold production firmware. */
__attribute__((hot))
int jit_run(jit_state_t *jit, xtensa_cpu_t *cpu, int max_cycles) {
    int executed = 0;
    uint64_t jit_insns_before = jit->stats.insns_jitted;

    while (__builtin_expect(cpu->running, 1) &&
           __builtin_expect(!cpu->breakpoint_hit, 1) &&
           executed < max_cycles) {
        int remaining = max_cycles - executed;

        if (__builtin_expect(cpu->halted, 0)) {
            /* xtensa_step_impl already advances halted time and checks the
             * wake interrupt on every cycle. Running one dispatch per outer
             * iteration turned WAITI-heavy firmware into thousands of tiny C
             * calls per frontend batch, more than doubling NerdMiner's wall
             * time. Let the tight runner consume the remaining batch; if an
             * interrupt wakes the core it naturally resumes guest code. */
            int ran = xtensa_run(cpu, remaining);
            if (ran <= 0) break;
            executed += ran;
            continue;
        }

        int batch = remaining;
        int ran = xtensa_run(cpu, batch);
        if (ran > 0)
            executed += ran;

        /* Hot-counting: check if current PC is a JIT candidate.
         * Triggers compilation for frequently-visited firmware PCs.
         * Once compiled, the bitmap ensures the hook dispatches directly. */
        uint32_t pc = cpu->pc;
        if (__builtin_expect(pc >= ESP32_FIRMWARE_INSN_ADDR_LOW &&
                             pc < ESP32_INSN_ADDR_HIGH, 1)) {
            jit_get_block(jit, cpu, pc);
        }
        /* Inside an active zero-overhead loop, sample LBEG as well.  LOOP
         * itself is not compilable, so a block sampled at an arbitrary PC in
         * the body can only ever reach LEND — yielding short tail fragments
         * that re-dispatch every iteration.  The profitable block is the one
         * starting at LBEG, which spans the whole body and which jit_pc_hook
         * already branches back to when a block ends at LEND.  Compilers emit
         * LOOP for exactly the hot inner loops that matter most, so without
         * this the JIT stays near interpreter speed on loop-heavy code. */
        if (__builtin_expect(cpu->lcount > 0, 0)) {
            uint32_t lbeg = cpu->lbeg;
            if (lbeg != pc && lbeg >= ESP32_FIRMWARE_INSN_ADDR_LOW &&
                lbeg < ESP32_INSN_ADDR_HIGH)
                jit_get_block(jit, cpu, lbeg);
        }

        if (__builtin_expect(ran < batch, 0)) {
            if (!cpu->running || cpu->halted || cpu->breakpoint_hit) break;
            if (ran <= 0) break;
        }
    }

    /* CCOUNT is hardware time, not an instruction-retirement counter: delay
     * and scheduler stubs legitimately fast-forward it by millions of ticks.
     * xtensa_run() already reports exact guest work, including native blocks,
     * so use that value for batching and statistics. */
    uint64_t jit_insns_delta = jit->stats.insns_jitted - jit_insns_before;
    if ((uint64_t)executed > jit_insns_delta)
        jit->stats.insns_interp += (uint64_t)executed - jit_insns_delta;
    return executed;
}

const jit_stats_t *jit_get_stats(const jit_state_t *jit) {
    return &jit->stats;
}

void jit_set_verify(jit_state_t *jit, bool enable) {
    if (jit) jit->verify = enable;
}

void jit_verify_summary(const jit_state_t *jit) {
    if (!jit || !jit->verify) return;
    fprintf(stderr, "[jit-verify] %llu blocks checked, %llu mismatching, "
            "%llu skipped (MMIO or interrupt)\n",
            (unsigned long long)jit->verify_blocks,
            (unsigned long long)jit->verify_mismatches,
            (unsigned long long)jit->verify_skipped);
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
