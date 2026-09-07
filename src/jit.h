#ifndef JIT_H
#define JIT_H

#include "xtensa.h"
#include <stdint.h>
#include <stdbool.h>

/* JIT compiler for Xtensa LX6 → x86-64 and ARM64 native code.
 * Translates hot basic blocks to native machine code, falling back
 * to the interpreter for cold code and complex instructions. */

/* Code cache: 128MB mmap'd executable region (lazily committed) */
#define JIT_CODE_CACHE_SIZE  (128u * 1024 * 1024)

/* Block lookup hash table: 64K entries */
#define JIT_HASH_BITS   16
#define JIT_HASH_SIZE   (1u << JIT_HASH_BITS)
#define JIT_HASH_MASK   (JIT_HASH_SIZE - 1)

/* The block table is set-associative. It used to be direct-mapped with one
 * entry per slot, and any colliding key reset the slot -- discarding compiled
 * code. Because hot-counting calls jit_get_or_create() for *every* sampled PC,
 * most of which never compile, cold code continuously destroyed hot code:
 * Marauder compiled 20,842 blocks covering only 87 distinct PCs, one pair of
 * them 5,487 times each. Four ways at 32 bytes per entry keeps a set inside
 * two cache lines. */
#define JIT_WAYS        4u
#define JIT_SET_COUNT   (JIT_HASH_SIZE / JIT_WAYS)
#define JIT_SET_MASK    (JIT_SET_COUNT - 1)

/* Compilation threshold: compile after N interpreter executions.
 *
 * Production firmware crosses thousands of cold control-flow targets during
 * startup. Compiling after only three observations spends more time toggling
 * W^X and emitting code than it saves; genuinely hot loops reach 16 quickly.
 *
 * Measured, so it does not need re-deriving: on the stock ROMs, lowering this
 * to 2 compiles twenty times as many blocks (18 -> 375 on NerdMiner) and
 * changes neither the share of instructions executed natively nor the wall
 * time. Sampling candidate PCs more often does not move it either. Those ROMs
 * simply have no further hot code to find -- 22.5% to 46% of retired
 * instructions already run natively and the rest is genuinely diffuse: the
 * single hottest PC accounts for about 0.06% of samples, and the interpreted
 * remainder averages 3.6 instructions per taken control transfer.
 *
 * The percentages here were re-derived on 2026-09-04 after the retired-
 * instruction counter was found to be inflated 6.7x; the *comparisons* were
 * unaffected, since both arms of each experiment shared the denominator. */
#define JIT_HOT_THRESHOLD  16

/* Maximum guest instructions per block */
#define JIT_MAX_BLOCK_INSNS  64

/* Max guest instructions executed in one chained JIT run before breaking
 * out to the dispatcher. Keeps timers, FreeRTOS preemption and the -c
 * batch budget live inside self-chaining loops. A run may overshoot by at
 * most the final block's length, since the cap is tested at block exits. */
#define JIT_CHAIN_CAP  400

/* Block entry in the hash table */
typedef struct {
    uint32_t pc;            /* Guest PC (tag for collision detection) */
    uint32_t end_pc;        /* First guest PC past the block */
    void    *code;          /* Pointer into code cache (NULL = empty) */
    void    *chain_entry;   /* Entry point for chained blocks (after prologue) */
    uint32_t exec_count;    /* Hot counter / execution count */
    uint16_t guest_insns;   /* Number of guest instructions in block */
    uint16_t flags;         /* JIT_BLK_* */
} jit_block_t;

/* Scanning this (pc, windowbase, loop-variant) produced nothing worth
 * compiling, and will again: the scan depends only on the instruction stream,
 * the loop context that the cache key already distinguishes, and the ROM-stub
 * bitmap, none of which change without a flush. Without remembering it, a hot
 * PC the JIT declines is re-scanned on every single execution. */
#define JIT_BLK_UNCOMPILABLE 0x1u

/* Way is in use. Needed because a zeroed way and a real entry are otherwise
 * indistinguishable by tag alone, and the victim search has to tell "free"
 * from "occupied but not yet compiled". */
#define JIT_BLK_VALID        0x2u

/* Block chaining: max pending chain slots */
#define MAX_CHAIN_SLOTS  131072

/* Pending chain sites for one (pc, wb): exits of blocks that branched here
 * before the target was compiled. Patched (to the target's chain_entry)
 * when it compiles. Collisions evict — a lost chain just costs an epilogue
 * round-trip, never correctness. */
#define CHAIN_PENDING_MAX 4
typedef struct {
    uint32_t  tag;                    /* pc ^ (wb<<28), 0 = empty */
    uint32_t  n;                      /* sites used (≤ CHAIN_PENDING_MAX) */
    uint8_t  *site[CHAIN_PENDING_MAX];
} chain_pending_t;

/* JIT statistics */
typedef struct {
    uint64_t blocks_compiled;
    uint64_t blocks_executed;
    uint64_t insns_jitted;      /* Guest insns executed via JIT */
    uint64_t insns_interp;      /* Guest insns executed via interpreter */
    uint64_t cache_flushes;
    uint64_t fallbacks;         /* Instructions that fell back to interpreter */
    uint64_t chains_patched;    /* Block chain links patched */
    /* Calls into jit_pc_hook. Against blocks_executed this is the *hit rate*
     * of the dispatch path -- 0.83 on Marauder, 0.50 on NerdMiner -- because
     * blocks_executed counts C dispatches that found a block, not native
     * blocks run. A chained run jumps block to block without returning, and
     * shows up only as a larger insns_jitted. So the remainder is dispatches
     * that found nothing: 13% misses and 4% ROM stubs on Marauder, 30% and
     * 20% on NerdMiner.
     *
     * Do not read this ratio as chaining reach; it cannot fall below 1.0 by
     * chaining more, and an earlier note here claimed exactly that. For
     * chaining, divide insns_jitted/blocks_executed by the natural basic-block
     * length: 9.1/6.1 = 1.5 blocks per dispatch on Marauder, 25.1/4.2 = 6.0 on
     * NerdMiner. Chaining is working. */
    uint64_t hook_calls;
    /* Cached blocks declined because a live loop ends inside them. Each one
     * is an interpreted dispatch, and because the decline follows a hash hit
     * no replacement is ever compiled — so a large count here is a hot loop
     * running interpreted, which costs far more than the count suggests. */
    uint64_t loop_bound_rejects;
} jit_stats_t;

/* Opaque JIT state */
typedef struct jit_state jit_state_t;

/* JIT compiled block function signature.
 * Returns: number of guest instructions executed in this block.
 * The block updates cpu->pc, cpu->ccount, cpu->cycle_count, etc. */
typedef int (*jit_block_fn)(xtensa_cpu_t *cpu);

/* Public API */
jit_state_t *jit_init(void);
void         jit_destroy(jit_state_t *jit);
void         jit_flush(jit_state_t *jit);

/* Look up or compile a block for the given PC.
 * Returns compiled block function, or NULL if not yet hot enough. */
jit_block_fn jit_get_block(jit_state_t *jit, xtensa_cpu_t *cpu, uint32_t pc);

/* Main JIT execution loop — replaces xtensa_run() when JIT is enabled */
int          jit_run(jit_state_t *jit, xtensa_cpu_t *cpu, int max_cycles);

/* Install JIT as a pc_hook on the given CPU, chaining with the existing hook.
 * After this, xtensa_run() will automatically dispatch to JIT blocks. */
void         jit_install_hook(jit_state_t *jit, xtensa_cpu_t *cpu);

/* Statistics */
const jit_stats_t *jit_get_stats(const jit_state_t *jit);
/* `retired_insns` is the sum of xtensa_retired_insns() over both cores.
 * Coverage is reported against it rather than against the JIT's own batch
 * counters, which include idle and fast-forwarded cycles. Pass 0 if unknown. */
void               jit_print_stats(const jit_state_t *jit,
                                   uint64_t retired_insns);

/* Differential verification mode: every compiled block is executed natively,
 * rolled back, re-executed through the interpreter, and the two architectural
 * states compared. Mismatches are reported and execution continues from the
 * interpreter's (reference) state. Blocks that touch MMIO cannot be replayed
 * and are skipped. Roughly an order of magnitude slower -- a debugging tool,
 * not a run mode. */
void               jit_set_verify(jit_state_t *jit, bool enable);
bool               jit_verify_enabled(const jit_state_t *jit);
void               jit_verify_summary(const jit_state_t *jit);

#endif /* JIT_H */
