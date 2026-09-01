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

/* Compilation threshold: compile after N interpreter executions */
/* Production firmware crosses thousands of cold control-flow targets during
 * startup. Compiling after only three observations spends more time toggling
 * W^X and emitting code than it saves; genuinely hot loops reach 16 quickly. */
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
    void    *code;          /* Pointer into code cache (NULL = empty) */
    void    *chain_entry;   /* Entry point for chained blocks (after prologue) */
    uint32_t exec_count;    /* Hot counter / execution count */
    uint16_t guest_insns;   /* Number of guest instructions in block */
    uint16_t flags;         /* Reserved */
} jit_block_t;

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
void               jit_print_stats(const jit_state_t *jit);

/* Differential verification mode: every compiled block is executed natively,
 * rolled back, re-executed through the interpreter, and the two architectural
 * states compared. Mismatches are reported and execution continues from the
 * interpreter's (reference) state. Blocks that touch MMIO cannot be replayed
 * and are skipped. Roughly an order of magnitude slower -- a debugging tool,
 * not a run mode. */
void               jit_set_verify(jit_state_t *jit, bool enable);
void               jit_verify_summary(const jit_state_t *jit);

#endif /* JIT_H */
