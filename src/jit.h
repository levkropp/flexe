#ifndef JIT_H
#define JIT_H

#include "xtensa.h"
#include <stdint.h>
#include <stdbool.h>

/* JIT compiler for Xtensa LX6 → x86-64 native code.
 * Translates hot basic blocks to native machine code, falling back
 * to the interpreter for cold code and complex instructions. */

/* Code cache: 32MB mmap'd executable region */
#define JIT_CODE_CACHE_SIZE  (32u * 1024 * 1024)

/* Block lookup hash table: 64K entries */
#define JIT_HASH_BITS   16
#define JIT_HASH_SIZE   (1u << JIT_HASH_BITS)
#define JIT_HASH_MASK   (JIT_HASH_SIZE - 1)

/* Compilation threshold: compile after N interpreter executions */
#define JIT_HOT_THRESHOLD  8

/* Maximum guest instructions per block */
#define JIT_MAX_BLOCK_INSNS  64

/* Block entry in the hash table */
typedef struct {
    uint32_t pc;            /* Guest PC (tag for collision detection) */
    void    *code;          /* Pointer into code cache (NULL = empty) */
    uint32_t exec_count;    /* Hot counter / execution count */
    uint16_t guest_insns;   /* Number of guest instructions in block */
    uint16_t flags;         /* Reserved */
} jit_block_t;

/* JIT statistics */
typedef struct {
    uint64_t blocks_compiled;
    uint64_t blocks_executed;
    uint64_t insns_jitted;      /* Guest insns executed via JIT */
    uint64_t insns_interp;      /* Guest insns executed via interpreter */
    uint64_t cache_flushes;
    uint64_t fallbacks;         /* Instructions that fell back to interpreter */
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

/* Statistics */
const jit_stats_t *jit_get_stats(const jit_state_t *jit);
void               jit_print_stats(const jit_state_t *jit);

/* Differential verification mode: compare each JIT block against interpreter */
void               jit_set_verify(jit_state_t *jit, bool enable);

#endif /* JIT_H */
