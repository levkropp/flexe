/*
 * aot.h — runtime loader for ahead-of-time recompiled Xtensa firmware.
 *
 * tools/aot_recompile.py translates an ESP-IDF .elf into a native shared
 * library exposing one C function per Xtensa function, named
 * `aot_func_<hex_pc>`. At runtime flexe `dlopen`s that dylib, walks its
 * symbol table to build a PC -> function-pointer hash map, and the
 * interpreter's hot path consults the map at each step. Hits jump
 * straight into native C; misses fall through to the interpreter.
 *
 * This is the highest-leverage performance path for ESP32 emulation,
 * because the C compiler's whole-function optimizer beats anything a
 * JIT can do at runtime.
 */
#ifndef FLEXE_AOT_H
#define FLEXE_AOT_H

#include "xtensa.h"
#include <stdint.h>

/* Returns number of guest instructions executed in the block.
 * Updates cpu->pc, cpu->ar[], cpu->ccount, etc. in place. */
typedef int (*aot_func_t)(xtensa_cpu_t *cpu);

typedef struct aot_state aot_state_t;

/* Load a dylib and walk its `aot_func_<hex>` symbols into a hash table.
 * Returns NULL on failure (file missing, no symbols, dlopen error). */
aot_state_t *aot_load(const char *dylib_path);
void         aot_destroy(aot_state_t *aot);

/* O(1) lookup. Returns NULL if no AOT function exists for this PC. */
aot_func_t   aot_lookup(const aot_state_t *aot, uint32_t pc);

/* Statistics for the benchmark report. */
typedef struct {
    uint64_t aot_blocks_executed;
    uint64_t aot_insns_executed;
    uint64_t aot_lookups;
    uint32_t functions_loaded;
} aot_stats_t;

const aot_stats_t *aot_get_stats(const aot_state_t *aot);

/* Returns the AOT entry-point bitmap: 1 bit per (PC>>2) modulo
 * HOOK_BITMAP_BITS. The hot path in xtensa_step_impl tests this
 * bitmap before calling aot_lookup — the test is a single AND + bit
 * shift, vs aot_lookup's full hash probe. Since only ~0.2% of PCs are
 * AOT entry points, the bitmap filters out ~99.8% of would-be
 * lookups. Returns NULL if `a` is NULL. */
const uint64_t *aot_get_bitmap(const aot_state_t *a);

#endif /* FLEXE_AOT_H */
