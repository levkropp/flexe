/* No-op JIT stubs for non-x86-64 targets.
 * Compiled in place of jit.c when the JIT backend is unavailable. */
#include "jit.h"
#include <stddef.h>

jit_state_t *jit_init(void) { return NULL; }
void         jit_destroy(jit_state_t *jit) { (void)jit; }
void         jit_flush(jit_state_t *jit) { (void)jit; }

jit_block_fn jit_get_block(jit_state_t *jit, xtensa_cpu_t *cpu, uint32_t pc) {
    (void)jit; (void)cpu; (void)pc;
    return NULL;
}

int jit_run(jit_state_t *jit, xtensa_cpu_t *cpu, int max_cycles) {
    (void)jit; (void)cpu; (void)max_cycles;
    return 0;
}

void jit_install_hook(jit_state_t *jit, xtensa_cpu_t *cpu) { (void)jit; (void)cpu; }

static const jit_stats_t g_empty_stats;
const jit_stats_t *jit_get_stats(const jit_state_t *jit) { (void)jit; return &g_empty_stats; }
void               jit_print_stats(const jit_state_t *jit) { (void)jit; }
void               jit_set_verify(jit_state_t *jit, bool enable) { (void)jit; (void)enable; }
