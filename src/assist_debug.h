/* Target-described per-core processor debug recorder. */
#ifndef FLEXE_ASSIST_DEBUG_H
#define FLEXE_ASSIST_DEBUG_H

#include <stdbool.h>
#include <stdint.h>

#include "memory.h"

typedef struct xtensa_cpu xtensa_cpu_t;
typedef struct flexe_assist_debug flexe_assist_debug_t;

/* A normalized view of the CPU state retained by the recorder. While both
 * controls are enabled, PC and SP are live; clearing either control freezes
 * the most recent values, matching the hardware crash-record workflow. */
typedef struct {
    bool pdebug_enabled;
    bool recording;
    uint32_t pc;
    uint32_t sp;
} flexe_assist_debug_core_state_t;

/* Unknown ASSIST_DEBUG offsets remain visible through the fallback handlers. */
flexe_assist_debug_t *flexe_assist_debug_create(
    xtensa_mem_t *mem, mmio_read_fn fallback_read,
    mmio_write_fn fallback_write, void *fallback_ctx);
void flexe_assist_debug_destroy(flexe_assist_debug_t *debug);

void flexe_assist_debug_attach_cpus(
    flexe_assist_debug_t *debug, xtensa_cpu_t *cpu0, xtensa_cpu_t *cpu1);
bool flexe_assist_debug_core_state(
    const flexe_assist_debug_t *debug, unsigned core,
    flexe_assist_debug_core_state_t *state);

#endif /* FLEXE_ASSIST_DEBUG_H */
