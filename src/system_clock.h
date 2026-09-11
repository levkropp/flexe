/* Descriptor-driven ESP32-family CPU/system-clock selection registers. */
#ifndef FLEXE_SYSTEM_CLOCK_H
#define FLEXE_SYSTEM_CLOCK_H

#include "memory.h"

typedef struct flexe_system_clock flexe_system_clock_t;

/* Create the V1 register block described by mem's target. Unrecognized
 * offsets delegate to the supplied owner, allowing clock selection,
 * secondary-core control, and software interrupts to share one SYSTEM page. */
flexe_system_clock_t *flexe_system_clock_create(
    xtensa_mem_t *mem, mmio_read_fn fallback_read,
    mmio_write_fn fallback_write, void *fallback_ctx);
void flexe_system_clock_destroy(flexe_system_clock_t *clock);

#endif /* FLEXE_SYSTEM_CLOCK_H */
