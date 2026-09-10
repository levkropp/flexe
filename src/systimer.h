/* Descriptor-driven ESP32-family system timer. */
#ifndef FLEXE_SYSTIMER_H
#define FLEXE_SYSTIMER_H

#include "memory.h"

typedef struct xtensa_cpu xtensa_cpu_t;
typedef struct flexe_systimer flexe_systimer_t;

typedef void (*flexe_systimer_state_fn)(void *ctx);
typedef void (*flexe_systimer_irq_fn)(void *ctx, unsigned alarm, bool level);

/* Create the V1 register block described by mem's target. Unknown offsets
 * remain visible through the supplied fallback instead of becoming a silent
 * register file. */
flexe_systimer_t *flexe_systimer_create(
    xtensa_mem_t *mem, mmio_read_fn fallback_read,
    mmio_write_fn fallback_write, void *fallback_ctx,
    flexe_systimer_state_fn state_changed, void *state_ctx,
    flexe_systimer_irq_fn irq_changed, void *irq_ctx);
void flexe_systimer_destroy(flexe_systimer_t *systimer);

/* Attach the execution engines which advance the shared virtual timeline. */
void flexe_systimer_attach_cpus(flexe_systimer_t *systimer,
                                xtensa_cpu_t *cpu0,
                                xtensa_cpu_t *cpu1);

/* Event-scheduler integration. next_event returns a deadline in the calling
 * CPU's CCOUNT frame; eval advances counter/alarm state at that boundary. */
uint32_t flexe_systimer_next_event(flexe_systimer_t *systimer,
                                   xtensa_cpu_t *cpu);
void flexe_systimer_eval(flexe_systimer_t *systimer);

#endif /* FLEXE_SYSTIMER_H */
