/* Descriptor-driven ESP32-family CPU/system-clock selection registers. */
#ifndef FLEXE_SYSTEM_CLOCK_H
#define FLEXE_SYSTEM_CLOCK_H

#include "memory.h"

typedef struct flexe_system_clock flexe_system_clock_t;
typedef void (*flexe_system_clock_gate_fn)(
    void *ctx, flexe_system_device_t device, unsigned instance,
    bool clock_enabled, bool reset_asserted);

/* Create the V1 register block described by mem's target. Unrecognized
 * offsets delegate to the supplied owner, allowing clock selection,
 * secondary-core control, and software interrupts to share one SYSTEM page.
 * Gate callbacks publish semantic device state only for descriptor mappings;
 * changing other writable control fields remains explicitly unsupported. */
flexe_system_clock_t *flexe_system_clock_create(
    xtensa_mem_t *mem, mmio_read_fn fallback_read,
    mmio_write_fn fallback_write, void *fallback_ctx,
    flexe_system_clock_gate_fn gate_changed, void *gate_ctx);
void flexe_system_clock_destroy(flexe_system_clock_t *clock);

/* Query or republish a target-described peripheral gate. publish_gates is
 * used after all device models have been constructed to apply reset state. */
bool flexe_system_clock_gate_state(
    const flexe_system_clock_t *clock, flexe_system_device_t device,
    unsigned instance, bool *clock_enabled, bool *reset_asserted);
void flexe_system_clock_publish_gates(flexe_system_clock_t *clock);

#endif /* FLEXE_SYSTEM_CLOCK_H */
