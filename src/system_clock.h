/* Descriptor-driven ESP32-family CPU/system-clock selection registers. */
#ifndef FLEXE_SYSTEM_CLOCK_H
#define FLEXE_SYSTEM_CLOCK_H

#include <stdbool.h>

#include "memory.h"
#include "target_types.h"

typedef struct flexe_system_clock flexe_system_clock_t;
typedef void (*flexe_system_clock_gate_fn)(
    void *ctx, flexe_system_device_t device, unsigned instance,
    bool clock_enabled, bool reset_asserted);

enum {
    FLEXE_SYSTEM_LOW_POWER_SOURCE_RTC_SLOW = 1u << 0,
    FLEXE_SYSTEM_LOW_POWER_SOURCE_INTERNAL = 1u << 1,
    FLEXE_SYSTEM_LOW_POWER_SOURCE_XTAL = 1u << 2,
    FLEXE_SYSTEM_LOW_POWER_SOURCE_XTAL32K = 1u << 3,
};

/* Resolved policy/state rather than raw target register encoding. Multiple
 * source bits remain visible so malformed guest configuration is not hidden
 * behind an invented source priority. The fractional divider is represented
 * exactly as the hardware's integer + B/A tuple. */
typedef struct {
    bool memory_power_down_allowed;
    bool rtc_clock_enabled;
    uint8_t selected_sources;
    uint32_t divider_integer;
    uint32_t divider_a;
    uint32_t divider_b;
} flexe_system_low_power_state_t;
typedef void (*flexe_system_low_power_fn)(
    void *ctx, const flexe_system_low_power_state_t *state);

/* Complete target peripheral clock/reset fabric. Bank zero occupies bits
 * 0..31 and bank one bits 32..63; valid_mask distinguishes implemented target
 * domains from reserved positions. Device-specific effects remain attached
 * through the gate mappings in the target descriptor. */
typedef struct {
    uint64_t clock_enabled;
    uint64_t reset_asserted;
    uint64_t valid_mask;
} flexe_system_peripheral_state_t;
typedef void (*flexe_system_peripheral_fn)(
    void *ctx, const flexe_system_peripheral_state_t *state);

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

/* Query or subscribe to target-described light-sleep memory policy and the
 * radio low-power clock. Installing a listener publishes current state
 * immediately; subsequent notifications are synchronous with MMIO writes. */
bool flexe_system_clock_low_power_state(
    const flexe_system_clock_t *clock,
    flexe_system_low_power_state_t *state);
void flexe_system_clock_set_low_power_listener(
    flexe_system_clock_t *clock, flexe_system_low_power_fn fn, void *ctx);

bool flexe_system_clock_peripheral_state(
    const flexe_system_clock_t *clock,
    flexe_system_peripheral_state_t *state);
void flexe_system_clock_set_peripheral_listener(
    flexe_system_clock_t *clock, flexe_system_peripheral_fn fn, void *ctx);

#endif /* FLEXE_SYSTEM_CLOCK_H */
