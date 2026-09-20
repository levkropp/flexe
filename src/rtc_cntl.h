/* Descriptor-driven ESP32-family always-on RTC controller. */
#ifndef FLEXE_RTC_CNTL_H
#define FLEXE_RTC_CNTL_H

#include <stdbool.h>

#include "memory.h"
#include "target_types.h"

typedef struct xtensa_cpu xtensa_cpu_t;
typedef struct flexe_rtc_cntl flexe_rtc_cntl_t;
typedef enum {
    FLEXE_RTC_CNTL_WDT_RESET_CPU = 2,
    FLEXE_RTC_CNTL_WDT_RESET_SYSTEM = 3,
    FLEXE_RTC_CNTL_WDT_RESET_RTC = 4,
    FLEXE_RTC_CNTL_SW_RESET_CPU = 5,
    FLEXE_RTC_CNTL_SW_RESET_SYSTEM = 6,
    FLEXE_RTC_CNTL_SW_RESET_CPU1 = 7,
} flexe_rtc_cntl_reset_action_t;
typedef void (*flexe_rtc_cntl_state_fn)(void *ctx);
typedef void (*flexe_rtc_cntl_irq_fn)(void *ctx, bool level);
typedef void (*flexe_rtc_cntl_reset_fn)(
    void *ctx, flexe_rtc_cntl_reset_action_t action);
typedef void (*flexe_rtc_cntl_pad_hold_fn)(void *ctx, uint64_t gpio_mask);
typedef void (*flexe_rtc_cntl_domain_state_fn)(
    void *ctx, uint32_t powered, uint32_t isolated);
typedef void (*flexe_rtc_cntl_supply_state_fn)(void *ctx, uint32_t powered);
typedef void (*flexe_rtc_cntl_config_fn)(void *ctx, uint16_t offset,
                                         uint32_t value);

/* Normalized state for the RTC controller's analog, oscillator, isolation,
 * and reset controls. Array-backed bitsets correspond to the same-numbered
 * entries in the target descriptor, so consumers never decode raw register
 * positions. Functional mode resolves absent force pairs to powered,
 * non-isolated, and out of reset; force-down/set wins a conflict. */
typedef struct {
    uint32_t powered_options_supplies;
    uint32_t isolated_options_domains;
    uint32_t reset_options_domains;
    uint32_t enabled_analog_controls;
    uint8_t xtal_enable_wait;
    bool analog_reset_por_powered;
    bool digital_pad_isolated;
    bool digital_pad_autohold_enabled;
    bool digital_isolation_enabled;
} flexe_rtc_cntl_control_state_t;
typedef void (*flexe_rtc_cntl_control_state_fn)(
    void *ctx, const flexe_rtc_cntl_control_state_t *state);

/* The RTC slow counter and STORE registers remain powered through an S3
 * software reset/deep-sleep wake. Volatile WDT, alarm, and interrupt state
 * are deliberately not part of this snapshot. */
typedef struct {
    uint64_t counter;
    uint64_t tick_denominator;
    uint64_t tick_remainder;
    uint32_t store[FLEXE_TARGET_RTC_STORE_MAX];
    uint32_t ext1_status;
} flexe_rtc_cntl_retained_t;

flexe_rtc_cntl_t *flexe_rtc_cntl_create(
    xtensa_mem_t *mem, mmio_read_fn fallback_read,
    mmio_write_fn fallback_write, void *fallback_ctx,
    flexe_rtc_cntl_state_fn state_changed, void *state_ctx,
    flexe_rtc_cntl_irq_fn irq_changed, void *irq_ctx,
    flexe_rtc_cntl_reset_fn reset_requested, void *reset_ctx);
void flexe_rtc_cntl_destroy(flexe_rtc_cntl_t *rtc);

/* The RTC register owns the hold bits; GPIO owns the physical output state.
 * The listener composes the two without letting either device claim the
 * other's MMIO range. */
void flexe_rtc_cntl_set_pad_hold_listener(flexe_rtc_cntl_t *rtc,
                                          flexe_rtc_cntl_pad_hold_fn fn,
                                          void *ctx);

/* Populate state that a second-stage bootloader normally hands to an
 * application loaded directly by Flexe. */
void flexe_rtc_cntl_application_handoff(flexe_rtc_cntl_t *rtc);

/* Attach the execution engines which advance the shared RTC timeline. */
void flexe_rtc_cntl_attach_cpus(flexe_rtc_cntl_t *rtc,
                                xtensa_cpu_t *cpu0,
                                xtensa_cpu_t *cpu1);

/* Event-scheduler integration. The returned deadline is in the calling
 * core's CCOUNT frame and covers the next enabled RTC-watchdog stage. */
uint32_t flexe_rtc_cntl_next_event(flexe_rtc_cntl_t *rtc,
                                   xtensa_cpu_t *cpu);
void flexe_rtc_cntl_eval(flexe_rtc_cntl_t *rtc);

/* Publish or withdraw one or more target-described RTC event conditions.
 * Raw state latches independently of the enable mask; the aggregate output
 * is level-sensitive and changes only when RAW & ENA crosses zero. */
void flexe_rtc_cntl_set_interrupts(flexe_rtc_cntl_t *rtc,
                                   uint32_t mask, bool asserted);

/* RTC_CNTL_ANA_CONF powers the internal SAR analog-register I2C slave.
 * A detached/missing RTC cannot claim that power domain is available. */
bool flexe_rtc_cntl_sar_i2c_powered(const flexe_rtc_cntl_t *rtc);

/* Read or subscribe to target-described RTC control state. Installing a
 * listener publishes the current state immediately and subsequent callbacks
 * occur only when normalized state changes. */
void flexe_rtc_cntl_control_state(
    const flexe_rtc_cntl_t *rtc, flexe_rtc_cntl_control_state_t *out);
void flexe_rtc_cntl_set_control_listener(
    flexe_rtc_cntl_t *rtc, flexe_rtc_cntl_control_state_fn fn, void *ctx);

/* Target-described retained configuration can feed peripheral state machines
 * whose controls physically reside in RTC_CNTL. Installing a listener
 * publishes every current config word before subsequent changed values. */
void flexe_rtc_cntl_set_config_listener(
    flexe_rtc_cntl_t *rtc, flexe_rtc_cntl_config_fn fn, void *ctx);

/* Nominal frequency selected by the target's RTC_FAST_CLK mux. Functional
 * mode exposes the selected source frequency without inventing oscillator
 * drift or analog settling time. */
uint32_t flexe_rtc_cntl_fast_clock_hz(const flexe_rtc_cntl_t *rtc);

/* Logical state of the target-described DIG_PWC/DIG_ISO domains. Bit N
 * corresponds to digital_domain[N] in the target descriptor. These queries
 * resolve force pairs and automatic sleep policy; they do not expose raw
 * register encoding to consuming devices. */
uint32_t flexe_rtc_cntl_powered_digital_domains(
    const flexe_rtc_cntl_t *rtc);
uint32_t flexe_rtc_cntl_isolated_digital_domains(
    const flexe_rtc_cntl_t *rtc);
/* Domain consumers receive every transition synchronously, including a full
 * power cycle with no intervening access to the consumer. This notification
 * is separate from the RTC timer scheduler and cannot manufacture a CPU
 * wakeup or timeslice boundary. Installing a listener publishes the current
 * state immediately. */
void flexe_rtc_cntl_set_digital_domain_listener(
    flexe_rtc_cntl_t *rtc, flexe_rtc_cntl_domain_state_fn fn, void *ctx);

/* Logical RTC-peripheral/RTC-memory state. Bit N corresponds to
 * rtc_power_domain[N] in the target descriptor. RTC memory follow-CPU fields
 * resolve against the target-described digital domain instead of assuming a
 * chip-specific index in the device model. */
uint32_t flexe_rtc_cntl_powered_rtc_domains(const flexe_rtc_cntl_t *rtc);
uint32_t flexe_rtc_cntl_isolated_rtc_domains(const flexe_rtc_cntl_t *rtc);
void flexe_rtc_cntl_set_rtc_domain_listener(
    flexe_rtc_cntl_t *rtc, flexe_rtc_cntl_domain_state_fn fn, void *ctx);

/* Logical state of target-described RTC regulator force pairs. Bit N
 * corresponds to regulator_supply[N]. Force-down wins conflicts; absent
 * forces mean powered in functional mode because analog sequencing and
 * voltage ramp timing are intentionally outside this model. */
uint32_t flexe_rtc_cntl_powered_supplies(const flexe_rtc_cntl_t *rtc);
void flexe_rtc_cntl_set_supply_listener(
    flexe_rtc_cntl_t *rtc, flexe_rtc_cntl_supply_state_fn fn, void *ctx);

/* S3 RTC USB mux selection. The default virtual board has its internal PHY
 * attached to USB Serial/JTAG unless software routes it to USB OTG. */
bool flexe_rtc_cntl_usb_serial_jtag_internal_phy(
    const flexe_rtc_cntl_t *rtc);

/* Software stall pauses instruction retirement without erasing CPU state. */
bool flexe_rtc_cntl_cpu_stalled(const flexe_rtc_cntl_t *rtc, unsigned core);

void flexe_rtc_cntl_retained_snapshot(
    flexe_rtc_cntl_t *rtc, flexe_rtc_cntl_retained_t *out);
void flexe_rtc_cntl_retained_restore(
    flexe_rtc_cntl_t *rtc, const flexe_rtc_cntl_retained_t *snapshot);

/* Supported timer/EXT0/EXT1 sleep is consumed by the session's virtual
 * clock/reset path. Unsupported wake sources never produce a synthetic wake. */
bool flexe_rtc_cntl_take_sleep_request(flexe_rtc_cntl_t *rtc,
                                       bool *deep, uint64_t *timeout_us);
bool flexe_rtc_cntl_has_gpio_wake(const flexe_rtc_cntl_t *rtc);
uint32_t flexe_rtc_cntl_poll_gpio_wake(flexe_rtc_cntl_t *rtc,
                                       int ext0_level,
                                       uint32_t ext1_high_mask);
void flexe_rtc_cntl_finish_wake(flexe_rtc_cntl_t *rtc, uint32_t cause);
void flexe_rtc_cntl_set_wake_state(flexe_rtc_cntl_t *rtc,
                                   uint32_t cause, uint32_t reset_cause);

#endif /* FLEXE_RTC_CNTL_H */
