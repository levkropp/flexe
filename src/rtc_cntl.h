/* Descriptor-driven ESP32-family always-on RTC controller. */
#ifndef FLEXE_RTC_CNTL_H
#define FLEXE_RTC_CNTL_H

#include "memory.h"

typedef struct xtensa_cpu xtensa_cpu_t;
typedef struct flexe_rtc_cntl flexe_rtc_cntl_t;
typedef enum {
    FLEXE_RTC_CNTL_WDT_RESET_CPU = 2,
    FLEXE_RTC_CNTL_WDT_RESET_SYSTEM = 3,
    FLEXE_RTC_CNTL_WDT_RESET_RTC = 4,
} flexe_rtc_cntl_wdt_action_t;
typedef void (*flexe_rtc_cntl_state_fn)(void *ctx);
typedef void (*flexe_rtc_cntl_irq_fn)(void *ctx, bool level);
typedef void (*flexe_rtc_cntl_reset_fn)(
    void *ctx, flexe_rtc_cntl_wdt_action_t action);
typedef void (*flexe_rtc_cntl_pad_hold_fn)(void *ctx, uint64_t gpio_mask);

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

#endif /* FLEXE_RTC_CNTL_H */
