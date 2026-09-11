/* Descriptor-driven ESP32-family always-on RTC controller. */
#ifndef FLEXE_RTC_CNTL_H
#define FLEXE_RTC_CNTL_H

#include "memory.h"

typedef struct xtensa_cpu xtensa_cpu_t;
typedef struct flexe_rtc_cntl flexe_rtc_cntl_t;
typedef void (*flexe_rtc_cntl_irq_fn)(void *ctx, bool level);

flexe_rtc_cntl_t *flexe_rtc_cntl_create(
    xtensa_mem_t *mem, mmio_read_fn fallback_read,
    mmio_write_fn fallback_write, void *fallback_ctx,
    flexe_rtc_cntl_irq_fn irq_changed, void *irq_ctx);
void flexe_rtc_cntl_destroy(flexe_rtc_cntl_t *rtc);

/* Populate state that a second-stage bootloader normally hands to an
 * application loaded directly by Flexe. */
void flexe_rtc_cntl_application_handoff(flexe_rtc_cntl_t *rtc);

/* Attach the execution engines which advance the shared RTC timeline. */
void flexe_rtc_cntl_attach_cpus(flexe_rtc_cntl_t *rtc,
                                xtensa_cpu_t *cpu0,
                                xtensa_cpu_t *cpu1);

/* Publish or withdraw one or more target-described RTC event conditions.
 * Raw state latches independently of the enable mask; the aggregate output
 * is level-sensitive and changes only when RAW & ENA crosses zero. */
void flexe_rtc_cntl_set_interrupts(flexe_rtc_cntl_t *rtc,
                                   uint32_t mask, bool asserted);

#endif /* FLEXE_RTC_CNTL_H */
