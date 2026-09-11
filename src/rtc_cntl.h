/* Descriptor-driven ESP32-family always-on RTC controller. */
#ifndef FLEXE_RTC_CNTL_H
#define FLEXE_RTC_CNTL_H

#include "memory.h"

typedef struct xtensa_cpu xtensa_cpu_t;
typedef struct flexe_rtc_cntl flexe_rtc_cntl_t;

flexe_rtc_cntl_t *flexe_rtc_cntl_create(
    xtensa_mem_t *mem, mmio_read_fn fallback_read,
    mmio_write_fn fallback_write, void *fallback_ctx);
void flexe_rtc_cntl_destroy(flexe_rtc_cntl_t *rtc);

/* Populate state that a second-stage bootloader normally hands to an
 * application loaded directly by Flexe. */
void flexe_rtc_cntl_application_handoff(flexe_rtc_cntl_t *rtc);

/* Attach the execution engines which advance the shared RTC timeline. */
void flexe_rtc_cntl_attach_cpus(flexe_rtc_cntl_t *rtc,
                                xtensa_cpu_t *cpu0,
                                xtensa_cpu_t *cpu1);

#endif /* FLEXE_RTC_CNTL_H */
