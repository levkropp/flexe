/* Descriptor-driven ESP32-family RTC scratch registers. */
#ifndef FLEXE_RTC_STORAGE_H
#define FLEXE_RTC_STORAGE_H

#include "memory.h"

typedef struct flexe_rtc_storage flexe_rtc_storage_t;

flexe_rtc_storage_t *flexe_rtc_storage_create(
    xtensa_mem_t *mem, mmio_read_fn fallback_read,
    mmio_write_fn fallback_write, void *fallback_ctx);
void flexe_rtc_storage_destroy(flexe_rtc_storage_t *storage);

/* Populate values a second-stage bootloader establishes before transferring
 * control directly to an application image. */
void flexe_rtc_storage_application_handoff(flexe_rtc_storage_t *storage);

#endif /* FLEXE_RTC_STORAGE_H */
