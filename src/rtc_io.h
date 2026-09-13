/* Target-described RTC GPIO bank and S3-generation pad-owner mux. */
#ifndef FLEXE_RTC_IO_H
#define FLEXE_RTC_IO_H

#include "gpio.h"

typedef struct flexe_rtc_io flexe_rtc_io_t;

/* RTCIO shares a 4 KiB MMIO page with RTC_CNTL and SENS. The exported
 * dispatchers allow those models to form an explicit fallback chain. */
flexe_rtc_io_t *flexe_rtc_io_create(
    xtensa_mem_t *mem, flexe_gpio_t *gpio,
    mmio_read_fn fallback_read, mmio_write_fn fallback_write,
    void *fallback_ctx);
void flexe_rtc_io_destroy(flexe_rtc_io_t *rtc_io);
uint32_t flexe_rtc_io_mmio_read(void *ctx, uint32_t addr);
void flexe_rtc_io_mmio_write(void *ctx, uint32_t addr, uint32_t value);

#endif /* FLEXE_RTC_IO_H */
