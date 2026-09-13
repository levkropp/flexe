/* Target-described RTC GPIO bank and S3-generation pad-owner mux. */
#ifndef FLEXE_RTC_IO_H
#define FLEXE_RTC_IO_H

#include "gpio.h"

typedef struct flexe_rtc_io flexe_rtc_io_t;

typedef struct {
    uint32_t mask;
    uint32_t pad[FLEXE_TARGET_RTC_IO_PIN_MAX];
} flexe_rtc_io_pad_hold_t;

/* RTCIO shares a 4 KiB MMIO page with RTC_CNTL and SENS. The exported
 * dispatchers allow those models to form an explicit fallback chain. */
flexe_rtc_io_t *flexe_rtc_io_create(
    xtensa_mem_t *mem, flexe_gpio_t *gpio,
    mmio_read_fn fallback_read, mmio_write_fn fallback_write,
    void *fallback_ctx);
void flexe_rtc_io_destroy(flexe_rtc_io_t *rtc_io);
/* RTC_CNTL owns the hold bits. RTCIO freezes effective input/owner mux while
 * its register latches remain writable; GPIO separately freezes the output. */
void flexe_rtc_io_set_pad_hold(flexe_rtc_io_t *rtc_io,
                                uint64_t held_pins);
void flexe_rtc_io_pad_hold_snapshot(const flexe_rtc_io_t *rtc_io,
                                    flexe_rtc_io_pad_hold_t *out);
void flexe_rtc_io_pad_hold_restore(flexe_rtc_io_t *rtc_io,
                                   const flexe_rtc_io_pad_hold_t *in);
uint32_t flexe_rtc_io_mmio_read(void *ctx, uint32_t addr);
void flexe_rtc_io_mmio_write(void *ctx, uint32_t addr, uint32_t value);

#endif /* FLEXE_RTC_IO_H */
