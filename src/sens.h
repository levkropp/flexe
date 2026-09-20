/* Descriptor-driven RTC-domain sensor controller. */
#ifndef FLEXE_SENS_H
#define FLEXE_SENS_H

#include <stdbool.h>

#include "memory.h"

typedef struct flexe_regi2c flexe_regi2c_t;
typedef struct flexe_apb_saradc flexe_apb_saradc_t;
typedef struct flexe_touch_v2 flexe_touch_v2_t;

typedef struct flexe_sens flexe_sens_t;

/* A completed conversion raises the target-described RTC interrupt source.
 * The RTC controller owns enable, status, and write-one-to-clear behavior. */
typedef void (*flexe_sens_conversion_fn)(void *ctx);

/* Resolved state of the complete SENS peripheral clock/reset fabric.  The
 * register encoding is target-specific; consumers receive named domains so
 * they do not need to decode S3 bit positions. */
typedef struct {
    bool io_mux_clock_enabled;
    bool adc_clock_enabled;
    bool temperature_clock_enabled;
    bool rtc_i2c_clock_enabled;
    bool adc_reset_asserted;
    bool temperature_reset_asserted;
    bool rtc_i2c_reset_asserted;
    bool coprocessor_reset_asserted;
} flexe_sens_peripheral_state_t;
typedef void (*flexe_sens_peripheral_fn)(
    void *ctx, const flexe_sens_peripheral_state_t *state);

flexe_sens_t *flexe_sens_create(
    xtensa_mem_t *mem, mmio_read_fn fallback_read,
    mmio_write_fn fallback_write, void *fallback_ctx,
    flexe_sens_conversion_fn conversion_done, void *conversion_ctx);
void flexe_sens_destroy(flexe_sens_t *sens);
void flexe_sens_attach_regi2c(flexe_sens_t *sens,
                              const flexe_regi2c_t *regi2c);
void flexe_sens_attach_apb_saradc(flexe_sens_t *sens,
                                  const flexe_apb_saradc_t *apb_saradc);
void flexe_sens_attach_touch_v2(flexe_sens_t *sens,
                                flexe_touch_v2_t *touch);

bool flexe_sens_peripheral_state(
    const flexe_sens_t *sens, flexe_sens_peripheral_state_t *state);
void flexe_sens_set_peripheral_listener(
    flexe_sens_t *sens, flexe_sens_peripheral_fn fn, void *ctx);

/* These are public so another device sharing the same 4-KiB MMIO page can
 * retain SENS as its fallback handler. */
uint32_t flexe_sens_mmio_read(void *ctx, uint32_t addr);
void flexe_sens_mmio_write(void *ctx, uint32_t addr, uint32_t value);

/* Drive the physical-side temperature ADC code. A conversion latches the
 * current value; changing the input does not rewrite an already completed
 * sample. Values wider than the target's output field are saturated. */
void flexe_sens_set_temperature_raw(flexe_sens_t *sens, uint16_t raw);
uint16_t flexe_sens_temperature_raw(const flexe_sens_t *sens);

/* Host-side ADC stimulus: zero-based unit/channel; a completed conversion
 * retains its sample until software pulses START again. */
void flexe_sens_set_adc_raw(flexe_sens_t *sens, unsigned unit,
                            unsigned channel, uint16_t raw);

#endif /* FLEXE_SENS_H */
