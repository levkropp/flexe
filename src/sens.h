/* Descriptor-driven RTC-domain sensor controller. */
#ifndef FLEXE_SENS_H
#define FLEXE_SENS_H

#include "memory.h"

typedef struct flexe_regi2c flexe_regi2c_t;

typedef struct flexe_sens flexe_sens_t;

/* A completed conversion raises the target-described RTC interrupt source.
 * The RTC controller owns enable, status, and write-one-to-clear behavior. */
typedef void (*flexe_sens_conversion_fn)(void *ctx);

flexe_sens_t *flexe_sens_create(
    xtensa_mem_t *mem, mmio_read_fn fallback_read,
    mmio_write_fn fallback_write, void *fallback_ctx,
    flexe_sens_conversion_fn conversion_done, void *conversion_ctx);
void flexe_sens_destroy(flexe_sens_t *sens);
void flexe_sens_attach_regi2c(flexe_sens_t *sens,
                              const flexe_regi2c_t *regi2c);

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
