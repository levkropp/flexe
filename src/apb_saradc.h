/* Target-described APB SAR ADC digital controller and ADC2 arbiter. */
#ifndef FLEXE_APB_SARADC_H
#define FLEXE_APB_SARADC_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "memory.h"

typedef struct flexe_apb_saradc flexe_apb_saradc_t;
typedef struct flexe_gdma flexe_gdma_t;
typedef uint16_t (*flexe_apb_saradc_sample_fn)(
    void *ctx, unsigned unit, unsigned channel);

flexe_apb_saradc_t *flexe_apb_saradc_create(
    xtensa_mem_t *mem, flexe_gdma_t *gdma,
    flexe_apb_saradc_sample_fn sample, void *sample_ctx,
    mmio_read_fn fallback_read, mmio_write_fn fallback_write,
    void *fallback_ctx);
void flexe_apb_saradc_destroy(flexe_apb_saradc_t *adc);
void flexe_apb_saradc_set_system_state(
    flexe_apb_saradc_t *adc, bool clock_enabled, bool reset_asserted);

/* Complete one configured conversion frame into the active trigger-routed
 * GDMA RX descriptor. Values are sampled through the board callback. */
size_t flexe_apb_saradc_inject_frame(flexe_apb_saradc_t *adc);
bool flexe_apb_saradc_stream_active(const flexe_apb_saradc_t *adc);
uint64_t flexe_apb_saradc_frame_count(const flexe_apb_saradc_t *adc);
uint64_t flexe_apb_saradc_sample_count(const flexe_apb_saradc_t *adc);

/* With no modeled competing requester, unforced arbitration grants RTC.
 * Forced grant is honored only when RTC is the sole selected controller. */
bool flexe_apb_saradc_rtc_granted(const flexe_apb_saradc_t *adc);

#endif /* FLEXE_APB_SARADC_H */
