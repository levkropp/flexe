/* Target-described APB SAR ADC2 arbiter. */
#ifndef FLEXE_APB_SARADC_H
#define FLEXE_APB_SARADC_H

#include <stdbool.h>
#include "memory.h"

typedef struct flexe_apb_saradc flexe_apb_saradc_t;

flexe_apb_saradc_t *flexe_apb_saradc_create(
    xtensa_mem_t *mem, mmio_read_fn fallback_read,
    mmio_write_fn fallback_write, void *fallback_ctx);
void flexe_apb_saradc_destroy(flexe_apb_saradc_t *adc);

/* With no modeled competing requester, unforced arbitration grants RTC.
 * Forced grant is honored only when RTC is the sole selected controller. */
bool flexe_apb_saradc_rtc_granted(const flexe_apb_saradc_t *adc);

#endif /* FLEXE_APB_SARADC_H */
