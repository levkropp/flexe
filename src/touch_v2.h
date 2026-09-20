/* Descriptor-driven ESP32-S2/S3 capacitive-touch v2 controller. */
#ifndef FLEXE_TOUCH_V2_H
#define FLEXE_TOUCH_V2_H

#include <stdbool.h>
#include <stdint.h>

#include "memory.h"

typedef struct flexe_touch_v2 flexe_touch_v2_t;

/* Touch events are raw RTC interrupt bits. RTC_CNTL owns enable, masked
 * status, write-one-to-clear, and CPU interrupt routing. */
typedef void (*flexe_touch_v2_irq_fn)(void *ctx, uint32_t mask);

flexe_touch_v2_t *flexe_touch_v2_create(
    xtensa_mem_t *mem, flexe_touch_v2_irq_fn irq, void *irq_ctx);
void flexe_touch_v2_destroy(flexe_touch_v2_t *touch);

/* RTC_CNTL publishes target-described configuration words to the touch
 * engine. SENS delegates only the offsets claimed by this IP block. */
void flexe_touch_v2_rtc_config_changed(
    flexe_touch_v2_t *touch, uint16_t offset, uint32_t value);
bool flexe_touch_v2_sens_read(
    flexe_touch_v2_t *touch, uint32_t offset, uint32_t *value);
bool flexe_touch_v2_sens_write(
    flexe_touch_v2_t *touch, uint32_t offset, uint32_t value);

/* Board-side electrode stimulus. Counts rise with capacitance on touch-v2
 * silicon. Changing a sample completes a functional scan while timer mode is
 * active; fast mode does not claim calibrated charge/discharge timing. */
void flexe_touch_v2_set_raw(
    flexe_touch_v2_t *touch, unsigned channel, uint32_t value);
uint64_t flexe_touch_v2_scan_count(const flexe_touch_v2_t *touch);
uint32_t flexe_touch_v2_active_mask(const flexe_touch_v2_t *touch);
bool flexe_touch_v2_running(const flexe_touch_v2_t *touch);

#endif /* FLEXE_TOUCH_V2_H */
