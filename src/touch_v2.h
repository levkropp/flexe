/* Descriptor-driven ESP32-S2/S3 capacitive-touch v2 controller. */
#ifndef FLEXE_TOUCH_V2_H
#define FLEXE_TOUCH_V2_H

#include <stdbool.h>
#include <stdint.h>

#include "memory.h"

#define FLEXE_TARGET_TOUCH_CHANNEL_MAX 15u
/* Registry tag is stable; descriptor layout changes stay local to this header. */
#define FLEXE_TARGET_EXTENSION_TOUCH_V2 UINT32_C(0x54435632) /* "TCV2" */

/* ESP32-S2/S3-generation capacitive-touch controller. The digital control
 * registers live in RTC_CNTL while thresholds and measurement results live
 * in SENS; keeping both maps in one extension lets the reusable state machine
 * span those independently owned MMIO pages without assuming chip addresses. */
typedef struct {
    uint8_t channel_count;
    uint8_t first_external_channel;

    uint16_t rtc_control2_offset;
    uint16_t rtc_scan_control_offset;
    uint16_t rtc_sleep_threshold_offset;
    uint16_t rtc_approach_offset;
    uint16_t rtc_filter_offset;
    uint32_t rtc_clock_enable_mask;
    uint32_t rtc_reset_mask;
    uint32_t rtc_start_force_mask;
    uint32_t rtc_start_enable_mask;
    uint32_t rtc_timer_enable_mask;
    uint32_t rtc_scan_channel_mask;
    uint8_t rtc_scan_channel_shift;
    uint32_t rtc_sleep_channel_mask;
    uint8_t rtc_sleep_channel_shift;
    uint32_t rtc_sleep_threshold_mask;
    uint32_t rtc_sleep_benchmark_clear_mask;

    uint16_t sens_config_offset;
    uint16_t sens_denoise_offset;
    uint16_t sens_threshold_base_offset;
    uint16_t sens_channel_status_offset;
    uint16_t sens_status_base_offset;
    uint16_t sens_sleep_status_offset;
    uint16_t sens_approach_status_offset;
    uint32_t sens_config_reset;
    uint32_t sens_approach_channel_mask[3];
    uint8_t sens_approach_channel_shift[3];
    uint32_t sens_unit_done_mask;
    uint32_t sens_denoise_done_mask;
    uint32_t sens_data_select_mask;
    uint8_t sens_data_select_shift;
    uint32_t sens_status_clear_mask;
    uint32_t sens_output_enable_mask;
    uint32_t sens_threshold_mask;
    uint32_t sens_measure_done_mask;
    uint32_t sens_channel_clear_mask;
    uint8_t sens_channel_clear_shift;
    uint32_t sens_active_mask;
    uint32_t sens_current_channel_mask;
    uint8_t sens_current_channel_shift;
    uint32_t sens_data_mask;
    uint32_t sens_debounce_mask;
    uint8_t sens_debounce_shift;

    uint32_t interrupt_done_mask;
    uint32_t interrupt_active_mask;
    uint32_t interrupt_inactive_mask;
    uint32_t interrupt_scan_done_mask;
    uint32_t interrupt_timeout_mask;
    uint32_t interrupt_approach_done_mask;
    uint32_t rtc_wakeup_mask;
} flexe_touch_v2_desc_t;

typedef struct flexe_touch_v2 flexe_touch_v2_t;

const flexe_touch_v2_desc_t *flexe_touch_v2_descriptor(
    const flexe_target_desc_t *target);

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
bool flexe_touch_v2_sleep_wake_asserted(const flexe_touch_v2_t *touch);
bool flexe_touch_v2_running(const flexe_touch_v2_t *touch);

#endif /* FLEXE_TOUCH_V2_H */
