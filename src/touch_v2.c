#include "touch_v2.h"
#include "target.h"

#include <stdlib.h>

enum {
    TOUCH_DATA_RAW = 0u,
    TOUCH_DATA_BENCHMARK = 2u,
    TOUCH_DATA_SMOOTH = 3u,
};

struct flexe_touch_v2 {
    const flexe_touch_v2_desc_t *desc;
    flexe_touch_v2_irq_fn irq;
    void *irq_ctx;
    uint32_t rtc_control2;
    uint32_t rtc_scan_control;
    uint32_t rtc_sleep_threshold;
    uint32_t rtc_approach;
    uint32_t rtc_filter;
    uint32_t sens_config;
    uint32_t threshold[FLEXE_TARGET_TOUCH_CHANNEL_MAX];
    uint32_t raw[FLEXE_TARGET_TOUCH_CHANNEL_MAX];
    uint32_t benchmark[FLEXE_TARGET_TOUCH_CHANNEL_MAX];
    uint32_t smooth[FLEXE_TARGET_TOUCH_CHANNEL_MAX];
    uint8_t debounce[FLEXE_TARGET_TOUCH_CHANNEL_MAX];
    uint32_t active;
    uint8_t current_channel;
    bool measurement_done;
    uint64_t scans;
};

static bool contiguous_field(uint32_t mask, unsigned shift)
{
    if (mask == 0u || shift >= 32u || (mask & ((1u << shift) - 1u)) != 0u)
        return false;
    uint32_t field = mask >> shift;
    return (field & (field + 1u)) == 0u;
}

const flexe_touch_v2_desc_t *flexe_touch_v2_descriptor(
    const flexe_target_desc_t *target)
{
    return flexe_target_extension(target, FLEXE_TARGET_EXTENSION_TOUCH_V2);
}

static bool touch_geometry_valid(const flexe_target_desc_t *target,
                                 const flexe_touch_v2_desc_t *desc)
{
    if (!target || !desc ||
        !(target->capabilities & FLEXE_TARGET_CAP_TOUCH_V2) ||
        !(target->capabilities & FLEXE_TARGET_CAP_RTC_CNTL_V1) ||
        !(target->capabilities & FLEXE_TARGET_CAP_SENS_V1))
        return false;
    if (desc->channel_count == 0u ||
        desc->channel_count > FLEXE_TARGET_TOUCH_CHANNEL_MAX ||
        desc->first_external_channel >= desc->channel_count ||
        desc->rtc_control2_offset >= target->rtc_cntl.register_size ||
        desc->rtc_scan_control_offset >= target->rtc_cntl.register_size ||
        desc->rtc_sleep_threshold_offset >=
            target->rtc_cntl.register_size ||
        desc->rtc_approach_offset >= target->rtc_cntl.register_size ||
        desc->rtc_filter_offset >= target->rtc_cntl.register_size ||
        desc->sens_config_offset >= target->sens.register_size ||
        desc->sens_denoise_offset >= target->sens.register_size ||
        desc->sens_threshold_base_offset +
            4u * (desc->channel_count - desc->first_external_channel) >
            target->sens.register_size ||
        desc->sens_channel_status_offset >= target->sens.register_size ||
        desc->sens_status_base_offset + 4u * desc->channel_count >
            target->sens.register_size ||
        desc->sens_sleep_status_offset >= target->sens.register_size ||
        desc->sens_approach_status_offset >= target->sens.register_size ||
        !contiguous_field(desc->rtc_scan_channel_mask,
                          desc->rtc_scan_channel_shift) ||
        !contiguous_field(desc->rtc_sleep_channel_mask,
                          desc->rtc_sleep_channel_shift) ||
        !contiguous_field(desc->sens_data_select_mask,
                          desc->sens_data_select_shift) ||
        !contiguous_field(desc->sens_channel_clear_mask,
                          desc->sens_channel_clear_shift) ||
        !contiguous_field(desc->sens_current_channel_mask,
                          desc->sens_current_channel_shift) ||
        !contiguous_field(desc->sens_debounce_mask,
                          desc->sens_debounce_shift) ||
        desc->sens_data_mask == 0u ||
        desc->sens_threshold_mask != desc->sens_data_mask ||
        desc->sens_active_mask !=
            ((1u << desc->channel_count) - 1u) ||
        desc->rtc_wakeup_mask == 0u ||
        (desc->rtc_wakeup_mask & (desc->rtc_wakeup_mask - 1u)) != 0u ||
        (desc->rtc_wakeup_mask &
         ~target->rtc_cntl.wakeup_valid_mask) != 0u ||
        (desc->rtc_wakeup_mask &
         (target->rtc_cntl.timer_wakeup_mask |
          target->rtc_cntl.ext0_wakeup_mask |
          target->rtc_cntl.ext1_wakeup_mask)) != 0u)
        return false;
    return true;
}

static uint32_t touch_channel_mask(const flexe_touch_v2_t *touch)
{
    return (1u << touch->desc->channel_count) - 1u;
}

static bool touch_running_internal(const flexe_touch_v2_t *touch)
{
    const flexe_touch_v2_desc_t *desc = touch->desc;
    if ((touch->rtc_control2 & desc->rtc_clock_enable_mask) == 0u ||
        (touch->rtc_control2 & desc->rtc_reset_mask) != 0u)
        return false;
    if ((touch->rtc_control2 & desc->rtc_timer_enable_mask) != 0u)
        return true;
    return (touch->rtc_control2 &
            (desc->rtc_start_force_mask | desc->rtc_start_enable_mask)) ==
           (desc->rtc_start_force_mask | desc->rtc_start_enable_mask);
}

static uint32_t touch_scan_mask(const flexe_touch_v2_t *touch)
{
    const flexe_touch_v2_desc_t *desc = touch->desc;
    uint32_t rtc = (touch->rtc_scan_control &
                    desc->rtc_scan_channel_mask) >>
                   desc->rtc_scan_channel_shift;
    return rtc & touch->sens_config & desc->sens_output_enable_mask &
           touch_channel_mask(touch);
}

static void touch_reset_measurements(flexe_touch_v2_t *touch)
{
    touch->active = 0u;
    touch->current_channel = 0u;
    touch->measurement_done = false;
    for (unsigned channel = 0u;
         channel < touch->desc->channel_count; channel++) {
        touch->benchmark[channel] = 0u;
        touch->smooth[channel] = 0u;
        touch->debounce[channel] = 0u;
    }
}

static void touch_scan(flexe_touch_v2_t *touch)
{
    const flexe_touch_v2_desc_t *desc = touch->desc;
    uint32_t enabled = touch_scan_mask(touch);
    uint32_t before = touch->active;
    uint32_t active = 0u;
    unsigned current = 0u;
    for (unsigned channel = desc->first_external_channel;
         channel < desc->channel_count; channel++) {
        uint32_t bit = 1u << channel;
        if ((enabled & bit) == 0u) {
            touch->debounce[channel] = 0u;
            continue;
        }
        current = channel;
        uint32_t raw = touch->raw[channel] & desc->sens_data_mask;
        if (touch->benchmark[channel] == 0u)
            touch->benchmark[channel] = raw;
        /* Fast mode collapses the configured IIR/jitter cadence to a stable
         * sample boundary while preserving all filter register state. */
        touch->smooth[channel] = raw;
        uint32_t threshold = touch->threshold[channel] &
                             desc->sens_threshold_mask;
        uint32_t baseline = touch->benchmark[channel];
        bool triggered = threshold != desc->sens_threshold_mask &&
                         raw > baseline && raw - baseline > threshold;
        if (triggered) active |= bit;
        touch->debounce[channel] = triggered ? 1u : 0u;
    }
    touch->active = active;
    touch->current_channel = (uint8_t)current;
    touch->measurement_done = true;
    touch->scans++;

    uint32_t interrupts = desc->interrupt_done_mask |
                          desc->interrupt_scan_done_mask;
    if ((active & ~before) != 0u)
        interrupts |= desc->interrupt_active_mask;
    if ((before & ~active) != 0u)
        interrupts |= desc->interrupt_inactive_mask;
    if (touch->irq && interrupts != 0u)
        touch->irq(touch->irq_ctx, interrupts);
}

static uint32_t touch_selected_data(const flexe_touch_v2_t *touch,
                                    unsigned channel)
{
    const flexe_touch_v2_desc_t *desc = touch->desc;
    unsigned selection = (touch->sens_config &
                          desc->sens_data_select_mask) >>
                         desc->sens_data_select_shift;
    if (selection == TOUCH_DATA_BENCHMARK)
        return touch->benchmark[channel] & desc->sens_data_mask;
    if (selection == TOUCH_DATA_SMOOTH)
        return touch->smooth[channel] & desc->sens_data_mask;
    return touch->raw[channel] & desc->sens_data_mask;
}

flexe_touch_v2_t *flexe_touch_v2_create(
    xtensa_mem_t *mem, flexe_touch_v2_irq_fn irq, void *irq_ctx)
{
    if (!mem) return NULL;
    const flexe_target_desc_t *target = mem_target(mem);
    const flexe_touch_v2_desc_t *desc =
        flexe_touch_v2_descriptor(target);
    if (!touch_geometry_valid(target, desc)) return NULL;
    flexe_touch_v2_t *touch = calloc(1u, sizeof(*touch));
    if (!touch) return NULL;
    touch->desc = desc;
    touch->irq = irq;
    touch->irq_ctx = irq ? irq_ctx : NULL;
    touch->sens_config = desc->sens_config_reset;
    return touch;
}

void flexe_touch_v2_destroy(flexe_touch_v2_t *touch)
{
    free(touch);
}

void flexe_touch_v2_rtc_config_changed(
    flexe_touch_v2_t *touch, uint16_t offset, uint32_t value)
{
    if (!touch) return;
    const flexe_touch_v2_desc_t *desc = touch->desc;
    if (offset == desc->rtc_control2_offset) {
        uint32_t before = touch->rtc_control2;
        touch->rtc_control2 = value;
        if ((value & desc->rtc_reset_mask) != 0u) {
            touch_reset_measurements(touch);
            return;
        }
        bool software_start =
            (before & desc->rtc_start_enable_mask) == 0u &&
            (value & (desc->rtc_start_force_mask |
                      desc->rtc_start_enable_mask)) ==
                (desc->rtc_start_force_mask |
                 desc->rtc_start_enable_mask);
        bool timer_start =
            (before & desc->rtc_timer_enable_mask) == 0u &&
            (value & desc->rtc_timer_enable_mask) != 0u;
        if (software_start || timer_start) touch_scan(touch);
        return;
    }
    if (offset == desc->rtc_scan_control_offset) {
        uint32_t before = touch->rtc_scan_control;
        touch->rtc_scan_control = value;
        if (before != value && touch_running_internal(touch))
            touch_scan(touch);
        return;
    }
    if (offset == desc->rtc_sleep_threshold_offset) {
        touch->rtc_sleep_threshold = value;
        return;
    }
    if (offset == desc->rtc_approach_offset) {
        touch->rtc_approach =
            value & ~desc->rtc_sleep_benchmark_clear_mask;
        if ((value & desc->rtc_sleep_benchmark_clear_mask) != 0u) {
            unsigned channel = (touch->rtc_sleep_threshold &
                                desc->rtc_sleep_channel_mask) >>
                               desc->rtc_sleep_channel_shift;
            if (channel < desc->channel_count)
                touch->benchmark[channel] = touch->raw[channel] &
                                            desc->sens_data_mask;
        }
        return;
    }
    if (offset == desc->rtc_filter_offset)
        touch->rtc_filter = value;
}

bool flexe_touch_v2_sens_read(
    flexe_touch_v2_t *touch, uint32_t offset, uint32_t *value)
{
    if (!touch || !value) return false;
    const flexe_touch_v2_desc_t *desc = touch->desc;
    if (offset == desc->sens_config_offset) {
        *value = touch->sens_config |
                 (touch->measurement_done ? desc->sens_unit_done_mask : 0u);
        return true;
    }
    if (offset == desc->sens_denoise_offset) {
        *value = touch_selected_data(touch, 0u);
        return true;
    }
    uint32_t threshold_end = desc->sens_threshold_base_offset +
        4u * (desc->channel_count - desc->first_external_channel);
    if (offset >= desc->sens_threshold_base_offset &&
        offset < threshold_end && (offset & 3u) == 0u) {
        unsigned channel = desc->first_external_channel +
            (offset - desc->sens_threshold_base_offset) / 4u;
        *value = touch->threshold[channel];
        return true;
    }
    if (offset == desc->sens_channel_status_offset) {
        *value = (touch->measurement_done ?
                  desc->sens_measure_done_mask : 0u) |
                 (touch->active & desc->sens_active_mask);
        return true;
    }
    if (offset == desc->sens_status_base_offset) {
        *value = ((uint32_t)touch->current_channel <<
                  desc->sens_current_channel_shift) &
                 desc->sens_current_channel_mask;
        *value |= touch_selected_data(touch, 0u);
        return true;
    }
    uint32_t status_end = desc->sens_status_base_offset +
                          4u * desc->channel_count;
    if (offset > desc->sens_status_base_offset && offset < status_end &&
        (offset & 3u) == 0u) {
        unsigned channel =
            (offset - desc->sens_status_base_offset) / 4u;
        *value = touch_selected_data(touch, channel) |
            (((uint32_t)touch->debounce[channel] <<
              desc->sens_debounce_shift) & desc->sens_debounce_mask);
        return true;
    }
    if (offset == desc->sens_sleep_status_offset) {
        unsigned channel = (touch->rtc_sleep_threshold &
                            desc->rtc_sleep_channel_mask) >>
                           desc->rtc_sleep_channel_shift;
        *value = channel < desc->channel_count ?
            touch_selected_data(touch, channel) : 0u;
        return true;
    }
    if (offset == desc->sens_approach_status_offset) {
        *value = 0u;
        return true;
    }
    return false;
}

bool flexe_touch_v2_sens_write(
    flexe_touch_v2_t *touch, uint32_t offset, uint32_t value)
{
    if (!touch) return false;
    const flexe_touch_v2_desc_t *desc = touch->desc;
    if (offset == desc->sens_config_offset) {
        uint32_t writable = desc->sens_approach_channel_mask[0] |
                            desc->sens_approach_channel_mask[1] |
                            desc->sens_approach_channel_mask[2] |
                            desc->sens_data_select_mask |
                            desc->sens_output_enable_mask;
        touch->sens_config = value & writable;
        if ((value & desc->sens_status_clear_mask) != 0u)
            touch->active = 0u;
        if (touch_running_internal(touch)) touch_scan(touch);
        return true;
    }
    if (offset == desc->sens_denoise_offset)
        return true; /* Physically read-only. */
    uint32_t threshold_end = desc->sens_threshold_base_offset +
        4u * (desc->channel_count - desc->first_external_channel);
    if (offset >= desc->sens_threshold_base_offset &&
        offset < threshold_end && (offset & 3u) == 0u) {
        unsigned channel = desc->first_external_channel +
            (offset - desc->sens_threshold_base_offset) / 4u;
        touch->threshold[channel] = value & desc->sens_threshold_mask;
        return true;
    }
    if (offset == desc->sens_channel_status_offset) {
        uint32_t clear = (value & desc->sens_channel_clear_mask) >>
                         desc->sens_channel_clear_shift;
        clear &= touch_channel_mask(touch);
        for (unsigned channel = 0u; channel < desc->channel_count;
             channel++) {
            if ((clear & (1u << channel)) == 0u) continue;
            touch->benchmark[channel] = touch->raw[channel] &
                                        desc->sens_data_mask;
            touch->smooth[channel] = touch->benchmark[channel];
            touch->debounce[channel] = 0u;
        }
        touch->active &= ~clear;
        touch->measurement_done = false;
        return true;
    }
    uint32_t status_end = desc->sens_status_base_offset +
                          4u * desc->channel_count;
    if ((offset >= desc->sens_status_base_offset && offset < status_end) ||
        offset == desc->sens_sleep_status_offset ||
        offset == desc->sens_approach_status_offset)
        return true; /* Physically read-only. */
    return false;
}

void flexe_touch_v2_set_raw(
    flexe_touch_v2_t *touch, unsigned channel, uint32_t value)
{
    if (!touch || channel >= touch->desc->channel_count) return;
    touch->raw[channel] = value & touch->desc->sens_data_mask;
    if (touch_running_internal(touch)) touch_scan(touch);
}

uint64_t flexe_touch_v2_scan_count(const flexe_touch_v2_t *touch)
{
    return touch ? touch->scans : 0u;
}

uint32_t flexe_touch_v2_active_mask(const flexe_touch_v2_t *touch)
{
    return touch ? touch->active : 0u;
}

bool flexe_touch_v2_sleep_wake_asserted(const flexe_touch_v2_t *touch)
{
    if (!touch || !touch_running_internal(touch)) return false;
    const flexe_touch_v2_desc_t *desc = touch->desc;
    unsigned channel = (touch->rtc_sleep_threshold &
                        desc->rtc_sleep_channel_mask) >>
                       desc->rtc_sleep_channel_shift;
    if (channel < desc->first_external_channel ||
        channel >= desc->channel_count ||
        (touch_scan_mask(touch) & (1u << channel)) == 0u)
        return false;
    uint32_t raw = touch->raw[channel] & desc->sens_data_mask;
    uint32_t baseline = touch->benchmark[channel] & desc->sens_data_mask;
    uint32_t threshold = touch->rtc_sleep_threshold &
                         desc->rtc_sleep_threshold_mask;
    return raw > baseline && raw - baseline > threshold;
}

bool flexe_touch_v2_running(const flexe_touch_v2_t *touch)
{
    return touch && touch_running_internal(touch);
}
