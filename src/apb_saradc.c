#include "apb_saradc.h"

#include "gdma.h"
#include "target.h"

#include <stdbool.h>
#include <stdlib.h>

#define APB_SARADC_UNIT_COUNT 2u

struct flexe_apb_saradc {
    xtensa_mem_t *mem;
    flexe_gdma_t *gdma;
    const flexe_target_desc_t *target;
    mmio_read_fn fallback_read;
    mmio_write_fn fallback_write;
    void *fallback_ctx;
    flexe_apb_saradc_sample_fn sample;
    void *sample_ctx;
    uint32_t control;
    uint32_t control2;
    uint32_t fsm_wait;
    uint32_t pattern[APB_SARADC_UNIT_COUNT][4];
    uint32_t arbiter;
    uint32_t filter_control;
    uint32_t dma_config;
    uint32_t clock_config;
    uint32_t date;
    uint32_t data_status[APB_SARADC_UNIT_COUNT];
    uint8_t pattern_index[APB_SARADC_UNIT_COUNT];
    uint8_t next_unit;
    bool clock_enabled;
    bool reset_asserted;
    uint64_t frames;
    uint64_t samples;
};

static bool one_bit(uint32_t value)
{
    return value != 0u && (value & (value - 1u)) == 0u;
}

static bool offset_valid(uint16_t offset, uint32_t register_size)
{
    return (offset & 3u) == 0u &&
           offset <= register_size - sizeof(uint32_t);
}

static unsigned mask_shift(uint32_t mask)
{
    unsigned shift = 0u;
    while (shift < 32u && (mask & (UINT32_C(1) << shift)) == 0u) shift++;
    return shift;
}

static bool geometry_valid(const flexe_target_desc_t *target)
{
    if (!target ||
        !(target->capabilities & FLEXE_TARGET_CAP_APB_SARADC_V1))
        return false;
    const flexe_apb_saradc_desc_t *desc = &target->apb_saradc;
    if ((desc->base & 0xFFFu) != 0u ||
        desc->register_size < sizeof(uint32_t) ||
        (desc->register_size & 0xFFFu) != 0u ||
        desc->base < target->peripheral_start ||
        desc->base >= target->peripheral_end ||
        desc->register_size > target->peripheral_end - desc->base ||
        !offset_valid(desc->control_offset, desc->register_size) ||
        !offset_valid(desc->control2_offset, desc->register_size) ||
        !offset_valid(desc->fsm_wait_offset, desc->register_size) ||
        !offset_valid(desc->arbiter_offset, desc->register_size) ||
        !offset_valid(desc->filter_control_offset, desc->register_size) ||
        !offset_valid(desc->dma_config_offset, desc->register_size) ||
        !offset_valid(desc->clock_config_offset, desc->register_size) ||
        !offset_valid(desc->date_offset, desc->register_size) ||
        desc->pattern_register_count == 0u ||
        desc->pattern_register_count > 4u ||
        desc->pattern_entries_per_register == 0u ||
        desc->pattern_entries_per_register > 4u ||
        desc->channels_per_unit == 0u || desc->channels_per_unit > 16u ||
        desc->sample_data_bits == 0u || desc->sample_data_bits > 16u ||
        desc->sample_channel_shift >= 32u ||
        desc->sample_unit_shift >= 32u ||
        desc->gdma_peripheral_id == FLEXE_TARGET_GDMA_PERIPHERAL_NONE ||
        desc->control_writable_mask == 0u ||
        desc->control2_writable_mask == 0u ||
        desc->fsm_wait_writable_mask == 0u ||
        desc->pattern_writable_mask == 0u ||
        desc->arbiter_writable_mask == 0u ||
        desc->dma_config_writable_mask == 0u ||
        desc->clock_config_writable_mask == 0u ||
        (desc->control_reset & ~desc->control_writable_mask) != 0u ||
        (desc->control2_reset & ~desc->control2_writable_mask) != 0u ||
        (desc->fsm_wait_reset & ~desc->fsm_wait_writable_mask) != 0u ||
        (desc->arbiter_reset & ~desc->arbiter_writable_mask) != 0u ||
        (desc->filter_control_reset &
         ~desc->filter_control_writable_mask) != 0u ||
        (desc->dma_config_reset & ~desc->dma_config_writable_mask) != 0u ||
        (desc->clock_config_reset &
         ~desc->clock_config_writable_mask) != 0u ||
        (desc->date_reset & ~desc->date_writable_mask) != 0u ||
        !one_bit(desc->unit_select_mask) ||
        !one_bit(desc->sar_clock_gate_mask) ||
        !one_bit(desc->timer_select_mask) ||
        !one_bit(desc->timer_enable_mask) ||
        !one_bit(desc->dma_enable_mask) ||
        !one_bit(desc->dma_reset_mask) ||
        !one_bit(desc->grant_force_mask) ||
        !one_bit(desc->apb_force_mask) ||
        !one_bit(desc->rtc_force_mask) ||
        (desc->grant_force_mask & desc->force_selection_mask) != 0u ||
        (desc->force_selection_mask & desc->apb_force_mask) == 0u ||
        (desc->force_selection_mask & desc->rtc_force_mask) == 0u ||
        (desc->grant_force_mask | desc->force_selection_mask) !=
            ((desc->grant_force_mask | desc->force_selection_mask) &
             desc->arbiter_writable_mask))
        return false;
    for (unsigned unit = 0u; unit < APB_SARADC_UNIT_COUNT; unit++) {
        if (!offset_valid(desc->pattern_offset[unit],
                          desc->register_size) ||
            !offset_valid(desc->data_status_offset[unit],
                          desc->register_size) ||
            desc->pattern_offset[unit] >
                desc->register_size -
                    desc->pattern_register_count * sizeof(uint32_t) ||
            desc->pattern_length_mask[unit] == 0u ||
            !one_bit(desc->pattern_clear_mask[unit]) ||
            !one_bit(desc->sample_invert_mask[unit]))
            return false;
    }
    return true;
}

static void apb_saradc_reset(flexe_apb_saradc_t *adc)
{
    const flexe_apb_saradc_desc_t *desc = &adc->target->apb_saradc;
    adc->control = desc->control_reset;
    adc->control2 = desc->control2_reset;
    adc->fsm_wait = desc->fsm_wait_reset;
    for (unsigned unit = 0u; unit < APB_SARADC_UNIT_COUNT; unit++) {
        for (unsigned index = 0u; index < 4u; index++)
            adc->pattern[unit][index] = 0u;
        adc->data_status[unit] = 0u;
        adc->pattern_index[unit] = 0u;
    }
    adc->arbiter = desc->arbiter_reset;
    adc->filter_control = desc->filter_control_reset;
    adc->dma_config = desc->dma_config_reset;
    adc->clock_config = desc->clock_config_reset;
    adc->date = desc->date_reset;
    adc->next_unit = 0u;
}

static void fallback_write(flexe_apb_saradc_t *adc, uint32_t addr,
                           uint32_t value)
{
    if (adc->fallback_write)
        adc->fallback_write(adc->fallback_ctx, addr, value);
}

static void retain(flexe_apb_saradc_t *adc, uint32_t addr,
                   uint32_t value, uint32_t mask, uint32_t *storage)
{
    *storage = value & mask;
    if ((value & ~mask) != 0u) fallback_write(adc, addr, value);
}

static bool apb_saradc_digital_granted(const flexe_apb_saradc_t *adc)
{
    const flexe_apb_saradc_desc_t *desc = &adc->target->apb_saradc;
    return (adc->arbiter & desc->grant_force_mask) == 0u ||
           (adc->arbiter & desc->force_selection_mask) ==
               desc->apb_force_mask;
}

static bool apb_saradc_streaming(const flexe_apb_saradc_t *adc)
{
    const flexe_apb_saradc_desc_t *desc = &adc->target->apb_saradc;
    return adc->clock_enabled && !adc->reset_asserted &&
           (adc->control & desc->sar_clock_gate_mask) != 0u &&
           (adc->control2 & desc->timer_select_mask) != 0u &&
           (adc->control2 & desc->timer_enable_mask) != 0u &&
           (adc->dma_config & desc->dma_enable_mask) != 0u;
}

static unsigned apb_saradc_pattern_length(
    const flexe_apb_saradc_t *adc, unsigned unit)
{
    const flexe_apb_saradc_desc_t *desc = &adc->target->apb_saradc;
    uint32_t mask = desc->pattern_length_mask[unit];
    return ((adc->control & mask) >> mask_shift(mask)) + 1u;
}

static uint8_t apb_saradc_pattern_entry(
    const flexe_apb_saradc_t *adc, unsigned unit, unsigned index)
{
    const flexe_apb_saradc_desc_t *desc = &adc->target->apb_saradc;
    unsigned per_register = desc->pattern_entries_per_register;
    unsigned reg = index / per_register;
    unsigned within = index % per_register;
    unsigned width = 24u / per_register;
    unsigned shift = 24u - width * (within + 1u);
    uint32_t mask = (UINT32_C(1) << width) - 1u;
    return (uint8_t)((adc->pattern[unit][reg] >> shift) & mask);
}

static unsigned apb_saradc_next_unit(flexe_apb_saradc_t *adc)
{
    const flexe_apb_saradc_desc_t *desc = &adc->target->apb_saradc;
    unsigned work_mode =
        (adc->control & desc->work_mode_mask) >>
        mask_shift(desc->work_mode_mask);
    if (work_mode == 0u)
        return (adc->control & desc->unit_select_mask) != 0u ? 1u : 0u;
    unsigned unit = adc->next_unit;
    adc->next_unit ^= 1u;
    return unit;
}

static uint32_t apb_saradc_next_sample(flexe_apb_saradc_t *adc)
{
    const flexe_apb_saradc_desc_t *desc = &adc->target->apb_saradc;
    unsigned unit = apb_saradc_next_unit(adc);
    unsigned length = apb_saradc_pattern_length(adc, unit);
    unsigned index = adc->pattern_index[unit]++ % length;
    uint8_t entry = apb_saradc_pattern_entry(adc, unit, index);
    unsigned channel = entry >> 2u;
    uint32_t data_mask = (UINT32_C(1) << desc->sample_data_bits) - 1u;
    uint32_t raw = 0u;
    bool valid = channel < desc->channels_per_unit &&
                 (unit == 0u || apb_saradc_digital_granted(adc));
    if (valid && adc->sample)
        raw = adc->sample(adc->sample_ctx, unit, channel) & data_mask;
    if (!valid) channel = desc->channels_per_unit;
    if ((adc->control2 & desc->sample_invert_mask[unit]) != 0u)
        raw ^= data_mask;
    uint32_t result = raw |
        ((uint32_t)channel << desc->sample_channel_shift) |
        ((uint32_t)unit << desc->sample_unit_shift);
    adc->data_status[unit] = result;
    adc->samples++;
    return result;
}

size_t flexe_apb_saradc_inject_frame(flexe_apb_saradc_t *adc)
{
    if (!adc || !adc->gdma || !apb_saradc_streaming(adc)) return 0u;
    const flexe_apb_saradc_desc_t *desc = &adc->target->apb_saradc;
    size_t capacity = 0u;
    if (!flexe_gdma_pending_length(adc->gdma, desc->gdma_peripheral_id,
                                   true, &capacity) ||
        capacity < sizeof(uint32_t))
        return 0u;
    size_t conversions = adc->dma_config & desc->dma_eof_mask;
    if (conversions == 0u) conversions = capacity / sizeof(uint32_t);
    size_t bytes = conversions * sizeof(uint32_t);
    if (bytes > capacity) bytes = capacity;
    bytes &= ~(sizeof(uint32_t) - 1u);
    if (bytes == 0u) return 0u;

    uint8_t *frame = malloc(bytes);
    if (!frame) return 0u;
    for (size_t offset = 0u; offset < bytes; offset += sizeof(uint32_t)) {
        uint32_t sample = apb_saradc_next_sample(adc);
        frame[offset] = (uint8_t)sample;
        frame[offset + 1u] = (uint8_t)(sample >> 8u);
        frame[offset + 2u] = (uint8_t)(sample >> 16u);
        frame[offset + 3u] = (uint8_t)(sample >> 24u);
    }
    flexe_gdma_descriptor_t completed = {0};
    int status = flexe_gdma_write_rx_descriptor(
        adc->gdma, desc->gdma_peripheral_id, frame, bytes, &completed);
    free(frame);
    if (status != 0) return 0u;
    adc->frames++;
    return bytes;
}

static uint32_t apb_saradc_read(void *ctx, uint32_t addr)
{
    flexe_apb_saradc_t *adc = ctx;
    const flexe_apb_saradc_desc_t *desc = &adc->target->apb_saradc;
    uint32_t offset = addr - desc->base;
    if (offset == desc->control_offset) return adc->control;
    if (offset == desc->control2_offset) return adc->control2;
    if (offset == desc->fsm_wait_offset) return adc->fsm_wait;
    for (unsigned unit = 0u; unit < APB_SARADC_UNIT_COUNT; unit++) {
        if (offset == desc->data_status_offset[unit])
            return adc->data_status[unit];
        if (offset >= desc->pattern_offset[unit] &&
            offset < desc->pattern_offset[unit] +
                     desc->pattern_register_count * sizeof(uint32_t))
            return adc->pattern[unit][
                (offset - desc->pattern_offset[unit]) / sizeof(uint32_t)];
    }
    if (offset == desc->arbiter_offset) return adc->arbiter;
    if (offset == desc->filter_control_offset) return adc->filter_control;
    if (offset == desc->dma_config_offset) return adc->dma_config;
    if (offset == desc->clock_config_offset) return adc->clock_config;
    if (offset == desc->date_offset) return adc->date;
    return adc->fallback_read ?
        adc->fallback_read(adc->fallback_ctx, addr) : 0u;
}

static void apb_saradc_write(void *ctx, uint32_t addr, uint32_t value)
{
    flexe_apb_saradc_t *adc = ctx;
    const flexe_apb_saradc_desc_t *desc = &adc->target->apb_saradc;
    uint32_t offset = addr - desc->base;
    if (offset == desc->control_offset) {
        uint32_t old = adc->control;
        retain(adc, addr, value, desc->control_writable_mask,
               &adc->control);
        for (unsigned unit = 0u; unit < APB_SARADC_UNIT_COUNT; unit++)
            if ((adc->control & desc->pattern_clear_mask[unit]) != 0u &&
                (old & desc->pattern_clear_mask[unit]) == 0u)
                adc->pattern_index[unit] = 0u;
        return;
    }
    if (offset == desc->control2_offset) {
        uint32_t old = adc->control2;
        retain(adc, addr, value, desc->control2_writable_mask,
               &adc->control2);
        if ((adc->control2 & desc->timer_enable_mask) != 0u &&
            (old & desc->timer_enable_mask) == 0u)
            (void)flexe_apb_saradc_inject_frame(adc);
        return;
    }
    if (offset == desc->fsm_wait_offset) {
        retain(adc, addr, value, desc->fsm_wait_writable_mask,
               &adc->fsm_wait);
        return;
    }
    for (unsigned unit = 0u; unit < APB_SARADC_UNIT_COUNT; unit++) {
        if (offset >= desc->pattern_offset[unit] &&
            offset < desc->pattern_offset[unit] +
                     desc->pattern_register_count * sizeof(uint32_t)) {
            unsigned index =
                (offset - desc->pattern_offset[unit]) / sizeof(uint32_t);
            retain(adc, addr, value, desc->pattern_writable_mask,
                   &adc->pattern[unit][index]);
            return;
        }
    }
    if (offset == desc->arbiter_offset) {
        adc->arbiter = value & desc->arbiter_writable_mask;
        uint32_t forced = adc->arbiter & desc->force_selection_mask;
        bool force_unknown =
            (adc->arbiter & desc->grant_force_mask) != 0u &&
            forced != desc->rtc_force_mask &&
            forced != desc->apb_force_mask;
        if (force_unknown ||
            (value & ~desc->arbiter_writable_mask) != 0u)
            fallback_write(adc, addr, value);
        return;
    }
    if (offset == desc->filter_control_offset) {
        retain(adc, addr, value, desc->filter_control_writable_mask,
               &adc->filter_control);
        return;
    }
    if (offset == desc->dma_config_offset) {
        uint32_t old = adc->dma_config;
        retain(adc, addr, value, desc->dma_config_writable_mask,
               &adc->dma_config);
        if ((adc->dma_config & desc->dma_reset_mask) != 0u &&
            (old & desc->dma_reset_mask) == 0u) {
            adc->pattern_index[0] = 0u;
            adc->pattern_index[1] = 0u;
            adc->next_unit = 0u;
        }
        return;
    }
    if (offset == desc->clock_config_offset) {
        retain(adc, addr, value, desc->clock_config_writable_mask,
               &adc->clock_config);
        return;
    }
    if (offset == desc->date_offset) {
        retain(adc, addr, value, desc->date_writable_mask, &adc->date);
        return;
    }
    fallback_write(adc, addr, value);
}

flexe_apb_saradc_t *flexe_apb_saradc_create(
    xtensa_mem_t *mem, flexe_gdma_t *gdma,
    flexe_apb_saradc_sample_fn sample, void *sample_ctx,
    mmio_read_fn fallback_read, mmio_write_fn fallback_write,
    void *fallback_ctx)
{
    if (!mem) return NULL;
    const flexe_target_desc_t *target = mem_target(mem);
    if (!geometry_valid(target)) return NULL;
    flexe_apb_saradc_t *adc = calloc(1u, sizeof(*adc));
    if (!adc) return NULL;
    adc->mem = mem;
    adc->gdma = gdma;
    adc->target = target;
    adc->fallback_read = fallback_read;
    adc->fallback_write = fallback_write;
    adc->fallback_ctx = fallback_ctx;
    adc->sample = sample;
    adc->sample_ctx = sample ? sample_ctx : NULL;
    adc->clock_enabled = true;
    apb_saradc_reset(adc);
    if (mem_register_mmio_range(mem, target->apb_saradc.base,
                                target->apb_saradc.register_size,
                                apb_saradc_read, apb_saradc_write, adc) != 0) {
        free(adc);
        return NULL;
    }
    return adc;
}

void flexe_apb_saradc_destroy(flexe_apb_saradc_t *adc)
{
    if (!adc) return;
    const flexe_apb_saradc_desc_t *desc = &adc->target->apb_saradc;
    (void)mem_register_mmio_range(
        adc->mem, desc->base, desc->register_size,
        adc->fallback_read, adc->fallback_write, adc->fallback_ctx);
    free(adc);
}

void flexe_apb_saradc_set_system_state(
    flexe_apb_saradc_t *adc, bool clock_enabled, bool reset_asserted)
{
    if (!adc) return;
    bool reset_rising = reset_asserted && !adc->reset_asserted;
    adc->clock_enabled = clock_enabled;
    adc->reset_asserted = reset_asserted;
    if (reset_rising) apb_saradc_reset(adc);
}

bool flexe_apb_saradc_rtc_granted(const flexe_apb_saradc_t *adc)
{
    if (!adc) return false;
    const flexe_apb_saradc_desc_t *desc = &adc->target->apb_saradc;
    return (adc->arbiter & desc->grant_force_mask) == 0u ||
           (adc->arbiter & desc->force_selection_mask) ==
               desc->rtc_force_mask;
}

bool flexe_apb_saradc_stream_active(const flexe_apb_saradc_t *adc)
{
    return adc && apb_saradc_streaming(adc);
}

uint64_t flexe_apb_saradc_frame_count(const flexe_apb_saradc_t *adc)
{
    return adc ? adc->frames : 0u;
}

uint64_t flexe_apb_saradc_sample_count(const flexe_apb_saradc_t *adc)
{
    return adc ? adc->samples : 0u;
}
