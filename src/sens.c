#include "sens.h"

#include <stdbool.h>
#include <stdlib.h>

struct flexe_sens {
    xtensa_mem_t *mem;
    const flexe_target_desc_t *target;
    mmio_read_fn fallback_read;
    mmio_write_fn fallback_write;
    void *fallback_ctx;
    flexe_sens_conversion_fn conversion_done;
    void *conversion_ctx;
    uint32_t control;
    uint32_t control2;
    uint32_t clock_gate;
    uint32_t reset;
    uint16_t input_raw;
    uint16_t output;
    bool ready;
};

static bool one_bit(uint32_t value)
{
    return value != 0u && (value & (value - 1u)) == 0u;
}

static bool register_offset_valid(uint16_t offset, uint32_t size)
{
    return (offset & 3u) == 0u &&
           offset <= size - sizeof(uint32_t);
}

static unsigned field_shift(uint32_t mask)
{
    unsigned shift = 0u;
    while ((mask & 1u) == 0u) {
        mask >>= 1u;
        shift++;
    }
    return shift;
}

static bool output_field_valid(uint32_t mask, uint16_t reset)
{
    if (mask == 0u) return false;
    unsigned shift = field_shift(mask);
    uint32_t field = mask >> shift;
    return field <= UINT16_MAX && (field & (field + 1u)) == 0u &&
           reset <= field;
}

static bool sens_geometry_valid(const flexe_target_desc_t *target)
{
    if (!target || !(target->capabilities & FLEXE_TARGET_CAP_SENS_V1))
        return false;
    const flexe_sens_desc_t *desc = &target->sens;
    if ((desc->base & 3u) != 0u || desc->register_size < 4u ||
        desc->base < target->peripheral_start ||
        desc->base >= target->peripheral_end ||
        desc->register_size > target->peripheral_end - desc->base ||
        !register_offset_valid(desc->control_offset,
                               desc->register_size) ||
        !register_offset_valid(desc->control2_offset,
                               desc->register_size) ||
        !register_offset_valid(desc->clock_gate_offset,
                               desc->register_size) ||
        !register_offset_valid(desc->reset_offset,
                               desc->register_size))
        return false;

    const uint16_t offsets[] = {
        desc->control_offset, desc->control2_offset,
        desc->clock_gate_offset, desc->reset_offset,
    };
    for (unsigned i = 0u; i < sizeof(offsets) / sizeof(offsets[0]); i++)
        for (unsigned j = 0u; j < i; j++)
            if (offsets[i] == offsets[j]) return false;

    const uint32_t control_bits[] = {
        desc->dump_out_mask, desc->power_up_force_mask,
        desc->power_up_mask, desc->input_invert_mask,
        desc->interrupt_enable_mask, desc->ready_mask,
    };
    for (unsigned i = 0u;
         i < sizeof(control_bits) / sizeof(control_bits[0]); i++) {
        if (!one_bit(control_bits[i])) return false;
        if ((control_bits[i] & desc->output_mask) != 0u) return false;
        for (unsigned j = 0u; j < i; j++)
            if ((control_bits[i] & control_bits[j]) != 0u) return false;
    }

    uint32_t required_writable =
        desc->dump_out_mask | desc->power_up_force_mask |
        desc->power_up_mask | desc->input_invert_mask |
        desc->interrupt_enable_mask;
    uint32_t dynamic = desc->ready_mask | desc->output_mask;
    if (!output_field_valid(desc->output_mask, desc->default_output) ||
        (required_writable & ~desc->control_writable_mask) != 0u ||
        (dynamic & desc->control_writable_mask) != 0u ||
        (desc->control_reset & ~desc->control_writable_mask) != 0u ||
        desc->xpd_force_mask == 0u ||
        (desc->xpd_force_mask & ~desc->control2_writable_mask) != 0u ||
        (desc->control2_reset & ~desc->control2_writable_mask) != 0u ||
        !one_bit(desc->clock_enable_mask) ||
        (desc->clock_enable_mask &
         ~desc->clock_gate_writable_mask) != 0u ||
        (desc->clock_gate_reset &
         ~desc->clock_gate_writable_mask) != 0u ||
        !one_bit(desc->reset_mask) ||
        (desc->reset_mask & ~desc->reset_writable_mask) != 0u ||
        (desc->reset_reset & ~desc->reset_writable_mask) != 0u)
        return false;

    if (desc->rtc_interrupt_mask != 0u) {
        if (!one_bit(desc->rtc_interrupt_mask) ||
            !(target->capabilities & FLEXE_TARGET_CAP_RTC_CNTL_V1) ||
            (desc->rtc_interrupt_mask &
             ~target->rtc_cntl.interrupt_valid_mask) != 0u)
            return false;
    }
    return true;
}

static uint16_t sens_output_max(const flexe_sens_desc_t *desc)
{
    return (uint16_t)(desc->output_mask >> field_shift(desc->output_mask));
}

static void sens_reset_temperature(flexe_sens_t *sens)
{
    const flexe_sens_desc_t *desc = &sens->target->sens;
    sens->control = desc->control_reset & desc->control_writable_mask;
    sens->control2 = desc->control2_reset & desc->control2_writable_mask;
    sens->output = 0u;
    sens->ready = false;
}

static bool sens_temperature_enabled(const flexe_sens_t *sens)
{
    const flexe_sens_desc_t *desc = &sens->target->sens;
    return (sens->clock_gate & desc->clock_enable_mask) != 0u &&
           (sens->reset & desc->reset_mask) == 0u &&
           (sens->control & desc->power_up_force_mask) != 0u &&
           (sens->control & desc->power_up_mask) != 0u &&
           (sens->control2 & desc->xpd_force_mask) != 0u;
}

static void sens_update_conversion(flexe_sens_t *sens)
{
    const flexe_sens_desc_t *desc = &sens->target->sens;
    if ((sens->control & desc->dump_out_mask) == 0u ||
        !sens_temperature_enabled(sens)) {
        sens->ready = false;
        return;
    }
    if (sens->ready) return;

    uint16_t field_max = sens_output_max(desc);
    sens->output = sens->input_raw;
    if (sens->control & desc->input_invert_mask)
        sens->output ^= field_max;
    sens->ready = true;
    /* Fast mode deliberately resolves the conversion at this write boundary.
     * Timed/cycle modes can schedule the same completion from CLK_DIV and
     * XPD_WAIT without changing the firmware-visible register contract. */
    if ((sens->control & desc->interrupt_enable_mask) != 0u &&
        desc->rtc_interrupt_mask != 0u && sens->conversion_done)
        sens->conversion_done(sens->conversion_ctx);
}

static void sens_fallback_write(flexe_sens_t *sens, uint32_t addr,
                                uint32_t value)
{
    if (sens->fallback_write)
        sens->fallback_write(sens->fallback_ctx, addr, value);
}

uint32_t flexe_sens_mmio_read(void *ctx, uint32_t addr)
{
    flexe_sens_t *sens = ctx;
    const flexe_sens_desc_t *desc = &sens->target->sens;
    if (addr < desc->base || addr - desc->base >= desc->register_size)
        return sens->fallback_read ?
            sens->fallback_read(sens->fallback_ctx, addr) : 0u;
    uint32_t offset = addr - desc->base;
    if (offset == desc->control_offset) {
        unsigned shift = field_shift(desc->output_mask);
        return sens->control |
               (sens->ready ? desc->ready_mask : 0u) |
               (((uint32_t)sens->output << shift) & desc->output_mask);
    }
    if (offset == desc->control2_offset) return sens->control2;
    if (offset == desc->clock_gate_offset) return sens->clock_gate;
    if (offset == desc->reset_offset) return sens->reset;
    return sens->fallback_read ?
        sens->fallback_read(sens->fallback_ctx, addr) : 0u;
}

void flexe_sens_mmio_write(void *ctx, uint32_t addr, uint32_t value)
{
    flexe_sens_t *sens = ctx;
    const flexe_sens_desc_t *desc = &sens->target->sens;
    if (addr < desc->base || addr - desc->base >= desc->register_size) {
        sens_fallback_write(sens, addr, value);
        return;
    }
    uint32_t offset = addr - desc->base;
    if (offset == desc->control_offset) {
        uint32_t known = desc->control_writable_mask |
                         desc->ready_mask | desc->output_mask;
        sens->control = value & desc->control_writable_mask;
        if ((value & ~known) != 0u)
            sens_fallback_write(sens, addr, value);
        sens_update_conversion(sens);
        return;
    }
    if (offset == desc->control2_offset) {
        sens->control2 = value & desc->control2_writable_mask;
        if ((value & ~desc->control2_writable_mask) != 0u)
            sens_fallback_write(sens, addr, value);
        sens_update_conversion(sens);
        return;
    }
    if (offset == desc->clock_gate_offset) {
        uint32_t old = sens->clock_gate;
        sens->clock_gate = value & desc->clock_gate_writable_mask;
        uint32_t unsupported =
            ((old ^ sens->clock_gate) &
             ~desc->clock_enable_mask) |
            (value & ~desc->clock_gate_writable_mask);
        if (unsupported != 0u)
            sens_fallback_write(sens, addr, value);
        sens_update_conversion(sens);
        return;
    }
    if (offset == desc->reset_offset) {
        uint32_t old = sens->reset;
        sens->reset = value & desc->reset_writable_mask;
        uint32_t unsupported =
            ((old ^ sens->reset) & ~desc->reset_mask) |
            (value & ~desc->reset_writable_mask);
        if (unsupported != 0u)
            sens_fallback_write(sens, addr, value);
        if ((sens->reset & desc->reset_mask) != 0u)
            sens_reset_temperature(sens);
        sens_update_conversion(sens);
        return;
    }
    sens_fallback_write(sens, addr, value);
}

flexe_sens_t *flexe_sens_create(
    xtensa_mem_t *mem, mmio_read_fn fallback_read,
    mmio_write_fn fallback_write, void *fallback_ctx,
    flexe_sens_conversion_fn conversion_done, void *conversion_ctx)
{
    if (!mem) return NULL;
    const flexe_target_desc_t *target = mem_target(mem);
    if (!sens_geometry_valid(target)) return NULL;

    flexe_sens_t *sens = calloc(1u, sizeof(*sens));
    if (!sens) return NULL;
    sens->mem = mem;
    sens->target = target;
    sens->fallback_read = fallback_read;
    sens->fallback_write = fallback_write;
    sens->fallback_ctx = fallback_ctx;
    sens->conversion_done = conversion_done;
    sens->conversion_ctx = conversion_ctx;
    const flexe_sens_desc_t *desc = &target->sens;
    sens->input_raw = desc->default_output;
    sens->clock_gate = desc->clock_gate_reset &
                       desc->clock_gate_writable_mask;
    sens->reset = desc->reset_reset & desc->reset_writable_mask;
    sens_reset_temperature(sens);

    if (mem_register_mmio_range(mem, desc->base, desc->register_size,
                                flexe_sens_mmio_read,
                                flexe_sens_mmio_write, sens) != 0) {
        free(sens);
        return NULL;
    }
    return sens;
}

void flexe_sens_destroy(flexe_sens_t *sens)
{
    if (!sens) return;
    const flexe_sens_desc_t *desc = &sens->target->sens;
    (void)mem_register_mmio_range(
        sens->mem, desc->base, desc->register_size,
        sens->fallback_read, sens->fallback_write, sens->fallback_ctx);
    free(sens);
}

void flexe_sens_set_temperature_raw(flexe_sens_t *sens, uint16_t raw)
{
    if (!sens) return;
    uint16_t maximum = sens_output_max(&sens->target->sens);
    sens->input_raw = raw > maximum ? maximum : raw;
}

uint16_t flexe_sens_temperature_raw(const flexe_sens_t *sens)
{
    return sens ? sens->input_raw : 0u;
}
