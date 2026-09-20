#include "sens.h"
#include "target.h"

#include "apb_saradc.h"
#include "regi2c.h"

#include <stdbool.h>
#include <stdlib.h>

/* S3 RTC SAR ADC MEASx_CTRL2: software pad bitmap, START pulse, and
 * hardware-owned DONE/DATA. See Espressif's esp32s3 sens_reg.h/adc_ll.h. */
#define SENS_ADC_PAD_FORCE  (1u << 31)
#define SENS_ADC_PAD_MASK   (0xFFFu << 19)
#define SENS_ADC_START_FORCE (1u << 18)
#define SENS_ADC_START     (1u << 17)
#define SENS_ADC_DONE      (1u << 16)
#define SENS_ADC_CONFIG    (SENS_ADC_PAD_FORCE | SENS_ADC_PAD_MASK | \
                            SENS_ADC_START_FORCE | SENS_ADC_START)

typedef struct {
    uint32_t reader;
    uint32_t measure;
    uint32_t mux;
    uint32_t atten;
    uint16_t input[12];
    uint16_t output;
    bool done;
} sens_adc_unit_t;

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
    flexe_sens_peripheral_fn peripheral_changed;
    void *peripheral_ctx;
    const flexe_regi2c_t *regi2c;
    const flexe_apb_saradc_t *apb_saradc;
    uint32_t adc_power;
    uint32_t adc_status_addr;
    sens_adc_unit_t adc[FLEXE_TARGET_SENS_ADC_UNIT_MAX];
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

static bool register_is_partitioned_by(
    uint32_t writable_mask, const uint32_t *fields, size_t count)
{
    uint32_t combined = 0u;
    for (size_t i = 0u; i < count; i++) {
        if (!one_bit(fields[i]) || (combined & fields[i]) != 0u)
            return false;
        combined |= fields[i];
    }
    return combined == writable_mask;
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
        (desc->clock_gate_reset &
         ~desc->clock_gate_writable_mask) != 0u ||
        (desc->reset_reset & ~desc->reset_writable_mask) != 0u)
        return false;

    const uint32_t clock_fields[] = {
        desc->io_mux_clock_enable_mask,
        desc->adc_clock_enable_mask,
        desc->temperature_clock_enable_mask,
        desc->rtc_i2c_clock_enable_mask,
    };
    const uint32_t reset_fields[] = {
        desc->adc_reset_mask,
        desc->temperature_reset_mask,
        desc->rtc_i2c_reset_mask,
        desc->coprocessor_reset_mask,
    };
    if (!register_is_partitioned_by(
            desc->clock_gate_writable_mask, clock_fields,
            sizeof(clock_fields) / sizeof(clock_fields[0])) ||
        !register_is_partitioned_by(
            desc->reset_writable_mask, reset_fields,
            sizeof(reset_fields) / sizeof(reset_fields[0])))
        return false;

    if (desc->rtc_interrupt_mask != 0u) {
        if (!one_bit(desc->rtc_interrupt_mask) ||
            !(target->capabilities & FLEXE_TARGET_CAP_RTC_CNTL_V1) ||
            (desc->rtc_interrupt_mask &
             ~target->rtc_cntl.interrupt_valid_mask) != 0u)
            return false;
    }
    if (desc->adc_unit_count > FLEXE_TARGET_SENS_ADC_UNIT_MAX ||
        (desc->adc_unit_count != 0u &&
         (desc->adc_channels_per_unit == 0u ||
          desc->adc_channels_per_unit > 12u ||
          desc->adc_output_mask == 0u ||
          (desc->adc_output_mask & (desc->adc_output_mask + 1u)) != 0u ||
          !register_offset_valid(desc->adc_power_offset,
                                 desc->register_size) ||
          !register_offset_valid(desc->adc_status_offset,
                                 desc->register_size) ||
          (desc->adc_clock_enable_mask &
           ~desc->clock_gate_writable_mask) != 0u ||
          (desc->adc_reset_mask & ~desc->reset_writable_mask) != 0u ||
          desc->adc_power_writable_mask == 0u)))
        return false;
    uint16_t adc_offsets[2u + FLEXE_TARGET_SENS_ADC_UNIT_MAX * 4u];
    unsigned adc_offset_count = 0u;
    if (desc->adc_unit_count != 0u) {
        for (unsigned j = 0u; j < 4u; j++)
            if (desc->adc_power_offset == offsets[j] ||
                desc->adc_status_offset == offsets[j]) return false;
        if (desc->adc_power_offset == desc->adc_status_offset ||
            (desc->adc_status_writable_mask & 0xFFC00000u) != 0u)
            return false;
        adc_offsets[adc_offset_count++] = desc->adc_power_offset;
        adc_offsets[adc_offset_count++] = desc->adc_status_offset;
    }
    for (unsigned unit = 0u; unit < desc->adc_unit_count; unit++) {
        const flexe_sens_adc_unit_desc_t *adc = &desc->adc_unit[unit];
        const uint16_t unit_offsets[] = {
            adc->reader_offset, adc->measure_offset,
            adc->mux_offset, adc->atten_offset,
        };
        if (!one_bit(adc->reader_invert_mask) ||
            (adc->reader_invert_mask & ~adc->reader_writable_mask) != 0u ||
            (adc->reader_reset & ~adc->reader_writable_mask) != 0u ||
            (adc->mux_rtc_block_mask & ~adc->mux_writable_mask) != 0u ||
            (adc->mux_rtc_bypass_mask & ~adc->mux_writable_mask) != 0u ||
            (adc->mux_unsupported_mask & ~adc->mux_writable_mask) != 0u ||
            (adc->arbiter_controlled &&
             !(target->capabilities & FLEXE_TARGET_CAP_APB_SARADC_V1)) ||
            (adc->calibration_ground_mask != 0u &&
             (!one_bit(adc->calibration_ground_mask) ||
              !(target->capabilities & FLEXE_TARGET_CAP_REGI2C) ||
              target->regi2c.slave_shift >= 32u ||
              target->regi2c.address_shift >= 32u ||
              desc->adc_calibration_slave >
                  (target->regi2c.slave_mask >>
                   target->regi2c.slave_shift) ||
              desc->adc_calibration_address >
                  (target->regi2c.address_mask >>
                   target->regi2c.address_shift))))
            return false;
        for (unsigned i = 0u; i < 4u; i++) {
            if (!register_offset_valid(unit_offsets[i],
                                       desc->register_size))
                return false;
            for (unsigned j = 0u; j < 4u; j++)
                if (unit_offsets[i] == offsets[j]) return false;
            for (unsigned j = 0u; j < adc_offset_count; j++)
                if (unit_offsets[i] == adc_offsets[j]) return false;
            adc_offsets[adc_offset_count++] = unit_offsets[i];
        }
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
    return (sens->clock_gate & desc->temperature_clock_enable_mask) != 0u &&
           (sens->reset & desc->temperature_reset_mask) == 0u &&
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

bool flexe_sens_peripheral_state(
    const flexe_sens_t *sens, flexe_sens_peripheral_state_t *state)
{
    if (!sens || !state) return false;
    const flexe_sens_desc_t *desc = &sens->target->sens;
    *state = (flexe_sens_peripheral_state_t) {
        .io_mux_clock_enabled =
            (sens->clock_gate & desc->io_mux_clock_enable_mask) != 0u,
        .adc_clock_enabled =
            (sens->clock_gate & desc->adc_clock_enable_mask) != 0u,
        .temperature_clock_enabled =
            (sens->clock_gate &
             desc->temperature_clock_enable_mask) != 0u,
        .rtc_i2c_clock_enabled =
            (sens->clock_gate & desc->rtc_i2c_clock_enable_mask) != 0u,
        .adc_reset_asserted =
            (sens->reset & desc->adc_reset_mask) != 0u,
        .temperature_reset_asserted =
            (sens->reset & desc->temperature_reset_mask) != 0u,
        .rtc_i2c_reset_asserted =
            (sens->reset & desc->rtc_i2c_reset_mask) != 0u,
        .coprocessor_reset_asserted =
            (sens->reset & desc->coprocessor_reset_mask) != 0u,
    };
    return true;
}

static void sens_publish_peripheral(flexe_sens_t *sens)
{
    if (!sens || !sens->peripheral_changed) return;
    flexe_sens_peripheral_state_t state;
    if (flexe_sens_peripheral_state(sens, &state))
        sens->peripheral_changed(sens->peripheral_ctx, &state);
}

static void sens_adc_reset(flexe_sens_t *sens)
{
    const flexe_sens_desc_t *desc = &sens->target->sens;
    sens->adc_power = 0u;
    sens->adc_status_addr = 0u;
    for (unsigned i = 0u; i < desc->adc_unit_count; i++) {
        sens_adc_unit_t *unit = &sens->adc[i];
        unit->reader = desc->adc_unit[i].reader_reset;
        unit->measure = 0u;
        unit->mux = 0u;
        unit->atten = UINT32_MAX;
        unit->output = 0u;
        unit->done = false;
    }
}

static bool sens_adc_powered(const flexe_sens_t *sens)
{
    const flexe_sens_desc_t *desc = &sens->target->sens;
    /* FORCE_XPD_SAR=2 forces off; 0 delegates power to the RTC FSM. */
    return (sens->clock_gate & desc->adc_clock_enable_mask) != 0u &&
           (sens->reset & desc->adc_reset_mask) == 0u &&
           ((sens->adc_power >> 29) & 3u) != 2u;
}

static bool sens_adc_internal_ground(const flexe_sens_t *sens,
                                     unsigned index)
{
    const flexe_sens_desc_t *desc = &sens->target->sens;
    uint8_t value = 0u;
    return desc->adc_unit[index].calibration_ground_mask != 0u &&
           flexe_regi2c_register_read(
               sens->regi2c, desc->adc_calibration_slave,
               desc->adc_calibration_address, &value) &&
           (value & desc->adc_unit[index].calibration_ground_mask) != 0u;
}

static void sens_adc_measure_write(flexe_sens_t *sens, unsigned index,
                                   uint32_t addr, uint32_t value)
{
    const flexe_sens_desc_t *desc = &sens->target->sens;
    const flexe_sens_adc_unit_desc_t *model = &desc->adc_unit[index];
    sens_adc_unit_t *unit = &sens->adc[index];
    uint32_t old = unit->measure;
    unit->measure = value & SENS_ADC_CONFIG;
    if ((unit->measure & SENS_ADC_START) == 0u) {
        unit->done = false;
        return;
    }
    if ((old & SENS_ADC_START) != 0u) return;

    uint32_t pads = (unit->measure & SENS_ADC_PAD_MASK) >> 19u;
    bool one_pad = one_bit(pads);
    bool internal_ground = sens_adc_internal_ground(sens, index);
    bool rtc_control = (unit->mux & model->mux_rtc_block_mask) == 0u;
    bool rtc_granted = !model->arbiter_controlled ||
        (unit->mux & model->mux_rtc_bypass_mask) != 0u ||
        (sens->apb_saradc &&
         flexe_apb_saradc_rtc_granted(sens->apb_saradc));
    if ((!internal_ground &&
         (!one_pad || pads >= (1u << desc->adc_channels_per_unit))) ||
        (unit->measure & (SENS_ADC_PAD_FORCE | SENS_ADC_START_FORCE)) !=
            (SENS_ADC_PAD_FORCE | SENS_ADC_START_FORCE) ||
        !rtc_control || !rtc_granted || !sens_adc_powered(sens)) {
        /* Do not claim a conversion that the modeled RTC path cannot do. */
        sens_fallback_write(sens, addr, value);
        return;
    }
    unsigned channel = 0u;
    if (!internal_ground)
        while ((pads & (1u << channel)) == 0u) channel++;
    uint16_t sample = internal_ground ? 0u :
        unit->input[channel] & desc->adc_output_mask;
    if ((unit->reader & model->reader_invert_mask) != 0u)
        sample ^= desc->adc_output_mask;
    unit->output = sample;
    unit->done = true;
}

static bool sens_adc_read(flexe_sens_t *sens, uint32_t offset,
                          uint32_t *value)
{
    const flexe_sens_desc_t *desc = &sens->target->sens;
    if (desc->adc_unit_count == 0u) return false;
    if (offset == desc->adc_power_offset) {
        *value = sens->adc_power;
        return true;
    }
    if (offset == desc->adc_status_offset) {
        /* MEAS_STATUS is idle after synchronous fast-mode conversions;
         * the low two slave-address fields are software owned. */
        *value = sens->adc_status_addr;
        return true;
    }
    for (unsigned i = 0u; i < desc->adc_unit_count; i++) {
        const flexe_sens_adc_unit_desc_t *model = &desc->adc_unit[i];
        const sens_adc_unit_t *unit = &sens->adc[i];
        if (offset == model->reader_offset) *value = unit->reader;
        else if (offset == model->measure_offset)
            *value = unit->measure |
                     (unit->done ? SENS_ADC_DONE : 0u) | unit->output;
        else if (offset == model->mux_offset) *value = unit->mux;
        else if (offset == model->atten_offset) *value = unit->atten;
        else continue;
        return true;
    }
    return false;
}

static bool sens_adc_write(flexe_sens_t *sens, uint32_t offset,
                           uint32_t addr, uint32_t value)
{
    const flexe_sens_desc_t *desc = &sens->target->sens;
    if (desc->adc_unit_count == 0u) return false;
    if (offset == desc->adc_power_offset) {
        sens->adc_power = value & desc->adc_power_writable_mask;
        if ((value & ~desc->adc_power_writable_mask) != 0u)
            sens_fallback_write(sens, addr, value);
        return true;
    }
    if (offset == desc->adc_status_offset) {
        sens->adc_status_addr = value & desc->adc_status_writable_mask;
        if ((value & ~desc->adc_status_writable_mask) != 0u)
            sens_fallback_write(sens, addr, value);
        return true;
    }
    for (unsigned i = 0u; i < desc->adc_unit_count; i++) {
        const flexe_sens_adc_unit_desc_t *model = &desc->adc_unit[i];
        sens_adc_unit_t *unit = &sens->adc[i];
        if (offset == model->reader_offset) {
            unit->reader = value & model->reader_writable_mask;
            if ((value & ~model->reader_writable_mask) != 0u)
                sens_fallback_write(sens, addr, value);
        } else if (offset == model->measure_offset) {
            sens_adc_measure_write(sens, i, addr, value);
        } else if (offset == model->mux_offset) {
            unit->mux = value & model->mux_writable_mask;
            if ((value & ~model->mux_writable_mask) != 0u ||
                (unit->mux & model->mux_unsupported_mask) != 0u ||
                ((unit->mux & model->mux_rtc_block_mask) != 0u &&
                 !sens->apb_saradc))
                sens_fallback_write(sens, addr, value);
        } else if (offset == model->atten_offset) {
            unit->atten = value;
        } else continue;
        return true;
    }
    return false;
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
    uint32_t adc_value = 0u;
    if (sens_adc_read(sens, offset, &adc_value)) return adc_value;
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
        if ((value & ~desc->clock_gate_writable_mask) != 0u)
            sens_fallback_write(sens, addr, value);
        if (old != sens->clock_gate) sens_publish_peripheral(sens);
        sens_update_conversion(sens);
        return;
    }
    if (offset == desc->reset_offset) {
        uint32_t old = sens->reset;
        sens->reset = value & desc->reset_writable_mask;
        if ((value & ~desc->reset_writable_mask) != 0u)
            sens_fallback_write(sens, addr, value);
        if ((sens->reset & desc->temperature_reset_mask) != 0u)
            sens_reset_temperature(sens);
        if ((sens->reset & desc->adc_reset_mask) != 0u)
            sens_adc_reset(sens);
        if (old != sens->reset) sens_publish_peripheral(sens);
        sens_update_conversion(sens);
        return;
    }
    if (sens_adc_write(sens, offset, addr, value)) return;
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
    sens_adc_reset(sens);

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

void flexe_sens_attach_regi2c(flexe_sens_t *sens,
                              const flexe_regi2c_t *regi2c)
{
    if (sens) sens->regi2c = regi2c;
}

void flexe_sens_attach_apb_saradc(flexe_sens_t *sens,
                                  const flexe_apb_saradc_t *apb_saradc)
{
    if (sens) sens->apb_saradc = apb_saradc;
}

void flexe_sens_set_peripheral_listener(
    flexe_sens_t *sens, flexe_sens_peripheral_fn fn, void *ctx)
{
    if (!sens) return;
    sens->peripheral_changed = fn;
    sens->peripheral_ctx = fn ? ctx : NULL;
    sens_publish_peripheral(sens);
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

void flexe_sens_set_adc_raw(flexe_sens_t *sens, unsigned unit,
                            unsigned channel, uint16_t raw)
{
    if (!sens || unit >= sens->target->sens.adc_unit_count ||
        channel >= sens->target->sens.adc_channels_per_unit) return;
    sens->adc[unit].input[channel] =
        raw & sens->target->sens.adc_output_mask;
}
