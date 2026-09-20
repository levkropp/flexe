#include "rtc_cntl.h"
#include "target.h"

#include "xtensa.h"

#include <limits.h>
#include <stdbool.h>
#include <stdlib.h>

typedef struct {
    uint64_t cycles;
    uint64_t core_cycles[2];
    uint32_t last_ccount[2];
    bool valid[2];
} rtc_clock_t;

struct flexe_rtc_cntl {
    xtensa_mem_t *mem;
    const flexe_target_desc_t *target;
    mmio_read_fn fallback_read;
    mmio_write_fn fallback_write;
    void *fallback_ctx;
    flexe_rtc_cntl_state_fn state_changed;
    void *state_ctx;
    flexe_rtc_cntl_irq_fn irq_changed;
    void *irq_ctx;
    flexe_rtc_cntl_reset_fn reset_requested;
    void *reset_ctx;
    xtensa_cpu_t *cpu[2];
    rtc_clock_t clock;
    uint64_t last_clock_cycles;
    uint64_t tick_denominator;
    uint64_t tick_remainder;
    uint64_t counter;
    uint64_t latched_counter;
    uint64_t sleep_alarm;
    uint32_t sleep_state;
    uint32_t wakeup_enable;
    uint32_t wakeup_cause;
    uint32_t regulator_control;
    uint32_t rtc_power_control;
    uint32_t digital_iso_control;
    uint32_t digital_power;
    uint32_t reset_state;
    uint32_t ext_wakeup_config;
    uint32_t ext1_select;
    uint32_t ext1_status;
    uint32_t brownout_config;
    bool sleep_alarm_armed;
    bool sleep_requested;
    uint32_t clock_conf;
    uint32_t analog_conf;
    uint32_t usb_conf;
    uint32_t date;
    uint32_t interrupt_enable;
    uint32_t interrupt_raw;
    bool interrupt_level;
    uint32_t wdt_config[FLEXE_TARGET_RTC_WDT_CONFIG_MAX];
    uint32_t sequence_reg[FLEXE_TARGET_RTC_SEQUENCE_REGISTER_MAX];
    uint32_t config_reg[FLEXE_TARGET_RTC_CONFIG_REGISTER_MAX];
    uint32_t wdt_write_protect;
    uint64_t wdt_stage_ticks;
    uint8_t wdt_stage;
    uint32_t store[FLEXE_TARGET_RTC_STORE_MAX];
    uint32_t rtc_pad_hold;
    uint32_t digital_pad_hold;
    uint32_t cpu_stall_options;
    uint32_t cpu_stall_high;
    flexe_rtc_cntl_pad_hold_fn pad_hold_changed;
    void *pad_hold_ctx;
    flexe_rtc_cntl_domain_state_fn digital_domain_changed;
    void *digital_domain_ctx;
    flexe_rtc_cntl_domain_state_fn rtc_domain_changed;
    void *rtc_domain_ctx;
    flexe_rtc_cntl_supply_state_fn supply_changed;
    void *supply_ctx;
    flexe_rtc_cntl_control_state_fn control_changed;
    void *control_ctx;
};

static bool rtc_offset_valid(uint16_t offset, uint32_t register_size);
static bool rtc_interrupt_offset(const flexe_rtc_cntl_desc_t *desc,
                                 uint16_t offset);
static bool rtc_wdt_offset(const flexe_rtc_cntl_desc_t *desc,
                           uint16_t offset);

static uint32_t rtc_stall_low_mask(const flexe_rtc_cntl_desc_t *desc)
{
    return (3u << desc->cpu_stall_low_shift[0]) |
           (3u << desc->cpu_stall_low_shift[1]);
}

static uint32_t rtc_stall_high_mask(const flexe_rtc_cntl_desc_t *desc)
{
    return (0x3Fu << desc->cpu_stall_high_shift[0]) |
           (0x3Fu << desc->cpu_stall_high_shift[1]);
}

static uint32_t rtc_hold_field_mask(unsigned first_bit, unsigned count)
{
    if (count == 0u) return 0u;
    return (UINT32_MAX >> (32u - count)) << first_bit;
}

static uint32_t rtc_hold_valid_mask(const flexe_target_desc_t *target,
                                    unsigned first_gpio,
                                    unsigned first_bit, unsigned count)
{
    return ((uint32_t)(target->gpio.valid_gpio_mask >>
                       first_gpio) << first_bit) &
           rtc_hold_field_mask(first_bit, count);
}

static uint64_t rtc_hold_pins(uint32_t value, unsigned first_gpio,
                              unsigned first_bit, unsigned count)
{
    if (count == 0u) return 0u;
    return (uint64_t)((value & rtc_hold_field_mask(first_bit, count)) >>
                      first_bit) << first_gpio;
}

static uint64_t rtc_all_pad_hold_pins(const flexe_rtc_cntl_t *rtc)
{
    const flexe_rtc_cntl_desc_t *desc = &rtc->target->rtc_cntl;
    uint64_t rtc_global =
        (rtc->rtc_power_control & desc->rtc_pad_force_hold_mask) != 0u ?
        ((UINT64_C(1) << rtc->target->rtc_io.gpio_count) - 1u) : 0u;
    uint64_t digital_global =
        (rtc->digital_iso_control & desc->digital_pad_force_hold_mask) != 0u &&
        (rtc->digital_iso_control & desc->digital_pad_force_unhold_mask) == 0u ?
        rtc_hold_pins(UINT32_MAX,
                      desc->digital_pad_hold_first_gpio,
                      desc->digital_pad_hold_first_bit,
                      desc->digital_pad_hold_count) &
        rtc->target->gpio.valid_gpio_mask : 0u;
    return rtc_global | digital_global |
           rtc_hold_pins(rtc->rtc_pad_hold,
                         desc->rtc_pad_hold_first_gpio,
                         desc->rtc_pad_hold_first_bit,
                         desc->rtc_pad_hold_count) |
           rtc_hold_pins(rtc->digital_pad_hold,
                         desc->digital_pad_hold_first_gpio,
                         desc->digital_pad_hold_first_bit,
                         desc->digital_pad_hold_count);
}

static bool rtc_hold_geometry_valid(const flexe_target_desc_t *target,
                                    uint16_t offset, unsigned first_gpio,
                                    unsigned first_bit, unsigned count)
{
    const flexe_rtc_cntl_desc_t *desc = &target->rtc_cntl;
    return count == 0u ||
           (rtc_offset_valid(offset, desc->register_size) &&
            first_bit < 32u && count <= 32u - first_bit &&
            first_gpio < 64u && count <= 64u - first_gpio &&
            (target->capabilities & FLEXE_TARGET_CAP_GPIO_V1) != 0u &&
            first_gpio + count <= target->gpio.gpio_count &&
            offset != desc->clock_conf_offset &&
            offset != desc->reset_state_offset &&
            offset != desc->time_update_offset &&
            offset != desc->time_low_offset &&
            offset != desc->time_high_offset &&
            offset != desc->date_offset &&
            !rtc_interrupt_offset(desc, offset) &&
            !rtc_wdt_offset(desc, offset));
}

static bool rtc_pad_hold_offset(const flexe_rtc_cntl_desc_t *desc,
                                uint16_t offset)
{
    return (desc->rtc_pad_hold_count != 0u &&
            offset == desc->rtc_pad_hold_offset) ||
           (desc->digital_pad_hold_count != 0u &&
            offset == desc->digital_pad_hold_offset);
}

static bool rtc_offset_valid(uint16_t offset, uint32_t register_size)
{
    return (offset & 3u) == 0u &&
           offset <= register_size - sizeof(uint32_t);
}

static bool rtc_interrupt_offset(const flexe_rtc_cntl_desc_t *desc,
                                 uint16_t offset)
{
    return offset == desc->interrupt_enable_offset ||
           offset == desc->interrupt_raw_offset ||
           offset == desc->interrupt_status_offset ||
           offset == desc->interrupt_clear_offset;
}

static bool rtc_wdt_offset(const flexe_rtc_cntl_desc_t *desc,
                           uint16_t offset)
{
    if (offset == desc->wdt_feed_offset ||
        offset == desc->wdt_write_protect_offset)
        return true;
    for (unsigned i = 0u; i < FLEXE_TARGET_RTC_WDT_CONFIG_MAX; i++)
        if (offset == desc->wdt_config_offset[i]) return true;
    return false;
}

static bool rtc_optional_one_bit(uint32_t value)
{
    return value == 0u || (value & (value - 1u)) == 0u;
}

static bool rtc_supply_powered(
    uint32_t value, const flexe_rtc_supply_desc_t *supply)
{
    if ((value & supply->force_power_down_mask) != 0u) return false;
    if ((value & supply->force_power_up_mask) != 0u) return true;
    return true;
}

static bool rtc_force_pair_set(
    uint32_t value, const flexe_rtc_force_pair_desc_t *pair)
{
    if ((value & pair->force_set_mask) != 0u) return true;
    if ((value & pair->force_clear_mask) != 0u) return false;
    return false;
}

static bool rtc_force_pair_valid(
    const flexe_rtc_force_pair_desc_t *pair, uint32_t reset,
    uint32_t writable_mask)
{
    uint32_t fields = pair->force_clear_mask | pair->force_set_mask;
    return rtc_optional_one_bit(pair->force_clear_mask) &&
           pair->force_clear_mask != 0u &&
           rtc_optional_one_bit(pair->force_set_mask) &&
           pair->force_set_mask != 0u &&
           pair->force_clear_mask != pair->force_set_mask &&
           (fields & ~writable_mask) == 0u &&
           !((reset & pair->force_clear_mask) != 0u &&
             (reset & pair->force_set_mask) != 0u);
}

static bool rtc_supply_array_valid(
    const flexe_rtc_supply_desc_t *supplies, unsigned count,
    unsigned maximum, uint32_t reset, uint32_t writable_mask,
    uint32_t *fields_out)
{
    if (count == 0u || count > maximum) return false;
    uint32_t fields = 0u;
    for (unsigned i = 0u; i < count; i++) {
        const flexe_rtc_supply_desc_t *supply = &supplies[i];
        uint32_t pair = supply->force_power_up_mask |
                        supply->force_power_down_mask;
        if (!rtc_optional_one_bit(supply->force_power_up_mask) ||
            supply->force_power_up_mask == 0u ||
            !rtc_optional_one_bit(supply->force_power_down_mask) ||
            supply->force_power_down_mask == 0u ||
            supply->force_power_up_mask == supply->force_power_down_mask ||
            (pair & fields) != 0u ||
            (pair & ~writable_mask) != 0u ||
            ((reset & supply->force_power_up_mask) != 0u &&
             (reset & supply->force_power_down_mask) != 0u))
            return false;
        fields |= pair;
    }
    if (fields_out) *fields_out = fields;
    return true;
}

static bool rtc_force_pair_array_valid(
    const flexe_rtc_force_pair_desc_t *pairs, unsigned count,
    unsigned maximum, uint32_t reset, uint32_t writable_mask,
    uint32_t *fields_out)
{
    if (count == 0u || count > maximum) return false;
    uint32_t fields = 0u;
    for (unsigned i = 0u; i < count; i++) {
        uint32_t pair = pairs[i].force_clear_mask |
                        pairs[i].force_set_mask;
        if (!rtc_force_pair_valid(&pairs[i], reset, writable_mask) ||
            (pair & fields) != 0u)
            return false;
        fields |= pair;
    }
    if (fields_out) *fields_out = fields;
    return true;
}

static bool rtc_masks_disjoint(const uint32_t *masks, unsigned count)
{
    uint32_t used = 0u;
    for (unsigned i = 0u; i < count; i++) {
        if ((masks[i] & used) != 0u) return false;
        used |= masks[i];
    }
    return true;
}

static bool rtc_options_controls_valid(
    const flexe_rtc_cntl_desc_t *desc)
{
    uint32_t supply_fields = 0u;
    uint32_t isolation_fields = 0u;
    uint32_t reset_fields = 0u;
    uint32_t wait = desc->options_xtal_wait_mask;
    if (desc->cpu_stall_options_writable_mask == 0u ||
        (desc->cpu_stall_options_reset &
         ~desc->cpu_stall_options_writable_mask) != 0u ||
        desc->options_xtal_wait_shift >= 32u ||
        wait == 0u ||
        (wait & ~desc->cpu_stall_options_writable_mask) != 0u ||
        (wait & ((1u << desc->options_xtal_wait_shift) - 1u)) != 0u)
        return false;
    uint32_t wait_value = wait >> desc->options_xtal_wait_shift;
    if (wait_value > UINT8_MAX ||
        (wait_value & (wait_value + 1u)) != 0u ||
        !rtc_supply_array_valid(
            desc->options_supply, desc->options_supply_count,
            FLEXE_TARGET_RTC_SUPPLY_MAX, desc->cpu_stall_options_reset,
            desc->cpu_stall_options_writable_mask, &supply_fields) ||
        !rtc_force_pair_array_valid(
            desc->options_isolation, desc->options_isolation_count,
            FLEXE_TARGET_RTC_OPTION_DOMAIN_MAX,
            desc->cpu_stall_options_reset,
            desc->cpu_stall_options_writable_mask, &isolation_fields) ||
        !rtc_force_pair_array_valid(
            desc->options_reset, desc->options_reset_count,
            FLEXE_TARGET_RTC_OPTION_DOMAIN_MAX,
            desc->cpu_stall_options_reset,
            desc->cpu_stall_options_writable_mask, &reset_fields))
        return false;
    const uint32_t masks[] = {
        rtc_stall_low_mask(desc), wait, supply_fields,
        isolation_fields, reset_fields,
    };
    return rtc_masks_disjoint(masks, sizeof(masks) / sizeof(masks[0])) &&
           (rtc_stall_low_mask(desc) | wait | supply_fields |
            isolation_fields | reset_fields) ==
               desc->cpu_stall_options_writable_mask;
}

static bool rtc_analog_controls_valid(
    const flexe_rtc_cntl_desc_t *desc)
{
    uint32_t fields = 0u;
    if (desc->analog_control_count == 0u ||
        desc->analog_control_count > FLEXE_TARGET_RTC_ANALOG_CONTROL_MAX ||
        !rtc_supply_array_valid(
            &desc->analog_reset_por_supply, 1u, 1u,
            desc->analog_conf_reset, desc->analog_conf_writable_mask,
            &fields))
        return false;

    for (unsigned i = 0u; i < desc->analog_control_count; i++) {
        uint32_t mask = desc->analog_control_mask[i];
        if (!rtc_optional_one_bit(mask) || mask == 0u ||
            (mask & fields) != 0u ||
            (mask & ~desc->analog_conf_writable_mask) != 0u)
            return false;
        fields |= mask;
    }
    return fields == desc->analog_conf_writable_mask &&
           (desc->sar_i2c_power_mask & fields) ==
               desc->sar_i2c_power_mask;
}

static uint32_t rtc_digital_power_fields(
    const flexe_rtc_cntl_desc_t *desc)
{
    uint32_t fields = 0u;
    for (unsigned i = 0u; i < desc->digital_domain_count; i++) {
        const flexe_rtc_digital_domain_desc_t *domain =
            &desc->digital_domain[i];
        fields |= domain->sleep_power_down_mask |
                  domain->force_power_up_mask |
                  domain->force_power_down_mask;
    }
    return fields;
}

static uint32_t rtc_regulator_supply_fields(
    const flexe_rtc_cntl_desc_t *desc)
{
    uint32_t fields = 0u;
    for (unsigned i = 0u; i < desc->regulator_supply_count; i++) {
        const flexe_rtc_supply_desc_t *supply =
            &desc->regulator_supply[i];
        fields |= supply->force_power_up_mask |
                  supply->force_power_down_mask;
    }
    return fields;
}

static bool rtc_regulator_supplies_valid(
    const flexe_rtc_cntl_desc_t *desc)
{
    if (desc->regulator_supply_count == 0u ||
        desc->regulator_supply_count > FLEXE_TARGET_RTC_SUPPLY_MAX ||
        desc->regulator_writable_mask == 0u ||
        (desc->regulator_reset & ~desc->regulator_writable_mask) != 0u)
        return false;

    uint32_t fields = 0u;
    for (unsigned i = 0u; i < desc->regulator_supply_count; i++) {
        const flexe_rtc_supply_desc_t *supply =
            &desc->regulator_supply[i];
        uint32_t pair = supply->force_power_up_mask |
                        supply->force_power_down_mask;
        if (!rtc_optional_one_bit(supply->force_power_up_mask) ||
            supply->force_power_up_mask == 0u ||
            !rtc_optional_one_bit(supply->force_power_down_mask) ||
            supply->force_power_down_mask == 0u ||
            supply->force_power_up_mask == supply->force_power_down_mask ||
            (pair & fields) != 0u ||
            (pair & ~desc->regulator_writable_mask) != 0u ||
            ((desc->regulator_reset & supply->force_power_up_mask) != 0u &&
             (desc->regulator_reset & supply->force_power_down_mask) != 0u))
            return false;
        fields |= pair;
    }
    return true;
}

static uint32_t rtc_power_domain_fields(
    const flexe_rtc_cntl_desc_t *desc)
{
    uint32_t fields = desc->rtc_pad_force_hold_mask;
    for (unsigned i = 0u; i < desc->rtc_power_domain_count; i++) {
        const flexe_rtc_power_domain_desc_t *domain =
            &desc->rtc_power_domain[i];
        fields |= domain->sleep_power_down_mask |
                  domain->force_power_up_mask |
                  domain->force_power_down_mask |
                  domain->follow_cpu_mask |
                  domain->force_noiso_mask |
                  domain->force_iso_mask;
    }
    return fields;
}

static bool rtc_power_domains_valid(const flexe_rtc_cntl_desc_t *desc)
{
    if (desc->rtc_power_domain_count == 0u ||
        desc->rtc_power_domain_count > FLEXE_TARGET_RTC_POWER_DOMAIN_MAX ||
        desc->rtc_power_writable_mask == 0u ||
        (desc->rtc_power_reset & ~desc->rtc_power_writable_mask) != 0u ||
        !rtc_optional_one_bit(desc->rtc_pad_force_hold_mask) ||
        desc->rtc_pad_force_hold_mask == 0u ||
        (desc->rtc_pad_force_hold_mask &
         ~desc->rtc_power_writable_mask) != 0u)
        return false;

    uint32_t fields = desc->rtc_pad_force_hold_mask;
    bool follows_cpu = false;
    for (unsigned i = 0u; i < desc->rtc_power_domain_count; i++) {
        const flexe_rtc_power_domain_desc_t *domain =
            &desc->rtc_power_domain[i];
        uint32_t power = domain->sleep_power_down_mask |
                         domain->force_power_up_mask |
                         domain->force_power_down_mask |
                         domain->follow_cpu_mask;
        uint32_t iso = domain->force_noiso_mask | domain->force_iso_mask;
        uint32_t all = power | iso;
        const uint32_t masks[] = {
            domain->sleep_power_down_mask,
            domain->force_power_up_mask,
            domain->force_power_down_mask,
            domain->follow_cpu_mask,
            domain->force_noiso_mask,
            domain->force_iso_mask,
        };
        if (!rtc_optional_one_bit(domain->sleep_power_down_mask) ||
            !rtc_optional_one_bit(domain->force_power_up_mask) ||
            domain->force_power_up_mask == 0u ||
            !rtc_optional_one_bit(domain->force_power_down_mask) ||
            domain->force_power_down_mask == 0u ||
            domain->force_power_up_mask == domain->force_power_down_mask ||
            !rtc_optional_one_bit(domain->follow_cpu_mask) ||
            !rtc_optional_one_bit(domain->force_noiso_mask) ||
            !rtc_optional_one_bit(domain->force_iso_mask) ||
            ((domain->force_noiso_mask == 0u) !=
             (domain->force_iso_mask == 0u)) ||
            (domain->force_noiso_mask != 0u &&
             domain->force_noiso_mask == domain->force_iso_mask) ||
            !rtc_masks_disjoint(masks,
                                sizeof(masks) / sizeof(masks[0])) ||
            (all & fields) != 0u ||
            (all & ~desc->rtc_power_writable_mask) != 0u ||
            ((desc->rtc_power_reset & domain->force_power_up_mask) != 0u &&
             (desc->rtc_power_reset & domain->force_power_down_mask) != 0u) ||
            ((desc->rtc_power_reset & domain->force_noiso_mask) != 0u &&
             (desc->rtc_power_reset & domain->force_iso_mask) != 0u))
            return false;
        fields |= all;
        follows_cpu |= domain->follow_cpu_mask != 0u;
    }
    if (fields != desc->rtc_power_writable_mask)
        return false;
    if (follows_cpu)
        return desc->rtc_follow_cpu_domain != 0u &&
               desc->rtc_follow_cpu_domain <= desc->digital_domain_count;
    return desc->rtc_follow_cpu_domain == 0u;
}

static bool rtc_digital_domains_valid(const flexe_rtc_cntl_desc_t *desc)
{
    if (desc->digital_domain_count == 0u ||
        desc->digital_domain_count > FLEXE_TARGET_RTC_DIGITAL_DOMAIN_MAX ||
        desc->digital_power_writable_mask == 0u ||
        (desc->digital_power_reset &
         ~desc->digital_power_writable_mask) != 0u ||
        desc->digital_iso_writable_mask == 0u ||
        (desc->digital_iso_writable_mask &
         desc->digital_iso_read_only_mask) != 0u ||
        (desc->digital_iso_strobe_mask &
         ~desc->digital_iso_writable_mask) != 0u ||
        (desc->digital_iso_reset &
         ~(desc->digital_iso_writable_mask |
           desc->digital_iso_read_only_mask)) != 0u ||
        (desc->digital_iso_reset & desc->digital_iso_strobe_mask) != 0u ||
        !rtc_optional_one_bit(desc->digital_iso_read_only_mask) ||
        desc->digital_iso_read_only_mask == 0u ||
        !rtc_optional_one_bit(desc->digital_iso_strobe_mask) ||
        desc->digital_iso_strobe_mask == 0u ||
        !rtc_optional_one_bit(desc->digital_pad_force_hold_mask) ||
        desc->digital_pad_force_hold_mask == 0u ||
        !rtc_optional_one_bit(desc->digital_pad_force_unhold_mask) ||
        desc->digital_pad_force_unhold_mask == 0u ||
        !rtc_optional_one_bit(desc->digital_pad_autohold_enable_mask) ||
        desc->digital_pad_autohold_enable_mask == 0u ||
        !rtc_force_pair_valid(&desc->digital_pad_isolation,
                              desc->digital_iso_reset,
                              desc->digital_iso_writable_mask) ||
        !rtc_force_pair_valid(&desc->digital_isolation,
                              desc->digital_iso_reset,
                              desc->digital_iso_writable_mask))
        return false;

    uint32_t power_fields = 0u;
    uint32_t iso_fields = 0u;
    for (unsigned i = 0u; i < desc->digital_domain_count; i++) {
        const flexe_rtc_digital_domain_desc_t *domain =
            &desc->digital_domain[i];
        uint32_t power = domain->sleep_power_down_mask |
                         domain->force_power_up_mask |
                         domain->force_power_down_mask;
        uint32_t iso = domain->force_noiso_mask | domain->force_iso_mask;
        const uint32_t power_masks[] = {
            domain->sleep_power_down_mask,
            domain->force_power_up_mask,
            domain->force_power_down_mask,
        };
        const uint32_t iso_masks[] = {
            domain->force_noiso_mask,
            domain->force_iso_mask,
        };
        if (!rtc_optional_one_bit(domain->sleep_power_down_mask) ||
            !rtc_optional_one_bit(domain->force_power_up_mask) ||
            domain->force_power_up_mask == 0u ||
            !rtc_optional_one_bit(domain->force_power_down_mask) ||
            domain->force_power_down_mask == 0u ||
            domain->force_power_up_mask == domain->force_power_down_mask ||
            !rtc_optional_one_bit(domain->force_noiso_mask) ||
            !rtc_optional_one_bit(domain->force_iso_mask) ||
            ((domain->force_noiso_mask == 0u) !=
             (domain->force_iso_mask == 0u)) ||
            (domain->force_noiso_mask != 0u &&
             domain->force_noiso_mask == domain->force_iso_mask) ||
            !rtc_masks_disjoint(
                power_masks,
                sizeof(power_masks) / sizeof(power_masks[0])) ||
            !rtc_masks_disjoint(iso_masks,
                                sizeof(iso_masks) / sizeof(iso_masks[0])) ||
            (power & power_fields) != 0u ||
            (iso & iso_fields) != 0u ||
            ((desc->digital_power_reset &
              domain->force_power_up_mask) != 0u &&
             (desc->digital_power_reset &
              domain->force_power_down_mask) != 0u) ||
            ((desc->digital_iso_reset &
              domain->force_noiso_mask) != 0u &&
             (desc->digital_iso_reset &
              domain->force_iso_mask) != 0u))
            return false;
        power_fields |= power;
        iso_fields |= iso;
    }
    uint32_t misc_fields =
        desc->digital_iso_strobe_mask |
        desc->digital_pad_force_hold_mask |
        desc->digital_pad_force_unhold_mask |
        desc->digital_pad_isolation.force_clear_mask |
        desc->digital_pad_isolation.force_set_mask |
        desc->digital_pad_autohold_enable_mask |
        desc->digital_isolation.force_clear_mask |
        desc->digital_isolation.force_set_mask;
    const uint32_t iso_groups[] = {
        iso_fields,
        desc->digital_iso_strobe_mask,
        desc->digital_iso_read_only_mask,
        desc->digital_pad_force_hold_mask,
        desc->digital_pad_force_unhold_mask,
        desc->digital_pad_isolation.force_clear_mask,
        desc->digital_pad_isolation.force_set_mask,
        desc->digital_pad_autohold_enable_mask,
        desc->digital_isolation.force_clear_mask,
        desc->digital_isolation.force_set_mask,
    };
    return power_fields == desc->digital_power_writable_mask &&
           rtc_masks_disjoint(
               iso_groups, sizeof(iso_groups) / sizeof(iso_groups[0])) &&
           (iso_fields | misc_fields) == desc->digital_iso_writable_mask &&
           (desc->digital_wrap_power_down_mask & power_fields) ==
               desc->digital_wrap_power_down_mask;
}

static int rtc_sequence_index(const flexe_rtc_cntl_desc_t *desc,
                              uint32_t offset)
{
    for (unsigned i = 0u; i < desc->sequence_register_count; i++)
        if (offset == desc->sequence_register[i].offset) return (int)i;
    return -1;
}

static int rtc_config_index(const flexe_rtc_cntl_desc_t *desc,
                            uint32_t offset)
{
    for (unsigned i = 0u; i < desc->config_register_count; i++)
        if (offset == desc->config_register[i].offset) return (int)i;
    return -1;
}

static bool rtc_existing_register_offset(
    const flexe_rtc_cntl_desc_t *desc, uint16_t offset)
{
    if (offset == desc->time_update_offset ||
        offset == desc->time_low_offset ||
        offset == desc->time_high_offset ||
        offset == desc->reset_state_offset ||
        offset == desc->clock_conf_offset ||
        offset == desc->analog_conf_offset ||
        offset == desc->date_offset ||
        rtc_interrupt_offset(desc, offset) ||
        rtc_wdt_offset(desc, offset) ||
        rtc_pad_hold_offset(desc, offset))
        return true;
    for (unsigned i = 0u; i < desc->store_count; i++)
        if (offset == desc->store_offset[i]) return true;
    for (unsigned i = 0u; i < desc->sequence_register_count; i++)
        if (offset == desc->sequence_register[i].offset) return true;
    if (desc->cpu_stall_high_offset != 0u &&
        (offset == desc->cpu_stall_options_offset ||
         offset == desc->cpu_stall_high_offset))
        return true;
    if (desc->usb_conf_offset != 0u && offset == desc->usb_conf_offset)
        return true;
    if (desc->sleep_timer_low_offset != 0u) {
        const uint16_t sleep_offsets[] = {
            desc->sleep_timer_low_offset, desc->sleep_timer_high_offset,
            desc->sleep_state_offset, desc->wakeup_state_offset,
            desc->digital_power_offset, desc->wakeup_cause_offset,
            desc->regulator_offset, desc->rtc_power_offset,
            desc->digital_iso_offset, desc->ext_wakeup_config_offset,
            desc->ext1_select_offset, desc->ext1_status_offset,
            desc->brownout_offset,
        };
        for (unsigned i = 0u;
             i < sizeof(sleep_offsets) / sizeof(sleep_offsets[0]); i++)
            if (offset == sleep_offsets[i]) return true;
    }
    return false;
}

static bool rtc_cntl_geometry_valid(const flexe_target_desc_t *target)
{
    if (!target || !(target->capabilities & FLEXE_TARGET_CAP_RTC_CNTL_V1))
        return false;

    const flexe_rtc_cntl_desc_t *desc = &target->rtc_cntl;
    if (((desc->base | desc->register_size) & 0xFFFu) != 0u ||
        desc->register_size == 0u ||
        desc->base < target->peripheral_start ||
        desc->base >= target->peripheral_end ||
        desc->register_size > target->peripheral_end - desc->base ||
        desc->store_count == 0u ||
        desc->store_count > FLEXE_TARGET_RTC_STORE_MAX ||
        desc->slow_clock_hz == 0u ||
        desc->slow_clock_hz > 1000000000u ||
        desc->xtal_frequency_mhz == 0u ||
        target->default_cpu_frequency_mhz == 0u ||
        target->default_cpu_frequency_mhz > 1000u ||
        desc->slow_clock_cal_store >= desc->store_count ||
        desc->xtal_frequency_store >= desc->store_count ||
        desc->time_update_mask == 0u ||
        desc->time_high_mask == 0u ||
        (desc->time_high_mask & (desc->time_high_mask + 1u)) != 0u ||
        !rtc_offset_valid(desc->time_update_offset, desc->register_size) ||
        !rtc_offset_valid(desc->time_low_offset, desc->register_size) ||
        !rtc_offset_valid(desc->time_high_offset, desc->register_size) ||
        !rtc_offset_valid(desc->reset_state_offset, desc->register_size) ||
        desc->time_update_offset == desc->time_low_offset ||
        desc->time_update_offset == desc->time_high_offset ||
        desc->time_update_offset == desc->reset_state_offset ||
        desc->time_update_offset == desc->clock_conf_offset ||
        desc->time_low_offset == desc->time_high_offset ||
        desc->time_low_offset == desc->reset_state_offset ||
        desc->time_low_offset == desc->clock_conf_offset ||
        desc->time_high_offset == desc->reset_state_offset ||
        rtc_interrupt_offset(desc, desc->time_update_offset) ||
        rtc_interrupt_offset(desc, desc->time_low_offset) ||
        rtc_interrupt_offset(desc, desc->time_high_offset) ||
        rtc_interrupt_offset(desc, desc->reset_state_offset) ||
        rtc_interrupt_offset(desc, desc->clock_conf_offset) ||
        rtc_wdt_offset(desc, desc->time_update_offset) ||
        rtc_wdt_offset(desc, desc->time_low_offset) ||
        rtc_wdt_offset(desc, desc->time_high_offset) ||
        rtc_wdt_offset(desc, desc->reset_state_offset) ||
        rtc_wdt_offset(desc, desc->clock_conf_offset))
        return false;

    if (!rtc_hold_geometry_valid(target, desc->rtc_pad_hold_offset,
                                 desc->rtc_pad_hold_first_gpio,
                                 desc->rtc_pad_hold_first_bit,
                                 desc->rtc_pad_hold_count) ||
        !rtc_hold_geometry_valid(target, desc->digital_pad_hold_offset,
                                 desc->digital_pad_hold_first_gpio,
                                 desc->digital_pad_hold_first_bit,
                                 desc->digital_pad_hold_count) ||
        (desc->rtc_pad_hold_count != 0u &&
         (!(target->capabilities & FLEXE_TARGET_CAP_RTC_IO_V1) ||
          desc->rtc_pad_hold_first_gpio + desc->rtc_pad_hold_count >
              target->rtc_io.gpio_count)) ||
        (desc->rtc_pad_hold_count != 0u &&
         desc->digital_pad_hold_count != 0u &&
         desc->rtc_pad_hold_offset == desc->digital_pad_hold_offset))
        return false;

    if (!rtc_offset_valid(desc->clock_conf_offset, desc->register_size) ||
        desc->time_high_offset == desc->clock_conf_offset ||
        desc->reset_state_offset == desc->clock_conf_offset ||
        desc->clock_conf_writable_mask == 0u ||
        (desc->clock_conf_reset & ~desc->clock_conf_writable_mask) != 0u ||
        desc->slow_clock_select_mask == 0u ||
        (desc->slow_clock_select_mask &
         ~desc->clock_conf_writable_mask) != 0u ||
        desc->slow_clock_select_shift > 30u ||
        desc->slow_clock_select_mask !=
            (3u << desc->slow_clock_select_shift) ||
        !rtc_optional_one_bit(desc->fast_clock_select_mask) ||
        desc->fast_clock_select_mask == 0u ||
        (desc->fast_clock_select_mask &
         ~desc->clock_conf_writable_mask) != 0u ||
        (desc->fast_clock_select_mask &
         desc->slow_clock_select_mask) != 0u ||
        desc->fast_clock_source_hz[0] == 0u ||
        desc->fast_clock_source_hz[0] > 1000000000u ||
        desc->fast_clock_source_hz[1] == 0u ||
        desc->fast_clock_source_hz[1] > 1000000000u ||
        desc->slow_clock_source_hz[0] != desc->slow_clock_hz ||
        !rtc_offset_valid(desc->interrupt_enable_offset,
                          desc->register_size) ||
        !rtc_offset_valid(desc->interrupt_raw_offset,
                          desc->register_size) ||
        !rtc_offset_valid(desc->interrupt_status_offset,
                          desc->register_size) ||
        !rtc_offset_valid(desc->interrupt_clear_offset,
                          desc->register_size) ||
        desc->interrupt_enable_offset == desc->interrupt_raw_offset ||
        desc->interrupt_enable_offset == desc->interrupt_status_offset ||
        desc->interrupt_enable_offset == desc->interrupt_clear_offset ||
        desc->interrupt_raw_offset == desc->interrupt_status_offset ||
        desc->interrupt_raw_offset == desc->interrupt_clear_offset ||
        desc->interrupt_status_offset == desc->interrupt_clear_offset ||
        desc->interrupt_valid_mask == 0u ||
        (desc->interrupt_enable_reset &
         ~desc->interrupt_valid_mask) != 0u ||
        (desc->interrupt_raw_reset &
         ~desc->interrupt_valid_mask) != 0u ||
        (desc->interrupt_raw_writable_mask &
         ~desc->interrupt_valid_mask) != 0u ||
        !rtc_offset_valid(desc->wdt_feed_offset,
                          desc->register_size) ||
        !rtc_offset_valid(desc->wdt_write_protect_offset,
                          desc->register_size) ||
        desc->wdt_feed_offset == desc->wdt_write_protect_offset ||
        rtc_interrupt_offset(desc, desc->wdt_feed_offset) ||
        rtc_interrupt_offset(desc, desc->wdt_write_protect_offset) ||
        desc->wdt_enable_mask == 0u ||
        (desc->wdt_enable_mask & (desc->wdt_enable_mask - 1u)) != 0u ||
        desc->wdt_flashboot_enable_mask == 0u ||
        (desc->wdt_flashboot_enable_mask &
         (desc->wdt_flashboot_enable_mask - 1u)) != 0u ||
        (desc->wdt_enable_mask & desc->wdt_flashboot_enable_mask) != 0u ||
        ((desc->wdt_enable_mask |
          desc->wdt_flashboot_enable_mask) &
         ~desc->wdt_config_writable_mask[0]) != 0u ||
        desc->wdt_feed_mask == 0u ||
        (desc->wdt_feed_mask & (desc->wdt_feed_mask - 1u)) != 0u ||
        desc->wdt_write_protect_key == 0u ||
        desc->wdt_interrupt_mask == 0u ||
        (desc->wdt_interrupt_mask &
         (desc->wdt_interrupt_mask - 1u)) != 0u ||
        (desc->wdt_interrupt_mask & ~desc->interrupt_valid_mask) != 0u ||
        desc->wdt_stage_action_mask != 0x7u ||
        desc->wdt_stage0_multiplier == 0u ||
        desc->wdt_stage0_multiplier > 16u ||
        (desc->wdt_stage0_multiplier &
         (desc->wdt_stage0_multiplier - 1u)) != 0u ||
        desc->interrupt_source >= FLEXE_TARGET_INTERRUPT_SOURCE_MAX ||
        ((target->capabilities & FLEXE_TARGET_CAP_INTERRUPT_MATRIX_V1) &&
         desc->interrupt_source >= target->interrupt_matrix.source_count))
        return false;

    if (!rtc_offset_valid(desc->analog_conf_offset, desc->register_size) ||
        desc->analog_conf_writable_mask == 0u ||
        desc->sar_i2c_power_mask == 0u ||
        !rtc_analog_controls_valid(desc) ||
        (desc->analog_conf_reset & ~desc->analog_conf_writable_mask) != 0u ||
        (desc->sar_i2c_power_mask & desc->analog_conf_writable_mask) !=
            desc->sar_i2c_power_mask ||
        (desc->sar_i2c_power_mask &
         (desc->sar_i2c_power_mask - 1u)) != 0u ||
        desc->analog_conf_offset == desc->time_update_offset ||
        desc->analog_conf_offset == desc->time_low_offset ||
        desc->analog_conf_offset == desc->time_high_offset ||
        desc->analog_conf_offset == desc->reset_state_offset ||
        desc->analog_conf_offset == desc->clock_conf_offset ||
        rtc_interrupt_offset(desc, desc->analog_conf_offset) ||
        rtc_wdt_offset(desc, desc->analog_conf_offset) ||
        rtc_pad_hold_offset(desc, desc->analog_conf_offset))
        return false;

    if (!rtc_offset_valid(desc->date_offset, desc->register_size) ||
        desc->date_writable_mask == 0u ||
        (desc->date_reset & ~desc->date_writable_mask) != 0u ||
        desc->date_offset == desc->time_update_offset ||
        desc->date_offset == desc->time_low_offset ||
        desc->date_offset == desc->time_high_offset ||
        desc->date_offset == desc->reset_state_offset ||
        desc->date_offset == desc->clock_conf_offset ||
        desc->date_offset == desc->analog_conf_offset ||
        rtc_interrupt_offset(desc, desc->date_offset) ||
        rtc_wdt_offset(desc, desc->date_offset) ||
        rtc_pad_hold_offset(desc, desc->date_offset))
        return false;

    uint32_t action_fields = 0u;
    for (unsigned stage = 0u; stage < FLEXE_TARGET_RTC_WDT_STAGE_MAX;
         stage++) {
        unsigned shift = desc->wdt_stage_action_shift[stage];
        if (shift > 29u) return false;
        uint32_t field = (uint32_t)desc->wdt_stage_action_mask << shift;
        if ((field & action_fields) != 0u ||
            (field & (desc->wdt_enable_mask |
                      desc->wdt_flashboot_enable_mask)) != 0u ||
            (field & ~desc->wdt_config_writable_mask[0]) != 0u)
            return false;
        action_fields |= field;
    }
    if (!rtc_optional_one_bit(desc->wdt_pause_in_sleep_mask) ||
        (desc->wdt_pause_in_sleep_mask != 0u &&
         (desc->sleep_timer_low_offset == 0u ||
          desc->sleep_enable_mask == 0u)) ||
        ((desc->wdt_pause_in_sleep_mask |
          desc->wdt_auxiliary_control_mask) &
         (desc->wdt_enable_mask | desc->wdt_flashboot_enable_mask |
          action_fields)) != 0u ||
        (desc->wdt_pause_in_sleep_mask &
         desc->wdt_auxiliary_control_mask) != 0u ||
        (desc->wdt_pause_in_sleep_mask &
         ~desc->wdt_config_writable_mask[0]) != 0u ||
        (desc->wdt_auxiliary_control_mask &
         ~desc->wdt_config_writable_mask[0]) != 0u)
        return false;

    for (unsigned i = 0u; i < FLEXE_TARGET_RTC_WDT_CONFIG_MAX; i++) {
        uint16_t offset = desc->wdt_config_offset[i];
        if (!rtc_offset_valid(offset, desc->register_size) ||
            offset == desc->wdt_feed_offset ||
            offset == desc->wdt_write_protect_offset ||
            rtc_interrupt_offset(desc, offset) ||
            offset == desc->time_update_offset ||
            offset == desc->time_low_offset ||
            offset == desc->time_high_offset ||
            offset == desc->reset_state_offset ||
            rtc_pad_hold_offset(desc, offset) ||
            offset == desc->clock_conf_offset ||
            offset == desc->analog_conf_offset ||
            offset == desc->date_offset ||
            (desc->wdt_config_reset[i] &
             ~desc->wdt_config_writable_mask[i]) != 0u)
            return false;
        for (unsigned j = 0u; j < i; j++)
            if (offset == desc->wdt_config_offset[j]) return false;
    }

    for (unsigned source = 0u; source < 3u; source++)
        if (desc->slow_clock_source_hz[source] == 0u ||
            desc->slow_clock_source_hz[source] > 1000000000u)
            return false;

    for (unsigned i = 0u; i < desc->store_count; i++) {
        uint16_t offset = desc->store_offset[i];
        if (!rtc_offset_valid(offset, desc->register_size) ||
            offset == desc->time_update_offset ||
            offset == desc->time_low_offset ||
            offset == desc->time_high_offset ||
            offset == desc->reset_state_offset ||
            rtc_pad_hold_offset(desc, offset) ||
            offset == desc->clock_conf_offset ||
            offset == desc->analog_conf_offset ||
            offset == desc->date_offset ||
            rtc_interrupt_offset(desc, offset) ||
            rtc_wdt_offset(desc, offset))
            return false;
        for (unsigned j = 0u; j < i; j++)
            if (offset == desc->store_offset[j]) return false;
    }
    if (desc->cpu_stall_high_offset != 0u) {
        uint16_t low = desc->cpu_stall_options_offset;
        uint16_t high = desc->cpu_stall_high_offset;
        if (!rtc_options_controls_valid(desc) ||
            !rtc_offset_valid(low, desc->register_size) ||
            !rtc_offset_valid(high, desc->register_size) ||
            low == high ||
            desc->cpu_stall_low_shift[0] > 30u ||
            desc->cpu_stall_low_shift[1] > 30u ||
            desc->cpu_stall_high_shift[0] > 26u ||
            desc->cpu_stall_high_shift[1] > 26u ||
            ((3u << desc->cpu_stall_low_shift[0]) &
             (3u << desc->cpu_stall_low_shift[1])) != 0u ||
            ((0x3Fu << desc->cpu_stall_high_shift[0]) &
             (0x3Fu << desc->cpu_stall_high_shift[1])) != 0u ||
            (desc->cpu_stall_options_reset &
             rtc_stall_low_mask(desc)) != 0u ||
            !rtc_optional_one_bit(desc->software_reset_cpu0_mask) ||
            desc->software_reset_cpu0_mask == 0u ||
            !rtc_optional_one_bit(desc->software_reset_cpu1_mask) ||
            desc->software_reset_cpu1_mask == 0u ||
            !rtc_optional_one_bit(desc->software_reset_system_mask) ||
            desc->software_reset_system_mask == 0u ||
            ((desc->software_reset_cpu0_mask |
              desc->software_reset_cpu1_mask |
              desc->software_reset_system_mask) &
             desc->cpu_stall_options_writable_mask) != 0u ||
            (desc->software_reset_cpu0_mask &
             (desc->software_reset_cpu1_mask |
              desc->software_reset_system_mask |
              rtc_stall_low_mask(desc))) != 0u ||
            (desc->software_reset_cpu1_mask &
             (desc->software_reset_system_mask |
              rtc_stall_low_mask(desc))) != 0u ||
            (desc->software_reset_system_mask &
             rtc_stall_low_mask(desc)) != 0u ||
            (desc->cpu_stall_options_reset &
             (desc->software_reset_cpu0_mask |
              desc->software_reset_cpu1_mask |
              desc->software_reset_system_mask)) != 0u)
            return false;
        for (unsigned i = 0u; i < 2u; i++) {
            uint16_t offset = i == 0u ? low : high;
            if (offset == desc->time_update_offset ||
                offset == desc->time_low_offset ||
                offset == desc->time_high_offset ||
                offset == desc->reset_state_offset ||
                offset == desc->clock_conf_offset ||
                offset == desc->analog_conf_offset ||
                offset == desc->date_offset ||
                rtc_interrupt_offset(desc, offset) ||
                rtc_wdt_offset(desc, offset) ||
                rtc_pad_hold_offset(desc, offset))
                return false;
            for (unsigned j = 0u; j < desc->store_count; j++)
                if (offset == desc->store_offset[j]) return false;
        }
    }
    if (desc->sleep_timer_low_offset != 0u) {
        const uint16_t offsets[] = {
            desc->sleep_timer_low_offset, desc->sleep_timer_high_offset,
            desc->sleep_state_offset, desc->wakeup_state_offset,
            desc->digital_power_offset, desc->wakeup_cause_offset,
            desc->regulator_offset, desc->rtc_power_offset,
            desc->digital_iso_offset,
            desc->ext_wakeup_config_offset, desc->ext1_select_offset,
            desc->ext1_status_offset, desc->brownout_offset,
        };
        if (!(target->capabilities & FLEXE_TARGET_CAP_RTC_IO_V1) ||
            target->rtc_io.gpio_count == 0u ||
            target->rtc_io.gpio_count >= 32u ||
            desc->rtc_pad_hold_count != target->rtc_io.gpio_count ||
            desc->rtc_pad_force_hold_mask == 0u ||
            (desc->rtc_pad_force_hold_mask &
             (desc->rtc_pad_force_hold_mask - 1u)) != 0u ||
            (desc->rtc_power_reset & desc->rtc_pad_force_hold_mask) != 0u ||
            !rtc_regulator_supplies_valid(desc) ||
            !rtc_power_domains_valid(desc) ||
            desc->digital_iso_offset == 0u ||
            desc->digital_pad_hold_count == 0u ||
            desc->digital_pad_force_hold_mask == 0u ||
            (desc->digital_pad_force_hold_mask &
             (desc->digital_pad_force_hold_mask - 1u)) != 0u ||
            desc->digital_pad_force_unhold_mask == 0u ||
            (desc->digital_pad_force_unhold_mask &
             (desc->digital_pad_force_unhold_mask - 1u)) != 0u ||
            (desc->digital_pad_force_hold_mask &
             desc->digital_pad_force_unhold_mask) != 0u ||
            (desc->digital_iso_reset &
             desc->digital_pad_force_hold_mask) != 0u ||
            (desc->digital_iso_reset &
             desc->digital_pad_force_unhold_mask) == 0u ||
            !rtc_digital_domains_valid(desc) ||
            (desc->digital_pad_force_hold_mask &
             ~desc->digital_iso_writable_mask) != 0u ||
            (desc->digital_pad_force_unhold_mask &
             ~desc->digital_iso_writable_mask) != 0u ||
            desc->sleep_enable_mask == 0u ||
            (desc->sleep_enable_mask & (desc->sleep_enable_mask - 1u)) != 0u ||
            desc->sleep_wakeup_mask == 0u ||
            (desc->sleep_wakeup_mask & (desc->sleep_wakeup_mask - 1u)) != 0u ||
            (desc->sleep_enable_mask & desc->sleep_wakeup_mask) != 0u ||
            desc->sleep_alarm_enable_mask == 0u ||
            (desc->sleep_alarm_enable_mask &
             (desc->sleep_alarm_enable_mask - 1u)) != 0u ||
            (desc->sleep_alarm_enable_mask & desc->time_high_mask) != 0u ||
            (desc->sleep_alarm_interrupt_mask &
             desc->interrupt_valid_mask) !=
                desc->sleep_alarm_interrupt_mask ||
            desc->sleep_alarm_interrupt_mask == 0u ||
            desc->digital_wrap_power_down_mask == 0u ||
            (desc->digital_wrap_power_down_mask &
             (desc->digital_wrap_power_down_mask - 1u)) != 0u ||
            desc->timer_wakeup_mask == 0u ||
            (desc->timer_wakeup_mask &
             (desc->timer_wakeup_mask - 1u)) != 0u ||
            (desc->timer_wakeup_mask & desc->wakeup_valid_mask) == 0u ||
            desc->ext0_wakeup_mask == 0u ||
            desc->ext1_wakeup_mask == 0u ||
            (desc->ext0_wakeup_mask &
             (desc->ext0_wakeup_mask - 1u)) != 0u ||
            (desc->ext1_wakeup_mask &
             (desc->ext1_wakeup_mask - 1u)) != 0u ||
            ((desc->timer_wakeup_mask | desc->ext0_wakeup_mask |
              desc->ext1_wakeup_mask) & ~desc->wakeup_valid_mask) != 0u ||
            (desc->ext0_wakeup_mask &
             (desc->timer_wakeup_mask | desc->ext1_wakeup_mask)) != 0u ||
            (desc->ext1_wakeup_mask & desc->timer_wakeup_mask) != 0u ||
            desc->ext0_wakeup_level_mask == 0u ||
            desc->ext1_wakeup_level_mask == 0u ||
            (desc->ext0_wakeup_level_mask &
             (desc->ext0_wakeup_level_mask - 1u)) != 0u ||
            (desc->ext1_wakeup_level_mask &
             (desc->ext1_wakeup_level_mask - 1u)) != 0u ||
            (desc->ext0_wakeup_level_mask &
             desc->ext1_wakeup_level_mask) != 0u ||
            desc->ext1_select_mask !=
                ((1u << target->rtc_io.gpio_count) - 1u) ||
            (desc->ext1_select_mask & desc->ext1_status_clear_mask) != 0u ||
            desc->ext1_status_clear_mask == 0u ||
            (desc->ext1_status_clear_mask &
             (desc->ext1_status_clear_mask - 1u)) != 0u ||
            desc->brownout_writable_mask == 0u ||
            (desc->brownout_reset & ~desc->brownout_writable_mask) != 0u ||
            desc->brownout_detect_mask == 0u ||
            (desc->brownout_detect_mask &
             (desc->brownout_detect_mask - 1u)) != 0u ||
            desc->brownout_count_clear_mask == 0u ||
            (desc->brownout_count_clear_mask &
             (desc->brownout_count_clear_mask - 1u)) != 0u ||
            ((desc->brownout_detect_mask |
              desc->brownout_count_clear_mask) &
             desc->brownout_writable_mask) != 0u ||
            (desc->brownout_detect_mask &
             desc->brownout_count_clear_mask) != 0u ||
            (desc->sleep_wakeup_interrupt_mask &
             desc->interrupt_valid_mask) !=
                desc->sleep_wakeup_interrupt_mask ||
            desc->sleep_wakeup_interrupt_mask == 0u ||
            desc->wakeup_enable_shift >= 32u ||
            desc->wakeup_valid_mask == 0u ||
            (desc->wakeup_enable_reset & ~desc->wakeup_valid_mask) != 0u ||
            desc->wakeup_valid_mask >
                (UINT32_MAX >> desc->wakeup_enable_shift))
            return false;
        for (unsigned i = 0u; i < sizeof(offsets) / sizeof(offsets[0]);
             i++) {
            uint16_t offset = offsets[i];
            if (!rtc_offset_valid(offset, desc->register_size) ||
                offset == desc->time_update_offset ||
                offset == desc->time_low_offset ||
                offset == desc->time_high_offset ||
                offset == desc->reset_state_offset ||
                offset == desc->clock_conf_offset ||
                offset == desc->analog_conf_offset ||
                offset == desc->date_offset ||
                rtc_interrupt_offset(desc, offset) ||
                rtc_wdt_offset(desc, offset) ||
                rtc_pad_hold_offset(desc, offset) ||
                (desc->cpu_stall_high_offset != 0u &&
                 (offset == desc->cpu_stall_options_offset ||
                  offset == desc->cpu_stall_high_offset)))
                return false;
            for (unsigned j = 0u; j < i; j++)
                if (offset == offsets[j]) return false;
            for (unsigned j = 0u; j < desc->store_count; j++)
                if (offset == desc->store_offset[j]) return false;
        }
    }
    if (desc->sequence_register_count >
        FLEXE_TARGET_RTC_SEQUENCE_REGISTER_MAX)
        return false;
    for (unsigned i = 0u; i < desc->sequence_register_count; i++) {
        const flexe_rtc_sequence_register_desc_t *reg =
            &desc->sequence_register[i];
        uint16_t offset = reg->offset;
        if (!rtc_offset_valid(offset, desc->register_size) ||
            reg->writable_mask == 0u ||
            (reg->reset & ~reg->writable_mask) != 0u ||
            offset == desc->time_update_offset ||
            offset == desc->time_low_offset ||
            offset == desc->time_high_offset ||
            offset == desc->reset_state_offset ||
            offset == desc->clock_conf_offset ||
            offset == desc->analog_conf_offset ||
            offset == desc->date_offset ||
            rtc_interrupt_offset(desc, offset) ||
            rtc_wdt_offset(desc, offset) ||
            rtc_pad_hold_offset(desc, offset) ||
            (desc->cpu_stall_high_offset != 0u &&
             (offset == desc->cpu_stall_options_offset ||
              offset == desc->cpu_stall_high_offset)))
            return false;
        if (desc->sleep_timer_low_offset != 0u) {
            const uint16_t sleep_offsets[] = {
                desc->sleep_timer_low_offset,
                desc->sleep_timer_high_offset,
                desc->sleep_state_offset,
                desc->wakeup_state_offset,
                desc->digital_power_offset,
                desc->wakeup_cause_offset,
                desc->regulator_offset,
                desc->rtc_power_offset,
                desc->digital_iso_offset,
                desc->ext_wakeup_config_offset,
                desc->ext1_select_offset,
                desc->ext1_status_offset,
                desc->brownout_offset,
            };
            for (unsigned j = 0u;
                 j < sizeof(sleep_offsets) / sizeof(sleep_offsets[0]); j++)
                if (offset == sleep_offsets[j]) return false;
        }
        for (unsigned j = 0u; j < desc->store_count; j++)
            if (offset == desc->store_offset[j]) return false;
        for (unsigned j = 0u; j < i; j++)
            if (offset == desc->sequence_register[j].offset) return false;
    }
    if (desc->cpu_stall_enable_mask != 0u) {
        int index = rtc_sequence_index(desc,
                                       desc->cpu_stall_enable_offset);
        if (desc->cpu_stall_high_offset == 0u || index < 0 ||
            (desc->cpu_stall_enable_mask &
             (desc->cpu_stall_enable_mask - 1u)) != 0u ||
            (desc->cpu_stall_enable_mask &
             desc->sequence_register[index].writable_mask) == 0u ||
            (desc->cpu_stall_enable_mask &
             desc->sequence_register[index].reset) == 0u)
            return false;
    }
    if (desc->usb_conf_offset != 0u) {
        uint16_t offset = desc->usb_conf_offset;
        if (!(target->capabilities & FLEXE_TARGET_CAP_USB_SERIAL_JTAG_V1) ||
            !rtc_offset_valid(offset, desc->register_size) ||
            desc->usb_phy_override_mask == 0u ||
            (desc->usb_phy_override_mask &
             (desc->usb_phy_override_mask - 1u)) != 0u ||
            desc->usb_phy_select_mask == 0u ||
            (desc->usb_phy_select_mask &
             (desc->usb_phy_select_mask - 1u)) != 0u ||
            (desc->usb_phy_override_mask & desc->usb_phy_select_mask) != 0u ||
            desc->usb_conf_writable_mask !=
                (desc->usb_phy_override_mask | desc->usb_phy_select_mask) ||
            (desc->usb_conf_reset & ~desc->usb_conf_writable_mask) != 0u ||
            offset == desc->time_update_offset ||
            offset == desc->time_low_offset ||
            offset == desc->time_high_offset ||
            offset == desc->reset_state_offset ||
            offset == desc->clock_conf_offset ||
            offset == desc->analog_conf_offset ||
            offset == desc->date_offset ||
            rtc_interrupt_offset(desc, offset) ||
            rtc_wdt_offset(desc, offset) ||
            rtc_pad_hold_offset(desc, offset) ||
            (desc->cpu_stall_high_offset != 0u &&
             (offset == desc->cpu_stall_options_offset ||
              offset == desc->cpu_stall_high_offset)))
            return false;
        for (unsigned i = 0u; i < desc->store_count; i++)
            if (offset == desc->store_offset[i]) return false;
        for (unsigned i = 0u; i < desc->sequence_register_count; i++)
            if (offset == desc->sequence_register[i].offset) return false;
        if (desc->sleep_timer_low_offset != 0u) {
            const uint16_t sleep_offsets[] = {
                desc->sleep_timer_low_offset, desc->sleep_timer_high_offset,
                desc->sleep_state_offset, desc->wakeup_state_offset,
                desc->digital_power_offset, desc->wakeup_cause_offset,
                desc->regulator_offset, desc->rtc_power_offset,
                desc->digital_iso_offset,
                desc->ext_wakeup_config_offset, desc->ext1_select_offset,
                desc->ext1_status_offset, desc->brownout_offset,
            };
            for (unsigned i = 0u;
                 i < sizeof(sleep_offsets) / sizeof(sleep_offsets[0]); i++)
                if (offset == sleep_offsets[i]) return false;
        }
    }
    if (desc->config_register_count >
        FLEXE_TARGET_RTC_CONFIG_REGISTER_MAX)
        return false;
    for (unsigned i = 0u; i < desc->config_register_count; i++) {
        const flexe_rtc_config_register_desc_t *reg =
            &desc->config_register[i];
        if (!rtc_offset_valid(reg->offset, desc->register_size) ||
            reg->writable_mask == 0u ||
            (reg->writable_mask & reg->read_only_mask) != 0u ||
            (reg->supported_mask & ~reg->writable_mask) != 0u ||
            (reg->reset &
             ~(reg->writable_mask | reg->read_only_mask)) != 0u ||
            rtc_existing_register_offset(desc, reg->offset))
            return false;
        for (unsigned j = 0u; j < i; j++)
            if (reg->offset == desc->config_register[j].offset)
                return false;
    }
    return true;
}

static int rtc_store_index(const flexe_rtc_cntl_desc_t *desc,
                           uint32_t offset)
{
    for (unsigned i = 0u; i < desc->store_count; i++)
        if (offset == desc->store_offset[i]) return (int)i;
    return -1;
}

static int rtc_wdt_config_index(const flexe_rtc_cntl_desc_t *desc,
                                uint32_t offset)
{
    for (unsigned i = 0u; i < FLEXE_TARGET_RTC_WDT_CONFIG_MAX; i++)
        if (offset == desc->wdt_config_offset[i]) return (int)i;
    return -1;
}

static void rtc_cntl_update_interrupt(flexe_rtc_cntl_t *rtc);

/* Both target cores observe one always-on RTC. CCOUNT is per-core, so track
 * each core's progress and publish the furthest shared virtual time without
 * double-counting sequential execution of the second core. */
static uint64_t rtc_clock_now(flexe_rtc_cntl_t *rtc)
{
    for (unsigned core = 0u; core < 2u; core++) {
        if (!rtc->cpu[core]) continue;
        uint32_t now = rtc->cpu[core]->ccount;
        if (!rtc->clock.valid[core]) {
            rtc->clock.last_ccount[core] = now;
            rtc->clock.core_cycles[core] = rtc->clock.cycles;
            rtc->clock.valid[core] = true;
            continue;
        }
        uint32_t elapsed = now - rtc->clock.last_ccount[core];
        if (elapsed < (uint32_t)INT32_MAX) {
            if (rtc->clock.core_cycles[core] > UINT64_MAX - elapsed)
                rtc->clock.core_cycles[core] = UINT64_MAX;
            else
                rtc->clock.core_cycles[core] += elapsed;
        }
        rtc->clock.last_ccount[core] = now;
        if (rtc->clock.core_cycles[core] > rtc->clock.cycles)
            rtc->clock.cycles = rtc->clock.core_cycles[core];
    }
    return rtc->clock.cycles;
}

static uint64_t rtc_cpu_hz(const flexe_rtc_cntl_t *rtc)
{
    const xtensa_cpu_t *cpu = rtc->cpu[0] ? rtc->cpu[0] : rtc->cpu[1];
    uint64_t mhz = cpu ? xtensa_cpu_freq_mhz(cpu) :
                   rtc->target->default_cpu_frequency_mhz;
    if (mhz == 0u) mhz = 160u;
    return mhz * UINT64_C(1000000);
}

static uint64_t rtc_slow_clock_hz(const flexe_rtc_cntl_t *rtc)
{
    const flexe_rtc_cntl_desc_t *desc = &rtc->target->rtc_cntl;
    unsigned source = (rtc->clock_conf & desc->slow_clock_select_mask) >>
                      desc->slow_clock_select_shift;
    uint64_t hz = desc->slow_clock_source_hz[source];
    return hz != 0u ? hz : desc->slow_clock_hz;
}

static uint64_t rtc_scaled_ticks(flexe_rtc_cntl_t *rtc,
                                 uint64_t elapsed, uint64_t cpu_hz)
{
    if (rtc->tick_denominator != cpu_hz) {
        rtc->tick_denominator = cpu_hz;
        rtc->tick_remainder = 0u;
    }
    uint64_t slow_hz = rtc_slow_clock_hz(rtc);
    uint64_t quotient = elapsed / cpu_hz;
    uint64_t remainder = elapsed % cpu_hz;
    uint64_t ticks = quotient > UINT64_MAX / slow_hz ?
                     UINT64_MAX : quotient * slow_hz;
    uint64_t fraction = remainder * slow_hz;
    if (fraction > UINT64_MAX - rtc->tick_remainder)
        fraction = UINT64_MAX;
    else
        fraction += rtc->tick_remainder;
    if (ticks <= UINT64_MAX - fraction / cpu_hz)
        ticks += fraction / cpu_hz;
    else
        ticks = UINT64_MAX;
    rtc->tick_remainder = fraction % cpu_hz;
    return ticks;
}

static bool rtc_wdt_active(const flexe_rtc_cntl_t *rtc)
{
    const flexe_rtc_cntl_desc_t *desc = &rtc->target->rtc_cntl;
    return rtc->wdt_stage < FLEXE_TARGET_RTC_WDT_STAGE_MAX &&
           (rtc->wdt_config[0] &
            (desc->wdt_enable_mask |
             desc->wdt_flashboot_enable_mask)) != 0u;
}

static bool rtc_wdt_paused(const flexe_rtc_cntl_t *rtc)
{
    const flexe_rtc_cntl_desc_t *desc = &rtc->target->rtc_cntl;
    return desc->wdt_pause_in_sleep_mask != 0u &&
           (rtc->wdt_config[0] & desc->wdt_pause_in_sleep_mask) != 0u &&
           (rtc->sleep_state & desc->sleep_enable_mask) != 0u;
}

static uint64_t rtc_wdt_stage_hold(const flexe_rtc_cntl_t *rtc)
{
    uint64_t hold = rtc->wdt_config[1u + rtc->wdt_stage];
    if (rtc->wdt_stage == 0u) {
        unsigned multiplier = rtc->target->rtc_cntl.wdt_stage0_multiplier;
        hold = hold > UINT64_MAX / multiplier ?
               UINT64_MAX : hold * multiplier;
    }
    return hold != 0u ? hold : 1u;
}

static unsigned rtc_wdt_stage_action(const flexe_rtc_cntl_t *rtc)
{
    const flexe_rtc_cntl_desc_t *desc = &rtc->target->rtc_cntl;
    return (rtc->wdt_config[0] >>
            desc->wdt_stage_action_shift[rtc->wdt_stage]) &
           desc->wdt_stage_action_mask;
}

static bool rtc_advance_wdt(flexe_rtc_cntl_t *rtc, uint64_t ticks)
{
    if (rtc_wdt_paused(rtc)) return false;
    bool changed = false;
    while (ticks != 0u && rtc_wdt_active(rtc)) {
        uint64_t hold = rtc_wdt_stage_hold(rtc);
        uint64_t remaining = rtc->wdt_stage_ticks < hold ?
                             hold - rtc->wdt_stage_ticks : 1u;
        if (ticks < remaining) {
            rtc->wdt_stage_ticks += ticks;
            break;
        }

        ticks -= remaining;
        rtc->wdt_stage_ticks = 0u;
        unsigned action = rtc_wdt_stage_action(rtc);
        changed = true;
        if (action == 1u) {
            rtc->interrupt_raw |=
                rtc->target->rtc_cntl.wdt_interrupt_mask;
        } else if (action >= FLEXE_RTC_CNTL_WDT_RESET_CPU &&
                   action <= FLEXE_RTC_CNTL_WDT_RESET_RTC) {
            rtc->wdt_stage = FLEXE_TARGET_RTC_WDT_STAGE_MAX;
            if (rtc->reset_requested)
                rtc->reset_requested(
                    rtc->reset_ctx,
                    (flexe_rtc_cntl_reset_action_t)action);
            break;
        }

        if (rtc->wdt_stage + 1u < FLEXE_TARGET_RTC_WDT_STAGE_MAX)
            rtc->wdt_stage++;
        else
            rtc->wdt_stage = FLEXE_TARGET_RTC_WDT_STAGE_MAX;
    }
    return changed;
}

static bool rtc_sync(flexe_rtc_cntl_t *rtc)
{
    uint64_t now = rtc_clock_now(rtc);
    uint64_t elapsed = now >= rtc->last_clock_cycles ?
                       now - rtc->last_clock_cycles : 0u;
    rtc->last_clock_cycles = now;
    if (elapsed == 0u) return false;

    const flexe_rtc_cntl_desc_t *desc = &rtc->target->rtc_cntl;
    uint64_t mask = UINT32_MAX |
                    ((uint64_t)desc->time_high_mask << 32u);
    uint64_t ticks = rtc_scaled_ticks(rtc, elapsed, rtc_cpu_hz(rtc));
    uint64_t before = rtc->counter;
    rtc->counter = (before + ticks) & mask;
    bool changed = rtc_advance_wdt(rtc, ticks);
    if (rtc->sleep_alarm_armed &&
        ((rtc->sleep_alarm - before) & mask) <= ticks) {
        rtc->sleep_alarm_armed = false;
        rtc->interrupt_raw |=
            desc->sleep_alarm_interrupt_mask;
        changed = true;
    }
    if (changed) rtc_cntl_update_interrupt(rtc);
    return changed;
}

static void rtc_cntl_update_interrupt(flexe_rtc_cntl_t *rtc)
{
    bool level = (rtc->interrupt_raw & rtc->interrupt_enable) != 0u;
    if (level == rtc->interrupt_level) return;
    rtc->interrupt_level = level;
    if (rtc->irq_changed) rtc->irq_changed(rtc->irq_ctx, level);
}

static void rtc_cntl_notify(flexe_rtc_cntl_t *rtc)
{
    if (rtc->state_changed) rtc->state_changed(rtc->state_ctx);
}

static void rtc_cntl_publish_digital_domains(flexe_rtc_cntl_t *rtc)
{
    if (!rtc || !rtc->digital_domain_changed) return;
    rtc->digital_domain_changed(
        rtc->digital_domain_ctx,
        flexe_rtc_cntl_powered_digital_domains(rtc),
        flexe_rtc_cntl_isolated_digital_domains(rtc));
}

static void rtc_cntl_publish_rtc_domains(flexe_rtc_cntl_t *rtc)
{
    if (!rtc || !rtc->rtc_domain_changed) return;
    rtc->rtc_domain_changed(
        rtc->rtc_domain_ctx,
        flexe_rtc_cntl_powered_rtc_domains(rtc),
        flexe_rtc_cntl_isolated_rtc_domains(rtc));
}

static void rtc_cntl_publish_supplies(flexe_rtc_cntl_t *rtc)
{
    if (!rtc || !rtc->supply_changed) return;
    rtc->supply_changed(rtc->supply_ctx,
                        flexe_rtc_cntl_powered_supplies(rtc));
}

static bool rtc_control_states_equal(
    const flexe_rtc_cntl_control_state_t *a,
    const flexe_rtc_cntl_control_state_t *b)
{
    return a->powered_options_supplies == b->powered_options_supplies &&
           a->isolated_options_domains == b->isolated_options_domains &&
           a->reset_options_domains == b->reset_options_domains &&
           a->enabled_analog_controls == b->enabled_analog_controls &&
           a->xtal_enable_wait == b->xtal_enable_wait &&
           a->analog_reset_por_powered == b->analog_reset_por_powered &&
           a->digital_pad_isolated == b->digital_pad_isolated &&
           a->digital_pad_autohold_enabled ==
               b->digital_pad_autohold_enabled &&
           a->digital_isolation_enabled == b->digital_isolation_enabled;
}

static void rtc_cntl_publish_controls(flexe_rtc_cntl_t *rtc)
{
    if (!rtc || !rtc->control_changed) return;
    flexe_rtc_cntl_control_state_t state;
    flexe_rtc_cntl_control_state(rtc, &state);
    rtc->control_changed(rtc->control_ctx, &state);
}

static void rtc_cntl_publish_controls_if_changed(
    flexe_rtc_cntl_t *rtc,
    const flexe_rtc_cntl_control_state_t *old_state)
{
    flexe_rtc_cntl_control_state_t state;
    flexe_rtc_cntl_control_state(rtc, &state);
    if (!rtc_control_states_equal(old_state, &state))
        rtc_cntl_publish_controls(rtc);
}

static uint32_t rtc_cntl_read(void *ctx, uint32_t addr)
{
    flexe_rtc_cntl_t *rtc = ctx;
    const flexe_rtc_cntl_desc_t *desc = &rtc->target->rtc_cntl;
    uint32_t offset = addr - desc->base;
    if (rtc_sync(rtc)) rtc_cntl_notify(rtc);
    int index = rtc_store_index(desc, offset);
    if (index >= 0) return rtc->store[index];
    index = rtc_sequence_index(desc, offset);
    if (index >= 0) return rtc->sequence_reg[index];
    index = rtc_config_index(desc, offset);
    if (index >= 0) return rtc->config_reg[index];
    index = rtc_wdt_config_index(desc, offset);
    if (index >= 0) return rtc->wdt_config[index];
    if (offset == desc->time_update_offset) return 0u;
    if (offset == desc->time_low_offset)
        return (uint32_t)rtc->latched_counter;
    if (offset == desc->time_high_offset)
        return (uint32_t)(rtc->latched_counter >> 32u) &
               desc->time_high_mask;
    if (offset == desc->reset_state_offset)
        return rtc->reset_state;
    if (desc->sleep_timer_low_offset != 0u) {
        if (offset == desc->sleep_timer_low_offset)
            return (uint32_t)rtc->sleep_alarm;
        if (offset == desc->sleep_timer_high_offset)
            return (uint32_t)(rtc->sleep_alarm >> 32u) &
                   desc->time_high_mask;
        if (offset == desc->sleep_state_offset)
            return rtc->sleep_state;
        if (offset == desc->wakeup_state_offset)
            return rtc->wakeup_enable << desc->wakeup_enable_shift;
        if (offset == desc->digital_power_offset)
            return rtc->digital_power;
        if (offset == desc->wakeup_cause_offset)
            return rtc->wakeup_cause;
        if (offset == desc->regulator_offset)
            return rtc->regulator_control;
        if (offset == desc->rtc_power_offset)
            return rtc->rtc_power_control;
        if (offset == desc->digital_iso_offset)
            return rtc->digital_iso_control;
        if (offset == desc->ext_wakeup_config_offset)
            return rtc->ext_wakeup_config;
        if (offset == desc->ext1_select_offset)
            return rtc->ext1_select;
        if (offset == desc->ext1_status_offset)
            return rtc->ext1_status;
        if (offset == desc->brownout_offset)
            return rtc->brownout_config;
    }
    if (desc->cpu_stall_high_offset != 0u) {
        if (offset == desc->cpu_stall_options_offset)
            return rtc->cpu_stall_options;
        if (offset == desc->cpu_stall_high_offset)
            return rtc->cpu_stall_high;
    }
    if (offset == desc->clock_conf_offset)
        return rtc->clock_conf;
    if (offset == desc->analog_conf_offset)
        return rtc->analog_conf;
    if (offset == desc->date_offset)
        return rtc->date;
    if (desc->usb_conf_offset != 0u &&
        offset == desc->usb_conf_offset)
        return rtc->usb_conf;
    if (offset == desc->interrupt_enable_offset)
        return rtc->interrupt_enable;
    if (offset == desc->interrupt_raw_offset)
        return rtc->interrupt_raw;
    if (offset == desc->interrupt_status_offset)
        return rtc->interrupt_raw & rtc->interrupt_enable;
    if (offset == desc->interrupt_clear_offset)
        return 0u;
    if (offset == desc->wdt_feed_offset)
        return 0u;
    if (offset == desc->wdt_write_protect_offset)
        return rtc->wdt_write_protect;
    if (desc->rtc_pad_hold_count != 0u &&
        offset == desc->rtc_pad_hold_offset)
        return rtc->rtc_pad_hold;
    if (desc->digital_pad_hold_count != 0u &&
        offset == desc->digital_pad_hold_offset)
        return rtc->digital_pad_hold;
    return rtc->fallback_read ?
        rtc->fallback_read(rtc->fallback_ctx, addr) : 0u;
}

static bool rtc_wdt_actions_supported(
    const flexe_rtc_cntl_desc_t *desc, uint32_t config)
{
    for (unsigned stage = 0u; stage < FLEXE_TARGET_RTC_WDT_STAGE_MAX;
         stage++) {
        unsigned action =
            (config >> desc->wdt_stage_action_shift[stage]) &
            desc->wdt_stage_action_mask;
        if (action > FLEXE_RTC_CNTL_WDT_RESET_RTC) return false;
    }
    return true;
}

static uint32_t rtc_wdt_config0_modeled_mask(
    const flexe_rtc_cntl_desc_t *desc)
{
    uint32_t mask = desc->wdt_enable_mask |
                    desc->wdt_flashboot_enable_mask |
                    desc->wdt_pause_in_sleep_mask |
                    desc->wdt_auxiliary_control_mask;
    for (unsigned stage = 0u; stage < FLEXE_TARGET_RTC_WDT_STAGE_MAX;
         stage++)
        mask |= (uint32_t)desc->wdt_stage_action_mask <<
                desc->wdt_stage_action_shift[stage];
    return mask;
}

static void rtc_cntl_write(void *ctx, uint32_t addr, uint32_t value)
{
    flexe_rtc_cntl_t *rtc = ctx;
    const flexe_rtc_cntl_desc_t *desc = &rtc->target->rtc_cntl;
    uint32_t offset = addr - desc->base;
    if (rtc_sync(rtc)) rtc_cntl_notify(rtc);
    int index = rtc_store_index(desc, offset);
    if (index >= 0) {
        rtc->store[index] = value;
        return;
    }
    index = rtc_sequence_index(desc, offset);
    if (index >= 0) {
        const flexe_rtc_sequence_register_desc_t *reg =
            &desc->sequence_register[index];
        uint32_t old = rtc->sequence_reg[index];
        uint32_t next = (old & ~reg->writable_mask) |
                        (value & reg->writable_mask);
        rtc->sequence_reg[index] = next;
        if ((value & ~reg->writable_mask) != 0u &&
            rtc->fallback_write)
            rtc->fallback_write(rtc->fallback_ctx, addr, value);
        if (offset == desc->cpu_stall_enable_offset &&
            ((old ^ next) & desc->cpu_stall_enable_mask) != 0u)
            rtc_cntl_notify(rtc);
        return;
    }
    index = rtc_config_index(desc, offset);
    if (index >= 0) {
        const flexe_rtc_config_register_desc_t *reg =
            &desc->config_register[index];
        uint32_t old = rtc->config_reg[index];
        uint32_t next = (old & ~reg->writable_mask) |
                        (value & reg->writable_mask);
        rtc->config_reg[index] = next;
        bool unsupported =
            (value & ~(reg->writable_mask | reg->read_only_mask)) != 0u ||
            ((old ^ next) & ~reg->supported_mask) != 0u;
        if (unsupported && rtc->fallback_write)
            rtc->fallback_write(rtc->fallback_ctx, addr, value);
        return;
    }
    index = rtc_wdt_config_index(desc, offset);
    if (index >= 0) {
        if (rtc->wdt_write_protect == desc->wdt_write_protect_key) {
            bool was_active = rtc_wdt_active(rtc);
            uint32_t old = rtc->wdt_config[index];
            uint32_t next =
                value & desc->wdt_config_writable_mask[index];
            rtc->wdt_config[index] = next;
            bool unsupported = index == 0 &&
                (!rtc_wdt_actions_supported(desc, next) ||
                 ((old ^ next) &
                  ~rtc_wdt_config0_modeled_mask(desc)) != 0u);
            if (unsupported && rtc->fallback_write)
                rtc->fallback_write(rtc->fallback_ctx, addr, value);
            if (!was_active && rtc_wdt_active(rtc)) {
                rtc->wdt_stage = 0u;
                rtc->wdt_stage_ticks = 0u;
            }
            rtc_cntl_notify(rtc);
        }
        return;
    }
    if (offset == desc->time_update_offset) {
        if (value & desc->time_update_mask) {
            (void)rtc_sync(rtc);
            rtc->latched_counter = rtc->counter;
        }
        if ((value & ~desc->time_update_mask) != 0u &&
            rtc->fallback_write)
            rtc->fallback_write(rtc->fallback_ctx, addr, value);
        return;
    }
    /* The captured time halves are physically read-only. */
    if (offset == desc->time_low_offset ||
        offset == desc->time_high_offset)
        return;
    if (desc->sleep_timer_low_offset != 0u) {
        uint64_t high_mask = desc->time_high_mask;
        if (offset == desc->sleep_timer_low_offset) {
            rtc->sleep_alarm = (rtc->sleep_alarm &
                                (high_mask << 32u)) | value;
            rtc_cntl_notify(rtc);
            return;
        }
        if (offset == desc->sleep_timer_high_offset) {
            rtc->sleep_alarm = (rtc->sleep_alarm & UINT32_MAX) |
                ((uint64_t)(value & desc->time_high_mask) << 32u);
            if ((value & desc->sleep_alarm_enable_mask) != 0u)
                rtc->sleep_alarm_armed = true;
            if ((value & ~(desc->time_high_mask |
                           desc->sleep_alarm_enable_mask)) != 0u &&
                rtc->fallback_write)
                rtc->fallback_write(rtc->fallback_ctx, addr, value);
            rtc_cntl_notify(rtc);
            return;
        }
        if (offset == desc->sleep_state_offset) {
            uint32_t old = rtc->sleep_state;
            uint32_t old_rtc_powered =
                flexe_rtc_cntl_powered_rtc_domains(rtc);
            uint32_t old_rtc_isolated =
                flexe_rtc_cntl_isolated_rtc_domains(rtc);
            rtc->sleep_state =
                (value & ~desc->sleep_wakeup_mask) |
                (old & desc->sleep_wakeup_mask);
            bool unsupported =
                ((old ^ rtc->sleep_state) &
                 ~(desc->sleep_enable_mask |
                   desc->sleep_wakeup_mask)) != 0u;
            if ((value & desc->sleep_enable_mask) != 0u) {
                rtc->sleep_state &= ~desc->sleep_wakeup_mask;
                uint32_t supported = desc->timer_wakeup_mask |
                    desc->ext0_wakeup_mask | desc->ext1_wakeup_mask;
                bool timer_ready =
                    (rtc->wakeup_enable & desc->timer_wakeup_mask) == 0u ||
                    rtc->sleep_alarm_armed ||
                    (rtc->interrupt_raw &
                     desc->sleep_alarm_interrupt_mask) != 0u;
                if (rtc->wakeup_enable != 0u &&
                    (rtc->wakeup_enable & ~supported) == 0u &&
                    timer_ready &&
                    ((rtc->wakeup_enable & desc->ext1_wakeup_mask) == 0u ||
                     rtc->ext1_select != 0u)) {
                    rtc->sleep_requested = true;
                } else {
                    /* An unknown source or unarmed timer cannot be reported
                     * as a successful wake by the session. */
                    rtc->sleep_requested = false;
                    unsupported = true;
                }
            } else {
                rtc->sleep_requested = false;
            }
            if (unsupported && rtc->fallback_write)
                rtc->fallback_write(rtc->fallback_ctx, addr, value);
            if (old != rtc->sleep_state) {
                rtc_cntl_publish_digital_domains(rtc);
                if (old_rtc_powered !=
                        flexe_rtc_cntl_powered_rtc_domains(rtc) ||
                    old_rtc_isolated !=
                        flexe_rtc_cntl_isolated_rtc_domains(rtc))
                    rtc_cntl_publish_rtc_domains(rtc);
                rtc_cntl_notify(rtc);
            }
            return;
        }
        if (offset == desc->wakeup_state_offset) {
            rtc->wakeup_enable =
                (value >> desc->wakeup_enable_shift) &
                desc->wakeup_valid_mask;
            if (((value & ~(desc->wakeup_valid_mask <<
                            desc->wakeup_enable_shift)) != 0u ||
                 (rtc->wakeup_enable &
                  ~(desc->timer_wakeup_mask |
                    desc->ext0_wakeup_mask |
                    desc->ext1_wakeup_mask)) != 0u) &&
                rtc->fallback_write)
                rtc->fallback_write(rtc->fallback_ctx, addr, value);
            return;
        }
        if (offset == desc->digital_power_offset) {
            uint32_t old_rtc_powered =
                flexe_rtc_cntl_powered_rtc_domains(rtc);
            uint32_t old_rtc_isolated =
                flexe_rtc_cntl_isolated_rtc_domains(rtc);
            uint32_t old = rtc->digital_power;
            rtc->digital_power = value &
                                 desc->digital_power_writable_mask;
            uint32_t changed = old ^ rtc->digital_power;
            if (((changed & ~rtc_digital_power_fields(desc)) != 0u ||
                 (value & ~desc->digital_power_writable_mask) != 0u) &&
                rtc->fallback_write)
                rtc->fallback_write(rtc->fallback_ctx, addr, value);
            if (changed != 0u) {
                rtc_cntl_publish_digital_domains(rtc);
                if (old_rtc_powered !=
                        flexe_rtc_cntl_powered_rtc_domains(rtc) ||
                    old_rtc_isolated !=
                        flexe_rtc_cntl_isolated_rtc_domains(rtc))
                    rtc_cntl_publish_rtc_domains(rtc);
            }
            return;
        }
        if (offset == desc->wakeup_cause_offset)
            return; /* Physically read-only. */
        if (offset == desc->regulator_offset) {
            uint32_t old_powered =
                flexe_rtc_cntl_powered_supplies(rtc);
            uint32_t old = rtc->regulator_control;
            rtc->regulator_control =
                value & desc->regulator_writable_mask;
            uint32_t changed = old ^ rtc->regulator_control;
            if ((((changed & ~rtc_regulator_supply_fields(desc)) != 0u) ||
                 (value & ~desc->regulator_writable_mask) != 0u) &&
                rtc->fallback_write)
                rtc->fallback_write(rtc->fallback_ctx, addr, value);
            if ((changed & rtc_regulator_supply_fields(desc)) != 0u &&
                old_powered != flexe_rtc_cntl_powered_supplies(rtc))
                rtc_cntl_publish_supplies(rtc);
            return;
        }
        if (offset == desc->rtc_power_offset) {
            uint32_t old_powered =
                flexe_rtc_cntl_powered_rtc_domains(rtc);
            uint32_t old_isolated =
                flexe_rtc_cntl_isolated_rtc_domains(rtc);
            uint32_t old = rtc->rtc_power_control;
            rtc->rtc_power_control =
                value & desc->rtc_power_writable_mask;
            uint32_t changed = old ^ rtc->rtc_power_control;
            if ((changed & desc->rtc_pad_force_hold_mask) != 0u &&
                rtc->pad_hold_changed)
                rtc->pad_hold_changed(rtc->pad_hold_ctx,
                                      rtc_all_pad_hold_pins(rtc));
            if ((value & ~desc->rtc_power_writable_mask) != 0u &&
                rtc->fallback_write)
                rtc->fallback_write(rtc->fallback_ctx, addr, value);
            if ((changed & (rtc_power_domain_fields(desc) &
                            ~desc->rtc_pad_force_hold_mask)) != 0u &&
                (old_powered != flexe_rtc_cntl_powered_rtc_domains(rtc) ||
                 old_isolated != flexe_rtc_cntl_isolated_rtc_domains(rtc)))
                rtc_cntl_publish_rtc_domains(rtc);
            return;
        }
        if (offset == desc->digital_iso_offset) {
            flexe_rtc_cntl_control_state_t old_control;
            flexe_rtc_cntl_control_state(rtc, &old_control);
            uint32_t old_powered =
                flexe_rtc_cntl_powered_digital_domains(rtc);
            uint32_t old_isolated =
                flexe_rtc_cntl_isolated_digital_domains(rtc);
            uint32_t old = rtc->digital_iso_control;
            /* Write-only commands do not latch; read-only status survives a
             * guest RMW but cannot be replaced by it. The clear command
             * consumes any modeled autohold status synchronously. */
            uint32_t next =
                (value & desc->digital_iso_writable_mask &
                 ~desc->digital_iso_strobe_mask) |
                (old & desc->digital_iso_read_only_mask);
            if ((value & desc->digital_iso_strobe_mask) != 0u)
                next &= ~desc->digital_iso_read_only_mask;
            rtc->digital_iso_control = next;
            uint32_t hold_bits = desc->digital_pad_force_hold_mask |
                                 desc->digital_pad_force_unhold_mask;
            if (((old ^ next) & hold_bits) != 0u &&
                rtc->pad_hold_changed)
                rtc->pad_hold_changed(rtc->pad_hold_ctx,
                                      rtc_all_pad_hold_pins(rtc));
            if ((value & ~(desc->digital_iso_writable_mask |
                           desc->digital_iso_read_only_mask)) != 0u &&
                rtc->fallback_write)
                rtc->fallback_write(rtc->fallback_ctx, addr, value);
            if (old_powered != flexe_rtc_cntl_powered_digital_domains(rtc) ||
                old_isolated !=
                    flexe_rtc_cntl_isolated_digital_domains(rtc))
                rtc_cntl_publish_digital_domains(rtc);
            rtc_cntl_publish_controls_if_changed(rtc, &old_control);
            return;
        }
        if (offset == desc->ext_wakeup_config_offset) {
            uint32_t old = rtc->ext_wakeup_config;
            rtc->ext_wakeup_config = value;
            if (((old ^ value) &
                 ~(desc->ext0_wakeup_level_mask |
                   desc->ext1_wakeup_level_mask)) != 0u &&
                rtc->fallback_write)
                rtc->fallback_write(rtc->fallback_ctx, addr, value);
            return;
        }
        if (offset == desc->ext1_select_offset) {
            rtc->ext1_select = value & desc->ext1_select_mask;
            if ((value & desc->ext1_status_clear_mask) != 0u)
                rtc->ext1_status = 0u;
            if ((value & ~(desc->ext1_select_mask |
                           desc->ext1_status_clear_mask)) != 0u &&
                rtc->fallback_write)
                rtc->fallback_write(rtc->fallback_ctx, addr, value);
            return;
        }
        if (offset == desc->ext1_status_offset)
            return; /* Physically read-only. */
        if (offset == desc->brownout_offset) {
            /* With a fixed nominal supply, the detection bit stays low.
             * CNT_CLR is a write-only strobe; the remaining documented
             * configuration fields read back until the next RTC reset. */
            rtc->brownout_config = value & desc->brownout_writable_mask;
            if ((value & ~(desc->brownout_writable_mask |
                           desc->brownout_count_clear_mask)) != 0u &&
                rtc->fallback_write)
                rtc->fallback_write(rtc->fallback_ctx, addr, value);
            return;
        }
    }
    if (desc->cpu_stall_high_offset != 0u &&
        offset == desc->cpu_stall_options_offset) {
        flexe_rtc_cntl_control_state_t old_control;
        flexe_rtc_cntl_control_state(rtc, &old_control);
        uint32_t reset_mask = desc->software_reset_cpu0_mask |
                              desc->software_reset_cpu1_mask |
                              desc->software_reset_system_mask;
        uint32_t next = value & desc->cpu_stall_options_writable_mask;
        rtc->cpu_stall_options = next;
        bool unsupported =
            (value & ~(desc->cpu_stall_options_writable_mask |
                       reset_mask)) != 0u ||
            (value & desc->software_reset_cpu1_mask) != 0u;
        /* APP CPU-only reset is not yet modeled. */
        if (unsupported && rtc->fallback_write)
            rtc->fallback_write(rtc->fallback_ctx, addr, value);
        if (rtc->reset_requested) {
            if (value & desc->software_reset_system_mask)
                rtc->reset_requested(rtc->reset_ctx,
                                     FLEXE_RTC_CNTL_SW_RESET_SYSTEM);
            else if (value & desc->software_reset_cpu0_mask)
                rtc->reset_requested(rtc->reset_ctx,
                                     FLEXE_RTC_CNTL_SW_RESET_CPU);
        }
        rtc_cntl_publish_controls_if_changed(rtc, &old_control);
        return;
    }
    if (desc->cpu_stall_high_offset != 0u &&
        offset == desc->cpu_stall_high_offset) {
        uint32_t changed = rtc->cpu_stall_high ^ value;
        rtc->cpu_stall_high = value;
        if ((changed & ~rtc_stall_high_mask(desc)) != 0u &&
            rtc->fallback_write)
            rtc->fallback_write(rtc->fallback_ctx, addr, value);
        return;
    }
    if (offset == desc->clock_conf_offset) {
        (void)rtc_sync(rtc);
        uint32_t old = rtc->clock_conf;
        rtc->clock_conf =
            (old & ~desc->clock_conf_writable_mask) |
            (value & desc->clock_conf_writable_mask);
        unsigned source =
            (rtc->clock_conf & desc->slow_clock_select_mask) >>
            desc->slow_clock_select_shift;
        /* All documented fields retain their exact value. Functional mode
         * resolves the two frequency muxes immediately and deliberately
         * collapses oscillator power/gating/trim timing. Reserved bits and
         * the reserved fourth slow-clock source remain diagnostic. */
        bool unsupported =
            (value & ~desc->clock_conf_writable_mask) != 0u ||
            desc->slow_clock_source_hz[source] == 0u;
        if (unsupported && rtc->fallback_write)
            rtc->fallback_write(rtc->fallback_ctx, addr, value);
        if (((old ^ rtc->clock_conf) &
             desc->slow_clock_select_mask) != 0u) {
            rtc->tick_denominator = 0u;
            rtc->tick_remainder = 0u;
            rtc_cntl_notify(rtc);
        }
        return;
    }
    if (offset == desc->analog_conf_offset) {
        flexe_rtc_cntl_control_state_t old_control;
        flexe_rtc_cntl_control_state(rtc, &old_control);
        rtc->analog_conf = value & desc->analog_conf_writable_mask;
        if ((value & ~desc->analog_conf_writable_mask) != 0u &&
            rtc->fallback_write)
            rtc->fallback_write(rtc->fallback_ctx, addr, value);
        rtc_cntl_publish_controls_if_changed(rtc, &old_control);
        return;
    }
    if (offset == desc->date_offset) {
        rtc->date = value & desc->date_writable_mask;
        if ((value & ~desc->date_writable_mask) != 0u &&
            rtc->fallback_write)
            rtc->fallback_write(rtc->fallback_ctx, addr, value);
        return;
    }
    if (desc->usb_conf_offset != 0u &&
        offset == desc->usb_conf_offset) {
        uint32_t old = rtc->usb_conf;
        rtc->usb_conf = value & desc->usb_conf_writable_mask;
        if ((value & ~desc->usb_conf_writable_mask) != 0u &&
            rtc->fallback_write)
            rtc->fallback_write(rtc->fallback_ctx, addr, value);
        if (old != rtc->usb_conf) rtc_cntl_notify(rtc);
        return;
    }
    if (offset == desc->interrupt_enable_offset) {
        rtc->interrupt_enable = value & desc->interrupt_valid_mask;
        rtc_cntl_update_interrupt(rtc);
        return;
    }
    if (offset == desc->interrupt_raw_offset) {
        rtc->interrupt_raw =
            (rtc->interrupt_raw & ~desc->interrupt_raw_writable_mask) |
            (value & desc->interrupt_raw_writable_mask);
        rtc_cntl_update_interrupt(rtc);
        return;
    }
    if (offset == desc->interrupt_status_offset)
        return; /* Masked status is physically read-only. */
    if (offset == desc->interrupt_clear_offset) {
        rtc->interrupt_raw &= ~(value & desc->interrupt_valid_mask);
        rtc_cntl_update_interrupt(rtc);
        return;
    }
    if (offset == desc->wdt_feed_offset) {
        if ((value & ~desc->wdt_feed_mask) != 0u && rtc->fallback_write)
            rtc->fallback_write(rtc->fallback_ctx, addr, value);
        if (rtc->wdt_write_protect == desc->wdt_write_protect_key &&
            (value & desc->wdt_feed_mask) != 0u) {
            rtc->wdt_stage = 0u;
            rtc->wdt_stage_ticks = 0u;
            rtc_cntl_notify(rtc);
        }
        return;
    }
    if (offset == desc->wdt_write_protect_offset) {
        rtc->wdt_write_protect = value;
        return;
    }
    if (rtc_pad_hold_offset(desc, offset)) {
        bool rtc_pad = offset == desc->rtc_pad_hold_offset &&
                       desc->rtc_pad_hold_count != 0u;
        unsigned first_gpio = rtc_pad ? desc->rtc_pad_hold_first_gpio :
                                        desc->digital_pad_hold_first_gpio;
        unsigned first_bit = rtc_pad ? desc->rtc_pad_hold_first_bit :
                                       desc->digital_pad_hold_first_bit;
        unsigned count = rtc_pad ? desc->rtc_pad_hold_count :
                                   desc->digital_pad_hold_count;
        uint32_t *hold = rtc_pad ? &rtc->rtc_pad_hold :
                                   &rtc->digital_pad_hold;
        uint32_t mask = rtc_hold_valid_mask(rtc->target, first_gpio,
                                            first_bit, count);
        uint32_t next = value & mask;
        if (next != *hold) {
            *hold = next;
            if (rtc->pad_hold_changed)
                rtc->pad_hold_changed(rtc->pad_hold_ctx,
                                      rtc_all_pad_hold_pins(rtc));
        }
        if ((value & ~mask) != 0u && rtc->fallback_write)
            rtc->fallback_write(rtc->fallback_ctx, addr, value);
        return;
    }
    if (rtc->fallback_write)
        rtc->fallback_write(rtc->fallback_ctx, addr, value);
}

flexe_rtc_cntl_t *flexe_rtc_cntl_create(
    xtensa_mem_t *mem, mmio_read_fn fallback_read,
    mmio_write_fn fallback_write, void *fallback_ctx,
    flexe_rtc_cntl_state_fn state_changed, void *state_ctx,
    flexe_rtc_cntl_irq_fn irq_changed, void *irq_ctx,
    flexe_rtc_cntl_reset_fn reset_requested, void *reset_ctx)
{
    if (!mem) return NULL;
    const flexe_target_desc_t *target = mem_target(mem);
    if (!rtc_cntl_geometry_valid(target)) return NULL;

    flexe_rtc_cntl_t *rtc = calloc(1u, sizeof(*rtc));
    if (!rtc) return NULL;
    rtc->mem = mem;
    rtc->target = target;
    rtc->fallback_read = fallback_read;
    rtc->fallback_write = fallback_write;
    rtc->fallback_ctx = fallback_ctx;
    rtc->state_changed = state_changed;
    rtc->state_ctx = state_ctx;
    rtc->irq_changed = irq_changed;
    rtc->irq_ctx = irq_ctx;
    rtc->reset_requested = reset_requested;
    rtc->reset_ctx = reset_ctx;
    const flexe_rtc_cntl_desc_t *desc = &target->rtc_cntl;
    rtc->clock_conf = desc->clock_conf_reset;
    rtc->reset_state = desc->reset_state_reset;
    rtc->wakeup_enable = desc->wakeup_enable_reset;
    rtc->regulator_control = desc->regulator_reset;
    rtc->rtc_power_control = desc->rtc_power_reset;
    rtc->digital_iso_control = desc->digital_iso_reset;
    rtc->digital_power = desc->digital_power_reset;
    rtc->brownout_config = desc->brownout_reset;
    rtc->cpu_stall_options = desc->cpu_stall_options_reset;
    rtc->analog_conf = desc->analog_conf_reset;
    rtc->usb_conf = desc->usb_conf_reset;
    rtc->date = desc->date_reset;
    rtc->interrupt_enable = desc->interrupt_enable_reset;
    rtc->interrupt_raw = desc->interrupt_raw_reset;
    for (unsigned i = 0u; i < FLEXE_TARGET_RTC_WDT_CONFIG_MAX; i++)
        rtc->wdt_config[i] = desc->wdt_config_reset[i];
    rtc->wdt_write_protect = desc->wdt_write_protect_key;
    for (unsigned i = 0u; i < desc->store_count; i++)
        rtc->store[i] = desc->store_reset[i];
    for (unsigned i = 0u; i < desc->sequence_register_count; i++)
        rtc->sequence_reg[i] = desc->sequence_register[i].reset;
    for (unsigned i = 0u; i < desc->config_register_count; i++)
        rtc->config_reg[i] = desc->config_register[i].reset;

    if (mem_register_mmio_range(mem, desc->base, desc->register_size,
                                rtc_cntl_read, rtc_cntl_write, rtc) != 0) {
        free(rtc);
        return NULL;
    }
    rtc_cntl_update_interrupt(rtc);
    return rtc;
}

void flexe_rtc_cntl_destroy(flexe_rtc_cntl_t *rtc)
{
    if (!rtc) return;
    if (rtc->interrupt_level && rtc->irq_changed)
        rtc->irq_changed(rtc->irq_ctx, false);
    const flexe_rtc_cntl_desc_t *desc = &rtc->target->rtc_cntl;
    (void)mem_register_mmio_range(
        rtc->mem, desc->base, desc->register_size,
        rtc->fallback_read, rtc->fallback_write, rtc->fallback_ctx);
    free(rtc);
}

bool flexe_rtc_cntl_cpu_stalled(const flexe_rtc_cntl_t *rtc, unsigned core)
{
    if (!rtc || core > 1u ||
        rtc->target->rtc_cntl.cpu_stall_high_offset == 0u)
        return false;
    const flexe_rtc_cntl_desc_t *desc = &rtc->target->rtc_cntl;
    if (desc->cpu_stall_enable_mask != 0u) {
        int index = rtc_sequence_index(desc,
                                       desc->cpu_stall_enable_offset);
        if (index < 0 || (rtc->sequence_reg[index] &
                          desc->cpu_stall_enable_mask) == 0u)
            return false;
    }
    unsigned low = (rtc->cpu_stall_options >>
                    desc->cpu_stall_low_shift[core]) & 3u;
    unsigned high = (rtc->cpu_stall_high >>
                     desc->cpu_stall_high_shift[core]) & 0x3Fu;
    return ((high << 2u) | low) == 0x86u;
}

void flexe_rtc_cntl_set_pad_hold_listener(flexe_rtc_cntl_t *rtc,
                                          flexe_rtc_cntl_pad_hold_fn fn,
                                          void *ctx)
{
    if (!rtc) return;
    rtc->pad_hold_changed = fn;
    rtc->pad_hold_ctx = ctx;
    if (fn) fn(ctx, rtc_all_pad_hold_pins(rtc));
}

void flexe_rtc_cntl_application_handoff(flexe_rtc_cntl_t *rtc)
{
    if (!rtc) return;
    const flexe_rtc_cntl_desc_t *desc = &rtc->target->rtc_cntl;
    /* The session enters the application directly, after the second-stage
     * bootloader's handoff. The power-on FLASHBOOT_MOD_EN bit is not supposed
     * to remain armed throughout user code in the default ESP-IDF boot flow.
     * Keep WDT_EN independent: firmware that enables the runtime RTC WDT
     * still gets its programmed stages and reset actions. */
    rtc->wdt_config[0] &= ~desc->wdt_flashboot_enable_mask;
    uint64_t calibration = (UINT64_C(1000000) << 19) /
                           desc->slow_clock_hz;
    rtc->store[desc->slow_clock_cal_store] = (uint32_t)calibration;

    uint32_t xtal = desc->xtal_frequency_mhz & UINT16_MAX;
    rtc->store[desc->xtal_frequency_store] = xtal | (xtal << 16u);
}

void flexe_rtc_cntl_attach_cpus(flexe_rtc_cntl_t *rtc,
                                xtensa_cpu_t *cpu0,
                                xtensa_cpu_t *cpu1)
{
    if (!rtc) return;
    (void)rtc_sync(rtc);
    rtc->cpu[0] = cpu0;
    rtc->cpu[1] = cpu1;
    for (unsigned core = 0u; core < 2u; core++) {
        xtensa_cpu_t *cpu = core == 0u ? cpu0 : cpu1;
        rtc->clock.core_cycles[core] = rtc->clock.cycles;
        rtc->clock.last_ccount[core] = cpu ? cpu->ccount : 0u;
        rtc->clock.valid[core] = cpu != NULL;
    }
    rtc->last_clock_cycles = rtc->clock.cycles;
    rtc_cntl_notify(rtc);
}

static uint32_t rtc_cycles_until_ticks(uint64_t ticks,
                                       uint64_t slow_hz,
                                       uint64_t cpu_hz,
                                       uint64_t remainder)
{
    if (ticks == 0u || slow_hz == 0u || cpu_hz == 0u) return 1u;
    uint64_t max_numerator = (uint64_t)INT32_MAX * slow_hz;
    if (max_numerator <= UINT64_MAX - remainder)
        max_numerator += remainder;
    else
        max_numerator = UINT64_MAX;
    if (ticks > max_numerator / cpu_hz)
        return (uint32_t)INT32_MAX;

    uint64_t needed = ticks * cpu_hz;
    if (needed <= remainder) return 1u;
    needed -= remainder;
    uint64_t cycles = needed / slow_hz + (needed % slow_hz != 0u);
    if (cycles == 0u) cycles = 1u;
    if (cycles > (uint64_t)INT32_MAX) cycles = (uint64_t)INT32_MAX;
    return (uint32_t)cycles;
}

uint32_t flexe_rtc_cntl_next_event(flexe_rtc_cntl_t *rtc,
                                   xtensa_cpu_t *cpu)
{
    if (!rtc || !cpu) return UINT32_MAX;
    (void)rtc_sync(rtc);
    uint32_t nearest = UINT32_MAX;
    if (rtc_wdt_active(rtc) && !rtc_wdt_paused(rtc)) {
        uint64_t hold = rtc_wdt_stage_hold(rtc);
        uint64_t ticks = rtc->wdt_stage_ticks < hold ?
                         hold - rtc->wdt_stage_ticks : 1u;
        nearest = rtc_cycles_until_ticks(
            ticks, rtc_slow_clock_hz(rtc), rtc_cpu_hz(rtc),
            rtc->tick_remainder);
    }
    if (rtc->sleep_alarm_armed) {
        const flexe_rtc_cntl_desc_t *desc = &rtc->target->rtc_cntl;
        uint64_t mask = UINT32_MAX |
                        ((uint64_t)desc->time_high_mask << 32u);
        uint64_t ticks = (rtc->sleep_alarm - rtc->counter) & mask;
        uint32_t cycles = rtc_cycles_until_ticks(
            ticks, rtc_slow_clock_hz(rtc), rtc_cpu_hz(rtc),
            rtc->tick_remainder);
        if (cycles < nearest) nearest = cycles;
    }
    return nearest == UINT32_MAX ? UINT32_MAX : cpu->ccount + nearest;
}

void flexe_rtc_cntl_eval(flexe_rtc_cntl_t *rtc)
{
    if (rtc && rtc_sync(rtc)) rtc_cntl_notify(rtc);
}

void flexe_rtc_cntl_set_interrupts(flexe_rtc_cntl_t *rtc,
                                   uint32_t mask, bool asserted)
{
    if (!rtc) return;
    mask &= rtc->target->rtc_cntl.interrupt_valid_mask;
    if (asserted)
        rtc->interrupt_raw |= mask;
    else
        rtc->interrupt_raw &= ~mask;
    rtc_cntl_update_interrupt(rtc);
}

bool flexe_rtc_cntl_sar_i2c_powered(const flexe_rtc_cntl_t *rtc)
{
    return rtc && (rtc->analog_conf &
                   rtc->target->rtc_cntl.sar_i2c_power_mask) != 0u;
}

void flexe_rtc_cntl_control_state(
    const flexe_rtc_cntl_t *rtc, flexe_rtc_cntl_control_state_t *out)
{
    if (!out) return;
    *out = (flexe_rtc_cntl_control_state_t){0};
    if (!rtc) return;

    const flexe_rtc_cntl_desc_t *desc = &rtc->target->rtc_cntl;
    for (unsigned i = 0u; i < desc->options_supply_count; i++)
        if (rtc_supply_powered(rtc->cpu_stall_options,
                               &desc->options_supply[i]))
            out->powered_options_supplies |= 1u << i;
    for (unsigned i = 0u; i < desc->options_isolation_count; i++)
        if (rtc_force_pair_set(rtc->cpu_stall_options,
                               &desc->options_isolation[i]))
            out->isolated_options_domains |= 1u << i;
    for (unsigned i = 0u; i < desc->options_reset_count; i++)
        if (rtc_force_pair_set(rtc->cpu_stall_options,
                               &desc->options_reset[i]))
            out->reset_options_domains |= 1u << i;
    out->xtal_enable_wait = (uint8_t)(
        (rtc->cpu_stall_options & desc->options_xtal_wait_mask) >>
        desc->options_xtal_wait_shift);
    for (unsigned i = 0u; i < desc->analog_control_count; i++)
        if ((rtc->analog_conf & desc->analog_control_mask[i]) != 0u)
            out->enabled_analog_controls |= 1u << i;
    out->analog_reset_por_powered = rtc_supply_powered(
        rtc->analog_conf, &desc->analog_reset_por_supply);
    out->digital_pad_isolated = rtc_force_pair_set(
        rtc->digital_iso_control, &desc->digital_pad_isolation);
    out->digital_pad_autohold_enabled =
        (rtc->digital_iso_control &
         desc->digital_pad_autohold_enable_mask) != 0u;
    out->digital_isolation_enabled = rtc_force_pair_set(
        rtc->digital_iso_control, &desc->digital_isolation);
}

void flexe_rtc_cntl_set_control_listener(
    flexe_rtc_cntl_t *rtc, flexe_rtc_cntl_control_state_fn fn, void *ctx)
{
    if (!rtc) return;
    rtc->control_changed = fn;
    rtc->control_ctx = fn ? ctx : NULL;
    rtc_cntl_publish_controls(rtc);
}

uint32_t flexe_rtc_cntl_fast_clock_hz(const flexe_rtc_cntl_t *rtc)
{
    if (!rtc) return 0u;
    const flexe_rtc_cntl_desc_t *desc = &rtc->target->rtc_cntl;
    unsigned source = (rtc->clock_conf & desc->fast_clock_select_mask) != 0u;
    return desc->fast_clock_source_hz[source];
}

static bool rtc_digital_domain_powered(const flexe_rtc_cntl_t *rtc,
                                       unsigned index)
{
    const flexe_rtc_cntl_desc_t *desc = &rtc->target->rtc_cntl;
    const flexe_rtc_digital_domain_desc_t *domain =
        &desc->digital_domain[index];
    if ((rtc->digital_power & domain->force_power_down_mask) != 0u)
        return false;
    if ((rtc->digital_power & domain->force_power_up_mask) != 0u)
        return true;
    bool sleeping = (rtc->sleep_state & desc->sleep_enable_mask) != 0u;
    return !sleeping || domain->sleep_power_down_mask == 0u ||
           (rtc->digital_power & domain->sleep_power_down_mask) == 0u;
}

uint32_t flexe_rtc_cntl_powered_digital_domains(
    const flexe_rtc_cntl_t *rtc)
{
    if (!rtc) return 0u;
    uint32_t powered = 0u;
    unsigned count = rtc->target->rtc_cntl.digital_domain_count;
    for (unsigned i = 0u; i < count; i++)
        if (rtc_digital_domain_powered(rtc, i)) powered |= 1u << i;
    return powered;
}

uint32_t flexe_rtc_cntl_isolated_digital_domains(
    const flexe_rtc_cntl_t *rtc)
{
    if (!rtc) return UINT32_MAX;
    const flexe_rtc_cntl_desc_t *desc = &rtc->target->rtc_cntl;
    uint32_t isolated = 0u;
    for (unsigned i = 0u; i < desc->digital_domain_count; i++) {
        const flexe_rtc_digital_domain_desc_t *domain =
            &desc->digital_domain[i];
        bool value = !rtc_digital_domain_powered(rtc, i);
        if ((rtc->digital_iso_control & domain->force_iso_mask) != 0u)
            value = true;
        else if ((rtc->digital_iso_control &
                  domain->force_noiso_mask) != 0u)
            value = false;
        if (value) isolated |= 1u << i;
    }
    return isolated;
}

void flexe_rtc_cntl_set_digital_domain_listener(
    flexe_rtc_cntl_t *rtc, flexe_rtc_cntl_domain_state_fn fn, void *ctx)
{
    if (!rtc) return;
    rtc->digital_domain_changed = fn;
    rtc->digital_domain_ctx = fn ? ctx : NULL;
    rtc_cntl_publish_digital_domains(rtc);
}

static bool rtc_power_domain_powered(const flexe_rtc_cntl_t *rtc,
                                     unsigned index)
{
    const flexe_rtc_cntl_desc_t *desc = &rtc->target->rtc_cntl;
    const flexe_rtc_power_domain_desc_t *domain =
        &desc->rtc_power_domain[index];
    if ((rtc->rtc_power_control & domain->force_power_down_mask) != 0u)
        return false;
    if ((rtc->rtc_power_control & domain->force_power_up_mask) != 0u)
        return true;
    if (domain->follow_cpu_mask != 0u &&
        (rtc->rtc_power_control & domain->follow_cpu_mask) != 0u) {
        unsigned cpu_domain = desc->rtc_follow_cpu_domain - 1u;
        return rtc_digital_domain_powered(rtc, cpu_domain);
    }
    bool sleeping = (rtc->sleep_state & desc->sleep_enable_mask) != 0u;
    return !sleeping || domain->sleep_power_down_mask == 0u ||
           (rtc->rtc_power_control & domain->sleep_power_down_mask) == 0u;
}

uint32_t flexe_rtc_cntl_powered_rtc_domains(const flexe_rtc_cntl_t *rtc)
{
    if (!rtc) return 0u;
    uint32_t powered = 0u;
    unsigned count = rtc->target->rtc_cntl.rtc_power_domain_count;
    for (unsigned i = 0u; i < count; i++)
        if (rtc_power_domain_powered(rtc, i)) powered |= 1u << i;
    return powered;
}

uint32_t flexe_rtc_cntl_isolated_rtc_domains(const flexe_rtc_cntl_t *rtc)
{
    if (!rtc) return UINT32_MAX;
    const flexe_rtc_cntl_desc_t *desc = &rtc->target->rtc_cntl;
    uint32_t isolated = 0u;
    for (unsigned i = 0u; i < desc->rtc_power_domain_count; i++) {
        const flexe_rtc_power_domain_desc_t *domain =
            &desc->rtc_power_domain[i];
        bool value = !rtc_power_domain_powered(rtc, i);
        if ((rtc->rtc_power_control & domain->force_iso_mask) != 0u)
            value = true;
        else if ((rtc->rtc_power_control &
                  domain->force_noiso_mask) != 0u)
            value = false;
        if (value) isolated |= 1u << i;
    }
    return isolated;
}

void flexe_rtc_cntl_set_rtc_domain_listener(
    flexe_rtc_cntl_t *rtc, flexe_rtc_cntl_domain_state_fn fn, void *ctx)
{
    if (!rtc) return;
    rtc->rtc_domain_changed = fn;
    rtc->rtc_domain_ctx = fn ? ctx : NULL;
    rtc_cntl_publish_rtc_domains(rtc);
}

uint32_t flexe_rtc_cntl_powered_supplies(const flexe_rtc_cntl_t *rtc)
{
    if (!rtc) return 0u;
    const flexe_rtc_cntl_desc_t *desc = &rtc->target->rtc_cntl;
    uint32_t powered = 0u;
    for (unsigned i = 0u; i < desc->regulator_supply_count; i++) {
        const flexe_rtc_supply_desc_t *supply =
            &desc->regulator_supply[i];
        if (rtc_supply_powered(rtc->regulator_control, supply))
            powered |= 1u << i;
    }
    return powered;
}

void flexe_rtc_cntl_set_supply_listener(
    flexe_rtc_cntl_t *rtc, flexe_rtc_cntl_supply_state_fn fn, void *ctx)
{
    if (!rtc) return;
    rtc->supply_changed = fn;
    rtc->supply_ctx = fn ? ctx : NULL;
    rtc_cntl_publish_supplies(rtc);
}

bool flexe_rtc_cntl_usb_serial_jtag_internal_phy(
    const flexe_rtc_cntl_t *rtc)
{
    if (!rtc || rtc->target->rtc_cntl.usb_conf_offset == 0u)
        return true;
    const flexe_rtc_cntl_desc_t *desc = &rtc->target->rtc_cntl;
    return (rtc->usb_conf & desc->usb_phy_override_mask) == 0u ||
           (rtc->usb_conf & desc->usb_phy_select_mask) == 0u;
}

void flexe_rtc_cntl_retained_snapshot(
    flexe_rtc_cntl_t *rtc, flexe_rtc_cntl_retained_t *out)
{
    if (!out) return;
    *out = (flexe_rtc_cntl_retained_t){0};
    if (!rtc) return;
    (void)rtc_sync(rtc);
    out->counter = rtc->counter;
    out->tick_denominator = rtc->tick_denominator;
    out->tick_remainder = rtc->tick_remainder;
    for (unsigned i = 0u; i < rtc->target->rtc_cntl.store_count; i++)
        out->store[i] = rtc->store[i];
    out->ext1_status = rtc->ext1_status;
}

void flexe_rtc_cntl_retained_restore(
    flexe_rtc_cntl_t *rtc, const flexe_rtc_cntl_retained_t *snapshot)
{
    if (!rtc || !snapshot) return;
    const flexe_rtc_cntl_desc_t *desc = &rtc->target->rtc_cntl;
    uint64_t mask = UINT32_MAX |
                    ((uint64_t)desc->time_high_mask << 32u);
    rtc->counter = snapshot->counter & mask;
    rtc->tick_denominator = snapshot->tick_denominator;
    rtc->tick_remainder = snapshot->tick_remainder;
    for (unsigned i = 0u; i < desc->store_count; i++)
        rtc->store[i] = snapshot->store[i];
    rtc->ext1_status = snapshot->ext1_status & desc->ext1_select_mask;
}

bool flexe_rtc_cntl_take_sleep_request(flexe_rtc_cntl_t *rtc,
                                       bool *deep, uint64_t *timeout_us)
{
    if (!rtc || !rtc->sleep_requested) return false;
    rtc->sleep_requested = false;
    (void)rtc_sync(rtc);
    const flexe_rtc_cntl_desc_t *desc = &rtc->target->rtc_cntl;
    uint32_t supported = desc->timer_wakeup_mask |
        desc->ext0_wakeup_mask | desc->ext1_wakeup_mask;
    if (rtc->wakeup_enable == 0u ||
        (rtc->wakeup_enable & ~supported) != 0u ||
        ((rtc->wakeup_enable & desc->ext1_wakeup_mask) != 0u &&
         rtc->ext1_select == 0u) ||
        ((rtc->wakeup_enable & desc->timer_wakeup_mask) != 0u &&
         !rtc->sleep_alarm_armed &&
         (rtc->interrupt_raw & desc->sleep_alarm_interrupt_mask) == 0u))
        return false;
    if (deep)
        *deep = (rtc->digital_power &
                 desc->digital_wrap_power_down_mask) != 0u;
    if (timeout_us) {
        uint64_t ticks = 0u;
        if ((rtc->wakeup_enable & desc->timer_wakeup_mask) == 0u) {
            *timeout_us = UINT64_MAX;
            return true;
        }
        if (rtc->sleep_alarm_armed) {
            uint64_t mask = UINT32_MAX |
                            ((uint64_t)desc->time_high_mask << 32u);
            ticks = (rtc->sleep_alarm - rtc->counter) & mask;
        }
        uint64_t hz = rtc_slow_clock_hz(rtc);
        uint64_t seconds = ticks / hz;
        uint64_t fraction = ticks % hz;
        *timeout_us = seconds > UINT64_MAX / UINT64_C(1000000) ?
            UINT64_MAX : seconds * UINT64_C(1000000) +
            (fraction * UINT64_C(1000000) + hz - 1u) / hz;
    }
    return true;
}

bool flexe_rtc_cntl_has_gpio_wake(const flexe_rtc_cntl_t *rtc)
{
    if (!rtc) return false;
    const flexe_rtc_cntl_desc_t *desc = &rtc->target->rtc_cntl;
    return (rtc->wakeup_enable &
            (desc->ext0_wakeup_mask | desc->ext1_wakeup_mask)) != 0u;
}

uint32_t flexe_rtc_cntl_poll_gpio_wake(flexe_rtc_cntl_t *rtc,
                                       int ext0_level,
                                       uint32_t ext1_high_mask)
{
    if (!rtc) return 0u;
    const flexe_rtc_cntl_desc_t *desc = &rtc->target->rtc_cntl;
    if ((rtc->sleep_state & desc->sleep_enable_mask) == 0u)
        return 0u;
    uint32_t cause = 0u;
    if ((rtc->wakeup_enable & desc->ext0_wakeup_mask) != 0u &&
        ext0_level >= 0 &&
        ext0_level == ((rtc->ext_wakeup_config &
                        desc->ext0_wakeup_level_mask) != 0u))
        cause |= desc->ext0_wakeup_mask;
    if ((rtc->wakeup_enable & desc->ext1_wakeup_mask) != 0u &&
        rtc->ext1_select != 0u) {
        uint32_t high = ext1_high_mask & rtc->ext1_select;
        bool any_high = (rtc->ext_wakeup_config &
                         desc->ext1_wakeup_level_mask) != 0u;
        if ((any_high && high != 0u) ||
            (!any_high && high == 0u)) {
            rtc->ext1_status = any_high ? high : rtc->ext1_select;
            cause |= desc->ext1_wakeup_mask;
        }
    }
    return cause;
}

void flexe_rtc_cntl_finish_wake(flexe_rtc_cntl_t *rtc, uint32_t cause)
{
    if (!rtc || rtc->target->rtc_cntl.sleep_timer_low_offset == 0u) return;
    /* Account for the final interval while SLEEP_ENA is still visible so a
     * watchdog configured to pause in sleep does not consume it on resume. */
    (void)rtc_sync(rtc);
    const flexe_rtc_cntl_desc_t *desc = &rtc->target->rtc_cntl;
    uint32_t old_rtc_powered =
        flexe_rtc_cntl_powered_rtc_domains(rtc);
    uint32_t old_rtc_isolated =
        flexe_rtc_cntl_isolated_rtc_domains(rtc);
    rtc->wakeup_cause = cause & desc->wakeup_valid_mask;
    rtc->sleep_state &= ~desc->sleep_enable_mask;
    rtc->sleep_state |= desc->sleep_wakeup_mask;
    rtc->sleep_alarm_armed = false;
    rtc->sleep_requested = false;
    rtc->interrupt_raw |= desc->sleep_wakeup_interrupt_mask;
    rtc_cntl_update_interrupt(rtc);
    rtc_cntl_publish_digital_domains(rtc);
    if (old_rtc_powered != flexe_rtc_cntl_powered_rtc_domains(rtc) ||
        old_rtc_isolated != flexe_rtc_cntl_isolated_rtc_domains(rtc))
        rtc_cntl_publish_rtc_domains(rtc);
    rtc_cntl_notify(rtc);
}

void flexe_rtc_cntl_set_wake_state(flexe_rtc_cntl_t *rtc,
                                   uint32_t cause, uint32_t reset_cause)
{
    if (!rtc || rtc->target->rtc_cntl.sleep_timer_low_offset == 0u) return;
    rtc->wakeup_cause = cause & rtc->target->rtc_cntl.wakeup_valid_mask;
    /* ESP32-S3 RESET_STATE has six-bit reset causes for PRO and APP CPU at
     * bits 0 and 6; retain the target's other reset-state fields. */
    rtc->reset_state =
        (rtc->reset_state & ~0xFFFu) |
        (reset_cause & 0x3Fu) |
        ((reset_cause & 0x3Fu) << 6u);
}
