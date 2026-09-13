#include "rtc_cntl.h"

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
    uint32_t clock_conf;
    uint32_t interrupt_enable;
    uint32_t interrupt_raw;
    bool interrupt_level;
    uint32_t wdt_config[FLEXE_TARGET_RTC_WDT_CONFIG_MAX];
    uint32_t wdt_write_protect;
    uint64_t wdt_stage_ticks;
    uint8_t wdt_stage;
    uint32_t store[FLEXE_TARGET_RTC_STORE_MAX];
    uint32_t digital_pad_hold;
    flexe_rtc_cntl_pad_hold_fn pad_hold_changed;
    void *pad_hold_ctx;
};

static uint32_t rtc_digital_pad_hold_mask(const flexe_rtc_cntl_desc_t *desc)
{
    if (desc->digital_pad_hold_count == 0u) return 0u;
    return (UINT32_MAX >> (32u - desc->digital_pad_hold_count))
           << desc->digital_pad_hold_first_bit;
}

static uint32_t rtc_digital_pad_hold_valid_mask(
    const flexe_target_desc_t *target)
{
    const flexe_rtc_cntl_desc_t *desc = &target->rtc_cntl;
    return ((uint32_t)(target->gpio.valid_gpio_mask >>
                       desc->digital_pad_hold_first_gpio)
            << desc->digital_pad_hold_first_bit) &
           rtc_digital_pad_hold_mask(desc);
}

static uint64_t rtc_digital_pad_hold_pins(const flexe_rtc_cntl_t *rtc)
{
    const flexe_rtc_cntl_desc_t *desc = &rtc->target->rtc_cntl;
    if (desc->digital_pad_hold_count == 0u) return 0u;
    return (uint64_t)(rtc->digital_pad_hold >>
                      desc->digital_pad_hold_first_bit)
           << desc->digital_pad_hold_first_gpio;
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

    if (desc->digital_pad_hold_count != 0u &&
        (!rtc_offset_valid(desc->digital_pad_hold_offset,
                           desc->register_size) ||
         desc->digital_pad_hold_first_bit >= 32u ||
         desc->digital_pad_hold_count >
             32u - desc->digital_pad_hold_first_bit ||
         desc->digital_pad_hold_first_gpio >= 64u ||
         desc->digital_pad_hold_count >
             64u - desc->digital_pad_hold_first_gpio ||
         !(target->capabilities & FLEXE_TARGET_CAP_GPIO_V1) ||
         desc->digital_pad_hold_first_gpio +
             desc->digital_pad_hold_count > target->gpio.gpio_count ||
         desc->digital_pad_hold_offset == desc->clock_conf_offset ||
         desc->digital_pad_hold_offset == desc->reset_state_offset ||
         desc->digital_pad_hold_offset == desc->time_update_offset ||
         desc->digital_pad_hold_offset == desc->time_low_offset ||
         desc->digital_pad_hold_offset == desc->time_high_offset ||
         rtc_interrupt_offset(desc, desc->digital_pad_hold_offset) ||
         rtc_wdt_offset(desc, desc->digital_pad_hold_offset)))
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
            (desc->digital_pad_hold_count != 0u &&
             offset == desc->digital_pad_hold_offset) ||
            offset == desc->clock_conf_offset ||
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
            (desc->digital_pad_hold_count != 0u &&
             offset == desc->digital_pad_hold_offset) ||
            offset == desc->clock_conf_offset ||
            rtc_interrupt_offset(desc, offset) ||
            rtc_wdt_offset(desc, offset))
            return false;
        for (unsigned j = 0u; j < i; j++)
            if (offset == desc->store_offset[j]) return false;
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
                    (flexe_rtc_cntl_wdt_action_t)action);
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
    rtc->counter = (rtc->counter + ticks) & mask;
    bool changed = rtc_advance_wdt(rtc, ticks);
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

static uint32_t rtc_cntl_read(void *ctx, uint32_t addr)
{
    flexe_rtc_cntl_t *rtc = ctx;
    const flexe_rtc_cntl_desc_t *desc = &rtc->target->rtc_cntl;
    uint32_t offset = addr - desc->base;
    if (rtc_sync(rtc)) rtc_cntl_notify(rtc);
    int index = rtc_store_index(desc, offset);
    if (index >= 0) return rtc->store[index];
    index = rtc_wdt_config_index(desc, offset);
    if (index >= 0) return rtc->wdt_config[index];
    if (offset == desc->time_update_offset) return 0u;
    if (offset == desc->time_low_offset)
        return (uint32_t)rtc->latched_counter;
    if (offset == desc->time_high_offset)
        return (uint32_t)(rtc->latched_counter >> 32u) &
               desc->time_high_mask;
    if (offset == desc->reset_state_offset)
        return desc->reset_state_reset;
    if (offset == desc->clock_conf_offset)
        return rtc->clock_conf;
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
                    desc->wdt_flashboot_enable_mask;
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
    if (offset == desc->clock_conf_offset) {
        (void)rtc_sync(rtc);
        uint32_t old = rtc->clock_conf;
        rtc->clock_conf =
            (old & ~desc->clock_conf_writable_mask) |
            (value & desc->clock_conf_writable_mask);
        unsigned source =
            (rtc->clock_conf & desc->slow_clock_select_mask) >>
            desc->slow_clock_select_shift;
        bool unsupported =
            ((old ^ rtc->clock_conf) &
             ~desc->slow_clock_select_mask) != 0u ||
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
    if (desc->digital_pad_hold_count != 0u &&
        offset == desc->digital_pad_hold_offset) {
        uint32_t mask = rtc_digital_pad_hold_valid_mask(rtc->target);
        uint32_t next = value & mask;
        if (next != rtc->digital_pad_hold) {
            rtc->digital_pad_hold = next;
            if (rtc->pad_hold_changed)
                rtc->pad_hold_changed(rtc->pad_hold_ctx,
                                      rtc_digital_pad_hold_pins(rtc));
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
    rtc->interrupt_enable = desc->interrupt_enable_reset;
    rtc->interrupt_raw = desc->interrupt_raw_reset;
    for (unsigned i = 0u; i < FLEXE_TARGET_RTC_WDT_CONFIG_MAX; i++)
        rtc->wdt_config[i] = desc->wdt_config_reset[i];
    rtc->wdt_write_protect = desc->wdt_write_protect_key;
    for (unsigned i = 0u; i < desc->store_count; i++)
        rtc->store[i] = desc->store_reset[i];

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

void flexe_rtc_cntl_set_pad_hold_listener(flexe_rtc_cntl_t *rtc,
                                          flexe_rtc_cntl_pad_hold_fn fn,
                                          void *ctx)
{
    if (!rtc) return;
    rtc->pad_hold_changed = fn;
    rtc->pad_hold_ctx = ctx;
    if (fn) fn(ctx, rtc_digital_pad_hold_pins(rtc));
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
    if (!rtc_wdt_active(rtc)) return UINT32_MAX;

    uint64_t hold = rtc_wdt_stage_hold(rtc);
    uint64_t ticks = rtc->wdt_stage_ticks < hold ?
                     hold - rtc->wdt_stage_ticks : 1u;
    uint32_t cycles = rtc_cycles_until_ticks(
        ticks, rtc_slow_clock_hz(rtc), rtc_cpu_hz(rtc),
        rtc->tick_remainder);
    return cpu->ccount + cycles;
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
