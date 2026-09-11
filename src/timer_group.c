#include "timer_group.h"

#include "xtensa.h"

#include <limits.h>
#include <stdbool.h>
#include <stdlib.h>

/* ESP32-S3-style timer-group V1 register layout. */
#define TIMER_GROUP_TIMER_STRIDE        0x024u
#define TIMER_GROUP_WDT_CONFIG0_OFF     0x048u
#define TIMER_GROUP_WDT_CONFIG1_OFF     0x04Cu
#define TIMER_GROUP_WDT_CONFIG2_OFF     0x050u
#define TIMER_GROUP_WDT_CONFIG5_OFF     0x05Cu
#define TIMER_GROUP_WDT_FEED_OFF        0x060u
#define TIMER_GROUP_WDT_PROTECT_OFF     0x064u
#define TIMER_GROUP_INT_ENA_OFF         0x070u
#define TIMER_GROUP_INT_RAW_OFF         0x074u
#define TIMER_GROUP_INT_ST_OFF          0x078u
#define TIMER_GROUP_INT_CLR_OFF         0x07Cu
#define TIMER_GROUP_DATE_OFF            0x0F8u
#define TIMER_GROUP_REGCLK_OFF          0x0FCu

#define TIMER_CONFIG_USE_XTAL           (1u << 9)
#define TIMER_CONFIG_ALARM_EN           (1u << 10)
#define TIMER_CONFIG_DIVCNT_RST         (1u << 12)
#define TIMER_CONFIG_DIVIDER_MASK       (0xFFFFu << 13)
#define TIMER_CONFIG_AUTORELOAD         (1u << 29)
#define TIMER_CONFIG_INCREASE           (1u << 30)
#define TIMER_CONFIG_ENABLE             (1u << 31)

#define WDT_CONFIG_FLASHBOOT_ENABLE     (1u << 14)
#define WDT_CONFIG_ENABLE               (1u << 31)
#define WDT_INTERRUPT_EVENT             2u
#define TIMER_GROUP_INTERRUPT_MASK      0x7u
#define TIMER_GROUP_WDT_STAGE_COUNT     4u

typedef struct {
    uint32_t config;
    uint64_t counter;
    uint64_t latched;
    uint64_t alarm;
    uint64_t load;
    uint64_t tick_remainder;
    uint64_t tick_denominator;
} timer_group_timer_t;

typedef struct {
    uint32_t config[6];
    uint32_t protect;
    uint64_t stage_ticks;
    uint64_t tick_remainder;
    uint64_t tick_denominator;
    uint8_t stage;
} timer_group_wdt_t;

typedef struct {
    timer_group_timer_t timer[FLEXE_TARGET_TIMER_GROUP_TIMER_MAX];
    timer_group_wdt_t wdt;
    uint32_t int_enable;
    uint32_t int_raw;
    uint32_t date;
    uint32_t regclk;
    bool irq_level[FLEXE_TARGET_TIMER_GROUP_EVENT_MAX];
} timer_group_state_t;

typedef struct {
    uint64_t cycles;
    uint64_t core_cycles[2];
    uint32_t last_ccount[2];
    bool valid[2];
} timer_group_clock_t;

struct flexe_timer_group {
    xtensa_mem_t *mem;
    const flexe_target_desc_t *target;
    mmio_read_fn fallback_read;
    mmio_write_fn fallback_write;
    void *fallback_ctx;
    flexe_timer_group_state_fn state_changed;
    void *state_ctx;
    flexe_timer_group_irq_fn irq_changed;
    void *irq_ctx;
    flexe_timer_group_reset_fn reset_requested;
    void *reset_ctx;
    xtensa_cpu_t *cpu[2];
    timer_group_clock_t clock;
    uint64_t last_clock_cycles;
    timer_group_state_t group[FLEXE_TARGET_TIMER_GROUP_MAX];
};

static bool timer_group_geometry_valid(const flexe_target_desc_t *target)
{
    if (!target || !(target->capabilities &
                     FLEXE_TARGET_CAP_TIMER_GROUP_V1))
        return false;
    const flexe_timer_group_desc_t *desc = &target->timer_group;
    if (desc->group_count == 0u ||
        desc->group_count > FLEXE_TARGET_TIMER_GROUP_MAX ||
        desc->timer_count != FLEXE_TARGET_TIMER_GROUP_TIMER_MAX ||
        desc->counter_width < 33u || desc->counter_width > 63u ||
        desc->register_size < TIMER_GROUP_REGCLK_OFF + sizeof(uint32_t) ||
        desc->apb_clock_hz == 0u || desc->apb_clock_hz > 1000000000u ||
        desc->xtal_clock_hz == 0u || desc->xtal_clock_hz > 1000000000u ||
        desc->wdt_write_protect_key == 0u ||
        (desc->timer_config_reset &
         ~desc->timer_config_writable_mask) != 0u ||
        (desc->date_reset & ~desc->date_writable_mask) != 0u ||
        (desc->regclk_reset & ~desc->regclk_writable_mask) != 0u)
        return false;

    for (unsigned index = 0u; index < 6u; index++) {
        if ((desc->wdt_config_reset[index] &
             ~desc->wdt_config_writable_mask[index]) != 0u)
            return false;
    }
    for (unsigned group = 0u; group < desc->group_count; group++) {
        uint32_t base = desc->base[group];
        if ((base & 0xFFFu) != 0u ||
            base < target->peripheral_start ||
            base >= target->peripheral_end ||
            desc->register_size > target->peripheral_end - base)
            return false;
        for (unsigned event = 0u;
             event < FLEXE_TARGET_TIMER_GROUP_EVENT_MAX; event++) {
            unsigned source = desc->interrupt_source[group][event];
            if (source >= FLEXE_TARGET_INTERRUPT_SOURCE_MAX ||
                ((target->capabilities &
                  FLEXE_TARGET_CAP_INTERRUPT_MATRIX_V1) &&
                 source >= target->interrupt_matrix.source_count))
                return false;
        }
    }
    return true;
}

static uint64_t timer_group_counter_mask(
    const flexe_timer_group_t *timer_group)
{
    return (UINT64_C(1) <<
            timer_group->target->timer_group.counter_width) - 1u;
}

static uint32_t timer_group_high_mask(
    const flexe_timer_group_t *timer_group)
{
    return (uint32_t)(timer_group_counter_mask(timer_group) >> 32);
}

static uint64_t timer_group_clock_now(flexe_timer_group_t *timer_group)
{
    timer_group_clock_t *clock = &timer_group->clock;
    for (unsigned core = 0u; core < 2u; core++) {
        xtensa_cpu_t *cpu = timer_group->cpu[core];
        if (!cpu) continue;
        uint32_t now = cpu->ccount;
        if (!clock->valid[core]) {
            clock->last_ccount[core] = now;
            clock->core_cycles[core] = clock->cycles;
            clock->valid[core] = true;
            continue;
        }
        uint32_t elapsed = now - clock->last_ccount[core];
        if (elapsed < (uint32_t)INT32_MAX) {
            if (clock->core_cycles[core] > UINT64_MAX - elapsed)
                clock->core_cycles[core] = UINT64_MAX;
            else
                clock->core_cycles[core] += elapsed;
        }
        clock->last_ccount[core] = now;
        if (clock->core_cycles[core] > clock->cycles)
            clock->cycles = clock->core_cycles[core];
    }
    return clock->cycles;
}

static uint64_t timer_group_cpu_hz(
    const flexe_timer_group_t *timer_group, const xtensa_cpu_t *preferred)
{
    const xtensa_cpu_t *cpu = preferred ? preferred : timer_group->cpu[0];
    if (!cpu) cpu = timer_group->cpu[1];
    uint64_t mhz = cpu ? xtensa_cpu_freq_mhz(cpu) :
                   timer_group->target->default_cpu_frequency_mhz;
    if (mhz == 0u) mhz = 160u;
    return mhz * UINT64_C(1000000);
}

/* Convert target CPU cycles into source-clock ticks while preserving the
 * fractional phase. clock_now only accepts deltas below INT32_MAX, and
 * descriptors cap source clocks at 1 GHz, so elapsed * source_hz fits in
 * uint64_t for every accepted update. */
static uint64_t timer_group_scaled_ticks(uint64_t elapsed,
                                         uint64_t source_hz,
                                         uint64_t denominator,
                                         uint64_t *old_denominator,
                                         uint64_t *remainder)
{
    if (denominator == 0u) return 0u;
    if (*old_denominator != denominator) {
        *old_denominator = denominator;
        *remainder = 0u;
    }
    if (elapsed > (UINT64_MAX - *remainder) / source_hz) {
        *remainder = 0u;
        return UINT64_MAX;
    }
    uint64_t numerator = elapsed * source_hz + *remainder;
    uint64_t ticks = numerator / denominator;
    *remainder = numerator % denominator;
    return ticks;
}

static uint32_t timer_group_timer_divider(
    const timer_group_timer_t *timer)
{
    uint32_t divider =
        (timer->config & TIMER_CONFIG_DIVIDER_MASK) >> 13;
    return divider != 0u ? divider : 65536u;
}

static uint64_t timer_group_timer_source_hz(
    const flexe_timer_group_t *timer_group,
    const timer_group_timer_t *timer)
{
    const flexe_timer_group_desc_t *desc =
        &timer_group->target->timer_group;
    return (timer->config & TIMER_CONFIG_USE_XTAL) ?
        desc->xtal_clock_hz : desc->apb_clock_hz;
}

static void timer_group_update_irqs(flexe_timer_group_t *timer_group,
                                    unsigned group)
{
    timer_group_state_t *state = &timer_group->group[group];
    uint32_t status = state->int_raw & state->int_enable;
    for (unsigned event = 0u;
         event < FLEXE_TARGET_TIMER_GROUP_EVENT_MAX; event++) {
        bool level = (status & (1u << event)) != 0u;
        if (level == state->irq_level[event]) continue;
        state->irq_level[event] = level;
        if (timer_group->irq_changed)
            timer_group->irq_changed(timer_group->irq_ctx, group,
                                     event, level);
    }
}

static bool timer_group_advance_timer(flexe_timer_group_t *timer_group,
                                      unsigned group, unsigned index,
                                      uint64_t ticks)
{
    if (ticks == 0u) return false;
    timer_group_state_t *state = &timer_group->group[group];
    timer_group_timer_t *timer = &state->timer[index];
    uint64_t mask = timer_group_counter_mask(timer_group);
    bool increase = (timer->config & TIMER_CONFIG_INCREASE) != 0u;
    uint64_t distance = increase ?
        (timer->alarm - timer->counter) & mask :
        (timer->counter - timer->alarm) & mask;

    /* Equality is the just-observed compare point, not a new edge. The next
     * match is one complete counter period away. */
    if (distance == 0u)
        distance = mask + 1u;

    bool fired = (timer->config & TIMER_CONFIG_ALARM_EN) != 0u &&
                 ticks >= distance;
    if (!fired) {
        timer->counter = increase ?
            (timer->counter + ticks) & mask :
            (timer->counter - ticks) & mask;
        return false;
    }

    timer->counter = timer->alarm & mask;
    ticks -= distance;
    bool autoreload =
        (timer->config & TIMER_CONFIG_AUTORELOAD) != 0u;
    timer->config &= ~TIMER_CONFIG_ALARM_EN;
    state->int_raw |= 1u << index;
    if (autoreload) timer->counter = timer->load & mask;
    if (ticks != 0u) {
        timer->counter = increase ?
            (timer->counter + ticks) & mask :
            (timer->counter - ticks) & mask;
    }
    return true;
}

static bool timer_group_wdt_active(const timer_group_wdt_t *wdt)
{
    return wdt->stage < TIMER_GROUP_WDT_STAGE_COUNT &&
           (wdt->config[0] &
            (WDT_CONFIG_ENABLE | WDT_CONFIG_FLASHBOOT_ENABLE)) != 0u;
}

static uint32_t timer_group_wdt_prescaler(
    const timer_group_wdt_t *wdt)
{
    uint32_t prescaler = wdt->config[1] >> 16;
    return prescaler != 0u ? prescaler : 1u;
}

static unsigned timer_group_wdt_action(const timer_group_wdt_t *wdt)
{
    static const uint8_t shift[TIMER_GROUP_WDT_STAGE_COUNT] = {
        29u, 27u, 25u, 23u,
    };
    return (wdt->config[0] >> shift[wdt->stage]) & 3u;
}

static bool timer_group_advance_wdt(flexe_timer_group_t *timer_group,
                                    unsigned group, uint64_t ticks)
{
    timer_group_state_t *state = &timer_group->group[group];
    timer_group_wdt_t *wdt = &state->wdt;
    bool changed = false;
    while (ticks != 0u && timer_group_wdt_active(wdt)) {
        uint64_t hold = wdt->config[2u + wdt->stage];
        if (hold == 0u) hold = 1u;
        uint64_t remaining = wdt->stage_ticks < hold ?
                             hold - wdt->stage_ticks : 1u;
        if (ticks < remaining) {
            wdt->stage_ticks += ticks;
            break;
        }

        ticks -= remaining;
        wdt->stage_ticks = 0u;
        unsigned action = timer_group_wdt_action(wdt);
        changed = true;
        if (action == 1u) {
            state->int_raw |= 1u << WDT_INTERRUPT_EVENT;
        } else if (action == FLEXE_TIMER_GROUP_WDT_RESET_CPU ||
                   action == FLEXE_TIMER_GROUP_WDT_RESET_SYSTEM) {
            wdt->stage = TIMER_GROUP_WDT_STAGE_COUNT;
            if (timer_group->reset_requested)
                timer_group->reset_requested(
                    timer_group->reset_ctx, group,
                    (flexe_timer_group_wdt_action_t)action);
            break;
        }

        if (wdt->stage + 1u < TIMER_GROUP_WDT_STAGE_COUNT)
            wdt->stage++;
        else
            wdt->stage = TIMER_GROUP_WDT_STAGE_COUNT;
    }
    return changed;
}

static bool timer_group_sync(flexe_timer_group_t *timer_group)
{
    uint64_t now = timer_group_clock_now(timer_group);
    uint64_t elapsed = now >= timer_group->last_clock_cycles ?
                       now - timer_group->last_clock_cycles : 0u;
    timer_group->last_clock_cycles = now;
    if (elapsed == 0u) return false;

    uint64_t cpu_hz = timer_group_cpu_hz(timer_group, NULL);
    const flexe_timer_group_desc_t *desc =
        &timer_group->target->timer_group;
    bool changed = false;
    for (unsigned group = 0u; group < desc->group_count; group++) {
        timer_group_state_t *state = &timer_group->group[group];
        for (unsigned index = 0u; index < desc->timer_count; index++) {
            timer_group_timer_t *timer = &state->timer[index];
            if (!(timer->config & TIMER_CONFIG_ENABLE)) continue;
            uint64_t source_hz =
                timer_group_timer_source_hz(timer_group, timer);
            uint64_t denominator =
                cpu_hz * timer_group_timer_divider(timer);
            uint64_t ticks = timer_group_scaled_ticks(
                elapsed, source_hz, denominator,
                &timer->tick_denominator, &timer->tick_remainder);
            changed |= timer_group_advance_timer(
                timer_group, group, index, ticks);
        }

        timer_group_wdt_t *wdt = &state->wdt;
        if (timer_group_wdt_active(wdt)) {
            uint64_t denominator =
                cpu_hz * timer_group_wdt_prescaler(wdt);
            uint64_t ticks = timer_group_scaled_ticks(
                elapsed, desc->apb_clock_hz, denominator,
                &wdt->tick_denominator, &wdt->tick_remainder);
            changed |= timer_group_advance_wdt(
                timer_group, group, ticks);
        }
        timer_group_update_irqs(timer_group, group);
    }
    return changed;
}

static uint32_t timer_group_cycles_until_ticks(uint64_t ticks,
                                               uint64_t source_hz,
                                               uint64_t denominator,
                                               uint64_t remainder)
{
    if (ticks == 0u || source_hz == 0u || denominator == 0u) return 1u;
    uint64_t max_numerator = (uint64_t)INT32_MAX * source_hz;
    if (max_numerator <= UINT64_MAX - remainder)
        max_numerator += remainder;
    else
        max_numerator = UINT64_MAX;
    if (ticks > max_numerator / denominator)
        return (uint32_t)INT32_MAX;

    uint64_t needed = ticks * denominator;
    if (needed <= remainder) return 1u;
    needed -= remainder;
    uint64_t cycles = needed / source_hz +
                      (needed % source_hz != 0u);
    if (cycles == 0u) cycles = 1u;
    if (cycles > (uint64_t)INT32_MAX) cycles = (uint64_t)INT32_MAX;
    return (uint32_t)cycles;
}

static uint64_t timer_group_alarm_distance(
    const flexe_timer_group_t *timer_group,
    const timer_group_timer_t *timer)
{
    uint64_t mask = timer_group_counter_mask(timer_group);
    uint64_t distance = (timer->config & TIMER_CONFIG_INCREASE) ?
        (timer->alarm - timer->counter) & mask :
        (timer->counter - timer->alarm) & mask;
    return distance != 0u ? distance : mask + 1u;
}

static int timer_group_decode(const flexe_timer_group_t *timer_group,
                              uint32_t addr, uint32_t *off)
{
    const flexe_timer_group_desc_t *desc =
        &timer_group->target->timer_group;
    for (unsigned group = 0u; group < desc->group_count; group++) {
        if (addr >= desc->base[group] &&
            addr - desc->base[group] < desc->register_size) {
            *off = addr - desc->base[group];
            return (int)group;
        }
    }
    return -1;
}

static uint32_t timer_group_read(void *ctx, uint32_t addr)
{
    flexe_timer_group_t *timer_group = ctx;
    uint32_t off = 0u;
    int group = timer_group_decode(timer_group, addr, &off);
    if (group < 0 || (off & 3u) != 0u) goto fallback;
    bool changed = timer_group_sync(timer_group);
    if (changed && timer_group->state_changed)
        timer_group->state_changed(timer_group->state_ctx);
    timer_group_state_t *state = &timer_group->group[group];

    if (off < TIMER_GROUP_WDT_CONFIG0_OFF) {
        unsigned index = off / TIMER_GROUP_TIMER_STRIDE;
        uint32_t relative = off % TIMER_GROUP_TIMER_STRIDE;
        if (index < timer_group->target->timer_group.timer_count) {
            const timer_group_timer_t *timer = &state->timer[index];
            switch (relative) {
            case 0x00u: return timer->config;
            case 0x04u: return (uint32_t)timer->latched;
            case 0x08u: return (uint32_t)(timer->latched >> 32) &
                               timer_group_high_mask(timer_group);
            case 0x0Cu: return 0u;
            case 0x10u: return (uint32_t)timer->alarm;
            case 0x14u: return (uint32_t)(timer->alarm >> 32) &
                               timer_group_high_mask(timer_group);
            case 0x18u: return (uint32_t)timer->load;
            case 0x1Cu: return (uint32_t)(timer->load >> 32) &
                               timer_group_high_mask(timer_group);
            case 0x20u: return 0u;
            default: break;
            }
        }
    }

    if (off >= TIMER_GROUP_WDT_CONFIG0_OFF &&
        off <= TIMER_GROUP_WDT_CONFIG5_OFF)
        return state->wdt.config[
            (off - TIMER_GROUP_WDT_CONFIG0_OFF) / 4u];
    switch (off) {
    case TIMER_GROUP_WDT_FEED_OFF: return 0u;
    case TIMER_GROUP_WDT_PROTECT_OFF: return state->wdt.protect;
    case TIMER_GROUP_INT_ENA_OFF: return state->int_enable;
    case TIMER_GROUP_INT_RAW_OFF: return state->int_raw;
    case TIMER_GROUP_INT_ST_OFF:
        return state->int_raw & state->int_enable;
    case TIMER_GROUP_INT_CLR_OFF: return 0u;
    case TIMER_GROUP_DATE_OFF: return state->date;
    case TIMER_GROUP_REGCLK_OFF: return state->regclk;
    default: break;
    }

fallback:
    return timer_group->fallback_read ?
        timer_group->fallback_read(timer_group->fallback_ctx, addr) : 0u;
}

static void timer_group_notify(flexe_timer_group_t *timer_group)
{
    if (timer_group->state_changed)
        timer_group->state_changed(timer_group->state_ctx);
}

static void timer_group_write(void *ctx, uint32_t addr, uint32_t value)
{
    flexe_timer_group_t *timer_group = ctx;
    uint32_t off = 0u;
    int group = timer_group_decode(timer_group, addr, &off);
    if (group < 0 || (off & 3u) != 0u) goto fallback;
    (void)timer_group_sync(timer_group);
    timer_group_state_t *state = &timer_group->group[group];
    const flexe_timer_group_desc_t *desc =
        &timer_group->target->timer_group;
    uint64_t mask = timer_group_counter_mask(timer_group);

    if (off < TIMER_GROUP_WDT_CONFIG0_OFF) {
        unsigned index = off / TIMER_GROUP_TIMER_STRIDE;
        uint32_t relative = off % TIMER_GROUP_TIMER_STRIDE;
        if (index < desc->timer_count) {
            timer_group_timer_t *timer = &state->timer[index];
            switch (relative) {
            case 0x00u: {
                uint32_t old = timer->config;
                timer->config = value & desc->timer_config_writable_mask;
                if ((value & TIMER_CONFIG_DIVCNT_RST) ||
                    ((old ^ timer->config) &
                     (TIMER_CONFIG_USE_XTAL |
                      TIMER_CONFIG_DIVIDER_MASK))) {
                    timer->tick_remainder = 0u;
                    timer->tick_denominator = 0u;
                }
                timer_group_update_irqs(timer_group, (unsigned)group);
                timer_group_notify(timer_group);
                return;
            }
            case 0x04u:
            case 0x08u:
                return; /* Captured values are read-only. */
            case 0x0Cu:
                timer->latched = timer->counter;
                timer_group_notify(timer_group);
                return;
            case 0x10u:
                timer->alarm = (timer->alarm &
                                UINT64_C(0xFFFFFFFF00000000)) | value;
                timer->alarm &= mask;
                timer_group_notify(timer_group);
                return;
            case 0x14u:
                timer->alarm = (timer->alarm & UINT32_MAX) |
                    ((uint64_t)(value & timer_group_high_mask(timer_group))
                     << 32);
                timer_group_notify(timer_group);
                return;
            case 0x18u:
                timer->load = (timer->load &
                               UINT64_C(0xFFFFFFFF00000000)) | value;
                timer->load &= mask;
                timer_group_notify(timer_group);
                return;
            case 0x1Cu:
                timer->load = (timer->load & UINT32_MAX) |
                    ((uint64_t)(value & timer_group_high_mask(timer_group))
                     << 32);
                timer_group_notify(timer_group);
                return;
            case 0x20u:
                timer->counter = timer->load & mask;
                timer->tick_remainder = 0u;
                timer->tick_denominator = 0u;
                timer_group_notify(timer_group);
                return;
            default: break;
            }
        }
    }

    if (off >= TIMER_GROUP_WDT_CONFIG0_OFF &&
        off <= TIMER_GROUP_WDT_CONFIG5_OFF) {
        unsigned index = (off - TIMER_GROUP_WDT_CONFIG0_OFF) / 4u;
        timer_group_wdt_t *wdt = &state->wdt;
        if (wdt->protect == desc->wdt_write_protect_key) {
            bool was_active = timer_group_wdt_active(wdt);
            uint32_t old = wdt->config[index];
            wdt->config[index] =
                value & desc->wdt_config_writable_mask[index];
            if (index == 1u && old != wdt->config[index]) {
                wdt->tick_remainder = 0u;
                wdt->tick_denominator = 0u;
            }
            if (index == 0u && !was_active &&
                timer_group_wdt_active(wdt)) {
                wdt->stage = 0u;
                wdt->stage_ticks = 0u;
                wdt->tick_remainder = 0u;
                wdt->tick_denominator = 0u;
            }
        }
        timer_group_update_irqs(timer_group, (unsigned)group);
        timer_group_notify(timer_group);
        return;
    }

    switch (off) {
    case TIMER_GROUP_WDT_FEED_OFF:
        if (state->wdt.protect == desc->wdt_write_protect_key) {
            state->wdt.stage = 0u;
            state->wdt.stage_ticks = 0u;
            state->wdt.tick_remainder = 0u;
            state->wdt.tick_denominator = 0u;
        }
        timer_group_notify(timer_group);
        return;
    case TIMER_GROUP_WDT_PROTECT_OFF:
        state->wdt.protect = value;
        timer_group_notify(timer_group);
        return;
    case TIMER_GROUP_INT_ENA_OFF:
        state->int_enable = value & TIMER_GROUP_INTERRUPT_MASK;
        timer_group_update_irqs(timer_group, (unsigned)group);
        timer_group_notify(timer_group);
        return;
    case TIMER_GROUP_INT_RAW_OFF:
    case TIMER_GROUP_INT_CLR_OFF:
        state->int_raw &= ~(value & TIMER_GROUP_INTERRUPT_MASK);
        timer_group_update_irqs(timer_group, (unsigned)group);
        timer_group_notify(timer_group);
        return;
    case TIMER_GROUP_INT_ST_OFF:
        return; /* Masked status is read-only. */
    case TIMER_GROUP_DATE_OFF:
        state->date = value & desc->date_writable_mask;
        timer_group_notify(timer_group);
        return;
    case TIMER_GROUP_REGCLK_OFF:
        state->regclk = value & desc->regclk_writable_mask;
        timer_group_notify(timer_group);
        return;
    default: break;
    }

fallback:
    if (timer_group->fallback_write)
        timer_group->fallback_write(timer_group->fallback_ctx, addr, value);
}

flexe_timer_group_t *flexe_timer_group_create(
    xtensa_mem_t *mem, mmio_read_fn fallback_read,
    mmio_write_fn fallback_write, void *fallback_ctx,
    flexe_timer_group_state_fn state_changed, void *state_ctx,
    flexe_timer_group_irq_fn irq_changed, void *irq_ctx,
    flexe_timer_group_reset_fn reset_requested, void *reset_ctx)
{
    if (!mem) return NULL;
    const flexe_target_desc_t *target = mem_target(mem);
    if (!timer_group_geometry_valid(target)) return NULL;

    flexe_timer_group_t *timer_group = calloc(1u, sizeof(*timer_group));
    if (!timer_group) return NULL;
    timer_group->mem = mem;
    timer_group->target = target;
    timer_group->fallback_read = fallback_read;
    timer_group->fallback_write = fallback_write;
    timer_group->fallback_ctx = fallback_ctx;
    timer_group->state_changed = state_changed;
    timer_group->state_ctx = state_ctx;
    timer_group->irq_changed = irq_changed;
    timer_group->irq_ctx = irq_ctx;
    timer_group->reset_requested = reset_requested;
    timer_group->reset_ctx = reset_ctx;

    const flexe_timer_group_desc_t *desc = &target->timer_group;
    for (unsigned group = 0u; group < desc->group_count; group++) {
        timer_group_state_t *state = &timer_group->group[group];
        state->date = desc->date_reset;
        state->regclk = desc->regclk_reset;
        for (unsigned timer = 0u; timer < desc->timer_count; timer++)
            state->timer[timer].config = desc->timer_config_reset;
        for (unsigned index = 0u; index < 6u; index++)
            state->wdt.config[index] = desc->wdt_config_reset[index];
        state->wdt.protect = desc->wdt_write_protect_key;
        if (mem_register_mmio_range(mem, desc->base[group],
                                    desc->register_size,
                                    timer_group_read, timer_group_write,
                                    timer_group) != 0) {
            for (unsigned old = 0u; old < group; old++)
                (void)mem_register_mmio_range(
                    mem, desc->base[old], desc->register_size,
                    fallback_read, fallback_write, fallback_ctx);
            free(timer_group);
            return NULL;
        }
    }
    return timer_group;
}

void flexe_timer_group_destroy(flexe_timer_group_t *timer_group)
{
    if (!timer_group) return;
    const flexe_timer_group_desc_t *desc =
        &timer_group->target->timer_group;
    for (unsigned group = 0u; group < desc->group_count; group++) {
        for (unsigned event = 0u;
             event < FLEXE_TARGET_TIMER_GROUP_EVENT_MAX; event++) {
            if (timer_group->group[group].irq_level[event] &&
                timer_group->irq_changed)
                timer_group->irq_changed(timer_group->irq_ctx, group,
                                         event, false);
        }
        (void)mem_register_mmio_range(
            timer_group->mem, desc->base[group], desc->register_size,
            timer_group->fallback_read, timer_group->fallback_write,
            timer_group->fallback_ctx);
    }
    free(timer_group);
}

void flexe_timer_group_attach_cpus(flexe_timer_group_t *timer_group,
                                   xtensa_cpu_t *cpu0,
                                   xtensa_cpu_t *cpu1)
{
    if (!timer_group) return;
    (void)timer_group_sync(timer_group);
    timer_group->cpu[0] = cpu0;
    timer_group->cpu[1] = cpu1;
    for (unsigned core = 0u; core < 2u; core++) {
        xtensa_cpu_t *cpu = core == 0u ? cpu0 : cpu1;
        timer_group->clock.core_cycles[core] =
            timer_group->clock.cycles;
        timer_group->clock.last_ccount[core] = cpu ? cpu->ccount : 0u;
        timer_group->clock.valid[core] = cpu != NULL;
    }
    timer_group->last_clock_cycles = timer_group->clock.cycles;
    timer_group_notify(timer_group);
}

uint32_t flexe_timer_group_next_event(flexe_timer_group_t *timer_group,
                                      xtensa_cpu_t *cpu)
{
    if (!timer_group || !cpu) return UINT32_MAX;
    (void)timer_group_sync(timer_group);
    const flexe_timer_group_desc_t *desc =
        &timer_group->target->timer_group;
    uint64_t cpu_hz = timer_group_cpu_hz(timer_group, cpu);
    uint32_t best = (uint32_t)INT32_MAX;
    bool have = false;

    for (unsigned group = 0u; group < desc->group_count; group++) {
        timer_group_state_t *state = &timer_group->group[group];
        for (unsigned index = 0u; index < desc->timer_count; index++) {
            timer_group_timer_t *timer = &state->timer[index];
            if (!(timer->config & TIMER_CONFIG_ENABLE) ||
                !(timer->config & TIMER_CONFIG_ALARM_EN) ||
                !(state->int_enable & (1u << index)))
                continue;
            uint64_t source_hz =
                timer_group_timer_source_hz(timer_group, timer);
            uint64_t denominator =
                cpu_hz * timer_group_timer_divider(timer);
            uint32_t cycles = timer_group_cycles_until_ticks(
                timer_group_alarm_distance(timer_group, timer),
                source_hz, denominator, timer->tick_remainder);
            if (!have || cycles < best) {
                best = cycles;
                have = true;
            }
        }

        timer_group_wdt_t *wdt = &state->wdt;
        if (timer_group_wdt_active(wdt)) {
            uint64_t hold = wdt->config[2u + wdt->stage];
            if (hold == 0u) hold = 1u;
            uint64_t ticks = wdt->stage_ticks < hold ?
                             hold - wdt->stage_ticks : 1u;
            uint64_t denominator =
                cpu_hz * timer_group_wdt_prescaler(wdt);
            uint32_t cycles = timer_group_cycles_until_ticks(
                ticks, desc->apb_clock_hz, denominator,
                wdt->tick_remainder);
            if (!have || cycles < best) {
                best = cycles;
                have = true;
            }
        }
    }

    return have ? cpu->ccount + best : UINT32_MAX;
}

void flexe_timer_group_eval(flexe_timer_group_t *timer_group)
{
    if (!timer_group) return;
    if (timer_group_sync(timer_group)) timer_group_notify(timer_group);
}
