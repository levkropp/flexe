#include "systimer.h"

#include "xtensa.h"

#include <limits.h>
#include <stdbool.h>
#include <stdlib.h>

/* ESP32-S2/S3-style SYSTIMER V1 register layout. */
#define SYSTIMER_CONF_OFF              0x000u
#define SYSTIMER_UNIT_OP_OFF           0x004u
#define SYSTIMER_UNIT_OP_STRIDE        0x004u
#define SYSTIMER_UNIT_LOAD_VALUE_OFF   0x00Cu
#define SYSTIMER_UNIT_LOAD_STRIDE      0x008u
#define SYSTIMER_TARGET_VALUE_OFF      0x01Cu
#define SYSTIMER_TARGET_VALUE_STRIDE   0x008u
#define SYSTIMER_TARGET_CONF_OFF       0x034u
#define SYSTIMER_UNIT_VALUE_OFF        0x040u
#define SYSTIMER_UNIT_VALUE_STRIDE     0x008u
#define SYSTIMER_COMP_LOAD_OFF         0x050u
#define SYSTIMER_UNIT_LOAD_OFF         0x05Cu
#define SYSTIMER_INT_ENA_OFF           0x064u
#define SYSTIMER_INT_RAW_OFF           0x068u
#define SYSTIMER_INT_CLR_OFF           0x06Cu
#define SYSTIMER_INT_ST_OFF            0x070u
#define SYSTIMER_REAL_TARGET_OFF       0x074u
#define SYSTIMER_REAL_TARGET_STRIDE    0x008u
#define SYSTIMER_DATE_OFF              0x0FCu

#define SYSTIMER_CONF_WRITABLE         0xFFC00001u
#define SYSTIMER_COUNTER_ENABLE(unit)  (1u << (30u - (unit)))
#define SYSTIMER_ALARM_ENABLE(alarm)   (1u << (24u - (alarm)))
#define SYSTIMER_VALUE_VALID           (1u << 29)
#define SYSTIMER_VALUE_UPDATE          (1u << 30)
#define SYSTIMER_TARGET_PERIOD_MASK    0x03FFFFFFu
#define SYSTIMER_TARGET_PERIOD_MODE    (1u << 30)
#define SYSTIMER_TARGET_UNIT_SELECT    (1u << 31)
#define SYSTIMER_TARGET_CONF_WRITABLE  0xC3FFFFFFu
#define SYSTIMER_HIGH_MASK             0x000FFFFFu

typedef struct {
    uint64_t value;
    uint64_t latched;
    uint64_t load;
    bool valid;
} systimer_counter_t;

typedef struct {
    uint64_t staged_target;
    uint64_t real_target;
    uint32_t config;
    uint32_t period;
    bool loaded;
    bool irq_level;
    bool target_dirty;
} systimer_alarm_t;

typedef struct {
    uint64_t cycles;
    uint64_t core_cycles[2];
    uint32_t last_ccount[2];
    bool valid[2];
} systimer_clock_t;

struct flexe_systimer {
    xtensa_mem_t *mem;
    const flexe_target_desc_t *target;
    mmio_read_fn fallback_read;
    mmio_write_fn fallback_write;
    void *fallback_ctx;
    flexe_systimer_state_fn state_changed;
    void *state_ctx;
    flexe_systimer_irq_fn irq_changed;
    void *irq_ctx;
    xtensa_cpu_t *cpu[2];
    systimer_clock_t clock;
    uint64_t last_clock_cycles;
    uint64_t tick_remainder;
    uint64_t tick_denominator;
    uint32_t config;
    uint32_t int_enable;
    uint32_t int_raw;
    uint32_t date;
    systimer_counter_t counter[FLEXE_TARGET_SYSTIMER_COUNTER_MAX];
    systimer_alarm_t alarm[FLEXE_TARGET_SYSTIMER_ALARM_MAX];
};

static bool systimer_geometry_valid(const flexe_target_desc_t *target)
{
    if (!target || !(target->capabilities & FLEXE_TARGET_CAP_SYSTIMER_V1))
        return false;
    const flexe_systimer_desc_t *desc = &target->systimer;
    if ((desc->base & 0xFFFu) != 0u ||
        desc->register_size < SYSTIMER_DATE_OFF + sizeof(uint32_t) ||
        desc->base < target->peripheral_start ||
        desc->base >= target->peripheral_end ||
        desc->register_size > target->peripheral_end - desc->base ||
        desc->counter_frequency_hz == 0u ||
        desc->counter_count != FLEXE_TARGET_SYSTIMER_COUNTER_MAX ||
        desc->alarm_count != FLEXE_TARGET_SYSTIMER_ALARM_MAX ||
        desc->counter_width < 33u || desc->counter_width > 63u ||
        (desc->config_reset & ~SYSTIMER_CONF_WRITABLE) != 0u)
        return false;
    return true;
}

static uint64_t systimer_mask(const flexe_systimer_t *systimer)
{
    return (UINT64_C(1) << systimer->target->systimer.counter_width) - 1u;
}

static uint64_t systimer_clock_now(flexe_systimer_t *systimer)
{
    systimer_clock_t *clock = &systimer->clock;
    for (unsigned core = 0; core < 2u; core++) {
        xtensa_cpu_t *cpu = systimer->cpu[core];
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

static uint64_t systimer_cpu_hz(const flexe_systimer_t *systimer,
                                const xtensa_cpu_t *preferred)
{
    const xtensa_cpu_t *cpu = preferred ? preferred : systimer->cpu[0];
    if (!cpu) cpu = systimer->cpu[1];
    uint64_t mhz = cpu ? xtensa_cpu_freq_mhz(cpu) :
                   systimer->target->default_cpu_frequency_mhz;
    if (mhz == 0u) mhz = 160u;
    return mhz * UINT64_C(1000000);
}

static bool systimer_reached(const flexe_systimer_t *systimer,
                             uint64_t current, uint64_t target)
{
    uint64_t mask = systimer_mask(systimer);
    uint64_t half = UINT64_C(1) <<
                    (systimer->target->systimer.counter_width - 1u);
    return ((current - target) & mask) < half;
}

static void systimer_update_irqs(flexe_systimer_t *systimer)
{
    for (unsigned alarm = 0;
         alarm < systimer->target->systimer.alarm_count; alarm++) {
        bool level = (systimer->int_raw & systimer->int_enable &
                      (1u << alarm)) != 0u;
        if (level == systimer->alarm[alarm].irq_level) continue;
        systimer->alarm[alarm].irq_level = level;
        if (systimer->irq_changed)
            systimer->irq_changed(systimer->irq_ctx, alarm, level);
    }
}

static void systimer_evaluate_alarms(flexe_systimer_t *systimer)
{
    uint64_t mask = systimer_mask(systimer);
    for (unsigned index = 0;
         index < systimer->target->systimer.alarm_count; index++) {
        systimer_alarm_t *alarm = &systimer->alarm[index];
        if (!alarm->loaded ||
            !(systimer->config & SYSTIMER_ALARM_ENABLE(index)))
            continue;
        unsigned unit = (alarm->config & SYSTIMER_TARGET_UNIT_SELECT) ? 1u : 0u;
        if (!(systimer->config & SYSTIMER_COUNTER_ENABLE(unit))) continue;
        uint64_t current = systimer->counter[unit].value;
        if (!systimer_reached(systimer, current, alarm->real_target))
            continue;

        systimer->int_raw |= 1u << index;
        if (alarm->config & SYSTIMER_TARGET_PERIOD_MODE) {
            uint64_t period = alarm->period;
            if (period == 0u) {
                systimer->config &= ~SYSTIMER_ALARM_ENABLE(index);
                continue;
            }
            uint64_t behind = (current - alarm->real_target) & mask;
            uint64_t periods = behind / period + 1u;
            alarm->real_target =
                (alarm->real_target + periods * period) & mask;
        } else {
            /* V1 one-shot comparators clear WORK_EN after the match. */
            systimer->config &= ~SYSTIMER_ALARM_ENABLE(index);
        }
    }
    systimer_update_irqs(systimer);
}

static uint64_t systimer_scaled_ticks(flexe_systimer_t *systimer,
                                      uint64_t elapsed_cycles,
                                      uint64_t cpu_hz)
{
    uint64_t timer_hz = systimer->target->systimer.counter_frequency_hz;
    if (systimer->tick_denominator != cpu_hz) {
        systimer->tick_denominator = cpu_hz;
        systimer->tick_remainder = 0u;
    }
    uint64_t quotient = elapsed_cycles / cpu_hz;
    uint64_t remainder = elapsed_cycles % cpu_hz;
    uint64_t ticks = quotient > UINT64_MAX / timer_hz ?
                     UINT64_MAX : quotient * timer_hz;
    uint64_t fraction = remainder * timer_hz;
    if (fraction > UINT64_MAX - systimer->tick_remainder)
        fraction = UINT64_MAX;
    else
        fraction += systimer->tick_remainder;
    if (ticks <= UINT64_MAX - fraction / cpu_hz)
        ticks += fraction / cpu_hz;
    else
        ticks = UINT64_MAX;
    systimer->tick_remainder = fraction % cpu_hz;
    return ticks;
}

static void systimer_sync(flexe_systimer_t *systimer)
{
    uint64_t now = systimer_clock_now(systimer);
    uint64_t elapsed = now >= systimer->last_clock_cycles ?
                       now - systimer->last_clock_cycles : 0u;
    systimer->last_clock_cycles = now;
    if (elapsed == 0u) {
        systimer_evaluate_alarms(systimer);
        return;
    }

    uint64_t ticks = systimer_scaled_ticks(
        systimer, elapsed, systimer_cpu_hz(systimer, NULL));
    uint64_t mask = systimer_mask(systimer);
    for (unsigned unit = 0;
         unit < systimer->target->systimer.counter_count; unit++) {
        if (systimer->config & SYSTIMER_COUNTER_ENABLE(unit))
            systimer->counter[unit].value =
                (systimer->counter[unit].value + ticks) & mask;
    }
    systimer_evaluate_alarms(systimer);
}

static int systimer_index(uint32_t off, uint32_t base, uint32_t stride,
                          unsigned count, uint32_t *part)
{
    if (off < base) return -1;
    uint32_t relative = off - base;
    unsigned index = relative / stride;
    uint32_t within = relative % stride;
    if (index >= count || (within & 3u) != 0u || within > 4u) return -1;
    if (part) *part = within / 4u;
    return (int)index;
}

static uint32_t systimer_read(void *ctx, uint32_t addr)
{
    flexe_systimer_t *systimer = ctx;
    const flexe_systimer_desc_t *desc = &systimer->target->systimer;
    uint32_t off = addr - desc->base;
    if ((off & 3u) != 0u) goto fallback;
    systimer_sync(systimer);

    if (off == SYSTIMER_CONF_OFF) return systimer->config;
    if (off >= SYSTIMER_UNIT_OP_OFF &&
        off < SYSTIMER_UNIT_OP_OFF + desc->counter_count * 4u) {
        unsigned unit = (off - SYSTIMER_UNIT_OP_OFF) / 4u;
        return systimer->counter[unit].valid ? SYSTIMER_VALUE_VALID : 0u;
    }

    uint32_t part = 0u;
    int index = systimer_index(off, SYSTIMER_UNIT_LOAD_VALUE_OFF,
                               SYSTIMER_UNIT_LOAD_STRIDE,
                               desc->counter_count, &part);
    if (index >= 0) {
        uint64_t value = systimer->counter[index].load;
        return part == 0u ? (uint32_t)(value >> 32) & SYSTIMER_HIGH_MASK :
                            (uint32_t)value;
    }
    index = systimer_index(off, SYSTIMER_TARGET_VALUE_OFF,
                           SYSTIMER_TARGET_VALUE_STRIDE,
                           desc->alarm_count, &part);
    if (index >= 0) {
        uint64_t value = systimer->alarm[index].staged_target;
        return part == 0u ? (uint32_t)(value >> 32) & SYSTIMER_HIGH_MASK :
                            (uint32_t)value;
    }
    if (off >= SYSTIMER_TARGET_CONF_OFF &&
        off < SYSTIMER_TARGET_CONF_OFF + desc->alarm_count * 4u)
        return systimer->alarm[(off - SYSTIMER_TARGET_CONF_OFF) / 4u].config;

    index = systimer_index(off, SYSTIMER_UNIT_VALUE_OFF,
                           SYSTIMER_UNIT_VALUE_STRIDE,
                           desc->counter_count, &part);
    if (index >= 0) {
        uint64_t value = systimer->counter[index].latched;
        return part == 0u ? (uint32_t)(value >> 32) & SYSTIMER_HIGH_MASK :
                            (uint32_t)value;
    }
    if (off >= SYSTIMER_COMP_LOAD_OFF &&
        off < SYSTIMER_COMP_LOAD_OFF + desc->alarm_count * 4u)
        return 0u; /* Write-trigger registers always read zero. */
    if (off >= SYSTIMER_UNIT_LOAD_OFF &&
        off < SYSTIMER_UNIT_LOAD_OFF + desc->counter_count * 4u)
        return 0u;
    if (off == SYSTIMER_INT_ENA_OFF) return systimer->int_enable;
    if (off == SYSTIMER_INT_RAW_OFF) return systimer->int_raw;
    if (off == SYSTIMER_INT_CLR_OFF) return 0u;
    if (off == SYSTIMER_INT_ST_OFF)
        return systimer->int_raw & systimer->int_enable;

    index = systimer_index(off, SYSTIMER_REAL_TARGET_OFF,
                           SYSTIMER_REAL_TARGET_STRIDE,
                           desc->alarm_count, &part);
    if (index >= 0) {
        uint64_t value = systimer->alarm[index].real_target;
        /* The read-only actual-target registers are LO followed by HI. */
        return part == 0u ? (uint32_t)value :
                            (uint32_t)(value >> 32) & SYSTIMER_HIGH_MASK;
    }
    if (off == SYSTIMER_DATE_OFF) return systimer->date;

fallback:
    return systimer->fallback_read ?
        systimer->fallback_read(systimer->fallback_ctx, addr) : 0u;
}

static void systimer_write(void *ctx, uint32_t addr, uint32_t value)
{
    flexe_systimer_t *systimer = ctx;
    const flexe_systimer_desc_t *desc = &systimer->target->systimer;
    uint32_t off = addr - desc->base;
    if ((off & 3u) != 0u) goto fallback;
    systimer_sync(systimer);

    if (off == SYSTIMER_CONF_OFF) {
        systimer->config = value & SYSTIMER_CONF_WRITABLE;
        systimer_evaluate_alarms(systimer);
        goto changed;
    }
    if (off >= SYSTIMER_UNIT_OP_OFF &&
        off < SYSTIMER_UNIT_OP_OFF + desc->counter_count * 4u) {
        unsigned unit = (off - SYSTIMER_UNIT_OP_OFF) / 4u;
        if (value & SYSTIMER_VALUE_VALID)
            systimer->counter[unit].valid = false;
        if (value & SYSTIMER_VALUE_UPDATE) {
            systimer->counter[unit].latched =
                systimer->counter[unit].value;
            systimer->counter[unit].valid = true;
        }
        goto changed;
    }

    uint32_t part = 0u;
    int index = systimer_index(off, SYSTIMER_UNIT_LOAD_VALUE_OFF,
                               SYSTIMER_UNIT_LOAD_STRIDE,
                               desc->counter_count, &part);
    if (index >= 0) {
        uint64_t *load = &systimer->counter[index].load;
        if (part == 0u)
            *load = (*load & UINT32_MAX) |
                    ((uint64_t)(value & SYSTIMER_HIGH_MASK) << 32);
        else
            *load = (*load & (UINT64_C(0xFFFFF) << 32)) | value;
        goto changed;
    }
    index = systimer_index(off, SYSTIMER_TARGET_VALUE_OFF,
                           SYSTIMER_TARGET_VALUE_STRIDE,
                           desc->alarm_count, &part);
    if (index >= 0) {
        uint64_t *target = &systimer->alarm[index].staged_target;
        if (part == 0u)
            *target = (*target & UINT32_MAX) |
                      ((uint64_t)(value & SYSTIMER_HIGH_MASK) << 32);
        else
            *target = (*target & (UINT64_C(0xFFFFF) << 32)) | value;
        systimer->alarm[index].target_dirty = true;
        goto changed;
    }
    if (off >= SYSTIMER_TARGET_CONF_OFF &&
        off < SYSTIMER_TARGET_CONF_OFF + desc->alarm_count * 4u) {
        systimer->alarm[(off - SYSTIMER_TARGET_CONF_OFF) / 4u].config =
            value & SYSTIMER_TARGET_CONF_WRITABLE;
        systimer_evaluate_alarms(systimer);
        goto changed;
    }
    if (off >= SYSTIMER_UNIT_VALUE_OFF &&
        off < SYSTIMER_UNIT_VALUE_OFF + desc->counter_count * 8u)
        return; /* Read-only captured values. */

    if (off >= SYSTIMER_COMP_LOAD_OFF &&
        off < SYSTIMER_COMP_LOAD_OFF + desc->alarm_count * 4u) {
        unsigned alarm_index = (off - SYSTIMER_COMP_LOAD_OFF) / 4u;
        if (value & 1u) {
            systimer_alarm_t *alarm = &systimer->alarm[alarm_index];
            unsigned unit =
                (alarm->config & SYSTIMER_TARGET_UNIT_SELECT) ? 1u : 0u;
            alarm->period = alarm->config & SYSTIMER_TARGET_PERIOD_MASK;
            /* COMPn_LOAD synchronizes both target and period shadows. The
             * ESP-IDF HAL deliberately writes a period and pulses LOAD while
             * PERIOD_MODE is still clear, then enables the comparator before
             * selecting periodic mode. When no one-shot target was staged,
             * the loaded comparator must therefore point one period into the
             * future; treating its untouched target shadow as zero raises an
             * immediate one-shot and clears WORK_EN during scheduler setup. */
            if ((alarm->config & SYSTIMER_TARGET_PERIOD_MODE) ||
                !alarm->target_dirty) {
                uint64_t period = alarm->period;
                alarm->real_target =
                    (systimer->counter[unit].value + period) &
                    systimer_mask(systimer);
            } else {
                alarm->real_target = alarm->staged_target &
                                     systimer_mask(systimer);
            }
            alarm->target_dirty = false;
            alarm->loaded = true;
            systimer_evaluate_alarms(systimer);
        }
        goto changed;
    }
    if (off >= SYSTIMER_UNIT_LOAD_OFF &&
        off < SYSTIMER_UNIT_LOAD_OFF + desc->counter_count * 4u) {
        unsigned unit = (off - SYSTIMER_UNIT_LOAD_OFF) / 4u;
        if (value & 1u) {
            systimer->counter[unit].value =
                systimer->counter[unit].load & systimer_mask(systimer);
            systimer_evaluate_alarms(systimer);
        }
        goto changed;
    }
    if (off == SYSTIMER_INT_ENA_OFF) {
        systimer->int_enable = value &
            ((1u << desc->alarm_count) - 1u);
        systimer_update_irqs(systimer);
        goto changed;
    }
    if (off == SYSTIMER_INT_RAW_OFF || off == SYSTIMER_INT_CLR_OFF) {
        systimer->int_raw &= ~(value & ((1u << desc->alarm_count) - 1u));
        systimer_update_irqs(systimer);
        goto changed;
    }
    if (off == SYSTIMER_INT_ST_OFF ||
        (off >= SYSTIMER_REAL_TARGET_OFF &&
         off < SYSTIMER_REAL_TARGET_OFF + desc->alarm_count * 8u))
        return; /* Read-only status. */
    if (off == SYSTIMER_DATE_OFF) {
        systimer->date = value;
        goto changed;
    }

fallback:
    if (systimer->fallback_write)
        systimer->fallback_write(systimer->fallback_ctx, addr, value);
    return;

changed:
    if (systimer->state_changed)
        systimer->state_changed(systimer->state_ctx);
}

flexe_systimer_t *flexe_systimer_create(
    xtensa_mem_t *mem, mmio_read_fn fallback_read,
    mmio_write_fn fallback_write, void *fallback_ctx,
    flexe_systimer_state_fn state_changed, void *state_ctx,
    flexe_systimer_irq_fn irq_changed, void *irq_ctx)
{
    if (!mem) return NULL;
    const flexe_target_desc_t *target = mem_target(mem);
    if (!systimer_geometry_valid(target)) return NULL;

    flexe_systimer_t *systimer = calloc(1, sizeof(*systimer));
    if (!systimer) return NULL;
    systimer->mem = mem;
    systimer->target = target;
    systimer->fallback_read = fallback_read;
    systimer->fallback_write = fallback_write;
    systimer->fallback_ctx = fallback_ctx;
    systimer->state_changed = state_changed;
    systimer->state_ctx = state_ctx;
    systimer->irq_changed = irq_changed;
    systimer->irq_ctx = irq_ctx;
    systimer->config = target->systimer.config_reset;
    systimer->date = target->systimer.date_reset;

    if (mem_register_mmio_range(mem, target->systimer.base,
                                target->systimer.register_size,
                                systimer_read, systimer_write,
                                systimer) != 0) {
        free(systimer);
        return NULL;
    }
    return systimer;
}

void flexe_systimer_destroy(flexe_systimer_t *systimer)
{
    if (!systimer) return;
    for (unsigned alarm = 0;
         alarm < systimer->target->systimer.alarm_count; alarm++) {
        if (systimer->alarm[alarm].irq_level && systimer->irq_changed)
            systimer->irq_changed(systimer->irq_ctx, alarm, false);
    }
    (void)mem_register_mmio_range(systimer->mem,
        systimer->target->systimer.base,
        systimer->target->systimer.register_size, NULL, NULL, NULL);
    free(systimer);
}

void flexe_systimer_attach_cpus(flexe_systimer_t *systimer,
                                xtensa_cpu_t *cpu0,
                                xtensa_cpu_t *cpu1)
{
    if (!systimer) return;
    systimer_sync(systimer);
    systimer->cpu[0] = cpu0;
    systimer->cpu[1] = cpu1;
    for (unsigned core = 0; core < 2u; core++) {
        xtensa_cpu_t *cpu = core == 0u ? cpu0 : cpu1;
        systimer->clock.core_cycles[core] = systimer->clock.cycles;
        systimer->clock.last_ccount[core] = cpu ? cpu->ccount : 0u;
        systimer->clock.valid[core] = cpu != NULL;
    }
    systimer->last_clock_cycles = systimer->clock.cycles;
    if (systimer->state_changed)
        systimer->state_changed(systimer->state_ctx);
}

uint32_t flexe_systimer_next_event(flexe_systimer_t *systimer,
                                   xtensa_cpu_t *cpu)
{
    if (!systimer || !cpu) return UINT32_MAX;
    systimer_sync(systimer);
    const flexe_systimer_desc_t *desc = &systimer->target->systimer;
    uint64_t mask = systimer_mask(systimer);
    bool have = false;
    uint64_t nearest_ticks = 0u;
    for (unsigned index = 0; index < desc->alarm_count; index++) {
        systimer_alarm_t *alarm = &systimer->alarm[index];
        if (!alarm->loaded ||
            !(systimer->config & SYSTIMER_ALARM_ENABLE(index)))
            continue;
        unsigned unit = (alarm->config & SYSTIMER_TARGET_UNIT_SELECT) ? 1u : 0u;
        if (!(systimer->config & SYSTIMER_COUNTER_ENABLE(unit))) continue;
        uint64_t distance =
            (alarm->real_target - systimer->counter[unit].value) & mask;
        if (!have || distance < nearest_ticks) {
            nearest_ticks = distance;
            have = true;
        }
    }
    if (!have) return UINT32_MAX;

    uint64_t cpu_hz = systimer_cpu_hz(systimer, cpu);
    uint64_t timer_hz = desc->counter_frequency_hz;
    uint64_t max_ticks = ((uint64_t)INT32_MAX * timer_hz) / cpu_hz;
    if (nearest_ticks > max_ticks)
        return cpu->ccount + (uint32_t)INT32_MAX;
    uint64_t cycles = (nearest_ticks * cpu_hz + timer_hz - 1u) /
                      timer_hz;
    if (cycles > (uint64_t)INT32_MAX) cycles = (uint64_t)INT32_MAX;
    return cpu->ccount + (uint32_t)cycles;
}

void flexe_systimer_eval(flexe_systimer_t *systimer)
{
    if (systimer) systimer_sync(systimer);
}
