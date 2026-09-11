#include "rtc_cntl.h"

#include "xtensa.h"

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
    xtensa_cpu_t *cpu[2];
    rtc_clock_t clock;
    uint64_t last_clock_cycles;
    uint64_t tick_denominator;
    uint64_t tick_remainder;
    uint64_t counter;
    uint64_t latched_counter;
    uint32_t store[FLEXE_TARGET_RTC_STORE_MAX];
};

static bool rtc_offset_valid(uint16_t offset, uint32_t register_size)
{
    return (offset & 3u) == 0u &&
           offset <= register_size - sizeof(uint32_t);
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
        desc->time_low_offset == desc->time_high_offset ||
        desc->time_low_offset == desc->reset_state_offset ||
        desc->time_high_offset == desc->reset_state_offset)
        return false;

    for (unsigned i = 0u; i < desc->store_count; i++) {
        uint16_t offset = desc->store_offset[i];
        if (!rtc_offset_valid(offset, desc->register_size) ||
            offset == desc->time_update_offset ||
            offset == desc->time_low_offset ||
            offset == desc->time_high_offset ||
            offset == desc->reset_state_offset)
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

static uint64_t rtc_scaled_ticks(flexe_rtc_cntl_t *rtc,
                                 uint64_t elapsed, uint64_t cpu_hz)
{
    if (rtc->tick_denominator != cpu_hz) {
        rtc->tick_denominator = cpu_hz;
        rtc->tick_remainder = 0u;
    }
    uint64_t slow_hz = rtc->target->rtc_cntl.slow_clock_hz;
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

static void rtc_sync(flexe_rtc_cntl_t *rtc)
{
    uint64_t now = rtc_clock_now(rtc);
    uint64_t elapsed = now >= rtc->last_clock_cycles ?
                       now - rtc->last_clock_cycles : 0u;
    rtc->last_clock_cycles = now;
    if (elapsed == 0u) return;

    const flexe_rtc_cntl_desc_t *desc = &rtc->target->rtc_cntl;
    uint64_t mask = UINT32_MAX |
                    ((uint64_t)desc->time_high_mask << 32u);
    rtc->counter = (rtc->counter +
                    rtc_scaled_ticks(rtc, elapsed, rtc_cpu_hz(rtc))) & mask;
}

static uint32_t rtc_cntl_read(void *ctx, uint32_t addr)
{
    flexe_rtc_cntl_t *rtc = ctx;
    const flexe_rtc_cntl_desc_t *desc = &rtc->target->rtc_cntl;
    uint32_t offset = addr - desc->base;
    int index = rtc_store_index(desc, offset);
    if (index >= 0) return rtc->store[index];
    if (offset == desc->time_update_offset) return 0u;
    if (offset == desc->time_low_offset)
        return (uint32_t)rtc->latched_counter;
    if (offset == desc->time_high_offset)
        return (uint32_t)(rtc->latched_counter >> 32u) &
               desc->time_high_mask;
    if (offset == desc->reset_state_offset)
        return desc->reset_state_reset;
    return rtc->fallback_read ?
        rtc->fallback_read(rtc->fallback_ctx, addr) : 0u;
}

static void rtc_cntl_write(void *ctx, uint32_t addr, uint32_t value)
{
    flexe_rtc_cntl_t *rtc = ctx;
    const flexe_rtc_cntl_desc_t *desc = &rtc->target->rtc_cntl;
    uint32_t offset = addr - desc->base;
    int index = rtc_store_index(desc, offset);
    if (index >= 0) {
        rtc->store[index] = value;
        return;
    }
    if (offset == desc->time_update_offset) {
        if (value & desc->time_update_mask) {
            rtc_sync(rtc);
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
    if (rtc->fallback_write)
        rtc->fallback_write(rtc->fallback_ctx, addr, value);
}

flexe_rtc_cntl_t *flexe_rtc_cntl_create(
    xtensa_mem_t *mem, mmio_read_fn fallback_read,
    mmio_write_fn fallback_write, void *fallback_ctx)
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
    const flexe_rtc_cntl_desc_t *desc = &target->rtc_cntl;
    for (unsigned i = 0u; i < desc->store_count; i++)
        rtc->store[i] = desc->store_reset[i];

    if (mem_register_mmio_range(mem, desc->base, desc->register_size,
                                rtc_cntl_read, rtc_cntl_write, rtc) != 0) {
        free(rtc);
        return NULL;
    }
    return rtc;
}

void flexe_rtc_cntl_destroy(flexe_rtc_cntl_t *rtc)
{
    if (!rtc) return;
    const flexe_rtc_cntl_desc_t *desc = &rtc->target->rtc_cntl;
    (void)mem_register_mmio_range(
        rtc->mem, desc->base, desc->register_size,
        rtc->fallback_read, rtc->fallback_write, rtc->fallback_ctx);
    free(rtc);
}

void flexe_rtc_cntl_application_handoff(flexe_rtc_cntl_t *rtc)
{
    if (!rtc) return;
    const flexe_rtc_cntl_desc_t *desc = &rtc->target->rtc_cntl;
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
    rtc_sync(rtc);
    rtc->cpu[0] = cpu0;
    rtc->cpu[1] = cpu1;
    for (unsigned core = 0u; core < 2u; core++) {
        xtensa_cpu_t *cpu = core == 0u ? cpu0 : cpu1;
        rtc->clock.core_cycles[core] = rtc->clock.cycles;
        rtc->clock.last_ccount[core] = cpu ? cpu->ccount : 0u;
        rtc->clock.valid[core] = cpu != NULL;
    }
    rtc->last_clock_cycles = rtc->clock.cycles;
}
