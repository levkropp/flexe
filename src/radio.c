#include "radio.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

typedef struct {
    uint32_t *words;
} flexe_radio_window_t;

typedef struct {
    uint64_t cycles;
    uint32_t last_ccount;
    bool valid;
} flexe_radio_clock_t;

struct flexe_radio {
    xtensa_mem_t *mem;
    const flexe_target_desc_t *target;
    mmio_read_fn fallback_read;
    mmio_write_fn fallback_write;
    void *fallback_ctx;
    flexe_radio_window_t window[FLEXE_TARGET_RADIO_WINDOW_MAX];
    uint64_t random_state;
    xtensa_cpu_t *cpu[2];
    flexe_radio_clock_t clock[2];
};

static uint32_t *radio_word(flexe_radio_t *radio, uint32_t address);

static uint64_t radio_add_saturating(uint64_t a, uint64_t b)
{
    return a > UINT64_MAX - b ? UINT64_MAX : a + b;
}

static uint64_t radio_clock_cycles(flexe_radio_t *radio, unsigned core)
{
    const xtensa_cpu_t *cpu = radio->cpu[core];
    flexe_radio_clock_t *clock = &radio->clock[core];
    if (!cpu) return 0u;
    if (!clock->valid) {
        clock->cycles = cpu->cycle_count;
        clock->last_ccount = cpu->ccount;
        clock->valid = true;
    } else {
        /* cycle_count is published at a batch boundary. CCOUNT also moves
         * inside a batch, so a ROM MMIO read must sample its live progress. */
        uint32_t elapsed = cpu->ccount - clock->last_ccount;
        if (elapsed < (uint32_t)INT32_MAX)
            clock->cycles = radio_add_saturating(clock->cycles, elapsed);
        clock->last_ccount = cpu->ccount;
        if (cpu->cycle_count > clock->cycles)
            clock->cycles = cpu->cycle_count;
    }
    return clock->cycles;
}

static uint64_t radio_cpu_ticks(flexe_radio_t *radio, unsigned core)
{
    const xtensa_cpu_t *cpu = radio->cpu[core];
    if (!cpu) return 0u;
    const flexe_radio_time_latch_desc_t *latch =
        &radio->target->radio.time_latch;
    uint32_t mhz = xtensa_cpu_freq_mhz(cpu);
    if (!mhz) return 0u;
    uint64_t cycles = radio_clock_cycles(radio, core);
    uint64_t skipped = cpu->virtual_time_us > UINT64_MAX / mhz ?
                       UINT64_MAX : cpu->virtual_time_us * mhz;
    cycles = radio_add_saturating(cycles, skipped);
    uint64_t cpu_hz = (uint64_t)mhz * 1000000u;
    uint64_t whole = cycles / cpu_hz;
    uint64_t ticks = whole > UINT64_MAX / latch->tick_hz ?
                     UINT64_MAX : whole * latch->tick_hz;
    uint64_t fraction =
        ((cycles % cpu_hz) * latch->tick_hz) / cpu_hz;
    return radio_add_saturating(ticks, fraction);
}

static uint64_t radio_latch_ticks(flexe_radio_t *radio)
{
    uint64_t ticks0 = radio_cpu_ticks(radio, 0u);
    uint64_t ticks1 = radio_cpu_ticks(radio, 1u);
    return ticks0 > ticks1 ? ticks0 : ticks1;
}

static void radio_capture_time(flexe_radio_t *radio)
{
    const flexe_radio_time_latch_desc_t *latch =
        &radio->target->radio.time_latch;
    uint64_t ticks = radio_latch_ticks(radio);
    uint32_t *count = radio_word(radio, latch->count_address);
    uint32_t *phase = radio_word(radio, latch->phase_address);
    *count = (uint32_t)(ticks / latch->ticks_per_half_slot) &
             latch->count_mask;
    *phase = latch->ticks_per_half_slot - 1u -
             (uint32_t)(ticks % latch->ticks_per_half_slot);
}

static bool radio_address_in_window(const flexe_radio_desc_t *desc,
                                    uint32_t address)
{
    if ((address & 3u) != 0u) return false;
    for (unsigned i = 0u; i < desc->window_count; i++) {
        uint32_t base = desc->window[i].base;
        if (address >= base &&
            address - base < desc->window[i].register_size)
            return true;
    }
    return false;
}

static bool radio_geometry_valid(const flexe_target_desc_t *target)
{
    if (!target || !(target->capabilities &
                     FLEXE_TARGET_CAP_RADIO_REGS_V1))
        return false;
    const flexe_radio_desc_t *desc = &target->radio;
    if (desc->window_count == 0u ||
        desc->window_count > FLEXE_TARGET_RADIO_WINDOW_MAX ||
        desc->completion_count > FLEXE_TARGET_RADIO_COMPLETION_MAX)
        return false;

    for (unsigned i = 0u; i < desc->window_count; i++) {
        const flexe_radio_window_desc_t *window = &desc->window[i];
        if (window->register_size == 0u ||
            ((window->base | window->register_size) & 0xFFFu) != 0u ||
            window->base < target->peripheral_start ||
            window->base >= target->peripheral_end ||
            window->register_size > target->peripheral_end - window->base)
            return false;
        for (unsigned j = 0u; j < i; j++) {
            const flexe_radio_window_desc_t *other = &desc->window[j];
            if (window->base < other->base + other->register_size &&
                other->base < window->base + window->register_size)
                return false;
        }
    }

    if ((desc->random_address == 0u) != (desc->random_seed == 0u) ||
        (desc->random_address != 0u &&
         !radio_address_in_window(desc, desc->random_address)))
        return false;

    const flexe_radio_time_latch_desc_t *latch = &desc->time_latch;
    if (latch->count_address != 0u || latch->phase_address != 0u ||
        latch->capture_mask != 0u || latch->count_mask != 0u ||
        latch->tick_hz != 0u || latch->ticks_per_half_slot != 0u) {
        if (!radio_address_in_window(desc, latch->count_address) ||
            !radio_address_in_window(desc, latch->phase_address) ||
            latch->count_address == latch->phase_address ||
            latch->capture_mask == 0u ||
            (latch->capture_mask & (latch->capture_mask - 1u)) != 0u ||
            latch->count_mask == 0u ||
            (latch->count_mask & latch->capture_mask) != 0u ||
            latch->tick_hz == 0u || latch->ticks_per_half_slot == 0u)
            return false;
    }

    for (unsigned i = 0u; i < desc->completion_count; i++) {
        const flexe_radio_completion_desc_t *completion =
            &desc->completion[i];
        if (!radio_address_in_window(desc, completion->control_address) ||
            (completion->active_mask == 0u &&
             completion->self_clear_mask == 0u) ||
            (completion->active_mask & completion->self_clear_mask) != 0u)
            return false;
        if (completion->active_mask != 0u) {
            if (completion->status_mask == 0u ||
                !radio_address_in_window(desc,
                                         completion->status_address) ||
                (completion->control_address ==
                     completion->status_address &&
                 ((completion->active_mask |
                   completion->self_clear_mask) &
                  completion->status_mask) != 0u))
                return false;
        } else if (completion->status_address != 0u ||
                   completion->status_mask != 0u) {
            return false;
        }

        for (unsigned j = 0u; j < i; j++) {
            const flexe_radio_completion_desc_t *other =
                &desc->completion[j];
            if (completion->control_address == other->control_address &&
                ((completion->active_mask |
                  completion->self_clear_mask) &
                 (other->active_mask | other->self_clear_mask)) != 0u)
                return false;
            if (completion->active_mask != 0u &&
                other->active_mask != 0u &&
                completion->status_address == other->status_address &&
                (completion->status_mask & other->status_mask) != 0u)
                return false;
        }
    }
    return true;
}

static uint32_t *radio_word(flexe_radio_t *radio, uint32_t address)
{
    if ((address & 3u) != 0u) return NULL;
    const flexe_radio_desc_t *desc = &radio->target->radio;
    for (unsigned i = 0u; i < desc->window_count; i++) {
        uint32_t base = desc->window[i].base;
        if (address >= base &&
            address - base < desc->window[i].register_size)
            return &radio->window[i].words[(address - base) / 4u];
    }
    return NULL;
}

static uint32_t radio_read(void *ctx, uint32_t address)
{
    flexe_radio_t *radio = ctx;
    const flexe_radio_desc_t *desc = &radio->target->radio;
    if (address == desc->random_address) {
        /* Deterministic per-machine entropy keeps replay exact while retaining
         * the hardware contract that consecutive reads normally differ. */
        radio->random_state ^= radio->random_state << 13;
        radio->random_state ^= radio->random_state >> 7;
        radio->random_state ^= radio->random_state << 17;
        return (uint32_t)radio->random_state;
    }
    uint32_t *word = radio_word(radio, address);
    if (word) return *word;
    return radio->fallback_read ?
        radio->fallback_read(radio->fallback_ctx, address) : 0u;
}

static void radio_write(void *ctx, uint32_t address, uint32_t value)
{
    flexe_radio_t *radio = ctx;
    const flexe_radio_desc_t *desc = &radio->target->radio;
    const flexe_radio_time_latch_desc_t *latch = &desc->time_latch;
    if (latch->count_address && address == latch->count_address) {
        if (value & latch->capture_mask) radio_capture_time(radio);
        if (value != latch->capture_mask && radio->fallback_write)
            radio->fallback_write(radio->fallback_ctx, address, value);
        return;
    }
    if (latch->phase_address && address == latch->phase_address) {
        if (radio->fallback_write)
            radio->fallback_write(radio->fallback_ctx, address, value);
        return;
    }
    uint32_t *word = radio_word(radio, address);
    if (!word) {
        if (radio->fallback_write)
            radio->fallback_write(radio->fallback_ctx, address, value);
        return;
    }
    if (address == desc->random_address)
        return; /* Hardware entropy source is read-only. */

    uint32_t read_only = 0u;
    for (unsigned i = 0u; i < desc->completion_count; i++) {
        const flexe_radio_completion_desc_t *completion =
            &desc->completion[i];
        if (completion->active_mask != 0u &&
            completion->status_address == address)
            read_only |= completion->status_mask;
    }
    *word = (value & ~read_only) | (*word & read_only);

    for (unsigned i = 0u; i < desc->completion_count; i++) {
        const flexe_radio_completion_desc_t *completion =
            &desc->completion[i];
        if (completion->control_address != address) continue;
        *word &= ~completion->self_clear_mask;
        if (completion->active_mask == 0u) continue;

        uint32_t *status = radio_word(radio, completion->status_address);
        if ((*word & completion->active_mask) == completion->active_mask)
            *status |= completion->status_mask;
        else
            *status &= ~completion->status_mask;
        /* Fast mode resolves calibration at the control-write boundary. A
         * timed implementation can schedule this same status transition
         * without changing the target descriptor or register contract. */
    }
}

flexe_radio_t *flexe_radio_create(
    xtensa_mem_t *mem, mmio_read_fn fallback_read,
    mmio_write_fn fallback_write, void *fallback_ctx)
{
    if (!mem) return NULL;
    const flexe_target_desc_t *target = mem_target(mem);
    if (!radio_geometry_valid(target)) return NULL;

    flexe_radio_t *radio = calloc(1u, sizeof(*radio));
    if (!radio) return NULL;
    radio->mem = mem;
    radio->target = target;
    radio->fallback_read = fallback_read;
    radio->fallback_write = fallback_write;
    radio->fallback_ctx = fallback_ctx;
    radio->random_state = target->radio.random_seed;

    unsigned registered = 0u;
    for (unsigned i = 0u; i < target->radio.window_count; i++) {
        const flexe_radio_window_desc_t *window = &target->radio.window[i];
        radio->window[i].words =
            calloc(window->register_size / sizeof(uint32_t),
                   sizeof(uint32_t));
        if (!radio->window[i].words ||
            mem_register_mmio_range(mem, window->base,
                                    window->register_size,
                                    radio_read, radio_write, radio) != 0)
            goto fail;
        registered++;
    }
    return radio;

fail:
    for (unsigned i = 0u; i < registered; i++) {
        const flexe_radio_window_desc_t *window = &target->radio.window[i];
        (void)mem_register_mmio_range(
            mem, window->base, window->register_size,
            fallback_read, fallback_write, fallback_ctx);
    }
    for (unsigned i = 0u; i < target->radio.window_count; i++)
        free(radio->window[i].words);
    free(radio);
    return NULL;
}

void flexe_radio_attach_cpus(flexe_radio_t *radio,
                              xtensa_cpu_t *cpu0, xtensa_cpu_t *cpu1)
{
    if (!radio) return;
    radio->cpu[0] = cpu0;
    radio->cpu[1] = cpu1;
    for (unsigned core = 0u; core < 2u; core++) {
        xtensa_cpu_t *cpu = radio->cpu[core];
        radio->clock[core].cycles = cpu ? cpu->cycle_count : 0u;
        radio->clock[core].last_ccount = cpu ? cpu->ccount : 0u;
        radio->clock[core].valid = cpu != NULL;
    }
}

void flexe_radio_destroy(flexe_radio_t *radio)
{
    if (!radio) return;
    const flexe_radio_desc_t *desc = &radio->target->radio;
    for (unsigned i = 0u; i < desc->window_count; i++) {
        const flexe_radio_window_desc_t *window = &desc->window[i];
        (void)mem_register_mmio_range(
            radio->mem, window->base, window->register_size,
            radio->fallback_read, radio->fallback_write,
            radio->fallback_ctx);
        free(radio->window[i].words);
    }
    free(radio);
}
