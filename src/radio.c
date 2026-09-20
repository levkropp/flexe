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
    uint32_t bb_config;
    uint32_t bb_config2;
    uint32_t clock_enable;
    uint32_t reset_enable;
    uint64_t random_state;
    uint32_t rtc_domains_powered;
    uint32_t rtc_domains_isolated;
    xtensa_cpu_t *cpu[2];
    flexe_radio_clock_t clock[2];
};

static uint32_t *radio_word(flexe_radio_t *radio, uint32_t address);

static bool radio_clock_enabled(const flexe_radio_t *radio, uint32_t mask)
{
    return mask == 0u || (radio->clock_enable & mask) == mask;
}

static uint32_t radio_valid_rtc_domains(
    const flexe_target_desc_t *target)
{
    unsigned count = target->rtc_cntl.digital_domain_count;
    if (count == 0u) return 0u;
    return count >= 32u ? UINT32_MAX : (1u << count) - 1u;
}

static bool radio_control_offset_valid(uint16_t offset, uint32_t size)
{
    return (offset & 3u) == 0u && size >= sizeof(uint32_t) &&
           offset <= size - sizeof(uint32_t);
}

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
    if (target->rtc_cntl.digital_domain_count >
        FLEXE_TARGET_RTC_DIGITAL_DOMAIN_MAX)
        return false;
    const flexe_radio_desc_t *desc = &target->radio;
    const flexe_radio_control_desc_t *control = &desc->control;
    if (desc->window_count == 0u ||
        desc->window_count > FLEXE_TARGET_RADIO_WINDOW_MAX ||
        desc->completion_count > FLEXE_TARGET_RADIO_COMPLETION_MAX ||
        desc->register_count > FLEXE_TARGET_RADIO_REGISTER_MAX)
        return false;

    bool has_control = control->register_size != 0u;
    if (!has_control) {
        if (control->base != 0u || control->bb_config_offset != 0u ||
            control->bb_config2_offset != 0u ||
            control->clock_offset != 0u || control->reset_offset != 0u ||
            control->bb_config_reset != 0u ||
            control->bb_config_writable_mask != 0u ||
            control->bb_config2_reset != 0u ||
            control->bb_config2_writable_mask != 0u ||
            control->clock_reset != 0u ||
            control->clock_writable_mask != 0u ||
            control->reset_reset != 0u ||
            control->reset_writable_mask != 0u)
            return false;
    } else {
        const uint16_t offsets[] = {
            control->bb_config_offset,
            control->bb_config2_offset,
            control->clock_offset,
            control->reset_offset,
        };
        if (((control->base | control->register_size) & 0xFFFu) != 0u ||
            control->base < target->peripheral_start ||
            control->base >= target->peripheral_end ||
            control->register_size > target->peripheral_end - control->base ||
            control->bb_config_writable_mask == 0u ||
            control->bb_config2_writable_mask == 0u ||
            control->clock_writable_mask == 0u ||
            control->reset_writable_mask == 0u ||
            (control->bb_config_reset &
             ~control->bb_config_writable_mask) != 0u ||
            (control->bb_config2_reset &
             ~control->bb_config2_writable_mask) != 0u ||
            (control->clock_reset & ~control->clock_writable_mask) != 0u ||
            (control->reset_reset & ~control->reset_writable_mask) != 0u)
            return false;
        for (unsigned i = 0u;
             i < sizeof(offsets) / sizeof(offsets[0]); i++) {
            if (!radio_control_offset_valid(offsets[i],
                                            control->register_size))
                return false;
            for (unsigned j = 0u; j < i; j++)
                if (offsets[i] == offsets[j]) return false;
        }
    }

    uint32_t mapped_reset_mask = 0u;
    for (unsigned i = 0u; i < desc->window_count; i++) {
        const flexe_radio_window_desc_t *window = &desc->window[i];
        if (window->register_size == 0u ||
            ((window->base | window->register_size) & 0xFFFu) != 0u ||
            window->base < target->peripheral_start ||
            window->base >= target->peripheral_end ||
            window->register_size > target->peripheral_end - window->base ||
            (!has_control && window->reset_mask != 0u) ||
            (has_control &&
             (window->reset_mask & ~control->reset_writable_mask) != 0u) ||
            (window->rtc_power_domain != 0u &&
             (!(target->capabilities & FLEXE_TARGET_CAP_RTC_CNTL_V1) ||
              window->rtc_power_domain >
                  FLEXE_TARGET_RTC_DIGITAL_DOMAIN_MAX ||
              window->rtc_power_domain >
                  target->rtc_cntl.digital_domain_count)))
            return false;
        mapped_reset_mask |= window->reset_mask;
        for (unsigned j = 0u; j < i; j++) {
            const flexe_radio_window_desc_t *other = &desc->window[j];
            if (window->base < other->base + other->register_size &&
                other->base < window->base + window->register_size)
                return false;
        }
        if (has_control &&
            window->base < control->base + control->register_size &&
            control->base < window->base + window->register_size)
            return false;
    }
    if (has_control && mapped_reset_mask == 0u) return false;

    if ((desc->random_address == 0u) != (desc->random_seed == 0u) ||
        (desc->random_address != 0u &&
         !radio_address_in_window(desc, desc->random_address)) ||
        (!has_control && desc->random_clock_mask != 0u) ||
        (has_control &&
         (desc->random_clock_mask & ~control->clock_writable_mask) != 0u))
        return false;

    const flexe_radio_time_latch_desc_t *latch = &desc->time_latch;
    if (latch->count_address != 0u || latch->phase_address != 0u ||
        latch->capture_mask != 0u || latch->count_mask != 0u ||
        latch->clock_mask != 0u ||
        latch->tick_hz != 0u || latch->ticks_per_half_slot != 0u) {
        if (!radio_address_in_window(desc, latch->count_address) ||
            !radio_address_in_window(desc, latch->phase_address) ||
            latch->count_address == latch->phase_address ||
            latch->capture_mask == 0u ||
            (latch->capture_mask & (latch->capture_mask - 1u)) != 0u ||
            latch->count_mask == 0u ||
            (latch->count_mask & latch->capture_mask) != 0u ||
            (!has_control && latch->clock_mask != 0u) ||
            (has_control &&
             (latch->clock_mask & ~control->clock_writable_mask) != 0u) ||
            latch->tick_hz == 0u || latch->ticks_per_half_slot == 0u)
            return false;
    }

    for (unsigned i = 0u; i < desc->completion_count; i++) {
        const flexe_radio_completion_desc_t *completion =
            &desc->completion[i];
        if (!radio_address_in_window(desc, completion->control_address) ||
            (completion->active_mask == 0u &&
             completion->self_clear_mask == 0u) ||
            (completion->active_mask & completion->self_clear_mask) != 0u ||
            (!has_control && completion->clock_mask != 0u) ||
            (has_control &&
             (completion->clock_mask &
              ~control->clock_writable_mask) != 0u))
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

    for (unsigned i = 0u; i < desc->register_count; i++) {
        const flexe_radio_register_desc_t *reg = &desc->reg[i];
        if (!radio_address_in_window(desc, reg->address)) return false;
        for (unsigned j = 0u; j < i; j++)
            if (reg->address == desc->reg[j].address) return false;
    }
    return true;
}

static int radio_window_index(const flexe_radio_desc_t *desc,
                              uint32_t address)
{
    if ((address & 3u) != 0u) return -1;
    for (unsigned i = 0u; i < desc->window_count; i++) {
        uint32_t base = desc->window[i].base;
        if (address >= base &&
            address - base < desc->window[i].register_size)
            return (int)i;
    }
    return -1;
}

static uint32_t *radio_word(flexe_radio_t *radio, uint32_t address)
{
    const flexe_radio_desc_t *desc = &radio->target->radio;
    int index = radio_window_index(desc, address);
    if (index < 0) return NULL;
    uint32_t base = desc->window[index].base;
    return &radio->window[index].words[(address - base) / 4u];
}

static bool radio_window_in_reset(const flexe_radio_t *radio,
                                  unsigned index)
{
    uint32_t mask = radio->target->radio.window[index].reset_mask;
    return mask != 0u && (radio->reset_enable & mask) != 0u;
}

static uint32_t radio_window_domain_bit(const flexe_radio_t *radio,
                                        unsigned index)
{
    unsigned domain = radio->target->radio.window[index].rtc_power_domain;
    return domain == 0u ? 0u : 1u << (domain - 1u);
}

static bool radio_window_available(const flexe_radio_t *radio,
                                   unsigned index)
{
    uint32_t domain = radio_window_domain_bit(radio, index);
    return domain == 0u ||
        ((radio->rtc_domains_powered & domain) != 0u &&
         (radio->rtc_domains_isolated & domain) == 0u);
}

static void radio_reset_window(flexe_radio_t *radio, unsigned index)
{
    const flexe_radio_desc_t *desc = &radio->target->radio;
    const flexe_radio_window_desc_t *window = &desc->window[index];
    for (uint32_t word = 0u;
         word < window->register_size / sizeof(uint32_t); word++)
        radio->window[index].words[word] = 0u;

    for (unsigned i = 0u; i < desc->register_count; i++) {
        const flexe_radio_register_desc_t *reg = &desc->reg[i];
        if (reg->address >= window->base &&
            reg->address - window->base < window->register_size)
            *radio_word(radio, reg->address) = reg->reset;
    }
    if (desc->random_address >= window->base &&
        desc->random_address - window->base < window->register_size)
        radio->random_state = desc->random_seed;
}

static bool radio_control_read(const flexe_radio_t *radio,
                               uint32_t address, uint32_t *value)
{
    const flexe_radio_control_desc_t *control =
        &radio->target->radio.control;
    if (control->register_size == 0u || address < control->base ||
        address - control->base >= control->register_size)
        return false;
    uint32_t offset = address - control->base;
    if (offset == control->bb_config_offset) *value = radio->bb_config;
    else if (offset == control->bb_config2_offset)
        *value = radio->bb_config2;
    else if (offset == control->clock_offset)
        *value = radio->clock_enable;
    else if (offset == control->reset_offset)
        *value = radio->reset_enable;
    else return false;
    return true;
}

static bool radio_control_write(flexe_radio_t *radio,
                                uint32_t address, uint32_t value)
{
    const flexe_radio_desc_t *desc = &radio->target->radio;
    const flexe_radio_control_desc_t *control = &desc->control;
    if (control->register_size == 0u || address < control->base ||
        address - control->base >= control->register_size)
        return false;
    uint32_t offset = address - control->base;
    if (offset == control->bb_config_offset) {
        radio->bb_config =
            (radio->bb_config & ~control->bb_config_writable_mask) |
            (value & control->bb_config_writable_mask);
    } else if (offset == control->bb_config2_offset) {
        radio->bb_config2 =
            (radio->bb_config2 & ~control->bb_config2_writable_mask) |
            (value & control->bb_config2_writable_mask);
    } else if (offset == control->clock_offset) {
        radio->clock_enable =
            (radio->clock_enable & ~control->clock_writable_mask) |
            (value & control->clock_writable_mask);
    } else if (offset == control->reset_offset) {
        radio->reset_enable =
            (radio->reset_enable & ~control->reset_writable_mask) |
            (value & control->reset_writable_mask);
        for (unsigned i = 0u; i < desc->window_count; i++)
            if (radio_window_in_reset(radio, i))
                radio_reset_window(radio, i);
    } else {
        return false;
    }
    return true;
}

static uint32_t radio_read(void *ctx, uint32_t address)
{
    flexe_radio_t *radio = ctx;
    const flexe_radio_desc_t *desc = &radio->target->radio;
    uint32_t control_value = 0u;
    if (radio_control_read(radio, address, &control_value))
        return control_value;
    if (address == desc->random_address) {
        /* The target's RNG is an independent peripheral endpoint even when
         * its address lies inside a WDEV aperture. RF power/isolation does
         * not remove it; its described clock controls sample progression. */
        if (radio_clock_enabled(radio, desc->random_clock_mask)) {
            radio->random_state ^= radio->random_state << 13;
            radio->random_state ^= radio->random_state >> 7;
            radio->random_state ^= radio->random_state << 17;
        }
        return (uint32_t)radio->random_state;
    }
    int window = radio_window_index(desc, address);
    uint32_t *word = radio_word(radio, address);
    if (word && window >= 0 &&
        !radio_window_available(radio, (unsigned)window))
        return 0u;
    if (word && window >= 0 && radio_window_in_reset(
            radio, (unsigned)window))
        return *word;
    if (word) return *word;
    return radio->fallback_read ?
        radio->fallback_read(radio->fallback_ctx, address) : 0u;
}

static void radio_write(void *ctx, uint32_t address, uint32_t value)
{
    flexe_radio_t *radio = ctx;
    const flexe_radio_desc_t *desc = &radio->target->radio;
    if (radio_control_write(radio, address, value)) return;

    int window = radio_window_index(desc, address);
    uint32_t *word = radio_word(radio, address);
    if (!word) {
        if (radio->fallback_write)
            radio->fallback_write(radio->fallback_ctx, address, value);
        return;
    }
    if (window >= 0 &&
        !radio_window_available(radio, (unsigned)window))
        return;
    if (window >= 0 && radio_window_in_reset(radio, (unsigned)window))
        return;

    const flexe_radio_time_latch_desc_t *latch = &desc->time_latch;
    if (latch->count_address && address == latch->count_address) {
        if ((value & latch->capture_mask) &&
            radio_clock_enabled(radio, latch->clock_mask))
            radio_capture_time(radio);
        if (value != latch->capture_mask && radio->fallback_write)
            radio->fallback_write(radio->fallback_ctx, address, value);
        return;
    }
    if (latch->phase_address && address == latch->phase_address) {
        if (radio->fallback_write)
            radio->fallback_write(radio->fallback_ctx, address, value);
        return;
    }
    if (address == desc->random_address)
        return; /* Hardware entropy source is read-only. */

    uint32_t writable = UINT32_MAX;
    for (unsigned i = 0u; i < desc->register_count; i++) {
        const flexe_radio_register_desc_t *reg = &desc->reg[i];
        if (reg->address == address) {
            writable &= reg->writable_mask;
            break;
        }
    }
    for (unsigned i = 0u; i < desc->completion_count; i++) {
        const flexe_radio_completion_desc_t *completion =
            &desc->completion[i];
        if (completion->active_mask != 0u &&
            completion->status_address == address)
            writable &= ~completion->status_mask;
    }
    *word = (value & writable) | (*word & ~writable);

    for (unsigned i = 0u; i < desc->completion_count; i++) {
        const flexe_radio_completion_desc_t *completion =
            &desc->completion[i];
        if (completion->control_address != address) continue;
        if (!radio_clock_enabled(radio, completion->clock_mask)) continue;
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
    radio->rtc_domains_powered = radio_valid_rtc_domains(target);
    const flexe_radio_control_desc_t *control = &target->radio.control;
    radio->bb_config = control->bb_config_reset;
    radio->bb_config2 = control->bb_config2_reset;
    radio->clock_enable = control->clock_reset;
    radio->reset_enable = control->reset_reset;

    unsigned registered = 0u;
    bool control_registered = false;
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
    if (control->register_size != 0u) {
        if (mem_register_mmio_range(mem, control->base,
                                    control->register_size,
                                    radio_read, radio_write, radio) != 0)
            goto fail;
        control_registered = true;
    }
    for (unsigned i = 0u; i < target->radio.register_count; i++) {
        const flexe_radio_register_desc_t *reg = &target->radio.reg[i];
        *radio_word(radio, reg->address) = reg->reset;
    }
    return radio;

fail:
    if (control_registered)
        (void)mem_register_mmio_range(
            mem, control->base, control->register_size,
            fallback_read, fallback_write, fallback_ctx);
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

void flexe_radio_set_rtc_domain_state(flexe_radio_t *radio,
                                      uint32_t powered,
                                      uint32_t isolated)
{
    if (!radio) return;
    uint32_t valid = radio_valid_rtc_domains(radio->target);
    powered &= valid;
    isolated &= valid;
    uint32_t lost_power = radio->rtc_domains_powered & ~powered;
    radio->rtc_domains_powered = powered;
    radio->rtc_domains_isolated = isolated;
    if (lost_power == 0u) return;
    for (unsigned i = 0u; i < radio->target->radio.window_count; i++)
        if ((radio_window_domain_bit(radio, i) & lost_power) != 0u)
            radio_reset_window(radio, i);
}

void flexe_radio_destroy(flexe_radio_t *radio)
{
    if (!radio) return;
    const flexe_radio_desc_t *desc = &radio->target->radio;
    if (desc->control.register_size != 0u)
        (void)mem_register_mmio_range(
            radio->mem, desc->control.base,
            desc->control.register_size,
            radio->fallback_read, radio->fallback_write,
            radio->fallback_ctx);
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
