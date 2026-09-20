#include "system_clock.h"
#include "target.h"

#include <stdbool.h>
#include <stdlib.h>

struct flexe_system_clock {
    xtensa_mem_t *mem;
    const flexe_target_desc_t *target;
    mmio_read_fn fallback_read;
    mmio_write_fn fallback_write;
    void *fallback_ctx;
    flexe_system_clock_gate_fn gate_changed;
    void *gate_ctx;
    flexe_system_low_power_fn low_power_changed;
    void *low_power_ctx;
    flexe_system_peripheral_fn peripheral_changed;
    void *peripheral_ctx;
    uint32_t cpu_per_conf;
    uint32_t sysclk_conf;
    uint32_t reg[FLEXE_TARGET_SYSTEM_REGISTER_MAX];
};

static bool system_clock_register_valid(uint32_t offset,
                                        uint32_t register_size)
{
    return (offset & 3u) == 0u && register_size >= sizeof(uint32_t) &&
           offset <= register_size - sizeof(uint32_t);
}

static int system_clock_register_index(
    const flexe_system_clock_desc_t *desc, uint32_t offset)
{
    for (unsigned index = 0u; index < desc->register_count; index++)
        if (desc->reg[index].offset == offset) return (int)index;
    return -1;
}

static bool system_clock_single_bit(uint32_t mask)
{
    return mask != 0u && (mask & (mask - 1u)) == 0u;
}

static bool system_clock_contiguous_mask(uint32_t mask)
{
    if (mask == 0u) return false;
    while ((mask & 1u) == 0u) mask >>= 1u;
    return (mask & (mask + 1u)) == 0u;
}

static unsigned system_clock_mask_shift(uint32_t mask)
{
    unsigned shift = 0u;
    while ((mask & 1u) == 0u) {
        mask >>= 1u;
        shift++;
    }
    return shift;
}

static int system_clock_peripheral_bank_index(
    const flexe_system_clock_desc_t *desc,
    uint32_t clock_offset, uint32_t reset_offset)
{
    const flexe_system_peripheral_banks_desc_t *banks =
        &desc->peripheral_banks;
    for (unsigned bank = 0u; bank < banks->bank_count; bank++)
        if (banks->clock_offset[bank] == clock_offset &&
            banks->reset_offset[bank] == reset_offset)
            return (int)bank;
    return -1;
}

static bool system_clock_peripheral_offset(
    const flexe_system_clock_desc_t *desc, uint32_t offset)
{
    const flexe_system_peripheral_banks_desc_t *banks =
        &desc->peripheral_banks;
    for (unsigned bank = 0u; bank < banks->bank_count; bank++)
        if (banks->clock_offset[bank] == offset ||
            banks->reset_offset[bank] == offset)
            return true;
    return false;
}

static bool system_clock_geometry_valid(const flexe_target_desc_t *target)
{
    if (!target || !(target->capabilities &
                     FLEXE_TARGET_CAP_SYSTEM_CLOCK_V1))
        return false;

    const flexe_system_clock_desc_t *desc = &target->system_clock;
    if (((desc->base | desc->register_size) & 0xFFFu) != 0u ||
        desc->register_size == 0u ||
        desc->base < target->peripheral_start ||
        desc->base >= target->peripheral_end ||
        desc->register_size > target->peripheral_end - desc->base ||
        !system_clock_register_valid(desc->cpu_per_conf_offset,
                                     desc->register_size) ||
        !system_clock_register_valid(desc->sysclk_conf_offset,
                                     desc->register_size) ||
        desc->cpu_per_conf_offset == desc->sysclk_conf_offset ||
        desc->cpu_per_conf_writable_mask == 0u ||
        desc->sysclk_conf_writable_mask == 0u ||
        desc->register_count > FLEXE_TARGET_SYSTEM_REGISTER_MAX ||
        desc->gate_count > FLEXE_TARGET_SYSTEM_GATE_MAX ||
        desc->peripheral_banks.bank_count == 0u ||
        desc->peripheral_banks.bank_count >
            FLEXE_TARGET_SYSTEM_PERIPHERAL_BANK_MAX)
        return false;

    for (unsigned index = 0u; index < desc->register_count; index++) {
        const flexe_system_register_desc_t *reg = &desc->reg[index];
        if (!system_clock_register_valid(reg->offset,
                                         desc->register_size) ||
            reg->offset == desc->cpu_per_conf_offset ||
            reg->offset == desc->sysclk_conf_offset ||
            reg->writable_mask == 0u ||
            (reg->reset & ~reg->writable_mask) != 0u)
            return false;
        for (unsigned old = 0u; old < index; old++)
            if (reg->offset == desc->reg[old].offset) return false;
    }

    const flexe_system_peripheral_banks_desc_t *banks =
        &desc->peripheral_banks;
    for (unsigned bank = 0u; bank < banks->bank_count; bank++) {
        int clock_reg = system_clock_register_index(
            desc, banks->clock_offset[bank]);
        int reset_reg = system_clock_register_index(
            desc, banks->reset_offset[bank]);
        if (clock_reg < 0 || reset_reg < 0 ||
            banks->clock_offset[bank] == banks->reset_offset[bank] ||
            banks->valid_mask[bank] == 0u ||
            (banks->valid_mask[bank] &
             ~desc->reg[clock_reg].writable_mask) != 0u ||
            (banks->valid_mask[bank] &
             ~desc->reg[reset_reg].writable_mask) != 0u)
            return false;
        for (unsigned old = 0u; old < bank; old++)
            if (banks->clock_offset[bank] == banks->clock_offset[old] ||
                banks->clock_offset[bank] == banks->reset_offset[old] ||
                banks->reset_offset[bank] == banks->clock_offset[old] ||
                banks->reset_offset[bank] == banks->reset_offset[old])
                return false;
    }

    for (unsigned index = 0u; index < desc->gate_count; index++) {
        const flexe_system_gate_desc_t *gate = &desc->gate[index];
        int clock_reg = system_clock_register_index(
            desc, gate->clock_offset);
        int reset_reg = system_clock_register_index(
            desc, gate->reset_offset);
        bool device_valid =
            (gate->device == FLEXE_SYSTEM_DEVICE_SYSTIMER &&
             gate->instance == 0u &&
             (target->capabilities & FLEXE_TARGET_CAP_SYSTIMER_V1)) ||
            (gate->device == FLEXE_SYSTEM_DEVICE_TIMER_GROUP &&
             gate->instance < FLEXE_TARGET_TIMER_GROUP_MAX &&
             gate->instance < target->timer_group.group_count &&
             (target->capabilities & FLEXE_TARGET_CAP_TIMER_GROUP_V1)) ||
            (gate->device == FLEXE_SYSTEM_DEVICE_I2C &&
             gate->instance < FLEXE_TARGET_I2C_MAX &&
             gate->instance < target->i2c.instance_count &&
             (target->capabilities & FLEXE_TARGET_CAP_I2C_V1)) ||
            (gate->device == FLEXE_SYSTEM_DEVICE_GP_SPI &&
             gate->instance < FLEXE_TARGET_GP_SPI_HOST_MAX &&
             gate->instance < target->gp_spi.host_count &&
             (target->capabilities & FLEXE_TARGET_CAP_GP_SPI)) ||
            (gate->device == FLEXE_SYSTEM_DEVICE_SHA &&
             gate->instance == 0u &&
             (target->capabilities & FLEXE_TARGET_CAP_SHA_V1)) ||
            (gate->device == FLEXE_SYSTEM_DEVICE_AES &&
             gate->instance == 0u &&
             (target->capabilities & FLEXE_TARGET_CAP_AES_V1)) ||
            (gate->device == FLEXE_SYSTEM_DEVICE_LEDC &&
             gate->instance == 0u &&
             (target->capabilities & FLEXE_TARGET_CAP_LEDC_V1)) ||
            (gate->device == FLEXE_SYSTEM_DEVICE_UART &&
             gate->instance < target->uart_count) ||
            (gate->device == FLEXE_SYSTEM_DEVICE_USB_SERIAL_JTAG &&
             gate->instance == 0u &&
             (target->capabilities &
              FLEXE_TARGET_CAP_USB_SERIAL_JTAG_V1)) ||
            (gate->device == FLEXE_SYSTEM_DEVICE_I2S &&
             gate->instance < FLEXE_TARGET_I2S_MAX &&
             gate->instance < target->i2s_v2.instance_count &&
             (target->capabilities & FLEXE_TARGET_CAP_I2S_V2)) ||
            (gate->device == FLEXE_SYSTEM_DEVICE_SDMMC &&
             gate->instance == 0u &&
             (target->capabilities & FLEXE_TARGET_CAP_SDMMC_HOST_V1)) ||
            (gate->device == FLEXE_SYSTEM_DEVICE_TWAI &&
             gate->instance == 0u &&
             (target->capabilities & FLEXE_TARGET_CAP_TWAI_V1)) ||
            (gate->device == FLEXE_SYSTEM_DEVICE_PCNT &&
             gate->instance == 0u &&
             (target->capabilities & FLEXE_TARGET_CAP_PCNT_V1)) ||
            (gate->device == FLEXE_SYSTEM_DEVICE_MCPWM &&
             gate->instance < FLEXE_TARGET_MCPWM_GROUP_MAX &&
             gate->instance < target->mcpwm.group_count &&
             (target->capabilities & FLEXE_TARGET_CAP_MCPWM_V1)) ||
            (gate->device == FLEXE_SYSTEM_DEVICE_EDMA &&
             gate->instance == 0u);
        int bank = system_clock_peripheral_bank_index(
            desc, gate->clock_offset, gate->reset_offset);
        bool references_bank = system_clock_peripheral_offset(
            desc, gate->clock_offset) ||
            system_clock_peripheral_offset(desc, gate->reset_offset);
        if (!device_valid || clock_reg < 0 || reset_reg < 0 ||
            !system_clock_single_bit(gate->clock_mask) ||
            !system_clock_single_bit(gate->reset_mask) ||
            (gate->clock_mask &
             ~desc->reg[clock_reg].writable_mask) != 0u ||
            (gate->reset_mask &
             ~desc->reg[reset_reg].writable_mask) != 0u ||
            (references_bank && bank < 0) ||
            (bank >= 0 &&
             ((gate->clock_mask & ~banks->valid_mask[bank]) != 0u ||
              (gate->reset_mask & ~banks->valid_mask[bank]) != 0u)))
            return false;
        for (unsigned old = 0u; old < index; old++)
            if (gate->device == desc->gate[old].device &&
                gate->instance == desc->gate[old].instance)
                return false;
    }

    const flexe_system_low_power_desc_t *low = &desc->low_power;
    uint32_t fraction_mask = low->divider_a_mask |
                             low->divider_b_mask |
                             low->source_rtc_slow_mask |
                             low->source_internal_mask |
                             low->source_xtal_mask |
                             low->source_xtal32k_mask |
                             low->rtc_clock_enable_mask;
    const uint32_t fraction_fields[] = {
        low->divider_a_mask,
        low->divider_b_mask,
        low->source_rtc_slow_mask,
        low->source_internal_mask,
        low->source_xtal_mask,
        low->source_xtal32k_mask,
        low->rtc_clock_enable_mask,
    };
    uint32_t occupied = 0u;
    for (unsigned index = 0u;
         index < sizeof(fraction_fields) / sizeof(fraction_fields[0]);
         index++) {
        if ((occupied & fraction_fields[index]) != 0u) return false;
        occupied |= fraction_fields[index];
    }
    int memory_reg = system_clock_register_index(
        desc, low->memory_power_down_offset);
    int integer_reg = system_clock_register_index(
        desc, low->divider_integer_offset);
    int fraction_reg = system_clock_register_index(
        desc, low->divider_fraction_offset);
    if (memory_reg < 0 || integer_reg < 0 || fraction_reg < 0 ||
        !system_clock_single_bit(low->memory_power_down_inhibit_mask) ||
        !system_clock_contiguous_mask(low->divider_integer_mask) ||
        !system_clock_contiguous_mask(low->divider_a_mask) ||
        !system_clock_contiguous_mask(low->divider_b_mask) ||
        !system_clock_single_bit(low->source_rtc_slow_mask) ||
        !system_clock_single_bit(low->source_internal_mask) ||
        !system_clock_single_bit(low->source_xtal_mask) ||
        !system_clock_single_bit(low->source_xtal32k_mask) ||
        !system_clock_single_bit(low->rtc_clock_enable_mask) ||
        (low->memory_power_down_inhibit_mask &
         ~desc->reg[memory_reg].writable_mask) != 0u ||
        (low->divider_integer_mask &
         ~desc->reg[integer_reg].writable_mask) != 0u ||
        (fraction_mask & ~desc->reg[fraction_reg].writable_mask) != 0u)
        return false;
    return true;
}

static uint32_t system_clock_mapped_mask(
    const flexe_system_clock_desc_t *desc, uint32_t offset)
{
    uint32_t mask = 0u;
    for (unsigned index = 0u; index < desc->gate_count; index++) {
        const flexe_system_gate_desc_t *gate = &desc->gate[index];
        if (gate->clock_offset == offset) mask |= gate->clock_mask;
        if (gate->reset_offset == offset) mask |= gate->reset_mask;
    }
    const flexe_system_low_power_desc_t *low = &desc->low_power;
    if (offset == low->memory_power_down_offset)
        mask |= low->memory_power_down_inhibit_mask;
    if (offset == low->divider_integer_offset)
        mask |= low->divider_integer_mask;
    if (offset == low->divider_fraction_offset)
        mask |= low->divider_a_mask | low->divider_b_mask |
                low->source_rtc_slow_mask | low->source_internal_mask |
                low->source_xtal_mask | low->source_xtal32k_mask |
                low->rtc_clock_enable_mask;
    const flexe_system_peripheral_banks_desc_t *banks =
        &desc->peripheral_banks;
    for (unsigned bank = 0u; bank < banks->bank_count; bank++) {
        if (offset == banks->clock_offset[bank] ||
            offset == banks->reset_offset[bank])
            mask |= banks->valid_mask[bank];
    }
    return mask;
}

static bool system_clock_low_power_offset(
    const flexe_system_low_power_desc_t *low, uint32_t offset)
{
    return offset == low->memory_power_down_offset ||
           offset == low->divider_integer_offset ||
           offset == low->divider_fraction_offset;
}

static void system_clock_publish_low_power(flexe_system_clock_t *clock)
{
    if (!clock->low_power_changed) return;
    flexe_system_low_power_state_t state;
    if (flexe_system_clock_low_power_state(clock, &state))
        clock->low_power_changed(clock->low_power_ctx, &state);
}

static void system_clock_publish_peripheral(flexe_system_clock_t *clock)
{
    if (!clock->peripheral_changed) return;
    flexe_system_peripheral_state_t state;
    if (flexe_system_clock_peripheral_state(clock, &state))
        clock->peripheral_changed(clock->peripheral_ctx, &state);
}

static void system_clock_publish_gate(flexe_system_clock_t *clock,
                                      unsigned index)
{
    if (!clock->gate_changed) return;
    const flexe_system_clock_desc_t *desc = &clock->target->system_clock;
    const flexe_system_gate_desc_t *gate = &desc->gate[index];
    int clock_reg = system_clock_register_index(
        desc, gate->clock_offset);
    int reset_reg = system_clock_register_index(
        desc, gate->reset_offset);
    if (clock_reg < 0 || reset_reg < 0) return;
    clock->gate_changed(
        clock->gate_ctx, gate->device, gate->instance,
        (clock->reg[clock_reg] & gate->clock_mask) != 0u,
        (clock->reg[reset_reg] & gate->reset_mask) != 0u);
}

static uint32_t system_clock_read(void *ctx, uint32_t addr)
{
    flexe_system_clock_t *clock = ctx;
    const flexe_system_clock_desc_t *desc =
        &clock->target->system_clock;
    uint32_t offset = addr - desc->base;

    if (offset == desc->cpu_per_conf_offset)
        return clock->cpu_per_conf;
    if (offset == desc->sysclk_conf_offset)
        return clock->sysclk_conf;
    int index = system_clock_register_index(desc, offset);
    if (index >= 0) return clock->reg[index];
    return clock->fallback_read ?
        clock->fallback_read(clock->fallback_ctx, addr) : 0u;
}

static void system_clock_write(void *ctx, uint32_t addr, uint32_t value)
{
    flexe_system_clock_t *clock = ctx;
    const flexe_system_clock_desc_t *desc =
        &clock->target->system_clock;
    uint32_t offset = addr - desc->base;

    if (offset == desc->cpu_per_conf_offset) {
        clock->cpu_per_conf =
            (clock->cpu_per_conf & ~desc->cpu_per_conf_writable_mask) |
            (value & desc->cpu_per_conf_writable_mask);
        return;
    }
    if (offset == desc->sysclk_conf_offset) {
        clock->sysclk_conf =
            (clock->sysclk_conf & ~desc->sysclk_conf_writable_mask) |
            (value & desc->sysclk_conf_writable_mask);
        return;
    }
    int index = system_clock_register_index(desc, offset);
    if (index >= 0) {
        const flexe_system_register_desc_t *reg = &desc->reg[index];
        uint32_t old = clock->reg[index];
        uint32_t next = (old & ~reg->writable_mask) |
                        (value & reg->writable_mask);
        if (old == next) return;
        clock->reg[index] = next;

        uint32_t changed = old ^ next;
        if ((changed & ~system_clock_mapped_mask(desc, offset)) != 0u &&
            clock->fallback_write)
            clock->fallback_write(clock->fallback_ctx, addr, value);
        for (unsigned gate = 0u; gate < desc->gate_count; gate++) {
            const flexe_system_gate_desc_t *mapping = &desc->gate[gate];
            bool affected =
                (mapping->clock_offset == offset &&
                 (changed & mapping->clock_mask) != 0u) ||
                (mapping->reset_offset == offset &&
                 (changed & mapping->reset_mask) != 0u);
            if (affected) system_clock_publish_gate(clock, gate);
        }
        if (system_clock_low_power_offset(&desc->low_power, offset) &&
            (changed & system_clock_mapped_mask(desc, offset)) != 0u)
            system_clock_publish_low_power(clock);
        if (system_clock_peripheral_offset(desc, offset))
            system_clock_publish_peripheral(clock);
        return;
    }
    if (clock->fallback_write)
        clock->fallback_write(clock->fallback_ctx, addr, value);
}

flexe_system_clock_t *flexe_system_clock_create(
    xtensa_mem_t *mem, mmio_read_fn fallback_read,
    mmio_write_fn fallback_write, void *fallback_ctx,
    flexe_system_clock_gate_fn gate_changed, void *gate_ctx)
{
    if (!mem) return NULL;
    const flexe_target_desc_t *target = mem_target(mem);
    if (!system_clock_geometry_valid(target)) return NULL;

    flexe_system_clock_t *clock = calloc(1u, sizeof(*clock));
    if (!clock) return NULL;
    clock->mem = mem;
    clock->target = target;
    clock->fallback_read = fallback_read;
    clock->fallback_write = fallback_write;
    clock->fallback_ctx = fallback_ctx;
    clock->gate_changed = gate_changed;
    clock->gate_ctx = gate_ctx;
    clock->cpu_per_conf = target->system_clock.cpu_per_conf_reset;
    clock->sysclk_conf = target->system_clock.sysclk_conf_reset;
    for (unsigned index = 0u;
         index < target->system_clock.register_count; index++)
        clock->reg[index] = target->system_clock.reg[index].reset;

    const flexe_system_clock_desc_t *desc = &target->system_clock;
    if (mem_register_mmio_range(mem, desc->base, desc->register_size,
                                system_clock_read, system_clock_write,
                                clock) != 0) {
        free(clock);
        return NULL;
    }
    return clock;
}

bool flexe_system_clock_gate_state(
    const flexe_system_clock_t *clock, flexe_system_device_t device,
    unsigned instance, bool *clock_enabled, bool *reset_asserted)
{
    if (!clock) return false;
    const flexe_system_clock_desc_t *desc = &clock->target->system_clock;
    for (unsigned index = 0u; index < desc->gate_count; index++) {
        const flexe_system_gate_desc_t *gate = &desc->gate[index];
        if (gate->device != device || gate->instance != instance) continue;
        int clock_reg = system_clock_register_index(
            desc, gate->clock_offset);
        int reset_reg = system_clock_register_index(
            desc, gate->reset_offset);
        if (clock_reg < 0 || reset_reg < 0) return false;
        if (clock_enabled)
            *clock_enabled =
                (clock->reg[clock_reg] & gate->clock_mask) != 0u;
        if (reset_asserted)
            *reset_asserted =
                (clock->reg[reset_reg] & gate->reset_mask) != 0u;
        return true;
    }
    return false;
}

void flexe_system_clock_publish_gates(flexe_system_clock_t *clock)
{
    if (!clock) return;
    for (unsigned index = 0u;
         index < clock->target->system_clock.gate_count; index++)
        system_clock_publish_gate(clock, index);
}

bool flexe_system_clock_low_power_state(
    const flexe_system_clock_t *clock,
    flexe_system_low_power_state_t *state)
{
    if (!clock || !state) return false;
    const flexe_system_clock_desc_t *desc = &clock->target->system_clock;
    const flexe_system_low_power_desc_t *low = &desc->low_power;
    int memory_reg = system_clock_register_index(
        desc, low->memory_power_down_offset);
    int integer_reg = system_clock_register_index(
        desc, low->divider_integer_offset);
    int fraction_reg = system_clock_register_index(
        desc, low->divider_fraction_offset);
    if (memory_reg < 0 || integer_reg < 0 || fraction_reg < 0) return false;

    uint32_t fraction = clock->reg[fraction_reg];
    *state = (flexe_system_low_power_state_t){
        .memory_power_down_allowed =
            (clock->reg[memory_reg] &
             low->memory_power_down_inhibit_mask) == 0u,
        .rtc_clock_enabled =
            (fraction & low->rtc_clock_enable_mask) != 0u,
        .divider_integer =
            (clock->reg[integer_reg] & low->divider_integer_mask) >>
            system_clock_mask_shift(low->divider_integer_mask),
        .divider_a = (fraction & low->divider_a_mask) >>
                     system_clock_mask_shift(low->divider_a_mask),
        .divider_b = (fraction & low->divider_b_mask) >>
                     system_clock_mask_shift(low->divider_b_mask),
    };
    if ((fraction & low->source_rtc_slow_mask) != 0u)
        state->selected_sources |= FLEXE_SYSTEM_LOW_POWER_SOURCE_RTC_SLOW;
    if ((fraction & low->source_internal_mask) != 0u)
        state->selected_sources |= FLEXE_SYSTEM_LOW_POWER_SOURCE_INTERNAL;
    if ((fraction & low->source_xtal_mask) != 0u)
        state->selected_sources |= FLEXE_SYSTEM_LOW_POWER_SOURCE_XTAL;
    if ((fraction & low->source_xtal32k_mask) != 0u)
        state->selected_sources |= FLEXE_SYSTEM_LOW_POWER_SOURCE_XTAL32K;
    return true;
}

void flexe_system_clock_set_low_power_listener(
    flexe_system_clock_t *clock, flexe_system_low_power_fn fn, void *ctx)
{
    if (!clock) return;
    clock->low_power_changed = fn;
    clock->low_power_ctx = fn ? ctx : NULL;
    system_clock_publish_low_power(clock);
}

bool flexe_system_clock_peripheral_state(
    const flexe_system_clock_t *clock,
    flexe_system_peripheral_state_t *state)
{
    if (!clock || !state) return false;
    const flexe_system_clock_desc_t *desc = &clock->target->system_clock;
    const flexe_system_peripheral_banks_desc_t *banks =
        &desc->peripheral_banks;
    *state = (flexe_system_peripheral_state_t){0};
    for (unsigned bank = 0u; bank < banks->bank_count; bank++) {
        int clock_reg = system_clock_register_index(
            desc, banks->clock_offset[bank]);
        int reset_reg = system_clock_register_index(
            desc, banks->reset_offset[bank]);
        if (clock_reg < 0 || reset_reg < 0) return false;
        unsigned shift = bank * 32u;
        uint64_t mask = (uint64_t)banks->valid_mask[bank] << shift;
        state->valid_mask |= mask;
        state->clock_enabled |=
            ((uint64_t)clock->reg[clock_reg] << shift) & mask;
        state->reset_asserted |=
            ((uint64_t)clock->reg[reset_reg] << shift) & mask;
    }
    return true;
}

void flexe_system_clock_set_peripheral_listener(
    flexe_system_clock_t *clock, flexe_system_peripheral_fn fn, void *ctx)
{
    if (!clock) return;
    clock->peripheral_changed = fn;
    clock->peripheral_ctx = fn ? ctx : NULL;
    system_clock_publish_peripheral(clock);
}

void flexe_system_clock_destroy(flexe_system_clock_t *clock)
{
    if (!clock) return;
    const flexe_system_clock_desc_t *desc =
        &clock->target->system_clock;
    (void)mem_register_mmio_range(
        clock->mem, desc->base, desc->register_size,
        clock->fallback_read, clock->fallback_write,
        clock->fallback_ctx);
    free(clock);
}
