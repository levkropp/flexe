#include "system_clock.h"

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
        desc->gate_count > FLEXE_TARGET_SYSTEM_GATE_MAX)
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
             (target->capabilities & FLEXE_TARGET_CAP_I2C_V1));
        if (!device_valid || clock_reg < 0 || reset_reg < 0 ||
            !system_clock_single_bit(gate->clock_mask) ||
            !system_clock_single_bit(gate->reset_mask) ||
            (gate->clock_mask &
             ~desc->reg[clock_reg].writable_mask) != 0u ||
            (gate->reset_mask &
             ~desc->reg[reset_reg].writable_mask) != 0u)
            return false;
        for (unsigned old = 0u; old < index; old++)
            if (gate->device == desc->gate[old].device &&
                gate->instance == desc->gate[old].instance)
                return false;
    }
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
    return mask;
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
