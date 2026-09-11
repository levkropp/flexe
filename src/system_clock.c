#include "system_clock.h"

#include <stdbool.h>
#include <stdlib.h>

struct flexe_system_clock {
    xtensa_mem_t *mem;
    const flexe_target_desc_t *target;
    mmio_read_fn fallback_read;
    mmio_write_fn fallback_write;
    void *fallback_ctx;
    uint32_t cpu_per_conf;
    uint32_t sysclk_conf;
};

static bool system_clock_register_valid(uint32_t offset,
                                        uint32_t register_size)
{
    return (offset & 3u) == 0u && register_size >= sizeof(uint32_t) &&
           offset <= register_size - sizeof(uint32_t);
}

static bool system_clock_geometry_valid(const flexe_target_desc_t *target)
{
    if (!target || !(target->capabilities &
                     FLEXE_TARGET_CAP_SYSTEM_CLOCK_V1))
        return false;

    const flexe_system_clock_desc_t *desc = &target->system_clock;
    return ((desc->base | desc->register_size) & 0xFFFu) == 0u &&
           desc->register_size != 0u &&
           desc->base >= target->peripheral_start &&
           desc->base < target->peripheral_end &&
           desc->register_size <= target->peripheral_end - desc->base &&
           system_clock_register_valid(desc->cpu_per_conf_offset,
                                       desc->register_size) &&
           system_clock_register_valid(desc->sysclk_conf_offset,
                                       desc->register_size) &&
           desc->cpu_per_conf_offset != desc->sysclk_conf_offset &&
           desc->cpu_per_conf_writable_mask != 0u &&
           desc->sysclk_conf_writable_mask != 0u;
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
    if (clock->fallback_write)
        clock->fallback_write(clock->fallback_ctx, addr, value);
}

flexe_system_clock_t *flexe_system_clock_create(
    xtensa_mem_t *mem, mmio_read_fn fallback_read,
    mmio_write_fn fallback_write, void *fallback_ctx)
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
    clock->cpu_per_conf = target->system_clock.cpu_per_conf_reset;
    clock->sysclk_conf = target->system_clock.sysclk_conf_reset;

    const flexe_system_clock_desc_t *desc = &target->system_clock;
    if (mem_register_mmio_range(mem, desc->base, desc->register_size,
                                system_clock_read, system_clock_write,
                                clock) != 0) {
        free(clock);
        return NULL;
    }
    return clock;
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
