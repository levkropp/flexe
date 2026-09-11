#include "io_mux.h"

#include <stdbool.h>
#include <stdlib.h>

struct flexe_io_mux {
    xtensa_mem_t *mem;
    const flexe_target_desc_t *target;
    mmio_read_fn fallback_read;
    mmio_write_fn fallback_write;
    void *fallback_ctx;
    uint32_t reg[FLEXE_TARGET_IO_MUX_REGISTER_MAX];
    uint64_t valid;
    uint64_t written;
};

static bool io_mux_geometry_valid(const flexe_target_desc_t *target)
{
    if (!target || !(target->capabilities & FLEXE_TARGET_CAP_IO_MUX_V1))
        return false;

    const flexe_io_mux_desc_t *desc = &target->io_mux;
    if (((desc->base | desc->register_size) & 0xFFFu) != 0u ||
        desc->register_size == 0u ||
        desc->base < target->peripheral_start ||
        desc->base >= target->peripheral_end ||
        desc->register_size > target->peripheral_end - desc->base ||
        desc->register_count == 0u ||
        desc->register_count > FLEXE_TARGET_IO_MUX_REGISTER_MAX ||
        (uint32_t)desc->register_count * sizeof(uint32_t) >
            desc->register_size ||
        desc->gpio_count > FLEXE_TARGET_GPIO_MAX ||
        (desc->control_offset & 3u) != 0u ||
        desc->control_offset >=
            (uint32_t)desc->register_count * sizeof(uint32_t) ||
        desc->control_writable_mask == 0u ||
        desc->register_writable_mask == 0u ||
        desc->function_shift >= 32u ||
        (desc->function_mask >> desc->function_shift) == 0u ||
        (desc->function_mask & ~desc->register_writable_mask) != 0u)
        return false;

    if (desc->date_offset != UINT32_MAX &&
        ((desc->date_offset & 3u) != 0u ||
         desc->date_offset > desc->register_size - sizeof(uint32_t) ||
         desc->date_offset == desc->control_offset))
        return false;

    uint32_t register_bytes =
        (uint32_t)desc->register_count * sizeof(uint32_t);
    for (unsigned gpio = 0u; gpio < desc->gpio_count; gpio++) {
        uint32_t offset = desc->gpio_register_offset[gpio];
        if (offset == FLEXE_TARGET_IO_MUX_OFFSET_NONE) continue;
        if ((offset & 3u) != 0u || offset >= register_bytes ||
            offset == desc->control_offset || offset == desc->date_offset)
            return false;
    }
    return true;
}

static uint32_t io_mux_read(void *ctx, uint32_t addr)
{
    flexe_io_mux_t *io_mux = ctx;
    const flexe_io_mux_desc_t *desc = &io_mux->target->io_mux;
    uint32_t offset = addr - desc->base;

    if (offset == desc->date_offset) return desc->date_reset;
    if ((offset & 3u) == 0u) {
        uint32_t index = offset / sizeof(uint32_t);
        if (index < desc->register_count &&
            (io_mux->valid & (UINT64_C(1) << index)) != 0u)
            return io_mux->reg[index];
    }
    return io_mux->fallback_read ?
        io_mux->fallback_read(io_mux->fallback_ctx, addr) : 0u;
}

static void io_mux_write(void *ctx, uint32_t addr, uint32_t value)
{
    flexe_io_mux_t *io_mux = ctx;
    const flexe_io_mux_desc_t *desc = &io_mux->target->io_mux;
    uint32_t offset = addr - desc->base;

    if (offset == desc->date_offset) return;
    if ((offset & 3u) == 0u) {
        uint32_t index = offset / sizeof(uint32_t);
        if (index < desc->register_count &&
            (io_mux->valid & (UINT64_C(1) << index)) != 0u) {
            uint32_t writable = offset == desc->control_offset ?
                desc->control_writable_mask :
                desc->register_writable_mask;
            io_mux->reg[index] =
                (io_mux->reg[index] & ~writable) | (value & writable);
            io_mux->written |= UINT64_C(1) << index;
            return;
        }
    }
    if (io_mux->fallback_write)
        io_mux->fallback_write(io_mux->fallback_ctx, addr, value);
}

flexe_io_mux_t *flexe_io_mux_create(
    xtensa_mem_t *mem, mmio_read_fn fallback_read,
    mmio_write_fn fallback_write, void *fallback_ctx)
{
    if (!mem) return NULL;
    const flexe_target_desc_t *target = mem_target(mem);
    if (!io_mux_geometry_valid(target)) return NULL;

    flexe_io_mux_t *io_mux = calloc(1u, sizeof(*io_mux));
    if (!io_mux) return NULL;
    io_mux->mem = mem;
    io_mux->target = target;
    io_mux->fallback_read = fallback_read;
    io_mux->fallback_write = fallback_write;
    io_mux->fallback_ctx = fallback_ctx;
    const flexe_io_mux_desc_t *desc = &target->io_mux;
    unsigned control = desc->control_offset / sizeof(uint32_t);
    io_mux->valid |= UINT64_C(1) << control;
    io_mux->reg[control] = desc->control_reset;
    for (unsigned gpio = 0u; gpio < desc->gpio_count; gpio++) {
        uint32_t offset = desc->gpio_register_offset[gpio];
        if (offset == FLEXE_TARGET_IO_MUX_OFFSET_NONE) continue;
        unsigned index = offset / sizeof(uint32_t);
        io_mux->valid |= UINT64_C(1) << index;
        io_mux->reg[index] = desc->register_reset;
    }

    if (mem_register_mmio_range(mem, desc->base, desc->register_size,
                                io_mux_read, io_mux_write, io_mux) != 0) {
        free(io_mux);
        return NULL;
    }
    return io_mux;
}

void flexe_io_mux_destroy(flexe_io_mux_t *io_mux)
{
    if (!io_mux) return;
    const flexe_io_mux_desc_t *desc = &io_mux->target->io_mux;
    (void)mem_register_mmio_range(
        io_mux->mem, desc->base, desc->register_size,
        io_mux->fallback_read, io_mux->fallback_write,
        io_mux->fallback_ctx);
    free(io_mux);
}

int flexe_io_mux_function(const flexe_io_mux_t *io_mux, unsigned gpio)
{
    if (!io_mux || gpio >= io_mux->target->io_mux.gpio_count) return -1;
    const flexe_io_mux_desc_t *desc = &io_mux->target->io_mux;
    uint32_t offset = desc->gpio_register_offset[gpio];
    if (offset == FLEXE_TARGET_IO_MUX_OFFSET_NONE) return -1;
    uint32_t index = offset / sizeof(uint32_t);
    if ((io_mux->written & (UINT64_C(1) << index)) == 0u) return -1;
    return (int)((io_mux->reg[index] & desc->function_mask) >>
                 desc->function_shift);
}
