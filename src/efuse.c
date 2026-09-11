#include "efuse.h"

#include <stdbool.h>
#include <stdlib.h>

struct flexe_efuse {
    xtensa_mem_t *mem;
    const flexe_target_desc_t *target;
    mmio_read_fn fallback_read;
    mmio_write_fn fallback_write;
    void *fallback_ctx;
    uint32_t date;
};

static bool efuse_geometry_valid(const flexe_target_desc_t *target)
{
    if (!target || !(target->capabilities & FLEXE_TARGET_CAP_EFUSE_READ_V1))
        return false;

    const flexe_efuse_desc_t *desc = &target->efuse;
    if (((desc->base | desc->register_size) & 0xFFFu) != 0u ||
        desc->register_size == 0u ||
        desc->base < target->peripheral_start ||
        desc->base >= target->peripheral_end ||
        desc->register_size > target->peripheral_end - desc->base ||
        desc->read_data_word_count == 0u ||
        desc->read_data_word_count > FLEXE_TARGET_EFUSE_READ_WORD_MAX ||
        (desc->read_data_offset & 3u) != 0u ||
        desc->read_data_offset >
            desc->register_size - sizeof(uint32_t) ||
        desc->read_data_word_count >
            (desc->register_size - desc->read_data_offset) /
                sizeof(uint32_t) ||
        (desc->date_offset & 3u) != 0u ||
        desc->date_offset > desc->register_size - sizeof(uint32_t) ||
        (desc->date_offset >= desc->read_data_offset &&
         (desc->date_offset - desc->read_data_offset) /
             sizeof(uint32_t) < desc->read_data_word_count) ||
        desc->date_writable_mask == 0u)
        return false;
    return true;
}

static bool efuse_read_word(const flexe_efuse_desc_t *desc, uint32_t offset,
                            unsigned *index)
{
    if (offset < desc->read_data_offset || (offset & 3u) != 0u)
        return false;
    uint32_t relative = offset - desc->read_data_offset;
    unsigned word = relative / sizeof(uint32_t);
    if (word >= desc->read_data_word_count) return false;
    *index = word;
    return true;
}

static uint32_t efuse_read(void *ctx, uint32_t addr)
{
    flexe_efuse_t *efuse = ctx;
    const flexe_efuse_desc_t *desc = &efuse->target->efuse;
    uint32_t offset = addr - desc->base;
    unsigned index = 0u;
    if (efuse_read_word(desc, offset, &index))
        return desc->read_data[index];
    if (offset == desc->date_offset) return efuse->date;
    return efuse->fallback_read ?
        efuse->fallback_read(efuse->fallback_ctx, addr) : 0u;
}

static void efuse_write(void *ctx, uint32_t addr, uint32_t value)
{
    flexe_efuse_t *efuse = ctx;
    const flexe_efuse_desc_t *desc = &efuse->target->efuse;
    uint32_t offset = addr - desc->base;
    unsigned index = 0u;
    if (efuse_read_word(desc, offset, &index))
        return; /* Physical fuse read views are read-only. */
    if (offset == desc->date_offset) {
        efuse->date = (efuse->date & ~desc->date_writable_mask) |
                      (value & desc->date_writable_mask);
        return;
    }
    if (efuse->fallback_write)
        efuse->fallback_write(efuse->fallback_ctx, addr, value);
}

flexe_efuse_t *flexe_efuse_create(
    xtensa_mem_t *mem, mmio_read_fn fallback_read,
    mmio_write_fn fallback_write, void *fallback_ctx)
{
    if (!mem) return NULL;
    const flexe_target_desc_t *target = mem_target(mem);
    if (!efuse_geometry_valid(target)) return NULL;

    flexe_efuse_t *efuse = calloc(1u, sizeof(*efuse));
    if (!efuse) return NULL;
    efuse->mem = mem;
    efuse->target = target;
    efuse->fallback_read = fallback_read;
    efuse->fallback_write = fallback_write;
    efuse->fallback_ctx = fallback_ctx;
    efuse->date = target->efuse.date_reset;

    const flexe_efuse_desc_t *desc = &target->efuse;
    if (mem_register_mmio_range(mem, desc->base, desc->register_size,
                                efuse_read, efuse_write, efuse) != 0) {
        free(efuse);
        return NULL;
    }
    return efuse;
}

void flexe_efuse_destroy(flexe_efuse_t *efuse)
{
    if (!efuse) return;
    const flexe_efuse_desc_t *desc = &efuse->target->efuse;
    (void)mem_register_mmio_range(
        efuse->mem, desc->base, desc->register_size,
        efuse->fallback_read, efuse->fallback_write,
        efuse->fallback_ctx);
    free(efuse);
}
