#include "apb_saradc.h"
#include "target.h"

#include <stdbool.h>
#include <stdlib.h>

struct flexe_apb_saradc {
    xtensa_mem_t *mem;
    const flexe_target_desc_t *target;
    mmio_read_fn fallback_read;
    mmio_write_fn fallback_write;
    void *fallback_ctx;
    uint32_t arbiter;
};

static bool one_bit(uint32_t value)
{
    return value != 0u && (value & (value - 1u)) == 0u;
}

static bool geometry_valid(const flexe_target_desc_t *target)
{
    if (!target ||
        !(target->capabilities & FLEXE_TARGET_CAP_APB_SARADC_V1))
        return false;
    const flexe_apb_saradc_desc_t *desc = &target->apb_saradc;
    return (desc->base & 0xFFFu) == 0u &&
           desc->register_size >= sizeof(uint32_t) &&
           (desc->register_size & 0xFFFu) == 0u &&
           desc->base >= target->peripheral_start &&
           desc->base < target->peripheral_end &&
           desc->register_size <= target->peripheral_end - desc->base &&
           (desc->arbiter_offset & 3u) == 0u &&
           desc->arbiter_offset <=
               desc->register_size - sizeof(uint32_t) &&
           desc->arbiter_writable_mask != 0u &&
           (desc->arbiter_reset & ~desc->arbiter_writable_mask) == 0u &&
           one_bit(desc->grant_force_mask) &&
           one_bit(desc->rtc_force_mask) &&
           (desc->grant_force_mask & desc->force_selection_mask) == 0u &&
           (desc->force_selection_mask & desc->rtc_force_mask) != 0u &&
           (desc->grant_force_mask | desc->force_selection_mask) ==
               ((desc->grant_force_mask | desc->force_selection_mask) &
                desc->arbiter_writable_mask);
}

static uint32_t apb_saradc_read(void *ctx, uint32_t addr)
{
    flexe_apb_saradc_t *adc = ctx;
    const flexe_apb_saradc_desc_t *desc = &adc->target->apb_saradc;
    if (addr == desc->base + desc->arbiter_offset)
        return adc->arbiter;
    return adc->fallback_read ?
        adc->fallback_read(adc->fallback_ctx, addr) : 0u;
}

static void apb_saradc_write(void *ctx, uint32_t addr, uint32_t value)
{
    flexe_apb_saradc_t *adc = ctx;
    const flexe_apb_saradc_desc_t *desc = &adc->target->apb_saradc;
    if (addr == desc->base + desc->arbiter_offset) {
        adc->arbiter = value & desc->arbiter_writable_mask;
        bool force_other =
            (adc->arbiter & desc->grant_force_mask) != 0u &&
            (adc->arbiter & desc->force_selection_mask) !=
                desc->rtc_force_mask;
        if (force_other || (value & ~desc->arbiter_writable_mask) != 0u) {
            /* Other forced controllers and reserved fields are retained or
             * ignored respectively, but remain diagnostic. */
            if (adc->fallback_write)
                adc->fallback_write(adc->fallback_ctx, addr, value);
        }
        return;
    }
    if (adc->fallback_write)
        adc->fallback_write(adc->fallback_ctx, addr, value);
}

flexe_apb_saradc_t *flexe_apb_saradc_create(
    xtensa_mem_t *mem, mmio_read_fn fallback_read,
    mmio_write_fn fallback_write, void *fallback_ctx)
{
    if (!mem) return NULL;
    const flexe_target_desc_t *target = mem_target(mem);
    if (!geometry_valid(target)) return NULL;
    flexe_apb_saradc_t *adc = calloc(1u, sizeof(*adc));
    if (!adc) return NULL;
    adc->mem = mem;
    adc->target = target;
    adc->fallback_read = fallback_read;
    adc->fallback_write = fallback_write;
    adc->fallback_ctx = fallback_ctx;
    adc->arbiter = target->apb_saradc.arbiter_reset;
    if (mem_register_mmio_range(mem, target->apb_saradc.base,
                                target->apb_saradc.register_size,
                                apb_saradc_read, apb_saradc_write, adc) != 0) {
        free(adc);
        return NULL;
    }
    return adc;
}

void flexe_apb_saradc_destroy(flexe_apb_saradc_t *adc)
{
    if (!adc) return;
    const flexe_apb_saradc_desc_t *desc = &adc->target->apb_saradc;
    (void)mem_register_mmio_range(
        adc->mem, desc->base, desc->register_size,
        adc->fallback_read, adc->fallback_write, adc->fallback_ctx);
    free(adc);
}

bool flexe_apb_saradc_rtc_granted(const flexe_apb_saradc_t *adc)
{
    if (!adc) return false;
    const flexe_apb_saradc_desc_t *desc = &adc->target->apb_saradc;
    return (adc->arbiter & desc->grant_force_mask) == 0u ||
           (adc->arbiter & desc->force_selection_mask) ==
               desc->rtc_force_mask;
}
