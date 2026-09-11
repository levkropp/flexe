#include "rtc_storage.h"

#include <stdbool.h>
#include <stdlib.h>

struct flexe_rtc_storage {
    xtensa_mem_t *mem;
    const flexe_target_desc_t *target;
    mmio_read_fn fallback_read;
    mmio_write_fn fallback_write;
    void *fallback_ctx;
    uint32_t store[FLEXE_TARGET_RTC_STORE_MAX];
};

static bool rtc_storage_geometry_valid(const flexe_target_desc_t *target)
{
    if (!target || !(target->capabilities &
                     FLEXE_TARGET_CAP_RTC_STORAGE_V1))
        return false;

    const flexe_rtc_storage_desc_t *desc = &target->rtc_storage;
    if (((desc->base | desc->register_size) & 0xFFFu) != 0u ||
        desc->register_size == 0u ||
        desc->base < target->peripheral_start ||
        desc->base >= target->peripheral_end ||
        desc->register_size > target->peripheral_end - desc->base ||
        desc->store_count == 0u ||
        desc->store_count > FLEXE_TARGET_RTC_STORE_MAX ||
        desc->slow_clock_hz == 0u ||
        desc->xtal_frequency_mhz == 0u ||
        desc->slow_clock_cal_store >= desc->store_count ||
        desc->xtal_frequency_store >= desc->store_count)
        return false;

    for (unsigned i = 0u; i < desc->store_count; i++) {
        uint32_t offset = desc->store_offset[i];
        if ((offset & 3u) != 0u ||
            offset > desc->register_size - sizeof(uint32_t))
            return false;
        for (unsigned j = 0u; j < i; j++)
            if (offset == desc->store_offset[j]) return false;
    }
    return true;
}

static int rtc_storage_index(const flexe_rtc_storage_desc_t *desc,
                             uint32_t offset)
{
    for (unsigned i = 0u; i < desc->store_count; i++)
        if (offset == desc->store_offset[i]) return (int)i;
    return -1;
}

static uint32_t rtc_storage_read(void *ctx, uint32_t addr)
{
    flexe_rtc_storage_t *storage = ctx;
    const flexe_rtc_storage_desc_t *desc = &storage->target->rtc_storage;
    int index = rtc_storage_index(desc, addr - desc->base);
    if (index >= 0) return storage->store[index];
    return storage->fallback_read ?
        storage->fallback_read(storage->fallback_ctx, addr) : 0u;
}

static void rtc_storage_write(void *ctx, uint32_t addr, uint32_t value)
{
    flexe_rtc_storage_t *storage = ctx;
    const flexe_rtc_storage_desc_t *desc = &storage->target->rtc_storage;
    int index = rtc_storage_index(desc, addr - desc->base);
    if (index >= 0) {
        storage->store[index] = value;
        return;
    }
    if (storage->fallback_write)
        storage->fallback_write(storage->fallback_ctx, addr, value);
}

flexe_rtc_storage_t *flexe_rtc_storage_create(
    xtensa_mem_t *mem, mmio_read_fn fallback_read,
    mmio_write_fn fallback_write, void *fallback_ctx)
{
    if (!mem) return NULL;
    const flexe_target_desc_t *target = mem_target(mem);
    if (!rtc_storage_geometry_valid(target)) return NULL;

    flexe_rtc_storage_t *storage = calloc(1u, sizeof(*storage));
    if (!storage) return NULL;
    storage->mem = mem;
    storage->target = target;
    storage->fallback_read = fallback_read;
    storage->fallback_write = fallback_write;
    storage->fallback_ctx = fallback_ctx;
    const flexe_rtc_storage_desc_t *desc = &target->rtc_storage;
    for (unsigned i = 0u; i < desc->store_count; i++)
        storage->store[i] = desc->store_reset[i];

    if (mem_register_mmio_range(mem, desc->base, desc->register_size,
                                rtc_storage_read, rtc_storage_write,
                                storage) != 0) {
        free(storage);
        return NULL;
    }
    return storage;
}

void flexe_rtc_storage_destroy(flexe_rtc_storage_t *storage)
{
    if (!storage) return;
    const flexe_rtc_storage_desc_t *desc = &storage->target->rtc_storage;
    (void)mem_register_mmio_range(
        storage->mem, desc->base, desc->register_size,
        storage->fallback_read, storage->fallback_write,
        storage->fallback_ctx);
    free(storage);
}

void flexe_rtc_storage_application_handoff(flexe_rtc_storage_t *storage)
{
    if (!storage) return;
    const flexe_rtc_storage_desc_t *desc = &storage->target->rtc_storage;
    uint64_t calibration = (UINT64_C(1000000) << 19) /
                           desc->slow_clock_hz;
    storage->store[desc->slow_clock_cal_store] = (uint32_t)calibration;

    uint32_t xtal = desc->xtal_frequency_mhz & UINT16_MAX;
    storage->store[desc->xtal_frequency_store] = xtal | (xtal << 16);
}
