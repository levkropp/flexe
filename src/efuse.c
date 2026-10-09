#include "efuse.h"
#include "target.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

struct flexe_efuse {
    xtensa_mem_t *mem;
    const flexe_target_desc_t *target;
    mmio_read_fn fallback_read;
    mmio_write_fn fallback_write;
    void *fallback_ctx;
    uint32_t date;
    /* Per-instance read image. Starts as the compiled profile; a blob file
     * or (in the future) guest burn commands merge fields into it. The
     * shared target descriptor is never written. */
    uint32_t image[FLEXE_TARGET_EFUSE_READ_WORD_MAX];
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
        return efuse->image[index];
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
    for (unsigned i = 0u; i < desc->read_data_word_count; i++)
        efuse->image[i] = desc->read_data[i];
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

int flexe_efuse_blob_load(const char *path,
                           uint32_t words[FLEXE_EFUSE_BLOB_WORDS])
{
    if (!path || !words) return -1;
    FILE *f = fopen(path, "rb");
    if (!f) return -1;
    uint8_t raw[FLEXE_EFUSE_BLOB_BYTES];
    size_t got = fread(raw, 1u, sizeof(raw), f);
    int trailing = fgetc(f);
    fclose(f);
    if (got != sizeof(raw) || trailing != EOF) return -1;
    for (unsigned i = 0u; i < FLEXE_EFUSE_BLOB_WORDS; i++) {
        words[i] = (uint32_t)raw[4u * i] |
                   ((uint32_t)raw[4u * i + 1u] << 8) |
                   ((uint32_t)raw[4u * i + 2u] << 16) |
                   ((uint32_t)raw[4u * i + 3u] << 24);
    }
    return 0;
}

/* One merged blob field: SN source bits land in one instance-image word.
 * Register/bit facts follow ESP-IDF's esp32s3 efuse table and register
 * definitions (RD_MAC_SPI_SYS_* hold BLOCK1 slices; RD_SYS_PART1_DATA4 holds
 * BLOCK2 word 4). Widths here never cross a word boundary. */
typedef struct {
    unsigned blob_word;
    unsigned blob_shift;
    unsigned width;
    unsigned image_word;
    unsigned image_shift;
} efuse_blob_field_t;

static void efuse_merge_field(uint32_t *image,
                               const uint32_t *blob,
                               const efuse_blob_field_t *field)
{
    uint32_t value = (blob[field->blob_word] >> field->blob_shift) &
                     (field->width >= 32u ? UINT32_MAX
                                          : ((1u << field->width) - 1u));
    uint32_t mask = (field->width >= 32u ? UINT32_MAX
                                         : ((1u << field->width) - 1u))
                    << field->image_shift;
    image[field->image_word] =
        (image[field->image_word] & ~mask) |
        ((value << field->image_shift) & mask);
}

/* ESP32-S3 identity fields: factory MAC (BLK1[47:0]), wafer revision
 * (BLK1 minor lo/hi + major), and ADC-calibration block versions (BLK1
 * minor, BLK2 major). Package, flash/PSRAM capacity, and analog trim stay at
 * profile defaults until a consumer needs them. */
static const efuse_blob_field_t efuse_s3_blob_fields[] = {
    { 6u, 0u, 32u, 6u, 0u },    /* MAC low 32 */
    { 7u, 0u, 16u, 7u, 0u },    /* MAC high 16 */
    { 9u, 18u, 3u, 9u, 18u },   /* WAFER_VERSION_MINOR_LO */
    { 9u, 24u, 3u, 9u, 24u },   /* BLK_VERSION_MINOR */
    { 11u, 23u, 1u, 11u, 23u }, /* WAFER_VERSION_MINOR_HI */
    { 11u, 24u, 2u, 11u, 24u }, /* WAFER_VERSION_MAJOR */
    { 16u, 0u, 2u, 16u, 0u },   /* BLK_VERSION_MAJOR */
};

int flexe_efuse_apply_blob(flexe_efuse_t *efuse,
                            const uint32_t words[FLEXE_EFUSE_BLOB_WORDS])
{
    if (!efuse || !words) return -1;
    if (efuse->target->id != FLEXE_TARGET_ESP32S3) return 1;
    if (efuse->target->efuse.read_data_word_count <= 16u) return -1;
    for (unsigned i = 0u;
         i < sizeof(efuse_s3_blob_fields) / sizeof(efuse_s3_blob_fields[0]);
         i++)
        efuse_merge_field(efuse->image, words, &efuse_s3_blob_fields[i]);
    return 0;
}
