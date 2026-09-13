#ifndef LOADER_H
#define LOADER_H

#include "memory.h"
#include "target.h"
#include <stddef.h>
#include <stdint.h>

#define MAX_SEGMENTS 16

typedef struct {
    uint32_t addr;
    uint32_t size;
    uint32_t image_off;  /* file offset of segment data within its image */
} segment_info_t;

typedef struct {
    flexe_target_id_t target;
    uint16_t chip_id;
    uint8_t min_chip_rev;
    uint16_t min_chip_rev_full;
    uint16_t max_chip_rev_full;
    uint32_t image_offset;
    uint32_t flash_size;   /* Capacity declared by the standard image header */
} loader_image_info_t;

typedef struct {
    uint32_t entry_point;
    int segment_count;
    int result;         /* 0 = success */
    char error[256];
    loader_image_info_t image;
    segment_info_t segments[MAX_SEGMENTS];
} load_result_t;

/* Identify an app image without mutating emulator memory. */
int loader_probe_bin(const char *path, loader_image_info_t *info,
                     char *error, size_t error_size);

/* Auto-detect by default, or require an explicit target. The memory object
 * must have been constructed for that target. Loading itself is permitted for
 * recognized experimental targets; execution readiness belongs to sessions. */
load_result_t loader_load_bin(xtensa_mem_t *mem, const char *path);
load_result_t loader_load_bin_for_target(xtensa_mem_t *mem, const char *path,
                                         flexe_target_id_t expected_target);

/* Rebuild an application after a software reset without reflashing the
 * original image over guest-written NVS/filesystem partitions. Internal
 * segments are restored from the image, while the live NOR backing and its
 * partition table survive. Changes to the originally loaded application
 * bytes are rejected; OTA slot selection is not modeled. Call only after a
 * successful cold load. */
load_result_t loader_rebuild_bin_for_target(xtensa_mem_t *mem, const char *path,
                                            flexe_target_id_t expected_target);

/* Describe what memory region an address falls in (for diagnostics) */
const char *loader_region_name(uint32_t addr);
const char *loader_region_name_for_target(const flexe_target_desc_t *target,
                                          uint32_t addr);

#endif /* LOADER_H */
