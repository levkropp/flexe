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

/* Auto-detect by default, or require an explicit target. */
load_result_t loader_load_bin(xtensa_mem_t *mem, const char *path);
load_result_t loader_load_bin_for_target(xtensa_mem_t *mem, const char *path,
                                         flexe_target_id_t expected_target);

/* Describe what memory region an address falls in (for diagnostics) */
const char *loader_region_name(uint32_t addr);

#endif /* LOADER_H */
