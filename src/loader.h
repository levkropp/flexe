#ifndef LOADER_H
#define LOADER_H

#include "memory.h"
#include <stdint.h>

#define MAX_SEGMENTS 16

typedef struct {
    uint32_t addr;
    uint32_t size;
} segment_info_t;

typedef struct {
    uint32_t entry_point;
    int segment_count;
    int result;         /* 0 = success */
    char error[256];
    segment_info_t segments[MAX_SEGMENTS];
} load_result_t;

load_result_t loader_load_bin(xtensa_mem_t *mem, const char *path);

/* Describe what memory region an address falls in (for diagnostics) */
const char *loader_region_name(uint32_t addr);

#endif /* LOADER_H */
