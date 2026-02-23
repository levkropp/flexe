#ifndef LOADER_H
#define LOADER_H

#include "memory.h"
#include <stdint.h>

typedef struct {
    uint32_t entry_point;
    int segment_count;
    int result;         /* 0 = success */
    char error[256];
} load_result_t;

load_result_t loader_load_bin(xtensa_mem_t *mem, const char *path);

#endif /* LOADER_H */
