/* Descriptor-driven ESP32-family read-only eFuse profiles. */
#ifndef FLEXE_EFUSE_H
#define FLEXE_EFUSE_H

#include "memory.h"

typedef struct flexe_efuse flexe_efuse_t;

flexe_efuse_t *flexe_efuse_create(
    xtensa_mem_t *mem, mmio_read_fn fallback_read,
    mmio_write_fn fallback_write, void *fallback_ctx);
void flexe_efuse_destroy(flexe_efuse_t *efuse);

#endif /* FLEXE_EFUSE_H */
