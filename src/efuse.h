/* Descriptor-driven ESP32-family eFuse profiles with optional blob override. */
#ifndef FLEXE_EFUSE_H
#define FLEXE_EFUSE_H

#include "memory.h"

/* QEMU-compatible eFuse blob: 84 little-endian words (336 bytes) holding
 * BLK0 (6 words), BLK1 (6), BLK2 (8), BLK3 (8), and the remaining key and
 * system blocks. Only identity fields are consumed today; see
 * flexe_efuse_apply_blob. */
#define FLEXE_EFUSE_BLOB_WORDS 84u
#define FLEXE_EFUSE_BLOB_BYTES (FLEXE_EFUSE_BLOB_WORDS * 4u)

typedef struct flexe_efuse flexe_efuse_t;

flexe_efuse_t *flexe_efuse_create(
    xtensa_mem_t *mem, mmio_read_fn fallback_read,
    mmio_write_fn fallback_write, void *fallback_ctx);
void flexe_efuse_destroy(flexe_efuse_t *efuse);

/* Read a blob file into words. Returns 0 on success, -1 on I/O error or when
 * the file is not exactly FLEXE_EFUSE_BLOB_BYTES long. */
int flexe_efuse_blob_load(const char *path,
                           uint32_t words[FLEXE_EFUSE_BLOB_WORDS]);

/* Merge blob identity fields into this instance's read image. Returns 0 when
 * applied, 1 when the session target has no blob field map yet (the compiled
 * profile stays in effect), -1 on bad input. Never touches the shared target
 * descriptor: each instance owns its image. */
int flexe_efuse_apply_blob(flexe_efuse_t *efuse,
                            const uint32_t words[FLEXE_EFUSE_BLOB_WORDS]);

#endif /* FLEXE_EFUSE_H */
