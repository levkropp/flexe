/* Target-described SENSITIVE v1 memory-protection register block. */
#ifndef FLEXE_SENSITIVE_MEMPROT_H
#define FLEXE_SENSITIVE_MEMPROT_H

#include "memory.h"

typedef struct flexe_sensitive_memprot flexe_sensitive_memprot_t;

/* Unknown offsets remain diagnostic through the fallback handlers. */
flexe_sensitive_memprot_t *flexe_sensitive_memprot_create(
    xtensa_mem_t *mem, mmio_read_fn fallback_read,
    mmio_write_fn fallback_write, void *fallback_ctx);
void flexe_sensitive_memprot_destroy(flexe_sensitive_memprot_t *memprot);

#endif /* FLEXE_SENSITIVE_MEMPROT_H */
