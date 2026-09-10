/* Target-described internal analog-register I2C fabric. */
#ifndef FLEXE_REGI2C_H
#define FLEXE_REGI2C_H

#include "memory.h"

typedef struct flexe_regi2c flexe_regi2c_t;

/* Unknown addresses are delegated to the supplied fallback handlers so a
 * partially modeled register page remains visible to diagnostic counters. */
flexe_regi2c_t *flexe_regi2c_create(xtensa_mem_t *mem,
                                    mmio_read_fn fallback_read,
                                    mmio_write_fn fallback_write,
                                    void *fallback_ctx);
void flexe_regi2c_destroy(flexe_regi2c_t *regi2c);

#endif /* FLEXE_REGI2C_H */
