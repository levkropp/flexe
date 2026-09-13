/* Target-described internal analog-register I2C fabric. */
#ifndef FLEXE_REGI2C_H
#define FLEXE_REGI2C_H

#include <stdbool.h>
#include "memory.h"

typedef struct flexe_regi2c flexe_regi2c_t;

/* Unknown addresses are delegated to the supplied fallback handlers so a
 * partially modeled register page remains visible to diagnostic counters. */
flexe_regi2c_t *flexe_regi2c_create(xtensa_mem_t *mem,
                                    mmio_read_fn fallback_read,
                                    mmio_write_fn fallback_write,
                                    void *fallback_ctx);
void flexe_regi2c_destroy(flexe_regi2c_t *regi2c);

/* Read an internal analog slave register for another modeled device that
 * shares this fabric (for example, the SAR ADC's calibration-ground mux). */
bool flexe_regi2c_register_read(const flexe_regi2c_t *regi2c,
                                unsigned slave, unsigned address,
                                uint8_t *value);

#endif /* FLEXE_REGI2C_H */
