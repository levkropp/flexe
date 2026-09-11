/* Descriptor-driven ESP32-family digital pad configuration. */
#ifndef FLEXE_IO_MUX_H
#define FLEXE_IO_MUX_H

#include "memory.h"

typedef struct flexe_io_mux flexe_io_mux_t;

flexe_io_mux_t *flexe_io_mux_create(
    xtensa_mem_t *mem, mmio_read_fn fallback_read,
    mmio_write_fn fallback_write, void *fallback_ctx);
void flexe_io_mux_destroy(flexe_io_mux_t *io_mux);

/* Return the explicitly programmed MCU_SEL value for a target GPIO, or -1
 * when the pin has no IO_MUX register or firmware has not selected one. */
int flexe_io_mux_function(const flexe_io_mux_t *io_mux, unsigned gpio);

#endif /* FLEXE_IO_MUX_H */
