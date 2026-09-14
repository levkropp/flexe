/* Descriptor-driven ESP32-family digital pad configuration. */
#ifndef FLEXE_IO_MUX_H
#define FLEXE_IO_MUX_H

#include <stdbool.h>

#include "memory.h"

typedef struct flexe_io_mux flexe_io_mux_t;
typedef void (*flexe_io_mux_input_changed_fn)(void *ctx, unsigned gpio,
                                               bool enabled);

flexe_io_mux_t *flexe_io_mux_create(
    xtensa_mem_t *mem, mmio_read_fn fallback_read,
    mmio_write_fn fallback_write, void *fallback_ctx);
void flexe_io_mux_destroy(flexe_io_mux_t *io_mux);

/* Return the explicitly programmed MCU_SEL value for a target GPIO, or -1
 * when the pin has no IO_MUX register or firmware has not selected one. */
int flexe_io_mux_function(const flexe_io_mux_t *io_mux, unsigned gpio);

/* Input-buffer state comes from the target's per-pad FUN_IE field. Return -1
 * for an unbonded pad or a target without an input-enable field. */
int flexe_io_mux_input_enabled(const flexe_io_mux_t *io_mux, unsigned gpio);
void flexe_io_mux_set_input_changed_handler(
    flexe_io_mux_t *io_mux, flexe_io_mux_input_changed_fn changed, void *ctx);

#endif /* FLEXE_IO_MUX_H */
