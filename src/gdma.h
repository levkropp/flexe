/* Target-described ESP32-family general-purpose DMA v1 controller. */
#ifndef FLEXE_GDMA_H
#define FLEXE_GDMA_H

#include "memory.h"

typedef struct flexe_gdma flexe_gdma_t;

flexe_gdma_t *flexe_gdma_create(
    xtensa_mem_t *mem, mmio_read_fn fallback_read,
    mmio_write_fn fallback_write, void *fallback_ctx);
void flexe_gdma_destroy(flexe_gdma_t *gdma);

/* Consume bytes from the active TX descriptor chain routed to peripheral_id.
 * Returns zero only when exactly `length` bytes were copied. The descriptor
 * format, owner write-back, completion registers, and link-state transition
 * follow GDMA v1; the peripheral using the stream owns its payload semantics. */
int flexe_gdma_read_tx(flexe_gdma_t *gdma, uint8_t peripheral_id,
                       uint8_t *data, size_t length);

/* Write bytes into the active RX descriptor chain routed from peripheral_id.
 * The transfer updates descriptor lengths/ownership and completion state just
 * as a target peripheral driving the GDMA input would. */
int flexe_gdma_write_rx(flexe_gdma_t *gdma, uint8_t peripheral_id,
                        const uint8_t *data, size_t length);

#endif /* FLEXE_GDMA_H */
