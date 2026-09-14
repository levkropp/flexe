/* Target-described ESP32-family general-purpose DMA v1 controller. */
#ifndef FLEXE_GDMA_H
#define FLEXE_GDMA_H

#include "memory.h"
#include <stdbool.h>

typedef struct flexe_gdma flexe_gdma_t;

/* The controller presents one level-sensitive interrupt per RX/TX channel.
 * `receive` selects the RX half; `level` is INT_RAW & INT_ENA != 0. */
typedef void (*flexe_gdma_irq_changed_fn)(void *ctx, unsigned channel,
                                          bool receive, bool level);

flexe_gdma_t *flexe_gdma_create(
    xtensa_mem_t *mem, mmio_read_fn fallback_read,
    mmio_write_fn fallback_write, void *fallback_ctx,
    flexe_gdma_irq_changed_fn irq_changed, void *irq_ctx);
void flexe_gdma_destroy(flexe_gdma_t *gdma);

/* Whether a started TX descriptor stream is routed to this peripheral.
 * A peripheral enable bit may remain set after the driver intentionally
 * omits a TX stream for a receive-only transaction. */
bool flexe_gdma_tx_active(const flexe_gdma_t *gdma, uint8_t peripheral_id);

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
