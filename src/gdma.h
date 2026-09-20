/* Target-described ESP32-family general-purpose DMA v1 controller. */
#ifndef FLEXE_GDMA_H
#define FLEXE_GDMA_H

#include "memory.h"
#include <stdbool.h>

typedef struct flexe_gdma flexe_gdma_t;

typedef void (*flexe_gdma_activity_fn)(void *ctx);

/* One descriptor completed by a streaming peripheral. Unlike the existing
 * fixed-length helpers below, a stream keeps following a non-null `next`
 * pointer (including a circular list) and raises EOF independently for each
 * descriptor. */
typedef struct {
    uint32_t descriptor_address;
    uint32_t buffer_address;
    size_t length;
    bool eof;
    bool chain_complete;
} flexe_gdma_descriptor_t;

/* The controller presents one level-sensitive interrupt per RX/TX channel.
 * `receive` selects the RX half; `level` is INT_RAW & INT_ENA != 0. */
typedef void (*flexe_gdma_irq_changed_fn)(void *ctx, unsigned channel,
                                          bool receive, bool level);

flexe_gdma_t *flexe_gdma_create(
    xtensa_mem_t *mem, mmio_read_fn fallback_read,
    mmio_write_fn fallback_write, void *fallback_ctx,
    flexe_gdma_irq_changed_fn irq_changed, void *irq_ctx);
void flexe_gdma_destroy(flexe_gdma_t *gdma);

/* Notify one composed peripheral when guest programming changes a channel's
 * route or running state. This is a scheduling hint; the peripheral must
 * still query its own trigger ID before doing work. */
void flexe_gdma_set_activity_handler(flexe_gdma_t *gdma,
                                     flexe_gdma_activity_fn changed,
                                     void *ctx);

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

/* Descriptor-at-a-time transport for continuous devices. `pending_length`
 * reports the next TX descriptor's valid byte count or the next RX
 * descriptor's capacity without mutating channel state. Completion follows
 * the descriptor's next pointer and parks only at the end of a finite chain
 * or on an architectural error. */
bool flexe_gdma_pending_length(const flexe_gdma_t *gdma,
                               uint8_t peripheral_id, bool receive,
                               size_t *length);
int flexe_gdma_read_tx_descriptor(flexe_gdma_t *gdma,
                                  uint8_t peripheral_id,
                                  uint8_t *data, size_t capacity,
                                  flexe_gdma_descriptor_t *completed);
int flexe_gdma_write_rx_descriptor(flexe_gdma_t *gdma,
                                   uint8_t peripheral_id,
                                   const uint8_t *data, size_t length,
                                   flexe_gdma_descriptor_t *completed);
/* Complete one RX descriptor while preserving the peripheral's independent
 * EOF boundary. Streaming peripherals may span one EOF interval across
 * several descriptors; the compatibility wrapper above treats each
 * descriptor as an EOF, matching its historical contract. */
int flexe_gdma_write_rx_descriptor_eof(
    flexe_gdma_t *gdma, uint8_t peripheral_id,
    const uint8_t *data, size_t length, bool peripheral_eof,
    flexe_gdma_descriptor_t *completed);

#endif /* FLEXE_GDMA_H */
