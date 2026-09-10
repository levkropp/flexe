/* Target-described native USB Serial/JTAG endpoint controller. */
#ifndef FLEXE_USB_SERIAL_JTAG_H
#define FLEXE_USB_SERIAL_JTAG_H

#include "memory.h"

#include <stdbool.h>
#include <stddef.h>

typedef struct flexe_usb_serial_jtag flexe_usb_serial_jtag_t;

typedef void (*flexe_usb_serial_jtag_tx_fn)(void *ctx, uint8_t byte);
typedef void (*flexe_usb_serial_jtag_irq_fn)(void *ctx, bool level);

/* Create the V1 register block described by mem's target. Reserved offsets
 * delegate to fallback hooks so missing controller behavior remains visible. */
flexe_usb_serial_jtag_t *flexe_usb_serial_jtag_create(
    xtensa_mem_t *mem, mmio_read_fn fallback_read,
    mmio_write_fn fallback_write, void *fallback_ctx,
    flexe_usb_serial_jtag_irq_fn irq_changed, void *irq_ctx);
void flexe_usb_serial_jtag_destroy(flexe_usb_serial_jtag_t *usb);

/* The virtual host consumes complete IN packets and can inject one OUT packet
 * at a time. The byte callback is optional; transmitted bytes are retained in
 * a bounded capture buffer independently of it. */
void flexe_usb_serial_jtag_set_tx_callback(
    flexe_usb_serial_jtag_t *usb,
    flexe_usb_serial_jtag_tx_fn callback, void *ctx);
size_t flexe_usb_serial_jtag_tx_count(
    const flexe_usb_serial_jtag_t *usb);
const uint8_t *flexe_usb_serial_jtag_tx_buf(
    const flexe_usb_serial_jtag_t *usb);
size_t flexe_usb_serial_jtag_rx_inject(
    flexe_usb_serial_jtag_t *usb, const uint8_t *data, size_t len);
size_t flexe_usb_serial_jtag_rx_pending(
    const flexe_usb_serial_jtag_t *usb);

/* Connection state represents a USB host attached to the native port. Fast
 * mode coalesces periodic SOFs at firmware observation points; host_sof is
 * also exposed for deterministic external event replay. */
void flexe_usb_serial_jtag_set_connected(
    flexe_usb_serial_jtag_t *usb, bool connected);
bool flexe_usb_serial_jtag_connected(
    const flexe_usb_serial_jtag_t *usb);
void flexe_usb_serial_jtag_host_sof(flexe_usb_serial_jtag_t *usb);

#endif /* FLEXE_USB_SERIAL_JTAG_H */
