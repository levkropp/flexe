#include "usb_serial_jtag.h"

#include <stdlib.h>
#include <string.h>

/* ESP32-S2/S3 USB Serial/JTAG V1 register layout. */
#define USB_EP1_OFF                 0x000u
#define USB_EP1_CONF_OFF            0x004u
#define USB_INT_RAW_OFF             0x008u
#define USB_INT_ST_OFF              0x00Cu
#define USB_INT_ENA_OFF             0x010u
#define USB_INT_CLR_OFF             0x014u
#define USB_CONF0_OFF               0x018u
#define USB_TEST_OFF                0x01Cu
#define USB_JFIFO_ST_OFF            0x020u
#define USB_FRAME_NUM_OFF           0x024u
#define USB_IN_EP0_ST_OFF           0x028u
#define USB_IN_EP1_ST_OFF           0x02Cu
#define USB_IN_EP2_ST_OFF           0x030u
#define USB_IN_EP3_ST_OFF           0x034u
#define USB_OUT_EP0_ST_OFF          0x038u
#define USB_OUT_EP1_ST_OFF          0x03Cu
#define USB_OUT_EP2_ST_OFF          0x040u
#define USB_MISC_CONF_OFF           0x044u
#define USB_MEM_CONF_OFF            0x048u
#define USB_DATE_OFF                0x080u

#define USB_EP_CONF_WR_DONE         (1u << 0)
#define USB_EP_CONF_IN_FREE         (1u << 1)
#define USB_EP_CONF_OUT_AVAIL       (1u << 2)

#define USB_INT_SOF                 (1u << 1)
#define USB_INT_OUT_RECV_PKT        (1u << 2)
#define USB_INT_IN_EMPTY            (1u << 3)
#define USB_INT_IN_TOKEN_EP1        (1u << 8)
#define USB_INT_BUS_RESET           (1u << 9)
#define USB_INT_OUT_EP1_ZERO        (1u << 10)

#define USB_CONF0_PAD_ENABLE        (1u << 14)
#define USB_MEM_POWER_DOWN          (1u << 0)

#define USB_JFIFO_IDLE              ((1u << 6) | (1u << 2))
#define USB_IN_EP_IDLE              1u
#define USB_ENDPOINT_MAX            128u
#define USB_CAPTURE_SIZE            4096u

struct flexe_usb_serial_jtag {
    xtensa_mem_t *mem;
    const flexe_target_desc_t *target;
    mmio_read_fn fallback_read;
    mmio_write_fn fallback_write;
    void *fallback_ctx;
    flexe_usb_serial_jtag_irq_fn irq_changed;
    void *irq_ctx;
    flexe_usb_serial_jtag_tx_fn tx_callback;
    void *tx_ctx;

    uint8_t tx_fifo[USB_ENDPOINT_MAX];
    uint8_t rx_fifo[USB_ENDPOINT_MAX];
    uint8_t tx_capture[USB_CAPTURE_SIZE];
    size_t tx_len;
    size_t rx_len;
    size_t rx_pos;
    size_t tx_capture_len;
    uint32_t int_raw;
    uint32_t int_enable;
    uint32_t conf0;
    uint32_t test;
    uint32_t misc_conf;
    uint32_t mem_conf;
    uint32_t date;
    uint16_t frame_number;
    bool tx_waiting_for_host;
    bool connected;
    bool irq_level;
};

static bool usb_geometry_valid(const flexe_target_desc_t *target)
{
    if (!target || !(target->capabilities &
                     FLEXE_TARGET_CAP_USB_SERIAL_JTAG_V1))
        return false;
    const flexe_usb_serial_jtag_desc_t *desc =
        &target->usb_serial_jtag;
    if ((desc->base & 0xFFFu) != 0u ||
        desc->register_size < USB_DATE_OFF + sizeof(uint32_t) ||
        desc->base < target->peripheral_start ||
        desc->base >= target->peripheral_end ||
        desc->register_size > target->peripheral_end - desc->base ||
        desc->endpoint_size == 0u ||
        desc->endpoint_size > USB_ENDPOINT_MAX ||
        desc->interrupt_valid_mask == 0u ||
        (desc->interrupt_raw_reset & ~desc->interrupt_valid_mask) != 0u ||
        (desc->conf0_reset & ~desc->conf0_writable_mask) != 0u ||
        (desc->test_reset & ~desc->test_writable_mask) != 0u ||
        (desc->misc_conf_reset & ~desc->misc_conf_writable_mask) != 0u ||
        (desc->mem_conf_reset & ~desc->mem_conf_writable_mask) != 0u ||
        (desc->date_reset & ~desc->date_writable_mask) != 0u)
        return false;
    if ((target->capabilities & FLEXE_TARGET_CAP_INTERRUPT_MATRIX_V1) &&
        desc->interrupt_source >= target->interrupt_matrix.source_count)
        return false;
    return true;
}

static bool usb_link_active(const flexe_usb_serial_jtag_t *usb)
{
    return usb->connected && (usb->conf0 & USB_CONF0_PAD_ENABLE) != 0u &&
           (usb->mem_conf & USB_MEM_POWER_DOWN) == 0u;
}

static void usb_update_irq(flexe_usb_serial_jtag_t *usb)
{
    const flexe_usb_serial_jtag_desc_t *desc =
        &usb->target->usb_serial_jtag;
    bool level = (usb->int_raw & usb->int_enable &
                  desc->interrupt_valid_mask) != 0u;
    if (level == usb->irq_level) return;
    usb->irq_level = level;
    if (usb->irq_changed) usb->irq_changed(usb->irq_ctx, level);
}

static void usb_raise(flexe_usb_serial_jtag_t *usb, uint32_t mask)
{
    usb->int_raw |= mask &
        usb->target->usb_serial_jtag.interrupt_valid_mask;
    usb_update_irq(usb);
}

static void usb_consume_tx_packet(flexe_usb_serial_jtag_t *usb)
{
    if (!usb->tx_waiting_for_host || !usb_link_active(usb)) return;
    size_t room = USB_CAPTURE_SIZE - usb->tx_capture_len;
    size_t captured = usb->tx_len < room ? usb->tx_len : room;
    if (captured != 0u) {
        memcpy(usb->tx_capture + usb->tx_capture_len,
               usb->tx_fifo, captured);
        usb->tx_capture_len += captured;
    }
    if (usb->tx_callback) {
        for (size_t i = 0; i < usb->tx_len; i++)
            usb->tx_callback(usb->tx_ctx, usb->tx_fifo[i]);
    }
    usb->tx_len = 0u;
    usb->tx_waiting_for_host = false;
    usb_raise(usb, USB_INT_IN_EMPTY | USB_INT_IN_TOKEN_EP1);
}

static void usb_flush_tx(flexe_usb_serial_jtag_t *usb)
{
    if (usb->tx_waiting_for_host) return;
    usb->tx_waiting_for_host = true;
    usb_consume_tx_packet(usb);
}

static void usb_observe_connected_host(flexe_usb_serial_jtag_t *usb)
{
    if (!usb_link_active(usb)) return;
    /* Fast functional mode has no USB bit clock yet. Coalesce the periodic
     * 1 ms host SOFs into each firmware observation so IDF's connection
     * monitor sees a live enumerated host without inventing line timing. */
    usb->frame_number = (uint16_t)((usb->frame_number + 1u) & 0x7FFu);
    uint32_t events = USB_INT_SOF;
    if (!usb->tx_waiting_for_host &&
        usb->tx_len < usb->target->usb_serial_jtag.endpoint_size)
        events |= USB_INT_IN_EMPTY | USB_INT_IN_TOKEN_EP1;
    usb_raise(usb, events);
}

static uint32_t usb_ep1_conf(const flexe_usb_serial_jtag_t *usb)
{
    uint32_t value = 0u;
    size_t endpoint_size = usb->target->usb_serial_jtag.endpoint_size;
    if (!usb->tx_waiting_for_host && usb->tx_len < endpoint_size &&
        (usb->mem_conf & USB_MEM_POWER_DOWN) == 0u)
        value |= USB_EP_CONF_IN_FREE;
    if (usb->rx_pos < usb->rx_len) value |= USB_EP_CONF_OUT_AVAIL;
    return value;
}

static uint32_t usb_in_ep1_status(const flexe_usb_serial_jtag_t *usb)
{
    uint32_t write = (uint32_t)usb->tx_len & 0x7Fu;
    return USB_IN_EP_IDLE | (write << 2);
}

static uint32_t usb_out_ep1_status(const flexe_usb_serial_jtag_t *usb)
{
    if (usb->rx_len == 0u) return 0u;
    uint32_t read = ((uint32_t)usb->rx_pos + 2u) & 0x7Fu;
    uint32_t write = ((uint32_t)usb->rx_len + 2u) & 0x7Fu;
    uint32_t count = (uint32_t)usb->rx_len & 0x7Fu;
    return (count << 16) | (read << 9) | (write << 2);
}

static uint32_t usb_read(void *ctx, uint32_t addr)
{
    flexe_usb_serial_jtag_t *usb = ctx;
    const flexe_usb_serial_jtag_desc_t *desc =
        &usb->target->usb_serial_jtag;
    uint32_t off = addr - desc->base;
    if ((off & 3u) != 0u) goto fallback;

    switch (off) {
    case USB_EP1_OFF: {
        if (usb->rx_pos >= usb->rx_len) return 0u;
        uint8_t byte = usb->rx_fifo[usb->rx_pos++];
        if (usb->rx_pos == usb->rx_len) {
            usb->rx_pos = 0u;
            usb->rx_len = 0u;
        }
        return byte;
    }
    case USB_EP1_CONF_OFF:
        return usb_ep1_conf(usb);
    case USB_INT_RAW_OFF:
        usb_observe_connected_host(usb);
        return usb->int_raw;
    case USB_INT_ST_OFF:
        usb_observe_connected_host(usb);
        return usb->int_raw & usb->int_enable;
    case USB_INT_ENA_OFF:
        return usb->int_enable;
    case USB_INT_CLR_OFF:
        return 0u;
    case USB_CONF0_OFF:
        return usb->conf0;
    case USB_TEST_OFF:
        return usb->test;
    case USB_JFIFO_ST_OFF:
        return USB_JFIFO_IDLE;
    case USB_FRAME_NUM_OFF:
        usb_observe_connected_host(usb);
        return usb->frame_number;
    case USB_IN_EP0_ST_OFF:
    case USB_IN_EP2_ST_OFF:
    case USB_IN_EP3_ST_OFF:
        return USB_IN_EP_IDLE;
    case USB_IN_EP1_ST_OFF:
        return usb_in_ep1_status(usb);
    case USB_OUT_EP0_ST_OFF:
    case USB_OUT_EP2_ST_OFF:
        return 0u;
    case USB_OUT_EP1_ST_OFF:
        return usb_out_ep1_status(usb);
    case USB_MISC_CONF_OFF:
        return usb->misc_conf;
    case USB_MEM_CONF_OFF:
        return usb->mem_conf;
    case USB_DATE_OFF:
        return usb->date;
    default:
        break;
    }

fallback:
    return usb->fallback_read ?
        usb->fallback_read(usb->fallback_ctx, addr) : 0u;
}

static void usb_write(void *ctx, uint32_t addr, uint32_t value)
{
    flexe_usb_serial_jtag_t *usb = ctx;
    const flexe_usb_serial_jtag_desc_t *desc =
        &usb->target->usb_serial_jtag;
    uint32_t off = addr - desc->base;
    if ((off & 3u) != 0u) goto fallback;

    switch (off) {
    case USB_EP1_OFF:
        if (!usb->tx_waiting_for_host &&
            usb->tx_len < desc->endpoint_size &&
            (usb->mem_conf & USB_MEM_POWER_DOWN) == 0u) {
            usb->tx_fifo[usb->tx_len++] = (uint8_t)value;
            if (usb->tx_len == desc->endpoint_size) usb_flush_tx(usb);
        }
        return;
    case USB_EP1_CONF_OFF:
        if (value & USB_EP_CONF_WR_DONE) usb_flush_tx(usb);
        return;
    case USB_INT_RAW_OFF:
    case USB_INT_CLR_OFF:
        usb->int_raw &= ~(value & desc->interrupt_valid_mask);
        usb_update_irq(usb);
        return;
    case USB_INT_ST_OFF:
        return; /* Read-only masked status. */
    case USB_INT_ENA_OFF:
        usb->int_enable = value & desc->interrupt_valid_mask;
        if ((usb->int_enable & USB_INT_IN_EMPTY) != 0u &&
            (usb_ep1_conf(usb) & USB_EP_CONF_IN_FREE) != 0u)
            usb->int_raw |= USB_INT_IN_EMPTY;
        usb_update_irq(usb);
        return;
    case USB_CONF0_OFF:
        usb->conf0 = value & desc->conf0_writable_mask;
        usb_consume_tx_packet(usb);
        return;
    case USB_TEST_OFF:
        usb->test = value & desc->test_writable_mask;
        return;
    case USB_JFIFO_ST_OFF:
        return; /* JTAG FIFO reset triggers; FIFOs are not exposed yet. */
    case USB_FRAME_NUM_OFF:
    case USB_IN_EP0_ST_OFF:
    case USB_IN_EP1_ST_OFF:
    case USB_IN_EP2_ST_OFF:
    case USB_IN_EP3_ST_OFF:
    case USB_OUT_EP0_ST_OFF:
    case USB_OUT_EP1_ST_OFF:
    case USB_OUT_EP2_ST_OFF:
        return; /* Hardware-owned status. */
    case USB_MISC_CONF_OFF:
        usb->misc_conf = value & desc->misc_conf_writable_mask;
        return;
    case USB_MEM_CONF_OFF:
        usb->mem_conf = value & desc->mem_conf_writable_mask;
        usb_consume_tx_packet(usb);
        return;
    case USB_DATE_OFF:
        usb->date = value & desc->date_writable_mask;
        return;
    default:
        break;
    }

fallback:
    if (usb->fallback_write)
        usb->fallback_write(usb->fallback_ctx, addr, value);
}

flexe_usb_serial_jtag_t *flexe_usb_serial_jtag_create(
    xtensa_mem_t *mem, mmio_read_fn fallback_read,
    mmio_write_fn fallback_write, void *fallback_ctx,
    flexe_usb_serial_jtag_irq_fn irq_changed, void *irq_ctx)
{
    if (!mem) return NULL;
    const flexe_target_desc_t *target = mem_target(mem);
    if (!usb_geometry_valid(target)) return NULL;
    const flexe_usb_serial_jtag_desc_t *desc =
        &target->usb_serial_jtag;

    flexe_usb_serial_jtag_t *usb = calloc(1, sizeof(*usb));
    if (!usb) return NULL;
    usb->mem = mem;
    usb->target = target;
    usb->fallback_read = fallback_read;
    usb->fallback_write = fallback_write;
    usb->fallback_ctx = fallback_ctx;
    usb->irq_changed = irq_changed;
    usb->irq_ctx = irq_ctx;
    usb->int_raw = desc->interrupt_raw_reset;
    usb->conf0 = desc->conf0_reset;
    usb->test = desc->test_reset;
    usb->misc_conf = desc->misc_conf_reset;
    usb->mem_conf = desc->mem_conf_reset;
    usb->date = desc->date_reset;
    /* A capture sink is a deterministic virtual USB host. Frontends may
     * explicitly disconnect it when testing cable/power behavior. */
    usb->connected = true;

    if (mem_register_mmio_range(mem, desc->base, desc->register_size,
                                usb_read, usb_write, usb) != 0) {
        free(usb);
        return NULL;
    }
    return usb;
}

void flexe_usb_serial_jtag_destroy(flexe_usb_serial_jtag_t *usb)
{
    if (!usb) return;
    if (usb->irq_level && usb->irq_changed)
        usb->irq_changed(usb->irq_ctx, false);
    const flexe_usb_serial_jtag_desc_t *desc =
        &usb->target->usb_serial_jtag;
    (void)mem_register_mmio_range(usb->mem, desc->base,
                                  desc->register_size, NULL, NULL, NULL);
    free(usb);
}

void flexe_usb_serial_jtag_set_tx_callback(
    flexe_usb_serial_jtag_t *usb,
    flexe_usb_serial_jtag_tx_fn callback, void *ctx)
{
    if (!usb) return;
    usb->tx_callback = callback;
    usb->tx_ctx = callback ? ctx : NULL;
}

size_t flexe_usb_serial_jtag_tx_count(
    const flexe_usb_serial_jtag_t *usb)
{
    return usb ? usb->tx_capture_len : 0u;
}

const uint8_t *flexe_usb_serial_jtag_tx_buf(
    const flexe_usb_serial_jtag_t *usb)
{
    return usb ? usb->tx_capture : NULL;
}

size_t flexe_usb_serial_jtag_rx_inject(
    flexe_usb_serial_jtag_t *usb, const uint8_t *data, size_t len)
{
    if (!usb || (!data && len != 0u) || !usb_link_active(usb) ||
        usb->rx_len != 0u)
        return 0u;
    size_t endpoint_size = usb->target->usb_serial_jtag.endpoint_size;
    size_t accepted = len < endpoint_size ? len : endpoint_size;
    if (accepted != 0u) memcpy(usb->rx_fifo, data, accepted);
    usb->rx_pos = 0u;
    usb->rx_len = accepted;
    uint32_t events = USB_INT_OUT_RECV_PKT;
    if (accepted == 0u) events |= USB_INT_OUT_EP1_ZERO;
    usb_raise(usb, events);
    return accepted;
}

size_t flexe_usb_serial_jtag_rx_pending(
    const flexe_usb_serial_jtag_t *usb)
{
    return usb && usb->rx_len > usb->rx_pos ?
        usb->rx_len - usb->rx_pos : 0u;
}

void flexe_usb_serial_jtag_set_connected(
    flexe_usb_serial_jtag_t *usb, bool connected)
{
    if (!usb || usb->connected == connected) return;
    usb->connected = connected;
    if (connected) {
        usb_observe_connected_host(usb);
        usb_consume_tx_packet(usb);
    }
}

bool flexe_usb_serial_jtag_connected(
    const flexe_usb_serial_jtag_t *usb)
{
    return usb ? usb->connected : false;
}

void flexe_usb_serial_jtag_host_sof(flexe_usb_serial_jtag_t *usb)
{
    if (usb) usb_observe_connected_host(usb);
}
