/* ESP32-S3 native USB Serial/JTAG endpoint tests. */
#include "test_helpers.h"
#include "peripherals.h"

#include <string.h>

#define S3_USB_BASE          0x60038000u
#define USB_EP1              0x000u
#define USB_EP1_CONF         0x004u
#define USB_INT_RAW          0x008u
#define USB_INT_ST           0x00Cu
#define USB_INT_ENA          0x010u
#define USB_INT_CLR          0x014u
#define USB_CONF0            0x018u
#define USB_TEST             0x01Cu
#define USB_JFIFO_ST         0x020u
#define USB_FRAME_NUM        0x024u
#define USB_IN_EP0_ST        0x028u
#define USB_IN_EP1_ST        0x02Cu
#define USB_OUT_EP1_ST       0x03Cu
#define USB_MISC_CONF        0x044u
#define USB_MEM_CONF         0x048u
#define USB_DATE             0x080u

#define USB_WR_DONE          (1u << 0)
#define USB_IN_FREE          (1u << 1)
#define USB_OUT_AVAIL        (1u << 2)
#define USB_INT_SOF          (1u << 1)
#define USB_INT_RX_PACKET    (1u << 2)
#define USB_INT_TX_EMPTY     (1u << 3)
#define USB_INT_IN_TOKEN     (1u << 8)
#define USB_INT_ZERO_PACKET  (1u << 10)
#define USB_INTR_SOURCE      96

static uint32_t usb_read_reg(xtensa_mem_t *mem, uint32_t off)
{
    return mem_read32(mem, S3_USB_BASE + off);
}

static void usb_write_reg(xtensa_mem_t *mem, uint32_t off, uint32_t value)
{
    mem_write32(mem, S3_USB_BASE + off, value);
}

typedef struct {
    uint8_t bytes[256];
    size_t count;
} usb_capture_t;

static void usb_capture_byte(void *ctx, uint8_t byte)
{
    usb_capture_t *capture = ctx;
    if (capture->count < sizeof(capture->bytes))
        capture->bytes[capture->count++] = byte;
}

TEST(usb_serial_jtag_reset_masks_and_reserved_fallback) {
    const flexe_target_desc_t *s3 =
        flexe_target_by_id(FLEXE_TARGET_ESP32S3);
    xtensa_mem_t *mem = mem_create_for_target(s3);
    esp32_periph_t *periph = periph_create(mem);
    ASSERT_TRUE(mem != NULL);
    ASSERT_TRUE(periph != NULL);
    if (!mem || !periph) {
        periph_destroy(periph);
        mem_destroy(mem);
        return;
    }

    ASSERT_TRUE(periph_usb_serial_jtag_connected(periph));
    ASSERT_EQ(usb_read_reg(mem, USB_EP1_CONF), USB_IN_FREE);
    ASSERT_EQ(usb_read_reg(mem, USB_CONF0), 0x00004200u);
    ASSERT_EQ(usb_read_reg(mem, USB_TEST), 0u);
    ASSERT_EQ(usb_read_reg(mem, USB_JFIFO_ST), 0x44u);
    ASSERT_EQ(usb_read_reg(mem, USB_IN_EP0_ST), 1u);
    ASSERT_EQ(usb_read_reg(mem, USB_IN_EP1_ST), 1u);
    ASSERT_EQ(usb_read_reg(mem, USB_OUT_EP1_ST), 0u);
    ASSERT_EQ(usb_read_reg(mem, USB_MISC_CONF), 0u);
    ASSERT_EQ(usb_read_reg(mem, USB_MEM_CONF), 2u);
    ASSERT_EQ(usb_read_reg(mem, USB_DATE), 0x02101200u);

    /* Disconnect before observing INT_RAW so the post-reset virtual host has
     * not yet contributed a coalesced SOF or IN token. */
    periph_usb_serial_jtag_set_connected(periph, false);
    ASSERT_EQ(usb_read_reg(mem, USB_INT_RAW), USB_INT_TX_EMPTY);
    ASSERT_EQ(usb_read_reg(mem, USB_INT_ST), 0u);

    usb_write_reg(mem, USB_CONF0, UINT32_MAX);
    usb_write_reg(mem, USB_TEST, UINT32_MAX);
    usb_write_reg(mem, USB_MISC_CONF, UINT32_MAX);
    usb_write_reg(mem, USB_MEM_CONF, UINT32_MAX);
    usb_write_reg(mem, USB_DATE, 0x12345678u);
    ASSERT_EQ(usb_read_reg(mem, USB_CONF0), 0x0001FFFFu);
    ASSERT_EQ(usb_read_reg(mem, USB_TEST), 0xFu);
    ASSERT_EQ(usb_read_reg(mem, USB_MISC_CONF), 1u);
    ASSERT_EQ(usb_read_reg(mem, USB_MEM_CONF), 3u);
    ASSERT_EQ(usb_read_reg(mem, USB_DATE), 0x12345678u);

    int before = periph_unhandled_count(periph);
    ASSERT_EQ(usb_read_reg(mem, 0x084u), 0u);
    usb_write_reg(mem, 0x084u, 1u);
    ASSERT_EQ(periph_unhandled_count(periph), before + 2);
    ASSERT_EQ(mem_unmapped_count(mem), 0u);

    periph_destroy(periph);
    mem_destroy(mem);
}

TEST(usb_serial_jtag_tx_packets_backpressure_and_interrupts) {
    const flexe_target_desc_t *s3 =
        flexe_target_by_id(FLEXE_TARGET_ESP32S3);
    xtensa_mem_t *mem = mem_create_for_target(s3);
    esp32_periph_t *periph = periph_create(mem);
    usb_capture_t callback = {0};
    ASSERT_TRUE(mem != NULL);
    ASSERT_TRUE(periph != NULL);
    if (!mem || !periph) {
        periph_destroy(periph);
        mem_destroy(mem);
        return;
    }
    periph_set_usb_serial_jtag_callback(
        periph, usb_capture_byte, &callback);

    usb_write_reg(mem, USB_INT_CLR, 0xFFFu);
    usb_write_reg(mem, USB_INT_ENA, USB_INT_TX_EMPTY);
    ASSERT_TRUE(periph_interrupt_pending(periph, USB_INTR_SOURCE));
    usb_write_reg(mem, USB_INT_CLR, USB_INT_TX_EMPTY);
    ASSERT_FALSE(periph_interrupt_pending(periph, USB_INTR_SOURCE));

    usb_write_reg(mem, USB_EP1, 'O');
    usb_write_reg(mem, USB_EP1, 'K');
    ASSERT_EQ(callback.count, 0u);
    ASSERT_EQ(periph_usb_serial_jtag_tx_count(periph), 0u);
    ASSERT_EQ(usb_read_reg(mem, USB_IN_EP1_ST), 1u | (2u << 2));
    usb_write_reg(mem, USB_EP1_CONF, USB_WR_DONE);
    ASSERT_EQ(callback.count, 2u);
    ASSERT_TRUE(memcmp(callback.bytes, "OK", 2u) == 0);
    ASSERT_EQ(periph_usb_serial_jtag_tx_count(periph), 2u);
    ASSERT_TRUE(memcmp(periph_usb_serial_jtag_tx_buf(periph),
                       "OK", 2u) == 0);
    ASSERT_EQ(usb_read_reg(mem, USB_EP1_CONF), USB_IN_FREE);
    ASSERT_TRUE(periph_interrupt_pending(periph, USB_INTR_SOURCE));
    ASSERT_TRUE((usb_read_reg(mem, USB_INT_RAW) &
                 (USB_INT_TX_EMPTY | USB_INT_IN_TOKEN)) ==
                (USB_INT_TX_EMPTY | USB_INT_IN_TOKEN));

    /* A full 64-byte endpoint packet flushes without an explicit WR_DONE. */
    for (unsigned i = 0; i < 64u; i++)
        usb_write_reg(mem, USB_EP1, i);
    ASSERT_EQ(callback.count, 66u);
    ASSERT_EQ(periph_usb_serial_jtag_tx_count(periph), 66u);
    for (unsigned i = 0; i < 64u; i++)
        ASSERT_EQ(callback.bytes[2u + i], i);

    /* A disconnected host holds the completed packet and applies endpoint
     * backpressure until reconnection consumes it. */
    periph_usb_serial_jtag_set_connected(periph, false);
    usb_write_reg(mem, USB_EP1, 'X');
    usb_write_reg(mem, USB_EP1_CONF, USB_WR_DONE);
    ASSERT_EQ(callback.count, 66u);
    ASSERT_EQ(usb_read_reg(mem, USB_EP1_CONF) & USB_IN_FREE, 0u);
    usb_write_reg(mem, USB_EP1, 'Y');
    ASSERT_EQ(callback.count, 66u);
    periph_usb_serial_jtag_set_connected(periph, true);
    ASSERT_EQ(callback.count, 67u);
    ASSERT_EQ(callback.bytes[66], 'X');
    ASSERT_EQ(usb_read_reg(mem, USB_EP1_CONF), USB_IN_FREE);

    periph_destroy(periph);
    mem_destroy(mem);
}

TEST(usb_serial_jtag_host_rx_sof_and_w1c_status) {
    const flexe_target_desc_t *s3 =
        flexe_target_by_id(FLEXE_TARGET_ESP32S3);
    xtensa_mem_t *mem = mem_create_for_target(s3);
    esp32_periph_t *periph = periph_create(mem);
    static const uint8_t packet[] = { 0x11u, 0x22u, 0x33u };
    ASSERT_TRUE(mem != NULL);
    ASSERT_TRUE(periph != NULL);
    if (!mem || !periph) {
        periph_destroy(periph);
        mem_destroy(mem);
        return;
    }

    usb_write_reg(mem, USB_INT_CLR, 0xFFFu);
    usb_write_reg(mem, USB_INT_ENA, USB_INT_RX_PACKET);
    ASSERT_EQ(periph_usb_serial_jtag_rx_inject(
                  periph, packet, sizeof(packet)), sizeof(packet));
    ASSERT_EQ(periph_usb_serial_jtag_rx_pending(periph), 3u);
    ASSERT_EQ(usb_read_reg(mem, USB_EP1_CONF) & USB_OUT_AVAIL,
              USB_OUT_AVAIL);
    ASSERT_EQ((usb_read_reg(mem, USB_OUT_EP1_ST) >> 16) & 0x7Fu, 3u);
    ASSERT_EQ((usb_read_reg(mem, USB_OUT_EP1_ST) >> 2) & 0x7Fu, 5u);
    ASSERT_TRUE(periph_interrupt_pending(periph, USB_INTR_SOURCE));
    ASSERT_EQ(usb_read_reg(mem, USB_INT_ST) & USB_INT_RX_PACKET,
              USB_INT_RX_PACKET);

    ASSERT_EQ(usb_read_reg(mem, USB_EP1), 0x11u);
    ASSERT_EQ(periph_usb_serial_jtag_rx_pending(periph), 2u);
    ASSERT_EQ((usb_read_reg(mem, USB_OUT_EP1_ST) >> 9) & 0x7Fu, 3u);
    ASSERT_EQ(usb_read_reg(mem, USB_EP1), 0x22u);
    ASSERT_EQ(usb_read_reg(mem, USB_EP1), 0x33u);
    ASSERT_EQ(periph_usb_serial_jtag_rx_pending(periph), 0u);
    ASSERT_EQ(usb_read_reg(mem, USB_EP1_CONF) & USB_OUT_AVAIL, 0u);
    ASSERT_TRUE(periph_interrupt_pending(periph, USB_INTR_SOURCE));
    usb_write_reg(mem, USB_INT_CLR, USB_INT_RX_PACKET);
    ASSERT_FALSE(periph_interrupt_pending(periph, USB_INTR_SOURCE));

    uint8_t long_packet[80];
    memset(long_packet, 0xA5, sizeof(long_packet));
    ASSERT_EQ(periph_usb_serial_jtag_rx_inject(
                  periph, long_packet, sizeof(long_packet)), 64u);
    ASSERT_EQ(periph_usb_serial_jtag_rx_inject(
                  periph, packet, sizeof(packet)), 0u);
    for (unsigned i = 0; i < 64u; i++)
        ASSERT_EQ(usb_read_reg(mem, USB_EP1), 0xA5u);

    usb_write_reg(mem, USB_INT_CLR, 0xFFFu);
    ASSERT_EQ(periph_usb_serial_jtag_rx_inject(periph, NULL, 0u), 0u);
    ASSERT_TRUE((usb_read_reg(mem, USB_INT_RAW) &
                 (USB_INT_RX_PACKET | USB_INT_ZERO_PACKET)) ==
                (USB_INT_RX_PACKET | USB_INT_ZERO_PACKET));

    usb_write_reg(mem, USB_INT_CLR, USB_INT_SOF);
    uint32_t frame_before = usb_read_reg(mem, USB_FRAME_NUM);
    ASSERT_TRUE((usb_read_reg(mem, USB_INT_RAW) & USB_INT_SOF) != 0u);
    ASSERT_TRUE(usb_read_reg(mem, USB_FRAME_NUM) != frame_before);
    periph_usb_serial_jtag_set_connected(periph, false);
    usb_write_reg(mem, USB_INT_CLR, USB_INT_SOF);
    periph_usb_serial_jtag_host_sof(periph);
    ASSERT_EQ(usb_read_reg(mem, USB_INT_RAW) & USB_INT_SOF, 0u);
    ASSERT_EQ(periph_usb_serial_jtag_rx_inject(
                  periph, packet, sizeof(packet)), 0u);

    periph_destroy(periph);
    mem_destroy(mem);
}

void run_usb_serial_jtag_tests(void) {
    TEST_SUITE("USB Serial/JTAG");
    RUN_TEST(usb_serial_jtag_reset_masks_and_reserved_fallback);
    RUN_TEST(usb_serial_jtag_tx_packets_backpressure_and_interrupts);
    RUN_TEST(usb_serial_jtag_host_rx_sof_and_w1c_status);
}
