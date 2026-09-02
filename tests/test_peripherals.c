#include "peripherals.h"
#include "sandbox_events.h"
#include "sdcard_stubs.h"
#include "spi_display.h"
#include <fcntl.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

/* ===== MMIO callback framework ===== */

static uint32_t test_hook_read_val;
static uint32_t test_hook_read(void *ctx, uint32_t addr) {
    (void)ctx; (void)addr;
    return test_hook_read_val;
}

static uint32_t test_hook_write_addr;
static uint32_t test_hook_write_val;
static void test_hook_write(void *ctx, uint32_t addr, uint32_t val) {
    (void)ctx;
    test_hook_write_addr = addr;
    test_hook_write_val = val;
}

TEST(mmio_hook_read32) {
    xtensa_mem_t *mem = mem_create();
    /* Page 0 = 0x3FF00000 */
    test_hook_read_val = 0xDEADBEEF;
    mem_register_mmio(mem, 0, test_hook_read, NULL, NULL);
    ASSERT_EQ(mem_read32(mem, 0x3FF00000), 0xDEADBEEF);
    ASSERT_EQ(mem_read32(mem, 0x3FF00004), 0xDEADBEEF);
    mem_destroy(mem);
}

TEST(mmio_hook_write32) {
    xtensa_mem_t *mem = mem_create();
    test_hook_write_addr = 0;
    test_hook_write_val = 0;
    mem_register_mmio(mem, 0, NULL, test_hook_write, NULL);
    mem_write32(mem, 0x3FF00010, 0x42);
    ASSERT_EQ(test_hook_write_addr, 0x3FF00010);
    ASSERT_EQ(test_hook_write_val, 0x42);
    mem_destroy(mem);
}

TEST(mmio_range_registration) {
    xtensa_mem_t *mem = mem_create();
    test_hook_read_val = 0xCAFE;
    /* Register 3 pages starting at 0x3FF10000 (page 16) */
    mem_register_mmio_range(mem, 0x3FF10000, 3 * 4096,
                            test_hook_read, NULL, NULL);
    ASSERT_EQ(mem_read32(mem, 0x3FF10000), 0xCAFE);  /* page 16 */
    ASSERT_EQ(mem_read32(mem, 0x3FF11000), 0xCAFE);  /* page 17 */
    ASSERT_EQ(mem_read32(mem, 0x3FF12000), 0xCAFE);  /* page 18 */
    ASSERT_EQ(mem_read32(mem, 0x3FF13000), 0);        /* page 19: not registered */
    mem_destroy(mem);
}

TEST(mmio_no_handler_returns_zero) {
    /* Bare mem with no handlers: existing behavior preserved */
    xtensa_mem_t *mem = mem_create();
    ASSERT_EQ(mem_read32(mem, 0x3FF00000), 0);
    ASSERT_EQ(mem_read32(mem, 0x3FF40000), 0);
    mem_write32(mem, 0x3FF00000, 0x1234); /* should not crash */
    mem_destroy(mem);
}

/* ===== ESP32 peripheral stubs ===== */

TEST(uart_tx_capture) {
    xtensa_mem_t *mem = mem_create();
    esp32_periph_t *p = periph_create(mem);
    /* Write bytes to UART0 FIFO */
    mem_write32(mem, 0x3FF40000, 'H');
    mem_write32(mem, 0x3FF40000, 'i');
    ASSERT_EQ(periph_uart_tx_count(p), 2);
    const uint8_t *buf = periph_uart_tx_buf(p);
    ASSERT_EQ(buf[0], 'H');
    ASSERT_EQ(buf[1], 'i');
    periph_destroy(p);
    mem_destroy(mem);
}

#define TEST_UHCI0_BASE 0x3FF54000u
#define TEST_UHCI1_BASE 0x3FF4C000u
#define TEST_UHCI_CONF0_RESET 0x00370100u
#define TEST_UHCI_CONF1_RESET 0x00000033u
#define TEST_UHCI_LINK_START (1u << 29)
#define TEST_UHCI_LINK_PARK (1u << 31)
#define TEST_UHCI_DESC_EOF (1u << 30)
#define TEST_UHCI_DESC_OWNER (1u << 31)

#define TEST_HINF_BASE 0x3FF4B000u
#define TEST_SLCHOST_BASE 0x3FF55000u
#define TEST_SLC_BASE 0x3FF58000u
#define TEST_SLC_LINK_START (1u << 29)
#define TEST_SLC_LINK_PARK (1u << 31)
#define TEST_SLC_DESC_EOF (1u << 30)
#define TEST_SLC_DESC_OWNER (1u << 31)
#define TEST_SLC_INT_TX_OVF (1u << 11)
#define TEST_SLC_INT_TOKEN1_EMPTY (1u << 13)
#define TEST_SLC_INT_TX_DONE (1u << 14)
#define TEST_SLC_INT_TX_SUC_EOF (1u << 15)
#define TEST_SLC_INT_RX_DONE (1u << 16)
#define TEST_SLC_INT_RX_EOF (1u << 17)
#define TEST_SLC_INT_TOHOST (1u << 18)
#define TEST_SLC_INT_TX_DSCR_ERR (1u << 19)
#define TEST_SLC_INT_RX_DSCR_ERR (1u << 20)

#define TEST_SDMMC_BASE 0x3FF68000u
#define TEST_SDMMC_CMD_RESP (1u << 6)
#define TEST_SDMMC_CMD_DATA (1u << 9)
#define TEST_SDMMC_CMD_WRITE (1u << 10)
#define TEST_SDMMC_CMD_START (1u << 31)
#define TEST_SDMMC_CTRL_INT (1u << 4)
#define TEST_SDMMC_CTRL_DMA (1u << 5)
#define TEST_SDMMC_CTRL_IDMAC (1u << 25)
#define TEST_SDMMC_DESC_LAST (1u << 2)
#define TEST_SDMMC_DESC_FIRST (1u << 3)
#define TEST_SDMMC_DESC_CHAINED (1u << 4)
#define TEST_SDMMC_DESC_OWNER (1u << 31)

typedef struct {
    uint8_t block[8][512];
    unsigned reads;
    unsigned writes;
} test_sdmmc_card_t;

static int test_sdmmc_read_blocks(void *ctx, uint32_t first,
                                  uint8_t *data, size_t count) {
    test_sdmmc_card_t *card = ctx;
    if (!card || first > 8u || count > 8u - first) return -1;
    memcpy(data, card->block[first], count * 512u);
    card->reads++;
    return 0;
}

static int test_sdmmc_write_blocks(void *ctx, uint32_t first,
                                   const uint8_t *data, size_t count) {
    test_sdmmc_card_t *card = ctx;
    if (!card || first > 8u || count > 8u - first) return -1;
    memcpy(card->block[first], data, count * 512u);
    card->writes++;
    return 0;
}

static void test_sdmmc_desc(xtensa_mem_t *mem, uint32_t desc,
                            uint32_t ctrl, uint32_t size,
                            uint32_t buffer, uint32_t next) {
    mem_write32(mem, desc, ctrl);
    mem_write32(mem, desc + 4u, size & 0x1FFFu);
    mem_write32(mem, desc + 8u, buffer);
    mem_write32(mem, desc + 12u, next);
}

static void test_sdmmc_command(xtensa_mem_t *mem, unsigned command,
                               uint32_t argument, uint32_t flags,
                               unsigned slot) {
    mem_write32(mem, TEST_SDMMC_BASE + 0x28u, argument);
    mem_write32(mem, TEST_SDMMC_BASE + 0x2Cu,
                TEST_SDMMC_CMD_START | flags | command |
                ((uint32_t)slot << 16));
}

static void test_slc_desc(xtensa_mem_t *mem, uint32_t desc,
                          uint32_t buffer, uint16_t size, uint16_t length,
                          bool eof, uint32_t next) {
    uint32_t control = ((uint32_t)size & 0xFFFu) |
                       (((uint32_t)length & 0xFFFu) << 12u) |
                       TEST_SLC_DESC_OWNER;
    if (eof) control |= TEST_SLC_DESC_EOF;
    mem_write32(mem, desc, control);
    mem_write32(mem, desc + 4u, buffer);
    mem_write32(mem, desc + 8u, next);
}

static uint32_t test_slchost_shared_addr(unsigned position) {
    uint32_t address = TEST_SLCHOST_BASE + 0x6Cu + position;
    if (position > 23u) address += 4u;
    if (position > 31u) address += 12u;
    return address;
}

static void test_spi_dma_desc(xtensa_mem_t *mem, uint32_t desc,
                              uint32_t buf, uint16_t size, uint16_t len,
                              int eof, uint32_t next);

TEST(uhci_reset_register_file_dual_instance_and_dport) {
    xtensa_mem_t *mem = mem_create();
    esp32_periph_t *p = periph_create(mem);

    for (unsigned port = 0; port < 2u; port++) {
        uint32_t base = port == 0u ? TEST_UHCI0_BASE : TEST_UHCI1_BASE;
        ASSERT_EQ(mem_read32(mem, base + 0x00u), TEST_UHCI_CONF0_RESET);
        ASSERT_EQ(mem_read32(mem, base + 0x14u), 2u);
        ASSERT_EQ(mem_read32(mem, base + 0x1Cu), 2u);
        ASSERT_EQ(mem_read32(mem, base + 0x24u), TEST_UHCI_LINK_PARK);
        ASSERT_EQ(mem_read32(mem, base + 0x28u),
                  TEST_UHCI_LINK_PARK | (1u << 20));
        ASSERT_EQ(mem_read32(mem, base + 0x2Cu), TEST_UHCI_CONF1_RESET);
        ASSERT_EQ(mem_read32(mem, base + 0x64u), 0x33u);
        ASSERT_EQ(mem_read32(mem, base + 0x68u), 0x00810810u);
        ASSERT_EQ(mem_read32(mem, base + 0xB0u), 0x00DCDBC0u);
        ASSERT_EQ(mem_read32(mem, base + 0xB4u), 0x00DDDBDBu);
        ASSERT_EQ(mem_read32(mem, base + 0xB8u), 0x00DEDB11u);
        ASSERT_EQ(mem_read32(mem, base + 0xBCu), 0x00DFDB13u);
        ASSERT_EQ(mem_read32(mem, base + 0xC0u), 0x80u);
        ASSERT_EQ(mem_read32(mem, base + 0xFCu), 0x16041001u);
    }

    mem_write32(mem, TEST_UHCI0_BASE + 0x64u, 0xA5u);
    mem_write32(mem, TEST_UHCI1_BASE + 0x64u, 0x5Au);
    mem_write32(mem, 0x3FF000C0u, (1u << 8) | (1u << 12));
    ASSERT_EQ(mem_read32(mem, TEST_UHCI0_BASE + 0x64u), 0xA5u);
    ASSERT_EQ(mem_read32(mem, TEST_UHCI1_BASE + 0x64u), 0x5Au);

    /* The two DPORT reset domains are independent. */
    mem_write32(mem, 0x3FF000C4u, 1u << 12);
    ASSERT_EQ(mem_read32(mem, TEST_UHCI0_BASE + 0x64u), 0xA5u);
    ASSERT_EQ(mem_read32(mem, TEST_UHCI1_BASE + 0x64u), 0x33u);
    ASSERT_EQ(mem_read32(mem, TEST_UHCI1_BASE + 0x00u),
              TEST_UHCI_CONF0_RESET);
    ASSERT_EQ(periph_unhandled_count(p), 0);

    periph_destroy(p);
    mem_destroy(mem);
}

TEST(uhci_transparent_tx_dma_wire_timing_quick_send_and_interrupt) {
    const uint32_t desc = 0x3FFB3000u;
    const uint32_t buf = 0x3FFB3100u;
    static const uint8_t expected[8] = {
        0x10, 0x21, 0x32, 0x43, 0x54, 0x65, 0x76, 0x87,
    };
    xtensa_mem_t *mem = mem_create();
    esp32_periph_t *p = periph_create(mem);
    xtensa_cpu_t cpu;
    xtensa_cpu_init(&cpu); cpu.mem = mem;
    periph_attach_cpus(p, &cpu, NULL);
    periph_intr_matrix_set(p, 0, 10, 12);
    mem_write32(mem, 0x3FFE01E0u, 240u);
    mem_write32(mem, 0x3FF000C0u, 1u << 8);

    /* UART1: APB clock, 8-N-1, one megabaud. */
    mem_write32(mem, 0x3FF50014u, 80u);
    mem_write32(mem, 0x3FF50020u, (1u << 27) | (3u << 2));
    for (size_t index = 0; index < sizeof(expected); index++)
        mem_write8(mem, buf + (uint32_t)index, expected[index]);
    test_spi_dma_desc(mem, desc, buf, sizeof(expected), sizeof(expected),
                      1, 0u);

    mem_write32(mem, TEST_UHCI0_BASE + 0x00u,
                (1u << 22) | (1u << 10)); /* transparent, UART1 */
    mem_write32(mem, TEST_UHCI0_BASE + 0x2Cu, 1u << 6); /* owner check */
    mem_write32(mem, TEST_UHCI0_BASE + 0x0Cu,
                (1u << 7) | (1u << 8) | (1u << 13));
    mem_write32(mem, TEST_UHCI0_BASE + 0x24u,
                (desc & 0xFFFFFu) | TEST_UHCI_LINK_START);

    ASSERT_EQ(periph_uart_tx_count_num(p, 1), 0);
    ASSERT_TRUE(cpu.next_timer_event != UINT32_MAX);
    ASSERT_EQ(cpu.next_timer_event - cpu.ccount, 19200u);
    cpu.ccount = cpu.next_timer_event;
    cpu.periph_event(&cpu);

    ASSERT_EQ(periph_uart_tx_count_num(p, 1), sizeof(expected));
    ASSERT_TRUE(memcmp(periph_uart_tx_buf_num(p, 1), expected,
                       sizeof(expected)) == 0);
    ASSERT_FALSE(mem_read32(mem, desc) & TEST_UHCI_DESC_OWNER);
    ASSERT_EQ(mem_read32(mem, TEST_UHCI0_BASE + 0x04u) &
              ((1u << 7) | (1u << 8) | (1u << 13)),
              (1u << 7) | (1u << 8) | (1u << 13));
    ASSERT_EQ(mem_read32(mem, TEST_UHCI0_BASE + 0x38u), desc);
    ASSERT_EQ(mem_read32(mem, TEST_UHCI0_BASE + 0x58u), desc);
    ASSERT_TRUE(mem_read32(mem, TEST_UHCI0_BASE + 0x24u) &
                TEST_UHCI_LINK_PARK);
    ASSERT_TRUE(periph_interrupt_pending(p, 12));
    ASSERT_EQ(cpu.interrupt & (1u << 10), 1u << 10);
    mem_write32(mem, TEST_UHCI0_BASE + 0x10u, 0x1FFFFu);
    ASSERT_FALSE(periph_interrupt_pending(p, 12));

    /* APB quick-send packets share the selected UART and its wire clock. */
    mem_write32(mem, TEST_UHCI0_BASE + 0x78u, 0x44332211u);
    mem_write32(mem, TEST_UHCI0_BASE + 0x7Cu, 0x88776655u);
    mem_write32(mem, TEST_UHCI0_BASE + 0x0Cu, 1u << 14);
    mem_write32(mem, TEST_UHCI0_BASE + 0x74u, 1u << 3);
    ASSERT_TRUE(cpu.next_timer_event != UINT32_MAX);
    cpu.ccount = cpu.next_timer_event;
    cpu.periph_event(&cpu);
    static const uint8_t quick[8] = {
        0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88,
    };
    ASSERT_EQ(periph_uart_tx_count_num(p, 1), 16u);
    ASSERT_TRUE(memcmp(periph_uart_tx_buf_num(p, 1) + 8u, quick,
                       sizeof(quick)) == 0);
    ASSERT_EQ(mem_read32(mem, TEST_UHCI0_BASE + 0x08u), 1u << 14);
    ASSERT_TRUE(periph_interrupt_pending(p, 12));
    ASSERT_EQ(periph_unhandled_count(p), 0);

    periph_destroy(p);
    mem_destroy(mem);
}

TEST(uhci_transparent_rx_descriptor_chain_idle_and_break_eof) {
    const uint32_t desc0 = 0x3FFB3200u;
    const uint32_t desc1 = 0x3FFB3210u;
    const uint32_t desc2 = 0x3FFB3220u;
    const uint32_t buf0 = 0x3FFB3300u;
    const uint32_t buf1 = 0x3FFB3310u;
    const uint32_t buf2 = 0x3FFB3320u;
    static const uint8_t input[6] = {0x91, 0x82, 0x73, 0x64, 0x55, 0x46};
    xtensa_mem_t *mem = mem_create();
    esp32_periph_t *p = periph_create(mem);
    xtensa_cpu_t cpu;
    xtensa_cpu_init(&cpu); cpu.mem = mem;
    periph_attach_cpus(p, &cpu, NULL);
    periph_intr_matrix_set(p, 0, 12, 13);
    mem_write32(mem, 0x3FF000C0u, 1u << 12);

    test_spi_dma_desc(mem, desc0, buf0, 4u, 0u, 0, desc1);
    test_spi_dma_desc(mem, desc1, buf1, 8u, 0u, 0, 0u);
    mem_write32(mem, TEST_UHCI1_BASE + 0x00u,
                (1u << 22) | (1u << 19) | (1u << 11));
    mem_write32(mem, TEST_UHCI1_BASE + 0x2Cu, 1u << 6);
    mem_write32(mem, TEST_UHCI1_BASE + 0x0Cu,
                (1u << 4) | (1u << 5));
    mem_write32(mem, TEST_UHCI1_BASE + 0x28u,
                (1u << 20) | (desc0 & 0xFFFFFu) |
                TEST_UHCI_LINK_START);
    ASSERT_EQ(periph_uart_rx_inject_num(p, 2, input, sizeof(input)),
              sizeof(input));
    ASSERT_EQ(periph_uart_rx_pending_num(p, 2), 0u);

    ASSERT_EQ((mem_read32(mem, desc0) >> 12) & 0xFFFu, 4u);
    ASSERT_FALSE(mem_read32(mem, desc0) & TEST_UHCI_DESC_EOF);
    ASSERT_FALSE(mem_read32(mem, desc0) & TEST_UHCI_DESC_OWNER);
    ASSERT_EQ((mem_read32(mem, desc1) >> 12) & 0xFFFu, 2u);
    ASSERT_TRUE(mem_read32(mem, desc1) & TEST_UHCI_DESC_EOF);
    ASSERT_FALSE(mem_read32(mem, desc1) & TEST_UHCI_DESC_OWNER);
    for (unsigned index = 0; index < 4u; index++)
        ASSERT_EQ(mem_read8(mem, buf0 + index), input[index]);
    ASSERT_EQ(mem_read8(mem, buf1), input[4]);
    ASSERT_EQ(mem_read8(mem, buf1 + 1u), input[5]);
    ASSERT_EQ(mem_read32(mem, TEST_UHCI1_BASE + 0x3Cu), desc1);
    ASSERT_EQ(mem_read32(mem, TEST_UHCI1_BASE + 0x4Cu), desc1);
    ASSERT_EQ(mem_read32(mem, TEST_UHCI1_BASE + 0x50u), desc0);
    ASSERT_EQ(mem_read32(mem, TEST_UHCI1_BASE + 0x04u) &
              ((1u << 4) | (1u << 5)), (1u << 4) | (1u << 5));
    ASSERT_TRUE(periph_interrupt_pending(p, 13));
    mem_write32(mem, TEST_UHCI1_BASE + 0x20u, 1u << 16);
    ASSERT_EQ(mem_read32(mem, TEST_UHCI1_BASE + 0x20u), input[0]);

    /* Break EOF completes a partial transparent descriptor independently of
     * the host-injection idle boundary. */
    mem_write32(mem, TEST_UHCI1_BASE + 0x10u, 0x1FFFFu);
    test_spi_dma_desc(mem, desc2, buf2, 8u, 0u, 0, 0u);
    mem_write32(mem, TEST_UHCI1_BASE + 0x00u,
                (1u << 22) | (1u << 23) | (1u << 11));
    mem_write32(mem, TEST_UHCI1_BASE + 0x28u,
                (desc2 & 0xFFFFFu) | TEST_UHCI_LINK_START);
    ASSERT_EQ(periph_uart_rx_inject_num(p, 2, input, 3u), 3u);
    ASSERT_TRUE(mem_read32(mem, desc2) & TEST_UHCI_DESC_OWNER);
    ASSERT_TRUE(periph_uart_rx_break_num(p, 2));
    ASSERT_EQ((mem_read32(mem, desc2) >> 12) & 0xFFFu, 3u);
    ASSERT_TRUE(mem_read32(mem, desc2) & TEST_UHCI_DESC_EOF);
    ASSERT_FALSE(mem_read32(mem, desc2) & TEST_UHCI_DESC_OWNER);
    ASSERT_EQ(periph_unhandled_count(p), 0);

    periph_destroy(p);
    mem_destroy(mem);
}

TEST(uhci_h5_slip_receive_transmit_and_checksum_error) {
    const uint32_t rx_desc = 0x3FFB3400u;
    const uint32_t bad_desc = 0x3FFB3410u;
    const uint32_t tx_desc = 0x3FFB3420u;
    const uint32_t recovery_desc = 0x3FFB3430u;
    const uint32_t rx_buf = 0x3FFB3500u;
    const uint32_t bad_buf = 0x3FFB3510u;
    const uint32_t tx_buf = 0x3FFB3520u;
    const uint32_t recovery_buf = 0x3FFB3530u;
    static const uint8_t payload[4] = {0xC0, 0xDB, 0x11, 0x13};
    static const uint8_t wire[12] = {
        0xC0, 0x00, 0x41, 0x00, 0xBE,
        0xDB, 0xDC, 0xDB, 0xDD, 0x11, 0x13, 0xC0,
    };
    static const uint8_t bad_wire[12] = {
        0xC0, 0x00, 0x41, 0x00, 0xBF,
        0xDB, 0xDC, 0xDB, 0xDD, 0x11, 0x13, 0xC0,
    };
    xtensa_mem_t *mem = mem_create();
    esp32_periph_t *p = periph_create(mem);
    xtensa_cpu_t cpu;
    xtensa_cpu_init(&cpu); cpu.mem = mem;
    periph_attach_cpus(p, &cpu, NULL);
    mem_write32(mem, 0x3FF000C0u, 1u << 8);
    mem_write32(mem, 0x3FFE01E0u, 240u);
    mem_write32(mem, 0x3FF40014u, 80u);
    mem_write32(mem, 0x3FF40020u, (1u << 27) | (3u << 2));

    uint32_t framed_conf =
        (TEST_UHCI_CONF0_RESET & ~(1u << 20)) | (1u << 22) | (1u << 9);
    mem_write32(mem, TEST_UHCI0_BASE + 0x00u, framed_conf);
    mem_write32(mem, TEST_UHCI0_BASE + 0x2Cu,
                TEST_UHCI_CONF1_RESET | (1u << 6));
    test_spi_dma_desc(mem, rx_desc, rx_buf, 16u, 0u, 0, 0u);
    mem_write32(mem, TEST_UHCI0_BASE + 0x28u,
                (rx_desc & 0xFFFFFu) | TEST_UHCI_LINK_START);
    ASSERT_EQ(periph_uart_rx_inject(p, wire, sizeof(wire)), sizeof(wire));
    ASSERT_EQ((mem_read32(mem, rx_desc) >> 12) & 0xFFFu,
              sizeof(payload));
    ASSERT_TRUE(mem_read32(mem, rx_desc) & TEST_UHCI_DESC_EOF);
    for (size_t index = 0; index < sizeof(payload); index++)
        ASSERT_EQ(mem_read8(mem, rx_buf + (uint32_t)index), payload[index]);
    ASSERT_EQ(mem_read32(mem, TEST_UHCI0_BASE + 0x70u), 0xBE004100u);
    ASSERT_EQ(mem_read32(mem, TEST_UHCI0_BASE + 0x04u) &
              ((1u << 1) | (1u << 4) | (1u << 5)),
              (1u << 1) | (1u << 4) | (1u << 5));

    mem_write32(mem, TEST_UHCI0_BASE + 0x10u, 0x1FFFFu);
    test_spi_dma_desc(mem, bad_desc, bad_buf, 16u, 0u, 0,
                      recovery_desc);
    test_spi_dma_desc(mem, recovery_desc, recovery_buf, 16u, 0u, 0, 0u);
    mem_write32(mem, TEST_UHCI0_BASE + 0x28u,
                (bad_desc & 0xFFFFFu) | TEST_UHCI_LINK_START);
    ASSERT_EQ(periph_uart_rx_inject(p, bad_wire, sizeof(bad_wire)),
              sizeof(bad_wire));
    ASSERT_EQ((mem_read32(mem, TEST_UHCI0_BASE + 0x1Cu) >> 4) & 7u, 1u);
    ASSERT_EQ(mem_read32(mem, TEST_UHCI0_BASE + 0x04u) &
              ((1u << 5) | (1u << 6)), 1u << 6);
    ASSERT_EQ(mem_read32(mem, TEST_UHCI0_BASE + 0x40u), bad_desc);

    /* Frame-local parser errors must not poison the next packet in the same
     * descriptor chain. */
    mem_write32(mem, TEST_UHCI0_BASE + 0x10u, 0x1FFFFu);
    ASSERT_EQ(periph_uart_rx_inject(p, wire, sizeof(wire)), sizeof(wire));
    ASSERT_EQ((mem_read32(mem, TEST_UHCI0_BASE + 0x1Cu) >> 4) & 7u, 0u);
    ASSERT_EQ((mem_read32(mem, recovery_desc) >> 12) & 0xFFFu,
              sizeof(payload));
    ASSERT_TRUE(mem_read32(mem, recovery_desc) & TEST_UHCI_DESC_EOF);
    for (size_t index = 0; index < sizeof(payload); index++)
        ASSERT_EQ(mem_read8(mem, recovery_buf + (uint32_t)index),
                  payload[index]);
    ASSERT_EQ(mem_read32(mem, TEST_UHCI0_BASE + 0x3Cu), recovery_desc);
    ASSERT_EQ(mem_read32(mem, TEST_UHCI0_BASE + 0x04u) & (1u << 5), 1u << 5);

    /* The matching TX encoder replaces checksum/ACK fields and escapes the
     * configured C0/DB octets back into the same H:5 wire representation. */
    static const uint8_t packet[8] = {
        0x00, 0x41, 0x00, 0x00, 0xC0, 0xDB, 0x11, 0x13,
    };
    for (size_t index = 0; index < sizeof(packet); index++)
        mem_write8(mem, tx_buf + (uint32_t)index, packet[index]);
    test_spi_dma_desc(mem, tx_desc, tx_buf, sizeof(packet), sizeof(packet),
                      1, 0u);
    mem_write32(mem, TEST_UHCI0_BASE + 0x24u,
                (tx_desc & 0xFFFFFu) | TEST_UHCI_LINK_START);
    ASSERT_TRUE(cpu.next_timer_event != UINT32_MAX);
    cpu.ccount = cpu.next_timer_event;
    cpu.periph_event(&cpu);
    ASSERT_EQ(periph_uart_tx_count(p), sizeof(wire));
    ASSERT_TRUE(memcmp(periph_uart_tx_buf(p), wire, sizeof(wire)) == 0);
    ASSERT_EQ(periph_unhandled_count(p), 0);

    periph_destroy(p);
    mem_destroy(mem);
}

TEST(uhci_h5_reliable_sequence_and_crc_validation) {
    const uint32_t good_desc = 0x3FFB3800u;
    const uint32_t seq_desc = 0x3FFB3810u;
    const uint32_t crc_desc = 0x3FFB3820u;
    const uint32_t good_buf = 0x3FFB3900u;
    const uint32_t seq_buf = 0x3FFB3910u;
    const uint32_t crc_buf = 0x3FFB3920u;
    static const uint8_t payload[4] = {0xC0, 0xDB, 0x11, 0x13};
    /* Reliable sequence zero, CRC-present H:5 frame. The logical CRC is
     * 0x8EE2; C0/DB octets are escaped on the SLIP wire. */
    static const uint8_t seq0_wire[15] = {
        0xC0, 0xDB, 0xDC, 0x41, 0x00, 0xFE,
        0xDB, 0xDC, 0xDB, 0xDD, 0x11, 0x13, 0x8E, 0xE2, 0xC0,
    };
    /* Sequence one has CRC 0x40DB; corrupt only the low CRC byte so header,
     * checksum, length, and sequence validation all still succeed first. */
    static const uint8_t seq1_bad_crc_wire[14] = {
        0xC0, 0xC1, 0x41, 0x00, 0xFD,
        0xDB, 0xDC, 0xDB, 0xDD, 0x11, 0x13, 0x40, 0xDA, 0xC0,
    };
    xtensa_mem_t *mem = mem_create();
    esp32_periph_t *p = periph_create(mem);
    mem_write32(mem, 0x3FF000C0u, 1u << 8);

    uint32_t framed_conf =
        (TEST_UHCI_CONF0_RESET & ~(1u << 20)) | (1u << 22) | (1u << 9);
    mem_write32(mem, TEST_UHCI0_BASE + 0x00u, framed_conf);
    mem_write32(mem, TEST_UHCI0_BASE + 0x2Cu,
                TEST_UHCI_CONF1_RESET | (1u << 6));
    test_spi_dma_desc(mem, good_desc, good_buf, 16u, 0u, 0, seq_desc);
    test_spi_dma_desc(mem, seq_desc, seq_buf, 16u, 0u, 0, crc_desc);
    test_spi_dma_desc(mem, crc_desc, crc_buf, 16u, 0u, 0, 0u);
    mem_write32(mem, TEST_UHCI0_BASE + 0x28u,
                (good_desc & 0xFFFFFu) | TEST_UHCI_LINK_START);

    ASSERT_EQ(periph_uart_rx_inject(p, seq0_wire, sizeof(seq0_wire)),
              sizeof(seq0_wire));
    ASSERT_EQ((mem_read32(mem, good_desc) >> 12) & 0xFFFu,
              sizeof(payload));
    ASSERT_TRUE(mem_read32(mem, good_desc) & TEST_UHCI_DESC_EOF);
    ASSERT_EQ((mem_read32(mem, TEST_UHCI0_BASE + 0x1Cu) >> 4) & 7u, 0u);
    ASSERT_EQ(mem_read32(mem, TEST_UHCI0_BASE + 0x6Cu), 1u);
    ASSERT_EQ(mem_read32(mem, TEST_UHCI0_BASE + 0x3Cu), good_desc);
    for (size_t index = 0; index < sizeof(payload); index++)
        ASSERT_EQ(mem_read8(mem, good_buf + (uint32_t)index), payload[index]);

    /* Replaying reliable sequence zero is a sequence error and must not
     * advance the expected sequence/ACK counter. */
    mem_write32(mem, TEST_UHCI0_BASE + 0x10u, 0x1FFFFu);
    ASSERT_EQ(periph_uart_rx_inject(p, seq0_wire, sizeof(seq0_wire)),
              sizeof(seq0_wire));
    ASSERT_EQ((mem_read32(mem, TEST_UHCI0_BASE + 0x1Cu) >> 4) & 7u, 2u);
    ASSERT_EQ(mem_read32(mem, TEST_UHCI0_BASE + 0x6Cu), 1u);
    ASSERT_EQ(mem_read32(mem, TEST_UHCI0_BASE + 0x40u), seq_desc);
    ASSERT_EQ(mem_read32(mem, TEST_UHCI0_BASE + 0x04u) & (1u << 6),
              1u << 6);

    mem_write32(mem, TEST_UHCI0_BASE + 0x10u, 0x1FFFFu);
    ASSERT_EQ(periph_uart_rx_inject(p, seq1_bad_crc_wire,
                                    sizeof(seq1_bad_crc_wire)),
              sizeof(seq1_bad_crc_wire));
    ASSERT_EQ((mem_read32(mem, TEST_UHCI0_BASE + 0x1Cu) >> 4) & 7u, 6u);
    ASSERT_EQ(mem_read32(mem, TEST_UHCI0_BASE + 0x6Cu), 1u);
    ASSERT_EQ(mem_read32(mem, TEST_UHCI0_BASE + 0x40u), crc_desc);
    ASSERT_EQ((mem_read32(mem, crc_desc) >> 12) & 0xFFFu,
              sizeof(payload));
    ASSERT_EQ(periph_unhandled_count(p), 0);

    periph_destroy(p);
    mem_destroy(mem);
}

TEST(uhci_malformed_descriptors_report_directional_errors) {
    const uint32_t tx_desc = 0x3FFB3600u;
    const uint32_t rx_desc = 0x3FFB3610u;
    const uint32_t tx_buf = 0x3FFB3700u;
    const uint32_t rx_buf = 0x3FFB3710u;
    static const uint8_t input[2] = {0xCA, 0xFE};
    xtensa_mem_t *mem = mem_create();
    esp32_periph_t *p = periph_create(mem);
    xtensa_cpu_t cpu;
    xtensa_cpu_init(&cpu); cpu.mem = mem;
    periph_attach_cpus(p, &cpu, NULL);
    periph_intr_matrix_set(p, 0, 12, 12);
    mem_write32(mem, 0x3FF000C0u, 1u << 8);
    mem_write32(mem, TEST_UHCI0_BASE + 0x00u,
                (1u << 22) | (1u << 9));
    mem_write32(mem, TEST_UHCI0_BASE + 0x2Cu, 1u << 6);
    mem_write32(mem, TEST_UHCI0_BASE + 0x0Cu,
                (1u << 9) | (1u << 10));

    /* Owner remains software-owned: OUT DMA must park and identify the
     * rejected descriptor without touching its payload. */
    test_spi_dma_desc(mem, tx_desc, tx_buf, 4u, 4u, 1, 0u);
    mem_write32(mem, tx_desc,
                mem_read32(mem, tx_desc) & ~TEST_UHCI_DESC_OWNER);
    mem_write32(mem, TEST_UHCI0_BASE + 0x24u,
                (tx_desc & 0xFFFFFu) | TEST_UHCI_LINK_START);
    ASSERT_TRUE(cpu.next_timer_event != UINT32_MAX);
    cpu.ccount = cpu.next_timer_event;
    cpu.periph_event(&cpu);
    ASSERT_EQ(mem_read32(mem, TEST_UHCI0_BASE + 0x04u) & (1u << 10),
              1u << 10);
    ASSERT_EQ(mem_read32(mem, TEST_UHCI0_BASE + 0x44u), tx_desc);
    ASSERT_TRUE(mem_read32(mem, TEST_UHCI0_BASE + 0x24u) &
                TEST_UHCI_LINK_PARK);
    ASSERT_TRUE(periph_interrupt_pending(p, 12));

    /* The same ownership violation on IN DMA raises the directional error;
     * bytes not accepted by UHCI remain available through the UART FIFO. */
    mem_write32(mem, TEST_UHCI0_BASE + 0x10u, 0x1FFFFu);
    test_spi_dma_desc(mem, rx_desc, rx_buf, 4u, 0u, 0, 0u);
    mem_write32(mem, rx_desc,
                mem_read32(mem, rx_desc) & ~TEST_UHCI_DESC_OWNER);
    mem_write32(mem, TEST_UHCI0_BASE + 0x28u,
                (rx_desc & 0xFFFFFu) | TEST_UHCI_LINK_START);
    ASSERT_EQ(periph_uart_rx_inject(p, input, sizeof(input)), sizeof(input));
    ASSERT_EQ(mem_read32(mem, TEST_UHCI0_BASE + 0x04u) & (1u << 9),
              1u << 9);
    ASSERT_EQ(mem_read32(mem, TEST_UHCI0_BASE + 0x40u), rx_desc);
    ASSERT_EQ(periph_uart_rx_pending(p), sizeof(input));
    ASSERT_TRUE(periph_interrupt_pending(p, 12));

    mem_write32(mem, 0x3FF000C4u, 1u << 8);
    ASSERT_FALSE(periph_interrupt_pending(p, 12));
    ASSERT_EQ(mem_read32(mem, TEST_UHCI0_BASE + 0x04u), 0u);
    ASSERT_EQ(periph_unhandled_count(p), 0);

    periph_destroy(p);
    mem_destroy(mem);
}

TEST(sdio_slave_reset_shared_interrupts_and_dport) {
    xtensa_mem_t *mem = mem_create();
    esp32_periph_t *p = periph_create(mem);
    xtensa_cpu_t cpu;
    xtensa_cpu_init(&cpu); cpu.mem = mem;
    periph_attach_cpus(p, &cpu, NULL);
    periph_intr_matrix_set(p, 0, 10, 10);
    periph_intr_matrix_set(p, 0, 12, 11);

    ASSERT_EQ(mem_read32(mem, TEST_HINF_BASE + 0x00u), 0x22226666u);
    ASSERT_EQ(mem_read32(mem, TEST_HINF_BASE + 0x04u), 0x01110011u);
    ASSERT_EQ(mem_read32(mem, TEST_HINF_BASE + 0x1Cu), 0x00020000u);
    ASSERT_EQ(mem_read32(mem, TEST_HINF_BASE + 0x20u), UINT32_MAX);
    ASSERT_EQ(mem_read32(mem, TEST_HINF_BASE + 0x40u), 0x33336666u);
    ASSERT_EQ(mem_read32(mem, TEST_HINF_BASE + 0xFCu), 0x15030200u);
    ASSERT_EQ(mem_read32(mem, TEST_SLC_BASE + 0x00u), 0xFF3CFF30u);
    ASSERT_EQ(mem_read32(mem, TEST_SLC_BASE + 0x60u), 0x00300078u);
    ASSERT_EQ(mem_read32(mem, TEST_SLC_BASE + 0x98u), 0x101B101Au);
    ASSERT_EQ(mem_read32(mem, TEST_SLC_BASE + 0x1F8u), 0x16022500u);
    ASSERT_EQ(mem_read32(mem, TEST_SLC_BASE + 0x1FCu), 0x00000100u);
    ASSERT_EQ(mem_read32(mem, TEST_SLCHOST_BASE + 0x178u), 0x16022500u);
    ASSERT_EQ(mem_read32(mem, TEST_SLCHOST_BASE + 0x17Cu), 0x00000600u);
    ASSERT_TRUE(mem_read32(mem, 0x3FF000CCu) & (1u << 4));
    ASSERT_FALSE(periph_sdio_slave_host_ready(p));

    mem_write32(mem, TEST_HINF_BASE + 0x04u,
                mem_read32(mem, TEST_HINF_BASE + 0x04u) | (1u << 1));
    ASSERT_TRUE(periph_sdio_slave_host_ready(p));

    static const unsigned positions[] = {0u, 23u, 24u, 31u, 32u, 63u};
    for (unsigned index = 0; index < sizeof(positions) / sizeof(positions[0]);
         index++) {
        unsigned position = positions[index];
        uint8_t value = (uint8_t)(0x31u + index * 0x17u);
        uint8_t observed = 0u;
        ASSERT_EQ(periph_sdio_slave_host_write_reg(p, position, value), 0);
        ASSERT_EQ(periph_sdio_slave_host_read_reg(p, position, &observed), 0);
        ASSERT_EQ(observed, value);
        uint32_t address = test_slchost_shared_addr(position);
        ASSERT_EQ(mem_read8(mem, address), value);
        ASSERT_EQ((mem_read32(mem, address & ~3u) >>
                   ((address & 3u) * 8u)) & 0xFFu, value);

        uint8_t guest_value = (uint8_t)(value ^ 0xFFu);
        uint32_t word = mem_read32(mem, address & ~3u);
        unsigned shift = (address & 3u) * 8u;
        mem_write32(mem, address & ~3u,
                    (word & ~(0xFFu << shift)) |
                    ((uint32_t)guest_value << shift));
        ASSERT_EQ(periph_sdio_slave_host_read_reg(p, position, &observed), 0);
        ASSERT_EQ(observed, guest_value);
    }
    uint8_t ignored = 0u;
    ASSERT_EQ(periph_sdio_slave_host_read_reg(p, 64u, &ignored), -1);
    ASSERT_EQ(periph_sdio_slave_host_write_reg(p, 64u, 0u), -1);

    /* Host-to-slave general interrupts occupy the low SLC raw bits and drive
     * the real SLC0 interrupt source when the guest enables them. */
    mem_write32(mem, TEST_SLC_BASE + 0x0Cu, 1u << 3);
    ASSERT_EQ(periph_sdio_slave_host_interrupt(p, 1u << 3), 0);
    ASSERT_EQ(mem_read32(mem, TEST_SLC_BASE + 0x04u) & 0xFFu, 1u << 3);
    ASSERT_EQ(mem_read32(mem, TEST_SLC_BASE + 0x08u), 1u << 3);
    ASSERT_TRUE(periph_interrupt_pending(p, 10));
    ASSERT_EQ(cpu.interrupt & (1u << 10), 1u << 10);
    mem_write32(mem, TEST_SLC_BASE + 0x10u, 1u << 3);
    ASSERT_FALSE(periph_interrupt_pending(p, 10));

    /* SLC1 is a separate source and its host vector occupies bits 16..23. */
    mem_write32(mem, TEST_SLC_BASE + 0x1Cu, TEST_SLC_INT_RX_DSCR_ERR);
    mem_write32(mem, TEST_SLC_BASE + 0x44u,
                0xFFFFFu | TEST_SLC_LINK_START);
    ASSERT_EQ(mem_read32(mem, TEST_SLC_BASE + 0x14u) &
              TEST_SLC_INT_RX_DSCR_ERR, TEST_SLC_INT_RX_DSCR_ERR);
    ASSERT_TRUE(periph_interrupt_pending(p, 11));
    mem_write32(mem, TEST_SLC_BASE + 0x20u, TEST_SLC_INT_RX_DSCR_ERR);
    ASSERT_FALSE(periph_interrupt_pending(p, 11));

    mem_write32(mem, TEST_SLCHOST_BASE + 0xDCu, 1u << 5);
    mem_write32(mem, TEST_SLCHOST_BASE + 0xE0u, 1u << 6);
    mem_write32(mem, TEST_SLC_BASE + 0x4Cu,
                (1u << 5) | (1u << (16u + 6u)));
    ASSERT_EQ(mem_read32(mem, TEST_SLC_BASE + 0x4Cu),
              (1u << 5) | (1u << (16u + 6u)));
    ASSERT_EQ(periph_sdio_slave_host_interrupt_raw(p) & (1u << 5),
              1u << 5);
    ASSERT_EQ(periph_sdio_slave_host_interrupt_pending(p), 1u << 5);
    ASSERT_EQ(mem_read32(mem, TEST_SLCHOST_BASE + 0x54u) & (1u << 6),
              1u << 6);
    ASSERT_EQ(mem_read32(mem, TEST_SLC_BASE + 0x14u) & TEST_SLC_INT_TOHOST,
              TEST_SLC_INT_TOHOST);
    periph_sdio_slave_host_interrupt_clear(p, 1u << 5);
    mem_write32(mem, TEST_SLCHOST_BASE + 0xD8u, 1u << 6);
    ASSERT_EQ(periph_sdio_slave_host_interrupt_raw(p) & (1u << 5), 0u);
    ASSERT_EQ(mem_read32(mem, TEST_SLCHOST_BASE + 0x54u) & (1u << 6), 0u);

    /* HINF masks delivery to the external host without destroying raw state. */
    mem_write32(mem, TEST_SLC_BASE + 0x4Cu, 1u << 5);
    mem_write32(mem, TEST_HINF_BASE + 0x04u,
                mem_read32(mem, TEST_HINF_BASE + 0x04u) | (1u << 6));
    ASSERT_EQ(periph_sdio_slave_host_interrupt_pending(p), 0u);
    ASSERT_EQ(periph_sdio_slave_host_interrupt_raw(p) & (1u << 5),
              1u << 5);

    uint32_t clocks = mem_read32(mem, 0x3FF000CCu);
    mem_write32(mem, 0x3FF000CCu, clocks & ~(1u << 4));
    ASSERT_FALSE(periph_sdio_slave_host_ready(p));
    ASSERT_EQ(periph_sdio_slave_host_interrupt(p, 1u), -1);
    mem_write32(mem, 0x3FF000CCu, clocks);

    periph_sdio_slave_host_write_reg(p, 0u, 0xA5u);
    mem_write32(mem, 0x3FF000D0u, 1u << 5);
    ASSERT_EQ(mem_read32(mem, TEST_HINF_BASE + 0x04u), 0x01110011u);
    ASSERT_EQ(mem_read32(mem, TEST_SLC_BASE + 0x04u), 0u);
    ASSERT_EQ(mem_read32(mem, TEST_SLCHOST_BASE + 0x6Cu), 0u);
    ASSERT_FALSE(periph_interrupt_pending(p, 10));
    ASSERT_FALSE(periph_interrupt_pending(p, 11));
    mem_write32(mem, 0x3FF000D0u, 0u);
    ASSERT_EQ(periph_unhandled_count(p), 0);

    periph_destroy(p);
    mem_destroy(mem);
}

TEST(sdio_slave_scatter_gather_packets_and_writeback) {
    const uint32_t tx_desc0 = 0x3FFB5000u;
    const uint32_t tx_desc1 = 0x3FFB5010u;
    const uint32_t tx_buf0 = 0x3FFB5100u;
    const uint32_t tx_buf1 = 0x3FFB5110u;
    const uint32_t rx_desc0 = 0x3FFB5200u;
    const uint32_t rx_desc1 = 0x3FFB5210u;
    const uint32_t rx_buf0 = 0x3FFB5300u;
    const uint32_t rx_buf1 = 0x3FFB5310u;
    static const uint8_t host_packet[13] = {
        0x10, 0x21, 0x32, 0x43, 0x54, 0x65, 0x76,
        0x87, 0x98, 0xA9, 0xBA, 0xCB, 0xDC,
    };
    static const uint8_t slave_packet[13] = {
        0xF0, 0xE1, 0xD2, 0xC3, 0xB4, 0xA5, 0x96,
        0x87, 0x78, 0x69, 0x5A, 0x4B, 0x3C,
    };
    xtensa_mem_t *mem = mem_create();
    esp32_periph_t *p = periph_create(mem);

    mem_write32(mem, TEST_HINF_BASE + 0x04u,
                mem_read32(mem, TEST_HINF_BASE + 0x04u) | (1u << 1));
    mem_write32(mem, TEST_SLC_BASE + 0x00u,
                mem_read32(mem, TEST_SLC_BASE + 0x00u) | (1u << 6));
    mem_write32(mem, TEST_SLC_BASE + 0x0Cu,
                TEST_SLC_INT_TOKEN1_EMPTY | TEST_SLC_INT_TX_DONE |
                TEST_SLC_INT_TX_SUC_EOF | TEST_SLC_INT_RX_DONE |
                TEST_SLC_INT_RX_EOF);

    /* The host writes one packet across two slave receive descriptors. */
    test_slc_desc(mem, tx_desc0, tx_buf0, 8u, 0u, false, tx_desc1);
    test_slc_desc(mem, tx_desc1, tx_buf1, 8u, 0u, false, 0u);
    mem_write32(mem, TEST_SLC_BASE + 0x40u,
                (tx_desc0 & 0xFFFFFu) | TEST_SLC_LINK_START);
    mem_write32(mem, TEST_SLC_BASE + 0x54u, (1u << 14) | 2u);
    ASSERT_EQ(periph_sdio_slave_host_send_buffers(p), 2u);
    ASSERT_EQ(mem_read32(mem, TEST_SLCHOST_BASE + 0x44u) >> 16u, 2u);
    ASSERT_EQ(periph_sdio_slave_host_write_packet(
                  p, host_packet, sizeof(host_packet)), 1);
    for (unsigned index = 0; index < 8u; index++)
        ASSERT_EQ(mem_read8(mem, tx_buf0 + index), host_packet[index]);
    for (unsigned index = 0; index < 5u; index++)
        ASSERT_EQ(mem_read8(mem, tx_buf1 + index), host_packet[index + 8u]);
    ASSERT_EQ((mem_read32(mem, tx_desc0) >> 12u) & 0xFFFu, 8u);
    ASSERT_EQ((mem_read32(mem, tx_desc1) >> 12u) & 0xFFFu, 5u);
    ASSERT_FALSE(mem_read32(mem, tx_desc0) & TEST_SLC_DESC_EOF);
    ASSERT_TRUE(mem_read32(mem, tx_desc1) & TEST_SLC_DESC_EOF);
    ASSERT_FALSE(mem_read32(mem, tx_desc0) & TEST_SLC_DESC_OWNER);
    ASSERT_FALSE(mem_read32(mem, tx_desc1) & TEST_SLC_DESC_OWNER);
    ASSERT_EQ(periph_sdio_slave_host_send_buffers(p), 0u);
    ASSERT_EQ(mem_read32(mem, TEST_SLC_BASE + 0x7Cu), tx_desc1);
    ASSERT_EQ(mem_read32(mem, TEST_SLC_BASE + 0x80u), tx_desc0);
    ASSERT_TRUE(mem_read32(mem, TEST_SLC_BASE + 0x40u) &
                TEST_SLC_LINK_PARK);
    ASSERT_EQ(mem_read32(mem, TEST_SLC_BASE + 0x04u) &
              (TEST_SLC_INT_TOKEN1_EMPTY | TEST_SLC_INT_TX_DONE |
               TEST_SLC_INT_TX_SUC_EOF),
              TEST_SLC_INT_TOKEN1_EMPTY | TEST_SLC_INT_TX_DONE |
              TEST_SLC_INT_TX_SUC_EOF);
    ASSERT_TRUE(periph_interrupt_pending(p, 10));
    mem_write32(mem, TEST_SLC_BASE + 0x10u, 0x07FFFFFFu);

    /* The host reads the cumulative packet length through two send
     * descriptors. An undersized destination is reported before writeback. */
    for (unsigned index = 0; index < 6u; index++)
        mem_write8(mem, rx_buf0 + index, slave_packet[index]);
    for (unsigned index = 0; index < 7u; index++)
        mem_write8(mem, rx_buf1 + index, slave_packet[index + 6u]);
    test_slc_desc(mem, rx_desc0, rx_buf0, 6u, 6u, false, rx_desc1);
    test_slc_desc(mem, rx_desc1, rx_buf1, 7u, 7u, true, 0u);
    mem_write32(mem, TEST_SLC_BASE + 0x3Cu,
                (rx_desc0 & 0xFFFFFu) | TEST_SLC_LINK_START);
    ASSERT_EQ(mem_read32(mem, TEST_SLC_BASE + 0x04u) &
              TEST_SLC_INT_RX_DONE, TEST_SLC_INT_RX_DONE);
    mem_write32(mem, TEST_SLC_BASE + 0xE4u,
                (1u << 20) | sizeof(slave_packet));
    ASSERT_EQ(periph_sdio_slave_host_receive_bytes(p),
              sizeof(slave_packet));

    uint8_t received[13] = {0};
    size_t received_len = 0u;
    ASSERT_EQ(periph_sdio_slave_host_read_packet(
                  p, received, sizeof(received) - 1u, &received_len), -1);
    ASSERT_EQ(received_len, sizeof(slave_packet));
    ASSERT_TRUE(mem_read32(mem, rx_desc0) & TEST_SLC_DESC_OWNER);
    ASSERT_TRUE(mem_read32(mem, rx_desc1) & TEST_SLC_DESC_OWNER);
    ASSERT_EQ(periph_sdio_slave_host_read_packet(
                  p, received, sizeof(received), &received_len), 1);
    ASSERT_EQ(received_len, sizeof(slave_packet));
    ASSERT_TRUE(memcmp(received, slave_packet, sizeof(slave_packet)) == 0);
    ASSERT_FALSE(mem_read32(mem, rx_desc0) & TEST_SLC_DESC_OWNER);
    ASSERT_FALSE(mem_read32(mem, rx_desc1) & TEST_SLC_DESC_OWNER);
    ASSERT_EQ(mem_read32(mem, TEST_SLC_BASE + 0x78u), rx_desc1);
    ASSERT_EQ(mem_read32(mem, TEST_SLC_BASE + 0x80u), rx_desc0);
    ASSERT_EQ(periph_sdio_slave_host_receive_bytes(p), 0u);
    ASSERT_EQ(mem_read32(mem, TEST_SLC_BASE + 0x04u) &
              (TEST_SLC_INT_RX_DONE | TEST_SLC_INT_RX_EOF),
              TEST_SLC_INT_RX_DONE | TEST_SLC_INT_RX_EOF);
    ASSERT_TRUE(periph_sdio_slave_host_interrupt_raw(p) & (1u << 23));
    ASSERT_EQ(periph_unhandled_count(p), 0);

    periph_destroy(p);
    mem_destroy(mem);
}

TEST(sdio_slave_malformed_dma_is_packet_atomic) {
    const uint32_t tx_desc = 0x3FFB5400u;
    const uint32_t tx_buf = 0x3FFB5500u;
    const uint32_t rx_desc = 0x3FFB5410u;
    const uint32_t rx_buf = 0x3FFB5510u;
    static const uint8_t packet[5] = {1u, 2u, 3u, 4u, 5u};
    xtensa_mem_t *mem = mem_create();
    esp32_periph_t *p = periph_create(mem);

    mem_write32(mem, TEST_HINF_BASE + 0x04u,
                mem_read32(mem, TEST_HINF_BASE + 0x04u) | (1u << 1));
    for (unsigned index = 0; index < 4u; index++)
        mem_write8(mem, tx_buf + index, 0xA5u);
    test_slc_desc(mem, tx_desc, tx_buf, 4u, 0u, false, 0u);
    mem_write32(mem, TEST_SLC_BASE + 0x40u,
                (tx_desc & 0xFFFFFu) | TEST_SLC_LINK_START);
    mem_write32(mem, TEST_SLC_BASE + 0x54u, 1u << 13);
    ASSERT_EQ(periph_sdio_slave_host_write_packet(p, packet,
                                                   sizeof(packet)), 0);
    for (unsigned index = 0; index < 4u; index++)
        ASSERT_EQ(mem_read8(mem, tx_buf + index), 0xA5u);
    ASSERT_TRUE(mem_read32(mem, tx_desc) & TEST_SLC_DESC_OWNER);
    ASSERT_EQ(periph_sdio_slave_host_send_buffers(p), 1u);
    ASSERT_EQ(mem_read32(mem, TEST_SLC_BASE + 0x04u) &
              TEST_SLC_INT_TX_OVF, TEST_SLC_INT_TX_OVF);

    /* A descriptor length larger than its capacity must neither copy bytes
     * nor transfer ownership to the host. */
    test_slc_desc(mem, rx_desc, rx_buf, 4u, 5u, true, 0u);
    mem_write32(mem, TEST_SLC_BASE + 0x3Cu,
                (rx_desc & 0xFFFFFu) | TEST_SLC_LINK_START);
    mem_write32(mem, TEST_SLC_BASE + 0xE4u, (1u << 20) | 5u);
    uint8_t output[5];
    memset(output, 0xCC, sizeof(output));
    size_t output_len = 0u;
    ASSERT_EQ(periph_sdio_slave_host_read_packet(
                  p, output, sizeof(output), &output_len), -1);
    ASSERT_EQ(output_len, sizeof(output));
    for (unsigned index = 0; index < sizeof(output); index++)
        ASSERT_EQ(output[index], 0xCCu);
    ASSERT_TRUE(mem_read32(mem, rx_desc) & TEST_SLC_DESC_OWNER);
    ASSERT_EQ(mem_read32(mem, TEST_SLC_BASE + 0x04u) &
              TEST_SLC_INT_RX_DSCR_ERR, TEST_SLC_INT_RX_DSCR_ERR);

    uint32_t clocks = mem_read32(mem, 0x3FF000CCu);
    mem_write32(mem, 0x3FF000CCu, clocks & ~(1u << 4));
    ASSERT_EQ(periph_sdio_slave_host_write_packet(p, packet,
                                                   sizeof(packet)), -1);
    ASSERT_EQ(periph_unhandled_count(p), 0);

    periph_destroy(p);
    mem_destroy(mem);
}

TEST(sdmmc_reset_slots_dport_and_card_status) {
    xtensa_mem_t *mem = mem_create();
    esp32_periph_t *p = periph_create(mem);
    test_sdmmc_card_t card = {0};

    ASSERT_EQ(mem_read32(mem, TEST_SDMMC_BASE + 0x14u), 0xFFFFFFFFu);
    ASSERT_EQ(mem_read32(mem, TEST_SDMMC_BASE + 0x1Cu), 512u);
    ASSERT_EQ(mem_read32(mem, TEST_SDMMC_BASE + 0x4Cu), 0x00070008u);
    ASSERT_EQ(mem_read32(mem, TEST_SDMMC_BASE + 0x50u), 3u);
    ASSERT_EQ(mem_read32(mem, TEST_SDMMC_BASE + 0x6Cu), 0x5342240Au);
    ASSERT_TRUE(mem_read32(mem, 0x3FF000CCu) & (1u << 13));

    ASSERT_EQ(periph_sdmmc_attach_card(p, 0, 2048u,
                                      test_sdmmc_read_blocks,
                                      test_sdmmc_write_blocks, &card), 0);
    ASSERT_EQ(mem_read32(mem, TEST_SDMMC_BASE + 0x50u), 2u);
    ASSERT_TRUE(mem_read32(mem, TEST_SDMMC_BASE + 0x48u) & (1u << 8));
    ASSERT_EQ(periph_sdmmc_set_write_protected(p, 0, true), 0);
    ASSERT_EQ(mem_read32(mem, TEST_SDMMC_BASE + 0x54u), 1u);
    ASSERT_EQ(periph_sdmmc_set_write_protected(p, 0, false), 0);

    ASSERT_EQ(periph_sdmmc_attach_card(p, 1, 4096u,
                                      test_sdmmc_read_blocks,
                                      test_sdmmc_write_blocks, &card), 0);
    ASSERT_EQ(mem_read32(mem, TEST_SDMMC_BASE + 0x50u), 0u);
    mem_write32(mem, TEST_SDMMC_BASE + 0x68u, 0x12345678u);
    mem_write32(mem, 0x3FF000D0u, 1u << 6);
    ASSERT_EQ(mem_read32(mem, TEST_SDMMC_BASE + 0x68u), 0u);
    ASSERT_EQ(mem_read32(mem, TEST_SDMMC_BASE + 0x50u), 0u);
    ASSERT_EQ(mem_read32(mem, 0x3FF000D0u), 1u << 6);
    mem_write32(mem, 0x3FF000D0u, 0u);
    ASSERT_EQ(periph_unhandled_count(p), 0);

    periph_destroy(p);
    mem_destroy(mem);
}

TEST(sdmmc_native_image_uses_existing_capacity) {
    char path[] = "/tmp/flexe-sdmmc-native-XXXXXX";
    int fd = mkstemp(path);
    ASSERT_TRUE(fd >= 0);
    if (fd < 0) return;
    ASSERT_EQ(ftruncate(fd, 2 * 1024 * 1024), 0);
    ASSERT_EQ(close(fd), 0);

    xtensa_mem_t *mem = mem_create();
    esp32_periph_t *p = periph_create(mem);
    sdcard_stubs_t *stubs = sdcard_stubs_create(NULL);
    ASSERT_TRUE(mem != NULL);
    ASSERT_TRUE(p != NULL);
    ASSERT_TRUE(stubs != NULL);
    if (!mem || !p || !stubs) {
        sdcard_stubs_destroy(stubs);
        periph_destroy(p);
        mem_destroy(mem);
        unlink(path);
        return;
    }

    /* requested_size is a minimum. A larger existing image must retain its
     * full capacity when exposed through the native host controller. */
    sdcard_stubs_set_image(stubs, path);
    sdcard_stubs_set_size(stubs, 1024u * 1024u);
    ASSERT_EQ(sdcard_stubs_attach_sdmmc(stubs, p), 0);
    ASSERT_FALSE(mem_read32(mem, TEST_SDMMC_BASE + 0x50u) & (1u << 1));

    test_sdmmc_command(mem, 9u, 2u << 16, TEST_SDMMC_CMD_RESP, 1u);
    uint32_t c_size = mem_read32(mem, TEST_SDMMC_BASE + 0x34u) >> 16;
    c_size |= (mem_read32(mem, TEST_SDMMC_BASE + 0x38u) & 0x3Fu) << 16;
    ASSERT_EQ(c_size, 3u); /* (C_SIZE + 1) * 1024 = 4096 sectors = 2 MiB */
    ASSERT_EQ(periph_unhandled_count(p), 0);

    sdcard_stubs_destroy(stubs);
    periph_destroy(p);
    mem_destroy(mem);
    ASSERT_EQ(unlink(path), 0);
}

TEST(sdmmc_commands_responses_and_pio_fifo) {
    xtensa_mem_t *mem = mem_create();
    esp32_periph_t *p = periph_create(mem);
    test_sdmmc_card_t card = {0};
    for (unsigned index = 0; index < 512u; index++)
        card.block[1][index] = (uint8_t)(0xA5u ^ index);
    ASSERT_EQ(periph_sdmmc_attach_card(p, 0, 2048u,
                                      test_sdmmc_read_blocks,
                                      test_sdmmc_write_blocks, &card), 0);
    mem_write32(mem, TEST_SDMMC_BASE + 0x00u, TEST_SDMMC_CTRL_INT);
    mem_write32(mem, TEST_SDMMC_BASE + 0x24u, 0xFFFFFFFFu);
    mem_write32(mem, TEST_SDMMC_BASE + 0x44u, 0xFFFFFFFFu);

    test_sdmmc_command(mem, 8u, 0x1AAu, TEST_SDMMC_CMD_RESP, 0u);
    ASSERT_FALSE(mem_read32(mem, TEST_SDMMC_BASE + 0x2Cu) &
                 TEST_SDMMC_CMD_START);
    ASSERT_EQ(mem_read32(mem, TEST_SDMMC_BASE + 0x30u), 0x1AAu);
    ASSERT_TRUE(mem_read32(mem, TEST_SDMMC_BASE + 0x44u) & (1u << 2));
    ASSERT_TRUE(periph_interrupt_pending(p, 37));
    mem_write32(mem, TEST_SDMMC_BASE + 0x44u, 0xFFFFFFFFu);

    test_sdmmc_command(mem, 55u, 0u, TEST_SDMMC_CMD_RESP, 0u);
    ASSERT_TRUE(mem_read32(mem, TEST_SDMMC_BASE + 0x30u) & (1u << 5));
    mem_write32(mem, TEST_SDMMC_BASE + 0x44u, 0xFFFFFFFFu);
    test_sdmmc_command(mem, 41u, 0x40FF8000u,
                       TEST_SDMMC_CMD_RESP, 0u);
    ASSERT_EQ(mem_read32(mem, TEST_SDMMC_BASE + 0x30u), 0xC0FF8000u);
    mem_write32(mem, TEST_SDMMC_BASE + 0x44u, 0xFFFFFFFFu);
    test_sdmmc_command(mem, 9u, 1u << 16, TEST_SDMMC_CMD_RESP, 0u);
    ASSERT_EQ(mem_read32(mem, TEST_SDMMC_BASE + 0x3Cu) >> 30, 1u);

    mem_write32(mem, TEST_SDMMC_BASE + 0x20u, 512u);
    mem_write32(mem, TEST_SDMMC_BASE + 0x1Cu, 512u);
    mem_write32(mem, TEST_SDMMC_BASE + 0x44u, 0xFFFFFFFFu);
    test_sdmmc_command(mem, 17u, 1u,
                       TEST_SDMMC_CMD_RESP | TEST_SDMMC_CMD_DATA, 0u);
    ASSERT_EQ(card.reads, 1u);
    for (unsigned word_index = 0; word_index < 128u; word_index++) {
        uint32_t expected = 0u;
        for (unsigned byte = 0; byte < 4u; byte++)
            expected |= (uint32_t)(uint8_t)(0xA5u ^
                        (word_index * 4u + byte)) << (byte * 8u);
        ASSERT_EQ(mem_read32(mem, TEST_SDMMC_BASE + 0x200u), expected);
    }
    ASSERT_TRUE(mem_read32(mem, TEST_SDMMC_BASE + 0x44u) & (1u << 3));

    mem_write32(mem, TEST_SDMMC_BASE + 0x44u, 0xFFFFFFFFu);
    test_sdmmc_command(mem, 24u, 2u,
                       TEST_SDMMC_CMD_RESP | TEST_SDMMC_CMD_DATA |
                       TEST_SDMMC_CMD_WRITE, 0u);
    for (unsigned word_index = 0; word_index < 128u; word_index++) {
        uint32_t value = 0u;
        for (unsigned byte = 0; byte < 4u; byte++)
            value |= (uint32_t)(uint8_t)(0x3Cu +
                     word_index * 4u + byte) << (byte * 8u);
        mem_write32(mem, TEST_SDMMC_BASE + 0x200u, value);
    }
    ASSERT_EQ(card.writes, 1u);
    for (unsigned index = 0; index < 512u; index++)
        ASSERT_EQ(card.block[2][index], (uint8_t)(0x3Cu + index));
    ASSERT_TRUE(mem_read32(mem, TEST_SDMMC_BASE + 0x44u) & (1u << 3));
    ASSERT_EQ(periph_unhandled_count(p), 0);

    periph_destroy(p);
    mem_destroy(mem);
}

TEST(sdmmc_idmac_chains_writeback_interrupts_and_errors) {
    const uint32_t desc0 = 0x3FFB3A00u;
    const uint32_t desc1 = 0x3FFB3A10u;
    const uint32_t bad_desc = 0x3FFB3A20u;
    const uint32_t buf0 = 0x3FFB3C00u;
    const uint32_t buf1 = 0x3FFB3D00u;
    xtensa_mem_t *mem = mem_create();
    esp32_periph_t *p = periph_create(mem);
    test_sdmmc_card_t card = {0};
    for (unsigned index = 0; index < 512u; index++)
        card.block[3][index] = (uint8_t)(index ^ 0x6Du);
    ASSERT_EQ(periph_sdmmc_attach_card(p, 0, 2048u,
                                      test_sdmmc_read_blocks,
                                      test_sdmmc_write_blocks, &card), 0);
    xtensa_cpu_t cpu;
    xtensa_cpu_init(&cpu);
    cpu.mem = mem;
    periph_attach_cpus(p, &cpu, NULL);

    mem_write32(mem, TEST_SDMMC_BASE + 0x00u,
                TEST_SDMMC_CTRL_INT | TEST_SDMMC_CTRL_DMA |
                TEST_SDMMC_CTRL_IDMAC);
    mem_write32(mem, TEST_SDMMC_BASE + 0x24u, (1u << 2) | (1u << 3));
    mem_write32(mem, TEST_SDMMC_BASE + 0x80u, 1u << 7);
    mem_write32(mem, TEST_SDMMC_BASE + 0x90u,
                (1u << 1) | (1u << 8) | (1u << 4) | (1u << 9));
    test_sdmmc_desc(mem, desc0,
                    TEST_SDMMC_DESC_OWNER | TEST_SDMMC_DESC_FIRST |
                    TEST_SDMMC_DESC_CHAINED,
                    256u, buf0, desc1);
    test_sdmmc_desc(mem, desc1,
                    TEST_SDMMC_DESC_OWNER | TEST_SDMMC_DESC_LAST,
                    256u, buf1, 0u);
    mem_write32(mem, TEST_SDMMC_BASE + 0x88u, desc0);
    mem_write32(mem, TEST_SDMMC_BASE + 0x20u, 512u);
    mem_write32(mem, TEST_SDMMC_BASE + 0x1Cu, 512u);
    mem_write32(mem, TEST_SDMMC_BASE + 0x44u, 0xFFFFFFFFu);
    test_sdmmc_command(mem, 17u, 3u,
                       TEST_SDMMC_CMD_RESP | TEST_SDMMC_CMD_DATA, 0u);
    ASSERT_TRUE(mem_read32(mem, TEST_SDMMC_BASE + 0x44u) & (1u << 2));
    mem_write32(mem, TEST_SDMMC_BASE + 0x44u, 1u << 2);
    ASSERT_TRUE(cpu.next_timer_event != UINT32_MAX);
    cpu.ccount = cpu.next_timer_event;
    cpu.periph_event(&cpu);
    ASSERT_FALSE(mem_read32(mem, desc0) & TEST_SDMMC_DESC_OWNER);
    ASSERT_TRUE(mem_read32(mem, desc1) & TEST_SDMMC_DESC_OWNER);
    ASSERT_EQ(mem_read32(mem, TEST_SDMMC_BASE + 0x8Cu) &
              ((1u << 1) | (1u << 8)), (1u << 1) | (1u << 8));
    for (unsigned index = 0; index < 256u; index++)
        ASSERT_EQ(mem_read8(mem, buf0 + index), (uint8_t)(index ^ 0x6Du));

    mem_write32(mem, TEST_SDMMC_BASE + 0x8Cu, 0xFFFFFFFFu);
    ASSERT_TRUE(cpu.next_timer_event != UINT32_MAX);
    cpu.ccount = cpu.next_timer_event;
    cpu.periph_event(&cpu);
    ASSERT_FALSE(mem_read32(mem, desc1) & TEST_SDMMC_DESC_OWNER);
    for (unsigned index = 0; index < 256u; index++)
        ASSERT_EQ(mem_read8(mem, buf1 + index),
                  (uint8_t)((index + 256u) ^ 0x6Du));
    ASSERT_TRUE(mem_read32(mem, TEST_SDMMC_BASE + 0x44u) & (1u << 3));
    ASSERT_EQ(mem_read32(mem, TEST_SDMMC_BASE + 0x94u), desc1);
    ASSERT_EQ(mem_read32(mem, TEST_SDMMC_BASE + 0x5Cu), 512u);

    /* A descriptor not owned by IDMAC reports DU/AIS and leaves its owner
     * clear rather than silently consuming the transfer. */
    mem_write32(mem, TEST_SDMMC_BASE + 0x44u, 0xFFFFFFFFu);
    mem_write32(mem, TEST_SDMMC_BASE + 0x8Cu, 0xFFFFFFFFu);
    test_sdmmc_desc(mem, bad_desc, TEST_SDMMC_DESC_FIRST |
                    TEST_SDMMC_DESC_LAST, 512u, buf0, 0u);
    mem_write32(mem, TEST_SDMMC_BASE + 0x88u, bad_desc);
    test_sdmmc_command(mem, 17u, 3u,
                       TEST_SDMMC_CMD_RESP | TEST_SDMMC_CMD_DATA, 0u);
    mem_write32(mem, TEST_SDMMC_BASE + 0x44u, 1u << 2);
    ASSERT_TRUE(cpu.next_timer_event != UINT32_MAX);
    cpu.ccount = cpu.next_timer_event;
    cpu.periph_event(&cpu);
    ASSERT_EQ(mem_read32(mem, TEST_SDMMC_BASE + 0x8Cu) &
              ((1u << 4) | (1u << 9)), (1u << 4) | (1u << 9));
    ASSERT_TRUE(mem_read32(mem, TEST_SDMMC_BASE + 0x44u) & (1u << 11));
    ASSERT_EQ(periph_unhandled_count(p), 0);

    periph_destroy(p);
    mem_destroy(mem);
}

TEST(sdmmc_slot1_clock_gate_idmac_write_and_media_errors) {
    const uint32_t desc = 0x3FFB3E00u;
    const uint32_t buf0 = 0x3FFB4000u;
    const uint32_t buf1 = 0x3FFB4100u;
    xtensa_mem_t *mem = mem_create();
    esp32_periph_t *p = periph_create(mem);
    test_sdmmc_card_t card = {0};
    ASSERT_EQ(periph_sdmmc_attach_card(p, 1, 4096u,
                                      test_sdmmc_read_blocks,
                                      test_sdmmc_write_blocks, &card), 0);
    /* Rejecting an invalid replacement must leave the existing card wired. */
    ASSERT_EQ(periph_sdmmc_attach_card(p, 1, 512u,
                                      test_sdmmc_read_blocks,
                                      test_sdmmc_write_blocks, &card), -1);
    ASSERT_FALSE(mem_read32(mem, TEST_SDMMC_BASE + 0x50u) & (1u << 1));

    xtensa_cpu_t cpu;
    xtensa_cpu_init(&cpu);
    cpu.mem = mem;
    periph_attach_cpus(p, &cpu, NULL);

    uint32_t clocks = mem_read32(mem, 0x3FF000CCu);
    mem_write32(mem, 0x3FF000CCu, clocks & ~(1u << 13));
    test_sdmmc_command(mem, 8u, 0x1AAu, TEST_SDMMC_CMD_RESP, 1u);
    ASSERT_TRUE(mem_read32(mem, TEST_SDMMC_BASE + 0x2Cu) &
                TEST_SDMMC_CMD_START);
    ASSERT_EQ(mem_read32(mem, TEST_SDMMC_BASE + 0x44u), 1u);
    mem_write32(mem, 0x3FF000CCu, clocks | (1u << 13));
    test_sdmmc_command(mem, 8u, 0x1AAu, TEST_SDMMC_CMD_RESP, 1u);
    ASSERT_EQ(mem_read32(mem, TEST_SDMMC_BASE + 0x30u), 0x1AAu);
    mem_write32(mem, TEST_SDMMC_BASE + 0x44u, 0xFFFFFFFFu);

    for (unsigned index = 0; index < 256u; index++) {
        mem_write8(mem, buf0 + index, (uint8_t)(0x27u + index));
        mem_write8(mem, buf1 + index, (uint8_t)(0xD8u - index));
    }
    mem_write32(mem, desc,
                TEST_SDMMC_DESC_OWNER | TEST_SDMMC_DESC_FIRST |
                TEST_SDMMC_DESC_LAST);
    mem_write32(mem, desc + 4u, 256u | (256u << 13));
    mem_write32(mem, desc + 8u, buf0);
    mem_write32(mem, desc + 12u, buf1);
    mem_write32(mem, TEST_SDMMC_BASE + 0x00u,
                TEST_SDMMC_CTRL_INT | TEST_SDMMC_CTRL_DMA |
                TEST_SDMMC_CTRL_IDMAC);
    mem_write32(mem, TEST_SDMMC_BASE + 0x80u, 1u << 7);
    mem_write32(mem, TEST_SDMMC_BASE + 0x90u, 0xFFFFFFFFu);
    ASSERT_EQ(mem_read32(mem, TEST_SDMMC_BASE + 0x90u), 0x337u);
    mem_write32(mem, TEST_SDMMC_BASE + 0x88u, desc);
    mem_write32(mem, TEST_SDMMC_BASE + 0x20u, 512u);
    mem_write32(mem, TEST_SDMMC_BASE + 0x1Cu, 512u);
    test_sdmmc_command(mem, 24u, 4u,
                       TEST_SDMMC_CMD_RESP | TEST_SDMMC_CMD_DATA |
                       TEST_SDMMC_CMD_WRITE, 1u);
    mem_write32(mem, TEST_SDMMC_BASE + 0x44u, 1u << 2);
    ASSERT_TRUE(cpu.next_timer_event != UINT32_MAX);
    cpu.ccount = cpu.next_timer_event;
    cpu.periph_event(&cpu);
    ASSERT_FALSE(mem_read32(mem, desc) & TEST_SDMMC_DESC_OWNER);
    ASSERT_EQ(mem_read32(mem, TEST_SDMMC_BASE + 0x8Cu) &
              ((1u << 0) | (1u << 8)), (1u << 0) | (1u << 8));
    ASSERT_EQ(card.writes, 1u);
    for (unsigned index = 0; index < 256u; index++) {
        ASSERT_EQ(card.block[4][index], (uint8_t)(0x27u + index));
        ASSERT_EQ(card.block[4][index + 256u], (uint8_t)(0xD8u - index));
    }

    mem_write32(mem, TEST_SDMMC_BASE + 0x44u, 0xFFFFFFFFu);
    mem_write32(mem, TEST_SDMMC_BASE + 0x8Cu, 0xFFFFFFFFu);
    ASSERT_EQ(periph_sdmmc_set_write_protected(p, 1, true), 0);
    test_sdmmc_command(mem, 24u, 5u,
                       TEST_SDMMC_CMD_RESP | TEST_SDMMC_CMD_DATA |
                       TEST_SDMMC_CMD_WRITE, 1u);
    ASSERT_EQ(mem_read32(mem, TEST_SDMMC_BASE + 0x44u) &
              ((1u << 2) | (1u << 3) | (1u << 9)),
              (1u << 2) | (1u << 3) | (1u << 9));
    ASSERT_EQ(card.writes, 1u);

    ASSERT_EQ(periph_sdmmc_attach_card(p, 1, 0u, NULL, NULL, NULL), 0);
    mem_write32(mem, TEST_SDMMC_BASE + 0x44u, 0xFFFFFFFFu);
    test_sdmmc_command(mem, 8u, 0x1AAu, TEST_SDMMC_CMD_RESP, 1u);
    ASSERT_EQ(mem_read32(mem, TEST_SDMMC_BASE + 0x44u) &
              ((1u << 2) | (1u << 8)), (1u << 2) | (1u << 8));
    ASSERT_EQ(periph_unhandled_count(p), 0);

    periph_destroy(p);
    mem_destroy(mem);
}

#define TEST_TWAI_BASE 0x3FF6B000u
#define TEST_TWAI_MODE_RESET (1u << 0)
#define TEST_TWAI_MODE_SINGLE_FILTER (1u << 3)
#define TEST_TWAI_STATUS_RBS (1u << 0)
#define TEST_TWAI_STATUS_DOS (1u << 1)
#define TEST_TWAI_STATUS_TBS (1u << 2)
#define TEST_TWAI_STATUS_TCS (1u << 3)
#define TEST_TWAI_STATUS_TS (1u << 5)
#define TEST_TWAI_STATUS_ES (1u << 6)
#define TEST_TWAI_STATUS_BS (1u << 7)
#define TEST_TWAI_INT_RI (1u << 0)
#define TEST_TWAI_INT_TI (1u << 1)
#define TEST_TWAI_INT_EI (1u << 2)
#define TEST_TWAI_INT_DOI (1u << 3)
#define TEST_TWAI_INT_EPI (1u << 5)
#define TEST_TWAI_INT_ALI (1u << 6)
#define TEST_TWAI_INT_BEI (1u << 7)

static void test_twai_configure(xtensa_mem_t *mem, uint8_t interrupts) {
    /* 500 kbit/s from the 80 MHz APB clock: BRP=8, TSEG1=15, TSEG2=4. */
    mem_write32(mem, TEST_TWAI_BASE + 0x00u,
                TEST_TWAI_MODE_RESET | TEST_TWAI_MODE_SINGLE_FILTER);
    mem_write32(mem, TEST_TWAI_BASE + 0x18u, 3u);
    mem_write32(mem, TEST_TWAI_BASE + 0x1Cu, 0x3Eu);
    mem_write32(mem, TEST_TWAI_BASE + 0x10u, interrupts);
    mem_write32(mem, TEST_TWAI_BASE + 0x7Cu, 0x80u);
}

static void test_twai_write_frame(xtensa_mem_t *mem,
                                  const periph_twai_frame_t *frame) {
    uint8_t encoded[13] = {0};
    encoded[0] = frame->data_length_code & 0x0Fu;
    if (frame->extended) encoded[0] |= 1u << 7;
    if (frame->remote) encoded[0] |= 1u << 6;
    unsigned data_off;
    if (frame->extended) {
        encoded[1] = (uint8_t)(frame->identifier >> 21);
        encoded[2] = (uint8_t)(frame->identifier >> 13);
        encoded[3] = (uint8_t)(frame->identifier >> 5);
        encoded[4] = (uint8_t)(frame->identifier << 3);
        data_off = 5u;
    } else {
        encoded[1] = (uint8_t)(frame->identifier >> 3);
        encoded[2] = (uint8_t)(frame->identifier << 5);
        data_off = 3u;
    }
    if (!frame->remote) {
        unsigned len = frame->data_length_code < 8u ?
                       frame->data_length_code : 8u;
        memcpy(&encoded[data_off], frame->data, len);
    }
    for (unsigned index = 0; index < 13u; index++)
        mem_write32(mem, TEST_TWAI_BASE + 0x40u + index * 4u,
                    encoded[index]);
}

static uint32_t test_twai_run_event(xtensa_cpu_t *cpu) {
    uint32_t event = cpu->periph_next_event(cpu);
    ASSERT_TRUE(event != UINT32_MAX);
    uint32_t elapsed = event - cpu->ccount;
    cpu->ccount = event;
    cpu->periph_event(cpu);
    return elapsed;
}

TEST(twai_reset_acceptance_fifo_overrun_interrupt_and_dport) {
    xtensa_mem_t *mem = mem_create();
    esp32_periph_t *p = periph_create(mem);
    xtensa_cpu_t cpu;
    xtensa_cpu_init(&cpu);
    cpu.mem = mem;
    periph_attach_cpus(p, &cpu, NULL);

    ASSERT_EQ(mem_read32(mem, TEST_TWAI_BASE + 0x00u),
              TEST_TWAI_MODE_RESET);
    ASSERT_EQ(mem_read32(mem, TEST_TWAI_BASE + 0x08u),
              TEST_TWAI_STATUS_TBS | TEST_TWAI_STATUS_TCS);
    ASSERT_EQ(mem_read32(mem, TEST_TWAI_BASE + 0x34u), 96u);
    ASSERT_EQ(mem_read32(mem, TEST_TWAI_BASE + 0x50u), 0xFFu);

    mem_write32(mem, 0x3FF000C0u, 1u << 19);
    test_twai_configure(mem, TEST_TWAI_INT_RI | TEST_TWAI_INT_DOI);
    ASSERT_EQ(mem_read32(mem, TEST_TWAI_BASE + 0x7Cu), 0x80u);

    /* Single-filter mode: accept standard ID 0x321, ignore RTR and data. */
    mem_write32(mem, TEST_TWAI_BASE + 0x40u, 0x64u);
    mem_write32(mem, TEST_TWAI_BASE + 0x44u, 0x20u);
    mem_write32(mem, TEST_TWAI_BASE + 0x48u, 0u);
    mem_write32(mem, TEST_TWAI_BASE + 0x4Cu, 0u);
    mem_write32(mem, TEST_TWAI_BASE + 0x50u, 0u);
    mem_write32(mem, TEST_TWAI_BASE + 0x54u, 0x1Fu);
    mem_write32(mem, TEST_TWAI_BASE + 0x58u, 0xFFu);
    mem_write32(mem, TEST_TWAI_BASE + 0x5Cu, 0xFFu);
    mem_write32(mem, TEST_TWAI_BASE + 0x00u,
                TEST_TWAI_MODE_SINGLE_FILTER);

    periph_twai_frame_t frame = {
        .identifier = 0x322u,
        .data_length_code = 4u,
        .data = {0xA0u, 0xA1u, 0xA2u, 0xA3u},
    };
    ASSERT_EQ(periph_twai_rx_inject(p, &frame), 0);
    frame.identifier = 0x321u;
    ASSERT_EQ(periph_twai_rx_inject(p, &frame), 1);
    ASSERT_EQ(periph_twai_rx_pending(p), 1u);
    ASSERT_EQ(mem_read32(mem, TEST_TWAI_BASE + 0x74u), 1u);
    ASSERT_TRUE(mem_read32(mem, TEST_TWAI_BASE + 0x08u) &
                TEST_TWAI_STATUS_RBS);
    ASSERT_EQ(mem_read32(mem, TEST_TWAI_BASE + 0x40u), 4u);
    ASSERT_EQ(mem_read32(mem, TEST_TWAI_BASE + 0x44u), 0x64u);
    ASSERT_EQ(mem_read32(mem, TEST_TWAI_BASE + 0x48u), 0x20u);
    ASSERT_EQ(mem_read32(mem, TEST_TWAI_BASE + 0x4Cu), 0xA0u);
    ASSERT_TRUE(periph_interrupt_pending(p, 45));
    ASSERT_EQ(mem_read32(mem, TEST_TWAI_BASE + 0x0Cu), TEST_TWAI_INT_RI);
    /* RI is level-like until the final receive buffer is released. */
    ASSERT_TRUE(periph_interrupt_pending(p, 45));
    mem_write32(mem, TEST_TWAI_BASE + 0x04u, 1u << 2);
    ASSERT_EQ(periph_twai_rx_pending(p), 0u);
    ASSERT_FALSE(periph_interrupt_pending(p, 45));

    frame.remote = true;
    frame.data_length_code = 0u;
    for (unsigned index = 0; index < 21u; index++)
        ASSERT_EQ(periph_twai_rx_inject(p, &frame), 1);
    ASSERT_EQ(periph_twai_rx_inject(p, &frame), 0);
    ASSERT_EQ(periph_twai_rx_pending(p), 21u);
    ASSERT_EQ(mem_read32(mem, TEST_TWAI_BASE + 0x0Cu),
              TEST_TWAI_INT_RI | TEST_TWAI_INT_DOI);
    ASSERT_TRUE(mem_read32(mem, TEST_TWAI_BASE + 0x08u) &
                TEST_TWAI_STATUS_DOS);
    for (unsigned index = 0; index < 21u; index++)
        mem_write32(mem, TEST_TWAI_BASE + 0x04u, 1u << 2);
    mem_write32(mem, TEST_TWAI_BASE + 0x04u, 1u << 3);
    ASSERT_EQ(mem_read32(mem, TEST_TWAI_BASE + 0x08u),
              TEST_TWAI_STATUS_TBS | TEST_TWAI_STATUS_TCS);

    mem_write32(mem, 0x3FF000C4u, 1u << 19);
    ASSERT_EQ(mem_read32(mem, TEST_TWAI_BASE + 0x00u),
              TEST_TWAI_MODE_RESET);
    ASSERT_EQ(mem_read32(mem, TEST_TWAI_BASE + 0x34u), 96u);
    ASSERT_FALSE(periph_interrupt_pending(p, 45));
    mem_write32(mem, 0x3FF000C4u, 0u);
    ASSERT_EQ(periph_unhandled_count(p), 0);

    periph_destroy(p);
    mem_destroy(mem);
}

typedef struct {
    unsigned calls;
    unsigned arbitration_failures;
    periph_twai_tx_result_t result;
    periph_twai_frame_t last;
} twai_test_capture_t;

static periph_twai_tx_result_t test_twai_capture(
    void *ctx, const periph_twai_frame_t *frame) {
    twai_test_capture_t *capture = ctx;
    capture->calls++;
    capture->last = *frame;
    if (capture->arbitration_failures != 0u) {
        capture->arbitration_failures--;
        return PERIPH_TWAI_TX_ARBITRATION_LOST;
    }
    return capture->result;
}

TEST(twai_wire_timing_self_reception_retry_busoff_and_recovery) {
    xtensa_mem_t *mem = mem_create();
    esp32_periph_t *p = periph_create(mem);
    xtensa_cpu_t cpu;
    xtensa_cpu_init(&cpu);
    cpu.mem = mem;
    periph_attach_cpus(p, &cpu, NULL);
    mem_write32(mem, 0x3FFE01E0u, 240u);
    mem_write32(mem, 0x3FF000C0u, 1u << 19);
    test_twai_configure(mem, TEST_TWAI_INT_RI | TEST_TWAI_INT_TI |
                       TEST_TWAI_INT_EI | TEST_TWAI_INT_EPI |
                       TEST_TWAI_INT_ALI | TEST_TWAI_INT_BEI);
    /* Accept every standard or extended frame. */
    for (unsigned index = 0; index < 4u; index++)
        mem_write32(mem, TEST_TWAI_BASE + 0x50u + index * 4u, 0xFFu);
    mem_write32(mem, TEST_TWAI_BASE + 0x00u,
                TEST_TWAI_MODE_SINGLE_FILTER);

    twai_test_capture_t capture = {.result = PERIPH_TWAI_TX_ACK};
    ASSERT_EQ(periph_set_twai_tx_callback(p, test_twai_capture, &capture), 0);
    periph_twai_frame_t frame = {
        .identifier = 0x123u,
        .data_length_code = 8u,
        .data = {0x00u, 0xFFu, 0x55u, 0xAAu,
                 0x11u, 0x22u, 0x33u, 0x44u},
    };
    test_twai_write_frame(mem, &frame);
    mem_write32(mem, TEST_TWAI_BASE + 0x04u, 1u);
    ASSERT_EQ(mem_read32(mem, TEST_TWAI_BASE + 0x08u) &
              (TEST_TWAI_STATUS_TBS | TEST_TWAI_STATUS_TCS |
               TEST_TWAI_STATUS_TS), TEST_TWAI_STATUS_TS);
    uint32_t elapsed = test_twai_run_event(&cpu);
    ASSERT_TRUE(elapsed > 40000u && elapsed < 100000u);
    ASSERT_EQ(capture.calls, 1u);
    ASSERT_EQ(capture.last.identifier, 0x123u);
    ASSERT_EQ(capture.last.data_length_code, 8u);
    ASSERT_EQ(capture.last.data[7], 0x44u);
    ASSERT_EQ(mem_read32(mem, TEST_TWAI_BASE + 0x08u) &
              (TEST_TWAI_STATUS_TBS | TEST_TWAI_STATUS_TCS),
              TEST_TWAI_STATUS_TBS | TEST_TWAI_STATUS_TCS);
    ASSERT_EQ(mem_read32(mem, TEST_TWAI_BASE + 0x0Cu), TEST_TWAI_INT_TI);

    frame.identifier = 0x1ABCDE3u;
    frame.data_length_code = 3u;
    frame.data[0] = 0xC1u;
    frame.data[1] = 0xC2u;
    frame.data[2] = 0xC3u;
    frame.extended = true;
    test_twai_write_frame(mem, &frame);
    mem_write32(mem, TEST_TWAI_BASE + 0x04u, 1u << 4);
    test_twai_run_event(&cpu);
    ASSERT_TRUE(capture.last.self_reception);
    ASSERT_EQ(periph_twai_rx_pending(p), 1u);
    ASSERT_EQ(mem_read32(mem, TEST_TWAI_BASE + 0x0Cu),
              TEST_TWAI_INT_RI | TEST_TWAI_INT_TI);
    ASSERT_EQ(mem_read32(mem, TEST_TWAI_BASE + 0x40u) & 0xCFu, 0x83u);
    ASSERT_EQ(mem_read32(mem, TEST_TWAI_BASE + 0x44u), 0x0Du);
    ASSERT_EQ(mem_read32(mem, TEST_TWAI_BASE + 0x48u), 0x5Eu);
    mem_write32(mem, TEST_TWAI_BASE + 0x04u, 1u << 2);

    capture.result = PERIPH_TWAI_TX_NO_ACK;
    frame.identifier = 0x456u;
    frame.extended = false;
    frame.data_length_code = 1u;
    frame.data[0] = 0x5Au;
    test_twai_write_frame(mem, &frame);
    mem_write32(mem, TEST_TWAI_BASE + 0x04u, 0x03u);
    test_twai_run_event(&cpu);
    ASSERT_EQ(mem_read32(mem, TEST_TWAI_BASE + 0x08u) &
              (TEST_TWAI_STATUS_TBS | TEST_TWAI_STATUS_TCS),
              TEST_TWAI_STATUS_TBS);
    ASSERT_EQ(mem_read32(mem, TEST_TWAI_BASE + 0x3Cu), 8u);
    ASSERT_EQ(mem_read32(mem, TEST_TWAI_BASE + 0x0Cu),
              TEST_TWAI_INT_TI | TEST_TWAI_INT_BEI);

    capture.result = PERIPH_TWAI_TX_ACK;
    capture.arbitration_failures = 1u;
    test_twai_write_frame(mem, &frame);
    mem_write32(mem, TEST_TWAI_BASE + 0x04u, 1u);
    test_twai_run_event(&cpu);
    ASSERT_TRUE(mem_read32(mem, TEST_TWAI_BASE + 0x08u) &
                TEST_TWAI_STATUS_TS);
    ASSERT_EQ(mem_read32(mem, TEST_TWAI_BASE + 0x0Cu), TEST_TWAI_INT_ALI);
    test_twai_run_event(&cpu);
    ASSERT_EQ(mem_read32(mem, TEST_TWAI_BASE + 0x0Cu), TEST_TWAI_INT_TI);

    capture.result = PERIPH_TWAI_TX_NO_ACK;
    test_twai_write_frame(mem, &frame);
    mem_write32(mem, TEST_TWAI_BASE + 0x04u, 1u);
    for (unsigned attempt = 0; attempt < 40u; attempt++) {
        test_twai_run_event(&cpu);
        (void)mem_read32(mem, TEST_TWAI_BASE + 0x0Cu);
        if (mem_read32(mem, TEST_TWAI_BASE + 0x08u) &
            TEST_TWAI_STATUS_BS)
            break;
    }
    ASSERT_TRUE(mem_read32(mem, TEST_TWAI_BASE + 0x08u) &
                TEST_TWAI_STATUS_BS);
    ASSERT_TRUE(mem_read32(mem, TEST_TWAI_BASE + 0x08u) &
                TEST_TWAI_STATUS_ES);
    ASSERT_TRUE(mem_read32(mem, TEST_TWAI_BASE + 0x00u) &
                TEST_TWAI_MODE_RESET);
    ASSERT_EQ(mem_read32(mem, TEST_TWAI_BASE + 0x3Cu), 128u);

    mem_write32(mem, TEST_TWAI_BASE + 0x00u, 0u);
    test_twai_run_event(&cpu);
    ASSERT_TRUE(mem_read32(mem, TEST_TWAI_BASE + 0x08u) &
                TEST_TWAI_STATUS_BS);
    ASSERT_FALSE(mem_read32(mem, TEST_TWAI_BASE + 0x08u) &
                 TEST_TWAI_STATUS_ES);
    ASSERT_EQ(mem_read32(mem, TEST_TWAI_BASE + 0x0Cu), TEST_TWAI_INT_EI);
    test_twai_run_event(&cpu);
    ASSERT_FALSE(mem_read32(mem, TEST_TWAI_BASE + 0x08u) &
                 TEST_TWAI_STATUS_BS);
    ASSERT_EQ(mem_read32(mem, TEST_TWAI_BASE + 0x0Cu), TEST_TWAI_INT_EI);
    ASSERT_EQ(periph_unhandled_count(p), 0);

    periph_destroy(p);
    mem_destroy(mem);
}

#define TEST_EMAC_DMA_BASE 0x3FF69000u
#define TEST_EMAC_EXT_BASE 0x3FF69800u
#define TEST_EMAC_MAC_BASE 0x3FF6A000u
#define TEST_EMAC_OWN (1u << 31)
#define TEST_EMAC_TX_IOC (1u << 30)
#define TEST_EMAC_TX_LAST (1u << 29)
#define TEST_EMAC_TX_FIRST (1u << 28)
#define TEST_EMAC_TX_CHAINED (1u << 20)
#define TEST_EMAC_TX_ERROR (1u << 15)
#define TEST_EMAC_TX_NO_CARRIER (1u << 10)
#define TEST_EMAC_RX_CHAINED (1u << 14)
#define TEST_EMAC_RX_FIRST (1u << 9)
#define TEST_EMAC_RX_LAST (1u << 8)
#define TEST_EMAC_RX_DA_FAIL (1u << 30)
#define TEST_EMAC_DMA_TX_INT (1u << 0)
#define TEST_EMAC_DMA_TX_UNAVAILABLE (1u << 2)
#define TEST_EMAC_DMA_RX_INT (1u << 6)
#define TEST_EMAC_DMA_RX_UNAVAILABLE (1u << 7)
#define TEST_EMAC_DMA_NORMAL_SUMMARY (1u << 16)

typedef struct {
    uint8_t frame[256];
    size_t len;
    unsigned count;
    int result;
} emac_test_capture_t;

typedef struct {
    unsigned calls;
    uint8_t phy;
    uint8_t reg;
    bool write;
    uint16_t value;
} emac_test_mdio_t;

static int test_emac_capture(void *ctx, const uint8_t *frame, size_t len) {
    emac_test_capture_t *capture = ctx;
    capture->count++;
    capture->len = len;
    if (len <= sizeof(capture->frame))
        memcpy(capture->frame, frame, len);
    return capture->result;
}

static int test_emac_mdio(void *ctx, uint8_t phy, uint8_t reg,
                          bool write, uint16_t *value) {
    emac_test_mdio_t *mdio = ctx;
    mdio->calls++;
    mdio->phy = phy;
    mdio->reg = reg;
    mdio->write = write;
    if (write) mdio->value = *value;
    else *value = mdio->value;
    return 0;
}

static uint32_t test_emac_crc32(const uint8_t *data, size_t len) {
    uint32_t crc = UINT32_MAX;
    for (size_t index = 0; index < len; index++) {
        uint8_t byte = data[index];
        for (unsigned bit = 0; bit < 8u; bit++) {
            uint32_t mix = (crc ^ byte) & 1u;
            crc >>= 1u;
            if (mix) crc ^= 0xEDB88320u;
            byte >>= 1u;
        }
    }
    return ~crc;
}

static void test_emac_descriptor(xtensa_mem_t *mem, uint32_t descriptor,
                                 uint32_t status, uint32_t control,
                                 uint32_t buffer, uint32_t next) {
    mem_write32(mem, descriptor, status);
    mem_write32(mem, descriptor + 4u, control);
    mem_write32(mem, descriptor + 8u, buffer);
    mem_write32(mem, descriptor + 12u, next);
    for (unsigned word = 4u; word < 8u; word++)
        mem_write32(mem, descriptor + word * 4u, 0u);
}

TEST(emac_reset_clock_extension_and_clause22_mdio) {
    xtensa_mem_t *mem = mem_create();
    esp32_periph_t *p = periph_create(mem);

    ASSERT_TRUE(mem_read32(mem, TEST_EMAC_MAC_BASE) & (1u << 15));
    ASSERT_TRUE(mem_read32(mem, TEST_EMAC_MAC_BASE + 0x40u) & (1u << 31));
    ASSERT_EQ(mem_read32(mem, TEST_EMAC_EXT_BASE + 0xFCu), 0x15040200u);
    ASSERT_EQ(periph_emac_phy_set_reg(p, 3u, 2u, 0x2000u), 0);

    uint32_t read_command = 1u | (2u << 6u) | (3u << 11u);
    mem_write32(mem, TEST_EMAC_MAC_BASE + 0x10u, read_command);
    ASSERT_FALSE(mem_read32(mem, TEST_EMAC_MAC_BASE + 0x10u) & 1u);
    ASSERT_EQ(mem_read32(mem, TEST_EMAC_MAC_BASE + 0x14u), 0x2000u);

    mem_write32(mem, TEST_EMAC_MAC_BASE + 0x14u, 0x55AAu);
    mem_write32(mem, TEST_EMAC_MAC_BASE + 0x10u,
                1u | (1u << 1u) | (4u << 6u) | (3u << 11u));
    uint16_t phy_value = 0u;
    ASSERT_EQ(periph_emac_phy_get_reg(p, 3u, 4u, &phy_value), 0);
    ASSERT_EQ(phy_value, 0x55AAu);

    mem_write32(mem, TEST_EMAC_MAC_BASE + 0x10u,
                1u | (2u << 6u) | (7u << 11u));
    ASSERT_EQ(mem_read32(mem, TEST_EMAC_MAC_BASE + 0x14u), 0xFFFFu);

    emac_test_mdio_t mdio = {.value = 0xBEEFu};
    ASSERT_EQ(periph_set_emac_mdio_callback(p, test_emac_mdio, &mdio), 0);
    mem_write32(mem, TEST_EMAC_MAC_BASE + 0x10u,
                1u | (5u << 6u) | (9u << 11u));
    ASSERT_EQ(mdio.calls, 1u);
    ASSERT_EQ(mdio.phy, 9u);
    ASSERT_EQ(mdio.reg, 5u);
    ASSERT_FALSE(mdio.write);
    ASSERT_EQ(mem_read32(mem, TEST_EMAC_MAC_BASE + 0x14u), 0xBEEFu);
    mem_write32(mem, TEST_EMAC_MAC_BASE + 0x14u, 0x1234u);
    mem_write32(mem, TEST_EMAC_MAC_BASE + 0x10u,
                1u | (1u << 1u) | (6u << 6u) | (9u << 11u));
    ASSERT_EQ(mdio.calls, 2u);
    ASSERT_EQ(mdio.reg, 6u);
    ASSERT_TRUE(mdio.write);
    ASSERT_EQ(mdio.value, 0x1234u);

    mem_write32(mem, TEST_EMAC_MAC_BASE, 0xDEADBEEFu);
    mem_write32(mem, 0x3FF000D0u, 1u << 7u);
    ASSERT_EQ(mem_read32(mem, TEST_EMAC_MAC_BASE), 1u << 15u);
    ASSERT_EQ(periph_emac_phy_get_reg(p, 3u, 4u, &phy_value), 0);
    ASSERT_EQ(phy_value, 0x55AAu);
    mem_write32(mem, 0x3FF000D0u, 0u);
    ASSERT_EQ(periph_unhandled_count(p), 0);

    periph_destroy(p);
    mem_destroy(mem);
}

TEST(emac_enhanced_chained_tx_completion_error_and_interrupt) {
    xtensa_mem_t *mem = mem_create();
    esp32_periph_t *p = periph_create(mem);
    emac_test_capture_t capture = {0};
    ASSERT_EQ(periph_set_emac_tx_callback(p, test_emac_capture, &capture), 0);

    const uint32_t desc0 = 0x3FFB0000u;
    const uint32_t desc1 = desc0 + 32u;
    const uint32_t buffer0 = 0x3FFB1000u;
    const uint32_t buffer1 = 0x3FFB1200u;
    uint8_t expected[70];
    for (unsigned index = 0; index < sizeof(expected); index++) {
        expected[index] = (uint8_t)(0x30u + index);
        mem_write8(mem, index < 40u ? buffer0 + index :
                   buffer1 + index - 40u, expected[index]);
    }
    test_emac_descriptor(mem, desc0,
                         TEST_EMAC_OWN | TEST_EMAC_TX_FIRST |
                         TEST_EMAC_TX_IOC | TEST_EMAC_TX_CHAINED,
                         40u, buffer0, desc1);
    test_emac_descriptor(mem, desc1,
                         TEST_EMAC_OWN | TEST_EMAC_TX_LAST |
                         TEST_EMAC_TX_CHAINED,
                         30u, buffer1, desc0);

    mem_write32(mem, TEST_EMAC_DMA_BASE, 1u << 7u);
    mem_write32(mem, TEST_EMAC_DMA_BASE + 0x10u, desc0);
    mem_write32(mem, TEST_EMAC_DMA_BASE + 0x1Cu,
                TEST_EMAC_DMA_TX_INT | TEST_EMAC_DMA_NORMAL_SUMMARY);
    mem_write32(mem, TEST_EMAC_MAC_BASE, (1u << 15u) | (1u << 3u));
    mem_write32(mem, TEST_EMAC_DMA_BASE + 0x18u, 1u << 13u);

    ASSERT_EQ(capture.count, 1u);
    ASSERT_EQ(capture.len, sizeof(expected));
    ASSERT_EQ(memcmp(capture.frame, expected, sizeof(expected)), 0);
    ASSERT_FALSE(mem_read32(mem, desc0) & TEST_EMAC_OWN);
    ASSERT_FALSE(mem_read32(mem, desc1) & TEST_EMAC_OWN);
    uint32_t status = mem_read32(mem, TEST_EMAC_DMA_BASE + 0x14u);
    ASSERT_TRUE(status & TEST_EMAC_DMA_TX_INT);
    ASSERT_TRUE(status & TEST_EMAC_DMA_TX_UNAVAILABLE);
    ASSERT_TRUE(status & TEST_EMAC_DMA_NORMAL_SUMMARY);
    ASSERT_TRUE(periph_interrupt_pending(p, 38));
    mem_write32(mem, TEST_EMAC_DMA_BASE + 0x14u,
                TEST_EMAC_DMA_TX_INT | TEST_EMAC_DMA_TX_UNAVAILABLE |
                TEST_EMAC_DMA_NORMAL_SUMMARY);
    ASSERT_FALSE(periph_interrupt_pending(p, 38));

    capture.result = -1;
    mem_write32(mem, desc0, mem_read32(mem, desc0) | TEST_EMAC_OWN);
    mem_write32(mem, desc1, mem_read32(mem, desc1) | TEST_EMAC_OWN);
    mem_write32(mem, TEST_EMAC_DMA_BASE + 0x04u, 0u);
    ASSERT_EQ(capture.count, 2u);
    ASSERT_TRUE(mem_read32(mem, desc1) & TEST_EMAC_TX_ERROR);
    ASSERT_TRUE(mem_read32(mem, desc1) & TEST_EMAC_TX_NO_CARRIER);
    ASSERT_EQ(periph_unhandled_count(p), 0);

    periph_destroy(p);
    mem_destroy(mem);
}

TEST(emac_rx_filter_fcs_descriptor_chain_unavailable_and_receive_all) {
    xtensa_mem_t *mem = mem_create();
    esp32_periph_t *p = periph_create(mem);
    const uint32_t desc0 = 0x3FFB2000u;
    const uint32_t desc1 = desc0 + 32u;
    const uint32_t buffer0 = 0x3FFB3000u;
    const uint32_t buffer1 = 0x3FFB3200u;
    test_emac_descriptor(mem, desc0, TEST_EMAC_OWN,
                         TEST_EMAC_RX_CHAINED | 32u,
                         buffer0, desc1);
    test_emac_descriptor(mem, desc1, TEST_EMAC_OWN,
                         TEST_EMAC_RX_CHAINED | 32u,
                         buffer1, desc0);

    /* MAC address 02:11:22:33:44:55. */
    mem_write32(mem, TEST_EMAC_MAC_BASE + 0x44u, 0x33221102u);
    mem_write32(mem, TEST_EMAC_MAC_BASE + 0x40u, 0x00005544u);
    mem_write32(mem, TEST_EMAC_DMA_BASE, 1u << 7u);
    mem_write32(mem, TEST_EMAC_DMA_BASE + 0x0Cu, desc0);
    mem_write32(mem, TEST_EMAC_DMA_BASE + 0x1Cu,
                TEST_EMAC_DMA_RX_INT | TEST_EMAC_DMA_NORMAL_SUMMARY);
    mem_write32(mem, TEST_EMAC_MAC_BASE,
                (1u << 15u) | (1u << 2u));
    mem_write32(mem, TEST_EMAC_DMA_BASE + 0x18u, 1u << 1u);

    uint8_t frame[60] = {
        0x02, 0x11, 0x22, 0x33, 0x44, 0x55,
        0x02, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE,
        0x08, 0x00,
    };
    for (unsigned index = 14u; index < sizeof(frame); index++)
        frame[index] = (uint8_t)index;
    ASSERT_EQ(periph_emac_rx_inject(p, frame, sizeof(frame)), 1);
    ASSERT_TRUE(mem_read32(mem, desc0) & TEST_EMAC_RX_FIRST);
    uint32_t last_status = mem_read32(mem, desc1);
    ASSERT_TRUE(last_status & TEST_EMAC_RX_LAST);
    ASSERT_EQ((last_status >> 16u) & 0x3FFFu, sizeof(frame) + 4u);
    ASSERT_FALSE(last_status & TEST_EMAC_OWN);
    for (unsigned index = 0; index < 32u; index++)
        ASSERT_EQ(mem_read8(mem, buffer0 + index), frame[index]);
    for (unsigned index = 32u; index < sizeof(frame); index++)
        ASSERT_EQ(mem_read8(mem, buffer1 + index - 32u), frame[index]);
    uint32_t expected_fcs = test_emac_crc32(frame, sizeof(frame));
    for (unsigned index = 0; index < 4u; index++)
        ASSERT_EQ(mem_read8(mem, buffer1 + 28u + index),
                  (uint8_t)(expected_fcs >> (index * 8u)));
    ASSERT_TRUE(mem_read32(mem, TEST_EMAC_DMA_BASE + 0x14u) &
                TEST_EMAC_DMA_RX_INT);
    ASSERT_TRUE(periph_interrupt_pending(p, 38));

    mem_write32(mem, TEST_EMAC_DMA_BASE + 0x14u,
                TEST_EMAC_DMA_RX_INT | TEST_EMAC_DMA_NORMAL_SUMMARY);
    ASSERT_EQ(periph_emac_rx_inject(p, frame, sizeof(frame)), 0);
    ASSERT_TRUE(mem_read32(mem, TEST_EMAC_DMA_BASE + 0x14u) &
                TEST_EMAC_DMA_RX_UNAVAILABLE);
    ASSERT_EQ(mem_read32(mem, TEST_EMAC_DMA_BASE + 0x20u), 1u);
    ASSERT_EQ(mem_read32(mem, TEST_EMAC_DMA_BASE + 0x20u), 0u);
    mem_write32(mem, TEST_EMAC_DMA_BASE + 0x14u,
                TEST_EMAC_DMA_RX_UNAVAILABLE);

    mem_write32(mem, desc0, TEST_EMAC_OWN);
    mem_write32(mem, desc1, TEST_EMAC_OWN);
    frame[0] = 0x06u;
    ASSERT_EQ(periph_emac_rx_inject(p, frame, sizeof(frame)), 0);
    ASSERT_TRUE(mem_read32(mem, desc0) & TEST_EMAC_OWN);
    mem_write32(mem, TEST_EMAC_MAC_BASE + 0x04u, 1u << 31u);
    ASSERT_EQ(periph_emac_rx_inject(p, frame, sizeof(frame)), 1);
    ASSERT_TRUE(mem_read32(mem, desc1) & TEST_EMAC_RX_DA_FAIL);
    ASSERT_EQ(periph_unhandled_count(p), 0);

    periph_destroy(p);
    mem_destroy(mem);
}

typedef struct {
    int count;
    uint8_t last;
} uart_test_callback_t;

static void uart_test_capture(void *ctx, uint8_t byte) {
    uart_test_callback_t *capture = ctx;
    capture->count++;
    capture->last = byte;
}

TEST(uart_controllers_are_independent) {
    xtensa_mem_t *mem = mem_create();
    esp32_periph_t *p = periph_create(mem);
    uart_test_callback_t uart1_capture = {0};
    uart_test_callback_t uart2_capture = {0};
    periph_set_uart_callback_num(p, 1, uart_test_capture, &uart1_capture);
    periph_set_uart_callback_num(p, 2, uart_test_capture, &uart2_capture);

    /* Exercise both the APB register and ESP-IDF's AHB FIFO alias. */
    mem_write32(mem, 0x3FF50000u, '1');
    mem_write32(mem, 0x6002E000u, '2');

    ASSERT_EQ(periph_uart_tx_count(p), 0);
    ASSERT_EQ(periph_uart_tx_count_num(p, 1), 1);
    ASSERT_EQ(periph_uart_tx_count_num(p, 2), 1);
    ASSERT_EQ(periph_uart_tx_buf_num(p, 1)[0], '1');
    ASSERT_EQ(periph_uart_tx_buf_num(p, 2)[0], '2');
    ASSERT_EQ(uart1_capture.count, 1);
    ASSERT_EQ(uart1_capture.last, '1');
    ASSERT_EQ(uart2_capture.count, 1);
    ASSERT_EQ(uart2_capture.last, '2');

    /* Configuration shadows are per-controller as well. */
    mem_write32(mem, 0x3FF50014u, 0x1111u);
    mem_write32(mem, 0x3FF6E014u, 0x2222u);
    ASSERT_EQ(mem_read32(mem, 0x3FF50014u), 0x1111u);
    ASSERT_EQ(mem_read32(mem, 0x3FF6E014u), 0x2222u);
    ASSERT_EQ(mem_read32(mem, 0x3FF40014u), 0u);

    periph_destroy(p);
    mem_destroy(mem);
}

TEST(uart_status_tx_ready) {
    xtensa_mem_t *mem = mem_create();
    esp32_periph_t *p = periph_create(mem);
    /* STATUS register should indicate TX ready (0 = empty FIFO) */
    ASSERT_EQ(mem_read32(mem, 0x3FF4001C), 0);
    periph_destroy(p);
    mem_destroy(mem);
}

TEST(uart_tx_empty_interrupt) {
    xtensa_mem_t *mem = mem_create();
    esp32_periph_t *p = periph_create(mem);
    xtensa_cpu_t cpu0;
    xtensa_cpu_init(&cpu0); cpu0.mem = mem;
    periph_attach_cpus(p, &cpu0, NULL);
    periph_intr_matrix_set(p, 0, 3, 34); /* UART0 -> CPU interrupt 3 */

    ASSERT_EQ(mem_read32(mem, 0x3FF40004), 0u); /* INT_RAW reset */
    mem_write32(mem, 0x3FF4000C, 1u << 1);      /* enable TXFIFO_EMPTY */
    ASSERT_EQ(mem_read32(mem, 0x3FF40004) & (1u << 1), 1u << 1);
    ASSERT_EQ(mem_read32(mem, 0x3FF40008), 1u << 1); /* INT_ST */
    ASSERT_EQ(cpu0.interrupt & (1u << 3), 1u << 3);

    /* ESP-IDF disables the level interrupt before acknowledging it. */
    mem_write32(mem, 0x3FF4000C, 0);
    ASSERT_EQ(cpu0.interrupt & (1u << 3), 0u);
    mem_write32(mem, 0x3FF40010, 1u << 1);
    ASSERT_EQ(mem_read32(mem, 0x3FF40004) & (1u << 1), 0u);

    /* Re-enabling with an empty FIFO must start the next dequeue pass. */
    mem_write32(mem, 0x3FF4000C, 1u << 1);
    ASSERT_EQ(cpu0.interrupt & (1u << 3), 1u << 3);

    periph_destroy(p);
    mem_destroy(mem);
}

TEST(uart_tx_done_interrupt) {
    xtensa_mem_t *mem = mem_create();
    esp32_periph_t *p = periph_create(mem);
    xtensa_cpu_t cpu0;
    xtensa_cpu_init(&cpu0); cpu0.mem = mem;
    periph_attach_cpus(p, &cpu0, NULL);
    periph_intr_matrix_set(p, 0, 4, 34);

    mem_write32(mem, 0x3FF4000C, 1u << 14); /* enable TX_DONE */
    ASSERT_EQ(cpu0.interrupt & (1u << 4), 0u);
    mem_write32(mem, 0x3FF40000, 'X');
    ASSERT_EQ(mem_read32(mem, 0x3FF40008), 1u << 14);
    ASSERT_EQ(cpu0.interrupt & (1u << 4), 1u << 4);
    mem_write32(mem, 0x3FF40010, 1u << 14);
    ASSERT_EQ(mem_read32(mem, 0x3FF40008), 0u);
    ASSERT_EQ(cpu0.interrupt & (1u << 4), 0u);

    periph_destroy(p);
    mem_destroy(mem);
}

TEST(uart_rx_fifo_injection) {
    xtensa_mem_t *mem = mem_create();
    esp32_periph_t *p = periph_create(mem);
    static const uint8_t input[] = {'O', 'K', '\n'};

    ASSERT_EQ(periph_uart_rx_inject(p, input, sizeof(input)), sizeof(input));
    ASSERT_EQ(periph_uart_rx_pending(p), sizeof(input));
    ASSERT_EQ(mem_read32(mem, 0x3FF4001C) & 0xFFu, sizeof(input));
    ASSERT_EQ((mem_read32(mem, 0x3FF40060) >> 2) & 0x7FFu, 0u);
    ASSERT_EQ((mem_read32(mem, 0x3FF40060) >> 13) & 0x7FFu, sizeof(input));
    ASSERT_EQ(mem_read32(mem, 0x3FF40000), 'O');
    ASSERT_EQ(mem_read32(mem, 0x3FF40000), 'K');
    ASSERT_EQ(mem_read32(mem, 0x3FF40000), '\n');
    ASSERT_EQ(mem_read32(mem, 0x3FF40000), 0u);
    ASSERT_EQ(periph_uart_rx_pending(p), 0u);
    ASSERT_EQ((mem_read32(mem, 0x3FF40060) >> 2) & 0x7FFu, sizeof(input));

    static const uint8_t second[] = {'G', 'O', '\n'};
    ASSERT_EQ(periph_uart_rx_inject(p, second, sizeof(second)),
              sizeof(second));
    ASSERT_EQ((mem_read32(mem, 0x3FF40060) >> 2) & 0x7FFu, sizeof(input));
    ASSERT_EQ((mem_read32(mem, 0x3FF40060) >> 13) & 0x7FFu,
              sizeof(input) + sizeof(second));
    ASSERT_EQ(mem_read32(mem, 0x3FF40000), 'G');
    ASSERT_EQ(mem_read32(mem, 0x3FF40000), 'O');
    ASSERT_EQ(mem_read32(mem, 0x3FF40000), '\n');

    periph_destroy(p);
    mem_destroy(mem);
}

TEST(uart_rx_timeout_interrupt) {
    xtensa_mem_t *mem = mem_create();
    esp32_periph_t *p = periph_create(mem);
    xtensa_cpu_t cpu0;
    xtensa_cpu_init(&cpu0); cpu0.mem = mem;
    periph_attach_cpus(p, &cpu0, NULL);
    periph_intr_matrix_set(p, 0, 5, 34); /* UART0 -> CPU interrupt 5 */

    /* Enable RX timeout generation in CONF1 and its interrupt in INT_ENA. */
    mem_write32(mem, 0x3FF40024, (1u << 31) | (10u << 24) | 2u);
    mem_write32(mem, 0x3FF4000C, 1u << 8);
    const uint8_t input = 'X';
    ASSERT_EQ(periph_uart_rx_inject(p, &input, 1), 1u);
    ASSERT_EQ(mem_read32(mem, 0x3FF40004) & (1u << 8), 1u << 8);
    ASSERT_EQ(mem_read32(mem, 0x3FF40008), 1u << 8);
    ASSERT_EQ(cpu0.interrupt & (1u << 5), 1u << 5);

    ASSERT_EQ(mem_read32(mem, 0x3FF40000), 'X');
    mem_write32(mem, 0x3FF40010, 1u << 8);
    ASSERT_EQ(mem_read32(mem, 0x3FF40008), 0u);
    ASSERT_EQ(cpu0.interrupt & (1u << 5), 0u);

    periph_destroy(p);
    mem_destroy(mem);
}

TEST(uart2_rx_fifo_and_interrupt) {
    xtensa_mem_t *mem = mem_create();
    esp32_periph_t *p = periph_create(mem);
    xtensa_cpu_t cpu0;
    xtensa_cpu_init(&cpu0); cpu0.mem = mem;
    periph_attach_cpus(p, &cpu0, NULL);
    periph_intr_matrix_set(p, 0, 8, 36); /* UART2 -> external CPU interrupt 8 */

    const uint32_t base = 0x3FF6E000u;
    mem_write32(mem, base + 0x24, (1u << 31) | (10u << 24) | 2u);
    mem_write32(mem, base + 0x0C,
                (1u << 0) | (1u << 8));
    static const uint8_t nmea[] = "$GPRMC\r\n";
    ASSERT_EQ(periph_uart_rx_inject_num(p, 2, nmea, sizeof(nmea) - 1),
              sizeof(nmea) - 1);
    ASSERT_EQ(periph_uart_rx_pending_num(p, 2), sizeof(nmea) - 1);
    ASSERT_EQ(periph_uart_rx_pending(p), 0u);
    ASSERT_EQ(mem_read32(mem, base + 0x1C) & 0xFFu,
              sizeof(nmea) - 1);
    ASSERT_EQ(mem_read32(mem, base + 0x08) & ((1u << 0) | (1u << 8)),
              (1u << 0) | (1u << 8));
    ASSERT_EQ(cpu0.interrupt & (1u << 8), 1u << 8);
    ASSERT_EQ(mem_read32(mem, base), '$');
    ASSERT_EQ(periph_uart_rx_pending_num(p, 2), sizeof(nmea) - 2);

    while (periph_uart_rx_pending_num(p, 2) != 0)
        (void)mem_read32(mem, base);
    mem_write32(mem, base + 0x10, (1u << 0) | (1u << 8));
    ASSERT_EQ(mem_read32(mem, base + 0x08), 0u);
    ASSERT_EQ(cpu0.interrupt & (1u << 8), 0u);

    periph_destroy(p);
    mem_destroy(mem);
}

/* Classic ESP32 I2C master command encoding: byte count [7:0], ACK check
 * enable [8], expected ACK [9], master ACK value [10], opcode [13:11]. */
#define TEST_I2C0_BASE 0x3FF53000u
#define TEST_I2C1_BASE 0x3FF67000u
#define TEST_RTC_I2C_BASE 0x3FF48C00u
#define TEST_SENS_BASE 0x3FF48800u

static uint32_t test_i2c_cmd(unsigned opcode, unsigned count, int ack_check) {
    return ((opcode & 7u) << 11) | (count & 0xFFu) |
           (ack_check ? 1u << 8 : 0);
}

typedef struct {
    int calls;
    int last_port;
    uint8_t last_address;
    uint8_t last_write[64];
    size_t last_write_len;
    size_t last_read_len;
    uint8_t cursor;
    uint8_t regs[256];
    int fail;
} test_i2c_device_t;

static int test_i2c_device(void *ctx, int port, uint8_t address,
                           const uint8_t *write_data, size_t write_len,
                           uint8_t *read_data, size_t read_len) {
    test_i2c_device_t *device = ctx;
    device->calls++;
    device->last_port = port;
    device->last_address = address;
    device->last_write_len = write_len;
    device->last_read_len = read_len;
    size_t capture = write_len < sizeof(device->last_write) ?
                     write_len : sizeof(device->last_write);
    if (capture != 0)
        memcpy(device->last_write, write_data, capture);
    if (write_len != 0) {
        device->cursor = write_data[0];
        for (size_t index = 1; index < write_len; index++)
            device->regs[device->cursor++] = write_data[index];
    }
    for (size_t index = 0; index < read_len; index++)
        read_data[index] = device->regs[device->cursor++];
    return device->fail;
}

TEST(i2c_master_repeated_start_read_and_interrupt) {
    xtensa_mem_t *mem = mem_create();
    esp32_periph_t *p = periph_create(mem);
    xtensa_cpu_t cpu0;
    xtensa_cpu_init(&cpu0); cpu0.mem = mem;
    periph_attach_cpus(p, &cpu0, NULL);
    periph_intr_matrix_set(p, 0, 8, 49); /* I2C0 -> external CPU interrupt 8 */

    test_i2c_device_t device = {0};
    device.regs[0x10] = 0xA5;
    device.regs[0x11] = 0x5A;
    ASSERT_EQ(periph_i2c_attach_device(p, 0, 0x34,
                                       test_i2c_device, &device), 0);

    /* Write register pointer 0x10, repeated-start into a two-byte read. Use
     * ESP-IDF's AHB FIFO alias for the three transmitted bytes. */
    mem_write32(mem, 0x6001301Cu, 0x34u << 1);
    mem_write32(mem, 0x6001301Cu, 0x10u);
    mem_write32(mem, 0x6001301Cu, (0x34u << 1) | 1u);
    const uint32_t commands[] = {
        test_i2c_cmd(0, 0, 0),
        test_i2c_cmd(1, 2, 1),
        test_i2c_cmd(0, 0, 0),
        test_i2c_cmd(1, 1, 1),
        test_i2c_cmd(2, 2, 0),
        test_i2c_cmd(3, 0, 0),
    };
    for (size_t index = 0; index < sizeof(commands) / sizeof(commands[0]);
         index++)
        mem_write32(mem, TEST_I2C0_BASE + 0x58u + (uint32_t)index * 4u,
                    commands[index]);
    mem_write32(mem, TEST_I2C0_BASE + 0x28u, 1u << 7);
    mem_write32(mem, TEST_I2C0_BASE + 0x04u, (1u << 4) | (1u << 5));

    ASSERT_EQ(device.calls, 1);
    ASSERT_EQ(device.last_port, 0);
    ASSERT_EQ(device.last_address, 0x34);
    ASSERT_EQ(device.last_write_len, 1u);
    ASSERT_EQ(device.last_write[0], 0x10u);
    ASSERT_EQ(device.last_read_len, 2u);
    ASSERT_EQ((mem_read32(mem, TEST_I2C0_BASE + 0x08u) >> 8) & 0x3Fu,
              2u);
    ASSERT_EQ(mem_read32(mem, TEST_I2C0_BASE + 0x1Cu), 0xA5u);
    ASSERT_EQ(mem_read32(mem, TEST_I2C0_BASE + 0x1Cu), 0x5Au);
    ASSERT_EQ(mem_read32(mem, TEST_I2C0_BASE + 0x1Cu), 0u);
    ASSERT_EQ(mem_read32(mem, TEST_I2C0_BASE + 0x20u) & (1u << 7),
              1u << 7);
    ASSERT_EQ(mem_read32(mem, TEST_I2C0_BASE + 0x2Cu), 1u << 7);
    ASSERT_EQ(cpu0.interrupt & (1u << 8), 1u << 8);
    for (size_t index = 0; index < sizeof(commands) / sizeof(commands[0]);
         index++)
        ASSERT_TRUE(mem_read32(mem, TEST_I2C0_BASE + 0x58u +
                               (uint32_t)index * 4u) & (1u << 31));

    mem_write32(mem, TEST_I2C0_BASE + 0x24u, 1u << 7);
    ASSERT_EQ(cpu0.interrupt & (1u << 8), 0u);
    ASSERT_EQ(mem_read32(mem, TEST_I2C0_BASE + 0xF8u), 0x16042000u);
    ASSERT_EQ(periph_unhandled_count(p), 0);

    periph_destroy(p);
    mem_destroy(mem);
}

TEST(i2c_end_command_streams_fifo_chunks_until_stop) {
    xtensa_mem_t *mem = mem_create();
    esp32_periph_t *p = periph_create(mem);
    test_i2c_device_t device = {0};
    ASSERT_EQ(periph_i2c_attach_device(p, 0, 0x20,
                                       test_i2c_device, &device), 0);

    /* Burst 1: START + address/register pointer, then END without STOP. */
    mem_write32(mem, TEST_I2C0_BASE + 0x1Cu, 0x20u << 1);
    mem_write32(mem, TEST_I2C0_BASE + 0x1Cu, 0x30u);
    mem_write32(mem, TEST_I2C0_BASE + 0x58u, test_i2c_cmd(0, 0, 0));
    mem_write32(mem, TEST_I2C0_BASE + 0x5Cu, test_i2c_cmd(1, 2, 1));
    mem_write32(mem, TEST_I2C0_BASE + 0x60u, test_i2c_cmd(4, 0, 0));
    mem_write32(mem, TEST_I2C0_BASE + 0x04u, (1u << 4) | (1u << 5));
    ASSERT_EQ(device.calls, 0);
    ASSERT_TRUE(mem_read32(mem, TEST_I2C0_BASE + 0x08u) & (1u << 4));
    ASSERT_TRUE(mem_read32(mem, TEST_I2C0_BASE + 0x20u) & (1u << 3));
    mem_write32(mem, TEST_I2C0_BASE + 0x24u, 0x1FFFu);

    /* Burst 2: the IDF driver refills FIFO while retaining bus/address state. */
    mem_write32(mem, TEST_I2C0_BASE + 0x1Cu, 0xDEu);
    mem_write32(mem, TEST_I2C0_BASE + 0x1Cu, 0xADu);
    mem_write32(mem, TEST_I2C0_BASE + 0x58u, test_i2c_cmd(1, 2, 1));
    mem_write32(mem, TEST_I2C0_BASE + 0x5Cu, test_i2c_cmd(4, 0, 0));
    mem_write32(mem, TEST_I2C0_BASE + 0x04u, (1u << 4) | (1u << 5));
    ASSERT_EQ(device.calls, 0);
    ASSERT_TRUE(mem_read32(mem, TEST_I2C0_BASE + 0x08u) & (1u << 4));
    mem_write32(mem, TEST_I2C0_BASE + 0x24u, 0x1FFFu);

    /* Burst 3: STOP commits the complete write atomically to the target. */
    mem_write32(mem, TEST_I2C0_BASE + 0x58u, test_i2c_cmd(3, 0, 0));
    mem_write32(mem, TEST_I2C0_BASE + 0x04u, (1u << 4) | (1u << 5));
    ASSERT_EQ(device.calls, 1);
    ASSERT_EQ(device.last_write_len, 3u);
    ASSERT_EQ(device.last_write[0], 0x30u);
    ASSERT_EQ(device.last_write[1], 0xDEu);
    ASSERT_EQ(device.last_write[2], 0xADu);
    ASSERT_EQ(device.regs[0x30], 0xDEu);
    ASSERT_EQ(device.regs[0x31], 0xADu);
    ASSERT_FALSE(mem_read32(mem, TEST_I2C0_BASE + 0x08u) & (1u << 4));
    ASSERT_TRUE(mem_read32(mem, TEST_I2C0_BASE + 0x20u) & (1u << 7));
    ASSERT_EQ(periph_unhandled_count(p), 0);

    periph_destroy(p);
    mem_destroy(mem);
}

TEST(i2c_dual_port_ahb_alias_and_address_nack) {
    xtensa_mem_t *mem = mem_create();
    esp32_periph_t *p = periph_create(mem);
    xtensa_cpu_t cpu0;
    xtensa_cpu_init(&cpu0); cpu0.mem = mem;
    periph_attach_cpus(p, &cpu0, NULL);
    periph_intr_matrix_set(p, 0, 8, 50); /* I2C1 */
    periph_intr_matrix_set(p, 0, 9, 49); /* I2C0 */

    test_i2c_device_t device = {0};
    ASSERT_EQ(periph_i2c_attach_device(p, 1, 0x50,
                                       test_i2c_device, &device), 0);
    mem_write32(mem, 0x6002701Cu, 0x50u << 1);
    mem_write32(mem, 0x6002701Cu, 0x01u);
    mem_write32(mem, 0x6002701Cu, 0xCCu);
    mem_write32(mem, TEST_I2C1_BASE + 0x58u, test_i2c_cmd(0, 0, 0));
    mem_write32(mem, TEST_I2C1_BASE + 0x5Cu, test_i2c_cmd(1, 3, 1));
    mem_write32(mem, TEST_I2C1_BASE + 0x60u, test_i2c_cmd(3, 0, 0));
    mem_write32(mem, TEST_I2C1_BASE + 0x28u, 1u << 7);
    mem_write32(mem, TEST_I2C1_BASE + 0x04u, (1u << 4) | (1u << 5));
    ASSERT_EQ(device.calls, 1);
    ASSERT_EQ(device.last_port, 1);
    ASSERT_EQ(device.regs[1], 0xCCu);
    ASSERT_EQ(cpu0.interrupt & (1u << 8), 1u << 8);

    /* An unattached address NACKs during the address WRITE and raises the
     * controller's ACK_ERR source instead of silently succeeding. */
    mem_write32(mem, 0x6001301Cu, 0x51u << 1);
    mem_write32(mem, TEST_I2C0_BASE + 0x58u, test_i2c_cmd(0, 0, 0));
    mem_write32(mem, TEST_I2C0_BASE + 0x5Cu, test_i2c_cmd(1, 1, 1));
    mem_write32(mem, TEST_I2C0_BASE + 0x60u, test_i2c_cmd(3, 0, 0));
    mem_write32(mem, TEST_I2C0_BASE + 0x28u, 1u << 10);
    mem_write32(mem, TEST_I2C0_BASE + 0x04u, (1u << 4) | (1u << 5));
    ASSERT_EQ(mem_read32(mem, TEST_I2C0_BASE + 0x20u) & (1u << 10),
              1u << 10);
    ASSERT_EQ(mem_read32(mem, TEST_I2C0_BASE + 0x2Cu), 1u << 10);
    ASSERT_TRUE(mem_read32(mem, TEST_I2C0_BASE + 0x08u) & 1u);
    ASSERT_EQ(cpu0.interrupt & (1u << 9), 1u << 9);
    ASSERT_EQ(periph_unhandled_count(p), 0);

    periph_destroy(p);
    mem_destroy(mem);
}

TEST(rtc_i2c_register_masks_and_command_file) {
    xtensa_mem_t *mem = mem_create();
    esp32_periph_t *p = periph_create(mem);

    ASSERT_EQ(mem_read32(mem, TEST_RTC_I2C_BASE + 0x00u), 0u);
    ASSERT_EQ(mem_read32(mem, TEST_RTC_I2C_BASE + 0x04u), 0u);
    ASSERT_EQ(mem_read32(mem, TEST_RTC_I2C_BASE + 0x20u), 0u);

    mem_write32(mem, TEST_RTC_I2C_BASE + 0x00u, UINT32_MAX);
    mem_write32(mem, TEST_RTC_I2C_BASE + 0x04u, UINT32_MAX);
    mem_write32(mem, TEST_RTC_I2C_BASE + 0x08u, UINT32_MAX);
    mem_write32(mem, TEST_RTC_I2C_BASE + 0x0Cu, UINT32_MAX);
    mem_write32(mem, TEST_RTC_I2C_BASE + 0x10u, UINT32_MAX);
    mem_write32(mem, TEST_RTC_I2C_BASE + 0x1Cu, 0xA5u);
    mem_write32(mem, TEST_RTC_I2C_BASE + 0x20u, UINT32_MAX);
    mem_write32(mem, TEST_RTC_I2C_BASE + 0x28u, UINT32_MAX);
    mem_write32(mem, TEST_RTC_I2C_BASE + 0x30u, UINT32_MAX);
    mem_write32(mem, TEST_RTC_I2C_BASE + 0x38u, UINT32_MAX);
    mem_write32(mem, TEST_RTC_I2C_BASE + 0x40u, UINT32_MAX);
    mem_write32(mem, TEST_RTC_I2C_BASE + 0x44u, UINT32_MAX);
    mem_write32(mem, TEST_RTC_I2C_BASE + 0x48u, UINT32_MAX);
    mem_write32(mem, TEST_RTC_I2C_BASE + 0x84u, UINT32_MAX);

    ASSERT_EQ(mem_read32(mem, TEST_RTC_I2C_BASE + 0x00u), 0x0007FFFFu);
    ASSERT_EQ(mem_read32(mem, TEST_RTC_I2C_BASE + 0x04u), 0x000000F3u);
    ASSERT_EQ(mem_read32(mem, TEST_RTC_I2C_BASE + 0x08u), 0x7E00007Fu);
    ASSERT_EQ(mem_read32(mem, TEST_RTC_I2C_BASE + 0x0Cu), 0x000FFFFFu);
    ASSERT_EQ(mem_read32(mem, TEST_RTC_I2C_BASE + 0x10u), 0x80007FFFu);
    ASSERT_EQ(mem_read32(mem, TEST_RTC_I2C_BASE + 0x1Cu), 0u);
    ASSERT_EQ(mem_read32(mem, TEST_RTC_I2C_BASE + 0x20u), 0u);
    ASSERT_EQ(mem_read32(mem, TEST_RTC_I2C_BASE + 0x28u), 0x000001E0u);
    ASSERT_EQ(mem_read32(mem, TEST_RTC_I2C_BASE + 0x30u), 0x000FFFFFu);
    ASSERT_EQ(mem_read32(mem, TEST_RTC_I2C_BASE + 0x38u), 0x000FFFFFu);
    ASSERT_EQ(mem_read32(mem, TEST_RTC_I2C_BASE + 0x40u), 0x000FFFFFu);
    ASSERT_EQ(mem_read32(mem, TEST_RTC_I2C_BASE + 0x44u), 0x000FFFFFu);
    ASSERT_EQ(mem_read32(mem, TEST_RTC_I2C_BASE + 0x48u), 0x80003FFFu);
    ASSERT_EQ(mem_read32(mem, TEST_RTC_I2C_BASE + 0x84u), 0x80003FFFu);
    ASSERT_EQ(mem_read32(mem, TEST_RTC_I2C_BASE + 0x34u), 0u);
    ASSERT_EQ(mem_read32(mem, TEST_RTC_I2C_BASE + 0x3Cu), 0u);
    ASSERT_EQ(periph_unhandled_count(p), 0);

    periph_destroy(p);
    mem_destroy(mem);
}

typedef struct {
    unsigned count;
    unsigned writes;
    unsigned reads;
    int last_port;
    uint8_t last_address;
    uint8_t last_data[2];
    uint16_t last_len;
} test_rtc_i2c_events_t;

static void capture_rtc_i2c_event(const sbx_event_t *event, void *ctx) {
    if (event->kind != SBX_EV_I2C_XFER)
        return;
    test_rtc_i2c_events_t *capture = ctx;
    capture->count++;
    capture->writes += event->i2c_xfer.read ? 0u : 1u;
    capture->reads += event->i2c_xfer.read ? 1u : 0u;
    capture->last_port = event->i2c_xfer.port;
    capture->last_address = event->i2c_xfer.addr;
    capture->last_len = event->i2c_xfer.len;
    size_t count = event->i2c_xfer.len < sizeof(capture->last_data) ?
                   event->i2c_xfer.len : sizeof(capture->last_data);
    if (count != 0)
        memcpy(capture->last_data, event->i2c_xfer.data, count);
}

static uint32_t test_rtc_i2c_control(bool write, unsigned selector,
                                     uint8_t subaddress, uint8_t value,
                                     unsigned low_bit, unsigned high_bit) {
    return (uint32_t)subaddress | ((uint32_t)value << 8u) |
           ((low_bit & 7u) << 16u) | ((high_bit & 7u) << 19u) |
           ((selector & 0xFu) << 22u) | (write ? 1u << 27u : 0u);
}

static void test_rtc_i2c_start(xtensa_mem_t *mem, uint32_t control) {
    mem_write32(mem, TEST_SENS_BASE + 0x50u, (1u << 29u) | control);
    mem_write32(mem, TEST_SENS_BASE + 0x50u,
                (1u << 29u) | (1u << 28u) | control);
}

TEST(rtc_i2c_sens_master_read_write_nack_and_timeout) {
    xtensa_mem_t *mem = mem_create();
    esp32_periph_t *p = periph_create(mem);
    test_i2c_device_t device = {0};
    test_rtc_i2c_events_t events = {0};
    sbx_events_set_sink(capture_rtc_i2c_event, &events);

    ASSERT_EQ(periph_i2c_attach_device(p, PERIPH_I2C_PORT_RTC, 0x34u,
                                       test_i2c_device, &device), 0);
    ASSERT_EQ(periph_i2c_attach_device(p, 3, 0x34u,
                                       test_i2c_device, &device), -1);
    ASSERT_EQ(periph_i2c_attach_device(p, PERIPH_I2C_PORT_RTC, 0xFFu,
                                       test_i2c_device, &device), -1);

    mem_write32(mem, TEST_RTC_I2C_BASE + 0x04u, 1u << 4u);
    mem_write32(mem, TEST_RTC_I2C_BASE + 0x28u, 0x1E0u);

    /* The four packed SENS registers expose all eight ULP address selectors. */
    for (unsigned pair = 0u; pair < 4u; pair++)
        mem_write32(mem, TEST_SENS_BASE + 0x3Cu + pair * 4u,
                    (0x34u << 11u) | 0x34u);
    for (unsigned selector = 0u; selector < 8u; selector++)
        test_rtc_i2c_start(mem, test_rtc_i2c_control(
            false, selector, 0u, 0u, 0u, 0u));
    ASSERT_EQ(device.calls, 8);
    ASSERT_EQ(events.count, 16u);
    memset(&device, 0, sizeof(device));
    memset(&events, 0, sizeof(events));
    mem_write32(mem, TEST_RTC_I2C_BASE + 0x24u, 0x1F0u);
    /* Selector 6 occupies bits 21:11; selector 7 occupies bits 10:0. */
    mem_write32(mem, TEST_SENS_BASE + 0x48u,
                (0x34u << 11u) | 0x55u);

    uint32_t write_control = test_rtc_i2c_control(
        true, 6u, 0x20u, 0xA5u, 0u, 7u);
    test_rtc_i2c_start(mem, write_control);
    ASSERT_EQ(device.calls, 1);
    ASSERT_EQ(device.last_port, PERIPH_I2C_PORT_RTC);
    ASSERT_EQ(device.last_address, 0x34u);
    ASSERT_EQ(device.last_write_len, 2u);
    ASSERT_EQ(device.last_write[0], 0x20u);
    ASSERT_EQ(device.last_write[1], 0xA5u);
    ASSERT_EQ(device.regs[0x20], 0xA5u);
    ASSERT_EQ(events.count, 1u);
    ASSERT_EQ(events.last_port, PERIPH_I2C_PORT_RTC);
    ASSERT_EQ(events.last_address, 0x34u);
    ASSERT_EQ(events.last_len, 2u);
    ASSERT_EQ(mem_read32(mem, TEST_SENS_BASE + 0x48u) & (1u << 30u),
              1u << 30u);
    ASSERT_EQ(mem_read32(mem, TEST_RTC_I2C_BASE + 0x20u), 0x60u);
    ASSERT_EQ(mem_read32(mem, TEST_RTC_I2C_BASE + 0x2Cu), 0xC0u);
    ASSERT_EQ(mem_read32(mem, TEST_RTC_I2C_BASE + 0x08u) & 0x51u, 0x40u);

    uint32_t read_control = test_rtc_i2c_control(
        false, 6u, 0x20u, 0u, 0u, 0u);
    test_rtc_i2c_start(mem, read_control);
    ASSERT_EQ(device.calls, 2);
    ASSERT_EQ(device.last_write_len, 1u);
    ASSERT_EQ(device.last_read_len, 1u);
    ASSERT_EQ(mem_read32(mem, TEST_RTC_I2C_BASE + 0x1Cu), 0xA5u);
    ASSERT_EQ((mem_read32(mem, TEST_SENS_BASE + 0x48u) >> 22u) & 0xFFu,
              0xA5u);
    ASSERT_EQ(events.count, 3u);
    ASSERT_EQ(events.writes, 2u);
    ASSERT_EQ(events.reads, 1u);
    ASSERT_EQ(events.last_data[0], 0xA5u);

    /* Partial writes place the low bits of DATA into the selected range. */
    test_rtc_i2c_start(mem, test_rtc_i2c_control(
        true, 6u, 0x21u, 5u, 2u, 4u));
    ASSERT_EQ(device.calls, 3);
    ASSERT_EQ(device.regs[0x21], 0x14u);
    ASSERT_EQ(device.last_write[1], 0x14u);
    ASSERT_EQ(events.count, 4u);

    /* Clear bits are shifted one position above the raw event bitmap. */
    mem_write32(mem, TEST_RTC_I2C_BASE + 0x24u, 0xC0u);
    ASSERT_EQ(mem_read32(mem, TEST_RTC_I2C_BASE + 0x20u), 0u);
    ASSERT_EQ(mem_read32(mem, TEST_RTC_I2C_BASE + 0x2Cu), 0u);

    /* Selector 7 names an unattached address: data floats high and ACK_VAL
     * reports the NACK while the fixed transaction still reaches STOP. */
    test_rtc_i2c_start(mem, test_rtc_i2c_control(
        false, 7u, 0x01u, 0u, 0u, 0u));
    ASSERT_EQ(device.calls, 3);
    ASSERT_EQ(mem_read32(mem, TEST_RTC_I2C_BASE + 0x1Cu), 0xFFu);
    ASSERT_TRUE(mem_read32(mem, TEST_RTC_I2C_BASE + 0x08u) & 1u);
    ASSERT_TRUE(mem_read32(mem, TEST_SENS_BASE + 0x48u) & (1u << 30u));
    ASSERT_EQ(events.count, 6u);
    ASSERT_EQ(events.last_address, 0x55u);

    /* A software launch while master mode is disabled times out without
     * touching the external target; timeout raw bit 7 maps to status bit 8. */
    mem_write32(mem, TEST_RTC_I2C_BASE + 0x24u, 0x1F0u);
    mem_write32(mem, TEST_RTC_I2C_BASE + 0x04u, 0u);
    test_rtc_i2c_start(mem, read_control);
    ASSERT_EQ(device.calls, 3);
    ASSERT_EQ(events.count, 6u);
    ASSERT_TRUE(mem_read32(mem, TEST_RTC_I2C_BASE + 0x08u) & (1u << 2u));
    ASSERT_EQ(mem_read32(mem, TEST_RTC_I2C_BASE + 0x20u), 0x80u);
    ASSERT_EQ(mem_read32(mem, TEST_RTC_I2C_BASE + 0x2Cu), 0x100u);
    mem_write32(mem, TEST_RTC_I2C_BASE + 0x24u, 0x100u);
    ASSERT_EQ(mem_read32(mem, TEST_RTC_I2C_BASE + 0x20u), 0u);
    ASSERT_EQ(periph_unhandled_count(p), 0);

    sbx_events_set_sink(NULL, NULL);
    periph_destroy(p);
    mem_destroy(mem);
}

typedef struct {
    int count;
    int last_source;
} test_irq_dispatch_t;

static void test_irq_dispatch(void *ctx, int source) {
    test_irq_dispatch_t *dispatch = ctx;
    dispatch->count++;
    dispatch->last_source = source;
}

TEST(irq_dispatch_observes_only_rising_edges) {
    xtensa_mem_t *mem = mem_create();
    esp32_periph_t *p = periph_create(mem);
    test_irq_dispatch_t dispatch = {0};

    ASSERT_EQ(periph_set_irq_dispatch(p, 49, test_irq_dispatch, &dispatch),
              0);
    periph_assert_interrupt(p, 49);
    ASSERT_EQ(dispatch.count, 1);
    ASSERT_EQ(dispatch.last_source, 49);
    ASSERT_TRUE(periph_interrupt_pending(p, 49));
    periph_assert_interrupt(p, 49);
    ASSERT_EQ(dispatch.count, 1);
    periph_deassert_interrupt(p, 49);
    ASSERT_FALSE(periph_interrupt_pending(p, 49));
    periph_assert_interrupt(p, 49);
    ASSERT_EQ(dispatch.count, 2);

    ASSERT_EQ(periph_set_irq_dispatch(p, 49, NULL, NULL), 0);
    periph_deassert_interrupt(p, 49);
    periph_assert_interrupt(p, 49);
    ASSERT_EQ(dispatch.count, 2);
    ASSERT_EQ(periph_set_irq_dispatch(p, -1, test_irq_dispatch, &dispatch),
              -1);
    ASSERT_EQ(periph_set_irq_dispatch(p, 71, test_irq_dispatch, &dispatch),
              -1);

    periph_destroy(p);
    mem_destroy(mem);
}

/* ESP32 SPI1 flash-controller register subset used by ESP-IDF's memspi
 * driver. */
#define TEST_SPI1_BASE       0x3FF42000u
#define TEST_SPI_CMD_REG     (TEST_SPI1_BASE + 0x00)
#define TEST_SPI_ADDR_REG    (TEST_SPI1_BASE + 0x04)
#define TEST_SPI_STATUS_REG  (TEST_SPI1_BASE + 0x10)
#define TEST_SPI_USER1_REG   (TEST_SPI1_BASE + 0x20)
#define TEST_SPI_USER2_REG   (TEST_SPI1_BASE + 0x24)
#define TEST_SPI_MOSI_DLEN   (TEST_SPI1_BASE + 0x28)
#define TEST_SPI_MISO_DLEN   (TEST_SPI1_BASE + 0x2C)
#define TEST_SPI_W0_REG      (TEST_SPI1_BASE + 0x80)

typedef struct {
    int count;
    uint32_t addr;
    size_t len;
} flash_code_invalidate_capture_t;

static void test_flash_code_invalidate(void *ctx, uint32_t addr, size_t len) {
    flash_code_invalidate_capture_t *capture = ctx;
    capture->count++;
    capture->addr = addr;
    capture->len = len;
}

static uint8_t test_flash_status(xtensa_mem_t *mem) {
    mem_write32(mem, TEST_SPI_USER2_REG, 0x05);       /* RDSR1 */
    mem_write32(mem, TEST_SPI_CMD_REG, 1u << 18);    /* USR */
    return (uint8_t)mem_read32(mem, TEST_SPI_W0_REG);
}

TEST(spi_flash_write_enable_latch) {
    xtensa_mem_t *mem = mem_create();
    esp32_periph_t *p = periph_create(mem);

    ASSERT_EQ(test_flash_status(mem) & (1u << 1), 0u);

    /* ESP-IDF's ESP32 HAL uses the dedicated command bits for WREN/WRDI,
     * then verifies the result with the generic 0x05 status command. */
    mem_write32(mem, TEST_SPI_CMD_REG, 1u << 30);     /* FLASH_WREN */
    ASSERT_EQ(test_flash_status(mem) & (1u << 1), 1u << 1);
    mem_write32(mem, TEST_SPI_CMD_REG, 1u << 29);     /* FLASH_WRDI */
    ASSERT_EQ(test_flash_status(mem) & (1u << 1), 0u);

    /* Generic USER commands must expose the same device latch. */
    mem_write32(mem, TEST_SPI_USER2_REG, 0x06);       /* WREN */
    mem_write32(mem, TEST_SPI_CMD_REG, 1u << 18);
    ASSERT_EQ(test_flash_status(mem) & (1u << 1), 1u << 1);
    mem_write32(mem, TEST_SPI_USER2_REG, 0x04);       /* WRDI */
    mem_write32(mem, TEST_SPI_CMD_REG, 1u << 18);
    ASSERT_EQ(test_flash_status(mem) & (1u << 1), 0u);

    periph_destroy(p);
    mem_destroy(mem);
}

TEST(spi_flash_program_erase_require_write_enable) {
    xtensa_mem_t *mem = mem_create();
    esp32_periph_t *p = periph_create(mem);
    xtensa_cpu_t cpu;
    xtensa_cpu_init(&cpu);
    cpu.mem = mem;
    flash_code_invalidate_capture_t capture = {0};
    cpu.code_invalidate = test_flash_code_invalidate;
    cpu.code_invalidate_ctx = &capture;
    periph_attach_cpus(p, &cpu, NULL);
    const uint32_t off = 0x9000;

    mem->flash_data[off] = 0xF0;
    mem->flash_insn[off] = 0xF0;
    /* IRAM0 entry 77 aliases physical flash page zero. Programming that
     * page must invalidate translated code at its virtual address. */
    mem_write32(mem, 0x3FF10000u + 77u * 4u, 0u);
    capture.count = 0;
    mem_write32(mem, TEST_SPI_ADDR_REG, off);
    mem_write32(mem, TEST_SPI_MOSI_DLEN, 7);          /* one byte */
    mem_write32(mem, TEST_SPI_W0_REG, 0xAA);

    /* A page-program command without WEL is ignored. */
    mem_write32(mem, TEST_SPI_CMD_REG, 1u << 25);     /* FLASH_PP */
    ASSERT_EQ(mem->flash_data[off], 0xF0);
    ASSERT_EQ(mem->flash_insn[off], 0xF0);
    ASSERT_EQ(capture.count, 0);

    mem_write32(mem, TEST_SPI_CMD_REG, 1u << 30);     /* FLASH_WREN */
    mem_write32(mem, TEST_SPI_CMD_REG, 1u << 25);     /* FLASH_PP */
    ASSERT_EQ(mem->flash_data[off], 0xA0);            /* NOR: 0xF0 & 0xAA */
    ASSERT_EQ(mem->flash_insn[off], 0xA0);
    ASSERT_EQ(capture.count, 1);
    ASSERT_EQ(capture.addr, 0x400D0000u);
    ASSERT_EQ(capture.len, 0x10000u);
    ASSERT_EQ(test_flash_status(mem) & (1u << 1), 0u); /* PP clears WEL */

    /* Erase has the same WEL requirement and restores the sector to 0xFF. */
    mem_write32(mem, TEST_SPI_CMD_REG, 1u << 24);     /* FLASH_SE, no WEL */
    ASSERT_EQ(mem->flash_data[off], 0xA0);
    ASSERT_EQ(mem->flash_insn[off], 0xA0);
    ASSERT_EQ(capture.count, 1);
    mem_write32(mem, TEST_SPI_CMD_REG, 1u << 30);     /* FLASH_WREN */
    mem_write32(mem, TEST_SPI_CMD_REG, 1u << 24);     /* FLASH_SE */
    ASSERT_EQ(mem->flash_data[off], 0xFF);
    ASSERT_EQ(mem->flash_insn[off], 0xFF);
    ASSERT_EQ(capture.count, 2);
    ASSERT_EQ(test_flash_status(mem) & (1u << 1), 0u); /* erase clears WEL */

    periph_destroy(p);
    mem_destroy(mem);
}

TEST(flash_mmu_maps_complete_pages_and_all_instruction_buses) {
    xtensa_mem_t *mem = mem_create();
    esp32_periph_t *p = periph_create(mem);
    xtensa_cpu_t cpu0, cpu1;
    xtensa_cpu_init(&cpu0);
    xtensa_cpu_init(&cpu1);
    cpu0.mem = mem;
    cpu1.mem = mem;
    flash_code_invalidate_capture_t capture = {0};
    cpu0.code_invalidate = test_flash_code_invalidate;
    cpu0.code_invalidate_ctx = &capture;
    cpu1.code_invalidate = test_flash_code_invalidate;
    cpu1.code_invalidate_ctx = &capture;
    periph_attach_cpus(p, &cpu0, &cpu1);

    const uint32_t pro = 0x3FF10000u;
    const uint32_t app = 0x3FF12000u;
    const uint32_t data_entry = 5u;
    const uint32_t data_vaddr = 0x3F450000u;
    mem->flash_data[0x20000u] = 0x12;
    mem->flash_data[0x21234u] = 0x34;
    mem->flash_data[0x2FFFCu] = 0x56;
    mem->flash_data[0x30000u] = 0xA1;
    mem->flash_data[0x31234u] = 0xA2;

    /* One hardware entry backs sixteen host pages, not just its first 4 KiB. */
    mem_write32(mem, pro + data_entry * 4u, 2u);
    ASSERT_EQ(mem_read32(mem, pro + data_entry * 4u), 2u);
    ASSERT_EQ(mem_read8(mem, data_vaddr), 0x12);
    ASSERT_EQ(mem_read8(mem, data_vaddr + 0x1234u), 0x34);
    ASSERT_EQ(mem_read8(mem, data_vaddr + 0xFFFCu), 0x56);

    /* The shared emulator map follows the latest valid core table, then
     * falls back to the other core until both entries are invalid. */
    mem_write32(mem, app + data_entry * 4u, 3u);
    ASSERT_EQ(mem_read8(mem, data_vaddr), 0xA1);
    ASSERT_EQ(mem_read8(mem, data_vaddr + 0x1234u), 0xA2);
    mem_write32(mem, app + data_entry * 4u, 0x100u);
    ASSERT_EQ(mem_read8(mem, data_vaddr), 0x12);
    mem_write32(mem, pro + data_entry * 4u, 0x100u);
    ASSERT_TRUE(mem_get_ptr(mem, data_vaddr) == NULL);

    /* Entries below IRAM0's first usable page must not overwrite ROM/IRAM. */
    const uint8_t *rom_page = mem_get_ptr(mem, 0x40000000u);
    mem_write32(mem, pro + 64u * 4u, 2u);
    ASSERT_TRUE(mem_get_ptr(mem, 0x40000000u) == rom_page);

    uint16_t nop_n = narrow(0xD, 15, 0, 3);
    memcpy(mem->flash_insn + 0x30000u, &nop_n, sizeof(nop_n));
    mem->flash_insn[0x3FFFFu] = 0xB1;
    mem->flash_insn[0x40000u] = 0xC2;
    mem->flash_insn[0x50000u] = 0xD3;
    capture.count = 0;

    mem_write32(mem, pro + 128u * 4u, 3u); /* IRAM1 */
    ASSERT_TRUE(mem_get_ptr(mem, 0x40400000u) == mem->flash_insn + 0x30000u);
    ASSERT_EQ(mem_read8(mem, 0x4040FFFFu), 0xB1);
    ASSERT_EQ(capture.count, 1); /* shared predecode/JIT invalidated once */
    ASSERT_EQ(capture.addr, 0x40400000u);

    /* The upper bus is executable, not merely addressable. */
    cpu0.pc = 0x40400000u;
    cpu0._pc_written = true;
    ASSERT_EQ(xtensa_step(&cpu0), 0);
    ASSERT_EQ(cpu0.pc, 0x40400002u);
    ASSERT_FALSE(cpu0.exception);

    mem_write32(mem, pro + 192u * 4u, 4u); /* IROM0 first page */
    ASSERT_TRUE(mem_get_ptr(mem, 0x40800000u) == mem->flash_insn + 0x40000u);
    ASSERT_EQ(mem_read8(mem, 0x40800000u), 0xC2);
    mem_write32(mem, pro + 255u * 4u, 5u); /* IROM0 final page */
    ASSERT_TRUE(mem_get_ptr(mem, 0x40BF0000u) == mem->flash_insn + 0x50000u);
    ASSERT_EQ(mem_read8(mem, 0x40BF0000u), 0xD3);
    ASSERT_EQ(capture.count, 3);

    /* A syntactically valid physical page beyond the emulated 4 MiB chip
     * remains visible in the raw table but cannot expose host memory. */
    mem_write32(mem, pro + 128u * 4u, 0x40u);
    ASSERT_EQ(mem_read32(mem, pro + 128u * 4u), 0x40u);
    ASSERT_TRUE(mem_get_ptr(mem, 0x40400000u) == NULL);

    periph_destroy(p);
    mem_destroy(mem);
}

TEST(flash_mmu_exposes_full_multicore_copy_windows) {
    xtensa_mem_t *mem = mem_create();
    esp32_periph_t *p = periph_create(mem);
    const uint32_t pro = 0x3FF10000u;
    const uint32_t app = 0x3FF12000u;

    /* ESP-IDF 5.5 copies all 2048 words when restoring APP CPU cache state,
     * not only the first 256 entries used for flash XIP mappings. */
    ASSERT_EQ(mem_read32(mem, pro), 0x100u);
    ASSERT_EQ(mem_read32(mem, pro + 2047u * 4u), 0x100u);
    ASSERT_EQ(mem_read32(mem, app), 0x100u);
    ASSERT_EQ(mem_read32(mem, app + 2047u * 4u), 0x100u);

    mem_write32(mem, pro + 0u * 4u, 0x12u);
    mem_write32(mem, pro + 255u * 4u, 0x34u);
    mem_write32(mem, pro + 256u * 4u, 0x56u);
    mem_write32(mem, pro + 1152u * 4u, 0x78u);
    mem_write32(mem, pro + 2047u * 4u, UINT32_MAX);

    for (uint32_t entry = 0; entry < 2048u; entry++)
        mem_write32(mem, app + entry * 4u,
                    mem_read32(mem, pro + entry * 4u));

    ASSERT_EQ(mem_read32(mem, app + 0u * 4u), 0x12u);
    ASSERT_EQ(mem_read32(mem, app + 255u * 4u), 0x34u);
    ASSERT_EQ(mem_read32(mem, app + 256u * 4u), 0x56u);
    ASSERT_EQ(mem_read32(mem, app + 1152u * 4u), 0x78u);
    ASSERT_EQ(mem_read32(mem, app + 2047u * 4u), 0x1FFu);
    ASSERT_EQ(periph_unhandled_count(p), 0);

    periph_destroy(p);
    mem_destroy(mem);
}

TEST(spi_flash_dual_io_mode_bits_are_not_address_bits) {
    xtensa_mem_t *mem = mem_create();
    esp32_periph_t *p = periph_create(mem);
    const uint32_t off = 0x3100FC;

    mem->flash_data[off] = 0xC9;
    mem->flash_data[off + 1] = 0x04;

    /* ESP-IDF's ESP32 DIO read uses a 28-bit wire phase: a 24-bit address
     * followed by four all-one mode bits.  SPI_ADDR contains the address
     * left-aligned by eight, not by four. */
    mem_write32(mem, TEST_SPI_USER1_REG, 27u << 26);
    mem_write32(mem, TEST_SPI_ADDR_REG, (off << 8) | 0xFFu);
    mem_write32(mem, TEST_SPI_USER2_REG, 0xBB);       /* DIO read */
    mem_write32(mem, TEST_SPI_MISO_DLEN, 15);        /* two bytes */
    mem_write32(mem, TEST_SPI_CMD_REG, 1u << 18);    /* USR */
    ASSERT_EQ(mem_read32(mem, TEST_SPI_W0_REG) & 0xFFFFu, 0x04C9u);

    periph_destroy(p);
    mem_destroy(mem);
}

TEST(wifi_mac_init_ready_handshake) {
    xtensa_mem_t *mem = mem_create();
    esp32_periph_t *p = periph_create(mem);
    const uint32_t mac_init_ctrl = 0x3FF73D24u;

    ASSERT_EQ(mem_read32(mem, mac_init_ctrl), 0u);
    mem_write32(mem, mac_init_ctrl, 1u << 1);
    ASSERT_EQ(mem_read32(mem, mac_init_ctrl) & 3u, 3u);

    periph_destroy(p);
    mem_destroy(mem);
}

TEST(radio_phy_calibration_register_files) {
    xtensa_mem_t *mem = mem_create();
    esp32_periph_t *p = periph_create(mem);

    static const struct {
        uint32_t addr;
        uint32_t value;
    } registers[] = {
        {0x3FF450F0u, 0x00000600u}, /* FE2 TX interpolation control */
        {0x3FF46090u, 0x00000030u}, /* FE general control */
        {0x3FF4E0C8u, 0x00000200u}, /* private PHY calibration data */
        {0x3FF51020u, 0x10203040u}, /* BT controller */
        {0x3FF5CC04u, 0x50607080u}, /* NRX private register space */
        {0x3FF5D040u, 0x90A0B0C0u}, /* WiFi baseband */
        {0x3FF7120Cu, 0xA5A5A5A4u}, /* BT private reference control */
        {0x3FF740B8u, 0xCAFEBABEu}, /* second WiFi MAC page */
    };

    for (size_t i = 0; i < sizeof(registers) / sizeof(registers[0]); i++)
        mem_write32(mem, registers[i].addr, registers[i].value);
    for (size_t i = 0; i < sizeof(registers) / sizeof(registers[0]); i++)
        ASSERT_EQ(mem_read32(mem, registers[i].addr), registers[i].value);

    /* Same offsets in distinct hardware blocks must not alias. */
    mem_write32(mem, 0x3FF45004u, 0x11111111u);
    mem_write32(mem, 0x3FF46004u, 0x22222222u);
    mem_write32(mem, 0x3FF75020u, 0x98000000u);
    ASSERT_EQ(mem_read32(mem, 0x3FF45004u), 0x11111111u);
    ASSERT_EQ(mem_read32(mem, 0x3FF46004u), 0x22222222u);
    ASSERT_EQ(mem_read32(mem, 0x3FF75020u), 0x98000000u);

    /* The indexed calibration command is a self-clearing trigger. */
    mem_write32(mem, 0x3FF4E0C4u, 0x200u);
    ASSERT_EQ(mem_read32(mem, 0x3FF4E0C4u), 0u);

    /* The WDEV random source advances independently of the shadow files. */
    uint32_t random_a = mem_read32(mem, 0x3FF75144u);
    uint32_t random_b = mem_read32(mem, 0x3FF75144u);
    ASSERT_TRUE(random_a != random_b);
    ASSERT_EQ(periph_unhandled_count(p), 0);

    periph_destroy(p);
    mem_destroy(mem);
}

typedef struct {
    int down;
    int x;
    int y;
} test_touch_state_t;

static int test_touch_read(int *x, int *y, void *ctx) {
    test_touch_state_t *touch = ctx;
    *x = touch->x;
    *y = touch->y;
    return touch->down;
}

/* Match Arduino-ESP32's MSB-first transfer16 byte swaps around W0. */
static uint16_t test_spi_transfer16(xtensa_mem_t *mem, uint8_t next_cmd) {
    const uint32_t spi = 0x3FF65000u;
    mem_write32(mem, spi + 0x28, 15);                 /* MOSI_DLEN */
    mem_write32(mem, spi + 0x2C, 15);                 /* MISO_DLEN */
    mem_write32(mem, spi + 0x80, (uint32_t)next_cmd << 8);
    mem_write32(mem, spi + 0x00, 1u << 18);           /* CMD.USR */
    uint16_t fifo = (uint16_t)mem_read32(mem, spi + 0x80);
    return (uint16_t)((fifo << 8) | (fifo >> 8));
}

static void test_gp_spi_bytes(xtensa_mem_t *mem, uint32_t spi,
                              const uint8_t *tx, uint8_t *rx, int len) {
    mem_write32(mem, spi + 0x1C, (1u << 27) | (1u << 28)); /* MOSI+MISO */
    mem_write32(mem, spi + 0x28, (uint32_t)(len * 8 - 1));
    mem_write32(mem, spi + 0x2C, (uint32_t)(len * 8 - 1));
    for (int off = 0; off < len; off += 4) {
        uint32_t word = 0;
        for (int byte = 0; byte < 4 && off + byte < len; byte++)
            word |= (uint32_t)(tx ? tx[off + byte] : 0xFF) << (byte * 8);
        mem_write32(mem, spi + 0x80 + (uint32_t)off, word);
    }
    mem_write32(mem, spi + 0x00, 1u << 18);           /* CMD.USR */
    if (rx) {
        for (int off = 0; off < len; off += 4) {
            uint32_t word = mem_read32(mem, spi + 0x80 + (uint32_t)off);
            for (int byte = 0; byte < 4 && off + byte < len; byte++)
                rx[off + byte] = (uint8_t)(word >> (byte * 8));
        }
    }
}

static void test_spi_dma_desc(xtensa_mem_t *mem, uint32_t desc,
                              uint32_t buf, uint16_t size, uint16_t len,
                              int eof, uint32_t next) {
    uint32_t ctrl = (uint32_t)(size & 0x0FFFu) |
                    ((uint32_t)(len & 0x0FFFu) << 12) |
                    (eof ? 1u << 30 : 0) | (1u << 31);
    mem_write32(mem, desc, ctrl);
    mem_write32(mem, desc + 4u, buf);
    mem_write32(mem, desc + 8u, next);
}

static void test_gpio_level(xtensa_mem_t *mem, int pin, int high) {
    const uint32_t gpio = 0x3FF44000u;
    uint32_t bit = 1u << (pin < 32 ? pin : pin - 32);
    uint32_t set = pin < 32 ? 0x08u : 0x14u;
    uint32_t clear = pin < 32 ? 0x0Cu : 0x18u;
    mem_write32(mem, gpio + (high ? set : clear), bit);
}

static void test_gpio_route(xtensa_mem_t *mem, int pin, int signal) {
    mem_write32(mem, 0x3FF44000u + 0x530u + (uint32_t)pin * 4,
                (uint32_t)signal);
}

static void test_sd_command_bytes(uint8_t out[6], uint8_t command,
                                  uint32_t argument) {
    out[0] = 0x40u | command;
    out[1] = (uint8_t)(argument >> 24);
    out[2] = (uint8_t)(argument >> 16);
    out[3] = (uint8_t)(argument >> 8);
    out[4] = (uint8_t)argument;
    out[5] = 0x01;
}

static uint8_t test_sd_command(xtensa_mem_t *mem, uint32_t spi,
                               uint8_t command, uint32_t argument) {
    uint8_t packet[6];
    uint8_t idle = 0xFF;
    uint8_t response = 0xFF;
    test_sd_command_bytes(packet, command, argument);
    test_gp_spi_bytes(mem, spi, packet, NULL, sizeof(packet));
    test_gp_spi_bytes(mem, spi, &idle, &response, 1);
    return response;
}

TEST(xpt2046_pipelined_conversions) {
    xtensa_mem_t *mem = mem_create();
    esp32_periph_t *p = periph_create(mem);
    test_touch_state_t touch = { .down = 1, .x = 240, .y = 60 };
    spi_display_config_t cfg = {
        .dc_pin = 2,
        .display_cs_pin = 15,
        .touch_cs_pin = 33,
        .sd_cs_pin = 5,
        .fb_w = 320,
        .fb_h = 240,
        .touch_fn = test_touch_read,
        .touch_ctx = &touch,
    };
    periph_enable_spi_display(p, &cfg);

    const uint32_t spi = 0x3FF65000u;
    const uint32_t gpio = 0x3FF44000u;
    mem_write32(mem, spi + 0x1C, (1u << 27) | (1u << 28)); /* MOSI+MISO */

    /* A configured output may be asserted before its first SPI transaction.
     * Output-enable distinguishes that low CS from an untouched reset pin. */
    mem_write32(mem, gpio + 0x30, 1u << 1);           /* GPIO33 output */
    mem_write32(mem, gpio + 0x18, 1u << 1);           /* GPIO33 low */
    ASSERT_EQ(periph_gpio_output_enabled(p, 33), 1);

    /* transfer(0xB1) starts Z1. Each following transfer16 returns the
     * previous conversion and queues its command from W0's second byte. */
    mem_write32(mem, spi + 0x28, 7);
    mem_write32(mem, spi + 0x2C, 7);
    mem_write32(mem, spi + 0x80, 0xB1);
    mem_write32(mem, spi + 0x00, 1u << 18);
    ASSERT_EQ(test_spi_transfer16(mem, 0xC1) >> 3, 600); /* Z1 */
    ASSERT_EQ(test_spi_transfer16(mem, 0x91) >> 3, 3500); /* Z2 */

    int panel_x = touch.y * 239 / (cfg.fb_h - 1);
    int panel_y = (cfg.fb_w - 1 - touch.x) * 319 / (cfg.fb_w - 1);
    uint16_t expected_p_x = (uint16_t)(200 + panel_x * 3500 / 239);
    uint16_t expected_p_y = (uint16_t)(240 + panel_y * 3560 / 319);
    ASSERT_EQ(test_spi_transfer16(mem, 0xD1) >> 3, expected_p_x); /* 0x91 */
    ASSERT_EQ(test_spi_transfer16(mem, 0x91) >> 3, expected_p_y); /* 0xD1 */

    /* Releasing the host touch makes subsequent pressure reads zero. */
    touch.down = 0;
    ASSERT_EQ(test_spi_transfer16(mem, 0xB1) >> 3, 0);          /* prior 0x91 */
    ASSERT_EQ(test_spi_transfer16(mem, 0xC1) >> 3, 0);          /* new Z1 */
    ASSERT_EQ(test_spi_transfer16(mem, 0x91) >> 3, 4095);       /* new Z2 */

    periph_destroy(p);
    mem_destroy(mem);
}

TEST(gp_spi_matrix_routing_and_hardware_cs) {
    const uint32_t spi2 = 0x3FF64000u;
    uint16_t framebuffer[320 * 240] = {0};
    xtensa_mem_t *mem = mem_create();
    esp32_periph_t *p = periph_create(mem);
    spi_display_config_t cfg = {
        .dc_pin = 2,
        .display_cs_pin = 15,
        .display_sck_pin = 14,
        .touch_cs_pin = 33,
        .touch_sck_pin = 25,
        .sd_cs_pin = 5,
        .sd_sck_pin = 18,
        .framebuf = framebuffer,
        .fb_w = 320,
        .fb_h = 240,
    };
    periph_enable_spi_display(p, &cfg);

    /* Marauder's panel and SD card both use SPI2, while touch uses SPI3.
     * First let the software-controlled CS pins be observed inactive. */
    test_gpio_route(mem, 14, 8);       /* HSPICLK -> display SCLK */
    test_gpio_route(mem, 18, 8);       /* HSPICLK -> SD SCLK */
    test_gpio_route(mem, 25, 63);      /* VSPICLK -> touch SCLK */
    test_gpio_route(mem, 15, 256);     /* display CS is plain GPIO */
    test_gpio_route(mem, 5, 256);      /* SD CS initially plain GPIO */
    test_gpio_level(mem, 15, 1);
    test_gpio_level(mem, 5, 1);
    const uint8_t idle = 0xFF;
    test_gp_spi_bytes(mem, spi2, &idle, NULL, 1);

    /* The SD stack then hands GPIO5 to SPI2 CS0.  Its stale GPIO latch is
     * low, but PIN.CS0_DIS says this panel transfer must not select SD. */
    test_gpio_route(mem, 5, 11);       /* HSPICS0 -> SD CS */
    test_gpio_level(mem, 15, 0);
    test_gpio_level(mem, 5, 0);
    mem_write32(mem, spi2 + 0x34, 0x7); /* all hardware CS lines disabled */
    ASSERT_EQ(mem_read32(mem, spi2 + 0x34), 0x7);

    const uint8_t caset_cmd[] = {0x2A};
    const uint8_t paset_cmd[] = {0x2B};
    const uint8_t ramwr_cmd[] = {0x2C};
    const uint8_t origin[] = {0, 0, 0, 0};
    const uint8_t red[] = {0xF8, 0x00};
    test_gpio_level(mem, 2, 0);
    test_gp_spi_bytes(mem, spi2, caset_cmd, NULL, sizeof(caset_cmd));
    test_gpio_level(mem, 2, 1);
    test_gp_spi_bytes(mem, spi2, origin, NULL, sizeof(origin));
    test_gpio_level(mem, 2, 0);
    test_gp_spi_bytes(mem, spi2, paset_cmd, NULL, sizeof(paset_cmd));
    test_gpio_level(mem, 2, 1);
    test_gp_spi_bytes(mem, spi2, origin, NULL, sizeof(origin));
    test_gpio_level(mem, 2, 0);
    test_gp_spi_bytes(mem, spi2, ramwr_cmd, NULL, sizeof(ramwr_cmd));
    test_gpio_level(mem, 2, 1);
    test_gp_spi_bytes(mem, spi2, red, NULL, sizeof(red));
    ASSERT_EQ(framebuffer[319], 0xF800);

    /* Enabling hardware CS0 selects the SD model even though GPIO5's output
     * latch is high.  CMD0 must return its idle-state R1 response. */
    test_gpio_level(mem, 15, 1);
    test_gpio_level(mem, 5, 1);
    mem_write32(mem, spi2 + 0x34, 0x6); /* CS0 enabled, CS1/2 disabled */
    const uint8_t cmd0[] = {0x40, 0, 0, 0, 0, 0x95};
    uint8_t response = 0xFF;
    test_gp_spi_bytes(mem, spi2, cmd0, NULL, sizeof(cmd0));
    test_gp_spi_bytes(mem, spi2, &idle, &response, 1);
    ASSERT_EQ(response, 0x01);

    periph_destroy(p);
    mem_destroy(mem);
}

TEST(gp_spi_native_iomux_routing_and_hardware_cs) {
    const uint32_t spi2 = 0x3FF64000u;
    uint16_t framebuffer[320 * 240] = {0};
    xtensa_mem_t *mem = mem_create();
    esp32_periph_t *p = periph_create(mem);
    spi_display_config_t cfg = {
        .dc_pin = 2,
        .display_cs_pin = 15,
        .display_sck_pin = 14,
        .touch_cs_pin = 33,
        .touch_sck_pin = 25,
        .sd_cs_pin = 5,
        .sd_sck_pin = 18,
        .framebuf = framebuffer,
        .fb_w = 320,
        .fb_h = 240,
    };
    periph_enable_spi_display(p, &cfg);

    /* ESP-IDF selects the native HSPI pins through IO_MUX function 1. It
     * does not program GPIO_FUNC_OUT_SEL for SCLK or hardware CS0. */
    mem_write32(mem, 0x3FF49030u, 1u << 12); /* GPIO14: HSPICLK */
    mem_write32(mem, 0x3FF4903Cu, 1u << 12); /* GPIO15: HSPICS0 */
    ASSERT_EQ(mem_read32(mem, 0x3FF49030u), 1u << 12);
    ASSERT_EQ(mem_read32(mem, 0x3FF4903Cu), 1u << 12);
    ASSERT_EQ(periph_iomux_function(p, 14), 1);
    ASSERT_EQ(periph_iomux_function(p, 15), 1);
    ASSERT_EQ(periph_iomux_function(p, 18), -1);
    mem_write32(mem, spi2 + 0x34, 0x6); /* enable hardware CS0 */

    const uint8_t caset[] = {0x2A};
    const uint8_t paset[] = {0x2B};
    const uint8_t ramwr[] = {0x2C};
    const uint8_t origin[] = {0, 0, 0, 0};
    const uint8_t green[] = {0x07, 0xE0};
    test_gpio_level(mem, 2, 0);
    test_gp_spi_bytes(mem, spi2, caset, NULL, sizeof(caset));
    test_gpio_level(mem, 2, 1);
    test_gp_spi_bytes(mem, spi2, origin, NULL, sizeof(origin));
    test_gpio_level(mem, 2, 0);
    test_gp_spi_bytes(mem, spi2, paset, NULL, sizeof(paset));
    test_gpio_level(mem, 2, 1);
    test_gp_spi_bytes(mem, spi2, origin, NULL, sizeof(origin));
    test_gpio_level(mem, 2, 0);
    test_gp_spi_bytes(mem, spi2, ramwr, NULL, sizeof(ramwr));
    test_gpio_level(mem, 2, 1);
    test_gp_spi_bytes(mem, spi2, green, NULL, sizeof(green));
    ASSERT_EQ(framebuffer[319], 0x07E0u);

    periph_destroy(p);
    mem_destroy(mem);
}

TEST(gp_spi_dma_descriptor_chain_and_interrupt) {
    const uint32_t spi2 = 0x3FF64000u;
    const uint32_t desc1 = 0x3FFB1000u;
    const uint32_t desc2 = 0x3FFB1010u;
    const uint32_t buf1 = 0x3FFB2000u;
    const uint32_t buf2 = 0x3FFB2010u;
    uint16_t framebuffer[320 * 240] = {0};
    xtensa_mem_t *mem = mem_create();
    esp32_periph_t *p = periph_create(mem);
    xtensa_cpu_t cpu0;
    xtensa_cpu_init(&cpu0); cpu0.mem = mem;
    periph_attach_cpus(p, &cpu0, NULL);
    periph_intr_matrix_set(p, 0, 8, 30); /* SPI2 -> external CPU interrupt 8 */

    spi_display_config_t cfg = {
        .dc_pin = 2,
        .display_cs_pin = 15,
        .display_sck_pin = 14,
        .touch_cs_pin = 33,
        .touch_sck_pin = 25,
        .sd_cs_pin = 5,
        .sd_sck_pin = 18,
        .framebuf = framebuffer,
        .fb_w = 320,
        .fb_h = 240,
    };
    periph_enable_spi_display(p, &cfg);
    test_gpio_route(mem, 14, 8);       /* HSPICLK -> display SCLK */
    test_gpio_route(mem, 15, 256);     /* software display CS */
    test_gpio_level(mem, 15, 1);
    const uint8_t idle = 0xFF;
    test_gp_spi_bytes(mem, spi2, &idle, NULL, 1); /* observe CS inactive */
    test_gpio_level(mem, 15, 0);

    const uint8_t caset[] = {0x2A};
    const uint8_t paset[] = {0x2B};
    const uint8_t ramwr[] = {0x2C};
    const uint8_t columns[] = {0, 0, 0, 1};
    const uint8_t row[] = {0, 0, 0, 0};
    test_gpio_level(mem, 2, 0);
    test_gp_spi_bytes(mem, spi2, caset, NULL, sizeof(caset));
    test_gpio_level(mem, 2, 1);
    test_gp_spi_bytes(mem, spi2, columns, NULL, sizeof(columns));
    test_gpio_level(mem, 2, 0);
    test_gp_spi_bytes(mem, spi2, paset, NULL, sizeof(paset));
    test_gpio_level(mem, 2, 1);
    test_gp_spi_bytes(mem, spi2, row, NULL, sizeof(row));
    test_gpio_level(mem, 2, 0);
    test_gp_spi_bytes(mem, spi2, ramwr, NULL, sizeof(ramwr));
    test_gpio_level(mem, 2, 1);

    /* Two lldesc entries carry red and green RGB565 pixels. */
    mem_write8(mem, buf1, 0xF8); mem_write8(mem, buf1 + 1u, 0x00);
    mem_write8(mem, buf2, 0x07); mem_write8(mem, buf2 + 1u, 0xE0);
    test_spi_dma_desc(mem, desc1, buf1, 2, 2, 0, desc2);
    test_spi_dma_desc(mem, desc2, buf2, 2, 2, 1, 0);

    mem_write32(mem, spi2 + 0x38, 1u << 9); /* TRANS_DONE interrupt enable */
    mem_write32(mem, spi2 + 0x110, (1u << 8) | (1u << 7) | (1u << 6));
    mem_write32(mem, spi2 + 0x1C, 1u << 27); /* MOSI data phase */
    mem_write32(mem, spi2 + 0x28, 31);       /* four bytes */
    mem_write32(mem, spi2 + 0x104,
                (desc1 & 0x000FFFFFu) | (1u << 29));
    ASSERT_EQ(mem_read32(mem, spi2 + 0x10C), 2u);
    mem_write32(mem, spi2, 1u << 18);        /* CMD.USR */

    ASSERT_EQ(framebuffer[319], 0xF800u);
    ASSERT_EQ(framebuffer[639], 0x07E0u);
    ASSERT_EQ(mem_read32(mem, desc1) & (1u << 31), 0u);
    ASSERT_EQ(mem_read32(mem, desc2) & (1u << 31), 0u);
    ASSERT_EQ(mem_read32(mem, spi2 + 0x114) & 0x1C0u, 0x1C0u);
    ASSERT_EQ(mem_read32(mem, spi2 + 0x118) & 0x1C0u, 0x1C0u);
    ASSERT_EQ(mem_read32(mem, spi2 + 0x138), desc2);
    ASSERT_EQ(mem_read32(mem, spi2 + 0x144), buf2);
    ASSERT_EQ(mem_read32(mem, spi2 + 0x104) & (1u << 29), 0u);
    ASSERT_EQ(mem_read32(mem, spi2 + 0x10C), 0u);
    ASSERT_EQ(cpu0.interrupt & (1u << 8), 1u << 8);

    mem_write32(mem, spi2 + 0x38, 1u << 9); /* clear TRANS_DONE */
    mem_write32(mem, spi2 + 0x11C, 0x1C0u); /* clear DMA completion */
    ASSERT_EQ(cpu0.interrupt & (1u << 8), 0u);

    periph_destroy(p);
    mem_destroy(mem);
}

TEST(gp_spi_dma_full_duplex_sd) {
    const uint32_t spi2 = 0x3FF64000u;
    const uint32_t tx_desc = 0x3FFB1100u;
    const uint32_t rx_desc = 0x3FFB1110u;
    const uint32_t tx_buf = 0x3FFB2100u;
    const uint32_t rx_buf = 0x3FFB2200u;
    xtensa_mem_t *mem = mem_create();
    esp32_periph_t *p = periph_create(mem);
    spi_display_config_t cfg = {
        .dc_pin = 2,
        .display_cs_pin = 15,
        .display_sck_pin = 14,
        .touch_cs_pin = 33,
        .touch_sck_pin = 25,
        .sd_cs_pin = 5,
        .sd_sck_pin = 18,
    };
    periph_enable_spi_display(p, &cfg);
    test_gpio_route(mem, 18, 8);       /* HSPICLK -> SD SCLK */
    test_gpio_route(mem, 5, 11);       /* HSPICS0 -> SD CS */
    mem_write32(mem, spi2 + 0x34, 0x6); /* hardware CS0 selected */

    uint8_t cmd0[6];
    test_sd_command_bytes(cmd0, 0, 0);
    for (int i = 0; i < 6; i++) mem_write8(mem, tx_buf + (uint32_t)i, cmd0[i]);
    test_spi_dma_desc(mem, tx_desc, tx_buf, 6, 6, 1, 0);
    mem_write32(mem, spi2 + 0x1C, 1u << 27);
    mem_write32(mem, spi2 + 0x28, 47);
    mem_write32(mem, spi2 + 0x104,
                (tx_desc & 0x000FFFFFu) | (1u << 29));
    mem_write32(mem, spi2, 1u << 18);

    /* Clock the queued R1 response through independent one-byte TX/RX DMA. */
    mem_write8(mem, tx_buf, 0xFF);
    mem_write8(mem, rx_buf, 0xAA);
    test_spi_dma_desc(mem, tx_desc, tx_buf, 1, 1, 1, 0);
    test_spi_dma_desc(mem, rx_desc, rx_buf, 1, 0, 1, 0);
    mem_write32(mem, spi2 + 0x1C, (1u << 27) | (1u << 28));
    mem_write32(mem, spi2 + 0x28, 7);
    mem_write32(mem, spi2 + 0x2C, 7);
    mem_write32(mem, spi2 + 0x104,
                (tx_desc & 0x000FFFFFu) | (1u << 29));
    mem_write32(mem, spi2 + 0x108,
                (rx_desc & 0x000FFFFFu) | (1u << 29));
    mem_write32(mem, spi2, 1u << 18);

    ASSERT_EQ(mem_read8(mem, rx_buf), 0x01u);
    ASSERT_EQ((mem_read32(mem, rx_desc) >> 12) & 0x0FFFu, 1u);
    ASSERT_EQ(mem_read32(mem, rx_desc) & (1u << 31), 0u);
    ASSERT_EQ(mem_read32(mem, spi2 + 0x124), rx_desc);
    ASSERT_EQ(mem_read32(mem, spi2 + 0x114) & ((1u << 5) | (1u << 3)),
              (1u << 5) | (1u << 3));

    periph_destroy(p);
    mem_destroy(mem);
}

TEST(raw_sd_multiblock_read_write) {
    const uint32_t spi2 = 0x3FF64000u;
    char path[] = "/tmp/flexe-sd-unit-XXXXXX";
    int fd = mkstemp(path);
    ASSERT_TRUE(fd >= 0);
    if (fd < 0) return;
    ASSERT_EQ(ftruncate(fd, 4 * 512), 0);

    xtensa_mem_t *mem = mem_create();
    esp32_periph_t *p = periph_create(mem);
    spi_display_config_t cfg = {
        .dc_pin = 2,
        .display_cs_pin = 15,
        .display_sck_pin = 14,
        .touch_cs_pin = 33,
        .touch_sck_pin = 25,
        .sd_cs_pin = 5,
        .sd_sck_pin = 18,
        .sdcard_path = path,
    };
    periph_enable_spi_display(p, &cfg);
    test_gpio_route(mem, 18, 8);       /* HSPICLK */
    test_gpio_route(mem, 5, 11);       /* HSPICS0 */
    mem_write32(mem, spi2 + 0x34, 0x6); /* hardware CS0 selected */

    /* Arduino's multi-write flow uses ACMD23, CMD25, 0xFC data tokens and
     * a final 0xFD stop token. */
    ASSERT_EQ(test_sd_command(mem, spi2, 55, 0), 0x01);
    ASSERT_EQ(test_sd_command(mem, spi2, 23, 2), 0x00);
    ASSERT_EQ(test_sd_command(mem, spi2, 25, 1), 0x00);

    uint8_t block[2][512];
    for (int i = 0; i < 512; i++) {
        block[0][i] = (uint8_t)(i ^ 0x5A);
        block[1][i] = (uint8_t)(i ^ 0xA5);
    }
    const uint8_t data_token = 0xFC;
    const uint8_t crc[2] = {0xFF, 0xFF};
    const uint8_t idle = 0xFF;
    for (int which = 0; which < 2; which++) {
        test_gp_spi_bytes(mem, spi2, &data_token, NULL, 1);
        for (int off = 0; off < 512; off += 64)
            test_gp_spi_bytes(mem, spi2, block[which] + off, NULL, 64);
        test_gp_spi_bytes(mem, spi2, crc, NULL, sizeof(crc));
        uint8_t response = 0xFF;
        test_gp_spi_bytes(mem, spi2, &idle, &response, 1);
        ASSERT_EQ(response & 0x1F, 0x05);
        do {
            test_gp_spi_bytes(mem, spi2, &idle, &response, 1);
        } while (response == 0x00);
        ASSERT_EQ(response, 0xFF);
    }
    const uint8_t stop_token = 0xFD;
    test_gp_spi_bytes(mem, spi2, &stop_token, NULL, 1);

    uint8_t disk[1024] = {0};
    ASSERT_EQ(pread(fd, disk, sizeof(disk), 512), sizeof(disk));
    ASSERT_TRUE(memcmp(disk, block, sizeof(disk)) == 0);

    /* CMD18 streams consecutive sectors.  CMD12 includes a seventh stuff
     * byte, which must not consume the R1 response returned to the driver. */
    ASSERT_EQ(test_sd_command(mem, spi2, 18, 1), 0x00);
    uint8_t readback[1024] = {0};
    for (int which = 0; which < 2; which++) {
        uint8_t token = 0xFF;
        test_gp_spi_bytes(mem, spi2, &idle, &token, 1);
        ASSERT_EQ(token, 0xFE);
        for (int off = 0; off < 512; off += 64)
            test_gp_spi_bytes(mem, spi2, NULL, readback + which * 512 + off, 64);
        uint8_t ignored_crc[2];
        test_gp_spi_bytes(mem, spi2, NULL, ignored_crc, sizeof(ignored_crc));
    }
    ASSERT_TRUE(memcmp(readback, block, sizeof(readback)) == 0);

    uint8_t stop_cmd[7];
    test_sd_command_bytes(stop_cmd, 12, 0);
    stop_cmd[6] = 0xFF;
    test_gp_spi_bytes(mem, spi2, stop_cmd, NULL, sizeof(stop_cmd));
    uint8_t stop_response = 0xFF;
    test_gp_spi_bytes(mem, spi2, &idle, &stop_response, 1);
    ASSERT_EQ(stop_response, 0x00);

    periph_destroy(p);
    mem_destroy(mem);
    close(fd);
    unlink(path);
}

TEST(dport_safe_defaults) {
    xtensa_mem_t *mem = mem_create();
    esp32_periph_t *p = periph_create(mem);
    /* Cache ctrl enabled */
    ASSERT_EQ(mem_read32(mem, 0x3FF00040), 0x0A);
    /* DPORT_DATE */
    ASSERT_EQ(mem_read32(mem, 0x3FF003A0), 0x16042000);
    /* Interrupt matrix: disabled (16) */
    ASSERT_EQ(mem_read32(mem, 0x3FF00104), 16);
    ASSERT_EQ(mem_read32(mem, 0x3FF002FC), 16);
    /* APPCPU in reset */
    ASSERT_EQ(mem_read32(mem, 0x3FF00018), 1);
    periph_destroy(p);
    mem_destroy(mem);
}

TEST(post_boot_cpu_clock_state_and_writes) {
    const uint32_t dport_cpu_per_conf = 0x3FF0003Cu;
    const uint32_t rtc_cpu_period_conf = 0x3FF48068u;
    const uint32_t rtc_clk_conf = 0x3FF48070u;
    xtensa_mem_t *mem = mem_create();
    esp32_periph_t *p = periph_create(mem);

    /* Flexe starts at the application entry point, after the second-stage
     * bootloader selected the SDK's default 160 MHz PLL clock. Keep all
     * app-visible clock selectors consistent with g_ticks_per_us. */
    ASSERT_EQ(mem_read32(mem, dport_cpu_per_conf), 1u);
    ASSERT_EQ(mem_read32(mem, rtc_cpu_period_conf), 1u << 30u);
    ASSERT_EQ(mem_read32(mem, rtc_clk_conf), 0x08002210u);
    ASSERT_EQ((mem_read32(mem, rtc_clk_conf) >> 27u) & 3u, 1u);

    /* Clock-management code uses read/modify/write on these registers. */
    mem_write32(mem, dport_cpu_per_conf, UINT32_MAX);
    ASSERT_EQ(mem_read32(mem, dport_cpu_per_conf), 0xFu);
    mem_write32(mem, rtc_cpu_period_conf, UINT32_MAX);
    ASSERT_EQ(mem_read32(mem, rtc_cpu_period_conf), 0xE0000000u);
    mem_write32(mem, rtc_clk_conf, UINT32_MAX);
    ASSERT_EQ(mem_read32(mem, rtc_clk_conf), 0xFFFFFFF0u);
    ASSERT_EQ(periph_unhandled_count(p), 0);

    periph_destroy(p);
    mem_destroy(mem);
}

#define TEST_FRC_BASE          0x3FF47000u
#define TEST_FRC_TIMER(n)      (TEST_FRC_BASE + (n) * 0x20u)
#define TEST_FRC_LEVEL_INT     (1u << 0)
#define TEST_FRC_PRESCALER_16  (2u << 1)
#define TEST_FRC_PRESCALER_256 (4u << 1)
#define TEST_FRC_AUTOLOAD      (1u << 6)
#define TEST_FRC_ENABLE        (1u << 7)
#define TEST_FRC_INT_STATUS    (1u << 8)

TEST(frc1_countdown_reload_prescalers_and_interrupt) {
    xtensa_mem_t *mem = mem_create();
    esp32_periph_t *p = periph_create(mem);
    xtensa_cpu_t cpu;
    xtensa_cpu_init(&cpu);
    cpu.mem = mem;
    periph_attach_cpus(p, &cpu, NULL);
    mem_write32(mem, 0x3FFE01E0u, 240u);

    const uint32_t frc1 = TEST_FRC_TIMER(0);
    ASSERT_EQ(mem_read32(mem, frc1 + 0x00u), 0u);
    ASSERT_EQ(mem_read32(mem, frc1 + 0x04u), 0u);
    ASSERT_EQ(mem_read32(mem, frc1 + 0x08u), 0u);
    mem_write32(mem, frc1 + 0x00u, UINT32_MAX);
    ASSERT_EQ(mem_read32(mem, frc1 + 0x00u), 0x007FFFFFu);
    ASSERT_EQ(mem_read32(mem, frc1 + 0x04u), 0x007FFFFFu);

    test_irq_dispatch_t dispatch = {0};
    ASSERT_EQ(periph_set_irq_dispatch(p, 56, test_irq_dispatch,
                                      &dispatch), 0);
    mem_write32(mem, frc1 + 0x00u, 10u);
    mem_write32(mem, frc1 + 0x08u,
                TEST_FRC_ENABLE | TEST_FRC_LEVEL_INT);
    ASSERT_EQ(cpu.next_timer_event, cpu.ccount + 30u);
    cpu.ccount += 15u;
    ASSERT_EQ(mem_read32(mem, frc1 + 0x04u), 5u);
    cpu.ccount += 15u;
    cpu.periph_event(&cpu);
    ASSERT_EQ(mem_read32(mem, frc1 + 0x04u), 0u);
    ASSERT_TRUE(mem_read32(mem, frc1 + 0x08u) & TEST_FRC_INT_STATUS);
    ASSERT_EQ(dispatch.count, 1);
    ASSERT_EQ(dispatch.last_source, 56);
    ASSERT_TRUE(periph_interrupt_pending(p, 56));
    mem_write32(mem, frc1 + 0x0Cu, 1u);
    ASSERT_FALSE(periph_interrupt_pending(p, 56));
    ASSERT_FALSE(mem_read32(mem, frc1 + 0x08u) & TEST_FRC_INT_STATUS);

    /* Auto-load restores the programmed interval at each zero crossing.
     * APB/16 takes 48 CPU cycles per timer tick at 240 MHz. */
    mem_write32(mem, frc1 + 0x00u, 4u);
    mem_write32(mem, frc1 + 0x08u,
                TEST_FRC_ENABLE | TEST_FRC_AUTOLOAD |
                TEST_FRC_PRESCALER_16 | TEST_FRC_LEVEL_INT);
    ASSERT_EQ(cpu.next_timer_event, cpu.ccount + 192u);
    cpu.ccount += 192u;
    cpu.periph_event(&cpu);
    ASSERT_EQ(mem_read32(mem, frc1 + 0x04u), 4u);
    ASSERT_EQ(dispatch.count, 2);
    mem_write32(mem, frc1 + 0x0Cu, 1u);
    cpu.ccount += 96u;
    ASSERT_EQ(mem_read32(mem, frc1 + 0x04u), 2u);
    cpu.ccount += 96u;
    cpu.periph_event(&cpu);
    ASSERT_EQ(mem_read32(mem, frc1 + 0x04u), 4u);
    ASSERT_EQ(dispatch.count, 3);

    /* Edge mode pulses without setting STATUS and repeats without INT_CLR. */
    mem_write32(mem, frc1 + 0x08u, 0u);
    mem_write32(mem, frc1 + 0x0Cu, 1u);
    mem_write32(mem, frc1 + 0x00u, 4u);
    mem_write32(mem, frc1 + 0x08u,
                TEST_FRC_ENABLE | TEST_FRC_AUTOLOAD |
                TEST_FRC_PRESCALER_16);
    cpu.ccount += 192u;
    cpu.periph_event(&cpu);
    ASSERT_EQ(dispatch.count, 4);
    ASSERT_FALSE(periph_interrupt_pending(p, 56));
    ASSERT_FALSE(mem_read32(mem, frc1 + 0x08u) & TEST_FRC_INT_STATUS);
    cpu.ccount += 192u;
    cpu.periph_event(&cpu);
    ASSERT_EQ(dispatch.count, 5);
    ASSERT_FALSE(periph_interrupt_pending(p, 56));

    mem_write32(mem, frc1 + 0x08u, 0u);
    mem_write32(mem, frc1 + 0x0Cu, 1u);
    cpu.ccount += 24000u;
    ASSERT_EQ(mem_read32(mem, frc1 + 0x04u), 4u);
    ASSERT_EQ(periph_unhandled_count(p), 0);

    periph_destroy(p);
    mem_destroy(mem);
}

TEST(frc2_countup_wrap_compare_and_runtime_frequency) {
    xtensa_mem_t *mem = mem_create();
    esp32_periph_t *p = periph_create(mem);
    xtensa_cpu_t cpu;
    xtensa_cpu_init(&cpu);
    cpu.mem = mem;
    periph_attach_cpus(p, &cpu, NULL);
    mem_write32(mem, 0x3FFE01E0u, 240u);

    const uint32_t frc2 = TEST_FRC_TIMER(1);
    test_irq_dispatch_t dispatch = {0};
    ASSERT_EQ(periph_set_irq_dispatch(p, 57, test_irq_dispatch,
                                      &dispatch), 0);
    mem_write32(mem, frc2 + 0x00u, 0xFFFFFFF0u);
    mem_write32(mem, frc2 + 0x10u, 4u);
    mem_write32(mem, frc2 + 0x08u,
                TEST_FRC_ENABLE | TEST_FRC_LEVEL_INT);
    ASSERT_EQ(cpu.next_timer_event, cpu.ccount + 60u);
    cpu.ccount += 30u;
    ASSERT_EQ(mem_read32(mem, frc2 + 0x04u), 0xFFFFFFFAu);
    cpu.ccount += 30u;
    cpu.periph_event(&cpu);
    ASSERT_EQ(mem_read32(mem, frc2 + 0x04u), 4u);
    ASSERT_EQ(mem_read32(mem, frc2 + 0x10u), 4u);
    ASSERT_TRUE(mem_read32(mem, frc2 + 0x08u) & TEST_FRC_INT_STATUS);
    ASSERT_EQ(dispatch.count, 1);
    ASSERT_EQ(dispatch.last_source, 57);
    ASSERT_TRUE(periph_interrupt_pending(p, 57));
    mem_write32(mem, frc2 + 0x0Cu, 1u);
    ASSERT_FALSE(periph_interrupt_pending(p, 57));

    /* Edge mode uses the same legacy TIMER2 source. APB/256 takes 768 CPU
     * cycles per timer tick, and FRC2 keeps counting after its compare. */
    mem_write32(mem, frc2 + 0x00u, 100u);
    mem_write32(mem, frc2 + 0x10u, 102u);
    mem_write32(mem, frc2 + 0x08u,
                TEST_FRC_ENABLE | TEST_FRC_AUTOLOAD |
                TEST_FRC_PRESCALER_256);
    ASSERT_EQ(cpu.next_timer_event, cpu.ccount + 1536u);
    cpu.ccount += 1536u;
    cpu.periph_event(&cpu);
    ASSERT_EQ(mem_read32(mem, frc2 + 0x04u), 102u);
    ASSERT_EQ(dispatch.count, 2);
    ASSERT_FALSE(periph_interrupt_pending(p, 57));
    ASSERT_FALSE(mem_read32(mem, frc2 + 0x08u) & TEST_FRC_INT_STATUS);

    /* Reserved divider encodings select /1 and read back normalized. */
    mem_write32(mem, frc2 + 0x08u, TEST_FRC_ENABLE | (1u << 1));
    ASSERT_EQ(mem_read32(mem, frc2 + 0x08u) & (7u << 1), 0u);

    /* The source remains the fixed 80 MHz APB clock when CPU frequency
     * changes. At 80 MHz with /16, one timer tick is 16 CCOUNT cycles. */
    mem_write32(mem, frc2 + 0x08u, 0u);
    mem_write32(mem, 0x3FFE01E0u, 80u);
    mem_write32(mem, frc2 + 0x00u, 0u);
    mem_write32(mem, frc2 + 0x10u, 10u);
    mem_write32(mem, frc2 + 0x08u,
                TEST_FRC_ENABLE | TEST_FRC_PRESCALER_16 |
                TEST_FRC_LEVEL_INT);
    ASSERT_EQ(cpu.next_timer_event, cpu.ccount + 160u);
    cpu.ccount += 160u;
    cpu.periph_event(&cpu);
    ASSERT_EQ(mem_read32(mem, frc2 + 0x04u), 10u);
    ASSERT_EQ(dispatch.count, 3);
    mem_write32(mem, frc2 + 0x08u, 0u);
    mem_write32(mem, frc2 + 0x0Cu, 1u);
    ASSERT_EQ(periph_unhandled_count(p), 0);

    periph_destroy(p);
    mem_destroy(mem);
}

#define TEST_TIMG0_BASE 0x3FF5F000u
#define TEST_TIMG1_BASE 0x3FF60000u
#define TEST_TIMG_CONFIG(en, increase, autoreload, divider, level, edge, alarm) \
    (((en) ? 1u << 31 : 0u) | ((increase) ? 1u << 30 : 0u) | \
     ((autoreload) ? 1u << 29 : 0u) | (((divider) & 0xFFFFu) << 13) | \
     ((edge) ? 1u << 12 : 0u) | ((level) ? 1u << 11 : 0u) | \
     ((alarm) ? 1u << 10 : 0u))

static uint32_t test_timg_timer_base(unsigned timer) {
    return timer * 0x24u;
}

static uint64_t test_timg_capture(xtensa_mem_t *mem, uint32_t group_base,
                                  unsigned timer) {
    uint32_t base = group_base + test_timg_timer_base(timer);
    mem_write32(mem, base + 0x0Cu, 1u);
    return (uint64_t)mem_read32(mem, base + 0x04u) |
           ((uint64_t)mem_read32(mem, base + 0x08u) << 32);
}

static void test_timg_load(xtensa_mem_t *mem, uint32_t group_base,
                           unsigned timer, uint64_t value) {
    uint32_t base = group_base + test_timg_timer_base(timer);
    mem_write32(mem, base + 0x18u, (uint32_t)value);
    mem_write32(mem, base + 0x1Cu, (uint32_t)(value >> 32));
    mem_write32(mem, base + 0x20u, 1u);
}

TEST(timg_general_timers_count_capture_pause_and_reset) {
    xtensa_mem_t *mem = mem_create();
    esp32_periph_t *p = periph_create(mem);
    xtensa_cpu_t cpu;
    xtensa_cpu_init(&cpu);
    cpu.mem = mem;
    periph_attach_cpus(p, &cpu, NULL);
    mem_write32(mem, 0x3FFE01E0u, 240u);

    ASSERT_EQ(mem_read32(mem, TEST_TIMG0_BASE + 0x00u), 0x60002000u);
    ASSERT_EQ(mem_read32(mem, TEST_TIMG0_BASE + 0x24u), 0x60002000u);
    ASSERT_EQ(mem_read32(mem, TEST_TIMG1_BASE + 0x00u), 0x60002000u);
    ASSERT_EQ(mem_read32(mem, TEST_TIMG1_BASE + 0x24u), 0x60002000u);
    ASSERT_EQ(mem_read32(mem, TEST_TIMG0_BASE + 0xF8u), 0x01604290u);

    /* APB / 80 is 1 MHz.  At a 240 MHz CPU, 240 CCOUNT cycles advance
     * the timer by exactly one tick. */
    mem_write32(mem, 0x3FF000C0u, (1u << 13) | (1u << 15));
    test_timg_load(mem, TEST_TIMG0_BASE, 0, 0x00000001FFFFFFF0ull);
    mem_write32(mem, TEST_TIMG0_BASE + 0x00u,
                TEST_TIMG_CONFIG(1, 1, 0, 80, 0, 0, 0));
    ASSERT_EQ(test_timg_capture(mem, TEST_TIMG0_BASE, 0),
              0x00000001FFFFFFF0ull);
    cpu.ccount += 2400u;
    /* LO/HI remain the previous software snapshot until UPDATE is written. */
    ASSERT_EQ(mem_read32(mem, TEST_TIMG0_BASE + 0x04u), 0xFFFFFFF0u);
    ASSERT_EQ(test_timg_capture(mem, TEST_TIMG0_BASE, 0),
              0x00000001FFFFFFFAull);

    /* Pausing freezes the counter; resuming preserves its 64-bit value. */
    mem_write32(mem, TEST_TIMG0_BASE + 0x00u,
                TEST_TIMG_CONFIG(0, 1, 0, 80, 0, 0, 0));
    cpu.ccount += 24000u;
    ASSERT_EQ(test_timg_capture(mem, TEST_TIMG0_BASE, 0),
              0x00000001FFFFFFFAull);
    mem_write32(mem, TEST_TIMG0_BASE + 0x00u,
                TEST_TIMG_CONFIG(1, 1, 0, 80, 0, 0, 0));
    cpu.ccount += 480u;
    ASSERT_EQ(test_timg_capture(mem, TEST_TIMG0_BASE, 0),
              0x00000001FFFFFFFCull);

    /* Timer 1 in the other group counts down independently. */
    test_timg_load(mem, TEST_TIMG1_BASE, 1, 0x0000000100000002ull);
    mem_write32(mem, TEST_TIMG1_BASE + 0x24u,
                TEST_TIMG_CONFIG(1, 0, 0, 80, 0, 0, 0));
    cpu.ccount += 480u;
    ASSERT_EQ(test_timg_capture(mem, TEST_TIMG1_BASE, 1),
              0x0000000100000000ull);

    /* Gating one module's APB clock pauses only that group. */
    mem_write32(mem, 0x3FF000C0u, 1u << 15);
    cpu.ccount += 2400u;
    ASSERT_EQ(test_timg_capture(mem, TEST_TIMG0_BASE, 0),
              0x00000001FFFFFFFEull);
    ASSERT_EQ(test_timg_capture(mem, TEST_TIMG1_BASE, 1),
              0x00000000FFFFFFF6ull);
    mem_write32(mem, 0x3FF000C0u, (1u << 13) | (1u << 15));

    /* DPORT reset restores both timer register files in the selected group
     * and leaves the other group's live counter untouched. */
    uint64_t group1_before = test_timg_capture(mem, TEST_TIMG1_BASE, 1);
    mem_write32(mem, 0x3FF000C4u, 1u << 13);
    ASSERT_EQ(mem_read32(mem, TEST_TIMG0_BASE + 0x00u), 0x60002000u);
    ASSERT_EQ(mem_read32(mem, TEST_TIMG0_BASE + 0x24u), 0x60002000u);
    ASSERT_EQ(test_timg_capture(mem, TEST_TIMG0_BASE, 0), 0u);
    ASSERT_EQ(test_timg_capture(mem, TEST_TIMG1_BASE, 1), group1_before);
    ASSERT_EQ(periph_unhandled_count(p), 0);

    periph_destroy(p);
    mem_destroy(mem);
}

TEST(timg_alarm_autoreload_mask_clear_and_sources) {
    xtensa_mem_t *mem = mem_create();
    esp32_periph_t *p = periph_create(mem);
    xtensa_cpu_t cpu;
    xtensa_cpu_init(&cpu);
    cpu.mem = mem;
    periph_attach_cpus(p, &cpu, NULL);
    mem_write32(mem, 0x3FFE01E0u, 240u);
    mem_write32(mem, 0x3FF000C0u, (1u << 13) | (1u << 15));

    test_irq_dispatch_t level_dispatch = {0};
    ASSERT_EQ(periph_set_irq_dispatch(p, 14, test_irq_dispatch,
                                      &level_dispatch), 0);
    test_timg_load(mem, TEST_TIMG0_BASE, 0, 2u);
    mem_write32(mem, TEST_TIMG0_BASE + 0x10u, 5u);
    mem_write32(mem, TEST_TIMG0_BASE + 0x14u, 0u);
    mem_write32(mem, TEST_TIMG0_BASE + 0x98u, 1u);
    mem_write32(mem, TEST_TIMG0_BASE + 0x00u,
                TEST_TIMG_CONFIG(1, 1, 1, 80, 1, 0, 1));
    ASSERT_EQ(cpu.next_timer_event, cpu.ccount + 720u);
    cpu.ccount += 960u; /* alarm at 5, reload to 2, then one further tick */
    cpu.periph_event(&cpu);
    ASSERT_EQ(mem_read32(mem, TEST_TIMG0_BASE + 0x9Cu) & 1u, 1u);
    ASSERT_EQ(mem_read32(mem, TEST_TIMG0_BASE + 0xA0u) & 1u, 1u);
    ASSERT_EQ(level_dispatch.count, 1);
    ASSERT_EQ(level_dispatch.last_source, 14);
    ASSERT_TRUE(periph_interrupt_pending(p, 14));
    ASSERT_FALSE(mem_read32(mem, TEST_TIMG0_BASE + 0x00u) & (1u << 10));
    ASSERT_EQ(test_timg_capture(mem, TEST_TIMG0_BASE, 0), 3u);

    mem_write32(mem, TEST_TIMG0_BASE + 0xA4u, 1u);
    ASSERT_FALSE(periph_interrupt_pending(p, 14));
    ASSERT_EQ(mem_read32(mem, TEST_TIMG0_BASE + 0x9Cu) & 1u, 0u);

    /* RAW latches even while masked, but neither level nor edge source does. */
    test_timg_load(mem, TEST_TIMG0_BASE, 0, 0u);
    mem_write32(mem, TEST_TIMG0_BASE + 0x98u, 0u);
    mem_write32(mem, TEST_TIMG0_BASE + 0x00u,
                TEST_TIMG_CONFIG(1, 1, 0, 80, 1, 0, 1));
    cpu.ccount += 1200u;
    cpu.periph_event(&cpu);
    ASSERT_EQ(mem_read32(mem, TEST_TIMG0_BASE + 0x9Cu) & 1u, 1u);
    ASSERT_FALSE(periph_interrupt_pending(p, 14));
    ASSERT_EQ(level_dispatch.count, 1);
    mem_write32(mem, TEST_TIMG0_BASE + 0xA4u, 1u);

    /* Exercise the four distinct edge sources while counting down.  Their
     * classic ESP32 source IDs are 58/59 for group 0 and 62/63 for group 1. */
    const uint32_t edge_group_base[4] = {
        TEST_TIMG0_BASE, TEST_TIMG0_BASE, TEST_TIMG1_BASE, TEST_TIMG1_BASE,
    };
    const unsigned edge_timer[4] = {0u, 1u, 0u, 1u};
    const int edge_source[4] = {58, 59, 62, 63};
    const int level_source[4] = {14, 15, 18, 19};
    test_irq_dispatch_t edge_dispatch[4] = {{0}};
    for (unsigned i = 0; i < 4u; i++)
        ASSERT_EQ(periph_set_irq_dispatch(p, edge_source[i],
                                          test_irq_dispatch,
                                          &edge_dispatch[i]), 0);

    for (unsigned i = 0; i < 4u; i++) {
        uint32_t group_base = edge_group_base[i];
        unsigned timer_index = edge_timer[i];
        uint32_t timer_base = group_base + test_timg_timer_base(timer_index);
        uint32_t bit = 1u << timer_index;
        test_timg_load(mem, group_base, timer_index, 9u);
        mem_write32(mem, timer_base + 0x10u, 4u);
        mem_write32(mem, timer_base + 0x14u, 0u);
        mem_write32(mem, group_base + 0x98u, bit);
        mem_write32(mem, timer_base,
                    TEST_TIMG_CONFIG(1, 0, 0, 80, 0, 1, 1));
        cpu.ccount += 1200u;
        cpu.periph_event(&cpu);

        ASSERT_EQ(mem_read32(mem, group_base + 0x9Cu), bit);
        ASSERT_EQ(edge_dispatch[i].count, 1);
        ASSERT_EQ(edge_dispatch[i].last_source, edge_source[i]);
        ASSERT_TRUE(periph_interrupt_pending(p, edge_source[i]));
        ASSERT_FALSE(periph_interrupt_pending(p, level_source[i]));
        ASSERT_EQ(test_timg_capture(mem, group_base, timer_index), 4u);
        for (unsigned j = i + 1u; j < 4u; j++)
            ASSERT_EQ(edge_dispatch[j].count, 0);

        mem_write32(mem, group_base + 0xA4u, bit);
        ASSERT_FALSE(periph_interrupt_pending(p, edge_source[i]));
        mem_write32(mem, timer_base,
                    TEST_TIMG_CONFIG(0, 0, 0, 80, 0, 1, 0));
    }
    ASSERT_EQ(periph_unhandled_count(p), 0);

    periph_destroy(p);
    mem_destroy(mem);
}

TEST(timg_dual_core_time_uses_monotonic_maximum) {
    xtensa_mem_t *mem = mem_create();
    esp32_periph_t *p = periph_create(mem);
    xtensa_cpu_t cpu0, cpu1;
    xtensa_cpu_init(&cpu0);
    xtensa_cpu_init(&cpu1);
    cpu0.mem = mem;
    cpu1.mem = mem;
    cpu1.core_id = 1;
    periph_attach_cpus(p, &cpu0, &cpu1);
    mem_write32(mem, 0x3FFE01E0u, 240u);
    mem_write32(mem, 0x3FF000C0u, 1u << 13);
    mem_write32(mem, TEST_TIMG0_BASE + 0x00u,
                TEST_TIMG_CONFIG(1, 1, 0, 80, 0, 0, 0));

    cpu0.ccount += 2400u;
    cpu0.periph_event(&cpu0);
    ASSERT_EQ(test_timg_capture(mem, TEST_TIMG0_BASE, 0), 10u);
    cpu1.ccount += 2400u;
    cpu1.periph_event(&cpu1);
    ASSERT_EQ(test_timg_capture(mem, TEST_TIMG0_BASE, 0), 10u);
    cpu1.ccount += 240u;
    cpu1.periph_event(&cpu1);
    ASSERT_EQ(test_timg_capture(mem, TEST_TIMG0_BASE, 0), 11u);

    /* The timer remains sourced from the fixed 80 MHz APB clock when the
     * emulated CPU frequency changes at runtime. */
    mem_write32(mem, 0x3FFE01E0u, 80u);
    cpu1.ccount += 80u;
    cpu1.periph_event(&cpu1);
    ASSERT_EQ(test_timg_capture(mem, TEST_TIMG0_BASE, 0), 12u);
    ASSERT_EQ(periph_unhandled_count(p), 0);

    periph_destroy(p);
    mem_destroy(mem);
}

TEST(wdt_disable) {
    xtensa_mem_t *mem = mem_create();
    esp32_periph_t *p = periph_create(mem);
    /* Unlock WDT (write protect key 0x50D83AA1) */
    mem_write32(mem, 0x3FF5F064, 0x50D83AA1);
    /* Set config0 = 0 (disabled) */
    mem_write32(mem, 0x3FF5F048, 0);
    ASSERT_EQ(mem_read32(mem, 0x3FF5F048), 0);
    /* Feed WDT */
    mem_write32(mem, 0x3FF5F060, 1);
    /* Re-lock */
    mem_write32(mem, 0x3FF5F064, 0);
    ASSERT_EQ(mem_read32(mem, 0x3FF5F064), 0);
    periph_destroy(p);
    mem_destroy(mem);
}

TEST(rtc_reset_cause) {
    xtensa_mem_t *mem = mem_create();
    esp32_periph_t *p = periph_create(mem);
    ASSERT_EQ(mem_read32(mem, 0x3FF48034), 1); /* POWERON */
    ASSERT_EQ(mem_read32(mem, 0x3FF480A8), 0x2210); /* CLK_CONF */
    periph_destroy(p);
    mem_destroy(mem);
}

TEST(sens_adc_single_conversions) {
    xtensa_mem_t *mem = mem_create();
    esp32_periph_t *p = periph_create(mem);
    const uint32_t sens = 0x3FF48800u;

    periph_set_adc_value(p, 6, 0x0A55u);  /* ADC1 channel 6 / GPIO34 */
    periph_set_adc_value(p, 10, 0x05AAu); /* ADC2 channel 0 / GPIO4 */

    /* Reset width is 12 bits. The driver's low/high START sequence clears
     * DONE and then synchronously latches the selected one-hot channel. */
    ASSERT_EQ(mem_read32(mem, sens + 0x2Cu) & 0xFu, 0xFu);
    uint32_t adc1_cfg = (1u << 31) | (1u << (19 + 6)) | (1u << 18);
    mem_write32(mem, sens + 0x54u, adc1_cfg);
    ASSERT_EQ(mem_read32(mem, sens + 0x54u) & (1u << 16), 0u);
    mem_write32(mem, sens + 0x54u, adc1_cfg | (1u << 17));
    uint32_t adc1 = mem_read32(mem, sens + 0x54u);
    ASSERT_EQ(adc1 & (1u << 16), 1u << 16);
    ASSERT_EQ(adc1 & 0xFFFFu, 0x0A55u);
    ASSERT_EQ(adc1 & 0xFFFE0000u, adc1_cfg | (1u << 17));

    /* ADC2 has an independent width field and channel bank. At nine bits,
     * the injected 0x5AA sample is truncated to 0x1AA. */
    mem_write32(mem, sens + 0x2Cu, 0x3u);
    uint32_t adc2_cfg = (1u << 31) | (1u << 19) | (1u << 18);
    mem_write32(mem, sens + 0x94u, adc2_cfg);
    mem_write32(mem, sens + 0x94u, adc2_cfg | (1u << 17));
    uint32_t adc2 = mem_read32(mem, sens + 0x94u);
    ASSERT_EQ(adc2 & (1u << 16), 1u << 16);
    ASSERT_EQ(adc2 & 0xFFFFu, 0x01AAu);

    /* Non-conversion SENS registers retain normal R/W state. */
    mem_write32(mem, sens + 0x34u, 0xDEADBEEFu);
    ASSERT_EQ(mem_read32(mem, sens + 0x34u), 0xDEADBEEFu);
    ASSERT_EQ(periph_unhandled_count(p), 0);
    periph_destroy(p);
    mem_destroy(mem);
}

typedef struct {
    unsigned count;
    uint8_t channel;
    uint8_t enabled;
    uint8_t value;
} dac_event_capture_t;

static void capture_dac_event(const sbx_event_t *ev, void *ctx) {
    if (ev->kind != SBX_EV_DAC_OUT) return;
    dac_event_capture_t *capture = ctx;
    capture->count++;
    capture->channel = ev->dac_out.channel;
    capture->enabled = ev->dac_out.enabled;
    capture->value = ev->dac_out.value;
}

TEST(rtcio_dac_state_and_events) {
    xtensa_mem_t *mem = mem_create();
    esp32_periph_t *p = periph_create(mem);
    const uint32_t rtcio = 0x3FF48400u;
    dac_event_capture_t capture = {0};
    sbx_events_set_sink(capture_dac_event, &capture);

    ASSERT_EQ(mem_read32(mem, rtcio + 0x84u), 2u << 30);
    ASSERT_EQ(mem_read32(mem, rtcio + 0x88u), 2u << 30);
    ASSERT_EQ(periph_dac_enabled(p, 0), 0);
    ASSERT_EQ(periph_dac_value(p, 0), 0);

    uint32_t dac1 = (2u << 30) | (0x35u << 19) |
                    (1u << 18) | (1u << 10);
    mem_write32(mem, rtcio + 0x84u, dac1);
    ASSERT_EQ(mem_read32(mem, rtcio + 0x84u), dac1);
    ASSERT_EQ(periph_dac_enabled(p, 0), 1);
    ASSERT_EQ(periph_dac_value(p, 0), 0x35u);
    ASSERT_EQ(capture.count, 1u);
    ASSERT_EQ(capture.channel, 0u);
    ASSERT_EQ(capture.enabled, 1u);
    ASSERT_EQ(capture.value, 0x35u);

    mem_write32(mem, rtcio + 0x84u,
                (dac1 & ~(0xFFu << 19)) | (0xCAu << 19));
    ASSERT_EQ(periph_dac_value(p, 0), 0xCAu);
    ASSERT_EQ(capture.count, 2u);
    ASSERT_EQ(capture.value, 0xCAu);

    mem_write32(mem, rtcio + 0x84u,
                mem_read32(mem, rtcio + 0x84u) & ~(1u << 10));
    ASSERT_EQ(periph_dac_enabled(p, 0), 0);
    ASSERT_EQ(capture.count, 3u);
    ASSERT_EQ(capture.enabled, 0u);

    uint32_t dac2 = (2u << 30) | (0x7Eu << 19) |
                    (1u << 18) | (1u << 10);
    mem_write32(mem, rtcio + 0x88u, dac2);
    ASSERT_EQ(periph_dac_enabled(p, 1), 1);
    ASSERT_EQ(periph_dac_value(p, 1), 0x7Eu);
    ASSERT_EQ(capture.channel, 1u);

    /* RTC GPIO W1 aliases update their backing state and remain write-only. */
    mem_write32(mem, rtcio + 0x04u, 1u << 14);
    ASSERT_EQ(mem_read32(mem, rtcio + 0x00u), 1u << 14);
    ASSERT_EQ(mem_read32(mem, rtcio + 0x04u), 0u);
    mem_write32(mem, rtcio + 0x08u, 1u << 14);
    ASSERT_EQ(mem_read32(mem, rtcio + 0x00u), 0u);
    ASSERT_EQ(periph_unhandled_count(p), 0);

    sbx_events_set_sink(NULL, NULL);
    periph_destroy(p);
    mem_destroy(mem);
}

typedef struct {
    unsigned callback_count;
    unsigned event_count;
    int channel;
    int gpio;
    uint32_t frequency_hz;
    int8_t duty;
    bool enabled;
    bool inverted;
} sigmadelta_capture_t;

static void capture_sigmadelta_output(void *ctx, int channel, int gpio,
                                      uint32_t frequency_hz, int8_t duty,
                                      bool enabled, bool inverted) {
    sigmadelta_capture_t *capture = ctx;
    capture->callback_count++;
    capture->channel = channel;
    capture->gpio = gpio;
    capture->frequency_hz = frequency_hz;
    capture->duty = duty;
    capture->enabled = enabled;
    capture->inverted = inverted;
}

static void capture_sigmadelta_event(const sbx_event_t *event, void *ctx) {
    if (event->kind != SBX_EV_SIGMADELTA_OUT) return;
    sigmadelta_capture_t *capture = ctx;
    capture->event_count++;
}

TEST(sigmadelta_registers_matrix_and_aggregate_output) {
    const uint32_t sd = 0x3FF44F00u;
    const uint32_t gpio18_route = 0x3FF44530u + 18u * 4u;
    xtensa_mem_t *mem = mem_create();
    esp32_periph_t *p = periph_create(mem);
    sigmadelta_capture_t capture = {0};
    sbx_events_set_sink(capture_sigmadelta_event, &capture);

    ASSERT_EQ(mem_read32(mem, sd + 0x00u), 0x0000FF00u);
    ASSERT_EQ(mem_read32(mem, sd + 0x1Cu), 0x0000FF00u);
    ASSERT_EQ(mem_read32(mem, sd + 0x20u), 0u);
    ASSERT_EQ(mem_read32(mem, sd + 0x24u), 0u);
    ASSERT_EQ(mem_read32(mem, sd + 0x28u), 0x01506190u);

    ASSERT_EQ(periph_set_sigmadelta_output_callback(
                  p, 0, capture_sigmadelta_output, &capture), 0);
    ASSERT_EQ(capture.callback_count, 1u);
    ASSERT_EQ(capture.channel, 0);
    ASSERT_EQ(capture.gpio, -1);
    ASSERT_EQ(capture.frequency_hz, 1220u);
    ASSERT_EQ(capture.duty, 0);
    ASSERT_FALSE(capture.enabled);

    /* Reserved channel bits read zero. Duty 0x40 is signed +64, yielding
     * 192 high samples in each 256-sample modulation period. */
    mem_write32(mem, sd + 0x00u, 0xDEAD0040u);
    ASSERT_EQ(mem_read32(mem, sd + 0x00u), 0x00000040u);
    ASSERT_EQ(capture.frequency_hz, 312500u);
    ASSERT_EQ(capture.duty, 64);

    mem_write32(mem, gpio18_route, 100u | (1u << 9u));
    ASSERT_EQ(capture.gpio, 18);
    ASSERT_FALSE(capture.enabled);
    ASSERT_TRUE(capture.inverted);
    mem_write32(mem, 0x3FF44024u, 1u << 18u);
    ASSERT_TRUE(capture.enabled);

    mem_write32(mem, sd + 0x20u, UINT32_MAX);
    mem_write32(mem, sd + 0x24u, UINT32_MAX);
    mem_write32(mem, sd + 0x28u, UINT32_MAX);
    ASSERT_EQ(mem_read32(mem, sd + 0x20u), 1u << 31u);
    ASSERT_EQ(mem_read32(mem, sd + 0x24u), 1u << 31u);
    ASSERT_EQ(mem_read32(mem, sd + 0x28u), 0x0FFFFFFFu);
    ASSERT_TRUE(capture.event_count >= capture.callback_count);
    ASSERT_EQ(periph_set_sigmadelta_output_callback(
                  p, 8, capture_sigmadelta_output, &capture), -1);
    ASSERT_EQ(periph_unhandled_count(p), 0);

    sbx_events_set_sink(NULL, NULL);
    periph_destroy(p);
    mem_destroy(mem);
}

TEST(sigmadelta_timed_density_prescaler_and_inversion) {
    const uint32_t sd = 0x3FF44F00u;
    const uint32_t gpio18_route = 0x3FF44530u + 18u * 4u;
    xtensa_mem_t *mem = mem_create();
    esp32_periph_t *p = periph_create(mem);
    xtensa_cpu_t cpu;
    xtensa_cpu_init(&cpu);
    cpu.mem = mem;
    periph_attach_cpus(p, &cpu, NULL);
    mem_write32(mem, 0x3FFE01E0u, 240u);

    mem_write32(mem, sd + 0x00u, 0x00000040u); /* +64: 192/256 */
    mem_write32(mem, gpio18_route, 100u);
    mem_write32(mem, 0x3FF44024u, 1u << 18u);

    unsigned high = 0u;
    for (unsigned sample = 1u; sample <= 256u; sample++) {
        cpu.ccount = sample * 3u; /* 80 MHz modulator on a 240 MHz CPU */
        high += (unsigned)periph_gpio_pin_level(p, 18);
    }
    ASSERT_EQ(high, 192u);

    /* Matrix inversion complements the pulse density without mutating the
     * programmed signed duty. */
    mem_write32(mem, gpio18_route, 100u | (1u << 9u));
    high = 0u;
    uint32_t base = cpu.ccount;
    for (unsigned sample = 1u; sample <= 256u; sample++) {
        cpu.ccount = base + sample * 3u;
        high += (unsigned)periph_gpio_pin_level(p, 18);
    }
    ASSERT_EQ(high, 64u);

    /* Prescale three divides the 80 MHz sample clock by four. Signed -64
     * therefore emits 64 high samples, now twelve CPU cycles apart. */
    mem_write32(mem, gpio18_route, 100u);
    mem_write32(mem, sd + 0x00u, (3u << 8u) | 0xC0u);
    high = 0u;
    base = cpu.ccount;
    for (unsigned sample = 1u; sample <= 256u; sample++) {
        cpu.ccount = base + sample * 12u;
        high += (unsigned)periph_gpio_pin_level(p, 18);
    }
    ASSERT_EQ(high, 64u);
    ASSERT_EQ(mem_read32(mem, sd + 0x00u), 0x000003C0u);
    ASSERT_EQ(periph_gpio_out_signal(p, 18), 100);
    ASSERT_EQ(periph_gpio_output_enabled(p, 18), 1);
    ASSERT_EQ(periph_unhandled_count(p), 0);

    periph_destroy(p);
    mem_destroy(mem);
}

TEST(gpio_set_clear) {
    xtensa_mem_t *mem = mem_create();
    esp32_periph_t *p = periph_create(mem);
    /* Set bits via W1TS */
    mem_write32(mem, 0x3FF44008, 0x0F);
    ASSERT_EQ(mem_read32(mem, 0x3FF44004), 0x0F);
    /* Clear bits via W1TC */
    mem_write32(mem, 0x3FF4400C, 0x03);
    ASSERT_EQ(mem_read32(mem, 0x3FF44004), 0x0C);
    periph_destroy(p);
    mem_destroy(mem);
}

TEST(gpio_high_pin_falling_edge_interrupt) {
    xtensa_mem_t *mem = mem_create();
    esp32_periph_t *p = periph_create(mem);
    xtensa_cpu_t cpu0, cpu1;
    xtensa_cpu_init(&cpu0); cpu0.mem = mem;
    xtensa_cpu_init(&cpu1); cpu1.mem = mem;
    periph_attach_cpus(p, &cpu0, &cpu1);

    /* GPIO source 22 -> APP CPU interrupt 5. GPIO36 is the CYD PENIRQ. */
    periph_intr_matrix_set(p, 1, 5, 22);
    uint32_t pin36 = 0x3FF44088u + 36u * 4u;
    mem_write32(mem, pin36, (2u << 7) | (1u << 13)); /* falling, APP */

    periph_gpio_set_input(p, 36, 1); /* idle high: no falling edge */
    ASSERT_EQ(mem_read32(mem, 0x3FF44040) & 0x10u, 0x10u);
    ASSERT_EQ(cpu1.interrupt & (1u << 5), 0);

    periph_gpio_set_input(p, 36, 0);
    ASSERT_EQ(mem_read32(mem, 0x3FF44050), 0x10u); /* STATUS1 */
    ASSERT_EQ(mem_read32(mem, 0x3FF44074), 0x10u); /* ACPU_INT1 */
    ASSERT_EQ(mem_read32(mem, 0x3FF4407C), 0u);    /* PCPU_INT1 */
    ASSERT_EQ(cpu1.interrupt & (1u << 5), (1u << 5));
    ASSERT_EQ(cpu0.interrupt & (1u << 5), 0);

    mem_write32(mem, 0x3FF44058, 0x10u); /* STATUS1_W1TC */
    ASSERT_EQ(mem_read32(mem, 0x3FF44050), 0u);
    ASSERT_EQ(cpu1.interrupt & (1u << 5), 0);

    periph_destroy(p);
    mem_destroy(mem);
}

TEST(gpio_interrupt_routes_to_selected_core) {
    xtensa_mem_t *mem = mem_create();
    esp32_periph_t *p = periph_create(mem);
    xtensa_cpu_t cpu0, cpu1;
    xtensa_cpu_init(&cpu0); cpu0.mem = mem;
    xtensa_cpu_init(&cpu1); cpu1.mem = mem;
    periph_attach_cpus(p, &cpu0, &cpu1);
    periph_intr_matrix_set(p, 0, 4, 22);
    periph_intr_matrix_set(p, 1, 4, 22);

    /* INT_ENA field value BIT(2) selects PRO CPU normal interrupt. */
    uint32_t pin4 = 0x3FF44088u + 4u * 4u;
    mem_write32(mem, pin4, (1u << 7) | (4u << 13)); /* rising, PRO */
    periph_gpio_set_input(p, 4, 1);

    ASSERT_EQ(mem_read32(mem, 0x3FF44068), 1u << 4); /* PCPU_INT */
    ASSERT_EQ(mem_read32(mem, 0x3FF44060), 0u);      /* ACPU_INT */
    ASSERT_EQ(cpu0.interrupt & (1u << 4), (1u << 4));
    ASSERT_EQ(cpu1.interrupt & (1u << 4), 0);

    mem_write32(mem, 0x3FF4404C, 1u << 4);
    ASSERT_EQ(cpu0.interrupt & (1u << 4), 0);
    periph_destroy(p);
    mem_destroy(mem);
}

TEST(gpio_level_interrupt_reasserts_until_inactive) {
    xtensa_mem_t *mem = mem_create();
    esp32_periph_t *p = periph_create(mem);
    xtensa_cpu_t cpu0, cpu1;
    xtensa_cpu_init(&cpu0); cpu0.mem = mem;
    xtensa_cpu_init(&cpu1); cpu1.mem = mem;
    periph_attach_cpus(p, &cpu0, &cpu1);
    periph_intr_matrix_set(p, 1, 3, 22);

    uint32_t pin18 = 0x3FF44088u + 18u * 4u;
    mem_write32(mem, pin18, (4u << 7) | (1u << 13)); /* low level, APP */
    ASSERT_EQ(cpu1.interrupt & (1u << 3), (1u << 3));

    /* Acknowledge while still low: hardware immediately re-latches it. */
    mem_write32(mem, 0x3FF4404C, 1u << 18);
    ASSERT_EQ(mem_read32(mem, 0x3FF44044) & (1u << 18), 1u << 18);
    ASSERT_EQ(cpu1.interrupt & (1u << 3), (1u << 3));

    periph_gpio_set_input(p, 18, 1);
    mem_write32(mem, 0x3FF4404C, 1u << 18);
    ASSERT_EQ(mem_read32(mem, 0x3FF44044) & (1u << 18), 0u);
    ASSERT_EQ(cpu1.interrupt & (1u << 3), 0);

    periph_destroy(p);
    mem_destroy(mem);
}

TEST(efuse_chip_info) {
    xtensa_mem_t *mem = mem_create();
    esp32_periph_t *p = periph_create(mem);
    ASSERT_EQ(mem_read32(mem, 0x3FF5A044), 0xAABBCCDD); /* MAC low */
    ASSERT_EQ(mem_read32(mem, 0x3FF5A048), 0x0000EEFF); /* MAC high */
    ASSERT_EQ(mem_read32(mem, 0x3FF5A058), 1);           /* Chip rev 1 */
    periph_destroy(p);
    mem_destroy(mem);
}

TEST(insn_reads_periph) {
    /* L32I from peripheral address should dispatch to MMIO handler */
    xtensa_cpu_t cpu;
    setup(&cpu);
    esp32_periph_t *p = periph_create(cpu.mem);

    /* a2 = 0x3FF5A044 (EFUSE MAC low) */
    ar_write(&cpu, 2, 0x3FF5A044);
    /* L32I a3, a2, 0  =>  op0=2, r=2(L32I), s=2, t=3, imm8=0 */
    uint32_t insn = (0u << 16) | (2 << 12) | (2 << 8) | (3 << 4) | 0x2;
    put_insn3(&cpu, BASE, insn);
    xtensa_step(&cpu);
    ASSERT_EQ(ar_read(&cpu, 3), 0xAABBCCDD);

    periph_destroy(p);
    teardown(&cpu);
}

/* ===== Interrupt matrix tests ===== */

TEST(intr_matrix_default_disabled) {
    xtensa_mem_t *mem = mem_create();
    esp32_periph_t *p = periph_create(mem);
    /* All interrupt matrix entries should default to 16 (disabled) */
    for (int ci = 0; ci < 32; ci++) {
        ASSERT_EQ(periph_intr_matrix_get(p, 0, ci), 16);
        ASSERT_EQ(periph_intr_matrix_get(p, 1, ci), 16);
    }
    periph_destroy(p);
    mem_destroy(mem);
}

TEST(intr_matrix_set_and_read) {
    xtensa_mem_t *mem = mem_create();
    esp32_periph_t *p = periph_create(mem);
    /* Set source 24 to CPU int 5 on core 0 */
    periph_intr_matrix_set(p, 0, 5, 24);
    ASSERT_EQ(periph_intr_matrix_get(p, 0, 5), 24);
    /* Other entries unchanged */
    ASSERT_EQ(periph_intr_matrix_get(p, 0, 4), 16);
    ASSERT_EQ(periph_intr_matrix_get(p, 1, 5), 16);
    periph_destroy(p);
    mem_destroy(mem);
}

TEST(intr_matrix_dport_rw) {
    xtensa_mem_t *mem = mem_create();
    esp32_periph_t *p = periph_create(mem);
    const uint32_t pro_map = 0x3FF00104u;
    const uint32_t app_map = 0x3FF00218u;

    /* Hardware owns one register per source, whose value selects the CPU
     * interrupt. All 69 registers on both cores reset disabled. */
    for (uint32_t source = 0; source < 69; source++) {
        ASSERT_EQ(mem_read32(mem, pro_map + source * 4u), 16u);
        ASSERT_EQ(mem_read32(mem, app_map + source * 4u), 16u);
    }

    /* Source 30 (SPI2) -> PRO CPU interrupt 3. */
    mem_write32(mem, pro_map + 30u * 4u, 3u);
    ASSERT_EQ(mem_read32(mem, pro_map + 30u * 4u), 3u);
    ASSERT_EQ(periph_intr_matrix_get(p, 0, 3), 30);

    /* Source 25 (FROM_CPU1) -> APP CPU interrupt 8. */
    mem_write32(mem, app_map + 25u * 4u, 8u);
    ASSERT_EQ(mem_read32(mem, app_map + 25u * 4u), 8u);
    ASSERT_EQ(periph_intr_matrix_get(p, 1, 8), 25);

    /* Source 68 is the last real ESP32 matrix source. */
    mem_write32(mem, pro_map + 68u * 4u, 9u);
    mem_write32(mem, app_map + 68u * 4u, 10u);
    ASSERT_EQ(mem_read32(mem, 0x3FF00214u), 9u);
    ASSERT_EQ(mem_read32(mem, 0x3FF00328u), 10u);
    ASSERT_EQ(periph_intr_matrix_get(p, 0, 9), 68);
    ASSERT_EQ(periph_intr_matrix_get(p, 1, 10), 68);
    periph_destroy(p);
    mem_destroy(mem);
}

TEST(intr_matrix_fan_in_remap_and_disable) {
    xtensa_mem_t *mem = mem_create();
    esp32_periph_t *p = periph_create(mem);
    xtensa_cpu_t cpu0;
    xtensa_cpu_init(&cpu0); cpu0.mem = mem;
    periph_attach_cpus(p, &cpu0, NULL);
    const uint32_t pro_map = 0x3FF00104u;

    /* Two asserted sources may share one level-triggered CPU line. Removing
     * either source must leave that line asserted until the other is gone. */
    mem_write32(mem, pro_map + 24u * 4u, 5u);
    mem_write32(mem, pro_map + 25u * 4u, 5u);
    periph_assert_interrupt(p, 24);
    periph_assert_interrupt(p, 25);
    ASSERT_EQ(cpu0.interrupt & (1u << 5), 1u << 5);
    periph_deassert_interrupt(p, 24);
    ASSERT_EQ(cpu0.interrupt & (1u << 5), 1u << 5);

    /* Remapping a live source transfers its level atomically. Selecting the
     * matrix reset value 16 disconnects it. */
    mem_write32(mem, pro_map + 25u * 4u, 8u);
    ASSERT_EQ(cpu0.interrupt & (1u << 5), 0u);
    ASSERT_EQ(cpu0.interrupt & (1u << 8), 1u << 8);
    mem_write32(mem, pro_map + 25u * 4u, 16u);
    ASSERT_EQ(cpu0.interrupt & (1u << 8), 0u);

    periph_destroy(p);
    mem_destroy(mem);
}

TEST(intr_matrix_internal_cpu_lines_are_isolated) {
    xtensa_mem_t *mem = mem_create();
    esp32_periph_t *p = periph_create(mem);
    xtensa_cpu_t cpu0;
    xtensa_cpu_init(&cpu0); cpu0.mem = mem;
    periph_attach_cpus(p, &cpu0, NULL);
    const uint32_t pro_map = 0x3FF00104u;
    static const uint8_t internal_lines[] = {6, 7, 11, 15, 16, 29};

    for (uint32_t i = 0;
         i < sizeof(internal_lines) / sizeof(internal_lines[0]); i++) {
        int source = (int)i;
        uint32_t line = internal_lines[i];
        uint32_t mask = 1u << line;

        /* The register retains the selector, but an asserted peripheral
         * source cannot drive an internal timer/software/profiling line. */
        mem_write32(mem, pro_map + i * 4u, line);
        ASSERT_EQ(mem_read32(mem, pro_map + i * 4u), line);
        periph_assert_interrupt(p, source);
        ASSERT_EQ(cpu0.interrupt & mask, 0u);

        /* Nor may deasserting that source clear an independently latched
         * core-local interrupt on the same numeric line. */
        cpu0.interrupt |= mask;
        periph_deassert_interrupt(p, source);
        ASSERT_EQ(cpu0.interrupt & mask, mask);
        cpu0.interrupt &= ~mask;
    }

    /* This is ESP-IDF's exact allocator initialization pattern. Every source
     * selects ETS_INVALID_INUM (CCOMPARE0 line 6), leaving the tick intact. */
    for (uint32_t source = 0; source < 69; source++)
        mem_write32(mem, pro_map + source * 4u, 6u);
    cpu0.interrupt |= 1u << 6;
    periph_deassert_interrupt(p, 43); /* LEDC regularly updates its level */
    ASSERT_EQ(cpu0.interrupt & (1u << 6), 1u << 6);

    periph_destroy(p);
    mem_destroy(mem);
}

TEST(intr_matrix_dual_core_pending_route) {
    xtensa_mem_t *mem = mem_create();
    esp32_periph_t *p = periph_create(mem);
    xtensa_cpu_t cpu0, cpu1;
    xtensa_cpu_init(&cpu0); cpu0.mem = mem;
    xtensa_cpu_init(&cpu1); cpu1.mem = mem;
    const uint32_t pro_map = 0x3FF00104u;
    const uint32_t app_map = 0x3FF00218u;

    /* RTC_CORE is source 46. Program both hardware register banks and assert
     * it before attaching CPUs, as can happen during staged core startup. */
    mem_write32(mem, pro_map + 46u * 4u, 9u);
    mem_write32(mem, app_map + 46u * 4u, 10u);
    ASSERT_EQ(mem_read32(mem, 0x3FF001BCu), 9u);
    ASSERT_EQ(mem_read32(mem, 0x3FF002D0u), 10u);
    periph_assert_interrupt(p, 46);
    periph_attach_cpus(p, &cpu0, &cpu1);
    ASSERT_EQ(cpu0.interrupt & (1u << 9), 1u << 9);
    ASSERT_EQ(cpu1.interrupt & (1u << 10), 1u << 10);

    periph_deassert_interrupt(p, 46);
    ASSERT_EQ(cpu0.interrupt & (1u << 9), 0u);
    ASSERT_EQ(cpu1.interrupt & (1u << 10), 0u);
    periph_destroy(p);
    mem_destroy(mem);
}

TEST(intr_matrix_assert_source) {
    xtensa_mem_t *mem = mem_create();
    esp32_periph_t *p = periph_create(mem);
    xtensa_cpu_t cpu0, cpu1;
    xtensa_cpu_init(&cpu0); cpu0.mem = mem;
    xtensa_cpu_init(&cpu1); cpu1.mem = mem;
    periph_attach_cpus(p, &cpu0, &cpu1);
    /* Map source 24 → CPU int 5 on core 0 */
    periph_intr_matrix_set(p, 0, 5, 24);
    /* Assert source 24 */
    periph_assert_interrupt(p, 24);
    ASSERT_EQ(cpu0.interrupt & (1u << 5), (1u << 5));
    ASSERT_EQ(cpu0.irq_check, true);
    /* Core 1 should not be affected */
    ASSERT_EQ(cpu1.interrupt & (1u << 5), 0);
    /* Deassert */
    periph_deassert_interrupt(p, 24);
    ASSERT_EQ(cpu0.interrupt & (1u << 5), 0);
    periph_destroy(p);
    mem_destroy(mem);
}

TEST(cross_core_interrupt) {
    xtensa_mem_t *mem = mem_create();
    esp32_periph_t *p = periph_create(mem);
    xtensa_cpu_t cpu0, cpu1;
    xtensa_cpu_init(&cpu0); cpu0.mem = mem;
    xtensa_cpu_init(&cpu1); cpu1.mem = mem;
    periph_attach_cpus(p, &cpu0, &cpu1);
    /* Map FROM_CPU_INTR0 (source 24) → CPU int 2 on core 1 */
    periph_intr_matrix_set(p, 1, 2, 24);
    /* Write 1 to DPORT_CPU_INTR_FROM_CPU_0_REG */
    mem_write32(mem, 0x3FF000DC, 1);
    ASSERT_EQ(cpu1.interrupt & (1u << 2), (1u << 2));
    ASSERT_EQ(cpu1.irq_check, true);
    /* Clear it */
    mem_write32(mem, 0x3FF000DC, 0);
    ASSERT_EQ(cpu1.interrupt & (1u << 2), 0);
    periph_destroy(p);
    mem_destroy(mem);
}

#define TEST_PCNT_BASE 0x3FF57000u

static uint32_t test_pcnt_ctrl_running(unsigned unit) {
    return (0x5555u & ~(1u << (unit * 2u))) | (1u << 16);
}

static void test_pcnt_route_input(xtensa_mem_t *mem, unsigned signal,
                                  unsigned gpio, bool inverted) {
    mem_write32(mem, 0x3FF44130u + signal * 4u,
                (gpio & 0x3Fu) | (inverted ? 1u << 6 : 0u) | (1u << 7));
}

static void test_pcnt_pulse(esp32_periph_t *p, int gpio) {
    periph_gpio_set_input(p, gpio, 1);
    periph_gpio_set_input(p, gpio, 0);
}

TEST(pcnt_matrix_control_limits_events_and_reset) {
    xtensa_mem_t *mem = mem_create();
    esp32_periph_t *p = periph_create(mem);
    xtensa_cpu_t cpu;
    xtensa_cpu_init(&cpu); cpu.mem = mem;
    periph_attach_cpus(p, &cpu, NULL);
    periph_intr_matrix_set(p, 0, 8, 48);
    mem_write32(mem, 0x3FF000C0u, 1u << 10); /* PCNT module clock */

    ASSERT_EQ(mem_read32(mem, TEST_PCNT_BASE + 0x000u), 0x3C10u);
    ASSERT_EQ(mem_read32(mem, TEST_PCNT_BASE + 0x0B0u), 0x5555u);
    ASSERT_EQ(mem_read32(mem, TEST_PCNT_BASE + 0x0FCu), 0x14122600u);

    /* Unit0 channel0: positive edges increment while control is low and
     * reverse direction while control is high. Threshold0=2, limits +/-3. */
    test_pcnt_route_input(mem, 39u, 18u, false); /* pulse0/unit0 */
    test_pcnt_route_input(mem, 41u, 19u, false); /* ctrl0/unit0 */
    uint32_t conf0 = (1u << 18) | (1u << 20) |
                     (1u << 15) | (1u << 14) | (1u << 13) |
                     (1u << 12) | (1u << 11);
    mem_write32(mem, TEST_PCNT_BASE + 0x000u, conf0);
    mem_write32(mem, TEST_PCNT_BASE + 0x004u,
                ((uint32_t)(uint16_t)-1 << 16) | 2u);
    mem_write32(mem, TEST_PCNT_BASE + 0x008u,
                ((uint32_t)(uint16_t)-3 << 16) | 3u);
    mem_write32(mem, TEST_PCNT_BASE + 0x0B0u,
                test_pcnt_ctrl_running(0));
    mem_write32(mem, TEST_PCNT_BASE + 0x088u, 1u);

    test_pcnt_pulse(p, 18);
    ASSERT_EQ(mem_read32(mem, TEST_PCNT_BASE + 0x060u), 1u);
    test_pcnt_pulse(p, 18);
    ASSERT_EQ(mem_read32(mem, TEST_PCNT_BASE + 0x060u), 2u);
    ASSERT_EQ(mem_read32(mem, TEST_PCNT_BASE + 0x080u), 1u);
    ASSERT_EQ(mem_read32(mem, TEST_PCNT_BASE + 0x084u), 1u);
    ASSERT_EQ(mem_read32(mem, TEST_PCNT_BASE + 0x090u) & (1u << 3),
              1u << 3);
    ASSERT_TRUE(periph_interrupt_pending(p, 48));
    ASSERT_EQ(cpu.interrupt & (1u << 8), 1u << 8);
    mem_write32(mem, TEST_PCNT_BASE + 0x08Cu, 1u);
    ASSERT_FALSE(periph_interrupt_pending(p, 48));

    /* Reaching the configured high limit raises its event and resets the
     * hardware counter to zero even though ZERO is a distinct event. */
    test_pcnt_pulse(p, 18);
    ASSERT_EQ(mem_read32(mem, TEST_PCNT_BASE + 0x060u), 0u);
    ASSERT_EQ(mem_read32(mem, TEST_PCNT_BASE + 0x090u) & (1u << 5),
              1u << 5);
    mem_write32(mem, TEST_PCNT_BASE + 0x08Cu, 1u);

    periph_gpio_set_input(p, 19, 1); /* reverse positive-edge action */
    test_pcnt_pulse(p, 18);
    ASSERT_EQ(mem_read32(mem, TEST_PCNT_BASE + 0x060u), 0xFFFFu);
    ASSERT_EQ(mem_read32(mem, TEST_PCNT_BASE + 0x090u) & (1u << 2),
              1u << 2); /* threshold1=-1 */
    mem_write32(mem, TEST_PCNT_BASE + 0x08Cu, 1u);
    uint32_t ctrl = test_pcnt_ctrl_running(0) | (1u << 1);
    mem_write32(mem, TEST_PCNT_BASE + 0x0B0u, ctrl); /* pause */
    test_pcnt_pulse(p, 18);
    ASSERT_EQ(mem_read32(mem, TEST_PCNT_BASE + 0x060u), 0xFFFFu);
    mem_write32(mem, TEST_PCNT_BASE + 0x0B0u,
                test_pcnt_ctrl_running(0));
    test_pcnt_pulse(p, 18);
    ASSERT_EQ(mem_read32(mem, TEST_PCNT_BASE + 0x060u), 0xFFFEu);

    test_pcnt_pulse(p, 18);
    ASSERT_EQ(mem_read32(mem, TEST_PCNT_BASE + 0x060u), 0u);
    ASSERT_EQ(mem_read32(mem, TEST_PCNT_BASE + 0x090u) & (1u << 4),
              1u << 4); /* low-limit reset */
    mem_write32(mem, TEST_PCNT_BASE + 0x08Cu, 1u);

    /* A normal sign crossing has its own ZERO event and direction mode. */
    test_pcnt_pulse(p, 18); /* reversed: -1 */
    mem_write32(mem, TEST_PCNT_BASE + 0x08Cu, 1u);
    periph_gpio_set_input(p, 19, 0);
    test_pcnt_pulse(p, 18); /* keep: -1 -> 0 */
    ASSERT_EQ(mem_read32(mem, TEST_PCNT_BASE + 0x060u), 0u);
    ASSERT_EQ(mem_read32(mem, TEST_PCNT_BASE + 0x090u) & (1u << 6),
              1u << 6);

    mem_write32(mem, TEST_PCNT_BASE + 0x0B0u,
                test_pcnt_ctrl_running(0) | 1u); /* unit reset */
    ASSERT_EQ(mem_read32(mem, TEST_PCNT_BASE + 0x060u), 0u);
    ASSERT_EQ(mem_read32(mem, TEST_PCNT_BASE + 0x090u), 0u);

    /* DPORT module reset restores all units and clears the shared IRQ while
     * leaving the external GPIO matrix wiring available for reconfiguration. */
    mem_write32(mem, 0x3FF000C4u, 1u << 10);
    ASSERT_EQ(mem_read32(mem, TEST_PCNT_BASE + 0x000u), 0x3C10u);
    ASSERT_EQ(mem_read32(mem, TEST_PCNT_BASE + 0x0B0u), 0x5555u);
    ASSERT_EQ(mem_read32(mem, TEST_PCNT_BASE + 0x080u), 0u);
    ASSERT_FALSE(periph_interrupt_pending(p, 48));
    ASSERT_EQ(periph_unhandled_count(p), 0);

    periph_destroy(p);
    mem_destroy(mem);
}

TEST(pcnt_filter_rejects_glitches_and_qualifies_edges) {
    xtensa_mem_t *mem = mem_create();
    esp32_periph_t *p = periph_create(mem);
    xtensa_cpu_t cpu;
    xtensa_cpu_init(&cpu); cpu.mem = mem;
    periph_attach_cpus(p, &cpu, NULL);
    mem_write32(mem, 0x3FFE01E0u, 240u);
    mem_write32(mem, 0x3FF000C0u, 1u << 10); /* PCNT module clock */

    /* Unit1 channel0 uses signal43. A ten-APB-cycle filter equals thirty
     * CPU cycles at 240 MHz. Falling edges are disabled. */
    test_pcnt_route_input(mem, 43u, 20u, false);
    uint32_t base = TEST_PCNT_BASE + 0x0Cu;
    mem_write32(mem, base + 0x00u,
                (1u << 18) | (1u << 10) | 10u);
    mem_write32(mem, base + 0x08u,
                ((uint32_t)(uint16_t)-100 << 16) | 100u);
    mem_write32(mem, TEST_PCNT_BASE + 0x0B0u,
                test_pcnt_ctrl_running(1));

    periph_gpio_set_input(p, 20, 1);
    cpu.ccount += 29u;
    cpu.periph_event(&cpu);
    ASSERT_EQ(mem_read32(mem, TEST_PCNT_BASE + 0x064u), 0u);
    periph_gpio_set_input(p, 20, 0); /* sub-threshold glitch */
    cpu.ccount += 30u;
    cpu.periph_event(&cpu);
    ASSERT_EQ(mem_read32(mem, TEST_PCNT_BASE + 0x064u), 0u);

    periph_gpio_set_input(p, 20, 1);
    cpu.ccount += 30u;
    cpu.periph_event(&cpu);
    ASSERT_EQ(mem_read32(mem, TEST_PCNT_BASE + 0x064u), 1u);
    periph_gpio_set_input(p, 20, 0);
    cpu.ccount += 30u;
    cpu.periph_event(&cpu); /* qualify low before another rising edge */
    periph_gpio_set_input(p, 20, 1);
    cpu.ccount += 30u;
    cpu.periph_event(&cpu);
    ASSERT_EQ(mem_read32(mem, TEST_PCNT_BASE + 0x064u), 2u);

    /* Matrix inversion turns the physical falling transition into a logical
     * positive edge and rebinds without creating a synthetic pulse. */
    test_pcnt_route_input(mem, 43u, 20u, true);
    ASSERT_EQ(mem_read32(mem, TEST_PCNT_BASE + 0x064u), 2u);
    periph_gpio_set_input(p, 20, 0);
    cpu.ccount += 30u;
    cpu.periph_event(&cpu);
    ASSERT_EQ(mem_read32(mem, TEST_PCNT_BASE + 0x064u), 3u);
    ASSERT_EQ(periph_unhandled_count(p), 0);

    periph_destroy(p);
    mem_destroy(mem);
}

#define TEST_MCPWM0_BASE 0x3FF5E000u
#define TEST_MCPWM1_BASE 0x3FF6C000u

typedef struct {
    unsigned calls;
    int unit;
    int operator_index;
    int generator;
    periph_mcpwm_output_info_t info;
} mcpwm_test_capture_t;

static void capture_mcpwm(void *ctx, int unit, int operator_index,
                          int generator,
                          const periph_mcpwm_output_info_t *info) {
    mcpwm_test_capture_t *capture = ctx;
    capture->calls++;
    capture->unit = unit;
    capture->operator_index = operator_index;
    capture->generator = generator;
    capture->info = *info;
}

static void test_mcpwm_advance(xtensa_cpu_t *cpu, uint32_t cycles) {
    cpu->ccount += cycles;
    cpu->periph_event(cpu);
}

static void test_mcpwm_route_output(xtensa_mem_t *mem, unsigned gpio,
                                    unsigned signal, bool inverted) {
    mem_write32(mem, 0x3FF44530u + gpio * 4u,
                signal | (inverted ? 1u << 9 : 0u));
}

static uint32_t test_mcpwm_timer_cfg(uint32_t prescale,
                                     uint32_t period) {
    return ((prescale - 1u) & 0xFFu) | ((period - 1u) << 8);
}

TEST(mcpwm_timed_waveform_shadow_force_and_interrupt) {
    xtensa_mem_t *mem = mem_create();
    esp32_periph_t *p = periph_create(mem);
    xtensa_cpu_t cpu;
    xtensa_cpu_init(&cpu); cpu.mem = mem;
    periph_attach_cpus(p, &cpu, NULL);
    periph_intr_matrix_set(p, 0, 8, 39); /* MCPWM0 -> external CPU interrupt 8 */
    mem_write32(mem, 0x3FFE01E0u, 240u);
    mem_write32(mem, 0x3FF000C0u, 1u << 17); /* MCPWM0 bus clock */

    ASSERT_EQ(mem_read32(mem, TEST_MCPWM0_BASE + 0x004u), 0x0000FF00u);
    ASSERT_EQ(mem_read32(mem, TEST_MCPWM0_BASE + 0x04Cu), 0x00000020u);
    ASSERT_EQ(mem_read32(mem, TEST_MCPWM0_BASE + 0x058u), 0x00018000u);
    ASSERT_EQ(mem_read32(mem, TEST_MCPWM0_BASE + 0x10Cu), 0x00000055u);
    ASSERT_EQ(mem_read32(mem, TEST_MCPWM0_BASE + 0x124u), 0x02107230u);

    mcpwm_test_capture_t capture = {0};
    ASSERT_EQ(periph_set_mcpwm_output_callback(
                  p, 0, 0, 0, capture_mcpwm, &capture), 0);
    ASSERT_EQ(periph_set_mcpwm_output_callback(
                  p, 2, 0, 0, capture_mcpwm, &capture), -1);
    test_mcpwm_route_output(mem, 18u, 32u, false);

    /* 160 MHz / 16 / 10 / 1000 = 1 kHz. Generator A is high from TEZ
     * through compare A=250, then low through the rest of the cycle. */
    mem_write32(mem, TEST_MCPWM0_BASE + 0x000u, 15u);
    mem_write32(mem, TEST_MCPWM0_BASE + 0x004u,
                test_mcpwm_timer_cfg(10u, 1000u));
    mem_write32(mem, TEST_MCPWM0_BASE + 0x040u, 250u);
    mem_write32(mem, TEST_MCPWM0_BASE + 0x050u,
                (2u << 0) | (1u << 4));
    mem_write32(mem, TEST_MCPWM0_BASE + 0x008u, 2u | (1u << 3));
    ASSERT_EQ(capture.unit, 0);
    ASSERT_EQ(capture.operator_index, 0);
    ASSERT_EQ(capture.generator, 0);
    ASSERT_EQ(capture.info.gpio, 18);
    ASSERT_EQ(capture.info.frequency_hz, 1000u);
    ASSERT_EQ(capture.info.period_ticks, 1000u);
    ASSERT_EQ(capture.info.compare_ticks, 250u);
    ASSERT_EQ(capture.info.count_mode, 1u);
    ASSERT_TRUE(capture.info.enabled);
    ASSERT_EQ(periph_gpio_pin_level(p, 18), 1);

    mem_write32(mem, TEST_MCPWM0_BASE + 0x11Cu, 0x3FFFFFFFu);
    mem_write32(mem, TEST_MCPWM0_BASE + 0x110u, 1u << 15);
    test_mcpwm_advance(&cpu, 59999u);
    ASSERT_EQ(mem_read32(mem, TEST_MCPWM0_BASE + 0x010u) & 0xFFFFu,
              249u);
    ASSERT_EQ(periph_gpio_pin_level(p, 18), 1);
    ASSERT_EQ(cpu.interrupt & (1u << 8), 0u);
    test_mcpwm_advance(&cpu, 1u);
    ASSERT_EQ(mem_read32(mem, TEST_MCPWM0_BASE + 0x010u) & 0xFFFFu,
              250u);
    ASSERT_EQ(periph_gpio_pin_level(p, 18), 0);
    ASSERT_EQ(mem_read32(mem, TEST_MCPWM0_BASE + 0x114u) & (1u << 15),
              1u << 15);
    ASSERT_EQ(mem_read32(mem, TEST_MCPWM0_BASE + 0x118u), 1u << 15);
    ASSERT_EQ(cpu.interrupt & (1u << 8), 1u << 8);
    mem_write32(mem, TEST_MCPWM0_BASE + 0x11Cu, 1u << 15);

    /* A shadow compare update requested for TEZ leaves this cycle intact and
     * becomes visible atomically at the next zero boundary. */
    mem_write32(mem, TEST_MCPWM0_BASE + 0x03Cu, 1u);
    mem_write32(mem, TEST_MCPWM0_BASE + 0x040u, 400u);
    ASSERT_EQ(capture.info.compare_ticks, 250u);
    ASSERT_EQ(mem_read32(mem, TEST_MCPWM0_BASE + 0x03Cu) & (1u << 8),
              1u << 8);
    test_mcpwm_advance(&cpu, 180000u);
    ASSERT_EQ(mem_read32(mem, TEST_MCPWM0_BASE + 0x010u) & 0xFFFFu, 0u);
    ASSERT_EQ(capture.info.compare_ticks, 400u);
    ASSERT_EQ(mem_read32(mem, TEST_MCPWM0_BASE + 0x03Cu) & (1u << 8), 0u);
    ASSERT_EQ(periph_gpio_pin_level(p, 18), 1);
    test_mcpwm_advance(&cpu, 96000u);
    ASSERT_EQ(periph_gpio_pin_level(p, 18), 0);

    /* Continuous software force supersedes the event generator, then
     * releasing it exposes the live waveform again. */
    mem_write32(mem, TEST_MCPWM0_BASE + 0x04Cu, 2u << 6);
    ASSERT_EQ(capture.info.forced_level, 1);
    ASSERT_EQ(periph_gpio_pin_level(p, 18), 1);
    mem_write32(mem, TEST_MCPWM0_BASE + 0x04Cu, 1u << 6);
    ASSERT_EQ(capture.info.forced_level, 0);
    ASSERT_EQ(periph_gpio_pin_level(p, 18), 0);
    mem_write32(mem, TEST_MCPWM0_BASE + 0x04Cu, 0u);
    ASSERT_EQ(capture.info.forced_level, -1);
    ASSERT_EQ(periph_gpio_pin_level(p, 18), 0);

    test_mcpwm_route_output(mem, 18u, 32u, true);
    ASSERT_TRUE(capture.info.inverted);
    ASSERT_EQ(periph_gpio_pin_level(p, 18), 1);

    unsigned calls_before_reset = capture.calls;
    mem_write32(mem, 0x3FF000C4u, 1u << 17);
    ASSERT_TRUE(capture.calls > calls_before_reset);
    ASSERT_FALSE(capture.info.enabled);
    ASSERT_EQ(capture.info.compare_ticks, 0u);
    ASSERT_EQ(mem_read32(mem, TEST_MCPWM0_BASE + 0x004u), 0x0000FF00u);
    ASSERT_EQ(mem_read32(mem, TEST_MCPWM0_BASE + 0x114u), 0u);
    ASSERT_FALSE(periph_interrupt_pending(p, 39));
    ASSERT_EQ(periph_unhandled_count(p), 0);

    periph_destroy(p);
    mem_destroy(mem);
}

TEST(mcpwm_units_and_upper_operators_are_independent) {
    xtensa_mem_t *mem = mem_create();
    esp32_periph_t *p = periph_create(mem);
    xtensa_cpu_t cpu;
    xtensa_cpu_init(&cpu); cpu.mem = mem;
    periph_attach_cpus(p, &cpu, NULL);
    mem_write32(mem, 0x3FFE01E0u, 240u);
    mem_write32(mem, 0x3FF000C0u, (1u << 17) | (1u << 20));

    mcpwm_test_capture_t unit0 = {0};
    mcpwm_test_capture_t unit1 = {0};
    ASSERT_EQ(periph_set_mcpwm_output_callback(
                  p, 0, 0, 0, capture_mcpwm, &unit0), 0);
    ASSERT_EQ(periph_set_mcpwm_output_callback(
                  p, 1, 2, 1, capture_mcpwm, &unit1), 0);
    test_mcpwm_route_output(mem, 18u, 32u, false);
    test_mcpwm_route_output(mem, 23u, 113u, false);

    mem_write32(mem, TEST_MCPWM0_BASE + 0x000u, 15u);
    mem_write32(mem, TEST_MCPWM0_BASE + 0x004u,
                test_mcpwm_timer_cfg(10u, 1000u));
    mem_write32(mem, TEST_MCPWM0_BASE + 0x040u, 250u);
    mem_write32(mem, TEST_MCPWM0_BASE + 0x050u,
                (2u << 0) | (1u << 4));
    mem_write32(mem, TEST_MCPWM0_BASE + 0x008u, 2u | (1u << 3));

    /* Unit 1, operator 2/generator B runs from timer 2 through the upper
     * register and GPIO-matrix signal ranges. */
    mem_write32(mem, TEST_MCPWM1_BASE + 0x000u, 31u);
    mem_write32(mem, TEST_MCPWM1_BASE + 0x038u, 2u << 4);
    mem_write32(mem, TEST_MCPWM1_BASE + 0x024u,
                test_mcpwm_timer_cfg(8u, 625u));
    mem_write32(mem, TEST_MCPWM1_BASE + 0x0B4u, 312u);
    mem_write32(mem, TEST_MCPWM1_BASE + 0x0C4u,
                (2u << 0) | (1u << 6));
    mem_write32(mem, TEST_MCPWM1_BASE + 0x028u, 2u | (1u << 3));

    ASSERT_EQ(unit0.info.frequency_hz, 1000u);
    ASSERT_EQ(unit0.info.gpio, 18);
    ASSERT_EQ(unit1.unit, 1);
    ASSERT_EQ(unit1.operator_index, 2);
    ASSERT_EQ(unit1.generator, 1);
    ASSERT_EQ(unit1.info.gpio, 23);
    ASSERT_EQ(unit1.info.frequency_hz, 1000u);
    ASSERT_EQ(unit1.info.period_ticks, 625u);
    ASSERT_EQ(unit1.info.compare_ticks, 312u);
    ASSERT_TRUE(unit0.info.enabled);
    ASSERT_TRUE(unit1.info.enabled);

    unsigned unit0_calls = unit0.calls;
    mem_write32(mem, 0x3FF000C4u, 1u << 20);
    ASSERT_TRUE(unit1.calls > 0u);
    ASSERT_FALSE(unit1.info.enabled);
    ASSERT_TRUE(unit0.info.enabled);
    ASSERT_EQ(unit0.calls, unit0_calls);
    ASSERT_EQ(mem_read32(mem, TEST_MCPWM1_BASE + 0x024u), 0x0000FF00u);
    ASSERT_EQ(mem_read32(mem, TEST_MCPWM0_BASE + 0x004u),
              test_mcpwm_timer_cfg(10u, 1000u));
    ASSERT_EQ(mem_read32(mem, TEST_MCPWM1_BASE + 0x124u), 0x02107230u);
    ASSERT_EQ(periph_unhandled_count(p), 0);

    periph_destroy(p);
    mem_destroy(mem);
}

TEST(mcpwm_dual_core_clock_and_deferred_stop_boundary) {
    xtensa_mem_t *mem = mem_create();
    esp32_periph_t *p = periph_create(mem);
    xtensa_cpu_t cpu0;
    xtensa_cpu_t cpu1;
    xtensa_cpu_init(&cpu0); cpu0.mem = mem;
    xtensa_cpu_init(&cpu1); cpu1.mem = mem; cpu1.core_id = 1;
    periph_attach_cpus(p, &cpu0, &cpu1);
    mem_write32(mem, 0x3FFE01E0u, 240u);
    mem_write32(mem, 0x3FF000C0u, 1u << 17);

    mcpwm_test_capture_t capture = {0};
    ASSERT_EQ(periph_set_mcpwm_output_callback(
                  p, 0, 0, 0, capture_mcpwm, &capture), 0);
    test_mcpwm_route_output(mem, 18u, 32u, false);
    mem_write32(mem, TEST_MCPWM0_BASE + 0x000u, 15u);
    mem_write32(mem, TEST_MCPWM0_BASE + 0x004u,
                test_mcpwm_timer_cfg(10u, 1000u));
    mem_write32(mem, TEST_MCPWM0_BASE + 0x040u, 250u);
    mem_write32(mem, TEST_MCPWM0_BASE + 0x050u,
                (2u << 0) | (1u << 4));
    mem_write32(mem, TEST_MCPWM0_BASE + 0x008u, 2u | (1u << 3));
    ASSERT_TRUE(capture.info.enabled);
    ASSERT_EQ(periph_gpio_pin_level(p, 18), 1);

    /* Both emulated cores cover the same wall-clock interval. Their shared
     * MCPWM clock takes the maximum progress instead of double-counting the
     * sequential host execution of core 0 and core 1. */
    cpu0.ccount += 60000u;
    cpu1.ccount += 60000u;
    ASSERT_EQ(mem_read32(mem, TEST_MCPWM0_BASE + 0x010u) & 0xFFFFu,
              250u);
    ASSERT_EQ(periph_gpio_pin_level(p, 18), 0);

    /* A driver task running only on APP CPU can update an operator and clock
     * it through the next boundary while PRO CPU remains stationary. */
    mem_write32(mem, TEST_MCPWM0_BASE + 0x050u,
                (2u << 0) | (2u << 2) | (2u << 4));
    cpu1.ccount += 180000u;
    ASSERT_EQ(mem_read32(mem, TEST_MCPWM0_BASE + 0x010u) & 0xFFFFu, 0u);
    ASSERT_EQ(periph_gpio_pin_level(p, 18), 1);

    /* STOP_EMPTY needs a future TEZ even when no timer interrupt is enabled.
     * It must therefore remain in the CPU deadline set until that boundary. */
    mem_write32(mem, TEST_MCPWM0_BASE + 0x008u, 1u << 3);
    ASSERT_TRUE(cpu0.next_timer_event != UINT32_MAX);
    cpu1.ccount += 240000u;
    (void)mem_read32(mem, TEST_MCPWM0_BASE + 0x010u);
    ASSERT_FALSE(capture.info.enabled);
    ASSERT_EQ(periph_unhandled_count(p), 0);

    periph_destroy(p);
    mem_destroy(mem);
}

TEST(mcpwm_gpio_capture_sync_fault_deadtime_and_carrier) {
    xtensa_mem_t *mem = mem_create();
    esp32_periph_t *p = periph_create(mem);
    xtensa_cpu_t cpu;
    xtensa_cpu_init(&cpu); cpu.mem = mem;
    periph_attach_cpus(p, &cpu, NULL);
    periph_intr_matrix_set(p, 0, 5, 39);
    mem_write32(mem, 0x3FFE01E0u, 240u);
    mem_write32(mem, 0x3FF000C0u, 1u << 17);
    mem_write32(mem, TEST_MCPWM0_BASE + 0x000u, 15u);

    /* Timer 1 runs at 1 MHz. GPIO SYNC0 reloads it to count 123, and a
     * software-sync toggle subsequently reloads a different phase. */
    mem_write32(mem, TEST_MCPWM0_BASE + 0x014u,
                test_mcpwm_timer_cfg(10u, 1000u));
    mem_write32(mem, TEST_MCPWM0_BASE + 0x034u, 4u << 3);
    mem_write32(mem, TEST_MCPWM0_BASE + 0x01Cu,
                1u | (123u << 4));
    test_pcnt_route_input(mem, 31u, 32u, false);
    mem_write32(mem, TEST_MCPWM0_BASE + 0x018u, 2u | (1u << 3));
    test_mcpwm_advance(&cpu, 24000u);
    ASSERT_EQ(mem_read32(mem, TEST_MCPWM0_BASE + 0x020u) & 0xFFFFu,
              100u);
    periph_gpio_set_input(p, 32, 1);
    ASSERT_EQ(mem_read32(mem, TEST_MCPWM0_BASE + 0x020u) & 0xFFFFu,
              123u);
    periph_gpio_set_input(p, 32, 0);
    /* An asynchronous GPIO edge first catches the timer up to the edge time,
     * then applies the phase reload. Stale elapsed time must not be charged a
     * second time after the synchronized count is installed. */
    cpu.ccount += 24000u;
    periph_gpio_set_input(p, 32, 1);
    ASSERT_EQ(mem_read32(mem, TEST_MCPWM0_BASE + 0x020u) & 0xFFFFu,
              123u);
    periph_gpio_set_input(p, 32, 0);
    mem_write32(mem, TEST_MCPWM0_BASE + 0x01Cu,
                1u | (77u << 4) | (1u << 1));
    ASSERT_EQ(mem_read32(mem, TEST_MCPWM0_BASE + 0x020u) & 0xFFFFu,
              77u);

    /* The independent APB capture timer timestamps real matrix edges and
     * reports both edge direction and source-39 interrupt state. */
    test_pcnt_route_input(mem, 109u, 34u, false);
    mem_write32(mem, TEST_MCPWM0_BASE + 0x0F0u,
                1u | (1u << 1) | (1u << 2));
    mem_write32(mem, TEST_MCPWM0_BASE + 0x0E8u, 1u);
    mem_write32(mem, TEST_MCPWM0_BASE + 0x110u, 1u << 27);
    test_mcpwm_advance(&cpu, 300u);
    periph_gpio_set_input(p, 34, 1);
    ASSERT_EQ(mem_read32(mem, TEST_MCPWM0_BASE + 0x0FCu), 100u);
    ASSERT_EQ(mem_read32(mem, TEST_MCPWM0_BASE + 0x108u) & 1u, 0u);
    ASSERT_EQ(mem_read32(mem, TEST_MCPWM0_BASE + 0x118u), 1u << 27);
    ASSERT_EQ(cpu.interrupt & (1u << 5), 1u << 5);
    mem_write32(mem, TEST_MCPWM0_BASE + 0x11Cu, 1u << 27);
    test_mcpwm_advance(&cpu, 150u);
    periph_gpio_set_input(p, 34, 0);
    ASSERT_EQ(mem_read32(mem, TEST_MCPWM0_BASE + 0x0FCu), 150u);
    ASSERT_EQ(mem_read32(mem, TEST_MCPWM0_BASE + 0x108u) & 1u, 1u);
    mem_write32(mem, TEST_MCPWM0_BASE + 0x11Cu, 1u << 27);
    test_mcpwm_advance(&cpu, 60u);
    mem_write32(mem, TEST_MCPWM0_BASE + 0x0F0u,
                1u | (1u << 1) | (1u << 2) | (1u << 12));
    ASSERT_EQ(mem_read32(mem, TEST_MCPWM0_BASE + 0x0FCu), 170u);
    ASSERT_EQ(mem_read32(mem, TEST_MCPWM0_BASE + 0x0F0u) & (1u << 12), 0u);
    ASSERT_EQ(mem_read32(mem, TEST_MCPWM0_BASE + 0x108u) & 1u, 0u);

    /* Operator 0 follows timer 1. Its endpoint reports dead-time/carrier
     * configuration and fault actions while raw GPIO levels show override
     * persistence for cycle-by-cycle and one-shot braking. */
    mcpwm_test_capture_t output = {0};
    ASSERT_EQ(periph_set_mcpwm_output_callback(
                  p, 0, 0, 0, capture_mcpwm, &output), 0);
    test_mcpwm_route_output(mem, 18u, 32u, false);
    mem_write32(mem, TEST_MCPWM0_BASE + 0x038u, 1u);
    mem_write32(mem, TEST_MCPWM0_BASE + 0x04Cu, 2u << 6);
    mem_write32(mem, TEST_MCPWM0_BASE + 0x05Cu, 7u);
    mem_write32(mem, TEST_MCPWM0_BASE + 0x060u, 11u);
    mem_write32(mem, TEST_MCPWM0_BASE + 0x058u,
                (1u << 15) | (1u << 16));
    mem_write32(mem, TEST_MCPWM0_BASE + 0x064u,
                1u | (3u << 1) | (5u << 5));
    ASSERT_EQ(output.info.rising_delay_ticks, 11u);
    ASSERT_EQ(output.info.falling_delay_ticks, 7u);
    ASSERT_EQ(output.info.deadtime_clock_hz, 10000000u);
    ASSERT_EQ(output.info.carrier_hz, 312500u);
    ASSERT_EQ(output.info.carrier_duty_eighths, 5u);
    ASSERT_EQ(periph_gpio_pin_level(p, 18), 1);

    test_pcnt_route_input(mem, 34u, 35u, false);
    mem_write32(mem, TEST_MCPWM0_BASE + 0x068u,
                (1u << 3) | (1u << 8) | (1u << 10));
    mem_write32(mem, TEST_MCPWM0_BASE + 0x0E4u,
                (1u << 0) | (1u << 3)); /* fault0 active high */
    periph_gpio_set_input(p, 35, 1);
    ASSERT_EQ(mem_read32(mem, TEST_MCPWM0_BASE + 0x0E4u) & (1u << 6),
              1u << 6);
    ASSERT_EQ(mem_read32(mem, TEST_MCPWM0_BASE + 0x070u) & 1u, 1u);
    ASSERT_EQ(mem_read32(mem, TEST_MCPWM0_BASE + 0x114u) &
              ((1u << 9) | (1u << 21)), (1u << 9) | (1u << 21));
    ASSERT_TRUE(output.info.fault_active);
    ASSERT_EQ(output.info.forced_level, 0);
    ASSERT_EQ(periph_gpio_pin_level(p, 18), 0);
    periph_gpio_set_input(p, 35, 0);
    ASSERT_EQ(mem_read32(mem, TEST_MCPWM0_BASE + 0x070u) & 1u, 0u);
    ASSERT_FALSE(output.info.fault_active);
    ASSERT_EQ(periph_gpio_pin_level(p, 18), 1);

    mem_write32(mem, TEST_MCPWM0_BASE + 0x068u,
                (1u << 7) | (1u << 12) | (1u << 14));
    periph_gpio_set_input(p, 35, 1);
    ASSERT_EQ(mem_read32(mem, TEST_MCPWM0_BASE + 0x070u) & 2u, 2u);
    ASSERT_EQ(periph_gpio_pin_level(p, 18), 0);
    periph_gpio_set_input(p, 35, 0);
    ASSERT_EQ(mem_read32(mem, TEST_MCPWM0_BASE + 0x070u) & 2u, 2u);
    ASSERT_EQ(periph_gpio_pin_level(p, 18), 0);
    mem_write32(mem, TEST_MCPWM0_BASE + 0x06Cu, 1u);
    ASSERT_EQ(mem_read32(mem, TEST_MCPWM0_BASE + 0x070u), 0u);
    ASSERT_EQ(periph_gpio_pin_level(p, 18), 1);
    ASSERT_EQ(periph_unhandled_count(p), 0);

    periph_destroy(p);
    mem_destroy(mem);
}

typedef struct {
    unsigned calls;
    int speed_mode;
    int channel;
    int gpio;
    uint32_t frequency_hz;
    uint32_t duty;
    uint32_t duty_max;
    bool enabled;
    bool inverted;
} ledc_capture_t;

static void capture_ledc(void *ctx, int speed_mode, int channel, int gpio,
                         uint32_t frequency_hz, uint32_t duty,
                         uint32_t duty_max, bool enabled, bool inverted) {
    ledc_capture_t *capture = ctx;
    capture->calls++;
    capture->speed_mode = speed_mode;
    capture->channel = channel;
    capture->gpio = gpio;
    capture->frequency_hz = frequency_hz;
    capture->duty = duty;
    capture->duty_max = duty_max;
    capture->enabled = enabled;
    capture->inverted = inverted;
}

static void ledc_advance(xtensa_cpu_t *cpu, uint32_t cycles) {
    cpu->ccount += cycles;
    cpu->periph_event(cpu);
}

static uint32_t test_ledc_timer_conf(uint32_t resolution,
                                     uint32_t divider_q8) {
    return (1u << 25) | (divider_q8 << 5) | resolution;
}

static uint32_t test_ledc_update_conf(uint32_t scale, uint32_t cycle_count,
                                      uint32_t steps, bool increase) {
    return (1u << 31) | (increase ? 1u << 30 : 0u) |
           ((steps & 0x3FFu) << 20) |
           ((cycle_count & 0x3FFu) << 10) | (scale & 0x3FFu);
}

TEST(ledc_timed_duty_update_and_gpio_output) {
    xtensa_mem_t *mem = mem_create();
    esp32_periph_t *p = periph_create(mem);
    xtensa_cpu_t cpu0;
    xtensa_cpu_init(&cpu0); cpu0.mem = mem;
    periph_attach_cpus(p, &cpu0, NULL);
    periph_intr_matrix_set(p, 0, 4, 43); /* LEDC -> CPU interrupt 4 */

    const uint32_t base = 0x3FF59000u;
    ASSERT_EQ(mem_read32(mem, base + 0x140), 1u << 24); /* timer reset */
    mem_write32(mem, 0x3FFE01E0u, 240u);
    uint32_t timer_conf = test_ledc_timer_conf(8u, 16000u);
    mem_write32(mem, base + 0x140, timer_conf); /* APB / 62.5 / 256 = 5 kHz */
    ASSERT_EQ(mem_read32(mem, base + 0x140), timer_conf);

    ledc_capture_t capture = {0};
    ASSERT_EQ(periph_set_ledc_output_callback(p, 0, 0,
                                               capture_ledc, &capture), 0);
    mem_write32(mem, 0x3FF44584u, 71u); /* GPIO21 <- LEDC_HS_SIG_OUT0 */
    mem_write32(mem, base + 0x000, 1u << 2); /* timer0 + signal enable */
    mem_write32(mem, base + 0x008, 64u << 4); /* Q21.4 duty */
    mem_write32(mem, base + 0x188, 1u << 8);     /* duty-end interrupt enable */
    mem_write32(mem, base + 0x00C,
                test_ledc_update_conf(0u, 1u, 1u, true));

    /* Updates latch at the next PWM boundary, not in the programming write. */
    ASSERT_EQ(mem_read32(mem, base + 0x010), 0u);
    ASSERT_EQ(mem_read32(mem, base + 0x00C) & (1u << 31), 1u << 31);
    ledc_advance(&cpu0, 24000u);
    ASSERT_EQ(mem_read32(mem, base + 0x144), 128u); /* live timer counter */
    ASSERT_EQ(mem_read32(mem, base + 0x010), 0u);
    ledc_advance(&cpu0, 23999u);
    ASSERT_EQ(mem_read32(mem, base + 0x010), 0u);
    ledc_advance(&cpu0, 1u);
    ASSERT_EQ(mem_read32(mem, base + 0x010), 64u << 4);
    ASSERT_EQ(mem_read32(mem, base + 0x00C) & (1u << 31), 0u);
    ASSERT_EQ(mem_read32(mem, base + 0x180) & (1u << 8), 1u << 8);
    ASSERT_EQ(mem_read32(mem, base + 0x184) & (1u << 8), 1u << 8);
    ASSERT_EQ(cpu0.interrupt & (1u << 4), 1u << 4);
    ASSERT_TRUE(capture.calls >= 2u);
    ASSERT_EQ(capture.speed_mode, 0);
    ASSERT_EQ(capture.channel, 0);
    ASSERT_EQ(capture.gpio, 21);
    ASSERT_EQ(capture.frequency_hz, 5000u);
    ASSERT_EQ(capture.duty, 64u);
    ASSERT_EQ(capture.duty_max, 255u);
    ASSERT_TRUE(capture.enabled);
    ASSERT_FALSE(capture.inverted);

    mem_write32(mem, base + 0x18C, 1u << 8);
    ASSERT_EQ(mem_read32(mem, base + 0x180) & (1u << 8), 0u);
    ASSERT_EQ(cpu0.interrupt & (1u << 4), 0u);
    ASSERT_EQ(mem_read32(mem, base + 0x1FC), 0x16031700u);
    ASSERT_EQ(periph_unhandled_count(p), 0);

    periph_destroy(p);
    mem_destroy(mem);
}

TEST(ledc_fade_progress_and_completion_deadline) {
    xtensa_mem_t *mem = mem_create();
    esp32_periph_t *p = periph_create(mem);
    xtensa_cpu_t cpu;
    xtensa_cpu_init(&cpu); cpu.mem = mem;
    periph_attach_cpus(p, &cpu, NULL);
    const uint32_t base = 0x3FF59000u;
    mem_write32(mem, 0x3FFE01E0u, 240u);
    mem_write32(mem, base + 0x140, test_ledc_timer_conf(8u, 16000u));
    mem_write32(mem, base + 0x000, 1u << 2);
    mem_write32(mem, base + 0x008, 64u << 4);
    mem_write32(mem, base + 0x00C,
                test_ledc_update_conf(0u, 1u, 1u, true));
    ledc_advance(&cpu, 48000u);
    mem_write32(mem, base + 0x18C, 1u << 8);

    /* Start at 64, then add two duty counts every three PWM periods ten
     * times. Hardware exposes the intermediate duty through DUTY_R. */
    mem_write32(mem, base + 0x008, 64u << 4);
    mem_write32(mem, base + 0x188, 1u << 8);
    mem_write32(mem, base + 0x00C,
                test_ledc_update_conf(2u, 3u, 10u, true));
    ledc_advance(&cpu, 48000u); /* starting duty latches */
    ASSERT_EQ(mem_read32(mem, base + 0x010), 64u << 4);
    ASSERT_EQ(mem_read32(mem, base + 0x180) & (1u << 8), 0u);

    ledc_advance(&cpu, 4u * 3u * 48000u);
    ASSERT_EQ(mem_read32(mem, base + 0x010), 72u << 4);
    ASSERT_EQ(mem_read32(mem, base + 0x00C) & (1u << 31), 1u << 31);
    ledc_advance(&cpu, 6u * 3u * 48000u);
    ASSERT_EQ(mem_read32(mem, base + 0x010), 84u << 4);
    ASSERT_EQ(mem_read32(mem, base + 0x00C) & (1u << 31), 0u);
    ASSERT_EQ(mem_read32(mem, base + 0x180) & (1u << 8), 1u << 8);
    ASSERT_TRUE(periph_interrupt_pending(p, 43));
    mem_write32(mem, base + 0x18C, 1u << 8);
    ASSERT_FALSE(periph_interrupt_pending(p, 43));
    ASSERT_EQ(periph_unhandled_count(p), 0);

    periph_destroy(p);
    mem_destroy(mem);
}

TEST(ledc_timer_overflow_pause_and_clear) {
    xtensa_mem_t *mem = mem_create();
    esp32_periph_t *p = periph_create(mem);
    xtensa_cpu_t cpu;
    xtensa_cpu_init(&cpu); cpu.mem = mem;
    periph_attach_cpus(p, &cpu, NULL);
    periph_intr_matrix_set(p, 0, 8, 43);
    const uint32_t base = 0x3FF59000u;
    mem_write32(mem, 0x3FFE01E0u, 240u);
    uint32_t conf = test_ledc_timer_conf(8u, 16000u);
    mem_write32(mem, base + 0x140, conf);
    mem_write32(mem, base + 0x188, 1u); /* timer0 overflow */
    ledc_advance(&cpu, 47999u);
    ASSERT_EQ(mem_read32(mem, base + 0x180) & 1u, 0u);
    ledc_advance(&cpu, 1u);
    ASSERT_EQ(mem_read32(mem, base + 0x180) & 1u, 1u);
    ASSERT_EQ(mem_read32(mem, base + 0x184) & 1u, 1u);
    ASSERT_EQ(cpu.interrupt & (1u << 8), 1u << 8);
    mem_write32(mem, base + 0x18C, 1u);
    ASSERT_FALSE(periph_interrupt_pending(p, 43));

    mem_write32(mem, base + 0x140, conf | (1u << 23)); /* pause */
    uint32_t paused = mem_read32(mem, base + 0x144);
    ledc_advance(&cpu, 96000u);
    ASSERT_EQ(mem_read32(mem, base + 0x144), paused);
    ASSERT_EQ(mem_read32(mem, base + 0x180) & 1u, 0u);
    mem_write32(mem, base + 0x140, conf | (1u << 24)); /* reset */
    ASSERT_EQ(mem_read32(mem, base + 0x144), 0u);
    ASSERT_EQ(periph_unhandled_count(p), 0);

    periph_destroy(p);
    mem_destroy(mem);
}

TEST(ledc_low_speed_output_and_dport_reset_preserve_endpoint) {
    xtensa_mem_t *mem = mem_create();
    esp32_periph_t *p = periph_create(mem);
    xtensa_cpu_t cpu;
    xtensa_cpu_init(&cpu); cpu.mem = mem;
    periph_attach_cpus(p, &cpu, NULL);
    const uint32_t base = 0x3FF59000u;
    mem_write32(mem, 0x3FFE01E0u, 240u);

    ledc_capture_t capture = {0};
    ASSERT_EQ(periph_set_ledc_output_callback(p, 1, 3,
                                               capture_ledc, &capture), 0);
    mem_write32(mem, 0x3FF44584u, 82u); /* GPIO21 <- LS channel3 */
    mem_write32(mem, base + 0x0DC, (3u & 0x3u) | (1u << 2));
    mem_write32(mem, base + 0x0E4, 200u << 4);
    mem_write32(mem, base + 0x0E8,
                test_ledc_update_conf(0u, 1u, 1u, true));
    ASSERT_EQ(mem_read32(mem, base + 0x0EC), 0u);

    /* DUTY_START waits for its selected timer. Start timer3 on RTC8M, then
     * change the shared low-speed source to APB without discontinuity. */
    mem_write32(mem, base + 0x178,
                (1u << 26) | test_ledc_timer_conf(8u, 16000u));
    ledc_advance(&cpu, 18750u);
    ASSERT_EQ(mem_read32(mem, base + 0x17C), 10u);
    mem_write32(mem, base + 0x190, 1u); /* low-speed slow_clk = APB */
    ASSERT_EQ(mem_read32(mem, base + 0x17C), 10u);
    ledc_advance(&cpu, 46124u);
    ASSERT_EQ(mem_read32(mem, base + 0x0EC), 0u);
    ledc_advance(&cpu, 1u);
    ASSERT_EQ(mem_read32(mem, base + 0x0EC), 200u << 4);
    ASSERT_EQ(capture.speed_mode, 1);
    ASSERT_EQ(capture.channel, 3);
    ASSERT_EQ(capture.gpio, 21);
    ASSERT_EQ(capture.frequency_hz, 5000u);
    ASSERT_EQ(capture.duty, 200u);
    ASSERT_TRUE(capture.enabled);

    unsigned before_reset = capture.calls;
    mem_write32(mem, 0x3FF000C4u, 1u << 11);
    ASSERT_TRUE(capture.calls > before_reset);
    ASSERT_FALSE(capture.enabled);
    ASSERT_EQ(capture.duty, 0u);
    ASSERT_EQ(mem_read32(mem, base + 0x140), 1u << 24);
    ASSERT_EQ(mem_read32(mem, base + 0x178), 1u << 24);
    ASSERT_EQ(mem_read32(mem, base + 0x180), 0u);
    ASSERT_EQ(mem_read32(mem, base + 0x188), 0u);
    ASSERT_EQ(mem_read32(mem, base + 0x1FC), 0x16031700u);
    ASSERT_EQ(periph_unhandled_count(p), 0);

    periph_destroy(p);
    mem_destroy(mem);
}

typedef struct {
    int count;
    int port;
    size_t len;
    uint32_t sample_rate;
    uint8_t bits_per_sample;
    uint8_t channels;
    uint8_t data[16];
} i2s_test_capture_t;

static void i2s_test_capture(void *ctx, int port, const uint8_t *data,
                             size_t len, uint32_t sample_rate,
                             uint8_t bits_per_sample, uint8_t channels) {
    i2s_test_capture_t *capture = ctx;
    capture->count++;
    capture->port = port;
    capture->len = len;
    capture->sample_rate = sample_rate;
    capture->bits_per_sample = bits_per_sample;
    capture->channels = channels;
    size_t copy_len = len < sizeof(capture->data) ? len : sizeof(capture->data);
    memcpy(capture->data, data, copy_len);
}

TEST(i2s_tx_dma_descriptor_and_interrupt) {
    const uint32_t i2s = 0x3FF4F000u;
    const uint32_t desc = 0x3FFB1300u;
    const uint32_t buf = 0x3FFB2300u;
    static const uint8_t expected[8] = {
        0x10, 0x32, 0x54, 0x76, 0x98, 0xBA, 0xDC, 0xFE,
    };
    xtensa_mem_t *mem = mem_create();
    esp32_periph_t *p = periph_create(mem);
    xtensa_cpu_t cpu0;
    xtensa_cpu_init(&cpu0);
    cpu0.mem = mem;
    periph_attach_cpus(p, &cpu0, NULL);
    periph_intr_matrix_set(p, 0, 8, 32); /* I2S0 -> CPU interrupt 8 */

    i2s_test_capture_t capture = {0};
    ASSERT_EQ(periph_set_i2s_tx_callback(p, 0, i2s_test_capture, &capture), 0);
    for (size_t i = 0; i < sizeof(expected); i++)
        mem_write8(mem, buf + (uint32_t)i, expected[i]);
    test_spi_dma_desc(mem, desc, buf, sizeof(expected), sizeof(expected),
                      1, desc);

    mem_write32(mem, i2s + 0x14u, 1u << 12); /* OUT_EOF_INT_ENA */
    mem_write32(mem, i2s + 0x30u,
                (desc & 0xFFFFFu) | (1u << 29)); /* OUTLINK_START */
    mem_write32(mem, i2s + 0x08u,
                mem_read32(mem, i2s + 0x08u) | (1u << 4)); /* TX_START */

    ASSERT_EQ(capture.count, 1);
    ASSERT_EQ(capture.port, 0);
    ASSERT_EQ(capture.len, sizeof(expected));
    ASSERT_TRUE(capture.sample_rate > 0);
    ASSERT_EQ(capture.bits_per_sample, 16);
    ASSERT_EQ(capture.channels, 2);
    ASSERT_TRUE(memcmp(capture.data, expected, sizeof(expected)) == 0);
    ASSERT_EQ(mem_read32(mem, i2s + 0x0Cu) & ((1u << 11) | (1u << 12)),
              (1u << 11) | (1u << 12));
    ASSERT_EQ(mem_read32(mem, i2s + 0x10u), 1u << 12);
    ASSERT_EQ(mem_read32(mem, i2s + 0x38u), desc);
    ASSERT_EQ(mem_read32(mem, i2s + 0x40u), buf);
    ASSERT_EQ(mem_read32(mem, i2s + 0x54u), desc);
    ASSERT_EQ(mem_read32(mem, i2s + 0x58u), desc);
    ASSERT_EQ(mem_read32(mem, i2s + 0x5Cu), buf);
    ASSERT_EQ(cpu0.interrupt & (1u << 8), 1u << 8);

    mem_write32(mem, i2s + 0x18u, 1u << 12); /* OUT_EOF_INT_CLR */
    ASSERT_EQ(mem_read32(mem, i2s + 0x10u), 0);
    ASSERT_EQ(cpu0.interrupt & (1u << 8), 0);

    /* The circular descriptor remains armed and completes again at the
     * sample-rate-derived peripheral timer deadline. */
    ASSERT_TRUE(cpu0.periph_event != NULL);
    ASSERT_TRUE(cpu0.next_timer_event != UINT32_MAX);
    if (cpu0.periph_event && cpu0.next_timer_event != UINT32_MAX) {
        cpu0.ccount = cpu0.next_timer_event;
        cpu0.periph_event(&cpu0);
    }
    ASSERT_EQ(capture.count, 2);
    ASSERT_EQ(mem_read32(mem, i2s + 0x10u), 1u << 12);
    ASSERT_EQ(cpu0.interrupt & (1u << 8), 1u << 8);
    mem_write32(mem, i2s + 0x18u, 1u << 12);

    mem_write32(mem, i2s + 0x30u, 1u << 28); /* OUTLINK_STOP */
    ASSERT_TRUE(mem_read32(mem, i2s + 0x30u) & (1u << 31));
    ASSERT_EQ(periph_unhandled_count(p), 0);

    periph_destroy(p);
    mem_destroy(mem);
}

TEST(i2s_rx_dma_injection_and_dual_port) {
    const uint32_t i2s0 = 0x3FF4F000u;
    const uint32_t i2s1 = 0x3FF6D000u;
    const uint32_t desc = 0x3FFB1400u;
    const uint32_t buf = 0x3FFB2400u;
    static const uint8_t injected[8] = {
        0x81, 0x72, 0x63, 0x54, 0x45, 0x36, 0x27, 0x18,
    };
    xtensa_mem_t *mem = mem_create();
    esp32_periph_t *p = periph_create(mem);
    xtensa_cpu_t cpu0;
    xtensa_cpu_init(&cpu0);
    cpu0.mem = mem;
    periph_attach_cpus(p, &cpu0, NULL);
    periph_intr_matrix_set(p, 0, 9, 33); /* I2S1 -> CPU interrupt 9 */

    ASSERT_EQ(periph_i2s_rx_inject(p, 1, injected, sizeof(injected)),
              sizeof(injected));
    ASSERT_EQ(periph_i2s_rx_pending(p, 1), sizeof(injected));
    test_spi_dma_desc(mem, desc, buf, sizeof(injected), 0, 1, desc);
    mem_write32(mem, i2s1 + 0x24u,
                sizeof(injected) / sizeof(uint32_t)); /* RXEOF_NUM words */
    mem_write32(mem, i2s1 + 0x14u, 1u << 9); /* IN_SUC_EOF_INT_ENA */
    mem_write32(mem, i2s1 + 0x34u,
                (desc & 0xFFFFFu) | (1u << 29)); /* INLINK_START */
    mem_write32(mem, i2s1 + 0x08u,
                mem_read32(mem, i2s1 + 0x08u) | (1u << 5)); /* RX_START */

    uint8_t received[sizeof(injected)];
    for (size_t i = 0; i < sizeof(received); i++)
        received[i] = mem_read8(mem, buf + (uint32_t)i);
    ASSERT_TRUE(memcmp(received, injected, sizeof(injected)) == 0);
    ASSERT_EQ(periph_i2s_rx_pending(p, 1), 0);
    ASSERT_EQ((mem_read32(mem, desc) >> 12) & 0xFFFu, sizeof(injected));
    ASSERT_EQ(mem_read32(mem, i2s1 + 0x0Cu) & ((1u << 8) | (1u << 9)),
              (1u << 8) | (1u << 9));
    ASSERT_EQ(mem_read32(mem, i2s1 + 0x10u), 1u << 9);
    ASSERT_EQ(mem_read32(mem, i2s1 + 0x3Cu), desc);
    ASSERT_EQ(mem_read32(mem, i2s1 + 0x48u), desc);
    ASSERT_EQ(mem_read32(mem, i2s1 + 0x4Cu), desc);
    ASSERT_EQ(mem_read32(mem, i2s1 + 0x50u), buf);
    ASSERT_EQ(cpu0.interrupt & (1u << 9), 1u << 9);

    /* The two controller register files and host RX queues are independent. */
    mem_write32(mem, i2s0 + 0xACu, 0x00200000u);
    mem_write32(mem, i2s1 + 0xACu, 0x00400000u);
    ASSERT_EQ(mem_read32(mem, i2s0 + 0xACu), 0x00200000u);
    ASSERT_EQ(mem_read32(mem, i2s1 + 0xACu), 0x00400000u);
    ASSERT_EQ(periph_i2s_rx_pending(p, 0), 0);

    mem_write32(mem, i2s1 + 0x18u, 1u << 9); /* IN_SUC_EOF_INT_CLR */
    ASSERT_EQ(mem_read32(mem, i2s1 + 0x10u), 0);
    ASSERT_EQ(cpu0.interrupt & (1u << 9), 0);
    mem_write32(mem, i2s1 + 0x34u, 1u << 28); /* INLINK_STOP */
    ASSERT_TRUE(mem_read32(mem, i2s1 + 0x34u) & (1u << 31));
    ASSERT_EQ(periph_unhandled_count(p), 0);

    periph_destroy(p);
    mem_destroy(mem);
}

TEST(i2s_clock_and_bt_private_readback) {
    xtensa_mem_t *mem = mem_create();
    esp32_periph_t *p = periph_create(mem);

    mem_write32(mem, 0x3FF4F0ACu, 0x00200000u);
    mem_write32(mem, 0x3FF6D0ACu, 0x00400000u);
    ASSERT_EQ(mem_read32(mem, 0x3FF4F0ACu), 0x00200000u);
    ASSERT_EQ(mem_read32(mem, 0x3FF6D0ACu), 0x00400000u);

    mem_write32(mem, 0x3FF7120Cu, 0xA5A5A5A4u);
    ASSERT_EQ(mem_read32(mem, 0x3FF7120Cu), 0xA5A5A5A4u);
    ASSERT_EQ(periph_unhandled_count(p), 0);

    periph_destroy(p);
    mem_destroy(mem);
}

#define TEST_RMT_BASE 0x3FF56000u

static uint32_t test_rmt_item(uint16_t duration0, bool level0,
                              uint16_t duration1, bool level1) {
    return (duration0 & 0x7FFFu) | (level0 ? 1u << 15 : 0u) |
           ((uint32_t)(duration1 & 0x7FFFu) << 16) |
           (level1 ? 1u << 31 : 0u);
}

typedef struct {
    uint32_t items[32];
    size_t count;
    unsigned chunks;
    unsigned finishes;
    uint32_t tick_hz;
    uint32_t carrier_hz;
} rmt_capture_t;

static void capture_rmt(void *ctx, int channel, const uint32_t *items,
                        size_t count, uint32_t tick_hz,
                        uint32_t carrier_hz, bool finished) {
    rmt_capture_t *capture = ctx;
    if (channel != 0) return;
    capture->chunks++;
    capture->tick_hz = tick_hz;
    capture->carrier_hz = carrier_hz;
    if (finished) capture->finishes++;
    for (size_t i = 0; i < count && capture->count < 32u; i++)
        capture->items[capture->count++] = items[i];
}

TEST(rmt_register_file_shared_ram_and_apb_fifo) {
    xtensa_mem_t *mem = mem_create();
    esp32_periph_t *p = periph_create(mem);

    ASSERT_EQ(mem_read32(mem, TEST_RMT_BASE + 0x20u), 0x31100002u);
    ASSERT_EQ(mem_read32(mem, TEST_RMT_BASE + 0x24u), 0x00000F00u);
    ASSERT_EQ(mem_read32(mem, TEST_RMT_BASE + 0xFCu), 0x16022600u);

    /* Two allocated blocks make the channel-0 APB FIFO 128 words deep. */
    mem_write32(mem, TEST_RMT_BASE + 0x20u, (2u << 24) | 80u);
    uint32_t conf1 = mem_read32(mem, TEST_RMT_BASE + 0x24u);
    mem_write32(mem, TEST_RMT_BASE + 0x24u, conf1 | (1u << 4));
    mem_write32(mem, TEST_RMT_BASE + 0x24u, conf1 & ~(1u << 4));
    mem_write32(mem, TEST_RMT_BASE + 0x00u, 0x11223344u);
    mem_write32(mem, TEST_RMT_BASE + 0x00u, 0x55667788u);
    ASSERT_EQ(mem_read32(mem, TEST_RMT_BASE + 0x80u), 2u);
    ASSERT_EQ(mem_read32(mem, TEST_RMT_BASE + 0x800u), 0x11223344u);
    ASSERT_EQ(mem_read32(mem, TEST_RMT_BASE + 0x804u), 0x55667788u);

    mem_write32(mem, TEST_RMT_BASE + 0x24u, conf1 | (1u << 4));
    mem_write32(mem, TEST_RMT_BASE + 0x24u, conf1 & ~(1u << 4));
    ASSERT_EQ(mem_read32(mem, TEST_RMT_BASE + 0x00u), 0x11223344u);
    ASSERT_EQ(mem_read32(mem, TEST_RMT_BASE + 0x00u), 0x55667788u);

    /* Direct RMTMEM covers all eight hardware blocks in the same page. */
    mem_write32(mem, TEST_RMT_BASE + 0xFFCu, 0xA5A55A5Au);
    ASSERT_EQ(mem_read32(mem, TEST_RMT_BASE + 0xFFCu), 0xA5A55A5Au);
    ASSERT_EQ(periph_unhandled_count(p), 0);

    periph_destroy(p);
    mem_destroy(mem);
}

TEST(rmt_timed_tx_threshold_and_completion_interrupts) {
    xtensa_mem_t *mem = mem_create();
    esp32_periph_t *p = periph_create(mem);
    xtensa_cpu_t cpu;
    xtensa_cpu_init(&cpu);
    cpu.mem = mem;
    periph_attach_cpus(p, &cpu, NULL);

    rmt_capture_t capture = {0};
    ASSERT_EQ(periph_set_rmt_tx_callback(p, 0, capture_rmt, &capture), 0);
    uint32_t words[] = {
        test_rmt_item(1, true, 1, false),
        test_rmt_item(1, false, 1, true),
        test_rmt_item(1, true, 1, true),
        test_rmt_item(1, false, 1, false),
        0,
    };
    for (size_t i = 0; i < sizeof(words) / sizeof(words[0]); i++)
        mem_write32(mem, TEST_RMT_BASE + 0x800u + (uint32_t)i * 4u,
                    words[i]);

    /* One APB-clocked block, threshold every two items, 1 MHz carrier. */
    mem_write32(mem, TEST_RMT_BASE + 0x20u,
                (1u << 28) | (1u << 24) | 1u);
    mem_write32(mem, TEST_RMT_BASE + 0xB0u, (40u << 16) | 40u);
    mem_write32(mem, TEST_RMT_BASE + 0xD0u, 2u);
    mem_write32(mem, TEST_RMT_BASE + 0xF0u, 1u << 1);
    mem_write32(mem, TEST_RMT_BASE + 0xA8u,
                (1u << 0) | (1u << 24));
    uint32_t tx_conf = 1u << 17;
    mem_write32(mem, TEST_RMT_BASE + 0x24u, tx_conf | (1u << 3));
    mem_write32(mem, TEST_RMT_BASE + 0x24u, tx_conf);
    mem_write32(mem, TEST_RMT_BASE + 0x24u, tx_conf | 1u);

    /* Two two-tick items at APB/1 consume exactly 12 CPU cycles. */
    cpu.ccount += 12u;
    cpu.periph_event(&cpu);
    ASSERT_EQ(capture.count, 2u);
    ASSERT_EQ(capture.chunks, 1u);
    ASSERT_EQ(capture.finishes, 0u);
    ASSERT_EQ(capture.tick_hz, 80000000u);
    ASSERT_EQ(capture.carrier_hz, 1000000u);
    ASSERT_EQ(mem_read32(mem, TEST_RMT_BASE + 0xA0u), 1u << 24);
    ASSERT_TRUE(periph_interrupt_pending(p, 47));
    mem_write32(mem, TEST_RMT_BASE + 0xACu, 1u << 24);

    cpu.ccount += 12u;
    cpu.periph_event(&cpu);
    ASSERT_EQ(capture.count, 4u);
    ASSERT_EQ(capture.chunks, 2u);
    ASSERT_EQ(mem_read32(mem, TEST_RMT_BASE + 0xA0u), 1u << 24);
    mem_write32(mem, TEST_RMT_BASE + 0xACu, 1u << 24);

    /* The zero-duration terminator follows at the next hardware event. */
    cpu.ccount += 1u;
    cpu.periph_event(&cpu);
    ASSERT_EQ(capture.chunks, 3u);
    ASSERT_EQ(capture.finishes, 1u);
    ASSERT_EQ(mem_read32(mem, TEST_RMT_BASE + 0xA0u), 1u);
    ASSERT_EQ(mem_read32(mem, TEST_RMT_BASE + 0xA4u), 1u);
    ASSERT_TRUE(periph_interrupt_pending(p, 47));
    ASSERT_EQ(mem_read32(mem, TEST_RMT_BASE + 0x60u) & (7u << 24), 0u);
    ASSERT_EQ(mem_read32(mem, TEST_RMT_BASE + 0x24u) & 1u, 0u);
    mem_write32(mem, TEST_RMT_BASE + 0xACu, 1u);
    ASSERT_FALSE(periph_interrupt_pending(p, 47));
    ASSERT_EQ(periph_unhandled_count(p), 0);

    periph_destroy(p);
    mem_destroy(mem);
}

TEST(rmt_large_time_jump_drains_all_due_tx_events) {
    xtensa_mem_t *mem = mem_create();
    esp32_periph_t *p = periph_create(mem);
    xtensa_cpu_t cpu;
    xtensa_cpu_init(&cpu);
    cpu.mem = mem;
    periph_attach_cpus(p, &cpu, NULL);

    rmt_capture_t capture = {0};
    ASSERT_EQ(periph_set_rmt_tx_callback(p, 0, capture_rmt, &capture), 0);
    for (unsigned i = 0; i < 4u; i++)
        mem_write32(mem, TEST_RMT_BASE + 0x800u + i * 4u,
                    test_rmt_item(1, (i & 1u) != 0, 1,
                                  (i & 1u) == 0));
    mem_write32(mem, TEST_RMT_BASE + 0x810u, 0);

    mem_write32(mem, TEST_RMT_BASE + 0x20u, (1u << 24) | 1u);
    mem_write32(mem, TEST_RMT_BASE + 0xD0u, 2u);
    mem_write32(mem, TEST_RMT_BASE + 0x24u, (1u << 17) | 1u);

    /* Deadlines are 12, 24, and 25 CPU ticks. One overshooting delay must
     * observe both thresholds and the terminator, not stretch the waveform. */
    cpu.ccount = 25u;
    cpu.periph_event(&cpu);
    ASSERT_EQ(capture.count, 4u);
    ASSERT_EQ(capture.chunks, 3u);
    ASSERT_EQ(capture.finishes, 1u);
    ASSERT_EQ(mem_read32(mem, TEST_RMT_BASE + 0xA0u),
              (1u << 24) | 1u);
    ASSERT_EQ(mem_read32(mem, TEST_RMT_BASE + 0x24u) & 1u, 0u);
    ASSERT_EQ(periph_unhandled_count(p), 0);

    periph_destroy(p);
    mem_destroy(mem);
}

TEST(rmt_tx_deadline_tracks_runtime_cpu_frequency) {
    xtensa_mem_t *mem = mem_create();
    esp32_periph_t *p = periph_create(mem);
    xtensa_cpu_t cpu;
    xtensa_cpu_init(&cpu);
    cpu.mem = mem;
    periph_attach_cpus(p, &cpu, NULL);

    rmt_capture_t capture = {0};
    ASSERT_EQ(periph_set_rmt_tx_callback(p, 0, capture_rmt, &capture), 0);
    mem_write32(mem, 0x3FFE01E0u, 80u);
    mem_write32(mem, TEST_RMT_BASE + 0x800u,
                test_rmt_item(1, true, 1, false));
    mem_write32(mem, TEST_RMT_BASE + 0x804u, 0u);
    mem_write32(mem, TEST_RMT_BASE + 0x20u, (1u << 24) | 1u);
    mem_write32(mem, TEST_RMT_BASE + 0x24u, (1u << 17) | 1u);

    /* Two APB ticks are two CCOUNT ticks while the CPU runs at 80 MHz. */
    cpu.ccount = 1u;
    cpu.periph_event(&cpu);
    ASSERT_EQ(capture.count, 0u);
    cpu.ccount = 2u;
    cpu.periph_event(&cpu);
    ASSERT_EQ(capture.count, 1u);
    ASSERT_EQ(capture.finishes, 1u);
    ASSERT_EQ(capture.tick_hz, 80000000u);
    ASSERT_EQ(periph_unhandled_count(p), 0);

    periph_destroy(p);
    mem_destroy(mem);
}

TEST(rmt_rx_injection_uses_channel_memory_and_interrupts) {
    xtensa_mem_t *mem = mem_create();
    esp32_periph_t *p = periph_create(mem);
    const unsigned channel = 3u;
    const uint32_t conf0 = TEST_RMT_BASE + 0x20u + channel * 8u;
    const uint32_t conf1 = TEST_RMT_BASE + 0x24u + channel * 8u;
    const uint32_t mem_base = TEST_RMT_BASE + 0x800u + channel * 0x100u;
    uint32_t items[] = {
        test_rmt_item(7, true, 9, false),
        test_rmt_item(11, false, 13, true),
        test_rmt_item(17, true, 19, false),
    };

    mem_write32(mem, conf0, (1u << 24) | 80u);
    mem_write32(mem, TEST_RMT_BASE + 0xA8u,
                (1u << (channel * 3u + 1u)) |
                (1u << (channel * 3u + 2u)));
    mem_write32(mem, conf1, (1u << 5) | (1u << 1));
    ASSERT_EQ(periph_rmt_rx_inject(p, (int)channel, items, 3u), 3u);
    ASSERT_EQ(mem_read32(mem, mem_base), items[0]);
    ASSERT_EQ(mem_read32(mem, mem_base + 8u), items[2]);
    ASSERT_EQ(mem_read32(mem, TEST_RMT_BASE + 0x60u + channel * 4u) &
              0x3FFu, channel * 64u + 3u);
    ASSERT_EQ(mem_read32(mem, TEST_RMT_BASE + 0xA0u),
              1u << (channel * 3u + 1u));
    ASSERT_TRUE(periph_interrupt_pending(p, 47));
    mem_write32(mem, TEST_RMT_BASE + 0xACu, 1u << (channel * 3u + 1u));
    ASSERT_FALSE(periph_interrupt_pending(p, 47));

    /* Channel 7 cannot borrow a nonexistent ninth block. */
    uint32_t many[70] = {0};
    mem_write32(mem, TEST_RMT_BASE + 0x20u + 7u * 8u,
                (2u << 24) | 80u);
    mem_write32(mem, TEST_RMT_BASE + 0x24u + 7u * 8u,
                (1u << 5) | (1u << 1));
    ASSERT_EQ(periph_rmt_rx_inject(p, 7, many, 70u), 64u);
    ASSERT_TRUE(mem_read32(mem, TEST_RMT_BASE + 0xA0u) &
                (1u << (7u * 3u + 2u)));
    ASSERT_EQ(periph_unhandled_count(p), 0);

    periph_destroy(p);
    mem_destroy(mem);
}

TEST(rmt_dport_module_reset_clears_hardware_and_preserves_endpoint) {
    xtensa_mem_t *mem = mem_create();
    esp32_periph_t *p = periph_create(mem);
    xtensa_cpu_t cpu;
    xtensa_cpu_init(&cpu);
    cpu.mem = mem;
    periph_attach_cpus(p, &cpu, NULL);

    rmt_capture_t capture = {0};
    ASSERT_EQ(periph_set_rmt_tx_callback(p, 0, capture_rmt, &capture), 0);

    /* Dirty every class of state and assert the RMT interrupt through RX. */
    uint32_t rx_item = test_rmt_item(7, true, 9, false);
    mem_write32(mem, 0x3FF000C0u, 1u << 9);
    mem_write32(mem, TEST_RMT_BASE + 0x20u, (1u << 24) | 80u);
    mem_write32(mem, TEST_RMT_BASE + 0xA8u, 1u << 1);
    mem_write32(mem, TEST_RMT_BASE + 0xB0u, 0x12345678u);
    mem_write32(mem, TEST_RMT_BASE + 0xD0u, 31u);
    mem_write32(mem, TEST_RMT_BASE + 0xF0u, 3u);
    mem_write32(mem, TEST_RMT_BASE + 0xFFCu, 0xA5A55A5Au);
    mem_write32(mem, TEST_RMT_BASE + 0x24u, (1u << 5) | (1u << 1));
    ASSERT_EQ(periph_rmt_rx_inject(p, 0, &rx_item, 1u), 1u);
    ASSERT_TRUE(periph_interrupt_pending(p, 47));

    mem_write32(mem, 0x3FF000C4u, 1u << 9);
    ASSERT_EQ(mem_read32(mem, 0x3FF000C0u), 1u << 9);
    ASSERT_EQ(mem_read32(mem, 0x3FF000C4u), 1u << 9);
    ASSERT_EQ(mem_read32(mem, TEST_RMT_BASE + 0x20u), 0x31100002u);
    ASSERT_EQ(mem_read32(mem, TEST_RMT_BASE + 0x24u), 0x00000F00u);
    ASSERT_EQ(mem_read32(mem, TEST_RMT_BASE + 0xA0u), 0u);
    ASSERT_EQ(mem_read32(mem, TEST_RMT_BASE + 0xA8u), 0u);
    ASSERT_EQ(mem_read32(mem, TEST_RMT_BASE + 0xB0u), 0u);
    ASSERT_EQ(mem_read32(mem, TEST_RMT_BASE + 0xD0u), 0u);
    ASSERT_EQ(mem_read32(mem, TEST_RMT_BASE + 0xF0u), 0u);
    ASSERT_EQ(mem_read32(mem, TEST_RMT_BASE + 0xFFCu), 0u);
    ASSERT_EQ(mem_read32(mem, TEST_RMT_BASE + 0xFCu), 0x16022600u);
    ASSERT_FALSE(periph_interrupt_pending(p, 47));
    mem_write32(mem, 0x3FF000C4u, 0u);
    ASSERT_EQ(mem_read32(mem, 0x3FF000C4u), 0u);

    /* Host wiring is outside the peripheral reset domain and remains bound. */
    mem_write32(mem, TEST_RMT_BASE + 0x800u,
                test_rmt_item(1, true, 1, false));
    mem_write32(mem, TEST_RMT_BASE + 0x804u, 0u);
    mem_write32(mem, TEST_RMT_BASE + 0x20u, (1u << 24) | 1u);
    mem_write32(mem, TEST_RMT_BASE + 0x24u, (1u << 17) | 1u);
    cpu.ccount += 6u;
    cpu.periph_event(&cpu);
    ASSERT_EQ(capture.count, 1u);
    ASSERT_EQ(capture.finishes, 1u);
    ASSERT_EQ(periph_unhandled_count(p), 0);

    periph_destroy(p);
    mem_destroy(mem);
}

TEST(excmlevel3_masks_level3) {
    /* With EXCMLEVEL=3, level-3 interrupts should be masked when EXCM=1 */
    xtensa_cpu_t cpu;
    setup_exc(&cpu);
    cpu.ps = 0;
    XT_PS_SET_EXCM(cpu.ps, 1);  /* EXCM=1 */
    cpu.int_level[3] = 3;
    cpu.interrupt = (1u << 3);
    cpu.intenable = (1u << 3);
    cpu.irq_check = true;
    put_insn3(&cpu, BASE, INSN_NOP);
    uint32_t pc_before = cpu.pc;
    xtensa_step(&cpu);
    /* Level-3 masked by EXCMLEVEL=3: should NOT dispatch */
    ASSERT_EQ(cpu.pc, pc_before + 3);  /* NOP executed, no interrupt */
    teardown(&cpu);
}

static void run_peripheral_tests(void) {
    TEST_SUITE("peripherals");
    RUN_TEST(mmio_hook_read32);
    RUN_TEST(mmio_hook_write32);
    RUN_TEST(mmio_range_registration);
    RUN_TEST(mmio_no_handler_returns_zero);
    RUN_TEST(uart_tx_capture);
    RUN_TEST(uhci_reset_register_file_dual_instance_and_dport);
    RUN_TEST(uhci_transparent_tx_dma_wire_timing_quick_send_and_interrupt);
    RUN_TEST(uhci_transparent_rx_descriptor_chain_idle_and_break_eof);
    RUN_TEST(uhci_h5_slip_receive_transmit_and_checksum_error);
    RUN_TEST(uhci_h5_reliable_sequence_and_crc_validation);
    RUN_TEST(uhci_malformed_descriptors_report_directional_errors);
    RUN_TEST(sdio_slave_reset_shared_interrupts_and_dport);
    RUN_TEST(sdio_slave_scatter_gather_packets_and_writeback);
    RUN_TEST(sdio_slave_malformed_dma_is_packet_atomic);
    RUN_TEST(sdmmc_reset_slots_dport_and_card_status);
    RUN_TEST(sdmmc_native_image_uses_existing_capacity);
    RUN_TEST(sdmmc_commands_responses_and_pio_fifo);
    RUN_TEST(sdmmc_idmac_chains_writeback_interrupts_and_errors);
    RUN_TEST(sdmmc_slot1_clock_gate_idmac_write_and_media_errors);
    RUN_TEST(twai_reset_acceptance_fifo_overrun_interrupt_and_dport);
    RUN_TEST(twai_wire_timing_self_reception_retry_busoff_and_recovery);
    RUN_TEST(emac_reset_clock_extension_and_clause22_mdio);
    RUN_TEST(emac_enhanced_chained_tx_completion_error_and_interrupt);
    RUN_TEST(emac_rx_filter_fcs_descriptor_chain_unavailable_and_receive_all);
    RUN_TEST(uart_controllers_are_independent);
    RUN_TEST(uart_status_tx_ready);
    RUN_TEST(uart_tx_empty_interrupt);
    RUN_TEST(uart_tx_done_interrupt);
    RUN_TEST(uart_rx_fifo_injection);
    RUN_TEST(uart_rx_timeout_interrupt);
    RUN_TEST(uart2_rx_fifo_and_interrupt);
    RUN_TEST(i2c_master_repeated_start_read_and_interrupt);
    RUN_TEST(i2c_end_command_streams_fifo_chunks_until_stop);
    RUN_TEST(i2c_dual_port_ahb_alias_and_address_nack);
    RUN_TEST(rtc_i2c_register_masks_and_command_file);
    RUN_TEST(rtc_i2c_sens_master_read_write_nack_and_timeout);
    RUN_TEST(irq_dispatch_observes_only_rising_edges);
    RUN_TEST(spi_flash_write_enable_latch);
    RUN_TEST(spi_flash_program_erase_require_write_enable);
    RUN_TEST(flash_mmu_maps_complete_pages_and_all_instruction_buses);
    RUN_TEST(flash_mmu_exposes_full_multicore_copy_windows);
    RUN_TEST(spi_flash_dual_io_mode_bits_are_not_address_bits);
    RUN_TEST(wifi_mac_init_ready_handshake);
    RUN_TEST(radio_phy_calibration_register_files);
    RUN_TEST(xpt2046_pipelined_conversions);
    RUN_TEST(gp_spi_matrix_routing_and_hardware_cs);
    RUN_TEST(gp_spi_native_iomux_routing_and_hardware_cs);
    RUN_TEST(gp_spi_dma_descriptor_chain_and_interrupt);
    RUN_TEST(gp_spi_dma_full_duplex_sd);
    RUN_TEST(raw_sd_multiblock_read_write);
    RUN_TEST(dport_safe_defaults);
    RUN_TEST(post_boot_cpu_clock_state_and_writes);
    RUN_TEST(frc1_countdown_reload_prescalers_and_interrupt);
    RUN_TEST(frc2_countup_wrap_compare_and_runtime_frequency);
    RUN_TEST(timg_general_timers_count_capture_pause_and_reset);
    RUN_TEST(timg_alarm_autoreload_mask_clear_and_sources);
    RUN_TEST(timg_dual_core_time_uses_monotonic_maximum);
    RUN_TEST(wdt_disable);
    RUN_TEST(rtc_reset_cause);
    RUN_TEST(sens_adc_single_conversions);
    RUN_TEST(rtcio_dac_state_and_events);
    RUN_TEST(sigmadelta_registers_matrix_and_aggregate_output);
    RUN_TEST(sigmadelta_timed_density_prescaler_and_inversion);
    RUN_TEST(gpio_set_clear);
    RUN_TEST(gpio_high_pin_falling_edge_interrupt);
    RUN_TEST(gpio_interrupt_routes_to_selected_core);
    RUN_TEST(gpio_level_interrupt_reasserts_until_inactive);
    RUN_TEST(efuse_chip_info);
    RUN_TEST(insn_reads_periph);
    RUN_TEST(intr_matrix_default_disabled);
    RUN_TEST(intr_matrix_set_and_read);
    RUN_TEST(intr_matrix_dport_rw);
    RUN_TEST(intr_matrix_fan_in_remap_and_disable);
    RUN_TEST(intr_matrix_internal_cpu_lines_are_isolated);
    RUN_TEST(intr_matrix_dual_core_pending_route);
    RUN_TEST(intr_matrix_assert_source);
    RUN_TEST(cross_core_interrupt);
    RUN_TEST(pcnt_matrix_control_limits_events_and_reset);
    RUN_TEST(pcnt_filter_rejects_glitches_and_qualifies_edges);
    RUN_TEST(mcpwm_timed_waveform_shadow_force_and_interrupt);
    RUN_TEST(mcpwm_units_and_upper_operators_are_independent);
    RUN_TEST(mcpwm_dual_core_clock_and_deferred_stop_boundary);
    RUN_TEST(mcpwm_gpio_capture_sync_fault_deadtime_and_carrier);
    RUN_TEST(ledc_timed_duty_update_and_gpio_output);
    RUN_TEST(ledc_fade_progress_and_completion_deadline);
    RUN_TEST(ledc_timer_overflow_pause_and_clear);
    RUN_TEST(ledc_low_speed_output_and_dport_reset_preserve_endpoint);
    RUN_TEST(i2s_tx_dma_descriptor_and_interrupt);
    RUN_TEST(i2s_rx_dma_injection_and_dual_port);
    RUN_TEST(i2s_clock_and_bt_private_readback);
    RUN_TEST(rmt_register_file_shared_ram_and_apb_fifo);
    RUN_TEST(rmt_timed_tx_threshold_and_completion_interrupts);
    RUN_TEST(rmt_large_time_jump_drains_all_due_tx_events);
    RUN_TEST(rmt_tx_deadline_tracks_runtime_cpu_frequency);
    RUN_TEST(rmt_rx_injection_uses_channel_memory_and_interrupts);
    RUN_TEST(rmt_dport_module_reset_clears_hardware_and_preserves_endpoint);
    RUN_TEST(excmlevel3_masks_level3);
}
