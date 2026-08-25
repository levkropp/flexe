#include "peripherals.h"
#include "spi_display.h"

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
    const uint32_t off = 0x9000;

    mem->flash_data[off] = 0xF0;
    mem_write32(mem, TEST_SPI_ADDR_REG, off);
    mem_write32(mem, TEST_SPI_MOSI_DLEN, 7);          /* one byte */
    mem_write32(mem, TEST_SPI_W0_REG, 0xAA);

    /* A page-program command without WEL is ignored. */
    mem_write32(mem, TEST_SPI_CMD_REG, 1u << 25);     /* FLASH_PP */
    ASSERT_EQ(mem->flash_data[off], 0xF0);

    mem_write32(mem, TEST_SPI_CMD_REG, 1u << 30);     /* FLASH_WREN */
    mem_write32(mem, TEST_SPI_CMD_REG, 1u << 25);     /* FLASH_PP */
    ASSERT_EQ(mem->flash_data[off], 0xA0);            /* NOR: 0xF0 & 0xAA */
    ASSERT_EQ(test_flash_status(mem) & (1u << 1), 0u); /* PP clears WEL */

    /* Erase has the same WEL requirement and restores the sector to 0xFF. */
    mem_write32(mem, TEST_SPI_CMD_REG, 1u << 24);     /* FLASH_SE, no WEL */
    ASSERT_EQ(mem->flash_data[off], 0xA0);
    mem_write32(mem, TEST_SPI_CMD_REG, 1u << 30);     /* FLASH_WREN */
    mem_write32(mem, TEST_SPI_CMD_REG, 1u << 24);     /* FLASH_SE */
    ASSERT_EQ(mem->flash_data[off], 0xFF);
    ASSERT_EQ(test_flash_status(mem) & (1u << 1), 0u); /* erase clears WEL */

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

    /* CS must first be observed inactive, then asserted. */
    mem_write32(mem, gpio + 0x14, 1u << 1);           /* GPIO33 high */
    mem_write32(mem, spi + 0x28, 7);
    mem_write32(mem, spi + 0x2C, 7);
    mem_write32(mem, spi + 0x80, 0);
    mem_write32(mem, spi + 0x00, 1u << 18);
    mem_write32(mem, gpio + 0x18, 1u << 1);           /* GPIO33 low */

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
    /* Write source 30 to PRO_CPU int 3 via DPORT register (0x3FF00104 + 3*4) */
    mem_write32(mem, 0x3FF00104 + 3 * 4, 30);
    ASSERT_EQ(mem_read32(mem, 0x3FF00104 + 3 * 4), 30);
    ASSERT_EQ(periph_intr_matrix_get(p, 0, 3), 30);
    /* Write source 25 to APP_CPU int 7 via DPORT register (0x3FF00204 + 7*4) */
    mem_write32(mem, 0x3FF00204 + 7 * 4, 25);
    ASSERT_EQ(mem_read32(mem, 0x3FF00204 + 7 * 4), 25);
    ASSERT_EQ(periph_intr_matrix_get(p, 1, 7), 25);
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
}

static void run_peripheral_tests(void) {
    TEST_SUITE("peripherals");
    RUN_TEST(mmio_hook_read32);
    RUN_TEST(mmio_hook_write32);
    RUN_TEST(mmio_range_registration);
    RUN_TEST(mmio_no_handler_returns_zero);
    RUN_TEST(uart_tx_capture);
    RUN_TEST(uart_status_tx_ready);
    RUN_TEST(uart_tx_empty_interrupt);
    RUN_TEST(uart_tx_done_interrupt);
    RUN_TEST(spi_flash_write_enable_latch);
    RUN_TEST(spi_flash_program_erase_require_write_enable);
    RUN_TEST(spi_flash_dual_io_mode_bits_are_not_address_bits);
    RUN_TEST(wifi_mac_init_ready_handshake);
    RUN_TEST(xpt2046_pipelined_conversions);
    RUN_TEST(dport_safe_defaults);
    RUN_TEST(wdt_disable);
    RUN_TEST(rtc_reset_cause);
    RUN_TEST(gpio_set_clear);
    RUN_TEST(gpio_high_pin_falling_edge_interrupt);
    RUN_TEST(gpio_interrupt_routes_to_selected_core);
    RUN_TEST(gpio_level_interrupt_reasserts_until_inactive);
    RUN_TEST(efuse_chip_info);
    RUN_TEST(insn_reads_periph);
    RUN_TEST(intr_matrix_default_disabled);
    RUN_TEST(intr_matrix_set_and_read);
    RUN_TEST(intr_matrix_dport_rw);
    RUN_TEST(intr_matrix_assert_source);
    RUN_TEST(cross_core_interrupt);
    RUN_TEST(excmlevel3_masks_level3);
}
