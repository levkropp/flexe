#include "peripherals.h"

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

static void run_peripheral_tests(void) {
    TEST_SUITE("peripherals");
    RUN_TEST(mmio_hook_read32);
    RUN_TEST(mmio_hook_write32);
    RUN_TEST(mmio_range_registration);
    RUN_TEST(mmio_no_handler_returns_zero);
    RUN_TEST(uart_tx_capture);
    RUN_TEST(uart_status_tx_ready);
    RUN_TEST(dport_safe_defaults);
    RUN_TEST(wdt_disable);
    RUN_TEST(rtc_reset_cause);
    RUN_TEST(gpio_set_clear);
    RUN_TEST(efuse_chip_info);
    RUN_TEST(insn_reads_periph);
}
