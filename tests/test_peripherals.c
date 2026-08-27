#include "peripherals.h"
#include "sandbox_events.h"
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
    periph_intr_matrix_set(p, 0, 6, 36); /* UART2 -> CPU interrupt 6 */

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
    ASSERT_EQ(cpu0.interrupt & (1u << 6), 1u << 6);
    ASSERT_EQ(mem_read32(mem, base), '$');
    ASSERT_EQ(periph_uart_rx_pending_num(p, 2), sizeof(nmea) - 2);

    while (periph_uart_rx_pending_num(p, 2) != 0)
        (void)mem_read32(mem, base);
    mem_write32(mem, base + 0x10, (1u << 0) | (1u << 8));
    ASSERT_EQ(mem_read32(mem, base + 0x08), 0u);
    ASSERT_EQ(cpu0.interrupt & (1u << 6), 0u);

    periph_destroy(p);
    mem_destroy(mem);
}

/* Classic ESP32 I2C master command encoding: byte count [7:0], ACK check
 * enable [8], expected ACK [9], master ACK value [10], opcode [13:11]. */
#define TEST_I2C0_BASE 0x3FF53000u
#define TEST_I2C1_BASE 0x3FF67000u

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
    periph_intr_matrix_set(p, 0, 7, 49); /* I2C0 -> CPU interrupt 7 */

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
    ASSERT_EQ(cpu0.interrupt & (1u << 7), 1u << 7);
    for (size_t index = 0; index < sizeof(commands) / sizeof(commands[0]);
         index++)
        ASSERT_TRUE(mem_read32(mem, TEST_I2C0_BASE + 0x58u +
                               (uint32_t)index * 4u) & (1u << 31));

    mem_write32(mem, TEST_I2C0_BASE + 0x24u, 1u << 7);
    ASSERT_EQ(cpu0.interrupt & (1u << 7), 0u);
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
    periph_intr_matrix_set(p, 0, 7, 30); /* SPI2 -> CPU interrupt 7 */

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
    ASSERT_EQ(cpu0.interrupt & (1u << 7), 1u << 7);

    mem_write32(mem, spi2 + 0x38, 1u << 9); /* clear TRANS_DONE */
    mem_write32(mem, spi2 + 0x11C, 0x1C0u); /* clear DMA completion */
    ASSERT_EQ(cpu0.interrupt & (1u << 7), 0u);

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

TEST(ledc_register_and_duty_completion) {
    xtensa_mem_t *mem = mem_create();
    esp32_periph_t *p = periph_create(mem);
    xtensa_cpu_t cpu0;
    xtensa_cpu_init(&cpu0); cpu0.mem = mem;
    periph_attach_cpus(p, &cpu0, NULL);
    periph_intr_matrix_set(p, 0, 4, 43); /* LEDC -> CPU interrupt 4 */

    const uint32_t base = 0x3FF59000u;
    ASSERT_EQ(mem_read32(mem, base + 0x140), 1u << 24); /* timer reset */
    mem_write32(mem, base + 0x140, 0x0207D00Au);
    ASSERT_EQ(mem_read32(mem, base + 0x140), 0x0207D00Au);

    mem_write32(mem, base + 0x008, 0x00123400u); /* HS channel 0 duty */
    ASSERT_EQ(mem_read32(mem, base + 0x010), 0x00123400u);
    mem_write32(mem, base + 0x188, 1u << 8);     /* duty-end interrupt enable */
    mem_write32(mem, base + 0x00C, 1u << 31);    /* start update */
    ASSERT_EQ(mem_read32(mem, base + 0x00C) & (1u << 31), 0u);
    ASSERT_EQ(mem_read32(mem, base + 0x180) & (1u << 8), 1u << 8);
    ASSERT_EQ(mem_read32(mem, base + 0x184) & (1u << 8), 1u << 8);
    ASSERT_EQ(cpu0.interrupt & (1u << 4), 1u << 4);

    mem_write32(mem, base + 0x18C, 1u << 8);
    ASSERT_EQ(mem_read32(mem, base + 0x180) & (1u << 8), 0u);
    ASSERT_EQ(cpu0.interrupt & (1u << 4), 0u);
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
    RUN_TEST(irq_dispatch_observes_only_rising_edges);
    RUN_TEST(spi_flash_write_enable_latch);
    RUN_TEST(spi_flash_program_erase_require_write_enable);
    RUN_TEST(flash_mmu_maps_complete_pages_and_all_instruction_buses);
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
    RUN_TEST(wdt_disable);
    RUN_TEST(rtc_reset_cause);
    RUN_TEST(sens_adc_single_conversions);
    RUN_TEST(rtcio_dac_state_and_events);
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
    RUN_TEST(ledc_register_and_duty_completion);
    RUN_TEST(i2s_tx_dma_descriptor_and_interrupt);
    RUN_TEST(i2s_rx_dma_injection_and_dual_port);
    RUN_TEST(i2s_clock_and_bt_private_readback);
    RUN_TEST(excmlevel3_masks_level3);
}
