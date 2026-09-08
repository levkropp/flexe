#include "sx127x.h"

#include <string.h>

static uint8_t sx127x_test_read(sx127x_t *radio, uint8_t address) {
    uint8_t wire[2] = {address, 0xFFu};
    sx127x_spi_transfer(radio, 3, wire, sizeof(wire), wire, sizeof(wire));
    return wire[1];
}

static void sx127x_test_write(sx127x_t *radio, uint8_t address,
                              uint8_t value) {
    uint8_t wire[2] = {(uint8_t)(address | 0x80u), value};
    sx127x_spi_transfer(radio, 3, wire, sizeof(wire), wire, sizeof(wire));
}

static int sx127x_test_dio0(xtensa_mem_t *mem) {
    return (mem_read32(mem, 0x3FF4403Cu) & (1u << 26)) != 0;
}

typedef struct {
    unsigned packets;
    uint8_t data[256];
    size_t len;
} sx127x_test_tx_t;

static void sx127x_test_capture_tx(void *ctx, const uint8_t *data, size_t len) {
    sx127x_test_tx_t *capture = ctx;
    capture->packets++;
    capture->len = len;
    memcpy(capture->data, data, len);
}

TEST(sx127x_registers_fifo_tx_and_dio0) {
    xtensa_mem_t *mem = mem_create();
    esp32_periph_t *periph = periph_create(mem);
    sx127x_config_t config = {.periph = periph, .dio0_pin = 26};
    sx127x_t *radio = sx127x_create(&config);
    ASSERT_TRUE(radio != NULL);

    ASSERT_EQ(sx127x_test_read(radio, 0x42u), 0x12u);
    sx127x_test_write(radio, 0x42u, 0x55u);
    ASSERT_EQ(sx127x_test_read(radio, 0x42u), 0x12u);
    sx127x_test_write(radio, 0x39u, 0x2Bu);
    ASSERT_EQ(sx127x_test_read(radio, 0x39u), 0x2Bu);

    sx127x_test_tx_t tx = {0};
    sx127x_set_tx_callback(radio, sx127x_test_capture_tx, &tx);
    sx127x_test_write(radio, 0x40u, 0x40u); /* DIO0 = TxDone */
    sx127x_test_write(radio, 0x0Eu, 0x20u);
    sx127x_test_write(radio, 0x0Du, 0x20u);
    sx127x_test_write(radio, 0x22u, 3u);
    uint8_t fifo[] = {0x80u, 0xA1u, 0xB2u, 0xC3u};
    sx127x_spi_transfer(radio, 3, fifo, sizeof(fifo), fifo, sizeof(fifo));
    sx127x_test_write(radio, 0x01u, 0x83u); /* LoRa TX */

    ASSERT_EQ(tx.packets, 1u);
    ASSERT_EQ(tx.len, 3u);
    static const uint8_t expected[] = {0xA1u, 0xB2u, 0xC3u};
    ASSERT_TRUE(memcmp(tx.data, expected, sizeof(expected)) == 0);
    ASSERT_EQ(sx127x_register(radio, 0x12u) & 0x08u, 0x08u);
    ASSERT_EQ(sx127x_test_dio0(mem), 1);

    sx127x_test_write(radio, 0x12u, 0x08u); /* IRQ flags are W1C */
    ASSERT_EQ(sx127x_register(radio, 0x12u) & 0x08u, 0u);
    ASSERT_EQ(sx127x_test_dio0(mem), 0);

    sx127x_stats_t stats;
    sx127x_get_stats(radio, &stats);
    ASSERT_EQ64(stats.tx_packets, 1u);
    ASSERT_EQ(stats.last_tx_len, 3u);
    ASSERT_TRUE(stats.spi_transfers >= 10u);

    sx127x_destroy(radio);
    periph_destroy(periph);
    mem_destroy(mem);
}

TEST(sx127x_packet_injection_populates_rx_fifo) {
    xtensa_mem_t *mem = mem_create();
    esp32_periph_t *periph = periph_create(mem);
    sx127x_config_t config = {.periph = periph, .dio0_pin = 26};
    sx127x_t *radio = sx127x_create(&config);
    ASSERT_TRUE(radio != NULL);

    static const uint8_t packet[] = {0x94u, 0x25u, 0x10u, 0x7Eu};
    ASSERT_EQ(sx127x_inject_packet(radio, packet, sizeof(packet),
                                   -71, 9), -1);
    sx127x_test_write(radio, 0x0Fu, 0x40u);
    sx127x_test_write(radio, 0x40u, 0x00u); /* DIO0 = RxDone */
    sx127x_test_write(radio, 0x01u, 0x85u); /* LoRa continuous RX */
    ASSERT_EQ(sx127x_inject_packet(radio, packet, sizeof(packet),
                                   -71, 9), 0);

    ASSERT_EQ(sx127x_register(radio, 0x10u), 0x40u);
    ASSERT_EQ(sx127x_register(radio, 0x13u), sizeof(packet));
    ASSERT_EQ(sx127x_register(radio, 0x19u), 9u);
    ASSERT_EQ(sx127x_register(radio, 0x1Au), 86u);
    ASSERT_EQ(sx127x_register(radio, 0x1Cu) & 0x40u, 0x40u);
    ASSERT_EQ(sx127x_register(radio, 0x12u) & 0x50u, 0x50u);
    ASSERT_EQ(sx127x_test_dio0(mem), 1);

    sx127x_test_write(radio, 0x0Du, sx127x_register(radio, 0x10u));
    uint8_t wire[sizeof(packet) + 1u] = {0};
    sx127x_spi_transfer(radio, 3, wire, sizeof(wire), wire, sizeof(wire));
    ASSERT_TRUE(memcmp(wire + 1u, packet, sizeof(packet)) == 0);

    sx127x_test_write(radio, 0x12u, 0x50u);
    ASSERT_EQ(sx127x_test_dio0(mem), 0);
    sx127x_stats_t stats;
    sx127x_get_stats(radio, &stats);
    ASSERT_EQ64(stats.rx_packets, 1u);

    sx127x_destroy(radio);
    periph_destroy(periph);
    mem_destroy(mem);
}

TEST(sx127x_preserves_command_across_byte_transfers) {
    xtensa_mem_t *mem = mem_create();
    esp32_periph_t *periph = periph_create(mem);
    sx127x_config_t config = {.periph = periph, .dio0_pin = 26};
    sx127x_t *radio = sx127x_create(&config);
    ASSERT_TRUE(radio != NULL);

    sx127x_spi_select(radio, 1);
    uint8_t byte = 0x42u;
    sx127x_spi_transfer(radio, 3, &byte, 1, &byte, 1);
    byte = 0;
    sx127x_spi_transfer(radio, 3, &byte, 1, &byte, 1);
    sx127x_spi_select(radio, 0);
    ASSERT_EQ(byte, 0x12u);

    sx127x_spi_select(radio, 1);
    byte = 0x81u;
    sx127x_spi_transfer(radio, 3, &byte, 1, &byte, 1);
    byte = 0x85u;
    sx127x_spi_transfer(radio, 3, &byte, 1, &byte, 1);
    sx127x_spi_select(radio, 0);
    ASSERT_EQ(sx127x_register(radio, 0x01u), 0x85u);

    sx127x_stats_t stats;
    sx127x_get_stats(radio, &stats);
    ASSERT_EQ64(stats.spi_frames, 2u);
    ASSERT_EQ64(stats.version_reads, 1u);
    ASSERT_EQ64(stats.register_writes, 1u);

    sx127x_destroy(radio);
    periph_destroy(periph);
    mem_destroy(mem);
}

static void run_sx127x_tests(void) {
    TEST_SUITE("SX127x radio");
    RUN_TEST(sx127x_registers_fifo_tx_and_dio0);
    RUN_TEST(sx127x_packet_injection_populates_rx_fifo);
    RUN_TEST(sx127x_preserves_command_across_byte_transfers);
}
