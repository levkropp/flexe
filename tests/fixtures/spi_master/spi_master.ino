/* Drive the real ESP-IDF spi_master driver against a host-side slave.
 *
 * The CYD display libraries (TFT_eSPI and friends) write the GP-SPI registers
 * directly, so a stock ROM never exercises the IDF driver at all -- not its
 * queued transactions, not its DMA descriptors, not the command/address
 * phases. This fixture uses the public driver so those paths are covered.
 *
 * The chip select is a pin the emulator models no device on, so the host
 * harness stands in as the slave: it sees what we transmit and chooses what
 * we receive.
 */
#include <Arduino.h>
#include <driver/spi_master.h>
#include <driver/gpio.h>

#define PIN_MISO  19
#define PIN_MOSI  23
#define PIN_SCLK  18
#define PIN_CS    27      /* modelled as nothing: the harness answers here */

#define SUCCESS_MARKER 0x5D100D1Eu
#define FAIL_BASE      0xBAD00000u

volatile uint32_t flexe_spi_stage = 0;
volatile uint32_t flexe_spi_result[12];

static spi_device_handle_t dev;

static void fail(uint32_t code) { flexe_spi_stage = FAIL_BASE | code; }

/* The harness replies with a byte derived from the transfer's own length and
 * the byte index, so a short read, a duplicated byte or a shifted buffer all
 * show up rather than looking plausible. */
static uint8_t expect_miso(size_t len, size_t i) {
  return (uint8_t)((len * 0x11u) ^ (i * 0x2Fu) ^ 0x5Au);
}

static bool full_duplex(size_t len, uint32_t slot) {
  uint8_t tx[64], rx[64];
  for (size_t i = 0; i < len; i++) tx[i] = (uint8_t)(0xA0u + i);
  memset(rx, 0, sizeof(rx));

  spi_transaction_t t = {};
  t.length = len * 8;
  t.rxlength = len * 8;
  t.tx_buffer = tx;
  t.rx_buffer = rx;
  if (spi_device_transmit(dev, &t) != ESP_OK) return false;

  for (size_t i = 0; i < len; i++)
    if (rx[i] != expect_miso(len, i)) {
      flexe_spi_result[slot] = (uint32_t)(i << 16) | (rx[i] << 8) |
                               expect_miso(len, i);
      return false;
    }
  flexe_spi_result[slot] = (uint32_t)len;
  return true;
}

void setup() {
  spi_bus_config_t bus = {};
  bus.mosi_io_num = PIN_MOSI;
  bus.miso_io_num = PIN_MISO;
  bus.sclk_io_num = PIN_SCLK;
  bus.quadwp_io_num = -1;
  bus.quadhd_io_num = -1;
  bus.max_transfer_sz = 4096;
  if (spi_bus_initialize(VSPI_HOST, &bus, 1) != ESP_OK) { fail(1); return; }

  spi_device_interface_config_t devcfg = {};
  devcfg.clock_speed_hz = 1000000;
  devcfg.mode = 0;
  devcfg.spics_io_num = PIN_CS;
  devcfg.queue_size = 4;
  devcfg.command_bits = 8;
  devcfg.address_bits = 8;
  if (spi_bus_add_device(VSPI_HOST, &devcfg, &dev) != ESP_OK) { fail(2); return; }

  flexe_spi_stage = 1;

  /* Lengths that straddle the 64-byte register file: a 1-byte transfer, one
   * that fills W0 exactly, one that spans several W registers, and one long
   * enough to force the DMA path. */
  static const size_t lens[] = {1, 4, 5, 17, 33};
  for (unsigned k = 0; k < sizeof(lens) / sizeof(lens[0]); k++) {
    if (!full_duplex(lens[k], k)) { fail(0x10u + k); return; }
  }
  flexe_spi_stage = 2;

  /* Command and address phases, which only the IDF driver emits. */
  uint8_t rx[8] = {0};
  spi_transaction_t t = {};
  t.cmd = 0x9F;
  t.addr = 0x42;
  t.length = 4 * 8;
  t.rxlength = 4 * 8;
  t.rx_buffer = rx;
  if (spi_device_transmit(dev, &t) != ESP_OK) { fail(0x20); return; }
  flexe_spi_result[8] = ((uint32_t)rx[0] << 24) | ((uint32_t)rx[1] << 16) |
                        ((uint32_t)rx[2] << 8) | rx[3];

  /* A queued transaction, exercising the driver's asynchronous path. */
  uint8_t qtx[8] = {0xDE, 0xAD, 0xBE, 0xEF};
  uint8_t qrx[8] = {0};
  spi_transaction_t q = {};
  q.length = 4 * 8;
  q.rxlength = 4 * 8;
  q.tx_buffer = qtx;
  q.rx_buffer = qrx;
  spi_transaction_t *done = nullptr;
  if (spi_device_queue_trans(dev, &q, portMAX_DELAY) != ESP_OK) { fail(0x30); return; }
  if (spi_device_get_trans_result(dev, &done, portMAX_DELAY) != ESP_OK) { fail(0x31); return; }
  if (done != &q) { fail(0x32); return; }
  for (size_t i = 0; i < 4; i++)
    if (qrx[i] != expect_miso(4, i)) { fail(0x33); return; }
  flexe_spi_result[9] = 1;

  flexe_spi_stage = SUCCESS_MARKER;
}

void loop() { delay(10); }
