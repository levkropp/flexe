#include <Arduino.h>
#include <driver/i2s.h>

volatile uint32_t flexe_i2s_stage = 0;
volatile uint32_t flexe_i2s_result[4] = {};

static constexpr size_t kDmaBytes = 32u * 2u * sizeof(int16_t);

static void fail(uint32_t stage, uint32_t detail) {
  flexe_i2s_result[3] = detail;
  flexe_i2s_stage = 0xBAD00000u | stage;
}

void setup() {
  flexe_i2s_stage = 1;
  i2s_config_t config = {};
  config.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX | I2S_MODE_RX);
  config.sample_rate = 16000;
  config.bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT;
  config.channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT;
  config.communication_format = I2S_COMM_FORMAT_STAND_I2S;
  config.intr_alloc_flags = 0;
  config.dma_buf_count = 4;
  config.dma_buf_len = 32;
  config.use_apll = false;
  config.tx_desc_auto_clear = false;
  config.fixed_mclk = 0;
  config.mclk_multiple = I2S_MCLK_MULTIPLE_DEFAULT;
  config.bits_per_chan = I2S_BITS_PER_CHAN_DEFAULT;

  esp_err_t err = i2s_driver_install(I2S_NUM_0, &config, 0, nullptr);
  flexe_i2s_result[0] = (uint32_t)err;
  if (err != ESP_OK) {
    fail(1, (uint32_t)err);
    return;
  }

  const i2s_pin_config_t pins = {
    .mck_io_num = I2S_PIN_NO_CHANGE,
    .bck_io_num = 26,
    .ws_io_num = 25,
    .data_out_num = 22,
    .data_in_num = 21,
  };
  err = i2s_set_pin(I2S_NUM_0, &pins);
  if (err != ESP_OK) {
    fail(2, (uint32_t)err);
    return;
  }

  flexe_i2s_stage = 2;
  uint8_t tx[kDmaBytes];
  for (size_t i = 0; i < sizeof(tx); ++i)
    tx[i] = (uint8_t)(0x5Au ^ i);
  size_t bytes_written = 0;
  err = i2s_write(I2S_NUM_0, tx, sizeof(tx), &bytes_written, portMAX_DELAY);
  flexe_i2s_result[1] = bytes_written;
  if (err != ESP_OK || bytes_written != sizeof(tx)) {
    fail(3, ((uint32_t)err << 16) | (uint32_t)bytes_written);
    return;
  }

  flexe_i2s_stage = 3;
  uint8_t rx[kDmaBytes] = {};
  size_t bytes_read = 0;
  err = i2s_read(I2S_NUM_0, rx, sizeof(rx), &bytes_read, portMAX_DELAY);
  flexe_i2s_result[2] = bytes_read;
  if (err != ESP_OK || bytes_read != sizeof(rx)) {
    fail(4, ((uint32_t)err << 16) | (uint32_t)bytes_read);
    return;
  }
  for (size_t i = 0; i < sizeof(rx); ++i) {
    uint8_t expected = (uint8_t)(0xA0u ^ i);
    if (rx[i] != expected) {
      fail(5, ((uint32_t)i << 16) | rx[i]);
      return;
    }
  }

  // A full TX buffer has been returned to the real ESP-IDF driver. Restarting
  // the stock peripheral path makes that descriptor immediately observable at
  // the emulator's host-audio boundary without relying on fixture-only MMIO.
  flexe_i2s_stage = 4;
  err = i2s_stop(I2S_NUM_0);
  if (err != ESP_OK) {
    fail(6, (uint32_t)err);
    return;
  }
  err = i2s_start(I2S_NUM_0);
  if (err != ESP_OK) {
    fail(7, (uint32_t)err);
    return;
  }
  err = i2s_driver_uninstall(I2S_NUM_0);
  if (err != ESP_OK) {
    fail(8, (uint32_t)err);
    return;
  }

  flexe_i2s_result[3] = 0;
  flexe_i2s_stage = 0x12D5DAA0u;
}

void loop() {
  delay(1000);
}
