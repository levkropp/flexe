#include <Arduino.h>
#include <driver/rmt.h>

volatile uint32_t flexe_rmt_stage = 0;
volatile uint32_t flexe_rmt_result[5] = {};
volatile bool flexe_rmt_tx_done = false;

static constexpr rmt_channel_t kChannel = RMT_CHANNEL_0;
static constexpr size_t kItemCount = 96;
static rmt_item32_t items[kItemCount];

static void IRAM_ATTR tx_done(rmt_channel_t channel, void *) {
  if (channel == kChannel) {
    flexe_rmt_tx_done = true;
  }
}

static void fail(uint32_t stage, uint32_t detail) {
  flexe_rmt_result[4] = detail;
  flexe_rmt_stage = 0xBAD00000u | stage;
}

void setup() {
  flexe_rmt_stage = 1;
  rmt_config_t config = RMT_DEFAULT_CONFIG_TX(GPIO_NUM_27, kChannel);
  config.clk_div = 80;  // 80 MHz APB / 80 = one microsecond per duration tick.
  config.mem_block_num = 1;
  config.tx_config.carrier_en = false;
  config.tx_config.loop_en = false;
  config.tx_config.idle_output_en = true;
  config.tx_config.idle_level = RMT_IDLE_LEVEL_LOW;

  esp_err_t err = rmt_config(&config);
  flexe_rmt_result[0] = (uint32_t)err;
  if (err != ESP_OK) {
    fail(1, (uint32_t)err);
    return;
  }

  err = rmt_driver_install(kChannel, 0, 0);
  flexe_rmt_result[1] = (uint32_t)err;
  if (err != ESP_OK) {
    fail(2, (uint32_t)err);
    return;
  }
  rmt_register_tx_end_callback(tx_done, nullptr);

  for (size_t i = 0; i < kItemCount; ++i) {
    items[i].duration0 = 5u + (i % 7u);
    items[i].level0 = (i & 1u) != 0;
    items[i].duration1 = 9u + (i % 5u);
    items[i].level1 = (i & 1u) == 0;
  }

  /* This is larger than the channel's 64-word RAM, forcing the genuine
     ESP-IDF ISR to refill alternating 32-word halves at threshold events. */
  flexe_rmt_stage = 2;
  err = rmt_write_items(kChannel, items, kItemCount, false);
  flexe_rmt_result[2] = (uint32_t)err;
  if (err != ESP_OK) {
    fail(3, (uint32_t)err);
    return;
  }

  flexe_rmt_stage = 3;
  for (unsigned waited = 0; waited < 2000 && !flexe_rmt_tx_done; ++waited) {
    delay(1);
  }
  flexe_rmt_result[3] = flexe_rmt_tx_done ? 1u : 0u;
  if (!flexe_rmt_tx_done) {
    fail(4, 0x54494D45u);  // "TIME"
    return;
  }

  err = rmt_driver_uninstall(kChannel);
  flexe_rmt_result[4] = (uint32_t)err;
  if (err != ESP_OK) {
    fail(5, (uint32_t)err);
    return;
  }
  flexe_rmt_stage = 0x524D54A0u;
}

void loop() {
  delay(1000);
}
