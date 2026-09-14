#include <Arduino.h>
#include "esp32-hal-rmt.h"

// These markers let the host test inject a frame only after the production
// Arduino/ESP-IDF RX driver has armed its channel.
extern "C" volatile uint32_t flexe_rmt_rx_stage = 0;
extern "C" volatile uint32_t flexe_rmt_rx_result[100] = {};

static constexpr int kRxPin = 4;
static rmt_data_t received[128] = {};
static size_t received_count = RMT_SYMBOLS_OF(received);

void setup() {
  Serial.begin(115200);
  if (!rmtInit(kRxPin, RMT_RX_MODE, RMT_MEM_NUM_BLOCKS_1, 1000000)) {
    flexe_rmt_rx_stage = 0xBAD00001;
    return;
  }
  if (!rmtSetRxMaxThreshold(kRxPin, 20)) {
    flexe_rmt_rx_stage = 0xBAD00002;
    return;
  }
  if (!rmtSetRxMinThreshold(kRxPin, 3)) {
    flexe_rmt_rx_stage = 0xBAD00005;
    return;
  }
  if (!rmtReadAsync(kRxPin, received, &received_count)) {
    flexe_rmt_rx_stage = 0xBAD00003;
    return;
  }
  flexe_rmt_rx_stage = 1;
}

void loop() {
  const uint32_t stage = flexe_rmt_rx_stage;
  if ((stage != 1 && stage != 2) || !rmtReceiveCompleted(kRxPin)) {
    delay(1);
    return;
  }
  if (stage == 1) {
    flexe_rmt_rx_result[0] = received_count;
    flexe_rmt_rx_result[1] = received[0].val;
    flexe_rmt_rx_result[2] = received[1].val;
    Serial.printf("S3_RMT_RX_SHORT count=%u\n", (unsigned)received_count);
    received_count = RMT_SYMBOLS_OF(received);
    if (!rmtReadAsync(kRxPin, received, &received_count)) {
      flexe_rmt_rx_stage = 0xBAD00004;
      return;
    }
    flexe_rmt_rx_stage = 2;
    return;
  }
  flexe_rmt_rx_result[3] = received_count;
  for (size_t i = 0; i < received_count && i < 96; i++)
    flexe_rmt_rx_result[4 + i] = received[i].val;
  flexe_rmt_rx_stage = 0x1A7C0DE;
  Serial.printf("S3_RMT_RX_LONG count=%u\n", (unsigned)received_count);
}
