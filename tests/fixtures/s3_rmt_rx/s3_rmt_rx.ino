#include <Arduino.h>
#include "esp32-hal-rmt.h"

// These markers let the host test inject a frame only after the production
// Arduino/ESP-IDF RX driver has armed its channel.
extern "C" volatile uint32_t flexe_rmt_rx_stage = 0;
extern "C" volatile uint32_t flexe_rmt_rx_result[3] = {};

static constexpr int kRxPin = 4;
static rmt_data_t received[16] = {};
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
  if (!rmtReadAsync(kRxPin, received, &received_count)) {
    flexe_rmt_rx_stage = 0xBAD00003;
    return;
  }
  flexe_rmt_rx_stage = 1;
}

void loop() {
  if (flexe_rmt_rx_stage != 1 || !rmtReceiveCompleted(kRxPin)) {
    delay(1);
    return;
  }
  flexe_rmt_rx_result[0] = received_count;
  flexe_rmt_rx_result[1] = received[0].val;
  flexe_rmt_rx_result[2] = received[1].val;
  flexe_rmt_rx_stage = 0x1A7C0DE;
  Serial.printf("S3_RMT_RX count=%u first=%08x second=%08x\n",
                (unsigned)received_count, received[0].val,
                received[1].val);
}
