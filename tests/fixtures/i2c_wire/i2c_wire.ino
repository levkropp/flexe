#include <Arduino.h>
#include <Wire.h>

volatile uint32_t flexe_i2c_stage = 0;
volatile uint32_t flexe_i2c_result[4] = {};
static constexpr uint8_t kLength = 40;

static void fail(uint32_t stage, uint32_t detail) {
  flexe_i2c_result[3] = detail;
  flexe_i2c_stage = 0xBAD00000u | stage;
}

void setup() {
  flexe_i2c_stage = 1;
  if (!Wire.begin(21, 22, 400000)) {
    fail(1, 0);
    return;
  }
  Wire.setTimeOut(1000);
  flexe_i2c_stage = 2;

  // Forty data bytes force ESP-IDF to refill the 32-byte hardware FIFO from
  // its real interrupt handler before issuing STOP.
  Wire.beginTransmission(0x34);
  Wire.write((uint8_t)0x20);
  for (uint8_t i = 0; i < kLength; ++i)
    Wire.write((uint8_t)(0x5Au ^ i));
  flexe_i2c_result[0] = Wire.endTransmission(true);
  if (flexe_i2c_result[0] != 0) {
    fail(2, flexe_i2c_result[0]);
    return;
  }
  flexe_i2c_stage = 3;

  // Arduino folds the no-STOP write into requestFrom(), producing the normal
  // register-device write/repeated-START/read transaction.
  Wire.beginTransmission(0x34);
  Wire.write((uint8_t)0x20);
  flexe_i2c_result[1] = Wire.endTransmission(false);
  if (flexe_i2c_result[1] != 0) {
    fail(3, flexe_i2c_result[1]);
    return;
  }

  size_t received = Wire.requestFrom((uint16_t)0x34,
                                     (size_t)kLength, true);
  flexe_i2c_result[2] = received;
  uint32_t checksum = 0;
  for (uint8_t i = 0; i < kLength; ++i) {
    if (!Wire.available()) {
      fail(4, i);
      return;
    }
    uint8_t value = (uint8_t)Wire.read();
    if (value != (uint8_t)(0x5Au ^ i)) {
      fail(5, ((uint32_t)i << 16) | value);
      return;
    }
    checksum = checksum * 33u + value;
  }
  if (received != kLength) {
    fail(6, received);
    return;
  }

  // An unattached address must report a Wire error rather than a false ACK.
  Wire.beginTransmission(0x35);
  flexe_i2c_result[3] = Wire.endTransmission(true);
  if (flexe_i2c_result[3] == 0) {
    fail(7, 0);
    return;
  }

  flexe_i2c_result[3] = checksum;
  flexe_i2c_stage = 0x1C2C0040u;
}

void loop() {
  delay(1000);
}
