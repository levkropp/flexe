#include <Arduino.h>

volatile uint32_t flexe_analog_stage = 0;
volatile uint32_t flexe_analog_result[4] = {};

static void fail(uint32_t stage, uint32_t detail) {
  flexe_analog_result[3] = detail;
  flexe_analog_stage = 0xBAD00000u | stage;
}

void setup() {
  flexe_analog_stage = 1;
  analogReadResolution(12);

  // GPIO34 is ADC1 channel 6; GPIO4 is ADC2 channel 0 on classic ESP32.
  flexe_analog_result[0] = analogRead(34);
  flexe_analog_result[1] = analogRead(4);
  if (flexe_analog_result[0] != 0x0A55u) {
    fail(1, flexe_analog_result[0]);
    return;
  }
  if (flexe_analog_result[1] != 0x05AAu) {
    fail(2, flexe_analog_result[1]);
    return;
  }

  flexe_analog_stage = 2;
  analogReadResolution(9);
  flexe_analog_result[2] = analogRead(34);
  flexe_analog_result[3] = analogRead(4);
  if (flexe_analog_result[2] != 0x0055u) {
    fail(3, flexe_analog_result[2]);
    return;
  }
  if (flexe_analog_result[3] != 0x01AAu) {
    fail(4, flexe_analog_result[3]);
    return;
  }

  // Exercise the stock Arduino-to-IDF-to-RTCIO DAC path. The host validates
  // both intermediate events and final register-backed channel state.
  flexe_analog_stage = 3;
  dacWrite(25, 0x35);
  dacWrite(26, 0xCA);
  flexe_analog_stage = 4;
  dacDisable(25);

  flexe_analog_stage = 0xADC0DAC0u;
}

void loop() {
  delay(1000);
}
