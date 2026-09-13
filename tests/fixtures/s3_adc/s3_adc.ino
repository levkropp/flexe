#include <Arduino.h>

// ESP32-S3 GPIO4 is ADC1 channel 3; GPIO11 is ADC2 channel 0.
// A host replay supplies their raw ADC samples through adc_in events.
void setup() {
  Serial.begin(115200);
  analogReadResolution(12);
  Serial.println("S3_ADC_READY");
}

void loop() {
  Serial.printf("S3_ADC %u %u\n", analogRead(4), analogRead(11));
  delay(1000);
}
