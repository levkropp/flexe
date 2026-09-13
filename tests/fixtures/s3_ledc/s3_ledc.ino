#include <Arduino.h>

static constexpr uint8_t kPwmPin = 4;
static constexpr uint8_t kDuties[] = {64, 192, 96, 0};
static unsigned phase_index = 0;

void setup() {
  Serial.begin(115200);
  if (!ledcAttach(kPwmPin, 5000, 8)) {
    Serial.println("S3_LEDC_ATTACH_FAIL");
    return;
  }
  Serial.println("S3_LEDC_READY");
}

void loop() {
  const uint8_t duty = kDuties[phase_index % 4];
  if (!ledcWrite(kPwmPin, duty)) {
    Serial.println("S3_LEDC_WRITE_FAIL");
  } else {
    delay(2);  // Read back after at least one 5 kHz PWM period.
    Serial.printf("S3_LEDC duty=%u read=%u freq=%u\n",
                  duty, ledcRead(kPwmPin), ledcReadFreq(kPwmPin));
  }
  phase_index++;
  delay(100);
}
