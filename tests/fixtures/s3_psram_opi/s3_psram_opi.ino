#include <Arduino.h>
#include <esp_heap_caps.h>

static constexpr size_t kBytes = 8192;
static uint8_t *buffer;
static unsigned pass_count;

void setup() {
  Serial.begin(115200);
  const uint32_t size = ESP.getPsramSize();
  if (size != 8u * 1024u * 1024u) {
    Serial.printf("S3_PSRAM_SIZE_FAIL %u\n", size);
    return;
  }
  buffer = static_cast<uint8_t *>(
      heap_caps_malloc(kBytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
  if (!buffer) {
    Serial.println("S3_PSRAM_ALLOC_FAIL");
    return;
  }
  Serial.printf("S3_PSRAM_READY size=%u\n", size);
}

void loop() {
  if (!buffer) {
    delay(1000);
    return;
  }
  for (size_t i = 0; i < kBytes; ++i)
    buffer[i] = static_cast<uint8_t>((i * 37u) ^ (i >> 3) ^ pass_count);
  for (size_t i = 0; i < kBytes; ++i) {
    const uint8_t expected =
        static_cast<uint8_t>((i * 37u) ^ (i >> 3) ^ pass_count);
    if (buffer[i] != expected) {
      Serial.printf("S3_PSRAM_DATA_FAIL pass=%u offset=%u got=%u want=%u\n",
                    pass_count, static_cast<unsigned>(i), buffer[i], expected);
      buffer = nullptr;
      return;
    }
  }
  Serial.printf("S3_PSRAM_PASS %u\n", ++pass_count);
  delay(100);
}
