#include <Arduino.h>
#include <esp_arduino_version.h>
#include <driver/sigmadelta.h>
#include <soc/gpio_reg.h>
#include <soc/gpio_sd_reg.h>
#include <soc/soc.h>

volatile uint32_t flexe_sigmadelta_stage = 0;
volatile uint32_t flexe_sigmadelta_result[16] = {};

static constexpr uint32_t kSuccessMarker = 0x5349474Du;  // "SIGM"

static void fail(uint32_t stage, uint32_t detail) {
  flexe_sigmadelta_result[15] = detail;
  flexe_sigmadelta_stage = 0xBAD00000u | stage;
}

void setup() {
  flexe_sigmadelta_stage = 1;

  // Exercise Arduino's public wrapper, which enters ESP-IDF's genuine
  // sigmadelta driver and HAL before touching the classic GPIO_SD registers.
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  // Arduino 3.x allocates the channel from the pin and returns success rather
  // than the selected frequency. Preserve the result ABI used by the host
  // while still driving the current public API all the way through its SDM
  // driver and HAL.
  bool attached = sigmaDeltaAttach(18, 312500);
  bool written = attached && sigmaDeltaWrite(18, 192);
  flexe_sigmadelta_result[0] = attached ? 312500u : 0u;
  flexe_sigmadelta_result[1] = written ? 192u : 0u;
  flexe_sigmadelta_result[14] = 3u;
#else
  flexe_sigmadelta_result[0] = sigmaDeltaSetup(18, 0, 312500);
  sigmaDeltaWrite(0, 192);  // Arduino unsigned duty -> hardware signed +64.
  flexe_sigmadelta_result[1] = sigmaDeltaRead(0);
#endif
  flexe_sigmadelta_result[2] = REG_READ(GPIO_SIGMADELTA0_REG);
  flexe_sigmadelta_result[3] =
      REG_READ(GPIO_FUNC18_OUT_SEL_CFG_REG) & 0x1FFu;
  if (flexe_sigmadelta_result[0] != 312500u ||
      flexe_sigmadelta_result[1] != 192u ||
#if ESP_ARDUINO_VERSION_MAJOR >= 3
      flexe_sigmadelta_result[2] != 0x0000FF40u ||
#else
      flexe_sigmadelta_result[2] != 0x00000040u ||
#endif
      flexe_sigmadelta_result[3] != 100u) {
    fail(1, flexe_sigmadelta_result[2]);
    return;
  }

  flexe_sigmadelta_stage = 2;
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  // IDF intentionally rejects mixing its new SDM and deprecated sigma-delta
  // drivers. Exercise a second dynamically allocated channel through the
  // pin-based Arduino 3.x API; it follows channel 0 and therefore uses 1.
  // The new API names the modulator sample clock directly. Prescalers 3 and
  // 7 therefore correspond to 20 MHz and 10 MHz, respectively; their visible
  // 256-step PDM periods remain 78.125 kHz and 39.062 kHz.
  bool second_attached = sigmaDeltaAttach(23, 20000000);
  bool second_written = second_attached && sigmaDeltaWrite(23, 64);
  flexe_sigmadelta_result[4] = second_attached && second_written ? 0u : 1u;
  flexe_sigmadelta_result[5] = REG_READ(GPIO_SIGMADELTA1_REG);
  flexe_sigmadelta_result[6] = sigmaDeltaDetach(23) ? 0u : 1u;
  flexe_sigmadelta_result[7] = sigmaDeltaAttach(22, 10000000) ? 0u : 1u;
  flexe_sigmadelta_result[8] = sigmaDeltaWrite(22, 96) ? 0u : 1u;
  flexe_sigmadelta_result[9] = REG_READ(GPIO_SIGMADELTA1_REG);
  flexe_sigmadelta_result[10] =
      REG_READ(GPIO_FUNC22_OUT_SEL_CFG_REG) & 0x1FFu;
  flexe_sigmadelta_result[11] = REG_READ(GPIO_ENABLE_REG);
  flexe_sigmadelta_result[12] = REG_READ(GPIO_SIGMADELTA_VERSION_REG);

  if (flexe_sigmadelta_result[4] != 0u ||
      flexe_sigmadelta_result[5] != 0x000003C0u ||
      flexe_sigmadelta_result[6] != 0u ||
      flexe_sigmadelta_result[7] != 0u ||
      flexe_sigmadelta_result[8] != 0u ||
      flexe_sigmadelta_result[9] != 0x000007E0u ||
      flexe_sigmadelta_result[10] != 101u ||
      (flexe_sigmadelta_result[11] & ((1u << 18) | (1u << 22))) !=
          ((1u << 18) | (1u << 22)) ||
      flexe_sigmadelta_result[12] != 0x01506190u) {
    fail(2, flexe_sigmadelta_result[9]);
    return;
  }
#else
  sigmadelta_config_t config = {};
  config.channel = SIGMADELTA_CHANNEL_7;
  config.sigmadelta_prescale = 3;
  config.sigmadelta_duty = -64;
  config.sigmadelta_gpio = GPIO_NUM_23;
  flexe_sigmadelta_result[4] = sigmadelta_config(&config);
  flexe_sigmadelta_result[5] = REG_READ(GPIO_SIGMADELTA7_REG);
  flexe_sigmadelta_result[6] =
      sigmadelta_set_prescale(SIGMADELTA_CHANNEL_7, 7);
  flexe_sigmadelta_result[7] =
      sigmadelta_set_duty(SIGMADELTA_CHANNEL_7, -32);
  flexe_sigmadelta_result[8] =
      sigmadelta_set_pin(SIGMADELTA_CHANNEL_7, GPIO_NUM_22);
  flexe_sigmadelta_result[9] = REG_READ(GPIO_SIGMADELTA7_REG);
  flexe_sigmadelta_result[10] =
      REG_READ(GPIO_FUNC22_OUT_SEL_CFG_REG) & 0x1FFu;
  flexe_sigmadelta_result[11] = REG_READ(GPIO_ENABLE_REG);
  flexe_sigmadelta_result[12] = REG_READ(GPIO_SIGMADELTA_VERSION_REG);

  if (flexe_sigmadelta_result[4] != ESP_OK ||
      flexe_sigmadelta_result[5] != 0x000003C0u ||
      flexe_sigmadelta_result[6] != ESP_OK ||
      flexe_sigmadelta_result[7] != ESP_OK ||
      flexe_sigmadelta_result[8] != ESP_OK ||
      flexe_sigmadelta_result[9] != 0x000007E0u ||
      flexe_sigmadelta_result[10] != 107u ||
      (flexe_sigmadelta_result[11] & ((1u << 18) | (1u << 22))) !=
          ((1u << 18) | (1u << 22)) ||
      flexe_sigmadelta_result[12] != 0x01506190u) {
    fail(2, flexe_sigmadelta_result[9]);
    return;
  }
  flexe_sigmadelta_result[14] = 2u;
#endif

  flexe_sigmadelta_result[13] = 1u;
  flexe_sigmadelta_result[15] = 0u;
  flexe_sigmadelta_stage = kSuccessMarker;
}

void loop() {
  delay(1000);
}
