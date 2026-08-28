#include <Arduino.h>
#include <driver/ledc.h>

volatile uint32_t flexe_ledc_stage = 0;
volatile uint32_t flexe_ledc_result[13] = {};

static constexpr ledc_mode_t kSpeedMode = LEDC_HIGH_SPEED_MODE;
static constexpr ledc_timer_t kTimer = LEDC_TIMER_0;
static constexpr ledc_channel_t kChannel = LEDC_CHANNEL_0;

static void fail(uint32_t stage, uint32_t detail) {
  flexe_ledc_result[12] = detail;
  flexe_ledc_stage = 0xBAD00000u | stage;
}

void setup() {
  flexe_ledc_stage = 1;

  ledc_timer_config_t timer = {};
  timer.speed_mode = kSpeedMode;
  timer.duty_resolution = LEDC_TIMER_8_BIT;
  timer.timer_num = kTimer;
  timer.freq_hz = 5000;
  timer.clk_cfg = LEDC_AUTO_CLK;
  esp_err_t err = ledc_timer_config(&timer);
  flexe_ledc_result[0] = (uint32_t)err;
  if (err != ESP_OK) {
    fail(1, (uint32_t)err);
    return;
  }

  ledc_channel_config_t channel = {};
  channel.gpio_num = GPIO_NUM_21;
  channel.speed_mode = kSpeedMode;
  channel.channel = kChannel;
  channel.intr_type = LEDC_INTR_DISABLE;
  channel.timer_sel = kTimer;
  channel.duty = 64;
  channel.hpoint = 0;
  channel.flags.output_invert = 0;
  err = ledc_channel_config(&channel);
  flexe_ledc_result[1] = (uint32_t)err;
  if (err != ESP_OK) {
    fail(2, (uint32_t)err);
    return;
  }

  delay(2);
  flexe_ledc_result[2] = ledc_get_freq(kSpeedMode, kTimer);
  flexe_ledc_result[3] = ledc_get_duty(kSpeedMode, kChannel);
  if (flexe_ledc_result[2] != 5000 || flexe_ledc_result[3] != 64) {
    fail(3, flexe_ledc_result[3]);
    return;
  }

  flexe_ledc_stage = 2;
  err = ledc_set_duty(kSpeedMode, kChannel, 192);
  flexe_ledc_result[4] = (uint32_t)err;
  if (err != ESP_OK) {
    fail(4, (uint32_t)err);
    return;
  }
  err = ledc_update_duty(kSpeedMode, kChannel);
  flexe_ledc_result[5] = (uint32_t)err;
  if (err != ESP_OK) {
    fail(5, (uint32_t)err);
    return;
  }
  delay(2);
  flexe_ledc_result[6] = ledc_get_duty(kSpeedMode, kChannel);
  if (flexe_ledc_result[6] != 192) {
    fail(6, flexe_ledc_result[6]);
    return;
  }

  /* A blocking fade only returns after the genuine ESP-IDF LEDC ISR handles
     the hardware fade-end interrupt and releases the driver's semaphore. */
  flexe_ledc_stage = 3;
  err = ledc_fade_func_install(0);
  flexe_ledc_result[7] = (uint32_t)err;
  if (err != ESP_OK) {
    fail(7, (uint32_t)err);
    return;
  }
  err = ledc_set_fade_with_step(kSpeedMode, kChannel, 96, 8, 2);
  flexe_ledc_result[8] = (uint32_t)err;
  if (err != ESP_OK) {
    fail(8, (uint32_t)err);
    return;
  }
  err = ledc_fade_start(kSpeedMode, kChannel, LEDC_FADE_WAIT_DONE);
  flexe_ledc_result[9] = (uint32_t)err;
  if (err != ESP_OK) {
    fail(9, (uint32_t)err);
    return;
  }
  flexe_ledc_result[10] = ledc_get_duty(kSpeedMode, kChannel);
  if (flexe_ledc_result[10] != 96) {
    fail(10, flexe_ledc_result[10]);
    return;
  }

  flexe_ledc_stage = 4;
  err = ledc_stop(kSpeedMode, kChannel, 0);
  flexe_ledc_result[11] = (uint32_t)err;
  if (err != ESP_OK) {
    fail(11, (uint32_t)err);
    return;
  }
  delay(1);
  ledc_fade_func_uninstall();
  flexe_ledc_stage = 0x1EDCC0DEu;
}

void loop() {
  delay(1000);
}
