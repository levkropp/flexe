#include <Arduino.h>
#include <driver/pcnt.h>

volatile uint32_t flexe_pcnt_stage = 0;
volatile uint32_t flexe_pcnt_command = 0;
volatile uint32_t flexe_pcnt_result[20] = {};
volatile uint32_t flexe_pcnt_isr_calls = 0;
volatile uint32_t flexe_pcnt_isr_status = 0;

static constexpr pcnt_unit_t kUnit = PCNT_UNIT_0;
static constexpr pcnt_unit_t kUpperUnit = PCNT_UNIT_7;

static void fail(uint32_t stage, uint32_t detail) {
  flexe_pcnt_result[19] = detail;
  flexe_pcnt_stage = 0xBAD00000u | stage;
}

static void IRAM_ATTR onPcnt(void *) {
  uint32_t status = 0;
  pcnt_get_event_status(kUnit, &status);
  flexe_pcnt_isr_status = status;
  flexe_pcnt_isr_calls++;
}

static bool waitForCommand(uint32_t command) {
  for (unsigned i = 0; i < 10000 && flexe_pcnt_command != command; i++) {
    delay(1);
  }
  return flexe_pcnt_command == command;
}

void setup() {
  flexe_pcnt_stage = 1;

  pcnt_config_t config = {};
  config.pulse_gpio_num = 18;
  config.ctrl_gpio_num = 19;
  config.lctrl_mode = PCNT_MODE_KEEP;
  config.hctrl_mode = PCNT_MODE_REVERSE;
  config.pos_mode = PCNT_COUNT_INC;
  config.neg_mode = PCNT_COUNT_DIS;
  config.counter_h_lim = 5;
  config.counter_l_lim = -5;
  config.unit = kUnit;
  config.channel = PCNT_CHANNEL_0;
  esp_err_t err = pcnt_unit_config(&config);
  flexe_pcnt_result[0] = (uint32_t)err;
  if (err != ESP_OK) {
    fail(1, (uint32_t)err);
    return;
  }

  pcnt_config_t upper = {};
  upper.pulse_gpio_num = 22;
  upper.ctrl_gpio_num = PCNT_PIN_NOT_USED;
  upper.lctrl_mode = PCNT_MODE_KEEP;
  upper.hctrl_mode = PCNT_MODE_KEEP;
  upper.pos_mode = PCNT_COUNT_INC;
  upper.neg_mode = PCNT_COUNT_DIS;
  upper.counter_h_lim = 20;
  upper.counter_l_lim = -20;
  upper.unit = kUpperUnit;
  upper.channel = PCNT_CHANNEL_1;
  err = pcnt_unit_config(&upper);
  flexe_pcnt_result[1] = (uint32_t)err;
  if (err != ESP_OK) {
    fail(2, (uint32_t)err);
    return;
  }

  err = pcnt_set_filter_value(kUnit, 10);
  flexe_pcnt_result[2] = (uint32_t)err;
  if (err != ESP_OK) {
    fail(3, (uint32_t)err);
    return;
  }
  err = pcnt_filter_enable(kUnit);
  flexe_pcnt_result[3] = (uint32_t)err;
  if (err != ESP_OK) {
    fail(4, (uint32_t)err);
    return;
  }
  uint16_t filter = 0;
  err = pcnt_get_filter_value(kUnit, &filter);
  flexe_pcnt_result[4] = (uint32_t)err;
  flexe_pcnt_result[5] = filter;
  if (err != ESP_OK || filter != 10) {
    fail(5, filter);
    return;
  }

  err = pcnt_set_event_value(kUnit, PCNT_EVT_THRES_0, 3);
  flexe_pcnt_result[6] = (uint32_t)err;
  if (err != ESP_OK) {
    fail(6, (uint32_t)err);
    return;
  }
  err = pcnt_event_enable(kUnit, PCNT_EVT_THRES_0);
  flexe_pcnt_result[7] = (uint32_t)err;
  if (err != ESP_OK) {
    fail(7, (uint32_t)err);
    return;
  }
  err = pcnt_event_enable(kUnit, PCNT_EVT_H_LIM);
  flexe_pcnt_result[8] = (uint32_t)err;
  if (err != ESP_OK) {
    fail(8, (uint32_t)err);
    return;
  }

  err = pcnt_isr_service_install(0);
  flexe_pcnt_result[9] = (uint32_t)err;
  if (err != ESP_OK) {
    fail(9, (uint32_t)err);
    return;
  }
  err = pcnt_isr_handler_add(kUnit, onPcnt, nullptr);
  flexe_pcnt_result[10] = (uint32_t)err;
  if (err != ESP_OK) {
    fail(10, (uint32_t)err);
    return;
  }

  pcnt_counter_clear(kUnit);
  pcnt_counter_clear(kUpperUnit);
  pcnt_counter_resume(kUnit);
  pcnt_counter_resume(kUpperUnit);

  flexe_pcnt_stage = 2;
  if (!waitForCommand(1)) {
    fail(11, flexe_pcnt_command);
    return;
  }
  int16_t count = 0;
  pcnt_get_counter_value(kUnit, &count);
  flexe_pcnt_result[11] = (uint16_t)count;
  flexe_pcnt_result[12] = flexe_pcnt_isr_calls;
  flexe_pcnt_result[13] = flexe_pcnt_isr_status;
  if (count != 3 || flexe_pcnt_isr_calls != 1 ||
      !(flexe_pcnt_isr_status & PCNT_EVT_THRES_0)) {
    fail(12, (uint16_t)count);
    return;
  }

  flexe_pcnt_stage = 3;
  if (!waitForCommand(2)) {
    fail(13, flexe_pcnt_command);
    return;
  }
  pcnt_get_counter_value(kUnit, &count);
  flexe_pcnt_result[14] = (uint16_t)count;
  if (count != 1) {
    fail(14, (uint16_t)count);
    return;
  }

  pcnt_counter_pause(kUnit);
  flexe_pcnt_stage = 4;
  if (!waitForCommand(3)) {
    fail(15, flexe_pcnt_command);
    return;
  }
  pcnt_get_counter_value(kUnit, &count);
  flexe_pcnt_result[15] = (uint16_t)count;
  if (count != 1) {
    fail(16, (uint16_t)count);
    return;
  }

  pcnt_counter_clear(kUnit);
  pcnt_counter_resume(kUnit);
  flexe_pcnt_stage = 5;
  if (!waitForCommand(4)) {
    fail(17, flexe_pcnt_command);
    return;
  }
  pcnt_get_counter_value(kUnit, &count);
  flexe_pcnt_result[16] = (uint16_t)count;
  flexe_pcnt_result[17] = flexe_pcnt_isr_calls;
  int16_t upperCount = 0;
  pcnt_get_counter_value(kUpperUnit, &upperCount);
  flexe_pcnt_result[18] = (uint16_t)upperCount;
  if (count != 0 || flexe_pcnt_isr_calls != 3 || upperCount != 1 ||
      !(flexe_pcnt_isr_status & PCNT_EVT_H_LIM)) {
    fail(18, (uint16_t)count);
    return;
  }

  pcnt_isr_handler_remove(kUnit);
  pcnt_isr_service_uninstall();
  flexe_pcnt_stage = 0xC01A7E00u;
}

void loop() {
  delay(1000);
}
