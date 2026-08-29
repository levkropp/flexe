#include <Arduino.h>
#include <driver/mcpwm.h>

volatile uint32_t flexe_mcpwm_stage = 0;
volatile uint32_t flexe_mcpwm_command = 0;
volatile uint32_t flexe_mcpwm_result[32] = {};
volatile uint32_t flexe_mcpwm_capture_calls = 0;
volatile uint32_t flexe_mcpwm_capture_first = 0;
volatile uint32_t flexe_mcpwm_capture_last = 0;
volatile uint32_t flexe_mcpwm_capture_edge = 0;

static void fail(uint32_t stage, uint32_t detail) {
  flexe_mcpwm_result[31] = detail;
  flexe_mcpwm_stage = 0xBAD00000u | stage;
}

static bool waitForCommand(uint32_t command) {
  for (unsigned i = 0; i < 10000 && flexe_mcpwm_command != command; i++) {
    delay(1);
  }
  return flexe_mcpwm_command == command;
}

static bool IRAM_ATTR onCapture(mcpwm_unit_t unit,
                                mcpwm_capture_channel_id_t channel,
                                const cap_event_data_t *event,
                                void *) {
  if (unit != MCPWM_UNIT_0 || channel != MCPWM_SELECT_CAP0 || !event) {
    return false;
  }
  if (flexe_mcpwm_capture_calls == 0) {
    flexe_mcpwm_capture_first = event->cap_value;
  }
  flexe_mcpwm_capture_last = event->cap_value;
  flexe_mcpwm_capture_edge = (uint32_t)event->cap_edge;
  flexe_mcpwm_capture_calls++;
  return false;
}

void setup() {
  flexe_mcpwm_stage = 1;

  flexe_mcpwm_result[0] = (uint32_t)mcpwm_gpio_init(
      MCPWM_UNIT_0, MCPWM0A, 18);
  flexe_mcpwm_result[1] = (uint32_t)mcpwm_gpio_init(
      MCPWM_UNIT_0, MCPWM0B, 19);
  flexe_mcpwm_result[2] = (uint32_t)mcpwm_gpio_init(
      MCPWM_UNIT_1, MCPWM2B, 23);
  flexe_mcpwm_result[3] = (uint32_t)mcpwm_gpio_init(
      MCPWM_UNIT_0, MCPWM_SYNC_0, 32);
  flexe_mcpwm_result[4] = (uint32_t)mcpwm_gpio_init(
      MCPWM_UNIT_0, MCPWM_FAULT_0, 35);
  flexe_mcpwm_result[5] = (uint32_t)mcpwm_gpio_init(
      MCPWM_UNIT_0, MCPWM_CAP_0, 34);
  for (unsigned i = 0; i < 6; i++) {
    if (flexe_mcpwm_result[i] != ESP_OK) {
      fail(1, flexe_mcpwm_result[i]);
      return;
    }
  }

  mcpwm_config_t primary = {};
  primary.frequency = 20000;
  primary.cmpr_a = 25.0f;
  primary.cmpr_b = 75.0f;
  primary.duty_mode = MCPWM_DUTY_MODE_0;
  primary.counter_mode = MCPWM_UP_COUNTER;
  flexe_mcpwm_result[6] = (uint32_t)mcpwm_init(
      MCPWM_UNIT_0, MCPWM_TIMER_0, &primary);

  mcpwm_config_t upper = {};
  upper.frequency = 1000;
  upper.cmpr_a = 40.0f;
  upper.cmpr_b = 60.0f;
  upper.duty_mode = MCPWM_DUTY_MODE_0;
  upper.counter_mode = MCPWM_UP_COUNTER;
  flexe_mcpwm_result[7] = (uint32_t)mcpwm_init(
      MCPWM_UNIT_1, MCPWM_TIMER_2, &upper);
  flexe_mcpwm_result[8] = mcpwm_get_frequency(
      MCPWM_UNIT_0, MCPWM_TIMER_0);
  flexe_mcpwm_result[9] = (uint32_t)(mcpwm_get_duty(
      MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_A) * 100.0f + 0.5f);
  flexe_mcpwm_result[10] = mcpwm_get_frequency(
      MCPWM_UNIT_1, MCPWM_TIMER_2);
  flexe_mcpwm_result[11] = (uint32_t)(mcpwm_get_duty(
      MCPWM_UNIT_1, MCPWM_TIMER_2, MCPWM_GEN_B) * 100.0f + 0.5f);

  mcpwm_carrier_config_t carrier = {};
  carrier.carrier_period = 3;
  carrier.carrier_duty = 5;
  carrier.pulse_width_in_os = 2;
  carrier.carrier_os_mode = MCPWM_ONESHOT_MODE_EN;
  carrier.carrier_ivt_mode = MCPWM_CARRIER_OUT_IVT_DIS;
  flexe_mcpwm_result[12] = (uint32_t)mcpwm_carrier_init(
      MCPWM_UNIT_0, MCPWM_TIMER_0, &carrier);
  flexe_mcpwm_result[13] = (uint32_t)mcpwm_deadtime_enable(
      MCPWM_UNIT_0, MCPWM_TIMER_0,
      MCPWM_ACTIVE_HIGH_COMPLIMENT_MODE, 9, 13);

  mcpwm_sync_config_t sync = {};
  sync.sync_sig = MCPWM_SELECT_GPIO_SYNC0;
  sync.timer_val = 500;
  sync.count_direction = MCPWM_TIMER_DIRECTION_UP;
  flexe_mcpwm_result[14] = (uint32_t)mcpwm_sync_configure(
      MCPWM_UNIT_0, MCPWM_TIMER_0, &sync);

  flexe_mcpwm_result[15] = (uint32_t)mcpwm_fault_init(
      MCPWM_UNIT_0, MCPWM_HIGH_LEVEL_TGR, MCPWM_SELECT_F0);
  flexe_mcpwm_result[16] = (uint32_t)mcpwm_fault_set_cyc_mode(
      MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_SELECT_F0,
      MCPWM_ACTION_FORCE_LOW, MCPWM_ACTION_FORCE_HIGH);

  mcpwm_capture_config_t capture = {};
  capture.cap_edge = MCPWM_BOTH_EDGE;
  capture.cap_prescale = 1;
  capture.capture_cb = onCapture;
  capture.user_data = nullptr;
  flexe_mcpwm_result[17] = (uint32_t)mcpwm_capture_enable_channel(
      MCPWM_UNIT_0, MCPWM_SELECT_CAP0, &capture);

  for (unsigned i = 6; i <= 7; i++) {
    if (flexe_mcpwm_result[i] != ESP_OK) {
      fail(2, flexe_mcpwm_result[i]);
      return;
    }
  }
  for (unsigned i = 12; i <= 17; i++) {
    if (flexe_mcpwm_result[i] != ESP_OK) {
      fail(3, flexe_mcpwm_result[i]);
      return;
    }
  }
  if (flexe_mcpwm_result[8] != 20000 ||
      flexe_mcpwm_result[9] != 2400 ||
      flexe_mcpwm_result[10] != 1000 ||
      flexe_mcpwm_result[11] != 6000) {
    fail(4, flexe_mcpwm_result[9]);
    return;
  }

  flexe_mcpwm_stage = 2;
  if (!waitForCommand(1)) {
    fail(5, flexe_mcpwm_command);
    return;
  }
  flexe_mcpwm_result[18] = flexe_mcpwm_capture_calls;
  flexe_mcpwm_result[19] = flexe_mcpwm_capture_edge;
  flexe_mcpwm_result[20] = flexe_mcpwm_capture_first;
  flexe_mcpwm_result[21] = flexe_mcpwm_capture_last;
  if (flexe_mcpwm_capture_calls != 2 ||
      flexe_mcpwm_capture_edge != MCPWM_NEG_EDGE ||
      flexe_mcpwm_capture_last <= flexe_mcpwm_capture_first) {
    fail(6, flexe_mcpwm_capture_calls);
    return;
  }

  flexe_mcpwm_result[22] = (uint32_t)mcpwm_set_signal_high(
      MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_A);
  delay(2);
  flexe_mcpwm_stage = 3;
  if (!waitForCommand(2)) {
    fail(7, flexe_mcpwm_command);
    return;
  }

  flexe_mcpwm_result[23] = (uint32_t)mcpwm_set_signal_low(
      MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_A);
  delay(2);
  flexe_mcpwm_stage = 4;
  if (!waitForCommand(3)) {
    fail(8, flexe_mcpwm_command);
    return;
  }

  flexe_mcpwm_result[24] = (uint32_t)mcpwm_set_duty_type(
      MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_A, MCPWM_DUTY_MODE_0);
  flexe_mcpwm_result[25] = (uint32_t)mcpwm_set_frequency(
      MCPWM_UNIT_0, MCPWM_TIMER_0, 10000);
  flexe_mcpwm_result[26] = (uint32_t)mcpwm_set_duty(
      MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_A, 75.0f);
  delay(2);
  flexe_mcpwm_result[27] = mcpwm_get_frequency(
      MCPWM_UNIT_0, MCPWM_TIMER_0);
  flexe_mcpwm_result[28] = (uint32_t)(mcpwm_get_duty(
      MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_A) * 100.0f + 0.5f);
  flexe_mcpwm_stage = 5;
  if (!waitForCommand(4)) {
    fail(9, flexe_mcpwm_command);
    return;
  }

  mcpwm_fault_deinit(MCPWM_UNIT_0, MCPWM_SELECT_F0);
  mcpwm_capture_disable_channel(MCPWM_UNIT_0, MCPWM_SELECT_CAP0);
  mcpwm_carrier_disable(MCPWM_UNIT_0, MCPWM_TIMER_0);
  mcpwm_deadtime_disable(MCPWM_UNIT_0, MCPWM_TIMER_0);
  flexe_mcpwm_result[29] = (uint32_t)mcpwm_stop(
      MCPWM_UNIT_0, MCPWM_TIMER_0);
  flexe_mcpwm_result[30] = (uint32_t)mcpwm_stop(
      MCPWM_UNIT_1, MCPWM_TIMER_2);
  delay(2);

  if (flexe_mcpwm_result[22] != ESP_OK ||
      flexe_mcpwm_result[23] != ESP_OK ||
      flexe_mcpwm_result[24] != ESP_OK ||
      flexe_mcpwm_result[25] != ESP_OK ||
      flexe_mcpwm_result[26] != ESP_OK ||
      flexe_mcpwm_result[27] != 10000 ||
      flexe_mcpwm_result[28] != 7500 ||
      flexe_mcpwm_result[29] != ESP_OK ||
      flexe_mcpwm_result[30] != ESP_OK) {
    fail(10, flexe_mcpwm_result[28]);
    return;
  }

  flexe_mcpwm_stage = 0x4D435057u;  // "MCPW"
}

void loop() {
  delay(1000);
}
