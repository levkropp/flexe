#include <Arduino.h>
#include <driver/timer.h>

volatile uint32_t flexe_timer_group_stage = 0;
volatile uint32_t flexe_timer_group_result[32] = {};
volatile uint32_t flexe_timer_group_calls[4] = {};
volatile uint64_t flexe_timer_group_next_alarm = 2500;

static void fail(uint32_t stage, uint32_t detail) {
  flexe_timer_group_result[31] = detail;
  flexe_timer_group_stage = 0xBAD00000u | stage;
}

static bool IRAM_ATTR onTimer(void *opaque) {
  unsigned index = (unsigned)(uintptr_t)opaque - 1u;
  if (index >= 4u) {
    return false;
  }
  uint32_t calls = ++flexe_timer_group_calls[index];
  if (index == 1u && calls < 4u) {
    flexe_timer_group_next_alarm += 2500u;
    timer_group_set_alarm_value_in_isr(
        TIMER_GROUP_0, TIMER_1, flexe_timer_group_next_alarm);
  }
  return false;
}

static bool checkOk(unsigned first, unsigned last) {
  for (unsigned i = first; i <= last; i++) {
    if (flexe_timer_group_result[i] != ESP_OK) {
      return false;
    }
  }
  return true;
}

void setup() {
  flexe_timer_group_stage = 1;

  const timer_group_t groups[4] = {
      TIMER_GROUP_0, TIMER_GROUP_0, TIMER_GROUP_1, TIMER_GROUP_1};
  const timer_idx_t timers[4] = {TIMER_0, TIMER_1, TIMER_0, TIMER_1};
  const timer_count_dir_t directions[4] = {
      TIMER_COUNT_UP, TIMER_COUNT_UP, TIMER_COUNT_DOWN, TIMER_COUNT_UP};
  const bool autoreload[4] = {true, false, true, true};
  const uint64_t initial[4] = {0, 0, 1000, 0};
  const uint64_t alarms[4] = {1000, 2500, 0, 2000};

  for (unsigned i = 0; i < 4; i++) {
    timer_config_t config = {};
    config.alarm_en = TIMER_ALARM_EN;
    config.counter_en = TIMER_PAUSE;
    config.intr_type = TIMER_INTR_LEVEL;
    config.counter_dir = directions[i];
    config.auto_reload = autoreload[i] ? TIMER_AUTORELOAD_EN
                                       : TIMER_AUTORELOAD_DIS;
    config.divider = 80;
    flexe_timer_group_result[i] =
        (uint32_t)timer_init(groups[i], timers[i], &config);
  }
  if (!checkOk(0, 3)) {
    fail(1, flexe_timer_group_result[0]);
    return;
  }

  for (unsigned i = 0; i < 4; i++) {
    flexe_timer_group_result[4 + i] =
        (uint32_t)timer_set_counter_value(groups[i], timers[i], initial[i]);
    flexe_timer_group_result[8 + i] =
        (uint32_t)timer_set_alarm_value(groups[i], timers[i], alarms[i]);
    flexe_timer_group_result[12 + i] =
        (uint32_t)timer_isr_callback_add(
            groups[i], timers[i], onTimer,
            reinterpret_cast<void *>(static_cast<uintptr_t>(i + 1u)), 0);
    flexe_timer_group_result[16 + i] =
        (uint32_t)timer_start(groups[i], timers[i]);
  }
  if (!checkOk(4, 19)) {
    fail(2, flexe_timer_group_result[12]);
    return;
  }

  flexe_timer_group_stage = 2;
  delay(12);
  for (unsigned i = 0; i < 4; i++) {
    flexe_timer_group_result[20 + i] = flexe_timer_group_calls[i];
    timer_pause(groups[i], timers[i]);
  }
  if (flexe_timer_group_calls[0] < 10u ||
      flexe_timer_group_calls[1] != 4u ||
      flexe_timer_group_calls[2] < 10u ||
      flexe_timer_group_calls[3] < 5u) {
    fail(3, flexe_timer_group_calls[1]);
    return;
  }

  uint64_t pausedBefore = 0;
  uint64_t pausedAfter = 0;
  timer_get_counter_value(TIMER_GROUP_0, TIMER_0, &pausedBefore);
  delay(3);
  timer_get_counter_value(TIMER_GROUP_0, TIMER_0, &pausedAfter);
  flexe_timer_group_result[24] = (uint32_t)pausedBefore;
  flexe_timer_group_result[25] = (uint32_t)pausedAfter;
  if (pausedBefore != pausedAfter) {
    fail(4, (uint32_t)pausedAfter);
    return;
  }

  timer_config_t readback = {};
  flexe_timer_group_result[26] =
      (uint32_t)timer_get_config(TIMER_GROUP_1, TIMER_0, &readback);
  flexe_timer_group_result[27] =
      readback.divider |
      ((uint32_t)readback.counter_dir << 16) |
      ((uint32_t)readback.auto_reload << 20) |
      ((uint32_t)readback.counter_en << 24);
  uint64_t alarmReadback = 0;
  timer_get_alarm_value(TIMER_GROUP_0, TIMER_1, &alarmReadback);
  flexe_timer_group_result[28] = (uint32_t)alarmReadback;

  constexpr uint64_t kWideCounter = 0x1234567800000042ull;
  timer_set_counter_value(TIMER_GROUP_1, TIMER_1, kWideCounter);
  uint64_t wideReadback = 0;
  timer_get_counter_value(TIMER_GROUP_1, TIMER_1, &wideReadback);
  flexe_timer_group_result[29] = (uint32_t)wideReadback;
  flexe_timer_group_result[30] = (uint32_t)(wideReadback >> 32);
  if (flexe_timer_group_result[26] != ESP_OK || readback.divider != 80u ||
      readback.counter_dir != TIMER_COUNT_DOWN ||
      readback.auto_reload != TIMER_AUTORELOAD_EN ||
      readback.counter_en != TIMER_PAUSE || alarmReadback != 10000u ||
      wideReadback != kWideCounter) {
    fail(5, flexe_timer_group_result[27]);
    return;
  }

  for (unsigned i = 0; i < 4; i++) {
    timer_isr_callback_remove(groups[i], timers[i]);
    timer_deinit(groups[i], timers[i]);
  }
  flexe_timer_group_stage = 0x54494D47u;  // "TIMG"
}

void loop() {
  delay(1000);
}
