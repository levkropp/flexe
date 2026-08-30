#include <Arduino.h>
#include <esp_intr_alloc.h>
#include <soc/frc_timer_reg.h>
#include <soc/soc.h>

volatile uint32_t flexe_frc_timer_stage = 0;
volatile uint32_t flexe_frc_timer_result[24] = {};
volatile uint32_t flexe_frc_timer_calls[2] = {};
volatile uint32_t flexe_frc_timer_first_count[2] = {};
volatile uint32_t flexe_frc_timer_next_alarm = 0;

static intr_handle_t frcHandles[2] = {};

static void fail(uint32_t stage, uint32_t detail) {
  flexe_frc_timer_result[23] = detail;
  flexe_frc_timer_stage = 0xBAD00000u | stage;
}

static void IRAM_ATTR onFrc1(void *) {
  uint32_t calls = ++flexe_frc_timer_calls[0];
  if (calls == 1u) {
    flexe_frc_timer_first_count[0] = REG_READ(FRC_TIMER_COUNT_REG(0));
  }
  REG_WRITE(FRC_TIMER_INT_REG(0), FRC_TIMER_INT_CLR);
}

static void IRAM_ATTR onFrc2(void *) {
  /* FRC2 is configured for edge mode, so STATUS is never latched and no
     explicit clear is required before programming the following compare. */
  uint32_t count = REG_READ(FRC_TIMER_COUNT_REG(1));
  uint32_t calls = ++flexe_frc_timer_calls[1];
  if (calls == 1u) {
    flexe_frc_timer_first_count[1] = count;
  }
  flexe_frc_timer_next_alarm = count + 80000u;
  REG_WRITE(FRC_TIMER_ALARM_REG(1), flexe_frc_timer_next_alarm);
}

void setup() {
  flexe_frc_timer_stage = 1;
  for (unsigned timer = 0; timer < 2; ++timer) {
    REG_WRITE(FRC_TIMER_CTRL_REG(timer), 0);
    REG_WRITE(FRC_TIMER_INT_REG(timer), FRC_TIMER_INT_CLR);
  }

  flexe_frc_timer_result[0] = (uint32_t)esp_intr_alloc(
      ETS_TIMER1_INTR_SOURCE, ESP_INTR_FLAG_LEVEL1 | ESP_INTR_FLAG_IRAM,
      onFrc1, nullptr, &frcHandles[0]);
  flexe_frc_timer_result[1] = (uint32_t)esp_intr_alloc(
      ETS_TIMER2_INTR_SOURCE, ESP_INTR_FLAG_LEVEL1 | ESP_INTR_FLAG_IRAM,
      onFrc2, nullptr, &frcHandles[1]);
  flexe_frc_timer_result[2] =
      (frcHandles[0] != nullptr ? 1u : 0u) |
      (frcHandles[1] != nullptr ? 2u : 0u);
  if (flexe_frc_timer_result[0] != ESP_OK ||
      flexe_frc_timer_result[1] != ESP_OK ||
      flexe_frc_timer_result[2] != 3u) {
    fail(1, flexe_frc_timer_result[2]);
    return;
  }

  /* FRC1 counts down at APB speed and reloads at zero. FRC2 starts just
     below UINT32_MAX and compares just above zero, exercising wraparound. */
  REG_WRITE(FRC_TIMER_LOAD_REG(0), 80000u);
  flexe_frc_timer_result[3] = REG_READ(FRC_TIMER_LOAD_REG(0));
  REG_WRITE(FRC_TIMER_CTRL_REG(0),
            FRC_TIMER_ENABLE | FRC_TIMER_AUTOLOAD |
                FRC_TIMER_PRESCALER_1 | FRC_TIMER_LEVEL_INT);
  REG_WRITE(FRC_TIMER_LOAD_REG(1), 0xFFFFF000u);
  REG_WRITE(FRC_TIMER_ALARM_REG(1), 0x00001000u);
  REG_WRITE(FRC_TIMER_CTRL_REG(1),
            FRC_TIMER_ENABLE | FRC_TIMER_PRESCALER_1);
  flexe_frc_timer_result[4] = REG_READ(FRC_TIMER_CTRL_REG(0));
  flexe_frc_timer_result[5] = REG_READ(FRC_TIMER_CTRL_REG(1));

  flexe_frc_timer_stage = 2;
  delay(7);
  REG_WRITE(FRC_TIMER_CTRL_REG(0), 0);
  REG_WRITE(FRC_TIMER_CTRL_REG(1), 0);
  REG_WRITE(FRC_TIMER_INT_REG(0), FRC_TIMER_INT_CLR);
  REG_WRITE(FRC_TIMER_INT_REG(1), FRC_TIMER_INT_CLR);
  flexe_frc_timer_result[6] = flexe_frc_timer_calls[0];
  flexe_frc_timer_result[7] = flexe_frc_timer_calls[1];
  flexe_frc_timer_result[8] = flexe_frc_timer_first_count[1];
  flexe_frc_timer_result[9] = flexe_frc_timer_next_alarm;
  flexe_frc_timer_result[20] = flexe_frc_timer_first_count[0];
  if (flexe_frc_timer_result[3] != 80000u ||
      flexe_frc_timer_calls[0] < 5u || flexe_frc_timer_calls[1] < 5u ||
      flexe_frc_timer_first_count[1] < 0x1000u ||
      flexe_frc_timer_first_count[1] > 0x20000u ||
      flexe_frc_timer_next_alarm == 0x1000u ||
      flexe_frc_timer_first_count[0] < 78000u ||
      flexe_frc_timer_first_count[0] > 80000u) {
    fail(2, (flexe_frc_timer_calls[0] << 16) | flexe_frc_timer_calls[1]);
    return;
  }

  /* FRC1 /16 should lose about 5000 ticks per millisecond, then freeze
     exactly when ENABLE is cleared. */
  REG_WRITE(FRC_TIMER_LOAD_REG(0), 10000u);
  REG_WRITE(FRC_TIMER_CTRL_REG(0),
            FRC_TIMER_ENABLE | FRC_TIMER_PRESCALER_16 |
                FRC_TIMER_LEVEL_INT);
  flexe_frc_timer_result[10] = REG_READ(FRC_TIMER_CTRL_REG(0));
  flexe_frc_timer_result[11] = REG_READ(FRC_TIMER_COUNT_REG(0));
  delay(1);
  flexe_frc_timer_result[12] = REG_READ(FRC_TIMER_COUNT_REG(0));
  REG_WRITE(FRC_TIMER_CTRL_REG(0), 0);
  flexe_frc_timer_result[13] = REG_READ(FRC_TIMER_COUNT_REG(0));
  delay(1);
  flexe_frc_timer_result[14] = REG_READ(FRC_TIMER_COUNT_REG(0));
  uint32_t frc1Delta = flexe_frc_timer_result[11] - flexe_frc_timer_result[12];
  if (frc1Delta < 3000u || frc1Delta > 7000u ||
      flexe_frc_timer_result[13] != flexe_frc_timer_result[14]) {
    fail(3, frc1Delta);
    return;
  }

  /* FRC2 remains an up-counter under /256 and likewise freezes when
     disabled. The alarm stays far enough away that no ISR is expected. */
  REG_WRITE(FRC_TIMER_LOAD_REG(1), 100u);
  REG_WRITE(FRC_TIMER_ALARM_REG(1), 100000u);
  REG_WRITE(FRC_TIMER_CTRL_REG(1),
            FRC_TIMER_ENABLE | FRC_TIMER_PRESCALER_256);
  flexe_frc_timer_result[15] = REG_READ(FRC_TIMER_CTRL_REG(1));
  flexe_frc_timer_result[16] = REG_READ(FRC_TIMER_COUNT_REG(1));
  delay(1);
  flexe_frc_timer_result[17] = REG_READ(FRC_TIMER_COUNT_REG(1));
  REG_WRITE(FRC_TIMER_CTRL_REG(1), 0);
  uint32_t frc2Delta = flexe_frc_timer_result[17] - flexe_frc_timer_result[16];
  if (frc2Delta < 150u || frc2Delta > 500u) {
    fail(4, frc2Delta);
    return;
  }

  REG_WRITE(FRC_TIMER_INT_REG(0), FRC_TIMER_INT_CLR);
  REG_WRITE(FRC_TIMER_INT_REG(1), FRC_TIMER_INT_CLR);
  flexe_frc_timer_result[18] = REG_READ(FRC_TIMER_CTRL_REG(0));
  flexe_frc_timer_result[19] = REG_READ(FRC_TIMER_CTRL_REG(1));
  esp_intr_free(frcHandles[0]);
  esp_intr_free(frcHandles[1]);
  flexe_frc_timer_result[23] = 0;
  flexe_frc_timer_stage = 0x46524332u;  // "FRC2"
}

void loop() {
  delay(1000);
}
