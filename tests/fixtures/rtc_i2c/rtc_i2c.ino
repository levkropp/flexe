#include <Arduino.h>
#include <soc/rtc_i2c_reg.h>
#include <soc/sens_reg.h>
#include <soc/soc.h>

static constexpr uint32_t kSuccessMarker = 0x12C0C0DEu;

volatile uint32_t flexe_rtc_i2c_stage = 0;
volatile uint32_t flexe_rtc_i2c_result[16] = {};

static void fail(uint32_t stage, uint32_t detail) {
  flexe_rtc_i2c_result[15] = detail;
  flexe_rtc_i2c_stage = 0xBAD00000u | stage;
}

static uint32_t rtcControl(bool write, uint8_t selector,
                           uint8_t subaddress, uint8_t value,
                           uint8_t lowBit, uint8_t highBit) {
  return static_cast<uint32_t>(subaddress) |
         (static_cast<uint32_t>(value) << 8) |
         ((static_cast<uint32_t>(lowBit) & 7u) << 16) |
         ((static_cast<uint32_t>(highBit) & 7u) << 19) |
         ((static_cast<uint32_t>(selector) & 0xFu) << 22) |
         (write ? 1u << 27 : 0u);
}

static bool rtcStart(uint32_t control) {
  REG_WRITE(SENS_SAR_I2C_CTRL_REG,
            SENS_SAR_I2C_START_FORCE | control);
  REG_WRITE(SENS_SAR_I2C_CTRL_REG,
            SENS_SAR_I2C_START_FORCE | SENS_SAR_I2C_START | control);
  for (unsigned spin = 0; spin < 100000u; ++spin) {
    if (REG_READ(SENS_SAR_SLAVE_ADDR4_REG) & SENS_I2C_DONE)
      return true;
  }
  return false;
}

void setup() {
  flexe_rtc_i2c_stage = 1;

  // Values are the 100 kHz examples from the classic ESP32 TRM.
  REG_WRITE(RTC_I2C_SCL_LOW_PERIOD_REG, 40u);
  REG_WRITE(RTC_I2C_SCL_HIGH_PERIOD_REG, 40u);
  REG_WRITE(RTC_I2C_SDA_DUTY_REG, 16u);
  REG_WRITE(RTC_I2C_SCL_START_PERIOD_REG, 30u);
  REG_WRITE(RTC_I2C_SCL_STOP_PERIOD_REG, 44u);
  REG_WRITE(RTC_I2C_TIMEOUT_REG, 200u);
  REG_WRITE(RTC_I2C_CTRL_REG, RTC_I2C_MS_MODE);
  REG_WRITE(RTC_I2C_INT_EN_REG, 0x1E0u);

  // Address selector 0 is the attached target; selector 1 intentionally NACKs.
  REG_WRITE(SENS_SAR_SLAVE_ADDR1_REG, (0x34u << 11) | 0x55u);
  flexe_rtc_i2c_result[0] = REG_READ(RTC_I2C_SCL_LOW_PERIOD_REG);
  flexe_rtc_i2c_result[1] = REG_READ(RTC_I2C_SCL_HIGH_PERIOD_REG);
  flexe_rtc_i2c_result[2] = REG_READ(RTC_I2C_TIMEOUT_REG);
  if (flexe_rtc_i2c_result[0] != 40u ||
      flexe_rtc_i2c_result[1] != 40u ||
      flexe_rtc_i2c_result[2] != 200u) {
    fail(1, flexe_rtc_i2c_result[0]);
    return;
  }

  flexe_rtc_i2c_stage = 2;
  if (!rtcStart(rtcControl(true, 0, 0x20, 0xA5, 0, 7))) {
    fail(2, 0);
    return;
  }
  flexe_rtc_i2c_result[3] =
      (REG_READ(SENS_SAR_SLAVE_ADDR4_REG) & SENS_I2C_DONE) != 0;
  flexe_rtc_i2c_result[4] = REG_READ(RTC_I2C_INT_RAW_REG);
  flexe_rtc_i2c_result[5] = REG_READ(RTC_I2C_INT_ST_REG);
  if (flexe_rtc_i2c_result[3] != 1u ||
      flexe_rtc_i2c_result[4] != 0x60u ||
      flexe_rtc_i2c_result[5] != 0xC0u) {
    fail(3, flexe_rtc_i2c_result[4]);
    return;
  }

  if (!rtcStart(rtcControl(false, 0, 0x20, 0, 0, 0))) {
    fail(4, 0);
    return;
  }
  flexe_rtc_i2c_result[6] = REG_READ(RTC_I2C_DATA_REG) & 0xFFu;
  flexe_rtc_i2c_result[7] =
      (REG_READ(SENS_SAR_SLAVE_ADDR4_REG) & SENS_I2C_RDATA_M) >>
      SENS_I2C_RDATA_S;
  if (flexe_rtc_i2c_result[6] != 0xA5u ||
      flexe_rtc_i2c_result[7] != 0xA5u) {
    fail(5, flexe_rtc_i2c_result[6]);
    return;
  }

  // DATA=5 written into bits [4:2] becomes 0x14 on the wire.
  if (!rtcStart(rtcControl(true, 0, 0x21, 5, 2, 4)) ||
      !rtcStart(rtcControl(false, 0, 0x21, 0, 0, 0))) {
    fail(6, 0);
    return;
  }
  flexe_rtc_i2c_result[8] = REG_READ(RTC_I2C_DATA_REG) & 0xFFu;
  REG_WRITE(RTC_I2C_CMD_REG(0), UINT32_MAX);
  flexe_rtc_i2c_result[9] = REG_READ(RTC_I2C_CMD_REG(0));
  if (flexe_rtc_i2c_result[8] != 0x14u ||
      flexe_rtc_i2c_result[9] != 0x80003FFFu) {
    fail(7, flexe_rtc_i2c_result[9]);
    return;
  }

  flexe_rtc_i2c_stage = 3;
  REG_WRITE(RTC_I2C_INT_CLR_REG, 0xC0u);
  if (!rtcStart(rtcControl(false, 1, 0x01, 0, 0, 0))) {
    fail(8, 0);
    return;
  }
  flexe_rtc_i2c_result[10] = REG_READ(RTC_I2C_DEBUG_STATUS_REG) & 1u;
  flexe_rtc_i2c_result[11] = REG_READ(RTC_I2C_DATA_REG) & 0xFFu;
  REG_WRITE(RTC_I2C_INT_CLR_REG, 0xC0u);
  flexe_rtc_i2c_result[12] = REG_READ(RTC_I2C_INT_RAW_REG);
  if (flexe_rtc_i2c_result[10] != 1u ||
      flexe_rtc_i2c_result[11] != 0xFFu ||
      flexe_rtc_i2c_result[12] != 0u) {
    fail(9, flexe_rtc_i2c_result[10]);
    return;
  }

  // A launch outside master mode reaches the controller timeout diagnostic.
  REG_WRITE(RTC_I2C_CTRL_REG, 0u);
  if (!rtcStart(rtcControl(false, 0, 0x20, 0, 0, 0))) {
    fail(10, 0);
    return;
  }
  flexe_rtc_i2c_result[13] = REG_READ(RTC_I2C_INT_RAW_REG);
  flexe_rtc_i2c_result[14] = REG_READ(RTC_I2C_INT_ST_REG);
  flexe_rtc_i2c_result[15] = REG_READ(RTC_I2C_DEBUG_STATUS_REG) & (1u << 2);
  if (flexe_rtc_i2c_result[13] != 0x80u ||
      flexe_rtc_i2c_result[14] != 0x100u ||
      flexe_rtc_i2c_result[15] != 4u) {
    fail(11, flexe_rtc_i2c_result[13]);
    return;
  }

  flexe_rtc_i2c_stage = kSuccessMarker;
}

void loop() {
  delay(1000);
}
