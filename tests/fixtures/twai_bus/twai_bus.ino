#include <Arduino.h>
#include <driver/twai.h>

volatile uint32_t flexe_twai_stage = 0;
volatile uint32_t flexe_twai_command = 0;
volatile uint32_t flexe_twai_result[32] = {};

static constexpr uint32_t kSuccessMarker = 0x54574149u;  // "TWAI"
static constexpr uint32_t kRequiredAlerts =
    TWAI_ALERT_TX_IDLE | TWAI_ALERT_TX_SUCCESS | TWAI_ALERT_RX_DATA;

static void fail(uint32_t stage, uint32_t detail) {
  flexe_twai_result[31] = detail;
  flexe_twai_stage = 0xBAD00000u | stage;
}

static bool waitForCommand(uint32_t command) {
  for (unsigned waited = 0;
       waited < 10000 && flexe_twai_command != command; ++waited) {
    delay(1);
  }
  return flexe_twai_command == command;
}

static uint32_t packFlagsAndDlc(const twai_message_t &message) {
  return (message.flags & 0xFFFFu) |
         ((uint32_t)message.data_length_code << 16);
}

static uint32_t pack4(const uint8_t *data) {
  return (uint32_t)data[0] | ((uint32_t)data[1] << 8) |
         ((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 24);
}

void setup() {
  flexe_twai_stage = 1;

  twai_general_config_t general =
      TWAI_GENERAL_CONFIG_DEFAULT(GPIO_NUM_27, GPIO_NUM_26,
                                  TWAI_MODE_NORMAL);
  general.tx_queue_len = 4;
  general.rx_queue_len = 8;
  general.alerts_enabled = kRequiredAlerts;
  twai_timing_config_t timing = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t filter = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  esp_err_t err = twai_driver_install(&general, &timing, &filter);
  flexe_twai_result[0] = (uint32_t)err;
  if (err != ESP_OK) {
    fail(1, (uint32_t)err);
    return;
  }

  err = twai_start();
  flexe_twai_result[1] = (uint32_t)err;
  if (err != ESP_OK) {
    fail(2, (uint32_t)err);
    return;
  }

  /* The second call must enter the genuine driver's software TX queue. The
     hardware TX-complete interrupt then advances that queue. */
  flexe_twai_stage = 2;
  twai_message_t standard = {};
  standard.identifier = 0x321u;
  standard.data_length_code = 8;
  for (unsigned i = 0; i < 8; ++i)
    standard.data[i] = (uint8_t)(0x10u + i);
  err = twai_transmit(&standard, pdMS_TO_TICKS(100));
  flexe_twai_result[2] = (uint32_t)err;
  if (err != ESP_OK) {
    fail(3, (uint32_t)err);
    return;
  }

  twai_message_t extended = {};
  extended.extd = 1;
  extended.identifier = 0x01ABCDE3u;
  extended.data_length_code = 4;
  extended.data[0] = 0xA5;
  extended.data[1] = 0x5A;
  extended.data[2] = 0xC3;
  extended.data[3] = 0x3C;
  err = twai_transmit(&extended, pdMS_TO_TICKS(100));
  flexe_twai_result[3] = (uint32_t)err;
  if (err != ESP_OK) {
    fail(4, (uint32_t)err);
    return;
  }

  twai_status_info_t status = {};
  esp_err_t status_err = ESP_FAIL;
  for (unsigned waited = 0; waited < 2000; ++waited) {
    status_err = twai_get_status_info(&status);
    if (status_err == ESP_OK && status.msgs_to_tx == 0)
      break;
    delay(1);
  }
  flexe_twai_result[4] = (uint32_t)status_err;
  flexe_twai_result[5] = status.msgs_to_tx;
  flexe_twai_result[6] = status.tx_failed_count;
  if (status_err != ESP_OK || status.state != TWAI_STATE_RUNNING ||
      status.msgs_to_tx != 0 || status.tx_failed_count != 0) {
    fail(5, ((uint32_t)status.state << 24) | status.msgs_to_tx);
    return;
  }

  /* Exercise the SJA1000 self-reception/single-shot command and receive the
     result through the stock ISR and RX queue, not by reading MMIO here. */
  twai_message_t self = {};
  self.ss = 1;
  self.self = 1;
  self.identifier = 0x5AAu;
  self.data_length_code = 3;
  self.data[0] = 0xDE;
  self.data[1] = 0xAD;
  self.data[2] = 0x42;
  err = twai_transmit(&self, pdMS_TO_TICKS(100));
  flexe_twai_result[7] = (uint32_t)err;
  if (err != ESP_OK) {
    fail(6, (uint32_t)err);
    return;
  }

  twai_message_t received = {};
  err = twai_receive(&received, pdMS_TO_TICKS(1000));
  flexe_twai_result[8] = (uint32_t)err;
  flexe_twai_result[9] = received.identifier;
  flexe_twai_result[10] = packFlagsAndDlc(received);
  flexe_twai_result[11] = pack4(received.data);
  if (err != ESP_OK || received.identifier != 0x5AAu || received.extd ||
      received.rtr || received.data_length_code != 3 ||
      received.data[0] != 0xDE || received.data[1] != 0xAD ||
      received.data[2] != 0x42) {
    fail(7, received.identifier);
    return;
  }

  uint32_t alerts = 0;
  err = twai_read_alerts(&alerts, 0);
  flexe_twai_result[12] = (uint32_t)err;
  flexe_twai_result[13] = alerts;
  if (err != ESP_OK || (alerts & kRequiredAlerts) != kRequiredAlerts) {
    fail(8, alerts);
    return;
  }

  /* The host now injects two frames into the physical RX FIFO before waking
     us. One interrupt must drain both frames into the driver's RX queue. */
  flexe_twai_stage = 3;
  if (!waitForCommand(1)) {
    fail(9, flexe_twai_command);
    return;
  }

  received = {};
  err = twai_receive(&received, pdMS_TO_TICKS(1000));
  flexe_twai_result[14] = (uint32_t)err;
  flexe_twai_result[15] = received.identifier;
  flexe_twai_result[16] = packFlagsAndDlc(received);
  flexe_twai_result[17] = pack4(received.data);
  flexe_twai_result[18] =
      (uint32_t)received.data[4] | ((uint32_t)received.data[5] << 8) |
      ((uint32_t)received.data[6] << 16) |
      ((uint32_t)received.data[7] << 24);
  if (err != ESP_OK || received.identifier != 0x456u || received.extd ||
      received.rtr || received.data_length_code != 8 ||
      pack4(received.data) != 0x23222120u ||
      flexe_twai_result[18] != 0x27262524u) {
    fail(10, received.identifier);
    return;
  }

  received = {};
  err = twai_receive(&received, pdMS_TO_TICKS(1000));
  flexe_twai_result[19] = (uint32_t)err;
  flexe_twai_result[20] = received.identifier;
  flexe_twai_result[21] = packFlagsAndDlc(received);
  if (err != ESP_OK || received.identifier != 0x0155AA55u ||
      !received.extd || !received.rtr || received.data_length_code != 6) {
    fail(11, received.identifier);
    return;
  }

  alerts = 0;
  err = twai_read_alerts(&alerts, 0);
  flexe_twai_result[22] = (uint32_t)err;
  flexe_twai_result[23] = alerts;
  if (err != ESP_OK || !(alerts & TWAI_ALERT_RX_DATA)) {
    fail(12, alerts);
    return;
  }

  status = {};
  err = twai_get_status_info(&status);
  flexe_twai_result[24] = (uint32_t)err;
  flexe_twai_result[25] = status.msgs_to_tx | (status.msgs_to_rx << 16);
  flexe_twai_result[26] = status.tx_error_counter |
                          (status.rx_error_counter << 16);
  flexe_twai_result[27] = status.tx_failed_count;
  flexe_twai_result[28] = status.rx_missed_count |
                          (status.rx_overrun_count << 16);
  if (err != ESP_OK || status.state != TWAI_STATE_RUNNING ||
      status.msgs_to_tx != 0 || status.msgs_to_rx != 0 ||
      status.tx_error_counter != 0 || status.rx_error_counter != 0 ||
      status.tx_failed_count != 0 || status.rx_missed_count != 0 ||
      status.rx_overrun_count != 0) {
    fail(13, flexe_twai_result[25]);
    return;
  }

  err = twai_stop();
  flexe_twai_result[29] = (uint32_t)err;
  if (err != ESP_OK) {
    fail(14, (uint32_t)err);
    return;
  }
  err = twai_driver_uninstall();
  flexe_twai_result[30] = (uint32_t)err;
  if (err != ESP_OK) {
    fail(15, (uint32_t)err);
    return;
  }

  flexe_twai_result[31] = 0;
  flexe_twai_stage = kSuccessMarker;
}

void loop() {
  delay(1000);
}
