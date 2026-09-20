/* Exercise ESP-IDF 5.3's stock TWAI driver, ISR, TX/RX queues, alerts, and
 * GPIO matrix against Flexe's target-described controller. */
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "driver/twai.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define TWAI_DONE UINT32_C(0x54574149)
#define REQUIRED_ALERTS \
    (TWAI_ALERT_TX_IDLE | TWAI_ALERT_TX_SUCCESS | TWAI_ALERT_RX_DATA)

volatile uint32_t flexe_twai_stage;
volatile uint32_t flexe_twai_command;
volatile uint32_t flexe_twai_result[32];

static void fail(uint32_t stage, uint32_t detail)
{
    flexe_twai_result[31] = detail;
    flexe_twai_stage = UINT32_C(0xBAD00000) | stage;
    printf("TWAI_FAIL stage=%u detail=0x%08X\n",
           (unsigned)stage, (unsigned)detail);
    fflush(stdout);
    for (;;) vTaskDelay(pdMS_TO_TICKS(100));
}

static bool wait_for_command(uint32_t command)
{
    for (unsigned waited = 0u;
         waited < 10000u && flexe_twai_command != command; waited++)
        vTaskDelay(pdMS_TO_TICKS(1));
    return flexe_twai_command == command;
}

static uint32_t pack_flags_and_dlc(const twai_message_t *message)
{
    return (message->flags & 0xFFFFu) |
           ((uint32_t)message->data_length_code << 16u);
}

static uint32_t pack4(const uint8_t *data)
{
    return (uint32_t)data[0] | ((uint32_t)data[1] << 8u) |
           ((uint32_t)data[2] << 16u) | ((uint32_t)data[3] << 24u);
}

void app_main(void)
{
    flexe_twai_stage = 1u;
    twai_general_config_t general =
        TWAI_GENERAL_CONFIG_DEFAULT(GPIO_NUM_5, GPIO_NUM_4,
                                    TWAI_MODE_NORMAL);
    general.tx_queue_len = 4u;
    general.rx_queue_len = 8u;
    general.alerts_enabled = REQUIRED_ALERTS;
    twai_timing_config_t timing = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t filter = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    esp_err_t error = twai_driver_install(&general, &timing, &filter);
    flexe_twai_result[0] = (uint32_t)error;
    if (error != ESP_OK) fail(1u, (uint32_t)error);
    error = twai_start();
    flexe_twai_result[1] = (uint32_t)error;
    if (error != ESP_OK) fail(2u, (uint32_t)error);

    flexe_twai_stage = 2u;
    twai_message_t standard = {0};
    standard.identifier = 0x321u;
    standard.data_length_code = 8u;
    for (unsigned index = 0u; index < 8u; index++)
        standard.data[index] = (uint8_t)(0x10u + index);
    error = twai_transmit(&standard, pdMS_TO_TICKS(100));
    flexe_twai_result[2] = (uint32_t)error;
    if (error != ESP_OK) fail(3u, (uint32_t)error);

    twai_message_t extended = {0};
    extended.extd = 1u;
    extended.identifier = 0x01ABCDE3u;
    extended.data_length_code = 4u;
    extended.data[0] = 0xA5u;
    extended.data[1] = 0x5Au;
    extended.data[2] = 0xC3u;
    extended.data[3] = 0x3Cu;
    error = twai_transmit(&extended, pdMS_TO_TICKS(100));
    flexe_twai_result[3] = (uint32_t)error;
    if (error != ESP_OK) fail(4u, (uint32_t)error);

    twai_status_info_t status = {0};
    esp_err_t status_error = ESP_FAIL;
    for (unsigned waited = 0u; waited < 2000u; waited++) {
        status_error = twai_get_status_info(&status);
        if (status_error == ESP_OK && status.msgs_to_tx == 0u) break;
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    flexe_twai_result[4] = (uint32_t)status_error;
    flexe_twai_result[5] = status.msgs_to_tx;
    flexe_twai_result[6] = status.tx_failed_count;
    if (status_error != ESP_OK || status.state != TWAI_STATE_RUNNING ||
        status.msgs_to_tx != 0u || status.tx_failed_count != 0u)
        fail(5u, ((uint32_t)status.state << 24u) | status.msgs_to_tx);

    twai_message_t self = {0};
    self.ss = 1u;
    self.self = 1u;
    self.identifier = 0x5AAu;
    self.data_length_code = 3u;
    self.data[0] = 0xDEu;
    self.data[1] = 0xADu;
    self.data[2] = 0x42u;
    error = twai_transmit(&self, pdMS_TO_TICKS(100));
    flexe_twai_result[7] = (uint32_t)error;
    if (error != ESP_OK) fail(6u, (uint32_t)error);

    twai_message_t received = {0};
    error = twai_receive(&received, pdMS_TO_TICKS(1000));
    flexe_twai_result[8] = (uint32_t)error;
    flexe_twai_result[9] = received.identifier;
    flexe_twai_result[10] = pack_flags_and_dlc(&received);
    flexe_twai_result[11] = pack4(received.data);
    if (error != ESP_OK || received.identifier != 0x5AAu ||
        received.extd || received.rtr || received.data_length_code != 3u ||
        received.data[0] != 0xDEu || received.data[1] != 0xADu ||
        received.data[2] != 0x42u)
        fail(7u, received.identifier);

    uint32_t alerts = 0u;
    error = twai_read_alerts(&alerts, 0u);
    flexe_twai_result[12] = (uint32_t)error;
    flexe_twai_result[13] = alerts;
    if (error != ESP_OK || (alerts & REQUIRED_ALERTS) != REQUIRED_ALERTS)
        fail(8u, alerts);

    flexe_twai_stage = 3u;
    if (!wait_for_command(1u)) fail(9u, flexe_twai_command);

    received = (twai_message_t){0};
    error = twai_receive(&received, pdMS_TO_TICKS(1000));
    flexe_twai_result[14] = (uint32_t)error;
    flexe_twai_result[15] = received.identifier;
    flexe_twai_result[16] = pack_flags_and_dlc(&received);
    flexe_twai_result[17] = pack4(received.data);
    flexe_twai_result[18] =
        (uint32_t)received.data[4] | ((uint32_t)received.data[5] << 8u) |
        ((uint32_t)received.data[6] << 16u) |
        ((uint32_t)received.data[7] << 24u);
    if (error != ESP_OK || received.identifier != 0x456u ||
        received.extd || received.rtr || received.data_length_code != 8u ||
        pack4(received.data) != 0x23222120u ||
        flexe_twai_result[18] != 0x27262524u)
        fail(10u, received.identifier);

    received = (twai_message_t){0};
    error = twai_receive(&received, pdMS_TO_TICKS(1000));
    flexe_twai_result[19] = (uint32_t)error;
    flexe_twai_result[20] = received.identifier;
    flexe_twai_result[21] = pack_flags_and_dlc(&received);
    if (error != ESP_OK || received.identifier != 0x0155AA55u ||
        !received.extd || !received.rtr || received.data_length_code != 6u)
        fail(11u, received.identifier);

    alerts = 0u;
    error = twai_read_alerts(&alerts, 0u);
    flexe_twai_result[22] = (uint32_t)error;
    flexe_twai_result[23] = alerts;
    if (error != ESP_OK || !(alerts & TWAI_ALERT_RX_DATA))
        fail(12u, alerts);

    status = (twai_status_info_t){0};
    error = twai_get_status_info(&status);
    flexe_twai_result[24] = (uint32_t)error;
    flexe_twai_result[25] = status.msgs_to_tx | (status.msgs_to_rx << 16u);
    flexe_twai_result[26] = status.tx_error_counter |
                            (status.rx_error_counter << 16u);
    flexe_twai_result[27] = status.tx_failed_count;
    flexe_twai_result[28] = status.rx_missed_count |
                            (status.rx_overrun_count << 16u);
    if (error != ESP_OK || status.state != TWAI_STATE_RUNNING ||
        status.msgs_to_tx != 0u || status.msgs_to_rx != 0u ||
        status.tx_error_counter != 0u || status.rx_error_counter != 0u ||
        status.tx_failed_count != 0u || status.rx_missed_count != 0u ||
        status.rx_overrun_count != 0u)
        fail(13u, flexe_twai_result[25]);

    error = twai_stop();
    flexe_twai_result[29] = (uint32_t)error;
    if (error != ESP_OK) fail(14u, (uint32_t)error);
    error = twai_driver_uninstall();
    flexe_twai_result[30] = (uint32_t)error;
    if (error != ESP_OK) fail(15u, (uint32_t)error);

    flexe_twai_result[31] = 0u;
    flexe_twai_stage = TWAI_DONE;
    printf("TWAI_DONE\n");
    fflush(stdout);
    for (;;) vTaskDelay(pdMS_TO_TICKS(100));
}
