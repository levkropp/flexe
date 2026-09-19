/* Stock ESP-IDF RMT TX/RX drivers on a shared ESP32-S3 GPIO pad. */
#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>

#include "driver/rmt_encoder.h"
#include "driver/rmt_rx.h"
#include "driver/rmt_tx.h"
#include "esp_err.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define LOOPBACK_GPIO 4
#define LOOPBACK_SYMBOLS 4u

static TaskHandle_t receiver_task;
static volatile size_t received_count;
static rmt_symbol_word_t received[32];
static volatile bool sync_tx0_done;
static volatile bool sync_tx1_done;

static bool receive_done(rmt_channel_handle_t channel,
                         const rmt_rx_done_event_data_t *event,
                         void *ctx)
{
    (void)channel;
    (void)ctx;
    BaseType_t woken = pdFALSE;
    received_count = event->num_symbols;
    vTaskNotifyGiveFromISR(receiver_task, &woken);
    return woken == pdTRUE;
}

static bool transmit_done(rmt_channel_handle_t channel,
                          const rmt_tx_done_event_data_t *event,
                          void *ctx)
{
    (void)channel;
    (void)event;
    *(volatile bool *)ctx = true;
    return false;
}

static void fail(const char *stage, esp_err_t error)
{
    printf("RMT_LOOPBACK_FAIL stage=%s error=%d\n", stage, (int)error);
    fflush(stdout);
    for (;;) vTaskDelay(pdMS_TO_TICKS(100));
}

static void check(const char *stage, esp_err_t error)
{
    if (error != ESP_OK) fail(stage, error);
}

static bool near(unsigned observed, unsigned expected)
{
    return observed + 1u >= expected && observed <= expected + 1u;
}

static bool near_carrier(unsigned observed, unsigned expected)
{
    /* RX can measure the envelope boundary only when a carrier edge
     * arrives. Permit one roughly 26-us carrier cycle at 38 kHz. */
    return observed + 30u >= expected && observed <= expected + 30u;
}

void app_main(void)
{
    receiver_task = xTaskGetCurrentTaskHandle();
    rmt_channel_handle_t rx = NULL;
    rmt_channel_handle_t tx = NULL;
    rmt_encoder_handle_t encoder = NULL;

    rmt_rx_channel_config_t rx_config = {
        .gpio_num = LOOPBACK_GPIO,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 1000000,
        .mem_block_symbols = 48,
        .flags.io_loop_back = true,
    };
    check("new-rx", rmt_new_rx_channel(&rx_config, &rx));
    rmt_rx_event_callbacks_t callbacks = {
        .on_recv_done = receive_done,
    };
    check("rx-callback", rmt_rx_register_event_callbacks(
        rx, &callbacks, NULL));

    rmt_tx_channel_config_t tx_config = {
        .gpio_num = LOOPBACK_GPIO,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 1000000,
        .mem_block_symbols = 48,
        .trans_queue_depth = 1,
        .flags.io_loop_back = true,
    };
    check("new-tx", rmt_new_tx_channel(&tx_config, &tx));
    rmt_copy_encoder_config_t encoder_config = {};
    check("copy-encoder", rmt_new_copy_encoder(&encoder_config, &encoder));
    check("enable-tx", rmt_enable(tx));
    check("enable-rx", rmt_enable(rx));

    rmt_receive_config_t receive_config = {
        .signal_range_min_ns = 1000,
        .signal_range_max_ns = 100000,
    };
    check("receive", rmt_receive(rx, received, sizeof(received),
                                  &receive_config));
    const rmt_symbol_word_t frame[LOOPBACK_SYMBOLS] = {
        {.level0 = 1, .duration0 = 10, .level1 = 0, .duration1 = 12},
        {.level0 = 1, .duration0 = 8,  .level1 = 0, .duration1 = 9},
        {.level0 = 1, .duration0 = 6,  .level1 = 0, .duration1 = 7},
        /* The final low joins the low idle level; only prior words have
         * a following edge that makes both half-durations measurable. */
        {.level0 = 1, .duration0 = 5,  .level1 = 0, .duration1 = 6},
    };
    rmt_transmit_config_t transmit_config = {.loop_count = 0};
    check("transmit", rmt_transmit(tx, encoder, frame, sizeof(frame),
                                    &transmit_config));
    check("tx-complete", rmt_tx_wait_all_done(tx, 1000));
    if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1000)) == 0)
        fail("rx-timeout", ESP_ERR_TIMEOUT);
    if (received_count != LOOPBACK_SYMBOLS ||
        received[0].level0 != 1u || received[0].level1 != 0u ||
        received[1].level0 != 1u || received[1].level1 != 0u ||
        received[2].level0 != 1u || received[2].level1 != 0u ||
        !near(received[0].duration0, 10u) ||
        !near(received[0].duration1, 12u) ||
        !near(received[1].duration0, 8u) ||
        !near(received[1].duration1, 9u) ||
        !near(received[2].duration0, 6u) ||
        !near(received[2].duration1, 7u))
        fail("pulse-data", ESP_ERR_INVALID_RESPONSE);

    printf("RMT_LOOPBACK_OK count=%u first=%u,%u second=%u,%u third=%u,%u\n",
           (unsigned)received_count,
           (unsigned)received[0].duration0,
           (unsigned)received[0].duration1,
           (unsigned)received[1].duration0,
           (unsigned)received[1].duration1,
           (unsigned)received[2].duration0,
           (unsigned)received[2].duration1);
    fflush(stdout);

    rmt_carrier_config_t tx_carrier = {
        .frequency_hz = 38000,
        .duty_cycle = 0.5,
    };
    rmt_carrier_config_t rx_carrier = {
        .frequency_hz = 25000,
        .duty_cycle = 0.5,
    };
    check("tx-carrier", rmt_apply_carrier(tx, &tx_carrier));
    check("rx-demod", rmt_apply_carrier(rx, &rx_carrier));
    received_count = 0;
    receive_config.signal_range_max_ns = 500000;
    check("carrier-receive", rmt_receive(rx, received, sizeof(received),
                                          &receive_config));
    const rmt_symbol_word_t carrier_frame[] = {
        {.level0 = 1, .duration0 = 200, .level1 = 0, .duration1 = 250},
        {.level0 = 1, .duration0 = 180, .level1 = 0, .duration1 = 220},
        {.level0 = 1, .duration0 = 160, .level1 = 0, .duration1 = 200},
    };
    check("carrier-transmit", rmt_transmit(tx, encoder, carrier_frame,
                                           sizeof(carrier_frame),
                                           &transmit_config));
    check("carrier-tx-complete", rmt_tx_wait_all_done(tx, 1000));
    if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1000)) == 0)
        fail("carrier-rx-timeout", ESP_ERR_TIMEOUT);
    printf("RMT_CARRIER_OBSERVED count=%u first=%u,%u second=%u,%u\n",
           (unsigned)received_count,
           (unsigned)received[0].duration0,
           (unsigned)received[0].duration1,
           (unsigned)received[1].duration0,
           (unsigned)received[1].duration1);
    fflush(stdout);
    if (received_count < 2u ||
        received[0].level0 != 1u || received[0].level1 != 0u ||
        received[1].level0 != 1u || received[1].level1 != 0u ||
        !near_carrier(received[0].duration0, 200u) ||
        !near_carrier(received[0].duration1, 250u) ||
        !near_carrier(received[1].duration0, 180u) ||
        !near_carrier(received[1].duration1, 220u))
        fail("carrier-pulse-data", ESP_ERR_INVALID_RESPONSE);
    printf("RMT_CARRIER_LOOPBACK_OK count=%u\n", (unsigned)received_count);
    fflush(stdout);

    check("tx-carrier-off", rmt_apply_carrier(tx, NULL));
    check("rx-demod-off", rmt_apply_carrier(rx, NULL));
    received_count = 0;
    check("loop-receive", rmt_receive(rx, received, sizeof(received),
                                       &receive_config));
    const rmt_symbol_word_t loop_frame[] = {
        {.level0 = 1, .duration0 = 10, .level1 = 0, .duration1 = 12},
        {.level0 = 1, .duration0 = 8,  .level1 = 0, .duration1 = 9},
    };
    rmt_transmit_config_t loop_config = {.loop_count = 3};
    check("loop-transmit", rmt_transmit(tx, encoder, loop_frame,
                                        sizeof(loop_frame), &loop_config));
    check("loop-tx-complete", rmt_tx_wait_all_done(tx, 1000));
    if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1000)) == 0)
        fail("loop-rx-timeout", ESP_ERR_TIMEOUT);
    printf("RMT_LOOP_COUNT_OBSERVED count=%u first=%u,%u third=%u,%u fifth=%u,%u\n",
           (unsigned)received_count,
           (unsigned)received[0].duration0,
           (unsigned)received[0].duration1,
           (unsigned)received[2].duration0,
           (unsigned)received[2].duration1,
           (unsigned)received[4].duration0,
           (unsigned)received[4].duration1);
    fflush(stdout);
    if (received_count != 6u ||
        !near(received[0].duration0, 10u) ||
        !near(received[0].duration1, 12u) ||
        !near(received[1].duration0, 8u) ||
        !near(received[1].duration1, 9u) ||
        !near(received[2].duration0, 10u) ||
        !near(received[2].duration1, 12u) ||
        !near(received[3].duration0, 8u) ||
        !near(received[3].duration1, 9u) ||
        !near(received[4].duration0, 10u) ||
        !near(received[4].duration1, 12u) ||
        /* The final low joins idle, so RX has no edge to measure its
         * programmed duration, just as in the plain-pulse frame. */
        !near(received[5].duration0, 8u))
        fail("loop-pulse-data", ESP_ERR_INVALID_RESPONSE);
    printf("RMT_LOOP_COUNT_OK count=%u\n", (unsigned)received_count);
    fflush(stdout);

    received_count = 0;
    check("infinite-receive", rmt_receive(rx, received, sizeof(received),
                                           &receive_config));
    const rmt_symbol_word_t infinite_frame[] = {
        {.level0 = 1, .duration0 = 100, .level1 = 0, .duration1 = 100},
    };
    rmt_transmit_config_t infinite_config = {.loop_count = -1};
    check("infinite-transmit", rmt_transmit(tx, encoder, infinite_frame,
                                             sizeof(infinite_frame),
                                             &infinite_config));
    esp_rom_delay_us(900);
    check("infinite-stop", rmt_disable(tx));
    if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1000)) == 0)
        fail("infinite-rx-timeout", ESP_ERR_TIMEOUT);
    if (received_count < 4u || received_count > 8u)
        fail("infinite-count", ESP_ERR_INVALID_RESPONSE);
    /* TX_STOP can truncate the final symbol, but every completed preceding
     * iteration must retain the programmed widths. */
    for (size_t i = 0; i + 1u < received_count; i++) {
        if (!near(received[i].duration0, 100u) ||
            !near(received[i].duration1, 100u))
            fail("infinite-pulse-data", ESP_ERR_INVALID_RESPONSE);
    }
    printf("RMT_INFINITE_LOOP_OK count=%u\n", (unsigned)received_count);
    fflush(stdout);
    check("re-enable-tx", rmt_enable(tx));
    check("disable-rx", rmt_disable(rx));
    /* The driver splits counts above the 10-bit hardware limit into
     * multiple loop-interrupt batches. The second batch must finish too. */
    rmt_transmit_config_t batch_config = {.loop_count = 1024};
    check("batch-transmit", rmt_transmit(tx, encoder, loop_frame,
                                         sizeof(loop_frame), &batch_config));
    check("batch-tx-complete", rmt_tx_wait_all_done(tx, 1000));
    printf("RMT_LOOP_BATCH_OK count=1024\n");
    fflush(stdout);

    rmt_channel_handle_t sync_tx = NULL;
    rmt_encoder_handle_t sync_encoder = NULL;
    rmt_tx_channel_config_t sync_tx_config = tx_config;
    sync_tx_config.gpio_num = LOOPBACK_GPIO + 1;
    check("new-sync-tx", rmt_new_tx_channel(&sync_tx_config, &sync_tx));
    check("new-sync-encoder", rmt_new_copy_encoder(
        &encoder_config, &sync_encoder));
    rmt_tx_event_callbacks_t tx_callbacks = {
        .on_trans_done = transmit_done,
    };
    check("tx0-callback", rmt_tx_register_event_callbacks(
        tx, &tx_callbacks, (void *)&sync_tx0_done));
    check("tx1-callback", rmt_tx_register_event_callbacks(
        sync_tx, &tx_callbacks, (void *)&sync_tx1_done));
    check("enable-sync-tx", rmt_enable(sync_tx));
    rmt_channel_handle_t sync_channels[] = {tx, sync_tx};
    rmt_sync_manager_config_t sync_config = {
        .tx_channel_array = sync_channels,
        .array_size = 2,
    };
    rmt_sync_manager_handle_t sync = NULL;
    check("new-sync-manager", rmt_new_sync_manager(&sync_config, &sync));
    check("sync-reset", rmt_sync_reset(sync));
    sync_tx0_done = false;
    sync_tx1_done = false;
    check("sync-transmit-0", rmt_transmit(tx, encoder, loop_frame,
                                           sizeof(loop_frame),
                                           &transmit_config));
    esp_rom_delay_us(500);
    if (sync_tx0_done)
        fail("sync-started-early", ESP_ERR_INVALID_STATE);
    check("sync-transmit-1", rmt_transmit(sync_tx, sync_encoder, loop_frame,
                                           sizeof(loop_frame),
                                           &transmit_config));
    check("sync-tx0-complete", rmt_tx_wait_all_done(tx, 1000));
    check("sync-tx1-complete", rmt_tx_wait_all_done(sync_tx, 1000));
    if (!sync_tx0_done || !sync_tx1_done)
        fail("sync-callbacks", ESP_ERR_INVALID_RESPONSE);
    printf("RMT_SYNC_START_OK channels=2\n");
    fflush(stdout);
    check("delete-sync-manager", rmt_del_sync_manager(sync));
    check("disable-sync-tx", rmt_disable(sync_tx));
    check("disable-tx", rmt_disable(tx));
    check("delete-sync-tx", rmt_del_channel(sync_tx));
    check("delete-tx", rmt_del_channel(tx));
    check("delete-rx", rmt_del_channel(rx));
    check("delete-sync-encoder", rmt_del_encoder(sync_encoder));
    check("delete-encoder", rmt_del_encoder(encoder));
    for (unsigned beat = 1u; beat <= 10u; beat++) {
        vTaskDelay(pdMS_TO_TICKS(100));
        printf("RMT_LOOPBACK_ALIVE %u\n", beat);
        fflush(stdout);
    }
    for (;;) vTaskDelay(pdMS_TO_TICKS(100));
}
