/* Exercise ESP-IDF 5.3's public continuous-ADC API through APB_SARADC,
 * trigger-8 GDMA, the driver's ISR callback, and its FreeRTOS ring buffer. */
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "esp_adc/adc_continuous.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define ADC_CONTINUOUS_DONE UINT32_C(0x41444344)
#define FRAME_BYTES 64u
#define FRAME_SAMPLES (FRAME_BYTES / SOC_ADC_DIGI_RESULT_BYTES)

volatile uint32_t flexe_adc_continuous_stage;
volatile uint32_t flexe_adc_continuous_result[10];

static TaskHandle_t consumer_task;

static void fail(uint32_t stage, int32_t detail)
{
    flexe_adc_continuous_result[9] = (uint32_t)detail;
    flexe_adc_continuous_stage = UINT32_C(0xBAD00000) | stage;
    printf("ADC_CONTINUOUS_FAIL stage=%u detail=%ld\n",
           (unsigned)stage, (long)detail);
    fflush(stdout);
    for (;;) vTaskDelay(pdMS_TO_TICKS(100));
}

static void require_ok(uint32_t stage, esp_err_t error)
{
    if (error != ESP_OK) fail(stage, error);
}

static bool IRAM_ATTR on_conversion(
    adc_continuous_handle_t handle,
    const adc_continuous_evt_data_t *event, void *context)
{
    (void)handle;
    (void)event;
    (void)context;
    flexe_adc_continuous_result[0]++;
    BaseType_t wake = pdFALSE;
    vTaskNotifyGiveFromISR(consumer_task, &wake);
    return wake == pdTRUE;
}

static uint32_t consume_frame(adc_continuous_handle_t handle,
                              uint32_t stage, uint16_t channel2,
                              uint16_t channel3)
{
    uint8_t bytes[FRAME_BYTES];
    uint32_t received = 0u;
    if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1000)) == 0u)
        fail(stage, -1);
    require_ok(stage, adc_continuous_read(
        handle, bytes, sizeof(bytes), &received, 0u));
    if (received != sizeof(bytes)) fail(stage, (int32_t)received);

    uint32_t hash = UINT32_C(2166136261);
    for (unsigned index = 0u; index < FRAME_SAMPLES; index++) {
        const adc_digi_output_data_t *sample =
            (const adc_digi_output_data_t *)&bytes[
                index * SOC_ADC_DIGI_RESULT_BYTES];
        uint32_t expected_channel = (index & 1u) ? 3u : 2u;
        uint32_t expected_value = (index & 1u) ? channel3 : channel2;
        if (sample->type2.unit != 0u ||
            sample->type2.channel != expected_channel ||
            sample->type2.data != expected_value)
            fail(stage, (int32_t)sample->val);
        hash = (hash ^ sample->val) * UINT32_C(16777619);
    }
    return hash;
}

void app_main(void)
{
    consumer_task = xTaskGetCurrentTaskHandle();
    adc_continuous_handle_t handle = NULL;
    adc_continuous_handle_cfg_t handle_config = {
        .max_store_buf_size = 512u,
        .conv_frame_size = FRAME_BYTES,
    };
    require_ok(1u, adc_continuous_new_handle(&handle_config, &handle));

    adc_digi_pattern_config_t pattern[] = {
        {
            .atten = ADC_ATTEN_DB_0,
            .channel = ADC_CHANNEL_2,
            .unit = ADC_UNIT_1,
            .bit_width = SOC_ADC_DIGI_MAX_BITWIDTH,
        },
        {
            .atten = ADC_ATTEN_DB_0,
            .channel = ADC_CHANNEL_3,
            .unit = ADC_UNIT_1,
            .bit_width = SOC_ADC_DIGI_MAX_BITWIDTH,
        },
    };
    adc_continuous_config_t conversion_config = {
        .pattern_num = sizeof(pattern) / sizeof(pattern[0]),
        .adc_pattern = pattern,
        .sample_freq_hz = 20000u,
        .conv_mode = ADC_CONV_SINGLE_UNIT_1,
        .format = ADC_DIGI_OUTPUT_FORMAT_TYPE2,
    };
    require_ok(2u, adc_continuous_config(handle, &conversion_config));
    adc_continuous_evt_cbs_t callbacks = {
        .on_conv_done = on_conversion,
    };
    require_ok(3u, adc_continuous_register_event_callbacks(
        handle, &callbacks, NULL));

    flexe_adc_continuous_stage = 1u;
    require_ok(4u, adc_continuous_start(handle));
    flexe_adc_continuous_stage = 2u;
    flexe_adc_continuous_result[1] = consume_frame(
        handle, 5u, UINT16_C(0x123), UINT16_C(0xABC));

    flexe_adc_continuous_stage = 3u;
    flexe_adc_continuous_result[2] = consume_frame(
        handle, 6u, UINT16_C(0x456), UINT16_C(0x789));
    flexe_adc_continuous_stage = 4u;

    require_ok(7u, adc_continuous_stop(handle));
    require_ok(8u, adc_continuous_deinit(handle));
    if (flexe_adc_continuous_result[0] != 2u)
        fail(9u, (int32_t)flexe_adc_continuous_result[0]);

    flexe_adc_continuous_result[9] = 0u;
    flexe_adc_continuous_stage = ADC_CONTINUOUS_DONE;
    printf("ADC_CONTINUOUS_DONE callbacks=%u hashes=%08X,%08X teardown=ok\n",
           (unsigned)flexe_adc_continuous_result[0],
           (unsigned)flexe_adc_continuous_result[1],
           (unsigned)flexe_adc_continuous_result[2]);
    fflush(stdout);
    for (;;) vTaskDelay(pdMS_TO_TICKS(100));
}
