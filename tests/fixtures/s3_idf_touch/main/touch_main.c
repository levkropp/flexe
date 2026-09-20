/* Exercise ESP-IDF 5.3's public ESP32-S3 touch-v2 API through the RTC/SENS
 * register path, RTC-core interrupt, and an ISR-to-task notification. */
#include <stdint.h>
#include <stdio.h>

#include "driver/touch_pad.h"
#include "esp_attr.h"
#include "esp_err.h"
#include "esp_sleep.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define TOUCH_DONE UINT32_C(0x544F5543)
#define TEST_PAD TOUCH_PAD_NUM4
#define TEST_PAD_MASK (UINT32_C(1) << TEST_PAD)

volatile uint32_t flexe_touch_stage;
volatile uint32_t flexe_touch_result[16];

static TaskHandle_t consumer_task;

static void fail(uint32_t stage, int32_t detail)
{
    flexe_touch_result[15] = (uint32_t)detail;
    flexe_touch_stage = UINT32_C(0xBAD00000) | stage;
    printf("TOUCH_FAIL stage=%u detail=%ld\n",
           (unsigned)stage, (long)detail);
    fflush(stdout);
    for (;;) vTaskDelay(pdMS_TO_TICKS(100));
}

static void require_ok(uint32_t stage, esp_err_t error)
{
    if (error != ESP_OK) fail(stage, error);
}

static void IRAM_ATTR touch_isr(void *context)
{
    (void)context;
    uint32_t interrupts = touch_pad_read_intr_status_mask();
    uint32_t status = touch_pad_get_status();
    uint32_t raw = 0u;
    (void)touch_pad_read_raw_data(TEST_PAD, &raw);
    if (interrupts & TOUCH_PAD_INTR_MASK_ACTIVE) {
        flexe_touch_result[0]++;
        flexe_touch_result[2] = interrupts;
        flexe_touch_result[3] = status;
        flexe_touch_result[4] = raw;
    }
    if (interrupts & TOUCH_PAD_INTR_MASK_INACTIVE) {
        flexe_touch_result[1]++;
        flexe_touch_result[5] = interrupts;
        flexe_touch_result[6] = status;
        flexe_touch_result[7] = raw;
    }
    (void)touch_pad_intr_clear(interrupts);
    BaseType_t wake = pdFALSE;
    vTaskNotifyGiveFromISR(consumer_task, &wake);
    if (wake == pdTRUE) portYIELD_FROM_ISR();
}

void app_main(void)
{
    consumer_task = xTaskGetCurrentTaskHandle();
    require_ok(1u, touch_pad_init());
    require_ok(2u, touch_pad_config(TEST_PAD));
    require_ok(3u, touch_pad_set_fsm_mode(TOUCH_FSM_MODE_TIMER));
    require_ok(4u, touch_pad_fsm_start());

    uint32_t raw = 0u;
    uint32_t benchmark = 0u;
    require_ok(5u, touch_pad_read_raw_data(TEST_PAD, &raw));
    require_ok(6u, touch_pad_read_benchmark(TEST_PAD, &benchmark));
    flexe_touch_result[8] = raw;
    flexe_touch_result[9] = benchmark;
    if (raw != 1000u || benchmark != 1000u)
        fail(7u, (int32_t)raw);

    require_ok(8u, touch_pad_set_thresh(TEST_PAD, 200u));
    require_ok(9u, touch_pad_isr_register(
        touch_isr, NULL,
        TOUCH_PAD_INTR_MASK_ACTIVE | TOUCH_PAD_INTR_MASK_INACTIVE));
    require_ok(10u, touch_pad_intr_enable(
        TOUCH_PAD_INTR_MASK_ACTIVE | TOUCH_PAD_INTR_MASK_INACTIVE));

    flexe_touch_stage = 1u;
    if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1000)) == 0u)
        fail(11u, -1);
    if (flexe_touch_result[0] != 1u ||
        flexe_touch_result[3] != TEST_PAD_MASK ||
        flexe_touch_result[4] != 1400u)
        fail(12u, (int32_t)flexe_touch_result[3]);

    flexe_touch_stage = 2u;
    if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1000)) == 0u)
        fail(13u, -1);
    if (flexe_touch_result[1] != 1u ||
        flexe_touch_result[6] != 0u ||
        flexe_touch_result[7] != 1050u)
        fail(14u, (int32_t)flexe_touch_result[6]);

    require_ok(15u, touch_pad_intr_disable(
        TOUCH_PAD_INTR_MASK_ACTIVE | TOUCH_PAD_INTR_MASK_INACTIVE));
    require_ok(16u, touch_pad_set_thresh(TEST_PAD, 1000u));
    require_ok(17u, touch_pad_sleep_channel_enable(TEST_PAD, true));
    require_ok(18u, touch_pad_sleep_set_threshold(TEST_PAD, 200u));
    require_ok(19u, esp_sleep_enable_touchpad_wakeup());
    require_ok(20u, esp_sleep_enable_timer_wakeup(500000u));

    flexe_touch_stage = 3u;
    require_ok(21u, esp_light_sleep_start());
    flexe_touch_result[10] = (uint32_t)esp_sleep_get_wakeup_cause();
    flexe_touch_result[11] =
        (uint32_t)esp_sleep_get_touchpad_wakeup_status();
    if (flexe_touch_result[10] != ESP_SLEEP_WAKEUP_TOUCHPAD ||
        flexe_touch_result[11] != TEST_PAD)
        fail(22u, (int32_t)flexe_touch_result[10]);
    require_ok(23u,
               esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL));

    require_ok(24u, touch_pad_fsm_stop());
    require_ok(25u, touch_pad_deinit());

    flexe_touch_result[13] = touch_pad_get_status();
    flexe_touch_result[15] = 0u;
    flexe_touch_stage = TOUCH_DONE;
    printf("TOUCH_DONE active=%u inactive=%u baseline=%u/%u "
           "active_raw=%u inactive_raw=%u wake=%u pad=%u "
           "teardown_status=%u\n",
           (unsigned)flexe_touch_result[0],
           (unsigned)flexe_touch_result[1],
           (unsigned)flexe_touch_result[8],
           (unsigned)flexe_touch_result[9],
           (unsigned)flexe_touch_result[4],
           (unsigned)flexe_touch_result[7],
           (unsigned)flexe_touch_result[10],
           (unsigned)flexe_touch_result[11],
           (unsigned)flexe_touch_result[13]);
    fflush(stdout);
    for (;;) vTaskDelay(pdMS_TO_TICKS(100));
}
