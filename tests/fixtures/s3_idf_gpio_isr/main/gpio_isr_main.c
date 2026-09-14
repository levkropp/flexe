/* Drive an unmodified ESP-IDF GPIO per-pin ISR service from host GPIO4
 * edges, including FreeRTOS notification delivery back to app_main. */
#include <stdint.h>
#include <stdio.h>

#include "driver/gpio.h"
#include "esp_attr.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

volatile uint32_t flexe_gpio_edge_count;
static TaskHandle_t app_task;

static void fail(const char *stage, int value)
{
    printf("GPIO_FAIL %s %d\n", stage, value);
    fflush(stdout);
    for (;;) vTaskDelay(pdMS_TO_TICKS(100));
}

static void IRAM_ATTR gpio4_rising(void *arg)
{
    (void)arg;
    flexe_gpio_edge_count++;
    BaseType_t woken = pdFALSE;
    vTaskNotifyGiveFromISR(app_task, &woken);
    if (woken) portYIELD_FROM_ISR();
}

void app_main(void)
{
    app_task = xTaskGetCurrentTaskHandle();
    gpio_config_t config = {
        .pin_bit_mask = UINT64_C(1) << GPIO_NUM_4,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_POSEDGE,
    };
    esp_err_t err = gpio_config(&config);
    if (err != ESP_OK) fail("config", err);
    err = gpio_install_isr_service(0);
    if (err != ESP_OK) fail("install", err);
    err = gpio_isr_handler_add(GPIO_NUM_4, gpio4_rising, NULL);
    if (err != ESP_OK) fail("handler", err);
    gpio_config_t open_drain = {
        .pin_bit_mask = UINT64_C(1) << GPIO_NUM_5,
        .mode = GPIO_MODE_OUTPUT_OD,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    err = gpio_config(&open_drain);
    if (err != ESP_OK) fail("open-drain config", err);
    err = gpio_set_level(GPIO_NUM_5, 1);
    if (err != ESP_OK) fail("open-drain release", err);

    printf("GPIO_ISR_READY\n");
    fflush(stdout);
    for (unsigned round = 1u; round <= 2u; round++) {
        uint32_t notified = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(5000));
        if (notified != 1u) fail("notify", (int)notified);
        uint32_t count = flexe_gpio_edge_count;
        int level = gpio_get_level(GPIO_NUM_4);
        if (count != round || level != 1)
            fail("edge", (int)count);
        printf("GPIO_ISR_EDGE round=%u count=%u level=%d\n",
               round, (unsigned)count, level);
        fflush(stdout);
        err = gpio_set_level(GPIO_NUM_5, round == 1u ? 0 : 1);
        if (err != ESP_OK) fail("open-drain level", err);
        printf("GPIO_OD_%s\n", round == 1u ? "LOW" : "RELEASED");
        fflush(stdout);
    }
    printf("GPIO_ISR_DONE\n");
    fflush(stdout);
    for (;;) vTaskDelay(pdMS_TO_TICKS(100));
}
