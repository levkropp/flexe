/* Native ESP-IDF/FreeRTOS inter-core handoff. Neither queue nor notification
 * is intercepted by Flexe's compatibility scheduler when run with -N. */
#include <stdint.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#define ROUNDS 32u

static QueueHandle_t messages;

static void fail(const char *stage, unsigned round)
{
    printf("CROSSCORE_FAIL %s %u\n", stage, round);
    fflush(stdout);
    for (;;) vTaskDelay(pdMS_TO_TICKS(100));
}

static void producer(void *arg)
{
    TaskHandle_t consumer = (TaskHandle_t)arg;
    if (xPortGetCoreID() != 1) fail("producer_core", 0u);
    printf("CROSSCORE_PRODUCER core=%d\n", xPortGetCoreID());
    for (unsigned round = 1u; round <= ROUNDS; round++) {
        uint32_t value = 0xC05E0000u | round;
        if (xQueueSend(messages, &value, pdMS_TO_TICKS(1000)) != pdTRUE)
            fail("send", round);
        if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1000)) != 1u)
            fail("ack", round);
    }
    xTaskNotifyGive(consumer);
    vTaskDelete(NULL);
}

void app_main(void)
{
    if (xPortGetCoreID() != 0) fail("consumer_core", 0u);
    messages = xQueueCreate(1u, sizeof(uint32_t));
    if (!messages) fail("queue_create", 0u);
    TaskHandle_t producer_task = NULL;
    if (xTaskCreatePinnedToCore(producer, "producer", 4096u,
                                xTaskGetCurrentTaskHandle(), 5u,
                                &producer_task, 1) != pdPASS)
        fail("task_create", 0u);

    for (unsigned round = 1u; round <= ROUNDS; round++) {
        uint32_t value = 0u;
        if (xQueueReceive(messages, &value,
                          pdMS_TO_TICKS(1000)) != pdTRUE)
            fail("receive", round);
        if (value != (0xC05E0000u | round)) fail("payload", round);
        xTaskNotifyGive(producer_task);
    }
    if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1000)) != 1u)
        fail("producer_done", ROUNDS);
    printf("CROSSCORE_OK rounds=%u consumer=0 producer=1\n", ROUNDS);
    fflush(stdout);
    for (unsigned tick = 0u;; tick++) {
        printf("CROSSCORE_ALIVE %u\n", tick);
        fflush(stdout);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
