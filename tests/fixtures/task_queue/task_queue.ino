/* Task-to-task queues and semaphores: the producer/consumer handoff.
 *
 * Every queue gate in the tree so far has a *peripheral* fill the queue from
 * an ISR, which Flexe services through a dedicated path. Nothing covers the
 * plain case of one guest task sending to another and the receiver waking on
 * it -- which is the single most common thing FreeRTOS is used for.
 *
 * The receive is checked three ways, because the failure modes differ: with a
 * finite timeout, with portMAX_DELAY, and with a timeout that genuinely
 * expires. Each checks the returned status, the payload and the elapsed time,
 * since a receive that returns the right status after sleeping through its
 * whole timeout is still wrong.
 */
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>

#define SUCCESS_MARKER 0x9EE00C0Bu
#define FAIL_BASE      0xBAD00000u

#define SEND_DELAY_MS  50u
#define LONG_TIMEOUT_MS 500u
#define SHORT_TIMEOUT_MS 100u

#define VAL_A 0xC0DE0001u
#define VAL_B 0xC0DE0002u

volatile uint32_t flexe_q_stage = 0;
volatile uint32_t flexe_q_result[16];

static QueueHandle_t q;
static SemaphoreHandle_t sem;

static void fail(uint32_t code) { flexe_q_stage = FAIL_BASE | code; }

/* Sends two items and gives the semaphore, each after a delay, so every wait
 * on the other side has to block first and then be woken by this task. */
static void producer_task(void *arg) {
    vTaskDelay(pdMS_TO_TICKS(SEND_DELAY_MS));
    uint32_t v = VAL_A;
    xQueueSend(q, &v, 0);

    vTaskDelay(pdMS_TO_TICKS(SEND_DELAY_MS));
    v = VAL_B;
    xQueueSend(q, &v, 0);

    vTaskDelay(pdMS_TO_TICKS(SEND_DELAY_MS));
    xSemaphoreGive(sem);
    vTaskDelete(NULL);
}

void setup() {
    q = xQueueCreate(4, sizeof(uint32_t));
    if (!q) { fail(1); return; }
    sem = xSemaphoreCreateBinary();
    if (!sem) { fail(2); return; }
    flexe_q_stage = 1;

    if (xTaskCreate(producer_task, "producer", 4096, NULL, 5, NULL) != pdPASS)
        { fail(3); return; }

    /* (a) Finite timeout, item arrives partway through. Must return pdTRUE
     * with the payload, at the time the producer sent it -- not at the
     * timeout, and not immediately. */
    uint32_t got = 0;
    int64_t t0 = esp_timer_get_time();
    BaseType_t ok = xQueueReceive(q, &got, pdMS_TO_TICKS(LONG_TIMEOUT_MS));
    flexe_q_result[0] = (uint32_t)ok;
    flexe_q_result[1] = got;
    flexe_q_result[2] = (uint32_t)(esp_timer_get_time() - t0);
    if (ok != pdTRUE || got != VAL_A) { fail(4); return; }
    flexe_q_stage = 2;

    /* (b) portMAX_DELAY, same handoff. */
    got = 0;
    t0 = esp_timer_get_time();
    ok = xQueueReceive(q, &got, portMAX_DELAY);
    flexe_q_result[3] = (uint32_t)ok;
    flexe_q_result[4] = got;
    flexe_q_result[5] = (uint32_t)(esp_timer_get_time() - t0);
    if (ok != pdTRUE || got != VAL_B) { fail(5); return; }
    flexe_q_stage = 3;

    /* (c) A semaphore given by another task, with a finite timeout. */
    t0 = esp_timer_get_time();
    ok = xSemaphoreTake(sem, pdMS_TO_TICKS(LONG_TIMEOUT_MS));
    flexe_q_result[6] = (uint32_t)ok;
    flexe_q_result[7] = (uint32_t)(esp_timer_get_time() - t0);
    if (ok != pdTRUE) { fail(6); return; }
    flexe_q_stage = 4;

    /* (d) Nothing is coming: the receive must block for its whole timeout and
     * then report failure, leaving the output buffer alone. */
    got = 0xFFFFFFFFu;
    t0 = esp_timer_get_time();
    ok = xQueueReceive(q, &got, pdMS_TO_TICKS(SHORT_TIMEOUT_MS));
    flexe_q_result[8] = (uint32_t)ok;
    flexe_q_result[9] = got;
    flexe_q_result[10] = (uint32_t)(esp_timer_get_time() - t0);
    if (ok != pdFALSE) { fail(7); return; }

    /* (e) An item already queued returns at once, and the queue count
     * tracks it. */
    uint32_t v = 0xABCD1234u;
    if (xQueueSend(q, &v, 0) != pdTRUE) { fail(8); return; }
    flexe_q_result[11] = (uint32_t)uxQueueMessagesWaiting(q);
    got = 0;
    t0 = esp_timer_get_time();
    ok = xQueueReceive(q, &got, pdMS_TO_TICKS(SHORT_TIMEOUT_MS));
    flexe_q_result[12] = (uint32_t)ok;
    flexe_q_result[13] = got;
    flexe_q_result[14] = (uint32_t)(esp_timer_get_time() - t0);
    flexe_q_result[15] = (uint32_t)uxQueueMessagesWaiting(q);
    if (ok != pdTRUE || got != 0xABCD1234u) { fail(9); return; }

    flexe_q_stage = SUCCESS_MARKER;
}

void loop() { delay(10); }
