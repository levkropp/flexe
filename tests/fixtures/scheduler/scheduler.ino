/* The rest of the FreeRTOS scheduling surface: periodic delays, a send that
 * blocks on a full queue, recursive mutexes, and suspend/resume.
 *
 * These are the blocking and task-control calls left over once the plain
 * receive, take and event-group waits are covered. Each is something ordinary
 * firmware does: vTaskDelayUntil is how every fixed-rate loop is written, a
 * bounded queue backs every producer/consumer with flow control, recursive
 * mutexes guard re-entrant library code, and suspend/resume is how a worker
 * is parked.
 *
 * Stages advance one per feature so that a hang names the feature that hung
 * rather than just failing the run.
 */
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>

#define SUCCESS_MARKER 0x5CED0C0Bu
#define FAIL_BASE      0xBAD00000u

#define PERIOD_MS    20u
#define PERIODS      10u
#define DRAIN_MS     50u
#define LONG_TIMEOUT_MS 500u

volatile uint32_t flexe_sched_stage = 0;
volatile uint32_t flexe_sched_result[16];

static QueueHandle_t q;
static SemaphoreHandle_t rmutex;
static volatile uint32_t worker_ticks = 0;
static TaskHandle_t worker_h;

static void fail(uint32_t code) { flexe_sched_stage = FAIL_BASE | code; }

/* Drains one item after a delay, so a send blocked on a full queue has to be
 * woken by the receive rather than by its own timeout. */
static void drain_task(void *arg) {
    vTaskDelay(pdMS_TO_TICKS(DRAIN_MS));
    uint32_t v;
    xQueueReceive(q, &v, 0);
    vTaskDelete(NULL);
}

/* Counts while it runs, so suspending it can be observed as the count
 * standing still. */
static void worker_task(void *arg) {
    for (;;) {
        worker_ticks++;
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

void setup() {
    flexe_sched_stage = 1;

    /* (a) vTaskDelayUntil: a fixed-rate loop. Ten periods of 20 ms is 200 ms
     * of wall time; an implementation that does not block at all returns
     * immediately, and one that treats the deadline as a plain delay drifts. */
    TickType_t last = xTaskGetTickCount();
    int64_t t0 = esp_timer_get_time();
    for (unsigned i = 0; i < PERIODS; i++)
        vTaskDelayUntil(&last, pdMS_TO_TICKS(PERIOD_MS));
    flexe_sched_result[0] = (uint32_t)(esp_timer_get_time() - t0);
    flexe_sched_stage = 2;

    /* (b) A send that has to block: fill the queue, then have another task
     * take one item 50 ms later. */
    q = xQueueCreate(2, sizeof(uint32_t));
    if (!q) { fail(1); return; }
    uint32_t v = 1;
    if (xQueueSend(q, &v, 0) != pdTRUE) { fail(2); return; }
    v = 2;
    if (xQueueSend(q, &v, 0) != pdTRUE) { fail(3); return; }
    /* Third send with no timeout must fail: the queue is full. */
    v = 3;
    flexe_sched_result[1] = (uint32_t)xQueueSend(q, &v, 0);
    if (flexe_sched_result[1] != pdFALSE) { fail(4); return; }

    if (xTaskCreate(drain_task, "drain", 4096, NULL, 5, NULL) != pdPASS)
        { fail(5); return; }
    t0 = esp_timer_get_time();
    BaseType_t ok = xQueueSend(q, &v, pdMS_TO_TICKS(LONG_TIMEOUT_MS));
    flexe_sched_result[2] = (uint32_t)ok;
    flexe_sched_result[3] = (uint32_t)(esp_timer_get_time() - t0);
    if (ok != pdTRUE) { fail(6); return; }
    flexe_sched_stage = 3;

    /* (c) A recursive mutex, taken twice by the same task. A plain binary
     * semaphore underneath would deadlock on the second take. */
    rmutex = xSemaphoreCreateRecursiveMutex();
    if (!rmutex) { fail(7); return; }
    t0 = esp_timer_get_time();
    if (xSemaphoreTakeRecursive(rmutex, pdMS_TO_TICKS(LONG_TIMEOUT_MS)) != pdTRUE)
        { fail(8); return; }
    if (xSemaphoreTakeRecursive(rmutex, pdMS_TO_TICKS(LONG_TIMEOUT_MS)) != pdTRUE)
        { fail(9); return; }
    flexe_sched_result[4] = (uint32_t)(esp_timer_get_time() - t0);
    if (xSemaphoreGiveRecursive(rmutex) != pdTRUE) { fail(10); return; }
    if (xSemaphoreGiveRecursive(rmutex) != pdTRUE) { fail(11); return; }
    /* Fully released, so a fresh take succeeds at once. */
    if (xSemaphoreTakeRecursive(rmutex, 0) != pdTRUE) { fail(12); return; }
    if (xSemaphoreGiveRecursive(rmutex) != pdTRUE) { fail(13); return; }
    flexe_sched_stage = 4;

    /* (d) Suspend and resume another task. The count must stand still while
     * it is suspended and move again after it is resumed. */
    if (xTaskCreate(worker_task, "worker", 4096, NULL, 4, &worker_h) != pdPASS)
        { fail(14); return; }
    delay(40);
    uint32_t before = worker_ticks;
    if (before == 0) { fail(15); return; }   /* it never ran at all */
    vTaskSuspend(worker_h);
    uint32_t at_suspend = worker_ticks;
    delay(40);
    flexe_sched_result[5] = before;
    flexe_sched_result[6] = at_suspend;
    flexe_sched_result[7] = worker_ticks;    /* must equal at_suspend */
    vTaskResume(worker_h);
    delay(40);
    flexe_sched_result[8] = worker_ticks;    /* must have moved on */
    flexe_sched_stage = 5;

    flexe_sched_stage = SUCCESS_MARKER;
}

void loop() { delay(10); }
