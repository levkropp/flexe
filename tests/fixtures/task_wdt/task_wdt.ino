/* The task watchdog API.
 *
 * esp_task_wdt_init/add/reset used to answer with a bare success and the rest
 * of the family was not modelled, so the real ESP-IDF code ran against a
 * subscription list nothing had populated. Arduino's disableCore0WDT() calls
 * esp_task_wdt_delete() on the idle task and reports the failure -- it is in
 * NerdMiner's boot log on every run. Firmware that subscribes a task and
 * later unsubscribes it could not.
 *
 * What matters is that the calls agree with each other: a task that was added
 * is reported subscribed, a deleted one is not, and the errors are the ones
 * ESP-IDF returns rather than a blanket success.
 */
#include <Arduino.h>
#include <esp_task_wdt.h>

#define SUCCESS_MARKER 0x7D060C0Bu
#define FAIL_BASE      0xBAD00000u

volatile uint32_t flexe_wdt_stage = 0;
volatile uint32_t flexe_wdt_result[12];

static void fail(uint32_t code) { flexe_wdt_stage = FAIL_BASE | code; }

static TaskHandle_t worker_h;
static volatile uint32_t worker_ran = 0;

/* A second subscriber, so the model has to track more than one handle. */
static void worker_task(void *arg) {
  esp_task_wdt_add(NULL);
  for (;;) {
    esp_task_wdt_reset();
    worker_ran++;
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void setup() {
  /* The Arduino core initialises the task watchdog before setup() runs and
   * may already have subscribed this task, so start from a known state
   * rather than assuming an empty one. */
  if (esp_task_wdt_init(5, false) != ESP_OK) { fail(2); return; }
  esp_task_wdt_delete(NULL);
  flexe_wdt_stage = 1;

  /* Unsubscribed: both the status and a feed must say so. */
  flexe_wdt_result[0] = (uint32_t)esp_task_wdt_reset();
  if (flexe_wdt_result[0] == ESP_OK) { fail(1); return; }
  flexe_wdt_result[1] = (uint32_t)esp_task_wdt_status(NULL);
  if (flexe_wdt_result[1] == ESP_OK) { fail(3); return; }

  if (esp_task_wdt_add(NULL) != ESP_OK) { fail(4); return; }
  flexe_wdt_result[2] = (uint32_t)esp_task_wdt_status(NULL);
  if (flexe_wdt_result[2] != ESP_OK) { fail(5); return; }

  /* Adding twice is an error, not a second subscription. */
  flexe_wdt_result[3] = (uint32_t)esp_task_wdt_add(NULL);
  if (flexe_wdt_result[3] == ESP_OK) { fail(6); return; }

  /* Feeding a subscribed task succeeds. */
  for (int i = 0; i < 5; i++)
    if (esp_task_wdt_reset() != ESP_OK) { fail(7); return; }
  flexe_wdt_stage = 2;

  /* A second task subscribes itself and feeds independently. */
  if (xTaskCreate(worker_task, "wdtworker", 4096, NULL, 4, &worker_h) != pdPASS)
    { fail(8); return; }
  delay(60);
  flexe_wdt_result[4] = worker_ran;
  if (worker_ran == 0) { fail(9); return; }

  /* Deinit must refuse while subscribers remain. */
  flexe_wdt_result[5] = (uint32_t)esp_task_wdt_deinit();
  if (flexe_wdt_result[5] == ESP_OK) { fail(10); return; }
  flexe_wdt_stage = 3;

  /* Unsubscribe, and the status must follow. */
  if (esp_task_wdt_delete(NULL) != ESP_OK) { fail(11); return; }
  flexe_wdt_result[6] = (uint32_t)esp_task_wdt_status(NULL);
  if (flexe_wdt_result[6] == ESP_OK) { fail(12); return; }

  /* Deleting again fails: it is already gone. */
  flexe_wdt_result[7] = (uint32_t)esp_task_wdt_delete(NULL);
  if (flexe_wdt_result[7] == ESP_OK) { fail(13); return; }

  /* Feeding an unsubscribed task fails too. */
  flexe_wdt_result[8] = (uint32_t)esp_task_wdt_reset();
  if (flexe_wdt_result[8] == ESP_OK) { fail(14); return; }

  vTaskDelete(worker_h);
  flexe_wdt_result[9] = 1;

  /* The watchdog must not have fired: we stopped feeding it long ago and the
   * device is still here. A model that reset the chip would never reach this. */
  delay(200);
  flexe_wdt_result[10] = 1;
  flexe_wdt_stage = SUCCESS_MARKER;
}

void loop() { delay(10); }
