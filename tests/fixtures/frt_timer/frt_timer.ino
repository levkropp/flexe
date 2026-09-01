/* FreeRTOS software timers: auto-reload, one-shot, period change and reset.
 *
 * Nothing hooks xTimerCreate and friends, so unlike the rest of the FreeRTOS
 * surface these run the guest's own implementation -- the Tmr Svc task, its
 * command queue and its callback dispatch -- on top of Flexe's stubbed
 * scheduler and queues. That seam is untested, and it is not a niche one:
 * Marauder's image carries a Tmr Svc task, and the WiFi and BLE stacks both
 * schedule work this way.
 *
 * As in the esp_timer gate, the deadlines are checked against a clock rather
 * than by counting callbacks alone: a timer service that fires the right
 * number of times but bunches them up passes a count check.
 */
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/timers.h>

#define SUCCESS_MARKER 0x71E20C0Bu
#define FAIL_BASE      0xBAD00000u

#define PERIOD_MS   20u
#define ONESHOT_MS  70u
#define RUN_MS      300u
#define NEWPERIOD_MS 40u

volatile uint32_t flexe_frt_timer_stage = 0;
volatile uint32_t flexe_frt_timer_result[12];

static TimerHandle_t reload_h, oneshot_h;

static volatile uint32_t reload_count = 0;
static volatile uint32_t oneshot_count = 0;
static volatile uint32_t id_ok = 1;
static volatile uint32_t gap_min = 0xFFFFFFFFu;
static volatile uint32_t gap_max = 0;
static volatile int64_t  last_cb_us = 0;
static volatile int64_t  first_cb_us = 0;
static volatile int64_t  oneshot_at_us = 0;
static volatile int64_t  start_us = 0;

static void fail(uint32_t code) { flexe_frt_timer_stage = FAIL_BASE | code; }

static void reload_cb(TimerHandle_t t) {
    int64_t now = esp_timer_get_time();
    if ((uint32_t)(uintptr_t)pvTimerGetTimerID(t) != 0xA11Eu) id_ok = 0;
    if (reload_count == 0) {
        first_cb_us = now;
    } else {
        uint32_t gap = (uint32_t)(now - last_cb_us);
        if (gap < gap_min) gap_min = gap;
        if (gap > gap_max) gap_max = gap;
    }
    last_cb_us = now;
    reload_count++;
}

static void oneshot_cb(TimerHandle_t t) {
    if ((uint32_t)(uintptr_t)pvTimerGetTimerID(t) != 0xB0B0u) id_ok = 0;
    oneshot_at_us = esp_timer_get_time();
    oneshot_count++;
}

void setup() {
    reload_h = xTimerCreate("reload", pdMS_TO_TICKS(PERIOD_MS), pdTRUE,
                            (void *)0xA11Eu, reload_cb);
    if (!reload_h) { fail(1); return; }
    oneshot_h = xTimerCreate("oneshot", pdMS_TO_TICKS(ONESHOT_MS), pdFALSE,
                             (void *)0xB0B0u, oneshot_cb);
    if (!oneshot_h) { fail(2); return; }

    start_us = esp_timer_get_time();
    if (xTimerStart(reload_h, 0) != pdPASS) { fail(3); return; }
    if (xTimerStart(oneshot_h, 0) != pdPASS) { fail(4); return; }
    flexe_frt_timer_stage = 1;

    delay(RUN_MS);

    /* A one-shot must not be active once it has fired; an auto-reload must. */
    flexe_frt_timer_result[6] = xTimerIsTimerActive(oneshot_h) ? 1 : 0;
    flexe_frt_timer_result[7] = xTimerIsTimerActive(reload_h) ? 1 : 0;

    if (xTimerStop(reload_h, 0) != pdPASS) { fail(5); return; }
    delay(30);                       /* let the stop command be serviced */
    uint32_t after_stop = reload_count;
    delay(60);
    flexe_frt_timer_result[0] = reload_count;
    flexe_frt_timer_result[1] = after_stop;
    flexe_frt_timer_result[2] = oneshot_count;
    flexe_frt_timer_result[3] = gap_min;
    flexe_frt_timer_result[4] = gap_max;
    flexe_frt_timer_result[5] = (uint32_t)(last_cb_us - first_cb_us);
    flexe_frt_timer_result[8] = (uint32_t)(oneshot_at_us - start_us);
    flexe_frt_timer_result[9] = id_ok;

    /* A stopped timer must stay stopped. */
    if (reload_count != after_stop) { fail(6); return; }

    /* Changing the period restarts the timer at the new one. Count callbacks
     * over a window that only matches if the new period took effect. */
    gap_min = 0xFFFFFFFFu; gap_max = 0; reload_count = 0;
    if (xTimerChangePeriod(reload_h, pdMS_TO_TICKS(NEWPERIOD_MS), 0) != pdPASS)
        { fail(7); return; }
    delay(200);
    flexe_frt_timer_result[10] = reload_count;
    flexe_frt_timer_result[11] = gap_max;
    if (xTimerStop(reload_h, 0) != pdPASS) { fail(8); return; }
    if (xTimerDelete(oneshot_h, 0) != pdPASS) { fail(9); return; }

    flexe_frt_timer_stage = SUCCESS_MARKER;
}

void loop() { delay(10); }
