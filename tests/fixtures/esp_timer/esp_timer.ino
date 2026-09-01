/* esp_timer callbacks: periodic, one-shot, restart and stop.
 *
 * Nothing else in the tree drives the public esp_timer API from guest code.
 * The unit tests poke the stub directly, and the peripheral gates all use the
 * timer-group hardware instead -- so the callback dispatch path, and above all
 * whether a callback lands at the *right* virtual time, is uncovered. Arduino
 * Ticker, LVGL's tick source and the WiFi/BLE stacks are all built on it, so a
 * timer that fires at the wrong rate would show up as firmware that is subtly
 * too fast or too slow rather than as anything that looks like a fault.
 *
 * Every deadline here is checked against esp_timer_get_time() rather than a
 * callback count alone: a dispatcher that fires the right number of times but
 * bunches them up, or one that lets a periodic timer drift, passes a count
 * check and fails this one.
 */
#include <Arduino.h>
#include <esp_timer.h>

#define SUCCESS_MARKER 0x71ED0C0Bu
#define FAIL_BASE      0xBAD00000u

#define PERIOD_US   10000u    /* 10 ms periodic */
#define ONESHOT_US  45000u    /* 45 ms one-shot */
#define RUN_MS      200u

volatile uint32_t flexe_timer_stage = 0;
volatile uint32_t flexe_timer_result[12];

static esp_timer_handle_t periodic_h, oneshot_h;

static volatile uint32_t periodic_count = 0;
static volatile uint32_t oneshot_count = 0;
static volatile uint32_t periodic_arg_ok = 1;
static volatile uint32_t oneshot_arg_ok = 1;

/* Widest gap seen between consecutive periodic callbacks, and the narrowest,
 * both in microseconds. A dispatcher that catches up by firing a burst shows
 * up as a min near zero; one that loses time shows up as a large max. */
static volatile uint32_t gap_min = 0xFFFFFFFFu;
static volatile uint32_t gap_max = 0;
static volatile int64_t  last_cb_us = 0;
static volatile int64_t  first_cb_us = 0;
static volatile int64_t  oneshot_at_us = 0;
static volatile int64_t  start_us = 0;

static void fail(uint32_t code) { flexe_timer_stage = FAIL_BASE | code; }

static void periodic_cb(void *arg) {
    int64_t now = esp_timer_get_time();
    if ((uint32_t)(uintptr_t)arg != 0xC0FFEEu) periodic_arg_ok = 0;
    if (periodic_count == 0) {
        first_cb_us = now;
    } else {
        uint32_t gap = (uint32_t)(now - last_cb_us);
        if (gap < gap_min) gap_min = gap;
        if (gap > gap_max) gap_max = gap;
    }
    last_cb_us = now;
    periodic_count++;
}

static void oneshot_cb(void *arg) {
    if ((uint32_t)(uintptr_t)arg != 0x5EEDu) oneshot_arg_ok = 0;
    oneshot_at_us = esp_timer_get_time();
    oneshot_count++;
}

void setup() {
    esp_timer_create_args_t pa = {};
    pa.callback = &periodic_cb;
    pa.arg = (void *)0xC0FFEEu;
    pa.name = "periodic";
    if (esp_timer_create(&pa, &periodic_h) != ESP_OK) { fail(1); return; }

    esp_timer_create_args_t oa = {};
    oa.callback = &oneshot_cb;
    oa.arg = (void *)0x5EEDu;
    oa.name = "oneshot";
    if (esp_timer_create(&oa, &oneshot_h) != ESP_OK) { fail(2); return; }

    start_us = esp_timer_get_time();
    if (esp_timer_start_periodic(periodic_h, PERIOD_US) != ESP_OK) { fail(3); return; }
    if (esp_timer_start_once(oneshot_h, ONESHOT_US) != ESP_OK) { fail(4); return; }
    flexe_timer_stage = 1;

    delay(RUN_MS);

    /* Stopping has to actually stop it: sample, wait, and require no further
     * callbacks. This is the half that a dispatcher ignoring `active` passes
     * by accident. */
    if (esp_timer_stop(periodic_h) != ESP_OK) { fail(5); return; }
    uint32_t after_stop = periodic_count;
    int64_t stopped_us = esp_timer_get_time();
    delay(50);
    flexe_timer_result[0] = periodic_count;
    flexe_timer_result[1] = after_stop;
    flexe_timer_result[2] = oneshot_count;
    flexe_timer_result[3] = gap_min;
    flexe_timer_result[4] = gap_max;
    flexe_timer_result[5] = (uint32_t)(last_cb_us - first_cb_us);
    flexe_timer_result[6] = (uint32_t)(oneshot_at_us - start_us);
    flexe_timer_result[7] = (uint32_t)(stopped_us - start_us);
    flexe_timer_result[8] = periodic_arg_ok && oneshot_arg_ok;

    if (periodic_count != after_stop) { fail(6); return; }

    /* A stopped timer must be restartable, and must not resume mid-period. */
    if (esp_timer_start_periodic(periodic_h, PERIOD_US) != ESP_OK) { fail(7); return; }
    delay(50);
    flexe_timer_result[9] = periodic_count - after_stop;
    if (esp_timer_stop(periodic_h) != ESP_OK) { fail(8); return; }
    if (esp_timer_delete(oneshot_h) != ESP_OK) { fail(9); return; }

    flexe_timer_result[10] = (uint32_t)(esp_timer_get_time() - start_us);
    flexe_timer_stage = SUCCESS_MARKER;
}

void loop() { delay(10); }
