/* FreeRTOS event groups: set, clear, wait-for-any, wait-for-all, timeout and
 * cross-task wakeup.
 *
 * Nothing hooks the xEventGroup API, so like the software timers before them
 * these run the guest's own implementation -- which blocks tasks by threading
 * them onto FreeRTOS's own unordered event lists, a structure Flexe's
 * scheduler knows nothing about. That is the seam under test. It is not a
 * niche one: this is how the WiFi stack, lwIP and most connect-then-wait
 * Arduino sketches synchronise, so Marauder's WiFi paths depend on it.
 *
 * The blocking cases are the point. Setting and reading bits from one task
 * can work while every wait is broken, so each wait here is checked for the
 * bits it returned *and* for how long it actually blocked.
 */
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>

#define SUCCESS_MARKER 0xE7600C0Bu
#define FAIL_BASE      0xBAD00000u

#define BIT_A  (1 << 0)
#define BIT_B  (1 << 1)
#define BIT_C  (1 << 2)

#define TIMEOUT_MS  120u
#define SETTER_MS   80u

volatile uint32_t flexe_eg_stage = 0;
volatile uint32_t flexe_eg_result[12];

static EventGroupHandle_t eg;

static void fail(uint32_t code) { flexe_eg_stage = FAIL_BASE | code; }

/* Sets BIT_B after a delay, so the main task's wait has to actually block and
 * then be woken by another task rather than returning early. */
static void setter_task(void *arg) {
    vTaskDelay(pdMS_TO_TICKS(SETTER_MS));
    xEventGroupSetBits(eg, BIT_B);
    vTaskDelete(NULL);
}

void setup() {
    eg = xEventGroupCreate();
    if (!eg) { fail(1); return; }
    flexe_eg_stage = 1;

    /* Bits start clear. */
    if (xEventGroupGetBits(eg) != 0) { fail(2); return; }

    /* Set and read back, no blocking involved. */
    xEventGroupSetBits(eg, BIT_A | BIT_C);
    flexe_eg_result[0] = xEventGroupGetBits(eg);
    if (flexe_eg_result[0] != (BIT_A | BIT_C)) { fail(3); return; }

    xEventGroupClearBits(eg, BIT_C);
    flexe_eg_result[1] = xEventGroupGetBits(eg);
    if (flexe_eg_result[1] != BIT_A) { fail(4); return; }

    /* A bit that is already set must return immediately, not after the
     * timeout. xClearOnExit=pdTRUE, so BIT_A is consumed. */
    int64_t t0 = esp_timer_get_time();
    EventBits_t got = xEventGroupWaitBits(eg, BIT_A, pdTRUE, pdFALSE,
                                          pdMS_TO_TICKS(TIMEOUT_MS));
    flexe_eg_result[2] = got;
    flexe_eg_result[3] = (uint32_t)(esp_timer_get_time() - t0);
    if (!(got & BIT_A)) { fail(5); return; }
    if (xEventGroupGetBits(eg) & BIT_A) { fail(6); return; }  /* cleared on exit */

    /* Nothing is set: this one has to block for the whole timeout and come
     * back without the bit. */
    t0 = esp_timer_get_time();
    got = xEventGroupWaitBits(eg, BIT_C, pdFALSE, pdFALSE,
                              pdMS_TO_TICKS(TIMEOUT_MS));
    flexe_eg_result[4] = got;
    flexe_eg_result[5] = (uint32_t)(esp_timer_get_time() - t0);
    if (got & BIT_C) { fail(7); return; }
    flexe_eg_stage = 2;

    /* Another task sets the bit partway through the timeout: the wait must
     * block, then be woken early by that task. */
    xEventGroupClearBits(eg, 0xFF);
    if (xTaskCreate(setter_task, "setter", 4096, NULL, 5, NULL) != pdPASS)
        { fail(8); return; }
    t0 = esp_timer_get_time();
    got = xEventGroupWaitBits(eg, BIT_B, pdTRUE, pdFALSE,
                              pdMS_TO_TICKS(TIMEOUT_MS * 4));
    flexe_eg_result[6] = got;
    flexe_eg_result[7] = (uint32_t)(esp_timer_get_time() - t0);
    if (!(got & BIT_B)) { fail(9); return; }
    flexe_eg_stage = 3;

    /* Wait-for-all: one of the two bits is set, so this must time out rather
     * than returning on the partial match. */
    xEventGroupClearBits(eg, 0xFF);
    xEventGroupSetBits(eg, BIT_A);
    t0 = esp_timer_get_time();
    got = xEventGroupWaitBits(eg, BIT_A | BIT_C, pdFALSE, pdTRUE,
                              pdMS_TO_TICKS(TIMEOUT_MS));
    flexe_eg_result[8] = got;
    flexe_eg_result[9] = (uint32_t)(esp_timer_get_time() - t0);
    if ((got & (BIT_A | BIT_C)) == (BIT_A | BIT_C)) { fail(10); return; }

    /* Now both are set, so the same wait returns at once. */
    xEventGroupSetBits(eg, BIT_C);
    t0 = esp_timer_get_time();
    got = xEventGroupWaitBits(eg, BIT_A | BIT_C, pdFALSE, pdTRUE,
                              pdMS_TO_TICKS(TIMEOUT_MS));
    flexe_eg_result[10] = got;
    flexe_eg_result[11] = (uint32_t)(esp_timer_get_time() - t0);
    if ((got & (BIT_A | BIT_C)) != (BIT_A | BIT_C)) { fail(11); return; }

    vEventGroupDelete(eg);
    flexe_eg_stage = SUCCESS_MARKER;
}

void loop() { delay(10); }
