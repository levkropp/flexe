/* Native ESP-IDF S3 timer-driven light/deep sleep. The RTC alarm, sleep
 * state machine, wake cause, and RTC-retained state are exercised by the
 * guest itself; none of these calls are Flexe compatibility shims. */
#include <inttypes.h>
#include <stdio.h>

#include "esp_attr.h"
#include "esp_sleep.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "soc/rtc_cntl_reg.h"
#include "soc/soc.h"

#define RTC_MARKER 0x51EECAFEu
#define RTC_STORE_MARKER 0xC0DE5A17u

RTC_DATA_ATTR static uint32_t rtc_marker;
RTC_DATA_ATTR static uint64_t rtc_before_deep;

static uint64_t rtc_ticks(void)
{
    REG_WRITE(RTC_CNTL_TIME_UPDATE_REG, RTC_CNTL_TIME_UPDATE);
    return REG_READ(RTC_CNTL_TIME0_REG) |
           ((uint64_t)REG_READ(RTC_CNTL_TIME1_REG) << 32u);
}

static void fail(const char *reason)
{
    printf("SLEEP_FAIL %s\n", reason);
    fflush(stdout);
    for (;;) vTaskDelay(pdMS_TO_TICKS(100));
}

void app_main(void)
{
    if (rtc_marker == RTC_MARKER) {
        esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
        uint32_t store = REG_READ(RTC_CNTL_STORE0_REG);
        esp_reset_reason_t reset = esp_reset_reason();
        uint64_t deep_ticks = rtc_ticks() - rtc_before_deep;
        printf("SLEEP_DEEP_WOKE cause=%d reset=%d marker=%08" PRIx32
               " store=%08" PRIx32 " ticks=%" PRIu64 "\n",
               cause, reset, rtc_marker, store, deep_ticks);
        fflush(stdout);
        if (cause != ESP_SLEEP_WAKEUP_TIMER) fail("deep_cause");
        if (reset != ESP_RST_DEEPSLEEP) fail("deep_reset");
        if (store != RTC_STORE_MARKER) fail("rtc_store");
        if (deep_ticks < 2000u || deep_ticks > 8000u)
            fail("deep_rtc_counter");
        for (unsigned i = 0u;; i++) {
            printf("SLEEP_ALIVE %u\n", i);
            fflush(stdout);
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }

    if (rtc_marker != 0u) fail("cold_marker");
    printf("SLEEP_BEGIN\n");
    fflush(stdout);
    if (esp_sleep_enable_timer_wakeup(50000u) != ESP_OK)
        fail("light_arm");
    uint64_t before_light = rtc_ticks();
    esp_err_t result = esp_light_sleep_start();
    esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
    uint64_t light_ticks = rtc_ticks() - before_light;
    printf("SLEEP_LIGHT_WOKE result=%d cause=%d ticks=%" PRIu64 "\n",
           result, cause, light_ticks);
    fflush(stdout);
    if (result != ESP_OK || cause != ESP_SLEEP_WAKEUP_TIMER)
        fail("light_cause");
    if (light_ticks < 6000u || light_ticks > 9000u)
        fail("light_rtc_counter");

    rtc_marker = RTC_MARKER;
    REG_WRITE(RTC_CNTL_STORE0_REG, RTC_STORE_MARKER);
    if (esp_sleep_enable_timer_wakeup(20000u) != ESP_OK)
        fail("deep_arm");
    rtc_before_deep = rtc_ticks();
    printf("SLEEP_DEEP_ENTER\n");
    fflush(stdout);
    esp_deep_sleep_start();
    fail("deep_returned");
}
