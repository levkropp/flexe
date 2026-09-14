/* Native ESP-IDF S3 EXT0 light sleep and EXT1 deep sleep. A host drives the
 * real guest-visible GPIO input while both emulated cores are asleep. */
#include <inttypes.h>
#include <stdio.h>

#include "esp_attr.h"
#include "esp_sleep.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define RTC_MARKER 0xE7A1C0DEu
#define EXT1_PINS ((UINT64_C(1) << 11u) | (UINT64_C(1) << 12u))

RTC_DATA_ATTR static uint32_t rtc_marker;

static void fail(const char *why)
{
    printf("GPIO_WAKE_FAIL %s\n", why);
    fflush(stdout);
    for (;;) vTaskDelay(pdMS_TO_TICKS(100));
}

void app_main(void)
{
    if (rtc_marker == RTC_MARKER) {
        esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
        uint64_t status = esp_sleep_get_ext1_wakeup_status();
        esp_reset_reason_t reset = esp_reset_reason();
        printf("GPIO_EXT1_WOKE cause=%d reset=%d status=%016" PRIx64
               " marker=%08" PRIx32 "\n",
               cause, reset, status, rtc_marker);
        fflush(stdout);
        if (cause != ESP_SLEEP_WAKEUP_EXT1) fail("ext1_cause");
        if (reset != ESP_RST_DEEPSLEEP) fail("ext1_reset");
        if (status != (UINT64_C(1) << 12u)) fail("ext1_status");
        for (unsigned n = 0u;; n++) {
            printf("GPIO_WAKE_ALIVE %u\n", n);
            fflush(stdout);
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }

    if (rtc_marker != 0u) fail("cold_marker");
    printf("GPIO_EXT0_ARMED\n");
    fflush(stdout);
    if (esp_sleep_enable_ext0_wakeup(GPIO_NUM_4, 1) != ESP_OK)
        fail("ext0_arm");
    esp_err_t result = esp_light_sleep_start();
    esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
    printf("GPIO_EXT0_WOKE result=%d cause=%d\n", result, cause);
    fflush(stdout);
    if (result != ESP_OK || cause != ESP_SLEEP_WAKEUP_EXT0)
        fail("ext0_cause");

    if (esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_EXT0) != ESP_OK)
        fail("ext0_disable");
    if (esp_sleep_enable_ext1_wakeup_io(
            EXT1_PINS, ESP_EXT1_WAKEUP_ANY_HIGH) != ESP_OK)
        fail("ext1_arm");
    rtc_marker = RTC_MARKER;
    printf("GPIO_EXT1_ARMED\n");
    fflush(stdout);
    esp_deep_sleep_start();
    fail("ext1_returned");
}
