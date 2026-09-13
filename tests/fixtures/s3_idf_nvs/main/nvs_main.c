/* Native ESP-IDF NVS and flash-persistence probe. Flexe must run these
 * library and SPI-flash paths as guest code, with no NVS API interception. */
#include <inttypes.h>
#include <stdio.h>

#include "esp_err.h"
#include "esp_system.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static void fail(const char *stage, esp_err_t error)
{
    printf("NVS_FAIL %s error=0x%x\n", stage, (unsigned)error);
    fflush(stdout);
    for (;;) vTaskDelay(pdMS_TO_TICKS(100));
}

void app_main(void)
{
    esp_err_t err = nvs_flash_init();
    if (err != ESP_OK) fail("flash_init", err);

    nvs_handle_t handle;
    err = nvs_open("flexe", NVS_READWRITE, &handle);
    if (err != ESP_OK) fail("open", err);

    uint32_t value = 0u;
    err = nvs_get_u32(handle, "counter", &value);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        printf("NVS_FIRST_BOOT empty\n");
        err = nvs_set_u32(handle, "counter", 0x5A17C0DEu);
        if (err != ESP_OK) fail("set", err);
        err = nvs_commit(handle);
        if (err != ESP_OK) fail("commit", err);
        nvs_close(handle);
        printf("NVS_COMMITTED value=0x%08" PRIx32 "\n",
               (uint32_t)0x5A17C0DEu);
        fflush(stdout);
        esp_restart();
        fail("restart_returned", ESP_FAIL);
    }
    if (err != ESP_OK) fail("get", err);
    if (value != 0x5A17C0DEu) fail("value_mismatch", ESP_FAIL);
    nvs_close(handle);
    printf("NVS_SECOND_BOOT value=0x%08" PRIx32 "\n", value);
    fflush(stdout);
    for (unsigned tick = 0u;; tick++) {
        printf("NVS_ALIVE %u\n", tick);
        fflush(stdout);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
