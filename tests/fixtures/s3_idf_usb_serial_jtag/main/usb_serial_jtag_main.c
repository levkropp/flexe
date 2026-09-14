/* Exercise the stock ESP-IDF USB Serial/JTAG interrupt-driven driver with a
 * host-supplied OUT packet and a reply sent through its buffered IN path. */
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "driver/usb_serial_jtag.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static void fail(const char *stage, int value)
{
    printf("USJ_FAIL %s %d\n", stage, value);
    fflush(stdout);
    for (;;) vTaskDelay(pdMS_TO_TICKS(100));
}

void app_main(void)
{
    usb_serial_jtag_driver_config_t config =
        USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
    esp_err_t err = usb_serial_jtag_driver_install(&config);
    if (err != ESP_OK) fail("install", err);

    static const char greeting[] = "USJ_TX_READY\n";
    int written = usb_serial_jtag_write_bytes(
        greeting, sizeof(greeting) - 1u, pdMS_TO_TICKS(1000));
    if (written != (int)(sizeof(greeting) - 1u))
        fail("greeting", written);
    printf("USJ_DRIVER_READY\n");
    fflush(stdout);

    uint8_t input[4];
    for (unsigned round = 1u;; round++) {
        size_t received = 0u;
        while (received < sizeof(input)) {
            int count = usb_serial_jtag_read_bytes(
                input + received, sizeof(input) - received,
                pdMS_TO_TICKS(1000));
            if (count < 0) fail("receive", count);
            received += (size_t)count;
        }
        if (memcmp(input, "ping", sizeof(input)) != 0)
            fail("payload", round);

        char reply[32];
        int length = snprintf(reply, sizeof(reply), "USJ_PONG_%u\n", round);
        if (length <= 0 || length >= (int)sizeof(reply))
            fail("format", length);
        written = usb_serial_jtag_write_bytes(
            reply, (size_t)length, pdMS_TO_TICKS(1000));
        if (written != length) fail("reply", written);
        printf("USJ_ROUND %u count=%u\n", round, (unsigned)received);
        fflush(stdout);
    }
}
