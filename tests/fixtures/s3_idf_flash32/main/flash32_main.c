/* 32 MiB data-flash exercise above the 16 MiB address line.
 *
 * Prints the JEDEC ID and SDK size, then erases, programs, and reads back one
 * page below the line (control) and two pages at/above 16 MiB through the
 * stock esp_flash driver. App and bootloader stay below the line; only data
 * partitions use 4-byte addressing, matching the silicon rule that
 * cache-mapped execution stays under 16 MiB without the experimental
 * BOOTLOADER_CACHE_32BIT_ADDR_QUAD_FLASH option.
 */
#include <stdio.h>
#include <string.h>
#include "esp_flash.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define CTRL_OFF 0x00400000u
#define HIGH_OFF 0x01000000u

static uint32_t checksum(const uint8_t *p, unsigned n)
{
    uint32_t acc = 0x811C9DC5u;
    for (unsigned i = 0u; i < n; i++) {
        acc ^= p[i];
        acc *= 0x01000193u;
    }
    return acc;
}

static void fill(uint8_t *p, unsigned n, unsigned seed)
{
    for (unsigned i = 0u; i < n; i++)
        p[i] = (uint8_t)((i * 3u + seed) & 0xFFu);
}

static void check(esp_err_t err, unsigned stage)
{
    if (err != ESP_OK)
        printf("FLASH32_FAIL stage=%u detail=%d\n", stage, (int)err);
}

int app_main(void)
{
    uint32_t jedec = 0u;
    check(esp_flash_read_id(NULL, &jedec), 1u);
    printf("FLASH_ID=%06x\n", (unsigned)jedec);

    uint32_t size = 0u;
    check(esp_flash_get_size(NULL, &size), 2u);
    printf("FLASH_SIZE=%u\n", (unsigned)size);

    static uint8_t wbuf[256];
    static uint8_t rbuf[256];
    uint32_t ctrl_sum = 0u;
    uint32_t hi_sum = 0u;

    fill(wbuf, sizeof(wbuf), 1u);
    check(esp_flash_erase_region(NULL, CTRL_OFF, 0x1000u), 3u);
    check(esp_flash_write(NULL, wbuf, CTRL_OFF, sizeof(wbuf)), 4u);
    check(esp_flash_read(NULL, rbuf, CTRL_OFF, sizeof(rbuf)), 5u);
    ctrl_sum = checksum(rbuf, sizeof(rbuf));
    printf("FLASH32_CTRL checksum=%08x\n", (unsigned)ctrl_sum);

    check(esp_flash_erase_region(NULL, HIGH_OFF, 0x2000u), 6u);
    fill(wbuf, sizeof(wbuf), 7u);
    check(esp_flash_write(NULL, wbuf, HIGH_OFF, sizeof(wbuf)), 7u);
    fill(wbuf, sizeof(wbuf), 9u);
    check(esp_flash_write(NULL, wbuf, HIGH_OFF + 0x1000u, sizeof(wbuf)),
          8u);
    check(esp_flash_read(NULL, rbuf, HIGH_OFF, sizeof(rbuf)), 9u);
    hi_sum = checksum(rbuf, sizeof(rbuf));
    check(esp_flash_read(NULL, rbuf, HIGH_OFF + 0x1000u, sizeof(rbuf)),
          10u);
    hi_sum ^= checksum(rbuf, sizeof(rbuf));
    printf("FLASH32_DONE ctrl=%08x hi=%08x\n", (unsigned)ctrl_sum,
           (unsigned)hi_sum);
    fflush(stdout);
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    return 0;
}
