/* Exercise ESP-IDF 5.3's public LCD i80 driver through the S3 LCD_CAM and
 * GDMA register paths: command-only and parameter polling transactions,
 * queued color descriptors, shared-source ISR callbacks, and teardown. */
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "esp_err.h"
#include "esp_lcd_io_i80.h"
#include "esp_lcd_panel_io.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

volatile uint32_t flexe_lcd_i80_stage;
volatile uint32_t flexe_lcd_i80_callbacks;
volatile uint32_t flexe_lcd_i80_result[8];

static void fail(uint32_t stage, int32_t detail)
{
    flexe_lcd_i80_result[7] = (uint32_t)detail;
    flexe_lcd_i80_stage = UINT32_C(0xBAD00000) | stage;
    printf("LCD_I80_FAIL stage=%u detail=%ld\n",
           (unsigned)stage, (long)detail);
    fflush(stdout);
    for (;;) vTaskDelay(pdMS_TO_TICKS(100));
}

static void require_ok(uint32_t stage, esp_err_t error)
{
    flexe_lcd_i80_result[stage & 7u] = (uint32_t)error;
    if (error != ESP_OK) fail(stage, error);
}

static bool on_color_done(esp_lcd_panel_io_handle_t io,
                          esp_lcd_panel_io_event_data_t *event,
                          void *context)
{
    (void)io;
    (void)event;
    uint32_t expected_tag = (uint32_t)(uintptr_t)context;
    if (expected_tag != UINT32_C(0x1CD18080))
        flexe_lcd_i80_result[6] = expected_tag;
    flexe_lcd_i80_callbacks++;
    return false;
}

void app_main(void)
{
    flexe_lcd_i80_stage = 1u;
    esp_lcd_i80_bus_handle_t bus = NULL;
    esp_lcd_i80_bus_config_t bus_config = {
        .dc_gpio_num = 12,
        .wr_gpio_num = 13,
        .clk_src = LCD_CLK_SRC_DEFAULT,
        .data_gpio_nums = { 4, 5, 6, 7, 8, 9, 10, 11 },
        .bus_width = 8,
        .max_transfer_bytes = 64,
        .dma_burst_size = 32,
    };
    require_ok(2u, esp_lcd_new_i80_bus(&bus_config, &bus));

    esp_lcd_panel_io_handle_t io = NULL;
    esp_lcd_panel_io_i80_config_t io_config = {
        .cs_gpio_num = 14,
        .pclk_hz = 5000000,
        .trans_queue_depth = 3,
        .on_color_trans_done = on_color_done,
        .user_ctx = (void *)(uintptr_t)UINT32_C(0x1CD18080),
        .dc_levels = {
            .dc_idle_level = 0,
            .dc_cmd_level = 0,
            .dc_dummy_level = 0,
            .dc_data_level = 1,
        },
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
    };
    require_ok(3u, esp_lcd_new_panel_io_i80(bus, &io_config, &io));

    const uint8_t parameters[] = {
        0x01u, 0x23u, 0x45u, 0x67u, 0x89u, 0xABu,
    };
    require_ok(4u, esp_lcd_panel_io_tx_param(
        io, 0xA5, parameters, sizeof(parameters)));
    require_ok(5u, esp_lcd_panel_io_tx_param(io, 0x11, NULL, 0u));

    uint8_t colors0[32];
    uint8_t colors1[18];
    for (unsigned index = 0u; index < sizeof(colors0); index++)
        colors0[index] = (uint8_t)(0x40u + index);
    for (unsigned index = 0u; index < sizeof(colors1); index++)
        colors1[index] = (uint8_t)(0xC0u + index);
    require_ok(6u, esp_lcd_panel_io_tx_color(
        io, 0x2C, colors0, sizeof(colors0)));
    require_ok(7u, esp_lcd_panel_io_tx_color(
        io, -1, colors1, sizeof(colors1)));

    for (unsigned waited = 0u;
         waited < 100u && flexe_lcd_i80_callbacks < 2u; waited++)
        vTaskDelay(pdMS_TO_TICKS(1));
    if (flexe_lcd_i80_callbacks != 2u)
        fail(8u, (int32_t)flexe_lcd_i80_callbacks);
    if (flexe_lcd_i80_result[6] != 0u)
        fail(9u, (int32_t)flexe_lcd_i80_result[6]);

    require_ok(10u, esp_lcd_panel_io_del(io));
    require_ok(11u, esp_lcd_del_i80_bus(bus));
    flexe_lcd_i80_stage = UINT32_C(0x4C434480);
    printf("LCD_CAM_DONE params=2 colors=2 callbacks=2 teardown=ok\n");
    fflush(stdout);
}
