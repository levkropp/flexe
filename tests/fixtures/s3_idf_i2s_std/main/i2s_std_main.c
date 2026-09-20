/* Exercise ESP-IDF 5.3's standard-mode I2S driver, its circular GDMA ring,
 * blocking writer wakeup, clock/reset gates, and GPIO matrix setup without
 * replacing any guest driver function. */
#include <stdint.h>
#include <stdio.h>

#include "driver/i2s_std.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define I2S_DONE UINT32_C(0x1232A5D0)
#define AUDIO_BYTES 1024u

volatile uint32_t flexe_i2s_stage;
volatile uint32_t flexe_i2s_result[5];

static void fail(const char *stage, esp_err_t error)
{
    flexe_i2s_result[4] = (uint32_t)error;
    flexe_i2s_stage = UINT32_C(0xBAD00000);
    printf("I2S_FAIL %s %d\n", stage, (int)error);
    fflush(stdout);
    for (;;) vTaskDelay(pdMS_TO_TICKS(100));
}

static void check(const char *stage, esp_err_t error)
{
    if (error != ESP_OK) fail(stage, error);
}

static uint32_t checksum(const uint8_t *data, size_t length)
{
    uint32_t value = UINT32_C(2166136261);
    for (size_t index = 0u; index < length; index++)
        value = (value ^ data[index]) * UINT32_C(16777619);
    return value;
}

void app_main(void)
{
    static uint8_t audio[AUDIO_BYTES];
    for (size_t index = 0u; index < sizeof(audio); index++)
        audio[index] = (uint8_t)((index * 73u + 19u) ^ (index >> 2u));
    flexe_i2s_result[3] = checksum(audio, sizeof(audio));

    i2s_chan_config_t channel_config =
        I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    channel_config.dma_desc_num = 4u;
    channel_config.dma_frame_num = 64u;
    i2s_chan_handle_t tx;
    check("new-channel", i2s_new_channel(&channel_config, &tx, NULL));

    i2s_std_config_t standard_config = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(48000),
        .slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(
            I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = GPIO_NUM_4,
            .ws = GPIO_NUM_5,
            .dout = GPIO_NUM_6,
            .din = I2S_GPIO_UNUSED,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false,
            },
        },
    };
    check("init-standard", i2s_channel_init_std_mode(tx, &standard_config));

    size_t preloaded_total = 0u;
    for (;;) {
        size_t loaded = 0u;
        check("preload", i2s_channel_preload_data(
            tx, audio, sizeof(audio), &loaded));
        preloaded_total += loaded;
        if (loaded == 0u) break;
    }
    flexe_i2s_result[0] = (uint32_t)preloaded_total;
    flexe_i2s_stage = 1u;
    check("enable", i2s_channel_enable(tx));

    for (unsigned round = 0u; round < 2u; round++) {
        size_t written = 0u;
        check("write", i2s_channel_write(
            tx, audio, sizeof(audio), &written, pdMS_TO_TICKS(1000)));
        if (written != sizeof(audio)) fail("short-write", ESP_FAIL);
        flexe_i2s_result[1u + round] = (uint32_t)written;
        flexe_i2s_stage = 2u + round;
    }

    check("disable", i2s_channel_disable(tx));
    check("delete", i2s_del_channel(tx));
    flexe_i2s_stage = I2S_DONE;
    printf("I2S_STD_DONE preload=%u writes=%u,%u checksum=%08X\n",
           (unsigned)flexe_i2s_result[0],
           (unsigned)flexe_i2s_result[1],
           (unsigned)flexe_i2s_result[2],
           (unsigned)flexe_i2s_result[3]);
    fflush(stdout);
    for (;;) vTaskDelay(pdMS_TO_TICKS(100));
}
