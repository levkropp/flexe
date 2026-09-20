/* Exercise Espressif's pinned esp32-camera component without replacing any
 * guest driver function: OV2640 discovery/configuration over SCCB, native S3
 * LCD_CAM/GDMA capture, a complete RGB565 frame, and clean teardown. */
#include <stdint.h>
#include <stdio.h>

#include "esp_camera.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define CAMERA_DONE UINT32_C(0x43414D80)
#define FRAME_BYTES (160u * 120u * 2u)

volatile uint32_t flexe_camera_stage;
volatile uint32_t flexe_camera_result[8];

static void fail(uint32_t stage, int32_t detail)
{
    flexe_camera_result[7] = (uint32_t)detail;
    flexe_camera_stage = UINT32_C(0xBAD00000) | stage;
    printf("CAMERA_FAIL stage=%u detail=%ld\n",
           (unsigned)stage, (long)detail);
    fflush(stdout);
    for (;;) vTaskDelay(pdMS_TO_TICKS(100));
}

static uint8_t expected_byte(size_t index)
{
    return (uint8_t)(((index * 37u + 11u) ^ (index >> 5u)) & 0xFFu);
}

static uint32_t frame_checksum(const uint8_t *data, size_t length)
{
    uint32_t digest = UINT32_C(2166136261);
    for (size_t index = 0u; index < length; index++)
        digest = (digest ^ data[index]) * UINT32_C(16777619);
    return digest;
}

void app_main(void)
{
    flexe_camera_stage = 1u;
    const camera_config_t config = {
        .pin_pwdn = -1,
        .pin_reset = -1,
        .pin_xclk = 15,
        .pin_sccb_sda = 4,
        .pin_sccb_scl = 5,
        .pin_d7 = 13,
        .pin_d6 = 12,
        .pin_d5 = 11,
        .pin_d4 = 10,
        .pin_d3 = 9,
        .pin_d2 = 8,
        .pin_d1 = 7,
        .pin_d0 = 6,
        .pin_vsync = 14,
        .pin_href = 16,
        .pin_pclk = 17,
        .xclk_freq_hz = 20000000,
        .ledc_timer = LEDC_TIMER_0,
        .ledc_channel = LEDC_CHANNEL_0,
        .pixel_format = PIXFORMAT_RGB565,
        .frame_size = FRAMESIZE_QQVGA,
        .jpeg_quality = 12,
        .fb_count = 1,
        .fb_location = CAMERA_FB_IN_DRAM,
        .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
        .sccb_i2c_port = 1,
    };
    esp_err_t error = esp_camera_init(&config);
    flexe_camera_result[0] = (uint32_t)error;
    if (error != ESP_OK) fail(2u, error);

    sensor_t *sensor = esp_camera_sensor_get();
    if (!sensor || sensor->id.PID != OV2640_PID)
        fail(3u, sensor ? (int32_t)sensor->id.PID : -1);
    flexe_camera_result[1] = sensor->id.PID;
    flexe_camera_stage = 4u;

    camera_fb_t *frame = esp_camera_fb_get();
    if (!frame) fail(5u, -1);
    flexe_camera_result[2] = (uint32_t)frame->len;
    flexe_camera_result[3] = (uint32_t)frame->width;
    flexe_camera_result[4] = (uint32_t)frame->height;
    flexe_camera_result[5] = frame_checksum(frame->buf, frame->len);
    if (frame->len != FRAME_BYTES || frame->width != 160u ||
        frame->height != 120u || frame->format != PIXFORMAT_RGB565)
        fail(6u, (int32_t)frame->len);
    for (size_t index = 0u; index < frame->len; index++) {
        if (frame->buf[index] != expected_byte(index))
            fail(7u, (int32_t)index);
    }
    esp_camera_fb_return(frame);
    error = esp_camera_deinit();
    flexe_camera_result[6] = (uint32_t)error;
    if (error != ESP_OK) fail(8u, error);

    flexe_camera_stage = CAMERA_DONE;
    printf("CAMERA_DONE sensor=OV2640 size=160x120 bytes=%u fnv=%08lX teardown=ok\n",
           (unsigned)FRAME_BYTES, (unsigned long)flexe_camera_result[5]);
    fflush(stdout);
}
