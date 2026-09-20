/* Exercise ESP-IDF 5.3's stock pulse-counter driver through GPIO-matrix
 * loopback, including APB glitch filtering, watch-point interrupts, limits,
 * direction control, and complete driver teardown. */
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "driver/gpio.h"
#include "driver/pulse_cnt.h"
#include "esp_err.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define PCNT_DONE UINT32_C(0x50434E54)

volatile uint32_t flexe_pcnt_stage;
volatile uint32_t flexe_pcnt_callback_count;
volatile int32_t flexe_pcnt_events[8];
volatile uint32_t flexe_pcnt_result[16];

static void fail(uint32_t stage, int32_t detail)
{
    flexe_pcnt_result[15] = (uint32_t)detail;
    flexe_pcnt_stage = UINT32_C(0xBAD00000) | stage;
    printf("PCNT_FAIL stage=%u detail=%ld\n",
           (unsigned)stage, (long)detail);
    fflush(stdout);
    for (;;) vTaskDelay(pdMS_TO_TICKS(100));
}

static bool on_reach(pcnt_unit_handle_t unit,
                     const pcnt_watch_event_data_t *event, void *context)
{
    (void)unit;
    (void)context;
    uint32_t index = flexe_pcnt_callback_count;
    if (index < 8u) flexe_pcnt_events[index] = event->watch_point_value;
    flexe_pcnt_callback_count = index + 1u;
    return false;
}

static void require_ok(uint32_t stage, esp_err_t error)
{
    if (error != ESP_OK) fail(stage, error);
}

static void pulse(unsigned count)
{
    for (unsigned index = 0u; index < count; index++) {
        require_ok(20u, gpio_set_level(GPIO_NUM_4, 1));
        esp_rom_delay_us(20u);
        require_ok(21u, gpio_set_level(GPIO_NUM_4, 0));
        esp_rom_delay_us(20u);
    }
}

static void wait_callbacks(uint32_t expected)
{
    for (unsigned waited = 0u;
         waited < 1000u && flexe_pcnt_callback_count < expected; waited++)
        vTaskDelay(pdMS_TO_TICKS(1));
    if (flexe_pcnt_callback_count != expected)
        fail(22u, (int32_t)flexe_pcnt_callback_count);
}

static int read_count(pcnt_unit_handle_t unit, uint32_t stage)
{
    int value = INT32_MIN;
    require_ok(stage, pcnt_unit_get_count(unit, &value));
    return value;
}

void app_main(void)
{
    flexe_pcnt_stage = 1u;
    pcnt_unit_handle_t unit = NULL;
    pcnt_channel_handle_t channel = NULL;
    pcnt_unit_config_t unit_config = {
        .low_limit = -4,
        .high_limit = 4,
    };
    require_ok(1u, pcnt_new_unit(&unit_config, &unit));

    pcnt_chan_config_t channel_config = {
        .edge_gpio_num = GPIO_NUM_4,
        .level_gpio_num = GPIO_NUM_5,
        .flags.io_loop_back = 1u,
    };
    require_ok(2u, pcnt_new_channel(unit, &channel_config, &channel));
    require_ok(3u, pcnt_channel_set_edge_action(
        channel, PCNT_CHANNEL_EDGE_ACTION_INCREASE,
        PCNT_CHANNEL_EDGE_ACTION_HOLD));
    require_ok(4u, pcnt_channel_set_level_action(
        channel, PCNT_CHANNEL_LEVEL_ACTION_INVERSE,
        PCNT_CHANNEL_LEVEL_ACTION_KEEP));

    pcnt_glitch_filter_config_t filter = {
        .max_glitch_ns = 1000u,
    };
    require_ok(5u, pcnt_unit_set_glitch_filter(unit, &filter));
    require_ok(6u, pcnt_unit_add_watch_point(unit, 2));
    require_ok(7u, pcnt_unit_add_watch_point(unit, -2));
    require_ok(8u, pcnt_unit_add_watch_point(unit, 0));
    require_ok(9u, pcnt_unit_add_watch_point(unit, 4));
    pcnt_event_callbacks_t callbacks = {
        .on_reach = on_reach,
    };
    require_ok(10u, pcnt_unit_register_event_callbacks(
        unit, &callbacks, NULL));

    require_ok(11u, gpio_set_level(GPIO_NUM_4, 0));
    require_ok(12u, gpio_set_level(GPIO_NUM_5, 0));
    require_ok(13u, pcnt_unit_enable(unit));
    require_ok(14u, pcnt_unit_start(unit));

    flexe_pcnt_stage = 2u;
    pulse(2u);
    wait_callbacks(1u);
    flexe_pcnt_result[0] = (uint32_t)read_count(unit, 23u);
    if ((int32_t)flexe_pcnt_result[0] != 2) fail(24u, flexe_pcnt_result[0]);

    pulse(2u);
    wait_callbacks(2u);
    flexe_pcnt_result[1] = (uint32_t)read_count(unit, 25u);
    if ((int32_t)flexe_pcnt_result[1] != 0) fail(26u, flexe_pcnt_result[1]);

    require_ok(27u, gpio_set_level(GPIO_NUM_5, 1));
    pulse(2u);
    wait_callbacks(3u);
    flexe_pcnt_result[2] = (uint32_t)read_count(unit, 28u);
    if ((int32_t)flexe_pcnt_result[2] != -2) fail(29u, flexe_pcnt_result[2]);

    require_ok(30u, gpio_set_level(GPIO_NUM_5, 0));
    pulse(2u);
    wait_callbacks(4u);
    flexe_pcnt_result[3] = (uint32_t)read_count(unit, 31u);
    if ((int32_t)flexe_pcnt_result[3] != 0) fail(32u, flexe_pcnt_result[3]);

    static const int32_t expected[] = { 2, 4, -2, 0 };
    for (unsigned index = 0u; index < 4u; index++) {
        flexe_pcnt_result[4u + index] = (uint32_t)flexe_pcnt_events[index];
        if (flexe_pcnt_events[index] != expected[index])
            fail(33u + index, flexe_pcnt_events[index]);
    }

    require_ok(40u, pcnt_unit_stop(unit));
    require_ok(41u, pcnt_unit_disable(unit));
    require_ok(42u, pcnt_unit_remove_watch_point(unit, 4));
    require_ok(43u, pcnt_unit_remove_watch_point(unit, 0));
    require_ok(44u, pcnt_unit_remove_watch_point(unit, -2));
    require_ok(45u, pcnt_unit_remove_watch_point(unit, 2));
    require_ok(46u, pcnt_del_channel(channel));
    require_ok(47u, pcnt_del_unit(unit));

    flexe_pcnt_result[8] = flexe_pcnt_callback_count;
    flexe_pcnt_result[15] = 0u;
    flexe_pcnt_stage = PCNT_DONE;
    printf("PCNT_DONE count=0 callbacks=%u events=%ld,%ld,%ld,%ld\n",
           (unsigned)flexe_pcnt_callback_count,
           (long)flexe_pcnt_events[0], (long)flexe_pcnt_events[1],
           (long)flexe_pcnt_events[2], (long)flexe_pcnt_events[3]);
    fflush(stdout);
    for (;;) vTaskDelay(pdMS_TO_TICKS(100));
}
