/* Exercise ESP-IDF 5.3's public MCPWM driver across both S3 groups: timer
 * and compare interrupts, GPIO-matrix generator output, live compare updates,
 * force levels, capture input, and complete resource teardown. */
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "driver/gpio.h"
#include "driver/mcpwm_prelude.h"
#include "esp_err.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define MCPWM_DONE UINT32_C(0x4D435057)
#define PWM_PERIOD_TICKS 1000u

typedef struct {
    mcpwm_timer_handle_t timer;
    mcpwm_oper_handle_t oper;
    mcpwm_cmpr_handle_t comparator;
    mcpwm_gen_handle_t generator;
    uint32_t group;
    uint32_t gpio;
} pwm_channel_t;

volatile uint32_t flexe_mcpwm_stage;
volatile uint32_t flexe_mcpwm_timer_callbacks[2];
volatile uint32_t flexe_mcpwm_compare_callbacks[2];
volatile uint32_t flexe_mcpwm_last_compare[2];
volatile uint32_t flexe_mcpwm_capture_callbacks;
volatile uint32_t flexe_mcpwm_capture_value[2];
volatile uint32_t flexe_mcpwm_capture_edge[2];
volatile uint32_t flexe_mcpwm_result[16];

static void fail(uint32_t stage, int32_t detail)
{
    flexe_mcpwm_result[15] = (uint32_t)detail;
    flexe_mcpwm_stage = UINT32_C(0xBAD00000) | stage;
    printf("MCPWM_FAIL stage=%u detail=%ld\n",
           (unsigned)stage, (long)detail);
    fflush(stdout);
    for (;;) vTaskDelay(pdMS_TO_TICKS(100));
}

static void require_ok(uint32_t stage, esp_err_t error)
{
    if (error != ESP_OK) fail(stage, error);
}

static bool on_timer_empty(mcpwm_timer_handle_t timer,
                           const mcpwm_timer_event_data_t *event,
                           void *context)
{
    (void)timer;
    (void)event;
    uint32_t group = (uint32_t)(uintptr_t)context;
    flexe_mcpwm_timer_callbacks[group]++;
    return false;
}

static bool on_compare(mcpwm_cmpr_handle_t comparator,
                       const mcpwm_compare_event_data_t *event,
                       void *context)
{
    (void)comparator;
    uint32_t group = (uint32_t)(uintptr_t)context;
    flexe_mcpwm_last_compare[group] = event->compare_ticks;
    flexe_mcpwm_compare_callbacks[group]++;
    return false;
}

static bool on_capture(mcpwm_cap_channel_handle_t channel,
                       const mcpwm_capture_event_data_t *event,
                       void *context)
{
    (void)channel;
    (void)context;
    uint32_t index = flexe_mcpwm_capture_callbacks;
    if (index < 2u) {
        flexe_mcpwm_capture_value[index] = event->cap_value;
        flexe_mcpwm_capture_edge[index] = event->cap_edge;
    }
    flexe_mcpwm_capture_callbacks = index + 1u;
    return false;
}

static void wait_pwm_callbacks(uint32_t stage, uint32_t minimum)
{
    for (uint32_t waited = 0u; waited < 500u; waited++) {
        if (flexe_mcpwm_timer_callbacks[0] >= minimum &&
            flexe_mcpwm_timer_callbacks[1] >= minimum &&
            flexe_mcpwm_compare_callbacks[0] >= minimum &&
            flexe_mcpwm_compare_callbacks[1] >= minimum)
            return;
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    fail(stage, (int32_t)(
        (flexe_mcpwm_timer_callbacks[0] & 0xFFu) |
        ((flexe_mcpwm_timer_callbacks[1] & 0xFFu) << 8u) |
        ((flexe_mcpwm_compare_callbacks[0] & 0xFFu) << 16u) |
        ((flexe_mcpwm_compare_callbacks[1] & 0xFFu) << 24u)));
}

static void create_pwm_channel(pwm_channel_t *channel, uint32_t group,
                               uint32_t gpio, uint32_t compare,
                               uint32_t stage)
{
    channel->group = group;
    channel->gpio = gpio;

    mcpwm_timer_config_t timer_config = {
        .group_id = (int)group,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = 1000000u,
        .period_ticks = PWM_PERIOD_TICKS,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
    };
    require_ok(stage + 0u,
               mcpwm_new_timer(&timer_config, &channel->timer));

    mcpwm_operator_config_t operator_config = {
        .group_id = (int)group,
    };
    require_ok(stage + 1u,
               mcpwm_new_operator(&operator_config, &channel->oper));
    require_ok(stage + 2u,
               mcpwm_operator_connect_timer(channel->oper, channel->timer));

    mcpwm_comparator_config_t comparator_config = {
        .flags.update_cmp_on_tez = true,
    };
    require_ok(stage + 3u, mcpwm_new_comparator(
        channel->oper, &comparator_config, &channel->comparator));
    require_ok(stage + 4u, mcpwm_comparator_set_compare_value(
        channel->comparator, compare));

    mcpwm_generator_config_t generator_config = {
        .gen_gpio_num = (int)gpio,
        .flags.io_loop_back = true,
    };
    require_ok(stage + 5u, mcpwm_new_generator(
        channel->oper, &generator_config, &channel->generator));
    require_ok(stage + 6u, mcpwm_generator_set_action_on_timer_event(
        channel->generator,
        MCPWM_GEN_TIMER_EVENT_ACTION(
            MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY,
            MCPWM_GEN_ACTION_HIGH)));
    require_ok(stage + 7u, mcpwm_generator_set_action_on_compare_event(
        channel->generator,
        MCPWM_GEN_COMPARE_EVENT_ACTION(
            MCPWM_TIMER_DIRECTION_UP, channel->comparator,
            MCPWM_GEN_ACTION_LOW)));

    mcpwm_timer_event_callbacks_t timer_callbacks = {
        .on_empty = on_timer_empty,
    };
    require_ok(stage + 8u, mcpwm_timer_register_event_callbacks(
        channel->timer, &timer_callbacks, (void *)(uintptr_t)group));
    mcpwm_comparator_event_callbacks_t comparator_callbacks = {
        .on_reach = on_compare,
    };
    require_ok(stage + 9u, mcpwm_comparator_register_event_callbacks(
        channel->comparator, &comparator_callbacks,
        (void *)(uintptr_t)group));
    require_ok(stage + 10u, mcpwm_timer_enable(channel->timer));
    require_ok(stage + 11u, mcpwm_timer_start_stop(
        channel->timer, MCPWM_TIMER_START_NO_STOP));
}

static void destroy_pwm_channel(pwm_channel_t *channel, uint32_t stage)
{
    require_ok(stage + 0u, mcpwm_timer_start_stop(
        channel->timer, MCPWM_TIMER_STOP_EMPTY));
    vTaskDelay(pdMS_TO_TICKS(3));
    require_ok(stage + 1u, mcpwm_timer_disable(channel->timer));
    require_ok(stage + 2u, mcpwm_del_generator(channel->generator));
    require_ok(stage + 3u, mcpwm_del_comparator(channel->comparator));
    require_ok(stage + 4u, mcpwm_del_operator(channel->oper));
    require_ok(stage + 5u, mcpwm_del_timer(channel->timer));
}

void app_main(void)
{
    pwm_channel_t pwm[2] = {0};
    flexe_mcpwm_stage = 1u;
    create_pwm_channel(&pwm[0], 0u, GPIO_NUM_4, 250u, 10u);
    create_pwm_channel(&pwm[1], 1u, GPIO_NUM_5, 750u, 30u);
    wait_pwm_callbacks(50u, 4u);
    if (flexe_mcpwm_last_compare[0] != 250u)
        fail(51u, flexe_mcpwm_last_compare[0]);
    if (flexe_mcpwm_last_compare[1] != 750u)
        fail(52u, flexe_mcpwm_last_compare[1]);

    /* Continuous force goes through each group's real generator and GPIO
     * matrix route, while GPIO loopback lets the public GPIO API sample it. */
    require_ok(53u, mcpwm_generator_set_force_level(
        pwm[0].generator, 0, true));
    require_ok(54u, mcpwm_generator_set_force_level(
        pwm[1].generator, 1, true));
    esp_rom_delay_us(50u);
    if (gpio_get_level(GPIO_NUM_4) != 0) fail(55u, gpio_get_level(GPIO_NUM_4));
    if (gpio_get_level(GPIO_NUM_5) != 1) fail(56u, gpio_get_level(GPIO_NUM_5));
    require_ok(57u, mcpwm_generator_set_force_level(
        pwm[0].generator, 1, true));
    require_ok(58u, mcpwm_generator_set_force_level(
        pwm[1].generator, 0, true));
    esp_rom_delay_us(50u);
    if (gpio_get_level(GPIO_NUM_4) != 1) fail(59u, gpio_get_level(GPIO_NUM_4));
    if (gpio_get_level(GPIO_NUM_5) != 0) fail(60u, gpio_get_level(GPIO_NUM_5));
    require_ok(61u, mcpwm_generator_set_force_level(
        pwm[0].generator, -1, true));
    require_ok(62u, mcpwm_generator_set_force_level(
        pwm[1].generator, -1, true));

    flexe_mcpwm_compare_callbacks[0] = 0u;
    flexe_mcpwm_compare_callbacks[1] = 0u;
    require_ok(63u, mcpwm_comparator_set_compare_value(
        pwm[0].comparator, 400u));
    require_ok(64u, mcpwm_comparator_set_compare_value(
        pwm[1].comparator, 600u));
    wait_pwm_callbacks(65u, 3u);
    if (flexe_mcpwm_last_compare[0] != 400u)
        fail(66u, flexe_mcpwm_last_compare[0]);
    if (flexe_mcpwm_last_compare[1] != 600u)
        fail(67u, flexe_mcpwm_last_compare[1]);

    /* A stock capture channel receives real GPIO-matrix loopback edges and
     * reports them through the shared group-0 interrupt source. */
    mcpwm_cap_timer_handle_t cap_timer = NULL;
    mcpwm_cap_channel_handle_t cap_channel = NULL;
    mcpwm_capture_timer_config_t cap_timer_config = {
        .group_id = 0,
        .clk_src = MCPWM_CAPTURE_CLK_SRC_DEFAULT,
    };
    require_ok(70u, mcpwm_new_capture_timer(
        &cap_timer_config, &cap_timer));
    require_ok(71u, gpio_set_level(GPIO_NUM_6, 0));
    mcpwm_capture_channel_config_t cap_channel_config = {
        .gpio_num = GPIO_NUM_6,
        .prescale = 1u,
        .flags.pos_edge = true,
        .flags.neg_edge = true,
        .flags.io_loop_back = true,
    };
    require_ok(72u, mcpwm_new_capture_channel(
        cap_timer, &cap_channel_config, &cap_channel));
    mcpwm_capture_event_callbacks_t capture_callbacks = {
        .on_cap = on_capture,
    };
    require_ok(73u, mcpwm_capture_channel_register_event_callbacks(
        cap_channel, &capture_callbacks, NULL));
    require_ok(74u, mcpwm_capture_channel_enable(cap_channel));
    require_ok(75u, mcpwm_capture_timer_enable(cap_timer));
    require_ok(76u, mcpwm_capture_timer_start(cap_timer));
    esp_rom_delay_us(100u);
    require_ok(77u, gpio_set_level(GPIO_NUM_6, 1));
    esp_rom_delay_us(100u);
    require_ok(78u, gpio_set_level(GPIO_NUM_6, 0));
    for (uint32_t waited = 0u;
         waited < 100u && flexe_mcpwm_capture_callbacks < 2u; waited++)
        vTaskDelay(pdMS_TO_TICKS(1));
    if (flexe_mcpwm_capture_callbacks != 2u)
        fail(79u, flexe_mcpwm_capture_callbacks);
    if (flexe_mcpwm_capture_edge[0] != MCPWM_CAP_EDGE_POS)
        fail(80u, flexe_mcpwm_capture_edge[0]);
    if (flexe_mcpwm_capture_edge[1] != MCPWM_CAP_EDGE_NEG)
        fail(81u, flexe_mcpwm_capture_edge[1]);
    if (flexe_mcpwm_capture_value[1] <= flexe_mcpwm_capture_value[0])
        fail(82u, flexe_mcpwm_capture_value[1] -
                  flexe_mcpwm_capture_value[0]);

    require_ok(83u, mcpwm_capture_timer_stop(cap_timer));
    require_ok(84u, mcpwm_capture_channel_disable(cap_channel));
    require_ok(85u, mcpwm_capture_timer_disable(cap_timer));
    require_ok(86u, mcpwm_del_capture_channel(cap_channel));
    require_ok(87u, mcpwm_del_capture_timer(cap_timer));
    destroy_pwm_channel(&pwm[1], 90u);
    destroy_pwm_channel(&pwm[0], 100u);

    flexe_mcpwm_result[0] = flexe_mcpwm_timer_callbacks[0];
    flexe_mcpwm_result[1] = flexe_mcpwm_timer_callbacks[1];
    flexe_mcpwm_result[2] = flexe_mcpwm_capture_callbacks;
    flexe_mcpwm_result[15] = 0u;
    flexe_mcpwm_stage = MCPWM_DONE;
    printf("MCPWM_DONE groups=2 timer=ok compare=250,750->400,600 force=ok capture=pos,neg teardown=ok\n");
    fflush(stdout);
    for (;;) vTaskDelay(pdMS_TO_TICKS(100));
}
