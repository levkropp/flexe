#include "test_helpers.h"
#include "sandbox_input.h"

#include <string.h>

TEST(sandbox_input_parses_existing_physical_inputs)
{
    sbx_input_event_t event;

    ASSERT_TRUE(sbx_input_parse(
        "{\"lvl\":1,\"t\":\"gpio_in\",\"pin\":48}", &event));
    ASSERT_EQ(event.kind, SBX_INPUT_GPIO);
    ASSERT_EQ(event.gpio.pin, 48);
    ASSERT_EQ(event.gpio.level, 1);

    ASSERT_TRUE(sbx_input_parse(
        "{ \"t\" : \"touch_in\", \"x\" : -4, \"y\": 91, \"pressed\":0 }",
        &event));
    ASSERT_EQ(event.kind, SBX_INPUT_TOUCH);
    ASSERT_EQ(event.touch.x, (uint32_t)-4);
    ASSERT_EQ(event.touch.y, 91);
    ASSERT_EQ(event.touch.pressed, 0);

    ASSERT_TRUE(sbx_input_parse(
        "{\"t\":\"adc_in\",\"ch\":10,\"raw\":65535}", &event));
    ASSERT_EQ(event.kind, SBX_INPUT_ADC);
    ASSERT_EQ(event.adc.channel, 10);
    ASSERT_EQ(event.adc.raw, 65535);
}

TEST(sandbox_input_parses_binary_uart_and_break_events)
{
    sbx_input_event_t event;

    ASSERT_TRUE(sbx_input_parse(
        "{\"t\":\"uart_in\",\"u\":0,\"b\":10}", &event));
    ASSERT_EQ(event.kind, SBX_INPUT_UART);
    ASSERT_EQ(event.uart.port, 0);
    ASSERT_EQ(event.uart.len, 1);
    ASSERT_EQ(event.uart.data[0], 10);

    ASSERT_TRUE(sbx_input_parse(
        "{\"hex\":\"0068656C700AFF\",\"port\":2,\"t\":\"uart_in\"}",
        &event));
    static const uint8_t expected[] = {0x00, 'h', 'e', 'l', 'p', '\n', 0xFF};
    ASSERT_EQ(event.kind, SBX_INPUT_UART);
    ASSERT_EQ(event.uart.port, 2);
    ASSERT_EQ(event.uart.len, sizeof(expected));
    ASSERT_TRUE(memcmp(event.uart.data, expected, sizeof(expected)) == 0);

    ASSERT_TRUE(sbx_input_parse(
        "{\"t\":\"uart_break\",\"port\":1}", &event));
    ASSERT_EQ(event.kind, SBX_INPUT_UART_BREAK);
    ASSERT_EQ(event.uart_break.port, 1);
}

TEST(sandbox_input_rejects_ambiguous_or_malformed_events)
{
    sbx_input_event_t event;
    ASSERT_FALSE(sbx_input_parse(
        "{\"t\":\"not_gpio_in\",\"pin\":1,\"lvl\":1}", &event));
    ASSERT_FALSE(sbx_input_parse(
        "{\"t\":\"gpio_in\",\"pin\":1,\"lvl\":2}", &event));
    ASSERT_FALSE(sbx_input_parse(
        "{\"t\":\"adc_in\",\"ch\":1,\"raw\":65536}", &event));
    ASSERT_FALSE(sbx_input_parse(
        "{\"t\":\"uart_in\",\"u\":0,\"b\":256}", &event));
    ASSERT_FALSE(sbx_input_parse(
        "{\"t\":\"uart_in\",\"u\":0,\"hex\":\"123\"}", &event));
    ASSERT_FALSE(sbx_input_parse(
        "{\"t\":\"uart_in\",\"u\":0,\"hex\":\"xz\"}", &event));
    ASSERT_FALSE(sbx_input_parse(
        "{\"t\":\"uart_in\",\"u\":-1,\"b\":1}", &event));
}

static void run_sandbox_input_tests(void)
{
    TEST_SUITE("Sandbox host input");
    RUN_TEST(sandbox_input_parses_existing_physical_inputs);
    RUN_TEST(sandbox_input_parses_binary_uart_and_break_events);
    RUN_TEST(sandbox_input_rejects_ambiguous_or_malformed_events);
}
