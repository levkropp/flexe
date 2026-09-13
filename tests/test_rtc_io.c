/* S3 RTC GPIO pad ownership and shared physical-pin state. */
#include "test_helpers.h"
#include "peripherals.h"
#include "rtc_io.h"

TEST(rtc_io_s3_mux_selects_rtc_or_digital_output)
{
    const flexe_target_desc_t *s3 =
        flexe_target_by_id(FLEXE_TARGET_ESP32S3);
    const flexe_rtc_io_desc_t *rtc = &s3->rtc_io;
    xtensa_mem_t *mem = mem_create_for_target(s3);
    esp32_periph_t *periph = mem ? periph_create(mem) : NULL;
    ASSERT_TRUE(periph != NULL);
    if (!periph) {
        mem_destroy(mem);
        return;
    }
    uint32_t pad4 = rtc->base + rtc->pad_base_offset + 4u * 4u;
    uint32_t pad11 = rtc->base + rtc->pad_base_offset + 4u * 11u;
    uint32_t pad21 = rtc->base + rtc->pad_base_offset + 4u * 21u;
    uint32_t bit4 = 1u << (rtc->data_shift + 4u);
    uint32_t bit11 = 1u << (rtc->data_shift + 11u);
    uint32_t bit21 = 1u << (rtc->data_shift + 21u);
    uint32_t gpio = s3->gpio.base;

    ASSERT_EQ(rtc->gpio_count, 22u);
    ASSERT_EQ(mem_read32(mem, pad4), 0x50000000u);
    ASSERT_EQ(mem_read32(mem, pad11), 0x40000000u);
    ASSERT_EQ(mem_read32(mem, rtc->base + 0x000u), 0u);
    ASSERT_EQ(mem_read32(mem, rtc->base + 0x00Cu), 0u);

    mem_write32(mem, gpio + 0x008u, 1u << 4u);
    mem_write32(mem, gpio + 0x024u, 1u << 4u);
    ASSERT_EQ(periph_gpio_pin_level(periph, 4), 1);
    ASSERT_EQ(periph_gpio_output_enabled(periph, 4), 1);

    mem_write32(mem, rtc->base + 0x010u, bit4);
    mem_write32(mem, pad4, rtc->pad_reset[4] | rtc->pad_mux_mask);
    ASSERT_EQ(periph_gpio_pin_level(periph, 4), 0);
    ASSERT_EQ(periph_gpio_output_enabled(periph, 4), 1);
    ASSERT_EQ(mem_read32(mem, pad4),
              rtc->pad_reset[4] | rtc->pad_mux_mask);
    mem_write32(mem, rtc->base + 0x004u, bit4);
    ASSERT_EQ(periph_gpio_pin_level(periph, 4), 1);
    mem_write32(mem, gpio + 0x00Cu, 1u << 4u);
    ASSERT_EQ(periph_gpio_pin_level(periph, 4), 1);
    mem_write32(mem, rtc->base + 0x008u, bit4);
    ASSERT_EQ(periph_gpio_pin_level(periph, 4), 0);
    ASSERT_EQ(mem_read32(mem, rtc->base + 0x000u) & bit4, 0u);
    ASSERT_EQ(mem_read32(mem, rtc->base + 0x00Cu) & bit4, bit4);
    ASSERT_EQ(mem_read32(mem, rtc->base + 0x004u), 0u);

    mem_write32(mem, pad4, rtc->pad_reset[4]);
    ASSERT_EQ(periph_gpio_pin_level(periph, 4), 0);
    ASSERT_EQ(periph_gpio_output_enabled(periph, 4), 1);
    mem_write32(mem, gpio + 0x008u, 1u << 4u);
    ASSERT_EQ(periph_gpio_pin_level(periph, 4), 1);

    periph_gpio_set_input(periph, 11, 1);
    ASSERT_EQ(mem_read32(mem, rtc->base + 0x024u) & bit11, 0u);
    mem_write32(mem, pad11,
                rtc->pad_reset[11] | rtc->pad_mux_mask | (1u << 13));
    ASSERT_EQ(mem_read32(mem, rtc->base + 0x024u) & bit11, bit11);
    ASSERT_EQ(mem_read32(mem, gpio + 0x03Cu) & (1u << 11), 0u);
    mem_write32(mem, pad11, rtc->pad_reset[11]);
    ASSERT_EQ(mem_read32(mem, gpio + 0x03Cu) & (1u << 11), 1u << 11);
    periph_gpio_set_input(periph, 11, 0);
    ASSERT_EQ(mem_read32(mem, rtc->base + 0x024u) & bit11, 0u);
    /* The highest RTC GPIO lives in bit 31 of the shifted register bank. */
    mem_write32(mem, rtc->base + 0x000u, bit21);
    mem_write32(mem, rtc->base + 0x00Cu, bit21);
    mem_write32(mem, pad21, rtc->pad_reset[21] | rtc->pad_mux_mask);
    ASSERT_EQ(periph_gpio_pin_level(periph, 21), 1);
    ASSERT_EQ(periph_gpio_output_enabled(periph, 21), 1);
    mem_write32(mem, rtc->base + 0x014u, bit21);
    ASSERT_EQ(periph_gpio_output_enabled(periph, 21), 0);
    ASSERT_EQ(periph_unhandled_count(periph), 0u);
    ASSERT_EQ(mem_unmapped_count(mem), 0u);

    periph_destroy(periph);
    mem_destroy(mem);
}

TEST(rtc_io_s3_unknown_pad_function_stays_diagnostic)
{
    const flexe_target_desc_t *s3 =
        flexe_target_by_id(FLEXE_TARGET_ESP32S3);
    const flexe_rtc_io_desc_t *rtc = &s3->rtc_io;
    xtensa_mem_t *mem = mem_create_for_target(s3);
    esp32_periph_t *periph = mem ? periph_create(mem) : NULL;
    ASSERT_TRUE(periph != NULL);
    if (!periph) {
        mem_destroy(mem);
        return;
    }
    uint32_t pad11 = rtc->base + rtc->pad_base_offset + 4u * 11u;
    uint32_t bit11 = 1u << (rtc->data_shift + 11u);
    mem_write32(mem, rtc->base + 0x004u, bit11);
    mem_write32(mem, rtc->base + 0x010u, bit11);
    mem_write32(mem, pad11, rtc->pad_reset[11] | rtc->pad_mux_mask);
    ASSERT_EQ(periph_gpio_pin_level(periph, 11), 1);
    ASSERT_EQ(periph_gpio_output_enabled(periph, 11), 1);

    int before = periph_unhandled_count(periph);
    mem_write32(mem, pad11,
                rtc->pad_reset[11] | rtc->pad_mux_mask | (1u << 17));
    ASSERT_EQ(mem_read32(mem, pad11),
              rtc->pad_reset[11] | rtc->pad_mux_mask | (1u << 17));
    ASSERT_EQ(periph_gpio_pin_level(periph, 11), -1);
    ASSERT_EQ(periph_gpio_output_enabled(periph, 11), -1);
    ASSERT_EQ(periph_unhandled_count(periph), before + 1);

    mem_write32(mem, pad11, rtc->pad_reset[11] | rtc->pad_mux_mask);
    ASSERT_EQ(periph_gpio_pin_level(periph, 11), 1);
    ASSERT_EQ(periph_unhandled_count(periph), before + 2);
    ASSERT_EQ(mem_read32(mem, rtc->base + 0x100u), 0u);
    mem_write32(mem, rtc->base + 0x100u, 1u);
    ASSERT_EQ(periph_unhandled_count(periph), before + 4);

    periph_destroy(periph);
    mem_destroy(mem);
}

TEST(rtc_io_rejects_target_without_capability)
{
    const flexe_target_desc_t *classic =
        flexe_target_by_id(FLEXE_TARGET_ESP32);
    xtensa_mem_t *mem = mem_create_for_target(classic);
    ASSERT_TRUE(mem != NULL);
    flexe_gpio_t *gpio = flexe_gpio_create(
        mem, NULL, NULL, NULL, NULL, NULL, NULL, NULL);
    ASSERT_TRUE(gpio == NULL);
    ASSERT_TRUE(flexe_rtc_io_create(
        mem, gpio, NULL, NULL, NULL) == NULL);
    mem_destroy(mem);
}

TEST(rtc_io_s3_mux_blocks_digital_matrix_input)
{
    const flexe_target_desc_t *s3 =
        flexe_target_by_id(FLEXE_TARGET_ESP32S3);
    xtensa_mem_t *mem = mem_create_for_target(s3);
    flexe_gpio_t *gpio = mem ? flexe_gpio_create(
        mem, NULL, NULL, NULL, NULL, NULL, NULL, NULL) : NULL;
    flexe_rtc_io_t *rtc = gpio ? flexe_rtc_io_create(
        mem, gpio, NULL, NULL, NULL) : NULL;
    ASSERT_TRUE(rtc != NULL);
    if (!rtc) {
        flexe_gpio_destroy(gpio);
        mem_destroy(mem);
        return;
    }

    uint32_t signal = s3->gpio.base + 0x154u + 42u * 4u;
    uint32_t pad4 = s3->rtc_io.base +
                    s3->rtc_io.pad_base_offset + 4u * 4u;
    flexe_gpio_set_input(gpio, 4u, true);
    mem_write32(mem, signal, (1u << 7) | 4u);
    ASSERT_EQ(flexe_gpio_input_signal_level(gpio, 42u), 1);
    mem_write32(mem, pad4,
                s3->rtc_io.pad_reset[4] |
                s3->rtc_io.pad_mux_mask | (1u << 13));
    ASSERT_EQ(flexe_gpio_input_signal_level(gpio, 42u), -1);
    ASSERT_EQ(mem_read32(mem, s3->rtc_io.base + 0x024u) &
              (1u << (s3->rtc_io.data_shift + 4u)),
              1u << (s3->rtc_io.data_shift + 4u));
    mem_write32(mem, pad4, s3->rtc_io.pad_reset[4]);
    ASSERT_EQ(flexe_gpio_input_signal_level(gpio, 42u), 1);

    flexe_rtc_io_destroy(rtc);
    flexe_gpio_destroy(gpio);
    mem_destroy(mem);
}

void run_rtc_io_tests(void)
{
    TEST_SUITE("Target RTC IO");
    RUN_TEST(rtc_io_s3_mux_selects_rtc_or_digital_output);
    RUN_TEST(rtc_io_s3_unknown_pad_function_stays_diagnostic);
    RUN_TEST(rtc_io_rejects_target_without_capability);
    RUN_TEST(rtc_io_s3_mux_blocks_digital_matrix_input);
}
