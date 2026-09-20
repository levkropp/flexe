/* ESP32-S3 descriptor-driven capacitive-touch v2 controller tests. */
#include "test_helpers.h"
#include "peripherals.h"
#include "target.h"
#include "touch_v2.h"

static uint32_t touch_field(uint32_t value, uint32_t mask,
                            unsigned shift)
{
    return (value & mask) >> shift;
}

TEST(touch_v2_scans_host_samples_and_routes_rtc_interrupts)
{
    const flexe_target_desc_t *s3 =
        flexe_target_by_id(FLEXE_TARGET_ESP32S3);
    const flexe_touch_v2_desc_t *touch =
        flexe_touch_v2_descriptor(s3);
    const flexe_rtc_cntl_desc_t *rtc = &s3->rtc_cntl;
    const flexe_sens_desc_t *sens = &s3->sens;
    xtensa_mem_t *mem = mem_create_for_target(s3);
    esp32_periph_t *periph = mem ? periph_create(mem) : NULL;
    ASSERT_TRUE(touch != NULL);
    ASSERT_TRUE(periph != NULL);
    if (!touch || !periph) {
        periph_destroy(periph);
        mem_destroy(mem);
        return;
    }

    const unsigned channel = 4u;
    const uint32_t channel_bit = 1u << channel;
    uint32_t threshold_addr = sens->base +
        touch->sens_threshold_base_offset +
        4u * (channel - touch->first_external_channel);
    uint32_t status_addr = sens->base + touch->sens_status_base_offset +
                           4u * channel;
    uint32_t config_addr = sens->base + touch->sens_config_offset;
    uint32_t channel_status_addr =
        sens->base + touch->sens_channel_status_offset;
    uint32_t rtc_raw_addr = rtc->base + rtc->interrupt_raw_offset;
    uint32_t rtc_status_addr = rtc->base + rtc->interrupt_status_offset;
    uint32_t rtc_clear_addr = rtc->base + rtc->interrupt_clear_offset;

    periph_touch_set_value(periph, (int)channel, 1000u);
    mem_write32(mem, threshold_addr, 200u);
    uint32_t scan = mem_read32(
        mem, rtc->base + touch->rtc_scan_control_offset);
    scan = (scan & ~touch->rtc_scan_channel_mask) |
           (channel_bit << touch->rtc_scan_channel_shift);
    mem_write32(mem, rtc->base + touch->rtc_scan_control_offset, scan);
    uint32_t control = mem_read32(
        mem, rtc->base + touch->rtc_control2_offset);
    control &= ~touch->rtc_reset_mask;
    control |= touch->rtc_clock_enable_mask |
               touch->rtc_timer_enable_mask;
    mem_write32(mem, rtc->base + touch->rtc_control2_offset, control);

    ASSERT_TRUE(periph_touch_running(periph));
    ASSERT_TRUE(periph_touch_scan_count(periph) >= 1u);
    ASSERT_EQ(periph_touch_active_mask(periph), 0u);
    ASSERT_EQ(mem_read32(mem, channel_status_addr) &
              touch->sens_measure_done_mask,
              touch->sens_measure_done_mask);
    ASSERT_EQ(mem_read32(mem, status_addr) & touch->sens_data_mask, 1000u);

    uint32_t irq_mask = touch->interrupt_done_mask |
                        touch->interrupt_active_mask |
                        touch->interrupt_inactive_mask |
                        touch->interrupt_scan_done_mask;
    ASSERT_EQ(mem_read32(mem,
                         rtc->base + rtc->interrupt_enable_set_offset), 0u);
    mem_write32(mem, rtc_clear_addr, rtc->interrupt_valid_mask);
    mem_write32(mem, rtc->base + rtc->interrupt_enable_set_offset,
                irq_mask);
    ASSERT_EQ(mem_read32(mem, rtc->base + rtc->interrupt_enable_offset) &
              irq_mask, irq_mask);

    periph_touch_set_value(periph, (int)channel, 1400u);
    ASSERT_EQ(periph_touch_active_mask(periph), channel_bit);
    ASSERT_EQ(mem_read32(mem, channel_status_addr) &
              touch->sens_active_mask, channel_bit);
    ASSERT_EQ(mem_read32(mem, status_addr) & touch->sens_data_mask, 1400u);
    ASSERT_EQ(mem_read32(mem, rtc_raw_addr) &
              touch->interrupt_active_mask,
              touch->interrupt_active_mask);
    ASSERT_EQ(mem_read32(mem, rtc_status_addr) &
              touch->interrupt_active_mask,
              touch->interrupt_active_mask);

    uint32_t config = mem_read32(mem, config_addr);
    config = (config & ~touch->sens_data_select_mask) |
             (2u << touch->sens_data_select_shift);
    mem_write32(mem, config_addr, config);
    ASSERT_EQ(mem_read32(mem, status_addr) & touch->sens_data_mask, 1000u);
    config = (config & ~touch->sens_data_select_mask) |
             (3u << touch->sens_data_select_shift);
    mem_write32(mem, config_addr, config);
    ASSERT_EQ(mem_read32(mem, status_addr) & touch->sens_data_mask, 1400u);

    mem_write32(mem, rtc_clear_addr, rtc->interrupt_valid_mask);
    periph_touch_set_value(periph, (int)channel, 1050u);
    ASSERT_EQ(periph_touch_active_mask(periph), 0u);
    ASSERT_EQ(mem_read32(mem, rtc_raw_addr) &
              touch->interrupt_inactive_mask,
              touch->interrupt_inactive_mask);
    ASSERT_TRUE(periph_touch_scan_count(periph) >= 3u);

    mem_write32(mem, rtc->base + rtc->interrupt_enable_clear_offset,
                touch->interrupt_inactive_mask);
    ASSERT_EQ(mem_read32(mem, rtc->base + rtc->interrupt_enable_offset) &
              touch->interrupt_inactive_mask, 0u);
    ASSERT_EQ(mem_read32(mem,
                         rtc->base + rtc->interrupt_enable_clear_offset), 0u);

    mem_write32(mem, channel_status_addr,
                channel_bit << touch->sens_channel_clear_shift);
    config = (config & ~touch->sens_data_select_mask) |
             (2u << touch->sens_data_select_shift);
    mem_write32(mem, config_addr, config);
    ASSERT_EQ(mem_read32(mem, status_addr) & touch->sens_data_mask, 1050u);
    ASSERT_EQ(touch_field(mem_read32(mem, status_addr),
                          touch->sens_debounce_mask,
                          touch->sens_debounce_shift), 0u);

    mem_write32(mem, rtc->base + touch->rtc_control2_offset,
                control | touch->rtc_reset_mask);
    ASSERT_FALSE(periph_touch_running(periph));
    ASSERT_EQ(periph_touch_active_mask(periph), 0u);
    ASSERT_EQ(periph_unhandled_count(periph), 0u);
    ASSERT_EQ(mem_unmapped_count(mem), 0u);

    periph_destroy(periph);
    mem_destroy(mem);
}

TEST(touch_v2_rejects_invalid_or_absent_geometry)
{
    const flexe_target_desc_t *classic =
        flexe_target_by_id(FLEXE_TARGET_ESP32);
    xtensa_mem_t *mem = mem_create_for_target(classic);
    ASSERT_TRUE(mem != NULL);
    ASSERT_TRUE(flexe_touch_v2_create(mem, NULL, NULL) == NULL);
    mem_destroy(mem);

    const flexe_target_desc_t *s3 =
        flexe_target_by_id(FLEXE_TARGET_ESP32S3);
    flexe_target_desc_t invalid = *s3;
    flexe_touch_v2_desc_t invalid_touch =
        *flexe_touch_v2_descriptor(s3);
    invalid_touch.channel_count = FLEXE_TARGET_TOUCH_CHANNEL_MAX + 1u;
    invalid.extension[0].descriptor = &invalid_touch;
    mem = mem_create_for_target(&invalid);
    ASSERT_TRUE(mem != NULL);
    ASSERT_TRUE(flexe_touch_v2_create(mem, NULL, NULL) == NULL);
    mem_destroy(mem);
}

void run_touch_v2_tests(void)
{
    TEST_SUITE("ESP32-S3 capacitive touch v2");
    RUN_TEST(touch_v2_scans_host_samples_and_routes_rtc_interrupts);
    RUN_TEST(touch_v2_rejects_invalid_or_absent_geometry);
}
