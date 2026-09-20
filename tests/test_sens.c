/* Target-described RTC-domain sensor controller tests. */
#include "test_helpers.h"
#include "apb_saradc.h"
#include "peripherals.h"
#include "regi2c.h"
#include "sens.h"
#include "target.h"

typedef struct {
    unsigned reads;
    unsigned writes;
    uint32_t last_addr;
    uint32_t last_value;
} sens_fallback_t;

typedef struct {
    unsigned completions;
} sens_conversion_probe_t;

static uint32_t sens_test_fallback_read(void *ctx, uint32_t addr)
{
    sens_fallback_t *fallback = ctx;
    fallback->reads++;
    fallback->last_addr = addr;
    return addr ^ 0xA55AA55Au;
}

static void sens_test_fallback_write(void *ctx, uint32_t addr,
                                     uint32_t value)
{
    sens_fallback_t *fallback = ctx;
    fallback->writes++;
    fallback->last_addr = addr;
    fallback->last_value = value;
}

static void sens_test_conversion_done(void *ctx)
{
    sens_conversion_probe_t *probe = ctx;
    probe->completions++;
}

static uint32_t sens_field_value(uint32_t value, uint32_t mask)
{
    unsigned shift = 0u;
    while (((mask >> shift) & 1u) == 0u) shift++;
    return (value & mask) >> shift;
}

TEST(sens_temperature_conversion_obeys_power_clock_and_reset)
{
    const flexe_target_desc_t *s3 =
        flexe_target_by_id(FLEXE_TARGET_ESP32S3);
    const flexe_sens_desc_t *desc = &s3->sens;
    xtensa_mem_t *mem = mem_create_for_target(s3);
    sens_fallback_t fallback = {0};
    sens_conversion_probe_t conversion = {0};
    flexe_sens_t *sens = flexe_sens_create(
        mem, sens_test_fallback_read, sens_test_fallback_write, &fallback,
        sens_test_conversion_done, &conversion);
    ASSERT_TRUE(mem != NULL);
    ASSERT_TRUE(sens != NULL);
    if (!mem || !sens) {
        flexe_sens_destroy(sens);
        mem_destroy(mem);
        return;
    }

    uint32_t control_addr = desc->base + desc->control_offset;
    uint32_t control2_addr = desc->base + desc->control2_offset;
    uint32_t clock_addr = desc->base + desc->clock_gate_offset;
    uint32_t reset_addr = desc->base + desc->reset_offset;
    ASSERT_EQ(mem_read32(mem, control_addr), desc->control_reset);
    ASSERT_EQ(mem_read32(mem, control2_addr), desc->control2_reset);
    ASSERT_EQ(flexe_sens_temperature_raw(sens), desc->default_output);

    uint32_t powered = desc->control_reset |
        desc->dump_out_mask | desc->power_up_force_mask |
        desc->power_up_mask;
    mem_write32(mem, control2_addr,
                desc->control2_reset |
                (desc->xpd_force_mask & (0u - desc->xpd_force_mask)));
    mem_write32(mem, control_addr, powered);
    ASSERT_EQ(mem_read32(mem, control_addr) & desc->ready_mask, 0u);
    ASSERT_EQ(conversion.completions, 0u);

    mem_write32(mem, clock_addr, desc->clock_enable_mask);
    uint32_t completed = mem_read32(mem, control_addr);
    ASSERT_EQ(completed & desc->ready_mask, desc->ready_mask);
    ASSERT_EQ(sens_field_value(completed, desc->output_mask),
              desc->default_output);
    ASSERT_EQ(conversion.completions, 1u);

    /* A completed result is latched until a fresh dump-out pulse. */
    flexe_sens_set_temperature_raw(sens, UINT16_MAX);
    ASSERT_EQ(flexe_sens_temperature_raw(sens), 0xFFu);
    flexe_sens_set_temperature_raw(sens, 0x42u);
    ASSERT_EQ(sens_field_value(mem_read32(mem, control_addr),
                               desc->output_mask),
              desc->default_output);
    mem_write32(mem, control_addr, completed & ~desc->dump_out_mask);
    ASSERT_EQ(mem_read32(mem, control_addr) & desc->ready_mask, 0u);
    mem_write32(mem, control_addr,
                powered | desc->input_invert_mask);
    completed = mem_read32(mem, control_addr);
    ASSERT_EQ(completed & desc->ready_mask, desc->ready_mask);
    ASSERT_EQ(sens_field_value(completed, desc->output_mask),
              0xBDu);
    ASSERT_EQ(conversion.completions, 2u);

    /* Read-only status/output bits are harmless in a register RMW. */
    mem_write32(mem, control_addr, completed);
    ASSERT_EQ(sens_field_value(mem_read32(mem, control_addr),
                               desc->output_mask),
              0xBDu);
    ASSERT_EQ(fallback.writes, 0u);

    mem_write32(mem, reset_addr, desc->reset_mask);
    ASSERT_EQ(mem_read32(mem, control_addr), desc->control_reset);
    ASSERT_EQ(mem_read32(mem, control2_addr), desc->control2_reset);
    mem_write32(mem, reset_addr, 0u);

    /* Other documented clock fields retain readback but explicitly report
     * that their device behavior is outside this model. */
    uint32_t other_clock =
        desc->clock_gate_writable_mask & ~desc->clock_enable_mask;
    other_clock &= 0u - other_clock;
    mem_write32(mem, clock_addr, other_clock);
    ASSERT_EQ(mem_read32(mem, clock_addr), other_clock);
    ASSERT_EQ(fallback.writes, 1u);
    ASSERT_EQ(fallback.last_addr, clock_addr);

    uint32_t outside = desc->base - 4u;
    ASSERT_EQ(mem_read32(mem, outside), outside ^ 0xA55AA55Au);
    ASSERT_EQ(fallback.reads, 1u);

    flexe_sens_destroy(sens);
    mem_destroy(mem);
}

TEST(sens_composes_with_rtc_page_and_latches_interrupt)
{
    const flexe_target_desc_t *s3 =
        flexe_target_by_id(FLEXE_TARGET_ESP32S3);
    const flexe_sens_desc_t *sens = &s3->sens;
    const flexe_rtc_cntl_desc_t *rtc = &s3->rtc_cntl;
    xtensa_mem_t *mem = mem_create_for_target(s3);
    esp32_periph_t *periph = periph_create(mem);
    ASSERT_TRUE(mem != NULL);
    ASSERT_TRUE(periph != NULL);
    if (!mem || !periph) {
        periph_destroy(periph);
        mem_destroy(mem);
        return;
    }

    /* RTC_CNTL and SENS occupy subranges of one physical 4-KiB page. Both
     * models must remain reachable through the composed fallback chain. */
    uint32_t store_addr = rtc->base + rtc->store_offset[0];
    mem_write32(mem, store_addr, 0x12345678u);
    ASSERT_EQ(mem_read32(mem, store_addr), 0x12345678u);

    periph_set_temperature_raw(periph, 0x55u);
    ASSERT_EQ(periph_get_temperature_raw(periph), 0x55u);
    mem_write32(mem, sens->base + sens->clock_gate_offset,
                sens->clock_enable_mask);
    mem_write32(mem, sens->base + sens->control2_offset,
                sens->control2_reset |
                (sens->xpd_force_mask & (0u - sens->xpd_force_mask)));
    mem_write32(mem, sens->base + sens->control_offset,
                sens->control_reset | sens->dump_out_mask |
                sens->power_up_force_mask | sens->power_up_mask);
    uint32_t sample =
        mem_read32(mem, sens->base + sens->control_offset);
    ASSERT_EQ(sample & sens->ready_mask, sens->ready_mask);
    ASSERT_EQ(sens_field_value(sample, sens->output_mask), 0x55u);

    uint32_t raw_addr = rtc->base + rtc->interrupt_raw_offset;
    uint32_t enable_addr = rtc->base + rtc->interrupt_enable_offset;
    uint32_t status_addr = rtc->base + rtc->interrupt_status_offset;
    uint32_t clear_addr = rtc->base + rtc->interrupt_clear_offset;
    ASSERT_EQ(mem_read32(mem, raw_addr) & sens->rtc_interrupt_mask,
              sens->rtc_interrupt_mask);
    ASSERT_EQ(mem_read32(mem, status_addr) & sens->rtc_interrupt_mask, 0u);
    mem_write32(mem, enable_addr, sens->rtc_interrupt_mask);
    ASSERT_EQ(mem_read32(mem, status_addr) & sens->rtc_interrupt_mask,
              sens->rtc_interrupt_mask);
    mem_write32(mem, clear_addr, sens->rtc_interrupt_mask);
    ASSERT_EQ(mem_read32(mem, raw_addr) & sens->rtc_interrupt_mask, 0u);
    ASSERT_EQ(periph_unhandled_count(periph), 0);

    periph_destroy(periph);
    mem_destroy(mem);
}

TEST(sens_s3_rtc_adc_latches_channel_and_obeys_clock_reset)
{
    const flexe_target_desc_t *s3 =
        flexe_target_by_id(FLEXE_TARGET_ESP32S3);
    const flexe_sens_desc_t *desc = &s3->sens;
    xtensa_mem_t *mem = mem_create_for_target(s3);
    sens_fallback_t fallback = {0};
    flexe_sens_t *sens = flexe_sens_create(
        mem, sens_test_fallback_read, sens_test_fallback_write,
        &fallback, NULL, NULL);
    flexe_apb_saradc_t *apb = flexe_apb_saradc_create(
        mem, sens_test_fallback_read, sens_test_fallback_write, &fallback);
    ASSERT_TRUE(mem != NULL);
    ASSERT_TRUE(sens != NULL);
    ASSERT_TRUE(apb != NULL);
    if (!mem || !sens || !apb) {
        flexe_sens_destroy(sens);
        flexe_apb_saradc_destroy(apb);
        mem_destroy(mem);
        return;
    }
    flexe_sens_attach_apb_saradc(sens, apb);

    const flexe_sens_adc_unit_desc_t *adc1 = &desc->adc_unit[0];
    const flexe_sens_adc_unit_desc_t *adc2 = &desc->adc_unit[1];
    uint32_t measure1 = desc->base + adc1->measure_offset;
    uint32_t measure2 = desc->base + adc2->measure_offset;
    uint32_t clock = desc->base + desc->clock_gate_offset;
    uint32_t reset = desc->base + desc->reset_offset;
    uint32_t power = desc->base + desc->adc_power_offset;
    const uint32_t start1 = (1u << 31) | (1u << (19 + 3)) |
                            (1u << 18) | (1u << 17);
    const uint32_t start2 = (1u << 31) | (1u << (19 + 8)) |
                            (1u << 18) | (1u << 17);
    ASSERT_EQ(mem_read32(mem, measure1), 0u);
    ASSERT_EQ(mem_read32(mem, measure2), 0u);
    ASSERT_EQ(mem_read32(mem, desc->base + adc1->reader_offset),
              adc1->reader_reset);
    ASSERT_EQ(mem_read32(mem, desc->base + adc2->reader_offset),
              adc2->reader_reset);
    ASSERT_EQ(mem_read32(mem, desc->base + adc1->atten_offset), UINT32_MAX);
    ASSERT_EQ(mem_read32(mem, desc->base + desc->adc_status_offset), 0u);
    mem_write32(mem, desc->base + desc->adc_status_offset, 0x00123456u);
    ASSERT_EQ(mem_read32(mem, desc->base + desc->adc_status_offset),
              0x00123456u);

    flexe_sens_set_adc_raw(sens, 0u, 3u, 0x1234u);
    mem_write32(mem, measure1, start1);
    ASSERT_EQ(mem_read32(mem, measure1) & (1u << 16), 0u);
    ASSERT_EQ(fallback.writes, 1u); /* ADC clock is off. */
    mem_write32(mem, measure1, start1 & ~(1u << 17));
    mem_write32(mem, clock, desc->adc_clock_enable_mask);
    mem_write32(mem, power, 3u << 29);
    mem_write32(mem, measure1, start1);
    ASSERT_EQ(mem_read32(mem, measure1) & 0x1FFFFu,
              (1u << 16) | 0x1234u);
    ASSERT_EQ(fallback.writes, 1u);

    /* DATA is latched; writing a read/modify/write value cannot replace it. */
    flexe_sens_set_adc_raw(sens, 0u, 3u, 0x2ABCu);
    mem_write32(mem, measure1, mem_read32(mem, measure1));
    ASSERT_EQ(mem_read32(mem, measure1) & 0x1FFFFu,
              (1u << 16) | 0x1234u);
    mem_write32(mem, measure1, start1 & ~(1u << 17));
    ASSERT_EQ(mem_read32(mem, measure1) & (1u << 16), 0u);
    mem_write32(mem, measure1, start1);
    ASSERT_EQ(mem_read32(mem, measure1) & 0x1FFFFu,
              (1u << 16) | (0x2ABCu & desc->adc_output_mask));

    mem_write32(mem, power, 2u << 29); /* Forced-off SAR power. */
    mem_write32(mem, measure1, start1 & ~(1u << 17));
    mem_write32(mem, measure1, start1);
    ASSERT_EQ(mem_read32(mem, measure1) & (1u << 16), 0u);
    ASSERT_EQ(fallback.writes, 2u);
    mem_write32(mem, measure1, start1 & ~(1u << 17));
    mem_write32(mem, power, 3u << 29);

    flexe_sens_set_adc_raw(sens, 1u, 8u, 0x1234u);
    mem_write32(mem, desc->base + adc2->reader_offset,
                adc2->reader_reset | adc2->reader_invert_mask);
    mem_write32(mem, desc->base + adc2->atten_offset, 0x01234567u);
    ASSERT_EQ(mem_read32(mem, desc->base + adc2->atten_offset), 0x01234567u);
    mem_write32(mem, measure2, start2);
    ASSERT_EQ(mem_read32(mem, measure2) & 0x1FFFFu,
              (1u << 16) | (0x1234u ^ desc->adc_output_mask));
    ASSERT_EQ(fallback.writes, 2u);

    mem_write32(mem, reset, desc->adc_reset_mask);
    ASSERT_EQ(mem_read32(mem, measure1), 0u);
    ASSERT_EQ(mem_read32(mem, measure2), 0u);
    ASSERT_EQ(mem_read32(mem, desc->base + adc2->reader_offset),
              adc2->reader_reset);
    ASSERT_EQ(mem_read32(mem, desc->base + adc2->atten_offset), UINT32_MAX);
    ASSERT_EQ(mem_read32(mem, power), 0u);
    ASSERT_EQ(mem_read32(mem, desc->base + desc->adc_status_offset), 0u);
    ASSERT_EQ(fallback.writes, 2u);

    flexe_sens_destroy(sens);
    flexe_apb_saradc_destroy(apb);
    mem_destroy(mem);
}

TEST(sens_s3_rtc_adc_rejects_unmodeled_conversions)
{
    const flexe_target_desc_t *s3 =
        flexe_target_by_id(FLEXE_TARGET_ESP32S3);
    const flexe_sens_desc_t *desc = &s3->sens;
    xtensa_mem_t *mem = mem_create_for_target(s3);
    sens_fallback_t fallback = {0};
    flexe_sens_t *sens = flexe_sens_create(
        mem, sens_test_fallback_read, sens_test_fallback_write,
        &fallback, NULL, NULL);
    ASSERT_TRUE(mem != NULL);
    ASSERT_TRUE(sens != NULL);
    if (!mem || !sens) {
        flexe_sens_destroy(sens);
        mem_destroy(mem);
        return;
    }
    uint32_t measure = desc->base + desc->adc_unit[0].measure_offset;
    uint32_t mux = desc->base + desc->adc_unit[0].mux_offset;
    mem_write32(mem, desc->base + desc->clock_gate_offset,
                desc->adc_clock_enable_mask);
    uint32_t start = (1u << 31) | (1u << 18) | (1u << 17);
    mem_write32(mem, measure, start | (3u << 19));
    ASSERT_EQ(mem_read32(mem, measure) & (1u << 16), 0u);
    ASSERT_EQ(fallback.writes, 1u); /* More than one pad. */
    mem_write32(mem, measure, 0u);
    mem_write32(mem, mux, desc->adc_unit[0].mux_rtc_block_mask);
    ASSERT_EQ(fallback.writes, 2u); /* DIG controller selection is visible. */
    mem_write32(mem, measure, start | (1u << 19));
    ASSERT_EQ(mem_read32(mem, measure) & (1u << 16), 0u);
    ASSERT_EQ(fallback.writes, 3u); /* Digital controller is not RTC ADC. */
    /* Power-detector capacitance is an ordinary retained analog tuning
     * field. It does not fabricate an RF sample on the quiet-input model. */
    mem_write32(mem, desc->base + desc->adc_unit[1].mux_offset, 7u << 28);
    ASSERT_EQ(mem_read32(mem, desc->base +
                        desc->adc_unit[1].mux_offset), 7u << 28);
    ASSERT_EQ(fallback.writes, 3u);

    flexe_sens_destroy(sens);
    mem_destroy(mem);
}

TEST(sens_s3_host_adc_injection_reaches_mmio)
{
    const flexe_target_desc_t *s3 =
        flexe_target_by_id(FLEXE_TARGET_ESP32S3);
    xtensa_mem_t *mem = mem_create_for_target(s3);
    esp32_periph_t *periph = periph_create(mem);
    ASSERT_TRUE(mem != NULL);
    ASSERT_TRUE(periph != NULL);
    if (!mem || !periph) {
        periph_destroy(periph);
        mem_destroy(mem);
        return;
    }
    const flexe_sens_desc_t *desc = &s3->sens;
    const flexe_sens_adc_unit_desc_t *unit = &desc->adc_unit[1];
    periph_set_adc_value(periph, 17, 0x0A55u); /* ADC2 channel 7. */
    ASSERT_EQ(periph_get_adc_value(periph, 17), 0x0A55u);
    mem_write32(mem, desc->base + desc->clock_gate_offset,
                desc->adc_clock_enable_mask);
    mem_write32(mem, desc->base + unit->measure_offset,
                (1u << 31) | (1u << (19 + 7)) |
                (1u << 18) | (1u << 17));
    ASSERT_EQ(mem_read32(mem, desc->base + unit->measure_offset) & 0x1FFFFu,
              (1u << 16) | 0x0A55u);
    ASSERT_EQ(periph_unhandled_count(periph), 0u);
    periph_destroy(periph);
    mem_destroy(mem);
}

TEST(sens_s3_internal_ground_requires_analog_mux)
{
    const flexe_target_desc_t *s3 =
        flexe_target_by_id(FLEXE_TARGET_ESP32S3);
    const flexe_sens_desc_t *desc = &s3->sens;
    xtensa_mem_t *mem = mem_create_for_target(s3);
    sens_fallback_t fallback = {0};
    flexe_regi2c_t *regi2c = flexe_regi2c_create(
        mem, NULL, NULL, NULL);
    flexe_sens_t *sens = flexe_sens_create(
        mem, sens_test_fallback_read, sens_test_fallback_write,
        &fallback, NULL, NULL);
    ASSERT_TRUE(mem != NULL);
    ASSERT_TRUE(regi2c != NULL);
    ASSERT_TRUE(sens != NULL);
    if (!mem || !regi2c || !sens) {
        flexe_sens_destroy(sens);
        flexe_regi2c_destroy(regi2c);
        mem_destroy(mem);
        return;
    }
    flexe_sens_attach_regi2c(sens, regi2c);
    uint32_t measure = desc->base + desc->adc_unit[0].measure_offset;
    uint32_t start = (1u << 31) | (1u << 18) | (1u << 17);
    mem_write32(mem, desc->base + desc->clock_gate_offset,
                desc->adc_clock_enable_mask);
    mem_write32(mem, measure, start);
    ASSERT_EQ(mem_read32(mem, measure) & (1u << 16), 0u);
    ASSERT_EQ(fallback.writes, 1u);
    mem_write32(mem, measure, start & ~(1u << 17));

    const flexe_regi2c_desc_t *bus = &s3->regi2c;
    uint32_t command = bus->command_start_mask |
        bus->command_write_mask | desc->adc_calibration_slave |
        ((uint32_t)desc->adc_calibration_address << bus->address_shift) |
        ((uint32_t)desc->adc_unit[0].calibration_ground_mask <<
         bus->data_shift);
    mem_write32(mem, bus->base + bus->command_offset +
                bus->command_stride, command);
    uint8_t analog = 0u;
    ASSERT_TRUE(flexe_regi2c_register_read(
        regi2c, desc->adc_calibration_slave,
        desc->adc_calibration_address, &analog));
    ASSERT_EQ(analog, desc->adc_unit[0].calibration_ground_mask);
    mem_write32(mem, measure, start);
    ASSERT_EQ(mem_read32(mem, measure) & 0x1FFFFu, 1u << 16);
    ASSERT_EQ(fallback.writes, 1u);

    flexe_sens_destroy(sens);
    flexe_regi2c_destroy(regi2c);
    mem_destroy(mem);
}

TEST(sens_rejects_targets_without_the_capability)
{
    const flexe_target_desc_t *classic =
        flexe_target_by_id(FLEXE_TARGET_ESP32);
    xtensa_mem_t *mem = mem_create_for_target(classic);
    ASSERT_TRUE(mem != NULL);
    ASSERT_TRUE(flexe_sens_create(
        mem, NULL, NULL, NULL, NULL, NULL) == NULL);
    mem_destroy(mem);
}

void run_sens_tests(void)
{
    TEST_SUITE("Target SENS controller");
    RUN_TEST(sens_temperature_conversion_obeys_power_clock_and_reset);
    RUN_TEST(sens_composes_with_rtc_page_and_latches_interrupt);
    RUN_TEST(sens_s3_rtc_adc_latches_channel_and_obeys_clock_reset);
    RUN_TEST(sens_s3_rtc_adc_rejects_unmodeled_conversions);
    RUN_TEST(sens_s3_host_adc_injection_reaches_mmio);
    RUN_TEST(sens_s3_internal_ground_requires_analog_mux);
    RUN_TEST(sens_rejects_targets_without_the_capability);
}
