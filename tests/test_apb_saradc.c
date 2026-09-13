#include "test_helpers.h"

#include "apb_saradc.h"
#include "peripherals.h"
#include "sens.h"

typedef struct {
    unsigned reads;
    unsigned writes;
    uint32_t last_addr;
    uint32_t last_value;
} apb_saradc_fallback_t;

static uint32_t apb_saradc_test_read(void *ctx, uint32_t addr)
{
    apb_saradc_fallback_t *fallback = ctx;
    fallback->reads++;
    fallback->last_addr = addr;
    return addr ^ 0xA5A5A5A5u;
}

static void apb_saradc_test_write(void *ctx, uint32_t addr, uint32_t value)
{
    apb_saradc_fallback_t *fallback = ctx;
    fallback->writes++;
    fallback->last_addr = addr;
    fallback->last_value = value;
}

TEST(apb_saradc_retains_priorities_and_forced_grant)
{
    const flexe_target_desc_t *s3 =
        flexe_target_by_id(FLEXE_TARGET_ESP32S3);
    const flexe_apb_saradc_desc_t *desc = &s3->apb_saradc;
    xtensa_mem_t *mem = mem_create_for_target(s3);
    apb_saradc_fallback_t fallback = {0};
    flexe_apb_saradc_t *adc = flexe_apb_saradc_create(
        mem, apb_saradc_test_read, apb_saradc_test_write, &fallback);
    ASSERT_TRUE(mem != NULL);
    ASSERT_TRUE(adc != NULL);
    if (!mem || !adc) {
        flexe_apb_saradc_destroy(adc);
        mem_destroy(mem);
        return;
    }
    uint32_t arb = desc->base + desc->arbiter_offset;
    ASSERT_EQ(mem_read32(mem, arb), desc->arbiter_reset);
    ASSERT_TRUE(flexe_apb_saradc_rtc_granted(adc));

    uint32_t rtc = desc->arbiter_reset |
        desc->grant_force_mask | desc->rtc_force_mask;
    mem_write32(mem, arb, rtc);
    ASSERT_EQ(mem_read32(mem, arb), rtc);
    ASSERT_TRUE(flexe_apb_saradc_rtc_granted(adc));
    ASSERT_EQ(fallback.writes, 0u);

    uint32_t wifi = desc->arbiter_reset |
        desc->grant_force_mask | (1u << 4);
    mem_write32(mem, arb, wifi);
    ASSERT_EQ(mem_read32(mem, arb), wifi);
    ASSERT_FALSE(flexe_apb_saradc_rtc_granted(adc));
    ASSERT_EQ(fallback.writes, 1u);
    ASSERT_EQ(fallback.last_addr, arb);

    /* When grant is not forced, an RTC-only request wins regardless of
     * priority values; the currently unmodeled contenders are idle. */
    mem_write32(mem, arb, wifi & ~desc->grant_force_mask);
    ASSERT_TRUE(flexe_apb_saradc_rtc_granted(adc));
    ASSERT_EQ(fallback.writes, 1u);

    uint32_t reserved = (1u << 31) | desc->arbiter_reset;
    mem_write32(mem, arb, reserved);
    ASSERT_EQ(mem_read32(mem, arb), desc->arbiter_reset);
    ASSERT_EQ(fallback.writes, 2u);
    ASSERT_EQ(fallback.last_value, reserved);
    uint32_t unknown = desc->base + 0x03Cu;
    ASSERT_EQ(mem_read32(mem, unknown), unknown ^ 0xA5A5A5A5u);
    ASSERT_EQ(fallback.reads, 1u);

    flexe_apb_saradc_destroy(adc);
    mem_destroy(mem);
}

TEST(apb_saradc_forced_ownership_controls_s3_adc2)
{
    const flexe_target_desc_t *s3 =
        flexe_target_by_id(FLEXE_TARGET_ESP32S3);
    const flexe_apb_saradc_desc_t *arb = &s3->apb_saradc;
    const flexe_sens_desc_t *sens = &s3->sens;
    xtensa_mem_t *mem = mem_create_for_target(s3);
    esp32_periph_t *p = periph_create(mem);
    ASSERT_TRUE(mem != NULL);
    ASSERT_TRUE(p != NULL);
    if (!mem || !p) {
        periph_destroy(p);
        mem_destroy(mem);
        return;
    }

    uint32_t arb_addr = arb->base + arb->arbiter_offset;
    uint32_t measure = sens->base + sens->adc_unit[1].measure_offset;
    uint32_t mux = sens->base + sens->adc_unit[1].mux_offset;
    uint32_t start = (1u << 31) | (1u << 19) |
                     (1u << 18) | (1u << 17);
    periph_set_adc_value(p, 10, 0x05AAu);
    mem_write32(mem, sens->base + sens->clock_gate_offset,
                sens->adc_clock_enable_mask);

    mem_write32(mem, arb_addr, arb->arbiter_reset |
                arb->grant_force_mask | (1u << 4));
    ASSERT_EQ(periph_unhandled_count(p), 1u);
    mem_write32(mem, measure, start);
    ASSERT_EQ(mem_read32(mem, measure) & (1u << 16), 0u);
    ASSERT_EQ(periph_unhandled_count(p), 2u);

    mem_write32(mem, measure, start & ~(1u << 17));
    mem_write32(mem, arb_addr, arb->arbiter_reset |
                arb->grant_force_mask | arb->rtc_force_mask);
    mem_write32(mem, measure, start);
    ASSERT_EQ(mem_read32(mem, measure) & 0x1FFFFu,
              (1u << 16) | 0x05AAu);
    ASSERT_EQ(periph_unhandled_count(p), 2u);

    /* RTC_FORCE on the SENS ADC2 mux bypasses a shielded APB arbiter. */
    mem_write32(mem, measure, start & ~(1u << 17));
    mem_write32(mem, arb_addr, arb->arbiter_reset |
                arb->grant_force_mask | (1u << 4));
    mem_write32(mem, mux, sens->adc_unit[1].mux_rtc_bypass_mask);
    mem_write32(mem, measure, start);
    ASSERT_EQ(mem_read32(mem, measure) & 0x1FFFFu,
              (1u << 16) | 0x05AAu);
    ASSERT_EQ(periph_unhandled_count(p), 3u);

    periph_destroy(p);
    mem_destroy(mem);
}

TEST(apb_saradc_rejects_classic_target)
{
    xtensa_mem_t *mem = mem_create_for_target(
        flexe_target_by_id(FLEXE_TARGET_ESP32));
    ASSERT_TRUE(mem != NULL);
    ASSERT_TRUE(flexe_apb_saradc_create(
        mem, NULL, NULL, NULL) == NULL);
    mem_destroy(mem);
}

TEST(apb_saradc_missing_model_cannot_complete_adc2)
{
    const flexe_target_desc_t *s3 =
        flexe_target_by_id(FLEXE_TARGET_ESP32S3);
    xtensa_mem_t *mem = mem_create_for_target(s3);
    apb_saradc_fallback_t fallback = {0};
    flexe_sens_t *sens = flexe_sens_create(
        mem, apb_saradc_test_read, apb_saradc_test_write,
        &fallback, NULL, NULL);
    ASSERT_TRUE(mem != NULL);
    ASSERT_TRUE(sens != NULL);
    if (!mem || !sens) {
        flexe_sens_destroy(sens);
        mem_destroy(mem);
        return;
    }
    const flexe_sens_desc_t *desc = &s3->sens;
    mem_write32(mem, desc->base + desc->clock_gate_offset,
                desc->adc_clock_enable_mask);
    uint32_t measure = desc->base + desc->adc_unit[1].measure_offset;
    mem_write32(mem, measure,
                (1u << 31) | (1u << 19) | (1u << 18) | (1u << 17));
    ASSERT_EQ(mem_read32(mem, measure) & (1u << 16), 0u);
    ASSERT_EQ(fallback.writes, 1u);
    ASSERT_EQ(fallback.last_addr, measure);
    flexe_sens_destroy(sens);
    mem_destroy(mem);
}

void run_apb_saradc_tests(void)
{
    TEST_SUITE("S3 APB SAR ADC2 arbiter");
    RUN_TEST(apb_saradc_retains_priorities_and_forced_grant);
    RUN_TEST(apb_saradc_forced_ownership_controls_s3_adc2);
    RUN_TEST(apb_saradc_rejects_classic_target);
    RUN_TEST(apb_saradc_missing_model_cannot_complete_adc2);
}
