#include "test_helpers.h"

#include "apb_saradc.h"
#include "peripherals.h"
#include "sens.h"
#include "target.h"

#define S3_APB_SARADC_BASE       0x60040000u
#define S3_APB_SARADC_CTRL       (S3_APB_SARADC_BASE + 0x000u)
#define S3_APB_SARADC_CTRL2      (S3_APB_SARADC_BASE + 0x004u)
#define S3_APB_SARADC_PATTERN1   (S3_APB_SARADC_BASE + 0x018u)
#define S3_APB_SARADC_DMA_CONF   (S3_APB_SARADC_BASE + 0x06Cu)
#define S3_APB_SARADC_DATA1      (S3_APB_SARADC_BASE + 0x040u)
#define S3_APB_SARADC_DATE       (S3_APB_SARADC_BASE + 0x3FCu)
#define S3_APB_SARADC_TIMER_SEL  (1u << 11)
#define S3_APB_SARADC_TIMER_EN   (1u << 24)
#define S3_APB_SARADC_DMA_EN     (1u << 31)

#define S3_SYSTEM_CLK_EN0        0x600C0018u
#define S3_SYSTEM_RST_EN0        0x600C0020u
#define S3_SYSTEM_APB_SARADC     (1u << 28)

#define S3_GDMA_BASE             0x6003F000u
#define S3_GDMA_IN_INT_RAW       (S3_GDMA_BASE + 0x008u)
#define S3_GDMA_IN_LINK          (S3_GDMA_BASE + 0x020u)
#define S3_GDMA_IN_PERI_SEL      (S3_GDMA_BASE + 0x048u)
#define S3_GDMA_IN_LINK_START    (1u << 22)
#define S3_GDMA_DESC_OWNER       (1u << 31)
#define S3_GDMA_ADC_TRIGGER      8u

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
        mem, NULL, NULL, NULL,
        apb_saradc_test_read, apb_saradc_test_write, &fallback);
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
    uint32_t unknown = desc->base + 0x080u;
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

TEST(apb_saradc_streams_patterned_samples_through_gdma)
{
    const flexe_target_desc_t *s3 =
        flexe_target_by_id(FLEXE_TARGET_ESP32S3);
    const flexe_apb_saradc_desc_t *desc = &s3->apb_saradc;
    xtensa_mem_t *mem = mem_create_for_target(s3);
    esp32_periph_t *p = periph_create(mem);
    ASSERT_TRUE(mem != NULL);
    ASSERT_TRUE(p != NULL);
    if (!mem || !p) {
        periph_destroy(p);
        mem_destroy(mem);
        return;
    }

    const uint32_t descriptor = 0x3FC8F000u;
    const uint32_t buffer = 0x3FC90000u;
    const uint32_t bytes = 16u;
    mem_write32(mem, descriptor, bytes | S3_GDMA_DESC_OWNER);
    mem_write32(mem, descriptor + 4u, buffer);
    mem_write32(mem, descriptor + 8u, 0u);
    mem_write32(mem, S3_GDMA_IN_PERI_SEL, S3_GDMA_ADC_TRIGGER);
    mem_write32(mem, S3_GDMA_IN_LINK,
                (descriptor & 0xFFFFFu) | S3_GDMA_IN_LINK_START);

    periph_set_adc_value(p, 2, 0x123u);
    periph_set_adc_value(p, 3, 0xABCu);
    /* Four six-bit entries are packed MSB first into each 24-bit pattern
     * word. Configure two ADC1 channels and a four-conversion EOF. */
    mem_write32(mem, S3_APB_SARADC_PATTERN1,
                ((2u << 2u) << 18u) | ((3u << 2u) << 12u));
    uint32_t control = desc->control_reset &
                       ~desc->pattern_length_mask[0];
    control |= 1u << 15u; /* two entries are encoded as length minus one */
    mem_write32(mem, S3_APB_SARADC_CTRL, control);
    mem_write32(mem, S3_APB_SARADC_DMA_CONF,
                S3_APB_SARADC_DMA_EN | 4u);
    mem_write32(mem, S3_APB_SARADC_CTRL2,
                desc->control2_reset | S3_APB_SARADC_TIMER_SEL |
                S3_APB_SARADC_TIMER_EN);

    ASSERT_TRUE(periph_adc_continuous_active(p));
    ASSERT_EQ64(periph_adc_continuous_frame_count(p), 1u);
    ASSERT_EQ64(periph_adc_continuous_sample_count(p), 4u);
    ASSERT_EQ(mem_read32(mem, buffer), 0x00004123u);
    ASSERT_EQ(mem_read32(mem, buffer + 4u), 0x00006ABCu);
    ASSERT_EQ(mem_read32(mem, buffer + 8u), 0x00004123u);
    ASSERT_EQ(mem_read32(mem, buffer + 12u), 0x00006ABCu);
    ASSERT_EQ(mem_read32(mem, S3_APB_SARADC_DATA1), 0x00006ABCu);
    ASSERT_EQ((mem_read32(mem, descriptor) >> 12u) & 0xFFFu, bytes);
    ASSERT_EQ(mem_read32(mem, descriptor) & S3_GDMA_DESC_OWNER, 0u);
    ASSERT_EQ(mem_read32(mem, S3_GDMA_IN_INT_RAW), 3u);

    /* The architectural SYSTEM clock and reset own functional operation. */
    mem_write32(mem, S3_SYSTEM_CLK_EN0,
                mem_read32(mem, S3_SYSTEM_CLK_EN0) &
                ~S3_SYSTEM_APB_SARADC);
    ASSERT_FALSE(periph_adc_continuous_active(p));
    ASSERT_EQ(periph_adc_continuous_inject(p), 0u);
    mem_write32(mem, S3_SYSTEM_RST_EN0,
                mem_read32(mem, S3_SYSTEM_RST_EN0) |
                S3_SYSTEM_APB_SARADC);
    ASSERT_EQ(mem_read32(mem, S3_APB_SARADC_CTRL), desc->control_reset);
    ASSERT_EQ(mem_read32(mem, S3_APB_SARADC_CTRL2), desc->control2_reset);
    ASSERT_EQ(mem_read32(mem, S3_APB_SARADC_DATE), desc->date_reset);
    mem_write32(mem, S3_APB_SARADC_DATE, 0x12345678u);
    ASSERT_EQ(mem_read32(mem, S3_APB_SARADC_DATE), 0x12345678u);
    ASSERT_EQ(periph_unhandled_count(p), 0u);

    periph_destroy(p);
    mem_destroy(mem);
}

TEST(apb_saradc_rejects_classic_target)
{
    xtensa_mem_t *mem = mem_create_for_target(
        flexe_target_by_id(FLEXE_TARGET_ESP32));
    ASSERT_TRUE(mem != NULL);
    ASSERT_TRUE(flexe_apb_saradc_create(
        mem, NULL, NULL, NULL, NULL, NULL, NULL) == NULL);
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
    TEST_SUITE("S3 APB SAR ADC digital controller");
    RUN_TEST(apb_saradc_retains_priorities_and_forced_grant);
    RUN_TEST(apb_saradc_forced_ownership_controls_s3_adc2);
    RUN_TEST(apb_saradc_streams_patterned_samples_through_gdma);
    RUN_TEST(apb_saradc_rejects_classic_target);
    RUN_TEST(apb_saradc_missing_model_cannot_complete_adc2);
}
