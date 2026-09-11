/* Target-described RTC scratch-register tests. */
#include "test_helpers.h"
#include "peripherals.h"
#include "rtc_storage.h"

typedef struct {
    unsigned reads;
    unsigned writes;
} rtc_storage_fallback_t;

static uint32_t rtc_storage_test_fallback_read(void *ctx, uint32_t addr)
{
    rtc_storage_fallback_t *fallback = ctx;
    fallback->reads++;
    return addr ^ 0x5A5A5A5Au;
}

static void rtc_storage_test_fallback_write(void *ctx, uint32_t addr,
                                            uint32_t value)
{
    rtc_storage_fallback_t *fallback = ctx;
    fallback->writes++;
    (void)addr;
    (void)value;
}

TEST(rtc_storage_resets_persists_and_delegates)
{
    const flexe_target_desc_t *s3 =
        flexe_target_by_id(FLEXE_TARGET_ESP32S3);
    const flexe_rtc_storage_desc_t *desc = &s3->rtc_storage;
    xtensa_mem_t *mem = mem_create_for_target(s3);
    rtc_storage_fallback_t fallback = {0};
    flexe_rtc_storage_t *storage = flexe_rtc_storage_create(
        mem, rtc_storage_test_fallback_read,
        rtc_storage_test_fallback_write, &fallback);
    ASSERT_TRUE(mem != NULL);
    ASSERT_TRUE(storage != NULL);
    if (!mem || !storage) {
        flexe_rtc_storage_destroy(storage);
        mem_destroy(mem);
        return;
    }

    uint32_t store0 = desc->base + desc->store_offset[0];
    uint32_t store4 = desc->base + desc->store_offset[4];
    ASSERT_EQ(mem_read32(mem, store0), 0u);
    ASSERT_EQ(mem_read32(mem, store4), 0u);
    mem_write32(mem, store0, 0x01234567u);
    mem_write32(mem, store4, 0x89ABCDEFu);
    ASSERT_EQ(mem_read32(mem, store0), 0x01234567u);
    ASSERT_EQ(mem_read32(mem, store4), 0x89ABCDEFu);

    uint32_t reserved = desc->base + 0x1F8u;
    ASSERT_EQ(mem_read32(mem, reserved), reserved ^ 0x5A5A5A5Au);
    mem_write32(mem, reserved, 1u);
    ASSERT_EQ(fallback.reads, 1u);
    ASSERT_EQ(fallback.writes, 1u);

    flexe_rtc_storage_destroy(storage);
    mem_destroy(mem);
}

TEST(rtc_storage_application_handoff_uses_target_clocks)
{
    const flexe_target_desc_t *s3 =
        flexe_target_by_id(FLEXE_TARGET_ESP32S3);
    const flexe_rtc_storage_desc_t *desc = &s3->rtc_storage;
    xtensa_mem_t *mem = mem_create_for_target(s3);
    flexe_rtc_storage_t *storage =
        flexe_rtc_storage_create(mem, NULL, NULL, NULL);
    ASSERT_TRUE(mem != NULL);
    ASSERT_TRUE(storage != NULL);
    if (!mem || !storage) {
        flexe_rtc_storage_destroy(storage);
        mem_destroy(mem);
        return;
    }

    flexe_rtc_storage_application_handoff(storage);
    uint32_t cal_addr = desc->base +
                        desc->store_offset[desc->slow_clock_cal_store];
    uint32_t xtal_addr = desc->base +
                         desc->store_offset[desc->xtal_frequency_store];
    ASSERT_EQ(mem_read32(mem, cal_addr),
              (uint32_t)((UINT64_C(1000000) << 19) /
                         desc->slow_clock_hz));
    ASSERT_EQ(mem_read32(mem, xtal_addr), 0x00280028u);

    flexe_rtc_storage_destroy(storage);
    mem_destroy(mem);
}

TEST(rtc_storage_s3_peripheral_handoff_is_visible)
{
    const flexe_target_desc_t *s3 =
        flexe_target_by_id(FLEXE_TARGET_ESP32S3);
    const flexe_rtc_storage_desc_t *desc = &s3->rtc_storage;
    xtensa_mem_t *mem = mem_create_for_target(s3);
    esp32_periph_t *periph = periph_create(mem);
    ASSERT_TRUE(mem != NULL);
    ASSERT_TRUE(periph != NULL);
    if (!mem || !periph) {
        periph_destroy(periph);
        mem_destroy(mem);
        return;
    }

    uint32_t xtal_addr = desc->base +
                         desc->store_offset[desc->xtal_frequency_store];
    ASSERT_EQ(mem_read32(mem, xtal_addr), 0x00280028u);
    ASSERT_EQ(periph_unhandled_count(periph), 0);

    int before = periph_unhandled_count(periph);
    ASSERT_EQ(mem_read32(mem, desc->base + 0x38u), 0u);
    ASSERT_EQ(periph_unhandled_count(periph), before + 1);

    periph_destroy(periph);
    mem_destroy(mem);
}

void run_rtc_storage_tests(void)
{
    TEST_SUITE("Target RTC storage");
    RUN_TEST(rtc_storage_resets_persists_and_delegates);
    RUN_TEST(rtc_storage_application_handoff_uses_target_clocks);
    RUN_TEST(rtc_storage_s3_peripheral_handoff_is_visible);
}
