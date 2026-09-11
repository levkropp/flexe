/* Target-described always-on RTC controller tests. */
#include "test_helpers.h"
#include "peripherals.h"
#include "rtc_cntl.h"

typedef struct {
    unsigned reads;
    unsigned writes;
} rtc_cntl_fallback_t;

static uint32_t rtc_cntl_test_fallback_read(void *ctx, uint32_t addr)
{
    rtc_cntl_fallback_t *fallback = ctx;
    fallback->reads++;
    return addr ^ 0x5A5A5A5Au;
}

static void rtc_cntl_test_fallback_write(void *ctx, uint32_t addr,
                                         uint32_t value)
{
    rtc_cntl_fallback_t *fallback = ctx;
    fallback->writes++;
    (void)addr;
    (void)value;
}

TEST(rtc_cntl_storage_resets_persists_and_delegates)
{
    const flexe_target_desc_t *s3 =
        flexe_target_by_id(FLEXE_TARGET_ESP32S3);
    const flexe_rtc_cntl_desc_t *desc = &s3->rtc_cntl;
    xtensa_mem_t *mem = mem_create_for_target(s3);
    rtc_cntl_fallback_t fallback = {0};
    flexe_rtc_cntl_t *rtc = flexe_rtc_cntl_create(
        mem, rtc_cntl_test_fallback_read,
        rtc_cntl_test_fallback_write, &fallback);
    ASSERT_TRUE(mem != NULL);
    ASSERT_TRUE(rtc != NULL);
    if (!mem || !rtc) {
        flexe_rtc_cntl_destroy(rtc);
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

    flexe_rtc_cntl_destroy(rtc);
    mem_destroy(mem);
}

TEST(rtc_cntl_application_handoff_uses_target_clocks)
{
    const flexe_target_desc_t *s3 =
        flexe_target_by_id(FLEXE_TARGET_ESP32S3);
    const flexe_rtc_cntl_desc_t *desc = &s3->rtc_cntl;
    xtensa_mem_t *mem = mem_create_for_target(s3);
    flexe_rtc_cntl_t *rtc =
        flexe_rtc_cntl_create(mem, NULL, NULL, NULL);
    ASSERT_TRUE(mem != NULL);
    ASSERT_TRUE(rtc != NULL);
    if (!mem || !rtc) {
        flexe_rtc_cntl_destroy(rtc);
        mem_destroy(mem);
        return;
    }

    flexe_rtc_cntl_application_handoff(rtc);
    uint32_t cal_addr = desc->base +
                        desc->store_offset[desc->slow_clock_cal_store];
    uint32_t xtal_addr = desc->base +
                         desc->store_offset[desc->xtal_frequency_store];
    ASSERT_EQ(mem_read32(mem, cal_addr),
              (uint32_t)((UINT64_C(1000000) << 19) /
                         desc->slow_clock_hz));
    ASSERT_EQ(mem_read32(mem, xtal_addr), 0x00280028u);

    flexe_rtc_cntl_destroy(rtc);
    mem_destroy(mem);
}

static uint64_t rtc_capture(xtensa_mem_t *mem,
                            const flexe_rtc_cntl_desc_t *desc)
{
    mem_write32(mem, desc->base + desc->time_update_offset,
                desc->time_update_mask);
    uint64_t low = mem_read32(mem, desc->base + desc->time_low_offset);
    uint64_t high = mem_read32(mem, desc->base + desc->time_high_offset);
    return low | (high << 32u);
}

TEST(rtc_cntl_counter_tracks_shared_time_and_frequency)
{
    const flexe_target_desc_t *s3 =
        flexe_target_by_id(FLEXE_TARGET_ESP32S3);
    const flexe_rtc_cntl_desc_t *desc = &s3->rtc_cntl;
    xtensa_mem_t *mem = mem_create_for_target(s3);
    esp32_periph_t *periph = periph_create(mem);
    xtensa_cpu_t cpu[2];
    ASSERT_TRUE(mem != NULL);
    ASSERT_TRUE(periph != NULL);
    if (!mem || !periph) {
        periph_destroy(periph);
        mem_destroy(mem);
        return;
    }

    xtensa_cpu_init_for_target(&cpu[0], s3);
    xtensa_cpu_init_for_target(&cpu[1], s3);
    cpu[0].mem = mem;
    cpu[1].mem = mem;
    cpu[1].core_id = 1;
    periph_attach_cpus(periph, &cpu[0], &cpu[1]);

    ASSERT_EQ(mem_read32(mem, desc->base + desc->reset_state_offset),
              0x00003041u);
    ASSERT_EQ(mem_read32(mem, desc->base + desc->time_update_offset), 0u);
    ASSERT_EQ64(rtc_capture(mem, desc), 0u);

    /* One millisecond at 160 MHz is 136 ticks of the target-described
     * nominal S3 RC slow clock. The second core cannot double-count it. */
    cpu[0].ccount = 160000u;
    ASSERT_EQ64(rtc_capture(mem, desc), 136u);
    cpu[1].ccount = 80000u;
    ASSERT_EQ64(rtc_capture(mem, desc), 136u);
    cpu[1].ccount = 320000u;
    ASSERT_EQ64(rtc_capture(mem, desc), 272u);

    /* Scaling changes at an observed frequency boundary without disturbing
     * the accumulated always-on count. */
    mem_write32(mem, s3->cpu_frequency_word, 80u);
    cpu[0].ccount = 400000u;
    ASSERT_EQ64(rtc_capture(mem, desc), 408u);
    ASSERT_EQ(periph_unhandled_count(periph), 0);

    periph_destroy(periph);
    mem_destroy(mem);
}

TEST(rtc_cntl_unmodeled_power_registers_remain_unsupported)
{
    const flexe_target_desc_t *s3 =
        flexe_target_by_id(FLEXE_TARGET_ESP32S3);
    const flexe_rtc_cntl_desc_t *desc = &s3->rtc_cntl;
    xtensa_mem_t *mem = mem_create_for_target(s3);
    esp32_periph_t *periph = periph_create(mem);
    ASSERT_TRUE(mem != NULL);
    ASSERT_TRUE(periph != NULL);
    if (!mem || !periph) {
        periph_destroy(periph);
        mem_destroy(mem);
        return;
    }

    int before = periph_unhandled_count(periph);
    ASSERT_EQ(mem_read32(mem, desc->base + 0x74u), 0u);
    mem_write32(mem, desc->base + 0x74u, 1u);
    ASSERT_EQ(periph_unhandled_count(periph), before + 2);

    /* Unsupported timestamp-control bits share TIME_UPDATE with the modeled
     * latch command and must still produce an explicit diagnostic. */
    mem_write32(mem, desc->base + desc->time_update_offset,
                desc->time_update_mask | (1u << 29u));
    ASSERT_EQ(periph_unhandled_count(periph), before + 3);

    periph_destroy(periph);
    mem_destroy(mem);
}

void run_rtc_cntl_tests(void)
{
    TEST_SUITE("Target RTC controller");
    RUN_TEST(rtc_cntl_storage_resets_persists_and_delegates);
    RUN_TEST(rtc_cntl_application_handoff_uses_target_clocks);
    RUN_TEST(rtc_cntl_counter_tracks_shared_time_and_frequency);
    RUN_TEST(rtc_cntl_unmodeled_power_registers_remain_unsupported);
}
