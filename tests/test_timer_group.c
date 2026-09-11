/* Descriptor-driven ESP32-family timer-group and MWDT tests. */
#include "test_helpers.h"
#include "peripherals.h"

#define S3_TIMG0_BASE       0x6001F000u
#define S3_TIMG1_BASE       0x60020000u

#define TG_TIMER_CONFIG     0x000u
#define TG_TIMER_LO         0x004u
#define TG_TIMER_HI         0x008u
#define TG_TIMER_UPDATE     0x00Cu
#define TG_TIMER_ALARM_LO   0x010u
#define TG_TIMER_ALARM_HI   0x014u
#define TG_TIMER_LOAD_LO    0x018u
#define TG_TIMER_LOAD_HI    0x01Cu
#define TG_TIMER_LOAD       0x020u
#define TG_TIMER_STRIDE     0x024u
#define TG_WDT_CONFIG0      0x048u
#define TG_WDT_CONFIG1      0x04Cu
#define TG_WDT_CONFIG2      0x050u
#define TG_WDT_CONFIG3      0x054u
#define TG_WDT_CONFIG4      0x058u
#define TG_WDT_CONFIG5      0x05Cu
#define TG_WDT_FEED         0x060u
#define TG_WDT_PROTECT      0x064u
#define TG_RTC_CALI_CONFIG  0x068u
#define TG_INT_ENA          0x070u
#define TG_INT_RAW          0x074u
#define TG_INT_ST           0x078u
#define TG_INT_CLR          0x07Cu
#define TG_DATE             0x0F8u
#define TG_REGCLK           0x0FCu

#define TG_TIMER_USE_XTAL   (1u << 9)
#define TG_TIMER_ALARM_EN   (1u << 10)
#define TG_TIMER_AUTORELOAD (1u << 29)
#define TG_TIMER_INCREASE   (1u << 30)
#define TG_TIMER_ENABLE     (1u << 31)

#define TG_WDT_FLASHBOOT    (1u << 14)
#define TG_WDT_ENABLE       (1u << 31)
#define TG_WDT_STAGE0_INT   (1u << 29)
#define TG_WDT_STAGE1_RESET (3u << 27)
#define TG_WDT_KEY          0x50D83AA1u

static uint32_t tg_read(xtensa_mem_t *mem, uint32_t base, uint32_t off)
{
    return mem_read32(mem, base + off);
}

static void tg_write(xtensa_mem_t *mem, uint32_t base, uint32_t off,
                     uint32_t value)
{
    mem_write32(mem, base + off, value);
}

static uint32_t tg_timer_off(unsigned timer, uint32_t relative)
{
    return timer * TG_TIMER_STRIDE + relative;
}

static void tg_load(xtensa_mem_t *mem, uint32_t base, unsigned timer,
                    uint64_t value)
{
    tg_write(mem, base, tg_timer_off(timer, TG_TIMER_LOAD_LO),
             (uint32_t)value);
    tg_write(mem, base, tg_timer_off(timer, TG_TIMER_LOAD_HI),
             (uint32_t)(value >> 32));
    tg_write(mem, base, tg_timer_off(timer, TG_TIMER_LOAD), 1u);
}

static uint64_t tg_capture(xtensa_mem_t *mem, uint32_t base,
                           unsigned timer)
{
    tg_write(mem, base, tg_timer_off(timer, TG_TIMER_UPDATE), 0u);
    return (uint64_t)tg_read(
               mem, base, tg_timer_off(timer, TG_TIMER_LO)) |
           ((uint64_t)tg_read(
                mem, base, tg_timer_off(timer, TG_TIMER_HI)) << 32);
}

TEST(target_timer_group_reset_masks_and_rtc_fallback)
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

    ASSERT_TRUE(s3->capabilities & FLEXE_TARGET_CAP_TIMER_GROUP_V1);
    for (unsigned group = 0u; group < 2u; group++) {
        uint32_t base = group == 0u ? S3_TIMG0_BASE : S3_TIMG1_BASE;
        ASSERT_EQ(tg_read(mem, base, TG_TIMER_CONFIG), 0x60002000u);
        ASSERT_EQ(tg_read(mem, base,
                          tg_timer_off(1u, TG_TIMER_CONFIG)),
                  0x60002000u);
        ASSERT_EQ(tg_read(mem, base, TG_WDT_CONFIG0), 0x0004C000u);
        ASSERT_EQ(tg_read(mem, base, TG_WDT_CONFIG1), 0x00010000u);
        ASSERT_EQ(tg_read(mem, base, TG_WDT_CONFIG2), 0x018CBA80u);
        ASSERT_EQ(tg_read(mem, base, TG_WDT_CONFIG3), 0x07FFFFFFu);
        ASSERT_EQ(tg_read(mem, base, TG_WDT_CONFIG4), 0x000FFFFFu);
        ASSERT_EQ(tg_read(mem, base, TG_WDT_CONFIG5), 0x000FFFFFu);
        ASSERT_EQ(tg_read(mem, base, TG_WDT_PROTECT), TG_WDT_KEY);
        ASSERT_EQ(tg_read(mem, base, TG_DATE), 0x02003071u);
        ASSERT_EQ(tg_read(mem, base, TG_REGCLK), 0u);
    }

    tg_write(mem, S3_TIMG0_BASE, TG_TIMER_CONFIG, UINT32_MAX);
    ASSERT_EQ(tg_read(mem, S3_TIMG0_BASE, TG_TIMER_CONFIG),
              0xFFFFE600u);
    tg_write(mem, S3_TIMG0_BASE, TG_DATE, UINT32_MAX);
    tg_write(mem, S3_TIMG0_BASE, TG_REGCLK, UINT32_MAX);
    ASSERT_EQ(tg_read(mem, S3_TIMG0_BASE, TG_DATE), 0x0FFFFFFFu);
    ASSERT_EQ(tg_read(mem, S3_TIMG0_BASE, TG_REGCLK), 0x80000000u);

    /* High halves are physically 22 bits on this 54-bit generation. */
    tg_write(mem, S3_TIMG0_BASE, TG_TIMER_ALARM_HI, UINT32_MAX);
    tg_write(mem, S3_TIMG0_BASE, TG_TIMER_LOAD_HI, UINT32_MAX);
    ASSERT_EQ(tg_read(mem, S3_TIMG0_BASE, TG_TIMER_ALARM_HI),
              0x003FFFFFu);
    ASSERT_EQ(tg_read(mem, S3_TIMG0_BASE, TG_TIMER_LOAD_HI),
              0x003FFFFFu);

    /* RTC calibration is a separate model layered into these same pages. */
    int before = periph_unhandled_count(periph);
    (void)tg_read(mem, S3_TIMG0_BASE, TG_RTC_CALI_CONFIG);
    tg_write(mem, S3_TIMG0_BASE, TG_RTC_CALI_CONFIG, 0u);
    ASSERT_EQ(periph_unhandled_count(periph), before);

    ASSERT_EQ(tg_read(mem, S3_TIMG0_BASE, 0x084u), 0u);
    tg_write(mem, S3_TIMG0_BASE, 0x084u, 1u);
    ASSERT_EQ(periph_unhandled_count(periph), before + 2);
    ASSERT_EQ(mem_unmapped_count(mem), 0u);

    periph_destroy(periph);
    mem_destroy(mem);
}

TEST(target_timer_group_counts_shared_time_and_routes_alarms)
{
    const flexe_target_desc_t *s3 =
        flexe_target_by_id(FLEXE_TARGET_ESP32S3);
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

    /* APB / 80 is 1 MHz. At 160 MHz, 1,600 target CPU cycles are ten
     * timer ticks. Sequentially stepping the second core does not double
     * count the shared passage of target time. */
    tg_load(mem, S3_TIMG0_BASE, 0u, 0u);
    tg_write(mem, S3_TIMG0_BASE, TG_TIMER_CONFIG,
             TG_TIMER_ENABLE | TG_TIMER_INCREASE | (80u << 13));
    cpu[0].ccount = 1600u;
    ASSERT_EQ(tg_capture(mem, S3_TIMG0_BASE, 0u), 10u);
    cpu[1].ccount = 800u;
    ASSERT_EQ(tg_capture(mem, S3_TIMG0_BASE, 0u), 10u);
    cpu[1].ccount = 2400u;
    ASSERT_EQ(tg_capture(mem, S3_TIMG0_BASE, 0u), 15u);

    /* XTAL / 40 is independently also 1 MHz and changing sources resets the
     * divider phase without disturbing the live counter. */
    tg_write(mem, S3_TIMG0_BASE, TG_TIMER_CONFIG,
             TG_TIMER_ENABLE | TG_TIMER_INCREASE | TG_TIMER_USE_XTAL |
             (40u << 13));
    cpu[1].ccount += 1600u;
    ASSERT_EQ(tg_capture(mem, S3_TIMG0_BASE, 0u), 25u);

    /* Arm an APB-clocked one-shot three ticks ahead. Alarm completion sets
     * RAW, clears ALARM_EN, reloads, and drives S3 interrupt source 50. */
    tg_load(mem, S3_TIMG0_BASE, 0u, 2u);
    tg_write(mem, S3_TIMG0_BASE, TG_TIMER_ALARM_LO, 5u);
    tg_write(mem, S3_TIMG0_BASE, TG_TIMER_ALARM_HI, 0u);
    tg_write(mem, S3_TIMG0_BASE, TG_INT_ENA, 1u);
    tg_write(mem, S3_TIMG0_BASE, TG_TIMER_CONFIG,
             TG_TIMER_ENABLE | TG_TIMER_INCREASE |
             TG_TIMER_AUTORELOAD | TG_TIMER_ALARM_EN | (80u << 13));
    ASSERT_EQ(cpu[1].next_timer_event, cpu[1].ccount + 480u);
    cpu[1].ccount += 479u;
    cpu[1].periph_event(&cpu[1]);
    ASSERT_EQ(tg_read(mem, S3_TIMG0_BASE, TG_INT_RAW), 0u);
    cpu[1].ccount++;
    cpu[1].periph_event(&cpu[1]);
    ASSERT_EQ(tg_read(mem, S3_TIMG0_BASE, TG_INT_RAW), 1u);
    ASSERT_EQ(tg_read(mem, S3_TIMG0_BASE, TG_INT_ST), 1u);
    ASSERT_TRUE(periph_interrupt_pending(periph, 50));
    ASSERT_EQ(tg_read(mem, S3_TIMG0_BASE, TG_TIMER_CONFIG) &
              TG_TIMER_ALARM_EN, 0u);
    ASSERT_EQ(tg_capture(mem, S3_TIMG0_BASE, 0u), 2u);
    tg_write(mem, S3_TIMG0_BASE, TG_INT_CLR, 1u);
    ASSERT_FALSE(periph_interrupt_pending(periph, 50));
    ASSERT_EQ(periph_unhandled_count(periph), 0);

    periph_destroy(periph);
    mem_destroy(mem);
}

TEST(target_timer_group_watchdog_feed_protection_interrupt_and_reset)
{
    const flexe_target_desc_t *s3 =
        flexe_target_by_id(FLEXE_TARGET_ESP32S3);
    xtensa_mem_t *mem = mem_create_for_target(s3);
    esp32_periph_t *periph = periph_create(mem);
    xtensa_cpu_t cpu;
    ASSERT_TRUE(mem != NULL);
    ASSERT_TRUE(periph != NULL);
    if (!mem || !periph) {
        periph_destroy(periph);
        mem_destroy(mem);
        return;
    }
    xtensa_cpu_init_for_target(&cpu, s3);
    cpu.mem = mem;
    periph_attach_cpus(periph, &cpu, NULL);

    /* Disable flash-boot protection while programming, then run the MWDT at
     * APB / 80 = 1 MHz: stage 0 interrupts after three ticks and stage 1
     * requests a system reset two ticks later. */
    uint32_t reset_lengths =
        tg_read(mem, S3_TIMG0_BASE, TG_WDT_CONFIG0) &
        ~TG_WDT_FLASHBOOT;
    tg_write(mem, S3_TIMG0_BASE, TG_WDT_CONFIG0, reset_lengths);
    tg_write(mem, S3_TIMG0_BASE, TG_WDT_CONFIG1, 80u << 16);
    tg_write(mem, S3_TIMG0_BASE, TG_WDT_CONFIG2, 3u);
    tg_write(mem, S3_TIMG0_BASE, TG_WDT_CONFIG3, 2u);
    tg_write(mem, S3_TIMG0_BASE, TG_INT_ENA, 1u << 2);
    tg_write(mem, S3_TIMG0_BASE, TG_WDT_CONFIG0,
             reset_lengths | TG_WDT_ENABLE | TG_WDT_STAGE0_INT |
             TG_WDT_STAGE1_RESET);
    ASSERT_EQ(cpu.next_timer_event, cpu.ccount + 480u);

    /* Feeding at two ticks restarts stage 0 rather than merely clearing its
     * eventual interrupt status. */
    cpu.ccount += 320u;
    cpu.periph_event(&cpu);
    tg_write(mem, S3_TIMG0_BASE, TG_WDT_FEED, 1u);
    cpu.ccount += 320u;
    cpu.periph_event(&cpu);
    ASSERT_EQ(tg_read(mem, S3_TIMG0_BASE, TG_INT_RAW) & (1u << 2), 0u);
    cpu.ccount += 160u;
    cpu.periph_event(&cpu);
    ASSERT_EQ(tg_read(mem, S3_TIMG0_BASE, TG_INT_RAW) & (1u << 2),
              1u << 2);
    ASSERT_TRUE(periph_interrupt_pending(periph, 52));
    tg_write(mem, S3_TIMG0_BASE, TG_INT_CLR, 1u << 2);
    ASSERT_FALSE(periph_interrupt_pending(periph, 52));

    /* A locked watchdog ignores both configuration writes and feed pulses.
     * The already-running stage 1 therefore reaches its reset action. */
    tg_write(mem, S3_TIMG0_BASE, TG_WDT_PROTECT, 0u);
    tg_write(mem, S3_TIMG0_BASE, TG_WDT_CONFIG3, 1000u);
    ASSERT_EQ(tg_read(mem, S3_TIMG0_BASE, TG_WDT_CONFIG3), 2u);
    tg_write(mem, S3_TIMG0_BASE, TG_WDT_FEED, 1u);
    cpu.ccount += 319u;
    cpu.periph_event(&cpu);
    ASSERT_FALSE(periph_take_reset_request(periph));
    cpu.ccount++;
    cpu.periph_event(&cpu);
    ASSERT_TRUE(periph_take_reset_request(periph));
    ASSERT_FALSE(periph_take_reset_request(periph));
    ASSERT_EQ(periph_unhandled_count(periph), 0);

    periph_destroy(periph);
    mem_destroy(mem);
}

void run_timer_group_target_tests(void)
{
    TEST_SUITE("Target timer groups and MWDT");
    RUN_TEST(target_timer_group_reset_masks_and_rtc_fallback);
    RUN_TEST(target_timer_group_counts_shared_time_and_routes_alarms);
    RUN_TEST(target_timer_group_watchdog_feed_protection_interrupt_and_reset);
}
