/* Descriptor-driven ESP32-family SYSTIMER tests. */
#include "test_helpers.h"
#include "peripherals.h"

#define S3_SYSTIMER_BASE       0x60023000u
#define ST_CONF                0x000u
#define ST_UNIT0_OP            0x004u
#define ST_UNIT1_OP            0x008u
#define ST_UNIT0_LOAD_HI       0x00Cu
#define ST_UNIT0_LOAD_LO       0x010u
#define ST_UNIT1_LOAD_HI       0x014u
#define ST_UNIT1_LOAD_LO       0x018u
#define ST_TARGET0_HI          0x01Cu
#define ST_TARGET0_LO          0x020u
#define ST_TARGET1_CONF        0x038u
#define ST_UNIT0_VALUE_HI      0x040u
#define ST_UNIT0_VALUE_LO      0x044u
#define ST_UNIT1_VALUE_LO      0x04Cu
#define ST_COMP0_LOAD          0x050u
#define ST_COMP1_LOAD          0x054u
#define ST_UNIT0_LOAD          0x05Cu
#define ST_UNIT1_LOAD          0x060u
#define ST_INT_ENA             0x064u
#define ST_INT_RAW             0x068u
#define ST_INT_CLR             0x06Cu
#define ST_INT_ST              0x070u
#define ST_REAL_TARGET1_LO     0x07Cu
#define ST_REAL_TARGET1_HI     0x080u
#define ST_DATE                0x0FCu

#define ST_COUNTER0_ENABLE     (1u << 30)
#define ST_COUNTER1_ENABLE     (1u << 29)
#define ST_TARGET0_ENABLE      (1u << 24)
#define ST_TARGET1_ENABLE      (1u << 23)
#define ST_VALUE_VALID         (1u << 29)
#define ST_VALUE_UPDATE        (1u << 30)
#define ST_PERIOD_MODE         (1u << 30)
#define ST_UNIT1_SELECT        (1u << 31)

static uint32_t st_read(xtensa_mem_t *mem, uint32_t off)
{
    return mem_read32(mem, S3_SYSTIMER_BASE + off);
}

static void st_write(xtensa_mem_t *mem, uint32_t off, uint32_t value)
{
    mem_write32(mem, S3_SYSTIMER_BASE + off, value);
}

TEST(systimer_snapshots_loads_and_tracks_shared_target_time) {
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
    periph_attach_cpus(periph, &cpu[0], &cpu[1]);

    ASSERT_EQ(st_read(mem, ST_CONF), 0x46000000u);
    ASSERT_EQ(st_read(mem, ST_DATE), 0x02012251u);
    ASSERT_EQ(st_read(mem, ST_UNIT0_OP), 0u);

    /* The 16 MHz timer advances once per ten cycles at the S3's 160 MHz
     * application clock, and two cores share one timeline rather than
     * advancing the peripheral twice. */
    cpu[0].ccount = 100u;
    cpu[1].ccount = 40u;
    st_write(mem, ST_UNIT0_OP, ST_VALUE_UPDATE);
    ASSERT_EQ(st_read(mem, ST_UNIT0_OP), ST_VALUE_VALID);
    ASSERT_EQ(st_read(mem, ST_UNIT0_VALUE_HI), 0u);
    ASSERT_EQ(st_read(mem, ST_UNIT0_VALUE_LO), 10u);

    cpu[1].ccount = 200u;
    ASSERT_EQ(st_read(mem, ST_UNIT0_VALUE_LO), 10u); /* captured, not live */
    st_write(mem, ST_UNIT0_OP, ST_VALUE_UPDATE);
    ASSERT_EQ(st_read(mem, ST_UNIT0_VALUE_LO), 20u);
    st_write(mem, ST_UNIT0_OP, ST_VALUE_VALID);
    ASSERT_EQ(st_read(mem, ST_UNIT0_OP), 0u);

    st_write(mem, ST_UNIT0_LOAD_HI, 0xABC00001u);
    st_write(mem, ST_UNIT0_LOAD_LO, 0x23456789u);
    st_write(mem, ST_UNIT0_LOAD, 1u);
    st_write(mem, ST_UNIT0_OP, ST_VALUE_UPDATE);
    ASSERT_EQ(st_read(mem, ST_UNIT0_VALUE_HI), 1u);
    ASSERT_EQ(st_read(mem, ST_UNIT0_VALUE_LO), 0x23456789u);

    /* Unit 1 is stopped at reset, then begins at the exact enable boundary. */
    cpu[0].ccount = 300u;
    st_write(mem, ST_UNIT1_OP, ST_VALUE_UPDATE);
    ASSERT_EQ(st_read(mem, ST_UNIT1_VALUE_LO), 0u);
    st_write(mem, ST_CONF, st_read(mem, ST_CONF) | ST_COUNTER1_ENABLE);
    cpu[0].ccount = 400u;
    st_write(mem, ST_UNIT1_OP, ST_VALUE_UPDATE);
    ASSERT_EQ(st_read(mem, ST_UNIT1_VALUE_LO), 10u);

    /* The fixed 16 MHz timer keeps its rate when firmware lowers the CPU
     * clock; the target's ROM ABI word supplies the new conversion ratio. */
    mem_write32(mem, s3->cpu_frequency_word, 80u);
    cpu[0].ccount = 480u;
    st_write(mem, ST_UNIT1_OP, ST_VALUE_UPDATE);
    ASSERT_EQ(st_read(mem, ST_UNIT1_VALUE_LO), 26u);
    ASSERT_EQ(periph_unhandled_count(periph), 0);

    periph_destroy(periph);
    mem_destroy(mem);
}

TEST(systimer_oneshot_periodic_alarms_and_interrupt_status) {
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

    st_write(mem, ST_TARGET0_HI, 0u);
    st_write(mem, ST_TARGET0_LO, 16u);
    st_write(mem, ST_COMP0_LOAD, 1u);
    st_write(mem, ST_INT_ENA, 1u);
    st_write(mem, ST_CONF, st_read(mem, ST_CONF) | ST_TARGET0_ENABLE);
    ASSERT_EQ(cpu.next_timer_event, 160u);

    cpu.ccount = 159u;
    cpu.periph_event(&cpu);
    ASSERT_EQ(st_read(mem, ST_INT_RAW), 0u);
    cpu.ccount = 160u;
    cpu.periph_event(&cpu);
    ASSERT_EQ(st_read(mem, ST_INT_RAW), 1u);
    ASSERT_EQ(st_read(mem, ST_INT_ST), 1u);
    ASSERT_EQ(st_read(mem, ST_CONF) & ST_TARGET0_ENABLE, 0u);
    ASSERT_TRUE(periph_interrupt_pending(periph, 57));
    st_write(mem, ST_INT_CLR, 1u);
    ASSERT_EQ(st_read(mem, ST_INT_RAW), 0u);
    ASSERT_FALSE(periph_interrupt_pending(periph, 57));

    /* Alarm 1 periodically compares against independently loadable unit 1. */
    st_write(mem, ST_UNIT1_LOAD_HI, 0u);
    st_write(mem, ST_UNIT1_LOAD_LO, 0u);
    st_write(mem, ST_UNIT1_LOAD, 1u);
    /* Match ESP-IDF's HAL ordering: it synchronizes the period while the
     * comparator is still in one-shot mode, enables WORK_EN, and selects
     * periodic mode afterward. The untouched one-shot target must not cause
     * an immediate interrupt that disables the alarm. */
    st_write(mem, ST_TARGET1_CONF, ST_UNIT1_SELECT | 8u);
    st_write(mem, ST_COMP1_LOAD, 1u);
    st_write(mem, ST_INT_ENA, 1u << 1);
    st_write(mem, ST_CONF, st_read(mem, ST_CONF) |
             ST_COUNTER1_ENABLE | ST_TARGET1_ENABLE);
    ASSERT_EQ(st_read(mem, ST_INT_RAW), 0u);
    ASSERT_TRUE(st_read(mem, ST_CONF) & ST_TARGET1_ENABLE);
    ASSERT_EQ(st_read(mem, ST_REAL_TARGET1_LO), 8u);
    st_write(mem, ST_TARGET1_CONF,
             ST_UNIT1_SELECT | ST_PERIOD_MODE | 8u);
    ASSERT_EQ(cpu.next_timer_event, 240u);

    cpu.ccount = 240u;
    cpu.periph_event(&cpu);
    ASSERT_EQ(st_read(mem, ST_INT_RAW), 1u << 1);
    ASSERT_EQ(st_read(mem, ST_REAL_TARGET1_LO), 16u);
    ASSERT_EQ(st_read(mem, ST_REAL_TARGET1_HI), 0u);
    ASSERT_TRUE(st_read(mem, ST_CONF) & ST_TARGET1_ENABLE);
    ASSERT_TRUE(periph_interrupt_pending(periph, 58));
    st_write(mem, ST_INT_CLR, 1u << 1);

    /* A delayed evaluation coalesces the level interrupt but advances the
     * periodic target past every elapsed period. */
    cpu.ccount = 400u;
    cpu.periph_event(&cpu);
    ASSERT_EQ(st_read(mem, ST_INT_RAW), 1u << 1);
    ASSERT_EQ(st_read(mem, ST_REAL_TARGET1_LO), 32u);

    int before = periph_unhandled_count(periph);
    ASSERT_EQ(st_read(mem, 0x090u), 0u);
    st_write(mem, 0x090u, 1u);
    ASSERT_EQ(periph_unhandled_count(periph), before + 2);
    ASSERT_EQ(mem_unmapped_count(mem), 0u);

    periph_destroy(periph);
    mem_destroy(mem);
}

void run_systimer_tests(void) {
    TEST_SUITE("Target system timer");
    RUN_TEST(systimer_snapshots_loads_and_tracks_shared_target_time);
    RUN_TEST(systimer_oneshot_periodic_alarms_and_interrupt_status);
}
