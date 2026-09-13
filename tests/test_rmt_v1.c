/* ESP32-S3 RMT V1 TX register, pulse-timing and interrupt tests. */
#include "test_helpers.h"
#include "peripherals.h"

#define S3_RMT_BASE       0x60016000u
#define S3_RMT_CONF0      0x020u
#define S3_RMT_CONF3      0x02Cu
#define S3_RMT_RX_CONF4   0x030u
#define S3_RMT_STATUS0    0x050u
#define S3_RMT_INT_RAW    0x070u
#define S3_RMT_INT_ST     0x074u
#define S3_RMT_INT_ENA    0x078u
#define S3_RMT_INT_CLR    0x07Cu
#define S3_RMT_TX_LIMIT0  0x0A0u
#define S3_RMT_SYS_CONF   0x0C0u
#define S3_RMT_TX_SIM     0x0C4u
#define S3_RMT_DATE       0x0CCu
#define S3_RMT_MEM0       0x800u

typedef struct {
    unsigned calls;
    unsigned items;
    unsigned completions;
    uint32_t last_item;
    uint32_t tick_hz;
    uint32_t carrier_hz;
} rmt_v1_probe_t;

static void rmt_v1_tx_observed(void *ctx, int channel,
                               const uint32_t *items, size_t count,
                               uint32_t tick_hz, uint32_t carrier_hz,
                               bool complete)
{
    rmt_v1_probe_t *probe = ctx;
    if (channel != 0) return;
    probe->calls++;
    probe->items += (unsigned)count;
    if (count) probe->last_item = items[count - 1u];
    probe->tick_hz = tick_hz;
    probe->carrier_hz = carrier_hz;
    if (complete) probe->completions++;
}

TEST(s3_rmt_v1_tx_completes_on_pulse_deadline_and_asserts_irq)
{
    const flexe_target_desc_t *s3 =
        flexe_target_by_id(FLEXE_TARGET_ESP32S3);
    xtensa_mem_t *mem = mem_create_for_target(s3);
    esp32_periph_t *periph = periph_create(mem);
    xtensa_cpu_t cpu0;
    ASSERT_TRUE(mem != NULL);
    ASSERT_TRUE(periph != NULL);
    if (!mem || !periph) {
        periph_destroy(periph);
        mem_destroy(mem);
        return;
    }
    xtensa_cpu_init_for_target(&cpu0, s3);
    cpu0.mem = mem;
    periph_attach_cpus(periph, &cpu0, NULL);
    rmt_v1_probe_t probe = {0};
    ASSERT_EQ(periph_set_rmt_tx_callback(periph, 0,
                                         rmt_v1_tx_observed, &probe), 0);
    ASSERT_EQ(mem_read32(mem, S3_RMT_BASE + S3_RMT_CONF0), 0x00710200u);
    ASSERT_EQ(mem_read32(mem, S3_RMT_BASE + S3_RMT_TX_LIMIT0), 128u);
    ASSERT_EQ(mem_read32(mem, S3_RMT_BASE + S3_RMT_STATUS0), 0u);
    ASSERT_EQ(mem_read32(mem, S3_RMT_BASE + S3_RMT_MEM0), 0u);

    const uint32_t pulse = 10u | (1u << 15) | (20u << 16);
    mem_write32(mem, S3_RMT_BASE + S3_RMT_MEM0, pulse);
    mem_write32(mem, S3_RMT_BASE + S3_RMT_MEM0 + 4u, 0u);
    mem_write32(mem, S3_RMT_BASE + S3_RMT_INT_ENA, 1u);
    mem_write32(mem, S3_RMT_BASE + S3_RMT_CONF0,
                mem_read32(mem, S3_RMT_BASE + S3_RMT_CONF0) | 1u);
    ASSERT_EQ((mem_read32(mem, S3_RMT_BASE + S3_RMT_STATUS0) >> 22) & 7u,
              1u);
    cpu0.ccount = 239u;
    ASSERT_EQ((mem_read32(mem, S3_RMT_BASE + S3_RMT_STATUS0) >> 22) & 7u,
              1u);
    ASSERT_EQ(mem_read32(mem, S3_RMT_BASE + S3_RMT_INT_RAW), 0u);
    cpu0.ccount = 240u;
    ASSERT_EQ((mem_read32(mem, S3_RMT_BASE + S3_RMT_STATUS0) >> 22) & 7u,
              0u);
    ASSERT_EQ(mem_read32(mem, S3_RMT_BASE + S3_RMT_INT_RAW), 1u);
    ASSERT_EQ(mem_read32(mem, S3_RMT_BASE + S3_RMT_INT_ST), 1u);
    ASSERT_TRUE(periph_interrupt_pending(periph, s3->rmt_v1.interrupt_source));
    ASSERT_EQ(probe.calls, 1u);
    ASSERT_EQ(probe.items, 1u);
    ASSERT_EQ(probe.completions, 1u);
    ASSERT_EQ(probe.last_item, pulse);
    ASSERT_EQ(probe.tick_hz, 20000000u);
    mem_write32(mem, S3_RMT_BASE + S3_RMT_INT_CLR, 1u);
    ASSERT_FALSE(periph_interrupt_pending(periph,
                                           s3->rmt_v1.interrupt_source));
    ASSERT_EQ(periph_unhandled_count(periph), 0u);
    periph_destroy(periph);
    mem_destroy(mem);
}

TEST(s3_rmt_v1_threshold_and_empty_terminator_are_distinct_events)
{
    const flexe_target_desc_t *s3 =
        flexe_target_by_id(FLEXE_TARGET_ESP32S3);
    xtensa_mem_t *mem = mem_create_for_target(s3);
    esp32_periph_t *periph = periph_create(mem);
    xtensa_cpu_t cpu0;
    ASSERT_TRUE(mem != NULL);
    ASSERT_TRUE(periph != NULL);
    if (!mem || !periph) {
        periph_destroy(periph);
        mem_destroy(mem);
        return;
    }
    xtensa_cpu_init_for_target(&cpu0, s3);
    cpu0.mem = mem;
    periph_attach_cpus(periph, &cpu0, NULL);
    rmt_v1_probe_t probe = {0};
    ASSERT_EQ(periph_set_rmt_tx_callback(periph, 0,
                                         rmt_v1_tx_observed, &probe), 0);
    uint32_t pulse = 10u | (20u << 16);
    mem_write32(mem, S3_RMT_BASE + S3_RMT_MEM0, pulse);
    mem_write32(mem, S3_RMT_BASE + S3_RMT_MEM0 + 4u, pulse);
    mem_write32(mem, S3_RMT_BASE + S3_RMT_MEM0 + 8u, 0u);
    mem_write32(mem, S3_RMT_BASE + S3_RMT_TX_LIMIT0, 2u);
    mem_write32(mem, S3_RMT_BASE + S3_RMT_INT_ENA,
                (1u << 8) | 1u);
    mem_write32(mem, S3_RMT_BASE + S3_RMT_CONF0,
                mem_read32(mem, S3_RMT_BASE + S3_RMT_CONF0) | 1u);
    cpu0.ccount = 479u;
    ASSERT_EQ(mem_read32(mem, S3_RMT_BASE + S3_RMT_INT_RAW), 0u);
    cpu0.ccount = 480u;
    ASSERT_EQ(mem_read32(mem, S3_RMT_BASE + S3_RMT_INT_RAW), 1u << 8);
    ASSERT_EQ((mem_read32(mem, S3_RMT_BASE + S3_RMT_STATUS0) >> 22) & 7u,
              1u);
    ASSERT_EQ(probe.calls, 1u);
    ASSERT_EQ(probe.items, 2u);
    ASSERT_EQ(probe.completions, 0u);
    mem_write32(mem, S3_RMT_BASE + S3_RMT_INT_CLR, 1u << 8);
    cpu0.ccount = 481u;
    ASSERT_EQ(mem_read32(mem, S3_RMT_BASE + S3_RMT_INT_RAW), 1u);
    ASSERT_EQ((mem_read32(mem, S3_RMT_BASE + S3_RMT_STATUS0) >> 22) & 7u,
              0u);
    ASSERT_EQ(probe.calls, 2u);
    ASSERT_EQ(probe.items, 2u);
    ASSERT_EQ(probe.completions, 1u);
    ASSERT_EQ(periph_unhandled_count(periph), 0u);
    periph_destroy(periph);
    mem_destroy(mem);
}

TEST(s3_rmt_v1_fractional_divider_uses_exact_pulse_deadline)
{
    const flexe_target_desc_t *s3 =
        flexe_target_by_id(FLEXE_TARGET_ESP32S3);
    xtensa_mem_t *mem = mem_create_for_target(s3);
    esp32_periph_t *periph = periph_create(mem);
    xtensa_cpu_t cpu0;
    ASSERT_TRUE(mem != NULL);
    ASSERT_TRUE(periph != NULL);
    if (!mem || !periph) {
        periph_destroy(periph);
        mem_destroy(mem);
        return;
    }
    xtensa_cpu_init_for_target(&cpu0, s3);
    cpu0.mem = mem;
    periph_attach_cpus(periph, &cpu0, NULL);
    rmt_v1_probe_t probe = {0};
    ASSERT_EQ(periph_set_rmt_tx_callback(periph, 0,
                                         rmt_v1_tx_observed, &probe), 0);
    ASSERT_EQ(mem_read32(mem, S3_RMT_BASE + S3_RMT_DATE), 34607489u);
    /* Source 80 MHz / SCLK divider (1 + A/B = 1.5) / channel divider 2:
     * three RMT ticks take exactly 18 CPU cycles at 160 MHz. */
    mem_write32(mem, S3_RMT_BASE + S3_RMT_SYS_CONF, 0x05081000u);
    mem_write32(mem, S3_RMT_BASE + S3_RMT_MEM0, 3u | (1u << 15));
    mem_write32(mem, S3_RMT_BASE + S3_RMT_CONF0,
                mem_read32(mem, S3_RMT_BASE + S3_RMT_CONF0) | 1u);
    cpu0.ccount = 17u;
    ASSERT_EQ(mem_read32(mem, S3_RMT_BASE + S3_RMT_INT_RAW), 0u);
    cpu0.ccount = 18u;
    ASSERT_EQ(mem_read32(mem, S3_RMT_BASE + S3_RMT_INT_RAW), 1u);
    ASSERT_EQ(probe.tick_hz, 26666666u);
    ASSERT_EQ(probe.completions, 1u);
    ASSERT_EQ(periph_unhandled_count(periph), 0u);
    periph_destroy(periph);
    mem_destroy(mem);
}

TEST(s3_rmt_v1_unmodeled_modes_remain_diagnostic)
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
    mem_write32(mem, S3_RMT_BASE + S3_RMT_CONF0, 1u << 3);
    mem_write32(mem, S3_RMT_BASE + S3_RMT_CONF3, 1u << 25);
    mem_write32(mem, S3_RMT_BASE + S3_RMT_TX_SIM, 1u << 4);
    mem_write32(mem, S3_RMT_BASE + S3_RMT_RX_CONF4, 1u);
    ASSERT_EQ(periph_unhandled_count(periph), 4u);
    ASSERT_EQ(mem_read32(mem, S3_RMT_BASE + S3_RMT_CONF3) & (1u << 25),
              0u);
    periph_destroy(periph);
    mem_destroy(mem);
}

static void run_rmt_v1_tests(void)
{
    TEST_SUITE("ESP32-S3 RMT V1");
    RUN_TEST(s3_rmt_v1_tx_completes_on_pulse_deadline_and_asserts_irq);
    RUN_TEST(s3_rmt_v1_threshold_and_empty_terminator_are_distinct_events);
    RUN_TEST(s3_rmt_v1_fractional_divider_uses_exact_pulse_deadline);
    RUN_TEST(s3_rmt_v1_unmodeled_modes_remain_diagnostic);
}
