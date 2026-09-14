/* ESP32-S3 RMT V1 TX/RX register, pulse-timing and interrupt tests. */
#include "test_helpers.h"
#include "peripherals.h"

#define S3_RMT_BASE       0x60016000u
#define S3_RMT_CONF0      0x020u
#define S3_RMT_CONF3      0x02Cu
#define S3_RMT_RX_CONF4   0x030u
#define S3_RMT_RX_CTRL4   0x034u
#define S3_RMT_STATUS0    0x050u
#define S3_RMT_RX_STATUS4 0x060u
#define S3_RMT_INT_RAW    0x070u
#define S3_RMT_INT_ST     0x074u
#define S3_RMT_INT_ENA    0x078u
#define S3_RMT_INT_CLR    0x07Cu
#define S3_RMT_CARRIER0   0x080u
#define S3_RMT_RX_CARRIER4 0x090u
#define S3_RMT_TX_LIMIT0  0x0A0u
#define S3_RMT_RX_LIMIT4  0x0B0u
#define S3_RMT_SYS_CONF   0x0C0u
#define S3_RMT_TX_SIM     0x0C4u
#define S3_RMT_REF_RST    0x0C8u
#define S3_RMT_DATE       0x0CCu
#define S3_RMT_MEM0       0x800u
#define S3_RMT_RX_MEM4    0xB00u
#define S3_GPIO_BASE      0x60004000u
#define S3_GPIO_RMT_RX0   (0x154u + 81u * 4u)
#define S3_GPIO_FUNC_OUT4 (0x554u + 4u * 4u)

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

TEST(s3_rmt_v1_rx_decoded_symbols_complete_after_idle_and_raise_irq)
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

    ASSERT_EQ(mem_read32(mem, S3_RMT_BASE + S3_RMT_RX_CONF4),
              0x317FFF02u);
    ASSERT_EQ(mem_read32(mem, S3_RMT_BASE + S3_RMT_RX_CTRL4),
              0x000001E8u);
    ASSERT_EQ(mem_read32(mem, S3_RMT_BASE + S3_RMT_RX_LIMIT4), 128u);
    mem_write32(mem, S3_RMT_BASE + S3_RMT_REF_RST, 1u << 4);
    ASSERT_EQ(mem_read32(mem, S3_RMT_BASE + S3_RMT_REF_RST), 0u);
    ASSERT_EQ(mem_read32(mem, S3_RMT_BASE + S3_RMT_RX_STATUS4) & 0x3FFu,
              192u);
    mem_write32(mem, S3_RMT_BASE + S3_RMT_RX_CONF4,
                (1u << 24) | (5u << 8) | 2u);
    mem_write32(mem, S3_RMT_BASE + S3_RMT_RX_CTRL4,
                (1u << 15) | (1u << 3) | 1u);
    mem_write32(mem, S3_RMT_BASE + S3_RMT_INT_ENA, 1u << 16);
    const uint32_t symbols[] = {
        10u | (1u << 15) | (20u << 16),
        5u | (5u << 16),
    };
    ASSERT_EQ(periph_rmt_rx_inject(periph, 0, symbols, 2u), 0u);
    ASSERT_EQ(periph_rmt_rx_inject(periph, 4, symbols, 2u), 2u);
    ASSERT_EQ(mem_read32(mem, S3_RMT_BASE + S3_RMT_RX_MEM4), 0u);
    ASSERT_EQ(mem_read32(mem, S3_RMT_BASE + S3_RMT_RX_STATUS4) & 0x3FFu,
              192u);
    ASSERT_EQ((mem_read32(mem, S3_RMT_BASE + S3_RMT_RX_STATUS4) >> 22) & 7u,
              1u);
    /* 30 + 10 + 5 symbol/idle ticks at 8 CPU cycles per RMT tick. */
    cpu0.ccount = 239u;
    ASSERT_EQ(mem_read32(mem, S3_RMT_BASE + S3_RMT_RX_STATUS4) & 0x3FFu,
              192u);
    cpu0.ccount = 240u;
    ASSERT_EQ(mem_read32(mem, S3_RMT_BASE + S3_RMT_RX_MEM4), symbols[0]);
    ASSERT_EQ(mem_read32(mem, S3_RMT_BASE + S3_RMT_RX_STATUS4) & 0x3FFu,
              193u);
    cpu0.ccount = 319u;
    ASSERT_EQ(mem_read32(mem, S3_RMT_BASE + S3_RMT_RX_MEM4 + 4u), 0u);
    cpu0.ccount = 320u;
    ASSERT_EQ(mem_read32(mem, S3_RMT_BASE + S3_RMT_RX_MEM4 + 4u),
              symbols[1]);
    ASSERT_EQ(mem_read32(mem, S3_RMT_BASE + S3_RMT_RX_STATUS4) & 0x3FFu,
              194u);
    cpu0.ccount = 359u;
    ASSERT_EQ(mem_read32(mem, S3_RMT_BASE + S3_RMT_INT_RAW), 0u);
    cpu0.ccount = 360u;
    ASSERT_EQ(mem_read32(mem, S3_RMT_BASE + S3_RMT_INT_RAW), 1u << 16);
    ASSERT_EQ(mem_read32(mem, S3_RMT_BASE + S3_RMT_INT_ST), 1u << 16);
    ASSERT_EQ((mem_read32(mem, S3_RMT_BASE + S3_RMT_RX_STATUS4) >> 22) & 7u,
              0u);
    ASSERT_TRUE(periph_interrupt_pending(periph,
                                          s3->rmt_v1.interrupt_source));
    mem_write32(mem, S3_RMT_BASE + S3_RMT_INT_CLR, 1u << 16);
    ASSERT_FALSE(periph_interrupt_pending(periph,
                                           s3->rmt_v1.interrupt_source));
    ASSERT_EQ(periph_unhandled_count(periph), 0u);
    periph_destroy(periph);
    mem_destroy(mem);
}

TEST(s3_rmt_v1_rx_captures_gpio_matrix_edges_in_guest_time)
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

    /* RX0 is physical channel 4. GPIO4 routes to S3 matrix input 81. */
    mem_write32(mem, s3->io_mux.base +
                     s3->io_mux.gpio_register_offset[4],
                s3->io_mux.input_enable_mask);
    mem_write32(mem, S3_GPIO_BASE + S3_GPIO_RMT_RX0, 0x80u | 4u);
    mem_write32(mem, S3_RMT_BASE + S3_RMT_RX_CONF4,
                (1u << 24) | (20u << 8) | 2u);
    mem_write32(mem, S3_RMT_BASE + S3_RMT_RX_CTRL4,
                (1u << 15) | (1u << 3) | 1u);
    mem_write32(mem, S3_RMT_BASE + S3_RMT_INT_ENA, 1u << 16);
    periph_gpio_set_input(periph, 5, 1); /* unrelated pad */
    ASSERT_EQ(mem_read32(mem, S3_RMT_BASE + S3_RMT_RX_MEM4), 0u);

    periph_gpio_set_input(periph, 4, 1); /* first rising edge */
    cpu0.ccount = 80u;
    periph_gpio_set_input(periph, 4, 0); /* 10 high ticks */
    cpu0.ccount = 176u;
    periph_gpio_set_input(periph, 4, 1); /* 12 low ticks */
    ASSERT_EQ(mem_read32(mem, S3_RMT_BASE + S3_RMT_RX_MEM4),
              10u | (1u << 15) | (12u << 16));
    cpu0.ccount = 240u;
    periph_gpio_set_input(periph, 4, 0); /* 8 high ticks */
    cpu0.ccount = 312u;
    periph_gpio_set_input(periph, 4, 1); /* 9 low ticks */
    ASSERT_EQ(mem_read32(mem, S3_RMT_BASE + S3_RMT_RX_MEM4 + 4u),
              8u | (1u << 15) | (9u << 16));
    cpu0.ccount = 471u;
    ASSERT_EQ(mem_read32(mem, S3_RMT_BASE + S3_RMT_INT_RAW), 0u);
    cpu0.ccount = 472u;
    ASSERT_EQ(mem_read32(mem, S3_RMT_BASE + S3_RMT_INT_ST), 1u << 16);
    ASSERT_EQ(mem_read32(mem, S3_RMT_BASE + S3_RMT_RX_STATUS4) & 0x3FFu,
              194u);
    ASSERT_EQ(periph_unhandled_count(periph), 0u);

    /* Re-arm with an inverted matrix input. Physical low is now logical
     * high, so the receiver must still encode the same high/low pair. */
    mem_write32(mem, S3_RMT_BASE + S3_RMT_INT_CLR, 1u << 16);
    mem_write32(mem, S3_GPIO_BASE + S3_GPIO_RMT_RX0, 0xC0u | 4u);
    mem_write32(mem, S3_RMT_BASE + S3_RMT_RX_CTRL4,
                (1u << 15) | (1u << 3) | (1u << 1) | 1u);
    periph_gpio_set_input(periph, 4, 0);
    cpu0.ccount = 552u;
    periph_gpio_set_input(periph, 4, 1);
    cpu0.ccount = 648u;
    periph_gpio_set_input(periph, 4, 0);
    ASSERT_EQ(mem_read32(mem, S3_RMT_BASE + S3_RMT_RX_MEM4),
              10u | (1u << 15) | (12u << 16));
    cpu0.ccount = 808u;
    ASSERT_EQ(mem_read32(mem, S3_RMT_BASE + S3_RMT_INT_ST), 1u << 16);
    ASSERT_EQ(mem_read32(mem, S3_RMT_BASE + S3_RMT_RX_STATUS4) & 0x3FFu,
              193u);
    ASSERT_EQ(periph_unhandled_count(periph), 0u);

    periph_destroy(periph);
    mem_destroy(mem);
}

TEST(s3_rmt_v1_rx_captures_gpio_output_loopback_edges)
{
    const flexe_target_desc_t *s3 =
        flexe_target_by_id(FLEXE_TARGET_ESP32S3);
    xtensa_mem_t *mem = mem_create_for_target(s3);
    esp32_periph_t *periph = mem ? periph_create(mem) : NULL;
    xtensa_cpu_t cpu0;
    ASSERT_TRUE(periph != NULL);
    if (!periph) {
        mem_destroy(mem);
        return;
    }
    xtensa_cpu_init_for_target(&cpu0, s3);
    cpu0.mem = mem;
    periph_attach_cpus(periph, &cpu0, NULL);

    uint32_t mask = 1u << 4u;
    uint32_t mux4 = s3->io_mux.base +
                    s3->io_mux.gpio_register_offset[4];
    mem_write32(mem, S3_GPIO_BASE + S3_GPIO_RMT_RX0, 0x80u | 4u);
    mem_write32(mem, S3_GPIO_BASE + 0x024u, mask);
    mem_write32(mem, S3_RMT_BASE + S3_RMT_RX_CONF4,
                (1u << 24) | (20u << 8) | 2u);
    mem_write32(mem, S3_RMT_BASE + S3_RMT_RX_CTRL4,
                (1u << 15) | (1u << 3) | 1u);

    /* A driven output is visible to GPIO_IN only after FUN_IE. Neither the
     * disabled input nor its RMT matrix route should see these toggles. */
    mem_write32(mem, S3_GPIO_BASE + 0x008u, mask);
    cpu0.ccount = 80u;
    mem_write32(mem, S3_GPIO_BASE + 0x00Cu, mask);
    ASSERT_EQ(mem_read32(mem, S3_RMT_BASE + S3_RMT_RX_MEM4), 0u);
    mem_write32(mem, mux4, s3->io_mux.input_enable_mask);

    cpu0.ccount = 100u;
    mem_write32(mem, S3_GPIO_BASE + 0x008u, mask);
    ASSERT_EQ(mem_read32(mem, S3_GPIO_BASE + 0x03Cu) & mask, mask);
    cpu0.ccount = 180u;
    mem_write32(mem, S3_GPIO_BASE + 0x00Cu, mask);
    cpu0.ccount = 276u;
    mem_write32(mem, S3_GPIO_BASE + 0x008u, mask);
    ASSERT_EQ(mem_read32(mem, S3_RMT_BASE + S3_RMT_RX_MEM4),
              10u | (1u << 15) | (12u << 16));
    ASSERT_EQ(periph_unhandled_count(periph), 0u);

    periph_destroy(periph);
    mem_destroy(mem);
}

TEST(s3_rmt_v1_rx_captures_rmt_tx_pad_waveform)
{
    const flexe_target_desc_t *s3 =
        flexe_target_by_id(FLEXE_TARGET_ESP32S3);
    xtensa_mem_t *mem = mem_create_for_target(s3);
    esp32_periph_t *periph = mem ? periph_create(mem) : NULL;
    xtensa_cpu_t cpu0;
    ASSERT_TRUE(periph != NULL);
    if (!periph) {
        mem_destroy(mem);
        return;
    }
    xtensa_cpu_init_for_target(&cpu0, s3);
    cpu0.mem = mem;
    periph_attach_cpus(periph, &cpu0, NULL);

    uint32_t mask = 1u << 4u;
    mem_write32(mem, s3->io_mux.base +
                     s3->io_mux.gpio_register_offset[4],
                s3->io_mux.input_enable_mask);
    mem_write32(mem, S3_GPIO_BASE + S3_GPIO_RMT_RX0, 0x80u | 4u);
    mem_write32(mem, S3_GPIO_BASE + S3_GPIO_FUNC_OUT4,
                s3->rmt_v1.output_signal_base);
    mem_write32(mem, S3_RMT_BASE + S3_RMT_RX_CONF4,
                (1u << 24) | (20u << 8) | 2u);
    mem_write32(mem, S3_RMT_BASE + S3_RMT_RX_CTRL4,
                (1u << 15) | (1u << 3) | 1u);

    const uint32_t first = 10u | (1u << 15) | (12u << 16);
    const uint32_t second = 8u | (1u << 15) | (9u << 16);
    mem_write32(mem, S3_RMT_BASE + S3_RMT_MEM0, first);
    mem_write32(mem, S3_RMT_BASE + S3_RMT_MEM0 + 4u, second);
    mem_write32(mem, S3_RMT_BASE + S3_RMT_MEM0 + 8u, 0u);
    /* Unmodulated TX, idle output held low, 8 CPU cycles per RMT tick. */
    uint32_t conf = mem_read32(mem, S3_RMT_BASE + S3_RMT_CONF0);
    mem_write32(mem, S3_RMT_BASE + S3_RMT_CONF0,
                (conf & ~(1u << 21)) | (1u << 6) | 1u);
    ASSERT_EQ(mem_read32(mem, S3_GPIO_BASE + 0x03Cu) & mask, mask);
    cpu0.ccount = 79u;
    ASSERT_EQ(mem_read32(mem, S3_GPIO_BASE + 0x03Cu) & mask, mask);
    cpu0.ccount = 80u;
    (void)mem_read32(mem, S3_RMT_BASE + S3_RMT_STATUS0);
    ASSERT_EQ(mem_read32(mem, S3_GPIO_BASE + 0x03Cu) & mask, 0u);
    cpu0.ccount = 175u;
    ASSERT_EQ(mem_read32(mem, S3_RMT_BASE + S3_RMT_RX_MEM4), 0u);
    cpu0.ccount = 176u;
    (void)mem_read32(mem, S3_RMT_BASE + S3_RMT_STATUS0);
    ASSERT_EQ(mem_read32(mem, S3_GPIO_BASE + 0x03Cu) & mask, mask);
    ASSERT_EQ(mem_read32(mem, S3_RMT_BASE + S3_RMT_RX_MEM4), first);
    cpu0.ccount = 312u;
    (void)mem_read32(mem, S3_RMT_BASE + S3_RMT_STATUS0);
    ASSERT_EQ(mem_read32(mem, S3_GPIO_BASE + 0x03Cu) & mask, 0u);
    ASSERT_EQ(mem_read32(mem, S3_RMT_BASE + S3_RMT_INT_RAW) & 1u, 1u);
    ASSERT_EQ(periph_unhandled_count(periph), 0u);

    /* A delayed device evaluation must replay each edge at its own guest
     * cycle, not collapse the pulse train at the CPU's eventual read time.
     * Inverting both matrix sides preserves the logical RX symbols. */
    cpu0.ccount = 400u;
    (void)mem_read32(mem, S3_RMT_BASE + S3_RMT_INT_RAW);
    mem_write32(mem, S3_GPIO_BASE + S3_GPIO_FUNC_OUT4,
                s3->rmt_v1.output_signal_base | (1u << 9u));
    mem_write32(mem, S3_GPIO_BASE + S3_GPIO_RMT_RX0,
                0xC0u | 4u);
    mem_write32(mem, S3_RMT_BASE + S3_RMT_RX_CTRL4,
                (1u << 15) | (1u << 3) | (1u << 1) | 1u);
    mem_write32(mem, S3_RMT_BASE + S3_RMT_CONF0,
                (conf & ~(1u << 21)) | (1u << 6) | 1u);
    cpu0.ccount = 640u;
    ASSERT_EQ(mem_read32(mem, S3_RMT_BASE + S3_RMT_RX_MEM4), first);
    ASSERT_EQ(mem_read32(mem, S3_GPIO_BASE + 0x03Cu) & mask, mask);
    ASSERT_EQ(periph_unhandled_count(periph), 0u);

    periph_destroy(periph);
    mem_destroy(mem);
}

TEST(s3_rmt_v1_gpio_in_polls_unobserved_tx_without_edge_events)
{
    const flexe_target_desc_t *s3 =
        flexe_target_by_id(FLEXE_TARGET_ESP32S3);
    xtensa_mem_t *mem = mem_create_for_target(s3);
    esp32_periph_t *periph = mem ? periph_create(mem) : NULL;
    xtensa_cpu_t cpu0;
    ASSERT_TRUE(periph != NULL);
    if (!periph) {
        mem_destroy(mem);
        return;
    }
    xtensa_cpu_init_for_target(&cpu0, s3);
    cpu0.mem = mem;
    periph_attach_cpus(periph, &cpu0, NULL);

    const uint32_t mask = 1u << 4u;
    const uint32_t first = 10u | (1u << 15) | (12u << 16);
    const uint32_t second = 8u | (1u << 15) | (9u << 16);
    mem_write32(mem, s3->io_mux.base +
                     s3->io_mux.gpio_register_offset[4],
                s3->io_mux.input_enable_mask);
    mem_write32(mem, S3_GPIO_BASE + S3_GPIO_FUNC_OUT4,
                s3->rmt_v1.output_signal_base);
    mem_write32(mem, S3_RMT_BASE + S3_RMT_MEM0, first);
    mem_write32(mem, S3_RMT_BASE + S3_RMT_MEM0 + 4u, second);
    mem_write32(mem, S3_RMT_BASE + S3_RMT_MEM0 + 8u, 0u);
    uint32_t conf = mem_read32(mem, S3_RMT_BASE + S3_RMT_CONF0);
    conf = (conf & ~(1u << 21u)) | (1u << 6u);
    mem_write32(mem, S3_RMT_BASE + S3_RMT_CONF0, conf | 1u);
    ASSERT_EQ(mem_read32(mem, S3_GPIO_BASE + 0x03Cu) & mask, mask);
    cpu0.ccount = 79u;
    ASSERT_EQ(mem_read32(mem, S3_GPIO_BASE + 0x03Cu) & mask, mask);
    mem_write32(mem, s3->io_mux.base +
                     s3->io_mux.gpio_register_offset[4], 0u);
    ASSERT_EQ(mem_read32(mem, S3_GPIO_BASE + 0x03Cu) & mask, 0u);
    mem_write32(mem, s3->io_mux.base +
                     s3->io_mux.gpio_register_offset[4],
                s3->io_mux.input_enable_mask);
    ASSERT_EQ(mem_read32(mem, S3_GPIO_BASE + 0x03Cu) & mask, mask);
    cpu0.ccount = 80u;
    ASSERT_EQ(mem_read32(mem, S3_GPIO_BASE + 0x03Cu) & mask, 0u);
    cpu0.ccount = 175u;
    ASSERT_EQ(mem_read32(mem, S3_GPIO_BASE + 0x03Cu) & mask, 0u);
    cpu0.ccount = 176u;
    ASSERT_EQ(mem_read32(mem, S3_GPIO_BASE + 0x03Cu) & mask, mask);
    cpu0.ccount = 240u;
    ASSERT_EQ(mem_read32(mem, S3_GPIO_BASE + 0x03Cu) & mask, 0u);
    cpu0.ccount = 312u;
    ASSERT_EQ(mem_read32(mem, S3_GPIO_BASE + 0x03Cu) & mask, 0u);
    ASSERT_EQ(mem_read32(mem, S3_RMT_BASE + S3_RMT_INT_RAW) & 1u, 1u);
    ASSERT_EQ(periph_unhandled_count(periph), 0u);

    /* Matrix inversion and software output-enable selection apply after
     * sampling the RMT signal, not to pulse RAM. A one-word TX limit also
     * makes the poll cross a threshold-refill segment boundary. */
    cpu0.ccount = 400u;
    mem_write32(mem, S3_RMT_BASE + S3_RMT_TX_LIMIT0, 1u);
    mem_write32(mem, S3_GPIO_BASE + S3_GPIO_FUNC_OUT4,
                s3->rmt_v1.output_signal_base | (1u << 9u) | (1u << 10u));
    mem_write32(mem, S3_RMT_BASE + S3_RMT_CONF0, conf | 1u);
    ASSERT_EQ(mem_read32(mem, S3_GPIO_BASE + 0x03Cu) & mask, 0u);
    cpu0.ccount = 480u;
    ASSERT_EQ(mem_read32(mem, S3_GPIO_BASE + 0x03Cu) & mask, 0u);
    mem_write32(mem, S3_GPIO_BASE + 0x024u, mask);
    ASSERT_EQ(mem_read32(mem, S3_GPIO_BASE + 0x03Cu) & mask, mask);
    cpu0.ccount = 576u;
    ASSERT_EQ(mem_read32(mem, S3_GPIO_BASE + 0x03Cu) & mask, 0u);
    ASSERT_EQ(periph_unhandled_count(periph), 0u);

    /* A data-only carrier is sampled from the group-clock oscillator even
     * when no consumer has asked the device to schedule pad edges. */
    cpu0.ccount = 800u;
    mem_write32(mem, S3_GPIO_BASE + S3_GPIO_FUNC_OUT4,
                s3->rmt_v1.output_signal_base);
    mem_write32(mem, S3_RMT_BASE + S3_RMT_CARRIER0,
                (1u << 16u)); /* high two SCLK ticks, low one */
    mem_write32(mem, S3_RMT_BASE + S3_RMT_CONF0,
                conf | (1u << 21u) | 1u);
    ASSERT_EQ(mem_read32(mem, S3_GPIO_BASE + 0x03Cu) & mask, mask);
    cpu0.ccount = 807u;
    ASSERT_EQ(mem_read32(mem, S3_GPIO_BASE + 0x03Cu) & mask, mask);
    cpu0.ccount = 808u;
    ASSERT_EQ(mem_read32(mem, S3_GPIO_BASE + 0x03Cu) & mask, 0u);
    cpu0.ccount = 812u;
    ASSERT_EQ(mem_read32(mem, S3_GPIO_BASE + 0x03Cu) & mask, mask);
    ASSERT_EQ(periph_unhandled_count(periph), 0u);

    /* Always-on carrier includes the idle oscillator, which is still
     * unsupported and must not be mistaken for a valid sampled level. */
    cpu0.ccount = 1200u;
    mem_write32(mem, S3_RMT_BASE + S3_RMT_CONF0,
                (conf & ~(1u << 20u)) | (1u << 21u));
    ASSERT_EQ(periph_unhandled_count(periph), 1u);

    periph_destroy(periph);
    mem_destroy(mem);
}

TEST(s3_rmt_v1_data_only_carrier_reaches_gpio_and_rx_demodulator)
{
    const flexe_target_desc_t *s3 =
        flexe_target_by_id(FLEXE_TARGET_ESP32S3);
    xtensa_mem_t *mem = mem_create_for_target(s3);
    esp32_periph_t *periph = mem ? periph_create(mem) : NULL;
    xtensa_cpu_t cpu0;
    ASSERT_TRUE(periph != NULL);
    if (!periph) {
        mem_destroy(mem);
        return;
    }
    xtensa_cpu_init_for_target(&cpu0, s3);
    cpu0.mem = mem;
    periph_attach_cpus(periph, &cpu0, NULL);
    rmt_v1_probe_t probe = {0};
    ASSERT_EQ(periph_set_rmt_tx_callback(periph, 0,
                                         rmt_v1_tx_observed, &probe), 0);
    mem_write32(mem, s3->io_mux.base +
                     s3->io_mux.gpio_register_offset[4],
                s3->io_mux.input_enable_mask);
    mem_write32(mem, S3_GPIO_BASE + S3_GPIO_RMT_RX0, 0x80u | 4u);
    mem_write32(mem, S3_GPIO_BASE + S3_GPIO_FUNC_OUT4,
                s3->rmt_v1.output_signal_base);
    uint32_t conf = mem_read32(mem, S3_RMT_BASE + S3_RMT_CONF0) |
                    (1u << 6);
    mem_write32(mem, S3_RMT_BASE + S3_RMT_CONF0, conf);
    ASSERT_EQ(mem_read32(mem, S3_GPIO_BASE + 0x03Cu) & (1u << 4),
              0u);
    mem_write32(mem, S3_RMT_BASE + S3_RMT_RX_CONF4,
                (1u << 29) | (1u << 28) | (1u << 24) |
                (40u << 8) | 2u);
    mem_write32(mem, S3_RMT_BASE + S3_RMT_RX_CARRIER4, 1u);
    mem_write32(mem, S3_RMT_BASE + S3_RMT_RX_CTRL4,
                (1u << 15) | (1u << 3) | 1u);
    mem_write32(mem, S3_RMT_BASE + S3_RMT_MEM0,
                10u | (1u << 15) | (20u << 16));
    mem_write32(mem, S3_RMT_BASE + S3_RMT_CARRIER0,
                1u << 16u); /* 8 high + 4 low CPU cycles */
    /* Reset TX configuration already selects data-only/high carrier. */
    mem_write32(mem, S3_RMT_BASE + S3_RMT_CONF0,
                conf | (1u << 5) | 1u);
    ASSERT_EQ(periph_gpio_pin_level(periph, 4), 1);
    ASSERT_EQ(periph_unhandled_count(periph), 1u);
    ASSERT_EQ(mem_read32(mem, S3_GPIO_BASE + 0x03Cu) & (1u << 4),
              1u << 4);
    cpu0.ccount = 7u;
    (void)mem_read32(mem, S3_RMT_BASE + S3_RMT_STATUS0);
    ASSERT_EQ(periph_gpio_pin_level(periph, 4), 1);
    cpu0.ccount = 8u;
    (void)mem_read32(mem, S3_RMT_BASE + S3_RMT_STATUS0);
    ASSERT_EQ(periph_gpio_pin_level(periph, 4), 0);
    cpu0.ccount = 12u;
    (void)mem_read32(mem, S3_RMT_BASE + S3_RMT_STATUS0);
    ASSERT_EQ(periph_gpio_pin_level(periph, 4), 1);
    cpu0.ccount = 239u;
    ASSERT_EQ(mem_read32(mem, S3_RMT_BASE + S3_RMT_RX_MEM4), 0u);
    cpu0.ccount = 240u;
    ASSERT_EQ(mem_read32(mem, S3_RMT_BASE + S3_RMT_RX_MEM4),
              10u | (1u << 15) | (20u << 16));
    ASSERT_EQ(probe.carrier_hz, 13333333u);
    ASSERT_EQ(probe.completions, 1u);
    ASSERT_EQ(periph_gpio_pin_level(periph, 4), 1);
    ASSERT_EQ(periph_unhandled_count(periph), 1u);
    periph_destroy(periph);
    mem_destroy(mem);
}

TEST(s3_rmt_v1_carrier_delayed_eval_replays_full_pad_waveform)
{
    const flexe_target_desc_t *s3 =
        flexe_target_by_id(FLEXE_TARGET_ESP32S3);
    xtensa_mem_t *mem = mem_create_for_target(s3);
    esp32_periph_t *periph = mem ? periph_create(mem) : NULL;
    xtensa_cpu_t cpu0;
    ASSERT_TRUE(periph != NULL);
    if (!periph) {
        mem_destroy(mem);
        return;
    }
    xtensa_cpu_init_for_target(&cpu0, s3);
    cpu0.mem = mem;
    periph_attach_cpus(periph, &cpu0, NULL);
    mem_write32(mem, s3->io_mux.base +
                     s3->io_mux.gpio_register_offset[4],
                s3->io_mux.input_enable_mask);
    mem_write32(mem, S3_GPIO_BASE + S3_GPIO_FUNC_OUT4,
                s3->rmt_v1.output_signal_base);
    mem_write32(mem, S3_GPIO_BASE + 0x074u + 4u * 4u,
                (1u << 13) | (1u << 7)); /* watched positive-edge IRQ */
    mem_write32(mem, S3_RMT_BASE + S3_RMT_CARRIER0,
                1u << 16u); /* 8 high + 4 low CPU cycles */
    mem_write32(mem, S3_RMT_BASE + S3_RMT_MEM0,
                2000u | (1u << 15) | (10u << 16));
    mem_write32(mem, S3_RMT_BASE + S3_RMT_MEM0 + 4u, 0u);
    uint32_t conf = mem_read32(mem, S3_RMT_BASE + S3_RMT_CONF0);
    mem_write32(mem, S3_RMT_BASE + S3_RMT_CONF0,
                conf | (1u << 6) | 1u);
    ASSERT_EQ(periph_gpio_pin_level(periph, 4), 1);

    /* One delayed read must not drop the 2,666 carrier transitions before
     * the 2,000-tick envelope boundary. The safety bound counts segments,
     * not observable pad edges. */
    cpu0.ccount = 16000u;
    ASSERT_EQ((mem_read32(mem, S3_RMT_BASE + S3_RMT_STATUS0) >> 22) & 7u,
              1u);
    ASSERT_EQ(periph_gpio_pin_level(periph, 4), 0);
    cpu0.ccount = 16080u;
    ASSERT_EQ(mem_read32(mem, S3_RMT_BASE + S3_RMT_INT_RAW), 1u);
    ASSERT_EQ(periph_unhandled_count(periph), 0u);
    periph_destroy(periph);
    mem_destroy(mem);
}

TEST(s3_rmt_v1_data_only_carrier_can_modulate_low_symbols)
{
    const flexe_target_desc_t *s3 =
        flexe_target_by_id(FLEXE_TARGET_ESP32S3);
    xtensa_mem_t *mem = mem_create_for_target(s3);
    esp32_periph_t *periph = mem ? periph_create(mem) : NULL;
    xtensa_cpu_t cpu0;
    ASSERT_TRUE(periph != NULL);
    if (!periph) {
        mem_destroy(mem);
        return;
    }
    xtensa_cpu_init_for_target(&cpu0, s3);
    cpu0.mem = mem;
    periph_attach_cpus(periph, &cpu0, NULL);
    mem_write32(mem, s3->io_mux.base +
                     s3->io_mux.gpio_register_offset[4],
                s3->io_mux.input_enable_mask);
    mem_write32(mem, S3_GPIO_BASE + S3_GPIO_FUNC_OUT4,
                s3->rmt_v1.output_signal_base);
    mem_write32(mem, S3_RMT_BASE + S3_RMT_CARRIER0, 1u << 16u);
    mem_write32(mem, S3_RMT_BASE + S3_RMT_MEM0,
                10u | (10u << 16u) | (1u << 31u));
    uint32_t conf = mem_read32(mem, S3_RMT_BASE + S3_RMT_CONF0);
    mem_write32(mem, S3_RMT_BASE + S3_RMT_CONF0,
                (conf & ~(1u << 22u)) | (1u << 6u) | 1u);
    uint32_t mask = 1u << 4u;
    ASSERT_EQ(mem_read32(mem, S3_GPIO_BASE + 0x03Cu) & mask, 0u);
    cpu0.ccount = 8u;
    ASSERT_EQ(mem_read32(mem, S3_GPIO_BASE + 0x03Cu) & mask, mask);
    cpu0.ccount = 12u;
    ASSERT_EQ(mem_read32(mem, S3_GPIO_BASE + 0x03Cu) & mask, 0u);
    cpu0.ccount = 80u;
    ASSERT_EQ(mem_read32(mem, S3_GPIO_BASE + 0x03Cu) & mask, mask);
    cpu0.ccount = 159u;
    ASSERT_EQ(mem_read32(mem, S3_GPIO_BASE + 0x03Cu) & mask, mask);
    ASSERT_EQ(periph_unhandled_count(periph), 0u);
    periph_destroy(periph);
    mem_destroy(mem);
}

TEST(s3_rmt_v1_tx_pad_edges_raise_gpio_interrupt)
{
    const flexe_target_desc_t *s3 =
        flexe_target_by_id(FLEXE_TARGET_ESP32S3);
    xtensa_mem_t *mem = mem_create_for_target(s3);
    esp32_periph_t *periph = mem ? periph_create(mem) : NULL;
    xtensa_cpu_t cpu0;
    ASSERT_TRUE(periph != NULL);
    if (!periph) {
        mem_destroy(mem);
        return;
    }
    xtensa_cpu_init_for_target(&cpu0, s3);
    cpu0.mem = mem;
    periph_attach_cpus(periph, &cpu0, NULL);
    uint32_t mask = 1u << 4u;
    mem_write32(mem, s3->io_mux.base +
                     s3->io_mux.gpio_register_offset[4],
                s3->io_mux.input_enable_mask);
    mem_write32(mem, S3_GPIO_BASE + S3_GPIO_FUNC_OUT4,
                s3->rmt_v1.output_signal_base);
    mem_write32(mem, S3_GPIO_BASE + 0x074u + 4u * 4u,
                (1u << 13) | (1u << 7)); /* positive-edge GPIO IRQ */
    const uint32_t first = 10u | (1u << 15) | (12u << 16);
    const uint32_t second = 8u | (1u << 15) | (9u << 16);
    mem_write32(mem, S3_RMT_BASE + S3_RMT_MEM0, first);
    mem_write32(mem, S3_RMT_BASE + S3_RMT_MEM0 + 4u, second);
    mem_write32(mem, S3_RMT_BASE + S3_RMT_MEM0 + 8u, 0u);
    uint32_t conf = mem_read32(mem, S3_RMT_BASE + S3_RMT_CONF0);
    mem_write32(mem, S3_RMT_BASE + S3_RMT_CONF0,
                (conf & ~(1u << 21)) | (1u << 6) | 1u);
    ASSERT_EQ(mem_read32(mem, S3_GPIO_BASE + 0x044u) & mask, mask);
    ASSERT_TRUE(periph_interrupt_pending(periph,
                                         s3->gpio.interrupt_source));
    mem_write32(mem, S3_GPIO_BASE + 0x04Cu, mask);
    cpu0.ccount = 175u;
    (void)mem_read32(mem, S3_RMT_BASE + S3_RMT_STATUS0);
    ASSERT_EQ(mem_read32(mem, S3_GPIO_BASE + 0x044u) & mask, 0u);
    cpu0.ccount = 176u;
    (void)mem_read32(mem, S3_RMT_BASE + S3_RMT_STATUS0);
    ASSERT_EQ(mem_read32(mem, S3_GPIO_BASE + 0x044u) & mask, mask);
    ASSERT_EQ(periph_unhandled_count(periph), 0u);
    periph_destroy(periph);
    mem_destroy(mem);
}

TEST(s3_rmt_v1_rx_filter_rejects_glitch_and_qualifies_at_group_clock)
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

    /* Default SCLK is 80 MHz / 2 and the channel divides by 2 again.
     * A threshold of three GROUP ticks is 12 CPU cycles, not 24. */
    mem_write32(mem, s3->io_mux.base +
                     s3->io_mux.gpio_register_offset[4],
                s3->io_mux.input_enable_mask);
    mem_write32(mem, S3_GPIO_BASE + S3_GPIO_RMT_RX0, 0x80u | 4u);
    mem_write32(mem, S3_RMT_BASE + S3_RMT_RX_CONF4,
                (1u << 24) | (20u << 8) | 2u);
    mem_write32(mem, S3_RMT_BASE + S3_RMT_RX_CTRL4,
                (1u << 15) | (3u << 5) | (1u << 4) | (1u << 3) | 1u);
    mem_write32(mem, S3_RMT_BASE + S3_RMT_INT_ENA, 1u << 16);

    periph_gpio_set_input(periph, 4, 1);
    cpu0.ccount = 11u;
    periph_gpio_set_input(periph, 4, 0);
    cpu0.ccount = 19u;
    ASSERT_EQ(mem_read32(mem, S3_RMT_BASE + S3_RMT_RX_MEM4), 0u);
    ASSERT_EQ(mem_read32(mem, S3_RMT_BASE + S3_RMT_INT_RAW), 0u);

    cpu0.ccount = 20u;
    periph_gpio_set_input(periph, 4, 1);
    cpu0.ccount = 31u;
    ASSERT_EQ(mem_read32(mem, S3_RMT_BASE + S3_RMT_RX_MEM4), 0u);
    cpu0.ccount = 100u;
    periph_gpio_set_input(periph, 4, 0);
    cpu0.ccount = 140u;
    periph_gpio_set_input(periph, 4, 1); /* glitch inside the low half */
    cpu0.ccount = 151u;
    periph_gpio_set_input(periph, 4, 0);
    cpu0.ccount = 196u;
    periph_gpio_set_input(periph, 4, 1);
    cpu0.ccount = 207u;
    ASSERT_EQ(mem_read32(mem, S3_RMT_BASE + S3_RMT_RX_MEM4), 0u);
    cpu0.ccount = 208u;
    ASSERT_EQ(mem_read32(mem, S3_RMT_BASE + S3_RMT_RX_MEM4),
              10u | (1u << 15) | (12u << 16));
    cpu0.ccount = 260u;
    periph_gpio_set_input(periph, 4, 0);
    cpu0.ccount = 332u;
    periph_gpio_set_input(periph, 4, 1);
    cpu0.ccount = 344u;
    ASSERT_EQ(mem_read32(mem, S3_RMT_BASE + S3_RMT_RX_MEM4 + 4u),
              8u | (1u << 15) | (9u << 16));
    cpu0.ccount = 503u;
    ASSERT_EQ(mem_read32(mem, S3_RMT_BASE + S3_RMT_INT_RAW), 0u);
    cpu0.ccount = 504u;
    ASSERT_EQ(mem_read32(mem, S3_RMT_BASE + S3_RMT_INT_ST), 1u << 16);

    /* Equal-to-threshold pulses survive: eval qualifies the high edge
     * before the raw input falls on cycle 532. */
    mem_write32(mem, S3_RMT_BASE + S3_RMT_INT_CLR, 1u << 16);
    periph_gpio_set_input(periph, 4, 0);
    mem_write32(mem, S3_RMT_BASE + S3_RMT_RX_CTRL4,
                (1u << 15) | (3u << 5) | (1u << 4) |
                (1u << 3) | (1u << 1) | 1u);
    cpu0.ccount = 520u;
    periph_gpio_set_input(periph, 4, 1);
    cpu0.ccount = 532u;
    periph_gpio_set_input(periph, 4, 0);
    cpu0.ccount = 620u;
    periph_gpio_set_input(periph, 4, 1);
    cpu0.ccount = 632u;
    ASSERT_EQ(mem_read32(mem, S3_RMT_BASE + S3_RMT_RX_MEM4),
              1u | (1u << 15) | (11u << 16));
    ASSERT_EQ(periph_unhandled_count(periph), 0u);
    periph_destroy(periph);
    mem_destroy(mem);
}

TEST(s3_rmt_v1_rx_demodulates_both_carrier_polarities)
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
    mem_write32(mem, s3->io_mux.base +
                     s3->io_mux.gpio_register_offset[4],
                s3->io_mux.input_enable_mask);
    mem_write32(mem, S3_GPIO_BASE + S3_GPIO_RMT_RX0, 0x80u | 4u);

    /* Four short high carrier pulses separated by two-tick low gaps form
     * one 14-tick mark. A ten-tick low gap ends it; the low threshold is
     * register value + 1 = five channel ticks. */
    mem_write32(mem, S3_RMT_BASE + S3_RMT_RX_CONF4,
                (1u << 29) | (1u << 28) | (1u << 24) |
                (40u << 8) | 2u);
    mem_write32(mem, S3_RMT_BASE + 0x90u, (4u << 16) | 4u);
    mem_write32(mem, S3_RMT_BASE + S3_RMT_RX_CTRL4,
                (1u << 15) | (1u << 3) | 1u);
    periph_gpio_set_input(periph, 4, 1);
    for (unsigned i = 0u; i < 3u; i++) {
        cpu0.ccount = 16u + i * 32u;
        periph_gpio_set_input(periph, 4, 0);
        cpu0.ccount = 32u + i * 32u;
        periph_gpio_set_input(periph, 4, 1);
    }
    cpu0.ccount = 112u;
    periph_gpio_set_input(periph, 4, 0);
    cpu0.ccount = 151u;
    ASSERT_EQ(mem_read32(mem, S3_RMT_BASE + S3_RMT_RX_MEM4), 0u);
    cpu0.ccount = 192u;
    periph_gpio_set_input(periph, 4, 1);
    ASSERT_EQ(mem_read32(mem, S3_RMT_BASE + S3_RMT_RX_MEM4),
              14u | (1u << 15) | (10u << 16));

    /* The opposite polarity uses the high-gap threshold instead. */
    mem_write32(mem, S3_RMT_BASE + S3_RMT_RX_CTRL4,
                (1u << 15) | (1u << 1));
    mem_write32(mem, S3_RMT_BASE + S3_RMT_RX_CONF4,
                (1u << 28) | (1u << 24) | (40u << 8) | 2u);
    mem_write32(mem, S3_RMT_BASE + S3_RMT_RX_CTRL4,
                (1u << 15) | (1u << 3) | 1u);
    cpu0.ccount = 300u;
    periph_gpio_set_input(periph, 4, 0);
    for (unsigned i = 0u; i < 3u; i++) {
        cpu0.ccount = 316u + i * 32u;
        periph_gpio_set_input(periph, 4, 1);
        cpu0.ccount = 332u + i * 32u;
        periph_gpio_set_input(periph, 4, 0);
    }
    cpu0.ccount = 412u;
    periph_gpio_set_input(periph, 4, 1);
    cpu0.ccount = 492u;
    periph_gpio_set_input(periph, 4, 0);
    ASSERT_EQ(mem_read32(mem, S3_RMT_BASE + S3_RMT_RX_MEM4),
              14u | ((10u | (1u << 15)) << 16));
    ASSERT_EQ(periph_unhandled_count(periph), 2u);
    periph_destroy(periph);
    mem_destroy(mem);
}

TEST(s3_rmt_v1_rx_threshold_capacity_and_ownership_are_visible)
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
    mem_write32(mem, S3_RMT_BASE + S3_RMT_RX_CONF4,
                (1u << 24) | (5u << 8) | 2u);
    mem_write32(mem, S3_RMT_BASE + S3_RMT_RX_LIMIT4, 2u);
    mem_write32(mem, S3_RMT_BASE + S3_RMT_INT_ENA,
                (1u << 20) | (1u << 24));
    mem_write32(mem, S3_RMT_BASE + S3_RMT_RX_CTRL4,
                (1u << 15) | (1u << 3) | 1u);
    uint32_t symbols[49];
    for (unsigned i = 0u; i < 49u; i++) symbols[i] = 1u | (1u << 16);
    ASSERT_EQ(periph_rmt_rx_inject(periph, 4, symbols, 49u), 48u);
    ASSERT_EQ(mem_read32(mem, S3_RMT_BASE + S3_RMT_INT_ST), 0u);
    ASSERT_EQ(mem_read32(mem, S3_RMT_BASE + S3_RMT_RX_STATUS4) &
              (1u << 26), 0u);
    cpu0.ccount = 31u;
    ASSERT_EQ(mem_read32(mem, S3_RMT_BASE + S3_RMT_INT_ST), 0u);
    cpu0.ccount = 32u;
    ASSERT_EQ(mem_read32(mem, S3_RMT_BASE + S3_RMT_INT_ST), 1u << 24);
    mem_write32(mem, S3_RMT_BASE + S3_RMT_INT_CLR, 1u << 24);
    cpu0.ccount = 767u;
    ASSERT_EQ(mem_read32(mem, S3_RMT_BASE + S3_RMT_RX_STATUS4) &
              (1u << 26), 0u);
    cpu0.ccount = 768u;
    ASSERT_EQ(mem_read32(mem, S3_RMT_BASE + S3_RMT_RX_STATUS4) &
              (1u << 26), 1u << 26);
    cpu0.ccount = 807u;
    ASSERT_EQ(mem_read32(mem, S3_RMT_BASE + S3_RMT_INT_ST), 0u);
    cpu0.ccount = 808u;
    ASSERT_EQ(mem_read32(mem, S3_RMT_BASE + S3_RMT_INT_ST), 1u << 20);
    ASSERT_EQ(mem_read32(mem, S3_RMT_BASE + S3_RMT_RX_STATUS4) &
              (1u << 26), 1u << 26);
    mem_write32(mem, S3_RMT_BASE + S3_RMT_INT_CLR, 1u << 20);

    /* Ownership is a hardware gate: software-owned RX RAM rejects input. */
    mem_write32(mem, S3_RMT_BASE + S3_RMT_RX_CTRL4,
                (1u << 15) | (1u << 1) | 1u);
    ASSERT_EQ(periph_rmt_rx_inject(periph, 4, symbols, 1u), 0u);
    ASSERT_EQ(mem_read32(mem, S3_RMT_BASE + S3_RMT_RX_STATUS4) &
              (1u << 25), 1u << 25);
    ASSERT_EQ(mem_read32(mem, S3_RMT_BASE + S3_RMT_INT_ST), 1u << 20);
    ASSERT_EQ(periph_unhandled_count(periph), 0u);
    periph_destroy(periph);
    mem_destroy(mem);
}

TEST(s3_rmt_v1_rx_wrap_refills_both_halves_before_end)
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
    mem_write32(mem, S3_RMT_BASE + S3_RMT_RX_CONF4,
                (1u << 24) | (5u << 8) | 2u);
    mem_write32(mem, S3_RMT_BASE + S3_RMT_RX_LIMIT4, 24u);
    mem_write32(mem, S3_RMT_BASE + S3_RMT_INT_ENA,
                (1u << 16) | (1u << 24));
    mem_write32(mem, S3_RMT_BASE + S3_RMT_RX_CTRL4,
                (1u << 15) | (1u << 13) | (1u << 3) | 1u);

    uint32_t symbols[96];
    for (unsigned i = 0u; i < 96u; i++)
        symbols[i] = (1u + i % 16u) | ((1u + i % 7u) << 16) |
                     ((i & 1u) << 15);
    ASSERT_EQ(periph_rmt_rx_inject(periph, 4, symbols, 96u), 96u);

    uint64_t ticks = 0u;
    for (unsigned half = 0u; half < 4u; half++) {
        for (unsigned i = half * 24u; i < (half + 1u) * 24u; i++)
            ticks += (symbols[i] & 0x7FFFu) +
                     ((symbols[i] >> 16) & 0x7FFFu);
        cpu0.ccount = (uint32_t)(ticks * 8u - 1u);
        ASSERT_EQ(mem_read32(mem, S3_RMT_BASE + S3_RMT_INT_ST), 0u);
        cpu0.ccount++;
        ASSERT_EQ(mem_read32(mem, S3_RMT_BASE + S3_RMT_INT_ST),
                  1u << 24);
        unsigned memory_half = half & 1u;
        for (unsigned i = 0u; i < 24u; i++)
            ASSERT_EQ(mem_read32(mem, S3_RMT_BASE + S3_RMT_RX_MEM4 +
                                 (memory_half * 24u + i) * 4u),
                      symbols[half * 24u + i]);
        ASSERT_EQ(mem_read32(mem, S3_RMT_BASE + S3_RMT_RX_STATUS4) &
                  0x3FFu, 192u + ((half + 1u) * 24u) % 48u);
        mem_write32(mem, S3_RMT_BASE + S3_RMT_INT_CLR, 1u << 24);
    }
    cpu0.ccount = (uint32_t)((ticks + 5u) * 8u - 1u);
    ASSERT_EQ(mem_read32(mem, S3_RMT_BASE + S3_RMT_INT_ST), 0u);
    cpu0.ccount++;
    ASSERT_EQ(mem_read32(mem, S3_RMT_BASE + S3_RMT_INT_ST), 1u << 16);
    ASSERT_EQ(mem_read32(mem, S3_RMT_BASE + S3_RMT_RX_STATUS4) &
              (1u << 26), 0u);
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
    mem_write32(mem, S3_RMT_BASE + S3_RMT_RX_CONF4, 1u << 23);
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
    RUN_TEST(s3_rmt_v1_rx_decoded_symbols_complete_after_idle_and_raise_irq);
    RUN_TEST(s3_rmt_v1_rx_captures_gpio_matrix_edges_in_guest_time);
    RUN_TEST(s3_rmt_v1_rx_captures_gpio_output_loopback_edges);
    RUN_TEST(s3_rmt_v1_rx_captures_rmt_tx_pad_waveform);
    RUN_TEST(s3_rmt_v1_gpio_in_polls_unobserved_tx_without_edge_events);
    RUN_TEST(s3_rmt_v1_data_only_carrier_reaches_gpio_and_rx_demodulator);
    RUN_TEST(s3_rmt_v1_carrier_delayed_eval_replays_full_pad_waveform);
    RUN_TEST(s3_rmt_v1_data_only_carrier_can_modulate_low_symbols);
    RUN_TEST(s3_rmt_v1_tx_pad_edges_raise_gpio_interrupt);
    RUN_TEST(s3_rmt_v1_rx_filter_rejects_glitch_and_qualifies_at_group_clock);
    RUN_TEST(s3_rmt_v1_rx_demodulates_both_carrier_polarities);
    RUN_TEST(s3_rmt_v1_rx_threshold_capacity_and_ownership_are_visible);
    RUN_TEST(s3_rmt_v1_rx_wrap_refills_both_halves_before_end);
    RUN_TEST(s3_rmt_v1_unmodeled_modes_remain_diagnostic);
}
