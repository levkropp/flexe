/* Target-described TWAI/CAN controller tests. */
#include "test_helpers.h"

#include "peripherals.h"
#include "target.h"

#include <limits.h>
#include <string.h>

#define S3_SYSTEM_CLK_EN0       0x600C0018u
#define S3_SYSTEM_RST_EN0       0x600C0020u
#define S3_SYSTEM_TWAI          (1u << 19)

#define TWAI_MODE_OFF           0x000u
#define TWAI_COMMAND_OFF        0x004u
#define TWAI_STATUS_OFF         0x008u
#define TWAI_INTERRUPT_OFF      0x00Cu
#define TWAI_INTERRUPT_ENA_OFF  0x010u
#define TWAI_BUS_TIMING_0_OFF   0x018u
#define TWAI_BUS_TIMING_1_OFF   0x01Cu
#define TWAI_BUFFER_OFF         0x040u
#define TWAI_CLOCK_DIVIDER_OFF  0x07Cu

#define TWAI_MODE_RESET         (1u << 0)
#define TWAI_MODE_SINGLE_FILTER (1u << 3)
#define TWAI_COMMAND_RELEASE_RX (1u << 2)
#define TWAI_COMMAND_SELF_RX    (1u << 4)
#define TWAI_STATUS_RX_BUFFER   (1u << 0)
#define TWAI_STATUS_TX_BUFFER   (1u << 2)
#define TWAI_STATUS_TX_COMPLETE (1u << 3)
#define TWAI_INT_RX             (1u << 0)
#define TWAI_INT_TX             (1u << 1)

typedef struct {
    xtensa_mem_t *mem;
    esp32_periph_t *periph;
    xtensa_cpu_t cpu;
    const flexe_target_desc_t *target;
} twai_fixture_t;

typedef struct {
    periph_twai_frame_t frame;
    unsigned calls;
} twai_capture_t;

static bool twai_fixture_init(twai_fixture_t *fixture)
{
    memset(fixture, 0, sizeof(*fixture));
    fixture->target = flexe_target_by_id(FLEXE_TARGET_ESP32S3);
    fixture->mem = mem_create_for_target(fixture->target);
    if (!fixture->mem) return false;
    fixture->periph = periph_create(fixture->mem);
    if (!fixture->periph) return false;
    xtensa_cpu_init_for_target(&fixture->cpu, fixture->target);
    fixture->cpu.mem = fixture->mem;
    periph_attach_cpus(fixture->periph, &fixture->cpu, NULL);
    return true;
}

static void twai_fixture_destroy(twai_fixture_t *fixture)
{
    periph_destroy(fixture->periph);
    mem_destroy(fixture->mem);
}

static periph_twai_tx_result_t twai_capture(void *ctx,
                                            const periph_twai_frame_t *frame)
{
    twai_capture_t *capture = ctx;
    capture->frame = *frame;
    capture->calls++;
    return PERIPH_TWAI_TX_ACK;
}

static void twai_write_standard_frame(xtensa_mem_t *mem, uint32_t base,
                                      uint32_t identifier, uint8_t data)
{
    mem_write32(mem, base + TWAI_BUFFER_OFF, 1u);
    mem_write32(mem, base + TWAI_BUFFER_OFF + 4u, identifier >> 3u);
    mem_write32(mem, base + TWAI_BUFFER_OFF + 8u, identifier << 5u);
    mem_write32(mem, base + TWAI_BUFFER_OFF + 12u, data);
}

TEST(esp32s3_twai_uses_wide_timing_system_gate_irq_and_matrix_wiring)
{
    twai_fixture_t fixture;
    bool ready = twai_fixture_init(&fixture);
    ASSERT_TRUE(ready);
    if (!ready) {
        twai_fixture_destroy(&fixture);
        return;
    }

    const flexe_twai_desc_t *desc = &fixture.target->twai;
    uint32_t base = desc->base;
    ASSERT_EQ(base, 0x6002B000u);
    ASSERT_EQ(desc->interrupt_source, 37u);
    ASSERT_EQ(mem_read32(fixture.mem, base + TWAI_MODE_OFF),
              TWAI_MODE_RESET);
    ASSERT_EQ(mem_read32(fixture.mem, base + TWAI_STATUS_OFF),
              TWAI_STATUS_TX_BUFFER | TWAI_STATUS_TX_COMPLETE);

    periph_twai_frame_t injected = {
        .identifier = 0x456u,
        .data_length_code = 1u,
        .data = {0xA5u},
    };
    ASSERT_EQ(periph_twai_rx_inject(fixture.periph, &injected), 0u);

    uint32_t clocks = mem_read32(fixture.mem, S3_SYSTEM_CLK_EN0);
    mem_write32(fixture.mem, S3_SYSTEM_CLK_EN0, clocks | S3_SYSTEM_TWAI);
    mem_write32(fixture.mem, base + TWAI_MODE_OFF,
                TWAI_MODE_RESET | TWAI_MODE_SINGLE_FILTER);
    mem_write32(fixture.mem, base + TWAI_BUS_TIMING_0_OFF, 0x0000C123u);
    mem_write32(fixture.mem, base + TWAI_BUS_TIMING_1_OFF, 0x3Eu);
    mem_write32(fixture.mem, base + TWAI_CLOCK_DIVIDER_OFF, 0x1A5u);
    mem_write32(fixture.mem, base + TWAI_INTERRUPT_ENA_OFF, UINT32_MAX);
    ASSERT_EQ(mem_read32(fixture.mem, base + TWAI_BUS_TIMING_0_OFF),
              0x0000C123u);
    ASSERT_EQ(mem_read32(fixture.mem, base + TWAI_CLOCK_DIVIDER_OFF),
              0x1A5u);
    ASSERT_EQ(mem_read32(fixture.mem, base + TWAI_INTERRUPT_ENA_OFF),
              0xEFu);
    for (unsigned index = 0u; index < 4u; index++)
        mem_write32(fixture.mem, base + TWAI_BUFFER_OFF +
                    (4u + index) * 4u, 0xFFu);

    /* These are ordinary target-described producers, so routing them must
     * not create unsupported-MMIO diagnostics. RX remains a matrix input and
     * is consumed at the functional CAN-frame boundary by host injection. */
    mem_write32(fixture.mem,
                fixture.target->gpio.base + 0x554u + 5u * 4u,
                desc->tx_output_signal);
    mem_write32(fixture.mem,
                fixture.target->gpio.base + 0x554u + 6u * 4u,
                desc->bus_off_output_signal);
    mem_write32(fixture.mem,
                fixture.target->gpio.base + 0x554u + 7u * 4u,
                desc->clock_output_signal);
    mem_write32(fixture.mem,
                fixture.target->gpio.base + 0x154u +
                desc->rx_input_signal * 4u,
                (1u << 7u) | 4u);
    ASSERT_EQ(periph_gpio_out_signal(fixture.periph, 5),
              desc->tx_output_signal);
    ASSERT_EQ(periph_gpio_out_signal(fixture.periph, 6),
              desc->bus_off_output_signal);
    ASSERT_EQ(periph_gpio_out_signal(fixture.periph, 7),
              desc->clock_output_signal);

    mem_write32(fixture.mem, base + TWAI_MODE_OFF,
                TWAI_MODE_SINGLE_FILTER);
    ASSERT_EQ(periph_twai_rx_inject(fixture.periph, &injected), 1u);
    ASSERT_EQ(periph_twai_rx_pending(fixture.periph), 1u);
    ASSERT_TRUE(periph_interrupt_pending(fixture.periph,
                                         desc->interrupt_source));
    ASSERT_EQ(mem_read32(fixture.mem, base + TWAI_INTERRUPT_OFF),
              TWAI_INT_RX);
    mem_write32(fixture.mem, base + TWAI_COMMAND_OFF,
                TWAI_COMMAND_RELEASE_RX);
    ASSERT_FALSE(periph_interrupt_pending(fixture.periph,
                                          desc->interrupt_source));

    twai_capture_t capture = {0};
    ASSERT_EQ(periph_set_twai_tx_callback(
                  fixture.periph, twai_capture, &capture), 0u);
    twai_write_standard_frame(fixture.mem, base, 0x321u, 0x5Au);
    mem_write32(fixture.mem, base + TWAI_COMMAND_OFF,
                TWAI_COMMAND_SELF_RX);
    uint32_t deadline = fixture.cpu.periph_next_event(&fixture.cpu);
    ASSERT_TRUE(deadline != UINT32_MAX);
    /* BRP[12:0] must affect scheduling; truncating this S3 register to the
     * classic six-bit field would complete this frame far earlier. */
    ASSERT_TRUE(deadline > 1000000u);
    fixture.cpu.ccount = deadline;
    fixture.cpu.periph_event(&fixture.cpu);
    ASSERT_EQ(capture.calls, 1u);
    ASSERT_EQ(capture.frame.identifier, 0x321u);
    ASSERT_EQ(capture.frame.data[0], 0x5Au);
    ASSERT_EQ(periph_twai_rx_pending(fixture.periph), 1u);
    ASSERT_EQ(mem_read32(fixture.mem, base + TWAI_INTERRUPT_OFF),
              TWAI_INT_RX | TWAI_INT_TX);

    twai_write_standard_frame(fixture.mem, base, 0x123u, 0x77u);
    mem_write32(fixture.mem, base + TWAI_COMMAND_OFF,
                TWAI_COMMAND_SELF_RX);
    ASSERT_TRUE(fixture.cpu.periph_next_event(&fixture.cpu) != UINT32_MAX);
    uint32_t resets = mem_read32(fixture.mem, S3_SYSTEM_RST_EN0);
    mem_write32(fixture.mem, S3_SYSTEM_RST_EN0, resets | S3_SYSTEM_TWAI);
    ASSERT_EQ(mem_read32(fixture.mem, base + TWAI_MODE_OFF),
              TWAI_MODE_RESET);
    ASSERT_EQ(mem_read32(fixture.mem, base + TWAI_BUS_TIMING_0_OFF), 0u);
    ASSERT_FALSE(periph_interrupt_pending(fixture.periph,
                                          desc->interrupt_source));
    mem_write32(fixture.mem, S3_SYSTEM_RST_EN0, resets);
    mem_write32(fixture.mem, S3_SYSTEM_CLK_EN0, clocks);
    ASSERT_EQ(periph_twai_rx_inject(fixture.periph, &injected), 0u);
    ASSERT_EQ(periph_unhandled_count(fixture.periph), 0u);

    twai_fixture_destroy(&fixture);
}

void run_twai_target_tests(void)
{
    TEST_SUITE("Target-described TWAI/CAN");
    RUN_TEST(
        esp32s3_twai_uses_wide_timing_system_gate_irq_and_matrix_wiring);
}
