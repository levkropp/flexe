/* Target-described ESP32-S3 motor-control PWM tests. */
#include "test_helpers.h"

#include "peripherals.h"
#include "target.h"

#include <string.h>

#define S3_SYSTEM_CLK_EN0 0x600C0018u
#define S3_SYSTEM_RST_EN0 0x600C0020u
#define S3_SYSTEM_PWM0    (1u << 17u)
#define S3_SYSTEM_PWM1    (1u << 20u)

typedef struct {
    const flexe_target_desc_t *target;
    xtensa_mem_t *mem;
    esp32_periph_t *periph;
    xtensa_cpu_t cpu;
} mcpwm_fixture_t;

typedef struct {
    unsigned calls;
    int group;
    int operator_index;
    int generator;
    periph_mcpwm_output_info_t info;
} mcpwm_capture_t;

static bool mcpwm_fixture_init(mcpwm_fixture_t *fixture)
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

static void mcpwm_fixture_destroy(mcpwm_fixture_t *fixture)
{
    periph_destroy(fixture->periph);
    mem_destroy(fixture->mem);
}

static void mcpwm_capture_output(
    void *ctx, int group, int operator_index, int generator,
    const periph_mcpwm_output_info_t *info)
{
    mcpwm_capture_t *capture = ctx;
    capture->calls++;
    capture->group = group;
    capture->operator_index = operator_index;
    capture->generator = generator;
    capture->info = *info;
}

static uint32_t mcpwm_timer_config(uint32_t prescale, uint32_t period)
{
    return ((prescale - 1u) & 0xFFu) | ((period - 1u) << 8u);
}

static void mcpwm_route_output(mcpwm_fixture_t *fixture, unsigned gpio,
                                unsigned signal, bool inverted)
{
    mem_write32(fixture->mem,
                fixture->target->gpio.base + 0x554u + gpio * 4u,
                signal | (inverted ? 1u << 9u : 0u));
}

static void mcpwm_route_input(mcpwm_fixture_t *fixture, unsigned signal,
                               unsigned gpio, bool inverted)
{
    uint32_t io_mux_offset =
        fixture->target->io_mux.gpio_register_offset[gpio];
    mem_write32(fixture->mem,
                fixture->target->io_mux.base + io_mux_offset, 1u << 9u);
    mem_write32(fixture->mem,
                fixture->target->gpio.base + 0x154u + signal * 4u,
                (1u << 7u) | (inverted ? 1u << 6u : 0u) | gpio);
}

static void mcpwm_advance(mcpwm_fixture_t *fixture, uint32_t cycles)
{
    fixture->cpu.ccount += cycles;
    fixture->cpu.periph_event(&fixture->cpu);
}

TEST(esp32s3_mcpwm_drives_both_groups_and_obeys_system_gates)
{
    mcpwm_fixture_t fixture;
    bool ready = mcpwm_fixture_init(&fixture);
    ASSERT_TRUE(ready);
    if (!ready) {
        mcpwm_fixture_destroy(&fixture);
        return;
    }

    const flexe_mcpwm_desc_t *desc = &fixture.target->mcpwm;
    uint32_t clocks = mem_read32(fixture.mem, S3_SYSTEM_CLK_EN0);
    ASSERT_FALSE(clocks & S3_SYSTEM_PWM0);
    ASSERT_FALSE(clocks & S3_SYSTEM_PWM1);
    ASSERT_EQ(mem_read32(fixture.mem, desc->base[0] + 0x004u),
              0x0000FF00u);
    ASSERT_EQ(mem_read32(fixture.mem, desc->base[1] + 0x124u),
              desc->version_reset);

    mcpwm_capture_t group0 = {0};
    mcpwm_capture_t group1 = {0};
    ASSERT_EQ(periph_set_mcpwm_output_callback(
                  fixture.periph, 0, 0, 0,
                  mcpwm_capture_output, &group0), 0);
    ASSERT_EQ(periph_set_mcpwm_output_callback(
                  fixture.periph, 1, 2, 1,
                  mcpwm_capture_output, &group1), 0);
    mcpwm_route_output(&fixture, 4u, desc->output_signal[0][0][0], false);
    mcpwm_route_output(&fixture, 5u, desc->output_signal[1][2][1], true);
    mem_write32(fixture.mem,
                fixture.target->io_mux.base +
                    fixture.target->io_mux.gpio_register_offset[4],
                fixture.target->io_mux.input_enable_mask);
    mem_write32(fixture.mem, S3_SYSTEM_CLK_EN0,
                clocks | S3_SYSTEM_PWM0 | S3_SYSTEM_PWM1);

    /* Group 0: 160 MHz / 16 / 10 / 1000 = 1 kHz on operator 0A. */
    uint32_t base0 = desc->base[0];
    mem_write32(fixture.mem, base0 + 0x000u, 15u);
    mem_write32(fixture.mem, base0 + 0x004u,
                mcpwm_timer_config(10u, 1000u));
    mem_write32(fixture.mem, base0 + 0x040u, 250u);
    mem_write32(fixture.mem, base0 + 0x050u,
                (2u << 0u) | (1u << 4u));
    mem_write32(fixture.mem, base0 + 0x008u, 2u | (1u << 3u));

    ASSERT_EQ(group0.group, 0);
    ASSERT_EQ(group0.operator_index, 0);
    ASSERT_EQ(group0.generator, 0);
    ASSERT_EQ(group0.info.gpio, 4);
    ASSERT_EQ(group0.info.frequency_hz, 1000u);
    ASSERT_EQ(group0.info.period_ticks, 1000u);
    ASSERT_EQ(group0.info.compare_ticks, 250u);
    ASSERT_TRUE(group0.info.enabled);
    ASSERT_EQ(periph_gpio_pin_level(fixture.periph, 4), 1);
    ASSERT_EQ(periph_gpio_output_enabled(fixture.periph, 4), 1);
    ASSERT_EQ(mem_read32(fixture.mem, fixture.target->gpio.base + 0x03Cu) &
              (1u << 4u), 1u << 4u);
    mcpwm_advance(&fixture, 39999u);
    ASSERT_EQ(mem_read32(fixture.mem, base0 + 0x010u) & 0xFFFFu, 249u);
    ASSERT_EQ(periph_gpio_pin_level(fixture.periph, 4), 1);
    ASSERT_EQ(mem_read32(fixture.mem, fixture.target->gpio.base + 0x03Cu) &
              (1u << 4u), 1u << 4u);
    mcpwm_advance(&fixture, 1u);
    ASSERT_EQ(mem_read32(fixture.mem, base0 + 0x010u) & 0xFFFFu, 250u);
    ASSERT_EQ(periph_gpio_pin_level(fixture.periph, 4), 0);
    ASSERT_EQ(mem_read32(fixture.mem, fixture.target->gpio.base + 0x03Cu) &
              (1u << 4u), 0u);

    /* Group 1 independently selects timer 2 for operator 2B. Matrix
     * inversion makes its physical pin low during the generator's high
     * phase without changing the callback's logical compare metadata. */
    uint32_t base1 = desc->base[1];
    mem_write32(fixture.mem, base1 + 0x000u, 31u);
    mem_write32(fixture.mem, base1 + 0x038u, 2u << 4u);
    mem_write32(fixture.mem, base1 + 0x024u,
                mcpwm_timer_config(8u, 625u));
    mem_write32(fixture.mem, base1 + 0x0B4u, 312u);
    mem_write32(fixture.mem, base1 + 0x0C4u,
                (2u << 0u) | (1u << 6u));
    mem_write32(fixture.mem, base1 + 0x028u, 2u | (1u << 3u));
    ASSERT_EQ(group1.group, 1);
    ASSERT_EQ(group1.operator_index, 2);
    ASSERT_EQ(group1.generator, 1);
    ASSERT_EQ(group1.info.gpio, 5);
    ASSERT_EQ(group1.info.frequency_hz, 1000u);
    ASSERT_EQ(group1.info.period_ticks, 625u);
    ASSERT_EQ(group1.info.compare_ticks, 312u);
    ASSERT_TRUE(group1.info.enabled);
    ASSERT_TRUE(group1.info.inverted);
    ASSERT_EQ(periph_gpio_pin_level(fixture.periph, 5), 0);

    /* Gating group 0 freezes its counter and tri-states only its outputs. */
    uint32_t count0 = mem_read32(fixture.mem, base0 + 0x010u) & 0xFFFFu;
    mem_write32(fixture.mem, S3_SYSTEM_CLK_EN0,
                clocks | S3_SYSTEM_PWM1);
    ASSERT_FALSE(group0.info.enabled);
    ASSERT_TRUE(group1.info.enabled);
    ASSERT_EQ(periph_gpio_output_enabled(fixture.periph, 4), 0);
    mcpwm_advance(&fixture, 16000u);
    ASSERT_EQ(mem_read32(fixture.mem, base0 + 0x010u) & 0xFFFFu, count0);
    mem_write32(fixture.mem, S3_SYSTEM_CLK_EN0,
                clocks | S3_SYSTEM_PWM0 | S3_SYSTEM_PWM1);
    mcpwm_advance(&fixture, 160u);
    ASSERT_EQ(mem_read32(fixture.mem, base0 + 0x010u) & 0xFFFFu,
              count0 + 1u);

    /* A group-local reset restores group 0 without perturbing group 1. */
    uint32_t group1_timer = mem_read32(fixture.mem, base1 + 0x024u);
    uint32_t resets = mem_read32(fixture.mem, S3_SYSTEM_RST_EN0);
    mem_write32(fixture.mem, S3_SYSTEM_RST_EN0, resets | S3_SYSTEM_PWM0);
    ASSERT_FALSE(group0.info.enabled);
    ASSERT_TRUE(group1.info.enabled);
    ASSERT_EQ(mem_read32(fixture.mem, base0 + 0x004u), 0x0000FF00u);
    ASSERT_EQ(mem_read32(fixture.mem, base0 + 0x114u), 0u);
    ASSERT_EQ(mem_read32(fixture.mem, base1 + 0x024u), group1_timer);
    mem_write32(fixture.mem, S3_SYSTEM_RST_EN0, resets);
    ASSERT_EQ(periph_unhandled_count(fixture.periph), 0u);
    ASSERT_EQ64(mem_unmapped_count(fixture.mem), 0u);

    mcpwm_fixture_destroy(&fixture);
}

TEST(esp32s3_mcpwm_capture_uses_target_matrix_irq_and_clock)
{
    mcpwm_fixture_t fixture;
    bool ready = mcpwm_fixture_init(&fixture);
    ASSERT_TRUE(ready);
    if (!ready) {
        mcpwm_fixture_destroy(&fixture);
        return;
    }

    const flexe_mcpwm_desc_t *desc = &fixture.target->mcpwm;
    uint32_t base = desc->base[0];
    uint32_t clocks = mem_read32(fixture.mem, S3_SYSTEM_CLK_EN0);
    mem_write32(fixture.mem, S3_SYSTEM_CLK_EN0, clocks | S3_SYSTEM_PWM0);
    periph_intr_matrix_set(fixture.periph, 0, 8,
                           desc->interrupt_source[0]);

    periph_gpio_set_input(fixture.periph, 6, 0);
    mcpwm_route_input(&fixture, desc->capture_input_signal[0][0],
                       6u, false);
    mem_write32(fixture.mem, base + 0x0F0u,
                1u | (1u << 1u) | (1u << 2u));
    mem_write32(fixture.mem, base + 0x0E8u, 1u);
    mem_write32(fixture.mem, base + 0x110u, 1u << 27u);

    /* The independent capture counter is APB-clocked at 80 MHz, so 320
     * cycles of a 160-MHz LX7 produce timestamp 160. */
    mcpwm_advance(&fixture, 320u);
    periph_gpio_set_input(fixture.periph, 6, 1);
    ASSERT_EQ(mem_read32(fixture.mem, base + 0x0FCu), 160u);
    ASSERT_EQ(mem_read32(fixture.mem, base + 0x108u) & 1u, 0u);
    ASSERT_EQ(mem_read32(fixture.mem, base + 0x118u), 1u << 27u);
    ASSERT_TRUE(periph_interrupt_pending(fixture.periph,
                                         desc->interrupt_source[0]));
    ASSERT_EQ(fixture.cpu.interrupt & (1u << 8u), 1u << 8u);
    mem_write32(fixture.mem, base + 0x11Cu, 1u << 27u);

    mcpwm_advance(&fixture, 160u);
    periph_gpio_set_input(fixture.periph, 6, 0);
    ASSERT_EQ(mem_read32(fixture.mem, base + 0x0FCu), 240u);
    ASSERT_EQ(mem_read32(fixture.mem, base + 0x108u) & 1u, 1u);
    mem_write32(fixture.mem, base + 0x11Cu, 1u << 27u);

    /* While the group clock is gated, neither time nor matrix edges can
     * mutate capture state or assert the target-described interrupt. */
    mem_write32(fixture.mem, S3_SYSTEM_CLK_EN0, clocks);
    mcpwm_advance(&fixture, 320u);
    periph_gpio_set_input(fixture.periph, 6, 1);
    ASSERT_EQ(mem_read32(fixture.mem, base + 0x0FCu), 240u);
    ASSERT_FALSE(periph_interrupt_pending(fixture.periph,
                                          desc->interrupt_source[0]));
    mem_write32(fixture.mem, S3_SYSTEM_CLK_EN0, clocks | S3_SYSTEM_PWM0);
    mcpwm_advance(&fixture, 160u);
    periph_gpio_set_input(fixture.periph, 6, 0);
    ASSERT_EQ(mem_read32(fixture.mem, base + 0x0FCu), 320u);
    ASSERT_TRUE(periph_interrupt_pending(fixture.periph,
                                         desc->interrupt_source[0]));
    ASSERT_EQ(periph_unhandled_count(fixture.periph), 0u);
    ASSERT_EQ64(mem_unmapped_count(fixture.mem), 0u);

    mcpwm_fixture_destroy(&fixture);
}

void run_mcpwm_target_tests(void)
{
    TEST_SUITE("Target-described MCPWM");
    RUN_TEST(esp32s3_mcpwm_drives_both_groups_and_obeys_system_gates);
    RUN_TEST(esp32s3_mcpwm_capture_uses_target_matrix_irq_and_clock);
}
