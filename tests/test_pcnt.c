/* Target-described ESP32-S3 pulse-counter tests. */
#include "test_helpers.h"

#include "peripherals.h"
#include "target.h"

#include <string.h>

#define S3_SYSTEM_CLK_EN0 0x600C0018u
#define S3_SYSTEM_RST_EN0 0x600C0020u
#define S3_SYSTEM_PCNT    (1u << 10)
#define TEST_PCNT_FILTER_ENABLE (1u << 10)
#define TEST_PCNT_EVENT_THRESHOLD0 (1u << 3)
#define TEST_PCNT_EVENT_HIGH_LIMIT (1u << 5)

typedef struct {
    const flexe_target_desc_t *target;
    xtensa_mem_t *mem;
    esp32_periph_t *periph;
    xtensa_cpu_t cpu;
} pcnt_fixture_t;

static bool pcnt_fixture_init(pcnt_fixture_t *fixture)
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

static void pcnt_fixture_destroy(pcnt_fixture_t *fixture)
{
    periph_destroy(fixture->periph);
    mem_destroy(fixture->mem);
}

static void pcnt_route_input(pcnt_fixture_t *fixture, unsigned signal,
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

static void pcnt_set_input(pcnt_fixture_t *fixture, unsigned gpio,
                           bool level)
{
    periph_gpio_set_input(fixture->periph, (int)gpio, level ? 1 : 0);
}

static void pcnt_pulse(pcnt_fixture_t *fixture, unsigned gpio)
{
    pcnt_set_input(fixture, gpio, true);
    pcnt_set_input(fixture, gpio, false);
}

TEST(esp32s3_pcnt_uses_target_geometry_matrix_limits_irq_and_system_gate)
{
    pcnt_fixture_t fixture;
    bool ready = pcnt_fixture_init(&fixture);
    ASSERT_TRUE(ready);
    if (!ready) {
        pcnt_fixture_destroy(&fixture);
        return;
    }

    const flexe_pcnt_desc_t *desc = &fixture.target->pcnt;
    uint32_t base = desc->base;
    ASSERT_TRUE(fixture.target->capabilities & FLEXE_TARGET_CAP_PCNT_V1);
    ASSERT_EQ(base, 0x60017000u);
    ASSERT_EQ(desc->unit_count, 4u);
    ASSERT_EQ(desc->interrupt_source, 41u);
    ASSERT_EQ(mem_read32(fixture.mem, base), 0x00003C10u);
    ASSERT_EQ(mem_read32(fixture.mem, base + desc->control_offset), 0x55u);
    ASSERT_EQ(mem_read32(fixture.mem, base + desc->date_offset),
              0x19072601u);

    pcnt_set_input(&fixture, 4u, false);
    pcnt_set_input(&fixture, 5u, false);
    pcnt_route_input(&fixture, desc->pulse_input_signal[0][0], 4u, false);
    pcnt_route_input(&fixture, desc->control_input_signal[0][0], 5u, false);
    periph_intr_matrix_set(fixture.periph, 0, 8,
                           desc->interrupt_source);

    uint32_t clocks = mem_read32(fixture.mem, S3_SYSTEM_CLK_EN0);
    mem_write32(fixture.mem, S3_SYSTEM_CLK_EN0,
                clocks | S3_SYSTEM_PCNT);
    uint32_t conf0 = (1u << 18u) | (1u << 20u) |
                     (1u << 15u) | (1u << 14u) | (1u << 13u) |
                     (1u << 12u) | (1u << 11u);
    mem_write32(fixture.mem, base, conf0);
    mem_write32(fixture.mem, base + 0x04u,
                ((uint32_t)(uint16_t)-1 << 16u) | 2u);
    mem_write32(fixture.mem, base + 0x08u,
                ((uint32_t)(uint16_t)-3 << 16u) | 3u);
    mem_write32(fixture.mem, base + desc->control_offset,
                (desc->control_reset & ~1u) | (1u << 16u));
    mem_write32(fixture.mem, base + desc->interrupt_enable_offset, 1u);

    pcnt_pulse(&fixture, 4u);
    pcnt_pulse(&fixture, 4u);
    ASSERT_EQ(mem_read32(fixture.mem, base + desc->count_offset), 2u);
    ASSERT_EQ(mem_read32(fixture.mem, base + desc->interrupt_raw_offset),
              1u);
    ASSERT_EQ(mem_read32(fixture.mem, base + desc->unit_status_offset) &
              TEST_PCNT_EVENT_THRESHOLD0, TEST_PCNT_EVENT_THRESHOLD0);
    ASSERT_TRUE(periph_interrupt_pending(fixture.periph,
                                         desc->interrupt_source));
    ASSERT_EQ(fixture.cpu.interrupt & (1u << 8u), 1u << 8u);
    mem_write32(fixture.mem, base + desc->interrupt_clear_offset, 1u);
    ASSERT_FALSE(periph_interrupt_pending(fixture.periph,
                                          desc->interrupt_source));

    pcnt_pulse(&fixture, 4u);
    ASSERT_EQ(mem_read32(fixture.mem, base + desc->count_offset), 0u);
    ASSERT_EQ(mem_read32(fixture.mem, base + desc->unit_status_offset) &
              TEST_PCNT_EVENT_HIGH_LIMIT, TEST_PCNT_EVENT_HIGH_LIMIT);

    pcnt_set_input(&fixture, 5u, true);
    pcnt_pulse(&fixture, 4u);
    ASSERT_EQ(mem_read32(fixture.mem, base + desc->count_offset), 0xFFFFu);
    mem_write32(fixture.mem, base + desc->interrupt_clear_offset, 1u);

    /* A gated block retains state but cannot count or drive its IRQ line. */
    mem_write32(fixture.mem, S3_SYSTEM_CLK_EN0, clocks);
    pcnt_pulse(&fixture, 4u);
    ASSERT_EQ(mem_read32(fixture.mem, base + desc->count_offset), 0xFFFFu);
    ASSERT_FALSE(periph_interrupt_pending(fixture.periph,
                                          desc->interrupt_source));
    mem_write32(fixture.mem, S3_SYSTEM_CLK_EN0,
                clocks | S3_SYSTEM_PCNT);
    pcnt_pulse(&fixture, 4u);
    ASSERT_EQ(mem_read32(fixture.mem, base + desc->count_offset), 0xFFFEu);

    mem_write32(fixture.mem, S3_SYSTEM_RST_EN0, S3_SYSTEM_PCNT);
    ASSERT_EQ(mem_read32(fixture.mem, base), 0x00003C10u);
    ASSERT_EQ(mem_read32(fixture.mem, base + desc->count_offset), 0u);
    ASSERT_EQ(mem_read32(fixture.mem, base + desc->control_offset), 0x55u);
    ASSERT_EQ(mem_read32(fixture.mem, base + desc->interrupt_raw_offset),
              0u);
    mem_write32(fixture.mem, S3_SYSTEM_RST_EN0, 0u);
    ASSERT_EQ(periph_unhandled_count(fixture.periph), 0u);

    pcnt_fixture_destroy(&fixture);
}

TEST(esp32s3_pcnt_filter_uses_apb_cycles_and_matrix_inversion)
{
    pcnt_fixture_t fixture;
    bool ready = pcnt_fixture_init(&fixture);
    ASSERT_TRUE(ready);
    if (!ready) {
        pcnt_fixture_destroy(&fixture);
        return;
    }

    const flexe_pcnt_desc_t *desc = &fixture.target->pcnt;
    uint32_t base = desc->base + desc->conf_stride;
    uint32_t clocks = mem_read32(fixture.mem, S3_SYSTEM_CLK_EN0);
    mem_write32(fixture.mem, S3_SYSTEM_CLK_EN0,
                clocks | S3_SYSTEM_PCNT);
    pcnt_set_input(&fixture, 6u, false);
    pcnt_route_input(&fixture, desc->pulse_input_signal[1][0], 6u, false);
    mem_write32(fixture.mem, base, (1u << 18u) |
                (1u << 14u) | TEST_PCNT_FILTER_ENABLE | 10u);
    mem_write32(fixture.mem, base + 0x04u, 1u);
    mem_write32(fixture.mem, base + 0x08u,
                ((uint32_t)(uint16_t)-100 << 16u) | 100u);
    mem_write32(fixture.mem, desc->base + desc->control_offset,
                desc->control_reset & ~(1u << 2u));
    mem_write32(fixture.mem,
                desc->base + desc->interrupt_enable_offset, 1u << 1u);

    /* Ten 80-MHz APB samples require twenty LX7 cycles at the descriptor's
     * direct-handoff 160-MHz CPU clock. */
    pcnt_set_input(&fixture, 6u, true);
    fixture.cpu.ccount += 19u;
    fixture.cpu.periph_event(&fixture.cpu);
    ASSERT_EQ(mem_read32(fixture.mem,
                         desc->base + desc->count_offset + 4u), 0u);
    pcnt_set_input(&fixture, 6u, false); /* reject the short high glitch */
    fixture.cpu.ccount += 20u;
    fixture.cpu.periph_event(&fixture.cpu);
    ASSERT_EQ(mem_read32(fixture.mem,
                         desc->base + desc->count_offset + 4u), 0u);

    pcnt_set_input(&fixture, 6u, true);
    fixture.cpu.ccount += 20u;
    fixture.cpu.periph_event(&fixture.cpu);
    /* The shared deadline scheduler must service the filter without an MMIO
     * read opportunistically evaluating it. */
    ASSERT_TRUE(periph_interrupt_pending(
        fixture.periph, desc->interrupt_source));
    ASSERT_EQ(mem_read32(fixture.mem,
                         desc->base + desc->count_offset + 4u), 1u);
    pcnt_set_input(&fixture, 6u, false);
    fixture.cpu.ccount += 20u;
    fixture.cpu.periph_event(&fixture.cpu);

    pcnt_route_input(&fixture, desc->pulse_input_signal[1][0], 6u, true);
    ASSERT_EQ(mem_read32(fixture.mem,
                         desc->base + desc->count_offset + 4u), 1u);
    mem_write32(fixture.mem, base, (1u << 18u));
    pcnt_set_input(&fixture, 6u, true);
    pcnt_set_input(&fixture, 6u, false);
    ASSERT_EQ(mem_read32(fixture.mem,
                         desc->base + desc->count_offset + 4u), 2u);
    ASSERT_EQ(periph_unhandled_count(fixture.periph), 0u);

    pcnt_fixture_destroy(&fixture);
}

void run_pcnt_target_tests(void)
{
    TEST_SUITE("Target-described PCNT");
    RUN_TEST(esp32s3_pcnt_uses_target_geometry_matrix_limits_irq_and_system_gate);
    RUN_TEST(esp32s3_pcnt_filter_uses_apb_cycles_and_matrix_inversion);
}
