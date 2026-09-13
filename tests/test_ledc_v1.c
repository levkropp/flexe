/* ESP32-S3 low-speed-only LEDC register, fade, gate, and IRQ tests. */
#include "test_helpers.h"
#include "peripherals.h"

#define S3_LEDC_BASE         0x60019000u
#define S3_LEDC_CH0_CONF0    0x000u
#define S3_LEDC_CH0_DUTY     0x008u
#define S3_LEDC_CH0_CONF1    0x00Cu
#define S3_LEDC_CH0_DUTY_R   0x010u
#define S3_LEDC_TIMER0_CONF  0x0A0u
#define S3_LEDC_TIMER0_VALUE 0x0A4u
#define S3_LEDC_INT_RAW      0x0C0u
#define S3_LEDC_INT_ST       0x0C4u
#define S3_LEDC_INT_ENA      0x0C8u
#define S3_LEDC_INT_CLR      0x0CCu
#define S3_LEDC_CONF         0x0D0u
#define S3_LEDC_DATE         0x0FCu
#define S3_SYSTEM_CLK_EN0    0x600C0018u
#define S3_SYSTEM_RST_EN0    0x600C0020u
#define S3_LEDC_GATE         (1u << 11)

typedef struct {
    unsigned calls;
    int gpio;
    unsigned frequency;
    unsigned duty;
    unsigned duty_max;
    bool enabled;
} s3_ledc_probe_t;

static void s3_ledc_changed(void *ctx, int speed_mode, int channel,
                            int gpio, uint32_t frequency_hz, uint32_t duty,
                            uint32_t duty_max, bool enabled, bool inverted)
{
    s3_ledc_probe_t *probe = ctx;
    if (speed_mode != 1 || channel != 0 || inverted) return;
    probe->calls++;
    probe->gpio = gpio;
    probe->frequency = frequency_hz;
    probe->duty = duty;
    probe->duty_max = duty_max;
    probe->enabled = enabled;
}

TEST(s3_ledc_v1_runs_pwm_fade_and_obeys_system_gate_reset)
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

    ASSERT_TRUE(s3->capabilities & FLEXE_TARGET_CAP_LEDC_V1);
    ASSERT_EQ(s3->ledc_v1.base, S3_LEDC_BASE);
    ASSERT_EQ(mem_read32(mem, S3_LEDC_BASE + S3_LEDC_DATE),
              0x19040200u);
    ASSERT_EQ(mem_read32(mem, S3_LEDC_BASE + S3_LEDC_TIMER0_CONF),
              1u << 23);

    s3_ledc_probe_t probe = {0};
    ASSERT_EQ(periph_set_ledc_output_callback(periph, 0, 0,
                                               s3_ledc_changed, &probe), -1);
    ASSERT_EQ(periph_set_ledc_output_callback(periph, 1, 0,
                                               s3_ledc_changed, &probe), 0);
    ASSERT_FALSE(probe.enabled);
    mem_write32(mem, S3_LEDC_BASE + S3_LEDC_CONF, 3u);
    ASSERT_EQ(mem_read32(mem, S3_LEDC_BASE + S3_LEDC_CONF), 0u);

    uint32_t clocks = mem_read32(mem, S3_SYSTEM_CLK_EN0);
    mem_write32(mem, S3_SYSTEM_CLK_EN0, clocks | S3_LEDC_GATE);
    mem_write32(mem, S3_LEDC_BASE + S3_LEDC_CONF, 3u); /* 40 MHz XTAL */
    mem_write32(mem, S3_LEDC_BASE + S3_LEDC_TIMER0_CONF,
                (0x1F40u << 4) | 8u | (1u << 25));
    ASSERT_EQ(mem_read32(mem, S3_LEDC_BASE + S3_LEDC_TIMER0_CONF),
              (0x1F40u << 4) | 8u);

    /* GPIO4 selects LEDC low-speed signal 73 through the output matrix. */
    mem_write32(mem, s3->gpio.base + 0x554u + 4u * 4u, 73u);
    mem_write32(mem, S3_LEDC_BASE + S3_LEDC_CH0_CONF0, 1u << 2);
    mem_write32(mem, S3_LEDC_BASE + S3_LEDC_INT_ENA, 1u << 4);
    mem_write32(mem, S3_LEDC_BASE + S3_LEDC_CH0_DUTY, 64u << 4);
    mem_write32(mem, S3_LEDC_BASE + S3_LEDC_CH0_CONF1,
                (1u << 31) | (1u << 30) | (1u << 20) | (1u << 10));
    ASSERT_EQ(mem_read32(mem, S3_LEDC_BASE + S3_LEDC_CH0_DUTY_R), 0u);
    cpu.ccount = 100000u;
    ASSERT_EQ(mem_read32(mem, S3_LEDC_BASE + S3_LEDC_CH0_DUTY_R),
              64u << 4);
    ASSERT_TRUE(probe.enabled);
    ASSERT_EQ(probe.gpio, 4);
    ASSERT_EQ(probe.frequency, 5000u);
    ASSERT_EQ(probe.duty, 64u);
    ASSERT_EQ(probe.duty_max, 255u);
    ASSERT_EQ(mem_read32(mem, S3_LEDC_BASE + S3_LEDC_INT_RAW) & (1u << 4),
              1u << 4);
    ASSERT_EQ(mem_read32(mem, S3_LEDC_BASE + S3_LEDC_INT_ST) & (1u << 4),
              1u << 4);
    ASSERT_TRUE(periph_interrupt_pending(periph,
                                          s3->ledc_v1.interrupt_source));
    mem_write32(mem, S3_LEDC_BASE + S3_LEDC_INT_CLR, 1u << 4);
    ASSERT_FALSE(periph_interrupt_pending(periph,
                                           s3->ledc_v1.interrupt_source));

    /* Timer overflow is a separate bit and source condition from fade end. */
    mem_write32(mem, S3_LEDC_BASE + S3_LEDC_INT_CLR, 1u);
    mem_write32(mem, S3_LEDC_BASE + S3_LEDC_INT_ENA, 1u);
    cpu.ccount = 150000u;
    ASSERT_EQ(mem_read32(mem, S3_LEDC_BASE + S3_LEDC_INT_RAW) & 1u, 1u);
    ASSERT_TRUE(periph_interrupt_pending(periph,
                                          s3->ledc_v1.interrupt_source));
    mem_write32(mem, S3_LEDC_BASE + S3_LEDC_INT_CLR, 1u);
    ASSERT_FALSE(periph_interrupt_pending(periph,
                                           s3->ledc_v1.interrupt_source));

    uint32_t value = mem_read32(mem, S3_LEDC_BASE + S3_LEDC_TIMER0_VALUE);
    mem_write32(mem, S3_SYSTEM_CLK_EN0, clocks);
    ASSERT_FALSE(probe.enabled);
    cpu.ccount = 200000u;
    ASSERT_EQ(mem_read32(mem, S3_LEDC_BASE + S3_LEDC_TIMER0_VALUE), value);
    mem_write32(mem, S3_SYSTEM_CLK_EN0, clocks | S3_LEDC_GATE);
    ASSERT_TRUE(probe.enabled);

    mem_write32(mem, S3_SYSTEM_RST_EN0, S3_LEDC_GATE);
    ASSERT_EQ(mem_read32(mem, S3_LEDC_BASE + S3_LEDC_CH0_DUTY_R), 0u);
    ASSERT_EQ(mem_read32(mem, S3_LEDC_BASE + S3_LEDC_TIMER0_CONF),
              1u << 23);
    ASSERT_EQ(mem_read32(mem, S3_LEDC_BASE + S3_LEDC_CONF), 0u);
    ASSERT_FALSE(probe.enabled);
    mem_write32(mem, S3_SYSTEM_RST_EN0, 0u);
    ASSERT_EQ(periph_unhandled_count(periph), 0u);

    periph_destroy(periph);
    mem_destroy(mem);
}

void run_ledc_v1_tests(void)
{
    TEST_SUITE("ESP32-S3 LEDC V1");
    RUN_TEST(s3_ledc_v1_runs_pwm_fade_and_obeys_system_gate_reset);
}
