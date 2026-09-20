#include "test_helpers.h"

#include "peripherals.h"
#include "target.h"

#include <limits.h>
#include <string.h>

#define S3_SYSTEM_CLK_EN0       0x600C0018u
#define S3_SYSTEM_RST_EN0       0x600C0020u
#define S3_SYSTEM_I2S0          (1u << 4)
#define S3_SYSTEM_I2S1          (1u << 21)

#define S3_I2S0_BASE            0x6000F000u
#define S3_I2S1_BASE            0x6002D000u
#define S3_I2S_INT_RAW_OFF      0x00Cu
#define S3_I2S_INT_ST_OFF       0x010u
#define S3_I2S_INT_ENA_OFF      0x014u
#define S3_I2S_INT_CLR_OFF      0x018u
#define S3_I2S_RX_CONF_OFF      0x020u
#define S3_I2S_TX_CONF_OFF      0x024u
#define S3_I2S_RX_CONF1_OFF     0x028u
#define S3_I2S_TX_CONF1_OFF     0x02Cu
#define S3_I2S_RX_CLKM_CONF_OFF 0x030u
#define S3_I2S_TX_CLKM_CONF_OFF 0x034u
#define S3_I2S_RX_TDM_CTRL_OFF  0x050u
#define S3_I2S_TX_TDM_CTRL_OFF  0x054u
#define S3_I2S_DATE_OFF         0x080u
#define S3_I2S0_REG(off)        (S3_I2S0_BASE + (off))

#define S3_I2S_START            (1u << 2)
#define S3_I2S_UPDATE           (1u << 8)
#define S3_I2S_CLK_ACTIVE       (1u << 26)
#define S3_I2S_CLK_SEL_160M     (2u << 27)
#define S3_I2S_CORE_CLK_EN      (1u << 29)

#define S3_GDMA_BASE            0x6003F000u
#define S3_GDMA_IN_INT_RAW      (S3_GDMA_BASE + 0x008u)
#define S3_GDMA_IN_LINK         (S3_GDMA_BASE + 0x020u)
#define S3_GDMA_IN_PERI_SEL     (S3_GDMA_BASE + 0x048u)
#define S3_GDMA_OUT_INT_RAW     (S3_GDMA_BASE + 0x068u)
#define S3_GDMA_OUT_LINK        (S3_GDMA_BASE + 0x080u)
#define S3_GDMA_OUT_EOF_DESC    (S3_GDMA_BASE + 0x088u)
#define S3_GDMA_OUT_DESC        (S3_GDMA_BASE + 0x090u)
#define S3_GDMA_OUT_PERI_SEL    (S3_GDMA_BASE + 0x0A8u)
#define S3_GDMA_IN_LINK_START   (1u << 22)
#define S3_GDMA_OUT_LINK_START  (1u << 21)
#define S3_GDMA_DESC_EOF        (1u << 30)
#define S3_GDMA_DESC_OWNER      (1u << 31)
#define S3_GDMA_I2S0_TRIGGER    3u
#define S3_GDMA_I2S1_TRIGGER    4u

typedef struct {
    xtensa_mem_t *mem;
    esp32_periph_t *periph;
    xtensa_cpu_t cpu;
} i2s_v2_fixture_t;

typedef struct {
    uint8_t data[64];
    size_t length;
    unsigned calls;
    uint32_t sample_rate;
    uint8_t bits_per_sample;
    uint8_t channels;
} i2s_v2_capture_t;

static bool i2s_v2_fixture_init(i2s_v2_fixture_t *fixture)
{
    memset(fixture, 0, sizeof(*fixture));
    const flexe_target_desc_t *target =
        flexe_target_by_id(FLEXE_TARGET_ESP32S3);
    fixture->mem = mem_create_for_target(target);
    if (!fixture->mem) return false;
    fixture->periph = periph_create(fixture->mem);
    if (!fixture->periph) return false;
    xtensa_cpu_init_for_target(&fixture->cpu, target);
    fixture->cpu.mem = fixture->mem;
    periph_attach_cpus(fixture->periph, &fixture->cpu, NULL);
    return true;
}

static void i2s_v2_fixture_destroy(i2s_v2_fixture_t *fixture)
{
    periph_destroy(fixture->periph);
    mem_destroy(fixture->mem);
}

static void i2s_v2_descriptor(xtensa_mem_t *mem, uint32_t descriptor,
                              uint32_t buffer, size_t size, size_t length,
                              bool eof, uint32_t next)
{
    uint32_t control = (uint32_t)size | ((uint32_t)length << 12u) |
                       S3_GDMA_DESC_OWNER;
    if (eof) control |= S3_GDMA_DESC_EOF;
    mem_write32(mem, descriptor, control);
    mem_write32(mem, descriptor + 4u, buffer);
    mem_write32(mem, descriptor + 8u, next);
}

static void i2s_v2_capture(void *ctx, int port, const uint8_t *data,
                           size_t length, uint32_t sample_rate,
                           uint8_t bits_per_sample, uint8_t channels)
{
    i2s_v2_capture_t *capture = ctx;
    if (port != 0 || capture->length + length > sizeof(capture->data))
        return;
    memcpy(capture->data + capture->length, data, length);
    capture->length += length;
    capture->calls++;
    capture->sample_rate = sample_rate;
    capture->bits_per_sample = bits_per_sample;
    capture->channels = channels;
}

static uint32_t i2s_v2_service_next(i2s_v2_fixture_t *fixture)
{
    uint32_t deadline = fixture->cpu.periph_next_event ?
        fixture->cpu.periph_next_event(&fixture->cpu) : UINT32_MAX;
    if (deadline != UINT32_MAX) {
        fixture->cpu.ccount = deadline;
        fixture->cpu.periph_event(&fixture->cpu);
    }
    return deadline;
}

static void i2s_v2_configure_standard_clock(xtensa_mem_t *mem,
                                            uint32_t base, bool receive)
{
    uint32_t conf1 = (15u << 24u) | (15u << 13u) | (9u << 7u);
    uint32_t clkm = S3_I2S_CORE_CLK_EN | S3_I2S_CLK_SEL_160M |
                    S3_I2S_CLK_ACTIVE | 10u;
    mem_write32(mem, base + (receive ? S3_I2S_RX_CONF1_OFF :
                                      S3_I2S_TX_CONF1_OFF),
                conf1);
    mem_write32(mem, base + (receive ? S3_I2S_RX_TDM_CTRL_OFF :
                                      S3_I2S_TX_TDM_CTRL_OFF),
                (1u << 16u) | 3u);
    mem_write32(mem, base + (receive ? S3_I2S_RX_CLKM_CONF_OFF :
                                      S3_I2S_TX_CLKM_CONF_OFF),
                clkm);
}

TEST(esp32s3_i2s_v2_streams_circular_tx_descriptors_at_audio_rate)
{
    i2s_v2_fixture_t fixture;
    bool ready = i2s_v2_fixture_init(&fixture);
    ASSERT_TRUE(ready);
    if (!ready) {
        i2s_v2_fixture_destroy(&fixture);
        return;
    }

    i2s_v2_capture_t capture = {0};
    ASSERT_EQ(periph_set_i2s_tx_callback(
                  fixture.periph, 0, i2s_v2_capture, &capture), 0);
    ASSERT_EQ(mem_read32(fixture.mem, S3_I2S0_REG(S3_I2S_DATE_OFF)),
              0x02009070u);
    uint32_t unrelated_deadline =
        fixture.cpu.periph_next_event(&fixture.cpu);

    const uint32_t descriptor0 = 0x3FC8F000u;
    const uint32_t descriptor1 = descriptor0 + 12u;
    const uint32_t buffer0 = 0x3FC90000u;
    const uint32_t buffer1 = buffer0 + 8u;
    for (unsigned index = 0u; index < 8u; index++) {
        mem_write8(fixture.mem, buffer0 + index, (uint8_t)(0x10u + index));
        mem_write8(fixture.mem, buffer1 + index, (uint8_t)(0x20u + index));
    }
    i2s_v2_descriptor(fixture.mem, descriptor0, buffer0, 8u, 8u, true,
                      descriptor1);
    i2s_v2_descriptor(fixture.mem, descriptor1, buffer1, 8u, 8u, true,
                      descriptor0);

    i2s_v2_configure_standard_clock(fixture.mem, S3_I2S0_BASE, false);
    mem_write32(fixture.mem, S3_GDMA_OUT_PERI_SEL, S3_GDMA_I2S0_TRIGGER);
    mem_write32(fixture.mem, S3_GDMA_OUT_LINK,
                (descriptor0 & 0xFFFFFu) | S3_GDMA_OUT_LINK_START);
    mem_write32(fixture.mem, S3_I2S0_REG(S3_I2S_TX_CONF_OFF),
                S3_I2S_START | S3_I2S_UPDATE);

    /* The descriptor is ready, but the architectural SYSTEM gate owns the
     * module clock and must be enabled before time can advance. */
    ASSERT_EQ(fixture.cpu.periph_next_event(&fixture.cpu),
              unrelated_deadline);
    ASSERT_EQ(capture.calls, 0u);
    mem_write32(fixture.mem, S3_SYSTEM_CLK_EN0,
                mem_read32(fixture.mem, S3_SYSTEM_CLK_EN0) |
                S3_SYSTEM_I2S0);
    mem_write32(fixture.mem, S3_SYSTEM_RST_EN0,
                mem_read32(fixture.mem, S3_SYSTEM_RST_EN0) &
                ~S3_SYSTEM_I2S0);

    ASSERT_EQ(i2s_v2_service_next(&fixture), 6400u);
    ASSERT_EQ(capture.calls, 1u);
    ASSERT_EQ(capture.length, 8u);
    ASSERT_EQ(capture.sample_rate, 50000u);
    ASSERT_EQ(capture.bits_per_sample, 16u);
    ASSERT_EQ(capture.channels, 2u);
    ASSERT_EQ(mem_read32(fixture.mem, S3_GDMA_OUT_DESC), descriptor0);
    ASSERT_EQ(mem_read32(fixture.mem, S3_GDMA_OUT_EOF_DESC), descriptor0);
    ASSERT_EQ(mem_read32(fixture.mem, S3_GDMA_OUT_INT_RAW), 3u);

    ASSERT_EQ(i2s_v2_service_next(&fixture), 12800u);
    ASSERT_EQ(capture.calls, 2u);
    ASSERT_EQ(capture.length, 16u);
    ASSERT_EQ(mem_read32(fixture.mem, S3_GDMA_OUT_DESC), descriptor1);
    ASSERT_EQ(mem_read32(fixture.mem, descriptor0) & S3_GDMA_DESC_OWNER,
              S3_GDMA_DESC_OWNER);
    for (unsigned index = 0u; index < 8u; index++) {
        ASSERT_EQ(capture.data[index], 0x10u + index);
        ASSERT_EQ(capture.data[8u + index], 0x20u + index);
    }
    ASSERT_EQ(periph_unhandled_count(fixture.periph), 0u);

    i2s_v2_fixture_destroy(&fixture);
}

TEST(esp32s3_i2s_v2_port1_rx_waits_for_host_samples_and_reports_completion)
{
    i2s_v2_fixture_t fixture;
    bool ready = i2s_v2_fixture_init(&fixture);
    ASSERT_TRUE(ready);
    if (!ready) {
        i2s_v2_fixture_destroy(&fixture);
        return;
    }

    mem_write32(fixture.mem, S3_SYSTEM_CLK_EN0,
                mem_read32(fixture.mem, S3_SYSTEM_CLK_EN0) |
                S3_SYSTEM_I2S1);
    const uint8_t input[] = {0x91u, 0x82u, 0x73u, 0x64u, 0x55u, 0x46u};

    const uint32_t descriptor = 0x3FC8F100u;
    const uint32_t buffer = 0x3FC90100u;
    i2s_v2_descriptor(fixture.mem, descriptor, buffer,
                      sizeof(input), 0u, true, 0u);
    i2s_v2_configure_standard_clock(fixture.mem, S3_I2S1_BASE, true);
    mem_write32(fixture.mem, S3_GDMA_IN_PERI_SEL, S3_GDMA_I2S1_TRIGGER);
    mem_write32(fixture.mem, S3_GDMA_IN_LINK,
                (descriptor & 0xFFFFFu) | S3_GDMA_IN_LINK_START);
    mem_write32(fixture.mem, S3_I2S1_BASE + S3_I2S_INT_ENA_OFF, 1u);
    periph_intr_matrix_set(fixture.periph, 0, 8, 26);
    uint32_t unrelated_deadline =
        fixture.cpu.periph_next_event(&fixture.cpu);
    mem_write32(fixture.mem, S3_I2S1_BASE + S3_I2S_RX_CONF_OFF,
                S3_I2S_START | S3_I2S_UPDATE);

    ASSERT_EQ(fixture.cpu.periph_next_event(&fixture.cpu),
              unrelated_deadline);
    ASSERT_EQ(mem_read32(fixture.mem, descriptor) & S3_GDMA_DESC_OWNER,
              S3_GDMA_DESC_OWNER);
    ASSERT_EQ(periph_i2s_rx_inject(
                  fixture.periph, 1, input, sizeof(input)), sizeof(input));
    ASSERT_EQ(periph_i2s_rx_pending(fixture.periph, 1), sizeof(input));
    ASSERT_EQ(i2s_v2_service_next(&fixture), 4800u);
    ASSERT_EQ(periph_i2s_rx_pending(fixture.periph, 1), 0u);
    for (unsigned index = 0u; index < sizeof(input); index++)
        ASSERT_EQ(mem_read8(fixture.mem, buffer + index), input[index]);
    ASSERT_EQ((mem_read32(fixture.mem, descriptor) >> 12u) & 0xFFFu,
              sizeof(input));
    ASSERT_EQ(mem_read32(fixture.mem, descriptor) & S3_GDMA_DESC_OWNER, 0u);
    ASSERT_EQ(mem_read32(fixture.mem, S3_GDMA_IN_INT_RAW), 3u);
    ASSERT_EQ(mem_read32(fixture.mem,
                         S3_I2S1_BASE + S3_I2S_INT_RAW_OFF), 1u);
    ASSERT_EQ(mem_read32(fixture.mem,
                         S3_I2S1_BASE + S3_I2S_INT_ST_OFF), 1u);
    ASSERT_TRUE(periph_interrupt_pending(fixture.periph, 26));
    ASSERT_EQ(fixture.cpu.interrupt & (1u << 8u), 1u << 8u);
    mem_write32(fixture.mem, S3_I2S1_BASE + S3_I2S_INT_CLR_OFF, 1u);
    ASSERT_FALSE(periph_interrupt_pending(fixture.periph, 26));
    ASSERT_EQ(periph_unhandled_count(fixture.periph), 0u);

    i2s_v2_fixture_destroy(&fixture);
}

TEST(esp32s3_i2s_v2_system_reset_cancels_an_armed_stream)
{
    i2s_v2_fixture_t fixture;
    bool ready = i2s_v2_fixture_init(&fixture);
    ASSERT_TRUE(ready);
    if (!ready) {
        i2s_v2_fixture_destroy(&fixture);
        return;
    }

    const uint32_t descriptor = 0x3FC8F200u;
    const uint32_t buffer = 0x3FC90200u;
    mem_write8(fixture.mem, buffer, 0xA5u);
    uint32_t unrelated_deadline =
        fixture.cpu.periph_next_event(&fixture.cpu);
    i2s_v2_descriptor(fixture.mem, descriptor, buffer, 1u, 1u,
                      true, 0u);
    mem_write32(fixture.mem, S3_SYSTEM_CLK_EN0,
                mem_read32(fixture.mem, S3_SYSTEM_CLK_EN0) |
                S3_SYSTEM_I2S0);
    i2s_v2_configure_standard_clock(fixture.mem, S3_I2S0_BASE, false);
    mem_write32(fixture.mem, S3_GDMA_OUT_PERI_SEL, S3_GDMA_I2S0_TRIGGER);
    mem_write32(fixture.mem, S3_GDMA_OUT_LINK,
                (descriptor & 0xFFFFFu) | S3_GDMA_OUT_LINK_START);
    mem_write32(fixture.mem, S3_I2S0_REG(S3_I2S_TX_CONF_OFF), S3_I2S_START);
    ASSERT_TRUE(fixture.cpu.periph_next_event(&fixture.cpu) != UINT32_MAX);

    mem_write32(fixture.mem, S3_SYSTEM_RST_EN0,
                mem_read32(fixture.mem, S3_SYSTEM_RST_EN0) |
                S3_SYSTEM_I2S0);
    ASSERT_EQ(fixture.cpu.periph_next_event(&fixture.cpu),
              unrelated_deadline);
    ASSERT_EQ(mem_read32(fixture.mem, S3_I2S0_REG(S3_I2S_TX_CONF_OFF)),
              (1u << 15u) | (1u << 13u) | (1u << 12u) | (1u << 9u));
    ASSERT_EQ(mem_read32(fixture.mem, S3_I2S0_REG(S3_I2S_DATE_OFF)),
              0x02009070u);
    ASSERT_EQ(periph_unhandled_count(fixture.periph), 0u);

    i2s_v2_fixture_destroy(&fixture);
}

void run_i2s_v2_tests(void)
{
    TEST_SUITE("S3 I2S v2/GDMA");
    RUN_TEST(esp32s3_i2s_v2_streams_circular_tx_descriptors_at_audio_rate);
    RUN_TEST(
        esp32s3_i2s_v2_port1_rx_waits_for_host_samples_and_reports_completion);
    RUN_TEST(esp32s3_i2s_v2_system_reset_cancels_an_armed_stream);
}
