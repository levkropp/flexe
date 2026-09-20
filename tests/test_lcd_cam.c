#include "test_helpers.h"

#include "peripherals.h"
#include "target.h"

#include <string.h>

#define S3_SYSTEM_CLK_EN1       0x600C001Cu
#define S3_SYSTEM_RST_EN1       0x600C0024u
#define S3_SYSTEM_LCD_CAM       (1u << 8)

#define S3_LCD_CAM_BASE         0x60041000u
#define S3_LCD_USER             (S3_LCD_CAM_BASE + 0x014u)
#define S3_LCD_MISC             (S3_LCD_CAM_BASE + 0x018u)
#define S3_LCD_CTRL             (S3_LCD_CAM_BASE + 0x01Cu)
#define S3_LCD_CTRL2            (S3_LCD_CAM_BASE + 0x024u)
#define S3_LCD_CMD              (S3_LCD_CAM_BASE + 0x028u)
#define S3_LCD_INT_ENA          (S3_LCD_CAM_BASE + 0x064u)
#define S3_LCD_INT_RAW          (S3_LCD_CAM_BASE + 0x068u)
#define S3_LCD_INT_ST           (S3_LCD_CAM_BASE + 0x06Cu)
#define S3_LCD_INT_CLR          (S3_LCD_CAM_BASE + 0x070u)
#define S3_LCD_DATE             (S3_LCD_CAM_BASE + 0x0FCu)
#define S3_CAM_CTRL1            (S3_LCD_CAM_BASE + 0x008u)

#define S3_LCD_ALWAYS_OUT       (1u << 13)
#define S3_LCD_SWIZZLE          (1u << 19)
#define S3_LCD_UPDATE           (1u << 20)
#define S3_LCD_2BYTE            (1u << 23)
#define S3_LCD_DOUT             (1u << 24)
#define S3_LCD_DUMMY            (1u << 25)
#define S3_LCD_COMMAND          (1u << 26)
#define S3_LCD_START            (1u << 27)
#define S3_LCD_TRANS_DONE       (1u << 1)

#define S3_GDMA_BASE            0x6003F000u
#define S3_GDMA_OUT_INT_RAW     (S3_GDMA_BASE + 0x068u)
#define S3_GDMA_OUT_LINK        (S3_GDMA_BASE + 0x080u)
#define S3_GDMA_OUT_EOF_DESC    (S3_GDMA_BASE + 0x088u)
#define S3_GDMA_OUT_DESC        (S3_GDMA_BASE + 0x090u)
#define S3_GDMA_OUT_PERI_SEL    (S3_GDMA_BASE + 0x0A8u)
#define S3_GDMA_OUT_LINK_START  (1u << 21)
#define S3_GDMA_DESC_EOF        (1u << 30)
#define S3_GDMA_DESC_OWNER      (1u << 31)
#define S3_GDMA_LCD_TRIGGER     5u

typedef struct {
    xtensa_mem_t *mem;
    esp32_periph_t *periph;
    xtensa_cpu_t cpu;
} lcd_cam_fixture_t;

typedef struct {
    uint8_t data[64];
    size_t length;
    unsigned calls;
    unsigned first_calls;
    unsigned last_calls;
    flexe_lcd_cam_i80_transfer_t last;
} lcd_cam_capture_t;

static bool lcd_cam_fixture_init(lcd_cam_fixture_t *fixture)
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

static void lcd_cam_fixture_destroy(lcd_cam_fixture_t *fixture)
{
    periph_destroy(fixture->periph);
    mem_destroy(fixture->mem);
}

static void lcd_cam_enable(lcd_cam_fixture_t *fixture)
{
    mem_write32(fixture->mem, S3_SYSTEM_CLK_EN1,
                mem_read32(fixture->mem, S3_SYSTEM_CLK_EN1) |
                S3_SYSTEM_LCD_CAM);
    mem_write32(fixture->mem, S3_SYSTEM_RST_EN1,
                mem_read32(fixture->mem, S3_SYSTEM_RST_EN1) &
                ~S3_SYSTEM_LCD_CAM);
}

static void lcd_cam_descriptor(xtensa_mem_t *mem, uint32_t descriptor,
                               uint32_t buffer, size_t length, bool eof,
                               uint32_t next)
{
    uint32_t control = (uint32_t)length |
                       ((uint32_t)length << 12u) |
                       S3_GDMA_DESC_OWNER;
    if (eof) control |= S3_GDMA_DESC_EOF;
    mem_write32(mem, descriptor, control);
    mem_write32(mem, descriptor + 4u, buffer);
    mem_write32(mem, descriptor + 8u, next);
}

static void lcd_cam_capture(void *ctx,
                            const flexe_lcd_cam_i80_transfer_t *transfer,
                            const uint8_t *data, size_t length)
{
    lcd_cam_capture_t *capture = ctx;
    if (capture->length + length <= sizeof(capture->data) && length)
        memcpy(capture->data + capture->length, data, length);
    capture->length += length;
    capture->calls++;
    capture->first_calls += transfer->first;
    capture->last_calls += transfer->last;
    capture->last = *transfer;
}

TEST(esp32s3_lcd_cam_register_gate_command_and_interrupt_semantics)
{
    lcd_cam_fixture_t fixture;
    bool ready = lcd_cam_fixture_init(&fixture);
    ASSERT_TRUE(ready);
    if (!ready) {
        lcd_cam_fixture_destroy(&fixture);
        return;
    }

    lcd_cam_capture_t capture = {0};
    ASSERT_EQ(periph_set_lcd_cam_i80_callback(
                  fixture.periph, lcd_cam_capture, &capture), 0);
    ASSERT_EQ(mem_read32(fixture.mem, S3_LCD_MISC), 0x22u);
    ASSERT_EQ(mem_read32(fixture.mem, S3_LCD_DATE), 0x02003020u);
    mem_write32(fixture.mem, S3_LCD_CTRL2, UINT32_MAX);
    ASSERT_EQ(mem_read32(fixture.mem, S3_LCD_CTRL2), 0xFFFF03FFu);

    mem_write32(fixture.mem, S3_LCD_INT_ENA, S3_LCD_TRANS_DONE);
    periph_intr_matrix_set(fixture.periph, 0, 9, 24);
    mem_write32(fixture.mem, S3_LCD_USER,
                S3_LCD_DUMMY | S3_LCD_UPDATE | S3_LCD_START);
    ASSERT_EQ(capture.calls, 0u);
    ASSERT_EQ(mem_read32(fixture.mem, S3_LCD_INT_RAW), 0u);

    lcd_cam_enable(&fixture);
    ASSERT_EQ(capture.calls, 1u);
    ASSERT_EQ(capture.length, 0u);
    ASSERT_EQ(capture.first_calls, 1u);
    ASSERT_EQ(capture.last_calls, 1u);
    ASSERT_TRUE(capture.last.dummy_enabled);
    ASSERT_FALSE(capture.last.data_enabled);
    ASSERT_EQ(mem_read32(fixture.mem, S3_LCD_USER) &
              (S3_LCD_UPDATE | S3_LCD_START), 0u);
    ASSERT_EQ(mem_read32(fixture.mem, S3_LCD_INT_RAW),
              S3_LCD_TRANS_DONE);
    ASSERT_EQ(mem_read32(fixture.mem, S3_LCD_INT_ST),
              S3_LCD_TRANS_DONE);
    ASSERT_TRUE(periph_interrupt_pending(fixture.periph, 24));
    ASSERT_EQ(fixture.cpu.interrupt & (1u << 9u), 1u << 9u);
    mem_write32(fixture.mem, S3_LCD_INT_CLR, S3_LCD_TRANS_DONE);
    ASSERT_FALSE(periph_interrupt_pending(fixture.periph, 24));

    mem_write32(fixture.mem, S3_SYSTEM_RST_EN1,
                mem_read32(fixture.mem, S3_SYSTEM_RST_EN1) |
                S3_SYSTEM_LCD_CAM);
    ASSERT_EQ(mem_read32(fixture.mem, S3_LCD_MISC), 0x22u);
    ASSERT_EQ(mem_read32(fixture.mem, S3_LCD_CTRL2), 0u);
    ASSERT_EQ(mem_read32(fixture.mem, S3_LCD_DATE), 0x02003020u);
    ASSERT_EQ(periph_unhandled_count(fixture.periph), 0u);

    lcd_cam_fixture_destroy(&fixture);
}

TEST(esp32s3_lcd_cam_i80_drains_gdma_chain_in_wire_order)
{
    lcd_cam_fixture_t fixture;
    bool ready = lcd_cam_fixture_init(&fixture);
    ASSERT_TRUE(ready);
    if (!ready) {
        lcd_cam_fixture_destroy(&fixture);
        return;
    }
    lcd_cam_enable(&fixture);
    lcd_cam_capture_t capture = {0};
    ASSERT_EQ(periph_set_lcd_cam_i80_callback(
                  fixture.periph, lcd_cam_capture, &capture), 0);

    const uint32_t descriptor0 = 0x3FC8F300u;
    const uint32_t descriptor1 = descriptor0 + 12u;
    const uint32_t buffer0 = 0x3FC90300u;
    const uint32_t buffer1 = buffer0 + 4u;
    const uint8_t bytes[] = { 0x12u, 0x34u, 0x56u, 0x78u,
                              0x9Au, 0xBCu };
    for (unsigned index = 0u; index < sizeof(bytes); index++)
        mem_write8(fixture.mem, buffer0 + index, bytes[index]);
    lcd_cam_descriptor(fixture.mem, descriptor0, buffer0, 4u, false,
                       descriptor1);
    lcd_cam_descriptor(fixture.mem, descriptor1, buffer1, 2u, true, 0u);

    mem_write32(fixture.mem, S3_GDMA_OUT_PERI_SEL,
                S3_GDMA_LCD_TRIGGER);
    mem_write32(fixture.mem, S3_GDMA_OUT_LINK,
                (descriptor0 & 0xFFFFFu) | S3_GDMA_OUT_LINK_START);
    mem_write32(fixture.mem, S3_LCD_CMD, 0x00002C00u);
    mem_write32(fixture.mem, S3_LCD_INT_ENA, S3_LCD_TRANS_DONE);
    mem_write32(fixture.mem, S3_LCD_USER,
                S3_LCD_ALWAYS_OUT | S3_LCD_SWIZZLE | S3_LCD_2BYTE |
                S3_LCD_COMMAND | S3_LCD_DOUT | S3_LCD_START);

    ASSERT_EQ(capture.calls, 2u);
    ASSERT_EQ(capture.first_calls, 1u);
    ASSERT_EQ(capture.last_calls, 1u);
    ASSERT_EQ(capture.length, sizeof(bytes));
    const uint8_t expected[] = { 0x34u, 0x12u, 0x78u, 0x56u,
                                 0xBCu, 0x9Au };
    for (unsigned index = 0u; index < sizeof(expected); index++)
        ASSERT_EQ(capture.data[index], expected[index]);
    ASSERT_TRUE(capture.last.command_enabled);
    ASSERT_TRUE(capture.last.data_enabled);
    ASSERT_EQ(capture.last.command, 0x00002C00u);
    ASSERT_EQ(capture.last.command_cycles, 1u);
    ASSERT_EQ(capture.last.bus_width, 16u);
    ASSERT_EQ(mem_read32(fixture.mem, S3_GDMA_OUT_DESC), descriptor1);
    ASSERT_EQ(mem_read32(fixture.mem, S3_GDMA_OUT_EOF_DESC), descriptor1);
    ASSERT_EQ(mem_read32(fixture.mem, S3_GDMA_OUT_INT_RAW), 0xBu);
    ASSERT_EQ(mem_read32(fixture.mem, S3_LCD_INT_ST),
              S3_LCD_TRANS_DONE);
    ASSERT_EQ(periph_unhandled_count(fixture.periph), 0u);

    lcd_cam_fixture_destroy(&fixture);
}

TEST(esp32s3_lcd_cam_diagnoses_unimplemented_transfer_modes)
{
    lcd_cam_fixture_t fixture;
    bool ready = lcd_cam_fixture_init(&fixture);
    ASSERT_TRUE(ready);
    if (!ready) {
        lcd_cam_fixture_destroy(&fixture);
        return;
    }
    lcd_cam_enable(&fixture);
    ASSERT_EQ(periph_unhandled_count(fixture.periph), 0u);

    mem_write32(fixture.mem, S3_CAM_CTRL1, 1u << 29);
    ASSERT_EQ(periph_unhandled_count(fixture.periph), 1u);
    mem_write32(fixture.mem, S3_LCD_CTRL, 1u << 31);
    mem_write32(fixture.mem, S3_LCD_USER,
                S3_LCD_DUMMY | S3_LCD_START);
    ASSERT_EQ(periph_unhandled_count(fixture.periph), 2u);
    ASSERT_EQ(mem_read32(fixture.mem, S3_LCD_INT_RAW), 0u);

    mem_write32(fixture.mem, S3_LCD_CTRL, 0u);
    mem_write32(fixture.mem, S3_LCD_USER,
                S3_LCD_DOUT | S3_LCD_START | 7u);
    ASSERT_EQ(periph_unhandled_count(fixture.periph), 3u);
    ASSERT_EQ(mem_read32(fixture.mem, S3_LCD_INT_RAW), 0u);

    lcd_cam_fixture_destroy(&fixture);
}

void run_lcd_cam_tests(void)
{
    TEST_SUITE("S3 LCD_CAM i80/GDMA");
    RUN_TEST(esp32s3_lcd_cam_register_gate_command_and_interrupt_semantics);
    RUN_TEST(esp32s3_lcd_cam_i80_drains_gdma_chain_in_wire_order);
    RUN_TEST(esp32s3_lcd_cam_diagnoses_unimplemented_transfer_modes);
}
