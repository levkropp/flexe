/* Target-described digital pad configuration tests. */
#include "test_helpers.h"
#include "peripherals.h"

TEST(io_mux_preserves_classic_routing_contract)
{
    const flexe_target_desc_t *classic =
        flexe_target_by_id(FLEXE_TARGET_ESP32);
    xtensa_mem_t *mem = mem_create_for_target(classic);
    esp32_periph_t *periph = periph_create(mem);
    ASSERT_TRUE(mem != NULL);
    ASSERT_TRUE(periph != NULL);
    if (!mem || !periph) {
        periph_destroy(periph);
        mem_destroy(mem);
        return;
    }

    ASSERT_TRUE(classic->capabilities & FLEXE_TARGET_CAP_IO_MUX_V1);
    uint32_t gpio0 = classic->io_mux.base +
                     classic->io_mux.gpio_register_offset[0];
    ASSERT_EQ(mem_read32(mem, gpio0), 0x00001800u);
    ASSERT_EQ(periph_iomux_function(periph, 0), -1);
    mem_write32(mem, gpio0, UINT32_MAX);
    ASSERT_EQ(mem_read32(mem, gpio0), 0x0000FFFFu);
    ASSERT_EQ(periph_iomux_function(periph, 0), 7);
    ASSERT_EQ(periph_unhandled_count(periph), 0);

    periph_destroy(periph);
    mem_destroy(mem);
}

TEST(io_mux_models_s3_pad_map_masks_date_and_reserved_offsets)
{
    const flexe_target_desc_t *s3 =
        flexe_target_by_id(FLEXE_TARGET_ESP32S3);
    const flexe_io_mux_desc_t *desc = &s3->io_mux;
    xtensa_mem_t *mem = mem_create_for_target(s3);
    esp32_periph_t *periph = periph_create(mem);
    ASSERT_TRUE(mem != NULL);
    ASSERT_TRUE(periph != NULL);
    if (!mem || !periph) {
        periph_destroy(periph);
        mem_destroy(mem);
        return;
    }

    ASSERT_TRUE(s3->capabilities & FLEXE_TARGET_CAP_IO_MUX_V1);
    ASSERT_EQ(desc->base, 0x60009000u);
    ASSERT_EQ(desc->gpio_register_offset[0], 0x004u);
    ASSERT_EQ(desc->gpio_register_offset[21], 0x058u);
    ASSERT_EQ(desc->gpio_register_offset[22],
              FLEXE_TARGET_IO_MUX_OFFSET_NONE);
    ASSERT_EQ(desc->gpio_register_offset[26], 0x06Cu);
    ASSERT_EQ(desc->gpio_register_offset[48], 0x0C4u);

    uint32_t gpio0 = desc->base + desc->gpio_register_offset[0];
    uint32_t gpio26 = desc->base + desc->gpio_register_offset[26];
    ASSERT_EQ(mem_read32(mem, desc->base + desc->control_offset), 0u);
    ASSERT_EQ(mem_read32(mem, gpio0), 0u);
    ASSERT_EQ(periph_iomux_function(periph, 0), -1);

    mem_write32(mem, gpio0, UINT32_MAX);
    ASSERT_EQ(mem_read32(mem, gpio0), 0x0000FFFFu);
    ASSERT_EQ(periph_iomux_function(periph, 0), 7);
    mem_write32(mem, gpio26, 3u << 12);
    ASSERT_EQ(periph_iomux_function(periph, 26), 3);
    ASSERT_EQ(periph_iomux_function(periph, 22), -1);
    ASSERT_EQ(periph_iomux_function(periph, 49), -1);

    uint32_t date = desc->base + desc->date_offset;
    ASSERT_EQ(mem_read32(mem, date), 0x01907160u);
    mem_write32(mem, date, UINT32_MAX);
    ASSERT_EQ(mem_read32(mem, date), 0x01907160u);

    int before = periph_unhandled_count(periph);
    ASSERT_EQ(mem_read32(mem, desc->base + 0x05Cu), 0u);
    mem_write32(mem, desc->base + 0x05Cu, 1u);
    ASSERT_EQ(periph_unhandled_count(periph), before + 2);
    ASSERT_EQ(mem_unmapped_count(mem), 0u);

    periph_destroy(periph);
    mem_destroy(mem);
}

void run_io_mux_tests(void)
{
    TEST_SUITE("Target IO MUX");
    RUN_TEST(io_mux_preserves_classic_routing_contract);
    RUN_TEST(io_mux_models_s3_pad_map_masks_date_and_reserved_offsets);
}
