/* Target-described read-only eFuse tests. */
#include "test_helpers.h"
#include "efuse.h"
#include "peripherals.h"

typedef struct {
    unsigned reads;
    unsigned writes;
} efuse_fallback_t;

static uint32_t efuse_test_fallback_read(void *ctx, uint32_t addr)
{
    efuse_fallback_t *fallback = ctx;
    fallback->reads++;
    return addr ^ 0xA5A5A5A5u;
}

static void efuse_test_fallback_write(void *ctx, uint32_t addr,
                                      uint32_t value)
{
    efuse_fallback_t *fallback = ctx;
    fallback->writes++;
    (void)addr;
    (void)value;
}

TEST(efuse_exposes_read_only_virtual_silicon_profile)
{
    const flexe_target_desc_t *s3 =
        flexe_target_by_id(FLEXE_TARGET_ESP32S3);
    const flexe_efuse_desc_t *desc = &s3->efuse;
    xtensa_mem_t *mem = mem_create_for_target(s3);
    efuse_fallback_t fallback = {0};
    flexe_efuse_t *efuse = flexe_efuse_create(
        mem, efuse_test_fallback_read, efuse_test_fallback_write, &fallback);
    ASSERT_TRUE(mem != NULL);
    ASSERT_TRUE(efuse != NULL);
    if (!mem || !efuse) {
        flexe_efuse_destroy(efuse);
        mem_destroy(mem);
        return;
    }

    uint32_t mac0 = desc->base + 0x044u;
    uint32_t mac1 = desc->base + 0x048u;
    uint32_t revision = desc->base + 0x058u;
    ASSERT_EQ(mem_read32(mem, mac0), 0x00000001u);
    ASSERT_EQ(mem_read32(mem, mac1), 0x00000200u);
    ASSERT_EQ(mem_read32(mem, revision), 0u);
    mem_write32(mem, mac0, UINT32_MAX);
    ASSERT_EQ(mem_read32(mem, mac0), 0x00000001u);

    /* ESP_EFUSE_MAC_FACTORY lists bytes from BLK1 bit 40 down to bit 0.
     * Reconstruct that public API view so a word-order regression cannot
     * silently turn the virtual station identity into a multicast address. */
    uint32_t mac_low = mem_read32(mem, mac0);
    uint32_t mac_high = mem_read32(mem, mac1);
    uint8_t mac[6] = {
        (uint8_t)(mac_high >> 8), (uint8_t)mac_high,
        (uint8_t)(mac_low >> 24), (uint8_t)(mac_low >> 16),
        (uint8_t)(mac_low >> 8), (uint8_t)mac_low,
    };
    ASSERT_EQ(mac[0], 0x02u);
    ASSERT_EQ(mac[1], 0x00u);
    ASSERT_EQ(mac[2], 0x00u);
    ASSERT_EQ(mac[3], 0x00u);
    ASSERT_EQ(mac[4], 0x00u);
    ASSERT_EQ(mac[5], 0x01u);
    ASSERT_EQ(mac[0] & 1u, 0u);
    ASSERT_TRUE(mac[0] & 2u);

    uint32_t date = desc->base + desc->date_offset;
    ASSERT_EQ(mem_read32(mem, date), 0x02101290u);
    mem_write32(mem, date, UINT32_MAX);
    ASSERT_EQ(mem_read32(mem, date), 0x0FFFFFFFu);
    ASSERT_EQ(fallback.reads, 0u);
    ASSERT_EQ(fallback.writes, 0u);

    flexe_efuse_destroy(efuse);
    mem_destroy(mem);
}

TEST(efuse_programming_controls_remain_explicitly_unsupported)
{
    const flexe_target_desc_t *s3 =
        flexe_target_by_id(FLEXE_TARGET_ESP32S3);
    const flexe_efuse_desc_t *desc = &s3->efuse;
    xtensa_mem_t *mem = mem_create_for_target(s3);
    esp32_periph_t *periph = periph_create(mem);
    ASSERT_TRUE(mem != NULL);
    ASSERT_TRUE(periph != NULL);
    if (!mem || !periph) {
        periph_destroy(periph);
        mem_destroy(mem);
        return;
    }

    ASSERT_EQ(mem_read32(mem, desc->base + 0x03Cu), 0u);
    ASSERT_EQ(periph_unhandled_count(periph), 0);
    mem_write32(mem, desc->base + 0x000u, 1u);
    ASSERT_EQ(periph_unhandled_count(periph), 1);
    ASSERT_EQ(mem_read32(mem, desc->base + 0x1D4u), 0u);
    ASSERT_EQ(periph_unhandled_count(periph), 2);

    periph_destroy(periph);
    mem_destroy(mem);
}

void run_efuse_tests(void)
{
    TEST_SUITE("Target eFuse");
    RUN_TEST(efuse_exposes_read_only_virtual_silicon_profile);
    RUN_TEST(efuse_programming_controls_remain_explicitly_unsupported);
}
