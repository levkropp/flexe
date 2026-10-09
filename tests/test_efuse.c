/* Target-described read-only eFuse tests. */
#include "test_helpers.h"
#include "efuse.h"
#include "peripherals.h"
#include "target.h"

#include <stdio.h>

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

TEST(efuse_blob_load_validates_size)
{
    static const char *path = "/tmp/xt_test_efuse.bin";
    uint8_t exact[FLEXE_EFUSE_BLOB_BYTES];
    for (unsigned i = 0u; i < sizeof(exact); i++)
        exact[i] = (uint8_t)(i * 7u + 1u);
    uint32_t words[FLEXE_EFUSE_BLOB_WORDS];

    ASSERT_EQ(flexe_efuse_blob_load(NULL, words), -1);
    ASSERT_EQ(flexe_efuse_blob_load(path, NULL), -1);
    ASSERT_EQ(flexe_efuse_blob_load("/tmp/nonexistent_efuse_blob_12345.bin",
                                     words),
              -1);

    FILE *f = fopen(path, "wb");
    ASSERT_TRUE(f != NULL);
    if (!f) return;
    fwrite(exact, 1u, sizeof(exact) - 1u, f);
    fclose(f);
    ASSERT_EQ(flexe_efuse_blob_load(path, words), -1);

    f = fopen(path, "wb");
    ASSERT_TRUE(f != NULL);
    if (!f) return;
    fwrite(exact, 1u, sizeof(exact), f);
    fputc(0, f);
    fclose(f);
    ASSERT_EQ(flexe_efuse_blob_load(path, words), -1);

    f = fopen(path, "wb");
    ASSERT_TRUE(f != NULL);
    if (!f) return;
    fwrite(exact, 1u, sizeof(exact), f);
    fclose(f);
    ASSERT_EQ(flexe_efuse_blob_load(path, words), 0);
    ASSERT_EQ(words[0], (uint32_t)exact[0] | ((uint32_t)exact[1] << 8) |
                            ((uint32_t)exact[2] << 16) |
                            ((uint32_t)exact[3] << 24));
    ASSERT_EQ(words[FLEXE_EFUSE_BLOB_WORDS - 1u],
              (uint32_t)exact[sizeof(exact) - 4u] |
                  ((uint32_t)exact[sizeof(exact) - 3u] << 8) |
                  ((uint32_t)exact[sizeof(exact) - 2u] << 16) |
                  ((uint32_t)exact[sizeof(exact) - 1u] << 24));
    remove(path);
}

TEST(efuse_blob_overrides_s3_identity_fields)
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

    /* MAC 24:0A:C4:11:22:33, silicon rev 1.1, ADC-calibrated blocks. */
    uint32_t blob[FLEXE_EFUSE_BLOB_WORDS] = {0};
    blob[6] = 0xC4112233u;
    blob[7] = 0x240Au;
    blob[9] = (1u << 18) | (1u << 24);
    blob[11] = (1u << 24);
    blob[16] = 1u;
    ASSERT_EQ(flexe_efuse_apply_blob(NULL, blob), -1);
    ASSERT_EQ(flexe_efuse_apply_blob(efuse, NULL), -1);
    ASSERT_EQ(flexe_efuse_apply_blob(efuse, blob), 0);

    uint32_t mac0 = desc->base + 0x044u;
    uint32_t mac1 = desc->base + 0x048u;
    ASSERT_EQ(mem_read32(mem, mac0), 0xC4112233u);
    ASSERT_EQ(mem_read32(mem, mac1), 0x240Au);
    uint32_t sys3 = mem_read32(mem, desc->base + 0x050u);
    ASSERT_EQ((sys3 >> 18) & 7u, 1u);
    ASSERT_EQ((sys3 >> 24) & 7u, 1u);
    uint32_t sys5 = mem_read32(mem, desc->base + 0x058u);
    ASSERT_EQ((sys5 >> 23) & 1u, 0u);
    ASSERT_EQ((sys5 >> 24) & 3u, 1u);
    ASSERT_EQ(mem_read32(mem, desc->base + 0x06Cu) & 3u, 1u);

    /* Untouched profile words and the read-only contract survive. */
    ASSERT_EQ(mem_read32(mem, desc->base + 0x02Cu), desc->read_data[0]);
    mem_write32(mem, mac0, UINT32_MAX);
    ASSERT_EQ(mem_read32(mem, mac0), 0xC4112233u);

    /* The shared descriptor keeps its compiled defaults: the blob only
     * changed this instance. */
    ASSERT_EQ(desc->read_data[6], 0x00000001u);
    ASSERT_EQ(desc->read_data[7], 0x00000200u);
    ASSERT_EQ(desc->read_data[9], 0u);
    ASSERT_EQ(desc->read_data[11], 0u);

    flexe_efuse_destroy(efuse);
    mem_destroy(mem);
}

TEST(efuse_blob_without_device_reports_absence)
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

    uint32_t blob[FLEXE_EFUSE_BLOB_WORDS] = {0};
    ASSERT_EQ(periph_apply_efuse_blob(NULL, blob), -1);
    ASSERT_EQ(periph_apply_efuse_blob(periph, NULL), -1);
    /* Classic ESP32 has no eFuse device yet, so there is nothing to merge
     * into; the session turns this into a clear --efuse diagnostic. */
    ASSERT_EQ(periph_apply_efuse_blob(periph, blob), -2);

    periph_destroy(periph);
    mem_destroy(mem);
}

void run_efuse_tests(void)
{
    TEST_SUITE("Target eFuse");
    RUN_TEST(efuse_exposes_read_only_virtual_silicon_profile);
    RUN_TEST(efuse_programming_controls_remain_explicitly_unsupported);
    RUN_TEST(efuse_blob_load_validates_size);
    RUN_TEST(efuse_blob_overrides_s3_identity_fields);
    RUN_TEST(efuse_blob_without_device_reports_absence);
}
