/* ESP32-S3 EXTMEM cache-controller tests. */
#include "test_helpers.h"
#include "esp32s3_extmem.h"
#include "peripherals.h"

#define S3_EXTMEM_BASE 0x600C4000u

typedef struct {
    unsigned calls;
    uint32_t addr;
    size_t len;
} s3_invalidate_probe_t;

static void s3_invalidate(void *ctx, uint32_t addr, size_t len)
{
    s3_invalidate_probe_t *probe = ctx;
    probe->calls++;
    probe->addr = addr;
    probe->len = len;
}

TEST(esp32s3_extmem_exposes_documented_reset_state) {
    const flexe_target_desc_t *s3 =
        flexe_target_by_id(FLEXE_TARGET_ESP32S3);
    xtensa_mem_t *mem = mem_create_for_target(s3);
    flexe_esp32s3_extmem_t *extmem =
        flexe_esp32s3_extmem_create(mem);
    ASSERT_TRUE(mem != NULL);
    ASSERT_TRUE(extmem != NULL);
    if (!mem || !extmem) {
        flexe_esp32s3_extmem_destroy(extmem);
        mem_destroy(mem);
        return;
    }

    ASSERT_EQ(mem_read32(mem, S3_EXTMEM_BASE + 0x000u), 0u);
    ASSERT_EQ(mem_read32(mem, S3_EXTMEM_BASE + 0x004u), 3u);
    ASSERT_EQ(mem_read32(mem, S3_EXTMEM_BASE + 0x060u), 0u);
    ASSERT_EQ(mem_read32(mem, S3_EXTMEM_BASE + 0x064u), 3u);
    ASSERT_EQ(mem_read32(mem, S3_EXTMEM_BASE + 0x130u), 0x1001u);
    ASSERT_EQ(mem_read32(mem, S3_EXTMEM_BASE + 0x3FCu), 0x02012310u);
    ASSERT_EQ(mem_unmapped_count(mem), 0u);

    flexe_esp32s3_extmem_destroy(extmem);
    mem_destroy(mem);
}

TEST(esp32s3_extmem_application_handoff_enables_cache_buses) {
    const flexe_target_desc_t *s3 =
        flexe_target_by_id(FLEXE_TARGET_ESP32S3);
    xtensa_mem_t *mem = mem_create_for_target(s3);
    flexe_esp32s3_extmem_t *extmem =
        flexe_esp32s3_extmem_create(mem);
    ASSERT_TRUE(mem != NULL);
    ASSERT_TRUE(extmem != NULL);
    if (!mem || !extmem) {
        flexe_esp32s3_extmem_destroy(extmem);
        mem_destroy(mem);
        return;
    }

    flexe_esp32s3_extmem_application_handoff(extmem);
    ASSERT_EQ(mem_read32(mem, S3_EXTMEM_BASE + 0x000u) & 1u, 1u);
    ASSERT_EQ(mem_read32(mem, S3_EXTMEM_BASE + 0x004u), 0u);
    ASSERT_EQ(mem_read32(mem, S3_EXTMEM_BASE + 0x060u) & 1u, 1u);
    ASSERT_EQ(mem_read32(mem, S3_EXTMEM_BASE + 0x064u), 0u);
    ASSERT_EQ(mem_read32(mem, S3_EXTMEM_BASE + 0x0B4u), 0x42000000u);
    ASSERT_EQ(mem_read32(mem, S3_EXTMEM_BASE + 0x0B8u), 0x43FFFFFFu);
    ASSERT_EQ(mem_read32(mem, S3_EXTMEM_BASE + 0x0BCu), 0x3C000000u);
    ASSERT_EQ(mem_read32(mem, S3_EXTMEM_BASE + 0x0C0u), 0x3DFFFFFFu);

    flexe_esp32s3_extmem_destroy(extmem);
    mem_destroy(mem);
}

TEST(esp32s3_extmem_operations_complete_and_invalidate_code) {
    const flexe_target_desc_t *s3 =
        flexe_target_by_id(FLEXE_TARGET_ESP32S3);
    xtensa_mem_t *mem = mem_create_for_target(s3);
    flexe_esp32s3_extmem_t *extmem =
        flexe_esp32s3_extmem_create(mem);
    xtensa_cpu_t cpu[2];
    s3_invalidate_probe_t probe[2] = {0};
    ASSERT_TRUE(mem != NULL);
    ASSERT_TRUE(extmem != NULL);
    if (!mem || !extmem) {
        flexe_esp32s3_extmem_destroy(extmem);
        mem_destroy(mem);
        return;
    }

    xtensa_cpu_init_for_target(&cpu[0], s3);
    xtensa_cpu_init_for_target(&cpu[1], s3);
    cpu[0].code_invalidate = s3_invalidate;
    cpu[0].code_invalidate_ctx = &probe[0];
    cpu[1].code_invalidate = s3_invalidate;
    cpu[1].code_invalidate_ctx = &probe[1];
    flexe_esp32s3_extmem_attach_cpus(extmem, &cpu[0], &cpu[1]);

    mem_write32(mem, S3_EXTMEM_BASE + 0x08Cu, 0x42012340u);
    mem_write32(mem, S3_EXTMEM_BASE + 0x090u, 4u);
    mem_write32(mem, S3_EXTMEM_BASE + 0x088u, 1u);
    ASSERT_EQ(mem_read32(mem, S3_EXTMEM_BASE + 0x088u), 2u);
    ASSERT_EQ(probe[0].calls, 1u);
    ASSERT_EQ(probe[1].calls, 1u);
    ASSERT_EQ(probe[0].addr, 0x42012340u);
    ASSERT_EQ(probe[0].len, 64u);

    mem_write32(mem, S3_EXTMEM_BASE + 0x028u, 7u);
    ASSERT_EQ(mem_read32(mem, S3_EXTMEM_BASE + 0x028u), 8u);
    mem_write32(mem, S3_EXTMEM_BASE + 0x01Cu, 3u);
    ASSERT_EQ(mem_read32(mem, S3_EXTMEM_BASE + 0x01Cu), 4u);
    mem_write32(mem, S3_EXTMEM_BASE + 0x040u, 5u);
    ASSERT_EQ(mem_read32(mem, S3_EXTMEM_BASE + 0x040u), 6u);

    mem_write32(mem, S3_EXTMEM_BASE + 0x150u, 1u);
    ASSERT_EQ(mem_read32(mem, S3_EXTMEM_BASE + 0x150u), 5u);
    mem_write32(mem, S3_EXTMEM_BASE + 0x150u, 4u);
    ASSERT_EQ(mem_read32(mem, S3_EXTMEM_BASE + 0x150u), 0u);
    mem_write32(mem, S3_EXTMEM_BASE + 0x154u, 3u);
    ASSERT_EQ(mem_read32(mem, S3_EXTMEM_BASE + 0x154u), 7u);
    mem_write32(mem, S3_EXTMEM_BASE + 0x154u, 6u);
    ASSERT_EQ(mem_read32(mem, S3_EXTMEM_BASE + 0x154u), 2u);

    flexe_esp32s3_extmem_destroy(extmem);
    mem_destroy(mem);
}

TEST(peripherals_compose_s3_devices_without_classic_aliases) {
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

    /* periph_create models direct application entry and owns the shared MMU. */
    ASSERT_EQ(mem_read32(mem, S3_EXTMEM_BASE) & 1u, 1u);
    ASSERT_EQ(mem_read32(mem, S3_EXTMEM_BASE + 0x60u) & 1u, 1u);
    ASSERT_EQ(mem_read32(mem, 0x600C5000u), 0x4000u);
    ASSERT_TRUE(mem_get_ptr(mem, 0x42000000u) == NULL);

    /* A classic DPORT address is S3 D-ROM, not an alias into native MMIO. */
    mem_write32(mem, 0x40040000u, 0xA55A1234u);
    uint64_t before = mem_unmapped_count(mem);
    ASSERT_EQ(mem_read32(mem, 0x3FF00000u), 0xA55A1234u);
    ASSERT_EQ(mem_unmapped_count(mem), before);

    periph_destroy(periph);
    mem_destroy(mem);
}

void run_esp32s3_extmem_tests(void) {
    TEST_SUITE("ESP32-S3 EXTMEM");
    RUN_TEST(esp32s3_extmem_exposes_documented_reset_state);
    RUN_TEST(esp32s3_extmem_application_handoff_enables_cache_buses);
    RUN_TEST(esp32s3_extmem_operations_complete_and_invalidate_code);
    RUN_TEST(peripherals_compose_s3_devices_without_classic_aliases);
}
