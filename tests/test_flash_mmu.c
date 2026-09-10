/* Shared instruction/data flash-MMU tests (ESP32-S3 geometry). */
#include "test_helpers.h"
#include "flash_mmu.h"

typedef struct {
    unsigned calls;
    uint32_t addr;
    size_t len;
} flash_mmu_invalidate_probe_t;

static void flash_mmu_test_invalidate(void *ctx, uint32_t addr, size_t len) {
    flash_mmu_invalidate_probe_t *probe = ctx;
    probe->calls++;
    probe->addr = addr;
    probe->len = len;
}

TEST(shared_flash_mmu_maps_flash_on_both_cache_buses) {
    const flexe_target_desc_t *s3 =
        flexe_target_by_id(FLEXE_TARGET_ESP32S3);
    xtensa_mem_t *mem = mem_create_for_target(s3);
    ASSERT_TRUE(mem != NULL);
    if (!mem) return;

    /* Constructor mappings are only a loader convenience. The MMU's reset
     * state is invalid, so attaching the device removes them. */
    ASSERT_TRUE(mem_get_ptr(mem, 0x3C0D0000u) != NULL);
    flexe_flash_mmu_t *mmu = flexe_flash_mmu_create(mem);
    ASSERT_TRUE(mmu != NULL);
    if (!mmu) {
        mem_destroy(mem);
        return;
    }
    ASSERT_TRUE(mem_get_ptr(mem, 0x3C0D0000u) == NULL);
    ASSERT_TRUE(mem_get_ptr(mem, 0x420D0000u) == NULL);
    ASSERT_EQ(mem_read32(mem, 0x600C5000u + 13u * 4u), 0x4000u);

    mem->flash_data[0x20000u + 0x123u] = 0x5Au;
    mem->flash_insn[0x20000u + 0x123u] = 0xA5u;
    mem_write32(mem, 0x600C5000u + 13u * 4u, 2u);
    ASSERT_EQ(mem_read32(mem, 0x600C5000u + 13u * 4u), 2u);
    ASSERT_EQ(mem_read8(mem, 0x3C0D0123u), 0x5Au);
    ASSERT_EQ(mem_read8(mem, 0x420D0123u), 0xA5u);
    ASSERT_TRUE(mem_get_ptr(mem, 0x3C0D0000u) ==
                mem->flash_data + 0x20000u);
    ASSERT_TRUE(mem_get_ptr(mem, 0x420D0000u) ==
                mem->flash_insn + 0x20000u);

    flexe_flash_mmu_destroy(mmu);
    mem_destroy(mem);
}

TEST(shared_flash_mmu_invalid_and_unbacked_entries_unmap) {
    const flexe_target_desc_t *s3 =
        flexe_target_by_id(FLEXE_TARGET_ESP32S3);
    xtensa_mem_t *mem = mem_create_for_target(s3);
    flexe_flash_mmu_t *mmu = flexe_flash_mmu_create(mem);
    ASSERT_TRUE(mem != NULL);
    ASSERT_TRUE(mmu != NULL);
    if (!mem || !mmu) {
        flexe_flash_mmu_destroy(mmu);
        mem_destroy(mem);
        return;
    }

    const uint32_t table = 0x600C5000u;
    mem_write32(mem, table, 1u);
    ASSERT_TRUE(mem_get_ptr(mem, 0x3C000000u) != NULL);
    ASSERT_TRUE(mem_get_ptr(mem, 0x42000000u) != NULL);

    mem_write32(mem, table, 0x4001u);
    ASSERT_TRUE(mem_get_ptr(mem, 0x3C000000u) == NULL);
    ASSERT_TRUE(mem_get_ptr(mem, 0x42000000u) == NULL);

    /* Type bit 15 selects PSRAM. This descriptor has no PSRAM backing, so
     * the raw entry remains visible while accesses are deliberately unmapped. */
    mem_write32(mem, table, 0x8001u);
    ASSERT_EQ(mem_read32(mem, table), 0x8001u);
    ASSERT_TRUE(mem_get_ptr(mem, 0x3C000000u) == NULL);
    ASSERT_TRUE(mem_get_ptr(mem, 0x42000000u) == NULL);

    /* Page 64 starts just beyond Flexe's current 4 MiB flash backing. */
    mem_write32(mem, table, 64u);
    ASSERT_EQ(mem_read32(mem, table), 64u);
    ASSERT_TRUE(mem_get_ptr(mem, 0x3C000000u) == NULL);
    ASSERT_TRUE(mem_get_ptr(mem, 0x42000000u) == NULL);

    flexe_flash_mmu_destroy(mmu);
    mem_destroy(mem);
}

TEST(shared_flash_mmu_invalidates_each_independent_engine) {
    const flexe_target_desc_t *s3 =
        flexe_target_by_id(FLEXE_TARGET_ESP32S3);
    xtensa_mem_t *mem = mem_create_for_target(s3);
    flexe_flash_mmu_t *mmu = flexe_flash_mmu_create(mem);
    xtensa_cpu_t cpu[2];
    flash_mmu_invalidate_probe_t probe[2] = {0};
    ASSERT_TRUE(mem != NULL);
    ASSERT_TRUE(mmu != NULL);
    if (!mem || !mmu) {
        flexe_flash_mmu_destroy(mmu);
        mem_destroy(mem);
        return;
    }

    xtensa_cpu_init_for_target(&cpu[0], s3);
    xtensa_cpu_init_for_target(&cpu[1], s3);
    cpu[0].code_invalidate = flash_mmu_test_invalidate;
    cpu[0].code_invalidate_ctx = &probe[0];
    cpu[1].code_invalidate = flash_mmu_test_invalidate;
    cpu[1].code_invalidate_ctx = &probe[1];
    flexe_flash_mmu_attach_cpus(mmu, &cpu[0], &cpu[1]);

    mem_write32(mem, 0x600C5000u + 7u * 4u, 3u);
    ASSERT_EQ(probe[0].calls, 1u);
    ASSERT_EQ(probe[1].calls, 1u);
    ASSERT_EQ(probe[0].addr, 0x42070000u);
    ASSERT_EQ(probe[0].len, 0x10000u);

    /* Identical writes do not invalidate a translated page twice. */
    mem_write32(mem, 0x600C5000u + 7u * 4u, 3u);
    ASSERT_EQ(probe[0].calls, 1u);
    ASSERT_EQ(probe[1].calls, 1u);

    flexe_flash_mmu_destroy(mmu);
    mem_destroy(mem);
}

TEST(shared_flash_mmu_rejects_classic_split_tables) {
    xtensa_mem_t *mem = mem_create();
    ASSERT_TRUE(mem != NULL);
    ASSERT_TRUE(flexe_flash_mmu_create(mem) == NULL);
    /* Refusal must not alter the classic constructor map. */
    ASSERT_TRUE(mem_get_ptr(mem, 0x3F400000u) != NULL);
    mem_destroy(mem);
}

void run_flash_mmu_tests(void) {
    TEST_SUITE("Shared flash MMU");
    RUN_TEST(shared_flash_mmu_maps_flash_on_both_cache_buses);
    RUN_TEST(shared_flash_mmu_invalid_and_unbacked_entries_unmap);
    RUN_TEST(shared_flash_mmu_invalidates_each_independent_engine);
    RUN_TEST(shared_flash_mmu_rejects_classic_split_tables);
}
