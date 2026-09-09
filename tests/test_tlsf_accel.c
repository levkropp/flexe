/* Tests for the transactional ESP-IDF 4.4 TLSF guest-memory translator. */
#include "test_helpers.h"
#include "tlsf_accel.h"

#include <string.h>

enum {
    TLSF_TEST_HEAP = 0x3FFB1000u,
    TLSF_TEST_CONTROL = TLSF_TEST_HEAP + 20u,
    TLSF_TEST_METADATA_SIZE = 360u,
    TLSF_TEST_POOL = TLSF_TEST_CONTROL + TLSF_TEST_METADATA_SIZE,
    TLSF_TEST_BLOCK = TLSF_TEST_POOL - 4u,
    TLSF_TEST_BLOCK_SIZE = 4096u,
    TLSF_TEST_SENTINEL = TLSF_TEST_POOL + TLSF_TEST_BLOCK_SIZE,
    TLSF_TEST_FREE_BYTES = TLSF_TEST_BLOCK_SIZE + 8u,
    TLSF_TEST_POOL_SIZE = TLSF_TEST_METADATA_SIZE + TLSF_TEST_FREE_BYTES,
    TLSF_TEST_SL_BITMAP = TLSF_TEST_CONTROL + 36u,
    TLSF_TEST_BLOCKS = TLSF_TEST_SL_BITMAP + 9u * 4u,
};

static void tlsf_test_init_heap(xtensa_mem_t *mem) {
    /* multi_heap heap_t */
    mem_write32(mem, TLSF_TEST_HEAP + 0u, 0u);
    mem_write32(mem, TLSF_TEST_HEAP + 4u, TLSF_TEST_FREE_BYTES);
    mem_write32(mem, TLSF_TEST_HEAP + 8u, TLSF_TEST_FREE_BYTES);
    mem_write32(mem, TLSF_TEST_HEAP + 12u, TLSF_TEST_POOL_SIZE);
    mem_write32(mem, TLSF_TEST_HEAP + 16u, TLSF_TEST_CONTROL);

    /* ESP-IDF 4.4's dynamic control_t for a 4 KiB initial free block:
     * FL count/shift/max = 9/5/13 and SL count/log2 = 8/3. */
    mem_write32(mem, TLSF_TEST_CONTROL + 8u, TLSF_TEST_CONTROL);
    mem_write32(mem, TLSF_TEST_CONTROL + 12u, TLSF_TEST_BLOCK);
    mem_write32(mem, TLSF_TEST_CONTROL + 16u, 0x10320DA9u);
    mem_write32(mem, TLSF_TEST_CONTROL + 20u, TLSF_TEST_METADATA_SIZE);
    mem_write32(mem, TLSF_TEST_CONTROL + 24u, 1u << 8);
    mem_write32(mem, TLSF_TEST_CONTROL + 28u, TLSF_TEST_SL_BITMAP);
    mem_write32(mem, TLSF_TEST_CONTROL + 32u, TLSF_TEST_BLOCKS);

    for (unsigned fl = 0; fl < 9u; fl++)
        mem_write32(mem, TLSF_TEST_SL_BITMAP + fl * 4u,
                    fl == 8u ? 1u : 0u);
    for (unsigned i = 0; i < 9u * 8u; i++)
        mem_write32(mem, TLSF_TEST_BLOCKS + i * 4u, TLSF_TEST_CONTROL);
    mem_write32(mem, TLSF_TEST_BLOCKS + 64u * 4u, TLSF_TEST_BLOCK);

    /* tlsf_add_pool() places the first block's unused prev_phys word four
     * bytes before the pool and a zero-sized sentinel after the block. */
    mem_write32(mem, TLSF_TEST_BLOCK + 4u, TLSF_TEST_BLOCK_SIZE | 1u);
    mem_write32(mem, TLSF_TEST_BLOCK + 8u, TLSF_TEST_CONTROL);
    mem_write32(mem, TLSF_TEST_BLOCK + 12u, TLSF_TEST_CONTROL);
    mem_write32(mem, TLSF_TEST_SENTINEL + 0u, TLSF_TEST_BLOCK);
    mem_write32(mem, TLSF_TEST_SENTINEL + 4u, 2u);
}

TEST(tlsf_accel_malloc_is_transactional_and_exact) {
    xtensa_mem_t *mem = mem_create();
    tlsf_test_init_heap(mem);
    tlsf_accel_plan_t plan;

    ASSERT_TRUE(tlsf_accel_plan_idf44_malloc(
            mem, TLSF_TEST_HEAP, 99u, &plan));
    ASSERT_EQ(plan.result, TLSF_TEST_POOL + 4u);
    ASSERT_TRUE(plan.write_count > 0u);
    ASSERT_TRUE(plan.write_count <= TLSF_ACCEL_MAX_WRITES);

    /* Planning is side-effect free. */
    ASSERT_EQ(mem_read32(mem, TLSF_TEST_HEAP + 4u), TLSF_TEST_FREE_BYTES);
    ASSERT_EQ(mem_read32(mem, TLSF_TEST_BLOCK + 4u),
              TLSF_TEST_BLOCK_SIZE | 1u);
    ASSERT_FALSE(tlsf_accel_plan_matches(&plan));

    tlsf_accel_plan_commit(&plan);
    const uint32_t remaining = TLSF_TEST_BLOCK + 104u;
    ASSERT_TRUE(tlsf_accel_plan_matches(&plan));
    ASSERT_EQ(mem_read32(mem, TLSF_TEST_HEAP + 4u), 4000u);
    ASSERT_EQ(mem_read32(mem, TLSF_TEST_HEAP + 8u), 4000u);
    ASSERT_EQ(mem_read32(mem, TLSF_TEST_BLOCK + 4u), 100u);
    ASSERT_EQ(mem_read32(mem, remaining + 0u), TLSF_TEST_BLOCK);
    ASSERT_EQ(mem_read32(mem, remaining + 4u), 3992u | 1u);
    ASSERT_EQ(mem_read32(mem, remaining + 8u), TLSF_TEST_CONTROL);
    ASSERT_EQ(mem_read32(mem, remaining + 12u), TLSF_TEST_CONTROL);
    ASSERT_EQ(mem_read32(mem, TLSF_TEST_SENTINEL + 0u), remaining);
    ASSERT_EQ(mem_read32(mem, TLSF_TEST_SENTINEL + 4u), 2u);
    ASSERT_EQ(mem_read32(mem, TLSF_TEST_CONTROL + 24u), 1u << 7);
    ASSERT_EQ(mem_read32(mem, TLSF_TEST_SL_BITMAP + 7u * 4u), 1u << 7);
    ASSERT_EQ(mem_read32(mem, TLSF_TEST_SL_BITMAP + 8u * 4u), 0u);
    ASSERT_EQ(mem_read32(mem, TLSF_TEST_BLOCKS + 63u * 4u), remaining);
    ASSERT_EQ(mem_read32(mem, TLSF_TEST_BLOCKS + 64u * 4u),
              TLSF_TEST_CONTROL);

    mem_destroy(mem);
}

TEST(tlsf_accel_free_coalesces_and_preserves_low_watermark) {
    xtensa_mem_t *mem = mem_create();
    tlsf_test_init_heap(mem);
    tlsf_accel_plan_t plan;

    ASSERT_TRUE(tlsf_accel_plan_idf44_malloc(
            mem, TLSF_TEST_HEAP, 100u, &plan));
    uint32_t ptr = plan.result;
    tlsf_accel_plan_commit(&plan);
    ASSERT_TRUE(tlsf_accel_plan_idf44_free(
            mem, TLSF_TEST_HEAP, ptr, &plan));
    ASSERT_EQ(plan.result, 0u);
    tlsf_accel_plan_commit(&plan);

    ASSERT_EQ(mem_read32(mem, TLSF_TEST_HEAP + 4u), TLSF_TEST_FREE_BYTES);
    ASSERT_EQ(mem_read32(mem, TLSF_TEST_HEAP + 8u), 4000u);
    ASSERT_EQ(mem_read32(mem, TLSF_TEST_BLOCK + 4u),
              TLSF_TEST_BLOCK_SIZE | 1u);
    ASSERT_EQ(mem_read32(mem, TLSF_TEST_BLOCK + 8u), TLSF_TEST_CONTROL);
    ASSERT_EQ(mem_read32(mem, TLSF_TEST_BLOCK + 12u), TLSF_TEST_CONTROL);
    ASSERT_EQ(mem_read32(mem, TLSF_TEST_SENTINEL + 0u), TLSF_TEST_BLOCK);
    ASSERT_EQ(mem_read32(mem, TLSF_TEST_SENTINEL + 4u), 2u);
    ASSERT_EQ(mem_read32(mem, TLSF_TEST_CONTROL + 24u), 1u << 8);
    ASSERT_EQ(mem_read32(mem, TLSF_TEST_SL_BITMAP + 8u * 4u), 1u);
    ASSERT_EQ(mem_read32(mem, TLSF_TEST_BLOCKS + 64u * 4u),
              TLSF_TEST_BLOCK);

    mem_destroy(mem);
}

TEST(tlsf_accel_free_merges_both_neighbors) {
    xtensa_mem_t *mem = mem_create();
    tlsf_test_init_heap(mem);
    tlsf_accel_plan_t plan;
    uint32_t ptr[3];

    for (unsigned i = 0; i < 3u; i++) {
        ASSERT_TRUE(tlsf_accel_plan_idf44_malloc(
                mem, TLSF_TEST_HEAP, 100u, &plan));
        ptr[i] = plan.result;
        tlsf_accel_plan_commit(&plan);
    }
    ASSERT_TRUE(tlsf_accel_plan_idf44_free(
            mem, TLSF_TEST_HEAP, ptr[1], &plan));
    tlsf_accel_plan_commit(&plan);
    ASSERT_TRUE(tlsf_accel_plan_idf44_free(
            mem, TLSF_TEST_HEAP, ptr[2], &plan));
    ASSERT_TRUE(plan.write_count <= TLSF_ACCEL_MAX_WRITES);
    tlsf_accel_plan_commit(&plan);

    /* The third block absorbs both the free middle block and the trailing
     * remainder, leaving one free extent after the first allocation. */
    const uint32_t merged = ptr[1] - 8u;
    ASSERT_EQ(mem_read32(mem, merged + 4u), 3992u | 1u);
    ASSERT_EQ(mem_read32(mem, merged + 8u), TLSF_TEST_CONTROL);
    ASSERT_EQ(mem_read32(mem, merged + 12u), TLSF_TEST_CONTROL);
    ASSERT_EQ(mem_read32(mem, TLSF_TEST_SENTINEL + 0u), merged);
    ASSERT_EQ(mem_read32(mem, TLSF_TEST_HEAP + 4u), 4000u);

    ASSERT_TRUE(tlsf_accel_plan_idf44_free(
            mem, TLSF_TEST_HEAP, ptr[0], &plan));
    tlsf_accel_plan_commit(&plan);
    ASSERT_EQ(mem_read32(mem, TLSF_TEST_BLOCK + 4u),
              TLSF_TEST_BLOCK_SIZE | 1u);
    ASSERT_EQ(mem_read32(mem, TLSF_TEST_HEAP + 4u), TLSF_TEST_FREE_BYTES);

    mem_destroy(mem);
}

TEST(tlsf_accel_rejects_corruption_without_partial_writes) {
    xtensa_mem_t *mem = mem_create();
    tlsf_test_init_heap(mem);
    tlsf_accel_plan_t plan;
    enum { SNAPSHOT_WORDS = (TLSF_TEST_SENTINEL + 8u - TLSF_TEST_HEAP) / 4u };
    uint32_t before[SNAPSHOT_WORDS];

    mem_write32(mem, TLSF_TEST_BLOCK + 8u, 0x10000000u);
    for (unsigned i = 0; i < SNAPSHOT_WORDS; i++)
        before[i] = mem_read32(mem, TLSF_TEST_HEAP + i * 4u);

    ASSERT_FALSE(tlsf_accel_plan_idf44_malloc(
            mem, TLSF_TEST_HEAP, 100u, &plan));
    bool unchanged = true;
    for (unsigned i = 0; i < SNAPSHOT_WORDS; i++) {
        if (mem_read32(mem, TLSF_TEST_HEAP + i * 4u) != before[i]) {
            unchanged = false;
            break;
        }
    }
    ASSERT_TRUE(unchanged);

    mem_destroy(mem);
}

TEST(tlsf_accel_rejects_malformed_layout_and_out_of_pool_free) {
    xtensa_mem_t *mem = mem_create();
    tlsf_test_init_heap(mem);
    tlsf_accel_plan_t plan;

    mem_write32(mem, TLSF_TEST_CONTROL + 20u,
                TLSF_TEST_METADATA_SIZE + 4u);
    ASSERT_FALSE(tlsf_accel_plan_idf44_malloc(
            mem, TLSF_TEST_HEAP, 100u, &plan));
    mem_write32(mem, TLSF_TEST_CONTROL + 20u, TLSF_TEST_METADATA_SIZE);

    ASSERT_FALSE(tlsf_accel_plan_idf44_free(
            mem, TLSF_TEST_HEAP, TLSF_TEST_HEAP + 8u, &plan));
    ASSERT_FALSE(tlsf_accel_plan_idf44_free(
            mem, TLSF_TEST_HEAP,
            TLSF_TEST_POOL + TLSF_TEST_POOL_SIZE, &plan));

    mem_destroy(mem);
}

TEST(tlsf_accel_models_exhaustion_without_writes) {
    xtensa_mem_t *mem = mem_create();
    tlsf_test_init_heap(mem);
    tlsf_accel_plan_t plan;

    mem_write32(mem, TLSF_TEST_CONTROL + 24u, 0u);
    mem_write32(mem, TLSF_TEST_SL_BITMAP + 8u * 4u, 0u);
    mem_write32(mem, TLSF_TEST_BLOCKS + 64u * 4u, TLSF_TEST_CONTROL);

    ASSERT_TRUE(tlsf_accel_plan_idf44_malloc(
            mem, TLSF_TEST_HEAP, 100u, &plan));
    ASSERT_EQ(plan.result, 0u);
    ASSERT_EQ(plan.write_count, 0u);
    ASSERT_TRUE(tlsf_accel_plan_matches(&plan));
    ASSERT_EQ(mem_read32(mem, TLSF_TEST_HEAP + 4u), TLSF_TEST_FREE_BYTES);

    ASSERT_TRUE(tlsf_accel_plan_idf44_malloc(
            mem, TLSF_TEST_HEAP, 1u << 13, &plan));
    ASSERT_EQ(plan.result, 0u);
    ASSERT_EQ(plan.write_count, 0u);

    mem_destroy(mem);
}

TEST(tlsf_accel_raw_operations_leave_multi_heap_counters_to_caller) {
    xtensa_mem_t *mem = mem_create();
    tlsf_test_init_heap(mem);
    tlsf_accel_plan_t plan;

    ASSERT_TRUE(tlsf_accel_plan_idf44_raw_malloc(
            mem, TLSF_TEST_CONTROL, 100u, &plan));
    uint32_t ptr = plan.result;
    tlsf_accel_plan_commit(&plan);
    ASSERT_EQ(ptr, TLSF_TEST_POOL + 4u);
    ASSERT_EQ(mem_read32(mem, TLSF_TEST_HEAP + 4u), TLSF_TEST_FREE_BYTES);

    ASSERT_TRUE(tlsf_accel_plan_idf44_raw_free(
            mem, TLSF_TEST_CONTROL, ptr, &plan));
    tlsf_accel_plan_commit(&plan);
    ASSERT_EQ(mem_read32(mem, TLSF_TEST_BLOCK + 4u),
              TLSF_TEST_BLOCK_SIZE | 1u);
    ASSERT_EQ(mem_read32(mem, TLSF_TEST_HEAP + 4u), TLSF_TEST_FREE_BYTES);

    mem_destroy(mem);
}

static void run_tlsf_accel_tests(void) {
    TEST_SUITE("TLSF accelerator");
    RUN_TEST(tlsf_accel_malloc_is_transactional_and_exact);
    RUN_TEST(tlsf_accel_free_coalesces_and_preserves_low_watermark);
    RUN_TEST(tlsf_accel_free_merges_both_neighbors);
    RUN_TEST(tlsf_accel_rejects_corruption_without_partial_writes);
    RUN_TEST(tlsf_accel_rejects_malformed_layout_and_out_of_pool_free);
    RUN_TEST(tlsf_accel_models_exhaustion_without_writes);
    RUN_TEST(tlsf_accel_raw_operations_leave_multi_heap_counters_to_caller);
}
