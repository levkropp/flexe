/* ESP32-S3 SENSITIVE memory allocation and protection policy tests. */
#include "test_helpers.h"
#include "sensitive_memprot.h"
#include "target.h"

typedef struct {
    unsigned calls;
    flexe_sensitive_memory_usage_state_t state;
} memory_usage_probe_t;

typedef struct {
    unsigned reads;
    unsigned writes;
    uint32_t address;
    uint32_t value;
} sensitive_fallback_t;

static void memory_usage_changed(
    void *ctx, const flexe_sensitive_memory_usage_state_t *state)
{
    memory_usage_probe_t *probe = ctx;
    probe->calls++;
    probe->state = *state;
}

static uint32_t sensitive_fallback_read(void *ctx, uint32_t address)
{
    sensitive_fallback_t *fallback = ctx;
    fallback->reads++;
    fallback->address = address;
    return 0xA5A55A5Au;
}

static void sensitive_fallback_write(
    void *ctx, uint32_t address, uint32_t value)
{
    sensitive_fallback_t *fallback = ctx;
    fallback->writes++;
    fallback->address = address;
    fallback->value = value;
}

TEST(sensitive_memory_usage_has_exact_reset_masks_and_locks)
{
    const flexe_target_desc_t *s3 =
        flexe_target_by_id(FLEXE_TARGET_ESP32S3);
    uint32_t base = s3->sensitive_memprot.base;
    xtensa_mem_t *mem = mem_create_for_target(s3);
    flexe_sensitive_memprot_t *memprot =
        flexe_sensitive_memprot_create(mem, NULL, NULL, NULL);
    memory_usage_probe_t probe = {0};
    ASSERT_TRUE(mem != NULL);
    ASSERT_TRUE(memprot != NULL);
    if (!mem || !memprot) {
        flexe_sensitive_memprot_destroy(memprot);
        mem_destroy(mem);
        return;
    }

    ASSERT_EQ(mem_read32(mem, base + 0x000u), 0u);
    ASSERT_EQ(mem_read32(mem, base + 0x004u), 0xFFu);
    ASSERT_EQ(mem_read32(mem, base + 0x010u), 0u);
    ASSERT_EQ(mem_read32(mem, base + 0x014u), 0x7FFu);
    ASSERT_EQ(mem_read32(mem, base + 0x018u), 0u);
    ASSERT_EQ(mem_read32(mem, base + 0x01Cu), 0u);
    ASSERT_EQ(mem_read32(mem, base + 0x020u), 0u);
    ASSERT_EQ(mem_read32(mem, base + 0x024u), 0u);

    flexe_sensitive_memprot_set_memory_usage_listener(
        memprot, memory_usage_changed, &probe);
    ASSERT_EQ(probe.calls, 1u);
    ASSERT_FALSE(probe.state.cache_data_array_locked);
    ASSERT_EQ(probe.state.cache_data_array_connection, 0xFFu);
    ASSERT_FALSE(probe.state.internal_sram_usage_locked);
    ASSERT_EQ(probe.state.internal_sram_icache_usage, 3u);
    ASSERT_EQ(probe.state.internal_sram_dcache_usage, 3u);
    ASSERT_EQ(probe.state.internal_sram_cpu_usage, 0x7Fu);
    ASSERT_FALSE(probe.state.retention_disabled);

    mem_write32(mem, base + 0x004u, 0xFFFFFF20u);
    mem_write32(mem, base + 0x014u, 0xFFFFF129u);
    mem_write32(mem, base + 0x018u, 0xFF03A121u);
    mem_write32(mem, base + 0x01Cu, 0xFFFFFFFAu);
    mem_write32(mem, base + 0x020u, 0xFFFFFFD5u);
    mem_write32(mem, base + 0x024u, UINT32_MAX);
    ASSERT_EQ(probe.calls, 7u);
    ASSERT_EQ(mem_read32(mem, base + 0x004u), 0x20u);
    ASSERT_EQ(mem_read32(mem, base + 0x014u), 0x129u);
    ASSERT_EQ(mem_read32(mem, base + 0x018u), 0x3A121u);
    ASSERT_EQ(mem_read32(mem, base + 0x01Cu), 0xAu);
    ASSERT_EQ(mem_read32(mem, base + 0x020u), 0x55u);
    ASSERT_EQ(mem_read32(mem, base + 0x024u), 1u);
    ASSERT_EQ(probe.state.cache_data_array_connection, 0x20u);
    ASSERT_EQ(probe.state.internal_sram_icache_usage, 1u);
    ASSERT_EQ(probe.state.internal_sram_dcache_usage, 2u);
    ASSERT_EQ(probe.state.internal_sram_cpu_usage, 0x12u);
    ASSERT_EQ(probe.state.core0_trace_usage, 0x21u);
    ASSERT_EQ(probe.state.core1_trace_usage, 0x42u);
    ASSERT_EQ(probe.state.core0_trace_allocation, 2u);
    ASSERT_EQ(probe.state.core1_trace_allocation, 3u);
    ASSERT_EQ(probe.state.mac_dump_usage, 0xAu);
    ASSERT_EQ(probe.state.log_usage, 0x55u);
    ASSERT_TRUE(probe.state.retention_disabled);

    mem_write32(mem, base + 0x000u, 1u);
    mem_write32(mem, base + 0x010u, 1u);
    ASSERT_EQ(probe.calls, 9u);
    mem_write32(mem, base + 0x000u, 0u);
    mem_write32(mem, base + 0x010u, 0u);
    mem_write32(mem, base + 0x004u, 0xFFu);
    mem_write32(mem, base + 0x014u, 0x7FFu);
    mem_write32(mem, base + 0x018u, 0u);
    mem_write32(mem, base + 0x01Cu, 0u);
    mem_write32(mem, base + 0x020u, 0u);
    ASSERT_EQ(probe.calls, 9u);
    ASSERT_EQ(mem_read32(mem, base + 0x000u), 1u);
    ASSERT_EQ(mem_read32(mem, base + 0x004u), 0x20u);
    ASSERT_EQ(mem_read32(mem, base + 0x010u), 1u);
    ASSERT_EQ(mem_read32(mem, base + 0x014u), 0x129u);

    flexe_sensitive_memory_usage_state_t state = {0};
    ASSERT_TRUE(flexe_sensitive_memprot_memory_usage(memprot, &state));
    ASSERT_TRUE(state.cache_data_array_locked);
    ASSERT_TRUE(state.internal_sram_usage_locked);
    ASSERT_EQ(state.core1_trace_usage, 0x42u);

    flexe_sensitive_memprot_set_memory_usage_listener(memprot, NULL, NULL);
    mem_write32(mem, base + 0x024u, 0u);
    ASSERT_EQ(probe.calls, 9u);
    ASSERT_FALSE(flexe_sensitive_memprot_memory_usage(NULL, &state));
    ASSERT_FALSE(flexe_sensitive_memprot_memory_usage(memprot, NULL));

    flexe_sensitive_memprot_destroy(memprot);
    mem_destroy(mem);
}

TEST(sensitive_memory_usage_preserves_diagnostic_fallback)
{
    const flexe_target_desc_t *s3 =
        flexe_target_by_id(FLEXE_TARGET_ESP32S3);
    uint32_t base = s3->sensitive_memprot.base;
    xtensa_mem_t *mem = mem_create_for_target(s3);
    sensitive_fallback_t fallback = {0};
    flexe_sensitive_memprot_t *memprot = flexe_sensitive_memprot_create(
        mem, sensitive_fallback_read, sensitive_fallback_write, &fallback);
    ASSERT_TRUE(memprot != NULL);
    if (!memprot) {
        mem_destroy(mem);
        return;
    }

    /* APB access policy is deliberately outside this slice. */
    ASSERT_EQ(mem_read32(mem, base + 0x008u), 0xA5A55A5Au);
    ASSERT_EQ(fallback.reads, 1u);
    ASSERT_EQ(fallback.address, base + 0x008u);
    mem_write32(mem, base + 0x00Cu, 0x11223344u);
    ASSERT_EQ(fallback.writes, 1u);
    ASSERT_EQ(fallback.address, base + 0x00Cu);
    ASSERT_EQ(fallback.value, 0x11223344u);

    flexe_sensitive_memprot_destroy(memprot);
    ASSERT_EQ(mem_read32(mem, base + 0x004u), 0xA5A55A5Au);
    ASSERT_EQ(fallback.reads, 2u);
    mem_destroy(mem);
}

TEST(sensitive_memory_usage_rejects_incompatible_targets)
{
    const flexe_target_desc_t *s3 =
        flexe_target_by_id(FLEXE_TARGET_ESP32S3);
    flexe_target_desc_t invalid = *s3;
    invalid.capabilities &= ~FLEXE_TARGET_CAP_SENSITIVE_MEMPROT_V1;
    xtensa_mem_t *mem = mem_create_for_target(&invalid);
    ASSERT_TRUE(mem != NULL);
    ASSERT_TRUE(flexe_sensitive_memprot_create(mem, NULL, NULL, NULL) == NULL);
    mem_destroy(mem);

    invalid = *s3;
    invalid.sensitive_memprot.register_size = 0x100u;
    mem = mem_create_for_target(&invalid);
    ASSERT_TRUE(mem != NULL);
    ASSERT_TRUE(flexe_sensitive_memprot_create(mem, NULL, NULL, NULL) == NULL);
    mem_destroy(mem);
}

void run_sensitive_memprot_tests(void)
{
    TEST_SUITE("S3 SENSITIVE memory policy");
    RUN_TEST(sensitive_memory_usage_has_exact_reset_masks_and_locks);
    RUN_TEST(sensitive_memory_usage_preserves_diagnostic_fallback);
    RUN_TEST(sensitive_memory_usage_rejects_incompatible_targets);
}
