/* Target-described S3 SYSCON memory clock and power policy tests. */
#include "test_helpers.h"
#include "peripherals.h"
#include "syscon_memory.h"
#include "target.h"

typedef struct {
    unsigned reads;
    unsigned writes;
    uint32_t address;
    uint32_t value;
} syscon_fallback_t;

typedef struct {
    unsigned calls;
    flexe_syscon_memory_state_t state;
} syscon_state_probe_t;

static uint32_t syscon_fallback_read(void *ctx, uint32_t address)
{
    syscon_fallback_t *fallback = ctx;
    fallback->reads++;
    fallback->address = address;
    return 0xA5A55A5Au;
}

static void syscon_fallback_write(
    void *ctx, uint32_t address, uint32_t value)
{
    syscon_fallback_t *fallback = ctx;
    fallback->writes++;
    fallback->address = address;
    fallback->value = value;
}

static void syscon_state_changed(
    void *ctx, const flexe_syscon_memory_state_t *state)
{
    syscon_state_probe_t *probe = ctx;
    probe->calls++;
    probe->state = *state;
}

TEST(syscon_memory_exposes_s3_reset_and_masked_readback)
{
    const flexe_target_desc_t *s3 =
        flexe_target_by_id(FLEXE_TARGET_ESP32S3);
    const flexe_syscon_memory_desc_t *desc = &s3->syscon_memory;
    xtensa_mem_t *mem = mem_create_for_target(s3);
    esp32_periph_t *periph = periph_create(mem);
    ASSERT_TRUE(mem != NULL);
    ASSERT_TRUE(periph != NULL);
    if (!mem || !periph) {
        periph_destroy(periph);
        mem_destroy(mem);
        return;
    }

    ASSERT_TRUE(s3->capabilities & FLEXE_TARGET_CAP_SYSCON_MEMORY_V1);
    ASSERT_EQ(mem_read32(mem, desc->base + desc->front_end_power_offset),
              desc->front_end_power_reset);
    ASSERT_EQ(mem_read32(mem, desc->base + desc->clock_force_on_offset),
              desc->clock_force_on_reset);
    ASSERT_EQ(mem_read32(mem, desc->base + desc->power_down_offset),
              desc->power_down_reset);
    ASSERT_EQ(mem_read32(mem, desc->base + desc->power_up_offset),
              desc->power_up_reset);

    mem_write32(mem, desc->base + desc->front_end_power_offset,
                UINT32_MAX);
    mem_write32(mem, desc->base + desc->clock_force_on_offset,
                UINT32_MAX);
    mem_write32(mem, desc->base + desc->power_down_offset,
                UINT32_MAX);
    mem_write32(mem, desc->base + desc->power_up_offset,
                UINT32_MAX);
    ASSERT_EQ(mem_read32(mem, desc->base + desc->front_end_power_offset),
              desc->front_end_force_power_down_mask |
              desc->front_end_force_power_up_mask);
    ASSERT_EQ(mem_read32(mem, desc->base + desc->clock_force_on_offset),
              desc->sram_bank_mask | desc->rom_bank_mask);
    ASSERT_EQ(mem_read32(mem, desc->base + desc->power_down_offset),
              desc->sram_bank_mask | desc->rom_bank_mask);
    ASSERT_EQ(mem_read32(mem, desc->base + desc->power_up_offset),
              desc->sram_bank_mask | desc->rom_bank_mask);
    ASSERT_EQ(periph_unhandled_count(periph), 0u);

    /* The radio controller shares the page through the explicit fallback
     * chain; its existing controls remain independent. */
    uint32_t radio_clock = s3->radio.control.base +
                           s3->radio.control.clock_offset;
    mem_write32(mem, radio_clock, 0x12345678u);
    ASSERT_EQ(mem_read32(mem, radio_clock), 0x12345678u);
    ASSERT_EQ(periph_unhandled_count(periph), 0u);

    periph_destroy(periph);
    mem_destroy(mem);
}

TEST(syscon_memory_publishes_normalized_bank_policy)
{
    const flexe_target_desc_t *s3 =
        flexe_target_by_id(FLEXE_TARGET_ESP32S3);
    const flexe_syscon_memory_desc_t *desc = &s3->syscon_memory;
    xtensa_mem_t *mem = mem_create_for_target(s3);
    flexe_syscon_memory_t *memory = flexe_syscon_memory_create(
        mem, NULL, NULL, NULL);
    syscon_state_probe_t probe = {0};
    ASSERT_TRUE(memory != NULL);
    if (!memory) {
        mem_destroy(mem);
        return;
    }

    flexe_syscon_memory_set_state_listener(
        memory, syscon_state_changed, &probe);
    ASSERT_EQ(probe.calls, 1u);
    ASSERT_EQ(probe.state.front_end_force_power_down, 0u);
    ASSERT_EQ(probe.state.front_end_force_power_up, 0xFu);
    ASSERT_EQ(probe.state.sram_clock_force_on, 0x7FFu);
    ASSERT_EQ(probe.state.rom_clock_force_on, 0x7u);
    ASSERT_EQ(probe.state.sram_force_power_down, 0u);
    ASSERT_EQ(probe.state.rom_force_power_down, 0u);
    ASSERT_EQ(probe.state.sram_force_power_up, 0x7FFu);
    ASSERT_EQ(probe.state.rom_force_power_up, 0x7u);

    mem_write32(mem, desc->base + desc->front_end_power_offset, 0xAAu);
    mem_write32(mem, desc->base + desc->clock_force_on_offset, 0x0AADu);
    mem_write32(mem, desc->base + desc->power_down_offset, 0x001Au);
    mem_write32(mem, desc->base + desc->power_up_offset, 0x2001u);
    ASSERT_EQ(probe.calls, 5u);
    ASSERT_EQ(probe.state.front_end_force_power_down, 0xFu);
    ASSERT_EQ(probe.state.front_end_force_power_up, 0u);
    ASSERT_EQ(probe.state.sram_clock_force_on, 0x155u);
    ASSERT_EQ(probe.state.rom_clock_force_on, 0x5u);
    ASSERT_EQ(probe.state.sram_force_power_down, 0x3u);
    ASSERT_EQ(probe.state.rom_force_power_down, 0x2u);
    ASSERT_EQ(probe.state.sram_force_power_up, 0x400u);
    ASSERT_EQ(probe.state.rom_force_power_up, 0x1u);

    flexe_syscon_memory_state_t queried = {0};
    ASSERT_TRUE(flexe_syscon_memory_state(memory, &queried));
    ASSERT_EQ(queried.sram_clock_force_on, 0x155u);
    flexe_syscon_memory_set_state_listener(memory, NULL, NULL);
    mem_write32(mem, desc->base + desc->power_up_offset, 0u);
    ASSERT_EQ(probe.calls, 5u);

    flexe_syscon_memory_destroy(memory);
    mem_destroy(mem);
}

TEST(syscon_memory_delegates_unknown_words)
{
    const flexe_target_desc_t *s3 =
        flexe_target_by_id(FLEXE_TARGET_ESP32S3);
    xtensa_mem_t *mem = mem_create_for_target(s3);
    syscon_fallback_t fallback = {0};
    flexe_syscon_memory_t *memory = flexe_syscon_memory_create(
        mem, syscon_fallback_read, syscon_fallback_write, &fallback);
    ASSERT_TRUE(memory != NULL);
    if (!memory) {
        mem_destroy(mem);
        return;
    }

    uint32_t unknown = s3->syscon_memory.base + 0x0A4u;
    ASSERT_EQ(mem_read32(mem, unknown), 0xA5A55A5Au);
    ASSERT_EQ(fallback.reads, 1u);
    ASSERT_EQ(fallback.address, unknown);
    mem_write32(mem, unknown, 0x11223344u);
    ASSERT_EQ(fallback.writes, 1u);
    ASSERT_EQ(fallback.address, unknown);
    ASSERT_EQ(fallback.value, 0x11223344u);

    flexe_syscon_memory_destroy(memory);
    mem_destroy(mem);
}

TEST(syscon_memory_rejects_invalid_descriptors)
{
    const flexe_target_desc_t *s3 =
        flexe_target_by_id(FLEXE_TARGET_ESP32S3);
    flexe_target_desc_t invalid = *s3;
    invalid.capabilities &= ~FLEXE_TARGET_CAP_SYSCON_MEMORY_V1;
    xtensa_mem_t *mem = mem_create_for_target(&invalid);
    ASSERT_TRUE(mem != NULL);
    flexe_syscon_memory_t *memory = flexe_syscon_memory_create(
        mem, NULL, NULL, NULL);
    ASSERT_TRUE(memory == NULL);
    mem_destroy(mem);

    invalid = *s3;
    invalid.syscon_memory.power_up_offset =
        invalid.syscon_memory.power_down_offset;
    mem = mem_create_for_target(&invalid);
    ASSERT_TRUE(mem != NULL);
    memory = flexe_syscon_memory_create(mem, NULL, NULL, NULL);
    ASSERT_TRUE(memory == NULL);
    mem_destroy(mem);

    invalid = *s3;
    invalid.syscon_memory.rom_bank_mask =
        invalid.syscon_memory.sram_bank_mask;
    mem = mem_create_for_target(&invalid);
    ASSERT_TRUE(mem != NULL);
    memory = flexe_syscon_memory_create(mem, NULL, NULL, NULL);
    ASSERT_TRUE(memory == NULL);
    mem_destroy(mem);

    invalid = *s3;
    invalid.syscon_memory.front_end_power_reset = UINT32_MAX;
    mem = mem_create_for_target(&invalid);
    ASSERT_TRUE(mem != NULL);
    memory = flexe_syscon_memory_create(mem, NULL, NULL, NULL);
    ASSERT_TRUE(memory == NULL);
    mem_destroy(mem);
}

void run_syscon_memory_tests(void)
{
    TEST_SUITE("Target SYSCON memory policy");
    RUN_TEST(syscon_memory_exposes_s3_reset_and_masked_readback);
    RUN_TEST(syscon_memory_publishes_normalized_bank_policy);
    RUN_TEST(syscon_memory_delegates_unknown_words);
    RUN_TEST(syscon_memory_rejects_invalid_descriptors);
}
