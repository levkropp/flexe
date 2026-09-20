/* ESP32-S3 per-core ASSIST_DEBUG recorder tests. */
#include "test_helpers.h"

#include "assist_debug.h"
#include "peripherals.h"
#include "target.h"

typedef struct {
    unsigned reads;
    unsigned writes;
    uint32_t address;
    uint32_t value;
} assist_debug_fallback_t;

static uint32_t assist_debug_fallback_read(void *ctx, uint32_t address)
{
    assist_debug_fallback_t *fallback = ctx;
    fallback->reads++;
    fallback->address = address;
    return 0xA55A3CC3u;
}

static void assist_debug_fallback_write(
    void *ctx, uint32_t address, uint32_t value)
{
    assist_debug_fallback_t *fallback = ctx;
    fallback->writes++;
    fallback->address = address;
    fallback->value = value;
}

TEST(assist_debug_records_live_and_frozen_dual_core_state)
{
    const flexe_target_desc_t *s3 =
        flexe_target_by_id(FLEXE_TARGET_ESP32S3);
    const flexe_assist_debug_desc_t *desc = &s3->assist_debug;
    xtensa_mem_t *mem = mem_create_for_target(s3);
    flexe_assist_debug_t *debug =
        flexe_assist_debug_create(mem, NULL, NULL, NULL);
    xtensa_cpu_t cpu[2];
    ASSERT_TRUE(mem != NULL);
    ASSERT_TRUE(debug != NULL);
    if (!mem || !debug) {
        flexe_assist_debug_destroy(debug);
        mem_destroy(mem);
        return;
    }

    for (unsigned core = 0u; core < 2u; core++) {
        xtensa_cpu_init_for_target(&cpu[core], s3);
        cpu[core].mem = mem;
    }
    flexe_assist_debug_attach_cpus(debug, &cpu[0], &cpu[1]);

    ASSERT_EQ(mem_read32(mem, desc->base + desc->date_offset),
              desc->date_reset);
    for (unsigned core = 0u; core < 2u; core++) {
        uint32_t base = desc->base + core * desc->core_stride;
        ASSERT_EQ(mem_read32(mem, base + desc->pdebug_enable_offset), 0u);
        ASSERT_EQ(mem_read32(mem, base + desc->recording_offset), 0u);
        ASSERT_EQ(mem_read32(mem, base + desc->pc_offset), 0u);
        ASSERT_EQ(mem_read32(mem, base + desc->sp_offset), 0u);

        cpu[core].pc = 0x40370000u + core * 0x1000u;
        ar_write(&cpu[core], 1, 0x3FCEB000u + core * 0x2000u);
        mem_write32(mem, base + desc->pdebug_enable_offset, UINT32_MAX);
        mem_write32(mem, base + desc->recording_offset, UINT32_MAX);
        ASSERT_EQ(mem_read32(mem, base + desc->pdebug_enable_offset),
                  desc->pdebug_enable_mask);
        ASSERT_EQ(mem_read32(mem, base + desc->recording_offset),
                  desc->recording_mask);
        ASSERT_EQ(mem_read32(mem, base + desc->pc_offset), cpu[core].pc);
        ASSERT_EQ(mem_read32(mem, base + desc->sp_offset),
                  ar_read(&cpu[core], 1));
    }

    cpu[0].pc = 0x42012345u;
    ar_write(&cpu[0], 1, 0x3FCEA100u);
    ASSERT_EQ(mem_read32(mem, desc->base + desc->pc_offset), 0x42012345u);
    ASSERT_EQ(mem_read32(mem, desc->base + desc->sp_offset), 0x3FCEA100u);

    /* RECORDING=0 freezes the latest processor-debug sample. */
    uint32_t core1_base = desc->base + desc->core_stride;
    cpu[1].pc = 0x42056789u;
    ar_write(&cpu[1], 1, 0x3FCEC100u);
    mem_write32(mem, core1_base + desc->recording_offset, 0u);
    cpu[1].pc = 0x420AAAAAu;
    ar_write(&cpu[1], 1, 0x3FCEEEEEu);
    ASSERT_EQ(mem_read32(mem, core1_base + desc->pc_offset), 0x42056789u);
    ASSERT_EQ(mem_read32(mem, core1_base + desc->sp_offset), 0x3FCEC100u);

    flexe_assist_debug_core_state_t state = {0};
    ASSERT_TRUE(flexe_assist_debug_core_state(debug, 1u, &state));
    ASSERT_TRUE(state.pdebug_enabled);
    ASSERT_FALSE(state.recording);
    ASSERT_EQ(state.pc, 0x42056789u);
    ASSERT_EQ(state.sp, 0x3FCEC100u);

    /* Re-enabling recording immediately resumes the live feed. */
    mem_write32(mem, core1_base + desc->recording_offset,
                desc->recording_mask);
    ASSERT_EQ(mem_read32(mem, core1_base + desc->pc_offset), 0x420AAAAAu);
    ASSERT_EQ(mem_read32(mem, core1_base + desc->sp_offset), 0x3FCEEEEEu);

    /* PDEBUGENABLE gates and freezes the same feed independently. */
    mem_write32(mem, desc->base + desc->pdebug_enable_offset, 0u);
    cpu[0].pc = 0x420BBBBBu;
    ar_write(&cpu[0], 1, 0x3FCEDDDDu);
    ASSERT_EQ(mem_read32(mem, desc->base + desc->pc_offset), 0x42012345u);
    ASSERT_EQ(mem_read32(mem, desc->base + desc->sp_offset), 0x3FCEA100u);

    /* Record fields are read-only and DATE masks its reserved high bits. */
    mem_write32(mem, desc->base + desc->pc_offset, 0x12345678u);
    ASSERT_EQ(mem_read32(mem, desc->base + desc->pc_offset), 0x42012345u);
    mem_write32(mem, desc->base + desc->date_offset, UINT32_MAX);
    ASSERT_EQ(mem_read32(mem, desc->base + desc->date_offset),
              desc->date_writable_mask);

    ASSERT_FALSE(flexe_assist_debug_core_state(debug, 2u, &state));
    ASSERT_FALSE(flexe_assist_debug_core_state(NULL, 0u, &state));
    ASSERT_FALSE(flexe_assist_debug_core_state(debug, 0u, NULL));

    flexe_assist_debug_destroy(debug);
    mem_destroy(mem);
}

TEST(assist_debug_preserves_unmodeled_register_diagnostics)
{
    const flexe_target_desc_t *s3 =
        flexe_target_by_id(FLEXE_TARGET_ESP32S3);
    uint32_t base = s3->assist_debug.base;
    xtensa_mem_t *mem = mem_create_for_target(s3);
    assist_debug_fallback_t fallback = {0};
    flexe_assist_debug_t *debug = flexe_assist_debug_create(
        mem, assist_debug_fallback_read, assist_debug_fallback_write,
        &fallback);
    ASSERT_TRUE(debug != NULL);
    if (!debug) {
        mem_destroy(mem);
        return;
    }

    /* Area watchpoints and the detailed instruction/load-store record are
     * deliberately not claimed by the PC/SP recorder slice. */
    ASSERT_EQ(mem_read32(mem, base + 0x010u), 0xA55A3CC3u);
    ASSERT_EQ(mem_read32(mem, base + 0x050u), 0xA55A3CC3u);
    ASSERT_EQ(fallback.reads, 2u);
    ASSERT_EQ(fallback.address, base + 0x050u);
    mem_write32(mem, base + 0x010u, 0x11223344u);
    ASSERT_EQ(fallback.writes, 1u);
    ASSERT_EQ(fallback.address, base + 0x010u);
    ASSERT_EQ(fallback.value, 0x11223344u);

    flexe_assist_debug_destroy(debug);
    ASSERT_EQ(mem_read32(mem, base + 0x048u), 0xA55A3CC3u);
    ASSERT_EQ(fallback.reads, 3u);
    mem_destroy(mem);
}

TEST(assist_debug_rejects_incompatible_geometry)
{
    const flexe_target_desc_t *s3 =
        flexe_target_by_id(FLEXE_TARGET_ESP32S3);
    flexe_target_desc_t invalid = *s3;
    invalid.capabilities &= ~FLEXE_TARGET_CAP_ASSIST_DEBUG_V1;
    xtensa_mem_t *mem = mem_create_for_target(&invalid);
    ASSERT_TRUE(mem != NULL);
    ASSERT_TRUE(flexe_assist_debug_create(mem, NULL, NULL, NULL) == NULL);
    mem_destroy(mem);

    invalid = *s3;
    invalid.assist_debug.core_stride = 0x6Cu;
    mem = mem_create_for_target(&invalid);
    ASSERT_TRUE(mem != NULL);
    ASSERT_TRUE(flexe_assist_debug_create(mem, NULL, NULL, NULL) == NULL);
    mem_destroy(mem);

    invalid = *s3;
    invalid.assist_debug.pc_offset = invalid.assist_debug.sp_offset;
    mem = mem_create_for_target(&invalid);
    ASSERT_TRUE(mem != NULL);
    ASSERT_TRUE(flexe_assist_debug_create(mem, NULL, NULL, NULL) == NULL);
    mem_destroy(mem);

    invalid = *s3;
    invalid.assist_debug.date_offset = 0x0FCu;
    mem = mem_create_for_target(&invalid);
    ASSERT_TRUE(mem != NULL);
    ASSERT_TRUE(flexe_assist_debug_create(mem, NULL, NULL, NULL) == NULL);
    mem_destroy(mem);
}

TEST(assist_debug_is_composed_into_the_s3_machine)
{
    const flexe_target_desc_t *s3 =
        flexe_target_by_id(FLEXE_TARGET_ESP32S3);
    const flexe_assist_debug_desc_t *desc = &s3->assist_debug;
    xtensa_mem_t *mem = mem_create_for_target(s3);
    esp32_periph_t *periph = mem ? periph_create(mem) : NULL;
    xtensa_cpu_t cpu[2];
    ASSERT_TRUE(mem != NULL);
    ASSERT_TRUE(periph != NULL);
    if (!mem || !periph) {
        periph_destroy(periph);
        mem_destroy(mem);
        return;
    }

    for (unsigned core = 0u; core < 2u; core++) {
        xtensa_cpu_init_for_target(&cpu[core], s3);
        cpu[core].mem = mem;
    }
    cpu[1].pc = 0x40378094u;
    ar_write(&cpu[1], 1, 0x3FCECAFEu);
    periph_attach_cpus(periph, &cpu[0], &cpu[1]);

    uint32_t core1_base = desc->base + desc->core_stride;
    mem_write32(mem, core1_base + desc->pdebug_enable_offset, 1u);
    mem_write32(mem, core1_base + desc->recording_offset, 1u);
    ASSERT_EQ(mem_read32(mem, core1_base + desc->pc_offset), 0x40378094u);
    ASSERT_EQ(mem_read32(mem, core1_base + desc->sp_offset), 0x3FCECAFEu);

    periph_destroy(periph);
    mem_destroy(mem);
}

void run_assist_debug_tests(void)
{
    TEST_SUITE("S3 ASSIST_DEBUG recorder");
    RUN_TEST(assist_debug_records_live_and_frozen_dual_core_state);
    RUN_TEST(assist_debug_preserves_unmodeled_register_diagnostics);
    RUN_TEST(assist_debug_rejects_incompatible_geometry);
    RUN_TEST(assist_debug_is_composed_into_the_s3_machine);
}
