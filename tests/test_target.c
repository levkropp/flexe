/* Target/core-descriptor tests. */
#include "test_helpers.h"
#include "savestate.h"

TEST(target_reset_uses_lx7_core_configuration) {
    const flexe_target_desc_t *s3 =
        flexe_target_by_id(FLEXE_TARGET_ESP32S3);
    xtensa_cpu_t cpu;

    xtensa_cpu_reset_for_target(&cpu, s3);

    ASSERT_TRUE(cpu.target == s3);
    ASSERT_EQ(cpu.pc, 0x40000400u);
    ASSERT_EQ(cpu.vecbase, 0x40000000u);
    ASSERT_EQ(cpu.configid0, 0xC2F0FFFEu);
    ASSERT_EQ(cpu.configid1, 0x23090F1Fu);
    ASSERT_EQ(cpu.int_level[14], 7u);
    ASSERT_EQ(flexe_target_bootstrap_stack(s3, 0), 0x3FCEB710u);
    ASSERT_EQ(flexe_target_bootstrap_stack(s3, 1), 0x3FCED710u);
    ASSERT_EQ(flexe_target_bootstrap_stack(s3, 2), 0u);
}

TEST(target_lx7_bootstrap_spills_preserve_rom_abi) {
    const flexe_target_desc_t *s3 =
        flexe_target_by_id(FLEXE_TARGET_ESP32S3);
    const uint32_t cache_interface_ptr = 0x3FCEFFC4u;
    const uint32_t cache_table = 0x3FF1E2B4u;
    xtensa_mem_t *mem = mem_create_for_target(s3);

    ASSERT_TRUE(mem != NULL);
    if (!mem) return;

    mem_write32(mem, cache_interface_ptr, cache_table);
    for (unsigned core = 0; core < 2u; core++) {
        uint32_t stack_top = flexe_target_bootstrap_stack(s3, core);
        /* A CALL12 overflow can save a complete 64-byte register window
         * below the incoming stack pointer. This is the operation that the
         * former APP_CPU boundary directed onto cache_interface_ptr. */
        for (uint32_t offset = 4u; offset <= 64u; offset += 4u)
            mem_write32(mem, stack_top - offset,
                        0xA5000000u | (core << 8) | offset);
    }

    ASSERT_EQ(mem_read32(mem, cache_interface_ptr), cache_table);
    ASSERT_EQ(mem_unmapped_count(mem), 0u);
    mem_destroy(mem);
}

TEST(target_backing_ranges_cover_complete_regions) {
    const flexe_target_desc_t *classic =
        flexe_target_by_id(FLEXE_TARGET_ESP32);
    const flexe_target_desc_t *s3 =
        flexe_target_by_id(FLEXE_TARGET_ESP32S3);

    ASSERT_TRUE(flexe_target_range_uses_backing(
        classic, 0x3FFB0000u, 16u, FLEXE_MEM_SRAM));
    ASSERT_TRUE(flexe_target_range_uses_backing(
        s3, 0x3FCDFF80u, 16u, FLEXE_MEM_SRAM));
    ASSERT_FALSE(flexe_target_range_uses_backing(
        s3, 0x3FC87FFCu, 8u, FLEXE_MEM_SRAM));
    ASSERT_FALSE(flexe_target_range_uses_backing(
        s3, UINT32_MAX - 3u, 8u, FLEXE_MEM_SRAM));
    ASSERT_TRUE(flexe_target_range_uses_backing(
        s3, 0x3FC88000u, 0u, FLEXE_MEM_SRAM));
}

TEST(target_lx7_interprets_common_isa_in_s3_iram) {
    const uint32_t pc = 0x40370000u;
    const flexe_target_desc_t *s3 =
        flexe_target_by_id(FLEXE_TARGET_ESP32S3);
    xtensa_cpu_t cpu;

    xtensa_cpu_reset_for_target(&cpu, s3);
    cpu.mem = mem_create_for_target(s3);
    ASSERT_TRUE(cpu.mem != NULL);
    if (!cpu.mem) return;

    cpu.pc = pc;
    XT_PS_SET_EXCM(cpu.ps, 0);
    put_insn3(&cpu, pc, rrr(8, 0, 3, 4, 5)); /* ADD a3, a4, a5 */
    ar_write(&cpu, 4, 19u);
    ar_write(&cpu, 5, 23u);

    ASSERT_EQ(xtensa_step(&cpu), 0);
    ASSERT_EQ(ar_read(&cpu, 3), 42u);
    ASSERT_EQ(cpu.pc, pc + 3u);
    ASSERT_FALSE(cpu.exception);

    mem_destroy(cpu.mem);
}

TEST(target_cpu_frequency_uses_target_rom_abi_word) {
    const flexe_target_desc_t *classic =
        flexe_target_by_id(FLEXE_TARGET_ESP32);
    const flexe_target_desc_t *s3 =
        flexe_target_by_id(FLEXE_TARGET_ESP32S3);
    xtensa_cpu_t cpu;

    xtensa_cpu_init_for_target(&cpu, s3);
    cpu.mem = mem_create_for_target(s3);
    ASSERT_TRUE(cpu.mem != NULL);
    if (!cpu.mem) return;

    ASSERT_EQ(xtensa_cpu_freq_mhz(&cpu), 160u);
    mem_write32(cpu.mem, s3->cpu_frequency_word, 240u);
    ASSERT_EQ(xtensa_cpu_freq_mhz(&cpu), 240u);
    /* A classic-only address must not override S3's ROM-maintained word. */
    mem_write32(cpu.mem, classic->cpu_frequency_word, 80u);
    ASSERT_EQ(xtensa_cpu_freq_mhz(&cpu), 240u);

    mem_destroy(cpu.mem);
}

TEST(target_lx7_does_not_build_classic_predecode_table) {
    const flexe_target_desc_t *s3 =
        flexe_target_by_id(FLEXE_TARGET_ESP32S3);
    xtensa_cpu_t cpu;

    xtensa_cpu_init_for_target(&cpu, s3);
    cpu.mem = mem_create();
    ASSERT_TRUE(cpu.mem != NULL);
    if (!cpu.mem) return;

    xtensa_predecode_build(&cpu);
    ASSERT_TRUE(cpu.predecode == NULL);

    mem_destroy(cpu.mem);
}

TEST(target_lx7_savestate_uses_descriptor_backing_sizes) {
    const char *path = "/tmp/flexe-s3-savestate-unit.bin";
    const flexe_target_desc_t *s3 =
        flexe_target_by_id(FLEXE_TARGET_ESP32S3);
    xtensa_cpu_t saved;
    xtensa_cpu_t restored;

    xtensa_cpu_init_for_target(&saved, s3);
    xtensa_cpu_init_for_target(&restored, s3);
    saved.mem = mem_create_for_target(s3);
    restored.mem = mem_create_for_target(s3);
    ASSERT_TRUE(saved.mem != NULL);
    ASSERT_TRUE(restored.mem != NULL);
    if (!saved.mem || !restored.mem) {
        mem_destroy(saved.mem);
        mem_destroy(restored.mem);
        return;
    }

    saved.pc = 0x403752F4u;
    saved.insn_count = 12345u;
    mem_write32(saved.mem, 0x3FC92300u, 0xA1B2C3D4u);
    mem_write32(saved.mem, 0x40374000u, 0x55667788u);
    mem_write32(saved.mem, 0x600FE000u, 0x0BADCAFEu);
    ASSERT_EQ(savestate_save(&saved, NULL, path, "esp32-s3-unit"), 0);
    ASSERT_EQ(savestate_restore(&restored, NULL, path), 0);
    ASSERT_TRUE(restored.target == s3);
    ASSERT_EQ(restored.pc, 0x403752F4u);
    ASSERT_EQ(restored.insn_count, 12345u);
    ASSERT_EQ(mem_read32(restored.mem, 0x3FC92300u), 0xA1B2C3D4u);
    ASSERT_EQ(mem_read32(restored.mem, 0x40374000u), 0x55667788u);
    ASSERT_EQ(mem_read32(restored.mem, 0x600FE000u), 0x0BADCAFEu);

    remove(path);
    mem_destroy(saved.mem);
    mem_destroy(restored.mem);
}

void run_target_tests(void) {
    TEST_SUITE("Xtensa target descriptors");

    RUN_TEST(target_reset_uses_lx7_core_configuration);
    RUN_TEST(target_lx7_bootstrap_spills_preserve_rom_abi);
    RUN_TEST(target_backing_ranges_cover_complete_regions);
    RUN_TEST(target_lx7_interprets_common_isa_in_s3_iram);
    RUN_TEST(target_cpu_frequency_uses_target_rom_abi_word);
    RUN_TEST(target_lx7_does_not_build_classic_predecode_table);
    RUN_TEST(target_lx7_savestate_uses_descriptor_backing_sizes);
}
