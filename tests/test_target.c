/* Target/core-descriptor tests. */
#include "test_helpers.h"

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
}

TEST(target_lx7_interprets_common_isa_in_s3_iram) {
    const uint32_t pc = 0x40370000u;
    const flexe_target_desc_t *s3 =
        flexe_target_by_id(FLEXE_TARGET_ESP32S3);
    xtensa_cpu_t cpu;

    xtensa_cpu_reset_for_target(&cpu, s3);
    cpu.mem = mem_create();
    ASSERT_TRUE(cpu.mem != NULL);
    if (!cpu.mem) return;

    /* Until the target-aware memory constructor lands, map one official S3
     * IRAM page onto test backing. This tests the core's target validity and
     * shared LX6/LX7 decoder without pretending the complete S3 SoC exists. */
    cpu.mem->page_table[pc >> 12] = cpu.mem->sram;
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

void run_target_tests(void) {
    TEST_SUITE("Xtensa target descriptors");

    RUN_TEST(target_reset_uses_lx7_core_configuration);
    RUN_TEST(target_lx7_interprets_common_isa_in_s3_iram);
    RUN_TEST(target_lx7_does_not_build_classic_predecode_table);
}
