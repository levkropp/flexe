/*
 * Tests for the memory subsystem (address translation, read/write, mem_load).
 */
#include "test_helpers.h"
#include <string.h>

/* ===== SRAM data region ===== */

TEST(mem_rw8_sram_data) {
    xtensa_mem_t *mem = mem_create();
    mem_write8(mem, 0x3FFB0000, 0xAB);
    ASSERT_EQ(mem_read8(mem, 0x3FFB0000), 0xAB);
    mem_destroy(mem);
}

TEST(mem_rw16_sram_data) {
    xtensa_mem_t *mem = mem_create();
    mem_write16(mem, 0x3FFB0000, 0x1234);
    ASSERT_EQ(mem_read16(mem, 0x3FFB0000), 0x1234);
    mem_destroy(mem);
}

TEST(mem_rw32_sram_data) {
    xtensa_mem_t *mem = mem_create();
    mem_write32(mem, 0x3FFB0000, 0xDEADBEEF);
    ASSERT_EQ(mem_read32(mem, 0x3FFB0000), 0xDEADBEEF);
    mem_destroy(mem);
}

/* ===== SRAM instruction region (alias) ===== */

TEST(mem_sram_alias) {
    xtensa_mem_t *mem = mem_create();
    /* Write via data bus, read via instruction bus */
    mem_write32(mem, 0x3FFB0000, 0xCAFEBABE);
    /* SRAM instruction base = 0x40070000, same physical offset */
    ASSERT_EQ(mem_read32(mem, 0x40070000), 0xCAFEBABE);
    /* And the reverse */
    mem_write32(mem, 0x40070004, 0x12345678);
    ASSERT_EQ(mem_read32(mem, 0x3FFB0004), 0x12345678);
    mem_destroy(mem);
}

/* ===== Flash data/instruction regions ===== */

TEST(mem_rw32_flash_data) {
    xtensa_mem_t *mem = mem_create();
    mem_write32(mem, 0x3F400000, 0xF1A5CAFE);
    ASSERT_EQ(mem_read32(mem, 0x3F400000), 0xF1A5CAFE);
    mem_destroy(mem);
}

TEST(mem_flash_alias) {
    xtensa_mem_t *mem = mem_create();
    /* Flash data base = 0x3F400000, flash insn base = 0x400C2000 */
    /* Both map to same physical flash at the same offset */
    mem_write32(mem, 0x3F400000, 0xAAAABBBB);
    ASSERT_EQ(mem_read32(mem, 0x400C2000), 0xAAAABBBB);
    mem_destroy(mem);
}

/* ===== RTC fast/slow ===== */

TEST(mem_rw32_rtc_fast) {
    xtensa_mem_t *mem = mem_create();
    mem_write32(mem, 0x50000000, 0x11111111);
    ASSERT_EQ(mem_read32(mem, 0x50000000), 0x11111111);
    mem_destroy(mem);
}

TEST(mem_rw32_rtc_slow) {
    xtensa_mem_t *mem = mem_create();
    mem_write32(mem, 0x60000000, 0x22222222);
    ASSERT_EQ(mem_read32(mem, 0x60000000), 0x22222222);
    mem_destroy(mem);
}

/* ===== Unmapped / peripheral ===== */

TEST(mem_unmapped_read_zero) {
    xtensa_mem_t *mem = mem_create();
    /* Some address that's not mapped to any region */
    ASSERT_EQ(mem_read32(mem, 0x10000000), 0);
    ASSERT_EQ(mem_read8(mem, 0x10000000), 0);
    mem_destroy(mem);
}

TEST(mem_unmapped_write_silent) {
    xtensa_mem_t *mem = mem_create();
    /* Writing to unmapped address should not crash */
    mem_write32(mem, 0x10000000, 0xDEAD);
    ASSERT_EQ(mem_read32(mem, 0x10000000), 0); /* still zero */
    mem_destroy(mem);
}

TEST(mem_periph_returns_zero) {
    xtensa_mem_t *mem = mem_create();
    /* Peripheral region: 0x3FF00000 - 0x3FF7FFFF */
    ASSERT_EQ(mem_read32(mem, 0x3FF00000), 0);
    /* Should not crash */
    mem_write32(mem, 0x3FF00000, 0x1234);
    mem_destroy(mem);
}

/* ===== mem_load bulk copy ===== */

TEST(mem_load_basic) {
    xtensa_mem_t *mem = mem_create();
    uint8_t data[] = {0x11, 0x22, 0x33, 0x44, 0x55};
    int rc = mem_load(mem, 0x3FFB0100, data, sizeof(data));
    ASSERT_EQ(rc, 0);
    ASSERT_EQ(mem_read8(mem, 0x3FFB0100), 0x11);
    ASSERT_EQ(mem_read8(mem, 0x3FFB0101), 0x22);
    ASSERT_EQ(mem_read8(mem, 0x3FFB0102), 0x33);
    ASSERT_EQ(mem_read8(mem, 0x3FFB0103), 0x44);
    ASSERT_EQ(mem_read8(mem, 0x3FFB0104), 0x55);
    mem_destroy(mem);
}

TEST(mem_load_unmapped_fails) {
    xtensa_mem_t *mem = mem_create();
    uint8_t data[] = {0x11};
    int rc = mem_load(mem, 0x10000000, data, sizeof(data));
    ASSERT_EQ(rc, -1);
    mem_destroy(mem);
}

void run_memory_tests(void) {
    TEST_SUITE("Memory Subsystem");

    RUN_TEST(mem_rw8_sram_data);
    RUN_TEST(mem_rw16_sram_data);
    RUN_TEST(mem_rw32_sram_data);
    RUN_TEST(mem_sram_alias);
    RUN_TEST(mem_rw32_flash_data);
    RUN_TEST(mem_flash_alias);
    RUN_TEST(mem_rw32_rtc_fast);
    RUN_TEST(mem_rw32_rtc_slow);
    RUN_TEST(mem_unmapped_read_zero);
    RUN_TEST(mem_unmapped_write_silent);
    RUN_TEST(mem_periph_returns_zero);
    RUN_TEST(mem_load_basic);
    RUN_TEST(mem_load_unmapped_fails);
}
