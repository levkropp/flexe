/*
 * Tests for the memory subsystem (address translation, read/write, mem_load).
 */
#include "test_helpers.h"
#include <string.h>

typedef struct {
    uint32_t last_addr;
    uint32_t value;
} target_mmio_probe_t;

static uint32_t target_mmio_read(void *ctx, uint32_t addr) {
    target_mmio_probe_t *probe = ctx;
    probe->last_addr = addr;
    return probe->value;
}

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

/* ===== SRAM instruction region (separate from data bus) ===== */

TEST(mem_sram_alias) {
    xtensa_mem_t *mem = mem_create();
    /* Data bus and instruction bus are separate regions (ESP32 aliasing
     * has reversed byte order which we don't model) */
    mem_write32(mem, 0x3FFB0000, 0xCAFEBABE);
    ASSERT_EQ(mem_read32(mem, 0x3FFB0000), 0xCAFEBABE);
    mem_write32(mem, 0x40070000, 0x12345678);
    ASSERT_EQ(mem_read32(mem, 0x40070000), 0x12345678);
    /* Verify they are independent */
    ASSERT_EQ(mem_read32(mem, 0x3FFB0000), 0xCAFEBABE);
    mem_destroy(mem);
}

TEST(mem_esp32s3_native_map_and_diram_alias) {
    const flexe_target_desc_t *s3 =
        flexe_target_by_id(FLEXE_TARGET_ESP32S3);
    xtensa_mem_t *mem = mem_create_for_target(s3);
    ASSERT_TRUE(mem != NULL);
    if (!mem) return;

    ASSERT_TRUE(mem_target(mem) == s3);
    ASSERT_EQ(mem_backing_size(mem, FLEXE_MEM_SRAM), 0x80000u);
    ASSERT_TRUE(mem_get_ptr(mem, 0x3FC87FFFu) == NULL);
    ASSERT_TRUE(mem_get_ptr(mem, 0x3FC88000u) != NULL);
    ASSERT_TRUE(mem_get_ptr(mem, 0x3FCFFFFFu) != NULL);
    ASSERT_TRUE(mem_get_ptr(mem, 0x3FD00000u) == NULL);
    ASSERT_TRUE(mem_get_ptr(mem, 0x4036FFFFu) == NULL);
    ASSERT_TRUE(mem_get_ptr(mem, 0x40370000u) != NULL);
    ASSERT_TRUE(mem_get_ptr(mem, 0x403DFFFFu) != NULL);
    ASSERT_TRUE(mem_get_ptr(mem, 0x403E0000u) == NULL);

    /* S3 D/IRAM is a real alias above the 32 KiB cache-SRAM prefix. */
    mem_write32(mem, 0x3FC88000u, 0x12345678u);
    ASSERT_EQ(mem_read32(mem, 0x40378000u), 0x12345678u);
    mem_write32(mem, 0x4037FFF0u, 0xA5A55A5Au);
    ASSERT_EQ(mem_read32(mem, 0x3FC8FFF0u), 0xA5A55A5Au);
    mem_write32(mem, 0x40370000u, 0xC001CAFEu);
    ASSERT_EQ(mem_read32(mem, 0x3FC88000u), 0x12345678u);

    /* The ROM D-bus window aliases the last 128 KiB of mask ROM. */
    mem_write32(mem, 0x40058C00u, 0x0BADF00Du);
    ASSERT_EQ(mem_read32(mem, 0x3FF18C00u), 0x0BADF00Du);
    ASSERT_TRUE(mem_get_ptr(mem, 0x3FF00000u) ==
                mem_backing_ptr(mem, FLEXE_MEM_ROM) + 0x40000u);

    ASSERT_TRUE(mem_get_ptr(mem, 0x3C000000u) != NULL);
    ASSERT_TRUE(mem_get_ptr(mem, 0x3C3FFFFFu) != NULL);
    ASSERT_TRUE(mem_get_ptr(mem, 0x3C400000u) == NULL);
    ASSERT_TRUE(mem_get_ptr(mem, 0x42000000u) != NULL);
    ASSERT_TRUE(mem_get_ptr(mem, 0x423FFFFFu) != NULL);
    ASSERT_TRUE(mem_get_ptr(mem, 0x42400000u) == NULL);
    ASSERT_TRUE(mem_get_ptr(mem, 0x600FE000u) != NULL);
    ASSERT_TRUE(mem_get_ptr(mem, 0x600FFFFFu) != NULL);

    mem_destroy(mem);
}

TEST(mem_flash_capacity_follows_image_and_rom_handoff) {
    const flexe_target_desc_t *s3 =
        flexe_target_by_id(FLEXE_TARGET_ESP32S3);
    xtensa_mem_t *mem = mem_create_for_target_with_flash(s3, 0x00800000u);
    ASSERT_TRUE(mem != NULL);
    if (!mem) return;

    ASSERT_EQ(mem_flash_physical_size(mem), 0x00800000u);
    ASSERT_EQ(mem_flash_usable_size(mem), 0x00800000u);
    ASSERT_EQ(mem_flash_jedec_id(mem), 0x001740C8u);
    ASSERT_EQ(mem_backing_size(mem, FLEXE_MEM_FLASH_DATA), 0x00800000u);
    ASSERT_EQ(mem_backing_size(mem, FLEXE_MEM_FLASH_INSN), 0x00800000u);
    ASSERT_EQ(mem->flash_data[0x007FFFFFu], 0xFFu);
    ASSERT_EQ(mem->flash_insn[0x007FFFFFu], 0xFFu);

    const uint32_t handoff = 0x3FCEF6A4u;
    ASSERT_EQ(mem_prepare_rom_flash(mem, handoff, 0x00800000u), 0);
    ASSERT_EQ(mem_read32(mem, handoff + 0u), 0x00C84017u);
    ASSERT_EQ(mem_read32(mem, handoff + 4u), 0x00800000u);
    ASSERT_EQ(mem_read32(mem, handoff + 8u), 0x00010000u);
    ASSERT_EQ(mem_read32(mem, handoff + 12u), 0x00001000u);
    ASSERT_EQ(mem_read32(mem, handoff + 16u), 0x00000100u);
    ASSERT_EQ(mem_read32(mem, handoff + 20u), 0x0000FFFFu);
    mem_destroy(mem);

    ASSERT_TRUE(mem_create_for_target_with_flash(s3, 0x00300000u) == NULL);
    ASSERT_TRUE(mem_create_for_target_with_flash(s3, 0x10000000u) == NULL);

    mem = mem_create();
    ASSERT_TRUE(mem != NULL);
    if (!mem) return;
    ASSERT_EQ(mem_read32(mem, 0x3FFAE270u), 0x00C84016u);
    ASSERT_EQ(mem_read32(mem, 0x3FFAE274u), 0x00400000u);
    mem_destroy(mem);
}

TEST(mem_esp32s3_mmio_is_native_not_classic_alias) {
    const flexe_target_desc_t *s3 =
        flexe_target_by_id(FLEXE_TARGET_ESP32S3);
    xtensa_mem_t *mem = mem_create_for_target(s3);
    target_mmio_probe_t low = {0, 0x11223344u};
    target_mmio_probe_t high = {0, 0x55667788u};
    ASSERT_TRUE(mem != NULL);
    if (!mem) return;

    ASSERT_EQ(mem_register_mmio_range(mem, 0x60000000u, 0x1000u,
                                      target_mmio_read, NULL, &low), 0);
    ASSERT_EQ(mem_register_mmio_range(mem, 0x600C5000u, 0x1000u,
                                      target_mmio_read, NULL, &high), 0);
    ASSERT_EQ(mem_read32(mem, 0x60000020u), 0x11223344u);
    ASSERT_EQ(low.last_addr, 0x60000020u);
    ASSERT_EQ(mem_read32(mem, 0x600C5000u), 0x55667788u);
    ASSERT_EQ(high.last_addr, 0x600C5000u);

    /* On classic ESP32, 0x60000020 mirrors 0x3ff40020. On S3 it is the
     * canonical UART address, and the classic APB address is unmapped. */
    ASSERT_EQ(mem_read32(mem, 0x3FF40020u), 0u);
    ASSERT_EQ(low.last_addr, 0x60000020u);
    ASSERT_EQ(mem_register_mmio_range(mem, 0x600D1000u, 0x1000u,
                                      target_mmio_read, NULL, &high), -1);

    mem_destroy(mem);
}

/* ===== On-chip ROM instruction/data buses ===== */

TEST(mem_full_esp32_rom_map) {
    xtensa_mem_t *mem = mem_create();

    ASSERT_TRUE(mem_get_ptr(mem, 0x3FF8FFFFu) == NULL);
    ASSERT_TRUE(mem_get_ptr(mem, 0x3FF90000u) != NULL);
    ASSERT_TRUE(mem_get_ptr(mem, 0x3FF9FFFFu) != NULL);
    ASSERT_TRUE(mem_get_ptr(mem, 0x40000000u) != NULL);
    ASSERT_TRUE(mem_get_ptr(mem, 0x4006FFFFu) != NULL);
    ASSERT_TRUE(mem_get_ptr(mem, 0x40070000u) != NULL); /* IRAM begins here */

    mem_write32(mem, 0x3FF96000u, 0xDADA1234u);
    mem_write32(mem, 0x40064EECu, 0xC0DE5678u);
    ASSERT_EQ(mem_read32(mem, 0x3FF96000u), 0xDADA1234u);
    ASSERT_EQ(mem_read32(mem, 0x40064EECu), 0xC0DE5678u);

    /* The two ROM buses and adjacent IRAM have independent backing. */
    ASSERT_EQ(mem_read32(mem, 0x40000000u), 0u);
    ASSERT_EQ(mem_read32(mem, 0x40070000u), 0u);
    mem_destroy(mem);
}

TEST(mem_builtin_rom_ctype_table) {
    static const uint32_t ptr_addr = 0x3FF96350u;
    static const uint32_t table_addr = 0x3FF96354u;
    xtensa_mem_t *mem = mem_create();

    ASSERT_EQ(mem_read32(mem, ptr_addr), table_addr);
    ASSERT_EQ(mem_read8(mem, table_addr), 0u); /* reserved first entry */
    ASSERT_EQ(mem_read8(mem, table_addr + 1u + '\0'), 0x20u);
    ASSERT_EQ(mem_read8(mem, table_addr + 1u + '\t'), 0x28u);
    ASSERT_EQ(mem_read8(mem, table_addr + 1u + ' '), 0x88u);
    ASSERT_EQ(mem_read8(mem, table_addr + 1u + '!'), 0x10u);
    ASSERT_EQ(mem_read8(mem, table_addr + 1u + '0'), 0x04u);
    ASSERT_EQ(mem_read8(mem, table_addr + 1u + 'A'), 0x41u);
    ASSERT_EQ(mem_read8(mem, table_addr + 1u + 'F'), 0x41u);
    ASSERT_EQ(mem_read8(mem, table_addr + 1u + 'G'), 0x01u);
    ASSERT_EQ(mem_read8(mem, table_addr + 1u + 'a'), 0x42u);
    ASSERT_EQ(mem_read8(mem, table_addr + 1u + 'f'), 0x42u);
    ASSERT_EQ(mem_read8(mem, table_addr + 1u + 'g'), 0x02u);
    ASSERT_EQ(mem_read8(mem, table_addr + 1u + 0x7Fu), 0x20u);
    ASSERT_EQ(mem_read8(mem, table_addr + 1u + 0x80u), 0u);
    ASSERT_EQ(mem_read8(mem, table_addr + 1u + 0xFFu), 0u);

    /* Reset clears volatile RAM, not immutable ROM D-bus contents. */
    mem_reset(mem);
    ASSERT_EQ(mem_read32(mem, ptr_addr), table_addr);
    ASSERT_EQ(mem_read8(mem, table_addr + 1u + ' '), 0x88u);
    mem_destroy(mem);
}

/* ===== Flash data/instruction regions ===== */

TEST(mem_rw32_flash_data) {
    xtensa_mem_t *mem = mem_create();
    mem_write32(mem, 0x3F400000, 0xF1A5CAFE);
    ASSERT_EQ(mem_read32(mem, 0x3F400000), 0xF1A5CAFE);
    mem_destroy(mem);
}

TEST(mem_flash_erased_at_reset) {
    xtensa_mem_t *mem = mem_create();
    ASSERT_EQ(mem_read32(mem, 0x3F400000), 0xFFFFFFFFu);
    ASSERT_EQ(mem_read32(mem, 0x400D0000), 0xFFFFFFFFu);
    mem_destroy(mem);
}

TEST(mem_flash_alias) {
    xtensa_mem_t *mem = mem_create();
    /* Flash data and insn buses use separate backing arrays (like real MMU pages) */
    mem_write32(mem, 0x3F400000, 0xAAAABBBB);
    ASSERT_EQ(mem_read32(mem, 0x3F400000), 0xAAAABBBB);
    mem_write32(mem, 0x400D0000, 0xCCCCDDDD);
    ASSERT_EQ(mem_read32(mem, 0x400D0000), 0xCCCCDDDD);
    /* They are independent */
    ASSERT_EQ(mem_read32(mem, 0x3F400000), 0xAAAABBBB);
    mem_destroy(mem);
}

TEST(mem_backing_ranges_can_be_remapped_and_unmapped) {
    xtensa_mem_t *mem = mem_create();
    const uint32_t guest = 0x3F600000u;
    const uint32_t flash_offset = 0x00020000u;

    mem->flash_data[flash_offset + 0x123u] = 0x5Au;
    ASSERT_EQ(mem_map_backing_range(mem, guest, FLEXE_MEM_FLASH_DATA,
                                    flash_offset, 0x10000u), 0);
    ASSERT_EQ(mem_read8(mem, guest + 0x123u), 0x5Au);
    ASSERT_TRUE(mem_get_ptr(mem, guest) ==
                mem->flash_data + flash_offset);

    ASSERT_EQ(mem_unmap_range(mem, guest, 0x10000u), 0);
    ASSERT_TRUE(mem_get_ptr(mem, guest) == NULL);
    ASSERT_TRUE(mem_get_ptr(mem, guest + 0xF000u) == NULL);

    /* Reject partial pages, wraparound, and mappings beyond a backing. */
    ASSERT_EQ(mem_map_backing_range(mem, guest + 1u,
                                    FLEXE_MEM_FLASH_DATA,
                                    flash_offset, 0x1000u), -1);
    ASSERT_EQ(mem_map_backing_range(mem, guest, FLEXE_MEM_FLASH_DATA,
                                    mem_backing_size(
                                        mem, FLEXE_MEM_FLASH_DATA),
                                    0x1000u), -1);
    ASSERT_EQ(mem_unmap_range(mem, 0xFFFFF000u, 0x2000u), -1);
    mem_destroy(mem);
}

TEST(mem_flash_instruction_window_boundary) {
    xtensa_mem_t *mem = mem_create();
    ASSERT_TRUE(mem_get_ptr(mem, 0x400CFFFFu) == NULL);
    ASSERT_TRUE(mem_get_ptr(mem, 0x400D0000u) != NULL);
    ASSERT_TRUE(mem_get_ptr(mem, 0x403FFFFFu) != NULL);
    ASSERT_TRUE(mem_get_ptr(mem, 0x40400000u) == NULL);
    mem_destroy(mem);
}

/* ===== RTC slow / AHB peripheral window ===== */

TEST(mem_rw32_rtc_slow) {
    xtensa_mem_t *mem = mem_create();
    mem_write32(mem, 0x50000000, 0x11111111);
    ASSERT_EQ(mem_read32(mem, 0x50000000), 0x11111111);
    mem_destroy(mem);
}

TEST(mem_ahb_window_is_not_rtc_memory) {
    xtensa_mem_t *mem = mem_create();
    ASSERT_TRUE(mem_get_ptr(mem, 0x60000000) == NULL);
    ASSERT_TRUE(mem_get_ptr(mem, 0x60001000) == NULL);
    mem_write32(mem, 0x60001000, 0x22222222);
    ASSERT_EQ(mem_read32(mem, 0x60001000), 0);
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

static uint32_t journal_mmio_read(void *ctx, uint32_t addr) {
    unsigned *reads = ctx;
    (*reads)++;
    return addr ^ 0xA5A5A5A5u;
}

TEST(mem_journal_rejects_mmio_reads) {
    xtensa_mem_t *mem = mem_create();
    unsigned reads = 0;
    const uint32_t addr = 0x3FF50020u;
    ASSERT_EQ(mem_register_mmio(mem, 0x50, journal_mmio_read, NULL, &reads), 0);

    /* Ordinary mapped reads are deterministic and remain replayable. */
    mem_journal_begin();
    ASSERT_EQ(mem_read32(mem, 0x3FFB0000u), 0u);
    ASSERT_FALSE(g_mem_journal_unsafe);
    mem_journal_end();

    /* Every slow read width is potentially stateful. In particular, live
     * TIMG/LACT counters can advance between a native run and its interpreter
     * replay even though neither execution wrote memory. */
    mem_journal_begin();
    ASSERT_EQ(mem_read8(mem, addr),
              (uint8_t)(addr ^ 0xA5A5A5A5u));
    ASSERT_TRUE(g_mem_journal_unsafe);
    mem_journal_end();

    mem_journal_begin();
    ASSERT_EQ(mem_read16(mem, addr),
              (uint16_t)(addr ^ 0xA5A5A5A5u));
    ASSERT_TRUE(g_mem_journal_unsafe);
    mem_journal_end();

    mem_journal_begin();
    ASSERT_EQ(mem_read32(mem, addr), addr ^ 0xA5A5A5A5u);
    ASSERT_TRUE(g_mem_journal_unsafe);
    mem_journal_end();

    ASSERT_EQ(reads, 3u);
    mem_destroy(mem);
}

TEST(mem_journal_pause_preserves_recorded_writes) {
    xtensa_mem_t *mem = mem_create();
    const uint32_t recorded = 0x3FFB0000u;
    const uint32_t unrecorded = recorded + 4u;

    mem_write32(mem, recorded, 0x11111111u);
    mem_write32(mem, unrecorded, 0x22222222u);
    mem_journal_begin();
    mem_write32(mem, recorded, 0xAAAAAAAAu);
    ASSERT_EQ(g_mem_journal_count, 1);

    mem_journal_pause();
    ASSERT_FALSE(g_mem_journal_en);
    ASSERT_EQ(g_mem_journal_count, 1);
    mem_write32(mem, unrecorded, 0xBBBBBBBBu);
    ASSERT_EQ(g_mem_journal_count, 1);

    mem_journal_rollback(mem);
    ASSERT_EQ(mem_read32(mem, recorded), 0x11111111u);
    ASSERT_EQ(mem_read32(mem, unrecorded), 0xBBBBBBBBu);
    mem_journal_end();
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
    RUN_TEST(mem_esp32s3_native_map_and_diram_alias);
    RUN_TEST(mem_flash_capacity_follows_image_and_rom_handoff);
    RUN_TEST(mem_esp32s3_mmio_is_native_not_classic_alias);
    RUN_TEST(mem_full_esp32_rom_map);
    RUN_TEST(mem_builtin_rom_ctype_table);
    RUN_TEST(mem_rw32_flash_data);
    RUN_TEST(mem_flash_erased_at_reset);
    RUN_TEST(mem_flash_alias);
    RUN_TEST(mem_backing_ranges_can_be_remapped_and_unmapped);
    RUN_TEST(mem_flash_instruction_window_boundary);
    RUN_TEST(mem_rw32_rtc_slow);
    RUN_TEST(mem_ahb_window_is_not_rtc_memory);
    RUN_TEST(mem_unmapped_read_zero);
    RUN_TEST(mem_unmapped_write_silent);
    RUN_TEST(mem_periph_returns_zero);
    RUN_TEST(mem_journal_rejects_mmio_reads);
    RUN_TEST(mem_journal_pause_preserves_recorded_writes);
    RUN_TEST(mem_load_basic);
    RUN_TEST(mem_load_unmapped_fails);
}
