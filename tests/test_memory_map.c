#include "test_helpers.h"
#include "loader.h"
#include "peripherals.h"

/* ====== RTC DRAM tests ====== */

TEST(test_rtc_dram_read_write) {
    xtensa_cpu_t cpu;
    setup(&cpu);
    /* Write via D-bus alias at 0x3FF80000 */
    mem_write32(cpu.mem, 0x3FF80000u, 0xDEADBEEF);
    ASSERT_EQ(mem_read32(cpu.mem, 0x3FF80000u), 0xDEADBEEF);
    /* Byte access */
    mem_write8(cpu.mem, 0x3FF80010u, 0x42);
    ASSERT_EQ(mem_read8(cpu.mem, 0x3FF80010u), 0x42);
    teardown(&cpu);
}

TEST(test_rtc_dram_iram_alias) {
    xtensa_cpu_t cpu;
    setup(&cpu);
    /* Write via D-bus (0x3FF80000), read via I-bus (0x400C0000) */
    mem_write32(cpu.mem, 0x3FF80000u, 0xCAFEBABE);
    ASSERT_EQ(mem_read32(cpu.mem, 0x400C0000u), 0xCAFEBABE);
    /* Write via I-bus, read via D-bus */
    mem_write32(cpu.mem, 0x400C0100u, 0x12345678);
    ASSERT_EQ(mem_read32(cpu.mem, 0x3FF80100u), 0x12345678);
    teardown(&cpu);
}

TEST(test_rtc_dram_boundary) {
    xtensa_cpu_t cpu;
    setup(&cpu);
    /* Last valid word in 8KB region */
    mem_write32(cpu.mem, 0x3FF81FFCu, 0xABCD1234);
    ASSERT_EQ(mem_read32(cpu.mem, 0x3FF81FFCu), 0xABCD1234);
    /* Corresponding I-bus alias */
    ASSERT_EQ(mem_read32(cpu.mem, 0x400C1FFCu), 0xABCD1234);
    teardown(&cpu);
}

TEST(test_sram_insn_boundary) {
    xtensa_cpu_t cpu;
    setup(&cpu);
    /* SRAM instruction bus should end at 0x400C0000, not 0x400C2000 */
    /* Write to last valid SRAM I-bus word */
    mem_write32(cpu.mem, 0x400BFFFCu, 0x11223344);
    ASSERT_EQ(mem_read32(cpu.mem, 0x400BFFFCu), 0x11223344);
    /* 0x400C0000 should be RTC IRAM, not SRAM */
    mem_write32(cpu.mem, 0x400C0000u, 0x55667788);
    /* Verify it's separate from SRAM (the SRAM offset at 0x400C0000 would be 0x50000) */
    ASSERT_EQ(mem_read32(cpu.mem, 0x400C0000u), 0x55667788);
    /* Verify it aliases to RTC DRAM, not SRAM */
    ASSERT_EQ(mem_read32(cpu.mem, 0x3FF80000u), 0x55667788);
    teardown(&cpu);
}

TEST(test_rtc_dram_mem_load) {
    xtensa_cpu_t cpu;
    setup(&cpu);
    /* mem_load should succeed for RTC DRAM region */
    uint8_t data[] = {0x01, 0x02, 0x03, 0x04};
    int rc = mem_load(cpu.mem, 0x3FF80000u, data, sizeof(data));
    ASSERT_EQ(rc, 0);
    ASSERT_EQ(mem_read32(cpu.mem, 0x3FF80000u), 0x04030201);
    teardown(&cpu);
}

/* ====== Firmware segment load addresses test ====== */

TEST(test_firmware_segment_regions) {
    xtensa_cpu_t cpu;
    setup(&cpu);
    /* Test that all typical ESP-IDF segment addresses are loadable */
    uint8_t data[4] = {0xAA, 0xBB, 0xCC, 0xDD};
    /* Flash data: 0x3F400020 */
    ASSERT_EQ(mem_load(cpu.mem, 0x3F400020u, data, 4), 0);
    /* RTC DRAM: 0x3FF80000 */
    ASSERT_EQ(mem_load(cpu.mem, 0x3FF80000u, data, 4), 0);
    /* SRAM data: 0x3FFB0000 */
    ASSERT_EQ(mem_load(cpu.mem, 0x3FFB0000u, data, 4), 0);
    /* SRAM insn: 0x40080000 */
    ASSERT_EQ(mem_load(cpu.mem, 0x40080000u, data, 4), 0);
    /* RTC IRAM: 0x400C0000 */
    ASSERT_EQ(mem_load(cpu.mem, 0x400C0000u, data, 4), 0);
    teardown(&cpu);
}

/* ====== Peripheral stub tests ====== */

TEST(test_peripheral_spi0_stub) {
    xtensa_cpu_t cpu;
    setup(&cpu);
    esp32_periph_t *p = periph_create(cpu.mem);
    /* SPI0 reads should not crash and return sensible values */
    uint32_t cmd = mem_read32(cpu.mem, 0x3FF43000u);  /* SPI_CMD_REG */
    ASSERT_EQ(cmd, 0);  /* Not busy */
    uint32_t status = mem_read32(cpu.mem, 0x3FF43010u);  /* SPI_STATUS_REG */
    ASSERT_EQ(status, 0);  /* Ready */
    /* Write should not crash */
    mem_write32(cpu.mem, 0x3FF43000u, 0x01);
    periph_destroy(p);
    teardown(&cpu);
}

TEST(test_peripheral_syscon_stub) {
    xtensa_cpu_t cpu;
    setup(&cpu);
    esp32_periph_t *p = periph_create(cpu.mem);
    /* SYSCON read should return 0 for most registers */
    uint32_t val = mem_read32(cpu.mem, 0x3FF66000u);
    ASSERT_EQ(val, 0);
    /* Write should not crash */
    mem_write32(cpu.mem, 0x3FF66000u, 0x1234);
    periph_destroy(p);
    teardown(&cpu);
}

/* ====== Loader region name test ====== */

TEST(test_loader_region_names) {
    ASSERT_TRUE(strcmp(loader_region_name(0x3F400020), "flash_data") == 0);
    ASSERT_TRUE(strcmp(loader_region_name(0x3FF80000), "rtc_dram") == 0);
    ASSERT_TRUE(strcmp(loader_region_name(0x3FFB0000), "sram_data") == 0);
    ASSERT_TRUE(strcmp(loader_region_name(0x40080000), "sram_insn") == 0);
    ASSERT_TRUE(strcmp(loader_region_name(0x400C0000), "rtc_iram") == 0);
    ASSERT_TRUE(strcmp(loader_region_name(0x400D0000), "flash_insn") == 0);
    ASSERT_TRUE(strcmp(loader_region_name(0x3FF40000), "peripheral") == 0);
    ASSERT_TRUE(strcmp(loader_region_name(0x10000000), "unmapped") == 0);
}

/* ====== Loader segment info test ====== */

TEST(test_loader_segment_info) {
    /* Create a minimal .bin in memory with 2 segments */
    const char *path = "/tmp/test_seg_info.bin";
    FILE *f = fopen(path, "wb");
    /* Header: magic=0xE9, seg_count=2, ..., entry=0x40080000 */
    uint8_t hdr[24] = {0};
    hdr[0] = 0xE9;  /* magic */
    hdr[1] = 2;     /* 2 segments */
    hdr[4] = 0x00; hdr[5] = 0x00; hdr[6] = 0x08; hdr[7] = 0x40;  /* entry 0x40080000 */
    fwrite(hdr, 1, 24, f);
    /* Segment 0: addr=0x3FFB0000, size=4 */
    uint8_t seg0_hdr[8] = {0x00, 0x00, 0xFB, 0x3F, 0x04, 0x00, 0x00, 0x00};
    fwrite(seg0_hdr, 1, 8, f);
    uint8_t seg0_data[4] = {0x11, 0x22, 0x33, 0x44};
    fwrite(seg0_data, 1, 4, f);
    /* Segment 1: addr=0x3FF80000, size=4 */
    uint8_t seg1_hdr[8] = {0x00, 0x00, 0xF8, 0x3F, 0x04, 0x00, 0x00, 0x00};
    fwrite(seg1_hdr, 1, 8, f);
    uint8_t seg1_data[4] = {0xAA, 0xBB, 0xCC, 0xDD};
    fwrite(seg1_data, 1, 4, f);
    fclose(f);

    xtensa_cpu_t cpu;
    setup(&cpu);
    load_result_t res = loader_load_bin(cpu.mem, path);
    ASSERT_EQ(res.result, 0);
    ASSERT_EQ(res.segment_count, 2);
    ASSERT_EQ(res.segments[0].addr, 0x3FFB0000u);
    ASSERT_EQ(res.segments[0].size, 4);
    ASSERT_EQ(res.segments[1].addr, 0x3FF80000u);
    ASSERT_EQ(res.segments[1].size, 4);
    /* Verify data loaded correctly */
    ASSERT_EQ(mem_read32(cpu.mem, 0x3FFB0000u), 0x44332211);
    ASSERT_EQ(mem_read32(cpu.mem, 0x3FF80000u), 0xDDCCBBAA);
    teardown(&cpu);
    remove(path);
}

/* ====== Suite runner ====== */

static void run_memory_map_tests(void) {
    TEST_SUITE("Memory Map");
    RUN_TEST(test_rtc_dram_read_write);
    RUN_TEST(test_rtc_dram_iram_alias);
    RUN_TEST(test_rtc_dram_boundary);
    RUN_TEST(test_sram_insn_boundary);
    RUN_TEST(test_rtc_dram_mem_load);
    RUN_TEST(test_firmware_segment_regions);
    RUN_TEST(test_peripheral_spi0_stub);
    RUN_TEST(test_peripheral_syscon_stub);
    RUN_TEST(test_loader_region_names);
    RUN_TEST(test_loader_segment_info);
}
