/*
 * Tests for the ESP32 .bin loader.
 */
#include "test_helpers.h"
#include "loader.h"
#include <string.h>
#include <stdio.h>

/* Helper: write a little-endian uint32_t to a buffer */
static void put_le32(uint8_t *buf, uint32_t val) {
    buf[0] = (uint8_t)(val);
    buf[1] = (uint8_t)(val >> 8);
    buf[2] = (uint8_t)(val >> 16);
    buf[3] = (uint8_t)(val >> 24);
}

/* Helper: create a temp file with given contents, return path */
static const char *write_temp(const uint8_t *data, size_t len) {
    static char path[256];
    snprintf(path, sizeof(path), "/tmp/xt_test_loader.bin");
    FILE *f = fopen(path, "wb");
    if (!f) return NULL;
    fwrite(data, 1, len, f);
    fclose(f);
    return path;
}

TEST(loader_single_segment) {
    /* Build a minimal .bin: 24-byte header + 1 segment */
    uint8_t bin[64];
    memset(bin, 0, sizeof(bin));
    bin[0] = 0xE9;         /* magic */
    bin[1] = 1;            /* 1 segment */
    put_le32(&bin[4], 0x40080000); /* entry point */
    /* Segment at offset 24: load_addr, data_len, data */
    put_le32(&bin[24], 0x3FFB0000);  /* load to SRAM data */
    put_le32(&bin[28], 4);           /* 4 bytes of data */
    put_le32(&bin[32], 0xDEADBEEF);  /* the data */

    const char *path = write_temp(bin, 36);
    ASSERT_TRUE(path != NULL);

    xtensa_mem_t *mem = mem_create();
    load_result_t res = loader_load_bin(mem, path);
    ASSERT_EQ(res.result, 0);
    ASSERT_EQ(res.entry_point, 0x40080000);
    ASSERT_EQ(res.segment_count, 1);
    ASSERT_EQ(mem_read32(mem, 0x3FFB0000), 0xDEADBEEF);
    mem_destroy(mem);
}

TEST(loader_multi_segment) {
    uint8_t bin[128];
    memset(bin, 0, sizeof(bin));
    bin[0] = 0xE9;
    bin[1] = 2;            /* 2 segments */
    put_le32(&bin[4], 0x40080000);

    /* Segment 0 */
    put_le32(&bin[24], 0x3FFB0000);
    put_le32(&bin[28], 4);
    put_le32(&bin[32], 0x11111111);

    /* Segment 1 */
    put_le32(&bin[36], 0x3FFB0100);
    put_le32(&bin[40], 4);
    put_le32(&bin[44], 0x22222222);

    const char *path = write_temp(bin, 48);
    ASSERT_TRUE(path != NULL);

    xtensa_mem_t *mem = mem_create();
    load_result_t res = loader_load_bin(mem, path);
    ASSERT_EQ(res.result, 0);
    ASSERT_EQ(res.segment_count, 2);
    ASSERT_EQ(mem_read32(mem, 0x3FFB0000), 0x11111111);
    ASSERT_EQ(mem_read32(mem, 0x3FFB0100), 0x22222222);
    mem_destroy(mem);
}

TEST(loader_bad_magic) {
    uint8_t bin[32];
    memset(bin, 0, sizeof(bin));
    bin[0] = 0x00; /* wrong magic */
    bin[1] = 1;
    const char *path = write_temp(bin, 32);
    ASSERT_TRUE(path != NULL);

    xtensa_mem_t *mem = mem_create();
    load_result_t res = loader_load_bin(mem, path);
    ASSERT_EQ(res.result, -1);
    mem_destroy(mem);
}

TEST(loader_null_path) {
    xtensa_mem_t *mem = mem_create();
    load_result_t res = loader_load_bin(mem, NULL);
    ASSERT_EQ(res.result, -1);
    mem_destroy(mem);
}

TEST(loader_null_mem) {
    load_result_t res = loader_load_bin(NULL, "/tmp/whatever");
    ASSERT_EQ(res.result, -1);
}

void run_loader_tests(void) {
    TEST_SUITE("ESP32 .bin Loader");

    RUN_TEST(loader_single_segment);
    RUN_TEST(loader_multi_segment);
    RUN_TEST(loader_bad_magic);
    RUN_TEST(loader_null_path);
    RUN_TEST(loader_null_mem);
}
