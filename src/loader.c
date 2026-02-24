#include "loader.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

load_result_t loader_load_bin(xtensa_mem_t *mem, const char *path) {
    load_result_t res = {0};

    if (!path) {
        res.result = -1;
        snprintf(res.error, sizeof(res.error), "NULL path");
        return res;
    }

    if (!mem) {
        res.result = -1;
        snprintf(res.error, sizeof(res.error), "NULL memory");
        return res;
    }

    FILE *f = fopen(path, "rb");
    if (!f) {
        res.result = -1;
        snprintf(res.error, sizeof(res.error), "Cannot open file: %s", path);
        return res;
    }

    /* Read 24-byte header */
    uint8_t hdr[24];
    if (fread(hdr, 1, 24, f) != 24) {
        res.result = -1;
        snprintf(res.error, sizeof(res.error), "Header too short");
        fclose(f);
        return res;
    }

    /* Magic byte */
    if (hdr[0] != 0xE9) {
        res.result = -1;
        snprintf(res.error, sizeof(res.error), "Bad magic: 0x%02X (expected 0xE9)", hdr[0]);
        fclose(f);
        return res;
    }

    /* Segment count */
    int seg_count = hdr[1];
    if (seg_count < 1 || seg_count > 16) {
        res.result = -1;
        snprintf(res.error, sizeof(res.error), "Bad segment count: %d", seg_count);
        fclose(f);
        return res;
    }

    /* Entry point (little-endian at offset 4) */
    uint32_t entry = (uint32_t)hdr[4]
                   | ((uint32_t)hdr[5] << 8)
                   | ((uint32_t)hdr[6] << 16)
                   | ((uint32_t)hdr[7] << 24);

    /* Load segments */
    for (int i = 0; i < seg_count; i++) {
        uint8_t seg_hdr[8];
        if (fread(seg_hdr, 1, 8, f) != 8) {
            res.result = -1;
            snprintf(res.error, sizeof(res.error), "Segment %d header truncated", i);
            fclose(f);
            return res;
        }

        uint32_t load_addr = (uint32_t)seg_hdr[0]
                           | ((uint32_t)seg_hdr[1] << 8)
                           | ((uint32_t)seg_hdr[2] << 16)
                           | ((uint32_t)seg_hdr[3] << 24);
        uint32_t data_len  = (uint32_t)seg_hdr[4]
                           | ((uint32_t)seg_hdr[5] << 8)
                           | ((uint32_t)seg_hdr[6] << 16)
                           | ((uint32_t)seg_hdr[7] << 24);

        if (data_len > 16 * 1024 * 1024) {
            res.result = -1;
            snprintf(res.error, sizeof(res.error), "Segment %d too large: %u", i, data_len);
            fclose(f);
            return res;
        }

        uint8_t *buf = malloc(data_len);
        if (!buf) {
            res.result = -1;
            snprintf(res.error, sizeof(res.error), "Segment %d malloc failed", i);
            fclose(f);
            return res;
        }

        if (fread(buf, 1, data_len, f) != data_len) {
            res.result = -1;
            snprintf(res.error, sizeof(res.error), "Segment %d data truncated", i);
            free(buf);
            fclose(f);
            return res;
        }

        /* Record segment info */
        if (i < MAX_SEGMENTS) {
            res.segments[i].addr = load_addr;
            res.segments[i].size = data_len;
        }

        if (mem_load(mem, load_addr, buf, data_len) != 0) {
            res.result = -1;
            snprintf(res.error, sizeof(res.error),
                     "Segment %d load failed at 0x%08X (%u bytes, region: %s)",
                     i, load_addr, data_len, loader_region_name(load_addr));
            free(buf);
            fclose(f);
            return res;
        }

        free(buf);
    }

    /* Write the 24-byte image header at 0x3F400000 so firmware can
     * verify its own magic byte via the flash data cache mapping */
    mem_load(mem, 0x3F400000u, hdr, 24);

    res.entry_point = entry;
    res.segment_count = seg_count;
    res.result = 0;

    fclose(f);
    return res;
}

const char *loader_region_name(uint32_t addr) {
    if (addr >= 0x3F400000u && addr < 0x3F800000u) return "flash_data";
    if (addr >= 0x3FF00000u && addr < 0x3FF80000u) return "peripheral";
    if (addr >= 0x3FF80000u && addr < 0x3FF82000u) return "rtc_dram";
    if (addr >= 0x3FFB0000u && addr < 0x40000000u) return "sram_data";
    if (addr >= 0x40000000u && addr < 0x40060000u) return "rom";
    if (addr >= 0x40070000u && addr < 0x400C0000u) return "sram_insn";
    if (addr >= 0x400C0000u && addr < 0x400C2000u) return "rtc_iram";
    if (addr >= 0x400C2000u && addr < 0x40C00000u) return "flash_insn";
    if (addr >= 0x50000000u && addr < 0x50002000u) return "rtc_fast";
    if (addr >= 0x60000000u && addr < 0x60002000u) return "rtc_slow";
    return "unmapped";
}
