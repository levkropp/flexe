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

        if (mem_load(mem, load_addr, buf, data_len) != 0) {
            res.result = -1;
            snprintf(res.error, sizeof(res.error), "Segment %d load failed at 0x%08X", i, load_addr);
            free(buf);
            fclose(f);
            return res;
        }

        free(buf);
    }

    res.entry_point = entry;
    res.segment_count = seg_count;
    res.result = 0;

    fclose(f);
    return res;
}
