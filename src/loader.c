#include "loader.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

/* Parse an ESP32 image header at the given file offset, loading segments into memory.
 * The image header (24 bytes) is also written to flash_hdr_out if non-NULL. */
static int loader_parse_image(xtensa_mem_t *mem, FILE *f, long offset,
                              load_result_t *res, uint8_t *flash_hdr_out) {
    if (fseek(f, offset, SEEK_SET) != 0) {
        snprintf(res->error, sizeof(res->error), "Seek to 0x%lX failed", offset);
        return -1;
    }

    uint8_t hdr[24];
    if (fread(hdr, 1, 24, f) != 24) {
        snprintf(res->error, sizeof(res->error), "Header too short at 0x%lX", offset);
        return -1;
    }

    if (hdr[0] != 0xE9) {
        snprintf(res->error, sizeof(res->error),
                 "Bad magic: 0x%02X at offset 0x%lX (expected 0xE9)", hdr[0], offset);
        return -1;
    }

    if (flash_hdr_out)
        memcpy(flash_hdr_out, hdr, 24);

    int seg_count = hdr[1];
    if (seg_count < 1 || seg_count > 16) {
        snprintf(res->error, sizeof(res->error), "Bad segment count: %d", seg_count);
        return -1;
    }

    uint32_t entry = (uint32_t)hdr[4]
                   | ((uint32_t)hdr[5] << 8)
                   | ((uint32_t)hdr[6] << 16)
                   | ((uint32_t)hdr[7] << 24);

    for (int i = 0; i < seg_count; i++) {
        uint8_t seg_hdr[8];
        if (fread(seg_hdr, 1, 8, f) != 8) {
            snprintf(res->error, sizeof(res->error), "Segment %d header truncated", i);
            return -1;
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
            snprintf(res->error, sizeof(res->error), "Segment %d too large: %u", i, data_len);
            return -1;
        }

        uint8_t *buf = malloc(data_len);
        if (!buf) {
            snprintf(res->error, sizeof(res->error), "Segment %d malloc failed", i);
            return -1;
        }

        if (fread(buf, 1, data_len, f) != data_len) {
            snprintf(res->error, sizeof(res->error), "Segment %d data truncated", i);
            free(buf);
            return -1;
        }

        if (i < MAX_SEGMENTS) {
            res->segments[i].addr = load_addr;
            res->segments[i].size = data_len;
        }

        if (mem_load(mem, load_addr, buf, data_len) != 0) {
            snprintf(res->error, sizeof(res->error),
                     "Segment %d load failed at 0x%08X (%u bytes, region: %s)",
                     i, load_addr, data_len, loader_region_name(load_addr));
            free(buf);
            return -1;
        }

        free(buf);
    }

    res->entry_point = entry;
    res->segment_count = seg_count;
    return 0;
}

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

    /* Read first byte to detect factory image vs app-only */
    uint8_t magic;
    if (fread(&magic, 1, 1, f) != 1) {
        res.result = -1;
        snprintf(res.error, sizeof(res.error), "File too short");
        fclose(f);
        return res;
    }

    if (magic != 0xE9) {
        /* Not a standalone app image — check if this is a factory (merged flash)
         * image with bootloader at 0x1000 and app at 0x10000 */
        fseek(f, 0, SEEK_END);
        long file_size = ftell(f);

        if (file_size < 0x10000 + 24) {
            res.result = -1;
            snprintf(res.error, sizeof(res.error),
                     "Bad magic 0x%02X and file too small for factory image", magic);
            fclose(f);
            return res;
        }

        /* Check for app header at 0x10000 */
        fseek(f, 0x10000, SEEK_SET);
        uint8_t app_magic;
        if (fread(&app_magic, 1, 1, f) != 1 || app_magic != 0xE9) {
            res.result = -1;
            snprintf(res.error, sizeof(res.error),
                     "Bad magic: 0x%02X (expected 0xE9), no app at 0x10000 either", magic);
            fclose(f);
            return res;
        }

        /* Factory image detected — load entire flash image into flash memory */
        size_t flash_len = (size_t)file_size;
        uint8_t *flash_buf = malloc(flash_len);
        if (!flash_buf) {
            res.result = -1;
            snprintf(res.error, sizeof(res.error), "Flash image malloc failed (%zu bytes)", flash_len);
            fclose(f);
            return res;
        }

        fseek(f, 0, SEEK_SET);
        if (fread(flash_buf, 1, flash_len, f) != flash_len) {
            res.result = -1;
            snprintf(res.error, sizeof(res.error), "Flash image read truncated");
            free(flash_buf);
            fclose(f);
            return res;
        }

        mem_load_flash(mem, flash_buf, flash_len);
        free(flash_buf);

        /* Parse the app image at 0x10000 for SRAM segment loading */
        uint8_t hdr[24];
        if (loader_parse_image(mem, f, 0x10000, &res, hdr) != 0) {
            res.result = -1;
            fclose(f);
            return res;
        }

        /* Write the app header at flash data base so firmware can verify magic */
        mem_load(mem, 0x3F400000u, hdr, 24);

        res.result = 0;
        fclose(f);
        return res;
    }

    /* Standalone app image — parse from offset 0 */
    uint8_t hdr[24];
    if (loader_parse_image(mem, f, 0, &res, hdr) != 0) {
        res.result = -1;
        fclose(f);
        return res;
    }

    /* Write the 24-byte image header at 0x3F400000 so firmware can
     * verify its own magic byte via the flash data cache mapping */
    mem_load(mem, 0x3F400000u, hdr, 24);

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
