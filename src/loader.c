#include "loader.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <openssl/md5.h>

/* ---- default partition table for app-only images ------------------------
 * A bare app .bin carries no partition table, but IDF reads one from flash
 * offset 0x8000 during startup (load_partitions, esp_ota_get_running_
 * partition, nvs_flash_init). Synthesize a minimal valid table: factory app
 * at 0x10000 covering the loaded image, plus nvs/phy_init/coredump data
 * partitions, followed by MD5 and CRC32 digests exactly as esptool emits.
 * The nvs/phy_init/coredump flash regions are pre-set to erased (0xFF). */
#define PT_OFFSET        0x8000u
#define PT_FLASH_SIZE    (4u * 1024 * 1024)
#define PT_TYPE_APP      0x00u
#define PT_TYPE_DATA     0x01u
#define PT_SUB_FACTORY   0x00u
#define PT_SUB_PHY       0x01u
#define PT_SUB_NVS       0x02u
#define PT_SUB_COREDUMP  0x03u

static uint32_t pt_crc32_le(uint32_t crc, const uint8_t *buf, size_t len) {
    /* ESP-ROM crc32_le semantics: zlib CRC32 (poly 0xEDB88320, reflected) */
    crc = ~crc;
    for (size_t i = 0; i < len; i++) {
        crc ^= buf[i];
        for (int b = 0; b < 8; b++)
            crc = (crc >> 1) ^ ((crc & 1) ? 0xEDB88320u : 0);
    }
    return ~crc;
}

static void pt_entry(uint8_t *p, uint8_t type, uint8_t subtype,
                     uint32_t offset, uint32_t size, const char *label) {
    p[0] = 0xAA; p[1] = 0x50;               /* magic 0x50AA little-endian */
    p[2] = type; p[3] = subtype;
    memcpy(p + 4, &offset, 4);
    memcpy(p + 8, &size, 4);
    memset(p + 12, 0, 16);
    strncpy((char *)p + 12, label, 15);
    memset(p + 28, 0, 4);                   /* flags */
}

static void loader_synthesize_partition_table(xtensa_mem_t *mem, long app_size) {
    /* Respect a pre-existing valid table (shouldn't happen for app-only
     * images, but cheap insurance against clobbering real data). */
    if (mem->flash_data[PT_OFFSET] == 0xAA && mem->flash_data[PT_OFFSET + 1] == 0x50)
        return;

    uint8_t pt[0x1000];
    memset(pt, 0xFF, sizeof(pt));

    uint32_t factory_size = ((uint32_t)app_size + 0xFFFFu) & ~0xFFFFu;
    uint32_t off = 0x10000u;
    int n = 0;
    pt_entry(pt, PT_TYPE_APP, PT_SUB_FACTORY, off, factory_size, "factory");
    n++;
    off += factory_size;
    uint32_t nvs_off = 0, phy_off = 0, cd_off = 0;
    if (off + 0x6000u <= PT_FLASH_SIZE) {
        nvs_off = off;
        pt_entry(pt + n * 32, PT_TYPE_DATA, PT_SUB_NVS, off, 0x6000u, "nvs");
        n++; off += 0x6000u;
    }
    if (off + 0x1000u <= PT_FLASH_SIZE) {
        phy_off = off;
        pt_entry(pt + n * 32, PT_TYPE_DATA, PT_SUB_PHY, off, 0x1000u, "phy_init");
        n++; off += 0x1000u;
    }
    if (off + 0x10000u <= PT_FLASH_SIZE) {
        cd_off = off;
        pt_entry(pt + n * 32, PT_TYPE_DATA, PT_SUB_COREDUMP, off, 0x10000u, "coredump");
        n++; off += 0x10000u;
    }

    /* MD5 entry: magic 0xEBEB, digest at +16 (ESP_PARTITION_MD5_OFFSET),
     * digest covers the concatenated 32-byte partition entries (as
     * esp_partition's load_partitions computes it). CRC32 follows, over
     * entries + MD5 entry. */
    uint8_t digest[16];
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wdeprecated-declarations"
    MD5_CTX mctx;
    MD5_Init(&mctx);
    MD5_Update(&mctx, pt, (size_t)n * 32);
    MD5_Final(digest, &mctx);
#pragma clang diagnostic pop
    uint8_t *md5e = pt + n * 32;
    md5e[0] = 0xEB; md5e[1] = 0xEB;             /* ESP_PARTITION_MAGIC_MD5 */
    memcpy(md5e + 16, digest, 16);
    uint32_t crc = pt_crc32_le(0, pt, (size_t)n * 32 + 32);
    memcpy(pt + n * 32 + 32, &crc, 4);

    memcpy(mem->flash_data + PT_OFFSET, pt, sizeof(pt));
    memcpy(mem->flash_insn + PT_OFFSET, pt, sizeof(pt));
    if (nvs_off) memset(mem->flash_data + nvs_off, 0xFF, 0x6000u);
    if (phy_off) memset(mem->flash_data + phy_off, 0xFF, 0x1000u);
    if (cd_off)  memset(mem->flash_data + cd_off,  0xFF, 0x10000u);
    if (getenv("FLEXE_PTDBG")) {
        fprintf(stderr, "[PT] synthesized %d entries, factory_size=0x%X crc=0x%08X\n",
                n, factory_size, crc);
        fprintf(stderr, "[PT] flash[0x8000..0x8010]:");
        for (int i = 0; i < 16; i++) fprintf(stderr, " %02X", mem->flash_data[PT_OFFSET + i]);
        fprintf(stderr, "\n");
    }
}


/* Parse an ESP32 image header at the given file offset, loading segments into memory.
 * The image header (24 bytes) is also written to flash_hdr_out if non-NULL. */
static int loader_parse_image(xtensa_mem_t *mem, FILE *f, long offset,
                              load_result_t *res, uint8_t *flash_hdr_out,
                              int skip_drom) {
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

    long data_off = offset + 24;   /* file offset of the current segment header */
    for (int i = 0; i < seg_count; i++) {
        uint8_t seg_hdr[8];
        if (fread(seg_hdr, 1, 8, f) != 8) {
            snprintf(res->error, sizeof(res->error), "Segment %d header truncated", i);
            return -1;
        }
        data_off += 8;

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
            res->segments[i].image_off = (uint32_t)(data_off - offset);
        }

        /* Factory mode: DROM segments live in the whole-flash image already
         * (and the DROM cache window is remapped onto it), so don't load
         * them again at their cache vaddrs — that would clobber the
         * partition table and other sub-0x10000 regions. */
        int is_drom = (load_addr >= 0x3F400000u && load_addr < 0x3F800000u);
        if (!(skip_drom && is_drom) && mem_load(mem, load_addr, buf, data_len) != 0) {
            snprintf(res->error, sizeof(res->error),
                     "Segment %d load failed at 0x%08X (%u bytes, region: %s)",
                     i, load_addr, data_len, loader_region_name(load_addr));
            free(buf);
            return -1;
        }

        free(buf);
        data_off += (long)data_len;
    }

    res->entry_point = entry;
    res->segment_count = seg_count;
    return 0;
}

/* Seed the DPORT flash MMU tables the way the ROM bootloader leaves them:
 * entries covering the app's own rodata/text are used, all other entries
 * stay invalid (0x100 = free) so esp_mmu_map / ROM spi_flash_mmap can find
 * free vaddr slots for partition-table and NVS mappings.
 *
 * Entry layout (IDF mmu_ll_get_entry_id, ESP32): 0-63 = DROM0
 * (0x3F400000+), 64-127 = IRAM0 cache window (0x400D0000+; entry 64 covers
 * vaddr 0x40000000). Entry e maps its 64 KB vaddr page to flash page =
 * value. Flash offsets follow the emulator's layout (app image at flash
 * 0x10000). Correct IROM entries matter for spi_flash_cache2phys(), which
 * esp_ota_get_running_partition() uses to locate the running app partition
 * by its own code address.
 *
 * BOTH the PRO and APP tables must be written: the IDF startup contains a
 * table-consistency sweep (seen at 0x40081D41) that invalidates any entry
 * whose PRO and APP values differ — PRO-only seeding is wiped out by it.
 *
 * The mem_write32 path also remaps the corresponding cache window pages
 * (flash_mmu_map_entry) — for DROM that is identical to the loader's own
 * remap, so no conflict. */
static void loader_seed_flash_mmu(xtensa_mem_t *mem, const load_result_t *res) {
    uint32_t drom_pages = 0;
    for (int i = 0; i < res->segment_count && i < MAX_SEGMENTS; i++) {
        uint32_t a = res->segments[i].addr;
        uint32_t e = a + res->segments[i].size;
        if (a >= 0x3F400000u && a < 0x3F800000u && e > 0x3F400000u) {
            uint32_t pages = (e - 0x3F400000u + 0xFFFFu) / 0x10000u;
            if (pages > drom_pages) drom_pages = pages;
        }
    }
    if (drom_pages == 0) drom_pages = 1;   /* image header at 0x3F400000 */
    if (drom_pages > 64) drom_pages = 64;
    for (uint32_t e = 0; e < drom_pages; e++) {
        mem_write32(mem, 0x3FF10000u + e * 4, e + 1);   /* PRO table */
        mem_write32(mem, 0x3FF12000u + e * 4, e + 1);   /* APP table */
    }

    /* IROM: mark entries covering flash text segments, with true flash
     * pages. Only segments in the flash instruction window (IDF
     * SOC_IRAM0_CACHE: 0x400D0000-0x40400000) are flash-backed — internal
     * IRAM segments (0x40080000 etc.) have no cache mapping on real
     * hardware. Entry e covers vaddr 0x40000000 + (e-64)*64KB; the content
     * at vaddr v sits at flash 0x10000 + image_off + (v - seg_addr). */
    for (int i = 0; i < res->segment_count && i < MAX_SEGMENTS; i++) {
        uint32_t a = res->segments[i].addr;
        uint32_t sz = res->segments[i].size;
        uint32_t foff = res->segments[i].image_off;
        if (a >= 0x400D0000u && a < 0x40400000u && sz) {
            uint32_t first = 64 + ((a & 0x3FFFFFu) >> 16);
            uint32_t last  = 64 + (((a + sz - 1) & 0x3FFFFFu) >> 16);
            for (uint32_t e = first; e <= last && e < 128; e++) {
                uint32_t vbase = 0x40000000u + (e - 64) * 0x10000u;
                uint32_t page = (0x10000u + foff + (vbase - a)) / 0x10000u;
                mem_write32(mem, 0x3FF10000u + e * 4, page);   /* PRO */
                mem_write32(mem, 0x3FF12000u + e * 4, page);   /* APP */
            }
        }
    }
    if (getenv("FLEXE_PTDBG"))
        fprintf(stderr, "[PT] flash MMU: %u DROM entries used, rest free\n", drom_pages);
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

        /* Remap the DROM cache window (0x3F400000-0x3F800000) onto the app
         * at flash offset 0x10000 — this is what the ESP32 flash MMU does
         * for an app flashed at 0x10000. Bootloader and partition table
         * stay at their raw offsets for SPI (esp_flash_read) access. */
        for (uint32_t page = 0x3F400000u; page < 0x3F800000u; page += 4096) {
            uint32_t off = 0x10000u + (page - 0x3F400000u);
            if (off + 4096 <= (4u * 1024 * 1024))
                mem->page_table[page >> 12] = mem->flash_data + off;
        }

        /* Parse the app image at 0x10000 for SRAM segment loading;
         * skip DROM segments (already in the raw image, now remapped) */
        uint8_t hdr[24];
        if (loader_parse_image(mem, f, 0x10000, &res, hdr, 1) != 0) {
            res.result = -1;
            free(flash_buf);
            fclose(f);
            return res;
        }

        /* Re-pristine flash_insn: parsing's mem_load of the IROM segment
         * writes through the vaddr-linear page table into the same backing
         * as the flash image copy, clobbering the overlap. The flash MMU
         * IROM entries (and cache2phys) need a pristine flash image. */
        memcpy(mem->flash_insn, flash_buf, flash_len);
        free(flash_buf);

        /* Write the app header at flash data base so firmware can verify magic */
        mem_load(mem, 0x3F400000u, hdr, 24);

        loader_seed_flash_mmu(mem, &res);

        res.result = 0;
        fclose(f);
        return res;
    }

    /* Standalone app image — lay it out like a factory image:
     * full copy at flash offset 0x10000, DROM cache window remapped onto
     * it, synthesized partition table at 0x8000. IDF validates the running
     * app by reading the image at the partition offset via SPI flash
     * reads, so the image must exist at 0x10000 (not just at the DROM
     * cache vaddrs). */
    fseek(f, 0, SEEK_END);
    long app_size = ftell(f);
    uint8_t *img = NULL;
    if (app_size > 0 && app_size + 0x10000 <= (long)PT_FLASH_SIZE) {
        img = malloc((size_t)app_size);
        if (img) {
            fseek(f, 0, SEEK_SET);
            if (fread(img, 1, (size_t)app_size, f) == (size_t)app_size)
                memcpy(mem->flash_data + 0x10000, img, (size_t)app_size);
        }
        /* Remap the DROM cache window (0x3F400000-0x3F800000) onto the app
         * at flash offset 0x10000 — same as the factory-image path. */
        for (uint32_t page = 0x3F400000u; page < 0x3F800000u; page += 4096) {
            uint32_t off = 0x10000u + (page - 0x3F400000u);
            if (off + 4096 <= PT_FLASH_SIZE)
                mem->page_table[page >> 12] = mem->flash_data + off;
        }
    }

    /* Standalone app image — parse from offset 0 */
    uint8_t hdr[24];
    if (loader_parse_image(mem, f, 0, &res, hdr, 0) != 0) {
        res.result = -1;
        free(img);
        fclose(f);
        return res;
    }

    /* Copy the image into flash_insn only AFTER segment loading: parsing's
     * mem_load of the IROM segment writes through the vaddr-linear page
     * table into this same backing, clobbering the overlap. flash MMU IROM
     * entries (and cache2phys) need a pristine flash image. */
    if (img) {
        memcpy(mem->flash_insn + 0x10000, img, (size_t)app_size);
        free(img);
    }

    /* Write the 24-byte image header at 0x3F400000 so firmware can
     * verify its own magic byte via the flash data cache mapping */
    mem_load(mem, 0x3F400000u, hdr, 24);

    /* Bare app images carry no partition table; synthesize one at 0x8000 */
    if (app_size > 0)
        loader_synthesize_partition_table(mem, app_size);

    loader_seed_flash_mmu(mem, &res);

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
