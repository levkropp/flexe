#include "loader.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <openssl/md5.h>

#define ESP_IMAGE_HEADER_SIZE 24u

static uint16_t read_le16(const uint8_t *p) {
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static int loader_read_image_header(FILE *f, long offset, uint8_t hdr[24],
                                    char *error, size_t error_size) {
    if (fseek(f, offset, SEEK_SET) != 0) {
        snprintf(error, error_size, "Seek to 0x%lX failed", offset);
        return -1;
    }
    if (fread(hdr, 1, ESP_IMAGE_HEADER_SIZE, f) != ESP_IMAGE_HEADER_SIZE) {
        snprintf(error, error_size, "Header too short at 0x%lX", offset);
        return -1;
    }
    if (hdr[0] != 0xE9) {
        snprintf(error, error_size,
                 "Bad magic: 0x%02X at offset 0x%lX (expected 0xE9)",
                 hdr[0], offset);
        return -1;
    }
    return 0;
}

int loader_probe_bin(const char *path, loader_image_info_t *info,
                     char *error, size_t error_size) {
    if (!path || !info || !error || error_size == 0) return -1;
    memset(info, 0, sizeof(*info));
    error[0] = '\0';

    FILE *f = fopen(path, "rb");
    if (!f) {
        snprintf(error, error_size, "Cannot open file: %s", path);
        return -1;
    }

    uint8_t hdr[ESP_IMAGE_HEADER_SIZE];
    long offset = 0;
    if (fread(hdr, 1, 1, f) != 1) {
        snprintf(error, error_size, "File too short");
        fclose(f);
        return -1;
    }
    if (hdr[0] != 0xE9) {
        if (fseek(f, 0, SEEK_END) != 0 || ftell(f) < 0x10000 + 24) {
            snprintf(error, error_size,
                     "Bad magic 0x%02X and file too small for factory image",
                     hdr[0]);
            fclose(f);
            return -1;
        }
        offset = 0x10000;
    }
    if (loader_read_image_header(f, offset, hdr, error, error_size) != 0) {
        if (offset != 0)
            snprintf(error, error_size,
                     "No ESP app image at factory offset 0x10000");
        fclose(f);
        return -1;
    }
    fclose(f);

    uint16_t chip_id = read_le16(&hdr[12]);
    const flexe_target_desc_t *target =
        flexe_target_by_image_chip_id(chip_id);
    if (!target) {
        snprintf(error, error_size,
                 "Unknown ESP image chip ID 0x%04X", chip_id);
        return -1;
    }

    info->target = target->id;
    info->chip_id = chip_id;
    info->min_chip_rev = hdr[14];
    info->min_chip_rev_full = read_le16(&hdr[15]);
    info->max_chip_rev_full = read_le16(&hdr[17]);
    info->image_offset = (uint32_t)offset;
    return 0;
}

/* ---- default partition table for app-only images ------------------------
 * A bare app .bin carries no partition table, but IDF reads one from flash
 * offset 0x8000 during startup (load_partitions, esp_ota_get_running_
 * partition, nvs_flash_init). Synthesize a minimal valid table: factory app
 * at 0x10000 covering the loaded image, plus nvs/phy_init/coredump data
 * partitions, followed by MD5 and CRC32 digests exactly as esptool emits.
 * The nvs/phy_init/coredump flash regions are pre-set to erased (0xFF). */
#define PT_TYPE_APP      0x00u
#define PT_TYPE_DATA     0x01u
#define PT_SUB_FACTORY   0x00u
#define PT_SUB_PHY       0x01u
#define PT_SUB_NVS       0x02u
#define PT_SUB_COREDUMP  0x03u
#define PT_SUB_OTA0      0x10u
#define PT_SUB_OTA1      0x11u
#define PT_SUB_SPIFFS    0x82u

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

static int loader_synthesize_partition_table(xtensa_mem_t *mem, long app_size,
                                             uint32_t entry_point) {
    const flexe_target_desc_t *target = mem_target(mem);
    uint32_t flash_size = mem_backing_size(mem, FLEXE_MEM_FLASH_DATA);
    uint32_t pt_offset = target ? target->partition_table_offset : 0;
    if (!target || pt_offset > flash_size || flash_size - pt_offset < 0x1000u)
        return -1;

    /* Respect a pre-existing valid table (shouldn't happen for app-only
     * images, but cheap insurance against clobbering real data). */
    if (mem->flash_data[pt_offset] == 0xAA &&
        mem->flash_data[pt_offset + 1] == 0x50)
        return 0;

    uint8_t pt[0x1000];
    memset(pt, 0xFF, sizeof(pt));

    uint32_t factory_size = ((uint32_t)app_size + 0xFFFFu) & ~0xFFFFu;
    uint32_t off = target->default_app_offset;
    int n = 0;
    uint32_t nvs_off = 0, nvs_size = 0, phy_off = 0, cd_off = 0;
    if (target->id == FLEXE_TARGET_ESP32 &&
        entry_point == 0x40089268u && app_size <= 0x300000l) {
        /* NerdMiner v1.8.3 ESP32-2432S028R uses Arduino's huge_app.csv.
         * Reconstruct it exactly so its unmodified SPIFFS mount/format path
         * sees the same partition as it does on a flashed CYD. */
        nvs_off = 0x9000u;
        nvs_size = 0x5000u;
        cd_off = 0x3F0000u;
        pt_entry(pt + n++ * 32, PT_TYPE_DATA, PT_SUB_NVS,
                 nvs_off, nvs_size, "nvs");
        pt_entry(pt + n++ * 32, PT_TYPE_DATA, 0x00u,
                 0xE000u, 0x2000u, "otadata");
        pt_entry(pt + n++ * 32, PT_TYPE_APP, PT_SUB_OTA0,
                 0x10000u, 0x300000u, "app0");
        pt_entry(pt + n++ * 32, PT_TYPE_DATA, PT_SUB_SPIFFS,
                 0x310000u, 0x0E0000u, "spiffs");
        pt_entry(pt + n++ * 32, PT_TYPE_DATA, PT_SUB_COREDUMP,
                 cd_off, 0x10000u, "coredump");
    } else if (target->id == FLEXE_TARGET_ESP32 &&
               entry_point == 0x400831D8u && app_size <= 0x1E0000l) {
        /* ESP32 Marauder v1.14 CYD production partition table. */
        nvs_off = 0x9000u;
        nvs_size = 0x5000u;
        cd_off = 0x3F0000u;
        pt_entry(pt + n++ * 32, PT_TYPE_DATA, PT_SUB_NVS,
                 nvs_off, nvs_size, "nvs");
        pt_entry(pt + n++ * 32, PT_TYPE_DATA, 0x00u,
                 0xE000u, 0x2000u, "otadata");
        pt_entry(pt + n++ * 32, PT_TYPE_APP, PT_SUB_OTA0,
                 0x10000u, 0x1E0000u, "app0");
        pt_entry(pt + n++ * 32, PT_TYPE_APP, PT_SUB_OTA1,
                 0x1F0000u, 0x1E0000u, "app1");
        pt_entry(pt + n++ * 32, PT_TYPE_DATA, PT_SUB_SPIFFS,
                 0x3D0000u, 0x20000u, "spiffs");
        pt_entry(pt + n++ * 32, PT_TYPE_DATA, PT_SUB_COREDUMP,
                 cd_off, 0x10000u, "coredump");
    } else {
        pt_entry(pt, PT_TYPE_APP, PT_SUB_FACTORY, off, factory_size,
                 "factory");
        n++;
        off += factory_size;
        if (off <= flash_size && flash_size - off >= 0x6000u) {
            nvs_off = off;
            nvs_size = 0x6000u;
            pt_entry(pt + n * 32, PT_TYPE_DATA, PT_SUB_NVS, off, nvs_size,
                     "nvs");
            n++; off += 0x6000u;
        }
        if (off <= flash_size && flash_size - off >= 0x1000u) {
            phy_off = off;
            pt_entry(pt + n * 32, PT_TYPE_DATA, PT_SUB_PHY, off, 0x1000u,
                     "phy_init");
            n++; off += 0x1000u;
        }
        if (off <= flash_size && flash_size - off >= 0x10000u) {
            cd_off = off;
            pt_entry(pt + n * 32, PT_TYPE_DATA, PT_SUB_COREDUMP, off,
                     0x10000u, "coredump");
            n++; off += 0x10000u;
        }
        /* Whatever flash is left becomes a filesystem partition.
         *
         * Modern firmware assumes one exists and says so when it does not:
         * Meshtastic prints `esp_littlefs: partition "spiffs" could not be
         * found`, openHASP `config partition not found`. Both SPIFFS and
         * LittleFS look it up by the subtype, not the label, so one entry
         * serves either. Aligned to 64 KiB because a filesystem partition has
         * to start on a flash erase-block boundary. */
        off = (off + 0xFFFFu) & ~0xFFFFu;
        if (off <= flash_size && flash_size - off >= 0x10000u) {
            pt_entry(pt + n * 32, PT_TYPE_DATA, PT_SUB_SPIFFS, off,
                     flash_size - off, "spiffs");
            n++;
        }
    }

    /* MD5 entry: magic 0xEBEB, digest at +16 (ESP_PARTITION_MD5_OFFSET),
     * digest covers the concatenated 32-byte partition entries (as
     * esp_partition's load_partitions computes it). CRC32 follows, over
     * entries + MD5 entry. */
    uint8_t digest[16];
/* OpenSSL 3.0 deprecates the low-level MD5_* API. This is a partition-table
 * checksum, not a security primitive, so the legacy calls are fine; silence
 * the deprecation with the "GCC" pragma spelling, which clang also honors. */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
    MD5_CTX mctx;
    MD5_Init(&mctx);
    MD5_Update(&mctx, pt, (size_t)n * 32);
    MD5_Final(digest, &mctx);
#pragma GCC diagnostic pop
    uint8_t *md5e = pt + n * 32;
    md5e[0] = 0xEB; md5e[1] = 0xEB;             /* ESP_PARTITION_MAGIC_MD5 */
    memcpy(md5e + 16, digest, 16);
    uint32_t crc = pt_crc32_le(0, pt, (size_t)n * 32 + 32);
    memcpy(pt + n * 32 + 32, &crc, 4);

    memcpy(mem->flash_data + pt_offset, pt, sizeof(pt));
    memcpy(mem->flash_insn + pt_offset, pt, sizeof(pt));
    if (nvs_off) memset(mem->flash_data + nvs_off, 0xFF, nvs_size);
    if (phy_off) memset(mem->flash_data + phy_off, 0xFF, 0x1000u);
    if (cd_off)  memset(mem->flash_data + cd_off,  0xFF, 0x10000u);
    if (getenv("FLEXE_PTDBG")) {
        fprintf(stderr, "[PT] synthesized %d entries, factory_size=0x%X crc=0x%08X\n",
                n, factory_size, crc);
        fprintf(stderr, "[PT] flash[0x8000..0x8010]:");
        for (int i = 0; i < 16; i++)
            fprintf(stderr, " %02X", mem->flash_data[pt_offset + i]);
        fprintf(stderr, "\n");
    }
    return 0;
}


static int addr_in_range(uint32_t addr, uint32_t start, uint32_t end) {
    return addr >= start && addr < end;
}

static int range_fits(uint32_t addr, uint32_t size,
                      uint32_t start, uint32_t end) {
    return addr >= start && (uint64_t)addr + size <= end;
}

/* Parse an ESP image header at the given file offset, loading internal-memory
 * segments. Flash segments remain in the raw flash image and are exposed by
 * the target's MMU mappings after parsing.
 * The image header (24 bytes) is also written to flash_hdr_out if non-NULL. */
static int loader_parse_image(xtensa_mem_t *mem, FILE *f, long offset,
                              const flexe_target_desc_t *target,
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

        int is_drom = addr_in_range(load_addr, target->drom_start,
                                    target->drom_end);
        int is_irom = addr_in_range(load_addr, target->irom_start,
                                    target->irom_end);
        if ((is_drom && !range_fits(load_addr, data_len, target->drom_start,
                                    target->drom_end)) ||
            (is_irom && !range_fits(load_addr, data_len, target->irom_start,
                                    target->irom_end))) {
            snprintf(res->error, sizeof(res->error),
                     "Segment %d crosses its flash window at 0x%08X (%u bytes)",
                     i, load_addr, data_len);
            free(buf);
            return -1;
        }
        if (!is_drom && !is_irom &&
            mem_load(mem, load_addr, buf, data_len) != 0) {
            snprintf(res->error, sizeof(res->error),
                     "Segment %d load failed at 0x%08X (%u bytes, region: %s)",
                     i, load_addr, data_len,
                     loader_region_name_for_target(target, load_addr));
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
 * Entry layout (IDF mmu_ll_get_entry_id, ESP32): 0-63 = DROM0; 64-127,
 * 128-191, and 192-255 = IRAM0, IRAM1, and IROM0. Entry e maps its 64 KB
 * virtual page to the physical flash page in its value. Flash offsets follow
 * the emulator's layout (app image at flash 0x10000). Correct IRAM0 entries
 * matter for spi_flash_cache2phys(), which esp_ota_get_running_partition()
 * uses to locate the running app partition by its own code address.
 *
 * BOTH the PRO and APP tables must be written: the IDF startup contains a
 * table-consistency sweep (seen at 0x40081D41) that invalidates any entry
 * whose PRO and APP values differ — PRO-only seeding is wiped out by it.
 *
 * The mem_write32 path also remaps the corresponding cache window pages. */
/* The image loader may be used by tools which do not instantiate a complete
 * SoC. Keep its host mappings self-contained, while mirroring bootloader MMU
 * state into a device model when one is present. Skipping an absent handler
 * also keeps loader-only use from looking like guest unmapped-MMIO traffic. */
static void loader_write_mmio_if_modeled(xtensa_mem_t *mem, uint32_t addr,
                                         uint32_t value) {
    const flexe_target_desc_t *target = mem_target(mem);
    if (!target || addr < target->peripheral_start ||
        addr >= target->peripheral_end)
        return;
    uint32_t page = (addr - target->peripheral_start) >> 12;
    if (page < mem->mmio_page_count && mem->mmio[page].write)
        mem_write32(mem, addr, value);
}

static int loader_seed_esp32_flash_mmu(xtensa_mem_t *mem,
                                       const load_result_t *res,
                                       uint32_t app_flash_offset) {
    const flexe_target_desc_t *target = mem_target(mem);
    const flexe_flash_mmu_desc_t *mmu = &target->flash_mmu;
    /* Remove constructor mappings, then install only the bootloader's active
     * entries. Memory's mapping API keeps the loader independent of a SoC
     * device; MMIO writes also seed the firmware-visible DPORT tables when
     * that device model has already been attached. */
    if (mem_unmap_range(mem, target->drom_start,
                        target->drom_end - target->drom_start) != 0 ||
        mem_unmap_range(mem, target->irom_start,
                        target->irom_end - target->irom_start) != 0)
        return -1;
    for (uint32_t e = 0; e < mmu->entry_count; e++) {
        loader_write_mmio_if_modeled(mem, mmu->table_base[0] + e * 4u,
                                     mmu->invalid_entry); /* PRO invalid */
        loader_write_mmio_if_modeled(mem, mmu->table_base[1] + e * 4u,
                                     mmu->invalid_entry); /* APP invalid */
    }

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
        uint32_t physical = e + app_flash_offset / mmu->page_size;
        loader_write_mmio_if_modeled(mem, mmu->table_base[0] + e * 4,
                                     physical); /* PRO */
        loader_write_mmio_if_modeled(mem, mmu->table_base[1] + e * 4,
                                     physical); /* APP */
        if (mem_map_backing_range(mem,
                                  target->drom_start + e * mmu->page_size,
                                  FLEXE_MEM_FLASH_DATA,
                                  physical * mmu->page_size,
                                  mmu->page_size) != 0)
            return -1;
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
                uint32_t page =
                    (app_flash_offset + foff + (vbase - a)) /
                    mmu->page_size;
                loader_write_mmio_if_modeled(
                    mem, mmu->table_base[0] + e * 4, page); /* PRO */
                loader_write_mmio_if_modeled(
                    mem, mmu->table_base[1] + e * 4, page); /* APP */
                if (mem_map_backing_range(mem, vbase,
                                          FLEXE_MEM_FLASH_INSN,
                                          page * mmu->page_size,
                                          mmu->page_size) != 0)
                    return -1;
            }
        }
    }
    if (getenv("FLEXE_PTDBG"))
        fprintf(stderr, "[PT] flash MMU: %u DROM entries used, rest free\n", drom_pages);
    return 0;
}

/* ESP32-S3 has one 512-entry, 64 KiB MMU table shared by the I- and D-cache
 * virtual windows. App-image segment offsets are constrained to have the same
 * low 16 bits as their virtual addresses. Rebuild that table directly in the
 * page map for now; the S3 MMIO table model will make runtime writes use the
 * same operation. */
static int loader_seed_shared_flash_mmu(xtensa_mem_t *mem,
                                        const load_result_t *res,
                                        uint32_t app_flash_offset) {
    const flexe_target_desc_t *target = mem_target(mem);
    const flexe_flash_mmu_desc_t *mmu = &target->flash_mmu;
    uint32_t physical_page[512] = {0};
    uint8_t valid[512] = {0};
    uint32_t flash_data_size =
        mem_backing_size(mem, FLEXE_MEM_FLASH_DATA);
    uint32_t flash_insn_size =
        mem_backing_size(mem, FLEXE_MEM_FLASH_INSN);

    if (!mmu->shared_instruction_data || mmu->page_size == 0 ||
        (mmu->page_size & (mmu->page_size - 1u)) != 0 ||
        mmu->entry_count == 0 || mmu->entry_count > 512 ||
        target->drom_end - target->drom_start !=
            (uint32_t)mmu->entry_count * mmu->page_size ||
        target->irom_end - target->irom_start !=
            (uint32_t)mmu->entry_count * mmu->page_size)
        return -1;

    for (int i = 0; i < res->segment_count && i < MAX_SEGMENTS; i++) {
        uint32_t addr = res->segments[i].addr;
        uint32_t size = res->segments[i].size;
        uint32_t image_off = res->segments[i].image_off;
        int flash_segment =
            addr_in_range(addr, target->drom_start, target->drom_end) ||
            addr_in_range(addr, target->irom_start, target->irom_end);
        if (!flash_segment || size == 0) continue;

        uint64_t physical = (uint64_t)app_flash_offset + image_off;
        uint32_t linear = addr & mmu->linear_addr_mask;
        if ((physical & (mmu->page_size - 1u)) !=
                (linear & (mmu->page_size - 1u)) ||
            (uint64_t)linear + size >
                (uint64_t)mmu->entry_count * mmu->page_size) {
            return -1;
        }

        uint32_t first = linear / mmu->page_size;
        uint32_t last = (linear + size - 1u) / mmu->page_size;
        uint32_t first_physical = (uint32_t)(physical / mmu->page_size);
        for (uint32_t entry = first; entry <= last; entry++) {
            uint32_t page = first_physical + entry - first;
            if (((uint64_t)page + 1u) * mmu->page_size > flash_data_size ||
                ((uint64_t)page + 1u) * mmu->page_size > flash_insn_size ||
                (valid[entry] && physical_page[entry] != page))
                return -1;
            valid[entry] = 1;
            physical_page[entry] = page;
        }
    }

    /* Seed the firmware-visible table as well as the host page map whenever
     * a target MMU device is attached. This is the state a second-stage
     * bootloader hands to the application, and lets later IDF mmap calls
     * inspect and replace the same entries. */
    for (uint32_t entry = 0; entry < mmu->entry_count; entry++)
        loader_write_mmio_if_modeled(mem, mmu->table_base[0] + entry * 4u,
                                     mmu->invalid_entry);

    if (mem_unmap_range(mem, target->drom_start,
                        target->drom_end - target->drom_start) != 0 ||
        mem_unmap_range(mem, target->irom_start,
                        target->irom_end - target->irom_start) != 0)
        return -1;
    for (uint32_t entry = 0; entry < mmu->entry_count; entry++) {
        if (!valid[entry]) continue;
        uint32_t physical = physical_page[entry] * mmu->page_size;
        loader_write_mmio_if_modeled(mem,
                                     mmu->table_base[0] + entry * 4u,
                                     physical_page[entry]);
        if (mem_map_backing_range(
                mem, target->drom_start + entry * mmu->page_size,
                FLEXE_MEM_FLASH_DATA, physical, mmu->page_size) != 0 ||
            mem_map_backing_range(
                mem, target->irom_start + entry * mmu->page_size,
                FLEXE_MEM_FLASH_INSN, physical, mmu->page_size) != 0)
            return -1;
    }
    return 0;
}

static int loader_seed_flash_mmu(xtensa_mem_t *mem,
                                 const load_result_t *res,
                                 uint32_t app_flash_offset) {
    const flexe_target_desc_t *target = mem_target(mem);
    if (target->flash_mmu.shared_instruction_data)
        return loader_seed_shared_flash_mmu(mem, res, app_flash_offset);
    return loader_seed_esp32_flash_mmu(mem, res, app_flash_offset);
}

load_result_t loader_load_bin_for_target(xtensa_mem_t *mem, const char *path,
                                         flexe_target_id_t expected_target) {
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

    if (loader_probe_bin(path, &res.image, res.error, sizeof(res.error)) != 0) {
        res.result = -1;
        return res;
    }
    const flexe_target_desc_t *detected = flexe_target_by_id(res.image.target);
    if (!detected) {
        res.result = -1;
        snprintf(res.error, sizeof(res.error),
                 "No descriptor for detected target %d", res.image.target);
        return res;
    }
    if (expected_target != FLEXE_TARGET_AUTO &&
        expected_target != detected->id) {
        const flexe_target_desc_t *expected =
            flexe_target_by_id(expected_target);
        res.result = -1;
        snprintf(res.error, sizeof(res.error),
                 "Image targets %s (chip ID 0x%04X), not requested target %s",
                 detected->display_name, detected->image_chip_id,
                 expected ? expected->display_name : "unknown");
        return res;
    }
    const flexe_target_desc_t *memory_target = mem_target(mem);
    if (!memory_target || memory_target->id != detected->id) {
        res.result = -1;
        snprintf(res.error, sizeof(res.error),
                 "Image targets %s (chip ID 0x%04X), but memory is "
                 "configured for %s",
                 detected->display_name, detected->image_chip_id,
                 memory_target ? memory_target->display_name : "no target");
        return res;
    }

    uint32_t flash_capacity =
        mem_backing_size(mem, FLEXE_MEM_FLASH_DATA);
    if (mem_backing_size(mem, FLEXE_MEM_FLASH_INSN) < flash_capacity)
        flash_capacity = mem_backing_size(mem, FLEXE_MEM_FLASH_INSN);
    if (flash_capacity == 0 || !mem->flash_data || !mem->flash_insn) {
        res.result = -1;
        snprintf(res.error, sizeof(res.error),
                 "%s memory has no flash backing", detected->display_name);
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
        if (fseek(f, 0, SEEK_END) != 0) {
            res.result = -1;
            snprintf(res.error, sizeof(res.error), "Cannot size factory image");
            fclose(f);
            return res;
        }
        long file_size = ftell(f);

        if (file_size < 0x10000 + 24) {
            res.result = -1;
            snprintf(res.error, sizeof(res.error),
                     "Bad magic 0x%02X and file too small for factory image", magic);
            fclose(f);
            return res;
        }
        if ((uint64_t)file_size > flash_capacity) {
            res.result = -1;
            snprintf(res.error, sizeof(res.error),
                     "Factory image is %ld bytes, larger than %s flash "
                     "capacity %u", file_size, detected->display_name,
                     flash_capacity);
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

        if (mem_load_flash(mem, flash_buf, flash_len) != 0) {
            res.result = -1;
            snprintf(res.error, sizeof(res.error), "Flash image load failed");
            free(flash_buf);
            fclose(f);
            return res;
        }
        free(flash_buf);

        /* Flash segments already live in the raw image. Parse only copies
         * internal-memory segments, then the MMU exposes flash segments. */
        uint32_t app_flash_offset = res.image.image_offset;
        if (loader_parse_image(mem, f, (long)app_flash_offset, detected,
                               &res, NULL) != 0) {
            res.result = -1;
            fclose(f);
            return res;
        }
        if (loader_seed_flash_mmu(mem, &res, app_flash_offset) != 0) {
            res.result = -1;
            snprintf(res.error, sizeof(res.error),
                     "%s flash MMU mapping rejected the image layout",
                     detected->display_name);
            fclose(f);
            return res;
        }

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
    if (fseek(f, 0, SEEK_END) != 0) {
        res.result = -1;
        snprintf(res.error, sizeof(res.error), "Cannot size app image");
        fclose(f);
        return res;
    }
    long app_size = ftell(f);
    uint32_t app_flash_offset = detected->default_app_offset;
    if (app_size <= 0 || app_flash_offset > flash_capacity ||
        (uint64_t)app_size > flash_capacity - app_flash_offset) {
        res.result = -1;
        snprintf(res.error, sizeof(res.error),
                 "App image size %ld does not fit %s flash at offset 0x%X",
                 app_size, detected->display_name, app_flash_offset);
        fclose(f);
        return res;
    }
    uint8_t *img = malloc((size_t)app_size);
    if (!img) {
        res.result = -1;
        snprintf(res.error, sizeof(res.error),
                 "App image malloc failed (%ld bytes)", app_size);
        fclose(f);
        return res;
    }
    if (fseek(f, 0, SEEK_SET) != 0 ||
        fread(img, 1, (size_t)app_size, f) != (size_t)app_size) {
        res.result = -1;
        snprintf(res.error, sizeof(res.error), "App image read truncated");
        free(img);
        fclose(f);
        return res;
    }
    memcpy(mem->flash_data + app_flash_offset, img, (size_t)app_size);
    memcpy(mem->flash_insn + app_flash_offset, img, (size_t)app_size);
    free(img);

    /* Standalone app image — parse from offset 0 */
    if (loader_parse_image(mem, f, 0, detected, &res, NULL) != 0) {
        res.result = -1;
        fclose(f);
        return res;
    }

    /* Bare app images carry no partition table; synthesize one at 0x8000 */
    if (loader_synthesize_partition_table(mem, app_size,
                                          res.entry_point) != 0) {
        res.result = -1;
        snprintf(res.error, sizeof(res.error),
                 "Cannot synthesize %s partition table",
                 detected->display_name);
        fclose(f);
        return res;
    }

    if (loader_seed_flash_mmu(mem, &res, app_flash_offset) != 0) {
        res.result = -1;
        snprintf(res.error, sizeof(res.error),
                 "%s flash MMU mapping rejected the image layout",
                 detected->display_name);
        fclose(f);
        return res;
    }

    res.result = 0;
    fclose(f);
    return res;
}

load_result_t loader_load_bin(xtensa_mem_t *mem, const char *path) {
    return loader_load_bin_for_target(mem, path, FLEXE_TARGET_AUTO);
}

const char *loader_region_name_for_target(const flexe_target_desc_t *target,
                                          uint32_t addr) {
    if (!target) return "unmapped";
    if (addr_in_range(addr, target->drom_start, target->drom_end))
        return "flash_data";
    if (addr_in_range(addr, target->peripheral_start,
                      target->peripheral_end) ||
        addr_in_range(addr, target->peripheral_alias_start,
                      target->peripheral_alias_end))
        return "peripheral";
    for (unsigned i = 0; i < target->memory_region_count; i++) {
        const flexe_target_mem_region_t *region = &target->memory_region[i];
        if (addr_in_range(addr, region->start, region->end))
            return region->name;
    }
    /* Classic ESP32 can map flash into upper instruction buses which app
     * image segments do not normally occupy. Keep diagnostics accurate for
     * those runtime MMU mappings without widening the loader's image range. */
    if (addr >= target->irom_start &&
        flexe_target_pc_is_executable(target, addr))
        return "flash_insn";
    return "unmapped";
}

const char *loader_region_name(uint32_t addr) {
    return loader_region_name_for_target(
        flexe_target_by_id(FLEXE_TARGET_ESP32), addr);
}
