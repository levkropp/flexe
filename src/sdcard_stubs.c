#include "sdcard_stubs.h"
#include "rom_stubs.h"
#include "memory.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#define DEFAULT_SD_SIZE (4ULL * 1024 * 1024 * 1024)  /* 4 GB */

struct sdcard_stubs {
    xtensa_cpu_t       *cpu;
    esp32_rom_stubs_t  *rom;
    FILE               *img_file;
    uint64_t            img_size;
    uint64_t            requested_size;   /* 0 = auto-detect from file */
    const char         *img_path;
    uint32_t            card_ptr;         /* emulator address of sdmmc_card_t */
};

/* ===== Calling convention helpers ===== */

static uint32_t sd_arg(xtensa_cpu_t *cpu, int n) {
    int ci = XT_PS_CALLINC(cpu->ps);
    return ar_read(cpu, ci * 4 + 2 + n);
}

static void sd_return(xtensa_cpu_t *cpu, uint32_t retval) {
    int ci = XT_PS_CALLINC(cpu->ps);
    if (ci > 0) {
        ar_write(cpu, ci * 4 + 2, retval);
        uint32_t a0 = ar_read(cpu, ci * 4);
        cpu->pc = (cpu->pc & 0xC0000000u) | (a0 & 0x3FFFFFFFu);
        XT_PS_SET_CALLINC(cpu->ps, 0);
    } else {
        ar_write(cpu, 2, retval);
        cpu->pc = ar_read(cpu, 0);
    }
}

static void sd_return64(xtensa_cpu_t *cpu, uint64_t retval) {
    int ci = XT_PS_CALLINC(cpu->ps);
    if (ci > 0) {
        ar_write(cpu, ci * 4 + 2, (uint32_t)retval);
        ar_write(cpu, ci * 4 + 3, (uint32_t)(retval >> 32));
        uint32_t a0 = ar_read(cpu, ci * 4);
        cpu->pc = (cpu->pc & 0xC0000000u) | (a0 & 0x3FFFFFFFu);
        XT_PS_SET_CALLINC(cpu->ps, 0);
    } else {
        ar_write(cpu, 2, (uint32_t)retval);
        ar_write(cpu, 3, (uint32_t)(retval >> 32));
        cpu->pc = ar_read(cpu, 0);
    }
}

static void sd_return_void(xtensa_cpu_t *cpu) {
    int ci = XT_PS_CALLINC(cpu->ps);
    if (ci > 0) {
        uint32_t a0 = ar_read(cpu, ci * 4);
        cpu->pc = (cpu->pc & 0xC0000000u) | (a0 & 0x3FFFFFFFu);
        XT_PS_SET_CALLINC(cpu->ps, 0);
    } else {
        cpu->pc = ar_read(cpu, 0);
    }
}

/* Open image file if not already open (lazy init for SDMMC hooks) */
static int ensure_image_open(sdcard_stubs_t *ss) {
    if (ss->img_file) return 0;
    if (!ss->img_path) return -1;
    ss->img_file = fopen(ss->img_path, "r+b");
    if (!ss->img_file)
        ss->img_file = fopen(ss->img_path, "w+b");
    if (!ss->img_file) return -1;
    fseek(ss->img_file, 0, SEEK_END);
    ss->img_size = (uint64_t)ftell(ss->img_file);
    uint64_t target = ss->requested_size ? ss->requested_size : DEFAULT_SD_SIZE;
    if (ss->img_size < target) {
        if (ftruncate(fileno(ss->img_file), (off_t)target) == 0)
            ss->img_size = target;
    }
    return 0;
}

/* ===== SD card stub implementations ===== */

/* sdcard_init() -> 0 on success, -1 on failure */
static void stub_sdcard_init(xtensa_cpu_t *cpu, void *ctx) {
    sdcard_stubs_t *ss = ctx;
    int rc = ensure_image_open(ss);
    fprintf(stderr, "[SD] sdcard_init() path=%s rc=%d img_size=%llu\n",
            ss->img_path ? ss->img_path : "(null)", rc,
            (unsigned long long)ss->img_size);
    sd_return(cpu, rc == 0 ? 0 : (uint32_t)-1);
}

/* sdcard_deinit() */
static void stub_sdcard_deinit(xtensa_cpu_t *cpu, void *ctx) {
    sdcard_stubs_t *ss = ctx;
    if (ss->img_file) {
        fclose(ss->img_file);
        ss->img_file = NULL;
    }
    sd_return_void(cpu);
}

/* sdcard_size() -> uint64_t in (a2,a3) */
static void stub_sdcard_size(xtensa_cpu_t *cpu, void *ctx) {
    sdcard_stubs_t *ss = ctx;
    fprintf(stderr, "[SD] sdcard_size() -> %llu\n", (unsigned long long)ss->img_size);
    sd_return64(cpu, ss->img_size);
}

/* sdcard_sector_size() -> 512 */
static void stub_sdcard_sector_size(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    sd_return(cpu, 512);
}

/* sdcard_read(lba, count, data) -> 0 on success */
static void stub_sdcard_read(xtensa_cpu_t *cpu, void *ctx) {
    sdcard_stubs_t *ss = ctx;
    uint32_t lba   = sd_arg(cpu, 0);
    uint32_t count = sd_arg(cpu, 1);
    uint32_t data  = sd_arg(cpu, 2);

    fprintf(stderr, "[SD] sdcard_read(lba=%u, count=%u, dst=0x%08X)\n", lba, count, data);
    if (!ss->img_file) { fprintf(stderr, "[SD]   -> FAIL (no image)\n"); sd_return(cpu, (uint32_t)-1); return; }

    uint64_t offset = (uint64_t)lba * 512;
    uint8_t buf[512];

    fseek(ss->img_file, (long)offset, SEEK_SET);
    for (uint32_t i = 0; i < count; i++) {
        size_t got = fread(buf, 1, 512, ss->img_file);
        if (got < 512)
            memset(buf + got, 0, 512 - got);
        for (int j = 0; j < 512; j++)
            mem_write8(cpu->mem, data + i * 512 + (uint32_t)j, buf[j]);
        /* Verify first read: dump first 16 bytes */
        if (lba == 0 && i == 0) {
            fprintf(stderr, "[SD]   sector0: %02X %02X %02X %02X %02X %02X %02X %02X"
                    " %02X %02X %02X %02X %02X %02X %02X %02X\n",
                    buf[0],buf[1],buf[2],buf[3],buf[4],buf[5],buf[6],buf[7],
                    buf[8],buf[9],buf[10],buf[11],buf[12],buf[13],buf[14],buf[15]);
        }
    }
    sd_return(cpu, 0);
}

/* sdcard_write(lba, count, data) -> 0 on success */
static void stub_sdcard_write(xtensa_cpu_t *cpu, void *ctx) {
    sdcard_stubs_t *ss = ctx;
    uint32_t lba   = sd_arg(cpu, 0);
    uint32_t count = sd_arg(cpu, 1);
    uint32_t data  = sd_arg(cpu, 2);

    fprintf(stderr, "[SD] sdcard_write(lba=%u, count=%u, src=0x%08X)\n", lba, count, data);
    if (!ss->img_file) { fprintf(stderr, "[SD]   -> FAIL (no image)\n"); sd_return(cpu, (uint32_t)-1); return; }

    uint64_t offset = (uint64_t)lba * 512;
    uint8_t buf[512];

    fseek(ss->img_file, (long)offset, SEEK_SET);
    for (uint32_t i = 0; i < count; i++) {
        for (int j = 0; j < 512; j++)
            buf[j] = mem_read8(cpu->mem, data + i * 512 + (uint32_t)j);
        fwrite(buf, 1, 512, ss->img_file);
    }
    fflush(ss->img_file);
    sd_return(cpu, 0);
}

/* ===== SDMMC-level stubs (for ESP-IDF filesystem path) ===== */

/* sdmmc_card_init(sdmmc_card_t *card) -> ESP_OK
 * The card struct is large and version-dependent. We populate the
 * minimum fields needed: csd.capacity and csd.sector_size.
 * We find them by searching for where sdmmc_read_sectors checks capacity. */
static void stub_sdmmc_card_init(xtensa_cpu_t *cpu, void *ctx) {
    sdcard_stubs_t *ss = ctx;
    ensure_image_open(ss);
    uint32_t card = sd_arg(cpu, 0);
    fprintf(stderr, "[SD] sdmmc_card_init(card=0x%08X) img_size=%llu\n",
            card, (unsigned long long)ss->img_size);
    if (card && ss->img_file) {
        /* Write capacity info into the card struct.
         * sdmmc_card_t layout (ESP-IDF v4.4):
         *   offset 0:   sdmmc_host_t host (varies, ~72 bytes)
         *   offset 72:  uint32_t ocr
         *   offset 76:  sdmmc_cid_t cid (varies, ~16 bytes)
         *   offset 92:  sdmmc_csd_t csd
         * sdmmc_csd_t: { int csd_ver; int mmc_ver; int capacity; int sector_size;
         *                 int read_block_len; int card_command_class; int tr_speed; }
         * capacity at csd+8, sector_size at csd+12
         *
         * Rather than guessing the exact host struct size, we scan for
         * a known pattern. For safety, just write the CSD at several
         * candidate offsets that cover ESP-IDF v4.x and v5.x layouts.
         * The firmware's sdmmc_read_sectors will read capacity from one of them.
         */
        uint32_t sectors = (uint32_t)(ss->img_size / 512);
        /* sdmmc_csd_t layout: { csd_ver, mmc_ver, capacity, sector_size, ... }
         * capacity at csd+8, sector_size at csd+12.
         * The CSD offset within sdmmc_card_t varies by ESP-IDF version
         * (depends on sdmmc_host_t size). Write at candidate offsets. */
        static const int csd_offsets[] = { 92, 100, 108, 116, 124, 128, 132 };
        for (int i = 0; i < 7; i++) {
            int off = csd_offsets[i];
            mem_write32(cpu->mem, card + (uint32_t)off + 8, sectors);  /* capacity */
            mem_write32(cpu->mem, card + (uint32_t)off + 12, 512);    /* sector_size */
        }
        /* Also set is_mem flag (typically after ext_csd, around offset 160+) */
        /* Set a basic max_freq_khz so the driver doesn't bail */
        ss->card_ptr = card;
    }
    sd_return(cpu, 0);  /* ESP_OK */
}

/* sdmmc_read_sectors(card, dst, start_sector, sector_count) -> ESP_OK */
static void stub_sdmmc_read_sectors(xtensa_cpu_t *cpu, void *ctx) {
    sdcard_stubs_t *ss = ctx;
    /* arg0 = card (ignored), arg1 = dst, arg2 = start_sector, arg3 = count */
    uint32_t dst   = sd_arg(cpu, 1);
    uint32_t start = sd_arg(cpu, 2);
    uint32_t count = sd_arg(cpu, 3);

    fprintf(stderr, "[SD] sdmmc_read_sectors(start=%u, count=%u, dst=0x%08X)\n", start, count, dst);
    if (!ss->img_file) { fprintf(stderr, "[SD]   -> FAIL (no image)\n"); sd_return(cpu, 0x102); return; }  /* ESP_ERR_INVALID_STATE */

    uint64_t offset = (uint64_t)start * 512;
    uint8_t buf[512];

    fseek(ss->img_file, (long)offset, SEEK_SET);
    for (uint32_t i = 0; i < count; i++) {
        size_t got = fread(buf, 1, 512, ss->img_file);
        if (got < 512)
            memset(buf + got, 0, 512 - got);
        for (int j = 0; j < 512; j++)
            mem_write8(cpu->mem, dst + i * 512 + (uint32_t)j, buf[j]);
    }
    sd_return(cpu, 0);
}

/* sdmmc_write_sectors(card, src, start_sector, sector_count) -> ESP_OK */
static void stub_sdmmc_write_sectors(xtensa_cpu_t *cpu, void *ctx) {
    sdcard_stubs_t *ss = ctx;
    uint32_t src   = sd_arg(cpu, 1);
    uint32_t start = sd_arg(cpu, 2);
    uint32_t count = sd_arg(cpu, 3);

    fprintf(stderr, "[SD] sdmmc_write_sectors(start=%u, count=%u, src=0x%08X)\n", start, count, src);
    if (!ss->img_file) { fprintf(stderr, "[SD]   -> FAIL (no image)\n"); sd_return(cpu, 0x102); return; }

    uint64_t offset = (uint64_t)start * 512;
    uint8_t buf[512];

    fseek(ss->img_file, (long)offset, SEEK_SET);
    for (uint32_t i = 0; i < count; i++) {
        for (int j = 0; j < 512; j++)
            buf[j] = mem_read8(cpu->mem, src + i * 512 + (uint32_t)j);
        fwrite(buf, 1, 512, ss->img_file);
    }
    fflush(ss->img_file);
    sd_return(cpu, 0);
}

/* sdspi_host_init_device(config, host, card_handle_out) -> ESP_OK */
static void stub_sdspi_host_init_device(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    /* Write a fake card handle to the output pointer if provided */
    uint32_t handle_out = sd_arg(cpu, 2);
    if (handle_out)
        mem_write32(cpu->mem, handle_out, 1);
    sd_return(cpu, 0);
}

/* ===== Native FAT32/exFAT volume info (bypass emulated scan) ===== */

/* Read a 512-byte sector from the image file into buf.
 * Returns 0 on success, -1 on failure. */
static int img_read_sector(sdcard_stubs_t *ss, uint32_t lba, uint8_t *buf) {
    if (!ss->img_file) return -1;
    if (fseek(ss->img_file, (long)lba * 512, SEEK_SET) != 0) return -1;
    size_t got = fread(buf, 1, 512, ss->img_file);
    if (got < 512) memset(buf + got, 0, 512 - got);
    return 0;
}

/*
 * fat32_volume_info(uint64_t *total_bytes, uint64_t *free_bytes) -> int
 *
 * The firmware's version scans the entire FAT table on the emulated CPU,
 * which is extremely slow (~seconds). We replace it by scanning the FAT
 * natively from the host image file, which takes milliseconds.
 */
static void stub_fat32_volume_info(xtensa_cpu_t *cpu, void *ctx) {
    sdcard_stubs_t *ss = ctx;
    uint32_t total_ptr = sd_arg(cpu, 0);
    uint32_t free_ptr  = sd_arg(cpu, 1);

    if (!ss->img_file) { sd_return(cpu, (uint32_t)-1); return; }

    /* Find partition start (same logic as firmware's find_partition) */
    uint8_t sec[512];
    uint32_t part_start = 0;
    if (img_read_sector(ss, 0, sec) == 0 && sec[510] == 0x55 && sec[511] == 0xAA) {
        uint8_t ptype = sec[446 + 4];
        uint32_t start_lba = sec[446+8] | (sec[446+9]<<8) |
                             (sec[446+10]<<16) | (sec[446+11]<<24);
        if (ptype == 0x0B || ptype == 0x0C || ptype == 0x07)
            part_start = start_lba;
        /* GPT or other: leave at 0 (superfloppy) */
    }

    /* Read BPB */
    if (img_read_sector(ss, part_start, sec) != 0 ||
        sec[510] != 0x55 || sec[511] != 0xAA) {
        sd_return(cpu, (uint32_t)-1);
        return;
    }

    uint32_t spc = sec[0x0D];
    uint32_t reserved = sec[0x0E] | (sec[0x0F] << 8);
    uint32_t num_fats = sec[0x10];
    uint32_t fat_size = sec[0x24] | (sec[0x25]<<8) | (sec[0x26]<<16) | (sec[0x27]<<24);
    uint32_t total_sectors = sec[0x20] | (sec[0x21]<<8) | (sec[0x22]<<16) | (sec[0x23]<<24);

    if (spc == 0 || fat_size == 0) { sd_return(cpu, (uint32_t)-1); return; }

    uint32_t fat_start = part_start + reserved;
    uint32_t total_clusters = (total_sectors - reserved - fat_size * num_fats) / spc;
    uint32_t cluster_size = spc * 512;

    /* Scan FAT natively — count free entries (value == 0) */
    uint32_t free_count = 0;
    uint32_t cl = 2;
    for (uint32_t s = 0; s < fat_size && cl < total_clusters + 2; s++) {
        if (img_read_sector(ss, fat_start + s, sec) != 0) break;
        uint32_t *fat = (uint32_t *)sec;
        uint32_t start = (s == 0) ? 2 : 0;
        for (uint32_t i = start; i < 128 && cl < total_clusters + 2; i++, cl++) {
            if ((fat[i] & 0x0FFFFFFF) == 0)
                free_count++;
        }
    }

    uint64_t total_bytes = (uint64_t)total_clusters * cluster_size;
    uint64_t free_bytes  = (uint64_t)free_count * cluster_size;

    /* Write uint64_t results to emulator memory (little-endian) */
    if (total_ptr) {
        mem_write32(cpu->mem, total_ptr,     (uint32_t)total_bytes);
        mem_write32(cpu->mem, total_ptr + 4, (uint32_t)(total_bytes >> 32));
    }
    if (free_ptr) {
        mem_write32(cpu->mem, free_ptr,     (uint32_t)free_bytes);
        mem_write32(cpu->mem, free_ptr + 4, (uint32_t)(free_bytes >> 32));
    }

    fprintf(stderr, "[SD] fat32_volume_info: total=%lluMB free=%lluMB (native scan)\n",
            (unsigned long long)(total_bytes / (1024*1024)),
            (unsigned long long)(free_bytes / (1024*1024)));
    sd_return(cpu, 0);
}

/*
 * exfat_volume_info(vol, uint64_t *total_bytes, uint64_t *free_bytes) -> int
 *
 * Similar native acceleration for exFAT. Parses the exFAT boot sector
 * and allocation bitmap directly from the image file.
 */
static void stub_exfat_volume_info(xtensa_cpu_t *cpu, void *ctx) {
    sdcard_stubs_t *ss = ctx;
    (void)sd_arg(cpu, 0);  /* vol ptr — ignored, we read from image */
    uint32_t total_ptr = sd_arg(cpu, 1);
    uint32_t free_ptr  = sd_arg(cpu, 2);

    if (!ss->img_file) { sd_return(cpu, (uint32_t)-1); return; }

    /* Find partition start */
    uint8_t sec[512];
    uint32_t part_start = 0;
    if (img_read_sector(ss, 0, sec) == 0 && sec[510] == 0x55 && sec[511] == 0xAA) {
        uint8_t ptype = sec[446 + 4];
        uint32_t start_lba = sec[446+8] | (sec[446+9]<<8) |
                             (sec[446+10]<<16) | (sec[446+11]<<24);
        if (ptype == 0x0B || ptype == 0x0C || ptype == 0x07 || ptype == 0xEE)
            part_start = start_lba;
    }

    /* Read exFAT boot sector */
    if (img_read_sector(ss, part_start, sec) != 0) {
        sd_return(cpu, (uint32_t)-1);
        return;
    }

    /* Verify "EXFAT   " signature at offset 3 */
    if (memcmp(sec + 3, "EXFAT   ", 8) != 0) {
        sd_return(cpu, (uint32_t)-1);
        return;
    }

    uint32_t cluster_heap_offset = sec[88] | (sec[89]<<8) | (sec[90]<<16) | (sec[91]<<24);
    uint32_t cluster_count = sec[92] | (sec[93]<<8) | (sec[94]<<16) | (sec[95]<<24);
    uint8_t  bps_shift = sec[108];
    uint8_t  spc_shift = sec[109];
    uint32_t bytes_per_sector = 1u << bps_shift;
    uint32_t spc = 1u << spc_shift;
    uint32_t cluster_size = spc * bytes_per_sector;

    uint64_t total_bytes = (uint64_t)cluster_count * cluster_size;

    /* Scan allocation bitmap for free clusters.
     * The bitmap is typically in cluster 2, at cluster_heap_offset.
     * Each bit = one cluster: 0 = free, 1 = allocated. */
    uint32_t bitmap_lba = part_start + cluster_heap_offset;
    uint32_t free_count = 0;
    uint32_t checked = 0;
    uint32_t bitmap_sectors = (cluster_count + (bytes_per_sector * 8) - 1) / (bytes_per_sector * 8);

    for (uint32_t s = 0; s < bitmap_sectors && checked < cluster_count; s++) {
        if (img_read_sector(ss, bitmap_lba + s, sec) != 0) break;
        for (int b = 0; b < 512 && checked < cluster_count; b++) {
            for (int bit = 0; bit < 8 && checked < cluster_count; bit++, checked++) {
                if (!(sec[b] & (1 << bit)))
                    free_count++;
            }
        }
    }

    uint64_t free_bytes = (uint64_t)free_count * cluster_size;

    if (total_ptr) {
        mem_write32(cpu->mem, total_ptr,     (uint32_t)total_bytes);
        mem_write32(cpu->mem, total_ptr + 4, (uint32_t)(total_bytes >> 32));
    }
    if (free_ptr) {
        mem_write32(cpu->mem, free_ptr,     (uint32_t)free_bytes);
        mem_write32(cpu->mem, free_ptr + 4, (uint32_t)(free_bytes >> 32));
    }

    fprintf(stderr, "[SD] exfat_volume_info: total=%lluMB free=%lluMB (native scan)\n",
            (unsigned long long)(total_bytes / (1024*1024)),
            (unsigned long long)(free_bytes / (1024*1024)));
    sd_return(cpu, 0);
}

/* ===== Public API ===== */

sdcard_stubs_t *sdcard_stubs_create(xtensa_cpu_t *cpu) {
    sdcard_stubs_t *ss = calloc(1, sizeof(*ss));
    if (!ss) return NULL;
    ss->cpu = cpu;
    return ss;
}

void sdcard_stubs_destroy(sdcard_stubs_t *ss) {
    if (!ss) return;
    if (ss->img_file) {
        fclose(ss->img_file);
        ss->img_file = NULL;
    }
    free(ss);
}

void sdcard_stubs_set_image(sdcard_stubs_t *ss, const char *path) {
    if (!ss) return;
    ss->img_path = path;
}

void sdcard_stubs_set_size(sdcard_stubs_t *ss, uint64_t size_bytes) {
    if (!ss) return;
    ss->requested_size = size_bytes;
}

int sdcard_stubs_hook_symbols(sdcard_stubs_t *ss, const elf_symbols_t *syms) {
    if (!ss || !syms) return 0;

    esp32_rom_stubs_t *rom = ss->cpu->pc_hook_ctx;
    if (!rom) return 0;
    ss->rom = rom;

    int hooked = 0;
    struct {
        const char *name;
        rom_stub_fn fn;
    } hooks[] = {
        /* Firmware-level SD card API */
        { "sdcard_init",        stub_sdcard_init },
        { "sdcard_deinit",      stub_sdcard_deinit },
        { "sdcard_size",        stub_sdcard_size },
        { "sdcard_sector_size", stub_sdcard_sector_size },
        { "sdcard_read",        stub_sdcard_read },
        { "sdcard_write",       stub_sdcard_write },
        /* ESP-IDF SDMMC layer (used by filesystem: exFAT, FAT32) */
        { "sdmmc_card_init",       stub_sdmmc_card_init },
        { "sdmmc_read_sectors",    stub_sdmmc_read_sectors },
        { "sdmmc_write_sectors",   stub_sdmmc_write_sectors },
        { "sdmmc_read_sectors_dma", stub_sdmmc_read_sectors },
        { "sdmmc_write_sectors_dma", stub_sdmmc_write_sectors },
        { "sdspi_host_init_device", stub_sdspi_host_init_device },
        /* Native-accelerated volume info (bypass slow emulated FAT scan) */
        { "fat32_volume_info",     stub_fat32_volume_info },
        { "exfat_volume_info",     stub_exfat_volume_info },
        { NULL, NULL }
    };

    for (int i = 0; hooks[i].name; i++) {
        uint32_t addr;
        if (elf_symbols_find(syms, hooks[i].name, &addr) == 0) {
            rom_stubs_register_ctx(rom, addr, hooks[i].fn, hooks[i].name, ss);
            hooked++;
        }
    }

    return hooked;
}
