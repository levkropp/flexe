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
        /* Try common CSD offset candidates for ESP-IDF v4.4/v5.x.
         * CSD.capacity is typically 8 bytes into the CSD struct.
         * CSD.sector_size is 12 bytes into the CSD struct. */
        static const int csd_offsets[] = { 92, 100, 108, 116, 124 };
        for (int i = 0; i < 5; i++) {
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
