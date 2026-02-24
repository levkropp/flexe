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

/* ===== SD card stub implementations ===== */

/* sdcard_init() -> 0 on success, -1 on failure */
static void stub_sdcard_init(xtensa_cpu_t *cpu, void *ctx) {
    sdcard_stubs_t *ss = ctx;
    if (ss->img_file) {
        sd_return(cpu, 0);
        return;
    }
    if (ss->img_path) {
        ss->img_file = fopen(ss->img_path, "r+b");
        if (!ss->img_file)
            ss->img_file = fopen(ss->img_path, "w+b");
        if (ss->img_file) {
            fseek(ss->img_file, 0, SEEK_END);
            ss->img_size = (uint64_t)ftell(ss->img_file);
            /* Expand file to requested size if it's too small */
            uint64_t target = ss->requested_size ? ss->requested_size : DEFAULT_SD_SIZE;
            if (ss->img_size < target) {
                if (ftruncate(fileno(ss->img_file), (off_t)target) == 0)
                    ss->img_size = target;
            }
            sd_return(cpu, 0);
            return;
        }
    }
    sd_return(cpu, (uint32_t)-1);
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

    if (!ss->img_file) { sd_return(cpu, (uint32_t)-1); return; }

    uint64_t offset = (uint64_t)lba * 512;
    uint32_t bytes = count * 512;
    uint8_t buf[512];

    fseek(ss->img_file, (long)offset, SEEK_SET);
    for (uint32_t i = 0; i < count; i++) {
        size_t got = fread(buf, 1, 512, ss->img_file);
        if (got < 512)
            memset(buf + got, 0, 512 - got);
        for (int j = 0; j < 512; j++)
            mem_write8(cpu->mem, data + i * 512 + (uint32_t)j, buf[j]);
    }
    (void)bytes;
    sd_return(cpu, 0);
}

/* sdcard_write(lba, count, data) -> 0 on success */
static void stub_sdcard_write(xtensa_cpu_t *cpu, void *ctx) {
    sdcard_stubs_t *ss = ctx;
    uint32_t lba   = sd_arg(cpu, 0);
    uint32_t count = sd_arg(cpu, 1);
    uint32_t data  = sd_arg(cpu, 2);

    if (!ss->img_file) { sd_return(cpu, (uint32_t)-1); return; }

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
        { "sdcard_init",        stub_sdcard_init },
        { "sdcard_deinit",      stub_sdcard_deinit },
        { "sdcard_size",        stub_sdcard_size },
        { "sdcard_sector_size", stub_sdcard_sector_size },
        { "sdcard_read",        stub_sdcard_read },
        { "sdcard_write",       stub_sdcard_write },
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
