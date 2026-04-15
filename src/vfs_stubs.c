/*
 * vfs_stubs.c — ESP32 VFS (Virtual File System) stubs
 *
 * Maps /spiffs/ (or /littlefs/) paths in guest to a host directory.
 * Hooks esp_vfs_open/read/write/close/stat and SPIFFSFS::begin().
 */

#include "vfs_stubs.h"
#include "rom_stubs.h"
#include "memory.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>

/* ===== Calling convention helpers (same as rom_stubs.c) ===== */

static uint32_t vfs_arg(xtensa_cpu_t *cpu, int n) {
    int ci = XT_PS_CALLINC(cpu->ps);
    return ar_read(cpu, ci * 4 + 2 + n);
}

static void vfs_return(xtensa_cpu_t *cpu, uint32_t retval) {
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

/* ===== Guest memory string helper ===== */

static void vfs_read_str(xtensa_cpu_t *cpu, uint32_t addr, char *buf, int maxlen) {
    for (int i = 0; i < maxlen - 1; i++) {
        uint8_t c = mem_read8(cpu->mem, addr + (uint32_t)i);
        buf[i] = (char)c;
        if (c == 0) return;
    }
    buf[maxlen - 1] = '\0';
}

/* ===== File descriptor table ===== */

#define VFS_MAX_FDS     16
#define VFS_FD_OFFSET   64   /* guest FDs start at 64 to avoid collisions */
#define VFS_PATH_MAX    256
#define VFS_MAX_MOUNTS  4

/* Pool of guest-side FILE structs (newlib __sFILE).
 * We hand these out from stub___sfp, since real ROM __sfp is just a
 * placeholder and the firmware's _GLOBAL_REENT->__sglue is empty.
 *
 * Pool lives in DRAM scratch at 0x3FFE3D00. Each FILE is 96 bytes. */
#define VFS_FILE_POOL_ADDR  0x3FFE3D00u
#define VFS_FILE_SIZE       96
#define VFS_FILE_POOL_SLOTS VFS_MAX_FDS

/* newlib __sFILE field offsets (32-bit, ESP32) */
#define SFILE_FLAGS_OFS  12  /* short */
#define SFILE_FILE_OFS   14  /* short */
#define SFILE_BF_BASE    16
#define SFILE_BF_SIZE    20
#define SFILE_COOKIE_OFS 32
#define SFILE_READ_OFS   36
#define SFILE_WRITE_OFS  40
#define SFILE_SEEK_OFS   44
#define SFILE_CLOSE_OFS  48

#define SFILE_FLAG_RD    0x0004  /* __SRD */
#define SFILE_FLAG_WR    0x0008  /* __SWR */
#define SFILE_FLAG_NBF   0x0002  /* __SNBF unbuffered */

/* ROM addresses of newlib stdio dispatch functions (also used as FILE
 * function-pointer slots so that fwrite/fread/fseek/fclose come back
 * through our PC hook). */
#define ROM_SREAD   0x40001118u
#define ROM_SWRITE  0x40001150u
#define ROM_SSEEK   0x40001184u
#define ROM_SCLOSE  0x400011B8u
#define ROM_SFP     0x40001E90u

typedef struct {
    FILE *fp;
    int   in_use;
} vfs_fd_entry_t;

typedef struct {
    char prefix[32];     /* e.g., "/spiffs" */
    char host_dir[256];  /* host directory path */
} vfs_mount_t;

struct vfs_stubs {
    xtensa_cpu_t   *cpu;
    vfs_fd_entry_t  fds[VFS_MAX_FDS];
    vfs_mount_t     mounts[VFS_MAX_MOUNTS];
    int             mount_count;
    uint8_t         file_slot_used[VFS_FILE_POOL_SLOTS];
};

static int vfs_alloc_fd(vfs_stubs_t *v, FILE *fp) {
    for (int i = 0; i < VFS_MAX_FDS; i++) {
        if (!v->fds[i].in_use) {
            v->fds[i].fp = fp;
            v->fds[i].in_use = 1;
            return i + VFS_FD_OFFSET;
        }
    }
    return -1;
}

static FILE *vfs_get_fp(vfs_stubs_t *v, int guest_fd) {
    int idx = guest_fd - VFS_FD_OFFSET;
    if (idx < 0 || idx >= VFS_MAX_FDS || !v->fds[idx].in_use)
        return NULL;
    return v->fds[idx].fp;
}

static void vfs_free_fd(vfs_stubs_t *v, int guest_fd) {
    int idx = guest_fd - VFS_FD_OFFSET;
    if (idx >= 0 && idx < VFS_MAX_FDS) {
        v->fds[idx].fp = NULL;
        v->fds[idx].in_use = 0;
    }
}

/* Resolve guest path to host path.
 * Returns 1 if resolved, 0 if no matching mount. */
static int vfs_resolve_path(vfs_stubs_t *v, const char *guest_path,
                             char *host_path, int host_max) {
    for (int i = 0; i < v->mount_count; i++) {
        int plen = (int)strlen(v->mounts[i].prefix);
        if (strncmp(guest_path, v->mounts[i].prefix, (size_t)plen) == 0 &&
            (guest_path[plen] == '/' || guest_path[plen] == '\0')) {
            const char *rel = guest_path + plen;
            if (rel[0] == '/') rel++;
            snprintf(host_path, (size_t)host_max, "%s/%s",
                     v->mounts[i].host_dir, rel);
            return 1;
        }
    }
    return 0;
}

/* ===== Stub implementations ===== */

/* esp_vfs_open(struct _reent *r, const char *path, int flags, int mode) -> fd */
void stub_vfs_open(xtensa_cpu_t *cpu, void *ctx) {
    vfs_stubs_t *v = (vfs_stubs_t *)ctx;
    /* arg0 = _reent*, arg1 = path, arg2 = flags, arg3 = mode */
    uint32_t path_ptr = vfs_arg(cpu, 1);
    uint32_t flags = vfs_arg(cpu, 2);

    char guest_path[VFS_PATH_MAX];
    vfs_read_str(cpu, path_ptr, guest_path, VFS_PATH_MAX);

    char host_path[VFS_PATH_MAX];
    if (!vfs_resolve_path(v, guest_path, host_path, VFS_PATH_MAX)) {
        vfs_return(cpu, (uint32_t)-1);  /* no mount for this path */
        return;
    }

    /* Map POSIX O_flags to fopen mode.
     * O_RDONLY=0, O_WRONLY=1, O_RDWR=2, O_CREAT=0x100, O_TRUNC=0x200 */
    const char *mode = "rb";
    if (flags & 1) {
        mode = (flags & 0x200) ? "wb" : "ab";  /* O_WRONLY: truncate or append */
    } else if (flags & 2) {
        mode = "r+b";  /* O_RDWR */
    }

    FILE *fp = fopen(host_path, mode);
    if (!fp) {
        vfs_return(cpu, (uint32_t)-1);
        return;
    }

    int fd = vfs_alloc_fd(v, fp);
    if (fd < 0) {
        fclose(fp);
        vfs_return(cpu, (uint32_t)-1);
        return;
    }

    fprintf(stderr, "[VFS] open(\"%s\") -> fd=%d\n", guest_path, fd);
    vfs_return(cpu, (uint32_t)fd);
}

/* esp_vfs_read(struct _reent *r, int fd, void *dst, size_t size) -> bytes_read */
void stub_vfs_read(xtensa_cpu_t *cpu, void *ctx) {
    vfs_stubs_t *v = (vfs_stubs_t *)ctx;
    /* arg0 = _reent*, arg1 = fd, arg2 = buf, arg3 = size */
    uint32_t fd = vfs_arg(cpu, 1);
    uint32_t buf_ptr = vfs_arg(cpu, 2);
    uint32_t size = vfs_arg(cpu, 3);

    FILE *fp = vfs_get_fp(v, (int)fd);
    if (!fp) {
        vfs_return(cpu, (uint32_t)-1);
        return;
    }

    uint8_t tmp[512];
    uint32_t total = 0;
    while (total < size) {
        uint32_t chunk = size - total;
        if (chunk > sizeof(tmp)) chunk = sizeof(tmp);
        size_t got = fread(tmp, 1, chunk, fp);
        if (got == 0) break;
        for (size_t i = 0; i < got; i++)
            mem_write8(cpu->mem, buf_ptr + total + (uint32_t)i, tmp[i]);
        total += (uint32_t)got;
    }
    vfs_return(cpu, total);
}

/* esp_vfs_write(struct _reent *r, int fd, const void *data, size_t size) -> bytes_written */
static void stub_vfs_write(xtensa_cpu_t *cpu, void *ctx) {
    vfs_stubs_t *v = (vfs_stubs_t *)ctx;
    /* arg0 = _reent*, arg1 = fd, arg2 = buf, arg3 = size */
    uint32_t fd = vfs_arg(cpu, 1);
    uint32_t buf_ptr = vfs_arg(cpu, 2);
    uint32_t size = vfs_arg(cpu, 3);

    /* stdout/stderr: route newlib printf() straight to UART0 TX FIFO.
     * ESP-IDF does not open these as vfs_stubs host-backed files, so the
     * normal fd table lookup would return NULL and the write would be
     * dropped. Writing to the FIFO register lets the UART TX callback
     * capture the bytes just like ets_printf and ESP_LOG output do. */
    if (fd == 1 || fd == 2) {
        for (uint32_t i = 0; i < size; i++) {
            uint8_t c = mem_read8(cpu->mem, buf_ptr + i);
            mem_write32(cpu->mem, 0x3FF40000u, c);
        }
        vfs_return(cpu, size);
        return;
    }

    FILE *fp = vfs_get_fp(v, (int)fd);
    if (!fp) {
        vfs_return(cpu, (uint32_t)-1);
        return;
    }

    uint8_t tmp[512];
    uint32_t total = 0;
    while (total < size) {
        uint32_t chunk = size - total;
        if (chunk > sizeof(tmp)) chunk = sizeof(tmp);
        for (uint32_t i = 0; i < chunk; i++)
            tmp[i] = mem_read8(cpu->mem, buf_ptr + total + i);
        size_t wrote = fwrite(tmp, 1, chunk, fp);
        if (wrote == 0) break;
        total += (uint32_t)wrote;
    }
    vfs_return(cpu, total);
}

/* esp_vfs_close(struct _reent *r, int fd) -> 0 on success */
void stub_vfs_close(xtensa_cpu_t *cpu, void *ctx) {
    vfs_stubs_t *v = (vfs_stubs_t *)ctx;
    /* arg0 = _reent*, arg1 = fd */
    uint32_t fd = vfs_arg(cpu, 1);

    FILE *fp = vfs_get_fp(v, (int)fd);
    if (!fp) {
        vfs_return(cpu, (uint32_t)-1);
        return;
    }

    fprintf(stderr, "[VFS] close(fd=%u)\n", fd);
    fclose(fp);
    vfs_free_fd(v, (int)fd);
    vfs_return(cpu, 0);
}

/* esp_vfs_stat(struct _reent *r, const char *path, struct stat *st) -> 0 on success */
static void stub_vfs_stat(xtensa_cpu_t *cpu, void *ctx) {
    vfs_stubs_t *v = (vfs_stubs_t *)ctx;
    /* arg0 = _reent*, arg1 = path, arg2 = st */
    uint32_t path_ptr = vfs_arg(cpu, 1);
    uint32_t st_ptr = vfs_arg(cpu, 2);

    char guest_path[VFS_PATH_MAX];
    vfs_read_str(cpu, path_ptr, guest_path, VFS_PATH_MAX);

    char host_path[VFS_PATH_MAX];
    if (!vfs_resolve_path(v, guest_path, host_path, VFS_PATH_MAX)) {
        vfs_return(cpu, (uint32_t)-1);
        return;
    }

    struct stat st;
    if (stat(host_path, &st) != 0) {
        vfs_return(cpu, (uint32_t)-1);
        return;
    }

    /* Write a minimal ESP32 struct stat to guest memory.
     * ESP32 newlib struct stat layout (simplified):
     *   +0x00: st_dev (2 bytes)
     *   +0x04: st_ino (2 bytes)
     *   +0x08: st_mode (4 bytes)
     *   ...
     *   +0x30: st_size (4 bytes)
     * We zero-fill and just set st_size and st_mode. */
    if (st_ptr) {
        /* Zero-fill 64 bytes (typical struct stat size on ESP32) */
        for (int i = 0; i < 64; i += 4)
            mem_write32(cpu->mem, st_ptr + (uint32_t)i, 0);
        /* st_mode at offset 0x08: S_IFREG=0100000 for files, S_IFDIR=0040000 for dirs */
        uint32_t mode = S_ISDIR(st.st_mode) ? 0040755 : 0100644;
        mem_write32(cpu->mem, st_ptr + 0x08, mode);
        /* st_size at offset 0x30 */
        mem_write32(cpu->mem, st_ptr + 0x30, (uint32_t)st.st_size);
    }
    vfs_return(cpu, 0);
}

/* Add a mount point to the table. Creates host_dir on the host side. */
static int vfs_add_mount(vfs_stubs_t *v, const char *prefix, const char *host_dir) {
    if (!v || !prefix || !host_dir) return -1;
    /* Skip if prefix already registered */
    for (int i = 0; i < v->mount_count; i++) {
        if (strcmp(v->mounts[i].prefix, prefix) == 0) {
            fprintf(stderr, "[VFS] Mount '%s' already registered -> %s\n",
                    prefix, v->mounts[i].host_dir);
            return 0;
        }
    }
    if (v->mount_count >= VFS_MAX_MOUNTS) return -1;
    vfs_mount_t *m = &v->mounts[v->mount_count++];
    strncpy(m->prefix, prefix, sizeof(m->prefix) - 1);
    m->prefix[sizeof(m->prefix) - 1] = '\0';
    strncpy(m->host_dir, host_dir, sizeof(m->host_dir) - 1);
    m->host_dir[sizeof(m->host_dir) - 1] = '\0';

    /* mkdir -p on host */
    char cmd[600];
    snprintf(cmd, sizeof(cmd), "mkdir -p '%s'", host_dir);
    int rc = system(cmd);
    (void)rc;
    fprintf(stderr, "[VFS] Mounted %s -> %s\n", prefix, host_dir);
    return 0;
}

/* Compute host directory under /tmp/flexe-spiffs/<basename> for a given prefix. */
static void vfs_default_host_dir(const char *prefix, char *out, int outmax) {
    const char *base = prefix;
    while (*base == '/') base++;
    if (!*base) base = "spiffs";
    snprintf(out, (size_t)outmax, "/tmp/flexe-spiffs/%s", base);
}

/* esp_vfs_spiffs_register(const esp_vfs_spiffs_conf_t *conf) -> esp_err_t
 * conf layout (ESP-IDF):
 *   +0x00 const char *base_path
 *   +0x04 const char *partition_label
 *   +0x08 size_t      max_files
 *   +0x0C bool        format_if_mount_failed
 */
static void stub_vfs_spiffs_register(xtensa_cpu_t *cpu, void *ctx) {
    vfs_stubs_t *v = (vfs_stubs_t *)ctx;
    uint32_t conf_ptr = vfs_arg(cpu, 0);

    char base_path[64] = "/spiffs";
    if (conf_ptr) {
        uint32_t bp = mem_read32(cpu->mem, conf_ptr + 0x00);
        if (bp) vfs_read_str(cpu, bp, base_path, sizeof(base_path));
    }

    char host_dir[VFS_PATH_MAX];
    vfs_default_host_dir(base_path, host_dir, sizeof(host_dir));
    vfs_add_mount(v, base_path, host_dir);

    fprintf(stderr, "[VFS] esp_vfs_spiffs_register(base_path=\"%s\") -> ESP_OK\n",
            base_path);
    vfs_return(cpu, 0);
}

/* esp_vfs_fat_spiflash_mount_rw_wl(base_path, partition_label, mount_config*, wl_handle_t*)
 * esp_vfs_fat_spiflash_mount_ro(base_path, partition_label, mount_config*)
 * Treat both as host-backed mounts. */
static void stub_vfs_fat_mount(xtensa_cpu_t *cpu, void *ctx) {
    vfs_stubs_t *v = (vfs_stubs_t *)ctx;
    uint32_t bp_ptr = vfs_arg(cpu, 0);

    char base_path[64] = "/spiflash";
    if (bp_ptr) vfs_read_str(cpu, bp_ptr, base_path, sizeof(base_path));

    char host_dir[VFS_PATH_MAX];
    vfs_default_host_dir(base_path, host_dir, sizeof(host_dir));
    vfs_add_mount(v, base_path, host_dir);

    /* If a wl_handle_t* was passed (arg 3), write a dummy non-zero handle. */
    uint32_t wl_ptr = vfs_arg(cpu, 3);
    if (wl_ptr) mem_write32(cpu->mem, wl_ptr, 1);

    fprintf(stderr, "[VFS] esp_vfs_fat_spiflash_mount(base_path=\"%s\") -> ESP_OK\n",
            base_path);
    vfs_return(cpu, 0);
}

/* esp_vfs_fat_sdmmc_mount(base_path, host, slot_config, mount_config, out_card)
 * esp_vfs_fat_sdspi_mount(base_path, host, dev_config, mount_config, out_card)
 *
 * Short-circuit the SDMMC/SDSPI bring-up: register the base_path as a
 * host-backed mount (mirroring the SPIFFS pattern under /tmp/flexe-sdcard)
 * and write a dummy non-NULL sdmmc_card_t* to the out-parameter so caller
 * code that dereferences it doesn't fault. We do not model the SD protocol
 * — VFS/FATFS plumbing already handles the file I/O. */
#define VFS_SDMMC_CARD_ADDR  0x3FFE4400u   /* 64-byte scratch in DRAM */
#define VFS_SDMMC_CARD_TAG   0xDEADBEEFu

static void vfs_default_sdcard_host_dir(const char *prefix, char *out, int outmax) {
    const char *base = prefix;
    while (*base == '/') base++;
    if (!*base) base = "sdcard";
    snprintf(out, (size_t)outmax, "/tmp/flexe-sdcard/%s", base);
}

static void stub_vfs_fat_sdmmc_mount(xtensa_cpu_t *cpu, void *ctx) {
    vfs_stubs_t *v = (vfs_stubs_t *)ctx;
    uint32_t bp_ptr      = vfs_arg(cpu, 0);
    uint32_t out_card_pp = vfs_arg(cpu, 4);

    char base_path[64] = "/sdcard";
    if (bp_ptr) vfs_read_str(cpu, bp_ptr, base_path, sizeof(base_path));

    char host_dir[VFS_PATH_MAX];
    vfs_default_sdcard_host_dir(base_path, host_dir, sizeof(host_dir));
    vfs_add_mount(v, base_path, host_dir);

    /* Tag the dummy sdmmc_card_t so any code that inspects it sees a
     * recognizable marker (and so the pointer is non-NULL). */
    mem_write32(cpu->mem, VFS_SDMMC_CARD_ADDR, VFS_SDMMC_CARD_TAG);
    for (uint32_t i = 4; i < 64; i += 4)
        mem_write32(cpu->mem, VFS_SDMMC_CARD_ADDR + i, 0);

    if (out_card_pp)
        mem_write32(cpu->mem, out_card_pp, VFS_SDMMC_CARD_ADDR);

    fprintf(stderr, "[VFS] esp_vfs_fat_sdmmc/sdspi_mount(base_path=\"%s\") -> ESP_OK\n",
            base_path);
    vfs_return(cpu, 0);
}

/* SPIFFSFS::begin(formatOnFail, basePath, maxOpenFiles, partLabel) -> true */
static void stub_spiffsfs_begin(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    fprintf(stderr, "[VFS] SPIFFSFS::begin() -> true\n");
    vfs_return(cpu, 1);  /* true */
}

/* esp_spiffs_mounted(partition_label) -> true */
static void stub_spiffs_mounted(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    vfs_return(cpu, 1);
}

/* esp_spiffs_info(partition_label, *total, *used) -> ESP_OK */
static void stub_spiffs_info(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    uint32_t total_ptr = vfs_arg(cpu, 1);
    uint32_t used_ptr = vfs_arg(cpu, 2);
    if (total_ptr) mem_write32(cpu->mem, total_ptr, 1048576);  /* 1MB total */
    if (used_ptr)  mem_write32(cpu->mem, used_ptr, 0);         /* 0 used */
    vfs_return(cpu, 0);
}

/* Generic no-op returning ESP_OK */
static void stub_vfs_ok(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    vfs_return(cpu, 0);
}

/* esp_vfs_lseek(struct _reent *r, int fd, off_t offset, int whence) -> new_pos */
static void stub_vfs_lseek(xtensa_cpu_t *cpu, void *ctx) {
    vfs_stubs_t *v = (vfs_stubs_t *)ctx;
    /* arg0 = _reent*, arg1 = fd, arg2 = offset, arg3 = whence */
    uint32_t fd = vfs_arg(cpu, 1);
    int32_t offset = (int32_t)vfs_arg(cpu, 2);
    uint32_t whence = vfs_arg(cpu, 3);

    FILE *fp = vfs_get_fp(v, (int)fd);
    if (!fp) {
        vfs_return(cpu, (uint32_t)-1);
        return;
    }

    int w = SEEK_SET;
    if (whence == 1) w = SEEK_CUR;
    else if (whence == 2) w = SEEK_END;

    if (fseek(fp, offset, w) != 0) {
        vfs_return(cpu, (uint32_t)-1);
        return;
    }
    vfs_return(cpu, (uint32_t)ftell(fp));
}

/* esp_vfs_unlink(struct _reent *r, const char *path) -> 0 on success */
static void stub_vfs_unlink(xtensa_cpu_t *cpu, void *ctx) {
    vfs_stubs_t *v = (vfs_stubs_t *)ctx;
    uint32_t path_ptr = vfs_arg(cpu, 1);
    char guest_path[VFS_PATH_MAX];
    vfs_read_str(cpu, path_ptr, guest_path, VFS_PATH_MAX);
    char host_path[VFS_PATH_MAX];
    if (!vfs_resolve_path(v, guest_path, host_path, VFS_PATH_MAX)) {
        vfs_return(cpu, (uint32_t)-1);
        return;
    }
    int rc = remove(host_path);
    fprintf(stderr, "[VFS] unlink(\"%s\") -> %d\n", guest_path, rc);
    vfs_return(cpu, (uint32_t)(rc == 0 ? 0 : -1));
}

/* esp_vfs_rename(struct _reent *r, const char *src, const char *dst) -> 0 on success */
static void stub_vfs_rename(xtensa_cpu_t *cpu, void *ctx) {
    vfs_stubs_t *v = (vfs_stubs_t *)ctx;
    uint32_t src_ptr = vfs_arg(cpu, 1);
    uint32_t dst_ptr = vfs_arg(cpu, 2);
    char src_g[VFS_PATH_MAX], dst_g[VFS_PATH_MAX];
    vfs_read_str(cpu, src_ptr, src_g, VFS_PATH_MAX);
    vfs_read_str(cpu, dst_ptr, dst_g, VFS_PATH_MAX);
    char src_h[VFS_PATH_MAX], dst_h[VFS_PATH_MAX];
    if (!vfs_resolve_path(v, src_g, src_h, VFS_PATH_MAX) ||
        !vfs_resolve_path(v, dst_g, dst_h, VFS_PATH_MAX)) {
        vfs_return(cpu, (uint32_t)-1);
        return;
    }
    int rc = rename(src_h, dst_h);
    fprintf(stderr, "[VFS] rename(\"%s\" -> \"%s\") -> %d\n", src_g, dst_g, rc);
    vfs_return(cpu, (uint32_t)(rc == 0 ? 0 : -1));
}

/* esp_vfs_fstat(struct _reent *r, int fd, struct stat *st) -> 0 on success */
static void stub_vfs_fstat(xtensa_cpu_t *cpu, void *ctx) {
    vfs_stubs_t *v = (vfs_stubs_t *)ctx;
    /* arg0 = _reent*, arg1 = fd, arg2 = st */
    uint32_t fd = vfs_arg(cpu, 1);
    uint32_t st_ptr = vfs_arg(cpu, 2);

    FILE *fp = vfs_get_fp(v, (int)fd);
    if (!fp) {
        vfs_return(cpu, (uint32_t)-1);
        return;
    }

    /* Get file size via fseek/ftell */
    long cur = ftell(fp);
    fseek(fp, 0, SEEK_END);
    long size = ftell(fp);
    fseek(fp, cur, SEEK_SET);

    if (st_ptr) {
        for (int i = 0; i < 64; i += 4)
            mem_write32(cpu->mem, st_ptr + (uint32_t)i, 0);
        mem_write32(cpu->mem, st_ptr + 0x08, 0100644);  /* S_IFREG */
        mem_write32(cpu->mem, st_ptr + 0x30, (uint32_t)size);
    }
    vfs_return(cpu, 0);
}

/* ===== Newlib FILE pool + stdio dispatch shims ===== */

/* Allocate a FILE slot from the guest-memory pool, init fields. */
static uint32_t vfs_alloc_file_slot(vfs_stubs_t *v) {
    for (int i = 0; i < VFS_FILE_POOL_SLOTS; i++) {
        if (!v->file_slot_used[i]) {
            v->file_slot_used[i] = 1;
            uint32_t addr = VFS_FILE_POOL_ADDR + (uint32_t)i * VFS_FILE_SIZE;
            /* Zero the FILE */
            for (uint32_t off = 0; off < VFS_FILE_SIZE; off += 4)
                mem_write32(v->cpu->mem, addr + off, 0);
            /* Set _file = -1 (will be filled in by fopen after _open_r) */
            mem_write8(v->cpu->mem, addr + SFILE_FILE_OFS,     0xFF);
            mem_write8(v->cpu->mem, addr + SFILE_FILE_OFS + 1, 0xFF);
            /* _flags = __SRD | __SWR | __SNBF */
            uint16_t flags = SFILE_FLAG_RD | SFILE_FLAG_WR | SFILE_FLAG_NBF;
            mem_write8(v->cpu->mem, addr + SFILE_FLAGS_OFS,     (uint8_t)(flags & 0xFF));
            mem_write8(v->cpu->mem, addr + SFILE_FLAGS_OFS + 1, (uint8_t)(flags >> 8));
            /* _cookie = self */
            mem_write32(v->cpu->mem, addr + SFILE_COOKIE_OFS, addr);
            /* dispatch fn ptrs -> ROM stub addresses (we intercept these) */
            mem_write32(v->cpu->mem, addr + SFILE_READ_OFS,  ROM_SREAD);
            mem_write32(v->cpu->mem, addr + SFILE_WRITE_OFS, ROM_SWRITE);
            mem_write32(v->cpu->mem, addr + SFILE_SEEK_OFS,  ROM_SSEEK);
            mem_write32(v->cpu->mem, addr + SFILE_CLOSE_OFS, ROM_SCLOSE);
            return addr;
        }
    }
    return 0;
}

static void vfs_free_file_slot(vfs_stubs_t *v, uint32_t addr) {
    if (addr < VFS_FILE_POOL_ADDR) return;
    uint32_t idx = (addr - VFS_FILE_POOL_ADDR) / VFS_FILE_SIZE;
    if (idx < VFS_FILE_POOL_SLOTS) v->file_slot_used[idx] = 0;
}

/* Read short _file from a FILE struct */
static int16_t vfs_file_get_fd(xtensa_cpu_t *cpu, uint32_t fp) {
    uint8_t lo = mem_read8(cpu->mem, fp + SFILE_FILE_OFS);
    uint8_t hi = mem_read8(cpu->mem, fp + SFILE_FILE_OFS + 1);
    return (int16_t)((hi << 8) | lo);
}

/* __sfp(reent) -> FILE *  — allocate a FILE struct from the pool */
static void stub___sfp(xtensa_cpu_t *cpu, void *ctx) {
    vfs_stubs_t *v = (vfs_stubs_t *)ctx;
    uint32_t fp = vfs_alloc_file_slot(v);
    if (fp == 0) {
        fprintf(stderr, "[VFS] __sfp: pool exhausted\n");
        vfs_return(cpu, 0);
        return;
    }
    fprintf(stderr, "[VFS] __sfp -> 0x%08X\n", fp);
    vfs_return(cpu, fp);
}

/* __swrite(reent, cookie, buf, n) -> bytes_written
 * Cookie is the FILE*; we route writes by inspecting fp->_file.
 * - host-backed fd (>= VFS_FD_OFFSET): write to host file
 * - fd 1/2: route to UART0 FIFO (stdout/stderr)
 * - other:   write to UART0 FIFO as fallback so output is never silently dropped */
static void stub___swrite(xtensa_cpu_t *cpu, void *ctx) {
    vfs_stubs_t *v = (vfs_stubs_t *)ctx;
    uint32_t cookie = vfs_arg(cpu, 1);
    uint32_t buf    = vfs_arg(cpu, 2);
    uint32_t n      = vfs_arg(cpu, 3);

    int16_t fd = cookie ? vfs_file_get_fd(cpu, cookie) : -1;

    if (fd >= VFS_FD_OFFSET) {
        FILE *fphost = vfs_get_fp(v, fd);
        if (!fphost) { vfs_return(cpu, (uint32_t)-1); return; }
        uint8_t tmp[256];
        uint32_t total = 0;
        while (total < n) {
            uint32_t chunk = n - total;
            if (chunk > sizeof(tmp)) chunk = sizeof(tmp);
            for (uint32_t i = 0; i < chunk; i++)
                tmp[i] = mem_read8(cpu->mem, buf + total + i);
            size_t wrote = fwrite(tmp, 1, chunk, fphost);
            if (wrote == 0) break;
            total += (uint32_t)wrote;
        }
        vfs_return(cpu, total);
        return;
    }

    /* Stdout/stderr/unknown -> UART FIFO (newlib default behaviour) */
    for (uint32_t i = 0; i < n; i++) {
        uint8_t c = mem_read8(cpu->mem, buf + i);
        mem_write32(cpu->mem, 0x3FF40000u, c);
    }
    vfs_return(cpu, n);
}

/* __sread(reent, cookie, buf, n) -> bytes_read */
static void stub___sread(xtensa_cpu_t *cpu, void *ctx) {
    vfs_stubs_t *v = (vfs_stubs_t *)ctx;
    uint32_t cookie = vfs_arg(cpu, 1);
    uint32_t buf    = vfs_arg(cpu, 2);
    uint32_t n      = vfs_arg(cpu, 3);

    int16_t fd = cookie ? vfs_file_get_fd(cpu, cookie) : -1;
    if (fd < VFS_FD_OFFSET) { vfs_return(cpu, 0); return; }

    FILE *fphost = vfs_get_fp(v, fd);
    if (!fphost) { vfs_return(cpu, (uint32_t)-1); return; }

    uint8_t tmp[256];
    uint32_t total = 0;
    while (total < n) {
        uint32_t chunk = n - total;
        if (chunk > sizeof(tmp)) chunk = sizeof(tmp);
        size_t got = fread(tmp, 1, chunk, fphost);
        if (got == 0) break;
        for (size_t i = 0; i < got; i++)
            mem_write8(cpu->mem, buf + total + (uint32_t)i, tmp[i]);
        total += (uint32_t)got;
    }
    vfs_return(cpu, total);
}

/* __sseek(reent, cookie, offset, whence) -> new_pos */
static void stub___sseek(xtensa_cpu_t *cpu, void *ctx) {
    vfs_stubs_t *v = (vfs_stubs_t *)ctx;
    uint32_t cookie = vfs_arg(cpu, 1);
    int32_t offset  = (int32_t)vfs_arg(cpu, 2);
    uint32_t whence = vfs_arg(cpu, 3);
    int16_t fd = cookie ? vfs_file_get_fd(cpu, cookie) : -1;
    if (fd < VFS_FD_OFFSET) { vfs_return(cpu, 0); return; }
    FILE *fphost = vfs_get_fp(v, fd);
    if (!fphost) { vfs_return(cpu, (uint32_t)-1); return; }
    int w = SEEK_SET;
    if (whence == 1) w = SEEK_CUR;
    else if (whence == 2) w = SEEK_END;
    if (fseek(fphost, offset, w) != 0) { vfs_return(cpu, (uint32_t)-1); return; }
    vfs_return(cpu, (uint32_t)ftell(fphost));
}

/* __sclose(reent, cookie) -> 0 on success */
static void stub___sclose(xtensa_cpu_t *cpu, void *ctx) {
    vfs_stubs_t *v = (vfs_stubs_t *)ctx;
    uint32_t cookie = vfs_arg(cpu, 1);
    int16_t fd = cookie ? vfs_file_get_fd(cpu, cookie) : -1;
    if (fd >= VFS_FD_OFFSET) {
        FILE *fphost = vfs_get_fp(v, fd);
        if (fphost) fclose(fphost);
        vfs_free_fd(v, fd);
    }
    /* Free the FILE pool slot — cookie IS the fp (we set _cookie = self in __sfp). */
    if (cookie) vfs_free_file_slot(v, cookie);
    vfs_return(cpu, 0);
}

/* ===== High-level stdio shims ===== */
/* The compiler often turns fprintf(fp, "literal") into fwrite, and newlib's
 * own buffered fwrite path is too tangled (smakebuf/swsetup needs working
 * malloc + reentrant locks).  We just intercept fwrite/fread/fclose/fgets
 * directly and bypass the FILE buffer entirely. */

static int vfs_fp_in_pool(uint32_t fp) {
    return fp >= VFS_FILE_POOL_ADDR &&
           fp < VFS_FILE_POOL_ADDR + (uint32_t)(VFS_FILE_POOL_SLOTS * VFS_FILE_SIZE);
}

/* fwrite(ptr, size, nmemb, fp) -> nmemb on success */
static void stub_fwrite(xtensa_cpu_t *cpu, void *ctx) {
    vfs_stubs_t *v = (vfs_stubs_t *)ctx;
    uint32_t buf  = vfs_arg(cpu, 0);
    uint32_t sz   = vfs_arg(cpu, 1);
    uint32_t nm   = vfs_arg(cpu, 2);
    uint32_t fp   = vfs_arg(cpu, 3);
    uint32_t total = sz * nm;
    if (total == 0 || !fp) { vfs_return(cpu, 0); return; }

    if (vfs_fp_in_pool(fp)) {
        int16_t fd = vfs_file_get_fd(cpu, fp);
        FILE *fphost = (fd >= VFS_FD_OFFSET) ? vfs_get_fp(v, fd) : NULL;
        if (!fphost) { vfs_return(cpu, 0); return; }
        uint8_t tmp[512];
        uint32_t done = 0;
        while (done < total) {
            uint32_t chunk = total - done;
            if (chunk > sizeof(tmp)) chunk = sizeof(tmp);
            for (uint32_t i = 0; i < chunk; i++)
                tmp[i] = mem_read8(cpu->mem, buf + done + i);
            size_t w = fwrite(tmp, 1, chunk, fphost);
            if (w == 0) break;
            done += (uint32_t)w;
        }
        fflush(fphost);
        fprintf(stderr, "[VFS] fwrite(fd=%d, %u bytes)\n", fd, done);
        vfs_return(cpu, sz ? done / sz : 0);
        return;
    }

    /* Unknown fp -> assume stdout/stderr, route to UART */
    for (uint32_t i = 0; i < total; i++) {
        uint8_t c = mem_read8(cpu->mem, buf + i);
        mem_write32(cpu->mem, 0x3FF40000u, c);
    }
    vfs_return(cpu, nm);
}

/* fread(ptr, size, nmemb, fp) -> items read */
static void stub_fread(xtensa_cpu_t *cpu, void *ctx) {
    vfs_stubs_t *v = (vfs_stubs_t *)ctx;
    uint32_t buf  = vfs_arg(cpu, 0);
    uint32_t sz   = vfs_arg(cpu, 1);
    uint32_t nm   = vfs_arg(cpu, 2);
    uint32_t fp   = vfs_arg(cpu, 3);
    uint32_t total = sz * nm;
    if (total == 0 || !vfs_fp_in_pool(fp)) { vfs_return(cpu, 0); return; }
    int16_t fd = vfs_file_get_fd(cpu, fp);
    FILE *fphost = (fd >= VFS_FD_OFFSET) ? vfs_get_fp(v, fd) : NULL;
    if (!fphost) { vfs_return(cpu, 0); return; }

    uint8_t tmp[512];
    uint32_t done = 0;
    while (done < total) {
        uint32_t chunk = total - done;
        if (chunk > sizeof(tmp)) chunk = sizeof(tmp);
        size_t got = fread(tmp, 1, chunk, fphost);
        if (got == 0) break;
        for (size_t i = 0; i < got; i++)
            mem_write8(cpu->mem, buf + done + (uint32_t)i, tmp[i]);
        done += (uint32_t)got;
    }
    vfs_return(cpu, sz ? done / sz : 0);
}

/* fclose(fp) -> 0 on success */
static void stub_fclose(xtensa_cpu_t *cpu, void *ctx) {
    vfs_stubs_t *v = (vfs_stubs_t *)ctx;
    uint32_t fp = vfs_arg(cpu, 0);
    if (vfs_fp_in_pool(fp)) {
        int16_t fd = vfs_file_get_fd(cpu, fp);
        if (fd >= VFS_FD_OFFSET) {
            FILE *fphost = vfs_get_fp(v, fd);
            if (fphost) fclose(fphost);
            vfs_free_fd(v, fd);
        }
        vfs_free_file_slot(v, fp);
        fprintf(stderr, "[VFS] fclose(fp=0x%08X, fd=%d)\n", fp, fd);
    }
    vfs_return(cpu, 0);
}

/* fgets(s, n, fp) -> s on success, NULL on EOF/error */
static void stub_fgets(xtensa_cpu_t *cpu, void *ctx) {
    vfs_stubs_t *v = (vfs_stubs_t *)ctx;
    uint32_t s = vfs_arg(cpu, 0);
    int32_t  n = (int32_t)vfs_arg(cpu, 1);
    uint32_t fp = vfs_arg(cpu, 2);
    if (n <= 0 || !vfs_fp_in_pool(fp)) { vfs_return(cpu, 0); return; }
    int16_t fd = vfs_file_get_fd(cpu, fp);
    FILE *fphost = (fd >= VFS_FD_OFFSET) ? vfs_get_fp(v, fd) : NULL;
    if (!fphost) { vfs_return(cpu, 0); return; }

    int i = 0;
    int got_any = 0;
    while (i < n - 1) {
        int c = fgetc(fphost);
        if (c == EOF) break;
        got_any = 1;
        mem_write8(cpu->mem, s + (uint32_t)i, (uint8_t)c);
        i++;
        if (c == '\n') break;
    }
    mem_write8(cpu->mem, s + (uint32_t)i, 0);
    fprintf(stderr, "[VFS] fgets(fd=%d) -> %d bytes\n", fd, i);
    vfs_return(cpu, got_any ? s : 0);
}

/* ===== Module lifecycle ===== */

vfs_stubs_t *vfs_stubs_create(xtensa_cpu_t *cpu) {
    vfs_stubs_t *v = calloc(1, sizeof(*v));
    if (!v) return NULL;
    v->cpu = cpu;
    return v;
}

void vfs_stubs_destroy(vfs_stubs_t *v) {
    if (!v) return;
    /* Close any open files */
    for (int i = 0; i < VFS_MAX_FDS; i++) {
        if (v->fds[i].in_use && v->fds[i].fp) {
            fclose(v->fds[i].fp);
            v->fds[i].in_use = 0;
        }
    }
    free(v);
}

void vfs_stubs_set_spiffs_dir(vfs_stubs_t *v, const char *host_dir) {
    if (!v || !host_dir) return;
    if (v->mount_count >= VFS_MAX_MOUNTS) return;
    vfs_mount_t *m = &v->mounts[v->mount_count++];
    strncpy(m->prefix, "/spiffs", sizeof(m->prefix) - 1);
    strncpy(m->host_dir, host_dir, sizeof(m->host_dir) - 1);
    fprintf(stderr, "[VFS] Mounted /spiffs -> %s\n", host_dir);
}

int vfs_stubs_hook_symbols(vfs_stubs_t *v, const elf_symbols_t *syms) {
    if (!v || !syms) return 0;

    esp32_rom_stubs_t *rom = (esp32_rom_stubs_t *)v->cpu->pc_hook_ctx;
    if (!rom) return 0;

    int hooked = 0;
    uint32_t addr;

    /* Hook VFS file operations — these need the vfs_stubs context */
    struct { const char *name; void (*fn)(xtensa_cpu_t *, void *); } ctx_hooks[] = {
        { "esp_vfs_open",   stub_vfs_open },
        { "esp_vfs_read",   stub_vfs_read },
        { "esp_vfs_write",  stub_vfs_write },
        { "esp_vfs_close",  stub_vfs_close },
        { "esp_vfs_stat",   stub_vfs_stat },
        { "esp_vfs_lseek",  stub_vfs_lseek },
        { "esp_vfs_fstat",  stub_vfs_fstat },
        { "esp_vfs_unlink", stub_vfs_unlink },
        { "esp_vfs_rename", stub_vfs_rename },
        { NULL, NULL }
    };
    for (int i = 0; ctx_hooks[i].name; i++) {
        if (elf_symbols_find(syms, ctx_hooks[i].name, &addr) == 0) {
            rom_stubs_register_ctx(rom, addr, ctx_hooks[i].fn,
                                    ctx_hooks[i].name, v);
            hooked++;
        }
    }

    /* SPIFFS / FAT mount stubs — register the base_path as a host-backed mount.
     * These need ctx so they can mutate the vfs_stubs mount table. */
    struct { const char *name; void (*fn)(xtensa_cpu_t *, void *); } mount_hooks[] = {
        { "esp_vfs_spiffs_register",            stub_vfs_spiffs_register },
        { "esp_vfs_fat_spiflash_mount_rw_wl",   stub_vfs_fat_mount },
        { "esp_vfs_fat_spiflash_mount_ro",      stub_vfs_fat_mount },
        { "esp_vfs_fat_spiflash_mount",         stub_vfs_fat_mount },  /* legacy name */
        { "esp_vfs_fat_sdmmc_mount",            stub_vfs_fat_sdmmc_mount },
        { "esp_vfs_fat_sdspi_mount",            stub_vfs_fat_sdmmc_mount },
        { NULL, NULL }
    };
    for (int i = 0; mount_hooks[i].name; i++) {
        if (elf_symbols_find(syms, mount_hooks[i].name, &addr) == 0) {
            rom_stubs_register_ctx(rom, addr, mount_hooks[i].fn,
                                    mount_hooks[i].name, v);
            hooked++;
        }
    }

    /* No-context SPIFFS info stubs */
    struct { const char *name; void (*fn)(xtensa_cpu_t *, void *); } spiffs_hooks[] = {
        { "_ZN2fs8SPIFFSFS5beginEbPKchS2_",   stub_spiffsfs_begin },
        { "esp_spiffs_mounted",               stub_spiffs_mounted },
        { "esp_spiffs_info",                  stub_spiffs_info },
        { NULL, NULL }
    };
    for (int i = 0; spiffs_hooks[i].name; i++) {
        if (elf_symbols_find(syms, spiffs_hooks[i].name, &addr) == 0) {
            rom_stubs_register(rom, addr, spiffs_hooks[i].fn, spiffs_hooks[i].name);
            hooked++;
        }
    }

    /* Generic VFS OK stubs */
    static const char *ok_fns[] = {
        "esp_vfs_spiffs_unregister",
        "esp_spiffs_init",
        "esp_spiffs_check",
        "_ZN2fs8SPIFFSFS3endEv",
        "_ZN2fs8SPIFFSFS6formatEv",
        NULL
    };
    for (int i = 0; ok_fns[i]; i++) {
        if (elf_symbols_find(syms, ok_fns[i], &addr) == 0) {
            rom_stubs_register(rom, addr, stub_vfs_ok, ok_fns[i]);
            hooked++;
        }
    }

    /* High-level stdio wrappers: hook by symbol name when present in firmware. */
    struct { const char *name; void (*fn)(xtensa_cpu_t *, void *); } stdio_hooks[] = {
        { "fwrite", stub_fwrite },
        { "fread",  stub_fread  },
        { "fgets",  stub_fgets  },
        { "fclose", stub_fclose },
        { NULL, NULL }
    };
    for (int i = 0; stdio_hooks[i].name; i++) {
        if (elf_symbols_find(syms, stdio_hooks[i].name, &addr) == 0) {
            rom_stubs_register_ctx(rom, addr, stdio_hooks[i].fn,
                                    stdio_hooks[i].name, v);
            hooked++;
        }
    }
    /* fclose lives in ESP32 ROM at a fixed address; ELF lookup may not see it. */
    rom_stubs_register_ctx(rom, 0x400020ACu, stub_fclose, "fclose@rom", v);
    hooked++;

    /* Newlib stdio dispatch shims at fixed ESP32 ROM addresses.
     * These override the rom_stubs.c default __swrite (which only goes to
     * UART) so that fwrite/fread/fseek/fclose on host-backed FILEs route
     * to the host-side libc. */
    rom_stubs_register_ctx(rom, ROM_SFP,    stub___sfp,    "__sfp",    v);
    rom_stubs_register_ctx(rom, ROM_SWRITE, stub___swrite, "__swrite", v);
    rom_stubs_register_ctx(rom, ROM_SREAD,  stub___sread,  "__sread",  v);
    rom_stubs_register_ctx(rom, ROM_SSEEK,  stub___sseek,  "__sseek",  v);
    rom_stubs_register_ctx(rom, ROM_SCLOSE, stub___sclose, "__sclose", v);
    hooked += 5;

    if (hooked > 0)
        fprintf(stderr, "[VFS] Hooked %d VFS/SPIFFS symbols\n", hooked);

    return hooked;
}
