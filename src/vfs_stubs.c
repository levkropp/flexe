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

/* esp_vfs_spiffs_register(conf) -> ESP_OK (0) */
static void stub_vfs_spiffs_register(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    fprintf(stderr, "[VFS] esp_vfs_spiffs_register -> ESP_OK\n");
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
        { NULL, NULL }
    };
    for (int i = 0; ctx_hooks[i].name; i++) {
        if (elf_symbols_find(syms, ctx_hooks[i].name, &addr) == 0) {
            rom_stubs_register_ctx(rom, addr, ctx_hooks[i].fn,
                                    ctx_hooks[i].name, v);
            hooked++;
        }
    }

    /* SPIFFS mount stubs — return success */
    struct { const char *name; void (*fn)(xtensa_cpu_t *, void *); } spiffs_hooks[] = {
        { "esp_vfs_spiffs_register",          stub_vfs_spiffs_register },
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

    if (hooked > 0)
        fprintf(stderr, "[VFS] Hooked %d VFS/SPIFFS symbols\n", hooked);

    return hooked;
}
