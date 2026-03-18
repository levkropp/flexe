#ifndef VFS_STUBS_H
#define VFS_STUBS_H

#include "xtensa.h"
#include "elf_symbols.h"

typedef struct vfs_stubs vfs_stubs_t;

vfs_stubs_t *vfs_stubs_create(xtensa_cpu_t *cpu);
void vfs_stubs_destroy(vfs_stubs_t *v);

/* Set host directory that maps to /spiffs/ in guest.
 * e.g., vfs_stubs_set_spiffs_dir(v, "./spiffs_data")
 * means guest open("/spiffs/config.json") -> host open("./spiffs_data/config.json") */
void vfs_stubs_set_spiffs_dir(vfs_stubs_t *v, const char *host_dir);

/* Hook firmware symbols for VFS file operations */
int vfs_stubs_hook_symbols(vfs_stubs_t *v, const elf_symbols_t *syms);

#endif /* VFS_STUBS_H */
