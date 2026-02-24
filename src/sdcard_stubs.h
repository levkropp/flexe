#ifndef SDCARD_STUBS_H
#define SDCARD_STUBS_H

#include "xtensa.h"
#include "elf_symbols.h"

typedef struct sdcard_stubs sdcard_stubs_t;

sdcard_stubs_t *sdcard_stubs_create(xtensa_cpu_t *cpu);
void sdcard_stubs_destroy(sdcard_stubs_t *ss);

/* Look up ELF symbols and register PC hooks for sdcard functions */
int sdcard_stubs_hook_symbols(sdcard_stubs_t *ss, const elf_symbols_t *syms);

/* Set the backing image file path */
void sdcard_stubs_set_image(sdcard_stubs_t *ss, const char *path);

#endif /* SDCARD_STUBS_H */
