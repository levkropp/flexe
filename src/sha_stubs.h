#ifndef SHA_STUBS_H
#define SHA_STUBS_H

#include "xtensa.h"
#include "elf_symbols.h"
#include "gdma.h"

typedef struct sha_stubs sha_stubs_t;

sha_stubs_t *sha_stubs_create(xtensa_cpu_t *cpu, flexe_gdma_t *gdma);
void sha_stubs_destroy(sha_stubs_t *ss);

/* Discover and register compatible crypto implementations in stripped
 * firmware without relying on an image identity or link address. */
int sha_stubs_hook_firmware(sha_stubs_t *ss);

/* Look up ELF symbols and register PC hooks for SHA HAL functions */
int sha_stubs_hook_symbols(sha_stubs_t *ss, const elf_symbols_t *syms);

#endif /* SHA_STUBS_H */
