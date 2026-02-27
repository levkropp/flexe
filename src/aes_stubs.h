#ifndef AES_STUBS_H
#define AES_STUBS_H

#include "xtensa.h"
#include "elf_symbols.h"

typedef struct aes_stubs aes_stubs_t;

aes_stubs_t *aes_stubs_create(xtensa_cpu_t *cpu);
void aes_stubs_destroy(aes_stubs_t *as);

/* Look up ELF symbols and register PC hooks for AES HAL functions */
int aes_stubs_hook_symbols(aes_stubs_t *as, const elf_symbols_t *syms);

#endif /* AES_STUBS_H */
