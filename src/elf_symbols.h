#ifndef ELF_SYMBOLS_H
#define ELF_SYMBOLS_H

#include <stdint.h>

typedef struct elf_symbols elf_symbols_t;

/* Result of a symbol lookup */
typedef struct {
    const char *name;       /* Symbol name (owned by elf_symbols_t) */
    uint32_t    addr;       /* Symbol start address */
    uint32_t    size;       /* Symbol size (0 if unknown) */
    uint32_t    offset;     /* query_addr - sym_addr */
} elf_sym_info_t;

/* Load symbols from an ELF file. Returns NULL on error. */
elf_symbols_t *elf_symbols_load(const char *path);
void           elf_symbols_destroy(elf_symbols_t *syms);

/* Lookup by address: returns 1 if found, 0 if not */
int elf_symbols_lookup(const elf_symbols_t *syms, uint32_t addr, elf_sym_info_t *out);

/* Find by name: returns 0 on success, -1 if not found */
int elf_symbols_find(const elf_symbols_t *syms, const char *name, uint32_t *addr_out);

/* Number of loaded symbols */
int elf_symbols_count(const elf_symbols_t *syms);

#endif /* ELF_SYMBOLS_H */
