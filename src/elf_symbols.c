#include "elf_symbols.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

/* ===== Minimal ELF32 definitions ===== */

#define EI_NIDENT   16
#define ELFMAG      "\x7f""ELF"
#define ELFCLASS32  1
#define ELFDATA2LSB 1
#define SHT_SYMTAB  2
#define STT_OBJECT  1
#define STT_FUNC    2
#define ELF_ST_TYPE(info) ((info) & 0xF)
#define IS_USEFUL_SYM(info) (ELF_ST_TYPE(info) == STT_FUNC || ELF_ST_TYPE(info) == STT_OBJECT)

typedef struct {
    uint8_t  e_ident[EI_NIDENT];
    uint16_t e_type;
    uint16_t e_machine;
    uint32_t e_version;
    uint32_t e_entry;
    uint32_t e_phoff;
    uint32_t e_shoff;
    uint32_t e_flags;
    uint16_t e_ehsize;
    uint16_t e_phentsize;
    uint16_t e_phnum;
    uint16_t e_shentsize;
    uint16_t e_shnum;
    uint16_t e_shstrndx;
} elf32_ehdr_t;

typedef struct {
    uint32_t sh_name;
    uint32_t sh_type;
    uint32_t sh_flags;
    uint32_t sh_addr;
    uint32_t sh_offset;
    uint32_t sh_size;
    uint32_t sh_link;
    uint32_t sh_info;
    uint32_t sh_addralign;
    uint32_t sh_entsize;
} elf32_shdr_t;

typedef struct {
    uint32_t st_name;
    uint32_t st_value;
    uint32_t st_size;
    uint8_t  st_info;
    uint8_t  st_other;
    uint16_t st_shndx;
} elf32_sym_t;

/* ===== Internal types ===== */

typedef struct {
    uint32_t addr;
    uint32_t size;
    uint32_t name_offset;   /* offset into names pool */
} sym_entry_t;

struct elf_symbols {
    sym_entry_t *entries;   /* sorted by addr */
    int          count;
    char        *names;     /* concatenated NUL-terminated strings */
};

/* ===== Comparator for qsort ===== */

static int sym_cmp(const void *a, const void *b) {
    const sym_entry_t *sa = a, *sb = b;
    if (sa->addr < sb->addr) return -1;
    if (sa->addr > sb->addr) return 1;
    return 0;
}

/* ===== Public API ===== */

elf_symbols_t *elf_symbols_load(const char *path) {
    if (!path) return NULL;

    FILE *f = fopen(path, "rb");
    if (!f) return NULL;

    /* Read entire file */
    fseek(f, 0, SEEK_END);
    long fsize = ftell(f);
    fseek(f, 0, SEEK_SET);

    if (fsize < (long)sizeof(elf32_ehdr_t)) {
        fclose(f);
        return NULL;
    }

    uint8_t *buf = malloc((size_t)fsize);
    if (!buf) { fclose(f); return NULL; }
    if (fread(buf, 1, (size_t)fsize, f) != (size_t)fsize) {
        free(buf);
        fclose(f);
        return NULL;
    }
    fclose(f);

    /* Validate ELF header */
    elf32_ehdr_t *ehdr = (elf32_ehdr_t *)buf;
    if (memcmp(ehdr->e_ident, ELFMAG, 4) != 0) {
        free(buf);
        return NULL;
    }
    if (ehdr->e_ident[4] != ELFCLASS32) {
        free(buf);
        return NULL;
    }
    if (ehdr->e_ident[5] != ELFDATA2LSB) {
        free(buf);
        return NULL;
    }

    /* Find .symtab section */
    if (ehdr->e_shoff == 0 || ehdr->e_shnum == 0) {
        free(buf);
        return NULL;
    }

    elf32_shdr_t *shdrs = (elf32_shdr_t *)(buf + ehdr->e_shoff);
    elf32_shdr_t *symtab_shdr = NULL;

    for (int i = 0; i < ehdr->e_shnum; i++) {
        if (shdrs[i].sh_type == SHT_SYMTAB) {
            symtab_shdr = &shdrs[i];
            break;
        }
    }

    if (!symtab_shdr || symtab_shdr->sh_entsize == 0) {
        /* No symbol table — return empty but valid */
        elf_symbols_t *syms = calloc(1, sizeof(*syms));
        free(buf);
        return syms;
    }

    /* Get associated string table */
    uint32_t strtab_idx = symtab_shdr->sh_link;
    if (strtab_idx >= ehdr->e_shnum) {
        free(buf);
        return NULL;
    }
    elf32_shdr_t *strtab_shdr = &shdrs[strtab_idx];
    char *strtab = (char *)(buf + strtab_shdr->sh_offset);

    /* Count useful symbols (FUNC + OBJECT) */
    int nsyms = (int)(symtab_shdr->sh_size / symtab_shdr->sh_entsize);
    elf32_sym_t *elf_syms = (elf32_sym_t *)(buf + symtab_shdr->sh_offset);

    int func_count = 0;
    for (int i = 0; i < nsyms; i++) {
        if (IS_USEFUL_SYM(elf_syms[i].st_info) && elf_syms[i].st_value != 0)
            func_count++;
    }

    /* Allocate result */
    elf_symbols_t *syms = calloc(1, sizeof(*syms));
    if (!syms) { free(buf); return NULL; }

    if (func_count == 0) {
        free(buf);
        return syms;
    }

    syms->entries = malloc((size_t)func_count * sizeof(sym_entry_t));
    if (!syms->entries) { free(syms); free(buf); return NULL; }

    /* First pass: calculate total name length */
    size_t total_names = 0;
    for (int i = 0; i < nsyms; i++) {
        if (IS_USEFUL_SYM(elf_syms[i].st_info) && elf_syms[i].st_value != 0) {
            const char *name = strtab + elf_syms[i].st_name;
            total_names += strlen(name) + 1;
        }
    }

    syms->names = malloc(total_names);
    if (!syms->names) { free(syms->entries); free(syms); free(buf); return NULL; }

    /* Second pass: copy symbols and names */
    int idx = 0;
    size_t name_off = 0;
    for (int i = 0; i < nsyms; i++) {
        if (!IS_USEFUL_SYM(elf_syms[i].st_info) || elf_syms[i].st_value == 0)
            continue;
        const char *name = strtab + elf_syms[i].st_name;
        size_t nlen = strlen(name) + 1;
        memcpy(syms->names + name_off, name, nlen);
        syms->entries[idx].addr = elf_syms[i].st_value;
        syms->entries[idx].size = elf_syms[i].st_size;
        syms->entries[idx].name_offset = (uint32_t)name_off;
        name_off += nlen;
        idx++;
    }
    syms->count = func_count;

    /* Sort by address */
    qsort(syms->entries, (size_t)syms->count, sizeof(sym_entry_t), sym_cmp);

    free(buf);
    return syms;
}

void elf_symbols_destroy(elf_symbols_t *syms) {
    if (!syms) return;
    free(syms->entries);
    free(syms->names);
    free(syms);
}

int elf_symbols_lookup(const elf_symbols_t *syms, uint32_t addr, elf_sym_info_t *out) {
    if (!syms || syms->count == 0) return 0;

    /* Binary search: find largest entry.addr <= addr */
    int lo = 0, hi = syms->count - 1, best = -1;
    while (lo <= hi) {
        int mid = lo + (hi - lo) / 2;
        if (syms->entries[mid].addr <= addr) {
            best = mid;
            lo = mid + 1;
        } else {
            hi = mid - 1;
        }
    }

    if (best < 0) return 0;

    sym_entry_t *e = &syms->entries[best];
    uint32_t offset = addr - e->addr;

    /* If symbol has a known size, check bounds */
    if (e->size > 0 && offset >= e->size) return 0;

    if (out) {
        out->name = syms->names + e->name_offset;
        out->addr = e->addr;
        out->size = e->size;
        out->offset = offset;
    }
    return 1;
}

int elf_symbols_find(const elf_symbols_t *syms, const char *name, uint32_t *addr_out) {
    if (!syms || !name) return -1;
    for (int i = 0; i < syms->count; i++) {
        if (strcmp(syms->names + syms->entries[i].name_offset, name) == 0) {
            if (addr_out) *addr_out = syms->entries[i].addr;
            return 0;
        }
    }
    return -1;
}

int elf_symbols_count(const elf_symbols_t *syms) {
    return syms ? syms->count : 0;
}
