#include "elf_symbols.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <limits.h>

/* ===== Minimal ELF32 definitions ===== */

#define EI_NIDENT   16
#define ELFMAG      "\x7f""ELF"
#define ELFCLASS32  1
#define ELFDATA2LSB 1
#define SHT_SYMTAB  2
#define SHT_STRTAB  3
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

static int file_range_valid(size_t offset, size_t length, size_t file_size) {
    return offset <= file_size && length <= file_size - offset;
}

static int elf_read_shdr(const uint8_t *buf, size_t file_size,
                         const elf32_ehdr_t *ehdr, uint32_t index,
                         elf32_shdr_t *out) {
    if (index >= ehdr->e_shnum) return 0;
    size_t stride = ehdr->e_shentsize;
    size_t table_off = ehdr->e_shoff;
    if (stride < sizeof(*out) || index > (SIZE_MAX - table_off) / stride)
        return 0;
    size_t offset = table_off + (size_t)index * stride;
    if (!file_range_valid(offset, sizeof(*out), file_size)) return 0;
    memcpy(out, buf + offset, sizeof(*out));
    return 1;
}

static int elf_read_sym(const uint8_t *buf, size_t file_size,
                        const elf32_shdr_t *symtab, size_t index,
                        elf32_sym_t *out) {
    size_t stride = symtab->sh_entsize;
    size_t table_off = symtab->sh_offset;
    if (stride < sizeof(*out) || index > (SIZE_MAX - table_off) / stride)
        return 0;
    size_t offset = table_off + index * stride;
    if (!file_range_valid(offset, sizeof(*out), file_size)) return 0;
    memcpy(out, buf + offset, sizeof(*out));
    return 1;
}

static int elf_get_name(const uint8_t *buf, const elf32_shdr_t *strtab,
                        uint32_t name_index, const char **name_out,
                        size_t *length_out) {
    if (name_index >= strtab->sh_size) return 0;
    const char *name = (const char *)buf + strtab->sh_offset + name_index;
    size_t available = (size_t)strtab->sh_size - name_index;
    const char *end = memchr(name, '\0', available);
    if (!end) return 0;
    *name_out = name;
    *length_out = (size_t)(end - name) + 1u;
    return 1;
}

static elf_symbols_t *elf_symbols_empty(void) {
    return calloc(1, sizeof(elf_symbols_t));
}

/* ===== Public API ===== */

elf_symbols_t *elf_symbols_load(const char *path) {
    if (!path) return NULL;

    FILE *f = fopen(path, "rb");
    if (!f) return NULL;

    /* Read entire file */
    if (fseek(f, 0, SEEK_END) != 0) {
        fclose(f);
        return NULL;
    }
    long file_length = ftell(f);
    if (file_length < 0 || fseek(f, 0, SEEK_SET) != 0) {
        fclose(f);
        return NULL;
    }

    if (file_length < (long)sizeof(elf32_ehdr_t)) {
        fclose(f);
        return NULL;
    }
    size_t file_size = (size_t)file_length;
    if ((long)file_size != file_length) {
        fclose(f);
        return NULL;
    }

    uint8_t *buf = malloc(file_size);
    if (!buf) { fclose(f); return NULL; }
    if (fread(buf, 1, file_size, f) != file_size) {
        free(buf);
        fclose(f);
        return NULL;
    }
    fclose(f);

    /* Validate ELF header */
    elf32_ehdr_t ehdr;
    memcpy(&ehdr, buf, sizeof(ehdr));
    if (memcmp(ehdr.e_ident, ELFMAG, 4) != 0) {
        free(buf);
        return NULL;
    }
    if (ehdr.e_ident[4] != ELFCLASS32) {
        free(buf);
        return NULL;
    }
    if (ehdr.e_ident[5] != ELFDATA2LSB) {
        free(buf);
        return NULL;
    }

    /* Find .symtab section */
    if (ehdr.e_shoff == 0 || ehdr.e_shnum == 0 ||
        ehdr.e_shentsize < sizeof(elf32_shdr_t) ||
        !file_range_valid(ehdr.e_shoff, 0, file_size) ||
        ehdr.e_shnum > (file_size - ehdr.e_shoff) / ehdr.e_shentsize) {
        free(buf);
        return NULL;
    }

    elf32_shdr_t symtab_shdr;
    int found_symtab = 0;

    for (uint32_t i = 0; i < ehdr.e_shnum; i++) {
        elf32_shdr_t shdr;
        if (!elf_read_shdr(buf, file_size, &ehdr, i, &shdr)) {
            free(buf);
            return NULL;
        }
        if (shdr.sh_type == SHT_SYMTAB) {
            symtab_shdr = shdr;
            found_symtab = 1;
            break;
        }
    }

    if (!found_symtab) {
        /* No symbol table — return empty but valid */
        elf_symbols_t *syms = elf_symbols_empty();
        free(buf);
        return syms;
    }

    if (symtab_shdr.sh_entsize < sizeof(elf32_sym_t) ||
        symtab_shdr.sh_size % symtab_shdr.sh_entsize != 0 ||
        !file_range_valid(symtab_shdr.sh_offset, symtab_shdr.sh_size, file_size)) {
        free(buf);
        return NULL;
    }

    /* Get associated string table */
    uint32_t strtab_idx = symtab_shdr.sh_link;
    elf32_shdr_t strtab_shdr;
    if (!elf_read_shdr(buf, file_size, &ehdr, strtab_idx, &strtab_shdr) ||
        strtab_shdr.sh_type != SHT_STRTAB ||
        !file_range_valid(strtab_shdr.sh_offset, strtab_shdr.sh_size, file_size)) {
        free(buf);
        return NULL;
    }

    /* Count useful symbols (FUNC + OBJECT) */
    size_t nsyms = symtab_shdr.sh_size / symtab_shdr.sh_entsize;
    size_t useful_count = 0;
    size_t total_names = 0;
    for (size_t i = 0; i < nsyms; i++) {
        elf32_sym_t sym;
        if (!elf_read_sym(buf, file_size, &symtab_shdr, i, &sym)) {
            free(buf);
            return NULL;
        }
        if (!IS_USEFUL_SYM(sym.st_info) || sym.st_value == 0) continue;
        const char *name;
        size_t name_len;
        if (!elf_get_name(buf, &strtab_shdr, sym.st_name, &name, &name_len) ||
            name_len > SIZE_MAX - total_names) {
            free(buf);
            return NULL;
        }
        total_names += name_len;
        useful_count++;
    }

    if (useful_count > INT_MAX || total_names > UINT32_MAX) {
        free(buf);
        return NULL;
    }

    /* Allocate result */
    elf_symbols_t *syms = calloc(1, sizeof(*syms));
    if (!syms) { free(buf); return NULL; }

    if (useful_count == 0) {
        free(buf);
        return syms;
    }

    if (useful_count > SIZE_MAX / sizeof(sym_entry_t)) {
        free(syms);
        free(buf);
        return NULL;
    }
    syms->entries = malloc(useful_count * sizeof(sym_entry_t));
    if (!syms->entries) { free(syms); free(buf); return NULL; }

    syms->names = malloc(total_names);
    if (!syms->names) { free(syms->entries); free(syms); free(buf); return NULL; }

    /* Second pass: copy symbols and names */
    size_t idx = 0;
    size_t name_off = 0;
    for (size_t i = 0; i < nsyms; i++) {
        elf32_sym_t sym;
        const char *name;
        size_t nlen;
        if (!elf_read_sym(buf, file_size, &symtab_shdr, i, &sym)) goto malformed;
        if (!IS_USEFUL_SYM(sym.st_info) || sym.st_value == 0) continue;
        if (!elf_get_name(buf, &strtab_shdr, sym.st_name, &name, &nlen))
            goto malformed;
        memcpy(syms->names + name_off, name, nlen);
        syms->entries[idx].addr = sym.st_value;
        syms->entries[idx].size = sym.st_size;
        syms->entries[idx].name_offset = (uint32_t)name_off;
        name_off += nlen;
        idx++;
    }
    syms->count = (int)useful_count;

    /* Sort by address */
    qsort(syms->entries, (size_t)syms->count, sizeof(sym_entry_t), sym_cmp);

    free(buf);
    return syms;

malformed:
    free(syms->names);
    free(syms->entries);
    free(syms);
    free(buf);
    return NULL;
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

int elf_symbols_iterate(const elf_symbols_t *syms, elf_symbols_iter_fn callback, void *ctx) {
    if (!syms || !callback) return 0;

    for (int i = 0; i < syms->count; i++) {
        const char *name = syms->names + syms->entries[i].name_offset;
        uint32_t addr = syms->entries[i].addr;
        uint32_t size = syms->entries[i].size;

        /* Call callback - stop if it returns non-zero */
        if (callback(name, addr, size, ctx) != 0)
            return i + 1;  /* Return count of symbols processed so far */
    }

    return syms->count;  /* Processed all symbols */
}
