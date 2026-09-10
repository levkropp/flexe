#include "rom_elf.h"

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define ELFCLASS32  1u
#define ELFDATA2LSB 1u
#define EV_CURRENT  1u
#define EM_XTENSA   94u
#define SHT_PROGBITS 1u
#define SHT_SYMTAB   2u
#define SHT_STRTAB   3u
#define SHF_WRITE    0x1u
#define SHF_ALLOC    0x2u

#define ROM_ELF_MAX_BYTES         (64u * 1024u * 1024u)
#define ROM_ELF_MAX_DATA_SECTIONS 32u
#define ROM_DATA_DESC_SIZE        16u
#define ROM_DATA_DESC_MAX_BYTES   4096u

typedef struct {
    uint8_t  e_ident[16];
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

typedef struct {
    const char *name;
    uint32_t addr;
    uint32_t size;
    uint32_t offset;
} rom_data_section_t;

_Static_assert(sizeof(elf32_ehdr_t) == 52, "ELF32 header layout");
_Static_assert(sizeof(elf32_shdr_t) == 40, "ELF32 section layout");
_Static_assert(sizeof(elf32_sym_t) == 16, "ELF32 symbol layout");

static void rom_error(rom_elf_load_result_t *res, const char *fmt, ...)
{
    va_list ap;
    res->result = -1;
    va_start(ap, fmt);
    vsnprintf(res->error, sizeof(res->error), fmt, ap);
    va_end(ap);
}

static int file_range_valid(size_t offset, size_t length, size_t file_size)
{
    return offset <= file_size && length <= file_size - offset;
}

static int read_shdr(const uint8_t *buf, size_t file_size,
                     const elf32_ehdr_t *ehdr, uint32_t index,
                     elf32_shdr_t *out)
{
    if (index >= ehdr->e_shnum || ehdr->e_shentsize < sizeof(*out)) return 0;
    if (index > (SIZE_MAX - ehdr->e_shoff) / ehdr->e_shentsize) return 0;
    size_t off = ehdr->e_shoff + (size_t)index * ehdr->e_shentsize;
    if (!file_range_valid(off, sizeof(*out), file_size)) return 0;
    memcpy(out, buf + off, sizeof(*out));
    return 1;
}

static int section_name(const uint8_t *buf, size_t file_size,
                        const elf32_shdr_t *shstr, uint32_t offset,
                        const char **out)
{
    if (shstr->sh_type != SHT_STRTAB ||
        !file_range_valid(shstr->sh_offset, shstr->sh_size, file_size) ||
        offset >= shstr->sh_size)
        return 0;
    const char *name = (const char *)buf + shstr->sh_offset + offset;
    if (!memchr(name, '\0', (size_t)shstr->sh_size - offset)) return 0;
    *out = name;
    return 1;
}

static int rom_range_contains(const xtensa_mem_t *mem,
                              uint32_t addr, uint32_t size)
{
    return flexe_target_range_uses_backing(mem_target(mem), addr, size,
                                           FLEXE_MEM_ROM);
}

static int guest_range_mapped(xtensa_mem_t *mem, uint32_t addr, uint32_t size)
{
    if (!size) return 1;
    uint64_t end64 = (uint64_t)addr + size;
    if (end64 > (UINT64_C(1) << 32)) return 0;
    uint32_t last = (uint32_t)(end64 - 1u);
    uint32_t page = addr & ~0xFFFu;
    for (;;) {
        if (!mem_get_ptr(mem, page)) return 0;
        if (page >= (last & ~0xFFFu)) break;
        if (page > UINT32_MAX - 0x1000u) return 0;
        page += 0x1000u;
    }
    return 1;
}

/* Return 1 when found, 0 when absent, and -1 for a malformed symbol table. */
static int find_symbol(const uint8_t *buf, size_t file_size,
                       const elf32_ehdr_t *ehdr, const char *wanted,
                       uint32_t *value_out)
{
    for (uint32_t si = 0; si < ehdr->e_shnum; si++) {
        elf32_shdr_t symtab;
        if (!read_shdr(buf, file_size, ehdr, si, &symtab)) return -1;
        if (symtab.sh_type != SHT_SYMTAB) continue;
        if (symtab.sh_entsize < sizeof(elf32_sym_t) ||
            symtab.sh_size % symtab.sh_entsize != 0 ||
            !file_range_valid(symtab.sh_offset, symtab.sh_size, file_size))
            return -1;

        elf32_shdr_t strtab;
        if (!read_shdr(buf, file_size, ehdr, symtab.sh_link, &strtab) ||
            strtab.sh_type != SHT_STRTAB ||
            !file_range_valid(strtab.sh_offset, strtab.sh_size, file_size))
            return -1;

        size_t count = symtab.sh_size / symtab.sh_entsize;
        for (size_t i = 0; i < count; i++) {
            if (i > (SIZE_MAX - symtab.sh_offset) / symtab.sh_entsize)
                return -1;
            size_t off = symtab.sh_offset + i * symtab.sh_entsize;
            elf32_sym_t sym;
            if (!file_range_valid(off, sizeof(sym), file_size)) return -1;
            memcpy(&sym, buf + off, sizeof(sym));
            if (sym.st_name >= strtab.sh_size) return -1;
            const char *name = (const char *)buf + strtab.sh_offset + sym.st_name;
            size_t available = (size_t)strtab.sh_size - sym.st_name;
            if (!memchr(name, '\0', available)) return -1;
            if (strcmp(name, wanted) == 0) {
                *value_out = sym.st_value;
                return 1;
            }
        }
    }
    return 0;
}

static int descriptor_source(xtensa_mem_t *mem,
                             const rom_data_section_t *section,
                             uint32_t table_start, uint32_t table_end,
                             uint32_t *source_out, uint32_t *size_out)
{
    if (table_end < table_start || table_end - table_start > ROM_DATA_DESC_MAX_BYTES ||
        (table_end - table_start) % ROM_DATA_DESC_SIZE != 0 ||
        !guest_range_mapped(mem, table_start, table_end - table_start))
        return -1;

    for (uint32_t p = table_start; p < table_end; p += ROM_DATA_DESC_SIZE) {
        uint32_t start = mem_read32(mem, p);
        uint32_t end = mem_read32(mem, p + 4u);
        if (start == section->addr && end >= start &&
            (uint64_t)end <= (uint64_t)section->addr + section->size) {
            *source_out = mem_read32(mem, p + 8u);
            *size_out = end - start;
            return 1;
        }
    }
    return 0;
}

rom_elf_load_result_t rom_elf_load(xtensa_mem_t *mem, const char *path)
{
    rom_elf_load_result_t res = {0};
    if (!mem) {
        rom_error(&res, "NULL memory");
        return res;
    }
    if (!path || !*path) {
        rom_error(&res, "NULL or empty path");
        return res;
    }

    FILE *f = fopen(path, "rb");
    if (!f) {
        rom_error(&res, "Cannot open file: %s", path);
        return res;
    }
    if (fseek(f, 0, SEEK_END) != 0) {
        rom_error(&res, "Cannot seek file: %s", path);
        fclose(f);
        return res;
    }
    long length = ftell(f);
    if (length < (long)sizeof(elf32_ehdr_t) ||
        (unsigned long)length > ROM_ELF_MAX_BYTES ||
        fseek(f, 0, SEEK_SET) != 0) {
        rom_error(&res, "Invalid ROM ELF size: %ld", length);
        fclose(f);
        return res;
    }

    size_t file_size = (size_t)length;
    uint8_t *buf = malloc(file_size);
    if (!buf) {
        rom_error(&res, "Out of memory reading %s", path);
        fclose(f);
        return res;
    }
    if (fread(buf, 1, file_size, f) != file_size) {
        rom_error(&res, "Truncated read: %s", path);
        free(buf);
        fclose(f);
        return res;
    }
    fclose(f);

    elf32_ehdr_t ehdr;
    memcpy(&ehdr, buf, sizeof(ehdr));
    if (memcmp(ehdr.e_ident, "\x7f" "ELF", 4) != 0 ||
        ehdr.e_ident[4] != ELFCLASS32 || ehdr.e_ident[5] != ELFDATA2LSB ||
        ehdr.e_ident[6] != EV_CURRENT || ehdr.e_machine != EM_XTENSA) {
        rom_error(&res, "Not a 32-bit little-endian Xtensa ELF");
        free(buf);
        return res;
    }
    if (!ehdr.e_shoff || !ehdr.e_shnum || ehdr.e_shstrndx >= ehdr.e_shnum ||
        ehdr.e_shentsize < sizeof(elf32_shdr_t) ||
        ehdr.e_shoff > file_size ||
        ehdr.e_shnum > (file_size - ehdr.e_shoff) / ehdr.e_shentsize) {
        rom_error(&res, "Malformed ELF section table");
        free(buf);
        return res;
    }

    elf32_shdr_t shstr;
    if (!read_shdr(buf, file_size, &ehdr, ehdr.e_shstrndx, &shstr) ||
        shstr.sh_type != SHT_STRTAB ||
        !file_range_valid(shstr.sh_offset, shstr.sh_size, file_size)) {
        rom_error(&res, "Malformed section-name table");
        free(buf);
        return res;
    }

    rom_data_section_t data_sections[ROM_ELF_MAX_DATA_SECTIONS];
    unsigned data_count = 0;
    for (uint32_t i = 0; i < ehdr.e_shnum; i++) {
        elf32_shdr_t sh;
        const char *name;
        if (!read_shdr(buf, file_size, &ehdr, i, &sh) ||
            !section_name(buf, file_size, &shstr, sh.sh_name, &name)) {
            rom_error(&res, "Malformed section %u", i);
            free(buf);
            return res;
        }
        if (sh.sh_type != SHT_PROGBITS || sh.sh_size == 0) continue;
        if (!file_range_valid(sh.sh_offset, sh.sh_size, file_size)) {
            rom_error(&res, "Section %s extends past end of file", name);
            free(buf);
            return res;
        }

        /* The target map, not section flags, is authoritative for mask-ROM
         * contents. Espressif's S3 .rodata.interface is ROM-resident
         * PROGBITS but is marked WRITE and not ALLOC, so filtering only for
         * conventional ELF read-only sections loses required ROM ABI data. */
        int in_rom = rom_range_contains(mem, sh.sh_addr, sh.sh_size);
        if (in_rom) {
            if (!guest_range_mapped(mem, sh.sh_addr, sh.sh_size) ||
                mem_load(mem, sh.sh_addr, buf + sh.sh_offset, sh.sh_size) != 0) {
                rom_error(&res, "ROM section %s does not fit at 0x%08X",
                          name, sh.sh_addr);
                free(buf);
                return res;
            }
            res.sections_loaded++;
            res.bytes_loaded += sh.sh_size;
        } else if ((sh.sh_flags & SHF_ALLOC) &&
                   !(sh.sh_flags & SHF_WRITE)) {
            rom_error(&res, "ROM section %s does not fit at 0x%08X",
                      name, sh.sh_addr);
            free(buf);
            return res;
        }

        /* Espressif ROM ELFs expose stable ROM/application ABI pointers as
         * .data.interface.* snapshots at their live SRAM VMAs. They are not
         * part of the ROM startup-copy table: a direct application handoff
         * must install them explicitly, while BSS remains calloc-zeroed. */
        if (strncmp(name, ".data.interface.", 16) == 0) {
            if (!flexe_target_range_uses_backing(mem_target(mem), sh.sh_addr,
                                                 sh.sh_size, FLEXE_MEM_SRAM) ||
                !guest_range_mapped(mem, sh.sh_addr, sh.sh_size) ||
                mem_load(mem, sh.sh_addr, buf + sh.sh_offset, sh.sh_size) != 0) {
                rom_error(&res,
                          "ROM interface section %s does not fit SRAM at "
                          "0x%08X", name, sh.sh_addr);
                free(buf);
                return res;
            }
            res.interface_sections_loaded++;
            res.interface_bytes_loaded += sh.sh_size;
        }

        if (strncmp(name, ".data_", 6) == 0) {
            if (data_count == ROM_ELF_MAX_DATA_SECTIONS) {
                rom_error(&res, "Too many ROM data sections");
                free(buf);
                return res;
            }
            data_sections[data_count++] = (rom_data_section_t){
                .name = name,
                .addr = sh.sh_addr,
                .size = sh.sh_size,
                .offset = sh.sh_offset,
            };
        }
    }
    if (res.sections_loaded == 0) {
        rom_error(&res, "ELF contains no sections in the target ROM map");
        free(buf);
        return res;
    }

    uint32_t table_start = 0, table_end = 0, btdm_source_slot = 0;
    int have_start = find_symbol(buf, file_size, &ehdr, "_data_start", &table_start);
    int have_end = find_symbol(buf, file_size, &ehdr, "_data_end", &table_end);
    int have_btdm = find_symbol(buf, file_size, &ehdr,
                                "_data_start_btdm_rom", &btdm_source_slot);
    if (have_start < 0 || have_end < 0 || have_btdm < 0 ||
        ((have_start == 1) != (have_end == 1))) {
        rom_error(&res, "Malformed ROM data symbols");
        free(buf);
        return res;
    }

    for (unsigned i = 0; i < data_count; i++) {
        const rom_data_section_t *section = &data_sections[i];
        uint32_t source = 0;
        uint32_t image_size = section->size;
        int found = 0;
        if (strcmp(section->name, ".data_btdm") == 0) {
            if (have_btdm == 1 &&
                guest_range_mapped(mem, btdm_source_slot, sizeof(uint32_t))) {
                source = mem_read32(mem, btdm_source_slot);
                found = 1;
            }
        } else if (have_start == 1) {
            found = descriptor_source(mem, section, table_start, table_end,
                                      &source, &image_size);
        }
        /* APP-CPU data snapshots in Espressif's rev0 ELF are zero-filled and
         * deliberately have no PRO-CPU startup descriptor. The ROM backing
         * is already zero, so there is no load image to reconstruct. */
        if (found == 0) {
            const uint8_t *bytes = buf + section->offset;
            uint32_t j = 0;
            while (j < section->size && bytes[j] == 0) j++;
            if (j == section->size) continue;
        }
        if (found != 1 || !rom_range_contains(mem, source, image_size) ||
            !guest_range_mapped(mem, source, image_size) ||
            mem_load(mem, source, buf + section->offset, image_size) != 0) {
            rom_error(&res, "Cannot locate ROM initializer for %s", section->name);
            free(buf);
            return res;
        }
        const flexe_target_desc_t *target = mem_target(mem);
        if ((target->capabilities &
             FLEXE_TARGET_CAP_DIRECT_ROM_DATA_INIT) != 0u &&
            (!flexe_target_range_uses_backing(mem_target(mem), section->addr,
                                              image_size, FLEXE_MEM_SRAM) ||
             !guest_range_mapped(mem, section->addr, image_size) ||
             mem_load(mem, section->addr, buf + section->offset,
                      image_size) != 0)) {
            rom_error(&res, "Cannot initialize live ROM data for %s",
                      section->name);
            free(buf);
            return res;
        }
        res.data_images_loaded++;
        res.data_image_bytes += image_size;
    }

    res.result = 0;
    free(buf);
    return res;
}
