/* Tests for loading Espressif's ESP32 ROM ELF data into guest ROM. */
#include "test_helpers.h"
#include "rom_elf.h"

#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#pragma pack(push, 1)
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
} rom_test_ehdr_t;

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
} rom_test_shdr_t;

typedef struct {
    uint32_t st_name;
    uint32_t st_value;
    uint32_t st_size;
    uint8_t  st_info;
    uint8_t  st_other;
    uint16_t st_shndx;
} rom_test_sym_t;
#pragma pack(pop)

static const char *build_rom_test_elf(void)
{
    static const char path[] = "/tmp/flexe-rom-elf-test.elf";
    enum {
        STR_BTDM_ROM = 1,
        STR_DATA_START = STR_BTDM_ROM + sizeof("_data_start_btdm_rom"),
        STR_DATA_END = STR_DATA_START + sizeof("_data_start"),
    };
    static const char strtab[] =
        "\0_data_start_btdm_rom\0_data_start\0_data_end\0";
    static const char shstrtab[] =
        "\0.text\0.rodata\0.data_btdm\0.data_test\0.symtab\0.strtab\0"
        ".shstrtab\0";
    enum {
        SHN_TEXT = 1,
        SHN_RODATA = SHN_TEXT + sizeof(".text"),
        SHN_DATA_BTDM = SHN_RODATA + sizeof(".rodata"),
        SHN_DATA_TEST = SHN_DATA_BTDM + sizeof(".data_btdm"),
        SHN_SYMTAB = SHN_DATA_TEST + sizeof(".data_test"),
        SHN_STRTAB = SHN_SYMTAB + sizeof(".symtab"),
        SHN_SHSTRTAB = SHN_STRTAB + sizeof(".strtab"),
        TEXT_OFF = 0x100,
        RODATA_OFF = 0x120,
        DATA_OFF = 0x130,
        DATA_TEST_OFF = 0x140,
        SYMTAB_OFF = 0x150,
        STRTAB_OFF = 0x1A0,
        SHSTRTAB_OFF = 0x1E0,
        SHDR_OFF = 0x240,
        SECTION_COUNT = 8,
        FILE_SIZE = SHDR_OFF + SECTION_COUNT * sizeof(rom_test_shdr_t),
    };

    uint8_t *buf = calloc(1, FILE_SIZE);
    if (!buf) return NULL;

    rom_test_ehdr_t *ehdr = (rom_test_ehdr_t *)buf;
    memcpy(ehdr->e_ident, "\x7f" "ELF", 4);
    ehdr->e_ident[4] = 1;
    ehdr->e_ident[5] = 1;
    ehdr->e_ident[6] = 1;
    ehdr->e_type = 2;
    ehdr->e_machine = 94;
    ehdr->e_version = 1;
    ehdr->e_entry = 0x40000400u;
    ehdr->e_shoff = SHDR_OFF;
    ehdr->e_ehsize = sizeof(*ehdr);
    ehdr->e_shentsize = sizeof(rom_test_shdr_t);
    ehdr->e_shnum = SECTION_COUNT;
    ehdr->e_shstrndx = 7;

    put_le32(buf + TEXT_OFF, 0x40002000u); /* BT data source pointer */
    put_le32(buf + TEXT_OFF + 0x10, 0x3FFB0000u);
    put_le32(buf + TEXT_OFF + 0x14, 0x3FFB0004u); /* logical, not padded end */
    put_le32(buf + TEXT_OFF + 0x18, 0x40003000u);
    put_le32(buf + RODATA_OFF, 0x11223344u);
    put_le32(buf + DATA_OFF, 0xA1B2C3D4u);
    put_le32(buf + DATA_OFF + 4, 0x55667788u);
    put_le32(buf + DATA_TEST_OFF, 0x0BADF00Du);
    put_le32(buf + DATA_TEST_OFF + 4, 0xCAFEBABEu); /* alignment padding */
    rom_test_sym_t syms[4] = {0};
    syms[1].st_name = STR_BTDM_ROM;
    syms[1].st_value = 0x40001000u;
    syms[1].st_info = 0x12; /* STB_GLOBAL | STT_FUNC */
    syms[1].st_shndx = 1;
    syms[2].st_name = STR_DATA_START;
    syms[2].st_value = 0x40001010u;
    syms[2].st_info = 0x10;
    syms[2].st_shndx = 1;
    syms[3].st_name = STR_DATA_END;
    syms[3].st_value = 0x40001020u;
    syms[3].st_info = 0x10;
    syms[3].st_shndx = 1;
    memcpy(buf + SYMTAB_OFF, syms, sizeof(syms));
    memcpy(buf + STRTAB_OFF, strtab, sizeof(strtab));
    memcpy(buf + SHSTRTAB_OFF, shstrtab, sizeof(shstrtab));

    rom_test_shdr_t *sh = (rom_test_shdr_t *)(buf + SHDR_OFF);
    sh[1] = (rom_test_shdr_t){
        .sh_name = SHN_TEXT, .sh_type = 1, .sh_flags = 0x6,
        .sh_addr = 0x40001000u, .sh_offset = TEXT_OFF, .sh_size = 0x20,
    };
    sh[2] = (rom_test_shdr_t){
        .sh_name = SHN_RODATA, .sh_type = 1, .sh_flags = 0x2,
        .sh_addr = 0x3FF96000u, .sh_offset = RODATA_OFF, .sh_size = 4,
    };
    sh[3] = (rom_test_shdr_t){
        .sh_name = SHN_DATA_BTDM, .sh_type = 1, .sh_flags = 0x1,
        .sh_addr = 0x3FFAE6E0u, .sh_offset = DATA_OFF, .sh_size = 8,
    };
    sh[4] = (rom_test_shdr_t){
        .sh_name = SHN_DATA_TEST, .sh_type = 1, .sh_flags = 0x1,
        .sh_addr = 0x3FFB0000u, .sh_offset = DATA_TEST_OFF, .sh_size = 8,
    };
    sh[5] = (rom_test_shdr_t){
        .sh_name = SHN_SYMTAB, .sh_type = 2, .sh_offset = SYMTAB_OFF,
        .sh_size = sizeof(syms), .sh_link = 6,
        .sh_entsize = sizeof(rom_test_sym_t),
    };
    sh[6] = (rom_test_shdr_t){
        .sh_name = SHN_STRTAB, .sh_type = 3, .sh_offset = STRTAB_OFF,
        .sh_size = sizeof(strtab),
    };
    sh[7] = (rom_test_shdr_t){
        .sh_name = SHN_SHSTRTAB, .sh_type = 3, .sh_offset = SHSTRTAB_OFF,
        .sh_size = sizeof(shstrtab),
    };

    FILE *f = fopen(path, "wb");
    if (!f) {
        free(buf);
        return NULL;
    }
    size_t written = fwrite(buf, 1, FILE_SIZE, f);
    fclose(f);
    free(buf);
    return written == FILE_SIZE ? path : NULL;
}

TEST(rom_elf_loads_immutable_sections_and_data_images) {
    const char *path = build_rom_test_elf();
    ASSERT_TRUE(path != NULL);
    xtensa_mem_t *mem = mem_create();
    rom_elf_load_result_t res = rom_elf_load(mem, path);

    ASSERT_EQ(res.result, 0);
    ASSERT_EQ(res.sections_loaded, 2u);
    ASSERT_EQ(res.bytes_loaded, 36u);
    ASSERT_EQ(res.data_images_loaded, 2u);
    ASSERT_EQ(res.data_image_bytes, 12u);
    ASSERT_EQ(mem_read32(mem, 0x40001000u), 0x40002000u);
    ASSERT_EQ(mem_read32(mem, 0x3FF96000u), 0x11223344u);
    ASSERT_EQ(mem_read32(mem, 0x40002000u), 0xA1B2C3D4u);
    ASSERT_EQ(mem_read32(mem, 0x40002004u), 0x55667788u);
    ASSERT_EQ(mem_read32(mem, 0x40003000u), 0x0BADF00Du);
    ASSERT_EQ(mem_read32(mem, 0x40003004u), 0u); /* padding was not copied */
    /* DRAM is initialized later by ROM startup/controller code. */
    ASSERT_EQ(mem_read32(mem, 0x3FFAE6E0u), 0u);
    ASSERT_EQ(mem_unmapped_count(mem), 0u);
    mem_destroy(mem);
}

TEST(rom_elf_rejects_truncated_section) {
    const char *path = build_rom_test_elf();
    ASSERT_TRUE(path != NULL);
    FILE *f = fopen(path, "r+b");
    ASSERT_TRUE(f != NULL);
    if (!f) return;
    rom_test_ehdr_t ehdr;
    ASSERT_EQ(fread(&ehdr, 1, sizeof(ehdr), f), sizeof(ehdr));
    uint32_t bad_offset = UINT32_MAX;
    long field = (long)ehdr.e_shoff + 2L * (long)sizeof(rom_test_shdr_t) +
                 (long)offsetof(rom_test_shdr_t, sh_offset);
    ASSERT_EQ(fseek(f, field, SEEK_SET), 0);
    ASSERT_EQ(fwrite(&bad_offset, 1, sizeof(bad_offset), f),
              sizeof(bad_offset));
    fclose(f);

    xtensa_mem_t *mem = mem_create();
    rom_elf_load_result_t res = rom_elf_load(mem, path);
    ASSERT_EQ(res.result, -1);
    ASSERT_TRUE(strstr(res.error, "past end") != NULL);
    mem_destroy(mem);
}

TEST(rom_elf_rejects_non_xtensa_elf) {
    const char *path = build_rom_test_elf();
    ASSERT_TRUE(path != NULL);
    FILE *f = fopen(path, "r+b");
    ASSERT_TRUE(f != NULL);
    if (!f) return;
    uint16_t wrong_machine = 62; /* EM_X86_64 */
    ASSERT_EQ(fseek(f, (long)offsetof(rom_test_ehdr_t, e_machine), SEEK_SET), 0);
    ASSERT_EQ(fwrite(&wrong_machine, 1, sizeof(wrong_machine), f),
              sizeof(wrong_machine));
    fclose(f);

    xtensa_mem_t *mem = mem_create();
    rom_elf_load_result_t res = rom_elf_load(mem, path);
    ASSERT_EQ(res.result, -1);
    ASSERT_TRUE(strstr(res.error, "Xtensa") != NULL);
    mem_destroy(mem);
}

static void run_rom_elf_tests(void)
{
    TEST_SUITE("ESP32 ROM ELF Loader");
    RUN_TEST(rom_elf_loads_immutable_sections_and_data_images);
    RUN_TEST(rom_elf_rejects_truncated_section);
    RUN_TEST(rom_elf_rejects_non_xtensa_elf);
}
