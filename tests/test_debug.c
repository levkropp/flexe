/*
 * Tests for M11 debug tooling: ELF symbol loading, breakpoints, ROM stub stats.
 */
#include "test_helpers.h"
#include "elf_symbols.h"
#include "rom_stubs.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

/* ===== Minimal ELF32 builder ===== */

#define EI_NIDENT 16

#pragma pack(push, 1)
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
} test_ehdr_t;

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
} test_shdr_t;

typedef struct {
    uint32_t st_name;
    uint32_t st_value;
    uint32_t st_size;
    uint8_t  st_info;
    uint8_t  st_other;
    uint16_t st_shndx;
} test_sym_t;
#pragma pack(pop)

#define SHT_NULL    0
#define SHT_SYMTAB  2
#define SHT_STRTAB  3
#define STT_FUNC    2
#define STT_OBJECT  1
#define STB_GLOBAL  1
#define ELF_ST_INFO(b,t) (((b)<<4)|((t)&0xf))

/*
 * Build a minimal valid ELF with FUNC symbols at /tmp/xt_test.elf
 * Symbols: app_main @ 0x40080000 size 0x100
 *          uart_init @ 0x40080100 size 0x40
 *          some_data (OBJECT, should be skipped) @ 0x3FFB0000 size 4
 */
static const char *build_test_elf(void) {
    static const char *path = "/tmp/xt_test_debug.elf";

    /* String table: \0 app_main\0 uart_init\0 some_data\0 .symtab\0 .strtab\0 .shstrtab\0 */
    const char strtab[] = "\0app_main\0uart_init\0some_data\0";
    int strtab_size = sizeof(strtab);

    /* Section header string table */
    const char shstrtab[] = "\0.symtab\0.strtab\0.shstrtab\0";
    int shstrtab_size = sizeof(shstrtab);

    /* Symbol table: null sym + 3 symbols */
    test_sym_t syms[4];
    memset(syms, 0, sizeof(syms));
    /* [0] = null */
    /* [1] = app_main: name_idx=1, value=0x40080000, size=0x100, FUNC */
    syms[1].st_name = 1;
    syms[1].st_value = 0x40080000;
    syms[1].st_size = 0x100;
    syms[1].st_info = ELF_ST_INFO(STB_GLOBAL, STT_FUNC);
    syms[1].st_shndx = 1;
    /* [2] = uart_init: name_idx=10, value=0x40080100, size=0x40, FUNC */
    syms[2].st_name = 10;
    syms[2].st_value = 0x40080100;
    syms[2].st_size = 0x40;
    syms[2].st_info = ELF_ST_INFO(STB_GLOBAL, STT_FUNC);
    syms[2].st_shndx = 1;
    /* [3] = some_data: name_idx=20, value=0x3FFB0000, size=4, OBJECT */
    syms[3].st_name = 20;
    syms[3].st_value = 0x3FFB0000;
    syms[3].st_size = 4;
    syms[3].st_info = ELF_ST_INFO(STB_GLOBAL, STT_OBJECT);
    syms[3].st_shndx = 2;
    int symtab_size = (int)sizeof(syms);

    /* Layout:
     * [0x00] ELF header (52 bytes)
     * [0x34] strtab data
     * [0x34+strtab_size] symtab data
     * [0x34+strtab_size+symtab_size] shstrtab data
     * [aligned] section headers (5 sections: null, .symtab, .strtab, .shstrtab, sentinel-free)
     */
    uint32_t strtab_off = 52;
    uint32_t symtab_off = strtab_off + (uint32_t)strtab_size;
    uint32_t shstrtab_off = symtab_off + (uint32_t)symtab_size;
    uint32_t shdr_off = shstrtab_off + (uint32_t)shstrtab_size;
    /* Align to 4 */
    shdr_off = (shdr_off + 3) & ~3u;

    int num_sections = 4;  /* null, .symtab, .strtab, .shstrtab */
    uint32_t total = shdr_off + (uint32_t)(num_sections * (int)sizeof(test_shdr_t));

    uint8_t *buf = calloc(1, total);
    if (!buf) return NULL;

    /* ELF header */
    test_ehdr_t *ehdr = (test_ehdr_t *)buf;
    ehdr->e_ident[0] = 0x7f;
    ehdr->e_ident[1] = 'E';
    ehdr->e_ident[2] = 'L';
    ehdr->e_ident[3] = 'F';
    ehdr->e_ident[4] = 1;  /* ELFCLASS32 */
    ehdr->e_ident[5] = 1;  /* ELFDATA2LSB */
    ehdr->e_ident[6] = 1;  /* EV_CURRENT */
    ehdr->e_type = 2;       /* ET_EXEC */
    ehdr->e_machine = 94;   /* EM_XTENSA */
    ehdr->e_version = 1;
    ehdr->e_entry = 0x40080000;
    ehdr->e_shoff = shdr_off;
    ehdr->e_ehsize = 52;
    ehdr->e_shentsize = (uint16_t)sizeof(test_shdr_t);
    ehdr->e_shnum = (uint16_t)num_sections;
    ehdr->e_shstrndx = 3;   /* .shstrtab is section 3 */

    /* Copy data sections */
    memcpy(buf + strtab_off, strtab, (size_t)strtab_size);
    memcpy(buf + symtab_off, syms, (size_t)symtab_size);
    memcpy(buf + shstrtab_off, shstrtab, (size_t)shstrtab_size);

    /* Section headers */
    test_shdr_t *shdrs = (test_shdr_t *)(buf + shdr_off);
    /* [0] null */
    /* [1] .symtab */
    shdrs[1].sh_name = 1;  /* offset in shstrtab */
    shdrs[1].sh_type = SHT_SYMTAB;
    shdrs[1].sh_offset = symtab_off;
    shdrs[1].sh_size = (uint32_t)symtab_size;
    shdrs[1].sh_link = 2;  /* associated strtab is section 2 */
    shdrs[1].sh_entsize = (uint32_t)sizeof(test_sym_t);
    /* [2] .strtab */
    shdrs[2].sh_name = 9;  /* offset in shstrtab */
    shdrs[2].sh_type = SHT_STRTAB;
    shdrs[2].sh_offset = strtab_off;
    shdrs[2].sh_size = (uint32_t)strtab_size;
    /* [3] .shstrtab */
    shdrs[3].sh_name = 17;  /* offset in shstrtab */
    shdrs[3].sh_type = SHT_STRTAB;
    shdrs[3].sh_offset = shstrtab_off;
    shdrs[3].sh_size = (uint32_t)shstrtab_size;

    FILE *f = fopen(path, "wb");
    if (!f) { free(buf); return NULL; }
    fwrite(buf, 1, total, f);
    fclose(f);
    free(buf);
    return path;
}

/* ===== NOP instruction builder ===== */

static uint32_t dbg_nop_insn(void) {
    return rrr(0, 0, 2, 15, 0);
}

/* ===== ELF Symbol Tests ===== */

TEST(test_elf_load_null_path) {
    elf_symbols_t *s = elf_symbols_load(NULL);
    ASSERT_TRUE(s == NULL);
}

TEST(test_elf_load_nonexistent) {
    elf_symbols_t *s = elf_symbols_load("/tmp/nonexistent_elf_file_12345.elf");
    ASSERT_TRUE(s == NULL);
}

TEST(test_elf_load_bad_magic) {
    /* Write a file with bad magic */
    const char *path = "/tmp/xt_test_badmagic.elf";
    FILE *f = fopen(path, "wb");
    uint8_t data[] = { 0x00, 0x00, 0x00, 0x00 };
    fwrite(data, 1, sizeof(data), f);
    fclose(f);

    elf_symbols_t *s = elf_symbols_load(path);
    ASSERT_TRUE(s == NULL);
}

TEST(test_elf_load_bad_class) {
    /* Write ELF with class=64-bit */
    const char *path = "/tmp/xt_test_badclass.elf";
    uint8_t buf[64];
    memset(buf, 0, sizeof(buf));
    buf[0] = 0x7f; buf[1] = 'E'; buf[2] = 'L'; buf[3] = 'F';
    buf[4] = 2;  /* ELFCLASS64 — should fail */
    buf[5] = 1;  /* ELFDATA2LSB */
    FILE *f = fopen(path, "wb");
    fwrite(buf, 1, sizeof(buf), f);
    fclose(f);

    elf_symbols_t *s = elf_symbols_load(path);
    ASSERT_TRUE(s == NULL);
}

TEST(test_elf_load_valid) {
    const char *path = build_test_elf();
    ASSERT_TRUE(path != NULL);

    elf_symbols_t *s = elf_symbols_load(path);
    ASSERT_TRUE(s != NULL);
    /* Should have 2 FUNC symbols (app_main, uart_init), not some_data (OBJECT) */
    ASSERT_EQ(elf_symbols_count(s), 2);

    elf_symbols_destroy(s);
}

TEST(test_elf_lookup_exact) {
    const char *path = build_test_elf();
    elf_symbols_t *s = elf_symbols_load(path);
    ASSERT_TRUE(s != NULL);

    elf_sym_info_t info;
    int found = elf_symbols_lookup(s, 0x40080000, &info);
    ASSERT_TRUE(found);
    ASSERT_TRUE(strcmp(info.name, "app_main") == 0);
    ASSERT_EQ(info.addr, 0x40080000);
    ASSERT_EQ(info.offset, 0);
    ASSERT_EQ(info.size, 0x100);

    elf_symbols_destroy(s);
}

TEST(test_elf_lookup_with_offset) {
    const char *path = build_test_elf();
    elf_symbols_t *s = elf_symbols_load(path);

    elf_sym_info_t info;
    int found = elf_symbols_lookup(s, 0x40080010, &info);
    ASSERT_TRUE(found);
    ASSERT_TRUE(strcmp(info.name, "app_main") == 0);
    ASSERT_EQ(info.offset, 0x10);

    elf_symbols_destroy(s);
}

TEST(test_elf_lookup_no_match) {
    const char *path = build_test_elf();
    elf_symbols_t *s = elf_symbols_load(path);

    elf_sym_info_t info;
    /* Address before any symbol */
    int found = elf_symbols_lookup(s, 0x10000000, &info);
    ASSERT_FALSE(found);

    elf_symbols_destroy(s);
}

TEST(test_elf_lookup_size_boundary) {
    const char *path = build_test_elf();
    elf_symbols_t *s = elf_symbols_load(path);

    elf_sym_info_t info;
    /* app_main is 0x40080000 size 0x100, so 0x40080100 is OUT of range */
    /* But uart_init starts at 0x40080100, so it should match uart_init */
    int found = elf_symbols_lookup(s, 0x40080100, &info);
    ASSERT_TRUE(found);
    ASSERT_TRUE(strcmp(info.name, "uart_init") == 0);
    ASSERT_EQ(info.offset, 0);

    /* Beyond uart_init (0x40080100 + 0x40 = 0x40080140) — no match */
    found = elf_symbols_lookup(s, 0x40080140, &info);
    ASSERT_FALSE(found);

    elf_symbols_destroy(s);
}

TEST(test_elf_find_by_name) {
    const char *path = build_test_elf();
    elf_symbols_t *s = elf_symbols_load(path);

    uint32_t addr;
    int rc = elf_symbols_find(s, "uart_init", &addr);
    ASSERT_EQ(rc, 0);
    ASSERT_EQ(addr, 0x40080100);

    elf_symbols_destroy(s);
}

TEST(test_elf_find_not_found) {
    const char *path = build_test_elf();
    elf_symbols_t *s = elf_symbols_load(path);

    uint32_t addr;
    int rc = elf_symbols_find(s, "nonexistent_func", &addr);
    ASSERT_EQ(rc, (uint32_t)-1);

    elf_symbols_destroy(s);
}

TEST(test_elf_skips_non_func) {
    const char *path = build_test_elf();
    elf_symbols_t *s = elf_symbols_load(path);

    /* some_data is an OBJECT symbol — should NOT be found */
    uint32_t addr;
    int rc = elf_symbols_find(s, "some_data", &addr);
    ASSERT_EQ(rc, (uint32_t)-1);

    elf_symbols_destroy(s);
}

/* ===== Breakpoint Tests ===== */

TEST(test_bp_set_and_hit) {
    xtensa_cpu_t cpu;
    setup(&cpu);

    /* Place NOPs at BASE, BASE+3, BASE+6 */
    put_insn3(&cpu, BASE, dbg_nop_insn());
    put_insn3(&cpu, BASE + 3, dbg_nop_insn());
    put_insn3(&cpu, BASE + 6, dbg_nop_insn());

    /* Set breakpoint at BASE+3 */
    int rc = xtensa_set_breakpoint(&cpu, BASE + 3);
    ASSERT_EQ(rc, 0);

    /* Step once — NOP at BASE */
    xtensa_step(&cpu);
    ASSERT_EQ(cpu.pc, BASE + 3);
    ASSERT_FALSE(cpu.breakpoint_hit);

    /* Step again — should hit breakpoint at BASE+3 */
    rc = xtensa_step(&cpu);
    ASSERT_EQ(rc, (uint32_t)-1);
    ASSERT_TRUE(cpu.breakpoint_hit);
    ASSERT_EQ(cpu.breakpoint_hit_addr, BASE + 3);
    /* PC should still be at BASE+3 (didn't execute) */
    ASSERT_EQ(cpu.pc, BASE + 3);

    teardown(&cpu);
}

TEST(test_bp_clear) {
    xtensa_cpu_t cpu;
    setup(&cpu);

    put_insn3(&cpu, BASE, dbg_nop_insn());
    put_insn3(&cpu, BASE + 3, dbg_nop_insn());
    put_insn3(&cpu, BASE + 6, dbg_nop_insn());

    xtensa_set_breakpoint(&cpu, BASE + 3);
    xtensa_clear_breakpoint(&cpu, BASE + 3);

    /* Step twice — should pass through BASE+3 without stopping */
    xtensa_step(&cpu);
    ASSERT_EQ(cpu.pc, BASE + 3);
    xtensa_step(&cpu);
    ASSERT_EQ(cpu.pc, BASE + 6);
    ASSERT_FALSE(cpu.breakpoint_hit);

    teardown(&cpu);
}

TEST(test_bp_multiple) {
    xtensa_cpu_t cpu;
    setup(&cpu);

    put_insn3(&cpu, BASE, dbg_nop_insn());
    put_insn3(&cpu, BASE + 3, dbg_nop_insn());
    put_insn3(&cpu, BASE + 6, dbg_nop_insn());

    xtensa_set_breakpoint(&cpu, BASE + 3);
    xtensa_set_breakpoint(&cpu, BASE + 6);

    /* Should hit first breakpoint */
    xtensa_step(&cpu);  /* execute NOP at BASE */
    xtensa_step(&cpu);  /* hit BP at BASE+3 */
    ASSERT_TRUE(cpu.breakpoint_hit);
    ASSERT_EQ(cpu.breakpoint_hit_addr, BASE + 3);

    teardown(&cpu);
}

TEST(test_bp_max_exceeded) {
    xtensa_cpu_t cpu;
    setup(&cpu);

    for (int i = 0; i < MAX_BREAKPOINTS; i++)
        ASSERT_EQ(xtensa_set_breakpoint(&cpu, BASE + (uint32_t)(i * 3)), 0);

    /* 17th should fail */
    int rc = xtensa_set_breakpoint(&cpu, 0xDEADBEEF);
    ASSERT_EQ(rc, (uint32_t)-1);

    teardown(&cpu);
}

TEST(test_bp_duplicate) {
    xtensa_cpu_t cpu;
    setup(&cpu);

    ASSERT_EQ(xtensa_set_breakpoint(&cpu, BASE), 0);
    ASSERT_EQ(xtensa_set_breakpoint(&cpu, BASE), 0);  /* duplicate ok */
    ASSERT_EQ(cpu.breakpoint_count, 1);  /* still just one */

    teardown(&cpu);
}

/* ===== ROM Stub Stats Test ===== */

TEST(test_rom_stub_call_count) {
    xtensa_cpu_t cpu;
    setup(&cpu);

    esp32_rom_stubs_t *rom = rom_stubs_create(&cpu);

    /* Call ets_write_char_uart 3 times */
    for (int i = 0; i < 3; i++) {
        cpu.pc = 0x40007cf8;
        XT_PS_SET_CALLINC(cpu.ps, 0);
        ar_write(&cpu, 0, BASE);
        ar_write(&cpu, 2, 'A');
        xtensa_step(&cpu);
    }

    /* Find ets_write_char_uart in stats */
    int nstubs = rom_stubs_stub_count(rom);
    ASSERT_TRUE(nstubs > 0);

    int found = 0;
    for (int i = 0; i < nstubs; i++) {
        const char *name;
        uint32_t addr, count;
        rom_stubs_get_stats(rom, i, &name, &addr, &count);
        if (addr == 0x40007cf8) {
            ASSERT_EQ(count, 3);
            found = 1;
            break;
        }
    }
    ASSERT_TRUE(found);

    rom_stubs_destroy(rom);
    teardown(&cpu);
}

/* ===== Run all ===== */

static void run_debug_tests(void) {
    TEST_SUITE("ELF Symbols");
    RUN_TEST(test_elf_load_null_path);
    RUN_TEST(test_elf_load_nonexistent);
    RUN_TEST(test_elf_load_bad_magic);
    RUN_TEST(test_elf_load_bad_class);
    RUN_TEST(test_elf_load_valid);
    RUN_TEST(test_elf_lookup_exact);
    RUN_TEST(test_elf_lookup_with_offset);
    RUN_TEST(test_elf_lookup_no_match);
    RUN_TEST(test_elf_lookup_size_boundary);
    RUN_TEST(test_elf_find_by_name);
    RUN_TEST(test_elf_find_not_found);
    RUN_TEST(test_elf_skips_non_func);

    TEST_SUITE("Breakpoints");
    RUN_TEST(test_bp_set_and_hit);
    RUN_TEST(test_bp_clear);
    RUN_TEST(test_bp_multiple);
    RUN_TEST(test_bp_max_exceeded);
    RUN_TEST(test_bp_duplicate);

    TEST_SUITE("ROM Stub Stats");
    RUN_TEST(test_rom_stub_call_count);
}
