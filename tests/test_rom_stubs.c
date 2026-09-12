/*
 * Tests for ROM function stubs (M10).
 * PC hook mechanism, calling convention, and built-in ROM stubs.
 */
#include "test_helpers.h"
#include "peripherals.h"
#include "rom_stubs.h"
#include <string.h>

/* NOP: op0=0, op1=0, op2=0, r=2, s=0, t=15 */
static uint32_t rom_nop_insn(void) {
    return rrr(0, 0, 2, 0, 15);
}

/* ===== Test: pc_hook fires ===== */

static int test_hook_fired;
static uint32_t test_hook_captured_pc;

static int test_pc_hook_cb(xtensa_cpu_t *cpu, uint32_t pc, void *ctx) {
    (void)ctx;
    if (pc == 0x40001000) {
        test_hook_fired = 1;
        test_hook_captured_pc = pc;
        cpu->pc = BASE;  /* redirect to avoid infinite loop */
        return 1;
    }
    return 0;
}

TEST(test_pc_hook_fires) {
    xtensa_cpu_t cpu;
    setup(&cpu);
    cpu.pc = 0x40001000;
    cpu.pc_hook = test_pc_hook_cb;
    cpu.pc_hook_ctx = NULL;
    test_hook_fired = 0;
    test_hook_captured_pc = 0;

    xtensa_step(&cpu);
    ASSERT_TRUE(test_hook_fired);
    ASSERT_EQ(test_hook_captured_pc, 0x40001000);
    teardown(&cpu);
}

/* ===== Test: pc_hook skips non-match ===== */

TEST(test_pc_hook_skips_non_match) {
    xtensa_cpu_t cpu;
    setup(&cpu);
    cpu.pc = BASE;
    cpu.pc_hook = test_pc_hook_cb;
    cpu.pc_hook_ctx = NULL;
    test_hook_fired = 0;

    /* Put a NOP at BASE so normal execution proceeds */
    put_insn3(&cpu, BASE, rom_nop_insn());
    xtensa_step(&cpu);
    ASSERT_FALSE(test_hook_fired);
    ASSERT_EQ(cpu.pc, BASE + 3);
    teardown(&cpu);
}

TEST(test_loaded_rom_executes_unregistered_entry) {
    xtensa_cpu_t cpu;
    setup(&cpu);
    esp32_rom_stubs_t *rom = rom_stubs_create(&cpu);
    const uint32_t rom_pc = 0x40010010u;

    /* Without an external ROM image, an unknown mask-ROM call retains the
     * compatibility fallback: return zero to its caller and count it. */
    cpu.pc = rom_pc;
    cpu._pc_written = true;
    ar_write(&cpu, 0, BASE);
    XT_PS_SET_CALLINC(cpu.ps, 0);
    xtensa_step(&cpu);
    ASSERT_EQ(cpu.pc, BASE);
    ASSERT_EQ(rom_stubs_unregistered_count(rom), 1);

    /* Once a validated ROM ELF has been loaded, the same unknown entry runs
     * its real instructions. A NOP stands in for that loaded code here. */
    put_insn3(&cpu, rom_pc, rom_nop_insn());
    rom_stubs_set_real_rom(rom, true);
    cpu.pc = rom_pc;
    cpu._pc_written = true;
    xtensa_step(&cpu);
    ASSERT_EQ(cpu.pc, rom_pc + 3u);
    ASSERT_EQ(rom_stubs_unregistered_count(rom), 1);

    rom_stubs_destroy(rom);
    teardown(&cpu);
}

TEST(test_classic_rom_abi_is_not_installed_on_s3) {
    const flexe_target_desc_t *s3 =
        flexe_target_by_id(FLEXE_TARGET_ESP32S3);
    xtensa_mem_t *mem = mem_create_for_target(s3);
    xtensa_cpu_t cpu;
    ASSERT_TRUE(mem != NULL);
    if (!mem) return;

    xtensa_cpu_reset_for_target(&cpu, s3);
    cpu.mem = mem;
    uint64_t unmapped = mem_unmapped_count(mem);
    esp32_rom_stubs_t *rom = rom_stubs_create(&cpu);

    ASSERT_TRUE(rom != NULL);
    ASSERT_EQ(rom_stubs_stub_count(rom), 0);
    ASSERT_TRUE(cpu.pc_hook == NULL);
    ASSERT_TRUE(cpu.pc_hook_bitmap == NULL);
    ASSERT_EQ(mem_unmapped_count(mem), unmapped);

    rom_stubs_destroy(rom);
    mem_destroy(mem);
}

/* ===== Test: rom_stub_dispatch ===== */

static int dispatch_called;

static void test_dispatch_stub(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    dispatch_called = 1;
    /* Simulate return to caller: set PC to some known value */
    cpu->pc = BASE;
}

TEST(test_rom_stub_dispatch) {
    xtensa_cpu_t cpu;
    setup(&cpu);

    esp32_rom_stubs_t *rom = rom_stubs_create(&cpu);
    dispatch_called = 0;

    /* Register a custom stub at a ROM address */
    rom_stubs_register(rom, 0x40050000, test_dispatch_stub, "test_stub");

    cpu.pc = 0x40050000;
    xtensa_step(&cpu);
    ASSERT_TRUE(dispatch_called);
    ASSERT_EQ(cpu.pc, BASE);

    rom_stubs_destroy(rom);
    teardown(&cpu);
}

/* ===== Test: conditional hook can observe or consume ===== */

typedef struct {
    int calls;
    uint32_t handled_pc;
} conditional_hook_test_t;

static int test_conditional_stub(xtensa_cpu_t *cpu, void *ctx)
{
    conditional_hook_test_t *test = ctx;
    test->calls++;
    if (ar_read(cpu, 2) == 0)
        return 0;
    cpu->pc = test->handled_pc;
    return 1;
}

TEST(test_rom_conditional_stub) {
    xtensa_cpu_t cpu;
    setup(&cpu);
    esp32_rom_stubs_t *rom = rom_stubs_create(&cpu);
    conditional_hook_test_t test = {0, BASE + 0x300u};
    uint32_t hook_addr = BASE + 0x200u;
    put_insn3(&cpu, hook_addr, rom_nop_insn());
    ASSERT_EQ(rom_stubs_register_conditional_ctx(
                      rom, hook_addr, test_conditional_stub,
                      "conditional", &test), 0);

    cpu.pc = hook_addr;
    cpu._pc_written = true;
    ar_write(&cpu, 2, 0);
    xtensa_step(&cpu);
    ASSERT_EQ(cpu.pc, hook_addr + 3u);
    ASSERT_EQ(test.calls, 1);

    cpu.pc = hook_addr;
    cpu._pc_written = true;
    ar_write(&cpu, 2, 1);
    xtensa_step(&cpu);
    ASSERT_EQ(cpu.pc, test.handled_pc);
    ASSERT_EQ(test.calls, 2);

    rom_stubs_destroy(rom);
    teardown(&cpu);
}

TEST(test_rom_conditional_fallback_preserves_existing_hook) {
    xtensa_cpu_t cpu;
    setup(&cpu);
    esp32_rom_stubs_t *rom = rom_stubs_create(&cpu);
    conditional_hook_test_t fallback = {0, BASE + 0x480u};
    uint32_t hook_addr = BASE + 0x400u;
    put_insn3(&cpu, hook_addr, 0x004136u);

    dispatch_called = 0;
    ASSERT_EQ(rom_stubs_register_exact_ctx(
                  rom, hook_addr, test_dispatch_stub,
                  "authoritative", NULL), 0u);
    ASSERT_EQ(rom_stubs_register_conditional_exact_if_absent_ctx(
                  rom, hook_addr, test_conditional_stub,
                  "discovered-fallback", &fallback), 1u);

    cpu.pc = hook_addr;
    cpu._pc_written = true;
    ar_write(&cpu, 2, 1u);
    ASSERT_EQ(xtensa_step(&cpu), 0u);
    ASSERT_TRUE(dispatch_called);
    ASSERT_EQ(fallback.calls, 0u);
    ASSERT_EQ(cpu.pc, BASE);

    rom_stubs_destroy(rom);
    teardown(&cpu);
}

TEST(test_rom_registration_backscans_only_post_entry_symbols) {
    xtensa_cpu_t cpu;
    setup(&cpu);
    esp32_rom_stubs_t *rom = rom_stubs_create(&cpu);
    conditional_hook_test_t test = {0, BASE + 0x700u};

    /* An exact ENTRY must not cause a coincidental preceding ENTRY encoding
     * to inherit its hook. */
    uint32_t exact_entry = BASE + 0x500u;
    put_insn3(&cpu, exact_entry, 0x004136u);
    put_insn3(&cpu, exact_entry - 8u, 0x004136u);
    ASSERT_EQ(rom_stubs_register_conditional_ctx(
                      rom, exact_entry, test_conditional_stub,
                      "exact-entry", &test), 0u);
    cpu.pc = exact_entry - 8u;
    cpu._pc_written = true;
    cpu.windowbase = 0u;
    cpu.windowstart = 1u;
    XT_PS_SET_CALLINC(cpu.ps, 0u);
    ar_write(&cpu, 1, 0x3FFB7000u);
    ASSERT_EQ(xtensa_step(&cpu), 0u);
    ASSERT_EQ(cpu.pc, exact_entry - 5u);
    ASSERT_EQ(test.calls, 0u);

    /* Preserve the narrowly scoped legacy case: a symbol immediately after
     * ENTRY receives an alias at the actual call target. */
    uint32_t aliased_entry = BASE + 0x600u;
    put_insn3(&cpu, aliased_entry, 0x004136u);
    put_insn3(&cpu, aliased_entry + 3u, rom_nop_insn());
    ASSERT_EQ(rom_stubs_register_conditional_ctx(
                      rom, aliased_entry + 3u, test_conditional_stub,
                      "post-entry-symbol", &test), 0u);
    cpu.pc = aliased_entry;
    cpu._pc_written = true;
    ar_write(&cpu, 2, 1u);
    ASSERT_EQ(xtensa_step(&cpu), 0u);
    ASSERT_EQ(cpu.pc, test.handled_pc);
    ASSERT_EQ(test.calls, 1u);

    rom_stubs_destroy(rom);
    teardown(&cpu);
}

/* ===== Test: rom_arg with CALL4 ===== */

TEST(test_rom_arg_call4) {
    xtensa_cpu_t cpu;
    setup(&cpu);

    /* Enable windowed calls */
    cpu.ps = 0x00040000;  /* WOE=1 */
    cpu.windowbase = 0;
    cpu.windowstart = 1;

    /* Put args in caller's registers:
     * After CALL4 (callinc=1), args are at ar[6], ar[7], ...
     * which = callinc*4+2 = 6 */
    ar_write(&cpu, 6, 0xAAAA0001);
    ar_write(&cpu, 7, 0xAAAA0002);
    ar_write(&cpu, 8, 0xAAAA0003);

    /* Build CALL4 at BASE targeting a ROM address 0x40007cf8
     * After CALL4, PC = target, CALLINC=1, a4 = retaddr */
    /* We'll manually set up state as if CALL4 just executed */
    cpu.pc = 0x40007cf8;
    XT_PS_SET_CALLINC(cpu.ps, 1);
    /* a4 = return address with callinc bits */
    ar_write(&cpu, 4, (1u << 30) | (BASE & 0x3FFFFFFF));

    esp32_rom_stubs_t *rom = rom_stubs_create(&cpu);

    /* ets_write_char_uart at 0x40007cf8 reads arg0 = ar[6] */
    xtensa_step(&cpu);

    /* Should have written char 0x01 (low byte of 0xAAAA0001) */
    ASSERT_EQ(rom_stubs_output_count(rom), 1);
    ASSERT_EQ((uint8_t)rom_stubs_output_buf(rom)[0], 0x01);

    /* Should have returned: PC = caller, CALLINC = 0 */
    ASSERT_EQ(XT_PS_CALLINC(cpu.ps), 0);

    rom_stubs_destroy(rom);
    teardown(&cpu);
}

/* ===== Test: rom_arg with CALL0 ===== */

TEST(test_rom_arg_call0) {
    xtensa_cpu_t cpu;
    setup(&cpu);

    /* CALL0: callinc=0, args in a2, a3, ... */
    cpu.pc = 0x40007cf8;
    XT_PS_SET_CALLINC(cpu.ps, 0);
    ar_write(&cpu, 0, BASE);      /* return address */
    ar_write(&cpu, 2, (uint32_t)'Z');  /* arg0 */

    esp32_rom_stubs_t *rom = rom_stubs_create(&cpu);

    xtensa_step(&cpu);

    ASSERT_EQ(rom_stubs_output_count(rom), 1);
    ASSERT_EQ(rom_stubs_output_buf(rom)[0], 'Z');
    ASSERT_EQ(cpu.pc, BASE);

    rom_stubs_destroy(rom);
    teardown(&cpu);
}

/* ===== Test: stub_write_char ===== */

TEST(test_stub_write_char) {
    xtensa_cpu_t cpu;
    setup(&cpu);

    esp32_rom_stubs_t *rom = rom_stubs_create(&cpu);

    /* Call ets_write_char_uart via CALL0 convention */
    cpu.pc = 0x40007cf8;
    XT_PS_SET_CALLINC(cpu.ps, 0);
    ar_write(&cpu, 0, BASE);
    ar_write(&cpu, 2, 'H');
    xtensa_step(&cpu);

    cpu.pc = 0x40007cf8;
    ar_write(&cpu, 0, BASE);
    ar_write(&cpu, 2, 'i');
    xtensa_step(&cpu);

    ASSERT_EQ(rom_stubs_output_count(rom), 2);
    ASSERT_TRUE(memcmp(rom_stubs_output_buf(rom), "Hi", 2) == 0);

    rom_stubs_output_clear(rom);
    ASSERT_EQ(rom_stubs_output_count(rom), 0);

    rom_stubs_destroy(rom);
    teardown(&cpu);
}

/* ===== Test: stub_printf_basic ===== */

TEST(test_stub_printf_basic) {
    xtensa_cpu_t cpu;
    setup(&cpu);

    esp32_rom_stubs_t *rom = rom_stubs_create(&cpu);

    /* Write format string "Hello %d" into memory at 0x3FFB0000 */
    const char *fmt = "Hello %d";
    uint32_t fmt_addr = 0x3FFB0000;
    for (int i = 0; fmt[i]; i++)
        mem_write8(cpu.mem, fmt_addr + (uint32_t)i, (uint8_t)fmt[i]);
    mem_write8(cpu.mem, fmt_addr + (uint32_t)strlen(fmt), 0);

    /* CALL0 to ets_printf */
    cpu.pc = 0x40007d54;
    XT_PS_SET_CALLINC(cpu.ps, 0);
    ar_write(&cpu, 0, BASE);
    ar_write(&cpu, 2, fmt_addr);  /* arg0: fmt */
    ar_write(&cpu, 3, 42);        /* arg1: 42 */

    xtensa_step(&cpu);

    ASSERT_EQ(rom_stubs_output_count(rom), 8);
    ASSERT_TRUE(memcmp(rom_stubs_output_buf(rom), "Hello 42", 8) == 0);
    /* Return value = bytes written */
    ASSERT_EQ(ar_read(&cpu, 2), 8);

    rom_stubs_destroy(rom);
    teardown(&cpu);
}

/* ===== Test: stub_printf_hex ===== */

TEST(test_stub_printf_hex) {
    xtensa_cpu_t cpu;
    setup(&cpu);

    esp32_rom_stubs_t *rom = rom_stubs_create(&cpu);

    const char *fmt = "%08x";
    uint32_t fmt_addr = 0x3FFB0000;
    for (int i = 0; fmt[i]; i++)
        mem_write8(cpu.mem, fmt_addr + (uint32_t)i, (uint8_t)fmt[i]);
    mem_write8(cpu.mem, fmt_addr + (uint32_t)strlen(fmt), 0);

    cpu.pc = 0x40007d54;
    XT_PS_SET_CALLINC(cpu.ps, 0);
    ar_write(&cpu, 0, BASE);
    ar_write(&cpu, 2, fmt_addr);
    ar_write(&cpu, 3, 0xDEAD);

    xtensa_step(&cpu);

    ASSERT_EQ(rom_stubs_output_count(rom), 8);
    ASSERT_TRUE(memcmp(rom_stubs_output_buf(rom), "0000dead", 8) == 0);

    rom_stubs_destroy(rom);
    teardown(&cpu);
}

/* ===== Test: stub_printf_string ===== */

TEST(test_stub_printf_string) {
    xtensa_cpu_t cpu;
    setup(&cpu);

    esp32_rom_stubs_t *rom = rom_stubs_create(&cpu);

    /* Format string */
    const char *fmt = "name=%s";
    uint32_t fmt_addr = 0x3FFB0000;
    for (int i = 0; fmt[i]; i++)
        mem_write8(cpu.mem, fmt_addr + (uint32_t)i, (uint8_t)fmt[i]);
    mem_write8(cpu.mem, fmt_addr + (uint32_t)strlen(fmt), 0);

    /* String argument */
    const char *str = "ESP32";
    uint32_t str_addr = 0x3FFB0100;
    for (int i = 0; str[i]; i++)
        mem_write8(cpu.mem, str_addr + (uint32_t)i, (uint8_t)str[i]);
    mem_write8(cpu.mem, str_addr + (uint32_t)strlen(str), 0);

    cpu.pc = 0x40007d54;
    XT_PS_SET_CALLINC(cpu.ps, 0);
    ar_write(&cpu, 0, BASE);
    ar_write(&cpu, 2, fmt_addr);
    ar_write(&cpu, 3, str_addr);

    xtensa_step(&cpu);

    ASSERT_EQ(rom_stubs_output_count(rom), 10);
    ASSERT_TRUE(memcmp(rom_stubs_output_buf(rom), "name=ESP32", 10) == 0);

    rom_stubs_destroy(rom);
    teardown(&cpu);
}

/* ===== Test: stub_delay_us ===== */

TEST(test_stub_delay_us) {
    xtensa_cpu_t cpu;
    setup(&cpu);

    esp32_rom_stubs_t *rom = rom_stubs_create(&cpu);

    uint64_t vtime_before = cpu.virtual_time_us;

    /* CALL0 to ets_delay_us(100) */
    cpu.pc = 0x40008534;
    XT_PS_SET_CALLINC(cpu.ps, 0);
    ar_write(&cpu, 0, BASE);
    ar_write(&cpu, 2, 100);  /* 100 microseconds */

    xtensa_step(&cpu);

    /* virtual_time_us should advance by 100 us */
    ASSERT_EQ(cpu.virtual_time_us - vtime_before, 100);
    ASSERT_EQ(cpu.pc, BASE);

    rom_stubs_destroy(rom);
    teardown(&cpu);
}

/* ===== Test: stub_cache_noop ===== */

TEST(test_stub_cache_noop) {
    xtensa_cpu_t cpu;
    setup(&cpu);

    esp32_rom_stubs_t *rom = rom_stubs_create(&cpu);

    /* Call Cache_Read_Enable — should return without crash */
    cpu.pc = 0x40009a84;
    XT_PS_SET_CALLINC(cpu.ps, 0);
    ar_write(&cpu, 0, BASE);

    xtensa_step(&cpu);
    ASSERT_EQ(cpu.pc, BASE);

    rom_stubs_destroy(rom);
    teardown(&cpu);
}

static uint32_t call_cache_flash_mmu_set0(xtensa_cpu_t *cpu,
                                          uint32_t core, uint32_t pid,
                                          uint32_t vaddr, uint32_t paddr,
                                          uint32_t psize, uint32_t num) {
    cpu->pc = 0x400095E0u;
    XT_PS_SET_CALLINC(cpu->ps, 0);
    ar_write(cpu, 0, BASE);
    ar_write(cpu, 2, core);
    ar_write(cpu, 3, pid);
    ar_write(cpu, 4, vaddr);
    ar_write(cpu, 5, paddr);
    ar_write(cpu, 6, psize);
    ar_write(cpu, 7, num);
    xtensa_step(cpu);
    return ar_read(cpu, 2);
}

static uint32_t call_cache_sram_mmu_set0(xtensa_cpu_t *cpu,
                                         uint32_t core, uint32_t pid,
                                         uint32_t vaddr, uint32_t paddr,
                                         uint32_t psize, uint32_t num) {
    cpu->pc = 0x400097F4u;
    XT_PS_SET_CALLINC(cpu->ps, 0);
    ar_write(cpu, 0, BASE);
    ar_write(cpu, 2, core);
    ar_write(cpu, 3, pid);
    ar_write(cpu, 4, vaddr);
    ar_write(cpu, 5, paddr);
    ar_write(cpu, 6, psize);
    ar_write(cpu, 7, num);
    xtensa_step(cpu);
    return ar_read(cpu, 2);
}

TEST(test_cache_flash_mmu_rom_api_uses_byte_addresses) {
    xtensa_cpu_t cpu;
    setup(&cpu);
    esp32_periph_t *periph = periph_create(cpu.mem);
    periph_attach_cpus(periph, &cpu, NULL);
    esp32_rom_stubs_t *rom = rom_stubs_create(&cpu);
    rom_stubs_set_periph(rom, periph);

    cpu.mem->flash_data[0x20000u] = 0x21;
    cpu.mem->flash_data[0x2F234u] = 0x43;
    ASSERT_EQ(call_cache_flash_mmu_set0(&cpu, 0, 0, 0x3F500000u,
                                        0x20000u, 64u, 1u), 0u);
    ASSERT_EQ(mem_read32(cpu.mem, 0x3FF10000u + 16u * 4u), 2u);
    ASSERT_EQ(mem_read8(cpu.mem, 0x3F500000u), 0x21);
    ASSERT_EQ(mem_read8(cpu.mem, 0x3F50F234u), 0x43);

    cpu.mem->flash_insn[0x30000u] = 0x65;
    cpu.mem->flash_insn[0x3FFFFu] = 0x87;
    ASSERT_EQ(call_cache_flash_mmu_set0(&cpu, 0, 0, 0x40400000u,
                                        0x30000u, 64u, 1u), 0u);
    ASSERT_EQ(mem_read32(cpu.mem, 0x3FF10000u + 128u * 4u), 3u);
    ASSERT_EQ(mem_read8(cpu.mem, 0x40400000u), 0x65);
    ASSERT_EQ(mem_read8(cpu.mem, 0x4040FFFFu), 0x87);

    ASSERT_EQ(call_cache_flash_mmu_set0(&cpu, 0, 0, 0x3F500001u,
                                        0x20000u, 64u, 1u), 1u);
    ASSERT_EQ(call_cache_flash_mmu_set0(&cpu, 0, 0, 0x3F500000u,
                                        0x20000u, 32u, 1u), 3u);
    ASSERT_EQ(call_cache_flash_mmu_set0(&cpu, 0, 0, 0x3F7F0000u,
                                        0x20000u, 64u, 2u), 4u);
    ASSERT_EQ(call_cache_flash_mmu_set0(&cpu, 0, 0, 0x400C0000u,
                                        0x20000u, 64u, 1u), 5u);
    ASSERT_EQ(call_cache_flash_mmu_set0(&cpu, 0, 0, 0x3F500000u,
                                        0xFF0000u, 64u, 2u), 4u);

    /* mmu_init is modeled as a functional unmap because Flexe does not
     * otherwise expose the real cache-disable bus mask around this ROM call. */
    cpu.pc = 0x400095A4u;
    XT_PS_SET_CALLINC(cpu.ps, 0);
    ar_write(&cpu, 0, BASE);
    ar_write(&cpu, 2, 0u);
    xtensa_step(&cpu);
    ASSERT_EQ(mem_read32(cpu.mem, 0x3FF10000u + 16u * 4u), 0x100u);
    ASSERT_TRUE(mem_get_ptr(cpu.mem, 0x3F500000u) == NULL);
    ASSERT_TRUE(mem_get_ptr(cpu.mem, 0x40400000u) == NULL);

    rom_stubs_destroy(rom);
    periph_destroy(periph);
    teardown(&cpu);
}

TEST(test_cache_sram_mmu_rom_api_maps_target_psram) {
    xtensa_cpu_t cpu;
    setup(&cpu);
    esp32_rom_stubs_t *rom = rom_stubs_create(&cpu);

    cpu.mem->psram[0x20000u] = 0xA5u;
    cpu.mem->psram[0x27FFFu] = 0x5Au;
    ASSERT_EQ(call_cache_sram_mmu_set0(&cpu, 0, 0, 0x3F800000u,
                                       0x20000u, 32u, 1u), 0u);
    ASSERT_TRUE(mem_get_ptr(cpu.mem, 0x3F800000u) ==
                cpu.mem->psram + 0x20000u);
    ASSERT_EQ(mem_read8(cpu.mem, 0x3F800000u), 0xA5u);
    ASSERT_EQ(mem_read8(cpu.mem, 0x3F807FFFu), 0x5Au);

    ASSERT_EQ(call_cache_sram_mmu_set0(&cpu, 0, 0, 0x3F800001u,
                                       0x20000u, 32u, 1u), 1u);
    ASSERT_EQ(call_cache_sram_mmu_set0(&cpu, 0, 8, 0x3F800000u,
                                       0x20000u, 32u, 1u), 2u);
    ASSERT_EQ(call_cache_sram_mmu_set0(&cpu, 0, 0, 0x3F800000u,
                                       0x20000u, 2u, 1u), 3u);
    ASSERT_EQ(call_cache_sram_mmu_set0(&cpu, 0, 0, 0x3F800000u,
                                       0x400000u, 32u, 1u), 4u);
    ASSERT_EQ(call_cache_sram_mmu_set0(&cpu, 0, 0, 0x3F700000u,
                                       0x20000u, 32u, 1u), 5u);
    ASSERT_EQ(rom_stubs_unregistered_count(rom), 0u);

    rom_stubs_destroy(rom);
    teardown(&cpu);
}

/* ===== Test: stub_memcpy ===== */

TEST(test_stub_memcpy) {
    xtensa_cpu_t cpu;
    setup(&cpu);

    esp32_rom_stubs_t *rom = rom_stubs_create(&cpu);

    /* Write source data */
    uint32_t src = 0x3FFB0000;
    uint32_t dst = 0x3FFB1000;
    mem_write8(cpu.mem, src + 0, 0xDE);
    mem_write8(cpu.mem, src + 1, 0xAD);
    mem_write8(cpu.mem, src + 2, 0xBE);
    mem_write8(cpu.mem, src + 3, 0xEF);

    /* CALL0 to memcpy(dst, src, 4) */
    cpu.pc = 0x4000c2c8;
    XT_PS_SET_CALLINC(cpu.ps, 0);
    ar_write(&cpu, 0, BASE);
    ar_write(&cpu, 2, dst);  /* arg0: dst */
    ar_write(&cpu, 3, src);  /* arg1: src */
    ar_write(&cpu, 4, 4);    /* arg2: len */

    xtensa_step(&cpu);

    ASSERT_EQ(mem_read8(cpu.mem, dst + 0), 0xDE);
    ASSERT_EQ(mem_read8(cpu.mem, dst + 1), 0xAD);
    ASSERT_EQ(mem_read8(cpu.mem, dst + 2), 0xBE);
    ASSERT_EQ(mem_read8(cpu.mem, dst + 3), 0xEF);
    /* Return value = dst */
    ASSERT_EQ(ar_read(&cpu, 2), dst);

    rom_stubs_destroy(rom);
    teardown(&cpu);
}

static uint32_t call_builtin_rom0(xtensa_cpu_t *cpu, uint32_t addr,
                                  uint32_t arg0) {
    cpu->pc = addr;
    XT_PS_SET_CALLINC(cpu->ps, 0);
    ar_write(cpu, 0, BASE);
    ar_write(cpu, 2, arg0);
    xtensa_step(cpu);
    return ar_read(cpu, 2);
}

static uint32_t call_builtin_rom_args(xtensa_cpu_t *cpu, uint32_t addr,
                                      const uint32_t *args, size_t count) {
    cpu->pc = addr;
    cpu->_pc_written = true;
    XT_PS_SET_CALLINC(cpu->ps, 0);
    ar_write(cpu, 0, BASE);
    for (size_t i = 0; i < count; i++)
        ar_write(cpu, 2 + (int)i, args[i]);
    xtensa_step(cpu);
    return ar_read(cpu, 2);
}

TEST(test_rom_string_spans_and_bounded_concat) {
    xtensa_cpu_t cpu;
    setup(&cpu);
    esp32_rom_stubs_t *rom = rom_stubs_create(&cpu);
    const uint32_t string = 0x3FFB1000u;
    const uint32_t set = 0x3FFB1100u;
    const uint32_t dst = 0x3FFB1200u;
    const uint32_t src = 0x3FFB1300u;

    static const uint8_t span_string[] = "aaab42";
    static const uint8_t accept[] = "ab";
    mem_load(cpu.mem, string, span_string, sizeof(span_string));
    mem_load(cpu.mem, set, accept, sizeof(accept));
    uint32_t span_args[] = { string, set };
    ASSERT_EQ(call_builtin_rom_args(&cpu, 0x4000C648u, span_args, 2), 4u);

    static const uint8_t reject_string[] = "alpha,beta";
    static const uint8_t reject[] = ",;";
    mem_load(cpu.mem, string, reject_string, sizeof(reject_string));
    mem_load(cpu.mem, set, reject, sizeof(reject));
    ASSERT_EQ(call_builtin_rom_args(&cpu, 0x4000C558u, span_args, 2), 5u);

    static const uint8_t flex[] = "flex";
    static const uint8_t suffix[] = "e-s3";
    mem_load(cpu.mem, dst, flex, sizeof(flex));
    mem_load(cpu.mem, src, suffix, sizeof(suffix));
    uint32_t concat_args[] = { dst, src, 12u };
    ASSERT_EQ(call_builtin_rom_args(&cpu, 0x40001470u, concat_args, 3), 8u);
    ASSERT_TRUE(strcmp((const char *)mem_get_ptr(cpu.mem, dst),
                       "flexe-s3") == 0);

    static const uint8_t short_dst[] = "abc";
    static const uint8_t long_src[] = "defghi";
    mem_load(cpu.mem, dst, short_dst, sizeof(short_dst));
    mem_load(cpu.mem, src, long_src, sizeof(long_src));
    concat_args[2] = 6u;
    ASSERT_EQ(call_builtin_rom_args(&cpu, 0x40001470u, concat_args, 3), 9u);
    ASSERT_TRUE(strcmp((const char *)mem_get_ptr(cpu.mem, dst), "abcde") == 0);

    mem_load(cpu.mem, dst, short_dst, sizeof(short_dst));
    concat_args[2] = 2u;
    ASSERT_EQ(call_builtin_rom_args(&cpu, 0x40001470u, concat_args, 3), 8u);
    ASSERT_TRUE(strcmp((const char *)mem_get_ptr(cpu.mem, dst), "abc") == 0);
    ASSERT_EQ(rom_stubs_unregistered_count(rom), 0);

    rom_stubs_destroy(rom);
    teardown(&cpu);
}

TEST(test_cpu_frequency_rom_pair) {
    xtensa_cpu_t cpu;
    setup(&cpu);
    esp32_rom_stubs_t *rom = rom_stubs_create(&cpu);

    ASSERT_EQ(call_builtin_rom0(&cpu, 0x4000855Cu, 0), 160u);
    (void)call_builtin_rom0(&cpu, 0x40008550u, 240u);
    ASSERT_EQ(mem_read32(cpu.mem, 0x3FFE01E0u), 240u);
    ASSERT_EQ(call_builtin_rom0(&cpu, 0x4000855Cu, 0), 240u);
    ASSERT_EQ(rom_stubs_unregistered_count(rom), 0);

    rom_stubs_destroy(rom);
    teardown(&cpu);
}

static uint32_t rom_syscall_test_reent;
static uint32_t rom_syscall_test_args[4];
static uint32_t rom_syscall_test_alloc_next;
static uint32_t rom_syscall_test_alloc_args[2];
static uint32_t rom_syscall_test_lock_arg;
static uint32_t rom_syscall_test_write_args[4];
static uint32_t rom_syscall_test_close_args[2];
static uint32_t rom_syscall_test_freed[4];
static int rom_syscall_test_alloc_calls;
static int rom_syscall_test_lock_calls;
static int rom_syscall_test_write_calls;
static int rom_syscall_test_close_calls;
static int rom_syscall_test_free_calls;
static int rom_syscall_test_close_result;
static int rom_syscall_test_io_calls;
static uint32_t rom_syscall_test_io_result;

static void return_from_test_call8(xtensa_cpu_t *cpu, uint32_t value) {
    ar_write(cpu, 10, value);
    cpu->pc = 0x40000000u | (ar_read(cpu, 8) & 0x3FFFFFFFu);
    XT_PS_SET_CALLINC(cpu->ps, 0);
}

static void test_rom_getreent_handler(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    return_from_test_call8(cpu, rom_syscall_test_reent);
}

static void test_rom_open_r_handler(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    for (int i = 0; i < 4; i++)
        rom_syscall_test_args[i] = ar_read(cpu, 10 + i);
    return_from_test_call8(cpu, 37u);
}

static void test_rom_io_handler(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    for (int i = 0; i < 4; i++)
        rom_syscall_test_args[i] = ar_read(cpu, 10 + i);
    rom_syscall_test_io_calls++;
    return_from_test_call8(cpu, rom_syscall_test_io_result);
}

static void test_rom_malloc_r_handler(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    rom_syscall_test_alloc_args[0] = ar_read(cpu, 10);
    rom_syscall_test_alloc_args[1] = ar_read(cpu, 11);
    uint32_t result = rom_syscall_test_alloc_next;
    rom_syscall_test_alloc_next += 0x100u;
    rom_syscall_test_alloc_calls++;
    return_from_test_call8(cpu, result);
}

static void test_rom_lock_init_recursive_handler(xtensa_cpu_t *cpu,
                                                 void *ctx) {
    (void)ctx;
    rom_syscall_test_lock_arg = ar_read(cpu, 10);
    rom_syscall_test_lock_calls++;
    mem_write32(cpu->mem, rom_syscall_test_lock_arg,
                0xF17E0000u + (uint32_t)rom_syscall_test_lock_calls);
    return_from_test_call8(cpu, 0);
}

static void test_rom_write_handler(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    for (int i = 0; i < 4; i++)
        rom_syscall_test_write_args[i] = ar_read(cpu, 10 + i);
    rom_syscall_test_write_calls++;
    return_from_test_call8(cpu, rom_syscall_test_write_args[3]);
}

static void test_rom_close_handler(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    rom_syscall_test_close_args[0] = ar_read(cpu, 10);
    rom_syscall_test_close_args[1] = ar_read(cpu, 11);
    rom_syscall_test_close_calls++;
    return_from_test_call8(cpu, (uint32_t)rom_syscall_test_close_result);
}

static void test_rom_free_handler(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    if (rom_syscall_test_free_calls < 4)
        rom_syscall_test_freed[rom_syscall_test_free_calls] = ar_read(cpu, 11);
    rom_syscall_test_free_calls++;
    return_from_test_call8(cpu, 0);
}

static void test_rom_lock_close_recursive_handler(xtensa_cpu_t *cpu,
                                                  void *ctx) {
    (void)ctx;
    rom_syscall_test_lock_arg = ar_read(cpu, 10);
    rom_syscall_test_lock_calls++;
    return_from_test_call8(cpu, 0);
}

TEST(test_rom_newlib_scalar_helpers) {
    xtensa_cpu_t cpu;
    setup(&cpu);
    esp32_rom_stubs_t *rom = rom_stubs_create(&cpu);
    const uint32_t buf = 0x3FFB1000u;
    const uint32_t reent = 0x3FFB2000u;
    const uint32_t offset = 0x3FFB2100u;

    uint32_t utoa_args[] = { 0xDEADBEEFu, buf, 16u };
    ASSERT_EQ(call_builtin_rom_args(&cpu, 0x40056258u, utoa_args, 3), buf);
    ASSERT_TRUE(strcmp((const char *)mem_get_ptr(cpu.mem, buf),
                       "deadbeef") == 0);

    uint32_t itoa_args[] = { 0x80000000u, buf, 10u };
    ASSERT_EQ(call_builtin_rom_args(&cpu, 0x400566B4u, itoa_args, 3), buf);
    ASSERT_TRUE(strcmp((const char *)mem_get_ptr(cpu.mem, buf),
                       "-2147483648") == 0);

    uint32_t div_args[] = { (uint32_t)-17, 5u };
    ASSERT_EQ(call_builtin_rom_args(&cpu, 0x40056348u, div_args, 2),
              (uint32_t)-3);
    ASSERT_EQ(ar_read(&cpu, 3), (uint32_t)-2);

    mem_write32(cpu.mem, offset, 0xA5A5A5A5u);
    uint32_t findenv_args[] = { reent, buf, offset };
    ASSERT_EQ(call_builtin_rom_args(&cpu, 0x40001F44u, findenv_args, 3), 0u);
    ASSERT_EQ(mem_read32(cpu.mem, offset), 0xA5A5A5A5u);

    uint32_t wctomb_args[] = { reent, buf, 0xE9u };
    ASSERT_EQ(call_builtin_rom_args(&cpu, 0x40058EF0u, wctomb_args, 3), 1u);
    ASSERT_EQ(mem_read8(cpu.mem, buf), 0xE9u);
    wctomb_args[2] = 0x100u;
    ASSERT_EQ(call_builtin_rom_args(&cpu, 0x40058EF0u, wctomb_args, 3),
              (uint32_t)-1);
    ASSERT_EQ(mem_read32(cpu.mem, reent), 138u);
    wctomb_args[1] = 0;
    ASSERT_EQ(call_builtin_rom_args(&cpu, 0x40058EF0u, wctomb_args, 3), 0u);

    const uint32_t first = 0x3FFB1100u;
    const uint32_t second = 0x3FFB1200u;
    mem_load(cpu.mem, first, (const uint8_t *)"alpha-beta", 11u);
    mem_load(cpu.mem, second, (const uint8_t *)"alpha-zeta", 11u);
    uint32_t memchr_args[] = { first, '-', 10u };
    ASSERT_EQ(call_builtin_rom_args(&cpu, 0x4000C244u, memchr_args, 3),
              first + 5u);
    uint32_t strchr_args[] = { first, 'b' };
    ASSERT_EQ(call_builtin_rom_args(&cpu, 0x4000C53Cu, strchr_args, 2),
              first + 6u);
    strchr_args[1] = 0;
    ASSERT_EQ(call_builtin_rom_args(&cpu, 0x4000C53Cu, strchr_args, 2),
              first + 10u);

    const uint32_t repeated = 0x3FFB2FF8u;
    mem_load(cpu.mem, repeated, (const uint8_t *)"one/two/three", 14u);
    uint32_t strrchr_args[] = { repeated, '/' };
    ASSERT_EQ(call_builtin_rom_args(&cpu, 0x40001708u, strrchr_args, 2),
              repeated + 7u);
    strrchr_args[1] = 0;
    ASSERT_EQ(call_builtin_rom_args(&cpu, 0x40001708u, strrchr_args, 2),
              repeated + 13u);
    strrchr_args[1] = 'z';
    ASSERT_EQ(call_builtin_rom_args(&cpu, 0x40001708u, strrchr_args, 2), 0u);

    uint32_t strncmp_args[] = { first, second, 6u };
    ASSERT_EQ(call_builtin_rom_args(&cpu, 0x4000C5F4u, strncmp_args, 3), 0u);
    strncmp_args[2] = 10u;
    ASSERT_TRUE((int32_t)call_builtin_rom_args(
                    &cpu, 0x4000C5F4u, strncmp_args, 3) < 0);

    uint32_t getenv_args[] = { reent, first };
    ASSERT_EQ(call_builtin_rom_args(&cpu, 0x40001FBCu, getenv_args, 2), 0u);

    const uint32_t env_table = 0x3FFB1300u;
    const uint32_t env_entry = 0x3FFB1400u;
    const uint32_t env_name = 0x3FFB1500u;
    mem_load(cpu.mem, env_entry, (const uint8_t *)"MODE=production", 16u);
    mem_load(cpu.mem, env_name, (const uint8_t *)"MODE", 5u);
    mem_write32(cpu.mem, env_table, env_entry);
    mem_write32(cpu.mem, env_table + 4u, 0u);
    mem_write32(cpu.mem, 0x3FFAE0B4u, env_table);
    mem_write32(cpu.mem, offset, 0xA5A5A5A5u);
    uint32_t populated_findenv_args[] = { reent, env_name, offset };
    ASSERT_EQ(call_builtin_rom_args(&cpu, 0x40001F44u,
                                    populated_findenv_args, 3),
              env_entry + 5u);
    ASSERT_EQ(mem_read32(cpu.mem, offset), 0u);
    getenv_args[1] = env_name;
    ASSERT_EQ(call_builtin_rom_args(&cpu, 0x40001FBCu, getenv_args, 2),
              env_entry + 5u);
    uint32_t lock_args[] = { reent };
    (void)call_builtin_rom_args(&cpu, 0x40001E08u, lock_args, 1);
    (void)call_builtin_rom_args(&cpu, 0x40001E14u, lock_args, 1);
    ASSERT_EQ(rom_stubs_unregistered_count(rom), 0);

    rom_stubs_destroy(rom);
    teardown(&cpu);
}

TEST(test_rom_strdup_uses_guest_allocator) {
    xtensa_cpu_t cpu;
    setup(&cpu);
    esp32_rom_stubs_t *rom = rom_stubs_create(&cpu);
    const uint32_t table = 0x3FFB1000u;
    const uint32_t getreent = 0x400D0100u;
    const uint32_t malloc_r = 0x400D0300u;
    const uint32_t src = 0x3FFB2000u;
    const uint32_t dst = 0x3FFB3000u;
    rom_syscall_test_reent = 0x3FFB4000u;
    rom_syscall_test_alloc_next = dst;
    rom_syscall_test_alloc_calls = 0;

    mem_write32(cpu.mem, 0x3FFAE024u, table);
    mem_write32(cpu.mem, table + 0x00u, getreent);
    mem_write32(cpu.mem, table + 0x04u, malloc_r);
    mem_load(cpu.mem, src, (const uint8_t *)"bruce.cfg", 10u);
    rom_stubs_register(rom, getreent, test_rom_getreent_handler,
                       "test_getreent");
    rom_stubs_register(rom, malloc_r, test_rom_malloc_r_handler,
                       "test_malloc_r");

    uint32_t args[] = { src };
    ASSERT_EQ(call_builtin_rom_args(&cpu, 0x4000143Cu, args, 1), dst);
    ASSERT_TRUE(strcmp((const char *)mem_get_ptr(cpu.mem, dst),
                       "bruce.cfg") == 0);
    ASSERT_EQ(rom_syscall_test_alloc_calls, 1);
    ASSERT_EQ(rom_syscall_test_alloc_args[0], rom_syscall_test_reent);
    ASSERT_EQ(rom_syscall_test_alloc_args[1], 10u);
    ASSERT_EQ(rom_stubs_unregistered_count(rom), 0);

    rom_stubs_destroy(rom);
    teardown(&cpu);
}

TEST(test_rom_sfp_initializes_and_reuses_guest_files) {
    xtensa_cpu_t cpu;
    setup(&cpu);
    esp32_rom_stubs_t *rom = rom_stubs_create(&cpu);
    const uint32_t table = 0x3FFB1000u;
    const uint32_t malloc_r = 0x400D0300u;
    const uint32_t lock_init = 0x400D0400u;
    const uint32_t reent = 0x3FFB2000u;
    const uint32_t first = 0x3FFB3000u;
    rom_syscall_test_alloc_next = first;
    rom_syscall_test_alloc_calls = 0;
    rom_syscall_test_lock_calls = 0;
    rom_syscall_test_lock_arg = 0;

    mem_write32(cpu.mem, 0x3FFAE024u, table);
    mem_write32(cpu.mem, table + 0x04u, malloc_r);
    mem_write32(cpu.mem, table + 0x64u, lock_init);
    rom_stubs_register(rom, malloc_r, test_rom_malloc_r_handler,
                       "test_malloc_r");
    rom_stubs_register(rom, lock_init, test_rom_lock_init_recursive_handler,
                       "test_lock_init_recursive");

    uint32_t args[] = { reent };
    ASSERT_EQ(call_builtin_rom_args(&cpu, 0x40001E90u, args, 1), first);
    ASSERT_EQ(rom_syscall_test_alloc_args[0], reent);
    ASSERT_EQ(rom_syscall_test_alloc_args[1], 104u);
    ASSERT_EQ(mem_read16(cpu.mem, first + 12u), 1u);
    ASSERT_EQ(mem_read16(cpu.mem, first + 14u), 0xFFFFu);
    ASSERT_EQ(rom_syscall_test_lock_arg, first + 88u);
    ASSERT_EQ(mem_read32(cpu.mem, first + 88u), 0xF17E0001u);
    ASSERT_EQ(mem_read32(cpu.mem, first + 100u), 0u);

    /* An active FILE consumes a second slot. */
    ASSERT_EQ(call_builtin_rom_args(&cpu, 0x40001E90u, args, 1),
              first + 0x100u);
    ASSERT_EQ(rom_syscall_test_alloc_calls, 2);

    /* fclose marks _flags free; __sfp must reuse and reinitialize that slot. */
    mem_write16(cpu.mem, first + 12u, 0u);
    mem_write32(cpu.mem, first + 4u, 0xA5A5A5A5u);
    ASSERT_EQ(call_builtin_rom_args(&cpu, 0x40001E90u, args, 1), first);
    ASSERT_EQ(rom_syscall_test_alloc_calls, 2);
    ASSERT_EQ(rom_syscall_test_lock_calls, 3);
    ASSERT_EQ(mem_read32(cpu.mem, first + 4u), 0u);
    ASSERT_EQ(mem_read32(cpu.mem, first + 88u), 0xF17E0003u);
    ASSERT_EQ(rom_stubs_unregistered_count(rom), 0);

    rom_stubs_destroy(rom);
    teardown(&cpu);
}

TEST(test_rom_fclose_flushes_closes_and_releases_guest_file) {
    xtensa_cpu_t cpu;
    setup(&cpu);
    esp32_rom_stubs_t *rom = rom_stubs_create(&cpu);
    const uint32_t table = 0x3FFB1000u;
    const uint32_t getreent = 0x400D0100u;
    const uint32_t free_r = 0x400D0200u;
    const uint32_t write_fn = 0x400D0300u;
    const uint32_t close_r = 0x400D0400u;
    const uint32_t lock_close = 0x400D0500u;
    const uint32_t read_r = 0x400D0600u;
    const uint32_t lseek_r = 0x400D0700u;
    const uint32_t reent = 0x3FFB2000u;
    const uint32_t fp = 0x3FFB3000u;
    const uint32_t buffer = 0x3FFB4000u;
    const uint32_t ungetc_buffer = 0x3FFB4100u;
    const uint32_t line_buffer = 0x3FFB4200u;
    const uint32_t cookie = 37u;
    uint8_t empty_file[104] = {0};

    rom_syscall_test_reent = reent;
    memset(rom_syscall_test_write_args, 0,
           sizeof(rom_syscall_test_write_args));
    memset(rom_syscall_test_close_args, 0,
           sizeof(rom_syscall_test_close_args));
    memset(rom_syscall_test_freed, 0, sizeof(rom_syscall_test_freed));
    rom_syscall_test_write_calls = 0;
    rom_syscall_test_close_calls = 0;
    rom_syscall_test_free_calls = 0;
    rom_syscall_test_lock_calls = 0;
    rom_syscall_test_close_result = 0;
    rom_syscall_test_io_calls = 0;

    mem_write32(cpu.mem, 0x3FFAE024u, table);
    mem_write32(cpu.mem, table + 0x00u, getreent);
    mem_write32(cpu.mem, table + 0x08u, free_r);
    mem_write32(cpu.mem, table + 0x4Cu, close_r);
    mem_write32(cpu.mem, table + 0x58u, lseek_r);
    mem_write32(cpu.mem, table + 0x5Cu, read_r);
    mem_write32(cpu.mem, table + 0x6Cu, lock_close);
    rom_stubs_register(rom, getreent, test_rom_getreent_handler,
                       "test_getreent");
    rom_stubs_register(rom, free_r, test_rom_free_handler, "test_free_r");
    rom_stubs_register(rom, write_fn, test_rom_write_handler,
                       "test_file_write");
    rom_stubs_register(rom, close_r, test_rom_close_handler,
                       "test_close_r");
    rom_stubs_register(rom, read_r, test_rom_io_handler, "test_read_r");
    rom_stubs_register(rom, lseek_r, test_rom_io_handler, "test_lseek_r");
    rom_stubs_register(rom, lock_close,
                       test_rom_lock_close_recursive_handler,
                       "test_lock_close_recursive");

    rom_syscall_test_io_result = 5u;
    uint32_t read_args[] = { reent, cookie, buffer, 5u };
    ASSERT_EQ(call_builtin_rom_args(&cpu, 0x40001118u, read_args, 4), 5u);
    ASSERT_EQ(rom_syscall_test_io_calls, 1);
    for (int i = 0; i < 4; i++)
        ASSERT_EQ(rom_syscall_test_args[i], read_args[i]);

    rom_syscall_test_io_result = 123u;
    uint32_t seek_args[] = { reent, cookie, 23u, 1u };
    ASSERT_EQ(call_builtin_rom_args(&cpu, 0x40001184u, seek_args, 4), 123u);
    ASSERT_EQ(rom_syscall_test_io_calls, 2);
    for (int i = 0; i < 4; i++)
        ASSERT_EQ(rom_syscall_test_args[i], seek_args[i]);

    mem_load(cpu.mem, fp, empty_file, sizeof(empty_file));
    mem_load(cpu.mem, buffer, (const uint8_t *)"flexe", 5u);
    mem_write32(cpu.mem, fp + 0u, buffer + 5u);  /* _p */
    mem_write32(cpu.mem, fp + 8u, 0u);           /* _w */
    mem_write16(cpu.mem, fp + 12u, 0x0088u);     /* __SMBF | __SWR */
    mem_write32(cpu.mem, fp + 16u, buffer);      /* _bf._base */
    mem_write32(cpu.mem, fp + 20u, 64u);         /* _bf._size */
    mem_write32(cpu.mem, fp + 32u, cookie);      /* _cookie */
    mem_write32(cpu.mem, fp + 40u, write_fn);    /* _write */
    mem_write32(cpu.mem, fp + 48u, 0x400011B8u); /* _close = __sclose */
    mem_write32(cpu.mem, fp + 52u, ungetc_buffer);
    mem_write32(cpu.mem, fp + 72u, line_buffer);
    mem_write32(cpu.mem, fp + 88u, 0xF17E1234u);

    uint32_t fclose_args[] = { fp };
    ASSERT_EQ(call_builtin_rom_args(&cpu, 0x400020ACu, fclose_args, 1), 0u);
    ASSERT_EQ(rom_syscall_test_write_calls, 1);
    ASSERT_EQ(rom_syscall_test_write_args[0], reent);
    ASSERT_EQ(rom_syscall_test_write_args[1], cookie);
    ASSERT_EQ(rom_syscall_test_write_args[2], buffer);
    ASSERT_EQ(rom_syscall_test_write_args[3], 5u);
    ASSERT_EQ(rom_syscall_test_close_calls, 1);
    ASSERT_EQ(rom_syscall_test_close_args[0], reent);
    ASSERT_EQ(rom_syscall_test_close_args[1], cookie);
    ASSERT_EQ(rom_syscall_test_free_calls, 3);
    ASSERT_EQ(rom_syscall_test_freed[0], buffer);
    ASSERT_EQ(rom_syscall_test_freed[1], ungetc_buffer);
    ASSERT_EQ(rom_syscall_test_freed[2], line_buffer);
    ASSERT_EQ(rom_syscall_test_lock_calls, 1);
    ASSERT_EQ(rom_syscall_test_lock_arg, fp + 88u);
    ASSERT_EQ(mem_read32(cpu.mem, fp + 0u), buffer);
    ASSERT_EQ(mem_read32(cpu.mem, fp + 8u), 64u);
    ASSERT_EQ(mem_read16(cpu.mem, fp + 12u), 0u);
    ASSERT_EQ(mem_read32(cpu.mem, fp + 52u), 0u);
    ASSERT_EQ(mem_read32(cpu.mem, fp + 72u), 0u);

    /* The reentrant entry point shares the same lifecycle. An already-free
     * FILE is a successful no-op, while close callback failure still frees
     * the slot and is reported as EOF. */
    uint32_t fclose_r_args[] = { reent, fp };
    ASSERT_EQ(call_builtin_rom_args(&cpu, 0x40001FECu,
                                    fclose_r_args, 2), 0u);
    rom_syscall_test_close_result = -1;
    mem_write16(cpu.mem, fp + 12u, 0x0004u); /* __SRD */
    mem_write32(cpu.mem, fp + 32u, cookie + 1u);
    mem_write32(cpu.mem, fp + 48u, 0x400011B8u);
    ASSERT_EQ(call_builtin_rom_args(&cpu, 0x40001FECu,
                                    fclose_r_args, 2), (uint32_t)-1);
    ASSERT_EQ(rom_syscall_test_close_calls, 2);
    ASSERT_EQ(rom_syscall_test_close_args[1], cookie + 1u);
    ASSERT_EQ(rom_syscall_test_lock_calls, 2);
    ASSERT_EQ(mem_read16(cpu.mem, fp + 12u), 0u);
    ASSERT_EQ(rom_stubs_unregistered_count(rom), 0);

    rom_stubs_destroy(rom);
    teardown(&cpu);
}

TEST(test_rom_open_dispatches_through_guest_syscall_table) {
    xtensa_cpu_t cpu;
    setup(&cpu);
    esp32_rom_stubs_t *rom = rom_stubs_create(&cpu);

    const uint32_t table = 0x3FFB1000u;
    const uint32_t getreent = 0x400D0100u;
    const uint32_t open_r = 0x400D0200u;
    const uint32_t path = 0x3FFB2000u;
    rom_syscall_test_reent = 0x3FFB3000u;
    memset(rom_syscall_test_args, 0, sizeof(rom_syscall_test_args));

    /* Core 0 selects syscall_table_ptr_pro. */
    mem_write32(cpu.mem, 0x3FFAE024u, table);
    mem_write32(cpu.mem, table + 0x00u, getreent);
    mem_write32(cpu.mem, table + 0x50u, open_r);
    rom_stubs_register(rom, getreent, test_rom_getreent_handler,
                       "test_getreent");
    rom_stubs_register(rom, open_r, test_rom_open_r_handler,
                       "test_open_r");

    cpu.pc = 0x4000178Cu;
    XT_PS_SET_CALLINC(cpu.ps, 0);
    ar_write(&cpu, 0, BASE);
    ar_write(&cpu, 2, path);
    ar_write(&cpu, 3, 2u);
    ar_write(&cpu, 4, 0644u);
    xtensa_step(&cpu);

    ASSERT_EQ(cpu.pc, BASE);
    ASSERT_EQ(ar_read(&cpu, 2), 37u);
    ASSERT_EQ(rom_syscall_test_args[0], rom_syscall_test_reent);
    ASSERT_EQ(rom_syscall_test_args[1], path);
    ASSERT_EQ(rom_syscall_test_args[2], 2u);
    ASSERT_EQ(rom_syscall_test_args[3], 0644u);
    ASSERT_EQ(rom_stubs_unregistered_count(rom), 0);

    rom_stubs_destroy(rom);
    teardown(&cpu);
}

TEST(test_rom_open_fails_when_syscall_table_is_uninitialized) {
    xtensa_cpu_t cpu;
    setup(&cpu);
    esp32_rom_stubs_t *rom = rom_stubs_create(&cpu);

    cpu.pc = 0x4000178Cu;
    XT_PS_SET_CALLINC(cpu.ps, 0);
    ar_write(&cpu, 0, BASE);
    ar_write(&cpu, 2, 0x3FFB2000u);
    ar_write(&cpu, 3, 0u);
    ar_write(&cpu, 4, 0u);
    xtensa_step(&cpu);

    ASSERT_EQ(cpu.pc, BASE);
    ASSERT_EQ(ar_read(&cpu, 2), (uint32_t)-1);
    ASSERT_EQ(rom_stubs_unregistered_count(rom), 0);

    rom_stubs_destroy(rom);
    teardown(&cpu);
}

TEST(test_firmware_phy_wrapper_installs_virtual_table) {
    xtensa_cpu_t cpu;
    setup(&cpu);
    esp32_rom_stubs_t *rom = rom_stubs_create(&cpu);

    const uint32_t wrapper = 0x40189A2Cu;
    const uint32_t rom_literal = wrapper - 0x104u;
    const uint32_t global_literal = wrapper - 0x100u;
    const uint32_t phy_global = 0x3FFB2000u;

    /* Complete production wrapper with only its two L32R relocations moved. */
    put_insn3(&cpu, wrapper, 0x004136u); /* entry a1, 32 */
    put_insn3(&cpu, wrapper + 3u,
              encode_test_l32r(wrapper + 3u, rom_literal, 8));
    put_insn3(&cpu, wrapper + 6u, 0x0008E0u); /* callx8 a8 */
    put_insn3(&cpu, wrapper + 9u,
              encode_test_l32r(wrapper + 9u, global_literal, 8));
    put_insn2(&cpu, wrapper + 12u, 0x08A9u); /* s32i.n a10, a8, 0 */
    put_insn2(&cpu, wrapper + 14u, 0xF03Du); /* retw.n */
    mem_write32(cpu.mem, rom_literal, 0x40004100u);
    mem_write32(cpu.mem, global_literal, phy_global);

    ASSERT_EQ(rom_stubs_hook_firmware_addrs(rom, 0x40089268u), 4);
    cpu.pc = wrapper;
    XT_PS_SET_CALLINC(cpu.ps, 0);
    ar_write(&cpu, 0, BASE);
    xtensa_step(&cpu);

    uint32_t phy = mem_read32(cpu.mem, phy_global);
    ASSERT_EQ(cpu.pc, BASE);
    ASSERT_EQ(phy, 0x50001900u);
    ASSERT_EQ(mem_read32(cpu.mem, phy), 0x4006FFF0u);
    ASSERT_EQ(mem_read32(cpu.mem, phy + 0x11Cu), 0x4006FFF0u);
    ASSERT_EQ(mem_read32(cpu.mem, phy + 0x120u), 0);
    ASSERT_EQ(mem_read32(cpu.mem, phy + 0x180u), 0);
    ASSERT_EQ(mem_read32(cpu.mem, phy + 0x1A4u), 0x4006FFF0u);

    /* An indirect call through any populated slot is deterministic and is
     * accounted as a known virtual operation, not an unknown ROM call. */
    ar_write(&cpu, 0, BASE);
    ar_write(&cpu, 2, 0xDEADBEEFu);
    cpu.pc = mem_read32(cpu.mem, phy);
    xtensa_step(&cpu);
    ASSERT_EQ(cpu.pc, BASE);
    ASSERT_EQ(ar_read(&cpu, 2), 0);
    ASSERT_EQ(rom_stubs_unregistered_count(rom), 0);

    rom_stubs_destroy(rom);
    teardown(&cpu);
}

TEST(test_firmware_phy_wrapper_has_no_address_fallback) {
    xtensa_cpu_t cpu;
    setup(&cpu);
    esp32_rom_stubs_t *rom = rom_stubs_create(&cpu);
    const uint32_t wrapper = 0x40189A2Cu;
    const uint32_t former_fallback_global = 0x3FFC87ECu;

    /* NerdMiner's historical entry point still selects its compatibility
     * hooks, but an absent/corrupt wrapper must not authorize a guessed DRAM
     * write merely because the PC happens to equal an old release address. */
    mem_write32(cpu.mem, former_fallback_global, 0xA5A5A5A5u);
    ASSERT_EQ(rom_stubs_hook_firmware_addrs(rom, 0x40089268u), 4);
    cpu.pc = wrapper;
    cpu._pc_written = true;
    XT_PS_SET_CALLINC(cpu.ps, 0);
    ar_write(&cpu, 0, BASE);
    xtensa_step(&cpu);
    ASSERT_EQ(cpu.pc, BASE);
    ASSERT_EQ(mem_read32(cpu.mem, former_fallback_global), 0xA5A5A5A5u);

    rom_stubs_destroy(rom);
    teardown(&cpu);
}

TEST(test_structural_abi_accels_do_not_require_firmware_profile) {
    xtensa_cpu_t cpu;
    setup(&cpu);
    esp32_rom_stubs_t *rom = rom_stubs_create(&cpu);
    const uint32_t vector_base = 0x4007C000u;
    const uint32_t poll0 = 0x4007A123u;
    const uint32_t poll1 = 0x4007B234u;

    seed_canonical_window_vectors(&cpu, vector_base);
    seed_flash_poll_loop(&cpu, poll0);
    seed_flash_poll_loop(&cpu, poll1);
    ASSERT_TRUE(xtensa_window_vectors_are_canonical(cpu.mem, vector_base));
    mem_write8(cpu.mem, vector_base + VECOFS_WINDOW_UNDERFLOW12 + 1u, 0u);
    ASSERT_FALSE(xtensa_window_vectors_are_canonical(cpu.mem, vector_base));
    seed_canonical_window_vectors(&cpu, vector_base);

    /* The entry is deliberately unknown. Discovery depends only on complete
     * ABI instruction bodies, not an application fingerprint or fixed PC. */
    ASSERT_EQ(rom_stubs_hook_firmware_addrs(rom, 0x40081234u), 6u);
    ASSERT_EQ(cpu.poll_spin_count, 2u);
    ASSERT_EQ(cpu.poll_spin_pc[0], poll0);
    ASSERT_EQ(cpu.poll_spin_pc[1], poll1);
    ASSERT_EQ(cpu.poll_spin_insns, 4u);
    ASSERT_TRUE(cpu.accelerated_blocks);

    const uint32_t frame_top = 0x3FFB6000u;
    const uint32_t resume = BASE + 0x600u;
    const uint32_t saved[] = {
        0x40001234u, 0x3FFB7000u, 0xA5A50002u, 0xA5A50003u,
    };
    cpu.real_window_vectors = true;
    cpu.vecbase = vector_base;
    cpu.windowbase = 5u;
    cpu.windowstart = (1u << 2) | (1u << 5);
    cpu.ps = 0u;
    XT_PS_SET_EXCM(cpu.ps, 1u);
    XT_PS_SET_OWB(cpu.ps, 2u);
    cpu.epc[0] = resume;
    for (unsigned i = 0; i < 4u; i++)
        ar_write(&cpu, (int)i, saved[i]);
    ar_write(&cpu, 5, frame_top);
    cpu.pc = vector_base + VECOFS_WINDOW_OVERFLOW4;
    cpu._pc_written = true;
    cpu.running = true;

    ASSERT_EQ(xtensa_run(&cpu, 5), 5u);
    ASSERT_EQ(cpu.pc, resume);
    ASSERT_EQ(cpu.windowbase, 2u);
    ASSERT_FALSE(cpu.windowstart & (1u << 5));
    for (unsigned i = 0; i < 4u; i++)
        ASSERT_EQ(mem_read32(cpu.mem, frame_top - 16u + i * 4u), saved[i]);

    rom_stubs_destroy(rom);
    teardown(&cpu);
}

TEST(test_newlib_memcmp_is_relocated_and_cycle_exact) {
    xtensa_cpu_t cpu;
    setup(&cpu);
    esp32_rom_stubs_t *rom = rom_stubs_create(&cpu);
    const uint32_t addr = 0x4007D100u;
    const uint32_t lhs = 0x3FFB0100u;
    const uint32_t rhs = 0x3FFB0200u;
    static const uint8_t lhs_bytes[] = { 0x11, 0x20, 0x33, 0x44 };
    static const uint8_t rhs_bytes[] = { 0x11, 0x40, 0x33, 0x44 };

    seed_newlib_optimized_memcmp(&cpu, addr);
    mem_write8(cpu.mem, addr + 81u, 0u);
    ASSERT_EQ(rom_stubs_hook_firmware_addrs(rom, 0x40081234u), 0u);

    seed_newlib_optimized_memcmp(&cpu, addr);
    ASSERT_EQ(rom_stubs_hook_firmware_addrs(rom, 0x40081234u), 1u);
    ASSERT_TRUE(cpu.accelerated_blocks);
    put_test_bytes(&cpu, lhs, lhs_bytes, sizeof(lhs_bytes));
    put_test_bytes(&cpu, rhs, rhs_bytes, sizeof(rhs_bytes));

    cpu.pc = addr;
    cpu._pc_written = true;
    cpu.running = true;
    cpu.windowbase = 0u;
    cpu.windowstart = 1u;
    cpu.next_timer_event = UINT32_MAX;
    XT_PS_SET_CALLINC(cpu.ps, 2u);
    ar_write(&cpu, 8, (2u << 30) | (BASE & 0x3FFFFFFFu));
    ar_write(&cpu, 10, lhs);
    ar_write(&cpu, 11, rhs);
    ar_write(&cpu, 12, sizeof(lhs_bytes));
    /* One unequal aligned word falls back to one equal and one unequal byte:
     * the canonical implementation retires exactly 32 instructions. */
    ASSERT_EQ(xtensa_run(&cpu, 32), 32u);
    ASSERT_EQ(cpu.pc, BASE);
    ASSERT_EQ(ar_read(&cpu, 10), (uint32_t)-0x20);
    ASSERT_EQ(cpu.ccount, 32u);
    ASSERT_EQ64(cpu.cycle_count, 32u);
    ASSERT_EQ64(cpu.insn_count, 32u);

    mem_write8(cpu.mem, rhs + 1u, 0x20u);
    cpu.pc = addr;
    cpu._pc_written = true;
    cpu.ccount = 100u;
    cpu.cycle_count = 100u;
    cpu.insn_count = 0u;
    XT_PS_SET_CALLINC(cpu.ps, 2u);
    ar_write(&cpu, 8, (2u << 30) | (BASE & 0x3FFFFFFFu));
    ar_write(&cpu, 10, lhs);
    ar_write(&cpu, 11, rhs);
    ar_write(&cpu, 12, sizeof(lhs_bytes));
    ASSERT_EQ(xtensa_run(&cpu, 23), 23u);
    ASSERT_EQ(cpu.pc, BASE);
    ASSERT_EQ(ar_read(&cpu, 10), 0u);
    ASSERT_EQ(cpu.ccount, 123u);
    ASSERT_EQ64(cpu.cycle_count, 123u);
    ASSERT_EQ64(cpu.insn_count, 23u);
    ASSERT_EQ(rom_stubs_total_calls(rom), 2u);

    rom_stubs_destroy(rom);
    teardown(&cpu);
}

static void init_newlib_memcmp_call(xtensa_cpu_t *cpu, uint32_t addr,
                                    uint32_t lhs, uint32_t rhs,
                                    uint32_t size, unsigned callinc) {
    for (unsigned i = 0u; i < 64u; i++)
        cpu->ar[i] = 0xA5000000u + i;
    cpu->pc = addr;
    cpu->_pc_written = true;
    cpu->windowbase = 3u;
    cpu->windowstart = 1u << 3;
    cpu->ps = (1u << 18) | (9u << 8) | 1u;
    XT_PS_SET_CALLINC(cpu->ps, callinc);
    cpu->window_callsize[(3u + callinc) & 15u] = 0xBu;
    cpu->ccount = 1000u;
    cpu->cycle_count = 2000u;
    cpu->insn_count = 0u;
    cpu->next_timer_event = UINT32_MAX;
    cpu->running = true;
    cpu->halted = false;
    cpu->exception = false;
    cpu->seed_entry_link = false;
    cpu->breakpoint_count = 0;
    ar_write(cpu, 1, 0x3FFB7000u);
    ar_write(cpu, (int)(callinc * 4u),
             (callinc << 30) | (BASE & 0x3FFFFFFFu));
    ar_write(cpu, (int)(callinc * 4u + 2u), lhs);
    ar_write(cpu, (int)(callinc * 4u + 3u), rhs);
    ar_write(cpu, (int)(callinc * 4u + 4u), size);
}

TEST(test_newlib_memcmp_hook_matches_original_routine) {
    xtensa_cpu_t reference;
    xtensa_cpu_t accelerated;
    setup(&reference);
    setup(&accelerated);
    const uint32_t addr = 0x4007D100u;
    const uint32_t lhs = 0x3FFB0100u;
    const uint32_t rhs = 0x3FFB0200u;
    static const uint8_t lhs_bytes[] = {
        0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88,
        0x99, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0x00,
    };
    static const uint8_t rhs_bytes[] = {
        0x11, 0x22, 0x33, 0x44, 0x55, 0x46, 0x77, 0x88,
        0x99, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0x00,
    };
    static const struct {
        uint8_t lhs_offset;
        uint8_t rhs_offset;
        uint8_t size;
        uint8_t callinc;
        uint8_t same_buffer;
        int32_t result;
    } cases[] = {
        { 0u, 0u, 0u, 1u, 0u, 0 },     /* short, empty */
        { 1u, 1u, 3u, 2u, 0u, 0 },     /* short, all equal */
        { 1u, 1u, 5u, 1u, 0u, 0x20 },  /* unaligned byte mismatch */
        { 4u, 4u, 4u, 2u, 0u, 0x20 },  /* first word mismatch */
        { 0u, 0u, 8u, 1u, 0u, 0x20 },  /* later word mismatch */
        { 0u, 0u, 8u, 2u, 1u, 0 },     /* complete equal word loop */
    };

    seed_newlib_optimized_memcmp(&reference, addr);
    seed_newlib_optimized_memcmp(&accelerated, addr);
    put_test_bytes(&reference, lhs, lhs_bytes, sizeof(lhs_bytes));
    put_test_bytes(&reference, rhs, rhs_bytes, sizeof(rhs_bytes));
    put_test_bytes(&accelerated, lhs, lhs_bytes, sizeof(lhs_bytes));
    put_test_bytes(&accelerated, rhs, rhs_bytes, sizeof(rhs_bytes));
    esp32_rom_stubs_t *rom = rom_stubs_create(&accelerated);
    ASSERT_EQ(rom_stubs_hook_firmware_addrs(rom, 0x40081234u), 1u);

    for (unsigned c = 0u; c < sizeof(cases) / sizeof(cases[0]); c++) {
        uint32_t case_lhs = lhs + cases[c].lhs_offset;
        uint32_t case_rhs = cases[c].same_buffer
                          ? lhs + cases[c].rhs_offset
                          : rhs + cases[c].rhs_offset;
        init_newlib_memcmp_call(&reference, addr, case_lhs, case_rhs,
                                cases[c].size, cases[c].callinc);
        init_newlib_memcmp_call(&accelerated, addr, case_lhs, case_rhs,
                                cases[c].size, cases[c].callinc);
        for (unsigned step = 0u; step < 150u && reference.pc != BASE; step++)
            ASSERT_EQ(xtensa_step(&reference), 0u);
        ASSERT_EQ(reference.pc, BASE);
        ASSERT_EQ(xtensa_step(&accelerated), 0u);
        ASSERT_EQ(ar_read(&accelerated, (int)(cases[c].callinc * 4u + 2u)),
                  (uint32_t)cases[c].result);

        ASSERT_EQ(accelerated.pc, reference.pc);
        ASSERT_EQ(accelerated.ccount, reference.ccount);
        ASSERT_EQ64(accelerated.cycle_count, reference.cycle_count);
        ASSERT_EQ64(accelerated.insn_count, reference.insn_count);
        ASSERT_EQ(accelerated.ps, reference.ps);
        ASSERT_EQ(accelerated.windowbase, reference.windowbase);
        ASSERT_EQ(accelerated.windowstart, reference.windowstart);
        ASSERT_EQ(accelerated._pc_written, reference._pc_written);
        for (unsigned i = 0u; i < 64u; i++)
            ASSERT_EQ(accelerated.ar[i], reference.ar[i]);
        for (unsigned i = 0u; i < 16u; i++)
            ASSERT_EQ(accelerated.window_callsize[i],
                      reference.window_callsize[i]);
    }
    ASSERT_EQ(rom_stubs_total_calls(rom),
              sizeof(cases) / sizeof(cases[0]));

    rom_stubs_destroy(rom);
    teardown(&accelerated);
    teardown(&reference);
}

TEST(test_wled_v1601_uses_structural_memcmp_and_scanned_phy) {
    xtensa_cpu_t cpu;
    setup(&cpu);
    esp32_rom_stubs_t *rom = rom_stubs_create(&cpu);
    seed_wled_v1601_profile(&cpu);

    const uint32_t memcmp_entry = 0x4008B79Cu;
    const uint32_t wrapper = 0x401A7B00u;
    const uint32_t rom_literal = wrapper - 0x104u;
    const uint32_t global_literal = wrapper - 0x100u;
    const uint32_t phy_global = 0x3FFB2000u;
    seed_newlib_optimized_memcmp(&cpu, memcmp_entry);
    seed_flash_poll_loop(&cpu, 0x40083A68u);
    seed_flash_poll_loop(&cpu, 0x40083B29u);
    put_insn3(&cpu, wrapper, 0x004136u);
    put_insn3(&cpu, wrapper + 3u,
              encode_test_l32r(wrapper + 3u, rom_literal, 8));
    put_insn3(&cpu, wrapper + 6u, 0x0008E0u); /* callx8 a8 */
    put_insn3(&cpu, wrapper + 9u,
              encode_test_l32r(wrapper + 9u, global_literal, 8));
    put_insn2(&cpu, wrapper + 12u, 0x08A9u); /* s32i.n a10, a8, 0 */
    put_insn2(&cpu, wrapper + 14u, 0xF03Du); /* retw.n */
    mem_write32(cpu.mem, rom_literal, 0x40004100u);
    mem_write32(cpu.mem, global_literal, phy_global);

    ASSERT_EQ(rom_stubs_hook_firmware_addrs(rom, 0x40083E68u), 2);
    ASSERT_EQ(cpu.poll_spin_count, 2u);
    ASSERT_EQ(cpu.poll_spin_pc[0], 0x40083A68u);
    ASSERT_EQ(cpu.poll_spin_pc[1], 0x40083B29u);
    ASSERT_EQ(cpu.poll_spin_insns, 4u);

    const uint32_t lhs = 0x3FFB0100u;
    const uint32_t rhs = 0x3FFB0200u;
    static const uint8_t lhs_bytes[] = { 0x11, 0x20, 0x33, 0x44 };
    static const uint8_t rhs_bytes[] = { 0x11, 0x40, 0x33, 0x44 };
    put_test_bytes(&cpu, lhs, lhs_bytes, sizeof(lhs_bytes));
    put_test_bytes(&cpu, rhs, rhs_bytes, sizeof(rhs_bytes));
    cpu.pc = memcmp_entry;
    cpu._pc_written = true;
    cpu.windowbase = 0u;
    cpu.windowstart = 1u;
    cpu.next_timer_event = UINT32_MAX;
    XT_PS_SET_CALLINC(cpu.ps, 2u);
    ar_write(&cpu, 8, (2u << 30) | (BASE & 0x3FFFFFFFu));
    ar_write(&cpu, 10, lhs);
    ar_write(&cpu, 11, rhs);
    ar_write(&cpu, 12, sizeof(lhs_bytes));
    xtensa_step(&cpu);
    ASSERT_EQ(cpu.pc, BASE);
    ASSERT_EQ(ar_read(&cpu, 10), (uint32_t)-0x20);

    cpu.pc = wrapper;
    cpu._pc_written = true;
    XT_PS_SET_CALLINC(cpu.ps, 0u);
    ar_write(&cpu, 0, BASE);
    xtensa_step(&cpu);
    ASSERT_EQ(cpu.pc, BASE);
    ASSERT_EQ(mem_read32(cpu.mem, phy_global), 0x50001900u);
    ASSERT_EQ(rom_stubs_total_calls(rom), 2u);

    rom_stubs_destroy(rom);
    teardown(&cpu);
}

static const uint8_t idf_watchpoint_code[] = {
    0x36, 0x41, 0x00, 0x0C, 0x18, 0xBD, 0x05, 0xCD,
    0x08, 0x8C, 0xB5, 0x0B, 0x55, 0x92, 0xA0, 0x00,
    0x50, 0x98, 0x93, 0x90, 0xC0, 0x74, 0xBD, 0x08,
    0x3C, 0xF8, 0x0C, 0x0A, 0x52, 0xA0, 0x01, 0x0C,
    0x79, 0x76, 0x89, 0x0D, 0x00, 0x1A, 0x40, 0x00,
    0xD5, 0xA1, 0xD7, 0x14, 0x04, 0xF0, 0x88, 0x11,
    0x1B, 0xAA, 0x80, 0x80, 0x54, 0x8C, 0x4C, 0x51,
    0x53, 0xC0, 0x50, 0x88, 0x20, 0x8C, 0x4B, 0x51,
    0x59, 0xC0, 0x50, 0x88, 0x20, 0x8C, 0x72, 0x30,
    0x91, 0x13, 0x80, 0xA1, 0x13, 0x1D, 0xF0, 0x00,
    0x30, 0x90, 0x13, 0x80, 0xA0, 0x13, 0xC6, 0xFC,
    0xFF,
};

static void seed_idf_watchpoint(xtensa_cpu_t *cpu, uint32_t addr) {
    const uint32_t read_literal = (addr - 0x100u) & ~3u;
    const uint32_t write_literal = read_literal + 4u;
    put_test_bytes(cpu, addr, idf_watchpoint_code,
                   sizeof(idf_watchpoint_code));
    put_insn3(cpu, addr + 55u,
              encode_test_l32r(addr + 55u, read_literal, 5));
    put_insn3(cpu, addr + 63u,
              encode_test_l32r(addr + 63u, write_literal, 5));
    mem_write32(cpu->mem, read_literal, 0x40000000u);
    mem_write32(cpu->mem, write_literal, 0x80000000u);
}

static void init_idf_watchpoint_call(xtensa_cpu_t *cpu, uint32_t addr,
                                      uint32_t id,
                                      uint32_t address, uint32_t size,
                                      uint32_t trigger) {
    for (unsigned i = 0u; i < 64u; i++)
        cpu->ar[i] = 0xA5000000u + i;
    cpu->pc = addr;
    cpu->_pc_written = true;
    cpu->windowbase = 3u;
    cpu->windowstart = 1u << 3;
    cpu->ps = (1u << 18) | (9u << 8) | 1u;
    XT_PS_SET_CALLINC(cpu->ps, 2u);
    cpu->window_callsize[5] = 0xBu;
    cpu->sar = 11u;
    cpu->lbeg = 0x41234560u;
    cpu->lend = 0x41234570u;
    cpu->lcount = 17u;
    cpu->dbreaka[0] = 0x11111111u;
    cpu->dbreaka[1] = 0x22222222u;
    cpu->dbreakc[0] = 0x33333333u;
    cpu->dbreakc[1] = 0x44444444u;
    cpu->ccount = 1000u;
    cpu->cycle_count = 2000u;
    cpu->insn_count = 0u;
    cpu->next_timer_event = UINT32_MAX;
    cpu->running = true;
    cpu->halted = false;
    cpu->exception = false;
    cpu->seed_entry_link = false;
    cpu->breakpoint_count = 0;
    ar_write(cpu, 1, 0x3FFB7000u);
    ar_write(cpu, 8, (2u << 30) | (BASE & 0x3FFFFFFFu));
    ar_write(cpu, 10, id);
    ar_write(cpu, 11, address);
    ar_write(cpu, 12, size);
    ar_write(cpu, 13, trigger);
}

TEST(test_idf_watchpoint_is_relocated_and_matches_original_routine) {
    const uint32_t addr = 0x4007D300u;
    struct watchpoint_case {
        uint32_t id;
        uint32_t address;
        uint32_t size;
        uint32_t trigger;
        uint32_t insns;
    } cases[] = {
        { 0u, 0x3FFB1234u, 4u, 2u, 40u },
        { 1u, 0x3FFB5678u, 1u, 0u, 22u },
        { 7u, 0x3FFB9ABCu, 3u, 1u, 59u },
    };

    /* Neither a bad relocation target nor one corrupt body byte may authorize
     * a native substitute, regardless of firmware identity. */
    xtensa_cpu_t invalid;
    setup(&invalid);
    esp32_rom_stubs_t *invalid_rom = rom_stubs_create(&invalid);
    seed_idf_watchpoint(&invalid, addr);
    mem_write32(invalid.mem, (addr - 0x100u) & ~3u, 0u);
    ASSERT_EQ(rom_stubs_hook_firmware_addrs(
                      invalid_rom, 0x40081234u), 0u);
    seed_idf_watchpoint(&invalid, addr);
    mem_write8(invalid.mem, addr + 20u, 0u);
    ASSERT_EQ(rom_stubs_hook_firmware_addrs(
                      invalid_rom, 0x40081234u), 0u);
    rom_stubs_destroy(invalid_rom);
    teardown(&invalid);

    /* A structurally known ENTRY is exact. A coincidental ENTRY encoding
     * immediately before it must never acquire a second hook. */
    xtensa_cpu_t exact;
    setup(&exact);
    seed_idf_watchpoint(&exact, addr);
    put_insn3(&exact, addr - 8u, 0x004136u);
    esp32_rom_stubs_t *exact_rom = rom_stubs_create(&exact);
    ASSERT_EQ(rom_stubs_hook_firmware_addrs(
                      exact_rom, 0x40081234u), 1u);
    exact.pc = addr - 8u;
    exact._pc_written = true;
    exact.windowbase = 0u;
    exact.windowstart = 1u;
    XT_PS_SET_CALLINC(exact.ps, 0u);
    ar_write(&exact, 1, 0x3FFB7000u);
    ASSERT_EQ(xtensa_step(&exact), 0u);
    ASSERT_EQ(exact.pc, addr - 5u);
    ASSERT_EQ(rom_stubs_total_calls(exact_rom), 0u);
    rom_stubs_destroy(exact_rom);
    teardown(&exact);

    /* A native span must not postpone an interrupt that guest execution
     * would take after its first instruction. */
    xtensa_cpu_t irq_reference;
    xtensa_cpu_t irq_accelerated;
    setup(&irq_reference);
    setup(&irq_accelerated);
    seed_idf_watchpoint(&irq_reference, addr);
    seed_idf_watchpoint(&irq_accelerated, addr);
    esp32_rom_stubs_t *irq_rom = rom_stubs_create(&irq_accelerated);
    ASSERT_EQ(rom_stubs_hook_firmware_addrs(
                      irq_rom, 0x40081234u), 1u);
    init_idf_watchpoint_call(
            &irq_reference, addr, 0u, 0x3FFB1234u, 4u, 2u);
    init_idf_watchpoint_call(
            &irq_accelerated, addr, 0u, 0x3FFB1234u, 4u, 2u);
    irq_reference.interrupt = irq_accelerated.interrupt = 1u << 11;
    irq_reference.intenable = irq_accelerated.intenable = 1u << 11;
    irq_reference.irq_check = irq_accelerated.irq_check = true;
    ASSERT_EQ(xtensa_step(&irq_reference), (uint32_t)-1);
    ASSERT_EQ(xtensa_step(&irq_accelerated), (uint32_t)-1);
    ASSERT_EQ(irq_accelerated.pc, irq_reference.pc);
    ASSERT_EQ(irq_accelerated.ps, irq_reference.ps);
    ASSERT_EQ(irq_accelerated.windowbase, irq_reference.windowbase);
    ASSERT_EQ(irq_accelerated.windowstart, irq_reference.windowstart);
    ASSERT_EQ(irq_accelerated.epc[2], irq_reference.epc[2]);
    ASSERT_EQ(irq_accelerated.eps[2], irq_reference.eps[2]);
    ASSERT_EQ(irq_accelerated.exception, irq_reference.exception);
    ASSERT_EQ(irq_accelerated.running, irq_reference.running);
    ASSERT_EQ(irq_accelerated.ccount, irq_reference.ccount);
    ASSERT_EQ64(irq_accelerated.cycle_count, irq_reference.cycle_count);
    ASSERT_EQ64(irq_accelerated.insn_count, irq_reference.insn_count);
    for (unsigned i = 0u; i < 64u; i++)
        ASSERT_EQ(irq_accelerated.ar[i], irq_reference.ar[i]);
    rom_stubs_destroy(irq_rom);
    teardown(&irq_accelerated);
    teardown(&irq_reference);

    for (unsigned c = 0u; c < sizeof(cases) / sizeof(cases[0]); c++) {
        xtensa_cpu_t reference;
        xtensa_cpu_t accelerated;
        setup(&reference);
        setup(&accelerated);
        seed_idf_watchpoint(&reference, addr);
        seed_idf_watchpoint(&accelerated, addr);
        esp32_rom_stubs_t *rom = rom_stubs_create(&accelerated);
        ASSERT_EQ(rom_stubs_hook_firmware_addrs(
                          rom, 0x40081234u), 1u);

        init_idf_watchpoint_call(&reference, addr, cases[c].id,
                                 cases[c].address, cases[c].size,
                                 cases[c].trigger);
        init_idf_watchpoint_call(&accelerated, addr, cases[c].id,
                                 cases[c].address, cases[c].size,
                                 cases[c].trigger);

        for (unsigned step = 0u; step < 100u && reference.pc != BASE; step++)
            ASSERT_EQ(xtensa_step(&reference), 0u);
        ASSERT_EQ(reference.pc, BASE);
        ASSERT_EQ64(reference.insn_count, cases[c].insns);

        ASSERT_EQ(xtensa_step(&accelerated), 0u);
        ASSERT_EQ(accelerated.pc, reference.pc);
        ASSERT_EQ(accelerated.ccount, reference.ccount);
        ASSERT_EQ64(accelerated.cycle_count, reference.cycle_count);
        ASSERT_EQ64(accelerated.insn_count, reference.insn_count);
        ASSERT_EQ(accelerated.ps, reference.ps);
        ASSERT_EQ(accelerated.sar, reference.sar);
        ASSERT_EQ(accelerated.lbeg, reference.lbeg);
        ASSERT_EQ(accelerated.lend, reference.lend);
        ASSERT_EQ(accelerated.lcount, reference.lcount);
        ASSERT_EQ(accelerated.windowbase, reference.windowbase);
        ASSERT_EQ(accelerated.windowstart, reference.windowstart);
        ASSERT_EQ(accelerated._pc_written, reference._pc_written);
        ASSERT_EQ(accelerated.dbreaka[0], reference.dbreaka[0]);
        ASSERT_EQ(accelerated.dbreaka[1], reference.dbreaka[1]);
        ASSERT_EQ(accelerated.dbreakc[0], reference.dbreakc[0]);
        ASSERT_EQ(accelerated.dbreakc[1], reference.dbreakc[1]);
        for (unsigned i = 0u; i < 64u; i++)
            ASSERT_EQ(accelerated.ar[i], reference.ar[i]);
        for (unsigned i = 0u; i < 16u; i++)
            ASSERT_EQ(accelerated.window_callsize[i],
                      reference.window_callsize[i]);

        rom_stubs_destroy(rom);
        teardown(&accelerated);
        teardown(&reference);
    }

    /* A timer due at the final instruction is observable inside the original
     * routine, so the accelerator must leave that invocation to the guest. */
    xtensa_cpu_t boundary;
    setup(&boundary);
    seed_idf_watchpoint(&boundary, addr);
    esp32_rom_stubs_t *boundary_rom = rom_stubs_create(&boundary);
    ASSERT_EQ(rom_stubs_hook_firmware_addrs(
                      boundary_rom, 0x40081234u), 1u);
    init_idf_watchpoint_call(
            &boundary, addr, 0u, 0x3FFB1234u, 4u, 2u);
    boundary.next_timer_event = boundary.ccount + 40u;
    ASSERT_EQ(xtensa_step(&boundary), 0u);
    ASSERT_EQ(boundary.pc, addr + 3u);
    ASSERT_EQ(boundary.ccount, 1001u);
    ASSERT_EQ64(boundary.insn_count, 1u);
    ASSERT_EQ(boundary.dbreaka[0], 0x11111111u);
    rom_stubs_destroy(boundary_rom);
    teardown(&boundary);

    /* Nor may a native span cross the dual-core scheduler's batch edge. The
     * 40-instruction call executes 39 real instructions in this batch and
     * returns on the first instruction of the next one. */
    xtensa_cpu_t batch_edge;
    setup(&batch_edge);
    seed_idf_watchpoint(&batch_edge, addr);
    esp32_rom_stubs_t *batch_rom = rom_stubs_create(&batch_edge);
    ASSERT_EQ(rom_stubs_hook_firmware_addrs(
                      batch_rom, 0x40081234u), 1u);
    init_idf_watchpoint_call(
            &batch_edge, addr, 0u, 0x3FFB1234u, 4u, 2u);
    ASSERT_EQ(xtensa_run(&batch_edge, 39), 39u);
    ASSERT_EQ(batch_edge.pc, addr + 0x4Du);
    ASSERT_EQ(batch_edge.ccount, 1039u);
    ASSERT_EQ64(batch_edge.insn_count, 39u);
    ASSERT_EQ(xtensa_run(&batch_edge, 1), 1u);
    ASSERT_EQ(batch_edge.pc, BASE);
    ASSERT_EQ(batch_edge.ccount, 1040u);
    ASSERT_EQ64(batch_edge.insn_count, 40u);
    rom_stubs_destroy(batch_rom);
    teardown(&batch_edge);
}

static const uint8_t idf_port_enter_code[] = {
    0x36, 0x81, 0x00, 0x9D, 0x03, 0x70, 0x63, 0x00,
    0x60, 0x63, 0x00, 0x26, 0x03, 0x02, 0xD0, 0xEA,
    0x03, 0x50, 0xEB, 0x03, 0x41, 0x41, 0xC8, 0x40,
    0x45, 0x30, 0x81, 0x40, 0xC8, 0x8A, 0x82, 0xE1,
    0x04, 0xC7, 0x31, 0x3F, 0xC8, 0x87, 0xBE, 0x28,
    0x66, 0x09, 0x12, 0x8D, 0x05, 0x30, 0x0C, 0x13,
    0x82, 0xE2, 0x00, 0x89, 0x71, 0x87, 0x14, 0xF2,
    0xC0, 0x20, 0x00, 0x86, 0x1C, 0x00, 0xAD, 0x05,
    0x30, 0x0C, 0x13, 0xA2, 0xE2, 0x00, 0xA9, 0x71,
    0xA7, 0x14, 0x49, 0xC0, 0x20, 0x00, 0xC6, 0x17,
    0x00, 0x66, 0x09, 0x19, 0xC2, 0xC1, 0x1C, 0xBD,
    0x03, 0xAD, 0x02, 0x59, 0x71, 0xC0, 0x20, 0x00,
    0x81, 0x33, 0xC8, 0xE0, 0x08, 0x00, 0x88, 0x71,
    0x87, 0x14, 0xE8, 0x86, 0x10, 0x00, 0xAD, 0x02,
    0xC2, 0xC1, 0x1C, 0xBD, 0x03, 0x89, 0x21, 0x99,
    0x11, 0xD9, 0x31, 0xE9, 0x01, 0x59, 0x71, 0xC0,
    0x20, 0x00, 0x81, 0x2A, 0xC8, 0xE0, 0x08, 0x00,
    0xA8, 0x71, 0x88, 0x21, 0x98, 0x11, 0xD8, 0x31,
    0xE8, 0x01, 0xA7, 0x94, 0x1B, 0xA0, 0xEA, 0x03,
    0xD0, 0xAA, 0xC0, 0xA7, 0xB9, 0x86, 0x60, 0xE6,
    0x13, 0x10, 0x20, 0x00, 0xAD, 0x07, 0x81, 0x2E,
    0xC6, 0xE0, 0x08, 0x00, 0x0C, 0x02, 0x46, 0x0C,
    0x00, 0x38, 0x12, 0x1B, 0x33, 0x39, 0x12, 0x60,
    0xE6, 0x13, 0x10, 0x20, 0x00, 0x40, 0xEB, 0x03,
    0x40, 0x4D, 0x04, 0xE0, 0x44, 0x11, 0x31, 0x17,
    0xC8, 0x4A, 0x33, 0x28, 0x03, 0x1B, 0x22, 0x29,
    0x03, 0x66, 0x12, 0x0C, 0x31, 0x15, 0xC8, 0x4A,
    0x43, 0x79, 0x04, 0xC0, 0x20, 0x00, 0x46, 0x00,
    0x00, 0x0C, 0x12, 0x1D, 0xF0,
};

static uint32_t idf_port_enter_literal_base(uint32_t addr) {
    return (addr - 0x100u) & ~3u;
}

static void seed_idf_port_enter(xtensa_cpu_t *cpu, uint32_t addr,
                                uint32_t nesting_addr,
                                uint32_t old_state_addr) {
    uint32_t literal = idf_port_enter_literal_base(addr);
    put_test_bytes(cpu, addr, idf_port_enter_code,
                   sizeof(idf_port_enter_code));
    put_insn3(cpu, addr + 20u,
              encode_test_l32r(addr + 20u, literal, 4));
    put_insn3(cpu, addr + 26u,
              encode_test_l32r(addr + 26u, literal + 4u, 8));
    put_insn3(cpu, addr + 31u,
              encode_test_l32r(addr + 31u, literal + 8u, 14));
    put_insn3(cpu, addr + 34u,
              encode_test_l32r(addr + 34u, literal + 12u, 3));
    put_insn3(cpu, addr + 96u,
              encode_test_l32r(addr + 96u, literal + 16u, 8));
    put_insn3(cpu, addr + 130u,
              encode_test_l32r(addr + 130u, literal + 16u, 8));
    put_insn3(cpu, addr + 166u,
              encode_test_l32r(addr + 166u, literal + 20u, 8));
    put_insn3(cpu, addr + 198u,
              encode_test_l32r(addr + 198u, literal + 24u, 3));
    put_insn3(cpu, addr + 212u,
              encode_test_l32r(addr + 212u, literal + 28u, 3));
    mem_write32(cpu->mem, literal,
                XTENSA_SPINLOCK_OWNER_CORE0 ^
                XTENSA_SPINLOCK_OWNER_CORE1);
    mem_write32(cpu->mem, literal + 4u, 0xC0800000u);
    mem_write32(cpu->mem, literal + 8u, 0x003FFFFFu);
    mem_write32(cpu->mem, literal + 12u, XTENSA_SPINLOCK_FREE);
    mem_write32(cpu->mem, literal + 16u, 0x40081200u);
    mem_write32(cpu->mem, literal + 20u, 0x4000BFDCu);
    mem_write32(cpu->mem, literal + 24u, nesting_addr);
    mem_write32(cpu->mem, literal + 28u, old_state_addr);
}

static void init_idf_port_enter_call(xtensa_cpu_t *cpu, uint32_t addr,
                                     uint32_t mux, uint32_t nesting_addr,
                                     uint32_t old_state_addr,
                                     uint32_t owner, uint32_t lock_count,
                                     uint32_t nesting, uint32_t old_state,
                                     uint32_t timeout, unsigned callinc,
                                     unsigned core) {
    for (unsigned i = 0u; i < 64u; i++)
        cpu->ar[i] = 0x5A000000u + i;
    cpu->pc = addr;
    cpu->_pc_written = true;
    cpu->windowbase = 3u;
    cpu->windowstart = 1u << 3;
    cpu->ps = (1u << 18) | (9u << 8) | 1u;
    XT_PS_SET_CALLINC(cpu->ps, callinc);
    cpu->window_callsize[(3u + callinc) & 15u] = 0xBu;
    cpu->prid = core == 0u ? XTENSA_SPINLOCK_OWNER_CORE0
                           : XTENSA_SPINLOCK_OWNER_CORE1;
    cpu->core_id = (int)core;
    cpu->ccount = 1000u;
    cpu->cycle_count = 2000u;
    cpu->insn_count = 0u;
    cpu->next_timer_event = UINT32_MAX;
    cpu->running = true;
    cpu->halted = false;
    cpu->exception = false;
    cpu->seed_entry_link = false;
    cpu->breakpoint_count = 0;
    ar_write(cpu, 1, 0x3FFB7000u);
    ar_write(cpu, (int)(callinc * 4u),
             (callinc << 30) | (BASE & 0x3FFFFFFFu));
    ar_write(cpu, (int)(callinc * 4u + 2u), mux);
    ar_write(cpu, (int)(callinc * 4u + 3u), timeout);

    uint32_t offset = core * 4u;
    mem_write32(cpu->mem, mux, owner);
    mem_write32(cpu->mem, mux + 4u, lock_count);
    mem_write32(cpu->mem, nesting_addr, 0x11111111u);
    mem_write32(cpu->mem, nesting_addr + 4u, 0x22222222u);
    mem_write32(cpu->mem, old_state_addr, 0x33333333u);
    mem_write32(cpu->mem, old_state_addr + 4u, 0x44444444u);
    mem_write32(cpu->mem, nesting_addr + offset, nesting);
    mem_write32(cpu->mem, old_state_addr + offset, old_state);
    mem_write32(cpu->mem, 0x3FFB7000u - 36u, 0xDEADBEEFu);
}

TEST(test_idf_port_enter_is_relocated_and_matches_original_routine) {
    const uint32_t addr = 0x4007D300u;
    const uint32_t mux = 0x3FFB4000u;
    const uint32_t nesting_addr = 0x3FFB4100u;
    const uint32_t old_state_addr = 0x3FFB4200u;
    static const struct {
        uint32_t owner;
        uint8_t lock_count;
        uint8_t nesting;
        uint8_t old_state;
        uint8_t callinc;
        uint8_t core;
        uint8_t insns;
    } cases[] = {
        { XTENSA_SPINLOCK_FREE, 0u, 0u, 2u, 2u, 0u, 41u },
        { XTENSA_SPINLOCK_OWNER_CORE0, 2u, 3u, 1u, 1u, 0u, 37u },
        { XTENSA_SPINLOCK_FREE, 1u, 0u, 3u, 3u, 1u, 41u },
        { XTENSA_SPINLOCK_OWNER_CORE1, 4u, 2u, 0u, 2u, 1u, 37u },
    };

    /* A complete body match is insufficient unless every decoded semantic
     * literal also describes the expected ESP-IDF primitive. */
    for (unsigned variant = 0u; variant < 5u; variant++) {
        xtensa_cpu_t invalid;
        setup(&invalid);
        esp32_rom_stubs_t *rom = rom_stubs_create(&invalid);
        seed_idf_port_enter(&invalid, addr, nesting_addr, old_state_addr);
        uint32_t literal = idf_port_enter_literal_base(addr);
        if (variant == 0u)
            mem_write8(invalid.mem, addr + 54u, 0u);
        else if (variant == 1u)
            mem_write32(invalid.mem, literal, 0u);
        else if (variant == 2u)
            mem_write32(invalid.mem, literal + 12u, 0u);
        else if (variant == 3u)
            mem_write32(invalid.mem, literal + 20u, 0x40001234u);
        else
            mem_write32(invalid.mem, literal + 24u, 0x3F800000u);
        ASSERT_EQ(rom_stubs_hook_firmware_addrs(
                          rom, 0x40081234u), 0u);
        rom_stubs_destroy(rom);
        teardown(&invalid);
    }

    for (unsigned c = 0u; c < sizeof(cases) / sizeof(cases[0]); c++) {
        xtensa_cpu_t reference;
        xtensa_cpu_t accelerated;
        setup(&reference);
        setup(&accelerated);
        seed_idf_port_enter(&reference, addr, nesting_addr, old_state_addr);
        seed_idf_port_enter(&accelerated, addr, nesting_addr, old_state_addr);
        esp32_rom_stubs_t *reference_rom = rom_stubs_create(&reference);
        esp32_rom_stubs_t *accelerated_rom = rom_stubs_create(&accelerated);
        ASSERT_EQ(rom_stubs_hook_firmware_addrs(
                          accelerated_rom, 0x40081234u), 1u);

        init_idf_port_enter_call(
                &reference, addr, mux, nesting_addr, old_state_addr,
                cases[c].owner, cases[c].lock_count, cases[c].nesting,
                cases[c].old_state, UINT32_MAX, cases[c].callinc,
                cases[c].core);
        init_idf_port_enter_call(
                &accelerated, addr, mux, nesting_addr, old_state_addr,
                cases[c].owner, cases[c].lock_count, cases[c].nesting,
                cases[c].old_state, UINT32_MAX, cases[c].callinc,
                cases[c].core);

        for (unsigned step = 0u; step < 100u && reference.pc != BASE; step++)
            ASSERT_EQ(xtensa_step(&reference), 0u);
        ASSERT_EQ(reference.pc, BASE);
        ASSERT_EQ64(reference.insn_count, cases[c].insns);
        ASSERT_EQ(xtensa_step(&accelerated), 0u);

        ASSERT_EQ(accelerated.pc, reference.pc);
        ASSERT_EQ(accelerated.ccount, reference.ccount);
        ASSERT_EQ64(accelerated.cycle_count, reference.cycle_count);
        ASSERT_EQ64(accelerated.insn_count, reference.insn_count);
        ASSERT_EQ(accelerated.ps, reference.ps);
        ASSERT_EQ(accelerated.scompare1, reference.scompare1);
        ASSERT_EQ(accelerated.windowbase, reference.windowbase);
        ASSERT_EQ(accelerated.windowstart, reference.windowstart);
        ASSERT_EQ(accelerated._pc_written, reference._pc_written);
        ASSERT_EQ(accelerated.irq_check, reference.irq_check);
        for (unsigned i = 0u; i < 64u; i++)
            ASSERT_EQ(accelerated.ar[i], reference.ar[i]);
        for (unsigned i = 0u; i < 16u; i++)
            ASSERT_EQ(accelerated.window_callsize[i],
                      reference.window_callsize[i]);
        ASSERT_EQ(mem_read32(accelerated.mem, mux),
                  mem_read32(reference.mem, mux));
        ASSERT_EQ(mem_read32(accelerated.mem, mux + 4u),
                  mem_read32(reference.mem, mux + 4u));
        ASSERT_EQ(mem_read32(accelerated.mem, nesting_addr),
                  mem_read32(reference.mem, nesting_addr));
        ASSERT_EQ(mem_read32(accelerated.mem, nesting_addr + 4u),
                  mem_read32(reference.mem, nesting_addr + 4u));
        ASSERT_EQ(mem_read32(accelerated.mem, old_state_addr),
                  mem_read32(reference.mem, old_state_addr));
        ASSERT_EQ(mem_read32(accelerated.mem, old_state_addr + 4u),
                  mem_read32(reference.mem, old_state_addr + 4u));
        ASSERT_EQ(mem_read32(accelerated.mem, 0x3FFB7000u - 36u),
                  mem_read32(reference.mem, 0x3FFB7000u - 36u));

        rom_stubs_destroy(accelerated_rom);
        rom_stubs_destroy(reference_rom);
        teardown(&accelerated);
        teardown(&reference);
    }

    /* Contention, finite timeouts, pending interrupts, and a native span
     * shorter than the complete successful path all remain in guest code. */
    xtensa_cpu_t boundary;
    setup(&boundary);
    seed_idf_port_enter(&boundary, addr, nesting_addr, old_state_addr);
    esp32_rom_stubs_t *boundary_rom = rom_stubs_create(&boundary);
    ASSERT_EQ(rom_stubs_hook_firmware_addrs(
                      boundary_rom, 0x40081234u), 1u);

    init_idf_port_enter_call(
            &boundary, addr, mux, nesting_addr, old_state_addr,
            XTENSA_SPINLOCK_OWNER_CORE1, 0u, 0u, 0u, UINT32_MAX, 2u, 0u);
    ASSERT_EQ(xtensa_step(&boundary), 0u);
    ASSERT_EQ(boundary.pc, addr + 3u);

    init_idf_port_enter_call(
            &boundary, addr, mux, nesting_addr, old_state_addr,
            XTENSA_SPINLOCK_FREE, 0u, 0u, 0u, 10u, 2u, 0u);
    ASSERT_EQ(xtensa_step(&boundary), 0u);
    ASSERT_EQ(boundary.pc, addr + 3u);

    init_idf_port_enter_call(
            &boundary, addr, mux, nesting_addr, old_state_addr,
            XTENSA_SPINLOCK_FREE, 0u, 0u, 0u, UINT32_MAX, 2u, 0u);
    boundary.interrupt = 1u << 11;
    boundary.intenable = 1u << 11;
    ASSERT_EQ(xtensa_step(&boundary), 0u);
    ASSERT_EQ(boundary.pc, addr + 3u);

    init_idf_port_enter_call(
            &boundary, addr, mux, nesting_addr, old_state_addr,
            XTENSA_SPINLOCK_FREE, 0u, 0u, 0u, UINT32_MAX, 2u, 0u);
    ASSERT_EQ(xtensa_run(&boundary, 40), 40u);
    ASSERT_TRUE(boundary.pc != BASE);
    ASSERT_EQ(xtensa_run(&boundary, 1), 1u);
    ASSERT_EQ(boundary.pc, BASE);
    rom_stubs_destroy(boundary_rom);
    teardown(&boundary);
}

static const uint8_t idf_port_exit_code[] = {
    0x36, 0x41, 0x00, 0x90, 0x63, 0x00, 0x80, 0xEB,
    0x03, 0x88, 0x12, 0x0B, 0x88, 0x89, 0x12, 0x56,
    0x58, 0x00, 0x81, 0x09, 0xC8, 0x82, 0x62, 0x00,
    0x90, 0xE6, 0x13, 0x10, 0x20, 0x00, 0xA0, 0xEB,
    0x03, 0xA0, 0xAD, 0x04, 0xE0, 0xAA, 0x11, 0x81,
    0x05, 0xC8, 0xAA, 0x88, 0x98, 0x08, 0xA6, 0x19,
    0x16, 0x0B, 0x99, 0x99, 0x08, 0xCC, 0xF9, 0x21,
    0x02, 0xC8, 0xAA, 0xA2, 0xA2, 0x2A, 0x00, 0xC0,
    0x20, 0x00, 0x81, 0x0D, 0xC6, 0xE0, 0x08, 0x00,
    0x1D, 0xF0,
};

static uint32_t idf_port_exit_literal_base(uint32_t addr) {
    return (addr - 0x100u) & ~3u;
}

static void seed_idf_port_exit(xtensa_cpu_t *cpu, uint32_t addr,
                               uint32_t nesting_addr,
                               uint32_t old_state_addr) {
    uint32_t literal = idf_port_exit_literal_base(addr);
    put_test_bytes(cpu, addr, idf_port_exit_code,
                   sizeof(idf_port_exit_code));
    put_insn3(cpu, addr + 18u,
              encode_test_l32r(addr + 18u, literal, 8));
    put_insn3(cpu, addr + 39u,
              encode_test_l32r(addr + 39u, literal + 4u, 8));
    put_insn3(cpu, addr + 55u,
              encode_test_l32r(addr + 55u, literal + 8u, 2));
    put_insn3(cpu, addr + 66u,
              encode_test_l32r(addr + 66u, literal + 12u, 8));
    mem_write32(cpu->mem, literal, XTENSA_SPINLOCK_FREE);
    mem_write32(cpu->mem, literal + 4u, nesting_addr);
    mem_write32(cpu->mem, literal + 8u, old_state_addr);
    mem_write32(cpu->mem, literal + 12u, 0x4000BFDcu);
}

static void init_idf_port_exit_call(xtensa_cpu_t *cpu, uint32_t addr,
                                    uint32_t mux, uint32_t nesting_addr,
                                    uint32_t old_state_addr,
                                    uint32_t lock_count,
                                    uint32_t nesting, uint32_t old_state,
                                    unsigned callinc, unsigned core) {
    for (unsigned i = 0u; i < 64u; i++)
        cpu->ar[i] = 0xA5000000u + i;
    cpu->pc = addr;
    cpu->_pc_written = true;
    cpu->windowbase = 3u;
    cpu->windowstart = 1u << 3;
    cpu->ps = (1u << 18) | (9u << 8) | 1u;
    XT_PS_SET_CALLINC(cpu->ps, callinc);
    cpu->window_callsize[(3u + callinc) & 15u] = 0xBu;
    cpu->prid = core == 0u ? XTENSA_SPINLOCK_OWNER_CORE0
                           : XTENSA_SPINLOCK_OWNER_CORE1;
    cpu->core_id = (int)core;
    cpu->ccount = 1000u;
    cpu->cycle_count = 2000u;
    cpu->insn_count = 0u;
    cpu->next_timer_event = UINT32_MAX;
    cpu->running = true;
    cpu->halted = false;
    cpu->exception = false;
    cpu->seed_entry_link = false;
    cpu->breakpoint_count = 0;
    ar_write(cpu, 1, 0x3FFB7000u);
    ar_write(cpu, (int)(callinc * 4u),
             (callinc << 30) | (BASE & 0x3FFFFFFFu));
    ar_write(cpu, (int)(callinc * 4u + 2u), mux);

    uint32_t offset = core * 4u;
    mem_write32(cpu->mem, mux, cpu->prid);
    mem_write32(cpu->mem, mux + 4u, lock_count);
    mem_write32(cpu->mem, nesting_addr, 0x11111111u);
    mem_write32(cpu->mem, nesting_addr + 4u, 0x22222222u);
    mem_write32(cpu->mem, old_state_addr, 0x33333333u);
    mem_write32(cpu->mem, old_state_addr + 4u, 0x44444444u);
    mem_write32(cpu->mem, nesting_addr + offset, nesting);
    mem_write32(cpu->mem, old_state_addr + offset, old_state);
}

TEST(test_idf_port_exit_is_relocated_and_matches_original_routine) {
    const uint32_t addr = 0x4007D500u;
    const uint32_t mux = 0x3FFB4000u;
    const uint32_t nesting_addr = 0x3FFB4100u;
    const uint32_t old_state_addr = 0x3FFB4200u;
    static const struct {
        uint8_t lock_count;
        uint8_t nesting;
        uint8_t old_state;
        uint8_t callinc;
        uint8_t core;
        uint8_t insns;
    } cases[] = {
        { 1u, 1u, 0u, 2u, 0u, 29u }, /* final release and IRQ restore */
        { 2u, 3u, 2u, 1u, 0u, 20u }, /* recursive lock and critical */
        { 1u, 2u, 1u, 3u, 1u, 22u }, /* core 1, outer critical remains */
        { 1u, 0u, 3u, 2u, 1u, 19u }, /* defensive zero-nesting path */
    };

    /* Every instruction and every semantic literal participates in
     * authorization. A familiar entry address alone never installs code. */
    for (unsigned variant = 0u; variant < 4u; variant++) {
        xtensa_cpu_t invalid;
        setup(&invalid);
        esp32_rom_stubs_t *rom = rom_stubs_create(&invalid);
        seed_idf_port_exit(&invalid, addr, nesting_addr, old_state_addr);
        uint32_t literal = idf_port_exit_literal_base(addr);
        if (variant == 0u)
            mem_write8(invalid.mem, addr + 30u, 0u);
        else if (variant == 1u)
            mem_write32(invalid.mem, literal, 0u);
        else if (variant == 2u)
            mem_write32(invalid.mem, literal + 4u, 0x3F800000u);
        else
            mem_write32(invalid.mem, literal + 12u, 0x40001234u);
        ASSERT_EQ(rom_stubs_hook_firmware_addrs(
                          rom, 0x40081234u), 0u);
        rom_stubs_destroy(rom);
        teardown(&invalid);
    }

    for (unsigned c = 0u; c < sizeof(cases) / sizeof(cases[0]); c++) {
        xtensa_cpu_t reference;
        xtensa_cpu_t accelerated;
        setup(&reference);
        setup(&accelerated);
        seed_idf_port_exit(&reference, addr, nesting_addr, old_state_addr);
        seed_idf_port_exit(&accelerated, addr, nesting_addr, old_state_addr);
        esp32_rom_stubs_t *reference_rom = rom_stubs_create(&reference);
        esp32_rom_stubs_t *accelerated_rom = rom_stubs_create(&accelerated);
        ASSERT_EQ(rom_stubs_hook_firmware_addrs(
                          accelerated_rom, 0x40081234u), 1u);

        init_idf_port_exit_call(
                &reference, addr, mux, nesting_addr, old_state_addr,
                cases[c].lock_count, cases[c].nesting, cases[c].old_state,
                cases[c].callinc, cases[c].core);
        init_idf_port_exit_call(
                &accelerated, addr, mux, nesting_addr, old_state_addr,
                cases[c].lock_count, cases[c].nesting, cases[c].old_state,
                cases[c].callinc, cases[c].core);

        for (unsigned step = 0u; step < 100u && reference.pc != BASE; step++)
            ASSERT_EQ(xtensa_step(&reference), 0u);
        ASSERT_EQ(reference.pc, BASE);
        ASSERT_EQ64(reference.insn_count, cases[c].insns);
        ASSERT_EQ(xtensa_step(&accelerated), 0u);

        ASSERT_EQ(accelerated.pc, reference.pc);
        ASSERT_EQ(accelerated.ccount, reference.ccount);
        ASSERT_EQ64(accelerated.cycle_count, reference.cycle_count);
        ASSERT_EQ64(accelerated.insn_count, reference.insn_count);
        ASSERT_EQ(accelerated.ps, reference.ps);
        ASSERT_EQ(accelerated.windowbase, reference.windowbase);
        ASSERT_EQ(accelerated.windowstart, reference.windowstart);
        ASSERT_EQ(accelerated._pc_written, reference._pc_written);
        ASSERT_EQ(accelerated.irq_check, reference.irq_check);
        for (unsigned i = 0u; i < 64u; i++)
            ASSERT_EQ(accelerated.ar[i], reference.ar[i]);
        for (unsigned i = 0u; i < 16u; i++)
            ASSERT_EQ(accelerated.window_callsize[i],
                      reference.window_callsize[i]);
        ASSERT_EQ(mem_read32(accelerated.mem, mux),
                  mem_read32(reference.mem, mux));
        ASSERT_EQ(mem_read32(accelerated.mem, mux + 4u),
                  mem_read32(reference.mem, mux + 4u));
        ASSERT_EQ(mem_read32(accelerated.mem, nesting_addr),
                  mem_read32(reference.mem, nesting_addr));
        ASSERT_EQ(mem_read32(accelerated.mem, nesting_addr + 4u),
                  mem_read32(reference.mem, nesting_addr + 4u));
        ASSERT_EQ(mem_read32(accelerated.mem, old_state_addr),
                  mem_read32(reference.mem, old_state_addr));
        ASSERT_EQ(mem_read32(accelerated.mem, old_state_addr + 4u),
                  mem_read32(reference.mem, old_state_addr + 4u));

        rom_stubs_destroy(accelerated_rom);
        rom_stubs_destroy(reference_rom);
        teardown(&accelerated);
        teardown(&reference);
    }

    /* A pending interrupt and an exact timer/scheduler boundary each expose
     * an internal instruction PC, so all three invocations stay interpreted. */
    xtensa_cpu_t boundary;
    setup(&boundary);
    seed_idf_port_exit(&boundary, addr, nesting_addr, old_state_addr);
    esp32_rom_stubs_t *boundary_rom = rom_stubs_create(&boundary);
    ASSERT_EQ(rom_stubs_hook_firmware_addrs(
                      boundary_rom, 0x40081234u), 1u);
    init_idf_port_exit_call(&boundary, addr, mux, nesting_addr,
                            old_state_addr, 1u, 1u, 0u, 2u, 0u);
    boundary.interrupt = 1u << 11;
    boundary.intenable = 1u << 11;
    boundary.irq_check = false;
    ASSERT_EQ(xtensa_step(&boundary), 0u);
    ASSERT_EQ(boundary.pc, addr + 3u);
    ASSERT_EQ(mem_read32(boundary.mem, mux + 4u), 1u);

    init_idf_port_exit_call(&boundary, addr, mux, nesting_addr,
                            old_state_addr, 1u, 1u, 0u, 2u, 0u);
    boundary.interrupt = 0u;
    boundary.intenable = 0u;
    boundary.next_timer_event = boundary.ccount + 29u;
    ASSERT_EQ(xtensa_step(&boundary), 0u);
    ASSERT_EQ(boundary.pc, addr + 3u);
    ASSERT_EQ(mem_read32(boundary.mem, mux + 4u), 1u);

    init_idf_port_exit_call(&boundary, addr, mux, nesting_addr,
                            old_state_addr, 1u, 1u, 0u, 2u, 0u);
    ASSERT_EQ(xtensa_run(&boundary, 28), 28u);
    ASSERT_TRUE(boundary.pc != BASE);
    ASSERT_EQ(xtensa_run(&boundary, 1), 1u);
    ASSERT_EQ(boundary.pc, BASE);
    rom_stubs_destroy(boundary_rom);
    teardown(&boundary);
}

static void seed_idf_heap_lock_wrappers(xtensa_cpu_t *cpu,
                                        uint32_t lock_addr,
                                        uint32_t unlock_addr,
                                        uint32_t enter_addr,
                                        uint32_t exit_addr) {
    static const uint8_t lock_code[] = {
        0x36, 0x41, 0x00,       /* entry a1, 32 */
        0xA8, 0x02,             /* l32i.n a10, a2, 0 */
        0x8C, 0x3A,             /* beqz.n a10, return */
        0x7C, 0xFB,             /* movi.n a11, -1 */
        0x00, 0x00, 0x00,       /* relocated call8 */
        0x1D, 0xF0,             /* retw.n */
    };
    static const uint8_t unlock_code[] = {
        0x36, 0x41, 0x00,       /* entry a1, 32 */
        0xA8, 0x02,             /* l32i.n a10, a2, 0 */
        0x8C, 0x1A,             /* beqz.n a10, return */
        0x00, 0x00, 0x00,       /* relocated call8 */
        0x1D, 0xF0,             /* retw.n */
    };
    put_test_bytes(cpu, lock_addr, lock_code, sizeof(lock_code));
    put_test_bytes(cpu, unlock_addr, unlock_code, sizeof(unlock_code));
    put_insn3(cpu, lock_addr + 9u,
              encode_test_calln(lock_addr + 9u, enter_addr, 2u));
    put_insn3(cpu, unlock_addr + 7u,
              encode_test_calln(unlock_addr + 7u, exit_addr, 2u));
}

static void init_idf_heap_lock_call(
        xtensa_cpu_t *cpu, uint32_t wrapper_addr, uint32_t heap,
        uint32_t mux, uint32_t nesting_addr, uint32_t old_state_addr,
        uint32_t owner, uint32_t lock_count, uint32_t nesting,
        uint32_t old_state, unsigned callinc, unsigned core) {
    for (unsigned i = 0u; i < 64u; i++)
        cpu->ar[i] = 0xC3000000u + i;
    cpu->pc = wrapper_addr;
    cpu->_pc_written = true;
    cpu->windowbase = 3u;
    cpu->windowstart = 1u << 3;
    cpu->ps = (1u << 18) | (9u << 8) | 1u;
    XT_PS_SET_CALLINC(cpu->ps, callinc);
    cpu->window_callsize[(3u + callinc) & 15u] = 0xBu;
    cpu->prid = core == 0u ? XTENSA_SPINLOCK_OWNER_CORE0
                           : XTENSA_SPINLOCK_OWNER_CORE1;
    cpu->core_id = (int)core;
    cpu->ccount = 1000u;
    cpu->cycle_count = 2000u;
    cpu->insn_count = 0u;
    cpu->next_timer_event = UINT32_MAX;
    cpu->running = true;
    cpu->halted = false;
    cpu->exception = false;
    cpu->seed_entry_link = false;
    cpu->breakpoint_count = 0;
    cpu->irq_check = false;
    ar_write(cpu, 1, 0x3FFB7000u);
    ar_write(cpu, (int)(callinc * 4u),
             (callinc << 30) | (BASE & 0x3FFFFFFFu));
    ar_write(cpu, (int)(callinc * 4u + 2u), heap);

    uint32_t core_offset = core * 4u;
    mem_write32(cpu->mem, heap, mux);
    mem_write32(cpu->mem, mux, owner);
    mem_write32(cpu->mem, mux + 4u, lock_count);
    mem_write32(cpu->mem, nesting_addr, 0x11111111u);
    mem_write32(cpu->mem, nesting_addr + 4u, 0x22222222u);
    mem_write32(cpu->mem, old_state_addr, 0x33333333u);
    mem_write32(cpu->mem, old_state_addr + 4u, 0x44444444u);
    mem_write32(cpu->mem, nesting_addr + core_offset, nesting);
    mem_write32(cpu->mem, old_state_addr + core_offset, old_state);
    mem_write32(cpu->mem, 0x3FFB7000u - 68u, 0xDEADBEEFu);
}

TEST(test_idf_heap_lock_wrappers_compose_verified_critical_sections) {
    const uint32_t enter_addr = 0x4007D300u;
    const uint32_t exit_addr = 0x4007D500u;
    const uint32_t lock_addr = 0x4007D700u;
    const uint32_t unlock_addr = 0x4007D720u;
    const uint32_t heap = 0x3FFB3F00u;
    const uint32_t mux = 0x3FFB4000u;
    const uint32_t nesting_addr = 0x3FFB4100u;
    const uint32_t old_state_addr = 0x3FFB4200u;
    static const struct {
        bool enter;
        uint32_t owner;
        uint8_t lock_count;
        uint8_t nesting;
        uint8_t old_state;
        uint8_t callinc;
        uint8_t core;
        uint8_t insns;
    } cases[] = {
        { true, XTENSA_SPINLOCK_FREE, 0u, 0u, 2u, 1u, 0u, 47u },
        { true, XTENSA_SPINLOCK_OWNER_CORE1, 3u, 2u, 1u, 3u, 1u, 43u },
        { false, XTENSA_SPINLOCK_OWNER_CORE0, 1u, 1u, 0u, 2u, 0u, 34u },
        { false, XTENSA_SPINLOCK_OWNER_CORE1, 2u, 3u, 2u, 1u, 1u, 25u },
    };

    /* Neither a lookalike wrapper nor a valid wrapper calling an unverified
     * body may inherit native critical-section behavior. */
    {
        xtensa_cpu_t invalid;
        setup(&invalid);
        seed_idf_port_enter(&invalid, enter_addr, nesting_addr,
                            old_state_addr);
        seed_idf_port_exit(&invalid, exit_addr, nesting_addr,
                           old_state_addr);
        seed_idf_heap_lock_wrappers(&invalid, lock_addr, unlock_addr,
                                    enter_addr, exit_addr);
        mem_write8(invalid.mem, lock_addr + 5u, 0u);
        put_insn3(&invalid, unlock_addr + 7u,
                  encode_test_calln(unlock_addr + 7u,
                                    0x4007D900u, 2u));
        esp32_rom_stubs_t *invalid_rom = rom_stubs_create(&invalid);
        ASSERT_EQ(rom_stubs_hook_firmware_addrs(
                          invalid_rom, 0x40081234u), 2u);
        rom_stubs_destroy(invalid_rom);
        teardown(&invalid);
    }

    for (unsigned c = 0u; c < sizeof(cases) / sizeof(cases[0]); c++) {
        xtensa_cpu_t reference;
        xtensa_cpu_t accelerated;
        setup(&reference);
        setup(&accelerated);
        seed_idf_port_enter(&reference, enter_addr, nesting_addr,
                            old_state_addr);
        seed_idf_port_exit(&reference, exit_addr, nesting_addr,
                           old_state_addr);
        seed_idf_heap_lock_wrappers(&reference, lock_addr, unlock_addr,
                                    enter_addr, exit_addr);
        seed_idf_port_enter(&accelerated, enter_addr, nesting_addr,
                            old_state_addr);
        seed_idf_port_exit(&accelerated, exit_addr, nesting_addr,
                           old_state_addr);
        seed_idf_heap_lock_wrappers(&accelerated, lock_addr, unlock_addr,
                                    enter_addr, exit_addr);
        esp32_rom_stubs_t *reference_rom = rom_stubs_create(&reference);
        esp32_rom_stubs_t *accelerated_rom = rom_stubs_create(&accelerated);
        ASSERT_EQ(rom_stubs_hook_firmware_addrs(
                          accelerated_rom, 0x40081234u), 4u);

        uint32_t wrapper = cases[c].enter ? lock_addr : unlock_addr;
        init_idf_heap_lock_call(
                &reference, wrapper, heap, mux, nesting_addr, old_state_addr,
                cases[c].owner, cases[c].lock_count, cases[c].nesting,
                cases[c].old_state, cases[c].callinc, cases[c].core);
        init_idf_heap_lock_call(
                &accelerated, wrapper, heap, mux, nesting_addr, old_state_addr,
                cases[c].owner, cases[c].lock_count, cases[c].nesting,
                cases[c].old_state, cases[c].callinc, cases[c].core);

        for (unsigned step = 0u; step < 100u && reference.pc != BASE; step++)
            ASSERT_EQ(xtensa_step(&reference), 0u);
        ASSERT_EQ(reference.pc, BASE);
        ASSERT_EQ64(reference.insn_count, cases[c].insns);
        ASSERT_EQ(xtensa_step(&accelerated), 0u);

        ASSERT_EQ(accelerated.pc, reference.pc);
        ASSERT_EQ(accelerated.ccount, reference.ccount);
        ASSERT_EQ64(accelerated.cycle_count, reference.cycle_count);
        ASSERT_EQ64(accelerated.insn_count, reference.insn_count);
        ASSERT_EQ(accelerated.ps, reference.ps);
        ASSERT_EQ(accelerated.scompare1, reference.scompare1);
        ASSERT_EQ(accelerated.windowbase, reference.windowbase);
        ASSERT_EQ(accelerated.windowstart, reference.windowstart);
        ASSERT_EQ(accelerated._pc_written, reference._pc_written);
        ASSERT_EQ(accelerated.irq_check, reference.irq_check);
        for (unsigned i = 0u; i < 64u; i++)
            ASSERT_EQ(accelerated.ar[i], reference.ar[i]);
        for (unsigned i = 0u; i < 16u; i++)
            ASSERT_EQ(accelerated.window_callsize[i],
                      reference.window_callsize[i]);
        ASSERT_EQ(mem_read32(accelerated.mem, mux),
                  mem_read32(reference.mem, mux));
        ASSERT_EQ(mem_read32(accelerated.mem, mux + 4u),
                  mem_read32(reference.mem, mux + 4u));
        ASSERT_EQ(mem_read32(accelerated.mem, nesting_addr),
                  mem_read32(reference.mem, nesting_addr));
        ASSERT_EQ(mem_read32(accelerated.mem, nesting_addr + 4u),
                  mem_read32(reference.mem, nesting_addr + 4u));
        ASSERT_EQ(mem_read32(accelerated.mem, old_state_addr),
                  mem_read32(reference.mem, old_state_addr));
        ASSERT_EQ(mem_read32(accelerated.mem, old_state_addr + 4u),
                  mem_read32(reference.mem, old_state_addr + 4u));
        ASSERT_EQ(mem_read32(accelerated.mem, 0x3FFB7000u - 68u),
                  mem_read32(reference.mem, 0x3FFB7000u - 68u));

        rom_stubs_destroy(accelerated_rom);
        rom_stubs_destroy(reference_rom);
        teardown(&accelerated);
        teardown(&reference);
    }

    /* A null heap mux follows the wrapper's real four-instruction early
     * return. A contended lock and a scheduler boundary both decline before
     * changing any architectural state and continue in guest code. */
    xtensa_cpu_t boundary;
    setup(&boundary);
    seed_idf_port_enter(&boundary, enter_addr, nesting_addr, old_state_addr);
    seed_idf_port_exit(&boundary, exit_addr, nesting_addr, old_state_addr);
    seed_idf_heap_lock_wrappers(&boundary, lock_addr, unlock_addr,
                                enter_addr, exit_addr);
    esp32_rom_stubs_t *boundary_rom = rom_stubs_create(&boundary);
    ASSERT_EQ(rom_stubs_hook_firmware_addrs(
                      boundary_rom, 0x40081234u), 4u);

    init_idf_heap_lock_call(
            &boundary, lock_addr, heap, mux, nesting_addr, old_state_addr,
            XTENSA_SPINLOCK_FREE, 0u, 0u, 0u, 2u, 0u);
    mem_write32(boundary.mem, heap, 0u);
    ASSERT_EQ(xtensa_step(&boundary), 0u);
    ASSERT_EQ(boundary.pc, BASE);
    ASSERT_EQ64(boundary.insn_count, 4u);

    init_idf_heap_lock_call(
            &boundary, lock_addr, heap, mux, nesting_addr, old_state_addr,
            XTENSA_SPINLOCK_OWNER_CORE1, 1u, 0u, 0u, 2u, 0u);
    ASSERT_EQ(xtensa_step(&boundary), 0u);
    ASSERT_EQ(boundary.pc, lock_addr + 3u);
    ASSERT_EQ(mem_read32(boundary.mem, mux),
              XTENSA_SPINLOCK_OWNER_CORE1);

    init_idf_heap_lock_call(
            &boundary, lock_addr, heap, mux, nesting_addr, old_state_addr,
            XTENSA_SPINLOCK_FREE, 0u, 0u, 0u, 2u, 0u);
    ASSERT_EQ(xtensa_run(&boundary, 46), 46u);
    ASSERT_TRUE(boundary.pc != BASE);
    ASSERT_EQ(xtensa_run(&boundary, 1), 1u);
    ASSERT_EQ(boundary.pc, BASE);

    rom_stubs_destroy(boundary_rom);
    teardown(&boundary);
}

TEST(test_openhasp_lanbon_requires_complete_fingerprint) {
    xtensa_cpu_t cpu;
    setup(&cpu);
    esp32_rom_stubs_t *rom = rom_stubs_create(&cpu);

    ASSERT_EQ(rom_stubs_identify_firmware(rom, 0x40086E2Cu),
              ROM_FIRMWARE_UNKNOWN);
    seed_openhasp_v070rc13_profile(&cpu);
    ASSERT_EQ(rom_stubs_identify_firmware(rom, 0x40086E2Cu),
              ROM_FIRMWARE_OPENHASP_V070RC13_LANBON_L8);

    /* A reused entry point cannot authorize fixed lwIP addresses by itself. */
    mem_write8(cpu.mem, 0x4015C758u, 0u);
    ASSERT_EQ(rom_stubs_identify_firmware(rom, 0x40086E2Cu),
              ROM_FIRMWARE_UNKNOWN);

    rom_stubs_destroy(rom);
    teardown(&cpu);
}

TEST(test_tasmota32_requires_complete_fingerprint) {
    xtensa_cpu_t cpu;
    setup(&cpu);
    esp32_rom_stubs_t *rom = rom_stubs_create(&cpu);

    ASSERT_EQ(rom_stubs_identify_firmware(rom, 0x40082A58u),
              ROM_FIRMWARE_UNKNOWN);
    seed_tasmota32_v1560_profile(&cpu);
    ASSERT_EQ(rom_stubs_identify_firmware(rom, 0x40082A58u),
              ROM_FIRMWARE_TASMOTA32_V1560);
    seed_flash_poll_loop(&cpu, 0x400826D5u);
    seed_flash_poll_loop(&cpu, 0x40082740u);
    ASSERT_EQ(rom_stubs_hook_firmware_addrs(rom, 0x40082A58u), 0);
    ASSERT_EQ(cpu.poll_spin_count, 2u);
    ASSERT_EQ(cpu.poll_spin_pc[0], 0x400826D5u);
    ASSERT_EQ(cpu.poll_spin_pc[1], 0x40082740u);

    /* The entry point alone cannot authorize fixed production addresses. */
    mem_write8(cpu.mem, 0x4019BA54u, 0u);
    ASSERT_EQ(rom_stubs_identify_firmware(rom, 0x40082A58u),
              ROM_FIRMWARE_UNKNOWN);

    rom_stubs_destroy(rom);
    teardown(&cpu);
}

TEST(test_marauder_same_entry_uses_instruction_fingerprint) {
    xtensa_cpu_t cpu;
    setup(&cpu);
    esp32_rom_stubs_t *rom = rom_stubs_create(&cpu);

    ASSERT_EQ(rom_stubs_identify_firmware(rom, 0x400831D8u),
              ROM_FIRMWARE_UNKNOWN);
    seed_marauder_v11401_profile(&cpu);
    ASSERT_EQ(rom_stubs_identify_firmware(rom, 0x400831D8u),
              ROM_FIRMWARE_MARAUDER_V1140_1);

    /* Invalidate one legacy anchor before planting the newer layout. */
    mem_write8(cpu.mem, 0x401BDE2Cu, 0u);
    seed_marauder_v11423_profile(&cpu);
    ASSERT_EQ(rom_stubs_identify_firmware(rom, 0x400831D8u),
              ROM_FIRMWARE_MARAUDER_V1142_3);

    /* The third layout shares the entry point again and must be told apart
     * from both older ones by its own anchors. */
    mem_write8(cpu.mem, 0x401BE628u, 0u);
    seed_marauder_v1151_profile(&cpu);
    ASSERT_EQ(rom_stubs_identify_firmware(rom, 0x400831D8u),
              ROM_FIRMWARE_MARAUDER_V1151);

    /* A reused entry with a near miss must not receive address hooks. */
    mem_write8(cpu.mem, 0x401990A0u, 0u);
    mem_write8(cpu.mem, 0x4019BE94u, 0u);
    ASSERT_EQ(rom_stubs_identify_firmware(rom, 0x400831D8u),
              ROM_FIRMWARE_UNKNOWN);
    ASSERT_EQ(rom_stubs_hook_firmware_addrs(rom, 0x400831D8u), 0);

    rom_stubs_destroy(rom);
    teardown(&cpu);
}

TEST(test_marauder_v1121_cyd2usb_uses_independent_fingerprint) {
    xtensa_cpu_t cpu;
    setup(&cpu);
    esp32_rom_stubs_t *rom = rom_stubs_create(&cpu);

    ASSERT_EQ(rom_stubs_identify_firmware(rom, 0x40081E90u),
              ROM_FIRMWARE_UNKNOWN);
    seed_marauder_v1121_cyd2usb_profile(&cpu);
    ASSERT_EQ(rom_stubs_identify_firmware(rom, 0x40081E90u),
              ROM_FIRMWARE_MARAUDER_V1121_CYD2USB);

    /* The entry point is not sufficient evidence: one damaged anchor must
     * reject the profile and all of its fixed-address hooks. */
    mem_write8(cpu.mem, 0x401A861Cu, 0u);
    ASSERT_EQ(rom_stubs_identify_firmware(rom, 0x40081E90u),
              ROM_FIRMWARE_UNKNOWN);
    mem_write8(cpu.mem, 0x401C374Cu, 0u);
    ASSERT_EQ(rom_stubs_hook_firmware_addrs(rom, 0x40081E90u), 0);

    rom_stubs_destroy(rom);
    teardown(&cpu);
}

TEST(test_marauder_v1121_virtualizes_only_its_phy_and_sync_state) {
    xtensa_cpu_t cpu;
    setup(&cpu);
    esp32_rom_stubs_t *rom = rom_stubs_create(&cpu);
    seed_marauder_v1121_cyd2usb_profile(&cpu);

    const uint32_t phy_global = 0x3FFD003Cu;
    const uint32_t old_table_global = 0x3FFCD974u;
    const uint32_t old_shifted_table_global = 0x3FFCD984u;
    seed_flash_poll_loop(&cpu, 0x40081A5Du);
    seed_flash_poll_loop(&cpu, 0x40081B0Du);
    mem_write32(cpu.mem, 0x401C3748u, 0x40004100u);
    mem_write32(cpu.mem, 0x401C18C0u, phy_global);
    mem_write32(cpu.mem, old_table_global, 0xA5A5A5A5u);
    mem_write32(cpu.mem, old_shifted_table_global, 0x5A5A5A5Au);

    ASSERT_EQ(rom_stubs_hook_firmware_addrs(rom, 0x40081E90u), 5);
    ASSERT_EQ(cpu.poll_spin_count, 2u);
    ASSERT_EQ(cpu.poll_spin_pc[0], 0x40081A5Du);
    ASSERT_EQ(cpu.poll_spin_pc[1], 0x40081B0Du);
    cpu.pc = 0x401C374Cu;
    XT_PS_SET_CALLINC(cpu.ps, 0);
    ar_write(&cpu, 0, BASE);
    xtensa_step(&cpu);
    ASSERT_EQ(cpu.pc, BASE);
    ASSERT_EQ(mem_read32(cpu.mem, phy_global), 0x50001900u);
    ASSERT_EQ(mem_read32(cpu.mem, old_table_global), 0xA5A5A5A5u);
    ASSERT_EQ(mem_read32(cpu.mem, old_shifted_table_global), 0x5A5A5A5Au);

    /* The IDF 5.5 wait loop loads m_synced through a7.  A deliberately wrong
     * a2 catches accidental reuse of the older profile's register-based spy. */
    put_insn3(&cpu, 0x4010EF53u, 0x000722u); /* l8ui a2, a7, 0 */
    mem_write8(cpu.mem, 0x3FFC9060u, 0);
    ar_write(&cpu, 2, 0x3FFB1000u);
    ar_write(&cpu, 7, 0x3FFC9060u);
    cpu.pc = 0x4010EF53u;
    cpu._pc_written = true;
    xtensa_step(&cpu);
    ASSERT_EQ(mem_read8(cpu.mem, 0x3FFC9060u), 1);
    ASSERT_EQ(ar_read(&cpu, 2), 1);
    ASSERT_EQ(cpu.pc, 0x4010EF56u);

    rom_stubs_destroy(rom);
    teardown(&cpu);
}

TEST(test_marauder_v1151_hooks_use_third_layout) {
    xtensa_cpu_t cpu;
    setup(&cpu);
    esp32_rom_stubs_t *rom = rom_stubs_create(&cpu);
    seed_marauder_v1151_profile(&cpu);

    /* Neither older layout's BLE dispatch global may be touched. */
    const uint32_t v11401_table_global = 0x3FFCD974u;
    const uint32_t v11423_table_global = 0x3FFCD984u;
    const uint32_t v1151_table_global = 0x3FFCDB0Cu;
    mem_write32(cpu.mem, v11401_table_global, 0xA5A5A5A5u);
    mem_write32(cpu.mem, v11423_table_global, 0x5A5A5A5Au);
    ASSERT_EQ(rom_stubs_hook_firmware_addrs(rom, 0x400831D8u), 5);

    cpu.pc = 0x401C1438u;
    XT_PS_SET_CALLINC(cpu.ps, 0);
    ar_write(&cpu, 0, BASE);
    xtensa_step(&cpu);
    ASSERT_EQ(cpu.pc, BASE);
    ASSERT_EQ(mem_read32(cpu.mem, v11401_table_global), 0xA5A5A5A5u);
    ASSERT_EQ(mem_read32(cpu.mem, v11423_table_global), 0x5A5A5A5Au);
    ASSERT_EQ(mem_read32(cpu.mem, v1151_table_global), 0x50000400u);
    ASSERT_EQ(mem_read32(cpu.mem, 0x3FFD06DCu), 0x50000800u);
    ASSERT_EQ(mem_read32(cpu.mem, 0x3FFD06E0u), 0x50000800u);
    ASSERT_EQ(mem_read32(cpu.mem, 0x50000400u), 0x401DF918u);
    ASSERT_EQ(mem_read32(cpu.mem, 0x50000800u), 0x401DF918u);

    rom_stubs_destroy(rom);
    teardown(&cpu);
}

TEST(test_marauder_v11423_hooks_use_shifted_phy_and_data_layout) {
    xtensa_cpu_t cpu;
    setup(&cpu);
    esp32_rom_stubs_t *rom = rom_stubs_create(&cpu);
    seed_marauder_v11423_profile(&cpu);

    const uint32_t old_table_global = 0x3FFCD974u;
    const uint32_t phy_global = 0x3FFCD9ACu;
    const uint32_t new_table_global = 0x3FFCD984u;
    mem_write32(cpu.mem, 0x401BE624u, 0x40004100u);
    mem_write32(cpu.mem, 0x401BA944u, phy_global);
    mem_write32(cpu.mem, old_table_global, 0xA5A5A5A5u);
    ASSERT_EQ(rom_stubs_hook_firmware_addrs(rom, 0x400831D8u), 5);

    cpu.pc = 0x401BE628u;
    XT_PS_SET_CALLINC(cpu.ps, 0);
    ar_write(&cpu, 0, BASE);
    xtensa_step(&cpu);
    ASSERT_EQ(cpu.pc, BASE);
    ASSERT_EQ(mem_read32(cpu.mem, phy_global), 0x50001900u);
    ASSERT_EQ(mem_read32(cpu.mem, old_table_global), 0xA5A5A5A5u);
    ASSERT_EQ(mem_read32(cpu.mem, new_table_global), 0x50000400u);
    ASSERT_EQ(mem_read32(cpu.mem, 0x3FFD0554u), 0x50000800u);
    ASSERT_EQ(mem_read32(cpu.mem, 0x3FFD0558u), 0x50000800u);
    ASSERT_EQ(mem_read32(cpu.mem, 0x50000400u), 0x401DC890u);
    ASSERT_EQ(mem_read32(cpu.mem, 0x50000800u), 0x401DC890u);

    rom_stubs_destroy(rom);
    teardown(&cpu);
}

TEST(test_marauder_nimble_deinit_preserves_cpp_lists) {
    xtensa_cpu_t cpu;
    setup(&cpu);
    esp32_rom_stubs_t *rom = rom_stubs_create(&cpu);
    seed_marauder_v11401_profile(&cpu);

    const uint32_t ignore_list = 0x3FFC9534u;
    const uint32_t connected_list = 0x3FFC9540u;
    const uint32_t synced = 0x3FFC9550u;
    const uint32_t initialized = 0x3FFC955Cu;
    mem_write32(cpu.mem, ignore_list, ignore_list);
    mem_write32(cpu.mem, ignore_list + 4u, ignore_list);
    mem_write32(cpu.mem, ignore_list + 8u, 0);
    mem_write32(cpu.mem, connected_list, connected_list);
    mem_write32(cpu.mem, connected_list + 4u, connected_list);
    mem_write32(cpu.mem, connected_list + 8u, 0);
    mem_write8(cpu.mem, synced, 1);
    mem_write8(cpu.mem, initialized, 1);

    ASSERT_EQ(rom_stubs_hook_firmware_addrs(rom, 0x400831D8u), 5);
    cpu.pc = 0x4010463Cu;
    XT_PS_SET_CALLINC(cpu.ps, 0);
    ar_write(&cpu, 0, BASE);
    ar_write(&cpu, 2, 1);
    xtensa_step(&cpu);

    ASSERT_EQ(cpu.pc, BASE);
    ASSERT_EQ(ar_read(&cpu, 2), 0);
    ASSERT_EQ(mem_read8(cpu.mem, synced), 1);
    ASSERT_EQ(mem_read8(cpu.mem, initialized), 1);
    ASSERT_EQ(mem_read32(cpu.mem, ignore_list), ignore_list);
    ASSERT_EQ(mem_read32(cpu.mem, ignore_list + 4u), ignore_list);
    ASSERT_EQ(mem_read32(cpu.mem, ignore_list + 8u), 0);
    ASSERT_EQ(mem_read32(cpu.mem, connected_list), connected_list);
    ASSERT_EQ(mem_read32(cpu.mem, connected_list + 4u), connected_list);
    ASSERT_EQ(mem_read32(cpu.mem, connected_list + 8u), 0);

    rom_stubs_destroy(rom);
    teardown(&cpu);
}

TEST(test_marauder_dport_cache_stall_is_coherent_noop) {
    xtensa_cpu_t cpu;
    setup(&cpu);
    esp32_rom_stubs_t *rom = rom_stubs_create(&cpu);
    seed_marauder_v11401_profile(&cpu);

    ASSERT_EQ(rom_stubs_hook_firmware_addrs(rom, 0x400831D8u), 5);
    static const uint32_t hook_pc[] = { 0x400816FCu, 0x40081760u };
    for (size_t i = 0; i < sizeof(hook_pc) / sizeof(hook_pc[0]); i++) {
        cpu.pc = hook_pc[i];
        cpu._pc_written = true;
        XT_PS_SET_CALLINC(cpu.ps, 0);
        ar_write(&cpu, 0, BASE);
        ar_write(&cpu, 2, 0xA5A5A5A5u);
        xtensa_step(&cpu);
        ASSERT_EQ(cpu.pc, BASE);
        ASSERT_EQ(ar_read(&cpu, 2), 0xA5A5A5A5u);
    }

    rom_stubs_destroy(rom);
    teardown(&cpu);
}

TEST(test_bt_rom_table_accessors_use_bounded_scratch) {
    xtensa_cpu_t cpu;
    setup(&cpu);
    esp32_rom_stubs_t *rom = rom_stubs_create(&cpu);

    uint32_t rf = call_builtin_rom0(&cpu, 0x40054298u, 0);
    uint32_t ip = call_builtin_rom0(&cpu, 0x40019AF0u, 0);
    uint32_t modules = call_builtin_rom0(&cpu, 0x4005427Cu, 0);
    uint32_t options = call_builtin_rom0(&cpu, 0x40010004u, 0);
    uint32_t phy = call_builtin_rom0(&cpu, 0x40004100u, 0);
    ASSERT_TRUE(rf >= 0x50000000u && rf < 0x50002000u);
    ASSERT_TRUE(ip >= 0x50000000u && ip < 0x50002000u);
    ASSERT_TRUE(modules >= 0x50000000u && modules + 0x19Cu < 0x50002000u);
    ASSERT_TRUE(options >= 0x50000000u && options + 0x1Cu < 0x50002000u);
    ASSERT_TRUE(phy >= 0x50000000u && phy + 0x1A4u < 0x50002000u);
    ASSERT_EQ(mem_read32(cpu.mem, phy + 0x000u), 0x40002F6Cu);
    ASSERT_EQ(mem_read32(cpu.mem, phy + 0x0A8u), 0x400041FCu);
    ASSERT_EQ(mem_read32(cpu.mem, phy + 0x120u), 0);
    ASSERT_EQ(mem_read32(cpu.mem, phy + 0x1A4u), 0x4000662Cu);

    /* Pointer targets are writable and independent. */
    mem_write32(cpu.mem, rf, 0x11111111u);
    mem_write32(cpu.mem, modules + 0x19Cu, 0x22222222u);
    mem_write32(cpu.mem, phy + 0x1A4u, 0x33333333u);
    ASSERT_EQ(mem_read32(cpu.mem, rf), 0x11111111u);
    ASSERT_EQ(mem_read32(cpu.mem, modules + 0x19Cu), 0x22222222u);
    ASSERT_EQ(mem_read32(cpu.mem, phy + 0x1A4u), 0x33333333u);

    /* The ROM accessor returns the persistent table; it must not erase the
     * function pointers which libphy patched after its first call. */
    ASSERT_EQ(call_builtin_rom0(&cpu, 0x40004100u, 0), phy);
    ASSERT_EQ(mem_read32(cpu.mem, phy + 0x1A4u), 0x33333333u);

    uint32_t lc_default = call_builtin_rom0(&cpu, 0x4002F494u, 0);
    uint32_t lc_hci = call_builtin_rom0(&cpu, 0x4002F488u, 0);
    ASSERT_EQ(mem_read16(cpu.mem, lc_default), 0x0522u);
    ASSERT_EQ(mem_read16(cpu.mem, lc_hci), 0x0C7Cu);

    uint32_t llcp = call_builtin_rom0(&cpu, 0x40043F64u, 0);
    uint32_t llm = call_builtin_rom0(&cpu, 0x4004C920u, 0);
    ASSERT_TRUE(llcp >= 0x50000000u && llcp + 171u < 0x50002000u);
    ASSERT_TRUE(llm >= 0x50000000u && llm + 37u * 8u <= 0x50002000u);

    uint32_t lb_default = call_builtin_rom0(&cpu, 0x4001C198u, 0);
    uint32_t lb_hci = call_builtin_rom0(&cpu, 0x4001C18Cu, 0);
    ASSERT_TRUE(lb_default >= 0x50000000u &&
                lb_default + 15u * 8u <= 0x50002000u);
    ASSERT_TRUE(lb_hci >= 0x50000000u &&
                lb_hci + 12u * 8u <= 0x50002000u);
    ASSERT_EQ(mem_read16(cpu.mem, lb_default + 0u * 8u), 0x0805u);
    ASSERT_EQ(mem_read16(cpu.mem, lb_default + 5u * 8u), 0x0607u);
    ASSERT_EQ(mem_read16(cpu.mem, lb_default + 14u * 8u), 0x060Du);
    ASSERT_EQ(mem_read16(cpu.mem, lb_hci + 0u * 8u), 0x0417u);
    ASSERT_EQ(mem_read16(cpu.mem, lb_hci + 7u * 8u), 0x0C76u);
    ASSERT_EQ(mem_read16(cpu.mem, lb_hci + 11u * 8u), 0x0444u);
    ASSERT_EQ(mem_read32(cpu.mem, lb_default + 4u), 0x4006FFF0u);
    ASSERT_EQ(mem_read32(cpu.mem, lb_hci + 4u), 0x4006FFF0u);

    static const struct {
        uint32_t accessor;
        uint16_t first_tag;
    } tagged_tables[] = {
        {0x40046058u, 0x0104u}, /* LLC default-state table */
        {0x4005425Cu, 0x0408u}, /* LM HCI command table */
        {0x40054268u, 0x0401u}, /* LM default-state table */
        {0x40042358u, 0x2013u}, /* LLC HCI command table */
        {0x4004E718u, 0x0009u}, /* LLM default-state table */
    };
    for (size_t i = 0;
         i < sizeof(tagged_tables) / sizeof(tagged_tables[0]); i++) {
        uint32_t table = call_builtin_rom0(&cpu, tagged_tables[i].accessor, 0);
        ASSERT_TRUE(table >= 0x50000000u && table + 8u < 0x50002000u);
        ASSERT_EQ(mem_read16(cpu.mem, table), tagged_tables[i].first_tag);
    }

    call_builtin_rom0(&cpu, 0x40054288u, 0x3FFB1234u);
    ASSERT_EQ(mem_read32(cpu.mem, 0x50000DFCu), 0x3FFB1234u);
    /* The virtual HCI host owns observable BLE state; the closed ROM link
     * controller therefore initializes as a recognized no-op. */
    call_builtin_rom0(&cpu, 0x4001C948u, 0);
    ASSERT_EQ(rom_stubs_unregistered_count(rom), 0);

    rom_stubs_destroy(rom);
    teardown(&cpu);
}

/* ===== Run all ===== */

static void run_rom_stub_tests(void) {
    TEST_SUITE("ROM Stubs (M10)");
    RUN_TEST(test_pc_hook_fires);
    RUN_TEST(test_pc_hook_skips_non_match);
    RUN_TEST(test_loaded_rom_executes_unregistered_entry);
    RUN_TEST(test_classic_rom_abi_is_not_installed_on_s3);
    RUN_TEST(test_rom_stub_dispatch);
    RUN_TEST(test_rom_conditional_stub);
    RUN_TEST(test_rom_conditional_fallback_preserves_existing_hook);
    RUN_TEST(test_rom_registration_backscans_only_post_entry_symbols);
    RUN_TEST(test_rom_arg_call4);
    RUN_TEST(test_rom_arg_call0);
    RUN_TEST(test_stub_write_char);
    RUN_TEST(test_stub_printf_basic);
    RUN_TEST(test_stub_printf_hex);
    RUN_TEST(test_stub_printf_string);
    RUN_TEST(test_stub_delay_us);
    RUN_TEST(test_stub_cache_noop);
    RUN_TEST(test_cache_flash_mmu_rom_api_uses_byte_addresses);
    RUN_TEST(test_cache_sram_mmu_rom_api_maps_target_psram);
    RUN_TEST(test_stub_memcpy);
    RUN_TEST(test_rom_string_spans_and_bounded_concat);
    RUN_TEST(test_cpu_frequency_rom_pair);
    RUN_TEST(test_rom_newlib_scalar_helpers);
    RUN_TEST(test_rom_strdup_uses_guest_allocator);
    RUN_TEST(test_rom_sfp_initializes_and_reuses_guest_files);
    RUN_TEST(test_rom_fclose_flushes_closes_and_releases_guest_file);
    RUN_TEST(test_rom_open_dispatches_through_guest_syscall_table);
    RUN_TEST(test_rom_open_fails_when_syscall_table_is_uninitialized);
    RUN_TEST(test_firmware_phy_wrapper_installs_virtual_table);
    RUN_TEST(test_firmware_phy_wrapper_has_no_address_fallback);
    RUN_TEST(test_structural_abi_accels_do_not_require_firmware_profile);
    RUN_TEST(test_newlib_memcmp_is_relocated_and_cycle_exact);
    RUN_TEST(test_newlib_memcmp_hook_matches_original_routine);
    RUN_TEST(test_wled_v1601_uses_structural_memcmp_and_scanned_phy);
    RUN_TEST(test_idf_watchpoint_is_relocated_and_matches_original_routine);
    RUN_TEST(test_idf_port_enter_is_relocated_and_matches_original_routine);
    RUN_TEST(test_idf_port_exit_is_relocated_and_matches_original_routine);
    RUN_TEST(test_idf_heap_lock_wrappers_compose_verified_critical_sections);
    RUN_TEST(test_openhasp_lanbon_requires_complete_fingerprint);
    RUN_TEST(test_tasmota32_requires_complete_fingerprint);
    RUN_TEST(test_marauder_same_entry_uses_instruction_fingerprint);
    RUN_TEST(test_marauder_v1121_cyd2usb_uses_independent_fingerprint);
    RUN_TEST(test_marauder_v1121_virtualizes_only_its_phy_and_sync_state);
    RUN_TEST(test_marauder_v11423_hooks_use_shifted_phy_and_data_layout);
    RUN_TEST(test_marauder_v1151_hooks_use_third_layout);
    RUN_TEST(test_marauder_nimble_deinit_preserves_cpp_lists);
    RUN_TEST(test_marauder_dport_cache_stall_is_coherent_noop);
    RUN_TEST(test_bt_rom_table_accessors_use_bounded_scratch);
}
