/*
 * Tests for ROM function stubs (M10).
 * PC hook mechanism, calling convention, and built-in ROM stubs.
 */
#include "test_helpers.h"
#include "peripherals.h"
#include "rom_stubs.h"
#include <string.h>

/* NOP: op0=0, op1=0, op2=0, r=2, t=0, s=15 */
static uint32_t rom_nop_insn(void) {
    return rrr(0, 0, 2, 15, 0);
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

static uint32_t encode_test_l32r(uint32_t pc, uint32_t literal, int reg) {
    uint32_t base = (pc + 3u) & ~3u;
    uint32_t delta = literal - base;
    return 1u | ((uint32_t)reg << 4) |
           (((delta >> 2) & 0xFFFFu) << 8);
}

TEST(test_firmware_phy_wrapper_installs_virtual_table) {
    xtensa_cpu_t cpu;
    setup(&cpu);
    esp32_rom_stubs_t *rom = rom_stubs_create(&cpu);

    const uint32_t wrapper = 0x40189A2Cu;
    const uint32_t rom_literal = wrapper - 0x104u;
    const uint32_t global_literal = wrapper - 0x100u;
    const uint32_t phy_global = 0x3FFB2000u;

    /* Minimal version of the production wrapper's instruction pattern:
     * NOP; L32R a8, phy_get_romfuncs; NOP; L32R a8, table_global. */
    put_insn3(&cpu, wrapper, rom_nop_insn());
    put_insn3(&cpu, wrapper + 3u,
              encode_test_l32r(wrapper + 3u, rom_literal, 8));
    put_insn3(&cpu, wrapper + 6u, rom_nop_insn());
    put_insn3(&cpu, wrapper + 9u,
              encode_test_l32r(wrapper + 9u, global_literal, 8));
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

TEST(test_marauder_nimble_deinit_preserves_cpp_lists) {
    xtensa_cpu_t cpu;
    setup(&cpu);
    esp32_rom_stubs_t *rom = rom_stubs_create(&cpu);

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
    RUN_TEST(test_rom_stub_dispatch);
    RUN_TEST(test_rom_conditional_stub);
    RUN_TEST(test_rom_arg_call4);
    RUN_TEST(test_rom_arg_call0);
    RUN_TEST(test_stub_write_char);
    RUN_TEST(test_stub_printf_basic);
    RUN_TEST(test_stub_printf_hex);
    RUN_TEST(test_stub_printf_string);
    RUN_TEST(test_stub_delay_us);
    RUN_TEST(test_stub_cache_noop);
    RUN_TEST(test_cache_flash_mmu_rom_api_uses_byte_addresses);
    RUN_TEST(test_stub_memcpy);
    RUN_TEST(test_firmware_phy_wrapper_installs_virtual_table);
    RUN_TEST(test_marauder_nimble_deinit_preserves_cpp_lists);
    RUN_TEST(test_marauder_dport_cache_stall_is_coherent_noop);
    RUN_TEST(test_bt_rom_table_accessors_use_bounded_scratch);
}
