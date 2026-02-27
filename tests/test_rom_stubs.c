/*
 * Tests for ROM function stubs (M10).
 * PC hook mechanism, calling convention, and built-in ROM stubs.
 */
#include "test_helpers.h"
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

/* ===== Run all ===== */

static void run_rom_stub_tests(void) {
    TEST_SUITE("ROM Stubs (M10)");
    RUN_TEST(test_pc_hook_fires);
    RUN_TEST(test_pc_hook_skips_non_match);
    RUN_TEST(test_rom_stub_dispatch);
    RUN_TEST(test_rom_arg_call4);
    RUN_TEST(test_rom_arg_call0);
    RUN_TEST(test_stub_write_char);
    RUN_TEST(test_stub_printf_basic);
    RUN_TEST(test_stub_printf_hex);
    RUN_TEST(test_stub_printf_string);
    RUN_TEST(test_stub_delay_us);
    RUN_TEST(test_stub_cache_noop);
    RUN_TEST(test_stub_memcpy);
}
