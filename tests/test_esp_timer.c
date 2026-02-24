/*
 * esp_timer stub tests
 */
#include "esp_timer_stubs.h"
#include "rom_stubs.h"

/* Forward declarations of stub functions for direct testing */
extern void stub_esp_timer_create(xtensa_cpu_t *, void *);
extern void stub_esp_timer_start_periodic(xtensa_cpu_t *, void *);
extern void stub_esp_timer_start_once(xtensa_cpu_t *, void *);
extern void stub_esp_timer_stop(xtensa_cpu_t *, void *);
extern void stub_esp_timer_delete(xtensa_cpu_t *, void *);
extern void stub_esp_timer_get_time(xtensa_cpu_t *, void *);

static void et_setup(xtensa_cpu_t *cpu, esp32_rom_stubs_t **rom_out, esp_timer_stubs_t **et_out) {
    xtensa_cpu_init(cpu);
    cpu->mem = mem_create();
    cpu->pc = BASE;
    *rom_out = rom_stubs_create(cpu);
    *et_out = esp_timer_stubs_create(cpu);
}

static void et_teardown(xtensa_cpu_t *cpu, esp32_rom_stubs_t *rom, esp_timer_stubs_t *et) {
    esp_timer_stubs_destroy(et);
    rom_stubs_destroy(rom);
    mem_destroy(cpu->mem);
}

TEST(test_esp_timer_create_returns_ok) {
    xtensa_cpu_t cpu;
    esp32_rom_stubs_t *rom;
    esp_timer_stubs_t *et;
    et_setup(&cpu, &rom, &et);

    uint32_t create_addr = 0x400D1000;
    rom_stubs_register_ctx(rom, create_addr, (rom_stub_fn)stub_esp_timer_create, "esp_timer_create", et);

    /* Set up args struct in memory: callback=0x400D2000, arg=0x42 */
    uint32_t args_addr = 0x3FFB3000;
    mem_write32(cpu.mem, args_addr, 0x400D2000);     /* callback */
    mem_write32(cpu.mem, args_addr + 4, 0x42);        /* arg */
    mem_write32(cpu.mem, args_addr + 8, 0);            /* dispatch_method */
    mem_write32(cpu.mem, args_addr + 12, 0);           /* name */

    uint32_t handle_out = 0x3FFB3100;
    mem_write32(cpu.mem, handle_out, 0);

    XT_PS_SET_CALLINC(cpu.ps, 0);
    ar_write(&cpu, 0, BASE + 0x100);
    ar_write(&cpu, 2, args_addr);
    ar_write(&cpu, 3, handle_out);
    cpu.pc = create_addr;
    xtensa_step(&cpu);

    ASSERT_EQ(ar_read(&cpu, 2), 0);  /* ESP_OK */
    ASSERT_TRUE(mem_read32(cpu.mem, handle_out) != 0);  /* handle assigned */
    ASSERT_EQ(esp_timer_stubs_timer_count(et), 1);

    et_teardown(&cpu, rom, et);
}

TEST(test_esp_timer_start_stop) {
    xtensa_cpu_t cpu;
    esp32_rom_stubs_t *rom;
    esp_timer_stubs_t *et;
    et_setup(&cpu, &rom, &et);

    uint32_t create_addr = 0x400D1000;
    uint32_t start_addr  = 0x400D1010;
    uint32_t stop_addr   = 0x400D1020;
    rom_stubs_register_ctx(rom, create_addr, (rom_stub_fn)stub_esp_timer_create, "esp_timer_create", et);
    rom_stubs_register_ctx(rom, start_addr, (rom_stub_fn)stub_esp_timer_start_periodic, "esp_timer_start_periodic", et);
    rom_stubs_register_ctx(rom, stop_addr, (rom_stub_fn)stub_esp_timer_stop, "esp_timer_stop", et);

    /* Create timer */
    uint32_t args_addr = 0x3FFB3000;
    mem_write32(cpu.mem, args_addr, 0x400D2000);
    mem_write32(cpu.mem, args_addr + 4, 0);
    uint32_t handle_out = 0x3FFB3100;
    XT_PS_SET_CALLINC(cpu.ps, 0);
    ar_write(&cpu, 0, BASE + 0x100);
    ar_write(&cpu, 2, args_addr);
    ar_write(&cpu, 3, handle_out);
    cpu.pc = create_addr;
    xtensa_step(&cpu);
    uint32_t handle = mem_read32(cpu.mem, handle_out);

    /* Start periodic with 1000us period */
    XT_PS_SET_CALLINC(cpu.ps, 0);
    ar_write(&cpu, 0, BASE + 0x100);
    ar_write(&cpu, 2, handle);
    ar_write(&cpu, 3, 1000);
    cpu.pc = start_addr;
    xtensa_step(&cpu);
    ASSERT_EQ(ar_read(&cpu, 2), 0);  /* ESP_OK */

    /* Stop */
    XT_PS_SET_CALLINC(cpu.ps, 0);
    ar_write(&cpu, 0, BASE + 0x100);
    ar_write(&cpu, 2, handle);
    cpu.pc = stop_addr;
    xtensa_step(&cpu);
    ASSERT_EQ(ar_read(&cpu, 2), 0);  /* ESP_OK */

    et_teardown(&cpu, rom, et);
}

TEST(test_esp_timer_delete) {
    xtensa_cpu_t cpu;
    esp32_rom_stubs_t *rom;
    esp_timer_stubs_t *et;
    et_setup(&cpu, &rom, &et);

    uint32_t create_addr = 0x400D1000;
    uint32_t delete_addr = 0x400D1030;
    rom_stubs_register_ctx(rom, create_addr, (rom_stub_fn)stub_esp_timer_create, "esp_timer_create", et);
    rom_stubs_register_ctx(rom, delete_addr, (rom_stub_fn)stub_esp_timer_delete, "esp_timer_delete", et);

    /* Create */
    uint32_t args_addr = 0x3FFB3000;
    mem_write32(cpu.mem, args_addr, 0x400D2000);
    mem_write32(cpu.mem, args_addr + 4, 0);
    uint32_t handle_out = 0x3FFB3100;
    XT_PS_SET_CALLINC(cpu.ps, 0);
    ar_write(&cpu, 0, BASE + 0x100);
    ar_write(&cpu, 2, args_addr);
    ar_write(&cpu, 3, handle_out);
    cpu.pc = create_addr;
    xtensa_step(&cpu);
    uint32_t handle = mem_read32(cpu.mem, handle_out);

    /* Delete */
    XT_PS_SET_CALLINC(cpu.ps, 0);
    ar_write(&cpu, 0, BASE + 0x100);
    ar_write(&cpu, 2, handle);
    cpu.pc = delete_addr;
    xtensa_step(&cpu);
    ASSERT_EQ(ar_read(&cpu, 2), 0);  /* ESP_OK */

    et_teardown(&cpu, rom, et);
}

TEST(test_esp_timer_get_time) {
    xtensa_cpu_t cpu;
    esp32_rom_stubs_t *rom;
    esp_timer_stubs_t *et;
    et_setup(&cpu, &rom, &et);

    uint32_t addr = 0x400D1040;
    rom_stubs_register_ctx(rom, addr, (rom_stub_fn)stub_esp_timer_get_time, "esp_timer_get_time", et);

    /* Set ccount to 160,000 cycles = 1000 microseconds at 160 MHz */
    cpu.ccount = 160000;

    XT_PS_SET_CALLINC(cpu.ps, 0);
    ar_write(&cpu, 0, BASE + 0x100);
    cpu.pc = addr;
    xtensa_step(&cpu);

    /* Return value: a2 = low 32 bits = 1000 */
    ASSERT_EQ(ar_read(&cpu, 2), 1000);

    et_teardown(&cpu, rom, et);
}

TEST(test_esp_timer_start_once) {
    xtensa_cpu_t cpu;
    esp32_rom_stubs_t *rom;
    esp_timer_stubs_t *et;
    et_setup(&cpu, &rom, &et);

    uint32_t create_addr = 0x400D1000;
    uint32_t once_addr   = 0x400D1050;
    rom_stubs_register_ctx(rom, create_addr, (rom_stub_fn)stub_esp_timer_create, "esp_timer_create", et);
    rom_stubs_register_ctx(rom, once_addr, (rom_stub_fn)stub_esp_timer_start_once, "esp_timer_start_once", et);

    /* Create */
    uint32_t args_addr = 0x3FFB3000;
    mem_write32(cpu.mem, args_addr, 0x400D2000);
    mem_write32(cpu.mem, args_addr + 4, 0);
    uint32_t handle_out = 0x3FFB3100;
    XT_PS_SET_CALLINC(cpu.ps, 0);
    ar_write(&cpu, 0, BASE + 0x100);
    ar_write(&cpu, 2, args_addr);
    ar_write(&cpu, 3, handle_out);
    cpu.pc = create_addr;
    xtensa_step(&cpu);

    uint32_t handle = mem_read32(cpu.mem, handle_out);

    /* Start once with 5000us timeout */
    XT_PS_SET_CALLINC(cpu.ps, 0);
    ar_write(&cpu, 0, BASE + 0x100);
    ar_write(&cpu, 2, handle);
    ar_write(&cpu, 3, 5000);
    cpu.pc = once_addr;
    xtensa_step(&cpu);
    ASSERT_EQ(ar_read(&cpu, 2), 0);  /* ESP_OK */

    et_teardown(&cpu, rom, et);
}

TEST(test_esp_timer_multiple_creates) {
    xtensa_cpu_t cpu;
    esp32_rom_stubs_t *rom;
    esp_timer_stubs_t *et;
    et_setup(&cpu, &rom, &et);

    uint32_t create_addr = 0x400D1000;
    rom_stubs_register_ctx(rom, create_addr, (rom_stub_fn)stub_esp_timer_create, "esp_timer_create", et);

    uint32_t args_addr = 0x3FFB3000;
    uint32_t handle_out1 = 0x3FFB3100;
    uint32_t handle_out2 = 0x3FFB3110;

    /* Create timer 1 */
    mem_write32(cpu.mem, args_addr, 0x400D2000);
    mem_write32(cpu.mem, args_addr + 4, 0x10);
    XT_PS_SET_CALLINC(cpu.ps, 0);
    ar_write(&cpu, 0, BASE + 0x100);
    ar_write(&cpu, 2, args_addr);
    ar_write(&cpu, 3, handle_out1);
    cpu.pc = create_addr;
    xtensa_step(&cpu);
    ASSERT_EQ(ar_read(&cpu, 2), 0);

    /* Create timer 2 */
    mem_write32(cpu.mem, args_addr, 0x400D3000);
    mem_write32(cpu.mem, args_addr + 4, 0x20);
    XT_PS_SET_CALLINC(cpu.ps, 0);
    ar_write(&cpu, 0, BASE + 0x100);
    ar_write(&cpu, 2, args_addr);
    ar_write(&cpu, 3, handle_out2);
    cpu.pc = create_addr;
    xtensa_step(&cpu);
    ASSERT_EQ(ar_read(&cpu, 2), 0);

    /* Handles should be different */
    ASSERT_TRUE(mem_read32(cpu.mem, handle_out1) != mem_read32(cpu.mem, handle_out2));
    ASSERT_EQ(esp_timer_stubs_timer_count(et), 2);

    et_teardown(&cpu, rom, et);
}

static void run_esp_timer_tests(void) {
    TEST_SUITE("esp_timer_stubs");
    RUN_TEST(test_esp_timer_create_returns_ok);
    RUN_TEST(test_esp_timer_start_stop);
    RUN_TEST(test_esp_timer_delete);
    RUN_TEST(test_esp_timer_get_time);
    RUN_TEST(test_esp_timer_start_once);
    RUN_TEST(test_esp_timer_multiple_creates);
}
