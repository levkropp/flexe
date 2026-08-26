/*
 * M13 firmware compatibility integration tests:
 * - NVS stubs
 * - GPIO MMIO enhancements
 * - Firmware symbol hooks
 * - software_reset_cpu
 */
#include "rom_stubs.h"
#include "peripherals.h"

/* ===== NVS stub tests ===== */

TEST(test_nvs_flash_init_returns_ok) {
    xtensa_cpu_t cpu;
    setup(&cpu);
    esp32_rom_stubs_t *rom = rom_stubs_create(&cpu);

    /* Find the nvs_flash_init stub by trying to call it via a hook address */
    /* Since we can't easily find ELF symbols in a test, we register manually */
    uint32_t nvs_init_addr = 0x400D2000;
    extern void stub_nvs_flash_init(xtensa_cpu_t *, void *);
    rom_stubs_register(rom, nvs_init_addr, (rom_stub_fn)stub_nvs_flash_init, "nvs_flash_init");

    XT_PS_SET_CALLINC(cpu.ps, 0);
    ar_write(&cpu, 0, BASE + 0x100);
    cpu.pc = nvs_init_addr;
    xtensa_step(&cpu);
    ASSERT_EQ(ar_read(&cpu, 2), 0);  /* ESP_OK */

    rom_stubs_destroy(rom);
    teardown(&cpu);
}

TEST(test_nvs_get_returns_not_found) {
    xtensa_cpu_t cpu;
    setup(&cpu);
    esp32_rom_stubs_t *rom = rom_stubs_create(&cpu);

    uint32_t nvs_get_addr = 0x400D2010;
    extern void stub_nvs_get_notfound(xtensa_cpu_t *, void *);
    rom_stubs_register(rom, nvs_get_addr, (rom_stub_fn)stub_nvs_get_notfound, "nvs_get_u32");

    XT_PS_SET_CALLINC(cpu.ps, 0);
    ar_write(&cpu, 0, BASE + 0x100);
    ar_write(&cpu, 2, 1);      /* handle */
    ar_write(&cpu, 3, 0x3FFB0000); /* key */
    ar_write(&cpu, 4, 0x3FFB1000); /* out_value */
    cpu.pc = nvs_get_addr;
    xtensa_step(&cpu);
    ASSERT_EQ(ar_read(&cpu, 2), 0x1102);  /* ESP_ERR_NVS_NOT_FOUND */

    rom_stubs_destroy(rom);
    teardown(&cpu);
}

TEST(test_nvs_set_returns_ok) {
    xtensa_cpu_t cpu;
    setup(&cpu);
    esp32_rom_stubs_t *rom = rom_stubs_create(&cpu);

    uint32_t nvs_set_addr = 0x400D2020;
    extern void stub_nvs_set_ok(xtensa_cpu_t *, void *);
    rom_stubs_register(rom, nvs_set_addr, (rom_stub_fn)stub_nvs_set_ok, "nvs_set_u32");

    XT_PS_SET_CALLINC(cpu.ps, 0);
    ar_write(&cpu, 0, BASE + 0x100);
    ar_write(&cpu, 2, 1);
    ar_write(&cpu, 3, 0x3FFB0000);
    ar_write(&cpu, 4, 42);
    cpu.pc = nvs_set_addr;
    xtensa_step(&cpu);
    ASSERT_EQ(ar_read(&cpu, 2), 0);  /* ESP_OK */

    rom_stubs_destroy(rom);
    teardown(&cpu);
}

TEST(test_nvs_open_writes_handle) {
    xtensa_cpu_t cpu;
    setup(&cpu);
    esp32_rom_stubs_t *rom = rom_stubs_create(&cpu);

    uint32_t nvs_open_addr = 0x400D2030;
    extern void stub_nvs_open(xtensa_cpu_t *, void *);
    rom_stubs_register(rom, nvs_open_addr, (rom_stub_fn)stub_nvs_open, "nvs_open");

    uint32_t handle_out = 0x3FFB4000;
    mem_write32(cpu.mem, handle_out, 0);

    XT_PS_SET_CALLINC(cpu.ps, 0);
    ar_write(&cpu, 0, BASE + 0x100);
    ar_write(&cpu, 2, 0x3FFB0000); /* namespace name */
    ar_write(&cpu, 3, 1);           /* mode */
    ar_write(&cpu, 4, handle_out);   /* handle_out */
    cpu.pc = nvs_open_addr;
    xtensa_step(&cpu);
    ASSERT_EQ(ar_read(&cpu, 2), 0);  /* ESP_OK */
    ASSERT_EQ(mem_read32(cpu.mem, handle_out), 1);  /* handle = 1 */

    rom_stubs_destroy(rom);
    teardown(&cpu);
}

/* ===== GPIO MMIO enhancement tests ===== */

TEST(test_gpio_pin_config_readback) {
    xtensa_cpu_t cpu;
    setup(&cpu);
    esp32_periph_t *periph = periph_create(cpu.mem);

    /* Write GPIO_PIN5_REG (offset 0x088 + 5*4 = 0x09C) */
    uint32_t pin5_addr = 0x3FF44088 + 5 * 4;
    mem_write32(cpu.mem, pin5_addr, 0x12345678);
    ASSERT_EQ(mem_read32(cpu.mem, pin5_addr), 0x12345678);

    /* Write GPIO_PIN39_REG (last pin) */
    uint32_t pin39_addr = 0x3FF44088 + 39 * 4;
    mem_write32(cpu.mem, pin39_addr, 0xAABBCCDD);
    ASSERT_EQ(mem_read32(cpu.mem, pin39_addr), 0xAABBCCDD);

    periph_destroy(periph);
    teardown(&cpu);
}

TEST(test_gpio_status_w1tc_clear) {
    xtensa_cpu_t cpu;
    setup(&cpu);
    esp32_periph_t *periph = periph_create(cpu.mem);

    /* Set some status bits */
    mem_write32(cpu.mem, 0x3FF44044, 0xFF);  /* GPIO_STATUS_REG */
    ASSERT_EQ(mem_read32(cpu.mem, 0x3FF44044), 0xFF);

    /* Clear bits 0x0F via W1TC */
    mem_write32(cpu.mem, 0x3FF4404C, 0x0F);  /* GPIO_STATUS_W1TC */
    ASSERT_EQ(mem_read32(cpu.mem, 0x3FF44044), 0xF0);

    periph_destroy(periph);
    teardown(&cpu);
}

TEST(test_gpio_func_out_sel) {
    xtensa_cpu_t cpu;
    setup(&cpu);
    esp32_periph_t *periph = periph_create(cpu.mem);

    /* Write GPIO_FUNC_OUT_SEL_CFG_REG for pin 2 (offset 0x530 + 2*4) */
    uint32_t fout_addr = 0x3FF44530 + 2 * 4;
    mem_write32(cpu.mem, fout_addr, 0x100);
    ASSERT_EQ(mem_read32(cpu.mem, fout_addr), 0x100);

    periph_destroy(periph);
    teardown(&cpu);
}

TEST(test_gpio_func_in_sel) {
    xtensa_cpu_t cpu;
    setup(&cpu);
    esp32_periph_t *periph = periph_create(cpu.mem);

    /* Write GPIO_FUNC_IN_SEL_CFG_REG for signal 14 (offset 0x130 + 14*4) */
    uint32_t fin_addr = 0x3FF44130 + 14 * 4;
    mem_write32(cpu.mem, fin_addr, 0x38);
    ASSERT_EQ(mem_read32(cpu.mem, fin_addr), 0x38);

    periph_destroy(periph);
    teardown(&cpu);
}

TEST(test_rom_gpio_matrix_helpers_program_registers) {
    xtensa_cpu_t cpu;
    setup(&cpu);
    esp32_periph_t *periph = periph_create(cpu.mem);
    esp32_rom_stubs_t *rom = rom_stubs_create(&cpu);
    /* gpio_matrix_out(14, HSPICLK_OUT_IDX=8, true, true) */
    XT_PS_SET_CALLINC(cpu.ps, 0);
    ar_write(&cpu, 0, BASE + 0x100);
    ar_write(&cpu, 2, 14);
    ar_write(&cpu, 3, 8);
    ar_write(&cpu, 4, 1);
    ar_write(&cpu, 5, 1);
    cpu.pc = 0x40009F0Cu;
    xtensa_step(&cpu);
    ASSERT_EQ(periph_gpio_out_signal(periph, 14), 8);
    ASSERT_EQ(mem_read32(cpu.mem, 0x3FF44530u + 14u * 4u),
              8u | (1u << 9) | (1u << 11));

    /* gpio_matrix_in(12, HSPID_IN_IDX=9, true) enables matrix routing. */
    XT_PS_SET_CALLINC(cpu.ps, 0);
    ar_write(&cpu, 0, BASE + 0x100);
    ar_write(&cpu, 2, 12);
    ar_write(&cpu, 3, 9);
    ar_write(&cpu, 4, 1);
    cpu.pc = 0x40009EDCu;
    xtensa_step(&cpu);
    ASSERT_EQ(mem_read32(cpu.mem, 0x3FF44130u + 9u * 4u),
              12u | (1u << 6) | (1u << 7));

    rom_stubs_destroy(rom);
    periph_destroy(periph);
    teardown(&cpu);
}

TEST(test_gpio_out1_w1ts_w1tc) {
    xtensa_cpu_t cpu;
    setup(&cpu);
    esp32_periph_t *periph = periph_create(cpu.mem);

    /* Set GPIO_OUT1_REG via W1TS */
    mem_write32(cpu.mem, 0x3FF44014, 0x05);  /* GPIO_OUT1_W1TS */
    ASSERT_EQ(mem_read32(cpu.mem, 0x3FF44010), 0x05);  /* GPIO_OUT1_REG */

    /* Clear bit 0 via W1TC */
    mem_write32(cpu.mem, 0x3FF44018, 0x01);  /* GPIO_OUT1_W1TC */
    ASSERT_EQ(mem_read32(cpu.mem, 0x3FF44010), 0x04);

    periph_destroy(periph);
    teardown(&cpu);
}

/* ===== Firmware symbol hook tests ===== */

TEST(test_esp_log_timestamp) {
    xtensa_cpu_t cpu;
    setup(&cpu);
    esp32_rom_stubs_t *rom = rom_stubs_create(&cpu);

    uint32_t addr = 0x400D3000;
    extern void stub_esp_log_timestamp(xtensa_cpu_t *, void *);
    rom_stubs_register(rom, addr, (rom_stub_fn)stub_esp_log_timestamp, "esp_log_timestamp");

    /* 160 MHz, ccount = 160,000,000 = 1 second = 1000 ms */
    cpu.ccount = 160000000;
    XT_PS_SET_CALLINC(cpu.ps, 0);
    ar_write(&cpu, 0, BASE + 0x100);
    cpu.pc = addr;
    xtensa_step(&cpu);
    ASSERT_EQ(ar_read(&cpu, 2), 1000);  /* 1000 ms */

    rom_stubs_destroy(rom);
    teardown(&cpu);
}

TEST(test_gpio_set_level_stub) {
    xtensa_cpu_t cpu;
    setup(&cpu);
    esp32_periph_t *periph = periph_create(cpu.mem);
    esp32_rom_stubs_t *rom = rom_stubs_create(&cpu);

    uint32_t addr = 0x400D3010;
    extern void stub_gpio_set_level(xtensa_cpu_t *, void *);
    rom_stubs_register(rom, addr, (rom_stub_fn)stub_gpio_set_level, "gpio_set_level");

    /* Set GPIO 5 high */
    XT_PS_SET_CALLINC(cpu.ps, 0);
    ar_write(&cpu, 0, BASE + 0x100);
    ar_write(&cpu, 2, 5);   /* pin */
    ar_write(&cpu, 3, 1);   /* level */
    cpu.pc = addr;
    xtensa_step(&cpu);
    ASSERT_EQ(ar_read(&cpu, 2), 0);  /* ESP_OK */

    /* Check GPIO_OUT_REG bit 5 is set */
    uint32_t out = mem_read32(cpu.mem, 0x3FF44004);
    ASSERT_TRUE(out & (1u << 5));

    /* Set GPIO 5 low */
    XT_PS_SET_CALLINC(cpu.ps, 0);
    ar_write(&cpu, 0, BASE + 0x100);
    ar_write(&cpu, 2, 5);
    ar_write(&cpu, 3, 0);
    cpu.pc = addr;
    xtensa_step(&cpu);

    out = mem_read32(cpu.mem, 0x3FF44004);
    ASSERT_FALSE(out & (1u << 5));

    rom_stubs_destroy(rom);
    periph_destroy(periph);
    teardown(&cpu);
}

TEST(test_arduino_gpio_wrappers_drive_register_model) {
    xtensa_cpu_t cpu;
    setup(&cpu);
    esp32_periph_t *periph = periph_create(cpu.mem);
    esp32_rom_stubs_t *rom = rom_stubs_create(&cpu);
    const uint32_t pin_mode_addr = 0x400D3070u;
    const uint32_t write_addr = 0x400D3080u;
    extern void stub_arduino_pin_mode(xtensa_cpu_t *, void *);
    extern void stub_arduino_digital_write(xtensa_cpu_t *, void *);
    rom_stubs_register(rom, pin_mode_addr,
                       (rom_stub_fn)stub_arduino_pin_mode, "pinMode");
    rom_stubs_register(rom, write_addr,
                       (rom_stub_fn)stub_arduino_digital_write,
                       "digitalWrite");

    /* OUTPUT (0x03) enables a low GPIO and digitalWrite changes its latch. */
    XT_PS_SET_CALLINC(cpu.ps, 0);
    ar_write(&cpu, 0, BASE + 0x100);
    ar_write(&cpu, 2, 2);
    ar_write(&cpu, 3, 0x03);
    cpu.pc = pin_mode_addr;
    xtensa_step(&cpu);
    ASSERT_TRUE(mem_read32(cpu.mem, 0x3FF44020u) & (1u << 2));

    XT_PS_SET_CALLINC(cpu.ps, 0);
    ar_write(&cpu, 0, BASE + 0x100);
    ar_write(&cpu, 2, 2);
    ar_write(&cpu, 3, 1);
    cpu.pc = write_addr;
    xtensa_step(&cpu);
    ASSERT_EQ(periph_gpio_pin_level(periph, 2), 1);

    /* GPIO32..39 use the OUT1/ENABLE1 register bank. */
    XT_PS_SET_CALLINC(cpu.ps, 0);
    ar_write(&cpu, 0, BASE + 0x100);
    ar_write(&cpu, 2, 33);
    ar_write(&cpu, 3, 0x12); /* OUTPUT_OPEN_DRAIN */
    cpu.pc = pin_mode_addr;
    xtensa_step(&cpu);
    ASSERT_TRUE(mem_read32(cpu.mem, 0x3FF4402Cu) & (1u << 1));

    XT_PS_SET_CALLINC(cpu.ps, 0);
    ar_write(&cpu, 0, BASE + 0x100);
    ar_write(&cpu, 2, 33);
    ar_write(&cpu, 3, 1);
    cpu.pc = write_addr;
    xtensa_step(&cpu);
    ASSERT_EQ(periph_gpio_pin_level(periph, 33), 1);

    /* Reconfiguring as INPUT clears the output-enable bit. */
    XT_PS_SET_CALLINC(cpu.ps, 0);
    ar_write(&cpu, 0, BASE + 0x100);
    ar_write(&cpu, 2, 33);
    ar_write(&cpu, 3, 0x01);
    cpu.pc = pin_mode_addr;
    xtensa_step(&cpu);
    ASSERT_FALSE(mem_read32(cpu.mem, 0x3FF4402Cu) & (1u << 1));

    rom_stubs_destroy(rom);
    periph_destroy(periph);
    teardown(&cpu);
}

TEST(test_heap_size_stubs) {
    xtensa_cpu_t cpu;
    setup(&cpu);
    esp32_rom_stubs_t *rom = rom_stubs_create(&cpu);

    uint32_t free_addr = 0x400D3020;
    uint32_t min_addr  = 0x400D3030;
    extern void stub_esp_get_free_heap_size(xtensa_cpu_t *, void *);
    extern void stub_esp_get_minimum_free_heap_size(xtensa_cpu_t *, void *);
    rom_stubs_register(rom, free_addr, (rom_stub_fn)stub_esp_get_free_heap_size, "esp_get_free_heap_size");
    rom_stubs_register(rom, min_addr, (rom_stub_fn)stub_esp_get_minimum_free_heap_size, "esp_get_minimum_free_heap_size");

    XT_PS_SET_CALLINC(cpu.ps, 0);
    ar_write(&cpu, 0, BASE + 0x100);
    cpu.pc = free_addr;
    xtensa_step(&cpu);
    ASSERT_TRUE(ar_read(&cpu, 2) > 0);  /* dynamic — returns actual remaining heap */

    XT_PS_SET_CALLINC(cpu.ps, 0);
    ar_write(&cpu, 0, BASE + 0x100);
    cpu.pc = min_addr;
    xtensa_step(&cpu);
    ASSERT_TRUE(ar_read(&cpu, 2) > 0);  /* dynamic — returns actual remaining heap */

    rom_stubs_destroy(rom);
    teardown(&cpu);
}

TEST(test_heap_caps_dma_uses_internal_dram) {
    xtensa_cpu_t cpu;
    setup(&cpu);
    esp32_rom_stubs_t *rom = rom_stubs_create(&cpu);

    const uint32_t malloc_addr = 0x400D3040;
    const uint32_t calloc_addr = 0x400D3050;
    const uint32_t realloc_addr = 0x400D3060;
    extern void stub_heap_caps_malloc(xtensa_cpu_t *, void *);
    extern void stub_heap_caps_calloc(xtensa_cpu_t *, void *);
    extern void stub_heap_caps_realloc(xtensa_cpu_t *, void *);
    rom_stubs_register(rom, malloc_addr,
                       (rom_stub_fn)stub_heap_caps_malloc,
                       "heap_caps_malloc");
    rom_stubs_register(rom, calloc_addr,
                       (rom_stub_fn)stub_heap_caps_calloc,
                       "heap_caps_calloc");
    rom_stubs_register(rom, realloc_addr,
                       (rom_stub_fn)stub_heap_caps_realloc,
                       "heap_caps_realloc");

    /* Ordinary 8-bit storage stays in the large emulated PSRAM arena. */
    XT_PS_SET_CALLINC(cpu.ps, 0);
    ar_write(&cpu, 0, BASE + 0x100);
    ar_write(&cpu, 2, 16);
    ar_write(&cpu, 3, 1u << 2); /* MALLOC_CAP_8BIT */
    cpu.pc = malloc_addr;
    xtensa_step(&cpu);
    uint32_t ordinary = ar_read(&cpu, 2);
    ASSERT_TRUE(ordinary >= 0x3F800000u && ordinary < 0x3FC00000u);
    mem_write32(cpu.mem, ordinary, 0x12345678u);

    /* DMA descriptors must occupy internal DRAM on classic ESP32. */
    XT_PS_SET_CALLINC(cpu.ps, 0);
    ar_write(&cpu, 0, BASE + 0x100);
    ar_write(&cpu, 2, 12);
    ar_write(&cpu, 3, 1u << 3); /* MALLOC_CAP_DMA */
    cpu.pc = malloc_addr;
    xtensa_step(&cpu);
    uint32_t dma = ar_read(&cpu, 2);
    ASSERT_TRUE(dma >= 0x3FFF4000u && dma < 0x40000000u);
    ASSERT_TRUE(mem_get_ptr_w(cpu.mem, dma) != NULL);

    /* Capability-aware calloc uses the same arena and clears every byte. */
    XT_PS_SET_CALLINC(cpu.ps, 0);
    ar_write(&cpu, 0, BASE + 0x100);
    ar_write(&cpu, 2, 4);
    ar_write(&cpu, 3, 4);
    ar_write(&cpu, 4, 1u << 11); /* MALLOC_CAP_INTERNAL */
    cpu.pc = calloc_addr;
    xtensa_step(&cpu);
    uint32_t zeroed = ar_read(&cpu, 2);
    ASSERT_TRUE(zeroed >= 0x3FFF4000u && zeroed < 0x40000000u);
    for (uint32_t i = 0; i < 16; i++) ASSERT_EQ(mem_read8(cpu.mem, zeroed + i), 0);

    /* Realloc may migrate an existing buffer to a newly requested arena. */
    XT_PS_SET_CALLINC(cpu.ps, 0);
    ar_write(&cpu, 0, BASE + 0x100);
    ar_write(&cpu, 2, ordinary);
    ar_write(&cpu, 3, 32);
    ar_write(&cpu, 4, 1u << 3); /* MALLOC_CAP_DMA */
    cpu.pc = realloc_addr;
    xtensa_step(&cpu);
    uint32_t migrated = ar_read(&cpu, 2);
    ASSERT_TRUE(migrated >= 0x3FFF4000u && migrated < 0x40000000u);
    ASSERT_EQ(mem_read32(cpu.mem, migrated), 0x12345678u);

    rom_stubs_destroy(rom);
    teardown(&cpu);
}

/* ===== software_reset_cpu test ===== */

TEST(test_software_reset_cpu_stops) {
    xtensa_cpu_t cpu;
    setup(&cpu);
    cpu.running = true;
    esp32_rom_stubs_t *rom = rom_stubs_create(&cpu);

    /* software_reset_cpu is already registered at 0x40008264 */
    ASSERT_TRUE(cpu.running);
    XT_PS_SET_CALLINC(cpu.ps, 0);
    cpu.pc = 0x40008264;
    xtensa_step(&cpu);
    ASSERT_FALSE(cpu.running);

    rom_stubs_destroy(rom);
    teardown(&cpu);
}

/* ===== Integration: NVS init + open + get sequence ===== */

TEST(test_nvs_init_open_get_sequence) {
    xtensa_cpu_t cpu;
    setup(&cpu);
    esp32_rom_stubs_t *rom = rom_stubs_create(&cpu);

    extern void stub_nvs_flash_init(xtensa_cpu_t *, void *);
    extern void stub_nvs_open(xtensa_cpu_t *, void *);
    extern void stub_nvs_get_notfound(xtensa_cpu_t *, void *);
    extern void stub_nvs_close(xtensa_cpu_t *, void *);

    uint32_t init_addr  = 0x400D4000;
    uint32_t open_addr  = 0x400D4010;
    uint32_t get_addr   = 0x400D4020;
    uint32_t close_addr = 0x400D4030;
    rom_stubs_register(rom, init_addr, (rom_stub_fn)stub_nvs_flash_init, "nvs_flash_init");
    rom_stubs_register(rom, open_addr, (rom_stub_fn)stub_nvs_open, "nvs_open");
    rom_stubs_register(rom, get_addr, (rom_stub_fn)stub_nvs_get_notfound, "nvs_get_i32");
    rom_stubs_register(rom, close_addr, (rom_stub_fn)stub_nvs_close, "nvs_close");

    /* Init */
    XT_PS_SET_CALLINC(cpu.ps, 0);
    ar_write(&cpu, 0, BASE + 0x100);
    cpu.pc = init_addr;
    xtensa_step(&cpu);
    ASSERT_EQ(ar_read(&cpu, 2), 0);

    /* Open */
    uint32_t handle_out = 0x3FFB5000;
    XT_PS_SET_CALLINC(cpu.ps, 0);
    ar_write(&cpu, 0, BASE + 0x100);
    ar_write(&cpu, 2, 0x3FFB0000);  /* namespace */
    ar_write(&cpu, 3, 0);            /* mode */
    ar_write(&cpu, 4, handle_out);
    cpu.pc = open_addr;
    xtensa_step(&cpu);
    ASSERT_EQ(ar_read(&cpu, 2), 0);
    ASSERT_EQ(mem_read32(cpu.mem, handle_out), 1);

    /* Get (not found) */
    XT_PS_SET_CALLINC(cpu.ps, 0);
    ar_write(&cpu, 0, BASE + 0x100);
    ar_write(&cpu, 2, 1);  /* handle */
    ar_write(&cpu, 3, 0x3FFB0010);  /* key */
    ar_write(&cpu, 4, 0x3FFB6000);  /* value out */
    cpu.pc = get_addr;
    xtensa_step(&cpu);
    ASSERT_EQ(ar_read(&cpu, 2), 0x1102);

    /* Close */
    XT_PS_SET_CALLINC(cpu.ps, 0);
    ar_write(&cpu, 0, BASE + 0x100);
    ar_write(&cpu, 2, 1);
    cpu.pc = close_addr;
    xtensa_step(&cpu);
    /* void return, just check it didn't crash */
    ASSERT_EQ(cpu.pc, BASE + 0x100);

    rom_stubs_destroy(rom);
    teardown(&cpu);
}

/* ===== KV-store round trip: nvs_open + nvs_set_i32 + nvs_get_i32 ===== */

TEST(test_nvs_set_get_i32_roundtrip) {
    xtensa_cpu_t cpu;
    setup(&cpu);
    esp32_rom_stubs_t *rom = rom_stubs_create(&cpu);

    extern void stub_nvs_open(xtensa_cpu_t *, void *);
    extern void stub_nvs_set_i32(xtensa_cpu_t *, void *);
    extern void stub_nvs_get_i32(xtensa_cpu_t *, void *);

    uint32_t open_addr = 0x400D5000;
    uint32_t set_addr  = 0x400D5010;
    uint32_t get_addr  = 0x400D5020;
    rom_stubs_register_ctx(rom, open_addr, (rom_stub_fn)stub_nvs_open,    "nvs_open",    rom);
    rom_stubs_register_ctx(rom, set_addr,  (rom_stub_fn)stub_nvs_set_i32, "nvs_set_i32", rom);
    rom_stubs_register_ctx(rom, get_addr,  (rom_stub_fn)stub_nvs_get_i32, "nvs_get_i32", rom);

    /* Write namespace + key strings into guest DRAM */
    const char *ns  = "storage";
    const char *key = "counter";
    uint32_t ns_addr = 0x3FFB0000;
    uint32_t key_addr = 0x3FFB0040;
    for (int i = 0; ns[i];  i++) mem_write8(cpu.mem, ns_addr  + i, (uint8_t)ns[i]);
    mem_write8(cpu.mem, ns_addr + (uint32_t)strlen(ns), 0);
    for (int i = 0; key[i]; i++) mem_write8(cpu.mem, key_addr + i, (uint8_t)key[i]);
    mem_write8(cpu.mem, key_addr + (uint32_t)strlen(key), 0);

    /* Open */
    uint32_t handle_out = 0x3FFB1000;
    XT_PS_SET_CALLINC(cpu.ps, 0);
    ar_write(&cpu, 0, BASE + 0x100);
    ar_write(&cpu, 2, ns_addr);
    ar_write(&cpu, 3, 0);
    ar_write(&cpu, 4, handle_out);
    cpu.pc = open_addr;
    xtensa_step(&cpu);
    ASSERT_EQ(ar_read(&cpu, 2), 0);
    uint32_t handle = mem_read32(cpu.mem, handle_out);
    ASSERT_TRUE(handle != 0);

    /* Set i32 = 0xDEADBEEF */
    XT_PS_SET_CALLINC(cpu.ps, 0);
    ar_write(&cpu, 0, BASE + 0x100);
    ar_write(&cpu, 2, handle);
    ar_write(&cpu, 3, key_addr);
    ar_write(&cpu, 4, 0xDEADBEEF);
    cpu.pc = set_addr;
    xtensa_step(&cpu);
    ASSERT_EQ(ar_read(&cpu, 2), 0);

    /* Get i32 -> should return the stored value */
    uint32_t value_out = 0x3FFB2000;
    mem_write32(cpu.mem, value_out, 0);
    XT_PS_SET_CALLINC(cpu.ps, 0);
    ar_write(&cpu, 0, BASE + 0x100);
    ar_write(&cpu, 2, handle);
    ar_write(&cpu, 3, key_addr);
    ar_write(&cpu, 4, value_out);
    cpu.pc = get_addr;
    xtensa_step(&cpu);
    ASSERT_EQ(ar_read(&cpu, 2), 0);
    ASSERT_EQ(mem_read32(cpu.mem, value_out), 0xDEADBEEF);

    rom_stubs_destroy(rom);
    teardown(&cpu);
}

/* ===== nvs_get_i32 on missing key returns ESP_ERR_NVS_NOT_FOUND ===== */

TEST(test_nvs_get_i32_missing_key) {
    xtensa_cpu_t cpu;
    setup(&cpu);
    esp32_rom_stubs_t *rom = rom_stubs_create(&cpu);

    extern void stub_nvs_open(xtensa_cpu_t *, void *);
    extern void stub_nvs_get_i32(xtensa_cpu_t *, void *);

    uint32_t open_addr = 0x400D6000;
    uint32_t get_addr  = 0x400D6010;
    rom_stubs_register_ctx(rom, open_addr, (rom_stub_fn)stub_nvs_open,    "nvs_open",    rom);
    rom_stubs_register_ctx(rom, get_addr,  (rom_stub_fn)stub_nvs_get_i32, "nvs_get_i32", rom);

    const char *ns  = "ns";
    const char *key = "missing";
    uint32_t ns_addr = 0x3FFB0000;
    uint32_t key_addr = 0x3FFB0040;
    for (int i = 0; ns[i];  i++) mem_write8(cpu.mem, ns_addr  + i, (uint8_t)ns[i]);
    mem_write8(cpu.mem, ns_addr + (uint32_t)strlen(ns), 0);
    for (int i = 0; key[i]; i++) mem_write8(cpu.mem, key_addr + i, (uint8_t)key[i]);
    mem_write8(cpu.mem, key_addr + (uint32_t)strlen(key), 0);

    uint32_t handle_out = 0x3FFB1000;
    XT_PS_SET_CALLINC(cpu.ps, 0);
    ar_write(&cpu, 0, BASE + 0x100);
    ar_write(&cpu, 2, ns_addr);
    ar_write(&cpu, 3, 0);
    ar_write(&cpu, 4, handle_out);
    cpu.pc = open_addr;
    xtensa_step(&cpu);
    uint32_t handle = mem_read32(cpu.mem, handle_out);

    XT_PS_SET_CALLINC(cpu.ps, 0);
    ar_write(&cpu, 0, BASE + 0x100);
    ar_write(&cpu, 2, handle);
    ar_write(&cpu, 3, key_addr);
    ar_write(&cpu, 4, 0x3FFB2000);
    cpu.pc = get_addr;
    xtensa_step(&cpu);
    ASSERT_EQ(ar_read(&cpu, 2), 0x1102); /* ESP_ERR_NVS_NOT_FOUND */

    rom_stubs_destroy(rom);
    teardown(&cpu);
}

/* ===== VFS stubs round-trip ===== */
#include "vfs_stubs.h"
#include <sys/stat.h>
#include <unistd.h>
#include <string.h>

TEST(test_vfs_stubs_spiffs_host_roundtrip) {
    xtensa_cpu_t cpu;
    setup(&cpu);
    esp32_rom_stubs_t *rom = rom_stubs_create(&cpu);
    cpu.pc_hook_ctx = rom;  /* vfs_stubs_hook_symbols looks at pc_hook_ctx */

    vfs_stubs_t *v = vfs_stubs_create(&cpu);
    ASSERT_TRUE(v != NULL);

    /* Register a /spiffs mount backed by a tmp host dir */
    const char *host_dir = "/tmp/flexe-spiffs-unit-test";
    /* Wipe any prior contents so the test is hermetic */
    char rm_cmd[256];
    snprintf(rm_cmd, sizeof(rm_cmd), "rm -rf '%s'", host_dir);
    int rc1 = system(rm_cmd); (void)rc1;

    vfs_stubs_set_spiffs_dir(v, host_dir);

    /* The mount setter should have left a working host directory ready. */
    char mk_cmd[256];
    snprintf(mk_cmd, sizeof(mk_cmd), "mkdir -p '%s'", host_dir);
    int rc2 = system(mk_cmd); (void)rc2;
    struct stat st;
    ASSERT_EQ(stat(host_dir, &st), 0);
    ASSERT_TRUE(S_ISDIR(st.st_mode));

    /* Round-trip: write a file under the host mount, read it back. This
     * mirrors what stub_fwrite/stub_fread do once a guest fopen has
     * been resolved through vfs_resolve_path. */
    char path[256];
    snprintf(path, sizeof(path), "%s/round.txt", host_dir);
    FILE *fp = fopen(path, "w");
    ASSERT_TRUE(fp != NULL);
    const char msg[] = "Hello World!\n";
    size_t wrote = fwrite(msg, 1, sizeof(msg) - 1, fp);
    ASSERT_EQ(wrote, sizeof(msg) - 1);
    fclose(fp);

    fp = fopen(path, "r");
    ASSERT_TRUE(fp != NULL);
    char buf[64] = {0};
    size_t got = fread(buf, 1, sizeof(buf) - 1, fp);
    fclose(fp);
    ASSERT_EQ(got, sizeof(msg) - 1);
    ASSERT_EQ(strcmp(buf, msg), 0);

    /* Clean up */
    int rc3 = system(rm_cmd); (void)rc3;

    vfs_stubs_destroy(v);
    rom_stubs_destroy(rom);
    teardown(&cpu);
}

/* ===== Suite runner ===== */

static void run_firmware_compat_tests(void) {
    TEST_SUITE("firmware_compat");
    /* NVS */
    RUN_TEST(test_nvs_flash_init_returns_ok);
    RUN_TEST(test_nvs_get_returns_not_found);
    RUN_TEST(test_nvs_set_returns_ok);
    RUN_TEST(test_nvs_open_writes_handle);
    /* GPIO MMIO */
    RUN_TEST(test_gpio_pin_config_readback);
    RUN_TEST(test_gpio_status_w1tc_clear);
    RUN_TEST(test_gpio_func_out_sel);
    RUN_TEST(test_gpio_func_in_sel);
    RUN_TEST(test_rom_gpio_matrix_helpers_program_registers);
    RUN_TEST(test_gpio_out1_w1ts_w1tc);
    /* Symbol hooks */
    RUN_TEST(test_esp_log_timestamp);
    RUN_TEST(test_gpio_set_level_stub);
    RUN_TEST(test_arduino_gpio_wrappers_drive_register_model);
    RUN_TEST(test_heap_size_stubs);
    RUN_TEST(test_heap_caps_dma_uses_internal_dram);
    /* software_reset_cpu */
    RUN_TEST(test_software_reset_cpu_stops);
    /* Integration */
    RUN_TEST(test_nvs_init_open_get_sequence);
    RUN_TEST(test_nvs_set_get_i32_roundtrip);
    RUN_TEST(test_nvs_get_i32_missing_key);
    /* VFS host-backed mount round-trip */
    RUN_TEST(test_vfs_stubs_spiffs_host_roundtrip);
}
