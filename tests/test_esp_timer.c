/*
 * esp_timer stub tests
 */
#include "esp_timer_stubs.h"
#include "rom_stubs.h"
#include "peripherals.h"

/* Forward declarations of stub functions for direct testing */
extern void stub_esp_timer_create(xtensa_cpu_t *, void *);
extern void stub_esp_timer_start_periodic(xtensa_cpu_t *, void *);
extern void stub_esp_timer_start_once(xtensa_cpu_t *, void *);
extern void stub_esp_timer_stop(xtensa_cpu_t *, void *);
extern void stub_esp_timer_delete(xtensa_cpu_t *, void *);
extern void stub_esp_timer_get_time(xtensa_cpu_t *, void *);
extern void stub_delay(xtensa_cpu_t *, void *);

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

    /* esp_timer_get_time returns host wall-clock elapsed microseconds.
     * Just verify it returns a small non-negative value (test runs quickly). */
    XT_PS_SET_CALLINC(cpu.ps, 0);
    ar_write(&cpu, 0, BASE + 0x100);
    cpu.pc = addr;
    xtensa_step(&cpu);

    /* Return value: a2 = low 32 bits, should be small (< 1 second) */
    ASSERT_TRUE(ar_read(&cpu, 2) < 1000000);  /* less than 1 second since boot */

    et_teardown(&cpu, rom, et);
}

static const uint8_t test_idf_lact_reader[] = {
    0x36, 0x41, 0x00, 0x21, 0xD3, 0xE8, 0x0C, 0x1A,
    0x91, 0xD1, 0xE8, 0xC0, 0x20, 0x00, 0x38, 0x09,
    0xC0, 0x20, 0x00, 0x88, 0x02, 0x80, 0x8D, 0xF4,
    0xAA, 0x88, 0x21, 0xCE, 0xE8, 0xC0, 0x20, 0x00,
    0xA9, 0x02, 0x76, 0x88, 0x07, 0xC0, 0x20, 0x00,
    0x28, 0x09, 0x27, 0x93, 0xFF, 0xA1, 0xCA, 0xE8,
    0x91, 0xC7, 0xE8, 0xC0, 0x20, 0x00, 0x38, 0x0A,
    0xC0, 0x20, 0x00, 0x88, 0x09, 0x87, 0x92, 0x04,
    0xC0, 0x20, 0x00, 0x1D, 0xF0, 0x2D, 0x08, 0x06,
    0xFA, 0xFF,
};

static const uint8_t test_idf_lact_accessor[] = {
    0x36, 0x41, 0x00, 0x25, 0xFB, 0xFF, 0x10, 0x2B,
    0x01, 0xA0, 0xA1, 0x41, 0xA0, 0x22, 0x20, 0xB0,
    0x31, 0x41, 0x1D, 0xF0,
};

static uint32_t seed_idf_lact_accessor(xtensa_cpu_t *cpu,
                                       uint32_t reader, uint32_t entry,
                                       uint32_t config_addr) {
    uint32_t literals = (reader - 0x100u) & ~3u;
    put_test_bytes(cpu, reader, test_idf_lact_reader,
                   sizeof(test_idf_lact_reader));
    put_test_bytes(cpu, entry, test_idf_lact_accessor,
                   sizeof(test_idf_lact_accessor));
    put_insn3(cpu, entry + 3u,
              encode_test_calln(entry + 3u, reader, 2u));
    put_insn3(cpu, reader + 3u,
              encode_test_l32r(reader + 3u, literals, 2u));
    put_insn3(cpu, reader + 8u,
              encode_test_l32r(reader + 8u, literals + 4u, 9u));
    put_insn3(cpu, reader + 26u,
              encode_test_l32r(reader + 26u, literals + 8u, 2u));
    put_insn3(cpu, reader + 45u,
              encode_test_l32r(reader + 45u, literals + 12u, 10u));
    put_insn3(cpu, reader + 48u,
              encode_test_l32r(reader + 48u, literals + 4u, 9u));
    mem_write32(cpu->mem, literals, config_addr);
    mem_write32(cpu->mem, literals + 4u, config_addr + 8u);
    mem_write32(cpu->mem, literals + 8u, config_addr + 16u);
    mem_write32(cpu->mem, literals + 12u, config_addr + 12u);
    return literals;
}

TEST(test_stripped_idf_lact_timer_is_discovered_structurally) {
    xtensa_cpu_t cpu;
    esp32_rom_stubs_t *rom;
    esp_timer_stubs_t *et;
    et_setup(&cpu, &rom, &et);

    const uint32_t reader = 0x4007D200u;
    const uint32_t addr = 0x4007D300u;
    const uint32_t config = 0x3FF60070u;

    ASSERT_EQ(esp_timer_stubs_hook_firmware(et), 0u);

    /* The complete inner body and its decoded register layout are both part
     * of authorization; an outer prefix or a plausible MMIO pointer alone is
     * insufficient. */
    uint32_t literals = seed_idf_lact_accessor(
        &cpu, reader, addr, config);
    mem_write8(cpu.mem, reader + 20u, 0u);
    ASSERT_EQ(esp_timer_stubs_hook_firmware(et), 0u);
    seed_idf_lact_accessor(&cpu, reader, addr, config);
    mem_write32(cpu.mem, literals + 4u, config + 0x24u);
    ASSERT_EQ(esp_timer_stubs_hook_firmware(et), 0u);

    seed_idf_lact_accessor(&cpu, reader, addr, config);
    put_insn3(&cpu, addr - 8u, 0x004136u);
    ASSERT_EQ(esp_timer_stubs_hook_firmware(et), 1u);
    ASSERT_EQ(esp_timer_stubs_hook_firmware(et), 0u);
    ASSERT_TRUE(cpu.accelerated_blocks);

    /* Structural entries use exact registration, so an ENTRY-shaped word in
     * preceding data cannot inherit the native hook. */
    cpu.pc = addr - 8u;
    cpu._pc_written = true;
    cpu.windowbase = 0u;
    cpu.windowstart = 1u;
    XT_PS_SET_CALLINC(cpu.ps, 0u);
    ar_write(&cpu, 1, 0x3FFB4000u);
    ASSERT_EQ(xtensa_step(&cpu), 0u);
    ASSERT_EQ(cpu.pc, addr - 5u);
    ASSERT_EQ(rom_stubs_total_calls(rom), 0u);

    /* Relocate the implementation and point it at timer group 1. Neither the
     * WLED link address nor a firmware profile participates. */
    cpu.ccount = 0u;
    cpu.cycle_count = 0u;
    cpu.virtual_time_us = 0u;
    cpu.insn_count = 0u;
    esp32_periph_t *periph = periph_create(cpu.mem);
    ASSERT_TRUE(periph != NULL);
    periph_attach_cpus(periph, &cpu, NULL);
    mem_write32(cpu.mem, 0x3FFE01E0u, 160u);
    mem_write32(cpu.mem, 0x3FF000C0u, 1u << 15);
    mem_write32(cpu.mem, config + 0x1Cu, 0u);
    mem_write32(cpu.mem, config + 0x20u, 0u);
    mem_write32(cpu.mem, config + 0x24u, 1u);
    mem_write32(cpu.mem, config,
                (1u << 31) | (1u << 30) | (40u << 13) | (1u << 11));
    cpu.pc = addr;
    cpu._pc_written = true;
    cpu.windowbase = 0u;
    cpu.windowstart = 1u;
    cpu.ccount = 160000u;
    cpu.cycle_count = 160000u;
    cpu.virtual_time_us = 500u;
    cpu.next_timer_event = UINT32_MAX;
    XT_PS_SET_CALLINC(cpu.ps, 2);
    ar_write(&cpu, 8, (2u << 30) | (BASE & 0x3FFFFFFFu));

    xtensa_step(&cpu);
    ASSERT_EQ(cpu.pc, BASE);
    ASSERT_EQ(ar_read(&cpu, 10), 1000u);
    ASSERT_EQ(ar_read(&cpu, 11), 0u);
    ASSERT_EQ(cpu.ccount, 160096u);
    ASSERT_EQ64(cpu.cycle_count, 160096u);
    ASSERT_EQ64(cpu.insn_count, 96u);

    /* A live window the nested calls could touch makes the hook decline. */
    cpu.pc = addr;
    cpu._pc_written = true;
    cpu.windowbase = 0u;
    cpu.windowstart = 1u | (1u << 6);
    cpu.ccount = 200000u;
    cpu.cycle_count = 200000u;
    cpu.next_timer_event = UINT32_MAX;
    cpu.insn_count = 0u;
    cpu.seed_entry_link = false;
    XT_PS_SET_CALLINC(cpu.ps, 2);
    ar_write(&cpu, 1, 0x3FFB4000u);
    xtensa_step(&cpu);
    ASSERT_EQ(cpu.pc, addr + 3u);
    ASSERT_EQ(cpu.ccount, 200001u);
    ASSERT_EQ64(cpu.insn_count, 1u);

    /* So does a timer event that belongs anywhere inside the native span. */
    cpu.pc = addr;
    cpu._pc_written = true;
    cpu.windowbase = 0u;
    cpu.windowstart = 1u;
    cpu.ccount = 300000u;
    cpu.cycle_count = 300000u;
    cpu.next_timer_event = 300033u;
    cpu.insn_count = 0u;
    XT_PS_SET_CALLINC(cpu.ps, 2);
    ar_write(&cpu, 1, 0x3FFB4000u);
    xtensa_step(&cpu);
    ASSERT_EQ(cpu.pc, addr + 3u);
    ASSERT_EQ(cpu.ccount, 300001u);
    ASSERT_EQ64(cpu.insn_count, 1u);

    /* A pending enabled interrupt must be taken after the first guest
     * instruction, not postponed until the collapsed function returns. */
    cpu.pc = addr;
    cpu._pc_written = true;
    cpu.windowbase = 0u;
    cpu.windowstart = 1u;
    cpu.ccount = 400000u;
    cpu.cycle_count = 400000u;
    cpu.next_timer_event = UINT32_MAX;
    cpu.insn_count = 0u;
    cpu.interrupt = 1u << 11;
    cpu.intenable = 1u << 11;
    cpu.irq_check = true;
    XT_PS_SET_CALLINC(cpu.ps, 2u);
    ar_write(&cpu, 1, 0x3FFB4000u);
    ar_write(&cpu, 8, (2u << 30) | (BASE & 0x3FFFFFFFu));
    ASSERT_EQ(xtensa_step(&cpu), (uint32_t)-1);
    ASSERT_TRUE(cpu.pc != BASE);
    ASSERT_EQ(cpu.ccount, 400001u);
    ASSERT_EQ64(cpu.insn_count, 1u);

    periph_destroy(periph);
    et_teardown(&cpu, rom, et);
}

static esp32_periph_t *init_idf_lact_call(xtensa_cpu_t *cpu,
                                          uint32_t entry,
                                          uint32_t config,
                                          uint32_t start_ccount,
                                          uint64_t initial_counter,
                                          unsigned windowbase) {
    esp32_periph_t *periph = periph_create(cpu->mem);
    if (!periph)
        return NULL;
    periph_attach_cpus(periph, cpu, NULL);
    mem_write32(cpu->mem, 0x3FFE01E0u, 160u);
    mem_write32(cpu->mem, 0x3FF000C0u, 1u << 15);
    mem_write32(cpu->mem, config + 0x1Cu, (uint32_t)initial_counter);
    mem_write32(cpu->mem, config + 0x20u,
                (uint32_t)(initial_counter >> 32));
    mem_write32(cpu->mem, config + 0x24u, 1u);
    mem_write32(cpu->mem, config,
                (1u << 31) | (1u << 30) | (40u << 13) | (1u << 11));

    for (unsigned i = 0u; i < 64u; i++)
        cpu->ar[i] = 0xA5000000u + i;
    cpu->pc = entry;
    cpu->_pc_written = true;
    cpu->windowbase = windowbase & 15u;
    cpu->windowstart = 1u << cpu->windowbase;
    cpu->ccount = start_ccount;
    cpu->cycle_count = start_ccount;
    cpu->insn_count = 0u;
    cpu->virtual_time_us = 500u;
    cpu->next_timer_event = UINT32_MAX;
    XT_PS_SET_CALLINC(cpu->ps, 2u);
    ar_write(cpu, 1, 0x3FFB4000u);
    ar_write(cpu, 8, (2u << 30) | (BASE & 0x3FFFFFFFu));
    return periph;
}

TEST(test_idf_lact_timer_native_path_matches_guest_execution) {
    const uint32_t reader = 0x4007D200u;
    const uint32_t entry = 0x4007D300u;
    const uint32_t config = 0x3FF60070u;
    static const struct {
        uint32_t ccount;
        uint64_t counter;
        unsigned windowbase;
    } cases[] = {
        {160000u, 0u, 0u},
        {160040u, 0u, 3u},
        {160060u, 0u, 13u},
        {160073u, 0u, 7u},
        {0u, 0x00000001FFFFFFFFull, 15u},
    };

    for (unsigned c = 0u; c < sizeof(cases) / sizeof(cases[0]); c++) {
        xtensa_cpu_t reference;
        xtensa_cpu_t accelerated;
        esp32_rom_stubs_t *reference_rom;
        esp32_rom_stubs_t *accelerated_rom;
        esp_timer_stubs_t *reference_et;
        esp_timer_stubs_t *accelerated_et;
        et_setup(&reference, &reference_rom, &reference_et);
        et_setup(&accelerated, &accelerated_rom, &accelerated_et);
        seed_idf_lact_accessor(&reference, reader, entry, config);
        seed_idf_lact_accessor(&accelerated, reader, entry, config);
        ASSERT_EQ(esp_timer_stubs_hook_firmware(accelerated_et), 1u);

        esp32_periph_t *reference_periph = init_idf_lact_call(
            &reference, entry, config, cases[c].ccount, cases[c].counter,
            cases[c].windowbase);
        esp32_periph_t *accelerated_periph = init_idf_lact_call(
            &accelerated, entry, config, cases[c].ccount, cases[c].counter,
            cases[c].windowbase);
        ASSERT_TRUE(reference_periph != NULL);
        ASSERT_TRUE(accelerated_periph != NULL);

        for (unsigned step = 0u;
             step < 200u && reference.pc != BASE; step++)
            ASSERT_EQ(xtensa_step(&reference), 0u);
        ASSERT_EQ(reference.pc, BASE);
        ASSERT_EQ(xtensa_step(&accelerated), 0u);

        ASSERT_EQ(accelerated.pc, reference.pc);
        ASSERT_EQ(accelerated.ps, reference.ps);
        ASSERT_EQ(accelerated.windowbase, reference.windowbase);
        ASSERT_EQ(accelerated.windowstart, reference.windowstart);
        ASSERT_EQ(accelerated.ccount, reference.ccount);
        ASSERT_EQ64(accelerated.cycle_count, reference.cycle_count);
        ASSERT_EQ64(accelerated.insn_count, reference.insn_count);
        ASSERT_EQ(accelerated.sar, reference.sar);
        ASSERT_EQ(accelerated.lbeg, reference.lbeg);
        ASSERT_EQ(accelerated.lend, reference.lend);
        ASSERT_EQ(accelerated.lcount, reference.lcount);
        for (unsigned i = 0u; i < 64u; i++)
            ASSERT_EQ(accelerated.ar[i], reference.ar[i]);
        for (unsigned i = 0u; i < 16u; i++)
            ASSERT_EQ(accelerated.window_callsize[i],
                      reference.window_callsize[i]);

        periph_destroy(accelerated_periph);
        periph_destroy(reference_periph);
        et_teardown(&accelerated, accelerated_rom, accelerated_et);
        et_teardown(&reference, reference_rom, reference_et);
    }
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

TEST(test_delay_advances_virtual_time_and_ccount_at_runtime_frequency) {
    xtensa_cpu_t cpu;
    esp32_rom_stubs_t *rom;
    esp_timer_stubs_t *et;
    et_setup(&cpu, &rom, &et);

    uint32_t delay_addr = 0x400D1060;
    rom_stubs_register_ctx(rom, delay_addr, (rom_stub_fn)stub_delay,
                           "delay", et);
    mem_write32(cpu.mem, 0x3FFE01E0u, 240u);
    uint32_t ccount_before = cpu.ccount;
    uint64_t virtual_before = cpu.virtual_time_us;
    XT_PS_SET_CALLINC(cpu.ps, 0);
    ar_write(&cpu, 0, BASE + 0x100);
    ar_write(&cpu, 2, 3u);
    cpu.pc = delay_addr;
    xtensa_step(&cpu);

    ASSERT_EQ(cpu.virtual_time_us - virtual_before, 3000u);
    ASSERT_EQ(cpu.ccount - ccount_before, 720001u);
    ASSERT_EQ(cpu.pc, BASE + 0x100);

    et_teardown(&cpu, rom, et);
}

static void run_esp_timer_tests(void) {
    TEST_SUITE("esp_timer_stubs");
    RUN_TEST(test_esp_timer_create_returns_ok);
    RUN_TEST(test_esp_timer_start_stop);
    RUN_TEST(test_esp_timer_delete);
    RUN_TEST(test_esp_timer_get_time);
    RUN_TEST(test_stripped_idf_lact_timer_is_discovered_structurally);
    RUN_TEST(test_idf_lact_timer_native_path_matches_guest_execution);
    RUN_TEST(test_esp_timer_start_once);
    RUN_TEST(test_esp_timer_multiple_creates);
    RUN_TEST(test_delay_advances_virtual_time_and_ccount_at_runtime_frequency);
}
