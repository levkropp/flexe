/*
 * FreeRTOS stub tests
 */
#include "freertos_stubs.h"
#include "rom_stubs.h"
#include "peripherals.h"

/* ===== Test helpers: set up a full emulator context for stub testing ===== */

static void frt_setup(xtensa_cpu_t *cpu, esp32_rom_stubs_t **rom_out, freertos_stubs_t **frt_out) {
    xtensa_cpu_init(cpu);
    cpu->mem = mem_create();
    cpu->pc = BASE;
    *rom_out = rom_stubs_create(cpu);
    *frt_out = freertos_stubs_create(cpu);
}

static void frt_teardown(xtensa_cpu_t *cpu, esp32_rom_stubs_t *rom, freertos_stubs_t *frt) {
    freertos_stubs_destroy(frt);
    rom_stubs_destroy(rom);
    mem_destroy(cpu->mem);
}

/* ===== Tests ===== */

TEST(test_vTaskDelay_advances_ccount) {
    xtensa_cpu_t cpu;
    esp32_rom_stubs_t *rom;
    freertos_stubs_t *frt;
    frt_setup(&cpu, &rom, &frt);

    /* Register vTaskDelay at a known address */
    uint32_t vtd_addr = 0x400D0000;
    extern void stub_vTaskDelay(xtensa_cpu_t *, void *);
    /* We use the rom_stubs_register to hook the address */
    rom_stubs_register_ctx(rom, vtd_addr, (rom_stub_fn)stub_vTaskDelay, "vTaskDelay", frt);

    uint64_t vtime_before = cpu.virtual_time_us;
    uint32_t ccount_before = cpu.ccount;
    /* Set up call: a2 = ticks = 10 */
    XT_PS_SET_CALLINC(cpu.ps, 0);
    ar_write(&cpu, 0, BASE + 0x100);
    ar_write(&cpu, 2, 10);  /* 10 ticks */
    cpu.pc = vtd_addr;

    /* Step to trigger the PC hook */
    xtensa_step(&cpu);

    /* Arduino-ESP32 uses a 1 kHz tick: ten ticks are ten milliseconds. */
    ASSERT_EQ(cpu.virtual_time_us - vtime_before, 10000);
    /* Fast-forwarded wall time advances the architectural cycle counter;
     * xtensa_step charges one additional cycle for dispatching the stub. */
    ASSERT_EQ(cpu.ccount - ccount_before, 1600001u);
    /* PC should have returned */
    ASSERT_EQ(cpu.pc, BASE + 0x100);

    frt_teardown(&cpu, rom, frt);
}

TEST(test_xTaskCreate_returns_pdPASS) {
    xtensa_cpu_t cpu;
    esp32_rom_stubs_t *rom;
    freertos_stubs_t *frt;
    frt_setup(&cpu, &rom, &frt);

    uint32_t xtc_addr = 0x400D0010;
    extern void stub_xTaskCreate(xtensa_cpu_t *, void *);
    rom_stubs_register_ctx(rom, xtc_addr, (rom_stub_fn)stub_xTaskCreate, "xTaskCreate", frt);

    /* xTaskCreate(func, name, stack, param, prio, handle_out) */
    /* With CALL0 convention, args at a2..a7 */
    XT_PS_SET_CALLINC(cpu.ps, 0);
    ar_write(&cpu, 0, BASE + 0x100);
    ar_write(&cpu, 2, 0x400D1000); /* func */
    ar_write(&cpu, 3, 0);          /* name */
    ar_write(&cpu, 4, 2048);       /* stack */
    ar_write(&cpu, 5, 0);          /* param */
    ar_write(&cpu, 6, 1);          /* prio */

    /* handle_out at a7 (arg index 5) */
    uint32_t handle_out_addr = 0x3FFB1000;
    mem_write32(cpu.mem, handle_out_addr, 0);
    ar_write(&cpu, 7, handle_out_addr);

    cpu.pc = xtc_addr;
    xtensa_step(&cpu);

    /* Return value in a2 should be pdPASS (1) */
    ASSERT_EQ(ar_read(&cpu, 2), 1);
    /* handle_out should be non-zero */
    ASSERT_TRUE(mem_read32(cpu.mem, handle_out_addr) != 0);

    frt_teardown(&cpu, rom, frt);
}

TEST(test_pinned_task_reads_core_affinity_from_windowed_stack) {
    xtensa_cpu_t cpu0, cpu1;
    esp32_rom_stubs_t *rom;
    freertos_stubs_t *frt;
    frt_setup(&cpu0, &rom, &frt);
    xtensa_cpu_init(&cpu1);
    cpu1.mem = cpu0.mem;
    cpu1.core_id = 1;
    freertos_stubs_attach_cpu(frt, 1, &cpu1);

    const uint32_t entry = 0x400D2000u;
    const uint32_t name_addr = 0x3FFB2000u;
    const uint32_t handle_addr = 0x3FFB2040u;
    const uint32_t caller_sp = 0x3FFB2200u;
    static const char name[] = "pinned1";
    static const char dummy_name[] = "dummy";
    for (size_t i = 0; i < sizeof(name); i++)
        mem_write8(cpu0.mem, name_addr + (uint32_t)i, (uint8_t)name[i]);
    for (size_t i = 0; i < sizeof(dummy_name); i++)
        mem_write8(cpu0.mem, name_addr + 0x20u + (uint32_t)i,
                   (uint8_t)dummy_name[i]);
    mem_write32(cpu0.mem, handle_addr, 0u);

    /* A first unpinned task represents the legacy task already executing on
     * PRO_CPU.  Registering the second task promotes the compatibility
     * scheduler and leaves the pinned task ready for APP_CPU. */
    extern void stub_xTaskCreate(xtensa_cpu_t *, void *);
    XT_PS_SET_CALLINC(cpu0.ps, 0);
    ar_write(&cpu0, 0, BASE + 0x100u);
    ar_write(&cpu0, 2, 0x400D1000u);
    ar_write(&cpu0, 3, name_addr + 0x20u);
    ar_write(&cpu0, 4, 2048u);
    ar_write(&cpu0, 5, 0u);
    ar_write(&cpu0, 6, 1u);
    ar_write(&cpu0, 7, 0u);
    stub_xTaskCreate(&cpu0, frt);

    /* CALL8 supplies the six register arguments in a10..a15.  The seventh
     * argument is at caller_sp; poison a16 with core 0 so the regression
     * fails if the stub reads past the ABI's argument-register bank. */
    XT_PS_SET_CALLINC(cpu0.ps, 2);
    ar_write(&cpu0, 1, caller_sp);
    ar_write(&cpu0, 8, BASE + 0x180u);
    ar_write(&cpu0, 10, entry);
    ar_write(&cpu0, 11, name_addr);
    ar_write(&cpu0, 12, 2048u);
    ar_write(&cpu0, 13, 0u);
    ar_write(&cpu0, 14, 3u);
    ar_write(&cpu0, 15, handle_addr);
    ar_write(&cpu0, 16, 0u);
    mem_write32(cpu0.mem, caller_sp, 1u);

    extern void stub_xTaskCreatePinnedToCore(xtensa_cpu_t *, void *);
    stub_xTaskCreatePinnedToCore(&cpu0, frt);
    ASSERT_EQ(ar_read(&cpu0, 10), 1u);
    ASSERT_TRUE(mem_read32(cpu0.mem, handle_addr) != 0u);

    ASSERT_TRUE(freertos_stubs_scheduler_active(frt));
    ASSERT_TRUE(strcmp(freertos_stubs_current_task_name(frt, 0),
                       dummy_name) == 0);
    ASSERT_TRUE(freertos_stubs_check_preempt_core(frt, 1));
    ASSERT_TRUE(strcmp(freertos_stubs_current_task_name(frt, 1), name) == 0);
    ASSERT_EQ(cpu1.pc, entry);

    frt_teardown(&cpu0, rom, frt);
}

TEST(test_xTaskGetTickCount) {
    xtensa_cpu_t cpu;
    esp32_rom_stubs_t *rom;
    freertos_stubs_t *frt;
    frt_setup(&cpu, &rom, &frt);

    uint32_t addr = 0x400D0020;
    extern void stub_xTaskGetTickCount(xtensa_cpu_t *, void *);
    rom_stubs_register_ctx(rom, addr, (rom_stub_fn)stub_xTaskGetTickCount, "xTaskGetTickCount", frt);

    cpu.ccount = 1600000;  /* 10 ticks at 160K cycles per tick */
    XT_PS_SET_CALLINC(cpu.ps, 0);
    ar_write(&cpu, 0, BASE + 0x100);
    cpu.pc = addr;
    xtensa_step(&cpu);

    ASSERT_EQ(ar_read(&cpu, 2), 10);

    frt_teardown(&cpu, rom, frt);
}

TEST(test_scheduler_start_sets_dual_core_ready_flags) {
    xtensa_cpu_t cpu0, cpu1;
    esp32_rom_stubs_t *rom;
    freertos_stubs_t *frt;
    frt_setup(&cpu0, &rom, &frt);
    xtensa_cpu_init(&cpu1);
    cpu1.mem = cpu0.mem;
    cpu1.core_id = 1;
    freertos_stubs_attach_cpu(frt, 1, &cpu1);

    const char *path = build_test_elf();
    elf_symbols_t *syms = elf_symbols_load(path);
    ASSERT_TRUE(syms != NULL);
    ASSERT_TRUE(freertos_stubs_hook_symbols(frt, syms) >= 2);

    const uint32_t startup_done = 0x3FFB0100u;
    mem_write32(cpu0.mem, startup_done, 0u);
    mem_write32(cpu0.mem, startup_done + 4u, 0u);
    XT_PS_SET_CALLINC(cpu0.ps, 0);
    ar_write(&cpu0, 0, BASE + 0x100u);
    cpu0.pc = 0x40080200u;
    xtensa_step(&cpu0);

    ASSERT_TRUE(freertos_stubs_scheduler_active(frt));
    ASSERT_EQ(mem_read32(cpu0.mem, startup_done), 1u);
    ASSERT_EQ(mem_read32(cpu0.mem, startup_done + 4u), 1u);
    ASSERT_EQ(rom_stubs_unregistered_count(rom), 0);

    elf_symbols_destroy(syms);
    frt_teardown(&cpu0, rom, frt);
}

TEST(test_queue_send_receive) {
    xtensa_cpu_t cpu;
    esp32_rom_stubs_t *rom;
    freertos_stubs_t *frt;
    frt_setup(&cpu, &rom, &frt);

    uint32_t create_addr = 0x400D0030;
    uint32_t send_addr = 0x400D0040;
    uint32_t recv_addr = 0x400D0050;
    extern void stub_xQueueCreate(xtensa_cpu_t *, void *);
    extern void stub_xQueueSend(xtensa_cpu_t *, void *);
    extern void stub_xQueueReceive(xtensa_cpu_t *, void *);
    extern void stub_xQueueReceiveFromISR(xtensa_cpu_t *, void *);
    extern void stub_xQueueIsQueueFullFromISR(xtensa_cpu_t *, void *);
    rom_stubs_register_ctx(rom, create_addr, (rom_stub_fn)stub_xQueueCreate, "xQueueCreate", frt);
    rom_stubs_register_ctx(rom, send_addr, (rom_stub_fn)stub_xQueueSend, "xQueueSend", frt);
    rom_stubs_register_ctx(rom, recv_addr, (rom_stub_fn)stub_xQueueReceive, "xQueueReceive", frt);
    rom_stubs_register_ctx(rom, 0x400D0060, stub_xQueueReceiveFromISR,
                           "xQueueReceiveFromISR", frt);
    rom_stubs_register_ctx(rom, 0x400D0070, stub_xQueueIsQueueFullFromISR,
                           "xQueueIsQueueFullFromISR", frt);

    /* Create queue: length=4, item_size=4 */
    XT_PS_SET_CALLINC(cpu.ps, 0);
    ar_write(&cpu, 0, BASE + 0x100);
    ar_write(&cpu, 2, 4);  /* length */
    ar_write(&cpu, 3, 4);  /* item_size */
    cpu.pc = create_addr;
    xtensa_step(&cpu);
    uint32_t handle = ar_read(&cpu, 2);
    ASSERT_TRUE(handle != 0);

    /* Send item: value = 0xDEADBEEF */
    uint32_t item_addr = 0x3FFB2000;
    mem_write32(cpu.mem, item_addr, 0xDEADBEEF);
    XT_PS_SET_CALLINC(cpu.ps, 0);
    ar_write(&cpu, 0, BASE + 0x100);
    ar_write(&cpu, 2, handle);
    ar_write(&cpu, 3, item_addr);
    ar_write(&cpu, 4, 0);  /* timeout */
    cpu.pc = send_addr;
    xtensa_step(&cpu);
    ASSERT_EQ(ar_read(&cpu, 2), 1);  /* pdTRUE */

    /* Receive item */
    uint32_t recv_buf = 0x3FFB2010;
    mem_write32(cpu.mem, recv_buf, 0);
    XT_PS_SET_CALLINC(cpu.ps, 0);
    ar_write(&cpu, 0, BASE + 0x100);
    ar_write(&cpu, 2, handle);
    ar_write(&cpu, 3, recv_buf);
    ar_write(&cpu, 4, 0);  /* timeout */
    cpu.pc = recv_addr;
    xtensa_step(&cpu);
    ASSERT_EQ(ar_read(&cpu, 2), 1);  /* pdTRUE */
    ASSERT_EQ(mem_read32(cpu.mem, recv_buf), 0xDEADBEEF);

    /* The I2S ISR checks fullness before replacing the oldest descriptor. */
    ar_write(&cpu, 0, BASE + 0x100);
    ar_write(&cpu, 2, handle);
    cpu.pc = 0x400D0070;
    xtensa_step(&cpu);
    ASSERT_EQ(ar_read(&cpu, 2), 0);

    mem_write32(cpu.mem, item_addr, 0x12345678u);
    ar_write(&cpu, 0, BASE + 0x100);
    ar_write(&cpu, 2, handle);
    ar_write(&cpu, 3, item_addr);
    ar_write(&cpu, 4, 0);
    cpu.pc = send_addr;
    xtensa_step(&cpu);
    ar_write(&cpu, 0, BASE + 0x100);
    ar_write(&cpu, 2, handle);
    cpu.pc = 0x400D0070;
    xtensa_step(&cpu);
    ASSERT_EQ(ar_read(&cpu, 2), 0); /* capacity is four */
    uint32_t woken = 0x3FFB2020u;
    mem_write32(cpu.mem, woken, 0xFFFFFFFFu);
    ar_write(&cpu, 0, BASE + 0x100);
    ar_write(&cpu, 2, handle);
    ar_write(&cpu, 3, recv_buf);
    ar_write(&cpu, 4, woken);
    cpu.pc = 0x400D0060;
    xtensa_step(&cpu);
    ASSERT_EQ(ar_read(&cpu, 2), 1);
    ASSERT_EQ(mem_read32(cpu.mem, recv_buf), 0x12345678u);
    ASSERT_EQ(mem_read32(cpu.mem, woken), 0);

    for (uint32_t i = 0; i < 4; ++i) {
        mem_write32(cpu.mem, item_addr, i);
        ar_write(&cpu, 0, BASE + 0x100);
        ar_write(&cpu, 2, handle);
        ar_write(&cpu, 3, item_addr);
        ar_write(&cpu, 4, 0);
        cpu.pc = send_addr;
        xtensa_step(&cpu);
        ASSERT_EQ(ar_read(&cpu, 2), 1);
    }
    ar_write(&cpu, 0, BASE + 0x100);
    ar_write(&cpu, 2, handle);
    cpu.pc = 0x400D0070;
    xtensa_step(&cpu);
    ASSERT_EQ(ar_read(&cpu, 2), 1);

    frt_teardown(&cpu, rom, frt);
}

TEST(test_queue_receive_empty_returns_false) {
    xtensa_cpu_t cpu;
    esp32_rom_stubs_t *rom;
    freertos_stubs_t *frt;
    frt_setup(&cpu, &rom, &frt);

    uint32_t create_addr = 0x400D0030;
    uint32_t recv_addr = 0x400D0050;
    extern void stub_xQueueCreate(xtensa_cpu_t *, void *);
    extern void stub_xQueueReceive(xtensa_cpu_t *, void *);
    rom_stubs_register_ctx(rom, create_addr, (rom_stub_fn)stub_xQueueCreate, "xQueueCreate", frt);
    rom_stubs_register_ctx(rom, recv_addr, (rom_stub_fn)stub_xQueueReceive, "xQueueReceive", frt);

    /* Create queue */
    XT_PS_SET_CALLINC(cpu.ps, 0);
    ar_write(&cpu, 0, BASE + 0x100);
    ar_write(&cpu, 2, 4);
    ar_write(&cpu, 3, 4);
    cpu.pc = create_addr;
    xtensa_step(&cpu);
    uint32_t handle = ar_read(&cpu, 2);

    /* Receive from empty queue with timeout=10 */
    uint32_t recv_buf = 0x3FFB2010;
    uint64_t vtime_before = cpu.virtual_time_us;
    XT_PS_SET_CALLINC(cpu.ps, 0);
    ar_write(&cpu, 0, BASE + 0x100);
    ar_write(&cpu, 2, handle);
    ar_write(&cpu, 3, recv_buf);
    ar_write(&cpu, 4, 10);  /* timeout = 10 ticks */
    cpu.pc = recv_addr;
    xtensa_step(&cpu);
    ASSERT_EQ(ar_read(&cpu, 2), 0);  /* pdFALSE */
    /* Ten 1 kHz ticks advance virtual time by ten milliseconds. */
    ASSERT_EQ(cpu.virtual_time_us - vtime_before, 10000);

    frt_teardown(&cpu, rom, frt);
}

TEST(test_queue_overwrite_and_reset) {
    xtensa_cpu_t cpu;
    esp32_rom_stubs_t *rom;
    freertos_stubs_t *frt;
    frt_setup(&cpu, &rom, &frt);

    uint32_t create_addr = 0x400D0030;
    uint32_t send_addr = 0x400D0040;
    uint32_t recv_addr = 0x400D0050;
    uint32_t reset_addr = 0x400D0060;
    extern void stub_xQueueCreate(xtensa_cpu_t *, void *);
    extern void stub_xQueueGenericSendFromISR(xtensa_cpu_t *, void *);
    extern void stub_xQueueReceive(xtensa_cpu_t *, void *);
    extern void stub_xQueueGenericReset(xtensa_cpu_t *, void *);
    rom_stubs_register_ctx(rom, create_addr, stub_xQueueCreate,
                           "xQueueCreate", frt);
    rom_stubs_register_ctx(rom, send_addr, stub_xQueueGenericSendFromISR,
                           "xQueueGenericSendFromISR", frt);
    rom_stubs_register_ctx(rom, recv_addr, stub_xQueueReceive,
                           "xQueueReceive", frt);
    rom_stubs_register_ctx(rom, reset_addr, stub_xQueueGenericReset,
                           "xQueueGenericReset", frt);

    /* The IDF I2C completion queue has length one. Its ISR first publishes
     * ALIVE, then overwrites that entry with DONE before the task runs. */
    XT_PS_SET_CALLINC(cpu.ps, 0);
    ar_write(&cpu, 0, BASE + 0x100);
    ar_write(&cpu, 2, 1);
    ar_write(&cpu, 3, 4);
    cpu.pc = create_addr;
    xtensa_step(&cpu);
    uint32_t handle = ar_read(&cpu, 2);
    ASSERT_TRUE(handle != 0);

    uint32_t item_addr = 0x3FFB2000;
    mem_write32(cpu.mem, item_addr, 0x11111111u);
    ar_write(&cpu, 0, BASE + 0x100);
    ar_write(&cpu, 2, handle);
    ar_write(&cpu, 3, item_addr);
    ar_write(&cpu, 4, 0); /* pxHigherPriorityTaskWoken */
    ar_write(&cpu, 5, 0); /* queueSEND_TO_BACK */
    cpu.pc = send_addr;
    xtensa_step(&cpu);
    ASSERT_EQ(ar_read(&cpu, 2), 1);

    mem_write32(cpu.mem, item_addr, 0x22222222u);
    ar_write(&cpu, 0, BASE + 0x100);
    ar_write(&cpu, 2, handle);
    ar_write(&cpu, 3, item_addr);
    ar_write(&cpu, 4, 0);
    ar_write(&cpu, 5, 2); /* queueOVERWRITE */
    cpu.pc = send_addr;
    xtensa_step(&cpu);
    ASSERT_EQ(ar_read(&cpu, 2), 1);

    uint32_t recv_buf = 0x3FFB2010;
    ar_write(&cpu, 0, BASE + 0x100);
    ar_write(&cpu, 2, handle);
    ar_write(&cpu, 3, recv_buf);
    ar_write(&cpu, 4, 0);
    cpu.pc = recv_addr;
    xtensa_step(&cpu);
    ASSERT_EQ(ar_read(&cpu, 2), 1);
    ASSERT_EQ(mem_read32(cpu.mem, recv_buf), 0x22222222u);

    /* Reset discards a queued entry, matching xQueueReset between IDF bus
     * transactions. */
    mem_write32(cpu.mem, item_addr, 0x33333333u);
    ar_write(&cpu, 0, BASE + 0x100);
    ar_write(&cpu, 2, handle);
    ar_write(&cpu, 3, item_addr);
    ar_write(&cpu, 4, 0);
    ar_write(&cpu, 5, 0);
    cpu.pc = send_addr;
    xtensa_step(&cpu);

    ar_write(&cpu, 0, BASE + 0x100);
    ar_write(&cpu, 2, handle);
    ar_write(&cpu, 3, 0);
    cpu.pc = reset_addr;
    xtensa_step(&cpu);
    ASSERT_EQ(ar_read(&cpu, 2), 1);

    ar_write(&cpu, 0, BASE + 0x100);
    ar_write(&cpu, 2, handle);
    ar_write(&cpu, 3, recv_buf);
    ar_write(&cpu, 4, 0);
    cpu.pc = recv_addr;
    xtensa_step(&cpu);
    ASSERT_EQ(ar_read(&cpu, 2), 0);

    frt_teardown(&cpu, rom, frt);
}

TEST(test_semaphore_create_take_give) {
    xtensa_cpu_t cpu;
    esp32_rom_stubs_t *rom;
    freertos_stubs_t *frt;
    frt_setup(&cpu, &rom, &frt);

    uint32_t create_addr = 0x400D0060;
    uint32_t binary_addr = 0x400D0064;
    uint32_t take_addr = 0x400D0070;
    uint32_t give_addr = 0x400D0080;
    extern void stub_xSemaphoreCreateMutex(xtensa_cpu_t *, void *);
    extern void stub_xSemaphoreCreateBinary(xtensa_cpu_t *, void *);
    extern void stub_xSemaphoreTake(xtensa_cpu_t *, void *);
    extern void stub_xSemaphoreGive(xtensa_cpu_t *, void *);
    rom_stubs_register_ctx(rom, create_addr, (rom_stub_fn)stub_xSemaphoreCreateMutex, "xSemaphoreCreateMutex", frt);
    rom_stubs_register_ctx(rom, binary_addr, (rom_stub_fn)stub_xSemaphoreCreateBinary, "xSemaphoreCreateBinary", frt);
    rom_stubs_register_ctx(rom, take_addr, (rom_stub_fn)stub_xSemaphoreTake, "xSemaphoreTake", frt);
    rom_stubs_register_ctx(rom, give_addr, (rom_stub_fn)stub_xSemaphoreGive, "xSemaphoreGive", frt);

    /* Create */
    XT_PS_SET_CALLINC(cpu.ps, 0);
    ar_write(&cpu, 0, BASE + 0x100);
    cpu.pc = create_addr;
    xtensa_step(&cpu);
    uint32_t handle = ar_read(&cpu, 2);
    ASSERT_TRUE(handle != 0);

    /* Take */
    XT_PS_SET_CALLINC(cpu.ps, 0);
    ar_write(&cpu, 0, BASE + 0x100);
    ar_write(&cpu, 2, handle);
    ar_write(&cpu, 3, 0);
    cpu.pc = take_addr;
    xtensa_step(&cpu);
    ASSERT_EQ(ar_read(&cpu, 2), 1);  /* pdTRUE */

    /* Give */
    XT_PS_SET_CALLINC(cpu.ps, 0);
    ar_write(&cpu, 0, BASE + 0x100);
    ar_write(&cpu, 2, handle);
    cpu.pc = give_addr;
    xtensa_step(&cpu);
    ASSERT_EQ(ar_read(&cpu, 2), 1);  /* pdTRUE */

    /* Binary semaphores begin empty and consume exactly one give token. */
    ar_write(&cpu, 0, BASE + 0x100);
    cpu.pc = binary_addr;
    xtensa_step(&cpu);
    uint32_t binary = ar_read(&cpu, 2);
    ASSERT_TRUE(binary != 0);

    ar_write(&cpu, 0, BASE + 0x100);
    ar_write(&cpu, 2, binary);
    ar_write(&cpu, 3, 0);
    cpu.pc = take_addr;
    xtensa_step(&cpu);
    ASSERT_EQ(ar_read(&cpu, 2), 0);

    ar_write(&cpu, 0, BASE + 0x100);
    ar_write(&cpu, 2, binary);
    cpu.pc = give_addr;
    xtensa_step(&cpu);
    ASSERT_EQ(ar_read(&cpu, 2), 1);

    for (int expected = 1; expected >= 0; expected--) {
        ar_write(&cpu, 0, BASE + 0x100);
        ar_write(&cpu, 2, binary);
        ar_write(&cpu, 3, 0);
        cpu.pc = take_addr;
        xtensa_step(&cpu);
        ASSERT_EQ(ar_read(&cpu, 2), (uint32_t)expected);
    }

    frt_teardown(&cpu, rom, frt);
}

TEST(test_counting_semaphore_supports_idf_unbounded_capacity) {
    xtensa_cpu_t cpu;
    esp32_rom_stubs_t *rom;
    freertos_stubs_t *frt;
    frt_setup(&cpu, &rom, &frt);

    uint32_t create_addr = 0x400D0060;
    uint32_t take_addr = 0x400D0070;
    uint32_t give_addr = 0x400D0080;
    extern void stub_xQueueCreateCountingSemaphore(xtensa_cpu_t *, void *);
    extern void stub_xSemaphoreTake(xtensa_cpu_t *, void *);
    extern void stub_xSemaphoreGive(xtensa_cpu_t *, void *);
    rom_stubs_register_ctx(rom, create_addr,
                           stub_xQueueCreateCountingSemaphore,
                           "xQueueCreateCountingSemaphore", frt);
    rom_stubs_register_ctx(rom, take_addr, stub_xSemaphoreTake,
                           "xSemaphoreTake", frt);
    rom_stubs_register_ctx(rom, give_addr, stub_xSemaphoreGive,
                           "xSemaphoreGive", frt);

    XT_PS_SET_CALLINC(cpu.ps, 0);
    ar_write(&cpu, 0, BASE + 0x100);
    ar_write(&cpu, 2, UINT32_MAX);
    ar_write(&cpu, 3, 2u);
    cpu.pc = create_addr;
    xtensa_step(&cpu);
    uint32_t handle = ar_read(&cpu, 2);
    ASSERT_TRUE(handle != 0u);

    for (unsigned expected = 1u; expected <= 3u; expected++) {
        ar_write(&cpu, 0, BASE + 0x100);
        ar_write(&cpu, 2, handle);
        ar_write(&cpu, 3, 0u);
        cpu.pc = take_addr;
        xtensa_step(&cpu);
        ASSERT_EQ(ar_read(&cpu, 2), expected < 3u ? 1u : 0u);
    }

    ar_write(&cpu, 0, BASE + 0x100);
    ar_write(&cpu, 2, handle);
    cpu.pc = give_addr;
    xtensa_step(&cpu);
    ASSERT_EQ(ar_read(&cpu, 2), 1u);
    ar_write(&cpu, 0, BASE + 0x100);
    ar_write(&cpu, 2, handle);
    ar_write(&cpu, 3, 0u);
    cpu.pc = take_addr;
    xtensa_step(&cpu);
    ASSERT_EQ(ar_read(&cpu, 2), 1u);

    frt_teardown(&cpu, rom, frt);
}

static uint32_t frt_create_sched_task(xtensa_cpu_t *cpu,
                                      freertos_stubs_t *frt,
                                      uint32_t entry, const char *name,
                                      uint32_t priority,
                                      uint32_t name_addr,
                                      uint32_t handle_addr) {
    extern void stub_xTaskCreate(xtensa_cpu_t *, void *);
    for (size_t index = 0; index <= strlen(name); index++)
        mem_write8(cpu->mem, name_addr + (uint32_t)index,
                   (uint8_t)name[index]);
    mem_write32(cpu->mem, handle_addr, 0u);
    XT_PS_SET_CALLINC(cpu->ps, 0);
    ar_write(cpu, 0, BASE + 0x180u);
    ar_write(cpu, 2, entry);
    ar_write(cpu, 3, name_addr);
    ar_write(cpu, 4, 2048u);
    ar_write(cpu, 5, 0u);
    ar_write(cpu, 6, priority);
    ar_write(cpu, 7, handle_addr);
    cpu->pc = BASE + 0x40u;
    stub_xTaskCreate(cpu, frt);
    ASSERT_EQ(ar_read(cpu, 2), 1u);
    return mem_read32(cpu->mem, handle_addr);
}

TEST(test_task_notify_isr_preempts_and_preserves_window_context) {
    xtensa_cpu_t cpu;
    esp32_rom_stubs_t *rom;
    freertos_stubs_t *frt;
    frt_setup(&cpu, &rom, &frt);

    const uint32_t low_entry = 0x400D3000u;
    const uint32_t high_entry = 0x400D4000u;
    uint32_t low_handle = frt_create_sched_task(
        &cpu, frt, low_entry, "low", 1u, 0x3FFB3000u, 0x3FFB3040u);
    uint32_t high_handle = frt_create_sched_task(
        &cpu, frt, high_entry, "high", 5u, 0x3FFB3020u, 0x3FFB3044u);
    ASSERT_TRUE(low_handle != 0u);
    ASSERT_TRUE(high_handle != 0u);
    ASSERT_TRUE(freertos_stubs_scheduler_active(frt));
    ASSERT_TRUE(strcmp(freertos_stubs_current_task_name(frt, 0), "low") == 0);

    /* Preserve data in the part of the 32-deep spill history that the old
     * eight-entry TCB copy could not hold.  Saving this task used to overwrite
     * the following TCB before the higher-priority task was restored. */
    cpu.pc = low_entry + 0x20u;
    cpu.spill_stack[3].depth = SPILL_STACK_DEPTH;
    cpu.spill_stack[3].base[SPILL_STACK_DEPTH - 1] = 0x3FFBEE00u;
    cpu.spill_stack[3].core[SPILL_STACK_DEPTH - 1][2] = 0x12345678u;
    cpu.spill_stack[3].extra[SPILL_STACK_DEPTH - 1][7] = 0x89ABCDEFu;
    cpu.spill_shadow[3].regs[11] = 0xCAFEBABEu;
    cpu.spill_shadow[3].base = 0x3FFBDD00u;
    cpu.spill_shadow[3].count = 12;
    cpu.window_callsize[3] = 3u;

    /* Higher priorities preempt immediately; a tick is required only for an
     * equal-priority round-robin switch. */
    ASSERT_TRUE(freertos_stubs_check_preempt(frt));
    ASSERT_TRUE(strcmp(freertos_stubs_current_task_name(frt, 0), "high") == 0);
    ASSERT_EQ(cpu.pc, high_entry);

    extern void stub_ulTaskGenericNotifyTake(xtensa_cpu_t *, void *);
    XT_PS_SET_CALLINC(cpu.ps, 0);
    ar_write(&cpu, 0, 0x400D4100u);
    ar_write(&cpu, 2, 0u);          /* notification slot */
    ar_write(&cpu, 3, 1u);          /* clear count on exit */
    ar_write(&cpu, 4, UINT32_MAX);  /* wait indefinitely */
    cpu.pc = 0x400D4080u;
    stub_ulTaskGenericNotifyTake(&cpu, frt);
    ASSERT_TRUE(strcmp(freertos_stubs_current_task_name(frt, 0), "low") == 0);
    ASSERT_EQ(cpu.pc, low_entry + 0x20u);
    ASSERT_EQ(cpu.spill_stack[3].depth, SPILL_STACK_DEPTH);
    ASSERT_EQ(cpu.spill_stack[3].base[SPILL_STACK_DEPTH - 1], 0x3FFBEE00u);
    ASSERT_EQ(cpu.spill_stack[3].core[SPILL_STACK_DEPTH - 1][2], 0x12345678u);
    ASSERT_EQ(cpu.spill_stack[3].extra[SPILL_STACK_DEPTH - 1][7], 0x89ABCDEFu);
    ASSERT_EQ(cpu.spill_shadow[3].regs[11], 0xCAFEBABEu);
    ASSERT_EQ(cpu.spill_shadow[3].base, 0x3FFBDD00u);
    ASSERT_EQ(cpu.spill_shadow[3].count, 12);
    ASSERT_EQ(cpu.window_callsize[3], 3u);

    extern void stub_vTaskGenericNotifyGiveFromISR(xtensa_cpu_t *, void *);
    uint32_t woken_addr = 0x3FFB3050u;
    mem_write32(cpu.mem, woken_addr, 0u);
    XT_PS_SET_CALLINC(cpu.ps, 0);
    ar_write(&cpu, 0, 0x400D3100u);
    ar_write(&cpu, 2, high_handle);
    ar_write(&cpu, 3, 0u);
    ar_write(&cpu, 4, woken_addr);
    cpu.pc = 0x400D3080u;
    stub_vTaskGenericNotifyGiveFromISR(&cpu, frt);
    ASSERT_EQ(mem_read32(cpu.mem, woken_addr), 1u);

    /* The unblocked task must preempt even though zero cycles of the low
     * priority task's current slice have elapsed. */
    ASSERT_TRUE(freertos_stubs_check_preempt(frt));
    ASSERT_TRUE(strcmp(freertos_stubs_current_task_name(frt, 0), "high") == 0);
    ASSERT_EQ(cpu.pc, 0x400D4100u);
    ASSERT_EQ(ar_read(&cpu, 2), 1u);

    /* clear-on-exit consumed the notification count. */
    ar_write(&cpu, 0, 0x400D4110u);
    ar_write(&cpu, 2, 0u);
    ar_write(&cpu, 3, 1u);
    ar_write(&cpu, 4, 0u);
    cpu.pc = 0x400D4080u;
    stub_ulTaskGenericNotifyTake(&cpu, frt);
    ASSERT_EQ(ar_read(&cpu, 2), 0u);

    frt_teardown(&cpu, rom, frt);
}

/* A task that blocks inside a windowed call returns into the caller's
 * a(4*CALLINC+2), not a2. Patching a2 instead silently overwrote the
 * callee's first argument: ESP-IDF's emac_esp32_rx_task lost its `emac`
 * pointer the first time an ISR notified it, and every later receive()
 * dereferenced the notification count as a driver handle. */
TEST(test_task_notify_give_targets_the_windowed_return_register) {
    xtensa_cpu_t cpu;
    esp32_rom_stubs_t *rom;
    freertos_stubs_t *frt;
    frt_setup(&cpu, &rom, &frt);

    const uint32_t waiter_entry = 0x400D5000u;
    const uint32_t other_entry  = 0x400D6000u;
    uint32_t other_handle = frt_create_sched_task(
        &cpu, frt, other_entry, "other", 1u, 0x3FFB3020u, 0x3FFB3044u);
    uint32_t waiter_handle = frt_create_sched_task(
        &cpu, frt, waiter_entry, "waiter", 5u, 0x3FFB3000u, 0x3FFB3040u);
    ASSERT_TRUE(waiter_handle != 0u && other_handle != 0u);
    ASSERT_TRUE(freertos_stubs_check_preempt(frt));
    ASSERT_TRUE(strcmp(freertos_stubs_current_task_name(frt, 0), "waiter") == 0);

    /* Block the waiter from inside a CALL8 frame: a0/a2 belong to the
     * callee's window at CALLINC 2, so the caller's slots are a8 and a10. */
    extern void stub_ulTaskGenericNotifyTake(xtensa_cpu_t *, void *);
    const uint32_t kArg = 0x3F8001E4u;   /* the callee's first argument */
    XT_PS_SET_CALLINC(cpu.ps, 2);
    ar_write(&cpu, 2, kArg);
    ar_write(&cpu, 8, 0x400D5100u);      /* caller's a0: return address */
    ar_write(&cpu, 10, 0u);              /* slot 0 */
    ar_write(&cpu, 11, 1u);              /* clear count on exit */
    ar_write(&cpu, 12, UINT32_MAX);      /* wait indefinitely */
    cpu.pc = 0x400D5080u;
    stub_ulTaskGenericNotifyTake(&cpu, frt);
    ASSERT_TRUE(strcmp(freertos_stubs_current_task_name(frt, 0), "other") == 0);

    extern void stub_vTaskGenericNotifyGiveFromISR(xtensa_cpu_t *, void *);
    uint32_t woken_addr = 0x3FFB3050u;
    mem_write32(cpu.mem, woken_addr, 0u);
    XT_PS_SET_CALLINC(cpu.ps, 0);
    ar_write(&cpu, 0, 0x400D6100u);
    ar_write(&cpu, 2, waiter_handle);
    ar_write(&cpu, 3, 0u);
    ar_write(&cpu, 4, woken_addr);
    cpu.pc = 0x400D6080u;
    stub_vTaskGenericNotifyGiveFromISR(&cpu, frt);
    ASSERT_EQ(mem_read32(cpu.mem, woken_addr), 1u);

    ASSERT_TRUE(freertos_stubs_check_preempt(frt));
    ASSERT_TRUE(strcmp(freertos_stubs_current_task_name(frt, 0), "waiter") == 0);
    ASSERT_EQ(cpu.pc, 0x400D5100u);
    /* The count lands in the caller's return slot ... */
    ASSERT_EQ(ar_read(&cpu, 10), 1u);
    /* ... and the callee's argument is untouched. */
    ASSERT_EQ(ar_read(&cpu, 2), kArg);

    frt_teardown(&cpu, rom, frt);
}

TEST(test_bump_allocator) {
    xtensa_cpu_t cpu;
    esp32_rom_stubs_t *rom;
    freertos_stubs_t *frt;
    frt_setup(&cpu, &rom, &frt);

    uint32_t addr = 0x400D0090;
    extern void stub_pvPortMalloc(xtensa_cpu_t *, void *);
    rom_stubs_register_ctx(rom, addr, (rom_stub_fn)stub_pvPortMalloc, "pvPortMalloc", frt);

    /* Allocate 100 bytes */
    XT_PS_SET_CALLINC(cpu.ps, 0);
    ar_write(&cpu, 0, BASE + 0x100);
    ar_write(&cpu, 2, 100);
    cpu.pc = addr;
    xtensa_step(&cpu);
    uint32_t ptr1 = ar_read(&cpu, 2);
    ASSERT_EQ(ptr1, 0x3FFF0000u);

    /* Allocate 200 more bytes */
    XT_PS_SET_CALLINC(cpu.ps, 0);
    ar_write(&cpu, 0, BASE + 0x100);
    ar_write(&cpu, 2, 200);
    cpu.pc = addr;
    xtensa_step(&cpu);
    uint32_t ptr2 = ar_read(&cpu, 2);
    ASSERT_EQ(ptr2, 0x3FFF0064u);  /* 100 rounded up to 100 = 0x64 */

    frt_teardown(&cpu, rom, frt);
}

TEST(test_vTaskDelay_caps_large_ticks) {
    xtensa_cpu_t cpu;
    esp32_rom_stubs_t *rom;
    freertos_stubs_t *frt;
    frt_setup(&cpu, &rom, &frt);

    uint32_t vtd_addr = 0x400D0000;
    extern void stub_vTaskDelay(xtensa_cpu_t *, void *);
    rom_stubs_register_ctx(rom, vtd_addr, (rom_stub_fn)stub_vTaskDelay, "vTaskDelay", frt);

    uint64_t vtime_before = cpu.virtual_time_us;
    /* Very large ticks value that would overflow without cap */
    XT_PS_SET_CALLINC(cpu.ps, 0);
    ar_write(&cpu, 0, BASE + 0x100);
    ar_write(&cpu, 2, 0xFFFFFFFF);
    cpu.pc = vtd_addr;
    xtensa_step(&cpu);

    /* Capped at 200M cycles / 160 MHz = 1,250,000 us */
    ASSERT_EQ(cpu.virtual_time_us - vtime_before, 200000000 / 160);

    frt_teardown(&cpu, rom, frt);
}

TEST(test_vTaskDelete_noop) {
    xtensa_cpu_t cpu;
    esp32_rom_stubs_t *rom;
    freertos_stubs_t *frt;
    frt_setup(&cpu, &rom, &frt);

    uint32_t addr = 0x400D00A0;
    extern void stub_vTaskDelete(xtensa_cpu_t *, void *);
    rom_stubs_register_ctx(rom, addr, (rom_stub_fn)stub_vTaskDelete, "vTaskDelete", frt);

    XT_PS_SET_CALLINC(cpu.ps, 0);
    ar_write(&cpu, 0, BASE + 0x100);
    ar_write(&cpu, 2, 0);  /* handle */
    cpu.pc = addr;
    xtensa_step(&cpu);
    /* Should return cleanly */
    ASSERT_EQ(cpu.pc, BASE + 0x100);

    frt_teardown(&cpu, rom, frt);
}

TEST(test_vPortFree_noop) {
    xtensa_cpu_t cpu;
    esp32_rom_stubs_t *rom;
    freertos_stubs_t *frt;
    frt_setup(&cpu, &rom, &frt);

    uint32_t addr = 0x400D00B0;
    extern void stub_vPortFree(xtensa_cpu_t *, void *);
    rom_stubs_register_ctx(rom, addr, (rom_stub_fn)stub_vPortFree, "vPortFree", frt);

    XT_PS_SET_CALLINC(cpu.ps, 0);
    ar_write(&cpu, 0, BASE + 0x100);
    ar_write(&cpu, 2, 0x3FFF0000);
    cpu.pc = addr;
    xtensa_step(&cpu);
    ASSERT_EQ(cpu.pc, BASE + 0x100);

    frt_teardown(&cpu, rom, frt);
}

/* ===== Suite runner ===== */

static void run_freertos_tests(void) {
    TEST_SUITE("freertos_stubs");
    RUN_TEST(test_vTaskDelay_advances_ccount);
    RUN_TEST(test_xTaskCreate_returns_pdPASS);
    RUN_TEST(test_pinned_task_reads_core_affinity_from_windowed_stack);
    RUN_TEST(test_xTaskGetTickCount);
    RUN_TEST(test_scheduler_start_sets_dual_core_ready_flags);
    RUN_TEST(test_queue_send_receive);
    RUN_TEST(test_queue_receive_empty_returns_false);
    RUN_TEST(test_queue_overwrite_and_reset);
    RUN_TEST(test_semaphore_create_take_give);
    RUN_TEST(test_counting_semaphore_supports_idf_unbounded_capacity);
    RUN_TEST(test_task_notify_isr_preempts_and_preserves_window_context);
    RUN_TEST(test_task_notify_give_targets_the_windowed_return_register);
    RUN_TEST(test_bump_allocator);
    RUN_TEST(test_vTaskDelay_caps_large_ticks);
    RUN_TEST(test_vTaskDelete_noop);
    RUN_TEST(test_vPortFree_noop);
}
