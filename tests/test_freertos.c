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
    uint32_t take_addr = 0x400D0070;
    uint32_t give_addr = 0x400D0080;
    extern void stub_xSemaphoreCreateMutex(xtensa_cpu_t *, void *);
    extern void stub_xSemaphoreTake(xtensa_cpu_t *, void *);
    extern void stub_xSemaphoreGive(xtensa_cpu_t *, void *);
    rom_stubs_register_ctx(rom, create_addr, (rom_stub_fn)stub_xSemaphoreCreateMutex, "xSemaphoreCreateMutex", frt);
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
    RUN_TEST(test_xTaskGetTickCount);
    RUN_TEST(test_queue_send_receive);
    RUN_TEST(test_queue_receive_empty_returns_false);
    RUN_TEST(test_queue_overwrite_and_reset);
    RUN_TEST(test_semaphore_create_take_give);
    RUN_TEST(test_bump_allocator);
    RUN_TEST(test_vTaskDelay_caps_large_ticks);
    RUN_TEST(test_vTaskDelete_noop);
    RUN_TEST(test_vPortFree_noop);
}
