/* Tests for production NimBLE observation and synthetic advertisements. */
#include "test_helpers.h"
#include "bt_stubs.h"
#include "rom_stubs.h"

#include <string.h>

#define TEST_MARAUDER_ENTRY          0x400831D8u
#define TEST_SCAN_START_ADDR         0x401048B4u
#define TEST_SCAN_HANDLER_ADDR       0x40104A9Cu
#define TEST_SCAN_SET_CALLBACKS_ADDR 0x401DA8D0u
#define TEST_BLE_EVENT_ADDR          0x50001C00u

typedef struct {
    uint64_t calls;
    uint32_t event_addr;
    uint32_t scan_addr;
    uint8_t type;
    uint8_t adv_type;
    uint8_t data_len;
    uint8_t addr_type;
    uint8_t addr[6];
    int8_t rssi;
    uint8_t data[31];
} test_gap_capture_t;

static uint32_t test_bt_arg(xtensa_cpu_t *cpu, int n)
{
    int ci = XT_PS_CALLINC(cpu->ps);
    return ar_read(cpu, ci * 4 + 2 + n);
}

static void test_bt_return(xtensa_cpu_t *cpu, uint32_t value)
{
    int ci = XT_PS_CALLINC(cpu->ps);
    ar_write(cpu, ci * 4 + 2, value);
    uint32_t a0 = ar_read(cpu, ci * 4);
    cpu->pc = (cpu->pc & 0xC0000000u) | (a0 & 0x3FFFFFFFu);
    XT_PS_SET_CALLINC(cpu->ps, 0);
}

static void capture_gap_event(xtensa_cpu_t *cpu, void *ctx)
{
    test_gap_capture_t *capture = ctx;
    capture->calls++;
    capture->event_addr = test_bt_arg(cpu, 0);
    capture->scan_addr = test_bt_arg(cpu, 1);
    capture->type = mem_read8(cpu->mem, capture->event_addr);
    capture->adv_type = mem_read8(cpu->mem, capture->event_addr + 4u);
    capture->data_len = mem_read8(cpu->mem, capture->event_addr + 5u);
    capture->addr_type = mem_read8(cpu->mem, capture->event_addr + 6u);
    for (unsigned i = 0; i < 6; i++)
        capture->addr[i] = mem_read8(cpu->mem,
                                     capture->event_addr + 7u + i);
    capture->rssi = (int8_t)mem_read8(cpu->mem, capture->event_addr + 13u);
    uint32_t data_addr = mem_read32(cpu->mem, capture->event_addr + 16u);
    for (unsigned i = 0; i < capture->data_len; i++)
        capture->data[i] = mem_read8(cpu->mem, data_addr + i);
    test_bt_return(cpu, 0);
}

static void invoke_observed_call8(xtensa_cpu_t *cpu, uint32_t addr,
                                  const uint32_t *args, size_t arg_count)
{
    memset(cpu->ar, 0, sizeof(cpu->ar));
    cpu->windowbase = 0;
    cpu->windowstart = 1;
    cpu->ps = 1u << 18;
    XT_PS_SET_CALLINC(cpu->ps, 2);
    ar_write(cpu, 1, 0x3FFE0000u);
    ar_write(cpu, 8, (2u << 30) | ((BASE + 0x100u) & 0x3FFFFFFFu));
    for (size_t i = 0; i < arg_count; i++)
        ar_write(cpu, 10 + (int)i, args[i]);
    cpu->pc = addr;
    cpu->running = true;
    cpu->halted = false;
    cpu->exception = false;
    cpu->_pc_written = true;
    xtensa_step(cpu);
}

TEST(production_scan_delivers_nimble_gap_advertisement) {
    xtensa_cpu_t cpu;
    setup(&cpu);
    esp32_rom_stubs_t *rom = rom_stubs_create(&cpu);
    bt_stubs_t *bt = bt_stubs_create(&cpu);
    test_gap_capture_t capture = {0};

    /* The observers let these genuine function entries execute. */
    put_insn3(&cpu, TEST_SCAN_SET_CALLBACKS_ADDR, 0x004136u);
    put_insn2(&cpu, TEST_SCAN_SET_CALLBACKS_ADDR + 3u, 0xF01Du);
    put_insn3(&cpu, TEST_SCAN_START_ADDR, 0x004136u);
    put_insn2(&cpu, TEST_SCAN_START_ADDR + 3u, 0xF01Du);

    ASSERT_EQ(bt_stubs_hook_firmware_addrs(bt, TEST_MARAUDER_ENTRY), 3);
    ASSERT_EQ(rom_stubs_register_ctx(rom, TEST_SCAN_HANDLER_ADDR,
                                     capture_gap_event,
                                     "test_gap_handler", &capture), 0);

    const uint32_t scan_addr = 0x3FFDF600u;
    const uint32_t callback_addr = 0x3FFE3F14u;
    uint32_t callback_args[] = {scan_addr, callback_addr, 0};
    invoke_observed_call8(&cpu, TEST_SCAN_SET_CALLBACKS_ADDR,
                          callback_args, 3);
    uint32_t start_args[] = {scan_addr, 0, 0, 0};
    invoke_observed_call8(&cpu, TEST_SCAN_START_ADDR, start_args, 4);

    static const uint8_t address[6] = {0x45, 0x58, 0x45, 0x4C, 0x46, 0x02};
    static const uint8_t payload[] = {
        0x02, 0x01, 0x06,
        0x09, 0x09, 'F', 'l', 'e', 'x', 'e', 'B', 'L', 'E',
    };
    ASSERT_EQ(bt_stubs_inject_advertisement(bt, address, 1, -47,
                                             payload, sizeof(payload)), 0);

    bt_stubs_stats_t stats = {0};
    bt_stubs_get_stats(bt, &stats);
    ASSERT_EQ64(stats.scan_callback_config_calls, 1);
    ASSERT_EQ64(stats.scan_start_calls, 1);
    ASSERT_EQ64(stats.advertisement_frames, 1);
    ASSERT_EQ64(stats.advertisement_callback_failures, 0);
    ASSERT_EQ64(capture.calls, 1);
    ASSERT_EQ(capture.event_addr, TEST_BLE_EVENT_ADDR);
    ASSERT_EQ(capture.scan_addr, scan_addr);
    ASSERT_EQ(capture.type, 7);
    ASSERT_EQ(capture.adv_type, 3);
    ASSERT_EQ(capture.data_len, sizeof(payload));
    ASSERT_EQ(capture.addr_type, 1);
    ASSERT_EQ((uint8_t)capture.rssi, (uint8_t)-47);
    ASSERT_EQ(memcmp(capture.addr, address, sizeof(address)), 0);
    ASSERT_EQ(memcmp(capture.data, payload, sizeof(payload)), 0);

    uint8_t oversized[32] = {0};
    ASSERT_EQ(bt_stubs_inject_advertisement(bt, address, 0, -1,
                                             oversized,
                                             sizeof(oversized)), -3);

    bt_stubs_destroy(bt);
    rom_stubs_destroy(rom);
    teardown(&cpu);
}

static void run_bt_stub_tests(void) {
    TEST_SUITE("Bluetooth stubs");
    RUN_TEST(production_scan_delivers_nimble_gap_advertisement);
}
