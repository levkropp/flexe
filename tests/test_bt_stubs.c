/* Tests for production NimBLE observation and synthetic advertisements. */
#include "test_helpers.h"
#include "bt_stubs.h"
#include "rom_stubs.h"

#include <string.h>

#define TEST_MARAUDER_ENTRY          0x400831D8u
#define TEST_SCAN_START_ADDR         0x401048B4u
#define TEST_SCAN_HANDLER_ADDR       0x40104A9Cu
#define TEST_SCAN_SET_CALLBACKS_ADDR 0x401DA8D0u
#define TEST_GAP_ADV_START_ADDR      0x401096E0u
#define TEST_HCI_CMD_TX_ADDR         0x40110254u
#define TEST_CONN_CAN_ALLOC_ADDR     0x4010FC58u
#define TEST_ID_USE_ADDR             0x40110CC0u
#define TEST_HS_SYNC_STATE_ADDR      0x3FFCDA20u
#define TEST_HS_ENABLED_STATE_ADDR   0x3FFCDA44u
#define TEST_HS_PUBLIC_ADDR          0x3FFCA032u
#define TEST_HS_ENABLED_LITERAL      0x4010B4A8u
#define TEST_HS_SYNC_LITERAL         0x4010B4B4u
#define TEST_HS_PUBLIC_LITERAL       0x4010B5DCu
#define TEST_BLE_EVENT_ADDR          0x50001C00u

typedef struct {
    uint64_t calls;
    uint32_t core_id;
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

typedef struct {
    uint64_t calls;
    uint8_t advertisement[31];
    size_t advertisement_len;
    uint8_t scan_response[31];
    size_t scan_response_len;
} test_adv_tx_capture_t;

static void capture_advertisement_tx(void *ctx,
                                     const uint8_t *advertisement,
                                     size_t advertisement_len,
                                     const uint8_t *scan_response,
                                     size_t scan_response_len)
{
    test_adv_tx_capture_t *capture = ctx;
    capture->calls++;
    capture->advertisement_len = advertisement_len;
    capture->scan_response_len = scan_response_len;
    memcpy(capture->advertisement, advertisement, advertisement_len);
    memcpy(capture->scan_response, scan_response, scan_response_len);
}

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
    capture->core_id = cpu->core_id;
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
    seed_marauder_v11401_profile(&cpu);
    bt_stubs_t *bt = bt_stubs_create(&cpu);
    xtensa_cpu_t cpu1;
    xtensa_cpu_init(&cpu1);
    cpu1.mem = cpu.mem;
    cpu1.core_id = 1;
    cpu1.pc_hook = cpu.pc_hook;
    cpu1.pc_hook_ctx = cpu.pc_hook_ctx;
    cpu1.pc_hook_bitmap = cpu.pc_hook_bitmap;
    test_gap_capture_t capture = {0};
    test_adv_tx_capture_t tx_capture = {0};

    /* The observers let these genuine function entries execute. */
    put_insn3(&cpu, TEST_SCAN_SET_CALLBACKS_ADDR, 0x004136u);
    put_insn2(&cpu, TEST_SCAN_SET_CALLBACKS_ADDR + 3u, 0xF01Du);
    put_insn3(&cpu, TEST_SCAN_START_ADDR, 0x004136u);
    put_insn2(&cpu, TEST_SCAN_START_ADDR + 3u, 0xF01Du);
    put_insn3(&cpu, TEST_GAP_ADV_START_ADDR, 0x004136u);
    put_insn2(&cpu, TEST_GAP_ADV_START_ADDR + 3u, 0xF01Du);
    put_insn3(&cpu, TEST_HCI_CMD_TX_ADDR, 0x004136u);
    put_insn2(&cpu, TEST_HCI_CMD_TX_ADDR + 3u, 0xF01Du);
    put_insn3(&cpu, TEST_CONN_CAN_ALLOC_ADDR, 0x004136u);
    put_insn2(&cpu, TEST_CONN_CAN_ALLOC_ADDR + 3u, 0xF01Du);
    put_insn3(&cpu, TEST_ID_USE_ADDR, 0x004136u);
    put_insn2(&cpu, TEST_ID_USE_ADDR + 3u, 0xF01Du);
    mem_write32(cpu.mem, TEST_HS_ENABLED_LITERAL,
                TEST_HS_ENABLED_STATE_ADDR);
    mem_write32(cpu.mem, TEST_HS_SYNC_LITERAL, TEST_HS_SYNC_STATE_ADDR);
    mem_write32(cpu.mem, TEST_HS_PUBLIC_LITERAL, TEST_HS_PUBLIC_ADDR);

    ASSERT_EQ(bt_stubs_hook_firmware_addrs(bt, TEST_MARAUDER_ENTRY), 7);
    bt_stubs_set_advertisement_tx_callback(bt, capture_advertisement_tx,
                                            &tx_capture);
    ASSERT_EQ(rom_stubs_register_ctx(rom, TEST_SCAN_HANDLER_ADDR,
                                     capture_gap_event,
                                     "test_gap_handler", &capture), 0);

    const uint32_t scan_addr = 0x3FFDF600u;
    const uint32_t callback_addr = 0x3FFE3F14u;
    uint32_t callback_args[] = {scan_addr, callback_addr, 0};
    /* Marauder owns the NimBLE scan object on APP CPU. Advertisement delivery
     * must preserve that affinity because its callback can reacquire locks
     * already held recursively by the owning core. */
    invoke_observed_call8(&cpu1, TEST_SCAN_SET_CALLBACKS_ADDR,
                          callback_args, 3);
    uint32_t start_args[] = {scan_addr, 0, 0, 0};
    invoke_observed_call8(&cpu1, TEST_SCAN_START_ADDR, start_args, 4);
    ASSERT_EQ(mem_read8(cpu.mem, TEST_HS_SYNC_STATE_ADDR), 2);
    ASSERT_EQ(mem_read8(cpu.mem, TEST_HS_ENABLED_STATE_ADDR), 2);
    static const uint8_t expected_public_addr[6] = {
        0xFE, 0xCA, 0xEF, 0xBE, 0xAD, 0xDE
    };
    for (unsigned i = 0; i < sizeof(expected_public_addr); i++)
        ASSERT_EQ(mem_read8(cpu.mem, TEST_HS_PUBLIC_ADDR + i),
                  expected_public_addr[i]);

    /* Controller state can be republished after a NimBLE host reset, and the
     * virtual link reports capacity for connectable advertising. */
    for (unsigned i = 0; i < sizeof(expected_public_addr); i++)
        mem_write8(cpu.mem, TEST_HS_PUBLIC_ADDR + i, 0);
    uint32_t identity_args[] = {0};
    invoke_observed_call8(&cpu, TEST_ID_USE_ADDR, identity_args, 1);
    for (unsigned i = 0; i < sizeof(expected_public_addr); i++)
        ASSERT_EQ(mem_read8(cpu.mem, TEST_HS_PUBLIC_ADDR + i),
                  expected_public_addr[i]);
    invoke_observed_call8(&cpu, TEST_CONN_CAN_ALLOC_ADDR, NULL, 0);
    ASSERT_EQ(ar_read(&cpu, 10), 1);

    static const uint8_t firmware_public_addr[6] = {1, 2, 3, 4, 5, 6};
    for (unsigned i = 0; i < sizeof(firmware_public_addr); i++)
        mem_write8(cpu.mem, TEST_HS_PUBLIC_ADDR + i,
                   firmware_public_addr[i]);

    const uint32_t adv_params_addr = 0x3FFB2F00u;
    mem_write8(cpu.mem, adv_params_addr, 1);
    mem_write8(cpu.mem, adv_params_addr + 1u, 2);
    mem_write8(cpu.mem, adv_params_addr + 8u, 0);
    uint32_t adv_start_args[] = {
        0, 0, UINT32_MAX, adv_params_addr, 0, 0
    };
    invoke_observed_call8(&cpu, TEST_GAP_ADV_START_ADDR,
                          adv_start_args, 6);
    for (unsigned i = 0; i < sizeof(firmware_public_addr); i++)
        ASSERT_EQ(mem_read8(cpu.mem, TEST_HS_PUBLIC_ADDR + i),
                  firmware_public_addr[i]);

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
    ASSERT_EQ64(stats.gap_advertising_start_calls, 1);
    ASSERT_EQ(stats.last_advertising_own_addr_type, 0);
    ASSERT_EQ(stats.last_advertising_duration_ms, UINT32_MAX);
    ASSERT_EQ(stats.last_advertising_conn_mode, 1);
    ASSERT_EQ(stats.last_advertising_disc_mode, 2);
    ASSERT_EQ(stats.last_advertising_high_duty, 0);
    ASSERT_EQ64(stats.advertisement_frames, 1);
    ASSERT_EQ64(stats.advertisement_callback_failures, 0);
    ASSERT_EQ64(capture.calls, 1);
    ASSERT_EQ(capture.core_id, 1);
    ASSERT_EQ(capture.event_addr, TEST_BLE_EVENT_ADDR);
    ASSERT_EQ(capture.scan_addr, scan_addr);
    ASSERT_EQ(capture.type, 7);
    ASSERT_EQ(capture.adv_type, 3);
    ASSERT_EQ(capture.data_len, sizeof(payload));
    ASSERT_EQ(capture.addr_type, 1);
    ASSERT_EQ((uint8_t)capture.rssi, (uint8_t)-47);
    ASSERT_EQ(memcmp(capture.addr, address, sizeof(address)), 0);
    ASSERT_EQ(memcmp(capture.data, payload, sizeof(payload)), 0);

    /* Exercise scan setup plus the complete legacy advertising command
     * sequence at the host-to-controller boundary. */
    const uint32_t hci_cmd_addr = 0x3FFB3000u;
    uint32_t scan_params_args[] = {
        0x200Bu, hci_cmd_addr, 7u, 0, 0
    };
    invoke_observed_call8(&cpu, TEST_HCI_CMD_TX_ADDR,
                          scan_params_args, 5);
    mem_write8(cpu.mem, hci_cmd_addr + 8u, 1);
    mem_write8(cpu.mem, hci_cmd_addr + 9u, 0);
    uint32_t scan_enable_args[] = {
        0x200Cu, hci_cmd_addr + 8u, 2u, 0, 0
    };
    invoke_observed_call8(&cpu, TEST_HCI_CMD_TX_ADDR,
                          scan_enable_args, 5);
    mem_write8(cpu.mem, hci_cmd_addr + 8u, 0);
    invoke_observed_call8(&cpu, TEST_HCI_CMD_TX_ADDR,
                          scan_enable_args, 5);

    uint32_t adv_params_args[] = {
        0x2006u, hci_cmd_addr + 16u, 15u, 0, 0
    };
    invoke_observed_call8(&cpu, TEST_HCI_CMD_TX_ADDR,
                          adv_params_args, 5);

    mem_write8(cpu.mem, hci_cmd_addr + 32u, sizeof(payload));
    for (unsigned i = 0; i < sizeof(payload); i++)
        mem_write8(cpu.mem, hci_cmd_addr + 33u + i, payload[i]);
    uint32_t set_data_args[] = {
        0x2008u, hci_cmd_addr + 32u, 32u, 0, 0
    };
    invoke_observed_call8(&cpu, TEST_HCI_CMD_TX_ADDR, set_data_args, 5);

    static const uint8_t scan_response[] = {0x02, 0x0A, 0xEC};
    mem_write8(cpu.mem, hci_cmd_addr + 64u, sizeof(scan_response));
    for (unsigned i = 0; i < sizeof(scan_response); i++)
        mem_write8(cpu.mem, hci_cmd_addr + 65u + i, scan_response[i]);
    uint32_t set_scan_response_args[] = {
        0x2009u, hci_cmd_addr + 64u, 32u, 0, 0
    };
    invoke_observed_call8(&cpu, TEST_HCI_CMD_TX_ADDR,
                          set_scan_response_args, 5);

    mem_write8(cpu.mem, hci_cmd_addr + 96u, 1);
    uint32_t enable_args[] = {
        0x200Au, hci_cmd_addr + 96u, 1u, 0, 0
    };
    invoke_observed_call8(&cpu, TEST_HCI_CMD_TX_ADDR, enable_args, 5);

    bt_stubs_get_stats(bt, &stats);
    ASSERT_EQ64(tx_capture.calls, 1);
    ASSERT_EQ(tx_capture.advertisement_len, sizeof(payload));
    ASSERT_EQ(tx_capture.scan_response_len, sizeof(scan_response));
    ASSERT_EQ(memcmp(tx_capture.advertisement, payload, sizeof(payload)), 0);
    ASSERT_EQ(memcmp(tx_capture.scan_response, scan_response,
                     sizeof(scan_response)), 0);

    mem_write8(cpu.mem, hci_cmd_addr + 96u, 0);
    invoke_observed_call8(&cpu, TEST_HCI_CMD_TX_ADDR, enable_args, 5);

    /* Malformed virtualized commands fail at this boundary, while unrelated
     * HCI commands fall through to the stock implementation. */
    mem_write8(cpu.mem, hci_cmd_addr + 32u, 32);
    invoke_observed_call8(&cpu, TEST_HCI_CMD_TX_ADDR, set_data_args, 5);
    ASSERT_EQ(ar_read(&cpu, 10), 0x12);
    uint32_t invalid_enable_args[] = {0x200Au, 0, 0, 0, 0};
    invoke_observed_call8(&cpu, TEST_HCI_CMD_TX_ADDR,
                          invalid_enable_args, 5);
    ASSERT_EQ(ar_read(&cpu, 10), 0x12);

    /* Zero-length advertising data is valid after reset and still produces
     * a controller transmission event. */
    mem_write8(cpu.mem, hci_cmd_addr + 32u, 0);
    invoke_observed_call8(&cpu, TEST_HCI_CMD_TX_ADDR, set_data_args, 5);
    mem_write8(cpu.mem, hci_cmd_addr + 96u, 1);
    invoke_observed_call8(&cpu, TEST_HCI_CMD_TX_ADDR, enable_args, 5);
    mem_write8(cpu.mem, hci_cmd_addr + 96u, 0);
    invoke_observed_call8(&cpu, TEST_HCI_CMD_TX_ADDR, enable_args, 5);

    uint32_t passthrough_args[] = {
        0x1234u, hci_cmd_addr, 0u, 0, 0
    };
    invoke_observed_call8(&cpu, TEST_HCI_CMD_TX_ADDR,
                          passthrough_args, 5);
    ASSERT_EQ(cpu.pc, TEST_HCI_CMD_TX_ADDR + 3u);

    bt_stubs_get_stats(bt, &stats);
    ASSERT_EQ64(stats.connection_capacity_queries, 1);
    ASSERT_EQ64(stats.identity_address_queries, 1);
    ASSERT_EQ64(stats.hci_command_calls, 14);
    ASSERT_EQ64(stats.hci_scan_parameters_calls, 1);
    ASSERT_EQ64(stats.hci_scan_enable_calls, 1);
    ASSERT_EQ64(stats.hci_scan_disable_calls, 1);
    ASSERT_EQ64(stats.advertising_parameters_calls, 1);
    ASSERT_EQ64(stats.advertising_data_calls, 3);
    ASSERT_EQ64(stats.advertising_scan_response_calls, 1);
    ASSERT_EQ64(stats.advertising_enable_calls, 2);
    ASSERT_EQ64(stats.advertising_disable_calls, 2);
    ASSERT_EQ64(stats.advertisement_tx_frames, 2);
    ASSERT_EQ64(stats.advertisement_tx_bytes,
                sizeof(payload) + 2 * sizeof(scan_response));
    ASSERT_EQ64(stats.advertisement_tx_failures, 2);
    ASSERT_EQ64(tx_capture.calls, 2);
    ASSERT_EQ(tx_capture.advertisement_len, 0);
    ASSERT_EQ(tx_capture.scan_response_len, sizeof(scan_response));
    ASSERT_EQ(memcmp(tx_capture.scan_response, scan_response,
                     sizeof(scan_response)), 0);

    uint8_t oversized[32] = {0};
    ASSERT_EQ(bt_stubs_inject_advertisement(bt, address, 0, -1,
                                             oversized,
                                             sizeof(oversized)), -3);

    bt_stubs_destroy(bt);
    rom_stubs_destroy(rom);
    teardown(&cpu);
}

TEST(v11423_fingerprint_selects_shifted_nimble_entries) {
    const uint32_t scan_start = 0x40104F9Cu;
    const uint32_t enabled_literal = 0x4010B4DCu;
    const uint32_t sync_literal = 0x4010B4E8u;
    const uint32_t public_literal = 0x4010B60Cu;

    xtensa_cpu_t cpu;
    setup(&cpu);
    esp32_rom_stubs_t *rom = rom_stubs_create(&cpu);
    seed_marauder_v11423_profile(&cpu);
    bt_stubs_t *bt = bt_stubs_create(&cpu);

    put_insn3(&cpu, scan_start, 0x004136u);
    put_insn2(&cpu, scan_start + 3u, 0xF01Du);
    mem_write32(cpu.mem, enabled_literal, TEST_HS_ENABLED_STATE_ADDR);
    mem_write32(cpu.mem, sync_literal, TEST_HS_SYNC_STATE_ADDR);
    mem_write32(cpu.mem, public_literal, TEST_HS_PUBLIC_ADDR);

    ASSERT_EQ(bt_stubs_hook_firmware_addrs(bt, TEST_MARAUDER_ENTRY), 7);
    const uint32_t scan_args[] = {0x3FFDF600u, 0, 0, 0};
    invoke_observed_call8(&cpu, scan_start, scan_args, 4);

    ASSERT_EQ(mem_read8(cpu.mem, TEST_HS_SYNC_STATE_ADDR), 2);
    ASSERT_EQ(mem_read8(cpu.mem, TEST_HS_ENABLED_STATE_ADDR), 2);
    static const uint8_t expected_public_addr[6] = {
        0xFE, 0xCA, 0xEF, 0xBE, 0xAD, 0xDE
    };
    for (unsigned i = 0; i < sizeof(expected_public_addr); i++)
        ASSERT_EQ(mem_read8(cpu.mem, TEST_HS_PUBLIC_ADDR + i),
                  expected_public_addr[i]);

    bt_stubs_stats_t stats = {0};
    bt_stubs_get_stats(bt, &stats);
    ASSERT_EQ64(stats.scan_start_calls, 1);

    bt_stubs_destroy(bt);
    rom_stubs_destroy(rom);
    teardown(&cpu);
}

static void run_bt_stub_tests(void) {
    TEST_SUITE("Bluetooth stubs");
    RUN_TEST(production_scan_delivers_nimble_gap_advertisement);
    RUN_TEST(v11423_fingerprint_selects_shifted_nimble_entries);
}
