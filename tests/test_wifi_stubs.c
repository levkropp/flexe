/* Tests for host-backed WiFi and synthetic promiscuous-radio delivery. */
#include "test_helpers.h"
#include "rom_stubs.h"
#include "wifi_stubs.h"

#include <string.h>

#define TEST_MARAUDER_ENTRY          0x400831D8u
#define TEST_WIFI_SET_PROMISC_ADDR   0x40199190u
#define TEST_WIFI_SET_RX_CB_ADDR     0x401991FCu
#define TEST_WIFI_RAW_TX_ADDR        0x401A55CCu
#define TEST_PROMISC_PACKET_ADDR     0x50001D00u
#define TEST_PROMISC_RX_CTRL_SIZE    28u
#define TEST_WLED_ENTRY              0x40083E68u
#define TEST_WLED_EVENT_REGISTER     0x40150E34u
#define TEST_WLED_EVENT_POST         0x40151768u
#define TEST_WLED_DISCONNECT         0x40182F54u

typedef struct {
    uint64_t calls;
    uint32_t iface;
    uint8_t frame[32];
    size_t len;
    bool en_sys_seq;
} test_raw_tx_capture_t;

typedef struct {
    uint64_t calls;
    uint32_t base;
    int32_t event_id;
    uint32_t data;
    uint32_t size;
    uint32_t ticks_to_wait;
    uint8_t payload[64];
} test_event_post_capture_t;

static void capture_native_event_post(xtensa_cpu_t *cpu, void *ctx)
{
    test_event_post_capture_t *capture = ctx;
    capture->calls++;
    capture->base = ar_read(cpu, 10);
    capture->event_id = (int32_t)ar_read(cpu, 11);
    capture->data = ar_read(cpu, 12);
    capture->size = ar_read(cpu, 13);
    capture->ticks_to_wait = ar_read(cpu, 14);
    uint32_t copy = capture->size < sizeof(capture->payload) ?
                    capture->size : (uint32_t)sizeof(capture->payload);
    for (uint32_t i = 0; i < copy; i++)
        capture->payload[i] = mem_read8(cpu->mem, capture->data + i);

    /* guest_call8 enters a windowed function with CALLINC=2, a8 holding the
     * encoded return address, and its result slot at a10. */
    ar_write(cpu, 10, 0);
    cpu->pc = 0x40000000u | (ar_read(cpu, 8) & 0x3FFFFFFFu);
    XT_PS_SET_CALLINC(cpu->ps, 0);
}

static void capture_raw_tx(void *ctx, uint32_t iface, const uint8_t *frame,
                           size_t len, bool en_sys_seq)
{
    test_raw_tx_capture_t *capture = ctx;
    capture->calls++;
    capture->iface = iface;
    capture->len = len;
    capture->en_sys_seq = en_sys_seq;
    size_t copy_len = len < sizeof(capture->frame) ? len :
                      sizeof(capture->frame);
    memcpy(capture->frame, frame, copy_len);
}

static void invoke_wifi_call0(xtensa_cpu_t *cpu, uint32_t addr,
                              uint32_t arg0)
{
    cpu->pc = addr;
    XT_PS_SET_CALLINC(cpu->ps, 0);
    ar_write(cpu, 0, BASE + 0x100u);
    ar_write(cpu, 2, arg0);
    xtensa_step(cpu);
}

static void invoke_wifi_call0_4(xtensa_cpu_t *cpu, uint32_t addr,
                                uint32_t arg0, uint32_t arg1,
                                uint32_t arg2, uint32_t arg3)
{
    cpu->pc = addr;
    XT_PS_SET_CALLINC(cpu->ps, 0);
    ar_write(cpu, 0, BASE + 0x100u);
    ar_write(cpu, 2, arg0);
    ar_write(cpu, 3, arg1);
    ar_write(cpu, 4, arg2);
    ar_write(cpu, 5, arg3);
    xtensa_step(cpu);
}

TEST(promiscuous_frame_requires_enabled_callback) {
    xtensa_cpu_t cpu;
    setup(&cpu);
    esp32_rom_stubs_t *rom = rom_stubs_create(&cpu);
    wifi_stubs_t *wifi = wifi_stubs_create(&cpu);
    uint8_t frame = 0x80;

    ASSERT_EQ(wifi_stubs_inject_promiscuous_frame(
                      wifi, &frame, sizeof(frame), -42, 1, 0), -2);

    wifi_stubs_destroy(wifi);
    rom_stubs_destroy(rom);
    teardown(&cpu);
}

TEST(promiscuous_frame_runs_callback_and_restores_cpu) {
    xtensa_cpu_t cpu;
    setup(&cpu);
    esp32_rom_stubs_t *rom = rom_stubs_create(&cpu);
    seed_marauder_v11401_profile(&cpu);
    wifi_stubs_t *wifi = wifi_stubs_create(&cpu);

    ASSERT_EQ(wifi_stubs_hook_firmware_addrs(wifi, TEST_MARAUDER_ENTRY), 15);

    /* A minimal windowed callback: entry a1, 32; retw.n. */
    put_insn3(&cpu, BASE, 0x004136u);
    put_insn2(&cpu, BASE + 3u, 0xF01Du);
    invoke_wifi_call0(&cpu, TEST_WIFI_SET_PROMISC_ADDR, 1);
    invoke_wifi_call0(&cpu, TEST_WIFI_SET_RX_CB_ADDR, BASE);

    uint32_t expected_ar[64];
    for (unsigned i = 0; i < 64; i++)
        cpu.ar[i] = expected_ar[i] = 0xA5000000u + i;
    cpu.pc = BASE + 0x100u;
    cpu.ps = 0x00040023u;
    cpu.windowbase = 3;
    cpu.windowstart = 1u << 3;
    cpu.sar = 17;
    cpu.lbeg = BASE + 0x20u;
    cpu.lend = BASE + 0x30u;
    cpu.lcount = 9;
    cpu.br = 0x55AAu;
    cpu.running = true;
    /* Asynchronous callback delivery wakes a core that was in WAITI while
     * preserving its architectural task context. */
    cpu.halted = true;
    cpu.exception = false;
    cpu.irq_check = true;
    cpu.accelerated_blocks = true;
    cpu.virtual_time_us = 0x12345678u;

    static const uint8_t frame[] = {0x80, 0x00, 0x12, 0x34, 0x56};
    ASSERT_EQ(wifi_stubs_inject_promiscuous_frame(
                      wifi, frame, sizeof(frame), -42, 6, 0), 0);

    wifi_stubs_stats_t stats = {0};
    wifi_stubs_get_stats(wifi, &stats);
    ASSERT_EQ64(stats.raw_rx_frames, 1);
    ASSERT_EQ64(stats.raw_rx_callback_failures, 0);
    ASSERT_EQ(mem_read8(cpu.mem, TEST_PROMISC_PACKET_ADDR), (uint8_t)-42);
    ASSERT_EQ(mem_read8(cpu.mem, TEST_PROMISC_PACKET_ADDR + 10u), 6);
    ASSERT_EQ(mem_read32(cpu.mem, TEST_PROMISC_PACKET_ADDR + 12u),
              0x12345678u);
    ASSERT_EQ(mem_read32(cpu.mem, TEST_PROMISC_PACKET_ADDR + 24u),
              sizeof(frame) + 4u);
    for (unsigned i = 0; i < sizeof(frame); i++)
        ASSERT_EQ(mem_read8(cpu.mem, TEST_PROMISC_PACKET_ADDR +
                            TEST_PROMISC_RX_CTRL_SIZE + i), frame[i]);

    ASSERT_EQ(cpu.pc, BASE + 0x100u);
    ASSERT_EQ(cpu.ps, 0x00040023u);
    ASSERT_EQ(cpu.windowbase, 3);
    ASSERT_EQ(cpu.windowstart, 1u << 3);
    ASSERT_EQ(cpu.sar, 17);
    ASSERT_EQ(cpu.lbeg, BASE + 0x20u);
    ASSERT_EQ(cpu.lend, BASE + 0x30u);
    ASSERT_EQ(cpu.lcount, 9);
    ASSERT_EQ(cpu.br, 0x55AAu);
    ASSERT_TRUE(cpu.running);
    ASSERT_FALSE(cpu.halted);
    ASSERT_FALSE(cpu.exception);
    ASSERT_TRUE(cpu.irq_check);
    ASSERT_TRUE(cpu.accelerated_blocks);
    ASSERT_EQ(memcmp(cpu.ar, expected_ar, sizeof(expected_ar)), 0);

    uint8_t oversized[229] = {0};
    ASSERT_EQ(wifi_stubs_inject_promiscuous_frame(
                      wifi, oversized, sizeof(oversized), -1, 1, 0), -3);

    wifi_stubs_destroy(wifi);
    rom_stubs_destroy(rom);
    teardown(&cpu);
}

TEST(raw_tx_crosses_host_radio_boundary) {
    xtensa_cpu_t cpu;
    setup(&cpu);
    esp32_rom_stubs_t *rom = rom_stubs_create(&cpu);
    seed_marauder_v11401_profile(&cpu);
    wifi_stubs_t *wifi = wifi_stubs_create(&cpu);
    test_raw_tx_capture_t capture = {0};

    ASSERT_EQ(wifi_stubs_hook_firmware_addrs(wifi, TEST_MARAUDER_ENTRY), 15);
    wifi_stubs_set_raw_tx_callback(wifi, capture_raw_tx, &capture);

    static const uint8_t beacon[] = {
        0x80, 0x00, 0x00, 0x00,
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
        0x02, 0x46, 0x4C, 0x45, 0x58, 0x45,
        0x02, 0x46, 0x4C, 0x45, 0x58, 0x45,
        0x00, 0x00,
    };
    const uint32_t frame_addr = 0x3FFB1000u;
    for (unsigned i = 0; i < sizeof(beacon); i++)
        mem_write8(cpu.mem, frame_addr + i, beacon[i]);
    invoke_wifi_call0_4(&cpu, TEST_WIFI_RAW_TX_ADDR, 1, frame_addr,
                        sizeof(beacon), 1);

    wifi_stubs_stats_t stats = {0};
    wifi_stubs_get_stats(wifi, &stats);
    ASSERT_EQ64(capture.calls, 1);
    ASSERT_EQ(capture.iface, 1);
    ASSERT_EQ(capture.len, sizeof(beacon));
    ASSERT_TRUE(capture.en_sys_seq);
    ASSERT_EQ(memcmp(capture.frame, beacon, sizeof(beacon)), 0);
    ASSERT_EQ64(stats.raw_tx_frames, 1);
    ASSERT_EQ64(stats.raw_tx_bytes, sizeof(beacon));
    ASSERT_EQ64(stats.raw_tx_failures, 0);

    invoke_wifi_call0_4(&cpu, TEST_WIFI_RAW_TX_ADDR, 1, 0,
                        sizeof(beacon), 0);
    ASSERT_EQ(ar_read(&cpu, 2), 0x102u);
    wifi_stubs_get_stats(wifi, &stats);
    ASSERT_EQ64(stats.raw_tx_failures, 1);
    ASSERT_EQ64(capture.calls, 1);

    wifi_stubs_destroy(wifi);
    rom_stubs_destroy(rom);
    teardown(&cpu);
}

TEST(v11423_fingerprint_selects_shifted_wifi_entries) {
    xtensa_cpu_t cpu;
    setup(&cpu);
    esp32_rom_stubs_t *rom = rom_stubs_create(&cpu);
    seed_marauder_v11423_profile(&cpu);
    wifi_stubs_t *wifi = wifi_stubs_create(&cpu);

    ASSERT_EQ(wifi_stubs_hook_firmware_addrs(wifi, TEST_MARAUDER_ENTRY), 15);
    invoke_wifi_call0(&cpu, 0x4013B518u, 0u);
    ASSERT_EQ(ar_read(&cpu, 2), 0u);
    invoke_wifi_call0(&cpu, 0x401990A0u, 0u);
    ASSERT_EQ(ar_read(&cpu, 2), 0u);

    wifi_stubs_stats_t stats = {0};
    wifi_stubs_get_stats(wifi, &stats);
    ASSERT_EQ64(stats.wifi_init_calls, 1u);
    ASSERT_EQ64(stats.wifi_start_calls, 1u);

    wifi_stubs_destroy(wifi);
    rom_stubs_destroy(rom);
    teardown(&cpu);
}

TEST(v1121_cyd2usb_fingerprint_selects_idf55_wifi_entries) {
    xtensa_cpu_t cpu;
    setup(&cpu);
    esp32_rom_stubs_t *rom = rom_stubs_create(&cpu);
    seed_marauder_v1121_cyd2usb_profile(&cpu);
    wifi_stubs_t *wifi = wifi_stubs_create(&cpu);

    ASSERT_EQ(wifi_stubs_hook_firmware_addrs(wifi, 0x40081E90u), 15);
    invoke_wifi_call0(&cpu, 0x401730CCu, 0u);
    ASSERT_EQ(ar_read(&cpu, 2), 0u);
    invoke_wifi_call0(&cpu, 0x401A861Cu, 0u);
    ASSERT_EQ(ar_read(&cpu, 2), 0u);

    wifi_stubs_stats_t stats = {0};
    wifi_stubs_get_stats(wifi, &stats);
    ASSERT_EQ64(stats.wifi_init_calls, 1u);
    ASSERT_EQ64(stats.wifi_start_calls, 1u);

    wifi_stubs_destroy(wifi);
    rom_stubs_destroy(rom);
    teardown(&cpu);
}

TEST(wled_posts_disconnect_on_native_event_loop) {
    xtensa_cpu_t cpu;
    setup(&cpu);
    esp32_rom_stubs_t *rom = rom_stubs_create(&cpu);
    seed_wled_v1601_profile(&cpu);
    wifi_stubs_t *wifi = wifi_stubs_create(&cpu);
    test_event_post_capture_t capture = {0};
    const uint32_t event_base = 0x3FFB1000u;
    static const char name[] = "WIFI_EVENT";

    ASSERT_EQ(wifi_stubs_hook_firmware_addrs(wifi, TEST_WLED_ENTRY), 18);
    ASSERT_EQ(rom_stubs_register_ctx(rom, TEST_WLED_EVENT_POST,
                                     capture_native_event_post,
                                     "test_event_post", &capture), 0);
    for (size_t i = 0; i < sizeof(name); i++)
        mem_write8(cpu.mem, event_base + (uint32_t)i, (uint8_t)name[i]);

    /* The registration hook is a spy: it records WLED's handler but allows
     * the firmware implementation to execute. RET.N is a minimal stand-in
     * for that implementation in this unit test. */
    put_insn2(&cpu, TEST_WLED_EVENT_REGISTER, narrow(0xD, 15, 0, 0));
    invoke_wifi_call0_4(&cpu, TEST_WLED_EVENT_REGISTER, event_base, 5,
                        BASE + 0x200u, 0x11223344u);
    ASSERT_EQ(cpu.pc, BASE + 0x100u);

    invoke_wifi_call0(&cpu, TEST_WLED_DISCONNECT, 0);
    wifi_stubs_tick(wifi, &cpu, NULL);

    ASSERT_EQ64(capture.calls, 1u);
    ASSERT_EQ(capture.base, event_base);
    ASSERT_EQ(capture.event_id, 5);
    ASSERT_EQ(capture.data, 0x3FFE9000u);
    ASSERT_EQ(capture.size, 41u);
    ASSERT_EQ(capture.ticks_to_wait, 0u);
    ASSERT_EQ(capture.payload[33], 0x02u); /* synthetic BSSID */
    ASSERT_EQ(capture.payload[39], 8u);    /* WIFI_REASON_ASSOC_LEAVE */
    ASSERT_EQ(capture.payload[40], (uint8_t)(int8_t)-55);

    wifi_stubs_stats_t stats = {0};
    wifi_stubs_get_stats(wifi, &stats);
    ASSERT_EQ64(stats.events_delivered, 1u);

    wifi_stubs_destroy(wifi);
    rom_stubs_destroy(rom);
    teardown(&cpu);
}

static void run_wifi_stub_tests(void) {
    TEST_SUITE("WiFi stubs");
    RUN_TEST(promiscuous_frame_requires_enabled_callback);
    RUN_TEST(promiscuous_frame_runs_callback_and_restores_cpu);
    RUN_TEST(raw_tx_crosses_host_radio_boundary);
    RUN_TEST(v11423_fingerprint_selects_shifted_wifi_entries);
    RUN_TEST(v1121_cyd2usb_fingerprint_selects_idf55_wifi_entries);
    RUN_TEST(wled_posts_disconnect_on_native_event_loop);
}
