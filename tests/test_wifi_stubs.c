/* Tests for host-backed WiFi and synthetic promiscuous-radio delivery. */
#include "test_helpers.h"
#include "rom_stubs.h"
#include "wifi_stubs.h"

#include <string.h>
#ifndef _WIN32
#include <arpa/inet.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

#define TEST_MARAUDER_ENTRY          0x400831D8u
#define TEST_WIFI_SET_PROMISC_ADDR   0x40199190u
#define TEST_WIFI_SET_RX_CB_ADDR     0x401991FCu
#define TEST_WIFI_RAW_TX_ADDR        0x401A55CCu
#define TEST_PROMISC_PACKET_ADDR     0x50001D00u
#define TEST_PROMISC_RX_CTRL_SIZE    28u
#define TEST_NERDMINER_ENTRY         0x40089268u
#define TEST_WLED_ENTRY              0x40083E68u
#define TEST_WLED_EVENT_REGISTER     0x40150E34u
#define TEST_WLED_EVENT_POST         0x40151768u
#define TEST_WLED_DISCONNECT         0x40182F54u

typedef struct {
    const uint8_t *bytes;
    size_t size;
    uint32_t offset;
} test_lwip_fingerprint_t;

/* Instruction prefixes from one ordinary ESP-IDF 4.x link. The production
 * matcher hashes their normalized instructions; placing them at unrelated
 * addresses here proves that application identity and final link position
 * are not inputs to discovery. */
static void seed_relocated_idf4_lwip_family(xtensa_cpu_t *cpu,
                                            uint32_t base) {
    static const uint8_t gethostbyname[] = {
        0x36, 0x81, 0x00, 0x0C, 0x2C, 0x8B, 0xB1, 0xAD,
        0x02, 0x25, 0x26, 0x12, 0xA0, 0x30, 0x74, 0x8C,
        0xD3, 0x21, 0x06, 0xDA, 0x32, 0xA0, 0xD2, 0x39,
        0x02, 0x0C, 0x02, 0xC0, 0x20, 0x00, 0x1D, 0xF0,
    };
    static const uint8_t read_fn[] = {
        0x36, 0x41, 0x00, 0xF2, 0xA0, 0x00, 0xF0, 0xEF,
        0x20, 0xF0, 0xDF, 0x20, 0xCD, 0x04, 0xBD, 0x03,
        0xAD, 0x02, 0x25, 0xF2, 0xFF, 0x2D, 0x0A, 0x1D,
        0xF0, 0x00, 0x00, 0x00,
    };
    static const uint8_t sendto_fn[] = {
        0x36, 0xC1, 0x00, 0x20, 0xA2, 0x20, 0x72, 0x61,
        0x03, 0xC0, 0x20, 0x00, 0xE5, 0x1C, 0xFF, 0xA0,
        0x7A, 0x20, 0x88, 0x31, 0xCC, 0x5A, 0x7C, 0xF2,
        0x86, 0x08, 0x00, 0x00, 0x00, 0x92, 0x2A, 0x00,
    };
    static const uint8_t send_fn[] = {
        0x36, 0x61, 0x00, 0xAD, 0x02, 0x25, 0x06, 0xFF,
        0x6D, 0x0A, 0xCC, 0x3A, 0x7C, 0xF2, 0x1D, 0xF0,
        0x00, 0xA2, 0x2A, 0x00, 0xD2, 0xA0, 0xF0, 0x82,
        0x2A, 0x00, 0x80, 0xDD, 0x10, 0x26, 0xBD,
    };
    static const uint8_t write_fn[] = {
        0x36, 0x41, 0x00, 0xD2, 0xA0, 0x00, 0x40, 0xC4,
        0x20, 0x30, 0xB3, 0x20, 0xAD, 0x02, 0xE5, 0xEB,
        0xFF, 0x2D, 0x0A, 0x1D, 0xF0, 0x00, 0x00, 0x00,
        0x36, 0x81, 0x00, 0x20, 0xA2, 0x20,
    };
    static const uint8_t dns[] = {
        0x36, 0x41, 0x00, 0x0C, 0x2E, 0xDD, 0x05, 0xCD,
        0x04, 0xBD, 0x03, 0xAD, 0x02, 0xE5, 0xBC, 0xFF,
        0xA0, 0x20, 0x74, 0x1D, 0xF0, 0x00, 0x00, 0x00,
        0x36, 0x41, 0x00, 0x25, 0x10, 0x0E,
    };
    static const test_lwip_fingerprint_t functions[] = {
        { gethostbyname, sizeof(gethostbyname), 0x000u },
        { read_fn,       sizeof(read_fn),       0x080u },
        { sendto_fn,     sizeof(sendto_fn),     0x100u },
        { send_fn,       sizeof(send_fn),       0x180u },
        { write_fn,      sizeof(write_fn),      0x200u },
        { dns,           sizeof(dns),           0x280u },
    };
    for (size_t i = 0u; i < sizeof(functions) / sizeof(functions[0]); i++)
        put_test_bytes(cpu, base + functions[i].offset, functions[i].bytes,
                       functions[i].size);
}

static void seed_relocated_idf4_socket_members(xtensa_cpu_t *cpu,
                                                uint32_t base) {
    static const uint8_t close_fn[] = {
        0x36, 0xA1, 0x00, 0x20, 0xA2, 0x20, 0x25, 0x3E,
        0xFF, 0x20, 0x62, 0x20, 0xA0, 0x4A, 0x20, 0x7C,
        0xF2, 0x16, 0x1A, 0x10, 0x28, 0x0A, 0x0C, 0x05,
        0x57, 0x12, 0x0F, 0x38, 0x02, 0x22, 0xA0, 0xF0,
    };
    static const uint8_t socket_fn[] = {
        0x36, 0x41, 0x00, 0x26, 0x23, 0x29, 0x26, 0x33,
        0x12, 0x66, 0x13, 0x4F, 0x22, 0xC2, 0xFE, 0xC1,
        0x3F, 0xD4, 0x0C, 0x0B, 0x1C, 0x0A, 0x1C, 0x83,
        0x46, 0x03, 0x00, 0x00, 0xC1, 0x3C,
    };
    static const uint8_t errno_template[] = {
        0x36, 0x41, 0x00, 0x81, 0x78, 0xD5, 0xE0, 0x08,
        0x00, 0x2D, 0x0A, 0x1D, 0xF0, 0x00, 0x00, 0x00,
        0x36, 0x41, 0x00, 0x20, 0xA2, 0x20, 0xB2, 0xA0,
        0x00, 0x25, 0xBD, 0x07, 0x81, 0x73, 0xD5,
    };
    static const uint8_t errno_target_template[] = {
        0x36, 0x41, 0x00,             /* ENTRY */
        0x21, 0x00, 0x00,             /* L32R a2, task errno literal */
        0x1D, 0xF0,                   /* RETW.N */
    };
    const uint32_t errno_fn_addr = base + 0x100u;
    const uint32_t errno_fn_literal = base + 0x0F0u;
    const uint32_t errno_target = base + 0x180u;
    const uint32_t errno_target_literal = base + 0x170u;
    uint8_t errno_fn[sizeof(errno_template)];
    uint8_t errno_target_fn[sizeof(errno_target_template)];
    memcpy(errno_fn, errno_template, sizeof(errno_fn));
    memcpy(errno_target_fn, errno_target_template, sizeof(errno_target_fn));

    /* Relocate both L32Rs. Their immediate fields are intentionally absent
     * from the structural hash, while the destination register and every
     * executable byte remain authenticated. */
    uint32_t displacement = errno_fn_literal - ((errno_fn_addr + 6u) & ~3u);
    uint16_t immediate = (uint16_t)(displacement >> 2);
    errno_fn[4] = (uint8_t)immediate;
    errno_fn[5] = (uint8_t)(immediate >> 8);
    displacement = errno_target_literal - ((errno_target + 6u) & ~3u);
    immediate = (uint16_t)(displacement >> 2);
    errno_target_fn[4] = (uint8_t)immediate;
    errno_target_fn[5] = (uint8_t)(immediate >> 8);

    put_test_bytes(cpu, base, close_fn, sizeof(close_fn));
    put_test_bytes(cpu, base + 0x80u, socket_fn, sizeof(socket_fn));
    put_test_bytes(cpu, errno_fn_addr, errno_fn, sizeof(errno_fn));
    put_test_bytes(cpu, errno_target, errno_target_fn,
                   sizeof(errno_target_fn));
    mem_write32(cpu->mem, errno_fn_literal, errno_target);
    mem_write32(cpu->mem, errno_target_literal, 0x3FFE3D00u);
}

/* Relocated ESP-IDF 5.5/picolibc implementations from an ordinary release
 * link. None of the original application's addresses are retained. */
static void seed_relocated_idf5_lwip_family(xtensa_cpu_t *cpu,
                                            uint32_t base) {
    static const uint8_t gethostbyname[] = {
        0x36, 0x81, 0x00, 0xC2, 0xA0, 0x18, 0xB2, 0xA0,
        0x00, 0x10, 0xA1, 0x20, 0x81, 0x4C, 0xB8, 0xE0,
        0x08, 0x00, 0x0C, 0x2C, 0xBD, 0x01, 0xAD, 0x02,
        0xA5, 0xA7, 0x12, 0x7D, 0x0A, 0x56, 0xCA, 0x04,
    };
    static const uint8_t read_fn[] = {
        0x36, 0x41, 0x00, 0x20, 0xA2, 0x20, 0x30, 0xB3,
        0x20, 0x40, 0xC4, 0x20, 0x0C, 0x0F, 0x0C, 0x0E,
        0x0C, 0x0D, 0xA5, 0xF2, 0xFF, 0x2D, 0x0A, 0xC0,
        0x20, 0x00, 0x1D, 0xF0,
    };
    static const uint8_t sendto_fn[] = {
        0x36, 0xC1, 0x00, 0x20, 0xA2, 0x20, 0x79, 0xC1,
        0x25, 0xBD, 0xFE, 0x7D, 0x0A, 0x16, 0xDA, 0x03,
        0x98, 0x0A, 0xC8, 0x09, 0xC0, 0x94, 0x34, 0x88,
        0xC1, 0x66, 0x19, 0x14, 0xA5, 0xB4, 0xFE,
    };
    static const uint8_t send_fn[] = {
        0x36, 0x61, 0x00, 0x20, 0xA2, 0x20, 0xE5, 0xA6,
        0xFE, 0x40, 0x74, 0x20, 0xA0, 0x4A, 0x20, 0xCC,
        0x5A, 0x7C, 0xF2, 0xC0, 0x20, 0x00, 0x1D, 0xF0,
        0xA2, 0x2A, 0x00, 0x82, 0xA0, 0xF0,
    };
    static const uint8_t write_fn[] = {
        0x36, 0x41, 0x00, 0x20, 0xA2, 0x20, 0x30, 0xB3,
        0x20, 0x40, 0xC4, 0x20, 0x0C, 0x0D, 0x25, 0xEC,
        0xFF, 0x2D, 0x0A, 0xC0, 0x20, 0x00, 0x1D, 0xF0,
        0x36, 0x21, 0x01, 0x22, 0x61, 0x18, 0x4C, 0x08,
    };
    static const uint8_t dns[] = {
        0x36, 0x81, 0x00, 0x90, 0xF3, 0x40, 0x49, 0x21,
        0x7D, 0x03, 0x60, 0x60, 0x74, 0x90, 0x95, 0x41,
        0x59, 0x31, 0x16, 0x33, 0x34, 0x0C, 0x18, 0x20,
        0x89, 0x93, 0x56, 0xB8, 0x33, 0x92, 0x02, 0x00,
    };
    static const test_lwip_fingerprint_t functions[] = {
        { gethostbyname, sizeof(gethostbyname), 0x000u },
        { read_fn,       sizeof(read_fn),       0x080u },
        { sendto_fn,     sizeof(sendto_fn),     0x100u },
        { send_fn,       sizeof(send_fn),       0x180u },
        { write_fn,      sizeof(write_fn),      0x200u },
        { dns,           sizeof(dns),           0x280u },
    };
    for (size_t i = 0u; i < sizeof(functions) / sizeof(functions[0]); i++)
        put_test_bytes(cpu, base + functions[i].offset, functions[i].bytes,
                       functions[i].size);
}

#define TEST_IDF5_CLOSE_OFS   0x000u
#define TEST_IDF5_SOCKET_OFS  0x080u
#define TEST_IDF5_ACCEPT_OFS  0x100u
#define TEST_IDF5_BIND_OFS    0x180u
#define TEST_IDF5_LISTEN_OFS  0x200u
#define TEST_IDF5_RECV_OFS    0x280u

static void seed_relocated_idf5_server_members(xtensa_cpu_t *cpu,
                                               uint32_t base) {
    static const uint8_t close_fn[] = {
        0x36, 0xA1, 0x00, 0x20, 0xA2, 0x20, 0xA5, 0xF3,
        0xFE, 0x20, 0x42, 0x20, 0x7D, 0x0A, 0x16, 0xFA,
        0x0F, 0x88, 0x0A, 0x0C, 0x06, 0x8C, 0xE8, 0x98,
        0x08, 0x82, 0xA0, 0xF0, 0x90, 0x88, 0x10,
    };
    static const uint8_t socket_template[] = {
        0x36, 0x41, 0x00, 0x26, 0x23, 0x3D, 0x26, 0x33,
        0x15, 0x26, 0x13, 0x56, 0x91, 0x00, 0x00, 0x70,
        0x8E, 0xE3, 0x9A, 0x88, 0x1C, 0x69, 0x99, 0x08,
        0x7C, 0xF2, 0xC0, 0x20, 0x00, 0x1D, 0xF0,
    };
    static const uint8_t accept_fn[] = {
        0x36, 0xC1, 0x00, 0x82, 0xA0, 0x00, 0xAD, 0x02,
        0x82, 0x51, 0x1C, 0xC0, 0x20, 0x00, 0x25, 0x19,
        0xFF, 0x7D, 0x03, 0x6D, 0x04, 0x3D, 0x0A, 0xAC,
        0x4A, 0xA8, 0x0A, 0xB2, 0xC1, 0x34,
    };
    static const uint8_t bind_fn[] = {
        0x36, 0x81, 0x00, 0x20, 0xA2, 0x20, 0xA5, 0xFF,
        0xFE, 0xA0, 0x2A, 0x20, 0x16, 0x1A, 0x05, 0x82,
        0x03, 0x01, 0x66, 0x28, 0x24, 0x82, 0x2A, 0x00,
        0x88, 0x08, 0x37, 0xE8, 0x26, 0x26, 0xB4, 0x04,
    };
    static const uint8_t listen_fn[] = {
        0x36, 0x41, 0x00, 0x20, 0xA2, 0x20, 0xE5, 0xD3,
        0xFE, 0x2D, 0x0A, 0xAC, 0xCA, 0x0C, 0x08, 0x80,
        0x33, 0x53, 0xB2, 0xA0, 0xFF, 0xA8, 0x0A, 0xB0,
        0xB3, 0x43, 0xE5, 0xC0, 0x10, 0x16, 0x0A, 0x04,
    };
    static const uint8_t recv_fn[] = {
        0x36, 0x41, 0x00, 0x20, 0xA2, 0x20, 0x30, 0xB3,
        0x20, 0x40, 0xC4, 0x20, 0xDD, 0x05, 0x0C, 0x0F,
        0x0C, 0x0E, 0xE5, 0xF0, 0xFF, 0x2D, 0x0A, 0x1D,
        0xF0, 0x00, 0x00, 0x00,
    };
    const uint32_t socket_addr = base + TEST_IDF5_SOCKET_OFS;
    const uint32_t literal_addr = base + 0x060u;
    uint8_t socket_fn[sizeof(socket_template)];
    memcpy(socket_fn, socket_template, sizeof(socket_fn));
    uint32_t displacement = literal_addr - ((socket_addr + 15u) & ~3u);
    uint16_t immediate = (uint16_t)(displacement >> 2);
    socket_fn[13] = (uint8_t)immediate;
    socket_fn[14] = (uint8_t)(immediate >> 8);

    put_test_bytes(cpu, base + TEST_IDF5_CLOSE_OFS,
                   close_fn, sizeof(close_fn));
    put_test_bytes(cpu, socket_addr, socket_fn, sizeof(socket_fn));
    put_test_bytes(cpu, base + TEST_IDF5_ACCEPT_OFS,
                   accept_fn, sizeof(accept_fn));
    put_test_bytes(cpu, base + TEST_IDF5_BIND_OFS,
                   bind_fn, sizeof(bind_fn));
    put_test_bytes(cpu, base + TEST_IDF5_LISTEN_OFS,
                   listen_fn, sizeof(listen_fn));
    put_test_bytes(cpu, base + TEST_IDF5_RECV_OFS,
                   recv_fn, sizeof(recv_fn));
    mem_write32(cpu->mem, literal_addr, 8u);
}
#define TEST_REENT_ERRNO              0x3FFE3C00u
#define TEST_TASK_ERRNO               0x3FFE3D00u
#define TEST_TASK_TLS                 0x3FFE3E00u
#define TEST_TASK_TLS_ERRNO           (TEST_TASK_TLS + 8u)

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

    ASSERT_EQ(wifi_stubs_hook_firmware(wifi, TEST_MARAUDER_ENTRY), 15);

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

    ASSERT_EQ(wifi_stubs_hook_firmware(wifi, TEST_MARAUDER_ENTRY), 15);
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

TEST(dns_override_applies_to_raw_lwip_dns_api) {
    xtensa_cpu_t cpu;
    setup(&cpu);
    esp32_rom_stubs_t *rom = rom_stubs_create(&cpu);
    wifi_stubs_t *wifi = wifi_stubs_create(&cpu);
    const uint32_t name_addr = 0x3FFB1000u;
    const uint32_t result_addr = 0x3FFB1100u;
    static const char hostname[] = "must-not-resolve.invalid";
    static const uint8_t loopback_bytes[] = {127u, 0u, 0u, 1u};
    uint32_t loopback = 0;
    const uint32_t family = BASE + 0x1000u;

    memcpy(&loopback, loopback_bytes, sizeof(loopback));
    seed_relocated_idf4_lwip_family(&cpu, family);
    ASSERT_EQ(wifi_stubs_hook_firmware(wifi, TEST_NERDMINER_ENTRY), 16);
    wifi_stubs_set_dns_override(wifi, loopback);
    put_test_bytes(&cpu, name_addr, (const uint8_t *)hostname,
                   sizeof(hostname));
    mem_write32(cpu.mem, result_addr, 0xA5A5A5A5u);

    invoke_wifi_call0_4(&cpu, family + 0x280u, name_addr, result_addr,
                        0u, 0u);

    ASSERT_EQ(ar_read(&cpu, 2), 0u); /* lwIP ERR_OK */
    ASSERT_EQ(mem_read32(cpu.mem, result_addr), loopback);
    wifi_stubs_stats_t stats = {0};
    wifi_stubs_get_stats(wifi, &stats);
    ASSERT_EQ64(stats.dns_calls, 1u);

    wifi_stubs_destroy(wifi);
    rom_stubs_destroy(rom);
    teardown(&cpu);
}

TEST(stripped_idf4_lwip_family_relocates_without_firmware_profile) {
    xtensa_cpu_t cpu;
    setup(&cpu);
    esp32_rom_stubs_t *rom = rom_stubs_create(&cpu);
    wifi_stubs_t *wifi = wifi_stubs_create(&cpu);
    const uint32_t family = BASE + 0x1000u;
    const uint32_t dns_addr = family + 0x280u;
    const uint32_t name_addr = 0x3FFB1000u;
    const uint32_t result_addr = 0x3FFB1100u;
    static const char hostname[] = "relocated.invalid";
    static const uint8_t loopback_bytes[] = {127u, 0u, 0u, 1u};
    uint32_t loopback = 0u;

    seed_relocated_idf4_lwip_family(&cpu, family);

    ASSERT_TRUE(firmware_xtensa_reloc_crc32_matches(
            cpu.mem, family + 0x000u, 32u, 0xC5D40426u));
    ASSERT_TRUE(firmware_xtensa_reloc_crc32_matches(
            cpu.mem, family + 0x080u, 28u, 0xCF4C91E4u));
    ASSERT_TRUE(firmware_xtensa_reloc_crc32_matches(
            cpu.mem, family + 0x100u, 32u, 0xC9D7E309u));
    ASSERT_TRUE(firmware_xtensa_reloc_crc32_matches(
            cpu.mem, family + 0x180u, 31u, 0x0A84DC55u));
    ASSERT_TRUE(firmware_xtensa_reloc_crc32_matches(
            cpu.mem, family + 0x200u, 30u, 0x188638AAu));
    ASSERT_TRUE(firmware_xtensa_reloc_crc32_matches(
            cpu.mem, family + 0x280u, 30u, 0xCDFF35FFu));

    /* Five correct functions and one damaged instruction are not a family;
     * discovery is transactional and installs none of them. */
    mem_write8(cpu.mem, family + 0x200u + 3u, 0xC2u);
    ASSERT_EQ(wifi_stubs_hook_firmware(
                      wifi, 0x40081234u), 0);
    mem_write8(cpu.mem, family + 0x200u + 3u, 0xD2u);

    ASSERT_EQ(wifi_stubs_hook_firmware(
                      wifi, 0x40081234u), 6);
    memcpy(&loopback, loopback_bytes, sizeof(loopback));
    wifi_stubs_set_dns_override(wifi, loopback);
    put_test_bytes(&cpu, name_addr, (const uint8_t *)hostname,
                   sizeof(hostname));
    invoke_wifi_call0_4(&cpu, dns_addr, name_addr, result_addr, 0u, 0u);
    ASSERT_EQ(ar_read(&cpu, 2), 0u);
    ASSERT_EQ(mem_read32(cpu.mem, result_addr), loopback);

    wifi_stubs_stats_t stats = {0};
    wifi_stubs_get_stats(wifi, &stats);
    ASSERT_EQ64(stats.dns_calls, 1u);

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

    ASSERT_EQ(wifi_stubs_hook_firmware(wifi, TEST_MARAUDER_ENTRY), 15);
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

    ASSERT_EQ(wifi_stubs_hook_firmware(wifi, 0x40081E90u), 15);
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
    const uint32_t sockets = BASE + 0x1400u;
    static const char name[] = "WIFI_EVENT";

    seed_relocated_idf4_lwip_family(&cpu, BASE + 0x1000u);
    seed_relocated_idf4_socket_members(&cpu, sockets);
    ASSERT_EQ(wifi_stubs_hook_firmware(wifi, TEST_WLED_ENTRY), 26);
    ASSERT_EQ(rom_stubs_register_ctx(rom, TEST_WLED_EVENT_POST,
                                     capture_native_event_post,
                                     "test_event_post", &capture), 0);

    /* The symbol-less production image's WiFiUDP boundary is hooked too. */
    invoke_wifi_call0_4(&cpu, sockets + 0x80u, 2u, 2u, 0u, 0u);
    uint32_t socket_fd = ar_read(&cpu, 2);
    ASSERT_EQ(socket_fd, 46u); /* first ESP-IDF/LWIP_SOCKET_OFFSET slot */
    invoke_wifi_call0(&cpu, sockets, socket_fd);
    ASSERT_EQ(ar_read(&cpu, 2), 0u);
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

    /* Event production is independent from delivery. An external callback
     * must remain queued while guest code is active, then wake a core only at
     * a genuine WAITI boundary. */
    cpu.running = true;
    cpu.halted = false;
    cpu.exception = false;
    cpu.ps = 1u << 18; /* WOE, INTLEVEL=0, EXCM=0 */
    wifi_stubs_tick(wifi, &cpu, NULL);
    ASSERT_EQ64(capture.calls, 0u);
    wifi_stubs_stats_t stats = {0};
    wifi_stubs_get_stats(wifi, &stats);
    ASSERT_EQ64(stats.events_delivered, 0u);

    cpu.halted = true;
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

    wifi_stubs_get_stats(wifi, &stats);
    ASSERT_EQ64(stats.events_delivered, 1u);
    ASSERT_EQ64(stats.socket_calls, 1u);
    ASSERT_EQ64(stats.socket_successes, 1u);
    ASSERT_EQ64(stats.close_calls, 1u);

    wifi_stubs_destroy(wifi);
    rom_stubs_destroy(rom);
    teardown(&cpu);
}

TEST(stripped_idf4_socket_members_need_no_application_profile) {
    xtensa_cpu_t cpu;
    setup(&cpu);
    esp32_rom_stubs_t *rom = rom_stubs_create(&cpu);
    wifi_stubs_t *wifi = wifi_stubs_create(&cpu);
    const uint32_t family = BASE + 0x1000u;
    const uint32_t sockets = BASE + 0x1400u;

    seed_relocated_idf4_lwip_family(&cpu, family);
    seed_relocated_idf4_socket_members(&cpu, sockets);
    ASSERT_EQ(wifi_stubs_hook_firmware(wifi, 0x40081234u), 8);
    invoke_wifi_call0_4(&cpu, sockets + 0x80u, 10u, 1u, 0u, 0u);
    uint32_t socket_fd = ar_read(&cpu, 2);
    ASSERT_EQ(socket_fd, 46u);
    invoke_wifi_call0(&cpu, sockets, socket_fd);
    ASSERT_EQ(ar_read(&cpu, 2), 0u);

    /* A native socket error is written through the firmware's relocated
     * __errno accessor. The compatibility scratch word must remain untouched,
     * proving that discovering socket acceleration did not replace newlib's
     * process-wide accessor or collapse task-local errno state. */
    mem_write32(cpu.mem, TEST_REENT_ERRNO, 0x11223344u);
    mem_write32(cpu.mem, TEST_TASK_ERRNO, 0u);
    invoke_wifi_call0_4(&cpu, family + 0x80u, 123u,
                        BASE + 0x1C00u, 1u, 0u);
    ASSERT_EQ(ar_read(&cpu, 2), UINT32_MAX);
    ASSERT_EQ(mem_read32(cpu.mem, TEST_TASK_ERRNO), 9u);
    ASSERT_EQ(mem_read32(cpu.mem, TEST_REENT_ERRNO), 0x11223344u);

    wifi_stubs_stats_t stats = {0};
    wifi_stubs_get_stats(wifi, &stats);
    ASSERT_EQ64(stats.socket_calls, 1u);
    ASSERT_EQ64(stats.socket_successes, 1u);
    ASSERT_EQ64(stats.close_calls, 1u);

    wifi_stubs_destroy(wifi);
    rom_stubs_destroy(rom);
    teardown(&cpu);
}

TEST(stripped_idf5_picolibc_family_relocates_without_profile) {
    xtensa_cpu_t cpu;
    setup(&cpu);
    esp32_rom_stubs_t *rom = rom_stubs_create(&cpu);
    wifi_stubs_t *wifi = wifi_stubs_create(&cpu);
    const uint32_t family = BASE + 0x1000u;
    const uint32_t sockets = BASE + 0x1400u;

    seed_relocated_idf5_lwip_family(&cpu, family);
    seed_relocated_idf5_server_members(&cpu, sockets);
    cpu.threadptr = TEST_TASK_TLS;

    ASSERT_TRUE(firmware_xtensa_reloc_crc32_matches(
            cpu.mem, family, 32u, 0x1C9CFE9Eu));
    ASSERT_TRUE(firmware_xtensa_reloc_crc32_matches(
            cpu.mem, sockets + TEST_IDF5_SOCKET_OFS, 31u, 0x5D6DBF97u));

    /* The code fingerprints alone cannot authorize a bridge whose task-local
     * errno ABI cannot be recovered. Literal contents are link data and are
     * deliberately outside the normalized instruction hash. */
    mem_write32(cpu.mem, sockets + 0x060u, 0x3FF00000u);
    ASSERT_EQ(wifi_stubs_hook_firmware(wifi, 0x40081234u), 0);
    mem_write32(cpu.mem, sockets + 0x060u, 8u);
    ASSERT_EQ(wifi_stubs_hook_firmware(wifi, 0x40081234u), 12);

    invoke_wifi_call0_4(&cpu, sockets + TEST_IDF5_SOCKET_OFS,
                        2u, 1u, 0u, 0u);
    uint32_t socket_fd = ar_read(&cpu, 2);
    ASSERT_EQ(socket_fd, 46u);
    invoke_wifi_call0(&cpu, sockets + TEST_IDF5_CLOSE_OFS, socket_fd);
    ASSERT_EQ(ar_read(&cpu, 2), 0u);

    /* The linked TLS literal, not a profile or fixed RAM word, selects the
     * calling task's picolibc errno slot. */
    mem_write32(cpu.mem, TEST_REENT_ERRNO, 0x11223344u);
    mem_write32(cpu.mem, TEST_TASK_TLS_ERRNO, 0u);
    invoke_wifi_call0_4(&cpu, family + 0x080u, 123u,
                        BASE + 0x1C00u, 1u, 0u);
    ASSERT_EQ(ar_read(&cpu, 2), UINT32_MAX);
    ASSERT_EQ(mem_read32(cpu.mem, TEST_TASK_TLS_ERRNO), 9u);
    ASSERT_EQ(mem_read32(cpu.mem, TEST_REENT_ERRNO), 0x11223344u);

    wifi_stubs_stats_t stats = {0};
    wifi_stubs_get_stats(wifi, &stats);
    ASSERT_EQ64(stats.socket_calls, 1u);
    ASSERT_EQ64(stats.socket_successes, 1u);
    ASSERT_EQ64(stats.close_calls, 1u);

    wifi_stubs_destroy(wifi);
    rom_stubs_destroy(rom);
    teardown(&cpu);
}

TEST(idf5_picolibc_recv_peek_preserves_http_request) {
#ifndef _WIN32
    xtensa_cpu_t cpu;
    setup(&cpu);
    esp32_rom_stubs_t *rom = rom_stubs_create(&cpu);
    wifi_stubs_t *wifi = wifi_stubs_create(&cpu);
    const uint32_t family = BASE + 0x1000u;
    const uint32_t sockets = BASE + 0x1400u;
    const uint32_t sockaddr_addr = 0x3FFB1000u;
    const uint32_t sockaddr_len_addr = 0x3FFB1020u;
    const uint32_t recv_addr = 0x3FFB1040u;

    seed_relocated_idf5_lwip_family(&cpu, family);
    seed_relocated_idf5_server_members(&cpu, sockets);
    cpu.threadptr = TEST_TASK_TLS;
    ASSERT_EQ(wifi_stubs_hook_firmware(wifi, 0x40081234u), 12);
    invoke_wifi_call0_4(&cpu, sockets + TEST_IDF5_SOCKET_OFS,
                        2u, 1u, 0u, 0u);
    uint32_t listen_fd = ar_read(&cpu, 2);
    ASSERT_EQ(listen_fd, 46u);

    mem_write8(cpu.mem, sockaddr_addr + 0u, 16u);
    mem_write8(cpu.mem, sockaddr_addr + 1u, 2u);
    mem_write16(cpu.mem, sockaddr_addr + 2u, htons(80));
    mem_write32(cpu.mem, sockaddr_addr + 4u, htonl(INADDR_ANY));
    invoke_wifi_call0_4(&cpu, sockets + TEST_IDF5_BIND_OFS, listen_fd,
                        sockaddr_addr, 16u, 0u);
    ASSERT_EQ(ar_read(&cpu, 2), 0u);
    invoke_wifi_call0_4(&cpu, sockets + TEST_IDF5_LISTEN_OFS,
                        listen_fd, 1u, 0u, 0u);
    ASSERT_EQ(ar_read(&cpu, 2), 0u);

    uint16_t host_port = 0;
    ASSERT_EQ(wifi_stubs_get_bound_host_port(wifi, 80, false, &host_port), 0);
    int host_fd = socket(AF_INET, SOCK_STREAM, 0);
    ASSERT_TRUE(host_fd >= 0);
    struct sockaddr_in peer = {
        .sin_family = AF_INET,
        .sin_port = htons(host_port),
        .sin_addr.s_addr = htonl(INADDR_LOOPBACK),
    };
    ASSERT_EQ(connect(host_fd, (struct sockaddr *)&peer, sizeof(peer)), 0);
    ASSERT_EQ(send(host_fd, "GET", 3, 0), 3);

    mem_write32(cpu.mem, sockaddr_len_addr, 16u);
    uint32_t client_fd = UINT32_MAX;
    for (int attempt = 0; attempt < 100; attempt++) {
        invoke_wifi_call0_4(&cpu, sockets + TEST_IDF5_ACCEPT_OFS, listen_fd,
                            sockaddr_addr, sockaddr_len_addr, 0u);
        client_fd = ar_read(&cpu, 2);
        if (client_fd != UINT32_MAX) break;
        poll(NULL, 0, 1);
    }
    ASSERT_EQ(client_fd, 47u);

    /* lwIP's MSG_PEEK=0x01 and MSG_DONTWAIT=0x08 are not Darwin's flag
     * values. Two NetworkClient::connected() probes must leave the first
     * byte untouched for WebServer's parser. */
    uint32_t first_recv = 0;
    for (int attempt = 0; attempt < 100; attempt++) {
        invoke_wifi_call0_4(&cpu, sockets + TEST_IDF5_RECV_OFS, client_fd,
                            recv_addr, 1u, 0x09u);
        first_recv = ar_read(&cpu, 2);
        if (first_recv != UINT32_MAX) break;
        poll(NULL, 0, 1);
    }
    ASSERT_EQ(first_recv, 1u);
    ASSERT_EQ(mem_read8(cpu.mem, recv_addr), 'G');
    invoke_wifi_call0_4(&cpu, sockets + TEST_IDF5_RECV_OFS, client_fd,
                        recv_addr, 1u, 0x09u);
    ASSERT_EQ(ar_read(&cpu, 2), 1u);
    ASSERT_EQ(mem_read8(cpu.mem, recv_addr), 'G');
    invoke_wifi_call0_4(&cpu, sockets + TEST_IDF5_RECV_OFS, client_fd,
                        recv_addr, 1u, 0x08u);
    ASSERT_EQ(ar_read(&cpu, 2), 1u);
    ASSERT_EQ(mem_read8(cpu.mem, recv_addr), 'G');
    invoke_wifi_call0_4(&cpu, sockets + TEST_IDF5_RECV_OFS, client_fd,
                        recv_addr, 1u, 0x08u);
    ASSERT_EQ(ar_read(&cpu, 2), 1u);
    ASSERT_EQ(mem_read8(cpu.mem, recv_addr), 'E');
    invoke_wifi_call0_4(&cpu, sockets + TEST_IDF5_RECV_OFS, client_fd,
                        recv_addr, 1u, 0x08u);
    ASSERT_EQ(ar_read(&cpu, 2), 1u);
    ASSERT_EQ(mem_read8(cpu.mem, recv_addr), 'T');

    shutdown(host_fd, SHUT_RDWR);
    close(host_fd);
    /* The accepted host descriptor is private to wifi_stubs, so wait for the
     * FIN through the guest boundary below; a short scheduling race may still
     * report EWOULDBLOCK once before the EOF becomes visible. */
    /* A preceding empty nonblocking read commonly leaves EWOULDBLOCK in the
     * guest. Orderly EOF must clear it or NetworkClient::connected() treats
     * the closed peer as live forever. */
    mem_write32(cpu.mem, TEST_REENT_ERRNO, 0x11223344u);
    mem_write32(cpu.mem, TEST_TASK_TLS_ERRNO, 11u);
    for (int attempt = 0; attempt < 100; attempt++) {
        invoke_wifi_call0_4(&cpu, sockets + TEST_IDF5_RECV_OFS, client_fd,
                            recv_addr, 1u, 0x09u);
        if (ar_read(&cpu, 2) == 0u) break;
        poll(NULL, 0, 1);
    }
    ASSERT_EQ(ar_read(&cpu, 2), 0u);
    ASSERT_EQ(mem_read32(cpu.mem, TEST_TASK_TLS_ERRNO), 0u);
    ASSERT_EQ(mem_read32(cpu.mem, TEST_REENT_ERRNO), 0x11223344u);
    invoke_wifi_call0(&cpu, sockets + TEST_IDF5_CLOSE_OFS, client_fd);
    invoke_wifi_call0(&cpu, sockets + TEST_IDF5_CLOSE_OFS, listen_fd);
    wifi_stubs_destroy(wifi);
    rom_stubs_destroy(rom);
    teardown(&cpu);
#endif
}

static void run_wifi_stub_tests(void) {
    TEST_SUITE("WiFi stubs");
    RUN_TEST(promiscuous_frame_requires_enabled_callback);
    RUN_TEST(promiscuous_frame_runs_callback_and_restores_cpu);
    RUN_TEST(raw_tx_crosses_host_radio_boundary);
    RUN_TEST(dns_override_applies_to_raw_lwip_dns_api);
    RUN_TEST(stripped_idf4_lwip_family_relocates_without_firmware_profile);
    RUN_TEST(v11423_fingerprint_selects_shifted_wifi_entries);
    RUN_TEST(v1121_cyd2usb_fingerprint_selects_idf55_wifi_entries);
    RUN_TEST(wled_posts_disconnect_on_native_event_loop);
    RUN_TEST(stripped_idf4_socket_members_need_no_application_profile);
    RUN_TEST(stripped_idf5_picolibc_family_relocates_without_profile);
    RUN_TEST(idf5_picolibc_recv_peek_preserves_http_request);
}
