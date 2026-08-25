/*
 * Headless end-to-end smoke test for external production CYD ROMs.
 *
 * This deliberately lives outside the always-on unit suite: the firmware
 * images are redistributable artifacts supplied by the caller, not fixtures
 * checked into Flexe. It exercises the same flexe_session API as GUI clients,
 * including dual-core execution, raw SPI display capture, and touch input.
 */

#include "flexe_session.h"

#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <time.h>
#include <unistd.h>

#define FB_W 320
#define FB_H 240
#define FB_PIXELS (FB_W * FB_H)
#define RUN_BATCH 10000

#define NERDMINER_SPIFFS_OFFSET 0x310000u
#define NERDMINER_SPIFFS_SIZE   0x0E0000u
#define NERDMINER_SPIFFS_BLOCK  0x1000u
#define NERDMINER_SPIFFS_PAGE   0x0100u
#define SPIFFS_MAGIC             0x0529u
#define STOCK_SD_BYTES           (16u * 1024u * 1024u)
#define STOCK_SD_SECTORS         (STOCK_SD_BYTES / 512u)
#define STOCK_SD_ROOT_SECTOR     65u

/* Verified production v1.14 WiFiScan layout. */
#define MARAUDER_MGMT_FRAMES_ADDR   0x3FFC8C98u
#define MARAUDER_DATA_FRAMES_ADDR   0x3FFC8C9Cu
#define MARAUDER_BEACON_FRAMES_ADDR 0x3FFC8CA0u
#define MARAUDER_SCAN_MODE_ADDR     0x3FFC9384u
#define MARAUDER_RAW_SCAN_MODE      25u
#define MARAUDER_RICKROLL_MODE      9u
#define MARAUDER_BT_SCAN_MODE       10u

/* touch_stubs uses the frontend's lifetime flag while servicing blocking
 * touch APIs. Standalone main and the SDL frontend provide the same symbol. */
volatile int emu_app_running = 1;

typedef struct {
    int pressed;
    int x;
    int y;
} touch_state_t;

typedef struct {
    uint64_t count;
    char log[4096];
    size_t log_len;
} uart_state_t;

typedef struct {
    int tcp_fd;
    int udp_fd;
    uint16_t tcp_port;
    uint16_t udp_port;
    uint8_t http_response[8192];
    size_t http_len;
    uint8_t dns_response[512];
    size_t dns_len;
} nerd_network_probe_t;

typedef struct {
    uint64_t frames;
    uint64_t bytes;
    uint64_t beacon_frames;
    uint32_t last_iface;
    size_t last_len;
    bool last_en_sys_seq;
} raw_tx_probe_t;

#define MAX_UNREGISTERED_ROM_ADDRS 32
typedef struct {
    uint32_t addr[MAX_UNREGISTERED_ROM_ADDRS];
    uint32_t caller[MAX_UNREGISTERED_ROM_ADDRS];
    uint32_t count[MAX_UNREGISTERED_ROM_ADDRS];
    int used;
} rom_audit_t;

static int touch_read(int *x, int *y, void *ctx)
{
    touch_state_t *touch = ctx;
    *x = touch->x;
    *y = touch->y;
    return touch->pressed;
}

static void uart_count(void *ctx, uint8_t byte)
{
    uart_state_t *uart = ctx;
    uart->count++;
    if (uart->log_len + 1 < sizeof(uart->log)) {
        uart->log[uart->log_len++] = (char)byte;
        uart->log[uart->log_len] = '\0';
    }
}

static void capture_raw_tx(void *ctx, uint32_t iface, const uint8_t *frame,
                           size_t len, bool en_sys_seq)
{
    raw_tx_probe_t *probe = ctx;
    probe->frames++;
    probe->bytes += len;
    if (len >= 24 && frame[0] == 0x80)
        probe->beacon_frames++;
    probe->last_iface = iface;
    probe->last_len = len;
    probe->last_en_sys_seq = en_sys_seq;
}

static void audit_rom_call(void *ctx, uint32_t addr, const char *name,
                           const xtensa_cpu_t *cpu)
{
    (void)cpu;
    rom_audit_t *audit = ctx;
    if (strcmp(name, "UNREGISTERED") != 0)
        return;
    for (int i = 0; i < audit->used; i++) {
        if (audit->addr[i] == addr) {
            audit->count[i]++;
            return;
        }
    }
    if (audit->used < MAX_UNREGISTERED_ROM_ADDRS) {
        audit->addr[audit->used] = addr;
        int callinc = XT_PS_CALLINC(cpu->ps);
        audit->caller[audit->used] =
                (cpu->pc & 0xC0000000u) |
                (ar_read(cpu, callinc * 4) & 0x3FFFFFFFu);
        audit->count[audit->used] = 1;
        audit->used++;
    }
}

static void put_le16(uint8_t *dst, uint16_t value)
{
    dst[0] = (uint8_t)value;
    dst[1] = (uint8_t)(value >> 8);
}

static void put_le32(uint8_t *dst, uint32_t value)
{
    dst[0] = (uint8_t)value;
    dst[1] = (uint8_t)(value >> 8);
    dst[2] = (uint8_t)(value >> 16);
    dst[3] = (uint8_t)(value >> 24);
}

static int write_all_at(int fd, const void *buf, size_t len, off_t offset)
{
    const uint8_t *src = buf;
    while (len > 0) {
        ssize_t wrote = pwrite(fd, src, len, offset);
        if (wrote < 0 && errno == EINTR) continue;
        if (wrote <= 0) return -1;
        src += wrote;
        len -= (size_t)wrote;
        offset += wrote;
    }
    return 0;
}

/* Build a deterministic, empty FAT16 superfloppy.  Supplying a valid medium
 * makes the production-ROM gate exercise SD command/sector reads and FatFs
 * mounting instead of merely confirming the expected error for an all-zero
 * virtual card. */
static int create_stock_sd_image(char path[64])
{
    strcpy(path, "/tmp/flexe-stock-sd-XXXXXX");
    int fd = mkstemp(path);
    if (fd < 0) return -1;
    if (ftruncate(fd, STOCK_SD_BYTES) != 0) goto fail;

    uint8_t boot[512] = {0};
    boot[0] = 0xEB;
    boot[1] = 0x3C;
    boot[2] = 0x90;
    memcpy(boot + 3, "FLEXE   ", 8);
    put_le16(boot + 11, 512);                    /* bytes per sector */
    boot[13] = 4;                                /* sectors per cluster */
    put_le16(boot + 14, 1);                      /* reserved sectors */
    boot[16] = 2;                                /* FAT copies */
    put_le16(boot + 17, 512);                    /* root entries */
    put_le16(boot + 19, STOCK_SD_SECTORS);       /* total sectors */
    boot[21] = 0xF8;                             /* fixed disk */
    put_le16(boot + 22, 32);                     /* sectors per FAT */
    put_le16(boot + 24, 63);                     /* sectors per track */
    put_le16(boot + 26, 255);                    /* heads */
    boot[36] = 0x80;
    boot[38] = 0x29;
    put_le32(boot + 39, 0x46584C45u);
    memcpy(boot + 43, "FLEXE SD   ", 11);
    memcpy(boot + 54, "FAT16   ", 8);
    put_le16(boot + 510, 0xAA55);
    if (write_all_at(fd, boot, sizeof(boot), 0) != 0) goto fail;

    uint8_t fat[32 * 512] = {0};
    put_le16(fat + 0, 0xFFF8);
    put_le16(fat + 2, 0xFFFF);
    if (write_all_at(fd, fat, sizeof(fat), 1 * 512) != 0 ||
        write_all_at(fd, fat, sizeof(fat), 33 * 512) != 0)
        goto fail;

    if (close(fd) != 0) {
        unlink(path);
        return -1;
    }
    return 0;

fail:
    close(fd);
    unlink(path);
    return -1;
}

static int fat16_root_has_directory(const char *path, const char name[11])
{
    int fd = open(path, O_RDONLY);
    if (fd < 0) return -1;
    uint8_t root[512];
    ssize_t got;
    do {
        got = pread(fd, root, sizeof(root),
                    (off_t)STOCK_SD_ROOT_SECTOR * 512);
    } while (got < 0 && errno == EINTR);
    close(fd);
    if (got != (ssize_t)sizeof(root)) return -1;
    for (size_t off = 0; off < sizeof(root); off += 32) {
        if (root[off] == 0x00) break;
        if (root[off] == 0xE5 || root[off + 11] == 0x0F) continue;
        if ((root[off + 11] & 0x10) && memcmp(root + off, name, 11) == 0)
            return 1;
    }
    return 0;
}

static uint64_t monotonic_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ull + (uint64_t)ts.tv_nsec;
}

static int framebuffer_nonblack(const uint16_t *fb, pthread_mutex_t *mutex)
{
    int count = 0;
    pthread_mutex_lock(mutex);
    for (int i = 0; i < FB_PIXELS; i++)
        count += fb[i] != 0;
    pthread_mutex_unlock(mutex);
    return count;
}

static int framebuffer_diff(const uint16_t *a, const uint16_t *b,
                            pthread_mutex_t *mutex)
{
    int count = 0;
    pthread_mutex_lock(mutex);
    for (int i = 0; i < FB_PIXELS; i++)
        count += a[i] != b[i];
    pthread_mutex_unlock(mutex);
    return count;
}

static void framebuffer_copy(uint16_t *dst, const uint16_t *src,
                             pthread_mutex_t *mutex)
{
    pthread_mutex_lock(mutex);
    memcpy(dst, src, FB_PIXELS * sizeof(*dst));
    pthread_mutex_unlock(mutex);
}

static int session_alive(flexe_session_t *session)
{
    xtensa_cpu_t *cpu0 = flexe_session_cpu(session, 0);
    xtensa_cpu_t *cpu1 = flexe_session_cpu(session, 1);
    return (cpu0 && cpu0->running) || (cpu1 && cpu1->running);
}

/* NerdMiner's factory image intentionally leaves its SPIFFS partition erased.
 * Its `SPIFFS.begin(false) || SPIFFS.begin(true)` path must therefore format
 * the real emulated flash before the UI is useful.  Verify every block magic,
 * rather than treating the expected first-mount error message as a failure or
 * papering the operation over with a high-level VFS hook. */
static int nerdminer_spiffs_formatted(flexe_session_t *session)
{
    xtensa_mem_t *mem = flexe_session_mem(session);
    if (!mem || !mem->flash_data) return -1;

    const uint32_t blocks = NERDMINER_SPIFFS_SIZE / NERDMINER_SPIFFS_BLOCK;
    for (uint32_t block = 0; block < blocks; block++) {
        uint32_t marker_offset = NERDMINER_SPIFFS_OFFSET +
                                 block * NERDMINER_SPIFFS_BLOCK +
                                 NERDMINER_SPIFFS_PAGE - sizeof(uint16_t) * 2;
        uint16_t actual;
        memcpy(&actual, mem->flash_data + marker_offset, sizeof(actual));
        uint16_t expected = (uint16_t)((blocks - block) ^
                                       NERDMINER_SPIFFS_PAGE ^ SPIFFS_MAGIC);
        if (actual != expected)
            return (int)block;
    }
    return (int)blocks;
}

static void run_one_batch(flexe_session_t *session)
{
    xtensa_cpu_t *cpu0 = flexe_session_cpu(session, 0);
    if (cpu0 && cpu0->running)
        (void)flexe_session_run_core(session, 0, RUN_BATCH);
    flexe_session_post_batch(session, RUN_BATCH);
}

static int run_for_virtual_cycles(flexe_session_t *session, uint64_t cycles)
{
    xtensa_cpu_t *cpu0 = flexe_session_cpu(session, 0);
    uint64_t target = cpu0->cycle_count + cycles;
    while (cpu0->cycle_count < target) {
        if (!session_alive(session)) return -1;
        run_one_batch(session);
    }
    return 0;
}

static int run_until_screen(flexe_session_t *session, const uint16_t *fb,
                            pthread_mutex_t *mutex, int min_nonblack,
                            uint64_t max_cycles, int *nonblack_out,
                            uint64_t *cycles_out)
{
    xtensa_cpu_t *cpu0 = flexe_session_cpu(session, 0);
    uint64_t start = cpu0->cycle_count;
    unsigned batches = 0;

    while (cpu0->cycle_count - start < max_cycles) {
        if (!session_alive(session)) return -1;
        run_one_batch(session);
        if (++batches % 100 == 0) {
            int nonblack = framebuffer_nonblack(fb, mutex);
            if (nonblack >= min_nonblack) {
                *nonblack_out = nonblack;
                *cycles_out = cpu0->cycle_count - start;
                return 0;
            }
        }
    }

    *nonblack_out = framebuffer_nonblack(fb, mutex);
    *cycles_out = cpu0->cycle_count - start;
    return 1;
}

static int run_until_changed(flexe_session_t *session, const uint16_t *before,
                             const uint16_t *fb, pthread_mutex_t *mutex,
                             int min_changed, uint64_t max_cycles,
                             int *changed_out)
{
    xtensa_cpu_t *cpu0 = flexe_session_cpu(session, 0);
    uint64_t start = cpu0->cycle_count;
    unsigned batches = 0;

    while (cpu0->cycle_count - start < max_cycles) {
        if (!session_alive(session)) return -1;
        run_one_batch(session);
        if (++batches % 100 == 0) {
            int changed = framebuffer_diff(before, fb, mutex);
            if (changed >= min_changed) {
                *changed_out = changed;
                return 0;
            }
        }
    }

    *changed_out = framebuffer_diff(before, fb, mutex);
    return 1;
}

static bool uart_contains(const uart_state_t *uart, const char *needle)
{
    size_t needle_len = strlen(needle);
    if (needle_len == 0) return true;
    if (needle_len > uart->log_len) return false;
    for (size_t i = 0; i <= uart->log_len - needle_len; i++) {
        if (memcmp(uart->log + i, needle, needle_len) == 0)
            return true;
    }
    return false;
}

static bool uart_contains_from(const uart_state_t *uart, size_t start,
                               const char *needle)
{
    size_t needle_len = strlen(needle);
    if (start > uart->log_len || needle_len == 0)
        return needle_len == 0;
    size_t available = uart->log_len - start;
    if (needle_len > available)
        return false;
    for (size_t i = start; i <= uart->log_len - needle_len; i++) {
        if (memcmp(uart->log + i, needle, needle_len) == 0)
            return true;
    }
    return false;
}

static int run_until_marauder_sniffer(flexe_session_t *session,
                                      const uart_state_t *uart,
                                      const wifi_stubs_stats_t *before,
                                      uint64_t max_cycles,
                                      wifi_stubs_stats_t *stats_out,
                                      uint64_t *cycles_out)
{
    xtensa_cpu_t *cpu0 = flexe_session_cpu(session, 0);
    wifi_stubs_t *wifi = flexe_session_wifi(session);
    uint64_t start = cpu0->cycle_count;
    wifi_stubs_stats_t stats = {0};

    while (cpu0->cycle_count - start < max_cycles) {
        if (!session_alive(session)) return -1;
        run_one_batch(session);
        wifi_stubs_get_stats(wifi, &stats);
        if (uart_contains(uart, "#sniffraw") &&
            stats.wifi_init_calls > before->wifi_init_calls &&
            stats.wifi_start_calls > before->wifi_start_calls &&
            stats.wifi_set_mode_calls > before->wifi_set_mode_calls &&
            stats.promisc_enable_calls > before->promisc_enable_calls &&
            stats.promisc_callback_calls > before->promisc_callback_calls &&
            mem_read8(flexe_session_mem(session),
                      MARAUDER_SCAN_MODE_ADDR) == MARAUDER_RAW_SCAN_MODE) {
            *stats_out = stats;
            *cycles_out = cpu0->cycle_count - start;
            return 0;
        }
    }

    wifi_stubs_get_stats(wifi, stats_out);
    *cycles_out = cpu0->cycle_count - start;
    return 1;
}

static int run_until_marauder_stopped(flexe_session_t *session,
                                      const uart_state_t *uart,
                                      uint64_t max_cycles,
                                      uint64_t *cycles_out)
{
    xtensa_cpu_t *cpu0 = flexe_session_cpu(session, 0);
    uint64_t start = cpu0->cycle_count;

    while (cpu0->cycle_count - start < max_cycles) {
        if (!session_alive(session)) return -1;
        run_one_batch(session);
        if (uart_contains(uart, "#stopscan") &&
            mem_read8(flexe_session_mem(session),
                      MARAUDER_SCAN_MODE_ADDR) == 0) {
            *cycles_out = cpu0->cycle_count - start;
            return 0;
        }
    }

    *cycles_out = cpu0->cycle_count - start;
    return 1;
}

static int run_until_marauder_tx(flexe_session_t *session,
                                 const uart_state_t *uart,
                                 const wifi_stubs_stats_t *before,
                                 uint64_t max_cycles,
                                 wifi_stubs_stats_t *stats_out,
                                 uint64_t *cycles_out)
{
    xtensa_cpu_t *cpu0 = flexe_session_cpu(session, 0);
    wifi_stubs_t *wifi = flexe_session_wifi(session);
    uint64_t start = cpu0->cycle_count;
    wifi_stubs_stats_t stats = {0};

    while (cpu0->cycle_count - start < max_cycles) {
        if (!session_alive(session)) return -1;
        run_one_batch(session);
        wifi_stubs_get_stats(wifi, &stats);
        if (uart_contains(uart, "#attack -t rickroll") &&
            stats.raw_tx_frames > before->raw_tx_frames &&
            mem_read8(flexe_session_mem(session),
                      MARAUDER_SCAN_MODE_ADDR) == MARAUDER_RICKROLL_MODE) {
            *stats_out = stats;
            *cycles_out = cpu0->cycle_count - start;
            return 0;
        }
    }

    wifi_stubs_get_stats(wifi, stats_out);
    *cycles_out = cpu0->cycle_count - start;
    return 1;
}

static int run_until_marauder_bt_scan(flexe_session_t *session,
                                      const uart_state_t *uart,
                                      uint64_t max_cycles,
                                      bt_stubs_stats_t *stats_out,
                                      uint64_t *cycles_out)
{
    xtensa_cpu_t *cpu0 = flexe_session_cpu(session, 0);
    bt_stubs_t *bt = flexe_session_bt(session);
    uint64_t start = cpu0->cycle_count;
    bt_stubs_stats_t stats = {0};

    while (cpu0->cycle_count - start < max_cycles) {
        if (!session_alive(session)) return -1;
        run_one_batch(session);
        bt_stubs_get_stats(bt, &stats);
        if (uart_contains(uart, "#sniffbt") &&
            stats.scan_callback_config_calls > 0 &&
            stats.scan_start_calls > 0 &&
            mem_read8(flexe_session_mem(session),
                      MARAUDER_SCAN_MODE_ADDR) == MARAUDER_BT_SCAN_MODE) {
            *stats_out = stats;
            *cycles_out = cpu0->cycle_count - start;
            return 0;
        }
    }

    bt_stubs_get_stats(bt, stats_out);
    *cycles_out = cpu0->cycle_count - start;
    return 1;
}

static int run_until_nerd_network(flexe_session_t *session,
                                  uint64_t max_cycles,
                                  wifi_stubs_stats_t *stats_out,
                                  uint64_t *cycles_out)
{
    xtensa_cpu_t *cpu0 = flexe_session_cpu(session, 0);
    wifi_stubs_t *wifi = flexe_session_wifi(session);
    uint64_t start = cpu0->cycle_count;
    unsigned batches = 0;
    wifi_stubs_stats_t stats = {0};

    while (cpu0->cycle_count - start < max_cycles) {
        if (!session_alive(session)) return -1;
        run_one_batch(session);
        if (++batches % 100 == 0) {
            wifi_stubs_get_stats(wifi, &stats);
            if (stats.socket_successes >= 2 &&
                stats.bind_successes >= 2 &&
                stats.listen_successes >= 1) {
                *stats_out = stats;
                *cycles_out = cpu0->cycle_count - start;
                return 0;
            }
        }
    }

    wifi_stubs_get_stats(wifi, stats_out);
    *cycles_out = cpu0->cycle_count - start;
    return 1;
}

static void nerd_probe_close(nerd_network_probe_t *probe)
{
    if (probe->tcp_fd >= 0) close(probe->tcp_fd);
    if (probe->udp_fd >= 0) close(probe->udp_fd);
    probe->tcp_fd = -1;
    probe->udp_fd = -1;
}

static int nerd_probe_nonblocking(int fd)
{
    int flags = fcntl(fd, F_GETFL, 0);
    return flags < 0 ? -1 : fcntl(fd, F_SETFL, flags | O_NONBLOCK);
}

static int nerd_probe_send_all(int fd, const void *data, size_t len)
{
    const uint8_t *src = data;
    while (len > 0) {
        ssize_t sent = send(fd, src, len, 0);
        if (sent < 0 && errno == EINTR) continue;
        if (sent <= 0) return -1;
        src += sent;
        len -= (size_t)sent;
    }
    return 0;
}

static int nerd_probe_open(wifi_stubs_t *wifi, nerd_network_probe_t *probe)
{
    static const char http_request[] =
        "GET / HTTP/1.0\r\nHost: 192.168.4.1\r\nConnection: close\r\n\r\n";
    static const uint8_t dns_query[] = {
        0x46, 0x58, 0x01, 0x00, 0x00, 0x01, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00,
        0x07, 'e', 'x', 'a', 'm', 'p', 'l', 'e',
        0x03, 'c', 'o', 'm', 0x00,
        0x00, 0x01, 0x00, 0x01,
    };

    memset(probe, 0, sizeof(*probe));
    probe->tcp_fd = -1;
    probe->udp_fd = -1;
    if (wifi_stubs_get_bound_host_port(wifi, 80, false,
                                       &probe->tcp_port) != 0 ||
        wifi_stubs_get_bound_host_port(wifi, 53, true,
                                       &probe->udp_port) != 0)
        return -1;

    struct sockaddr_in peer;
    memset(&peer, 0, sizeof(peer));
    peer.sin_family = AF_INET;
    peer.sin_addr.s_addr = htonl(INADDR_LOOPBACK);

    probe->tcp_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (probe->tcp_fd < 0) goto fail;
    peer.sin_port = htons(probe->tcp_port);
    if (connect(probe->tcp_fd, (struct sockaddr *)&peer, sizeof(peer)) != 0 ||
        nerd_probe_send_all(probe->tcp_fd, http_request,
                            sizeof(http_request) - 1) != 0 ||
        nerd_probe_nonblocking(probe->tcp_fd) != 0)
        goto fail;

    probe->udp_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (probe->udp_fd < 0) goto fail;
    peer.sin_port = htons(probe->udp_port);
    if (sendto(probe->udp_fd, dns_query, sizeof(dns_query), 0,
               (struct sockaddr *)&peer, sizeof(peer)) !=
            (ssize_t)sizeof(dns_query) ||
        nerd_probe_nonblocking(probe->udp_fd) != 0)
        goto fail;
    return 0;

fail:
    nerd_probe_close(probe);
    return -1;
}

static void nerd_probe_receive(nerd_network_probe_t *probe)
{
    if (probe->tcp_fd >= 0 && probe->http_len < sizeof(probe->http_response)) {
        ssize_t n = recv(probe->tcp_fd,
                         probe->http_response + probe->http_len,
                         sizeof(probe->http_response) - probe->http_len,
                         MSG_DONTWAIT);
        if (n > 0) probe->http_len += (size_t)n;
    }
    if (probe->udp_fd >= 0 && probe->dns_len < sizeof(probe->dns_response)) {
        ssize_t n = recv(probe->udp_fd,
                         probe->dns_response + probe->dns_len,
                         sizeof(probe->dns_response) - probe->dns_len,
                         MSG_DONTWAIT);
        if (n > 0) probe->dns_len += (size_t)n;
    }
}

static int run_until_nerd_services(flexe_session_t *session,
                                   nerd_network_probe_t *probe,
                                   uint64_t max_cycles,
                                   wifi_stubs_stats_t *stats_out,
                                   uint64_t *cycles_out)
{
    xtensa_cpu_t *cpu0 = flexe_session_cpu(session, 0);
    wifi_stubs_t *wifi = flexe_session_wifi(session);
    uint64_t start = cpu0->cycle_count;
    wifi_stubs_stats_t stats = {0};

    while (cpu0->cycle_count - start < max_cycles) {
        if (!session_alive(session)) return -1;
        run_one_batch(session);
        nerd_probe_receive(probe);
        wifi_stubs_get_stats(wifi, &stats);

        bool http_ok = probe->http_len >= 5 &&
                       memcmp(probe->http_response, "HTTP/", 5) == 0;
        bool dns_ok = probe->dns_len >= 12 &&
                      probe->dns_response[0] == 0x46 &&
                      probe->dns_response[1] == 0x58 &&
                      (probe->dns_response[2] & 0x80) != 0;
        if (http_ok && dns_ok && stats.accept_successes >= 1 &&
            stats.recv_bytes > 0 && stats.send_bytes > 0 &&
            stats.recvfrom_bytes > 0 && stats.sendto_bytes > 0) {
            *stats_out = stats;
            *cycles_out = cpu0->cycle_count - start;
            return 0;
        }
    }

    nerd_probe_receive(probe);
    wifi_stubs_get_stats(wifi, stats_out);
    *cycles_out = cpu0->cycle_count - start;
    return 1;
}

static void usage(const char *argv0)
{
    fprintf(stderr,
            "Usage: %s [--no-jit] marauder|nerdminer <firmware.bin>\n",
            argv0);
}

int main(int argc, char **argv)
{
    int disable_jit = 0;
    int argi = 1;
    if (argi < argc && strcmp(argv[argi], "--no-jit") == 0) {
        disable_jit = 1;
        argi++;
    }
    if (argc - argi != 2) {
        usage(argv[0]);
        return 2;
    }

    const char *profile = argv[argi++];
    const char *rom_path = argv[argi];
    int is_marauder = strcmp(profile, "marauder") == 0;
    int is_nerdminer = strcmp(profile, "nerdminer") == 0;
    if (!is_marauder && !is_nerdminer) {
        usage(argv[0]);
        return 2;
    }

    uint16_t *framebuf = calloc(FB_PIXELS, sizeof(*framebuf));
    uint16_t *before = calloc(FB_PIXELS, sizeof(*before));
    if (!framebuf || !before) {
        fprintf(stderr, "error: framebuffer allocation failed\n");
        free(before);
        free(framebuf);
        return 1;
    }

    char sd_path[64];
    if (create_stock_sd_image(sd_path) != 0) {
        fprintf(stderr, "error: failed to create FAT16 SD fixture\n");
        free(before);
        free(framebuf);
        return 1;
    }

    pthread_mutex_t framebuffer_mutex;
    pthread_mutex_init(&framebuffer_mutex, NULL);
    touch_state_t touch = {0, 50, 139};
    uart_state_t uart = {0};
    raw_tx_probe_t raw_tx_probe = {0};
    rom_audit_t rom_audit = {0};
    flexe_session_config_t cfg = {
        .bin_path = rom_path,
        .initial_sp = 0x3FFF8000u,
        .disable_jit = disable_jit,
        .sdcard_path = sd_path,
        .sdcard_size = STOCK_SD_BYTES,
        .uart_cb = uart_count,
        .uart_ctx = &uart,
        .framebuf = framebuf,
        .framebuf_mutex = &framebuffer_mutex,
        .framebuf_w = FB_W,
        .framebuf_h = FB_H,
        .touch_fn = touch_read,
        .touch_ctx = &touch,
    };

    uint64_t wall_start = monotonic_ns();
    flexe_session_t *session = flexe_session_create(&cfg);
    if (!session) {
        fprintf(stderr, "error: failed to create stock-ROM session\n");
        pthread_mutex_destroy(&framebuffer_mutex);
        unlink(sd_path);
        free(before);
        free(framebuf);
        return 1;
    }
    flexe_session_set_rom_log_cb(session, audit_rom_call, &rom_audit);
    if (is_marauder)
        wifi_stubs_set_raw_tx_callback(flexe_session_wifi(session),
                                       capture_raw_tx, &raw_tx_probe);

    int min_nonblack = is_marauder ? 8000 : 20000;
    uint64_t boot_limit = is_marauder ? 8000000000ull : 2000000000ull;
    int nonblack = 0;
    uint64_t boot_cycles = 0;
    int screen_result = run_until_screen(session, framebuf, &framebuffer_mutex,
                                         min_nonblack, boot_limit, &nonblack,
                                         &boot_cycles);
    if (screen_result != 0) {
        fprintf(stderr,
                "FAIL profile=%s reason=%s nonblack=%d virtual_cycles=%llu\n",
                profile, screen_result < 0 ? "cpus-stopped" : "screen-timeout",
                nonblack, (unsigned long long)boot_cycles);
        flexe_session_destroy(session);
        pthread_mutex_destroy(&framebuffer_mutex);
        unlink(sd_path);
        free(before);
        free(framebuf);
        return 1;
    }

    int touch_changed = 0;
    int spiffs_blocks = 0;
    wifi_stubs_stats_t wifi_stats = {0};
    uint64_t network_cycles = 0;
    uint64_t service_cycles = 0;
    uint64_t cli_cycles = 0;
    uint64_t stop_cycles = 0;
    uint64_t tx_cycles = 0;
    uint64_t bt_cycles = 0;
    size_t cli_rx_bytes = 0;
    size_t tx_cli_rx_bytes = 0;
    size_t bt_cli_rx_bytes = 0;
    uint64_t marauder_raw_tx_frames = 0;
    uint64_t marauder_raw_tx_bytes = 0;
    uint64_t marauder_host_tx_frames = 0;
    uint64_t marauder_host_beacons = 0;
    uint64_t marauder_bt_advertisements = 0;
    uint32_t marauder_mgmt_frames = 0;
    uint32_t marauder_beacon_frames = 0;
    bt_stubs_stats_t bt_stats = {0};
    nerd_network_probe_t network_probe = {.tcp_fd = -1, .udp_fd = -1};
    if (is_nerdminer) {
        if (strstr(uart.log, "sdcard_mount(): f_mount failed") != NULL) {
            fprintf(stderr,
                    "FAIL profile=nerdminer reason=sd-fat-mount-failed\n");
            flexe_session_destroy(session);
            pthread_mutex_destroy(&framebuffer_mutex);
            unlink(sd_path);
            free(before);
            free(framebuf);
            return 1;
        }
        spiffs_blocks = nerdminer_spiffs_formatted(session);
        int expected_blocks = NERDMINER_SPIFFS_SIZE / NERDMINER_SPIFFS_BLOCK;
        if (spiffs_blocks != expected_blocks) {
            fprintf(stderr,
                    "FAIL profile=nerdminer reason=spiffs-not-formatted "
                    "first_bad_block=%d\n",
                    spiffs_blocks);
            flexe_session_destroy(session);
            pthread_mutex_destroy(&framebuffer_mutex);
            unlink(sd_path);
            free(before);
            free(framebuf);
            return 1;
        }

        /* With erased factory settings NerdMiner enters its captive portal.
         * Run far enough to prove that both its HTTP server (TCP/80) and DNS
         * server (UDP/53) reached successful host-backed socket setup. */
        int network_result = run_until_nerd_network(session, 1500000000ull,
                                                    &wifi_stats,
                                                    &network_cycles);
        if (network_result != 0) {
            fprintf(stderr,
                    "FAIL profile=nerdminer reason=%s network_cycles=%llu "
                    "sockets=%llu/%llu binds=%llu/%llu listens=%llu/%llu\n",
                    network_result < 0 ? "cpus-stopped" : "network-timeout",
                    (unsigned long long)network_cycles,
                    (unsigned long long)wifi_stats.socket_successes,
                    (unsigned long long)wifi_stats.socket_calls,
                    (unsigned long long)wifi_stats.bind_successes,
                    (unsigned long long)wifi_stats.bind_calls,
                    (unsigned long long)wifi_stats.listen_successes,
                    (unsigned long long)wifi_stats.listen_calls);
            flexe_session_destroy(session);
            pthread_mutex_destroy(&framebuffer_mutex);
            unlink(sd_path);
            free(before);
            free(framebuf);
            return 1;
        }

        if (nerd_probe_open(flexe_session_wifi(session), &network_probe) != 0) {
            fprintf(stderr,
                    "FAIL profile=nerdminer reason=host-network-probe-open\n");
            flexe_session_destroy(session);
            pthread_mutex_destroy(&framebuffer_mutex);
            unlink(sd_path);
            free(before);
            free(framebuf);
            return 1;
        }
        int service_result = run_until_nerd_services(
                session, &network_probe, 1500000000ull, &wifi_stats,
                &service_cycles);
        if (service_result != 0) {
            fprintf(stderr,
                    "FAIL profile=nerdminer reason=%s service_cycles=%llu "
                    "http_bytes=%zu dns_bytes=%zu accepts=%llu "
                    "tcp_rx=%llu tcp_tx=%llu udp_rx=%llu udp_tx=%llu\n",
                    service_result < 0 ? "cpus-stopped" : "service-timeout",
                    (unsigned long long)service_cycles,
                    network_probe.http_len, network_probe.dns_len,
                    (unsigned long long)wifi_stats.accept_successes,
                    (unsigned long long)wifi_stats.recv_bytes,
                    (unsigned long long)wifi_stats.send_bytes,
                    (unsigned long long)wifi_stats.recvfrom_bytes,
                    (unsigned long long)wifi_stats.sendto_bytes);
            nerd_probe_close(&network_probe);
            flexe_session_destroy(session);
            pthread_mutex_destroy(&framebuffer_mutex);
            unlink(sd_path);
            free(before);
            free(framebuf);
            return 1;
        }
    }

    if (is_marauder) {
        if (strstr(uart.log, "Failed to mount SD Card") != NULL) {
            fprintf(stderr, "FAIL profile=marauder reason=sd-fat-mount-failed\n");
            flexe_session_destroy(session);
            pthread_mutex_destroy(&framebuffer_mutex);
            unlink(sd_path);
            free(before);
            free(framebuf);
            return 1;
        }
        static const char scripts_dir[] = "SCRIPTS    ";
        if (fat16_root_has_directory(sd_path, scripts_dir) != 1) {
            fprintf(stderr,
                    "FAIL profile=marauder reason=sd-directory-write-failed\n");
            flexe_session_destroy(session);
            pthread_mutex_destroy(&framebuffer_mutex);
            unlink(sd_path);
            free(before);
            free(framebuf);
            return 1;
        }

        /* An inserted SD card adds asynchronous setup work.  The coarse pixel
         * threshold can be reached while that redraw is still in flight, so
         * let the production UI settle before taking the touch baseline. */
        if (run_for_virtual_cycles(session, 240000000ull) != 0) {
            fprintf(stderr, "FAIL profile=marauder reason=cpus-stopped-before-touch\n");
            flexe_session_destroy(session);
            pthread_mutex_destroy(&framebuffer_mutex);
            unlink(sd_path);
            free(before);
            free(framebuf);
            return 1;
        }
        framebuffer_copy(before, framebuf, &framebuffer_mutex);
        touch.pressed = 1;
        if (run_for_virtual_cycles(session, 24000000ull) != 0) {
            fprintf(stderr, "FAIL profile=marauder reason=cpus-stopped-during-touch\n");
            flexe_session_destroy(session);
            pthread_mutex_destroy(&framebuffer_mutex);
            unlink(sd_path);
            free(before);
            free(framebuf);
            return 1;
        }
        touch.pressed = 0;
        int touch_result = run_until_changed(session, before, framebuf,
                                             &framebuffer_mutex, 2500,
                                             1000000000ull, &touch_changed);
        if (touch_result != 0) {
            fprintf(stderr,
                    "FAIL profile=marauder reason=%s changed_pixels=%d\n",
                    touch_result < 0 ? "cpus-stopped" : "touch-timeout",
                    touch_changed);
            flexe_session_destroy(session);
            pthread_mutex_destroy(&framebuffer_mutex);
            unlink(sd_path);
            free(before);
            free(framebuf);
            return 1;
        }

        /* Drive the stock firmware's real Arduino/ESP-IDF UART receive path,
         * including the hardware FIFO interrupt and driver ring buffer. */
        static const uint8_t sniff_command[] = "sniffraw\n";
        wifi_stubs_stats_t wifi_before_cli = {0};
        wifi_stubs_get_stats(flexe_session_wifi(session), &wifi_before_cli);
        cli_rx_bytes = periph_uart_rx_inject(flexe_session_periph(session),
                                              sniff_command,
                                              sizeof(sniff_command) - 1);
        if (cli_rx_bytes != sizeof(sniff_command) - 1) {
            fprintf(stderr,
                    "FAIL profile=marauder reason=uart-rx-fifo accepted=%zu\n",
                    cli_rx_bytes);
            flexe_session_destroy(session);
            pthread_mutex_destroy(&framebuffer_mutex);
            unlink(sd_path);
            free(before);
            free(framebuf);
            return 1;
        }
        int cli_result = run_until_marauder_sniffer(
                session, &uart, &wifi_before_cli, 500000000ull, &wifi_stats,
                &cli_cycles);
        if (cli_result != 0) {
            fprintf(stderr,
                    "FAIL profile=marauder reason=%s cli_cycles=%llu "
                    "uart_bytes=%llu pending_rx=%zu\n",
                    cli_result < 0 ? "cpus-stopped-during-cli" : "cli-timeout",
                    (unsigned long long)cli_cycles,
                    (unsigned long long)uart.count,
                    periph_uart_rx_pending(flexe_session_periph(session)));
            flexe_session_destroy(session);
            pthread_mutex_destroy(&framebuffer_mutex);
            unlink(sd_path);
            free(before);
            free(framebuf);
            return 1;
        }

        /* Feed a real beacon frame through Marauder's registered callback
         * and require its own raw-capture counters to consume it. */
        uint32_t mgmt_before = mem_read32(flexe_session_mem(session),
                                          MARAUDER_MGMT_FRAMES_ADDR);
        uint32_t beacon_before = mem_read32(flexe_session_mem(session),
                                            MARAUDER_BEACON_FRAMES_ADDR);
        uint8_t beacon[49] = {0};
        static const uint8_t bssid[6] = {0x02, 0x46, 0x4C, 0x45, 0x58, 0x45};
        beacon[0] = 0x80; /* management beacon */
        memset(beacon + 4, 0xFF, 6);
        memcpy(beacon + 10, bssid, sizeof(bssid));
        memcpy(beacon + 16, bssid, sizeof(bssid));
        beacon[36] = 0; /* SSID information element */
        beacon[37] = 8;
        memcpy(beacon + 38, "FlexeLab", 8);
        beacon[46] = 3; /* DS parameter set: channel 1 */
        beacon[47] = 1;
        beacon[48] = 1;
        int inject_result = wifi_stubs_inject_promiscuous_frame(
                flexe_session_wifi(session), beacon, sizeof(beacon), -42, 1, 0);
        wifi_stubs_get_stats(flexe_session_wifi(session), &wifi_stats);
        marauder_mgmt_frames = mem_read32(flexe_session_mem(session),
                                          MARAUDER_MGMT_FRAMES_ADDR);
        marauder_beacon_frames = mem_read32(flexe_session_mem(session),
                                            MARAUDER_BEACON_FRAMES_ADDR);
        if (inject_result != 0 || marauder_mgmt_frames != mgmt_before + 1 ||
            marauder_beacon_frames != beacon_before + 1 ||
            wifi_stats.raw_rx_frames == 0) {
            fprintf(stderr,
                    "FAIL profile=marauder reason=promisc-frame result=%d "
                    "mgmt=%u/%u data=%u beacon=%u/%u mode=%u rx=%llu "
                    "cb_fail=%llu\n",
                    inject_result, marauder_mgmt_frames, mgmt_before,
                    mem_read32(flexe_session_mem(session),
                               MARAUDER_DATA_FRAMES_ADDR),
                    marauder_beacon_frames, beacon_before,
                    mem_read8(flexe_session_mem(session),
                              MARAUDER_SCAN_MODE_ADDR),
                    (unsigned long long)wifi_stats.raw_rx_frames,
                    (unsigned long long)wifi_stats.raw_rx_callback_failures);
            flexe_session_destroy(session);
            pthread_mutex_destroy(&framebuffer_mutex);
            unlink(sd_path);
            free(before);
            free(framebuf);
            return 1;
        }

        /* Leave capture mode through the firmware command path, then start a
         * production Rick Roll attack and require real raw 802.11 TX calls. */
        static const uint8_t stop_command[] = "stopscan\n";
        size_t stop_rx_bytes = periph_uart_rx_inject(
                flexe_session_periph(session), stop_command,
                sizeof(stop_command) - 1);
        if (stop_rx_bytes != sizeof(stop_command) - 1) {
            fprintf(stderr,
                    "FAIL profile=marauder reason=uart-stop-fifo accepted=%zu\n",
                    stop_rx_bytes);
            flexe_session_destroy(session);
            pthread_mutex_destroy(&framebuffer_mutex);
            unlink(sd_path);
            free(before);
            free(framebuf);
            return 1;
        }
        int stop_result = run_until_marauder_stopped(
                session, &uart, 500000000ull, &stop_cycles);
        if (stop_result != 0) {
            fprintf(stderr,
                    "FAIL profile=marauder reason=%s stop_cycles=%llu "
                    "mode=%u pending_rx=%zu marker=%d uart_log_len=%zu "
                    "uart_bytes=%llu raw=0x%08x ena=0x%08x conf1=0x%08x\n",
                    stop_result < 0 ? "cpus-stopped-during-stop" :
                                      "stop-timeout",
                    (unsigned long long)stop_cycles,
                    mem_read8(flexe_session_mem(session),
                              MARAUDER_SCAN_MODE_ADDR),
                    periph_uart_rx_pending(flexe_session_periph(session)),
                    uart_contains(&uart, "#stopscan"), uart.log_len,
                    (unsigned long long)uart.count,
                    mem_read32(flexe_session_mem(session), 0x3FF40004u),
                    mem_read32(flexe_session_mem(session), 0x3FF4000Cu),
                    mem_read32(flexe_session_mem(session), 0x3FF40024u));
            flexe_session_destroy(session);
            pthread_mutex_destroy(&framebuffer_mutex);
            unlink(sd_path);
            free(before);
            free(framebuf);
            return 1;
        }

        static const uint8_t attack_command[] = "attack -t rickroll\n";
        wifi_stubs_stats_t wifi_before_tx = {0};
        wifi_stubs_get_stats(flexe_session_wifi(session), &wifi_before_tx);
        uint64_t host_tx_before = raw_tx_probe.frames;
        uint64_t host_beacons_before = raw_tx_probe.beacon_frames;
        tx_cli_rx_bytes = periph_uart_rx_inject(
                flexe_session_periph(session), attack_command,
                sizeof(attack_command) - 1);
        if (tx_cli_rx_bytes != sizeof(attack_command) - 1) {
            fprintf(stderr,
                    "FAIL profile=marauder reason=uart-attack-fifo "
                    "accepted=%zu\n", tx_cli_rx_bytes);
            flexe_session_destroy(session);
            pthread_mutex_destroy(&framebuffer_mutex);
            unlink(sd_path);
            free(before);
            free(framebuf);
            return 1;
        }
        int tx_result = run_until_marauder_tx(
                session, &uart, &wifi_before_tx, 500000000ull, &wifi_stats,
                &tx_cycles);
        if (tx_result != 0 || raw_tx_probe.frames <= host_tx_before ||
            raw_tx_probe.beacon_frames <= host_beacons_before) {
            fprintf(stderr,
                    "FAIL profile=marauder reason=%s tx_cycles=%llu "
                    "mode=%u raw_tx=%llu/%llu host_tx=%llu/%llu "
                    "host_beacons=%llu/%llu pending_rx=%zu\n",
                    tx_result < 0 ? "cpus-stopped-during-attack" :
                    tx_result > 0 ? "attack-timeout" : "raw-tx-boundary",
                    (unsigned long long)tx_cycles,
                    mem_read8(flexe_session_mem(session),
                              MARAUDER_SCAN_MODE_ADDR),
                    (unsigned long long)wifi_stats.raw_tx_frames,
                    (unsigned long long)wifi_before_tx.raw_tx_frames,
                    (unsigned long long)raw_tx_probe.frames,
                    (unsigned long long)host_tx_before,
                    (unsigned long long)raw_tx_probe.beacon_frames,
                    (unsigned long long)host_beacons_before,
                    periph_uart_rx_pending(flexe_session_periph(session)));
            flexe_session_destroy(session);
            pthread_mutex_destroy(&framebuffer_mutex);
            unlink(sd_path);
            free(before);
            free(framebuf);
            return 1;
        }
        marauder_raw_tx_frames = wifi_stats.raw_tx_frames -
                                 wifi_before_tx.raw_tx_frames;
        marauder_raw_tx_bytes = wifi_stats.raw_tx_bytes -
                                wifi_before_tx.raw_tx_bytes;
        marauder_host_tx_frames = raw_tx_probe.frames - host_tx_before;
        marauder_host_beacons = raw_tx_probe.beacon_frames -
                                host_beacons_before;

        size_t stop_attack_rx = periph_uart_rx_inject(
                flexe_session_periph(session), stop_command,
                sizeof(stop_command) - 1);
        if (stop_attack_rx != sizeof(stop_command) - 1 ||
            run_until_marauder_stopped(session, &uart, 500000000ull,
                                       &stop_cycles) != 0) {
            fprintf(stderr,
                    "FAIL profile=marauder reason=stop-attack accepted=%zu "
                    "mode=%u\n", stop_attack_rx,
                    mem_read8(flexe_session_mem(session),
                              MARAUDER_SCAN_MODE_ADDR));
            flexe_session_destroy(session);
            pthread_mutex_destroy(&framebuffer_mutex);
            unlink(sd_path);
            free(before);
            free(framebuf);
            return 1;
        }

        static const uint8_t bt_command[] = "sniffbt\n";
        bt_cli_rx_bytes = periph_uart_rx_inject(
                flexe_session_periph(session), bt_command,
                sizeof(bt_command) - 1);
        int bt_result = bt_cli_rx_bytes == sizeof(bt_command) - 1 ?
                run_until_marauder_bt_scan(session, &uart, 1000000000ull,
                                           &bt_stats, &bt_cycles) : 1;
        if (bt_result != 0) {
            fprintf(stderr,
                    "FAIL profile=marauder reason=%s accepted=%zu "
                    "bt_cycles=%llu mode=%u pending_rx=%zu config=%llu "
                    "starts=%llu\n",
                    bt_result < 0 ? "cpus-stopped-during-bt" :
                                    "bt-scan-timeout",
                    bt_cli_rx_bytes, (unsigned long long)bt_cycles,
                    mem_read8(flexe_session_mem(session),
                              MARAUDER_SCAN_MODE_ADDR),
                    periph_uart_rx_pending(flexe_session_periph(session)),
                    (unsigned long long)
                            bt_stats.scan_callback_config_calls,
                    (unsigned long long)bt_stats.scan_start_calls);
            flexe_session_destroy(session);
            pthread_mutex_destroy(&framebuffer_mutex);
            unlink(sd_path);
            free(before);
            free(framebuf);
            return 1;
        }

        /* Send a standard legacy advertisement into the genuine static
         * NimBLEScan::handleGapEvent function.  The stock callback must parse
         * its complete local name and print the discovered device. */
        static const uint8_t ble_addr[6] = {
            0x45, 0x58, 0x45, 0x4C, 0x46, 0x02
        };
        static const uint8_t ble_advertisement[] = {
            0x02, 0x01, 0x06,
            0x09, 0x09, 'F', 'l', 'e', 'x', 'e', 'B', 'L', 'E',
        };
        size_t bt_uart_before = uart.log_len;
        uint64_t bt_adv_before = bt_stats.advertisement_frames;
        int bt_inject_result = bt_stubs_inject_advertisement(
                flexe_session_bt(session), ble_addr, 1, -47,
                ble_advertisement, sizeof(ble_advertisement));
        bt_stubs_get_stats(flexe_session_bt(session), &bt_stats);
        if (bt_inject_result != 0 ||
            bt_stats.advertisement_frames != bt_adv_before + 1 ||
            !uart_contains_from(&uart, bt_uart_before, "Device: FlexeBLE")) {
            fprintf(stderr,
                    "FAIL profile=marauder reason=ble-advertisement result=%d "
                    "adv=%llu/%llu failures=%llu uart_marker=%d "
                    "uart_log_len=%zu\n",
                    bt_inject_result,
                    (unsigned long long)bt_stats.advertisement_frames,
                    (unsigned long long)bt_adv_before,
                    (unsigned long long)
                            bt_stats.advertisement_callback_failures,
                    uart_contains_from(&uart, bt_uart_before,
                                       "Device: FlexeBLE"),
                    uart.log_len);
            flexe_session_destroy(session);
            pthread_mutex_destroy(&framebuffer_mutex);
            unlink(sd_path);
            free(before);
            free(framebuf);
            return 1;
        }
        marauder_bt_advertisements = bt_stats.advertisement_frames;
    }

    int unregistered_rom = rom_stubs_unregistered_count(
            flexe_session_rom(session));
    if (unregistered_rom != 0) {
        fprintf(stderr,
                "FAIL profile=%s reason=unregistered-rom calls=%d addresses=",
                profile, unregistered_rom);
        for (int i = 0; i < rom_audit.used; i++) {
            fprintf(stderr, "%s0x%08X@0x%08X(x%u)", i ? "," : "",
                    rom_audit.addr[i], rom_audit.caller[i],
                    rom_audit.count[i]);
        }
        fputc('\n', stderr);
        nerd_probe_close(&network_probe);
        flexe_session_destroy(session);
        pthread_mutex_destroy(&framebuffer_mutex);
        unlink(sd_path);
        free(before);
        free(framebuf);
        return 1;
    }

    int unhandled_mmio = periph_unhandled_count(flexe_session_periph(session));
    if (unhandled_mmio != 0) {
        fprintf(stderr,
                "FAIL profile=%s reason=unhandled-mmio accesses=%d\n",
                profile, unhandled_mmio);
        nerd_probe_close(&network_probe);
        flexe_session_destroy(session);
        pthread_mutex_destroy(&framebuffer_mutex);
        unlink(sd_path);
        free(before);
        free(framebuf);
        return 1;
    }

    uint64_t wall_ns = monotonic_ns() - wall_start;
    uint64_t total_cycles = flexe_session_cpu(session, 0)->cycle_count;
    const char *engine = flexe_session_jit(session) ? "jit" : "interp";
    printf("PASS profile=%s engine=%s wall=%.3fs virtual_cycles=%llu "
           "nonblack=%d uart_bytes=%llu",
           profile, engine, (double)wall_ns / 1e9,
           (unsigned long long)total_cycles, nonblack,
           (unsigned long long)uart.count);
    printf(" mmio_unhandled=%d rom_unregistered=%d",
           unhandled_mmio, unregistered_rom);
    if (is_marauder)
        printf(" touch_changed=%d sd_fat=mounted sd_write=SCRIPTS "
               "uart_rx=%zu cli=sniffraw cli_cycles=%llu wifi_init=%llu "
               "wifi_start=%llu promisc=%llu promisc_cb=%llu raw_rx=%llu "
               "mgmt_frames=%u beacon_frames=%u stop_cycles=%llu "
               "tx_uart_rx=%zu tx_cli=rickroll tx_cycles=%llu raw_tx=%llu "
               "raw_tx_bytes=%llu host_tx=%llu host_beacons=%llu",
               touch_changed, cli_rx_bytes, (unsigned long long)cli_cycles,
               (unsigned long long)wifi_stats.wifi_init_calls,
               (unsigned long long)wifi_stats.wifi_start_calls,
               (unsigned long long)wifi_stats.promisc_enable_calls,
               (unsigned long long)wifi_stats.promisc_callback_calls,
               (unsigned long long)wifi_stats.raw_rx_frames,
               marauder_mgmt_frames, marauder_beacon_frames,
               (unsigned long long)stop_cycles, tx_cli_rx_bytes,
               (unsigned long long)tx_cycles,
               (unsigned long long)marauder_raw_tx_frames,
               (unsigned long long)marauder_raw_tx_bytes,
               (unsigned long long)marauder_host_tx_frames,
               (unsigned long long)marauder_host_beacons);
    if (is_marauder)
        printf(" bt_uart_rx=%zu bt_cli=sniffbt bt_cycles=%llu "
               "bt_config=%llu bt_starts=%llu bt_adv=%llu "
               "bt_device=FlexeBLE",
               bt_cli_rx_bytes, (unsigned long long)bt_cycles,
               (unsigned long long)bt_stats.scan_callback_config_calls,
               (unsigned long long)bt_stats.scan_start_calls,
               (unsigned long long)marauder_bt_advertisements);
    if (is_nerdminer)
        printf(" spiffs_blocks=%d sd_fat=mounted network_cycles=%llu "
               "service_cycles=%llu tcp_udp_sockets=%llu binds=%llu "
               "listens=%llu accepts=%llu http_bytes=%zu dns_bytes=%zu "
               "tcp_rx=%llu tcp_tx=%llu udp_rx=%llu udp_tx=%llu",
               spiffs_blocks, (unsigned long long)network_cycles,
               (unsigned long long)service_cycles,
               (unsigned long long)wifi_stats.socket_successes,
               (unsigned long long)wifi_stats.bind_successes,
               (unsigned long long)wifi_stats.listen_successes,
               (unsigned long long)wifi_stats.accept_successes,
               network_probe.http_len, network_probe.dns_len,
               (unsigned long long)wifi_stats.recv_bytes,
               (unsigned long long)wifi_stats.send_bytes,
               (unsigned long long)wifi_stats.recvfrom_bytes,
               (unsigned long long)wifi_stats.sendto_bytes);
    putchar('\n');

    nerd_probe_close(&network_probe);
    flexe_session_destroy(session);
    pthread_mutex_destroy(&framebuffer_mutex);
    unlink(sd_path);
    free(before);
    free(framebuf);
    return 0;
}
