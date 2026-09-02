/* Station provisioning and an outbound TCP client, with the host standing in
 * as the server.
 *
 * The harness listens on an ephemeral port, publishes it into the guest, and
 * echoes whatever arrives. That makes the guest's send and receive paths
 * check each other: the fixture compares the echo byte for byte, and this
 * side compares the checksums, so a truncated, duplicated or reordered
 * transfer fails on both sides rather than looking plausible on either.
 */
#include "elf_symbols.h"
#include "flexe_session.h"
#include "memory.h"
#include "peripherals.h"
#include "rom_stubs.h"
#include "wifi_stubs.h"

#include <arpa/inet.h>
#include <errno.h>
#include <netinet/in.h>
#include <pthread.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

#define SUCCESS_MARKER 0x9F1C0C0Bu
#define MAX_CYCLES     8000000000ull
#define RESULT_COUNT   12u
#define PAYLOAD_LEN    1000

#define TEST_SSID "flexe-net"
#define TEST_PASS "flexe-secret"

volatile int emu_app_running = 1;

static uint32_t fnv1a(const uint8_t *p, size_t n) {
    uint32_t h = 2166136261u;
    for (size_t i = 0; i < n; i++) { h ^= p[i]; h *= 16777619u; }
    return h;
}

typedef struct {
    int      listen_fd;
    volatile int accepted;
    volatile size_t echoed;
} server_t;

/* Echo server. Runs on its own thread so the emulation loop stays simple;
 * the guest's socket calls are host-backed, so this is a genuine TCP
 * connection rather than a shortcut through the model. */
static void *server_thread(void *arg) {
    server_t *sv = arg;
    int fd = accept(sv->listen_fd, NULL, NULL);
    if (fd < 0) return NULL;
    sv->accepted = 1;
    uint8_t buf[512];
    for (;;) {
        ssize_t n = recv(fd, buf, sizeof(buf), 0);
        if (n <= 0) break;
        ssize_t off = 0;
        while (off < n) {
            ssize_t w = send(fd, buf + off, (size_t)(n - off), 0);
            if (w <= 0) break;
            off += w;
        }
        sv->echoed += (size_t)n;
    }
    close(fd);
    return NULL;
}

int main(int argc, char **argv) {
    int argi = 1;
    int disable_jit = 0;
    if (argi < argc && strcmp(argv[argi], "--no-jit") == 0) {
        disable_jit = 1;
        argi++;
    }
    if (argc - argi != 2) {
        fprintf(stderr, "usage: %s [--no-jit] FIRMWARE.bin FIRMWARE.elf\n",
                argv[0]);
        return 2;
    }

    elf_symbols_t *symbols = elf_symbols_load(argv[argi + 1]);
    uint32_t stage_addr = 0, result_addr = 0, port_addr = 0;
    if (!symbols ||
        elf_symbols_find(symbols, "flexe_wifi_stage", &stage_addr) != 0 ||
        elf_symbols_find(symbols, "flexe_wifi_result", &result_addr) != 0 ||
        elf_symbols_find(symbols, "flexe_wifi_port", &port_addr) != 0) {
        fprintf(stderr, "error: fixture marker symbols are missing\n");
        elf_symbols_destroy(symbols);
        return 2;
    }

    /* Listen before the guest runs, so the port is known and published the
     * moment the fixture asks for it. */
    server_t sv = {0};
    sv.listen_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (sv.listen_fd < 0) { perror("socket"); return 2; }
    int one = 1;
    setsockopt(sv.listen_fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    addr.sin_port = 0;                       /* ephemeral */
    if (bind(sv.listen_fd, (struct sockaddr *)&addr, sizeof(addr)) != 0 ||
        listen(sv.listen_fd, 1) != 0) {
        perror("bind/listen");
        close(sv.listen_fd);
        return 2;
    }
    socklen_t alen = sizeof(addr);
    getsockname(sv.listen_fd, (struct sockaddr *)&addr, &alen);
    uint16_t port = ntohs(addr.sin_port);

    pthread_t tid;
    if (pthread_create(&tid, NULL, server_thread, &sv) != 0) {
        fprintf(stderr, "error: cannot start echo server\n");
        close(sv.listen_fd);
        return 2;
    }

    flexe_session_config_t config = {
        .bin_path = argv[argi],
        .elf_path = argv[argi + 1],
        .disable_jit = disable_jit,
    };
    flexe_session_t *session = flexe_session_create(&config);
    if (!session) {
        elf_symbols_destroy(symbols);
        return 2;
    }
    wifi_stubs_set_sta_credentials(flexe_session_wifi(session), TEST_SSID,
                                   TEST_PASS);

    xtensa_cpu_t *cpu = flexe_session_cpu(session, 0);
    xtensa_mem_t *mem = flexe_session_mem(session);
    uint32_t stage = 0, last_stage = UINT32_MAX;
    int published = 0;
    while (cpu->cycle_count < MAX_CYCLES) {
        stage = mem_read32(mem, stage_addr);
        if (stage != last_stage) {
            fprintf(stderr, "[wifi-client] stage=0x%08X cycles=%llu\n",
                    stage, (unsigned long long)cpu->cycle_count);
            last_stage = stage;
        }
        /* The fixture waits for a non-zero port before connecting. */
        if (!published && stage >= 2) {
            mem_write32(mem, port_addr, port);
            published = 1;
        }
        if (stage == SUCCESS_MARKER || (stage & 0xFFF00000u) == 0xBAD00000u)
            break;
        (void)flexe_session_run_core(session, 0, 10000);
        flexe_session_post_batch(session, 10000);
    }

    stage = mem_read32(mem, stage_addr);
    uint32_t r[RESULT_COUNT];
    for (unsigned i = 0; i < RESULT_COUNT; i++)
        r[i] = mem_read32(mem, result_addr + i * 4u);
    if (stage != SUCCESS_MARKER) {
        char dump[4096];
        if (freertos_stubs_dump_tasks(flexe_session_frt(session), dump,
                                      sizeof dump) > 0)
            fprintf(stderr, "[wifi-client] tasks:\n%s", dump);
        fprintf(stderr, "[wifi-client] pc=0x%08X cycles=%llu\n", cpu->pc,
                (unsigned long long)cpu->cycle_count);
    }

    shutdown(sv.listen_fd, SHUT_RDWR);
    close(sv.listen_fd);
    pthread_join(tid, NULL);

    int unhandled = periph_unhandled_count(flexe_session_periph(session));
    int unregistered = rom_stubs_unregistered_count(flexe_session_rom(session));

    uint32_t want_ssid_hash = fnv1a((const uint8_t *)TEST_SSID,
                                    strlen(TEST_SSID));

    printf("engine=%s stage=0x%08X ssid=%u/%08X status=%u@%ums ping=%u/%08X "
           "echo=%u/%08X vs %08X ip=%08X begin=%u joined=%08X "
           "accepted=%d server_bytes=%zu unhandled=%d unregistered=%d\n",
           flexe_session_jit(session) ? "jit" : "interp", stage,
           r[0], r[1], r[2], r[3], r[4], r[5], r[6], r[7], r[8], r[9], r[10],
           r[11], sv.accepted, sv.echoed, unhandled, unregistered);

    /* The guest saw exactly the credentials the host provisioned, both as
     * saved config before connecting and as the associated AP afterwards. */
    bool ssid_ok = r[0] == strlen(TEST_SSID) && r[1] == want_ssid_hash &&
                   r[11] == want_ssid_hash;
    /* Both events arrived, and the address in GOT_IP is the one the model
     * assigns (10.0.2.15, little-endian in the guest's u32). */
    bool status_ok = r[2] >= 1u && r[9] == 0x0A00020Fu;
    bool ping_ok = r[4] == 4u;
    /* The echo came back whole, and matches what was sent. */
    bool echo_ok = r[6] == (uint32_t)PAYLOAD_LEN && r[7] == r[8];
    bool server_ok = sv.accepted == 1 &&
                     sv.echoed >= (size_t)PAYLOAD_LEN + 4u;

    /* WiFi.begin() -- the Arduino path real firmware uses -- also associated. */
    bool begin_ok = r[10] >= 1u;
    int ok = stage == SUCCESS_MARKER && ssid_ok && status_ok && ping_ok &&
             echo_ok && server_ok && begin_ok &&
             unhandled == 0 && unregistered == 0;
    if (!ok)
        fprintf(stderr, "[wifi-client] ssid=%d status=%d ping=%d echo=%d "
                        "server=%d begin=%d (want ssid %zu/%08X)\n",
                ssid_ok, status_ok, ping_ok, echo_ok, server_ok, begin_ok,
                strlen(TEST_SSID), want_ssid_hash);

    flexe_session_destroy(session);
    elf_symbols_destroy(symbols);
    return ok ? 0 : 1;
}
