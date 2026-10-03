/* Optional process boundary to the pinned upstream eZ80/MOS test peer.
 * UART bytes traverse both firmware stacks. Decoded VSync drives eZ80 GPIO B1.
 * Commands enter the native VDP's supported UART0 console, not MOS memory.
 */
#ifndef FLEXE_AGON_MOS_BRIDGE_H
#define FLEXE_AGON_MOS_BRIDGE_H
#include <errno.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <signal.h>
#include <sys/socket.h>
#include <unistd.h>

typedef struct {
    int fd;
    bool failed;
    uint8_t pending[8192];
    size_t len;
    uint64_t uart_bytes;
} mos_link_t;

/* Three-byte header: tag, payload length (little endian). Tags are UART=0,
 * VSync pulse=1, CTS=2, shutdown=3. TCP reads/writes can split any frame. */
static bool mos_queue(mos_link_t *link, uint8_t tag, const uint8_t *data, size_t len) {
    if (len > 256u || link->len + len + 3u > sizeof(link->pending)) {
        link->failed = true;
        return false;
    }
    uint8_t *out = link->pending + link->len;
    out[0] = tag; out[1] = (uint8_t)len; out[2] = (uint8_t)(len >> 8);
    if (len) memcpy(out + 3u, data, len);
    link->len += len + 3u;
    return true;
}

static bool mos_flush(mos_link_t *link) {
    while (link->len) {
        ssize_t n = send(link->fd, link->pending, link->len, 0);
        if (n < 0 && errno == EINTR) continue;
        if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) return true;
        if (n <= 0) { link->failed = true; return false; }
        link->len -= (size_t)n;
        memmove(link->pending, link->pending + n, link->len);
    }
    return true;
}

static void mos_uart(void *opaque, uint8_t byte) {
    mos_link_t *link = opaque;
    link->uart_bytes++;
    mos_queue(link, 0, &byte, 1);
}

static bool mos_frame(const vga_t *v, uint32_t *hash_out) {
    if (!v->complete || v->columns != 400u || v->rows != 524u) return false;
    uint32_t hash = 2166136261u;
    for (unsigned y = 0; y < 240u; y++) {
        for (unsigned x = 0; x < 320u; x++) {
            uint8_t pixel = v->frame[33u + y * 2u][72u + x] & 0x3fu;
            if (pixel != 0x02u && (pixel != 0x3fu || y >= 48u || x >= 96u))
                return false;
            hash = (hash ^ pixel) * 16777619u;
        }
    }
    *hash_out = hash;
    return hash == 0xa53304aeu;
}

static bool run_mos(flexe_session_t *s, vga_t *v, unsigned port, const char *artifacts) {
    mos_link_t link = {.fd = socket(AF_INET, SOCK_STREAM, 0)};
    struct sockaddr_in address = {.sin_family = AF_INET, .sin_port = htons((uint16_t)port),
                                  .sin_addr.s_addr = htonl(INADDR_LOOPBACK)};
    if (link.fd < 0) return false;
    if (connect(link.fd, (struct sockaddr *)&address, sizeof(address))) {
        perror("MOS peer connect"); close(link.fd); return false;
    }
    signal(SIGPIPE, SIG_IGN);
    int yes = 1;
    setsockopt(link.fd, IPPROTO_TCP, TCP_NODELAY, &yes, sizeof(yes));
    static const char magic[] = "FLEXE-AGON-MOS\n";
    if (send(link.fd, magic, sizeof(magic) - 1u, 0) != (ssize_t)(sizeof(magic) - 1u)) {
        close(link.fd); return false;
    }
    if (fcntl(link.fd, F_SETFL, O_NONBLOCK) < 0) { close(link.fd); return false; }
    esp32_periph_t *p = flexe_session_periph(s);
    xtensa_cpu_t *cpu = flexe_session_cpu(s, 0), *other = flexe_session_cpu(s, 1);
    periph_set_uart_callback_num(p, 2, mos_uart, &link);
    char transcript[65536] = {0};
    size_t used = 0, buffered = 0;
    uint8_t incoming[8192];
    uint64_t frames = v->frames, start = monotonic_ns(), last_key = start, last_rx = start;
    uint64_t cycle_start = cpu->cycle_count, uart_rx = 0, syncs = 0;
    unsigned stage = 0, stable = 0;
    size_t key = 0;
    int last_cts = -1;
    static const char *commands[] = {
        "help\r", "type /flexe.txt\r", "load /bin/bbcbasic24.bin\r", "run\r",
        "10 MODE 8:VDU 23,1,0:COLOUR 129:CLS\r", "20 PRINT \"FLEXE-MOS-OK\"\r",
        "25 *FX 19\r",
        "30 PRINT 6*7\r", "40 FOR I=1 TO 3:PRINT I:NEXT\r", "50 END\r", "RUN\r"
    };
    const unsigned command_count = sizeof(commands) / sizeof(commands[0]);
    FILE *raw = NULL;
    if (artifacts) {
        char path[4096];
        snprintf(path, sizeof(path), "%s/mos-uart.bin", artifacts);
        raw = fopen(path, "wb");
        if (!raw) link.failed = true;
    }
    bool done = false;
    uint32_t hash = 0;
    while (monotonic_ns() - start < 90000000000ull && !link.failed) {
        if (!step(s)) break;
        if (v->frames != frames) {
            mos_queue(&link, 1, NULL, 0); syncs++; frames = v->frames;
            if (stage > command_count && mos_frame(v, &hash)) stable++;
            else stable = 0;
        }
        int cts = periph_uart_rx_pending_num(p, 2) < 128u;
        if (cts != last_cts) {
            uint8_t value = (uint8_t)cts;
            mos_queue(&link, 2, &value, 1); last_cts = cts;
        }
        if (!mos_flush(&link)) break;
        ssize_t n = recv(link.fd, incoming + buffered, sizeof(incoming) - buffered, 0);
        if (n > 0) buffered += (size_t)n;
        else if (!n || (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR)) break;
        while (buffered >= 3u) {
            size_t len = incoming[1] | (size_t)incoming[2] << 8;
            if (len > 256u || incoming[0] != 0 || !len) { link.failed = true; break; }
            if (buffered < len + 3u || periph_uart_rx_pending_num(p, 2) + len > 120u) break;
            if (!send_bytes(p, incoming + 3u, len)) { link.failed = true; break; }
            uart_rx += len;
            if (raw && fwrite(incoming + 3u, 1, len, raw) != len) link.failed = true;
            for (size_t i = 0; i < len; i++) {
                uint8_t ch = incoming[3u + i];
                if ((ch >= 32u && ch < 127u) || ch == 10u) {
                    if (used + 1u == sizeof(transcript)) { link.failed = true; break; }
                    transcript[used++] = (char)ch;
                }
            }
            transcript[used] = 0;
            memmove(incoming, incoming + len + 3u, buffered - len - 3u);
            buffered -= len + 3u;
            last_rx = monotonic_ns();
        }
        uint64_t now = monotonic_ns();
        if (now - last_rx > 10000000000ull) break;
        if (stage == 0u && strstr(transcript, "Agon Platform MOS Version 3.0.2") &&
            now - last_rx > 300000000ull) {
            static const uint8_t console[] = {23,0,254,1};
            if (!send_bytes(p, console, sizeof(console))) break;
            stage = 1; last_key = now;
            fprintf(stderr, "[mos] native MOS boot confirmed\n");
        } else if (stage >= 1u && stage <= command_count && now - last_key > 50000000ull) {
            const char *command = commands[stage - 1u];
            if (command[key]) {
                uint8_t ch = (uint8_t)command[key];
                if (periph_uart_rx_inject_num(p, 0, &ch, 1) != 1u) break;
                key++; last_key = now;
            } else if (now - last_rx > 500000000ull) {
                stage++; key = 0;
            }
        } else if (stage > command_count && stable >= 3u && now - last_rx > 500000000ull) {
            done = true; break;
        }
        /* Pace faster hosts at the modeled ESP32 clock, matching the upstream
         * peer's wall-clock 18.432 MHz eZ80. Slow hosts run at their own rate. */
        uint64_t cycles = cpu->cycle_count - cycle_start;
        uint64_t target = (cycles / 240000000ull) * 1000000000ull +
                          (cycles % 240000000ull) * 1000000000ull / 240000000ull;
        uint64_t elapsed = monotonic_ns() - start;
        if (target > elapsed + 1000000ull) {
            struct timespec delay = {.tv_nsec = 1000000};
            nanosleep(&delay, NULL);
        }
    }
    bool program_ok = strstr(transcript, "List of commands:") &&
                      strstr(transcript, "FLEXE-FS-OK\n") &&
                      strstr(transcript, "BBC BASIC (Agon ADL)") &&
                      strstr(transcript, "FLEXE-MOS-OK\n        42\n         1\n         2\n         3\n>");
    bool saved = true;
    if (raw && fclose(raw)) saved = false;
    if (artifacts) {
        char path[4096]; snprintf(path, sizeof(path), "%s/mos-transcript.txt", artifacts);
        FILE *out = fopen(path, "wb");
        if (!out) saved = false;
        else {
            if (fwrite(transcript, 1, used, out) != used) saved = false;
            if (fclose(out)) saved = false;
        }
        saved = save_frame(v, 320, 240, artifacts, "mos") && saved;
    }
    bool ok = done && program_ok && saved && !link.failed && !v->overflow &&
              !cpu->debug_break && !other->debug_break &&
              !cpu->breakpoint_hit && !other->breakpoint_hit &&
              !periph_unhandled_count(p) && !mem_unmapped_count(flexe_session_mem(s)) &&
              !rom_stubs_unregistered_count(flexe_session_rom(s));
    printf("%s mos=3.0.2 basic=adl engine=%s commands=%u program=%d pixels=%08x "
           "uart_rx=%llu uart_tx=%llu vsync=%llu wall=%.3f\n", ok ? "PASS" : "FAIL",
           flexe_session_jit(s) ? "jit" : "interp", stage ? stage - 1u : 0u, program_ok,
           hash, (unsigned long long)uart_rx, (unsigned long long)link.uart_bytes,
           (unsigned long long)syncs, (double)(monotonic_ns() - start) / 1e9);
    if (!ok) fprintf(stderr, "[mos] transcript: %s\n", transcript);
    mos_queue(&link, 3, NULL, 0); mos_flush(&link);
    close(link.fd);
    periph_set_uart_callback_num(p, 2, NULL, NULL);
    return ok;
}
#endif
