/*
 * wifi_stubs.c — lwip socket bridge to host TCP/IP
 *
 * Hooks lwip_* symbols in the firmware ELF and bridges them to real host
 * sockets, giving the emulated firmware actual network connectivity.
 */

#include "wifi_stubs.h"
#include "rom_stubs.h"
#include "memory.h"

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <netinet/in.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <poll.h>

#include <openssl/ssl.h>
#include <openssl/err.h>

/* ===== Constants ===== */

#define MAX_EMU_SOCKETS 16

/* Socket fd offset — matches ESP-IDF's LWIP_SOCKET_OFFSET (46).
 * The VFS layer assigns file descriptors 0-45 for stdio, SPIFFS, NVS, etc.
 * Socket fds start at 46 to avoid collision with VFS fds, which prevents
 * the ROM close()/read()/write() hooks from accidentally operating on
 * sockets when the firmware closes VFS files (and vice versa).
 *
 * Mapping: firmware fd = array_index + SOCKET_FD_BASE
 *          array_index = firmware fd - SOCKET_FD_BASE */
#define SOCKET_FD_BASE 46

/* Firmware's reent address (set by rom_stubs esp_newlib_init).
 * newlib struct _reent has _errno at offset 0. */
#define REENT_ADDR       0x3FFE3C00u
#define REENT_ERRNO_OFS  0

/* newlib EINPROGRESS (different from Linux host value) */
#define NEWLIB_EAGAIN      11
#define NEWLIB_EINPROGRESS 119

/* lwip error codes */
#define ERR_OK   0
#define ERR_VAL -6

/* ===== Socket state ===== */

typedef struct {
    int      host_fd;            /* real host socket fd, or -1 if unused */
    bool     nonblocking;
    uint64_t total_received;     /* bytes received so far (for timeout policy) */
    bool     awaiting_response;  /* true after send(), cleared after recv() */
    SSL     *ssl;               /* OpenSSL TLS session, or NULL for plain TCP */
    SSL_CTX *ssl_ctx;           /* OpenSSL context (owned per-socket) */
} emu_socket_t;

struct wifi_stubs {
    xtensa_cpu_t      *cpu;
    esp32_rom_stubs_t *rom;
    emu_socket_t       sockets[MAX_EMU_SOCKETS];

    /* Static buffer for gethostbyname result written into emulator memory.
     * We allocate a scratch region in emulator address space for the
     * struct hostent and associated data. */
    uint32_t           hostent_buf;  /* emulator address of scratch area */
    bool               event_log;    /* use [cycle] WIFI prefix instead of [wifi] */
};

/* ===== Calling convention helpers ===== */

static uint32_t ws_arg(xtensa_cpu_t *cpu, int n)
{
    int ci = XT_PS_CALLINC(cpu->ps);
    return ar_read(cpu, ci * 4 + 2 + n);
}

static void ws_return(xtensa_cpu_t *cpu, uint32_t retval)
{
    int ci = XT_PS_CALLINC(cpu->ps);
    if (ci > 0) {
        ar_write(cpu, ci * 4 + 2, retval);
        uint32_t a0 = ar_read(cpu, ci * 4);
        cpu->pc = (cpu->pc & 0xC0000000u) | (a0 & 0x3FFFFFFFu);
        XT_PS_SET_CALLINC(cpu->ps, 0);
    } else {
        ar_write(cpu, 2, retval);
        cpu->pc = ar_read(cpu, 0);
    }
}

/* ===== WiFi log helper ===== */

static void wifi_log(wifi_stubs_t *ws, const char *fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    if (ws->event_log)
        fprintf(stderr, "[%10llu] WIFI  ", (unsigned long long)ws->cpu->cycle_count);
    else
        fprintf(stderr, "[wifi] ");
    vfprintf(stderr, fmt, ap);
    va_end(ap);
}

/* ===== Socket slot management ===== */

/* Convert firmware fd to array index. Returns -1 if out of range. */
static int fd_to_idx(int fd)
{
    int idx = fd - SOCKET_FD_BASE;
    if (idx < 0 || idx >= MAX_EMU_SOCKETS) return -1;
    return idx;
}

static int slot_alloc(wifi_stubs_t *ws, int host_fd)
{
    for (int i = 0; i < MAX_EMU_SOCKETS; i++) {
        if (ws->sockets[i].host_fd == -1) {
            ws->sockets[i].host_fd = host_fd;
            ws->sockets[i].nonblocking = false;
            ws->sockets[i].total_received = 0;
            ws->sockets[i].awaiting_response = false;
            ws->sockets[i].ssl = NULL;
            ws->sockets[i].ssl_ctx = NULL;
            return i + SOCKET_FD_BASE;  /* return firmware fd */
        }
    }
    return -1;
}

static emu_socket_t *slot_get(wifi_stubs_t *ws, int fd)
{
    int idx = fd_to_idx(fd);
    if (idx < 0) return NULL;
    if (ws->sockets[idx].host_fd == -1) return NULL;
    return &ws->sockets[idx];
}

static void slot_free(wifi_stubs_t *ws, int fd)
{
    int idx = fd_to_idx(fd);
    if (idx >= 0) {
        if (ws->sockets[idx].ssl) {
            SSL_shutdown(ws->sockets[idx].ssl);
            SSL_free(ws->sockets[idx].ssl);
        }
        if (ws->sockets[idx].ssl_ctx)
            SSL_CTX_free(ws->sockets[idx].ssl_ctx);
        ws->sockets[idx].host_fd = -1;
        ws->sockets[idx].nonblocking = false;
        ws->sockets[idx].total_received = 0;
        ws->sockets[idx].awaiting_response = false;
        ws->sockets[idx].ssl = NULL;
        ws->sockets[idx].ssl_ctx = NULL;
    }
}

/* ===== Memory helpers ===== */

/* Read a C string from emulator memory */
static void read_emu_string(xtensa_cpu_t *cpu, uint32_t addr, char *buf, int max)
{
    for (int i = 0; i < max - 1; i++) {
        uint8_t c = mem_read8(cpu->mem, addr + i);
        buf[i] = (char)c;
        if (c == 0) return;
    }
    buf[max - 1] = '\0';
}

/* Read a sockaddr_in from emulator memory (ESP-IDF layout matches POSIX) */
static void read_emu_sockaddr_in(xtensa_cpu_t *cpu, uint32_t addr,
                                 struct sockaddr_in *sa)
{
    memset(sa, 0, sizeof(*sa));
    /* ESP-IDF lwip sockaddr_in: sin_len(1), sin_family(1), sin_port(2), sin_addr(4)
     * sin_port is in NBO in memory. On LE Xtensa, mem_read16 returns the LE
     * (host-native) value, so no swap needed — both emulator host and the
     * in-memory representation agree on byte order for a LE→LE read.
     * sin_addr.s_addr is also NBO; mem_read32 on LE gives the same bytes. */
    sa->sin_family      = mem_read8(cpu->mem, addr + 1);
    sa->sin_port        = mem_read16(cpu->mem, addr + 2);
    sa->sin_addr.s_addr = mem_read32(cpu->mem, addr + 4);
}

/* Write a sockaddr_in to emulator memory */
static void write_emu_sockaddr_in(xtensa_cpu_t *cpu, uint32_t addr,
                                  const struct sockaddr_in *sa)
{
    mem_write8(cpu->mem, addr + 0, 16); /* sin_len */
    mem_write8(cpu->mem, addr + 1, (uint8_t)sa->sin_family);
    mem_write16(cpu->mem, addr + 2, sa->sin_port);
    mem_write32(cpu->mem, addr + 4, sa->sin_addr.s_addr);
    /* zero-pad rest */
    mem_write32(cpu->mem, addr + 8, 0);
    mem_write32(cpu->mem, addr + 12, 0);
}

/* ===== Tier 1: Connection ===== */

static void stub_lwip_socket(xtensa_cpu_t *cpu, void *ctx)
{
    wifi_stubs_t *ws = ctx;
    uint32_t domain   = ws_arg(cpu, 0);
    uint32_t type     = ws_arg(cpu, 1);
    uint32_t protocol = ws_arg(cpu, 2);

    (void)domain; (void)protocol;

    int hfd = socket(AF_INET, (int)type, 0);
    if (hfd < 0) {
        wifi_log(ws, "socket() failed: %s\n", strerror(errno));
        ws_return(cpu, (uint32_t)-1);
        return;
    }

    int idx = slot_alloc(ws, hfd);
    if (idx < 0) {
        close(hfd);
        wifi_log(ws, "no free socket slots\n");
        ws_return(cpu, (uint32_t)-1);
        return;
    }

    wifi_log(ws, "socket() → slot %d (host fd %d)\n", idx, hfd);
    ws_return(cpu, (uint32_t)idx);
}

static void set_firmware_errno(xtensa_cpu_t *cpu, int eno)
{
    mem_write32(cpu->mem, REENT_ADDR + REENT_ERRNO_OFS, (uint32_t)eno);
}

static void stub_lwip_connect(xtensa_cpu_t *cpu, void *ctx)
{
    wifi_stubs_t *ws = ctx;
    uint32_t fd      = ws_arg(cpu, 0);
    uint32_t sa_addr = ws_arg(cpu, 1);
    /* uint32_t addrlen = ws_arg(cpu, 2); */

    emu_socket_t *s = slot_get(ws, (int)fd);
    if (!s) {
        ws_return(cpu, (uint32_t)-1);
        return;
    }

    struct sockaddr_in sa;
    read_emu_sockaddr_in(cpu, sa_addr, &sa);

    char ip_str[INET_ADDRSTRLEN];
    inet_ntop(AF_INET, &sa.sin_addr, ip_str, sizeof(ip_str));
    wifi_log(ws, "connect(slot %u) → %s:%d\n",
            fd, ip_str, ntohs(sa.sin_port));

    /* Ensure host socket matches firmware non-blocking state */
    int flags = fcntl(s->host_fd, F_GETFL, 0);
    if (s->nonblocking)
        fcntl(s->host_fd, F_SETFL, flags | O_NONBLOCK);
    else
        fcntl(s->host_fd, F_SETFL, flags & ~O_NONBLOCK);

    int ret = connect(s->host_fd, (struct sockaddr *)&sa, sizeof(sa));

    if (ret < 0 && errno == EINPROGRESS) {
        if (s->nonblocking) {
            /* Firmware expects non-blocking connect: return -1/EINPROGRESS.
             * Firmware will call select() then getsockopt(SO_ERROR). */
            wifi_log(ws, "connect: non-blocking, EINPROGRESS\n");
            set_firmware_errno(cpu, NEWLIB_EINPROGRESS);
            ws_return(cpu, (uint32_t)-1);
            return;
        }
        /* Blocking mode: wait with 5-second timeout */
        fd_set wset;
        FD_ZERO(&wset);
        FD_SET(s->host_fd, &wset);
        struct timeval tv = { .tv_sec = 5, .tv_usec = 0 };
        int sel = select(s->host_fd + 1, NULL, &wset, NULL, &tv);
        if (sel > 0) {
            int err = 0;
            socklen_t elen = sizeof(err);
            getsockopt(s->host_fd, SOL_SOCKET, SO_ERROR, &err, &elen);
            if (err == 0) {
                ret = 0; /* connected */
            } else {
                wifi_log(ws, "connect failed: %s\n", strerror(err));
                ret = -1;
            }
        } else {
            wifi_log(ws, "connect timed out (5s)\n");
            ret = -1;
        }
    } else if (ret < 0) {
        wifi_log(ws, "connect failed: %s\n", strerror(errno));
    }

    set_firmware_errno(cpu, ret < 0 ? errno : 0);
    ws_return(cpu, (uint32_t)(ret < 0 ? -1 : 0));
}

static void stub_lwip_close(xtensa_cpu_t *cpu, void *ctx)
{
    wifi_stubs_t *ws = ctx;
    uint32_t fd = ws_arg(cpu, 0);

    emu_socket_t *s = slot_get(ws, (int)fd);
    if (s) {
        wifi_log(ws, "close(slot %u, host fd %d)\n", fd, s->host_fd);
        close(s->host_fd);
        slot_free(ws, (int)fd);
    } else if (fd >= SOCKET_FD_BASE && fd < SOCKET_FD_BASE + MAX_EMU_SOCKETS) {
        wifi_log(ws, "close(fd %u): no slot (passthrough)\n", fd);
    }
    ws_return(cpu, 0);
}

/* ===== Tier 2: Data transfer ===== */

static void stub_lwip_write(xtensa_cpu_t *cpu, void *ctx)
{
    wifi_stubs_t *ws = ctx;
    uint32_t fd   = ws_arg(cpu, 0);
    uint32_t buf  = ws_arg(cpu, 1);
    uint32_t len  = ws_arg(cpu, 2);

    emu_socket_t *s = slot_get(ws, (int)fd);
    if (!s) {
        wifi_log(ws, "write(fd=%u, len=%u): no such slot\n", fd, len);
        ws_return(cpu, (uint32_t)-1);
        return;
    }

    /* Copy data from emulator memory */
    uint8_t *tmp = malloc(len);
    if (!tmp) { ws_return(cpu, (uint32_t)-1); return; }
    for (uint32_t i = 0; i < len; i++)
        tmp[i] = mem_read8(cpu->mem, buf + i);

    ssize_t n = send(s->host_fd, tmp, len, MSG_NOSIGNAL);
    int saved_errno = errno;

    if (n > 0) {
        wifi_log(ws, "send(slot %u, %zd bytes)\n", fd, n);
        s->awaiting_response = true;
    }

    free(tmp);

    if (n < 0) {
        set_firmware_errno(cpu, saved_errno);
        ws_return(cpu, (uint32_t)-1);
        return;
    }
    ws_return(cpu, (uint32_t)n);
}

static void stub_lwip_send(xtensa_cpu_t *cpu, void *ctx)
{
    /* lwip_send(fd, buf, len, flags) — same as write, ignore flags */
    stub_lwip_write(cpu, ctx);
}

/* lwip_read / lwip_recv — used for TCP data (including TLS via VFS).
 * For blocking sockets (nonblocking=false), polls with a timeout so that
 * TLS handshakes can complete. Non-blocking sockets use MSG_DONTWAIT. */
static void stub_lwip_read(xtensa_cpu_t *cpu, void *ctx)
{
    wifi_stubs_t *ws = ctx;
    uint32_t fd   = ws_arg(cpu, 0);
    uint32_t buf  = ws_arg(cpu, 1);
    uint32_t len  = ws_arg(cpu, 2);

    emu_socket_t *s = slot_get(ws, (int)fd);
    if (!s) {
        ws_return(cpu, (uint32_t)-1);
        return;
    }

    /* len=0 is a connection check (WiFiClient::connected()), not a data
     * read.  Return 0 without treating it as EOF. */
    if (len == 0) {
        ws_return(cpu, 0);
        return;
    }

    uint8_t *tmp = malloc(len);
    if (!tmp) { ws_return(cpu, (uint32_t)-1); return; }

    ssize_t n;
    if (s->nonblocking) {
        n = recv(s->host_fd, tmp, len, MSG_DONTWAIT);
    } else {
        /* Blocking socket: adaptive timeout based on socket state.
         *
         * Three modes:
         * 1. Handshake (no data received yet): 5 seconds (50 × 100ms).
         *    mbedTLS doesn't retry on EAGAIN so we must wait for the
         *    server's TLS response.
         * 2. Awaiting response (just sent data): 500ms single poll.
         *    Expecting a server reply (authorize, Stratum, etc.).
         * 3. Idle polling (data received, nothing recently sent): 1ms.
         *    Quick check for async data (mining.notify), then let
         *    FreeRTOS run other tasks (mining, display updates). */
        int max_attempts, poll_ms;
        if (s->total_received == 0) {
            max_attempts = 50; poll_ms = 100;  /* handshake: up to 5s */
        } else if (s->awaiting_response) {
            max_attempts = 1;  poll_ms = 500;  /* expecting reply */
        } else {
            max_attempts = 1;  poll_ms = 1;    /* idle check */
        }
        struct pollfd pfd = { .fd = s->host_fd, .events = POLLIN };
        n = -1;
        for (int attempt = 0; attempt < max_attempts; attempt++) {
            int pr = poll(&pfd, 1, poll_ms);
            if (pr > 0) {
                n = recv(s->host_fd, tmp, len, 0);
                if (n == 0) {
                    wifi_log(ws, "recv(slot %u): EOF (host_fd=%d)\n", fd, s->host_fd);
                }
                break;
            }
            if (pr < 0) {
                wifi_log(ws, "recv(slot %u): poll error errno=%d\n", fd, errno);
                break;
            }
            if (pfd.revents & (POLLHUP | POLLERR)) {
                n = 0; /* EOF */
                wifi_log(ws, "recv(slot %u): POLLHUP/POLLERR revents=0x%x\n", fd, pfd.revents);
                break;
            }
        }
        if (n > 0) {
            s->total_received += (uint64_t)n;
            s->awaiting_response = false;
            wifi_log(ws, "recv(slot %u, %zd/%u bytes)\n", fd, n, len);
        } else if (n == 0 && s->total_received > 0) {
            /* Only log EOF for sockets that previously received data;
             * first-recv EOF already logged above with detail */
            s->awaiting_response = false;
            wifi_log(ws, "recv(slot %u): EOF\n", fd);
        } else if (n == -1) {
            /* timeout: no data available within poll window */
        }
    }

    if (n > 0) {
        for (ssize_t i = 0; i < n; i++)
            mem_write8(cpu->mem, buf + (uint32_t)i, tmp[i]);
    }
    free(tmp);

    if (n < 0) {
        set_firmware_errno(cpu, NEWLIB_EAGAIN);
        ws_return(cpu, (uint32_t)-1);
        return;
    }
    ws_return(cpu, (uint32_t)n);
}

static void stub_lwip_recv(xtensa_cpu_t *cpu, void *ctx)
{
    /* lwip_recv(fd, buf, len, flags) — same as read, ignore flags */
    stub_lwip_read(cpu, ctx);
}

/* ===== Tier 3: DNS ===== */

/*
 * lwip_gethostbyname(name) — resolve hostname, return pointer to static
 * struct hostent in emulator memory.
 *
 * Layout at hostent_buf (64 bytes):
 *   +0:  h_name      (4 bytes, ptr)
 *   +4:  h_aliases   (4 bytes, ptr to NULL)
 *   +8:  h_addrtype  (4 bytes, AF_INET=2)
 *   +12: h_length    (4 bytes, 4)
 *   +16: h_addr_list (4 bytes, ptr to addr_list)
 *   +20: addr_list[0] (4 bytes, ptr to addr)
 *   +24: addr_list[1] (4 bytes, NULL)
 *   +28: addr (4 bytes, IP in network byte order)
 *   +32: null_ptr (4 bytes, 0)
 *   +36: name string (up to 28 bytes)
 */
static void stub_lwip_gethostbyname(xtensa_cpu_t *cpu, void *ctx)
{
    wifi_stubs_t *ws = ctx;
    uint32_t name_addr = ws_arg(cpu, 0);

    char hostname[256];
    read_emu_string(cpu, name_addr, hostname, sizeof(hostname));

    wifi_log(ws, "gethostbyname(\"%s\")\n", hostname);

    struct addrinfo hints = { .ai_family = AF_INET, .ai_socktype = SOCK_STREAM };
    struct addrinfo *res = NULL;
    int err = getaddrinfo(hostname, NULL, &hints, &res);
    if (err != 0 || !res) {
        wifi_log(ws, "DNS failed for %s: %s\n", hostname,
                gai_strerror(err));
        ws_return(cpu, 0); /* NULL */
        return;
    }

    struct sockaddr_in *resolved = (struct sockaddr_in *)res->ai_addr;
    uint32_t ip = resolved->sin_addr.s_addr;

    char ip_str[INET_ADDRSTRLEN];
    inet_ntop(AF_INET, &resolved->sin_addr, ip_str, sizeof(ip_str));
    wifi_log(ws, "DNS: %s → %s\n", hostname, ip_str);
    freeaddrinfo(res);

    /* Write struct hostent into scratch area */
    uint32_t base = ws->hostent_buf;
    /* h_name → points to name string at base+36 */
    mem_write32(cpu->mem, base + 0, base + 36);
    /* h_aliases → points to null_ptr at base+32 */
    mem_write32(cpu->mem, base + 4, base + 32);
    /* h_addrtype = AF_INET (2) */
    mem_write32(cpu->mem, base + 8, 2);
    /* h_length = 4 */
    mem_write32(cpu->mem, base + 12, 4);
    /* h_addr_list → points to addr_list at base+20 */
    mem_write32(cpu->mem, base + 16, base + 20);
    /* addr_list[0] → points to addr at base+28 */
    mem_write32(cpu->mem, base + 20, base + 28);
    /* addr_list[1] = NULL */
    mem_write32(cpu->mem, base + 24, 0);
    /* addr = resolved IP */
    mem_write32(cpu->mem, base + 28, ip);
    /* null_ptr */
    mem_write32(cpu->mem, base + 32, 0);
    /* name string */
    int namelen = (int)strlen(hostname);
    if (namelen > 27) namelen = 27;
    for (int i = 0; i < namelen; i++)
        mem_write8(cpu->mem, base + 36 + (uint32_t)i, (uint8_t)hostname[i]);
    mem_write8(cpu->mem, base + 36 + (uint32_t)namelen, 0);

    ws_return(cpu, base);
}

/*
 * dns_gethostbyname(hostname, addr, found_cb, cb_arg)
 * Synchronous resolve. Writes IP to *addr and returns ERR_OK.
 * addr points to an ip_addr_t (lwip) which is just a uint32_t for IPv4.
 */
static void stub_dns_gethostbyname(xtensa_cpu_t *cpu, void *ctx)
{
    wifi_stubs_t *ws = ctx;
    uint32_t name_addr = ws_arg(cpu, 0);
    uint32_t addr_ptr  = ws_arg(cpu, 1);
    /* uint32_t found_cb = ws_arg(cpu, 2); */
    /* uint32_t cb_arg   = ws_arg(cpu, 3); */

    char hostname[256];
    read_emu_string(cpu, name_addr, hostname, sizeof(hostname));

    wifi_log(ws, "dns_gethostbyname(\"%s\")\n", hostname);

    struct addrinfo hints = { .ai_family = AF_INET, .ai_socktype = SOCK_STREAM };
    struct addrinfo *res = NULL;
    int err = getaddrinfo(hostname, NULL, &hints, &res);
    if (err != 0 || !res) {
        wifi_log(ws, "DNS failed for %s: %s\n", hostname,
                gai_strerror(err));
        ws_return(cpu, (uint32_t)ERR_VAL);
        return;
    }

    struct sockaddr_in *resolved = (struct sockaddr_in *)res->ai_addr;
    uint32_t ip = resolved->sin_addr.s_addr;

    char ip_str[INET_ADDRSTRLEN];
    inet_ntop(AF_INET, &resolved->sin_addr, ip_str, sizeof(ip_str));
    wifi_log(ws, "DNS: %s → %s\n", hostname, ip_str);
    freeaddrinfo(res);

    /* Write IP to ip_addr_t in emulator memory */
    if (addr_ptr)
        mem_write32(cpu->mem, addr_ptr, ip);

    ws_return(cpu, ERR_OK);
}

/* ===== Tier 4: Multiplexing & config ===== */

static void stub_lwip_select(xtensa_cpu_t *cpu, void *ctx)
{
    wifi_stubs_t *ws = ctx;
    uint32_t nfds      = ws_arg(cpu, 0);
    uint32_t read_ptr  = ws_arg(cpu, 1);
    uint32_t write_ptr = ws_arg(cpu, 2);
    uint32_t exc_ptr   = ws_arg(cpu, 3);
    /* timeout is arg 4, on stack for windowed calls */

    /* Build host fd_sets from emulator fd_sets.
     * ESP-IDF lwip fd_set: array of uint32_t bitmasks, FD_SETSIZE=64.
     * With SOCKET_FD_BASE=46, socket bits are in the second uint32_t
     * (bits 32-63 of the fd_set). */
    fd_set hread, hwrite, hexc;
    FD_ZERO(&hread); FD_ZERO(&hwrite); FD_ZERO(&hexc);
    int maxfd = -1;

    /* Read the fd_set bitmask words covering our socket range.
     * Bits are at bit position = fd, so fd 46 is at word[1] bit 14.
     * We read 2 uint32_t words (bits 0-63) from each fd_set. */
    uint32_t emu_rbits[2] = {0, 0};
    uint32_t emu_wbits[2] = {0, 0};
    uint32_t emu_ebits[2] = {0, 0};
    if (read_ptr)  { emu_rbits[0] = mem_read32(cpu->mem, read_ptr);
                     emu_rbits[1] = mem_read32(cpu->mem, read_ptr + 4); }
    if (write_ptr) { emu_wbits[0] = mem_read32(cpu->mem, write_ptr);
                     emu_wbits[1] = mem_read32(cpu->mem, write_ptr + 4); }
    if (exc_ptr)   { emu_ebits[0] = mem_read32(cpu->mem, exc_ptr);
                     emu_ebits[1] = mem_read32(cpu->mem, exc_ptr + 4); }

    /* Check each possible socket fd */
    for (int i = 0; i < MAX_EMU_SOCKETS; i++) {
        int fd = i + SOCKET_FD_BASE;
        if ((uint32_t)fd >= nfds) break;
        emu_socket_t *s = slot_get(ws, fd);
        if (!s) continue;
        int word = fd / 32;
        uint32_t bit = 1u << (fd % 32);
        if (emu_rbits[word] & bit) { FD_SET(s->host_fd, &hread); }
        if (emu_wbits[word] & bit) { FD_SET(s->host_fd, &hwrite); }
        if (emu_ebits[word] & bit) { FD_SET(s->host_fd, &hexc); }
        if (s->host_fd > maxfd) maxfd = s->host_fd;
    }

    /* Use a short timeout to avoid blocking the emulator */
    struct timeval tv = { .tv_sec = 0, .tv_usec = 1000 }; /* 1ms */
    int ret = select(maxfd + 1, &hread, &hwrite, &hexc, &tv);

    /* Translate results back — clear both words, then set ready bits */
    if (read_ptr)  { mem_write32(cpu->mem, read_ptr,     0);
                     mem_write32(cpu->mem, read_ptr + 4, 0); }
    if (write_ptr) { mem_write32(cpu->mem, write_ptr,     0);
                     mem_write32(cpu->mem, write_ptr + 4, 0); }
    if (exc_ptr)   { mem_write32(cpu->mem, exc_ptr,       0);
                     mem_write32(cpu->mem, exc_ptr + 4,   0); }

    if (ret > 0) {
        uint32_t out_r[2] = {0, 0}, out_w[2] = {0, 0}, out_e[2] = {0, 0};
        for (int i = 0; i < MAX_EMU_SOCKETS; i++) {
            int fd = i + SOCKET_FD_BASE;
            emu_socket_t *s = slot_get(ws, fd);
            if (!s) continue;
            int word = fd / 32;
            uint32_t bit = 1u << (fd % 32);
            if (FD_ISSET(s->host_fd, &hread))  out_r[word] |= bit;
            if (FD_ISSET(s->host_fd, &hwrite)) out_w[word] |= bit;
            if (FD_ISSET(s->host_fd, &hexc))   out_e[word] |= bit;
        }
        if (read_ptr)  { mem_write32(cpu->mem, read_ptr,     out_r[0]);
                         mem_write32(cpu->mem, read_ptr + 4, out_r[1]); }
        if (write_ptr) { mem_write32(cpu->mem, write_ptr,     out_w[0]);
                         mem_write32(cpu->mem, write_ptr + 4, out_w[1]); }
        if (exc_ptr)   { mem_write32(cpu->mem, exc_ptr,       out_e[0]);
                         mem_write32(cpu->mem, exc_ptr + 4,   out_e[1]); }
    }

    ws_return(cpu, (uint32_t)ret);
}

static void stub_lwip_ioctl(xtensa_cpu_t *cpu, void *ctx)
{
    wifi_stubs_t *ws = ctx;
    uint32_t fd   = ws_arg(cpu, 0);
    uint32_t cmd  = ws_arg(cpu, 1);
    uint32_t argp = ws_arg(cpu, 2);

    emu_socket_t *s = slot_get(ws, (int)fd);
    if (!s) {
        ws_return(cpu, (uint32_t)-1);
        return;
    }

    /* FIONREAD = 0x541B on Linux, 0x4004667F on lwip — handle both */
    int avail = 0;
    (void)cmd;
    if (ioctl(s->host_fd, FIONREAD, &avail) < 0)
        avail = 0;

    if (argp)
        mem_write32(cpu->mem, argp, (uint32_t)avail);

    ws_return(cpu, 0);
}

static void stub_lwip_setsockopt(xtensa_cpu_t *cpu, void *ctx)
{
    wifi_stubs_t *ws = ctx;
    uint32_t fd      = ws_arg(cpu, 0);
    /* uint32_t level   = ws_arg(cpu, 1); */
    /* uint32_t optname = ws_arg(cpu, 2); */
    /* uint32_t optval  = ws_arg(cpu, 3); */

    emu_socket_t *s = slot_get(ws, (int)fd);
    if (!s) {
        ws_return(cpu, (uint32_t)-1);
        return;
    }

    /* Accept but ignore most socket options — they're usually
     * SO_RCVTIMEO, TCP_NODELAY, etc. which aren't critical. */
    ws_return(cpu, 0);
}

static void stub_lwip_getsockopt(xtensa_cpu_t *cpu, void *ctx)
{
    wifi_stubs_t *ws = ctx;
    uint32_t fd      = ws_arg(cpu, 0);
    /* uint32_t level   = ws_arg(cpu, 1); */
    uint32_t optname = ws_arg(cpu, 2);
    uint32_t optval  = ws_arg(cpu, 3);

    emu_socket_t *s = slot_get(ws, (int)fd);
    if (!s) {
        ws_return(cpu, (uint32_t)-1);
        return;
    }

    /* SO_ERROR (0x1007 on lwip, 4 on Linux) — relay real host socket error.
     * WiFiClient::connect() checks this after select() to verify connection. */
    if (optval && (optname == 0x1007 || optname == 4)) {
        int err = 0;
        socklen_t elen = sizeof(err);
        getsockopt(s->host_fd, SOL_SOCKET, SO_ERROR, &err, &elen);
        mem_write32(cpu->mem, optval, (uint32_t)err);
    }

    ws_return(cpu, 0);
}

static void stub_lwip_fcntl(xtensa_cpu_t *cpu, void *ctx)
{
    wifi_stubs_t *ws = ctx;
    uint32_t fd  = ws_arg(cpu, 0);
    uint32_t cmd = ws_arg(cpu, 1);
    uint32_t val = ws_arg(cpu, 2);

    emu_socket_t *s = slot_get(ws, (int)fd);
    if (!s) {
        ws_return(cpu, (uint32_t)-1);
        return;
    }

    /* F_GETFL=3, F_SETFL=4, O_NONBLOCK=0x800 (lwip) / 0x800 (Linux) */
    if (cmd == 3) { /* F_GETFL */
        ws_return(cpu, s->nonblocking ? 0x800u : 0);
        return;
    }
    if (cmd == 4) { /* F_SETFL */
        s->nonblocking = (val & 0x800) != 0;
        int flags = fcntl(s->host_fd, F_GETFL, 0);
        if (s->nonblocking)
            fcntl(s->host_fd, F_SETFL, flags | O_NONBLOCK);
        else
            fcntl(s->host_fd, F_SETFL, flags & ~O_NONBLOCK);
        ws_return(cpu, 0);
        return;
    }

    ws_return(cpu, 0);
}

static void stub_lwip_getsockname(xtensa_cpu_t *cpu, void *ctx)
{
    wifi_stubs_t *ws = ctx;
    uint32_t fd       = ws_arg(cpu, 0);
    uint32_t sa_addr  = ws_arg(cpu, 1);
    uint32_t len_addr = ws_arg(cpu, 2);

    emu_socket_t *s = slot_get(ws, (int)fd);
    if (!s) {
        ws_return(cpu, (uint32_t)-1);
        return;
    }

    struct sockaddr_in sa;
    socklen_t slen = sizeof(sa);
    if (getsockname(s->host_fd, (struct sockaddr *)&sa, &slen) < 0) {
        ws_return(cpu, (uint32_t)-1);
        return;
    }

    if (sa_addr)
        write_emu_sockaddr_in(cpu, sa_addr, &sa);
    if (len_addr)
        mem_write32(cpu->mem, len_addr, sizeof(struct sockaddr_in));

    ws_return(cpu, 0);
}

/* ===== Tier 5: Stubs (unlikely used) ===== */

static void stub_lwip_bind(xtensa_cpu_t *cpu, void *ctx)
{
    wifi_stubs_t *ws = ctx;
    uint32_t fd      = ws_arg(cpu, 0);
    uint32_t sa_addr = ws_arg(cpu, 1);
    /* uint32_t addrlen = ws_arg(cpu, 2); */

    emu_socket_t *s = slot_get(ws, (int)fd);
    if (!s) {
        ws_return(cpu, (uint32_t)-1);
        return;
    }

    struct sockaddr_in sa;
    read_emu_sockaddr_in(cpu, sa_addr, &sa);

    /* Allow port reuse for bind */
    int reuse = 1;
    setsockopt(s->host_fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    int ret = bind(s->host_fd, (struct sockaddr *)&sa, sizeof(sa));
    if (ret < 0)
        wifi_log(ws, "bind(slot %u) failed: %s\n", fd, strerror(errno));
    else
        wifi_log(ws, "bind(slot %u) port %d\n", fd, ntohs(sa.sin_port));

    ws_return(cpu, (uint32_t)ret);
}

static void stub_lwip_listen(xtensa_cpu_t *cpu, void *ctx)
{
    (void)ctx;
    ws_return(cpu, (uint32_t)-1);
}

static void stub_lwip_accept(xtensa_cpu_t *cpu, void *ctx)
{
    (void)ctx;
    ws_return(cpu, (uint32_t)-1);
}

static void stub_lwip_sendto(xtensa_cpu_t *cpu, void *ctx)
{
    wifi_stubs_t *ws = ctx;
    uint32_t fd       = ws_arg(cpu, 0);
    uint32_t buf      = ws_arg(cpu, 1);
    uint32_t len      = ws_arg(cpu, 2);
    /* uint32_t flags = ws_arg(cpu, 3); */
    uint32_t sa_addr  = ws_arg(cpu, 4);
    uint32_t addrlen  = ws_arg(cpu, 5);

    emu_socket_t *s = slot_get(ws, (int)fd);
    if (!s) {
        ws_return(cpu, (uint32_t)-1);
        return;
    }

    uint8_t *tmp = malloc(len);
    if (!tmp) { ws_return(cpu, (uint32_t)-1); return; }
    for (uint32_t i = 0; i < len; i++)
        tmp[i] = mem_read8(cpu->mem, buf + i);

    ssize_t n;
    if (sa_addr && addrlen >= sizeof(struct sockaddr_in)) {
        /* Unconnected UDP: read destination address from emulator memory */
        struct sockaddr_in sa;
        read_emu_sockaddr_in(cpu, sa_addr, &sa);
        n = sendto(s->host_fd, tmp, len, MSG_NOSIGNAL,
                   (struct sockaddr *)&sa, sizeof(sa));
    } else {
        /* Connected socket or no dest: use send() */
        n = sendto(s->host_fd, tmp, len, MSG_NOSIGNAL, NULL, 0);
    }
    int saved_errno = errno;
    free(tmp);

    if (n > 0)
        wifi_log(ws, "sendto(slot %u, %zd bytes)\n", fd, n);

    if (n < 0) {
        set_firmware_errno(cpu, saved_errno);
        ws_return(cpu, (uint32_t)-1);
        return;
    }
    ws_return(cpu, (uint32_t)n);
}

/* lwip_recvfrom — always non-blocking (used for UDP polling like NTP).
 * Unlike lwip_read, never blocks even on blocking sockets. */
static void stub_lwip_recvfrom(xtensa_cpu_t *cpu, void *ctx)
{
    wifi_stubs_t *ws = ctx;
    uint32_t fd   = ws_arg(cpu, 0);
    uint32_t buf  = ws_arg(cpu, 1);
    uint32_t len  = ws_arg(cpu, 2);

    emu_socket_t *s = slot_get(ws, (int)fd);
    if (!s) {
        ws_return(cpu, (uint32_t)-1);
        return;
    }

    uint8_t *tmp = malloc(len);
    if (!tmp) { ws_return(cpu, (uint32_t)-1); return; }

    ssize_t n = recv(s->host_fd, tmp, len, MSG_DONTWAIT);

    if (n > 0) {
        for (ssize_t i = 0; i < n; i++)
            mem_write8(cpu->mem, buf + (uint32_t)i, tmp[i]);
    }
    free(tmp);

    if (n < 0) {
        set_firmware_errno(cpu, NEWLIB_EAGAIN);
        ws_return(cpu, (uint32_t)-1);
        return;
    }
    ws_return(cpu, (uint32_t)n);
}

/* ===== Host-side TLS (replaces firmware mbedtls) ===== */

/* sslclient_context layout (from ESP32 Arduino ssl_client.h):
 *   offset 0: int socket;    // lwip socket fd (our slot index)
 *   offset 4: mbedtls_ssl_context ssl_ctx;
 *   ...
 * We only need offset 0 to get the socket slot. */
#define SSLCTX_SOCKET_OFS  0

/* Read the emulator fd from a sslclient_context pointer in emu memory.
 * slot_get() handles fd→slot conversion internally. */
static emu_socket_t *ssl_get_socket(wifi_stubs_t *ws, xtensa_cpu_t *cpu,
                                    uint32_t ctx_addr, int *out_fd)
{
    int fd = (int)mem_read32(cpu->mem, ctx_addr + SSLCTX_SOCKET_OFS);
    if (out_fd) *out_fd = fd;
    return slot_get(ws, fd);
}

/*
 * start_ssl_client(sslclient_context*, IPAddress&, port, rootCA, ...)
 *
 * Replaces the firmware's mbedtls TLS handshake with host-side OpenSSL.
 * This function creates its own socket, connects, and attempts TLS.
 * If TLS fails (plain-text server like Stratum): reconnects as plain TCP.
 * Returns socket slot on success, negative on failure.
 */
static void stub_start_ssl_client(xtensa_cpu_t *cpu, void *ctx)
{
    wifi_stubs_t *ws = ctx;
    uint32_t ssl_ctx_addr = ws_arg(cpu, 0);
    uint32_t ip_ref       = ws_arg(cpu, 1);  /* IPAddress const& */
    uint32_t port         = ws_arg(cpu, 2);

    /* Read IP address from IPAddress object.
     * IPAddress inherits from Printable (vtable), so layout is:
     *   offset 0: vtable pointer (4 bytes)
     *   offset 4: _address.dword (4 bytes, network byte order) */
    uint32_t ip_nbo = mem_read32(cpu->mem, ip_ref + 4);
    struct sockaddr_in peer;
    memset(&peer, 0, sizeof(peer));
    peer.sin_family = AF_INET;
    peer.sin_port = htons((uint16_t)port);
    peer.sin_addr.s_addr = ip_nbo;

    char ip_str[INET_ADDRSTRLEN];
    inet_ntop(AF_INET, &peer.sin_addr, ip_str, sizeof(ip_str));
    wifi_log(ws, "start_ssl_client → %s:%u\n", ip_str, port);

    /* Create host socket */
    int hfd = socket(AF_INET, SOCK_STREAM, 0);
    if (hfd < 0) {
        wifi_log(ws, "start_ssl_client: socket() failed: %s\n", strerror(errno));
        ws_return(cpu, (uint32_t)-1);
        return;
    }

    /* Connect with 5-second timeout */
    int cret = connect(hfd, (struct sockaddr *)&peer, sizeof(peer));
    if (cret < 0 && errno == EINPROGRESS) {
        fd_set wset;
        FD_ZERO(&wset);
        FD_SET(hfd, &wset);
        struct timeval ctv = { .tv_sec = 5, .tv_usec = 0 };
        int sel = select(hfd + 1, NULL, &wset, NULL, &ctv);
        if (sel <= 0) {
            wifi_log(ws, "start_ssl_client: connect timed out\n");
            close(hfd);
            ws_return(cpu, (uint32_t)-1);
            return;
        }
        int err = 0;
        socklen_t elen = sizeof(err);
        getsockopt(hfd, SOL_SOCKET, SO_ERROR, &err, &elen);
        if (err != 0) {
            wifi_log(ws, "start_ssl_client: connect failed: %s\n", strerror(err));
            close(hfd);
            ws_return(cpu, (uint32_t)-1);
            return;
        }
    } else if (cret < 0) {
        wifi_log(ws, "start_ssl_client: connect failed: %s\n", strerror(errno));
        close(hfd);
        ws_return(cpu, (uint32_t)-1);
        return;
    }

    /* Allocate socket slot */
    int slot = slot_alloc(ws, hfd);
    if (slot < 0) {
        wifi_log(ws, "start_ssl_client: no free slots\n");
        close(hfd);
        ws_return(cpu, (uint32_t)-1);
        return;
    }

    emu_socket_t *s = &ws->sockets[slot];

    /* Attempt host-side TLS handshake */
    SSL_CTX *sctx = SSL_CTX_new(TLS_client_method());
    if (!sctx) {
        wifi_log(ws, "start_ssl_client: SSL_CTX_new failed\n");
        close(hfd);
        slot_free(ws, slot);
        ws_return(cpu, (uint32_t)-1);
        return;
    }

    SSL *ssl = SSL_new(sctx);
    SSL_set_fd(ssl, hfd);

    /* Set a 5-second recv timeout for the TLS handshake */
    struct timeval tv = { .tv_sec = 5, .tv_usec = 0 };
    setsockopt(hfd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    int ret = SSL_connect(ssl);

    /* Remove timeout */
    tv.tv_sec = 0; tv.tv_usec = 0;
    setsockopt(hfd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    if (ret == 1) {
        /* TLS handshake succeeded */
        s->ssl = ssl;
        s->ssl_ctx = sctx;
        wifi_log(ws, "start_ssl_client(fd %d, port %u): TLS OK\n", slot, port);
    } else {
        /* TLS failed — server doesn't speak TLS. Reconnect as plain TCP. */
        wifi_log(ws, "start_ssl_client(fd %d, port %u): TLS failed, plain TCP fallback\n",
                 slot, port);
        SSL_free(ssl);
        SSL_CTX_free(sctx);

        /* Reconnect: the TLS ClientHello corrupted the TCP stream */
        close(hfd);
        hfd = socket(AF_INET, SOCK_STREAM, 0);
        if (hfd < 0) {
            slot_free(ws, slot);
            ws_return(cpu, (uint32_t)-1);
            return;
        }
        s->host_fd = hfd;
        cret = connect(hfd, (struct sockaddr *)&peer, sizeof(peer));
        if (cret < 0 && errno == EINPROGRESS) {
            fd_set wset2;
            FD_ZERO(&wset2);
            FD_SET(hfd, &wset2);
            struct timeval ctv2 = { .tv_sec = 5, .tv_usec = 0 };
            select(hfd + 1, NULL, &wset2, NULL, &ctv2);
        } else if (cret < 0) {
            wifi_log(ws, "start_ssl_client: reconnect failed\n");
            slot_free(ws, slot);
            ws_return(cpu, (uint32_t)-1);
            return;
        }
        wifi_log(ws, "start_ssl_client(fd %d, port %u): reconnected as plain TCP\n",
                 slot, port);
    }

    /* Write socket slot to sslclient_context->socket */
    mem_write32(cpu->mem, ssl_ctx_addr + SSLCTX_SOCKET_OFS, (uint32_t)slot);

    ws_return(cpu, (uint32_t)slot);
}

/*
 * send_ssl_data(sslclient_context*, data, len)
 * Sends data via TLS (SSL_write) or plain TCP (send).
 */
static void stub_send_ssl_data(xtensa_cpu_t *cpu, void *ctx)
{
    wifi_stubs_t *ws = ctx;
    uint32_t ssl_ctx_addr = ws_arg(cpu, 0);
    uint32_t buf_addr     = ws_arg(cpu, 1);
    uint32_t len          = ws_arg(cpu, 2);

    int emufd;
    emu_socket_t *s = ssl_get_socket(ws, cpu, ssl_ctx_addr, &emufd);
    if (!s) { ws_return(cpu, (uint32_t)-1); return; }

    uint8_t *tmp = malloc(len);
    if (!tmp) { ws_return(cpu, (uint32_t)-1); return; }
    for (uint32_t i = 0; i < len; i++)
        tmp[i] = mem_read8(cpu->mem, buf_addr + i);

    ssize_t n;
    if (s->ssl)
        n = SSL_write(s->ssl, tmp, (int)len);
    else
        n = send(s->host_fd, tmp, len, MSG_NOSIGNAL);
    free(tmp);

    if (n > 0) {
        wifi_log(ws, "send_ssl(fd %d, %zd bytes)%s\n",
                 emufd, n, s->ssl ? " [TLS]" : "");
        s->awaiting_response = true;
    }

    ws_return(cpu, (uint32_t)(n > 0 ? (int)n : -1));
}

/*
 * get_ssl_receive(sslclient_context*, buf, len)
 * Receives data via TLS (SSL_read) or plain TCP (recv).
 */
static void stub_get_ssl_receive(xtensa_cpu_t *cpu, void *ctx)
{
    wifi_stubs_t *ws = ctx;
    uint32_t ssl_ctx_addr = ws_arg(cpu, 0);
    uint32_t buf_addr     = ws_arg(cpu, 1);
    int32_t  len          = (int32_t)ws_arg(cpu, 2);

    int emufd;
    emu_socket_t *s = ssl_get_socket(ws, cpu, ssl_ctx_addr, &emufd);
    if (!s || len <= 0) { ws_return(cpu, (uint32_t)-1); return; }

    uint8_t *tmp = malloc((size_t)len);
    if (!tmp) { ws_return(cpu, (uint32_t)-1); return; }

    ssize_t n;
    if (s->ssl) {
        /* Check for buffered data in OpenSSL first */
        int pending = SSL_pending(s->ssl);
        if (pending <= 0) {
            /* Poll the socket for incoming data */
            int poll_ms = s->awaiting_response ? 500 : 1;
            struct pollfd pfd = { .fd = s->host_fd, .events = POLLIN };
            if (poll(&pfd, 1, poll_ms) <= 0) {
                free(tmp);
                ws_return(cpu, (uint32_t)-1);
                return;
            }
        }
        n = SSL_read(s->ssl, tmp, len);
    } else {
        /* Plain TCP with adaptive timeout */
        int poll_ms;
        if (s->total_received == 0)
            poll_ms = 5000;    /* first data: up to 5s */
        else if (s->awaiting_response)
            poll_ms = 500;     /* expecting reply */
        else
            poll_ms = 1;       /* idle check */
        struct pollfd pfd = { .fd = s->host_fd, .events = POLLIN };
        if (poll(&pfd, 1, poll_ms) <= 0) {
            free(tmp);
            ws_return(cpu, (uint32_t)-1);
            return;
        }
        n = recv(s->host_fd, tmp, (size_t)len, 0);
    }

    if (n > 0) {
        for (ssize_t i = 0; i < n; i++)
            mem_write8(cpu->mem, buf_addr + (uint32_t)i, tmp[i]);
        s->total_received += (uint64_t)n;
        s->awaiting_response = false;
        wifi_log(ws, "recv_ssl(fd %d, %zd/%d bytes)%s\n",
                 emufd, n, len, s->ssl ? " [TLS]" : "");
    }

    free(tmp);
    ws_return(cpu, (uint32_t)(n > 0 ? (int)n : -1));
}

/*
 * data_to_read(sslclient_context*)
 * Returns number of bytes available to read.
 */
static void stub_data_to_read(xtensa_cpu_t *cpu, void *ctx)
{
    wifi_stubs_t *ws = ctx;
    uint32_t ssl_ctx_addr = ws_arg(cpu, 0);

    int emufd;
    emu_socket_t *s = ssl_get_socket(ws, cpu, ssl_ctx_addr, &emufd);
    if (!s) { ws_return(cpu, 0); return; }

    int avail = 0;
    if (s->ssl) {
        avail = SSL_pending(s->ssl);
        if (avail <= 0) {
            struct pollfd pfd = { .fd = s->host_fd, .events = POLLIN };
            if (poll(&pfd, 1, 0) > 0)
                avail = 1;  /* data on wire, not yet decrypted */
        }
    } else {
        ioctl(s->host_fd, FIONREAD, &avail);
    }

    ws_return(cpu, (uint32_t)avail);
}

/*
 * stop_ssl_socket(sslclient_context*, rootCA, cli_cert, cli_key)
 * Cleans up TLS session and closes the underlying socket.
 */
static void stub_stop_ssl_socket(xtensa_cpu_t *cpu, void *ctx)
{
    wifi_stubs_t *ws = ctx;
    uint32_t ssl_ctx_addr = ws_arg(cpu, 0);

    int emufd;
    emu_socket_t *s = ssl_get_socket(ws, cpu, ssl_ctx_addr, &emufd);
    if (s) {
        /* Only close sockets that were created by start_ssl_client
         * (they have SSL state or were created via the SSL path).
         * Refuse to close sockets that were created by lwip_socket
         * to prevent accidental closure of unrelated connections
         * (e.g., Stratum) due to uninitialized sslclient_context. */
        if (s->ssl || s->ssl_ctx) {
            wifi_log(ws, "stop_ssl(fd %d)%s\n", emufd, s->ssl ? " [TLS]" : "");
            close(s->host_fd);
            slot_free(ws, emufd);
        } else {
            wifi_log(ws, "stop_ssl(fd %d): SKIPPED — not an SSL socket (protecting plain TCP)\n", emufd);
        }
    } else {
        wifi_log(ws, "stop_ssl: ctx at 0x%08x has fd=%d (no slot)\n", ssl_ctx_addr, emufd);
    }

    /* Mark the firmware's sslclient socket as closed */
    mem_write32(cpu->mem, ssl_ctx_addr + SSLCTX_SOCKET_OFS, (uint32_t)-1);

    ws_return(cpu, 0);
}

/* ===== VFS wrappers ===== */

/* VFS wrappers — same logic as lwip_* versions.
 * WiFiClient calls these instead of lwip_fcntl/lwip_select. */
static void stub_vfs_fcntl(xtensa_cpu_t *cpu, void *ctx)
{
    stub_lwip_fcntl(cpu, ctx);
}

static void stub_vfs_select(xtensa_cpu_t *cpu, void *ctx)
{
    stub_lwip_select(cpu, ctx);
}

/* ===== Scratch memory allocation ===== */

/* Allocate a small region in emulator address space for hostent data.
 * We use the top of DRAM (0x3FFFFFFF area) — a 64-byte scratch. */
#define HOSTENT_SCRATCH_ADDR  0x3FFBFF00u
#define HOSTENT_SCRATCH_SIZE  64

/* ===== Public API ===== */

wifi_stubs_t *wifi_stubs_create(xtensa_cpu_t *cpu)
{
    wifi_stubs_t *ws = calloc(1, sizeof(*ws));
    if (!ws) return NULL;
    ws->cpu = cpu;
    ws->hostent_buf = HOSTENT_SCRATCH_ADDR;

    for (int i = 0; i < MAX_EMU_SOCKETS; i++)
        ws->sockets[i].host_fd = -1;

    return ws;
}

void wifi_stubs_destroy(wifi_stubs_t *ws)
{
    if (!ws) return;
    /* Close any open sockets */
    for (int i = 0; i < MAX_EMU_SOCKETS; i++) {
        if (ws->sockets[i].host_fd >= 0)
            close(ws->sockets[i].host_fd);
    }
    free(ws);
}

int wifi_stubs_hook_symbols(wifi_stubs_t *ws, const elf_symbols_t *syms)
{
    if (!ws || !syms) return 0;

    esp32_rom_stubs_t *rom = ws->cpu->pc_hook_ctx;
    if (!rom) return 0;
    ws->rom = rom;

    int hooked = 0;

    struct {
        const char *name;
        rom_stub_fn fn;
    } hooks[] = {
        /* Tier 1: Connection */
        { "lwip_socket",        stub_lwip_socket },
        { "lwip_connect",       stub_lwip_connect },
        { "lwip_close",         stub_lwip_close },

        /* Tier 2: Data transfer */
        { "lwip_write",         stub_lwip_write },
        { "lwip_send",          stub_lwip_send },
        { "lwip_read",          stub_lwip_read },
        { "lwip_recv",          stub_lwip_recv },
        { "lwip_sendto",        stub_lwip_sendto },
        { "lwip_recvfrom",      stub_lwip_recvfrom },

        /* Tier 3: DNS */
        { "lwip_gethostbyname", stub_lwip_gethostbyname },
        { "dns_gethostbyname",  stub_dns_gethostbyname },

        /* Tier 4: Multiplexing & config */
        { "lwip_select",        stub_lwip_select },
        { "lwip_ioctl",         stub_lwip_ioctl },
        { "lwip_setsockopt",    stub_lwip_setsockopt },
        { "lwip_getsockopt",    stub_lwip_getsockopt },
        { "lwip_fcntl",         stub_lwip_fcntl },
        { "lwip_getsockname",   stub_lwip_getsockname },

        /* VFS wrappers (WiFiClient uses these instead of lwip_*) */
        { "fcntl",              stub_vfs_fcntl },
        { "select",             stub_vfs_select },
        { "esp_vfs_select",     stub_vfs_select },

        /* Tier 5: Server stubs */
        { "lwip_bind",          stub_lwip_bind },
        { "lwip_listen",        stub_lwip_listen },
        { "lwip_accept",        stub_lwip_accept },

        /* Tier 6: Host-side TLS (replaces firmware mbedtls) */
        { "_Z16start_ssl_clientP17sslclient_contextRK9IPAddressjPKciS5_bS5_S5_S5_S5_bPS5_",
                                stub_start_ssl_client },
        { "_Z13send_ssl_dataP17sslclient_contextPKhj",
                                stub_send_ssl_data },
        { "_Z15get_ssl_receiveP17sslclient_contextPhi",
                                stub_get_ssl_receive },
        { "_Z12data_to_readP17sslclient_context",
                                stub_data_to_read },
        { "_Z15stop_ssl_socketP17sslclient_contextPKcS2_S2_",
                                stub_stop_ssl_socket },

        { NULL, NULL }
    };

    for (int i = 0; hooks[i].name; i++) {
        uint32_t addr;
        if (elf_symbols_find(syms, hooks[i].name, &addr) == 0) {
            rom_stubs_register_ctx(rom, addr, hooks[i].fn,
                                   hooks[i].name, ws);
            hooked++;
        }
    }

    /* Override POSIX syscall ROM stubs with socket-aware versions.
     * These are fallback paths — most socket I/O goes through lwip_*
     * hooks above. But some code may call read()/write()/close() directly. */
    rom_stubs_register_ctx(rom, 0x4000181C, stub_lwip_write, "write", ws);
    rom_stubs_register_ctx(rom, 0x400017DC, stub_lwip_read,  "read",  ws);
    rom_stubs_register_ctx(rom, 0x40001778, stub_lwip_close, "close", ws);
    hooked += 3;

    if (hooked > 0)
        fprintf(stderr, "[wifi] hooked %d lwip symbols\n", hooked);

    return hooked;
}

void wifi_stubs_set_event_log(wifi_stubs_t *ws, bool enabled) {
    if (ws) ws->event_log = enabled;
}
