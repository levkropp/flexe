#include "uart_tcp.h"
#include "peripherals.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#pragma comment(lib, "ws2_32.lib")
typedef int socklen_t;
#define CLOSESOCK(s) closesocket(s)
#else
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#define CLOSESOCK(s) close(s)
#endif
#define INVALID_SOCK (-1)

#define UART_TCP_OUTBOX_BYTES (64u * 1024u)
#define UART_TCP_PENDING_BYTES (64u * 1024u)
#define CTRL_TCP_LINE_MAX 4096u

#ifdef _WIN32
static int uart_tcp_wsa_ready(void)
{
    static int ready = -1;
    if (ready < 0) {
        WSADATA data;
        ready = WSAStartup(MAKEWORD(2, 2), &data) == 0 ? 1 : 0;
    }
    return ready;
}
#endif

static int uart_tcp_nonblock(int fd)
{
#ifdef _WIN32
    unsigned long mode = 1u;
    return ioctlsocket(fd, (long)FIONBIO, &mode);
#else
    int flags = fcntl(fd, F_GETFL, 0);
    if (flags < 0) return -1;
    return fcntl(fd, F_SETFL, flags | O_NONBLOCK);
#endif
}

static int uart_tcp_would_block(void)
{
#ifdef _WIN32
    return WSAGetLastError() == WSAEWOULDBLOCK;
#else
    return errno == EAGAIN || errno == EWOULDBLOCK;
#endif
}

int uart_tcp_parse_endpoint(const char *spec, char *host, size_t host_len,
                             int *port)
{
    if (!spec || !host || host_len == 0u || !port) return -1;
    const char *colon = strrchr(spec, ':');
    if (!colon || colon[1] == '\0') return -1;
    size_t host_bytes = (size_t)(colon - spec);
    if (host_bytes >= host_len) return -1;
    if (host_bytes == 0u) {
        if (host_len < 10u) return -1;
        memcpy(host, "127.0.0.1", 10u);
    } else {
        memcpy(host, spec, host_bytes);
        host[host_bytes] = '\0';
    }
    char *end = NULL;
    long value = strtol(colon + 1, &end, 10);
    if (!end || *end != '\0' || value < 1L || value > 65535L) return -1;
    *port = (int)value;
    return 0;
}

static int uart_tcp_resolve(const char *host, uint32_t *out)
{
    if (strcmp(host, "localhost") == 0) {
        *out = htonl(INADDR_LOOPBACK);
        return 0;
    }
    struct in_addr parsed;
#ifdef _WIN32
    parsed.s_addr = inet_addr(host);
    if (parsed.s_addr == INADDR_NONE &&
        strcmp(host, "255.255.255.255") != 0)
        return -1;
#else
    if (inet_pton(AF_INET, host, &parsed) != 1) return -1;
#endif
    /* in_addr already holds network byte order for sin_addr. */
    *out = parsed.s_addr;
    return 0;
}

static int uart_tcp_listen(const char *host, int port)
{
#ifdef _WIN32
    if (!uart_tcp_wsa_ready()) return INVALID_SOCK;
#endif
    uint32_t bind_addr;
    if (uart_tcp_resolve(host, &bind_addr) != 0) return INVALID_SOCK;
    int fd = (int)socket(AF_INET, SOCK_STREAM, 0);
    if (fd == INVALID_SOCK) return INVALID_SOCK;
    int reuse = 1;
    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, (const char *)&reuse,
               (socklen_t)sizeof(reuse));
    struct sockaddr_in sa;
    memset(&sa, 0, sizeof(sa));
    sa.sin_family = AF_INET;
    sa.sin_addr.s_addr = bind_addr;
    sa.sin_port = htons((uint16_t)port);
    if (bind(fd, (struct sockaddr *)&sa, (socklen_t)sizeof(sa)) != 0 ||
        listen(fd, 1) != 0 || uart_tcp_nonblock(fd) != 0) {
        CLOSESOCK(fd);
        return INVALID_SOCK;
    }
    return fd;
}

static int uart_tcp_accept(int listen_fd)
{
    int fd = (int)accept(listen_fd, NULL, NULL);
    if (fd == INVALID_SOCK) return INVALID_SOCK;
    if (uart_tcp_nonblock(fd) != 0) {
        CLOSESOCK(fd);
        return INVALID_SOCK;
    }
    return fd;
}

struct uart_tcp_bridge {
    int listen_fd;
    int client_fd;
    uint8_t outbox[UART_TCP_OUTBOX_BYTES];
    size_t outbox_len;
    uint8_t pending[UART_TCP_PENDING_BYTES];
    size_t pending_len;
};

uart_tcp_bridge_t *uart_tcp_bridge_create(const char *host, int port)
{
    int fd = uart_tcp_listen(host, port);
    if (fd == INVALID_SOCK) return NULL;
    uart_tcp_bridge_t *b = calloc(1u, sizeof(*b));
    if (!b) {
        CLOSESOCK(fd);
        return NULL;
    }
    b->listen_fd = fd;
    b->client_fd = INVALID_SOCK;
    return b;
}

void uart_tcp_bridge_destroy(uart_tcp_bridge_t *b)
{
    if (!b) return;
    if (b->client_fd != INVALID_SOCK) CLOSESOCK(b->client_fd);
    CLOSESOCK(b->listen_fd);
    free(b);
}

int uart_tcp_bridge_has_client(const uart_tcp_bridge_t *b)
{
    return b != NULL && b->client_fd != INVALID_SOCK;
}

static int uart_tcp_socket_port(int fd)
{
    struct sockaddr_in sa;
    socklen_t len = (socklen_t)sizeof(sa);
    memset(&sa, 0, sizeof(sa));
    if (getsockname(fd, (struct sockaddr *)&sa, &len) != 0 ||
        sa.sin_family != AF_INET)
        return -1;
    return (int)ntohs(sa.sin_port);
}

int uart_tcp_bridge_port(const uart_tcp_bridge_t *b)
{
    if (!b) return -1;
    return uart_tcp_socket_port(b->listen_fd);
}

static void uart_tcp_drop_client(uart_tcp_bridge_t *b)
{
    if (b->client_fd != INVALID_SOCK) {
        CLOSESOCK(b->client_fd);
        b->client_fd = INVALID_SOCK;
    }
    b->outbox_len = 0u;
    b->pending_len = 0u;
}

void uart_tcp_bridge_send(uart_tcp_bridge_t *b, uint8_t byte)
{
    if (!b || b->client_fd == INVALID_SOCK) return;
    if (b->outbox_len > 0u ||
        send(b->client_fd, (const char *)&byte, 1, 0) != 1) {
        if (b->outbox_len >= sizeof(b->outbox)) {
            uart_tcp_drop_client(b);
            return;
        }
        if (b->outbox_len == 0u &&
            send(b->client_fd, (const char *)&byte, 1, 0) == 1)
            return;
        b->outbox[b->outbox_len++] = byte;
    }
}

static void uart_tcp_flush_outbox(uart_tcp_bridge_t *b)
{
    while (b->outbox_len > 0u) {
#ifdef _WIN32
        int done = send(b->client_fd, (const char *)b->outbox,
                        (int)b->outbox_len, 0);
#else
        ssize_t done = send(b->client_fd, b->outbox, b->outbox_len, 0);
#endif
        if (done <= 0) {
            if (!uart_tcp_would_block()) uart_tcp_drop_client(b);
            return;
        }
        memmove(b->outbox, b->outbox + done, b->outbox_len - (size_t)done);
        b->outbox_len -= (size_t)done;
    }
}

void uart_tcp_bridge_pump(uart_tcp_bridge_t *b, struct esp32_periph *periph)
{
    if (!b) return;
    int fresh = uart_tcp_accept(b->listen_fd);
    if (fresh != INVALID_SOCK) {
        /* One client at a time like a UART cable: a new connection
         * replaces the old one, and the listener stays up. */
        uart_tcp_drop_client(b);
        b->client_fd = fresh;
    }
    if (b->client_fd == INVALID_SOCK || !periph) return;
    uart_tcp_flush_outbox(b);
    if (b->client_fd == INVALID_SOCK) return;
    for (;;) {
        uint8_t chunk[4096];
        size_t room = sizeof(b->pending) - b->pending_len;
        size_t want = sizeof(chunk) < room ? sizeof(chunk) : room;
        if (want == 0u) break;
#ifdef _WIN32
        int got = recv(b->client_fd, (char *)chunk, (int)want, 0);
#else
        ssize_t got = recv(b->client_fd, chunk, want, 0);
#endif
        if (got == 0) {
            uart_tcp_drop_client(b);
            return;
        }
        if (got < 0) {
            if (!uart_tcp_would_block()) uart_tcp_drop_client(b);
            break;
        }
        memcpy(b->pending + b->pending_len, chunk, (size_t)got);
        b->pending_len += (size_t)got;
        if (b->pending_len >= sizeof(b->pending)) break;
    }
    while (b->pending_len > 0u) {
        size_t accepted =
            periph_uart_rx_inject(periph, b->pending, b->pending_len);
        if (accepted == 0u) break;
        memmove(b->pending, b->pending + accepted,
                b->pending_len - accepted);
        b->pending_len -= accepted;
    }
}

struct ctrl_tcp_server {
    int listen_fd;
    int client_fd;
    char line[CTRL_TCP_LINE_MAX + 1u];
    size_t line_len;
    int discarding;
};

ctrl_tcp_server_t *ctrl_tcp_create(const char *host, int port)
{
    int fd = uart_tcp_listen(host, port);
    if (fd == INVALID_SOCK) return NULL;
    ctrl_tcp_server_t *c = calloc(1u, sizeof(*c));
    if (!c) {
        CLOSESOCK(fd);
        return NULL;
    }
    c->listen_fd = fd;
    c->client_fd = INVALID_SOCK;
    return c;
}

void ctrl_tcp_destroy(ctrl_tcp_server_t *c)
{
    if (!c) return;
    if (c->client_fd != INVALID_SOCK) CLOSESOCK(c->client_fd);
    CLOSESOCK(c->listen_fd);
    free(c);
}

int ctrl_tcp_port(const ctrl_tcp_server_t *c)
{
    if (!c) return -1;
    return uart_tcp_socket_port(c->listen_fd);
}

void ctrl_tcp_pump(ctrl_tcp_server_t *c)
{
    if (!c) return;
    int fresh = uart_tcp_accept(c->listen_fd);
    if (fresh != INVALID_SOCK) {
        if (c->client_fd != INVALID_SOCK) CLOSESOCK(c->client_fd);
        c->client_fd = fresh;
        c->line_len = 0u;
        c->discarding = 0;
    }
}

int ctrl_tcp_poll_line(ctrl_tcp_server_t *c, char *line, size_t cap)
{
    if (!c || !line || cap == 0u) return -1;
    if (c->client_fd == INVALID_SOCK) return 0;
    for (;;) {
        char chunk[1024];
#ifdef _WIN32
        int got = recv(c->client_fd, chunk, (int)sizeof(chunk), 0);
#else
        ssize_t got = recv(c->client_fd, chunk, sizeof(chunk), 0);
#endif
        if (got == 0) {
            CLOSESOCK(c->client_fd);
            c->client_fd = INVALID_SOCK;
            c->line_len = 0u;
            c->discarding = 0;
            return 0;
        }
        if (got < 0) {
            if (!uart_tcp_would_block()) {
                CLOSESOCK(c->client_fd);
                c->client_fd = INVALID_SOCK;
                c->line_len = 0u;
                c->discarding = 0;
            }
            return 0;
        }
        for (ssize_t i = 0; i < got; i++) {
            char ch = chunk[i];
            if (c->discarding) {
                if (ch == '\n') c->discarding = 0;
                continue;
            }
            if (ch == '\n') {
                size_t n = c->line_len < cap - 1u ? c->line_len : cap - 1u;
                memcpy(line, c->line, n);
                line[n] = '\0';
                c->line_len = 0u;
                return 1;
            }
            if (ch == '\r') continue;
            if (c->line_len >= sizeof(c->line) - 1u) {
                c->discarding = 1;
                c->line_len = 0u;
                return -1;
            }
            c->line[c->line_len++] = ch;
        }
    }
}

int ctrl_tcp_reply(ctrl_tcp_server_t *c, const char *text)
{
    if (!c || !text || c->client_fd == INVALID_SOCK) return -1;
    size_t len = strlen(text);
    /* Bounded retries: a client that never drains must not hang the
     * emulator; persistent backpressure drops the connection instead. */
    for (size_t off = 0u, spins = 0u; off < len;) {
#ifdef _WIN32
        int done = send(c->client_fd, text + off, (int)(len - off), 0);
#else
        ssize_t done = send(c->client_fd, text + off, len - off, 0);
#endif
        if (done > 0) {
            off += (size_t)done;
            spins = 0u;
            continue;
        }
        if (done < 0 && uart_tcp_would_block() && ++spins < 100000u)
            continue;
        CLOSESOCK(c->client_fd);
        c->client_fd = INVALID_SOCK;
        c->line_len = 0u;
        c->discarding = 0;
        return -1;
    }
    const char newline = '\n';
    if (send(c->client_fd, &newline, 1, 0) != 1) {
        CLOSESOCK(c->client_fd);
        c->client_fd = INVALID_SOCK;
        c->line_len = 0u;
        c->discarding = 0;
        return -1;
    }
    return 0;
}
