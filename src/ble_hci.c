#include "ble_hci.h"

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#pragma comment(lib, "ws2_32.lib")
typedef int socklen_t;
#define CLOSESOCK(s) closesocket(s)
#define poll(fds, nfds, timeout) WSAPoll((fds), (nfds), (timeout))
typedef WSAPOLLFD ble_pollfd_t;
#define BLE_POLLIN POLLRDNORM
#define BLE_POLLOUT POLLWRNORM
#else
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <poll.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#define CLOSESOCK(s) close(s)
typedef struct pollfd ble_pollfd_t;
#define BLE_POLLIN POLLIN
#define BLE_POLLOUT POLLOUT
#endif
#define INVALID_SOCK (-1)

struct ble_hci_conn {
    int fd;
    /* Receive reassembly: a partial packet waits here between polls. */
    uint8_t pending[BLE_HCI_MAX_PACKET + 8u];
    size_t pending_len;
};

#ifdef _WIN32
static int ble_hci_wsa_ready(void)
{
    static int ready = -1;
    if (ready < 0) {
        WSADATA data;
        ready = WSAStartup(MAKEWORD(2, 2), &data) == 0 ? 1 : 0;
    }
    return ready;
}
#endif

static int ble_hci_nonblock(int fd, bool enable)
{
#ifdef _WIN32
    unsigned long mode = enable ? 1u : 0u;
    return ioctlsocket(fd, (long)FIONBIO, &mode);
#else
    int flags = fcntl(fd, F_GETFL, 0);
    if (flags < 0) return -1;
    if (enable)
        flags |= O_NONBLOCK;
    else
        flags &= ~O_NONBLOCK;
    return fcntl(fd, F_SETFL, flags);
#endif
}

ble_hci_conn_t *ble_hci_connect(const char *host, int port)
{
    if (!host || port < 1 || port > 65535) return NULL;
#ifdef _WIN32
    if (!ble_hci_wsa_ready()) return NULL;
#endif
    uint32_t addr;
    if (strcmp(host, "localhost") == 0) {
        addr = htonl(INADDR_LOOPBACK);
    } else {
        struct in_addr parsed;
#ifdef _WIN32
        parsed.s_addr = inet_addr(host);
        if (parsed.s_addr == INADDR_NONE &&
            strcmp(host, "255.255.255.255") != 0)
            return NULL;
#else
        if (inet_pton(AF_INET, host, &parsed) != 1) return NULL;
#endif
        addr = parsed.s_addr;
    }
    int fd = (int)socket(AF_INET, SOCK_STREAM, 0);
    if (fd == INVALID_SOCK) return NULL;
    if (ble_hci_nonblock(fd, true) != 0) {
        CLOSESOCK(fd);
        return NULL;
    }
    struct sockaddr_in sa;
    memset(&sa, 0, sizeof(sa));
    sa.sin_family = AF_INET;
    sa.sin_addr.s_addr = addr;
    sa.sin_port = htons((uint16_t)port);
    int rc = connect(fd, (struct sockaddr *)&sa, (socklen_t)sizeof(sa));
    if (rc != 0) {
#ifdef _WIN32
        if (WSAGetLastError() != WSAEWOULDBLOCK) {
            CLOSESOCK(fd);
            return NULL;
        }
#else
        if (errno != EINPROGRESS) {
            CLOSESOCK(fd);
            return NULL;
        }
#endif
        ble_pollfd_t pfd;
        pfd.fd = fd;
        pfd.events = BLE_POLLOUT;
        pfd.revents = 0;
        if (poll(&pfd, 1, 5000) <= 0) {
            CLOSESOCK(fd);
            return NULL;
        }
        int err = 0;
        socklen_t errlen = (socklen_t)sizeof(err);
        if (getsockopt(fd, SOL_SOCKET, SO_ERROR, (char *)&err, &errlen) != 0 ||
            err != 0) {
            CLOSESOCK(fd);
            return NULL;
        }
    }
    ble_hci_conn_t *conn = calloc(1u, sizeof(*conn));
    if (!conn) {
        CLOSESOCK(fd);
        return NULL;
    }
    conn->fd = fd;
    return conn;
}

void ble_hci_close(ble_hci_conn_t *conn)
{
    if (!conn) return;
    if (conn->fd != INVALID_SOCK) CLOSESOCK(conn->fd);
    free(conn);
}

int ble_hci_send_cmd(ble_hci_conn_t *conn, uint16_t opcode,
                     const uint8_t *params, size_t params_len)
{
    if (!conn || params_len > 255u ||
        (params_len != 0u && !params))
        return -1;
    uint8_t frame[4 + 255u];
    frame[0] = BLE_HCI_H4_CMD;
    frame[1] = (uint8_t)opcode;
    frame[2] = (uint8_t)(opcode >> 8);
    frame[3] = (uint8_t)params_len;
    if (params_len != 0u) memcpy(frame + 4u, params, params_len);
    size_t total = 4u + params_len;
    /* Localhost controller links drain immediately; retry transient
     * backpressure briefly rather than failing the guest transaction. */
    size_t done = 0u;
    for (unsigned attempt = 0u; attempt < 1000u && done < total;
         attempt++) {
#ifdef _WIN32
        int n = send(conn->fd, (const char *)frame + done,
                     (int)(total - done), 0);
#else
        ssize_t n = send(conn->fd, frame + done, total - done, 0);
#endif
        if (n <= 0) {
#ifdef _WIN32
            if (WSAGetLastError() != WSAEWOULDBLOCK) return -1;
#else
            if (errno != EAGAIN && errno != EWOULDBLOCK) return -1;
#endif
            continue;
        }
        done += (size_t)n;
    }
    return done == total ? 0 : -1;
}

int ble_hci_send_raw(ble_hci_conn_t *conn, uint8_t type,
                     const uint8_t *payload, size_t payload_len)
{
    if (!conn || payload_len > BLE_HCI_MAX_PACKET ||
        (payload_len != 0u && !payload) ||
        (type != BLE_HCI_H4_ACL && type != BLE_HCI_H4_EVT &&
         type != BLE_HCI_H4_CMD))
        return -1;
    /* One contiguous frame: the type byte and payload are not adjacent in
     * the caller's buffers, and partial sends must resume by offset. */
    size_t total = 1u + payload_len;
    uint8_t *frame = malloc(total);
    if (!frame) return -1;
    frame[0] = type;
    if (payload_len != 0u) memcpy(frame + 1u, payload, payload_len);
    size_t done = 0u;
    for (unsigned attempt = 0u; attempt < 1000u && done < total;
         attempt++) {
#ifdef _WIN32
        int n = send(conn->fd, (const char *)frame + done,
                     (int)(total - done), 0);
#else
        ssize_t n = send(conn->fd, frame + done, total - done, 0);
#endif
        if (n <= 0) {
#ifdef _WIN32
            if (WSAGetLastError() != WSAEWOULDBLOCK) break;
#else
            if (errno != EAGAIN && errno != EWOULDBLOCK) break;
#endif
            continue;
        }
        done += (size_t)n;
    }
    free(frame);
    return done == total ? 0 : -1;
}

int ble_hci_recv(ble_hci_conn_t *conn, ble_hci_packet_t *packet,
                 int timeout_ms)
{
    if (!conn || !packet) return -1;
    /* timeout_ms < 0 waits indefinitely; 0 polls the buffer and the
     * socket once; positive values bound the total wait. */
    int waited = 0;
    bool polled = false;
    for (;;) {
        /* 1. Try to complete a packet from buffered bytes. */
        if (conn->pending_len >= 1u) {
            uint8_t type = conn->pending[0];
            size_t need = 0u;
            if (type == BLE_HCI_H4_EVT && conn->pending_len >= 3u) {
                need = 3u + conn->pending[2];
            } else if (type == BLE_HCI_H4_ACL && conn->pending_len >= 5u) {
                need = 5u + (size_t)conn->pending[3] +
                       ((size_t)conn->pending[4] << 8);
            } else if (type != BLE_HCI_H4_EVT && type != BLE_HCI_H4_ACL) {
                /* Unknown indicator: drop one byte and resynchronize. */
                memmove(conn->pending, conn->pending + 1u,
                        conn->pending_len - 1u);
                conn->pending_len--;
                continue;
            }
            if (need != 0u) {
                if (need - 1u > BLE_HCI_MAX_PACKET) {
                    /* Over-long: drop the type byte and resynchronize. */
                    memmove(conn->pending, conn->pending + 1u,
                            conn->pending_len - 1u);
                    conn->pending_len--;
                    return -1;
                }
                if (conn->pending_len >= need) {
                    packet->type = type;
                    packet->payload_len = need - 1u;
                    memcpy(packet->payload, conn->pending + 1u,
                           need - 1u);
                    memmove(conn->pending, conn->pending + need,
                            conn->pending_len - need);
                    conn->pending_len -= need;
                    return 1;
                }
            }
        }
        /* 2. Need more bytes: stop if the budget is spent. */
        if (timeout_ms == 0) {
            if (polled) return 0;
        } else if (timeout_ms > 0 && waited >= timeout_ms) {
            return 0;
        }
        /* 3. Wait for readability, then move bytes into the buffer. */
        int step = 0;
        if (timeout_ms != 0) {
            step = timeout_ms < 0 ? 50 : timeout_ms - waited;
            if (step > 50) step = 50;
            if (step < 0) step = 0;
        }
        ble_pollfd_t pfd;
        pfd.fd = conn->fd;
        pfd.events = BLE_POLLIN;
        pfd.revents = 0;
        int ready = poll(&pfd, 1, step);
        polled = true;
        if (ready < 0) return -1;
        if (ready == 0) {
            waited += step;
            continue;
        }
        size_t room = sizeof(conn->pending) - conn->pending_len;
        if (room == 0u) {
            /* Buffer full without a complete packet: resynchronize. */
            conn->pending_len = 0u;
            return -1;
        }
#ifdef _WIN32
        int got = recv(conn->fd, (char *)conn->pending + conn->pending_len,
                       (int)room, 0);
#else
        ssize_t got = recv(conn->fd, conn->pending + conn->pending_len,
                           room, 0);
#endif
        if (got == 0) return -1;
        if (got < 0) {
#ifdef _WIN32
            if (WSAGetLastError() != WSAEWOULDBLOCK) return -1;
#else
            if (errno != EAGAIN && errno != EWOULDBLOCK) return -1;
#endif
            waited += step;
            continue;
        }
        conn->pending_len += (size_t)got;
    }
}
