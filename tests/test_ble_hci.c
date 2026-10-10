/* H4 transport framing tests over loopback (no controller needed). */
#include "test_helpers.h"
#include "ble_hci.h"

#include <string.h>

#ifdef _WIN32
#include <winsock2.h>
#else
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#endif

static int loopback_listen(void)
{
    int fd = (int)socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) return -1;
    int reuse = 1;
    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, (const char *)&reuse,
               (socklen_t)sizeof(reuse));
    struct sockaddr_in sa;
    memset(&sa, 0, sizeof(sa));
    sa.sin_family = AF_INET;
    sa.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    sa.sin_port = 0;
    if (bind(fd, (struct sockaddr *)&sa, (socklen_t)sizeof(sa)) != 0 ||
        listen(fd, 1) != 0)
        return -1;
    return fd;
}

static int loopback_port(int fd)
{
    struct sockaddr_in sa;
    socklen_t len = (socklen_t)sizeof(sa);
    if (getsockname(fd, (struct sockaddr *)&sa, &len) != 0) return -1;
    return (int)ntohs(sa.sin_port);
}

/* recv(0) polls once, so loopback delivery can lag it by microseconds.
 * Retry briefly like the production pump loop does every batch. */
static int recv_until(ble_hci_conn_t *conn, ble_hci_packet_t *packet)
{
    for (int i = 0; i < 2000; i++) {
        int rc = ble_hci_recv(conn, packet, 0);
        if (rc != 0) return rc;
    }
    return ble_hci_recv(conn, packet, 0);
}

TEST(ble_hci_frames_events_across_partial_reads)
{
    int listen_fd = loopback_listen();
    ASSERT_TRUE(listen_fd >= 0);
    if (listen_fd < 0) return;
    int port = loopback_port(listen_fd);
    ASSERT_TRUE(port > 0);
    ble_hci_conn_t *conn = ble_hci_connect("127.0.0.1", port);
    ASSERT_TRUE(conn != NULL);
    if (!conn) {
#ifdef _WIN32
        closesocket(listen_fd);
#else
        close(listen_fd);
#endif
        return;
    }
    int peer = (int)accept(listen_fd, NULL, NULL);
    ASSERT_TRUE(peer >= 0);

    /* Event split across two segments reassembles. */
    const uint8_t part1[] = {0x04, 0x0E};
    const uint8_t part2[] = {0x04, 0x01, 0x03, 0x0C, 0x00};
    ble_hci_packet_t packet;
    ASSERT_EQ(ble_hci_recv(conn, &packet, 0), 0);
#ifdef _WIN32
    send(peer, (const char *)part1, 2, 0);
#else
    ASSERT_TRUE(send(peer, part1, 2, 0) == 2);
#endif
    ASSERT_EQ(ble_hci_recv(conn, &packet, 0), 0);
#ifdef _WIN32
    send(peer, (const char *)part2, 5, 0);
#else
    ASSERT_TRUE(send(peer, part2, 5, 0) == 5);
#endif
    ASSERT_EQ(recv_until(conn, &packet), 1);
    ASSERT_EQ(packet.type, BLE_HCI_H4_EVT);
    ASSERT_EQ(packet.payload_len, 6u);
    ASSERT_EQ(packet.payload[0], 0x0Eu);
    ASSERT_EQ(packet.payload[5], 0x00u);

    /* Unknown indicator byte resynchronizes without losing the packet. */
    const uint8_t junk[] = {0xFF, 0x04, 0x0E, 0x01, 0x01, 0x00};
#ifdef _WIN32
    send(peer, (const char *)junk, 6, 0);
#else
    ASSERT_TRUE(send(peer, junk, 6, 0) == 6);
#endif
    ASSERT_EQ(recv_until(conn, &packet), 1);
    ASSERT_EQ(packet.type, BLE_HCI_H4_EVT);
    ASSERT_EQ(packet.payload[0], 0x0Eu);

    /* ACL framing with a 16-bit length. */
    const uint8_t acl[] = {0x02, 0x01, 0x00, 0x03, 0x00, 0xAA, 0xBB, 0xCC};
#ifdef _WIN32
    send(peer, (const char *)acl, 8, 0);
#else
    ASSERT_TRUE(send(peer, acl, 8, 0) == 8);
#endif
    ASSERT_EQ(recv_until(conn, &packet), 1);
    ASSERT_EQ(packet.type, BLE_HCI_H4_ACL);
    ASSERT_EQ(packet.payload_len, 7u);
    ASSERT_EQ(packet.payload[6], 0xCCu);

    ble_hci_close(conn);
#ifdef _WIN32
    closesocket(peer);
    closesocket(listen_fd);
#else
    close(peer);
    close(listen_fd);
#endif
}

TEST(ble_hci_send_cmd_frames_wire_bytes)
{
    int listen_fd = loopback_listen();
    ASSERT_TRUE(listen_fd >= 0);
    if (listen_fd < 0) return;
    int port = loopback_port(listen_fd);
    ble_hci_conn_t *conn = ble_hci_connect("127.0.0.1", port);
    ASSERT_TRUE(conn != NULL);
    if (!conn) return;
    int peer = (int)accept(listen_fd, NULL, NULL);
    ASSERT_TRUE(peer >= 0);

    const uint8_t params[2] = {0x01, 0x02};
    ASSERT_EQ(ble_hci_send_cmd(conn, 0x0C03u, params, 2u), 0);
    uint8_t wire[6] = {0};
    size_t got = 0u;
    while (got < sizeof(wire)) {
#ifdef _WIN32
        int n = recv(peer, (char *)wire + got, (int)(sizeof(wire) - got),
                     0);
#else
        ssize_t n = recv(peer, wire + got, sizeof(wire) - got, 0);
#endif
        if (n <= 0) break;
        got += (size_t)n;
    }
    ASSERT_EQ(got, 6u);
    ASSERT_EQ(wire[0], 0x01u);
    ASSERT_EQ(wire[1], 0x03u);
    ASSERT_EQ(wire[2], 0x0Cu);
    ASSERT_EQ(wire[3], 0x02u);
    ASSERT_EQ(wire[4], 0x01u);
    ASSERT_EQ(wire[5], 0x02u);

    ble_hci_close(conn);
#ifdef _WIN32
    closesocket(peer);
    closesocket(listen_fd);
#else
    close(peer);
    close(listen_fd);
#endif
}

void run_ble_hci_tests(void)
{
    TEST_SUITE("BLE HCI transport");
    RUN_TEST(ble_hci_frames_events_across_partial_reads);
    RUN_TEST(ble_hci_send_cmd_frames_wire_bytes);
}
