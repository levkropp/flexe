/* Host TCP bridge and control-channel tests (loopback only). */
#include "test_helpers.h"
#include "uart_tcp.h"
#include "peripherals.h"
#include "target.h"

#include <string.h>

#ifdef _WIN32
#include <winsock2.h>
#else
#include <unistd.h>
#include <sys/socket.h>
#include <sys/select.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#endif

/* Bounded wait helpers: CI runners are shared and heavily loaded, so
 * synchronization points wait on wall-clock deadlines (with a short sleep
 * to stay friendly) instead of spinning a fixed syscall count. Usual case
 * still resolves on the first iteration. */
static void test_sleep_ms(unsigned ms)
{
#ifdef _WIN32
    Sleep(ms);
#else
    usleep(ms * 1000u);
#endif
}

TEST(uart_tcp_parse_endpoint_accepts_host_port_forms)
{
    char host[256];
    int port = 0;
    ASSERT_EQ(uart_tcp_parse_endpoint(NULL, host, sizeof(host), &port), -1);
    ASSERT_EQ(uart_tcp_parse_endpoint("127.0.0.1:5555", NULL, 0, &port),
              -1);
    ASSERT_EQ(uart_tcp_parse_endpoint("127.0.0.1:5555", host, sizeof(host),
                                       NULL),
              -1);
    ASSERT_EQ(uart_tcp_parse_endpoint("no-port", host, sizeof(host), &port),
              -1);
    ASSERT_EQ(uart_tcp_parse_endpoint("127.0.0.1:0", host, sizeof(host),
                                       &port),
              -1);
    ASSERT_EQ(uart_tcp_parse_endpoint("127.0.0.1:99999", host, sizeof(host),
                                       &port),
              -1);
    ASSERT_EQ(uart_tcp_parse_endpoint("127.0.0.1:5555", host, sizeof(host),
                                       &port),
              0);
    ASSERT_TRUE(strcmp(host, "127.0.0.1") == 0);
    ASSERT_EQ(port, 5555);
    ASSERT_EQ(uart_tcp_parse_endpoint(":5556", host, sizeof(host), &port),
              0);
    ASSERT_TRUE(strcmp(host, "127.0.0.1") == 0);
    ASSERT_EQ(port, 5556);
    ASSERT_EQ(uart_tcp_parse_endpoint("localhost:1234", host, sizeof(host),
                                       &port),
              0);
    ASSERT_EQ(port, 1234);
}

static int test_tcp_connect(int port)
{
    int fd = (int)socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) return -1;
    struct sockaddr_in sa;
    memset(&sa, 0, sizeof(sa));
    sa.sin_family = AF_INET;
    sa.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    sa.sin_port = htons((uint16_t)port);
    if (connect(fd, (struct sockaddr *)&sa, (socklen_t)sizeof(sa)) != 0) {
#ifdef _WIN32
        closesocket(fd);
#else
        close(fd);
#endif
        return -1;
    }
    return fd;
}

static void test_tcp_close(int fd)
{
#ifdef _WIN32
    closesocket(fd);
#else
    close(fd);
#endif
}

/* Pump until the bridge has accepted a client (accept() can lag a
 * completed connect() call, so a single pump is not enough). */
static void pump_until_client(uart_tcp_bridge_t *b, esp32_periph_t *periph)
{
    for (int i = 0;
         i < 10000 && !uart_tcp_bridge_has_client(b); i++) {
        uart_tcp_bridge_pump(b, periph);
        test_sleep_ms(1u);
    }
}

static int client_readable(int fd)
{
    fd_set set;
    FD_ZERO(&set);
    FD_SET((unsigned)fd, &set);
    struct timeval instant = {0, 0};
    int ready = select(fd + 1, &set, NULL, NULL, &instant);
    return ready > 0 && FD_ISSET(fd, &set);
}

TEST(uart_tcp_bridge_moves_bytes_both_ways)
{
    uart_tcp_bridge_t *bridge = uart_tcp_bridge_create("127.0.0.1", 0);
    /* Port 0 asks the OS for an ephemeral port: valid, like any listen. */
    ASSERT_TRUE(bridge != NULL);
    uart_tcp_bridge_destroy(bridge);

    bridge = uart_tcp_bridge_create("127.0.0.1", 0);
    ASSERT_TRUE(bridge != NULL);
    if (!bridge) return;
    int bridge_port = uart_tcp_bridge_port(bridge);
    ASSERT_TRUE(bridge_port > 0);
    ASSERT_EQ(uart_tcp_bridge_port(NULL), -1);
    ASSERT_EQ(uart_tcp_bridge_has_client(bridge), 0);
    /* Sending with no client is a silent no-op. */
    uart_tcp_bridge_send(bridge, 0xAAu);

    int client = test_tcp_connect(bridge_port);
    ASSERT_TRUE(client >= 0);
    if (client < 0) {
        uart_tcp_bridge_destroy(bridge);
        return;
    }

    const flexe_target_desc_t *s3 =
        flexe_target_by_id(FLEXE_TARGET_ESP32S3);
    xtensa_mem_t *mem = mem_create_for_target(s3);
    esp32_periph_t *periph = periph_create(mem);
    ASSERT_TRUE(mem != NULL);
    ASSERT_TRUE(periph != NULL);
    if (!mem || !periph) {
        periph_destroy(periph);
        mem_destroy(mem);
        test_tcp_close(client);
        uart_tcp_bridge_destroy(bridge);
        return;
    }

    /* Guest TX reaches the socket. */
    pump_until_client(bridge, periph);
    ASSERT_EQ(uart_tcp_bridge_has_client(bridge), 1);
    uart_tcp_bridge_send(bridge, 0x41u);
    uart_tcp_bridge_send(bridge, 0x42u);
    uart_tcp_bridge_pump(bridge, periph);
    char rx[4] = {0};
    size_t got = 0u;
    while (got < 2u) {
#ifdef _WIN32
        int n = recv(client, rx + got, (int)(2u - got), 0);
#else
        ssize_t n = recv(client, rx + got, 2u - got, 0);
#endif
        if (n <= 0) break;
        got += (size_t)n;
    }
    ASSERT_EQ(got, 2u);
    ASSERT_EQ(rx[0], 'A');
    ASSERT_EQ(rx[1], 'B');

    /* Socket bytes land in UART0 RX. */
    const char *msg = "hi";
#ifdef _WIN32
    send(client, msg, 2, 0);
#else
    ASSERT_TRUE(send(client, msg, 2, 0) == 2);
#endif
    const uint8_t direct[2] = {'x', 'y'};
    ASSERT_EQ(periph_uart_rx_inject(periph, direct, 2u), 2u);
    ASSERT_EQ(periph_uart_rx_pending_num(periph, 0), 2u);
    for (int i = 0;
         i < 10000 && periph_uart_rx_pending_num(periph, 0) < 4u; i++) {
        uart_tcp_bridge_pump(bridge, periph);
        test_sleep_ms(1u);
    }
    ASSERT_TRUE(periph_uart_rx_pending_num(periph, 0) >= 4u);

    /* A second connection replaces the first, like recabling UART. The
     * replacement send repeats until it lands: an early copy may still
     * address the previous client while its replacement pends in accept. */
    int client2 = test_tcp_connect(bridge_port);
    ASSERT_TRUE(client2 >= 0);
    if (client2 < 0) {
        test_tcp_close(client);
        periph_destroy(periph);
        mem_destroy(mem);
        uart_tcp_bridge_destroy(bridge);
        return;
    }
    char c = 0;
    int got_c = 0;
    for (unsigned spins = 0u; spins < 10000u && !got_c; spins++) {
        uart_tcp_bridge_pump(bridge, periph);
        uart_tcp_bridge_send(bridge, 0x43u);
        uart_tcp_bridge_pump(bridge, periph);
        if (client_readable(client2)) {
#ifdef _WIN32
            int n2 = recv(client2, &c, 1, 0);
#else
            ssize_t n2 = recv(client2, &c, 1, 0);
#endif
            got_c = n2 == 1 && c == 'C';
        }
        test_sleep_ms(1u);
    }
    ASSERT_TRUE(got_c);
    ASSERT_EQ(c, 'C');

    test_tcp_close(client);
    test_tcp_close(client2);
    periph_destroy(periph);
    mem_destroy(mem);
    uart_tcp_bridge_destroy(bridge);
}

TEST(ctrl_tcp_serves_line_commands)
{
    ctrl_tcp_server_t *ctrl = ctrl_tcp_create("127.0.0.1", 0);
    ASSERT_TRUE(ctrl != NULL);
    if (!ctrl) return;
    int ctrl_port = ctrl_tcp_port(ctrl);
    ASSERT_TRUE(ctrl_port > 0);
    ASSERT_EQ(ctrl_tcp_port(NULL), -1);
    int client = test_tcp_connect(ctrl_port);
    ASSERT_TRUE(client >= 0);
    if (client < 0) {
        ctrl_tcp_destroy(ctrl);
        return;
    }

    char line[256];
    ctrl_tcp_pump(ctrl);
    ASSERT_EQ(ctrl_tcp_poll_line(ctrl, line, sizeof(line)), 0);

    const char *cmd = "ping\n";
#ifdef _WIN32
    send(client, cmd, (int)strlen(cmd), 0);
#else
    ASSERT_TRUE(send(client, cmd, strlen(cmd), 0) == (ssize_t)strlen(cmd));
#endif
    int got_ping = 0;
    for (unsigned spins = 0u; spins < 10000u && !got_ping; spins++) {
        ctrl_tcp_pump(ctrl);
        if (ctrl_tcp_poll_line(ctrl, line, sizeof(line)) == 1)
            got_ping = strcmp(line, "ping") == 0;
        test_sleep_ms(1u);
    }
    ASSERT_TRUE(got_ping);
    ASSERT_EQ(ctrl_tcp_reply(ctrl, "ok"), 0);
    char reply[8] = {0};
    size_t reply_got = 0u;
    for (unsigned spins = 0u; spins < 10000u && reply_got < 3u; spins++) {
        if (client_readable(client)) {
#ifdef _WIN32
            int n = recv(client, reply + reply_got, (int)(3u - reply_got),
                         0);
#else
            ssize_t n = recv(client, reply + reply_got, 3u - reply_got, 0);
#endif
            if (n <= 0) break;
            reply_got += (size_t)n;
        }
        test_sleep_ms(1u);
    }
    ASSERT_EQ(reply_got, 3u);
    ASSERT_TRUE(memcmp(reply, "ok\n", 3) == 0);

    /* Partial lines wait; CRLF is tolerated. */
    const char *part1 = "re";
    const char *part2 = "set\r\n";
#ifdef _WIN32
    send(client, part1, 2, 0);
    send(client, part2, 5, 0);
#else
    ASSERT_TRUE(send(client, part1, 2, 0) == 2);
    ASSERT_TRUE(send(client, part2, 5, 0) == 5);
#endif
    int got_reset = 0;
    for (unsigned spins = 0u; spins < 10000u && !got_reset; spins++) {
        ctrl_tcp_pump(ctrl);
        if (ctrl_tcp_poll_line(ctrl, line, sizeof(line)) == 1)
            got_reset = strcmp(line, "reset") == 0;
        test_sleep_ms(1u);
    }
    ASSERT_TRUE(got_reset);

    /* Client disconnect is observed, not fatal. */
    test_tcp_close(client);
    ctrl_tcp_pump(ctrl);
    ASSERT_EQ(ctrl_tcp_poll_line(ctrl, line, sizeof(line)), 0);

    /* With no client at all, replies fail instead of blocking. */
    ctrl_tcp_server_t *lonely = ctrl_tcp_create("127.0.0.1", 0);
    ASSERT_TRUE(lonely != NULL);
    if (lonely) {
        ASSERT_EQ(ctrl_tcp_reply(lonely, "ok"), -1);
        ctrl_tcp_destroy(lonely);
    }

    ctrl_tcp_destroy(ctrl);
}

void run_uart_tcp_tests(void)
{
    TEST_SUITE("Host TCP bridges");
    RUN_TEST(uart_tcp_parse_endpoint_accepts_host_port_forms);
    RUN_TEST(uart_tcp_bridge_moves_bytes_both_ways);
    RUN_TEST(ctrl_tcp_serves_line_commands);
}
