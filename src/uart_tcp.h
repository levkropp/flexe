/* Host TCP bridges: a UART0 socket bridge for esptool `socket://` and a
 * newline-delimited control channel. Both servers are nonblocking and owned
 * outside the emulator session, so a software reset never drops the listen
 * sockets; per-UART taps are re-read from the live session on every pump. */
#ifndef FLEXE_UART_TCP_H
#define FLEXE_UART_TCP_H

#include <stddef.h>
#include <stdint.h>

typedef struct uart_tcp_bridge uart_tcp_bridge_t;
typedef struct ctrl_tcp_server ctrl_tcp_server_t;

/* Split "HOST:PORT" into host ("127.0.0.1" when empty) and port (1-65535).
 * Returns 0 on success, -1 otherwise. */
int uart_tcp_parse_endpoint(const char *spec, char *host, size_t host_len,
                             int *port);

/* UART0 bridge. Listens immediately; at most one client. While a client is
 * connected, guest TX belongs to the socket and host stdin/stdout stays out
 * of the UART path (the caller fans out accordingly). */
uart_tcp_bridge_t *uart_tcp_bridge_create(const char *host, int port);
void uart_tcp_bridge_destroy(uart_tcp_bridge_t *b);
int uart_tcp_bridge_has_client(const uart_tcp_bridge_t *b);
/* Bound port (host order), or -1 on error. Useful with port 0. */
int uart_tcp_bridge_port(const uart_tcp_bridge_t *b);
/* Guest TX byte toward the client; buffered while the socket would block. */
void uart_tcp_bridge_send(uart_tcp_bridge_t *b, uint8_t byte);
/* Accept/reconnect, move socket bytes into UART0 RX, flush the TX outbox.
 * Leftover RX past a full guest FIFO waits in the bridge for the next pump. */
struct esp32_periph;
void uart_tcp_bridge_pump(uart_tcp_bridge_t *b, struct esp32_periph *periph);

/* Control channel. One client at a time; commands are LF-terminated lines. */
ctrl_tcp_server_t *ctrl_tcp_create(const char *host, int port);
void ctrl_tcp_destroy(ctrl_tcp_server_t *c);
/* Returns 1 with the next complete line (NUL-terminated, newline stripped),
 * 0 when no full line is waiting, -1 when the line overflowed (the remainder
 * is discarded up to the newline). */
int ctrl_tcp_poll_line(ctrl_tcp_server_t *c, char *line, size_t cap);
/* Sends text plus a trailing newline. Returns 0 on success, -1 with no
 * client or on a send error (which drops the client). */
int ctrl_tcp_reply(ctrl_tcp_server_t *c, const char *text);
/* Bound port (host order), or -1 on error. Useful with port 0. */
int ctrl_tcp_port(const ctrl_tcp_server_t *c);
/* Service accepts and socket errors without transferring any command. */
void ctrl_tcp_pump(ctrl_tcp_server_t *c);

#endif /* FLEXE_UART_TCP_H */
