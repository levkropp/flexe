/* HCI H4-over-TCP client for an external BLE controller (Bumble and any
 * H4-speaking peer). The emulator connects out; the controller answers
 * commands and emits events/ACL over the same stream. Framing follows the
 * standard H4 packet indicators (0x01 command, 0x02 ACL, 0x04 event). */
#ifndef FLEXE_BLE_HCI_H
#define FLEXE_BLE_HCI_H

#include <stddef.h>
#include <stdint.h>

#define BLE_HCI_H4_CMD 0x01u
#define BLE_HCI_H4_ACL 0x02u
#define BLE_HCI_H4_EVT 0x04u

/* Largest single H4 payload accepted (generous over the 27-byte legacy and
 * 251-byte extended controller limits). */
#define BLE_HCI_MAX_PACKET 2048u

typedef struct ble_hci_conn ble_hci_conn_t;

typedef struct {
    uint8_t type;
    /* Event: bytes are code, length, params. ACL: handle/flags, length, data.
     * Command complete/status parsing is the caller's job. */
    uint8_t payload[BLE_HCI_MAX_PACKET];
    size_t payload_len;
} ble_hci_packet_t;

/* Blocking connect with a bounded wait. Returns NULL on failure. */
ble_hci_conn_t *ble_hci_connect(const char *host, int port);
void ble_hci_close(ble_hci_conn_t *conn);

/* Send one H4 command. Returns 0 on success, -1 on transport failure. */
int ble_hci_send_cmd(ble_hci_conn_t *conn, uint16_t opcode,
                     const uint8_t *params, size_t params_len);

/* Send one pre-framed H4 payload (type byte + packet, without re-framing).
 * Returns 0 on success, -1 on transport failure. */
int ble_hci_send_raw(ble_hci_conn_t *conn, uint8_t type,
                     const uint8_t *payload, size_t payload_len);

/* Receive one H4 event/ACL packet, waiting up to timeout_ms (0 polls once).
 * Returns 1 with the packet filled, 0 on timeout, -1 on transport failure
 * or a malformed over-long packet (which resynchronizes the stream). */
int ble_hci_recv(ble_hci_conn_t *conn, ble_hci_packet_t *packet,
                 int timeout_ms);

#endif /* FLEXE_BLE_HCI_H */
