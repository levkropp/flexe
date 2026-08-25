#ifndef WIFI_STUBS_H
#define WIFI_STUBS_H

#include "xtensa.h"
#include "elf_symbols.h"
#include <stddef.h>

typedef struct wifi_stubs wifi_stubs_t;

typedef struct {
    uint64_t socket_calls;
    uint64_t socket_successes;
    uint64_t connect_calls;
    uint64_t connect_successes;
    uint64_t close_calls;
    uint64_t bind_calls;
    uint64_t bind_successes;
    uint64_t listen_calls;
    uint64_t listen_successes;
    uint64_t accept_calls;
    uint64_t accept_successes;
    uint64_t send_calls;
    uint64_t send_bytes;
    uint64_t recv_calls;
    uint64_t recv_bytes;
    uint64_t sendto_calls;
    uint64_t sendto_bytes;
    uint64_t recvfrom_calls;
    uint64_t recvfrom_bytes;
    uint64_t dns_calls;
    uint64_t wifi_connect_calls;
    uint64_t scan_start_calls;
    uint64_t wifi_init_calls;
    uint64_t wifi_start_calls;
    uint64_t wifi_set_mode_calls;
    uint64_t promisc_enable_calls;
    uint64_t promisc_callback_calls;
    uint64_t raw_tx_frames;
    uint64_t raw_rx_frames;
    uint64_t raw_rx_callback_failures;
} wifi_stubs_stats_t;

wifi_stubs_t *wifi_stubs_create(xtensa_cpu_t *cpu);
void wifi_stubs_destroy(wifi_stubs_t *ws);

/* Look up ELF symbols and register PC hooks for lwip socket functions */
int wifi_stubs_hook_symbols(wifi_stubs_t *ws, const elf_symbols_t *syms);

/* Register verified entry points for supported symbol-less production ROMs. */
int wifi_stubs_hook_firmware_addrs(wifi_stubs_t *ws, uint32_t entry_point);

/* Snapshot host-network and virtual-radio activity for integration gates. */
void wifi_stubs_get_stats(const wifi_stubs_t *ws, wifi_stubs_stats_t *stats);

/* Resolve a firmware-side bound port to the loopback port selected by the
 * host.  `datagram` distinguishes UDP from TCP when both use the same port. */
int wifi_stubs_get_bound_host_port(const wifi_stubs_t *ws,
                                   uint16_t firmware_port, bool datagram,
                                   uint16_t *host_port_out);

/* Deliver one host-supplied 802.11 frame through the firmware's registered
 * promiscuous callback. packet_type uses ESP-IDF's WIFI_PKT_* numeric values
 * (0 management, 1 control, 2 data, 3 misc). */
int wifi_stubs_inject_promiscuous_frame(wifi_stubs_t *ws,
                                        const uint8_t *frame, size_t len,
                                        int8_t rssi, uint8_t channel,
                                        uint32_t packet_type);

/* Enable event log mode: prefix wifi output with [cycle] WIFI format */
void wifi_stubs_set_event_log(wifi_stubs_t *ws, bool enabled);

#endif /* WIFI_STUBS_H */
