#ifndef WIFI_STUBS_H
#define WIFI_STUBS_H

#include "xtensa.h"
#include "elf_symbols.h"
#include <stddef.h>

typedef struct wifi_stubs wifi_stubs_t;

/* Host-side virtual-radio sink for raw frames emitted by esp_wifi_80211_tx. */
typedef void (*wifi_raw_tx_cb)(void *ctx, uint32_t iface,
                               const uint8_t *frame, size_t len,
                               bool en_sys_seq);

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
    /* WiFi/IP events actually delivered to a handler the firmware
     * registered. Zero means the firmware never learned it associated. */
    uint64_t events_delivered;
    uint64_t scan_start_calls;
    uint64_t wifi_init_calls;
    uint64_t wifi_start_calls;
    uint64_t wifi_set_mode_calls;
    uint64_t promisc_enable_calls;
    uint64_t promisc_callback_calls;
    uint64_t raw_tx_frames;
    uint64_t raw_tx_bytes;
    uint64_t raw_tx_failures;
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

/* Connect the firmware's raw 802.11 transmit path to a host radio backend. */
void wifi_stubs_set_raw_tx_callback(wifi_stubs_t *ws, wifi_raw_tx_cb cb,
                                    void *ctx);

/* Deliver one host-supplied 802.11 frame through the firmware's registered
 * promiscuous callback. packet_type uses ESP-IDF's WIFI_PKT_* numeric values
 * (0 management, 1 control, 2 data, 3 misc). */
int wifi_stubs_inject_promiscuous_frame(wifi_stubs_t *ws,
                                        const uint8_t *frame, size_t len,
                                        int8_t rssi, uint8_t channel,
                                        uint32_t packet_type);

/* Enable event log mode: prefix wifi output with [cycle] WIFI format */
void wifi_stubs_set_event_log(wifi_stubs_t *ws, bool enabled);

/* Pre-provision station credentials, reported by esp_wifi_get_config() as
 * though a previous run had saved them. Firmware that provisions WiFi through
 * a captive portal checks for saved credentials to decide whether to start
 * the portal, so without these it can never be driven past provisioning. */
void wifi_stubs_set_sta_credentials(wifi_stubs_t *ws, const char *ssid,
                                    const char *password);

/* Deliver one queued WiFi/IP event to the firmware's registered handlers.
 * Call between execution batches: handlers run guest code and call back into
 * these stubs, so they must not be dispatched from inside one. */
void wifi_stubs_tick(wifi_stubs_t *ws, xtensa_cpu_t *cpu);

/* Resolve every hostname to this address (network byte order); 0 disables.
 * An emulated device has no route to the real internet, and an unresolvable
 * name surfaces as EINVAL from sendto(), far from its cause. */
void wifi_stubs_set_dns_override(wifi_stubs_t *ws, uint32_t addr_net_order);

/* Host-supplied settings that must outlive a software reset: the firmware
 * reboots precisely to come back up with what it saved, and the emulated
 * network it comes back to has to be the same one. */
typedef struct {
    char     sta_ssid[33];
    char     sta_password[65];
    uint32_t dns_override;
} wifi_host_config_t;

void wifi_stubs_snapshot_host_config(const wifi_stubs_t *ws,
                                     wifi_host_config_t *out);
void wifi_stubs_apply_host_config(wifi_stubs_t *ws,
                                  const wifi_host_config_t *cfg);

#endif /* WIFI_STUBS_H */
