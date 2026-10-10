#ifndef BT_STUBS_H
#define BT_STUBS_H

#include "xtensa.h"
#include "elf_symbols.h"
#include "ble_hci.h"
#include <stddef.h>

typedef struct bt_stubs bt_stubs_t;

/* Host-side virtual-controller sink for a legacy BLE advertising event. */
typedef void (*bt_advertisement_tx_cb)(void *ctx,
                                       const uint8_t *advertisement,
                                       size_t advertisement_len,
                                       const uint8_t *scan_response,
                                       size_t scan_response_len);

typedef struct {
    uint64_t scan_callback_config_calls;
    uint64_t scan_start_calls;
    uint64_t scan_stop_calls;
    uint64_t hci_scan_parameters_calls;
    uint64_t hci_scan_enable_calls;
    uint64_t hci_scan_disable_calls;
    uint64_t connection_capacity_queries;
    uint64_t identity_address_queries;
    uint64_t gap_advertising_start_calls;
    uint32_t last_advertising_duration_ms;
    uint8_t last_advertising_own_addr_type;
    uint8_t last_advertising_conn_mode;
    uint8_t last_advertising_disc_mode;
    uint8_t last_advertising_high_duty;
    uint64_t advertisement_frames;
    uint64_t advertisement_callback_failures;
    uint64_t hci_command_calls;
    uint64_t hci_virtual_completions;
    uint64_t advertising_data_calls;
    uint64_t advertising_scan_response_calls;
    uint64_t advertising_parameters_calls;
    uint64_t advertising_enable_calls;
    uint64_t advertising_disable_calls;
    uint64_t advertisement_tx_frames;
    uint64_t advertisement_tx_bytes;
    uint64_t advertisement_tx_failures;
    uint64_t hci_forwarded_commands;
    uint64_t hci_forwarded_events;
    uint64_t hci_forwarded_acl;
    uint64_t hci_backend_timeouts;
    uint64_t hci_injected_events;
    uint64_t hci_injection_failures;
} bt_stubs_stats_t;

bt_stubs_t *bt_stubs_create(xtensa_cpu_t *cpu);
void bt_stubs_destroy(bt_stubs_t *bt);

/* Look up ELF symbols and register PC hooks for BT/BLE functions */
int bt_stubs_hook_symbols(bt_stubs_t *bt, const elf_symbols_t *syms);

/* Observe the real NimBLE lifecycle and attach its HCI transport to the
 * virtual controller in verified symbol-less production ROMs. */
int bt_stubs_hook_firmware_addrs(bt_stubs_t *bt, uint32_t entry_point);

/* Hook only the NimBLE HCI transport for an external controller backend:
 * forwards ble_hs_hci_cmd_tx and resolves (without hooking) the receive and
 * mbuf entry points. Controller init, GAP, and GATT keep running natively,
 * unlike the broader compatibility hook set above. Requires --elf. */
int bt_stubs_hook_hci_transport(bt_stubs_t *bt, const elf_symbols_t *syms);

/* Attach an external HCI controller. All commands forward instead of using
 * the virtual controller; events/ACL inject through the pump below.
 * NULL detaches and restores virtual behavior. */
void bt_stubs_set_hci_backend(bt_stubs_t *bt, ble_hci_conn_t *conn);

/* Drain controller packets into the guest host at a safe boundary. Queues
 * what cannot be delivered yet for the next pump. `peer` is the other
 * core (or NULL); delivery borrows whichever core is quiescent. */
void bt_stubs_hci_pump(bt_stubs_t *bt, xtensa_cpu_t *peer);

/* Snapshot virtual-controller activity for integration gates. */
void bt_stubs_get_stats(const bt_stubs_t *bt, bt_stubs_stats_t *stats);

/* Deliver one legacy BLE advertisement through the stock firmware's actual
 * NimBLE GAP handler and registered advertised-device callback. */
int bt_stubs_inject_advertisement(bt_stubs_t *bt, const uint8_t addr[6],
                                  uint8_t addr_type, int8_t rssi,
                                  const uint8_t *data, size_t len);

/* Connect NimBLE's outgoing HCI advertising path to a host radio backend. */
void bt_stubs_set_advertisement_tx_callback(bt_stubs_t *bt,
                                             bt_advertisement_tx_cb cb,
                                             void *ctx);

/* Enable event log mode: prefix output with [cycle] BT format */
void bt_stubs_set_event_log(bt_stubs_t *bt, bool enabled);

#endif /* BT_STUBS_H */
