#ifndef BT_STUBS_H
#define BT_STUBS_H

#include "xtensa.h"
#include "elf_symbols.h"
#include <stddef.h>

typedef struct bt_stubs bt_stubs_t;

typedef struct {
    uint64_t scan_callback_config_calls;
    uint64_t scan_start_calls;
    uint64_t scan_stop_calls;
    uint64_t advertisement_frames;
    uint64_t advertisement_callback_failures;
} bt_stubs_stats_t;

bt_stubs_t *bt_stubs_create(xtensa_cpu_t *cpu);
void bt_stubs_destroy(bt_stubs_t *bt);

/* Look up ELF symbols and register PC hooks for BT/BLE functions */
int bt_stubs_hook_symbols(bt_stubs_t *bt, const elf_symbols_t *syms);

/* Observe the real NimBLE scan lifecycle in verified symbol-less production
 * ROMs without replacing the firmware's scanner implementation. */
int bt_stubs_hook_firmware_addrs(bt_stubs_t *bt, uint32_t entry_point);

/* Snapshot virtual-controller activity for integration gates. */
void bt_stubs_get_stats(const bt_stubs_t *bt, bt_stubs_stats_t *stats);

/* Deliver one legacy BLE advertisement through the stock firmware's actual
 * NimBLE GAP handler and registered advertised-device callback. */
int bt_stubs_inject_advertisement(bt_stubs_t *bt, const uint8_t addr[6],
                                  uint8_t addr_type, int8_t rssi,
                                  const uint8_t *data, size_t len);

/* Enable event log mode: prefix output with [cycle] BT format */
void bt_stubs_set_event_log(bt_stubs_t *bt, bool enabled);

#endif /* BT_STUBS_H */
