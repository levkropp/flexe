#ifndef BT_STUBS_H
#define BT_STUBS_H

#include "xtensa.h"
#include "elf_symbols.h"

typedef struct bt_stubs bt_stubs_t;

bt_stubs_t *bt_stubs_create(xtensa_cpu_t *cpu);
void bt_stubs_destroy(bt_stubs_t *bt);

/* Look up ELF symbols and register PC hooks for BT/BLE functions */
int bt_stubs_hook_symbols(bt_stubs_t *bt, const elf_symbols_t *syms);

/* Enable event log mode: prefix output with [cycle] BT format */
void bt_stubs_set_event_log(bt_stubs_t *bt, bool enabled);

#endif /* BT_STUBS_H */
