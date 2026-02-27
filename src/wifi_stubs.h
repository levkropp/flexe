#ifndef WIFI_STUBS_H
#define WIFI_STUBS_H

#include "xtensa.h"
#include "elf_symbols.h"

typedef struct wifi_stubs wifi_stubs_t;

wifi_stubs_t *wifi_stubs_create(xtensa_cpu_t *cpu);
void wifi_stubs_destroy(wifi_stubs_t *ws);

/* Look up ELF symbols and register PC hooks for lwip socket functions */
int wifi_stubs_hook_symbols(wifi_stubs_t *ws, const elf_symbols_t *syms);

/* Enable event log mode: prefix wifi output with [cycle] WIFI format */
void wifi_stubs_set_event_log(wifi_stubs_t *ws, bool enabled);

#endif /* WIFI_STUBS_H */
