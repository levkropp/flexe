#ifndef WIFI_STUBS_H
#define WIFI_STUBS_H

#include "xtensa.h"
#include "elf_symbols.h"

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
} wifi_stubs_stats_t;

wifi_stubs_t *wifi_stubs_create(xtensa_cpu_t *cpu);
void wifi_stubs_destroy(wifi_stubs_t *ws);

/* Look up ELF symbols and register PC hooks for lwip socket functions */
int wifi_stubs_hook_symbols(wifi_stubs_t *ws, const elf_symbols_t *syms);

/* Register verified entry points for supported symbol-less production ROMs. */
int wifi_stubs_hook_firmware_addrs(wifi_stubs_t *ws, uint32_t entry_point);

/* Snapshot host-network and virtual-radio activity for integration gates. */
void wifi_stubs_get_stats(const wifi_stubs_t *ws, wifi_stubs_stats_t *stats);

/* Enable event log mode: prefix wifi output with [cycle] WIFI format */
void wifi_stubs_set_event_log(wifi_stubs_t *ws, bool enabled);

#endif /* WIFI_STUBS_H */
