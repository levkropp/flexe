/* Target-described SENSITIVE v1 memory-protection register block. */
#ifndef FLEXE_SENSITIVE_MEMPROT_H
#define FLEXE_SENSITIVE_MEMPROT_H

#include <stdbool.h>
#include <stdint.h>

#include "memory.h"

typedef struct flexe_sensitive_memprot flexe_sensitive_memprot_t;

/* Firmware assigns the eight cache data-array banks and seven internal SRAM
 * banks before enabling caches.  Keep the state normalized instead of making
 * downstream cache/power models decode SENSITIVE register words. */
typedef struct {
    bool cache_data_array_locked;
    uint8_t cache_data_array_connection;
    bool internal_sram_usage_locked;
    uint8_t internal_sram_icache_usage;
    uint8_t internal_sram_dcache_usage;
    uint8_t internal_sram_cpu_usage;
    uint8_t core0_trace_usage;
    uint8_t core1_trace_usage;
    uint8_t core0_trace_allocation;
    uint8_t core1_trace_allocation;
    uint8_t mac_dump_usage;
    uint8_t log_usage;
    bool retention_disabled;
} flexe_sensitive_memory_usage_state_t;

typedef void (*flexe_sensitive_memory_usage_fn)(
    void *ctx, const flexe_sensitive_memory_usage_state_t *state);

/* Unknown offsets remain diagnostic through the fallback handlers. */
flexe_sensitive_memprot_t *flexe_sensitive_memprot_create(
    xtensa_mem_t *mem, mmio_read_fn fallback_read,
    mmio_write_fn fallback_write, void *fallback_ctx);
void flexe_sensitive_memprot_destroy(flexe_sensitive_memprot_t *memprot);

bool flexe_sensitive_memprot_memory_usage(
    const flexe_sensitive_memprot_t *memprot,
    flexe_sensitive_memory_usage_state_t *state);
void flexe_sensitive_memprot_set_memory_usage_listener(
    flexe_sensitive_memprot_t *memprot,
    flexe_sensitive_memory_usage_fn fn, void *ctx);

#endif /* FLEXE_SENSITIVE_MEMPROT_H */
