#ifndef ROM_STUBS_H
#define ROM_STUBS_H

#include "xtensa.h"
#include "elf_symbols.h"
#include <stdint.h>

typedef struct esp32_rom_stubs esp32_rom_stubs_t;
typedef void (*rom_stub_fn)(xtensa_cpu_t *cpu, void *user_ctx);

esp32_rom_stubs_t *rom_stubs_create(xtensa_cpu_t *cpu);
void rom_stubs_destroy(esp32_rom_stubs_t *stubs);
int  rom_stubs_register(esp32_rom_stubs_t *stubs, uint32_t addr,
                        rom_stub_fn fn, const char *name);
int  rom_stubs_register_ctx(esp32_rom_stubs_t *stubs, uint32_t addr,
                             rom_stub_fn fn, const char *name, void *user_ctx);
int  rom_stubs_register_spy(esp32_rom_stubs_t *stubs, uint32_t addr,
                             rom_stub_fn fn, const char *name, void *user_ctx);

/* Output capture (ets_printf / ets_write_char go here) */
int  rom_stubs_output_count(const esp32_rom_stubs_t *stubs);
const char *rom_stubs_output_buf(const esp32_rom_stubs_t *stubs);
void rom_stubs_output_clear(esp32_rom_stubs_t *stubs);

/* Verbose logging callback (called before each ROM stub executes) */
typedef void (*rom_log_fn)(void *ctx, uint32_t addr, const char *name,
                           const xtensa_cpu_t *cpu);
void rom_stubs_set_log_callback(esp32_rom_stubs_t *stubs, rom_log_fn fn, void *ctx);

/* Statistics: iterate over stubs with call counts */
int rom_stubs_stub_count(const esp32_rom_stubs_t *stubs);
int rom_stubs_get_stats(const esp32_rom_stubs_t *stubs, int index,
                        const char **name_out, uint32_t *addr_out, uint32_t *count_out);

/* Hook known firmware functions by symbol name (e.g. newlib locks) */
int rom_stubs_hook_symbols(esp32_rom_stubs_t *stubs, const elf_symbols_t *syms);

/* Total stub calls (running counter across all stubs) */
uint32_t rom_stubs_total_calls(const esp32_rom_stubs_t *stubs);

/* Count of unregistered ROM calls (fallback handler) */
int rom_stubs_unregistered_count(const esp32_rom_stubs_t *stubs);

/* PC hook bitmap: fast test to skip pc_hook calls for non-hooked addresses.
 * Covers instruction-space addresses (word-aligned, >> 2) with possible
 * false positives but no false negatives. */
#define HOOK_BITMAP_BITS  (1u << 19)  /* 512K bits = 64KB */
#define HOOK_BITMAP_WORDS (HOOK_BITMAP_BITS / 64)

static inline int rom_stubs_hook_bitmap_test(const uint64_t *bitmap, uint32_t pc) {
    uint32_t idx = (pc >> 2) & (HOOK_BITMAP_BITS - 1);
    return (bitmap[idx / 64] >> (idx & 63)) & 1;
}

/* Get the hook bitmap pointer (for use in hot path) */
const uint64_t *rom_stubs_get_hook_bitmap(const esp32_rom_stubs_t *stubs);

/* Dual-core boot support */
void rom_stubs_set_single_core(esp32_rom_stubs_t *stubs, bool single_core);
bool rom_stubs_app_cpu_start_requested(const esp32_rom_stubs_t *stubs);
uint32_t rom_stubs_app_cpu_boot_addr(const esp32_rom_stubs_t *stubs);
void rom_stubs_clear_app_cpu_start(esp32_rom_stubs_t *stubs);

#endif /* ROM_STUBS_H */
