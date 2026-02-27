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

#endif /* ROM_STUBS_H */
