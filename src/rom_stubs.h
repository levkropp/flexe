#ifndef ROM_STUBS_H
#define ROM_STUBS_H

#include "xtensa.h"
#include <stdint.h>

typedef struct esp32_rom_stubs esp32_rom_stubs_t;
typedef void (*rom_stub_fn)(xtensa_cpu_t *cpu, void *user_ctx);

esp32_rom_stubs_t *rom_stubs_create(xtensa_cpu_t *cpu);
void rom_stubs_destroy(esp32_rom_stubs_t *stubs);
int  rom_stubs_register(esp32_rom_stubs_t *stubs, uint32_t addr,
                        rom_stub_fn fn, const char *name);

/* Output capture (ets_printf / ets_write_char go here) */
int  rom_stubs_output_count(const esp32_rom_stubs_t *stubs);
const char *rom_stubs_output_buf(const esp32_rom_stubs_t *stubs);
void rom_stubs_output_clear(esp32_rom_stubs_t *stubs);

#endif /* ROM_STUBS_H */
