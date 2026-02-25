#ifndef FREERTOS_STUBS_H
#define FREERTOS_STUBS_H

#include "xtensa.h"
#include "elf_symbols.h"

typedef struct freertos_stubs freertos_stubs_t;

freertos_stubs_t *freertos_stubs_create(xtensa_cpu_t *cpu);
void freertos_stubs_destroy(freertos_stubs_t *frt);

/* Look up ELF symbols and register PC hooks for FreeRTOS functions */
int freertos_stubs_hook_symbols(freertos_stubs_t *frt, const elf_symbols_t *syms);

/* Access the bump allocator pointer (for testing) */
uint32_t freertos_stubs_bump_ptr(const freertos_stubs_t *frt);

/* Get deferred task info (saved by xTaskCreate/xTaskCreatePinnedToCore) */
uint32_t freertos_stubs_deferred_task(const freertos_stubs_t *frt, uint32_t *param_out);

/* Consume (get and clear) the deferred task — one-shot, returns 0 on second call */
uint32_t freertos_stubs_consume_deferred_task(freertos_stubs_t *frt, uint32_t *param_out);

#endif /* FREERTOS_STUBS_H */
