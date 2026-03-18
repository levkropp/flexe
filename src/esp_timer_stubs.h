#ifndef ESP_TIMER_STUBS_H
#define ESP_TIMER_STUBS_H

#include "xtensa.h"
#include "elf_symbols.h"

typedef struct esp_timer_stubs esp_timer_stubs_t;

esp_timer_stubs_t *esp_timer_stubs_create(xtensa_cpu_t *cpu);
void esp_timer_stubs_destroy(esp_timer_stubs_t *et);

/* Look up ELF symbols and register PC hooks */
int esp_timer_stubs_hook_symbols(esp_timer_stubs_t *et, const elf_symbols_t *syms);

/* Access timer count for testing */
int esp_timer_stubs_timer_count(const esp_timer_stubs_t *et);

/* Use virtual time for all timing (native FreeRTOS mode) */
void esp_timer_stubs_set_virtual_time(esp_timer_stubs_t *et, int enable);

#endif /* ESP_TIMER_STUBS_H */
