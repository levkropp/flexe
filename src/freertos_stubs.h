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

/* Consume (get and clear) the deferred task — one-shot, returns 0 on second call.
 * If multiple tasks were registered, starts the cooperative scheduler and returns 0. */
uint32_t freertos_stubs_consume_deferred_task(freertos_stubs_t *frt, uint32_t *param_out);

/* Returns true if the cooperative scheduler is running (multi-task mode) */
bool freertos_stubs_scheduler_active(const freertos_stubs_t *frt);

/* Start the cooperative scheduler (called on self-loop detection) */
void freertos_stubs_start_scheduler(freertos_stubs_t *frt);

/* Check if current task's timeslice expired; if so, preempt.
 * Call from main run loop after each xtensa_run() batch. */
bool freertos_stubs_check_preempt(freertos_stubs_t *frt);

#endif /* FREERTOS_STUBS_H */
