#ifndef TOUCH_STUBS_H
#define TOUCH_STUBS_H

#include "xtensa.h"
#include "elf_symbols.h"

typedef struct touch_stubs touch_stubs_t;

/* Callback to read touch state from the host (cyd-emulator).
 * Returns 1 if pressed, 0 if not. Writes x/y coords. */
typedef int (*touch_state_fn)(int *x, int *y, void *ctx);

touch_stubs_t *touch_stubs_create(xtensa_cpu_t *cpu);
void touch_stubs_destroy(touch_stubs_t *ts);

/* Look up ELF symbols and register PC hooks for touch functions */
int touch_stubs_hook_symbols(touch_stubs_t *ts, const elf_symbols_t *syms);

/* Set the callback for reading touch state */
void touch_stubs_set_state_fn(touch_stubs_t *ts, touch_state_fn fn, void *ctx);

#endif /* TOUCH_STUBS_H */
