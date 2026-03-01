#ifndef DISPLAY_STUBS_H
#define DISPLAY_STUBS_H

#include "xtensa.h"
#include "elf_symbols.h"
#include <stdint.h>
#include <pthread.h>

typedef struct display_stubs display_stubs_t;

display_stubs_t *display_stubs_create(xtensa_cpu_t *cpu);
void display_stubs_destroy(display_stubs_t *ds);

/* Look up ELF symbols and register PC hooks for display functions */
int display_stubs_hook_symbols(display_stubs_t *ds, const elf_symbols_t *syms);

/* Hook TFT_eSPI C++ methods for framebuffer rendering (NerdMiner etc.) */
int display_stubs_hook_tft_espi(display_stubs_t *ds, const elf_symbols_t *syms);

/* Hook TFT_eSprite C++ methods for off-screen sprite rendering */
int display_stubs_hook_tft_esprite(display_stubs_t *ds, const elf_symbols_t *syms);

/* Hook OpenFontRender methods for FreeType text rendering */
int display_stubs_hook_ofr(display_stubs_t *ds, const elf_symbols_t *syms);

/* Set the framebuffer to render into (owned by cyd-emulator) */
void display_stubs_set_framebuf(display_stubs_t *ds, uint16_t *fb,
                                 pthread_mutex_t *mtx, int w, int h);

/* Query current display state (rotation-aware) */
uint8_t display_stubs_get_rotation(const display_stubs_t *ds);
int display_stubs_get_width(const display_stubs_t *ds);
int display_stubs_get_height(const display_stubs_t *ds);

#endif /* DISPLAY_STUBS_H */
