/*
 * flexe_session.h — Shared ESP32 emulator session (init, run, cleanup)
 *
 * Encapsulates all stub module lifecycle, dual-core management, and
 * cycle synchronization.  Both the standalone CLI (main.c) and the
 * GUI wrapper (emu_flexe.c) use this as the single source of truth.
 */

#ifndef FLEXE_SESSION_H
#define FLEXE_SESSION_H

#include "xtensa.h"
#include "elf_symbols.h"
#include "rom_stubs.h"
#include "freertos_stubs.h"
#include "peripherals.h"
#include <stdint.h>
#include <pthread.h>

typedef struct flexe_session flexe_session_t;

typedef struct {
    /* Required */
    const char *bin_path;           /* Firmware .bin file */

    /* Optional — ELF / SD card */
    const char *elf_path;           /* ELF symbol file (NULL = no symbols) */
    const char *sdcard_path;        /* SD card backing image (NULL = none) */
    uint64_t    sdcard_size;        /* SD card size override (0 = auto) */

    /* CPU configuration */
    uint32_t    entry_override;     /* Override entry point (0 = use .bin) */
    uint32_t    initial_sp;         /* Override initial SP (0 = 0x3FFE0000) */
    int         single_core;        /* 1 = no APP_CPU */
    int         window_trace;       /* Enable window spill/fill trace */
    int         spill_verify;       /* Enable spill/fill verification */

    /* UART output callback (NULL = no UART output) */
    void      (*uart_cb)(void *ctx, uint8_t byte);
    void       *uart_ctx;

    /* Display framebuffer (NULL = headless / no rendering) */
    uint16_t   *framebuf;
    pthread_mutex_t *framebuf_mutex;
    int         framebuf_w;
    int         framebuf_h;

    /* Touch input callback (NULL = no touch) */
    int       (*touch_fn)(int *x, int *y, void *ctx);
    void       *touch_ctx;
} flexe_session_config_t;

/* Create a fully-initialized emulator session.
 * Loads firmware, creates all stub modules, hooks symbols.
 * Returns NULL on failure (errors printed to stderr). */
flexe_session_t *flexe_session_create(const flexe_session_config_t *cfg);

/* Destroy session and free all resources. */
void flexe_session_destroy(flexe_session_t *s);

/* Accessors — pointers are valid for session lifetime. */
xtensa_cpu_t      *flexe_session_cpu(flexe_session_t *s, int core);
xtensa_mem_t      *flexe_session_mem(flexe_session_t *s);
const elf_symbols_t *flexe_session_syms(const flexe_session_t *s);
esp32_periph_t    *flexe_session_periph(flexe_session_t *s);
esp32_rom_stubs_t *flexe_session_rom(flexe_session_t *s);
freertos_stubs_t  *flexe_session_frt(flexe_session_t *s);

/* Post-batch hook: call after each core 0 batch.
 * - Checks preempt on core 0
 * - Detects core 1 start condition
 * - Runs core 1 batch (batch_size instructions)
 * - Syncs cycle counts between cores */
void flexe_session_post_batch(flexe_session_t *s, int batch_size);

/* Configure optional callbacks (call after create, before running). */
void flexe_session_set_rom_log_cb(flexe_session_t *s, rom_log_fn fn, void *ctx);
void flexe_session_set_event_log(flexe_session_t *s, int enable);
void flexe_session_set_freertos_event_fn(flexe_session_t *s,
        freertos_event_fn fn, void *ctx);

#endif /* FLEXE_SESSION_H */
