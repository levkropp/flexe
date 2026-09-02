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
#include "display_stubs.h"
#include "wifi_stubs.h"
#include "bt_stubs.h"
#include "peripherals.h"
#include "jit.h"
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
    int         native_freertos;    /* -N: let firmware run real FreeRTOS */
    int         disable_jit;        /* 1 = interpreter only; default is native JIT */

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

    /* Raw SPI display/touch capture pins (0 = use CYD 2432S028R defaults,
     * -1 = disable raw SPI sniffing) */
    int         spi_dc_pin;
    int         spi_display_cs_pin;
    int         spi_display_sck_pin;
    int         spi_touch_cs_pin;
    int         spi_touch_sck_pin;
    int         spi_touch_irq_pin;
    int         spi_sd_cs_pin;
    int         spi_sd_sck_pin;
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
display_stubs_t   *flexe_session_display(flexe_session_t *s);
wifi_stubs_t      *flexe_session_wifi(flexe_session_t *s);
bt_stubs_t        *flexe_session_bt(flexe_session_t *s);
int                flexe_session_is_native_freertos(const flexe_session_t *s);
jit_state_t       *flexe_session_jit(flexe_session_t *s);

/* Execute up to max_cycles on one core using the session's configured
 * engine.  Native JIT is the default on supported hosts; disable_jit falls
 * back to the interpreter.  Returns the guest instruction-count advance. */
int flexe_session_run_core(flexe_session_t *s, int core, int max_cycles);

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

/* Rebuild the machine as a software reset does. Flash -- and with it NVS and
 * SPIFFS -- is preserved, exactly as across a real reboot. */
void flexe_session_reset(flexe_session_t *s);

#endif /* FLEXE_SESSION_H */
