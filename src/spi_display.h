#ifndef SPI_DISPLAY_H
#define SPI_DISPLAY_H

#include <stdint.h>
#include <pthread.h>
#include "peripherals.h"

/* Raw GP-SPI display/touch capture for symbol-less firmware (Marauder,
 * NerdMiner). Display libraries for the CYD (TFT_eSPI, Adafruit_ILI9341,
 * XPT2046_Touchscreen) drive the panel through SPI2/SPI3 register writes
 * with CS and D/C as plain GPIOs. We emulate enough of the GP-SPI register
 * file for transactions to complete, capture the byte stream attributed by
 * CS/D/C, interpret ILI9341 commands into the emulator framebuffer, and
 * answer XPT2046 touch reads. */

typedef struct {
    int      dc_pin;          /* ILI9341 D/C GPIO (default 2 on 2432S028R) */
    int      display_cs_pin;  /* ILI9341 CS GPIO (default 15) */
    int      display_sck_pin; /* ILI9341 SCLK GPIO (default 14) */
    int      touch_cs_pin;    /* XPT2046 CS GPIO (default 33) */
    int      touch_sck_pin;   /* XPT2046 SCLK GPIO (default 25) */
    int      sd_cs_pin;       /* SD card CS GPIO (default 5) */
    int      sd_sck_pin;      /* SD card SCLK GPIO (default 18) */
    const char *sdcard_path;  /* SD card backing image (NULL = zeros) */

    /* Framebuffer to render into (NULL = capture but don't render) */
    uint16_t *framebuf;
    pthread_mutex_t *framebuf_mtx;
    int      fb_w, fb_h;

    /* Touch input (NULL = always report "no touch") */
    int     (*touch_fn)(int *x, int *y, void *ctx);
    void    *touch_ctx;
} spi_display_config_t;

/* Register MMIO handlers for SPI2 (0x3FF64000) and SPI3 (0x3FF65000).
 * Called from flexe_session_create after periph_create. */
void periph_enable_spi_display(esp32_periph_t *p, const spi_display_config_t *cfg);

/* Release raw-SPI backing resources owned by a peripheral instance. */
void periph_disable_spi_display(esp32_periph_t *p);

/* Observe a transaction addressed to no modelled device -- that is, one whose
 * chip select is none of display/touch/SD. Lets a test harness stand in as an
 * arbitrary SPI slave: it sees the MOSI bytes and fills the MISO buffer.
 * Without this the only way to exercise the SPI controller is through a
 * device model, which fixes both the protocol and the pins. */
typedef void (*spi_probe_fn)(const uint8_t *mosi, size_t mosi_len,
                             uint8_t *miso, size_t miso_len, void *ctx);
void periph_spi_attach_probe(esp32_periph_t *p, spi_probe_fn fn, void *ctx);

#endif /* SPI_DISPLAY_H */
