#ifndef SPI_DISPLAY_H
#define SPI_DISPLAY_H

#include <stdint.h>
#include <pthread.h>
#include "peripherals.h"

/* Target-described GP-SPI controller plus optional board devices. The shared
 * controller handles native ESP32 and S2/S3-generation register layouts;
 * CYD display/touch/SD interpretation is only one pluggable board-facing use
 * of the bytes that cross it. */

typedef struct flexe_gp_spi flexe_gp_spi_t;

typedef struct {
    int      dc_pin;          /* ILI9341 D/C GPIO (default 2 on 2432S028R) */
    int      display_cs_pin;  /* ILI9341 CS GPIO (default 15) */
    int      display_sck_pin; /* ILI9341 SCLK GPIO (default 14) */
    int      touch_cs_pin;    /* XPT2046 CS GPIO (default 33) */
    int      touch_sck_pin;   /* XPT2046 SCLK GPIO (default 25) */
    int      touch_mosi_pin;  /* XPT2046 MOSI GPIO (default 32) */
    int      touch_miso_pin;  /* XPT2046 MISO GPIO (default 39) */
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

/* Replace the optional board-facing configuration without changing the SoC
 * controller instances registered from the selected target descriptor. */
void periph_enable_spi_display(esp32_periph_t *p, const spi_display_config_t *cfg);

/* Release raw-SPI backing resources owned by a peripheral instance. */
void periph_disable_spi_display(esp32_periph_t *p);

/* Feed GPIO output edges to the software-SPI XPT2046 model. Some CYD
 * libraries drive the touch controller by bit-banging GPIO rather than using
 * either GP-SPI host, so register-level SPI emulation alone cannot serve
 * their MISO reads. Called by the GPIO peripheral after its output latch has
 * changed. */
void spi_display_gpio_changed(esp32_periph_t *p, int pin, int level);

/* Observe a transaction addressed to no modelled device -- that is, one whose
 * chip select is none of display/touch/SD. Lets a test harness stand in as an
 * arbitrary SPI slave: it sees the MOSI bytes and fills the MISO buffer.
 * Without this the only way to exercise the SPI controller is through a
 * device model, which fixes both the protocol and the pins. */
typedef void (*spi_probe_fn)(const uint8_t *mosi, size_t mosi_len,
                             uint8_t *miso, size_t miso_len, void *ctx);
void periph_spi_attach_probe(esp32_periph_t *p, spi_probe_fn fn, void *ctx);

/* Attach a target to one GP-SPI host and its board-level wires. The transfer
 * callback receives each controller transaction clocked while cs_pin is
 * asserted and may fill the MISO phase in place. A software-controlled CS can
 * remain asserted across several transactions, so the optional select
 * callback reports its GPIO edges and lets a device preserve framing. host is
 * 2 (HSPI) or 3 (VSPI), and sck_pin rejects traffic routed through the other
 * controller. Passing NULL as fn detaches the matching host/CS target. */
typedef void (*periph_spi_device_fn)(void *ctx, int host,
                                     const uint8_t *mosi, size_t mosi_len,
                                     uint8_t *miso, size_t miso_len);
typedef void (*periph_spi_select_fn)(void *ctx, int selected);
int periph_spi_attach_device(esp32_periph_t *p, int host, int cs_pin,
                             int sck_pin, periph_spi_device_fn fn, void *ctx);
int periph_spi_attach_device_ex(esp32_periph_t *p, int host, int cs_pin,
                                int sck_pin, periph_spi_device_fn fn,
                                periph_spi_select_fn select_fn, void *ctx);

/* Total bytes fed to the panel since start. A firmware whose display task has
 * stopped shows up as this standing still while the CPU stays busy, which a
 * framebuffer checksum alone cannot distinguish from a UI redrawing the same
 * picture. */
uint64_t spi_display_bytes_fed(void);

/* Completed XPT2046 commands received over GPIO software SPI. */
uint64_t spi_touch_bitbang_commands(void);

/* Machine-construction API. These are called by peripherals.c so a GP-SPI
 * capability always creates its controllers, even for headless sessions. */
flexe_gp_spi_t *flexe_gp_spi_create(
    esp32_periph_t *p, mmio_read_fn fallback_read,
    mmio_write_fn fallback_write, void *fallback_ctx);
void flexe_gp_spi_destroy(flexe_gp_spi_t *spi);
void flexe_gp_spi_set_system_state(flexe_gp_spi_t *spi, unsigned instance,
                                   bool clock_enabled,
                                   bool reset_asserted);

#endif /* SPI_DISPLAY_H */
