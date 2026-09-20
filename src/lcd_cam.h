/* Target-described ESP32-S3-generation LCD_CAM display/camera controller. */
#ifndef FLEXE_LCD_CAM_H
#define FLEXE_LCD_CAM_H

#include "gdma.h"
#include "memory.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef struct flexe_lcd_cam flexe_lcd_cam_t;

/* One chunk observed on the external i80 bus. `data` is in wire order after
 * the controller's bit/byte-order controls and remains valid only during the
 * callback. A command-only transaction is delivered once with length zero.
 * Multi-descriptor transfers set first/last around the complete transaction. */
typedef struct {
    uint32_t command;
    uint8_t command_cycles;
    uint8_t bus_width;
    bool command_enabled;
    bool dummy_enabled;
    bool data_enabled;
    bool first;
    bool last;
} flexe_lcd_cam_i80_transfer_t;

typedef void (*flexe_lcd_cam_i80_tx_fn)(
    void *ctx, const flexe_lcd_cam_i80_transfer_t *transfer,
    const uint8_t *data, size_t length);
typedef void (*flexe_lcd_cam_irq_fn)(void *ctx, bool level);

flexe_lcd_cam_t *flexe_lcd_cam_create(
    xtensa_mem_t *mem, flexe_gdma_t *gdma,
    mmio_read_fn fallback_read, mmio_write_fn fallback_write,
    void *fallback_ctx, flexe_lcd_cam_irq_fn irq_changed, void *irq_ctx);
void flexe_lcd_cam_destroy(flexe_lcd_cam_t *lcd_cam);
void flexe_lcd_cam_set_system_state(flexe_lcd_cam_t *lcd_cam,
                                    bool clock_enabled,
                                    bool reset_asserted);
int flexe_lcd_cam_set_i80_callback(flexe_lcd_cam_t *lcd_cam,
                                   flexe_lcd_cam_i80_tx_fn callback,
                                   void *ctx);

/* Feed one complete camera DMA quantum and signal frame boundaries. RX
 * injection consumes at most one descriptor (and never a partial descriptor)
 * so the host can let the guest service each GDMA EOF interrupt before
 * supplying more data. VSYNC raises the shared LCD_CAM interrupt independently
 * of GDMA, as the physical parallel sensor does. */
size_t flexe_lcd_cam_camera_rx_inject(flexe_lcd_cam_t *lcd_cam,
                                      const uint8_t *data, size_t length);
int flexe_lcd_cam_camera_vsync(flexe_lcd_cam_t *lcd_cam);

#endif /* FLEXE_LCD_CAM_H */
