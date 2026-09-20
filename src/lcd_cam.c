#include "lcd_cam.h"
#include "target.h"

#include <stdlib.h>
#include <string.h>

#define LCD_CAM_LCD_CLOCK_OFF          0x000u
#define LCD_CAM_CAM_CTRL_OFF           0x004u
#define LCD_CAM_CAM_CTRL1_OFF          0x008u
#define LCD_CAM_CAM_RGB_YUV_OFF        0x00Cu
#define LCD_CAM_LCD_RGB_YUV_OFF        0x010u
#define LCD_CAM_LCD_USER_OFF           0x014u
#define LCD_CAM_LCD_MISC_OFF           0x018u
#define LCD_CAM_LCD_CTRL_OFF           0x01Cu
#define LCD_CAM_LCD_CTRL1_OFF          0x020u
#define LCD_CAM_LCD_CTRL2_OFF          0x024u
#define LCD_CAM_LCD_CMD_VAL_OFF        0x028u
#define LCD_CAM_LCD_DLY_MODE_OFF       0x030u
#define LCD_CAM_LCD_DATA_DOUT_MODE_OFF 0x038u
#define LCD_CAM_INT_ENA_OFF            0x064u
#define LCD_CAM_INT_RAW_OFF            0x068u
#define LCD_CAM_INT_ST_OFF             0x06Cu
#define LCD_CAM_INT_CLR_OFF            0x070u
#define LCD_CAM_DATE_OFF               0x0FCu

#define LCD_CAM_INT_VALID_MASK         0x0000000Fu
#define LCD_CAM_INT_LCD_TRANS_DONE     (1u << 1)
#define LCD_CAM_INT_CAM_VSYNC          (1u << 2)

#define LCD_CAM_CAM_UPDATE             (1u << 4)
#define LCD_CAM_CAM_BYTE_ORDER         (1u << 5)
#define LCD_CAM_CAM_BIT_ORDER          (1u << 6)
#define LCD_CAM_CAM_START              (1u << 29)
#define LCD_CAM_CAM_RESET              (1u << 30)
#define LCD_CAM_CAM_AFIFO_RESET        (1u << 31)
#define LCD_CAM_CAM_2BYTE_ENABLE       (1u << 24)
#define LCD_CAM_CAM_CONVERTER_ENABLE   (1u << 31)

#define LCD_CAM_LCD_ALWAYS_OUT         (1u << 13)
#define LCD_CAM_LCD_8BITS_ORDER        (1u << 19)
#define LCD_CAM_LCD_UPDATE             (1u << 20)
#define LCD_CAM_LCD_BIT_ORDER          (1u << 21)
#define LCD_CAM_LCD_BYTE_ORDER         (1u << 22)
#define LCD_CAM_LCD_2BYTE_ENABLE       (1u << 23)
#define LCD_CAM_LCD_DOUT               (1u << 24)
#define LCD_CAM_LCD_DUMMY              (1u << 25)
#define LCD_CAM_LCD_CMD                (1u << 26)
#define LCD_CAM_LCD_START              (1u << 27)
#define LCD_CAM_LCD_RESET              (1u << 28)
#define LCD_CAM_LCD_CMD_2_CYCLE        (1u << 31)
#define LCD_CAM_LCD_AFIFO_RESET        (1u << 27)
#define LCD_CAM_LCD_RGB_MODE           (1u << 31)
#define LCD_CAM_LCD_CONVERTER_ENABLE   (1u << 31)

#define LCD_CAM_DMA_DESCRIPTOR_MAX     4095u
#define LCD_CAM_DMA_CHAIN_MAX          4096u

struct flexe_lcd_cam {
    xtensa_mem_t *mem;
    flexe_gdma_t *gdma;
    const flexe_lcd_cam_desc_t *desc;
    mmio_read_fn fallback_read;
    mmio_write_fn fallback_write;
    void *fallback_ctx;
    flexe_lcd_cam_irq_fn irq_changed;
    void *irq_ctx;
    flexe_lcd_cam_i80_tx_fn tx_callback;
    void *tx_ctx;
    uint32_t regs[0x100u / sizeof(uint32_t)];
    size_t camera_bytes_until_eof;
    bool clock_enabled;
    bool reset_asserted;
    bool irq_level;
};

static bool lcd_cam_register_defined(uint32_t offset)
{
    switch (offset) {
    case LCD_CAM_LCD_CLOCK_OFF:
    case LCD_CAM_CAM_CTRL_OFF:
    case LCD_CAM_CAM_CTRL1_OFF:
    case LCD_CAM_CAM_RGB_YUV_OFF:
    case LCD_CAM_LCD_RGB_YUV_OFF:
    case LCD_CAM_LCD_USER_OFF:
    case LCD_CAM_LCD_MISC_OFF:
    case LCD_CAM_LCD_CTRL_OFF:
    case LCD_CAM_LCD_CTRL1_OFF:
    case LCD_CAM_LCD_CTRL2_OFF:
    case LCD_CAM_LCD_CMD_VAL_OFF:
    case LCD_CAM_LCD_DLY_MODE_OFF:
    case LCD_CAM_LCD_DATA_DOUT_MODE_OFF:
    case LCD_CAM_INT_ENA_OFF:
    case LCD_CAM_INT_RAW_OFF:
    case LCD_CAM_INT_ST_OFF:
    case LCD_CAM_INT_CLR_OFF:
    case LCD_CAM_DATE_OFF:
        return true;
    default:
        return false;
    }
}

static void lcd_cam_update_irq(flexe_lcd_cam_t *lcd_cam)
{
    bool level = (lcd_cam->regs[LCD_CAM_INT_RAW_OFF / 4u] &
                  lcd_cam->regs[LCD_CAM_INT_ENA_OFF / 4u] &
                  LCD_CAM_INT_VALID_MASK) != 0u;
    if (level == lcd_cam->irq_level) return;
    lcd_cam->irq_level = level;
    if (lcd_cam->irq_changed)
        lcd_cam->irq_changed(lcd_cam->irq_ctx, level);
}

static void lcd_cam_reset(flexe_lcd_cam_t *lcd_cam)
{
    flexe_lcd_cam_i80_tx_fn callback = lcd_cam->tx_callback;
    void *callback_ctx = lcd_cam->tx_ctx;
    bool clock_enabled = lcd_cam->clock_enabled;
    bool reset_asserted = lcd_cam->reset_asserted;
    bool old_irq = lcd_cam->irq_level;
    memset(lcd_cam->regs, 0, sizeof(lcd_cam->regs));
    lcd_cam->regs[LCD_CAM_LCD_MISC_OFF / 4u] = 17u << 1u;
    lcd_cam->regs[LCD_CAM_DATE_OFF / 4u] = lcd_cam->desc->date_reset;
    lcd_cam->camera_bytes_until_eof = 0u;
    lcd_cam->tx_callback = callback;
    lcd_cam->tx_ctx = callback_ctx;
    lcd_cam->clock_enabled = clock_enabled;
    lcd_cam->reset_asserted = reset_asserted;
    lcd_cam->irq_level = false;
    if (old_irq && lcd_cam->irq_changed)
        lcd_cam->irq_changed(lcd_cam->irq_ctx, false);
}

static uint8_t lcd_cam_reverse8(uint8_t value)
{
    value = (uint8_t)((value >> 4u) | (value << 4u));
    value = (uint8_t)(((value & 0xCCu) >> 2u) |
                      ((value & 0x33u) << 2u));
    return (uint8_t)(((value & 0xAAu) >> 1u) |
                     ((value & 0x55u) << 1u));
}

static void lcd_cam_order_i80_data(uint8_t *data, size_t length,
                                    uint32_t user)
{
    bool wide = (user & LCD_CAM_LCD_2BYTE_ENABLE) != 0u;
    if (user & LCD_CAM_LCD_BIT_ORDER) {
        if (wide) {
            for (size_t index = 0u; index + 1u < length; index += 2u) {
                uint8_t low = lcd_cam_reverse8(data[index]);
                data[index] = lcd_cam_reverse8(data[index + 1u]);
                data[index + 1u] = low;
            }
            if (length & 1u)
                data[length - 1u] = lcd_cam_reverse8(data[length - 1u]);
        } else {
            for (size_t index = 0u; index < length; index++)
                data[index] = lcd_cam_reverse8(data[index]);
        }
    }
    /* Byte-order inversion is defined only for the 16-bit DMA stride. The
     * post-converter AB->BA swizzle applies to either bus width. With the
     * converter bypassed, the two independent swaps cancel each other. */
    bool swap_bytes =
        (wide && (user & LCD_CAM_LCD_BYTE_ORDER) != 0u) !=
        ((user & LCD_CAM_LCD_8BITS_ORDER) != 0u);
    if (swap_bytes) {
        for (size_t index = 0u; index + 1u < length; index += 2u) {
            uint8_t swap = data[index];
            data[index] = data[index + 1u];
            data[index + 1u] = swap;
        }
    }
}

static void lcd_cam_order_camera_data(uint8_t *data, size_t length,
                                      uint32_t control,
                                      uint32_t control1)
{
    if (control & LCD_CAM_CAM_BIT_ORDER) {
        for (size_t index = 0u; index < length; index++)
            data[index] = lcd_cam_reverse8(data[index]);
    }
    if ((control & LCD_CAM_CAM_BYTE_ORDER) != 0u &&
        (control1 & LCD_CAM_CAM_2BYTE_ENABLE) != 0u) {
        for (size_t index = 0u; index + 1u < length; index += 2u) {
            uint8_t swap = data[index];
            data[index] = data[index + 1u];
            data[index + 1u] = swap;
        }
    }
}

static flexe_lcd_cam_i80_transfer_t lcd_cam_transfer_info(
    const flexe_lcd_cam_t *lcd_cam, bool first, bool last)
{
    uint32_t user = lcd_cam->regs[LCD_CAM_LCD_USER_OFF / 4u];
    return (flexe_lcd_cam_i80_transfer_t){
        .command = lcd_cam->regs[LCD_CAM_LCD_CMD_VAL_OFF / 4u],
        .command_cycles = (user & LCD_CAM_LCD_CMD) ?
            (uint8_t)((user & LCD_CAM_LCD_CMD_2_CYCLE) ? 2u : 1u) : 0u,
        .bus_width = (user & LCD_CAM_LCD_2BYTE_ENABLE) ? 16u : 8u,
        .command_enabled = (user & LCD_CAM_LCD_CMD) != 0u,
        .dummy_enabled = (user & LCD_CAM_LCD_DUMMY) != 0u,
        .data_enabled = (user & LCD_CAM_LCD_DOUT) != 0u,
        .first = first,
        .last = last,
    };
}

static void lcd_cam_report_unsupported(flexe_lcd_cam_t *lcd_cam,
                                       uint32_t offset, uint32_t value)
{
    if (lcd_cam->fallback_write)
        lcd_cam->fallback_write(lcd_cam->fallback_ctx,
                                lcd_cam->desc->base + offset, value);
}

static bool lcd_cam_emit_descriptor_chain(flexe_lcd_cam_t *lcd_cam)
{
    uint8_t data[LCD_CAM_DMA_DESCRIPTOR_MAX];
    bool first = true;
    for (unsigned count = 0u; count < LCD_CAM_DMA_CHAIN_MAX; count++) {
        size_t length = 0u;
        if (!flexe_gdma_pending_length(lcd_cam->gdma,
                                       lcd_cam->desc->gdma_peripheral_id,
                                       false, &length) ||
            length == 0u || length > sizeof(data))
            return false;
        flexe_gdma_descriptor_t completed = {0};
        if (flexe_gdma_read_tx_descriptor(
                lcd_cam->gdma, lcd_cam->desc->gdma_peripheral_id,
                data, sizeof(data), &completed) != 0)
            return false;
        lcd_cam_order_i80_data(
            data, completed.length,
            lcd_cam->regs[LCD_CAM_LCD_USER_OFF / 4u]);
        if (lcd_cam->tx_callback) {
            flexe_lcd_cam_i80_transfer_t info = lcd_cam_transfer_info(
                lcd_cam, first, completed.chain_complete);
            lcd_cam->tx_callback(lcd_cam->tx_ctx, &info,
                                 data, completed.length);
        }
        first = false;
        if (completed.chain_complete) return true;
    }
    lcd_cam_report_unsupported(
        lcd_cam, LCD_CAM_LCD_USER_OFF,
        lcd_cam->regs[LCD_CAM_LCD_USER_OFF / 4u]);
    return false;
}

static void lcd_cam_start_i80(flexe_lcd_cam_t *lcd_cam)
{
    uint32_t *user = &lcd_cam->regs[LCD_CAM_LCD_USER_OFF / 4u];
    if (!lcd_cam->clock_enabled || lcd_cam->reset_asserted ||
        (*user & LCD_CAM_LCD_START) == 0u)
        return;

    /* Retaining the shared configuration words must not imply that camera,
     * RGB scanout, or the color converter is functionally implemented. */
    if ((lcd_cam->regs[LCD_CAM_LCD_CTRL_OFF / 4u] &
         LCD_CAM_LCD_RGB_MODE) != 0u ||
        (lcd_cam->regs[LCD_CAM_LCD_RGB_YUV_OFF / 4u] &
         LCD_CAM_LCD_CONVERTER_ENABLE) != 0u) {
        lcd_cam_report_unsupported(lcd_cam, LCD_CAM_LCD_USER_OFF, *user);
        return;
    }

    bool data_enabled = (*user & LCD_CAM_LCD_DOUT) != 0u;
    if (data_enabled) {
        /* The public S3 i80 driver uses DMA-controlled length. A programmed
         * finite DOUT cycle count can stop in the middle of a descriptor,
         * which the GDMA transport cannot yet preserve for a later start. */
        if ((*user & LCD_CAM_LCD_ALWAYS_OUT) == 0u) {
            lcd_cam_report_unsupported(lcd_cam, LCD_CAM_LCD_USER_OFF, *user);
            return;
        }
        if (!flexe_gdma_tx_active(lcd_cam->gdma,
                                  lcd_cam->desc->gdma_peripheral_id) ||
            !lcd_cam_emit_descriptor_chain(lcd_cam))
            return;
    } else if (lcd_cam->tx_callback) {
        flexe_lcd_cam_i80_transfer_t info =
            lcd_cam_transfer_info(lcd_cam, true, true);
        lcd_cam->tx_callback(lcd_cam->tx_ctx, &info, NULL, 0u);
    }

    *user &= ~LCD_CAM_LCD_START;
    lcd_cam->regs[LCD_CAM_INT_RAW_OFF / 4u] |=
        LCD_CAM_INT_LCD_TRANS_DONE;
    lcd_cam_update_irq(lcd_cam);
}

static uint32_t lcd_cam_read(void *ctx, uint32_t address)
{
    flexe_lcd_cam_t *lcd_cam = ctx;
    uint32_t offset = address - lcd_cam->desc->base;
    if ((offset & 3u) != 0u || !lcd_cam_register_defined(offset))
        return lcd_cam->fallback_read(lcd_cam->fallback_ctx, address);
    if (offset == LCD_CAM_INT_ST_OFF)
        return lcd_cam->regs[LCD_CAM_INT_RAW_OFF / 4u] &
               lcd_cam->regs[LCD_CAM_INT_ENA_OFF / 4u] &
               LCD_CAM_INT_VALID_MASK;
    if (offset == LCD_CAM_INT_CLR_OFF) return 0u;
    return lcd_cam->regs[offset / 4u];
}

static void lcd_cam_write(void *ctx, uint32_t address, uint32_t value)
{
    flexe_lcd_cam_t *lcd_cam = ctx;
    uint32_t offset = address - lcd_cam->desc->base;
    if ((offset & 3u) != 0u || !lcd_cam_register_defined(offset)) {
        lcd_cam->fallback_write(lcd_cam->fallback_ctx, address, value);
        return;
    }
    switch (offset) {
    case LCD_CAM_LCD_CLOCK_OFF:
    case LCD_CAM_LCD_CTRL_OFF:
    case LCD_CAM_LCD_CTRL1_OFF:
    case LCD_CAM_LCD_CMD_VAL_OFF:
    case LCD_CAM_LCD_DATA_DOUT_MODE_OFF:
        lcd_cam->regs[offset / 4u] = value;
        return;
    case LCD_CAM_CAM_CTRL_OFF:
        lcd_cam->regs[offset / 4u] = value & 0x7FFFFFFFu &
                                     ~LCD_CAM_CAM_UPDATE;
        return;
    case LCD_CAM_CAM_CTRL1_OFF:
        lcd_cam->regs[offset / 4u] = value &
            ~(LCD_CAM_CAM_RESET | LCD_CAM_CAM_AFIFO_RESET);
        if ((value & (LCD_CAM_CAM_RESET | LCD_CAM_CAM_AFIFO_RESET)) != 0u ||
            (value & LCD_CAM_CAM_START) == 0u)
            lcd_cam->camera_bytes_until_eof = 0u;
        return;
    case LCD_CAM_CAM_RGB_YUV_OFF:
        lcd_cam->regs[offset / 4u] = value & 0xFFE00000u;
        return;
    case LCD_CAM_LCD_RGB_YUV_OFF:
        lcd_cam->regs[offset / 4u] = value & 0xFFF00000u;
        return;
    case LCD_CAM_LCD_USER_OFF:
        lcd_cam->regs[offset / 4u] = value & 0xFFF83FFFu &
            ~(LCD_CAM_LCD_UPDATE | LCD_CAM_LCD_RESET);
        lcd_cam_start_i80(lcd_cam);
        return;
    case LCD_CAM_LCD_MISC_OFF:
        lcd_cam->regs[offset / 4u] = value & 0xFFFFFFFEu &
                                     ~LCD_CAM_LCD_AFIFO_RESET;
        return;
    case LCD_CAM_LCD_CTRL2_OFF:
        lcd_cam->regs[offset / 4u] = value & 0xFFFF03FFu;
        return;
    case LCD_CAM_LCD_DLY_MODE_OFF:
        lcd_cam->regs[offset / 4u] = value & 0x000000FFu;
        return;
    case LCD_CAM_INT_ENA_OFF:
        lcd_cam->regs[offset / 4u] = value & LCD_CAM_INT_VALID_MASK;
        lcd_cam_update_irq(lcd_cam);
        return;
    case LCD_CAM_INT_CLR_OFF:
        lcd_cam->regs[LCD_CAM_INT_RAW_OFF / 4u] &=
            ~(value & LCD_CAM_INT_VALID_MASK);
        lcd_cam_update_irq(lcd_cam);
        return;
    case LCD_CAM_DATE_OFF:
        lcd_cam->regs[offset / 4u] = value & 0x0FFFFFFFu;
        return;
    case LCD_CAM_INT_RAW_OFF:
    case LCD_CAM_INT_ST_OFF:
        return;
    default:
        return;
    }
}

static bool lcd_cam_geometry_valid(const flexe_target_desc_t *target,
                                   const flexe_gdma_t *gdma)
{
    if (!target || !gdma ||
        !(target->capabilities & FLEXE_TARGET_CAP_LCD_CAM_V1) ||
        !(target->capabilities & FLEXE_TARGET_CAP_GDMA_V1) ||
        !(target->capabilities & FLEXE_TARGET_CAP_INTERRUPT_MATRIX_V1) ||
        !(target->capabilities & FLEXE_TARGET_CAP_SYSTEM_CLOCK_V1))
        return false;
    const flexe_lcd_cam_desc_t *desc = &target->lcd_cam;
    return (desc->base & 0xFFFu) == 0u &&
           desc->register_size >= LCD_CAM_DATE_OFF + sizeof(uint32_t) &&
           (desc->register_size & 0xFFFu) == 0u &&
           desc->base >= target->peripheral_start &&
           desc->base < target->peripheral_end &&
           desc->register_size <= target->peripheral_end - desc->base &&
           desc->date_reset != 0u &&
           desc->interrupt_source < target->interrupt_matrix.source_count &&
           desc->gdma_peripheral_id != FLEXE_TARGET_GDMA_PERIPHERAL_NONE &&
           desc->data_output_count > 0u &&
           desc->data_output_count <= FLEXE_TARGET_LCD_CAM_DATA_MAX;
}

flexe_lcd_cam_t *flexe_lcd_cam_create(
    xtensa_mem_t *mem, flexe_gdma_t *gdma,
    mmio_read_fn fallback_read, mmio_write_fn fallback_write,
    void *fallback_ctx, flexe_lcd_cam_irq_fn irq_changed, void *irq_ctx)
{
    const flexe_target_desc_t *target = mem ? mem_target(mem) : NULL;
    if (!mem || !fallback_read || !fallback_write ||
        !lcd_cam_geometry_valid(target, gdma))
        return NULL;
    flexe_lcd_cam_t *lcd_cam = calloc(1u, sizeof(*lcd_cam));
    if (!lcd_cam) return NULL;
    lcd_cam->mem = mem;
    lcd_cam->gdma = gdma;
    lcd_cam->desc = &target->lcd_cam;
    lcd_cam->fallback_read = fallback_read;
    lcd_cam->fallback_write = fallback_write;
    lcd_cam->fallback_ctx = fallback_ctx;
    lcd_cam->irq_changed = irq_changed;
    lcd_cam->irq_ctx = irq_ctx;
    lcd_cam_reset(lcd_cam);
    if (mem_register_mmio_range(mem, lcd_cam->desc->base,
                                lcd_cam->desc->register_size,
                                lcd_cam_read, lcd_cam_write, lcd_cam) != 0) {
        free(lcd_cam);
        return NULL;
    }
    return lcd_cam;
}

void flexe_lcd_cam_destroy(flexe_lcd_cam_t *lcd_cam)
{
    if (!lcd_cam) return;
    if (lcd_cam->irq_level && lcd_cam->irq_changed)
        lcd_cam->irq_changed(lcd_cam->irq_ctx, false);
    (void)mem_register_mmio_range(
        lcd_cam->mem, lcd_cam->desc->base, lcd_cam->desc->register_size,
        lcd_cam->fallback_read, lcd_cam->fallback_write,
        lcd_cam->fallback_ctx);
    free(lcd_cam);
}

void flexe_lcd_cam_set_system_state(flexe_lcd_cam_t *lcd_cam,
                                    bool clock_enabled,
                                    bool reset_asserted)
{
    if (!lcd_cam) return;
    bool reset_edge = reset_asserted && !lcd_cam->reset_asserted;
    lcd_cam->clock_enabled = clock_enabled;
    lcd_cam->reset_asserted = reset_asserted;
    if (reset_edge) lcd_cam_reset(lcd_cam);
    if (!reset_asserted && clock_enabled) lcd_cam_start_i80(lcd_cam);
}

int flexe_lcd_cam_set_i80_callback(flexe_lcd_cam_t *lcd_cam,
                                   flexe_lcd_cam_i80_tx_fn callback,
                                   void *ctx)
{
    if (!lcd_cam) return -1;
    lcd_cam->tx_callback = callback;
    lcd_cam->tx_ctx = callback ? ctx : NULL;
    return 0;
}

size_t flexe_lcd_cam_camera_rx_inject(flexe_lcd_cam_t *lcd_cam,
                                      const uint8_t *data, size_t length)
{
    if (!lcd_cam || (!data && length != 0u) || length == 0u ||
        !lcd_cam->clock_enabled || lcd_cam->reset_asserted)
        return 0u;
    uint32_t control = lcd_cam->regs[LCD_CAM_CAM_CTRL_OFF / 4u];
    uint32_t control1 = lcd_cam->regs[LCD_CAM_CAM_CTRL1_OFF / 4u];
    if ((control1 & LCD_CAM_CAM_START) == 0u)
        return 0u;
    if ((lcd_cam->regs[LCD_CAM_CAM_RGB_YUV_OFF / 4u] &
         LCD_CAM_CAM_CONVERTER_ENABLE) != 0u) {
        lcd_cam_report_unsupported(lcd_cam, LCD_CAM_CAM_CTRL1_OFF, control1);
        return 0u;
    }
    size_t capacity = 0u;
    if (!flexe_gdma_pending_length(lcd_cam->gdma,
                                   lcd_cam->desc->gdma_peripheral_id,
                                   true, &capacity))
        return 0u;
    size_t bytes_until_eof = lcd_cam->camera_bytes_until_eof ?
        lcd_cam->camera_bytes_until_eof : (control1 & 0xFFFFu) + 1u;
    size_t required = capacity < bytes_until_eof ?
                      capacity : bytes_until_eof;
    if (required == 0u || required > LCD_CAM_DMA_DESCRIPTOR_MAX ||
        length < required)
        return 0u;
    if ((control1 & LCD_CAM_CAM_2BYTE_ENABLE) != 0u &&
        (required & 1u) != 0u) {
        lcd_cam_report_unsupported(lcd_cam, LCD_CAM_CAM_CTRL1_OFF, control1);
        return 0u;
    }
    uint8_t ordered[LCD_CAM_DMA_DESCRIPTOR_MAX];
    memcpy(ordered, data, required);
    lcd_cam_order_camera_data(ordered, required, control, control1);
    bool eof = required == bytes_until_eof;
    if (flexe_gdma_write_rx_descriptor_eof(
            lcd_cam->gdma, lcd_cam->desc->gdma_peripheral_id,
            ordered, required, eof, NULL) != 0)
        return 0u;
    lcd_cam->camera_bytes_until_eof = eof ? 0u :
        bytes_until_eof - required;
    return required;
}

int flexe_lcd_cam_camera_vsync(flexe_lcd_cam_t *lcd_cam)
{
    if (!lcd_cam || !lcd_cam->clock_enabled || lcd_cam->reset_asserted ||
        (lcd_cam->regs[LCD_CAM_CAM_CTRL1_OFF / 4u] &
         LCD_CAM_CAM_START) == 0u)
        return 0;
    lcd_cam->camera_bytes_until_eof = 0u;
    lcd_cam->regs[LCD_CAM_INT_RAW_OFF / 4u] |= LCD_CAM_INT_CAM_VSYNC;
    lcd_cam_update_irq(lcd_cam);
    return 1;
}
