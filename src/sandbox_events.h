/*
 * sandbox_events.h — unified peripheral-event bus for the litegraph
 * hardware sandbox frontend.
 *
 * flexe already emits individual callbacks for UART TX. The sandbox
 * UI additionally wants to observe GPIO output level changes (for LEDs,
 * buttons, logic analyzers), SPI bus transactions (for display panels),
 * and I²C transactions (for sensors / expanders). Rather than hard-wire
 * each subsystem to the outside world, peripherals fire into a single
 * per-session event sink — the CLI (or a future in-process websocket)
 * registers one callback and receives the whole stream.
 *
 * Events are plain C structs; serialisation (JSON for websockets,
 * Protobuf for a future binary channel, etc.) is the consumer's job.
 */
#ifndef SANDBOX_EVENTS_H
#define SANDBOX_EVENTS_H

#include <stdint.h>

typedef enum {
    SBX_EV_GPIO_OUT   = 1,  /* GPIO output level transition */
    SBX_EV_UART_TX    = 2,  /* byte written to a UART TX FIFO */
    SBX_EV_SPI_XFER   = 3,  /* a SPI half-duplex transfer completed */
    SBX_EV_I2C_XFER   = 4,  /* an I²C transaction completed */
    SBX_EV_LCD_PIXELS = 5,  /* framebuffer bytes pushed to an LCD panel */
} sbx_event_kind_t;

typedef struct {
    sbx_event_kind_t kind;
    uint64_t         cycle;     /* cpu->cycle_count at emission */
    union {
        struct { uint8_t pin; uint8_t level; } gpio_out;
        struct { uint8_t uart_num; uint8_t byte; } uart_tx;
        struct { uint8_t host; uint16_t len; const uint8_t *data; } spi_xfer;
        struct {
            uint8_t port;
            uint8_t addr;
            uint8_t read;
            uint16_t len;
            const uint8_t *data;
        } i2c_xfer;
        struct { uint32_t x, y, w, h; uint16_t bpp; const uint8_t *pixels; } lcd_pixels;
    };
} sbx_event_t;

typedef void (*sbx_event_fn)(const sbx_event_t *ev, void *ctx);

/* Bus is a singleton per-process; the CLI registers one consumer and
 * peripherals route all their events into it. Passing NULL clears. */
void sbx_events_set_sink(sbx_event_fn fn, void *ctx);

/* Called by peripheral handlers to publish an event. No-op if no sink. */
void sbx_events_emit(const sbx_event_t *ev);

#endif /* SANDBOX_EVENTS_H */
