/*
 * sandbox_input.h - host-to-guest peripheral events.
 *
 * The sandbox transport is NDJSON, but parsing it is deliberately separate
 * from the CLI run loop.  Other frontends can therefore share the same
 * bounded event vocabulary without depending on main.c or a particular
 * firmware image.
 */
#ifndef SANDBOX_INPUT_H
#define SANDBOX_INPUT_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define SBX_INPUT_UART_MAX 2048u

typedef enum {
    SBX_INPUT_NONE = 0,
    SBX_INPUT_GPIO,
    SBX_INPUT_TOUCH,
    SBX_INPUT_ADC,
    SBX_INPUT_UART,
    SBX_INPUT_UART_BREAK,
} sbx_input_kind_t;

typedef struct {
    sbx_input_kind_t kind;
    union {
        struct { int pin; int level; } gpio;
        struct { int x; int y; int pressed; } touch;
        struct { int channel; uint16_t raw; } adc;
        struct {
            int port;
            size_t len;
            uint8_t data[SBX_INPUT_UART_MAX];
        } uart;
        struct { int port; } uart_break;
    };
} sbx_input_event_t;

/* Parse one complete NDJSON object.  Unknown or malformed input is rejected;
 * callers may ignore false to keep an untrusted frontend from affecting the
 * guest. UART input accepts either one byte ("b") or a bounded bulk "hex"
 * string, so it can carry arbitrary binary protocols without JSON escaping. */
bool sbx_input_parse(const char *line, sbx_input_event_t *out);

#endif
