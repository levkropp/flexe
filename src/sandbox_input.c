#include "sandbox_input.h"

#include <errno.h>
#include <limits.h>
#include <stdlib.h>
#include <string.h>

static const char *json_value(const char *line, const char *key)
{
    char pattern[40];
    size_t key_len = strlen(key);
    if (key_len + 3u > sizeof(pattern)) return NULL;
    pattern[0] = '"';
    memcpy(pattern + 1, key, key_len);
    pattern[key_len + 1u] = '"';
    pattern[key_len + 2u] = '\0';

    const char *p = line;
    while ((p = strstr(p, pattern)) != NULL) {
        const char *value = p + key_len + 2u;
        while (*value == ' ' || *value == '\t' ||
               *value == '\r' || *value == '\n')
            value++;
        if (*value++ != ':') {
            p++;
            continue;
        }
        while (*value == ' ' || *value == '\t' ||
               *value == '\r' || *value == '\n')
            value++;
        return value;
    }
    return NULL;
}

static bool json_long(const char *line, const char *key, long *out)
{
    const char *p = json_value(line, key);
    if (!p) return false;
    errno = 0;
    char *end = NULL;
    long value = strtol(p, &end, 10);
    if (end == p || errno == ERANGE) return false;
    while (*end == ' ' || *end == '\t' || *end == '\r' || *end == '\n')
        end++;
    if (*end != ',' && *end != '}') return false;
    *out = value;
    return true;
}

static bool json_string(const char *line, const char *key,
                        char *out, size_t capacity, size_t *length)
{
    const char *p = json_value(line, key);
    if (!p || *p++ != '"' || capacity == 0u) return false;
    size_t n = 0u;
    while (*p && *p != '"') {
        /* Event names and hex payloads have no reason to use escapes.  Reject
         * them instead of accepting an ambiguous, partially decoded value. */
        if (*p == '\\' || n + 1u >= capacity) return false;
        out[n++] = *p++;
    }
    if (*p != '"') return false;
    p++;
    while (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n') p++;
    if (*p != ',' && *p != '}') return false;
    out[n] = '\0';
    if (length) *length = n;
    return true;
}

static int hex_nibble(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

static bool parse_port(const char *line, int *port)
{
    long value;
    if (!json_long(line, "u", &value) &&
        !json_long(line, "port", &value))
        return false;
    if (value < 0 || value > INT_MAX) return false;
    *port = (int)value;
    return true;
}

bool sbx_input_parse(const char *line, sbx_input_event_t *out)
{
    if (!line || !out) return false;
    memset(out, 0, sizeof(*out));

    char type[32];
    if (!json_string(line, "t", type, sizeof(type), NULL)) return false;

    if (strcmp(type, "gpio_in") == 0) {
        long pin, level;
        if (!json_long(line, "pin", &pin) ||
            !json_long(line, "lvl", &level) ||
            pin < 0 || pin > INT_MAX || (level != 0 && level != 1))
            return false;
        out->kind = SBX_INPUT_GPIO;
        out->gpio.pin = (int)pin;
        out->gpio.level = (int)level;
        return true;
    }

    if (strcmp(type, "touch_in") == 0) {
        long x, y, pressed;
        if (!json_long(line, "x", &x) || !json_long(line, "y", &y) ||
            !json_long(line, "pressed", &pressed) ||
            x < INT_MIN || x > INT_MAX || y < INT_MIN || y > INT_MAX ||
            (pressed != 0 && pressed != 1))
            return false;
        out->kind = SBX_INPUT_TOUCH;
        out->touch.x = (int)x;
        out->touch.y = (int)y;
        out->touch.pressed = (int)pressed;
        return true;
    }

    if (strcmp(type, "adc_in") == 0) {
        long channel, raw;
        if (!json_long(line, "ch", &channel) ||
            !json_long(line, "raw", &raw) ||
            channel < 0 || channel > INT_MAX || raw < 0 || raw > UINT16_MAX)
            return false;
        out->kind = SBX_INPUT_ADC;
        out->adc.channel = (int)channel;
        out->adc.raw = (uint16_t)raw;
        return true;
    }

    if (strcmp(type, "uart_in") == 0) {
        if (!parse_port(line, &out->uart.port)) return false;
        long byte;
        if (json_long(line, "b", &byte)) {
            if (byte < 0 || byte > UINT8_MAX) return false;
            out->kind = SBX_INPUT_UART;
            out->uart.data[0] = (uint8_t)byte;
            out->uart.len = 1u;
            return true;
        }

        char hex[SBX_INPUT_UART_MAX * 2u + 1u];
        size_t hex_len;
        if (!json_string(line, "hex", hex, sizeof(hex), &hex_len) ||
            hex_len == 0u || (hex_len & 1u) != 0u)
            return false;
        size_t bytes = hex_len / 2u;
        for (size_t i = 0u; i < bytes; i++) {
            int high = hex_nibble(hex[i * 2u]);
            int low = hex_nibble(hex[i * 2u + 1u]);
            if (high < 0 || low < 0) return false;
            out->uart.data[i] = (uint8_t)((high << 4) | low);
        }
        out->kind = SBX_INPUT_UART;
        out->uart.len = bytes;
        return true;
    }

    if (strcmp(type, "uart_break") == 0) {
        if (!parse_port(line, &out->uart_break.port)) return false;
        out->kind = SBX_INPUT_UART_BREAK;
        return true;
    }

    return false;
}
