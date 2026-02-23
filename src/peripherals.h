#ifndef PERIPHERALS_H
#define PERIPHERALS_H

#include <stdint.h>
#include "memory.h"

typedef struct esp32_periph esp32_periph_t;

/* UART TX callback: called for each byte written to UART FIFO */
typedef void (*uart_tx_cb)(void *ctx, uint8_t byte);

esp32_periph_t *periph_create(xtensa_mem_t *mem);
void periph_destroy(esp32_periph_t *p);

void periph_set_uart_callback(esp32_periph_t *p, uart_tx_cb cb, void *ctx);
int  periph_uart_tx_count(const esp32_periph_t *p);
const uint8_t *periph_uart_tx_buf(const esp32_periph_t *p);
int  periph_unhandled_count(const esp32_periph_t *p);

#endif /* PERIPHERALS_H */
