#ifndef PERIPHERALS_H
#define PERIPHERALS_H

#include <stdint.h>
#include <stdbool.h>
#include "memory.h"

/* Forward declaration */
typedef struct xtensa_cpu xtensa_cpu_t;

typedef struct esp32_periph esp32_periph_t;

/* UART TX callback: called for each byte written to UART FIFO */
typedef void (*uart_tx_cb)(void *ctx, uint8_t byte);

esp32_periph_t *periph_create(xtensa_mem_t *mem);
void periph_destroy(esp32_periph_t *p);

void periph_set_uart_callback(esp32_periph_t *p, uart_tx_cb cb, void *ctx);
int  periph_uart_tx_count(const esp32_periph_t *p);
const uint8_t *periph_uart_tx_buf(const esp32_periph_t *p);
int  periph_unhandled_count(const esp32_periph_t *p);

/* Returns true once the APP_CPU has been released from reset (DPORT write) */
bool periph_app_cpu_released(const esp32_periph_t *p);

/* Attach CPU pointers for interrupt delivery (call after cpu init) */
void periph_attach_cpus(esp32_periph_t *p, xtensa_cpu_t *cpu0, xtensa_cpu_t *cpu1);

/* Assert/deassert a peripheral interrupt source (0-70).
 * Scans the interrupt matrix to find mapped CPU interrupt lines and
 * sets/clears the corresponding bits in cpu->interrupt. */
void periph_assert_interrupt(esp32_periph_t *p, int source);
void periph_deassert_interrupt(esp32_periph_t *p, int source);

/* Direct interrupt matrix access (used by intr_matrix_set ROM stub) */
void periph_intr_matrix_set(esp32_periph_t *p, int core, int cpu_int, int source);
int  periph_intr_matrix_get(const esp32_periph_t *p, int core, int cpu_int);

#endif /* PERIPHERALS_H */
