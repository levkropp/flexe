#ifndef PERIPHERALS_H
#define PERIPHERALS_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
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
/* Numbered variants expose all three ESP32 UART controllers.  The original
 * functions above and below remain UART0 convenience wrappers for existing
 * frontends. */
void periph_set_uart_callback_num(esp32_periph_t *p, int uart_num,
                                  uart_tx_cb cb, void *ctx);
int  periph_uart_tx_count_num(const esp32_periph_t *p, int uart_num);
const uint8_t *periph_uart_tx_buf_num(const esp32_periph_t *p, int uart_num);

/* Inject bytes arriving from the host. Returns the number accepted by the
 * selected 128-byte hardware RX FIFO; the normal ESP32 RX interrupt path then
 * moves them into the firmware driver's ring buffer. */
size_t periph_uart_rx_inject(esp32_periph_t *p, const uint8_t *data,
                             size_t len);
size_t periph_uart_rx_pending(const esp32_periph_t *p);
size_t periph_uart_rx_inject_num(esp32_periph_t *p, int uart_num,
                                 const uint8_t *data, size_t len);
size_t periph_uart_rx_pending_num(const esp32_periph_t *p, int uart_num);
int  periph_unhandled_count(const esp32_periph_t *p);

/* Returns true once the APP_CPU has been released from reset (DPORT write) */
bool periph_app_cpu_released(const esp32_periph_t *p);

/* Attach CPU pointers for interrupt delivery (call after cpu init) */
void periph_attach_cpus(esp32_periph_t *p, xtensa_cpu_t *cpu0, xtensa_cpu_t *cpu1);

/* Access the backing memory object (used by spi_display) */
xtensa_mem_t *periph_mem(esp32_periph_t *p);

/* Current GPIO output level of a pin (0/1, -1 if invalid). Used by the
 * GP-SPI display/touch sniffer to sample CS and D/C lines. */
int periph_gpio_pin_level(const esp32_periph_t *p, int pin);

/* GPIO-matrix output signal selected for a pin (bits OUT_SEL[8:0] of
 * GPIO_FUNCn_OUT_SEL_CFG_REG), or -1 for an invalid pin.  GP-SPI device
 * emulation uses this to distinguish SPI2 and SPI3 wiring even when a chip
 * select is controlled by the peripheral instead of the GPIO output latch. */
int periph_gpio_out_signal(const esp32_periph_t *p, int pin);

/* Assert/deassert a peripheral interrupt source (0-70).
 * Scans the interrupt matrix to find mapped CPU interrupt lines and
 * sets/clears the corresponding bits in cpu->interrupt. */
void periph_assert_interrupt(esp32_periph_t *p, int source);
void periph_deassert_interrupt(esp32_periph_t *p, int source);

/* Direct interrupt matrix access (used by intr_matrix_set ROM stub) */
void periph_intr_matrix_set(esp32_periph_t *p, int core, int cpu_int, int source);
int  periph_intr_matrix_get(const esp32_periph_t *p, int core, int cpu_int);

/* Drive a GPIO input pin to a level from outside the emulator (sandbox
 * frontend, host-side tests). Updates GPIO_IN_REG / GPIO_IN1_REG so the
 * firmware's gpio_get_level() returns the new value on its next read. */
void periph_gpio_set_input(esp32_periph_t *p, int pin, int level);

/* ADC input injection. Channels 0-39 cover both ADC1 (0-9) and ADC2 (0-9)
 * plus the GPIO-number oriented indexing used by some APIs; we over-allocate
 * to 40 slots for simplicity. Default value is 0. */
void     periph_set_adc_value(esp32_periph_t *p, int channel, uint16_t raw);
uint16_t periph_get_adc_value(const esp32_periph_t *p, int channel);

#endif /* PERIPHERALS_H */
