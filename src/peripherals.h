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

/* Virtual I2C target. A registered address ACKs on the bus; Flexe invokes the
 * callback when a write ends or when a read needs data. For a repeated-start
 * register read, write_data contains the bytes sent before the restart and
 * read_data is the response buffer. Return 0 to ACK the transfer or nonzero
 * to report a target-side NACK/error. */
typedef int (*periph_i2c_device_fn)(void *ctx, int port, uint8_t address,
                                    const uint8_t *write_data,
                                    size_t write_len, uint8_t *read_data,
                                    size_t read_len);

/* Compatibility-mode firmware still registers real ESP-IDF peripheral ISRs,
 * even though Flexe replaces the FreeRTOS scheduler. A source callback lets
 * the ROM-stub layer dispatch those guest handlers when a modelled interrupt
 * makes a low-to-high transition. */
typedef void (*periph_irq_dispatch_fn)(void *ctx, int source);

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

/* Attach/detach a 7-bit target address on either classic ESP32 I2C master.
 * Passing NULL as fn detaches the address. */
int periph_i2c_attach_device(esp32_periph_t *p, int port, uint8_t address,
                             periph_i2c_device_fn fn, void *ctx);

/* Observe rising edges for an ESP32 peripheral source (0-70). Passing NULL
 * detaches the observer. This is separate from the hardware interrupt matrix:
 * native-FreeRTOS sessions continue to use the emulated CPU interrupt lines. */
int periph_set_irq_dispatch(esp32_periph_t *p, int source,
                            periph_irq_dispatch_fn fn, void *ctx);

/* Returns true once the APP_CPU has been released from reset (DPORT write) */
bool periph_app_cpu_released(const esp32_periph_t *p);

/* Attach CPU pointers for interrupt delivery (call after cpu init) */
void periph_attach_cpus(esp32_periph_t *p, xtensa_cpu_t *cpu0, xtensa_cpu_t *cpu1);

/* Access the backing memory object (used by spi_display) */
xtensa_mem_t *periph_mem(esp32_periph_t *p);

/* Current GPIO output level of a pin (0/1, -1 if invalid). Used by the
 * GP-SPI display/touch sniffer to sample CS and D/C lines. */
int periph_gpio_pin_level(const esp32_periph_t *p, int pin);

/* Whether the GPIO output driver is enabled for a pin. This distinguishes an
 * intentionally driven-low software chip select from an untouched reset pin. */
int periph_gpio_output_enabled(const esp32_periph_t *p, int pin);

/* GPIO-matrix output signal selected for a pin (bits OUT_SEL[8:0] of
 * GPIO_FUNCn_OUT_SEL_CFG_REG), or -1 for an invalid pin.  GP-SPI device
 * emulation uses this to distinguish SPI2 and SPI3 wiring even when a chip
 * select is controlled by the peripheral instead of the GPIO output latch. */
int periph_gpio_out_signal(const esp32_periph_t *p, int pin);

/* Explicit IO_MUX function selected for a pin (MCU_SEL bits), or -1 when the
 * firmware has not written that pin's mux register. Native HSPI/VSPI routing
 * uses function 1 on the classic ESP32 pin sets. */
int periph_iomux_function(const esp32_periph_t *p, int pin);

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
