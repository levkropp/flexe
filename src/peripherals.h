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

/* Native SDMMC cards expose 512-byte logical sectors to the two-slot host.
 * Callbacks return zero on success and nonzero for a card-side I/O failure. */
typedef int (*periph_sdmmc_read_blocks_fn)(void *ctx, uint32_t first_sector,
                                           uint8_t *data, size_t count);
typedef int (*periph_sdmmc_write_blocks_fn)(void *ctx,
                                            uint32_t first_sector,
                                            const uint8_t *data,
                                            size_t count);

/* One classic CAN 2.0A/B frame crossing the ESP32 TWAI controller's external
 * bus boundary. DLC values 9-15 are preserved for non-compliant frames, but
 * only the first eight data bytes exist in classic CAN hardware. */
typedef struct {
    uint32_t identifier;
    uint8_t data_length_code;
    uint8_t data[8];
    bool extended;
    bool remote;
    bool single_shot;
    bool self_reception;
} periph_twai_frame_t;

typedef enum {
    PERIPH_TWAI_TX_ACK = 0,
    PERIPH_TWAI_TX_NO_ACK,
    PERIPH_TWAI_TX_ARBITRATION_LOST,
    PERIPH_TWAI_TX_BUS_ERROR,
} periph_twai_tx_result_t;

/* Called once per physical transmit attempt. Returning a bus error lets a
 * host model exercise automatic retry, error-passive, and bus-off behavior. */
typedef periph_twai_tx_result_t (*periph_twai_tx_fn)(
    void *ctx, const periph_twai_frame_t *frame);

/* One Ethernet frame leaving the classic ESP32 EMAC. The frame excludes the
 * preamble and FCS, matching TAP/raw-socket host APIs. Return zero when the
 * virtual link accepted the frame or nonzero to report a carrier/link error
 * in the completed DMA descriptor. */
typedef int (*periph_emac_tx_fn)(void *ctx, const uint8_t *frame,
                                 size_t len);

/* Clause-22 MDIO transaction at the MAC/PHY boundary. For a write, *value is
 * the guest-provided register value; for a read, the callback fills *value.
 * A nonzero return models an absent/unresponsive PHY (reads become 0xFFFF). */
typedef int (*periph_emac_mdio_fn)(void *ctx, uint8_t phy_address,
                                   uint8_t reg, bool write,
                                   uint16_t *value);

/* Raw PCM bytes consumed by one classic ESP32 I2S TX DMA descriptor. The
 * buffer is valid only for the duration of the callback. */
typedef void (*periph_i2s_tx_fn)(void *ctx, int port, const uint8_t *data,
                                 size_t len, uint32_t sample_rate,
                                 uint8_t bits_per_sample, uint8_t channels);

/* One completed portion of an ESP32 RMT transmission. Long writes can arrive
 * in multiple chunks as the genuine driver refills the peripheral's shared
 * RAM at threshold interrupts. `items` is valid only during the callback;
 * tick_hz describes the duration fields and carrier_hz is zero when carrier
 * modulation is disabled. */
typedef void (*periph_rmt_tx_fn)(void *ctx, int channel,
                                 const uint32_t *items, size_t count,
                                 uint32_t tick_hz, uint32_t carrier_hz,
                                 bool finished);

/* Aggregate output state for one classic ESP32 LEDC PWM channel. The GPIO is
 * resolved through the live GPIO matrix (-1 while unrouted); duty and
 * duty_max describe the hardware timer resolution, while frequency_hz is
 * the configured PWM carrier. `inverted` reflects GPIO matrix output
 * inversion rather than rewriting the programmed duty value. */
typedef void (*periph_ledc_output_fn)(void *ctx, int speed_mode, int channel,
                                     int gpio, uint32_t frequency_hz,
                                     uint32_t duty, uint32_t duty_max,
                                     bool enabled, bool inverted);

/* Aggregate output state for one classic ESP32 sigma-delta channel. The
 * signed duty has the hardware range -128..127 and therefore represents
 * (duty + 128) high samples out of every 256 modulator samples. The reported
 * frequency matches Arduino's sigmaDeltaSetup API: 80 MHz divided by the
 * channel prescaler and the 256-sample modulation period. */
typedef void (*periph_sigmadelta_output_fn)(void *ctx, int channel, int gpio,
                                            uint32_t frequency_hz,
                                            int8_t duty, bool enabled,
                                            bool inverted);

/* Aggregate configuration for one classic ESP32 MCPWM generator output.
 * `period_ticks` and `compare_ticks` use the selected operator timer's
 * resolution. Dead-time delays use `deadtime_clock_hz`; carrier_duty_eighths
 * is the hardware's 0/8..7/8 encoding. `forced_level` is -1 when neither a
 * software force nor a fault action overrides the event generator. */
typedef struct {
    int gpio;
    uint32_t frequency_hz;
    uint32_t period_ticks;
    uint32_t compare_ticks;
    uint32_t rising_delay_ticks;
    uint32_t falling_delay_ticks;
    uint32_t deadtime_clock_hz;
    uint32_t carrier_hz;
    uint8_t carrier_duty_eighths;
    uint8_t count_mode;
    bool enabled;
    bool inverted;
    bool fault_active;
    int8_t forced_level;
} periph_mcpwm_output_info_t;

typedef void (*periph_mcpwm_output_fn)(
    void *ctx, int unit, int operator_index, int generator,
    const periph_mcpwm_output_info_t *info);

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
/* Signal a UART break condition to a UHCI receiver configured to use break
 * as packet EOF. Returns true when an active controller consumed the event. */
bool periph_uart_rx_break_num(esp32_periph_t *p, int uart_num);
int  periph_unhandled_count(const esp32_periph_t *p);

/* Attach/detach a 7-bit target address on either classic ESP32 I2C master.
 * Passing NULL as fn detaches the address. */
int periph_i2c_attach_device(esp32_periph_t *p, int port, uint8_t address,
                             periph_i2c_device_fn fn, void *ctx);

/* Insert or remove an SDHC card in native SDMMC slot 0/1. sector_count is
 * rounded down to the CSD-v2 capacity granularity (1024 sectors) when the
 * card reports its geometry. A NULL read callback detaches the slot. */
int periph_sdmmc_attach_card(esp32_periph_t *p, int slot,
                             uint32_t sector_count,
                             periph_sdmmc_read_blocks_fn read_fn,
                             periph_sdmmc_write_blocks_fn write_fn,
                             void *ctx);
int periph_sdmmc_set_write_protected(esp32_periph_t *p, int slot,
                                     bool write_protected);

/* Act as the external SDIO host connected to the classic ESP32 slave block.
 * Packet helpers walk the guest's real SLC lldesc chains and drive source-10
 * interrupts. `host_write_packet` is host-to-ESP32 and consumes TOKEN1 receive
 * buffers; `host_read_packet` is ESP32-to-host and consumes the cumulative
 * PKT_LEN delta. Packet calls return 1 on transfer, 0 when no packet/buffer is
 * ready, and -1 for an invalid argument, undersized destination, or malformed
 * descriptor chain. On an undersized read, out_len reports the required size.
 * Shared register positions use the SDIO protocol's 0..63 numbering. */
bool periph_sdio_slave_host_ready(const esp32_periph_t *p);
uint16_t periph_sdio_slave_host_send_buffers(const esp32_periph_t *p);
uint32_t periph_sdio_slave_host_receive_bytes(const esp32_periph_t *p);
int periph_sdio_slave_host_write_packet(esp32_periph_t *p,
                                        const uint8_t *data, size_t len);
int periph_sdio_slave_host_read_packet(esp32_periph_t *p, uint8_t *data,
                                       size_t capacity, size_t *out_len);
int periph_sdio_slave_host_read_reg(const esp32_periph_t *p,
                                    unsigned position, uint8_t *value);
int periph_sdio_slave_host_write_reg(esp32_periph_t *p, unsigned position,
                                     uint8_t value);

/* General-purpose interrupts crossing the SDIO link. `host_interrupt`
 * injects bits 0..7 toward the ESP32; raw/pending/clear operate on interrupts
 * raised toward the host. Pending applies the guest-programmed function-1
 * enable mask and HINF interrupt mask. */
int periph_sdio_slave_host_interrupt(esp32_periph_t *p, uint8_t mask);
uint32_t periph_sdio_slave_host_interrupt_raw(const esp32_periph_t *p);
uint32_t periph_sdio_slave_host_interrupt_pending(const esp32_periph_t *p);
void periph_sdio_slave_host_interrupt_clear(esp32_periph_t *p,
                                            uint32_t mask);

/* Attach a virtual CAN bus peer and inject a received frame. With no transmit
 * callback attached, a virtual peer ACKs valid transmissions by default.
 * Injection returns 1 when accepted into the 64-byte hardware FIFO and 0 when
 * filtered, inactive, invalid, or overrun. */
int periph_set_twai_tx_callback(esp32_periph_t *p, periph_twai_tx_fn fn,
                                void *ctx);
int periph_twai_rx_inject(esp32_periph_t *p,
                          const periph_twai_frame_t *frame);
size_t periph_twai_rx_pending(const esp32_periph_t *p);

/* Attach Ethernet wire/MDIO peers. RX injection is frame-atomic and returns
 * one only when the running MAC accepted the destination filter and committed
 * the complete frame through DMA-owned descriptors. The built-in Clause-22
 * register bank is useful for simple PHY models when no callback is attached;
 * setting any register marks that PHY address present. */
int periph_set_emac_tx_callback(esp32_periph_t *p, periph_emac_tx_fn fn,
                                void *ctx);
int periph_set_emac_mdio_callback(esp32_periph_t *p,
                                  periph_emac_mdio_fn fn, void *ctx);
int periph_emac_phy_set_reg(esp32_periph_t *p, uint8_t phy_address,
                            uint8_t reg, uint16_t value);
int periph_emac_phy_get_reg(const esp32_periph_t *p, uint8_t phy_address,
                            uint8_t reg, uint16_t *value);
int periph_emac_rx_inject(esp32_periph_t *p, const uint8_t *frame,
                          size_t len);

/* Attach a TX audio sink and inject bytes for RX DMA. I2S ports 0/1 are
 * independent; the RX FIFO accepts as many bytes as fit and zero-fills when
 * firmware consumes faster than the host supplies. */
int periph_set_i2s_tx_callback(esp32_periph_t *p, int port,
                               periph_i2s_tx_fn fn, void *ctx);
size_t periph_i2s_rx_inject(esp32_periph_t *p, int port,
                            const uint8_t *data, size_t len);
size_t periph_i2s_rx_pending(const esp32_periph_t *p, int port);

/* Attach a host pulse sink to one of the eight classic ESP32 RMT channels.
 * RX injection accepts already-decoded RMT items after firmware has enabled
 * the channel, writes them through the hardware RAM/interrupt path, and
 * returns the number accepted by the configured memory blocks. */
int periph_set_rmt_tx_callback(esp32_periph_t *p, int channel,
                               periph_rmt_tx_fn fn, void *ctx);
size_t periph_rmt_rx_inject(esp32_periph_t *p, int channel,
                            const uint32_t *items, size_t count);

/* Attach a PWM sink to one of the 16 classic LEDC outputs. speed_mode 0 is
 * the high-speed group and 1 is the low-speed group. The current aggregate
 * state is delivered immediately, and subsequent timer/channel/GPIO-matrix
 * changes produce another callback. */
int periph_set_ledc_output_callback(esp32_periph_t *p, int speed_mode,
                                    int channel, periph_ledc_output_fn fn,
                                    void *ctx);

/* Attach a pulse-density sink to one of the eight classic GPIO sigma-delta
 * channels. The current aggregate configuration is delivered immediately;
 * later duty, prescaler, GPIO-enable, and matrix-routing changes produce
 * another callback. */
int periph_set_sigmadelta_output_callback(
    esp32_periph_t *p, int channel, periph_sigmadelta_output_fn fn, void *ctx);

/* Attach a motor-control PWM sink to either MCPWM unit, one of its three
 * operators, and generator A/B. The current aggregate configuration is
 * delivered immediately; later timer, generator, dead-time, carrier, fault,
 * force, and GPIO-matrix changes produce another callback. */
int periph_set_mcpwm_output_callback(esp32_periph_t *p, int unit,
                                     int operator_index, int generator,
                                     periph_mcpwm_output_fn fn, void *ctx);

/* Observe rising edges for an ESP32 peripheral source (0-70). Passing NULL
 * detaches the observer. This is separate from the hardware interrupt matrix:
 * native-FreeRTOS sessions continue to use the emulated CPU interrupt lines. */
int periph_set_irq_dispatch(esp32_periph_t *p, int source,
                            periph_irq_dispatch_fn fn, void *ctx);
bool periph_interrupt_pending(const esp32_periph_t *p, int source);

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

/* Current classic-ESP32 DAC state. Channels 0/1 correspond to GPIO25/26.
 * Invalid channels return -1 for enabled and 0 for value. */
int     periph_dac_enabled(const esp32_periph_t *p, int channel);
uint8_t periph_dac_value(const esp32_periph_t *p, int channel);

#endif /* PERIPHERALS_H */
