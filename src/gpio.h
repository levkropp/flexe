/* Descriptor-driven S2/S3-generation digital GPIO matrix. */
#ifndef FLEXE_GPIO_H
#define FLEXE_GPIO_H

#include <stdbool.h>

#include "memory.h"

typedef struct {
    uint64_t mask;
    uint64_t levels;
    uint64_t level_known;
    uint64_t enables;
    uint64_t enable_known;
} flexe_gpio_pad_hold_t;

typedef struct flexe_gpio flexe_gpio_t;

/* `level` or `enabled` is -1 when a peripheral-selected signal has no
 * attached producer. This is deliberately distinct from inventing a level. */
typedef void (*flexe_gpio_output_fn)(void *ctx, unsigned gpio,
                                     int level, int enabled);
typedef void (*flexe_gpio_irq_fn)(void *ctx, bool nmi, bool level);
typedef void (*flexe_gpio_input_signal_fn)(void *ctx, unsigned signal,
                                           bool old_level, bool level);

flexe_gpio_t *flexe_gpio_create(
    xtensa_mem_t *mem, mmio_read_fn fallback_read,
    mmio_write_fn fallback_write, void *fallback_ctx,
    flexe_gpio_output_fn output_changed, void *output_ctx,
    flexe_gpio_irq_fn irq_changed, void *irq_ctx);
void flexe_gpio_destroy(flexe_gpio_t *gpio);

/* Software-visible output latch and effective pad drive-enable. In
 * open-drain mode a high output level releases the pad (enabled == 0), not
 * a driven-high voltage. A valid pin may still return -1 when its output or
 * output-enable comes from an unattached peripheral signal. */
int flexe_gpio_pin_level(const flexe_gpio_t *gpio, unsigned pin);
int flexe_gpio_output_enabled(const flexe_gpio_t *gpio, unsigned pin);
int flexe_gpio_out_signal(const flexe_gpio_t *gpio, unsigned pin);
int flexe_gpio_out_route(const flexe_gpio_t *gpio, unsigned pin);

/* A held digital pad retains its physical level and output-enable while the
 * GPIO output latches continue to accept writes. Releasing hold immediately
 * exposes the current latch. RTC owns the hold register and calls this API. */
void flexe_gpio_set_pad_hold(flexe_gpio_t *gpio, uint64_t held_pins);
void flexe_gpio_pad_hold_snapshot(const flexe_gpio_t *gpio,
                                  flexe_gpio_pad_hold_t *out);
void flexe_gpio_pad_hold_restore(flexe_gpio_t *gpio,
                                 const flexe_gpio_pad_hold_t *in);

/* RTCIO selects the physical pad owner. Digital GPIO latches continue to
 * change while RTC owns a pad; unmodeled RTC pad functions resolve as unknown
 * instead of leaking the digital output onto the pin. */
void flexe_gpio_set_rtc_state(flexe_gpio_t *gpio, uint64_t owned,
                              uint64_t unknown, uint64_t output,
                              uint64_t enabled);

/* Register a peripheral output producer implemented by the emulator. Matrix
 * selections for unregistered producers remain visible but are diagnosed via
 * the fallback handler, so adding a controller does not require weakening the
 * GPIO model's unsupported-behavior accounting. */
void flexe_gpio_set_output_signal_modeled(flexe_gpio_t *gpio,
                                          unsigned signal);

/* Watch selected GPIO-matrix input signals for digital transitions. Both
 * host samples and output-to-input feedback use the same notification path;
 * unwatched signals have no per-edge callback cost. */
void flexe_gpio_set_input_signal_handler(flexe_gpio_t *gpio,
                                         flexe_gpio_input_signal_fn changed,
                                         void *ctx);
void flexe_gpio_watch_input_signal(flexe_gpio_t *gpio, unsigned signal);

/* IO_MUX controls the digital input buffer. Standalone GPIO models default
 * to enabled inputs; a target with IO_MUX supplies the reset/programmed state
 * through this API. */
void flexe_gpio_set_input_enable(flexe_gpio_t *gpio, unsigned pin,
                                 bool enabled);

/* Supply an observed host-pad sample. This overrides digital output feedback
 * on that pin; invalid or unbonded pins are ignored. */
void flexe_gpio_set_input(flexe_gpio_t *gpio, unsigned pin, bool level);
/* Raw pad sample for RTCIO and wake logic, independent of digital FUN_IE. */
int flexe_gpio_input_level(const flexe_gpio_t *gpio, unsigned pin);

/* Resolve a peripheral input routed through the GPIO matrix. Returns -1 for
 * an invalid signal, IO_MUX bypass, or an unbonded selected pin. */
int flexe_gpio_input_signal_level(const flexe_gpio_t *gpio, unsigned signal);

#endif /* FLEXE_GPIO_H */
