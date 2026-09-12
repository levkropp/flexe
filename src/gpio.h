/* Descriptor-driven S2/S3-generation digital GPIO matrix. */
#ifndef FLEXE_GPIO_H
#define FLEXE_GPIO_H

#include <stdbool.h>

#include "memory.h"

typedef struct flexe_gpio flexe_gpio_t;

/* `level` or `enabled` is -1 when a peripheral-selected signal has no
 * attached producer. This is deliberately distinct from inventing a level. */
typedef void (*flexe_gpio_output_fn)(void *ctx, unsigned gpio,
                                     int level, int enabled);
typedef void (*flexe_gpio_irq_fn)(void *ctx, bool nmi, bool level);

flexe_gpio_t *flexe_gpio_create(
    xtensa_mem_t *mem, mmio_read_fn fallback_read,
    mmio_write_fn fallback_write, void *fallback_ctx,
    flexe_gpio_output_fn output_changed, void *output_ctx,
    flexe_gpio_irq_fn irq_changed, void *irq_ctx);
void flexe_gpio_destroy(flexe_gpio_t *gpio);

/* Software-visible output state. A valid pin may still return -1 when its
 * output or output-enable comes from a peripheral signal whose producer is
 * not attached to this matrix model. */
int flexe_gpio_pin_level(const flexe_gpio_t *gpio, unsigned pin);
int flexe_gpio_output_enabled(const flexe_gpio_t *gpio, unsigned pin);
int flexe_gpio_out_signal(const flexe_gpio_t *gpio, unsigned pin);

/* Register a peripheral output producer implemented by the emulator. Matrix
 * selections for unregistered producers remain visible but are diagnosed via
 * the fallback handler, so adding a controller does not require weakening the
 * GPIO model's unsupported-behavior accounting. */
void flexe_gpio_set_output_signal_modeled(flexe_gpio_t *gpio,
                                          unsigned signal);

/* Drive the post-pad digital input sampled by GPIO_IN/IN1. Invalid or
 * unbonded pins are ignored. */
void flexe_gpio_set_input(flexe_gpio_t *gpio, unsigned pin, bool level);

/* Resolve a peripheral input routed through the GPIO matrix. Returns -1 for
 * an invalid signal, IO_MUX bypass, or an unbonded selected pin. */
int flexe_gpio_input_signal_level(const flexe_gpio_t *gpio, unsigned signal);

#endif /* FLEXE_GPIO_H */
