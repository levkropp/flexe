#ifndef FLEXE_I2S_V2_H
#define FLEXE_I2S_V2_H

#include "gdma.h"
#include "memory.h"
#include "xtensa.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef struct flexe_i2s_v2 flexe_i2s_v2_t;

typedef void (*flexe_i2s_v2_state_fn)(void *ctx);
typedef void (*flexe_i2s_v2_irq_fn)(void *ctx, unsigned port, bool level);
typedef void (*flexe_i2s_v2_tx_fn)(void *ctx, int port,
                                   const uint8_t *data, size_t length,
                                   uint32_t sample_rate,
                                   uint8_t bits_per_sample,
                                   uint8_t channels);

flexe_i2s_v2_t *flexe_i2s_v2_create(
    xtensa_mem_t *mem, flexe_gdma_t *gdma,
    mmio_read_fn fallback_read, mmio_write_fn fallback_write,
    void *fallback_ctx, flexe_i2s_v2_state_fn state_changed,
    void *state_ctx, flexe_i2s_v2_irq_fn irq_changed, void *irq_ctx);
void flexe_i2s_v2_destroy(flexe_i2s_v2_t *i2s);
void flexe_i2s_v2_attach_cpus(flexe_i2s_v2_t *i2s,
                              xtensa_cpu_t *cpu0, xtensa_cpu_t *cpu1);
uint32_t flexe_i2s_v2_next_event(flexe_i2s_v2_t *i2s,
                                 xtensa_cpu_t *cpu);
void flexe_i2s_v2_eval(flexe_i2s_v2_t *i2s);
void flexe_i2s_v2_set_system_state(flexe_i2s_v2_t *i2s, unsigned port,
                                   bool clock_enabled,
                                   bool reset_asserted);

int flexe_i2s_v2_set_tx_callback(flexe_i2s_v2_t *i2s, unsigned port,
                                 flexe_i2s_v2_tx_fn callback, void *ctx);
size_t flexe_i2s_v2_rx_inject(flexe_i2s_v2_t *i2s, unsigned port,
                              const uint8_t *data, size_t length);
size_t flexe_i2s_v2_rx_pending(const flexe_i2s_v2_t *i2s, unsigned port);

#endif /* FLEXE_I2S_V2_H */
