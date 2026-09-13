#ifndef FLEXE_RMT_V1_H
#define FLEXE_RMT_V1_H

#include "memory.h"
#include "xtensa.h"

#include <stddef.h>
#include <stdint.h>

typedef struct flexe_rmt_v1 flexe_rmt_v1_t;
typedef void (*flexe_rmt_v1_state_fn)(void *ctx);
typedef void (*flexe_rmt_v1_irq_fn)(void *ctx, uint32_t status);
typedef void (*flexe_rmt_v1_tx_fn)(void *ctx, int channel,
                                   const uint32_t *items, size_t count,
                                   uint32_t tick_hz, uint32_t carrier_hz,
                                   bool complete);

flexe_rmt_v1_t *flexe_rmt_v1_create(xtensa_mem_t *mem,
                                    mmio_read_fn fallback_read,
                                    mmio_write_fn fallback_write,
                                    void *fallback_ctx,
                                    flexe_rmt_v1_state_fn state_changed,
                                    void *state_ctx,
                                    flexe_rmt_v1_irq_fn irq_changed,
                                    void *irq_ctx);
void flexe_rmt_v1_destroy(flexe_rmt_v1_t *rmt);
void flexe_rmt_v1_attach_cpus(flexe_rmt_v1_t *rmt,
                               xtensa_cpu_t *cpu0, xtensa_cpu_t *cpu1);
uint32_t flexe_rmt_v1_next_event(flexe_rmt_v1_t *rmt,
                                  xtensa_cpu_t *cpu);
void flexe_rmt_v1_eval(flexe_rmt_v1_t *rmt);
int flexe_rmt_v1_set_tx_callback(flexe_rmt_v1_t *rmt, unsigned channel,
                                  flexe_rmt_v1_tx_fn fn, void *ctx);

#endif
