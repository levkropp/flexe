#ifndef FLEXE_RMT_V1_H
#define FLEXE_RMT_V1_H

#include "memory.h"
#include "xtensa.h"

#include <stddef.h>
#include <stdint.h>

typedef struct flexe_rmt_v1 flexe_rmt_v1_t;
#define FLEXE_RMT_V1_RX_CHANNELS_MAX 4u
typedef void (*flexe_rmt_v1_state_fn)(void *ctx);
typedef void (*flexe_rmt_v1_irq_fn)(void *ctx, uint32_t status);
typedef void (*flexe_rmt_v1_tx_fn)(void *ctx, int channel,
                                   const uint32_t *items, size_t count,
                                   uint32_t tick_hz, uint32_t carrier_hz,
                                   bool complete);
typedef void (*flexe_rmt_v1_tx_edge_fn)(void *ctx, unsigned channel,
                                        int level, int enabled,
                                        uint64_t cycle);
typedef bool (*flexe_rmt_v1_tx_edge_needed_fn)(void *ctx,
                                               unsigned channel);

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
void flexe_rmt_v1_set_tx_edge_handler(flexe_rmt_v1_t *rmt,
                                      flexe_rmt_v1_tx_edge_fn changed,
                                      flexe_rmt_v1_tx_edge_needed_fn needed,
                                      void *ctx);
/* Sample the unmodulated TX pad at the current guest time for GPIO_IN polling
 * without scheduling every half-symbol edge. Unsupported carrier/loop modes
 * return false, preserving the GPIO unknown-input diagnostic. */
bool flexe_rmt_v1_tx_sample(flexe_rmt_v1_t *rmt, unsigned channel,
                            int *level, int *enabled);
/* Inject a complete, already-decoded pulse frame into physical RX channel
 * 4..7. Returns symbols accepted (one RAM capacity without wrap, or the
 * whole frame with wrap/ping-pong enabled). */
size_t flexe_rmt_v1_rx_inject(flexe_rmt_v1_t *rmt, unsigned channel,
                              const uint32_t *items, size_t count);

/* Feed a GPIO-matrix input transition into a physical RX channel (4..7).
 * Both levels are needed to seed the RX filter when the first edge arrives.
 * Pulse widths are measured on the guest CPU/RMT clock timeline. */
void flexe_rmt_v1_rx_input_edge(flexe_rmt_v1_t *rmt, unsigned channel,
                                 bool old_level, bool level);
/* For a TX edge emitted during RMT evaluation, use its exact event cycle
 * without recursively evaluating TX or substituting the later CPU time. */
void flexe_rmt_v1_rx_input_edge_at(flexe_rmt_v1_t *rmt, unsigned channel,
                                    bool old_level, bool level,
                                    uint64_t cycle);

#endif
