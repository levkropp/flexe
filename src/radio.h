/* Descriptor-driven Wi-Fi/Bluetooth register and calibration surfaces. */
#ifndef FLEXE_RADIO_H
#define FLEXE_RADIO_H

#include "memory.h"
#include "xtensa.h"

typedef struct flexe_radio flexe_radio_t;

/* The model retains opaque RF/controller configuration registers and models
 * only target-described completion and entropy semantics. This is the fast
 * functional contract; it is not an analog or packet-level RF medium. */
flexe_radio_t *flexe_radio_create(
    xtensa_mem_t *mem, mmio_read_fn fallback_read,
    mmio_write_fn fallback_write, void *fallback_ctx);
void flexe_radio_attach_cpus(flexe_radio_t *radio,
                              xtensa_cpu_t *cpu0, xtensa_cpu_t *cpu1);
void flexe_radio_destroy(flexe_radio_t *radio);

#endif /* FLEXE_RADIO_H */
