/* ESP32-S3 external-memory/cache controller. */
#ifndef FLEXE_ESP32S3_EXTMEM_H
#define FLEXE_ESP32S3_EXTMEM_H

#include "memory.h"

typedef struct xtensa_cpu xtensa_cpu_t;
typedef struct flexe_esp32s3_extmem flexe_esp32s3_extmem_t;

/* Create the register block in its documented reset state. The target must
 * advertise FLEXE_TARGET_CAP_ESP32S3_EXTMEM and describe its MMIO range. */
flexe_esp32s3_extmem_t *flexe_esp32s3_extmem_create(xtensa_mem_t *mem);
void flexe_esp32s3_extmem_destroy(flexe_esp32s3_extmem_t *extmem);

/* Flexe currently enters an application after the second-stage bootloader.
 * Reproduce the cache/bus state handed to that application without claiming
 * that the skipped bootloader ran. */
void flexe_esp32s3_extmem_application_handoff(
    flexe_esp32s3_extmem_t *extmem);

/* I-cache synchronization invalidates decoded/JIT code on every attached
 * execution engine. D-cache operations are coherent no-ops in fast mode. */
void flexe_esp32s3_extmem_attach_cpus(flexe_esp32s3_extmem_t *extmem,
                                      xtensa_cpu_t *cpu0,
                                      xtensa_cpu_t *cpu1);

#endif /* FLEXE_ESP32S3_EXTMEM_H */
