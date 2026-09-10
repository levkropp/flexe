/* Descriptor-driven flash/PSRAM MMU model for shared I/D cache targets. */
#ifndef FLEXE_FLASH_MMU_H
#define FLEXE_FLASH_MMU_H

#include "memory.h"

typedef struct xtensa_cpu xtensa_cpu_t;
typedef struct flexe_flash_mmu flexe_flash_mmu_t;

/* Create and register the MMIO table described by mem's target. The current
 * implementation accepts targets whose instruction and data buses share one
 * table (ESP32-S3); classic ESP32 keeps its independent per-core tables in
 * the classic peripheral model. */
flexe_flash_mmu_t *flexe_flash_mmu_create(xtensa_mem_t *mem);
void flexe_flash_mmu_destroy(flexe_flash_mmu_t *mmu);

/* Attach execution engines after CPU initialization. Runtime remaps then
 * invalidate translated instruction pages on each independent engine. */
void flexe_flash_mmu_attach_cpus(flexe_flash_mmu_t *mmu,
                                 xtensa_cpu_t *cpu0,
                                 xtensa_cpu_t *cpu1);

#endif /* FLEXE_FLASH_MMU_H */
