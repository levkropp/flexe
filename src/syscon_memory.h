/* Target-described SYSCON memory clock and power controls. */
#ifndef FLEXE_SYSCON_MEMORY_H
#define FLEXE_SYSCON_MEMORY_H

#include <stdbool.h>
#include <stdint.h>

#include "memory.h"
#include "target.h"

typedef struct flexe_syscon_memory flexe_syscon_memory_t;

typedef struct {
    uint32_t attributes;
    uint32_t address;
    uint32_t size_pages;
} flexe_syscon_ace_region_state_t;

/* Bank selections are normalized to bit zero, independent of their raw
 * register placement. Front-end bits are packed in ascending register order.
 * External-memory access-control regions retain their target-native units. */
typedef struct {
    uint8_t ace_region_count;
    flexe_syscon_ace_region_state_t
        flash_ace[FLEXE_TARGET_SYSCON_ACE_REGION_MAX];
    flexe_syscon_ace_region_state_t
        sram_ace[FLEXE_TARGET_SYSCON_ACE_REGION_MAX];
    uint8_t front_end_force_power_down;
    uint8_t front_end_force_power_up;
    uint16_t sram_clock_force_on;
    uint8_t rom_clock_force_on;
    uint16_t sram_force_power_down;
    uint8_t rom_force_power_down;
    uint16_t sram_force_power_up;
    uint8_t rom_force_power_up;
} flexe_syscon_memory_state_t;

typedef void (*flexe_syscon_memory_state_fn)(
    void *ctx, const flexe_syscon_memory_state_t *state);

flexe_syscon_memory_t *flexe_syscon_memory_create(
    xtensa_mem_t *mem, mmio_read_fn fallback_read,
    mmio_write_fn fallback_write, void *fallback_ctx);
void flexe_syscon_memory_destroy(flexe_syscon_memory_t *memory);

/* These dispatchers let another owner share the same page through an explicit
 * fallback chain instead of turning the whole SYSCON aperture into readback. */
uint32_t flexe_syscon_memory_mmio_read(void *ctx, uint32_t address);
void flexe_syscon_memory_mmio_write(
    void *ctx, uint32_t address, uint32_t value);

bool flexe_syscon_memory_state(
    const flexe_syscon_memory_t *memory,
    flexe_syscon_memory_state_t *state);
void flexe_syscon_memory_set_state_listener(
    flexe_syscon_memory_t *memory, flexe_syscon_memory_state_fn fn,
    void *ctx);

#endif /* FLEXE_SYSCON_MEMORY_H */
