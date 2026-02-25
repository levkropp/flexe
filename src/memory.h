#ifndef MEMORY_H
#define MEMORY_H

#include <stdint.h>
#include <stddef.h>

typedef struct xtensa_mem xtensa_mem_t;

/* MMIO callback types */
typedef uint32_t (*mmio_read_fn)(void *ctx, uint32_t addr);
typedef void     (*mmio_write_fn)(void *ctx, uint32_t addr, uint32_t val);

xtensa_mem_t *mem_create(void);
void mem_destroy(xtensa_mem_t *mem);
void mem_reset(xtensa_mem_t *mem);

uint8_t  mem_read8(xtensa_mem_t *mem, uint32_t addr);
uint16_t mem_read16(xtensa_mem_t *mem, uint32_t addr);
uint32_t mem_read32(xtensa_mem_t *mem, uint32_t addr);

void mem_write8(xtensa_mem_t *mem, uint32_t addr, uint8_t val);
void mem_write16(xtensa_mem_t *mem, uint32_t addr, uint16_t val);
void mem_write32(xtensa_mem_t *mem, uint32_t addr, uint32_t val);

/* Bulk load (for firmware loading) */
int mem_load(xtensa_mem_t *mem, uint32_t addr, const uint8_t *data, size_t len);

/* Bulk load raw flash image (offset 0 = flash base) */
int mem_load_flash(xtensa_mem_t *mem, const uint8_t *data, size_t len);

/* Direct pointer access (for instruction fetch) */
const uint8_t *mem_get_ptr(xtensa_mem_t *mem, uint32_t addr);

/* MMIO peripheral registration */
int mem_register_mmio(xtensa_mem_t *mem, int page_index,
                      mmio_read_fn read_fn, mmio_write_fn write_fn, void *ctx);
int mem_register_mmio_range(xtensa_mem_t *mem, uint32_t base, uint32_t size,
                            mmio_read_fn read_fn, mmio_write_fn write_fn, void *ctx);

#endif /* MEMORY_H */
