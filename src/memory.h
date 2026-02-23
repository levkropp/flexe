#ifndef MEMORY_H
#define MEMORY_H

#include <stdint.h>
#include <stddef.h>

typedef struct xtensa_mem xtensa_mem_t;

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

/* Direct pointer access (for instruction fetch) */
const uint8_t *mem_get_ptr(xtensa_mem_t *mem, uint32_t addr);

#endif /* MEMORY_H */
