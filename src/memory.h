#ifndef MEMORY_H
#define MEMORY_H

#include <stdint.h>
#include <stddef.h>
#include <string.h>

#ifdef _MSC_VER
#include "msvc_compat.h"
#endif

/* MMIO callback types */
typedef uint32_t (*mmio_read_fn)(void *ctx, uint32_t addr);
typedef void     (*mmio_write_fn)(void *ctx, uint32_t addr, uint32_t val);

/* Constants for struct definition */
#define MEM_PERIPH_PAGES    128         /* 512KB / 4KB */
#define MEM_PAGE_TABLE_SIZE (1u << 20)  /* 1M pages covering 4GB */

typedef struct {
    mmio_read_fn  read;
    mmio_write_fn write;
    void         *ctx;
} mmio_handler_t;

/* Full struct exposed for inline access in hot path */
struct xtensa_mem {
    uint8_t *sram;
    uint8_t *rom;
    uint8_t *flash_data;
    uint8_t *flash_insn;
    uint8_t *rtc_dram;
    uint8_t *rtc_fast;
    uint8_t *rtc_slow;
    uint8_t *psram;
    mmio_handler_t mmio[MEM_PERIPH_PAGES];
    uint8_t *page_table[MEM_PAGE_TABLE_SIZE];
};

typedef struct xtensa_mem xtensa_mem_t;

/* Lifecycle */
xtensa_mem_t *mem_create(void);
void mem_destroy(xtensa_mem_t *mem);
void mem_reset(xtensa_mem_t *mem);

/* Bulk load */
int mem_load(xtensa_mem_t *mem, uint32_t addr, const uint8_t *data, size_t len);
int mem_load_flash(xtensa_mem_t *mem, const uint8_t *data, size_t len);

/* MMIO peripheral registration */
int mem_register_mmio(xtensa_mem_t *mem, int page_index,
                      mmio_read_fn read_fn, mmio_write_fn write_fn, void *ctx);
int mem_register_mmio_range(xtensa_mem_t *mem, uint32_t base, uint32_t size,
                            mmio_read_fn read_fn, mmio_write_fn write_fn, void *ctx);

/* MMIO slow-path functions (called from inline fast paths on page table miss) */
uint32_t mem_read32_slow(xtensa_mem_t *mem, uint32_t addr);
uint16_t mem_read16_slow(xtensa_mem_t *mem, uint32_t addr);
uint8_t  mem_read8_slow(xtensa_mem_t *mem, uint32_t addr);
void     mem_write32_slow(xtensa_mem_t *mem, uint32_t addr, uint32_t val);
void     mem_write16_slow(xtensa_mem_t *mem, uint32_t addr, uint16_t val);
void     mem_write8_slow(xtensa_mem_t *mem, uint32_t addr, uint8_t val);

/*
 * Inline fast-path memory access.
 * Page table hit (~99% of accesses) → single pointer arithmetic + memcpy.
 * Miss → fall through to MMIO slow path in memory.c.
 */

static inline const uint8_t *mem_get_ptr(xtensa_mem_t *mem, uint32_t addr) {
    uint8_t *page = mem->page_table[addr >> 12];
    if (__builtin_expect(page != NULL, 1))
        return page + (addr & 0xFFF);
    return NULL;
}

static inline uint8_t mem_read8(xtensa_mem_t *mem, uint32_t addr) {
    uint8_t *page = mem->page_table[addr >> 12];
    if (__builtin_expect(page != NULL, 1))
        return page[addr & 0xFFF];
    return mem_read8_slow(mem, addr);
}

static inline uint16_t mem_read16(xtensa_mem_t *mem, uint32_t addr) {
    uint8_t *page = mem->page_table[addr >> 12];
    if (__builtin_expect(page != NULL, 1)) {
        uint16_t val;
        memcpy(&val, page + (addr & 0xFFF), 2);
        return val;
    }
    return mem_read16_slow(mem, addr);
}

static inline uint32_t mem_read32(xtensa_mem_t *mem, uint32_t addr) {
    uint8_t *page = mem->page_table[addr >> 12];
    if (__builtin_expect(page != NULL, 1)) {
        uint32_t val;
        memcpy(&val, page + (addr & 0xFFF), 4);
        return val;
    }
    return mem_read32_slow(mem, addr);
}

static inline void mem_write8(xtensa_mem_t *mem, uint32_t addr, uint8_t val) {
    uint8_t *page = mem->page_table[addr >> 12];
    if (__builtin_expect(page != NULL, 1)) {
        page[addr & 0xFFF] = val;
        return;
    }
    mem_write8_slow(mem, addr, val);
}

static inline void mem_write16(xtensa_mem_t *mem, uint32_t addr, uint16_t val) {
    uint8_t *page = mem->page_table[addr >> 12];
    if (__builtin_expect(page != NULL, 1)) {
        memcpy(page + (addr & 0xFFF), &val, 2);
        return;
    }
    mem_write16_slow(mem, addr, val);
}

static inline void mem_write32(xtensa_mem_t *mem, uint32_t addr, uint32_t val) {
    uint8_t *page = mem->page_table[addr >> 12];
    if (__builtin_expect(page != NULL, 1)) {
        memcpy(page + (addr & 0xFFF), &val, 4);
        return;
    }
    mem_write32_slow(mem, addr, val);
}

#endif /* MEMORY_H */
