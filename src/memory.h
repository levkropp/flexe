#ifndef MEMORY_H
#define MEMORY_H

#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>

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

/* Resolve guest address to host pointer (read-only).
 * Returns NULL for MMIO or unmapped addresses. */
static inline const uint8_t *mem_get_ptr(xtensa_mem_t *mem, uint32_t addr) {
    uint8_t *page = mem->page_table[addr >> 12];
    if (__builtin_expect(page != NULL, 1))
        return page + (addr & 0xFFF);
    return NULL;
}

/* Resolve guest address to writable host pointer.
 * Returns NULL for MMIO or unmapped addresses. */
static inline uint8_t *mem_get_ptr_w(xtensa_mem_t *mem, uint32_t addr) {
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


/* ===== Speculative-write journal =====
 *
 * Records the pre-write contents of every RAM word a run touches so the run
 * can be undone. The JIT's differential verify mode needs this: it has to
 * execute the same guest instructions twice -- once natively, once
 * interpreted -- and a block containing a read-modify-write would otherwise
 * apply its update twice.
 *
 * MMIO is deliberately *not* journalled. Reading a device register to save it
 * can itself have side effects, and replaying a device write is not
 * meaningful, so any MMIO touch marks the window unreplayable and the
 * verifier skips that block rather than guessing.
 *
 * Entirely inert when disabled: one predicted-not-taken test per store.
 */
#define MEM_JOURNAL_MAX 8192
typedef struct { uint32_t addr; uint32_t old; } mem_journal_entry_t;
extern int      g_mem_journal_en;       /* 1 = recording */
extern int      g_mem_journal_unsafe;   /* MMIO touched, or capacity exceeded */
extern int      g_mem_journal_count;
extern mem_journal_entry_t g_mem_journal[MEM_JOURNAL_MAX];

void mem_journal_begin(void);
void mem_journal_rollback(xtensa_mem_t *mem);
void mem_journal_end(void);

static inline void mem_journal_word(xtensa_mem_t *mem, uint32_t addr) {
    uint32_t w = addr & ~3u;
    uint8_t *page = mem->page_table[w >> 12];
    if (!page) { g_mem_journal_unsafe = 1; return; }
    if (g_mem_journal_count >= MEM_JOURNAL_MAX) {
        g_mem_journal_unsafe = 1;
        return;
    }
    mem_journal_entry_t *e = &g_mem_journal[g_mem_journal_count++];
    e->addr = w;
    memcpy(&e->old, page + (w & 0xFFF), 4);
}

/* Journal every word an access of `len` bytes at `addr` can reach. */
static inline void mem_journal_note(xtensa_mem_t *mem, uint32_t addr,
                                    unsigned len) {
    mem_journal_word(mem, addr);
    uint32_t last = addr + len - 1u;
    if ((last & ~3u) != (addr & ~3u)) mem_journal_word(mem, last);
}

static inline void mem_write8(xtensa_mem_t *mem, uint32_t addr, uint8_t val) {
    if (__builtin_expect(g_mem_journal_en, 0)) mem_journal_note(mem, addr, 1);
    uint8_t *page = mem->page_table[addr >> 12];
    if (__builtin_expect(page != NULL, 1)) {
        page[addr & 0xFFF] = val;
        return;
    }
    mem_write8_slow(mem, addr, val);
}

static inline void mem_write16(xtensa_mem_t *mem, uint32_t addr, uint16_t val) {
    if (__builtin_expect(g_mem_journal_en, 0)) mem_journal_note(mem, addr, 2);
    uint8_t *page = mem->page_table[addr >> 12];
    if (__builtin_expect(page != NULL, 1)) {
        memcpy(page + (addr & 0xFFF), &val, 2);
        return;
    }
    mem_write16_slow(mem, addr, val);
}

extern uint32_t g_dbg_pc;
extern int g_dbg_core;
extern int g_dbg_watch_en;
extern uint32_t g_dbg_watch_addr;
extern int g_dbg_pcwatch_en;
extern uint32_t g_dbg_pcwatch;
extern uint32_t g_dbg_pcwatch2;
extern uint32_t g_dbg_watch_addr2;
extern uint32_t g_dbg_watch_val;

static inline void mem_write32(xtensa_mem_t *mem, uint32_t addr, uint32_t val) {
    /* TEMP DEBUG: watch writes to up to two addresses (env FLEXE_WATCH/2) */
    if (__builtin_expect(g_dbg_watch_en &&
        (addr == g_dbg_watch_addr || addr == g_dbg_watch_addr2), 0)) {
        uint32_t old = mem_read32(mem, addr);
        fprintf(stderr, "[W] 0x%08X: 0x%08X -> 0x%08X  pc=0x%08X core%d\n",
                addr, old, val, g_dbg_pc, g_dbg_core);
    }
    if (__builtin_expect(g_dbg_watch_val && val == g_dbg_watch_val, 0)) {
        fprintf(stderr, "[WV] 0x%08X <- 0x%08X  pc=0x%08X core%d\n",
                addr, val, g_dbg_pc, g_dbg_core);
    }
    /* TEMP DEBUG: log stores issued from specific PCs (env FLEXE_PCWATCH/2) */
    if (__builtin_expect(g_dbg_pcwatch_en &&
        (g_dbg_pc == g_dbg_pcwatch || g_dbg_pc == g_dbg_pcwatch2), 0)) {
        fprintf(stderr, "[PW] pc=0x%08X store 0x%08X <- 0x%08X core%d\n",
                g_dbg_pc, addr, val, g_dbg_core);
    }
    if (__builtin_expect(g_mem_journal_en, 0)) mem_journal_note(mem, addr, 4);
    uint8_t *page = mem->page_table[addr >> 12];
    if (__builtin_expect(page != NULL, 1)) {
        memcpy(page + (addr & 0xFFF), &val, 4);
        return;
    }
    mem_write32_slow(mem, addr, val);
}

#endif /* MEMORY_H */
