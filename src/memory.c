#include "memory.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

/*
 * ESP32 memory regions:
 *   SRAM:      0x3FFB0000-0x3FFFFFFF (data), 0x40070000-0x400BFFFF (instruction)
 *              Both are aliases for the same 520 KB physical SRAM.
 *   ROM:       0x40000000-0x4005FFFF (384 KB)
 *   Flash:     0x3F400000-0x3F7FFFFF (data), 0x400C2000-0x40BFFFFF (instruction)
 *   RTC DRAM:  0x3FF80000-0x3FF81FFF (8 KB, data bus alias of RTC Fast)
 *   RTC IRAM:  0x400C0000-0x400C1FFF (8 KB, instruction bus alias of RTC Fast)
 *   RTC Fast:  0x50000000-0x50001FFF (8 KB)
 *   RTC Slow:  0x60000000-0x60001FFF (8 KB)
 *   Periph:    0x3FF00000-0x3FF7FFFF
 */

#define SRAM_SIZE       (704 * 1024)        /* 384 KB data bus + 320 KB insn bus */
#define ROM_SIZE        (384 * 1024)        /* 384 KB internal ROM */
#define FLASH_SIZE      (4 * 1024 * 1024)   /* 4 MB flash (expandable) */
#define RTC_FAST_SIZE   (8 * 1024)          /* 8 KB RTC FAST */
#define RTC_SLOW_SIZE   (8 * 1024)          /* 8 KB RTC SLOW */

/* Base addresses */
#define SRAM_DATA_BASE  0x3FFA0000u   /* includes SRAM2 (0x3FFAE000+) */
#define SRAM_DATA_END   0x40000000u
#define SRAM_INSN_BASE  0x40070000u
#define SRAM_INSN_END   0x400C0000u
#define ROM_BASE        0x40000000u
#define ROM_END         0x40060000u
#define FLASH_DATA_BASE 0x3F400000u
#define FLASH_DATA_END  0x3F800000u
#define FLASH_INSN_BASE 0x400C2000u
#define FLASH_INSN_END  0x40C00000u
#define PERIPH_BASE     0x3FF00000u
#define PERIPH_END      0x3FF80000u
#define RTC_DRAM_BASE   0x3FF80000u   /* RTC Fast Memory (D-bus alias) */
#define RTC_DRAM_END    0x3FF82000u
#define RTC_IRAM_BASE   0x400C0000u   /* RTC Fast Memory (I-bus alias) */
#define RTC_IRAM_END    0x400C2000u
#define RTC_DRAM_SIZE   (8 * 1024)
#define RTC_FAST_BASE   0x50000000u
#define RTC_FAST_END    0x50002000u
#define RTC_SLOW_BASE   0x60000000u
#define RTC_SLOW_END    0x60002000u

/* PSRAM (external SPI RAM): ESP32 maps at 0x3F800000 */
#define PSRAM_BASE      0x3F800000u
#define PSRAM_END       0x3FC00000u
#define PSRAM_SIZE      (4 * 1024 * 1024)

#define PERIPH_PAGES 128  /* 512KB / 4KB */
#define PAGE_SIZE    4096

typedef struct {
    mmio_read_fn  read;
    mmio_write_fn write;
    void         *ctx;
} mmio_handler_t;

/* Page table: 1M entries covering full 32-bit address space, indexed by addr >> 12.
 * Each entry points to the start of the corresponding 4KB page in host memory.
 * NULL entries indicate MMIO or unmapped pages.
 * Lookup: page_table[addr >> 12] + (addr & 0xFFF) → host pointer */
#define PAGE_TABLE_SIZE (1u << 20)  /* 1M pages */

struct xtensa_mem {
    uint8_t *sram;        /* Internal SRAM (data + instruction alias) */
    uint8_t *rom;         /* Internal ROM */
    uint8_t *flash_data;  /* Flash data cache (0x3F400000), separate from insn */
    uint8_t *flash_insn;  /* Flash instruction cache (0x400C2000), separate from data */
    uint8_t *rtc_dram;    /* RTC Fast Memory (D-bus + I-bus alias) */
    uint8_t *rtc_fast;    /* RTC FAST memory (0x50000000) */
    uint8_t *rtc_slow;    /* RTC SLOW memory */
    uint8_t *psram;       /* External PSRAM (SPI RAM) */
    mmio_handler_t mmio[PERIPH_PAGES];
    uint8_t *page_table[PAGE_TABLE_SIZE];
};

/* Populate page table entries for a contiguous region */
static void page_table_map(xtensa_mem_t *mem, uint32_t base, uint32_t end, uint8_t *host) {
    for (uint32_t page = base; page < end; page += PAGE_SIZE)
        mem->page_table[page >> 12] = host + (page - base);
}

static void page_table_init(xtensa_mem_t *mem) {
    /* SRAM data bus: 0x3FFA0000-0x3FFFFFFF */
    page_table_map(mem, SRAM_DATA_BASE, SRAM_DATA_END, mem->sram);

    /* SRAM instruction bus: 0x40070000-0x400BFFFF (alias, offset into same sram) */
    page_table_map(mem, SRAM_INSN_BASE, SRAM_INSN_END,
                   mem->sram + (SRAM_DATA_END - SRAM_DATA_BASE));

    /* ROM: 0x40000000-0x4005FFFF */
    page_table_map(mem, ROM_BASE, ROM_END, mem->rom);

    /* Flash data bus: 0x3F400000-0x3F7FFFFF */
    page_table_map(mem, FLASH_DATA_BASE, FLASH_DATA_END, mem->flash_data);

    /* Flash instruction bus: 0x400C2000-0x40BFFFFF */
    page_table_map(mem, FLASH_INSN_BASE, FLASH_INSN_END, mem->flash_insn);

    /* RTC DRAM (D-bus alias): 0x3FF80000-0x3FF81FFF */
    page_table_map(mem, RTC_DRAM_BASE, RTC_DRAM_END, mem->rtc_dram);

    /* RTC IRAM (I-bus alias of same memory): 0x400C0000-0x400C1FFF */
    page_table_map(mem, RTC_IRAM_BASE, RTC_IRAM_END, mem->rtc_dram);

    /* PSRAM: 0x3F800000-0x3FBFFFFF */
    page_table_map(mem, PSRAM_BASE, PSRAM_END, mem->psram);

    /* RTC FAST: 0x50000000-0x50001FFF */
    page_table_map(mem, RTC_FAST_BASE, RTC_FAST_END, mem->rtc_fast);

    /* RTC SLOW: 0x60000000-0x60001FFF */
    page_table_map(mem, RTC_SLOW_BASE, RTC_SLOW_END, mem->rtc_slow);
}

xtensa_mem_t *mem_create(void) {
    xtensa_mem_t *mem = calloc(1, sizeof(xtensa_mem_t));
    if (!mem) return NULL;

    mem->sram       = calloc(1, SRAM_SIZE);
    mem->rom        = calloc(1, ROM_SIZE);
    mem->flash_data = calloc(1, FLASH_SIZE);
    mem->flash_insn = calloc(1, FLASH_SIZE);
    mem->rtc_dram   = calloc(1, RTC_DRAM_SIZE);
    mem->rtc_fast   = calloc(1, RTC_FAST_SIZE);
    mem->rtc_slow   = calloc(1, RTC_SLOW_SIZE);
    mem->psram      = calloc(1, PSRAM_SIZE);

    if (!mem->sram || !mem->rom || !mem->flash_data || !mem->flash_insn ||
        !mem->rtc_dram || !mem->rtc_fast || !mem->rtc_slow || !mem->psram) {
        mem_destroy(mem);
        return NULL;
    }

    page_table_init(mem);

    return mem;
}

void mem_destroy(xtensa_mem_t *mem) {
    if (!mem) return;
    free(mem->sram);
    free(mem->rom);
    free(mem->flash_data);
    free(mem->flash_insn);
    free(mem->rtc_dram);
    free(mem->rtc_fast);
    free(mem->rtc_slow);
    free(mem->psram);
    free(mem);
}

void mem_reset(xtensa_mem_t *mem) {
    if (!mem) return;
    memset(mem->sram, 0, SRAM_SIZE);
    /* ROM and flash are not cleared on reset */
    memset(mem->rtc_dram, 0, RTC_DRAM_SIZE);
    memset(mem->rtc_fast, 0, RTC_FAST_SIZE);
    memset(mem->rtc_slow, 0, RTC_SLOW_SIZE);
}

/*
 * Address translation: ESP32 address -> host pointer
 * Uses flat page table for O(1) lookup.
 * Returns NULL for unmapped or peripheral (MMIO) addresses.
 */
static inline uint8_t *mem_translate(xtensa_mem_t *mem, uint32_t addr) {
    uint8_t *page = mem->page_table[addr >> 12];
    if (page) return page + (addr & 0xFFF);
    return NULL;
}

/* MMIO dispatch helper: returns handler for peripheral address, or NULL */
static mmio_handler_t *mmio_lookup(xtensa_mem_t *mem, uint32_t addr) {
    if (addr >= PERIPH_BASE && addr < PERIPH_END) {
        int page = (addr - PERIPH_BASE) / PAGE_SIZE;
        mmio_handler_t *h = &mem->mmio[page];
        if (h->read || h->write)
            return h;
    }
    return NULL;
}

uint8_t mem_read8(xtensa_mem_t *mem, uint32_t addr) {
    uint8_t *ptr = mem_translate(mem, addr);
    if (ptr) return *ptr;
    mmio_handler_t *h = mmio_lookup(mem, addr);
    if (h && h->read) return (uint8_t)h->read(h->ctx, addr);
    return 0;
}

uint16_t mem_read16(xtensa_mem_t *mem, uint32_t addr) {
    uint8_t *ptr = mem_translate(mem, addr);
    if (ptr) return (uint16_t)(ptr[0] | (ptr[1] << 8));
    mmio_handler_t *h = mmio_lookup(mem, addr);
    if (h && h->read) return (uint16_t)h->read(h->ctx, addr);
    return 0;
}

uint32_t mem_read32(xtensa_mem_t *mem, uint32_t addr) {
    uint8_t *ptr = mem_translate(mem, addr);
    if (ptr) return (uint32_t)(ptr[0] | (ptr[1] << 8) | (ptr[2] << 16) | (ptr[3] << 24));
    mmio_handler_t *h = mmio_lookup(mem, addr);
    if (h && h->read) return h->read(h->ctx, addr);
    return 0;
}

void mem_write8(xtensa_mem_t *mem, uint32_t addr, uint8_t val) {
    uint8_t *ptr = mem_translate(mem, addr);
    if (ptr) { *ptr = val; return; }
    mmio_handler_t *h = mmio_lookup(mem, addr);
    if (h && h->write) h->write(h->ctx, addr, val);
}

void mem_write16(xtensa_mem_t *mem, uint32_t addr, uint16_t val) {
    uint8_t *ptr = mem_translate(mem, addr);
    if (ptr) { ptr[0] = (uint8_t)(val & 0xFF); ptr[1] = (uint8_t)((val >> 8) & 0xFF); return; }
    mmio_handler_t *h = mmio_lookup(mem, addr);
    if (h && h->write) h->write(h->ctx, addr, val);
}

void mem_write32(xtensa_mem_t *mem, uint32_t addr, uint32_t val) {
    uint8_t *ptr = mem_translate(mem, addr);
    if (ptr) {
        ptr[0] = (uint8_t)(val & 0xFF);
        ptr[1] = (uint8_t)((val >> 8) & 0xFF);
        ptr[2] = (uint8_t)((val >> 16) & 0xFF);
        ptr[3] = (uint8_t)((val >> 24) & 0xFF);
        return;
    }
    mmio_handler_t *h = mmio_lookup(mem, addr);
    if (h && h->write) h->write(h->ctx, addr, val);
}

int mem_load(xtensa_mem_t *mem, uint32_t addr, const uint8_t *data, size_t len) {
    for (size_t i = 0; i < len; i++) {
        uint8_t *ptr = mem_translate(mem, addr + (uint32_t)i);
        if (!ptr) return -1;
        *ptr = data[i];
    }
    return 0;
}

int mem_load_flash(xtensa_mem_t *mem, const uint8_t *data, size_t len) {
    if (!mem || !data) return -1;
    if (len > FLASH_SIZE) len = FLASH_SIZE;
    /* Load raw flash into both data and instruction arrays.
     * Segment loading will overwrite the correct regions later. */
    memcpy(mem->flash_data, data, len);
    memcpy(mem->flash_insn, data, len);
    return 0;
}

const uint8_t *mem_get_ptr(xtensa_mem_t *mem, uint32_t addr) {
    return mem_translate(mem, addr);
}

int mem_register_mmio(xtensa_mem_t *mem, int page_index,
                      mmio_read_fn read_fn, mmio_write_fn write_fn, void *ctx) {
    if (!mem || page_index < 0 || page_index >= PERIPH_PAGES)
        return -1;
    mem->mmio[page_index].read  = read_fn;
    mem->mmio[page_index].write = write_fn;
    mem->mmio[page_index].ctx   = ctx;
    return 0;
}

int mem_register_mmio_range(xtensa_mem_t *mem, uint32_t base, uint32_t size,
                            mmio_read_fn read_fn, mmio_write_fn write_fn, void *ctx) {
    if (!mem || base < PERIPH_BASE || base + size > PERIPH_END)
        return -1;
    int start_page = (base - PERIPH_BASE) / PAGE_SIZE;
    int num_pages  = (size + PAGE_SIZE - 1) / PAGE_SIZE;
    for (int i = 0; i < num_pages; i++)
        mem_register_mmio(mem, start_page + i, read_fn, write_fn, ctx);
    return 0;
}
