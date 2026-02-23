#include "memory.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

/*
 * ESP32 memory regions:
 *   SRAM:  0x3FFB0000-0x3FFFFFFF (data), 0x40070000-0x400C1FFF (instruction)
 *          Both are aliases for the same 520 KB physical SRAM.
 *   ROM:   0x40000000-0x4005FFFF (384 KB)
 *   Flash: 0x3F400000-0x3F7FFFFF (data), 0x400C2000-0x40BFFFFF (instruction)
 *   RTC:   0x50000000-0x50001FFF (fast), 0x60000000-0x60001FFF (slow)
 *   Periph:0x3FF00000-0x3FF7FFFF
 */

#define SRAM_SIZE       (520 * 1024)        /* 520 KB internal SRAM */
#define ROM_SIZE        (384 * 1024)        /* 384 KB internal ROM */
#define FLASH_SIZE      (4 * 1024 * 1024)   /* 4 MB flash (expandable) */
#define RTC_FAST_SIZE   (8 * 1024)          /* 8 KB RTC FAST */
#define RTC_SLOW_SIZE   (8 * 1024)          /* 8 KB RTC SLOW */

/* Base addresses */
#define SRAM_DATA_BASE  0x3FFB0000u
#define SRAM_DATA_END   0x40000000u
#define SRAM_INSN_BASE  0x40070000u
#define SRAM_INSN_END   0x400C2000u
#define ROM_BASE        0x40000000u
#define ROM_END         0x40060000u
#define FLASH_DATA_BASE 0x3F400000u
#define FLASH_DATA_END  0x3F800000u
#define FLASH_INSN_BASE 0x400C2000u
#define FLASH_INSN_END  0x40C00000u
#define PERIPH_BASE     0x3FF00000u
#define PERIPH_END      0x3FF80000u
#define RTC_FAST_BASE   0x50000000u
#define RTC_FAST_END    0x50002000u
#define RTC_SLOW_BASE   0x60000000u
#define RTC_SLOW_END    0x60002000u

struct xtensa_mem {
    uint8_t *sram;      /* Internal SRAM (data + instruction alias) */
    uint8_t *rom;       /* Internal ROM */
    uint8_t *flash;     /* External flash image */
    uint8_t *rtc_fast;  /* RTC FAST memory */
    uint8_t *rtc_slow;  /* RTC SLOW memory */
};

xtensa_mem_t *mem_create(void) {
    xtensa_mem_t *mem = calloc(1, sizeof(xtensa_mem_t));
    if (!mem) return NULL;

    mem->sram     = calloc(1, SRAM_SIZE);
    mem->rom      = calloc(1, ROM_SIZE);
    mem->flash    = calloc(1, FLASH_SIZE);
    mem->rtc_fast = calloc(1, RTC_FAST_SIZE);
    mem->rtc_slow = calloc(1, RTC_SLOW_SIZE);

    if (!mem->sram || !mem->rom || !mem->flash || !mem->rtc_fast || !mem->rtc_slow) {
        mem_destroy(mem);
        return NULL;
    }

    return mem;
}

void mem_destroy(xtensa_mem_t *mem) {
    if (!mem) return;
    free(mem->sram);
    free(mem->rom);
    free(mem->flash);
    free(mem->rtc_fast);
    free(mem->rtc_slow);
    free(mem);
}

void mem_reset(xtensa_mem_t *mem) {
    if (!mem) return;
    memset(mem->sram, 0, SRAM_SIZE);
    /* ROM and flash are not cleared on reset */
    memset(mem->rtc_fast, 0, RTC_FAST_SIZE);
    memset(mem->rtc_slow, 0, RTC_SLOW_SIZE);
}

/*
 * Address translation: ESP32 address -> host pointer
 * Returns NULL for unmapped or peripheral addresses.
 */
static uint8_t *mem_translate(xtensa_mem_t *mem, uint32_t addr) {
    /* SRAM data bus: 0x3FFB0000-0x3FFFFFFF */
    if (addr >= SRAM_DATA_BASE && addr < SRAM_DATA_END)
        return mem->sram + (addr - SRAM_DATA_BASE);

    /* SRAM instruction bus: 0x40070000-0x400C1FFF (alias) */
    if (addr >= SRAM_INSN_BASE && addr < SRAM_INSN_END)
        return mem->sram + (addr - SRAM_INSN_BASE);

    /* ROM: 0x40000000-0x4005FFFF */
    if (addr >= ROM_BASE && addr < ROM_END)
        return mem->rom + (addr - ROM_BASE);

    /* Flash data bus: 0x3F400000-0x3F7FFFFF */
    if (addr >= FLASH_DATA_BASE && addr < FLASH_DATA_END)
        return mem->flash + (addr - FLASH_DATA_BASE);

    /* Flash instruction bus: 0x400C2000-0x40BFFFFF */
    if (addr >= FLASH_INSN_BASE && addr < FLASH_INSN_END)
        return mem->flash + (addr - FLASH_INSN_BASE);

    /* RTC FAST: 0x50000000-0x50001FFF */
    if (addr >= RTC_FAST_BASE && addr < RTC_FAST_END)
        return mem->rtc_fast + (addr - RTC_FAST_BASE);

    /* RTC SLOW: 0x60000000-0x60001FFF */
    if (addr >= RTC_SLOW_BASE && addr < RTC_SLOW_END)
        return mem->rtc_slow + (addr - RTC_SLOW_BASE);

    /* Peripheral region: return NULL (handled separately) */
    if (addr >= PERIPH_BASE && addr < PERIPH_END)
        return NULL;

    return NULL;  /* Unmapped */
}

uint8_t mem_read8(xtensa_mem_t *mem, uint32_t addr) {
    uint8_t *ptr = mem_translate(mem, addr);
    if (!ptr) return 0;
    return *ptr;
}

uint16_t mem_read16(xtensa_mem_t *mem, uint32_t addr) {
    uint8_t *ptr = mem_translate(mem, addr);
    if (!ptr) return 0;
    /* Little-endian */
    return (uint16_t)(ptr[0] | (ptr[1] << 8));
}

uint32_t mem_read32(xtensa_mem_t *mem, uint32_t addr) {
    uint8_t *ptr = mem_translate(mem, addr);
    if (!ptr) return 0;
    /* Little-endian */
    return (uint32_t)(ptr[0] | (ptr[1] << 8) | (ptr[2] << 16) | (ptr[3] << 24));
}

void mem_write8(xtensa_mem_t *mem, uint32_t addr, uint8_t val) {
    uint8_t *ptr = mem_translate(mem, addr);
    if (!ptr) return;
    *ptr = val;
}

void mem_write16(xtensa_mem_t *mem, uint32_t addr, uint16_t val) {
    uint8_t *ptr = mem_translate(mem, addr);
    if (!ptr) return;
    ptr[0] = (uint8_t)(val & 0xFF);
    ptr[1] = (uint8_t)((val >> 8) & 0xFF);
}

void mem_write32(xtensa_mem_t *mem, uint32_t addr, uint32_t val) {
    uint8_t *ptr = mem_translate(mem, addr);
    if (!ptr) return;
    ptr[0] = (uint8_t)(val & 0xFF);
    ptr[1] = (uint8_t)((val >> 8) & 0xFF);
    ptr[2] = (uint8_t)((val >> 16) & 0xFF);
    ptr[3] = (uint8_t)((val >> 24) & 0xFF);
}

int mem_load(xtensa_mem_t *mem, uint32_t addr, const uint8_t *data, size_t len) {
    for (size_t i = 0; i < len; i++) {
        uint8_t *ptr = mem_translate(mem, addr + (uint32_t)i);
        if (!ptr) return -1;
        *ptr = data[i];
    }
    return 0;
}

const uint8_t *mem_get_ptr(xtensa_mem_t *mem, uint32_t addr) {
    return mem_translate(mem, addr);
}
