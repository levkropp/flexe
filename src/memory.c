#include "memory.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

/*
 * ESP32 memory regions:
 *   SRAM:      0x3FFB0000-0x3FFFFFFF (data), 0x40070000-0x400BFFFF (instruction)
 *              Both are aliases for the same 520 KB physical SRAM.
 *   ROM I-bus: 0x40000000-0x4006FFFF (448 KB)
 *   ROM D-bus: 0x3FF90000-0x3FF9FFFF (64 KB)
 *   Flash:     0x3F400000-0x3F7FFFFF (data), 0x400D0000-0x403FFFFF (initial
 *              IRAM0 instruction mapping; upper cache buses are MMU-mapped)
 *   RTC DRAM:  0x3FF80000-0x3FF81FFF (8 KB, data bus alias of RTC Fast)
 *   RTC IRAM:  0x400C0000-0x400C1FFF (8 KB, instruction bus alias of RTC Fast)
 *   RTC Slow:  0x50000000-0x50001FFF (8 KB)
 *   AHB MMIO:  0x60000000-0x6003FFFF (mirror of 0x3FF40000-0x3FF7FFFF)
 *   Periph:    0x3FF00000-0x3FF7FFFF
 */

#define SRAM_SIZE       (704 * 1024)
#define ROM_INSN_SIZE   (448 * 1024)
#define ROM_DATA_SIZE   (64 * 1024)
#define ROM_SIZE        (ROM_INSN_SIZE + ROM_DATA_SIZE)
#define FLASH_SIZE      (4 * 1024 * 1024)
#define RTC_SLOW_SIZE   (8 * 1024)

#define SRAM_DATA_BASE  0x3FFA0000u
#define SRAM_DATA_END   0x40000000u
#define SRAM_INSN_BASE  0x40070000u
#define SRAM_INSN_END   0x400C0000u
#define ROM_DATA_BASE   0x3FF90000u
#define ROM_DATA_END    0x3FFA0000u
#define ROM_BASE        0x40000000u
#define ROM_END         0x40070000u
#define FLASH_DATA_BASE 0x3F400000u
#define FLASH_DATA_END  0x3F800000u
#define FLASH_INSN_BASE 0x400D0000u
#define FLASH_INSN_END  0x40400000u
#define PERIPH_BASE     0x3FF00000u
#define PERIPH_END      0x3FF80000u
#define RTC_DRAM_BASE   0x3FF80000u
#define RTC_DRAM_END    0x3FF82000u
#define RTC_IRAM_BASE   0x400C0000u
#define RTC_IRAM_END    0x400C2000u
#define RTC_DRAM_SIZE   (8 * 1024)
#define RTC_SLOW_BASE   0x50000000u
#define RTC_SLOW_END    0x50002000u

#define AHB_PERIPH_BASE 0x60000000u
#define AHB_PERIPH_END  0x60040000u
#define AHB_TO_APB_BIAS 0x200C0000u

#define PSRAM_BASE      0x3F800000u
#define PSRAM_END       0x3FC00000u
#define PSRAM_SIZE      (4 * 1024 * 1024)

#define PAGE_SIZE    4096

/* Populate page table entries for a contiguous region */
static void page_table_map(xtensa_mem_t *mem, uint32_t base, uint32_t end, uint8_t *host) {
    for (uint32_t page = base; page < end; page += PAGE_SIZE)
        mem->page_table[page >> 12] = host + (page - base);
}

static void page_table_init(xtensa_mem_t *mem) {
    page_table_map(mem, SRAM_DATA_BASE, SRAM_DATA_END, mem->sram);
    page_table_map(mem, SRAM_INSN_BASE, SRAM_INSN_END,
                   mem->sram + (SRAM_DATA_END - SRAM_DATA_BASE));
    page_table_map(mem, ROM_BASE, ROM_END, mem->rom);
    page_table_map(mem, ROM_DATA_BASE, ROM_DATA_END,
                   mem->rom + ROM_INSN_SIZE);
    page_table_map(mem, FLASH_DATA_BASE, FLASH_DATA_END, mem->flash_data);
    page_table_map(mem, FLASH_INSN_BASE, FLASH_INSN_END, mem->flash_insn);
    page_table_map(mem, RTC_DRAM_BASE, RTC_DRAM_END, mem->rtc_dram);
    page_table_map(mem, RTC_IRAM_BASE, RTC_IRAM_END, mem->rtc_dram);
    page_table_map(mem, PSRAM_BASE, PSRAM_END, mem->psram);
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
    mem->rtc_slow   = calloc(1, RTC_SLOW_SIZE);
    mem->psram      = calloc(1, PSRAM_SIZE);

    if (!mem->sram || !mem->rom || !mem->flash_data || !mem->flash_insn ||
        !mem->rtc_dram || !mem->rtc_slow || !mem->psram) {
        mem_destroy(mem);
        return NULL;
    }

    /* NOR flash powers up erased outside the bytes supplied by an image.
     * Factory images are commonly sparse/truncated before later data
     * partitions (SPIFFS, coredump); leaving that tail calloc-zeroed makes
     * programming impossible because NOR writes can only clear bits. */
    memset(mem->flash_data, 0xFF, FLASH_SIZE);
    memset(mem->flash_insn, 0xFF, FLASH_SIZE);

    page_table_init(mem);

    /* Pre-populate the ESP32 ROM spiflash chip struct (ROM BSS, fixed
     * address 0x3FFAE270). On hardware the boot ROM fills this during its
     * flash setup; flexe skips the boot ROM, and firmware built with
     * CONFIG_SPI_FLASH_ROM_IMPL reads rom_spiflash_chip.chip_size from
     * here (spi_flash_mmap validates mappings against it). */
    mem_write32(mem, 0x3FFAE270, 0x00C84016u);  /* device_id: GD25Q32 */
    mem_write32(mem, 0x3FFAE274, 0x00400000u);  /* chip_size: 4 MB */
    mem_write32(mem, 0x3FFAE278, 0x00010000u);  /* block_size: 64 KB */
    mem_write32(mem, 0x3FFAE27C, 0x00001000u);  /* sector_size: 4 KB */
    mem_write32(mem, 0x3FFAE280, 0x00000100u);  /* page_size: 256 B */
    mem_write32(mem, 0x3FFAE284, 0x0000FFFFu);  /* status_mask */

    return mem;
}

void mem_destroy(xtensa_mem_t *mem) {
    if (!mem) return;
    free(mem->sram);
    free(mem->rom);
    free(mem->flash_data);
    free(mem->flash_insn);
    free(mem->rtc_dram);
    free(mem->rtc_slow);
    free(mem->psram);
    free(mem);
}

void mem_reset(xtensa_mem_t *mem) {
    if (!mem) return;
    memset(mem->sram, 0, SRAM_SIZE);
    memset(mem->rtc_dram, 0, RTC_DRAM_SIZE);
    memset(mem->rtc_slow, 0, RTC_SLOW_SIZE);
}

/* The ESP32 exposes the entire 256 KiB APB peripheral window through a
 * second AHB-Lite address window. ESP-IDF uses it for UART FIFO accesses and
 * recent proprietary PHY blobs use it for radio register traffic. Preserve
 * the low 18 address bits so both windows reach exactly the same handlers and
 * register state:
 *
 *     AHB 0x6000_0000..0x6003_FFFF
 *     APB 0x3FF4_0000..0x3FF7_FFFF
 */
static inline uint32_t translate_ahb_alias(uint32_t addr) {
    if (addr >= AHB_PERIPH_BASE && addr < AHB_PERIPH_END)
        return addr - AHB_TO_APB_BIAS;
    return addr;
}

/* MMIO dispatch helper */
static mmio_handler_t *mmio_lookup(xtensa_mem_t *mem, uint32_t addr) {
    addr = translate_ahb_alias(addr);
    if (addr >= PERIPH_BASE && addr < PERIPH_END) {
        int page = (addr - PERIPH_BASE) / PAGE_SIZE;
        mmio_handler_t *h = &mem->mmio[page];
        if (h->read || h->write)
            return h;
    }
    return NULL;
}

/* MMIO slow-path functions (called from inline fast paths on page table miss) */

uint8_t mem_read8_slow(xtensa_mem_t *mem, uint32_t addr) {
    uint32_t xaddr = translate_ahb_alias(addr);
    mmio_handler_t *h = mmio_lookup(mem, xaddr);
    if (h && h->read) return (uint8_t)h->read(h->ctx, xaddr);
    return 0;
}

uint16_t mem_read16_slow(xtensa_mem_t *mem, uint32_t addr) {
    uint32_t xaddr = translate_ahb_alias(addr);
    mmio_handler_t *h = mmio_lookup(mem, xaddr);
    if (h && h->read) return (uint16_t)h->read(h->ctx, xaddr);
    return 0;
}

/* An access that matches no RAM page and no MMIO handler.
 *
 * Reading one returns 0 and writing one is dropped, which is deliberate --
 * firmware probes addresses that a given board does not populate, and
 * faulting on those would be worse than useless. But it also means a *null
 * pointer dereference reads 0 and keeps going*, where hardware would raise
 * LoadProhibited and panic with a backtrace.
 *
 * That difference turns a diagnosable crash into a silent hang. Tasmota
 * 15.6.0 walks a list whose head is NULL, loads from 0+68, gets 0, compares
 * equal to the terminator and loops for ever -- 4 billion cycles with no
 * further output and no indication anything is wrong. Counting these makes
 * the condition visible without changing behaviour. */
static void note_unmapped(xtensa_mem_t *mem, uint32_t addr) {
    if (mem->unmapped_count < UINT64_MAX) mem->unmapped_count++;
    if (mem->unmapped_first == 0u) mem->unmapped_first = addr;
}

uint32_t mem_read32_slow(xtensa_mem_t *mem, uint32_t addr) {
    uint32_t xaddr = translate_ahb_alias(addr);
    mmio_handler_t *h = mmio_lookup(mem, xaddr);
    if (h && h->read) return h->read(h->ctx, xaddr);
    note_unmapped(mem, addr);
    return 0;
}

uint64_t mem_unmapped_count(const xtensa_mem_t *mem) {
    return mem ? mem->unmapped_count : 0u;
}

uint32_t mem_unmapped_first(const xtensa_mem_t *mem) {
    return mem ? mem->unmapped_first : 0u;
}

void mem_write8_slow(xtensa_mem_t *mem, uint32_t addr, uint8_t val) {
    uint32_t xaddr = translate_ahb_alias(addr);
    mmio_handler_t *h = mmio_lookup(mem, xaddr);
    if (h && h->write) h->write(h->ctx, xaddr, val);
}

void mem_write16_slow(xtensa_mem_t *mem, uint32_t addr, uint16_t val) {
    uint32_t xaddr = translate_ahb_alias(addr);
    mmio_handler_t *h = mmio_lookup(mem, xaddr);
    if (h && h->write) h->write(h->ctx, xaddr, val);
}

void mem_write32_slow(xtensa_mem_t *mem, uint32_t addr, uint32_t val) {
    uint32_t xaddr = translate_ahb_alias(addr);
    mmio_handler_t *h = mmio_lookup(mem, xaddr);
    if (h && h->write) h->write(h->ctx, xaddr, val);
}

int mem_load(xtensa_mem_t *mem, uint32_t addr, const uint8_t *data, size_t len) {
    for (size_t i = 0; i < len; i++) {
        uint8_t *page = mem->page_table[(addr + (uint32_t)i) >> 12];
        if (!page) return -1;
        page[(addr + (uint32_t)i) & 0xFFF] = data[i];
    }
    return 0;
}

int mem_load_flash(xtensa_mem_t *mem, const uint8_t *data, size_t len) {
    if (!mem || !data) return -1;
    if (len > FLASH_SIZE) len = FLASH_SIZE;
    memcpy(mem->flash_data, data, len);
    memcpy(mem->flash_insn, data, len);
    return 0;
}

int mem_register_mmio(xtensa_mem_t *mem, int page_index,
                      mmio_read_fn read_fn, mmio_write_fn write_fn, void *ctx) {
    if (!mem || page_index < 0 || page_index >= MEM_PERIPH_PAGES)
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

/* ===== Speculative-write journal (see memory.h) ===== */
int      g_mem_journal_en = 0;
int      g_mem_journal_unsafe = 0;
int      g_mem_journal_count = 0;
mem_journal_entry_t g_mem_journal[MEM_JOURNAL_MAX];

void mem_journal_begin(void) {
    g_mem_journal_count = 0;
    g_mem_journal_unsafe = 0;
    g_mem_journal_en = 1;
}

/* Undo in reverse so the earliest recorded value for a repeatedly-written
 * word wins. Writes go straight to the page: the journal only ever holds RAM
 * addresses, and routing through mem_write32 would re-enter the journal. */
void mem_journal_rollback(xtensa_mem_t *mem) {
    for (int i = g_mem_journal_count - 1; i >= 0; i--) {
        uint32_t addr = g_mem_journal[i].addr;
        uint8_t *page = mem->page_table[addr >> 12];
        if (page) memcpy(page + (addr & 0xFFF), &g_mem_journal[i].old, 4);
    }
    g_mem_journal_count = 0;
}

void mem_journal_end(void) {
    g_mem_journal_en = 0;
    g_mem_journal_count = 0;
}

/* Watchpoint reporting for mem_write32. Reached only when g_dbg_mem_watch is
 * armed, so the individual arm-tests can be re-checked here at no hot-path
 * cost. Kept out of line to keep the store path a single predicted branch. */
int g_dbg_mem_watch;

void mem_watch_report32(xtensa_mem_t *mem, uint32_t addr, uint32_t val) {
    if (g_dbg_watch_en &&
        (addr == g_dbg_watch_addr || addr == g_dbg_watch_addr2)) {
        uint32_t old = mem_read32(mem, addr);
        fprintf(stderr, "[W] 0x%08X: 0x%08X -> 0x%08X  pc=0x%08X core%d\n",
                addr, old, val, g_dbg_pc, g_dbg_core);
    }
    if (g_dbg_watch_val && val == g_dbg_watch_val) {
        fprintf(stderr, "[WV] 0x%08X <- 0x%08X  pc=0x%08X core%d\n",
                addr, val, g_dbg_pc, g_dbg_core);
    }
    if (g_dbg_pcwatch_en &&
        (g_dbg_pc == g_dbg_pcwatch || g_dbg_pc == g_dbg_pcwatch2)) {
        fprintf(stderr, "[PW] pc=0x%08X store 0x%08X <- 0x%08X core%d\n",
                g_dbg_pc, addr, val, g_dbg_core);
    }
}

/* Out-of-line stores for the JIT's verification mode.
 *
 * A compiled block normally stores through inline page-table code, which the
 * write journal cannot observe. Routing its stores through these instead
 * makes a native block's memory effects replayable, which is what lets the
 * verifier run the block first, undo it, and then compare it against the
 * interpreter from the identical starting state. Only verification pays for
 * the call. */
void mem_write8_journaled(xtensa_mem_t *mem, uint32_t addr, uint8_t val) {
    mem_write8(mem, addr, val);
}
void mem_write16_journaled(xtensa_mem_t *mem, uint32_t addr, uint16_t val) {
    mem_write16(mem, addr, val);
}
void mem_write32_journaled(xtensa_mem_t *mem, uint32_t addr, uint32_t val) {
    mem_write32(mem, addr, val);
}
