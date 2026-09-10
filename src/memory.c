#include "memory.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#define ROM_CTYPE_PTR   0x3FF96350u
#define ROM_CTYPE_TABLE 0x3FF96354u

#define PAGE_SIZE    4096

static uint8_t **backing_slot(xtensa_mem_t *mem,
                              flexe_mem_backing_t backing) {
    switch (backing) {
    case FLEXE_MEM_SRAM:       return &mem->sram;
    case FLEXE_MEM_ROM:        return &mem->rom;
    case FLEXE_MEM_FLASH_DATA: return &mem->flash_data;
    case FLEXE_MEM_FLASH_INSN: return &mem->flash_insn;
    case FLEXE_MEM_RTC_FAST:   return &mem->rtc_dram;
    case FLEXE_MEM_RTC_SLOW:   return &mem->rtc_slow;
    case FLEXE_MEM_PSRAM:      return &mem->psram;
    default:                   return NULL;
    }
}

/* Populate page table entries for a contiguous region */
static void page_table_map(xtensa_mem_t *mem, uint32_t base, uint32_t end,
                           uint8_t *host) {
    for (uint32_t page = base; page < end; page += PAGE_SIZE)
        mem->page_table[page >> 12] = host + (page - base);
}

static int page_table_init(xtensa_mem_t *mem) {
    const flexe_target_desc_t *target = mem->target;
    if (!target || target->memory_region_count > FLEXE_TARGET_MEM_REGION_MAX)
        return -1;
    for (unsigned i = 0; i < target->memory_region_count; i++) {
        const flexe_target_mem_region_t *region = &target->memory_region[i];
        uint8_t **slot = backing_slot(mem, region->backing);
        uint32_t size;
        if (!slot || !*slot || region->end <= region->start ||
            (region->start & (PAGE_SIZE - 1u)) != 0 ||
            (region->end & (PAGE_SIZE - 1u)) != 0 ||
            (region->backing_offset & (PAGE_SIZE - 1u)) != 0)
            return -1;
        size = region->end - region->start;
        if (region->backing_offset > mem->backing_size[region->backing] ||
            size > mem->backing_size[region->backing] - region->backing_offset)
            return -1;
        page_table_map(mem, region->start, region->end,
                       *slot + region->backing_offset);
    }
    return 0;
}

/* ESP32 rev3's newlib routines execute partly from IRAM but still consult
 * immutable data in the ROM D-bus window.  In particular, String::trim() and
 * strtol() reach __ctype_ptr__/_ctype_ even when all callable ROM routines are
 * replaced by Flexe stubs.  A zero-filled ROM therefore makes whitespace stop
 * being whitespace and breaks ordinary HTTP parsing.
 *
 * Seed this small architectural dependency for runs without a ROM ELF.  An
 * official ROM ELF is loaded after mem_create() and overwrites these bytes
 * with the same values. */
static uint8_t esp32_rom_ctype(unsigned ch) {
    enum {
        CTYPE_UPPER = 0x01,
        CTYPE_LOWER = 0x02,
        CTYPE_DIGIT = 0x04,
        CTYPE_SPACE = 0x08,
        CTYPE_PUNCT = 0x10,
        CTYPE_CNTRL = 0x20,
        CTYPE_HEX   = 0x40,
        CTYPE_BLANK = 0x80,
    };

    if (ch < 0x20u)
        return (uint8_t)(CTYPE_CNTRL |
                         (ch >= '\t' && ch <= '\r' ? CTYPE_SPACE : 0));
    if (ch == ' ') return CTYPE_SPACE | CTYPE_BLANK;
    if (ch >= '0' && ch <= '9') return CTYPE_DIGIT;
    if (ch >= 'A' && ch <= 'Z')
        return (uint8_t)(CTYPE_UPPER | (ch <= 'F' ? CTYPE_HEX : 0));
    if (ch >= 'a' && ch <= 'z')
        return (uint8_t)(CTYPE_LOWER | (ch <= 'f' ? CTYPE_HEX : 0));
    if (ch >= 0x21u && ch <= 0x7eu) return CTYPE_PUNCT;
    if (ch == 0x7fu) return CTYPE_CNTRL;
    return 0;
}

static void init_builtin_rom_data(xtensa_mem_t *mem) {
    mem_write32(mem, ROM_CTYPE_PTR, ROM_CTYPE_TABLE);
    mem_write8(mem, ROM_CTYPE_TABLE, 0); /* reserved EOF/signed-char entry */
    for (unsigned ch = 0; ch <= UINT8_MAX; ch++)
        mem_write8(mem, ROM_CTYPE_TABLE + 1u + ch, esp32_rom_ctype(ch));
}

xtensa_mem_t *mem_create_for_target(const flexe_target_desc_t *target) {
    if (!target ||
        target->descriptor_version != FLEXE_TARGET_DESCRIPTOR_VERSION ||
        target->peripheral_end <= target->peripheral_start ||
        (target->peripheral_start & (PAGE_SIZE - 1u)) != 0 ||
        (target->peripheral_end & (PAGE_SIZE - 1u)) != 0)
        return NULL;

    xtensa_mem_t *mem = calloc(1, sizeof(xtensa_mem_t));
    if (!mem) return NULL;

    mem->target = target;
    for (unsigned i = 0; i < FLEXE_MEM_BACKING_COUNT; i++) {
        uint8_t **slot = backing_slot(mem, (flexe_mem_backing_t)i);
        mem->backing_size[i] = target->backing_size[i];
        if (target->backing_size[i] != 0) {
            *slot = calloc(1, target->backing_size[i]);
            if (!*slot) {
                mem_destroy(mem);
                return NULL;
            }
        }
    }

    mem->mmio_page_count =
        (target->peripheral_end - target->peripheral_start) / PAGE_SIZE;
    mem->mmio = calloc(mem->mmio_page_count, sizeof(*mem->mmio));
    if (!mem->mmio) {
        mem_destroy(mem);
        return NULL;
    }

    /* NOR flash powers up erased outside the bytes supplied by an image.
     * Factory images are commonly sparse/truncated before later data
     * partitions (SPIFFS, coredump); leaving that tail calloc-zeroed makes
     * programming impossible because NOR writes can only clear bits. */
    if (mem->flash_data)
        memset(mem->flash_data, 0xFF,
               mem->backing_size[FLEXE_MEM_FLASH_DATA]);
    if (mem->flash_insn)
        memset(mem->flash_insn, 0xFF,
               mem->backing_size[FLEXE_MEM_FLASH_INSN]);

    if (page_table_init(mem) != 0) {
        mem_destroy(mem);
        return NULL;
    }
    if (target->id == FLEXE_TARGET_ESP32)
        init_builtin_rom_data(mem);

    /* Pre-populate the ESP32 ROM spiflash chip struct (ROM BSS, fixed
     * address 0x3FFAE270). On hardware the boot ROM fills this during its
     * flash setup; flexe skips the boot ROM, and firmware built with
     * CONFIG_SPI_FLASH_ROM_IMPL reads rom_spiflash_chip.chip_size from
     * here (spi_flash_mmap validates mappings against it). */
    if (target->id == FLEXE_TARGET_ESP32) {
        mem_write32(mem, 0x3FFAE270, 0x00C84016u); /* device_id: GD25Q32 */
        mem_write32(mem, 0x3FFAE274, 0x00400000u); /* chip_size: 4 MB */
        mem_write32(mem, 0x3FFAE278, 0x00010000u); /* block_size: 64 KB */
        mem_write32(mem, 0x3FFAE27C, 0x00001000u); /* sector_size: 4 KB */
        mem_write32(mem, 0x3FFAE280, 0x00000100u); /* page_size: 256 B */
        mem_write32(mem, 0x3FFAE284, 0x0000FFFFu); /* status_mask */
    }

    return mem;
}

xtensa_mem_t *mem_create(void) {
    return mem_create_for_target(flexe_target_by_id(FLEXE_TARGET_ESP32));
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
    free(mem->mmio);
    free(mem);
}

void mem_reset(xtensa_mem_t *mem) {
    if (!mem) return;
    if (mem->sram)
        memset(mem->sram, 0, mem->backing_size[FLEXE_MEM_SRAM]);
    if (mem->rtc_dram)
        memset(mem->rtc_dram, 0, mem->backing_size[FLEXE_MEM_RTC_FAST]);
    if (mem->rtc_slow)
        memset(mem->rtc_slow, 0, mem->backing_size[FLEXE_MEM_RTC_SLOW]);
}

const flexe_target_desc_t *mem_target(const xtensa_mem_t *mem) {
    return mem ? mem->target : NULL;
}

uint32_t mem_backing_size(const xtensa_mem_t *mem,
                          flexe_mem_backing_t backing) {
    if (!mem || (unsigned)backing >= FLEXE_MEM_BACKING_COUNT) return 0;
    return mem->backing_size[backing];
}

uint8_t *mem_backing_ptr(xtensa_mem_t *mem, flexe_mem_backing_t backing) {
    uint8_t **slot;
    if (!mem || (unsigned)backing >= FLEXE_MEM_BACKING_COUNT) return NULL;
    slot = backing_slot(mem, backing);
    return slot ? *slot : NULL;
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
static inline uint32_t translate_peripheral_alias(const xtensa_mem_t *mem,
                                                   uint32_t addr) {
    const flexe_target_desc_t *target = mem->target;
    if (addr >= target->peripheral_alias_start &&
        addr < target->peripheral_alias_end)
        return (uint32_t)((int64_t)addr + target->peripheral_alias_delta);
    return addr;
}

/* MMIO dispatch helper */
static mmio_handler_t *mmio_lookup(xtensa_mem_t *mem, uint32_t addr) {
    const flexe_target_desc_t *target = mem->target;
    addr = translate_peripheral_alias(mem, addr);
    if (addr >= target->peripheral_start && addr < target->peripheral_end) {
        uint32_t page = (addr - target->peripheral_start) / PAGE_SIZE;
        mmio_handler_t *h = &mem->mmio[page];
        if (h->read || h->write)
            return h;
    }
    return NULL;
}

/* MMIO slow-path functions (called from inline fast paths on page table miss) */

uint8_t mem_read8_slow(xtensa_mem_t *mem, uint32_t addr) {
    /* A verifier replay cannot reproduce a device read reliably: registers
     * may be live counters, clear-on-read FIFOs, or otherwise stateful. The
     * write journal already rejects MMIO stores through mem_journal_note();
     * reject slow-path reads for the same reason. Mapped RAM/ROM reads never
     * reach this path and remain fully comparable. */
    if (__builtin_expect(g_mem_journal_en, 0)) g_mem_journal_unsafe = 1;
    uint32_t xaddr = translate_peripheral_alias(mem, addr);
    mmio_handler_t *h = mmio_lookup(mem, xaddr);
    if (h && h->read) return (uint8_t)h->read(h->ctx, xaddr);
    return 0;
}

uint16_t mem_read16_slow(xtensa_mem_t *mem, uint32_t addr) {
    if (__builtin_expect(g_mem_journal_en, 0)) g_mem_journal_unsafe = 1;
    uint32_t xaddr = translate_peripheral_alias(mem, addr);
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
    if (__builtin_expect(g_mem_journal_en, 0)) g_mem_journal_unsafe = 1;
    uint32_t xaddr = translate_peripheral_alias(mem, addr);
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
    uint32_t xaddr = translate_peripheral_alias(mem, addr);
    mmio_handler_t *h = mmio_lookup(mem, xaddr);
    if (h && h->write) h->write(h->ctx, xaddr, val);
}

void mem_write16_slow(xtensa_mem_t *mem, uint32_t addr, uint16_t val) {
    uint32_t xaddr = translate_peripheral_alias(mem, addr);
    mmio_handler_t *h = mmio_lookup(mem, xaddr);
    if (h && h->write) h->write(h->ctx, xaddr, val);
}

void mem_write32_slow(xtensa_mem_t *mem, uint32_t addr, uint32_t val) {
    uint32_t xaddr = translate_peripheral_alias(mem, addr);
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
    uint32_t flash_size = mem->backing_size[FLEXE_MEM_FLASH_DATA];
    if (mem->backing_size[FLEXE_MEM_FLASH_INSN] < flash_size)
        flash_size = mem->backing_size[FLEXE_MEM_FLASH_INSN];
    if (len > flash_size) len = flash_size;
    memcpy(mem->flash_data, data, len);
    memcpy(mem->flash_insn, data, len);
    return 0;
}

int mem_register_mmio(xtensa_mem_t *mem, int page_index,
                      mmio_read_fn read_fn, mmio_write_fn write_fn, void *ctx) {
    if (!mem || page_index < 0 || (uint32_t)page_index >= mem->mmio_page_count)
        return -1;
    mem->mmio[page_index].read  = read_fn;
    mem->mmio[page_index].write = write_fn;
    mem->mmio[page_index].ctx   = ctx;
    return 0;
}

int mem_register_mmio_range(xtensa_mem_t *mem, uint32_t base, uint32_t size,
                            mmio_read_fn read_fn, mmio_write_fn write_fn, void *ctx) {
    if (!mem || size == 0 || base < mem->target->peripheral_start ||
        base >= mem->target->peripheral_end ||
        size > mem->target->peripheral_end - base)
        return -1;
    uint32_t first_offset = base - mem->target->peripheral_start;
    uint32_t last_offset = first_offset + size - 1u;
    int start_page = first_offset / PAGE_SIZE;
    int num_pages = (int)(last_offset / PAGE_SIZE) - start_page + 1;
    for (int i = 0; i < num_pages; i++)
        mem_register_mmio(mem, start_page + i, read_fn, write_fn, ctx);
    return 0;
}

/* ===== Speculative-write journal (see memory.h) ===== */
int      g_mem_journal_en = 0;
int      g_mem_journal_unsafe = 0;
int      g_mem_journal_count = 0;
int      g_mem_write32_observe = 0;
mem_journal_entry_t g_mem_journal[MEM_JOURNAL_MAX];

void mem_write32_observers_refresh(void) {
    g_mem_write32_observe = g_dbg_mem_watch || g_mem_journal_en;
}

void mem_journal_begin(void) {
    g_mem_journal_count = 0;
    g_mem_journal_unsafe = 0;
    g_mem_journal_en = 1;
    mem_write32_observers_refresh();
}

void mem_journal_pause(void) {
    g_mem_journal_en = 0;
    mem_write32_observers_refresh();
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
    mem_journal_pause();
    g_mem_journal_count = 0;
}

/* Optional 32-bit store observers. Kept out of line so ordinary stores pay for
 * one predicted branch regardless of how many diagnostics are available. */
int g_dbg_mem_watch;

void mem_write32_observed(xtensa_mem_t *mem, uint32_t addr, uint32_t val) {
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
    if (g_mem_journal_en) mem_journal_note(mem, addr, 4);
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
