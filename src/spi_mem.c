#include "spi_mem.h"

#include <errno.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define SPI_MEM_REGISTER_WORDS 64u
#define SPI_MEM_BUFFER_WORDS   16u

/* CMD bits shared by the ESP32 and S2/S3 SPI-memory generations. */
#define SPI_CMD_FLASH_PE       (1u << 17)
#define SPI_CMD_USR            (1u << 18)
#define SPI_CMD_FLASH_HPM      (1u << 19)
#define SPI_CMD_FLASH_RES      (1u << 20)
#define SPI_CMD_FLASH_DP       (1u << 21)
#define SPI_CMD_FLASH_CE       (1u << 22)
#define SPI_CMD_FLASH_BE       (1u << 23)
#define SPI_CMD_FLASH_SE       (1u << 24)
#define SPI_CMD_FLASH_PP       (1u << 25)
#define SPI_CMD_FLASH_WRSR     (1u << 26)
#define SPI_CMD_FLASH_RDSR     (1u << 27)
#define SPI_CMD_FLASH_RDID     (1u << 28)
#define SPI_CMD_FLASH_WRDI     (1u << 29)
#define SPI_CMD_FLASH_WREN     (1u << 30)
#define SPI_CMD_FLASH_READ     (1u << 31)

#define SPI_USER_MISO_HIGHPART (1u << 24)
#define SPI_USER_MOSI_HIGHPART (1u << 25)
#define SPI_USER_MOSI          (1u << 27)
#define SPI_USER_MISO          (1u << 28)
#define SPI_USER_ADDR          (1u << 30)
#define SPI_USER_COMMAND       (1u << 31)

#define FLASH_SR_WIP           (1u << 0)
#define FLASH_SR_WEL           (1u << 1)

typedef struct {
    uint16_t status_offset;
    uint16_t user_offset;
    uint16_t user1_offset;
    uint16_t user2_offset;
    uint16_t mosi_dlen_offset;
    uint16_t miso_dlen_offset;
    uint16_t chip_select_offset;
    uint16_t fsm_offset;
    uint16_t buffer_offset;
    uint16_t date_offset;
    uint32_t dlen_mask;
    uint32_t user_reset;
    uint32_t user1_reset;
    uint32_t user2_reset;
    uint32_t chip_select_reset;
    uint8_t chip_select_count;
    bool packed_program_length;
    bool date_writable;
} spi_mem_layout_desc_t;

typedef struct {
    /* The complete low 256-byte register aperture is small enough to retain
     * directly. Reads and writes are still admitted only for documented
     * offsets, so reserved accesses remain visible to diagnostics. */
    uint32_t reg[SPI_MEM_REGISTER_WORDS];
    uint32_t date;
} spi_mem_host_t;

/* SPI0 and SPI1 are two controllers in front of the same physical flash and
 * PSRAM buses. Device state therefore lives here, not in either controller's
 * register file. A write-enable issued through one host is visible to the
 * other host just as it is on the SoC. */
typedef struct {
    uint8_t status[3];
    bool reset_armed;
    bool powered_down;
    bool address_4byte;
} spi_nor_state_t;

typedef struct {
    bool reset_armed;
    bool qpi;
    bool burst_32;
} spi_psram_state_t;

typedef struct {
    uint8_t opcode;
    uint32_t address;
    const uint8_t *mosi;
    int mosi_bytes;
    int miso_bytes;
    bool opcode_valid;
} spi_mem_transaction_t;

struct flexe_spi_mem {
    xtensa_mem_t *mem;
    const flexe_target_desc_t *target;
    const spi_mem_layout_desc_t *layout;
    mmio_read_fn fallback_read;
    mmio_write_fn fallback_write;
    void *fallback_ctx;
    flexe_spi_mem_flash_changed_fn flash_changed;
    void *flash_changed_ctx;
    spi_mem_host_t host[FLEXE_TARGET_SPI_MEM_HOST_MAX];
    spi_nor_state_t flash;
    spi_psram_state_t psram;
};

static const spi_mem_layout_desc_t SPI_MEM_LAYOUT_ESP32 = {
    .status_offset = 0x010u,
    .user_offset = 0x01Cu,
    .user1_offset = 0x020u,
    .user2_offset = 0x024u,
    .mosi_dlen_offset = 0x028u,
    .miso_dlen_offset = 0x02Cu,
    .chip_select_offset = 0x034u, /* PIN */
    .fsm_offset = 0x0F8u, /* EXT2.ST */
    .buffer_offset = 0x080u,
    .date_offset = 0x3FCu,
    .dlen_mask = 0x00FFFFFFu,
    .user_reset = 0x80000040u,
    .user1_reset = 0x5C000007u,
    .user2_reset = 0x70000000u,
    /* CS0 enabled; CS1 and CS2 disabled. */
    .chip_select_reset = 0x00000006u,
    .chip_select_count = 3u,
    .packed_program_length = true,
};

static const spi_mem_layout_desc_t SPI_MEM_LAYOUT_S2_S3 = {
    .status_offset = 0x02Cu,
    .user_offset = 0x018u,
    .user1_offset = 0x01Cu,
    .user2_offset = 0x020u,
    .mosi_dlen_offset = 0x024u,
    .miso_dlen_offset = 0x028u,
    .chip_select_offset = 0x034u, /* MISC */
    .fsm_offset = 0x054u,
    .buffer_offset = 0x058u,
    .date_offset = 0x3FCu,
    .dlen_mask = 0x000003FFu,
    .user_reset = 0x80000000u,
    .user1_reset = 0x5C000007u,
    .user2_reset = 0x70000000u,
    /* CS0 enabled; CS1 disabled. */
    .chip_select_reset = 0x00000002u,
    .chip_select_count = 2u,
    .date_writable = true,
};

static const spi_mem_layout_desc_t *spi_mem_layout(
        flexe_spi_mem_layout_t layout) {
    switch (layout) {
    case FLEXE_SPI_MEM_LAYOUT_ESP32: return &SPI_MEM_LAYOUT_ESP32;
    case FLEXE_SPI_MEM_LAYOUT_S2_S3: return &SPI_MEM_LAYOUT_S2_S3;
    default: return NULL;
    }
}

static bool spi_mem_geometry_valid(const flexe_target_desc_t *target,
                                   const spi_mem_layout_desc_t *layout) {
    if (!target || !layout ||
        !(target->capabilities & FLEXE_TARGET_CAP_SPI_MEM))
        return false;
    const flexe_spi_mem_desc_t *desc = &target->spi_mem;
    if (desc->host_count == 0u ||
        desc->host_count > FLEXE_TARGET_SPI_MEM_HOST_MAX ||
        desc->register_size < layout->date_offset + sizeof(uint32_t) ||
        (desc->register_size & 0xFFFu) != 0u ||
        desc->default_jedec_id == 0u ||
        desc->default_jedec_id > 0x00FFFFFFu ||
        desc->maximum_flash_size == 0u ||
        (desc->maximum_flash_size & (desc->maximum_flash_size - 1u)) != 0u ||
        desc->maximum_flash_size <
            target->backing_size[FLEXE_MEM_FLASH_DATA] ||
        desc->maximum_flash_size <
            target->backing_size[FLEXE_MEM_FLASH_INSN] ||
        desc->flash_chip_select >= layout->chip_select_count)
        return false;
    bool has_psram = desc->default_psram_id != 0u;
    if ((has_psram &&
         (desc->psram_chip_select >= layout->chip_select_count ||
          desc->psram_chip_select == desc->flash_chip_select ||
          target->backing_size[FLEXE_MEM_PSRAM] == 0u)) ||
        (!has_psram && desc->psram_chip_select != FLEXE_SPI_MEM_CS_NONE))
        return false;
    for (unsigned host = 0; host < desc->host_count; host++) {
        uint32_t base = desc->base[host];
        if ((base & 0xFFFu) != 0u || base < target->peripheral_start ||
            base >= target->peripheral_end ||
            desc->register_size > target->peripheral_end - base)
            return false;
        for (unsigned other = 0; other < host; other++) {
            uint32_t other_base = desc->base[other];
            if (base < other_base + desc->register_size &&
                other_base < base + desc->register_size)
                return false;
        }
    }
    return target->backing_size[FLEXE_MEM_FLASH_DATA] != 0u &&
           target->backing_size[FLEXE_MEM_FLASH_INSN] != 0u;
}

static int spi_mem_host_index(const flexe_spi_mem_t *spi_mem,
                              uint32_t addr) {
    const flexe_spi_mem_desc_t *desc = &spi_mem->target->spi_mem;
    for (unsigned host = 0; host < desc->host_count; host++)
        if (addr >= desc->base[host] &&
            addr - desc->base[host] < desc->register_size)
            return (int)host;
    return -1;
}

static bool spi_mem_known_offset(flexe_spi_mem_layout_t generation,
                                 uint32_t offset) {
    offset &= ~3u;
    switch (generation) {
    case FLEXE_SPI_MEM_LAYOUT_ESP32:
        return offset <= 0x064u ||
               (offset >= 0x080u && offset <= 0x0C0u) ||
               (offset >= 0x0F0u && offset <= 0x0FCu) ||
               offset == 0x3FCu;
    case FLEXE_SPI_MEM_LAYOUT_S2_S3:
        return offset <= 0x0B4u ||
               (offset >= 0x0BCu && offset <= 0x0D4u) ||
               (offset >= 0x0DCu && offset <= 0x0FCu) ||
               offset == 0x3FCu;
    default:
        return false;
    }
}

static uint32_t spi_mem_flash_size(const flexe_spi_mem_t *spi_mem) {
    return mem_flash_physical_size(spi_mem->mem);
}

static uint32_t *spi_mem_buffer(flexe_spi_mem_t *spi_mem,
                                spi_mem_host_t *host, bool input) {
    uint32_t *buffer = &host->reg[spi_mem->layout->buffer_offset / 4u];
    uint32_t user = host->reg[spi_mem->layout->user_offset / 4u];
    uint32_t highpart = input ? SPI_USER_MISO_HIGHPART :
                                SPI_USER_MOSI_HIGHPART;
    return buffer + ((user & highpart) ? 8u : 0u);
}

static int spi_mem_data_bytes(const flexe_spi_mem_t *spi_mem,
                              uint32_t dlen, const spi_mem_host_t *host,
                              bool input) {
    uint32_t user = host->reg[spi_mem->layout->user_offset / 4u];
    uint32_t highpart = input ? SPI_USER_MISO_HIGHPART :
                                SPI_USER_MOSI_HIGHPART;
    uint32_t max_bytes = (user & highpart) ? 32u : 64u;
    uint32_t bits = (dlen & spi_mem->layout->dlen_mask) + 1u;
    uint32_t bytes = (bits + 7u) / 8u;
    return (int)(bytes > max_bytes ? max_bytes : bytes);
}

/* SPI_ADDR holds the address phase left-aligned. Convert the low `keep_bits`
 * of that phase to an integer after optionally consuming a command byte from
 * its front. Mode bits which make a phase non-byte-aligned remain excluded,
 * matching the controller's DIO/QIO address packing. */
static uint32_t spi_mem_address_tail(uint32_t value, unsigned phase_bits,
                                     unsigned consumed_bits) {
    if (phase_bits > 32u || consumed_bits > phase_bits) return 0u;
    unsigned keep_bits = (phase_bits - consumed_bits) & ~7u;
    if (keep_bits == 0u) return 0u;
    unsigned shift = 32u - consumed_bits - keep_bits;
    uint32_t phase = shift == 0u ? value : value >> shift;
    if (keep_bits >= 32u) return phase;
    return phase & ((UINT32_C(1) << keep_bits) - 1u);
}

static uint32_t spi_mem_command_address(const flexe_spi_mem_t *spi_mem,
                                         uint32_t value,
                                         unsigned phase_bits) {
    if (phase_bits > 32u) return 0u;
    if (spi_mem->target->spi_mem.layout == FLEXE_SPI_MEM_LAYOUT_S2_S3) {
        /* SPI_MEM_ADDR on S2/S3 stores a 24-bit flash address in bits
         * 23:0 (or the whole word in 4-byte address mode). Unlike the
         * original ESP32 controller, firmware does not left-align a
         * 24-bit phase before writing the register. */
        unsigned keep_bits = phase_bits & ~7u;
        if (keep_bits == 0u) return 0u;
        if (keep_bits == 32u) return value;
        return value & ((UINT32_C(1) << keep_bits) - 1u);
    }
    return spi_mem_address_tail(value, phase_bits, 0u);
}

static int spi_mem_program_bytes(const flexe_spi_mem_t *spi_mem,
                                 const spi_mem_host_t *host) {
    uint32_t addr = host->reg[1];
    int bytes = spi_mem->layout->packed_program_length ?
                (int)(addr >> 24) : 0;
    if (bytes == 0)
        bytes = spi_mem_data_bytes(
            spi_mem,
            host->reg[spi_mem->layout->mosi_dlen_offset / 4u],
            host, false);
    return bytes > 64 ? 64 : bytes;
}

static spi_mem_transaction_t spi_mem_decode_user(
        flexe_spi_mem_t *spi_mem, spi_mem_host_t *host) {
    spi_mem_transaction_t transaction = {0};
    uint32_t user = host->reg[spi_mem->layout->user_offset / 4u];
    uint32_t user1 = host->reg[spi_mem->layout->user1_offset / 4u];
    uint32_t user2 = host->reg[spi_mem->layout->user2_offset / 4u];
    uint32_t address = host->reg[1];

    transaction.mosi = (const uint8_t *)spi_mem_buffer(
        spi_mem, host, false);
    if (user & SPI_USER_MOSI)
        transaction.mosi_bytes = spi_mem_data_bytes(
            spi_mem,
            host->reg[spi_mem->layout->mosi_dlen_offset / 4u],
            host, false);
    if (user & SPI_USER_MISO)
        transaction.miso_bytes = spi_mem_data_bytes(
            spi_mem,
            host->reg[spi_mem->layout->miso_dlen_offset / 4u],
            host, true);

    unsigned address_bits = 0u;
    if (user & SPI_USER_ADDR)
        address_bits = ((user1 >> 26) & 0x3Fu) + 1u;

    if (user & SPI_USER_COMMAND) {
        unsigned command_bits = ((user2 >> 28) & 0xFu) + 1u;
        if (command_bits == 8u) {
            transaction.opcode = (uint8_t)user2;
            transaction.opcode_valid = true;
            transaction.address = spi_mem_command_address(
                spi_mem, address, address_bits);
            return transaction;
        }

        /* ESP32's original 32-Mbit PSRAM needs two idle clocks before some
         * commands. ESP-IDF expresses those clocks as a short all-zero
         * command phase and carries the actual opcode in the first byte of
         * the address phase. Decode the wire phases, not a particular PC or
         * firmware build. */
        uint32_t command_mask = (UINT32_C(1) << command_bits) - 1u;
        if ((user2 & command_mask) != 0u || address_bits < 8u)
            return transaction;
    }

    if ((user & SPI_USER_ADDR) && address_bits >= 8u) {
        transaction.opcode = (uint8_t)(address >> 24);
        transaction.opcode_valid = true;
        transaction.address = spi_mem_address_tail(
            address, address_bits, 8u);
        return transaction;
    }

    /* QPI-exit on the same device is intentionally emitted as raw MOSI data.
     * Some clock modes insert complete zero bytes before the opcode. Treat
     * only leading zeroes as padding; subsequent bytes remain payload. */
    if ((user & SPI_USER_MOSI) && transaction.mosi_bytes > 0) {
        int byte = 0;
        while (byte < transaction.mosi_bytes &&
               transaction.mosi[byte] == 0u)
            byte++;
        if (byte < transaction.mosi_bytes) {
            transaction.opcode = transaction.mosi[byte++];
            transaction.opcode_valid = true;
            transaction.mosi += byte;
            transaction.mosi_bytes -= byte;
        }
    }
    return transaction;
}

typedef enum {
    SPI_MEM_DEVICE_NONE,
    SPI_MEM_DEVICE_FLASH,
    SPI_MEM_DEVICE_PSRAM,
    SPI_MEM_DEVICE_CONTENTION,
} spi_mem_device_t;

static spi_mem_device_t spi_mem_selected_device(
        const flexe_spi_mem_t *spi_mem, const spi_mem_host_t *host) {
    const flexe_spi_mem_desc_t *desc = &spi_mem->target->spi_mem;
    uint32_t disabled = host->reg[
        spi_mem->layout->chip_select_offset / 4u];
    bool flash = (disabled & (UINT32_C(1) <<
                              desc->flash_chip_select)) == 0u;
    bool psram = desc->default_psram_id != 0u &&
                 (disabled & (UINT32_C(1) <<
                              desc->psram_chip_select)) == 0u;
    if (flash && psram) return SPI_MEM_DEVICE_CONTENTION;
    if (flash) return SPI_MEM_DEVICE_FLASH;
    if (psram) return SPI_MEM_DEVICE_PSRAM;
    return SPI_MEM_DEVICE_NONE;
}

enum { SPI_DBG_OFF, SPI_DBG_BOOT, SPI_DBG_ALL, SPI_DBG_RANGE };
static int spi_dbg_mode = -1;
static unsigned long spi_dbg_first;
static unsigned long spi_dbg_last;

static void spi_debug_resolve(void) {
    const char *spec = getenv("FLEXE_SPIDBG");
    if (!spec) { spi_dbg_mode = SPI_DBG_OFF; return; }
    if (*spec == '\0') { spi_dbg_mode = SPI_DBG_BOOT; return; }
    if (strcmp(spec, "all") == 0) { spi_dbg_mode = SPI_DBG_ALL; return; }

    spi_dbg_mode = SPI_DBG_BOOT;
    errno = 0;
    char *end = NULL;
    unsigned long first = strtoul(spec, &end, 0);
    if (errno || end == spec) return;
    unsigned long last = first;
    if (*end == '-') {
        const char *tail = end + 1;
        errno = 0;
        last = strtoul(tail, &end, 0);
        if (errno || end == tail) return;
    }
    if (*end != '\0') return;
    spi_dbg_first = first;
    spi_dbg_last = last;
    spi_dbg_mode = SPI_DBG_RANGE;
}

static bool spi_debug_offset(uint32_t offset) {
    if (spi_dbg_mode < 0) spi_debug_resolve();
    switch (spi_dbg_mode) {
    case SPI_DBG_OFF: return false;
    case SPI_DBG_ALL: return true;
    case SPI_DBG_RANGE:
        return (unsigned long)offset >= spi_dbg_first &&
               (unsigned long)offset <= spi_dbg_last;
    default: return offset < 0x20000u;
    }
}

static void spi_debug_command(unsigned host, const spi_mem_host_t *state,
                              const spi_mem_layout_desc_t *layout,
                              uint8_t opcode, uint32_t offset,
                              int mosi, int miso) {
    if (!spi_debug_offset(offset)) return;
    fprintf(stderr,
            "[SPI%u] op=%02X off=0x%06X mosi=%d miso=%d pc=%08X core=%d "
            "user=%08X user1=%08X addr=%08X\n",
            host, opcode, offset, mosi, miso, g_dbg_pc, g_dbg_core,
            state->reg[layout->user_offset / 4u],
            state->reg[layout->user1_offset / 4u], state->reg[1]);
}

static uint8_t *spi_mem_prepare_input(flexe_spi_mem_t *spi_mem,
                                      spi_mem_host_t *host) {
    uint32_t *all = &host->reg[spi_mem->layout->buffer_offset / 4u];
    memset(all, 0xFF, SPI_MEM_BUFFER_WORDS * sizeof(*all));
    return (uint8_t *)spi_mem_buffer(spi_mem, host, true);
}

static void spi_mem_set_input_word(flexe_spi_mem_t *spi_mem,
                                   spi_mem_host_t *host, uint32_t value) {
    uint8_t *dst = spi_mem_prepare_input(spi_mem, host);
    for (unsigned byte = 0; byte < sizeof(value); byte++)
        dst[byte] = (uint8_t)(value >> (byte * 8u));
}

static void spi_mem_set_input_u64(flexe_spi_mem_t *spi_mem,
                                  spi_mem_host_t *host, uint64_t value) {
    uint8_t *dst = spi_mem_prepare_input(spi_mem, host);
    for (unsigned byte = 0; byte < sizeof(value); byte++)
        dst[byte] = (uint8_t)(value >> (byte * 8u));
}

static void spi_mem_read_data(flexe_spi_mem_t *spi_mem,
                              spi_mem_host_t *host,
                              uint32_t offset, int bytes) {
    uint32_t size = spi_mem_flash_size(spi_mem);
    uint8_t *dst = spi_mem_prepare_input(spi_mem, host);
    if (offset < size && spi_mem->mem->flash_data) {
        uint32_t available = size - offset;
        if ((uint32_t)bytes > available) bytes = (int)available;
        memcpy(dst, spi_mem->mem->flash_data + offset, (size_t)bytes);
    }
    if (spi_debug_offset(offset)) {
        fprintf(stderr,
                "[SPIRD] off=0x%X bytes=%d w0=%02X %02X %02X %02X "
                "%02X %02X %02X %02X\n",
                offset, bytes, dst[0], dst[1], dst[2], dst[3],
                dst[4], dst[5], dst[6], dst[7]);
    }
}

static void spi_mem_read_psram(flexe_spi_mem_t *spi_mem,
                               spi_mem_host_t *host,
                               uint32_t offset, int bytes) {
    uint8_t *dst = spi_mem_prepare_input(spi_mem, host);
    uint32_t size = mem_backing_size(spi_mem->mem, FLEXE_MEM_PSRAM);
    uint8_t *src = mem_backing_ptr(spi_mem->mem, FLEXE_MEM_PSRAM);
    if (!src || size == 0u) return;
    for (int byte = 0; byte < bytes; byte++)
        dst[byte] = src[(offset + (uint32_t)byte) % size];
}

static void spi_mem_write_psram(flexe_spi_mem_t *spi_mem,
                                uint32_t offset, const uint8_t *src,
                                int bytes) {
    uint32_t size = mem_backing_size(spi_mem->mem, FLEXE_MEM_PSRAM);
    uint8_t *dst = mem_backing_ptr(spi_mem->mem, FLEXE_MEM_PSRAM);
    if (!src || !dst || size == 0u) return;
    for (int byte = 0; byte < bytes; byte++)
        dst[(offset + (uint32_t)byte) % size] = src[byte];
}

static void spi_mem_note_flash_change(flexe_spi_mem_t *spi_mem,
                                      uint32_t offset, uint32_t size) {
    if (spi_mem->flash_changed)
        spi_mem->flash_changed(spi_mem->flash_changed_ctx, offset, size);
}

static void spi_mem_program(flexe_spi_mem_t *spi_mem,
                            uint32_t offset, const uint8_t *src, int bytes) {
    uint32_t size = spi_mem_flash_size(spi_mem);
    if (offset >= size || !spi_mem->mem->flash_data ||
        !spi_mem->mem->flash_insn || !src)
        return;
    uint32_t available = size - offset;
    if ((uint32_t)bytes > available) bytes = (int)available;
    if (spi_debug_offset(offset)) {
        fprintf(stderr,
                "[SPIWR] off=0x%X bytes=%d w0=%02X %02X %02X %02X "
                "%02X %02X %02X %02X\n",
                offset, bytes, src[0], src[1], src[2], src[3],
                src[4], src[5], src[6], src[7]);
    }
    for (int i = 0; i < bytes; i++) {
        spi_mem->mem->flash_data[offset + (uint32_t)i] &= src[i];
        spi_mem->mem->flash_insn[offset + (uint32_t)i] &= src[i];
    }
    spi_mem_note_flash_change(spi_mem, offset, (uint32_t)bytes);
}

static void spi_mem_erase(flexe_spi_mem_t *spi_mem,
                          uint32_t offset, uint32_t size) {
    uint32_t flash_size = spi_mem_flash_size(spi_mem);
    if (offset >= flash_size || !spi_mem->mem->flash_data ||
        !spi_mem->mem->flash_insn)
        return;
    if (size > flash_size - offset) size = flash_size - offset;
    if (spi_debug_offset(offset))
        fprintf(stderr, "[SPIERASE] off=0x%X bytes=%u\n", offset, size);
    memset(spi_mem->mem->flash_data + offset, 0xFF, size);
    memset(spi_mem->mem->flash_insn + offset, 0xFF, size);
    spi_mem_note_flash_change(spi_mem, offset, size);
}

static bool spi_mem_execute_flash(flexe_spi_mem_t *spi_mem,
                                  spi_mem_host_t *host,
                                  const spi_mem_transaction_t *transaction) {
    spi_nor_state_t *flash = &spi_mem->flash;
    uint8_t opcode = transaction->opcode;
    uint32_t offset = transaction->address;
    int mosi = transaction->mosi_bytes;

    if (flash->powered_down && opcode != 0xABu) {
        spi_mem_prepare_input(spi_mem, host);
        return true;
    }

    switch (opcode) {
    case 0x9Fu: /* RDID */
    case 0x90u: /* REMID */
        spi_mem_set_input_word(spi_mem, host,
                               mem_flash_jedec_id(spi_mem->mem));
        return true;
    case 0xABu: /* release power-down / electronic signature */
        flash->powered_down = false;
        spi_mem_set_input_word(spi_mem, host,
                               mem_flash_jedec_id(spi_mem->mem));
        return true;
    case 0xB9u: flash->powered_down = true; return true;
    case 0x05u:
        spi_mem_set_input_word(spi_mem, host, flash->status[0]);
        return true;
    case 0x35u:
        spi_mem_set_input_word(spi_mem, host, flash->status[1]);
        return true;
    case 0x15u:
        spi_mem_set_input_word(spi_mem, host, flash->status[2]);
        return true;
    case 0x01u:
        if (!(flash->status[0] & FLASH_SR_WEL) || mosi == 0) return true;
        flash->status[0] = (uint8_t)(
            (flash->status[0] & (FLASH_SR_WIP | FLASH_SR_WEL)) |
            (transaction->mosi[0] & ~(FLASH_SR_WIP | FLASH_SR_WEL)));
        if (mosi >= 2) flash->status[1] = transaction->mosi[1];
        flash->status[0] &= (uint8_t)~FLASH_SR_WEL;
        return true;
    case 0x31u:
        if ((flash->status[0] & FLASH_SR_WEL) && mosi > 0) {
            flash->status[1] = transaction->mosi[0];
            flash->status[0] &= (uint8_t)~FLASH_SR_WEL;
        }
        return true;
    case 0x11u:
        if ((flash->status[0] & FLASH_SR_WEL) && mosi > 0) {
            flash->status[2] = transaction->mosi[0];
            flash->status[0] &= (uint8_t)~FLASH_SR_WEL;
        }
        return true;
    case 0x06u: flash->status[0] |= FLASH_SR_WEL; return true;
    case 0x04u: flash->status[0] &= (uint8_t)~FLASH_SR_WEL; return true;
    case 0x03u: case 0x0Bu: case 0x3Bu:
    case 0x6Bu: case 0xBBu: case 0xEBu:
        spi_mem_read_data(spi_mem, host, offset, transaction->miso_bytes);
        return true;
    case 0x02u: case 0x32u:
        if (flash->status[0] & FLASH_SR_WEL) {
            spi_mem_program(spi_mem, offset, transaction->mosi, mosi);
            flash->status[0] &= (uint8_t)~FLASH_SR_WEL;
        }
        return true;
    case 0x20u:
        if (flash->status[0] & FLASH_SR_WEL) {
            spi_mem_erase(spi_mem, offset & ~0xFFFu, 0x1000u);
            flash->status[0] &= (uint8_t)~FLASH_SR_WEL;
        }
        return true;
    case 0x52u:
        if (flash->status[0] & FLASH_SR_WEL) {
            spi_mem_erase(spi_mem, offset & ~0x7FFFu, 0x8000u);
            flash->status[0] &= (uint8_t)~FLASH_SR_WEL;
        }
        return true;
    case 0xD8u:
        if (flash->status[0] & FLASH_SR_WEL) {
            spi_mem_erase(spi_mem, offset & ~0xFFFFu, 0x10000u);
            flash->status[0] &= (uint8_t)~FLASH_SR_WEL;
        }
        return true;
    case 0x60u: case 0xC7u:
        if (flash->status[0] & FLASH_SR_WEL) {
            spi_mem_erase(spi_mem, 0u, spi_mem_flash_size(spi_mem));
            flash->status[0] &= (uint8_t)~FLASH_SR_WEL;
        }
        return true;
    case 0x66u: flash->reset_armed = true; return true;
    case 0x99u:
        if (flash->reset_armed) {
            memset(flash->status, 0, sizeof(flash->status));
            flash->powered_down = false;
            flash->address_4byte = false;
            flash->reset_armed = false;
        }
        return true;
    case 0xB7u: flash->address_4byte = true; return true;
    case 0xE9u: flash->address_4byte = false; return true;
    default: return false;
    }
}

static bool spi_mem_execute_psram(flexe_spi_mem_t *spi_mem,
                                  spi_mem_host_t *host,
                                  const spi_mem_transaction_t *transaction) {
    spi_psram_state_t *psram = &spi_mem->psram;
    switch (transaction->opcode) {
    case 0x9Fu: /* device ID */
        spi_mem_set_input_u64(
            spi_mem, host, spi_mem->target->spi_mem.default_psram_id);
        return true;
    case 0x03u: /* read */
    case 0x0Bu: /* fast read */
    case 0xEBu: /* quad fast read */
        spi_mem_read_psram(spi_mem, host, transaction->address,
                           transaction->miso_bytes);
        return true;
    case 0x02u: /* write */
    case 0x38u: /* quad write */
        spi_mem_write_psram(spi_mem, transaction->address,
                            transaction->mosi, transaction->mosi_bytes);
        return true;
    case 0x35u: psram->qpi = true; return true;
    case 0xF5u: psram->qpi = false; return true;
    case 0x66u: psram->reset_armed = true; return true;
    case 0x99u:
        if (psram->reset_armed) {
            psram->reset_armed = false;
            psram->qpi = false;
            psram->burst_32 = false;
        }
        return true;
    case 0xC0u:
        psram->burst_32 = !psram->burst_32;
        return true;
    /* ESP32's optional 2T training sequence uses these documented device
     * commands. The intervening electrical training clocks do not change the
     * byte-addressable storage model, but the commands are valid. */
    case 0x5Eu:
    case 0x5Fu:
        return true;
    default:
        return false;
    }
}

static bool spi_mem_execute_user(flexe_spi_mem_t *spi_mem, unsigned index,
                                 spi_mem_host_t *host) {
    spi_mem_transaction_t transaction =
        spi_mem_decode_user(spi_mem, host);
    spi_debug_command(index, host, spi_mem->layout, transaction.opcode,
                      transaction.address, transaction.mosi_bytes,
                      transaction.miso_bytes);
    if (!transaction.opcode_valid) return false;

    switch (spi_mem_selected_device(spi_mem, host)) {
    case SPI_MEM_DEVICE_FLASH:
        return spi_mem_execute_flash(spi_mem, host, &transaction);
    case SPI_MEM_DEVICE_PSRAM:
        return spi_mem_execute_psram(spi_mem, host, &transaction);
    case SPI_MEM_DEVICE_NONE:
        /* A valid transaction to an unpopulated chip select completes; no
         * slave drives MISO, so pull-ups return all ones. */
        if (transaction.miso_bytes > 0)
            spi_mem_prepare_input(spi_mem, host);
        return true;
    case SPI_MEM_DEVICE_CONTENTION:
    default:
        return false;
    }
}

static bool spi_mem_execute_dedicated(flexe_spi_mem_t *spi_mem,
                                      spi_mem_host_t *host,
                                      uint32_t command) {
    spi_nor_state_t *flash = &spi_mem->flash;
    bool handled = false;
    uint32_t addr = host->reg[1];
    uint32_t offset = spi_mem->layout->packed_program_length ||
                      !flash->address_4byte ? addr & 0x00FFFFFFu : addr;
    if (command & SPI_CMD_FLASH_RDID) {
        spi_mem_set_input_word(spi_mem, host,
                               mem_flash_jedec_id(spi_mem->mem));
        handled = true;
    }
    if (command & SPI_CMD_FLASH_RDSR) {
        host->reg[spi_mem->layout->status_offset / 4u] =
            flash->status[0] | ((uint32_t)flash->status[1] << 8) |
            ((uint32_t)flash->status[2] << 16);
        handled = true;
    }
    if (command & SPI_CMD_FLASH_WRDI) {
        flash->status[0] &= (uint8_t)~FLASH_SR_WEL;
        handled = true;
    }
    if (command & SPI_CMD_FLASH_WREN) {
        flash->status[0] |= FLASH_SR_WEL;
        handled = true;
    }
    if (command & SPI_CMD_FLASH_WRSR) {
        if (flash->status[0] & FLASH_SR_WEL) {
            uint32_t value = *spi_mem_buffer(spi_mem, host, false);
            flash->status[0] = (uint8_t)(
                (flash->status[0] & (FLASH_SR_WIP | FLASH_SR_WEL)) |
                (value & ~(FLASH_SR_WIP | FLASH_SR_WEL)));
            flash->status[1] = (uint8_t)(value >> 8);
            flash->status[0] &= (uint8_t)~FLASH_SR_WEL;
        }
        handled = true;
    }
    if (command & SPI_CMD_FLASH_READ) {
        spi_mem_read_data(
            spi_mem, host, offset,
            spi_mem_data_bytes(
                spi_mem,
                host->reg[spi_mem->layout->miso_dlen_offset / 4u],
                host, true));
        handled = true;
    }
    if (command & SPI_CMD_FLASH_PP) {
        if (flash->status[0] & FLASH_SR_WEL) {
            spi_mem_program(
                spi_mem, offset,
                (const uint8_t *)spi_mem_buffer(spi_mem, host, false),
                spi_mem_program_bytes(spi_mem, host));
            flash->status[0] &= (uint8_t)~FLASH_SR_WEL;
        }
        handled = true;
    }
    if (command & SPI_CMD_FLASH_SE) {
        if (flash->status[0] & FLASH_SR_WEL) {
            spi_mem_erase(spi_mem, offset & ~0xFFFu, 0x1000u);
            flash->status[0] &= (uint8_t)~FLASH_SR_WEL;
        }
        handled = true;
    }
    if (command & SPI_CMD_FLASH_BE) {
        if (flash->status[0] & FLASH_SR_WEL) {
            spi_mem_erase(spi_mem, offset & ~0xFFFFu, 0x10000u);
            flash->status[0] &= (uint8_t)~FLASH_SR_WEL;
        }
        handled = true;
    }
    if (command & SPI_CMD_FLASH_CE) {
        if (flash->status[0] & FLASH_SR_WEL) {
            spi_mem_erase(spi_mem, 0u, spi_mem_flash_size(spi_mem));
            flash->status[0] &= (uint8_t)~FLASH_SR_WEL;
        }
        handled = true;
    }
    if (command & SPI_CMD_FLASH_RES) {
        flash->powered_down = false;
        host->reg[spi_mem->layout->status_offset / 4u] =
            mem_flash_jedec_id(spi_mem->mem);
        handled = true;
    }
    if (command & SPI_CMD_FLASH_DP) {
        flash->powered_down = true;
        handled = true;
    }
    if (command & SPI_CMD_FLASH_HPM) handled = true;
    return handled;
}

static void spi_mem_execute(flexe_spi_mem_t *spi_mem, unsigned index,
                            uint32_t command) {
    spi_mem_host_t *host = &spi_mem->host[index];
    bool handled = false;
    if (command & SPI_CMD_USR)
        handled = spi_mem_execute_user(spi_mem, index, host);
    else
        handled = spi_mem_execute_dedicated(spi_mem, host, command);

    /* FLASH_PE qualifies a USR transaction rather than naming a command by
     * itself. An unknown transaction must remain visible instead of looking
     * like successful flash I/O merely because CMD self-cleared. */
    uint32_t qualifiers = SPI_CMD_FLASH_PE;
    if (!handled && (command & ~qualifiers) != 0u &&
        spi_mem->fallback_write)
        spi_mem->fallback_write(
            spi_mem->fallback_ctx,
            spi_mem->target->spi_mem.base[index], command);
}

static uint32_t spi_mem_read(void *ctx, uint32_t addr) {
    flexe_spi_mem_t *spi_mem = ctx;
    int index = spi_mem_host_index(spi_mem, addr);
    if (index < 0) goto fallback;
    uint32_t offset = addr - spi_mem->target->spi_mem.base[index];
    if (!spi_mem_known_offset(spi_mem->target->spi_mem.layout, offset))
        goto fallback;
    uint32_t shift = (offset & 3u) * 8u;
    offset &= ~3u;
    spi_mem_host_t *host = &spi_mem->host[index];
    uint32_t value = 0u;
    if (offset == 0u || offset == spi_mem->layout->fsm_offset)
        value = 0u; /* command completed and state machine idle */
    else if (offset == spi_mem->layout->date_offset)
        value = host->date;
    else if (spi_mem->target->spi_mem.layout ==
                 FLEXE_SPI_MEM_LAYOUT_S2_S3 &&
             offset == 0x0FCu)
        value = host->reg[0x0F8u / 4u] & host->reg[0x0F0u / 4u];
    else if (offset < sizeof(host->reg))
        value = host->reg[offset / 4u];
    else
        goto fallback;
    /* Memory's width-specific slow paths truncate the returned value. Shift
     * the addressed byte lane down first so byte copies of W0..W15 observe
     * the actual data buffer rather than four copies of each word's byte 0. */
    return value >> shift;

fallback:
    return spi_mem->fallback_read ?
        spi_mem->fallback_read(spi_mem->fallback_ctx, addr) : 0u;
}

static void spi_mem_write(void *ctx, uint32_t addr, uint32_t value) {
    flexe_spi_mem_t *spi_mem = ctx;
    int index = spi_mem_host_index(spi_mem, addr);
    if (index < 0) goto fallback;
    uint32_t offset = addr - spi_mem->target->spi_mem.base[index];
    if (!spi_mem_known_offset(spi_mem->target->spi_mem.layout, offset))
        goto fallback;
    uint32_t byte_lane = offset & 3u;
    offset &= ~3u;
    spi_mem_host_t *host = &spi_mem->host[index];
    if (offset == 0u && byte_lane == 0u) {
        spi_mem_execute(spi_mem, (unsigned)index, value);
        return;
    }
    if (offset == spi_mem->layout->status_offset ||
        offset == spi_mem->layout->fsm_offset)
        return;
    if (offset == spi_mem->layout->date_offset) {
        if (spi_mem->layout->date_writable) host->date = value;
        return;
    }
    if (spi_mem->target->spi_mem.layout == FLEXE_SPI_MEM_LAYOUT_S2_S3) {
        if (offset == 0x0F4u || offset == 0x0F8u) {
            host->reg[0x0F8u / 4u] &= ~value;
            return;
        }
        if (offset == 0x0FCu) return;
    }
    if (offset < sizeof(host->reg)) {
        if (byte_lane == 0u) {
            host->reg[offset / 4u] = value;
        } else {
            uint32_t shift = byte_lane * 8u;
            uint32_t mask = 0xFFu << shift;
            host->reg[offset / 4u] =
                (host->reg[offset / 4u] & ~mask) |
                ((value & 0xFFu) << shift);
        }
        return;
    }

fallback:
    if (spi_mem->fallback_write)
        spi_mem->fallback_write(spi_mem->fallback_ctx, addr, value);
}

flexe_spi_mem_t *flexe_spi_mem_create(
    xtensa_mem_t *mem, mmio_read_fn fallback_read,
    mmio_write_fn fallback_write, void *fallback_ctx,
    flexe_spi_mem_flash_changed_fn flash_changed, void *flash_changed_ctx) {
    if (!mem) return NULL;
    const flexe_target_desc_t *target = mem_target(mem);
    const spi_mem_layout_desc_t *layout =
        target ? spi_mem_layout(target->spi_mem.layout) : NULL;
    if (!spi_mem_geometry_valid(target, layout)) return NULL;

    flexe_spi_mem_t *spi_mem = calloc(1, sizeof(*spi_mem));
    if (!spi_mem) return NULL;
    spi_mem->mem = mem;
    spi_mem->target = target;
    spi_mem->layout = layout;
    spi_mem->fallback_read = fallback_read;
    spi_mem->fallback_write = fallback_write;
    spi_mem->fallback_ctx = fallback_ctx;
    spi_mem->flash_changed = flash_changed;
    spi_mem->flash_changed_ctx = flash_changed_ctx;

    for (unsigned host = 0; host < target->spi_mem.host_count; host++) {
        spi_mem_host_t *state = &spi_mem->host[host];
        state->reg[layout->user_offset / 4u] = layout->user_reset;
        state->reg[layout->user1_offset / 4u] = layout->user1_reset;
        state->reg[layout->user2_offset / 4u] = layout->user2_reset;
        state->reg[layout->chip_select_offset / 4u] =
            layout->chip_select_reset;
        state->date = target->spi_mem.date_reset;
        if (mem_register_mmio_range(mem, target->spi_mem.base[host],
                                    target->spi_mem.register_size,
                                    spi_mem_read, spi_mem_write,
                                    spi_mem) != 0) {
            for (unsigned undo = 0; undo < host; undo++)
                (void)mem_register_mmio_range(
                    mem, target->spi_mem.base[undo],
                    target->spi_mem.register_size, NULL, NULL, NULL);
            free(spi_mem);
            return NULL;
        }
    }
    return spi_mem;
}

void flexe_spi_mem_destroy(flexe_spi_mem_t *spi_mem) {
    if (!spi_mem) return;
    for (unsigned host = 0;
         host < spi_mem->target->spi_mem.host_count; host++)
        (void)mem_register_mmio_range(
            spi_mem->mem, spi_mem->target->spi_mem.base[host],
            spi_mem->target->spi_mem.register_size, NULL, NULL, NULL);
    free(spi_mem);
}
