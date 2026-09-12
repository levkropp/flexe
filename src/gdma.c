#include "gdma.h"

#include <stdbool.h>
#include <stdlib.h>

/* ESP32-S3 AHB GDMA v1 channel layout. Each channel contains an IN half at
 * +0x00 and an OUT/TX half at +0x60. Registers not involved in descriptor
 * transport remain ordinary retained configuration words. */
#define GDMA_V1_IN_CONF0_OFF        0x000u
#define GDMA_V1_IN_CONF1_OFF        0x004u
#define GDMA_V1_IN_INT_RAW_OFF      0x008u
#define GDMA_V1_IN_INT_ST_OFF       0x00Cu
#define GDMA_V1_IN_INT_ENA_OFF      0x010u
#define GDMA_V1_IN_INT_CLR_OFF      0x014u
#define GDMA_V1_IN_LINK_OFF         0x020u
#define GDMA_V1_IN_STATE_OFF        0x024u
#define GDMA_V1_IN_SUC_EOF_DESC_OFF 0x028u
#define GDMA_V1_IN_ERR_EOF_DESC_OFF 0x02Cu
#define GDMA_V1_IN_DESC_OFF         0x030u
#define GDMA_V1_IN_DESC_PREV_OFF    0x034u
#define GDMA_V1_IN_DESC_PREV2_OFF   0x038u
#define GDMA_V1_IN_PERI_SEL_OFF     0x048u

#define GDMA_V1_OUT_CONF0_OFF       0x060u
#define GDMA_V1_OUT_CONF1_OFF       0x064u
#define GDMA_V1_OUT_INT_RAW_OFF     0x068u
#define GDMA_V1_OUT_INT_ST_OFF      0x06Cu
#define GDMA_V1_OUT_INT_ENA_OFF     0x070u
#define GDMA_V1_OUT_INT_CLR_OFF     0x074u
#define GDMA_V1_OUT_LINK_OFF        0x080u
#define GDMA_V1_OUT_STATE_OFF       0x084u
#define GDMA_V1_OUT_EOF_DESC_OFF    0x088u
#define GDMA_V1_OUT_EOF_PREV_OFF    0x08Cu
#define GDMA_V1_OUT_DESC_OFF        0x090u
#define GDMA_V1_OUT_DESC_PREV_OFF   0x094u
#define GDMA_V1_OUT_DESC_PREV2_OFF  0x098u
#define GDMA_V1_OUT_PERI_SEL_OFF    0x0A8u

#define GDMA_V1_OUT_RESET           (1u << 0)
#define GDMA_V1_OUT_AUTO_WRBACK     (1u << 2)
#define GDMA_V1_OUT_CHECK_OWNER     (1u << 12)
#define GDMA_V1_IN_RESET            (1u << 0)
#define GDMA_V1_IN_CHECK_OWNER      (1u << 12)
#define GDMA_V1_IN_LINK_STOP        (1u << 21)
#define GDMA_V1_IN_LINK_START       (1u << 22)
#define GDMA_V1_IN_LINK_RESTART     (1u << 23)
#define GDMA_V1_IN_LINK_PARK        (1u << 24)
#define GDMA_V1_IN_LINK_AUTO_RETURN (1u << 20)
#define GDMA_V1_LINK_ADDR_MASK      0x000FFFFFu
#define GDMA_V1_LINK_STOP           (1u << 20)
#define GDMA_V1_LINK_START          (1u << 21)
#define GDMA_V1_LINK_RESTART        (1u << 22)
#define GDMA_V1_LINK_PARK           (1u << 23)
#define GDMA_V1_PERI_SEL_MASK       0x3Fu
#define GDMA_V1_PERI_SEL_RESET      0x3Fu

#define GDMA_V1_DESC_SIZE_MASK      0x00000FFFu
#define GDMA_V1_DESC_LENGTH_MASK    0x00FFF000u
#define GDMA_V1_DESC_LENGTH_SHIFT   12u
#define GDMA_V1_DESC_EOF            (1u << 30)
#define GDMA_V1_DESC_OWNER          (1u << 31)
#define GDMA_V1_DESC_BYTES          12u
#define GDMA_V1_DESC_LIMIT          1024u

#define GDMA_V1_OUT_DONE_INT        (1u << 0)
#define GDMA_V1_OUT_EOF_INT         (1u << 1)
#define GDMA_V1_OUT_TOTAL_EOF_INT   (1u << 3)
#define GDMA_V1_OUT_DSCR_ERR_INT    (1u << 2)
#define GDMA_V1_IN_DONE_INT         (1u << 0)
#define GDMA_V1_IN_SUC_EOF_INT      (1u << 1)
#define GDMA_V1_IN_ERR_EOF_INT      (1u << 2)
#define GDMA_V1_IN_DSCR_ERR_INT     (1u << 3)
#define GDMA_V1_IN_DSCR_EMPTY_INT   (1u << 4)
#define GDMA_V1_IN_INT_MASK          0x000003FFu
#define GDMA_V1_OUT_INT_MASK         0x000000FFu

typedef struct {
    uint32_t link_address;
    uint32_t current_desc;
    uint32_t previous_desc;
    bool active;
} gdma_tx_channel_t;

typedef struct {
    uint32_t link_address;
    uint32_t current_desc;
    uint32_t previous_desc;
    bool auto_return;
    bool active;
} gdma_rx_channel_t;

struct flexe_gdma {
    xtensa_mem_t *mem;
    const flexe_target_desc_t *target;
    mmio_read_fn fallback_read;
    mmio_write_fn fallback_write;
    void *fallback_ctx;
    uint32_t *regs;
    gdma_rx_channel_t rx[FLEXE_TARGET_GDMA_CHANNEL_MAX];
    gdma_tx_channel_t tx[FLEXE_TARGET_GDMA_CHANNEL_MAX];
};

static bool gdma_geometry_valid(const flexe_target_desc_t *target)
{
    if (!target || !(target->capabilities & FLEXE_TARGET_CAP_GDMA_V1))
        return false;
    const flexe_gdma_desc_t *desc = &target->gdma;
    if (desc->base < target->peripheral_start ||
        desc->base >= target->peripheral_end ||
        desc->register_size == 0u ||
        (desc->base & 3u) != 0u ||
        (desc->register_size & 0xFFFu) != 0u ||
        desc->register_size > target->peripheral_end - desc->base ||
        desc->channel_count == 0u ||
        desc->channel_count > FLEXE_TARGET_GDMA_CHANNEL_MAX ||
        desc->channel_stride < 0x0ACu ||
        (desc->channel_stride & 3u) != 0u ||
        (uint64_t)(desc->channel_count - 1u) * desc->channel_stride +
            GDMA_V1_OUT_PERI_SEL_OFF + sizeof(uint32_t) >
            desc->register_size ||
        (desc->descriptor_address_prefix & GDMA_V1_LINK_ADDR_MASK) != 0u)
        return false;
    return true;
}

static bool gdma_decode_channel(const flexe_gdma_t *gdma, uint32_t addr,
                                unsigned *channel_out,
                                uint32_t *channel_offset_out)
{
    const flexe_gdma_desc_t *desc = &gdma->target->gdma;
    if (addr < desc->base || addr >= desc->base + desc->register_size)
        return false;
    uint32_t offset = addr - desc->base;
    unsigned channel = offset / desc->channel_stride;
    if (channel >= desc->channel_count) return false;
    if (channel_out) *channel_out = channel;
    if (channel_offset_out)
        *channel_offset_out = offset - channel * desc->channel_stride;
    return true;
}

static uint32_t *gdma_reg(flexe_gdma_t *gdma, uint32_t addr)
{
    const flexe_gdma_desc_t *desc = &gdma->target->gdma;
    if (addr < desc->base || addr >= desc->base + desc->register_size ||
        (addr & 3u) != 0u)
        return NULL;
    return &gdma->regs[(addr - desc->base) / sizeof(uint32_t)];
}

static uint32_t gdma_channel_addr(const flexe_gdma_t *gdma,
                                  unsigned channel, uint32_t offset)
{
    const flexe_gdma_desc_t *desc = &gdma->target->gdma;
    return desc->base + channel * desc->channel_stride + offset;
}

static void gdma_rx_reset_fsm(flexe_gdma_t *gdma, unsigned channel)
{
    gdma_rx_channel_t *rx = &gdma->rx[channel];
    rx->current_desc = 0u;
    rx->previous_desc = 0u;
    rx->active = false;

    const uint32_t offsets[] = {
        GDMA_V1_IN_STATE_OFF, GDMA_V1_IN_SUC_EOF_DESC_OFF,
        GDMA_V1_IN_ERR_EOF_DESC_OFF, GDMA_V1_IN_DESC_OFF,
        GDMA_V1_IN_DESC_PREV_OFF, GDMA_V1_IN_DESC_PREV2_OFF,
    };
    for (size_t i = 0u; i < sizeof(offsets) / sizeof(offsets[0]); i++) {
        uint32_t *reg = gdma_reg(
            gdma, gdma_channel_addr(gdma, channel, offsets[i]));
        if (reg) *reg = 0u;
    }
    uint32_t *link = gdma_reg(
        gdma, gdma_channel_addr(gdma, channel, GDMA_V1_IN_LINK_OFF));
    if (link)
        *link = rx->link_address |
                (rx->auto_return ? GDMA_V1_IN_LINK_AUTO_RETURN : 0u) |
                GDMA_V1_IN_LINK_PARK;
}

static void gdma_rx_power_on_reset(flexe_gdma_t *gdma, unsigned channel)
{
    gdma->rx[channel].link_address = 0u;
    gdma->rx[channel].auto_return = true;
    gdma_rx_reset_fsm(gdma, channel);
    uint32_t *raw = gdma_reg(
        gdma, gdma_channel_addr(gdma, channel, GDMA_V1_IN_INT_RAW_OFF));
    uint32_t *ena = gdma_reg(
        gdma, gdma_channel_addr(gdma, channel, GDMA_V1_IN_INT_ENA_OFF));
    uint32_t *conf1 = gdma_reg(
        gdma, gdma_channel_addr(gdma, channel, GDMA_V1_IN_CONF1_OFF));
    uint32_t *peri = gdma_reg(
        gdma, gdma_channel_addr(gdma, channel, GDMA_V1_IN_PERI_SEL_OFF));
    if (raw) *raw = 0u;
    if (ena) *ena = 0u;
    if (conf1) *conf1 = 0xCu; /* reset RX FIFO full threshold */
    if (peri) *peri = GDMA_V1_PERI_SEL_RESET;
}

static void gdma_tx_reset_fsm(flexe_gdma_t *gdma, unsigned channel)
{
    gdma_tx_channel_t *tx = &gdma->tx[channel];
    tx->current_desc = 0u;
    tx->previous_desc = 0u;
    tx->active = false;

    const uint32_t offsets[] = {
        GDMA_V1_OUT_STATE_OFF, GDMA_V1_OUT_EOF_DESC_OFF,
        GDMA_V1_OUT_EOF_PREV_OFF, GDMA_V1_OUT_DESC_OFF,
        GDMA_V1_OUT_DESC_PREV_OFF, GDMA_V1_OUT_DESC_PREV2_OFF,
    };
    for (size_t i = 0u; i < sizeof(offsets) / sizeof(offsets[0]); i++) {
        uint32_t *reg = gdma_reg(
            gdma, gdma_channel_addr(gdma, channel, offsets[i]));
        if (reg) *reg = 0u;
    }
    uint32_t *link = gdma_reg(
        gdma, gdma_channel_addr(gdma, channel, GDMA_V1_OUT_LINK_OFF));
    if (link) *link = tx->link_address | GDMA_V1_LINK_PARK;
}

static void gdma_tx_power_on_reset(flexe_gdma_t *gdma, unsigned channel)
{
    gdma->tx[channel].link_address = 0u;
    gdma_tx_reset_fsm(gdma, channel);
    uint32_t *raw = gdma_reg(
        gdma, gdma_channel_addr(gdma, channel, GDMA_V1_OUT_INT_RAW_OFF));
    uint32_t *ena = gdma_reg(
        gdma, gdma_channel_addr(gdma, channel, GDMA_V1_OUT_INT_ENA_OFF));
    uint32_t *peri = gdma_reg(
        gdma, gdma_channel_addr(gdma, channel, GDMA_V1_OUT_PERI_SEL_OFF));
    if (raw) *raw = 0u;
    if (ena) *ena = 0u;
    if (peri) *peri = GDMA_V1_PERI_SEL_RESET;
}

static uint32_t gdma_read(void *ctx, uint32_t addr)
{
    flexe_gdma_t *gdma = ctx;
    unsigned channel;
    uint32_t off;
    if (!gdma_decode_channel(gdma, addr, &channel, &off))
        return gdma->fallback_read
            ? gdma->fallback_read(gdma->fallback_ctx, addr) : 0u;

    if (off == GDMA_V1_IN_LINK_OFF) {
        uint32_t value = gdma->rx[channel].link_address;
        if (gdma->rx[channel].auto_return)
            value |= GDMA_V1_IN_LINK_AUTO_RETURN;
        if (!gdma->rx[channel].active) value |= GDMA_V1_IN_LINK_PARK;
        return value;
    }
    if (off == GDMA_V1_IN_INT_ST_OFF) {
        uint32_t *raw = gdma_reg(gdma, gdma_channel_addr(
            gdma, channel, GDMA_V1_IN_INT_RAW_OFF));
        uint32_t *ena = gdma_reg(gdma, gdma_channel_addr(
            gdma, channel, GDMA_V1_IN_INT_ENA_OFF));
        return raw && ena ? *raw & *ena : 0u;
    }
    if (off == GDMA_V1_OUT_LINK_OFF) {
        uint32_t value = gdma->tx[channel].link_address;
        if (!gdma->tx[channel].active) value |= GDMA_V1_LINK_PARK;
        return value;
    }
    if (off == GDMA_V1_OUT_INT_ST_OFF) {
        uint32_t *raw = gdma_reg(gdma, gdma_channel_addr(
            gdma, channel, GDMA_V1_OUT_INT_RAW_OFF));
        uint32_t *ena = gdma_reg(gdma, gdma_channel_addr(
            gdma, channel, GDMA_V1_OUT_INT_ENA_OFF));
        return raw && ena ? *raw & *ena : 0u;
    }
    uint32_t *reg = gdma_reg(gdma, addr);
    return reg ? *reg : 0u;
}

static void gdma_write(void *ctx, uint32_t addr, uint32_t value)
{
    flexe_gdma_t *gdma = ctx;
    unsigned channel;
    uint32_t off;
    if (!gdma_decode_channel(gdma, addr, &channel, &off)) {
        if (gdma->fallback_write)
            gdma->fallback_write(gdma->fallback_ctx, addr, value);
        return;
    }

    uint32_t *reg = gdma_reg(gdma, addr);
    if (!reg) return;
    if (off == GDMA_V1_IN_CONF0_OFF) {
        if (value & GDMA_V1_IN_RESET) gdma_rx_reset_fsm(gdma, channel);
        *reg = value & ~GDMA_V1_IN_RESET;
        return;
    }
    if (off == GDMA_V1_IN_INT_ST_OFF || off == GDMA_V1_IN_STATE_OFF ||
        (off >= GDMA_V1_IN_SUC_EOF_DESC_OFF &&
         off <= GDMA_V1_IN_DESC_PREV2_OFF))
        return; /* Read-only RX status. */
    if (off == GDMA_V1_IN_INT_RAW_OFF)
        return; /* Read-only raw interrupt status. */
    if (off == GDMA_V1_IN_INT_ENA_OFF) {
        *reg = value & GDMA_V1_IN_INT_MASK;
        return;
    }
    if (off == GDMA_V1_IN_INT_CLR_OFF) {
        uint32_t *raw = gdma_reg(gdma, gdma_channel_addr(
            gdma, channel, GDMA_V1_IN_INT_RAW_OFF));
        if (raw) *raw &= ~(value & GDMA_V1_IN_INT_MASK);
        return;
    }
    if (off == GDMA_V1_IN_LINK_OFF) {
        gdma_rx_channel_t *rx = &gdma->rx[channel];
        rx->link_address = value & GDMA_V1_LINK_ADDR_MASK;
        rx->auto_return = (value & GDMA_V1_IN_LINK_AUTO_RETURN) != 0u;
        if (value & GDMA_V1_IN_LINK_STOP) rx->active = false;
        if (value & (GDMA_V1_IN_LINK_START | GDMA_V1_IN_LINK_RESTART))
            rx->active = true;
        *reg = rx->link_address |
               (rx->auto_return ? GDMA_V1_IN_LINK_AUTO_RETURN : 0u) |
               (rx->active ? 0u : GDMA_V1_IN_LINK_PARK);
        return;
    }
    if (off == GDMA_V1_IN_PERI_SEL_OFF) {
        *reg = value & GDMA_V1_PERI_SEL_MASK;
        return;
    }
    if (off == GDMA_V1_OUT_CONF0_OFF) {
        if (value & GDMA_V1_OUT_RESET) gdma_tx_reset_fsm(gdma, channel);
        *reg = value & ~GDMA_V1_OUT_RESET;
        return;
    }
    if (off == GDMA_V1_OUT_INT_ST_OFF || off == GDMA_V1_OUT_STATE_OFF ||
        (off >= GDMA_V1_OUT_EOF_DESC_OFF &&
         off <= GDMA_V1_OUT_DESC_PREV2_OFF))
        return; /* Read-only status. */
    if (off == GDMA_V1_OUT_INT_RAW_OFF)
        return; /* Read-only raw interrupt status. */
    if (off == GDMA_V1_OUT_INT_ENA_OFF) {
        *reg = value & GDMA_V1_OUT_INT_MASK;
        return;
    }
    if (off == GDMA_V1_OUT_INT_CLR_OFF) {
        uint32_t *raw = gdma_reg(gdma, gdma_channel_addr(
            gdma, channel, GDMA_V1_OUT_INT_RAW_OFF));
        if (raw) *raw &= ~(value & GDMA_V1_OUT_INT_MASK);
        return;
    }
    if (off == GDMA_V1_OUT_LINK_OFF) {
        gdma_tx_channel_t *tx = &gdma->tx[channel];
        tx->link_address = value & GDMA_V1_LINK_ADDR_MASK;
        if (value & GDMA_V1_LINK_STOP) tx->active = false;
        if (value & (GDMA_V1_LINK_START | GDMA_V1_LINK_RESTART))
            tx->active = true;
        *reg = tx->link_address | (tx->active ? 0u : GDMA_V1_LINK_PARK);
        return;
    }
    if (off == GDMA_V1_OUT_PERI_SEL_OFF) {
        *reg = value & GDMA_V1_PERI_SEL_MASK;
        return;
    }
    *reg = value;
}

static bool gdma_ram_range_valid(xtensa_mem_t *mem, uint32_t address,
                                 size_t length, bool writable)
{
    if (length == 0u || length > UINT32_MAX ||
        address > UINT32_MAX - (uint32_t)(length - 1u))
        return false;
    uint32_t last = address + (uint32_t)length - 1u;
    for (uint32_t page = address & ~0xFFFu;; page += 0x1000u) {
        uint32_t probe = page < address ? address : page;
        if (writable ? !mem_get_ptr_w(mem, probe) : !mem_get_ptr(mem, probe))
            return false;
        if (page >= (last & ~0xFFFu)) break;
    }
    return true;
}

static void gdma_tx_finish(flexe_gdma_t *gdma, unsigned channel,
                           bool success, bool eof)
{
    gdma_tx_channel_t *tx = &gdma->tx[channel];
    tx->active = false;
    uint32_t *link = gdma_reg(gdma, gdma_channel_addr(
        gdma, channel, GDMA_V1_OUT_LINK_OFF));
    uint32_t *raw = gdma_reg(gdma, gdma_channel_addr(
        gdma, channel, GDMA_V1_OUT_INT_RAW_OFF));
    uint32_t *eof_desc = gdma_reg(gdma, gdma_channel_addr(
        gdma, channel, GDMA_V1_OUT_EOF_DESC_OFF));
    uint32_t *eof_prev = gdma_reg(gdma, gdma_channel_addr(
        gdma, channel, GDMA_V1_OUT_EOF_PREV_OFF));
    if (link) *link = tx->link_address | GDMA_V1_LINK_PARK;
    if (raw) {
        if (success)
            *raw |= GDMA_V1_OUT_DONE_INT | GDMA_V1_OUT_TOTAL_EOF_INT |
                    (eof ? GDMA_V1_OUT_EOF_INT : 0u);
        else
            *raw |= GDMA_V1_OUT_DSCR_ERR_INT;
    }
    if (success && eof_desc) *eof_desc = tx->current_desc;
    if (success && eof_prev) *eof_prev = tx->previous_desc;
}

static void gdma_rx_finish(flexe_gdma_t *gdma, unsigned channel,
                           uint32_t interrupt_status)
{
    gdma_rx_channel_t *rx = &gdma->rx[channel];
    rx->active = false;
    uint32_t *link = gdma_reg(gdma, gdma_channel_addr(
        gdma, channel, GDMA_V1_IN_LINK_OFF));
    uint32_t *raw = gdma_reg(gdma, gdma_channel_addr(
        gdma, channel, GDMA_V1_IN_INT_RAW_OFF));
    uint32_t *success_desc = gdma_reg(gdma, gdma_channel_addr(
        gdma, channel, GDMA_V1_IN_SUC_EOF_DESC_OFF));
    uint32_t *error_desc = gdma_reg(gdma, gdma_channel_addr(
        gdma, channel, GDMA_V1_IN_ERR_EOF_DESC_OFF));
    if (link)
        *link = rx->link_address |
                (rx->auto_return ? GDMA_V1_IN_LINK_AUTO_RETURN : 0u) |
                GDMA_V1_IN_LINK_PARK;
    if (raw) *raw |= interrupt_status & GDMA_V1_IN_INT_MASK;
    if ((interrupt_status & GDMA_V1_IN_SUC_EOF_INT) && success_desc)
        *success_desc = rx->current_desc;
    if ((interrupt_status & GDMA_V1_IN_ERR_EOF_INT) && error_desc)
        *error_desc = rx->current_desc;
}

int flexe_gdma_write_rx(flexe_gdma_t *gdma, uint8_t peripheral_id,
                        const uint8_t *data, size_t length)
{
    if (!gdma || (!data && length != 0u)) return -1;
    const flexe_gdma_desc_t *geometry = &gdma->target->gdma;
    unsigned channel = geometry->channel_count;
    for (unsigned i = 0u; i < geometry->channel_count; i++) {
        uint32_t *peri = gdma_reg(gdma, gdma_channel_addr(
            gdma, i, GDMA_V1_IN_PERI_SEL_OFF));
        if (gdma->rx[i].active && peri &&
            (*peri & GDMA_V1_PERI_SEL_MASK) == peripheral_id) {
            channel = i;
            break;
        }
    }
    if (channel == geometry->channel_count) return -1;

    gdma_rx_channel_t *rx = &gdma->rx[channel];
    uint32_t desc = geometry->descriptor_address_prefix |
                    rx->link_address;
    uint32_t *conf1 = gdma_reg(gdma, gdma_channel_addr(
        gdma, channel, GDMA_V1_IN_CONF1_OFF));
    bool check_owner = conf1 && (*conf1 & GDMA_V1_IN_CHECK_OWNER) != 0u;
    size_t copied = 0u;
    for (unsigned count = 0u; count < GDMA_V1_DESC_LIMIT && copied < length;
         count++) {
        if ((desc & 3u) != 0u ||
            !gdma_ram_range_valid(gdma->mem, desc, GDMA_V1_DESC_BYTES,
                                  true)) {
            rx->current_desc = desc;
            gdma_rx_finish(gdma, channel, GDMA_V1_IN_DSCR_ERR_INT);
            return -1;
        }
        uint32_t dw0 = mem_read32(gdma->mem, desc);
        uint32_t buffer = mem_read32(gdma->mem, desc + 4u);
        uint32_t next = mem_read32(gdma->mem, desc + 8u);
        size_t size = dw0 & GDMA_V1_DESC_SIZE_MASK;
        if ((check_owner && (dw0 & GDMA_V1_DESC_OWNER) == 0u) ||
            size == 0u ||
            !gdma_ram_range_valid(gdma->mem, buffer, size, true)) {
            rx->current_desc = desc;
            gdma_rx_finish(gdma, channel, GDMA_V1_IN_DSCR_ERR_INT);
            return -1;
        }

        size_t chunk = size;
        if (chunk > length - copied) chunk = length - copied;
        for (size_t i = 0u; i < chunk; i++)
            mem_write8(gdma->mem, buffer + (uint32_t)i, data[copied + i]);
        copied += chunk;

        rx->previous_desc = rx->current_desc;
        rx->current_desc = desc;
        uint32_t *current = gdma_reg(gdma, gdma_channel_addr(
            gdma, channel, GDMA_V1_IN_DESC_OFF));
        uint32_t *previous = gdma_reg(gdma, gdma_channel_addr(
            gdma, channel, GDMA_V1_IN_DESC_PREV_OFF));
        uint32_t *previous2 = gdma_reg(gdma, gdma_channel_addr(
            gdma, channel, GDMA_V1_IN_DESC_PREV2_OFF));
        if (previous2) *previous2 = previous ? *previous : 0u;
        if (previous) *previous = current ? *current : 0u;
        if (current) *current = desc;

        dw0 &= ~(GDMA_V1_DESC_OWNER | GDMA_V1_DESC_LENGTH_MASK);
        dw0 |= ((uint32_t)chunk << GDMA_V1_DESC_LENGTH_SHIFT) &
               GDMA_V1_DESC_LENGTH_MASK;
        mem_write32(gdma->mem, desc, dw0);
        if (copied == length) break;
        if (next == 0u) {
            gdma_rx_finish(gdma, channel,
                           GDMA_V1_IN_DONE_INT |
                           GDMA_V1_IN_DSCR_EMPTY_INT);
            return -1;
        }
        if (next == desc) {
            gdma_rx_finish(gdma, channel, GDMA_V1_IN_DSCR_ERR_INT);
            return -1;
        }
        desc = next;
    }

    bool success = copied == length;
    gdma_rx_finish(gdma, channel,
                   success ? GDMA_V1_IN_DONE_INT |
                             GDMA_V1_IN_SUC_EOF_INT
                           : GDMA_V1_IN_DSCR_ERR_INT);
    return success ? 0 : -1;
}

int flexe_gdma_read_tx(flexe_gdma_t *gdma, uint8_t peripheral_id,
                       uint8_t *data, size_t length)
{
    if (!gdma || (!data && length != 0u)) return -1;
    const flexe_gdma_desc_t *geometry = &gdma->target->gdma;
    unsigned channel = geometry->channel_count;
    for (unsigned i = 0u; i < geometry->channel_count; i++) {
        uint32_t *peri = gdma_reg(gdma, gdma_channel_addr(
            gdma, i, GDMA_V1_OUT_PERI_SEL_OFF));
        if (gdma->tx[i].active && peri &&
            (*peri & GDMA_V1_PERI_SEL_MASK) == peripheral_id) {
            channel = i;
            break;
        }
    }
    if (channel == geometry->channel_count) return -1;

    gdma_tx_channel_t *tx = &gdma->tx[channel];
    uint32_t desc = geometry->descriptor_address_prefix |
                    tx->link_address;
    uint32_t *conf0 = gdma_reg(gdma, gdma_channel_addr(
        gdma, channel, GDMA_V1_OUT_CONF0_OFF));
    uint32_t *conf1 = gdma_reg(gdma, gdma_channel_addr(
        gdma, channel, GDMA_V1_OUT_CONF1_OFF));
    bool auto_writeback = conf0 && (*conf0 & GDMA_V1_OUT_AUTO_WRBACK) != 0u;
    bool check_owner = conf1 && (*conf1 & GDMA_V1_OUT_CHECK_OWNER) != 0u;
    size_t copied = 0u;
    bool eof = false;
    for (unsigned count = 0u; count < GDMA_V1_DESC_LIMIT && copied < length;
         count++) {
        if ((desc & 3u) != 0u ||
            !gdma_ram_range_valid(gdma->mem, desc, GDMA_V1_DESC_BYTES,
                                  auto_writeback)) {
            gdma_tx_finish(gdma, channel, false, false);
            return -1;
        }
        uint32_t dw0 = mem_read32(gdma->mem, desc);
        uint32_t buffer = mem_read32(gdma->mem, desc + 4u);
        uint32_t next = mem_read32(gdma->mem, desc + 8u);
        size_t size = dw0 & GDMA_V1_DESC_SIZE_MASK;
        size_t valid = (dw0 & GDMA_V1_DESC_LENGTH_MASK) >>
                       GDMA_V1_DESC_LENGTH_SHIFT;
        eof = (dw0 & GDMA_V1_DESC_EOF) != 0u;
        if ((check_owner && (dw0 & GDMA_V1_DESC_OWNER) == 0u) ||
            valid == 0u || valid > size || valid > length - copied ||
            !gdma_ram_range_valid(gdma->mem, buffer, valid, false)) {
            gdma_tx_finish(gdma, channel, false, false);
            return -1;
        }

        for (size_t i = 0u; i < valid; i++)
            data[copied + i] = mem_read8(gdma->mem, buffer + (uint32_t)i);
        copied += valid;

        tx->previous_desc = tx->current_desc;
        tx->current_desc = desc;
        uint32_t *current = gdma_reg(gdma, gdma_channel_addr(
            gdma, channel, GDMA_V1_OUT_DESC_OFF));
        uint32_t *previous = gdma_reg(gdma, gdma_channel_addr(
            gdma, channel, GDMA_V1_OUT_DESC_PREV_OFF));
        uint32_t *previous2 = gdma_reg(gdma, gdma_channel_addr(
            gdma, channel, GDMA_V1_OUT_DESC_PREV2_OFF));
        if (previous2) *previous2 = previous ? *previous : 0u;
        if (previous) *previous = current ? *current : 0u;
        if (current) *current = desc;
        if (auto_writeback)
            mem_write32(gdma->mem, desc, dw0 & ~GDMA_V1_DESC_OWNER);

        if (copied == length) {
            if (!eof) {
                gdma_tx_finish(gdma, channel, false, false);
                return -1;
            }
            break;
        }
        if (eof || next == 0u || next == desc) {
            gdma_tx_finish(gdma, channel, false, eof);
            return -1;
        }
        desc = next;
    }

    bool success = copied == length;
    gdma_tx_finish(gdma, channel, success, eof);
    return success ? 0 : -1;
}

flexe_gdma_t *flexe_gdma_create(
    xtensa_mem_t *mem, mmio_read_fn fallback_read,
    mmio_write_fn fallback_write, void *fallback_ctx)
{
    const flexe_target_desc_t *target = mem_target(mem);
    if (!mem || !gdma_geometry_valid(target)) return NULL;

    flexe_gdma_t *gdma = calloc(1u, sizeof(*gdma));
    if (!gdma) return NULL;
    gdma->regs = calloc(target->gdma.register_size / sizeof(uint32_t),
                        sizeof(uint32_t));
    if (!gdma->regs) {
        free(gdma);
        return NULL;
    }
    gdma->mem = mem;
    gdma->target = target;
    gdma->fallback_read = fallback_read;
    gdma->fallback_write = fallback_write;
    gdma->fallback_ctx = fallback_ctx;
    for (unsigned channel = 0u; channel < target->gdma.channel_count;
         channel++) {
        gdma_rx_power_on_reset(gdma, channel);
        gdma_tx_power_on_reset(gdma, channel);
    }

    if (mem_register_mmio_range(mem, target->gdma.base,
                                target->gdma.register_size,
                                gdma_read, gdma_write, gdma) != 0) {
        free(gdma->regs);
        free(gdma);
        return NULL;
    }
    return gdma;
}

void flexe_gdma_destroy(flexe_gdma_t *gdma)
{
    if (!gdma) return;
    (void)mem_register_mmio_range(
        gdma->mem, gdma->target->gdma.base,
        gdma->target->gdma.register_size,
        gdma->fallback_read, gdma->fallback_write, gdma->fallback_ctx);
    free(gdma->regs);
    free(gdma);
}
