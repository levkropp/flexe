#include "rmt_v1.h"

#include <limits.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

/* ESP32-S3 RMT V1 register layout from Espressif's rmt_reg.h. Four TX and
 * four RX channels share a 384-word pulse RAM; RX is intentionally left to
 * the fallback handler until an input waveform source is modeled. */
#define RMT_TX_CONF_OFF       0x020u
#define RMT_STATUS_OFF        0x050u
#define RMT_INT_RAW_OFF       0x070u
#define RMT_INT_ST_OFF        0x074u
#define RMT_INT_ENA_OFF       0x078u
#define RMT_INT_CLR_OFF       0x07Cu
#define RMT_CARRIER_OFF       0x080u
#define RMT_TX_LIMIT_OFF      0x0A0u
#define RMT_SYS_CONF_OFF      0x0C0u
#define RMT_TX_SIM_OFF        0x0C4u
#define RMT_DATE_OFF          0x0CCu
#define RMT_DATE_RESET        34607489u
#define RMT_TX_LIMIT_REG_MASK 0x003FFFFFu
#define RMT_LOOP_COUNT_RESET  (1u << 20)

#define RMT_TX_START          (1u << 0)
#define RMT_MEM_RD_RST        (1u << 1)
#define RMT_APB_MEM_RST       (1u << 2)
#define RMT_TX_CONTINUOUS     (1u << 3)
#define RMT_MEM_TX_WRAP       (1u << 4)
#define RMT_TX_STOP           (1u << 7)
#define RMT_AFIFO_RST         (1u << 23)
#define RMT_CONF_UPDATE       (1u << 24)
#define RMT_DMA_ACCESS        (1u << 25)
#define RMT_COMMAND_MASK      (RMT_TX_START | RMT_MEM_RD_RST | \
                               RMT_APB_MEM_RST | RMT_TX_STOP | \
                               RMT_AFIFO_RST | RMT_CONF_UPDATE | \
                               RMT_DMA_ACCESS)

#define RMT_TX_END_INT(ch)    (1u << (ch))
#define RMT_ERROR_INT(ch)     (1u << (4u + (ch)))
#define RMT_THRESHOLD_INT(ch) (1u << (8u + (ch)))

#define RMT_MAX_CHANNELS      8u
#define RMT_MAX_TX_CHANNELS   4u
#define RMT_MAX_WORDS         (RMT_MAX_CHANNELS * 48u)
#define RMT_MAX_SEGMENT_WORDS (RMT_MAX_CHANNELS * 48u)

typedef enum {
    RMT_EVENT_NONE,
    RMT_EVENT_THRESHOLD,
    RMT_EVENT_END,
    RMT_EVENT_ERROR,
} rmt_event_kind_t;

typedef struct {
    uint32_t conf;
    uint32_t carrier;
    uint32_t tx_limit;
    uint32_t read_index;
    uint32_t apb_index;
    uint32_t segment_items[RMT_MAX_SEGMENT_WORDS];
    uint32_t segment_count;
    uint64_t deadline;
    rmt_event_kind_t event;
    bool active;
} rmt_tx_channel_t;

struct flexe_rmt_v1 {
    xtensa_mem_t *mem;
    const flexe_rmt_v1_desc_t *desc;
    mmio_read_fn fallback_read;
    mmio_write_fn fallback_write;
    void *fallback_ctx;
    flexe_rmt_v1_state_fn state_changed;
    void *state_ctx;
    flexe_rmt_v1_irq_fn irq_changed;
    void *irq_ctx;
    flexe_rmt_v1_tx_fn tx_cb[RMT_MAX_TX_CHANNELS];
    void *tx_ctx[RMT_MAX_TX_CHANNELS];
    xtensa_cpu_t *cpu[2];
    uint64_t core_cycles[2];
    uint32_t last_ccount[2];
    bool clock_valid[2];
    uint64_t cycles;
    uint32_t memory[RMT_MAX_WORDS];
    rmt_tx_channel_t tx[RMT_MAX_TX_CHANNELS];
    uint32_t int_raw;
    uint32_t int_ena;
    uint32_t sys_conf;
    uint32_t tx_sim;
    uint32_t date;
};

static uint64_t rmt_gcd(uint64_t a, uint64_t b)
{
    while (b) {
        uint64_t remainder = a % b;
        a = b;
        b = remainder;
    }
    return a;
}

static uint64_t rmt_mul_saturating(uint64_t a, uint64_t b)
{
    return b && a > UINT64_MAX / b ? UINT64_MAX : a * b;
}

static uint64_t rmt_clock_now(flexe_rmt_v1_t *rmt)
{
    for (unsigned core = 0u; core < 2u; core++) {
        xtensa_cpu_t *cpu = rmt->cpu[core];
        if (!cpu) continue;
        uint32_t now = cpu->ccount;
        if (!rmt->clock_valid[core]) {
            rmt->last_ccount[core] = now;
            rmt->core_cycles[core] = rmt->cycles;
            rmt->clock_valid[core] = true;
            continue;
        }
        uint32_t elapsed = now - rmt->last_ccount[core];
        if (elapsed < (uint32_t)INT32_MAX) {
            rmt->core_cycles[core] =
                rmt->core_cycles[core] > UINT64_MAX - elapsed ?
                UINT64_MAX : rmt->core_cycles[core] + elapsed;
            if (rmt->core_cycles[core] > rmt->cycles)
                rmt->cycles = rmt->core_cycles[core];
        }
        rmt->last_ccount[core] = now;
    }
    return rmt->cycles;
}

static void rmt_notify_state(flexe_rmt_v1_t *rmt)
{
    if (rmt->state_changed) rmt->state_changed(rmt->state_ctx);
}

static void rmt_notify_irq(flexe_rmt_v1_t *rmt)
{
    if (rmt->irq_changed)
        rmt->irq_changed(rmt->irq_ctx, rmt->int_raw & rmt->int_ena);
}

static uint32_t rmt_source_hz(const flexe_rmt_v1_t *rmt)
{
    switch ((rmt->sys_conf >> 24) & 3u) {
    case 1u: return rmt->desc->apb_clock_hz;
    case 2u: return rmt->desc->ref_clock_hz;
    case 3u: return rmt->desc->xtal_clock_hz;
    default: return 0u;
    }
}

static bool rmt_tick_ratio(const flexe_rmt_v1_t *rmt,
                           const rmt_tx_channel_t *tx,
                           uint64_t *source_hz, uint64_t *div_num,
                           uint64_t *div_den)
{
    uint32_t source = rmt_source_hz(rmt);
    uint32_t channel_div = (tx->conf >> 8) & 0xFFu;
    uint32_t sclk_div = ((rmt->sys_conf >> 4) & 0xFFu) + 1u;
    if (!source) return false;
    /* The group divider is stored minus one; the channel divider uses
     * zero to encode divide-by-256. Both encodings come from rmt_ll.h. */
    if (!channel_div) channel_div = 256u;
    /* SCLK_DIV_A is the fractional numerator, B the denominator. A
     * zero B means there is no fractional part. */
    uint32_t frac_num = (rmt->sys_conf >> 12) & 0x3Fu;
    uint32_t frac_den = (rmt->sys_conf >> 18) & 0x3Fu;
    uint32_t den = frac_den ? frac_den : 1u;
    *source_hz = source;
    *div_num = (uint64_t)channel_div *
               ((uint64_t)sclk_div * den +
                (frac_den ? frac_num : 0u));
    *div_den = den;
    return true;
}

static uint32_t rmt_tick_hz(const flexe_rmt_v1_t *rmt,
                             const rmt_tx_channel_t *tx)
{
    uint64_t source, div_num, div_den;
    if (!rmt_tick_ratio(rmt, tx, &source, &div_num, &div_den)) return 0u;
    return (uint32_t)((source * div_den) / div_num);
}

static uint64_t rmt_ticks_to_cycles(const flexe_rmt_v1_t *rmt,
                                     const rmt_tx_channel_t *tx,
                                     uint64_t ticks)
{
    uint64_t source, div_num, div_den;
    uint32_t cpu_mhz = rmt->cpu[0] ?
        xtensa_cpu_freq_mhz(rmt->cpu[0]) :
        rmt->desc->apb_clock_hz / 1000000u;
    if (!rmt_tick_ratio(rmt, tx, &source, &div_num, &div_den) ||
        !cpu_mhz) return 1u;
    /* Reduce the host/target clock ratio before multiplication, keeping
     * exact fractional-divider deadlines without a 128-bit dependency. */
    uint64_t cpu_hz = (uint64_t)cpu_mhz * 1000000u;
    uint64_t common = rmt_gcd(cpu_hz, source);
    uint64_t numerator = rmt_mul_saturating(
        rmt_mul_saturating(ticks, cpu_hz / common), div_num);
    uint64_t denominator = (source / common) * div_den;
    uint64_t cycles = numerator / denominator +
                      (numerator % denominator != 0u);
    return cycles ? cycles : 1u;
}

static uint32_t rmt_capacity(const flexe_rmt_v1_t *rmt, unsigned channel)
{
    uint32_t blocks = (rmt->tx[channel].conf >> 16) & 0xFu;
    uint32_t available = rmt->desc->tx_channel_count - channel;
    if (blocks > available) blocks = available;
    return blocks * rmt->desc->words_per_channel;
}

static void rmt_plan_segment(flexe_rmt_v1_t *rmt, unsigned channel,
                              uint64_t start_cycle)
{
    rmt_tx_channel_t *tx = &rmt->tx[channel];
    uint32_t capacity = rmt_capacity(rmt, channel);
    uint32_t limit = tx->tx_limit & 0x1FFu;
    uint64_t ticks = 0u;
    tx->segment_count = 0u;
    tx->event = RMT_EVENT_NONE;
    if (capacity == 0u || !rmt_source_hz(rmt)) {
        tx->event = RMT_EVENT_ERROR;
        tx->deadline = start_cycle + 1u;
        return;
    }
    if (limit == 0u || limit > capacity) limit = capacity;

    while (tx->segment_count < limit) {
        if (tx->read_index >= capacity) {
            if (tx->conf & RMT_MEM_TX_WRAP)
                tx->read_index = 0u;
            else {
                tx->event = RMT_EVENT_ERROR;
                break;
            }
        }
        unsigned word_index = channel * rmt->desc->words_per_channel +
                              tx->read_index;
        if (word_index >= RMT_MAX_WORDS) {
            tx->event = RMT_EVENT_ERROR;
            break;
        }
        uint32_t item = rmt->memory[word_index];
        uint32_t low = item & 0x7FFFu;
        uint32_t high = (item >> 16) & 0x7FFFu;
        if (low == 0u) {
            tx->event = RMT_EVENT_END;
            break;
        }
        tx->segment_items[tx->segment_count++] = item;
        tx->read_index++;
        ticks += low;
        if (high == 0u) {
            tx->event = RMT_EVENT_END;
            break;
        }
        ticks += high;
    }
    if (tx->event == RMT_EVENT_NONE) {
        if (tx->segment_count == limit && limit < capacity)
            tx->event = RMT_EVENT_THRESHOLD;
        else if (tx->conf & (RMT_MEM_TX_WRAP | RMT_TX_CONTINUOUS))
            tx->event = RMT_EVENT_THRESHOLD;
        else
            tx->event = RMT_EVENT_ERROR;
    }
    uint64_t duration = rmt_ticks_to_cycles(rmt, tx, ticks);
    tx->deadline = start_cycle > UINT64_MAX - duration ?
                   UINT64_MAX : start_cycle + duration;
}

static void rmt_finish_segment(flexe_rmt_v1_t *rmt, unsigned channel,
                                uint64_t event_cycle)
{
    rmt_tx_channel_t *tx = &rmt->tx[channel];
    rmt_event_kind_t event = tx->event;
    tx->event = RMT_EVENT_NONE;
    if (rmt->tx_cb[channel] &&
        (tx->segment_count || event == RMT_EVENT_END ||
         event == RMT_EVENT_ERROR)) {
        uint32_t tick_hz = rmt_tick_hz(rmt, tx);
        uint32_t carrier_hz = 0u;
        uint32_t carrier_period = (tx->carrier & 0xFFFFu) +
                                  (tx->carrier >> 16);
        if ((tx->conf & (1u << 21)) && carrier_period)
            carrier_hz = tick_hz / carrier_period;
        rmt->tx_cb[channel](rmt->tx_ctx[channel], (int)channel,
                            tx->segment_items, tx->segment_count,
                            tick_hz, carrier_hz,
                            event == RMT_EVENT_END || event == RMT_EVENT_ERROR);
    }
    switch (event) {
    case RMT_EVENT_END:
        tx->active = false;
        rmt->int_raw |= RMT_TX_END_INT(channel);
        break;
    case RMT_EVENT_ERROR:
        tx->active = false;
        rmt->int_raw |= RMT_ERROR_INT(channel);
        break;
    case RMT_EVENT_THRESHOLD:
        rmt->int_raw |= RMT_THRESHOLD_INT(channel);
        break;
    case RMT_EVENT_NONE:
        break;
    }
    rmt_notify_irq(rmt);
    if (tx->active && event == RMT_EVENT_THRESHOLD)
        rmt_plan_segment(rmt, channel, event_cycle);
}

void flexe_rmt_v1_eval(flexe_rmt_v1_t *rmt)
{
    if (!rmt) return;
    uint64_t now = rmt_clock_now(rmt);
    bool changed = false;
    for (unsigned channel = 0u; channel < rmt->desc->tx_channel_count;
         channel++) {
        rmt_tx_channel_t *tx = &rmt->tx[channel];
        unsigned safety = 0u;
        while (tx->active && tx->event != RMT_EVENT_NONE &&
               tx->deadline <= now && safety++ < 1024u) {
            uint64_t event_cycle = tx->deadline;
            rmt_finish_segment(rmt, channel, event_cycle);
            changed = true;
        }
    }
    if (changed) rmt_notify_state(rmt);
}

uint32_t flexe_rmt_v1_next_event(flexe_rmt_v1_t *rmt,
                                  xtensa_cpu_t *cpu)
{
    if (!rmt || !cpu) return UINT32_MAX;
    uint64_t now = rmt_clock_now(rmt);
    uint64_t distance = UINT64_MAX;
    for (unsigned channel = 0u; channel < rmt->desc->tx_channel_count;
         channel++) {
        const rmt_tx_channel_t *tx = &rmt->tx[channel];
        if (!tx->active || tx->event == RMT_EVENT_NONE) continue;
        uint64_t d = tx->deadline > now ? tx->deadline - now : 0u;
        if (d < distance) distance = d;
    }
    if (distance == UINT64_MAX) return UINT32_MAX;
    if (distance > (uint32_t)INT32_MAX) distance = INT32_MAX;
    return cpu->ccount + (uint32_t)distance;
}

static uint32_t rmt_status(const flexe_rmt_v1_t *rmt, unsigned channel)
{
    if (channel >= rmt->desc->tx_channel_count) return 0u;
    const rmt_tx_channel_t *tx = &rmt->tx[channel];
    return (tx->read_index & 0x3FFu) |
           ((tx->apb_index & 0x3FFu) << 11) |
           (tx->active ? 1u << 22 : 0u);
}

static uint32_t rmt_read(void *ctx, uint32_t address)
{
    flexe_rmt_v1_t *rmt = ctx;
    uint32_t off = address - rmt->desc->base;
    if ((off & 3u) != 0u || off >= rmt->desc->register_size)
        return rmt->fallback_read(rmt->fallback_ctx, address);
    flexe_rmt_v1_eval(rmt);
    if (off >= rmt->desc->memory_offset &&
        off < rmt->desc->memory_offset +
              rmt->desc->channel_count * rmt->desc->words_per_channel * 4u)
        return rmt->memory[(off - rmt->desc->memory_offset) / 4u];
    if (off >= RMT_TX_CONF_OFF && off < RMT_TX_CONF_OFF +
                                            rmt->desc->tx_channel_count * 4u)
        return rmt->tx[(off - RMT_TX_CONF_OFF) / 4u].conf;
    if (off >= RMT_STATUS_OFF && off < RMT_STATUS_OFF +
                                        rmt->desc->tx_channel_count * 4u)
        return rmt_status(rmt, (off - RMT_STATUS_OFF) / 4u);
    if (off == RMT_INT_RAW_OFF) return rmt->int_raw;
    if (off == RMT_INT_ST_OFF) return rmt->int_raw & rmt->int_ena;
    if (off == RMT_INT_ENA_OFF) return rmt->int_ena;
    if (off == RMT_INT_CLR_OFF) return 0u;
    if (off >= RMT_CARRIER_OFF && off < RMT_CARRIER_OFF +
                                         rmt->desc->tx_channel_count * 4u)
        return rmt->tx[(off - RMT_CARRIER_OFF) / 4u].carrier;
    if (off >= RMT_TX_LIMIT_OFF && off < RMT_TX_LIMIT_OFF +
                                          rmt->desc->tx_channel_count * 4u)
        return rmt->tx[(off - RMT_TX_LIMIT_OFF) / 4u].tx_limit;
    if (off == RMT_SYS_CONF_OFF) return rmt->sys_conf;
    if (off == RMT_TX_SIM_OFF) return rmt->tx_sim;
    if (off == RMT_DATE_OFF) return rmt->date;
    return rmt->fallback_read(rmt->fallback_ctx, address);
}

static void rmt_write(void *ctx, uint32_t address, uint32_t value)
{
    flexe_rmt_v1_t *rmt = ctx;
    uint32_t off = address - rmt->desc->base;
    if ((off & 3u) != 0u || off >= rmt->desc->register_size) {
        rmt->fallback_write(rmt->fallback_ctx, address, value);
        return;
    }
    flexe_rmt_v1_eval(rmt);
    if (off >= rmt->desc->memory_offset &&
        off < rmt->desc->memory_offset +
              rmt->desc->channel_count * rmt->desc->words_per_channel * 4u) {
        rmt->memory[(off - rmt->desc->memory_offset) / 4u] = value;
        return;
    }
    if (off >= RMT_TX_CONF_OFF && off < RMT_TX_CONF_OFF +
                                            rmt->desc->tx_channel_count * 4u) {
        unsigned channel = (off - RMT_TX_CONF_OFF) / 4u;
        rmt_tx_channel_t *tx = &rmt->tx[channel];
        /* The finite-stream path supports wrap and threshold refills. The
         * channel loop-count engine is not modeled yet, so make that mode
         * visible in unsupported-MMIO diagnostics instead of accepting it. */
        if (value & (RMT_TX_CONTINUOUS | RMT_DMA_ACCESS))
            rmt->fallback_write(rmt->fallback_ctx, address, value);
        tx->conf = value & ~RMT_COMMAND_MASK;
        if (value & RMT_MEM_RD_RST) tx->read_index = 0u;
        if (value & RMT_APB_MEM_RST) tx->apb_index = 0u;
        if (value & RMT_TX_STOP) {
            tx->active = false;
            tx->event = RMT_EVENT_NONE;
        }
        if (value & RMT_TX_START) {
            tx->active = true;
            tx->read_index = 0u;
            rmt_plan_segment(rmt, channel, rmt_clock_now(rmt));
        }
        rmt_notify_state(rmt);
        return;
    }
    if (off == RMT_INT_ENA_OFF) {
        rmt->int_ena = value;
        rmt_notify_irq(rmt);
        return;
    }
    if (off == RMT_INT_CLR_OFF) {
        rmt->int_raw &= ~value;
        rmt_notify_irq(rmt);
        return;
    }
    if (off >= RMT_CARRIER_OFF && off < RMT_CARRIER_OFF +
                                         rmt->desc->tx_channel_count * 4u) {
        rmt->tx[(off - RMT_CARRIER_OFF) / 4u].carrier = value;
        return;
    }
    if (off >= RMT_TX_LIMIT_OFF && off < RMT_TX_LIMIT_OFF +
                                          rmt->desc->tx_channel_count * 4u) {
        if (value & ~RMT_TX_LIMIT_REG_MASK)
            rmt->fallback_write(rmt->fallback_ctx, address, value);
        rmt->tx[(off - RMT_TX_LIMIT_OFF) / 4u].tx_limit =
            value & RMT_TX_LIMIT_REG_MASK & ~RMT_LOOP_COUNT_RESET;
        return;
    }
    if (off == RMT_SYS_CONF_OFF) {
        if (value & (1u << 2))
            rmt->fallback_write(rmt->fallback_ctx, address, value);
        rmt->sys_conf = value;
        return;
    }
    if (off == RMT_TX_SIM_OFF) {
        if (value & (1u << 4))
            rmt->fallback_write(rmt->fallback_ctx, address, value);
        rmt->tx_sim = value;
        return;
    }
    if (off == RMT_DATE_OFF) {
        rmt->date = value & 0x0FFFFFFFu;
        return;
    }
    rmt->fallback_write(rmt->fallback_ctx, address, value);
}

flexe_rmt_v1_t *flexe_rmt_v1_create(xtensa_mem_t *mem,
                                    mmio_read_fn fallback_read,
                                    mmio_write_fn fallback_write,
                                    void *fallback_ctx,
                                    flexe_rmt_v1_state_fn state_changed,
                                    void *state_ctx,
                                    flexe_rmt_v1_irq_fn irq_changed,
                                    void *irq_ctx)
{
    if (!mem || !fallback_read || !fallback_write) return NULL;
    const flexe_target_desc_t *target = mem_target(mem);
    if (!target || !(target->capabilities & FLEXE_TARGET_CAP_RMT_V1))
        return NULL;
    const flexe_rmt_v1_desc_t *desc = &target->rmt_v1;
    if (!desc->base || (desc->base & 0xFFFu) ||
        desc->register_size != 0x1000u ||
        desc->tx_channel_count == 0u ||
        desc->tx_channel_count > RMT_MAX_TX_CHANNELS ||
        desc->channel_count < desc->tx_channel_count ||
        desc->channel_count > RMT_MAX_CHANNELS ||
        desc->words_per_channel != 48u ||
        desc->memory_offset != 0x800u ||
        desc->memory_offset +
            desc->channel_count * desc->words_per_channel * 4u >
            desc->register_size ||
        desc->interrupt_source >= FLEXE_TARGET_INTERRUPT_SOURCE_MAX ||
        !desc->apb_clock_hz || !desc->ref_clock_hz || !desc->xtal_clock_hz)
        return NULL;
    flexe_rmt_v1_t *rmt = calloc(1, sizeof(*rmt));
    if (!rmt) return NULL;
    rmt->mem = mem;
    rmt->desc = desc;
    rmt->fallback_read = fallback_read;
    rmt->fallback_write = fallback_write;
    rmt->fallback_ctx = fallback_ctx;
    rmt->state_changed = state_changed;
    rmt->state_ctx = state_ctx;
    rmt->irq_changed = irq_changed;
    rmt->irq_ctx = irq_ctx;
    rmt->sys_conf = 0x05000010u;
    rmt->date = RMT_DATE_RESET;
    for (unsigned channel = 0u; channel < desc->tx_channel_count; channel++)
        rmt->tx[channel].conf = 0x00710200u;
    for (unsigned channel = 0u; channel < desc->tx_channel_count; channel++)
        rmt->tx[channel].carrier = 0x00400040u;
    for (unsigned channel = 0u; channel < desc->tx_channel_count; channel++)
        rmt->tx[channel].tx_limit = 128u;
    if (mem_register_mmio_range(mem, desc->base, desc->register_size,
                                rmt_read, rmt_write, rmt) != 0) {
        free(rmt);
        return NULL;
    }
    return rmt;
}

void flexe_rmt_v1_destroy(flexe_rmt_v1_t *rmt)
{
    if (!rmt) return;
    (void)mem_register_mmio_range(rmt->mem, rmt->desc->base,
                                  rmt->desc->register_size,
                                  NULL, NULL, NULL);
    free(rmt);
}

void flexe_rmt_v1_attach_cpus(flexe_rmt_v1_t *rmt,
                               xtensa_cpu_t *cpu0, xtensa_cpu_t *cpu1)
{
    if (!rmt) return;
    rmt->cpu[0] = cpu0;
    rmt->cpu[1] = cpu1;
    for (unsigned core = 0u; core < 2u; core++) {
        xtensa_cpu_t *cpu = rmt->cpu[core];
        rmt->clock_valid[core] = cpu != NULL;
        rmt->last_ccount[core] = cpu ? cpu->ccount : 0u;
        rmt->core_cycles[core] = rmt->cycles;
    }
}

int flexe_rmt_v1_set_tx_callback(flexe_rmt_v1_t *rmt, unsigned channel,
                                  flexe_rmt_v1_tx_fn fn, void *ctx)
{
    if (!rmt || channel >= rmt->desc->tx_channel_count) return -1;
    rmt->tx_cb[channel] = fn;
    rmt->tx_ctx[channel] = fn ? ctx : NULL;
    return 0;
}
