#include "rmt_v1.h"

#include <limits.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

/* ESP32-S3 RMT V1 register layout from Espressif's rmt_reg.h. Four TX and
 * four RX channels share a 384-word pulse RAM. Host RX injection supplies
 * already-decoded symbols; GPIO-matrix edges pass through the timed RX
 * glitch filter and carrier remover before pulse decoding. */
#define RMT_TX_CONF_OFF       0x020u
#define RMT_RX_CONF_OFF       0x030u
#define RMT_STATUS_OFF        0x050u
#define RMT_RX_STATUS_OFF     0x060u
#define RMT_INT_RAW_OFF       0x070u
#define RMT_INT_ST_OFF        0x074u
#define RMT_INT_ENA_OFF       0x078u
#define RMT_INT_CLR_OFF       0x07Cu
#define RMT_CARRIER_OFF       0x080u
#define RMT_RX_CARRIER_OFF    0x090u
#define RMT_TX_LIMIT_OFF      0x0A0u
#define RMT_RX_LIMIT_OFF      0x0B0u
#define RMT_SYS_CONF_OFF      0x0C0u
#define RMT_TX_SIM_OFF        0x0C4u
#define RMT_REF_CNT_RST_OFF   0x0C8u
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
#define RMT_RX_END_INT(ch)    (1u << (16u + (ch)))
#define RMT_RX_ERROR_INT(ch)  (1u << (20u + (ch)))
#define RMT_RX_THRESHOLD_INT(ch) (1u << (24u + (ch)))

#define RMT_RX_EN             (1u << 0)
#define RMT_RX_MEM_WR_RST     (1u << 1)
#define RMT_RX_APB_MEM_RST    (1u << 2)
#define RMT_RX_MEM_OWNER     (1u << 3)
#define RMT_RX_FILTER_EN     (1u << 4)
#define RMT_RX_FILTER_THRES_SHIFT 5u
#define RMT_RX_FILTER_THRES_MASK  0xFFu
#define RMT_RX_WRAP          (1u << 13)
#define RMT_RX_AFIFO_RST     (1u << 14)
#define RMT_RX_CONF_UPDATE   (1u << 15)
#define RMT_RX_DMA_ACCESS    (1u << 23)
#define RMT_RX_DEMOD_EN      (1u << 28)
#define RMT_RX_CARRIER_LEVEL (1u << 29)
#define RMT_RX_COMMAND_MASK  (RMT_RX_MEM_WR_RST | RMT_RX_APB_MEM_RST | \
                              RMT_RX_AFIFO_RST | RMT_RX_CONF_UPDATE)

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

typedef struct {
    uint32_t conf0;
    uint32_t conf1;
    uint32_t carrier;
    uint32_t limit;
    size_t write_index;
    uint32_t apb_index;
    uint32_t status_flags;
    uint32_t *frame;
    size_t frame_count;
    size_t next_word;
    uint64_t frame_start;
    uint64_t elapsed_ticks;
    uint64_t deadline;
    bool active;
    bool pending_end;
    bool pending_error;
    bool edge_started;
    bool edge_level;
    bool edge_first_valid;
    uint32_t edge_first;
    uint64_t edge_start;
    uint64_t edge_deadline;
    bool filter_pending;
    bool filter_level;
    bool filter_target;
    uint64_t filter_deadline;
    bool demod_pending;
    uint64_t demod_start;
    uint64_t demod_deadline;
} rmt_rx_channel_t;

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
    rmt_rx_channel_t rx[RMT_MAX_CHANNELS - RMT_MAX_TX_CHANNELS];
    uint32_t int_raw;
    uint32_t int_ena;
    uint32_t sys_conf;
    uint32_t tx_sim;
    uint32_t date;
};

static unsigned rmt_rx_channel_count(const flexe_rmt_v1_desc_t *desc)
{
    return (unsigned)desc->channel_count -
           (unsigned)desc->tx_channel_count;
}

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
                           uint32_t channel_div,
                           uint64_t *source_hz, uint64_t *div_num,
                           uint64_t *div_den)
{
    uint32_t source = rmt_source_hz(rmt);
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
    if (!rmt_tick_ratio(rmt, (tx->conf >> 8) & 0xFFu,
                        &source, &div_num, &div_den)) return 0u;
    return (uint32_t)((source * div_den) / div_num);
}

static uint64_t rmt_ticks_to_cycles_div(const flexe_rmt_v1_t *rmt,
                                        uint32_t channel_div,
                                        uint64_t ticks)
{
    uint64_t source, div_num, div_den;
    uint32_t cpu_mhz = rmt->cpu[0] ?
        xtensa_cpu_freq_mhz(rmt->cpu[0]) :
        rmt->desc->apb_clock_hz / 1000000u;
    if (!rmt_tick_ratio(rmt, channel_div, &source, &div_num, &div_den) ||
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

static uint64_t rmt_ticks_to_cycles(const flexe_rmt_v1_t *rmt,
                                    const rmt_tx_channel_t *tx,
                                    uint64_t ticks)
{
    return rmt_ticks_to_cycles_div(rmt, (tx->conf >> 8) & 0xFFu, ticks);
}

static uint64_t rmt_cycles_to_ticks_div(const flexe_rmt_v1_t *rmt,
                                        uint32_t channel_div,
                                        uint64_t cycles)
{
    uint64_t source, div_num, div_den;
    uint32_t cpu_mhz = rmt->cpu[0] ?
        xtensa_cpu_freq_mhz(rmt->cpu[0]) :
        rmt->desc->apb_clock_hz / 1000000u;
    if (!rmt_tick_ratio(rmt, channel_div, &source, &div_num, &div_den) ||
        !cpu_mhz) return 0u;
    uint64_t cpu_hz = (uint64_t)cpu_mhz * 1000000u;
    uint64_t common = rmt_gcd(cpu_hz, source);
    uint64_t numerator = rmt_mul_saturating(
        rmt_mul_saturating(cycles, source / common), div_den);
    uint64_t denominator = (cpu_hz / common) * div_num;
    return numerator / denominator;
}

static uint32_t rmt_capacity(const flexe_rmt_v1_t *rmt, unsigned channel)
{
    uint32_t blocks = (rmt->tx[channel].conf >> 16) & 0xFu;
    uint32_t available = rmt->desc->tx_channel_count - channel;
    if (blocks > available) blocks = available;
    return blocks * rmt->desc->words_per_channel;
}

static uint32_t rmt_rx_capacity(const flexe_rmt_v1_t *rmt,
                                unsigned channel)
{
    unsigned physical = rmt->desc->tx_channel_count + channel;
    uint32_t blocks = (rmt->rx[channel].conf0 >> 24) & 0xFu;
    uint32_t available = rmt->desc->channel_count - physical;
    if (blocks > available) blocks = available;
    return blocks * rmt->desc->words_per_channel;
}

static void rmt_abort_rx_frame(rmt_rx_channel_t *rx)
{
    free(rx->frame);
    rx->frame = NULL;
    rx->frame_count = 0u;
    rx->next_word = 0u;
    rx->pending_end = false;
    rx->pending_error = false;
    rx->edge_started = false;
    rx->edge_first_valid = false;
    rx->edge_deadline = UINT64_MAX;
    rx->filter_pending = false;
    rx->filter_deadline = UINT64_MAX;
    rx->demod_pending = false;
    rx->demod_deadline = UINT64_MAX;
}

static void rmt_finish_rx(flexe_rmt_v1_t *rmt, unsigned channel)
{
    rmt_rx_channel_t *rx = &rmt->rx[channel];
    bool error = rx->pending_error;
    rmt_abort_rx_frame(rx);
    rx->active = false;
    if (error) rx->status_flags |= 1u << 26;
    rmt->int_raw |= error ?
        RMT_RX_ERROR_INT(channel) : RMT_RX_END_INT(channel);
    rmt_notify_irq(rmt);
}

static bool rmt_rx_store_word(flexe_rmt_v1_t *rmt, unsigned channel,
                              uint32_t word)
{
    rmt_rx_channel_t *rx = &rmt->rx[channel];
    uint32_t capacity = rmt_rx_capacity(rmt, channel);
    if (!capacity || (!(rx->conf1 & RMT_RX_WRAP) &&
                      rx->write_index >= capacity)) {
        rx->pending_error = true;
        rmt_finish_rx(rmt, channel);
        return false;
    }
    unsigned physical = rmt->desc->tx_channel_count + channel;
    unsigned base = physical * rmt->desc->words_per_channel;
    size_t slot = (rx->conf1 & RMT_RX_WRAP) ?
        rx->write_index % capacity : rx->write_index;
    rmt->memory[base + slot] = word;
    rx->write_index++;
    if (rx->limit != 0u &&
        ((rx->conf1 & RMT_RX_WRAP) ?
         rx->write_index % rx->limit == 0u :
         rx->write_index == rx->limit)) {
        rmt->int_raw |= RMT_RX_THRESHOLD_INT(channel);
        rmt_notify_irq(rmt);
    }
    return true;
}

static void rmt_rx_commit_edge(flexe_rmt_v1_t *rmt, unsigned index,
                               bool level, uint64_t now)
{
    rmt_rx_channel_t *rx = &rmt->rx[index];
    if (!rx->active) return;
    if (rx->edge_started) {
        if (rx->edge_level == level) return;
        uint64_t elapsed = now - rx->edge_start;
        uint64_t ticks = rmt_cycles_to_ticks_div(
            rmt, rx->conf0 & 0xFFu, elapsed);
        if (ticks == 0u) ticks = 1u;
        if (ticks > 0x7FFFu) {
            rx->pending_error = true;
            rmt_finish_rx(rmt, index);
            rmt_notify_state(rmt);
            return;
        }
        uint32_t half = (uint32_t)ticks |
                        ((uint32_t)rx->edge_level << 15u);
        if (!rx->edge_first_valid) {
            rx->edge_first = half;
            rx->edge_first_valid = true;
        } else {
            rx->edge_first_valid = false;
            if (!rmt_rx_store_word(rmt, index,
                                   rx->edge_first | (half << 16u))) {
                rmt_notify_state(rmt);
                return;
            }
        }
    }
    rx->edge_started = true;
    rx->edge_level = level;
    rx->edge_start = now;
    uint32_t idle_ticks = (rx->conf0 >> 8u) & 0x7FFFu;
    uint64_t idle_cycles = idle_ticks ? rmt_ticks_to_cycles_div(
        rmt, rx->conf0 & 0xFFu, idle_ticks) : UINT64_MAX;
    rx->edge_deadline = now > UINT64_MAX - idle_cycles ?
                        UINT64_MAX : now + idle_cycles;
    rmt_notify_state(rmt);
}

static void rmt_rx_accept_level(flexe_rmt_v1_t *rmt, unsigned index,
                                bool level, uint64_t now)
{
    rmt_rx_channel_t *rx = &rmt->rx[index];
    if (!(rx->conf0 & RMT_RX_DEMOD_EN)) {
        rmt_rx_commit_edge(rmt, index, level, now);
        return;
    }

    bool carrier_level = (rx->conf0 & RMT_RX_CARRIER_LEVEL) != 0u;
    if (level == carrier_level) {
        /* A short gap between carrier cycles never reaches the symbol
         * counter. A gap that qualified has already committed its edge. */
        if (rx->demod_pending) {
            rx->demod_pending = false;
            rx->demod_deadline = UINT64_MAX;
            rmt_notify_state(rmt);
        }
        if (!rx->edge_started || rx->edge_level != level)
            rmt_rx_commit_edge(rmt, index, level, now);
        return;
    }
    if (!rx->edge_started || rx->edge_level != carrier_level) return;

    /* The opposite-level gap must remain for its programmed number of
     * channel ticks before the carrier envelope ends. The register encodes
     * a period minus one (S3 RMT_CHm_RX_CARRIER_RM_REG). Preserve the
     * original edge timestamp when the gap qualifies. Same-polarity duty
     * discrimination is outside this functional envelope model. */
    uint32_t threshold = carrier_level ?
        (rx->carrier & 0xFFFFu) + 1u :
        ((rx->carrier >> 16u) & 0xFFFFu) + 1u;
    uint64_t cycles = rmt_ticks_to_cycles_div(
        rmt, rx->conf0 & 0xFFu, threshold);
    rx->demod_pending = true;
    rx->demod_start = now;
    rx->demod_deadline = now > UINT64_MAX - cycles ?
                         UINT64_MAX : now + cycles;
    rmt_notify_state(rmt);
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
    for (unsigned channel = 0u;
         channel < rmt_rx_channel_count(rmt->desc);
         channel++) {
        rmt_rx_channel_t *rx = &rmt->rx[channel];
        while (rx->active) {
            bool filter_due = rx->filter_pending &&
                              rx->filter_deadline <= now;
            bool demod_due = rx->demod_pending &&
                             rx->demod_deadline <= now;
            bool idle_due = rx->edge_started &&
                            rx->edge_deadline <= now;
            if (!filter_due && !demod_due && !idle_due) break;
            if (idle_due && (!filter_due ||
                             rx->edge_deadline <= rx->filter_deadline) &&
                            (!demod_due ||
                             rx->edge_deadline <= rx->demod_deadline)) {
                /* Idle completion precedes a later qualifying edge. A
                 * completed first half with no second edge is terminal. */
                if (rx->edge_first_valid)
                    (void)rmt_rx_store_word(rmt, channel, rx->edge_first);
                if (rx->active) rmt_finish_rx(rmt, channel);
                changed = true;
                break;
            }
            if (demod_due && (!filter_due ||
                              rx->demod_deadline <= rx->filter_deadline)) {
                uint64_t edge_cycle = rx->demod_start;
                bool inactive = (rx->conf0 & RMT_RX_CARRIER_LEVEL) == 0u;
                rx->demod_pending = false;
                rx->demod_deadline = UINT64_MAX;
                rmt_rx_commit_edge(rmt, channel, inactive, edge_cycle);
                changed = true;
                continue;
            }
            uint64_t deadline = rx->filter_deadline;
            bool level = rx->filter_target;
            rx->filter_pending = false;
            rx->filter_level = level;
            rmt_rx_accept_level(rmt, channel, level, deadline);
            changed = true;
        }
        while (rx->pending_end && rx->deadline <= now) {
            if (rx->next_word == rx->frame_count) {
                rmt_finish_rx(rmt, channel);
            } else {
                uint32_t word = rx->frame[rx->next_word++];
                if (!rmt_rx_store_word(rmt, channel, word)) {
                    changed = true;
                    continue;
                }
                if (rx->next_word == rx->frame_count && rx->pending_error)
                    rx->status_flags |= 1u << 26;
                if (rx->next_word < rx->frame_count) {
                    uint32_t item = rx->frame[rx->next_word];
                    rx->elapsed_ticks += (item & 0x7FFFu) +
                                         ((item >> 16) & 0x7FFFu);
                } else {
                    rx->elapsed_ticks += (rx->conf0 >> 8) & 0x7FFFu;
                }
                uint64_t duration = rmt_ticks_to_cycles_div(
                    rmt, rx->conf0 & 0xFFu, rx->elapsed_ticks);
                rx->deadline = rx->frame_start > UINT64_MAX - duration ?
                               UINT64_MAX : rx->frame_start + duration;
            }
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
    for (unsigned channel = 0u;
         channel < rmt_rx_channel_count(rmt->desc);
         channel++) {
        const rmt_rx_channel_t *rx = &rmt->rx[channel];
        if (rx->pending_end) {
            uint64_t d = rx->deadline > now ? rx->deadline - now : 0u;
            if (d < distance) distance = d;
        }
        if (rx->edge_started && rx->edge_deadline != UINT64_MAX) {
            uint64_t d = rx->edge_deadline > now ?
                rx->edge_deadline - now : 0u;
            if (d < distance) distance = d;
        }
        if (rx->filter_pending) {
            uint64_t d = rx->filter_deadline > now ?
                rx->filter_deadline - now : 0u;
            if (d < distance) distance = d;
        }
        if (rx->demod_pending) {
            uint64_t d = rx->demod_deadline > now ?
                rx->demod_deadline - now : 0u;
            if (d < distance) distance = d;
        }
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

static uint32_t rmt_rx_status(const flexe_rmt_v1_t *rmt, unsigned channel)
{
    unsigned physical = rmt->desc->tx_channel_count + channel;
    const rmt_rx_channel_t *rx = &rmt->rx[channel];
    uint32_t base = physical * rmt->desc->words_per_channel;
    size_t writer = rx->write_index;
    uint32_t capacity = rmt_rx_capacity(rmt, channel);
    if ((rx->conf1 & RMT_RX_WRAP) && capacity) writer %= capacity;
    return (((uint32_t)(base + writer)) & 0x3FFu) |
           (((base + rx->apb_index) & 0x3FFu) << 11) |
           (rx->active ? 1u << 22 : 0u) | rx->status_flags;
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
    if (off >= RMT_RX_CONF_OFF && off < RMT_RX_CONF_OFF +
            rmt_rx_channel_count(rmt->desc) * 8u) {
        unsigned channel = (off - RMT_RX_CONF_OFF) / 8u;
        return (off & 4u) ? rmt->rx[channel].conf1 :
                            rmt->rx[channel].conf0;
    }
    if (off >= RMT_STATUS_OFF && off < RMT_STATUS_OFF +
                                        rmt->desc->tx_channel_count * 4u)
        return rmt_status(rmt, (off - RMT_STATUS_OFF) / 4u);
    if (off >= RMT_RX_STATUS_OFF && off < RMT_RX_STATUS_OFF +
            rmt_rx_channel_count(rmt->desc) * 4u)
        return rmt_rx_status(rmt, (off - RMT_RX_STATUS_OFF) / 4u);
    if (off == RMT_INT_RAW_OFF) return rmt->int_raw;
    if (off == RMT_INT_ST_OFF) return rmt->int_raw & rmt->int_ena;
    if (off == RMT_INT_ENA_OFF) return rmt->int_ena;
    if (off == RMT_INT_CLR_OFF) return 0u;
    if (off >= RMT_CARRIER_OFF && off < RMT_CARRIER_OFF +
                                         rmt->desc->tx_channel_count * 4u)
        return rmt->tx[(off - RMT_CARRIER_OFF) / 4u].carrier;
    if (off >= RMT_RX_CARRIER_OFF && off < RMT_RX_CARRIER_OFF +
            rmt_rx_channel_count(rmt->desc) * 4u)
        return rmt->rx[(off - RMT_RX_CARRIER_OFF) / 4u].carrier;
    if (off >= RMT_TX_LIMIT_OFF && off < RMT_TX_LIMIT_OFF +
                                          rmt->desc->tx_channel_count * 4u)
        return rmt->tx[(off - RMT_TX_LIMIT_OFF) / 4u].tx_limit;
    if (off >= RMT_RX_LIMIT_OFF && off < RMT_RX_LIMIT_OFF +
            rmt_rx_channel_count(rmt->desc) * 4u)
        return rmt->rx[(off - RMT_RX_LIMIT_OFF) / 4u].limit;
    if (off == RMT_SYS_CONF_OFF) return rmt->sys_conf;
    if (off == RMT_TX_SIM_OFF) return rmt->tx_sim;
    if (off == RMT_REF_CNT_RST_OFF) return 0u; /* write-trigger pulses */
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
    if (off >= RMT_RX_CONF_OFF && off < RMT_RX_CONF_OFF +
            rmt_rx_channel_count(rmt->desc) * 8u) {
        unsigned channel = (off - RMT_RX_CONF_OFF) / 8u;
        rmt_rx_channel_t *rx = &rmt->rx[channel];
        if ((off & 4u) == 0u) {
            if (value & (RMT_RX_DMA_ACCESS | 0xC0000000u))
                rmt->fallback_write(rmt->fallback_ctx, address, value);
            rx->conf0 = value & ~(RMT_RX_DMA_ACCESS | 0xC0000000u);
        } else {
            if (value & (RMT_RX_AFIFO_RST | 0xFFFF0000u))
                rmt->fallback_write(rmt->fallback_ctx, address, value);
            rx->conf1 = (value & 0xFFFFu) & ~RMT_RX_COMMAND_MASK;
            if (value & RMT_RX_MEM_WR_RST) {
                rx->write_index = 0u;
                rx->status_flags = 0u;
                rmt_abort_rx_frame(rx);
            }
            if (value & RMT_RX_APB_MEM_RST) rx->apb_index = 0u;
            if (value & RMT_RX_CONF_UPDATE) {
                bool was_active = rx->active;
                rx->active = (rx->conf1 & RMT_RX_EN) != 0u;
                /* The envelope path models opposite-level gaps, but not
                 * same-level carrier duty recognition or silicon phase.
                 * Keep that partial mode visible in the MMIO audit. */
                if (rx->active && (rx->conf0 & RMT_RX_DEMOD_EN))
                    rmt->fallback_write(rmt->fallback_ctx, address, value);
                if (!was_active || !rx->active) rmt_abort_rx_frame(rx);
                rmt_notify_state(rmt);
            }
        }
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
    if (off >= RMT_RX_CARRIER_OFF && off < RMT_RX_CARRIER_OFF +
            rmt_rx_channel_count(rmt->desc) * 4u) {
        rmt->rx[(off - RMT_RX_CARRIER_OFF) / 4u].carrier = value;
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
    if (off >= RMT_RX_LIMIT_OFF && off < RMT_RX_LIMIT_OFF +
            rmt_rx_channel_count(rmt->desc) * 4u) {
        if (value & ~0x1FFu)
            rmt->fallback_write(rmt->fallback_ctx, address, value);
        rmt->rx[(off - RMT_RX_LIMIT_OFF) / 4u].limit = value & 0x1FFu;
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
    if (off == RMT_REF_CNT_RST_OFF) {
        /* Divider phase is reset on every arm/injection boundary. The WT
         * bits do not latch; reserved bits retain a diagnostic. */
        if (value & ~0xFFu)
            rmt->fallback_write(rmt->fallback_ctx, address, value);
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
        rmt_rx_channel_count(desc) >
            RMT_MAX_CHANNELS - RMT_MAX_TX_CHANNELS ||
        desc->channel_count > RMT_MAX_CHANNELS ||
        desc->words_per_channel != 48u ||
        desc->memory_offset != 0x800u ||
        !(target->capabilities & FLEXE_TARGET_CAP_GPIO_V1) ||
        desc->input_signal_base + rmt_rx_channel_count(desc) > 256u ||
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
    for (unsigned channel = 0u;
         channel < rmt_rx_channel_count(desc); channel++) {
        rmt->rx[channel].conf0 = 0x317FFF02u;
        rmt->rx[channel].conf1 = 0x000001E8u;
        rmt->rx[channel].limit = 128u;
    }
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
    for (unsigned channel = 0u;
         channel < rmt_rx_channel_count(rmt->desc); channel++)
        rmt_abort_rx_frame(&rmt->rx[channel]);
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

size_t flexe_rmt_v1_rx_inject(flexe_rmt_v1_t *rmt, unsigned channel,
                              const uint32_t *items, size_t count)
{
    if (!rmt || channel < rmt->desc->tx_channel_count ||
        channel >= rmt->desc->channel_count || (!items && count != 0u))
        return 0u;
    unsigned index = channel - rmt->desc->tx_channel_count;
    rmt_rx_channel_t *rx = &rmt->rx[index];
    if (!rx->active || rx->pending_end || rx->edge_started ||
        rx->filter_pending || rx->demod_pending ||
        !rmt_source_hz(rmt)) return 0u;
    if (!(rx->conf1 & RMT_RX_MEM_OWNER)) {
        rx->status_flags |= 1u << 25;
        rmt->int_raw |= RMT_RX_ERROR_INT(index);
        rmt_notify_irq(rmt);
        return 0u;
    }

    uint32_t capacity = rmt_rx_capacity(rmt, index);
    size_t available = (rx->conf1 & RMT_RX_WRAP) ? count :
        (rx->write_index < capacity ? capacity - rx->write_index : 0u);
    size_t accepted = count < available ? count : available;
    if (capacity == 0u) accepted = 0u;
    if (accepted > SIZE_MAX / sizeof(uint32_t)) return 0u;
    uint32_t *frame = accepted ? malloc(accepted * sizeof(uint32_t)) : NULL;
    if (accepted && !frame) return 0u;
    if (accepted) memcpy(frame, items, accepted * sizeof(uint32_t));
    rx->frame = frame;
    rx->frame_count = accepted;
    rx->next_word = 0u;
    rx->pending_error = accepted < count || capacity == 0u;
    /* One call supplies one input frame at its first edge. The host provides
     * symbols after filtering/demodulation; the RX_DONE edge follows the
     * pulse train and configured idle threshold in guest time. */
    uint64_t ticks = accepted ?
        (rx->frame[0] & 0x7FFFu) + ((rx->frame[0] >> 16) & 0x7FFFu) :
        (rx->conf0 >> 8) & 0x7FFFu;
    rx->elapsed_ticks = ticks;
    uint64_t duration = rmt_ticks_to_cycles_div(
        rmt, rx->conf0 & 0xFFu, ticks);
    rx->frame_start = rmt_clock_now(rmt);
    rx->deadline = rx->frame_start > UINT64_MAX - duration ?
                   UINT64_MAX : rx->frame_start + duration;
    rx->pending_end = true;
    rmt_notify_state(rmt);
    return accepted;
}

void flexe_rmt_v1_rx_input_edge(flexe_rmt_v1_t *rmt, unsigned channel,
                                 bool old_level, bool level)
{
    if (!rmt || channel < rmt->desc->tx_channel_count ||
        channel >= rmt->desc->channel_count) return;
    unsigned index = channel - rmt->desc->tx_channel_count;
    flexe_rmt_v1_eval(rmt);
    rmt_rx_channel_t *rx = &rmt->rx[index];
    if (!rx->active || rx->pending_end || !rmt_source_hz(rmt)) return;
    if (!(rx->conf1 & RMT_RX_MEM_OWNER)) {
        rx->status_flags |= 1u << 25;
        rmt->int_raw |= RMT_RX_ERROR_INT(index);
        rmt_notify_irq(rmt);
        return;
    }
    uint64_t now = rmt_clock_now(rmt);
    if (!(rx->conf1 & RMT_RX_FILTER_EN) ||
        !((rx->conf1 >> RMT_RX_FILTER_THRES_SHIFT) &
          RMT_RX_FILTER_THRES_MASK)) {
        rmt_rx_accept_level(rmt, index, level, now);
        return;
    }
    if (!rx->edge_started && !rx->filter_pending)
        rx->filter_level = old_level;
    if (level == rx->filter_level) {
        /* The raw line returned before it qualified: discard the glitch. */
        rx->filter_pending = false;
        rx->filter_deadline = UINT64_MAX;
        rmt_notify_state(rmt);
        return;
    }
    uint32_t threshold = (rx->conf1 >> RMT_RX_FILTER_THRES_SHIFT) &
                         RMT_RX_FILTER_THRES_MASK;
    /* On S3 the RX filter counts group-clock ticks, before the per-channel
     * divider. ESP-IDF uses group->resolution_hz to program this register. */
    uint64_t filter_cycles = rmt_ticks_to_cycles_div(rmt, 1u, threshold);
    rx->filter_pending = true;
    rx->filter_target = level;
    rx->filter_deadline = now > UINT64_MAX - filter_cycles ?
                          UINT64_MAX : now + filter_cycles;
    rmt_notify_state(rmt);
}
