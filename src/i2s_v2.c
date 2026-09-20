#include "i2s_v2.h"
#include "target.h"

#include <limits.h>
#include <stdlib.h>
#include <string.h>

#define I2S_V2_INT_RAW_OFF        0x00Cu
#define I2S_V2_INT_ST_OFF         0x010u
#define I2S_V2_INT_ENA_OFF        0x014u
#define I2S_V2_INT_CLR_OFF        0x018u
#define I2S_V2_RX_CONF_OFF        0x020u
#define I2S_V2_TX_CONF_OFF        0x024u
#define I2S_V2_RX_CONF1_OFF       0x028u
#define I2S_V2_TX_CONF1_OFF       0x02Cu
#define I2S_V2_RX_CLKM_CONF_OFF   0x030u
#define I2S_V2_TX_CLKM_CONF_OFF   0x034u
#define I2S_V2_RX_CLKM_DIV_OFF    0x038u
#define I2S_V2_TX_CLKM_DIV_OFF    0x03Cu
#define I2S_V2_TX_PCM2PDM_OFF     0x040u
#define I2S_V2_TX_PCM2PDM1_OFF    0x044u
#define I2S_V2_RX_TDM_CTRL_OFF    0x050u
#define I2S_V2_TX_TDM_CTRL_OFF    0x054u
#define I2S_V2_RX_TIMING_OFF      0x058u
#define I2S_V2_TX_TIMING_OFF      0x05Cu
#define I2S_V2_LC_HUNG_CONF_OFF   0x060u
#define I2S_V2_RX_EOF_NUM_OFF     0x064u
#define I2S_V2_SINGLE_DATA_OFF    0x068u
#define I2S_V2_STATE_OFF          0x06Cu
#define I2S_V2_DATE_OFF           0x080u

#define I2S_V2_INT_RX_DONE        (1u << 0)
#define I2S_V2_INT_TX_DONE        (1u << 1)
#define I2S_V2_INT_RX_HUNG        (1u << 2)
#define I2S_V2_INT_TX_HUNG        (1u << 3)
#define I2S_V2_INT_VALID_MASK     0xFu

#define I2S_V2_CONF_RESET         (1u << 0)
#define I2S_V2_CONF_FIFO_RESET    (1u << 1)
#define I2S_V2_CONF_START         (1u << 2)
#define I2S_V2_CONF_MONO          (1u << 5)
#define I2S_V2_CONF_UPDATE        (1u << 8)
#define I2S_V2_CONF_TDM_ENABLE    (1u << 19)
#define I2S_V2_CLK_ACTIVE         (1u << 26)
#define I2S_V2_CLK_SELECT_SHIFT   27u
#define I2S_V2_CLK_SELECT_MASK    3u
#define I2S_V2_CORE_CLOCK_ENABLE  (1u << 29)
#define I2S_V2_MAX_DMA_BUFFER     4095u
#define I2S_V2_RX_FIFO_SIZE       (64u * 1024u)

typedef struct {
    uint32_t regs[0x100u / sizeof(uint32_t)];
    uint8_t rx_fifo[I2S_V2_RX_FIFO_SIZE];
    size_t rx_head;
    size_t rx_length;
    uint64_t tx_deadline;
    uint64_t rx_deadline;
    bool tx_armed;
    bool rx_armed;
    bool clock_enabled;
    bool reset_asserted;
    bool irq_level;
    flexe_i2s_v2_tx_fn tx_callback;
    void *tx_ctx;
} i2s_v2_port_t;

struct flexe_i2s_v2 {
    xtensa_mem_t *mem;
    flexe_gdma_t *gdma;
    const flexe_target_desc_t *target;
    const flexe_i2s_v2_desc_t *desc;
    mmio_read_fn fallback_read;
    mmio_write_fn fallback_write;
    void *fallback_ctx;
    flexe_i2s_v2_state_fn state_changed;
    void *state_ctx;
    flexe_i2s_v2_irq_fn irq_changed;
    void *irq_ctx;
    xtensa_cpu_t *cpu[2];
    uint64_t core_cycles[2];
    uint32_t last_ccount[2];
    bool clock_valid[2];
    uint64_t cycles;
    i2s_v2_port_t port[FLEXE_TARGET_I2S_MAX];
};

static uint64_t i2s_v2_clock_now(flexe_i2s_v2_t *i2s)
{
    for (unsigned core = 0u; core < 2u; core++) {
        xtensa_cpu_t *cpu = i2s->cpu[core];
        if (!cpu) continue;
        uint32_t now = cpu->ccount;
        if (!i2s->clock_valid[core]) {
            i2s->last_ccount[core] = now;
            i2s->core_cycles[core] = i2s->cycles;
            i2s->clock_valid[core] = true;
            continue;
        }
        uint32_t elapsed = now - i2s->last_ccount[core];
        if (elapsed < (uint32_t)INT32_MAX) {
            i2s->core_cycles[core] =
                i2s->core_cycles[core] > UINT64_MAX - elapsed ?
                UINT64_MAX : i2s->core_cycles[core] + elapsed;
            if (i2s->core_cycles[core] > i2s->cycles)
                i2s->cycles = i2s->core_cycles[core];
        }
        i2s->last_ccount[core] = now;
    }
    return i2s->cycles;
}

static void i2s_v2_notify_state(flexe_i2s_v2_t *i2s)
{
    if (i2s->state_changed) i2s->state_changed(i2s->state_ctx);
}

static void i2s_v2_update_irq(flexe_i2s_v2_t *i2s, unsigned port)
{
    i2s_v2_port_t *state = &i2s->port[port];
    bool level = (state->regs[I2S_V2_INT_RAW_OFF / 4u] &
                  state->regs[I2S_V2_INT_ENA_OFF / 4u] &
                  I2S_V2_INT_VALID_MASK) != 0u;
    if (level == state->irq_level) return;
    state->irq_level = level;
    if (i2s->irq_changed)
        i2s->irq_changed(i2s->irq_ctx, port, level);
}

static void i2s_v2_reset_port(flexe_i2s_v2_t *i2s, unsigned port)
{
    i2s_v2_port_t *state = &i2s->port[port];
    flexe_i2s_v2_tx_fn callback = state->tx_callback;
    void *callback_ctx = state->tx_ctx;
    bool clock_enabled = state->clock_enabled;
    bool reset_asserted = state->reset_asserted;
    bool old_irq = state->irq_level;
    memset(state, 0, sizeof(*state));
    state->tx_callback = callback;
    state->tx_ctx = callback_ctx;
    state->clock_enabled = clock_enabled;
    state->reset_asserted = reset_asserted;
    state->regs[I2S_V2_RX_CONF_OFF / 4u] =
        (1u << 15) | (1u << 12) | (1u << 10) | (1u << 9);
    state->regs[I2S_V2_TX_CONF_OFF / 4u] =
        (1u << 15) | (1u << 13) | (1u << 12) | (1u << 9);
    state->regs[I2S_V2_RX_CONF1_OFF / 4u] =
        (1u << 29) | (15u << 24) | (15u << 18) |
        (15u << 13) | (6u << 7);
    state->regs[I2S_V2_TX_CONF1_OFF / 4u] =
        (1u << 30) | (1u << 29) | (15u << 24) | (15u << 18) |
        (15u << 13) | (6u << 7);
    state->regs[I2S_V2_RX_CLKM_CONF_OFF / 4u] = 2u;
    state->regs[I2S_V2_TX_CLKM_CONF_OFF / 4u] = 2u;
    state->regs[I2S_V2_RX_CLKM_DIV_OFF / 4u] = 1u << 9;
    state->regs[I2S_V2_TX_CLKM_DIV_OFF / 4u] = 1u << 9;
    state->regs[I2S_V2_TX_PCM2PDM_OFF / 4u] =
        (1u << 22) | (1u << 19) | (1u << 17) |
        (1u << 15) | (1u << 13) | (2u << 1);
    state->regs[I2S_V2_TX_PCM2PDM1_OFF / 4u] =
        (7u << 23) | (7u << 20) | (480u << 10) | 960u;
    state->regs[I2S_V2_RX_TDM_CTRL_OFF / 4u] = 0xFFFFu;
    state->regs[I2S_V2_TX_TDM_CTRL_OFF / 4u] = 0xFFFFu;
    state->regs[I2S_V2_LC_HUNG_CONF_OFF / 4u] = (1u << 11) | 0x10u;
    state->regs[I2S_V2_RX_EOF_NUM_OFF / 4u] = 0x40u;
    state->regs[I2S_V2_STATE_OFF / 4u] = 1u;
    state->regs[I2S_V2_DATE_OFF / 4u] = i2s->desc->date_reset;
    if (old_irq && i2s->irq_changed)
        i2s->irq_changed(i2s->irq_ctx, port, false);
}

static uint32_t i2s_v2_fraction_denominator(uint32_t value,
                                            uint32_t *numerator)
{
    uint32_t z = value & 0x1FFu;
    uint32_t y = (value >> 9u) & 0x1FFu;
    uint32_t x = (value >> 18u) & 0x1FFu;
    bool yn1 = (value & (1u << 27u)) != 0u;
    if (!z) {
        *numerator = 0u;
        return 1u;
    }
    uint32_t denominator = (x + 1u) * z + y;
    if (!denominator) {
        *numerator = 0u;
        return 1u;
    }
    *numerator = yn1 ? denominator - z : z;
    return denominator;
}

static uint32_t i2s_v2_sample_rate(const flexe_i2s_v2_t *i2s,
                                   unsigned port, bool receive)
{
    const i2s_v2_port_t *state = &i2s->port[port];
    uint32_t clkm = state->regs[(receive ? I2S_V2_RX_CLKM_CONF_OFF :
                                I2S_V2_TX_CLKM_CONF_OFF) / 4u];
    uint32_t source = i2s->desc->source_clock_hz[
        (clkm >> I2S_V2_CLK_SELECT_SHIFT) & I2S_V2_CLK_SELECT_MASK];
    uint32_t integer = clkm & 0xFFu;
    uint32_t fractional = state->regs[(receive ? I2S_V2_RX_CLKM_DIV_OFF :
                                      I2S_V2_TX_CLKM_DIV_OFF) / 4u];
    uint32_t numerator = 0u;
    uint32_t denominator =
        i2s_v2_fraction_denominator(fractional, &numerator);
    uint64_t clock_divisor = (uint64_t)integer * denominator + numerator;
    uint32_t conf1 = state->regs[(receive ? I2S_V2_RX_CONF1_OFF :
                                    I2S_V2_TX_CONF1_OFF) / 4u];
    uint32_t bck_divisor = ((conf1 >> 7u) & 0x3Fu) + 1u;
    uint32_t channel_bits = ((conf1 >> 24u) & 0x1Fu) + 1u;
    uint32_t tdm = state->regs[(receive ? I2S_V2_RX_TDM_CTRL_OFF :
                                  I2S_V2_TX_TDM_CTRL_OFF) / 4u];
    uint32_t slots = ((tdm >> 16u) & 0xFu) + 1u;
    uint64_t divisor = clock_divisor * bck_divisor * channel_bits * slots;
    if (!source || !clock_divisor || !divisor) return 0u;
    uint64_t rate = (uint64_t)source * denominator / divisor;
    return rate > UINT32_MAX ? UINT32_MAX : (uint32_t)rate;
}

static uint8_t i2s_v2_bits_per_sample(const i2s_v2_port_t *state,
                                      bool receive)
{
    uint32_t conf1 = state->regs[(receive ? I2S_V2_RX_CONF1_OFF :
                                    I2S_V2_TX_CONF1_OFF) / 4u];
    return (uint8_t)(((conf1 >> 13u) & 0x1Fu) + 1u);
}

static uint8_t i2s_v2_channel_count(const i2s_v2_port_t *state,
                                    bool receive)
{
    uint32_t conf = state->regs[(receive ? I2S_V2_RX_CONF_OFF :
                                   I2S_V2_TX_CONF_OFF) / 4u];
    if (conf & I2S_V2_CONF_MONO) return 1u;
    uint32_t tdm = state->regs[(receive ? I2S_V2_RX_TDM_CTRL_OFF :
                                  I2S_V2_TX_TDM_CTRL_OFF) / 4u];
    uint32_t slots = ((tdm >> 16u) & 0xFu) + 1u;
    uint32_t enabled = tdm & (slots == 16u ? 0xFFFFu : (1u << slots) - 1u);
    unsigned count = 0u;
    while (enabled) {
        count += enabled & 1u;
        enabled >>= 1u;
    }
    return (uint8_t)(count ? count : slots);
}

static uint64_t i2s_v2_descriptor_cycles(const flexe_i2s_v2_t *i2s,
                                         unsigned port, bool receive,
                                         size_t length)
{
    const i2s_v2_port_t *state = &i2s->port[port];
    uint32_t rate = i2s_v2_sample_rate(i2s, port, receive);
    if (!rate) rate = 48000u;
    uint8_t bits = i2s_v2_bits_per_sample(state, receive);
    uint8_t channels = i2s_v2_channel_count(state, receive);
    uint32_t bytes_per_sample = ((uint32_t)bits + 15u) / 16u * 2u;
    uint64_t bytes_per_second =
        (uint64_t)rate * bytes_per_sample * (channels ? channels : 1u);
    uint32_t cpu_mhz = i2s->cpu[0] ?
        xtensa_cpu_freq_mhz(i2s->cpu[0]) :
        i2s->target->default_cpu_frequency_mhz;
    uint64_t cpu_hz = (uint64_t)cpu_mhz * 1000000u;
    if (!bytes_per_second || !cpu_hz) return 1u;
    uint64_t cycles = ((uint64_t)length * cpu_hz +
                       bytes_per_second - 1u) / bytes_per_second;
    return cycles ? cycles : 1u;
}

static bool i2s_v2_direction_enabled(const flexe_i2s_v2_t *i2s,
                                     unsigned port, bool receive)
{
    const i2s_v2_port_t *state = &i2s->port[port];
    uint32_t conf = state->regs[(receive ? I2S_V2_RX_CONF_OFF :
                                   I2S_V2_TX_CONF_OFF) / 4u];
    uint32_t clkm = state->regs[(receive ? I2S_V2_RX_CLKM_CONF_OFF :
                                   I2S_V2_TX_CLKM_CONF_OFF) / 4u];
    if (!state->clock_enabled || state->reset_asserted ||
        !(conf & I2S_V2_CONF_START) || !(clkm & I2S_V2_CLK_ACTIVE))
        return false;
    if (!receive && !(state->regs[I2S_V2_TX_CLKM_CONF_OFF / 4u] &
                      I2S_V2_CORE_CLOCK_ENABLE))
        return false;
    return true;
}

static bool i2s_v2_arm_direction(flexe_i2s_v2_t *i2s, unsigned port,
                                 bool receive, uint64_t now)
{
    i2s_v2_port_t *state = &i2s->port[port];
    bool *armed = receive ? &state->rx_armed : &state->tx_armed;
    if (!i2s_v2_direction_enabled(i2s, port, receive)) {
        bool changed = *armed;
        *armed = false;
        return changed;
    }
    size_t length = 0u;
    uint8_t trigger = i2s->desc->instance[port].gdma_peripheral_id;
    if (!flexe_gdma_pending_length(i2s->gdma, trigger, receive, &length)) {
        bool changed = *armed;
        *armed = false;
        return changed;
    }
    /* Host-fed receive samples are an asynchronous peripheral input. Keep the
     * descriptor owned by hardware, and therefore a blocking guest reader
     * asleep, until one complete DMA buffer is available. Silently padding an
     * empty host stream with zeroes makes a disconnected source look like a
     * successful capture and races input injected after the channel starts. */
    if (receive && state->rx_length < length) {
        bool changed = *armed;
        *armed = false;
        return changed;
    }
    if (*armed) return false;
    uint64_t duration = i2s_v2_descriptor_cycles(
        i2s, port, receive, length);
    uint64_t deadline = now > UINT64_MAX - duration ?
                        UINT64_MAX : now + duration;
    if (receive) state->rx_deadline = deadline;
    else state->tx_deadline = deadline;
    *armed = true;
    return true;
}

static void i2s_v2_refresh(flexe_i2s_v2_t *i2s)
{
    if (!i2s) return;
    uint64_t now = i2s_v2_clock_now(i2s);
    bool changed = false;
    for (unsigned port = 0u; port < i2s->desc->instance_count; port++) {
        changed |= i2s_v2_arm_direction(i2s, port, false, now);
        changed |= i2s_v2_arm_direction(i2s, port, true, now);
    }
    if (changed) i2s_v2_notify_state(i2s);
}

static void i2s_v2_gdma_changed(void *ctx)
{
    i2s_v2_refresh(ctx);
}

static uint8_t i2s_v2_rx_pop(i2s_v2_port_t *state)
{
    if (!state->rx_length) return 0u;
    uint8_t value = state->rx_fifo[state->rx_head];
    state->rx_head = (state->rx_head + 1u) % I2S_V2_RX_FIFO_SIZE;
    state->rx_length--;
    return value;
}

static void i2s_v2_service_tx(flexe_i2s_v2_t *i2s, unsigned port)
{
    i2s_v2_port_t *state = &i2s->port[port];
    uint8_t data[I2S_V2_MAX_DMA_BUFFER];
    flexe_gdma_descriptor_t completed;
    uint8_t trigger = i2s->desc->instance[port].gdma_peripheral_id;
    if (flexe_gdma_read_tx_descriptor(i2s->gdma, trigger, data,
                                      sizeof(data), &completed) != 0) {
        state->regs[I2S_V2_INT_RAW_OFF / 4u] |= I2S_V2_INT_TX_HUNG;
        i2s_v2_update_irq(i2s, port);
        return;
    }
    if (state->tx_callback)
        state->tx_callback(state->tx_ctx, (int)port, data,
                           completed.length,
                           i2s_v2_sample_rate(i2s, port, false),
                           i2s_v2_bits_per_sample(state, false),
                           i2s_v2_channel_count(state, false));
    if (completed.chain_complete) {
        state->regs[I2S_V2_INT_RAW_OFF / 4u] |= I2S_V2_INT_TX_DONE;
        i2s_v2_update_irq(i2s, port);
    }
}

static void i2s_v2_service_rx(flexe_i2s_v2_t *i2s, unsigned port)
{
    i2s_v2_port_t *state = &i2s->port[port];
    size_t length = 0u;
    uint8_t trigger = i2s->desc->instance[port].gdma_peripheral_id;
    if (!flexe_gdma_pending_length(i2s->gdma, trigger, true, &length) ||
        length > I2S_V2_MAX_DMA_BUFFER) {
        state->regs[I2S_V2_INT_RAW_OFF / 4u] |= I2S_V2_INT_RX_HUNG;
        i2s_v2_update_irq(i2s, port);
        return;
    }
    uint8_t data[I2S_V2_MAX_DMA_BUFFER];
    for (size_t index = 0u; index < length; index++)
        data[index] = i2s_v2_rx_pop(state);
    flexe_gdma_descriptor_t completed;
    if (flexe_gdma_write_rx_descriptor(i2s->gdma, trigger, data, length,
                                       &completed) != 0) {
        state->regs[I2S_V2_INT_RAW_OFF / 4u] |= I2S_V2_INT_RX_HUNG;
        i2s_v2_update_irq(i2s, port);
        return;
    }
    if (completed.chain_complete) {
        state->regs[I2S_V2_INT_RAW_OFF / 4u] |= I2S_V2_INT_RX_DONE;
        i2s_v2_update_irq(i2s, port);
    }
}

void flexe_i2s_v2_eval(flexe_i2s_v2_t *i2s)
{
    if (!i2s) return;
    uint64_t now = i2s_v2_clock_now(i2s);
    bool serviced = false;
    for (unsigned port = 0u; port < i2s->desc->instance_count; port++) {
        i2s_v2_port_t *state = &i2s->port[port];
        if (state->tx_armed && state->tx_deadline <= now) {
            state->tx_armed = false;
            i2s_v2_service_tx(i2s, port);
            serviced = true;
        }
        if (state->rx_armed && state->rx_deadline <= now) {
            state->rx_armed = false;
            i2s_v2_service_rx(i2s, port);
            serviced = true;
        }
    }
    i2s_v2_refresh(i2s);
    if (serviced) i2s_v2_notify_state(i2s);
}

uint32_t flexe_i2s_v2_next_event(flexe_i2s_v2_t *i2s,
                                 xtensa_cpu_t *cpu)
{
    if (!i2s || !cpu) return UINT32_MAX;
    uint64_t now = i2s_v2_clock_now(i2s);
    uint64_t distance = UINT64_MAX;
    for (unsigned port = 0u; port < i2s->desc->instance_count; port++) {
        const i2s_v2_port_t *state = &i2s->port[port];
        if (state->tx_armed) {
            uint64_t candidate = state->tx_deadline > now ?
                state->tx_deadline - now : 0u;
            if (candidate < distance) distance = candidate;
        }
        if (state->rx_armed) {
            uint64_t candidate = state->rx_deadline > now ?
                state->rx_deadline - now : 0u;
            if (candidate < distance) distance = candidate;
        }
    }
    if (distance == UINT64_MAX) return UINT32_MAX;
    if (distance > (uint32_t)INT32_MAX) distance = INT32_MAX;
    return cpu->ccount + (uint32_t)distance;
}

static int i2s_v2_port_from_address(const flexe_i2s_v2_t *i2s,
                                    uint32_t address)
{
    for (unsigned port = 0u; port < i2s->desc->instance_count; port++) {
        uint32_t base = i2s->desc->instance[port].base;
        if (address >= base && address - base < i2s->desc->register_size)
            return (int)port;
    }
    return -1;
}

static uint32_t i2s_v2_read(void *ctx, uint32_t address)
{
    flexe_i2s_v2_t *i2s = ctx;
    int decoded = i2s_v2_port_from_address(i2s, address);
    if (decoded < 0)
        return i2s->fallback_read(i2s->fallback_ctx, address);
    unsigned port = (unsigned)decoded;
    uint32_t off = address - i2s->desc->instance[port].base;
    if ((off & 3u) != 0u || off > I2S_V2_DATE_OFF)
        return i2s->fallback_read(i2s->fallback_ctx, address);
    flexe_i2s_v2_eval(i2s);
    i2s_v2_port_t *state = &i2s->port[port];
    if (off == I2S_V2_INT_ST_OFF)
        return state->regs[I2S_V2_INT_RAW_OFF / 4u] &
               state->regs[I2S_V2_INT_ENA_OFF / 4u] &
               I2S_V2_INT_VALID_MASK;
    if (off == I2S_V2_INT_CLR_OFF) return 0u;
    if (off == I2S_V2_STATE_OFF)
        return state->tx_armed ? 0u : 1u;
    return state->regs[off / 4u];
}

static void i2s_v2_write(void *ctx, uint32_t address, uint32_t value)
{
    flexe_i2s_v2_t *i2s = ctx;
    int decoded = i2s_v2_port_from_address(i2s, address);
    if (decoded < 0) {
        i2s->fallback_write(i2s->fallback_ctx, address, value);
        return;
    }
    unsigned port = (unsigned)decoded;
    uint32_t off = address - i2s->desc->instance[port].base;
    if ((off & 3u) != 0u || off > I2S_V2_DATE_OFF) {
        i2s->fallback_write(i2s->fallback_ctx, address, value);
        return;
    }
    flexe_i2s_v2_eval(i2s);
    i2s_v2_port_t *state = &i2s->port[port];
    switch (off) {
    case I2S_V2_INT_RAW_OFF:
    case I2S_V2_INT_ST_OFF:
    case I2S_V2_STATE_OFF:
        return;
    case I2S_V2_INT_ENA_OFF:
        state->regs[off / 4u] = value & I2S_V2_INT_VALID_MASK;
        i2s_v2_update_irq(i2s, port);
        return;
    case I2S_V2_INT_CLR_OFF:
        state->regs[I2S_V2_INT_RAW_OFF / 4u] &=
            ~(value & I2S_V2_INT_VALID_MASK);
        i2s_v2_update_irq(i2s, port);
        return;
    case I2S_V2_RX_CONF_OFF:
    case I2S_V2_TX_CONF_OFF: {
        bool receive = off == I2S_V2_RX_CONF_OFF;
        uint32_t stored = value & ~(I2S_V2_CONF_RESET |
                                    I2S_V2_CONF_FIFO_RESET |
                                    I2S_V2_CONF_UPDATE);
        state->regs[off / 4u] = stored;
        if (value & (I2S_V2_CONF_RESET | I2S_V2_CONF_FIFO_RESET)) {
            if (receive) {
                state->rx_armed = false;
                state->rx_head = 0u;
                state->rx_length = 0u;
            } else {
                state->tx_armed = false;
            }
        }
        i2s_v2_refresh(i2s);
        return;
    }
    case I2S_V2_DATE_OFF:
        state->regs[off / 4u] = value & 0x0FFFFFFFu;
        return;
    default:
        state->regs[off / 4u] = value;
        i2s_v2_refresh(i2s);
        return;
    }
}

static bool i2s_v2_geometry_valid(const flexe_target_desc_t *target,
                                  const flexe_gdma_t *gdma)
{
    if (!target || !gdma ||
        !(target->capabilities & FLEXE_TARGET_CAP_I2S_V2) ||
        !(target->capabilities & FLEXE_TARGET_CAP_GDMA_V1) ||
        !(target->capabilities & FLEXE_TARGET_CAP_INTERRUPT_MATRIX_V1) ||
        !(target->capabilities & FLEXE_TARGET_CAP_SYSTEM_CLOCK_V1))
        return false;
    const flexe_i2s_v2_desc_t *desc = &target->i2s_v2;
    if (!desc->instance_count ||
        desc->instance_count > FLEXE_TARGET_I2S_MAX ||
        desc->register_size < I2S_V2_DATE_OFF + sizeof(uint32_t) ||
        (desc->register_size & 3u) != 0u || !desc->date_reset)
        return false;
    for (unsigned port = 0u; port < desc->instance_count; port++) {
        const flexe_i2s_v2_instance_desc_t *instance =
            &desc->instance[port];
        if ((instance->base & 0xFFFu) != 0u ||
            instance->base < target->peripheral_start ||
            instance->base >= target->peripheral_end ||
            desc->register_size > target->peripheral_end - instance->base ||
            instance->interrupt_source >=
                target->interrupt_matrix.source_count ||
            instance->gdma_peripheral_id ==
                FLEXE_TARGET_GDMA_PERIPHERAL_NONE ||
            instance->data_output_count >
                FLEXE_TARGET_I2S_DATA_OUT_MAX)
            return false;
        for (unsigned prior = 0u; prior < port; prior++) {
            const flexe_i2s_v2_instance_desc_t *other =
                &desc->instance[prior];
            if (instance->base == other->base ||
                instance->interrupt_source == other->interrupt_source ||
                instance->gdma_peripheral_id == other->gdma_peripheral_id)
                return false;
        }
    }
    return true;
}

flexe_i2s_v2_t *flexe_i2s_v2_create(
    xtensa_mem_t *mem, flexe_gdma_t *gdma,
    mmio_read_fn fallback_read, mmio_write_fn fallback_write,
    void *fallback_ctx, flexe_i2s_v2_state_fn state_changed,
    void *state_ctx, flexe_i2s_v2_irq_fn irq_changed, void *irq_ctx)
{
    const flexe_target_desc_t *target = mem ? mem_target(mem) : NULL;
    if (!mem || !fallback_read || !fallback_write ||
        !i2s_v2_geometry_valid(target, gdma))
        return NULL;
    flexe_i2s_v2_t *i2s = calloc(1u, sizeof(*i2s));
    if (!i2s) return NULL;
    i2s->mem = mem;
    i2s->gdma = gdma;
    i2s->target = target;
    i2s->desc = &target->i2s_v2;
    i2s->fallback_read = fallback_read;
    i2s->fallback_write = fallback_write;
    i2s->fallback_ctx = fallback_ctx;
    i2s->state_changed = state_changed;
    i2s->state_ctx = state_ctx;
    i2s->irq_changed = irq_changed;
    i2s->irq_ctx = irq_ctx;
    for (unsigned port = 0u; port < i2s->desc->instance_count; port++) {
        i2s->port[port].clock_enabled = false;
        i2s->port[port].reset_asserted = false;
        i2s_v2_reset_port(i2s, port);
        if (mem_register_mmio_range(
                mem, i2s->desc->instance[port].base,
                i2s->desc->register_size,
                i2s_v2_read, i2s_v2_write, i2s) != 0) {
            flexe_i2s_v2_destroy(i2s);
            return NULL;
        }
    }
    flexe_gdma_set_activity_handler(gdma, i2s_v2_gdma_changed, i2s);
    return i2s;
}

void flexe_i2s_v2_destroy(flexe_i2s_v2_t *i2s)
{
    if (!i2s) return;
    flexe_gdma_set_activity_handler(i2s->gdma, NULL, NULL);
    for (unsigned port = 0u; port < i2s->desc->instance_count; port++) {
        if (i2s->port[port].irq_level && i2s->irq_changed)
            i2s->irq_changed(i2s->irq_ctx, port, false);
        (void)mem_register_mmio_range(
            i2s->mem, i2s->desc->instance[port].base,
            i2s->desc->register_size,
            i2s->fallback_read, i2s->fallback_write,
            i2s->fallback_ctx);
    }
    free(i2s);
}

void flexe_i2s_v2_attach_cpus(flexe_i2s_v2_t *i2s,
                              xtensa_cpu_t *cpu0, xtensa_cpu_t *cpu1)
{
    if (!i2s) return;
    i2s->cpu[0] = cpu0;
    i2s->cpu[1] = cpu1;
    i2s->clock_valid[0] = false;
    i2s->clock_valid[1] = false;
    (void)i2s_v2_clock_now(i2s);
    i2s_v2_refresh(i2s);
}

void flexe_i2s_v2_set_system_state(flexe_i2s_v2_t *i2s, unsigned port,
                                   bool clock_enabled,
                                   bool reset_asserted)
{
    if (!i2s || port >= i2s->desc->instance_count) return;
    i2s_v2_port_t *state = &i2s->port[port];
    bool reset_edge = reset_asserted && !state->reset_asserted;
    state->clock_enabled = clock_enabled;
    state->reset_asserted = reset_asserted;
    if (reset_edge) i2s_v2_reset_port(i2s, port);
    i2s_v2_refresh(i2s);
}

int flexe_i2s_v2_set_tx_callback(flexe_i2s_v2_t *i2s, unsigned port,
                                 flexe_i2s_v2_tx_fn callback, void *ctx)
{
    if (!i2s || port >= i2s->desc->instance_count) return -1;
    i2s->port[port].tx_callback = callback;
    i2s->port[port].tx_ctx = callback ? ctx : NULL;
    return 0;
}

size_t flexe_i2s_v2_rx_inject(flexe_i2s_v2_t *i2s, unsigned port,
                              const uint8_t *data, size_t length)
{
    if (!i2s || port >= i2s->desc->instance_count ||
        (!data && length != 0u))
        return 0u;
    i2s_v2_port_t *state = &i2s->port[port];
    size_t accepted = length;
    if (accepted > I2S_V2_RX_FIFO_SIZE - state->rx_length)
        accepted = I2S_V2_RX_FIFO_SIZE - state->rx_length;
    for (size_t index = 0u; index < accepted; index++) {
        size_t tail = (state->rx_head + state->rx_length) %
                      I2S_V2_RX_FIFO_SIZE;
        state->rx_fifo[tail] = data[index];
        state->rx_length++;
    }
    if (accepted) i2s_v2_refresh(i2s);
    return accepted;
}

size_t flexe_i2s_v2_rx_pending(const flexe_i2s_v2_t *i2s, unsigned port)
{
    return i2s && port < i2s->desc->instance_count ?
           i2s->port[port].rx_length : 0u;
}
