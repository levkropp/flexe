#include "gpio.h"
#include "target.h"

#include <limits.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

/* S2/S3 GPIO register layout. Addresses never appear in firmware hooks: the
 * target descriptor selects this IP generation and supplies its base. */
#define GPIO_BT_SELECT_OFF          0x000u
#define GPIO_OUT_OFF                0x004u
#define GPIO_OUT_SET_OFF            0x008u
#define GPIO_OUT_CLEAR_OFF          0x00Cu
#define GPIO_OUT1_OFF               0x010u
#define GPIO_OUT1_SET_OFF           0x014u
#define GPIO_OUT1_CLEAR_OFF         0x018u
#define GPIO_SDIO_SELECT_OFF        0x01Cu
#define GPIO_ENABLE_OFF             0x020u
#define GPIO_ENABLE_SET_OFF         0x024u
#define GPIO_ENABLE_CLEAR_OFF       0x028u
#define GPIO_ENABLE1_OFF            0x02Cu
#define GPIO_ENABLE1_SET_OFF        0x030u
#define GPIO_ENABLE1_CLEAR_OFF      0x034u
#define GPIO_STRAP_OFF              0x038u
#define GPIO_IN_OFF                 0x03Cu
#define GPIO_IN1_OFF                0x040u
#define GPIO_STATUS_OFF             0x044u
#define GPIO_STATUS_SET_OFF         0x048u
#define GPIO_STATUS_CLEAR_OFF       0x04Cu
#define GPIO_STATUS1_OFF            0x050u
#define GPIO_STATUS1_SET_OFF        0x054u
#define GPIO_STATUS1_CLEAR_OFF      0x058u
#define GPIO_CPU_INT_OFF            0x05Cu
#define GPIO_CPU_NMI_INT_OFF        0x060u
#define GPIO_CPUSDIO_INT_OFF        0x064u
#define GPIO_CPU_INT1_OFF           0x068u
#define GPIO_CPU_NMI_INT1_OFF       0x06Cu
#define GPIO_CPUSDIO_INT1_OFF       0x070u
#define GPIO_PIN_BASE_OFF           0x074u
#define GPIO_PIN_REGISTER_COUNT     54u
#define GPIO_STATUS_NEXT_OFF        0x14Cu
#define GPIO_STATUS_NEXT1_OFF       0x150u
#define GPIO_FUNC_IN_BASE_OFF       0x154u
#define GPIO_FUNC_IN_COUNT          256u
#define GPIO_FUNC_IN_WORDS          (GPIO_FUNC_IN_COUNT / 64u)
#define GPIO_FUNC_OUT_BASE_OFF      0x554u
#define GPIO_CLOCK_GATE_OFF         0x62Cu
#define GPIO_DATE_OFF               0x6FCu
#define GPIO_REQUIRED_SIZE          0x700u

#define GPIO_HIGH_REGISTER_MASK     0x003FFFFFu
#define GPIO_SDIO_SELECT_MASK       0x000000FFu
#define GPIO_STRAP_MASK             0x0000FFFFu
#define GPIO_PIN_WRITABLE_MASK      0x0003FF9Fu
#define GPIO_PIN_INT_TYPE_MASK      (7u << 7u)
#define GPIO_PIN_INT_TYPE_SHIFT     7u
#define GPIO_PIN_INT_ENABLE_MASK    (0x1Fu << 13u)
#define GPIO_PIN_OPEN_DRAIN         (1u << 2u)
#define GPIO_PIN_NORMAL_ENABLE      (1u << 13u)
#define GPIO_PIN_NMI_ENABLE         (1u << 14u)
#define GPIO_PIN_SDIO_ENABLE        (1u << 17u)
#define GPIO_PIN_MODELED_MASK       \
    (GPIO_PIN_INT_TYPE_MASK | GPIO_PIN_NORMAL_ENABLE | GPIO_PIN_NMI_ENABLE | \
     GPIO_PIN_OPEN_DRAIN)
#define GPIO_FUNC_IN_MASK           0x000000FFu
#define GPIO_FUNC_IN_INVERT         (1u << 6u)
#define GPIO_FUNC_IN_MATRIX         (1u << 7u)
#define GPIO_FUNC_OUT_MASK          0x00000FFFu
#define GPIO_FUNC_OUT_SIGNAL_MASK   0x000001FFu
#define GPIO_FUNC_OUT_SIGNAL_COUNT  FLEXE_TARGET_GPIO_MATRIX_OUTPUT_COUNT
#define GPIO_FUNC_OUT_SIGNAL_WORDS  (GPIO_FUNC_OUT_SIGNAL_COUNT / 64u)
#define GPIO_FUNC_OUT_INVERT        (1u << 9u)
#define GPIO_FUNC_OUT_OEN_SELECT    (1u << 10u)
#define GPIO_FUNC_OUT_OEN_INVERT    (1u << 11u)
#define GPIO_FUNC_OUT_SOFTWARE      FLEXE_TARGET_GPIO_MATRIX_SOFTWARE_OUTPUT
#define GPIO_CLOCK_GATE_ENABLE      1u
#define GPIO_DATE_MASK              0x0FFFFFFFu

struct flexe_gpio {
    xtensa_mem_t *mem;
    const flexe_target_desc_t *target;
    mmio_read_fn fallback_read;
    mmio_write_fn fallback_write;
    void *fallback_ctx;
    flexe_gpio_output_fn output_changed;
    void *output_ctx;
    flexe_gpio_irq_fn irq_changed;
    void *irq_ctx;
    flexe_gpio_input_signal_fn input_signal_changed;
    void *input_signal_ctx;
    flexe_gpio_input_route_fn input_route_changed;
    void *input_route_ctx;
    flexe_gpio_output_sample_fn output_sample;
    void *output_sample_ctx;

    uint32_t bt_select;
    uint32_t sdio_select;
    uint32_t out[2];
    uint32_t enable[2];
    uint32_t input[2];
    uint32_t peripheral_unknown[2];
    bool unknown_input_read_reported[2];
    uint32_t input_enable[2];
    uint32_t host_input[2];
    uint32_t host_valid[2];
    uint32_t status[2];
    uint32_t pin[GPIO_PIN_REGISTER_COUNT];
    uint32_t func_in[GPIO_FUNC_IN_COUNT];
    uint64_t watched_input_signal[GPIO_FUNC_IN_WORDS];
    uint32_t func_out[GPIO_PIN_REGISTER_COUNT];
    uint64_t modeled_output_signal[GPIO_FUNC_OUT_SIGNAL_WORDS];
    uint64_t sampled_output_signal[GPIO_FUNC_OUT_SIGNAL_WORDS];
    uint64_t driven_output_signal[GPIO_FUNC_OUT_SIGNAL_WORDS];
    uint64_t output_signal_level[GPIO_FUNC_OUT_SIGNAL_WORDS];
    uint64_t output_signal_enable[GPIO_FUNC_OUT_SIGNAL_WORDS];
    uint32_t clock_gate;
    uint32_t date;
    bool irq_level[2];
    uint64_t held_pins;
    uint64_t rtc_owned;
    uint64_t rtc_unknown;
    uint64_t rtc_output;
    uint64_t rtc_enabled;
    int8_t held_level[GPIO_PIN_REGISTER_COUNT];
    int8_t held_enable[GPIO_PIN_REGISTER_COUNT];
};

static uint64_t gpio_count_mask(unsigned count)
{
    if (count == 0u) return 0u;
    if (count >= 64u) return UINT64_MAX;
    return (UINT64_C(1) << count) - 1u;
}

static bool gpio_geometry_valid(const flexe_target_desc_t *target)
{
    if (!target || !(target->capabilities & FLEXE_TARGET_CAP_GPIO_V1))
        return false;
    const flexe_gpio_desc_t *desc = &target->gpio;
    if ((desc->base & 0xFFFu) != 0u ||
        (desc->register_size & 0xFFFu) != 0u ||
        desc->register_size < GPIO_REQUIRED_SIZE ||
        desc->base < target->peripheral_start ||
        desc->base >= target->peripheral_end ||
        desc->register_size > target->peripheral_end - desc->base ||
        desc->gpio_count == 0u ||
        desc->gpio_count > FLEXE_TARGET_GPIO_MAX ||
        desc->valid_gpio_mask == 0u ||
        (desc->valid_gpio_mask & ~gpio_count_mask(desc->gpio_count)) != 0u ||
        desc->matrix_const_one_input >= 64u ||
        desc->matrix_const_zero_input >= 64u ||
        desc->matrix_const_one_input == desc->matrix_const_zero_input ||
        desc->interrupt_source == desc->nmi_interrupt_source ||
        desc->interrupt_source >= FLEXE_TARGET_INTERRUPT_SOURCE_MAX ||
        desc->nmi_interrupt_source >= FLEXE_TARGET_INTERRUPT_SOURCE_MAX)
        return false;

    if (target->capabilities & FLEXE_TARGET_CAP_INTERRUPT_MATRIX_V1) {
        if (desc->interrupt_source >= target->interrupt_matrix.source_count ||
            desc->nmi_interrupt_source >=
                target->interrupt_matrix.source_count)
            return false;
    }
    return true;
}

static uint32_t gpio_bank_mask(unsigned bank)
{
    return bank == 0u ? UINT32_MAX : GPIO_HIGH_REGISTER_MASK;
}

static uint32_t gpio_valid_bank_mask(const flexe_gpio_t *gpio, unsigned bank)
{
    uint64_t valid = gpio->target->gpio.valid_gpio_mask;
    return (uint32_t)(valid >> (bank * 32u)) & gpio_bank_mask(bank);
}

static bool gpio_pin_valid(const flexe_gpio_t *gpio, unsigned pin)
{
    return gpio && pin < gpio->target->gpio.gpio_count &&
           (gpio->target->gpio.valid_gpio_mask &
            (UINT64_C(1) << pin)) != 0u;
}

static bool gpio_pin_bit(unsigned pin, unsigned *bank, uint32_t *mask)
{
    if (pin >= 64u) return false;
    *bank = pin / 32u;
    *mask = 1u << (pin % 32u);
    return *bank < 2u;
}

int flexe_gpio_out_signal(const flexe_gpio_t *gpio, unsigned pin)
{
    if (!gpio_pin_valid(gpio, pin)) return -1;
    return (int)(gpio->func_out[pin] & GPIO_FUNC_OUT_SIGNAL_MASK);
}

int flexe_gpio_out_route(const flexe_gpio_t *gpio, unsigned pin)
{
    if (!gpio_pin_valid(gpio, pin)) return -1;
    return (int)gpio->func_out[pin];
}

static void gpio_notify_pin(flexe_gpio_t *gpio, unsigned pin);

void flexe_gpio_set_output_signal_modeled(flexe_gpio_t *gpio,
                                          unsigned signal)
{
    if (!gpio || signal >= GPIO_FUNC_OUT_SIGNAL_COUNT ||
        signal == GPIO_FUNC_OUT_SOFTWARE)
        return;
    gpio->modeled_output_signal[signal / 64u] |=
        UINT64_C(1) << (signal % 64u);
}

void flexe_gpio_set_output_signal_sampled(flexe_gpio_t *gpio,
                                          unsigned signal)
{
    if (!gpio || signal >= GPIO_FUNC_OUT_SIGNAL_COUNT ||
        signal == GPIO_FUNC_OUT_SOFTWARE)
        return;
    flexe_gpio_set_output_signal_modeled(gpio, signal);
    gpio->sampled_output_signal[signal / 64u] |=
        UINT64_C(1) << (signal % 64u);
}

void flexe_gpio_drive_output_signal(flexe_gpio_t *gpio, unsigned signal,
                                    int level, int enabled)
{
    if (!gpio || signal >= GPIO_FUNC_OUT_SIGNAL_COUNT ||
        (gpio->modeled_output_signal[signal / 64u] &
         (UINT64_C(1) << (signal % 64u))) == 0u)
        return;
    uint64_t mask = UINT64_C(1) << (signal % 64u);
    unsigned word = signal / 64u;
    bool valid = level >= 0 && enabled >= 0;
    bool old_valid = (gpio->driven_output_signal[word] & mask) != 0u;
    bool old_level = (gpio->output_signal_level[word] & mask) != 0u;
    bool old_enable = (gpio->output_signal_enable[word] & mask) != 0u;
    if (old_valid == valid && (!valid ||
        (old_level == (level != 0) && old_enable == (enabled != 0)))) return;
    if (valid) gpio->driven_output_signal[word] |= mask;
    else gpio->driven_output_signal[word] &= ~mask;
    if (level > 0) gpio->output_signal_level[word] |= mask;
    else gpio->output_signal_level[word] &= ~mask;
    if (enabled > 0) gpio->output_signal_enable[word] |= mask;
    else gpio->output_signal_enable[word] &= ~mask;
    for (unsigned pin = 0u; pin < gpio->target->gpio.gpio_count; pin++)
        if ((gpio->func_out[pin] & GPIO_FUNC_OUT_SIGNAL_MASK) == signal)
            gpio_notify_pin(gpio, pin);
}

void flexe_gpio_set_output_sample_handler(flexe_gpio_t *gpio,
                                          flexe_gpio_output_sample_fn sample,
                                          void *ctx)
{
    if (!gpio) return;
    gpio->output_sample = sample;
    gpio->output_sample_ctx = sample ? ctx : NULL;
}

static bool gpio_output_signal_modeled(const flexe_gpio_t *gpio,
                                       unsigned signal)
{
    return signal < GPIO_FUNC_OUT_SIGNAL_COUNT &&
           (gpio->modeled_output_signal[signal / 64u] &
            (UINT64_C(1) << (signal % 64u))) != 0u;
}

static bool gpio_output_signal_sampled(const flexe_gpio_t *gpio,
                                       unsigned signal)
{
    return signal < GPIO_FUNC_OUT_SIGNAL_COUNT &&
           (gpio->sampled_output_signal[signal / 64u] &
            (UINT64_C(1) << (signal % 64u))) != 0u;
}

static uint32_t gpio_output_sample_candidates(const flexe_gpio_t *gpio,
                                              unsigned bank)
{
    if (!gpio->output_sample) return 0u;
    uint64_t unavailable = gpio->held_pins | gpio->rtc_owned;
    uint32_t blocked = bank == 0u ? (uint32_t)unavailable :
                                    (uint32_t)(unavailable >> 32u);
    uint32_t pins = gpio->input_enable[bank] &
                    gpio_valid_bank_mask(gpio, bank) &
                    ~gpio->host_valid[bank] & ~blocked;
    uint32_t sampled = 0u;
    while (pins != 0u) {
        unsigned bit = (unsigned)__builtin_ctz(pins);
        uint32_t mask = 1u << bit;
        pins &= pins - 1u;
        unsigned pin = bank * 32u + bit;
        unsigned signal = gpio->func_out[pin] & GPIO_FUNC_OUT_SIGNAL_MASK;
        if (gpio_output_signal_sampled(gpio, signal)) sampled |= mask;
    }
    return sampled;
}

static int gpio_sample_output_pad(flexe_gpio_t *gpio, unsigned pin)
{
    if (!gpio->output_sample) return -1;
    uint32_t route = gpio->func_out[pin];
    unsigned signal = route & GPIO_FUNC_OUT_SIGNAL_MASK;
    if (!gpio_output_signal_modeled(gpio, signal)) return -1;
    int level = -1;
    int enabled = -1;
    if (!gpio->output_sample(gpio->output_sample_ctx, signal,
                             &level, &enabled) || level < 0 ||
        (enabled < 0 && (route & GPIO_FUNC_OUT_OEN_SELECT) == 0u))
        return -1;
    if (route & GPIO_FUNC_OUT_INVERT) level = !level;
    if (route & GPIO_FUNC_OUT_OEN_SELECT) {
        unsigned bank;
        uint32_t mask;
        if (!gpio_pin_bit(pin, &bank, &mask)) return -1;
        enabled = (gpio->enable[bank] & mask) != 0u;
    }
    if (route & GPIO_FUNC_OUT_OEN_INVERT) enabled = !enabled;
    if (enabled && (gpio->pin[pin] & GPIO_PIN_OPEN_DRAIN) && level)
        enabled = 0;
    return enabled && level;
}

bool flexe_gpio_output_signal_has_input_consumer(const flexe_gpio_t *gpio,
                                                  unsigned signal)
{
    if (!gpio || signal >= GPIO_FUNC_OUT_SIGNAL_COUNT) return false;
    for (unsigned pin = 0u; pin < gpio->target->gpio.gpio_count; pin++) {
        if ((gpio->func_out[pin] & GPIO_FUNC_OUT_SIGNAL_MASK) != signal ||
            !gpio_pin_valid(gpio, pin) ||
            (gpio->held_pins & (UINT64_C(1) << pin)) != 0u ||
            (gpio->rtc_owned & (UINT64_C(1) << pin)) != 0u)
            continue;
        unsigned bank;
        uint32_t mask;
        if (!gpio_pin_bit(pin, &bank, &mask)) continue;
        if ((gpio->input_enable[bank] & mask) == 0u ||
            (gpio->host_valid[bank] & mask) != 0u)
            continue;
        if ((gpio->pin[pin] & GPIO_PIN_INT_ENABLE_MASK) != 0u)
            return true;
        for (unsigned word = 0u; word < GPIO_FUNC_IN_WORDS; word++) {
            uint64_t watched = gpio->watched_input_signal[word];
            while (watched != 0u) {
                unsigned input = word * 64u +
                                 (unsigned)__builtin_ctzll(watched);
                watched &= watched - 1u;
                uint32_t route = gpio->func_in[input];
                if ((route & GPIO_FUNC_IN_MATRIX) != 0u &&
                    (route & 0x3Fu) == pin)
                    return true;
            }
        }
    }
    return false;
}

void flexe_gpio_set_input_signal_handler(flexe_gpio_t *gpio,
                                         flexe_gpio_input_signal_fn changed,
                                         void *ctx)
{
    if (!gpio) return;
    gpio->input_signal_changed = changed;
    gpio->input_signal_ctx = changed ? ctx : NULL;
}

void flexe_gpio_set_input_route_handler(flexe_gpio_t *gpio,
                                        flexe_gpio_input_route_fn changed,
                                        void *ctx)
{
    if (!gpio) return;
    gpio->input_route_changed = changed;
    gpio->input_route_ctx = changed ? ctx : NULL;
}

void flexe_gpio_watch_input_signal(flexe_gpio_t *gpio, unsigned signal)
{
    if (!gpio || signal >= GPIO_FUNC_IN_COUNT) return;
    gpio->watched_input_signal[signal / 64u] |=
        UINT64_C(1) << (signal % 64u);
}

int flexe_gpio_pin_level(const flexe_gpio_t *gpio, unsigned pin)
{
    if (!gpio_pin_valid(gpio, pin)) return -1;
    if (gpio->held_pins & (UINT64_C(1) << pin))
        return gpio->held_level[pin];
    if (gpio->rtc_owned & (UINT64_C(1) << pin)) {
        if (gpio->rtc_unknown & (UINT64_C(1) << pin)) return -1;
        return (gpio->rtc_output & (UINT64_C(1) << pin)) != 0u;
    }
    uint32_t route = gpio->func_out[pin];
    unsigned signal = route & GPIO_FUNC_OUT_SIGNAL_MASK;
    if (signal != GPIO_FUNC_OUT_SOFTWARE) {
        if ((gpio->driven_output_signal[signal / 64u] &
             (UINT64_C(1) << (signal % 64u))) == 0u)
            return -1;
        int level = (gpio->output_signal_level[signal / 64u] &
                     (UINT64_C(1) << (signal % 64u))) != 0u;
        return (route & GPIO_FUNC_OUT_INVERT) ? !level : level;
    }
    unsigned bank;
    uint32_t mask;
    if (!gpio_pin_bit(pin, &bank, &mask)) return -1;
    int level = (gpio->out[bank] & mask) != 0u;
    if (route & GPIO_FUNC_OUT_INVERT) level = !level;
    return level;
}

int flexe_gpio_output_enabled(const flexe_gpio_t *gpio, unsigned pin)
{
    if (!gpio_pin_valid(gpio, pin)) return -1;
    if (gpio->held_pins & (UINT64_C(1) << pin))
        return gpio->held_enable[pin];
    if (gpio->rtc_owned & (UINT64_C(1) << pin)) {
        if (gpio->rtc_unknown & (UINT64_C(1) << pin)) return -1;
        return (gpio->rtc_enabled & (UINT64_C(1) << pin)) != 0u;
    }
    uint32_t route = gpio->func_out[pin];
    unsigned signal = route & GPIO_FUNC_OUT_SIGNAL_MASK;
    if (signal != GPIO_FUNC_OUT_SOFTWARE &&
        (route & GPIO_FUNC_OUT_OEN_SELECT) == 0u) {
        if ((gpio->driven_output_signal[signal / 64u] &
             (UINT64_C(1) << (signal % 64u))) == 0u)
            return -1;
        int enabled = (gpio->output_signal_enable[signal / 64u] &
                       (UINT64_C(1) << (signal % 64u))) != 0u;
        if (route & GPIO_FUNC_OUT_OEN_INVERT) enabled = !enabled;
        if (enabled && (gpio->pin[pin] & GPIO_PIN_OPEN_DRAIN) != 0u) {
            int level = flexe_gpio_pin_level(gpio, pin);
            if (level < 0) return -1;
            if (level != 0) enabled = 0;
        }
        return enabled;
    }
    unsigned bank;
    uint32_t mask;
    if (!gpio_pin_bit(pin, &bank, &mask)) return -1;
    int enabled = (gpio->enable[bank] & mask) != 0u;
    if (route & GPIO_FUNC_OUT_OEN_INVERT) enabled = !enabled;
    if (enabled && (gpio->pin[pin] & GPIO_PIN_OPEN_DRAIN) != 0u) {
        int level = flexe_gpio_pin_level(gpio, pin);
        if (level < 0) return -1;
        if (level != 0) enabled = 0;
    }
    return enabled;
}

static void gpio_update_irq(flexe_gpio_t *gpio);

static void gpio_notify_input_signals(flexe_gpio_t *gpio, unsigned pin,
                                      bool old_pad, bool new_pad,
                                      bool old_enabled, bool new_enabled)
{
    bool before_pad = old_enabled && old_pad;
    bool after_pad = new_enabled && new_pad;
    if (!gpio->input_signal_changed || before_pad == after_pad) return;
    for (unsigned word = 0u; word < GPIO_FUNC_IN_WORDS; word++) {
        uint64_t watched = gpio->watched_input_signal[word];
        while (watched != 0u) {
            unsigned signal = word * 64u +
                              (unsigned)__builtin_ctzll(watched);
            watched &= watched - 1u;
            uint32_t route = gpio->func_in[signal];
            if ((route & GPIO_FUNC_IN_MATRIX) == 0u ||
                (route & 0x3Fu) != pin)
                continue;
            bool invert = (route & GPIO_FUNC_IN_INVERT) != 0u;
            gpio->input_signal_changed(gpio->input_signal_ctx, signal,
                                       before_pad ^ invert,
                                       after_pad ^ invert);
        }
    }
}

static void gpio_refresh_input_pin(flexe_gpio_t *gpio, unsigned pin)
{
    unsigned bank;
    uint32_t mask;
    if (!gpio_pin_bit(pin, &bank, &mask)) return;
    bool old = (gpio->input[bank] & mask) != 0u;
    bool old_unknown = (gpio->peripheral_unknown[bank] & mask) != 0u;
    bool level;
    bool unknown = false;
    if (gpio->host_valid[bank] & mask) {
        level = (gpio->host_input[bank] & mask) != 0u;
    } else {
        /* Without an external sample, only a known actively driven digital
         * output feeds back. An unattached/unsupported peripheral producer
         * is unknown, not a falling edge from its former idle value. */
        int enabled = flexe_gpio_output_enabled(gpio, pin);
        int drive = flexe_gpio_pin_level(gpio, pin);
        unknown = enabled < 0 || drive < 0;
        level = enabled == 1 && drive == 1;
    }
    if (old == level && old_unknown == unknown) return;
    if (old_unknown != unknown)
        gpio->unknown_input_read_reported[bank] = false;
    if (unknown) gpio->peripheral_unknown[bank] |= mask;
    else gpio->peripheral_unknown[bank] &= ~mask;
    if (level) gpio->input[bank] |= mask;
    else       gpio->input[bank] &= ~mask;

    if (!old_unknown && !unknown && old != level &&
        (gpio->rtc_owned & (UINT64_C(1) << pin)) == 0u &&
        (gpio->input_enable[bank] & mask) != 0u) {
        uint32_t config = gpio->pin[pin];
        if (config & GPIO_PIN_INT_ENABLE_MASK) {
            unsigned type = (config & GPIO_PIN_INT_TYPE_MASK) >>
                            GPIO_PIN_INT_TYPE_SHIFT;
            bool fire = type == 3u || (type == 1u && level) ||
                        (type == 2u && !level) ||
                        (type == 4u && !level) ||
                        (type == 5u && level);
            if (fire) gpio->status[bank] |= mask;
        }
    }
    gpio_update_irq(gpio);
    if (!old_unknown && !unknown && old != level &&
        (gpio->rtc_owned & (UINT64_C(1) << pin)) == 0u)
        gpio_notify_input_signals(gpio, pin, old, level,
                                  (gpio->input_enable[bank] & mask) != 0u,
                                  (gpio->input_enable[bank] & mask) != 0u);
}

static void gpio_notify_pin(flexe_gpio_t *gpio, unsigned pin)
{
    if (!gpio_pin_valid(gpio, pin)) return;
    if (gpio->held_pins & (UINT64_C(1) << pin)) return;
    gpio_refresh_input_pin(gpio, pin);
    if (gpio->output_changed)
        gpio->output_changed(gpio->output_ctx, pin,
                             flexe_gpio_pin_level(gpio, pin),
                             flexe_gpio_output_enabled(gpio, pin));
}

void flexe_gpio_set_rtc_state(flexe_gpio_t *gpio, uint64_t owned,
                              uint64_t unknown, uint64_t output,
                              uint64_t enabled)
{
    if (!gpio) return;
    uint64_t valid = gpio->target->gpio.valid_gpio_mask;
    owned &= valid;
    unknown &= owned;
    output &= valid;
    enabled &= valid;
    uint64_t changed = (gpio->rtc_owned ^ owned) |
        (gpio->rtc_unknown ^ unknown) |
        ((gpio->rtc_output ^ output) & (gpio->rtc_owned | owned)) |
        ((gpio->rtc_enabled ^ enabled) & (gpio->rtc_owned | owned));
    gpio->rtc_owned = owned;
    gpio->rtc_unknown = unknown;
    gpio->rtc_output = output;
    gpio->rtc_enabled = enabled;
    while (changed) {
        unsigned pin = (unsigned)__builtin_ctzll(changed);
        changed &= changed - 1u;
        gpio_notify_pin(gpio, pin);
    }
}

void flexe_gpio_set_pad_hold(flexe_gpio_t *gpio, uint64_t held_pins)
{
    if (!gpio) return;
    uint64_t changed = gpio->held_pins ^ held_pins;
    while (changed) {
        unsigned pin = (unsigned)__builtin_ctzll(changed);
        uint64_t bit = UINT64_C(1) << pin;
        changed &= changed - 1u;
        if (!gpio_pin_valid(gpio, pin)) continue;
        if (held_pins & bit) {
            /* Capture the resolved pad before enabling hold, including an
             * unknown peripheral-produced signal (-1). */
            gpio->held_level[pin] = (int8_t)flexe_gpio_pin_level(gpio, pin);
            gpio->held_enable[pin] =
                (int8_t)flexe_gpio_output_enabled(gpio, pin);
        } else {
            int old_level = gpio->held_level[pin];
            int old_enable = gpio->held_enable[pin];
            gpio->held_pins &= ~bit;
            if (old_level != flexe_gpio_pin_level(gpio, pin) ||
                old_enable != flexe_gpio_output_enabled(gpio, pin))
                gpio_notify_pin(gpio, pin);
        }
    }
    gpio->held_pins = held_pins & gpio->target->gpio.valid_gpio_mask;
}

void flexe_gpio_pad_hold_snapshot(const flexe_gpio_t *gpio,
                                  flexe_gpio_pad_hold_t *out)
{
    if (!out) return;
    memset(out, 0, sizeof(*out));
    if (!gpio) return;
    out->mask = gpio->held_pins;
    uint64_t pins = out->mask;
    while (pins) {
        unsigned pin = (unsigned)__builtin_ctzll(pins);
        uint64_t bit = UINT64_C(1) << pin;
        pins &= pins - 1u;
        if (gpio->held_level[pin] >= 0) {
            out->level_known |= bit;
            if (gpio->held_level[pin]) out->levels |= bit;
        }
        if (gpio->held_enable[pin] >= 0) {
            out->enable_known |= bit;
            if (gpio->held_enable[pin]) out->enables |= bit;
        }
    }
}

void flexe_gpio_pad_hold_restore(flexe_gpio_t *gpio,
                                 const flexe_gpio_pad_hold_t *in)
{
    if (!gpio || !in) return;
    flexe_gpio_set_pad_hold(gpio, in->mask);
    uint64_t pins = in->mask & gpio->target->gpio.valid_gpio_mask;
    while (pins) {
        unsigned pin = (unsigned)__builtin_ctzll(pins);
        uint64_t bit = UINT64_C(1) << pin;
        pins &= pins - 1u;
        int level = (in->level_known & bit) ?
                    (in->levels & bit) != 0u : -1;
        int enabled = (in->enable_known & bit) ?
                      (in->enables & bit) != 0u : -1;
        if (gpio->held_level[pin] != level ||
            gpio->held_enable[pin] != enabled) {
            gpio->held_level[pin] = (int8_t)level;
            gpio->held_enable[pin] = (int8_t)enabled;
            gpio_refresh_input_pin(gpio, pin);
            if (gpio->output_changed)
                gpio->output_changed(gpio->output_ctx, pin,
                                     level, enabled);
        }
    }
}

static void gpio_notify_mask(flexe_gpio_t *gpio, unsigned bank,
                             uint32_t changed)
{
    changed &= gpio_valid_bank_mask(gpio, bank);
    while (changed != 0u) {
        unsigned bit = (unsigned)__builtin_ctz(changed);
        changed &= changed - 1u;
        gpio_notify_pin(gpio, bank * 32u + bit);
    }
}

static uint32_t gpio_routed_status(const flexe_gpio_t *gpio, unsigned bank,
                                   uint32_t enable_mask)
{
    uint32_t routed = 0u;
    uint32_t pending = gpio->status[bank] & gpio_valid_bank_mask(gpio, bank);
    while (pending != 0u) {
        unsigned bit = (unsigned)__builtin_ctz(pending);
        uint32_t mask = 1u << bit;
        pending &= pending - 1u;
        unsigned pin = bank * 32u + bit;
        if (gpio->pin[pin] & enable_mask) routed |= mask;
    }
    return routed;
}

static void gpio_latch_active_levels(flexe_gpio_t *gpio)
{
    uint64_t valid = gpio->target->gpio.valid_gpio_mask;
    for (unsigned pin = 0u; pin < gpio->target->gpio.gpio_count; pin++) {
        if ((valid & (UINT64_C(1) << pin)) == 0u) continue;
        uint32_t config = gpio->pin[pin];
        if ((config & GPIO_PIN_INT_ENABLE_MASK) == 0u) continue;
        unsigned type = (config & GPIO_PIN_INT_TYPE_MASK) >>
                        GPIO_PIN_INT_TYPE_SHIFT;
        if (type != 4u && type != 5u) continue;
        unsigned bank;
        uint32_t mask;
        if (!gpio_pin_bit(pin, &bank, &mask)) continue;
        if ((gpio->rtc_owned & (UINT64_C(1) << pin)) != 0u ||
            (gpio->peripheral_unknown[bank] & mask) != 0u ||
            (gpio->input_enable[bank] & mask) == 0u)
            continue;
        bool high = (gpio->input[bank] & mask) != 0u;
        if ((type == 4u && !high) || (type == 5u && high))
            gpio->status[bank] |= mask;
    }
}

static void gpio_update_irq(flexe_gpio_t *gpio)
{
    gpio_latch_active_levels(gpio);
    bool level[2] = {
        (gpio_routed_status(gpio, 0u, GPIO_PIN_NORMAL_ENABLE) |
         gpio_routed_status(gpio, 1u, GPIO_PIN_NORMAL_ENABLE)) != 0u,
        (gpio_routed_status(gpio, 0u, GPIO_PIN_NMI_ENABLE) |
         gpio_routed_status(gpio, 1u, GPIO_PIN_NMI_ENABLE)) != 0u,
    };
    for (unsigned nmi = 0u; nmi < 2u; nmi++) {
        if (level[nmi] == gpio->irq_level[nmi]) continue;
        gpio->irq_level[nmi] = level[nmi];
        if (gpio->irq_changed)
            gpio->irq_changed(gpio->irq_ctx, nmi != 0u, level[nmi]);
    }
}

static uint32_t gpio_read(void *ctx, uint32_t addr)
{
    flexe_gpio_t *gpio = ctx;
    const flexe_gpio_desc_t *desc = &gpio->target->gpio;
    uint32_t off = addr - desc->base;
    if ((off & 3u) != 0u) goto fallback;

    switch (off) {
    case GPIO_BT_SELECT_OFF: return gpio->bt_select;
    case GPIO_OUT_OFF: return gpio->out[0];
    case GPIO_OUT_SET_OFF:
    case GPIO_OUT_CLEAR_OFF:
    case GPIO_OUT1_SET_OFF:
    case GPIO_OUT1_CLEAR_OFF:
    case GPIO_ENABLE_SET_OFF:
    case GPIO_ENABLE_CLEAR_OFF:
    case GPIO_ENABLE1_SET_OFF:
    case GPIO_ENABLE1_CLEAR_OFF:
    case GPIO_STATUS_SET_OFF:
    case GPIO_STATUS_CLEAR_OFF:
    case GPIO_STATUS1_SET_OFF:
    case GPIO_STATUS1_CLEAR_OFF:
        return 0u; /* Write-only aliases. */
    case GPIO_OUT1_OFF: return gpio->out[1];
    case GPIO_SDIO_SELECT_OFF: return gpio->sdio_select;
    case GPIO_ENABLE_OFF: return gpio->enable[0];
    case GPIO_ENABLE1_OFF: return gpio->enable[1];
    case GPIO_STRAP_OFF: return desc->strap_reset & GPIO_STRAP_MASK;
    case GPIO_IN_OFF:
    case GPIO_IN1_OFF: {
        unsigned bank = off == GPIO_IN_OFF ? 0u : 1u;
        uint32_t rtc_owned = bank == 0u ?
            (uint32_t)gpio->rtc_owned :
            (uint32_t)(gpio->rtc_owned >> 32u);
        uint32_t unknown = gpio->peripheral_unknown[bank] &
                           gpio->input_enable[bank] & ~rtc_owned;
        uint32_t value = gpio->input[bank] &
                         gpio->input_enable[bank] & ~rtc_owned;
        uint32_t candidates = unknown |
            gpio_output_sample_candidates(gpio, bank);
        while (candidates != 0u) {
            unsigned bit = (unsigned)__builtin_ctz(candidates);
            uint32_t mask = 1u << bit;
            candidates &= candidates - 1u;
            int level = gpio_sample_output_pad(gpio, bank * 32u + bit);
            if (level < 0) continue;
            unknown &= ~mask;
            if (level) value |= mask;
            else       value &= ~mask;
        }
        if (!unknown) gpio->unknown_input_read_reported[bank] = false;
        if (unknown && !gpio->unknown_input_read_reported[bank]) {
            gpio->unknown_input_read_reported[bank] = true;
            if (gpio->fallback_read)
                (void)gpio->fallback_read(gpio->fallback_ctx, addr);
        }
        return value;
    }
    case GPIO_STATUS_OFF: return gpio->status[0];
    case GPIO_STATUS1_OFF: return gpio->status[1];
    case GPIO_CPU_INT_OFF:
        return gpio_routed_status(gpio, 0u, GPIO_PIN_NORMAL_ENABLE);
    case GPIO_CPU_NMI_INT_OFF:
        return gpio_routed_status(gpio, 0u, GPIO_PIN_NMI_ENABLE);
    case GPIO_CPUSDIO_INT_OFF:
        return gpio_routed_status(gpio, 0u, GPIO_PIN_SDIO_ENABLE);
    case GPIO_CPU_INT1_OFF:
        return gpio_routed_status(gpio, 1u, GPIO_PIN_NORMAL_ENABLE);
    case GPIO_CPU_NMI_INT1_OFF:
        return gpio_routed_status(gpio, 1u, GPIO_PIN_NMI_ENABLE);
    case GPIO_CPUSDIO_INT1_OFF:
        return gpio_routed_status(gpio, 1u, GPIO_PIN_SDIO_ENABLE);
    case GPIO_STATUS_NEXT_OFF: return gpio->status[0];
    case GPIO_STATUS_NEXT1_OFF: return gpio->status[1];
    case GPIO_CLOCK_GATE_OFF: return gpio->clock_gate;
    case GPIO_DATE_OFF: return gpio->date;
    default: break;
    }

    if (off >= GPIO_PIN_BASE_OFF &&
        off < GPIO_PIN_BASE_OFF + GPIO_PIN_REGISTER_COUNT * 4u)
        return gpio->pin[(off - GPIO_PIN_BASE_OFF) / 4u];
    if (off >= GPIO_FUNC_IN_BASE_OFF &&
        off < GPIO_FUNC_IN_BASE_OFF + GPIO_FUNC_IN_COUNT * 4u)
        return gpio->func_in[(off - GPIO_FUNC_IN_BASE_OFF) / 4u];
    if (off >= GPIO_FUNC_OUT_BASE_OFF &&
        off < GPIO_FUNC_OUT_BASE_OFF + GPIO_PIN_REGISTER_COUNT * 4u)
        return gpio->func_out[(off - GPIO_FUNC_OUT_BASE_OFF) / 4u];

fallback:
    return gpio->fallback_read ?
        gpio->fallback_read(gpio->fallback_ctx, addr) : 0u;
}

static void gpio_report_unsupported(flexe_gpio_t *gpio, uint32_t addr,
                                    uint32_t value)
{
    if (gpio->fallback_write)
        gpio->fallback_write(gpio->fallback_ctx, addr, value);
}

static void gpio_write(void *ctx, uint32_t addr, uint32_t value)
{
    flexe_gpio_t *gpio = ctx;
    const flexe_gpio_desc_t *desc = &gpio->target->gpio;
    uint32_t off = addr - desc->base;
    uint32_t old;
    uint32_t next;
    uint32_t changed;
    if ((off & 3u) != 0u) goto fallback;

    switch (off) {
    case GPIO_BT_SELECT_OFF:
        old = gpio->bt_select;
        gpio->bt_select = value;
        if (old != gpio->bt_select)
            gpio_report_unsupported(gpio, addr, value);
        return;
    case GPIO_OUT_OFF:
        old = gpio->out[0];
        gpio->out[0] = value;
        gpio_notify_mask(gpio, 0u, old ^ gpio->out[0]);
        return;
    case GPIO_OUT_SET_OFF:
        old = gpio->out[0];
        gpio->out[0] |= value;
        gpio_notify_mask(gpio, 0u, old ^ gpio->out[0]);
        return;
    case GPIO_OUT_CLEAR_OFF:
        old = gpio->out[0];
        gpio->out[0] &= ~value;
        gpio_notify_mask(gpio, 0u, old ^ gpio->out[0]);
        return;
    case GPIO_OUT1_OFF:
        old = gpio->out[1];
        gpio->out[1] = value & GPIO_HIGH_REGISTER_MASK;
        gpio_notify_mask(gpio, 1u, old ^ gpio->out[1]);
        return;
    case GPIO_OUT1_SET_OFF:
        old = gpio->out[1];
        gpio->out[1] |= value & GPIO_HIGH_REGISTER_MASK;
        gpio_notify_mask(gpio, 1u, old ^ gpio->out[1]);
        return;
    case GPIO_OUT1_CLEAR_OFF:
        old = gpio->out[1];
        gpio->out[1] &= ~(value & GPIO_HIGH_REGISTER_MASK);
        gpio_notify_mask(gpio, 1u, old ^ gpio->out[1]);
        return;
    case GPIO_SDIO_SELECT_OFF:
        old = gpio->sdio_select;
        gpio->sdio_select = value & GPIO_SDIO_SELECT_MASK;
        if (old != gpio->sdio_select)
            gpio_report_unsupported(gpio, addr, value);
        return;
    case GPIO_ENABLE_OFF:
        old = gpio->enable[0];
        gpio->enable[0] = value;
        gpio_notify_mask(gpio, 0u, old ^ gpio->enable[0]);
        return;
    case GPIO_ENABLE_SET_OFF:
        old = gpio->enable[0];
        gpio->enable[0] |= value;
        gpio_notify_mask(gpio, 0u, old ^ gpio->enable[0]);
        return;
    case GPIO_ENABLE_CLEAR_OFF:
        old = gpio->enable[0];
        gpio->enable[0] &= ~value;
        gpio_notify_mask(gpio, 0u, old ^ gpio->enable[0]);
        return;
    case GPIO_ENABLE1_OFF:
        old = gpio->enable[1];
        gpio->enable[1] = value & GPIO_HIGH_REGISTER_MASK;
        gpio_notify_mask(gpio, 1u, old ^ gpio->enable[1]);
        return;
    case GPIO_ENABLE1_SET_OFF:
        old = gpio->enable[1];
        gpio->enable[1] |= value & GPIO_HIGH_REGISTER_MASK;
        gpio_notify_mask(gpio, 1u, old ^ gpio->enable[1]);
        return;
    case GPIO_ENABLE1_CLEAR_OFF:
        old = gpio->enable[1];
        gpio->enable[1] &= ~(value & GPIO_HIGH_REGISTER_MASK);
        gpio_notify_mask(gpio, 1u, old ^ gpio->enable[1]);
        return;
    case GPIO_STATUS_OFF:
        gpio->status[0] = value;
        gpio_update_irq(gpio);
        return;
    case GPIO_STATUS_SET_OFF:
        gpio->status[0] |= value;
        gpio_update_irq(gpio);
        return;
    case GPIO_STATUS_CLEAR_OFF:
        gpio->status[0] &= ~value;
        gpio_update_irq(gpio);
        return;
    case GPIO_STATUS1_OFF:
        gpio->status[1] = value & GPIO_HIGH_REGISTER_MASK;
        gpio_update_irq(gpio);
        return;
    case GPIO_STATUS1_SET_OFF:
        gpio->status[1] |= value & GPIO_HIGH_REGISTER_MASK;
        gpio_update_irq(gpio);
        return;
    case GPIO_STATUS1_CLEAR_OFF:
        gpio->status[1] &= ~(value & GPIO_HIGH_REGISTER_MASK);
        gpio_update_irq(gpio);
        return;
    case GPIO_STRAP_OFF:
    case GPIO_IN_OFF:
    case GPIO_IN1_OFF:
    case GPIO_CPU_INT_OFF:
    case GPIO_CPU_NMI_INT_OFF:
    case GPIO_CPUSDIO_INT_OFF:
    case GPIO_CPU_INT1_OFF:
    case GPIO_CPU_NMI_INT1_OFF:
    case GPIO_CPUSDIO_INT1_OFF:
    case GPIO_STATUS_NEXT_OFF:
    case GPIO_STATUS_NEXT1_OFF:
        return; /* Hardware-owned/read-only. */
    case GPIO_CLOCK_GATE_OFF:
        old = gpio->clock_gate;
        gpio->clock_gate = value & GPIO_CLOCK_GATE_ENABLE;
        if (old != gpio->clock_gate && gpio->clock_gate == 0u)
            gpio_report_unsupported(gpio, addr, value);
        return;
    case GPIO_DATE_OFF:
        gpio->date = value & GPIO_DATE_MASK;
        return;
    default: break;
    }

    if (off >= GPIO_PIN_BASE_OFF &&
        off < GPIO_PIN_BASE_OFF + GPIO_PIN_REGISTER_COUNT * 4u) {
        unsigned pin = (off - GPIO_PIN_BASE_OFF) / 4u;
        old = gpio->pin[pin];
        next = value & GPIO_PIN_WRITABLE_MASK;
        changed = old ^ next;
        gpio->pin[pin] = next;
        unsigned type = (next & GPIO_PIN_INT_TYPE_MASK) >>
                        GPIO_PIN_INT_TYPE_SHIFT;
        if ((changed & ~GPIO_PIN_MODELED_MASK) != 0u || type > 5u ||
            !gpio_pin_valid(gpio, pin))
            gpio_report_unsupported(gpio, addr, value);
        if ((changed & GPIO_PIN_OPEN_DRAIN) != 0u)
            gpio_notify_pin(gpio, pin);
        gpio_update_irq(gpio);
        return;
    }

    if (off >= GPIO_FUNC_IN_BASE_OFF &&
        off < GPIO_FUNC_IN_BASE_OFF + GPIO_FUNC_IN_COUNT * 4u) {
        unsigned signal = (off - GPIO_FUNC_IN_BASE_OFF) / 4u;
        uint32_t route = value & GPIO_FUNC_IN_MASK;
        old = gpio->func_in[signal];
        gpio->func_in[signal] = route;
        if (old != route && gpio->input_route_changed)
            gpio->input_route_changed(gpio->input_route_ctx, signal);
        return;
    }

    if (off >= GPIO_FUNC_OUT_BASE_OFF &&
        off < GPIO_FUNC_OUT_BASE_OFF + GPIO_PIN_REGISTER_COUNT * 4u) {
        unsigned pin = (off - GPIO_FUNC_OUT_BASE_OFF) / 4u;
        unsigned signal;
        old = gpio->func_out[pin];
        next = value & GPIO_FUNC_OUT_MASK;
        gpio->func_out[pin] = next;
        if (old != next) {
            gpio_notify_pin(gpio, pin);
            signal = next & GPIO_FUNC_OUT_SIGNAL_MASK;
            if (!gpio_pin_valid(gpio, pin) ||
                (signal != GPIO_FUNC_OUT_SOFTWARE &&
                 !gpio_output_signal_modeled(gpio, signal)))
                gpio_report_unsupported(gpio, addr, value);
        }
        return;
    }

fallback:
    gpio_report_unsupported(gpio, addr, value);
}

flexe_gpio_t *flexe_gpio_create(
    xtensa_mem_t *mem, mmio_read_fn fallback_read,
    mmio_write_fn fallback_write, void *fallback_ctx,
    flexe_gpio_output_fn output_changed, void *output_ctx,
    flexe_gpio_irq_fn irq_changed, void *irq_ctx)
{
    if (!mem) return NULL;
    const flexe_target_desc_t *target = mem_target(mem);
    if (!gpio_geometry_valid(target)) return NULL;

    flexe_gpio_t *gpio = calloc(1u, sizeof(*gpio));
    if (!gpio) return NULL;
    gpio->mem = mem;
    gpio->target = target;
    gpio->fallback_read = fallback_read;
    gpio->fallback_write = fallback_write;
    gpio->fallback_ctx = fallback_ctx;
    gpio->output_changed = output_changed;
    gpio->output_ctx = output_ctx;
    gpio->irq_changed = irq_changed;
    gpio->irq_ctx = irq_ctx;
    gpio->clock_gate = GPIO_CLOCK_GATE_ENABLE;
    gpio->date = target->gpio.date_reset & GPIO_DATE_MASK;
    gpio->input_enable[0] = (uint32_t)target->gpio.valid_gpio_mask;
    gpio->input_enable[1] =
        (uint32_t)(target->gpio.valid_gpio_mask >> 32u);
    for (unsigned pin = 0u; pin < GPIO_PIN_REGISTER_COUNT; pin++)
        gpio->func_out[pin] = GPIO_FUNC_OUT_SOFTWARE;

    if (mem_register_mmio_range(mem, target->gpio.base,
                                target->gpio.register_size,
                                gpio_read, gpio_write, gpio) != 0) {
        free(gpio);
        return NULL;
    }
    return gpio;
}

void flexe_gpio_destroy(flexe_gpio_t *gpio)
{
    if (!gpio) return;
    if (gpio->irq_changed) {
        if (gpio->irq_level[0])
            gpio->irq_changed(gpio->irq_ctx, false, false);
        if (gpio->irq_level[1])
            gpio->irq_changed(gpio->irq_ctx, true, false);
    }
    const flexe_gpio_desc_t *desc = &gpio->target->gpio;
    (void)mem_register_mmio_range(
        gpio->mem, desc->base, desc->register_size,
        gpio->fallback_read, gpio->fallback_write, gpio->fallback_ctx);
    free(gpio);
}

void flexe_gpio_set_input_enable(flexe_gpio_t *gpio, unsigned pin,
                                 bool enabled)
{
    if (!gpio_pin_valid(gpio, pin)) return;
    unsigned bank;
    uint32_t mask;
    if (!gpio_pin_bit(pin, &bank, &mask)) return;
    bool old_enabled = (gpio->input_enable[bank] & mask) != 0u;
    if (old_enabled == enabled) return;
    if (enabled) gpio->input_enable[bank] |= mask;
    else         gpio->input_enable[bank] &= ~mask;
    gpio_update_irq(gpio);
    if ((gpio->rtc_owned & (UINT64_C(1) << pin)) == 0u &&
        (gpio->peripheral_unknown[bank] & mask) == 0u) {
        bool pad = (gpio->input[bank] & mask) != 0u;
        gpio_notify_input_signals(gpio, pin, pad, pad, old_enabled, enabled);
    }
}

void flexe_gpio_set_input(flexe_gpio_t *gpio, unsigned pin, bool level)
{
    if (!gpio_pin_valid(gpio, pin)) return;
    unsigned bank;
    uint32_t mask;
    if (!gpio_pin_bit(pin, &bank, &mask)) return;
    gpio->host_valid[bank] |= mask;
    if (level) gpio->host_input[bank] |= mask;
    else       gpio->host_input[bank] &= ~mask;
    gpio_refresh_input_pin(gpio, pin);
    gpio_update_irq(gpio);
}

int flexe_gpio_input_level(const flexe_gpio_t *gpio, unsigned pin)
{
    if (!gpio_pin_valid(gpio, pin)) return -1;
    unsigned bank;
    uint32_t mask;
    if (!gpio_pin_bit(pin, &bank, &mask)) return -1;
    if ((gpio->peripheral_unknown[bank] & mask) != 0u) return -1;
    return (gpio->input[bank] & mask) != 0u;
}

int flexe_gpio_input_signal_level(const flexe_gpio_t *gpio, unsigned signal)
{
    if (!gpio || signal >= GPIO_FUNC_IN_COUNT) return -1;
    uint32_t route = gpio->func_in[signal];
    if ((route & GPIO_FUNC_IN_MATRIX) == 0u) return -1;
    unsigned input = route & 0x3Fu;
    int level;
    if (input == gpio->target->gpio.matrix_const_one_input) {
        level = 1;
    } else if (input == gpio->target->gpio.matrix_const_zero_input) {
        level = 0;
    } else {
        if (!gpio_pin_valid(gpio, input)) return -1;
        if ((gpio->rtc_owned & (UINT64_C(1) << input)) != 0u)
            return -1;
        unsigned bank;
        uint32_t mask;
        if (!gpio_pin_bit(input, &bank, &mask)) return -1;
        if ((gpio->peripheral_unknown[bank] & mask) != 0u)
            return -1;
        level = (gpio->input_enable[bank] & mask) != 0u &&
                (gpio->input[bank] & mask) != 0u;
    }
    if (route & GPIO_FUNC_IN_INVERT) level = !level;
    return level;
}
