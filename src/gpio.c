#include "gpio.h"

#include <limits.h>
#include <stdint.h>
#include <stdlib.h>

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
#define GPIO_PIN_NORMAL_ENABLE      (1u << 13u)
#define GPIO_PIN_NMI_ENABLE         (1u << 14u)
#define GPIO_PIN_SDIO_ENABLE        (1u << 17u)
#define GPIO_PIN_MODELED_MASK       \
    (GPIO_PIN_INT_TYPE_MASK | GPIO_PIN_NORMAL_ENABLE | GPIO_PIN_NMI_ENABLE)
#define GPIO_FUNC_IN_MASK           0x000000FFu
#define GPIO_FUNC_IN_INVERT         (1u << 6u)
#define GPIO_FUNC_IN_MATRIX         (1u << 7u)
#define GPIO_FUNC_OUT_MASK          0x00000FFFu
#define GPIO_FUNC_OUT_SIGNAL_MASK   0x000001FFu
#define GPIO_FUNC_OUT_INVERT        (1u << 9u)
#define GPIO_FUNC_OUT_OEN_SELECT    (1u << 10u)
#define GPIO_FUNC_OUT_OEN_INVERT    (1u << 11u)
#define GPIO_FUNC_OUT_SOFTWARE      256u
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

    uint32_t bt_select;
    uint32_t sdio_select;
    uint32_t out[2];
    uint32_t enable[2];
    uint32_t input[2];
    uint32_t status[2];
    uint32_t pin[GPIO_PIN_REGISTER_COUNT];
    uint32_t func_in[GPIO_FUNC_IN_COUNT];
    uint32_t func_out[GPIO_PIN_REGISTER_COUNT];
    uint32_t clock_gate;
    uint32_t date;
    bool irq_level[2];
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

int flexe_gpio_pin_level(const flexe_gpio_t *gpio, unsigned pin)
{
    if (!gpio_pin_valid(gpio, pin)) return -1;
    uint32_t route = gpio->func_out[pin];
    if ((route & GPIO_FUNC_OUT_SIGNAL_MASK) != GPIO_FUNC_OUT_SOFTWARE)
        return -1;
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
    uint32_t route = gpio->func_out[pin];
    if ((route & GPIO_FUNC_OUT_SIGNAL_MASK) != GPIO_FUNC_OUT_SOFTWARE &&
        (route & GPIO_FUNC_OUT_OEN_SELECT) == 0u)
        return -1;
    unsigned bank;
    uint32_t mask;
    if (!gpio_pin_bit(pin, &bank, &mask)) return -1;
    int enabled = (gpio->enable[bank] & mask) != 0u;
    if (route & GPIO_FUNC_OUT_OEN_INVERT) enabled = !enabled;
    return enabled;
}

static void gpio_notify_pin(flexe_gpio_t *gpio, unsigned pin)
{
    if (!gpio->output_changed || !gpio_pin_valid(gpio, pin)) return;
    gpio->output_changed(gpio->output_ctx, pin,
                         flexe_gpio_pin_level(gpio, pin),
                         flexe_gpio_output_enabled(gpio, pin));
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
        (void)gpio_pin_bit(pin, &bank, &mask);
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
    case GPIO_IN_OFF: return gpio->input[0];
    case GPIO_IN1_OFF: return gpio->input[1];
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
        gpio_update_irq(gpio);
        return;
    }

    if (off >= GPIO_FUNC_IN_BASE_OFF &&
        off < GPIO_FUNC_IN_BASE_OFF + GPIO_FUNC_IN_COUNT * 4u) {
        unsigned signal = (off - GPIO_FUNC_IN_BASE_OFF) / 4u;
        gpio->func_in[signal] = value & GPIO_FUNC_IN_MASK;
        return;
    }

    if (off >= GPIO_FUNC_OUT_BASE_OFF &&
        off < GPIO_FUNC_OUT_BASE_OFF + GPIO_PIN_REGISTER_COUNT * 4u) {
        unsigned pin = (off - GPIO_FUNC_OUT_BASE_OFF) / 4u;
        old = gpio->func_out[pin];
        next = value & GPIO_FUNC_OUT_MASK;
        gpio->func_out[pin] = next;
        if (old != next) {
            gpio_notify_pin(gpio, pin);
            if (!gpio_pin_valid(gpio, pin) ||
                (next & GPIO_FUNC_OUT_SIGNAL_MASK) !=
                    GPIO_FUNC_OUT_SOFTWARE)
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

void flexe_gpio_set_input(flexe_gpio_t *gpio, unsigned pin, bool level)
{
    if (!gpio_pin_valid(gpio, pin)) return;
    unsigned bank;
    uint32_t mask;
    (void)gpio_pin_bit(pin, &bank, &mask);
    bool old = (gpio->input[bank] & mask) != 0u;
    if (level) gpio->input[bank] |= mask;
    else       gpio->input[bank] &= ~mask;

    if (old != level) {
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
        unsigned bank;
        uint32_t mask;
        (void)gpio_pin_bit(input, &bank, &mask);
        level = (gpio->input[bank] & mask) != 0u;
    }
    if (route & GPIO_FUNC_IN_INVERT) level = !level;
    return level;
}
