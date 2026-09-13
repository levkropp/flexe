#include "rtc_io.h"

#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

/* ESP32-S3 rtc_io_reg.h. Output, output-enable, sampled input and the
 * pad-owner mux have functional effects, including RTC_CNTL pad hold. Other
 * documented pad fields retain their register value but remain diagnostic
 * when changed. */
#define RTC_GPIO_OUT_OFF         0x000u
#define RTC_GPIO_OUT_SET_OFF     0x004u
#define RTC_GPIO_OUT_CLEAR_OFF   0x008u
#define RTC_GPIO_ENABLE_OFF      0x00Cu
#define RTC_GPIO_ENABLE_SET_OFF  0x010u
#define RTC_GPIO_ENABLE_CLEAR_OFF 0x014u
#define RTC_GPIO_IN_OFF          0x024u
#define RTC_PAD_COMMON_MASK      0x780FE000u
#define RTC_PAD_TOUCH_MASK       0x00700000u
#define RTC_PAD_DAC_MASK         0x00001FF8u
#define RTC_PAD_FUNCTION_MASK    (3u << 17)
#define RTC_PAD_INPUT_ENABLE     (1u << 13)
#define RTC_PAD_TOUCH_ACTIVE     ((1u << 22) | (1u << 20))
#define RTC_PAD_DAC_ACTIVE       ((1u << 12) | (1u << 11))

struct flexe_rtc_io {
    xtensa_mem_t *mem;
    flexe_gpio_t *gpio;
    const flexe_target_desc_t *target;
    mmio_read_fn fallback_read;
    mmio_write_fn fallback_write;
    void *fallback_ctx;
    uint32_t output;
    uint32_t enabled;
    uint32_t pad[FLEXE_TARGET_RTC_IO_PIN_MAX];
    uint32_t held_mask;
    uint32_t held_pad[FLEXE_TARGET_RTC_IO_PIN_MAX];
};

static bool one_bit(uint32_t value)
{
    return value != 0u && (value & (value - 1u)) == 0u;
}

static uint32_t pin_mask(const flexe_rtc_io_desc_t *desc)
{
    return (1u << desc->gpio_count) - 1u;
}

static uint32_t pad_writable_mask(unsigned pin)
{
    uint32_t mask = RTC_PAD_COMMON_MASK;
    if (pin < 15u) mask |= RTC_PAD_TOUCH_MASK;
    if (pin == 17u || pin == 18u) mask |= RTC_PAD_DAC_MASK;
    return mask;
}

static uint32_t rtc_io_effective_pad(const flexe_rtc_io_t *rtc_io,
                                     unsigned pin)
{
    return (rtc_io->held_mask & (1u << pin)) != 0u ?
           rtc_io->held_pad[pin] : rtc_io->pad[pin];
}

static bool rtc_io_geometry_valid(const flexe_target_desc_t *target)
{
    if (!target || !(target->capabilities & FLEXE_TARGET_CAP_RTC_IO_V1) ||
        !(target->capabilities & FLEXE_TARGET_CAP_GPIO_V1))
        return false;
    const flexe_rtc_io_desc_t *desc = &target->rtc_io;
    if ((desc->base & 3u) != 0u ||
        desc->register_size < 0x200u ||
        desc->base < target->peripheral_start ||
        desc->base >= target->peripheral_end ||
        desc->register_size > target->peripheral_end - desc->base ||
        desc->gpio_count != FLEXE_TARGET_RTC_IO_PIN_MAX ||
        desc->data_shift >= 32u ||
        desc->data_shift + desc->gpio_count > 32u ||
        desc->pad_base_offset < RTC_GPIO_IN_OFF + 4u ||
        (desc->pad_base_offset & 3u) != 0u ||
        desc->pad_base_offset + 4u * desc->gpio_count >
            desc->register_size ||
        !one_bit(desc->pad_mux_mask) ||
        desc->pad_mux_mask != (1u << 19) ||
        (target->gpio.valid_gpio_mask & pin_mask(desc)) !=
            pin_mask(desc))
        return false;
    for (unsigned pin = 0u; pin < desc->gpio_count; pin++)
        if ((desc->pad_reset[pin] &
             ~pad_writable_mask(pin)) != 0u ||
            (desc->pad_reset[pin] & desc->pad_mux_mask) != 0u)
            return false;
    return true;
}

static void rtc_io_publish(flexe_rtc_io_t *rtc_io)
{
    const flexe_rtc_io_desc_t *desc = &rtc_io->target->rtc_io;
    uint64_t owned = 0u;
    uint64_t unknown = 0u;
    for (unsigned pin = 0u; pin < desc->gpio_count; pin++) {
        uint32_t pad = rtc_io_effective_pad(rtc_io, pin);
        uint64_t bit = UINT64_C(1) << pin;
        if ((pad & desc->pad_mux_mask) == 0u) continue;
        owned |= bit;
        if ((pad & RTC_PAD_FUNCTION_MASK) != 0u ||
            (pin < 15u && (pad & RTC_PAD_TOUCH_ACTIVE) != 0u) ||
            ((pin == 17u || pin == 18u) &&
             (pad & RTC_PAD_DAC_ACTIVE) != 0u))
            unknown |= bit;
    }
    flexe_gpio_set_rtc_state(rtc_io->gpio, owned, unknown,
                             rtc_io->output, rtc_io->enabled);
}

static uint32_t rtc_io_fallback_read(flexe_rtc_io_t *rtc_io,
                                     uint32_t addr)
{
    return rtc_io->fallback_read ?
        rtc_io->fallback_read(rtc_io->fallback_ctx, addr) : 0u;
}

static void rtc_io_fallback_write(flexe_rtc_io_t *rtc_io,
                                  uint32_t addr, uint32_t value)
{
    if (rtc_io->fallback_write)
        rtc_io->fallback_write(rtc_io->fallback_ctx, addr, value);
}

uint32_t flexe_rtc_io_mmio_read(void *ctx, uint32_t addr)
{
    flexe_rtc_io_t *rtc_io = ctx;
    const flexe_rtc_io_desc_t *desc = &rtc_io->target->rtc_io;
    if (addr < desc->base || addr - desc->base >= desc->register_size)
        return rtc_io_fallback_read(rtc_io, addr);
    uint32_t off = addr - desc->base;
    switch (off) {
    case RTC_GPIO_OUT_OFF:
        return rtc_io->output << desc->data_shift;
    case RTC_GPIO_ENABLE_OFF:
        return rtc_io->enabled << desc->data_shift;
    case RTC_GPIO_OUT_SET_OFF:
    case RTC_GPIO_OUT_CLEAR_OFF:
    case RTC_GPIO_ENABLE_SET_OFF:
    case RTC_GPIO_ENABLE_CLEAR_OFF:
        return 0u;
    case RTC_GPIO_IN_OFF: {
        uint32_t inputs = 0u;
        for (unsigned pin = 0u; pin < desc->gpio_count; pin++)
            if ((rtc_io_effective_pad(rtc_io, pin) &
                 (desc->pad_mux_mask | RTC_PAD_INPUT_ENABLE |
                  RTC_PAD_FUNCTION_MASK)) ==
                    (desc->pad_mux_mask | RTC_PAD_INPUT_ENABLE) &&
                flexe_gpio_input_level(rtc_io->gpio, pin) == 1)
                inputs |= 1u << pin;
        return inputs << desc->data_shift;
    }
    default:
        break;
    }
    if (off >= desc->pad_base_offset &&
        off < desc->pad_base_offset + 4u * desc->gpio_count &&
        (off & 3u) == 0u)
        return rtc_io->pad[(off - desc->pad_base_offset) / 4u];
    return rtc_io_fallback_read(rtc_io, addr);
}

void flexe_rtc_io_mmio_write(void *ctx, uint32_t addr, uint32_t value)
{
    flexe_rtc_io_t *rtc_io = ctx;
    const flexe_rtc_io_desc_t *desc = &rtc_io->target->rtc_io;
    if (addr < desc->base || addr - desc->base >= desc->register_size) {
        rtc_io_fallback_write(rtc_io, addr, value);
        return;
    }
    uint32_t off = addr - desc->base;
    uint32_t data_mask = pin_mask(desc) << desc->data_shift;
    uint32_t data = (value & data_mask) >> desc->data_shift;
    switch (off) {
    case RTC_GPIO_OUT_OFF:
        rtc_io->output = data;
        break;
    case RTC_GPIO_OUT_SET_OFF:
        rtc_io->output |= data;
        break;
    case RTC_GPIO_OUT_CLEAR_OFF:
        rtc_io->output &= ~data;
        break;
    case RTC_GPIO_ENABLE_OFF:
        rtc_io->enabled = data;
        break;
    case RTC_GPIO_ENABLE_SET_OFF:
        rtc_io->enabled |= data;
        break;
    case RTC_GPIO_ENABLE_CLEAR_OFF:
        rtc_io->enabled &= ~data;
        break;
    default:
        goto pad_or_fallback;
    }
    if ((value & ~data_mask) != 0u)
        rtc_io_fallback_write(rtc_io, addr, value);
    rtc_io_publish(rtc_io);
    return;

pad_or_fallback:
    if (off >= desc->pad_base_offset &&
        off < desc->pad_base_offset + 4u * desc->gpio_count &&
        (off & 3u) == 0u) {
        unsigned pin = (off - desc->pad_base_offset) / 4u;
        uint32_t old = rtc_io->pad[pin];
        uint32_t writable = pad_writable_mask(pin);
        rtc_io->pad[pin] = value & writable;
        if (((old ^ rtc_io->pad[pin]) &
             ~(desc->pad_mux_mask | RTC_PAD_INPUT_ENABLE)) != 0u ||
            (value & ~writable) != 0u)
            rtc_io_fallback_write(rtc_io, addr, value);
        rtc_io_publish(rtc_io);
        return;
    }
    rtc_io_fallback_write(rtc_io, addr, value);
}

flexe_rtc_io_t *flexe_rtc_io_create(
    xtensa_mem_t *mem, flexe_gpio_t *gpio,
    mmio_read_fn fallback_read, mmio_write_fn fallback_write,
    void *fallback_ctx)
{
    if (!mem || !gpio) return NULL;
    const flexe_target_desc_t *target = mem_target(mem);
    if (!rtc_io_geometry_valid(target)) return NULL;
    flexe_rtc_io_t *rtc_io = calloc(1u, sizeof(*rtc_io));
    if (!rtc_io) return NULL;
    rtc_io->mem = mem;
    rtc_io->gpio = gpio;
    rtc_io->target = target;
    rtc_io->fallback_read = fallback_read;
    rtc_io->fallback_write = fallback_write;
    rtc_io->fallback_ctx = fallback_ctx;
    for (unsigned pin = 0u; pin < target->rtc_io.gpio_count; pin++)
        rtc_io->pad[pin] = target->rtc_io.pad_reset[pin];
    if (mem_register_mmio_range(mem, target->rtc_io.base,
                                target->rtc_io.register_size,
                                flexe_rtc_io_mmio_read,
                                flexe_rtc_io_mmio_write, rtc_io) != 0) {
        free(rtc_io);
        return NULL;
    }
    return rtc_io;
}

void flexe_rtc_io_destroy(flexe_rtc_io_t *rtc_io)
{
    if (!rtc_io) return;
    flexe_gpio_set_rtc_state(rtc_io->gpio, 0u, 0u, 0u, 0u);
    const flexe_rtc_io_desc_t *desc = &rtc_io->target->rtc_io;
    (void)mem_register_mmio_range(
        rtc_io->mem, desc->base, desc->register_size,
        rtc_io->fallback_read, rtc_io->fallback_write,
        rtc_io->fallback_ctx);
    free(rtc_io);
}

void flexe_rtc_io_set_pad_hold(flexe_rtc_io_t *rtc_io,
                                uint64_t held_pins)
{
    if (!rtc_io) return;
    uint32_t next = (uint32_t)held_pins & pin_mask(&rtc_io->target->rtc_io);
    uint32_t newly_held = next & ~rtc_io->held_mask;
    while (newly_held) {
        unsigned pin = (unsigned)__builtin_ctz(newly_held);
        newly_held &= newly_held - 1u;
        rtc_io->held_pad[pin] = rtc_io->pad[pin];
    }
    if (next != rtc_io->held_mask) {
        rtc_io->held_mask = next;
        rtc_io_publish(rtc_io);
    }
}

void flexe_rtc_io_pad_hold_snapshot(const flexe_rtc_io_t *rtc_io,
                                    flexe_rtc_io_pad_hold_t *out)
{
    if (!out) return;
    memset(out, 0, sizeof(*out));
    if (!rtc_io) return;
    out->mask = rtc_io->held_mask;
    for (unsigned pin = 0u; pin < rtc_io->target->rtc_io.gpio_count; pin++)
        if ((out->mask & (1u << pin)) != 0u)
            out->pad[pin] = rtc_io->held_pad[pin];
}

void flexe_rtc_io_pad_hold_restore(flexe_rtc_io_t *rtc_io,
                                   const flexe_rtc_io_pad_hold_t *in)
{
    if (!rtc_io || !in) return;
    flexe_rtc_io_set_pad_hold(rtc_io, in->mask);
    for (unsigned pin = 0u; pin < rtc_io->target->rtc_io.gpio_count; pin++)
        if ((rtc_io->held_mask & (1u << pin)) != 0u)
            rtc_io->held_pad[pin] = in->pad[pin];
    rtc_io_publish(rtc_io);
}
