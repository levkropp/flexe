#include "regi2c.h"

#include <stdbool.h>
#include <stdlib.h>

struct flexe_regi2c {
    xtensa_mem_t *mem;
    const flexe_target_desc_t *target;
    mmio_read_fn fallback_read;
    mmio_write_fn fallback_write;
    void *fallback_ctx;
    uint32_t command[FLEXE_TARGET_REGI2C_HOST_MAX];
    uint32_t analog_control;
    uint32_t config;
    uint32_t config2;
    uint32_t aux_register[FLEXE_TARGET_REGI2C_AUX_REGISTER_MAX];
    uint8_t *registers;
    size_t address_count;
    unsigned bbpll_reads;
    bool bbpll_calibrating;
    bool bbpll_done;
};

static bool one_bit(uint32_t value)
{
    return value != 0u && (value & (value - 1u)) == 0u;
}

static bool byte_field_valid(uint32_t mask, unsigned shift)
{
    if (mask == 0u || shift >= 32u) return false;
    uint32_t below = shift == 0u ? 0u : (UINT32_MAX >> (32u - shift));
    uint32_t field = mask >> shift;
    return (mask & below) == 0u && field <= UINT8_MAX &&
           (field & (field + 1u)) == 0u;
}

static bool regi2c_geometry_valid(const flexe_target_desc_t *target)
{
    if (!target || !(target->capabilities & FLEXE_TARGET_CAP_REGI2C))
        return false;
    const flexe_regi2c_desc_t *desc = &target->regi2c;
    if ((desc->base & 0xFFFu) != 0u || desc->register_size < 4u ||
        desc->base < target->peripheral_start ||
        desc->base >= target->peripheral_end ||
        desc->register_size > target->peripheral_end - desc->base ||
        desc->host_count == 0u ||
        desc->host_count > FLEXE_TARGET_REGI2C_HOST_MAX ||
        desc->aux_register_count >
            FLEXE_TARGET_REGI2C_AUX_REGISTER_MAX ||
        (desc->command_offset & 3u) != 0u ||
        (desc->command_stride & 3u) != 0u ||
        desc->command_stride == 0u ||
        (desc->analog_control_offset & 3u) != 0u ||
        (desc->config_offset & 3u) != 0u ||
        (desc->config2_offset & 3u) != 0u ||
        desc->analog_control_offset > desc->register_size - 4u ||
        desc->config_offset > desc->register_size - 4u ||
        desc->config2_offset > desc->register_size - 4u)
        return false;

    for (unsigned index = 0u; index < desc->aux_register_count; index++) {
        uint32_t offset = desc->aux_register[index].offset;
        if ((offset & 3u) != 0u ||
            offset > desc->register_size - sizeof(uint32_t) ||
            offset == desc->analog_control_offset ||
            offset == desc->config_offset ||
            offset == desc->config2_offset)
            return false;
        for (unsigned host = 0u; host < desc->host_count; host++)
            if (offset == desc->command_offset +
                          host * desc->command_stride)
                return false;
        for (unsigned other = 0u; other < index; other++)
            if (offset == desc->aux_register[other].offset)
                return false;
    }

    uint64_t last_command = (uint64_t)desc->command_offset +
        (uint64_t)(desc->host_count - 1u) * desc->command_stride;
    if (last_command > desc->register_size - 4u ||
        !one_bit(desc->command_start_mask) ||
        !one_bit(desc->command_busy_mask) ||
        !one_bit(desc->command_write_mask) ||
        !one_bit(desc->bbpll_stop_high_mask) ||
        !one_bit(desc->bbpll_stop_low_mask) ||
        !one_bit(desc->bbpll_done_mask) ||
        !byte_field_valid(desc->slave_mask, desc->slave_shift) ||
        !byte_field_valid(desc->address_mask, desc->address_shift) ||
        !byte_field_valid(desc->data_mask, desc->data_shift))
        return false;

    uint32_t command_fields = desc->slave_mask | desc->address_mask |
                              desc->data_mask;
    uint32_t command_controls = desc->command_start_mask |
                                desc->command_busy_mask |
                                desc->command_write_mask;
    if ((command_fields & command_controls) != 0u ||
        (desc->slave_mask & (desc->address_mask | desc->data_mask)) != 0u ||
        (desc->address_mask & desc->data_mask) != 0u ||
        (desc->analog_control_writable_mask & desc->bbpll_done_mask) != 0u ||
        (desc->bbpll_stop_high_mask & desc->bbpll_stop_low_mask) != 0u ||
        ((desc->bbpll_stop_high_mask | desc->bbpll_stop_low_mask) &
         ~desc->analog_control_writable_mask) != 0u)
        return false;
    return true;
}

static int regi2c_host(const flexe_regi2c_t *regi2c, uint32_t off)
{
    const flexe_regi2c_desc_t *desc = &regi2c->target->regi2c;
    if (off < desc->command_offset) return -1;
    uint32_t relative = off - desc->command_offset;
    if (relative % desc->command_stride != 0u) return -1;
    uint32_t host = relative / desc->command_stride;
    return host < desc->host_count ? (int)host : -1;
}

static int regi2c_aux_register(const flexe_regi2c_t *regi2c, uint32_t off)
{
    const flexe_regi2c_desc_t *desc = &regi2c->target->regi2c;
    for (unsigned index = 0u; index < desc->aux_register_count; index++)
        if (desc->aux_register[index].offset == off) return (int)index;
    return -1;
}

static size_t regi2c_index(const flexe_regi2c_t *regi2c,
                           uint32_t command)
{
    const flexe_regi2c_desc_t *desc = &regi2c->target->regi2c;
    size_t slave = (command & desc->slave_mask) >> desc->slave_shift;
    size_t address = (command & desc->address_mask) >> desc->address_shift;
    return slave * regi2c->address_count + address;
}

static uint32_t regi2c_read(void *ctx, uint32_t addr)
{
    flexe_regi2c_t *regi2c = ctx;
    const flexe_regi2c_desc_t *desc = &regi2c->target->regi2c;
    uint32_t off = addr - desc->base;
    int host = regi2c_host(regi2c, off);
    if (host >= 0) return regi2c->command[host];

    if (off == desc->analog_control_offset) {
        if (regi2c->bbpll_calibrating && ++regi2c->bbpll_reads >= 2u) {
            regi2c->bbpll_calibrating = false;
            regi2c->bbpll_done = true;
        }
        return regi2c->analog_control |
               (regi2c->bbpll_done ? desc->bbpll_done_mask : 0u);
    }
    if (off == desc->config_offset) return regi2c->config;
    if (off == desc->config2_offset) return regi2c->config2;
    int aux = regi2c_aux_register(regi2c, off);
    if (aux >= 0) return regi2c->aux_register[aux];
    return regi2c->fallback_read
        ? regi2c->fallback_read(regi2c->fallback_ctx, addr) : 0u;
}

static void regi2c_write_command(flexe_regi2c_t *regi2c, unsigned host,
                                  uint32_t value)
{
    const flexe_regi2c_desc_t *desc = &regi2c->target->regi2c;
    uint32_t known = desc->command_start_mask | desc->command_write_mask |
                     desc->slave_mask | desc->address_mask | desc->data_mask;
    uint32_t command = value & known;
    if (command & desc->command_start_mask) {
        size_t index = regi2c_index(regi2c, command);
        if (command & desc->command_write_mask) {
            regi2c->registers[index] =
                (uint8_t)((command & desc->data_mask) >> desc->data_shift);
        } else {
            command &= ~desc->data_mask;
            command |= ((uint32_t)regi2c->registers[index] <<
                        desc->data_shift) & desc->data_mask;
        }
    }
    /* Fast mode completes synchronously. A future timed implementation can
     * expose command_busy_mask until its scheduled bus event completes. */
    regi2c->command[host] = command & ~desc->command_busy_mask;
}

static void regi2c_write(void *ctx, uint32_t addr, uint32_t value)
{
    flexe_regi2c_t *regi2c = ctx;
    const flexe_regi2c_desc_t *desc = &regi2c->target->regi2c;
    uint32_t off = addr - desc->base;
    int host = regi2c_host(regi2c, off);
    if (host >= 0) {
        regi2c_write_command(regi2c, (unsigned)host, value);
        return;
    }

    if (off == desc->analog_control_offset) {
        uint32_t old = regi2c->analog_control;
        regi2c->analog_control = value &
                                  desc->analog_control_writable_mask;
        bool start = (regi2c->analog_control &
                      desc->bbpll_stop_low_mask) != 0u &&
                     (regi2c->analog_control &
                      desc->bbpll_stop_high_mask) == 0u;
        bool rising = (old & desc->bbpll_stop_low_mask) == 0u;
        if (start && rising) {
            regi2c->bbpll_reads = 0u;
            regi2c->bbpll_calibrating = true;
            regi2c->bbpll_done = false;
        } else if (regi2c->analog_control &
                   desc->bbpll_stop_high_mask) {
            regi2c->bbpll_calibrating = false;
            regi2c->bbpll_done = false;
        }
        return;
    }
    if (off == desc->config_offset) {
        regi2c->config = value & desc->config_writable_mask;
        return;
    }
    if (off == desc->config2_offset) {
        regi2c->config2 = value & desc->config2_writable_mask;
        return;
    }
    int aux = regi2c_aux_register(regi2c, off);
    if (aux >= 0) {
        uint32_t writable = desc->aux_register[aux].writable_mask;
        regi2c->aux_register[aux] =
            (regi2c->aux_register[aux] & ~writable) |
            (value & writable);
        return;
    }
    if (regi2c->fallback_write)
        regi2c->fallback_write(regi2c->fallback_ctx, addr, value);
}

flexe_regi2c_t *flexe_regi2c_create(xtensa_mem_t *mem,
                                    mmio_read_fn fallback_read,
                                    mmio_write_fn fallback_write,
                                    void *fallback_ctx)
{
    if (!mem) return NULL;
    const flexe_target_desc_t *target = mem_target(mem);
    if (!regi2c_geometry_valid(target)) return NULL;
    const flexe_regi2c_desc_t *desc = &target->regi2c;

    flexe_regi2c_t *regi2c = calloc(1, sizeof(*regi2c));
    if (!regi2c) return NULL;
    regi2c->mem = mem;
    regi2c->target = target;
    regi2c->fallback_read = fallback_read;
    regi2c->fallback_write = fallback_write;
    regi2c->fallback_ctx = fallback_ctx;
    regi2c->address_count =
        ((desc->address_mask >> desc->address_shift) + 1u);
    size_t slave_count =
        ((desc->slave_mask >> desc->slave_shift) + 1u);
    regi2c->registers = calloc(slave_count, regi2c->address_count);
    if (!regi2c->registers) {
        free(regi2c);
        return NULL;
    }
    regi2c->analog_control = desc->analog_control_reset;
    regi2c->config = desc->config_reset;
    regi2c->config2 = desc->config2_reset;
    for (unsigned index = 0u; index < desc->aux_register_count; index++)
        regi2c->aux_register[index] = desc->aux_register[index].reset;

    if (mem_register_mmio_range(mem, desc->base, desc->register_size,
                                regi2c_read, regi2c_write, regi2c) != 0) {
        free(regi2c->registers);
        free(regi2c);
        return NULL;
    }
    return regi2c;
}

void flexe_regi2c_destroy(flexe_regi2c_t *regi2c)
{
    if (!regi2c) return;
    const flexe_regi2c_desc_t *desc = &regi2c->target->regi2c;
    (void)mem_register_mmio_range(regi2c->mem, desc->base,
                                  desc->register_size, NULL, NULL, NULL);
    free(regi2c->registers);
    free(regi2c);
}
