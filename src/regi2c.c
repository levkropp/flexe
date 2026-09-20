#include "regi2c.h"
#include "target.h"
#include "rtc_cntl.h"

#include <stdbool.h>
#include <stdlib.h>

struct flexe_regi2c {
    xtensa_mem_t *mem;
    const flexe_target_desc_t *target;
    mmio_read_fn fallback_read;
    mmio_write_fn fallback_write;
    void *fallback_ctx;
    const flexe_rtc_cntl_t *rtc_cntl;
    uint32_t command[FLEXE_TARGET_REGI2C_HOST_MAX];
    uint32_t analog_control;
    uint32_t config;
    uint32_t config2;
    uint32_t aux_register[FLEXE_TARGET_REGI2C_AUX_REGISTER_MAX];
    uint8_t *registers;
    uint32_t *private_registers;
    uint32_t *indexed_words;
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

static bool regi2c_private_offset_valid(
    const flexe_regi2c_desc_t *desc, uint32_t offset)
{
    return desc->private_register_size >= sizeof(uint32_t) &&
           (offset & 3u) == 0u &&
           offset >= desc->private_register_offset &&
           offset - desc->private_register_offset <=
               desc->private_register_size - sizeof(uint32_t);
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
        desc->config2_offset > desc->register_size - 4u ||
        (desc->private_register_size != 0u &&
         (desc->private_register_size < sizeof(uint32_t) ||
          ((desc->private_register_offset |
            desc->private_register_size) & 3u) != 0u ||
          desc->private_register_offset > desc->register_size ||
          desc->private_register_size >
              desc->register_size - desc->private_register_offset)))
        return false;

    if (desc->private_register_size != 0u) {
        if (regi2c_private_offset_valid(desc,
                                        desc->analog_control_offset) ||
            regi2c_private_offset_valid(desc, desc->config_offset) ||
            regi2c_private_offset_valid(desc, desc->config2_offset))
            return false;
        for (unsigned host = 0u; host < desc->host_count; host++)
            if (regi2c_private_offset_valid(
                    desc, desc->command_offset +
                          host * desc->command_stride))
                return false;
    }

    for (unsigned index = 0u; index < desc->aux_register_count; index++) {
        uint32_t offset = desc->aux_register[index].offset;
        if ((offset & 3u) != 0u ||
            offset > desc->register_size - sizeof(uint32_t) ||
            offset == desc->analog_control_offset ||
            offset == desc->config_offset ||
            offset == desc->config2_offset ||
            regi2c_private_offset_valid(desc, offset))
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

    const flexe_regi2c_result_bank_desc_t *results =
        &desc->result_bank;
    if (results->count != 0u) {
        uint64_t last = (uint64_t)results->offset +
            (uint64_t)(results->count - 1u) * results->stride;
        if (results->stride == 0u || (results->stride & 3u) != 0u ||
            results->value_mask == 0u || last > UINT32_MAX ||
            !regi2c_private_offset_valid(desc, results->offset) ||
            !regi2c_private_offset_valid(desc, (uint32_t)last))
            return false;
    }

    const flexe_regi2c_indexed_memory_desc_t *indexed =
        &desc->indexed_memory;
    if (indexed->word_count != 0u) {
        uint32_t offsets[] = {
            indexed->control_offset,
            indexed->read_data_offset,
            indexed->write_data_offset,
            indexed->status_offset,
            indexed->result_offset,
        };
        for (unsigned i = 0u;
             i < sizeof(offsets) / sizeof(offsets[0]); i++) {
            if (!regi2c_private_offset_valid(desc, offsets[i]))
                return false;
            for (unsigned j = 0u; j < i; j++)
                if (offsets[i] == offsets[j]) return false;
        }
        if (!byte_field_valid(indexed->index_mask,
                              indexed->index_shift) ||
            !byte_field_valid(indexed->result_index_mask,
                              indexed->result_index_shift) ||
            indexed->word_count !=
                (indexed->index_mask >> indexed->index_shift) + 1u ||
            !one_bit(indexed->write_trigger_mask) ||
            !one_bit(indexed->operation_trigger_mask) ||
            !one_bit(indexed->busy_mask) ||
            (indexed->index_mask &
             (indexed->write_trigger_mask |
              indexed->operation_trigger_mask)) != 0u ||
            (indexed->write_trigger_mask &
             indexed->operation_trigger_mask) != 0u ||
            indexed->index_to_result_shift >= 8u ||
            (((indexed->word_count - 1u) >>
              indexed->index_to_result_shift) >
             (indexed->result_index_mask >>
              indexed->result_index_shift)))
            return false;

        for (unsigned i = 0u; i < results->count; i++) {
            uint32_t offset = results->offset + i * results->stride;
            for (unsigned j = 0u;
                 j < sizeof(offsets) / sizeof(offsets[0]); j++)
                if (offset == offsets[j]) return false;
        }
    }
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

static uint32_t *regi2c_private_word(flexe_regi2c_t *regi2c,
                                     uint32_t offset)
{
    const flexe_regi2c_desc_t *desc = &regi2c->target->regi2c;
    if (!regi2c->private_registers ||
        !regi2c_private_offset_valid(desc, offset))
        return NULL;
    return &regi2c->private_registers[
        (offset - desc->private_register_offset) / sizeof(uint32_t)];
}

static bool regi2c_result_word(const flexe_regi2c_t *regi2c,
                               uint32_t offset)
{
    const flexe_regi2c_result_bank_desc_t *results =
        &regi2c->target->regi2c.result_bank;
    if (results->count == 0u || offset < results->offset)
        return false;
    uint32_t relative = offset - results->offset;
    return relative % results->stride == 0u &&
           relative / results->stride < results->count;
}

static void regi2c_write_indexed_control(flexe_regi2c_t *regi2c,
                                         uint32_t value)
{
    const flexe_regi2c_indexed_memory_desc_t *indexed =
        &regi2c->target->regi2c.indexed_memory;
    uint32_t *control = regi2c_private_word(
        regi2c, indexed->control_offset);
    uint32_t *read_data = regi2c_private_word(
        regi2c, indexed->read_data_offset);
    uint32_t *write_data = regi2c_private_word(
        regi2c, indexed->write_data_offset);
    uint32_t *status = regi2c_private_word(
        regi2c, indexed->status_offset);
    uint32_t *result = regi2c_private_word(
        regi2c, indexed->result_offset);
    size_t index =
        (value & indexed->index_mask) >> indexed->index_shift;

    *control = value;
    if (value & indexed->write_trigger_mask)
        regi2c->indexed_words[index] = *write_data;
    *read_data = regi2c->indexed_words[index];

    if (value & indexed->operation_trigger_mask) {
        uint32_t result_index =
            ((uint32_t)index >> indexed->index_to_result_shift) <<
            indexed->result_index_shift;
        *result = (*result & ~indexed->result_index_mask) |
                  (result_index & indexed->result_index_mask);
    }
    /* Functional mode completes the private operation synchronously. A
     * timed implementation can expose busy until its scheduled event. */
    *status &= ~indexed->busy_mask;
}

static size_t regi2c_index(const flexe_regi2c_t *regi2c,
                           uint32_t command)
{
    const flexe_regi2c_desc_t *desc = &regi2c->target->regi2c;
    size_t slave = (command & desc->slave_mask) >> desc->slave_shift;
    size_t address = (command & desc->address_mask) >> desc->address_shift;
    return slave * regi2c->address_count + address;
}

static bool regi2c_sar_unpowered(const flexe_regi2c_t *regi2c,
                                 unsigned slave)
{
    return regi2c->rtc_cntl &&
           (regi2c->target->capabilities & FLEXE_TARGET_CAP_SENS_V1) != 0u &&
           slave == regi2c->target->sens.adc_calibration_slave &&
           !flexe_rtc_cntl_sar_i2c_powered(regi2c->rtc_cntl);
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
    uint32_t *private_word = regi2c_private_word(regi2c, off);
    if (private_word) {
        if (regi2c_result_word(regi2c, off))
            return *private_word & desc->result_bank.value_mask;
        if (desc->indexed_memory.word_count != 0u &&
            off == desc->indexed_memory.status_offset)
            return *private_word & ~desc->indexed_memory.busy_mask;
        return *private_word;
    }
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
        unsigned slave = (command & desc->slave_mask) >> desc->slave_shift;
        if (regi2c_sar_unpowered(regi2c, slave)) {
            /* A powered-off analog slave cannot acknowledge or mutate its
             * register file. Keep the host busy and report the unsupported
             * transaction instead of fabricating a completed command. */
            regi2c->command[host] = command | desc->command_busy_mask;
            if (regi2c->fallback_write)
                regi2c->fallback_write(
                    regi2c->fallback_ctx,
                    desc->base + desc->command_offset +
                        host * desc->command_stride, value);
            return;
        }
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
    uint32_t *private_word = regi2c_private_word(regi2c, off);
    if (private_word) {
        const flexe_regi2c_indexed_memory_desc_t *indexed =
            &desc->indexed_memory;
        if (regi2c_result_word(regi2c, off))
            return;
        if (indexed->word_count != 0u) {
            if (off == indexed->control_offset) {
                regi2c_write_indexed_control(regi2c, value);
                return;
            }
            if (off == indexed->read_data_offset ||
                off == indexed->status_offset ||
                off == indexed->result_offset)
                return;
        }
        *private_word = value;
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
    if (!regi2c->registers) goto fail;
    if (desc->private_register_size != 0u) {
        regi2c->private_registers = calloc(
            desc->private_register_size / sizeof(uint32_t),
            sizeof(uint32_t));
        if (!regi2c->private_registers) goto fail;
    }
    if (desc->indexed_memory.word_count != 0u) {
        regi2c->indexed_words = calloc(
            desc->indexed_memory.word_count, sizeof(uint32_t));
        if (!regi2c->indexed_words) goto fail;
    }
    regi2c->analog_control = desc->analog_control_reset;
    regi2c->config = desc->config_reset;
    regi2c->config2 = desc->config2_reset;
    for (unsigned index = 0u; index < desc->aux_register_count; index++)
        regi2c->aux_register[index] = desc->aux_register[index].reset;

    if (mem_register_mmio_range(mem, desc->base, desc->register_size,
                                regi2c_read, regi2c_write, regi2c) != 0) {
        goto fail;
    }
    return regi2c;

fail:
    free(regi2c->indexed_words);
    free(regi2c->private_registers);
    free(regi2c->registers);
    free(regi2c);
    return NULL;
}

void flexe_regi2c_destroy(flexe_regi2c_t *regi2c)
{
    if (!regi2c) return;
    const flexe_regi2c_desc_t *desc = &regi2c->target->regi2c;
    (void)mem_register_mmio_range(regi2c->mem, desc->base,
                                  desc->register_size, NULL, NULL, NULL);
    free(regi2c->indexed_words);
    free(regi2c->private_registers);
    free(regi2c->registers);
    free(regi2c);
}

void flexe_regi2c_attach_rtc_cntl(flexe_regi2c_t *regi2c,
                                  const flexe_rtc_cntl_t *rtc_cntl)
{
    if (regi2c) regi2c->rtc_cntl = rtc_cntl;
}

bool flexe_regi2c_register_read(const flexe_regi2c_t *regi2c,
                                unsigned slave, unsigned address,
                                uint8_t *value)
{
    if (!regi2c || !value) return false;
    const flexe_regi2c_desc_t *desc = &regi2c->target->regi2c;
    if (slave > (desc->slave_mask >> desc->slave_shift) ||
        address >= regi2c->address_count)
        return false;
    if (regi2c_sar_unpowered(regi2c, slave)) return false;
    *value = regi2c->registers[slave * regi2c->address_count + address];
    return true;
}
