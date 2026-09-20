#include "syscon_memory.h"
#include "target.h"

#include <stdlib.h>

struct flexe_syscon_memory {
    xtensa_mem_t *mem;
    const flexe_target_desc_t *target;
    mmio_read_fn fallback_read;
    mmio_write_fn fallback_write;
    void *fallback_ctx;
    uint32_t front_end_power;
    uint32_t clock_force_on;
    uint32_t power_down;
    uint32_t power_up;
    flexe_syscon_memory_state_fn state_changed;
    void *state_ctx;
};

static unsigned mask_bit_count(uint32_t mask)
{
    unsigned count = 0u;
    while (mask != 0u) {
        count += mask & 1u;
        mask >>= 1u;
    }
    return count;
}

static uint32_t pack_masked_bits(uint32_t value, uint32_t mask)
{
    uint32_t packed = 0u;
    unsigned output = 0u;
    for (unsigned input = 0u; input < 32u; input++) {
        uint32_t bit = 1u << input;
        if ((mask & bit) == 0u) continue;
        if (value & bit) packed |= 1u << output;
        output++;
    }
    return packed;
}

static bool register_offset_valid(uint16_t offset, uint32_t size)
{
    return (offset & 3u) == 0u && size >= sizeof(uint32_t) &&
           offset <= size - sizeof(uint32_t);
}

static bool geometry_valid(const flexe_target_desc_t *target)
{
    if (!target || !(target->capabilities &
                     FLEXE_TARGET_CAP_SYSCON_MEMORY_V1))
        return false;
    const flexe_syscon_memory_desc_t *desc = &target->syscon_memory;
    const uint16_t offsets[] = {
        desc->front_end_power_offset,
        desc->clock_force_on_offset,
        desc->power_down_offset,
        desc->power_up_offset,
    };
    uint32_t front_mask = desc->front_end_force_power_down_mask |
                          desc->front_end_force_power_up_mask;
    uint32_t bank_mask = desc->sram_bank_mask | desc->rom_bank_mask;
    if (((desc->base | desc->register_size) & 0xFFFu) != 0u ||
        desc->base < target->peripheral_start ||
        desc->base >= target->peripheral_end ||
        desc->register_size > target->peripheral_end - desc->base ||
        desc->front_end_force_power_down_mask == 0u ||
        desc->front_end_force_power_up_mask == 0u ||
        (desc->front_end_force_power_down_mask &
         desc->front_end_force_power_up_mask) != 0u ||
        desc->sram_bank_mask == 0u || desc->rom_bank_mask == 0u ||
        (desc->sram_bank_mask & desc->rom_bank_mask) != 0u ||
        mask_bit_count(desc->front_end_force_power_down_mask) > 8u ||
        mask_bit_count(desc->front_end_force_power_up_mask) > 8u ||
        mask_bit_count(desc->sram_bank_mask) > 16u ||
        mask_bit_count(desc->rom_bank_mask) > 8u ||
        (desc->front_end_power_reset & ~front_mask) != 0u ||
        (desc->clock_force_on_reset & ~bank_mask) != 0u ||
        (desc->power_down_reset & ~bank_mask) != 0u ||
        (desc->power_up_reset & ~bank_mask) != 0u)
        return false;
    for (unsigned i = 0u; i < sizeof(offsets) / sizeof(offsets[0]); i++) {
        if (!register_offset_valid(offsets[i], desc->register_size))
            return false;
        for (unsigned j = 0u; j < i; j++)
            if (offsets[i] == offsets[j]) return false;
    }
    return true;
}

static bool read_state(const flexe_syscon_memory_t *memory,
                       flexe_syscon_memory_state_t *state)
{
    if (!memory || !state) return false;
    const flexe_syscon_memory_desc_t *desc =
        &memory->target->syscon_memory;
    *state = (flexe_syscon_memory_state_t) {
        .front_end_force_power_down = (uint8_t)pack_masked_bits(
            memory->front_end_power,
            desc->front_end_force_power_down_mask),
        .front_end_force_power_up = (uint8_t)pack_masked_bits(
            memory->front_end_power,
            desc->front_end_force_power_up_mask),
        .sram_clock_force_on = (uint16_t)pack_masked_bits(
            memory->clock_force_on, desc->sram_bank_mask),
        .rom_clock_force_on = (uint8_t)pack_masked_bits(
            memory->clock_force_on, desc->rom_bank_mask),
        .sram_force_power_down = (uint16_t)pack_masked_bits(
            memory->power_down, desc->sram_bank_mask),
        .rom_force_power_down = (uint8_t)pack_masked_bits(
            memory->power_down, desc->rom_bank_mask),
        .sram_force_power_up = (uint16_t)pack_masked_bits(
            memory->power_up, desc->sram_bank_mask),
        .rom_force_power_up = (uint8_t)pack_masked_bits(
            memory->power_up, desc->rom_bank_mask),
    };
    return true;
}

static void publish_state(flexe_syscon_memory_t *memory)
{
    if (!memory || !memory->state_changed) return;
    flexe_syscon_memory_state_t state;
    if (read_state(memory, &state))
        memory->state_changed(memory->state_ctx, &state);
}

static uint32_t *register_for_offset(
    flexe_syscon_memory_t *memory, uint32_t offset, uint32_t *mask)
{
    const flexe_syscon_memory_desc_t *desc =
        &memory->target->syscon_memory;
    if (offset == desc->front_end_power_offset) {
        *mask = desc->front_end_force_power_down_mask |
                desc->front_end_force_power_up_mask;
        return &memory->front_end_power;
    }
    if (offset == desc->clock_force_on_offset) {
        *mask = desc->sram_bank_mask | desc->rom_bank_mask;
        return &memory->clock_force_on;
    }
    if (offset == desc->power_down_offset) {
        *mask = desc->sram_bank_mask | desc->rom_bank_mask;
        return &memory->power_down;
    }
    if (offset == desc->power_up_offset) {
        *mask = desc->sram_bank_mask | desc->rom_bank_mask;
        return &memory->power_up;
    }
    return NULL;
}

uint32_t flexe_syscon_memory_mmio_read(void *ctx, uint32_t address)
{
    flexe_syscon_memory_t *memory = ctx;
    if (!memory) return 0u;
    const flexe_syscon_memory_desc_t *desc =
        &memory->target->syscon_memory;
    uint32_t mask = 0u;
    uint32_t *reg = address >= desc->base ?
        register_for_offset(memory, address - desc->base, &mask) : NULL;
    if (reg) return *reg;
    return memory->fallback_read ?
        memory->fallback_read(memory->fallback_ctx, address) : 0u;
}

void flexe_syscon_memory_mmio_write(
    void *ctx, uint32_t address, uint32_t value)
{
    flexe_syscon_memory_t *memory = ctx;
    if (!memory) return;
    const flexe_syscon_memory_desc_t *desc =
        &memory->target->syscon_memory;
    uint32_t mask = 0u;
    uint32_t *reg = address >= desc->base ?
        register_for_offset(memory, address - desc->base, &mask) : NULL;
    if (!reg) {
        if (memory->fallback_write)
            memory->fallback_write(memory->fallback_ctx, address, value);
        return;
    }
    uint32_t next = (*reg & ~mask) | (value & mask);
    if (next == *reg) return;
    *reg = next;
    publish_state(memory);
}

flexe_syscon_memory_t *flexe_syscon_memory_create(
    xtensa_mem_t *mem, mmio_read_fn fallback_read,
    mmio_write_fn fallback_write, void *fallback_ctx)
{
    if (!mem) return NULL;
    const flexe_target_desc_t *target = mem_target(mem);
    if (!geometry_valid(target)) return NULL;
    flexe_syscon_memory_t *memory = calloc(1u, sizeof(*memory));
    if (!memory) return NULL;
    memory->mem = mem;
    memory->target = target;
    memory->fallback_read = fallback_read;
    memory->fallback_write = fallback_write;
    memory->fallback_ctx = fallback_ctx;
    const flexe_syscon_memory_desc_t *desc = &target->syscon_memory;
    memory->front_end_power = desc->front_end_power_reset;
    memory->clock_force_on = desc->clock_force_on_reset;
    memory->power_down = desc->power_down_reset;
    memory->power_up = desc->power_up_reset;
    if (mem_register_mmio_range(mem, desc->base, desc->register_size,
                                flexe_syscon_memory_mmio_read,
                                flexe_syscon_memory_mmio_write,
                                memory) != 0) {
        free(memory);
        return NULL;
    }
    return memory;
}

void flexe_syscon_memory_destroy(flexe_syscon_memory_t *memory)
{
    if (!memory) return;
    const flexe_syscon_memory_desc_t *desc =
        &memory->target->syscon_memory;
    (void)mem_register_mmio_range(
        memory->mem, desc->base, desc->register_size,
        memory->fallback_read, memory->fallback_write,
        memory->fallback_ctx);
    free(memory);
}

bool flexe_syscon_memory_state(
    const flexe_syscon_memory_t *memory,
    flexe_syscon_memory_state_t *state)
{
    return read_state(memory, state);
}

void flexe_syscon_memory_set_state_listener(
    flexe_syscon_memory_t *memory, flexe_syscon_memory_state_fn fn,
    void *ctx)
{
    if (!memory) return;
    memory->state_changed = fn;
    memory->state_ctx = fn ? ctx : NULL;
    publish_state(memory);
}
