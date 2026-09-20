#include "syscon_memory.h"
#include "target.h"

#include <stdlib.h>

typedef struct {
    uint32_t attribute[FLEXE_TARGET_SYSCON_ACE_REGION_MAX];
    uint32_t address[FLEXE_TARGET_SYSCON_ACE_REGION_MAX];
    uint32_t size[FLEXE_TARGET_SYSCON_ACE_REGION_MAX];
} flexe_syscon_ace_bank_t;

struct flexe_syscon_memory {
    xtensa_mem_t *mem;
    const flexe_target_desc_t *target;
    mmio_read_fn fallback_read;
    mmio_write_fn fallback_write;
    void *fallback_ctx;
    flexe_syscon_ace_bank_t flash_ace;
    flexe_syscon_ace_bank_t sram_ace;
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

static bool register_offset_valid(uint32_t offset, uint32_t size)
{
    return (offset & 3u) == 0u && size >= sizeof(uint32_t) &&
           offset <= size - sizeof(uint32_t);
}

static bool add_register_offset(uint16_t *offsets, unsigned *count,
                                uint32_t offset, uint32_t size)
{
    if (!offsets || !count || offset > UINT16_MAX ||
        !register_offset_valid(offset, size))
        return false;
    for (unsigned i = 0u; i < *count; i++)
        if (offsets[i] == offset) return false;
    offsets[(*count)++] = (uint16_t)offset;
    return true;
}

static bool ace_desc_valid(const flexe_syscon_ace_desc_t *ace,
                           unsigned region_count)
{
    if (!ace || region_count == 0u ||
        ace->attribute_writable_mask == 0u ||
        ace->address_writable_mask == 0u ||
        ace->size_writable_mask == 0u ||
        (ace->attribute_reset & ~ace->attribute_writable_mask) != 0u ||
        (ace->size_reset & ~ace->size_writable_mask) != 0u)
        return false;
    for (unsigned i = 0u; i < region_count; i++)
        if ((ace->address_reset[i] &
             ~ace->address_writable_mask) != 0u)
            return false;
    return true;
}

static bool geometry_valid(const flexe_target_desc_t *target)
{
    if (!target || !(target->capabilities &
                     FLEXE_TARGET_CAP_SYSCON_MEMORY_V1))
        return false;
    const flexe_syscon_memory_desc_t *desc = &target->syscon_memory;
    uint32_t front_mask = desc->front_end_force_power_down_mask |
                          desc->front_end_force_power_up_mask;
    uint32_t bank_mask = desc->sram_bank_mask | desc->rom_bank_mask;
    if (((desc->base | desc->register_size) & 0xFFFu) != 0u ||
        desc->base < target->peripheral_start ||
        desc->base >= target->peripheral_end ||
        desc->register_size > target->peripheral_end - desc->base ||
        desc->ace_region_count == 0u ||
        desc->ace_region_count > FLEXE_TARGET_SYSCON_ACE_REGION_MAX ||
        desc->ace_region_stride == 0u ||
        (desc->ace_region_stride & 3u) != 0u ||
        !ace_desc_valid(&desc->flash_ace, desc->ace_region_count) ||
        !ace_desc_valid(&desc->sram_ace, desc->ace_region_count) ||
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

    uint16_t offsets[4u + 6u * FLEXE_TARGET_SYSCON_ACE_REGION_MAX];
    unsigned offset_count = 0u;
    if (!add_register_offset(offsets, &offset_count,
                             desc->front_end_power_offset,
                             desc->register_size) ||
        !add_register_offset(offsets, &offset_count,
                             desc->clock_force_on_offset,
                             desc->register_size) ||
        !add_register_offset(offsets, &offset_count,
                             desc->power_down_offset,
                             desc->register_size) ||
        !add_register_offset(offsets, &offset_count,
                             desc->power_up_offset,
                             desc->register_size))
        return false;
    const flexe_syscon_ace_desc_t *ace[] = {
        &desc->flash_ace, &desc->sram_ace,
    };
    for (unsigned bank = 0u; bank < 2u; bank++) {
        for (unsigned region = 0u; region < desc->ace_region_count;
             region++) {
            uint32_t delta = region * desc->ace_region_stride;
            if (!add_register_offset(offsets, &offset_count,
                                     ace[bank]->attribute_offset + delta,
                                     desc->register_size) ||
                !add_register_offset(offsets, &offset_count,
                                     ace[bank]->address_offset + delta,
                                     desc->register_size) ||
                !add_register_offset(offsets, &offset_count,
                                     ace[bank]->size_offset + delta,
                                     desc->register_size))
                return false;
        }
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
        .ace_region_count = desc->ace_region_count,
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
    for (unsigned i = 0u; i < desc->ace_region_count; i++) {
        state->flash_ace[i] = (flexe_syscon_ace_region_state_t) {
            .attributes = memory->flash_ace.attribute[i],
            .address = memory->flash_ace.address[i],
            .size_pages = memory->flash_ace.size[i],
        };
        state->sram_ace[i] = (flexe_syscon_ace_region_state_t) {
            .attributes = memory->sram_ace.attribute[i],
            .address = memory->sram_ace.address[i],
            .size_pages = memory->sram_ace.size[i],
        };
    }
    return true;
}

static void publish_state(flexe_syscon_memory_t *memory)
{
    if (!memory || !memory->state_changed) return;
    flexe_syscon_memory_state_t state;
    if (read_state(memory, &state))
        memory->state_changed(memory->state_ctx, &state);
}

static uint32_t *ace_register_for_offset(
    flexe_syscon_memory_t *memory, const flexe_syscon_ace_desc_t *desc,
    flexe_syscon_ace_bank_t *bank, uint32_t offset, uint32_t *mask)
{
    const flexe_syscon_memory_desc_t *memory_desc =
        &memory->target->syscon_memory;
    for (unsigned i = 0u; i < memory_desc->ace_region_count; i++) {
        uint32_t delta = i * memory_desc->ace_region_stride;
        if (offset == desc->attribute_offset + delta) {
            *mask = desc->attribute_writable_mask;
            return &bank->attribute[i];
        }
        if (offset == desc->address_offset + delta) {
            *mask = desc->address_writable_mask;
            return &bank->address[i];
        }
        if (offset == desc->size_offset + delta) {
            *mask = desc->size_writable_mask;
            return &bank->size[i];
        }
    }
    return NULL;
}

static uint32_t *register_for_offset(
    flexe_syscon_memory_t *memory, uint32_t offset, uint32_t *mask)
{
    const flexe_syscon_memory_desc_t *desc =
        &memory->target->syscon_memory;
    uint32_t *reg = ace_register_for_offset(
        memory, &desc->flash_ace, &memory->flash_ace, offset, mask);
    if (reg) return reg;
    reg = ace_register_for_offset(
        memory, &desc->sram_ace, &memory->sram_ace, offset, mask);
    if (reg) return reg;
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
    for (unsigned i = 0u; i < desc->ace_region_count; i++) {
        memory->flash_ace.attribute[i] = desc->flash_ace.attribute_reset;
        memory->flash_ace.address[i] = desc->flash_ace.address_reset[i];
        memory->flash_ace.size[i] = desc->flash_ace.size_reset;
        memory->sram_ace.attribute[i] = desc->sram_ace.attribute_reset;
        memory->sram_ace.address[i] = desc->sram_ace.address_reset[i];
        memory->sram_ace.size[i] = desc->sram_ace.size_reset;
    }
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
