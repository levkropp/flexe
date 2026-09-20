#include "assist_debug.h"

#include "target.h"
#include "xtensa.h"

#include <stdbool.h>
#include <stdlib.h>

typedef struct {
    bool pdebug_enabled;
    bool recording;
    uint32_t pc;
    uint32_t sp;
} assist_debug_core_t;

struct flexe_assist_debug {
    xtensa_mem_t *mem;
    const flexe_target_desc_t *target;
    mmio_read_fn fallback_read;
    mmio_write_fn fallback_write;
    void *fallback_ctx;
    xtensa_cpu_t *cpu[FLEXE_TARGET_INTERRUPT_CORE_MAX];
    assist_debug_core_t core[FLEXE_TARGET_INTERRUPT_CORE_MAX];
    uint32_t date;
};

static bool word_fits(uint32_t offset, uint32_t size)
{
    return (offset & 3u) == 0u && offset <= size &&
           size - offset >= sizeof(uint32_t);
}

static bool assist_debug_geometry_valid(const flexe_target_desc_t *target)
{
    if (!target || !(target->capabilities & FLEXE_TARGET_CAP_ASSIST_DEBUG_V1))
        return false;
    const flexe_assist_debug_desc_t *desc = &target->assist_debug;
    if ((desc->base & 0xFFFu) != 0u || desc->register_size == 0u ||
        (desc->register_size & 3u) != 0u ||
        desc->core_count == 0u ||
        desc->core_count > FLEXE_TARGET_INTERRUPT_CORE_MAX ||
        desc->core_count > target->core_count ||
        desc->core_stride == 0u || (desc->core_stride & 3u) != 0u ||
        desc->pdebug_enable_mask == 0u ||
        (desc->pdebug_enable_mask & (desc->pdebug_enable_mask - 1u)) != 0u ||
        desc->recording_mask == 0u ||
        (desc->recording_mask & (desc->recording_mask - 1u)) != 0u ||
        (desc->date_reset & ~desc->date_writable_mask) != 0u)
        return false;

    const uint32_t offsets[] = {
        desc->pdebug_enable_offset,
        desc->recording_offset,
        desc->pc_offset,
        desc->sp_offset,
    };
    for (unsigned i = 0u; i < sizeof(offsets) / sizeof(offsets[0]); i++) {
        if (!word_fits(offsets[i], desc->core_stride)) return false;
        for (unsigned j = i + 1u;
             j < sizeof(offsets) / sizeof(offsets[0]); j++)
            if (offsets[i] == offsets[j]) return false;
    }

    uint64_t core_end = (uint64_t)(desc->core_count - 1u) *
                        desc->core_stride + desc->core_stride;
    if (core_end > desc->register_size ||
        !word_fits(desc->date_offset, desc->register_size) ||
        desc->date_offset < core_end ||
        desc->base < target->peripheral_start ||
        desc->base >= target->peripheral_end ||
        desc->register_size > target->peripheral_end - desc->base)
        return false;
    return true;
}

static bool core_active(const assist_debug_core_t *core)
{
    return core->pdebug_enabled && core->recording;
}

static void capture_core(flexe_assist_debug_t *debug, unsigned core)
{
    if (!debug || core >= debug->target->assist_debug.core_count ||
        !debug->cpu[core])
        return;
    debug->core[core].pc = debug->cpu[core]->pc;
    debug->core[core].sp = ar_read(debug->cpu[core], 1);
}

static bool decode_core_register(
    const flexe_assist_debug_t *debug, uint32_t off,
    unsigned *core_out, uint32_t *core_off_out)
{
    const flexe_assist_debug_desc_t *desc = &debug->target->assist_debug;
    if (off >= (uint32_t)desc->core_count * desc->core_stride)
        return false;
    unsigned core = off / desc->core_stride;
    uint32_t core_off = off % desc->core_stride;
    if (core_out) *core_out = core;
    if (core_off_out) *core_off_out = core_off;
    return true;
}

bool flexe_assist_debug_core_state(
    const flexe_assist_debug_t *debug, unsigned core,
    flexe_assist_debug_core_state_t *state)
{
    if (!debug || !state || core >= debug->target->assist_debug.core_count)
        return false;
    const assist_debug_core_t *record = &debug->core[core];
    *state = (flexe_assist_debug_core_state_t) {
        .pdebug_enabled = record->pdebug_enabled,
        .recording = record->recording,
        .pc = record->pc,
        .sp = record->sp,
    };
    if (core_active(record) && debug->cpu[core]) {
        state->pc = debug->cpu[core]->pc;
        state->sp = ar_read(debug->cpu[core], 1);
    }
    return true;
}

static uint32_t assist_debug_read(void *ctx, uint32_t addr)
{
    flexe_assist_debug_t *debug = ctx;
    const flexe_assist_debug_desc_t *desc = &debug->target->assist_debug;
    uint32_t off = addr - desc->base;
    if (off == desc->date_offset) return debug->date;

    unsigned core;
    uint32_t core_off;
    if (decode_core_register(debug, off, &core, &core_off)) {
        const assist_debug_core_t *record = &debug->core[core];
        if (core_off == desc->pdebug_enable_offset)
            return record->pdebug_enabled ? desc->pdebug_enable_mask : 0u;
        if (core_off == desc->recording_offset)
            return record->recording ? desc->recording_mask : 0u;
        if (core_off == desc->pc_offset || core_off == desc->sp_offset) {
            flexe_assist_debug_core_state_t state;
            (void)flexe_assist_debug_core_state(debug, core, &state);
            return core_off == desc->pc_offset ? state.pc : state.sp;
        }
    }
    return debug->fallback_read
        ? debug->fallback_read(debug->fallback_ctx, addr) : 0u;
}

static void set_control(flexe_assist_debug_t *debug, unsigned core,
                        bool pdebug, bool enabled)
{
    assist_debug_core_t *record = &debug->core[core];
    bool was_active = core_active(record);
    if (was_active) capture_core(debug, core);
    if (pdebug) record->pdebug_enabled = enabled;
    else record->recording = enabled;
    if (!was_active && core_active(record)) capture_core(debug, core);
}

static void assist_debug_write(void *ctx, uint32_t addr, uint32_t value)
{
    flexe_assist_debug_t *debug = ctx;
    const flexe_assist_debug_desc_t *desc = &debug->target->assist_debug;
    uint32_t off = addr - desc->base;
    if (off == desc->date_offset) {
        debug->date = (debug->date & ~desc->date_writable_mask) |
                      (value & desc->date_writable_mask);
        return;
    }

    unsigned core;
    uint32_t core_off;
    if (decode_core_register(debug, off, &core, &core_off)) {
        if (core_off == desc->pdebug_enable_offset) {
            set_control(debug, core, true,
                        (value & desc->pdebug_enable_mask) != 0u);
            return;
        }
        if (core_off == desc->recording_offset) {
            set_control(debug, core, false,
                        (value & desc->recording_mask) != 0u);
            return;
        }
        if (core_off == desc->pc_offset || core_off == desc->sp_offset)
            return; /* Hardware-owned record fields are read-only. */
    }
    if (debug->fallback_write)
        debug->fallback_write(debug->fallback_ctx, addr, value);
}

flexe_assist_debug_t *flexe_assist_debug_create(
    xtensa_mem_t *mem, mmio_read_fn fallback_read,
    mmio_write_fn fallback_write, void *fallback_ctx)
{
    if (!mem) return NULL;
    const flexe_target_desc_t *target = mem_target(mem);
    if (!assist_debug_geometry_valid(target)) return NULL;

    flexe_assist_debug_t *debug = calloc(1, sizeof(*debug));
    if (!debug) return NULL;
    debug->mem = mem;
    debug->target = target;
    debug->fallback_read = fallback_read;
    debug->fallback_write = fallback_write;
    debug->fallback_ctx = fallback_ctx;
    debug->date = target->assist_debug.date_reset;

    const flexe_assist_debug_desc_t *desc = &target->assist_debug;
    if (mem_register_mmio_range(mem, desc->base, desc->register_size,
                                assist_debug_read, assist_debug_write,
                                debug) != 0) {
        free(debug);
        return NULL;
    }
    return debug;
}

void flexe_assist_debug_destroy(flexe_assist_debug_t *debug)
{
    if (!debug) return;
    const flexe_assist_debug_desc_t *desc = &debug->target->assist_debug;
    (void)mem_register_mmio_range(
        debug->mem, desc->base, desc->register_size,
        debug->fallback_read, debug->fallback_write, debug->fallback_ctx);
    free(debug);
}

void flexe_assist_debug_attach_cpus(
    flexe_assist_debug_t *debug, xtensa_cpu_t *cpu0, xtensa_cpu_t *cpu1)
{
    if (!debug) return;
    debug->cpu[0] = cpu0;
    if (debug->target->assist_debug.core_count > 1u)
        debug->cpu[1] = cpu1;
    for (unsigned core = 0u;
         core < debug->target->assist_debug.core_count; core++)
        if (core_active(&debug->core[core])) capture_core(debug, core);
}
