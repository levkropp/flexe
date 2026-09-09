/* Native helpers for the canonical ESP32 register-window save paths.
 *
 * These live outside xtensa.c deliberately.  The interpreter dispatch loop is
 * unusually sensitive to code placement on both Apple silicon and Rosetta;
 * adding cold helpers ahead of it can move the loop enough to erase the work
 * they save. */
#include "xtensa.h"
#include "memory.h"

#include <limits.h>
#include <string.h>

extern int g_flexe_shadow_fill;

static inline uint32_t phys_read_accel(const xtensa_cpu_t *cpu, int window,
                                       int reg) {
    return cpu->ar[((window * 4) + reg) & 63];
}

static int window_callsize_accel(const xtensa_cpu_t *cpu, int window) {
    int callsize = cpu->window_callsize[window & 15];
    if (callsize == 0)
        callsize = (int)(phys_read_accel(cpu, window, 0) >> 30) & 3;
    return callsize == 0 ? 2 : callsize;
}

static int find_callee_accel(const xtensa_cpu_t *cpu, int caller) {
    int window = (int)cpu->windowbase;
    for (int steps = 0; steps < 16; steps++) {
        int previous = (window - window_callsize_accel(cpu, window)) & 15;
        if (previous == caller) return window;
        window = previous;
        if (window == (int)cpu->windowbase) break;
    }
    for (int distance = 1; distance < 16; distance++) {
        int candidate = (caller + distance) & 15;
        if (cpu->windowstart & (1u << candidate)) return candidate;
    }
    return caller;
}

static void window_hazard_refresh_accel(xtensa_cpu_t *cpu) {
    if (!cpu->real_window_vectors) {
        cpu->window_hazard = 0u;
        return;
    }
    uint32_t windows = cpu->windowstart & 0xFFFFu;
    unsigned shift = (cpu->windowbase + 1u) & 15u;
    cpu->window_hazard =
        (uint8_t)(((windows | (windows << 16)) >> shift) & 7u);
}

static bool word_mapped(xtensa_mem_t *mem, uint32_t addr, bool write) {
    if (addr > UINT32_MAX - 3u) return false;
    if (write) {
        return mem_get_ptr_w(mem, addr) != NULL &&
               mem_get_ptr_w(mem, addr + 3u) != NULL;
    }
    return mem_get_ptr(mem, addr) != NULL &&
           mem_get_ptr(mem, addr + 3u) != NULL;
}

static uint8_t *ram_range(xtensa_mem_t *mem, uint32_t addr,
                          unsigned bytes) {
    if (bytes == 0u || addr > UINT32_MAX - (bytes - 1u) ||
        (addr >> 12) != ((addr + bytes - 1u) >> 12))
        return NULL;
    return mem_get_ptr_w(mem, addr);
}

/* Native equivalent of the six canonical WindowOverflow/WindowUnderflow
 * vectors shipped in classic ESP32 IDF images.  This deliberately models the
 * vector instruction order, including the temporary a0 link-register write,
 * rather than calling the legacy synthetic spill machinery. */
bool xtensa_fast_window_vector(xtensa_cpu_t *cpu, unsigned register_count,
                               bool underflow) {
    if (!cpu || !cpu->mem || !cpu->real_window_vectors ||
        !XT_PS_EXCM(cpu->ps) ||
        (register_count != 4u && register_count != 8u &&
         register_count != 12u))
        return false;

    const unsigned base_reg = register_count + 1u;
    const uint32_t frame_top = ar_read(cpu, (int)base_reg);
    if (frame_top < 16u || (!underflow && g_mem_write32_observe))
        return false;

    /* Save areas are contiguous and normally remain within one RAM page.
     * Resolving each area once is materially cheaper than one page-table
     * lookup per vector instruction.  Cross-page and observed writes retain
     * the exact guest implementation. */
    uint8_t *core_host = ram_range(cpu->mem, frame_top - 16u, 16u);
    if (!core_host) return false;

    uint32_t core[4];
    uint32_t extra[8];
    uint8_t *extra_host = NULL;
    uint32_t extra_top = 0u;
    if (underflow) {
        for (unsigned i = 0; i < 4u; i++)
            memcpy(&core[i], core_host + i * 4u, sizeof(core[i]));
        if (register_count > 4u) {
            if (core[1] < 12u) return false;
            uint8_t *link = ram_range(cpu->mem, core[1] - 12u, 4u);
            if (!link) return false;
            memcpy(&extra_top, link, sizeof(extra_top));
        }
    } else {
        for (unsigned i = 0; i < 4u; i++)
            core[i] = ar_read(cpu, (int)i);
        if (register_count > 4u) {
            uint32_t stack = ar_read(cpu, 1);
            if (stack < 12u) return false;
            uint8_t *link = ram_range(cpu->mem, stack - 12u, 4u);
            if (!link) return false;
            memcpy(&extra_top, link, sizeof(extra_top));
        }
    }

    if (register_count > 4u) {
        unsigned extra_count = register_count - 4u;
        uint32_t extra_offset = register_count == 8u ? 32u : 48u;
        if (extra_top < extra_offset) return false;
        extra_host = ram_range(cpu->mem, extra_top - extra_offset,
                               extra_count * 4u);
        if (!extra_host) return false;
        for (unsigned i = 0; i < extra_count; i++) {
            if (underflow)
                memcpy(&extra[i], extra_host + i * 4u, sizeof(extra[i]));
            else
                extra[i] = ar_read(cpu, (int)i + 4);
        }
    }

    if (underflow) {
        for (unsigned i = 0; i < 4u; i++)
            ar_write(cpu, (int)i, core[i]);
        for (unsigned i = 0; i < register_count - 4u; i++)
            ar_write(cpu, (int)i + 4, extra[i]);
    } else {
        memcpy(core_host, &core[0], sizeof(core[0]));
        if (register_count > 4u)
            ar_write(cpu, 0, extra_top);
        for (unsigned i = 1; i < 4u; i++)
            memcpy(core_host + i * 4u, &core[i], sizeof(core[i]));
        for (unsigned i = 0; i < register_count - 4u; i++)
            memcpy(extra_host + i * 4u, &extra[i], sizeof(extra[i]));
    }

    unsigned vector_window = cpu->windowbase & 15u;
    XT_PS_SET_EXCM(cpu->ps, 0);
    cpu->irq_check = true;
    if (underflow)
        cpu->windowstart |= 1u << vector_window;
    else
        cpu->windowstart &= ~(1u << vector_window);
    cpu->windowbase = XT_PS_OWB(cpu->ps);
    window_hazard_refresh_accel(cpu);
    cpu->pc = cpu->epc[0];
    cpu->_pc_written = true;
    return true;
}

bool xtensa_fast_spill_all_windows(xtensa_cpu_t *cpu) {
    if (!cpu || !cpu->mem || !cpu->real_window_vectors ||
        g_flexe_shadow_fill || cpu->spill_verify || cpu->window_trace ||
        cpu->vecbase < 0x40070000u || cpu->vecbase >= 0x40400000u ||
        !mem_get_ptr(cpu->mem,
                     cpu->vecbase + VECOFS_WINDOW_UNDERFLOW4))
        return false;

    /* Validate every save-area link before the first store.  Malformed task
     * stacks continue in guest code with its original fault behavior. */
    for (unsigned distance = 1u; distance < 16u; distance++) {
        int window = ((int)cpu->windowbase + (int)distance) & 15;
        if ((cpu->windowstart & (1u << window)) == 0u) continue;
        int callee = find_callee_accel(cpu, window);
        int callsize = window_callsize_accel(cpu, callee);
        uint32_t top = phys_read_accel(cpu, callee, 1);
        if (top < 16u) return false;
        for (unsigned i = 0; i < 4u; i++)
            if (!word_mapped(cpu->mem, top - 16u + i * 4u, true))
                return false;
        if (callsize >= 2) {
            uint32_t stack = phys_read_accel(cpu, window, 1);
            if (stack < 12u || !word_mapped(cpu->mem, stack - 12u, false))
                return false;
            uint32_t extra_top = mem_read32(cpu->mem, stack - 12u);
            uint32_t offset = callsize == 3 ? 48u : 32u;
            unsigned count = callsize == 3 ? 8u : 4u;
            if (extra_top < offset) return false;
            for (unsigned i = 0; i < count; i++)
                if (!word_mapped(cpu->mem,
                                 extra_top - offset + i * 4u, true))
                    return false;
        }
    }

    uint32_t saved_spill_base[16];
    memcpy(saved_spill_base, cpu->spill_base, sizeof(saved_spill_base));
    xtensa_flush_windows(cpu);
    memcpy(cpu->spill_base, saved_spill_base, sizeof(saved_spill_base));
    return true;
}
