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

/* These are the Xtensa windowed-ABI handlers shipped by classic ESP32
 * toolchains. They contain no link-specific addresses: each body consists
 * only of the architecturally defined S32E/L32E save-area accesses followed
 * by RFWO/RFWU. Recognizing the complete table therefore identifies an ABI
 * implementation, rather than a particular application or IDF link. */
static const uint8_t canonical_overflow4[] = {
    0x00, 0xC5, 0x49, 0x10, 0xD5, 0x49, 0x20, 0xE5, 0x49,
    0x30, 0xF5, 0x49, 0x00, 0x34, 0x00,
};
static const uint8_t canonical_underflow4[] = {
    0x00, 0xC5, 0x09, 0x10, 0xD5, 0x09, 0x20, 0xE5, 0x09,
    0x30, 0xF5, 0x09, 0x00, 0x35, 0x00,
};
static const uint8_t canonical_overflow8[] = {
    0x00, 0xC9, 0x49, 0x00, 0xD1, 0x09, 0x10, 0xD9, 0x49,
    0x20, 0xE9, 0x49, 0x30, 0xF9, 0x49, 0x40, 0x80, 0x49,
    0x50, 0x90, 0x49, 0x60, 0xA0, 0x49, 0x70, 0xB0, 0x49,
    0x00, 0x34, 0x00,
};
static const uint8_t canonical_underflow8[] = {
    0x00, 0xC9, 0x09, 0x10, 0xD9, 0x09, 0x20, 0xE9, 0x09,
    0x70, 0xD1, 0x09, 0x30, 0xF9, 0x09, 0x40, 0x87, 0x09,
    0x50, 0x97, 0x09, 0x60, 0xA7, 0x09, 0x70, 0xB7, 0x09,
    0x00, 0x35, 0x00,
};
static const uint8_t canonical_overflow12[] = {
    0x00, 0xCD, 0x49, 0x00, 0xD1, 0x09, 0x10, 0xDD, 0x49,
    0x20, 0xED, 0x49, 0x30, 0xFD, 0x49, 0x40, 0x40, 0x49,
    0x50, 0x50, 0x49, 0x60, 0x60, 0x49, 0x70, 0x70, 0x49,
    0x80, 0x80, 0x49, 0x90, 0x90, 0x49, 0xA0, 0xA0, 0x49,
    0xB0, 0xB0, 0x49, 0x00, 0x34, 0x00,
};
static const uint8_t canonical_underflow12[] = {
    0x00, 0xCD, 0x09, 0x10, 0xDD, 0x09, 0x20, 0xED, 0x09,
    0xB0, 0xD1, 0x09, 0x30, 0xFD, 0x09, 0x40, 0x4B, 0x09,
    0x50, 0x5B, 0x09, 0x60, 0x6B, 0x09, 0x70, 0x7B, 0x09,
    0x80, 0x8B, 0x09, 0x90, 0x9B, 0x09, 0xA0, 0xAB, 0x09,
    0xB0, 0xBB, 0x09, 0x00, 0x35, 0x00,
};

static bool canonical_code_matches(xtensa_mem_t *mem, uint32_t addr,
                                   const uint8_t *code, size_t size) {
    if (!mem || !code || size == 0u || addr > UINT32_MAX - (size - 1u))
        return false;
    for (size_t i = 0; i < size; i++) {
        if (mem_read8(mem, addr + (uint32_t)i) != code[i])
            return false;
    }
    return true;
}

bool xtensa_window_vectors_are_canonical(xtensa_mem_t *mem, uint32_t base) {
    /* VECBASE is 1 KiB aligned on LX6. Rejecting unaligned candidates also
     * keeps structural discovery bounded to actual vector-table locations. */
    if ((base & 0x3FFu) != 0u || base > UINT32_MAX - 0x16Au)
        return false;
    return canonical_code_matches(
                   mem, base + VECOFS_WINDOW_OVERFLOW4,
                   canonical_overflow4, sizeof(canonical_overflow4)) &&
           canonical_code_matches(
                   mem, base + VECOFS_WINDOW_UNDERFLOW4,
                   canonical_underflow4, sizeof(canonical_underflow4)) &&
           canonical_code_matches(
                   mem, base + VECOFS_WINDOW_OVERFLOW8,
                   canonical_overflow8, sizeof(canonical_overflow8)) &&
           canonical_code_matches(
                   mem, base + VECOFS_WINDOW_UNDERFLOW8,
                   canonical_underflow8, sizeof(canonical_underflow8)) &&
           canonical_code_matches(
                   mem, base + VECOFS_WINDOW_OVERFLOW12,
                   canonical_overflow12, sizeof(canonical_overflow12)) &&
           canonical_code_matches(
                   mem, base + VECOFS_WINDOW_UNDERFLOW12,
                   canonical_underflow12, sizeof(canonical_underflow12));
}

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
