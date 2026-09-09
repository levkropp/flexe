/* Guest-memory translation of the TLSF 3.1 allocator used by ESP-IDF 4.4.
 *
 * The algorithm and structure layout are derived from Espressif's BSD-3
 * licensed components/heap/heap_tlsf.c.  The original license follows.
 *
 * Copyright (c) 2006-2016, Matthew Conte
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE.
 */
#include "tlsf_accel.h"

#include <limits.h>
#include <string.h>

enum {
    BLOCK_PREV_PHYS = 0,
    BLOCK_SIZE = 4,
    BLOCK_NEXT_FREE = 8,
    BLOCK_PREV_FREE = 12,
    BLOCK_HEADER_SIZE = 16,
    BLOCK_HEADER_OVERHEAD = 4,
    BLOCK_START_OFFSET = 8,
    BLOCK_SIZE_MIN = 12,

    CONTROL_PARAMS = 16,
    CONTROL_SIZE = 20,
    CONTROL_FL_BITMAP = 24,
    CONTROL_SL_BITMAP = 28,
    CONTROL_BLOCKS = 32,
    CONTROL_BASE_SIZE = 36,

    MULTI_HEAP_FREE_BYTES = 4,
    MULTI_HEAP_MIN_FREE_BYTES = 8,
    MULTI_HEAP_POOL_SIZE = 12,
    MULTI_HEAP_TLSF = 16,
};

#define BLOCK_FREE_BIT      1u
#define BLOCK_PREV_FREE_BIT 2u
#define BLOCK_FLAGS         3u

typedef struct {
    uint32_t addr;
    unsigned fl_count;
    unsigned fl_shift;
    unsigned fl_max;
    unsigned sl_count;
    unsigned sl_log2;
    unsigned small_size;
    uint32_t fl_bitmap_addr;
    uint32_t sl_bitmap;
    uint32_t blocks;
} tlsf_control_t;

static void plan_init(tlsf_accel_plan_t *plan, xtensa_mem_t *mem,
                      uint32_t path_prefix) {
    memset(plan, 0, sizeof(*plan));
    plan->mem = mem;
    plan->path_key = path_prefix;
    plan->valid = mem != NULL;
}

static uint32_t plan_read32(tlsf_accel_plan_t *plan, uint32_t addr) {
    if (!plan->valid || (addr & 3u) != 0u || addr > UINT32_MAX - 3u) {
        plan->valid = false;
        return 0;
    }
    for (unsigned i = plan->write_count; i > 0; i--) {
        if (plan->writes[i - 1].addr == addr)
            return plan->writes[i - 1].value;
    }
    const uint8_t *ptr = mem_get_ptr(plan->mem, addr);
    if (!ptr) {
        plan->valid = false;
        return 0;
    }
    uint32_t value;
    memcpy(&value, ptr, sizeof(value));
    return value;
}

static void plan_write32(tlsf_accel_plan_t *plan, uint32_t addr,
                         uint32_t value) {
    if (!plan->valid || (addr & 3u) != 0u || addr > UINT32_MAX - 3u ||
        !mem_get_ptr_w(plan->mem, addr)) {
        plan->valid = false;
        return;
    }
    for (unsigned i = 0; i < plan->write_count; i++) {
        if (plan->writes[i].addr == addr) {
            plan->writes[i].value = value;
            return;
        }
    }
    if (plan->write_count >= TLSF_ACCEL_MAX_WRITES) {
        plan->valid = false;
        return;
    }
    plan->writes[plan->write_count++] = (tlsf_accel_write_t){addr, value};
}

static void path_mark(tlsf_accel_plan_t *plan, unsigned bit) {
    if (bit < 24u)
        plan->path_key |= 1u << bit;
}

static unsigned bit_floor(uint32_t word) {
    return 31u - (unsigned)__builtin_clz(word);
}

static unsigned first_set(uint32_t word) {
    return (unsigned)__builtin_ctz(word);
}

static uint32_t high_mask(unsigned bit) {
    return bit < 32u ? UINT32_MAX << bit : 0u;
}

static bool add_addr(uint32_t base, uint32_t offset, uint32_t *result) {
    uint64_t sum = (uint64_t)base + offset;
    if (sum > UINT32_MAX) return false;
    *result = (uint32_t)sum;
    return true;
}

static bool control_load(tlsf_accel_plan_t *plan, uint32_t addr,
                         tlsf_control_t *control) {
    if ((addr & 3u) != 0u || addr == 0u ||
        addr > UINT32_MAX - CONTROL_BASE_SIZE)
        return false;
    uint32_t params = plan_read32(plan, addr + CONTROL_PARAMS);
    uint32_t metadata_size = plan_read32(plan, addr + CONTROL_SIZE);
    *control = (tlsf_control_t){
        .addr = addr,
        .fl_count = params & 0x1fu,
        .fl_shift = (params >> 5) & 0x7u,
        .fl_max = (params >> 8) & 0x3fu,
        .sl_count = (params >> 14) & 0x3fu,
        .sl_log2 = (params >> 20) & 0x7u,
        .small_size = (params >> 23) & 0xffu,
        .fl_bitmap_addr = addr + CONTROL_FL_BITMAP,
        .sl_bitmap = plan_read32(plan, addr + CONTROL_SL_BITMAP),
        .blocks = plan_read32(plan, addr + CONTROL_BLOCKS),
    };
    uint64_t sl_bytes = (uint64_t)control->fl_count * 4u;
    uint64_t block_bytes = (uint64_t)control->fl_count *
                           control->sl_count * 4u;
    uint64_t expected_size = CONTROL_BASE_SIZE + sl_bytes + block_bytes;
    uint64_t expected_sl = (uint64_t)addr + CONTROL_BASE_SIZE;
    uint64_t expected_blocks = expected_sl + sl_bytes;
    uint64_t metadata_end = (uint64_t)addr + expected_size;
    if (!plan->valid || expected_size > UINT32_MAX ||
        metadata_end > (uint64_t)UINT32_MAX + 1u ||
        metadata_size != expected_size ||
        control->fl_count == 0u || control->fl_count > 31u ||
        control->fl_shift < 2u || control->fl_shift > 7u ||
        control->fl_max >= 32u || control->sl_log2 > 5u ||
        control->sl_count != (1u << control->sl_log2) ||
        control->small_size != (1u << control->fl_shift) ||
        control->fl_shift != control->sl_log2 + 2u ||
        control->fl_count != control->fl_max - control->fl_shift + 1u ||
        control->sl_bitmap != expected_sl ||
        control->blocks != expected_blocks)
        return false;
    return mem_get_ptr(plan->mem, control->sl_bitmap) != NULL &&
           mem_get_ptr(plan->mem,
                       control->sl_bitmap + (uint32_t)sl_bytes - 1u) != NULL &&
           mem_get_ptr(plan->mem, control->blocks) != NULL &&
           mem_get_ptr(plan->mem,
                       control->blocks + (uint32_t)block_bytes - 1u) != NULL;
}

static uint32_t block_size(tlsf_accel_plan_t *plan, uint32_t block) {
    return plan_read32(plan, block + BLOCK_SIZE) & ~BLOCK_FLAGS;
}

static bool block_next(tlsf_accel_plan_t *plan, uint32_t block,
                       uint32_t *next) {
    uint32_t size = block_size(plan, block);
    return plan->valid && add_addr(block, size + BLOCK_HEADER_OVERHEAD, next);
}

static bool mapping_insert(const tlsf_control_t *control, uint32_t size,
                           unsigned *fl, unsigned *sl) {
    if (size < control->small_size) {
        unsigned interval = control->small_size / control->sl_count;
        if (interval == 0u) return false;
        *fl = 0u;
        *sl = size / interval;
    } else {
        unsigned top = bit_floor(size);
        if (top < control->sl_log2) return false;
        *sl = (size >> (top - control->sl_log2)) ^ control->sl_count;
        if (top < control->fl_shift - 1u) return false;
        *fl = top - (control->fl_shift - 1u);
    }
    return *fl < control->fl_count && *sl < control->sl_count;
}

static bool remove_free_block(tlsf_accel_plan_t *plan,
                              const tlsf_control_t *control,
                              uint32_t block, unsigned fl, unsigned sl,
                              unsigned path_base) {
    if (fl >= control->fl_count || sl >= control->sl_count) return false;
    uint32_t prev = plan_read32(plan, block + BLOCK_PREV_FREE);
    uint32_t next = plan_read32(plan, block + BLOCK_NEXT_FREE);
    if (!plan->valid || prev == 0u || next == 0u ||
        (prev != control->addr &&
         plan_read32(plan, prev + BLOCK_NEXT_FREE) != block) ||
        (next != control->addr &&
         plan_read32(plan, next + BLOCK_PREV_FREE) != block))
        return false;

    plan_write32(plan, next + BLOCK_PREV_FREE, prev);
    plan_write32(plan, prev + BLOCK_NEXT_FREE, next);

    uint32_t slot = control->blocks +
                    (uint32_t)(fl * control->sl_count + sl) * 4u;
    if (plan_read32(plan, slot) == block) {
        path_mark(plan, path_base);
        plan_write32(plan, slot, next);
        if (next == control->addr) {
            path_mark(plan, path_base + 1u);
            uint32_t sl_addr = control->sl_bitmap + fl * 4u;
            uint32_t sl_map = plan_read32(plan, sl_addr) & ~(1u << sl);
            plan_write32(plan, sl_addr, sl_map);
            if (sl_map == 0u) {
                path_mark(plan, path_base + 2u);
                uint32_t fl_map = plan_read32(plan,
                                              control->fl_bitmap_addr);
                plan_write32(plan, control->fl_bitmap_addr,
                             fl_map & ~(1u << fl));
            }
        }
    }
    return plan->valid;
}

static bool block_remove(tlsf_accel_plan_t *plan,
                         const tlsf_control_t *control, uint32_t block,
                         unsigned map_bit, unsigned path_base) {
    uint32_t size = block_size(plan, block);
    unsigned fl, sl;
    if (!plan->valid || !mapping_insert(control, size, &fl, &sl))
        return false;
    if (size >= control->small_size) path_mark(plan, map_bit);
    return remove_free_block(plan, control, block, fl, sl, path_base);
}

static bool insert_free_block(tlsf_accel_plan_t *plan,
                              const tlsf_control_t *control, uint32_t block,
                              unsigned map_bit) {
    uint32_t size = block_size(plan, block);
    unsigned fl, sl;
    if (!plan->valid || !mapping_insert(control, size, &fl, &sl))
        return false;
    if (size >= control->small_size) path_mark(plan, map_bit);

    uint32_t slot = control->blocks +
                    (uint32_t)(fl * control->sl_count + sl) * 4u;
    uint32_t current = plan_read32(plan, slot);
    if (!plan->valid || current == 0u) return false;
    plan_write32(plan, block + BLOCK_NEXT_FREE, current);
    plan_write32(plan, block + BLOCK_PREV_FREE, control->addr);
    plan_write32(plan, current + BLOCK_PREV_FREE, block);
    plan_write32(plan, slot, block);
    plan_write32(plan, control->fl_bitmap_addr,
                 plan_read32(plan, control->fl_bitmap_addr) | (1u << fl));
    uint32_t sl_addr = control->sl_bitmap + fl * 4u;
    plan_write32(plan, sl_addr,
                 plan_read32(plan, sl_addr) | (1u << sl));
    return plan->valid;
}

static bool mark_block_free(tlsf_accel_plan_t *plan, uint32_t block) {
    uint32_t next;
    if (!block_next(plan, block, &next)) return false;
    plan_write32(plan, next + BLOCK_PREV_PHYS, block);
    plan_write32(plan, next + BLOCK_SIZE,
                 plan_read32(plan, next + BLOCK_SIZE) |
                 BLOCK_PREV_FREE_BIT);
    plan_write32(plan, block + BLOCK_SIZE,
                 plan_read32(plan, block + BLOCK_SIZE) | BLOCK_FREE_BIT);
    return plan->valid;
}

static bool mark_block_used(tlsf_accel_plan_t *plan, uint32_t block) {
    uint32_t next;
    if (!block_next(plan, block, &next)) return false;
    plan_write32(plan, next + BLOCK_SIZE,
                 plan_read32(plan, next + BLOCK_SIZE) &
                 ~BLOCK_PREV_FREE_BIT);
    plan_write32(plan, block + BLOCK_SIZE,
                 plan_read32(plan, block + BLOCK_SIZE) & ~BLOCK_FREE_BIT);
    return plan->valid;
}

static bool block_split(tlsf_accel_plan_t *plan, uint32_t block,
                        uint32_t size, uint32_t *remaining) {
    uint32_t old_size = block_size(plan, block);
    if (old_size < size + BLOCK_HEADER_OVERHEAD) return false;
    uint32_t remain_size = old_size - size - BLOCK_HEADER_OVERHEAD;
    if (remain_size < BLOCK_SIZE_MIN ||
        !add_addr(block, size + BLOCK_HEADER_OVERHEAD, remaining))
        return false;
    uint32_t remain_word = plan_read32(plan, *remaining + BLOCK_SIZE);
    uint32_t block_word = plan_read32(plan, block + BLOCK_SIZE);
    plan_write32(plan, *remaining + BLOCK_SIZE,
                 remain_size | (remain_word & BLOCK_FLAGS));
    plan_write32(plan, block + BLOCK_SIZE,
                 size | (block_word & BLOCK_FLAGS));
    return mark_block_free(plan, *remaining);
}

static bool tlsf_malloc_plan(tlsf_accel_plan_t *plan, uint32_t control_addr,
                             uint32_t requested, uint32_t *result) {
    tlsf_control_t control;
    if (!control_load(plan, control_addr, &control) ||
        requested > UINT32_MAX - 3u)
        return false;

    if (requested == 0u) {
        path_mark(plan, 12u);
        *result = 0u;
        return plan->valid;
    }

    uint32_t size = (requested + 3u) & ~3u;
    uint32_t max_size = 1u << control.fl_max;
    if (size >= max_size) {
        path_mark(plan, 13u);
        *result = 0u;
        return plan->valid;
    }
    if (size < BLOCK_SIZE_MIN) size = BLOCK_SIZE_MIN;

    uint32_t search_size = size;
    if (search_size >= control.small_size) {
        path_mark(plan, 0u);
        unsigned top = bit_floor(search_size);
        uint32_t round = (1u << (top - control.sl_log2)) - 1u;
        if (search_size > UINT32_MAX - round) return false;
        search_size += round;
    }

    unsigned fl, sl;
    if (!mapping_insert(&control, search_size, &fl, &sl)) return false;
    uint32_t sl_map = plan_read32(plan, control.sl_bitmap + fl * 4u) &
                      high_mask(sl);
    if (sl_map == 0u) {
        path_mark(plan, 1u);
        uint32_t fl_map = plan_read32(plan, control.fl_bitmap_addr) &
                          high_mask(fl + 1u);
        if (fl_map == 0u) {
            path_mark(plan, 14u);
            *result = 0u;
            return plan->valid;
        }
        fl = first_set(fl_map);
        if (fl >= control.fl_count) return false;
        sl_map = plan_read32(plan, control.sl_bitmap + fl * 4u);
    }
    if (!plan->valid || sl_map == 0u) return false;
    sl = first_set(sl_map);
    uint32_t slot = control.blocks +
                    (uint32_t)(fl * control.sl_count + sl) * 4u;
    uint32_t block = plan_read32(plan, slot);
    uint32_t old_size = block_size(plan, block);
    if (!plan->valid || block == 0u || block == control.addr ||
        (plan_read32(plan, block + BLOCK_SIZE) & BLOCK_FREE_BIT) == 0u ||
        old_size < size ||
        !remove_free_block(plan, &control, block, fl, sl, 2u))
        return false;

    if (old_size >= BLOCK_HEADER_SIZE + size) {
        path_mark(plan, 5u);
        uint32_t remaining;
        if (!block_split(plan, block, size, &remaining)) return false;
        plan_write32(plan, remaining + BLOCK_PREV_PHYS, block);
        plan_write32(plan, remaining + BLOCK_SIZE,
                     plan_read32(plan, remaining + BLOCK_SIZE) |
                     BLOCK_PREV_FREE_BIT);
        if (!insert_free_block(plan, &control, remaining, 6u)) return false;
    }
    if (!mark_block_used(plan, block)) return false;
    if (!add_addr(block, BLOCK_START_OFFSET, result)) return false;
    return plan->valid;
}

static bool absorb_next(tlsf_accel_plan_t *plan, uint32_t block,
                        uint32_t next) {
    uint32_t a = block_size(plan, block);
    uint32_t b = block_size(plan, next);
    if (!plan->valid || a > UINT32_MAX - b - BLOCK_HEADER_OVERHEAD)
        return false;
    uint32_t word = plan_read32(plan, block + BLOCK_SIZE);
    plan_write32(plan, block + BLOCK_SIZE,
                 a + b + BLOCK_HEADER_OVERHEAD | (word & BLOCK_FLAGS));
    uint32_t after;
    if (!block_next(plan, block, &after)) return false;
    plan_write32(plan, after + BLOCK_PREV_PHYS, block);
    return plan->valid;
}

static bool tlsf_free_plan(tlsf_accel_plan_t *plan, uint32_t control_addr,
                           uint32_t ptr) {
    tlsf_control_t control;
    if (!control_load(plan, control_addr, &control) || ptr < BLOCK_START_OFFSET)
        return false;
    uint32_t block = ptr - BLOCK_START_OFFSET;
    uint32_t word = plan_read32(plan, block + BLOCK_SIZE);
    uint32_t size = word & ~BLOCK_FLAGS;
    if (!plan->valid || size < BLOCK_SIZE_MIN || (size & 3u) != 0u ||
        (word & BLOCK_FREE_BIT) != 0u || !mark_block_free(plan, block))
        return false;

    if ((plan_read32(plan, block + BLOCK_SIZE) & BLOCK_PREV_FREE_BIT) != 0u) {
        path_mark(plan, 0u);
        uint32_t prev = plan_read32(plan, block + BLOCK_PREV_PHYS);
        uint32_t prev_word = plan_read32(plan, prev + BLOCK_SIZE);
        if (!plan->valid || prev == 0u ||
            (prev_word & BLOCK_FREE_BIT) == 0u ||
            !block_remove(plan, &control, prev, 1u, 2u) ||
            !absorb_next(plan, prev, block))
            return false;
        block = prev;
    }

    uint32_t next;
    if (!block_next(plan, block, &next)) return false;
    uint32_t next_word = plan_read32(plan, next + BLOCK_SIZE);
    if ((next_word & BLOCK_FREE_BIT) != 0u) {
        path_mark(plan, 5u);
        if (block_size(plan, block) == 0u ||
            !block_remove(plan, &control, next, 6u, 7u) ||
            !absorb_next(plan, block, next))
            return false;
    }
    return insert_free_block(plan, &control, block, 10u);
}

bool tlsf_accel_plan_idf44_raw_malloc(xtensa_mem_t *mem, uint32_t control,
                                      uint32_t size,
                                      tlsf_accel_plan_t *plan) {
    plan_init(plan, mem, 0x6D000000u); /* 'm' */
    if (control == 0u || (control & 3u) != 0u) return false;
    uint32_t result;
    if (!tlsf_malloc_plan(plan, control, size, &result)) return false;
    plan->result = result;
    return plan->valid;
}

bool tlsf_accel_plan_idf44_raw_free(xtensa_mem_t *mem, uint32_t control,
                                    uint32_t ptr,
                                    tlsf_accel_plan_t *plan) {
    plan_init(plan, mem, 0x66000000u); /* 'f' */
    if (control == 0u || ptr == 0u || (control & 3u) != 0u ||
        (ptr & 3u) != 0u || ptr < BLOCK_START_OFFSET)
        return false;
    if (!tlsf_free_plan(plan, control, ptr)) return false;
    plan->result = 0u;
    return plan->valid;
}

bool tlsf_accel_plan_idf44_malloc(xtensa_mem_t *mem, uint32_t heap,
                                  uint32_t size, tlsf_accel_plan_t *plan) {
    plan_init(plan, mem, 0x4d000000u); /* 'M' */
    if (heap == 0u || size == 0u || (heap & 3u) != 0u ||
        heap > UINT32_MAX - MULTI_HEAP_TLSF)
        return false;
    uint32_t control = plan_read32(plan, heap + MULTI_HEAP_TLSF);
    uint32_t result;
    if (!tlsf_malloc_plan(plan, control, size, &result)) return false;

    if (result == 0u) {
        plan->result = 0u;
        return plan->valid;
    }
    uint32_t allocated = block_size(plan, result - BLOCK_START_OFFSET);
    uint32_t free_bytes = plan_read32(plan, heap + MULTI_HEAP_FREE_BYTES);
    if (!plan->valid || allocated > UINT32_MAX - BLOCK_HEADER_OVERHEAD ||
        free_bytes < allocated + BLOCK_HEADER_OVERHEAD)
        return false;
    free_bytes -= allocated + BLOCK_HEADER_OVERHEAD;
    plan_write32(plan, heap + MULTI_HEAP_FREE_BYTES, free_bytes);
    uint32_t minimum = plan_read32(plan, heap + MULTI_HEAP_MIN_FREE_BYTES);
    if (free_bytes < minimum) {
        path_mark(plan, 11u);
        plan_write32(plan, heap + MULTI_HEAP_MIN_FREE_BYTES, free_bytes);
    }
    plan->result = result;
    return plan->valid;
}

bool tlsf_accel_plan_idf44_free(xtensa_mem_t *mem, uint32_t heap,
                                uint32_t ptr, tlsf_accel_plan_t *plan) {
    plan_init(plan, mem, 0x46000000u); /* 'F' */
    if (heap == 0u || ptr == 0u || (heap & 3u) != 0u || (ptr & 3u) != 0u ||
        ptr < BLOCK_START_OFFSET || heap > UINT32_MAX - MULTI_HEAP_TLSF)
        return false;
    uint32_t control = plan_read32(plan, heap + MULTI_HEAP_TLSF);
    if (!plan->valid || control > UINT32_MAX - CONTROL_SIZE) return false;
    uint32_t metadata_size = plan_read32(plan, control + CONTROL_SIZE);
    uint32_t pool_size = plan_read32(plan, heap + MULTI_HEAP_POOL_SIZE);
    uint32_t pool;
    uint64_t pool_end;
    if (!plan->valid || !add_addr(control, metadata_size, &pool) ||
        (pool_end = (uint64_t)pool + pool_size) > (uint64_t)UINT32_MAX + 1u ||
        ptr < pool || (uint64_t)ptr >= pool_end)
        return false;
    uint32_t allocated = block_size(plan, ptr - BLOCK_START_OFFSET);
    uint32_t free_bytes = plan_read32(plan, heap + MULTI_HEAP_FREE_BYTES);
    if (!plan->valid || allocated < BLOCK_SIZE_MIN ||
        allocated > UINT32_MAX - BLOCK_HEADER_OVERHEAD ||
        free_bytes > UINT32_MAX - allocated - BLOCK_HEADER_OVERHEAD)
        return false;
    plan_write32(plan, heap + MULTI_HEAP_FREE_BYTES,
                 free_bytes + allocated + BLOCK_HEADER_OVERHEAD);
    if (!tlsf_free_plan(plan, control, ptr)) return false;
    plan->result = 0u;
    return plan->valid;
}

void tlsf_accel_plan_commit(const tlsf_accel_plan_t *plan) {
    if (!plan || !plan->valid) return;
    for (unsigned i = 0; i < plan->write_count; i++)
        mem_write32(plan->mem, plan->writes[i].addr, plan->writes[i].value);
}

bool tlsf_accel_plan_matches(const tlsf_accel_plan_t *plan) {
    if (!plan || !plan->valid) return false;
    for (unsigned i = 0; i < plan->write_count; i++) {
        if (mem_read32(plan->mem, plan->writes[i].addr) !=
            plan->writes[i].value)
            return false;
    }
    return true;
}
