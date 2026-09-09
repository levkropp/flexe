#ifndef TLSF_ACCEL_H
#define TLSF_ACCEL_H

#include "memory.h"

#include <stdbool.h>
#include <stdint.h>

/* A TLSF operation only changes a small number of metadata words.  Build the
 * complete change set first, then commit it atomically after every guest
 * pointer and invariant has been validated. */
#define TLSF_ACCEL_MAX_WRITES 40u

typedef struct {
    uint32_t addr;
    uint32_t value;
} tlsf_accel_write_t;

typedef struct {
    xtensa_mem_t *mem;
    tlsf_accel_write_t writes[TLSF_ACCEL_MAX_WRITES];
    unsigned write_count;
    uint32_t result;
    uint32_t path_key;
    bool valid;
} tlsf_accel_plan_t;

/* Model ESP-IDF 4.4's multi_heap_*_impl operation, including its free-byte
 * accounting, against 32-bit pointers stored in guest memory.  No guest
 * memory is changed until tlsf_accel_plan_commit() is called. */
bool tlsf_accel_plan_idf44_malloc(xtensa_mem_t *mem, uint32_t heap,
                                  uint32_t size, tlsf_accel_plan_t *plan);
bool tlsf_accel_plan_idf44_free(xtensa_mem_t *mem, uint32_t heap,
                                uint32_t ptr, tlsf_accel_plan_t *plan);

/* The corresponding raw TLSF operations omit multi_heap's byte counters.
 * They are used at the inner firmware boundary when the outer implementation
 * cannot safely be collapsed because of its deeper register-window span. */
bool tlsf_accel_plan_idf44_raw_malloc(xtensa_mem_t *mem, uint32_t control,
                                      uint32_t size,
                                      tlsf_accel_plan_t *plan);
bool tlsf_accel_plan_idf44_raw_free(xtensa_mem_t *mem, uint32_t control,
                                    uint32_t ptr,
                                    tlsf_accel_plan_t *plan);

void tlsf_accel_plan_commit(const tlsf_accel_plan_t *plan);
bool tlsf_accel_plan_matches(const tlsf_accel_plan_t *plan);

#endif /* TLSF_ACCEL_H */
