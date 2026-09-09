#ifndef FIRMWARE_SCAN_H
#define FIRMWARE_SCAN_H

#include "memory.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Side-effect-free reads used while inspecting a loaded firmware image.
 * Unlike mem_read*, these helpers do not turn an unmapped scan probe into a
 * guest-visible unmapped-access report. */
bool firmware_peek(xtensa_mem_t *mem, uint32_t addr, size_t size,
                   uint32_t *value_out);

bool firmware_signature_matches(xtensa_mem_t *mem, uint32_t addr,
                                const uint8_t *signature, size_t size);
bool firmware_signature_matches_except(xtensa_mem_t *mem, uint32_t addr,
                                       const uint8_t *signature, size_t size,
                                       const size_t *ignored_offsets,
                                       size_t ignored_count);

bool firmware_crc32_matches(xtensa_mem_t *mem, uint32_t addr, size_t size,
                            uint32_t expected_crc);
/* Fingerprint a complete Xtensa instruction body while automatically
 * normalizing only relocation-bearing L32R immediates and CALLn offsets.
 * CALLn's opcode and CALLINC bits remain part of the hash. The body must end
 * on an instruction boundary. */
bool firmware_xtensa_crc32_matches(xtensa_mem_t *mem, uint32_t addr,
                                   size_t size, uint32_t expected_crc);

/* Match a relocatable function prefix. In addition to L32R and CALLn, this
 * form normalizes J's link-time displacement. It is intended for identifying
 * a family of stripped library functions whose public ABI is stable while
 * their final link addresses are not. A single prefix is not sufficient
 * evidence for a native replacement; callers should require several related
 * functions to resolve coherently. */
bool firmware_xtensa_reloc_crc32_matches(xtensa_mem_t *mem, uint32_t addr,
                                         size_t size,
                                         uint32_t expected_crc);

/* Search function ENTRY sites and count matches for one relocatable
 * fingerprint. `unique_addr_out` receives the address only when exactly one
 * match exists, and zero otherwise. The count is saturated at two because
 * callers only need to distinguish missing, unique, and ambiguous. */
unsigned firmware_find_unique_xtensa_function(
        xtensa_mem_t *mem, uint32_t start, uint32_t end, size_t size,
        uint32_t expected_crc, uint32_t *unique_addr_out);

/* Locate complete, address-independent bodies. The short prefix makes the
 * bytewise search cheap; CRC still covers every byte before a native
 * substitute is authorized. `end` is exclusive. */
bool firmware_find_crc32_body(xtensa_mem_t *mem, uint32_t start,
                              uint32_t end, const uint8_t *prefix,
                              size_t prefix_size, size_t body_size,
                              uint32_t expected_crc, uint32_t *addr_out);
bool firmware_find_xtensa_crc32_body(
        xtensa_mem_t *mem, uint32_t start, uint32_t end,
        const uint8_t *prefix, size_t prefix_size, size_t body_size,
        uint32_t expected_crc, uint32_t *addr_out);

/* Decode the relocation-bearing control-flow forms used by structurally
 * discovered Xtensa routines. */
bool firmware_xtensa_is_entry(uint32_t insn);
bool firmware_xtensa_l32r_target(xtensa_mem_t *mem, uint32_t pc,
                                uint32_t *target_out);
bool firmware_xtensa_call_target(xtensa_mem_t *mem, uint32_t pc,
                                 unsigned *callinc_out,
                                 uint32_t *target_out);

#endif /* FIRMWARE_SCAN_H */
