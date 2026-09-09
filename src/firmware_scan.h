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
