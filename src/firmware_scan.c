#include "firmware_scan.h"
#include "xtensa.h"
#include <limits.h>
#include <zlib.h>

bool firmware_peek(xtensa_mem_t *mem, uint32_t addr, size_t size,
                   uint32_t *value_out) {
    if (!mem || !value_out || size == 0u || size > sizeof(uint32_t) ||
        size - 1u > UINT32_MAX - addr)
        return false;

    uint32_t value = 0u;
    for (size_t i = 0u; i < size; i++) {
        const uint8_t *byte = mem_get_ptr(mem, addr + (uint32_t)i);
        if (!byte)
            return false;
        value |= (uint32_t)*byte << (8u * (unsigned)i);
    }
    *value_out = value;
    return true;
}

bool firmware_signature_matches(xtensa_mem_t *mem, uint32_t addr,
                                const uint8_t *signature, size_t size) {
    if (!mem || !signature || size == 0u ||
        size - 1u > UINT32_MAX - addr)
        return false;
    for (size_t i = 0u; i < size; i++) {
        const uint8_t *byte = mem_get_ptr(mem, addr + (uint32_t)i);
        if (!byte || *byte != signature[i])
            return false;
    }
    return true;
}

bool firmware_signature_matches_except(xtensa_mem_t *mem, uint32_t addr,
                                       const uint8_t *signature, size_t size,
                                       const size_t *ignored_offsets,
                                       size_t ignored_count) {
    if (!mem || !signature || size == 0u ||
        (ignored_count != 0u && !ignored_offsets) ||
        size - 1u > UINT32_MAX - addr)
        return false;
    for (size_t j = 0u; j < ignored_count; j++)
        if (ignored_offsets[j] >= size)
            return false;

    for (size_t i = 0u; i < size; i++) {
        const uint8_t *byte = mem_get_ptr(mem, addr + (uint32_t)i);
        if (!byte)
            return false;
        bool ignored = false;
        for (size_t j = 0u; j < ignored_count; j++)
            ignored |= i == ignored_offsets[j];
        if (ignored)
            continue;
        if (*byte != signature[i])
            return false;
    }
    return true;
}

bool firmware_crc32_matches(xtensa_mem_t *mem, uint32_t addr,
                            size_t size, uint32_t expected) {
    if (!mem || size == 0u || size - 1u > UINT32_MAX - addr)
        return false;

    uLong crc = crc32(0L, Z_NULL, 0);
    for (size_t i = 0u; i < size; i++) {
        const uint8_t *byte = mem_get_ptr(mem, addr + (uint32_t)i);
        if (!byte)
            return false;
        crc = crc32(crc, byte, 1u);
    }
    return (uint32_t)crc == expected;
}

static bool firmware_xtensa_crc32_matches_kind(
        xtensa_mem_t *mem, uint32_t addr, size_t size, uint32_t expected,
        bool normalize_jump) {
    if (!mem || size == 0u || size - 1u > UINT32_MAX - addr)
        return false;

    uLong crc = crc32(0L, Z_NULL, 0);
    size_t offset = 0u;
    while (offset < size) {
        uint8_t bytes[3];
        const uint8_t *first = mem_get_ptr(mem, addr + (uint32_t)offset);
        if (!first)
            return false;
        size_t length = (*first & 0x08u) ? 2u : 3u;
        if (length > size - offset)
            return false;
        for (size_t i = 0u; i < length; i++) {
            const uint8_t *byte = mem_get_ptr(
                    mem, addr + (uint32_t)(offset + i));
            if (!byte)
                return false;
            bytes[i] = *byte;
        }

        if (length == 3u && (bytes[0] & 0x0Fu) == 1u) {
            /* L32R: preserve its opcode and destination register. */
            bytes[1] = 0u;
            bytes[2] = 0u;
        } else if (length == 3u && (bytes[0] & 0x0Fu) == 5u) {
            /* CALLn: preserve op0 and CALLINC (bits 4..5), normalize the
             * 18-bit PC-relative offset in bits 6..23. */
            bytes[0] &= 0x3Fu;
            bytes[1] = 0u;
            bytes[2] = 0u;
        } else if (normalize_jump && length == 3u &&
                   (bytes[0] & 0x3Fu) == 6u) {
            /* J has the same relocation shape as CALLn. Preserve its six
             * opcode bits and normalize the high two displacement bits too. */
            bytes[0] &= 0x3Fu;
            bytes[1] = 0u;
            bytes[2] = 0u;
        }
        crc = crc32(crc, bytes, (uInt)length);
        offset += length;
    }
    return (uint32_t)crc == expected;
}

bool firmware_xtensa_crc32_matches(xtensa_mem_t *mem, uint32_t addr,
                                   size_t size, uint32_t expected) {
    return firmware_xtensa_crc32_matches_kind(
            mem, addr, size, expected, false);
}

bool firmware_xtensa_reloc_crc32_matches(xtensa_mem_t *mem, uint32_t addr,
                                         size_t size, uint32_t expected) {
    return firmware_xtensa_crc32_matches_kind(
            mem, addr, size, expected, true);
}

void firmware_scan_xtensa_functions(
        xtensa_mem_t *mem, uint32_t start, uint32_t end,
        firmware_xtensa_function_match_t *fingerprints, size_t count) {
    if (fingerprints)
        for (size_t i = 0u; i < count; i++) {
            fingerprints[i].addr = 0u;
            fingerprints[i].matches = 0u;
        }
    if (!mem || !fingerprints || count == 0u || start >= end ||
        end - start < 3u)
        return;

    const uint32_t last = end - 3u;
    for (uint32_t addr = start; addr <= last; addr++) {
        const uint8_t *first = mem_get_ptr(mem, addr);
        if (!first || *first != 0x36u)
            continue;
        uint32_t insn;
        if (!firmware_peek(mem, addr, 3u, &insn) ||
            !firmware_xtensa_is_entry(insn))
            continue;

        for (size_t i = 0u; i < count; i++) {
            firmware_xtensa_function_match_t *match = &fingerprints[i];
            if (match->matches >= 2u || match->size < 3u ||
                match->size > (size_t)(end - addr) ||
                !firmware_xtensa_reloc_crc32_matches(
                        mem, addr, match->size, match->crc32))
                continue;
            if (match->matches == 0u) {
                match->addr = addr;
                match->matches = 1u;
            } else if (match->addr != addr) {
                match->addr = 0u;
                match->matches = 2u;
            }
        }
    }
}

unsigned firmware_find_unique_xtensa_function(
        xtensa_mem_t *mem, uint32_t start, uint32_t end, size_t size,
        uint32_t expected_crc, uint32_t *unique_addr_out) {
    if (!unique_addr_out)
        return 0u;
    *unique_addr_out = 0u;
    if (!mem || size < 3u || start >= end ||
        size > (size_t)(end - start))
        return 0u;
    firmware_xtensa_function_match_t match = {
        .size = size,
        .crc32 = expected_crc,
    };
    firmware_scan_xtensa_functions(mem, start, end, &match, 1u);
    *unique_addr_out = match.addr;
    return match.matches;
}

static bool firmware_find_crc32_body_kind(
        xtensa_mem_t *mem, uint32_t start, uint32_t end,
        const uint8_t *prefix, size_t prefix_size, size_t body_size,
        uint32_t expected_crc, uint32_t *addr_out, bool xtensa_body) {
    if (!mem || !prefix || !addr_out || prefix_size == 0u ||
        prefix_size > body_size || start >= end ||
        body_size > (size_t)(end - start))
        return false;

    uint32_t last = end - (uint32_t)body_size;
    for (uint32_t addr = start; addr <= last; addr++) {
        const uint8_t *first = mem_get_ptr(mem, addr);
        if (!first || *first != prefix[0] ||
            !firmware_signature_matches(mem, addr, prefix, prefix_size))
            continue;
        bool matches = xtensa_body
                     ? firmware_xtensa_crc32_matches(
                            mem, addr, body_size, expected_crc)
                     : firmware_crc32_matches(
                            mem, addr, body_size, expected_crc);
        if (!matches)
            continue;
        *addr_out = addr;
        return true;
    }
    return false;
}

bool firmware_find_crc32_body(xtensa_mem_t *mem, uint32_t start,
                              uint32_t end, const uint8_t *prefix,
                              size_t prefix_size, size_t body_size,
                              uint32_t expected_crc, uint32_t *addr_out) {
    return firmware_find_crc32_body_kind(
            mem, start, end, prefix, prefix_size, body_size, expected_crc,
            addr_out, false);
}

bool firmware_find_xtensa_crc32_body(
        xtensa_mem_t *mem, uint32_t start, uint32_t end,
        const uint8_t *prefix, size_t prefix_size, size_t body_size,
        uint32_t expected_crc, uint32_t *addr_out) {
    return firmware_find_crc32_body_kind(
            mem, start, end, prefix, prefix_size, body_size, expected_crc,
            addr_out, true);
}

bool firmware_xtensa_is_entry(uint32_t insn) {
    return XT_OP0(insn) == 6u &&       /* op0 = SI */
           XT_N(insn) == 3u &&         /* n = BI1 */
           XT_M(insn) == 0u &&         /* m = 0 */
           XT_S(insn) == 1u;           /* source = a1 */
}

bool firmware_xtensa_l32r_target(xtensa_mem_t *mem, uint32_t pc,
                                uint32_t *target_out) {
    uint32_t insn;
    if (!target_out || !firmware_peek(mem, pc, 3u, &insn) ||
        XT_OP0(insn) != 1u)
        return false;
    *target_out = ((pc + 3u) & ~3u) +
                  (0xFFFC0000u | ((uint32_t)XT_IMM16(insn) << 2));
    return true;
}

bool firmware_xtensa_call_target(xtensa_mem_t *mem, uint32_t pc,
                                 unsigned *callinc_out,
                                 uint32_t *target_out) {
    uint32_t insn;
    if (!callinc_out || !target_out ||
        !firmware_peek(mem, pc, 3u, &insn) || XT_OP0(insn) != 5u)
        return false;

    uint32_t encoded = XT_OFFSET18(insn);
    int32_t offset = (int32_t)(encoded << 14) >> 14;
    *callinc_out = XT_N(insn);
    *target_out = (((pc >> 2) + (uint32_t)offset + 1u) << 2);
    return true;
}
