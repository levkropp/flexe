#include "sha_stubs.h"
#include "firmware_scan.h"
#include "rom_stubs.h"
#include "memory.h"
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

/* Suppress OpenSSL 3.0 deprecation warnings — we need the low-level
 * SHA Transform functions for single-block compression. */
#define OPENSSL_API_COMPAT 0x10100000L
#include <openssl/sha.h>
#include <openssl/md5.h>

/* SHA type constants (matches esp_sha_type enum) */
#define SHA_TYPE_1    0
#define SHA_TYPE_256  1
#define SHA_TYPE_384  2
#define SHA_TYPE_512  3

/* ESP32 has 3 SHA hardware engines; SHA-384 and SHA-512 share one */
#define SHA_NUM_ENGINES 3
#define MBED_SHA256_FAMILY_MAX 4

static int sha_engine_index(uint32_t sha_type) {
    switch (sha_type) {
    case SHA_TYPE_1:   return 0;
    case SHA_TYPE_256: return 1;
    case SHA_TYPE_384:
    case SHA_TYPE_512: return 2;
    default:           return 0;
    }
}

struct sha_stubs {
    xtensa_cpu_t      *cpu;
    esp32_rom_stubs_t *rom;
    const flexe_target_desc_t *target;
    flexe_gdma_t      *gdma;
    mmio_read_fn       fallback_read;
    mmio_write_fn      fallback_write;
    void              *fallback_ctx;

    /* Message and digest register files. Classic ESP32 aliases both through
     * SHA_TEXT; unified accelerators expose SHA_TEXT and SHA_H separately. */
    uint32_t sha_text[32];
    uint32_t sha_h[16];
    uint32_t control[16];
    bool sha_h_dirty;

    /* Internal compression state. */
    uint32_t sha1_engine[5];     /* SHA-1 engine internal state */
    uint32_t sha224_engine[8];   /* SHA-224 engine internal state */
    uint32_t sha256_engine[8];   /* SHA-256 engine internal state */
    uint64_t sha512_engine[8];   /* SHA-384/512 engine internal state */

    /* Per-engine lock state.  On real hardware, esp_sha_try_lock_engine()
     * uses a FreeRTOS semaphore per engine.  If the engine is already locked
     * by another context, try_lock returns false and the caller falls back
     * to software SHA.  Simple bools suffice since both cores execute
     * sequentially on the same host thread. */
    bool engine_locked[SHA_NUM_ENGINES];

    int current_type;   /* last sha_type used */

    /* mbedtls software crypto acceleration: native OpenSSL contexts
     * indexed by guest mbedtls_sha256_context address. */
#define MBED_CTX_SLOTS 32
    struct {
        uint32_t    guest_addr;  /* 0 = empty */
        SHA256_CTX  ctx;
    } mbed_sha256[MBED_CTX_SLOTS];
    uint32_t firmware_sha256_starts[MBED_SHA256_FAMILY_MAX];
    unsigned firmware_sha256_count;
    struct {
        uint32_t    guest_addr;
        SHA_CTX     ctx;
    } mbed_sha1[MBED_CTX_SLOTS];
    struct {
        uint32_t    guest_addr;
        MD5_CTX     ctx;
    } mbed_md5[MBED_CTX_SLOTS];

    uint32_t warned_mode_mask;
    bool warned_dma;
};

/* ===== SHA peripheral MMIO handler ===== */

static bool sha_algorithm_supported(flexe_sha_algorithm_t algorithm) {
    return algorithm >= FLEXE_SHA_ALGORITHM_SHA1 &&
           algorithm <= FLEXE_SHA_ALGORITHM_SHA512;
}

static size_t sha_block_size(flexe_sha_algorithm_t algorithm) {
    switch (algorithm) {
    case FLEXE_SHA_ALGORITHM_SHA1:
    case FLEXE_SHA_ALGORITHM_SHA224:
    case FLEXE_SHA_ALGORITHM_SHA256:
        return 64u;
    case FLEXE_SHA_ALGORITHM_SHA384:
    case FLEXE_SHA_ALGORITHM_SHA512:
        return 128u;
    default:
        return 0u;
    }
}

static flexe_sha_algorithm_t sha_mode_algorithm(const sha_stubs_t *ss,
                                                 uint32_t mode) {
    const flexe_sha_desc_t *desc = &ss->target->sha;
    if (mode >= desc->mode_count || mode >= FLEXE_TARGET_SHA_MODE_MAX)
        return FLEXE_SHA_ALGORITHM_NONE;
    return desc->mode[mode];
}

static void sha_warn_mode(sha_stubs_t *ss, uint32_t mode) {
    uint32_t bit = mode < 32u ? 1u << mode : 0u;
    if (bit != 0u && (ss->warned_mode_mask & bit) != 0u) return;
    ss->warned_mode_mask |= bit;
    fprintf(stderr,
            "[sha] unsupported mode %u for target %s; operation rejected\n",
            mode, ss->target->name);
}

/* Recover a direct-mode message block from SHA_TEXT. Classic ESP32's HAL
 * byte-swaps each word before MMIO, while the S2/S3 unified HAL copies native
 * little-endian words and lets the accelerator perform byte ordering. */
static void sha_text_to_block(const sha_stubs_t *ss, uint8_t *raw, int words) {
    bool classic = ss->target->sha.layout == FLEXE_SHA_LAYOUT_ESP32;
    for (int i = 0; i < words; i++) {
        uint32_t w = ss->sha_text[i];
        if (classic) {
            raw[i * 4 + 0] = (uint8_t)(w >> 24);
            raw[i * 4 + 1] = (uint8_t)(w >> 16);
            raw[i * 4 + 2] = (uint8_t)(w >> 8);
            raw[i * 4 + 3] = (uint8_t)w;
        } else {
            raw[i * 4 + 0] = (uint8_t)w;
            raw[i * 4 + 1] = (uint8_t)(w >> 8);
            raw[i * 4 + 2] = (uint8_t)(w >> 16);
            raw[i * 4 + 3] = (uint8_t)(w >> 24);
        }
    }
}

static void sha_engine_hash_raw(sha_stubs_t *ss,
                                flexe_sha_algorithm_t algorithm,
                                const uint8_t *raw, bool first) {
    switch (algorithm) {
    case FLEXE_SHA_ALGORITHM_SHA1: {
        SHA_CTX c;
        uint32_t *h = ss->sha1_engine;
        SHA1_Init(&c);
        if (first) {
            h[0] = c.h0; h[1] = c.h1; h[2] = c.h2; h[3] = c.h3; h[4] = c.h4;
        }
        c.h0 = h[0]; c.h1 = h[1]; c.h2 = h[2]; c.h3 = h[3]; c.h4 = h[4];
        SHA1_Transform(&c, raw);
        h[0] = c.h0; h[1] = c.h1; h[2] = c.h2; h[3] = c.h3; h[4] = c.h4;
        break;
    }
    case FLEXE_SHA_ALGORITHM_SHA224:
    case FLEXE_SHA_ALGORITHM_SHA256: {
        SHA256_CTX c;
        uint32_t *h = algorithm == FLEXE_SHA_ALGORITHM_SHA224
            ? ss->sha224_engine : ss->sha256_engine;
        if (algorithm == FLEXE_SHA_ALGORITHM_SHA224)
            SHA224_Init(&c);
        else
            SHA256_Init(&c);
        if (first) memcpy(h, c.h, sizeof c.h);
        memcpy(c.h, h, sizeof c.h);
        SHA256_Transform(&c, raw);
        memcpy(h, c.h, sizeof c.h);
        break;
    }
    case FLEXE_SHA_ALGORITHM_SHA384:
    case FLEXE_SHA_ALGORITHM_SHA512: {
        SHA512_CTX c;
        uint64_t *h = ss->sha512_engine;
        if (algorithm == FLEXE_SHA_ALGORITHM_SHA384)
            SHA384_Init(&c);
        else
            SHA512_Init(&c);
        if (first) memcpy(h, c.h, sizeof c.h);
        memcpy(c.h, h, sizeof c.h);
        SHA512_Transform(&c, raw);
        memcpy(h, c.h, sizeof c.h);
        break;
    }
    default:
        break;
    }
}

static void sha_engine_hash_text(sha_stubs_t *ss,
                                 flexe_sha_algorithm_t algorithm,
                                 bool first) {
    uint8_t raw[128];
    size_t block_size = sha_block_size(algorithm);
    if (block_size == 0u) return;
    sha_text_to_block(ss, raw, (int)(block_size / 4u));
    sha_engine_hash_raw(ss, algorithm, raw, first);
}

static void sha_engine_publish_to(sha_stubs_t *ss,
                                  flexe_sha_algorithm_t algorithm,
                                  uint32_t *destination) {
    switch (algorithm) {
    case FLEXE_SHA_ALGORITHM_SHA1:
        for (int i = 0; i < 5; i++) destination[i] = ss->sha1_engine[i];
        break;
    case FLEXE_SHA_ALGORITHM_SHA224:
        for (int i = 0; i < 8; i++) destination[i] = ss->sha224_engine[i];
        break;
    case FLEXE_SHA_ALGORITHM_SHA256:
        for (int i = 0; i < 8; i++) destination[i] = ss->sha256_engine[i];
        break;
    case FLEXE_SHA_ALGORITHM_SHA384:
    case FLEXE_SHA_ALGORITHM_SHA512:
        for (int i = 0; i < 8; i++) {
            destination[i * 2] = (uint32_t)(ss->sha512_engine[i] >> 32);
            destination[i * 2 + 1] = (uint32_t)ss->sha512_engine[i];
        }
        break;
    default:
        break;
    }
}

static void sha_engine_import_h(sha_stubs_t *ss,
                                flexe_sha_algorithm_t algorithm) {
    switch (algorithm) {
    case FLEXE_SHA_ALGORITHM_SHA1:
        memcpy(ss->sha1_engine, ss->sha_h, 5u * sizeof(uint32_t));
        break;
    case FLEXE_SHA_ALGORITHM_SHA224:
        memcpy(ss->sha224_engine, ss->sha_h, 8u * sizeof(uint32_t));
        break;
    case FLEXE_SHA_ALGORITHM_SHA256:
        memcpy(ss->sha256_engine, ss->sha_h, 8u * sizeof(uint32_t));
        break;
    case FLEXE_SHA_ALGORITHM_SHA384:
    case FLEXE_SHA_ALGORITHM_SHA512:
        for (int i = 0; i < 8; i++)
            ss->sha512_engine[i] = ((uint64_t)ss->sha_h[i * 2] << 32) |
                                   ss->sha_h[i * 2 + 1];
        break;
    default:
        break;
    }
    ss->sha_h_dirty = false;
}

static void sha_unified_finish_block(sha_stubs_t *ss,
                                     flexe_sha_algorithm_t algorithm,
                                     const uint8_t *raw, bool first) {
    if (!first && ss->sha_h_dirty) sha_engine_import_h(ss, algorithm);
    sha_engine_hash_raw(ss, algorithm, raw, first);
    sha_engine_publish_to(ss, algorithm, ss->sha_h);
    ss->sha_h_dirty = false;
}

static void sha_unified_direct(sha_stubs_t *ss, bool first) {
    uint32_t mode = ss->control[0];
    flexe_sha_algorithm_t algorithm = sha_mode_algorithm(ss, mode);
    if (!sha_algorithm_supported(algorithm)) {
        sha_warn_mode(ss, mode);
        return;
    }
    uint8_t raw[128];
    size_t block_size = sha_block_size(algorithm);
    sha_text_to_block(ss, raw, (int)(block_size / 4u));
    sha_unified_finish_block(ss, algorithm, raw, first);
}

static void sha_unified_dma(sha_stubs_t *ss, bool first) {
    uint32_t mode = ss->control[0];
    flexe_sha_algorithm_t algorithm = sha_mode_algorithm(ss, mode);
    size_t block_size = sha_block_size(algorithm);
    size_t block_count = ss->control[3] & 0x3Fu;
    if (!sha_algorithm_supported(algorithm) || block_size == 0u) {
        sha_warn_mode(ss, mode);
        return;
    }
    if (!ss->gdma || ss->target->sha.dma_peripheral_id == UINT8_MAX ||
        block_count == 0u || block_count > SIZE_MAX / block_size) {
        if (!ss->warned_dma) {
            fprintf(stderr,
                    "[sha] target %s DMA operation has no valid GDMA stream; "
                    "operation rejected\n", ss->target->name);
            ss->warned_dma = true;
        }
        return;
    }

    size_t length = block_count * block_size;
    uint8_t *raw = malloc(length);
    if (!raw || flexe_gdma_read_tx(
            ss->gdma, ss->target->sha.dma_peripheral_id,
            raw, length) != 0) {
        if (!ss->warned_dma) {
            fprintf(stderr,
                    "[sha] target %s rejected an absent or malformed GDMA "
                    "descriptor chain\n", ss->target->name);
            ss->warned_dma = true;
        }
        free(raw);
        return;
    }
    for (size_t block = 0u; block < block_count; block++)
        sha_unified_finish_block(
            ss, algorithm, raw + block * block_size,
            first && block == 0u);
    free(raw);
}

static uint32_t sha_mmio_read(void *ctx, uint32_t addr) {
    sha_stubs_t *ss = ctx;
    const flexe_sha_desc_t *desc = &ss->target->sha;
    if (addr < desc->base || addr >= desc->base + desc->register_size)
        return ss->fallback_read
            ? ss->fallback_read(ss->fallback_ctx, addr) : 0u;
    uint32_t off = addr - desc->base;

    if (desc->layout == FLEXE_SHA_LAYOUT_ESP32 && off < 0x80u) {
        /* SHA_TEXT registers (32 words) */
        return ss->sha_text[off / 4];
    }
    if (desc->layout == FLEXE_SHA_LAYOUT_ESP32 &&
        off >= 0x80u && off < 0xC0u)
        return 0u; /* Commands self-clear and BUSY is idle in fast mode. */
    if (desc->layout == FLEXE_SHA_LAYOUT_UNIFIED) {
        if (off < 0x40u) {
            if (off == 0x18u) return 0u; /* BUSY */
            return ss->control[off / 4u];
        }
        if (off < 0x80u) return ss->sha_h[(off - 0x40u) / 4u];
        if (off < 0x100u) return ss->sha_text[(off - 0x80u) / 4u];
    }
    return ss->fallback_read
        ? ss->fallback_read(ss->fallback_ctx, addr) : 0u;
}

static void sha_mmio_write(void *ctx, uint32_t addr, uint32_t val) {
    sha_stubs_t *ss = ctx;
    const flexe_sha_desc_t *desc = &ss->target->sha;
    if (addr < desc->base || addr >= desc->base + desc->register_size) {
        if (ss->fallback_write)
            ss->fallback_write(ss->fallback_ctx, addr, val);
        return;
    }
    uint32_t off = addr - desc->base;

    if (desc->layout == FLEXE_SHA_LAYOUT_ESP32 && off < 0x80u) {
        /* SHA_TEXT registers — firmware writes state here via sha_ll_write_digest */
        ss->sha_text[off / 4] = val;
        return;
    }

    if (desc->layout == FLEXE_SHA_LAYOUT_ESP32 &&
        off >= 0x80u && off < 0xC0u) {
        uint32_t mode = (off - 0x80u) / 0x10u;
        flexe_sha_algorithm_t algorithm = sha_mode_algorithm(ss, mode);
        switch (off & 0xFu) {
        case 0x0: sha_engine_hash_text(ss, algorithm, true);  break;
        case 0x4: sha_engine_hash_text(ss, algorithm, false); break;
        case 0x8:
            sha_engine_publish_to(ss, algorithm, ss->sha_text);
            break;
        default: break;
        }
        return;
    }

    if (desc->layout == FLEXE_SHA_LAYOUT_UNIFIED) {
        if (off >= 0x40u && off < 0x80u) {
            ss->sha_h[(off - 0x40u) / 4u] = val;
            ss->sha_h_dirty = true;
            return;
        }
        if (off >= 0x80u && off < 0x100u) {
            ss->sha_text[(off - 0x80u) / 4u] = val;
            return;
        }
        if (off < 0x40u) {
            switch (off) {
            case 0x00u: ss->control[0] = val; return; /* MODE */
            case 0x04u: ss->control[1] = val; return; /* T string */
            case 0x08u: ss->control[2] = val; return; /* T length */
            case 0x0Cu: ss->control[3] = val & 0x3Fu; return;
            case 0x10u: sha_unified_direct(ss, true); return;
            case 0x14u: sha_unified_direct(ss, false); return;
            case 0x18u: return; /* BUSY is read-only. */
            case 0x1Cu: sha_unified_dma(ss, true); return;
            case 0x20u: sha_unified_dma(ss, false); return;
            case 0x24u: ss->control[9] = 0u; return; /* clear IRQ */
            case 0x28u: ss->control[10] = val & 1u; return;
            default: ss->control[off / 4u] = val; return;
            }
        }
    }
    if (ss->fallback_write)
        ss->fallback_write(ss->fallback_ctx, addr, val);
}

static bool sha_geometry_valid(const flexe_target_desc_t *target) {
    if (!target || !(target->capabilities & FLEXE_TARGET_CAP_SHA_V1))
        return false;
    const flexe_sha_desc_t *desc = &target->sha;
    uint32_t minimum = desc->layout == FLEXE_SHA_LAYOUT_ESP32
        ? 0xC0u : desc->layout == FLEXE_SHA_LAYOUT_UNIFIED ? 0x100u : 0u;
    if (minimum == 0u || desc->base < target->peripheral_start ||
        desc->base >= target->peripheral_end ||
        (desc->base & 0xFFFu) != 0u ||
        desc->register_size != 0x1000u ||
        desc->register_size > target->peripheral_end - desc->base ||
        desc->register_size < minimum || desc->mode_count == 0u ||
        desc->mode_count > FLEXE_TARGET_SHA_MODE_MAX)
        return false;
    for (unsigned mode = 0u; mode < desc->mode_count; mode++)
        if (desc->mode[mode] <= FLEXE_SHA_ALGORITHM_NONE ||
            desc->mode[mode] > FLEXE_SHA_ALGORITHM_SHA512_T)
            return false;
    if (desc->dma_peripheral_id != UINT8_MAX &&
        !(target->capabilities & FLEXE_TARGET_CAP_GDMA_V1))
        return false;
    return true;
}

/* ===== Calling convention helpers ===== */

static uint32_t sha_arg(xtensa_cpu_t *cpu, int n) {
    int ci = XT_PS_CALLINC(cpu->ps);
    return ar_read(cpu, ci * 4 + 2 + n);
}

static void sha_return(xtensa_cpu_t *cpu, uint32_t retval) {
    int ci = XT_PS_CALLINC(cpu->ps);
    if (ci > 0) {
        ar_write(cpu, ci * 4 + 2, retval);
        uint32_t a0 = ar_read(cpu, ci * 4);
        cpu->pc = (cpu->pc & 0xC0000000u) | (a0 & 0x3FFFFFFFu);
        XT_PS_SET_CALLINC(cpu->ps, 0);
    } else {
        ar_write(cpu, 2, retval);
        cpu->pc = (cpu->pc & 0xC0000000u) | (ar_read(cpu, 0) & 0x3FFFFFFFu);
    }
}

static void sha_return_void(xtensa_cpu_t *cpu) {
    sha_return(cpu, 0);  /* return value ignored by caller */
}

/* ===== HAL stub implementations ===== */

/*
 * sha_hal_hash_block(sha_type, data_block, block_word_len, is_first_block)
 *
 * Reads raw bytes from data_block in emulator memory, then runs
 * OpenSSL's SHA*_Transform for single-block compression.
 *
 * OpenSSL's Transform functions take raw bytes and handle BE conversion
 * internally — same endianness conversion the ESP32 hardware does after
 * sha_ll_fill_text_block's __builtin_bswap32.
 */
static void stub_sha_hal_hash_block(xtensa_cpu_t *cpu, void *ctx) {
    sha_stubs_t *ss = ctx;
    uint32_t sha_type       = sha_arg(cpu, 0);
    uint32_t data_ptr       = sha_arg(cpu, 1);
    uint32_t block_word_len = sha_arg(cpu, 2);
    uint32_t is_first       = sha_arg(cpu, 3);

    ss->current_type = (int)sha_type;

    /* Read raw bytes from emulator memory */
    uint8_t raw[128]; /* max block: SHA-512 = 128 bytes */
    uint32_t block_bytes = block_word_len * 4;
    if (block_bytes > sizeof(raw)) block_bytes = sizeof(raw);
    for (uint32_t i = 0; i < block_bytes; i++)
        raw[i] = mem_read8(cpu->mem, data_ptr + i);

    switch (sha_type) {
    case SHA_TYPE_1: {
        SHA_CTX sha_ctx;
        uint32_t *h = ss->sha1_engine;
        if (is_first) {
            SHA1_Init(&sha_ctx);
            h[0] = sha_ctx.h0; h[1] = sha_ctx.h1; h[2] = sha_ctx.h2;
            h[3] = sha_ctx.h3; h[4] = sha_ctx.h4;
        }
        sha_ctx.h0 = h[0]; sha_ctx.h1 = h[1]; sha_ctx.h2 = h[2];
        sha_ctx.h3 = h[3]; sha_ctx.h4 = h[4];
        SHA1_Transform(&sha_ctx, raw);
        h[0] = sha_ctx.h0; h[1] = sha_ctx.h1; h[2] = sha_ctx.h2;
        h[3] = sha_ctx.h3; h[4] = sha_ctx.h4;
        for (int i = 0; i < 5; i++)
            ss->sha_text[i] = h[i];
        break;
    }
    case SHA_TYPE_256: {
        SHA256_CTX sha_ctx;
        uint32_t *h = ss->sha256_engine;
        if (is_first) {
            SHA256_Init(&sha_ctx);
            memcpy(h, sha_ctx.h, sizeof(sha_ctx.h));
        }
        memcpy(sha_ctx.h, h, sizeof(sha_ctx.h));
        SHA256_Transform(&sha_ctx, raw);
        memcpy(h, sha_ctx.h, sizeof(sha_ctx.h));
        for (int i = 0; i < 8; i++)
            ss->sha_text[i] = h[i];
        break;
    }
    case SHA_TYPE_384: {
        SHA512_CTX sha_ctx;
        uint64_t *h = ss->sha512_engine;
        if (is_first) {
            SHA384_Init(&sha_ctx);
            memcpy(h, sha_ctx.h, sizeof(sha_ctx.h));
        }
        memcpy(sha_ctx.h, h, sizeof(sha_ctx.h));
        SHA512_Transform(&sha_ctx, raw);
        memcpy(h, sha_ctx.h, sizeof(sha_ctx.h));
        for (int i = 0; i < 8; i++) {
            ss->sha_text[i*2]     = (uint32_t)(h[i] >> 32);
            ss->sha_text[i*2 + 1] = (uint32_t)h[i];
        }
        break;
    }
    case SHA_TYPE_512: {
        SHA512_CTX sha_ctx;
        uint64_t *h = ss->sha512_engine;
        if (is_first) {
            SHA512_Init(&sha_ctx);
            memcpy(h, sha_ctx.h, sizeof(sha_ctx.h));
        }
        memcpy(sha_ctx.h, h, sizeof(sha_ctx.h));
        SHA512_Transform(&sha_ctx, raw);
        memcpy(h, sha_ctx.h, sizeof(sha_ctx.h));
        for (int i = 0; i < 8; i++) {
            ss->sha_text[i*2]     = (uint32_t)(h[i] >> 32);
            ss->sha_text[i*2 + 1] = (uint32_t)h[i];
        }
        break;
    }
    }

    sha_return_void(cpu);
}

/*
 * sha_hal_read_digest(sha_type, digest_state, digest_word_len)
 *
 * Writes current state from sha_text[] to digest_state in emulator memory.
 * State is in sha_text[] as uint32_t words (matching hardware register layout).
 */
static void stub_sha_hal_read_digest(xtensa_cpu_t *cpu, void *ctx) {
    sha_stubs_t *ss = ctx;
    uint32_t sha_type        = sha_arg(cpu, 0);
    uint32_t digest_ptr      = sha_arg(cpu, 1);
    uint32_t digest_word_len = sha_arg(cpu, 2);

    /* Cap at register file size */
    uint32_t max;
    switch (sha_type) {
    case SHA_TYPE_1:   max = 5;  break;
    case SHA_TYPE_256: max = 8;  break;
    case SHA_TYPE_384: max = 16; break;
    case SHA_TYPE_512: max = 16; break;
    default:           max = 8;  break;
    }
    if (digest_word_len > max) digest_word_len = max;

    for (uint32_t i = 0; i < digest_word_len; i++)
        mem_write32(cpu->mem, digest_ptr + i * 4, ss->sha_text[i]);

    sha_return_void(cpu);
}

/* sha_hal_wait_idle() — hardware is always idle in emulation */
static void stub_sha_hal_wait_idle(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    sha_return_void(cpu);
}

/* Engine lock/unlock stubs.
 * On real hardware, esp_sha_try_lock_engine uses a per-engine FreeRTOS
 * semaphore.  If the engine is busy (locked by another mbedtls context),
 * try_lock returns false and the caller falls back to software SHA.
 * We replicate this so interleaving contexts don't corrupt engine state. */
static void stub_sha_lock_engine(xtensa_cpu_t *cpu, void *ctx) {
    sha_stubs_t *ss = ctx;
    uint32_t sha_type = sha_arg(cpu, 0);
    int engine = sha_engine_index(sha_type);
    ss->engine_locked[engine] = true;
    sha_return_void(cpu);
}

static void stub_sha_try_lock_engine(xtensa_cpu_t *cpu, void *ctx) {
    sha_stubs_t *ss = ctx;
    uint32_t sha_type = sha_arg(cpu, 0);
    int engine = sha_engine_index(sha_type);

    bool success = !ss->engine_locked[engine];
    if (success)
        ss->engine_locked[engine] = true;

    /* Return true (1) if lock acquired, false (0) if engine busy */
    int ci = XT_PS_CALLINC(cpu->ps);
    if (ci > 0) {
        ar_write(cpu, ci * 4 + 2, success ? 1 : 0);
        uint32_t a0 = ar_read(cpu, ci * 4);
        cpu->pc = (cpu->pc & 0xC0000000u) | (a0 & 0x3FFFFFFFu);
        XT_PS_SET_CALLINC(cpu->ps, 0);
    } else {
        ar_write(cpu, 2, success ? 1 : 0);
        cpu->pc = (cpu->pc & 0xC0000000u) | (ar_read(cpu, 0) & 0x3FFFFFFFu);
    }
}

static void stub_sha_unlock_engine(xtensa_cpu_t *cpu, void *ctx) {
    sha_stubs_t *ss = ctx;
    uint32_t sha_type = sha_arg(cpu, 0);
    int engine = sha_engine_index(sha_type);
    ss->engine_locked[engine] = false;
    sha_return_void(cpu);
}

static void stub_sha_lock_memory(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    sha_return_void(cpu);
}

static void stub_sha_unlock_memory(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    sha_return_void(cpu);
}

/* ===== Self-test ===== */

/* Verify SHA-256 via OpenSSL against known test vector: SHA-256("abc") */
static int sha256_self_test(void) {
    unsigned char md[SHA256_DIGEST_LENGTH];
    SHA256((const unsigned char *)"abc", 3, md);

    static const unsigned char expected[32] = {
        0xba,0x78,0x16,0xbf, 0x8f,0x01,0xcf,0xea,
        0x41,0x41,0x40,0xde, 0x5d,0xae,0x22,0x23,
        0xb0,0x03,0x61,0xa3, 0x96,0x17,0x7a,0x9c,
        0xb4,0x10,0xff,0x61, 0xf2,0x00,0x15,0xad
    };

    return memcmp(md, expected, sizeof(expected)) == 0;
}

/* ===== Public API ===== */

sha_stubs_t *sha_stubs_create(xtensa_cpu_t *cpu, flexe_gdma_t *gdma) {
    if (!cpu || !cpu->mem || !sha_geometry_valid(mem_target(cpu->mem)))
        return NULL;
    sha_stubs_t *ss = calloc(1, sizeof(*ss));
    if (!ss) return NULL;
    ss->cpu = cpu;
    ss->target = mem_target(cpu->mem);
    ss->gdma = gdma;
    ss->current_type = -1;

    /* Verify SHA-256 via OpenSSL is correct */
    if (!sha256_self_test()) {
        fprintf(stderr, "[sha] FATAL: SHA-256 self-test failed!\n");
        free(ss);
        return NULL;
    }

    uint32_t page = (ss->target->sha.base -
                     ss->target->peripheral_start) / 0x1000u;
    ss->fallback_read = cpu->mem->mmio[page].read;
    ss->fallback_write = cpu->mem->mmio[page].write;
    ss->fallback_ctx = cpu->mem->mmio[page].ctx;
    if (mem_register_mmio_range(cpu->mem, ss->target->sha.base,
                                ss->target->sha.register_size,
                                sha_mmio_read, sha_mmio_write, ss) != 0) {
        free(ss);
        return NULL;
    }

    return ss;
}

void sha_stubs_destroy(sha_stubs_t *ss) {
    if (!ss) return;
    (void)mem_register_mmio_range(
        ss->cpu->mem, ss->target->sha.base, ss->target->sha.register_size,
        ss->fallback_read, ss->fallback_write, ss->fallback_ctx);
    free(ss);
}

/* ===== mbedtls software SHA256 acceleration =====
 * Replaces interpreted mbedtls SHA256 with native OpenSSL.
 * These are GENERIC — they work with ANY ESP32 firmware using mbedtls. */

static int mbed_sha256_find(sha_stubs_t *ss, uint32_t addr) {
    for (int i = 0; i < MBED_CTX_SLOTS; i++)
        if (ss->mbed_sha256[i].guest_addr == addr) return i;
    return -1;
}

static int mbed_sha256_alloc(sha_stubs_t *ss, uint32_t addr) {
    /* Reuse existing slot or find empty */
    int idx = mbed_sha256_find(ss, addr);
    if (idx >= 0) return idx;
    for (int i = 0; i < MBED_CTX_SLOTS; i++) {
        if (ss->mbed_sha256[i].guest_addr == 0) {
            ss->mbed_sha256[i].guest_addr = addr;
            return i;
        }
    }
    return -1; /* full */
}

static void stub_mbedtls_sha256_starts(xtensa_cpu_t *cpu, void *ctx) {
    sha_stubs_t *ss = ctx;
    uint32_t ctx_addr = sha_arg(cpu, 0);
    uint32_t is224 = sha_arg(cpu, 1);
    int idx = mbed_sha256_alloc(ss, ctx_addr);
    if (idx >= 0) {
        if (is224)
            SHA224_Init(&ss->mbed_sha256[idx].ctx);
        else
            SHA256_Init(&ss->mbed_sha256[idx].ctx);
    }
    sha_return(cpu, 0); /* 0 = success */
}

static void stub_mbedtls_sha256_update(xtensa_cpu_t *cpu, void *ctx) {
    sha_stubs_t *ss = ctx;
    uint32_t ctx_addr = sha_arg(cpu, 0);
    uint32_t data_addr = sha_arg(cpu, 1);
    uint32_t len = sha_arg(cpu, 2);
    int idx = mbed_sha256_find(ss, ctx_addr);
    if (idx >= 0 && len > 0) {
        /* Read data from emulated memory in page-sized chunks */
        uint8_t tmp[4096];
        uint32_t off = 0;
        while (off < len) {
            uint32_t chunk = len - off;
            if (chunk > sizeof(tmp)) chunk = sizeof(tmp);
            const uint8_t *p = mem_get_ptr(cpu->mem, data_addr + off);
            if (p) {
                uint32_t page_rem = 0x1000 - ((data_addr + off) & 0xFFF);
                if (chunk > page_rem) chunk = page_rem;
                SHA256_Update(&ss->mbed_sha256[idx].ctx, p, chunk);
            } else {
                for (uint32_t i = 0; i < chunk; i++)
                    tmp[i] = mem_read8(cpu->mem, data_addr + off + i);
                SHA256_Update(&ss->mbed_sha256[idx].ctx, tmp, chunk);
            }
            off += chunk;
        }
    }
    sha_return(cpu, 0);
}

static void stub_mbedtls_sha256_finish(xtensa_cpu_t *cpu, void *ctx) {
    sha_stubs_t *ss = ctx;
    uint32_t ctx_addr = sha_arg(cpu, 0);
    uint32_t out_addr = sha_arg(cpu, 1);
    int idx = mbed_sha256_find(ss, ctx_addr);
    if (idx >= 0) {
        unsigned char digest[32];
        /* Use a copy so the context can be reused */
        SHA256_CTX copy = ss->mbed_sha256[idx].ctx;
        SHA256_Final(digest, &copy);
        /* Write digest to emulated memory */
        uint8_t *p = mem_get_ptr_w(cpu->mem, out_addr);
        if (p && ((out_addr & 0xFFF) <= 0xFE0))  /* fits in page */
            memcpy(p, digest, 32);
        else
            for (int i = 0; i < 32; i++)
                mem_write8(cpu->mem, out_addr + i, digest[i]);
    }
    sha_return(cpu, 0);
}

static void stub_mbedtls_sha256_free(xtensa_cpu_t *cpu, void *ctx) {
    sha_stubs_t *ss = ctx;
    uint32_t ctx_addr = sha_arg(cpu, 0);
    int idx = mbed_sha256_find(ss, ctx_addr);
    if (idx >= 0)
        ss->mbed_sha256[idx].guest_addr = 0; /* release slot */
    sha_return_void(cpu);
}

/* ===== mbedtls software SHA1 acceleration ===== */

static int mbed_sha1_find(sha_stubs_t *ss, uint32_t addr) {
    for (int i = 0; i < MBED_CTX_SLOTS; i++)
        if (ss->mbed_sha1[i].guest_addr == addr) return i;
    return -1;
}

static int mbed_sha1_alloc(sha_stubs_t *ss, uint32_t addr) {
    int idx = mbed_sha1_find(ss, addr);
    if (idx >= 0) return idx;
    for (int i = 0; i < MBED_CTX_SLOTS; i++) {
        if (ss->mbed_sha1[i].guest_addr == 0) {
            ss->mbed_sha1[i].guest_addr = addr;
            return i;
        }
    }
    return -1;
}

static void stub_mbedtls_sha1_starts(xtensa_cpu_t *cpu, void *ctx) {
    sha_stubs_t *ss = ctx;
    uint32_t ctx_addr = sha_arg(cpu, 0);
    int idx = mbed_sha1_alloc(ss, ctx_addr);
    if (idx >= 0)
        SHA1_Init(&ss->mbed_sha1[idx].ctx);
    sha_return(cpu, 0);
}

static void stub_mbedtls_sha1_update(xtensa_cpu_t *cpu, void *ctx) {
    sha_stubs_t *ss = ctx;
    uint32_t ctx_addr = sha_arg(cpu, 0);
    uint32_t data_addr = sha_arg(cpu, 1);
    uint32_t len = sha_arg(cpu, 2);
    int idx = mbed_sha1_find(ss, ctx_addr);
    if (idx >= 0 && len > 0) {
        uint32_t off = 0;
        while (off < len) {
            uint32_t chunk = len - off;
            const uint8_t *p = mem_get_ptr(cpu->mem, data_addr + off);
            if (p) {
                uint32_t page_rem = 0x1000 - ((data_addr + off) & 0xFFF);
                if (chunk > page_rem) chunk = page_rem;
                SHA1_Update(&ss->mbed_sha1[idx].ctx, p, chunk);
            } else {
                uint8_t tmp[1];
                tmp[0] = mem_read8(cpu->mem, data_addr + off);
                SHA1_Update(&ss->mbed_sha1[idx].ctx, tmp, 1);
                chunk = 1;
            }
            off += chunk;
        }
    }
    sha_return(cpu, 0);
}

static void stub_mbedtls_sha1_finish(xtensa_cpu_t *cpu, void *ctx) {
    sha_stubs_t *ss = ctx;
    uint32_t ctx_addr = sha_arg(cpu, 0);
    uint32_t out_addr = sha_arg(cpu, 1);
    int idx = mbed_sha1_find(ss, ctx_addr);
    if (idx >= 0) {
        unsigned char digest[20];
        SHA_CTX copy = ss->mbed_sha1[idx].ctx;
        SHA1_Final(digest, &copy);
        uint8_t *p = mem_get_ptr_w(cpu->mem, out_addr);
        if (p && ((out_addr & 0xFFF) <= 0xFEC))
            memcpy(p, digest, 20);
        else
            for (int i = 0; i < 20; i++)
                mem_write8(cpu->mem, out_addr + i, digest[i]);
    }
    sha_return(cpu, 0);
}

static void stub_mbedtls_sha1_free(xtensa_cpu_t *cpu, void *ctx) {
    sha_stubs_t *ss = ctx;
    uint32_t ctx_addr = sha_arg(cpu, 0);
    int idx = mbed_sha1_find(ss, ctx_addr);
    if (idx >= 0)
        ss->mbed_sha1[idx].guest_addr = 0;
    sha_return_void(cpu);
}

/* ===== mbedtls software MD5 acceleration ===== */

static int mbed_md5_find(sha_stubs_t *ss, uint32_t addr) {
    for (int i = 0; i < MBED_CTX_SLOTS; i++)
        if (ss->mbed_md5[i].guest_addr == addr) return i;
    return -1;
}

static int mbed_md5_alloc(sha_stubs_t *ss, uint32_t addr) {
    int idx = mbed_md5_find(ss, addr);
    if (idx >= 0) return idx;
    for (int i = 0; i < MBED_CTX_SLOTS; i++) {
        if (ss->mbed_md5[i].guest_addr == 0) {
            ss->mbed_md5[i].guest_addr = addr;
            return i;
        }
    }
    return -1;
}

static void stub_mbedtls_md5_starts(xtensa_cpu_t *cpu, void *ctx) {
    sha_stubs_t *ss = ctx;
    uint32_t ctx_addr = sha_arg(cpu, 0);
    int idx = mbed_md5_alloc(ss, ctx_addr);
    if (idx >= 0)
        MD5_Init(&ss->mbed_md5[idx].ctx);
    sha_return(cpu, 0);
}

static void stub_mbedtls_md5_update(xtensa_cpu_t *cpu, void *ctx) {
    sha_stubs_t *ss = ctx;
    uint32_t ctx_addr = sha_arg(cpu, 0);
    uint32_t data_addr = sha_arg(cpu, 1);
    uint32_t len = sha_arg(cpu, 2);
    int idx = mbed_md5_find(ss, ctx_addr);
    if (idx >= 0 && len > 0) {
        uint32_t off = 0;
        while (off < len) {
            uint32_t chunk = len - off;
            const uint8_t *p = mem_get_ptr(cpu->mem, data_addr + off);
            if (p) {
                uint32_t page_rem = 0x1000 - ((data_addr + off) & 0xFFF);
                if (chunk > page_rem) chunk = page_rem;
                MD5_Update(&ss->mbed_md5[idx].ctx, p, chunk);
            } else {
                uint8_t tmp = mem_read8(cpu->mem, data_addr + off);
                MD5_Update(&ss->mbed_md5[idx].ctx, &tmp, 1);
                chunk = 1;
            }
            off += chunk;
        }
    }
    sha_return(cpu, 0);
}

static void stub_mbedtls_md5_finish(xtensa_cpu_t *cpu, void *ctx) {
    sha_stubs_t *ss = ctx;
    uint32_t ctx_addr = sha_arg(cpu, 0);
    uint32_t out_addr = sha_arg(cpu, 1);
    int idx = mbed_md5_find(ss, ctx_addr);
    if (idx >= 0) {
        unsigned char digest[16];
        MD5_CTX copy = ss->mbed_md5[idx].ctx;
        MD5_Final(digest, &copy);
        uint8_t *p = mem_get_ptr_w(cpu->mem, out_addr);
        if (p && ((out_addr & 0xFFF) <= 0xFF0))
            memcpy(p, digest, 16);
        else
            for (int i = 0; i < 16; i++)
                mem_write8(cpu->mem, out_addr + i, digest[i]);
    }
    sha_return(cpu, 0);
}

static void stub_mbedtls_md5_free(xtensa_cpu_t *cpu, void *ctx) {
    sha_stubs_t *ss = ctx;
    uint32_t ctx_addr = sha_arg(cpu, 0);
    int idx = mbed_md5_find(ss, ctx_addr);
    if (idx >= 0)
        ss->mbed_md5[idx].guest_addr = 0;
    sha_return_void(cpu);
}

/* ===== stripped mbedTLS 3 SHA-256 discovery =====
 *
 * A native replacement is safe only when every API entry uses the expected
 * context ABI and every path feeds the same compression implementation. The
 * old accelerator selected four PCs from one Tasmota release. This matcher
 * instead fingerprints all six complete functions (including process and
 * platform zeroize), normalizing only decoded CALLn/L32R relocation bits,
 * then verifies every relocation's semantic target. The fingerprint follows
 * this exact mbedTLS implementation across links without following a firmware
 * name, entry point, or absolute application address. */

#define MBED_SHA256_FREE_DELTA       0x014u
#define MBED_SHA256_FREE_SIZE        0x012u
#define MBED_SHA256_STARTS_SIZE      0x06Bu
#define MBED_SHA256_PROCESS_DELTA    0x06Cu
#define MBED_SHA256_PROCESS_SIZE     0x876u
#define MBED_SHA256_UPDATE_DELTA     0x8E4u
#define MBED_SHA256_UPDATE_SIZE      0x08Du
#define MBED_SHA256_FINISH_DELTA     0x974u
#define MBED_SHA256_FINISH_SIZE      0x1BCu
#define MBED_SHA256_ZEROIZE_DELTA    0xB30u
#define MBED_SHA256_ZEROIZE_SIZE     0x018u
#define MBED_SHA256_FAMILY_SIZE      0xB48u

typedef struct {
    uint32_t free_entry;
    uint32_t starts_entry;
    uint32_t process_entry;
    uint32_t update_entry;
    uint32_t finish_entry;
    uint32_t zeroize_entry;
} mbed_sha256_family_t;

static bool mbed_sha256_l32r_value(xtensa_mem_t *mem, uint32_t pc,
                                  uint32_t *value_out) {
    uint32_t literal;
    return value_out &&
           firmware_xtensa_l32r_target(mem, pc, &literal) &&
           firmware_peek(mem, literal, 4u, value_out);
}

static bool mbed_sha256_l32r_is(xtensa_mem_t *mem, uint32_t pc,
                               uint32_t expected) {
    uint32_t value;
    return mbed_sha256_l32r_value(mem, pc, &value) && value == expected;
}

static bool mbed_sha256_call_is(xtensa_mem_t *mem, uint32_t pc,
                               uint32_t expected) {
    unsigned callinc;
    uint32_t target;
    return firmware_xtensa_call_target(mem, pc, &callinc, &target) &&
           callinc == 2u && target == expected;
}

static bool mbed_sha256_iv_matches(xtensa_mem_t *mem, uint32_t first_l32r,
                                  const uint32_t expected[8]) {
    for (unsigned i = 0u; i < 8u; i++)
        if (!mbed_sha256_l32r_is(
                    mem, first_l32r + i * 3u, expected[i]))
            return false;
    return true;
}

static bool mbed_sha256_family_matches(xtensa_mem_t *mem, uint32_t starts,
                                      mbed_sha256_family_t *family_out) {
    static const uint32_t sha224_iv[8] = {
        0xC1059ED8u, 0x367CD507u, 0x3070DD17u, 0xF70E5939u,
        0xFFC00B31u, 0x68581511u, 0x64F98FA7u, 0xBEFA4FA4u,
    };
    static const uint32_t sha256_iv[8] = {
        0x6A09E667u, 0xBB67AE85u, 0x3C6EF372u, 0xA54FF53Au,
        0x510E527Fu, 0x9B05688Cu, 0x1F83D9ABu, 0x5BE0CD19u,
    };
    static const uint32_t finish_bswap_l32rs[] = {
        0x03Cu, 0x05Du, 0x085u, 0x0A6u, 0x0C7u,
        0x0E8u, 0x109u, 0x12Au, 0x14Bu, 0x171u,
    };
    const uint32_t rom_bswap32 = 0x40064AE0u;

    if (!mem || !family_out || starts < MBED_SHA256_FREE_DELTA ||
        starts > UINT32_MAX - MBED_SHA256_FAMILY_SIZE)
        return false;

    mbed_sha256_family_t family = {
        .free_entry = starts - MBED_SHA256_FREE_DELTA,
        .starts_entry = starts,
        .process_entry = starts + MBED_SHA256_PROCESS_DELTA,
        .update_entry = starts + MBED_SHA256_UPDATE_DELTA,
        .finish_entry = starts + MBED_SHA256_FINISH_DELTA,
        .zeroize_entry = starts + MBED_SHA256_ZEROIZE_DELTA,
    };

    if (!firmware_xtensa_crc32_matches(
                mem, family.free_entry, MBED_SHA256_FREE_SIZE,
                0xBDEC3621u) ||
        !firmware_xtensa_crc32_matches(
                mem, family.starts_entry, MBED_SHA256_STARTS_SIZE,
                0x8BDD1D33u) ||
        !firmware_xtensa_crc32_matches(
                mem, family.process_entry, MBED_SHA256_PROCESS_SIZE,
                0x442D8BE9u) ||
        !firmware_xtensa_crc32_matches(
                mem, family.update_entry, MBED_SHA256_UPDATE_SIZE,
                0x547BE768u) ||
        !firmware_xtensa_crc32_matches(
                mem, family.finish_entry, MBED_SHA256_FINISH_SIZE,
                0xA54ABEFAu) ||
        !firmware_xtensa_crc32_matches(
                mem, family.zeroize_entry, MBED_SHA256_ZEROIZE_SIZE,
                0x2BBFDF6Cu))
        return false;

    if (!mbed_sha256_iv_matches(
                mem, family.starts_entry + 0x0Eu, sha224_iv) ||
        !mbed_sha256_iv_matches(
                mem, family.starts_entry + 0x50u, sha256_iv) ||
        !mbed_sha256_call_is(
                mem, family.free_entry + 0x0Cu, family.zeroize_entry) ||
        !mbed_sha256_call_is(
                mem, family.process_entry + 0x866u,
                family.zeroize_entry) ||
        !mbed_sha256_call_is(
                mem, family.update_entry + 0x3Fu,
                family.process_entry) ||
        !mbed_sha256_call_is(
                mem, family.update_entry + 0x56u,
                family.process_entry) ||
        !mbed_sha256_call_is(
                mem, family.finish_entry + 0x7Fu,
                family.process_entry) ||
        !mbed_sha256_call_is(
                mem, family.finish_entry + 0x191u,
                family.free_entry) ||
        !mbed_sha256_call_is(
                mem, family.finish_entry + 0x1B0u,
                family.process_entry))
        return false;

    uint32_t memcpy_entry;
    uint32_t update_memcpy_first;
    uint32_t update_memcpy_last;
    uint32_t memset_entry;
    uint32_t finish_memset_last;
    uint32_t helper_insn;
    if (!mbed_sha256_l32r_value(
                mem, family.process_entry + 0x12u, &memcpy_entry) ||
        !mbed_sha256_l32r_value(
                mem, family.update_entry + 0x30u,
                &update_memcpy_first) ||
        !mbed_sha256_l32r_value(
                mem, family.update_entry + 0x80u,
                &update_memcpy_last) ||
        update_memcpy_first != memcpy_entry ||
        update_memcpy_last != memcpy_entry ||
        !firmware_peek(mem, memcpy_entry, 3u, &helper_insn) ||
        !firmware_xtensa_is_entry(helper_insn) ||
        !mbed_sha256_l32r_value(
                mem, family.finish_entry + 0x2Au, &memset_entry) ||
        !mbed_sha256_l32r_value(
                mem, family.finish_entry + 0x1A4u,
                &finish_memset_last) ||
        finish_memset_last != memset_entry ||
        !firmware_peek(mem, memset_entry, 3u, &helper_insn) ||
        !firmware_xtensa_is_entry(helper_insn) ||
        !mbed_sha256_l32r_is(
                mem, family.process_entry + 0x41u, rom_bswap32))
        return false;
    for (size_t i = 0u;
         i < sizeof(finish_bswap_l32rs) /
                 sizeof(finish_bswap_l32rs[0]); i++)
        if (!mbed_sha256_l32r_is(
                    mem, family.finish_entry + finish_bswap_l32rs[i],
                    rom_bswap32))
            return false;

    uint32_t constants;
    uint32_t zeroize_slot;
    uint32_t zeroize_fn;
    if (!mbed_sha256_l32r_value(
                mem, family.process_entry + 0x4Cu, &constants) ||
        !firmware_crc32_matches(
                mem, constants, 64u * sizeof(uint32_t), 0x296FDE2Au) ||
        !mbed_sha256_l32r_value(
                mem, family.zeroize_entry + 0x07u, &zeroize_slot) ||
        !firmware_peek(mem, zeroize_slot, 4u, &zeroize_fn) ||
        zeroize_fn != memset_entry)
        return false;

    *family_out = family;
    return true;
}

static bool mbed_sha256_family_already_hooked(
        const sha_stubs_t *ss, uint32_t starts) {
    for (unsigned i = 0u; i < ss->firmware_sha256_count; i++)
        if (ss->firmware_sha256_starts[i] == starts)
            return true;
    return false;
}

static int mbed_sha256_scan_range(sha_stubs_t *ss,
                                  uint32_t start, uint32_t end) {
    static const uint8_t starts_prefix[] = {
        0x36, 0x41, 0x00, 0x8D, 0x02, 0x22,
        0xAF, 0x8C, 0xF6, 0x23, 0x3F, 0x16,
    };
    int hooked = 0;
    uint32_t cursor = start;
    uint32_t starts;
    while (ss->firmware_sha256_count < MBED_SHA256_FAMILY_MAX &&
           firmware_find_xtensa_crc32_body(
                ss->cpu->mem, cursor, end, starts_prefix,
                sizeof(starts_prefix), MBED_SHA256_STARTS_SIZE,
                0x8BDD1D33u, &starts)) {
        mbed_sha256_family_t family;
        if (!mbed_sha256_family_already_hooked(ss, starts) &&
            mbed_sha256_family_matches(ss->cpu->mem, starts, &family)) {
            struct {
                uint32_t addr;
                rom_stub_fn fn;
                const char *name;
            } hooks[] = {
                {family.free_entry, stub_mbedtls_sha256_free,
                 "mbedtls_sha256_free"},
                {family.starts_entry, stub_mbedtls_sha256_starts,
                 "mbedtls_sha256_starts"},
                {family.update_entry, stub_mbedtls_sha256_update,
                 "mbedtls_sha256_update"},
                {family.finish_entry, stub_mbedtls_sha256_finish,
                 "mbedtls_sha256_finish"},
            };
            bool installed = true;
            for (size_t i = 0u; i < sizeof(hooks) / sizeof(hooks[0]); i++)
                installed &= rom_stubs_register_exact_ctx(
                        ss->rom, hooks[i].addr, hooks[i].fn,
                        hooks[i].name, ss) == 0;
            if (installed) {
                ss->firmware_sha256_starts[
                    ss->firmware_sha256_count++] = starts;
                hooked += 4;
                fprintf(stderr,
                        "[sha] discovered mbedTLS SHA-256 family at "
                        "0x%08X\n", starts);
            }
        }
        cursor = starts + 1u;
    }
    return hooked;
}

int sha_stubs_hook_firmware(sha_stubs_t *ss) {
    if (!ss || !ss->cpu || !ss->cpu->mem)
        return 0;
    esp32_rom_stubs_t *rom = ss->cpu->pc_hook_ctx;
    if (!rom)
        return 0;
    ss->rom = rom;

    int hooked = mbed_sha256_scan_range(
            ss, ESP32_FIRMWARE_INSN_ADDR_LOW,
            ESP32_IRAM_INSN_ADDR_HIGH);
    hooked += mbed_sha256_scan_range(
            ss, ESP32_FLASH_INSN_ADDR_LOW,
            ESP32_FLASH_INSN_ADDR_HIGH);
    return hooked;
}

int sha_stubs_hook_symbols(sha_stubs_t *ss, const elf_symbols_t *syms) {
    if (!ss || !syms) return 0;

    esp32_rom_stubs_t *rom = ss->cpu->pc_hook_ctx;
    if (!rom) return 0;
    ss->rom = rom;

    int hooked = 0;
    struct {
        const char *name;
        rom_stub_fn fn;
    } hooks[] = {
        { "sha_hal_hash_block",        stub_sha_hal_hash_block },
        { "sha_hal_read_digest",       stub_sha_hal_read_digest },
        { "sha_hal_wait_idle",         stub_sha_hal_wait_idle },
        { "esp_sha_lock_engine",       stub_sha_lock_engine },
        { "esp_sha_try_lock_engine",   stub_sha_try_lock_engine },
        { "esp_sha_unlock_engine",     stub_sha_unlock_engine },
        { "esp_sha_lock_memory_block", stub_sha_lock_memory },
        { "esp_sha_unlock_memory_block", stub_sha_unlock_memory },
        { NULL, NULL }
    };

    for (int i = 0; hooks[i].name; i++) {
        uint32_t addr;
        if (elf_symbols_find(syms, hooks[i].name, &addr) == 0) {
            rom_stubs_register_ctx(rom, addr, hooks[i].fn, hooks[i].name, ss);
            hooked++;
        }
    }

    /* mbedtls software SHA acceleration — works with any ESP32 firmware.
     * Replaces interpreted mbedtls SHA with native OpenSSL. */
    struct {
        const char *name;
        rom_stub_fn fn;
    } mbed_hooks[] = {
        /* SHA-256 (and SHA-224) */
        { "mbedtls_sha256_starts_ret",  stub_mbedtls_sha256_starts },
        { "mbedtls_sha256_starts",      stub_mbedtls_sha256_starts },
        { "mbedtls_sha256_update_ret",  stub_mbedtls_sha256_update },
        { "mbedtls_sha256_update",      stub_mbedtls_sha256_update },
        { "mbedtls_sha256_finish_ret",  stub_mbedtls_sha256_finish },
        { "mbedtls_sha256_finish",      stub_mbedtls_sha256_finish },
        { "mbedtls_sha256_free",        stub_mbedtls_sha256_free },
        /* SHA-1 */
        { "mbedtls_sha1_starts_ret",    stub_mbedtls_sha1_starts },
        { "mbedtls_sha1_starts",        stub_mbedtls_sha1_starts },
        { "mbedtls_sha1_update_ret",    stub_mbedtls_sha1_update },
        { "mbedtls_sha1_update",        stub_mbedtls_sha1_update },
        { "mbedtls_sha1_finish_ret",    stub_mbedtls_sha1_finish },
        { "mbedtls_sha1_finish",        stub_mbedtls_sha1_finish },
        { "mbedtls_sha1_free",          stub_mbedtls_sha1_free },
        /* MD5 */
        { "mbedtls_md5_starts_ret",     stub_mbedtls_md5_starts },
        { "mbedtls_md5_starts",         stub_mbedtls_md5_starts },
        { "mbedtls_md5_update_ret",     stub_mbedtls_md5_update },
        { "mbedtls_md5_update",         stub_mbedtls_md5_update },
        { "mbedtls_md5_finish_ret",     stub_mbedtls_md5_finish },
        { "mbedtls_md5_finish",         stub_mbedtls_md5_finish },
        { "mbedtls_md5_free",           stub_mbedtls_md5_free },
        { NULL, NULL }
    };

    for (int i = 0; mbed_hooks[i].name; i++) {
        uint32_t addr;
        if (elf_symbols_find(syms, mbed_hooks[i].name, &addr) == 0) {
            rom_stubs_register_ctx(rom, addr, mbed_hooks[i].fn, mbed_hooks[i].name, ss);
            hooked++;
        }
    }

    return hooked;
}
