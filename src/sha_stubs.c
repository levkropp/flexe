#include "sha_stubs.h"
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

/* SHA type constants (matches esp_sha_type enum) */
#define SHA_TYPE_1    0
#define SHA_TYPE_256  1
#define SHA_TYPE_384  2
#define SHA_TYPE_512  3

/* ESP32 SHA peripheral base: 0x3FF03000
 * SHA_TEXT registers: 0x00-0x7C (32 x uint32_t) — message block AND digest state
 * Control registers: 0x80-0xBC (START/CONTINUE/LOAD/BUSY per engine)
 * The TEXT registers serve dual purpose: the firmware writes the message block
 * before START/CONTINUE, and reads the digest state after completion.
 * For state restore, the firmware writes state to TEXT then issues LOAD.
 * Our hooks intercept sha_hal_hash_block (so START/CONTINUE never fire),
 * but the firmware's inlined sha_ll_write_digest writes state to TEXT
 * before calling sha_hal_hash_block(is_first=0). We must capture those. */
#define SHA_PERIPH_BASE  0x3FF03000u
#define SHA_PERIPH_PAGE  3  /* (0x3FF03000 - 0x3FF00000) / 4096 */

/* ESP32 has 3 SHA hardware engines; SHA-384 and SHA-512 share one */
#define SHA_NUM_ENGINES 3

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

    /* SHA_TEXT register backing store (32 x uint32_t).
     * Mirrors the hardware SHA_TEXT registers at 0x3FF03000.
     * Used for firmware MMIO reads/writes (state save/restore). */
    uint32_t sha_text[32];

    /* Per-engine internal state (separate from SHA_TEXT, like real hardware).
     * On ESP32, each SHA engine has independent internal state registers.
     * The firmware relies on state persisting between consecutive blocks
     * of the same type without explicit save/restore via SHA_TEXT. */
    uint32_t sha1_engine[5];     /* SHA-1 engine internal state */
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
    struct {
        uint32_t    guest_addr;
        SHA_CTX     ctx;
    } mbed_sha1[MBED_CTX_SLOTS];
};

/* ===== SHA peripheral MMIO handler ===== */

static uint32_t sha_mmio_read(void *ctx, uint32_t addr) {
    sha_stubs_t *ss = ctx;
    uint32_t off = addr - SHA_PERIPH_BASE;

    if (off < 0x80) {
        /* SHA_TEXT registers (32 words) */
        return ss->sha_text[off / 4];
    }

    /* Control/status registers */
    switch (off) {
    case 0x8C: case 0x9C: case 0xAC: case 0xBC:
        return 0; /* SHA_*_BUSY: always idle */
    default:
        return 0;
    }
}

static void sha_mmio_write(void *ctx, uint32_t addr, uint32_t val) {
    sha_stubs_t *ss = ctx;
    uint32_t off = addr - SHA_PERIPH_BASE;

    if (off < 0x80) {
        /* SHA_TEXT registers — firmware writes state here via sha_ll_write_digest */
        ss->sha_text[off / 4] = val;
        return;
    }

    /* Control registers: handle LOAD commands.
     * On ESP32, LOAD copies SHA_TEXT into engine internal state registers.
     * Offsets: SHA1_LOAD=0x88, SHA256_LOAD=0x98, SHA384_LOAD=0xA8, SHA512_LOAD=0xB8 */
    switch (off) {
    case 0x88: /* SHA1_LOAD */
        for (int i = 0; i < 5; i++)
            ss->sha1_engine[i] = ss->sha_text[i];
        break;
    case 0x98: /* SHA256_LOAD */
        for (int i = 0; i < 8; i++)
            ss->sha256_engine[i] = ss->sha_text[i];
        break;
    case 0xA8: /* SHA384_LOAD */
    case 0xB8: /* SHA512_LOAD */
        for (int i = 0; i < 8; i++)
            ss->sha512_engine[i] = ((uint64_t)ss->sha_text[i*2] << 32) |
                                    ss->sha_text[i*2+1];
        break;
    default:
        break; /* START/CONTINUE/BUSY — no-op (our hooks handle processing) */
    }
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
        cpu->pc = ar_read(cpu, 0);
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
        cpu->pc = ar_read(cpu, 0);
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

sha_stubs_t *sha_stubs_create(xtensa_cpu_t *cpu) {
    sha_stubs_t *ss = calloc(1, sizeof(*ss));
    if (!ss) return NULL;
    ss->cpu = cpu;
    ss->current_type = -1;

    /* Verify SHA-256 via OpenSSL is correct */
    if (!sha256_self_test()) {
        fprintf(stderr, "[sha] FATAL: SHA-256 self-test failed!\n");
        free(ss);
        return NULL;
    }

    /* Register MMIO handler for SHA peripheral page (0x3FF03000).
     * This overrides the DPORT handler for page 3, capturing firmware's
     * direct register writes to SHA_TEXT (used for state save/restore). */
    mem_register_mmio(cpu->mem, SHA_PERIPH_PAGE,
                      sha_mmio_read, sha_mmio_write, ss);

    return ss;
}

void sha_stubs_destroy(sha_stubs_t *ss) {
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
