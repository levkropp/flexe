#include "sha_stubs.h"
#include "rom_stubs.h"
#include "memory.h"
#include <stdlib.h>
#include <string.h>

/* SHA type constants (matches esp_sha_type enum) */
#define SHA_TYPE_1    0
#define SHA_TYPE_256  1
#define SHA_TYPE_384  2
#define SHA_TYPE_512  3

struct sha_stubs {
    xtensa_cpu_t      *cpu;
    esp32_rom_stubs_t *rom;

    /* SHA-1 / SHA-256 state (32-bit words) */
    uint32_t h32[8];

    /* SHA-384 / SHA-512 state (64-bit words) */
    uint64_t h64[8];

    int current_type;   /* last sha_type used */
};

/* ===== Calling convention helpers ===== */

static uint32_t sha_arg(xtensa_cpu_t *cpu, int n) {
    int ci = XT_PS_CALLINC(cpu->ps);
    return ar_read(cpu, ci * 4 + 2 + n);
}

static void sha_return_void(xtensa_cpu_t *cpu) {
    int ci = XT_PS_CALLINC(cpu->ps);
    if (ci > 0) {
        uint32_t a0 = ar_read(cpu, ci * 4);
        cpu->pc = (cpu->pc & 0xC0000000u) | (a0 & 0x3FFFFFFFu);
        XT_PS_SET_CALLINC(cpu->ps, 0);
    } else {
        cpu->pc = ar_read(cpu, 0);
    }
}

/* ===== SHA-256 constants and compression ===== */

static const uint32_t K256[64] = {
    0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5,
    0x3956c25b, 0x59f111f1, 0x923f82a4, 0xab1c5ed5,
    0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3,
    0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174,
    0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc,
    0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da,
    0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7,
    0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967,
    0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13,
    0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85,
    0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3,
    0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070,
    0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5,
    0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
    0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208,
    0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2,
};

#define ROR32(x, n) (((x) >> (n)) | ((x) << (32 - (n))))
#define CH32(x, y, z)  (((x) & (y)) ^ (~(x) & (z)))
#define MAJ32(x, y, z) (((x) & (y)) ^ ((x) & (z)) ^ ((y) & (z)))
#define BSIG0_32(x) (ROR32(x,2) ^ ROR32(x,13) ^ ROR32(x,22))
#define BSIG1_32(x) (ROR32(x,6) ^ ROR32(x,11) ^ ROR32(x,25))
#define SSIG0_32(x) (ROR32(x,7) ^ ROR32(x,18) ^ ((x) >> 3))
#define SSIG1_32(x) (ROR32(x,17) ^ ROR32(x,19) ^ ((x) >> 10))

/* Single-block SHA-256 compression. block[] must be 16 big-endian uint32_t. */
static void sha256_compress(uint32_t state[8], const uint32_t block[16]) {
    uint32_t W[64];
    for (int i = 0; i < 16; i++)
        W[i] = block[i];
    for (int i = 16; i < 64; i++)
        W[i] = SSIG1_32(W[i-2]) + W[i-7] + SSIG0_32(W[i-15]) + W[i-16];

    uint32_t a = state[0], b = state[1], c = state[2], d = state[3];
    uint32_t e = state[4], f = state[5], g = state[6], h = state[7];

    for (int i = 0; i < 64; i++) {
        uint32_t T1 = h + BSIG1_32(e) + CH32(e, f, g) + K256[i] + W[i];
        uint32_t T2 = BSIG0_32(a) + MAJ32(a, b, c);
        h = g; g = f; f = e; e = d + T1;
        d = c; c = b; b = a; a = T1 + T2;
    }

    state[0] += a; state[1] += b; state[2] += c; state[3] += d;
    state[4] += e; state[5] += f; state[6] += g; state[7] += h;
}

/* ===== SHA-1 constants and compression ===== */

#define ROL32(x, n) (((x) << (n)) | ((x) >> (32 - (n))))

/* Single-block SHA-1 compression. block[] must be 16 big-endian uint32_t. */
static void sha1_compress(uint32_t state[5], const uint32_t block[16]) {
    uint32_t W[80];
    for (int i = 0; i < 16; i++)
        W[i] = block[i];
    for (int i = 16; i < 80; i++)
        W[i] = ROL32(W[i-3] ^ W[i-8] ^ W[i-14] ^ W[i-16], 1);

    uint32_t a = state[0], b = state[1], c = state[2];
    uint32_t d = state[3], e = state[4];

    for (int i = 0; i < 80; i++) {
        uint32_t f, k;
        if (i < 20) {
            f = (b & c) | (~b & d);
            k = 0x5A827999;
        } else if (i < 40) {
            f = b ^ c ^ d;
            k = 0x6ED9EBA1;
        } else if (i < 60) {
            f = (b & c) | (b & d) | (c & d);
            k = 0x8F1BBCDC;
        } else {
            f = b ^ c ^ d;
            k = 0xCA62C1D6;
        }
        uint32_t temp = ROL32(a, 5) + f + e + k + W[i];
        e = d; d = c; c = ROL32(b, 30); b = a; a = temp;
    }

    state[0] += a; state[1] += b; state[2] += c;
    state[3] += d; state[4] += e;
}

/* ===== SHA-512 constants and compression ===== */

static const uint64_t K512[80] = {
    0x428a2f98d728ae22ULL, 0x7137449123ef65cdULL,
    0xb5c0fbcfec4d3b2fULL, 0xe9b5dba58189dbbcULL,
    0x3956c25bf348b538ULL, 0x59f111f1b605d019ULL,
    0x923f82a4af194f9bULL, 0xab1c5ed5da6d8118ULL,
    0xd807aa98a3030242ULL, 0x12835b0145706fbeULL,
    0x243185be4ee4b28cULL, 0x550c7dc3d5ffb4e2ULL,
    0x72be5d74f27b896fULL, 0x80deb1fe3b1696b1ULL,
    0x9bdc06a725c71235ULL, 0xc19bf174cf692694ULL,
    0xe49b69c19ef14ad2ULL, 0xefbe4786384f25e3ULL,
    0x0fc19dc68b8cd5b5ULL, 0x240ca1cc77ac9c65ULL,
    0x2de92c6f592b0275ULL, 0x4a7484aa6ea6e483ULL,
    0x5cb0a9dcbd41fbd4ULL, 0x76f988da831153b5ULL,
    0x983e5152ee66dfabULL, 0xa831c66d2db43210ULL,
    0xb00327c898fb213fULL, 0xbf597fc7beef0ee4ULL,
    0xc6e00bf33da88fc2ULL, 0xd5a79147930aa725ULL,
    0x06ca6351e003826fULL, 0x142929670a0e6e70ULL,
    0x27b70a8546d22ffcULL, 0x2e1b21385c26c926ULL,
    0x4d2c6dfc5ac42aedULL, 0x53380d139d95b3dfULL,
    0x650a73548baf63deULL, 0x766a0abb3c77b2a8ULL,
    0x81c2c92e47edaee6ULL, 0x92722c851482353bULL,
    0xa2bfe8a14cf10364ULL, 0xa81a664bbc423001ULL,
    0xc24b8b70d0f89791ULL, 0xc76c51a30654be30ULL,
    0xd192e819d6ef5218ULL, 0xd69906245565a910ULL,
    0xf40e35855771202aULL, 0x106aa07032bbd1b8ULL,
    0x19a4c116b8d2d0c8ULL, 0x1e376c085141ab53ULL,
    0x2748774cdf8eeb99ULL, 0x34b0bcb5e19b48a8ULL,
    0x391c0cb3c5c95a63ULL, 0x4ed8aa4ae3418acbULL,
    0x5b9cca4f7763e373ULL, 0x682e6ff3d6b2b8a3ULL,
    0x748f82ee5defb2fcULL, 0x78a5636f43172f60ULL,
    0x84c87814a1f0ab72ULL, 0x8cc702081a6439ecULL,
    0x90befffa23631e28ULL, 0xa4506cebde82bde9ULL,
    0xbef9a3f7b2c67915ULL, 0xc67178f2e372532bULL,
    0xca273eceea26619cULL, 0xd186b8c721c0c207ULL,
    0xeada7dd6cde0eb1eULL, 0xf57d4f7fee6ed178ULL,
    0x06f067aa72176fbaULL, 0x0a637dc5a2c898a6ULL,
    0x113f9804bef90daeULL, 0x1b710b35131c471bULL,
    0x28db77f523047d84ULL, 0x32caab7b40c72493ULL,
    0x3c9ebe0a15c9bebcULL, 0x431d67c49c100d4cULL,
    0x4cc5d4becb3e42b6ULL, 0x597f299cfc657e2aULL,
    0x5fcb6fab3ad6faecULL, 0x6c44198c4a475817ULL,
};

#define ROR64(x, n) (((x) >> (n)) | ((x) << (64 - (n))))
#define CH64(x, y, z)  (((x) & (y)) ^ (~(x) & (z)))
#define MAJ64(x, y, z) (((x) & (y)) ^ ((x) & (z)) ^ ((y) & (z)))
#define BSIG0_64(x) (ROR64(x,28) ^ ROR64(x,34) ^ ROR64(x,39))
#define BSIG1_64(x) (ROR64(x,14) ^ ROR64(x,18) ^ ROR64(x,41))
#define SSIG0_64(x) (ROR64(x,1)  ^ ROR64(x,8)  ^ ((x) >> 7))
#define SSIG1_64(x) (ROR64(x,19) ^ ROR64(x,61) ^ ((x) >> 6))

/* Single-block SHA-512 compression. block[] must be 16 big-endian uint64_t.
 * We receive them as 32 uint32_t (high word first for each pair). */
static void sha512_compress(uint64_t state[8], const uint32_t block32[32]) {
    uint64_t W[80];
    for (int i = 0; i < 16; i++)
        W[i] = ((uint64_t)block32[i*2] << 32) | block32[i*2+1];
    for (int i = 16; i < 80; i++)
        W[i] = SSIG1_64(W[i-2]) + W[i-7] + SSIG0_64(W[i-15]) + W[i-16];

    uint64_t a = state[0], b = state[1], c = state[2], d = state[3];
    uint64_t e = state[4], f = state[5], g = state[6], h = state[7];

    for (int i = 0; i < 80; i++) {
        uint64_t T1 = h + BSIG1_64(e) + CH64(e, f, g) + K512[i] + W[i];
        uint64_t T2 = BSIG0_64(a) + MAJ64(a, b, c);
        h = g; g = f; f = e; e = d + T1;
        d = c; c = b; b = a; a = T1 + T2;
    }

    state[0] += a; state[1] += b; state[2] += c; state[3] += d;
    state[4] += e; state[5] += f; state[6] += g; state[7] += h;
}

/* ===== Initial hash values ===== */

static const uint32_t SHA1_IV[5] = {
    0x67452301, 0xefcdab89, 0x98badcfe, 0x10325476, 0xc3d2e1f0
};

static const uint32_t SHA256_IV[8] = {
    0x6a09e667, 0xbb67ae85, 0x3c6ef372, 0xa54ff53a,
    0x510e527f, 0x9b05688c, 0x1f83d9ab, 0x5be0cd19
};

static const uint64_t SHA384_IV[8] = {
    0xcbbb9d5dc1059ed8ULL, 0x629a292a367cd507ULL,
    0x9159015a3070dd17ULL, 0x152fecd8f70e5939ULL,
    0x67332667ffc00b31ULL, 0x8eb44a8768581511ULL,
    0xdb0c2e0d64f98fa7ULL, 0x47b5481dbefa4fa4ULL
};

static const uint64_t SHA512_IV[8] = {
    0x6a09e667f3bcc908ULL, 0xbb67ae8584caa73bULL,
    0x3c6ef372fe94f82bULL, 0xa54ff53a5f1d36f1ULL,
    0x510e527fade682d1ULL, 0x9b05688c2b3e6c1fULL,
    0x1f83d9abfb41bd6bULL, 0x5be0cd19137e2179ULL
};

/* ===== HAL stub implementations ===== */

static inline uint32_t bswap32(uint32_t x) {
    return ((x >> 24) & 0xff) | ((x >> 8) & 0xff00) |
           ((x << 8) & 0xff0000) | ((x << 24) & 0xff000000u);
}

/*
 * sha_hal_hash_block(sha_type, data_block, block_word_len, is_first_block)
 *
 * Reads block_word_len words from data_block in emulator memory,
 * byte-swaps each word to big-endian, then runs SHA compression.
 */
static void stub_sha_hal_hash_block(xtensa_cpu_t *cpu, void *ctx) {
    sha_stubs_t *ss = ctx;
    uint32_t sha_type       = sha_arg(cpu, 0);
    uint32_t data_ptr       = sha_arg(cpu, 1);
    uint32_t block_word_len = sha_arg(cpu, 2);
    uint32_t is_first       = sha_arg(cpu, 3);

    ss->current_type = (int)sha_type;

    /* Read block from emulator memory, byte-swap to big-endian */
    uint32_t block[32];
    if (block_word_len > 32) block_word_len = 32;
    for (uint32_t i = 0; i < block_word_len; i++)
        block[i] = bswap32(mem_read32(cpu->mem, data_ptr + i * 4));

    switch (sha_type) {
    case SHA_TYPE_1:
        if (is_first)
            memcpy(ss->h32, SHA1_IV, sizeof(SHA1_IV));
        sha1_compress(ss->h32, block);
        break;

    case SHA_TYPE_256:
        if (is_first)
            memcpy(ss->h32, SHA256_IV, sizeof(SHA256_IV));
        sha256_compress(ss->h32, block);
        break;

    case SHA_TYPE_384:
        if (is_first)
            memcpy(ss->h64, SHA384_IV, sizeof(SHA384_IV));
        sha512_compress(ss->h64, block);
        break;

    case SHA_TYPE_512:
        if (is_first)
            memcpy(ss->h64, SHA512_IV, sizeof(SHA512_IV));
        sha512_compress(ss->h64, block);
        break;
    }

    sha_return_void(cpu);
}

/*
 * sha_hal_read_digest(sha_type, digest_state, digest_word_len)
 *
 * Writes current state to digest_state in emulator memory.
 * For SHA-1/256: state is uint32_t words, written directly (big-endian).
 * For SHA-384/512: state is uint64_t, each stored as two uint32_t (high, low).
 */
static void stub_sha_hal_read_digest(xtensa_cpu_t *cpu, void *ctx) {
    sha_stubs_t *ss = ctx;
    uint32_t sha_type        = sha_arg(cpu, 0);
    uint32_t digest_ptr      = sha_arg(cpu, 1);
    uint32_t digest_word_len = sha_arg(cpu, 2);

    switch (sha_type) {
    case SHA_TYPE_1:
    case SHA_TYPE_256: {
        uint32_t max = (sha_type == SHA_TYPE_1) ? 5 : 8;
        if (digest_word_len > max) digest_word_len = max;
        for (uint32_t i = 0; i < digest_word_len; i++)
            mem_write32(cpu->mem, digest_ptr + i * 4, ss->h32[i]);
        break;
    }
    case SHA_TYPE_384:
    case SHA_TYPE_512: {
        /* 8 x uint64_t stored as pairs of uint32_t (high word first) */
        uint32_t max = 16;
        if (digest_word_len > max) digest_word_len = max;
        for (uint32_t i = 0; i < digest_word_len; i += 2) {
            uint64_t val = ss->h64[i / 2];
            mem_write32(cpu->mem, digest_ptr + i * 4, (uint32_t)(val >> 32));
            if (i + 1 < digest_word_len)
                mem_write32(cpu->mem, digest_ptr + (i + 1) * 4, (uint32_t)val);
        }
        break;
    }
    }

    sha_return_void(cpu);
}

/* sha_hal_wait_idle() — hardware is always idle in emulation */
static void stub_sha_hal_wait_idle(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    sha_return_void(cpu);
}

/* ===== Public API ===== */

sha_stubs_t *sha_stubs_create(xtensa_cpu_t *cpu) {
    sha_stubs_t *ss = calloc(1, sizeof(*ss));
    if (!ss) return NULL;
    ss->cpu = cpu;
    ss->current_type = -1;
    return ss;
}

void sha_stubs_destroy(sha_stubs_t *ss) {
    free(ss);
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
        { "sha_hal_hash_block",  stub_sha_hal_hash_block },
        { "sha_hal_read_digest", stub_sha_hal_read_digest },
        { "sha_hal_wait_idle",   stub_sha_hal_wait_idle },
        { NULL, NULL }
    };

    for (int i = 0; hooks[i].name; i++) {
        uint32_t addr;
        if (elf_symbols_find(syms, hooks[i].name, &addr) == 0) {
            rom_stubs_register_ctx(rom, addr, hooks[i].fn, hooks[i].name, ss);
            hooked++;
        }
    }

    return hooked;
}
