#include "sha_stubs.h"
#include "test_helpers.h"
#include "rom_stubs.h"
#include "aes_stubs.h"
#include "mpi_stubs.h"
#include "peripherals.h"
#include "gdma.h"

#define AES_BASE_ADDR 0x3FF01000u

static uint32_t crypto_le32(const uint8_t *p) {
    return (uint32_t)p[0] |
           ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) |
           ((uint32_t)p[3] << 24);
}

static void raw_aes_transform(xtensa_mem_t *mem, const uint8_t *key,
                              int key_bytes, int decrypt,
                              const uint8_t input[16], uint8_t output[16]) {
    uint32_t mode = (uint32_t)(key_bytes / 8 - 2);
    if (decrypt)
        mode |= 4u;

    for (int i = 0; i < key_bytes / 4; i++)
        mem_write32(mem, AES_BASE_ADDR + 0x10u + (uint32_t)i * 4,
                    crypto_le32(key + i * 4));
    for (int i = 0; i < 4; i++)
        mem_write32(mem, AES_BASE_ADDR + 0x30u + (uint32_t)i * 4,
                    crypto_le32(input + i * 4));
    mem_write32(mem, AES_BASE_ADDR + 0x08u, mode);
    mem_write32(mem, AES_BASE_ADDR + 0x00u, 1);

    for (int i = 0; i < 4; i++) {
        uint32_t word = mem_read32(mem, AES_BASE_ADDR + 0x30u + (uint32_t)i * 4);
        output[i * 4] = (uint8_t)word;
        output[i * 4 + 1] = (uint8_t)(word >> 8);
        output[i * 4 + 2] = (uint8_t)(word >> 16);
        output[i * 4 + 3] = (uint8_t)(word >> 24);
    }
}

static void assert_aes_block(const uint8_t actual[16],
                             const uint8_t expected[16]) {
    for (int i = 0; i < 4; i++)
        ASSERT_EQ(crypto_le32(actual + i * 4),
                  crypto_le32(expected + i * 4));
}

static void check_raw_aes_vector(const uint8_t *key, int key_bytes,
                                 const uint8_t plain[16],
                                 const uint8_t cipher[16]) {
    xtensa_cpu_t cpu;
    setup(&cpu);
    aes_stubs_t *aes = aes_stubs_create(&cpu);
    ASSERT_TRUE(aes != NULL);
    ASSERT_EQ(mem_read32(cpu.mem, AES_BASE_ADDR + 0x04u), 1);

    uint8_t output[16];
    raw_aes_transform(cpu.mem, key, key_bytes, 0, plain, output);
    assert_aes_block(output, cipher);
    raw_aes_transform(cpu.mem, key, key_bytes, 1, cipher, output);
    assert_aes_block(output, plain);

    aes_stubs_destroy(aes);
    teardown(&cpu);
}

TEST(raw_aes_128_encrypt_decrypt) {
    static const uint8_t key[16] = {
        0x00,0x01,0x02,0x03, 0x04,0x05,0x06,0x07,
        0x08,0x09,0x0a,0x0b, 0x0c,0x0d,0x0e,0x0f,
    };
    static const uint8_t plain[16] = {
        0x00,0x11,0x22,0x33, 0x44,0x55,0x66,0x77,
        0x88,0x99,0xaa,0xbb, 0xcc,0xdd,0xee,0xff,
    };
    static const uint8_t cipher[16] = {
        0x69,0xc4,0xe0,0xd8, 0x6a,0x7b,0x04,0x30,
        0xd8,0xcd,0xb7,0x80, 0x70,0xb4,0xc5,0x5a,
    };
    check_raw_aes_vector(key, sizeof(key), plain, cipher);
}

TEST(raw_aes_192_encrypt_decrypt) {
    static const uint8_t key[24] = {
        0x00,0x01,0x02,0x03, 0x04,0x05,0x06,0x07,
        0x08,0x09,0x0a,0x0b, 0x0c,0x0d,0x0e,0x0f,
        0x10,0x11,0x12,0x13, 0x14,0x15,0x16,0x17,
    };
    static const uint8_t plain[16] = {
        0x00,0x11,0x22,0x33, 0x44,0x55,0x66,0x77,
        0x88,0x99,0xaa,0xbb, 0xcc,0xdd,0xee,0xff,
    };
    static const uint8_t cipher[16] = {
        0xdd,0xa9,0x7c,0xa4, 0x86,0x4c,0xdf,0xe0,
        0x6e,0xaf,0x70,0xa0, 0xec,0x0d,0x71,0x91,
    };
    check_raw_aes_vector(key, sizeof(key), plain, cipher);
}

TEST(raw_aes_256_encrypt_decrypt) {
    static const uint8_t key[32] = {
        0x00,0x01,0x02,0x03, 0x04,0x05,0x06,0x07,
        0x08,0x09,0x0a,0x0b, 0x0c,0x0d,0x0e,0x0f,
        0x10,0x11,0x12,0x13, 0x14,0x15,0x16,0x17,
        0x18,0x19,0x1a,0x1b, 0x1c,0x1d,0x1e,0x1f,
    };
    static const uint8_t plain[16] = {
        0x00,0x11,0x22,0x33, 0x44,0x55,0x66,0x77,
        0x88,0x99,0xaa,0xbb, 0xcc,0xdd,0xee,0xff,
    };
    static const uint8_t cipher[16] = {
        0x8e,0xa2,0xb7,0xca, 0x51,0x67,0x45,0xbf,
        0xea,0xfc,0x49,0x90, 0x4b,0x49,0x60,0x89,
    };
    check_raw_aes_vector(key, sizeof(key), plain, cipher);
}

#define RSA_BASE_ADDR 0x3FF02000u

TEST(raw_mpi_montgomery_multiply) {
    xtensa_cpu_t cpu;
    setup(&cpu);
    esp32_periph_t *periph = periph_create(cpu.mem);
    periph_attach_cpus(periph, &cpu, NULL);
    mpi_stubs_t *mpi = mpi_stubs_create(&cpu);
    mpi_stubs_set_peripheral(mpi, periph);

    ASSERT_EQ(mem_read32(cpu.mem, RSA_BASE_ADDR + 0x818u), 1);
    periph_intr_matrix_set(periph, 0, 5, 51);
    mem_write32(cpu.mem, RSA_BASE_ADDR + 0x814u, 1);

    /* Mode 0 is a 512-bit Montgomery operation.  For M=19, R=2^512
     * mod M is 9 and R^-1 is 17, so 5*7*R^-1 mod M is 6. */
    mem_write32(cpu.mem, RSA_BASE_ADDR + 0x000u, 19);
    mem_write32(cpu.mem, RSA_BASE_ADDR + 0x600u, 5);
    mem_write32(cpu.mem, RSA_BASE_ADDR + 0x200u, 7);
    mem_write32(cpu.mem, RSA_BASE_ADDR + 0x80Cu, 0);
    mem_write32(cpu.mem, RSA_BASE_ADDR + 0x810u, 1);

    ASSERT_EQ(mem_read32(cpu.mem, RSA_BASE_ADDR + 0x200u), 6);
    ASSERT_EQ(mem_read32(cpu.mem, RSA_BASE_ADDR + 0x814u), 1);
    ASSERT_TRUE(cpu.interrupt & (1u << 5));
    mem_write32(cpu.mem, RSA_BASE_ADDR + 0x814u, 1);
    ASSERT_EQ(mem_read32(cpu.mem, RSA_BASE_ADDR + 0x814u), 0);
    ASSERT_FALSE(cpu.interrupt & (1u << 5));

    mpi_stubs_destroy(mpi);
    periph_destroy(periph);
    teardown(&cpu);
}

TEST(raw_mpi_plain_multiply) {
    xtensa_cpu_t cpu;
    setup(&cpu);
    mpi_stubs_t *mpi = mpi_stubs_create(&cpu);

    /* Mode 9 is a 16-word by 16-word multiply with a 32-word result.
     * The second input occupies the upper half of the Z block. */
    mem_write32(cpu.mem, RSA_BASE_ADDR + 0x600u, 0xFFFFFFFFu);
    mem_write32(cpu.mem, RSA_BASE_ADDR + 0x604u, 0xFFFFFFFFu);
    mem_write32(cpu.mem, RSA_BASE_ADDR + 0x200u + 16u * 4, 2);
    mem_write32(cpu.mem, RSA_BASE_ADDR + 0x80Cu, 9);
    mem_write32(cpu.mem, RSA_BASE_ADDR + 0x810u, 1);

    ASSERT_EQ(mem_read32(cpu.mem, RSA_BASE_ADDR + 0x200u), 0xFFFFFFFEu);
    ASSERT_EQ(mem_read32(cpu.mem, RSA_BASE_ADDR + 0x204u), 0xFFFFFFFFu);
    ASSERT_EQ(mem_read32(cpu.mem, RSA_BASE_ADDR + 0x208u), 1);
    ASSERT_EQ(mem_read32(cpu.mem, RSA_BASE_ADDR + 0x20Cu), 0);

    mpi_stubs_destroy(mpi);
    teardown(&cpu);
}

TEST(raw_mpi_modular_exponentiation) {
    xtensa_cpu_t cpu;
    setup(&cpu);
    mpi_stubs_t *mpi = mpi_stubs_create(&cpu);

    /* Classic RSA example: 4^13 mod 497 = 445.  Mode 0 selects the
     * original ESP32 accelerator's smallest, 512-bit (16-word) size. */
    mem_write32(cpu.mem, RSA_BASE_ADDR + 0x600u, 4);
    mem_write32(cpu.mem, RSA_BASE_ADDR + 0x400u, 13);
    mem_write32(cpu.mem, RSA_BASE_ADDR + 0x000u, 497);
    mem_write32(cpu.mem, RSA_BASE_ADDR + 0x804u, 0);
    mem_write32(cpu.mem, RSA_BASE_ADDR + 0x808u, 1);

    ASSERT_EQ(mem_read32(cpu.mem, RSA_BASE_ADDR + 0x200u), 445);
    ASSERT_EQ(mem_read32(cpu.mem, RSA_BASE_ADDR + 0x204u), 0);
    ASSERT_EQ(mem_read32(cpu.mem, RSA_BASE_ADDR + 0x814u), 1);

    mpi_stubs_destroy(mpi);
    teardown(&cpu);
}

/* ===== SHA accelerator, driven exactly as a symbol-less guest drives it =====
 *
 * The model only ever hashed through ELF-symbol hooks on sha_hal_hash_block.
 * A production image has no symbols, so nothing hooked, START/CONTINUE were
 * no-ops, and the guest read back part of its own message block as the
 * digest. NerdMiner performs 16k SHA-1 operations this way, so drive the
 * register interface the same way it does -- fill SHA_TEXT, START, CONTINUE
 * for the second block, LOAD, read the digest -- against known answers.
 */
#define SHA_BASE        0x3FF03000u
#define SHA1_START      (SHA_BASE + 0x80u)
#define SHA1_CONTINUE   (SHA_BASE + 0x84u)
#define SHA1_LOAD       (SHA_BASE + 0x88u)
#define SHA256_START    (SHA_BASE + 0x90u)
#define SHA256_CONTINUE (SHA_BASE + 0x94u)
#define SHA256_LOAD     (SHA_BASE + 0x98u)

/* IDF's sha_ll_fill_text_block byte-swaps on the way in, so a TEXT register
 * holds the big-endian reading of four message bytes. */
static void sha_fill_block(xtensa_cpu_t *cpu, const uint8_t *block) {
    for (int i = 0; i < 16; i++) {
        uint32_t w = ((uint32_t)block[i * 4 + 0] << 24) |
                     ((uint32_t)block[i * 4 + 1] << 16) |
                     ((uint32_t)block[i * 4 + 2] << 8) |
                      (uint32_t)block[i * 4 + 3];
        mem_write32(cpu->mem, SHA_BASE + (uint32_t)i * 4u, w);
    }
}

/* Pad as SHA-1/SHA-256 require: 0x80, zeros, then a 64-bit big-endian bit
 * count. Returns the number of 64-byte blocks. */
static int sha_pad(const char *msg, uint8_t *out) {
    size_t n = strlen(msg);
    size_t total = ((n + 8) / 64 + 1) * 64;
    memset(out, 0, total);
    memcpy(out, msg, n);
    out[n] = 0x80;
    uint64_t bits = (uint64_t)n * 8u;
    for (int i = 0; i < 8; i++)
        out[total - 1 - i] = (uint8_t)(bits >> (8 * i));
    return (int)(total / 64);
}

static void sha_check(xtensa_cpu_t *cpu, const char *msg, int words,
                      uint32_t start_reg, uint32_t cont_reg, uint32_t load_reg,
                      const uint32_t *expect, const char *name) {
    uint8_t padded[256];
    int blocks = sha_pad(msg, padded);
    for (int b = 0; b < blocks; b++) {
        sha_fill_block(cpu, padded + b * 64);
        mem_write32(cpu->mem, b == 0 ? start_reg : cont_reg, 1u);
    }
    mem_write32(cpu->mem, load_reg, 1u);
    for (int i = 0; i < words; i++) {
        uint32_t got = mem_read32(cpu->mem, SHA_BASE + (uint32_t)i * 4u);
        if (got != expect[i])
            fprintf(stderr, "  DIFF %s: word %d got %08X want %08X\n",
                    name, i, got, expect[i]);
        ASSERT_EQ(got, expect[i]);
    }
}

TEST(sha_accelerator_matches_known_answers) {
    xtensa_cpu_t cpu;
    setup(&cpu);
    sha_stubs_t *ss = sha_stubs_create(&cpu, NULL);
    ASSERT_TRUE(ss != NULL);

    static const uint32_t sha1_abc[5] = {
        0xA9993E36u, 0x4706816Au, 0xBA3E2571u, 0x7850C26Cu, 0x9CD0D89Du};
    sha_check(&cpu, "abc", 5, SHA1_START, SHA1_CONTINUE, SHA1_LOAD,
              sha1_abc, "sha1(abc)");

    /* Two blocks, so START then CONTINUE -- exactly the pattern NerdMiner
     * issues, and the one that was silently returning the message back. */
    static const uint32_t sha1_two[5] = {
        0x84983E44u, 0x1C3BD26Eu, 0xBAAE4AA1u, 0xF95129E5u, 0xE54670F1u};
    sha_check(&cpu,
              "abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq",
              5, SHA1_START, SHA1_CONTINUE, SHA1_LOAD, sha1_two, "sha1(2blk)");

    static const uint32_t sha256_abc[8] = {
        0xBA7816BFu, 0x8F01CFEAu, 0x414140DEu, 0x5DAE2223u,
        0xB00361A3u, 0x96177A9Cu, 0xB410FF61u, 0xF20015ADu};
    sha_check(&cpu, "abc", 8, SHA256_START, SHA256_CONTINUE, SHA256_LOAD,
              sha256_abc, "sha256(abc)");

    sha_stubs_destroy(ss);
    teardown(&cpu);
}

/* ESP32-S3 drives the unified SHA block through its native register layout.
 * Production ESP-IDF uses the same path without requiring ELF symbols. */
#define S3_SHA_BASE             0x6003B000u
#define S3_SHA_MODE             (S3_SHA_BASE + 0x00u)
#define S3_SHA_BLOCK_NUM        (S3_SHA_BASE + 0x0Cu)
#define S3_SHA_START            (S3_SHA_BASE + 0x10u)
#define S3_SHA_CONTINUE         (S3_SHA_BASE + 0x14u)
#define S3_SHA_BUSY             (S3_SHA_BASE + 0x18u)
#define S3_SHA_DMA_START        (S3_SHA_BASE + 0x1Cu)
#define S3_SHA_H                (S3_SHA_BASE + 0x40u)
#define S3_SHA_TEXT             (S3_SHA_BASE + 0x80u)

#define S3_GDMA_BASE            0x6003F000u
#define S3_GDMA_IN_CONF1        (S3_GDMA_BASE + 0x004u)
#define S3_GDMA_IN_INT_RAW      (S3_GDMA_BASE + 0x008u)
#define S3_GDMA_IN_INT_ST       (S3_GDMA_BASE + 0x00Cu)
#define S3_GDMA_IN_INT_ENA      (S3_GDMA_BASE + 0x010u)
#define S3_GDMA_IN_INT_CLR      (S3_GDMA_BASE + 0x014u)
#define S3_GDMA_IN_LINK         (S3_GDMA_BASE + 0x020u)
#define S3_GDMA_IN_SUC_EOF_DESC (S3_GDMA_BASE + 0x028u)
#define S3_GDMA_IN_ERR_EOF_DESC (S3_GDMA_BASE + 0x02Cu)
#define S3_GDMA_IN_DESC         (S3_GDMA_BASE + 0x030u)
#define S3_GDMA_IN_DESC_PREV    (S3_GDMA_BASE + 0x034u)
#define S3_GDMA_IN_PERI_SEL     (S3_GDMA_BASE + 0x048u)
#define S3_GDMA_OUT_CONF0       (S3_GDMA_BASE + 0x060u)
#define S3_GDMA_OUT_CONF1       (S3_GDMA_BASE + 0x064u)
#define S3_GDMA_OUT_INT_RAW     (S3_GDMA_BASE + 0x068u)
#define S3_GDMA_OUT_INT_ST      (S3_GDMA_BASE + 0x06Cu)
#define S3_GDMA_OUT_INT_ENA     (S3_GDMA_BASE + 0x070u)
#define S3_GDMA_OUT_INT_CLR     (S3_GDMA_BASE + 0x074u)
#define S3_GDMA_OUT_LINK        (S3_GDMA_BASE + 0x080u)
#define S3_GDMA_OUT_EOF_DESC    (S3_GDMA_BASE + 0x088u)
#define S3_GDMA_OUT_EOF_PREV    (S3_GDMA_BASE + 0x08Cu)
#define S3_GDMA_OUT_DESC        (S3_GDMA_BASE + 0x090u)
#define S3_GDMA_OUT_DESC_PREV   (S3_GDMA_BASE + 0x094u)
#define S3_GDMA_OUT_PERI_SEL    (S3_GDMA_BASE + 0x0A8u)

#define S3_GDMA_LINK_START      (1u << 21)
#define S3_GDMA_LINK_PARK       (1u << 23)
#define S3_GDMA_IN_LINK_AUTO_RET (1u << 20)
#define S3_GDMA_IN_LINK_START   (1u << 22)
#define S3_GDMA_IN_LINK_PARK    (1u << 24)
#define S3_GDMA_AUTO_WRITEBACK  (1u << 2)
#define S3_GDMA_CHECK_OWNER     (1u << 12)
#define S3_GDMA_DESC_EOF        (1u << 30)
#define S3_GDMA_DESC_OWNER      (1u << 31)

typedef struct {
    xtensa_cpu_t cpu;
    esp32_periph_t *periph;
    sha_stubs_t *sha;
} s3_sha_fixture_t;

static bool s3_sha_fixture_init(s3_sha_fixture_t *fixture) {
    memset(fixture, 0, sizeof(*fixture));
    const flexe_target_desc_t *target =
        flexe_target_by_id(FLEXE_TARGET_ESP32S3);
    xtensa_cpu_reset_for_target(&fixture->cpu, target);
    fixture->cpu.mem = mem_create_for_target(target);
    if (!fixture->cpu.mem) return false;
    fixture->periph = periph_create(fixture->cpu.mem);
    if (!fixture->periph || !periph_gdma(fixture->periph)) return false;
    fixture->sha = sha_stubs_create(
        &fixture->cpu, periph_gdma(fixture->periph));
    return fixture->sha != NULL;
}

static void s3_sha_fixture_destroy(s3_sha_fixture_t *fixture) {
    sha_stubs_destroy(fixture->sha);
    periph_destroy(fixture->periph);
    mem_destroy(fixture->cpu.mem);
}

static int sha_pad_for_block(const char *msg, uint8_t *out,
                             size_t block_size) {
    size_t n = strlen(msg);
    size_t length_size = block_size == 128u ? 16u : 8u;
    size_t total = (n + 1u + length_size + block_size - 1u) /
                   block_size * block_size;
    memset(out, 0, total);
    memcpy(out, msg, n);
    out[n] = 0x80u;
    uint64_t bits = (uint64_t)n * 8u;
    for (int i = 0; i < 8; i++)
        out[total - 1u - (size_t)i] = (uint8_t)(bits >> (8 * i));
    return (int)(total / block_size);
}

static void s3_sha_fill_block(xtensa_mem_t *mem, const uint8_t *block,
                              size_t block_size) {
    for (size_t i = 0u; i < block_size / sizeof(uint32_t); i++)
        mem_write32(mem, S3_SHA_TEXT + (uint32_t)i * 4u,
                    crypto_le32(block + i * 4u));
}

static void s3_sha_check_direct(xtensa_mem_t *mem, const char *msg,
                                uint32_t mode, size_t block_size,
                                const uint32_t *expected, size_t words) {
    uint8_t padded[256];
    int blocks = sha_pad_for_block(msg, padded, block_size);
    mem_write32(mem, S3_SHA_MODE, mode);
    for (int block = 0; block < blocks; block++) {
        s3_sha_fill_block(mem, padded + (size_t)block * block_size,
                          block_size);
        mem_write32(mem, block == 0 ? S3_SHA_START : S3_SHA_CONTINUE, 1u);
    }
    ASSERT_EQ(mem_read32(mem, S3_SHA_BUSY), 0u);
    for (size_t i = 0u; i < words; i++)
        ASSERT_EQ(mem_read32(mem, S3_SHA_H + (uint32_t)i * 4u), expected[i]);
}

TEST(esp32s3_sha_direct_modes_match_known_answers) {
    s3_sha_fixture_t fixture;
    bool ready = s3_sha_fixture_init(&fixture);
    ASSERT_TRUE(ready);
    if (!ready) {
        s3_sha_fixture_destroy(&fixture);
        return;
    }

    static const uint32_t sha1_abc[5] = {
        0xA9993E36u, 0x4706816Au, 0xBA3E2571u, 0x7850C26Cu,
        0x9CD0D89Du};
    static const uint32_t sha1_two[5] = {
        0x84983E44u, 0x1C3BD26Eu, 0xBAAE4AA1u, 0xF95129E5u,
        0xE54670F1u};
    static const uint32_t sha224_abc[7] = {
        0x23097D22u, 0x3405D822u, 0x8642A477u, 0xBDA255B3u,
        0x2AADBCE4u, 0xBDA0B3F7u, 0xE36C9DA7u};
    static const uint32_t sha256_abc[8] = {
        0xBA7816BFu, 0x8F01CFEAu, 0x414140DEu, 0x5DAE2223u,
        0xB00361A3u, 0x96177A9Cu, 0xB410FF61u, 0xF20015ADu};
    static const uint32_t sha384_abc[12] = {
        0xCB00753Fu, 0x45A35E8Bu, 0xB5A03D69u, 0x9AC65007u,
        0x272C32ABu, 0x0EDED163u, 0x1A8B605Au, 0x43FF5BEDu,
        0x8086072Bu, 0xA1E7CC23u, 0x58BAECA1u, 0x34C825A7u};
    static const uint32_t sha512_abc[16] = {
        0xDDAF35A1u, 0x93617ABAu, 0xCC417349u, 0xAE204131u,
        0x12E6FA4Eu, 0x89A97EA2u, 0x0A9EEEE6u, 0x4B55D39Au,
        0x2192992Au, 0x274FC1A8u, 0x36BA3C23u, 0xA3FEEBBDu,
        0x454D4423u, 0x643CE80Eu, 0x2A9AC94Fu, 0xA54CA49Fu};

    s3_sha_check_direct(fixture.cpu.mem, "abc", 0u, 64u,
                        sha1_abc, 5u);
    s3_sha_check_direct(
        fixture.cpu.mem,
        "abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq",
        0u, 64u, sha1_two, 5u);
    s3_sha_check_direct(fixture.cpu.mem, "abc", 1u, 64u,
                        sha224_abc, 7u);
    s3_sha_check_direct(fixture.cpu.mem, "abc", 2u, 64u,
                        sha256_abc, 8u);
    s3_sha_check_direct(fixture.cpu.mem, "abc", 3u, 128u,
                        sha384_abc, 12u);
    s3_sha_check_direct(fixture.cpu.mem, "abc", 4u, 128u,
                        sha512_abc, 16u);

    s3_sha_fixture_destroy(&fixture);
}

static void s3_gdma_descriptor(xtensa_mem_t *mem, uint32_t descriptor,
                               uint32_t buffer, uint32_t length,
                               bool eof, bool owner, uint32_t next) {
    uint32_t dw0 = length | (length << 12);
    if (eof) dw0 |= S3_GDMA_DESC_EOF;
    if (owner) dw0 |= S3_GDMA_DESC_OWNER;
    mem_write32(mem, descriptor, dw0);
    mem_write32(mem, descriptor + 4u, buffer);
    mem_write32(mem, descriptor + 8u, next);
}

TEST(esp32s3_sha_consumes_chained_gdma_descriptors) {
    const uint32_t descriptor0 = 0x3FC8F000u;
    const uint32_t descriptor1 = 0x3FC8F010u;
    const uint32_t buffer0 = 0x3FC90000u;
    const uint32_t buffer1 = 0x3FC90100u;
    s3_sha_fixture_t fixture;
    bool ready = s3_sha_fixture_init(&fixture);
    ASSERT_TRUE(ready);
    if (!ready) {
        s3_sha_fixture_destroy(&fixture);
        return;
    }

    uint8_t padded[64];
    ASSERT_EQ(sha_pad_for_block("abc", padded, 64u), 1u);
    for (uint32_t i = 0u; i < 20u; i++)
        mem_write8(fixture.cpu.mem, buffer0 + i, padded[i]);
    for (uint32_t i = 20u; i < sizeof(padded); i++)
        mem_write8(fixture.cpu.mem, buffer1 + i - 20u, padded[i]);
    s3_gdma_descriptor(fixture.cpu.mem, descriptor0, buffer0, 20u,
                       false, true, descriptor1);
    s3_gdma_descriptor(fixture.cpu.mem, descriptor1, buffer1, 44u,
                       true, true, 0u);

    ASSERT_EQ(mem_read32(fixture.cpu.mem, S3_GDMA_OUT_PERI_SEL), 0x3Fu);
    ASSERT_TRUE(mem_read32(fixture.cpu.mem, S3_GDMA_OUT_LINK) &
                S3_GDMA_LINK_PARK);
    mem_write32(fixture.cpu.mem, S3_GDMA_OUT_PERI_SEL, 7u);
    mem_write32(fixture.cpu.mem, S3_GDMA_OUT_INT_ENA, 0xFFu);
    mem_write32(fixture.cpu.mem, S3_GDMA_OUT_LINK,
                (descriptor0 & 0xFFFFFu) | S3_GDMA_LINK_START);
    mem_write32(fixture.cpu.mem, S3_SHA_MODE, 0u);
    mem_write32(fixture.cpu.mem, S3_SHA_BLOCK_NUM, 1u);
    mem_write32(fixture.cpu.mem, S3_SHA_DMA_START, 1u);

    static const uint32_t sha1_abc[5] = {
        0xA9993E36u, 0x4706816Au, 0xBA3E2571u, 0x7850C26Cu,
        0x9CD0D89Du};
    for (size_t i = 0u; i < 5u; i++)
        ASSERT_EQ(mem_read32(fixture.cpu.mem,
                             S3_SHA_H + (uint32_t)i * 4u), sha1_abc[i]);
    ASSERT_TRUE(mem_read32(fixture.cpu.mem, S3_GDMA_OUT_LINK) &
                S3_GDMA_LINK_PARK);
    ASSERT_EQ(mem_read32(fixture.cpu.mem, S3_GDMA_OUT_INT_RAW), 0x0Bu);
    ASSERT_EQ(mem_read32(fixture.cpu.mem, S3_GDMA_OUT_INT_ST), 0x0Bu);
    ASSERT_EQ(mem_read32(fixture.cpu.mem, S3_GDMA_OUT_EOF_DESC),
              descriptor1);
    ASSERT_EQ(mem_read32(fixture.cpu.mem, S3_GDMA_OUT_EOF_PREV),
              descriptor0);
    ASSERT_EQ(mem_read32(fixture.cpu.mem, S3_GDMA_OUT_DESC), descriptor1);
    ASSERT_EQ(mem_read32(fixture.cpu.mem, S3_GDMA_OUT_DESC_PREV),
              descriptor0);
    /* ESP-IDF does not request automatic descriptor write-back for its
     * shared crypto channel, so ownership remains with DMA. */
    ASSERT_TRUE(mem_read32(fixture.cpu.mem, descriptor0) &
                S3_GDMA_DESC_OWNER);
    ASSERT_TRUE(mem_read32(fixture.cpu.mem, descriptor1) &
                S3_GDMA_DESC_OWNER);
    mem_write32(fixture.cpu.mem, S3_GDMA_OUT_INT_CLR, 0x0Bu);
    ASSERT_EQ(mem_read32(fixture.cpu.mem, S3_GDMA_OUT_INT_RAW), 0u);

    s3_sha_fixture_destroy(&fixture);
}

TEST(esp32s3_gdma_honors_owner_check_and_writeback) {
    const uint32_t descriptor = 0x3FC8F000u;
    const uint32_t buffer = 0x3FC90000u;
    s3_sha_fixture_t fixture;
    bool ready = s3_sha_fixture_init(&fixture);
    ASSERT_TRUE(ready);
    if (!ready) {
        s3_sha_fixture_destroy(&fixture);
        return;
    }

    uint8_t payload[4] = {1u, 2u, 3u, 4u};
    for (uint32_t i = 0u; i < sizeof(payload); i++)
        mem_write8(fixture.cpu.mem, buffer + i, payload[i]);
    s3_gdma_descriptor(fixture.cpu.mem, descriptor, buffer, sizeof(payload),
                       true, false, 0u);
    mem_write32(fixture.cpu.mem, S3_GDMA_OUT_PERI_SEL, 7u);
    mem_write32(fixture.cpu.mem, S3_GDMA_OUT_CONF1, S3_GDMA_CHECK_OWNER);
    mem_write32(fixture.cpu.mem, S3_GDMA_OUT_LINK,
                (descriptor & 0xFFFFFu) | S3_GDMA_LINK_START);
    uint8_t received[4] = {0};
    ASSERT_EQ(flexe_gdma_read_tx(periph_gdma(fixture.periph), 7u,
                                 received, sizeof(received)), -1);
    ASSERT_EQ(mem_read32(fixture.cpu.mem, S3_GDMA_OUT_INT_RAW), 1u << 2);

    mem_write32(fixture.cpu.mem, S3_GDMA_OUT_INT_CLR, UINT32_MAX);
    s3_gdma_descriptor(fixture.cpu.mem, descriptor, buffer, sizeof(payload),
                       true, true, 0u);
    mem_write32(fixture.cpu.mem, S3_GDMA_OUT_CONF0,
                S3_GDMA_AUTO_WRITEBACK | 1u);
    mem_write32(fixture.cpu.mem, S3_GDMA_OUT_LINK,
                (descriptor & 0xFFFFFu) | S3_GDMA_LINK_START);
    ASSERT_EQ(flexe_gdma_read_tx(periph_gdma(fixture.periph), 7u,
                                 received, sizeof(received)), 0u);
    ASSERT_EQ(crypto_le32(received), crypto_le32(payload));
    ASSERT_FALSE(mem_read32(fixture.cpu.mem, descriptor) &
                 S3_GDMA_DESC_OWNER);

    s3_sha_fixture_destroy(&fixture);
}

TEST(esp32s3_gdma_receives_chained_descriptors_and_reports_errors) {
    const uint32_t descriptor0 = 0x3FC8F000u;
    const uint32_t descriptor1 = 0x3FC8F010u;
    const uint32_t buffer0 = 0x3FC90000u;
    const uint32_t buffer1 = 0x3FC90100u;
    static const uint8_t payload[] = {0x10u, 0x21u, 0x32u, 0x43u, 0x54u};
    s3_sha_fixture_t fixture;
    bool ready = s3_sha_fixture_init(&fixture);
    ASSERT_TRUE(ready);
    if (!ready) {
        s3_sha_fixture_destroy(&fixture);
        return;
    }

    ASSERT_EQ(mem_read32(fixture.cpu.mem, S3_GDMA_IN_PERI_SEL), 0x3Fu);
    ASSERT_EQ(mem_read32(fixture.cpu.mem, S3_GDMA_IN_LINK),
              S3_GDMA_IN_LINK_AUTO_RET | S3_GDMA_IN_LINK_PARK);
    mem_write32(fixture.cpu.mem, S3_GDMA_IN_INT_RAW, UINT32_MAX);
    ASSERT_EQ(mem_read32(fixture.cpu.mem, S3_GDMA_IN_INT_RAW), 0u);
    mem_write32(fixture.cpu.mem, S3_GDMA_IN_INT_ENA, UINT32_MAX);
    ASSERT_EQ(mem_read32(fixture.cpu.mem, S3_GDMA_IN_INT_ENA), 0x3FFu);
    mem_write32(fixture.cpu.mem, S3_GDMA_IN_PERI_SEL, 0u);
    mem_write32(fixture.cpu.mem, S3_GDMA_IN_CONF1,
                S3_GDMA_CHECK_OWNER);

    /* Owner checking rejects a CPU-owned receive descriptor and raises only
     * the architectural descriptor-error condition. */
    s3_gdma_descriptor(fixture.cpu.mem, descriptor0, buffer0, 3u,
                       false, false, 0u);
    mem_write32(fixture.cpu.mem, S3_GDMA_IN_LINK,
                (descriptor0 & 0xFFFFFu) | S3_GDMA_IN_LINK_AUTO_RET |
                S3_GDMA_IN_LINK_START);
    ASSERT_EQ(flexe_gdma_write_rx(periph_gdma(fixture.periph), 0u,
                                  payload, sizeof(payload)), -1);
    ASSERT_EQ(mem_read32(fixture.cpu.mem, S3_GDMA_IN_INT_RAW), 1u << 3);
    ASSERT_EQ(mem_read32(fixture.cpu.mem, S3_GDMA_IN_ERR_EOF_DESC), 0u);
    mem_write32(fixture.cpu.mem, S3_GDMA_IN_INT_CLR, UINT32_MAX);

    s3_gdma_descriptor(fixture.cpu.mem, descriptor0, buffer0, 3u,
                       false, true, descriptor1);
    s3_gdma_descriptor(fixture.cpu.mem, descriptor1, buffer1, 4u,
                       true, true, 0u);
    mem_write32(fixture.cpu.mem, S3_GDMA_IN_LINK,
                (descriptor0 & 0xFFFFFu) | S3_GDMA_IN_LINK_AUTO_RET |
                S3_GDMA_IN_LINK_START);
    ASSERT_EQ(flexe_gdma_write_rx(periph_gdma(fixture.periph), 0u,
                                  payload, sizeof(payload)), 0u);
    ASSERT_EQ(mem_read8(fixture.cpu.mem, buffer0), payload[0]);
    ASSERT_EQ(mem_read8(fixture.cpu.mem, buffer0 + 2u), payload[2]);
    ASSERT_EQ(mem_read8(fixture.cpu.mem, buffer1), payload[3]);
    ASSERT_EQ(mem_read8(fixture.cpu.mem, buffer1 + 1u), payload[4]);
    ASSERT_EQ((mem_read32(fixture.cpu.mem, descriptor0) >> 12) & 0xFFFu,
              3u);
    ASSERT_EQ((mem_read32(fixture.cpu.mem, descriptor1) >> 12) & 0xFFFu,
              2u);
    ASSERT_FALSE(mem_read32(fixture.cpu.mem, descriptor0) &
                 S3_GDMA_DESC_OWNER);
    ASSERT_FALSE(mem_read32(fixture.cpu.mem, descriptor1) &
                 S3_GDMA_DESC_OWNER);
    ASSERT_EQ(mem_read32(fixture.cpu.mem, S3_GDMA_IN_INT_RAW), 0x3u);
    ASSERT_EQ(mem_read32(fixture.cpu.mem, S3_GDMA_IN_INT_ST), 0x3u);
    ASSERT_EQ(mem_read32(fixture.cpu.mem, S3_GDMA_IN_SUC_EOF_DESC),
              descriptor1);
    ASSERT_EQ(mem_read32(fixture.cpu.mem, S3_GDMA_IN_DESC), descriptor1);
    ASSERT_EQ(mem_read32(fixture.cpu.mem, S3_GDMA_IN_DESC_PREV),
              descriptor0);
    ASSERT_EQ(mem_read32(fixture.cpu.mem, S3_GDMA_IN_LINK),
              (descriptor0 & 0xFFFFFu) | S3_GDMA_IN_LINK_AUTO_RET |
              S3_GDMA_IN_LINK_PARK);

    /* A valid buffer which fills before the peripheral transfer completes is
     * a descriptor-empty condition, not a malformed-descriptor condition. */
    mem_write32(fixture.cpu.mem, S3_GDMA_IN_INT_CLR, UINT32_MAX);
    s3_gdma_descriptor(fixture.cpu.mem, descriptor0, buffer0, 2u,
                       true, true, 0u);
    mem_write32(fixture.cpu.mem, S3_GDMA_IN_LINK,
                (descriptor0 & 0xFFFFFu) | S3_GDMA_IN_LINK_START);
    ASSERT_EQ(flexe_gdma_write_rx(periph_gdma(fixture.periph), 0u,
                                  payload, 3u), -1);
    ASSERT_EQ(mem_read32(fixture.cpu.mem, S3_GDMA_IN_INT_RAW),
              (1u << 4) | 1u);
    ASSERT_EQ(mem_read32(fixture.cpu.mem, S3_GDMA_IN_LINK),
              (descriptor0 & 0xFFFFFu) | S3_GDMA_IN_LINK_PARK);

    s3_sha_fixture_destroy(&fixture);
}

TEST(firmware_profile_does_not_authorize_mbedtls_sha256) {
    xtensa_cpu_t cpu;
    setup(&cpu);
    esp32_rom_stubs_t *rom = rom_stubs_create(&cpu);
    sha_stubs_t *ss = sha_stubs_create(&cpu, NULL);
    ASSERT_TRUE(ss != NULL);
    int initial_stub_count = rom_stubs_stub_count(rom);
    ASSERT_EQ(sha_stubs_hook_firmware(ss), 0);

    /* A recognized release profile and bytes at its former absolute entries
     * are no longer sufficient. Only the complete relocation-normalized
     * implementation family can install native SHA hooks. */
    seed_tasmota32_v1560_profile(&cpu);
    static const uint8_t former_starts_prefix[] = {
        0x36, 0x41, 0x00, 0x8D, 0x02, 0x22,
        0xAF, 0x8C, 0xF6, 0x23, 0x3F, 0x16,
    };
    put_test_bytes(&cpu, 0x401E11C8u, former_starts_prefix,
                   sizeof(former_starts_prefix));
    ASSERT_EQ(sha_stubs_hook_firmware(ss), 0);
    ASSERT_EQ(rom_stubs_stub_count(rom), initial_stub_count);

    sha_stubs_destroy(ss);
    rom_stubs_destroy(rom);
    teardown(&cpu);
}

static void run_crypto_tests(void) {
    TEST_SUITE("Crypto MMIO");
    RUN_TEST(sha_accelerator_matches_known_answers);
    RUN_TEST(esp32s3_sha_direct_modes_match_known_answers);
    RUN_TEST(esp32s3_sha_consumes_chained_gdma_descriptors);
    RUN_TEST(esp32s3_gdma_honors_owner_check_and_writeback);
    RUN_TEST(esp32s3_gdma_receives_chained_descriptors_and_reports_errors);
    RUN_TEST(firmware_profile_does_not_authorize_mbedtls_sha256);
    RUN_TEST(raw_aes_128_encrypt_decrypt);
    RUN_TEST(raw_aes_192_encrypt_decrypt);
    RUN_TEST(raw_aes_256_encrypt_decrypt);
    RUN_TEST(raw_mpi_montgomery_multiply);
    RUN_TEST(raw_mpi_plain_multiply);
    RUN_TEST(raw_mpi_modular_exponentiation);
}
