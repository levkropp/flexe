#include "sha_stubs.h"
#include "test_helpers.h"
#include "aes_stubs.h"
#include "mpi_stubs.h"

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
    sha_stubs_t *ss = sha_stubs_create(&cpu);
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

static void run_crypto_tests(void) {
    TEST_SUITE("Crypto MMIO");
    RUN_TEST(sha_accelerator_matches_known_answers);
    RUN_TEST(raw_aes_128_encrypt_decrypt);
    RUN_TEST(raw_aes_192_encrypt_decrypt);
    RUN_TEST(raw_aes_256_encrypt_decrypt);
    RUN_TEST(raw_mpi_montgomery_multiply);
    RUN_TEST(raw_mpi_plain_multiply);
    RUN_TEST(raw_mpi_modular_exponentiation);
}
