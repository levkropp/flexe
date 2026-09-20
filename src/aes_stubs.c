#include "aes_stubs.h"
#include "target.h"
#include "rom_stubs.h"
#include "memory.h"
#include "gdma.h"
#include "peripherals.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

/* ESP-IDF AES mode constants */
#define AES_MODE_ENCRYPT 1
#define AES_MODE_DECRYPT 0

/* Max expanded key: AES-256 = 60 uint32_t words */
#define AES_MAX_ROUNDS   14
#define AES_MAX_RK       60

#define AES_KEY_WORDS       8u
#define AES_BLOCK_WORDS     4u
#define AES_BLOCK_BYTES     16u
#define AES_DMA_DESC_LIMIT  1024u
#define AES_DMA_DESC_MAX    4095u
#define AES_DMA_MAX_BYTES   (AES_DMA_DESC_LIMIT * AES_DMA_DESC_MAX)

/* Original ESP32 AES register offsets. */
#define AES32_START_OFF     0x00u
#define AES32_IDLE_OFF      0x04u
#define AES32_MODE_OFF      0x08u
#define AES32_KEY_OFF       0x10u
#define AES32_TEXT_OFF      0x30u
#define AES32_ENDIAN_OFF    0x40u

/* S2/S3-generation AES register offsets. */
#define AES2_KEY_OFF        0x00u
#define AES2_TEXT_IN_OFF    0x20u
#define AES2_TEXT_OUT_OFF   0x30u
#define AES2_MODE_OFF       0x40u
#define AES2_ENDIAN_OFF     0x44u
#define AES2_TRIGGER_OFF    0x48u
#define AES2_STATE_OFF      0x4Cu
#define AES2_IV_OFF         0x50u
#define AES2_DMA_ENABLE_OFF 0x90u
#define AES2_BLOCK_MODE_OFF 0x94u
#define AES2_BLOCK_NUM_OFF  0x98u
#define AES2_INC_SEL_OFF    0x9Cu
#define AES2_AAD_NUM_OFF    0xA0u
#define AES2_VALID_BITS_OFF 0xA4u
#define AES2_CONTINUE_OFF   0xA8u
#define AES2_INT_CLEAR_OFF  0xACu
#define AES2_INT_ENABLE_OFF 0xB0u
#define AES2_DATE_OFF       0xB4u
#define AES2_DMA_EXIT_OFF   0xB8u

enum {
    AES_STATE_IDLE = 0u,
    AES_STATE_BUSY = 1u,
    AES_STATE_DONE = 2u,
};

enum {
    AES_BLOCK_MODE_ECB = 0u,
    AES_BLOCK_MODE_CBC = 1u,
    AES_BLOCK_MODE_OFB = 2u,
    AES_BLOCK_MODE_CTR = 3u,
    AES_BLOCK_MODE_CFB8 = 4u,
    AES_BLOCK_MODE_CFB128 = 5u,
};

struct aes_stubs {
    xtensa_cpu_t      *cpu;
    esp32_rom_stubs_t *rom;
    const flexe_target_desc_t *target;
    flexe_gdma_t      *gdma;
    mmio_read_fn       fallback_read;
    mmio_write_fn      fallback_write;
    void              *fallback_ctx;
    esp32_periph_t     *system_periph;
    bool                system_clock_enabled;
    bool                system_reset_asserted;
    bool                interrupt_pending;

    /* Per-core key state so interleaved batches don't corrupt */
    uint32_t round_key[2][AES_MAX_RK];
    int      nr[2];          /* number of rounds (10/12/14) */
    int      mode[2];        /* 0=decrypt, 1=encrypt */

    /* The physical accelerator is shared by both CPUs.  Keep its raw MMIO
     * register state separate from the per-core HAL-hook state above. */
    uint32_t hw_key[AES_KEY_WORDS];
    uint32_t hw_text_in[AES_BLOCK_WORDS];
    uint32_t hw_text_out[AES_BLOCK_WORDS];
    uint32_t hw_iv[AES_BLOCK_WORDS];
    uint32_t hw_mode;
    uint32_t hw_endian;
    uint32_t hw_state;
    uint32_t hw_dma_enable;
    uint32_t hw_block_mode;
    uint32_t hw_block_num;
    uint32_t hw_inc_sel;
    uint32_t hw_aad_num;
    uint32_t hw_valid_bits;
    uint32_t hw_int_enable;
    uint32_t hw_date;
    bool warned_dma;
    bool warned_continue;
};

/* ===== Calling convention helpers ===== */

static uint32_t aes_arg(xtensa_cpu_t *cpu, int n) {
    int ci = XT_PS_CALLINC(cpu->ps);
    return ar_read(cpu, ci * 4 + 2 + n);
}

static void aes_return(xtensa_cpu_t *cpu, uint32_t retval) {
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

static void aes_return_void(xtensa_cpu_t *cpu) {
    int ci = XT_PS_CALLINC(cpu->ps);
    if (ci > 0) {
        uint32_t a0 = ar_read(cpu, ci * 4);
        cpu->pc = (cpu->pc & 0xC0000000u) | (a0 & 0x3FFFFFFFu);
        XT_PS_SET_CALLINC(cpu->ps, 0);
    } else {
        cpu->pc = (cpu->pc & 0xC0000000u) | (ar_read(cpu, 0) & 0x3FFFFFFFu);
    }
}

/* ===== AES S-boxes ===== */

static const uint8_t SBOX[256] = {
    0x63,0x7c,0x77,0x7b,0xf2,0x6b,0x6f,0xc5,0x30,0x01,0x67,0x2b,0xfe,0xd7,0xab,0x76,
    0xca,0x82,0xc9,0x7d,0xfa,0x59,0x47,0xf0,0xad,0xd4,0xa2,0xaf,0x9c,0xa4,0x72,0xc0,
    0xb7,0xfd,0x93,0x26,0x36,0x3f,0xf7,0xcc,0x34,0xa5,0xe5,0xf1,0x71,0xd8,0x31,0x15,
    0x04,0xc7,0x23,0xc3,0x18,0x96,0x05,0x9a,0x07,0x12,0x80,0xe2,0xeb,0x27,0xb2,0x75,
    0x09,0x83,0x2c,0x1a,0x1b,0x6e,0x5a,0xa0,0x52,0x3b,0xd6,0xb3,0x29,0xe3,0x2f,0x84,
    0x53,0xd1,0x00,0xed,0x20,0xfc,0xb1,0x5b,0x6a,0xcb,0xbe,0x39,0x4a,0x4c,0x58,0xcf,
    0xd0,0xef,0xaa,0xfb,0x43,0x4d,0x33,0x85,0x45,0xf9,0x02,0x7f,0x50,0x3c,0x9f,0xa8,
    0x51,0xa3,0x40,0x8f,0x92,0x9d,0x38,0xf5,0xbc,0xb6,0xda,0x21,0x10,0xff,0xf3,0xd2,
    0xcd,0x0c,0x13,0xec,0x5f,0x97,0x44,0x17,0xc4,0xa7,0x7e,0x3d,0x64,0x5d,0x19,0x73,
    0x60,0x81,0x4f,0xdc,0x22,0x2a,0x90,0x88,0x46,0xee,0xb8,0x14,0xde,0x5e,0x0b,0xdb,
    0xe0,0x32,0x3a,0x0a,0x49,0x06,0x24,0x5c,0xc2,0xd3,0xac,0x62,0x91,0x95,0xe4,0x79,
    0xe7,0xc8,0x37,0x6d,0x8d,0xd5,0x4e,0xa9,0x6c,0x56,0xf4,0xea,0x65,0x7a,0xae,0x08,
    0xba,0x78,0x25,0x2e,0x1c,0xa6,0xb4,0xc6,0xe8,0xdd,0x74,0x1f,0x4b,0xbd,0x8b,0x8a,
    0x70,0x3e,0xb5,0x66,0x48,0x03,0xf6,0x0e,0x61,0x35,0x57,0xb9,0x86,0xc1,0x1d,0x9e,
    0xe1,0xf8,0x98,0x11,0x69,0xd9,0x8e,0x94,0x9b,0x1e,0x87,0xe9,0xce,0x55,0x28,0xdf,
    0x8c,0xa1,0x89,0x0d,0xbf,0xe6,0x42,0x68,0x41,0x99,0x2d,0x0f,0xb0,0x54,0xbb,0x16,
};

static const uint8_t INV_SBOX[256] = {
    0x52,0x09,0x6a,0xd5,0x30,0x36,0xa5,0x38,0xbf,0x40,0xa3,0x9e,0x81,0xf3,0xd7,0xfb,
    0x7c,0xe3,0x39,0x82,0x9b,0x2f,0xff,0x87,0x34,0x8e,0x43,0x44,0xc4,0xde,0xe9,0xcb,
    0x54,0x7b,0x94,0x32,0xa6,0xc2,0x23,0x3d,0xee,0x4c,0x95,0x0b,0x42,0xfa,0xc3,0x4e,
    0x08,0x2e,0xa1,0x66,0x28,0xd9,0x24,0xb2,0x76,0x5b,0xa2,0x49,0x6d,0x8b,0xd1,0x25,
    0x72,0xf8,0xf6,0x64,0x86,0x68,0x98,0x16,0xd4,0xa4,0x5c,0xcc,0x5d,0x65,0xb6,0x92,
    0x6c,0x70,0x48,0x50,0xfd,0xed,0xb9,0xda,0x5e,0x15,0x46,0x57,0xa7,0x8d,0x9d,0x84,
    0x90,0xd8,0xab,0x00,0x8c,0xbc,0xd3,0x0a,0xf7,0xe4,0x58,0x05,0xb8,0xb3,0x45,0x06,
    0xd0,0x2c,0x1e,0x8f,0xca,0x3f,0x0f,0x02,0xc1,0xaf,0xbd,0x03,0x01,0x13,0x8a,0x6b,
    0x3a,0x91,0x11,0x41,0x4f,0x67,0xdc,0xea,0x97,0xf2,0xcf,0xce,0xf0,0xb4,0xe6,0x73,
    0x96,0xac,0x74,0x22,0xe7,0xad,0x35,0x85,0xe2,0xf9,0x37,0xe8,0x1c,0x75,0xdf,0x6e,
    0x47,0xf1,0x1a,0x71,0x1d,0x29,0xc5,0x89,0x6f,0xb7,0x62,0x0e,0xaa,0x18,0xbe,0x1b,
    0xfc,0x56,0x3e,0x4b,0xc6,0xd2,0x79,0x20,0x9a,0xdb,0xc0,0xfe,0x78,0xcd,0x5a,0xf4,
    0x1f,0xdd,0xa8,0x33,0x88,0x07,0xc7,0x31,0xb1,0x12,0x10,0x59,0x27,0x80,0xec,0x5f,
    0x60,0x51,0x7f,0xa9,0x19,0xb5,0x4a,0x0d,0x2d,0xe5,0x7a,0x9f,0x93,0xc9,0x9c,0xef,
    0xa0,0xe0,0x3b,0x4d,0xae,0x2a,0xf5,0xb0,0xc8,0xeb,0xbb,0x3c,0x83,0x53,0x99,0x61,
    0x17,0x2b,0x04,0x7e,0xba,0x77,0xd6,0x26,0xe1,0x69,0x14,0x63,0x55,0x21,0x0c,0x7d,
};

/* ===== AES round constants ===== */

static const uint8_t RCON[11] = {
    0x00, 0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80, 0x1b, 0x36
};

/* ===== Key expansion ===== */

static uint32_t sub_word(uint32_t w) {
    return ((uint32_t)SBOX[(w >> 24) & 0xff] << 24) |
           ((uint32_t)SBOX[(w >> 16) & 0xff] << 16) |
           ((uint32_t)SBOX[(w >>  8) & 0xff] <<  8) |
           ((uint32_t)SBOX[ w        & 0xff]);
}

static uint32_t rot_word(uint32_t w) {
    return (w << 8) | (w >> 24);
}

/* Expand key into round_key[], return number of rounds */
static int aes_key_expand(const uint8_t *key, int key_bytes, uint32_t *rk) {
    int nk = key_bytes / 4;   /* key words: 4/6/8 */
    int nr;
    switch (nk) {
    case 4: nr = 10; break;
    case 6: nr = 12; break;
    case 8: nr = 14; break;
    default: nr = 10; nk = 4; break;
    }
    int total = 4 * (nr + 1);

    /* Copy key into first nk words */
    for (int i = 0; i < nk; i++)
        rk[i] = ((uint32_t)key[4*i] << 24) | ((uint32_t)key[4*i+1] << 16) |
                 ((uint32_t)key[4*i+2] << 8) | key[4*i+3];

    for (int i = nk; i < total; i++) {
        uint32_t temp = rk[i - 1];
        if (i % nk == 0)
            temp = sub_word(rot_word(temp)) ^ ((uint32_t)RCON[i / nk] << 24);
        else if (nk > 6 && i % nk == 4)
            temp = sub_word(temp);
        rk[i] = rk[i - nk] ^ temp;
    }
    return nr;
}

/* ===== AES encrypt/decrypt a single 128-bit block ===== */

static inline uint8_t xtime(uint8_t x) {
    return (uint8_t)((x << 1) ^ (((x >> 7) & 1) * 0x1b));
}

static void aes_encrypt_block(const uint8_t in[16], uint8_t out[16],
                               const uint32_t *rk, int nr) {
    uint8_t s[16];
    memcpy(s, in, 16);

    /* AddRoundKey (round 0) */
    for (int i = 0; i < 16; i++)
        s[i] ^= (uint8_t)(rk[i / 4] >> (24 - 8 * (i % 4)));

    for (int round = 1; round <= nr; round++) {
        /* SubBytes */
        for (int i = 0; i < 16; i++)
            s[i] = SBOX[s[i]];

        /* ShiftRows */
        uint8_t t;
        t = s[1]; s[1] = s[5]; s[5] = s[9]; s[9] = s[13]; s[13] = t;
        t = s[2]; s[2] = s[10]; s[10] = t; t = s[6]; s[6] = s[14]; s[14] = t;
        t = s[15]; s[15] = s[11]; s[11] = s[7]; s[7] = s[3]; s[3] = t;

        /* MixColumns (skip on last round) */
        if (round < nr) {
            for (int c = 0; c < 4; c++) {
                int j = c * 4;
                uint8_t a0 = s[j], a1 = s[j+1], a2 = s[j+2], a3 = s[j+3];
                uint8_t x0 = xtime(a0), x1 = xtime(a1), x2 = xtime(a2), x3 = xtime(a3);
                s[j]   = x0 ^ x1 ^ a1 ^ a2 ^ a3;
                s[j+1] = a0 ^ x1 ^ x2 ^ a2 ^ a3;
                s[j+2] = a0 ^ a1 ^ x2 ^ x3 ^ a3;
                s[j+3] = x0 ^ a0 ^ a1 ^ a2 ^ x3;
            }
        }

        /* AddRoundKey */
        const uint32_t *rr = &rk[round * 4];
        for (int i = 0; i < 16; i++)
            s[i] ^= (uint8_t)(rr[i / 4] >> (24 - 8 * (i % 4)));
    }

    memcpy(out, s, 16);
}

static inline uint8_t mul(uint8_t a, uint8_t b) {
    uint8_t r = 0;
    for (int i = 0; i < 8; i++) {
        if (b & 1) r ^= a;
        uint8_t hi = a & 0x80;
        a <<= 1;
        if (hi) a ^= 0x1b;
        b >>= 1;
    }
    return r;
}

static void aes_decrypt_block(const uint8_t in[16], uint8_t out[16],
                               const uint32_t *rk, int nr) {
    uint8_t s[16];
    memcpy(s, in, 16);

    /* AddRoundKey (last round key) */
    const uint32_t *rr = &rk[nr * 4];
    for (int i = 0; i < 16; i++)
        s[i] ^= (uint8_t)(rr[i / 4] >> (24 - 8 * (i % 4)));

    for (int round = nr - 1; round >= 0; round--) {
        /* InvShiftRows */
        uint8_t t;
        t = s[13]; s[13] = s[9]; s[9] = s[5]; s[5] = s[1]; s[1] = t;
        t = s[2]; s[2] = s[10]; s[10] = t; t = s[6]; s[6] = s[14]; s[14] = t;
        t = s[3]; s[3] = s[7]; s[7] = s[11]; s[11] = s[15]; s[15] = t;

        /* InvSubBytes */
        for (int i = 0; i < 16; i++)
            s[i] = INV_SBOX[s[i]];

        /* AddRoundKey */
        rr = &rk[round * 4];
        for (int i = 0; i < 16; i++)
            s[i] ^= (uint8_t)(rr[i / 4] >> (24 - 8 * (i % 4)));

        /* InvMixColumns (skip on round 0) */
        if (round > 0) {
            for (int c = 0; c < 4; c++) {
                int j = c * 4;
                uint8_t a0 = s[j], a1 = s[j+1], a2 = s[j+2], a3 = s[j+3];
                s[j]   = mul(a0,0x0e) ^ mul(a1,0x0b) ^ mul(a2,0x0d) ^ mul(a3,0x09);
                s[j+1] = mul(a0,0x09) ^ mul(a1,0x0e) ^ mul(a2,0x0b) ^ mul(a3,0x0d);
                s[j+2] = mul(a0,0x0d) ^ mul(a1,0x09) ^ mul(a2,0x0e) ^ mul(a3,0x0b);
                s[j+3] = mul(a0,0x0b) ^ mul(a1,0x0d) ^ mul(a2,0x09) ^ mul(a3,0x0e);
            }
        }
    }

    memcpy(out, s, 16);
}

/* ===== AES peripheral MMIO ===== */

static void aes_word_to_bytes(uint32_t word, uint8_t out[4]) {
    out[0] = (uint8_t)word;
    out[1] = (uint8_t)(word >> 8);
    out[2] = (uint8_t)(word >> 16);
    out[3] = (uint8_t)(word >> 24);
}

static uint32_t aes_bytes_to_word(const uint8_t in[4]) {
    return (uint32_t)in[0] |
           ((uint32_t)in[1] << 8) |
           ((uint32_t)in[2] << 16) |
           ((uint32_t)in[3] << 24);
}

static bool aes_mode_key(aes_stubs_t *as, uint32_t *round_key,
                         int *rounds) {
    unsigned key_size_code = as->hw_mode & 3u;
    if (key_size_code > 2u ||
        (as->target->aes.key_size_mask & (1u << key_size_code)) == 0u)
        return false;

    int key_bytes = 16 + (int)key_size_code * 8;
    uint8_t key[32] = {0};
    for (int i = 0; i < key_bytes / 4; i++)
        aes_word_to_bytes(as->hw_key[i], &key[i * 4]);
    *rounds = aes_key_expand(key, key_bytes, round_key);
    return true;
}

static void aes_words_to_block(const uint32_t words[AES_BLOCK_WORDS],
                               uint8_t block[AES_BLOCK_BYTES]) {
    for (unsigned i = 0u; i < AES_BLOCK_WORDS; i++)
        aes_word_to_bytes(words[i], &block[i * sizeof(uint32_t)]);
}

static void aes_block_to_words(const uint8_t block[AES_BLOCK_BYTES],
                               uint32_t words[AES_BLOCK_WORDS]) {
    for (unsigned i = 0u; i < AES_BLOCK_WORDS; i++)
        words[i] = aes_bytes_to_word(&block[i * sizeof(uint32_t)]);
}

static bool aes_transform_direct(aes_stubs_t *as) {
    uint8_t in[AES_BLOCK_BYTES];
    uint8_t out[AES_BLOCK_BYTES];
    uint32_t round_key[AES_MAX_RK];
    int rounds = 0;
    if (!aes_mode_key(as, round_key, &rounds)) return false;
    aes_words_to_block(as->hw_text_in, in);

    if (as->hw_mode & 4u)
        aes_decrypt_block(in, out, round_key, rounds);
    else
        aes_encrypt_block(in, out, round_key, rounds);

    if (as->target->aes.layout == FLEXE_AES_LAYOUT_ESP32)
        aes_block_to_words(out, as->hw_text_in);
    else
        aes_block_to_words(out, as->hw_text_out);
    return true;
}

static void aes_counter_increment(uint8_t counter[AES_BLOCK_BYTES],
                                  bool increment_128) {
    unsigned first = increment_128 ? 0u : 12u;
    for (unsigned i = AES_BLOCK_BYTES; i-- > first; )
        if (++counter[i] != 0u) break;
}

static void aes_xor_block(uint8_t out[AES_BLOCK_BYTES],
                          const uint8_t left[AES_BLOCK_BYTES],
                          const uint8_t right[AES_BLOCK_BYTES]) {
    for (unsigned i = 0u; i < AES_BLOCK_BYTES; i++)
        out[i] = left[i] ^ right[i];
}

static bool aes_transform_dma(aes_stubs_t *as) {
    uint32_t round_key[AES_MAX_RK];
    int rounds = 0;
    if (!aes_mode_key(as, round_key, &rounds) || !as->gdma ||
        as->target->aes.dma_peripheral_id == UINT8_MAX ||
        as->hw_block_num == 0u ||
        as->hw_block_num > AES_DMA_MAX_BYTES / AES_BLOCK_BYTES ||
        as->hw_block_mode > AES_BLOCK_MODE_CFB128)
        return false;

    size_t length = (size_t)as->hw_block_num * AES_BLOCK_BYTES;
    uint8_t *input = malloc(length);
    uint8_t *output = malloc(length);
    if (!input || !output ||
        flexe_gdma_read_tx(as->gdma, as->target->aes.dma_peripheral_id,
                           input, length) != 0) {
        free(input);
        free(output);
        return false;
    }

    uint8_t iv[AES_BLOCK_BYTES];
    uint8_t tmp[AES_BLOCK_BYTES];
    uint8_t stream[AES_BLOCK_BYTES];
    aes_words_to_block(as->hw_iv, iv);
    bool decrypt = (as->hw_mode & 4u) != 0u;

    for (size_t offset = 0u; offset < length; offset += AES_BLOCK_BYTES) {
        const uint8_t *in = &input[offset];
        uint8_t *out = &output[offset];
        switch (as->hw_block_mode) {
        case AES_BLOCK_MODE_ECB:
            if (decrypt)
                aes_decrypt_block(in, out, round_key, rounds);
            else
                aes_encrypt_block(in, out, round_key, rounds);
            break;
        case AES_BLOCK_MODE_CBC:
            if (decrypt) {
                aes_decrypt_block(in, tmp, round_key, rounds);
                aes_xor_block(out, tmp, iv);
                memcpy(iv, in, AES_BLOCK_BYTES);
            } else {
                aes_xor_block(tmp, in, iv);
                aes_encrypt_block(tmp, out, round_key, rounds);
                memcpy(iv, out, AES_BLOCK_BYTES);
            }
            break;
        case AES_BLOCK_MODE_OFB:
            aes_encrypt_block(iv, stream, round_key, rounds);
            aes_xor_block(out, in, stream);
            memcpy(iv, stream, AES_BLOCK_BYTES);
            break;
        case AES_BLOCK_MODE_CTR:
            aes_encrypt_block(iv, stream, round_key, rounds);
            aes_xor_block(out, in, stream);
            aes_counter_increment(iv, as->hw_inc_sel != 0u);
            break;
        case AES_BLOCK_MODE_CFB8:
            for (unsigned byte = 0u; byte < AES_BLOCK_BYTES; byte++) {
                aes_encrypt_block(iv, stream, round_key, rounds);
                out[byte] = in[byte] ^ stream[0];
                memmove(iv, iv + 1u, AES_BLOCK_BYTES - 1u);
                iv[AES_BLOCK_BYTES - 1u] = decrypt ? in[byte] : out[byte];
            }
            break;
        case AES_BLOCK_MODE_CFB128:
            aes_encrypt_block(iv, stream, round_key, rounds);
            aes_xor_block(out, in, stream);
            memcpy(iv, decrypt ? in : out, AES_BLOCK_BYTES);
            break;
        default:
            free(input);
            free(output);
            return false;
        }
    }

    bool success = flexe_gdma_write_rx(
        as->gdma, as->target->aes.dma_peripheral_id,
        output, length) == 0;
    if (success && as->hw_block_mode != AES_BLOCK_MODE_ECB)
        aes_block_to_words(iv, as->hw_iv);
    free(input);
    free(output);
    return success;
}

static bool aes_operational(const aes_stubs_t *as) {
    return as->system_clock_enabled && !as->system_reset_asserted;
}

static void aes_update_interrupt(aes_stubs_t *as) {
    if (!as || !as->system_periph ||
        as->target->aes.interrupt_source == UINT8_MAX)
        return;
    if (as->interrupt_pending && as->hw_int_enable &&
        aes_operational(as))
        periph_assert_interrupt_status(
            as->system_periph, as->target->aes.interrupt_source, 1u);
    else
        periph_deassert_interrupt(
            as->system_periph, as->target->aes.interrupt_source);
}

static void aes_hardware_reset(aes_stubs_t *as) {
    memset(as->hw_key, 0, sizeof(as->hw_key));
    memset(as->hw_text_in, 0, sizeof(as->hw_text_in));
    memset(as->hw_text_out, 0, sizeof(as->hw_text_out));
    memset(as->hw_iv, 0, sizeof(as->hw_iv));
    as->hw_mode = 0u;
    as->hw_endian = 0u;
    as->hw_state = AES_STATE_IDLE;
    as->hw_dma_enable = 0u;
    as->hw_block_mode = AES_BLOCK_MODE_ECB;
    as->hw_block_num = 0u;
    as->hw_inc_sel = 0u;
    as->hw_aad_num = 0u;
    as->hw_valid_bits = 0u;
    as->hw_int_enable = 0u;
    as->hw_date = as->target->aes.date_reset;
    as->interrupt_pending = false;
    aes_update_interrupt(as);
}

static void aes_system_state_changed(void *ctx, bool clock_enabled,
                                     bool reset_asserted) {
    aes_stubs_t *as = ctx;
    if (!as) return;
    bool reset_rising = reset_asserted && !as->system_reset_asserted;
    as->system_clock_enabled = clock_enabled;
    as->system_reset_asserted = reset_asserted;
    if (reset_rising) aes_hardware_reset(as);
    aes_update_interrupt(as);
}

static void aes_trigger(aes_stubs_t *as) {
    if (!aes_operational(as)) return;
    as->hw_state = AES_STATE_BUSY;
    if (as->target->aes.layout == FLEXE_AES_LAYOUT_S2_S3 &&
        as->hw_dma_enable) {
        if (aes_transform_dma(as)) {
            as->hw_state = AES_STATE_DONE;
            as->interrupt_pending = true;
            aes_update_interrupt(as);
        } else {
            as->hw_state = AES_STATE_IDLE;
            if (!as->warned_dma) {
                fprintf(stderr,
                        "[aes] target %s rejected an absent or malformed "
                        "GDMA operation\n", as->target->name);
                as->warned_dma = true;
            }
        }
        return;
    }
    (void)aes_transform_direct(as);
    as->hw_state = AES_STATE_IDLE;
}

static uint32_t aes_mmio_read(void *ctx, uint32_t addr) {
    aes_stubs_t *as = ctx;
    const flexe_aes_desc_t *desc = &as->target->aes;
    if (addr < desc->base || addr >= desc->base + desc->register_size)
        return as->fallback_read
            ? as->fallback_read(as->fallback_ctx, addr) : 0u;
    uint32_t off = addr - desc->base;

    if (desc->layout == FLEXE_AES_LAYOUT_ESP32) {
        if (off >= AES32_KEY_OFF &&
            off < AES32_KEY_OFF + sizeof(as->hw_key))
            return as->hw_key[(off - AES32_KEY_OFF) / 4u];
        if (off >= AES32_TEXT_OFF &&
            off < AES32_TEXT_OFF + sizeof(as->hw_text_in))
            return as->hw_text_in[(off - AES32_TEXT_OFF) / 4u];
        switch (off) {
        case AES32_START_OFF:  return 0u;
        case AES32_IDLE_OFF:   return 1u;
        case AES32_MODE_OFF:   return as->hw_mode;
        case AES32_ENDIAN_OFF: return as->hw_endian;
        default: break;
        }
    } else if (desc->layout == FLEXE_AES_LAYOUT_S2_S3) {
        if (off < AES2_KEY_OFF + sizeof(as->hw_key))
            return as->hw_key[(off - AES2_KEY_OFF) / 4u];
        if (off >= AES2_TEXT_IN_OFF &&
            off < AES2_TEXT_IN_OFF + sizeof(as->hw_text_in))
            return as->hw_text_in[(off - AES2_TEXT_IN_OFF) / 4u];
        if (off >= AES2_TEXT_OUT_OFF &&
            off < AES2_TEXT_OUT_OFF + sizeof(as->hw_text_out))
            return as->hw_text_out[(off - AES2_TEXT_OUT_OFF) / 4u];
        if (off >= AES2_IV_OFF && off < AES2_IV_OFF + sizeof(as->hw_iv))
            return as->hw_iv[(off - AES2_IV_OFF) / 4u];
        switch (off) {
        case AES2_MODE_OFF:       return as->hw_mode;
        case AES2_ENDIAN_OFF:     return as->hw_endian;
        case AES2_TRIGGER_OFF:    return 0u;
        case AES2_STATE_OFF:      return as->hw_state;
        case AES2_DMA_ENABLE_OFF: return as->hw_dma_enable;
        case AES2_BLOCK_MODE_OFF: return as->hw_block_mode;
        case AES2_BLOCK_NUM_OFF:  return as->hw_block_num;
        case AES2_INC_SEL_OFF:    return as->hw_inc_sel;
        case AES2_AAD_NUM_OFF:    return as->hw_aad_num;
        case AES2_VALID_BITS_OFF: return as->hw_valid_bits;
        case AES2_CONTINUE_OFF:   return 0u;
        case AES2_INT_CLEAR_OFF:  return 0u;
        case AES2_INT_ENABLE_OFF: return as->hw_int_enable;
        case AES2_DATE_OFF:       return as->hw_date;
        case AES2_DMA_EXIT_OFF:   return 0u;
        default: break;
        }
    }
    return as->fallback_read
        ? as->fallback_read(as->fallback_ctx, addr) : 0u;
}

static void aes_mmio_write(void *ctx, uint32_t addr, uint32_t val) {
    aes_stubs_t *as = ctx;
    const flexe_aes_desc_t *desc = &as->target->aes;
    if (addr < desc->base || addr >= desc->base + desc->register_size) {
        if (as->fallback_write)
            as->fallback_write(as->fallback_ctx, addr, val);
        return;
    }
    uint32_t off = addr - desc->base;
    if (as->system_reset_asserted) return;

    if (desc->layout == FLEXE_AES_LAYOUT_ESP32) {
        if (off >= AES32_KEY_OFF &&
            off < AES32_KEY_OFF + sizeof(as->hw_key)) {
            as->hw_key[(off - AES32_KEY_OFF) / 4u] = val;
            return;
        }
        if (off >= AES32_TEXT_OFF &&
            off < AES32_TEXT_OFF + sizeof(as->hw_text_in)) {
            as->hw_text_in[(off - AES32_TEXT_OFF) / 4u] = val;
            return;
        }
        switch (off) {
        case AES32_START_OFF:
            if (val & 1u) aes_trigger(as);
            return;
        case AES32_MODE_OFF: as->hw_mode = val & 7u; return;
        case AES32_ENDIAN_OFF: as->hw_endian = val; return;
        case AES32_IDLE_OFF: return;
        default: break;
        }
    } else if (desc->layout == FLEXE_AES_LAYOUT_S2_S3) {
        if (off < AES2_KEY_OFF + sizeof(as->hw_key)) {
            as->hw_key[(off - AES2_KEY_OFF) / 4u] = val;
            return;
        }
        if (off >= AES2_TEXT_IN_OFF &&
            off < AES2_TEXT_IN_OFF + sizeof(as->hw_text_in)) {
            as->hw_text_in[(off - AES2_TEXT_IN_OFF) / 4u] = val;
            return;
        }
        if (off >= AES2_TEXT_OUT_OFF &&
            off < AES2_TEXT_OUT_OFF + sizeof(as->hw_text_out)) {
            as->hw_text_out[(off - AES2_TEXT_OUT_OFF) / 4u] = val;
            return;
        }
        if (off >= AES2_IV_OFF && off < AES2_IV_OFF + sizeof(as->hw_iv)) {
            as->hw_iv[(off - AES2_IV_OFF) / 4u] = val;
            return;
        }
        switch (off) {
        case AES2_MODE_OFF: as->hw_mode = val & 7u; return;
        case AES2_ENDIAN_OFF: as->hw_endian = val & 0x3Fu; return;
        case AES2_TRIGGER_OFF:
            if (val & 1u) aes_trigger(as);
            return;
        case AES2_STATE_OFF: return;
        case AES2_DMA_ENABLE_OFF: as->hw_dma_enable = val & 1u; return;
        case AES2_BLOCK_MODE_OFF: as->hw_block_mode = val & 7u; return;
        case AES2_BLOCK_NUM_OFF: as->hw_block_num = val; return;
        case AES2_INC_SEL_OFF: as->hw_inc_sel = val & 1u; return;
        case AES2_AAD_NUM_OFF: as->hw_aad_num = val; return;
        case AES2_VALID_BITS_OFF: as->hw_valid_bits = val & 0x7Fu; return;
        case AES2_CONTINUE_OFF:
            if ((val & 1u) && !as->warned_continue) {
                fprintf(stderr,
                        "[aes] target %s GCM continuation is unsupported\n",
                        as->target->name);
                as->warned_continue = true;
            }
            return;
        case AES2_INT_CLEAR_OFF:
            if (val & 1u) {
                as->interrupt_pending = false;
                aes_update_interrupt(as);
            }
            return;
        case AES2_INT_ENABLE_OFF:
            as->hw_int_enable = val & 1u;
            aes_update_interrupt(as);
            return;
        case AES2_DATE_OFF:
            as->hw_date = val & 0x3FFFFFFFu;
            return;
        case AES2_DMA_EXIT_OFF:
            as->hw_state = AES_STATE_IDLE;
            return;
        default: break;
        }
    }
    if (as->fallback_write)
        as->fallback_write(as->fallback_ctx, addr, val);
}

/* ===== Hardware acquire/release stubs ===== */

static void stub_aes_acquire_hardware(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    aes_return_void(cpu);
}

static void stub_aes_release_hardware(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    aes_return_void(cpu);
}

/* ===== HAL stub implementations ===== */

/*
 * aes_hal_setkey(key_ptr, key_bytes, mode)
 *
 * key_ptr:   pointer to key in emulator memory
 * key_bytes: 16 (AES-128), 24 (AES-192), or 32 (AES-256)
 * mode:      0=decrypt, 1=encrypt
 *
 * Returns key_bytes (for fault injection check in ESP-IDF).
 */
static void stub_aes_hal_setkey(xtensa_cpu_t *cpu, void *ctx) {
    aes_stubs_t *as = ctx;
    uint32_t key_ptr   = aes_arg(cpu, 0);
    uint32_t key_bytes = aes_arg(cpu, 1);
    uint32_t mode      = aes_arg(cpu, 2);
    int c = cpu->core_id;

    if (key_bytes > 32) key_bytes = 32;

    /* Read key from emulator memory */
    uint8_t key[32];
    for (uint32_t i = 0; i < key_bytes; i++)
        key[i] = mem_read8(cpu->mem, key_ptr + i);

    as->mode[c] = (int)mode;
    as->nr[c] = aes_key_expand(key, (int)key_bytes, as->round_key[c]);

    aes_return(cpu, key_bytes);
}

/*
 * aes_hal_transform_block(input_block, output_block)
 *
 * Encrypts or decrypts a single 128-bit block using the previously set key.
 */
static void stub_aes_hal_transform_block(xtensa_cpu_t *cpu, void *ctx) {
    aes_stubs_t *as = ctx;
    uint32_t in_ptr  = aes_arg(cpu, 0);
    uint32_t out_ptr = aes_arg(cpu, 1);
    int c = cpu->core_id;

    /* Read 16-byte input block from emulator memory */
    uint8_t in[16], out[16];
    for (int i = 0; i < 16; i++)
        in[i] = mem_read8(cpu->mem, in_ptr + (uint32_t)i);

    if (as->mode[c] == AES_MODE_ENCRYPT)
        aes_encrypt_block(in, out, as->round_key[c], as->nr[c]);
    else
        aes_decrypt_block(in, out, as->round_key[c], as->nr[c]);

    /* Write 16-byte output block to emulator memory */
    for (int i = 0; i < 16; i++)
        mem_write8(cpu->mem, out_ptr + (uint32_t)i, out[i]);

    aes_return_void(cpu);
}

/* ===== Public API ===== */

static bool aes_geometry_valid(const flexe_target_desc_t *target) {
    if (!target || !(target->capabilities & FLEXE_TARGET_CAP_AES_V1))
        return false;
    const flexe_aes_desc_t *desc = &target->aes;
    uint32_t minimum = desc->layout == FLEXE_AES_LAYOUT_ESP32 ? 0x44u :
        desc->layout == FLEXE_AES_LAYOUT_S2_S3 ? 0xBCu : 0u;
    if (minimum == 0u || desc->base < target->peripheral_start ||
        desc->base >= target->peripheral_end ||
        (desc->base & 0xFFFu) != 0u ||
        desc->register_size != 0x1000u ||
        desc->register_size > target->peripheral_end - desc->base ||
        desc->register_size < minimum ||
        desc->key_size_mask == 0u || (desc->key_size_mask & ~0x07u) != 0u)
        return false;
    if (desc->layout == FLEXE_AES_LAYOUT_ESP32)
        return desc->dma_peripheral_id == UINT8_MAX &&
               desc->interrupt_source == UINT8_MAX;
    return desc->dma_peripheral_id != UINT8_MAX &&
           (target->capabilities & FLEXE_TARGET_CAP_GDMA_V1) &&
           desc->interrupt_source != UINT8_MAX &&
           (target->capabilities & FLEXE_TARGET_CAP_INTERRUPT_MATRIX_V1) &&
           desc->interrupt_source < target->interrupt_matrix.source_count;
}

aes_stubs_t *aes_stubs_create(xtensa_cpu_t *cpu, flexe_gdma_t *gdma) {
    if (!cpu || !cpu->mem || !aes_geometry_valid(mem_target(cpu->mem)))
        return NULL;
    aes_stubs_t *as = calloc(1, sizeof(*as));
    if (!as) return NULL;
    as->cpu = cpu;
    as->target = mem_target(cpu->mem);
    as->gdma = gdma;
    as->system_clock_enabled = true;
    as->nr[0] = as->nr[1] = 10;   /* default AES-128 */
    as->mode[0] = as->mode[1] = AES_MODE_ENCRYPT;
    aes_hardware_reset(as);

    uint32_t page = (as->target->aes.base -
                     as->target->peripheral_start) / 0x1000u;
    as->fallback_read = cpu->mem->mmio[page].read;
    as->fallback_write = cpu->mem->mmio[page].write;
    as->fallback_ctx = cpu->mem->mmio[page].ctx;
    if (mem_register_mmio_range(cpu->mem, as->target->aes.base,
                                as->target->aes.register_size,
                                aes_mmio_read, aes_mmio_write, as) != 0) {
        free(as);
        return NULL;
    }
    return as;
}

int aes_stubs_attach_system_clock(aes_stubs_t *as,
                                  esp32_periph_t *periph) {
    if (!as || !periph) return -1;
    if (as->system_periph) {
        if (as->target->aes.interrupt_source != UINT8_MAX)
            periph_deassert_interrupt(
                as->system_periph, as->target->aes.interrupt_source);
        (void)periph_set_system_state_handler(
            as->system_periph, FLEXE_SYSTEM_DEVICE_AES, 0u, NULL, NULL);
    }
    as->system_periph = periph;
    if (periph_set_system_state_handler(
            periph, FLEXE_SYSTEM_DEVICE_AES, 0u,
            aes_system_state_changed, as) != 0) {
        as->system_periph = NULL;
        as->system_clock_enabled = true;
        as->system_reset_asserted = false;
        return -1;
    }
    return 0;
}

void aes_stubs_destroy(aes_stubs_t *as) {
    if (!as) return;
    if (as->system_periph) {
        if (as->target->aes.interrupt_source != UINT8_MAX)
            periph_deassert_interrupt(
                as->system_periph, as->target->aes.interrupt_source);
        (void)periph_set_system_state_handler(
            as->system_periph, FLEXE_SYSTEM_DEVICE_AES, 0u, NULL, NULL);
    }
    (void)mem_register_mmio_range(
        as->cpu->mem, as->target->aes.base, as->target->aes.register_size,
        as->fallback_read, as->fallback_write, as->fallback_ctx);
    free(as);
}

int aes_stubs_hook_symbols(aes_stubs_t *as, const elf_symbols_t *syms) {
    if (!as || !syms ||
        as->target->aes.layout != FLEXE_AES_LAYOUT_ESP32)
        return 0;

    esp32_rom_stubs_t *rom = as->cpu->pc_hook_ctx;
    if (!rom) return 0;
    as->rom = rom;

    int hooked = 0;
    struct {
        const char *name;
        rom_stub_fn fn;
    } hooks[] = {
        { "aes_hal_setkey",              stub_aes_hal_setkey },
        { "aes_hal_transform_block",     stub_aes_hal_transform_block },
        { "esp_aes_acquire_hardware",    stub_aes_acquire_hardware },
        { "esp_aes_release_hardware",    stub_aes_release_hardware },
        { NULL, NULL }
    };

    for (int i = 0; hooks[i].name; i++) {
        uint32_t addr;
        if (elf_symbols_find(syms, hooks[i].name, &addr) == 0) {
            rom_stubs_register_ctx(rom, addr, hooks[i].fn, hooks[i].name, as);
            hooked++;
        }
    }

    return hooked;
}
