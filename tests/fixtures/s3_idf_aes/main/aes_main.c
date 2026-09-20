/* Exercise ESP-IDF 5.3's public mbedTLS AES API through the S3 hardware and
 * shared-GDMA drivers. The long CBC operation deliberately crosses the
 * driver's interrupt threshold and therefore covers its ISR/semaphore path. */
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "mbedtls/aes.h"

#define AES_DONE UINT32_C(0x41455344)
#define LONG_BYTES 4096u

volatile uint32_t flexe_aes_stage;
volatile uint32_t flexe_aes_result[8];

static uint8_t long_plain[LONG_BYTES];
static uint8_t long_cipher[LONG_BYTES];
static uint8_t long_result[LONG_BYTES];

static void fail(uint32_t stage, int detail)
{
    flexe_aes_result[7] = (uint32_t)detail;
    flexe_aes_stage = UINT32_C(0xBAD00000) | stage;
    printf("AES_FAIL stage=%u detail=%d\n", (unsigned)stage, detail);
    fflush(stdout);
    for (;;) vTaskDelay(pdMS_TO_TICKS(100));
}

static void require_ok(uint32_t stage, int result)
{
    if (result != 0) fail(stage, result);
}

static void require_bytes(uint32_t stage, const uint8_t *actual,
                          const uint8_t *expected, size_t length)
{
    for (size_t index = 0; index < length; index++)
        if (actual[index] != expected[index])
            fail(stage, (int)index);
}

static uint32_t checksum(const uint8_t *data, size_t length)
{
    uint32_t value = UINT32_C(2166136261);
    for (size_t index = 0; index < length; index++) {
        value ^= data[index];
        value *= UINT32_C(16777619);
    }
    return value;
}

void app_main(void)
{
    static const uint8_t key128[16] = {
        0x2b,0x7e,0x15,0x16,0x28,0xae,0xd2,0xa6,
        0xab,0xf7,0x15,0x88,0x09,0xcf,0x4f,0x3c,
    };
    static const uint8_t key256[32] = {
        0x60,0x3d,0xeb,0x10,0x15,0xca,0x71,0xbe,
        0x2b,0x73,0xae,0xf0,0x85,0x7d,0x77,0x81,
        0x1f,0x35,0x2c,0x07,0x3b,0x61,0x08,0xd7,
        0x2d,0x98,0x10,0xa3,0x09,0x14,0xdf,0xf4,
    };
    static const uint8_t iv_initial[16] = {
        0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,
        0x08,0x09,0x0a,0x0b,0x0c,0x0d,0x0e,0x0f,
    };
    static const uint8_t counter_initial[16] = {
        0xf0,0xf1,0xf2,0xf3,0xf4,0xf5,0xf6,0xf7,
        0xf8,0xf9,0xfa,0xfb,0xfc,0xfd,0xfe,0xff,
    };
    static const uint8_t plain[32] = {
        0x6b,0xc1,0xbe,0xe2,0x2e,0x40,0x9f,0x96,
        0xe9,0x3d,0x7e,0x11,0x73,0x93,0x17,0x2a,
        0xae,0x2d,0x8a,0x57,0x1e,0x03,0xac,0x9c,
        0x9e,0xb7,0x6f,0xac,0x45,0xaf,0x8e,0x51,
    };
    static const uint8_t ctr_plain[37] = {
        0x6b,0xc1,0xbe,0xe2,0x2e,0x40,0x9f,0x96,
        0xe9,0x3d,0x7e,0x11,0x73,0x93,0x17,0x2a,
        0xae,0x2d,0x8a,0x57,0x1e,0x03,0xac,0x9c,
        0x9e,0xb7,0x6f,0xac,0x45,0xaf,0x8e,0x51,
        0x30,0xc8,0x1c,0x46,0xa3,
    };
    static const uint8_t ecb128[16] = {
        0x3a,0xd7,0x7b,0xb4,0x0d,0x7a,0x36,0x60,
        0xa8,0x9e,0xca,0xf3,0x24,0x66,0xef,0x97,
    };
    static const uint8_t ecb256[16] = {
        0xf3,0xee,0xd1,0xbd,0xb5,0xd2,0xa0,0x3c,
        0x06,0x4b,0x5a,0x7e,0x3d,0xb1,0x81,0xf8,
    };
    static const uint8_t cbc[32] = {
        0x76,0x49,0xab,0xac,0x81,0x19,0xb2,0x46,
        0xce,0xe9,0x8e,0x9b,0x12,0xe9,0x19,0x7d,
        0x50,0x86,0xcb,0x9b,0x50,0x72,0x19,0xee,
        0x95,0xdb,0x11,0x3a,0x91,0x76,0x78,0xb2,
    };
    static const uint8_t ctr37[37] = {
        0x87,0x4d,0x61,0x91,0xb6,0x20,0xe3,0x26,
        0x1b,0xef,0x68,0x64,0x99,0x0d,0xb6,0xce,
        0x98,0x06,0xf6,0x6b,0x79,0x70,0xfd,0xff,
        0x86,0x17,0x18,0x7b,0xb9,0xff,0xfd,0xff,
        0x5a,0xe4,0xdf,0x3e,0xdb,
    };
    static const uint8_t ofb[32] = {
        0x3b,0x3f,0xd9,0x2e,0xb7,0x2d,0xad,0x20,
        0x33,0x34,0x49,0xf8,0xe8,0x3c,0xfb,0x4a,
        0x77,0x89,0x50,0x8d,0x16,0x91,0x8f,0x03,
        0xf5,0x3c,0x52,0xda,0xc5,0x4e,0xd8,0x25,
    };
    static const uint8_t cfb128[32] = {
        0x3b,0x3f,0xd9,0x2e,0xb7,0x2d,0xad,0x20,
        0x33,0x34,0x49,0xf8,0xe8,0x3c,0xfb,0x4a,
        0xc8,0xa6,0x45,0x37,0xa0,0xb3,0xa9,0x3f,
        0xcd,0xe3,0xcd,0xad,0x9f,0x1c,0xe5,0x8b,
    };

    mbedtls_aes_context context;
    uint8_t output[48] = {0};
    uint8_t result[48] = {0};
    uint8_t iv[16];
    uint8_t stream[16] = {0};
    size_t offset = 0;
    flexe_aes_stage = 1u;
    mbedtls_aes_init(&context);

    require_ok(1u, mbedtls_aes_setkey_enc(&context, key128, 128));
    require_ok(2u, mbedtls_aes_crypt_ecb(
        &context, MBEDTLS_AES_ENCRYPT, plain, output));
    require_bytes(3u, output, ecb128, sizeof(ecb128));
    require_ok(4u, mbedtls_aes_setkey_dec(&context, key128, 128));
    require_ok(5u, mbedtls_aes_crypt_ecb(
        &context, MBEDTLS_AES_DECRYPT, output, result));
    require_bytes(6u, result, plain, 16u);

    require_ok(7u, mbedtls_aes_setkey_enc(&context, key256, 256));
    require_ok(8u, mbedtls_aes_crypt_ecb(
        &context, MBEDTLS_AES_ENCRYPT, plain, output));
    require_bytes(9u, output, ecb256, sizeof(ecb256));

    require_ok(10u, mbedtls_aes_setkey_enc(&context, key128, 128));
    memcpy(iv, iv_initial, sizeof(iv));
    require_ok(11u, mbedtls_aes_crypt_cbc(
        &context, MBEDTLS_AES_ENCRYPT, sizeof(plain), iv, plain, output));
    require_bytes(12u, output, cbc, sizeof(cbc));
    require_ok(13u, mbedtls_aes_setkey_dec(&context, key128, 128));
    memcpy(iv, iv_initial, sizeof(iv));
    require_ok(14u, mbedtls_aes_crypt_cbc(
        &context, MBEDTLS_AES_DECRYPT, sizeof(cbc), iv, cbc, result));
    require_bytes(15u, result, plain, sizeof(plain));

    require_ok(16u, mbedtls_aes_setkey_enc(&context, key128, 128));
    memcpy(iv, counter_initial, sizeof(iv));
    memset(stream, 0, sizeof(stream));
    offset = 0u;
    require_ok(17u, mbedtls_aes_crypt_ctr(
        &context, sizeof(ctr_plain), &offset, iv, stream,
        ctr_plain, output));
    require_bytes(18u, output, ctr37, sizeof(ctr37));

    memcpy(iv, iv_initial, sizeof(iv));
    offset = 0u;
    require_ok(19u, mbedtls_aes_crypt_ofb(
        &context, sizeof(plain), &offset, iv, plain, output));
    require_bytes(20u, output, ofb, sizeof(ofb));
    memcpy(iv, iv_initial, sizeof(iv));
    offset = 0u;
    require_ok(21u, mbedtls_aes_crypt_cfb128(
        &context, MBEDTLS_AES_ENCRYPT, sizeof(plain), &offset,
        iv, plain, output));
    require_bytes(22u, output, cfb128, sizeof(cfb128));
    memcpy(iv, iv_initial, sizeof(iv));
    require_ok(23u, mbedtls_aes_crypt_cfb8(
        &context, MBEDTLS_AES_ENCRYPT, sizeof(plain), iv, plain, output));
    memcpy(iv, iv_initial, sizeof(iv));
    require_ok(24u, mbedtls_aes_crypt_cfb8(
        &context, MBEDTLS_AES_DECRYPT, sizeof(plain), iv, output, result));
    require_bytes(25u, result, plain, sizeof(plain));

    for (size_t index = 0u; index < LONG_BYTES; index++)
        long_plain[index] = (uint8_t)(index * 37u + 11u);
    require_ok(26u, mbedtls_aes_setkey_enc(&context, key256, 256));
    memcpy(iv, iv_initial, sizeof(iv));
    require_ok(27u, mbedtls_aes_crypt_cbc(
        &context, MBEDTLS_AES_ENCRYPT, LONG_BYTES, iv,
        long_plain, long_cipher));
    require_ok(28u, mbedtls_aes_setkey_dec(&context, key256, 256));
    memcpy(iv, iv_initial, sizeof(iv));
    require_ok(29u, mbedtls_aes_crypt_cbc(
        &context, MBEDTLS_AES_DECRYPT, LONG_BYTES, iv,
        long_cipher, long_result));
    require_bytes(30u, long_result, long_plain, LONG_BYTES);

    flexe_aes_result[0] = checksum(long_cipher, LONG_BYTES);
    flexe_aes_result[7] = 0u;
    flexe_aes_stage = AES_DONE;
    mbedtls_aes_free(&context);
    printf("AES_DONE modes=ecb128,ecb256,cbc,ctr,ofb,cfb128,cfb8 "
           "long=4096 checksum=%08lx\n",
           (unsigned long)flexe_aes_result[0]);
    fflush(stdout);
    for (;;) vTaskDelay(pdMS_TO_TICKS(1000));
}
