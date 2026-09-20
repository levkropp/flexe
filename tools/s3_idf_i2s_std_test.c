/* Replay the stock ESP-IDF 5.3 standard-I2S driver through S3 I2S v2,
 * circular GDMA, native FreeRTOS, GPIO matrix routing, and the host stream
 * API. No guest function is replaced. */
#include "elf_symbols.h"
#include "flexe_session.h"
#include "jit.h"
#include "memory.h"
#include "peripherals.h"
#include "target.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define I2S_DONE UINT32_C(0x1232A5D0)
#define AUDIO_BYTES 1024u
#define MAX_CYCLES UINT64_C(3000000000)
#define BATCH 10000

volatile int emu_app_running = 1;

typedef struct {
    char data[32768];
    size_t length;
    bool overflow;
} uart_capture_t;

typedef struct {
    uint8_t expected[AUDIO_BYTES];
    size_t match;
    uint64_t bytes;
    unsigned callbacks;
    unsigned pattern_matches;
    unsigned metadata_errors;
    int bclk_route;
    int ws_route;
    int data_route;
    bool sampled_routes;
    esp32_periph_t *periph;
} audio_capture_t;

static void capture_uart(void *ctx, uint8_t byte)
{
    uart_capture_t *capture = ctx;
    if (capture->length + 1u >= sizeof(capture->data)) {
        capture->overflow = true;
        return;
    }
    capture->data[capture->length++] = (char)byte;
    capture->data[capture->length] = '\0';
}

static void capture_audio(void *ctx, int port, const uint8_t *data,
                          size_t length, uint32_t sample_rate,
                          uint8_t bits_per_sample, uint8_t channels)
{
    audio_capture_t *capture = ctx;
    capture->callbacks++;
    capture->bytes += length;
    if (port != 0 || sample_rate != 48000u ||
        bits_per_sample != 16u || channels != 2u)
        capture->metadata_errors++;
    if (!capture->sampled_routes) {
        capture->bclk_route = periph_gpio_out_signal(capture->periph, 4);
        capture->ws_route = periph_gpio_out_signal(capture->periph, 5);
        capture->data_route = periph_gpio_out_signal(capture->periph, 6);
        capture->sampled_routes = true;
    }
    for (size_t index = 0u; index < length; index++) {
        uint8_t byte = data[index];
        if (byte == capture->expected[capture->match]) {
            capture->match++;
            if (capture->match == sizeof(capture->expected)) {
                capture->pattern_matches++;
                capture->match = 0u;
            }
        } else {
            capture->match = byte == capture->expected[0] ? 1u : 0u;
        }
    }
}

static uint32_t checksum(const uint8_t *data, size_t length)
{
    uint32_t value = UINT32_C(2166136261);
    for (size_t index = 0u; index < length; index++)
        value = (value ^ data[index]) * UINT32_C(16777619);
    return value;
}

int main(int argc, char **argv)
{
    int argi = 1;
    bool disable_jit = false;
    if (argi < argc && strcmp(argv[argi], "--no-jit") == 0) {
        disable_jit = true;
        argi++;
    }
    if (argc - argi != 3) {
        fprintf(stderr,
                "usage: %s [--no-jit] FIRMWARE.bin FIRMWARE.elf ROM.elf\n",
                argv[0]);
        return 2;
    }

    elf_symbols_t *symbols = elf_symbols_load(argv[argi + 1]);
    uint32_t stage_address = 0u;
    uint32_t result_address = 0u;
    if (!symbols ||
        elf_symbols_find(symbols, "flexe_i2s_stage", &stage_address) != 0 ||
        elf_symbols_find(symbols, "flexe_i2s_result", &result_address) != 0) {
        fprintf(stderr, "error: I2S fixture marker symbols are missing\n");
        elf_symbols_destroy(symbols);
        return 2;
    }

    uart_capture_t uart = {0};
    flexe_session_config_t config = {
        .bin_path = argv[argi],
        .elf_path = argv[argi + 1],
        .rom_elf_path = argv[argi + 2],
        .native_freertos = 1,
        .disable_jit = disable_jit,
        .target = FLEXE_TARGET_ESP32S3,
        .unhandled_audit = 1,
        .uart_cb = capture_uart,
        .uart_ctx = &uart,
    };
    flexe_session_t *session = flexe_session_create(&config);
    if (!session) {
        elf_symbols_destroy(symbols);
        return 2;
    }

    esp32_periph_t *periph = flexe_session_periph(session);
    audio_capture_t audio = { .periph = periph };
    for (size_t index = 0u; index < sizeof(audio.expected); index++)
        audio.expected[index] =
            (uint8_t)((index * 73u + 19u) ^ (index >> 2u));
    if (periph_set_i2s_tx_callback(periph, 0, capture_audio, &audio) != 0) {
        fprintf(stderr, "error: could not attach I2S stream sink\n");
        flexe_session_destroy(session);
        elf_symbols_destroy(symbols);
        return 2;
    }

    xtensa_cpu_t *cpu0 = flexe_session_cpu(session, 0);
    xtensa_cpu_t *cpu1 = flexe_session_cpu(session, 1);
    xtensa_mem_t *mem = flexe_session_mem(session);
    uint32_t stage = 0u;
    while (cpu0->cycle_count < MAX_CYCLES) {
        stage = mem_read32(mem, stage_address);
        if ((stage == I2S_DONE &&
             strstr(uart.data, "I2S_STD_DONE") != NULL) ||
            (stage & UINT32_C(0xFFFF0000)) == UINT32_C(0xBAD00000) ||
            strstr(uart.data, "I2S_FAIL") != NULL ||
            uart.overflow || !cpu0->running ||
            (cpu1 && cpu1->debug_break))
            break;
        if (flexe_session_run_core(session, 0, BATCH) < 0) break;
        flexe_session_post_batch(session, BATCH);
    }

    stage = mem_read32(mem, stage_address);
    uint32_t preload = mem_read32(mem, result_address);
    uint32_t write0 = mem_read32(mem, result_address + 4u);
    uint32_t write1 = mem_read32(mem, result_address + 8u);
    uint32_t guest_checksum = mem_read32(mem, result_address + 12u);
    uint32_t failure = mem_read32(mem, result_address + 16u);
    uint64_t jit_instructions = 0u;
    jit_state_t *jit = flexe_session_jit(session);
    if (jit) jit_instructions = jit_get_stats(jit)->insns_jitted;
    unsigned unhandled = (unsigned)periph_unhandled_count(periph);
    bool ok = stage == I2S_DONE && preload == AUDIO_BYTES &&
              write0 == AUDIO_BYTES && write1 == AUDIO_BYTES &&
              guest_checksum == checksum(audio.expected,
                                         sizeof(audio.expected)) &&
              audio.callbacks >= 8u && audio.pattern_matches >= 2u &&
              audio.metadata_errors == 0u && audio.sampled_routes &&
              audio.bclk_route == 22 && audio.ws_route == 24 &&
              audio.data_route == 25 && unhandled == 0u &&
              strstr(uart.data, "I2S_STD_DONE") != NULL &&
              (disable_jit || jit_instructions != 0u);

    printf("%s: ESP-IDF S3 I2S std engine=%s stage=0x%08X "
           "preload=%u writes=%u,%u checksum=%08X callbacks=%u "
           "patterns=%u metadata_errors=%u routes=%d,%d,%d "
           "unhandled=%u cycles=%llu jit_insns=%llu\n",
           ok ? "PASS" : "FAIL", disable_jit ? "interp" : "jit",
           stage, preload, write0, write1, guest_checksum,
           audio.callbacks, audio.pattern_matches, audio.metadata_errors,
           audio.bclk_route, audio.ws_route, audio.data_route, unhandled,
           (unsigned long long)cpu0->cycle_count,
           (unsigned long long)jit_instructions);
    if (!ok)
        fprintf(stderr,
                "failure=0x%08X audio_bytes=%llu UART tail: %s\n",
                failure, (unsigned long long)audio.bytes,
                uart.data + (uart.length > 800u ? uart.length - 800u : 0u));

    flexe_session_destroy(session);
    elf_symbols_destroy(symbols);
    return ok ? 0 : 1;
}
