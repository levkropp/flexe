/* Run the compiled Arduino/ESP-IDF I2S fixture through Flexe's public
 * session API. The firmware uses only the stock driver/i2s.h interface; this
 * host side injects RX PCM and validates TX PCM at the emulator boundary. */
#include "elf_symbols.h"
#include "flexe_session.h"
#include "memory.h"
#include "peripherals.h"
#include "rom_stubs.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define SUCCESS_MARKER 0x12D5DAA0u
#define MAX_CYCLES     30000000ull
#define DMA_BYTES      128u

volatile int emu_app_running = 1;

typedef struct {
    unsigned descriptors;
    size_t bytes;
    bool pattern_seen;
    uint32_t sample_rate;
    uint8_t bits_per_sample;
    uint8_t channels;
} i2s_capture_t;

static void capture_i2s(void *opaque, int port, const uint8_t *data,
                        size_t len, uint32_t sample_rate,
                        uint8_t bits_per_sample, uint8_t channels) {
    i2s_capture_t *capture = opaque;
    if (port != 0)
        return;
    capture->descriptors++;
    capture->bytes += len;
    capture->sample_rate = sample_rate;
    capture->bits_per_sample = bits_per_sample;
    capture->channels = channels;
    if (len != DMA_BYTES)
        return;
    for (size_t i = 0; i < DMA_BYTES; ++i) {
        if (data[i] != (uint8_t)(0x5Au ^ i))
            return;
    }
    capture->pattern_seen = true;
}

int main(int argc, char **argv) {
    if (argc != 3) {
        fprintf(stderr, "usage: %s FIRMWARE.bin FIRMWARE.elf\n", argv[0]);
        return 2;
    }

    elf_symbols_t *symbols = elf_symbols_load(argv[2]);
    uint32_t stage_addr = 0;
    uint32_t result_addr = 0;
    if (!symbols ||
        elf_symbols_find(symbols, "flexe_i2s_stage", &stage_addr) != 0 ||
        elf_symbols_find(symbols, "flexe_i2s_result", &result_addr) != 0) {
        fprintf(stderr, "error: fixture marker symbols are missing\n");
        elf_symbols_destroy(symbols);
        return 2;
    }

    flexe_session_config_t config = {
        .bin_path = argv[1],
        .elf_path = argv[2],
    };
    flexe_session_t *session = flexe_session_create(&config);
    if (!session) {
        elf_symbols_destroy(symbols);
        return 2;
    }

    esp32_periph_t *periph = flexe_session_periph(session);
    uint8_t rx_pcm[DMA_BYTES * 4u];
    for (size_t i = 0; i < sizeof(rx_pcm); ++i)
        rx_pcm[i] = (uint8_t)(0xA0u ^ (i % DMA_BYTES));
    size_t injected = periph_i2s_rx_inject(periph, 0, rx_pcm, sizeof(rx_pcm));
    i2s_capture_t capture = {0};
    if (periph_set_i2s_tx_callback(periph, 0, capture_i2s, &capture) != 0 ||
        injected != sizeof(rx_pcm)) {
        fprintf(stderr, "error: could not configure virtual I2S endpoint\n");
        flexe_session_destroy(session);
        elf_symbols_destroy(symbols);
        return 2;
    }

    xtensa_cpu_t *cpu = flexe_session_cpu(session, 0);
    xtensa_mem_t *mem = flexe_session_mem(session);
    uint32_t stage = 0;
    uint32_t last_stage = UINT32_MAX;
    while (cpu->cycle_count < MAX_CYCLES) {
        stage = mem_read32(mem, stage_addr);
        if (stage != last_stage) {
            fprintf(stderr, "[i2s-fixture] stage=0x%08X cycles=%llu pc=0x%08X\n",
                    stage, (unsigned long long)cpu->cycle_count, cpu->pc);
            if (stage >= 2 && stage < SUCCESS_MARKER) {
                const uint32_t base = 0x3FF4F000u;
                fprintf(stderr,
                        "[i2s-fixture] conf=%08X irq=%08X/%08X/%08X "
                        "fifo=%08X out=%08X in=%08X eof=%08X/%08X\n",
                        mem_read32(mem, base + 0x08u),
                        mem_read32(mem, base + 0x0Cu),
                        mem_read32(mem, base + 0x10u),
                        mem_read32(mem, base + 0x14u),
                        mem_read32(mem, base + 0x20u),
                        mem_read32(mem, base + 0x30u),
                        mem_read32(mem, base + 0x34u),
                        mem_read32(mem, base + 0x38u),
                        mem_read32(mem, base + 0x3Cu));
            }
            last_stage = stage;
        }
        if (stage == SUCCESS_MARKER ||
            (stage & 0xFFF00000u) == 0xBAD00000u)
            break;
        (void)flexe_session_run_core(session, 0, 10000);
        flexe_session_post_batch(session, 10000);
    }

    stage = mem_read32(mem, stage_addr);
    uint32_t results[4];
    for (unsigned i = 0; i < 4; ++i)
        results[i] = mem_read32(mem, result_addr + i * 4u);
    size_t rx_pending = periph_i2s_rx_pending(periph, 0);
    int unhandled = periph_unhandled_count(periph);
    int unregistered = rom_stubs_unregistered_count(
        flexe_session_rom(session));

    printf("stage=0x%08X result=%u/%u/%u/0x%08X "
           "tx_desc=%u tx_bytes=%zu tx_pattern=%d audio=%u/%u/%u "
           "rx=%zu/%zu unhandled=%d unregistered=%d cycles=%llu\n",
           stage, results[0], results[1], results[2], results[3],
           capture.descriptors, capture.bytes, capture.pattern_seen,
           capture.sample_rate, capture.bits_per_sample, capture.channels,
           injected, rx_pending, unhandled, unregistered,
           (unsigned long long)cpu->cycle_count);

    int ok = stage == SUCCESS_MARKER && results[0] == 0 &&
             results[1] == DMA_BYTES && results[2] == DMA_BYTES &&
             results[3] == 0 && capture.pattern_seen &&
             capture.bits_per_sample == 16 && capture.channels == 2 &&
             capture.sample_rate > 0 && injected == sizeof(rx_pcm) &&
             rx_pending <= sizeof(rx_pcm) - DMA_BYTES &&
             unhandled == 0 && unregistered == 0;

    flexe_session_destroy(session);
    elf_symbols_destroy(symbols);
    return ok ? 0 : 1;
}
