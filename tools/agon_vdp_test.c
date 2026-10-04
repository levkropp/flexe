/* Unmodified Agon VDP: UART2 MOS peer, VGA scanout, and native FreeRTOS.
 * The external release image is hash-pinned by check-agon-vdp.sh. No ELF
 * services, guest memory patches, or calls into guest application functions.
 */
#include "flexe_session.h"
#include "peripherals.h"
#include "rom_stubs.h"
#include "jit.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

volatile int emu_app_running = 1;
#define MAX_COLUMNS 1024u
#define MAX_ROWS 600u
#define BOOT_LIMIT 5000000000ull
#define BATCH 10000

typedef struct {
    uint8_t raster[MAX_ROWS][MAX_COLUMNS];
    uint8_t frame[MAX_ROWS][MAX_COLUMNS];
    uint64_t descriptors, bytes, frames;
    unsigned x, y, columns, rows, rate;
    uint8_t previous;
    bool started, overflow, complete;
} vga_t;

typedef struct {
    uint8_t bytes[4096];
    size_t len;
    bool overflow;
} uart_t;

typedef struct {
    uint8_t samples[32768];
    size_t len, non_silent, rises, first_rise, last_rise;
    uint32_t rate;
    uint8_t previous, minimum, maximum;
    bool enabled, invalid;
} audio_t;

static void capture_audio(void *opaque, int port, const uint8_t *data,
                           size_t len, uint32_t rate, uint8_t bits,
                           uint8_t channels) {
    audio_t *a = opaque;
    if (!a->enabled) return;
    a->rate = rate;
    if (bits != 16u || channels != 1u || (len & 3u)) {
        a->invalid = true;
        return;
    }
    for (size_t i = 0; i < len; i += 2u) {
        /* FabGL stores each unsigned DAC sample in the upper byte of a
         * 16-bit LCD word, swapping halfwords for the ESP32 FIFO. */
        uint8_t sample = data[(i ^ 2u) + 1u];
        if (data[i ^ 2u]) a->invalid = true;
        if (a->len == sizeof(a->samples)) { a->invalid = true; return; }
        a->samples[a->len] = sample;
        if (sample != 127u) a->non_silent++;
        if (sample < a->minimum) a->minimum = sample;
        if (sample > a->maximum) a->maximum = sample;
        if (sample > 127u && a->previous < 127u) {
            if (!a->rises) a->first_rise = a->len;
            a->last_rise = a->len;
            a->rises++;
        }
        a->previous = sample;
        a->len++;
    }
}

static uint64_t monotonic_ns(void) {
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    return (uint64_t)now.tv_sec * 1000000000ull + (uint64_t)now.tv_nsec;
}

static uint32_t digest(const uint8_t *data, size_t len) {
    uint32_t hash = 2166136261u;
    for (size_t i = 0; i < len; i++) hash = (hash ^ data[i]) * 16777619u;
    return hash;
}

static bool save_audio(const audio_t *a, const char *dir) {
    if (!dir) return true;
    char path[4096];
    if (snprintf(path, sizeof(path), "%s/tone.wav", dir) >= (int)sizeof(path)) return false;
    FILE *out = fopen(path, "wb");
    if (!out) return false;
    uint8_t header[44] = {0};
    memcpy(header, "RIFF", 4); memcpy(header + 8, "WAVEfmt ", 8);
    memcpy(header + 36, "data", 4);
    uint32_t fields[] = {(uint32_t)a->len + 36u, 16u, a->rate, a->rate, (uint32_t)a->len};
    const unsigned offsets[] = {4, 16, 24, 28, 40};
    for (unsigned i = 0; i < 5u; i++)
        for (unsigned j = 0; j < 4u; j++) header[offsets[i] + j] = (uint8_t)(fields[i] >> (j * 8u));
    header[20] = 1; header[22] = 1; header[32] = 1; header[34] = 8;
    bool ok = fwrite(header, 1, sizeof(header), out) == sizeof(header) &&
              fwrite(a->samples, 1, a->len, out) == a->len;
    return fclose(out) == 0 && ok;
}

static void capture_uart(void *opaque, uint8_t byte) {
    uart_t *u = opaque;
    if (u->len < sizeof(u->bytes)) u->bytes[u->len++] = byte;
    else u->overflow = true;
}

static void capture_vga(void *opaque, int port, const uint8_t *data,
                         size_t len, uint32_t rate, uint8_t bits,
                         uint8_t channels) {
    vga_t *v = opaque;
    v->descriptors++;
    v->bytes += len;
    v->rate = rate;
    if (bits != 8 || channels != 1 || (len & 3u)) {
        v->overflow = true;
        return;
    }
    /* The ESP32 LCD FIFO emits the upper 16-bit halfword first. FabGL
     * arranges its DMA bytes accordingly. Bits 6/7 are negative H/V sync,
     * with two bits per RGB channel in bits 0..5. */
    for (size_t i = 0; i < len; i++) {
        uint8_t pixel = data[i ^ 2u];
        if (!(pixel & 0x80u) && (v->previous & 0x80u)) {
            if (v->started && v->y < MAX_ROWS) {
                memcpy(v->frame, v->raster, sizeof(v->frame));
                v->rows = v->y + 1u;
                v->frames++;
                v->complete = true;
            }
            v->started = true;
            v->y = UINT32_MAX; /* the next HSync starts line zero */
        }
        if (!(pixel & 0x40u) && (v->previous & 0x40u)) {
            if (v->x) v->columns = v->x;
            v->x = 0;
            v->y++;
        }
        if (v->started && v->y < MAX_ROWS && v->x < MAX_COLUMNS)
            v->raster[v->y][v->x] = pixel;
        if (v->x >= MAX_COLUMNS || (v->y >= MAX_ROWS && v->y != UINT32_MAX))
            v->overflow = true;
        v->x++;
        v->previous = pixel;
    }
}

static bool send_bytes(esp32_periph_t *p, const uint8_t *data, size_t len) {
    return periph_uart_rx_inject_num(p, 2, data, len) == len;
}

static bool has_bytes(const uart_t *u, const uint8_t *data, size_t len) {
    if (u->len < len) return false;
    for (size_t i = 0; i + len <= u->len; i++)
        if (memcmp(u->bytes + i, data, len) == 0) return true;
    return false;
}

static bool step(flexe_session_t *s) {
    xtensa_cpu_t *c0 = flexe_session_cpu(s, 0);
    xtensa_cpu_t *c1 = flexe_session_cpu(s, 1);
    if (c0->debug_break || c1->debug_break || c0->breakpoint_hit || c1->breakpoint_hit)
        return false;
    if (flexe_session_run_core(s, 0, BATCH) < 0) return false;
    flexe_session_post_batch(s, BATCH);
    return !c0->breakpoint_hit && !c1->breakpoint_hit;
}

static bool frame_matches(const vga_t *v, unsigned width, unsigned height,
                           uint8_t background, uint32_t *hash_out) {
    unsigned left = width == 640u ? 144u : 72u;
    unsigned top = width == 640u ? 34u : 33u;
    unsigned stride = width == 640u ? 1u : 2u;
    unsigned total_rows = width == 640u ? 525u : 524u;
    if (!v->complete || v->columns != (width == 640u ? 800u : 400u) ||
        v->rows != total_rows) return false;
    unsigned foreground = 0;
    uint32_t hash = 2166136261u;
    for (unsigned y = 0; y < height; y++) {
        for (unsigned x = 0; x < width; x++) {
            uint8_t pixel = v->frame[top + y * stride][left + x] & 0x3fu;
            hash = (hash ^ pixel) * 16777619u;
            if (pixel == background) continue;
            /* Only the five requested glyphs may differ from the solid
             * background. The cursor is explicitly disabled in the command. */
            if (y >= 8u || x >= 40u || pixel != 0x3fu) return false;
            foreground++;
        }
    }
    if (foreground < 60u || foreground > 200u) return false;
    *hash_out = hash;
    return true;
}

static bool save_frame(const vga_t *v, unsigned width, unsigned height,
                        const char *dir, const char *name) {
    if (!dir) return true;
    char path[4096];
    if (snprintf(path, sizeof(path), "%s/%s.ppm", dir, name) >= (int)sizeof(path))
        return false;
    FILE *out = fopen(path, "wb");
    if (!out) return false;
    fprintf(out, "P6\n%u %u\n255\n", width, height);
    unsigned left = width == 640u ? 144u : 72u;
    unsigned top = width == 640u ? 34u : 33u;
    unsigned stride = width == 640u ? 1u : 2u;
    for (unsigned y = 0; y < height; y++) {
        for (unsigned x = 0; x < width; x++) {
            uint8_t p = v->frame[top + y * stride][left + x];
            uint8_t rgb[] = {(p & 3u) * 85u, ((p >> 2) & 3u) * 85u,
                             ((p >> 4) & 3u) * 85u};
            if (fwrite(rgb, 1, sizeof(rgb), out) != sizeof(rgb)) {
                fclose(out);
                return false;
            }
        }
    }
    return fclose(out) == 0;
}

#include "agon_mos_bridge.h"

int main(int argc, char **argv) {
    bool no_jit = false;
    unsigned mos_port = 0;
    const char *image = NULL, *artifacts = NULL;
    uint64_t soak_cycles = 240000000ull;
    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--no-jit")) no_jit = true;
        else if (!strcmp(argv[i], "--mos-peer") && i + 1 < argc) {
            char *end;
            unsigned long port = strtoul(argv[++i], &end, 10);
            if (*end || !port || port > 65535u) return 2;
            mos_port = (unsigned)port;
        }
        else if (!strcmp(argv[i], "--artifacts") && i + 1 < argc) artifacts = argv[++i];
        else if (!strcmp(argv[i], "--soak-cycles") && i + 1 < argc) {
            char *end;
            soak_cycles = strtoull(argv[++i], &end, 10);
            if (*end || !soak_cycles || soak_cycles > 20000000000ull) return 2;
        } else if (!image && argv[i][0] != '-') image = argv[i];
        else return 2;
    }
    if (!image) {
        fprintf(stderr, "usage: %s [--no-jit] [--artifacts DIR] [--soak-cycles N] [--mos-peer PORT] FIRMWARE.bin\n", argv[0]);
        return 2;
    }
    uint64_t wall_start = monotonic_ns();
    flexe_session_config_t cfg = {.bin_path = image, .native_freertos = 1,
                                  .disable_jit = no_jit};
    flexe_session_t *s = flexe_session_create(&cfg);
    if (!s) return 2;
    vga_t *v = calloc(1, sizeof(*v));
    uart_t u = {0};
    audio_t audio = {.minimum = 255u, .previous = 127u};
    if (!v) { flexe_session_destroy(s); return 2; }
    esp32_periph_t *p = flexe_session_periph(s);
    xtensa_cpu_t *cpu = flexe_session_cpu(s, 0);
    xtensa_cpu_t *other = flexe_session_cpu(s, 1);
    xtensa_mem_t *mem = flexe_session_mem(s);
    periph_set_i2s_tx_callback(p, 1, capture_vga, v);
    periph_set_i2s_tx_callback(p, 0, capture_audio, &audio);
    periph_set_uart_callback_num(p, 2, capture_uart, &u);
    /* Release firmware.map supplies this diagnostic stop only. It does not
     * change guest execution or decide success. */
    xtensa_set_breakpoint(cpu, 0x40088154u);
    xtensa_set_breakpoint(other, 0x40088154u);
    static const uint8_t hello[] = {23, 0, 0x80, 0x42};
    static const uint8_t handshake[] = {0x80, 1, 0x42, 0x86, 8, 0x80, 2,
                                        0xe0, 1, 80, 60, 16, 0};
    static const uint8_t red[] = {23, 1, 0, 17, 129, 12, 'F','L','E','X','E',
                                  23, 0, 0x83, 0,0,0,0,
                                  23, 0, 0x84, 100,0,100,0};
    static const uint8_t red_reply[] = {0x83, 1, 'F', 0x84, 4, 170,0,0,1};
    static const uint8_t blue[] = {22, 8, 23, 1, 0, 17, 132, 12, 'F','L','E','X','E',
                                   23, 0, 0x83, 0,0,0,0,
                                   23, 0, 0x84, 100,0,100,0};
    static const uint8_t blue_reply[] = {0x86,8,64,1,240,0,40,30,64,8,
                                         0x83,1,'F',0x84,4,0,0,170,4};
    static const uint8_t tone[] = {23,0,0x85,0,0,127,0,4,0xf4,1}; /* 1024 Hz, 500 ms */
    static const uint8_t tone_reply[] = {0x85,2,0,1};
    static const uint8_t console_on[] = {23,0,254,1};
    static const uint8_t console_off[] = {23,0,254,0};
    static const uint8_t key_reply[] = {0x81,4,'a',0,0,1};
    unsigned stage = 0, stable = 0;
    uint64_t last_frame = 0;
    uint32_t previous_hash = 0, red_hash = 0, blue_hash = 0;
    while (cpu->cycle_count < BOOT_LIMIT && stage < 5u) {
        if (!step(s)) break;
        if (stage == 0u && cpu->cycle_count >= 300000000ull) {
            if (!send_bytes(p, hello, sizeof(hello))) break;
            stage = 1u;
        } else if (stage == 1u && has_bytes(&u, handshake, sizeof(handshake))) {
            if (!send_bytes(p, red, sizeof(red))) break;
            stage = 2u;
            fprintf(stderr, "[agon] native UART2 handshake and 640x480 mode confirmed\n");
        } else if ((stage == 2u || stage == 4u) && v->frames != last_frame) {
            last_frame = v->frames;
            unsigned width = stage == 2u ? 640u : 320u;
            unsigned height = stage == 2u ? 480u : 240u;
            uint32_t hash = 0;
            bool match = frame_matches(v, width, height,
                                        stage == 2u ? 0x02u : 0x20u, &hash);
            if (match && hash == previous_hash) stable++;
            else stable = 0;
            previous_hash = hash;
            bool response = stage == 2u ? has_bytes(&u, red_reply, sizeof(red_reply)) :
                                          has_bytes(&u, blue_reply, sizeof(blue_reply));
            if (stable < 2u || !response) continue;
            if (!save_frame(v, width, height, artifacts, stage == 2u ? "red-640x480" : "blue-320x240")) break;
            fprintf(stderr, "[agon] %ux%u stable scanout digest=%08x rate=%u total=%u/%u\n",
                    width, height, hash, v->rate, v->columns, v->rows);
            if (stage == 2u) {
                red_hash = hash;
                if (!send_bytes(p, blue, sizeof(blue))) break;
                stage = 3u;
                stable = 0;
                v->complete = false;
            } else { blue_hash = hash; stage = 5u; }
        } else if (stage == 3u && has_bytes(&u, blue_reply, sizeof(blue_reply))) {
            stage = 4u;
            stable = 0;
        }
    }
    bool audio_ok = false;
    if (stage == 5u) {
        audio.enabled = true;
        uint64_t start = cpu->cycle_count;
        if (send_bytes(p, tone, sizeof(tone))) {
            while (cpu->cycle_count - start < 240000000ull)
                if (!step(s)) break;
        }
        audio.enabled = false;
        double frequency = audio.rises > 1u ?
            (double)(audio.rises - 1u) * audio.rate / (audio.last_rise - audio.first_rise) : 0;
        audio_ok = has_bytes(&u, tone_reply, sizeof(tone_reply)) && !audio.invalid &&
                   audio.rate >= 16380u && audio.rate <= 16390u &&
                   frequency > 1020 && frequency < 1028 &&
                   audio.non_silent > 7800u && audio.non_silent < 8600u &&
                   audio.minimum < 100u && audio.maximum > 154u &&
                   audio.len > 15000u && audio.samples[audio.len - 1u] == 127u;
        audio_ok = save_audio(&audio, artifacts) && audio_ok;
        fprintf(stderr, "[agon] audio ok=%d samples=%zu non_silent=%zu rate=%u tone=%.3fHz range=%u/%u rises=%zu\n",
                audio_ok, audio.len, audio.non_silent, audio.rate, frequency,
                audio.minimum, audio.maximum, audio.rises);
    }
    bool keyboard_ok = false;
    if (stage == 5u && send_bytes(p, console_on, sizeof(console_on))) {
        uint64_t start = cpu->cycle_count;
        while (cpu->cycle_count - start < 2400000ull)
            if (!step(s)) break;
        const uint8_t key = 'a';
        periph_uart_rx_inject_num(p, 0, &key, 1);
        while (cpu->cycle_count - start < 24000000ull &&
               !has_bytes(&u, key_reply, sizeof(key_reply)))
            if (!step(s)) break;
        keyboard_ok = has_bytes(&u, key_reply, sizeof(key_reply));
        send_bytes(p, console_off, sizeof(console_off));
        fprintf(stderr, "[agon] serial-console keyboard ok=%d\n", keyboard_ok);
    }
    uint64_t soak_start = cpu->cycle_count, frames_start = v->frames;
    uint64_t retired_start = cpu->insn_count + other->insn_count;
    jit_state_t *jit_state = flexe_session_jit(s);
    const jit_stats_t *jit = jit_state ? jit_get_stats(jit_state) : NULL;
    uint64_t native_start = jit ? jit->insns_jitted : 0;
    uint64_t soak_wall_start = monotonic_ns();
    if (stage == 5u) {
        while (cpu->cycle_count - soak_start < soak_cycles)
            if (!step(s)) break;
    }
    uint64_t soak_wall = monotonic_ns() - soak_wall_start;
    uint64_t elapsed = cpu->cycle_count - soak_start;
    uint32_t final_hash = 0;
    int unhandled = periph_unhandled_count(p);
    int unregistered = rom_stubs_unregistered_count(flexe_session_rom(s));
    uint64_t unmapped = mem_unmapped_count(mem);
    uint64_t retired = cpu->insn_count + other->insn_count;
    uint64_t native = jit ? jit->insns_jitted : 0;
    uint64_t soak_frames = v->frames - frames_start;
    uint64_t expected_frames = elapsed * v->rate / (240000000ull * 400u * 524u);
    bool timing_ok = soak_frames + 2u >= expected_frames &&
                     soak_frames <= expected_frames + 2u;
    bool uart_ok = u.len == sizeof(handshake) + sizeof(red_reply) + sizeof(blue_reply) + sizeof(tone_reply) + sizeof(key_reply) &&
                   memcmp(u.bytes, handshake, sizeof(handshake)) == 0 &&
                   memcmp(u.bytes + sizeof(handshake), red_reply, sizeof(red_reply)) == 0 &&
                   memcmp(u.bytes + sizeof(handshake) + sizeof(red_reply),
                          blue_reply, sizeof(blue_reply)) == 0 &&
                   memcmp(u.bytes + sizeof(handshake) + sizeof(red_reply) + sizeof(blue_reply),
                          tone_reply, sizeof(tone_reply)) == 0 &&
                   memcmp(u.bytes + sizeof(handshake) + sizeof(red_reply) + sizeof(blue_reply) + sizeof(tone_reply),
                          key_reply, sizeof(key_reply)) == 0;
    bool ok = stage == 5u && elapsed >= soak_cycles && !v->overflow && !u.overflow &&
              uart_ok && timing_ok && audio_ok && keyboard_ok &&
              frame_matches(v, 320, 240, 0x20u, &final_hash) && final_hash == blue_hash &&
              !cpu->debug_break && !other->debug_break &&
              !cpu->breakpoint_hit && !other->breakpoint_hit &&
              !unhandled && !unregistered && !unmapped && (no_jit || native > 0);
    double wall = (double)(monotonic_ns() - wall_start) / 1e9;
    printf("%s engine=%s stage=%u wall=%.6f soak_wall=%.6f soak_cycles=%llu "
           "cycles=%llu retired=%llu native=%llu soak_retired=%llu soak_native=%llu "
           "frames=%llu soak_frames=%llu "
           "pixel_hz=%u geometry=%u/%u uart=%zu uart_digest=%08x red=%08x blue=%08x audio=%d key=%d "
           "unhandled=%d unregistered=%d unmapped=%llu pc=%08x/%08x\n",
           ok ? "PASS" : "FAIL", no_jit ? "interp" : "jit", stage, wall,
           (double)soak_wall / 1e9, (unsigned long long)elapsed,
           (unsigned long long)cpu->cycle_count, (unsigned long long)retired,
           (unsigned long long)native, (unsigned long long)(retired - retired_start),
           (unsigned long long)(native - native_start), (unsigned long long)v->frames,
           (unsigned long long)soak_frames, v->rate,
           v->columns, v->rows, u.len, digest(u.bytes, u.len), red_hash, blue_hash, audio_ok, keyboard_ok,
           unhandled, unregistered, (unsigned long long)unmapped, cpu->pc, other->pc);
    if (!ok) {
        fprintf(stderr, "[agon] UART2:");
        for (size_t i = 0; i < u.len; i++) fprintf(stderr, " %02x", u.bytes[i]);
        fprintf(stderr, "\n[agon] video overflow=%d rows=%u columns=%u rate=%u hash=%08x stable=%u\n",
                v->overflow, v->rows, v->columns, v->rate, final_hash, stable);
        if (v->complete) save_frame(v, v->columns == 800 ? 640 : 320,
                                      v->columns == 800 ? 480 : 240, artifacts, "failure");
    }
    if (getenv("FLEXE_JIT_STATS") && jit_state)
        jit_print_stats(jit_state, retired);
    xtensa_profile_report();
    if (ok && mos_port) ok = run_mos(s, v, mos_port, artifacts);
    free(v);
    flexe_session_destroy(s);
    return ok ? 0 : 1;
}
