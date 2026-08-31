/* Run the public ESP-IDF TWAI driver through Flexe. The firmware exercises
 * the real ISR and FreeRTOS TX/RX queues; this host side is the virtual CAN
 * peer that acknowledges/captures TX and injects two RX frames. */
#include "elf_symbols.h"
#include "flexe_session.h"
#include "memory.h"
#include "peripherals.h"
#include "rom_stubs.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define SUCCESS_MARKER 0x54574149u
#define MAX_CYCLES     100000000ull
#define RESULT_COUNT   32u
#define TX_COUNT       3u

volatile int emu_app_running = 1;

typedef struct {
    periph_twai_frame_t frames[TX_COUNT];
    size_t count;
    bool overflow;
} twai_capture_t;

static periph_twai_tx_result_t capture_twai(
    void *opaque, const periph_twai_frame_t *frame) {
    twai_capture_t *capture = opaque;
    if (capture->count < TX_COUNT)
        capture->frames[capture->count] = *frame;
    else
        capture->overflow = true;
    capture->count++;
    return PERIPH_TWAI_TX_ACK;
}

static void run_batch(flexe_session_t *session, int cycles) {
    (void)flexe_session_run_core(session, 0, cycles);
    flexe_session_post_batch(session, cycles);
}

static bool frame_data_equal(const periph_twai_frame_t *frame,
                             const uint8_t *expected, size_t len) {
    return frame->data_length_code == len &&
           memcmp(frame->data, expected, len) == 0;
}

static bool captured_frames_ok(const twai_capture_t *capture) {
    static const uint8_t standard_data[8] = {
        0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17,
    };
    static const uint8_t extended_data[4] = {0xA5, 0x5A, 0xC3, 0x3C};
    static const uint8_t self_data[3] = {0xDE, 0xAD, 0x42};
    if (capture->count != TX_COUNT || capture->overflow)
        return false;
    const periph_twai_frame_t *standard = &capture->frames[0];
    const periph_twai_frame_t *extended = &capture->frames[1];
    const periph_twai_frame_t *self = &capture->frames[2];
    return standard->identifier == 0x321u && !standard->extended &&
           !standard->remote && !standard->single_shot &&
           !standard->self_reception &&
           frame_data_equal(standard, standard_data, sizeof(standard_data)) &&
           extended->identifier == 0x01ABCDE3u && extended->extended &&
           !extended->remote && !extended->single_shot &&
           !extended->self_reception &&
           frame_data_equal(extended, extended_data, sizeof(extended_data)) &&
           self->identifier == 0x5AAu && !self->extended && !self->remote &&
           self->single_shot && self->self_reception &&
           frame_data_equal(self, self_data, sizeof(self_data));
}

static void dump_twai(xtensa_mem_t *mem, flexe_session_t *session) {
    const uint32_t base = 0x3FF6B000u;
    fprintf(stderr,
            "[twai-fixture] mode=%02X cmd=%02X status=%02X "
            "irq=%02X ena=%02X btr=%02X/%02X alc=%02X ecc=%02X "
            "ewl=%02X rec=%02X tec=%02X rmc=%02X cdr=%02X\n",
            mem_read32(mem, base + 0x00u),
            mem_read32(mem, base + 0x04u),
            mem_read32(mem, base + 0x08u),
            mem_read32(mem, base + 0x0Cu),
            mem_read32(mem, base + 0x10u),
            mem_read32(mem, base + 0x18u),
            mem_read32(mem, base + 0x1Cu),
            mem_read32(mem, base + 0x2Cu),
            mem_read32(mem, base + 0x30u),
            mem_read32(mem, base + 0x34u),
            mem_read32(mem, base + 0x38u),
            mem_read32(mem, base + 0x3Cu),
            mem_read32(mem, base + 0x74u),
            mem_read32(mem, base + 0x7Cu));
    fprintf(stderr, "[twai-fixture] buffer=");
    for (unsigned i = 0; i < 13u; ++i)
        fprintf(stderr, "%s%02X", i ? "/" : "",
                mem_read32(mem, base + 0x40u + i * 4u));
    fputc('\n', stderr);

    char task_dump[4096];
    if (freertos_stubs_dump_tasks(flexe_session_frt(session), task_dump,
                                  sizeof(task_dump)) > 0)
        fprintf(stderr, "[twai-fixture] tasks:\n%s", task_dump);

    for (int index = 0;; ++index) {
        const char *name = NULL;
        uint32_t addr = 0;
        uint32_t calls = 0;
        if (rom_stubs_get_stats(flexe_session_rom(session), index, &name,
                                &addr, &calls) != 0)
            break;
        if (calls != 0 && name &&
            (strstr(name, "Queue") || strstr(name, "Port") ||
             strstr(name, "panic") || strstr(name, "stack_chk") ||
             strstr(name, "assert") || strstr(name, "abort")))
            fprintf(stderr, "[twai-fixture] stub %s@%08X calls=%u\n",
                    name, addr, calls);
    }
}

int main(int argc, char **argv) {
    int argi = 1;
    int disable_jit = 0;
    if (argi < argc && strcmp(argv[argi], "--no-jit") == 0) {
        disable_jit = 1;
        argi++;
    }
    if (argc - argi != 2) {
        fprintf(stderr,
                "usage: %s [--no-jit] FIRMWARE.bin FIRMWARE.elf\n",
                argv[0]);
        return 2;
    }

    const char *firmware_path = argv[argi];
    const char *elf_path = argv[argi + 1];
    elf_symbols_t *symbols = elf_symbols_load(elf_path);
    uint32_t stage_addr = 0;
    uint32_t command_addr = 0;
    uint32_t result_addr = 0;
    if (!symbols ||
        elf_symbols_find(symbols, "flexe_twai_stage", &stage_addr) != 0 ||
        elf_symbols_find(symbols, "flexe_twai_command", &command_addr) != 0 ||
        elf_symbols_find(symbols, "flexe_twai_result", &result_addr) != 0) {
        fprintf(stderr, "error: fixture marker symbols are missing\n");
        elf_symbols_destroy(symbols);
        return 2;
    }

    flexe_session_config_t config = {
        .bin_path = firmware_path,
        .elf_path = elf_path,
        .disable_jit = disable_jit,
    };
    flexe_session_t *session = flexe_session_create(&config);
    if (!session) {
        elf_symbols_destroy(symbols);
        return 2;
    }

    esp32_periph_t *periph = flexe_session_periph(session);
    xtensa_cpu_t *cpu = flexe_session_cpu(session, 0);
    xtensa_mem_t *mem = flexe_session_mem(session);
    twai_capture_t capture = {0};
    if (periph_set_twai_tx_callback(periph, capture_twai, &capture) != 0) {
        fprintf(stderr, "error: could not attach virtual TWAI peer\n");
        flexe_session_destroy(session);
        elf_symbols_destroy(symbols);
        return 2;
    }

    uint32_t stage = 0;
    uint32_t last_stage = UINT32_MAX;
    bool injected = false;
    int inject_standard = 0;
    int inject_extended = 0;
    while (cpu->cycle_count < MAX_CYCLES) {
        stage = mem_read32(mem, stage_addr);
        if (stage != last_stage) {
            fprintf(stderr,
                    "[twai-fixture] stage=0x%08X cycles=%llu pc=0x%08X "
                    "tx=%zu\n",
                    stage, (unsigned long long)cpu->cycle_count, cpu->pc,
                    capture.count);
            last_stage = stage;
        }
        if (stage == SUCCESS_MARKER ||
            (stage & 0xFFF00000u) == 0xBAD00000u)
            break;

        if (stage == 3u && !injected) {
            periph_twai_frame_t standard = {
                .identifier = 0x456u,
                .data_length_code = 8,
                .data = {0x20, 0x21, 0x22, 0x23,
                         0x24, 0x25, 0x26, 0x27},
            };
            periph_twai_frame_t extended = {
                .identifier = 0x0155AA55u,
                .data_length_code = 6,
                .extended = true,
                .remote = true,
            };
            inject_standard = periph_twai_rx_inject(periph, &standard);
            inject_extended = periph_twai_rx_inject(periph, &extended);
            injected = true;
            mem_write32(mem, command_addr, 1u);
        }

        run_batch(session, 10000);
    }

    stage = mem_read32(mem, stage_addr);
    uint32_t results[RESULT_COUNT];
    for (unsigned i = 0; i < RESULT_COUNT; ++i)
        results[i] = mem_read32(mem, result_addr + i * 4u);
    bool frames_ok = captured_frames_ok(&capture);
    size_t rx_pending = periph_twai_rx_pending(periph);
    int unhandled = periph_unhandled_count(periph);
    int unregistered = rom_stubs_unregistered_count(
        flexe_session_rom(session));

    if (stage != SUCCESS_MARKER)
        dump_twai(mem, session);

    printf("engine=%s stage=0x%08X result=",
           flexe_session_jit(session) ? "jit" : "interp", stage);
    for (unsigned i = 0; i < RESULT_COUNT; ++i)
        printf("%s%08X", i ? "/" : "", results[i]);
    printf(" tx=%zu frames=%d overflow=%d inject=%d/%d pending=%zu "
           "unhandled=%d unregistered=%d cycles=%llu\n",
           capture.count, frames_ok, capture.overflow,
           inject_standard, inject_extended, rx_pending,
           unhandled, unregistered,
           (unsigned long long)cpu->cycle_count);

    bool results_ok = true;
    for (unsigned i = 0; i < RESULT_COUNT; ++i) {
        if (i == 9u || i == 10u || i == 11u || i == 13u || i == 15u ||
            i == 16u || i == 17u || i == 18u || i == 20u || i == 21u ||
            i == 23u)
            continue;
        if (results[i] != 0u)
            results_ok = false;
    }
    bool guest_data_ok =
        results[9] == 0x5AAu && results[10] == (3u << 16) &&
        (results[11] & 0x00FFFFFFu) == 0x0042ADDEu &&
        (results[13] & 0x7u) == 0x7u &&
        results[15] == 0x456u && results[16] == (8u << 16) &&
        results[17] == 0x23222120u && results[18] == 0x27262524u &&
        results[20] == 0x0155AA55u &&
        results[21] == ((6u << 16) | 0x3u) &&
        (results[23] & 0x4u) != 0u;
    int ok = stage == SUCCESS_MARKER && results_ok && guest_data_ok &&
             frames_ok && injected && inject_standard == 1 &&
             inject_extended == 1 && rx_pending == 0u &&
             unhandled == 0 && unregistered == 0;

    flexe_session_destroy(session);
    elf_symbols_destroy(symbols);
    return ok ? 0 : 1;
}
