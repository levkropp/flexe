/* Run Espressif's compiled ESP32 EMAC HAL or public MAC driver through Flexe.
 * The guest owns the descriptor rings and ISR; this host side is the virtual
 * Ethernet wire/PHY that captures TX, services MDIO, and injects RX frames. */
#include "elf_symbols.h"
#include "flexe_session.h"
#include "memory.h"
#include "peripherals.h"
#include "rom_stubs.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define SUCCESS_MARKER 0x454D4143u
#define MAX_CYCLES     100000000ull
#define RESULT_COUNT   32u

volatile int emu_app_running = 1;

typedef struct {
    uint8_t frame[1024];
    size_t len;
    unsigned count;
    bool overflow;
} emac_capture_t;

static int capture_emac(void *opaque, const uint8_t *frame, size_t len) {
    emac_capture_t *capture = opaque;
    capture->count++;
    capture->len = len;
    if (len > sizeof(capture->frame)) {
        capture->overflow = true;
        return -1;
    }
    memcpy(capture->frame, frame, len);
    return 0;
}

static uint32_t checksum(const uint8_t *data, size_t len) {
    uint32_t value = 0u;
    for (size_t index = 0; index < len; index++)
        value = (value * 33u) ^ data[index];
    return value;
}

static void run_batch(flexe_session_t *session, int cycles) {
    (void)flexe_session_run_core(session, 0, cycles);
    flexe_session_post_batch(session, cycles);
}

static void build_rx_frames(uint8_t frame1[700], uint8_t frame2[100]) {
    static const uint8_t destination[6] = {0x02, 0x11, 0x22,
                                           0x33, 0x44, 0x55};
    static const uint8_t source[6] = {0x02, 0xAA, 0xBB,
                                      0xCC, 0xDD, 0xEE};
    for (size_t index = 0; index < 700u; index++)
        frame1[index] = (uint8_t)(index * 5u + 1u);
    memcpy(frame1, destination, sizeof(destination));
    memcpy(frame1 + 6u, source, sizeof(source));
    frame1[12] = 0x08;
    frame1[13] = 0x00;

    for (size_t index = 0; index < 100u; index++)
        frame2[index] = (uint8_t)(index * 9u + 7u);
    const uint8_t multicast[6] = {0x01, 0x00, 0x5E, 0x12, 0x34, 0x56};
    memcpy(frame2, multicast, sizeof(multicast));
    memcpy(frame2 + 6u, source, sizeof(source));
    frame2[12] = 0x08;
    frame2[13] = 0x06;
}

int main(int argc, char **argv) {
    int argi = 1;
    int disable_jit = 0;
    int public_driver = 0;
    while (argi < argc && argv[argi][0] == '-') {
        if (strcmp(argv[argi], "--no-jit") == 0)
            disable_jit = 1;
        else if (strcmp(argv[argi], "--driver") == 0)
            public_driver = 1;
        else
            break;
        argi++;
    }
    if (argc - argi != 2) {
        fprintf(stderr, "usage: %s [--no-jit] [--driver] "
                        "FIRMWARE.bin FIRMWARE.elf\n",
                argv[0]);
        return 2;
    }

    elf_symbols_t *symbols = elf_symbols_load(argv[argi + 1]);
    uint32_t stage_addr = 0u;
    uint32_t command_addr = 0u;
    uint32_t result_addr = 0u;
    if (!symbols ||
        elf_symbols_find(symbols, "flexe_emac_stage", &stage_addr) != 0 ||
        elf_symbols_find(symbols, "flexe_emac_command", &command_addr) != 0 ||
        elf_symbols_find(symbols, "flexe_emac_result", &result_addr) != 0) {
        fprintf(stderr, "error: fixture marker symbols are missing\n");
        elf_symbols_destroy(symbols);
        return 2;
    }

    flexe_session_config_t config = {
        .bin_path = argv[argi],
        .elf_path = argv[argi + 1],
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
    emac_capture_t capture = {0};
    if (periph_set_emac_tx_callback(periph, capture_emac, &capture) != 0 ||
        periph_emac_phy_set_reg(periph, 1u, 2u, 0x2000u) != 0) {
        fprintf(stderr, "error: could not attach virtual Ethernet peer\n");
        flexe_session_destroy(session);
        elf_symbols_destroy(symbols);
        return 2;
    }

    uint8_t rx1[700];
    uint8_t rx2[100];
    build_rx_frames(rx1, rx2);
    int inject1 = 0;
    int inject2 = 0;
    bool injected = false;
    uint32_t last_stage = UINT32_MAX;
    uint32_t stage = 0u;
    while (cpu->cycle_count < MAX_CYCLES) {
        stage = mem_read32(mem, stage_addr);
        if (stage != last_stage) {
            fprintf(stderr,
                    "[emac-fixture] stage=0x%08X cycles=%llu pc=0x%08X tx=%u\n",
                    stage, (unsigned long long)cpu->cycle_count, cpu->pc,
                    capture.count);
            last_stage = stage;
        }
        if (stage == SUCCESS_MARKER ||
            (stage & 0xFFF00000u) == 0xBAD00000u)
            break;
        if (stage == 2u && capture.count == 1u && !injected) {
            inject1 = periph_emac_rx_inject(periph, rx1, sizeof(rx1));
            inject2 = periph_emac_rx_inject(periph, rx2, sizeof(rx2));
            injected = true;
            mem_write32(mem, command_addr, 1u);
        }
        run_batch(session, 10000);
    }

    stage = mem_read32(mem, stage_addr);
    uint32_t results[RESULT_COUNT];
    for (unsigned index = 0; index < RESULT_COUNT; index++)
        results[index] = mem_read32(mem, result_addr + index * 4u);
    uint16_t phy_adv = 0u;
    int phy_result = periph_emac_phy_get_reg(periph, 1u, 4u, &phy_adv);
    int unhandled = periph_unhandled_count(periph);
    int unregistered = rom_stubs_unregistered_count(flexe_session_rom(session));

    bool tx_ok = capture.count == 1u && !capture.overflow &&
                 capture.len == 700u;
    for (size_t index = 0; tx_ok && index < capture.len; index++)
        if (capture.frame[index] != (uint8_t)(index * 7u + 3u))
            tx_ok = false;
    uint32_t tx_checksum = capture.len <= sizeof(capture.frame) ?
                           checksum(capture.frame, capture.len) : 0u;
    bool results_ok;
    if (public_driver) {
        results_ok =
            results[0] == 0u && results[1] == 0u &&
            results[2] == 0x2000u && results[3] == 700u &&
            results[5] == checksum(rx1, sizeof(rx1)) &&
            results[6] == 100u && results[8] == checksum(rx2, sizeof(rx2)) &&
            results[9] >= 2u && (results[10] & 3u) == 3u &&
            results[11] == tx_checksum &&
            results[12] == 0u && results[13] == 0u &&
            results[14] == 0xA802u && results[15] == 0x8201u &&
            results[16] == 0u && results[17] == 0u && results[18] == 0u &&
            results[31] == 0u;
    } else {
        results_ok =
            results[0] == 0u && results[1] == 700u &&
            results[2] == 0x2000u && results[3] == 700u &&
            results[5] == checksum(rx1, sizeof(rx1)) &&
            results[6] == 100u && results[8] == checksum(rx2, sizeof(rx2)) &&
            results[9] >= 2u && (results[10] & (1u << 6)) != 0u &&
            results[11] == tx_checksum &&
            results[12] == 0u && results[31] == 0u;
    }

    if (stage != SUCCESS_MARKER || !tx_ok || !results_ok) {
        uint32_t rx_base = mem_read32(mem, 0x3FF6900Cu);
        uint32_t tx_base = mem_read32(mem, 0x3FF69010u);
        fprintf(stderr, "[emac-fixture] results=");
        for (unsigned index = 0; index < RESULT_COUNT; index++)
            fprintf(stderr, "%s%08X", index ? "/" : "", results[index]);
        fprintf(stderr,
                "\n[emac-fixture] dma=%08X/%08X/%08X/%08X/%08X/%08X "
                "mac=%08X/%08X dport=%08X/%08X "
                "desc=%08X/%08X/%08X/%08X\n",
                mem_read32(mem, 0x3FF69000u),
                mem_read32(mem, 0x3FF6900Cu),
                mem_read32(mem, 0x3FF69010u),
                mem_read32(mem, 0x3FF69014u),
                mem_read32(mem, 0x3FF69018u),
                mem_read32(mem, 0x3FF6901Cu),
                mem_read32(mem, 0x3FF6A000u),
                mem_read32(mem, 0x3FF6A004u),
                mem_read32(mem, 0x3FF000CCu),
                mem_read32(mem, 0x3FF000D0u),
                rx_base ? mem_read32(mem, rx_base) : 0u,
                rx_base ? mem_read32(mem, rx_base + 32u) : 0u,
                tx_base ? mem_read32(mem, tx_base) : 0u,
                tx_base ? mem_read32(mem, tx_base + 32u) : 0u);
        char task_dump[4096];
        if (freertos_stubs_dump_tasks(flexe_session_frt(session), task_dump,
                                      sizeof(task_dump)) > 0)
            fprintf(stderr, "[emac-fixture] tasks:\n%s", task_dump);
        for (int index = 0;; index++) {
            const char *name = NULL;
            uint32_t address = 0u;
            uint32_t calls = 0u;
            if (rom_stubs_get_stats(flexe_session_rom(session), index,
                                    &name, &address, &calls) != 0)
                break;
            if (calls != 0u && name &&
                (strstr(name, "Notify") || strstr(name, "Task")))
                fprintf(stderr, "[emac-fixture] stub %s@%08X calls=%u\n",
                        name, address, calls);
        }
    }

    printf("profile=%s engine=%s stage=0x%08X tx=%u/%zu tx_ok=%d "
           "inject=%d/%d irq=%u status=%08X phy=%d/%04X "
           "unhandled=%d unregistered=%d cycles=%llu\n",
           public_driver ? "driver" : "hal",
           flexe_session_jit(session) ? "jit" : "interp", stage,
           capture.count, capture.len, tx_ok, inject1, inject2,
           results[9], results[10], phy_result, phy_adv,
           unhandled, unregistered,
           (unsigned long long)cpu->cycle_count);

    int ok = stage == SUCCESS_MARKER && injected && inject1 == 1 &&
             inject2 == 1 && tx_ok && results_ok &&
             phy_result == 0 && phy_adv == 0x01E1u &&
             unhandled == 0 && unregistered == 0;
    flexe_session_destroy(session);
    elf_symbols_destroy(symbols);
    return ok ? 0 : 1;
}
