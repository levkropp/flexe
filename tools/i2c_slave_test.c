/* Drive an I2C transfer at the guest, which is the slave.
 *
 * The host is the bus master. It writes four bytes the guest must receive and
 * reads three the guest staged. Both halves are needed: a model that loops the
 * guest's own TX FIFO back into its RX FIFO passes every guest-side check, and
 * only this side can tell whether the bytes read off the bus are the ones the
 * firmware actually staged.
 */
#include "elf_symbols.h"
#include "flexe_session.h"
#include "memory.h"
#include "peripherals.h"
#include "rom_stubs.h"
#include "target.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define SUCCESS_MARKER 0x12C51AEEu
#define MAX_CYCLES_CLASSIC 4000000000ull
#define MAX_CYCLES_S3      8000000000ull
#define RESULT_COUNT   8u

#define PORT       0
#define SLAVE_ADDR 0x42

volatile int emu_app_running = 1;

int main(int argc, char **argv) {
    int argi = 1;
    int disable_jit = 0;
    bool s3 = false;
    while (argi < argc) {
        if (strcmp(argv[argi], "--no-jit") == 0)
            disable_jit = 1;
        else if (strcmp(argv[argi], "--s3") == 0)
            s3 = true;
        else
            break;
        argi++;
    }
    if (argc - argi != (s3 ? 3 : 2)) {
        fprintf(stderr, "usage: %s [--no-jit] [--s3] FIRMWARE.bin "
                "FIRMWARE.elf [ROM.elf]\n", argv[0]);
        return 2;
    }

    elf_symbols_t *symbols = elf_symbols_load(argv[argi + 1]);
    uint32_t stage_addr = 0, result_addr = 0;
    if (!symbols ||
        elf_symbols_find(symbols, "flexe_i2cslave_stage", &stage_addr) != 0 ||
        elf_symbols_find(symbols, "flexe_i2cslave_result", &result_addr) != 0) {
        fprintf(stderr, "error: fixture marker symbols are missing\n");
        elf_symbols_destroy(symbols);
        return 2;
    }

    flexe_session_config_t config = {
        .bin_path = argv[argi],
        .elf_path = argv[argi + 1],
        .rom_elf_path = s3 ? argv[argi + 2] : NULL,
        .disable_jit = disable_jit,
        .native_freertos = s3,
        .target = s3 ? FLEXE_TARGET_ESP32S3 : FLEXE_TARGET_AUTO,
        .unhandled_audit = s3,
    };
    flexe_session_t *session = flexe_session_create(&config);
    if (!session) {
        elf_symbols_destroy(symbols);
        return 2;
    }

    xtensa_cpu_t *cpu = flexe_session_cpu(session, 0);
    xtensa_mem_t *mem = flexe_session_mem(session);
    esp32_periph_t *periph = flexe_session_periph(session);

    static const uint8_t SEND[4] = { 0xDE, 0xAD, 0xBE, 0xEF };
    static const uint8_t WANT_BACK[3] = { 0x11, 0x22, 0x33 };
    uint8_t got_back[3] = {0};
    int accepted = -1;
    bool xfer_done = false;
    uint32_t stage = 0, last_stage = UINT32_MAX;
    bool budget_stop = true;

    uint64_t max_cycles = s3 ? MAX_CYCLES_S3 : MAX_CYCLES_CLASSIC;
    while (cpu->cycle_count < max_cycles) {
        stage = mem_read32(mem, stage_addr);
        if (stage != last_stage) {
            fprintf(stderr, "[i2c-slave-fixture] stage=0x%08X cycles=%llu\n",
                    stage, (unsigned long long)cpu->cycle_count);
            last_stage = stage;
            if (stage == 1 && !xfer_done) {
                accepted = periph_i2c_master_xfer(periph, PORT, SLAVE_ADDR,
                                                  SEND, sizeof SEND,
                                                  got_back, sizeof got_back);
                xfer_done = true;
            }
        }
        if (stage == SUCCESS_MARKER || (stage & 0xFFF00000u) == 0xBAD00000u) {
            budget_stop = false;
            break;
        }
        (void)flexe_session_run_core(session, 0, 10000);
        flexe_session_post_batch(session, 10000);
    }

    stage = mem_read32(mem, stage_addr);
    uint32_t r[RESULT_COUNT];
    for (unsigned i = 0; i < RESULT_COUNT; i++)
        r[i] = mem_read32(mem, result_addr + i * 4u);

    if (budget_stop)
        fprintf(stderr, "[i2c-slave-fixture] exited on cycle budget (%llu) at "
                        "pc=0x%08X\n", (unsigned long long)max_cycles,
                        cpu->pc);

    int unhandled = periph_unhandled_count(flexe_session_periph(session));
    unsigned i2c_unhandled_sites = 0u;
    if (s3) {
        const flexe_target_desc_t *target = flexe_target_by_id(config.target);
        for (size_t i = 0; i < periph_unhandled_audit_count(periph); i++) {
            periph_unhandled_site_t site;
            if (!periph_unhandled_audit_get(periph, i, &site)) continue;
            for (unsigned port = 0; port < target->i2c.instance_count; port++) {
                uint32_t base = target->i2c.instance[port].base;
                if (site.address >= base &&
                    site.address - base < target->i2c.register_size) {
                    i2c_unhandled_sites++;
                    fprintf(stderr, "[s3-i2c-slave] unsupported port=%u %c "
                            "0x%08X pc=0x%08X count=%llu\n",
                            port, site.write ? 'W' : 'R', site.address,
                            site.pc, (unsigned long long)site.count);
                    break;
                }
            }
        }
    }
    int unregistered = rom_stubs_unregistered_count(flexe_session_rom(session));

    uint32_t packed = (uint32_t)SEND[0] | ((uint32_t)SEND[1] << 8) |
                      ((uint32_t)SEND[2] << 16) | ((uint32_t)SEND[3] << 24);

    printf("engine=%s stage=0x%08X staged=%u guest_got=%u guest_bytes=%08X "
           "accepted=%d read_back=%02X%02X%02X unhandled=%d "
           "i2c_unhandled_sites=%u unregistered=%d\n",
           flexe_session_jit(session) ? "jit" : "interp", stage,
           r[0], r[1], r[2], accepted,
           got_back[0], got_back[1], got_back[2], unhandled,
           i2c_unhandled_sites, unregistered);

    /* The slave took every byte the master sent, and reported the same ones. */
    bool rx_ok = accepted == (int)sizeof SEND && r[1] == sizeof SEND &&
                 r[2] == packed;
    /* And the master read back exactly what the guest staged -- not 0xFF
     * padding, and not the bytes it had just been sent. */
    bool tx_ok = memcmp(got_back, WANT_BACK, sizeof WANT_BACK) == 0;

    int ok = stage == SUCCESS_MARKER && rx_ok && tx_ok &&
             (s3 ? i2c_unhandled_sites == 0u : unhandled == 0) &&
             unregistered == 0;
    if (!ok)
        fprintf(stderr, "[i2c-slave-fixture] stage_ok=%d rx_ok=%d tx_ok=%d\n",
                stage == SUCCESS_MARKER, rx_ok, tx_ok);

    flexe_session_destroy(session);
    elf_symbols_destroy(symbols);
    return ok ? 0 : 1;
}
