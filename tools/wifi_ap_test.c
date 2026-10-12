/* Soft-AP bring-up with one virtual station, driven from the host.
 *
 * The harness provisions a client MAC before the guest runs, so the AP
 * auto-joins it at start; once the fixture reports the join, the harness
 * drops the station again. Both directions arrive as real WIFI_EVENTs,
 * which is what AP-mode firmware (status displays, captive portals)
 * builds on.
 */
#include "elf_symbols.h"
#include "flexe_session.h"
#include "memory.h"
#include "peripherals.h"
#include "rom_stubs.h"
#include "wifi_stubs.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define SUCCESS_MARKER 0x9F1C0C0Bu
#define MAX_CYCLES     8000000000ull
#define RESULT_COUNT   8u

#define TEST_MAC_0 0xAAu
#define TEST_MAC_1 0xBBu
#define TEST_MAC_2 0xCCu
#define TEST_MAC_3 0xDDu
#define TEST_MAC_4 0xEEu
#define TEST_MAC_5 0x01u

volatile int emu_app_running = 1;

int main(int argc, char **argv) {
    int argi = 1;
    int disable_jit = 0;
    if (argi < argc && strcmp(argv[argi], "--no-jit") == 0) {
        disable_jit = 1;
        argi++;
    }
    if (argc - argi != 2) {
        fprintf(stderr, "usage: %s [--no-jit] FIRMWARE.bin FIRMWARE.elf\n",
                argv[0]);
        return 2;
    }

    elf_symbols_t *symbols = elf_symbols_load(argv[argi + 1]);
    uint32_t stage_addr = 0, result_addr = 0;
    if (!symbols ||
        elf_symbols_find(symbols, "flexe_ap_stage", &stage_addr) != 0 ||
        elf_symbols_find(symbols, "flexe_ap_result", &result_addr) != 0) {
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
    static const uint8_t phone[6] = {
        TEST_MAC_0, TEST_MAC_1, TEST_MAC_2,
        TEST_MAC_3, TEST_MAC_4, TEST_MAC_5,
    };
    if (wifi_stubs_ap_provision_client(flexe_session_wifi(session),
                                       phone) != 0) {
        fprintf(stderr, "error: cannot provision AP client\n");
        flexe_session_destroy(session);
        elf_symbols_destroy(symbols);
        return 2;
    }

    xtensa_cpu_t *cpu = flexe_session_cpu(session, 0);
    xtensa_mem_t *mem = flexe_session_mem(session);
    uint32_t stage = 0, last_stage = UINT32_MAX;
    int dropped = 0;
    while (cpu->cycle_count < MAX_CYCLES) {
        stage = mem_read32(mem, stage_addr);
        if (stage != last_stage) {
            fprintf(stderr, "[wifi-ap] stage=0x%08X cycles=%llu\n",
                    stage, (unsigned long long)cpu->cycle_count);
            last_stage = stage;
        }
        /* Once the join lands, drop the station: the fixture must see
         * the leave event with the same AID. */
        if (!dropped && stage == 3) {
            if (wifi_stubs_ap_client_leave(flexe_session_wifi(session),
                                           phone) != 0)
                fprintf(stderr, "[wifi-ap] warning: leave failed\n");
            dropped = 1;
        }
        if (stage == SUCCESS_MARKER || (stage & 0xFFF00000u) == 0xBAD00000u)
            break;
        (void)flexe_session_run_core(session, 0, 10000);
        flexe_session_post_batch(session, 10000);
    }

    stage = mem_read32(mem, stage_addr);
    uint32_t r[RESULT_COUNT];
    for (unsigned i = 0; i < RESULT_COUNT; i++)
        r[i] = mem_read32(mem, result_addr + i * 4u);
    if (stage != SUCCESS_MARKER) {
        char dump[4096];
        if (freertos_stubs_dump_tasks(flexe_session_frt(session), dump,
                                      sizeof dump) > 0)
            fprintf(stderr, "[wifi-ap] tasks:\n%s", dump);
        fprintf(stderr, "[wifi-ap] pc=0x%08X cycles=%llu\n", cpu->pc,
                (unsigned long long)cpu->cycle_count);
    }

    int unhandled = periph_unhandled_count(flexe_session_periph(session));
    int unregistered = rom_stubs_unregistered_count(flexe_session_rom(session));

    printf("engine=%s stage=0x%08X start=%u join=%u mac=%08X/%04X aid=%u "
           "leave=%u leave_aid=%u unhandled=%d unregistered=%d\n",
           flexe_session_jit(session) ? "jit" : "interp", stage,
           r[0], r[1], r[2], r[3], r[4], r[5], r[6],
           unhandled, unregistered);

    uint32_t want_hi = ((uint32_t)TEST_MAC_0 << 24) |
                       ((uint32_t)TEST_MAC_1 << 16) |
                       ((uint32_t)TEST_MAC_2 << 8) | TEST_MAC_3;
    uint32_t want_lo = ((uint32_t)TEST_MAC_4 << 8) | TEST_MAC_5;
    bool start_ok = r[0] >= 1u;
    bool join_ok = r[1] >= 1u && r[2] == want_hi && r[3] == want_lo &&
                   r[4] == 1u;
    bool leave_ok = r[5] >= 1u && r[6] == 1u;
    int ok = stage == SUCCESS_MARKER && start_ok && join_ok && leave_ok &&
             unhandled == 0 && unregistered == 0;
    if (!ok)
        fprintf(stderr, "[wifi-ap] start=%d join=%d leave=%d "
                        "(want mac %08X/%04X aid 1)\n",
                start_ok, join_ok, leave_ok, want_hi, want_lo);

    flexe_session_destroy(session);
    elf_symbols_destroy(symbols);
    return ok ? 0 : 1;
}
