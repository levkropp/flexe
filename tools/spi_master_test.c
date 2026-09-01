/* Run the public ESP-IDF spi_master driver through Flexe's GP-SPI model,
 * with the host standing in as the slave.
 *
 * The CYD display libraries drive the SPI registers directly, so a stock ROM
 * never touches the IDF driver: its queued transactions, DMA descriptors and
 * command/address phases are otherwise untested. The chip select the fixture
 * uses has no device model behind it, so periph_spi_attach_probe() lets this
 * harness see every MOSI byte and choose every MISO byte.
 */
#include "elf_symbols.h"
#include "flexe_session.h"
#include "memory.h"
#include "peripherals.h"
#include "rom_stubs.h"
#include "spi_display.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define SUCCESS_MARKER 0x5D100D1Eu
#define MAX_CYCLES     400000000ull
#define RESULT_COUNT   12u

volatile int emu_app_running = 1;

typedef struct {
    unsigned transfers;
    size_t   total_mosi;
    uint8_t  first_mosi[8];
    size_t   first_len;
    bool     saw_expected_payload;
} probe_state_t;

/* Reply with a byte derived from the transfer's own length and the byte
 * index, matching the fixture: a short read, a duplicated byte or a shifted
 * buffer all show up instead of looking plausible. */
static uint8_t expect_miso(size_t len, size_t i) {
    return (uint8_t)((len * 0x11u) ^ (i * 0x2Fu) ^ 0x5Au);
}

static void spi_probe(const uint8_t *mosi, size_t mosi_len,
                      uint8_t *miso, size_t miso_len, void *ctx) {
    probe_state_t *st = ctx;
    st->transfers++;
    st->total_mosi += mosi_len;
    if (st->first_len == 0 && mosi_len > 0) {
        st->first_len = mosi_len < sizeof(st->first_mosi) ? mosi_len
                                                          : sizeof(st->first_mosi);
        memcpy(st->first_mosi, mosi, st->first_len);
    }
    /* The fixture's full-duplex transfers send 0xA0, 0xA1, ... */
    if (mosi_len >= 4 && mosi[0] == 0xA0u && mosi[1] == 0xA1u)
        st->saw_expected_payload = true;

    size_t len = miso_len ? miso_len : mosi_len;
    for (size_t i = 0; i < miso_len; i++)
        miso[i] = expect_miso(len, i);
}

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
        elf_symbols_find(symbols, "flexe_spi_stage", &stage_addr) != 0 ||
        elf_symbols_find(symbols, "flexe_spi_result", &result_addr) != 0) {
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

    probe_state_t probe = {0};
    periph_spi_attach_probe(flexe_session_periph(session), spi_probe, &probe);

    xtensa_cpu_t *cpu = flexe_session_cpu(session, 0);
    xtensa_mem_t *mem = flexe_session_mem(session);
    uint32_t stage = 0, last_stage = UINT32_MAX;
    while (cpu->cycle_count < MAX_CYCLES) {
        stage = mem_read32(mem, stage_addr);
        if (stage != last_stage) {
            fprintf(stderr, "[spi-fixture] stage=0x%08X cycles=%llu\n",
                    stage, (unsigned long long)cpu->cycle_count);
            last_stage = stage;
        }
        if (stage == SUCCESS_MARKER || (stage & 0xFFF00000u) == 0xBAD00000u)
            break;
        (void)flexe_session_run_core(session, 0, 10000);
        flexe_session_post_batch(session, 10000);
    }

    stage = mem_read32(mem, stage_addr);
    uint32_t results[RESULT_COUNT];
    for (unsigned i = 0; i < RESULT_COUNT; i++)
        results[i] = mem_read32(mem, result_addr + i * 4u);
    if (stage != SUCCESS_MARKER) {
        char dump[4096];
        if (freertos_stubs_dump_tasks(flexe_session_frt(session), dump,
                                      sizeof dump) > 0)
            fprintf(stderr, "[spi-fixture] tasks:\n%s", dump);
        fprintf(stderr, "[spi-fixture] pc=0x%08X\n", cpu->pc);
    }
    int unhandled = periph_unhandled_count(flexe_session_periph(session));
    int unregistered = rom_stubs_unregistered_count(flexe_session_rom(session));

    printf("engine=%s stage=0x%08X transfers=%u mosi_bytes=%zu "
           "first=%02X%02X payload=%d lens=%u/%u/%u/%u/%u cmdaddr=0x%08X "
           "queued=%u unhandled=%d unregistered=%d\n",
           flexe_session_jit(session) ? "jit" : "interp", stage,
           probe.transfers, probe.total_mosi,
           probe.first_len > 0 ? probe.first_mosi[0] : 0,
           probe.first_len > 1 ? probe.first_mosi[1] : 0,
           (int)probe.saw_expected_payload,
           results[0], results[1], results[2], results[3], results[4],
           results[8], results[9], unhandled, unregistered);

    /* The guest checked every received byte itself; results[0..4] carry the
     * transfer lengths it accepted, so a silently truncated transfer fails
     * here as well as there. */
    bool lengths_ok = results[0] == 1u && results[1] == 4u &&
                      results[2] == 5u && results[3] == 17u &&
                      results[4] == 33u;
    int ok = stage == SUCCESS_MARKER && lengths_ok &&
             probe.transfers >= 7u && probe.saw_expected_payload &&
             results[9] == 1u && unhandled == 0 && unregistered == 0;

    flexe_session_destroy(session);
    elf_symbols_destroy(symbols);
    return ok ? 0 : 1;
}
