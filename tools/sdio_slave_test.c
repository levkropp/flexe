/* Run Espressif's public SDIO-slave driver through a compiled Xtensa fixture.
 * The guest owns HINF/HOST/SLC configuration, descriptors, and source-10 ISR;
 * this host side models the external SDIO controller. */
#include "elf_symbols.h"
#include "flexe_session.h"
#include "memory.h"
#include "peripherals.h"
#include "rom_stubs.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define SUCCESS_MARKER 0x5344494Fu
#define MAX_CYCLES     120000000ull
#define RESULT_COUNT   24u
#define SLAVE_PACKET_LENGTH 21u
#define HOST_PACKET_LENGTH  24u

volatile int emu_app_running = 1;

static uint32_t checksum(const uint8_t *data, size_t length) {
    uint32_t value = 0u;
    for (size_t index = 0; index < length; index++)
        value = (value * 33u) ^ data[index];
    return value;
}

static void run_batch(flexe_session_t *session, int cycles) {
    (void)flexe_session_run_core(session, 0, cycles);
    flexe_session_post_batch(session, cycles);
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

    elf_symbols_t *symbols = elf_symbols_load(argv[argi + 1]);
    uint32_t stage_addr = 0u;
    uint32_t command_addr = 0u;
    uint32_t result_addr = 0u;
    if (!symbols ||
        elf_symbols_find(symbols, "flexe_sdio_stage", &stage_addr) != 0 ||
        elf_symbols_find(symbols, "flexe_sdio_command", &command_addr) != 0 ||
        elf_symbols_find(symbols, "flexe_sdio_result", &result_addr) != 0) {
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
    uint8_t slave_packet[SLAVE_PACKET_LENGTH] = {0};
    uint8_t host_packet[HOST_PACKET_LENGTH];
    for (size_t index = 0; index < sizeof(host_packet); index++)
        host_packet[index] = (uint8_t)(0xC0u ^ index);

    bool host_acted = false;
    bool ready = false;
    uint16_t send_buffers = 0u;
    uint32_t receive_bytes = 0u;
    uint8_t shared_before = 0u;
    uint32_t host_raw = 0u;
    uint32_t host_pending = 0u;
    int shared_read_result = -1;
    int slave_read_result = -1;
    size_t slave_read_length = 0u;
    int host_write_result = -1;
    int shared_write_result = -1;
    int interrupt_result = -1;
    uint32_t stage = 0u;
    uint32_t last_stage = UINT32_MAX;

    while (cpu->cycle_count < MAX_CYCLES) {
        stage = mem_read32(mem, stage_addr);
        if (stage != last_stage) {
            fprintf(stderr,
                    "[sdio-fixture] engine=%s stage=0x%08X cycles=%llu "
                    "pc=0x%08X\n",
                    flexe_session_jit(session) ? "jit" : "interp", stage,
                    (unsigned long long)cpu->cycle_count, cpu->pc);
            last_stage = stage;
        }
        if (stage == SUCCESS_MARKER ||
            (stage & 0xFFF00000u) == 0xBAD00000u)
            break;

        if (stage == 2u && !host_acted) {
            ready = periph_sdio_slave_host_ready(periph);
            send_buffers = periph_sdio_slave_host_send_buffers(periph);
            receive_bytes =
                periph_sdio_slave_host_receive_bytes(periph);
            if (ready && send_buffers >= 2u &&
                receive_bytes == SLAVE_PACKET_LENGTH) {
                shared_read_result = periph_sdio_slave_host_read_reg(
                    periph, 5u, &shared_before);
                host_raw = periph_sdio_slave_host_interrupt_raw(periph);
                host_pending =
                    periph_sdio_slave_host_interrupt_pending(periph);
                slave_read_result = periph_sdio_slave_host_read_packet(
                    periph, slave_packet, sizeof(slave_packet),
                    &slave_read_length);
                host_write_result = periph_sdio_slave_host_write_packet(
                    periph, host_packet, sizeof(host_packet));
                shared_write_result = periph_sdio_slave_host_write_reg(
                    periph, 5u, 0x5Au);
                interrupt_result =
                    periph_sdio_slave_host_interrupt(periph, 1u << 3);
                periph_sdio_slave_host_interrupt_clear(
                    periph, (1u << 2) | (1u << 23));
                host_acted = true;
                mem_write32(mem, command_addr, 1u);
            }
        }
        run_batch(session, 10000);
    }

    stage = mem_read32(mem, stage_addr);
    uint32_t result[RESULT_COUNT];
    for (unsigned index = 0; index < RESULT_COUNT; index++)
        result[index] = mem_read32(mem, result_addr + index * 4u);
    int unhandled = periph_unhandled_count(periph);
    int unregistered =
        rom_stubs_unregistered_count(flexe_session_rom(session));

    bool slave_data_ok = slave_read_length == sizeof(slave_packet);
    for (size_t index = 0; slave_data_ok && index < sizeof(slave_packet);
         index++) {
        if (slave_packet[index] != (uint8_t)(0x40u + index * 3u))
            slave_data_ok = false;
    }
    uint32_t expected_host_checksum0 = checksum(host_packet, 16u);
    uint32_t expected_host_checksum1 = checksum(host_packet + 16u, 8u);
    uint32_t expected_slave_checksum = checksum(slave_packet,
                                                 sizeof(slave_packet));
    bool result_ok =
        result[0] == 0u && result[1] == 3u &&
        result[2] == 0u && result[3] == 0u && result[4] == 0u &&
        result[5] == 0u &&
        result[6] == ((1u << 2) | (1u << 23)) &&
        result[7] == 0u && result[8] == 0u && result[9] == 1u &&
        (result[10] & (1u << 3)) != 0u && result[11] >= 1u &&
        result[12] != 0u && result[13] == 16u &&
        result[14] == expected_host_checksum0 &&
        result[15] == 0u && result[16] == 8u &&
        result[17] == expected_host_checksum1 &&
        result[18] == 0u && result[19] == 0x13579BDFu &&
        result[20] == 0x5Au && result[21] == 1u &&
        result[22] == expected_slave_checksum && result[23] == 0u;

    if (stage != SUCCESS_MARKER || !host_acted || !slave_data_ok ||
        !result_ok || unhandled != 0 || unregistered != 0) {
        fprintf(stderr, "[sdio-fixture] results=");
        for (unsigned index = 0; index < RESULT_COUNT; index++)
            fprintf(stderr, "%s%08X", index ? "/" : "", result[index]);
        fprintf(stderr,
                "\n[sdio-fixture] slc=%08X/%08X/%08X/%08X/%08X "
                "host=%08X/%08X/%08X hinf=%08X dport=%08X/%08X\n",
                mem_read32(mem, 0x3FF58004u),
                mem_read32(mem, 0x3FF58008u),
                mem_read32(mem, 0x3FF5800Cu),
                mem_read32(mem, 0x3FF5803Cu),
                mem_read32(mem, 0x3FF58040u),
                mem_read32(mem, 0x3FF55050u),
                mem_read32(mem, 0x3FF55060u),
                mem_read32(mem, 0x3FF550DCu),
                mem_read32(mem, 0x3FF4B004u),
                mem_read32(mem, 0x3FF000CCu),
                mem_read32(mem, 0x3FF000D0u));
    }

    printf("engine=%s stage=0x%08X ready=%d buffers=%u bytes=%u "
           "shared=%d/%02X/%d host_irq=%08X/%08X "
           "transfer=%d/%zu/%d/%d irq=%d data=%d "
           "unhandled=%d unregistered=%d cycles=%llu\n",
           flexe_session_jit(session) ? "jit" : "interp", stage,
           ready, send_buffers, receive_bytes,
           shared_read_result, shared_before, shared_write_result,
           host_raw, host_pending, slave_read_result, slave_read_length,
           host_write_result, interrupt_result, result[9], slave_data_ok,
           unhandled, unregistered,
           (unsigned long long)cpu->cycle_count);

    int ok = stage == SUCCESS_MARKER && host_acted && ready &&
             send_buffers >= 2u &&
             receive_bytes == SLAVE_PACKET_LENGTH &&
             shared_read_result == 0 && shared_before == 0xA5u &&
             (host_raw & ((1u << 2) | (1u << 23))) ==
                 ((1u << 2) | (1u << 23)) &&
             (host_pending & ((1u << 2) | (1u << 23))) ==
                 ((1u << 2) | (1u << 23)) &&
             slave_read_result == 1 && slave_data_ok &&
             host_write_result == 1 && shared_write_result == 0 &&
             interrupt_result == 0 && result_ok &&
             unhandled == 0 && unregistered == 0;

    flexe_session_destroy(session);
    elf_symbols_destroy(symbols);
    return ok ? 0 : 1;
}
