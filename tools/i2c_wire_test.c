/* Run the compiled Arduino Wire fixture through the public Flexe session API.
 * The firmware itself is built externally so the normal emulator build does
 * not depend on Arduino CLI or a particular ESP32 core release. */
#include "elf_symbols.h"
#include "flexe_session.h"
#include "memory.h"
#include "peripherals.h"

#include <stdint.h>
#include <stdio.h>

#define SUCCESS_MARKER 0x1C2C0040u
#define MAX_CYCLES     30000000ull

volatile int emu_app_running = 1;

typedef struct {
    uint8_t regs[256];
    uint8_t cursor;
    unsigned calls;
    size_t write_bytes;
    size_t read_bytes;
} register_device_t;

static int register_device(void *opaque, int port, uint8_t address,
                           const uint8_t *write_data, size_t write_len,
                           uint8_t *read_data, size_t read_len) {
    register_device_t *device = opaque;
    if (port != 0 || address != 0x34)
        return -1;
    device->calls++;
    device->write_bytes += write_len;
    device->read_bytes += read_len;
    if (write_len != 0) {
        device->cursor = write_data[0];
        for (size_t i = 1; i < write_len; ++i)
            device->regs[device->cursor++] = write_data[i];
    }
    for (size_t i = 0; i < read_len; ++i)
        read_data[i] = device->regs[device->cursor++];
    return 0;
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
        elf_symbols_find(symbols, "flexe_i2c_stage", &stage_addr) != 0 ||
        elf_symbols_find(symbols, "flexe_i2c_result", &result_addr) != 0) {
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

    register_device_t device = {0};
    esp32_periph_t *periph = flexe_session_periph(session);
    if (periph_i2c_attach_device(periph, 0, 0x34,
                                 register_device, &device) != 0) {
        fprintf(stderr, "error: could not attach virtual I2C target\n");
        flexe_session_destroy(session);
        elf_symbols_destroy(symbols);
        return 2;
    }

    xtensa_cpu_t *cpu = flexe_session_cpu(session, 0);
    xtensa_mem_t *mem = flexe_session_mem(session);
    uint32_t stage = 0;
    while (cpu->cycle_count < MAX_CYCLES) {
        stage = mem_read32(mem, stage_addr);
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

    uint32_t expected_checksum = 0;
    int memory_ok = 1;
    for (uint8_t i = 0; i < 40; ++i) {
        uint8_t expected = (uint8_t)(0x5Au ^ i);
        expected_checksum = expected_checksum * 33u + expected;
        if (device.regs[0x20u + i] != expected)
            memory_ok = 0;
    }
    int unhandled = periph_unhandled_count(periph);
    printf("stage=0x%08X result=%u/%u/%u/0x%08X calls=%u "
           "write_bytes=%zu read_bytes=%zu memory_ok=%d unhandled=%d "
           "cycles=%llu\n",
           stage, results[0], results[1], results[2], results[3],
           device.calls, device.write_bytes, device.read_bytes, memory_ok,
           unhandled, (unsigned long long)cpu->cycle_count);

    int ok = stage == SUCCESS_MARKER && results[0] == 0 &&
             results[1] == 0 && results[2] == 40 &&
             results[3] == expected_checksum && memory_ok &&
             device.write_bytes >= 42 && device.read_bytes == 40 &&
             unhandled == 0;
    flexe_session_destroy(session);
    elf_symbols_destroy(symbols);
    return ok ? 0 : 1;
}
