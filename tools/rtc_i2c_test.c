/* Run a compiled Arduino/ESP-IDF RTC-I2C fixture through Flexe's public
 * session API and validate the independent ULP-domain virtual bus. */
#include "elf_symbols.h"
#include "flexe_session.h"
#include "memory.h"
#include "peripherals.h"
#include "rom_stubs.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define SUCCESS_MARKER 0x12C0C0DEu
#define MAX_CYCLES     30000000ull

volatile int emu_app_running = 1;

typedef struct {
    uint8_t regs[256];
    uint8_t cursor;
    unsigned calls;
    size_t write_bytes;
    size_t read_bytes;
} rtc_register_device_t;

static int rtc_register_device(void *opaque, int port, uint8_t address,
                               const uint8_t *write_data, size_t write_len,
                               uint8_t *read_data, size_t read_len) {
    rtc_register_device_t *device = opaque;
    if (port != PERIPH_I2C_PORT_RTC || address != 0x34u)
        return -1;
    device->calls++;
    device->write_bytes += write_len;
    device->read_bytes += read_len;
    if (write_len != 0u) {
        device->cursor = write_data[0];
        for (size_t index = 1u; index < write_len; index++)
            device->regs[device->cursor++] = write_data[index];
    }
    for (size_t index = 0u; index < read_len; index++)
        read_data[index] = device->regs[device->cursor++];
    return 0;
}

static int run_fixture(const char *bin_path, const char *elf_path,
                       bool disable_jit) {
    elf_symbols_t *symbols = elf_symbols_load(elf_path);
    uint32_t stage_addr = 0u;
    uint32_t result_addr = 0u;
    if (!symbols ||
        elf_symbols_find(symbols, "flexe_rtc_i2c_stage", &stage_addr) != 0 ||
        elf_symbols_find(symbols, "flexe_rtc_i2c_result", &result_addr) != 0) {
        fprintf(stderr, "error: fixture marker symbols are missing\n");
        elf_symbols_destroy(symbols);
        return 2;
    }

    flexe_session_config_t config = {
        .bin_path = bin_path,
        .elf_path = elf_path,
        .disable_jit = disable_jit,
    };
    flexe_session_t *session = flexe_session_create(&config);
    if (!session) {
        elf_symbols_destroy(symbols);
        return 2;
    }

    rtc_register_device_t device = {0};
    esp32_periph_t *periph = flexe_session_periph(session);
    if (periph_i2c_attach_device(periph, PERIPH_I2C_PORT_RTC, 0x34u,
                                 rtc_register_device, &device) != 0) {
        fprintf(stderr, "error: could not attach virtual RTC-I2C target\n");
        flexe_session_destroy(session);
        elf_symbols_destroy(symbols);
        return 2;
    }

    xtensa_cpu_t *cpu = flexe_session_cpu(session, 0);
    xtensa_mem_t *mem = flexe_session_mem(session);
    uint32_t stage = 0u;
    while (cpu->cycle_count < MAX_CYCLES) {
        stage = mem_read32(mem, stage_addr);
        if (stage == SUCCESS_MARKER ||
            (stage & 0xFFF00000u) == 0xBAD00000u)
            break;
        (void)flexe_session_run_core(session, 0, 10000u);
        flexe_session_post_batch(session, 10000u);
    }

    stage = mem_read32(mem, stage_addr);
    uint32_t results[16];
    for (unsigned index = 0u; index < 16u; index++)
        results[index] = mem_read32(mem, result_addr + index * 4u);

    bool memory_ok = device.regs[0x20] == 0xA5u &&
                     device.regs[0x21] == 0x14u;
    int unhandled = periph_unhandled_count(periph);
    int unregistered = rom_stubs_unregistered_count(
        flexe_session_rom(session));
    const char *engine = disable_jit ? "interp" : "jit";
    printf("engine=%s stage=0x%08X timing=%u/%u/%u "
           "write=%u raw=%02X/%03X read=%02X/%02X masked=%02X "
           "command=%08X nack=%u/%02X clear=%u timeout=%02X/%03X/%u "
           "calls=%u bytes=%zu/%zu memory=%d unhandled=%d unregistered=%d "
           "cycles=%llu\n",
           engine, stage, results[0], results[1], results[2], results[3],
           results[4], results[5], results[6], results[7], results[8],
           results[9], results[10], results[11], results[12], results[13],
           results[14], results[15], device.calls, device.write_bytes,
           device.read_bytes, memory_ok, unhandled, unregistered,
           (unsigned long long)cpu->cycle_count);

    int ok = stage == SUCCESS_MARKER && results[0] == 40u &&
             results[1] == 40u && results[2] == 200u &&
             results[3] == 1u && results[4] == 0x60u &&
             results[5] == 0xC0u && results[6] == 0xA5u &&
             results[7] == 0xA5u && results[8] == 0x14u &&
             results[9] == 0x80003FFFu && results[10] == 1u &&
             results[11] == 0xFFu && results[12] == 0u &&
             results[13] == 0x80u && results[14] == 0x100u &&
             results[15] == 4u && device.calls == 4u &&
             device.write_bytes == 6u && device.read_bytes == 2u &&
             memory_ok && unhandled == 0 && unregistered == 0;

    flexe_session_destroy(session);
    elf_symbols_destroy(symbols);
    return ok ? 0 : 1;
}

int main(int argc, char **argv) {
    bool disable_jit = false;
    int arg = 1;
    if (argc > 1 && strcmp(argv[1], "--no-jit") == 0) {
        disable_jit = true;
        arg++;
    }
    if (argc - arg != 2) {
        fprintf(stderr, "usage: %s [--no-jit] FIRMWARE.bin FIRMWARE.elf\n",
                argv[0]);
        return 2;
    }
    return run_fixture(argv[arg], argv[arg + 1], disable_jit);
}
