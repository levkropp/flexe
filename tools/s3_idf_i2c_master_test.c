/* Replay the stock ESP-IDF 5.3 I2C master driver against a host register
 * device through S3 MMIO, native FreeRTOS, and the real interrupt path. */
#include "elf_symbols.h"
#include "flexe_session.h"
#include "memory.h"
#include "peripherals.h"
#include "target.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define I2C_DONE UINT32_C(0x1C2C5343)
#define MAX_CYCLES UINT64_C(3000000000)
#define BATCH 10000u
#define GPIO_I2C_SDA_ROUTE UINT32_C(0x60004574)
#define GPIO_I2C_SCL_ROUTE UINT32_C(0x60004578)

volatile int emu_app_running = 1;

typedef struct {
    uint8_t reg[256];
    uint8_t cursor;
    unsigned calls;
    size_t written;
    size_t read;
} register_device_t;

typedef struct {
    char data[32768];
    size_t len;
} capture_t;

static int register_xfer(void *ctx, int port, uint8_t address,
                         const uint8_t *write_data, size_t write_len,
                         uint8_t *read_data, size_t read_len)
{
    register_device_t *device = ctx;
    if (port != 0 || address != 0x34) return -1;
    device->calls++;
    device->written += write_len;
    device->read += read_len;
    if (write_len != 0u) {
        device->cursor = write_data[0];
        for (size_t i = 1u; i < write_len; i++)
            device->reg[device->cursor++] = write_data[i];
    }
    for (size_t i = 0u; i < read_len; i++)
        read_data[i] = device->reg[device->cursor++];
    return 0;
}

static void capture_byte(void *ctx, uint8_t byte)
{
    capture_t *capture = ctx;
    if (capture->len + 1u >= sizeof(capture->data)) return;
    capture->data[capture->len++] = (char)byte;
    capture->data[capture->len] = '\0';
}

static uint32_t digest_bytes(uint32_t digest, const void *data, size_t len)
{
    const uint8_t *bytes = data;
    for (size_t i = 0u; i < len; i++)
        digest = (digest ^ bytes[i]) * UINT32_C(16777619);
    return digest;
}

static uint32_t digest_word(uint32_t digest, uint64_t value)
{
    for (unsigned i = 0u; i < 8u; i++) {
        uint8_t byte = (uint8_t)(value >> (i * 8u));
        digest = digest_bytes(digest, &byte, 1u);
    }
    return digest;
}

int main(int argc, char **argv)
{
    if (argc != 4) {
        fprintf(stderr, "usage: %s FIRMWARE.bin FIRMWARE.elf ROM.elf\n",
                argv[0]);
        return 2;
    }
    elf_symbols_t *symbols = elf_symbols_load(argv[2]);
    uint32_t stage_addr = 0u;
    uint32_t result_addr = 0u;
    if (!symbols ||
        elf_symbols_find(symbols, "flexe_i2c_stage", &stage_addr) != 0 ||
        elf_symbols_find(symbols, "flexe_i2c_result", &result_addr) != 0) {
        fprintf(stderr, "I2C fixture symbols missing\n");
        elf_symbols_destroy(symbols);
        return 2;
    }

    capture_t uart = {0};
    flexe_session_config_t config = {
        .bin_path = argv[1],
        .elf_path = argv[2],
        .rom_elf_path = argv[3],
        .native_freertos = 1,
        .disable_jit = 1,
        .target = FLEXE_TARGET_ESP32S3,
        .unhandled_audit = 1,
        .uart_cb = capture_byte,
        .uart_ctx = &uart,
    };
    flexe_session_t *session = flexe_session_create(&config);
    if (!session) {
        elf_symbols_destroy(symbols);
        return 2;
    }
    esp32_periph_t *periph = flexe_session_periph(session);
    register_device_t device = {0};
    if (periph_i2c_attach_device(periph, 0, 0x34,
                                 register_xfer, &device) != 0) {
        fprintf(stderr, "could not attach host I2C device\n");
        flexe_session_destroy(session);
        elf_symbols_destroy(symbols);
        return 2;
    }

    xtensa_cpu_t *cpu = flexe_session_cpu(session, 0);
    xtensa_mem_t *mem = flexe_session_mem(session);
    uint32_t stage = 0u;
    while (cpu->cycle_count < MAX_CYCLES) {
        stage = mem_read32(mem, stage_addr);
        if ((stage == I2C_DONE &&
             strstr(uart.data, "I2C_MASTER_DONE") != NULL) ||
            (stage & 0xFFFF0000u) == 0xBAD00000u ||
            strstr(uart.data, "I2C_FAIL") != NULL || !cpu->running)
            break;
        if (flexe_session_run_core(session, 0, BATCH) < 0) break;
        flexe_session_post_batch(session, BATCH);
    }
    stage = mem_read32(mem, stage_addr);
    uint32_t checksum = mem_read32(mem, result_addr);
    uint32_t nack = mem_read32(mem, result_addr + 4u);
    uint32_t failure_result = mem_read32(mem, result_addr + 12u);
    uint32_t expected_checksum = 0u;
    bool memory_ok = true;
    for (uint8_t i = 0u; i < 40u; i++) {
        uint8_t expected = (uint8_t)(0x5Au ^ i);
        expected_checksum = expected_checksum * 33u + expected;
        if (device.reg[0x20u + i] != expected) memory_ok = false;
    }
    const flexe_target_desc_t *target = flexe_target_by_id(config.target);
    unsigned i2c_unhandled_sites = 0u;
    unsigned unmodeled_matrix_routes = 0u;
    uint32_t mmio_digest = UINT32_C(2166136261);
    for (size_t i = 0u; i < periph_unhandled_audit_count(periph); i++) {
        periph_unhandled_site_t site;
        if (!periph_unhandled_audit_get(periph, i, &site)) continue;
        if (site.write &&
            ((site.address == GPIO_I2C_SDA_ROUTE &&
              site.first_value == 90u) ||
             (site.address == GPIO_I2C_SCL_ROUTE &&
              site.first_value == 89u)))
            unmodeled_matrix_routes++;
        mmio_digest = digest_word(mmio_digest, site.address);
        mmio_digest = digest_word(mmio_digest, site.pc);
        mmio_digest = digest_word(mmio_digest, site.first_value);
        mmio_digest = digest_word(mmio_digest, site.count);
        mmio_digest = digest_word(mmio_digest, site.core);
        mmio_digest = digest_word(mmio_digest, site.write);
        for (unsigned port = 0u; port < target->i2c.instance_count; port++) {
            uint32_t base = target->i2c.instance[port].base;
            if (site.address >= base &&
                site.address - base < target->i2c.register_size) {
                i2c_unhandled_sites++;
                fprintf(stderr, "unsupported I2C port%u %c 0x%08X "
                        "pc=0x%08X count=%llu\n",
                        port, site.write ? 'W' : 'R', site.address,
                        site.pc, (unsigned long long)site.count);
            }
        }
    }
    bool ok = stage == I2C_DONE && checksum == expected_checksum &&
              nack == 0x105u &&
              memory_ok && device.calls == 4u &&
              device.written == 42u && device.read == 40u &&
              i2c_unhandled_sites == 0u && unmodeled_matrix_routes == 2u &&
              periph_gpio_out_signal(periph, 8) == 90 &&
              periph_gpio_out_signal(periph, 9) == 89 &&
              periph_gpio_pin_level(periph, 8) == -1 &&
              periph_gpio_pin_level(periph, 9) == -1 &&
              strstr(uart.data, "I2C_MASTER_DONE") != NULL;
    printf("%s: ESP-IDF S3 I2C master stage=0x%08X checksum=%08X "
           "nack=0x%08X calls=%u write=%zu read=%zu memory_ok=%d "
           "i2c_unhandled_sites=%u unmodeled_matrix_routes=%u "
           "unhandled=%d cycles=%llu\n",
           ok ? "PASS" : "FAIL", stage, checksum, nack, device.calls,
           device.written, device.read, memory_ok, i2c_unhandled_sites,
           unmodeled_matrix_routes,
           periph_unhandled_count(periph),
           (unsigned long long)cpu->cycle_count);
    printf("UART: %zu bytes fnv32=%08X\n", uart.len,
           digest_bytes(UINT32_C(2166136261), uart.data, uart.len));
    printf("MMIO: %zu sites fnv32=%08X\n",
           periph_unhandled_audit_count(periph), mmio_digest);
    if (!ok)
        fprintf(stderr, "failure_result=0x%08X UART tail: %s\n",
                failure_result, uart.data +
                (uart.len > 500u ? uart.len - 500u : 0u));
    flexe_session_destroy(session);
    elf_symbols_destroy(symbols);
    return ok ? 0 : 1;
}
