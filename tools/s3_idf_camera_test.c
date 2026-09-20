/* Replay Espressif's pinned esp32-camera component through the stock S3
 * SCCB, LCD_CAM, GDMA, interrupt, and FreeRTOS paths. No guest function is
 * replaced: the host provides an OV2640 register device and parallel pixels. */
#include "elf_symbols.h"
#include "flexe_session.h"
#include "jit.h"
#include "memory.h"
#include "peripherals.h"
#include "target.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define CAMERA_DONE UINT32_C(0x43414D80)
#define FRAME_BYTES (160u * 120u * 2u)
#define MAX_CYCLES UINT64_C(5000000000)
#define BATCH 10000u
#define OV2640_ADDRESS 0x30u
#define OV2640_BANK_SELECT 0xFFu
#define OV2640_BANK_DSP 0u
#define OV2640_BANK_SENSOR 1u
#define OV2640_COM7 0x12u
#define OV2640_COM7_RESET 0x80u
#define S3_GPIO_BASE UINT32_C(0x60004000)
#define S3_GPIO_FUNC_IN_BASE UINT32_C(0x154)
#define S3_GPIO_FUNC_IN_SEL UINT32_C(0x3F)
#define S3_GPIO_FUNC_IN_MATRIX (UINT32_C(1) << 7)

volatile int emu_app_running = 1;

typedef struct {
    uint8_t reg[2][256];
    uint8_t bank;
    uint8_t cursor;
    unsigned calls;
    unsigned probes;
    unsigned writes;
    unsigned reads;
    unsigned bank_switches;
    unsigned resets;
} ov2640_t;

typedef struct {
    char data[32768];
    size_t length;
    bool overflow;
} uart_capture_t;

static void ov2640_reset_registers(ov2640_t *sensor)
{
    memset(sensor->reg, 0, sizeof(sensor->reg));
    sensor->reg[OV2640_BANK_SENSOR][0x0Au] = 0x26u;
    sensor->reg[OV2640_BANK_SENSOR][0x0Bu] = 0x42u;
    sensor->reg[OV2640_BANK_SENSOR][0x1Cu] = 0x7Fu;
    sensor->reg[OV2640_BANK_SENSOR][0x1Du] = 0xA2u;
    sensor->bank = OV2640_BANK_DSP;
    sensor->cursor = 0u;
}

static int ov2640_xfer(void *ctx, int port, uint8_t address,
                       const uint8_t *write_data, size_t write_len,
                       uint8_t *read_data, size_t read_len)
{
    ov2640_t *sensor = ctx;
    if (port != 1 || address != OV2640_ADDRESS) return -1;
    sensor->calls++;
    if (write_len == 0u && read_len == 0u) sensor->probes++;
    if (write_len != 0u) {
        sensor->cursor = write_data[0];
        for (size_t index = 1u; index < write_len; index++) {
            uint8_t reg = sensor->cursor++;
            uint8_t value = write_data[index];
            sensor->writes++;
            if (reg == OV2640_BANK_SELECT) {
                sensor->bank = value & 1u;
                sensor->bank_switches++;
            } else if (sensor->bank == OV2640_BANK_SENSOR &&
                       reg == OV2640_COM7 &&
                       (value & OV2640_COM7_RESET) != 0u) {
                sensor->resets++;
                ov2640_reset_registers(sensor);
                sensor->bank = OV2640_BANK_SENSOR;
            } else {
                sensor->reg[sensor->bank][reg] = value;
            }
        }
    }
    for (size_t index = 0u; index < read_len; index++) {
        read_data[index] = sensor->reg[sensor->bank][sensor->cursor++];
        sensor->reads++;
    }
    return 0;
}

static void capture_uart(void *ctx, uint8_t byte)
{
    uart_capture_t *capture = ctx;
    if (capture->length + 1u >= sizeof(capture->data)) {
        capture->overflow = true;
        return;
    }
    capture->data[capture->length++] = (char)byte;
    capture->data[capture->length] = '\0';
}

static uint8_t frame_byte(size_t index)
{
    return (uint8_t)(((index * 37u + 11u) ^ (index >> 5u)) & 0xFFu);
}

static uint32_t checksum(const uint8_t *data, size_t length)
{
    uint32_t digest = UINT32_C(2166136261);
    for (size_t index = 0u; index < length; index++)
        digest = (digest ^ data[index]) * UINT32_C(16777619);
    return digest;
}

static int input_route(xtensa_mem_t *mem, unsigned signal)
{
    uint32_t config = mem_read32(mem,
        S3_GPIO_BASE + S3_GPIO_FUNC_IN_BASE + signal * sizeof(uint32_t));
    if ((config & S3_GPIO_FUNC_IN_MATRIX) == 0u) return -1;
    return (int)(config & S3_GPIO_FUNC_IN_SEL);
}

int main(int argc, char **argv)
{
    int argi = 1;
    bool disable_jit = false;
    if (argi < argc && strcmp(argv[argi], "--no-jit") == 0) {
        disable_jit = true;
        argi++;
    }
    if (argc - argi != 3) {
        fprintf(stderr,
                "usage: %s [--no-jit] FIRMWARE.bin FIRMWARE.elf ROM.elf\n",
                argv[0]);
        return 2;
    }

    elf_symbols_t *symbols = elf_symbols_load(argv[argi + 1]);
    uint32_t stage_address = 0u;
    uint32_t result_address = 0u;
    if (!symbols ||
        elf_symbols_find(symbols, "flexe_camera_stage", &stage_address) != 0 ||
        elf_symbols_find(symbols, "flexe_camera_result", &result_address) != 0) {
        fprintf(stderr, "error: camera fixture marker symbols are missing\n");
        elf_symbols_destroy(symbols);
        return 2;
    }

    uart_capture_t uart = {0};
    flexe_session_config_t config = {
        .bin_path = argv[argi],
        .elf_path = argv[argi + 1],
        .rom_elf_path = argv[argi + 2],
        .native_freertos = 1,
        .disable_jit = disable_jit,
        .target = FLEXE_TARGET_ESP32S3,
        .unhandled_audit = 1,
        .uart_cb = capture_uart,
        .uart_ctx = &uart,
    };
    flexe_session_t *session = flexe_session_create(&config);
    if (!session) {
        elf_symbols_destroy(symbols);
        return 2;
    }

    esp32_periph_t *periph = flexe_session_periph(session);
    ov2640_t sensor = {0};
    ov2640_reset_registers(&sensor);
    if (periph_i2c_attach_device(periph, 1, OV2640_ADDRESS,
                                 ov2640_xfer, &sensor) != 0) {
        fprintf(stderr, "error: could not attach virtual OV2640\n");
        flexe_session_destroy(session);
        elf_symbols_destroy(symbols);
        return 2;
    }

    uint8_t frame[FRAME_BYTES];
    for (size_t index = 0u; index < sizeof(frame); index++)
        frame[index] = frame_byte(index);
    uint32_t expected_checksum = checksum(frame, sizeof(frame));

    xtensa_cpu_t *cpu0 = flexe_session_cpu(session, 0);
    xtensa_cpu_t *cpu1 = flexe_session_cpu(session, 1);
    xtensa_mem_t *mem = flexe_session_mem(session);
    bool leading_vsync = false;
    bool trailing_vsync = false;
    unsigned drain_batches = 0u;
    size_t injected = 0u;
    unsigned descriptors = 0u;
    int xclk_route = -1;
    int data0_route = -1;
    int data7_route = -1;
    int pclk_route = -1;
    int href_route = -1;
    int vsync_route = -1;
    uint32_t stage = 0u;
    while (cpu0->cycle_count < MAX_CYCLES) {
        stage = mem_read32(mem, stage_address);
        if (stage == 4u) {
            if (!leading_vsync) {
                xclk_route = periph_gpio_out_signal(periph, 15);
                data0_route = input_route(mem, 133u);
                data7_route = input_route(mem, 140u);
                pclk_route = input_route(mem, 149u);
                href_route = input_route(mem, 150u);
                vsync_route = input_route(mem, 152u);
                leading_vsync =
                    periph_lcd_cam_camera_vsync(periph) == 1;
            } else if (injected < sizeof(frame)) {
                size_t consumed = periph_lcd_cam_camera_rx_inject(
                    periph, frame + injected, sizeof(frame) - injected);
                if (consumed != 0u) {
                    injected += consumed;
                    descriptors++;
                }
            } else if (!trailing_vsync && ++drain_batches >= 20u) {
                trailing_vsync =
                    periph_lcd_cam_camera_vsync(periph) == 1;
            }
        }
        if ((stage == CAMERA_DONE &&
             strstr(uart.data, "CAMERA_DONE") != NULL) ||
            (stage & UINT32_C(0xFFFF0000)) == UINT32_C(0xBAD00000) ||
            strstr(uart.data, "CAMERA_FAIL") != NULL || uart.overflow ||
            !cpu0->running || (cpu1 && cpu1->debug_break))
            break;
        if (flexe_session_run_core(session, 0, BATCH) < 0) break;
        flexe_session_post_batch(session, BATCH);
    }

    stage = mem_read32(mem, stage_address);
    uint32_t init_error = mem_read32(mem, result_address);
    uint32_t sensor_pid = mem_read32(mem, result_address + 4u);
    uint32_t frame_length = mem_read32(mem, result_address + 8u);
    uint32_t frame_width = mem_read32(mem, result_address + 12u);
    uint32_t frame_height = mem_read32(mem, result_address + 16u);
    uint32_t guest_checksum = mem_read32(mem, result_address + 20u);
    uint32_t deinit_error = mem_read32(mem, result_address + 24u);
    uint32_t failure = mem_read32(mem, result_address + 28u);
    uint64_t jit_instructions = 0u;
    jit_state_t *jit = flexe_session_jit(session);
    if (jit) jit_instructions = jit_get_stats(jit)->insns_jitted;
    unsigned unhandled = (unsigned)periph_unhandled_count(periph);
    bool ok = stage == CAMERA_DONE && init_error == 0u &&
              sensor_pid == 0x26u && frame_length == sizeof(frame) &&
              frame_width == 160u && frame_height == 120u &&
              guest_checksum == expected_checksum && deinit_error == 0u &&
              leading_vsync && trailing_vsync &&
              injected == sizeof(frame) && descriptors == 10u &&
              sensor.probes == 1u && sensor.writes >= 150u &&
              sensor.reads >= 10u && sensor.bank_switches >= 10u &&
              sensor.resets >= 1u && xclk_route == 149 &&
              data0_route == 6 && data7_route == 13 && pclk_route == 17 &&
              href_route == 16 && vsync_route == 14 && unhandled == 0u &&
              strstr(uart.data, "CAMERA_DONE") != NULL &&
              (disable_jit || jit_instructions != 0u);

    printf("%s: ESP-IDF S3 esp32-camera engine=%s stage=0x%08X "
           "sensor=%02X frame=%ux%u/%u checksum=%08X inject=%zu/%u "
           "descriptors=%u vsync=%u/%u sccb=%u/%u/%u banks=%u resets=%u "
           "routes=%d,%d,%d,%d,%d,%d unhandled=%u cycles=%llu "
           "jit_insns=%llu\n",
           ok ? "PASS" : "FAIL", disable_jit ? "interp" : "jit", stage,
           sensor_pid, frame_width, frame_height, frame_length,
           guest_checksum, injected, (unsigned)sizeof(frame), descriptors,
           leading_vsync, trailing_vsync, sensor.probes, sensor.writes,
           sensor.reads, sensor.bank_switches, sensor.resets, xclk_route,
           data0_route, data7_route, pclk_route, href_route, vsync_route,
           unhandled, (unsigned long long)cpu0->cycle_count,
           (unsigned long long)jit_instructions);
    if (!ok)
        fprintf(stderr,
                "failure=0x%08X expected_checksum=%08X calls=%u "
                "UART tail: %s\n",
                failure, expected_checksum, sensor.calls,
                uart.data + (uart.length > 1200u ?
                             uart.length - 1200u : 0u));

    flexe_session_destroy(session);
    elf_symbols_destroy(symbols);
    return ok ? 0 : 1;
}
