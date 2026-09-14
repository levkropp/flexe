/* Replay an ESP-IDF USB Serial/JTAG driver session through Flexe's native
 * FreeRTOS and interrupt matrix, including two host OUT/guest IN exchanges. */
#include "flexe_session.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define USB_BASE 0x60038000u
#define RTC_USB_CONF 0x60008120u
#define MAX_CYCLES UINT64_C(2000000000)
#define BATCH 10000

volatile int emu_app_running = 1;

typedef struct {
    char data[65536];
    size_t len;
    bool overflow;
} capture_t;

static void capture_byte(void *ctx, uint8_t byte)
{
    capture_t *capture = ctx;
    if (capture->len + 1u >= sizeof(capture->data)) {
        capture->overflow = true;
        return;
    }
    capture->data[capture->len++] = (char)byte;
    capture->data[capture->len] = '\0';
}

static bool has(const capture_t *capture, const char *needle)
{
    return strstr(capture->data, needle) != NULL;
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

static bool run_until(flexe_session_t *session, capture_t *uart,
                      capture_t *usb, const char *uart_marker,
                      const char *usb_marker)
{
    xtensa_cpu_t *cpu0 = flexe_session_cpu(session, 0);
    xtensa_cpu_t *cpu1 = flexe_session_cpu(session, 1);
    while (cpu0->cycle_count < MAX_CYCLES) {
        if (uart->overflow || usb->overflow || has(uart, "USJ_FAIL") ||
            !cpu0->running || (cpu1 && cpu1->debug_break))
            return false;
        if (has(uart, uart_marker) && has(usb, usb_marker)) return true;
        if (flexe_session_run_core(session, 0, BATCH) < 0) return false;
        flexe_session_post_batch(session, BATCH);
    }
    return false;
}

static void show_failure(flexe_session_t *session,
                         const capture_t *uart, const capture_t *usb)
{
    xtensa_cpu_t *cpu0 = flexe_session_cpu(session, 0);
    size_t uart_start = uart->len > 400u ? uart->len - 400u : 0u;
    size_t usb_start = usb->len > 200u ? usb->len - 200u : 0u;
    fprintf(stderr, "USJ replay failed at cycle %llu pc=0x%08X\n",
            (unsigned long long)cpu0->cycle_count, cpu0->pc);
    fprintf(stderr, "UART tail: %s\n", uart->data + uart_start);
    fprintf(stderr, "USB tail: %s\n", usb->data + usb_start);
}

int main(int argc, char **argv)
{
    if (argc != 4) {
        fprintf(stderr, "usage: %s FIRMWARE.bin FIRMWARE.elf ROM.elf\n",
                argv[0]);
        return 2;
    }
    capture_t uart = {0};
    capture_t usb = {0};
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
        .usb_serial_jtag_cb = capture_byte,
        .usb_serial_jtag_ctx = &usb,
    };
    flexe_session_t *session = flexe_session_create(&config);
    if (!session) return 2;
    esp32_periph_t *periph = flexe_session_periph(session);
    bool ok = run_until(session, &uart, &usb,
                        "USJ_DRIVER_READY", "USJ_TX_READY\n");
    for (unsigned round = 1u; ok && round <= 2u; round++) {
        static const uint8_t command[] = { 'p', 'i', 'n', 'g' };
        if (periph_usb_serial_jtag_rx_inject(periph, command,
                                            sizeof(command)) !=
            sizeof(command)) {
            ok = false;
            break;
        }
        char uart_marker[32];
        char usb_marker[32];
        (void)snprintf(uart_marker, sizeof(uart_marker),
                       "USJ_ROUND %u count=4", round);
        (void)snprintf(usb_marker, sizeof(usb_marker),
                       "USJ_PONG_%u\n", round);
        ok = run_until(session, &uart, &usb,
                       uart_marker, usb_marker);
    }

    uint32_t mmio_digest = UINT32_C(2166136261);
    for (size_t i = 0u; ok &&
         i < periph_unhandled_audit_count(periph); i++) {
        periph_unhandled_site_t site;
        if (!periph_unhandled_audit_get(periph, i, &site)) {
            ok = false;
            break;
        }
        if ((site.address >= USB_BASE &&
             site.address < USB_BASE + 0x1000u) ||
            site.address == RTC_USB_CONF) {
            fprintf(stderr, "unsupported USB control register 0x%08X\n",
                    site.address);
            ok = false;
        }
        mmio_digest = digest_word(mmio_digest, site.address);
        mmio_digest = digest_word(mmio_digest, site.pc);
        mmio_digest = digest_word(mmio_digest, site.first_value);
        mmio_digest = digest_word(mmio_digest, site.count);
        mmio_digest = digest_word(mmio_digest, site.core);
        mmio_digest = digest_word(mmio_digest, site.write);
    }

    int unsupported = periph_unhandled_count(periph);
    if (ok) {
        printf("PASS: ESP-IDF USB Serial/JTAG driver completed two "
               "host-to-guest-to-host ping/pong rounds; %d other "
               "unsupported accesses remain visible\n", unsupported);
        printf("UART: %zu bytes fnv32=%08X\n", uart.len,
               digest_bytes(UINT32_C(2166136261), uart.data, uart.len));
        printf("USB: %zu bytes fnv32=%08X\n", usb.len,
               digest_bytes(UINT32_C(2166136261), usb.data, usb.len));
        printf("MMIO: %zu sites fnv32=%08X\n",
               periph_unhandled_audit_count(periph), mmio_digest);
    } else {
        show_failure(session, &uart, &usb);
    }
    flexe_session_destroy(session);
    return ok ? 0 : 1;
}
