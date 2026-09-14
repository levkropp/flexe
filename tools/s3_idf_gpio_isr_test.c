/* Exercise the official S3 ESP-IDF GPIO ISR service from host pin edges. */
#include "flexe_session.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define GPIO_BASE 0x60004000u
#define GPIO_PIN4_MUX 0x60009014u
#define MAX_CYCLES UINT64_C(2000000000)
#define BATCH 10000

volatile int emu_app_running = 1;

typedef struct {
    char data[32768];
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

static bool run_batch(flexe_session_t *session)
{
    if (flexe_session_run_core(session, 0, BATCH) < 0) return false;
    flexe_session_post_batch(session, BATCH);
    return true;
}

static bool run_until(flexe_session_t *session, capture_t *uart,
                      const char *marker)
{
    xtensa_cpu_t *cpu0 = flexe_session_cpu(session, 0);
    xtensa_cpu_t *cpu1 = flexe_session_cpu(session, 1);
    while (cpu0->cycle_count < MAX_CYCLES) {
        if (uart->overflow || has(uart, "GPIO_FAIL") || !cpu0->running ||
            (cpu1 && cpu1->debug_break))
            return false;
        if (has(uart, marker)) return true;
        if (!run_batch(session)) return false;
    }
    return false;
}

static bool run_quiet(flexe_session_t *session, capture_t *uart)
{
    for (unsigned i = 0u; i < 20u; i++) {
        if (!run_batch(session) || uart->overflow ||
            has(uart, "GPIO_FAIL") ||
            has(uart, "GPIO_ISR_EDGE round=2"))
            return false;
    }
    return true;
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
    if (!session) return 2;
    esp32_periph_t *periph = flexe_session_periph(session);
    periph_gpio_set_input(periph, 4, 0);

    bool ok = run_until(session, &uart, "GPIO_ISR_READY");
    if (ok && (periph_gpio_pin_level(periph, 5) != 1 ||
               periph_gpio_output_enabled(periph, 5) != 0)) {
        fprintf(stderr, "GPIO5 open-drain did not release high\n");
        ok = false;
    }
    if (ok) {
        periph_gpio_set_input(periph, 4, 1);
        ok = run_until(session, &uart,
                       "GPIO_OD_LOW") &&
             has(&uart, "GPIO_ISR_EDGE round=1 count=1 level=1");
        if (ok && (periph_gpio_pin_level(periph, 5) != 0 ||
                   periph_gpio_output_enabled(periph, 5) != 1)) {
            fprintf(stderr, "GPIO5 open-drain did not drive low\n");
            ok = false;
        }
    }
    if (ok) {
        periph_gpio_set_input(periph, 4, 1); /* No new edge. */
        ok = run_quiet(session, &uart);
    }
    if (ok) {
        periph_gpio_set_input(periph, 4, 0); /* Falling is disabled. */
        ok = run_quiet(session, &uart);
    }
    if (ok) {
        periph_gpio_set_input(periph, 4, 1);
        ok = run_until(session, &uart, "GPIO_ISR_DONE") &&
             has(&uart, "GPIO_ISR_EDGE round=2 count=2 level=1") &&
             has(&uart, "GPIO_OD_RELEASED");
        if (ok && (periph_gpio_pin_level(periph, 5) != 1 ||
                   periph_gpio_output_enabled(periph, 5) != 0)) {
            fprintf(stderr, "GPIO5 open-drain did not release again\n");
            ok = false;
        }
    }

    uint32_t mmio_digest = UINT32_C(2166136261);
    for (size_t i = 0u; ok &&
         i < periph_unhandled_audit_count(periph); i++) {
        periph_unhandled_site_t site;
        if (!periph_unhandled_audit_get(periph, i, &site)) {
            ok = false;
            break;
        }
        if ((site.address >= GPIO_BASE &&
             site.address < GPIO_BASE + 0x1000u) ||
            site.address == GPIO_PIN4_MUX) {
            fprintf(stderr, "unsupported GPIO control register 0x%08X\n",
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
        printf("PASS: ESP-IDF S3 GPIO ISR service handled two host rising "
               "edges, ignored stable/falling input, notified its task, "
               "and released/drove GPIO5 open-drain; %d other unsupported "
               "accesses remain visible\n",
               unsupported);
        printf("UART: %zu bytes fnv32=%08X\n", uart.len,
               digest_bytes(UINT32_C(2166136261), uart.data, uart.len));
        printf("MMIO: %zu sites fnv32=%08X\n",
               periph_unhandled_audit_count(periph), mmio_digest);
    } else {
        xtensa_cpu_t *cpu0 = flexe_session_cpu(session, 0);
        size_t start = uart.len > 400u ? uart.len - 400u : 0u;
        fprintf(stderr, "GPIO ISR replay failed at cycle %llu "
                "pc=0x%08X\nUART tail: %s\n",
                (unsigned long long)cpu0->cycle_count, cpu0->pc,
                uart.data + start);
    }
    flexe_session_destroy(session);
    return ok ? 0 : 1;
}
