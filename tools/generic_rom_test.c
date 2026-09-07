/* Boot an arbitrary ESP32 firmware image and report whether Flexe coped.
 *
 * The stock-ROM runner knows two firmwares by name and asserts what each one
 * is supposed to draw and do. That is the right shape for images we curate,
 * and the wrong shape for asking "can the emulator run this at all?" of
 * something downloaded ten minutes ago. This runner knows nothing about the
 * image: it boots it, lets it run for a fixed slice of *simulated* time, and
 * reports the handful of things that are true of any working ESP32 firmware.
 *
 * What it checks, and why each one is worth checking:
 *
 *  - **Unhandled MMIO is zero.** A register window Flexe does not decode means
 *    the firmware is talking to hardware that is not there. It is the single
 *    most reliable sign of a missing peripheral.
 *  - **Unregistered ROM calls are zero.** A call into ROM that Flexe does not
 *    stub returns 0 and keeps going, so it does not crash -- it silently does
 *    the wrong thing, usually in a loop.
 *  - **It retires instructions.** Firmware that boots and then idles for ever
 *    looks healthy from outside; a floor on retired instructions catches the
 *    difference between running and merely not crashing.
 *  - **It said something.** Retiring instructions is not enough: WLED sat in a
 *    PHY poll loop for 174 million instructions, printed not one byte, and
 *    passed every other check here. Any ESP32 image that reaches its
 *    application prints *something*, if only the ROM boot banner.
 *  - **No panic.** The guest's own abort/panic strings, watched on the UART.
 *  - **Both cores are on a plausible PC.** A core that has run off into the
 *    weeds usually keeps retiring instructions and printing nothing, and can
 *    otherwise satisfy every check above. Tasmota ended a run at
 *    pc1 = 0xFFE0FFE0 and passed.
 *
 * It deliberately does *not* check what the firmware computed. Cross-engine
 * agreement is the harness's job: run twice and compare the UART transcript,
 * which is the only output every firmware has in common.
 */
#include "flexe_session.h"
#include "jit.h"
#include "memory.h"
#include "peripherals.h"
#include "rom_stubs.h"
#include "xtensa.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define DEFAULT_CYCLES 2000000000ull
#define UART_LOG_MAX   65536u

volatile int emu_app_running = 1;

typedef struct {
    char     log[UART_LOG_MAX];
    size_t   len;
    uint64_t count;
    int      panic;
} uart_state_t;

/* Substrings the ESP32 panic handler and the Arduino core emit. Matching on
 * the transcript rather than on a PC keeps this independent of the image. */
static const char *PANIC_MARKERS[] = {
    "Guru Meditation", "abort() was called", "assert failed",
    "Backtrace:", "CORRUPT HEAP", "Stack canary watchpoint",
    NULL
};
/* Deliberately not "rst:0x": the ROM bootloader prints that on every normal
 * power-on, so it would fail every image that echoes its boot banner. */

static void uart_sink(void *ctx, uint8_t byte)
{
    uart_state_t *u = ctx;
    u->count++;
    if (u->len + 1 < sizeof(u->log)) {
        u->log[u->len++] = (char)byte;
        u->log[u->len] = '\0';
        if (u->panic >= 0) return;   /* already latched */
        for (int i = 0; PANIC_MARKERS[i]; i++)
            if (strstr(u->log, PANIC_MARKERS[i])) { u->panic = i; return; }
    }
}

/* FNV-1a over the transcript, with timestamps masked.
 *
 * Two engines running the same image must produce the same bytes -- but not
 * the same *times*. Firmware stamps its log from its own millisecond clock and
 * the engines reach a given line at slightly different simulated moments:
 * Tasmota prints "00:00:00.121 QPC: Reset" under the JIT and "00:00:00.134"
 * interpreted, openHASP "[       2.097]" against "[       2.098]". Hashing
 * those makes every timestamping firmware look miscompiled.
 *
 * Two forms are recognised, and both are deliberately narrow:
 *
 *  - A bracketed group of nothing but digits, spaces and dots, anywhere in the
 *    line. That covers Arduino's "[  4716]" and openHASP's "[       2.097]",
 *    which is *not* line-leading -- it follows ANSI cursor moves. It leaves
 *    "[E]", "[esp32-hal-cpu.c:244]" and openHASP's "[110580/182884 39]" heap
 *    figures alone, because those carry letters or a slash.
 *  - A line-leading "00:00:00.121 " clock.
 *
 * Everything else is hashed, so numbers inside a message still count. A block
 * that miscompiles and prints a wrong checksum is exactly what this comparison
 * is for, and masking every digit would hide it.
 */
static bool skip_bracketed_timestamp(const char *p, size_t len, size_t *i)
{
    size_t j = *i;
    if (j >= len || p[j] != '[') return false;
    size_t k = j + 1;
    bool any = false;
    while (k < len && (p[k] == ' ' || p[k] == '.' ||
                       (p[k] >= '0' && p[k] <= '9'))) {
        if (p[k] != ' ') any = true;
        k++;
    }
    if (k < len && p[k] == ']' && any) { *i = k + 1; return true; }
    return false;
}

/* ESP-IDF's own logger: "E (26) esp_littlefs: ...". The millisecond count is
 * in parentheses and follows the level letter, and the line usually starts
 * with an ANSI colour escape, so it is not line-leading. Require a trailing
 * space so a parenthesised value at the end of a message is left alone. */
static bool skip_paren_timestamp(const char *p, size_t len, size_t *i)
{
    size_t j = *i;
    if (j >= len || p[j] != '(') return false;
    size_t k = j + 1;
    bool digit = false;
    while (k < len && (p[k] == ' ' || (p[k] >= '0' && p[k] <= '9'))) {
        if (p[k] != ' ') digit = true;
        k++;
    }
    if (digit && k + 1 < len && p[k] == ')' && p[k + 1] == ' ') {
        *i = k + 1;
        return true;
    }
    return false;
}

static bool skip_clock_timestamp(const char *p, size_t len, size_t *i)
{
    size_t j = *i, k = j;
    while (k < len && ((p[k] >= '0' && p[k] <= '9') || p[k] == ':' ||
                       p[k] == '.')) k++;
    if (k > j && k < len && p[k] == ' ' && memchr(p + j, ':', k - j)) {
        *i = k;
        return true;
    }
    return false;
}

static uint32_t uart_digest(const uart_state_t *u)
{
    uint32_t h = 2166136261u;
    bool at_line_start = true;

    for (size_t i = 0; i < u->len; ) {
        if (at_line_start && skip_clock_timestamp(u->log, u->len, &i)) {
            at_line_start = false;
            continue;
        }
        if (skip_bracketed_timestamp(u->log, u->len, &i) ||
            skip_paren_timestamp(u->log, u->len, &i)) {
            at_line_start = false;
            continue;
        }
        char c = u->log[i++];
        at_line_start = (c == '\n');
        h ^= (uint8_t)c;
        h *= 16777619u;
    }
    return h;
}

static void usage(const char *argv0)
{
    fprintf(stderr,
            "usage: %s [--no-jit] [--cycles N] [--min-insns N] "
            "[--min-uart N] [--dump-uart] FIRMWARE.bin\n", argv0);
}

int main(int argc, char **argv)
{
    int argi = 1;
    int disable_jit = 0, dump_uart = 0;
    uint64_t budget = DEFAULT_CYCLES;
    uint64_t min_insns = 10000000ull;
    /* Any output. Deliberately not a guess at what the image should say: WLED
     * legitimately emits only its five-byte Adalight prompt, "Ada\r\n". */
    uint64_t min_uart = 1;

    while (argi < argc && argv[argi][0] == '-') {
        if (strcmp(argv[argi], "--no-jit") == 0) { disable_jit = 1; argi++; }
        else if (strcmp(argv[argi], "--dump-uart") == 0) { dump_uart = 1; argi++; }
        else if (strcmp(argv[argi], "--cycles") == 0 && argi + 1 < argc) {
            budget = strtoull(argv[argi + 1], NULL, 0); argi += 2;
        } else if (strcmp(argv[argi], "--min-insns") == 0 && argi + 1 < argc) {
            min_insns = strtoull(argv[argi + 1], NULL, 0); argi += 2;
        } else if (strcmp(argv[argi], "--min-uart") == 0 && argi + 1 < argc) {
            min_uart = strtoull(argv[argi + 1], NULL, 0); argi += 2;
        } else { usage(argv[0]); return 2; }
    }
    if (argc - argi != 1) { usage(argv[0]); return 2; }

    const char *rom_path = argv[argi];
    uart_state_t uart = {0};
    uart.panic = -1;

    flexe_session_config_t config = {
        .bin_path = rom_path,
        .disable_jit = disable_jit,
        .uart_cb = uart_sink,
        .uart_ctx = &uart,
        /* Headless: no framebuffer, and raw SPI sniffing off, because the
         * board this image targets is unknown and guessing CYD pin
         * assignments would misread whatever it does drive. */
        .spi_dc_pin = -1,
        .spi_display_cs_pin = -1,
        .spi_display_sck_pin = -1,
        .spi_touch_cs_pin = -1,
        .spi_touch_sck_pin = -1,
        .spi_touch_irq_pin = -1,
        .spi_sd_cs_pin = -1,
        .spi_sd_sck_pin = -1,
    };

    flexe_session_t *session = flexe_session_create(&config);
    if (!session) {
        fprintf(stderr, "FAIL image=%s reason=load-failed\n", rom_path);
        return 1;
    }

    xtensa_cpu_t *cpu0 = flexe_session_cpu(session, 0);
    uint64_t target = cpu0->cycle_count + budget;
    const char *stop = "budget";

    while (cpu0->cycle_count < target) {
        if (!cpu0->running && !flexe_session_cpu(session, 1)->running) {
            stop = "cpus-stopped";
            break;
        }
        if (uart.panic >= 0) { stop = "panic"; break; }
        (void)flexe_session_run_core(session, 0, 100000);
        flexe_session_post_batch(session, 100000);
    }

    /* Reported per core, because which core does the work is the first thing
     * you want to know about an unfamiliar image: WLED and openHASP run
     * essentially single-core (core 1 idle), while Tasmota puts its work on
     * core 1. That decides what "real-time" even requires of the emulator. */
    uint64_t r0 = xtensa_retired_insns(flexe_session_cpu(session, 0));
    uint64_t r1 = xtensa_retired_insns(flexe_session_cpu(session, 1));
    uint64_t retired = r0 + r1;
    int unhandled = periph_unhandled_count(flexe_session_periph(session));
    int unregistered = rom_stubs_unregistered_count(flexe_session_rom(session));
    xtensa_cpu_t *cpu1 = flexe_session_cpu(session, 1);

    /* ESP32 fetches only from ROM (0x40000000-0x4005FFFF), IRAM
     * (0x40070000-0x4009FFFF), RTC-fast (0x400C0000-0x400C1FFF) and mapped
     * flash (0x400D0000-0x403FFFFF). Anything else and the core is lost. */
    bool pc_ok = true;
    for (int c = 0; c < 2; c++) {
        uint32_t p = flexe_session_cpu(session, c)->pc;
        if (!((p >= 0x40000000u && p < 0x40060000u) ||
              (p >= 0x40070000u && p < 0x400A0000u) ||
              (p >= 0x400C0000u && p < 0x400C2000u) ||
              (p >= 0x400D0000u && p < 0x40400000u)))
            pc_ok = false;
    }

    bool ok = strcmp(stop, "budget") == 0 && uart.panic < 0 &&
              unhandled == 0 && unregistered == 0 && retired >= min_insns &&
              uart.count >= min_uart && pc_ok;

    printf("%s image=%s engine=%s stop=%s retired=%llu uart_bytes=%llu "
           "uart_digest=%08X unhandled=%d unregistered=%d "
           "core0=%llu core1=%llu unmapped=%llu/0x%08X "
           "pc0=0x%08X pc1=0x%08X resets=%u\n",
           ok ? "PASS" : "FAIL", rom_path,
           flexe_session_jit(session) ? "jit" : "interp", stop,
           (unsigned long long)retired, (unsigned long long)uart.count,
           uart_digest(&uart), unhandled, unregistered,
           (unsigned long long)r0, (unsigned long long)r1,
           (unsigned long long)mem_unmapped_count(flexe_session_mem(session)),
           mem_unmapped_first(flexe_session_mem(session)),
           cpu0->pc, cpu1->pc, flexe_session_reset_count(session));

    if (!ok)
        fprintf(stderr, "  budget_ok=%d panic=%s unhandled=%d unregistered=%d "
                        "retired=%llu (min %llu) uart=%llu (min %llu) "
                        "pc_ok=%d\n",
                strcmp(stop, "budget") == 0,
                uart.panic >= 0 ? PANIC_MARKERS[uart.panic] : "none",
                unhandled, unregistered,
                (unsigned long long)retired, (unsigned long long)min_insns,
                (unsigned long long)uart.count, (unsigned long long)min_uart,
                pc_ok);

    if (dump_uart || !ok) {
        fprintf(stderr, "--- UART (%zu bytes) ---\n", uart.len);
        fwrite(uart.log, 1, uart.len, stderr);
        fputc('\n', stderr);
    }

    /* No-op unless built with -DFLEXE_PROFILE=ON and run with FLEXE_PROFILE=1.
     * "Which task is burning the machine, and where" is the first question to
     * ask of a production image that boots and then goes quiet. */
    xtensa_profile_report();

    if (getenv("FLEXE_JIT_STATS") && flexe_session_jit(session))
        jit_print_stats(flexe_session_jit(session), retired);

    flexe_session_destroy(session);
    return ok ? 0 : 1;
}
