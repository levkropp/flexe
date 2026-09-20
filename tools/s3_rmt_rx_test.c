/* Exercise the stock Arduino-ESP32 3.x RX API and ESP-IDF RMT ISR with short
 * and wrap/ping-pong pulse frames injected through the hardware boundary. */
#include "elf_symbols.h"
#include "flexe_session.h"
#include "jit.h"
#include "memory.h"
#include "peripherals.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define SUCCESS_MARKER 0x01A7C0DEu
#define MAX_CYCLES     3000000000ull
#define RMT_RX_CONF0   0x60016030u
#define RMT_RX_CONF4   0x60016034u
#define RMT_RX_CARRIER 0x60016090u
#define RMT_SYS_CONF   0x600160C0u
#define RX_GPIO        4

volatile int emu_app_running = 1;

static uint32_t guest_cycles_for_rmt_ticks(xtensa_cpu_t *cpu,
                                            xtensa_mem_t *mem,
                                            uint32_t ticks)
{
    uint32_t sys = mem_read32(mem, RMT_SYS_CONF);
    uint32_t conf = mem_read32(mem, RMT_RX_CONF0);
    uint32_t source = 0u;
    switch ((sys >> 24u) & 3u) {
    case 1u: source = 80000000u; break;
    case 2u: source = 8000000u; break;
    case 3u: source = 40000000u; break;
    default: return 0u;
    }
    uint32_t group_div = ((sys >> 4u) & 0xFFu) + 1u;
    uint32_t ch_div = conf & 0xFFu;
    if (!ch_div) ch_div = 256u;
    uint32_t frac_num = (sys >> 12u) & 0x3Fu;
    uint32_t frac_den = (sys >> 18u) & 0x3Fu;
    uint32_t den = frac_den ? frac_den : 1u;
    uint64_t numerator = (uint64_t)ticks *
        xtensa_cpu_freq_mhz(cpu) * 1000000u * ch_div *
        ((uint64_t)group_div * den + (frac_den ? frac_num : 0u));
    uint64_t denominator = (uint64_t)source * den;
    return (uint32_t)((numerator + denominator - 1u) / denominator);
}

static int run_guest_cycles(flexe_session_t *session, xtensa_cpu_t *cpu,
                            uint32_t requested)
{
    uint32_t start = cpu->ccount;
    while ((uint32_t)(cpu->ccount - start) < requested) {
        uint32_t remaining = requested - (uint32_t)(cpu->ccount - start);
        int ran = flexe_session_run_core(session, 0, (int)remaining);
        if (ran <= 0) return 0;
        flexe_session_post_batch(session, ran);
    }
    return 1;
}

static int feed_short_gpio_frame(flexe_session_t *session,
                                  xtensa_cpu_t *cpu, xtensa_mem_t *mem,
                                  esp32_periph_t *periph)
{
    static const uint32_t duration[] = {10u, 12u, 8u, 9u};
    fprintf(stderr, "[s3-rmt-rx] rx_conf0=%08X rx_conf1=%08X sys=%08X\n",
            mem_read32(mem, RMT_RX_CONF0), mem_read32(mem, RMT_RX_CONF4),
            mem_read32(mem, RMT_SYS_CONF));
    if (!(mem_read32(mem, RMT_RX_CONF4) & (1u << 4))) {
        fprintf(stderr, "[s3-rmt-rx] stock RX filter was not armed\n");
        return 0;
    }
    periph_gpio_set_input(periph, RX_GPIO, 0);
    /* One channel tick is shorter than the driver's three-tick minimum.
     * This high glitch must not start a frame or change the first symbol. */
    periph_gpio_set_input(periph, RX_GPIO, 1);
    uint32_t glitch_cycles = guest_cycles_for_rmt_ticks(cpu, mem, 1u);
    if (!glitch_cycles ||
        !run_guest_cycles(session, cpu, glitch_cycles)) return 0;
    periph_gpio_set_input(periph, RX_GPIO, 0);
    if (!run_guest_cycles(session, cpu, glitch_cycles)) return 0;
    periph_gpio_set_input(periph, RX_GPIO, 1);
    for (unsigned i = 0u; i < 4u; i++) {
        uint32_t cycles = guest_cycles_for_rmt_ticks(cpu, mem, duration[i]);
        if (!cycles || !run_guest_cycles(session, cpu, cycles)) return 0;
        periph_gpio_set_input(periph, RX_GPIO, (i & 1u) != 0u);
    }
    return 1;
}

static int feed_carrier_gpio_frame(flexe_session_t *session,
                                   xtensa_cpu_t *cpu, xtensa_mem_t *mem,
                                   esp32_periph_t *periph)
{
    uint32_t conf = mem_read32(mem, RMT_RX_CONF0);
    uint32_t carrier = mem_read32(mem, RMT_RX_CARRIER);
    fprintf(stderr, "[s3-rmt-rx] demod_conf=%08X carrier=%08X\n",
            conf, carrier);
    if (!(conf & (1u << 28)) || !(conf & (1u << 29)) ||
        (carrier & 0xFFFFu) < 20u) return 0;

    periph_gpio_set_input(periph, RX_GPIO, 0);
    if (!run_guest_cycles(session, cpu,
                          guest_cycles_for_rmt_ticks(cpu, mem, 5u)))
        return 0;
    /* Four 9-high/17-low carrier periods form an 87-tick high envelope.
     * A 50-tick low space separates it from the next high envelope. */
    for (unsigned i = 0u; i < 4u; i++) {
        periph_gpio_set_input(periph, RX_GPIO, 1);
        if (!run_guest_cycles(session, cpu,
                              guest_cycles_for_rmt_ticks(cpu, mem, 9u)))
            return 0;
        periph_gpio_set_input(periph, RX_GPIO, 0);
        if (!run_guest_cycles(session, cpu,
                              guest_cycles_for_rmt_ticks(
                                  cpu, mem, i == 3u ? 50u : 17u)))
            return 0;
    }
    periph_gpio_set_input(periph, RX_GPIO, 1);
    if (!run_guest_cycles(session, cpu,
                          guest_cycles_for_rmt_ticks(cpu, mem, 10u)))
        return 0;
    periph_gpio_set_input(periph, RX_GPIO, 0);
    return run_guest_cycles(session, cpu,
                            guest_cycles_for_rmt_ticks(cpu, mem, 230u));
}

static int pulse_close_to(uint32_t word, unsigned shift, unsigned level,
                          unsigned nominal)
{
    unsigned half = (word >> shift) & 0xFFFFu;
    unsigned ticks = half & 0x7FFFu;
    return (half >> 15u) == level && ticks + 1u >= nominal &&
           ticks <= nominal + 1u;
}

int main(int argc, char **argv)
{
    int argi = 1;
    int disable_jit = 0;
    if (argi < argc && strcmp(argv[argi], "--no-jit") == 0) {
        disable_jit = 1;
        argi++;
    }
    if (argc - argi != 3) {
        fprintf(stderr,
                "usage: %s [--no-jit] FIRMWARE.merged.bin FIRMWARE.elf ROM.elf\n",
                argv[0]);
        return 2;
    }
    elf_symbols_t *symbols = elf_symbols_load(argv[argi + 1]);
    uint32_t stage_addr = 0u;
    uint32_t result_addr = 0u;
    if (!symbols ||
        elf_symbols_find(symbols, "flexe_rmt_rx_stage", &stage_addr) != 0 ||
        elf_symbols_find(symbols, "flexe_rmt_rx_result", &result_addr) != 0) {
        fprintf(stderr, "error: fixture marker symbols are missing\n");
        elf_symbols_destroy(symbols);
        return 2;
    }
    flexe_session_config_t config = {
        .bin_path = argv[argi],
        .elf_path = argv[argi + 1],
        .rom_elf_path = argv[argi + 2],
        .disable_jit = disable_jit,
        .native_freertos = 1,
        .target = FLEXE_TARGET_ESP32S3,
        .unhandled_audit = 1,
    };
    flexe_session_t *session = flexe_session_create(&config);
    if (!session) {
        elf_symbols_destroy(symbols);
        return 2;
    }
    xtensa_cpu_t *cpu = flexe_session_cpu(session, 0);
    xtensa_mem_t *mem = flexe_session_mem(session);
    esp32_periph_t *periph = flexe_session_periph(session);
    uint32_t expected_long[96];
    for (unsigned i = 0u; i < 96u; i++)
        expected_long[i] = (1u + i % 16u) | ((1u + i % 7u) << 16) |
                           ((i & 1u) << 15);
    uint32_t stage = 0u;
    int gpio_short = 0;
    int injected_long = 0;
    int gpio_carrier = 0;
    while (cpu->cycle_count < MAX_CYCLES) {
        stage = mem_read32(mem, stage_addr);
        if (stage == 1u && !gpio_short &&
            (mem_read32(mem, RMT_RX_CONF4) & 1u)) {
            if (!feed_short_gpio_frame(session, cpu, mem, periph)) {
                fprintf(stderr, "short GPIO waveform stalled\n");
                break;
            }
            gpio_short = 1;
        } else if (stage == 2u && !injected_long &&
                   (mem_read32(mem, RMT_RX_CONF4) & 1u)) {
            size_t accepted = periph_rmt_rx_inject(periph, 4,
                                                   expected_long, 96u);
            if (accepted != 96u) {
                fprintf(stderr, "long RX accepted %zu/96 symbols\n",
                        accepted);
                break;
            }
            injected_long = 1;
        } else if (stage == 3u && !gpio_carrier &&
                   (mem_read32(mem, RMT_RX_CONF4) & 1u)) {
            if (!feed_carrier_gpio_frame(session, cpu, mem, periph)) {
                fprintf(stderr, "carrier GPIO waveform stalled\n");
                break;
            }
            gpio_carrier = 1;
        }
        if (stage == SUCCESS_MARKER || (stage & 0xFFF00000u) == 0xBAD00000u)
            break;
        (void)flexe_session_run_core(session, 0, 10000);
        flexe_session_post_batch(session, 10000);
    }
    stage = mem_read32(mem, stage_addr);
    uint32_t count = mem_read32(mem, result_addr);
    uint32_t first = mem_read32(mem, result_addr + 4u);
    uint32_t second = mem_read32(mem, result_addr + 8u);
    uint32_t long_count = mem_read32(mem, result_addr + 12u);
    uint32_t carrier_count = mem_read32(mem, result_addr + 100u * 4u);
    uint32_t carrier_first = mem_read32(mem, result_addr + 101u * 4u);
    uint32_t carrier_second = mem_read32(mem, result_addr + 102u * 4u);
    int long_match = long_count == 96u;
    for (unsigned i = 0u; i < 96u; i++) {
        uint32_t actual = mem_read32(mem, result_addr + (4u + i) * 4u);
        if (actual != expected_long[i]) {
            if (long_match)
                fprintf(stderr,
                        "[s3-rmt-rx] long symbol %u: got %08X expected %08X\n",
                        i, actual, expected_long[i]);
            long_match = 0;
        }
    }
    uint64_t cycles = cpu->cycle_count;
    jit_state_t *jit = flexe_session_jit(session);
    uint64_t jit_insns = jit ? jit_get_stats(jit)->insns_jitted : 0u;
    int jit_ok = disable_jit || jit_insns > 0u;
    unsigned unhandled = periph_unhandled_count(periph);
    unsigned rmt_unhandled = 0u;
    for (size_t i = 0u; i < periph_unhandled_audit_count(periph); i++) {
        periph_unhandled_site_t site;
        if (periph_unhandled_audit_get(periph, i, &site) &&
            site.address >= 0x60016000u && site.address < 0x60017000u) {
            rmt_unhandled++;
            fprintf(stderr, "[s3-rmt-rx] unsupported RMT %c 0x%08X "
                    "pc=0x%08X value=0x%08X count=%llu\n",
                    site.write ? 'W' : 'R', site.address, site.pc,
                    site.first_value, (unsigned long long)site.count);
        }
    }
    int short_match = count == 2u &&
        pulse_close_to(first, 0u, 1u, 10u) &&
        pulse_close_to(first, 16u, 0u, 12u) &&
        pulse_close_to(second, 0u, 1u, 8u) &&
        pulse_close_to(second, 16u, 0u, 9u);
    int carrier_match = carrier_count == 2u &&
        pulse_close_to(carrier_first, 0u, 1u, 87u) &&
        pulse_close_to(carrier_first, 16u, 0u, 50u) &&
        pulse_close_to(carrier_second, 0u, 1u, 10u);
    int ok = gpio_short && injected_long && gpio_carrier &&
             stage == SUCCESS_MARKER && short_match && long_match &&
             carrier_match && unhandled == 0u && rmt_unhandled == 0u &&
             jit_ok;
    fprintf(stderr,
            "[s3-rmt-rx] engine=%s stage=0x%08X gpio=%d injected_long=%d "
            "gpio_carrier=%d counts=%u,%u,%u short=%08X,%08X "
            "carrier=%08X,%08X short_match=%d long_match=%d "
            "carrier_match=%d "
            "cycles=%llu jit_insns=%llu unhandled=%u "
            "rmt_unhandled_sites=%u\n",
            disable_jit ? "interp" : "jit", stage, gpio_short,
            injected_long, gpio_carrier,
            count, long_count, carrier_count, first, second,
            carrier_first, carrier_second,
            short_match, long_match, carrier_match,
            (unsigned long long)cycles, (unsigned long long)jit_insns, unhandled,
            rmt_unhandled);
    flexe_session_destroy(session);
    elf_symbols_destroy(symbols);
    return ok ? 0 : 1;
}
