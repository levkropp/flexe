#include "test_helpers.h"
#include "test_suites.h"

#include <string.h>

int test_count = 0, test_passes = 0, test_failures = 0;
/* Session-based tests link the same touch-stub lifecycle as the CLI. */
volatile int emu_app_running = 1;

static const char *test_suite = "";
static const char *test_filters[64];
static int test_filter_count;
static int test_matches;
static bool test_suite_printed;
static bool test_list_only;
static bool test_quiet;

static unsigned char test_ascii_lower(unsigned char ch)
{
    return ch >= 'A' && ch <= 'Z' ?
        (unsigned char)(ch + ('a' - 'A')) : ch;
}

static bool test_contains_folded(const char *text, const char *pattern)
{
    if (!*pattern) return true;
    for (; *text; text++) {
        const char *candidate = text;
        const char *needle = pattern;
        while (*candidate && *needle &&
               test_ascii_lower((unsigned char)*candidate) ==
               test_ascii_lower((unsigned char)*needle)) {
            candidate++;
            needle++;
        }
        if (!*needle) return true;
    }
    return false;
}

static bool test_selected(const char *name)
{
    if (test_filter_count == 0) return true;
    for (int index = 0; index < test_filter_count; index++)
        if (test_contains_folded(test_suite, test_filters[index]) ||
            test_contains_folded(name, test_filters[index]))
            return true;
    return false;
}

void test_set_suite(const char *name)
{
    test_suite = name;
    test_suite_printed = false;
}

bool test_begin(const char *name)
{
    if (!test_selected(name)) return false;
    test_matches++;
    if (test_list_only) {
        printf("%s :: %s\n", test_suite, name);
        return false;
    }
    if (!test_quiet && !test_suite_printed) {
        printf("Suite: %s\n", test_suite);
        test_suite_printed = true;
    }
    if (!test_quiet) printf("  %s... ", name);
    return true;
}

void test_end(const char *name, int failures_before)
{
    test_count++;
    if (test_failures == failures_before) {
        if (!test_quiet) printf("ok\n");
    } else if (test_quiet) {
        fprintf(stderr, "  FAILED %s :: %s\n", test_suite, name);
    } else {
        printf("\n");
    }
}

#ifndef FLEXE_HAS_JIT
static void run_jit_tests(void)
{
    printf("JIT tests skipped (no native backend)\n");
}
#endif

static void test_usage(const char *program)
{
    fprintf(stderr,
            "usage: %s [--list] [--quiet] [FILTER ...]\n"
            "Run all tests by default, or tests whose suite/test name "
            "contains any FILTER. --quiet prints only failures and the "
            "summary.\n",
            program);
}

int main(int argc, char **argv)
{
    for (int index = 1; index < argc; index++) {
        if (strcmp(argv[index], "--list") == 0) {
            test_list_only = true;
        } else if (strcmp(argv[index], "-q") == 0 ||
                   strcmp(argv[index], "--quiet") == 0) {
            test_quiet = true;
        } else if (strcmp(argv[index], "-h") == 0 ||
                   strcmp(argv[index], "--help") == 0) {
            test_usage(argv[0]);
            return 0;
        } else if (test_filter_count <
                   (int)(sizeof(test_filters) / sizeof(test_filters[0]))) {
            test_filters[test_filter_count++] = argv[index];
        } else {
            fprintf(stderr, "error: too many test filters\n");
            return 2;
        }
    }

    if (!test_list_only && !test_quiet)
        printf("Running xtensa-emulator tests...\n\n");

    run_decode_tests();
    run_alu_tests();
    run_shift_tests();
    run_move_tests();
    run_loadstore_tests();
    run_memory_tests();
    run_flash_mmu_tests();
    run_efuse_tests();
    run_target_gpio_tests();
    run_io_mux_tests();
    run_rtc_cntl_tests();
    run_rtc_io_tests();
    run_sens_tests();
    run_apb_saradc_tests();
    run_radio_tests();
    run_esp32s3_extmem_tests();
    run_assist_debug_tests();
    run_sensitive_memprot_tests();
    run_system_clock_tests();
    run_syscon_memory_tests();
    run_sandbox_input_tests();
    run_systimer_tests();
    run_timer_group_target_tests();
    run_rmt_v1_tests();
    run_i2s_v2_tests();
    run_lcd_cam_tests();
    run_twai_target_tests();
    run_pcnt_target_tests();
    run_mcpwm_target_tests();
    run_ledc_v1_tests();
    run_usb_serial_jtag_tests();
    run_spi_mem_tests();
    run_firmware_scan_tests();
    run_loader_tests();
    run_target_tests();
    run_rom_elf_tests();
    run_branch_tests();
    run_loop_tests();
    run_integration_tests();
    run_window_tests();
    run_guest_call_tests();
    run_exception_tests();
    run_boolean_tests();
    run_mac16_tests();
    run_fp_ldst_tests();
    run_fp_arith_tests();
    run_peripheral_tests();
    run_sx127x_tests();
    run_axp192_tests();
    run_ublox_gps_tests();
    run_crypto_tests();
    run_wifi_stub_tests();
    run_bt_stub_tests();
    run_rom_stub_tests();
    run_debug_tests();
    run_memory_map_tests();
    run_freertos_tests();
    run_esp_timer_tests();
    run_firmware_compat_tests();
    run_jit_tests();

    if (test_matches == 0 && test_filter_count != 0) {
        fprintf(stderr, "error: no test suite or test matched the filter(s)\n");
        return 2;
    }
    if (test_list_only) return 0;
    printf(test_quiet ? "%d tests, %d passed, %d failed\n" :
                        "\n%d tests, %d passed, %d failed\n",
           test_count, test_passes, test_failures);
    return test_failures > 0 ? 1 : 0;
}
