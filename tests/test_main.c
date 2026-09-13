#include "test_helpers.h"

int test_count = 0, test_passes = 0, test_failures = 0;
/* Session-based tests link the same touch-stub lifecycle as the CLI. */
volatile int emu_app_running = 1;

#include "test_decode.c"
#include "test_alu.c"
#include "test_shift.c"
#include "test_move.c"
#include "test_loadstore.c"
#include "test_memory.c"
#include "test_flash_mmu.c"
#include "test_efuse.c"
#include "test_gpio.c"
#include "test_io_mux.c"
#include "test_rtc_cntl.c"
#include "test_sens.c"
#include "test_radio.c"
#include "test_esp32s3_extmem.c"
#include "test_system_clock.c"
#include "test_systimer.c"
#include "test_timer_group.c"
#include "test_rmt_v1.c"
#include "test_usb_serial_jtag.c"
#include "test_spi_mem.c"
#include "test_firmware_scan.c"
#include "test_loader.c"
#include "test_target.c"
#include "test_rom_elf.c"
#include "test_branch.c"
#include "test_loop.c"
#include "test_integration.c"
#include "test_window.c"
#include "test_guest_call.c"
#include "test_exception.c"
#include "test_boolean.c"
#include "test_mac16.c"
#include "test_fp_ldst.c"
#include "test_fp_arith.c"
#include "test_peripherals.c"
#include "test_sx127x.c"
#include "test_axp192.c"
#include "test_ublox_gps.c"
#include "test_crypto.c"
#include "test_wifi_stubs.c"
#include "test_bt_stubs.c"
#include "test_rom_stubs.c"
#include "test_debug.c"
#include "test_memory_map.c"
#include "test_freertos.c"
#include "test_esp_timer.c"
#include "test_firmware_compat.c"
#if defined(__x86_64__) || defined(_M_X64) || defined(__aarch64__) || defined(_M_ARM64)
#include "test_jit.c"
#else
static void run_jit_tests(void) { printf("JIT tests skipped (no native backend)\n"); }
#endif

int main(void) {
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
    run_sens_tests();
    run_radio_tests();
    run_esp32s3_extmem_tests();
    run_system_clock_tests();
    run_systimer_tests();
    run_timer_group_target_tests();
    run_rmt_v1_tests();
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

    printf("\n%d tests, %d passed, %d failed\n",
           test_count, test_passes, test_failures);
    return test_failures > 0 ? 1 : 0;
}
