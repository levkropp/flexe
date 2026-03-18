#include "test_helpers.h"

int test_count = 0, test_passes = 0, test_failures = 0;

#include "test_decode.c"
#include "test_alu.c"
#include "test_shift.c"
#include "test_move.c"
#include "test_loadstore.c"
#include "test_memory.c"
#include "test_loader.c"
#include "test_branch.c"
#include "test_loop.c"
#include "test_integration.c"
#include "test_window.c"
#include "test_exception.c"
#include "test_boolean.c"
#include "test_mac16.c"
#include "test_fp_ldst.c"
#include "test_fp_arith.c"
#include "test_peripherals.c"
#include "test_rom_stubs.c"
#include "test_debug.c"
#include "test_memory_map.c"
#include "test_freertos.c"
#include "test_esp_timer.c"
#include "test_firmware_compat.c"
#include "test_jit.c"

int main(void) {
    printf("Running xtensa-emulator tests...\n\n");

    run_decode_tests();
    run_alu_tests();
    run_shift_tests();
    run_move_tests();
    run_loadstore_tests();
    run_memory_tests();
    run_loader_tests();
    run_branch_tests();
    run_loop_tests();
    run_integration_tests();
    run_window_tests();
    run_exception_tests();
    run_boolean_tests();
    run_mac16_tests();
    run_fp_ldst_tests();
    run_fp_arith_tests();
    run_peripheral_tests();
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
