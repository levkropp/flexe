#include "test_helpers.h"

int test_count = 0, test_passes = 0, test_failures = 0;

#include "test_decode.c"
#include "test_alu.c"
#include "test_shift.c"
#include "test_move.c"

int main(void) {
    printf("Running xtensa-emulator tests...\n\n");

    run_decode_tests();
    run_alu_tests();
    run_shift_tests();
    run_move_tests();

    printf("\n%d tests, %d passed, %d failed\n",
           test_count, test_passes, test_failures);
    return test_failures > 0 ? 1 : 0;
}
