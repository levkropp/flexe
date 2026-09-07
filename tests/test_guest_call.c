/* Regression tests for emulator-to-guest callback entry. */
#include "guest_call.h"

static uint32_t async_test_entry(int source, uint32_t frame_size)
{
    return ((frame_size >> 3) << 12) | ((uint32_t)source << 8) |
           (3u << 4) | 6u;
}

TEST(async_call_preserves_window_spill_area)
{
    xtensa_cpu_t cpu;
    setup(&cpu);

    const uint32_t callback = BASE;
    const uint32_t resume = BASE + 0x100u;
    const uint32_t sp = 0x3FFB3000u;
    static const uint32_t save_area[4] = {
        0xA0A0A0A0u, 0xA1A1A1A1u, 0xA2A2A2A2u, 0xA3A3A3A3u,
    };
    uint32_t original[8];
    uint32_t args[2] = { 0x11223344u, 0x55667788u };

    /* A real running task with its call8 caller still live. The four words
     * below a1 are that caller's architectural overflow/underflow save area,
     * not scratch storage for the emulator. */
    cpu.running = true;
    cpu.halted = false;
    cpu.exception = false;
    cpu.ps = 1u << 18;                 /* WOE, INTLEVEL=0, EXCM=0 */
    cpu.windowbase = 6;
    cpu.windowstart = (1u << 4) | (1u << 6);
    cpu.pc = resume;
    ar_write(&cpu, 1, sp);
    for (int i = 0; i < 8; i++) {
        original[i] = 0xB0000000u + (uint32_t)i;
        ar_write(&cpu, 8 + i, original[i]);
    }
    for (int i = 0; i < 4; i++)
        mem_write32(cpu.mem, sp - 16u + (uint32_t)i * 4u, save_area[i]);

    /* callback: ENTRY a1,16; RETW.N. Resume lands on NOP.N. */
    put_insn3(&cpu, callback, async_test_entry(1, 16));
    put_insn2(&cpu, callback + 3u, narrow(0xD, 15, 0, 1));
    put_insn2(&cpu, resume, narrow(0xD, 15, 0, 3));

    ASSERT_EQ(guest_call_async(&cpu, callback, args, 2), 0);
    ASSERT_EQ(ar_read(&cpu, 1), sp);
    for (int i = 0; i < 4; i++)
        ASSERT_EQ(mem_read32(cpu.mem, sp - 16u + (uint32_t)i * 4u),
                  save_area[i]);

    ASSERT_EQ(xtensa_step(&cpu), 0);   /* ENTRY */
    ASSERT_EQ(cpu.windowbase, 8);
    ASSERT_EQ(xtensa_step(&cpu), 0);   /* RETW.N -> async sentinel */
    ASSERT_EQ(cpu.windowbase, 6);
    ASSERT_EQ(xtensa_step(&cpu), 0);   /* restore continuation; execute NOP.N */

    ASSERT_EQ(cpu.pc, resume + 2u);
    ASSERT_EQ(ar_read(&cpu, 1), sp);
    ASSERT_FALSE(guest_call_async_busy(&cpu));
    for (int i = 0; i < 8; i++)
        ASSERT_EQ(ar_read(&cpu, 8 + i), original[i]);
    for (int i = 0; i < 4; i++)
        ASSERT_EQ(mem_read32(cpu.mem, sp - 16u + (uint32_t)i * 4u),
                  save_area[i]);

    teardown(&cpu);
}

static void run_guest_call_tests(void)
{
    TEST_SUITE("Guest Calls");
    RUN_TEST(async_call_preserves_window_spill_area);
}
