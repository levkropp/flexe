/* Regression tests for emulator-to-guest callback entry. */
#include "guest_call.h"

static uint32_t guest_call_test_rri8(int subop, int s, int t, int imm8)
{
    return (uint32_t)(((imm8 & 0xFF) << 16) | (subop << 12) |
                      (s << 8) | (t << 4) | 2);
}

static uint32_t async_test_entry(int source, uint32_t frame_size)
{
    return ((frame_size >> 3) << 12) | ((uint32_t)source << 8) |
           (3u << 4) | 6u;
}

typedef struct {
    bool seen;
    uint32_t sp;
    uint32_t root_link;
} guest_root_probe_t;

static int guest_root_probe(xtensa_cpu_t *cpu, uint32_t pc, void *opaque)
{
    guest_root_probe_t *probe = opaque;
    if (pc != BASE)
        return 0;
    probe->seen = true;
    probe->sp = ar_read(cpu, 1);
    probe->root_link = mem_read32(cpu->mem, probe->sp - 12u);
    /* Complete the synthetic call without depending on a callback body. */
    cpu->pc = 0x40001FF8u;
    cpu->_pc_written = true;
    return 1;
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

TEST(sync_call_uses_only_transient_private_stack)
{
    xtensa_cpu_t cpu;
    setup(&cpu);

    const uint32_t callback = BASE;
    const uint32_t private_stack = 0x7FFF0000u;
    const uint32_t rtc_slow_guard = 0x50001FFCu;
    const uint32_t arg = 0x89ABCDEFu;

    /* ENTRY a1,16; WSR a2,CPENABLE; S32I a2,a1,0; RETW.N. The store proves
     * that callback frames can really use the temporary page rather than
     * merely returning before touching their fabricated stack. CPENABLE is
     * task context too, so the injected frame must not leak its value. */
    put_insn3(&cpu, callback, async_test_entry(1, 16));
    put_insn3(&cpu, callback + 3u,
              rrr(1, 3, XT_SR_CPENABLE >> 4,
                  XT_SR_CPENABLE & 15, 2));
    put_insn3(&cpu, callback + 6u, guest_call_test_rri8(0x6, 1, 2, 0));
    put_insn2(&cpu, callback + 9u, narrow(0xD, 15, 0, 1));
    mem_write32(cpu.mem, rtc_slow_guard, 0xA5A55A5Au);
    cpu.cpenable = 0x5Au;

    ASSERT_TRUE(mem_get_ptr(cpu.mem, private_stack) == NULL);
    ASSERT_EQ(guest_call8(&cpu, callback, &arg, 1, 100u, NULL), 0);
    ASSERT_TRUE(mem_get_ptr(cpu.mem, private_stack) == NULL);
    ASSERT_EQ(mem_read32(cpu.mem, rtc_slow_guard), 0xA5A55A5Au);
    ASSERT_EQ(cpu.cpenable, 0x5Au);

    teardown(&cpu);
}

TEST(sync_call_builds_linked_window_root)
{
    xtensa_cpu_t cpu;
    setup(&cpu);
    guest_root_probe_t probe = {0};
    cpu.pc_hook = guest_root_probe;
    cpu.pc_hook_ctx = &probe;

    ASSERT_EQ(guest_call8(&cpu, BASE, NULL, 0, 10u, NULL), 0);
    ASSERT_TRUE(probe.seen);
    ASSERT_EQ(probe.sp, 0x7FFF1000u - 48u);
    ASSERT_EQ(probe.root_link, 0x7FFF1000u);
    ASSERT_TRUE(mem_get_ptr(cpu.mem, 0x7FFF0000u) == NULL);

    teardown(&cpu);
}

TEST(sync_injection_requires_all_live_cores_to_be_quiescent)
{
    xtensa_cpu_t target;
    xtensa_cpu_t peer;
    setup(&target);
    setup(&peer);

    target.running = true;
    target.halted = true;
    target.ps = 1u << 18; /* WOE, INTLEVEL=0, EXCM=0 */
    peer.running = true;
    peer.halted = true;
    peer.ps = 1u << 18;

    ASSERT_TRUE(guest_call_injection_is_quiescent(&target, &peer));

    target.halted = false;
    ASSERT_FALSE(guest_call_injection_is_quiescent(&target, &peer));
    target.halted = true;

    peer.halted = false;
    ASSERT_FALSE(guest_call_injection_is_quiescent(&target, &peer));
    peer.running = false;
    ASSERT_TRUE(guest_call_injection_is_quiescent(&target, &peer));

    target.exception = true;
    ASSERT_FALSE(guest_call_injection_is_quiescent(&target, &peer));
    target.exception = false;
    XT_PS_SET_INTLEVEL(target.ps, 1);
    ASSERT_FALSE(guest_call_injection_is_quiescent(&target, &peer));

    teardown(&peer);
    teardown(&target);
}

static void run_guest_call_tests(void)
{
    TEST_SUITE("Guest Calls");
    RUN_TEST(async_call_preserves_window_spill_area);
    RUN_TEST(sync_call_uses_only_transient_private_stack);
    RUN_TEST(sync_call_builds_linked_window_root);
    RUN_TEST(sync_injection_requires_all_live_cores_to_be_quiescent);
}
