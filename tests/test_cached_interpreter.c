/* Differential tests: cached batches must expose the same instruction,
 * window, memory and interrupt boundaries as uncached single stepping. */
#include "test_helpers.h"
#include <stdlib.h>
#include <string.h>

static void pair_init(xtensa_cpu_t *cached, xtensa_cpu_t *reference) {
    setup(cached);
    setup(reference);
    cached->running = reference->running = true;
}

static void pair_build(xtensa_cpu_t *cached, xtensa_cpu_t *reference) {
    xtensa_mem_t *mem = reference->mem;
    *reference = *cached;
    reference->mem = mem;
    memcpy(mem->sram, cached->mem->sram, mem->backing_size[FLEXE_MEM_SRAM]);
    memcpy(mem->flash_insn, cached->mem->flash_insn, mem->backing_size[FLEXE_MEM_FLASH_INSN]);
    xtensa_predecode_build(cached);
}

static void pair_destroy(xtensa_cpu_t *cached, xtensa_cpu_t *reference) {
    free(cached->predecode);
    teardown(cached);
    teardown(reference);
}

static int reference_run(xtensa_cpu_t *cpu, int budget) {
    int ran = 0;
    if (cpu->debug_break) return 0;
    while (ran < budget) {
        int result = xtensa_step(cpu);
        ran++;
        if (result < 0 || !cpu->running || cpu->core_handoff) break;
    }
    return ran;
}

static void pair_compare(xtensa_cpu_t *cached, xtensa_cpu_t *reference, int budget) {
    ASSERT_EQ(xtensa_run(cached, budget), reference_run(reference, budget));
#define SAME(field) ASSERT_EQ(cached->field, reference->field)
    SAME(pc); SAME(dbg_prev_pc); SAME(ccount); SAME(ps); SAME(windowbase);
    SAME(windowstart); SAME(window_hazard); SAME(sar); SAME(br);
    SAME(lbeg); SAME(lend); SAME(lcount); SAME(interrupt); SAME(intenable);
    SAME(next_timer_event); SAME(irq_check); SAME(_pc_written);
    SAME(running); SAME(halted); SAME(exception); SAME(exccause); SAME(excvaddr);
    SAME(debug_break); SAME(debugcause); SAME(breakpoint_hit); SAME(breakpoint_hit_addr);
    SAME(core_handoff);
#undef SAME
    ASSERT_EQ64(cached->cycle_count, reference->cycle_count);
    ASSERT_EQ64(cached->insn_count, reference->insn_count);
    ASSERT_EQ(memcmp(cached->ar, reference->ar, sizeof(cached->ar)), 0);
    ASSERT_EQ(memcmp(cached->fr, reference->fr, sizeof(cached->fr)), 0);
    ASSERT_EQ(memcmp(cached->epc, reference->epc, sizeof(cached->epc)), 0);
    ASSERT_EQ(memcmp(cached->eps, reference->eps, sizeof(cached->eps)), 0);
    ASSERT_EQ(memcmp(cached->ccompare, reference->ccompare, sizeof(cached->ccompare)), 0);
    ASSERT_EQ(memcmp(cached->window_callsize, reference->window_callsize,
                     sizeof(cached->window_callsize)), 0);
    ASSERT_EQ(memcmp(cached->mem->sram, reference->mem->sram,
                     cached->mem->backing_size[FLEXE_MEM_SRAM]), 0);
}

TEST(test_cached_interpreter_mixed_batches_match_steps) {
    for (int accelerated = 0; accelerated <= 1; accelerated++) {
        xtensa_cpu_t cached, reference;
        pair_init(&cached, &reference);
        cached.accelerated_blocks = accelerated;
        cached.windowbase = 15; /* operands wrap around the physical AR file */
        ar_write(&cached, 1, 0x3FFB0200u);
        ar_write(&cached, 2, 7); ar_write(&cached, 3, 11);
        cached.fr[1] = 1.25f; cached.fr[2] = 2.0f;
        uint32_t pc = BASE;
#define W(insn) do { put_insn3(&cached, pc, (insn)); pc += 3; } while (0)
#define N(insn) do { put_insn2(&cached, pc, (insn)); pc += 2; } while (0)
        for (int repeat = 0; repeat < 12; repeat++) {
            N(narrow(10, 4, 2, 3));
            N(narrow(11, 4, 4, 0));
            W(rrr(9, 0, 5, 2, 3));
            W(rrr(3, 0, 6, 4, 5));
            W(rrr(6, 0, 7, 0, 6));
            W(rrr(4, 0, 0, 5, 0)); /* SSR */
            W(rrr(8, 1, 8, 3, 2)); /* SRC */
            W(rrr(8, 2, 9, 2, 3)); /* MULL */
            W(rrr(8, 3, 10, 9, 2)); /* MOVEQZ */
            W(rrr(5, 4, 11, 2, 5)); /* EXTUI */
            W(0x004142u); /* S8I a4,a1,0 */
            W(0x005152u); /* S16I a5,a1,0 */
            N(narrow(9, 2, 1, 4));
            W(0x006162u); /* S32I a6,a1,0 */
            W(0x002172u); /* L32I a7,a1,0 */
            W(rrr(0, 10, 3, 1, 2));
            W(rrr(2, 10, 4, 3, 2));
            W(rrr(15, 10, 5, 4, 6)); /* NEG.S */
            W(rrr(9, 10, 12, 4, 0)); /* TRUNC.S */
            W(rrr(12, 10, 6, 12, 1)); /* FLOAT.S */
            W(rrr(2, 11, 0, 1, 2)); /* OEQ.S */
            W(0x004143u); /* SSI f4,a1,0 */
            W(0x000173u); /* LSI f7,a1,0 */
            W(rrr(0, 3, XT_SR_CCOUNT >> 4, XT_SR_CCOUNT & 15, 13));
            N(narrow(13, 0, 13, 14));
            N(narrow(13, 15, 0, 3));
        }
        N(narrow(13, 15, 0, 2));
        pair_build(&cached, &reference);
        static const int budgets[] = {1, 2, 3, 7, 19, 64, 512};
        for (unsigned i = 0; i < sizeof(budgets) / sizeof(budgets[0]); i++)
            pair_compare(&cached, &reference, budgets[i]);
        ASSERT_TRUE(cached.debug_break);
        pair_destroy(&cached, &reference);
#undef W
#undef N
    }
}

TEST(test_cached_interpreter_window_calls_and_faults_match_steps) {
    for (unsigned callinc = 1; callinc <= 3; callinc++) {
        for (unsigned faults = 0; faults < 4; faults++) {
            unsigned collision = faults & 1u;
            bool underflow = (faults & 2u) != 0;
            xtensa_cpu_t cached, reference;
            pair_init(&cached, &reference);
            cached.ps = 1u << 18;
            cached.real_window_vectors = true;
            cached.vecbase = BASE + 0x2000u;
            seed_canonical_window_vectors(&cached, cached.vecbase);
            cached.windowbase = 15;
            cached.windowstart = (1u << 15) | (collision ? 1u << ((15u + callinc) & 15u) : 0u);
            ar_write(&cached, 1, 0x3FFB4000u);
            for (int i = 4; i < 16; i++) ar_write(&cached, i, 0x3FFB4400u + (uint32_t)i * 16u);
            put_insn3(&cached, BASE, encode_test_calln(BASE, BASE + 0x100u, callinc));
            put_insn2(&cached, BASE + 3u, narrow(11, 2, 2, 1));
            put_insn2(&cached, BASE + 5u, narrow(13, 15, 0, 2));
            put_insn3(&cached, BASE + 0x100u, 0x004136u); /* ENTRY a1,32 */
            put_insn2(&cached, BASE + 0x103u, narrow(11, 2, 2, 1));
            if (underflow) {
                unsigned callee_wb = (15u + callinc) & 15u;
                put_insn2(&cached, BASE + 0x105u, narrow(12, 1u << callee_wb, 3, 0));
                put_insn3(&cached, BASE + 0x107u,
                          rrr(1, 3, XT_SR_WINDOWSTART >> 4, XT_SR_WINDOWSTART & 15, 3));
                put_insn2(&cached, BASE + 0x10Au, narrow(13, 15, 0, 1));
                /* Canonical ABI base save area and the caller's extra-area
                 * link. Clearing its WS bit forces the guest underflow
                 * handler to reload the caller before retrying RETW. */
                mem_write32(cached.mem, 0x3FFB3FD0u, 0);
                mem_write32(cached.mem, 0x3FFB3FD4u, 0x3FFB4000u);
                mem_write32(cached.mem, 0x3FFB3FD8u, 123);
                mem_write32(cached.mem, 0x3FFB3FDCu, 456);
                mem_write32(cached.mem, 0x3FFB3FF4u, 0x3FFB4500u);
            } else put_insn2(&cached, BASE + 0x105u, narrow(13, 15, 0, 1));
            pair_build(&cached, &reference);
            for (int batch = 0; batch < 20 && !cached.debug_break && !cached.exception; batch++)
                pair_compare(&cached, &reference, 7);
            ASSERT_TRUE(cached.debug_break);
            ASSERT_EQ(cached.windowbase, 15);
            pair_destroy(&cached, &reference);
        }
    }
}

TEST(test_cached_interpreter_loops_and_ccount_match_steps) {
    xtensa_cpu_t cached, reference;
    pair_init(&cached, &reference);
    ar_write(&cached, 2, 9);
    put_insn3(&cached, BASE, (6u << 16) | (8u << 12) | (2u << 8) | 0x76u); /* LOOP a2,BASE+10 */
    put_insn2(&cached, BASE + 3u, narrow(11, 3, 3, 1));
    put_insn3(&cached, BASE + 5u, rrr(0, 3, 0, XT_SR_LCOUNT, 4));
    put_insn2(&cached, BASE + 8u, narrow(13, 15, 0, 3));
    put_insn2(&cached, BASE + 10u, narrow(13, 15, 0, 2));
    pair_build(&cached, &reference);
    pair_compare(&cached, &reference, 7);
    pair_compare(&cached, &reference, 64);
    ASSERT_EQ(ar_read(&cached, 3), 9);
    ASSERT_TRUE(cached.debug_break);
    pair_destroy(&cached, &reference);
}

TEST(test_cached_interpreter_breakpoint_inside_batch) {
    xtensa_cpu_t cached, reference;
    pair_init(&cached, &reference);
    for (unsigned i = 0; i < 10; i++) put_insn2(&cached, BASE + i * 2u, narrow(11, 2, 2, 1));
    xtensa_set_breakpoint(&cached, BASE + 14u);
    xtensa_set_breakpoint(&cached, BASE + 8u); /* exercise a non-first breakpoint */
    pair_build(&cached, &reference);
    pair_compare(&cached, &reference, 20);
    ASSERT_EQ(cached.pc, BASE + 8u);
    ASSERT_EQ(cached.ccount, 4);
    ASSERT_EQ(ar_read(&cached, 2), 4);
    ASSERT_TRUE(cached.breakpoint_hit);
    xtensa_clear_all_breakpoints(&cached);
    xtensa_clear_all_breakpoints(&reference);
    pair_compare(&cached, &reference, 5);
    ASSERT_EQ(cached.pc, BASE + 18u);
    pair_destroy(&cached, &reference);
}

TEST(test_cached_interpreter_ps_writes_and_timers_match_steps) {
    for (unsigned rsil = 0; rsil < 2; rsil++) {
        for (unsigned pending = 0; pending < 3; pending++) {
            xtensa_cpu_t cached, reference;
            pair_init(&cached, &reference);
            cached.ps = 3;
            cached.interrupt = pending ? 1u << 6 : 0;
            cached.intenable = pending == 2 ? 1u << 6 : 0;
            ar_write(&cached, 6, 0);
            for (unsigned i = 0; i < 20; i++) put_insn2(&cached, BASE + i * 2u, narrow(11, 2, 2, 1));
            put_insn3(&cached, BASE + 4u, rsil ? rrr(0, 0, 6, 0, 6) : rrr(1, 3, XT_SR_PS >> 4, XT_SR_PS & 15, 6));
            cached.vecbase = BASE + 0x2000u;
            for (unsigned i = 0; i < 20; i++) put_insn2(&cached, cached.vecbase + VECOFS_KERNEL_EXC + i * 2u, narrow(11, 3, 3, 1));
            cached.ccount = 100;
            cached.ccompare[0] = 107;
            xtensa_recompute_next_timer(&cached);
            pair_build(&cached, &reference);
            pair_compare(&cached, &reference, 7);
            pair_compare(&cached, &reference, 2);
            pair_destroy(&cached, &reference);
        }
    }
}

typedef struct { xtensa_cpu_t *cpu; unsigned reads; } cached_mmio_t;
static uint32_t cached_mmio_read(void *opaque, uint32_t addr) {
    (void)addr;
    cached_mmio_t *ctx = opaque;
    ctx->reads++;
    ctx->cpu->interrupt |= 1u << 6;
    xtensa_request_irq_check(ctx->cpu);
    return ctx->cpu->ccount;
}

TEST(test_cached_interpreter_mmio_irq_boundary_matches_steps) {
    xtensa_cpu_t cached, reference;
    pair_init(&cached, &reference);
    ar_write(&cached, 1, 0x3FF00000u);
    cached.intenable = 1u << 6;
    cached.vecbase = BASE + 0x2000u;
    put_insn2(&cached, BASE, narrow(11, 2, 2, 1));
    put_insn2(&cached, BASE + 2u, narrow(11, 2, 2, 1));
    put_insn3(&cached, BASE + 4u, 0x002142u); /* L32I a4,a1,0 */
    put_insn2(&cached, cached.vecbase + VECOFS_KERNEL_EXC, narrow(13, 15, 0, 2));
    pair_build(&cached, &reference);
    cached_mmio_t left = {&cached, 0}, right = {&reference, 0};
    ASSERT_EQ(mem_register_mmio(cached.mem, 0, cached_mmio_read, NULL, &left), 0);
    ASSERT_EQ(mem_register_mmio(reference.mem, 0, cached_mmio_read, NULL, &right), 0);
    pair_compare(&cached, &reference, 10);
    ASSERT_EQ(left.reads, 1); ASSERT_EQ(right.reads, 1);
    ASSERT_EQ(ar_read(&cached, 4), 2);
    ASSERT_TRUE(cached.debug_break);
    pair_destroy(&cached, &reference);
}

TEST(test_cached_interpreter_cross_page_memory_matches_steps) {
    xtensa_cpu_t cached, reference;
    pair_init(&cached, &reference);
    ar_write(&cached, 1, 0x3FFB0FFFu);
    ar_write(&cached, 2, 0xAABBCCDDu);
    put_insn2(&cached, BASE, narrow(9, 0, 1, 2));
    put_insn2(&cached, BASE + 2u, narrow(8, 0, 1, 3));
    put_insn2(&cached, BASE + 4u, narrow(13, 15, 0, 2));
    pair_build(&cached, &reference);
    pair_compare(&cached, &reference, 10);
    ASSERT_EQ(ar_read(&cached, 3), 0xAABBCCDDu);
    pair_destroy(&cached, &reference);
}

TEST(test_cached_interpreter_predecode_metadata_matches_raw_decode) {
    xtensa_cpu_t cpu;
    setup(&cpu);
    uint32_t random = 0x4B19A003u;
    for (unsigned i = 0; i < 4096; i++) {
        random = random * 1664525u + 1013904223u;
        mem_write8(cpu.mem, BASE + i, (uint8_t)(random >> 24));
    }
    xtensa_predecode_build(&cpu);
#if PREDECODE_SIZE > 0
    for (unsigned i = 0; i < 4093; i++) {
        uint32_t insn;
        int length = xtensa_fetch(&cpu, BASE + i, &insn);
        uint32_t packed = cpu.predecode[BASE + i - PREDECODE_BASE];
        ASSERT_EQ(PREDECODE_ILEN(packed), length);
        ASSERT_EQ(PREDECODE_INSN(packed), insn);
        bool entry = length == 3 && XT_OP0(insn) == 6 && XT_N(insn) == 3 && XT_M(insn) == 0;
        ASSERT_EQ(PREDECODE_WINDOW_NEED(packed), entry ? 0 : xtensa_window_operand_need(&cpu, insn, length));
    }
#endif
    free(cpu.predecode);
    teardown(&cpu);
}

typedef struct { unsigned calls; uint32_t ccount; } cached_hook_t;
static int cached_observer_hook(xtensa_cpu_t *cpu, uint32_t pc, void *opaque) {
    (void)pc;
    cached_hook_t *ctx = opaque;
    ctx->calls++;
    ctx->ccount = cpu->ccount;
    return 0;
}

TEST(test_cached_interpreter_control_flow_hook_boundaries) {
    xtensa_cpu_t cached, reference;
    pair_init(&cached, &reference);
    put_insn3(&cached, BASE, encode_test_calln(BASE, BASE + 0x100u, 0));
    put_insn2(&cached, BASE + 3u, narrow(13, 15, 0, 2));
    put_insn2(&cached, BASE + 0x100u, narrow(11, 2, 2, 1));
    put_insn2(&cached, BASE + 0x102u, narrow(13, 15, 0, 0));
    cached.pc_hook = cached_observer_hook;
    pair_build(&cached, &reference);
    cached_hook_t left = {0}, right = {0};
    cached.pc_hook_ctx = &left; reference.pc_hook_ctx = &right;
    pair_compare(&cached, &reference, 10);
    ASSERT_EQ(left.calls, 3); ASSERT_EQ(right.calls, 3);
    ASSERT_EQ(left.ccount, 3); ASSERT_EQ(right.ccount, 3);
    pair_destroy(&cached, &reference);
}

TEST(test_cached_interpreter_journal_preserves_stores) {
    xtensa_cpu_t cached, reference;
    pair_init(&cached, &reference);
    ar_write(&cached, 1, 0x3FFB0200u);
    ar_write(&cached, 2, 0xAABBCCDDu);
    mem_write32(cached.mem, ar_read(&cached, 1), 0x12345678u);
    put_insn2(&cached, BASE, narrow(11, 2, 2, 1));
    put_insn2(&cached, BASE + 2u, narrow(9, 0, 1, 2));
    put_insn2(&cached, BASE + 4u, narrow(13, 15, 0, 2));
    pair_build(&cached, &reference);
    mem_journal_begin();
    ASSERT_EQ(xtensa_run(&cached, 10), 3);
    ASSERT_EQ(g_mem_journal_count, 1);
    ASSERT_EQ(g_mem_journal[0].old, 0x12345678u);
    mem_journal_rollback(cached.mem);
    mem_journal_end();
    mem_journal_begin();
    ASSERT_EQ(reference_run(&reference, 10), 3);
    ASSERT_EQ(g_mem_journal_count, 1);
    mem_journal_rollback(reference.mem);
    mem_journal_end();
    ASSERT_EQ(mem_read32(cached.mem, 0x3FFB0200u), 0x12345678u);
    pair_compare(&cached, &reference, 1);
    pair_destroy(&cached, &reference);
}

void run_cached_interpreter_tests(void) {
    TEST_SUITE("Cached Interpreter");
    RUN_TEST(test_cached_interpreter_mixed_batches_match_steps);
    RUN_TEST(test_cached_interpreter_window_calls_and_faults_match_steps);
    RUN_TEST(test_cached_interpreter_loops_and_ccount_match_steps);
    RUN_TEST(test_cached_interpreter_breakpoint_inside_batch);
    RUN_TEST(test_cached_interpreter_ps_writes_and_timers_match_steps);
    RUN_TEST(test_cached_interpreter_mmio_irq_boundary_matches_steps);
    RUN_TEST(test_cached_interpreter_cross_page_memory_matches_steps);
    RUN_TEST(test_cached_interpreter_predecode_metadata_matches_raw_decode);
    RUN_TEST(test_cached_interpreter_control_flow_hook_boundaries);
    RUN_TEST(test_cached_interpreter_journal_preserves_stores);
}
