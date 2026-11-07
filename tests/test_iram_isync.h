/* Synthetic executable-RAM regression; no firmware or external assets.
 * Included by test_integration.c. The same test compiles on the old API:
 * only observer registration is conditional; the expected guest result is not.
 */
#include "jit.h"
#include <stdlib.h>

#define IRAM_CODE 0x40080000u
#define IRAM_WRITER 0x40080100u
#define IRAM_RETURN 0x40080200u
#define IRAM_SYNC 0x40080300u

typedef struct {
    uint8_t shadow[8];
    unsigned callbacks;
    unsigned invalidations;
} iram_coherence_t;

#ifdef XTENSA_HAS_INSTRUCTION_SYNC_OBSERVER
static void iram_sync_observer(xtensa_cpu_t *cpu, void *ctx) {
    iram_coherence_t *state = ctx;
    const uint8_t *current = mem_get_ptr(cpu->mem, IRAM_CODE);
    state->callbacks++;
    if (memcmp(current, state->shadow, sizeof state->shadow) != 0) {
        memcpy(state->shadow, current, sizeof state->shadow);
        xtensa_invalidate_code(cpu, IRAM_CODE, sizeof state->shadow);
        state->invalidations++;
    }
}
#endif

static void iram_observe(xtensa_cpu_t *cpu, iram_coherence_t *state) {
#ifdef XTENSA_HAS_INSTRUCTION_SYNC_OBSERVER
    cpu->instruction_sync = iram_sync_observer;
    cpu->instruction_sync_ctx = state;
#else
    (void)cpu;
    (void)state;
#endif
}

static void iram_cached_sync(xtensa_cpu_t *cpu) {
    xtensa_pc_hook_fn hook = cpu->pc_hook;
    void *ctx = cpu->pc_hook_ctx;
    const uint64_t *bitmap = cpu->pc_hook_bitmap;
    bool accelerated = cpu->accelerated_blocks;
    bool record = cpu->record_branch_targets;
    cpu->pc_hook = NULL;
    cpu->pc_hook_ctx = NULL;
    cpu->pc_hook_bitmap = NULL;
    cpu->record_branch_targets = false;
    cpu->accelerated_blocks = false;
    cpu->irq_check = false;
    cpu->next_timer_event = UINT32_MAX;
    cpu->pc = IRAM_SYNC;
    cpu->_pc_written = false;
    cpu->running = true;
    ar_write(cpu, 0, IRAM_RETURN);
    ASSERT_EQ(xtensa_run(cpu, 2), 2u);
    ASSERT_EQ(cpu->pc, IRAM_RETURN);
    cpu->pc_hook = hook;
    cpu->pc_hook_ctx = ctx;
    cpu->pc_hook_bitmap = bitmap;
    cpu->accelerated_blocks = accelerated;
    cpu->record_branch_targets = record;
}

#ifdef FLEXE_HAS_JIT
static jit_block_fn iram_compile(jit_state_t *jit, xtensa_cpu_t *cpu, uint32_t pc) {
    cpu->pc = pc;
    for (unsigned i = 0; i <= JIT_HOT_THRESHOLD; ++i)
        (void)jit_get_block(jit, cpu, pc);
    return jit_get_block(jit, cpu, pc);
}

static void iram_execute(jit_state_t *jit, xtensa_cpu_t *cpu, uint32_t pc) {
    ar_write(cpu, 0, IRAM_RETURN);
    jit_block_fn block = iram_compile(jit, cpu, pc);
    ASSERT_TRUE(block != NULL);
    if (block) {
        ASSERT_TRUE(block(cpu) >= 2);
        ASSERT_EQ(cpu->pc, IRAM_RETURN);
    }
}

TEST(iram_isync_replacement) {
    /* Each cell starts with a fresh JIT and executes 42 before replacing it.
     * Byte/halfword/word/copy stores and both guest store engines must all
     * return 43 after a real guest ISYNC. Cell 6 keeps a JIT store prefix
     * immediately before ISYNC, rather than invoking the callback from C.
     */
    for (unsigned mode = 0; mode < 7; ++mode) {
        xtensa_cpu_t cpu;
        xtensa_cpu_init(&cpu);
        cpu.mem = mem_create();
        jit_state_t *jit = jit_init();
        ASSERT_TRUE(cpu.mem != NULL && jit != NULL);
        if (!cpu.mem || !jit) {
            if (jit) jit_destroy(jit);
            if (cpu.mem) mem_destroy(cpu.mem);
            return;
        }
        jit_install_hook(jit, &cpu);
        uint8_t code[] = {0x22, 0xa0, 42, 0x3d, 0xf0, 0x0d, 0xf0, 0};
        const uint8_t barrier[] = {0x00, 0x20, 0x00, 0x0d, 0xf0};
        const uint8_t writer[] = {0x42, 0x63, 0x00, 0x3d, 0xf0,
                                   0x00, 0x20, 0x00, 0x0d, 0xf0};
        put_test_bytes(&cpu, IRAM_CODE, code, sizeof code);
        put_test_bytes(&cpu, IRAM_SYNC, barrier, sizeof barrier);
        put_test_bytes(&cpu, IRAM_WRITER, writer, sizeof writer);
        iram_coherence_t state = {0};
        memcpy(state.shadow, code, sizeof code);
        iram_observe(&cpu, &state);
        xtensa_predecode_build(&cpu);
        iram_execute(jit, &cpu, IRAM_CODE);
        ASSERT_EQ(ar_read(&cpu, 2), 42u);
        code[2] = 43;
        uint32_t word;
        memcpy(&word, code, sizeof word);
        if (mode == 0) mem_write8(cpu.mem, IRAM_CODE + 2, 43);
        else if (mode == 1) mem_write16(cpu.mem, IRAM_CODE + 2, 0x3d2b);
        else if (mode == 2) mem_write32(cpu.mem, IRAM_CODE, word);
        else if (mode == 3) memcpy(mem_get_ptr_w(cpu.mem, IRAM_CODE), code, sizeof code);
        else {
            ar_write(&cpu, 3, IRAM_CODE);
            ar_write(&cpu, 4, word);
            ar_write(&cpu, 0, IRAM_RETURN);
            cpu.pc = IRAM_WRITER;
            if (mode == 4) {
                jit_block_fn block = iram_compile(jit, &cpu, IRAM_WRITER);
                ASSERT_TRUE(block != NULL);
                if (block) ASSERT_EQ(block(&cpu), 2u);
                ASSERT_EQ(cpu.pc, IRAM_WRITER + 5u);
            } else if (mode == 5) {
                ASSERT_EQ(xtensa_step(&cpu), 0u);
            } else {
                ASSERT_TRUE(iram_compile(jit, &cpu, IRAM_WRITER) != NULL);
                uint64_t before = jit_get_stats(jit)->insns_jitted;
                cpu.running = true;
                cpu._pc_written = true;
                ASSERT_EQ(xtensa_run(&cpu, 4), 4u);
                ASSERT_EQ(cpu.pc, IRAM_RETURN);
                ASSERT_TRUE(jit_get_stats(jit)->insns_jitted > before);
            }
        }
        if (mode != 6) iram_cached_sync(&cpu);
        iram_execute(jit, &cpu, IRAM_CODE);
        ASSERT_EQ(ar_read(&cpu, 2), 43u);
#ifdef XTENSA_HAS_INSTRUCTION_SYNC_OBSERVER
        ASSERT_EQ(state.callbacks, 1u);
        ASSERT_EQ(state.invalidations, 1u);
#endif
        /* Explicit invalidation is an independent control on the same bytes. */
        xtensa_invalidate_code(&cpu, IRAM_CODE, sizeof code);
        iram_execute(jit, &cpu, IRAM_CODE);
        ASSERT_EQ(ar_read(&cpu, 2), 43u);
        ASSERT_FALSE(cpu.exception);
        jit_destroy(jit);
        free(cpu.predecode);
        mem_destroy(cpu.mem);
    }
}
#endif

TEST(iram_isync_first_load) {
    xtensa_cpu_t cpu;
    xtensa_cpu_init(&cpu);
    cpu.mem = mem_create();
    ASSERT_TRUE(cpu.mem != NULL);
    if (!cpu.mem) return;
    iram_coherence_t state = {0};
    iram_observe(&cpu, &state);
    const uint8_t barrier[] = {0x00, 0x20, 0x00, 0x0d, 0xf0};
    const uint8_t code[] = {0x22, 0xa0, 42, 0x3d, 0xf0, 0x0d, 0xf0, 0};
    put_test_bytes(&cpu, IRAM_SYNC, barrier, sizeof barrier);
    xtensa_predecode_build(&cpu);
    ASSERT_TRUE(cpu.predecode[IRAM_CODE - PREDECODE_BASE] != 0u);
    memcpy(mem_get_ptr_w(cpu.mem, IRAM_CODE), code, sizeof code);
    iram_cached_sync(&cpu);
    cpu.pc = IRAM_CODE;
    cpu._pc_written = false;
    ASSERT_EQ(xtensa_run(&cpu, 3), 3u);
    ASSERT_EQ(ar_read(&cpu, 2), 42u);
    ASSERT_FALSE(cpu.exception);
    free(cpu.predecode);
    mem_destroy(cpu.mem);
}

TEST(iram_isync_barrier_routing) {
    xtensa_cpu_t cpu;
    xtensa_cpu_init(&cpu);
    cpu.mem = mem_create();
    ASSERT_TRUE(cpu.mem != NULL);
    if (!cpu.mem) return;
    iram_coherence_t state = {0};
    iram_observe(&cpu, &state);
    /* Canonical stepping and quick decoding must observe ISYNC, but not MEMW. */
    put_insn3(&cpu, IRAM_SYNC, 0x002000u); /* ISYNC */
    cpu.pc = IRAM_SYNC;
    ASSERT_EQ(xtensa_step(&cpu), 0u);
    put_insn3(&cpu, IRAM_SYNC, 0x0020c0u); /* MEMW */
    cpu.pc = IRAM_SYNC;
    ASSERT_EQ(xtensa_step(&cpu), 0u);
#ifdef XTENSA_HAS_INSTRUCTION_SYNC_OBSERVER
    ASSERT_EQ(state.callbacks, 1u);
    cpu.instruction_sync = NULL;
#endif
    put_insn3(&cpu, IRAM_SYNC, 0x002000u);
    cpu.pc = IRAM_SYNC;
    ASSERT_EQ(xtensa_step(&cpu), 0u); /* Null observer remains valid. */
    ASSERT_FALSE(cpu.exception);
    mem_destroy(cpu.mem);
}
