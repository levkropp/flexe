/* Translated-code coherence: guest stores that rewrite RAM holding code must
 * be seen by the next execution, through the predecode table, the cached
 * interpreter and the JIT, and through every guest alias of the bytes. Stores
 * that change nothing translated must not invalidate anything. */
#include "test_helpers.h"
#include "target.h"
#ifdef FLEXE_HAS_JIT
#include "jit.h"
#endif
#include <stdlib.h>
#include <string.h>

/* Classic ESP32 IRAM layout for the engine tests. */
#define CC_CODE    0x40080200u   /* answer(): a2 = 42; return */
#define CC_RET     0x40080600u   /* spin here after returning */
#define CC_WRITER  0x40081000u   /* store a4 into the answer, then return */

/* movi a2, 42 ; nop.n ; ret.n */
static const uint8_t cc_answer42[] = {0x22, 0xA0, 42, 0x3D, 0xF0, 0x0D, 0xF0};
/* j . */
static const uint8_t cc_spin[] = {0x06, 0xFF, 0xFF};

/* The answer's immediate is byte 2; its first word with 43 instead of 42. */
#define CC_WORD43  0x3D2BA022u
#define CC_HALF43  0x3D2Bu

typedef enum {
    CC_HOST_BYTE, CC_HOST_HALF, CC_HOST_WORD,
    CC_GUEST_S8I, CC_GUEST_S16I, CC_GUEST_S32I,
    CC_WRITER_COUNT,
} cc_writer_t;

static const char *cc_writer_name[CC_WRITER_COUNT] = {
    "host byte", "host half", "host word", "s8i", "s16i", "s32i",
};

/* j target (CALL format, n = 0) at pc. */
static uint32_t cc_encode_j(uint32_t pc, uint32_t target) {
    return 6u | (((target - pc - 4u) & 0x3FFFFu) << 6);
}

static void cc_put_program(xtensa_cpu_t *cpu, uint32_t code, uint32_t ret) {
    put_test_bytes(cpu, code, cc_answer42, sizeof cc_answer42);
    put_test_bytes(cpu, ret, cc_spin, sizeof cc_spin);
}

/* Writer: one store of a4 at [a3 + offset], then ret.n. */
static void cc_put_writer(xtensa_cpu_t *cpu, cc_writer_t kind) {
    uint8_t store[3];
    switch (kind) {
    case CC_GUEST_S8I:  store[0] = 0x42; store[1] = 0x43; store[2] = 2; break;
    case CC_GUEST_S16I: store[0] = 0x42; store[1] = 0x53; store[2] = 1; break;
    default:            store[0] = 0x42; store[1] = 0x63; store[2] = 0; break;
    }
    static const uint8_t ret_n[] = {0x0D, 0xF0};
    put_test_bytes(cpu, CC_WRITER, store, sizeof store);
    put_test_bytes(cpu, CC_WRITER + 3u, ret_n, sizeof ret_n);
}

static uint32_t cc_store_value(cc_writer_t kind) {
    return kind == CC_GUEST_S8I ? 43u :
           kind == CC_GUEST_S16I ? CC_HALF43 : CC_WORD43;
}

/* Call fn with a0 = CC_RET and run until it spins there. */
static void cc_call(xtensa_cpu_t *cpu, uint32_t fn, bool stepped,
                    void *jit_ctx) {
    cpu->pc = fn;
    cpu->_pc_written = false;
    cpu->running = true;
    ar_write(cpu, 0, CC_RET);
    ar_write(cpu, 2, 0);
#ifdef FLEXE_HAS_JIT
    if (jit_ctx) {
        for (int i = 0; i < 8 && cpu->pc != CC_RET; i++)
            (void)jit_run((jit_state_t *)jit_ctx, cpu, 16);
        ASSERT_EQ(cpu->pc, CC_RET);
        return;
    }
#else
    (void)jit_ctx;
#endif
    if (stepped) {
        for (int i = 0; i < 8 && cpu->pc != CC_RET; i++)
            (void)xtensa_step(cpu);
    } else {
        (void)xtensa_run(cpu, 16);
    }
    ASSERT_EQ(cpu->pc, CC_RET);
}

/* Rewrite the answer's immediate to 43 with the given kind of store. Guest
 * writers run in the same interpreter mode as the check. */
static void cc_rewrite(xtensa_cpu_t *cpu, uint32_t code, cc_writer_t kind,
                       bool stepped) {
    switch (kind) {
    case CC_HOST_BYTE: mem_write8(cpu->mem, code + 2u, 43); return;
    case CC_HOST_HALF: mem_write16(cpu->mem, code + 2u, CC_HALF43); return;
    case CC_HOST_WORD: mem_write32(cpu->mem, code, CC_WORD43); return;
    default: break;
    }
    cc_put_writer(cpu, kind);
    ar_write(cpu, 3, code);
    ar_write(cpu, 4, cc_store_value(kind));
    cc_call(cpu, CC_WRITER, stepped, NULL);
}

/* ===== Memory-level contract ===== */

typedef struct {
    unsigned calls;
    uint32_t exec_addr;
    uint32_t len;
    unsigned kinds;
} cc_watch_log_t;

static void cc_watch(void *ctx, uint32_t exec_addr, uint32_t len,
                     unsigned kinds) {
    cc_watch_log_t *log = ctx;
    log->calls++;
    log->exec_addr = exec_addr;
    log->len = len;
    log->kinds = kinds;
}

TEST(code_coherence_diverts_only_pages_holding_translated_code) {
    xtensa_mem_t *mem = mem_create();
    ASSERT_TRUE(mem != NULL);
    if (!mem) return;
    cc_watch_log_t log = {0};
    ASSERT_EQ(mem_code_watch_add(mem, cc_watch, &log), 0);

    ASSERT_TRUE(mem->write_page_table[CC_CODE >> 12] != NULL);
    mem_code_protect(mem, CC_CODE, sizeof cc_answer42, MEM_CODE_JIT);
    ASSERT_TRUE(mem->page_table[CC_CODE >> 12] != NULL);
    ASSERT_TRUE(mem->write_page_table[CC_CODE >> 12] == NULL);
    ASSERT_TRUE(mem->write_page_table[CC_WRITER >> 12] != NULL);

    /* Flash and ROM are not writable RAM and stay on the fast path. */
    uint8_t *flash_page = mem->write_page_table[0x400D0000u >> 12];
    mem_code_protect(mem, 0x400D0000u, 64u, MEM_CODE_JIT);
    ASSERT_TRUE(mem->write_page_table[0x400D0000u >> 12] == flash_page);

    /* Rewriting identical bytes (RAM starts zeroed) reports nothing. */
    mem_write8(mem, CC_CODE + 2u, 0);
    mem_write32(mem, CC_CODE, 0);
    ASSERT_EQ(log.calls, 0u);

    /* A changed byte in a marked chunk is reported at its exec address. */
    mem_write8(mem, CC_CODE + 2u, 43);
    ASSERT_EQ(log.calls, 1u);
    ASSERT_EQ(log.exec_addr, CC_CODE + 2u);
    ASSERT_EQ(log.len, 1u);
    ASSERT_EQ(log.kinds, (unsigned)MEM_CODE_JIT);
    ASSERT_EQ(mem_read8(mem, CC_CODE + 2u), 43u);

    /* A changed word elsewhere on the diverted page is stored, unreported. */
    mem_write32(mem, CC_CODE + 0x400u, 0xA5A5A5A5u);
    ASSERT_EQ(log.calls, 1u);
    ASSERT_EQ(mem_read32(mem, CC_CODE + 0x400u), 0xA5A5A5A5u);

    /* Clearing the kind's marks silences further changes to those bytes. */
    mem_code_clear(mem, MEM_CODE_JIT);
    mem_write8(mem, CC_CODE + 2u, 44);
    ASSERT_EQ(log.calls, 1u);
    ASSERT_EQ(mem_read8(mem, CC_CODE + 2u), 44u);

    mem_code_watch_remove(mem, cc_watch, &log);
    mem_destroy(mem);
}

TEST(code_coherence_s3_data_alias_reports_instruction_address) {
    const flexe_target_desc_t *s3 = flexe_target_by_id(FLEXE_TARGET_ESP32S3);
    xtensa_mem_t *mem = mem_create_for_target(s3);
    ASSERT_TRUE(mem != NULL);
    if (!mem) return;
    const uint32_t iram = 0x40378100u, dram = 0x3FC88100u;
    ASSERT_TRUE(mem_get_ptr(mem, iram) == mem_get_ptr(mem, dram));

    cc_watch_log_t log = {0};
    ASSERT_EQ(mem_code_watch_add(mem, cc_watch, &log), 0);
    mem_code_protect(mem, iram, 8u, MEM_CODE_JIT);
    ASSERT_TRUE(mem->write_page_table[iram >> 12] == NULL);
    ASSERT_TRUE(mem->write_page_table[dram >> 12] == NULL);

    mem_write8(mem, dram + 2u, 43);
    ASSERT_EQ(log.calls, 1u);
    ASSERT_EQ(log.exec_addr, iram + 2u);
    ASSERT_EQ(mem_read8(mem, iram + 2u), 43u);

    /* A page mapped later onto the same bytes is diverted too. */
    ASSERT_EQ(mem_map_backing_range(mem, 0x20000000u, FLEXE_MEM_SRAM,
                                    0x00000000u, 0x1000u), 0);
    ASSERT_TRUE(mem->page_table[0x20000000u >> 12] != NULL);
    ASSERT_TRUE(mem->write_page_table[0x20000000u >> 12] == NULL);
    mem_write8(mem, 0x20000103u, 0x11);
    ASSERT_EQ(log.calls, 2u);
    ASSERT_EQ(log.exec_addr, iram + 3u);
    ASSERT_EQ(mem_unmap_range(mem, 0x20000000u, 0x1000u), 0);
    ASSERT_TRUE(mem->write_page_table[0x20000000u >> 12] == NULL);

    mem_code_watch_remove(mem, cc_watch, &log);
    mem_destroy(mem);
}

/* ===== Interpreter and predecode ===== */

TEST(code_coherence_interpreter_runs_rewritten_iram) {
    for (int kind = 0; kind < CC_WRITER_COUNT; kind++) {
        for (int stepped = 0; stepped < 2; stepped++) {
            xtensa_cpu_t cpu;
            xtensa_cpu_init(&cpu);
            cpu.mem = mem_create();
            ASSERT_TRUE(cpu.mem != NULL);
            if (!cpu.mem) return;
            cc_put_program(&cpu, CC_CODE, CC_RET);
            cc_put_writer(&cpu, (cc_writer_t)kind);
            xtensa_predecode_build(&cpu);

            cc_call(&cpu, CC_CODE, stepped, NULL);
            ASSERT_EQ(ar_read(&cpu, 2), 42u);
            cc_rewrite(&cpu, CC_CODE, (cc_writer_t)kind, stepped);
            cc_call(&cpu, CC_CODE, stepped, NULL);
            if (ar_read(&cpu, 2) != 43u)
                fprintf(stderr, "  writer %s, %s interpreter\n",
                        cc_writer_name[kind], stepped ? "stepped" : "cached");
            ASSERT_EQ(ar_read(&cpu, 2), 43u);

            xtensa_code_watch_detach(&cpu);
            free(cpu.predecode);
            mem_destroy(cpu.mem);
        }
    }
}

#ifdef FLEXE_HAS_JIT
/* ===== JIT ===== */

typedef struct {
    xtensa_cpu_t cpu;
    jit_state_t *jit;
} cc_jit_fixture_t;

static bool cc_jit_init(cc_jit_fixture_t *f, const flexe_target_desc_t *target,
                        bool predecode) {
    memset(f, 0, sizeof(*f));
    if (target) xtensa_cpu_init_for_target(&f->cpu, target);
    else xtensa_cpu_init(&f->cpu);
    f->cpu.mem = target ? mem_create_for_target(target) : mem_create();
    f->jit = jit_init();
    ASSERT_TRUE(f->cpu.mem != NULL && f->jit != NULL);
    if (!f->cpu.mem || !f->jit) {
        if (f->jit) jit_destroy(f->jit);
        if (f->cpu.mem) mem_destroy(f->cpu.mem);
        return false;
    }
    if (predecode) xtensa_predecode_build(&f->cpu);
    jit_install_hook(f->jit, &f->cpu);
    return true;
}

static void cc_jit_destroy(cc_jit_fixture_t *f) {
    xtensa_code_watch_detach(&f->cpu);
    jit_destroy(f->jit);
    free(f->cpu.predecode);
    mem_destroy(f->cpu.mem);
}

static void cc_jit_compile(cc_jit_fixture_t *f, uint32_t pc) {
    f->cpu.pc = pc;
    for (int i = 0; i <= JIT_HOT_THRESHOLD; i++)
        (void)jit_get_block(f->jit, &f->cpu, pc);
    ASSERT_TRUE(jit_get_block(f->jit, &f->cpu, pc) != NULL);
}

TEST(code_coherence_jit_retranslates_rewritten_iram) {
    /* Writers: host stores, an interpreted guest store, and guest stores
     * from a compiled block (the inline store path). */
    for (int kind = 0; kind < CC_WRITER_COUNT; kind++) {
        for (int compiled_writer = 0; compiled_writer < 2; compiled_writer++) {
            if (compiled_writer && kind < CC_GUEST_S8I) continue;
            cc_jit_fixture_t f;
            if (!cc_jit_init(&f, NULL, true)) return;
            cc_put_program(&f.cpu, CC_CODE, CC_RET);
            cc_put_writer(&f.cpu, (cc_writer_t)kind);
            /* Blocks dispatch at control-flow targets: reach the compiled
             * writer through a jump so its inline store really runs. */
            const uint32_t tramp = CC_WRITER + 0x100u;
            put_insn3(&f.cpu, tramp, cc_encode_j(tramp, CC_WRITER));
            cc_jit_compile(&f, CC_CODE);
            if (compiled_writer) cc_jit_compile(&f, CC_WRITER);

            cc_call(&f.cpu, CC_CODE, false, f.jit);
            ASSERT_EQ(ar_read(&f.cpu, 2), 42u);
            uint64_t flushes = jit_get_stats(f.jit)->cache_flushes;

            if (kind >= CC_GUEST_S8I) {
                ar_write(&f.cpu, 3, CC_CODE);
                ar_write(&f.cpu, 4, cc_store_value((cc_writer_t)kind));
                uint64_t native = jit_get_stats(f.jit)->insns_jitted;
                if (compiled_writer) {
                    cc_call(&f.cpu, tramp, false, f.jit);
                    ASSERT_TRUE(jit_get_stats(f.jit)->insns_jitted > native);
                } else {
                    cc_call(&f.cpu, CC_WRITER, true, NULL);
                }
            } else {
                cc_rewrite(&f.cpu, CC_CODE, (cc_writer_t)kind, true);
            }
            ASSERT_TRUE(jit_get_stats(f.jit)->cache_flushes > flushes);

            cc_call(&f.cpu, CC_CODE, false, f.jit);
            if (ar_read(&f.cpu, 2) != 43u)
                fprintf(stderr, "  writer %s%s\n", cc_writer_name[kind],
                        compiled_writer ? " (compiled)" : "");
            ASSERT_EQ(ar_read(&f.cpu, 2), 43u);
            cc_jit_destroy(&f);
        }
    }
}

TEST(code_coherence_jit_keeps_blocks_for_unrelated_stores) {
    cc_jit_fixture_t f;
    if (!cc_jit_init(&f, NULL, true)) return;
    cc_put_program(&f.cpu, CC_CODE, CC_RET);
    cc_jit_compile(&f, CC_CODE);
    cc_call(&f.cpu, CC_CODE, false, f.jit);
    ASSERT_EQ(ar_read(&f.cpu, 2), 42u);
    uint64_t flushes = jit_get_stats(f.jit)->cache_flushes;

    /* Same bytes, and other chunks of the code's own page. */
    mem_write8(f.cpu.mem, CC_CODE + 2u, 42);
    mem_write32(f.cpu.mem, CC_CODE + 0x100u, 0x12345678u);
    mem_write32(f.cpu.mem, CC_CODE - 0x100u, 0x9ABCDEF0u);
    ASSERT_EQ(jit_get_stats(f.jit)->cache_flushes, flushes);
    ASSERT_EQ(mem_read32(f.cpu.mem, CC_CODE + 0x100u), 0x12345678u);

    cc_call(&f.cpu, CC_CODE, false, f.jit);
    ASSERT_EQ(ar_read(&f.cpu, 2), 42u);
    ASSERT_EQ(jit_get_stats(f.jit)->cache_flushes, flushes);
    cc_jit_destroy(&f);
}

/* A compiled block rewrites the answer and jumps straight to it. The answer
 * was already compiled and chained, so the stale native successor must not
 * run after the store. */
TEST(code_coherence_jit_store_stops_stale_chained_successor) {
    cc_jit_fixture_t f;
    if (!cc_jit_init(&f, NULL, true)) return;
    cc_put_program(&f.cpu, CC_CODE, CC_RET);
    /* s32i a4, a3, 0 ; j CC_CODE */
    const uint8_t store[] = {0x42, 0x63, 0x00};
    put_test_bytes(&f.cpu, CC_WRITER, store, sizeof store);
    put_insn3(&f.cpu, CC_WRITER + 3u, cc_encode_j(CC_WRITER + 3u, CC_CODE));
    /* Blocks are dispatched at control-flow targets, so enter the writer
     * through a jump: it then runs natively and chains into the answer. */
    const uint32_t tramp = CC_WRITER + 0x100u;
    put_insn3(&f.cpu, tramp, cc_encode_j(tramp, CC_WRITER));

    cc_jit_compile(&f, CC_CODE);
    cc_jit_compile(&f, CC_WRITER);
    /* Warm the chain with an identical store. */
    for (int i = 0; i < 3; i++) {
        ar_write(&f.cpu, 3, CC_CODE);
        ar_write(&f.cpu, 4, mem_read32(f.cpu.mem, CC_CODE));
        cc_call(&f.cpu, tramp, false, f.jit);
        ASSERT_EQ(ar_read(&f.cpu, 2), 42u);
    }
    const jit_stats_t *st = jit_get_stats(f.jit);
    ASSERT_TRUE(st->chains_patched >= 1u);
    uint64_t native = st->insns_jitted;

    ar_write(&f.cpu, 3, CC_CODE);
    ar_write(&f.cpu, 4, CC_WORD43);
    cc_call(&f.cpu, tramp, false, f.jit);
    ASSERT_TRUE(st->insns_jitted > native);   /* the writer ran natively */
    ASSERT_EQ(ar_read(&f.cpu, 2), 43u);
    cc_jit_destroy(&f);
}

TEST(code_coherence_s3_jit_sees_dram_alias_store) {
    const flexe_target_desc_t *s3 = flexe_target_by_id(FLEXE_TARGET_ESP32S3);
    cc_jit_fixture_t f;
    if (!cc_jit_init(&f, s3, false)) return;
    const uint32_t code = 0x40378200u, ret = 0x40378600u;
    const uint32_t code_dram = 0x3FC88200u;
    put_test_bytes(&f.cpu, code, cc_answer42, sizeof cc_answer42);
    put_test_bytes(&f.cpu, ret, cc_spin, sizeof cc_spin);
    cc_jit_compile(&f, code);

    f.cpu.pc = code;
    f.cpu.running = true;
    ar_write(&f.cpu, 0, ret);
    for (int i = 0; i < 8 && f.cpu.pc != ret; i++)
        (void)jit_run(f.jit, &f.cpu, 16);
    ASSERT_EQ(ar_read(&f.cpu, 2), 42u);

    mem_write8(f.cpu.mem, code_dram + 2u, 43);
    f.cpu.pc = code;
    ar_write(&f.cpu, 0, ret);
    ar_write(&f.cpu, 2, 0);
    for (int i = 0; i < 8 && f.cpu.pc != ret; i++)
        (void)jit_run(f.jit, &f.cpu, 16);
    ASSERT_EQ(f.cpu.pc, ret);
    ASSERT_EQ(ar_read(&f.cpu, 2), 43u);
    cc_jit_destroy(&f);
}
#endif

void run_code_coherence_tests(void) {
    TEST_SUITE("Translated-code coherence");
    RUN_TEST(code_coherence_diverts_only_pages_holding_translated_code);
    RUN_TEST(code_coherence_s3_data_alias_reports_instruction_address);
    RUN_TEST(code_coherence_interpreter_runs_rewritten_iram);
#ifdef FLEXE_HAS_JIT
    RUN_TEST(code_coherence_jit_retranslates_rewritten_iram);
    RUN_TEST(code_coherence_jit_keeps_blocks_for_unrelated_stores);
    RUN_TEST(code_coherence_jit_store_stops_stale_chained_successor);
    RUN_TEST(code_coherence_s3_jit_sees_dram_alias_store);
#endif
}
