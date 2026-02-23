#ifndef TEST_HELPERS_H
#define TEST_HELPERS_H

#include <stdio.h>
#include <stdint.h>
#include "xtensa.h"
#include "memory.h"

extern int test_count;
extern int test_passes;
extern int test_failures;

#define ASSERT_EQ(a, b) do { \
    uint32_t _a = (uint32_t)(a), _b = (uint32_t)(b); \
    if (_a != _b) { \
        fprintf(stderr, "  FAIL %s:%d: %s == 0x%X, expected 0x%X\n", \
                __FILE__, __LINE__, #a, _a, _b); \
        test_failures++; \
    } else { test_passes++; } \
} while(0)

#define ASSERT_EQ64(a, b) do { \
    uint64_t _a = (uint64_t)(a), _b = (uint64_t)(b); \
    if (_a != _b) { \
        fprintf(stderr, "  FAIL %s:%d: %s == 0x%lX, expected 0x%lX\n", \
                __FILE__, __LINE__, #a, (unsigned long)_a, (unsigned long)_b); \
        test_failures++; \
    } else { test_passes++; } \
} while(0)

#define ASSERT_TRUE(x)  ASSERT_EQ(!!(x), 1)
#define ASSERT_FALSE(x) ASSERT_EQ(!!(x), 0)

#define TEST(name) static void name(void)
#define RUN_TEST(name) do { \
    printf("  %s... ", #name); \
    int _before = test_failures; \
    name(); \
    test_count++; \
    if (test_failures == _before) printf("ok\n"); \
    else printf("\n"); \
} while(0)

#define TEST_SUITE(name) printf("Suite: %s\n", name)

/*
 * Shared test helpers: instruction builders, setup/teardown
 */
#define BASE 0x40080000u

static inline void put_insn3(xtensa_cpu_t *cpu, uint32_t addr, uint32_t insn) {
    mem_write8(cpu->mem, addr,     (uint8_t)(insn & 0xFF));
    mem_write8(cpu->mem, addr + 1, (uint8_t)((insn >> 8) & 0xFF));
    mem_write8(cpu->mem, addr + 2, (uint8_t)((insn >> 16) & 0xFF));
}

static inline void put_insn2(xtensa_cpu_t *cpu, uint32_t addr, uint16_t insn) {
    mem_write8(cpu->mem, addr,     (uint8_t)(insn & 0xFF));
    mem_write8(cpu->mem, addr + 1, (uint8_t)((insn >> 8) & 0xFF));
}

static inline uint32_t rrr(int op2, int op1, int r, int s, int t) {
    return (uint32_t)((op2 << 20) | (op1 << 16) | (r << 12) | (s << 8) | (t << 4) | 0);
}

static inline uint16_t narrow(int op0, int r, int s, int t) {
    return (uint16_t)((r << 12) | (s << 8) | (t << 4) | op0);
}

static inline void setup(xtensa_cpu_t *cpu) {
    xtensa_cpu_init(cpu);
    cpu->mem = mem_create();
    cpu->pc = BASE;
}

static inline void teardown(xtensa_cpu_t *cpu) {
    mem_destroy(cpu->mem);
}

#endif /* TEST_HELPERS_H */
