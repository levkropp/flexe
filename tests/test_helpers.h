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

#endif /* TEST_HELPERS_H */
