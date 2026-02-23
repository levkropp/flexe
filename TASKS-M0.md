# M0 Tasks: Project Scaffold + Build System

Granular task breakdown for Milestone 0. Complete these in order.

---

## Task 0.1: Create CMakeLists.txt

Create the build system. Should produce two targets:
- `xtensa-emu` — the emulator executable (or library + thin main)
- `xtensa-tests` — the test runner
- `xt-dis` — the disassembler tool

```cmake
cmake_minimum_required(VERSION 3.14)
project(xtensa-emulator C)
set(CMAKE_C_STANDARD 17)

# Library (core emulator)
add_library(xtensa-emu-lib STATIC
    src/xtensa.c
    src/memory.c
    src/loader.c
)
target_include_directories(xtensa-emu-lib PUBLIC src/)
target_compile_options(xtensa-emu-lib PRIVATE -Wall -Wextra -Werror -Wno-unused-parameter)

# Main executable
add_executable(xtensa-emu src/main.c)
target_link_libraries(xtensa-emu PRIVATE xtensa-emu-lib)

# Test runner
add_executable(xtensa-tests tests/test_main.c)
target_link_libraries(xtensa-tests PRIVATE xtensa-emu-lib)

# Disassembler tool
add_executable(xt-dis tools/xt-dis.c)
target_link_libraries(xt-dis PRIVATE xtensa-emu-lib)
```

**Acceptance**: `cmake -S . -B build && cmake --build build` succeeds with zero warnings.

---

## Task 0.2: Create `src/xtensa.h` — CPU State and Constants

This is the most important header. It defines:

### 0.2a: Special register number constants

```c
#define XT_SR_LBEG          0
#define XT_SR_LEND          1
#define XT_SR_LCOUNT        2
#define XT_SR_SAR           3
#define XT_SR_BR            4
#define XT_SR_LITBASE       5
#define XT_SR_SCOMPARE1     12
#define XT_SR_ACCLO         16
#define XT_SR_ACCHI         17
#define XT_SR_MR0           32
#define XT_SR_MR1           33
#define XT_SR_MR2           34
#define XT_SR_MR3           35
#define XT_SR_WINDOWBASE    72
#define XT_SR_WINDOWSTART   73
#define XT_SR_IBREAKENABLE  96
#define XT_SR_MEMCTL        97
#define XT_SR_ATOMCTL       99
#define XT_SR_IBREAKA0      128
#define XT_SR_IBREAKA1      129
#define XT_SR_DBREAKA0      144
#define XT_SR_DBREAKA1      145
#define XT_SR_DBREAKC0      160
#define XT_SR_DBREAKC1      161
#define XT_SR_CONFIGID0     176
#define XT_SR_EPC1          177
// ... EPC2-EPC7 = 178-183
#define XT_SR_DEPC          192
#define XT_SR_EPS2          194
// ... EPS3-EPS7 = 195-200
#define XT_SR_CONFIGID1     208
#define XT_SR_EXCSAVE1      209
// ... EXCSAVE2-7 = 210-215
#define XT_SR_CPENABLE      224
#define XT_SR_INTSET        226
#define XT_SR_INTCLEAR      227
#define XT_SR_INTENABLE     228
#define XT_SR_PS            230
#define XT_SR_VECBASE       231
#define XT_SR_EXCCAUSE      232
#define XT_SR_DEBUGCAUSE    233
#define XT_SR_CCOUNT        234
#define XT_SR_PRID          235
#define XT_SR_ICOUNT        236
#define XT_SR_ICOUNTLEVEL   237
#define XT_SR_EXCVADDR      238
#define XT_SR_CCOMPARE0     240
#define XT_SR_CCOMPARE1     241
#define XT_SR_CCOMPARE2     242
#define XT_SR_MISC0         244
#define XT_SR_MISC1         245
#define XT_SR_MISC2         246
#define XT_SR_MISC3         247
```

### 0.2b: `xtensa_cpu_t` struct

See ARCHITECTURE.md section 2 and `../cyd-emulator/docs/xtensa-emulator-research.md`
section 2 for the full struct definition. Include all register state even if
not all is used in M0.

### 0.2c: Inline register access helpers

```c
static inline uint32_t ar_read(const xtensa_cpu_t *cpu, int n) {
    return cpu->ar[((cpu->windowbase * 4) + n) & 63];
}
static inline void ar_write(xtensa_cpu_t *cpu, int n, uint32_t val) {
    cpu->ar[((cpu->windowbase * 4) + n) & 63] = val;
}
```

### 0.2d: Instruction field extraction macros

See ARCHITECTURE.md section 3.2 for the full set of `XT_OP0`, `XT_T`, `XT_S`,
`XT_R`, etc. macros.

### 0.2e: PS register field accessors

```c
#define XT_PS_INTLEVEL(ps)  ((ps) & 0xF)
#define XT_PS_EXCM(ps)      (((ps) >> 4) & 1)
#define XT_PS_UM(ps)        (((ps) >> 5) & 1)
#define XT_PS_RING(ps)      (((ps) >> 6) & 3)
#define XT_PS_OWB(ps)       (((ps) >> 8) & 0xF)
#define XT_PS_CALLINC(ps)   (((ps) >> 12) & 3)  /* note: only 2 bits used (1,2,3) */
#define XT_PS_WOE(ps)       (((ps) >> 16) & 1)
```

### 0.2f: Public function declarations

```c
void xtensa_cpu_init(xtensa_cpu_t *cpu);
void xtensa_cpu_reset(xtensa_cpu_t *cpu);
int  xtensa_step(xtensa_cpu_t *cpu);
int  xtensa_run(xtensa_cpu_t *cpu, int max_cycles);
int  xtensa_disasm(const xtensa_cpu_t *cpu, uint32_t addr, char *buf, int bufsize);
```

**Acceptance**: Header compiles without errors when included from a .c file.

---

## Task 0.3: Create `src/xtensa.c` — Init/Reset Stubs

Implement:

- `xtensa_cpu_init()` — zero-initialize the entire struct
- `xtensa_cpu_reset()` — set initial register values per ISA spec:
  - PC = 0x40000400 (ESP32 reset vector)
  - PS = 0x0001001F (EXCM=1, INTLEVEL=15, WOE=1, UM=0, RING=0)
  - WINDOWBASE = 0
  - WINDOWSTART = 1 (window 0 is valid)
  - SAR = undefined (set to 0)
  - LCOUNT = 0
  - CCOUNT = 0
  - VECBASE = 0x40000000 (ESP32 default)
  - PRID = 0xCDCD (ESP32 PRO_CPU)
  - CPENABLE = 0
  - ATOMCTL = 0x28 (ESP32 default)
  - CONFIGID0 / CONFIGID1 = ESP32 values (or 0 for now)
- `xtensa_step()` — stub that just increments PC by 3 and returns 0
- `xtensa_run()` — loop calling `xtensa_step()` up to max_cycles

**Acceptance**: Functions exist, compile, and don't crash when called.

---

## Task 0.4: Create `src/memory.h` and `src/memory.c` — Memory Stubs

Define:

```c
typedef struct xtensa_mem xtensa_mem_t;

xtensa_mem_t *mem_create(void);
void mem_destroy(xtensa_mem_t *mem);
void mem_reset(xtensa_mem_t *mem);

uint8_t  mem_read8(xtensa_mem_t *mem, uint32_t addr);
uint16_t mem_read16(xtensa_mem_t *mem, uint32_t addr);
uint32_t mem_read32(xtensa_mem_t *mem, uint32_t addr);

void mem_write8(xtensa_mem_t *mem, uint32_t addr, uint8_t val);
void mem_write16(xtensa_mem_t *mem, uint32_t addr, uint16_t val);
void mem_write32(xtensa_mem_t *mem, uint32_t addr, uint32_t val);

/* Bulk load (for firmware loading) */
int mem_load(xtensa_mem_t *mem, uint32_t addr, const uint8_t *data, size_t len);

/* Direct pointer access (for instruction fetch, performance) */
const uint8_t *mem_get_ptr(xtensa_mem_t *mem, uint32_t addr);
```

Stub implementation: allocate a flat 520KB SRAM buffer + 384KB ROM buffer.
Read/write dispatch based on address range. Peripheral range returns 0 for now.

**Acceptance**: `mem_create()` succeeds, `mem_write32()` followed by `mem_read32()`
returns the written value for SRAM addresses.

---

## Task 0.5: Create `src/loader.h` and `src/loader.c` — Loader Stubs

Define:

```c
typedef struct {
    uint32_t entry_point;
    int segment_count;
    int result;  /* 0 = success */
    char error[256];
} load_result_t;

load_result_t loader_load_bin(xtensa_mem_t *mem, const char *path);
```

Stub implementation: open file, read first byte, check magic 0xE9, return
entry point = 0 and result = 0. Full parsing deferred to M3.

**Acceptance**: Function exists and compiles. Calling with NULL path sets error message.

---

## Task 0.6: Create `src/main.c` — Entry Point

Minimal main that:
1. Creates CPU and memory
2. Optionally loads a .bin file from argv[1]
3. Calls `xtensa_run(cpu, 100)` (execute 100 instructions)
4. Prints final PC
5. Cleans up

```c
int main(int argc, char *argv[]) {
    xtensa_cpu_t cpu;
    xtensa_mem_t *mem = mem_create();
    xtensa_cpu_init(&cpu);
    xtensa_cpu_reset(&cpu);
    cpu.mem = mem;

    if (argc > 1) {
        load_result_t res = loader_load_bin(mem, argv[1]);
        if (res.result != 0) {
            fprintf(stderr, "Load error: %s\n", res.error);
            return 1;
        }
        cpu.pc = res.entry_point;
    }

    int cycles = xtensa_run(&cpu, 100);
    printf("Executed %d cycles, PC = 0x%08X\n", cycles, cpu.pc);

    mem_destroy(mem);
    return 0;
}
```

**Acceptance**: Running `./build/xtensa-emu` prints "Executed 100 cycles, PC = ..."

---

## Task 0.7: Create `tests/test_helpers.h` — Test Macros

Simple test framework with no dependencies:

```c
#define ASSERT_EQ(a, b) do { \
    uint32_t _a = (a), _b = (b); \
    if (_a != _b) { \
        fprintf(stderr, "FAIL %s:%d: %s == 0x%X, expected 0x%X\n", \
                __FILE__, __LINE__, #a, _a, _b); \
        test_failures++; \
    } else { test_passes++; } \
} while(0)

#define ASSERT_TRUE(x) ASSERT_EQ(!!(x), 1)
#define ASSERT_FALSE(x) ASSERT_EQ(!!(x), 0)

#define TEST(name) static void name(void)
#define RUN_TEST(name) do { name(); test_count++; } while(0)
```

**Acceptance**: Macros compile and produce output on failure.

---

## Task 0.8: Create `tests/test_main.c` — Test Runner

```c
#include "test_helpers.h"

int test_count = 0, test_passes = 0, test_failures = 0;

/* Include test files */
// #include "test_alu.c"  (added in M2)

int main(void) {
    printf("Running xtensa-emulator tests...\n");

    /* Run test suites */
    // RUN_TEST(test_add);  (added in M2)

    printf("\n%d tests, %d passed, %d failed\n",
           test_count, test_passes, test_failures);
    return test_failures > 0 ? 1 : 0;
}
```

**Acceptance**: `./build/xtensa-tests` prints "0 tests, 0 passed, 0 failed" and exits 0.

---

## Task 0.9: Create `tools/xt-dis.c` — Disassembler Stub

Minimal tool that reads a binary file and prints hex bytes:

```c
int main(int argc, char *argv[]) {
    if (argc < 2) {
        fprintf(stderr, "Usage: xt-dis <binary> [base_addr]\n");
        return 1;
    }
    uint32_t base = (argc > 2) ? strtoul(argv[2], NULL, 0) : 0;
    /* Read file, print hex dump for now */
    /* Full disassembly added in M1 */
}
```

**Acceptance**: `./build/xt-dis somefile.bin 0x40080000` prints hex bytes.

---

## Task 0.10: Verify Full Build

Run the complete build and verify:

```bash
rm -rf build
cmake -S . -B build
cmake --build build 2>&1 | grep -c "warning:"  # should be 0
./build/xtensa-emu          # prints cycle/PC info
./build/xtensa-tests        # prints 0 tests, exits 0
./build/xt-dis              # prints usage message
```

**Acceptance**: All three binaries build and run without errors or warnings.

---

## Summary

| Task | File(s) | Description |
|------|---------|-------------|
| 0.1  | `CMakeLists.txt` | Build system |
| 0.2  | `src/xtensa.h` | CPU state, constants, macros, API |
| 0.3  | `src/xtensa.c` | Init, reset, step/run stubs |
| 0.4  | `src/memory.h`, `src/memory.c` | Memory subsystem stubs |
| 0.5  | `src/loader.h`, `src/loader.c` | Firmware loader stubs |
| 0.6  | `src/main.c` | Entry point |
| 0.7  | `tests/test_helpers.h` | Test macros |
| 0.8  | `tests/test_main.c` | Test runner |
| 0.9  | `tools/xt-dis.c` | Disassembler stub |
| 0.10 | (none) | Build verification |

Total: ~10 files, ~500-700 lines of code.
