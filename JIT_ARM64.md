# Porting the flexe JIT to ARM64

flexe ships an x86-64 JIT in `src/jit.c` that pre-compiles hot Xtensa
basic blocks to native machine code. This document explains its
architecture and what an ARM64 (Apple Silicon, Linux ARM64) port needs
to look like.

## What the JIT does today

- **Code cache**: 32 MB of `mmap`'d `PROT_READ | PROT_WRITE | PROT_EXEC`
  memory holds compiled blocks back-to-back.
- **Block lookup**: 64K-entry hash table keyed by `(pc, windowbase)`.
  Each entry stores the start address of the compiled block.
- **Hot detection**: counted via `pc_hook_bitmap`. Once a basic block
  hits `JIT_HOT_THRESHOLD` (3) interpreter executions, the JIT compiles
  it. Cold code stays in the interpreter forever.
- **Block translation** (`jit_compile_block` → `jit_compile_insn`):
  walks Xtensa instructions until a branch / call / return / unhandled
  opcode. For each, emits the equivalent x86-64 sequence directly into
  the code cache. Returns a function pointer.
- **Register allocation** (`regalloc_t`): caches `a1..a6` in callee-saved
  x86 regs (`R8..R13`); spilled `a0`, `a7..a15` are loaded/stored direct
  to memory. Dirty bits track deferred writes.
- **Block chaining**: when a block exits to a known PC, the JIT patches
  the trailing `0xE9 jmp rel32` to jump straight to the next block,
  eliminating dispatcher overhead between hot blocks.
- **Fallback**: if `jit_compile_insn` returns 0 (unknown opcode, exception,
  trap, complex instruction), the block compilation aborts and that PC
  stays interpreted.
- **PC-hook integration**: the JIT registers itself as a pc_hook so
  `xtensa_run`'s fast path can dispatch into JIT'd blocks without an
  extra function call.

The two architecture-specific files are:

- **`src/jit_emit_x64.h`** (~660 LOC) — pure x86-64 codegen primitives
  (`emit_load32_disp`, `emit_add_reg32`, `emit_jcc_rel32`, etc.). Static
  inline. No external dependencies beyond `<stdint.h>` and `<string.h>`.
- **`src/jit.c`** lines 796-1900 (~1100 LOC) — `jit_compile_insn` is a
  giant switch on the Xtensa opcode that calls the `emit_*` helpers from
  `jit_emit_x64.h` to produce x86 sequences. It uses hardcoded x86
  register name constants (RAX, RBX, RSI, RBP, R8..R13).

The portable infrastructure (~1300 LOC of `jit.c`) is the rest:
- Hash table & block lookup
- mmap'd code cache management
- Block-chain slot tracking & patching
- PC-hook registration
- Statistics & verification mode

## Why the x86-64 JIT was disabled on ARM64

CMake's `add_jit` block only enables `src/jit.c` if `CMAKE_SYSTEM_PROCESSOR`
matches `x86_64|AMD64`. On ARM64 macOS the build path swaps to
`src/jit_stub.c`, which provides no-op definitions for every public JIT
function so the rest of the codebase links cleanly. flexe runs entirely
in interpreter mode in that configuration.

## Current interpreter performance baseline

On Apple M-series (Asahi clock ~3.5 GHz), `bench.sh` reports:

| workload          | cycles      | wall (s) | MIPS  | real-time |
|-------------------|-------------|----------|-------|-----------|
| tjpgd (JPEG dec)  | 500,000,000 | ~1.9     | ~263  | 1.09×     |
| real_time_stats   | 500,000,000 | ~19      | ~26   | 0.11×     |
| blink             | 500,000,000 | ~22      | ~22   | 0.09×     |

Real ESP32 single-issue is ~240 MIPS at 240 MHz, so tjpgd is already
above hardware. The lower MIPS on rts/blink is mostly per-cycle stub
callback overhead (FreeRTOS scheduler, GPIO MMIO, ROM hooks) — not
something a JIT can speed up linearly. The biggest JIT wins will come
in tight loops of pure ALU/load/store, which is where tjpgd and the
spin-loop portions of real apps spend their time.

## The ARM64 port — minimum viable scope

### Phase 1: emit primitives (`src/jit_emit_arm64.h`)

A drop-in replacement for `jit_emit_x64.h` with the same function
**names**. The function bodies emit ARM64 instructions instead of x86.

Mapping that lets `jit.c` compile mostly unchanged:

| x86-64 name | ARM64 register | role                                |
|-------------|---------------|-------------------------------------|
| `RAX`       | `X0`          | first scratch / function return     |
| `RCX`       | `X1`          | scratch                             |
| `RDX`       | `X2`          | scratch                             |
| `RBX`       | `X3`          | scratch                             |
| `RBP`       | `X4`          | scratch                             |
| `RSI`       | `X5`          | scratch / second arg                |
| `RDI`       | `X6`          | scratch / first arg (was first on x86 SysV) |
| `R8..R13`   | `X19..X24`    | callee-saved (matches x86 callee-saved)|
| `R14`       | `X25`         | `cpu->mem` cache                    |
| `R15`       | `X26`         | `cpu` pointer                       |

(ARM64 calling convention reserves `X0..X7` for arguments, `X19..X28` as
callee-saved, `X29 = FP`, `X30 = LR`. The mapping above keeps all the
caller/callee semantics intact.)

The emit functions to implement (parallel to `jit_emit_x64.h`):

```c
// move / load constants
void emit_mov_reg_imm32(emit_t *e, int reg, uint32_t imm);
void emit_mov_reg_imm64(emit_t *e, int reg, uint64_t imm);
void emit_mov_reg_reg  (emit_t *e, int dst, int src);

// memory
void emit_load32_disp  (emit_t *e, int dst, int base, int32_t disp);
void emit_store32_disp (emit_t *e, int src, int base, int32_t disp);
void emit_load64_disp  (emit_t *e, int dst, int base, int32_t disp);
void emit_store64_disp (emit_t *e, int src, int base, int32_t disp);
void emit_load8u_disp  (emit_t *e, int dst, int base, int32_t disp);
void emit_load16u_disp (emit_t *e, int dst, int base, int32_t disp);
void emit_load16s_disp (emit_t *e, int dst, int base, int32_t disp);
void emit_store8_disp  (emit_t *e, int src, int base, int32_t disp);
void emit_store16_disp (emit_t *e, int src, int base, int32_t disp);

// arithmetic / logic (32-bit)
void emit_add_reg32    (emit_t *e, int dst, int src);
void emit_sub_reg32    (emit_t *e, int dst, int src);
void emit_and_reg32    (emit_t *e, int dst, int src);
void emit_or_reg32     (emit_t *e, int dst, int src);
void emit_xor_reg32    (emit_t *e, int dst, int src);
void emit_imul_reg32   (emit_t *e, int dst, int src);
void emit_neg_reg32    (emit_t *e, int reg);
void emit_add_reg32_imm32(emit_t *e, int reg, int32_t imm);
void emit_sub_reg32_imm32(emit_t *e, int reg, int32_t imm);
void emit_and_reg32_imm32(emit_t *e, int reg, int32_t imm);

// shifts
void emit_shl_reg32_imm(emit_t *e, int reg, uint8_t sh);
void emit_shr_reg32_imm(emit_t *e, int reg, uint8_t sh);
void emit_sar_reg32_imm(emit_t *e, int reg, uint8_t sh);
void emit_shl_reg32_cl (emit_t *e, int reg);
void emit_shr_reg32_cl (emit_t *e, int reg);
void emit_sar_reg32_cl (emit_t *e, int reg);

// compare / branch
void emit_cmp_reg32    (emit_t *e, int a, int b);
void emit_test_reg32   (emit_t *e, int a, int b);
int  emit_jcc_rel32    (emit_t *e, int cond);   // returns patch site
int  emit_jmp_rel32    (emit_t *e);             // returns patch site
void emit_patch_rel32  (emit_t *e, int site);   // patch target = current ptr
void emit_call_imm     (emit_t *e, void *target);
void emit_ret          (emit_t *e);
```

The x86 versions of all of these together are ~600 LOC of static-inline
C. The ARM64 versions will be similar in size — ARM64 has fixed-width
4-byte instructions and a regular encoding, so the bit-fiddling is
actually cleaner than x86 ModRM/SIB.

### Phase 2: `jit.c` source compatibility

Wrap the `RAX..R15` enum and the `REG_CPU/REG_MEM/REG_WB` macros in an
`#ifdef`:

```c
#if defined(__x86_64__) || defined(_M_X64)
#  include "jit_emit_x64.h"
#  define REG_CPU R15
#  define REG_MEM R14
#elif defined(__aarch64__)
#  include "jit_emit_arm64.h"
#  define REG_CPU R15  /* aliases X26 in arm64 enum */
#  define REG_MEM R14  /* aliases X25 */
#endif
```

The `enum { RAX, RCX, ..., R15 }` in `jit_emit_arm64.h` would assign
ARM64 register numbers to those names. `jit_compile_insn`'s body stays
unchanged.

### Phase 3: prologue / epilogue

Both architectures need a per-block prologue (set up `REG_CPU`, `REG_MEM`,
load callee-saved AR registers) and an epilogue (flush dirty AR regs,
return to dispatcher). x86 uses `push rbp; mov rbp, rsp; ...; ret`. ARM64
uses `stp x29, x30, [sp, #-16]!; ...; ldp x29, x30, [sp], #16; ret`. The
prologue/epilogue is emitted from `jit_compile_block` (jit.c around
line 2017), so that one function gets a small `#ifdef __aarch64__` branch.

### Phase 4: macOS-specific JIT memory permissions

Apple Silicon enforces W^X on JIT pages: a page can be either writable
or executable, never both. The standard recipe is:

```c
// Allocate writable+executable with the JIT entitlement
void *cache = mmap(NULL, JIT_CODE_CACHE_SIZE,
                   PROT_READ | PROT_WRITE | PROT_EXEC,
                   MAP_PRIVATE | MAP_ANON | MAP_JIT, -1, 0);

// Toggle to writable mode before emitting
pthread_jit_write_protect_np(0);
// ... emit ...
pthread_jit_write_protect_np(1);

// I-cache flush
sys_icache_invalidate(cache, JIT_CODE_CACHE_SIZE);
```

The flexe binary does NOT need a code-signing entitlement for `MAP_JIT`
on macOS 11+ when running locally — only signed app bundles do. For
distribution it would.

### Phase 5: enable in CMake

```cmake
if(NOT MSVC AND CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|AMD64|arm64|aarch64")
    list(APPEND LIB_SOURCES src/jit.c)
endif()
```

Drop `src/jit_stub.c` from the build path on ARM64.

## Expected speedup

Reasoning by analogy from QEMU's TCG (~3-5× over interpretation) and
the existing flexe x86 JIT's stated wins (~3× on x86):

- **tjpgd** at 263 MIPS interpreter → likely **600–900 MIPS** with JIT.
  That puts the emulator at 2.5–4× real ESP32 wall-clock speed.
- **rts / blink** stays bottlenecked on ROM-stub call overhead since
  every FreeRTOS context switch and ESP_LOGI exits to interpreted code.
  Maybe 50–80 MIPS, modest gain.
- **Pure ALU loops** (FFT, image filters, JSON parse) — the workloads
  most CYD apps spend their time in — should see the full 3× speedup.

## Bench harness

`bench.sh` already exists and supports `--jit`. Once the ARM64 path is
wired:

```bash
./bench.sh           # interpreter baseline
./bench.sh --jit     # JIT enabled
```

It runs each workload 3× and reports averaged MIPS + real-time ratio
so before/after numbers are directly comparable.

## Open questions

- **Branch-island distance.** ARM64 conditional branches have a ±1 MB
  range. The 32 MB code cache is bigger than that, so blocks far apart
  in the cache need branch islands or `BR <reg>` indirect jumps. Easy
  to handle but a thing to be aware of.
- **W^X overhead.** `pthread_jit_write_protect_np` is a syscall.
  Toggling per block could be slow; better to compile in batches or
  hold the write window open for the whole compilation pass.
- **Instruction cache invalidation.** `sys_icache_invalidate` is
  required after any code mutation. flexe's block chaining patches
  jmp targets in-place — every patch needs an i-cache flush of the
  affected range.
