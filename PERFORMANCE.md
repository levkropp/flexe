# flexe performance status — ARM64 macOS

This document captures the throughput baseline, the optimization
history, and what's known about the AOT path.

## Current numbers (Apple M-series, `bench.sh`)

| workload | cycles | wall(s) | MIPS | real-time | notes |
|---|---:|---:|---:|---:|---|
| **tjpgd** (JPEG decode) | 500M | ~1.7 | **~295** | **1.22×** | compute-bound; close to ideal interpreter case |
| **real_time_stats** | 500M | ~3.0 | **~167** | 0.70× | FreeRTOS dual-core, lots of context switches |
| **blink** | 500M | ~24 | ~21 | 0.09× | **virtual-time-dominated**: most cycles are `vTaskDelay` fast-forwards with zero actual instructions executing |

For comparison: a real ESP32 LX6 runs at 240 MHz / ~240 MIPS
single-issue. **flexe is 22% faster than real hardware on tjpgd**, and
~70% real-time on FreeRTOS-heavy workloads, on Apple Silicon. The
blink number is misleading — the metric counts virtual cycles not
actually-executed instructions.

## Optimization history

Before this session: **rts at 26 MIPS, blink at 22 MIPS, tjpgd at 263
MIPS**. The path to today's numbers:

### 1. The `getenv()` regression (huge: 5–7× on rts)

A debugging agent had left `getenv("FLEXE_CORE1_TRACE")` in the inner
`xtensa_run` loop:

```c
for (i = 0; i < max_cycles; i++) {
    if (cpu->core_id == 1 && getenv("FLEXE_CORE1_TRACE") && (int)cc > 30020)
        fprintf(...);
    xtensa_step_impl(...);
}
```

`getenv` does a linear strcmp through every environment variable on
every call. With ~30 env vars at ~100ns each, this added ~3μs per
iteration. Over 500M cycles that's ~25 seconds of pure scan overhead.

**Fix**: cache the env-var lookup once at startup into a static int.

```c
static int g_core1_trace_enabled = -1;
if (g_core1_trace_enabled < 0)
    g_core1_trace_enabled = (getenv("FLEXE_CORE1_TRACE") != NULL);
int trace_c1 = (g_core1_trace_enabled && cpu->core_id == 1);
for (i = 0; i < max_cycles; i++) {
    if (trace_c1 && ...) fprintf(...);
    ...
}
```

**Result**: rts 26 → 134 MIPS (5.2× speedup).

### 2. Force-inline the `exec_*` instruction handlers (15–35% across the board)

The interpreter's instruction dispatch went through `exec_qrst`,
`exec_lsai`, `exec_b`, `exec_calln`, `exec_si`, `exec_mac16` — six
file-static functions, none marked `inline`. clang at `-O2` was keeping
them as separate function calls (they're 1000+ LOC each and exceeded
the inline budget). Every Xtensa instruction was paying ~5-10 host
cycles of pure dispatch overhead.

**Fix**: mark each as `static inline __attribute__((always_inline))`.
The compiler now folds the entire instruction switch tree into one
giant `xtensa_step_impl` body that gets inlined into `xtensa_run`.

**Result**: tjpgd 263 → 295 MIPS, rts 134 → 167 MIPS. Combined with
fix #1, **rts is now 6.4× faster than the starting baseline**.

### 3. Core 1 dual-core boot trap (correctness, not perf)

Many firmwares (light_sleep, oneshot_read, spi_eeprom, spi_lcd_touch)
were trapping with `[TRAP] Invalid PC=0x00000000` very early on Core 1.
Root cause: the `esp_startup_start_app_other_cores` ROM stub was
returning to the caller, but the real ESP-IDF function never returns —
it enters the scheduler and waits for work. Returning sent Core 1 into
the literal pool right after the call, where misdecoded literals
clobbered `a0` and the next `retw` jumped to PC=0.

**Fix**: stub now parks Core 1 by setting `cpu->running = false` and
leaving PC at the function entry. `flexe_session_post_batch` polls for
a newly-eligible task and resumes Core 1 when one appears, matching
real-hardware semantics.

**Result**: all four reproducers no longer trap. spi_lcd_touch even
calls LVGL's `draw_buf_flush` 629 times — LVGL's render task is now
running on Core 1 as designed.

## What's NOT yet helping

### The x86-64 JIT (`src/jit.c`, 2,383 LOC)

Disabled on ARM64 — only the x86 codegen primitives exist. Phase 1 of
the port (writing `src/jit_emit_arm64.h` with parallel-named codegen
helpers) is **complete**: 660 LOC, all functions implemented,
self-test passes natively. Phases 2-5 (wiring jit.c to use the new
header on ARM64, prologue/epilogue, macOS W^X handling, CMake re-enable)
are pending. See `JIT_ARM64.md`.

Realistic JIT speedup for our use case is ~3× on compute workloads,
~0% on stub-bound workloads.

### The AOT static recompiler (`tools/aot_recompile.py`)

**Status**: pipeline complete, runtime correctness pending.

- Translation rate: **97% of typical ESP-IDF firmware** (blink/tjpgd/
  hello_world all >96% function coverage).
- Generation: ELF → Python decoder → C source → `clang -O3 -flto` →
  shared library + JSON manifest.
- Dispatcher: wired into `xtensa_step_impl` with O(1) hash lookup.
  flexe loads the dylib via `--aot <path>` and dispatches into native
  C functions before the interpreter runs.
- `--aot` flag works end-to-end. flexe correctly defers to ROM stubs
  via the pc_hook bitmap, so AOT only kicks in for non-stub firmware
  PCs.
- Synthetic benchmark on one verified function (`s_compare_reserved_
  regions`): **1.6-2.4× speedup** vs the in-process reference
  interpreter. This is a lower bound on what the full AOT path could
  deliver against flexe's actual interpreter (with all its bookkeeping)
  — likely 3-5× when the integration is correct.
- **The blocker**: real-firmware boot crashes early with the AOT
  enabled. The translator generates correct code for the synthetic
  test but bugs in some instructions (probably one of the new branch
  variants, `entry`, or the `l32r` literal resolver) corrupt cpu state
  in subtle ways during the dual-core init path.

The next session should build a **differential fuzzer** (task #35)
that for each translated function: snapshots cpu state, runs the AOT
version, snapshots again, restores, runs the interpreter, and compares.
The first divergent function is the bug.

Once the runtime correctness is sorted, the expected end-to-end
speedup on tjpgd-class workloads is in the 5-10× range — pushing
flexe to **2-3 GHz of equivalent ESP32 throughput** on Apple Silicon.

## What you should reach for

For the **interactive sandbox use case**, the current numbers are
already usable:
- 1.22× real-time on compute → animations and display refreshes look
  smooth in the litegraph display node.
- 0.7× real-time on FreeRTOS tasks → button presses and touch events
  feel responsive.
- All four reproducer firmwares boot cleanly and produce output.

For the **batch testing / fuzzing** use case, the AOT path is the
path forward — that's the real 5-10× lever, and it can stack with
PGO / LTO compilation of flexe itself.

## Reproducing the numbers

```bash
cd ~/flexe
cmake -S . -B build && cmake --build build -j
./build/xtensa-tests   # 468 / 869 / 0 failed
./bench.sh             # interpreter
./bench.sh --jit       # x86-64 only (no-op on ARM64)
```

For per-firmware sanity:
```bash
./build/xtensa-emu -q -s ~/esp/blink/build/blink.elf -c 30000000 ~/esp/blink/build/blink.bin | grep Turning
./build/xtensa-emu -q -s ~/esp/spi_lcd_touch/build/spi_lcd_touch.elf -c 200000000 ~/esp/spi_lcd_touch/build/spi_lcd_touch.bin | grep -E "TRAP|LVGL"
```

To experiment with AOT (knowing it's currently buggy on real
firmware):
```bash
python3 tools/aot_recompile.py ~/esp/blink/build/blink.elf ~/esp/blink/build/blink.bin
./build/xtensa-emu -q --aot /tmp/firmware.aot.dylib -s ~/esp/blink/build/blink.elf -c 50000000 ~/esp/blink/build/blink.bin
```
