# Testing & benchmarking flexe

This document explains the two ways flexe measures itself: the
correctness test suite (`./build/xtensa-tests`) and the throughput
benchmark (`./bench.sh`), what their numbers mean, and what to expect
on different hosts.

## Correctness: the test suite

```bash
cd ~/flexe
cmake --build build -j
./build/xtensa-tests
```

Output looks like:

```
JIT tests skipped (x86-64 only)
468 tests, 869 passed, 0 failed
```

**Read this carefully** — these numbers are *not* "468 out of 869 tests
passed". They are:

- **468** distinct test functions (each declared with the `TEST(...)`
  macro in `tests/test_*.c`)
- **869** total `ASSERT_*` invocations across all of those test
  functions
- **0** assertion failures

A test function can have many assertions. For example,
`test_alu_add_basic` calls `ASSERT_EQ` six times to verify the result
of six different additions. Both `468` and `869` go up over time as
the suite grows; what matters is that the failure column is `0`.

The "JIT tests skipped" line appears when running on ARM64. The JIT
test file `tests/test_jit.c` only compiles on x86-64 because the JIT
itself is x86-64-only today (see `JIT_ARM64.md` for the port plan).

### Test category breakdown

`tests/test_main.c` is the entry point. It runs every category in
order:

| Category               | What it verifies                          |
|------------------------|-------------------------------------------|
| `decode`               | Xtensa instruction field extraction       |
| `alu`                  | ADD/SUB/AND/OR/XOR/etc. semantics         |
| `shift`                | SLL/SRL/SRA/SLLI/SRLI/SRAI                |
| `move`                 | MOV.N, MOVI, MOVT, MOVF                   |
| `loadstore`            | L8UI / L16SI / L32I / S8I / S16I / S32I   |
| `memory`               | Page-table mapping, MMIO dispatch         |
| `loader`               | ESP32 .bin segment parsing                |
| `branch`               | All branch flavours (BEQ/BNE/BLT/BLTU…)   |
| `loop`                 | Zero-overhead loop (LOOP, LOOPGTZ, LOOPNEZ)|
| `integration`          | Multi-instruction sequences               |
| `window`               | Windowed register spill/fill (CALL4/8/12) |
| `exception`            | Exception cause routing, EPC/EPS update   |
| `boolean`              | Boolean register operations (BR)          |
| `mac16`                | MUL16, MULL, MACID                        |
| `fp_ldst`              | F32 load/store                            |
| `fp_arith`             | F32 arithmetic + conversion               |
| `peripherals`          | UART/GPIO/RTC MMIO behavior               |
| `rom_stubs`            | ets_printf, memcpy, etc.                  |
| `debug`                | Breakpoints, dump_mem                     |
| `memory_map`           | ESP32 region addresses                    |
| `freertos`             | xTaskCreate, vTaskDelay, queues, mutexes  |
| `esp_timer`            | esp_timer create/start/dispatch           |
| `firmware_compat`      | NVS, VFS/SPIFFS/FAT/SDMMC, deeper integration |
| `jit`                  | x86-64 JIT block compilation (x86 only)   |

When you add a new feature, add a corresponding test in the right
file. The pattern is `TEST(my_thing) { ... }` and a call to
`run_my_thing_tests()` in `tests/test_main.c`.

## Throughput: the benchmark

```bash
./bench.sh           # interpreter
./bench.sh --jit     # with -J (x86-64 only today)
```

Output:

```
workload           cycles    wall(s)       MIPS  real-time
--------           ------    -------       ----  ---------
tjpgd           500000000      1.904      262.6       1.09x
rts             500000000     18.789       26.6       0.11x
blink           500000000     22.589       22.1       0.09x
```

### How to read the columns

- **cycles** — number of emulated Xtensa cycles. Every workload runs
  with `-c 500000000` so wall time / MIPS / real-time are directly
  comparable.
- **wall(s)** — average host wall-clock seconds across 3 reps.
- **MIPS** — `cycles / wall / 1e6`. Since ESP32 LX6 is single-issue
  in-order, cycles ≈ instructions for the bulk of the workload, so
  this is the right unit.
- **real-time** — `cycles / 240e6 / wall`. A real ESP32 runs at
  240 MHz, so 1.0× means flexe is keeping up with hardware. >1.0×
  means the emulator is faster than the real chip.

### Why the spread

| workload | what it is | bottleneck                              |
|----------|-----------|------------------------------------------|
| `tjpgd`  | JPEG decoder pushing pixels to ILI9341 | raw integer ALU + memory loads — close to ideal interpreter case |
| `rts`    | FreeRTOS dual-core spin/print/stats demo | per-cycle stub call overhead (vTaskDelay, ESP_LOGI, scheduler) |
| `blink`  | LED toggle + ESP_LOGI in a loop | dominated by `gpio_set_level` + `esp_log_write` ROM stubs (3M+ calls per 100M cycles) |

The slow workloads aren't "slow because the interpreter is slow" —
they're slow because every ROM stub call exits the interpreter into
host C, runs the stub body, and returns. JIT can compile the inline
Xtensa code 3-4× faster, but the stub call boundaries remain the same
speed. The biggest JIT wins are on workloads that look like tjpgd
(tight inline math).

### Interpreting the real-time column

| value | meaning                                      |
|-------|----------------------------------------------|
| `1.0×` | flexe runs at the same wall-clock speed as a real ESP32 at 240 MHz |
| `> 1.0×` | flexe is faster than hardware (yes this happens — modern ARM64 hosts running pure-C interpretation can outrun a 240 MHz Xtensa) |
| `< 1.0×` | flexe is slower; usually a stub-bound workload |

For interactive sandbox use, anything above ~0.5× is "fast enough" —
the user won't perceive lag in animations or button responses. For
batch testing or fuzzing, higher is better.

## Running individual firmware

`bench.sh` is just a wrapper around manual flexe invocations:

```bash
./build/xtensa-emu -q -s firmware.elf -c 500000000 firmware.bin
```

Add `-T` for instruction trace, `-b symbol_name` for a breakpoint,
`-c N` to limit cycles, `-q` to suppress unhandled peripheral warnings.
See `~/flexe/CLAUDE.md` for the full flag list.

## Reproducing on different hosts

The benchmark numbers are sensitive to host CPU and frequency scaling.
Quick sanity check before reporting numbers from a new machine:

```bash
sysctl -n machdep.cpu.brand_string  # macOS
lscpu | grep "Model name"           # Linux
```

Disable thermal throttling / power saving for the duration of the
benchmark if you can. On macOS that means running plugged in, with
nothing else CPU-bound. On Linux: `sudo cpupower frequency-set -g performance`.

Re-run `bench.sh` 5+ times and take the median. Single-run numbers
move around 5–15% with cache and scheduler noise.
