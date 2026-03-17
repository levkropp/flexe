# Xtensa Emulator (flexe) - Project Context

MIT-licensed Xtensa LX6 CPU emulator for running ESP32 .bin firmware files.
Submodule of [cyd-emulator](../cyd-emulator) (SDL2 GUI wrapper).

## Build

```bash
cd flexe
cmake -S . -B build
cmake --build build
./build/xtensa-emu          # Run emulator
./build/xtensa-tests        # Run test suite (459 tests)
./build/xt-dis <file> [addr] # Disassemble binary
./build/trace-filter         # Post-process verbose trace output
```

## Running Firmware

```bash
# Basic run (quiet, with symbols, cycle limit)
./build/xtensa-emu -q -s firmware.elf -c 10000000 firmware.bin

# With progress heartbeat (lightweight observability, no verbose logging)
./build/xtensa-emu -q -P 10000000 -s firmware.elf -c 100000000 firmware.bin

# Long-duration test with checkpointing (100M cycles = 0.625 sec @ 160MHz)
./build/xtensa-emu -q -P 20000000 -s firmware.elf -c 100000000 \
    --checkpoint-interval 20000000 --checkpoint-dir ./checkpoints firmware.bin

# Restore from checkpoint and continue
./build/xtensa-emu -q -s firmware.elf --restore ./checkpoints/checkpoint-20000000.sav \
    -c 50000000 firmware.bin

# Single-core mode (disable APP_CPU)
./build/xtensa-emu -1 -q -s firmware.elf -c 10000000 firmware.bin

# Dual-core mode (default, both PRO_CPU and APP_CPU enabled)
./build/xtensa-emu -q -s firmware.elf -c 10000000 firmware.bin
```

## CLI Flags

| Flag | Description |
|------|-------------|
| `-c N` | Max cycles (default: 10M) |
| `-q` | Quiet: suppress per-access unhandled peripheral warnings |
| `-s file.elf` | Load ELF symbols for trace/breakpoints |
| `-t` | Basic instruction trace to stderr |
| `-T` | Verbose trace (always via ring buffer, auto-dumps on crash) |
| `-T START:END` | Windowed trace: collect only in cycle range, batch elsewhere |
| `-D mode` | Dump condition: `crash` (default), `flush`, `tail:N` (repeatable) |
| `-E` | Event log: stub calls, task switches, exceptions (fast, tiny output) |
| `-P N` | Progress heartbeat every N cycles |
| `-W` | Window trace (spill/fill/ENTRY/RETW events) |
| `-F` | Function-call trace (CALL/RET tree) |
| `-B N` | Ring-buffer size override (default: 50000 when `-T` used) |
| `-C cond` | Conditional trace (func:NAME, after:N, range:A-B, until:NAME) |
| `-A cond` | Trace assertion (a6=0, pc=0xADDR, mem:ADDR=V) |
| `-b addr` | Breakpoint (hex address or symbol name, repeatable) |
| `-m addr[:len]` | Dump memory on exit (repeatable) |
| `-v` | Verbose register dump on exit |
| `-e addr` | Override entry point (hex) |
| `-S file.img` | SD card backing image |
| `-Z bytes` | SD card size |
| `-1` | Single-core mode (disable APP_CPU/Core 1) |
| `--checkpoint-interval N` | Auto-save checkpoint every N cycles |
| `--checkpoint-dir PATH` | Directory for checkpoint files (default: `.`) |
| `--restore FILE` | Restore from checkpoint and resume execution |

## Trace Filter Tool

Post-processes verbose trace (`-T`) output:
```bash
# Generate trace to file
./build/xtensa-emu -q -T -s ELF -c CYCLES BIN >/dev/null 2>/tmp/trace.log

./build/trace-filter -u /tmp/trace.log          # Unregistered ROM calls
./build/trace-filter -e -c 5 -n /tmp/trace.log  # Exceptions with context
./build/trace-filter -p /tmp/trace.log           # Panic/abort path
./build/trace-filter -r /tmp/trace.log           # All ROM calls
./build/trace-filter -w /tmp/trace.log           # Window spill/underflow
./build/trace-filter -s funcname /tmp/trace.log  # Instructions in function
./build/trace-filter -R a1 /tmp/trace.log        # Track register changes
./build/trace-filter -A /tmp/trace.log           # All event types
```

## Source File Map

### Core emulator (`src/`)
| File | Purpose |
|------|---------|
| `xtensa.h/c` | CPU state, step/run, register access, windowed ops |
| `xtensa_decode.h` | Instruction field extraction macros |
| `xtensa_exec.c` | Instruction execution (switch-based) |
| `xtensa_disasm.c` | Disassembler |
| `memory.h/c` | Flat memory arrays (IRAM/DRAM/flash/PSRAM) |
| `loader.h/c` | ESP32 .bin segment loader |
| `peripherals.h/c` | MMIO peripheral dispatch + ESP32 stubs |
| `elf_symbols.h/c` | ELF symbol table loader (STT_FUNC + STT_OBJECT) |

### Stub modules (`src/`)
| File | Purpose |
|------|---------|
| `rom_stubs.h/c` | ROM function stubs (ets_printf, memcpy, etc.), PC hook mechanism, firmware symbol hooking, dual-core boot coordination |
| `freertos_stubs.h/c` | FreeRTOS task scheduler (cooperative + preemptive), dual-core support, queues, semaphores, task names, state save/restore |
| `esp_timer_stubs.h/c` | esp_timer and gettimeofday stubs |
| `display_stubs.h/c` | TFT_eSPI / eSprite display stubs |
| `touch_stubs.h/c` | XPT2046 touch input stubs |
| `sdcard_stubs.h/c` | SD card (FATFS) stubs with host file backing |
| `wifi_stubs.h/c` | lwip socket bridge to host TCP/IP (real network I/O) |
| `savestate.h/c` | Checkpoint save/restore system for CPU + memory + FreeRTOS state |

### Standalone tools (`src/`)
| File | Purpose |
|------|---------|
| `main.c` | Standalone firmware runner (all CLI flags, execution loop) |
| `xt_dis.c` | Standalone disassembler tool |
| `trace_filter.c` | Trace post-processor |

### Tests (`tests/`)
| File | Purpose |
|------|---------|
| `test_main.c` | All 459 unit tests |

## Architecture

- **Interpreter**: Switch-based decode, `xtensa_step()` (single) / `xtensa_run()` (batch)
- **Memory**: Flat arrays for IRAM(0x40000000)/DRAM(0x3FF00000)/flash/PSRAM, MMIO dispatch
- **PC Hook**: `cpu->pc_hook` callback fires before each instruction; ROM stubs use hash table for O(1) dispatch
- **Windows**: Synthesized spill/fill in C (no exception vector jump for perf)
- **Dual-core**: Full ESP32 dual-core support (PRO_CPU/Core 0 + APP_CPU/Core 1)
  - Both cores run in parallel with synchronized cycle counts
  - Core 1 starts when firmware releases it via `startup_resume_other_cores()`
  - Single-core mode available via `-1` flag (fakes `s_cpu_inited[1]=1` to skip Core 1)
  - Core 0 typically remains in `vTaskStartScheduler` loop while Core 1 executes tasks
- **Scheduler**: FreeRTOS preemptive round-robin with timeslice check after each batch
  - Dual-core aware: separate task queues per core
  - Preemption check runs after each batch for both cores
  - Task switching, delays, and synchronization primitives fully supported
- **Checkpoint/Restore**: Complete emulator state save/restore for long-duration testing
  - Automatic periodic checkpointing via `--checkpoint-interval N`
  - Full CPU state (all registers, windows, special regs) + memory (~14MB per checkpoint)
  - FreeRTOS scheduler state (tasks, queues, mutexes, timers) preserved
  - Enables debugging of rare crashes at billion+ cycle durations
- **Stub pattern**: Each module creates via `xxx_stubs_create(cpu)`, hooks symbols via `xxx_stubs_hook_symbols(stubs, syms)`

## Testing

### Unit Tests
```bash
cd flexe/build
./xtensa-tests        # Run 459 unit tests
```

### FreeRTOS Testing
Test firmware located in `../test-firmware/10-freertos-minimal/`:
```bash
# Quick validation (10M cycles)
./build/xtensa-emu -q -s ../test-firmware/10-freertos-minimal/build/freertos-minimal-test.elf \
    -c 10000000 ../test-firmware/10-freertos-minimal/build/freertos-minimal-test.bin

# Dual-core mode (default) - Core 1 executes tasks
./build/xtensa-emu -q -P 20000000 -s ../test-firmware/10-freertos-minimal/build/freertos-minimal-test.elf \
    -c 100000000 ../test-firmware/10-freertos-minimal/build/freertos-minimal-test.bin

# Single-core mode - All tasks on Core 0
./build/xtensa-emu -1 -q -P 20000000 -s ../test-firmware/10-freertos-minimal/build/freertos-minimal-test.elf \
    -c 100000000 ../test-firmware/10-freertos-minimal/build/freertos-minimal-test.bin

# Long-duration stress test with checkpoints (1B cycles = 6.25 sec @ 160MHz)
./build/xtensa-emu -q -P 100000000 -s ../test-firmware/10-freertos-minimal/build/freertos-minimal-test.elf \
    -c 1000000000 --checkpoint-interval 100000000 --checkpoint-dir ./checkpoints \
    ../test-firmware/10-freertos-minimal/build/freertos-minimal-test.bin
```

**Test Results** (as of 2026-03-17):
- ✅ 100M cycle dual-core test: PASSED
- ✅ Checkpoint creation: 5 checkpoints at 20M intervals
- ✅ Checkpoint restore: Successfully restored and continued execution
- ✅ Dual-core coordination: Core 0 in vTaskStartScheduler, Core 1 executing tasks
- 🔄 1B cycle stress test: In progress
- ⏸️ Single-core mode validation: Pending

## Coding Conventions

- C17, `-Wall -Wextra -Werror -Wno-unused-parameter`
- `snake_case` functions/vars, `UPPER_CASE` constants
- Public API: `xtensa_` prefix. Stub modules: `module_stubs_` prefix
- `<stdint.h>` fixed-width types. No malloc in hot path.
- Calling convention helpers: `xxx_arg(cpu, n)` reads windowed arg, `xxx_return(cpu, val)` returns + adjusts PC

## Key Reference Documents

- `../Xtensa.pdf` — Espressif "Overview of Xtensa ISA" (compact, 26 pages)
- `../isa-summary.pdf` — Cadence "Xtensa ISA Summary" (700+ pages, full spec)
- `../docs/xtensa-emulator-research.md` — ISA reference notes
