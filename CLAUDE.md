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

# With event log + heartbeat (lightweight observability)
./build/xtensa-emu -E -P 1000000 -q -s firmware.elf -c 100000000 firmware.bin

# NerdMiner specifically
./build/xtensa-emu -E -P 5000000 -q -s ../test-firmware/nerdminer.elf -c 100000000 ../test-firmware/nerdminer.bin
```

## CLI Flags

| Flag | Description |
|------|-------------|
| `-c N` | Max cycles (default: 10M) |
| `-q` | Quiet: suppress per-access unhandled peripheral warnings |
| `-s file.elf` | Load ELF symbols for trace/breakpoints |
| `-t` | Basic instruction trace to stderr |
| `-T` | Verbose trace (reg changes, ROM calls, exceptions) |
| `-T START:END` | Windowed trace: verbose only in cycle range, batch elsewhere |
| `-E` | Event log: stub calls, task switches, exceptions (fast, tiny output) |
| `-P N` | Progress heartbeat every N cycles |
| `-W` | Window trace (spill/fill/ENTRY/RETW events) |
| `-F` | Function-call trace (CALL/RET tree) |
| `-B N` | Ring-buffer trace: keep last N lines, dump on exit |
| `-C cond` | Conditional trace (func:NAME, after:N, range:A-B, until:NAME) |
| `-A cond` | Trace assertion (a6=0, pc=0xADDR, mem:ADDR=V) |
| `-b addr` | Breakpoint (hex address or symbol name, repeatable) |
| `-m addr[:len]` | Dump memory on exit (repeatable) |
| `-v` | Verbose register dump on exit |
| `-e addr` | Override entry point (hex) |
| `-S file.img` | SD card backing image |
| `-Z bytes` | SD card size |

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
| `rom_stubs.h/c` | ROM function stubs (ets_printf, memcpy, etc.), PC hook mechanism, firmware symbol hooking |
| `freertos_stubs.h/c` | FreeRTOS task scheduler (cooperative + preemptive), queues, semaphores, task names |
| `esp_timer_stubs.h/c` | esp_timer and gettimeofday stubs |
| `display_stubs.h/c` | TFT_eSPI / eSprite display stubs |
| `touch_stubs.h/c` | XPT2046 touch input stubs |
| `sdcard_stubs.h/c` | SD card (FATFS) stubs with host file backing |
| `wifi_stubs.h/c` | lwip socket bridge to host TCP/IP (real network I/O) |

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
- **Single core**: Only CPU 0 (PRO_CPU). Core 1 stubbed via `s_cpu_inited`
- **Scheduler**: FreeRTOS preemptive round-robin with timeslice check after each batch
- **Stub pattern**: Each module creates via `xxx_stubs_create(cpu)`, hooks symbols via `xxx_stubs_hook_symbols(stubs, syms)`

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
