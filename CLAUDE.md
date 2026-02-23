# Xtensa Emulator - Project Context

MIT-licensed Xtensa LX6 CPU emulator for running ESP32 .bin firmware files.
Sister project to [cyd-emulator](../cyd-emulator).

## Quick Links

- `PLAN.md` — High-level plan, phased execution, design decisions, risk register
- `MILESTONES.md` — M0-M11 milestone definitions with acceptance criteria
- `ARCHITECTURE.md` — Memory model, decode pipeline, window strategy, peripheral arch, API
- `TASKS-M0.md` — Granular task breakdown for the current milestone
- `../cyd-emulator/docs/xtensa-emulator-research.md` — ISA reference (instruction
  encodings, register map, ESP32 memory map, all extracted from official docs)

## Key Reference Documents

- `../cyd-emulator/Xtensa.pdf` — Espressif "Overview of Xtensa ISA" (compact, 26 pages,
  has encoding tables for every instruction)
- `../cyd-emulator/isa-summary.pdf` — Cadence "Xtensa ISA Summary" (700+ pages,
  full formal specification including all options)
- ESP32 Technical Reference Manual (online) for peripheral register definitions

## Coding Conventions

- **Language**: C17 (`-std=c17`)
- **Compiler flags**: `-Wall -Wextra -Werror -Wno-unused-parameter`
- **Naming**: `snake_case` for functions and variables, `UPPER_CASE` for constants/macros
- **Prefix**: Public API uses `xtensa_` prefix. Internal helpers use no prefix.
- **Inline**: Field extraction macros in `xtensa_decode.h`, register access helpers
  as `static inline` in `xtensa.h`
- **Types**: Use `<stdint.h>` fixed-width types (`uint32_t`, `int32_t`, etc.)
- **No dynamic allocation in hot path**: CPU state and memory are pre-allocated.
  No `malloc` during instruction execution.
- **Comments**: Only where logic is non-obvious. Instruction implementations should
  reference the ISA manual section (e.g., `/* ISA p.326: ADD */`).

## Build

```bash
cmake -S . -B build
cmake --build build
./build/xtensa-emu          # Run emulator
./build/xtensa-tests        # Run test suite
./build/xt-dis <file> [addr] # Disassemble binary
```

## Current Status

Track progress via milestones in `MILESTONES.md`. Check task breakdowns in
`TASKS-M*.md` files for the current work.

## Architecture Summary

- **Interpreter**: Switch-based decode, one instruction per `xtensa_step()` call
- **Memory**: Flat arrays for RAM/ROM/flash, function dispatch for peripherals
- **Windows**: Synthesized spill/fill in C (no exception vector jump for perf)
- **Single core**: Only emulates CPU 0 (PRO_CPU). Core 1 is stubbed.
- **No FP initially**: Floating-point instructions deferred until needed
- **Integration**: Builds as `libxtensa-emu.a`, clean C API for cyd-emulator
