# Xtensa Emulator - Plan of Plans

MIT-licensed Xtensa LX6 CPU emulator for running ESP32 .bin firmware files.

Two usage modes:
1. **Binary firmware loading** - Load pre-compiled ESP-IDF .bin files and execute them
2. **Emulator-targeted compilation** - Compile ESP-IDF projects to run directly on the
   emulator (with a custom ESP-IDF port/toolchain shim)

End goal: integrate with [cyd-emulator](../cyd-emulator) so users can either load
a .bin firmware or compile their CYD project for the emulator with full display/touch.

---

## Project Structure

```
xtensa-emulator/
  src/
    xtensa.h            # CPU state, constants, inline helpers
    xtensa.c            # Instruction interpreter (fetch/decode/execute)
    memory.h            # Memory bus (dispatch to RAM/ROM/peripherals)
    memory.c
    loader.h            # .bin firmware parser + ELF loader
    loader.c
    peripherals/
      uart.c            # UART0/1/2 (console I/O)
      gpio.c            # GPIO matrix
      timer.c           # TIMG0/TIMG1 watchdog + general-purpose timers
      spi.c             # SPI controllers (flash + display)
      i2c.c             # I2C
      dport.c           # DPORT system registers
      rtc.c             # RTC_CNTL (clock, reset cause, power)
      efuse.c           # eFuse (chip ID, MAC)
      intmatrix.c       # Interrupt matrix (routing peripheral IRQs to CPU)
      display.c         # ILI9341 SPI display model
      touch.c           # XPT2046 touch controller model
    interrupt.h         # Interrupt controller logic
    interrupt.c
    exception.h         # Exception dispatch (vectors, cause codes)
    exception.c
    window.h            # Register window overflow/underflow handlers
    window.c
  tests/
    test_alu.c          # Unit tests for arithmetic/logic instructions
    test_branch.c       # Branch instruction tests
    test_memory.c       # Load/store tests
    test_window.c       # Register window tests
    test_loader.c       # .bin format parsing tests
    test_programs/      # Hand-written Xtensa assembly test programs
      hello.S
      fibonacci.S
      window_call.S
  tools/
    xt-dis.c            # Standalone Xtensa disassembler (useful for debugging)
  CMakeLists.txt
  PLAN.md               # This file
  MILESTONES.md         # Milestone definitions and acceptance criteria
  ARCHITECTURE.md       # Detailed architecture decisions
```

---

## Milestones Overview

| # | Milestone | Gate | Dependencies |
|---|-----------|------|--------------|
| M0 | Project scaffold + build system | `cmake --build build` produces empty executable | None |
| M1 | Instruction decode + disassembler | Can disassemble real ESP32 .bin segments | M0 |
| M2 | Core ALU interpreter | Passes arithmetic/logic/shift/move unit tests | M1 |
| M3 | Memory subsystem | Loads/stores work; .bin loader places segments | M2 |
| M4 | Control flow | Jumps, branches, CALL0/RET work; runs flat (non-windowed) code | M3 |
| M5 | Windowed registers | CALL4/8/12, ENTRY, RETW, window overflow/underflow | M4 |
| M6 | Exceptions + interrupts | Exception vectors, PS management, timer interrupts | M5 |
| M7 | ESP-IDF bootloader | Second-stage bootloader runs, app_main() reached | M6 |
| M8 | FreeRTOS scheduler | Tasks created, context switches work, vTaskDelay returns | M7 |
| M9 | Peripheral models | UART output, GPIO, SPI display, touch input work | M8 |
| M10 | cyd-emulator integration | Load .bin from File menu, display renders via SDL | M9 |
| M11 | Emulator-targeted compilation | ESP-IDF project compiles + runs on emulator directly | M10 |

See [MILESTONES.md](MILESTONES.md) for detailed descriptions and acceptance criteria.

---

## Phased Execution Plan

### Phase 1: Foundation (M0-M2)

**Goal**: Standalone executable that can decode and execute individual Xtensa
instructions in isolation.

Work items:
- [ ] CMake project with `src/`, `tests/`, C17 standard, `-Wall -Werror`
- [ ] `xtensa.h`: CPU state struct (`xtensa_cpu_t`), field extraction macros,
      register window helpers (`ar_read`/`ar_write`)
- [ ] `xtensa.c`: Instruction fetch (16/24-bit detection), field extraction,
      top-level decode switch on `op0`
- [ ] Implement all RRR-format core instructions (ADD, SUB, AND, OR, XOR,
      shifts, moves, EXTUI, etc.)
- [ ] Implement RRI8-format instructions (ADDI, ADDMI, MOVI, loads, stores,
      all conditional branches)
- [ ] Implement BRI12 branches (BEQZ, BNEZ, BGEZ, BLTZ)
- [ ] Implement CALL/CALLX format (J, JX, CALL0, CALLX0, RET) - non-windowed only
- [ ] Implement RI16 (L32R)
- [ ] Implement RSR/WSR/XSR for SAR
- [ ] All 16-bit Code Density instructions
- [ ] B4const / B4constu lookup tables
- [ ] `tools/xt-dis.c`: Standalone disassembler that reads a binary blob and prints
      assembly
- [ ] Unit test framework (simple C assert macros, no dependencies)
- [ ] Test programs: hand-assemble or use `xtensa-esp32-elf-as` if available

### Phase 2: Memory + Control Flow (M3-M4)

**Goal**: Execute linear programs that use memory and non-windowed function calls.

Work items:
- [ ] `memory.c`: ESP32 memory map implementation
  - Sparse memory model (only allocate pages that are accessed)
  - Address dispatch: IRAM, DRAM, flash-mapped, peripheral, RTC
  - Alignment checking for L16/L32/S16/S32
- [ ] `loader.c`: Parse ESP32 .bin format (magic 0xE9, segment headers)
  - Load segments to correct addresses
  - Extract entry point
  - Handle extended header (v2 format)
- [ ] ELF loader (for emulator-targeted builds later)
- [ ] Connect loader -> memory -> CPU, run first real code
- [ ] Implement NOP, ILL, BREAK, MEMW, EXTW, ISYNC, RSYNC, ESYNC, DSYNC, FSYNC
      as no-ops (barriers are meaningless in a serial interpreter)
- [ ] Loop option: LOOP, LOOPGTZ, LOOPNEZ + loop-back check in main loop
- [ ] Multiply: MUL16S, MUL16U, MULL, MULUH, MULSH
- [ ] Divide: QUOS, QUOU, REMS, REMU (with div-by-zero exception)
- [ ] Misc ops: MIN, MAX, MINU, MAXU, NSA, NSAU, SEXT, CLAMPS
- [ ] S32C1I (compare-and-swap)
- [ ] Test: load an ESP32 .bin, start executing from entry point, trace first ~100
      instructions, verify against expected disassembly

### Phase 3: Windowed Registers (M5)

**Goal**: Full windowed calling convention works. Functions can call other functions
using CALL4/8/12 with automatic register rotation.

Work items:
- [ ] `window.c`: Window overflow check on CALL/ENTRY
  - Check `windowstart` bitmask for overlap
  - Trigger overflow exception when needed
  - Spill registers to stack via emulated exception handler (or synthesized spill)
- [ ] Window underflow on RETW
  - Detect when returning to an invalid window
  - Trigger underflow exception to restore registers from stack
- [ ] Implement CALL4, CALL8, CALL12, CALLX4, CALLX8, CALLX12
- [ ] Implement ENTRY (stack frame allocation + window rotate)
- [ ] Implement RETW, RETW.N
- [ ] Implement MOVSP (with overflow check)
- [ ] Implement ROTW, RFWO, RFWU (exception handler helpers)
- [ ] Implement L32E, S32E (exception handler memory access)
- [ ] PS register: CALLINC, WOE, EXCM, INTLEVEL, UM, RING fields
- [ ] RSR/WSR/XSR for WINDOWBASE, WINDOWSTART, PS
- [ ] Decision: emulate window exceptions fully (run the vector code) vs. synthesize
      spill/fill directly in C. Full emulation is more correct but slower; synthesis
      is faster and avoids needing ROM code. Start with synthesis, add full emulation
      if needed.
- [ ] Test: function call chains 8+ deep that trigger overflow, verify registers
      are correctly saved and restored

### Phase 4: Exceptions + Interrupts (M6)

**Goal**: Exception vectors work. Timer interrupts fire. Enough to boot ESP-IDF.

Work items:
- [ ] `exception.c`: Exception dispatch
  - Save PC to EPC[level], PS to EPS[level]
  - Set PS.EXCM, PS.INTLEVEL
  - Jump to VECBASE + offset based on cause
  - Exception causes: IllegalInstruction, Syscall, LoadStoreAlignment,
    IntegerDivideByZero, WindowOverflow4/8/12, WindowUnderflow4/8/12
- [ ] `interrupt.c`: Interrupt handling
  - Level-1 interrupts (software-triggered, edge/level)
  - Timer compare interrupts (CCOUNT vs CCOMPARE)
  - Interrupt matrix: route peripheral IRQs to CPU interrupt lines
  - WAITI instruction
  - RSIL (read and set interrupt level)
- [ ] Implement RFE, RFI, RFDE (return from exception/interrupt)
- [ ] CCOUNT incrementing strategy: +1 per instruction (simple) or +N per
      instruction class (more accurate but complex). Start with +1.
- [ ] SYSCALL instruction (EXCCAUSE = 1)
- [ ] RSR/WSR for all exception-related SRs: EPC, EPS, EXCSAVE, EXCCAUSE,
      EXCVADDR, DEPC, VECBASE, CCOUNT, CCOMPARE, INTENABLE, INTSET, INTCLEAR
- [ ] Test: set up a timer interrupt, verify it fires and returns correctly

### Phase 5: ESP-IDF Boot (M7)

**Goal**: ESP-IDF second-stage bootloader completes, jumps to app_main().

Work items:
- [ ] Peripheral stubs (return sensible defaults):
  - `dport.c`: Clock config reads, cache control, memory mapping registers
  - `rtc.c`: RTC_CNTL - reset cause (return POWERON), clock ready flags
  - `uart.c`: TX data register (capture output to emulator console)
  - `timer.c`: TIMG0/1 watchdog - always return "not triggered", WDT disable
  - `efuse.c`: Chip revision, MAC address (return fake values)
- [ ] ESP32 ROM function table: The bootloader calls ROM functions at fixed
      addresses (e.g., ets_printf, Cache_Read_Enable). Need to intercept these.
      Strategy: place HLT-like traps at ROM function addresses, handle in the
      emulator's illegal-instruction handler as "ROM call traps".
- [ ] Flash emulation: Map a flash image file at 0x3F400000 (data) and
      0x400C2000 (instruction) so the bootloader can read partition table
      and app image.
- [ ] Boot sequence: Reset vector at 0x40000400 -> ROM boot -> 2nd stage
      bootloader -> app startup. May need to skip ROM boot entirely and
      jump directly to the 2nd stage bootloader entry point.
- [ ] Iterative debugging: Run, hit unimplemented instruction/peripheral,
      add stub, repeat. This phase is largely empirical.

### Phase 6: FreeRTOS (M8)

**Goal**: FreeRTOS scheduler runs. Tasks can be created. vTaskDelay works.

Work items:
- [ ] S32C1I must work correctly (FreeRTOS spinlocks depend on it)
- [ ] Timer interrupts must fire reliably (tick interrupt)
- [ ] Context switch: FreeRTOS saves/restores register windows via
      window spill/fill. The emulator's window mechanism must be solid.
- [ ] Dual-core stub: ESP32 has two cores. FreeRTOS on ESP-IDF uses both.
      Strategy: emulate only PRO_CPU (core 0), stub out APP_CPU (core 1).
      Set PRID=0, make core 1 initialization a no-op.
- [ ] Cross-core interrupt: stub as no-op (since core 1 is not running)
- [ ] Test: FreeRTOS hello_world example reaches `app_main()`, creates a
      task, task runs and prints to UART

### Phase 7: Peripherals (M9)

**Goal**: SPI display and touch input work through the emulator.

Work items:
- [ ] `spi.c`: SPI master controller model
  - Register interface matching ESP32 SPI peripheral
  - Transaction-based: when MOSI data is written and CMD bit set,
    dispatch to connected SPI device model
  - Connect to display model (ILI9341) and touch model (XPT2046)
- [ ] `display.c`: ILI9341 SPI display model
  - Accept commands: CASET, RASET, RAMWR, SLPOUT, DISPON, etc.
  - Maintain framebuffer (320x240x16bpp)
  - Expose framebuffer to host for SDL rendering
- [ ] `touch.c`: XPT2046 touch controller model
  - Accept touch coordinate queries
  - Feed in coordinates from host (SDL mouse events)
- [ ] `gpio.c`: GPIO matrix model
  - Pin mux routing (connect SPI signals to pins)
  - Basic output (LED control)
- [ ] `i2c.c`: I2C master (if needed for specific CYD variants)
- [ ] `intmatrix.c`: Route peripheral interrupts to CPU lines

### Phase 8: cyd-emulator Integration (M10)

**Goal**: The xtensa-emulator library is embedded in cyd-emulator. Users can
load .bin firmware from the File menu and see it running.

Work items:
- [ ] Build xtensa-emulator as a static library (`libxtensa-emu.a`)
- [ ] Define clean C API:
  ```c
  xtensa_emu_t *xtensa_emu_create(void);
  int xtensa_emu_load_bin(xtensa_emu_t *emu, const char *path);
  int xtensa_emu_step(xtensa_emu_t *emu, int cycles);
  uint16_t *xtensa_emu_get_framebuffer(xtensa_emu_t *emu);
  void xtensa_emu_touch_event(xtensa_emu_t *emu, int x, int y, bool down);
  void xtensa_emu_destroy(xtensa_emu_t *emu);
  ```
- [ ] Integrate into cyd-emulator's CMakeLists.txt as a subdirectory or
      FetchContent dependency
- [ ] Add "Load Firmware (.bin)..." to File menu
- [ ] Run emulator in a background thread (same pattern as current app thread)
- [ ] Bridge display: emulator framebuffer -> SDL texture (same as current
      emu_display approach)
- [ ] Bridge touch: SDL mouse events -> emulator touch input
- [ ] Bridge UART: emulator UART output -> panel log
- [ ] Performance tuning: run emulator at ~240 MIPS, sync display at 30fps

### Phase 9: Emulator-Targeted Compilation (M11)

**Goal**: Users can compile an ESP-IDF project to run directly on the emulator
without needing real hardware, using a shim/HAL layer.

Work items:
- [ ] Create a custom ESP-IDF "board" / "target" for the emulator
  - Option A: Custom HAL that replaces low-level ESP-IDF hardware access
    with direct emulator calls (e.g., `xt_emu_write_display()`)
  - Option B: Compile for xtensa normally but link against emulator-provided
    ROM/peripheral stubs. This is essentially what .bin loading does, so this
    option is less useful.
  - Option C: Compile to native x86/ARM using the cyd-emulator's existing
    approach (C shim headers), but enhanced with the xtensa emulator's
    peripheral models for higher fidelity. This is a middle ground.
- [ ] The most practical approach is likely: keep the existing C-compilation
      approach for rapid development, and use the Xtensa emulator for
      validation/testing of actual firmware images. The emulator-targeted
      compilation would then be: "compile your ESP-IDF project normally,
      flash to .bin, load in emulator".
- [ ] Alternative: provide a `xtensa-esp32-elf-gcc` wrapper that compiles
      and automatically loads into the emulator for testing, similar to how
      `wokwi-server` works.
- [ ] Document the workflow: `idf.py build` -> `xtensa-emulator load build/app.bin`

---

## Key Design Decisions

### 1. Interpreter vs JIT

**Decision: Interpreter first.** A switch-based interpreter is simpler, portable,
and sufficient for ~100 MIPS performance. JIT can be added later if needed.

Optimization path: `switch` -> computed `goto` (threaded) -> basic-block cache ->
full JIT. Each step roughly doubles throughput.

### 2. Memory Model

**Decision: Flat array with page-fault-on-access.**

Allocate a 4 GB virtual address space using `mmap(MAP_NORESERVE)` on Linux. Pages
are allocated on first access by the OS. This gives O(1) address translation with
no bounds checking needed, at the cost of a 4 GB virtual address reservation (no
physical memory is used until pages are touched).

Fallback for platforms that don't support this: sparse page table (hash map from
page number to page pointer).

Peripheral addresses (0x3FF00000-0x3FF7FFFF) are intercepted before the memory
access and dispatched to peripheral handlers.

### 3. Window Overflow Strategy

**Decision: Synthesized spill/fill (not full exception emulation).**

When a window overflow is detected, the emulator directly saves the registers to
the stack in C code, without jumping to the exception vector. This is faster and
avoids needing ESP32 ROM code. If a firmware image provides custom window exception
handlers (unlikely for ESP-IDF apps), switch to full vector emulation.

### 4. Dual Core

**Decision: Single core only (core 0).** ESP-IDF can run in single-core mode.
Many apps don't meaningfully use core 1. Stub out core 1 startup and cross-core
interrupts.

### 5. Floating Point

**Decision: Defer.** ESP32 has single-precision FP, but most CYD firmware doesn't
use it heavily. Implement FP instructions only when a real firmware needs them.
Use host FP hardware (just cast to/from `float`).

### 6. License

**MIT.** All code written from scratch using only the ISA specification documents
(which describe a published instruction set architecture). No GPL-contaminated
code (QEMU, Unicorn) is referenced or used.

---

## Risk Register

| Risk | Likelihood | Impact | Mitigation |
|------|-----------|--------|------------|
| Window exception handling is wrong | High | High | Extensive testing with deep call chains; compare traces against QEMU if needed (black-box comparison, no code copying) |
| ESP-IDF boot requires undocumented ROM functions | High | Medium | Intercept ROM calls with stubs; use ESP-IDF ROM function tables (Apache-2.0) to identify what each address does |
| Peripheral register behavior is underspecified | Medium | Medium | Use ESP32 Technical Reference Manual; test against real hardware behavior via logic analyzer captures |
| Performance too slow for real-time display | Low | Medium | Threaded interpreter + skip idle loops. ESP32 at 240MHz is modest by emulation standards. |
| ESP-IDF version incompatibility | Medium | Low | Target ESP-IDF v4.4 LTS initially; newer versions may change bootloader behavior |

---

## Timeline Estimate

Not providing fixed dates, but rough ordering of effort:

- **M0-M2** (Foundation): First deliverable. Standalone interpreter + disassembler.
- **M3-M5** (Memory + Windows): Second deliverable. Can execute real functions.
- **M6-M7** (Exceptions + Boot): Third deliverable. ESP-IDF bootloader runs.
- **M8-M9** (FreeRTOS + Peripherals): Fourth deliverable. Full firmware runs.
- **M10** (Integration): Fifth deliverable. Works inside cyd-emulator.
- **M11** (Compilation target): Stretch goal.

Each milestone builds on the previous. The project is designed so that each
milestone produces a testable, useful artifact.
