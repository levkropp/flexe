# Milestones

Detailed acceptance criteria for each milestone. A milestone is complete when
all its acceptance criteria pass.

---

## M0: Project Scaffold + Build System

**Summary**: Empty project that builds, with all directories and headers in place.

**Acceptance criteria**:
- [ ] `cmake -S . -B build && cmake --build build` succeeds
- [ ] Produces `xtensa-emu` executable (or library + test runner)
- [ ] `xtensa.h` defines `xtensa_cpu_t` struct with all register state
- [ ] `xtensa.h` defines field extraction macros (`XT_OP0(insn)`, `XT_T(insn)`, etc.)
- [ ] `xtensa.h` defines `ar_read()` / `ar_write()` inline helpers
- [ ] `xtensa.h` defines all special register numbers as `#define XT_SR_*`
- [ ] `xtensa_cpu_init()` / `xtensa_cpu_reset()` functions exist and set initial state
- [ ] Test runner executes and reports 0 tests / 0 failures
- [ ] Compiles with `-Wall -Wextra -Werror` clean

**Files created**:
```
CMakeLists.txt
src/xtensa.h
src/xtensa.c          (stub: init/reset only)
src/memory.h
src/memory.c          (stub)
src/loader.h
src/loader.c          (stub)
tests/test_main.c     (test runner)
tests/test_helpers.h  (assert macros)
```

---

## M1: Instruction Decode + Disassembler

**Summary**: Can fetch, decode, and disassemble all Xtensa instruction formats.

**Acceptance criteria**:
- [ ] `xtensa_fetch()` correctly determines instruction length (2 or 3 bytes)
- [ ] All instruction fields extracted correctly for all 12 formats
- [ ] `xtensa_disasm()` produces human-readable assembly for:
  - All RRR core instructions
  - All RRI8 instructions (loads, stores, branches, ADDI, MOVI)
  - BRI12 instructions (BEQZ, BNEZ, BGEZ, BLTZ, LOOP variants)
  - CALL/CALLX format (J, JX, CALL0, RET, CALLn, CALLXn)
  - RSR/WSR/XSR (with symbolic register names)
  - RI16 (L32R)
  - All 16-bit narrow instructions
- [ ] `tools/xt-dis` reads a binary file + base address, outputs disassembly
- [ ] Disassembly of a real ESP32 .bin segment produces recognizable output
      (compared against `xtensa-esp32-elf-objdump -d`)
- [ ] Unit tests for tricky encodings: SLLI split field, MOVI 12-bit immediate,
      EXTUI shift+mask, ADDI.N imm=0 -> -1, MOVI.N 7-bit signed

**Files created/modified**:
```
src/xtensa.c          (fetch + decode + disasm)
src/xtensa_decode.h   (decode helpers, or inline in xtensa.c)
tools/xt-dis.c
tests/test_decode.c
```

---

## M2: Core ALU Interpreter

**Summary**: Can execute arithmetic, logic, shift, and move instructions.
CPU state changes are correct.

**Acceptance criteria**:
- [ ] `xtensa_step()` executes one instruction and advances PC
- [ ] All RRR arithmetic: ADD, SUB, ADDX2/4/8, SUBX2/4/8, NEG, ABS, SALT, SALTU
- [ ] All RRI8 arithmetic: ADDI, ADDMI
- [ ] All bitwise: AND, OR, XOR
- [ ] All shifts: SLL, SRL, SRA, SRC, SLLI, SRLI, SRAI, EXTUI,
      SSL, SSR, SSA8L, SSAI
- [ ] All moves: MOVI, MOVEQZ, MOVNEZ, MOVLTZ, MOVGEZ
- [ ] All narrow: ADD.N, ADDI.N, MOV.N, MOVI.N, NOP.N
- [ ] RSR/WSR/XSR for SAR works correctly
- [ ] NOP, ILL (raises illegal instruction flag), BREAK (sets debug flag)
- [ ] Unit tests: at least 3 tests per instruction covering normal case,
      edge case (overflow, zero, negative), and boundary values
- [ ] Test for EXTUI with various shift+mask combinations
- [ ] Test for SRC (funnel shift) with various SAR values

**Files modified**:
```
src/xtensa.c          (execute logic)
tests/test_alu.c
tests/test_shift.c
tests/test_move.c
```

---

## M3: Memory Subsystem

**Summary**: Load and store instructions work. Memory map matches ESP32.
.bin firmware files can be loaded.

**Acceptance criteria**:
- [ ] `memory_create()` allocates ESP32 address space
- [ ] `memory_read8/16/32()` and `memory_write8/16/32()` work
- [ ] Address ranges: IRAM, DRAM, flash-mapped, peripheral, RTC all dispatch correctly
- [ ] Unaligned 16-bit/32-bit access raises alignment flag (not crash)
- [ ] Peripheral region (0x3FF00000-0x3FF7FFFF) dispatches to peripheral handler
- [ ] `loader_load_bin()` parses ESP32 .bin format:
  - Reads magic byte 0xE9
  - Reads segment count
  - Reads entry point
  - Loads each segment to its specified address
  - Returns entry point
- [ ] L8UI, L16SI, L16UI, L32I, L32R work correctly
- [ ] S8I, S16I, S32I work correctly
- [ ] L32I.N, S32I.N (narrow) work correctly
- [ ] L32R: PC-relative negative offset works (test with known literal pool)
- [ ] Unit tests: load/store round-trip, boundary addresses, L32R offset calculation
- [ ] Integration test: load a real .bin, verify segments are at correct addresses

**Files modified**:
```
src/memory.c
src/loader.c
src/xtensa.c          (load/store instruction implementations)
tests/test_memory.c
tests/test_loader.c
```

---

## M4: Control Flow

**Summary**: Jumps, branches, and non-windowed calls work. Can execute linear
programs with loops and function calls.

**Acceptance criteria**:
- [ ] J (unconditional jump, PC-relative)
- [ ] JX (jump to register)
- [ ] CALL0 / CALLX0 / RET (non-windowed calling)
- [ ] All BRI8 conditional branches: BEQ, BNE, BGE, BLT, BGEU, BLTU,
      BEQI, BNEI, BGEI, BLTI, BGEUI, BLTUI,
      BALL, BNALL, BANY, BNONE, BBC, BBS, BBCI, BBSI
- [ ] All BRI12 branches: BEQZ, BNEZ, BGEZ, BLTZ
- [ ] Narrow branches: BEQZ.N, BNEZ.N
- [ ] RET.N
- [ ] B4const / B4constu tables used correctly by immediate branches
- [ ] Branch offset calculation correct (PC + sign_ext(imm) + 4)
- [ ] LOOP, LOOPGTZ, LOOPNEZ (zero-overhead loop)
  - LBEG, LEND, LCOUNT set correctly
  - Loop-back fires when PC == LEND and LCOUNT != 0
  - LOOPGTZ skips when AR[s] <= 0
  - LOOPNEZ skips when AR[s] == 0
- [ ] Multiply: MUL16S, MUL16U, MULL, MULUH, MULSH
- [ ] Divide: QUOS, QUOU, REMS, REMU (including div-by-zero)
- [ ] Misc ops: MIN, MAX, MINU, MAXU, NSA, NSAU, SEXT, CLAMPS
- [ ] S32C1I (compare-and-swap with SCOMPARE1)
- [ ] Integration test: a Fibonacci program runs and produces correct results
- [ ] Integration test: a bubble sort program runs correctly

**Files modified**:
```
src/xtensa.c          (branch, call, loop, mul/div, misc implementations)
tests/test_branch.c
tests/test_loop.c
tests/test_muldiv.c
tests/test_misc.c
tests/test_programs/fibonacci.S  (or .bin)
```

---

## M5: Windowed Registers

**Summary**: Full windowed calling convention works. Deep call chains with
automatic overflow/underflow handling.

**Acceptance criteria**:
- [ ] CALL4, CALL8, CALL12 (PC-relative windowed calls)
- [ ] CALLX4, CALLX8, CALLX12 (register-indirect windowed calls)
- [ ] ENTRY (window rotate + stack frame allocation)
- [ ] RETW / RETW.N (windowed return, reads call size from a0[31:30])
- [ ] PS.CALLINC set correctly by each CALL variant
- [ ] WINDOWBASE incremented by 1/2/3 for CALL4/8/12
- [ ] WINDOWBASE decremented correctly on RETW
- [ ] WINDOWSTART updated on ENTRY (mark new window valid)
- [ ] WINDOWSTART updated on RETW (mark returned window invalid — or not,
      depending on spill state)
- [ ] Window overflow: detected when CALL/ENTRY would overlap a valid window.
      Registers spilled to stack correctly.
- [ ] Window underflow: detected when RETW returns to an invalid window.
      Registers restored from stack correctly.
- [ ] MOVSP with overflow trigger
- [ ] ROTW (used in exception handlers)
- [ ] L32E / S32E (window exception handler memory access)
- [ ] RSR/WSR/XSR for WINDOWBASE, WINDOWSTART, PS
- [ ] Test: 8-deep CALL4 chain (fills all 16 windows), verify overflow triggers
- [ ] Test: CALL8 chain that wraps around, verify spill/fill
- [ ] Test: mixed CALL4/CALL8/CALL12 chain
- [ ] Test: RETW after overflow, verify registers restored correctly
- [ ] Test: recursive function (factorial) using windowed calls

**Files created/modified**:
```
src/window.c
src/window.h
src/xtensa.c          (windowed call/return, ENTRY integration)
tests/test_window.c
```

---

## M6: Exceptions + Interrupts

**Summary**: Exception vectors dispatch correctly. Timer interrupts fire.
PS management is complete.

**Acceptance criteria**:
- [ ] Exception dispatch: saves EPC[1], EPS[1], sets EXCCAUSE, jumps to vector
- [ ] Exception vectors at VECBASE + offset (offsets per cause)
- [ ] RFE (return from exception level 1): restores PS from EPS, PC from EPC
- [ ] RFDE (return from double exception)
- [ ] Level-1 interrupt dispatch (similar to exceptions)
- [ ] High-priority interrupt dispatch (levels 2-6): save to EPC[n]/EPS[n]
- [ ] RFI (return from high-priority interrupt)
- [ ] NMI (level 7)
- [ ] CCOUNT increments (1 per instruction or configurable)
- [ ] Timer interrupt: fires when CCOUNT == CCOMPARE[n]
- [ ] INTENABLE / INTSET / INTCLEAR register handling
- [ ] WAITI (halt until interrupt)
- [ ] Interrupt priority: higher levels preempt lower
- [ ] PS fields fully managed: INTLEVEL, EXCM, UM, RING, WOE, CALLINC, OWB
- [ ] RSIL instruction (read and set interrupt level)
- [ ] SYSCALL instruction (EXCCAUSE = 1)
- [ ] RSR/WSR/XSR for: EPC[1-7], EPS[2-7], EXCSAVE[1-7], EXCCAUSE,
      EXCVADDR, DEPC, VECBASE, CCOUNT, CCOMPARE[0-2], INTENABLE, PS
- [ ] Test: set up timer interrupt, verify it fires
- [ ] Test: nested interrupts (level 1 interrupted by level 3)
- [ ] Test: exception in exception handler -> double exception

**Files created/modified**:
```
src/exception.c
src/exception.h
src/interrupt.c
src/interrupt.h
src/xtensa.c          (exception/interrupt integration in main loop)
tests/test_exception.c
tests/test_interrupt.c
```

---

## M7: ESP-IDF Bootloader

**Summary**: Loading an ESP-IDF .bin causes the second-stage bootloader to run
and reach the application's `app_main()` function.

**Acceptance criteria**:
- [ ] DPORT peripheral stub returns sensible clock config values
- [ ] RTC_CNTL stub reports POWERON reset, clocks ready
- [ ] UART0 stub captures TX bytes -> emulator console
- [ ] TIMG0/TIMG1 watchdog stubs: always report "not triggered", accept disable
- [ ] EFUSE stub returns chip revision 1, fake MAC address
- [ ] ROM function intercepts: ets_printf -> capture output,
      Cache_Read_Enable -> enable flash mapping, other critical funcs
- [ ] Flash mapping: .bin flash image accessible at instruction + data flash
      address ranges
- [ ] Bootloader runs (ESP-IDF v4.4 or v5.x hello_world project)
- [ ] `app_main()` is called (verified by UART output or PC trace)
- [ ] No crashes or infinite loops in boot sequence
- [ ] Unimplemented instruction/peripheral hit rate < 5 (most things handled)

**Files created/modified**:
```
src/peripherals/dport.c
src/peripherals/rtc.c
src/peripherals/uart.c
src/peripherals/timer.c
src/peripherals/efuse.c
src/peripherals/intmatrix.c
src/memory.c           (ROM function intercept hooks)
```

---

## M8: FreeRTOS Scheduler

**Summary**: FreeRTOS boots, tasks are created and scheduled, vTaskDelay works.

**Acceptance criteria**:
- [ ] FreeRTOS `vTaskStartScheduler()` completes without crash
- [ ] Tick interrupt fires at configured rate
- [ ] `xTaskCreate()` creates a task that gets scheduled
- [ ] `vTaskDelay()` suspends and resumes correctly
- [ ] Task stack usage is correct (window spills land in right place)
- [ ] S32C1I works correctly for FreeRTOS spinlocks
- [ ] Single-core mode: core 1 startup is stubbed (PRID = 0 for core 0)
- [ ] Idle task runs when no other tasks are ready
- [ ] Multiple tasks (3+) context-switch correctly
- [ ] ESP-IDF `hello_world` example prints "Hello world!" via UART and loops
- [ ] No stack corruption or window register leaks after 10000+ context switches

**Files modified**:
```
src/xtensa.c           (any remaining instruction fixes)
src/peripherals/timer.c (tick interrupt timing)
src/interrupt.c         (context switch interrupt handling)
```

---

## M9: Peripheral Models

**Summary**: SPI display, touch input, and GPIO work. Visual output appears.

**Acceptance criteria**:
- [ ] SPI master controller model accepts register writes
- [ ] ILI9341 display model:
  - Accepts initialization sequence (SLPOUT, DISPON, CASET, RASET, RAMWR)
  - Receives pixel data via RAMWR
  - Maintains 320x240x16bpp framebuffer
  - Column/row address windowing works
- [ ] XPT2046 touch model:
  - Responds to SPI read commands
  - Returns coordinates from host input
  - Pressure/no-touch detection
- [ ] GPIO output: basic LED toggle visible in emulator log
- [ ] Display framebuffer accessible via API for host rendering
- [ ] Test with a simple ESP-IDF SPI display program
- [ ] Test with touch input example

**Files created/modified**:
```
src/peripherals/spi.c
src/peripherals/display.c
src/peripherals/touch.c
src/peripherals/gpio.c
src/peripherals/i2c.c
```

---

## M10: cyd-emulator Integration

**Summary**: xtensa-emulator embedded in cyd-emulator. Load .bin from menu,
see display output, interact with touch.

**Acceptance criteria**:
- [ ] xtensa-emulator builds as `libxtensa-emu.a` static library
- [ ] Clean C API (create, load, step, get_framebuffer, touch_event, destroy)
- [ ] cyd-emulator links against libxtensa-emu
- [ ] File > Load Firmware (.bin)... opens file dialog, loads firmware
- [ ] Emulator runs in background thread at ~240 MIPS
- [ ] Display output renders to SDL window (320x240, same as current)
- [ ] Touch input from SDL mouse events feeds into emulator
- [ ] UART output appears in side panel log
- [ ] Can pause/resume emulator execution
- [ ] Can restart emulator (reload .bin)
- [ ] Coexists with existing C-compilation mode (user chooses which mode)
- [ ] Frame rate stable at 30+ FPS

**Files modified**:
```
cyd-emulator/CMakeLists.txt    (add xtensa-emulator dependency)
cyd-emulator/src/emu_main.c    (add Load Firmware menu item, emulator thread)
cyd-emulator/src/emu_xtensa.c  (new: bridge between xtensa-emu and SDL)
cyd-emulator/src/emu_xtensa.h
```

---

## M11: Emulator-Targeted Compilation

**Summary**: Users can build an ESP-IDF project and run it directly in the emulator.

**Acceptance criteria**:
- [ ] Documented workflow: `idf.py build` -> load `build/project.bin` in emulator
- [ ] Helper script that automates: build -> extract .bin -> launch emulator
- [ ] Alternative: `cmake` target in cyd-emulator that builds the ESP-IDF project
      and creates a combined emulator+firmware binary
- [ ] Works with ESP-IDF v4.4 and v5.x projects
- [ ] Example CYD project compiles and runs in emulator with display output
- [ ] Documentation: README with setup instructions, requirements, examples

**Files created**:
```
tools/emu-run.sh           (build + load helper)
docs/getting-started.md
docs/building-firmware.md
```
