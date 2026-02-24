# flexe

**f**ree **l**ittle **x**tensa **e**mulator

a lightweight xtensa lx6 (esp32) emulator written in c. boots real esp-idf firmware binaries. no dependencies beyond a c11 compiler and cmake.

## what it does

flexe interprets the xtensa lx6 instruction set well enough to boot unmodified esp-idf applications. it handles the full init sequence — from reset vector through bootloader setup to `app_main`.

```
$ ./build/xtensa-emu -q -s build/hello_world.elf -c 5000000 build/hello_world.bin
I (0) cpu_start: Starting scheduler on PRO CPU.
Hello world!
```

### implemented

- full xtensa lx6 isa: alu, shifts, branches, loops, mac16, fpu
- windowed registers with synthesized spill/fill (call4/8/12, entry, retw)
- exception/interrupt dispatch (levels 1–7, timer ccompare, waiti)
- esp32 memory map (sram, rom, flash, rtc, psram, peripheral i/o)
- mmio peripherals: uart, gpio, dport, rtc, efuse, watchdog, timers, spi, i2c, ledc, adc
- rom function stubs: ets_printf, memcpy, memset, strlen, cache ops
- freertos stubs: tasks, queues, semaphores, delays
- esp_timer stubs with callback dispatch
- nvs flash stubs
- gpio driver stubs
- elf symbol loading, breakpoints, verbose trace mode
- 459 tests

## building

```
cmake -S . -B build
cmake --build build
```

produces two binaries:
- `build/xtensa-emu` — the emulator
- `build/xtensa-tests` — test suite

## usage

```
# basic run
./build/xtensa-emu firmware.bin

# with elf symbols and cycle limit
./build/xtensa-emu -s app.elf -c 10000000 firmware.bin

# quiet mode (suppress boot info)
./build/xtensa-emu -q -s app.elf -c 5000000 firmware.bin

# verbose trace to stderr
./build/xtensa-emu -T -s app.elf -c 1000000 firmware.bin 2>/tmp/trace.log
```

### flags

| flag | description |
|------|-------------|
| `-s ELF` | load elf for symbols and firmware hooks |
| `-c N` | stop after N cycles |
| `-q` | quiet (suppress emulator info, show only firmware output) |
| `-T` | verbose execution trace to stderr |
| `-b ADDR` | set breakpoint at address |
| `-d ADDR LEN` | hex dump memory region on exit |

### trace filter

a post-processing tool for verbose trace output:

```
./build/trace-filter -u trace.log    # unregistered rom calls
./build/trace-filter -e trace.log    # exceptions
./build/trace-filter -w trace.log    # window spill/fill events
./build/trace-filter -r trace.log    # all rom calls
./build/trace-filter -p trace.log    # panic/abort path
./build/trace-filter -s func trace.log  # instructions in a function
```

## architecture

switch-based interpreter. each step: fetch → decode → execute → loop check → interrupt check → advance ccount.

```
src/
  xtensa.c           interpreter core (~3500 lines, every isa instruction)
  xtensa.h           cpu state struct
  memory.c           address space: sram, rom, flash, psram, peripheral dispatch
  peripherals.c      mmio handlers for esp32 peripherals
  rom_stubs.c        pc-hook mechanism for rom + firmware function interception
  freertos_stubs.c   freertos task/queue/semaphore stubs
  esp_timer_stubs.c  esp_timer api stubs with callback dispatch
  loader.c           esp32 .bin + elf loading
  elf_symbols.c      elf symbol table parser
  xtensa_disasm.c    disassembler
  main.c             cli frontend
```

~15k lines of c total. see [ARCHITECTURE.md](ARCHITECTURE.md) for detailed design notes.

## testing

```
./build/xtensa-tests
# 459 tests, 774 passed, 0 failed
```

tests cover individual instructions, memory operations, windowed registers, exceptions, interrupts, peripherals, rom stubs, freertos, esp_timer, nvs, gpio driver, and end-to-end firmware compatibility.

## status

boots `hello_world`, `blink`, and `esp_timer` example apps from esp-idf. good enough for simple firmware. doesn't model timing, caches, or multicore.

## license

mit
