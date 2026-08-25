# flexe

**f**ree **l**ittle **x**tensa **e**mulator

a lightweight xtensa lx6 (esp32) emulator written in c. boots real esp-idf firmware binaries. no dependencies beyond a c11 compiler, cmake, and openssl.

oh, and it's **fast**. like, stupidly fast. there's a jit in here now.

## the numbers

a real esp32 lx6 hums along at 240 mhz. that's the bar. here's flexe on apple silicon (pgo build, 500M-1B cycle workloads):

| workload | interpreter | jit (default) | vs real esp32 |
|---|---:|---:|---:|
| tjpgd (jpeg decode) | 330 mips | **693 mips** | 2.9× faster |
| real_time_stats (dual-core freertos) | 334 mips | **499 mips** | 2.1× faster |
| cpu_bench (alu/mem loops) | 318 mips | **2476 mips** | 10.3× faster |

so yeah — flexe emulates an esp32 at 2–10× the speed of an actual esp32. the jit traces hot basic blocks through branches, chains them into long native runs, and constant-folds literal loads. cold code falls back to the interpreter, which itself is already above real-time.

```
$ ./build/xtensa-emu -q -s build/hello_world.elf -c 5000000 build/hello_world.bin
I (0) cpu_start: Starting scheduler on PRO CPU.
Hello world!
```

## what it does

flexe interprets (and now jits) the xtensa lx6 instruction set well enough to boot unmodified esp-idf applications. it handles the full init sequence — from reset vector through bootloader setup to `app_main`.

### implemented

- full xtensa lx6 isa: alu, shifts, branches, loops, mac16, fpu
- windowed registers with synthesized spill/fill (call4/8/12, entry, retw)
- exception/interrupt dispatch (levels 1–7, timer ccompare, waiti)
- esp32 memory map (sram, rom, flash, rtc, psram, peripheral i/o)
- mmio peripherals: uart, gpio, dport, rtc, efuse, watchdog, timers, spi, i2c, ledc, adc
- raw hardware crypto: aes-128/192/256, sha, rsa modular math and interrupts
- cyd devices: ili9341 display, xpt2046 touch, sd/fat and spiffs storage
- host-backed lwip sockets plus virtual wifi, bluetooth, and phy boundaries
- rom function stubs: ets_printf, memcpy, memset, strlen, cache ops
- freertos stubs: tasks, queues, semaphores, delays
- esp_timer stubs with callback dispatch
- nvs flash stubs
- gpio driver stubs
- elf symbol loading, breakpoints, verbose trace mode
- jit compiler: hot blocks → native code (arm64 + x86-64), on by default
- 542 tests

## building

```
cmake -S . -B build
cmake --build build -j
```

(macOS: point cmake at homebrew openssl with `-DOPENSSL_ROOT_DIR=$(brew --prefix openssl@3)`)

produces two binaries:
- `build/xtensa-emu` — the emulator
- `build/xtensa-tests` — test suite

## usage

```
# basic run (jit is on by default)
./build/xtensa-emu firmware.bin

# with elf symbols and cycle limit
./build/xtensa-emu -s app.elf -c 10000000 firmware.bin

# quiet mode (suppress emulator info, show only firmware output)
./build/xtensa-emu -q -s app.elf -c 5000000 firmware.bin

# interpreter-only mode (the slow lane, for comparison or debugging)
./build/xtensa-emu --no-jit -s app.elf -c 10000000 firmware.bin

# verbose trace to stderr
./build/xtensa-emu -T -s app.elf -c 1000000 firmware.bin 2>/tmp/trace.log
```

### flags

| flag | description |
|------|-------------|
| `-J` | enable jit (default: on where supported) |
| `--no-jit` | disable jit, run fully interpreted |
| `--jit-stats` | print jit block/coverage statistics on exit |
| `-s ELF` | load elf for symbols and firmware hooks |
| `-c N` | stop after N executed instructions (both cores counted) |
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

switch-based interpreter core + a tracing jit on top. interpreter: fetch → decode → execute → loop check → interrupt check → advance ccount. the jit watches hot pcs, compiles basic blocks (continuing through conditional branches as traces), chains blocks together natively, and folds flash literal loads into immediates. anything it can't handle simply runs interpreted — no correctness cliff.

```
src/
  xtensa.c           interpreter core (~3500 lines, every isa instruction)
  xtensa.h           cpu state struct
  jit.c              tracing jit: scan, compile, chain, dispatch
  jit_emit_arm64.h   arm64 machine code emitters
  jit_emit_x64.h     x86-64 machine code emitters
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

~20k lines of c total. see [ARCHITECTURE.md](ARCHITECTURE.md) for detailed design notes.

## testing

```
./build/xtensa-tests
# 542 tests, 1394 passed, 0 failed
```

tests cover individual instructions, memory operations, windowed registers, exceptions, interrupts, peripherals, rom stubs, freertos, esp_timer, nvs, gpio driver, and end-to-end firmware compatibility.

Production ROMs are kept outside the repository. Run the sustained stock-ROM
correctness/performance gate by supplying either image (or both):

```bash
MARAUDER_BIN=/path/to/marauder.bin \
NERDMINER_BIN=/path/to/nerdminer.bin \
./bench-stock-roms.sh
```

The gate uses Flexe's reported per-core virtual cycles to compare elapsed
simulated time with wall time, so dual-core workloads are not mistakenly
credited twice. It rejects traps and early stops and defaults to requiring at
least 1.0× real-time averaged over three 1.2-billion-cycle runs. `EMU`,
`CYCLES`, `REPS`, `ENGINE`, `MIN_REALTIME`, and `ESP_HZ` are configurable.

Current Release-build results on Apple silicon (three default-length runs):

| stock CYD image | jit vs 240 MHz ESP32 |
|---|---:|
| ESP32 Marauder v1.14 | **9.11× real-time** |
| NerdMiner v2 | **5.99× real-time** |

The headless integration runner exercises display output and storage. The
Marauder profile drives touch navigation, submits `sniffraw` through the real
UART0 FIFO/interrupt path, and delivers a beacon through the production ROM's
registered promiscuous callback; Marauder's own frame counters must advance.
It then exits capture mode through a second UART command, launches the stock
Rick Roll attack, and requires its raw beacon frames to cross Flexe's host
virtual-radio transmit boundary.
The NerdMiner profile connects to the stock ROM's captive portal through its
remapped host sockets, issues a real HTTP request and DNS query, and requires
valid responses from both:

```bash
./build/flexe-stock-rom-test nerdminer /path/to/nerdminer.bin
./build/flexe-stock-rom-test marauder /path/to/marauder.bin
```

## status

boots `hello_world`, `blink`, `tjpgd`, `real_time_stats`, `spi_lcd_touch` (lvgl!), and friends from esp-idf. runs them 2–10× faster than the real chip. doesn't model timing, caches, or multicore cache-coherence (both cores run, though).

## license

mit
