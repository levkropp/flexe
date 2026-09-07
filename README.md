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

On x86-64 the committed `scripts/bench-compute.sh` reports 1789 MIPS under the JIT
(7.45× real-time) against 187 interpreted, and the stock ROMs run at 3.8×
(NerdMiner) and 12.8× (Marauder) real-time.

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
- exception/interrupt dispatch (levels 1–7, timer ccompare, waiti), plus the
  complete 69-source dual-core ESP32 interrupt matrix with fan-in and live remapping
- esp32 memory map (sram, rom, flash, rtc, psram, peripheral i/o)
- hardware flash MMU: complete 64 KiB DROM0/IRAM0/IRAM1/IROM0 mappings,
  dual-core table invalidation, flash programming coherence, and translated-code invalidation
- mmio peripherals: all three uarts, gpio with eight-channel classic
  sigma-delta/PDM, dport, rtc/rtcio, efuse, watchdog,
  legacy FRC1/FRC2 timers, both timer groups with four 64-bit APB
  counters/alarms, dual UHCI UART DMA with H:5/SLIP framing, GP-SPI2/3 DMA,
  classic I2C0/1 masters, RTC-domain I2C/ULP transactions,
  SJA1000-compatible TWAI/CAN,
  Synopsys-based classic Ethernet MAC with descriptor DMA and Clause-22 MDIO,
  dual-slot native SDMMC with PIO/IDMAC DMA, classic SDIO slave
  HINF/HOST/SLC with scatter/gather descriptor DMA,
  I2S0/1 circular DMA,
  eight-channel classic RMT, 16-channel classic LEDC PWM/fades,
  eight-unit/two-channel classic PCNT, both classic MCPWM motor-control units
  with sync/capture/fault/dead-time/carrier paths, ADC1/2, DAC1/2
- raw hardware crypto: aes-128/192/256, sha, rsa modular math and interrupts
- cyd devices: ili9341 display, xpt2046 touch, sd/fat and spiffs storage
- host-backed lwip sockets plus virtual wifi, bluetooth, and phy boundaries
- rom function stubs: ets_printf, memcpy, memset, strlen, cache ops
- freertos stubs: tasks, queues, semaphores, notifications, delays
- esp_timer stubs with callback dispatch
- nvs flash stubs
- gpio driver stubs
- elf symbol loading, breakpoints, verbose trace mode
- jit compiler: hot blocks → native code (arm64 + x86-64), on by default

### profiling

`cmake -S . -B build-prof -DFLEXE_PROFILE=ON` builds an interpreter sampling
profiler; run any harness with `FLEXE_PROFILE=1` for the top guest PCs, or
`FLEXE_JIT_STATS=1` on the stock-ROM runner for JIT statistics. It reports hot
1 KB regions as well as hot PCs, because diffuse code spreads samples so thinly
that no single address stands out while the region still names the function.
`FLEXE_PROFILE_SP=0x3FFD7000` filters to one task's stack, which is the only
way to ask what a particular task is doing in a symbol-less ROM running its own
scheduler.

It is off by default and lives in its own translation unit for a reason: an
earlier version sat in `xtensa.c` behind the same compile-time switch and,
*even fully compiled out*, cost 4% on both stock ROMs — the handful of bytes it
added shifted the dispatch loop's code layout.

## how fast, and where the time goes

On production firmware the JIT covers **22.5% to 46%** of retired instructions:

| image | coverage | insns/entry |
|---|---:|---:|
| `marauder_v1_14_3` | 49.5% | 9.1 |
| `esp32_marauder_v1_15_1` (CYD 2432S028) | 46.6% | 9.8 |
| `esp32_marauder_v1_14_3` (3.5 inch) | 45.9% | 10.4 |
| `esp32_marauder_v1_14_3` (2432S024 guition) | 42.3% | 11.1 |
| `nerdminer_v1_8_3` | 22.5% | 25.1 |

That number read 1.4%, then 6.7%, and both were wrong — the denominator counted
idle and fast-forwarded cycles as retired work. Fixing it also stopped idle
being *simulated* one cycle at a time, worth 1.75× on Marauder. The cheap
cross-check that would have caught it: `insns_jitted` plus the instructions the
interpreter stepped must equal what the same scenario retires under `--no-jit`.

Splitting a Marauder run's 3.30s by measured per-instruction cost:

| | | |
|---|---|---|
| interpretation (235M insns, 30 host cycles each) | 1.69s | 51% |
| JIT execution + dispatch | 0.50s | 15% |
| peripherals, display, host | 1.11s | 34% |

So the interpreter, not the JIT, is the bottleneck — and at 49.5% compiled,
even an infinitely fast JIT caps the whole-workload speedup near 2×. Chaining
already carries 1.5 native blocks per dispatch on Marauder and 6.0 on
NerdMiner, against a natural basic block of 4–6 instructions.

## testing

```
./build/xtensa-tests
# 673 tests, 13205 passed, 0 failed

./scripts/check-stock-roms.sh          # five production ROMs, both engines
FLEXE_ROMS=~/roms ./scripts/check-firmware.sh   # anything else you have
ARDUINO_CLI=... ./scripts/bench-compute.sh
```

Unit tests cover instructions, memory, windowed registers, exceptions,
interrupts, peripherals, ROM stubs, FreeRTOS, esp_timer, NVS and the GPIO
driver. Build with `-fsanitize=address,undefined` for host-memory validation.

`scripts/test-*.sh` are 31 hardware gates that compile a real Arduino sketch
against the ESP32 core and run it on both engines — the only way to catch a
peripheral that reports success and does nothing, which is how most of them
were found. All 31 pass on x86-64 Linux (GCC 15, Arduino-ESP32 2.0.11).

`scripts/check-firmware.sh` asks a weaker question of *arbitrary* firmware:
does it boot, keep executing, print something, stay inside the hardware Flexe
models (no unhandled MMIO, no unregistered ROM calls), end on a PC the chip
could have fetched, and do both engines print the same thing? The UART
transcript is the comparison because it is the one output every ESP32 firmware
has; line timestamps are masked, since the engines reach a given line at
slightly different simulated moments.

`scripts/bench-firmware.sh` reports the real-time factor for any image, with
the retired-instruction and UART counts beside it — because a firmware that is
stuck in a spin loop has an excellent real-time factor and is doing nothing.
Read the two together, always.

| firmware | real-time (interp / JIT) | UART | status |
|---|---|---:|---|
| Meshtastic 2.7.26 (T-Beam) | 6.3× / 7.0× | 11535 | boots, both engines agree |
| openHASP 0.7.0-rc13 (Lanbon L8) | 1.9× / 4.0× | 6411 | boots through LVGL, mDNS, telnet, MQTT |
| WLED 16.0.1 | 1.00× / 0.87× | 6 | boots to its Adalight prompt, then **returns to address 0** |
| Tasmota 15.6.0 | 0.75× / 5.7× | 224 | **stops** after one bad window fill following a Berry longjmp |

**WLED's 1.00× does not mean WLED runs.** It emits six bytes and then spins;
the figure is measuring how cheaply Flexe simulates a spin. The giveaway is in
the same row twice over -- the UART column, and a JIT number *below* the
interpreter's, which only happens on an image that is not doing work. This is
the reason the UART column is in this table at all.

The two failures are correctness gaps, not throughput: both images stop doing
useful work, so their figures measure how fast Flexe simulates that. WLED's
number went *down* when its scheduler livelock was fixed, from 3.4x on the JIT
to 0.84x, because it now retires real instructions instead of walking a
circular list -- which is the clearest illustration there is that the
real-time factor on a broken image measures the breakage.

Both remaining failures are the same defect: a `retw` that returns somewhere
impossible, because a register window was filled from a base that no longer
describes the frame.

`FLEXE_WINDOW_VECTORS=1` runs the guest's own window handlers, which take that
base from the register the vector names instead of walking the call chain and
guessing. Together with `FLEXE_SHADOWFILL` (on by default, inert without it)
that is worth a great deal:

- **Tasmota boots completely** -- 447 bytes of log, Berry initialised, the
  version banner, WifiManager, and its web server on 192.168.4.1 -- against
  224 bytes and a stall on the default path. Unmapped accesses fall from 57.7
  million to 58.
- **openHASP's cross-engine mismatch disappears**, so both images move from
  KNOWN-BAD to PASS in `check-firmware.sh`.
- Marauder, NerdMiner and Meshtastic come out byte-identical.

It is nonetheless **off by default**, because of one scenario: Marauder's
stock-ROM run never creates its `SCRIPTS` directory on the SD card. It issues
13 `cmd24` block writes on the synthesized path and zero on the vector path --
the card initialises and the FAT boot sector is read either way, then the
filesystem layer stops and the guest goes idle. Waiting two billion extra
cycles does not help, and it is the vector path itself rather than the shadow
fills: with `FLEXE_SHADOWFILL=0` the scenario hangs outright. Fixing that one
scenario is what stands between this and two more passing images.

Their `esp_wifi_*` and `esp_event_*` entry points are not hand-found.
`scripts/locate-wifi-symbols.py` matches function prologues against a
throwaway sketch built with the same Arduino core the image links — masking the
PC-relative immediates of L32R, CALLn and J, and insisting on a unique match at
the longest prologue that gives one. Run against a Marauder build whose
addresses were already known by hand it returns all twelve it matches, exactly,
with no false positives. `scripts/disasm-firmware.sh` disassembles any range of
a stripped image at its real load address.

`scripts/check-stock-roms.sh` runs unmodified production images (ESP32
Marauder, NerdMiner) through a scripted scenario on both engines and requires
identical final framebuffers, pinned by SHA-256. `--verify` runs every compiled
block twice — once natively, once interpreted from the same state with memory
effects rolled back — and reports any divergence; it is what found the SRC
miscompile, the byte-store miscompile, and an LEND-spanning block that hung two
CYD builds. The five images verify clean over 77M blocks.

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

## status

boots `hello_world`, `blink`, `tjpgd`, `real_time_stats`, `spi_lcd_touch` (lvgl!), and friends from esp-idf. runs them 2–10× faster than the real chip. isn't cycle-accurate and doesn't model cache timing or true simultaneous multicore cache coherence (both cores run, though).

## license

mit
