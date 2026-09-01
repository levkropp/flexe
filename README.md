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
- 645 tests

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
# 645 tests, 4932 passed, 0 failed
```

tests cover individual instructions, memory operations, windowed registers, exceptions, interrupts, peripherals, rom stubs, freertos, esp_timer, nvs, gpio driver, and end-to-end firmware compatibility.

For host-memory and undefined-behavior validation, build with
`-fsanitize=address,undefined`; the default 4 MB predecode configuration is
covered by both the unit suite and the compiled/stock-ROM integration runners.

The compiled Arduino hardware gates below all pass — 17 of 17 on x86-64
Linux (GCC 15, Arduino-ESP32 2.0.11), under both the interpreter and the JIT.

The optional compiled Arduino gate builds an unmodified `Wire` client, runs
40-byte writes and repeated-start reads through the real ESP-IDF interrupt
driver, checks address NACK behavior, and requires zero unhandled MMIO:

```bash
ARDUINO_CLI=/path/to/arduino-cli ./test-i2c-wire.sh
# stage=0x1C2C0040 ... write_bytes=42 read_bytes=40 memory_ok=1 unhandled=0
```

It has been verified with Arduino-ESP32 2.0.11; `ARDUINO_CLI`,
`FLEXE_ARDUINO_FQBN`, `FLEXE_BUILD_DIR`, and `FLEXE_I2C_BUILD_DIR` are
configurable.

The independent RTC-domain I2C gate uses Espressif's real `rtc_i2c_reg.h`
and `sens_reg.h` register macros. It executes the software-start form of the
classic ULP `I2C_RD`/`I2C_WR` control word on host bus port 2, covering all
eight slave selectors, single-byte register reads and writes, partial-bit
writes, result/DONE latches, NACK and timeout diagnostics, the controller's
shifted raw/status/clear bitmap, and the 16 command registers. Both engines
must reach the attached virtual target with no fallback MMIO or ROM calls:

```bash
ARDUINO_CLI=/path/to/arduino-cli \
FLEXE_ARDUINO_CONFIG=/path/to/arduino-cli.yaml \
./test-rtc-i2c.sh
# engine=jit stage=0x12C0C0DE timing=40/40/200 ... calls=4 bytes=6/2 memory=1 unhandled=0 unregistered=0
# engine=interp stage=0x12C0C0DE timing=40/40/200 ... calls=4 bytes=6/2 memory=1 unhandled=0 unregistered=0
```

The fixture is verified with Arduino-ESP32 2.0.11 and 3.3.11.
`FLEXE_RTC_I2C_BUILD_DIR` optionally preserves its Arduino build directory.

The analog gate builds an unmodified Arduino sketch using `analogRead()`,
`analogReadResolution()`, `dacWrite()`, and `dacDisable()`. It verifies ADC1
and ADC2 at 12- and 9-bit widths, RTCIO-backed DAC state/events, and zero
unhandled MMIO or unregistered ROM calls:

```bash
ARDUINO_CLI=/path/to/arduino-cli ./test-analog-dac.sh
# stage=0xADC0DAC0 adc=2645/1450/85/426 dac=0:0x35/1:0xCA ... unhandled=0 unregistered=0
```

`FLEXE_ANALOG_BUILD_DIR` optionally preserves its Arduino build directory.

The I2S gate builds an unmodified Arduino sketch against ESP-IDF's public
`driver/i2s.h` API. It installs the real full-duplex driver, exercises its
circular `lldesc` rings and ISR queues, writes and reads exact 128-byte PCM
buffers, restarts DMA, and validates the host TX callback/RX injection boundary
at 16 kHz, 16-bit stereo with zero unhandled MMIO or unregistered ROM calls:

```bash
ARDUINO_CLI=/path/to/arduino-cli ./test-i2s-dma.sh
# stage=0x12D5DAA0 result=0/128/128/0x00000000 ... audio=16000/16/2 ... unhandled=0 unregistered=0
```

`FLEXE_I2S_BUILD_DIR` optionally preserves its Arduino build directory.

The RMT gate builds an unmodified sketch against ESP-IDF's public
`driver/rmt.h` API. It sends 96 exact pulse items through a one-block,
64-word channel, forcing the production ISR to refill both 32-word halves.
The gate validates pulse timing, DPORT module reset, TX threshold/end
interrupts, the host pulse boundary, and zero unhandled MMIO or unregistered
ROM calls under both the JIT and interpreter:

```bash
ARDUINO_CLI=/path/to/arduino-cli ./test-rmt-tx.sh
# engine=jit stage=0x524D54A0 ... chunks=4 items=96 pattern=1 ... unhandled=0 unregistered=0
# engine=interp stage=0x524D54A0 ... chunks=4 items=96 pattern=1 ... unhandled=0 unregistered=0
```

`FLEXE_RMT_BUILD_DIR` optionally preserves its Arduino build directory.

The LEDC gate builds an unmodified sketch against ESP-IDF's public
`driver/ledc.h` API. It configures a 5 kHz, 8-bit PWM output on GPIO21,
validates boundary-latched duty updates from 64 to 192, then runs a blocking
fade from 192 to 96 through the production driver's ISR and semaphore path.
It also checks the host PWM boundary, output stop, and zero unhandled MMIO or
unregistered ROM calls under both the JIT and interpreter:

```bash
ARDUINO_CLI=/path/to/arduino-cli ./test-ledc-pwm.sh
# engine=jit stage=0x1EDCC0DE result=0/0/5000/64/.../192/.../96/... events=9 ... valid=1 unhandled=0 unregistered=0
# engine=interp stage=0x1EDCC0DE result=0/0/5000/64/.../192/.../96/... events=9 ... valid=1 unhandled=0 unregistered=0
```

`FLEXE_LEDC_BUILD_DIR` optionally preserves its Arduino build directory.

The sigma-delta gate builds an unmodified sketch through Arduino's public
`sigmaDelta*` wrapper and ESP-IDF's public `driver/sigmadelta.h` API. It
exercises channels 0 and 7, signed duty and prescale updates, GPIO-matrix
routing, GPIO enable state, the host aggregate-output callback, and an exact
192/256 live pulse-density period. It also guards the post-boot PLL/160 MHz
clock selectors used by genuine ESP-IDF clock queries. Both engines must
finish with zero unhandled MMIO or unregistered ROM calls:

```bash
ARDUINO_CLI=/path/to/arduino-cli ./test-sigmadelta.sh
# engine=jit stage=0x5349474D api=312500/192 regs=0040/07E0 routes=100/107 enable=00C40000 callbacks=11/1/1/1 density=192/256 unhandled=0 unregistered=0
# engine=interp stage=0x5349474D api=312500/192 regs=0040/07E0 routes=100/107 enable=00C40000 callbacks=11/1/1/1 density=192/256 unhandled=0 unregistered=0
```

The fixture is verified with Arduino-ESP32 2.0.11.
`FLEXE_ARDUINO_CONFIG` selects an Arduino CLI configuration and
`FLEXE_SIGMADELTA_BUILD_DIR` optionally preserves the fixture build.

The PCNT gate builds an unmodified sketch against ESP-IDF's public
`driver/pcnt.h` API. Host GPIO transitions pass through the production GPIO
matrix and ten-APB-cycle glitch filter into unit 0, while unit 7/channel 1
checks the upper signal-index range. The gate verifies control-input direction
reversal, pause/clear/resume, threshold and high-limit events, automatic limit
reset, and the real shared PCNT ISR service with zero unhandled MMIO or
unregistered ROM calls under both engines:

```bash
ARDUINO_CLI=/path/to/arduino-cli ./test-pcnt-pulse.sh
# engine=jit stage=0xC01A7E00 result=0/0/0/0/0/10/.../3/1/11/1/1/0/3/1/0 handled=1/1/1/1 unhandled=0 unregistered=0
# engine=interp stage=0xC01A7E00 result=0/0/0/0/0/10/.../3/1/11/1/1/0/3/1/0 handled=1/1/1/1 unhandled=0 unregistered=0
```

`FLEXE_PCNT_BUILD_DIR` optionally preserves its Arduino build directory.

The MCPWM gate builds an unmodified sketch against ESP-IDF's public legacy
`driver/mcpwm.h` API. Unit 0 drives complementary 20 kHz outputs through
carrier and dead-time blocks while unit 1 independently drives operator 2 at
1 kHz. The host exercises real GPIO-matrix sync, capture ISR callbacks,
cycle-by-cycle fault actions, forced high/low generator modes, shadowed
frequency/duty updates, and stop-at-TEZ behavior. Both engines must report all
three outputs stopped with zero unhandled MMIO or unregistered ROM calls:

```bash
ARDUINO_CLI=/path/to/arduino-cli ./test-mcpwm-motor.sh
# engine=jit stage=0x4D435057 result=0/.../10000/7500/0/0 events=19/18/9 ... levels=1/1/1/1 unhandled=0 unregistered=0
# engine=interp stage=0x4D435057 result=0/.../10000/7500/0/0 events=19/18/9 ... levels=1/1/1/1 unhandled=0 unregistered=0
```

`FLEXE_MCPWM_BUILD_DIR` optionally preserves its Arduino build directory.

The Timer Group gate builds an unmodified sketch against ESP-IDF's public
legacy `driver/timer.h` API. It runs both timers in both groups concurrently,
covering 64-bit capture/load, count-up and count-down modes, pause/resume,
one-shot alarm rescheduling, auto-reload, DPORT clock/reset control, and the
four genuine per-timer ISR callbacks. Both engines must finish with zero
unhandled MMIO or unregistered ROM calls:

```bash
ARDUINO_CLI=/path/to/arduino-cli ./test-timer-group.sh
# engine=jit stage=0x54494D47 result=0/.../11/4/12/6/.../10000/66/305419896/0 unhandled=0 unregistered=0
# engine=interp stage=0x54494D47 result=0/.../11/4/12/6/.../10000/66/305419896/0 unhandled=0 unregistered=0
```

`FLEXE_TIMER_GROUP_BUILD_DIR` optionally preserves its Arduino build directory.

The legacy FRC timer gate builds a sketch against Espressif's public
`soc/frc_timer_reg.h` definitions and installs genuine guest handlers with
`esp_intr_alloc`. FRC1 runs as a 23-bit APB countdown timer with level
interrupts and automatic reload; FRC2 crosses its 32-bit wrap point, pulses an
edge interrupt without a status clear, and advances each following compare
from its ISR. The gate also checks `/16` and `/256` prescalers, live count
readback, disabled-counter freeze behavior, and zero unhandled MMIO or
unregistered ROM calls under both engines:

```bash
ARDUINO_CLI=/path/to/arduino-cli ./test-frc-timer.sh
# engine=jit stage=0x46524332 result=0/0/3/80000/193/128/6/7/4097/.../133/10000/5000/5000/5000/136/100/412/... unhandled=0 unregistered=0
# engine=interp stage=0x46524332 result=0/0/3/80000/193/128/6/7/4097/.../133/10000/5000/5000/5000/136/100/412/... unhandled=0 unregistered=0
```

`FLEXE_FRC_TIMER_BUILD_DIR` optionally preserves its Arduino build directory.

The UHCI gate builds a sketch against Espressif's public `soc/uhci_reg.h`
register definitions and genuine `lldesc` DMA structures. It verifies both
independent UHCI controllers, transparent UART TX/RX descriptor chains,
descriptor ownership/writeback and diagnostics, exact wire-time completion,
real interrupt allocation, and host RX injection. Raw MMIO regressions also
cover H:5 headers, checksum/sequence/CRC handling, configurable SLIP escaping,
quick-send packets, idle/length/break EOF, DPORT reset domains, and malformed
descriptor failures. Both execution engines must finish with zero unhandled
MMIO or unregistered ROM calls:

```bash
ARDUINO_CLI=/path/to/arduino-cli ./test-uhci-dma.sh
# engine=jit stage=0x55484349 ... isr_raw=0x000021B0 ... unhandled=0 unregistered=0
# engine=interp stage=0x55484349 ... isr_raw=0x000021B0 ... unhandled=0 unregistered=0
```

`FLEXE_UHCI_BUILD_DIR` optionally preserves its Arduino build directory.

The SDIO-slave gate builds a sketch against Espressif's public
`driver/sdio_slave.h` API and runs the genuine driver, HAL/LL code, descriptor
queues, and source-10 ISR. The virtual host reads a 21-byte slave packet,
writes a 24-byte packet across two 16-byte receive descriptors, and exercises
the cumulative packet-length and receive-token counters. It also verifies the
HAL's RX-done software-ISR doorbell, IOREADY and DPORT reset/clock control, all
shared-register addressing gaps, and interrupts in both directions. Both
engines must complete with zero unhandled MMIO or unregistered ROM calls:

```bash
ARDUINO_CLI=/path/to/arduino-cli ./test-sdio-slave.sh
# engine=jit stage=0x5344494F ready=1 buffers=2 bytes=21 ... transfer=1/21/1/0 ... unhandled=0 unregistered=0
# engine=interp stage=0x5344494F ready=1 buffers=2 bytes=21 ... transfer=1/21/1/0 ... unhandled=0 unregistered=0
```

The fixture is verified with Arduino-ESP32 2.0.11.
`FLEXE_ARDUINO_CONFIG` selects an Arduino CLI configuration and
`FLEXE_SDIO_SLAVE_BUILD_DIR` optionally preserves the fixture build.

The native SDMMC gate builds a sketch against Espressif's public
`driver/sdmmc_host.h` API and runs the genuine ESP-IDF host driver without
intercepting `sdmmc_host_do_transaction`. It covers slot initialization and
clock-update commands, SD command/short/long responses, single-block reads and
writes, and 20 KiB multiblock transfers. The latter requires five 4 KiB
descriptors and therefore forces the stock driver's four-entry IDMAC ring to
recycle a descriptor through its real ISR and FreeRTOS queue path. Both engines
must reproduce the host-backed card contents with zero unhandled MMIO or
unregistered ROM calls. Raw regressions also verify that an existing backing
image exposes its actual capacity rather than its configured minimum:

```bash
ARDUINO_CLI=/path/to/arduino-cli ./test-sdmmc-host.sh
# engine=jit stage=0x53444D4D ... reads=2/41 writes=2/41 media=1 unhandled=0 unregistered=0
# engine=interp stage=0x53444D4D ... reads=2/41 writes=2/41 media=1 unhandled=0 unregistered=0
```

`FLEXE_SDMMC_BUILD_DIR` optionally preserves its Arduino build directory.

The TWAI gate builds a sketch against Espressif's public `driver/twai.h` API
and runs the genuine ESP-IDF driver, ISR, and FreeRTOS TX/RX queues. It covers
500 kbit/s timing, queued standard and extended transmission, single-shot self
reception, data and remote-frame host injection, alerts, counters, and clean
driver shutdown. Raw regressions additionally cover acceptance filtering,
64-byte FIFO overrun/release behavior, arbitration loss, automatic retry,
error-warning/passive transitions, bus-off recovery, and the DPORT reset
domain. Both engines must exchange the exact frames with zero unhandled MMIO
or unregistered ROM calls:

```bash
ARDUINO_CLI=/path/to/arduino-cli ./test-twai-bus.sh
# engine=jit stage=0x54574149 ... tx=3 frames=1 inject=1/1 pending=0 unhandled=0 unregistered=0
# engine=interp stage=0x54574149 ... tx=3 frames=1 inject=1/1 pending=0 unhandled=0 unregistered=0
```

`FLEXE_TWAI_BUILD_DIR` optionally preserves its Arduino build directory.

The Ethernet gates drive Flexe's classic ESP32 EMAC through two independent
compiled paths. `test-emac-hal.sh` uses Espressif's genuine `emac_hal_*` and
LL interrupt APIs; `test-emac-driver.sh` uses the public
`esp_eth_mac_new_esp32` object, its real ISR and notification-backed RX task,
mediator callbacks, and full init/link/deinit lifecycle. Together with the raw
MMIO regressions, they cover enhanced chained descriptor rings, multi-buffer
700-byte frames, exact TX capture, perfect and multicast RX filtering, FCS and
descriptor writeback, source-38 interrupts, DMA unavailable/error states,
DPORT clock/reset behavior, and Clause-22 PHY reads/writes. Both engines must
finish with zero unhandled MMIO or unregistered ROM calls:

```bash
ARDUINO_CLI=/path/to/arduino-cli ./test-emac-hal.sh
ARDUINO_CLI=/path/to/arduino-cli ./test-emac-driver.sh
# profile=hal engine=jit stage=0x454D4143 ... tx_ok=1 inject=1/1 irq=2 ... unhandled=0 unregistered=0
# profile=driver engine=interp stage=0x454D4143 ... tx_ok=1 inject=1/1 irq=2 ... unhandled=0 unregistered=0
```

The fixtures are verified with Arduino-ESP32 2.0.11. `FLEXE_ARDUINO_CONFIG`
selects an Arduino CLI configuration, while `FLEXE_EMAC_BUILD_DIR` and
`FLEXE_EMAC_DRIVER_BUILD_DIR` optionally preserve the two fixture builds.

Production ROMs are kept outside the repository. Run the sustained stock-ROM
correctness/performance gate by supplying either image (or both):

The current official CYD baselines are pinned below. Flexe fingerprints three
incompatible Marauder link layouts despite their shared `0x400831D8` entry
point: v1.14.0/1 use the original layout, v1.14.2/3 the shifted layout, and
v1.15.x a third one (flash moved by a per-library delta and `.bss` by 0x188).
An unknown image with that entry point is rejected instead of receiving unsafe
address-based ROM, Wi-Fi, or NimBLE hooks.

| release | official asset | SHA-256 |
|---|---|---|
| NerdMiner v1.8.3 | `ESP32-2432S028R_factory.bin` | `e7aece42f24ad7fd4146b94eeb28d04de7ce27f0c45e19be1bf38ad39ce0582c` |
| Marauder v1.14.0 | `esp32_marauder_v1_14_0_20260731_cyd_2432S028.bin` | `f2d21c476be70b7525a0b7c3122b4917fb5bcbeda78b16316b84450342d77fe7` |
| Marauder v1.14.1 | `esp32_marauder_v1_14_1_20260801_cyd_2432S028.bin` | `b3be0ff11ed4d67d8d763abb94eb22c2df2057adfdca58b780ac756ca20497d7` |
| Marauder v1.14.2 | `esp32_marauder_v1_14_2_20260815_cyd_2432S028.bin` | `5965e59f0e6f599eae213941d5ccc9d8d1dab1ebd7faa12b694cdff1a5cd3047` |
| Marauder v1.14.3 | `esp32_marauder_v1_14_3_20260816_cyd_2432S028.bin` | `ad91696012f407bf782826793edd509119acf00e4751cd0d30eddd6223d6bf2d` |
| Marauder v1.15.1 | `esp32_marauder_v1_15_1_20260824_cyd_2432S028.bin` | `72fa27948cd7f3bce4b6eabaaa8757b0d0e7854c534e8a502ce197d2397d899b` |

Every v1.15.1 address was relocated from the v1.14.3 profile by unique
masked-signature matching (L32R/CALL immediates wildcarded, since those move
with the literal pool), and the three NimBLE literal-pool entries and five DRAM
globals were confirmed independently. v1.15.1 passes the gate end to end under
both engines.

`tools/relocate_profile.py` is that relocation, made repeatable. It wildcards
the operand fields a relink moves, requires a *unique* match before reporting
an address, and resolves literal slots and DRAM globals through the L32R that
references them rather than by assuming a uniform section delta. Run it
against the reference image itself first: every address must come back
unchanged, which is what makes a nonzero delta elsewhere trustworthy.

    python3 tools/relocate_profile.py REFERENCE.bin TARGET.bin ADDR [ADDR ...]

Marauder v1.14.3 is also published for three other CYD boards, which are
separate links with their own entry point (`0x400830D0`). The 2432S024
(guition) and 3.5-inch builds have relocated profiles and boot, run Wi-Fi end
to end, and drive BLE scan start/stop; injecting a BLE advertisement into them
does not yet complete, because their IRAM contains a NimBLE callback path with
no counterpart in the 2432S028 build, so they are not part of the stock-ROM
gate. The `2432S028_2usb` build is a different link generation again -- only 3
of its 41 profile addresses relocate from v1.14.3 -- and is still rejected as
an unsupported layout.

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
| ESP32 Marauder v1.14.3 | **8.50× real-time** |
| NerdMiner v1.8.3 | **5.86× real-time** |

Stock images idle a great deal, so that gate measures the emulator's ability
to keep up with real firmware rather than its peak throughput. For the latter
use `bench-compute.sh`, which builds an in-repo fixture and runs a fixed
number of *rounds* -- identical work every run and on both engines, rather
than a cycle budget -- across a dependent scalar chain, strided array traffic,
SHA-256-shaped rotate/xor mixing, and byte-wide load/store:

```bash
ARDUINO_CLI=/path/to/arduino-cli ./bench-compute.sh
# engine=interp rounds=12000 ... mips=187.1 realtime=0.78x checksum=0xC6B74AFC
# engine=jit    rounds=12000 ... mips=768.4 realtime=3.20x checksum=0xC6B74AFC
# checksum=0xC6B74AFC (engines agree)  jit-speedup=4.11x
```

Both engines must agree on the checksum, so this is a JIT correctness check on
real compiler output as much as a measurement. `BENCH_ROUNDS`, `BENCH_REPS`
and `MIN_REALTIME` are configurable. On x86-64 Linux (GCC 15, Release) a
loop-heavy sketch reaches **1878 MIPS, 7.83× real-time**, against 304 MIPS
interpreted.

`--jit-verify` runs every compiled block twice -- once natively, once through
the interpreter from the same starting state, with the interpreter's memory
effects rolled back in between -- and reports any block whose architectural
state or memory writes differ. It is roughly an order of magnitude slower and
is a debugging tool rather than a run mode, but it is what found the SRC and
byte-store miscompiles; all three stock ROMs now verify clean over 11M blocks.

The headless integration runner exercises display output and storage. The
Marauder profile drives touch navigation, submits `sniffraw` through the real
UART0 FIFO/interrupt path, feeds a valid NMEA fix through the independent
UART2 FIFO, and requires the production GPS parser and `gps -g lat` command to
return the expected latitude. It also verifies the GPS driver's real UART2
configuration exchange. The runner then delivers a beacon through the
production ROM's registered promiscuous callback; Marauder's own frame counters
must advance.
It then exits capture mode through a second UART command, launches the stock
Rick Roll attack, and requires its raw beacon frames to cross Flexe's host
virtual-radio transmit boundary. Finally, it starts `sniffbt`, injects a BLE
advertisement through the production NimBLE GAP handler, and requires
Marauder's registered device callback to parse and print `Device: FlexeBLE`.
It then submits `blespam -t windows`, lets the genuine NimBLE advertising
stack issue its controller HCI commands, and validates the emitted Microsoft
Swift Pair payload at Flexe's host BLE-radio boundary.
The NerdMiner profile connects to the stock ROM's captive portal through its
remapped host sockets, issues a real HTTP request and DNS query, and requires
valid responses from both:

```bash
./build/flexe-stock-rom-test nerdminer /path/to/nerdminer.bin
./build/flexe-stock-rom-test marauder /path/to/marauder.bin
```

## status

boots `hello_world`, `blink`, `tjpgd`, `real_time_stats`, `spi_lcd_touch` (lvgl!), and friends from esp-idf. runs them 2–10× faster than the real chip. isn't cycle-accurate and doesn't model cache timing or true simultaneous multicore cache coherence (both cores run, though).

## license

mit
