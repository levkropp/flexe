# Flexe

**F**ree **l**ittle **x**tensa **e**mulator: a lightweight ESP32/Xtensa LX6
emulator written in C. Flexe boots unmodified ESP-IDF and Arduino firmware,
models the peripherals used by real boards, and includes ARM64 and x86-64 JIT
backends.

Flexe is under active development. Bruce, Marauder, Meshtastic, NerdMiner,
openHASP, Tasmota, and WLED pass scripted end-to-end scenarios, and the broader
production corpus passes the generic interpreter/JIT gate described in
[Firmware compatibility](docs/compatibility.md).

## Highlights

- Xtensa LX6 integer, loop, MAC16, floating-point, exception, interrupt, and
  windowed-register execution
- ESP32 SRAM, ROM, flash, RTC memory, PSRAM, flash MMU, and dual-core model
- Functional and event-timed GPIO, UART, SPI, I2C, I2S, RMT, LEDC, PCNT,
  MCPWM, TWAI, Ethernet, SDMMC/SDIO, ADC/DAC, RTC, timer, watchdog, and crypto
  models, with each model's timing limits documented separately
- CYD ILI9341 display, XPT2046 touch, SD/FAT, and SPIFFS integration
- FreeRTOS, ESP timer, NVS, GPIO, Wi-Fi, Bluetooth, VFS, and ROM boundaries
  needed by production firmware
- Switch interpreter plus tracing JIT for Apple silicon and x86-64
- Differential JIT verification, compiled-firmware hardware gates, stock-ROM
  scenarios, and reproducible performance benchmarks

Flexe is not cycle accurate. It preserves firmware-visible time and device
ordering, but it does not model cache timing or truly simultaneous execution
of both cores.

## Build

Requirements: a C17 compiler, CMake, OpenSSL, zlib, and pthreads.

```sh
cmake -S . -B build
cmake --build build -j
```

On macOS with Homebrew OpenSSL:

```sh
cmake -S . -B build -DOPENSSL_ROOT_DIR="$(brew --prefix openssl@3)"
cmake --build build -j
```

The main outputs are:

- `build/xtensa-emu` — emulator CLI
- `build/xtensa-tests` — unit and differential test suite
- `build/flexe-stock-rom-test` — scripted production-firmware scenario runner
- `build/flexe-generic-rom-test` — arbitrary production-ROM probe

Release builds use LTO and host-native tuning by default. Pass
`-DNATIVE_ARCH=OFF` for portable binaries.

## Run firmware

```sh
# JIT is enabled by default
./build/xtensa-emu firmware.bin

# Load ELF symbols and stop after a fixed cycle budget
./build/xtensa-emu -s firmware.elf -c 10000000 firmware.bin

# Compare with the interpreter
./build/xtensa-emu --no-jit -s firmware.elf -c 10000000 firmware.bin

# Quiet firmware UART output, or trace instructions to stderr
./build/xtensa-emu -q firmware.bin
./build/xtensa-emu -T -c 1000000 firmware.bin 2>trace.log
```

Common options:

| Option | Meaning |
|---|---|
| `-J` | Enable the JIT (the default on ARM64 and x86-64) |
| `--no-jit` | Run only the interpreter |
| `--jit-stats` | Print compilation and coverage statistics |
| `--jit-verify` | Replay compiled blocks in the interpreter and compare state |
| `-s ELF` | Load symbols and firmware hooks from an ELF image |
| `-R ROM_ELF` | Load official ESP32 ROM code and data images |
| `--usb-console` | Use native USB Serial/JTAG instead of UART0 for console output |
| `-c N` | Stop after `N` aggregate emulated cycles |
| `-q` | Suppress emulator diagnostics |
| `-T` | Emit an instruction trace to stderr |
| `-b ADDR` | Set a breakpoint |
| `-m ADDR[:LEN]` | Dump guest memory on exit |

## Production status

Bruce, Marauder, Meshtastic, NerdMiner, openHASP, Tasmota, and WLED pass
scripted board-level scenarios in both engines. See
[Firmware compatibility](docs/compatibility.md) for pinned versions,
assertions, and remaining board-specific coverage.

## Test

```sh
./build/xtensa-tests
./scripts/test-fixtures.sh                 # all Arduino hardware gates
./scripts/test-fixtures.sh spi-master i2c-wire
./scripts/check-stock-roms.sh              # curated external ROMs
FLEXE_ROMS=/path/to/roms ./scripts/check-firmware.sh
```

Production ROMs are intentionally not committed. The stock runner accepts
`BRUCE_BIN`, `MARAUDER_BIN`, `MESHTASTIC_BIN`, `NERDMINER_BIN`,
`OPENHASP_BIN`, `TASMOTA_BIN`, and `WLED_BIN`; the generic runner accepts paths
or a `FLEXE_ROMS` directory. Set `FLEXE_ROM_ELF` for images that use data from
the official ESP32 mask ROM, including the pinned Meshtastic build.

See [Testing](docs/testing.md) for sanitizer builds, fixture configuration,
JIT verification, and what each gate asserts.

## Performance

Flexe measures two different things:

- real-time factor — simulated ESP32 time divided by host wall time;
  `1.0x` keeps pace with a 240 MHz ESP32
- retired MIPS — actual guest instructions executed per host second, excluding
  halted and fast-forwarded time

```sh
ARDUINO_CLI=/path/to/arduino-cli ./scripts/bench-compute.sh
MESHTASTIC_BIN=/path/to/meshtastic.bin ./scripts/bench-stock-roms.sh
./scripts/bench-firmware.sh /path/to/firmware.bin
```

In the current Apple-silicon release benchmark, every image in the five-ROM
generic corpus clears real time in both engines. The stricter repeated WLED
acceptance benchmark sustains 1.482x interpreted and 3.522x under the JIT. See
[Performance](docs/performance.md) for dated results and methodology.

## Architecture

The core is a switch interpreter with a tracing JIT. Cold or unsupported
instructions remain interpreted; hot basic blocks are compiled, chained, and
checked against firmware-visible timers and interrupts.

```text
src/                 CPU, JIT, memory, peripheral, and service models
tests/               unit and differential tests
tests/fixtures/      Arduino firmware used by hardware gates
tools/               host-side integration runners and diagnostics
scripts/             build, test, corpus, and benchmark entry points
docs/                design, compatibility, testing, and performance notes
```

Read [ARCHITECTURE.md](ARCHITECTURE.md) for the detailed design.

## License

MIT
