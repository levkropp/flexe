# Testing

Flexe uses layered tests because instruction correctness, peripheral behavior,
and a successful production boot catch different classes of defects.

## Unit and differential suite

```sh
cmake -S . -B build -DFLEXE_LTO=OFF
cmake --build build --target xtensa-tests -j
./build/xtensa-tests
```

For repeated clean or multi-configuration builds, an installed `ccache` can
be enabled with `-DCMAKE_C_COMPILER_LAUNCHER=ccache`; CI uses separate caches
for GCC, Clang, sanitizers, AArch64, and fixture runners so incompatible flags
cannot contaminate one another.

Each test file is compiled independently, so editing one suite rebuilds only
that suite and the final test executable. Case-insensitive filters select by
suite or test name; multiple filters are ORed. A filter matching nothing is an
error, which keeps misspelled focused checks from silently passing:

```sh
./build/xtensa-tests system_clock
./build/xtensa-tests --list rmt
./build/xtensa-tests --quiet
```

`--quiet` retains assertion diagnostics and the final totals without printing
one line per passing test. It is the default CI form and keeps full-suite logs
small during rapid hardware-model iterations.

The suite covers instruction decode and execution, memory translation,
register windows, exceptions, interrupts, peripheral registers and timing,
FreeRTOS/service stubs, and both JIT backends. The encoding-space sweep compiles
thousands of instruction forms and compares their architectural effects with
the interpreter. Its cases reuse two independently backed machines and one JIT
cache, with isolated instruction slots and bounded cache rollovers, so the
exhaustive comparison does not allocate a complete emulator per encoding.

For host-memory validation:

```sh
cmake -S . -B build-asan \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -DNATIVE_ARCH=OFF \
  -DFLEXE_SANITIZERS=ON
cmake --build build-asan --target xtensa-tests -j
ASAN_OPTIONS=halt_on_error=1 UBSAN_OPTIONS=halt_on_error=1 \
  ./build-asan/xtensa-tests
```

`FLEXE_SANITIZERS=ON` enables ASan+UBSan and disables LTO for this build, which
keeps instrumented relinks dependency-scoped. CI adds
`ASAN_OPTIONS=detect_leaks=1` on Linux; Apple's ASan runtime does not provide
LeakSanitizer.

## Compiled-firmware hardware gates

The fixtures under `tests/fixtures/` are real Arduino-ESP32 sketches. Their C
runners under `tools/` attach host endpoints, inject input, and assert the
guest's observable output. The consolidated entry point validates the request,
builds only the required host runners in one parallel CMake invocation, then
builds and runs every requested fixture with both the JIT and interpreter:

```sh
./scripts/test-fixtures.sh
```

Pass fixture names (hyphens or underscores are accepted) for a focused run:

```sh
./scripts/test-fixtures.sh gpio-isr spi-master i2c_wire
```

Configuration:

| Variable | Purpose |
|---|---|
| `ARDUINO_CLI` | Arduino CLI executable |
| `FLEXE_ARDUINO_CONFIG` | Optional Arduino CLI config file |
| `FLEXE_ARDUINO_FQBN` | Board and menu configuration |
| `FLEXE_BUILD_DIR` | Configured host CMake build directory |
| `FLEXE_FIXTURE_BUILD_ROOT` | Fixture build root, or `temporary` for disposable builds |
| `FLEXE_FIXTURE_REBUILD` | Set to `1` to ignore valid cached fixture firmware |

Compiled sketch outputs persist under `FLEXE_BUILD_DIR/arduino-fixtures` by
default, and a content fingerprint covers the fixture source, FQBN,
optimization, Arduino CLI and installed core versions, executable wrapper,
and explicit config. An unchanged focused rerun skips Arduino CLI entirely
instead of merely asking it to rediscover an unchanged dependency graph. A
cache miss leaves the intermediate build path under Arduino CLI's shared
compilation cache, so multiple changed fixtures reuse the same compiled core.
Set `FLEXE_FIXTURE_BUILD_ROOT` to another directory to isolate a
board/toolchain configuration, or to `temporary` for a clean disposable
build.
CI restores older per-fixture outputs as a fallback, recompiles only fixtures
whose fingerprints changed, and caches the shared compiled core without each
fixture's much larger intermediate build tree.

## Production ROM gates

The curated scenarios require external images:

```sh
BRUCE_BIN=/path/to/bruce.bin \
MARAUDER_BIN=/path/to/marauder.bin \
MESHTASTIC_BIN=/path/to/meshtastic.bin \
NERDMINER_BIN=/path/to/nerdminer.bin \
OPENHASP_BIN=/path/to/openhasp.bin \
TASMOTA_BIN=/path/to/tasmota32.bin \
WLED_BIN=/path/to/wled.bin \
./scripts/check-stock-roms.sh
```

Set `FLEXE_ROM_ELF` to the official ESP32 ROM ELF for the Meshtastic and other
newer ESP-IDF images that use mask-ROM data.

For a broader directory of images:

```sh
FLEXE_ROMS=/path/to/corpus ./scripts/check-firmware.sh
```

See [Firmware compatibility](compatibility.md) for the assertions and current
known failures.

The S3 production gates also use external pinned images and the official ROM
ELF. `check-s3-nerdminer-portal.sh` exercises provisioning and reset,
`check-s3-marauder.sh` runs the official v1.16.0 MultiBoard S3 image through
native Bluetooth/Wi-Fi initialization and its absent-GPS probe, then injects
`help` through UART0 and verifies the command response and following prompt,
and requires nonzero JIT retirement. `check-s3-wled-rmt.sh` runs both engines
concurrently, compares every completed LED frame plus the final CPU/time
summary, and also requires nonzero JIT retirement. `check-s3-wled-http.sh`
checks a matching
WLED source-build image/ELF pair through its own raw-lwIP web server. The HTTP
gate needs a build with
libslirp 4.9 or newer and `jq`; it binds a randomly selected host loopback
port and checks page delivery, a JSON state change, readback, and Ethernet
delivery.
See [Hardware completeness](hardware-completeness.md) for exact inputs and
scope.

The direct ESP-IDF S3 driver gates are separate from `test-fixtures.sh`.
For example, `check-s3-idf-i2c-master.sh` replays a pinned ESP-IDF 5.3
I2C-master image against a host register device, including repeated-START,
FIFO refill, and NACK handling. `check-s3-idf-rmt-loopback.sh` verifies the
stock RMT TX/RX drivers through GPIO4 and their ISR callback, including exact
plain pulse widths, carrier-demodulated envelopes, finite counted-loop
interrupts, explicitly stopped infinite loops, a two-channel simultaneous
start barrier, and sustained FreeRTOS execution. Build commands and
artifact hashes are in [Hardware completeness](hardware-completeness.md).

## JIT verification

`--jit-verify` runs each eligible compiled block natively, rolls back its memory
effects, replays the same guest-instruction count in the interpreter, and
compares architectural state. It is intentionally slow and is meant for
correctness investigations:

```sh
./build/xtensa-emu --jit-verify -c 100000000 firmware.bin
```

`FLEXE_JIT_DUMP=/path` writes generated blocks for host disassembly.
`FLEXE_JIT_STATS=1` prints compilation, coverage, and chaining counters from
the stock-ROM runner.

## Traces

Use `-T` to write a verbose emulator trace, then narrow it with
`build/trace-filter`:

```sh
./build/xtensa-emu -T -c 1000000 firmware.bin 2>trace.log
./build/trace-filter -e trace.log       # exceptions
./build/trace-filter -w trace.log       # window events
./build/trace-filter -u trace.log       # unregistered ROM calls
./build/trace-filter -p trace.log       # panic/abort paths
```

CI builds with GCC and Clang, runs ASan+UBSan, cross-compiles backend-relevant
sources for AArch64, and executes the compiled-firmware gate set.
