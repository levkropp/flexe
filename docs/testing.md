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
harnesses under `tools/` attach host endpoints, inject input, and assert the
guest's observable output. The consolidated entry point validates the request,
links their renamed entry points into one `flexe-fixture-test` multicall runner,
then builds and runs every requested fixture with both the JIT and interpreter:

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
| `ARDUINO_BUILD_CACHE_PATH` | Persistent compiled-core cache root |
| `FLEXE_BUILD_DIR` | Configured host CMake build directory |
| `FLEXE_FIXTURE_BUILD_ROOT` | Fixture build root, or `temporary` for disposable builds |
| `FLEXE_FIXTURE_REBUILD` | Set to `1` to ignore valid cached fixture firmware |
| `FLEXE_FIXTURE_BUILD_JOBS` | Concurrent cache-miss builds; host-aware by default |
| `FLEXE_FIXTURE_GATE_JOBS` | Concurrent fixture gates; host-aware by default |
| `FLEXE_FIXTURE_ENGINE_JOBS` | `2` (default) runs JIT and interpreter together; `1` serializes them |

Compiled sketch outputs persist under `FLEXE_BUILD_DIR/arduino-fixtures` by
default, and a content fingerprint covers the fixture source, FQBN,
optimization, Arduino CLI and installed core versions, executable wrapper,
and explicit config. An unchanged focused rerun skips Arduino CLI entirely
instead of merely asking it to rediscover an unchanged dependency graph. A
toolchain/FQBN cache miss primes one persistent compiled-core archive, then
independent sketches build through a bounded pool. Per-sketch intermediate
trees and redundant map/merged/boot/partition outputs are discarded after the
application image and ELF are retained, avoiding roughly 15 MiB of cache growth
per fixture. A core edit therefore performs one harness link rather than one
full-emulator link per fixture. After every mutable build completes,
independent gates use a second bounded pool; each gate runs its JIT and
interpreter together, and logs remain in request order. Set all three job
variables to `1` for completely serialized diagnosis.
Set `FLEXE_FIXTURE_BUILD_ROOT` to another directory to isolate a
board/toolchain configuration, or to `temporary` for a clean disposable
build.
CI restores older per-fixture outputs as a fallback, recompiles only fixtures
whose fingerprints changed, and caches the shared compiled core separately
without each fixture's much larger intermediate build tree.

The stock ESP32-S3 Arduino gates use Arduino-ESP32 3.3.11 and have their own
cached entry point:

```sh
./scripts/build-s3-arduino-fixture.sh --check ledc rmt-rx
./scripts/build-s3-arduino-fixture.sh --check all
```

It validates the installed core, derives a stable `SOURCE_DATE_EPOCH`, builds
only the required host runners, finds the official S3 ROM ELF, and passes the
resulting artifact hashes to each behavior gate. Firmware and source mirrors
stay under the user cache rather than dirtying the repository. Its fingerprint
covers every fixture input, board options, timestamp, CLI binary and config,
and installed core version. An unchanged run therefore bypasses Arduino CLI's
expensive dependency scan entirely; a cache miss still shares the compiled
core across fixtures. Generated `build/` trees are excluded from both the
fingerprint and staged source cache, and only the merged image, ELF, build log,
and stamp persist. Use `--rebuild` to force compilation, `--verbose` to see
the compiler output, or the variables listed by `--help` to relocate caches
and select nonstandard tools. After all mutable build work finishes, independent
gates run through a shared bounded process pool with ordered logs. The default
reserves two logical CPUs per gate and caps concurrency at four; set
`FLEXE_FIXTURE_GATE_JOBS=1` for serialized diagnosis or choose another positive
limit for the host.

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
Interactive gates use one bounded incremental output watcher for readiness and
UART markers. It reads only newly appended bytes (and can decode sandbox JSONL
UART events in-stream), replacing tight shell loops that repeatedly rescanned
ever-growing logs and launched thousands of `grep`, `awk`, and `sleep`
processes. The watcher has a wall deadline and monitors the emulator PID, so a
failed guest still terminates with the gate's normal diagnostics.
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
`check-s3-idf-i2s-std.sh` runs the stock standard-I2S driver in interpreter
and JIT concurrently, checking port-0 TX and asynchronously host-fed port-1 RX
through circular GDMA, blocking-task wakeup, audio metadata, exact captured
bytes, and both GPIO-matrix routes with zero unsupported MMIO.
`check-s3-idf-sdmmc-host.sh` reuses the classic SD/MMC endpoint runner to run
the stock S3 host driver and ISR queue in both engines, checking slot-1
command/response, timed IDMAC single- and multi-block media I/O, target GPIO
routes, and zero unsupported MMIO.
`check-s3-idf-twai.sh` similarly reuses the classic CAN-bus endpoint runner
for the stock S3 driver, checking target-specific timing-register widths,
GPIO routes, interrupt queues, alerts, and host-injected frames in both engines.
`check-s3-idf-pcnt.sh` runs the stock pulse-counter driver twice in each
engine, checking GPIO loopback, glitch filtering, direction control, limits,
ISR watch-point order, teardown, JIT retirement, and zero unsupported MMIO.
`check-s3-idf-mcpwm.sh` does the same for both MCPWM groups, covering native
timer/comparator/capture callbacks, GPIO loopback, live compare updates,
continuous force levels, complete teardown, and zero unsupported MMIO.
`check-s3-idf-lcd-i80.sh` runs the native LCD_CAM i80 driver through its
SYSTEM gate, GPIO setup, GDMA command/parameter/color transactions, shared
interrupt callbacks, and teardown in both engines with zero unsupported MMIO.
`check-s3-idf-camera.sh` pins the official `esp32-camera` 2.1.7 component and
runs its unmodified OV2640 SCCB, GPIO, LCD_CAM, circular-GDMA, ISR, camera-task,
full-frame validation, and teardown paths twice in each engine with zero
unsupported MMIO.
`check-s3-idf-adc-continuous.sh` runs ESP-IDF 5.3.2's public continuous-ADC
driver twice in each engine, checking ADC1 pattern decoding, two distinct
host-fed type-2 frames, trigger-8 GDMA ownership, ISR/task notification, the
driver ring buffer, teardown, deterministic replay, and zero unsupported MMIO.
`check-s3-idf-aes.sh` likewise runs the public mbedTLS AES API twice per engine,
covering all six S3 block modes, AES-128/256 known-answer vectors, partial CTR,
and a 4 KiB interrupt-driven GDMA round trip.

Use the fixture builder rather than making disposable build directories by
hand. It finds the pinned ESP-IDF v5.3.2 checkout from `FLEXE_IDF_PATH`, an
active IDF environment, or the conventional `~/esp/esp-idf` install, and
initializes that environment itself. It likewise finds the matching official
ROM in the normal Espressif tools directory; set either path explicitly for a
nonstandard install:

```sh
FLEXE_IDF_PATH=/path/to/esp-idf \
S3_ROM_ELF=/path/to/esp32s3_rev0_rom.elf \
  ./scripts/build-s3-idf-fixture.sh --check all
```

The helper accepts short names for every `tests/fixtures/s3_idf_*` project and
keeps Ninja output and `sdkconfig` files under the user cache, outside the
repository. Fixtures with an `idf_component.yml` are mirrored into that cache,
where `managed_components/` is preserved across edits; they must commit a
`dependencies.lock`, and a build fails if the component manager changes it.
Thus official external components remain content-pinned without dirtying or
bloating the source tree. A content fingerprint covers the project sources,
generated configuration, pinned IDF/tool metadata, build flags, deterministic
timestamp, and both artifact hashes. An unchanged run therefore skips the IDF
environment export and Ninja entirely; all nineteen artifact lookups take under
two seconds on the reference MacBook. A cache miss retains full logs and prints only progress,
exact artifact paths and hashes, and gate results. Pass `--verbose` when live
compiler output and ccache statistics are useful. `--check` also updates just
the selected host emulator and endpoint-harness targets, avoiding stale
runners. `--rebuild` performs a safe IDF full-clean and configure, and should
reproduce both hashes. The helper rejects an accidental ESP-IDF revision
mismatch. It also prevents ESP-IDF's generated Ninja graph from treating the
parent Flexe Git revision as a firmware input and retains the automatically
chosen source epoch per build directory, so committing already-validated
fixture source does not rebuild identical firmware. The cache format is
versioned only when firmware-generation semantics change; adding a fixture
alias, host runner, or behavior gate does not invalidate existing artifacts.
Interrupted configure/builds retain a pending configuration stamp and resume
their valid Ninja tree on the next invocation. When `ccache` is installed,
common ESP-IDF components are shared safely across independent projects:
generated header contents remain part of the cache key, so projects with
different `sdkconfig` values cannot reuse the wrong object. Like the S3 Arduino
builder, it defers read-only behavior gates until every firmware build is done,
then uses the shared `FLEXE_FIXTURE_GATE_JOBS` pool so emulator validation does
not compete with compilation. Set
`FLEXE_IDF_BUILD_ROOT` or
`FLEXE_IDF_CCACHE_DIR` to relocate those caches; the script's `--help` lists
the remaining controls.

The classic `test-fixtures.sh` gate keeps full per-engine loader and device
traces in temporary logs and prints only each successful runner's result line.
On failure it emits both complete logs so diagnostics are not lost. Set
`FLEXE_FIXTURE_VERBOSE=1` when full successful traces are useful. Successful
parallel compiler jobs are likewise quiet by default, while failed jobs retain
their complete output.

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

Use `--strict-mmio` in acceptance gates that must reject every unsupported
peripheral access. Unlike `--unhandled-report`, it only checks the aggregate
counter and therefore leaves the JIT enabled; rerun a failure with
`--unhandled-report` to attribute registers and guest PCs in the interpreter.

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
sources for AArch64, and executes the compiled-firmware gate set. Correctness
jobs disable expensive whole-program linking; normal optimized emulator builds
retain LTO by default.
