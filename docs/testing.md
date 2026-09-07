# Testing

Flexe uses layered tests because instruction correctness, peripheral behavior,
and a successful production boot catch different classes of defects.

## Unit and differential suite

```sh
cmake -S . -B build
cmake --build build -j
./build/xtensa-tests
```

The suite covers instruction decode and execution, memory translation,
register windows, exceptions, interrupts, peripheral registers and timing,
FreeRTOS/service stubs, and both JIT backends. The encoding-space sweep compiles
thousands of instruction forms and compares their architectural effects with
the interpreter.

For host-memory validation:

```sh
cmake -S . -B build-asan \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -DNATIVE_ARCH=OFF \
  -DCMAKE_C_FLAGS="-fsanitize=address,undefined -fno-omit-frame-pointer" \
  -DCMAKE_EXE_LINKER_FLAGS="-fsanitize=address,undefined"
cmake --build build-asan -j
ASAN_OPTIONS=detect_leaks=1 UBSAN_OPTIONS=halt_on_error=1 \
  ./build-asan/xtensa-tests
```

## Compiled-firmware hardware gates

The fixtures under `tests/fixtures/` are real Arduino-ESP32 sketches. Their C
runners under `tools/` attach host endpoints, inject input, and assert the
guest's observable output. The consolidated entry point builds and runs every
fixture with both the JIT and interpreter:

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
| `FLEXE_FIXTURE_BUILD_ROOT` | Persistent root for compiled fixture artifacts |

Without a persistent fixture root, each sketch is built in a temporary
directory and removed after its gate finishes.

## Production ROM gates

The curated scenarios require external images:

```sh
MARAUDER_BIN=/path/to/marauder.bin \
NERDMINER_BIN=/path/to/nerdminer.bin \
./scripts/check-stock-roms.sh
```

For a broader directory of images:

```sh
FLEXE_ROMS=/path/to/corpus ./scripts/check-firmware.sh
```

See [Firmware compatibility](compatibility.md) for the assertions and current
known failures.

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
