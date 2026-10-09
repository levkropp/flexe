<p align="center">
  <img src="docs/assets/flexe-banner.svg" alt="flexe — free little xtensa emulator. esp32 + esp32-s3, written in c, with interpreter and jit engines." width="960">
</p>

<p align="center">
  <a href="https://levkropp.github.io/flexe/">the little website</a> ·
  <a href="docs/compatibility.md">what runs</a> ·
  <a href="ARCHITECTURE.md">how it works</a> ·
  <a href="docs/testing.md">testing</a> ·
  <a href="docs/performance.md">the numbers</a>
</p>

<a id="flexe"></a>

# hello, flexe

a free little xtensa emulator for esp32/lx6 and esp32-s3/lx7. written in c,
with a reference interpreter and tracing jit backends for arm64 and x86-64.

feed it unmodified esp-idf or arduino firmware. give it virtual peripherals,
a board, and some input. watch the firmware do useful work.

<a id="highlights"></a>

## what's in the box

| a little piece | what it does |
|---|---|
| real firmware | bruce, marauder, meshtastic, nerdminer, openhasp, tasmota, and wled have scripted scenarios in both engines |
| a speedy core | register windows, integer and floating-point execution, loops, exceptions, and interrupts; hot code gets a native jit |
| a virtual board | shared memory, flash/mmu, optional psram, two cores, and target-described peripherals |
| things to interact with | display and touch, storage, serial, buses, audio, gpio, and host-backed network services |
| checks that matter | cpu differential tests, stock-driver fixtures, firmware interactions, and matching output digests |

esp32 and esp32-s3 are supported **functional** targets. timing, electrical,
and rf limits still apply; the [hardware matrix](docs/hardware-completeness.md)
tracks the exact boundary. the two cores take turns on one shared timeline.

<a id="build"></a>

## give it a spin

you'll need a c17 compiler, cmake, openssl, zlib, and pthreads.

```sh
git clone https://github.com/levkropp/flexe.git
cd flexe
cmake -S . -B build
cmake --build build --target xtensa-emu -j

# jit is on by default on arm64 and x86-64
./build/xtensa-emu firmware.bin

# symbols make debugging and service hooks more useful
./build/xtensa-emu -s firmware.elf -c 10000000 firmware.bin
```

<a id="run-firmware"></a>

for esp32-s3, use the firmware's own freertos and a matching official rom elf:

```sh
./build/xtensa-emu --target esp32s3 -N \
  -R /path/to/esp32s3_rev0_rom.elf firmware.bin
```

roms and production firmware are supplied separately.
[the compatibility guide](docs/compatibility.md#target-selection) explains
target detection, rom requirements, and optional psram.

<details>
<summary>build notes, including macos</summary>

with homebrew openssl:

```sh
cmake -S . -B build -DOPENSSL_ROOT_DIR="$(brew --prefix openssl@3)"
cmake --build build --target xtensa-emu -j
```

release builds use lto and host-native tuning. use `-DNATIVE_ARCH=OFF` for
portable binaries, `-DFLEXE_LTO=OFF` for faster edit/test links, or
`-DFLEXE_CCACHE=OFF` to disable automatic compiler caching.

s3 ethernet host forwarding additionally needs libslirp 4.9+.
test and fixture runners are built separately when their scripts need them.

</details>

<a id="architecture"></a>

## meet the machine

![a session connects target data, a firmware loader, execution engines, shared memory, device models, and host endpoints.](docs/assets/architecture-system.svg)

a session owns the machine. target descriptors choose its layout; the
interpreter defines cpu behavior; the jit accelerates eligible hot blocks.
board devices and host endpoints attach through controller APIs.

[walk through the architecture →](ARCHITECTURE.md)

<a id="production-status"></a>

## what runs?

the classic corpus covers seven firmware families, including display/touch,
serial, storage, network, radio-service, and led scenarios. s3 has its own
pinned production and stock-driver gates.

a pass belongs to a **specific image and scenario**. a service-backed network
workflow and a modeled hardware controller have different support boundaries.

[see versions, scenarios, and remaining gaps →](docs/compatibility.md)

## a few useful switches

| switch | what it's for |
|---|---|
| `--no-jit` | compare with the interpreter |
| `--jit-stats` / `--jit-verify` | inspect native coverage / compare replayable blocks |
| `-s ELF` / `-R ROM_ELF` | load application symbols / official mask-rom code and data |
| `--efuse BLOB` | override s3 mac and chip revision from a 336-byte efuse blob |
| `--strict-mmio` | fail on unsupported peripheral accesses while keeping the jit enabled |
| `--unhandled-report` | attribute unsupported accesses to registers and guest pcs in the interpreter |
| `--sandbox-events` | exchange peripheral events and host input as ndjson |
| `--usb-console` | use the native usb serial/jtag console |
| `-c N` / `-b ADDR` | set a cycle budget / breakpoint |
| `-T` / `-m ADDR[:LEN]` | trace instructions / dump memory |
| `-q` | suppress emulator diagnostics |

<a id="test"></a>

## keep it honest

```sh
cmake --build build --target xtensa-tests -j
./scripts/run-unit-tests.sh
./scripts/test-fixtures.sh spi-master i2c-wire
FLEXE_ROMS=/path/to/roms ./scripts/check-firmware.sh
```

[testing](docs/testing.md) covers focused suites, stock drivers, sanitizer
builds, and production gates.

<a id="performance"></a>

## how fast?

on the documented apple-silicon runs, the repeated classic wled gate measured
**1.482× interpreted / 3.522× jit**. the s3 wled gate measured
**1.34–1.49× / 2.29–2.43×**, with exact frame parity.

those are workload and host measurements. real-time factor measures guest
time; retired mips measures executed instructions. idle jumps count only
toward time.

[dated results and reproducible commands →](docs/performance.md)

## license

mit licensed. made with excessive respect for a little cpu.
