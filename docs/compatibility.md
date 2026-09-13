# Firmware compatibility

Flexe treats production compatibility as an end-to-end property. A firmware
image must keep executing useful work, interact through modeled hardware or
service boundaries, and produce equivalent results in the interpreter and JIT.
Booting to one UART line is not considered a pass.

## Target selection

Flexe reads the Espressif chip ID and revision bounds from each image header.
`--target auto` is the default; `--target esp32` or `--target esp32s3` turns
the selection into an assertion suitable for CI. Classic ESP32 execution is
supported. ESP32-S3 chip ID `0x0009` has experimental interpreter support for
the LX7 core, native memory map, flash/cache-MMU windows, mask ROM, dual-core
startup, system timer, timer groups and main watchdogs, SPI-memory controllers,
general-purpose SPI2/SPI3 controllers and bidirectional AHB GDMA,
CPU/system-clock selection, RTC boot-handoff storage, live slow-clock and
power-on reset state, RTC interrupt aggregation and watchdog, a read-only
revision-0 eFuse profile, digital pad configuration, UARTs, native USB
Serial/JTAG, digital GPIO matrix, external I2C controllers, and interrupt
matrix. Its unified SHA accelerator supports direct and GDMA-fed SHA-1,
SHA-224, SHA-256, SHA-384, and SHA-512 blocks.
Run S3 firmware with native FreeRTOS (`-N`) and an official matching ROM ELF
(`-R /path/to/esp32s3_rev0_rom.elf`). The classic compatibility services and
JIT are deliberately not composed into S3 sessions: their ABI and fixed ROM
addresses belong to the classic target. Missing S3 devices remain explicit
and S3 is not yet a production-supported target.

Flexe reads the configured flash capacity from the standard ESP image header
and grows the virtual NOR device beyond the 4 MiB board default when required
(up to 16 MiB on classic ESP32 and 128 MiB on ESP32-S3). It recognizes merged
S3 factory images even when an ESP bootloader header is at flash offset zero:
the target's partition table and matching application header distinguish them
from standalone apps. The application runs with the merged image's real
partition and flash contents, not a synthesized layout. The SPI controller's
JEDEC capacity byte, raw backing, cache-MMU bounds, SDK size queries, and the
ROM's live boot-handoff structure then describe the same device. Newer ROM
handoff pointers are resolved from the official ROM ELF, while older fixed
ROM ABI addresses remain target data. The functional SPI-memory model supports
raw reads, NOR page programming, sector/block/chip erase, status and power-down
commands. The S3-generation `SPI_MEM_ADDR` register keeps a 24-bit user-mode
flash address in bits 23:0; interpreting it as the classic controller's
left-aligned address would silently erase or program the wrong flash sector.
Read/program/erase tests now use the register sequence seen in an unmodified
ESP32-S3 NerdMiner 1.8.3 factory image. That image formats its actual `spiffs`
partition, reports `SPIFS: Mounted`, and reaches its configuration portal in
the interpreter. This is a focused firmware milestone, not an S3 production
pass: roughly 7,000 peripheral accesses in that run remain unhandled, and
portal interaction has not been validated. The classic ROM cache APIs
validate and apply both flash and
external-RAM mappings against their target backings. Fast mode completes these
operations immediately and does not yet model flash latency, separate
per-core/PID cache contents, bus contention, wear, or interrupted-write power
behavior.

Software resets retain the live NOR backing, including guest-written NVS and
filesystem partitions. Flexe reloads internal-memory application segments
and rebuilds peripherals without copying the original factory/app file over
flash. Factory and standalone app-image paths have regression tests for this
distinction. Changes to the originally loaded application bytes are rejected
with an OTA diagnostic. Selecting and booting a different OTA slot is not
modeled yet and must not be treated as a verified OTA workflow. Flash state
remains in the emulator session; it is not automatically written back to the
input `.bin` or retained across a new process.

Fast mode also recognizes complete relocated ESP-IDF 4.x critical-section
bodies and their standard heap lock wrappers structurally. It fuses only the
uncontended or recursive internal-RAM path when the full instruction span is
clear of timer, scheduler, interrupt, debugger, and register-window boundaries.
Lock contention, finite timeouts, external RAM, unfamiliar SDK code, and every
failed validation execute from the original firmware. Differential tests
compare the accelerated and interpreted paths across both cores, all windowed
call sizes, lock nesting, register state, memory effects, and boundary
fallbacks; no application name or linked address authorizes the optimization.

The S3 GPIO model implements the S2/S3-generation register layout, both data
and enable banks, package-valid GPIO0..48 (including the GPIO22..25 holes),
software-output selection and inversion, matrix input selection and constant
inputs, host-driven digital inputs, edge/level status latching, W1TS/W1TC
aliases, and the shared normal/NMI sources routed through both cores' target
interrupt matrices. The virtual target currently supplies zero-valued strap
inputs. Peripheral-produced matrix output levels, BT/SDIO pad ownership,
open-drain electrical resolution, input synchronizer/filter timing, GPIO wake,
and clock-gate effects are not modeled yet; selecting those behaviors produces
an unsupported-access diagnostic instead of an invented result. Pulls, drive
strength, and other pad electrical behavior remain part of the separate IO_MUX
and future board/net model. This is useful functional GPIO support for the
experimental S3 target, not hardware-calibrated timing or electrical evidence.
S3 RTC digital-pad hold is now connected to the GPIO output model: held
GPIO21..47 retain their physical output level and enable while the GPIO
latches keep accepting writes, and release reveals the current latches.
Held outputs also survive a session reset's machine rebuild without retaining
unheld GPIO state. Unbonded and undocumented hold bits remain diagnostic.
Global pad-force and auto-hold controls, the deep-sleep wake sequence, and
electrical drive effects are not yet modeled; a register readback alone is
not a claim that those behaviors work.

The descriptor-driven S3 timer-group model implements both 54-bit general-
purpose timers per group, APB/XTAL clock selection and division, software
capture/load, one-shot and autoreload alarms, interrupt status and routing,
and the four-stage main watchdog with prescaling, feed, write protection,
interrupt, and reset actions. It runs Espressif's unmodified ESP-IDF 5.3.2
`gptimer` example through its stop, autoreload, and dynamically rearmed alarm
scenarios at the requested 1 MHz resolution and one-second alarm cadence. This
remains functional fast-mode timing: silicon-calibrated interrupt latency is
not modeled yet, and an MWDT CPU-reset action currently requests the same
whole-machine reset as a system-reset action.

The target-described S3 SYSTEM bank exposes documented reset state and masked
readback for peripheral clock/reset controls, low-sleep memory power masking,
and Bluetooth low-power-clock division. SYSTIMER and both timer groups pause at
exact clock-gate boundaries, and their independent reset pulses restore device
state and interrupt lines. The external I2C and GP-SPI controllers and the
session-owned SHA accelerator also consume their target-described clock/reset
gates; SHA commands cannot complete with its clock off or reset asserted, and
a reset pulse clears its register and digest state. Effects for other peripheral,
memory-power, and Bluetooth clock fields remain explicit unsupported-access
diagnostics even though their architectural register values are retained.

The external I2C model is shared by classic ESP32 and ESP32-S3 through target
descriptors rather than fixed addresses or command encodings. It implements
both native instances, their FIFO command lists, master writes and repeated-
start reads, address NACKs, raw/enable/status/clear interrupt state, native
interrupt routing, host-attachable bus devices, and guest-slave transfers. The
S3 descriptor supplies its eight-command depth, different HAL opcodes, native
interrupt sources, peripheral identity, and independent SYSTEM clock/reset
gates. Tests cover both target layouts, and an S3 application built with the
official ESP-IDF 5.3.2 toolchain and legacy production `driver/i2c.h` path
reaches `app_main`, installs the driver, and completes an absent-device
transfer with a NACK instead of timing out. This is functional fast-mode
support: SCL/SDA edge timing, timing-register effects, arbitration, clock
stretching, multi-master contention, electrical line resolution, and error
injection are not modeled yet.

The GP-SPI model is likewise selected by target capability and descriptor, not
by firmware identity. Classic ESP32 and ESP32-S3 each supply their native
SPI2/SPI3 bases, register generation, interrupt sources, GPIO-matrix and IOMUX
routes, chip-select count, clock/reset gates, and DMA trigger IDs. In `fast`
mode, CPU-FIFO polling transfers and S3 AHB-GDMA transmit/receive descriptor
chains complete synchronously, including ownership and length write-back,
EOF/error status, software interrupt set/clear, and SPI completion routing.
Board models attach at the controller boundary; the CYD panel, touch, and SD
card are consumers of that API rather than conditions inside the SoC map.
Automated tests cover both hosts, matrix and native routes, clock/reset
isolation, full-duplex DMA, malformed and exhausted receive chains, and the
original classic display/SD paths. Slave mode, segmented/config-buffer
transactions, GDMA interrupt delivery, bus arbitration, signal edges, and
clock-derived transfer duration remain outside this functional envelope.
Unsupported framing and invalid DMA setup are diagnosed instead of silently
reported as successful transfers.

The S3 SHA model uses target-described mode mappings and consumes the active
AHB GDMA v1 transmit chain selected for SHA, including chained descriptors,
length and EOF validation, optional owner checking and write-back, and
completion status. The shared GDMA model also accepts receive streams from
modeled peripherals such as GP-SPI. It therefore works for stripped firmware
without an ELF symbol or firmware-specific hook. The S3 SYSTEM clock and reset
bits govern SHA execution and reset state; direct and GDMA commands do no work
while gated or held in reset. In an unmodified S3 NerdMiner application run,
this replaces roughly 131,000 repeated clock/reset unsupported diagnostics
with device state transitions. The firmware now reaches its configuration
portal, but has not passed an interactive end-to-end S3 scenario. Fast mode
completes each block immediately; SHA/GDMA
latency, arbitration, GDMA CPU interrupt delivery, and SHA-512/224, SHA-512/256,
and configurable SHA-512/t are not yet modeled.
Requests for the unsupported SHA modes or malformed DMA chains are rejected
with a diagnostic rather than returning invented digest data.

The S3 RTC counter advances on the same shared dual-core virtual timeline as
the other target-described timers. Its two-half latch, runtime CPU-frequency
scaling, and switching among the nominal 136 kHz RC slow, 32.768 kHz crystal,
and RC-fast/256 sources are modeled. Oscillator drift, calibration error,
sleep continuity, reset causes other than initial power-on, and the electrical
effects of the other RTC clock-control fields are not yet modeled. The RTC
interrupt bank implements target-described enable/raw/masked-status/W1C state
and level routing; physical producers such as brownout, touch, and ULP remain
unsupported until their respective device models attach to that API. The
four-stage RTC watchdog runs from the selected slow clock and implements the
revision-profile stage-0 multiplier, feed, write protection, interrupt, and
reset actions. Watchdog CPU/system/RTC reset actions currently converge on a
whole-machine reset; their distinct reset domains and post-reset causes are
not yet modeled. Pause-in-sleep, reset-signal widths, per-core reset selection,
and reserved stage actions remain explicit unsupported-access diagnostics when
firmware changes them. Changing any other unmodeled RTC control field remains
visible in the same diagnostics.

The experimental USB Serial/JTAG model implements the 64-byte serial endpoint
FIFOs, packet flush and backpressure behavior, host RX/TX, interrupt
enable/status/clear routing, connection state, and fast-mode SOF liveness.
`--usb-console` routes CLI output from this endpoint instead of UART0. It does
not yet implement JTAG transport, USB descriptors, line signaling, or timed
1 ms SOF generation; fast mode coalesces connected-host SOFs when firmware
observes the controller.

## Current corpus

| Firmware | Engines | Result | Remaining work |
|---|---|---|---|
| ESP32 Marauder 1.12.1 (CYD 2432S028 2USB) | Interpreter + JIT | Pass | None in the scripted scenario; requires the official ESP32 ROM ELF |
| ESP32 Marauder 1.14.3 (CYD) | Interpreter + JIT | Pass | None in the scripted scenario |
| ESP32 Marauder 1.15.1 (CYD 2432S028) | Interpreter + JIT | Pass | None in the scripted scenario |
| ESP32 Marauder 1.14.3 (3.5-inch and Guition variants) | Interpreter + JIT | Pass | None in the scripted scenario |
| NerdMiner 1.8.3 | Interpreter + JIT | Pass | None in the scripted scenario |
| Bruce 1.16.1 (CYD 2432S028) | Interpreter + JIT | Pass | Expand radio interaction coverage; requires the official ESP32 ROM ELF |
| Meshtastic 2.7.26 (T-Beam) | Interpreter + JIT | Pass | None in the scripted scenario; requires the official ESP32 ROM ELF |
| openHASP 0.7.0-rc13 (Lanbon L8) | Interpreter + JIT | Pass | Expand FT6336 touch and HTTP/MQTT interaction coverage |
| Tasmota 15.6.0 | Interpreter + JIT | Pass | HTTP Status and Berry-to-GPIO are covered; expand MQTT and device drivers |
| WLED 16.0.1 | Interpreter + JIT | Pass | DNRGB-to-RMT is covered; expand HTTP, DDP, and E1.31 coverage |

ROM images are not stored in this repository. Results are tied to the image
versions above and should be rechecked when a release changes.

Firmware that uses controller data embedded in the mask ROM (notably newer
ESP-IDF Bluetooth builds) needs Espressif's official ESP32 ROM ELF. Pass it as
`-R /path/to/esp32_rev300_rom.elf`, `--rom-elf` to either ROM test runner, or set
`FLEXE_ROM_ELF`. Flexe loads immutable ROM sections and their linker-described
data images. Hardware-facing shims retain priority, while other mask-ROM calls
execute the loaded instructions. The ROM binary is intentionally not copied
into this repository.

## Curated CYD scenarios

`scripts/check-stock-roms.sh` drives unmodified Bruce, Marauder, Meshtastic,
NerdMiner, openHASP, Tasmota, and WLED images through board-level scenarios on
both engines. It checks completion, modeled I/O, scenario-specific effects,
and matching output digests. Known official image hashes have pinned output
digests, so a model change is explicit.

The Bruce scenario covers:

- ILI9341 rendering and a menu transition through its GPIO-bit-banged XPT2046
- XPT2046 PENIRQ delivery on GPIO36
- SD-card initialization and FAT access

The Marauder scenario covers:

- ILI9341 rendering and XPT2046 touch navigation
- UART0 commands and independent UART2 GPS traffic
- SD-card initialization and filesystem access
- Wi-Fi promiscuous receive and raw-frame transmit boundaries
- BLE scan callbacks and advertising payloads

The Meshtastic scenario covers:

- T-Beam AXP192 power detection and rail configuration
- SX1276 discovery and complete LoRa radio initialization over board SPI
- serial API configuration, encrypted LoRa transmit, and loopback receive
  through the SX1276 FIFO and the firmware's RadioLib interrupt path
- NimBLE host/controller synchronization and BLE advertising
- u-blox NEO-M8 detection, UBX configuration/ACK traffic, and an accepted
  NMEA position and time fix over UART1

The NerdMiner scenario covers:

- Wi-Fi association and ESP event delivery
- captive-portal HTTP and DNS traffic
- host-backed pool networking and the mining task path

The openHASP scenario covers:

- ST7789V rendering over the Lanbon L8's production CS22/DC21/SCLK19 bus
- a host TCP client connected to the firmware's production lwIP/Telnet service
- three JSONL commands through ConsoleInput, the dispatcher, LVGL, and the
  resulting deterministic RGB framebuffer

The Tasmota scenario covers:

- access-point startup and the production HTTP service bound through lwIP
- a host `Status 0` request through Tasmota's command dispatcher
- Berry commands that drive GPIO4 high and low through the modeled GPIO API

The WLED scenario covers:

- access-point startup and the production WiFiUDP socket bound on port 21324
- a host-sent DNRGB frame parsed by WLED's own realtime protocol path
- the resulting 30-pixel GRB stream through RMT ping-pong refill interrupts
  and the completed 40 MHz WS2812 waveform endpoint

Run the scenarios with:

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

Set `FLEXE_ROM_ELF=/path/to/esp32_rev300_rom.elf` when the selected image
requires the official mask-ROM data described above.

Additional positional images use the Marauder profile and still require both
engines to agree.

## Arbitrary firmware corpus

`scripts/check-firmware.sh` applies a deliberately generic probe to every
`.bin` passed on the command line or found directly under `FLEXE_ROMS`. It
requires each engine to:

- reach the requested virtual-time budget without trapping or stopping early
- retire enough instructions to demonstrate useful execution
- avoid unhandled MMIO and unregistered ROM calls
- stay below a bounded number of unmapped-memory probes
- end on a fetchable guest PC
- produce the same normalized UART transcript digest

```sh
FLEXE_ROMS=/path/to/corpus ./scripts/check-firmware.sh
```

`BATCH` controls the dual-core scheduling quantum and `MAX_UNMAPPED` controls
the unmapped-access ceiling; their conservative defaults are 10,000 and 1,000.

Recent ESP-IDF PHY libraries access the classic ESP32 peripheral fabric through
its `0x60000000` AHB-Lite mirror. Flexe aliases the complete 256 KiB window to
the corresponding `0x3ff40000` APB registers, including the private Bluetooth
and Wi-Fi controller pages. RTC slow RAM remains distinct at `0x50000000`.

## Architectural register windows

Flexe executes the firmware's own WindowOverflow and WindowUnderflow vectors
by default. This is the ESP32 architectural path and keeps `setjmp`/`longjmp`,
dynamic stack allocation, borrowed callback frames, and wrapped register files
on the same ABI save areas the hardware uses. It is what allows Marauder,
Tasmota, WLED, and openHASP to pass the same configuration.

`FLEXE_WINDOW_VECTORS=0` selects the legacy synthesized spill/fill path for
diagnosis. `FLEXE_SHADOWFILL=1` separately enables the old shadow-record restore
shortcut; it is also off by default.

## Firmware-specific hooks

Address-based Wi-Fi and Bluetooth hooks are accepted only for verified image
fingerprints. Candidate entry points must be verified against a symbol-bearing
build made with the same ESP-IDF or Arduino core. `build/xt-dis` disassembles a
stripped app image at its guest load address (`-a ADDR -n LEN firmware.bin`).
