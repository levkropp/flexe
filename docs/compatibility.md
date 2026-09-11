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
CPU/system-clock selection, RTC boot-handoff storage, digital pad
configuration, UARTs, native USB Serial/JTAG, and interrupt matrix.
Run S3 firmware with native FreeRTOS (`-N`) and an official matching ROM ELF
(`-R /path/to/esp32s3_rev0_rom.elf`). The classic compatibility services and
JIT are deliberately not composed into S3 sessions: their ABI and fixed ROM
addresses belong to the classic target. Missing S3 devices remain explicit
and S3 is not yet a production-supported target.

The descriptor-driven S3 timer-group model implements both 54-bit general-
purpose timers per group, APB/XTAL clock selection and division, software
capture/load, one-shot and autoreload alarms, interrupt status and routing,
and the four-stage main watchdog with prescaling, feed, write protection,
interrupt, and reset actions. It runs Espressif's unmodified ESP-IDF 5.3.2
`gptimer` example through its stop, autoreload, and dynamically rearmed alarm
scenarios at the requested 1 MHz resolution and one-second alarm cadence. This
remains functional fast-mode timing: peripheral clock/reset
gating and silicon-calibrated interrupt latency are not modeled yet, and an
MWDT CPU-reset action currently requests the same whole-machine reset as a
system-reset action.

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
