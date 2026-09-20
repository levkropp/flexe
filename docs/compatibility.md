# Firmware compatibility

Flexe treats production compatibility as an end-to-end property. A firmware
image must keep executing useful work, interact through modeled hardware or
service boundaries, and produce equivalent results in the interpreter and JIT.
Booting to one UART line is not considered a pass.

## Target selection

Flexe reads the Espressif chip ID and revision bounds from each image header.
`--target auto` is the default; `--target esp32` or `--target esp32s3` turns
the selection into an assertion suitable for CI. Classic ESP32 execution is
supported. ESP32-S3 chip ID `0x0009` has experimental interpreter and JIT
support for the common windowed LX7 instruction profile, native memory map,
flash/cache-MMU windows, mask ROM, dual-core startup, system timer, timer
groups and main watchdogs, SPI-memory controllers,
general-purpose SPI2/SPI3 controllers and bidirectional AHB GDMA,
CPU/system-clock selection, RTC boot-handoff storage, live slow-clock and
power-on reset state, RTC interrupt aggregation and watchdog, a read-only
revision-0 eFuse profile, digital pad configuration, UARTs, native USB
Serial/JTAG, digital GPIO matrix, external I2C controllers, and interrupt
matrix. Its unified SHA accelerator supports direct and GDMA-fed SHA-1,
SHA-224, SHA-256, SHA-384, and SHA-512 blocks.
Run S3 firmware with native FreeRTOS (`-N`) and an official matching ROM ELF
(`-R /path/to/esp32s3_rev0_rom.elf`). Native translation is selected by the
target descriptor rather than a firmware identity or PC list. Unsupported
opcodes fall back to the interpreter; exact ROM/service hooks remain dispatch
boundaries while ordinary code in target-described ROM, IRAM, RTC-fast, and
mapped-flash ranges may compile. Missing S3 devices remain explicit, and S3
is not yet a production-supported target.

Boards populated with the [AP Memory APS6408L-3OBMx 64-Mbit octal
PSRAM](https://www.apmemory.com/en/downloadFiles/0324112221b2583847) can opt
in with `--psram ap-8m-opi`. This attaches 8 MiB on S3 CS1, supports its
16-bit doubled DDR mode-register and array commands, and exposes the backing
through the S3 cache MMU. The stock Arduino-ESP32 3.3.11
`FlashSize=8M,PSRAM=opi` fixture detects 8 MiB, allocates external RAM, and
repeats 8 KiB read/write checks under `scripts/check-s3-psram-opi.sh`.
Without the flag, S3 CS1 remains unpopulated and the same firmware reports
no PSRAM. This is functional byte-array support, not calibrated DQS timing,
refresh behavior, or a claim about other PSRAM devices.

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
commands. Page programs wrap inside their 256-byte physical page. For its
default 4 MiB GigaDevice `C8 40 16` profile, the model follows the
[GD25Q32C datasheet's protection tables](https://download.gigadevice.com/Datasheet/DS-00088-GD25Q32C-Rev4.1.pdf):
BP4..0 and CMP block page programs and whole sector/block erases that touch
the selected range; chip erase is refused if any region is protected.
Unprotected regions remain writable. A different JEDEC
capacity with protection bits set cannot reuse that map; attempted writes
remain diagnostic and leave flash unchanged. The 4 MiB profile also serves
the chip's documented SFDP header and parameter tables through opcode `0x5A`;
other capacities retain an unsupported-command diagnostic rather than
advertising a contradictory 4 MiB density. The NOR's `0x66`/`0x99` reset
sequence clears WEL and volatile modes while retaining nonvolatile status
bits. WP# pin-level locking and security-register protection are not modeled.
The S3-generation `SPI_MEM_ADDR` register keeps a 24-bit user-mode flash
address in bits 23:0; interpreting it as the classic controller's
left-aligned address would silently erase or program the wrong flash sector.
Read/program/erase tests now use the register sequence seen in an unmodified
ESP32-S3 NerdMiner 1.8.3 factory image. That image formats its actual `spiffs`
partition, reports `SPIFS: Mounted`, and completes a scripted portal
configuration/save/restart/reload session in the interpreter. This is a
focused firmware milestone, not an S3 production pass: thousands of
peripheral accesses in that run remain unhandled. The classic ROM cache APIs
validate and apply both flash and
external-RAM mappings against their target backings. Fast mode completes these
operations immediately and does not yet model flash latency, separate
per-core/PID cache contents, bus contention, wear, or interrupted-write power
behavior.

Software resets retain the live NOR backing, including guest-written NVS and
filesystem partitions. They also keep the external NOR's command-visible
status and mode state while resetting the SoC's SPI controller registers.
The optional S3 PSRAM array and mode registers likewise survive an SoC-only
reset; a chip power cycle resets its mode registers.
On S3 [deep-sleep flash power-down](https://docs.espressif.com/projects/esp-idf/en/v5.2/esp32s3/api-reference/system/sleep_modes.html#power-down-of-flash),
the modeled GD25Q32C retains only its documented nonvolatile status bits;
other capacity profiles do not claim a nonvolatile register layout and emit a
diagnostic if potentially nonvolatile status would be lost. Flexe
reloads internal-memory application segments and rebuilds peripherals without
copying the original factory/app file over flash. Factory and standalone
app-image paths have regression tests for this distinction. Changes to the
originally loaded application bytes are rejected
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
inputs, host-driven digital inputs, IO_MUX `FUN_IE` gating of digital reads,
matrix inputs and interrupts, driven-output input feedback, edge/level status
latching, W1TS/W1TC aliases, and the shared normal/NMI sources routed through
both cores' target interrupt matrices. The virtual target currently supplies
zero-valued strap inputs. Unattached peripheral matrix outputs and BT/SDIO
pad ownership are not modeled. Open-drain net voltage, input synchronizer/filter
timing, and clock-gate effects are also absent. Digital open-drain release is
modeled, but a floating pad without a host sample defaults low; host-injected
samples override output feedback. Pulls, drive strength, contention, and other
pad electrical behavior remain outside the current IO_MUX/RTCIO and board/net
models. The S3 RTCIO bank now models GPIO0..21
pad-owner selection, RTC output and enable latches, W1TS/W1TC aliases, and
sampled host-driven input. When an RTC pad owns the pin, digital output latches
remain writable but no longer drive the reported physical pin. Unmodeled RTC
pad functions report unknown output and retain diagnostics; RTC GPIO
interrupts remain unsupported. EXT0/EXT1 wake has a native ESP-IDF
replay gate. This follows Espressif's
[S3 RTC GPIO mapping](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-reference/peripherals/gpio.html).
This is useful functional GPIO support for the
experimental S3 target, not hardware-calibrated timing or electrical evidence.
S3 RTC and digital pad-hold registers are connected to the shared pad model:
RTC GPIO0..21 and digital GPIO21..47 retain physical output level and enable,
RTC owner mux and input-enable selection while the register latches keep
accepting writes. GPIO21 remains held until both hold sources are cleared.
Held pad state also survives a session reset's machine rebuild without
retaining unheld GPIO state. Unbonded and undocumented hold bits remain
diagnostic.
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
readback for peripheral clock/reset controls. Its low-sleep memory-power mask
now publishes whether SoC-memory power-down is allowed, and its Wi-Fi/Bluetooth
low-power clock publishes all source controls plus the exact integer and
fractional-divider tuple. SYSTIMER and both timer groups pause at
exact clock-gate boundaries, and their independent reset pulses restore device
state and interrupt lines. The external I2C and GP-SPI controllers and the
session-owned SHA accelerator also consume their target-described clock/reset
gates; SHA commands cannot complete with its clock off or reset asserted, and
a reset pulse clears its register and digest state. Both complete peripheral
clock/reset banks are available as a masked 64-domain state surface for future
consumers; domains without an attached device model do not imply functional
reset or clock effects. The memory/radio state is
available to downstream consumers without fabricating memory loss, RF, or
radio-controller behavior that is not modeled yet. Other SYSTEM fields remain
explicit unsupported-access diagnostics unless they have a semantic consumer.
Target-described UART TX and GP-SPI clock/data/chip-select producers are also
valid GPIO-matrix routes across firmware builds. UART exports complete bytes
and an idle-high pad, while GP-SPI exports complete transactions; fast mode
does not fabricate baud or serial-clock edges.

The S3 SYSCON memory-policy owner supplies exact reset and masked readback for
the RF front-end controls and the 11 SRAM/3 ROM bank clock, force-down, and
force-up policies. It publishes normalized semantic state for cache and power
consumers and composes with the radio owner on their shared MMIO page. Flexe
does not yet model cache timing or erase backing memory when firmware requests
that a bank be powered down.

The SENSITIVE v1 model likewise retains the cache-data-array and internal-SRAM
allocation policy with documented reset values, reserved-bit masks, and sticky
configuration locks. Its normalized state covers CPU and cache use, trace
allocation, MAC-dump/log use, and retention disable. Cache topology and access
latency are not yet derived from that policy, and protection violations are not
yet generated from the broader retained memory-protection register set.

The S3 ASSIST_DEBUG recorder is also a target-described device rather than a
startup-address exception. Its per-core PDEBUG and recording controls expose
live PC/SP state while active and freeze the latest sample when recording is
stopped, which supports the standard ESP-IDF crash-record setup without a
per-instruction performance tax. Stack/area watchpoint interrupts, detailed
debug-bus fields, exception records, and trace memory are still unsupported and
continue through the diagnostic fallback.

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
original classic display/SD paths. GDMA completion/error status now drives
the [S3's separate level interrupt source for each RX and TX channel](https://github.com/espressif/esp-idf/blob/v5.3.2/components/soc/esp32s3/include/soc/interrupts.h),
including both CPU interrupt matrices and late enable/clear transitions.
Slave mode, segmented/config-buffer transactions, bus arbitration, signal
edges, and clock-derived transfer duration remain outside this functional envelope.
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
with device state transitions. NerdMiner now passes an interactive portal,
SPIFFS save, software restart, and configuration-reload scenario; a separately
built WLED S3 image passes an AP web UI and JSON state-change scenario through
its own raw-lwIP stack. Fast mode
completes each block immediately; SHA/GDMA
latency, arbitration, GDMA CPU interrupt delivery, and SHA-512/224, SHA-512/256,
and configurable SHA-512/t are not yet modeled.
Requests for the unsupported SHA modes or malformed DMA chains are rejected
with a diagnostic rather than returning invented digest data.

The S3 RTC counter advances on the same shared dual-core virtual timeline as
the other target-described timers. Its two-half latch, runtime CPU-frequency
scaling, and switching among the nominal 136 kHz RC slow, 32.768 kHz crystal,
and RC-fast/256 sources are modeled. The independent fast-clock mux reports
the selected 20 MHz XTAL/2 or nominal 17.5 MHz RC_FAST source, and the S3
DATE/LDO-trim word has exact reset and masked readback. Oscillator drift,
calibration error, analog trim effects, and analog transition timing are not
modeled. OPTIONS0's oscillator/I2C supplies, isolation/reset pairs and crystal
wait plus ANA_CONF's analog enables/reset-POR pair publish normalized logical
state; DIG_ISO does the same for pad/global isolation and autohold policy.
Unattached consumers do not turn those controls into invented electrical or RF
behavior. The RTC
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
visible in the same diagnostics, as do reserved register bits.

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
