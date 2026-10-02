# peripheral implementation notes

start with [what runs](compatibility.md) for firmware results and setup.
this reference keeps the detailed register behavior, evidence, and limits behind
topic summaries. the [hardware matrix](hardware-completeness.md#current-capability-matrix)
tracks the current acceptance boundary and newer driver gates.

- [target and execution](#target-and-execution)
- [optional psram](#optional-psram)
- [flash and reset persistence](#flash-and-reset-persistence)
- [execution acceleration](#execution-acceleration)
- [gpio and pad ownership](#gpio-and-pad-ownership)
- [timer groups](#timer-groups)
- [system clocks and resets](#system-clocks-and-resets)
- [syscon memory policy](#syscon-memory-policy)
- [sram and cache allocation](#sram-and-cache-allocation)
- [assist debug](#assist-debug)
- [external i2c](#external-i2c)
- [gp-spi and gdma](#gp-spi-and-gdma)
- [sd and mmc](#sd-and-mmc)
- [twai and can](#twai-and-can)
- [i2s audio](#i2s-audio)
- [sha](#sha)
- [aes](#aes)
- [rtc clocks and watchdog](#rtc-clocks-and-watchdog)
- [usb serial and jtag](#usb-serial-and-jtag)

## target and execution

image headers select the target; s3 uses native freertos and a matching official rom elf.

<details>
<summary>register behavior, evidence, and limits</summary>

Flexe reads the Espressif chip ID and revision bounds from each image header.
`--target auto` is the default; `--target esp32` or `--target esp32s3` turns
the selection into an assertion suitable for CI. Classic ESP32 execution is
supported. ESP32-S3 chip ID `0x0009` supports functional interpreter and JIT
execution for the common windowed LX7 instruction profile, native memory map,
flash/cache-MMU windows, mask ROM, dual-core startup, system timer, timer
groups and main watchdogs, SPI-memory controllers,
general-purpose SPI2/SPI3 controllers and bidirectional AHB GDMA,
I2S0/I2S1 v2 controllers with timed circular-GDMA transport,
APB_SARADC continuous ADC1 pattern scans with host-fed GDMA frames,
the native SD/MMC host with timed internal DMA and host-backed block media,
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
mapped-flash ranges may compile. Missing S3 devices remain explicit. This is a
supported functional target, not a claim of cycle, RF, electrical, or complete
peripheral equivalence; the exact boundary is tracked in the
[hardware-completeness matrix](hardware-completeness.md#current-capability-matrix).

</details>

## optional psram

an opt-in 8 mib octal device supplies array/mode-register behavior; electrical timing is outside the model.

<details>
<summary>register behavior, evidence, and limits</summary>

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

</details>

## flash and reset persistence

image geometry, partitions, nor commands, protection, mmu mapping, and reset retention share one backing. alternate ota-slot boot is unsupported.

<details>
<summary>register behavior, evidence, and limits</summary>

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
configuration/save/restart/reload session in the interpreter. That earlier
filesystem-only milestone still had thousands of unhandled peripheral accesses.
The current full portal/save/restart/reload replay requires zero unsupported
MMIO sites; see the hardware-completeness matrix for the updated gate.

The classic ROM cache APIs
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

</details>

## execution acceleration

complete instruction signatures admit only validated critical-section and heap-lock paths. contention and failed guards return to guest code.

<details>
<summary>register behavior, evidence, and limits</summary>

Fast mode also recognizes complete relocated ESP-IDF 4.x critical-section
bodies and their standard heap lock wrappers structurally. It fuses only the
uncontended or recursive internal-RAM path when the full instruction span is
clear of timer, scheduler, interrupt, debugger, and register-window boundaries.
Lock contention, finite timeouts, external RAM, unfamiliar SDK code, and every
failed validation execute from the original firmware. Differential tests
compare the accelerated and interpreted paths across both cores, all windowed
call sizes, lock nesting, register state, memory effects, and boundary
fallbacks; no application name or linked address authorizes the optimization.

</details>

## gpio and pad ownership

digital routing, input feedback, rtc ownership, interrupts, and pad hold are functional. analog/electrical behavior remains outside the claim.

<details>
<summary>register behavior, evidence, and limits</summary>

The S3 GPIO model implements the S2/S3-generation register layout, both data
and enable banks, package-valid GPIO0..48 (including the GPIO22..25 holes),
software-output selection and inversion, matrix input selection and constant
inputs, host-driven digital inputs, IO_MUX `FUN_IE` gating of digital reads,
matrix inputs and interrupts, driven-output input feedback, edge/level status
latching, W1TS/W1TC aliases, and the shared normal/NMI sources routed through
both cores' target interrupt matrices. The virtual target currently supplies
zero-valued strap inputs. Time-varying producers such as RMT and MCPWM are
sampled at the current guest time on every relevant `GPIO_IN` read; only a
matrix-input or GPIO-interrupt consumer asks the event queue to retain their
individual edges. Host samples, pad hold, RTC ownership, output inversion,
output enable, and open-drain release keep their normal precedence on that
lazy path. Unattached peripheral matrix outputs and BT/SDIO
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
This is useful functional GPIO support for the S3 target, not
hardware-calibrated timing or electrical evidence.
S3 RTC and digital pad-hold registers freeze the physical pad state while
register latches keep accepting writes. RTC hold covers GPIO0..21. The digital
hold register maps bits 1..27 to GPIO22..48; GPIO22..25 are unbonded, so GPIO26
is the first usable digital-held pad and GPIO21 uses RTC hold. Global RTC hold
and digital force-hold use the same pad model. Force-unhold releases only its
global source, leaving individual holds intact.

ROM boot clears global hold sources; individually held pads retain their
latched state through a deep-sleep rebuild. Pad/global isolation and autohold
policy are observable, while unconnected electrical effects and reserved bits
remain explicit. See the hardware matrix and sleep/wake gates for the current
retention evidence.

</details>

## timer groups

general-purpose timers and four-stage watchdogs have driver-backed functional timing; reset-domain and calibrated-latency limits remain.

<details>
<summary>register behavior, evidence, and limits</summary>

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

</details>

## system clocks and resets

attached consumers obey clock/reset controls. register state for an unattached domain does not imply device behavior.

<details>
<summary>register behavior, evidence, and limits</summary>

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
Target-described UART TX, I2C SDA/SCL, and GP-SPI clock/data/chip-select
producers are also valid GPIO-matrix routes across firmware builds. UART
exports complete bytes, I2C exports aggregate transactions plus released-high
idle state, and GP-SPI exports complete transactions; fast mode does not
fabricate baud or serial-clock edges.

</details>

## syscon memory policy

memory and rf policy expose semantic state; access faults, cache timing, and actual memory power loss are not implemented.

<details>
<summary>register behavior, evidence, and limits</summary>

The S3 SYSCON memory-policy owner supplies exact reset and masked readback for
the four flash and four PSRAM access-control regions, RF front-end controls,
and the 11 SRAM/3 ROM bank clock, force-down, and force-up policies. It
publishes normalized semantic state for memory-protection, cache, and power
consumers and composes with the radio owner on their shared MMIO page. Flexe
does not yet generate access faults from restrictive external-memory regions,
model cache timing, or erase backing memory when firmware requests that a bank
be powered down.

</details>

## sram and cache allocation

reset values, masks, and sticky locks are retained. broader protection faults and derived cache topology remain unsupported.

<details>
<summary>register behavior, evidence, and limits</summary>

The SENSITIVE v1 model likewise retains the cache-data-array and internal-SRAM
allocation policy with documented reset values, reserved-bit masks, and sticky
configuration locks. Its normalized state covers CPU and cache use, trace
allocation, MAC-dump/log use, and retention disable. Cache topology and access
latency are not yet derived from that policy, and protection violations are not
yet generated from the broader retained memory-protection register set.

</details>

## assist debug

live or frozen pc/sp samples support crash-record setup. watchpoints, exception records, and trace memory remain diagnostic.

<details>
<summary>register behavior, evidence, and limits</summary>

The S3 ASSIST_DEBUG recorder is also a target-described device rather than a
startup-address exception. Its per-core PDEBUG and recording controls expose
live PC/SP state while active and freeze the latest sample when recording is
stopped, which supports the standard ESP-IDF crash-record setup without a
per-instruction performance tax. Stack/area watchpoint interrupts, detailed
debug-bus fields, exception records, and trace memory are still unsupported and
continue through the diagnostic fallback.

</details>

## external i2c

native instances support fifo commands, repeated-start, nacks, interrupts, and host attachments. arbitration, stretching, and wire timing remain limited.

<details>
<summary>register behavior, evidence, and limits</summary>

The external I2C model is shared by classic ESP32 and ESP32-S3 through target
descriptors rather than fixed addresses or command encodings. It implements
both native instances, their FIFO command lists, master writes and repeated-
start reads, address NACKs, raw/enable/status/clear interrupt state, native
interrupt routing, host-attachable bus devices, and guest-slave transfers. The
S3 descriptor supplies its eight-command depth, different HAL opcodes, native
interrupt sources, peripheral identity, and independent SYSTEM clock/reset
gates. Each instance also supplies its target-specific SDA/SCL GPIO-matrix
producer IDs. Fast mode publishes released-high open-drain state while a
controller is active and disconnects those producers when its clock is gated
or reset, without fabricating individual wire edges. Tests cover both target
layouts, and an S3 application built with the
official ESP-IDF 5.3.2 toolchain and legacy production `driver/i2c.h` path
reaches `app_main`, installs the driver, and completes an absent-device
transfer with a NACK instead of timing out. This is functional fast-mode
support: SCL/SDA edge timing, timing-register effects, arbitration, clock
stretching, multi-master contention, electrical line resolution, and error
injection are not modeled yet.

</details>

## gp-spi and gdma

fifo and bidirectional gdma transfers update ownership and completion state. slave/segmented modes and wire-level timing remain outside the gate.

<details>
<summary>register behavior, evidence, and limits</summary>

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
including both CPU interrupt matrices and late enable/clear transitions. The
controller-wide GDMA clock, arbitration-disable, and AHB-master reset controls
also have their architectural reset, masking, and readback behavior.
Slave mode, segmented/config-buffer transactions, bus arbitration, signal
edges, and clock-derived transfer duration remain outside this functional envelope.
Unsupported framing and invalid DMA setup are diagnosed instead of silently
reported as successful transfers.

</details>

## sd and mmc

host-backed sdhc media and timed internal dma have a stock-driver gate. sdio-card, uhs/ddr, removal, and media-fault coverage remain.

<details>
<summary>register behavior, evidence, and limits</summary>

The native SD/MMC host is selected by a target descriptor rather than a fixed
classic address. Classic ESP32 and ESP32-S3 supply their controller base,
interrupt source, version, slot count, and GPIO-matrix clock/command/data
signals; S3 also connects the model to its SYSTEM clock and reset gate. The
DesignWare command/response, FIFO, and internal-DMA paths support host-backed
SDHC block media, descriptor ownership and write-back, chained single- and
multi-block transfers, completion/error interrupts, and both logical slots.
Timed descriptors use the common peripheral deadline scheduler, including
when the device is introduced by a non-classic target capability.
`scripts/check-s3-idf-sdmmc-host.sh` runs ESP-IDF 5.3.2's stock host driver and
ISR queue through slot 1 in interpreter and JIT, verifies 41 read and 41 write
blocks, the target's GPIO routes, identical results, and zero unsupported
accesses. Its application image SHA-256 is
`cfca7b6f64c7053f55b7d9f53bb21c4a646b2707fea707cfd84c4afb49fcb613`
and matching ELF SHA-256 is
`5ddb0c08e529a9be44b6a17ab64616e52e4f2556817bfb24d93b1f37ef94c87d`.
SDIO functions, UHS/DDR signaling, wire-level bus width and timing, hot removal,
and general media-fault injection remain outside this functional model.

</details>

## twai and can

stock drivers exercise queues, alerts, self-reception, and host frames. electrical arbitration and transceiver state are outside the model.

<details>
<summary>register behavior, evidence, and limits</summary>

TWAI is likewise one target-described SJA1000-compatible model on classic
ESP32 and ESP32-S3. Target data supplies the base, APB source clock, interrupt,
GPIO-matrix signals, classic BRP divider extension, and S3's widened 13-bit
prescaler and 9-bit clock-divider register. The pinned stock ESP-IDF 5.3.2
driver gate exercises ISR-backed queues, alerts, self-reception, and host CAN
frames in interpreter and JIT with identical results and no unsupported MMIO.
Its application image SHA-256 is
`98b418fca45ac689a16e848ca63f3429912eca0857501a6e2f7e2ce79552e348`
and matching ELF SHA-256 is
`7210164d889b792d89ab8668664f9f800ccd4ea942ae5f5bf801f1144226c718`.
Frame-duration scheduling follows configured nominal bit timing; electrical
sampling, transceiver state, and simultaneous multi-node bit arbitration are
outside the functional model.

</details>

## i2s audio

both standard master ports use timed circular gdma; host-fed rx wakes the driver with exact bytes. pdm and external clocks remain unsupported.

<details>
<summary>register behavior, evidence, and limits</summary>

The S3 I2S v2 model is also selected entirely by target geometry: the two
controller bases, SYSTEM clock/reset bits, interrupt sources, GDMA trigger
IDs, source clocks, and GPIO-matrix producers are descriptor data. Standard
master TX and host-fed RX advance one GDMA descriptor at a time on a shared
dual-core virtual timeline; finite and circular lists retain the controller's
ownership, EOF, completion, park, and level-interrupt behavior. The pinned
ESP-IDF 5.3.2 `s3_idf_i2s_std` fixture uses the stock standard-mode driver to
preload a four-descriptor ring and complete two blocking 48 kHz, stereo,
16-bit writes on port 0. It then starts port 1 as a 32 kHz stereo receiver,
blocks with all four DMA descriptors still hardware-owned until the host feeds
1,024 deterministic bytes, and wakes to return the exact capture. The gate runs
interpreter and JIT in parallel and requires identical payloads, metadata,
both ports' BCLK/WS/data matrix routes, and zero unsupported MMIO. It does not
claim wire-level clock/data edges, PDM, slave/external clocks, calibrated
underrun/overrun latency, or general audio-codec behavior.

</details>

## sha

direct/gdma hashing and clock/reset behavior are functional; unsupported digest modes and malformed chains stay diagnostic.

<details>
<summary>register behavior, evidence, and limits</summary>

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
latency, arbitration timing, and SHA-512/224, SHA-512/256,
and configurable SHA-512/t are not yet modeled.
Requests for the unsupported SHA modes or malformed DMA chains are rejected
with a diagnostic rather than returning invented digest data.

</details>

## aes

stock mbedtls exercises aes-128/256 block modes and interrupt-driven gdma. accelerator latency and nondefault endian transforms remain outside the claim.

<details>
<summary>register behavior, evidence, and limits</summary>

The AES accelerator follows the same target-described composition. Classic
ESP32 retains its shared input/output register bank and optional symbol hooks;
S3 uses separate text banks plus its native GDMA trigger and interrupt source,
without depending on an ELF symbol or firmware PC. AES-128/256 ECB, CBC, CTR,
OFB, CFB8, and CFB128 update their architectural key, IV, state, descriptor,
and interrupt state. A pinned ESP-IDF 5.3.2 fixture calls only public mbedTLS
APIs and forces a 4 KiB CBC operation through the driver's interrupt/semaphore
path; two interpreter and two JIT runs produce the same ciphertext checksum
with zero unsupported MMIO. S3 does not advertise hardware AES-192 or GCM.
Accelerator latency and nondefault `ENDIAN` transformations remain outside the
functional model.

</details>

## rtc clocks and watchdog

rtc timing and interrupt state share the core timeline. oscillator/electrical behavior and distinct watchdog reset domains remain limited.

<details>
<summary>register behavior, evidence, and limits</summary>

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
and level routing. Touch-v2 attaches its interrupts and light-sleep trigger
through device-owned APIs; physical producers such as brownout and ULP remain
unsupported until their respective device models attach. The
four-stage RTC watchdog runs from the selected slow clock and implements the
revision-profile stage-0 multiplier, feed, write protection, interrupt, and
reset actions, including pause-in-sleep behavior. Watchdog CPU/system/RTC reset
actions currently converge on a whole-machine reset; their distinct reset
domains and post-reset causes are not yet modeled. Reset-signal widths and
per-core reset-selection fields retain their architectural values but converge
on that atomic reset boundary. Reserved stage actions remain explicit
unsupported-access diagnostics when firmware changes them. Changing any other
unmodeled RTC control field remains visible in the same diagnostics, as do
reserved register bits.

</details>

## usb serial and jtag

serial fifos, packets, host i/o, and interrupts work; jtag transport, descriptors, electrical signaling, and timed sof are not claimed.

<details>
<summary>register behavior, evidence, and limits</summary>

The functional USB Serial/JTAG model implements the 64-byte serial endpoint
FIFOs, packet flush and backpressure behavior, host RX/TX, interrupt
enable/status/clear routing, connection state, and fast-mode SOF liveness.
`--usb-console` routes CLI output from this endpoint instead of UART0. It does
not yet implement JTAG transport, USB descriptors, line signaling, or timed
1 ms SOF generation; fast mode coalesces connected-host SOFs when firmware
observes the controller.

</details>

## classic peripheral aliases

recent esp-idf phy libraries use the classic esp32 ahb-lite mirror at
`0x60000000`. the full 256 kib window aliases the matching `0x3ff40000` apb
registers, including private bluetooth and wi-fi pages. rtc slow ram remains
separate at `0x50000000`.
