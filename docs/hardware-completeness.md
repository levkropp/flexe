# Functional hardware-completeness milestone

This milestone asks whether Flexe is a dependable **functional** development
and CI target for common classic ESP32 and ESP32-S3 firmware. It does not claim
cycle accuracy, calibrated latency, electrical fidelity, RF certification, or
that a host-backed service shim is a modeled silicon device. Those are separate
accuracy envelopes; `fast` mode remains the default.

## Acceptance boundary

The milestone is complete only when:

1. An evidence-linked capability matrix covers the common classic ESP32 and
   S3/LX7 paths used by the production corpus: image/ROM loading,
   flash/partitions/NVS/filesystems, caches/MMU, dual-core startup,
   interrupts, GPIO, UART/USB console, SPI/I2C, timers/PWM/RMT, watchdogs,
   DMA, and network-facing controller behavior. Each row says **modeled**,
   **partial**, or **unsupported**, and distinguishes MMIO from service shims.
2. Representative, unmodified ESP-IDF and Arduino fixtures plus production
   firmware scenarios exercise the claimed paths through useful sustained
   output or interaction. A boot banner alone is insufficient. Classic
   production and WLED realtime gates remain green; S3 needs dual-core
   FreeRTOS, sustained Arduino loop, filesystem/NVS/device basics, and at
   least one interactive production scenario before its label advances past
   experimental. S3 now meets that functional-support boundary; the matrix
   below continues to mark incomplete hardware areas as partial or unsupported.
3. Each newly modeled behavior has a focused automated test, a reproducible
   firmware gate where available, architectural or hardware evidence, and an
   explicit documented limit. Interpreter/JIT paths must agree wherever JIT
   is enabled. Unsupported transactions and MMIO must stay diagnostic; a
   successful return with no corresponding state change is a failure.
4. The repository stays lean and CI stays green. Improvements are generic
   SoC/core/device mechanisms, not firmware-name or fixed-PC workarounds.

## Current capability matrix

“Modeled” means functional behavior under the cited gates, not complete silicon
or timing equivalence. “Partial” retains explicit unsupported paths and must
not be advertised as general hardware support. A service shim is named as such
even when a firmware workflow succeeds.

| Area | Status | Evidence | Remaining boundary |
|---|---|---|---|
| Classic LX6 and production corpus | Modeled (functional) | [Compatibility scenarios](compatibility.md#curated-cyd-scenarios), `scripts/check-stock-roms.sh`, CPU/JIT differential tests | No cache/cycle timing or simultaneous cores; expand interactive coverage without losing WLED throughput. |
| Classic flash, partitions, NVS/filesystems | Partial (MMIO and service shims) | `tests/test_flash_mmu.c`, `tests/test_spi_mem.c`, [CYD scenarios](compatibility.md#curated-cyd-scenarios) | Not every flash mode, filesystem, or persistence path has a hardware-format replay gate. |
| Classic GPIO, UART, SPI/I2C, DMA | Partial (MMIO plus host devices) | `tests/test_peripherals.c`, compiled Arduino hardware gates, `scripts/check-stock-roms.sh` | Electrical and all controller/driver combinations remain outside the validated set. |
| Classic timers, PWM/RMT, watchdogs | Partial (MMIO) | `tools/ledc_pwm_test.c`, `tools/rmt_tx_test.c`, timer/watchdog unit and firmware gates | Aggregate output and event timing do not imply cycle-accurate waveforms or all reset causes. |
| Classic network/Bluetooth | Partial (service shims and device models) | Meshtastic, Marauder, NerdMiner and WLED compatibility scenarios | RF/PHY propagation and general controller equivalence are unsupported. |
| S3 LX7, interrupts, dual-core startup | Partial (interpreter and JIT) | `scripts/check-s3-idf-hello.sh`, `scripts/check-s3-idf-crosscore.sh`, CPU/JIT differential tests, and the JIT-gated WLED/Marauder scenarios | The common windowed LX7 profile, sustained queue handoffs, and CPU1 stall/resume pass; LX7-specific extensions and broader FreeRTOS/interrupt workloads remain to validate. |
| S3 ROM, image, flash/MMU/partitions | Partial | `tests/test_loader.c`, `tests/test_spi_mem.c` (4/8 MiB GigaDevice SFDP; 4 MiB BP/CMP protection), `scripts/check-s3-nerdminer-portal.sh` | Other flash protection profiles, cache behavior, and bootloader paths remain unverified. |
| S3 optional 8 MiB octal PSRAM | Partial (MSPI/MMU) | `tests/test_spi_mem.c` covers mode registers, hybrid burst and row crossing; `scripts/check-s3-psram-opi.sh` checks stock Arduino-ESP32 3.3.11 external-RAM allocation and repeated array traffic in both engines with zero unsupported accesses; default board remains unpopulated | AP Memory APS6408L-3OBMx command subset is modeled; DQS/electrical timing, refresh/PASR retention, and other PSRAM chips/board wirings are not. |
| S3 NVS, SPIFFS, reset persistence | Partial | `tests/test_loader.c`, `tests/test_spi_mem.c`, `scripts/check-s3-idf-nvs.sh`, NerdMiner POST/save/restart/reload gate, `scripts/check-s3-idf-sleep.sh` | Flash array and NOR chip state survive SoC restart; S3 deep-sleep flash power-down retains profiled nonvolatile status, while other flash profiles and partition/filesystem variants need gates. |
| S3 GPIO/RTCIO/IO_MUX | Partial (MMIO) | `tests/test_gpio.c`, `tests/test_rtc_io.c`, `tests/test_rtc_cntl.c`, `scripts/check-s3-idf-gpio-isr.sh`, stock Arduino ADC gate, native EXT0/EXT1 wake gate | Stock ESP-IDF's per-pin ISR, FreeRTOS task notification, digital open-drain release, IO_MUX input-buffer gating, driven-output input feedback, RTC GPIO wake, and pad hold work; physical pulls, drive strength, and electrical levels are not complete. |
| S3 UART/USB console | Partial (MMIO and host I/O) | Official ESP-IDF and Arduino UART output gates; `tests/test_system_clock.c` covers independent UART clock/reset and host RX gating; target-described UART TX producers route through the GPIO matrix with an idle-high pad state; the pinned Marauder gate injects a binary-safe host UART event and verifies its CLI response; `tests/test_usb_serial_jtag.c` covers USB Serial/JTAG clock/reset, packet, and SOF gating; `scripts/check-s3-idf-usb-serial-jtag.sh` replays stock ESP-IDF driver RX/TX | USB protocol/electrical behavior, baud-rate edges, and all UART DMA modes are not claimed. |
| S3 I2C, GP-SPI | Partial (MMIO) | `tests/test_peripherals.c`, `tests/test_system_clock.c`, `tests/test_spi_mem.c`; target-described I2C SDA/SCL and GP-SPI clock/data/chip-select producers route through the GPIO matrix; stock Arduino Wire, I2C-slave, and SPI-master gates replay in both engines with zero unsupported accesses; direct ESP-IDF 5.3 I2C-master and SPI-master replay gates | More I2C guest-driver/device combinations, slave overflow/clock stretching, GPIO-matrix I2C waveforms, GP-SPI wire edges, and segmented/slave modes remain. |
| S3 I2S v2 | Partial (MMIO and timed GDMA stream) | `tests/test_i2s_v2.c`; `scripts/check-s3-idf-i2s-std.sh` runs ESP-IDF 5.3.2's unmodified standard-mode driver in interpreter and JIT, streams port-0 TX and host-fed port-1 RX through four-descriptor rings, wakes blocking writers/readers through GDMA EOF interrupts, verifies 48/32 kHz stereo 16-bit configuration and both GPIO-matrix routes, and leaves zero unsupported accesses | Standard master TX/RX and both ports are driver-gated. PDM, slave/external-clock behavior, wire-level BCLK/WS/data edges, underrun/overrun latency, and calibrated audio timing remain unverified. |
| S3 SD/MMC host | Partial (MMIO, timed IDMAC, and host media) | `tests/test_peripherals.c`; `scripts/check-s3-idf-sdmmc-host.sh` runs ESP-IDF 5.3.2's unmodified host driver and ISR queue in both engines, exercises slot-1 command/response plus single- and 20 KiB multi-block reads/writes, verifies GPIO-matrix routes and media, and leaves zero unsupported accesses | SDHC block media and both logical slots are modeled. SDIO cards, UHS/DDR signaling, bus-width electrical behavior, calibrated clock timing, card removal, and media-error injection remain unverified. |
| S3 TWAI/CAN | Partial (MMIO, timed frames, and host bus) | `tests/test_twai.c`; `scripts/check-s3-idf-twai.sh` runs ESP-IDF 5.3.2's unmodified driver in both engines through GPIO routing, ISR-backed TX/RX queues, alerts, self-reception, and host-injected standard/extended frames with zero unsupported accesses | CAN 2.0 frame/FIFO/filter, arbitration-loss, retry, error confinement, bus-off, and recovery state are modeled. Electrical bit arbitration, transceiver behavior, multi-node wire timing, and calibrated error injection remain unverified. |
| S3 GDMA | Partial (MMIO) | `tests/test_crypto.c` checks controller-wide clock/arbitration/AHB-reset configuration, finite chained TX/RX descriptors, ownership/writeback, errors, and per-channel level interrupts on both cores; `tests/test_i2s_v2.c` checks descriptor-at-a-time circular streams; GP-SPI and stock I2S gates exercise independent consumers plus both I2S trigger IDs and directions | Full priority/arbitration behavior and other streaming consumers such as LCD/camera/continuous ADC remain unverified. |
| S3 AES | Modeled (functional MMIO/GDMA) | `tests/test_crypto.c`; `scripts/check-s3-idf-aes.sh` runs ESP-IDF 5.3.2's unmodified mbedTLS driver twice in each engine through AES-128/256 ECB, CBC, CTR, OFB, CFB8, and CFB128 plus a 4 KiB interrupt-driven transfer | Operations complete immediately. Nondefault `ENDIAN` transformations are retained but not applied; AES-192 and accelerator GCM are not S3 hardware capabilities. |
| S3 SYSTEM/SYSCON clock and power policy | Partial (MMIO) | `tests/test_system_clock.c`, `tests/test_syscon_memory.c`, WLED/Marauder production audits | Both complete peripheral clock/reset banks, four flash/four PSRAM access-control regions, and the 11-SRAM/3-ROM plus RF front-end memory policies have exact reset/readback state and semantic observers; effects are attached for selected modeled devices. Cache timing, access-fault generation, unattached-device effects, and actual bank power loss remain unsupported. |
| S3 cache/SRAM allocation and memory protection | Partial (MMIO) | `tests/test_sensitive_memprot.c`, `tests/test_syscon_memory.c`, `tests/test_esp32s3_extmem.c`, WLED and zero-unsupported stock Arduino production audits | Cache-array/internal-SRAM ownership has exact reset, masking, and sticky locks; external flash/PSRAM access regions have exact reset, masking, and semantic state. Cache topology/timing, the external-region lock, and protection-fault generation remain unsupported. |
| S3 CPU assist/debug recorder | Partial (MMIO) | `tests/test_assist_debug.c`, zero-site WLED production audit | Both cores implement target-described PDEBUG enable, live/frozen PC and SP recording, and silicon DATE behavior without an instruction-loop hook. Detailed debug-bus payload, stack/area watchpoints, exception records, and trace memory remain diagnostic. |
| S3 timers, watchdogs, RTC | Partial (MMIO) | `tests/test_systimer.c`, `tests/test_timer_group.c`, `tests/test_rtc_cntl.c` (masked RTC configuration, supply, analog-control and power/isolation-domain resolution, fault-injection selection, CPU-follow, sleep/wake, and stall enable), ESP-IDF cross-core, restart, zero-unsupported native timer and GPIO light/deep-sleep gates | Timer and EXT0/EXT1 wake work; OPTIONS0, ANA_CONF, SLP_REJECT_CONF, SDIO/BIAS/drive configuration, FIB reset-source selection, digital-pad/global isolation, digital domains, and RTC-local domains have architectural state, with selected consumers connected. Brownout voltage detection/reset, touch/ULP wake, analog transition timing, and other reset causes remain unsupported. |
| S3 LEDC PWM | Partial (MMIO) | `tests/test_ledc_v1.c`, `scripts/check-s3-ledc.sh`: stock Arduino repeatedly drives GPIO4 at 5 kHz with four readback duties in both engines, byte-identical replay, and zero unsupported accesses | Aggregate PWM output and timed fade/interrupt are modeled; individual electrical edges and overflow-counter behavior are not. |
| S3 PCNT pulse counter | Partial (MMIO and timed GPIO-matrix input) | `tests/test_pcnt.c`; `scripts/check-s3-idf-pcnt.sh` runs ESP-IDF 5.3.2's unmodified driver in interpreter and JIT through GPIO loopback, APB glitch filtering, edge/control actions, high/low limits, four ISR watch points, and teardown with zero unsupported accesses | Four S3 units with two channels, event/interrupt state, SYSTEM clock/reset, and matrix inversion are modeled. Silicon synchronizer/metastability behavior, calibrated edge limits, software overflow accumulation, and broader quadrature workloads remain unverified. |
| S3 RMT TX | Partial (MMIO and GPIO-matrix output) | `tests/test_rmt_v1.c`, `scripts/check-s3-wled-rmt.sh`, `scripts/check-s3-idf-rmt-loopback.sh`; WLED 16.0.1 emits 317 sustained pulse frames, and stock ESP-IDF drivers exercise plain, carrier, finite, infinite, and synchronized two-channel output | Pad edges are scheduled only for a watched matrix input or GPIO interrupt; `GPIO_IN` polls on demand. End-marker finite loops work with auto-stop and batching beyond 1023; end-marker infinite loops run until `TX_STOP`; selected synchronous channels share the final `TX_START` timestamp. Markerless loops, counted loops without auto-stop, always-on carrier, dynamic sync-group changes, silicon-calibrated carrier phase, and fine status remain unsupported. |
| S3 RMT RX | Partial (MMIO, filtered/demodulated GPIO input, host symbols) | `tests/test_rmt_v1.c`, `scripts/check-s3-rmt-rx.sh`, `scripts/check-s3-idf-rmt-loopback.sh`; stock Arduino-ESP32 3.3.11 filters a GPIO4 glitch, demodulates a carrier waveform, and receives a 96-symbol host frame in both engines with zero unsupported startup/device accesses; stock ESP-IDF 5.3.2 receives both plain and modulated TX pad pulses through its ISR callback with zero unsupported accesses | Host samples, software GPIO feedback, and RMT TX loopback share the GPIO-matrix edge path; pulse RAM becomes inaccessible and loses contents under `RMT_MEM_FORCE_PD`. DMA, odd pulse tails, delayed-ISR overrun, and dynamic mid-segment route changes remain unsupported. |
| S3 SENS clocks, RTC SAR ADC and temperature sensor | Partial (MMIO plus host samples) | `tests/test_sens.c`, `tests/test_apb_saradc.c`, `scripts/check-s3-adc.sh` (deterministic interpreter/JIT replay with zero unsupported accesses), WLED production audit | The complete IO-mux/SARADC/temperature/RTC-I2C clock and SARADC/temperature/RTC-I2C/coprocessor reset fabric has exact state; ADC and temperature effects are connected. Digital/DMA conversion, ULP execution, unattached clock/reset effects, contention, and physical calibration remain unsupported. |
| S3 network-facing workflow | Partial (service shim) | NerdMiner BSD-socket portal, WLED native lwIP/Ethernet UI and JSON state, and `scripts/check-s3-idf-socket-range.sh` with a stock 10-socket ESP-IDF build | Wi-Fi RF/PHY, association realism, and general transport modes are unsupported; the socket bridge requires ELF symbols and a VFS range within its 64-FD `select()` layout. |
| S3 Bluetooth controller bootstrap | Partial (MMIO) | `tests/test_radio.c` checks modem clocks, selective reset and RTC power/isolation domains, baseband time, immutable controller identity, and command consumption; `scripts/check-s3-marauder.sh` boots the official v1.16.0 MultiBoard S3 image through native Bluetooth and Wi-Fi setup, injects `help` through UART0, and verifies its response, next prompt, and zero unsupported accesses | General controller scheduling, Bluetooth packets, coexistence fidelity, and RF remain unsupported. |
| Cycle/cache/electrical/RF fidelity | Unsupported | Outside this functional milestone | Requires calibrated hardware traces and declared tolerances. |

The AES engine is selected from target data rather than firmware identity:
classic ESP32 and S3 provide their native register layout, supported key-size
codes, base address, and optional GDMA/interrupt wiring. On S3, the model
consumes the live trigger-6 transmit chain and completes its receive chain,
updates CBC/OFB/CTR/CFB chaining state, publishes `IDLE/BUSY/DONE`, and routes
the level interrupt through source 77. Its SYSTEM clock and reset bits suppress
work and restore register state just like the other attached devices. The
pinned stock-driver image covers direct known-answer vectors, partial-length
CTR handling, every S3 hardware block mode, and the driver's interrupt/semaphore
path with identical interpreter/JIT output and zero unsupported MMIO. Its image
SHA-256 is
`27d35ac42f30490e43c81071dfe38bc184321e5b03421d5bf277814c817d03f1`
and matching ELF SHA-256 is
`ef6c993258461e85bbd816da17ea7afe2ea25554fd78a7d648ae24f344276388`.
Rebuild and replay it with:

```sh
S3_ROM_ELF=/path/to/esp32s3_rev0_rom.elf \
  ./scripts/build-s3-idf-fixture.sh --check aes
```

S3 UART0/1/2 now obey their independent
[SYSTEM peripheral clock and reset bits](https://github.com/espressif/esp-idf/blob/v5.5.1/components/soc/esp32s3/register/soc/system_reg.h):
an unclocked port cannot emit TX bytes, accept host RX, or assert its
interrupt; a reset edge clears its FIFO, configuration, and interrupt state
without erasing already-observed host output. This is a functional gate, not a
model of APB bus stalls, serial line timing, or the separate USB controller.
The `--sandbox-events` transport accepts strictly parsed NDJSON host events.
`uart_in` carries either one byte or a bounded hexadecimal byte string to any
target-described UART, and `uart_break` drives the controller's receive-break
path. These events enter the same FIFO, timeout, interrupt, and UHCI paths as
other host UART injection. `i2s_in` carries bounded hexadecimal sample bytes to
either I2S port and enters the same RX FIFO and DMA path as an attached audio
source. If a connected frontend is the only possible
wake source while all guest cores are in `WAITI`, Flexe freezes guest time and
waits in short host-time intervals instead of busy-spinning or exhausting the
cycle budget before the next input arrives.
Espressif's [S3 USB Serial/JTAG low-level driver](https://github.com/espressif/esp-idf/blob/v5.5.1/components/hal/esp32s3/include/hal/usb_serial_jtag_ll.h)
uses SYSTEM's `USB_DEVICE` clock/reset bits for that controller. Flexe now
blocks host RX, packet completion, synthetic SOF, and interrupt output while
its clock is off; a reset edge drops unsent/unread packets and restores
configuration while preserving cable connection and already-captured output.
The [S3 RTC USB PHY mux](https://github.com/espressif/esp-idf/blob/v5.3.2/components/hal/esp32s3/include/hal/usb_serial_jtag_ll.h)
retains its software override/selection bits and gates the virtual host when
the internal PHY is routed to USB OTG or `CONF0.PHY_SEL` chooses the external
PHY. The default virtual board assumes the internal-PHY eFuse route;
external PHY hardware, enumeration, and line signaling remain outside this
functional model.
The ESP-IDF v5.3.2 `tests/fixtures/s3_idf_usb_serial_jtag` application
installs the [stock interrupt-driven USB Serial/JTAG driver](https://github.com/espressif/esp-idf/blob/v5.3.2/components/esp_driver_usb_serial_jtag/include/driver/usb_serial_jtag.h), writes a greeting,
then receives two host-injected `ping` packets and replies `USJ_PONG_1/2`
through the driver's TX ring buffer. The gate runs the complete interaction
twice with byte-identical UART, USB, and unsupported-MMIO digests. The entire
driver replay now uses zero unsupported MMIO accesses, including the USB
controller aperture and RTC USB PHY mux. The pinned
application image SHA-256 is
`6afea3c1ad908202bdbaa4c3ae19b72f850aa96b38d3a1ff55b2796c1bff14d5`
and matching ELF SHA-256 is
`a47a4f5468108aebaaba6d3ab9aab74817dd52eded39848e1d81dc62cf8e6cbc`.
This checks driver-level packet and interrupt progress, not USB enumeration,
electrical timing, or large/bursty packet stress. Rebuild with ESP-IDF commit
`9d7f2d69f50d1288526d4f1027108e314e8c879f` and run:

```sh
S3_ROM_ELF=/path/to/esp32s3_rev0_rom.elf \
  ./scripts/build-s3-idf-fixture.sh --check usb-serial-jtag
```

The fixture helper derives a stable `SOURCE_DATE_EPOCH` and normalizes its
external build path, so a clean pinned build reproduces these hashes. An
intentional toolchain, source, flag, or timestamp change may provide matching
`*_SHA256` overrides to its gate.

S3 now meets the supported functional-target boundary. The NerdMiner
filesystem result is a meaningful end-to-end flash-format/mount check. With
the matching application ELF, the host-backed socket/select boundary lets the
unmodified firmware serve `GET /wifi` as `200 OK` with its 4,985-byte
configuration HTML. This is a service shim, not a modeled Wi-Fi radio: the
page reports no networks. An earlier full POST/restart/reload replay reported
about 35,000 unsupported accesses; the fixed 4-billion-cycle interpreter
audit and current full replay now require zero unsupported MMIO sites after
the target-described RF/PHY, clock/power, serial-routing, and GDMA control work
below. This does not turn the network service into a radio model. The S3
application handoff now clears
the power-on flash-boot watchdog mode skipped with the second-stage bootloader.
Restoring ROM-owned
BSS and interface state from the official ROM ELF during a software restart
lets NerdMiner initialize again and reload its saved settings instead of
panicking during PSRAM setup, while physical SRAM outside those sections
remains intact for app `.noinit`.
The scripted POST/restart/reload gate exercises that full path. WLED now
provides a second, independently sourced production interactive network and
output scenario; Marauder supplies an independent interactive UART/controller
scenario. The remaining RF/PHY gaps prohibit claims of radio or silicon
equivalence, but do not invalidate the documented functional support tier.
ROM images and third-party firmware binaries are not copied into this
repository.

The official ESP-IDF v5.3.2 `examples/get-started/hello_world` image built
from commit `9d7f2d69f50d1288526d4f1027108e314e8c879f` (application image
SHA-256 `aff0d18eac38ebeb05181d58fbe9c13e7897fb83680d924a0a48bc0abe7c4221`,
ELF `1ef4206eaabfc6b3b148b69bcb9244a2d756f983cefe14addbdfa190496384f3`)
now reaches `app_main()`, counts down through ten guest seconds, requests its
own software reset, and does so again after both cores restart. The separate
`tests/fixtures/s3_idf_crosscore` project, built with the same IDF, pins a
producer to CPU1 and checks 32 queue-message/notification round trips with
`app_main()` on CPU0. It then uses the official `esp_cpu_stall(1)` and
`esp_cpu_unstall(1)` calls to verify that an active CPU1 task stops making
progress and resumes without losing its state, before sustained 100 ms
heartbeats. Its image SHA-256 is
`184e23dfb61fcdd2bb3fc999efddd8a3f863c08a523789bb69515f05e4eafcbe`
and ELF SHA-256 is
`fb748fd11ef199d5cb268d0401e4dd8577196fb22282fd409af011e7bec5f8b3`.
Both external-image gates compare a complete second replay byte-for-byte and
require zero unsupported MMIO accesses. RTC `OPTIONS0` software-reset pulses
reset APP CPU architectural state independently at a safe execution boundary;
PRO CPU and system reset requests retain the full-machine reset path. This
proves these FreeRTOS interactions, not simultaneous core execution or timing
fidelity. Rebuild and run both gates with the pinned ESP-IDF checkout:

```sh
S3_ROM_ELF=/path/to/esp32s3_rev0_rom.elf \
  ./scripts/build-s3-idf-fixture.sh --check hello crosscore
```

The gates default to these pinned image/ELF hashes and the official revision-0
ROM ELF SHA-256 `c0ce0f338d1de1bdc6efbef1591779a2a42c1ab7d759d3c6ae8ae63a7dd34cfd`.
The fixture helper auto-discovers and initializes the pinned toolchain and ROM,
keeps persistent generated configuration outside the repository, and shares
safely cached ESP-IDF components across projects.

Clean builds through the fixture helper reproduce these hashes by fixing the
embedded timestamp and normalizing the external build path. Intentional input
changes can supply their matching `*_SHA256` overrides.

The S3 RTC controller now owns its documented 48-bit sleep alarm, wake enable,
sleep state, timer interrupt, target-described digital power/isolation fields,
and separate wake-cause register. The session advances the shared virtual
clock through timer-only light sleep or resets after timer-only deep sleep;
the always-on counter, fractional tick phase, and STORE0-7 survive that
rebuild. The
[S3 RTC TIMER1..TIMER6 registers](https://github.com/espressif/esp-idf/blob/v5.5.1/components/soc/esp32s3/register/soc/rtc_cntl_reg.h)
now retain their specified power-on and writable fields. TIMER1's CPU_STALL_EN
bit gates the real two-register CPU stall key; reserved writes remain
diagnostic. Fast mode still does not delay execution by programmed analog
power-settle intervals. In the pinned stock Arduino PSRAM boot, these registers
accounted for 34 of 123 formerly unsupported accesses; subsequent UART clock
gating accounts for one more, leaving 88. The
unmodified ESP-IDF 5.3.2 `tests/fixtures/s3_idf_sleep` application enters a
50 ms light sleep, observes `ESP_SLEEP_WAKEUP_TIMER`, then enters a 20 ms deep
sleep and checks the timer wake cause, deep-sleep reset reason, RTC_DATA marker,
and STORE0 marker on its second `app_main()`. The guest also measures RTC
ticks through each sleep, including the second boot's startup, and sustains a
FreeRTOS heartbeat afterward. Its image SHA-256 is
`bca98a4f1806266286ca5090a489e668d3e80d42b21b1cd8ecc6154259485246`
and matching ELF SHA-256 is
`b58b49d5b4012c4ac80445c070a602115b413403a77437914394484c45722be9`.
The two complete replays are byte-identical and the full reports contain zero
unsupported MMIO accesses. No unsupported sites remain in the validated S3
timer-sleep boot, sleep, reset, and second-boot path. The nominal
slow-clock model yielded about 48.9/19.3 ms for the guest's requested 50/20 ms,
so this is functional wake ordering, not calibrated sleep duration. Touch/ULP
wake and analog voltage-transition timing are not modeled; an unarmed timer or
those sources do not synthesize a wake.
The target-described RTC configuration bank gives SLP_REJECT_CONF, SDIO_CONF,
BIAS_CONF, REGULATOR_DRV_CTRL, and TOUCH_CTRL2 exact reset, masking, and
readback behavior without teaching the device model S3 addresses. Bias and
drive values are retained while their electrical voltage/settling effects are
deliberately collapsed in functional mode. SDIO's readiness field remains
read-only, reserved writes remain diagnostic, and activating the touch FSM
still reports an unsupported effect until a touch engine consumes it. Valid
oscillator gating fields in CLK_CONF likewise retain their exact value while
the modeled slow/fast muxes resolve immediately.
The separate RTC fast-clock mux now selects the target-described 20 MHz XTAL/2
or nominal 17.5 MHz RC_FAST source, while the slow counter keeps its own mux and
phase. `RTC_CNTL_DATE_REG` resets to the S3 revision value and retains its
documented 28-bit payload, including the overlapping six-bit LDO-slave trim.
Functional mode exposes that software-visible state but does not turn trim
values into simulated supply voltages or oscillator settling delays.
`RTC_CNTL_REG` likewise resolves the target-described RTC-regulator and
digital-boost force pairs, with force-down winning a conflicting pair and no
force meaning powered in functional mode. SCK_DCAP and DIG_CAL fields retain
their documented values but remain diagnostic because oscillator capacitance,
voltage ramps, and analog calibration timing are not simulated. Supply
consumers can subscribe to logical transitions without touching the RTC event
scheduler.
`RTC_CNTL_OPTIONS0_REG` now describes all retained fields independently of the
CPU-stall and software-reset commands: crystal/BBPLL/internal-I2C supply pairs,
analog/PLL/crystal isolation pairs, digital-wrapper reset, and crystal-enable
wait. `RTC_CNTL_ANA_CONF_REG` likewise exposes its analog enables and reset-POR
supply pair. The target includes the revision-0 ROM-private bit 29 because the
immutable `rom_open_i2c_xpd` routine explicitly sets it; this is silicon/ROM
evidence rather than a firmware-PC exception. A normalized listener reports
powered, isolated, reset, enabled, and wait state without requiring consumers
to decode S3 register positions. Force-down and force-isolation/reset win
conflicting pairs. Unattached RF/PLL controls still do not imply voltage,
calibration, or settling-time fidelity.
The S3 `RTC_CNTL_BROWN_OUT_REG` now retains documented configuration fields,
resets to the specified defaults, and treats counter-clear as a write-only
strobe. `RTC_CNTL_FIB_SEL_REG` likewise retains the three software/fault-
injection reset-source selectors used when the brownout HAL takes ownership.
The detector remains clear under Flexe's fixed nominal supply; analog
thresholds, voltage injection, brownout interrupts, and resets are not modeled.
`RTC_CNTL_PWC_REG` resolves target-described RTC-peripheral, slow-memory, and
fast-memory power/isolation domains in addition to its global pad-force-hold.
Force-down and force-isolation win conflicts, RTC-peripheral sleep PD applies
only during sleep, and each RTC-memory follow bit uses the descriptor's CPU-top
domain rather than a chip-specific index. Logical transitions are exposed to
future memory/peripheral consumers; this does not yet erase or hide backing
memory when a domain loses power. The global hold freezes all 22 RTC-capable
pads through the same physical hold model as individual bits. ROM boot clears
the global source, while individually held pads retain their state across a
deep-sleep rebuild. Releasing global hold does not release an individual pad.
The digital hold register maps bits 1–27 to GPIO22–48; GPIO22–25 are unbonded,
so GPIO26 is its first usable pad, while GPIO21 uses the RTC hold register.
`RTC_CNTL_DIG_ISO_REG`'s global digital force-hold freezes bonded GPIO26–48;
its force-unhold bit releases that source without clearing individual holds.
ROM boot clears global digital hold, but individually held pads keep their
latched state. Pad/global isolation and autohold-enable state, plus the
autohold-clear command, are target-described and observable; their remaining
unconnected electrical effects are explicit limitations rather than raw-MMIO
fallbacks. Reserved bits remain diagnostic.

`RTC_CNTL_DIG_PWC_REG` and `RTC_CNTL_DIG_ISO_REG` resolve six
target-described domains, including automatic sleep power-down and force-up,
force-down, force-no-isolation, and force-isolation pairs. Force-down and
force-isolation win conflicting pairs. Consumers receive logical domain state
rather than chip-specific register bits: S3 Wi-Fi and Bluetooth apertures are
independently connected today. Isolation hides an aperture while retaining its
register state; loss of power restores its reset image. The hardware RNG stays
accessible as its own target-described endpoint even though its address lies
inside the WDEV range. Other domain consumers and analog transition delays are
not yet connected, and reserved fields remain diagnostic.

Rebuild and run with the pinned ESP-IDF checkout:

```sh
S3_ROM_ELF=/path/to/esp32s3_rev0_rom.elf \
  ./scripts/build-s3-idf-fixture.sh --check sleep
```

The S3 RTC controller also models the documented EXT0/EXT1 wake configuration
(`rtc_cntl_reg.h`, `rtc_io_reg.h`): RTCIO selects EXT0's RTC-owned input pad;
EXT1 selects up to 22 RTC pads and triggers on any high or all low, latching
the matching status bits. While an external wake is armed, neither guest core
retires instructions. The host may change GPIO input through the existing
`gpio_in` sandbox event, and the session samples that level in bounded,
approximately real-time idle slices. An armed timer may provide a fallback;
invalid pin selection, an unarmed timer, and unknown sources stay diagnostic.
The unmodified ESP-IDF 5.3.2 `tests/fixtures/s3_idf_gpio_wake` application
enters EXT0 light sleep on GPIO4, then EXT1 deep sleep on GPIO11/12. The gate
raises GPIO4 and GPIO12 only after each sleep request; the guest verifies both
wake causes, EXT1's GPIO12 status across the deep-sleep reset, and sustained
FreeRTOS execution afterward. Two replays match on guest wake outcomes and
unsupported MMIO sites; the exact wake duration varies with host input timing.
Its image SHA-256 is
`3fd036c2e52c1ee50b826c1fb3f63d18e8d09c2d01f93bc455518d71ec8fe575`
and ELF SHA-256 is
`a007e5da15a99804a4ee8d228be7cf5912480a2bb9d9594511e7ede0348048a0`.
Both complete reports contain zero unsupported MMIO accesses; the EXT
selection/state/status, digital-domain, and shared RTC configuration sites are
all modeled.
Physical pull resistors, voltage thresholds, glitches/filtering, and pad
electrical behavior are not inferred from the host's digital level:

```sh
S3_ROM_ELF=/path/to/esp32s3_rev0_rom.elf \
  ./scripts/build-s3-idf-fixture.sh --check gpio-wake
```

The separate `tests/fixtures/s3_idf_gpio_isr` project uses the [stock
ESP-IDF GPIO ISR service](https://docs.espressif.com/projects/esp-idf/en/release-v5.3/esp32s3/api-reference/peripherals/gpio.html)
to register a rising-edge handler on GPIO4. Host-driven edges reach the
firmware's per-pin ISR, which notifies `app_main` through FreeRTOS; stable-high
and falling input cause no extra callback. The same firmware configures GPIO5
as open-drain and alternates released-high and driven-low states. The GPIO
model's output listener and output-enable query expose the effective drive
state rather than treating the high latch as a driven voltage. The older
sandbox GPIO event still carries only the output-latch level. Two interpreter
replays have identical UART and unsupported-MMIO digests, with zero
unsupported MMIO accesses. The pinned image SHA-256 is
`af96acd3c647184f2b92e6d07df25a20ed28e9a28bcb4084a59479d7c02c9186`
and matching ELF SHA-256 is
`d3bea1061d45252a05cc973720e4f0ab77ca4ea5945ac854bb6961de59ef66f1`.
Digital `GPIO_IN` reads, GPIO-matrix inputs, and edge/level interrupt sampling
now honor each pad's IO_MUX `FUN_IE` bit. An enabled input with no host sample
reads back a known driven output; a released open-drain pad has no modeled
pull voltage and defaults low. Host samples override output feedback. RTCIO
uses its separate input path. This follows Espressif's
[GPIO input-mode contract](https://docs.espressif.com/projects/esp-idf/en/v5.3.2/esp32s3/api-reference/peripherals/gpio.html)
and [S3 IO_MUX register definition](https://github.com/espressif/esp-idf/blob/v5.3.2/components/soc/esp32s3/include/soc/io_mux_reg.h).
The image and ELF hashes matched across two clean build directories with the
same pinned ESP-IDF toolchain.
This checks digital edge/interrupt and high-impedance output behavior, not
physical pull resistors, bus contention, voltage thresholds, or pin timing.
Rebuild with ESP-IDF commit
`9d7f2d69f50d1288526d4f1027108e314e8c879f`:

```sh
S3_ROM_ELF=/path/to/esp32s3_rev0_rom.elf \
  ./scripts/build-s3-idf-fixture.sh --check gpio-isr
```

Independent builds may supply matching `*_SHA256` overrides for the
external artifacts.

The separate `tests/fixtures/s3_idf_nvs` project uses ESP-IDF v5.3.2's real
`nvs_flash` implementation to open a namespace, write and commit a 32-bit
value, request a software reset, and read the value after the second boot. It
also verifies the software CPU reset reason and an RTC STORE0 marker retained
across that reset.
Neither its API nor its SPI-flash transactions are replaced by host NVS shims.
Its direct application image uses Flexe's documented synthesized NVS
partition; NerdMiner's factory-image gate separately exercises a real
partition table and SPIFFS volume. The pinned NVS image SHA-256 is
`86d87f33b9b8730f9d60e15456a23ee713e7dbcfd742ceac5502724a000a8777`
and ELF SHA-256 is
`f83e27a89adf22ad65d28193a7d12fd31ef4e4efc47292652e040681f5d4ae18`.
The gate checks two byte-identical runs and requires zero unsupported MMIO
accesses across its two boots:

```sh
S3_ROM_ELF=/path/to/esp32s3_rev0_rom.elf \
  ./scripts/build-s3-idf-fixture.sh --check nvs
```

The shared Arduino Wire fixture also runs unmodified on S3 with valid S3
GPIOs. Its 40-byte write exceeds the 32-byte controller FIFO, then a
repeated-START read returns the same bytes through the driver's interrupt
path; an unattached address returns a NACK. The virtual slave callback is a
host-side I2C device, not a substitute for guest Wire or the MMIO controller.
The stock Arduino-ESP32 3.3.11 image/ELF hashes are
`91440bdbb4f0a057bf95fab104bc27602555f254d904d7a97b3c032fcb3cb49e` /
`271630096105d3afef3946cf0d91f5413f50ac8ce99be39b05cd39e235f45121`.
The harness then requests a software reset and lets the same guest image run
the transfer again. The external slave registration and its host-held register
contents persist, while the SoC I2C controller is rebuilt. A separate unit
test covers all three classic I2C bus attachments across reset. Two complete
runs of each engine match byte-for-byte and both execute the two guest boots
with zero unsupported accesses. This gate does not test a guest-initiated
restart, electrical timing, bus contention, or other devices:

```sh
./scripts/build-s3-arduino-fixture.sh --check i2c-wire
```

The separate `tests/fixtures/s3_idf_i2c_master` project calls ESP-IDF
v5.3.2's [new I2C master driver](https://docs.espressif.com/projects/esp-idf/en/v5.3.2/esp32s3/api-reference/peripherals/i2c.html)
directly, without Arduino or guest-driver hooks. I2C0 sends a 41-byte
register write to a host-attached device, then performs a repeated-START
40-byte read through the controller's interrupt/FIFO path. It verifies every
returned byte and an unattached-address NACK (`ESP_ERR_NOT_FOUND`). Two
interpreter replays have identical UART and unsupported-MMIO digests, with
zero unsupported accesses. Target-described I2C0 SDA/SCL GPIO-matrix
producers 90/89 expose released-high open-drain pad state between aggregate
transactions; both controller instances and the classic ESP32 use their own
descriptor signal IDs. Flexe does not synthesize I2C wire edges or claim
electrical bus timing. The pinned image SHA-256 is
`10934e17ec7ca440c4d81689225373b7fc1809b898700a5a8b813ab9a02c1ccc`
and the matching ELF SHA-256 is
`5ef3ea1fb67a20e5748d98124c791197a9404cac0b6dd953a28b441e3d3fd734`;
both matched across two clean build directories using ESP-IDF commit
`9d7f2d69f50d1288526d4f1027108e314e8c879f`:

```sh
S3_ROM_ELF=/path/to/esp32s3_rev0_rom.elf \
  ./scripts/build-s3-idf-fixture.sh --check i2c-master
```

Matching independently built artifacts can supply `*_SHA256` overrides.

A separate stock Arduino-ESP32 3.3.11 S3 image configures ESP-IDF's I2C0
slave driver at address `0x42`. A host master writes `DE AD BE EF`; the guest
reads those bytes and returns its staged `11 22 33`. The fixture uses GPIO4
for SCL because S3 has no GPIO22, while GPIO26-32 are normally
reserved for flash/PSRAM ([Espressif GPIO guide](https://docs.espressif.com/projects/esp-idf/en/release-v5.2/esp32s3/api-reference/peripherals/gpio.html)).
The merged image and ELF hashes are
`bdfd7cdf30381de4f3c9e26a588c114ddd8721638c397bdd84cd6f9a018ed290`
and `5477d38fc97a858da0ea0e11cd1bd7a75c0f1a30bd28595dd37f80496c29fcfa`.
Two complete runs of each engine replay byte-for-byte with zero unsupported
accesses. Clock stretching, overflow, electrical bus timing, and other
slave-driver implementations remain unverified:

```sh
./scripts/build-s3-arduino-fixture.sh --check i2c-slave
```

The S3 GP-SPI gate compiles the same ESP-IDF `spi_master` fixture used for
classic ESP32 against Arduino-ESP32 3.3.11, selecting SPI3 and S3-valid pins.
The driver completes five full-duplex lengths (1, 4, 5, 17, and 33 bytes),
a command/address read, and one queued transaction through the MMIO/GDMA
engine and a host-attached slave. In the command/address read, ESP-IDF retains
SPI `DMA_TX_ENA` from the prior transfer but starts no TX GDMA link. Flexe
uses the live GDMA route to distinguish this valid receive-only case from a
broken TX descriptor; the unit test keeps the latter diagnostic. The gate's
merged image/ELF SHA-256 values are
`f4a50104a5cd08c91d563eb30cc38ad86bb9df72f36a5420a43cb5e62ca01940` /
`ea7494ab15e49a75da094b40b5cad7b4cf25e186832e89de4800f562e2b9dba4`.
The harness then resets the guest machine and repeats all seven transfers
through the same host-side probe endpoint. Its registration survives, while
the GP-SPI and GDMA register files start fresh; the session reset unit test
also covers per-host probe, device, and select callbacks. Two runs of each
engine match byte-for-byte and execute both guest boots with zero unsupported
accesses. The reset is harness-requested, not a guest `esp_restart()`. This
does not validate SPI slave mode, segmented transfer, exact bus timing, or
physical pin levels:

```sh
./scripts/build-s3-arduino-fixture.sh --check spi-master
```

The target-described PCNT model covers the classic ESP32 and S3 variants
without firmware-name or fixed-PC behavior. Target data supplies each
controller's base, register geometry, unit count, GPIO-matrix inputs,
interrupt source, source clock, reset image, and SYSTEM gate. The shared
engine applies both channels' edge and control actions, qualifies pulse and
control transitions with the APB glitch filter, enforces signed high/low
limits, latches threshold/zero/limit events, and drives the target interrupt
matrix. GPIO-matrix route changes rebind the selected source without turning
configuration into a phantom pulse. Byte-enable-aware MMIO preserves the
stock driver's byte and halfword bitfield stores without performing unsafe
read/modify/write cycles on W1C or strobe registers.

The native `tests/fixtures/s3_idf_pcnt` project uses ESP-IDF 5.3.2's
unmodified `pulse_cnt` driver. GPIO4 loops back to the edge input, GPIO5
selects direction, a 1 us filter qualifies transitions, and watch points at
2, 4, -2, and 0 wake the driver's ISR callback in that order before complete
driver teardown. Two runs of both interpreter and JIT are byte-identical,
the JIT retires native instructions, and all four runs have zero unsupported
MMIO. Its pinned application image SHA-256 is
`7f82157987f475be33b2d44b1df9bbbe0d78af5e20d88ee7c7d7ebc3f9731ac4`;
the matching ELF SHA-256 is
`4e1296eb8a8a9ccc6821fff20f913d84fb15a334e01cebc3c1c2831356cdc32c`.
Rebuild and replay it with:

```sh
S3_ROM_ELF=/path/to/esp32s3_rev0_rom.elf \
  ./scripts/build-s3-idf-fixture.sh --check pcnt
```

The S3 RMT V1 model handles direct pulse RAM, per-channel dividers,
threshold refill interrupts, end/error interrupts, pulse-timed TX and
non-DMA RX from GPIO-matrix edges or a host symbol-injection API. RX advances
the hardware writer offset, signals threshold interrupts, wraps the physical
RAM in ping-pong mode, and completes after the configured idle duration. The
stock Arduino-ESP32 3.3.11 RX driver copies a 2-symbol frame measured from
host-driven GPIO4 transitions and a 96-symbol host-decoded frame (four half-RAM
transfers) through its unmodified ESP-IDF interrupt handler. The S3 GPIO
matrix routes RMT RX0 through signal 81
([Espressif signal map](https://github.com/espressif/esp-idf/blob/v5.3.2/components/soc/esp32s3/include/soc/gpio_sig_map.h));
the model measures edge intervals on the guest clock. The input glitch
filter qualifies edges after the configured number of RMT group-clock ticks,
before the per-channel divider, matching ESP-IDF 5.5's S3 filter clock choice
([Espressif RX driver](https://github.com/espressif/esp-idf/blob/v5.5.1/components/esp_driver_rmt/src/rmt_rx.c)).
GPIO dispatches watched matrix-signal edges from host pad samples, driven
software GPIO outputs, and plain or data-only-carrier RMT TX signals. Unit
gates drive a software output and TX0 through GPIO4 into RX0, verifying
pulse widths in
guest time; the TX gate also samples `GPIO_IN` at half-symbol boundaries.
Espressif documents this [TX/RX GPIO loopback mode](https://docs.espressif.com/projects/esp-idf/en/v5.3.2/esp32s3/api-reference/peripherals/rmt.html).
The native `tests/fixtures/s3_idf_rmt_loopback` project uses unmodified
ESP-IDF 5.3.2 TX/RX drivers: a copy encoder first sends three plain payload
words and a trailing symbol from TX through GPIO4 while RX captures the pad,
then its interrupt callback wakes the main task. The three measured high/low pairs are
10/12, 8/9, and 6/7 channel ticks. The trailing low joins the TX idle level,
so its final half-duration is not treated as a measured pulse. The same
stock drivers then configure 38 kHz data-only TX modulation and 25 kHz RX
carrier removal, recovering a second three-symbol frame through GPIO4 with
the first two high/low pairs within one carrier cycle of their input widths.
Next, the stock driver sends a two-symbol frame three times with counted
auto-stop; RX measures all six symbols, including loop boundaries. It then
sends a 100/100-tick waveform with `loop_count = -1`; RX observes repeated
iterations until `rmt_disable()` issues `TX_STOP`, after which the same TX
channel is re-enabled and reused. A final
1,024-iteration transaction crosses the 10-bit hardware count limit and
completes through two loop-interrupt batches without a driver patch. This
follows the [S3 RMT loop-count contract](https://docs.espressif.com/projects/esp-idf/en/release-v5.3/esp32s3/api-reference/peripherals/rmt.html).
Finally, a stock two-channel sync manager arms TX0, verifies it remains
blocked for 500 us, then arms TX1 and receives both completion callbacks.
Both streams use the final channel's `TX_START` guest timestamp, matching the
[TRM simultaneous-TX sequence](https://documentation.espressif.com/esp32-s3_technical_reference_manual_en.pdf).
Channel teardown succeeds, and a ten-beat FreeRTOS heartbeat continues. Two
interpreter runs have identical guest and MMIO reports with zero unsupported
accesses. ESP-IDF's two `RMT_SYS_CONF` bitfield writes during teardown now
exercise the modeled pulse-RAM power domain: `RMT_MEM_FORCE_PD` gates APB and
engine capacity and loses RAM contents, while force-up or PMU control restores
access without restoring stale pulses. Rebuild and replay with the official
ROM ELF:

```sh
S3_ROM_ELF=/path/to/esp32s3_rev0_rom.elf \
  ./scripts/build-s3-idf-fixture.sh --check rmt-loopback
```

The fixture's pinned image SHA-256 is
`aa848af54fbe34266b9d4753b938728d3acf639aae5dc5b2230f710bd28e07df`
and its matching ELF SHA-256 is
`a993291223bdd136acee943b3bc7bc71736fb5ab7d39905b0224e917f65fd8aa`.
Individual TX pad edges are scheduled only when a watched matrix input or
GPIO interrupt can observe them; the latter has its own rising-edge unit gate.
With no such consumer, the aggregate pulse sink records the full stream
without scheduling every edge; the pinned WLED pulse digest is unchanged.
An infinite loop with neither a pad observer nor an aggregate pulse sink has
no interrupt to deliver, so its phase advances arithmetically on demand
instead of waking the CPU scheduler once per waveform period. This keeps
short background clocks from becoming a host-side event storm while
preserving `GPIO_IN` phase sampling and explicit stop behavior.
A `GPIO_IN` read samples plain or data-only-carrier TX directly from the
planned pulse words and SCLK phase at that guest cycle, including matrix
inversion, software output-enable selection, and IO_MUX input-buffer gating.
A sampled read does not invent past GPIO interrupts or RX transitions.
Always-on carrier, including its idle oscillator, remains explicitly
diagnostic/unknown rather than silently claiming a valid pad level. The
data-only oscillator starts high at TX start and uses the documented
high/low register encoding and group clock; its exact silicon start phase
has not been measured. Mid-segment route changes and electrical line behavior
are not modeled.
The carrier remover joins short opposite-polarity gaps according to the
selected carrier polarity and encoded channel-clock high/low thresholds in the
[S3 technical reference manual](https://documentation.espressif.com/esp32-s3_technical_reference_manual_en.pdf).
The stock RX fixture arms a three-channel-tick filter minimum, rejects a
one-tick GPIO4 glitch, and then uses Arduino's `rmtSetCarrier()` to demodulate
a 38 kHz-style carrier with 25 kHz tolerance into a two-symbol frame. Unit
tests cover both carrier polarities and exact threshold boundaries. Two pinned
runs of each engine match byte-for-byte, the JIT retires native instructions,
and both use zero unsupported RMT or common Arduino-S3 startup sites. The
digital envelope follows guest-clock thresholds;
electrical jitter and analog
tolerance are not modeled. Host-decoded injection still bypasses GPIO, filtering,
and demodulation. DMA and overrun/error behavior when the guest fails to
service a threshold in time remain unsupported. Markerless loops, counted
loops without auto-stop, and dynamic sync-group changes remain unsupported
and diagnostic. Odd final
half-symbol encoding and GPIO route changes mid-frame also lack hardware
validation. Recheck the RX path with the compiled fixture
and official ROM ELF:

```sh
./scripts/build-s3-arduino-fixture.sh --check rmt-rx
```

The RX fixture's pinned merged image SHA-256 is
`c2e9ac0fa7ee6540d4ff511c35a1be5126868b06e77dfe64f4f9ad3ea9cd9717`
and its matching ELF SHA-256 is
`b7e2cbf74fb0b5374c469d43578fbf548b93329557d14060e3c0b56854fefab3`.
`SOURCE_DATE_EPOCH` fixes Arduino-ESP32's embedded compile date and time to
the fixture revision, so a clean rebuild reproduces both hashes.
For the WLED 16.0.1 S3 4M QSPI image (SHA-256
`eb54c6c3648b7037d54df9f21fe02c9d9606b871faea04ce08b5f6f77dc79c81`),
4 billion aggregate cycles produced 317 completed RMT transmissions and
321,304 pulse words in 13,599 chunks on channel 0. The interpreter and JIT
match at every completed-frame boundary and finish with the same `46F65AC5`
pulse-stream digest, CPU state, and firmware-visible time, with zero
unsupported peripheral accesses in either engine. The JIT executes
1,780,691,463 of 1,819,518,818 retired instructions natively (97.9%). Ordinary
code in the target-described
mask-ROM range is eligible for translation, while the ROM loader's exact
service hooks remain interpreter boundaries. This establishes sustained,
engine-equivalent hardware-output progress, not correct colors on a physical
LED strip.

The `0x6000E000` page is shared by the internal analog I2C host and a private
PHY register bank. Flexe now retains the target-described `0x054..0x170`
configuration window, exposes the eight `rom_read_sar_dout` words as read-only
13-bit quiet-input results, and implements the PHY library's 256-word indexed
RF-frequency memory. Index selection publishes the saved word, the write and
operation strobes complete synchronously in functional mode, the busy bit
clears, and the result field reports the selected channel. The geometry and
bit protocol live in the S3 target descriptor; there are no firmware PCs or
WLED-specific hooks. This removed 6,800 unsupported accesses from the pinned
WLED run while preserving diagnostics outside the modeled bank. It does not
simulate RF propagation, analog calibration noise, or physical SAR voltages.

The radio model also composes the public S3
[SYSCON modem controls](https://github.com/espressif/esp-idf/blob/v5.5.1/components/soc/esp32s3/register/soc/apb_ctrl_reg.h)
at `0x6002600C..0x60026018` with those private apertures. The two baseband
configuration words retain their writable state; target-described reset bits
restore and hold only their selected FE, NRX/baseband, Bluetooth, Wi-Fi MAC,
or WDEV banks; and the documented clock bits qualify IQ estimation, Wi-Fi MAC
ready, Bluetooth register initialization, the random source, and the
baseband-time latch. Unknown words on the same SYSCON page remain diagnostic.
The SENS model now also retains the SAR2 power-detector capacitance trim while
its quiet RF input continues to produce no fabricated sample. These are
general register/domain relationships, not firmware-PC hooks or automatic
radio success.

The same shared SYSCON page now has a separate target-described owner for the
front-end memory power controls and the 11 SRAM/3 ROM bank clock, force-down,
and force-up policies. It supplies the documented reset values, masks reserved
bits, and publishes normalized semantic state to future cache/power consumers;
the radio register model composes with it through an explicit fallback chain.
This removed all 11 accesses at `0x6002609C`, `0x600260A8`, and `0x600260B0`
from WLED without claiming cache timing or erasing a backing store when a bank
policy changes.

That owner also exposes all four flash and four PSRAM access-control regions:
attributes, 32-bit start addresses, and 64-KiB-page counts have exact reset and
reserved-bit behavior and are published as target-independent semantic state.
Arduino-ESP32 3.3.11's normal cache-safety initialization therefore programs
the real region bank instead of producing 11 startup diagnostics. Restrictive
regions are not yet connected to memory-access fault/reject generation, so
configuration readback does not claim enforcement.

The SENSITIVE v1 owner now models the complete cache-data-array and internal
SRAM allocation group, including both write-once locks, seven CPU/cache SRAM
bank selectors, trace allocation, MAC-dump/log usage, and retention policy.
Documented reset values and reserved-bit masks are exact, and a normalized
observer makes the allocation state available to future cache and power
consumers. This removed all eight accesses at `0x600C1004` and `0x600C1014`
from WLED without pretending that selecting banks already models cache
topology or latency.

The target-described ASSIST_DEBUG recorder implements the independent
PDEBUG-enable and recording controls for both S3 cores. While both controls
are set, its PC and architectural SP registers reflect live emulator state;
clearing either control freezes the latest pair for the crash-record path.
The model also supplies the masked silicon DATE register and adds no work to
the instruction dispatch loop. ESP-IDF's normal APP-CPU startup writes at
`0x600CE0D8` and `0x600CE0DC` therefore exercise real recorder state rather
than an address whitelist. Area/stack watchpoint interrupts, detailed
instruction and load/store debug-bus fields, exception records, and trace
memory remain unsupported and diagnostic.

Both S3 SYSTEM peripheral clock/reset banks are also represented as one
target-described 64-domain state surface. Reserved bank-one positions remain
masked, device-specific callbacks still drive the attached timer, UART, I2C,
SPI, LEDC, SHA, and USB Serial/JTAG models, and the complete state is observable
for additional consumers. This removed all five accesses at `0x600C0018` and
`0x600C0020`; it does not claim reset or clock effects for device models that
have not yet attached a consumer.

The SENS peripheral clock/reset pair is likewise complete and target
described. It publishes named IO-mux, SAR ADC, temperature-sensor, RTC-I2C,
and coprocessor state while retaining the existing ADC and temperature reset
effects; reserved bits still reach the diagnostic owner. This removed WLED's
`0x60008904` write without teaching the model a firmware PC or treating the
rest of the SENS page as generic storage.

On an Apple-silicon MacBook, a `Release`/LTO/native build completed three
alternating interpreter runs in 11.21--12.40 seconds (1.34--1.49x real time)
and three JIT runs in 6.86--7.27 seconds (2.29--2.43x). These are throughput
results for this pinned scenario, not cycle-accuracy claims; host load and
thermal state still matter.

The same audit fell from 223 to 74 unsupported accesses when modem-control
behavior replaced fallback handling, then to 55 when the target-described RTC
digital domains became functional, and to 48 after modeling the independent
RTC fast-clock mux and DATE/LDO-trim readback, then to 38 after resolving the
RTC regulator force pairs and RTC-local PWC domains, then to 34 after exposing
the SYSTEM light-sleep memory policy and radio low-power clock state, then to 23
after modeling the SYSCON on-chip-memory policy, then to 15 after modeling the
SENSITIVE cache/SRAM allocation policy, then to 10 after exposing the complete
SYSTEM peripheral clock/reset banks, and then to 9 after completing the SENS
peripheral clock/reset fabric, and then to 2 after completing the target-
described RTC OPTIONS0, ANA_CONF, and digital pad/global isolation controls.
The final two sites disappeared after composing the dual-core ASSIST_DEBUG
recorder, leaving zero unsupported MMIO sites in the pinned four-billion-cycle
WLED run. DIG_PWC, PWC, REG's force pairs, the modeled domain fields of
DIG_ISO, CLK_CONF's fast selector, DATE, OPTIONS0, ANA_CONF, MEM_PD_MASK,
BT_LPCK_DIV, and the APP-CPU recorder controls no longer appear in the
inventory. This zero is specific to the exercised WLED path; unmodeled
ASSIST_DEBUG functions and unexercised peripherals remain explicitly
diagnostic.

Recheck with the external image and ROM ELF:

```sh
S3_WLED_BIN=/path/to/WLED_16.0.1_ESP32-S3_4M_qspi.bin \
S3_ROM_ELF=/path/to/esp32s3_rev0_rom.elf \
  ./scripts/check-s3-wled-rmt.sh
```

A separate build of WLED v16.0.1 from its tagged source (`29b389d`, image
SHA-256 `8e165290df301b0bcea5763637db1f0ec9d4ad5b1d07b8588a1baa5ebd50bad5`,
ELF SHA-256 `a6ca2cb74ce281fd4f3846b6347e3f2abc434d1ced2d553630ba3a8478fe1965`)
with its matching ELF reaches `AsyncServer::begin()` and then
`tcp_listen_with_backlog()` at about 1.16 billion guest cycles. Its UDP
sockets reach the host bridge. WLED's AsyncTCP web server uses lwIP's raw TCP
PCB API, which the S3 BSD-socket bridge alone does not expose. The optional
`--net-hostfwd ap:HOST_PORT:4.3.2.1:80` path connects a loopback host listener
to WLED's own AP netif through Ethernet frames (libslirp is required). The
source-built image served its gzip HTML UI, returned JSON state, accepted a
brightness/red-color JSON POST, and read that state back. The pinned external
gate repeats the complete interaction; it accepts either a parked `WAITI`
stop or a clean 12-billion-cycle budget stop with at least one billion
retired instructions, since concurrently active network/DMX tasks need not
park at the exact limit:

```sh
S3_WLED_BIN=/path/to/matching/firmware.bin \
S3_WLED_APP_ELF=/path/to/matching/firmware.elf \
S3_ROM_ELF=/path/to/esp32s3_rev0_rom.elf \
  ./scripts/check-s3-wled-http.sh
```

The guest AP address `4.3.2.1` was observed in WLED's gratuitous ARP frames;
it is a gate input, not a hardcoded emulator address. This source-built image
is not byte-identical to the pinned release image; its ELF must not be used
to symbolize that release binary.

The S3 RTC SAR ADC model follows Espressif's S3 `sens_reg.h` and `adc_ll.h`
register contract: software pad selection and START, hardware-owned DONE/DATA,
and an idle measurement status after a synchronous fast-mode conversion. An
internal analog-register I2C bit selects ground for the SDK's hardware
calibration path; RTC_CNTL_ANA_CONF's SAR-I2C power bit now gates that analog
slave, so a powered-off command cannot alter calibration state or report
completion. The other analog power/reset bits retain documented register
state but remain diagnostic because their PLL/RF effects are not modeled.
A ground conversion yields zero rather than claiming an
ordinary external-pad sample. `tests/fixtures/s3_adc/s3_adc.ino`, compiled with
Arduino-ESP32 3.3.11 for `esp32:esp32:esp32s3`, reads GPIO4 (ADC1 channel 3)
and GPIO11 (ADC2 channel 0) repeatedly. With injected raw codes 2645 and
1450, both engines emit the same five matching pairs within two billion
aggregate cycles, and two runs of each engine replay byte-for-byte. Its
merged image SHA-256 is
`5178a6d97110c3682664ecbe602e0bbce40367c639ebc46ab3f714d879c4ec34`
and matching ELF SHA-256 is
`7c156185527896c78a8cf8de1155b10f417350ac1dad27a7ef255298fdad99d0`.
The APB ADC2 arbiter retains its reset priorities and software configuration.
When grant is forced, only a forced RTC grant completes an RTC ADC2 conversion;
the SENS RTC_FORCE bit bypasses that arbiter. With no modeled competing
requester, unforced RTC conversion proceeds. Digital and Wi-Fi/PWDET
requesters and simultaneous contention are still unsupported, and forcing
those owners remains diagnostic. The interpreter's complete production run
uses zero unsupported accesses. This is raw-code functional behavior, not
physical analog, attenuation, calibration accuracy, or continuous/DMA ADC
fidelity. Recheck with the external compiled fixture:

```sh
./scripts/build-s3-arduino-fixture.sh --check adc
```

The S3 LEDC model follows Espressif's `esp32s3` `ledc_reg.h`,
`system_reg.h`, `gpio_sig_map.h`, and `interrupts.h`: eight low-speed-only
channels at `0x60019000`, four timers, shadowed divider/resolution updates,
immediate timer reset/pause, duty-change and timer-overflow interrupts, and
SYSTEM clock/reset gating. It resolves the live GPIO output-matrix route and
emits aggregate PWM state (frequency, duty, enable and inversion) to a host
sink; it does not emit electrical edges or model the S3 overflow-count
feature. The stock Arduino-ESP32 3.3.11
`tests/fixtures/s3_ledc/s3_ledc.ino` sets GPIO4 to 5 kHz at 8-bit
resolution, cycles through duties 64/192/96/0, and reads each back after a
PWM period. The pinned merged image SHA-256 is
`510702d6b837ed5115776882cd3658a900e82f3ad72d51b7b7ce87430df7268e`,
with ELF SHA-256
`61a0c58e32a915e09fbf15b4dd5f63c2346370fb8e436d0a995e14a381596ef7`.
`scripts/check-s3-ledc.sh` requires byte-identical event replay in both engines,
matching interpreter/JIT events, native JIT retirement, and zero unsupported
accesses. The fixture helper derives `SOURCE_DATE_EPOCH`, caches the exact
inputs, and can reproduce the artifact with `--rebuild`:

```sh
./scripts/build-s3-arduino-fixture.sh --check ledc
```

The optional AP Memory 8 MiB OPI profile is selected by the board, not
inferred from firmware: `--psram ap-8m-opi` attaches it on CS1. The pinned
stock Arduino-ESP32 3.3.11 fixture checks 8 KiB of allocated external RAM
repeatedly; two runs of each engine must be byte-identical and agree on guest
output, the JIT must retire native instructions, and the interpreter must use
zero unsupported accesses. The same image without the flag must report no
PSRAM. Build and run it with:

```sh
./scripts/build-s3-arduino-fixture.sh --check psram-opi
```

The native S3 `esp_wifi_internal_tx`/`tx_by_ref` and
`esp_wifi_internal_reg_rxcb` symbols now form an optional Ethernet-frame
service boundary. A host backend can accept guest frames and queue frames
through the netif's registered RX callback; the RX buffer remains mapped
until the guest calls `esp_wifi_internal_free_rx_buffer`. Without an attached
backend, guest transmit and free functions execute unchanged. A synthetic
S3 ELF unit gate checks callback registration, transmit fallback, frame
delivery, buffer reuse, and statistics. This is a network transport boundary,
not modeled RF/PHY, wireless client association, cache timing, or
cycle-accurate network delivery. The S3 network workflow is still partial
outside the validated AP scenario.

The S3 Bluetooth register window includes a captured baseband clock:
requesting a latch publishes a half-slot count and a down-counting subslot
phase from shared guest time. Its register protocol and 312.5 us half-slot
interpretation come from `r_rwip_time_get` in the official
`esp32s3_rev0_rom.elf`. The target descriptor also supplies the immutable
link-layer identity at `0x60031004` and consumes bit 31 of the adjacent
register-initialization command. Those two semantics were recovered from the
exact Arduino-ESP32 2.0.11 controller archive: `r_lld_core_init` validates
identity `0x09001B00`, while `r_rwip_driver_init` waits for hardware to consume
the command. They are reset-state and write-mask descriptors in the generic
radio aperture, not firmware-PC hooks or fabricated HCI success.

With those semantics, the [official v1.16.0 MultiBoard S3 release](https://github.com/justcallmekoko/ESP32Marauder/releases/tag/v1.16.0)
(SHA-256 `b6b61e6c6c41bc78422d405d14117ab5aa6ec0cdee751327ea568232e52dd0be`)
runs its native controller far enough to deliver and consume the NimBLE HCI
Reset completion, tears Bluetooth down, initializes/starts/stops/deinitializes
the native Wi-Fi stack, completes its LED and absent-GPS delays, and prints the
v1.16.0 command prompt without an assertion, watchdog, or software reset. The
gate now requires nonzero native instruction retirement, so this path is
exercised under the S3 JIT rather than merely with JIT-capable code present. Its
unsupported-access ceiling was 88 after the shared internal analog/private-PHY,
modem clock/reset, and RTC digital-domain models described below. The ROM's
read-modify-write baseband-clock capture protocol subsequently reduced that
ceiling to 6. Target-described GP-SPI data producers and UART TX producers now
make those last GPIO-matrix routes functional without board-pin or firmware-PC
exceptions, reducing the same gate to zero unsupported accesses. The GPS probe
makes the application-ready boundary occur near 5 billion aggregate cycles;
the earlier 2-billion-cycle cutoff was a normal `WAITI` during those firmware
delays, not a deadlock. Pin and replay that boundary with:

```sh
S3_MARAUDER_BIN=/path/to/esp32_marauder_v1_16_0_multiboardS3.bin \
S3_ROM_ELF=/path/to/esp32s3_rev0_rom.elf \
  ./scripts/check-s3-marauder.sh
```

The gate keeps the general sandbox transport connected through the initial
prompt, sends the bytes for `help\n` through UART0, checks the firmware's
command header and a representative entry, and requires a second prompt.
The accepted run requires zero unsupported accesses so the interaction cannot
hide a register-model regression. UART remains an aggregate-byte model with an
idle-high pad, and GP-SPI remains an aggregate-transaction model; neither claim
implies that fast mode synthesizes every serial wire edge.

This is a controller-bootstrap compatibility boundary, not a claim that
Bluetooth packets, scanning, coexistence timing, RF propagation, or the
private analog PHY are complete.

For the NerdMiner v1.8.3 S3 factory image (SHA-256
`8dd4bad43944def2287cf8b6bed7762c1881b6e7f04f7bd1556ad555202f8c22`),
an earlier interpreter audit, before RTC power-sequencer support, reported
6,919 unsupported accesses at 4 billion aggregate cycles. The same fixed audit
now reports zero unsupported MMIO sites. Its final pair was the documented
read-modify-write that enables the controller-wide AHB GDMA v1 clock; Flexe
now retains and masks the shared clock, arbitration-disable, and AHB-master
reset controls instead of routing them through generic MMIO fallback. The
full POST/reset replay subsequently exposed nine one-shot restart-path sites:
five RTC watchdog configuration writes and four external-memory DMA
clock/reset accesses. Target-described watchdog reset controls now retain
their exact readback, pause-in-sleep is functional, and reset selectors and
pulse widths converge on Flexe's atomic reset boundary. The SYSTEM model
publishes the EDMA clock/reset gate as standalone device state. The EDMA
transfer engine itself and reset-pulse electrical duration are not claimed.

GigaDevice `0x5A`
SFDP reads now use the documented
[GD25Q32C 4 MiB](https://download.gigadevice.com/Datasheet/DS-00088-GD25Q32C-Rev4.1.pdf)
or [GD25Q64C 8 MiB](https://download.gigadevice.com/Datasheet/DS-00111-GD25Q64C-Rev3.2.pdf)
parameter table only when both JEDEC ID and physical capacity match; other
profiles remain explicitly unsupported. The latter clears this image's one
bootloader SFDP diagnostic. Six previously reported SPI1 commands are
16-bit octal-PSRAM register probes on CS1: with no PSRAM attached in Flexe's
default S3 profile, they now clock through and return undriven high bits,
without pretending to supply PSRAM. Of these, 6,728 were in the `0x6000E000`
RF/PHY window; the hottest named callers included `wr_rf_freq_mem`,
`set_chan_freq_sw_start`, and `bt_txpwr_freq`. That inventory motivated the
target-described private-PHY bank and indexed-memory protocol above rather
than a firmware-address bypass. The remaining historical inventory includes
documented SYSCON, RTC, and sensor registers. The `--unhandled-report`
inventory and total accumulate across firmware-requested resets; neither a
smaller count nor the working network scenario proves real RF behavior.
Repeat the measurement with the matching application and ROM ELFs:

```sh
./build/xtensa-emu -N --target esp32s3 -R "$FLEXE_S3_ROM_ELF" \
  -s "$FLEXE_S3_APP_ELF" --usb-console --unhandled-report \
  -c 4000000000 "$FLEXE_S3_FACTORY_BIN"
```

The app/ROM binaries are external inputs. The interactive gate drives a real
`GET /wifi` request against the guest's WebServer and WiFiManager, submits the
provisioning form, waits for the firmware's requested reset, and checks that
the new boot loads the saved pool and wallet from SPIFFS. The complete replay
must finish with zero unsupported MMIO sites:

```sh
S3_FACTORY_BIN=/path/to/NerdminerV2_factory.bin \
S3_APP_ELF=/path/to/matching/firmware.elf \
S3_ROM_ELF=/path/to/esp32s3_rev0_rom.elf \
  ./scripts/check-s3-nerdminer-portal.sh
```

The S3 BSD-socket service no longer assumes this Arduino build's descriptor
base `48`. ESP-IDF's
[`esp_vfs_lwip_sockets_register`](https://github.com/espressif/esp-idf/blob/v5.5.1/components/lwip/port/esp32xx/vfs_lwip.c)
registers the linked `[LWIP_SOCKET_OFFSET, MAX_FDS)` interval with VFS;
Flexe observes that native guest call and uses its arguments, leaving the
guest implementation intact. The stock ESP-IDF v5.3.2 fixture in
`tests/fixtures/s3_idf_socket_range` builds with `CONFIG_LWIP_MAX_SOCKETS=10`
and demonstrates guest FDs `54..63`, `select()`, exhaustion, and byte-identical
interpreter replays (image SHA-256
`a3b70aeb8d5689d81d246a14ef3ed4df7add3139eee6a4cd3710c2ed6a53799d`,
ELF SHA-256
`282bca88e2aaffc68d399fce5f71cdd4175f3a4e5d2cef19be35efe3186de553`).
Without the VFS registration symbols or for a range beyond Flexe's current
64-FD `select()` layout, the bridge does not guess a base and reports the
unsupported condition. Rebuild and run the gate with:

```sh
S3_ROM_ELF=/path/to/esp32s3_rev0_rom.elf \
  ./scripts/build-s3-idf-fixture.sh --check socket-range
```

The
[default ESP-IDF RTC-WDT handoff](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-reference/system/wdts.html)
is represented, but images built to retain that watchdog into user code need
a separate boot-configuration path. Unsupported PHY diagnostics remain a
blocker for real radio behavior.
