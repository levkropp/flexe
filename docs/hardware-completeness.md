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
   experimental.
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
| S3 optional 8 MiB octal PSRAM | Partial (MSPI/MMU) | `tests/test_spi_mem.c` covers mode registers, hybrid burst and row crossing; `scripts/check-s3-psram-opi.sh` checks stock Arduino-ESP32 3.3.11 external-RAM allocation and repeated array traffic; default board remains unpopulated | AP Memory APS6408L-3OBMx command subset is modeled; DQS/electrical timing, refresh/PASR retention, and other PSRAM chips/board wirings are not. |
| S3 NVS, SPIFFS, reset persistence | Partial | `tests/test_loader.c`, `tests/test_spi_mem.c`, `scripts/check-s3-idf-nvs.sh`, NerdMiner POST/save/restart/reload gate, `scripts/check-s3-idf-sleep.sh` | Flash array and NOR chip state survive SoC restart; S3 deep-sleep flash power-down retains profiled nonvolatile status, while other flash profiles and partition/filesystem variants need gates. |
| S3 GPIO/RTCIO/IO_MUX | Partial (MMIO) | `tests/test_gpio.c`, `tests/test_rtc_io.c`, `tests/test_rtc_cntl.c`, `scripts/check-s3-idf-gpio-isr.sh`, stock Arduino ADC gate, native EXT0/EXT1 wake gate | Stock ESP-IDF's per-pin ISR, FreeRTOS task notification, digital open-drain release, IO_MUX input-buffer gating, driven-output input feedback, RTC GPIO wake, and pad hold work; physical pulls, drive strength, and electrical levels are not complete. |
| S3 UART/USB console | Partial (MMIO and host I/O) | Official ESP-IDF and Arduino UART output gates; `tests/test_system_clock.c` covers independent UART clock/reset and host RX gating; the pinned Marauder gate injects a binary-safe host UART event and verifies its CLI response; `tests/test_usb_serial_jtag.c` covers USB Serial/JTAG clock/reset, packet, and SOF gating; `scripts/check-s3-idf-usb-serial-jtag.sh` replays stock ESP-IDF driver RX/TX | USB protocol/electrical behavior and all UART DMA modes are not claimed. |
| S3 I2C, GP-SPI | Partial (MMIO) | `tests/test_peripherals.c`, `tests/test_system_clock.c`, `tests/test_spi_mem.c`; stock Arduino Wire and I2C-slave gates, direct ESP-IDF 5.3 I2C-master and SPI-master replay gates | More I2C guest-driver/device combinations, slave overflow/clock stretching, GPIO-matrix I2C waveforms, and GP-SPI segmented/slave modes remain. |
| S3 GDMA | Partial (MMIO) | `tests/test_crypto.c` checks chained TX/RX descriptors, ownership/writeback, errors, and per-channel level interrupts on both cores; `tests/test_peripherals.c` exercises GP-SPI full-duplex GDMA | Full priority and peripheral interactions remain unverified. |
| S3 SYSTEM/SYSCON clock and power policy | Partial (MMIO) | `tests/test_system_clock.c`, `tests/test_syscon_memory.c`, WLED/Marauder production audits | Peripheral gates and the 11-SRAM/3-ROM plus RF front-end memory policies have exact reset/readback state and semantic observers; cache timing and actual bank power loss remain unsupported. |
| S3 timers, watchdogs, RTC | Partial (MMIO) | `tests/test_systimer.c`, `tests/test_timer_group.c`, `tests/test_rtc_cntl.c` (supply and power-domain resolution, CPU-follow, sleep/wake, and stall enable), ESP-IDF cross-core, restart, native timer and GPIO light/deep-sleep gates | Timer and EXT0/EXT1 wake work; digital and RTC-local domain state is observable and selected digital consumers are connected, but brownout voltage detection/reset, touch/ULP wake, analog transition timing, and other reset causes remain unsupported. |
| S3 LEDC PWM | Partial (MMIO) | `tests/test_ledc_v1.c`, `scripts/check-s3-ledc.sh`: stock Arduino repeatedly drives GPIO4 at 5 kHz with four readback duties and byte-identical replay | Aggregate PWM output and timed fade/interrupt are modeled; individual electrical edges and overflow-counter behavior are not. |
| S3 RMT TX | Partial (MMIO and GPIO-matrix output) | `tests/test_rmt_v1.c`, `scripts/check-s3-wled-rmt.sh`, `scripts/check-s3-idf-rmt-loopback.sh`; WLED 16.0.1 emits 317 sustained pulse frames, and stock ESP-IDF drivers exercise plain, carrier, finite, infinite, and synchronized two-channel output | Pad edges are scheduled only for a watched matrix input or GPIO interrupt; `GPIO_IN` polls on demand. End-marker finite loops work with auto-stop and batching beyond 1023; end-marker infinite loops run until `TX_STOP`; selected synchronous channels share the final `TX_START` timestamp. Markerless loops, counted loops without auto-stop, always-on carrier, dynamic sync-group changes, silicon-calibrated carrier phase, and fine status remain unsupported. |
| S3 RMT RX | Partial (MMIO, filtered/demodulated GPIO input, host symbols) | `tests/test_rmt_v1.c`, `scripts/check-s3-rmt-rx.sh`, `scripts/check-s3-idf-rmt-loopback.sh`; stock Arduino-ESP32 3.3.11 filters a GPIO4 glitch, demodulates a carrier waveform, and receives a 96-symbol host frame; stock ESP-IDF 5.3.2 receives both plain and modulated TX pad pulses through its ISR callback | Host samples, software GPIO feedback, and RMT TX loopback share the GPIO-matrix edge path. One explicit demod diagnostic and two RMT memory-power-down diagnostics remain; DMA, odd pulse tails, delayed-ISR overrun, and dynamic mid-segment route changes remain unsupported. |
| S3 RTC SAR ADC | Partial (MMIO plus host samples) | `tests/test_sens.c`, `tests/test_apb_saradc.c`, `scripts/check-s3-adc.sh` | Digital/DMA conversion, ULP, contention, and physical calibration remain unsupported. |
| S3 network-facing workflow | Partial (service shim) | NerdMiner BSD-socket portal, WLED native lwIP/Ethernet UI and JSON state, and `scripts/check-s3-idf-socket-range.sh` with a stock 10-socket ESP-IDF build | Wi-Fi RF/PHY, association realism, and general transport modes are unsupported; the socket bridge requires ELF symbols and a VFS range within its 64-FD `select()` layout. |
| S3 Bluetooth controller bootstrap | Partial (MMIO) | `tests/test_radio.c` checks modem clocks, selective reset and RTC power/isolation domains, baseband time, immutable controller identity, and command consumption; `scripts/check-s3-marauder.sh` boots the official v1.16.0 MultiBoard S3 image through native Bluetooth and Wi-Fi setup, injects `help` through UART0, and verifies its response and next prompt | General controller scheduling, Bluetooth packets, coexistence fidelity, and RF remain unsupported. |
| Cycle/cache/electrical/RF fidelity | Unsupported | Outside this functional milestone | Requires calibrated hardware traces and declared tolerances. |

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
other host UART injection. If a connected frontend is the only possible
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
twice with byte-identical UART, USB, and unsupported-MMIO digests. Neither
the USB controller aperture nor the RTC USB PHY mux has unsupported
accesses; 70 other accesses remain diagnostic. The pinned
application image SHA-256 is
`b8cc669dbd8fb46f5dc93c5dad948cba266fb773ea6a62656455e9e60de82b37`
and matching ELF SHA-256 is
`42e6c6e2e38372272955ad2df6e09340178c619fb7bd82957c000b0c28608678`.
This checks driver-level packet and interrupt progress, not USB enumeration,
electrical timing, or large/bursty packet stress. Rebuild with ESP-IDF commit
`9d7f2d69f50d1288526d4f1027108e314e8c879f` and run with external artifacts:

```sh
idf.py -C tests/fixtures/s3_idf_usb_serial_jtag \
  -B /tmp/flexe-s3-usj-build -D SDKCONFIG=/tmp/flexe-s3-usj-sdkconfig \
  -D IDF_TARGET=esp32s3 build
cmake --build build --target flexe-s3-idf-usb-serial-jtag-test
S3_IDF_USJ_BIN=/tmp/flexe-s3-usj-build/s3_idf_usb_serial_jtag.bin \
S3_IDF_USJ_ELF=/tmp/flexe-s3-usj-build/s3_idf_usb_serial_jtag.elf \
S3_ROM_ELF=/path/to/esp32s3_rev0_rom.elf \
  scripts/check-s3-idf-usb-serial-jtag.sh
```

Independently rebuilt images may provide matching `*_SHA256` overrides,
because ESP-IDF embeds build metadata.

S3 remains experimental. The NerdMiner filesystem result is a meaningful
end-to-end flash-format/mount check. With the matching application ELF, the
host-backed socket/select boundary now lets the unmodified firmware serve
`GET /wifi` as `200 OK` with its 4,985-byte configuration HTML. This is a
service shim, not a modeled Wi-Fi radio: the page reports no networks, and
the full POST/restart/reload replay still reports about 35,000 unsupported
accesses. The separate 4-billion-cycle audit below counts 6,919 (mostly
RF/PHY). The S3 application handoff now clears the power-on flash-boot
watchdog mode skipped with the second-stage bootloader. Restoring ROM-owned
BSS and interface state from the official ROM ELF during a software restart
lets NerdMiner initialize again and reload its saved settings instead of
panicking during PSRAM setup, while physical SRAM outside those sections
remains intact for app `.noinit`.
The scripted POST/restart/reload gate exercises that full path. WLED now
provides a second, independently sourced production interactive network and
output scenario. The remaining RF/PHY gaps prohibit a
production-support claim. ROM images and third-party firmware binaries are
not copied into this repository.

The official ESP-IDF v5.3.2 `examples/get-started/hello_world` image built
from commit `9d7f2d69f50d1288526d4f1027108e314e8c879f` (application image
SHA-256 `e9ce7296ec826e19216ef9ee3940857f561184fefef06a4d4dfa20a5494dfc54`,
ELF `643073d572d06dce114bb9a70f41ef975ff2ce76dd87696316baf31af20216d8`)
now reaches `app_main()`, counts down through ten guest seconds, requests its
own software reset, and does so again after both cores restart. The separate
`tests/fixtures/s3_idf_crosscore` project, built with the same IDF, pins a
producer to CPU1 and checks 32 queue-message/notification round trips with
`app_main()` on CPU0. It then uses the official `esp_cpu_stall(1)` and
`esp_cpu_unstall(1)` calls to verify that an active CPU1 task stops making
progress and resumes without losing its state, before sustained 100 ms
heartbeats. Its image SHA-256 is
`f1b90e7ae15c5eba69d75aef6372fa0cbf387acadb75265a26d0d4cdb6943631`
and ELF SHA-256 is
`84e74cdcdfac1bbc67caef66b9fa49366d8f6940c81f794c8253d47301a4f230`.
Both external-image gates compare a complete second replay byte-for-byte,
including the unsupported-MMIO report; hello-world reports 156 unsupported
accesses across its two boots and cross-core reports 70 in one boot. This
proves these FreeRTOS interactions, not simultaneous core execution or timing
fidelity. Run with matching external artifacts:

```sh
S3_IDF_HELLO_BIN=/path/to/hello_world.bin \
S3_IDF_HELLO_ELF=/path/to/hello_world.elf \
S3_ROM_ELF=/path/to/esp32s3_rev0_rom.elf \
  ./scripts/check-s3-idf-hello.sh
S3_IDF_CROSSCORE_BIN=/path/to/s3_idf_crosscore.bin \
S3_IDF_CROSSCORE_ELF=/path/to/s3_idf_crosscore.elf \
S3_ROM_ELF=/path/to/esp32s3_rev0_rom.elf \
  ./scripts/check-s3-idf-crosscore.sh
```

The gates default to these pinned image/ELF hashes and the official revision-0
ROM ELF SHA-256 `c0ce0f338d1de1bdc6efbef1591779a2a42c1ab7d759d3c6ae8ae63a7dd34cfd`.
With ESP-IDF v5.3.2 installed, build from its unmodified example and the
in-tree cross-core fixture (keep generated configuration outside the repo):

```sh
idf.py -C "$IDF_PATH/examples/get-started/hello_world" \
  -B /tmp/flexe-s3-hello-build -D SDKCONFIG=/tmp/flexe-s3-hello-sdkconfig \
  -D IDF_TARGET=esp32s3 build
idf.py -C tests/fixtures/s3_idf_crosscore \
  -B /tmp/flexe-s3-crosscore-build -D SDKCONFIG=/tmp/flexe-s3-crosscore-sdkconfig \
  -D IDF_TARGET=esp32s3 build
```

Independently rebuilt images can supply matching `*_SHA256` overrides, since
ESP-IDF embeds build metadata in the application.

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
`cce3abfb4191663a0c6125180e0ea91804bf8f71387aeeff807e9b40bb2175a2`
and matching ELF SHA-256 is
`a7cbc0ff14284c65573b2ca075bfea0a85a28707c1df35c131d39fd39bb0e294`.
The two complete replays are byte-identical; 162 other unsupported accesses
across both boots remain visible. No unsupported sites remain for S3 sleep
timer, state, wake-enable, cause, or digital-domain fields. The nominal
slow-clock model yielded about 48.9/19.3 ms for the guest's requested 50/20 ms,
so this is functional wake ordering, not calibrated sleep duration. Touch/ULP
wake and analog voltage-transition timing are not modeled; an unarmed timer or
those sources do not synthesize a wake.
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
The S3 `RTC_CNTL_BROWN_OUT_REG` now retains documented configuration fields,
resets to the specified defaults, and treats counter-clear as a write-only
strobe. Its detector remains clear under Flexe's fixed nominal supply; analog
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
latched state. Digital autohold and unconnected pad-isolation effects remain
diagnostic.

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

Rebuild and run with the matching external artifacts:

```sh
idf.py -C tests/fixtures/s3_idf_sleep \
  -B /tmp/flexe-s3-sleep-build -D SDKCONFIG=/tmp/flexe-s3-sleep-build/sdkconfig \
  -D IDF_TARGET=esp32s3 build
S3_IDF_SLEEP_BIN=/tmp/flexe-s3-sleep-build/s3_idf_sleep.bin \
S3_IDF_SLEEP_ELF=/tmp/flexe-s3-sleep-build/s3_idf_sleep.elf \
S3_ROM_ELF=/path/to/esp32s3_rev0_rom.elf \
  ./scripts/check-s3-idf-sleep.sh
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
`d4897a5ea5b5805bfda3ac3f624788f2600c9f17653e07f8dee7bea67df91314`
and ELF SHA-256 is
`27a2d3710e867e0310494a29a6c3612878ec10796a5ea32beeb30a4eca627d4d`.
The gate reports 162 unrelated unsupported accesses across both boots; the
EXT selection/state/status and digital-domain sites are modeled.
Physical pull resistors, voltage thresholds, glitches/filtering, and pad
electrical behavior are not inferred from the host's digital level:

```sh
idf.py -C tests/fixtures/s3_idf_gpio_wake \
  -B /tmp/flexe-s3-gpio-wake-build \
  -D SDKCONFIG=/tmp/flexe-s3-gpio-wake-build/sdkconfig \
  -D IDF_TARGET=esp32s3 build
S3_IDF_GPIO_WAKE_BIN=/tmp/flexe-s3-gpio-wake-build/s3_idf_gpio_wake.bin \
S3_IDF_GPIO_WAKE_ELF=/tmp/flexe-s3-gpio-wake-build/s3_idf_gpio_wake.elf \
S3_ROM_ELF=/path/to/esp32s3_rev0_rom.elf \
  ./scripts/check-s3-idf-gpio-wake.sh
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
replays have
identical UART and unsupported-MMIO digests, with no unsupported GPIO
controller or GPIO4 IO_MUX access. The other 70 startup accesses remain
diagnostic. The pinned image SHA-256 is
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
idf.py -C tests/fixtures/s3_idf_gpio_isr \
  -B /tmp/flexe-s3-gpio-isr-build \
  -D SDKCONFIG=/tmp/flexe-s3-gpio-isr-sdkconfig \
  -D IDF_TARGET=esp32s3 build
cmake --build build --target flexe-s3-idf-gpio-isr-test
S3_IDF_GPIO_ISR_BIN=/tmp/flexe-s3-gpio-isr-build/s3_idf_gpio_isr.bin \
S3_IDF_GPIO_ISR_ELF=/tmp/flexe-s3-gpio-isr-build/s3_idf_gpio_isr.elf \
S3_ROM_ELF=/path/to/esp32s3_rev0_rom.elf \
  scripts/check-s3-idf-gpio-isr.sh
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
`6eb44365aa80da064ab9b861ecc8210c31c4326216d89902da8d432afee04989`
and ELF SHA-256 is
`0992d03138cdfd968b974968c6abcf1f4fec01ab35a233d737dada8b81414588`.
The gate checks two byte-identical runs and still reports 156 unsupported
accesses across its two boots:

```sh
idf.py -C tests/fixtures/s3_idf_nvs \
  -B /tmp/flexe-s3-nvs-build -D SDKCONFIG=/tmp/flexe-s3-nvs-sdkconfig \
  -D IDF_TARGET=esp32s3 build
S3_IDF_NVS_BIN=/path/to/s3_idf_nvs.bin \
S3_IDF_NVS_ELF=/path/to/s3_idf_nvs.elf \
S3_ROM_ELF=/path/to/esp32s3_rev0_rom.elf \
  ./scripts/check-s3-idf-nvs.sh
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
S3 replay runs match byte-for-byte, with no unsupported I2C MMIO sites;
350 unrelated startup accesses across the two boots remain unsupported. This
gate does not test a guest-initiated restart, electrical timing, bus
contention, or other devices:

```sh
arduino-cli compile --fqbn esp32:esp32:esp32s3 \
  --build-path /tmp/flexe-s3-i2c-wire-build \
  --build-property compiler.optimization_flags=-Os tests/fixtures/i2c_wire
S3_I2C_BIN=/tmp/flexe-s3-i2c-wire-build/i2c_wire.ino.merged.bin \
S3_I2C_ELF=/tmp/flexe-s3-i2c-wire-build/i2c_wire.ino.elf \
S3_ROM_ELF=/path/to/esp32s3_rev0_rom.elf \
RUNNER=./build/flexe-i2c-wire-test ./scripts/check-s3-i2c-wire.sh
```

The separate `tests/fixtures/s3_idf_i2c_master` project calls ESP-IDF
v5.3.2's [new I2C master driver](https://docs.espressif.com/projects/esp-idf/en/v5.3.2/esp32s3/api-reference/peripherals/i2c.html)
directly, without Arduino or guest-driver hooks. I2C0 sends a 41-byte
register write to a host-attached device, then performs a repeated-START
40-byte read through the controller's interrupt/FIFO path. It verifies every
returned byte and an unattached-address NACK (`ESP_ERR_NOT_FOUND`). Two
interpreter replays have identical UART and unsupported-MMIO digests, with
no unsupported I2C controller accesses. The other 72 accesses remain
diagnostic: 70 startup accesses plus the SDA/SCL GPIO-matrix output routes
for signals 90/89. The MMIO transaction works, but Flexe does not claim
to emit their electrical pin waveforms. The pinned image SHA-256 is
`10934e17ec7ca440c4d81689225373b7fc1809b898700a5a8b813ab9a02c1ccc`
and the matching ELF SHA-256 is
`5ef3ea1fb67a20e5748d98124c791197a9404cac0b6dd953a28b441e3d3fd734`;
both matched across two clean build directories using ESP-IDF commit
`9d7f2d69f50d1288526d4f1027108e314e8c879f`:

```sh
idf.py -C tests/fixtures/s3_idf_i2c_master \
  -B /tmp/flexe-s3-idf-i2c-master-build \
  -D SDKCONFIG=/tmp/flexe-s3-idf-i2c-master-sdkconfig \
  -D IDF_TARGET=esp32s3 build
cmake --build build --target flexe-s3-idf-i2c-master-test
S3_IDF_I2C_MASTER_BIN=/tmp/flexe-s3-idf-i2c-master-build/s3_idf_i2c_master.bin \
S3_IDF_I2C_MASTER_ELF=/tmp/flexe-s3-idf-i2c-master-build/s3_idf_i2c_master.elf \
S3_ROM_ELF=/path/to/esp32s3_rev0_rom.elf \
  scripts/check-s3-idf-i2c-master.sh
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
Two complete interpreter runs replay byte-for-byte with no unsupported I2C
MMIO sites. Clock stretching, overflow, electrical bus timing, and other
slave-driver implementations remain unverified:

```sh
arduino-cli compile --fqbn esp32:esp32:esp32s3 \
  --build-path /tmp/flexe-s3-i2c-slave-build \
  --build-property compiler.optimization_flags=-Os tests/fixtures/i2c_slave
S3_I2C_SLAVE_BIN=/tmp/flexe-s3-i2c-slave-build/i2c_slave.ino.merged.bin \
S3_I2C_SLAVE_ELF=/tmp/flexe-s3-i2c-slave-build/i2c_slave.ino.elf \
S3_ROM_ELF=/path/to/esp32s3_rev0_rom.elf \
RUNNER=./build/flexe-i2c-slave-test ./scripts/check-s3-i2c-slave.sh
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
also covers per-host probe, device, and select callbacks. Two interpreter
replays match byte-for-byte with no unsupported GP-SPI sites; 350 unrelated
startup accesses remain across the two boots. The reset is harness-requested,
not a guest `esp_restart()`. This does not validate SPI slave mode, segmented
transfer, exact bus timing, or physical pin levels:

```sh
arduino-cli compile --fqbn esp32:esp32:esp32s3 \
  --build-path /tmp/flexe-s3-spi-master-build \
  --build-property compiler.optimization_flags=-Os tests/fixtures/spi_master
S3_SPI_BIN=/tmp/flexe-s3-spi-master-build/spi_master.ino.merged.bin \
S3_SPI_ELF=/tmp/flexe-s3-spi-master-build/spi_master.ino.elf \
S3_ROM_ELF=/path/to/esp32s3_rev0_rom.elf \
RUNNER=./build/flexe-spi-master-test ./scripts/check-s3-spi-master.sh
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
interpreter runs have identical guest and MMIO reports. The remaining RMT
diagnostics are one RX demodulation configuration (exact silicon
phase/frequency tolerance is uncalibrated) and two `RMT_SYS_CONF` writes
powering down pulse RAM during driver teardown; Flexe does not yet model
whether contents survive that power transition. Rebuild and replay with the
official ROM ELF:

```sh
idf.py -C tests/fixtures/s3_idf_rmt_loopback \
  -B /tmp/flexe-s3-idf-rmt-loopback-final \
  -D SDKCONFIG=/tmp/flexe-s3-idf-rmt-loopback-final-sdkconfig \
  -D IDF_TARGET=esp32s3 build
S3_IDF_RMT_LOOPBACK_BIN=/tmp/flexe-s3-idf-rmt-loopback-final/s3_idf_rmt_loopback.bin \
S3_IDF_RMT_LOOPBACK_ELF=/tmp/flexe-s3-idf-rmt-loopback-final/s3_idf_rmt_loopback.elf \
S3_ROM_ELF=/path/to/esp32s3_rev0_rom.elf \
  ./scripts/check-s3-idf-rmt-loopback.sh
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
channel-clock thresholds in the
[S3 technical reference manual](https://documentation.espressif.com/esp32-s3_technical_reference_manual_en.pdf).
The stock RX fixture arms a three-channel-tick filter minimum, rejects a
one-tick GPIO4 glitch, and then uses Arduino's `rmtSetCarrier()` to demodulate
a 38 kHz-style carrier with 25 kHz tolerance into a two-symbol frame. Unit
tests cover both carrier polarities. Two pinned interpreter replays match
byte-for-byte with one expected demodulation MMIO diagnostic and no other
unsupported RMT sites. This is a functional
envelope model that uses the opposite-polarity gap threshold; same-polarity
carrier duty discrimination and exact demodulation phase/frequency tolerance
are not calibrated against physical S3 silicon. Host-decoded injection still
bypasses GPIO, filtering,
and demodulation. DMA and overrun/error behavior when the guest fails to
service a threshold in time remain unsupported. Markerless loops, counted
loops without auto-stop, and dynamic sync-group changes remain unsupported
and diagnostic. Odd final
half-symbol encoding and GPIO route changes mid-frame also lack hardware
validation. Recheck the RX path with the compiled fixture
and official ROM ELF:

```sh
arduino-cli compile --fqbn esp32:esp32:esp32s3 \
  --build-path /tmp/flexe-s3-rmt-rx-build tests/fixtures/s3_rmt_rx
S3_RMT_RX_BIN=/tmp/flexe-s3-rmt-rx-build/s3_rmt_rx.ino.merged.bin \
S3_RMT_RX_ELF=/tmp/flexe-s3-rmt-rx-build/s3_rmt_rx.ino.elf \
S3_ROM_ELF=/path/to/esp32s3_rev0_rom.elf \
RUNNER=./build/flexe-s3-rmt-rx-test ./scripts/check-s3-rmt-rx.sh
```

The RX fixture's pinned merged image SHA-256 is
`27c4a14c42ac478cd6bfbaf509f1068ffb15d7371c6626c148a088772eee41cd`
and its matching ELF SHA-256 is
`0eedeb00430d938805517f0ff008147f890111dfcbc3b67e6b2b18af8193f81f`.
For the WLED 16.0.1 S3 4M QSPI image (SHA-256
`eb54c6c3648b7037d54df9f21fe02c9d9606b871faea04ce08b5f6f77dc79c81`),
4 billion aggregate cycles produced 317 completed RMT transmissions and
321,304 pulse words in 13,599 chunks on channel 0. The interpreter and JIT
match at every completed-frame boundary and finish with the same `46F65AC5`
pulse-stream digest, CPU state, and firmware-visible time; 23 unsupported
peripheral accesses remain across 21 attributed sites. The JIT executes
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
after modeling the SYSCON on-chip-memory policy. DIG_PWC,
PWC, REG's force pairs, the modeled domain fields of DIG_ISO, CLK_CONF's fast
selector, DATE, MEM_PD_MASK, and BT_LPCK_DIV no longer appear in the inventory.
Remaining accesses stay visible: RTC analog
and pad-isolation configuration in `0x60008000`, SYSTEM/PCR and SENSITIVE
cache/SRAM-allocation setup in `0x600C0000`, plus two low-count
startup writes at `0x600CE0D8` and `0x600CE0DC`. Flexe does not turn those
accesses into generic readback merely to reach zero diagnostics.

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
1450, the interpreter emitted five matching pairs within two billion
aggregate cycles. Its merged image SHA-256 is
`5178a6d97110c3682664ecbe602e0bbce40367c639ebc46ab3f714d879c4ec34`
and matching ELF SHA-256 is
`7c156185527896c78a8cf8de1155b10f417350ac1dad27a7ef255298fdad99d0`.
The APB ADC2 arbiter retains its reset priorities and software configuration.
When grant is forced, only a forced RTC grant completes an RTC ADC2 conversion;
the SENS RTC_FORCE bit bypasses that arbiter. With no modeled competing
requester, unforced RTC conversion proceeds. Digital and Wi-Fi/PWDET
requesters and simultaneous contention are still unsupported, and forcing
those owners remains diagnostic. The gate keeps 145 other unsupported accesses
visible, including RF power-detector trim and RTC setup. This is raw-code
functional behavior, not physical analog, attenuation, calibration accuracy, or
continuous/DMA ADC fidelity. Recheck with the external compiled fixture:

```sh
arduino-cli compile --fqbn esp32:esp32:esp32s3 \
  --build-path /tmp/flexe-s3-adc-fixture-build \
  --build-property compiler.optimization_flags=-Os tests/fixtures/s3_adc
S3_ADC_BIN=/tmp/flexe-s3-adc-fixture-build/s3_adc.ino.merged.bin \
S3_ADC_ELF=/tmp/flexe-s3-adc-fixture-build/s3_adc.ino.elf \
S3_ROM_ELF=/path/to/esp32s3_rev0_rom.elf \
  ./scripts/check-s3-adc.sh
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
`72e3198ed282afaf1cffb6c478b715ec8d7c6d40965e7a90df4fd5c0f838e60e`,
with ELF SHA-256
`47b844193180ca6e8323157d33eeff5dc72f6ec38896589c1bef938006d23e83`.
`scripts/check-s3-ledc.sh` requires byte-identical event replay and no LEDC
MMIO fallback, while 132 unrelated unsupported accesses stay visible:

```sh
arduino-cli compile --fqbn esp32:esp32:esp32s3 \
  --build-path /tmp/flexe-s3-ledc-fixture-build \
  --build-property compiler.optimization_flags=-Os tests/fixtures/s3_ledc
S3_LEDC_BIN=/tmp/flexe-s3-ledc-fixture-build/s3_ledc.ino.merged.bin \
S3_LEDC_ELF=/tmp/flexe-s3-ledc-fixture-build/s3_ledc.ino.elf \
S3_ROM_ELF=/path/to/esp32s3_rev0_rom.elf \
  ./scripts/check-s3-ledc.sh
```

The optional AP Memory 8 MiB OPI profile is selected by the board, not
inferred from firmware: `--psram ap-8m-opi` attaches it on CS1. The pinned
stock Arduino-ESP32 3.3.11 fixture checks 8 KiB of allocated external RAM
repeatedly; its second replay must be byte-identical, and the same image
without the flag must report no PSRAM. Build and run it with:

```sh
arduino-cli compile \
  --fqbn 'esp32:esp32:esp32s3:FlashSize=8M,PSRAM=opi' \
  --build-path /tmp/flexe-s3-psram-opi-build \
  --build-property compiler.optimization_flags=-Os \
  tests/fixtures/s3_psram_opi
S3_PSRAM_BIN=/tmp/flexe-s3-psram-opi-build/s3_psram_opi.ino.merged.bin \
S3_PSRAM_ELF=/tmp/flexe-s3-psram-opi-build/s3_psram_opi.ino.elf \
S3_ROM_ELF=/path/to/esp32s3_rev0_rom.elf \
  ./scripts/check-s3-psram-opi.sh
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
unsupported-access ceiling is 88 after the shared internal analog/private-PHY,
modem clock/reset, and RTC digital-domain models described below. The GPS probe
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
The accepted run retains an upper bound of 88 unsupported accesses so the
interaction cannot hide a register-model regression.

This is a controller-bootstrap compatibility boundary, not a claim that
Bluetooth packets, scanning, coexistence timing, RF propagation, or the
private analog PHY are complete. Unsupported-access diagnostics remain
visible during the accepted boot instead of being converted into fake radio
success.

For the NerdMiner v1.8.3 S3 factory image (SHA-256
`8dd4bad43944def2287cf8b6bed7762c1881b6e7f04f7bd1556ad555202f8c22`),
an earlier interpreter audit, before RTC power-sequencer support, reported
6,919 unsupported accesses at 4 billion aggregate cycles. GigaDevice `0x5A`
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
the new boot loads the saved pool and wallet from SPIFFS. Unsupported-access
diagnostics must remain visible:

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
`9b28227195225f9ecf9b59edc5d9e6d962eb561a069a6b8d5501c41627dec353`,
ELF SHA-256
`2dbbafa1ac2915298c40918e76bb3ff0afb0dc63f58c23eb332bbf5ffbfef881`).
Without the VFS registration symbols or for a range beyond Flexe's current
64-FD `select()` layout, the bridge does not guess a base and reports the
unsupported condition. Run the external-image gate with:

```sh
S3_IDF_SOCKET_BIN=/path/to/s3_idf_socket_range.bin \
S3_IDF_SOCKET_ELF=/path/to/s3_idf_socket_range.elf \
S3_ROM_ELF=/path/to/esp32s3_rev0_rom.elf \
  ./scripts/check-s3-idf-socket-range.sh
```

The
[default ESP-IDF RTC-WDT handoff](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-reference/system/wdts.html)
is represented, but images built to retain that watchdog into user code need
a separate boot-configuration path. Unsupported PHY diagnostics remain a
blocker for real radio behavior.
