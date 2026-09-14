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
| S3 LX7, interrupts, dual-core startup | Partial | `scripts/check-s3-idf-hello.sh`, `scripts/check-s3-idf-crosscore.sh`, target and interrupt-matrix unit tests | Sustained queue handoffs and CPU1 stall/resume pass; broader FreeRTOS and interrupt workloads remain to validate. |
| S3 ROM, image, flash/MMU/partitions | Partial | `tests/test_loader.c`, `tests/test_spi_mem.c`, `scripts/check-s3-nerdminer-portal.sh` | Other flash modes, cache behavior, and bootloader paths remain unverified. |
| S3 NVS, SPIFFS, reset persistence | Partial | `scripts/check-s3-idf-nvs.sh`, NerdMiner POST/save/restart/reload gate, `scripts/check-s3-idf-sleep.sh` | RTC slow counter/STORE and RTC_DATA survive machine rebuild; additional partition and filesystem variants need end-to-end gates. |
| S3 GPIO/RTCIO/IO_MUX | Partial (MMIO) | `tests/test_gpio.c`, `tests/test_rtc_io.c`, `tests/test_rtc_cntl.c`, stock Arduino ADC gate, native EXT0/EXT1 wake gate | Host-driven RTC GPIO wake and per-pad/global RTC/digital hold work; physical pulls, drive strength, and electrical levels are not complete. |
| S3 UART/USB console | Partial (MMIO and host I/O) | Official ESP-IDF and Arduino UART output gates; `tests/test_usb_serial_jtag.c` | USB protocol/electrical behavior and all UART DMA modes are not claimed. |
| S3 I2C, GP-SPI | Partial (MMIO) | `tests/test_peripherals.c`, `tests/test_system_clock.c`, `tests/test_spi_mem.c`; stock Arduino Wire, I2C-slave, and ESP-IDF SPI-master replay gates | More I2C guest-driver/device combinations, slave overflow/clock stretching, and GP-SPI segmented/slave modes remain. |
| S3 GDMA | Partial (MMIO) | `tests/test_crypto.c` checks chained TX/RX descriptors, ownership/writeback and errors; `tests/test_peripherals.c` exercises GP-SPI full-duplex GDMA | Full priority and peripheral interactions remain unverified. |
| S3 timers, watchdogs, RTC | Partial (MMIO) | `tests/test_systimer.c`, `tests/test_timer_group.c`, `tests/test_rtc_cntl.c`, ESP-IDF cross-core, restart, native timer and GPIO light/deep-sleep gates | Timer and EXT0/EXT1 wake work; brownout configuration reads back under a nominal fixed supply, but voltage detection/reset, touch/ULP wake, power-transition fidelity, calibrated timing, and other reset causes remain unsupported. |
| S3 LEDC PWM | Partial (MMIO) | `tests/test_ledc_v1.c`, `scripts/check-s3-ledc.sh`: stock Arduino repeatedly drives GPIO4 at 5 kHz with four readback duties and byte-identical replay | Aggregate PWM output and timed fade/interrupt are modeled; individual electrical edges and overflow-counter behavior are not. |
| S3 RMT TX | Partial (MMIO) | `tests/test_rmt_v1.c`, `scripts/check-s3-wled-rmt.sh`; WLED 16.0.1 emits sustained pulse chunks | Counted loops, synchronized TX, fine status, and output-pad waveform validation remain. |
| S3 RMT RX | Partial (MMIO, GPIO input, host symbols) | `tests/test_rmt_v1.c`, `scripts/check-s3-rmt-rx.sh`; stock Arduino-ESP32 3.3.11 decodes a GPIO4 waveform and a 96-symbol host frame through the ESP-IDF ISR without RMT fallback | GPIO edges are measured in guest time, but glitch filtering, carrier demodulation, DMA, odd pulse tails, and overrun behavior under delayed ISR service remain unverified or unsupported. |
| S3 RTC SAR ADC | Partial (MMIO plus host samples) | `tests/test_sens.c`, `tests/test_apb_saradc.c`, `scripts/check-s3-adc.sh` | Digital/DMA conversion, ULP, contention, and physical calibration remain unsupported. |
| S3 network-facing workflow | Partial (service shim) | NerdMiner BSD-socket portal and WLED native lwIP/Ethernet UI and JSON state gates | Wi-Fi RF/PHY, association realism, and general transport modes are unsupported. |
| S3 Bluetooth baseband clock | Partial (MMIO) | `tests/test_radio.c` checks half-slot/subslot phase against guest time | Controller scheduler, packets, and RF are unsupported; Marauder still asserts before its CLI. |
| Cycle/cache/electrical/RF fidelity | Unsupported | Outside this functional milestone | Requires calibrated hardware traces and declared tolerances. |

S3 remains experimental. The NerdMiner filesystem result is a meaningful
end-to-end flash-format/mount check. With the matching application ELF, the
host-backed socket/select boundary now lets the unmodified firmware serve
`GET /wifi` as `200 OK` with its 4,985-byte configuration HTML. This is a
service shim, not a modeled Wi-Fi radio: the page reports no networks, the
roughly 7,000 unsupported accesses (mostly RF/PHY) remain visible. The S3
application handoff now clears the power-on flash-boot watchdog mode skipped
with the second-stage bootloader. Restoring ROM-owned BSS and interface state
from the official ROM ELF during a software restart lets NerdMiner initialize
again and reload its saved settings instead of panicking during PSRAM setup,
while physical SRAM outside those sections remains intact for app `.noinit`.
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
including the unsupported-MMIO report; hello-world reports 224 unsupported
accesses across its two boots and cross-core reports 104 in one boot. This
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
sleep state, timer interrupt, digital-wrap power-down selection, and separate
wake-cause register. The session advances the shared virtual clock through
timer-only light sleep or resets after timer-only deep sleep; the always-on
counter, fractional tick phase, and STORE0-7 survive that rebuild. The
unmodified ESP-IDF 5.3.2 `tests/fixtures/s3_idf_sleep` application enters a
50 ms light sleep, observes `ESP_SLEEP_WAKEUP_TIMER`, then enters a 20 ms deep
sleep and checks the timer wake cause, deep-sleep reset reason, RTC_DATA marker,
and STORE0 marker on its second `app_main()`. The guest also measures RTC
ticks through each sleep, including the second boot's startup, and sustains a
FreeRTOS heartbeat afterward. Its image SHA-256 is
`cce3abfb4191663a0c6125180e0ea91804bf8f71387aeeff807e9b40bb2175a2`
and matching ELF SHA-256 is
`a7cbc0ff14284c65573b2ca075bfea0a85a28707c1df35c131d39fd39bb0e294`.
The two complete replays are byte-identical; 288 other unsupported accesses
across both boots remain visible, mainly RTC power/isolation configuration.
No unsupported sites remain for S3 sleep timer, state, wake-enable, or cause
registers. The nominal slow-clock model yielded about 48.9/19.3 ms for the
guest's requested 50/20 ms, so this is functional wake ordering, not calibrated
sleep duration. Touch/ULP wake and voltage/power-domain transitions are not
modeled; an unarmed timer or those sources do not synthesize a wake.
The S3 `RTC_CNTL_BROWN_OUT_REG` now retains documented configuration fields,
resets to the specified defaults, and treats counter-clear as a write-only
strobe. Its detector remains clear under Flexe's fixed nominal supply; analog
thresholds, voltage injection, brownout interrupts, and resets are not modeled.
`RTC_CNTL_PWC_REG`'s global RTC pad-force-hold bit freezes all 22 RTC-capable
pads through the same physical hold model as individual bits. ROM boot clears
the global source, while individually held pads retain their state across a
deep-sleep rebuild. Releasing global hold does not release an individual pad.
The digital hold register maps bits 1–27 to GPIO22–48; GPIO22–25 are unbonded,
so GPIO26 is its first usable pad, while GPIO21 uses the RTC hold register.
`RTC_CNTL_DIG_ISO_REG`'s global digital force-hold freezes bonded GPIO26–48;
its force-unhold bit releases that source without clearing individual holds.
ROM boot clears global digital hold, but individually held pads keep their
latched state. Digital autohold and non-pad isolation effects remain diagnostic.
Other RTC power/isolation fields read back but their power
effects remain diagnostic and are not treated as implemented.
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
The gate reports 288 unrelated unsupported accesses across both boots, mainly
RTC power/isolation setup; the EXT selection/state/status sites are modeled.
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
The gate checks two byte-identical runs and still reports 224 unsupported
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
([Espressif signal map](https://github.com/espressif/esp-idf/blob/master/components/soc/esp32s3/include/soc/gpio_sig_map.h));
the model measures edge intervals on the guest clock. Two pinned interpreter
replays match byte-for-byte with no unsupported RMT MMIO sites. Host-decoded
injection still bypasses GPIO, and glitch filtering, carrier demodulation,
DMA, and overrun/error behavior when the guest fails to service a threshold
in time remain unsupported. Filter-enabled RX is diagnosed rather than
decoded as unfiltered edges. Counted TX loops, synchronized TX, and exact
waveform-to-GPIO output routing also remain unsupported; those paths retain
diagnostics. Odd final half-symbol encoding and GPIO route changes mid-frame
also lack hardware validation. Recheck the RX path with the compiled fixture
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
`471d8386375c01638c748b5f1da1899eb73747309a2582ac19b65dda812482ef`
and its matching ELF SHA-256 is
`5cee94043bbd57b3dee70a0bbb6461ac8b1a5a6f743f10c8059af237e32b78f2`.
For the WLED 16.0.1 S3 4M QSPI image (SHA-256
`eb54c6c3648b7037d54df9f21fe02c9d9606b871faea04ce08b5f6f77dc79c81`),
4 billion aggregate cycles produced 317 completed RMT transmissions and
321,448 pulse words on channel 0. Repeated interpreter runs yielded 13,605
chunks and the `30EAB266` pulse-stream digest; 7,070 unsupported peripheral
accesses remain. S3 JIT is not enabled, so no JIT parity is claimed. This
establishes sustained hardware-output progress, not
correct colors on a physical LED strip. Recheck
with the external image and ROM ELF:

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

The S3 Bluetooth register window now includes a captured baseband clock:
requesting a latch publishes a half-slot count and a down-counting subslot
phase from shared guest time. Its register protocol and 312.5 µs half-slot
interpretation are inferred from `r_rwip_time_get` in the official
`esp32s3_rev0_rom.elf`, not from a public Bluetooth-controller register
specification. This removes Marauder's unbounded poll of the latch command,
but does not make its Bluetooth controller functional. The unmodified Marauder
v1.12.1 multiboard S3 image (SHA-256
`62292a502635f02eab9e515bc3f18fbf43330f81abe2c006c1afe43bd21b57ea`)
then reports `assert lld.c 292` and reboots on an interrupt watchdog before
its CLI. The [official v1.16.0 MultiBoard S3 release](https://github.com/justcallmekoko/ESP32Marauder/releases/tag/v1.16.0)
(SHA-256 `b6b61e6c6c41bc78422d405d14117ab5aa6ec0cdee751327ea568232e52dd0be`)
still reaches the same assertion. A 2-billion-cycle interpreter run of that
image reports 20,879 unsupported accesses across 318 address/PC/core/
direction sites because the audit survives each of its three software resets;
the hottest sites are RF/PHY reads in `0x6000E000`. Neither image is an
accepted interactive S3 scenario. The controller scheduler and RF/packet
behavior remain unsupported, and the assertion is not suppressed.

```sh
./build/xtensa-emu -N -q --no-jit --target esp32s3 \
  -R "$FLEXE_S3_ROM_ELF" --unhandled-report -c 2000000000 \
  "$FLEXE_S3_MARAUDER_BIN"
```

For the NerdMiner v1.8.3 S3 factory image (SHA-256
`8dd4bad43944def2287cf8b6bed7762c1881b6e7f04f7bd1556ad555202f8c22`),
the following interpreter audit at 4 billion aggregate cycles reports 6,936
unsupported accesses at 291 distinct address/PC/core/direction sites. Of
these, 6,728 are in the `0x6000E000` RF/PHY window; the hottest named callers
include `wr_rf_freq_mem`, `set_chan_freq_sw_start`, and `bt_txpwr_freq`.
That concentration identifies a network-controller boundary, not evidence
that the reads and writes are harmless or that a zero-returning PHY model is
correct. The remaining inventory includes documented SYSCON, RTC, and sensor
registers. The `--unhandled-report` inventory and total now accumulate across
firmware-requested resets; neither this output nor the working network
scenario proves real RF behavior. Repeat the measurement with the matching
application and ROM ELFs:

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

The socket descriptor base `48` is this Arduino build's ESP-IDF
configuration, not a universal S3 constant; other SDK configurations need
discovery before this bridge can claim support. The
[default ESP-IDF RTC-WDT handoff](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-reference/system/wdts.html)
is represented, but images built to retain that watchdog into user code need
a separate boot-configuration path. Unsupported PHY diagnostics remain a
blocker for real radio behavior.
