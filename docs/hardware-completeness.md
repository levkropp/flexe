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

## Current evidence and next gates

| Area | Current evidence | Next gate |
|---|---|---|
| Classic ESP32 production corpus | [Compatibility scenarios](compatibility.md#curated-cyd-scenarios) and `scripts/check-stock-roms.sh` | Expand uncovered interactive device/network paths without losing WLED fast-mode throughput. |
| S3 image, ROM, and flash | `tests/test_loader.c`, `tests/test_spi_mem.c`, `scripts/check-s3-nerdminer-portal.sh`; NerdMiner 1.8.3 mounts SPIFFS, serves its configuration page, saves submitted settings, restarts, and reloads the same JSON from guest-written flash | Extend storage/peripheral coverage beyond this workflow. |
| S3 CPU, dual-core, and basic devices | [Target notes](compatibility.md#target-selection), `scripts/check-s3-idf-hello.sh` (official ESP-IDF countdown/restart), `scripts/check-s3-idf-crosscore.sh` (32 CPU1/CPU0 queue-notification handoffs and sustained output), and target-specific unit/Arduino fixtures; `--unhandled-report` ranks unsupported MMIO by call site | Extend cross-core and Arduino peripheral workflows and work down the measured unhandled-access inventory. |
| S3 RTC GPIO/RTCIO (partial) | `tests/test_rtc_io.c` covers GPIO0..21 RTC pad ownership, output/enable W1TS/W1TC, host input, unknown alternate functions, and return to digital GPIO. `tests/test_rtc_cntl.c` covers both pad-hold sources, frozen output/mux/input state, GPIO21 overlap, and reset rebuild. The stock Arduino ADC gate checks that `rtc_gpio_deinit()` for GPIO4/11 no longer falls back. | RTC wakeup/interrupt routing, digital IO_MUX function hold, pad pulls, drive strength, analog/electrical resolution, and deep-sleep wake sequencing are unsupported. |
| S3 network service (partial) | NerdMiner's BSD-socket portal completes a host-backed request/response session; a separately built WLED 16.0.1 serves its UI and accepts/readbacks a JSON LED-state change through native raw lwIP and the optional Ethernet user-mode backend | Exercise more network modes and production images; RF/PHY remains unsupported. |
| S3 RMT TX | `tests/test_rmt_v1.c` and `scripts/check-s3-wled-rmt.sh`; unmodified WLED 16.0.1 transmits sustained LED pulse chunks with a pinned interpreter output digest | Model RX, counted loops, synchronized TX and finer channel status; verify actual LED protocol and GPIO routing. |
| S3 RTC SAR ADC (partial) | `tests/test_sens.c` covers both units' pad selection, START/DONE/DATA latching, internal-ground calibration selection, reader inversion, clock/reset gating, and host ADC stimulus through `adc_in` (channels 0–9 = ADC1, 10–19 = ADC2). `tests/test_apb_saradc.c` covers ADC2 forced-grant and RTC bypass; `tests/test_rtc_cntl.c` covers SAR-I2C power gating. `scripts/check-s3-adc.sh` checks five sustained stock Arduino `analogRead()` pairs. | Digital/DMA conversion, ULP, ADC interrupts, competing-requester arbitration, and calibrated analog voltage/electrical behavior are unsupported. |
| S3 Bluetooth baseband clock (partial) | `tests/test_radio.c` checks the captured half-slot count and subslot phase against both cores' guest time | The controller scheduler, packet exchange, and RF path are unsupported; Marauder still asserts before its CLI. |
| Timed/cycle/electrical/RF fidelity | Not accepted by this functional milestone | Track separately with calibrated hardware traces and declared tolerances. |

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
`app_main()` on CPU0 before sustained 100 ms heartbeats. Its image SHA-256 is
`c38d4cf3a51d05885c263fa2c1fe88deaa13dbde2beb357562622433254772a1`
and ELF SHA-256 is
`9b2a6d8d0451e833f2050bd75534a10872d3046bc2eaeb369b3ec43dad5fcfc6`.
Both external-image gates compare a complete second replay byte-for-byte,
including the unsupported-MMIO report; each still reports 167 unsupported
accesses after boot. This proves these FreeRTOS interactions, not simultaneous
core execution or timing fidelity. Run with matching external artifacts:

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

The S3 RMT V1 TX model handles direct pulse RAM, per-channel dividers,
threshold refill interrupts, end/error interrupts, and pulse-timed
transmission. It is a partial device model: RX, counted loops, synchronized
TX, DMA, and exact waveform-to-GPIO routing are not yet supported. Unsupported
paths retain MMIO diagnostics. For the WLED 16.0.1 S3 4M QSPI image (SHA-256
`eb54c6c3648b7037d54df9f21fe02c9d9606b871faea04ce08b5f6f77dc79c81`),
4 billion aggregate cycles produced 317 completed RMT transmissions and
321,448 pulse words on channel 0. Repeated interpreter runs yielded 13,605
chunks and the `30EAB266` pulse-stream digest; 7,197 unsupported peripheral
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
those owners remains diagnostic. The gate keeps 222 other unsupported accesses
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
its CLI. It is not an accepted second interactive S3 scenario; the controller
scheduler and RF/packet behavior remain unsupported.

For the NerdMiner v1.8.3 S3 factory image (SHA-256
`8dd4bad43944def2287cf8b6bed7762c1881b6e7f04f7bd1556ad555202f8c22`),
the following interpreter audit at 4 billion aggregate cycles reports 7,065
unsupported accesses at 361 distinct address/PC/core/direction sites. Of
these, 6,728 are in the `0x6000E000` RF/PHY window; the hottest named callers
include `wr_rf_freq_mem`, `set_chan_freq_sw_start`, and `bt_txpwr_freq`.
That concentration identifies a network-controller boundary, not evidence
that the reads and writes are harmless or that a zero-returning PHY model is
correct. The remaining inventory includes documented SYSCON, RTC, and sensor
registers. Repeat the measurement with the matching application and ROM ELFs:

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
