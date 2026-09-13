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
| S3 CPU, dual-core, and basic devices | [Target notes](compatibility.md#target-selection) and target-specific unit/ESP-IDF fixtures; `--unhandled-report` ranks unsupported MMIO by call site | Finish sustained FreeRTOS/Arduino fixtures and work down the measured unhandled-access inventory. |
| S3 network service (partial) | NerdMiner's BSD-socket web portal completes a host-backed request/response session; WLED reaches raw lwIP TCP listen; a symbol-resolved Ethernet netif boundary now exposes guest TX and registered RX without replacing its TCP stack | Attach a host virtual-network backend and exercise WLED's HTTP UI; RF/PHY remains unsupported. |
| S3 RMT TX | `tests/test_rmt_v1.c` and `scripts/check-s3-wled-rmt.sh`; unmodified WLED 16.0.1 transmits sustained LED pulse chunks with a pinned interpreter output digest | Model RX, counted loops, synchronized TX and finer channel status; verify actual LED protocol and GPIO routing. |
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
provides a second, independently sourced production output scenario, but not
a second interactive network workflow. The remaining RF/PHY gaps prohibit a
production-support claim. ROM images and third-party firmware binaries are
not copied into this repository.

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
correct colors on a physical LED strip or a functioning WLED web UI. Recheck
with the external image and ROM ELF:

```sh
S3_WLED_BIN=/path/to/WLED_16.0.1_ESP32-S3_4M_qspi.bin \
S3_ROM_ELF=/path/to/esp32s3_rev0_rom.elf \
  ./scripts/check-s3-wled-rmt.sh
```

A separate build of WLED v16.0.1 from its tagged source (`29b389d`, image
SHA-256 `8e165290df301b0bcea5763637db1f0ec9d4ad5b1d07b8588a1baa5ebd50bad5`)
with its matching ELF reaches `AsyncServer::begin()` and then
`tcp_listen_with_backlog()` at about 1.16 billion guest cycles. Its UDP
sockets reach the host bridge, but no host HTTP listener appears. WLED's
AsyncTCP web server uses lwIP's raw TCP PCB API, which the current S3
BSD-socket bridge does not expose. A general raw-TCP or virtual-network path
and an HTTP request/response gate are still needed. This source-built image
is not byte-identical to the pinned release image; its ELF must not be used
to symbolize that release binary.

The native S3 `esp_wifi_internal_tx`/`tx_by_ref` and
`esp_wifi_internal_reg_rxcb` symbols now form an optional Ethernet-frame
service boundary. A host backend can accept guest frames and queue frames
through the netif's registered RX callback; the RX buffer remains mapped
until the guest calls `esp_wifi_internal_free_rx_buffer`. Without an attached
backend, guest transmit and free functions execute unchanged. A synthetic
S3 ELF unit gate checks callback registration, transmit fallback, frame
delivery, buffer reuse, and statistics. This is transport plumbing, not a
working WLED HTTP server yet.

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
