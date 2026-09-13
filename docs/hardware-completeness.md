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
| S3 RMT TX | `tests/test_rmt_v1.c` and `scripts/check-s3-wled-rmt.sh`; unmodified WLED 16.0.1 transmits sustained LED pulse chunks with a pinned interpreter output digest | Model RX, counted loops, synchronized TX and finer channel status; verify actual LED protocol and GPIO routing. |
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
