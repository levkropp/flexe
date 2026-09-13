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
| S3 image, ROM, and flash | `tests/test_loader.c`, `tests/test_spi_mem.c`; NerdMiner 1.8.3 factory image reaches a mounted SPIFFS volume and configuration portal; loader tests preserve NOR across reset | Make a reproducible interactive S3 production gate and test guest filesystem reads/writes across reset. |
| S3 CPU, dual-core, and basic devices | [Target notes](compatibility.md#target-selection) and target-specific unit/ESP-IDF fixtures | Finish sustained FreeRTOS/Arduino fixtures and work down the measured unhandled-access inventory. |
| Timed/cycle/electrical/RF fidelity | Not accepted by this functional milestone | Track separately with calibrated hardware traces and declared tolerances. |

S3 remains experimental. The NerdMiner filesystem result is a meaningful
end-to-end flash-format/mount check, but its roughly 7,000 unhandled accesses
and untested portal requests prohibit a production-support claim. ROM images
and third-party firmware binaries are not copied into this repository.
