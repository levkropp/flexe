# Firmware compatibility

Flexe treats production compatibility as an end-to-end property. A firmware
image must keep executing useful work, interact through modeled hardware or
service boundaries, and produce equivalent results in the interpreter and JIT.
Booting to one UART line is not considered a pass.

## Current corpus

| Firmware | Engines | Result | Remaining work |
|---|---|---|---|
| ESP32 Marauder 1.14.3 (CYD) | Interpreter + JIT | Pass | None in the scripted scenario |
| ESP32 Marauder 1.15.1 (CYD 2432S028) | Interpreter + JIT | Pass | None in the scripted scenario |
| ESP32 Marauder 1.14.3 (3.5-inch and Guition variants) | Interpreter + JIT | Pass | None in the scripted scenario |
| NerdMiner 1.8.3 | Interpreter + JIT | Pass | None in the scripted scenario |
| Meshtastic 2.7.26 (T-Beam) | Interpreter + JIT | Pass | Expand device-specific interaction coverage |
| openHASP 0.7.0-rc13 (Lanbon L8) | Interpreter + JIT | Known gap | Default runs differ in one heap-fragmentation value |
| Tasmota 15.6.0 | Interpreter + JIT | Known gap | Bad synthesized window fill after Berry `longjmp` |
| WLED 16.0.1 | Interpreter + JIT | Known gap | Borrowed frame spills, then `retw` restores address zero |
| Marauder 2432S028 2USB | Interpreter + JIT | Known gap | Early register-window spill runaway |

ROM images are not stored in this repository. Results are tied to the image
versions above and should be rechecked when a release changes.

## Curated CYD scenarios

`scripts/check-stock-roms.sh` drives unmodified Marauder and NerdMiner images
through board-level scenarios on both engines. It checks completion, modeled
I/O, and identical final framebuffers. Known official image hashes also have a
pinned framebuffer digest, so a display-model change is explicit.

The Marauder scenario covers:

- ILI9341 rendering and XPT2046 touch navigation
- UART0 commands and independent UART2 GPS traffic
- SD-card initialization and filesystem access
- Wi-Fi promiscuous receive and raw-frame transmit boundaries
- BLE scan callbacks and advertising payloads

The NerdMiner scenario covers:

- Wi-Fi association and ESP event delivery
- captive-portal HTTP and DNS traffic
- host-backed pool networking and the mining task path

Run the scenarios with:

```sh
MARAUDER_BIN=/path/to/marauder.bin \
NERDMINER_BIN=/path/to/nerdminer.bin \
./scripts/check-stock-roms.sh
```

Additional positional images use the Marauder profile and still require both
engines to agree.

## Arbitrary firmware corpus

`scripts/check-firmware.sh` applies a deliberately generic probe to every
`.bin` passed on the command line or found directly under `FLEXE_ROMS`. It
requires each engine to:

- reach the requested virtual-time budget without trapping or stopping early
- retire enough instructions to demonstrate useful execution
- avoid unhandled MMIO and unregistered ROM calls
- end on a fetchable guest PC
- produce the same normalized UART transcript digest

```sh
FLEXE_ROMS=/path/to/corpus ./scripts/check-firmware.sh
```

Known failures remain in the run and are reported as `KNOWN-BAD`. If one starts
passing, the script reports that its exception entry should be removed.

## Register-window blocker

Tasmota and WLED currently expose the same underlying weakness: a windowed
return restores a frame from a save-area base that no longer describes the
call chain. Tasmota reaches this after Berry changes control flow with
`longjmp`; WLED reaches it after a borrowed core frame spills inside the heap
allocator.

`FLEXE_WINDOW_VECTORS=1` executes the firmware's own spill/fill vectors instead
of synthesizing the operation. With shadow fills enabled (the default), that
path allows Tasmota to finish booting and removes openHASP's cross-engine
mismatch. It is not yet the default because Marauder then stops issuing the SD
filesystem writes required to create its `SCRIPTS` directory. Fixing that
interaction, then making the vector path universal, is the current correctness
priority.

## Firmware-specific hooks

Address-based Wi-Fi and Bluetooth hooks are accepted only for verified image
fingerprints. `scripts/locate-wifi-symbols.py` can derive candidate entry points
by matching masked function prologues against a throwaway build made with the
same Arduino core. `scripts/disasm-firmware.sh` disassembles stripped images at
their real load addresses for follow-up analysis.
