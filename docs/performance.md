# Performance

Flexe reports elapsed emulated time separately from retired guest
instructions. Keep those measurements separate when evaluating a workload.

## Metrics

- **Real-time factor** is simulated ESP32 seconds divided by host wall seconds.
  `1.0x` keeps pace with a 240 MHz ESP32.
- **Retired MIPS** counts guest instructions actually executed per host second.
  It excludes time advanced while both cores are halted or the scheduler jumps
  directly to a timer deadline.
- **JIT coverage** is the fraction of retired instructions executed by compiled
  blocks, not the fraction of virtual cycles.

Firmware that spins, traps, or idles prematurely can show a flattering
real-time factor while doing no useful work. The benchmark scripts therefore
print retired instructions and UART activity next to elapsed time.

## Current production baseline

This snapshot was recorded on 2026-09-07 on an Apple A18 Pro MacBook with
Apple Clang 21. The build used `Release`, LTO, and `NATIVE_ARCH=ON`; PGO was
off. Each result is the fastest of five 600-million-cycle runs with a
10,000-instruction scheduling batch and a zero-unmapped-access limit:

```sh
FLEXE_ROMS=/path/to/corpus MAX_UNMAPPED=0 \
  CYCLES=600000000 REPS=5 ./scripts/bench-firmware.sh
```

| Firmware | Interpreter | JIT | Retired instructions | UART bytes |
|---|---:|---:|---:|---:|
| Meshtastic 2.7.26 T-Beam | 13.15x | 19.03x | 31,615,162 | 11,562 |
| NerdMiner 1.8.3 | 5.73x | 8.70x | 90,678,580 | 578 |
| openHASP 0.7.0-rc13 | 3.27x | 6.15x | 152,399,237 | 6,411 |
| Tasmota 15.6.0 | 2.52x | 6.55x | 212,454,580 | 449 |
| WLED 16.0.1 | 0.92x | 1.64x | 472,823,757 | 5 |

Both engines passed the generic progress gate and produced the same UART digest
for every image. All five clear real time under the JIT; WLED is deliberately
kept in the corpus because it has the smallest margin. Host scheduling and
thermal state move these numbers, so rerun the command before comparing a
change.

Historical compute-focused PGO runs reached 693 MIPS for TJpgDec, 499 MIPS for
the dual-core FreeRTOS fixture, and 2,476 MIPS for the ALU/memory fixture on
Apple silicon. An x86-64 compute run reached 1,789 MIPS. These describe native
instruction throughput, not the production-firmware baseline above.

## Reproducible compute benchmark

`bench-compute.sh` builds an in-repository Arduino sketch and executes a fixed
number of rounds. It covers a dependent scalar chain, strided memory traffic,
SHA-256-shaped rotate/XOR work, and byte loads/stores. Both engines must return
the same checksum.

```sh
ARDUINO_CLI=/path/to/arduino-cli ./scripts/bench-compute.sh
```

Useful overrides are `BENCH_ROUNDS`, `BENCH_REPS`, and `MIN_REALTIME`.

## Production firmware

Use the scripted stock scenarios when the image has one:

```sh
MARAUDER_BIN=/path/to/marauder.bin \
NERDMINER_BIN=/path/to/nerdminer.bin \
./scripts/bench-stock-roms.sh
```

The default gate runs three two-billion-cycle samples and rejects runs that
stop early, trap, miss the real-time threshold, or spend most wall time blocked
in the host. Configure it with `CYCLES`, `REPS`, `ENGINE`, `MIN_REALTIME`, and
`ESP_HZ`.

For any other image:

```sh
./scripts/bench-firmware.sh /path/to/firmware.bin
FLEXE_ROMS=/path/to/corpus ./scripts/bench-firmware.sh
```

The benchmark refuses to report a speed unless every timed run passes the
generic progress gate and both engines produce the same UART digest. `BATCH`
and `MAX_UNMAPPED` override its 10,000-instruction scheduling quantum and
1,000-access unmapped-memory ceiling.

## JIT coverage

The WLED run above executes 90.1% of retired instructions in native blocks,
averaging 10.8 guest instructions per JIT entry. Coverage is workload-specific
and is not a speed score: compiling standalone one-instruction blocks raised
WLED coverage to 92.2% but slowed median wall time by about 1.8%. Flexe therefore
compiles hot standalone blocks from two instructions upward, while still
allowing shorter targets when an existing native chain removes their dispatch
cost.

## Profiling

Build the sampling profiler separately so its dispatch-loop layout does not
affect normal measurements:

```sh
cmake -S . -B build-prof -DFLEXE_PROFILE=ON
cmake --build build-prof -j
FLEXE_PROFILE=1 ./build-prof/flexe-generic-rom-test firmware.bin
```

`FLEXE_PROFILE_SP=0x...` restricts samples to a task stack, and
`FLEXE_JIT_STATS=1` enables JIT counters in compatible runners. Profile before
changing hot code: even compiled-out instrumentation can move the dispatch
loop enough to change results.
