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

## Recorded baselines

Representative Apple-silicon release/PGO measurements from the current
development history:

| Workload | Interpreter | JIT | JIT vs 240 MHz |
|---|---:|---:|---:|
| TJpgDec JPEG decode | 330 MIPS | 693 MIPS | 2.9x |
| dual-core FreeRTOS `real_time_stats` | 334 MIPS | 499 MIPS | 2.1x |
| ALU/memory compute fixture | 318 MIPS | 2,476 MIPS | 10.3x |

The recorded x86-64 compute fixture reached 1,789 MIPS under the JIT and
187 MIPS in the interpreter. In scripted stock-ROM runs, x86-64 recorded
approximately 12.8x real time for Marauder and 3.8x for NerdMiner. Host CPU,
compiler, thermal state, scenario, and build options all matter; rerun the
committed scripts instead of treating these figures as guarantees.

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

Recent production runs recorded:

| Image | JIT coverage | Guest instructions per JIT entry |
|---|---:|---:|
| Marauder 1.14.3 CYD | 49.5% | 9.1 |
| Marauder 1.15.1 CYD 2432S028 | 46.6% | 9.8 |
| Marauder 1.14.3 3.5-inch | 45.9% | 10.4 |
| Marauder 1.14.3 Guition | 42.3% | 11.1 |
| NerdMiner 1.8.3 | 22.5% | 25.1 |

This is why optimizing only emitted native instructions has a limited effect
on mixed firmware. Improving block eligibility, safe chaining, service and
peripheral fast paths, and the interpreter all affect end-to-end performance.

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
