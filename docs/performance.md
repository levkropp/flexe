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

This snapshot was recorded on 2026-09-08 on an Apple-silicon MacBook with
Apple Clang 21. The build used `Release`, LTO, and `NATIVE_ARCH=ON`; PGO was
off. The corpus was run sequentially so WLED includes sustained thermal load.
Each result is the fastest of five 600-million-cycle runs with a
10,000-instruction scheduling batch and a zero-unmapped-access limit:

```sh
FLEXE_ROMS=/path/to/corpus MAX_UNMAPPED=0 CYCLES=600000000 REPS=5 \
  ./scripts/bench-firmware.sh
```

| Firmware | Interpreter | JIT | Retired instructions | UART bytes |
|---|---:|---:|---:|---:|
| Meshtastic 2.7.26 T-Beam | 15.09x | 25.03x | 31,607,116 | 11,562 |
| NerdMiner 1.8.3 | 6.12x | 12.84x | 90,588,176 | 578 |
| openHASP 0.7.0-rc13 | 3.58x | 8.31x | 152,301,508 | 6,263 |
| Tasmota 15.6.0 | 2.85x | 8.29x | 210,912,982 | 449 |
| WLED 16.0.1 | 1.03x | 2.32x | 471,751,033 | 5 |

Both engines passed the generic progress gate and produced the same UART digest
for every image. All five clear real time in both engines; WLED is deliberately
kept in the corpus because it has the smallest margin. Host scheduling and
thermal state move these numbers, so rerun the command before comparing a
change.

The stricter repeated WLED acceptance gate was rerun on 2026-09-12 after
structural acceleration of the relocated ESP-IDF critical-section body and
its standard heap lock wrappers. With one warm-up and three two-billion-cycle
samples, the interpreter measured 1.482x median, 1.464x minimum, and 1.87% CV;
the JIT measured 3.522x median, 3.519x minimum, and 2.08% CV. Both engines
produced the pinned `F29E02EB` LED waveform. These accelerators are selected by
complete instruction signatures and decoded call/literal targets, never by a
WLED address; unsafe or unfamiliar calls continue in the guest.

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
BRUCE_BIN=/path/to/bruce.bin \
MARAUDER_BIN=/path/to/marauder.bin \
MESHTASTIC_BIN=/path/to/meshtastic.bin \
NERDMINER_BIN=/path/to/nerdminer.bin \
OPENHASP_BIN=/path/to/openhasp.bin \
TASMOTA_BIN=/path/to/tasmota32.bin \
WLED_BIN=/path/to/wled.bin \
./scripts/bench-stock-roms.sh
```

The default gate runs one unmeasured warm-up followed by three two-billion-
cycle samples. It rejects runs that stop early, trap, spend most wall time
blocked in the host, or let even the slowest accepted sample miss the real-time
threshold. The report includes median wall time, median and minimum realtime,
and realtime coefficient of variation (population standard deviation divided
by the mean). Configure it with `CYCLES`, `WARMUPS`, `REPS`, `ENGINE`,
`MIN_REALTIME`, and `ESP_HZ`.

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

In a six-billion-cycle WLED 16.0.1 scenario on 2026-09-09, compiled blocks
execute 80.6% of retired instructions. The remaining count includes work
performed by architectural and service hooks, so it is not all interpreter
dispatch. Coverage is workload-specific and is not itself a speed score; use
an interleaved A/B and compare observable output when evaluating JIT changes.

The same firmware also exposed a host-boundary cost: 353,092 empty guest UDP
polls in a four-billion-cycle run. Coalescing only the host syscalls for those
polls, with a 100-microsecond guest-time latency bound, reduced the count to
176,886. Six alternating ARM64 JIT A/B pairs reduced mean CPU time from 3.488
to 3.315 seconds (4.97%) with identical retired state and firmware output.

WLED also revisits many JIT entries that have already been hot-counted or
proven uncompileable. Reusing the dispatcher's exact set-and-way match avoids a
second full-table probe and stops rewriting dead hot counters. Eight alternating
four-billion-cycle ARM64 pairs reduced mean CPU time from 3.140 to 3.074 seconds
(2.1%); all retired and JIT instruction counts were identical. Three shorter
pairs across all seven production images were neutral within timer resolution
except WLED, which improved by 5.5%.

Windowed `RETW` is another frequent dispatch boundary because its destination
PC and windowbase come from architectural state rather than the opcode. An
exact runtime lookup can enter an already-compiled target's guarded chain entry
while retaining the normal loop, timer, invalidation, and verification bounds.
Eight alternating four-billion-cycle WLED pairs reduced mean CPU time from
3.140 to 2.583 seconds (17.8%) and hook dispatches from 97.3 million to 38.5
million. Across three two-billion-cycle pairs for all seven production images,
retired-instruction throughput improved by 1.6% to 21.9%; every scripted JIT
and interpreter artifact remained unchanged.

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
