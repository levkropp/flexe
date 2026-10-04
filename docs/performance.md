# performance

Flexe reports elapsed emulated time separately from retired guest
instructions. Keep those measurements separate when evaluating a workload.

## metrics

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

## current production baseline

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

## esp32-s3 wled baseline

The pinned WLED 16.0.1 ESP32-S3 4M QSPI scenario was measured on 2026-09-19
on an Apple-silicon MacBook with a `Release`, LTO, host-native build. Each run
executed 4 billion aggregate cycles (16.667 nominal ESP32 seconds) and had to
match all 317 completed RMT frames, 13,599 chunks, 321,304 pulse words, final
CPU/time state, and the `46F65AC5` pulse digest:

| Engine | Three wall-time samples | Real-time range | Native coverage |
|---|---:|---:|---:|
| Interpreter | 11.21, 12.13, 12.40 s | 1.34--1.49x | n/a |
| JIT | 6.86, 7.17, 7.27 s | 2.29--2.43x | 97.9% |

Both engines retired exactly 1,819,518,818 instructions and stopped on the
4,000,000,000-cycle aggregate boundary. The JIT retired 1,780,691,463 of
those instructions natively. Its chained-block horizon is checked before
entry guards that can raise a precise LX7 window exception, so a final short
timeslice cannot retire one instruction beyond the frontend budget.
The 2026-09-20 unsupported-MMIO audit of the same run reports zero sites; the
dual-core ASSIST_DEBUG recorder is read on demand and adds no dispatch-loop
work.

The JIT now translates ordinary code in every target-described executable
range, including unhooked mask-ROM code. The scanner still asks the exact ROM
hook registry before every instruction, so a service boundary cannot be
compiled merely because a neighboring ROM address is eligible. This is a
target-level mechanism shared by firmware rather than a WLED address list.

## agon vdp critical-section dispatch

The unmodified Agon VDP 2.16.0 workload keeps native FreeRTOS, audio generation
and I2S VGA DMA running. High JIT coverage does not guarantee long native runs:
profiling showed frequent returns to C around critical-section PS writes.
`RSIL` and `WSR PS` previously set the interrupt-check hint even when no enabled
source was pending, so the next instruction checkpoint always ended the run.

These two opcodes now keep the native chain when `INTERRUPT & INTENABLE` is
zero. An enabled pending source still exits at the exact PS-write boundary,
including when the new PS masks it. Timer horizons, MMIO interrupt checkpoints
and register-window collision guards remain active. This is an instruction
optimization shared by firmware, with no application addresses or hooks.

Measured on 2026-10-03 on Apple A18 Pro, macOS 26.4.1, Apple Clang 21.0.0,
Release `-O3`, native tuning and LTO, without PGO or profiling instrumentation.
One warmup per build preceded five interleaved before/after pairs, alternating
which build ran first. Each sample passed the complete UART/VGA/audio/key
scenario, then measured five modeled seconds of 320x240 scanout with capture.
The baseline was commit `1996950`.

| JIT build | median soak wall | aggregate retired MIPS | modeled realtime |
|---|---:|---:|---:|
| before PS-write optimization | 4.197 s | 287.61 | 1.191x |
| after PS-write optimization | 3.261 s | 370.18 | 1.533x |

Throughput increased 28.7%, with 22.3% less wall time. Every soak retired
1,207,016,237 instructions and captured 291 frames; UART and video digests,
DAC tone and keyboard assertions were identical. Separate diagnostic runs
covering boot plus soak reduced native dispatches from 121.0 million to
68.9 million and increased instructions per entry from 16.0 to 28.1. Native
coverage during the soak remained 96.6%.

The full MOS/BBC BASIC scenario also passed in both engines after the change.
See [the Agon test commands](testing.md#agon-mos-and-bbc-basic) and
[the native VDP benchmark](testing.md#agon-vdp). Set
`FLEXE_JIT_STATS=1` when invoking `flexe-agon-vdp-test` directly to print JIT
counters after the native VDP scenario. A separate `FLEXE_PROFILE=ON` build
reports interpreter samples when run with `FLEXE_PROFILE=1`.

### effect on the classic ESP32 corpus

The same `1996950` versus `a9ae927` comparison was repeated on seven pinned
production images, using identical Release settings on the same host. These
measurements cover the generic runner's normal ROM/RTOS service configuration,
including process startup and boot; Agon's native FreeRTOS soak above measures
a different workload. Each run executed two billion core-0 cycles with a
10,000-instruction scheduling batch, a zero-unmapped-access limit and the
official `esp32_rev300_rom.elf`. One warmup per image/build preceded five
interleaved pairs, alternating execution order. Runs were serialized.

| Firmware | Before median wall | After median wall | Throughput change |
|---|---:|---:|---:|
| Bruce 1.16.1 CYD | 0.422 s | 0.396 s | +6.8% |
| Marauder 1.15.1 CYD | 0.266 s | 0.247 s | +7.9% |
| NerdMiner 1.8.3 | 0.773 s | 0.683 s | +13.1% |
| Meshtastic 2.7.26 T-Beam | 0.358 s | 0.324 s | +10.4% |
| openHASP 0.7.0-rc13 Lanbon L8 | 0.586 s | 0.592 s | -0.9% |
| Tasmota 15.6.0 | 0.744 s | 0.689 s | +8.0% |
| WLED 16.0.1 | 4.124 s | 3.987 s | +3.4% |

Throughput change is `before_wall / after_wall - 1`, calculated from unrounded
medians. Every run passed, with identical retired instruction counts and UART
digests across both builds and all samples. The measured benefit extends to
six images; the small openHASP difference was investigated with ten additional
interleaved pairs, each timing five complete runs. Their median totals were
2.958 s before and 2.963 s after, a 0.2% wall-time difference; child CPU time
also differed by 0.2%. Paired timings varied in both directions, so no
meaningful performance change was detected for openHASP.

The seven driven stock-ROM scenarios also completed on both JIT builds and the
current interpreter, including each scenario's six-billion-cycle soak. They
agreed on every framebuffer or LED digest and passed their hardware/network
assertions with zero unhandled MMIO or unregistered ROM calls. This audit
refreshed two old framebuffer references in `check-stock-roms.sh`: Marauder's
converged Swift Pair screen (`B4586420`) and openHASP's complete RGB page
(`CCBF47C5`). Both outputs were already present before the PS optimization,
matched the interpreter and were inspected visually. The RGB digest also
matches an independently constructed 320x240 page of 107 red, 106 green and
107 blue columns.

To reproduce individual generic runs with each Release build:

```sh
FLEXE_ROM_ELF=/path/to/esp32_rev300_rom.elf \
  /usr/bin/time -p /path/to/build/flexe-generic-rom-test \
  --cycles 2000000000 --batch 10000 --max-unmapped 0 /path/to/firmware.bin
```

Alternate the two builds, warm them up first, compare median timings and
require equal instruction counts and UART digests. Use the
[stock-ROM gates](testing.md#production-rom-gates) for the driven scenarios.

## reproducible compute benchmark

`bench-compute.sh` builds an in-repository Arduino sketch and executes a fixed
number of rounds. It covers a dependent scalar chain, strided memory traffic,
SHA-256-shaped rotate/XOR work, and byte loads/stores. Both engines must return
the same checksum.

```sh
ARDUINO_CLI=/path/to/arduino-cli ./scripts/bench-compute.sh
```

Useful overrides are `BENCH_ROUNDS`, `BENCH_REPS`, and `MIN_REALTIME`.

## production firmware

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

## jit coverage

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

## profiling

Build the sampling profiler separately so its dispatch-loop layout does not
affect normal measurements:

```sh
cmake -S . -B build-prof -DFLEXE_PROFILE=ON
cmake --build build-prof --target flexe-generic-rom-test -j
FLEXE_PROFILE=1 ./build-prof/flexe-generic-rom-test firmware.bin
```

`FLEXE_PROFILE_SP=0x...` restricts samples to a task stack, and
`FLEXE_JIT_STATS=1` enables JIT counters in compatible runners. Profile before
changing hot code: even compiled-out instrumentation can move the dispatch
loop enough to change results.
