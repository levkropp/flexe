# Architecture

This document is a map of Flexe's current implementation. It explains where
architectural decisions live and how the major subsystems compose; it is not a
register-support checklist. See [hardware completeness](docs/hardware-completeness.md)
for the exact modeled boundary, [firmware compatibility](docs/compatibility.md)
for validated workflows, and [performance](docs/performance.md) for measured
throughput.

## Design contract

Flexe is a functional ESP32-family emulator built around four rules:

1. **Target data, not firmware identity, describes the machine.** Memory maps,
   core generation, reset state, interrupt wiring, MMU geometry, peripheral
   instances, and clock/reset connections come from versioned target
   descriptors.
2. **The interpreter defines CPU behavior.** Native translation is an
   optimization over the same architectural state. Cold or unsupported code
   falls back safely, and differential verification can compare both paths.
3. **Firmware-visible time and ordering are deterministic.** Timers,
   interrupts, DMA completion, sleep, and both cores share one emulated
   timeline. Flexe does not claim cache-, bus-, or cycle-accurate timing, nor
   truly simultaneous host execution of the two guest cores.
4. **Unsupported behavior stays visible.** Unknown MMIO and unregistered
   service calls are counted and can be attributed to guest PC/core. A quiet
   zero-valued fallback helps firmware continue far enough to diagnose the
   next gap, but it is never evidence that the device is implemented.

Classic ESP32/LX6 and ESP32-S3/LX7 are currently stable functional targets.
That status is narrower than complete silicon equivalence; the capability
matrix remains authoritative.

## System composition

```text
CLI / embedding / firmware harness
                  |
          flexe_session_t
      +-----------+-----------+
      |           |           |
 target desc   loader      host endpoints
      |        + ROM ELF   (UART, GPIO, buses,
      |           |         storage, network)
      +-----+-----+-----------+
            |
       shared memory
       + MMIO fabric
       /           \
  CPU0 / CPU1    device models
       |          + interrupts/DMA
 interpreter
 or tracing JIT
       |
 ROM and service-hook boundaries
```

[`flexe_session_t`](src/flexe_session.h) is the machine owner used by the CLI
and integration harnesses. Construction in
[`flexe_session.c`](src/flexe_session.c) follows this order:

1. Probe the ESP image and resolve or verify the selected target.
2. Allocate target-sized memory backings and optionally load the matching
   official ROM ELF.
3. Construct the target's MMIO devices from capabilities and descriptors.
4. Load the application, install its flash-MMU state, and synthesize only the
   boot state absent from a standalone app image.
5. Reset one or two Xtensa cores from target data and connect interrupts,
   timers, ROM/service hooks, board devices, and host endpoints.
6. Install the native JIT on supported hosts unless the caller selects the
   interpreter.

The session also owns batch scheduling, software/deep-sleep rebuilds, APP-CPU
startup and reset, asynchronous wake, and the lifetime of every attached
compatibility service.

## Target descriptions

[`target_types.h`](src/target_types.h) is the small stable boundary used by
hot CPU and memory headers. The complete versioned descriptor is declared in
[`target.h`](src/target.h) and instantiated in [`target.c`](src/target.c).
A descriptor contains:

- image chip ID, LX generation, core count, reset vectors, and core identity;
- physical backings, guest aliases, executable ranges, and flash-MMU layout;
- capability bits selecting reusable IP models;
- register geometry, reset values, interrupt sources, GPIO-matrix signals,
  clocks, resets, and DMA identities for each device instance;
- bootloader-to-application state such as flash geometry and ROM ABI data.

`periph_create()` consumes those capabilities to construct the SoC. A device
model should not infer the chip from a firmware entry point or duplicate a
target address map internally.

Large or fast-changing device descriptors live with their owning device and
attach through the target extension registry. For example,
[`touch_v2.h`](src/touch_v2.h) owns the touch-v2 descriptor and its stable
extension tag. This keeps peripheral changes from invalidating the entire
build through `target.h`.

Board population is separate from the SoC descriptor. Optional PSRAM and
external SPI/I2C/storage devices are attached during machine construction, so
the same controller model can serve a headless harness, a CYD, or another
board without firmware-specific controller logic.

## Images, ROM, and memory

[`loader.c`](src/loader.c) understands standalone ESP application images and
merged factory images. It validates the target and declared flash geometry,
copies internal-memory segments, retains flash-backed segments in NOR, and
seeds the target-described cache MMU. ESP-reserved non-loaded image blocks are
parsed for correct offsets but never copied to guest address zero.

A standalone app has no partition table or bootloader handoff, so the loader
creates the smallest coherent factory layout needed by the SDK. A merged
image keeps its real partitions and bytes. Software reset reconstructs
internal application segments from the original image without overwriting
live guest-modified flash; selecting a different OTA image is a separate,
currently unsupported operation.

[`rom_elf.c`](src/rom_elf.c) loads immutable mask-ROM sections and the ROM's
linker-described data/BSS interface from an official Espressif ROM ELF. ROM
symbol data also resolves ABI state that cannot safely be guessed from an SDK
version.

[`memory.c`](src/memory.c) uses host allocations for SRAM, ROM, flash data and
instruction views, RTC memory, and optional PSRAM. A 4 KiB direct page table
maps the 32-bit guest address space onto those backings, including physical
aliases and live cache-MMU remaps. Ordinary RAM access is a page lookup and
host load/store; a miss enters the MMIO path.

MMIO handlers are registered by page or range and may implement byte-enable-
aware writes for FIFO, strobe, and write-one-to-clear registers. A miss that
matches neither memory nor a device is counted separately from modeled
peripheral fallback. Instruction-stream changes notify the interpreter and
JIT so stale decoded or translated code is discarded.

## Xtensa execution

[`xtensa.c`](src/xtensa.c) is the reference implementation for the common
windowed Xtensa behavior used by LX6 and LX7. CPU state includes the 64-entry
physical register file, window state, special/user registers, exception and
debug state, interrupt inputs, compare timers, loop registers, floating-point
state, and separate elapsed-cycle and retired-instruction counters.

The interpreter predecodes executable memory, then executes instructions with
architectural register-window, exception-vector, interrupt, timer, and
zero-overhead-loop behavior. Guest window vectors are the default. The
optional window accelerator is admitted only for canonical handlers and must
preserve the same visible state.

[`jit.c`](src/jit.c) is a tracing basic-block JIT for ARM64 and x86-64 hosts.
It is selected by a target translation profile rather than by chip name or
firmware address. Its important correctness properties are:

- hot code is compiled lazily; unsupported instructions and contexts return
  to the interpreter;
- exact ROM/service-hook membership terminates translation at host boundaries;
- block keys include architectural context such as register-window and loop
  state;
- block chaining is bounded by scheduler, timer, interrupt, debugger, and
  batch-budget safepoints;
- flash writes and MMU remaps invalidate affected native code;
- `--jit-verify` journals RAM, runs replayable blocks through both engines,
  and compares architectural state. MMIO blocks are not replayed because
  device reads and writes can be destructive.

Backend-specific emission lives in
[`jit_emit_arm64.h`](src/jit_emit_arm64.h) and
[`jit_emit_x64.h`](src/jit_emit_x64.h). Unsupported hosts build
[`jit_stub.c`](src/jit_stub.c) and use the interpreter. The AOT recompiler in
`tools/` remains experimental and opt-in; it is not part of the supported
default execution path.

## Time, interrupts, and two cores

Flexe keeps work and time distinct:

- `insn_count` is guest instructions actually retired;
- `cycle_count` is elapsed emulated core cycles, including idle advancement;
- `virtual_time_us` records time skipped by functional waits and scheduler
  fast-forwarding rather than instruction execution.

The interpreter and JIT stop at a computed event horizon so compare timers,
peripheral deadlines, and newly deliverable interrupts remain observable.
Device models expose their next event and raise target interrupt sources
through the interrupt matrix. `WAITI` and idle firmware may advance directly
to useful work without being credited with retired instructions.

The two guest cores share memory, peripherals, and time, but execute
cooperatively in deterministic batches. `flexe_session_post_batch()` starts
CPU1 through the target's boot contract, alternates runnable work, handles
spinlock handoff, and reconciles both cores onto one timeline. This preserves
firmware-visible ordering and cross-core interrupts; it intentionally does
not model sub-instruction races or simultaneous cache/bus contention.

## Device models and host boundaries

[`peripherals.c`](src/peripherals.c) contains the classic SoC container and the
composition layer for reusable target-described devices. Larger IP blocks
live in focused files such as `gpio.c`, `gdma.c`, `timer_group.c`,
`spi_mem.c`, `i2s_v2.c`, and `lcd_cam.c`.

A functional device model normally owns:

- register reset/read/write behavior and reserved-bit masks;
- clock/reset gating and any retained state;
- scheduled completion and interrupt transitions;
- DMA descriptor ownership and memory effects where relevant;
- a controller-level host API for external wires or media.

Fast mode often completes a bus transaction as one ordered operation rather
than synthesizing every SCL, SPI clock, UART baud, or PWM edge. Aggregate
callbacks still expose meaningful transactions and output state. Individual
edges belong in a model only when firmware-visible behavior depends on them.

Host attachment APIs in [`peripherals.h`](src/peripherals.h) connect virtual
I2C targets, SPI devices, SD/MMC cards, CAN peers, Ethernet/MDIO, audio, PWM,
and other external endpoints without putting host policy inside a register
model. [`sandbox_input.h`](src/sandbox_input.h) and
[`sandbox_events.h`](src/sandbox_events.h) provide a bounded frontend-neutral
input/event vocabulary. Board devices such as the ILI9341/XPT2046/SD stack are
consumers of the same controller boundary.

## ROM and service compatibility

Not every useful fast-mode boundary is silicon MMIO. The hook registry in
[`rom_stubs.c`](src/rom_stubs.c) can implement mask-ROM services and selected
SDK/runtime services at their ABI boundary. Hooks are resolved from official
ROM symbols, application ELF symbols, stable ABI information, or complete
structural fingerprints. An exact hook bitmap keeps the normal instruction
path cheap and prevents the JIT from compiling across a service boundary.

Compatibility mode can replace selected FreeRTOS, display, Wi-Fi, Bluetooth,
VFS, and library services while still executing guest callbacks and device
drivers. Native mode (`-N`) runs the firmware's own FreeRTOS and relies more
heavily on modeled hardware. Documentation and tests must label service-shim
success separately from MMIO hardware support; one is not evidence for the
other.

## Reset, sleep, and retention

A system software reset rebuilds CPUs, controller registers, service state,
and the JIT while preserving state that lives outside the resetting SoC or is
architecturally retained. This includes live NOR contents and host device
attachments; target-specific RTC, pad-hold, and external-memory retention is
handled explicitly. Controller FIFOs and in-flight transactions do not leak
through the rebuild.

APP-CPU reset is applied at a safe scheduler boundary without rebuilding the
shared SoC. Light sleep advances or waits on the shared timeline and resumes
the machine; deep sleep takes the reset path with the target's RTC retention
and wake cause. Asynchronous GPIO/touch wake remains host-drivable while both
cores retire no instructions.

## Diagnostics and verification

The main correctness layers are:

1. Unit tests for CPU, memory, loaders, device state, timing, and service
   boundaries.
2. Interpreter/JIT differential tests, including broad encoding-space sweeps.
3. Compiled Arduino and ESP-IDF fixtures that drive real registers, DMA,
   interrupts, and host endpoints in both engines.
4. Production firmware scenarios that require sustained useful behavior,
   output parity, bounded execution, and explicit unsupported-access limits.
5. GCC, Clang, sanitizer, and cross-backend CI builds.

See [testing](docs/testing.md) for commands and gate structure. An emulator
change is not complete merely because firmware prints a boot banner: the
relevant state transition needs focused coverage, and any compatibility claim
needs an end-to-end gate.

## Source map

| Area | Primary sources |
|---|---|
| Machine ownership and scheduling | `src/flexe_session.[ch]` |
| Target data and extension registry | `src/target.[ch]`, `src/target_types.h` |
| CPU and window behavior | `src/xtensa.[ch]`, `src/window_accel.c` |
| Native translation | `src/jit.c`, `src/jit_emit_*.h` |
| Memory, image, ROM, and XIP | `src/memory.[ch]`, `src/loader.[ch]`, `src/rom_elf.[ch]`, `src/flash_mmu.[ch]` |
| SoC/device composition | `src/peripherals.[ch]` and focused device modules |
| ROM and compatibility services | `src/*_stubs.[ch]`, `src/guest_call.[ch]`, `src/firmware_scan.[ch]` |
| Board/external device models | `src/spi_display.[ch]`, `src/axp192.[ch]`, `src/sx127x.[ch]`, `src/ublox_gps.[ch]` |
| CLI and frontend event plumbing | `src/main.c`, `src/sandbox_*.[ch]` |
| Validation | `tests/`, `tests/fixtures/`, `tools/`, `scripts/` |

## Extending Flexe

For a new target, add architectural geometry to a target descriptor, opt into
only reusable capabilities the target actually has, and supply focused tests
for every differing register layout or connection. A descriptor existing is
not sufficient to claim execution support.

For a new device, keep its state machine and descriptor in the owning module,
register MMIO through the memory API, connect clock/reset/interrupt/DMA state,
and expose external behavior at a controller-level attachment boundary. Use a
target extension when adding the descriptor to `target.h` would create broad
rebuild coupling.

For a new instruction, implement and test interpreter semantics first, update
the disassembler, then add native translation only when its guards and exit
behavior can be compared against the interpreter. A JIT optimization must be
safe to decline at runtime.

For a host service acceleration, prefer symbols or complete structural
validation over firmware names and fixed PCs. Preserve the guest ABI, time,
interrupt, and memory effects, and keep the boundary visible to every
execution engine.
