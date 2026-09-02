# flexe

**f**ree **l**ittle **x**tensa **e**mulator

a lightweight xtensa lx6 (esp32) emulator written in c. boots real esp-idf firmware binaries. no dependencies beyond a c11 compiler, cmake, and openssl.

oh, and it's **fast**. like, stupidly fast. there's a jit in here now.

## the numbers

a real esp32 lx6 hums along at 240 mhz. that's the bar. here's flexe on apple silicon (pgo build, 500M-1B cycle workloads):

| workload | interpreter | jit (default) | vs real esp32 |
|---|---:|---:|---:|
| tjpgd (jpeg decode) | 330 mips | **693 mips** | 2.9× faster |
| real_time_stats (dual-core freertos) | 334 mips | **499 mips** | 2.1× faster |
| cpu_bench (alu/mem loops) | 318 mips | **2476 mips** | 10.3× faster |

On x86-64 the committed `bench-compute.sh` reports 1789 MIPS under the JIT
(7.45× real-time) against 187 interpreted, and the stock ROMs run at 3.2×
(NerdMiner) and 10.6× (Marauder) real-time.

so yeah — flexe emulates an esp32 at 2–10× the speed of an actual esp32. the jit traces hot basic blocks through branches, chains them into long native runs, and constant-folds literal loads. cold code falls back to the interpreter, which itself is already above real-time.

```
$ ./build/xtensa-emu -q -s build/hello_world.elf -c 5000000 build/hello_world.bin
I (0) cpu_start: Starting scheduler on PRO CPU.
Hello world!
```

## what it does

flexe interprets (and now jits) the xtensa lx6 instruction set well enough to boot unmodified esp-idf applications. it handles the full init sequence — from reset vector through bootloader setup to `app_main`.

### implemented

- full xtensa lx6 isa: alu, shifts, branches, loops, mac16, fpu
- windowed registers with synthesized spill/fill (call4/8/12, entry, retw)
- exception/interrupt dispatch (levels 1–7, timer ccompare, waiti), plus the
  complete 69-source dual-core ESP32 interrupt matrix with fan-in and live remapping
- esp32 memory map (sram, rom, flash, rtc, psram, peripheral i/o)
- hardware flash MMU: complete 64 KiB DROM0/IRAM0/IRAM1/IROM0 mappings,
  dual-core table invalidation, flash programming coherence, and translated-code invalidation
- mmio peripherals: all three uarts, gpio with eight-channel classic
  sigma-delta/PDM, dport, rtc/rtcio, efuse, watchdog,
  legacy FRC1/FRC2 timers, both timer groups with four 64-bit APB
  counters/alarms, dual UHCI UART DMA with H:5/SLIP framing, GP-SPI2/3 DMA,
  classic I2C0/1 masters, RTC-domain I2C/ULP transactions,
  SJA1000-compatible TWAI/CAN,
  Synopsys-based classic Ethernet MAC with descriptor DMA and Clause-22 MDIO,
  dual-slot native SDMMC with PIO/IDMAC DMA, classic SDIO slave
  HINF/HOST/SLC with scatter/gather descriptor DMA,
  I2S0/1 circular DMA,
  eight-channel classic RMT, 16-channel classic LEDC PWM/fades,
  eight-unit/two-channel classic PCNT, both classic MCPWM motor-control units
  with sync/capture/fault/dead-time/carrier paths, ADC1/2, DAC1/2
- raw hardware crypto: aes-128/192/256, sha, rsa modular math and interrupts
- cyd devices: ili9341 display, xpt2046 touch, sd/fat and spiffs storage
- host-backed lwip sockets plus virtual wifi, bluetooth, and phy boundaries
- rom function stubs: ets_printf, memcpy, memset, strlen, cache ops
- freertos stubs: tasks, queues, semaphores, notifications, delays
- esp_timer stubs with callback dispatch
- nvs flash stubs
- gpio driver stubs
- elf symbol loading, breakpoints, verbose trace mode
- jit compiler: hot blocks → native code (arm64 + x86-64), on by default
- 660 tests

### profiling

`cmake -S . -B build-prof -DFLEXE_PROFILE=ON` builds an interpreter sampling
profiler; run any harness with `FLEXE_PROFILE=1` for the top guest PCs, and
`FLEXE_JIT_STATS=1` on the stock-ROM runner for JIT coverage. It is off by
default and lives in its own translation unit for a reason: an earlier version
sat in `xtensa.c` behind the same compile-time switch and, *even fully
compiled out*, cost 4% on both stock ROMs — the handful of bytes it added
shifted the dispatch loop's code layout. Measuring that needed an interleaved
A/B, since a straight before/after was swamped by machine drift.

It reports hot PCs and, more usefully, hot 1 KB regions: diffuse code
(unrolled hashing, a large state machine) spreads samples so thinly that no
single PC stands out while the region still names the function.

What it found: **32% of NerdMiner's execution is inside
`xPortEnterCriticalTimeout`** — a third of the machine spent spinning on locks
held by the other core. Per-PC sampling had put that at 10%, because the loop
is spread over five addresses. This is an artifact of interleaving the two
cores a batch at a time: the core holding a lock does not run until the
spinning core's batch ends. It can also filter samples to one task's stack (`FLEXE_PROFILE_SP=0x3FFD7000`),
which is the only way to ask what a *particular* task is doing in a
symbol-less ROM running its own scheduler — there is no task list to read, but
each task has its own stack, so the stack-pointer histogram enumerates them.

`FLEXE_SOAK_STATS=1` on the stock-ROM runner reports retired instructions,
bytes fed to the display, and the framebuffer before and after the soak. That
distinguishes a firmware that is idle from one that is busy but not
progressing, and a display task that has stopped from a UI redrawing the same
picture: NerdMiner retires 2.7 billion instructions after provisioning with
`display_bytes=0`, while Marauder writes 5.3 million bytes over the same
window.

## building

```
cmake -S . -B build
cmake --build build -j
```

(macOS: point cmake at homebrew openssl with `-DOPENSSL_ROOT_DIR=$(brew --prefix openssl@3)`)

produces two binaries:
- `build/xtensa-emu` — the emulator
- `build/xtensa-tests` — test suite

## usage

```
# basic run (jit is on by default)
./build/xtensa-emu firmware.bin

# with elf symbols and cycle limit
./build/xtensa-emu -s app.elf -c 10000000 firmware.bin

# quiet mode (suppress emulator info, show only firmware output)
./build/xtensa-emu -q -s app.elf -c 5000000 firmware.bin

# interpreter-only mode (the slow lane, for comparison or debugging)
./build/xtensa-emu --no-jit -s app.elf -c 10000000 firmware.bin

# verbose trace to stderr
./build/xtensa-emu -T -s app.elf -c 1000000 firmware.bin 2>/tmp/trace.log
```

### flags

| flag | description |
|------|-------------|
| `-J` | enable jit (default: on where supported) |
| `--no-jit` | disable jit, run fully interpreted |
| `--jit-stats` | print jit block/coverage statistics on exit |
| `-s ELF` | load elf for symbols and firmware hooks |
| `-c N` | stop after N executed instructions (both cores counted) |
| `-q` | quiet (suppress emulator info, show only firmware output) |
| `-T` | verbose execution trace to stderr |
| `-b ADDR` | set breakpoint at address |
| `-d ADDR LEN` | hex dump memory region on exit |

### trace filter

a post-processing tool for verbose trace output:

```
./build/trace-filter -u trace.log    # unregistered rom calls
./build/trace-filter -e trace.log    # exceptions
./build/trace-filter -w trace.log    # window spill/fill events
./build/trace-filter -r trace.log    # all rom calls
./build/trace-filter -p trace.log    # panic/abort path
./build/trace-filter -s func trace.log  # instructions in a function
```

## architecture

switch-based interpreter core + a tracing jit on top. interpreter: fetch → decode → execute → loop check → interrupt check → advance ccount. the jit watches hot pcs, compiles basic blocks (continuing through conditional branches as traces), chains blocks together natively, and folds flash literal loads into immediates. anything it can't handle simply runs interpreted — no correctness cliff.

```
src/
  xtensa.c           interpreter core (~3500 lines, every isa instruction)
  xtensa.h           cpu state struct
  jit.c              tracing jit: scan, compile, chain, dispatch
  jit_emit_arm64.h   arm64 machine code emitters
  jit_emit_x64.h     x86-64 machine code emitters
  memory.c           address space: sram, rom, flash, psram, peripheral dispatch
  peripherals.c      mmio handlers for esp32 peripherals
  rom_stubs.c        pc-hook mechanism for rom + firmware function interception
  freertos_stubs.c   freertos task/queue/semaphore stubs
  esp_timer_stubs.c  esp_timer api stubs with callback dispatch
  loader.c           esp32 .bin + elf loading
  elf_symbols.c      elf symbol table parser
  xtensa_disasm.c    disassembler
  main.c             cli frontend
```

~20k lines of c total. see [ARCHITECTURE.md](ARCHITECTURE.md) for detailed design notes.

## testing

```
./build/xtensa-tests
# 660 tests, 13034 passed, 0 failed
```

tests cover individual instructions, memory operations, windowed registers, exceptions, interrupts, peripherals, rom stubs, freertos, esp_timer, nvs, gpio driver, and end-to-end firmware compatibility.

For host-memory and undefined-behavior validation, build with
`-fsanitize=address,undefined`; the default 4 MB predecode configuration is
covered by both the unit suite and the compiled/stock-ROM integration runners.

The compiled Arduino hardware gates below all pass — 26 of 26 on x86-64
Linux (GCC 15, Arduino-ESP32 2.0.11), under both the interpreter and the JIT.

One deliberate gap is worth stating rather than leaving to be rediscovered:
the ESP-IDF UART driver (`uart_driver_install`, `uart_read_bytes` and the rest
of `driver/uart.h`) is stubbed to report success and do nothing. Arduino's
`HardwareSerial` carries its own ISR and ring buffer and is modelled at the
register and `uartBegin` level instead, which is the path every CYD ROM takes;
the two cannot both own the UART. Firmware written against the IDF driver
directly would install cleanly and then never move a byte. Making it work
means unpicking that layering, not adding a register.

`test-gpio-isr.sh` covers `attachInterrupt()`. The other gates that touch GPIO
drive it as a level input and poll, so nothing exercised interrupt delivery —
and it did not work: a per-core interrupt source raised its CPU line without
ever running a registered handler, so no GPIO interrupt reached the guest by
any route. A CYD's touch controller signals with PENIRQ, so this matters on
the target hardware.

The stock-ROM scenario now provisions NerdMiner through its captive portal's
own `/wifisave` form and then keeps the firmware running for several more
seconds, checking it is still alive. Both were needed to find a crash that had
been sitting there the whole time: Flexe used to write `WL_CONNECTED` into a
hardcoded DRAM address to make NerdMiner believe it had associated, back when
no WiFi events were delivered. That address is not the status variable — the
firmware reads an `EventGroupHandle_t` from it — so the write left a handle of
3, `xEventGroupClearBits()` took a critical section on lock address 3+28, and
the ROM died on `assert failed: spinlock_acquire` and rebooted, about sixteen
seconds of guest time in. The scenario stopped just short of that, and the
firmware only associates at all once provisioned, so nothing looked. Firmware
now learns it is connected the way hardware tells it, through the event
handlers. The panic markers are matched in the UART byte stream rather than in
the captured log, because a firmware in a reboot loop emits hundreds of
kilobytes and the log is a fixed buffer that stops recording once full. The
render is still sampled at the converged point, before the soak: a live UI
keeps animating, and comparing two engines mid-animation makes them disagree.

The scenario also requires that the firmware *learns* it associated, which
means its own event handler has to be reached. Arduino registers that handler
through `esp_event_handler_instance_register`, which was not in NerdMiner's
profile; the address was found by relocating the symbol from a locally built
Arduino-ESP32 2.0.11 sketch and then confirmed by what the ROM passes it —
`("WIFI_EVENT", ESP_EVENT_ANY_ID, handler)`, matching the reference build,
where the one neighbouring candidate takes concrete event ids with a null
handler instead.

Every `errno` the socket layer set was invisible to the firmware. Newlib keeps
`errno` inside its reentrancy structure, reached through `__errno()`, while
Flexe wrote to a fixed scratch word — two different addresses, so firmware
read whatever happened to be in its own slot. Firmware branches on `errno`:
Arduino's `WiFiUDP::parsePacket()` treats `EWOULDBLOCK` as "nothing to read"
and logs an error for anything else, so a UDP poll with nothing pending — the
normal case, on every poll — was reported as a hard failure thousands of times
a second, over a megabyte of UART per run. `__errno()` is now hooked to return
the word the model writes. The scenario fails if the firmware reports a socket
error at all.

Socket error paths also set a truthful `errno` at the point of failure. Most of them returned -1
without touching it, leaving firmware to read whatever was last set, and
firmware branches on that: Arduino's `WiFiUDP::parsePacket()` treats
`EWOULDBLOCK` as "nothing to read" and logs an error for anything else.

`test-task-wdt.sh` covers the task watchdog. `esp_task_wdt_init`, `_add` and
`_reset` answered with a bare success and the rest of the family was not
modelled at all, so the real ESP-IDF code ran against a subscription list
nothing had populated. Arduino's `disableCore0WDT()` calls
`esp_task_wdt_delete()` on the idle task and reports the failure — it is in
NerdMiner's boot log on every run. Firmware that subscribes a task and later
unsubscribes it simply could not. The subscription set is modelled now, so the
calls agree with one another and return the `esp_err_t` values ESP-IDF returns
rather than a blanket success. The watchdog deliberately never fires: there is
no wall clock to miss a deadline against, and a spurious reset would be far
worse than a watchdog that never bites — the gate checks the device is still
alive long after it stopped feeding it.

`test-wifi-client.sh` covers station association and an outbound TCP client,
and closes the largest remaining gap for firmware built from source: WiFi
events were recorded and never delivered. Stock ROMs are driven past this by
poking a status address named in their profile, but a source build has no such
address — Arduino's `WiFi.status()` is updated purely from those events, so a
station connect never completed and anything waiting for `WL_CONNECTED` never
got on the air. Alongside that, `esp_wifi_get_config()` zeroed its output
unconditionally, so the guest could never see saved credentials, which is
exactly what WiFiManager and everything built on it checks to decide whether
to start a captive portal; `wifi_stubs_set_sta_credentials()` now lets the
host pre-provision them. `esp_wifi_sta_get_ap_info()` was not modelled at all,
so firmware could not report which network it was on. The gate also covers an
outbound connection, which neither stock-ROM scenario reaches — both only bind
and listen for their portals — with the host echoing a 1000-byte payload that
each side checks against the other. One more was needed to make the Arduino
path work at all: `esp_netif_dhcpc_start()` was unmodelled, and
`WiFiSTAClass::begin()` returns `WL_CONNECT_FAILED` if it reports failure,
*before* ever calling `esp_wifi_connect()`. So `WiFi.begin()` — what real
firmware calls — failed without the firmware ever associating, and the gate
now covers that path as well as the IDF calls beneath it.

`test-scheduler.sh` covers the rest of the scheduling surface, and found four
more of the same kind. `vTaskDelayUntil` — how every fixed-rate loop is
written — was not hooked, so the guest's version parked the task on FreeRTOS's
own delayed list and yielded, neither of which Flexe acts on: ten 20 ms
periods completed in 0 us and the loop then span at full speed. Unlike a
missing delay that does not look like a hang, it looks like firmware running
impossibly fast. `xQueueSend` into a full queue reported failure immediately
however long the caller was willing to wait, so a bounded queue had no flow
control. Recursive mutexes failed on the second take by the holder, which in
real firmware is a library deadlocking on its own re-entrant call. And
`vTaskSuspend` ignored its handle argument, so suspending a worker from
another task suspended the caller instead — and marked the TCB `TASK_UNUSED`,
the free-slot state, so it could never be resumed and its slot could be handed
to the next `xTaskCreate`. `vTaskResume` did not exist at all.

`test-task-queue.sh` covers the plain producer/consumer handoff: one guest
task sending to another, with the receiver blocked on it. Every queue gate
before it had a *peripheral* fill the queue from an ISR, which Flexe services
through a separate path, so the most common thing FreeRTOS is used for was
untested — and broken. A receive with a finite timeout parked in
`TASK_SLEEPING`, which the send-side wake path did not look at, so it slept
its entire timeout; and because the blocked call cannot re-run when the task
resumes, nothing ever dequeued the item. A 500 ms receive returned failure at
500 ms while its item had been sitting in the queue since 50 ms.
`xSemaphoreTake` had the mirror-image bug: it pre-set its return to `pdTRUE`,
so a take that timed out reported that it had acquired the semaphore. The same
gate found a family of queue accessors — `uxQueueMessagesWaiting`,
`uxQueueSpacesAvailable`, `xQueuePeek`, `xQueueReset` — that were never hooked
and so ran FreeRTOS's own code against a `Queue_t` Flexe never populates,
reporting an empty queue no matter what was in it.

`test-event-group.sh` covers FreeRTOS event groups, which hung. Like the
software timers these were not stubbed, so the guest's own implementation ran
— and it blocks waiters by threading them onto FreeRTOS's unordered event
lists, which Flexe's scheduler never reads, so the first
`xEventGroupWaitBits()` that had to actually wait never returned. The
non-blocking half kept working throughout, which is what made it easy to miss:
`xEventGroupGetBits()` is a macro for `xEventGroupClearBits(g, 0)`, so reading
the bits exercised none of the machinery. Event groups are now modelled
against Flexe's scheduler, blocking and waking like the task notifications
next to them. This is how the WiFi stack, lwIP and most connect-then-wait
sketches synchronise. Writing the gate also turned up a separate latent bug:
`cycles_per_tick` was read without refreshing the CPU frequency, so any
timeout computed before some other stub happened to refresh it — for many
firmwares, the very first blocking call — was a third short at 240 MHz. A
120 ms wait expired after 80 ms.

`test-frt-timer.sh` covers FreeRTOS software timers, which did not work at
all. Unlike the rest of the FreeRTOS surface these were not stubbed, so the
guest's own timer module was linked in but never driven: Flexe replaces
`vTaskStartScheduler`, which is where real FreeRTOS creates the `Tmr Svc`
daemon, so the daemon did not exist. Reviving it is not an option either —
it blocks on its command queue through `vQueueWaitForMessageRestricted`, which
reaches into the real queue structure, while every queue API the guest calls
is hooked to Flexe's own queue objects, so it would wait forever on an
always-empty queue. Software timers are now modelled directly, like tasks,
queues and semaphores. The failure mode this removes is the bad kind:
`xTimerCreate` returned a handle and `xTimerStart` returned `pdPASS`, and
nothing ever happened. Hooking `xTimerGenericCommand` covers start, stop,
reset, period change and delete in one entry point, since the rest of the API
is macros over it.

`test-esp-timer.sh` covers `esp_timer` callbacks — periodic, one-shot, stop and
restart. Nothing drove that API from guest code before, and two defects were
hiding behind it. The guest clock took `max(executed, skipped)` of two counters
that measure disjoint things, so after boot's fast-forwards `millis()`,
`micros()`, `esp_timer_get_time()` and `xTaskGetTickCount()` all stood still
until executed cycles caught up — 292 ms of frozen time on this fixture, with
every alarm armed in that window coming due late or not at all. Underneath
that, callbacks were dispatched onto a core parked in `WAITI` by the idle task,
where `xtensa_step()` executes nothing: the dispatcher rescheduled each
periodic timer and retired each one-shot on time while never running a single
callback body. The gate checks callbacks against `esp_timer_get_time()` rather
than counting them, because a dispatcher that fires the right number of times
but bunches them up passes a count check.

`test-spi-master.sh` covers the one peripheral a CYD depends on most and no
stock ROM exercises: TFT_eSPI and its relatives drive the GP-SPI registers
directly, so ESP-IDF's `spi_master` driver — queued transactions, DMA
descriptors, command and address phases — was never reached. The fixture puts
its chip select on a pin the emulator models no device on, and the harness
stands in as the slave, choosing every MISO byte from the transfer's own
length and byte index so a short read or a shifted buffer cannot look
plausible.

The optional compiled Arduino gate builds an unmodified `Wire` client, runs
40-byte writes and repeated-start reads through the real ESP-IDF interrupt
driver, checks address NACK behavior, and requires zero unhandled MMIO:

```bash
ARDUINO_CLI=/path/to/arduino-cli ./test-i2c-wire.sh
# stage=0x1C2C0040 ... write_bytes=42 read_bytes=40 memory_ok=1 unhandled=0
```

It has been verified with Arduino-ESP32 2.0.11; `ARDUINO_CLI`,
`FLEXE_ARDUINO_FQBN`, `FLEXE_BUILD_DIR`, and `FLEXE_I2C_BUILD_DIR` are
configurable.

The independent RTC-domain I2C gate uses Espressif's real `rtc_i2c_reg.h`
and `sens_reg.h` register macros. It executes the software-start form of the
classic ULP `I2C_RD`/`I2C_WR` control word on host bus port 2, covering all
eight slave selectors, single-byte register reads and writes, partial-bit
writes, result/DONE latches, NACK and timeout diagnostics, the controller's
shifted raw/status/clear bitmap, and the 16 command registers. Both engines
must reach the attached virtual target with no fallback MMIO or ROM calls:

```bash
ARDUINO_CLI=/path/to/arduino-cli \
FLEXE_ARDUINO_CONFIG=/path/to/arduino-cli.yaml \
./test-rtc-i2c.sh
# engine=jit stage=0x12C0C0DE timing=40/40/200 ... calls=4 bytes=6/2 memory=1 unhandled=0 unregistered=0
# engine=interp stage=0x12C0C0DE timing=40/40/200 ... calls=4 bytes=6/2 memory=1 unhandled=0 unregistered=0
```

The fixture is verified with Arduino-ESP32 2.0.11 and 3.3.11.
`FLEXE_RTC_I2C_BUILD_DIR` optionally preserves its Arduino build directory.

The analog gate builds an unmodified Arduino sketch using `analogRead()`,
`analogReadResolution()`, `dacWrite()`, and `dacDisable()`. It verifies ADC1
and ADC2 at 12- and 9-bit widths, RTCIO-backed DAC state/events, and zero
unhandled MMIO or unregistered ROM calls:

```bash
ARDUINO_CLI=/path/to/arduino-cli ./test-analog-dac.sh
# stage=0xADC0DAC0 adc=2645/1450/85/426 dac=0:0x35/1:0xCA ... unhandled=0 unregistered=0
```

`FLEXE_ANALOG_BUILD_DIR` optionally preserves its Arduino build directory.

The I2S gate builds an unmodified Arduino sketch against ESP-IDF's public
`driver/i2s.h` API. It installs the real full-duplex driver, exercises its
circular `lldesc` rings and ISR queues, writes and reads exact 128-byte PCM
buffers, restarts DMA, and validates the host TX callback/RX injection boundary
at 16 kHz, 16-bit stereo with zero unhandled MMIO or unregistered ROM calls:

```bash
ARDUINO_CLI=/path/to/arduino-cli ./test-i2s-dma.sh
# stage=0x12D5DAA0 result=0/128/128/0x00000000 ... audio=16000/16/2 ... unhandled=0 unregistered=0
```

`FLEXE_I2S_BUILD_DIR` optionally preserves its Arduino build directory.

The RMT gate builds an unmodified sketch against ESP-IDF's public
`driver/rmt.h` API. It sends 96 exact pulse items through a one-block,
64-word channel, forcing the production ISR to refill both 32-word halves.
The gate validates pulse timing, DPORT module reset, TX threshold/end
interrupts, the host pulse boundary, and zero unhandled MMIO or unregistered
ROM calls under both the JIT and interpreter:

```bash
ARDUINO_CLI=/path/to/arduino-cli ./test-rmt-tx.sh
# engine=jit stage=0x524D54A0 ... chunks=4 items=96 pattern=1 ... unhandled=0 unregistered=0
# engine=interp stage=0x524D54A0 ... chunks=4 items=96 pattern=1 ... unhandled=0 unregistered=0
```

`FLEXE_RMT_BUILD_DIR` optionally preserves its Arduino build directory.

The LEDC gate builds an unmodified sketch against ESP-IDF's public
`driver/ledc.h` API. It configures a 5 kHz, 8-bit PWM output on GPIO21,
validates boundary-latched duty updates from 64 to 192, then runs a blocking
fade from 192 to 96 through the production driver's ISR and semaphore path.
It also checks the host PWM boundary, output stop, and zero unhandled MMIO or
unregistered ROM calls under both the JIT and interpreter:

```bash
ARDUINO_CLI=/path/to/arduino-cli ./test-ledc-pwm.sh
# engine=jit stage=0x1EDCC0DE result=0/0/5000/64/.../192/.../96/... events=9 ... valid=1 unhandled=0 unregistered=0
# engine=interp stage=0x1EDCC0DE result=0/0/5000/64/.../192/.../96/... events=9 ... valid=1 unhandled=0 unregistered=0
```

`FLEXE_LEDC_BUILD_DIR` optionally preserves its Arduino build directory.

The sigma-delta gate builds an unmodified sketch through Arduino's public
`sigmaDelta*` wrapper and ESP-IDF's public `driver/sigmadelta.h` API. It
exercises channels 0 and 7, signed duty and prescale updates, GPIO-matrix
routing, GPIO enable state, the host aggregate-output callback, and an exact
192/256 live pulse-density period. It also guards the post-boot PLL/160 MHz
clock selectors used by genuine ESP-IDF clock queries. Both engines must
finish with zero unhandled MMIO or unregistered ROM calls:

```bash
ARDUINO_CLI=/path/to/arduino-cli ./test-sigmadelta.sh
# engine=jit stage=0x5349474D api=312500/192 regs=0040/07E0 routes=100/107 enable=00C40000 callbacks=11/1/1/1 density=192/256 unhandled=0 unregistered=0
# engine=interp stage=0x5349474D api=312500/192 regs=0040/07E0 routes=100/107 enable=00C40000 callbacks=11/1/1/1 density=192/256 unhandled=0 unregistered=0
```

The fixture is verified with Arduino-ESP32 2.0.11.
`FLEXE_ARDUINO_CONFIG` selects an Arduino CLI configuration and
`FLEXE_SIGMADELTA_BUILD_DIR` optionally preserves the fixture build.

The PCNT gate builds an unmodified sketch against ESP-IDF's public
`driver/pcnt.h` API. Host GPIO transitions pass through the production GPIO
matrix and ten-APB-cycle glitch filter into unit 0, while unit 7/channel 1
checks the upper signal-index range. The gate verifies control-input direction
reversal, pause/clear/resume, threshold and high-limit events, automatic limit
reset, and the real shared PCNT ISR service with zero unhandled MMIO or
unregistered ROM calls under both engines:

```bash
ARDUINO_CLI=/path/to/arduino-cli ./test-pcnt-pulse.sh
# engine=jit stage=0xC01A7E00 result=0/0/0/0/0/10/.../3/1/11/1/1/0/3/1/0 handled=1/1/1/1 unhandled=0 unregistered=0
# engine=interp stage=0xC01A7E00 result=0/0/0/0/0/10/.../3/1/11/1/1/0/3/1/0 handled=1/1/1/1 unhandled=0 unregistered=0
```

`FLEXE_PCNT_BUILD_DIR` optionally preserves its Arduino build directory.

The MCPWM gate builds an unmodified sketch against ESP-IDF's public legacy
`driver/mcpwm.h` API. Unit 0 drives complementary 20 kHz outputs through
carrier and dead-time blocks while unit 1 independently drives operator 2 at
1 kHz. The host exercises real GPIO-matrix sync, capture ISR callbacks,
cycle-by-cycle fault actions, forced high/low generator modes, shadowed
frequency/duty updates, and stop-at-TEZ behavior. Both engines must report all
three outputs stopped with zero unhandled MMIO or unregistered ROM calls:

```bash
ARDUINO_CLI=/path/to/arduino-cli ./test-mcpwm-motor.sh
# engine=jit stage=0x4D435057 result=0/.../10000/7500/0/0 events=19/18/9 ... levels=1/1/1/1 unhandled=0 unregistered=0
# engine=interp stage=0x4D435057 result=0/.../10000/7500/0/0 events=19/18/9 ... levels=1/1/1/1 unhandled=0 unregistered=0
```

`FLEXE_MCPWM_BUILD_DIR` optionally preserves its Arduino build directory.

The Timer Group gate builds an unmodified sketch against ESP-IDF's public
legacy `driver/timer.h` API. It runs both timers in both groups concurrently,
covering 64-bit capture/load, count-up and count-down modes, pause/resume,
one-shot alarm rescheduling, auto-reload, DPORT clock/reset control, and the
four genuine per-timer ISR callbacks. Both engines must finish with zero
unhandled MMIO or unregistered ROM calls:

```bash
ARDUINO_CLI=/path/to/arduino-cli ./test-timer-group.sh
# engine=jit stage=0x54494D47 result=0/.../11/4/12/6/.../10000/66/305419896/0 unhandled=0 unregistered=0
# engine=interp stage=0x54494D47 result=0/.../11/4/12/6/.../10000/66/305419896/0 unhandled=0 unregistered=0
```

`FLEXE_TIMER_GROUP_BUILD_DIR` optionally preserves its Arduino build directory.

The legacy FRC timer gate builds a sketch against Espressif's public
`soc/frc_timer_reg.h` definitions and installs genuine guest handlers with
`esp_intr_alloc`. FRC1 runs as a 23-bit APB countdown timer with level
interrupts and automatic reload; FRC2 crosses its 32-bit wrap point, pulses an
edge interrupt without a status clear, and advances each following compare
from its ISR. The gate also checks `/16` and `/256` prescalers, live count
readback, disabled-counter freeze behavior, and zero unhandled MMIO or
unregistered ROM calls under both engines:

```bash
ARDUINO_CLI=/path/to/arduino-cli ./test-frc-timer.sh
# engine=jit stage=0x46524332 result=0/0/3/80000/193/128/6/7/4097/.../133/10000/5000/5000/5000/136/100/412/... unhandled=0 unregistered=0
# engine=interp stage=0x46524332 result=0/0/3/80000/193/128/6/7/4097/.../133/10000/5000/5000/5000/136/100/412/... unhandled=0 unregistered=0
```

`FLEXE_FRC_TIMER_BUILD_DIR` optionally preserves its Arduino build directory.

The UHCI gate builds a sketch against Espressif's public `soc/uhci_reg.h`
register definitions and genuine `lldesc` DMA structures. It verifies both
independent UHCI controllers, transparent UART TX/RX descriptor chains,
descriptor ownership/writeback and diagnostics, exact wire-time completion,
real interrupt allocation, and host RX injection. Raw MMIO regressions also
cover H:5 headers, checksum/sequence/CRC handling, configurable SLIP escaping,
quick-send packets, idle/length/break EOF, DPORT reset domains, and malformed
descriptor failures. Both execution engines must finish with zero unhandled
MMIO or unregistered ROM calls:

```bash
ARDUINO_CLI=/path/to/arduino-cli ./test-uhci-dma.sh
# engine=jit stage=0x55484349 ... isr_raw=0x000021B0 ... unhandled=0 unregistered=0
# engine=interp stage=0x55484349 ... isr_raw=0x000021B0 ... unhandled=0 unregistered=0
```

`FLEXE_UHCI_BUILD_DIR` optionally preserves its Arduino build directory.

The SDIO-slave gate builds a sketch against Espressif's public
`driver/sdio_slave.h` API and runs the genuine driver, HAL/LL code, descriptor
queues, and source-10 ISR. The virtual host reads a 21-byte slave packet,
writes a 24-byte packet across two 16-byte receive descriptors, and exercises
the cumulative packet-length and receive-token counters. It also verifies the
HAL's RX-done software-ISR doorbell, IOREADY and DPORT reset/clock control, all
shared-register addressing gaps, and interrupts in both directions. Both
engines must complete with zero unhandled MMIO or unregistered ROM calls:

```bash
ARDUINO_CLI=/path/to/arduino-cli ./test-sdio-slave.sh
# engine=jit stage=0x5344494F ready=1 buffers=2 bytes=21 ... transfer=1/21/1/0 ... unhandled=0 unregistered=0
# engine=interp stage=0x5344494F ready=1 buffers=2 bytes=21 ... transfer=1/21/1/0 ... unhandled=0 unregistered=0
```

The fixture is verified with Arduino-ESP32 2.0.11.
`FLEXE_ARDUINO_CONFIG` selects an Arduino CLI configuration and
`FLEXE_SDIO_SLAVE_BUILD_DIR` optionally preserves the fixture build.

The native SDMMC gate builds a sketch against Espressif's public
`driver/sdmmc_host.h` API and runs the genuine ESP-IDF host driver without
intercepting `sdmmc_host_do_transaction`. It covers slot initialization and
clock-update commands, SD command/short/long responses, single-block reads and
writes, and 20 KiB multiblock transfers. The latter requires five 4 KiB
descriptors and therefore forces the stock driver's four-entry IDMAC ring to
recycle a descriptor through its real ISR and FreeRTOS queue path. Both engines
must reproduce the host-backed card contents with zero unhandled MMIO or
unregistered ROM calls. Raw regressions also verify that an existing backing
image exposes its actual capacity rather than its configured minimum:

```bash
ARDUINO_CLI=/path/to/arduino-cli ./test-sdmmc-host.sh
# engine=jit stage=0x53444D4D ... reads=2/41 writes=2/41 media=1 unhandled=0 unregistered=0
# engine=interp stage=0x53444D4D ... reads=2/41 writes=2/41 media=1 unhandled=0 unregistered=0
```

`FLEXE_SDMMC_BUILD_DIR` optionally preserves its Arduino build directory.

The TWAI gate builds a sketch against Espressif's public `driver/twai.h` API
and runs the genuine ESP-IDF driver, ISR, and FreeRTOS TX/RX queues. It covers
500 kbit/s timing, queued standard and extended transmission, single-shot self
reception, data and remote-frame host injection, alerts, counters, and clean
driver shutdown. Raw regressions additionally cover acceptance filtering,
64-byte FIFO overrun/release behavior, arbitration loss, automatic retry,
error-warning/passive transitions, bus-off recovery, and the DPORT reset
domain. Both engines must exchange the exact frames with zero unhandled MMIO
or unregistered ROM calls:

```bash
ARDUINO_CLI=/path/to/arduino-cli ./test-twai-bus.sh
# engine=jit stage=0x54574149 ... tx=3 frames=1 inject=1/1 pending=0 unhandled=0 unregistered=0
# engine=interp stage=0x54574149 ... tx=3 frames=1 inject=1/1 pending=0 unhandled=0 unregistered=0
```

`FLEXE_TWAI_BUILD_DIR` optionally preserves its Arduino build directory.

The Ethernet gates drive Flexe's classic ESP32 EMAC through two independent
compiled paths. `test-emac-hal.sh` uses Espressif's genuine `emac_hal_*` and
LL interrupt APIs; `test-emac-driver.sh` uses the public
`esp_eth_mac_new_esp32` object, its real ISR and notification-backed RX task,
mediator callbacks, and full init/link/deinit lifecycle. Together with the raw
MMIO regressions, they cover enhanced chained descriptor rings, multi-buffer
700-byte frames, exact TX capture, perfect and multicast RX filtering, FCS and
descriptor writeback, source-38 interrupts, DMA unavailable/error states,
DPORT clock/reset behavior, and Clause-22 PHY reads/writes. Both engines must
finish with zero unhandled MMIO or unregistered ROM calls:

```bash
ARDUINO_CLI=/path/to/arduino-cli ./test-emac-hal.sh
ARDUINO_CLI=/path/to/arduino-cli ./test-emac-driver.sh
# profile=hal engine=jit stage=0x454D4143 ... tx_ok=1 inject=1/1 irq=2 ... unhandled=0 unregistered=0
# profile=driver engine=interp stage=0x454D4143 ... tx_ok=1 inject=1/1 irq=2 ... unhandled=0 unregistered=0
```

The fixtures are verified with Arduino-ESP32 2.0.11. `FLEXE_ARDUINO_CONFIG`
selects an Arduino CLI configuration, while `FLEXE_EMAC_BUILD_DIR` and
`FLEXE_EMAC_DRIVER_BUILD_DIR` optionally preserve the two fixture builds.

Production ROMs are kept outside the repository. Run the sustained stock-ROM
correctness/performance gate by supplying either image (or both):

The current official CYD baselines are pinned below. Flexe fingerprints three
incompatible Marauder link layouts despite their shared `0x400831D8` entry
point: v1.14.0/1 use the original layout, v1.14.2/3 the shifted layout, and
v1.15.x a third one (flash moved by a per-library delta and `.bss` by 0x188).
An unknown image with that entry point is rejected instead of receiving unsafe
address-based ROM, Wi-Fi, or NimBLE hooks.

| release | official asset | SHA-256 |
|---|---|---|
| NerdMiner v1.8.3 | `ESP32-2432S028R_factory.bin` | `e7aece42f24ad7fd4146b94eeb28d04de7ce27f0c45e19be1bf38ad39ce0582c` |
| Marauder v1.14.0 | `esp32_marauder_v1_14_0_20260731_cyd_2432S028.bin` | `f2d21c476be70b7525a0b7c3122b4917fb5bcbeda78b16316b84450342d77fe7` |
| Marauder v1.14.1 | `esp32_marauder_v1_14_1_20260801_cyd_2432S028.bin` | `b3be0ff11ed4d67d8d763abb94eb22c2df2057adfdca58b780ac756ca20497d7` |
| Marauder v1.14.2 | `esp32_marauder_v1_14_2_20260815_cyd_2432S028.bin` | `5965e59f0e6f599eae213941d5ccc9d8d1dab1ebd7faa12b694cdff1a5cd3047` |
| Marauder v1.14.3 | `esp32_marauder_v1_14_3_20260816_cyd_2432S028.bin` | `ad91696012f407bf782826793edd509119acf00e4751cd0d30eddd6223d6bf2d` |
| Marauder v1.15.1 | `esp32_marauder_v1_15_1_20260824_cyd_2432S028.bin` | `72fa27948cd7f3bce4b6eabaaa8757b0d0e7854c534e8a502ce197d2397d899b` |
| Marauder v1.14.3 (2432S024 guition) | `esp32_marauder_v1_14_3_20260816_cyd_2432S024_guition.bin` | `6459db43b36b5d303485185e0fc9fa4e672c0409246592b9c955550fc3091a26` |
| Marauder v1.14.3 (3.5-inch) | `esp32_marauder_v1_14_3_20260816_cyd_3_5_inch.bin` | `968c1babf8b72c82a86e7e4cb3b86fcd4d619a67ad879aab02e7358f2a1a30d1` |

Every v1.15.1 address was relocated from the v1.14.3 profile by unique
masked-signature matching (L32R/CALL immediates wildcarded, since those move
with the literal pool), and the three NimBLE literal-pool entries and five DRAM
globals were confirmed independently. v1.15.1 passes the gate end to end under
both engines.

`tools/relocate_profile.py` is that relocation, made repeatable. It wildcards
the operand fields a relink moves, requires a *unique* match before reporting
an address, and resolves literal slots and DRAM globals through the L32R that
references them rather than by assuming a uniform section delta. Run it
against the reference image itself first: every address must come back
unchanged, which is what makes a nonzero delta elsewhere trustworthy.

    python3 tools/relocate_profile.py REFERENCE.bin TARGET.bin ADDR [ADDR ...]

Marauder v1.14.3 is also published for three other CYD boards, which are
separate links with their own entry point (`0x400830D0`). The 2432S024
(guition) and 3.5-inch builds have relocated profiles and pass the stock-ROM
gate end to end under both engines, exactly as the 2432S028 build does. The
`2432S028_2usb` build is a different code generation rather than a relink:
only 3 of its 41 profile addresses relocate from v1.14.3 and 10 from v1.15.1,
and its entry point (0x40081E90) matches neither. Signature relocation cannot
bridge that, so it is rejected as an unsupported layout rather than given
addresses that cannot be confirmed; it needs a profile built from its own
symbols.

```bash
MARAUDER_BIN=/path/to/marauder.bin \
NERDMINER_BIN=/path/to/nerdminer.bin \
./check-stock-roms.sh          # correctness
./bench-stock-roms.sh          # performance
```

`check-stock-roms.sh` runs each image through the scripted scenario on both
engines and requires them to agree, including on the final framebuffer. The
non-black pixel count the scenario already checks is sampled while the display
is still coming up, so it legitimately differs between engines and says
nothing about *what* was drawn; the framebuffer checksum is taken once the
scenario has converged and is identical across engines for every image above.
For the pinned images it is also held to a specific value, keyed by the ROM's
SHA-256, so a change to the display model shows up as a decision to make
rather than a silent difference.

The gate uses Flexe's reported per-core virtual cycles to compare elapsed
simulated time with wall time, so dual-core workloads are not mistakenly
credited twice. It rejects traps and early stops and defaults to requiring at
least 1.0× real-time averaged over three 2-billion-cycle runs. `EMU`,
`CYCLES`, `REPS`, `ENGINE`, `MIN_REALTIME`, and `ESP_HZ` are configurable.

It also rejects a run that spent most of its wall time *off*-CPU. A real-time
factor alone cannot tell emulating slowly apart from not emulating at all: a
50 ms poll per `accept()` once cost a NerdMiner run 35 of its 37 seconds
asleep while emulated time barely advanced, and every correctness gate passed
throughout. The window is 2 billion cycles rather than 1.2 because that is
what it takes to reach the phase where that happened.

Current Release-build results on Apple silicon (three default-length runs):

| stock CYD image | jit vs 240 MHz ESP32 |
|---|---:|
| ESP32 Marauder v1.14.3 | **8.50× real-time** |
| NerdMiner v1.8.3 | **5.86× real-time** |

Those figures are real-time factors, not throughput. `cycle_count` is elapsed
*simulated time*: it advances while a core is halted in `WAITI` and across the
scheduler's fast-forwards, because timers, peripherals and task wakeups are
all scheduled against it, and every `-c` budget is expressed in those units.
Stock images idle a great deal -- Marauder retires about 32M instructions per
100M simulated cycles -- so keeping up with real time and executing
instructions quickly are different claims. `insn_count` records the work, and
the emulator reports both:

```
Cycles:     200007053 (virtual: 100057053)
Insns:      32211607 retired (67845446 cycles idle)
```

For throughput use `bench-compute.sh`, which builds an in-repo fixture and
runs a fixed number of *rounds* -- identical work every run and on both
engines, rather than a cycle budget -- across a dependent scalar chain,
strided array traffic, SHA-256-shaped rotate/xor mixing, and byte-wide
load/store:

```bash
ARDUINO_CLI=/path/to/arduino-cli ./bench-compute.sh
# engine=interp rounds=12000 ... mips=176.7  realtime=0.74x checksum=0xC6B74AFC
# engine=jit    rounds=12000 ... mips=1458.5 realtime=6.08x checksum=0xC6B74AFC
# checksum=0xC6B74AFC (engines agree)  jit-speedup=8.25x
```

Both engines must agree on the checksum, so this is a JIT correctness check on
real compiler output as much as a measurement. `BENCH_ROUNDS`, `BENCH_REPS`
and `MIN_REALTIME` are configurable. MIPS counts retired instructions across
both cores, so it never credits idle time as work. On x86-64 Linux (GCC 15,
Release) a loop-heavy sketch reaches **1881 MIPS, 7.84× real-time**, against
304 MIPS interpreted.

`--jit-verify` runs every compiled block twice -- once natively, once through
the interpreter from the same starting state, with the interpreter's memory
effects rolled back in between -- and reports any block whose architectural
state or memory writes differ. It is roughly an order of magnitude slower and
is a debugging tool rather than a run mode, but it is what found the SRC
miscompile, the byte-store miscompile, and the LEND-spanning block that hung
two of the CYD board builds. The stock images verify clean over 9.4M blocks.

Chaining is switched off while verifying, and that is not a limitation worth
working around. The reference run replays the exact guest-instruction count
the native block reported, but a stub emulates a whole guest function in one
dispatch -- spending one instruction of that budget while standing in for many
-- and native code cannot enter ROM at all. Any window long enough to chain
crosses at least one stub, so skipping those cases removes all of the coverage
rather than the noise: measured across three stock ROMs, every single chained
block gets skipped. Block-level verification stays exact, and that chaining
preserves results is covered by the gates and by bench-compute's cross-engine
checksum instead.

The headless integration runner exercises display output and storage. The
Marauder profile drives touch navigation, submits `sniffraw` through the real
UART0 FIFO/interrupt path, feeds a valid NMEA fix through the independent
UART2 FIFO, and requires the production GPS parser and `gps -g lat` command to
return the expected latitude. It also verifies the GPS driver's real UART2
configuration exchange. The runner then delivers a beacon through the
production ROM's registered promiscuous callback; Marauder's own frame counters
must advance.
It then exits capture mode through a second UART command, launches the stock
Rick Roll attack, and requires its raw beacon frames to cross Flexe's host
virtual-radio transmit boundary. Finally, it starts `sniffbt`, injects a BLE
advertisement through the production NimBLE GAP handler, and requires
Marauder's registered device callback to parse and print `Device: FlexeBLE`.
It then submits `blespam -t windows`, lets the genuine NimBLE advertising
stack issue its controller HCI commands, and validates the emitted Microsoft
Swift Pair payload at Flexe's host BLE-radio boundary.
The NerdMiner profile connects to the stock ROM's captive portal through its
remapped host sockets, issues a real HTTP request and DNS query, and requires
valid responses from both:

```bash
./build/flexe-stock-rom-test nerdminer /path/to/nerdminer.bin
./build/flexe-stock-rom-test marauder /path/to/marauder.bin
```

## status

boots `hello_world`, `blink`, `tjpgd`, `real_time_stats`, `spi_lcd_touch` (lvgl!), and friends from esp-idf. runs them 2–10× faster than the real chip. isn't cycle-accurate and doesn't model cache timing or true simultaneous multicore cache coherence (both cores run, though).

## license

mit
