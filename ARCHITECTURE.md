# Architecture

Detailed design decisions and internal structure for the Xtensa LX6 emulator.

---

## 1. Execution Model

### 1.1 Interpreter Core

The emulator uses a **switch-based interpreter** as the execution engine. Each
iteration of the main loop:

1. Fetch instruction bytes from memory at PC
2. Determine length (2 or 3 bytes) by checking `op0 >= 8`
3. Extract fields from the instruction word
4. Dispatch to the instruction handler via nested `switch` on opcode fields
5. Execute the instruction (modify registers, memory, PC)
6. Check for loop-back (`PC == LEND && LCOUNT != 0`)
7. Check for pending interrupts
8. Advance cycle counter

```
xtensa_step(cpu):
    insn = fetch(cpu->pc)
    len = (op0 >= 8) ? 2 : 3
    next_pc = cpu->pc + len
    execute(cpu, insn)          // may modify next_pc
    loop_check(cpu, &next_pc)   // zero-overhead loop
    cpu->pc = next_pc
    cpu->ccount++
    interrupt_check(cpu)
```

### 1.2 Batched Execution

For performance, the public API exposes `xtensa_run(cpu, n_cycles)` which executes
up to `n_cycles` instructions before returning. This amortizes the per-call overhead
and allows the host to interleave display updates, input handling, etc.

```c
int xtensa_run(xtensa_cpu_t *cpu, int max_cycles) {
    for (int i = 0; i < max_cycles && cpu->running; i++) {
        xtensa_step(cpu);
    }
    return cpu->ccount;  /* cycles actually executed */
}
```

### 1.3 Future: Threaded Interpreter

GCC's computed-goto extension can eliminate the switch dispatch overhead:

```c
static void *dispatch_table[256] = { &&op_ADD, &&op_SUB, ... };
goto *dispatch_table[opcode];
op_ADD:
    ar[r] = ar[s] + ar[t];
    DISPATCH_NEXT;
```

This is a ~30% speedup but makes the code less readable. Defer until performance
is a measured bottleneck.

---

## 2. Memory Architecture

### 2.1 Address Space Layout

The ESP32 has a 32-bit address space. The emulator divides it into regions:

```
0x0000_0000 - 0x3EFF_FFFF  : Unmapped (fault on access)
0x3F00_0000 - 0x3F3F_FFFF  : External memory (PSRAM, if present)
0x3F40_0000 - 0x3F7F_FFFF  : External flash (data, cache-mapped)
0x3F80_0000 - 0x3FBF_FFFF  : External PSRAM (data)
0x3FF0_0000 - 0x3FF7_FFFF  : Peripheral registers (I/O)
0x3FF8_0000 - 0x3FFF_FFFF  : Internal SRAM (data bus view)
0x4000_0000 - 0x400C_1FFF  : Internal ROM + SRAM (instruction bus view)
0x400C_2000 - 0x40BF_FFFF  : External flash (instruction, cache-mapped)
0x5000_0000 - 0x5000_1FFF  : RTC FAST memory
0x6000_0000 - 0x6000_1FFF  : RTC SLOW memory
```

### 2.2 Implementation: Flat Array + Region Dispatch

Use a flat `uint8_t *mem` allocation for RAM/ROM regions and a dispatch function
for peripheral I/O:

```c
typedef struct {
    uint8_t *sram;         /* 520 KB internal SRAM at host base 0 */
    uint8_t *rom;          /* 384 KB internal ROM */
    uint8_t *flash;        /* External flash image (up to 16 MB) */
    uint8_t *rtc_fast;     /* 8 KB RTC FAST */
    uint8_t *rtc_slow;     /* 8 KB RTC SLOW */
    uint8_t *psram;        /* Optional external PSRAM (up to 4 MB) */

    /* Peripheral dispatch */
    peripheral_read_fn  periph_read;
    peripheral_write_fn periph_write;
    void *periph_ctx;
} xtensa_mem_t;
```

Address translation from ESP32 address to host pointer:

```c
uint8_t *mem_ptr(xtensa_mem_t *m, uint32_t addr) {
    if (addr >= 0x3FFB0000 && addr < 0x40000000)
        return m->sram + (addr - 0x3FFB0000);       /* SRAM data bus */
    if (addr >= 0x40070000 && addr < 0x400C2000)
        return m->sram + (addr - 0x40070000);        /* SRAM insn bus (alias) */
    if (addr >= 0x40000000 && addr < 0x40060000)
        return m->rom + (addr - 0x40000000);          /* ROM */
    if (addr >= 0x3F400000 && addr < 0x3F800000)
        return m->flash + (addr - 0x3F400000);        /* Flash data */
    if (addr >= 0x400C2000 && addr < 0x40C00000)
        return m->flash + (addr - 0x400C2000);        /* Flash insn */
    /* ... etc */
    return NULL;  /* unmapped -> exception */
}
```

Key detail: SRAM is accessible from both the data bus (0x3FFxxxxx) and instruction
bus (0x4007xxxx) at different addresses. Both map to the same physical memory. The
emulator uses a single `sram` buffer and translates both address ranges to it.

### 2.3 Peripheral Register Dispatch

Addresses in `0x3FF00000-0x3FF7FFFF` are peripheral registers. These are dispatched
to peripheral handler functions:

```c
uint32_t peripheral_read(void *ctx, uint32_t addr) {
    switch (addr & 0xFFFF0000) {
    case 0x3FF00000: return dport_read(addr);
    case 0x3FF40000: return uart_read(addr);
    case 0x3FF42000: return spi_read(addr, 0);
    case 0x3FF44000: return gpio_read(addr);
    case 0x3FF48000: return rtc_read(addr);
    case 0x3FF5A000: return efuse_read(addr);
    case 0x3FF5F000: return timer_read(addr, 0);
    case 0x3FF60000: return timer_read(addr, 1);
    /* ... */
    default:
        log_warn("Unhandled peripheral read: 0x%08X", addr);
        return 0;
    }
}
```

Unhandled peripheral accesses return 0 and log a warning. This allows iterative
development: run firmware, see what peripherals it accesses, add stubs.

### 2.4 Flash Image

The flash image is loaded from the .bin file and mapped at two address ranges:
- `0x3F400000` (data read via cache)
- `0x400C2000` (instruction fetch via cache)

Both map to the same underlying flash data. The emulator ignores cache behavior
and provides direct access (since we're not modeling timing).

---

## 3. Instruction Decode

### 3.1 Decode Strategy

Two-level decode using `op0` (4-bit) as the first dispatch, then format-specific
sub-dispatch:

```
op0 = 0: RRR format -> dispatch on op1, then op2
op0 = 1: L32R (RI16 format)
op0 = 2: RRI8 "LSAI" group -> dispatch on r field (loads/stores/ADDI/MOVI/branches)
op0 = 3: RRI8 "LSCI" group (FP loads/stores, if FP enabled)
op0 = 4: MAC16 group
op0 = 5: CALL format -> dispatch on n field (CALL0/4/8/12, J)
op0 = 6: BRI12 "SI" group -> dispatch on n,m fields (BEQZ/BNEZ/BGEZ/BLTZ/LOOP)
op0 = 7: BRI8 "B" group -> dispatch on r field (all 2-operand branches)
op0 = 8-15: Narrow (16-bit) instructions
```

### 3.2 Field Extraction Macros

```c
/* 24-bit instruction fields */
#define XT_OP0(i)    ((i) & 0xF)
#define XT_T(i)      (((i) >> 4) & 0xF)
#define XT_S(i)      (((i) >> 8) & 0xF)
#define XT_R(i)      (((i) >> 12) & 0xF)
#define XT_OP1(i)    (((i) >> 16) & 0xF)
#define XT_OP2(i)    (((i) >> 20) & 0xF)
#define XT_IMM8(i)   (((i) >> 16) & 0xFF)
#define XT_IMM12(i)  (((i) >> 12) & 0xFFF)
#define XT_IMM16(i)  (((i) >> 8) & 0xFFFF)
#define XT_SR(i)     (((i) >> 8) & 0xFF)

/* CALL format */
#define XT_N(i)      (((i) >> 4) & 0x3)
#define XT_OFFSET18(i) (((i) >> 6) & 0x3FFFF)

/* BRI8 format */
#define XT_M(i)      (((i) >> 6) & 0x3)

/* 16-bit narrow instruction fields */
#define XT_OP0_N(i)  ((i) & 0xF)
#define XT_T_N(i)    (((i) >> 4) & 0xF)
#define XT_S_N(i)    (((i) >> 8) & 0xF)
#define XT_R_N(i)    (((i) >> 12) & 0xF)
```

### 3.3 Sign Extension Helper

```c
static inline int32_t sign_extend(uint32_t val, int bits) {
    uint32_t sign_bit = 1u << (bits - 1);
    return (int32_t)((val ^ sign_bit) - sign_bit);
}
```

---

## 4. Register Window Emulation

### 4.1 Physical Register File

64 physical 32-bit registers, 16 visible at a time through the window:

```
Physical:  AR[0]  AR[1]  ...  AR[63]
                    ^--- windowbase * 4
Visible:   a0     a1     ...  a15
```

### 4.2 Window Overflow Detection

On `ENTRY`, check if the new window overlaps with an existing valid window:

```c
void window_check_overflow(xtensa_cpu_t *cpu, int callinc) {
    int new_wb = (cpu->windowbase + callinc) & 15;
    /* Check if windows at new_wb+0 through new_wb+3 are occupied */
    for (int i = 0; i < 4; i++) {
        int check = (new_wb + i) & 15;
        if (cpu->windowstart & (1 << check)) {
            /* Overlap detected: spill the overlapping window */
            window_spill(cpu, check);
        }
    }
}
```

### 4.3 Synthesized Spill Strategy

Rather than jumping to exception vectors (which requires ROM code), the emulator
directly implements the spill in C:

```c
void window_spill(xtensa_cpu_t *cpu, int window_idx) {
    /* Save a0-a3 to Base Save Area of callee's stack frame */
    int base = window_idx * 4;
    uint32_t sp = cpu->ar[(base + 1) & 63];  /* a1 of that window = its SP */
    for (int i = 0; i < 4; i++) {
        mem_write32(cpu->mem, sp - 16 + i*4, cpu->ar[(base + i) & 63]);
    }
    /* Clear windowstart bit */
    cpu->windowstart &= ~(1 << window_idx);
}
```

This is equivalent to what the ROM's WindowOverflow4 handler does, but without
the overhead of exception entry/exit. If firmware needs CALL8/CALL12 overflow
(which saves more registers to the Extra Save Area), the spill function handles
those cases too.

### 4.4 When to Use Full Exception Emulation

If a firmware image installs custom window exception handlers (writes to VECBASE
window overflow vector addresses), switch to full exception emulation:

1. Save EPC, EPS, set EXCCAUSE
2. Jump to VECBASE + WindowOverflow4/8/12 offset
3. Let the firmware's handler code run
4. Return via RFWO/RFWU

This is the fallback path. The synthesized path is the fast default.

---

## 5. Exception and Interrupt Architecture

### 5.1 Exception Dispatch

```
exception(cpu, cause):
    if (cpu->ps.excm):
        // Double exception
        cpu->depc = cpu->pc
        cpu->pc = cpu->vecbase + 0x1C0  // DoubleExceptionVector
    else:
        cpu->epc[1] = cpu->pc
        cpu->eps[1] = cpu->ps
        cpu->exccause = cause
        cpu->ps.excm = 1
        cpu->ps.intlevel = max(cpu->ps.intlevel, 1)  // mask level-1
        cpu->pc = vector_address(cpu, cause)
```

### 5.2 Vector Offsets (from VECBASE)

| Offset | Vector |
|--------|--------|
| 0x000  | WindowOverflow4 |
| 0x040  | WindowOverflow8 |
| 0x080  | WindowOverflow12 |
| 0x0C0  | WindowUnderflow4 |
| 0x100  | WindowUnderflow8 |
| 0x140  | WindowUnderflow12 |
| 0x180  | KernelExceptionVector |
| 0x1C0  | DoubleExceptionVector |
| 0x200  | UserExceptionVector |
| 0x240  | Level2InterruptVector |
| 0x280  | Level3InterruptVector |
| 0x2C0  | Level4InterruptVector |
| 0x300  | Level5InterruptVector |
| 0x340  | NMIExceptionVector (level 7) |

The reset vector is at a fixed address (0x40000400 for ESP32), not relative to VECBASE.

### 5.3 Interrupt Handling

Interrupts are checked between instructions. An interrupt is taken if:
1. `INTENABLE & INTERRUPT != 0` (interrupt is enabled)
2. The interrupt's level > `PS.INTLEVEL` (not masked)
3. `PS.EXCM == 0` (not already in exception handler — for level-1 only)

Timer interrupts: when `CCOUNT == CCOMPARE[n]`, set the corresponding interrupt
bit. The interrupt matrix routes this to a CPU interrupt line.

### 5.4 PS Register Detailed Layout

```
Bit 0-3:   INTLEVEL (current interrupt mask level, 0-15)
Bit 4:     EXCM     (exception mode, suppresses level-1 interrupts)
Bit 5:     UM       (user mode, 0=kernel 1=user)
Bit 6-7:   RING     (privilege ring, 0-3)
Bit 8-11:  OWB      (old window base, saved during exceptions)
Bit 12-15: (reserved)
Bit 16-17: CALLINC  (window increment for current call, 1/2/3 for CALL4/8/12)
Bit 18:    WOE      (window overflow enable)
```

---

## 6. Peripheral Model Architecture

### 6.1 Peripheral Interface

Each peripheral implements a standard interface:

```c
typedef struct {
    const char *name;
    uint32_t base_addr;
    uint32_t size;
    uint32_t (*read)(void *state, uint32_t offset);
    void (*write)(void *state, uint32_t offset, uint32_t value);
    void (*tick)(void *state, uint32_t cycles);  /* optional: periodic update */
    void (*reset)(void *state);
} peripheral_t;
```

The `tick` function is called periodically (not every cycle) for peripherals that
need time-based behavior (timers, UART TX timing, etc.).

### 6.2 SPI Device Bus

SPI peripherals (display, touch, flash) connect through a virtual SPI bus:

```c
typedef struct {
    uint8_t (*transfer)(void *dev, uint8_t mosi_byte);
    void (*cs_changed)(void *dev, bool active);
    void *device_state;
} spi_device_t;
```

The SPI controller model drives this interface based on register writes:
1. Firmware writes MOSI data to SPI registers
2. Firmware sets the "start" bit
3. Emulator calls `transfer()` on the selected device
4. MISO data (if any) is placed in read registers

### 6.3 Display Model

The ILI9341 model maintains:
- Current command state (waiting for command vs. data)
- Column/row address window (set by CASET/RASET)
- 320x240x16bpp framebuffer (RGB565)
- Current write position within the address window

On RAMWR data: pixels are written sequentially to the framebuffer at the current
position, advancing column-first then row, wrapping within the address window.

The framebuffer is exposed to the host via `xtensa_emu_get_framebuffer()` for
SDL rendering.

---

## 7. Integration API

### 7.1 Library API

```c
/* Lifecycle */
xtensa_emu_t *xtensa_emu_create(void);
void xtensa_emu_destroy(xtensa_emu_t *emu);
void xtensa_emu_reset(xtensa_emu_t *emu);

/* Loading */
int xtensa_emu_load_bin(xtensa_emu_t *emu, const char *firmware_path);
int xtensa_emu_load_elf(xtensa_emu_t *emu, const char *elf_path);

/* Execution */
int xtensa_emu_run(xtensa_emu_t *emu, int max_cycles);
void xtensa_emu_stop(xtensa_emu_t *emu);
bool xtensa_emu_is_running(xtensa_emu_t *emu);

/* Display */
const uint16_t *xtensa_emu_get_framebuffer(xtensa_emu_t *emu);
void xtensa_emu_get_display_size(xtensa_emu_t *emu, int *w, int *h);

/* Input */
void xtensa_emu_touch_event(xtensa_emu_t *emu, int x, int y, bool pressed);
void xtensa_emu_gpio_set(xtensa_emu_t *emu, int pin, bool level);

/* I/O */
typedef void (*xtensa_uart_cb)(void *ctx, const char *data, int len);
void xtensa_emu_set_uart_callback(xtensa_emu_t *emu, xtensa_uart_cb cb, void *ctx);

/* Debug */
void xtensa_emu_dump_regs(xtensa_emu_t *emu, FILE *out);
uint32_t xtensa_emu_get_pc(xtensa_emu_t *emu);
uint32_t xtensa_emu_read_reg(xtensa_emu_t *emu, int reg);
int xtensa_emu_disasm(xtensa_emu_t *emu, uint32_t addr, char *buf, int bufsize);
```

### 7.2 Thread Safety

The emulator is **not thread-safe internally**. The host application is responsible
for synchronization:

- Call `xtensa_emu_run()` from a dedicated emulator thread
- Use a mutex when calling `get_framebuffer()`, `touch_event()`, etc. from other threads
- The `stop()` function sets an atomic flag that `run()` checks each iteration

This matches cyd-emulator's existing pattern: app runs in a background thread,
main thread handles SDL events and rendering.

### 7.3 cyd-emulator Integration Pattern

```c
/* In emu_main.c */
void *emu_xtensa_thread(void *arg) {
    xtensa_emu_t *emu = (xtensa_emu_t *)arg;
    while (emu_running) {
        xtensa_emu_run(emu, 8000000);  /* ~33ms at 240MHz */
        /* Yield to allow display update */
        usleep(1000);
    }
    return NULL;
}

/* In render loop */
void render_xtensa_display(void) {
    const uint16_t *fb = xtensa_emu_get_framebuffer(emu);
    /* Convert RGB565 to SDL texture and render */
}

/* In event handler */
void handle_mouse_event(SDL_Event *e) {
    if (xtensa_mode) {
        xtensa_emu_touch_event(emu, x, y, pressed);
    }
}
```

---

## 8. Testing Strategy

### 8.1 Unit Tests

Each instruction category has a dedicated test file. Tests create a CPU, set up
registers, execute one instruction, and verify the result:

```c
void test_add(void) {
    xtensa_cpu_t cpu;
    xtensa_cpu_reset(&cpu);
    ar_write(&cpu, 4, 100);    /* a4 = 100 */
    ar_write(&cpu, 5, 200);    /* a5 = 200 */
    /* ADD a3, a4, a5:  op2=8 op1=0 r=3 s=4 t=5 op0=0 */
    execute_insn(&cpu, encode_rrr(0x8, 0x0, 3, 4, 5, 0x0));
    ASSERT_EQ(ar_read(&cpu, 3), 300);
}
```

### 8.2 Integration Tests

Load real .bin firmware images and verify behavior:
- **hello_world**: Boot to app_main, verify UART output contains "Hello world!"
- **blink**: Verify GPIO output toggles
- **display_test**: Verify framebuffer contains expected pixels

### 8.3 Differential Testing

For complex instructions (window operations, exceptions), compare behavior against
a reference implementation by running the same instruction sequence and comparing
register/memory state. Can use QEMU as a black-box reference (run same binary,
compare traces) without copying any code.

---

## 9. File Organization

```
xtensa-emulator/
  PLAN.md                    # High-level plan and milestones overview
  MILESTONES.md              # Detailed milestone acceptance criteria
  ARCHITECTURE.md            # This file
  CLAUDE.md                  # Project context for AI assistants
  TASKS-M0.md                # Granular tasks for milestone 0
  CMakeLists.txt             # Build system
  src/
    xtensa.h                 # Public API + CPU state
    xtensa.c                 # Instruction interpreter
    xtensa_decode.h          # Instruction field extraction (inline)
    memory.h                 # Memory subsystem API
    memory.c                 # Memory implementation
    loader.h                 # Firmware loader API
    loader.c                 # .bin + ELF loading
    window.h                 # Window overflow/underflow
    window.c
    exception.h              # Exception/interrupt dispatch
    exception.c
    interrupt.h              # Interrupt controller
    interrupt.c
    peripherals/
      peripheral.h           # Common peripheral interface
      dport.c                # System registers
      uart.c                 # UART console
      gpio.c                 # GPIO matrix
      spi.c                  # SPI controller
      timer.c                # TIMG watchdog/GP timers
      rtc.c                  # RTC_CNTL
      efuse.c                # eFuse block
      intmatrix.c            # Interrupt matrix
      display.c              # ILI9341 display model
      touch.c                # XPT2046 touch model
  tests/
    test_main.c              # Test runner entry point
    test_helpers.h           # Assert macros, CPU setup helpers
    test_alu.c               # Arithmetic instruction tests
    test_shift.c             # Shift instruction tests
    test_move.c              # Move instruction tests
    test_branch.c            # Branch instruction tests
    test_memory.c            # Load/store tests
    test_loader.c            # .bin parser tests
    test_loop.c              # Zero-overhead loop tests
    test_muldiv.c            # Multiply/divide tests
    test_misc.c              # Misc operations tests
    test_window.c            # Register window tests
    test_exception.c         # Exception handling tests
    test_interrupt.c         # Interrupt tests
    test_programs/           # Assembled test binaries
  tools/
    xt-dis.c                 # Standalone disassembler
```
