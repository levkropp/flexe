#ifndef XTENSA_H
#define XTENSA_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/* Forward declarations */
typedef struct xtensa_mem xtensa_mem_t;
typedef struct xtensa_cpu xtensa_cpu_t;

/* Classic ESP32 instruction-space geometry. The external cache has three
 * instruction buses after internal IRAM: IRAM0, IRAM1, and IROM0. */
#define ESP32_INSN_ADDR_LOW          0x40000000u
#define ESP32_FIRMWARE_INSN_ADDR_LOW 0x40070000u
#define ESP32_FLASH_INSN_ADDR_LOW    0x400D0000u
#define ESP32_INSN_ADDR_HIGH         0x40C00000u

/* Called when guest-visible instruction bytes change (flash-MMU remap,
 * self-programming flash, etc.). Execution engines can discard translated
 * code while the interpreter invalidates its predecode entries. */
typedef void (*xtensa_code_invalidate_fn)(void *ctx, uint32_t addr, size_t len);

/* PC hook: return zero to execute normally, or the number of guest
 * instructions handled. Ordinary ROM stubs return 1; accelerated blocks may
 * return more so the batch runner can account without rereading CCOUNT. */
typedef int (*xtensa_pc_hook_fn)(xtensa_cpu_t *cpu, uint32_t pc, void *ctx);

/* Pre-decoded instruction table: entire firmware decoded at load time.
 * Direct-indexed by (pc - PREDECODE_BASE), no tags, no cache misses.
 * Packed: bits 0-23 = instruction word, bits 24-25 = ilen (2 or 3).
 * 0 = invalid/not-decoded (unmapped memory).
 *
 * PREDECODE_END controls the flash size coverage. Benchmark with:
 *   -DPREDECODE_FLASH_MB=4  (default, covers 4MB flash = 20MB table)
 *   -DPREDECODE_FLASH_MB=2  (2MB flash = 12MB table)
 *   -DPREDECODE_FLASH_MB=8  (8MB flash = 36MB table)
 *   -DPREDECODE_FLASH_MB=16 (16MB flash = 68MB table)
 *   -DPREDECODE_FLASH_MB=0  (disabled — no predecode table)
 */
#define PREDECODE_BASE  0x40000000u

#ifndef PREDECODE_FLASH_MB
#define PREDECODE_FLASH_MB 4
#endif

#if PREDECODE_FLASH_MB > 0
/* PREDECODE_END = flash_base + flash_MB * 1048576 */
#define PREDECODE_END   (0x400D0000u + (PREDECODE_FLASH_MB * 0x100000u))
#define PREDECODE_SIZE  (PREDECODE_END - PREDECODE_BASE)
#else
#define PREDECODE_END   PREDECODE_BASE
#define PREDECODE_SIZE  0u
#endif

/* Packed: bits 0-23 = instruction word, bits 24-31 = ilen (2 or 3). */
#define PREDECODE_PACK(insn, ilen) ((insn) | ((uint32_t)(ilen) << 24))
#define PREDECODE_INSN(packed)     ((packed) & 0x00FFFFFFu)
#define PREDECODE_ILEN(packed)     ((packed) >> 24)

/*
 * Special Register Numbers (for RSR/WSR/XSR)
 */
#define XT_SR_LBEG          0
#define XT_SR_LEND          1
#define XT_SR_LCOUNT        2
#define XT_SR_SAR           3
#define XT_SR_BR            4
#define XT_SR_LITBASE       5
#define XT_SR_SCOMPARE1     12
#define XT_SR_ACCLO         16
#define XT_SR_ACCHI         17
#define XT_SR_MR0           32
#define XT_SR_MR1           33
#define XT_SR_MR2           34
#define XT_SR_MR3           35
#define XT_SR_WINDOWBASE    72
#define XT_SR_WINDOWSTART   73
#define XT_SR_PTEVADDR      83
#define XT_SR_RASID         90
#define XT_SR_ITLBCFG       91
#define XT_SR_DTLBCFG       92
#define XT_SR_IBREAKENABLE  96
#define XT_SR_MEMCTL        97
#define XT_SR_ATOMCTL       99
#define XT_SR_DDR           104
#define XT_SR_IBREAKA0      128
#define XT_SR_IBREAKA1      129
#define XT_SR_DBREAKA0      144
#define XT_SR_DBREAKA1      145
#define XT_SR_DBREAKC0      160
#define XT_SR_DBREAKC1      161
#define XT_SR_CONFIGID0     176
#define XT_SR_EPC1          177
#define XT_SR_EPC2          178
#define XT_SR_EPC3          179
#define XT_SR_EPC4          180
#define XT_SR_EPC5          181
#define XT_SR_EPC6          182
#define XT_SR_EPC7          183
#define XT_SR_DEPC          192
#define XT_SR_EPS2          194
#define XT_SR_EPS3          195
#define XT_SR_EPS4          196
#define XT_SR_EPS5          197
#define XT_SR_EPS6          198
#define XT_SR_EPS7          199
#define XT_SR_CONFIGID1     208
#define XT_SR_EXCSAVE1      209
#define XT_SR_EXCSAVE2      210
#define XT_SR_EXCSAVE3      211
#define XT_SR_EXCSAVE4      212
#define XT_SR_EXCSAVE5      213
#define XT_SR_EXCSAVE6      214
#define XT_SR_EXCSAVE7      215
#define XT_SR_CPENABLE      224
#define XT_SR_INTSET        226
#define XT_SR_INTCLEAR      227
#define XT_SR_INTENABLE     228
#define XT_SR_PS            230
#define XT_SR_VECBASE       231
#define XT_SR_EXCCAUSE      232
#define XT_SR_DEBUGCAUSE    233
#define XT_SR_CCOUNT        234
#define XT_SR_PRID          235
#define XT_SR_ICOUNT        236
#define XT_SR_ICOUNTLEVEL   237
#define XT_SR_EXCVADDR      238
#define XT_SR_CCOMPARE0     240
#define XT_SR_CCOMPARE1     241
#define XT_SR_CCOMPARE2     242
#define XT_SR_MISC0         244
#define XT_SR_MISC1         245
#define XT_SR_MISC2         246
#define XT_SR_MISC3         247

/*
 * Stop reason — why did execution end?
 */
typedef enum {
    STOP_RUNNING,           /* Still running (not stopped) */
    STOP_MAX_CYCLES,        /* Hit cycle limit */
    STOP_BREAKPOINT,        /* Hit a breakpoint address */
    STOP_HALT,              /* WAITI instruction, no wake */
    STOP_EXCEPTION_LOOP,    /* Same exception repeated at same PC */
    STOP_SOFTWARE_RESET,    /* software_reset ROM stub called */
    STOP_CPU_STOPPED,       /* cpu->running == false (other cause) */
} stop_reason_t;

/*
 * EXCCAUSE constants
 */
#define EXCCAUSE_ILLEGAL            0
#define EXCCAUSE_SYSCALL            1
#define EXCCAUSE_IFETCH_ERROR       2
#define EXCCAUSE_LOAD_STORE_ERROR   3
#define EXCCAUSE_LEVEL1_INT         4
#define EXCCAUSE_ALLOCA             5
#define EXCCAUSE_DIVIDE_BY_ZERO     6
#define EXCCAUSE_PRIVILEGED         8
#define EXCCAUSE_LOAD_STORE_ALIGN   9

/*
 * ESP32 vector offsets from VECBASE
 */
#define VECOFS_WINDOW_OVERFLOW4     0x000
#define VECOFS_WINDOW_UNDERFLOW4    0x040
#define VECOFS_WINDOW_OVERFLOW8     0x080
#define VECOFS_WINDOW_UNDERFLOW8    0x0C0
#define VECOFS_WINDOW_OVERFLOW12    0x100
#define VECOFS_WINDOW_UNDERFLOW12   0x140
#define VECOFS_LEVEL2_INT           0x180
#define VECOFS_LEVEL3_INT           0x1C0
#define VECOFS_LEVEL4_INT           0x200
#define VECOFS_LEVEL5_INT           0x240
#define VECOFS_DEBUG_EXC            0x280
#define VECOFS_NMI                  0x2C0
#define VECOFS_KERNEL_EXC           0x300
#define VECOFS_USER_EXC             0x340
#define VECOFS_DOUBLE_EXC           0x3C0

/*
 * Instruction field extraction macros (24-bit, little-endian)
 */
#define XT_OP0(i)       ((i) & 0xF)
#define XT_T(i)         (((i) >> 4) & 0xF)
#define XT_S(i)         (((i) >> 8) & 0xF)
#define XT_R(i)         (((i) >> 12) & 0xF)
#define XT_OP1(i)       (((i) >> 16) & 0xF)
#define XT_OP2(i)       (((i) >> 20) & 0xF)
#define XT_IMM8(i)      (((i) >> 16) & 0xFF)
#define XT_IMM12(i)     (((i) >> 12) & 0xFFF)
#define XT_IMM16(i)     (((i) >> 8) & 0xFFFF)
#define XT_SR_NUM(i)    (((i) >> 8) & 0xFF)  /* sr = s || r for RSR/WSR */

/* CALL format fields */
#define XT_N(i)         (((i) >> 4) & 0x3)
#define XT_OFFSET18(i)  (((i) >> 6) & 0x3FFFF)

/* BRI8 format */
#define XT_M(i)         (((i) >> 6) & 0x3)

/* 16-bit narrow instruction fields */
#define XT_OP0_N(i)     ((i) & 0xF)
#define XT_T_N(i)       (((i) >> 4) & 0xF)
#define XT_S_N(i)       (((i) >> 8) & 0xF)
#define XT_R_N(i)       (((i) >> 12) & 0xF)

/*
 * PS register field accessors
 */
#define XT_PS_INTLEVEL(ps)  ((ps) & 0xF)
#define XT_PS_EXCM(ps)      (((ps) >> 4) & 1)
#define XT_PS_UM(ps)        (((ps) >> 5) & 1)
#define XT_PS_RING(ps)      (((ps) >> 6) & 3)
#define XT_PS_OWB(ps)       (((ps) >> 8) & 0xF)
#define XT_PS_CALLINC(ps)   (((ps) >> 16) & 3)
#define XT_PS_WOE(ps)       (((ps) >> 18) & 1)

#define XT_PS_SET_CALLINC(ps, v) ((ps) = ((ps) & ~(3u << 16)) | (((v) & 3u) << 16))
#define XT_PS_SET_OWB(ps, v)     ((ps) = ((ps) & ~(0xFu << 8)) | (((v) & 0xFu) << 8))
#define XT_PS_SET_EXCM(ps, v)    ((ps) = ((ps) & ~(1u << 4)) | (((v) & 1u) << 4))
#define XT_PS_SET_INTLEVEL(ps, v) ((ps) = ((ps) & ~0xFu) | ((v) & 0xFu))
#define XT_PS_SET_UM(ps, v)       ((ps) = ((ps) & ~(1u << 5)) | (((v) & 1u) << 5))

/*
 * CPU State
 */
struct xtensa_cpu {
    /* ================================================================
     * HOT SECTION — fields accessed every instruction.
     * Packed into first cache lines to minimize L1 misses.
     * DO NOT add cold fields here without benchmarking.
     * ================================================================ */

    /* General-purpose registers: 64 physical, 16 visible via window */
    uint32_t ar[64];                    /* offset 0, 256 bytes, CL 0-3 */

    /* --- CL 4: decode/execute critical path --- */
    uint32_t pc;                        /* Program counter */
    uint32_t ccount;                    /* SR 234: Cycle counter */
    uint32_t next_timer_event;          /* Nearest ccompare value */
    uint32_t windowbase;                /* SR 72: Window base (0-15) */
    uint32_t ps;                        /* SR 230: Processor State */
    uint32_t sar;                       /* SR 3:  Shift Amount Register (6-bit) */
    uint32_t lbeg;                      /* SR 0:  Loop begin address */
    uint32_t lend;                      /* SR 1:  Loop end address */
    uint32_t lcount;                    /* SR 2:  Loop count */
    uint32_t intenable;                 /* SR 228: Interrupt enable mask */
    uint32_t interrupt;                 /* SR 226: Interrupt pending bits */
    uint32_t br;                        /* SR 4: 16 boolean bits, b0-b15 */
    bool     running;
    bool     halted;                    /* WAITI halt state */
    bool     exception;                 /* Exception pending flag */
    /* Set when an instruction modifies PC. 32-bit, not bool: the JIT stores
     * it with a 32-bit immediate store, which at a bool offset would also
     * overwrite the neighbouring irq_check and silently drop a pending
     * interrupt check on every block exit. */
    uint32_t _pc_written;
    /* Set by a JIT block whose scan was truncated at LEND, to tell its
     * fall-through exit (a genuine loop back-edge) from a side exit that
     * merely branches to LEND. Consumed and cleared by jit_pc_hook(). */
    uint32_t jit_loop_exit;
    /* Non-zero while guest_call8() is running guest code on a fabricated
     * register file (windowbase 0, its own stack, PS.INTLEVEL 15). The
     * FreeRTOS scheduler must not save that frame into a real task's TCB
     * or switch away from the in-flight call. */
    uint32_t in_guest_call;
    bool     irq_check;                 /* Set when interrupt/intenable changes */
    int      breakpoint_count;

    /* --- CL 5-6: pointers, cycle count, window --- */
    /* Elapsed simulated cycles. This is *time*, not work: it also advances
     * while the core is halted in WAITI and across the scheduler's
     * fast-forwards, because peripherals, timers and task wakeups are all
     * scheduled against it. Every -c budget and every harness cycle limit is
     * expressed in these units. */
    uint64_t cycle_count;
    /* Guest instructions this core actually retired. Kept separately because
     * cycle_count cannot answer the question: it advances while the core is
     * halted, and flexe_session_post_batch() publishes the *maximum* of the
     * two cores' values so both share one timeline. Throughput has to be
     * measured in these units -- crediting idle time as work would let a
     * firmware that mostly sleeps report a MIPS figure it never achieved.
     * Updated once per xtensa_run() batch, so the per-instruction path is
     * untouched. */
    uint64_t insn_count;
    xtensa_mem_t *mem;                  /* Memory subsystem */
    xtensa_pc_hook_fn pc_hook;          /* PC hook (for ROM stubs etc.) */
    void             *pc_hook_ctx;
    const uint64_t   *pc_hook_bitmap;   /* Fast-path bitmap: skip hook if bit not set */
    uint32_t         *predecode;        /* Pre-decoded instruction table (heap, 20MB) */
    /* AOT static-recompiled function table.
     * `aot` is opaque to xtensa.c; xtensa_step_impl just calls
     * `aot_lookup_fn(aot, pc)` which returns a function pointer or NULL.
     * Hits run native C; misses fall through to the interpreter. */
    void             *aot;
    void           *(*aot_lookup_fn)(void *aot, uint32_t pc);
    const uint64_t   *aot_bitmap;       /* Fast-path: skip aot_lookup if bit not set */
    uint32_t windowstart;               /* SR 73: Bitmask of valid windows */

    /* ================================================================
     * WARM SECTION — accessed on branches, exceptions, some instructions
     * ================================================================ */

    /* Exception state */
    uint32_t vecbase;           /* SR 231: Exception vector base */
    uint32_t exccause;          /* SR 232: Exception cause */
    uint32_t excvaddr;          /* SR 238: Exception virtual address */
    uint32_t epc[7];            /* SR 177-183: Exception PCs (levels 1-7) */
    uint32_t eps[7];            /* SR 194-200: Exception PS (levels 2-7, index 0 unused) */
    uint32_t excsave[7];        /* SR 209-215: Exception save regs */
    uint32_t depc;              /* SR 192: Double exception PC */

    /* Timers */
    uint32_t ccompare[3];       /* SR 240-242: Cycle compare */

    /* Misc special registers */
    uint32_t scompare1;         /* SR 12: Compare value for S32C1I */
    uint32_t misc[4];           /* SR 244-247: Misc scratch registers */
    uint32_t litbase;           /* SR 5:  Literal base (Extended L32R) */
    uint32_t atomctl;           /* SR 99: Atomic operation control */
    uint32_t memctl;            /* SR 97: Memory control */
    uint32_t icount;            /* SR 236: Instruction count */
    uint32_t icountlevel;       /* SR 237: Icount exception level */
    uint32_t debugcause;        /* SR 233: Debug exception cause */
    uint32_t ibreakenable;      /* SR 96: Instruction breakpoint enable */
    uint32_t ibreaka[2];        /* SR 128-129: Instruction breakpoints */
    uint32_t dbreaka[2];        /* SR 144-145: Data breakpoints */
    uint32_t dbreakc[2];        /* SR 160-161: Data breakpoint control */
    uint32_t configid0;         /* SR 176: Processor config ID 0 */
    uint32_t configid1;         /* SR 208: Processor config ID 1 */
    uint32_t prid;              /* SR 235: Processor ID */
    int      core_id;           /* Core ID (0=PRO_CPU, 1=APP_CPU) */
    uint32_t cpenable;          /* SR 224: Coprocessor enable */

    /* MAC16 Option state */
    uint32_t acclo;             /* SR 16: Accumulator low */
    uint32_t acchi;             /* SR 17: Accumulator high (8-bit) */
    uint32_t mr[4];             /* SR 32-35: MAC16 data registers */

    /* Floating-point coprocessor */
    uint32_t fcr;               /* Floating-point control */
    uint32_t fsr;               /* Floating-point status */
    float    fr[16];            /* Floating-point registers */

    /* Execution control */
    bool     debug_break;       /* Debug break requested */
    bool     window_trace;      /* Emit window spill/fill/ENTRY/RETW trace to stderr */
    bool     window_trace_active; /* Set by main loop to gate window trace */
    bool     spill_verify;      /* Enable spill/fill verification */
    bool     accelerated_blocks; /* PC hook/AOT may execute >1 guest insn */
    uint64_t virtual_time_us;   /* Simulated wall-clock microseconds */

    /* Interrupt configuration */
    uint8_t  int_level[32];     /* Interrupt level per line (default: 1) */

    /* Breakpoints */
#define MAX_BREAKPOINTS 16
    uint32_t breakpoints[MAX_BREAKPOINTS];
    bool     breakpoint_hit;
    uint32_t breakpoint_hit_addr;

    /* Debug: PC history ring buffer */
#define PC_HISTORY_SIZE 32
    uint32_t pc_history[PC_HISTORY_SIZE];
    int      pc_history_idx;

    /* ================================================================
     * COLD SECTION — large arrays, rarely accessed per-instruction.
     * Kept at end so they don't pollute cache lines of hot fields.
     * ================================================================ */

    /* Window spill base stack: each window slot can be spilled multiple times
     * (when the window ring wraps around). We track all spill locations so
     * underflow fill uses the correct base for each nesting level.
     * Extra registers (a4-a7 for CALL8, a4-a11 for CALL12) are stored in
     * CPU-side buffers rather than on the stack, to avoid corruption when
     * deeper calls overwrite the stack memory. */
#define SPILL_STACK_DEPTH 32
    struct {
        uint32_t base[SPILL_STACK_DEPTH];        /* stack of spill bases */
        uint32_t core[SPILL_STACK_DEPTH][4];     /* core[depth][0..3] = a0-a3 values */
        uint32_t extra[SPILL_STACK_DEPTH][8];    /* extra[depth][0..7] = a4-a11 values */
        int depth;               /* current stack depth */
    } spill_stack[16];
    uint32_t spill_base[16]; /* legacy: most recent base per window (for -W trace) */

    /* Spill verification: shadow copies to detect stack corruption */
    struct {
        uint32_t regs[12];   /* saved register values (a0-a11 max) */
        uint32_t base;       /* base address used during spill */
        int count;           /* number of regs saved (4, 8, or 12) */
    } spill_shadow[16];

    /* Per-slot callsize of the window currently occupying it (1=call4,
     * 2=call8, 3=call12), recorded at ENTRY. A window's callsize is a
     * property of its creation call, not of its a0 (tail-called frames can
     * have a0=0), so it must be tracked here for correct spill/fill gating. */
    uint8_t window_callsize[16];
    uint32_t dbg_prev_pc;   /* previous step PC, for trap diagnosis */

    /* Guest-instruction accumulator for a single chained JIT run.  ARM64
     * keeps this in a callee-saved register (X27); x86-64 has no free
     * register (R8-R13 are guest a1-a6, R14/R15 are mem/cpu, and the rest
     * are body scratch), so it accumulates here and the epilogue loads the
     * total into EAX.  Transient: written only between a block prologue and
     * its epilogue, so it is deliberately not serialized in savestates. */
    uint32_t jit_acc;

    /* Peripheral timer-event hooks (TIMG LACT): fold peripheral alarms into
     * the next_timer_event mechanism so alarm interrupts fire on time and
     * can wake the core from WAITI. */
    void *periph_event_ctx;
    uint32_t (*periph_next_event)(xtensa_cpu_t *cpu);
    void (*periph_event)(xtensa_cpu_t *cpu);

    /* Optional execution-engine code-cache invalidation hook. */
    xtensa_code_invalidate_fn code_invalidate;
    void *code_invalidate_ctx;
};

/*
 * Inline register access helpers
 * Maps architectural register an to physical register via window
 */
static inline uint32_t ar_read(const xtensa_cpu_t *cpu, int n) {
    return cpu->ar[((cpu->windowbase * 4) + n) & 63];
}

static inline void ar_write(xtensa_cpu_t *cpu, int n, uint32_t val) {
    cpu->ar[((cpu->windowbase * 4) + n) & 63] = val;
}

/*
 * Sign extension helper
 */
static inline int32_t sign_extend(uint32_t val, int bits) {
    uint32_t sign_bit = 1u << (bits - 1);
    val &= (sign_bit << 1) - 1;  /* mask to 'bits' width */
    return (int32_t)((val ^ sign_bit) - sign_bit);
}

/*
 * Instruction fetch: read 2 or 3 bytes from memory, return instruction word and length.
 * Returns instruction length (2 or 3), or 0 on error.
 */
int xtensa_fetch(const xtensa_cpu_t *cpu, uint32_t addr, uint32_t *insn_out);

/*
 * Special register access helpers for RSR/WSR/XSR
 */
uint32_t sr_read(const xtensa_cpu_t *cpu, int sr);
void     sr_write(xtensa_cpu_t *cpu, int sr, uint32_t val);

/*
 * Public API
 */
void xtensa_cpu_init(xtensa_cpu_t *cpu);

/* Guest instructions this core retired. See cycle_count / insn_count. */
static inline uint64_t xtensa_retired_insns(const xtensa_cpu_t *cpu) {
    return cpu->insn_count;
}
void xtensa_cpu_reset(xtensa_cpu_t *cpu);
void xtensa_predecode_build(xtensa_cpu_t *cpu);  /* Pre-decode instruction memory */
void xtensa_invalidate_code(xtensa_cpu_t *cpu, uint32_t addr, size_t len);
int  xtensa_step(xtensa_cpu_t *cpu);
int  xtensa_run(xtensa_cpu_t *cpu, int max_cycles);
int  xtensa_disasm(const xtensa_cpu_t *cpu, uint32_t addr, char *buf, int bufsize);

/*
 * Exception/Interrupt support
 */
void xtensa_raise_exception(xtensa_cpu_t *cpu, int cause, uint32_t fault_pc, uint32_t vaddr);
void xtensa_check_interrupts(xtensa_cpu_t *cpu);
void xtensa_flush_windows(xtensa_cpu_t *cpu);
void xtensa_recompute_next_timer(xtensa_cpu_t *cpu);

/* Raise any ccompare/peripheral timer interrupts whose ccount has arrived and
 * recompute next_timer_event.  Callers that move ccount forward themselves
 * (the FreeRTOS idle fast-forward) must invoke this at each event boundary,
 * or every interrupt scheduled inside the skipped window is silently lost. */
void xtensa_fire_due_timers(xtensa_cpu_t *cpu);

/*
 * Breakpoint support
 */
int  xtensa_set_breakpoint(xtensa_cpu_t *cpu, uint32_t addr);
int  xtensa_clear_breakpoint(xtensa_cpu_t *cpu, uint32_t addr);
void xtensa_clear_all_breakpoints(xtensa_cpu_t *cpu);

/*
 * JIT execution loop (defined in jit.c, used by main.c with -J flag).
 * Runs hot blocks as compiled native code, falling back to interpreter
 * for cold code and complex instructions.
 */
struct jit_state;
int xtensa_run_jit(struct jit_state *jit, xtensa_cpu_t *cpu, int max_cycles);

#endif /* XTENSA_H */
