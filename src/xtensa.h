#ifndef XTENSA_H
#define XTENSA_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/* Forward declaration */
typedef struct xtensa_mem xtensa_mem_t;

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
typedef struct {
    /* General-purpose registers: 64 physical, 16 visible via window */
    uint32_t ar[64];

    /* Program counter */
    uint32_t pc;

    /* Special registers */
    uint32_t sar;           /* SR 3:  Shift Amount Register (6-bit) */
    uint32_t lbeg;          /* SR 0:  Loop begin address */
    uint32_t lend;          /* SR 1:  Loop end address */
    uint32_t lcount;        /* SR 2:  Loop count */
    uint32_t windowbase;    /* SR 72: Window base (0-15) */
    uint32_t windowstart;   /* SR 73: Bitmask of valid windows */
    uint32_t ps;            /* SR 230: Processor State */
    uint32_t vecbase;       /* SR 231: Exception vector base */
    uint32_t exccause;      /* SR 232: Exception cause */
    uint32_t excvaddr;      /* SR 238: Exception virtual address */
    uint32_t epc[7];        /* SR 177-183: Exception PCs (levels 1-7) */
    uint32_t eps[7];        /* SR 194-200: Exception PS (levels 2-7, index 0 unused) */
    uint32_t excsave[7];    /* SR 209-215: Exception save regs */
    uint32_t depc;          /* SR 192: Double exception PC */
    uint32_t ccount;        /* SR 234: Cycle counter */
    uint32_t ccompare[3];   /* SR 240-242: Cycle compare */
    uint32_t scompare1;     /* SR 12: Compare value for S32C1I */
    uint32_t misc[4];       /* SR 244-247: Misc scratch registers */
    uint32_t litbase;       /* SR 5:  Literal base (Extended L32R) */
    uint32_t atomctl;       /* SR 99: Atomic operation control */
    uint32_t memctl;        /* SR 97: Memory control */
    uint32_t icount;        /* SR 236: Instruction count */
    uint32_t icountlevel;   /* SR 237: Icount exception level */
    uint32_t debugcause;    /* SR 233: Debug exception cause */
    uint32_t ibreakenable;  /* SR 96: Instruction breakpoint enable */
    uint32_t ibreaka[2];    /* SR 128-129: Instruction breakpoints */
    uint32_t dbreaka[2];    /* SR 144-145: Data breakpoints */
    uint32_t dbreakc[2];    /* SR 160-161: Data breakpoint control */
    uint32_t configid0;     /* SR 176: Processor config ID 0 */
    uint32_t configid1;     /* SR 208: Processor config ID 1 */
    uint32_t prid;          /* SR 235: Processor ID */
    uint32_t intenable;     /* SR 228: Interrupt enable mask */
    uint32_t interrupt;     /* SR 226: Interrupt pending bits */
    uint32_t cpenable;      /* SR 224: Coprocessor enable */

    /* Boolean registers (Boolean Option) */
    uint32_t br;            /* SR 4: 16 boolean bits, b0-b15 */

    /* MAC16 Option state */
    uint32_t acclo;         /* SR 16: Accumulator low */
    uint32_t acchi;         /* SR 17: Accumulator high (8-bit) */
    uint32_t mr[4];         /* SR 32-35: MAC16 data registers */

    /* Floating-point coprocessor */
    uint32_t fcr;           /* Floating-point control */
    uint32_t fsr;           /* Floating-point status */
    float    fr[16];        /* Floating-point registers */

    /* Interrupt configuration */
    uint8_t  int_level[32]; /* Interrupt level per line (default: 1) */

    /* Execution state */
    bool     running;
    bool     exception;     /* Exception pending flag */
    bool     debug_break;   /* Debug break requested */
    bool     halted;        /* WAITI halt state */
    uint64_t cycle_count;   /* Total emulated cycles */

    /* Memory subsystem (set by caller) */
    xtensa_mem_t *mem;
} xtensa_cpu_t;

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
void xtensa_cpu_reset(xtensa_cpu_t *cpu);
int  xtensa_step(xtensa_cpu_t *cpu);
int  xtensa_run(xtensa_cpu_t *cpu, int max_cycles);
int  xtensa_disasm(const xtensa_cpu_t *cpu, uint32_t addr, char *buf, int bufsize);

/*
 * Exception/Interrupt support
 */
void xtensa_raise_exception(xtensa_cpu_t *cpu, int cause, uint32_t fault_pc, uint32_t vaddr);
void xtensa_check_interrupts(xtensa_cpu_t *cpu);

#endif /* XTENSA_H */
