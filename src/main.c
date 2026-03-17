#ifdef _MSC_VER
#include "msvc_compat.h"
#endif

#include "flexe_session.h"
#include "xtensa.h"
#include "memory.h"
#include "peripherals.h"
#include "rom_stubs.h"
#include "elf_symbols.h"
#include "freertos_stubs.h"
#include "savestate.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <ctype.h>
#ifndef _MSC_VER
#include <getopt.h>
#endif
#include <pthread.h>

/* Provided by cyd-emulator's emu_touch.c in --firmware mode;
 * standalone binary always keeps this set to 1. */
volatile int emu_app_running = 1;

/* ===== Configuration ===== */

#define MAX_BP_ARGS     16
#define MAX_MEM_DUMPS   8
#define EXC_LOOP_THRESH 3
#define MAX_COND_FUNCS  8
#define MAX_COND_RANGES 8
#define MAX_ASSERTIONS  16

typedef struct {
    uint32_t addr;
    uint32_t len;
} mem_dump_t;

/* Forward declarations */
static void format_addr(const elf_symbols_t *syms, uint32_t addr, char *buf, int bufsize);

/* ===== Ring-buffer trace (Feature 4) ===== */

typedef struct {
    char **lines;
    int capacity;
    int head;
    int count;
    uint64_t total;
} ring_buf_t;

static ring_buf_t *ring_create(int capacity) {
    ring_buf_t *rb = calloc(1, sizeof(ring_buf_t));
    if (!rb) return NULL;
    rb->lines = calloc((size_t)capacity, sizeof(char *));
    if (!rb->lines) { free(rb); return NULL; }
    for (int i = 0; i < capacity; i++) {
        rb->lines[i] = malloc(512);
        if (!rb->lines[i]) {
            for (int j = 0; j < i; j++) free(rb->lines[j]);
            free(rb->lines); free(rb); return NULL;
        }
        rb->lines[i][0] = '\0';
    }
    rb->capacity = capacity;
    return rb;
}

static void ring_push(ring_buf_t *rb, const char *line) {
    int idx = (rb->head + rb->count) % rb->capacity;
    if (rb->count == rb->capacity) {
        idx = rb->head;
        rb->head = (rb->head + 1) % rb->capacity;
    } else {
        rb->count++;
    }
    size_t len = strlen(line);
    if (len > 511) len = 511;
    memcpy(rb->lines[idx], line, len);
    rb->lines[idx][len] = '\0';
    rb->total++;
}

static void ring_flush(ring_buf_t *rb) {
    if (rb->total > (uint64_t)rb->count)
        fprintf(stderr, "\n--- Ring buffer: last %d of %llu trace lines ---\n",
                rb->count, (unsigned long long)rb->total);
    else
        fprintf(stderr, "\n--- Ring buffer: %d trace lines ---\n", rb->count);
    for (int i = 0; i < rb->count; i++) {
        int idx = (rb->head + i) % rb->capacity;
        fputs(rb->lines[idx], stderr);
    }
}

static void ring_flush_tail(ring_buf_t *rb, int n) {
    if (n <= 0 || rb->count == 0) return;
    if (n > rb->count) n = rb->count;
    int skip = rb->count - n;
    fprintf(stderr, "\n--- Ring buffer: last %d of %llu trace lines ---\n",
            n, (unsigned long long)rb->total);
    for (int i = skip; i < rb->count; i++) {
        int idx = (rb->head + i) % rb->capacity;
        fputs(rb->lines[idx], stderr);
    }
}

static void ring_destroy(ring_buf_t *rb) {
    if (!rb) return;
    for (int i = 0; i < rb->capacity; i++) free(rb->lines[i]);
    free(rb->lines);
    free(rb);
}

/* Global ring buffer pointer — always non-NULL when tracing */
static ring_buf_t *g_ring = NULL;

/* Dump condition flags */
#define DUMP_CRASH  1   /* dump ring buffer on panic/exception stop */
#define DUMP_FLUSH  2   /* flush entire ring buffer to stderr on exit */
#define DUMP_TAIL   4   /* dump last N lines on exit */
#define DEFAULT_RING_SIZE 50000

static int g_dump_mode = 0;
static int g_dump_tail_n = 0;

static void trace_emit(const char *fmt, ...) {
    char buf[512];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (g_ring)
        ring_push(g_ring, buf);
    else
        fputs(buf, stderr);
}

/* ===== Conditional trace (Feature 2) ===== */

typedef struct {
    struct { char name[128]; uint32_t addr, end; int resolved; } funcs[MAX_COND_FUNCS];
    int func_count;
    uint64_t after_cycle;
    struct { uint32_t lo, hi; } ranges[MAX_COND_RANGES];
    int range_count;
    char until_name[128];
    uint32_t until_addr;
    int until_resolved;
    int until_triggered;
    int active;
} cond_trace_t;

static void cond_trace_resolve(cond_trace_t *ct, const elf_symbols_t *syms) {
    if (!syms) return;
    for (int i = 0; i < ct->func_count; i++) {
        if (ct->funcs[i].resolved) continue;
        uint32_t addr;
        if (elf_symbols_find(syms, ct->funcs[i].name, &addr) == 0) {
            elf_sym_info_t info;
            ct->funcs[i].addr = addr;
            if (elf_symbols_lookup(syms, addr, &info) && info.size > 0)
                ct->funcs[i].end = addr + info.size;
            else
                ct->funcs[i].end = addr + 0x10000; /* fallback: 64K */
            ct->funcs[i].resolved = 1;
            fprintf(stderr, "Conditional trace: func '%s' resolved to 0x%08X-0x%08X\n",
                    ct->funcs[i].name, ct->funcs[i].addr, ct->funcs[i].end);
        } else {
            fprintf(stderr, "Warning: cannot resolve conditional trace func '%s'\n",
                    ct->funcs[i].name);
        }
    }
    if (ct->until_name[0] && !ct->until_resolved) {
        if (elf_symbols_find(syms, ct->until_name, &ct->until_addr) == 0) {
            ct->until_resolved = 1;
            fprintf(stderr, "Conditional trace: until '%s' resolved to 0x%08X\n",
                    ct->until_name, ct->until_addr);
        } else {
            fprintf(stderr, "Warning: cannot resolve 'until:%s'\n", ct->until_name);
        }
    }
}

static int cond_trace_active(const cond_trace_t *ct, uint32_t pc, uint64_t cycle) {
    if (ct->until_triggered) return 0;
    if (ct->after_cycle > 0 && cycle < ct->after_cycle) return 0;
    if (ct->func_count > 0) {
        int in_func = 0;
        for (int i = 0; i < ct->func_count; i++) {
            if (ct->funcs[i].resolved && pc >= ct->funcs[i].addr && pc < ct->funcs[i].end) {
                in_func = 1;
                break;
            }
        }
        if (!in_func) return 0;
    }
    if (ct->range_count > 0) {
        int in_range = 0;
        for (int i = 0; i < ct->range_count; i++) {
            if (pc >= ct->ranges[i].lo && pc <= ct->ranges[i].hi) {
                in_range = 1;
                break;
            }
        }
        if (!in_range) return 0;
    }
    return 1;
}

static void cond_trace_check_until(cond_trace_t *ct, uint32_t pc) {
    if (ct->until_resolved && !ct->until_triggered && pc == ct->until_addr) {
        ct->until_triggered = 1;
        trace_emit("     [COND] until:%s triggered, tracing stopped\n", ct->until_name);
    }
}

static int parse_cond(cond_trace_t *ct, const char *arg) {
    if (strncmp(arg, "func:", 5) == 0) {
        if (ct->func_count >= MAX_COND_FUNCS) return -1;
        strncpy(ct->funcs[ct->func_count].name, arg + 5, 127);
        ct->funcs[ct->func_count].name[127] = '\0';
        ct->func_count++;
    } else if (strncmp(arg, "after:", 6) == 0) {
        ct->after_cycle = strtoull(arg + 6, NULL, 0);
    } else if (strncmp(arg, "range:", 6) == 0) {
        if (ct->range_count >= MAX_COND_RANGES) return -1;
        char *dash = strchr(arg + 6, '-');
        if (!dash) return -1;
        ct->ranges[ct->range_count].lo = (uint32_t)strtoul(arg + 6, NULL, 16);
        ct->ranges[ct->range_count].hi = (uint32_t)strtoul(dash + 1, NULL, 16);
        ct->range_count++;
    } else if (strncmp(arg, "until:", 6) == 0) {
        strncpy(ct->until_name, arg + 6, 127);
        ct->until_name[127] = '\0';
    } else {
        return -1;
    }
    ct->active = 1;
    return 0;
}

/* ===== Function-call trace (Feature 3) ===== */

typedef enum {
    INSN_OTHER = 0,
    INSN_CALL,
    INSN_CALLX,
    INSN_ENTRY,
    INSN_RET,
    INSN_RETW,
} insn_type_t;

static insn_type_t classify_insn(const xtensa_cpu_t *cpu) {
    uint32_t insn;
    int len = xtensa_fetch(cpu, cpu->pc, &insn);
    if (len <= 0) return INSN_OTHER;

    if (len == 2) {
        /* Narrow instructions: RET.N (op0=13, r=15, t=0), RETW.N (op0=13, r=15, t=1) */
        uint8_t op0 = insn & 0xF;
        uint8_t r = (insn >> 12) & 0xF;
        uint8_t t = (insn >> 4) & 0xF;
        if (op0 == 0xD && r == 0xF) {
            if (t == 0) return INSN_RET;
            if (t == 1) return INSN_RETW;
        }
        return INSN_OTHER;
    }

    /* 3-byte instructions */
    uint8_t op0 = insn & 0xF;

    /* CALL4/8/12: op0=5 */
    if (op0 == 5) return INSN_CALL;

    /* CALLX4/8/12, RET, RETW, ENTRY all have op0=0 */
    if (op0 == 0) {
        uint8_t op1 = (insn >> 16) & 0xF;
        uint8_t op2 = (insn >> 20) & 0xF;
        uint8_t r = (insn >> 12) & 0xF;
        uint8_t m = (insn >> 6) & 0x3;
        uint8_t n = (insn >> 4) & 0x3;

        /* CALLX: op0=0, op1=0, op2=0, r!=0, m=3 */
        if (op1 == 0 && op2 == 0 && m == 3 && r != 0) return INSN_CALLX;

        /* RET: op0=0, op1=0, op2=0, r=0, m=2, n=0 */
        if (op1 == 0 && op2 == 0 && r == 0 && m == 2 && n == 0) return INSN_RET;

        /* RETW: op0=0, op1=0, op2=0, r=0, m=2, n=1 */
        if (op1 == 0 && op2 == 0 && r == 0 && m == 2 && n == 1) return INSN_RETW;

        /* ENTRY: op0=6 in the n field... actually ENTRY is op0=6, let me fix */
    }

    /* ENTRY: op0=6, subop n=3, m=0 — wait, ENTRY encoding: op0=0b0110 = 6 */
    if (op0 == 6) return INSN_ENTRY;

    return INSN_OTHER;
}

/* ===== Trace assertions (Feature 5) ===== */

typedef enum {
    ASSERT_REG_EQ,
    ASSERT_PC_EQ,
    ASSERT_MEM_EQ,
} assert_type_t;

typedef struct {
    assert_type_t type;
    int reg_idx;        /* 0-15 for a0-a15 */
    uint32_t addr;      /* for PC or MEM assertions */
    uint32_t value;
    char desc[64];
} trace_assert_t;

static int parse_assert(trace_assert_t *ta, const char *arg) {
    /* pc=0xADDR */
    if (strncmp(arg, "pc=", 3) == 0) {
        ta->type = ASSERT_PC_EQ;
        ta->value = (uint32_t)strtoul(arg + 3, NULL, 16);
        snprintf(ta->desc, sizeof(ta->desc), "pc==0x%08X", ta->value);
        return 0;
    }
    /* mem:ADDR=VAL */
    if (strncmp(arg, "mem:", 4) == 0) {
        ta->type = ASSERT_MEM_EQ;
        char *eq = strchr(arg + 4, '=');
        if (!eq) return -1;
        ta->addr = (uint32_t)strtoul(arg + 4, NULL, 16);
        ta->value = (uint32_t)strtoul(eq + 1, NULL, 16);
        snprintf(ta->desc, sizeof(ta->desc), "mem:0x%08X==0x%X", ta->addr, ta->value);
        return 0;
    }
    /* aNN=VAL */
    if (arg[0] == 'a' && isdigit((unsigned char)arg[1])) {
        char *eq = strchr(arg, '=');
        if (!eq) return -1;
        ta->type = ASSERT_REG_EQ;
        ta->reg_idx = atoi(arg + 1);
        if (ta->reg_idx < 0 || ta->reg_idx > 15) return -1;
        ta->value = (uint32_t)strtoul(eq + 1, NULL, 0);
        snprintf(ta->desc, sizeof(ta->desc), "a%d==0x%X", ta->reg_idx, ta->value);
        return 0;
    }
    return -1;
}

static void check_assertions(const trace_assert_t *asserts, int count,
                              const xtensa_cpu_t *cpu, const elf_symbols_t *syms,
                              uint64_t cycle, xtensa_mem_t *mem) {
    for (int i = 0; i < count; i++) {
        int triggered = 0;
        switch (asserts[i].type) {
        case ASSERT_REG_EQ:
            triggered = (ar_read(cpu, asserts[i].reg_idx) == asserts[i].value);
            break;
        case ASSERT_PC_EQ:
            triggered = (cpu->pc == asserts[i].value);
            break;
        case ASSERT_MEM_EQ:
            triggered = (mem_read32(mem, asserts[i].addr) == asserts[i].value);
            break;
        }
        if (triggered) {
            char sym_buf[128];
            format_addr(syms, cpu->pc, sym_buf, sizeof(sym_buf));
            fprintf(stderr, "[ASSERT] cycle=%llu pc=%s: %s\n",
                    (unsigned long long)cycle, sym_buf, asserts[i].desc);
        }
    }
}

/* ===== UART stdout callback ===== */

static void uart_stdout_cb(void *ctx, uint8_t byte) {
    (void)ctx;
    putchar(byte);
}

/* ===== Exception cause names ===== */

static const char *exc_cause_name(uint32_t cause) {
    static const char *names[] = {
        [0] = "ILL", [1] = "SYSCALL", [2] = "IFETCH_ERR", [3] = "LD_ST_ERR",
        [4] = "L1_INT", [5] = "ALLOCA", [6] = "DIV_ZERO",
        [8] = "PRIV", [9] = "ALIGN",
    };
    if (cause < sizeof(names)/sizeof(names[0]) && names[cause])
        return names[cause];
    return "?";
}

/* ===== Stop reason to string ===== */

static const char *stop_reason_str(stop_reason_t reason) {
    switch (reason) {
    case STOP_RUNNING:        return "running";
    case STOP_MAX_CYCLES:     return "max_cycles";
    case STOP_BREAKPOINT:     return "breakpoint";
    case STOP_HALT:           return "halt (WAITI)";
    case STOP_EXCEPTION_LOOP: return "exception_loop";
    case STOP_SOFTWARE_RESET: return "software_reset";
    case STOP_CPU_STOPPED:    return "cpu_stopped";
    }
    return "unknown";
}

/* ===== Address formatting with symbols ===== */

static void format_addr(const elf_symbols_t *syms, uint32_t addr, char *buf, int bufsize) {
    elf_sym_info_t info;
    if (syms && elf_symbols_lookup(syms, addr, &info)) {
        if (info.offset == 0)
            snprintf(buf, bufsize, "%s", info.name);
        else
            snprintf(buf, bufsize, "%s+0x%x", info.name, info.offset);
    } else {
        snprintf(buf, bufsize, "0x%08X", addr);
    }
}

/* ===== ROM stub log callback for verbose trace ===== */

static void rom_log_cb(void *ctx, uint32_t addr, const char *name,
                        const xtensa_cpu_t *cpu) {
    (void)ctx; (void)addr; (void)cpu;
    trace_emit("     [ROM] %s @ 0x%08X\n", name, addr);
}

/* ===== ROM stub log callback for call trace ===== */

static const elf_symbols_t *g_call_syms = NULL;
static int g_call_depth = 0;

static void rom_log_call_cb(void *ctx, uint32_t addr, const char *name,
                             const xtensa_cpu_t *cpu) {
    (void)ctx; (void)addr; (void)cpu;
    int indent = g_call_depth < 128 ? g_call_depth : 128;
    fprintf(stderr, "%*s-> [ROM] %s (0x%08X)\n", indent * 2, "", name, addr);
}

/* ===== Event log callbacks ===== */

static void rom_log_event_cb(void *ctx, uint32_t addr, const char *name,
                              const xtensa_cpu_t *cpu) {
    (void)ctx; (void)addr;
    fprintf(stderr, "[%10llu] STUB  %s\n",
            (unsigned long long)cpu->cycle_count, name);
}

static void freertos_event_cb(const char *from, const char *to,
                               uint64_t cycle, void *ctx) {
    (void)ctx;
    fprintf(stderr, "[%10llu] SCHED %s → %s\n",
            (unsigned long long)cycle, from ? from : "?", to ? to : "?");
}

/* ===== Hex dump ===== */

static void hexdump(xtensa_mem_t *mem, uint32_t addr, uint32_t len) {
    for (uint32_t off = 0; off < len; off += 16) {
        fprintf(stderr, "%08X: ", addr + off);
        uint8_t line[16];
        int n = (len - off >= 16) ? 16 : (int)(len - off);
        for (int i = 0; i < n; i++) {
            line[i] = mem_read8(mem, addr + off + (uint32_t)i);
            fprintf(stderr, "%02X ", line[i]);
            if (i == 7) fprintf(stderr, " ");
        }
        for (int i = n; i < 16; i++) {
            fprintf(stderr, "   ");
            if (i == 7) fprintf(stderr, " ");
        }
        fprintf(stderr, " |");
        for (int i = 0; i < n; i++)
            fprintf(stderr, "%c", (line[i] >= 0x20 && line[i] <= 0x7E) ? line[i] : '.');
        fprintf(stderr, "|\n");
    }
}

/* ===== Usage ===== */

static void usage(const char *prog) {
    fprintf(stderr, "Usage: %s [options] <firmware.bin>\n", prog);
    fprintf(stderr, "  -1              Single-core mode (no APP_CPU)\n");
    fprintf(stderr, "  -c <cycles>     Max cycles (default: 10000000)\n");
    fprintf(stderr, "  -t              Instruction trace to stderr\n");
    fprintf(stderr, "  -T              Verbose trace (reg changes, ROM calls, exceptions)\n");
    fprintf(stderr, "  -v              Verbose register dump on exit\n");
    fprintf(stderr, "  -q              Quiet: suppress per-access unhandled peripheral warnings\n");
    fprintf(stderr, "  -e <addr>       Override entry point (hex)\n");
    fprintf(stderr, "  -s <file.elf>   Load ELF symbols for trace/breakpoints\n");
    fprintf(stderr, "  -b <addr|name>  Set breakpoint (repeatable)\n");
    fprintf(stderr, "  -m <addr[:len]> Dump memory on exit (repeatable, default len=256)\n");
    fprintf(stderr, "  -W              Window trace (spill/fill/ENTRY/RETW events)\n");
    fprintf(stderr, "  -S <file.img>   SD card backing image file\n");
    fprintf(stderr, "  -Z <bytes>      SD card size (default: auto from file, or 4GB)\n");
    fprintf(stderr, "  -C <cond>       Conditional trace (repeatable, implies -T):\n");
    fprintf(stderr, "                    func:NAME  — only trace inside function\n");
    fprintf(stderr, "                    after:N    — start tracing after virtual cycle N\n");
    fprintf(stderr, "                    range:A-B  — only trace when PC in hex range\n");
    fprintf(stderr, "                    until:NAME — trace until function entered\n");
    fprintf(stderr, "  -F              Function-call trace (CALL/RET tree)\n");
    fprintf(stderr, "  -B <N>          Ring-buffer size (default: 50000 when -T used)\n");
    fprintf(stderr, "  -D <mode>       Dump condition (repeatable, default: crash):\n");
    fprintf(stderr, "                    crash    — dump ring buffer on panic/exception\n");
    fprintf(stderr, "                    flush    — flush entire ring buffer on exit\n");
    fprintf(stderr, "                    tail:N   — dump last N lines on exit\n");
    fprintf(stderr, "  -A <cond>       Trace assertion (repeatable):\n");
    fprintf(stderr, "                    a6=0       — log when register equals value\n");
    fprintf(stderr, "                    pc=0xADDR  — log when PC hits address\n");
    fprintf(stderr, "                    mem:ADDR=V — log when memory equals value\n");
    fprintf(stderr, "  -E              Event log (stubs, task switches, exceptions)\n");
    fprintf(stderr, "  -P <N>          Progress heartbeat every N cycles (default: 1000000)\n");
    fprintf(stderr, "  -V              Verify window spill/fill integrity (detect stack corruption)\n");
    fprintf(stderr, "\nCheckpoint options:\n");
    fprintf(stderr, "  --checkpoint-interval <N>   Auto-save checkpoint every N cycles\n");
    fprintf(stderr, "  --checkpoint-dir <PATH>     Directory for checkpoint files (default: .)\n");
    fprintf(stderr, "  --restore <FILE>            Restore from checkpoint and resume execution\n");
}

/* ===== Parse -m argument ===== */

static int parse_mem_dump(const char *arg, mem_dump_t *out) {
    char *colon = strchr(arg, ':');
    if (colon) {
        out->addr = (uint32_t)strtoul(arg, NULL, 16);
        out->len = (uint32_t)strtoul(colon + 1, NULL, 0);
    } else {
        out->addr = (uint32_t)strtoul(arg, NULL, 16);
        out->len = 256;
    }
    return 0;
}

/* ===== Detect interrupt vector entry ===== */

static const char *detect_vector(uint32_t pc, uint32_t vecbase) {
    uint32_t off = pc - vecbase;
    switch (off) {
    case 0x000: return "WindowOverflow4";
    case 0x040: return "WindowUnderflow4";
    case 0x080: return "WindowOverflow8";
    case 0x0C0: return "WindowUnderflow8";
    case 0x100: return "WindowOverflow12";
    case 0x140: return "WindowUnderflow12";
    case 0x180: return "Level2Int";
    case 0x1C0: return "Level3Int";
    case 0x200: return "Level4Int";
    case 0x240: return "Level5Int";
    case 0x280: return "DebugExc";
    case 0x2C0: return "NMI";
    case 0x300: return "KernelExc";
    case 0x340: return "UserExc";
    case 0x3C0: return "DoubleExc";
    default:    return NULL;
    }
}

/* ===== Main ===== */

int main(int argc, char *argv[]) {
    long long max_cycles = 10000000;
    int trace = 0;
    int verbose_trace = 0;
    int window_trace = 0;
    int verbose = 0;
    int quiet_unhandled = 0;
    int call_trace = 0;
    int ring_size = 0;
    uint32_t entry_override = 0;
    int has_entry_override = 0;
    const char *elf_path = NULL;
    const char *sdcard_path = NULL;
    uint64_t sdcard_size = 0;
    const char *bp_args[MAX_BP_ARGS];
    int bp_count = 0;
    mem_dump_t mem_dumps[MAX_MEM_DUMPS];
    int dump_count = 0;
    cond_trace_t cond = {0};
    trace_assert_t asserts[MAX_ASSERTIONS];
    int assert_count = 0;
    int event_log = 0;
    int spill_verify = 0;
    int single_core = 0;
    uint64_t heartbeat_interval = 0;
    uint64_t trace_start = 0, trace_end = UINT64_MAX;
    /* Checkpoint options */
    const char *checkpoint_dir = NULL;
    uint64_t checkpoint_interval = 0;
    const char *restore_file = NULL;

    /* Manual parsing for long options (--checkpoint-*, --restore) */
    int i = 1;
    while (i < argc) {
        if (strcmp(argv[i], "--checkpoint-dir") == 0 && i + 1 < argc) {
            checkpoint_dir = argv[i + 1];
            /* Remove these args from argv */
            memmove(&argv[i], &argv[i + 2], (size_t)(argc - i - 1) * sizeof(char *));
            argc -= 2;
            continue;
        } else if (strcmp(argv[i], "--checkpoint-interval") == 0 && i + 1 < argc) {
            checkpoint_interval = strtoull(argv[i + 1], NULL, 0);
            memmove(&argv[i], &argv[i + 2], (size_t)(argc - i - 1) * sizeof(char *));
            argc -= 2;
            continue;
        } else if (strcmp(argv[i], "--restore") == 0 && i + 1 < argc) {
            restore_file = argv[i + 1];
            memmove(&argv[i], &argv[i + 2], (size_t)(argc - i - 1) * sizeof(char *));
            argc -= 2;
            continue;
        }
        i++;
    }

    int opt;
    while ((opt = getopt(argc, argv, "1c:tT::WVvqe:s:b:m:S:Z:C:FB:A:EP:D:")) != -1) {
        switch (opt) {
        case '1': single_core = 1; break;
        case 'c': max_cycles = strtoll(optarg, NULL, 10); break;
        case 't': trace = 1; break;
        case 'T':
            verbose_trace = 1; trace = 1;
            if (optarg && strchr(optarg, ':')) {
                char *colon = strchr(optarg, ':');
                trace_start = strtoull(optarg, NULL, 0);
                trace_end = strtoull(colon + 1, NULL, 0);
            }
            break;
        case 'W': window_trace = 1; trace = 1; break;
        case 'V': spill_verify = 1; break;
        case 'v': verbose = 1; break;
        case 'q': quiet_unhandled = 1; break;
        case 'e':
            entry_override = (uint32_t)strtoul(optarg, NULL, 16);
            has_entry_override = 1;
            break;
        case 's': elf_path = optarg; break;
        case 'S': sdcard_path = optarg; break;
        case 'Z': sdcard_size = strtoull(optarg, NULL, 0); break;
        case 'b':
            if (bp_count < MAX_BP_ARGS) bp_args[bp_count++] = optarg;
            break;
        case 'm':
            if (dump_count < MAX_MEM_DUMPS) parse_mem_dump(optarg, &mem_dumps[dump_count++]);
            break;
        case 'C':
            if (parse_cond(&cond, optarg) != 0) {
                fprintf(stderr, "Invalid -C condition: %s\n", optarg);
                return 1;
            }
            verbose_trace = 1; trace = 1;
            break;
        case 'F':
            call_trace = 1; trace = 1;
            break;
        case 'B':
            ring_size = atoi(optarg);
            if (ring_size <= 0) {
                fprintf(stderr, "Invalid -B size: %s\n", optarg);
                return 1;
            }
            break;
        case 'D':
            if (strcmp(optarg, "crash") == 0)
                g_dump_mode |= DUMP_CRASH;
            else if (strcmp(optarg, "flush") == 0)
                g_dump_mode |= DUMP_FLUSH;
            else if (strncmp(optarg, "tail:", 5) == 0) {
                g_dump_mode |= DUMP_TAIL;
                g_dump_tail_n = atoi(optarg + 5);
            } else {
                fprintf(stderr, "Invalid -D mode: %s\n", optarg);
                return 1;
            }
            break;
        case 'A':
            if (assert_count < MAX_ASSERTIONS) {
                if (parse_assert(&asserts[assert_count], optarg) != 0) {
                    fprintf(stderr, "Invalid -A assertion: %s\n", optarg);
                    return 1;
                }
                assert_count++;
            }
            trace = 1; /* force per-step mode */
            break;
        case 'E': event_log = 1; break;
        case 'P': heartbeat_interval = strtoull(optarg, NULL, 0); break;
        default: usage(argv[0]); return 1;
        }
    }

    /* -T with optional space-separated START:END — if -T was given bare
       and the next non-option arg looks like a trace range, consume it. */
    if (verbose_trace && trace_start == 0 && optind < argc) {
        char *maybe = argv[optind];
        char *colon = strchr(maybe, ':');
        if (colon && colon != maybe) {
            /* Check both sides are numeric */
            char *endA, *endB;
            uint64_t a = strtoull(maybe, &endA, 0);
            uint64_t b = strtoull(colon + 1, &endB, 0);
            if (endA == colon && *endB == '\0' && a < b) {
                trace_start = a;
                trace_end = b;
                optind++; /* consume this arg */
            }
        }
    }

    /* -C after:N implies a trace window start — use batch execution
       until we reach the conditional trace activation cycle. */
    if (cond.active && cond.after_cycle > 0 && trace_start < cond.after_cycle)
        trace_start = cond.after_cycle;

    if (optind >= argc) {
        usage(argv[0]);
        return 1;
    }
    const char *firmware = argv[optind];

    /* Set up ring buffer: always used when -T is active.
     * -B N overrides default size.  Without -B, default to 50K lines. */
    if (verbose_trace && ring_size == 0)
        ring_size = DEFAULT_RING_SIZE;
    if (ring_size > 0) {
        g_ring = ring_create(ring_size);
        if (!g_ring) {
            fprintf(stderr, "Failed to allocate ring buffer (%d slots)\n", ring_size);
            return 1;
        }
        /* Default dump mode: crash if none specified */
        if (g_dump_mode == 0)
            g_dump_mode = DUMP_CRASH;
    }

    /* Create emulator session (loads firmware, creates all stubs) */
    flexe_session_config_t sess_cfg = {
        .bin_path = firmware,
        .elf_path = elf_path,
        .sdcard_path = sdcard_path,
        .sdcard_size = sdcard_size,
        .entry_override = has_entry_override ? entry_override : 0,
        .single_core = single_core,
        .window_trace = window_trace,
        .spill_verify = spill_verify,
        .uart_cb = uart_stdout_cb,
    };
    flexe_session_t *session = flexe_session_create(&sess_cfg);
    if (!session) {
        fprintf(stderr, "Failed to create emulator session\n");
        ring_destroy(g_ring);
        return 1;
    }

    /* Pull out pointers for use in the execution loop */
    xtensa_cpu_t *cpu = flexe_session_cpu(session, 0);
    xtensa_mem_t *mem = flexe_session_mem(session);
    const elf_symbols_t *syms = flexe_session_syms(session);
    esp32_periph_t *periph = flexe_session_periph(session);
    esp32_rom_stubs_t *rom = flexe_session_rom(session);
    freertos_stubs_t *frt = flexe_session_frt(session);

    /* Restore from checkpoint if requested (overrides firmware load) */
    if (restore_file) {
        fprintf(stderr, "Restoring from checkpoint: %s\n", restore_file);
        if (savestate_restore(cpu, frt, restore_file) != 0) {
            fprintf(stderr, "Failed to restore checkpoint\n");
            flexe_session_destroy(session);
            ring_destroy(g_ring);
            return 1;
        }
        fprintf(stderr, "Checkpoint restored successfully!\n");
        fprintf(stderr, "  Cycle:      %llu\n", (unsigned long long)cpu->cycle_count);
        fprintf(stderr, "  Core 0 PC:  0x%08X\n", cpu->pc);

        if (frt) {
            const char *task_c0 = freertos_stubs_current_task_name(frt, 0);
            const char *task_c1 = freertos_stubs_current_task_name(frt, 1);
            fprintf(stderr, "  Core 0 task:%s\n", task_c0 ? task_c0 : "(none)");
            fprintf(stderr, "  Core 1 task:%s\n", task_c1 ? task_c1 : "(none)");

            if (freertos_stubs_scheduler_active(frt)) {
                fprintf(stderr, "  Scheduler:  ACTIVE\n");
            } else {
                fprintf(stderr, "  Scheduler:  not started\n");
            }
        }

        /* No manual timer/interrupt reconfiguration needed.
         * The checkpoint was saved during active task execution - FreeRTOS scheduler
         * state has been restored and tasks will continue from their saved state.
         * Note: Core 0 may remain in vTaskStartScheduler loop if all tasks are pinned
         * to Core 1, which is normal for dual-core firmware. */
    }

    /* Window trace gating: session sets window_trace_active = window_trace,
     * but if we have a trace window start, disable until we reach it. */
    if (window_trace && trace_start > 0)
        cpu->window_trace_active = false;

    /* Install ROM log callback for verbose trace, call trace, or event log */
    if (call_trace) {
        g_call_syms = syms;
        flexe_session_set_rom_log_cb(session, rom_log_call_cb, NULL);
    } else if (event_log) {
        flexe_session_set_rom_log_cb(session, rom_log_event_cb, NULL);
    } else if (verbose_trace) {
        flexe_session_set_rom_log_cb(session, rom_log_cb, NULL);
    }

    /* Install event log callbacks */
    if (event_log) {
        flexe_session_set_freertos_event_fn(session, freertos_event_cb, NULL);
        flexe_session_set_event_log(session, 1);
    }

    /* Resolve conditional trace symbols */
    if (cond.active)
        cond_trace_resolve(&cond, syms);

    /* Resolve and install breakpoints */
    for (int i = 0; i < bp_count; i++) {
        const char *arg = bp_args[i];
        uint32_t bp_addr;
        if (arg[0] == '0' && (arg[1] == 'x' || arg[1] == 'X')) {
            bp_addr = (uint32_t)strtoul(arg, NULL, 16);
        } else if (isdigit((unsigned char)arg[0])) {
            bp_addr = (uint32_t)strtoul(arg, NULL, 16);
        } else {
            /* Symbol name */
            if (!syms || elf_symbols_find(syms, arg, &bp_addr) != 0) {
                fprintf(stderr, "Warning: cannot resolve breakpoint '%s' (no symbols?)\n", arg);
                continue;
            }
        }
        if (xtensa_set_breakpoint(cpu, bp_addr) == 0) {
            char sym_buf[128];
            format_addr(syms, bp_addr, sym_buf, sizeof(sym_buf));
            fprintf(stderr, "Breakpoint set at 0x%08X (%s)\n", bp_addr, sym_buf);
        }
    }

    /* ===== Execute ===== */
    uint64_t cycles = 0;
    uint64_t max_cycles_u64 = (uint64_t)(max_cycles > 0 ? max_cycles : 10000000);
    stop_reason_t stop_reason = STOP_RUNNING;
    char disasm_buf[128];

    /* Exception loop detection */
    uint32_t last_exc_pc = 0;
    uint32_t last_exc_cause = 0xFFFFFFFF;
    int exc_repeat = 0;

    /* Heartbeat state */
    uint64_t last_heartbeat = 0;
    uint32_t hb_stub_count = 0;

    /* Checkpoint state */
    uint64_t next_checkpoint_cycle = checkpoint_interval;

    /* Trace state (used in trace windows) */
    uint32_t prev_ar[16];
    uint32_t prev_sar = 0, prev_ps = 0;
    uint32_t prev_pc = cpu->pc;
    int call_depth = 0;

    /* Determine if we need per-step execution at all */
    int need_step = trace || call_trace || assert_count;

    /* Unified execution loop */
    int batch = 10000;
    while (cycles < max_cycles_u64 && cpu->running && !cpu->halted && !cpu->breakpoint_hit) {
        /* Are we in a trace window?
         * Use cpu->cycle_count (virtual time including FreeRTOS skips)
         * so trace windows match event log timestamps. */
        uint64_t ccnt = cpu->cycle_count;
        /* Update window trace gating */
        if (window_trace)
            cpu->window_trace_active = (trace_start == 0 || ccnt >= trace_start) &&
                                      (ccnt < trace_end);
        int in_trace_window = need_step &&
            (trace_start == 0 || ccnt >= trace_start) &&
            (ccnt < trace_end);

        if (in_trace_window) {
            /* --- Single-step with full trace output --- */
            int do_trace_output = (verbose_trace || (!call_trace && !assert_count));
            if (do_trace_output && cond.active)
                do_trace_output = cond_trace_active(&cond, cpu->pc, ccnt);

            insn_type_t itype = INSN_OTHER;
            if (call_trace)
                itype = classify_insn(cpu);

            if (do_trace_output) {
                xtensa_disasm(cpu, cpu->pc, disasm_buf, sizeof(disasm_buf));
                if (syms) {
                    char sym_buf[128];
                    format_addr(syms, cpu->pc, sym_buf, sizeof(sym_buf));
                    trace_emit("[%08X] %s: %s\n", cpu->pc, sym_buf, disasm_buf);
                } else {
                    trace_emit("[%08X] %s\n", cpu->pc, disasm_buf);
                }
            }

            if (do_trace_output && verbose_trace) {
                for (int r = 0; r < 16; r++) prev_ar[r] = ar_read(cpu, r);
                prev_sar = cpu->sar;
                prev_ps = cpu->ps;
            }
            prev_pc = cpu->pc;

            int rc = xtensa_step(cpu);
            cycles++;

            if (do_trace_output && verbose_trace) {
                for (int r = 0; r < 16; r++) {
                    uint32_t cur = ar_read(cpu, r);
                    if (cur != prev_ar[r])
                        trace_emit("     a%-2d: 0x%08X -> 0x%08X\n", r, prev_ar[r], cur);
                }
                if (cpu->sar != prev_sar)
                    trace_emit("     SAR: %u -> %u\n", prev_sar, cpu->sar);
                if (cpu->ps != prev_ps)
                    trace_emit("     PS:  0x%08X -> 0x%08X\n", prev_ps, cpu->ps);
            }

            if (call_trace) {
                g_call_depth = call_depth;
                if (itype == INSN_CALL || itype == INSN_CALLX) {
                    char sym_buf[128];
                    format_addr(syms, cpu->pc, sym_buf, sizeof(sym_buf));
                    int indent = call_depth < 128 ? call_depth : 128;
                    fprintf(stderr, "%*s-> %s (0x%08X)\n", indent * 2, "", sym_buf, cpu->pc);
                    if (call_depth < 128) call_depth++;
                } else if (itype == INSN_ENTRY) {
                    /* ENTRY is inside the callee — CALL already printed */
                } else if (itype == INSN_RET || itype == INSN_RETW) {
                    if (call_depth > 0) call_depth--;
                    char sym_buf[128];
                    format_addr(syms, prev_pc, sym_buf, sizeof(sym_buf));
                    int indent = call_depth < 128 ? call_depth : 128;
                    fprintf(stderr, "%*s<- %s\n", indent * 2, "", sym_buf);
                }
            }

            if (assert_count > 0)
                check_assertions(asserts, assert_count, cpu, syms, cycles, mem);

            if (cond.active)
                cond_trace_check_until(&cond, cpu->pc);

            if (rc < 0) {
                if (cpu->breakpoint_hit) {
                    char sym_buf[128];
                    format_addr(syms, cpu->breakpoint_hit_addr, sym_buf, sizeof(sym_buf));
                    trace_emit("[BP] Hit breakpoint at 0x%08X (%s)\n",
                            cpu->breakpoint_hit_addr, sym_buf);
                    stop_reason = STOP_BREAKPOINT;
                    break;
                }
                if (cpu->halted) {
                    trace_emit("CPU halted (WAITI) at cycle %llu\n",
                            (unsigned long long)cycles);
                    stop_reason = STOP_HALT;
                    break;
                }
                if (cpu->exception && do_trace_output && verbose_trace) {
                    const char *vec = detect_vector(cpu->pc, cpu->vecbase);
                    if (vec)
                        trace_emit("     [INT] %s vector at 0x%08X\n", vec, cpu->pc);
                    else
                        trace_emit("     [EXC] cause=%u (%s) at 0x%08X\n",
                                cpu->exccause, exc_cause_name(cpu->exccause), cpu->epc[0]);
                }
                /* Event log: exception events */
                if (cpu->exception && event_log && !do_trace_output) {
                    const char *vec = detect_vector(cpu->pc, cpu->vecbase);
                    if (vec)
                        fprintf(stderr, "[%10llu] EXC   %s at 0x%08X\n",
                                (unsigned long long)cycles, vec, cpu->pc);
                    else
                        fprintf(stderr, "[%10llu] EXC   cause=%u (%s) at 0x%08X\n",
                                (unsigned long long)cycles,
                                cpu->exccause, exc_cause_name(cpu->exccause), cpu->epc[0]);
                }
                if (cpu->exception) {
                    if (cpu->epc[0] == last_exc_pc && cpu->exccause == last_exc_cause) {
                        exc_repeat++;
                        if (exc_repeat >= EXC_LOOP_THRESH) {
                            trace_emit("[STOP] Exception loop: cause=%u (%s) at 0x%08X, repeated %dx\n",
                                    cpu->exccause, exc_cause_name(cpu->exccause), cpu->epc[0], exc_repeat);
                            stop_reason = STOP_EXCEPTION_LOOP;
                            break;
                        }
                    } else {
                        last_exc_pc = cpu->epc[0];
                        last_exc_cause = cpu->exccause;
                        exc_repeat = 1;
                    }
                }
            } else {
                if (do_trace_output && verbose_trace && cpu->pc != prev_pc + 2 && cpu->pc != prev_pc + 3) {
                    const char *vec = detect_vector(cpu->pc, cpu->vecbase);
                    if (vec)
                        trace_emit("     [INT] %s vector at 0x%08X\n", vec, cpu->pc);
                }
                exc_repeat = 0;
            }

            if (!cpu->running) {
                stop_reason = STOP_CPU_STOPPED;
                break;
            }
            if (cpu->pc == prev_pc && frt) {
                uint32_t param;
                uint32_t fn = freertos_stubs_consume_deferred_task(frt, &param);
                if (fn) {
                    ar_write(cpu, 1, 0x3FFE0000u);
                    ar_write(cpu, 2, param);
                    cpu->pc = fn;
                    cpu->ps = 0x00040020u;
                }
            }
        } else {
            /* --- Batch execution --- */
            uint32_t pc_before = cpu->pc;
            uint64_t remaining = max_cycles_u64 - cycles;
            int n = remaining < (uint64_t)batch ? (int)remaining : batch;
            /* Cap batch at trace window boundary if approaching.
             * Use cpu->cycle_count to match event log timestamps. */
            if (need_step && cpu->cycle_count < trace_start) {
                uint64_t to_window = trace_start - cpu->cycle_count;
                if (to_window < (uint64_t)n) n = (int)to_window;
            }
            int ran = xtensa_run(cpu, n);
            cycles += ran;

            /* Event log: check exception after batch */
            if (event_log && cpu->exception) {
                const char *vec = detect_vector(cpu->pc, cpu->vecbase);
                if (vec)
                    fprintf(stderr, "[%10llu] EXC   %s at 0x%08X\n",
                            (unsigned long long)cycles, vec, cpu->pc);
                else
                    fprintf(stderr, "[%10llu] EXC   cause=%u (%s) at 0x%08X\n",
                            (unsigned long long)cycles,
                            cpu->exccause, exc_cause_name(cpu->exccause), cpu->epc[0]);
            }

            if (ran < n) {
                /* Check if we stopped for a good reason, or exception loop */
                if (cpu->breakpoint_hit || cpu->halted || !cpu->running) break;
            }
            if (cpu->pc == pc_before && frt) {
                uint32_t param;
                uint32_t fn = freertos_stubs_consume_deferred_task(frt, &param);
                if (fn) {
                    ar_write(cpu, 1, 0x3FFE0000u);
                    ar_write(cpu, 2, param);
                    cpu->pc = fn;
                    cpu->ps = 0x00040020u;
                }
            }
        }

        /* Preemptive timeslice + core 1 management */
        flexe_session_post_batch(session, batch);

        /* Progress heartbeat */
        if (heartbeat_interval > 0 &&
            cpu->cycle_count / heartbeat_interval > last_heartbeat) {
            last_heartbeat = cpu->cycle_count / heartbeat_interval;
            uint32_t cur_stubs = rom_stubs_total_calls(rom);
            char sym_buf[128];
            format_addr(syms, cpu->pc, sym_buf, sizeof(sym_buf));
            const char *task_name = frt ? freertos_stubs_current_task_name(frt, 0) : NULL;
            fprintf(stderr, "[%10llu] ---- %s | stubs:%u | task:%s ----\n",
                    (unsigned long long)cpu->cycle_count, sym_buf,
                    cur_stubs - hb_stub_count,
                    task_name ? task_name : "(none)");
            hb_stub_count = cur_stubs;
        }

        /* Automatic checkpoint - only save when FreeRTOS scheduler is active */
        if (checkpoint_interval > 0 && cpu->cycle_count >= next_checkpoint_cycle) {
            /* Validate checkpoint safety: scheduler must be running with an active task.
             * This prevents saving during initialization or in vTaskStartScheduler loop
             * where the system hasn't fully transitioned to task execution yet. */
            bool safe_to_checkpoint = true;
            const char *defer_reason = NULL;

            /* Check 1: FreeRTOS scheduler must be active (not in initialization) */
            if (frt && !freertos_stubs_scheduler_active(frt)) {
                safe_to_checkpoint = false;
                defer_reason = "scheduler not started";
            }

            /* Check 2: Must have an active task on at least one core (not stuck in vTaskStartScheduler) */
            if (safe_to_checkpoint && frt) {
                const char *task_name_core0 = freertos_stubs_current_task_name(frt, 0);
                const char *task_name_core1 = freertos_stubs_current_task_name(frt, 1);
                bool has_task_core0 = (task_name_core0 && strcmp(task_name_core0, "(none)") != 0);
                bool has_task_core1 = (task_name_core1 && strcmp(task_name_core1, "(none)") != 0);

                if (!has_task_core0 && !has_task_core1) {
                    safe_to_checkpoint = false;
                    defer_reason = "no active task on any core";
                }
            }

            /* Check 3: CPU must not be halted */
            if (safe_to_checkpoint && cpu->halted) {
                safe_to_checkpoint = false;
                defer_reason = "CPU halted";
            }

            if (safe_to_checkpoint) {
                char checkpoint_path[512];
                const char *dir = checkpoint_dir ? checkpoint_dir : ".";
                snprintf(checkpoint_path, sizeof(checkpoint_path),
                         "%s/checkpoint-%llu.sav", dir,
                         (unsigned long long)cpu->cycle_count);

                const char *task_c0 = frt ? freertos_stubs_current_task_name(frt, 0) : "(none)";
                const char *task_c1 = frt ? freertos_stubs_current_task_name(frt, 1) : "(none)";
                fprintf(stderr, "[%10llu] Creating checkpoint: %s\n",
                        (unsigned long long)cpu->cycle_count, checkpoint_path);
                fprintf(stderr, "  Core0 task: %s, Core1 task: %s\n",
                        task_c0 ? task_c0 : "(null)", task_c1 ? task_c1 : "(null)");
                fprintf(stderr, "  INTENABLE=0x%08X, CCOMPARE[0]=%u\n",
                        cpu->intenable, cpu->ccompare[0]);

                char desc[256];
                snprintf(desc, sizeof(desc), "auto-checkpoint at %llu cycles (c0:%s c1:%s)",
                         (unsigned long long)cpu->cycle_count,
                         task_c0 ? task_c0 : "(null)", task_c1 ? task_c1 : "(null)");

                if (savestate_save(cpu, frt, checkpoint_path, desc) != 0) {
                    fprintf(stderr, "[%10llu] WARNING: Failed to save checkpoint: %s\n",
                            (unsigned long long)cpu->cycle_count, checkpoint_path);
                } else {
                    fprintf(stderr, "[%10llu] Checkpoint saved successfully\n",
                            (unsigned long long)cpu->cycle_count);
                }

                next_checkpoint_cycle += checkpoint_interval;
            } else if (event_log && defer_reason) {
                fprintf(stderr, "[%10llu] Checkpoint DEFERRED (%s)\n",
                        (unsigned long long)cpu->cycle_count, defer_reason);
            }
            /* If unsafe, we'll try again on the next cycle */
        }
    }

    /* Determine stop reason */
    if (stop_reason == STOP_RUNNING) {
        if (cpu->breakpoint_hit)
            stop_reason = STOP_BREAKPOINT;
        else if (cpu->halted)
            stop_reason = STOP_HALT;
        else if (!cpu->running)
            stop_reason = STOP_CPU_STOPPED;
        else
            stop_reason = STOP_MAX_CYCLES;
    }

    /* Flush any buffered output to stdout before summary */
    fflush(stdout);

    /* Dump ring buffer based on dump conditions */
    if (g_ring) {
        int is_crash = (stop_reason == STOP_CPU_STOPPED ||
                        stop_reason == STOP_EXCEPTION_LOOP);
        if ((g_dump_mode & DUMP_CRASH) && is_crash)
            ring_flush(g_ring);
        else if (g_dump_mode & DUMP_FLUSH)
            ring_flush(g_ring);
        else if (g_dump_mode & DUMP_TAIL)
            ring_flush_tail(g_ring, g_dump_tail_n);
    }

    /* ===== Summary ===== */
    fprintf(stderr, "\n--- Execution summary ---\n");

    /* Stop reason with details */
    if (stop_reason == STOP_EXCEPTION_LOOP) {
        fprintf(stderr, "Stop reason: %s (cause=%u %s at 0x%08X, repeated %dx)\n",
                stop_reason_str(stop_reason), last_exc_cause,
                exc_cause_name(last_exc_cause), last_exc_pc, exc_repeat);
    } else if (stop_reason == STOP_BREAKPOINT) {
        char sym_buf[128];
        format_addr(syms, cpu->breakpoint_hit_addr, sym_buf, sizeof(sym_buf));
        fprintf(stderr, "Stop reason: %s at 0x%08X (%s)\n",
                stop_reason_str(stop_reason), cpu->breakpoint_hit_addr, sym_buf);
    } else {
        fprintf(stderr, "Stop reason: %s\n", stop_reason_str(stop_reason));
    }

    fprintf(stderr, "Cycles:     %llu (virtual: %llu)\n",
            (unsigned long long)cycles, (unsigned long long)cpu->cycle_count);

    /* Final PC with symbol */
    char pc_sym[128];
    format_addr(syms, cpu->pc, pc_sym, sizeof(pc_sym));
    fprintf(stderr, "Final PC:   0x%08X (%s)\n", cpu->pc, pc_sym);
    if (!single_core) {
        xtensa_cpu_t *cpu1 = flexe_session_cpu(session, 1);
        char pc1_sym[128];
        format_addr(syms, cpu1->pc, pc1_sym, sizeof(pc1_sym));
        fprintf(stderr, "Core 1 PC:  0x%08X (%s)\n", cpu1->pc, pc1_sym);
    }

    fprintf(stderr, "UART TX:    %d bytes\n", periph_uart_tx_count(periph));

    /* ROM stub call stats */
    int nstubs = rom_stubs_stub_count(rom);
    int first_stat = 1;
    fprintf(stderr, "ROM calls:  ");
    for (int i = 0; i < nstubs; i++) {
        const char *name;
        uint32_t addr, count;
        rom_stubs_get_stats(rom, i, &name, &addr, &count);
        if (count > 0) {
            if (!first_stat) fprintf(stderr, ", ");
            fprintf(stderr, "%s x%u", name, count);
            first_stat = 0;
        }
    }
    if (first_stat) fprintf(stderr, "(none)");
    fprintf(stderr, "\n");
    if (rom_stubs_unregistered_count(rom) > 0)
        fprintf(stderr, "Unregistered ROM calls: %d\n", rom_stubs_unregistered_count(rom));

    if (!quiet_unhandled || periph_unhandled_count(periph) > 0)
        fprintf(stderr, "Unhandled:  %d peripheral accesses\n", periph_unhandled_count(periph));

    /* Register dump */
    if (verbose || stop_reason == STOP_BREAKPOINT) {
        fprintf(stderr, "\n--- Registers ---\n");
        fprintf(stderr, "PS=0x%08X  SAR=%u  LBEG=0x%08X  LEND=0x%08X  LCOUNT=%u\n",
                cpu->ps, cpu->sar, cpu->lbeg, cpu->lend, cpu->lcount);
        fprintf(stderr, "WINDOWBASE=%u  WINDOWSTART=0x%04X  VECBASE=0x%08X\n",
                cpu->windowbase, cpu->windowstart, cpu->vecbase);
        fprintf(stderr, "EXCCAUSE=%u  EXCVADDR=0x%08X\n", cpu->exccause, cpu->excvaddr);
        for (int i = 0; i < 16; i += 4)
            fprintf(stderr, "a%d=0x%08X  a%d=0x%08X  a%d=0x%08X  a%d=0x%08X\n",
                    i, ar_read(cpu, i), i+1, ar_read(cpu, i+1),
                    i+2, ar_read(cpu, i+2), i+3, ar_read(cpu, i+3));
    }

    /* Memory dumps */
    for (int i = 0; i < dump_count; i++) {
        fprintf(stderr, "\n--- Memory dump: 0x%08X (%u bytes) ---\n",
                mem_dumps[i].addr, mem_dumps[i].len);
        hexdump(mem, mem_dumps[i].addr, mem_dumps[i].len);
    }

    /* Cleanup */
    ring_destroy(g_ring);
    flexe_session_destroy(session);
    return 0;
}
