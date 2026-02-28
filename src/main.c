#include "xtensa.h"
#include "memory.h"
#include "loader.h"
#include "peripherals.h"
#include "rom_stubs.h"
#include "elf_symbols.h"
#include "freertos_stubs.h"
#include "esp_timer_stubs.h"
#include "display_stubs.h"
#include "touch_stubs.h"
#include "sdcard_stubs.h"
#include "wifi_stubs.h"
#include "sha_stubs.h"
#include "aes_stubs.h"
#include "mpi_stubs.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <ctype.h>
#include <getopt.h>
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
    int max_cycles = 10000000;
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

    int opt;
    while ((opt = getopt(argc, argv, "1c:tT::WVvqe:s:b:m:S:Z:C:FB:A:EP:D:")) != -1) {
        switch (opt) {
        case '1': single_core = 1; break;
        case 'c': max_cycles = atoi(optarg); break;
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

    /* Load ELF symbols if requested */
    elf_symbols_t *syms = NULL;
    if (elf_path) {
        syms = elf_symbols_load(elf_path);
        if (syms)
            fprintf(stderr, "Loaded %d symbols from %s\n", elf_symbols_count(syms), elf_path);
        else
            fprintf(stderr, "Warning: failed to load symbols from %s\n", elf_path);
    }

    /* Create memory and peripherals */
    xtensa_mem_t *mem = mem_create();
    if (!mem) {
        fprintf(stderr, "Failed to allocate memory\n");
        elf_symbols_destroy(syms);
        ring_destroy(g_ring);
        return 1;
    }

    esp32_periph_t *periph = periph_create(mem);
    if (!periph) {
        fprintf(stderr, "Failed to create peripherals\n");
        mem_destroy(mem);
        elf_symbols_destroy(syms);
        ring_destroy(g_ring);
        return 1;
    }
    periph_set_uart_callback(periph, uart_stdout_cb, NULL);

    /* Load firmware */
    load_result_t res = loader_load_bin(mem, firmware);
    if (res.result != 0) {
        fprintf(stderr, "Load error: %s\n", res.error);
        periph_destroy(periph);
        mem_destroy(mem);
        elf_symbols_destroy(syms);
        ring_destroy(g_ring);
        return 1;
    }
    fprintf(stderr, "Loaded %s: %d segments, entry=0x%08X\n",
            firmware, res.segment_count, res.entry_point);
    for (int i = 0; i < res.segment_count; i++) {
        fprintf(stderr, "  Segment %d: 0x%08X (%u bytes) -> %s\n",
                i, res.segments[i].addr, res.segments[i].size,
                loader_region_name(res.segments[i].addr));
    }

    /* Initialize CPUs (core 0 = PRO_CPU, core 1 = APP_CPU) */
    xtensa_cpu_t cpu[2];
    xtensa_cpu_init(&cpu[0]);
    xtensa_cpu_reset(&cpu[0]);
    cpu[0].mem = mem;
    cpu[0].window_trace = window_trace;
    cpu[0].window_trace_active = (window_trace && trace_start == 0);
    cpu[0].spill_verify = spill_verify;

    xtensa_cpu_init(&cpu[1]);
    xtensa_cpu_reset(&cpu[1]);
    cpu[1].mem = mem;
    cpu[1].core_id = 1;
    cpu[1].prid = 0xABAB;    /* APP_CPU PRID */
    cpu[1].running = false;   /* Not started yet */
    cpu[1].window_trace = window_trace;
    cpu[1].window_trace_active = false;
    cpu[1].spill_verify = spill_verify;

    /* Install ROM function stubs */
    esp32_rom_stubs_t *rom = rom_stubs_create(&cpu[0]);
    if (!rom) {
        fprintf(stderr, "Failed to create ROM stubs\n");
        periph_destroy(periph);
        mem_destroy(mem);
        elf_symbols_destroy(syms);
        ring_destroy(g_ring);
        return 1;
    }

    /* Set single-core mode on ROM stubs */
    rom_stubs_set_single_core(rom, single_core);

    /* Hook firmware functions by symbol name (newlib locks, NVS, GPIO driver etc.) */
    if (syms)
        rom_stubs_hook_symbols(rom, syms);

    /* FreeRTOS stubs */
    freertos_stubs_t *frt = freertos_stubs_create(&cpu[0]);
    if (frt) {
        if (!single_core)
            freertos_stubs_attach_cpu(frt, 1, &cpu[1]);
        if (syms)
            freertos_stubs_hook_symbols(frt, syms);
    }

    /* esp_timer stubs */
    esp_timer_stubs_t *etimer = esp_timer_stubs_create(&cpu[0]);
    if (etimer && syms)
        esp_timer_stubs_hook_symbols(etimer, syms);

    /* Display stubs (no framebuffer in standalone mode — just hooks) */
    display_stubs_t *dstubs = display_stubs_create(&cpu[0]);
    if (dstubs && syms) {
        display_stubs_hook_symbols(dstubs, syms);
        display_stubs_hook_tft_espi(dstubs, syms);
        display_stubs_hook_tft_esprite(dstubs, syms);
    }

    /* Touch stubs (no input in standalone mode) */
    touch_stubs_t *tstubs = touch_stubs_create(&cpu[0]);
    if (tstubs && syms)
        touch_stubs_hook_symbols(tstubs, syms);

    /* SD card stubs */
    sdcard_stubs_t *sstubs = sdcard_stubs_create(&cpu[0]);
    if (sstubs) {
        if (sdcard_path)
            sdcard_stubs_set_image(sstubs, sdcard_path);
        if (sdcard_size > 0)
            sdcard_stubs_set_size(sstubs, sdcard_size);
        if (syms)
            sdcard_stubs_hook_symbols(sstubs, syms);
    }

    /* SHA hardware accelerator stubs */
    sha_stubs_t *shstubs = sha_stubs_create(&cpu[0]);
    if (shstubs && syms)
        sha_stubs_hook_symbols(shstubs, syms);

    /* AES hardware accelerator stubs */
    aes_stubs_t *astubs = aes_stubs_create(&cpu[0]);
    if (astubs && syms)
        aes_stubs_hook_symbols(astubs, syms);

    /* MPI (RSA) hardware accelerator stubs */
    mpi_stubs_t *mstubs = mpi_stubs_create(&cpu[0]);
    if (mstubs && syms)
        mpi_stubs_hook_symbols(mstubs, syms);

    /* WiFi / lwip socket bridge */
    wifi_stubs_t *wstubs = wifi_stubs_create(&cpu[0]);
    if (wstubs && syms)
        wifi_stubs_hook_symbols(wstubs, syms);

    /* Core 1 shares memory and PC hook table with core 0 */
    if (!single_core) {
        cpu[1].pc_hook = cpu[0].pc_hook;
        cpu[1].pc_hook_ctx = cpu[0].pc_hook_ctx;
    }

    /* Install ROM log callback for verbose trace, call trace, or event log */
    if (call_trace) {
        g_call_syms = syms;
        rom_stubs_set_log_callback(rom, rom_log_call_cb, NULL);
    } else if (event_log) {
        rom_stubs_set_log_callback(rom, rom_log_event_cb, NULL);
    } else if (verbose_trace) {
        rom_stubs_set_log_callback(rom, rom_log_cb, NULL);
    }

    /* Install FreeRTOS event callback for event log */
    if (event_log && frt)
        freertos_stubs_set_event_fn(frt, freertos_event_cb, NULL);

    /* Install WiFi event log mode */
    if (event_log && wstubs)
        wifi_stubs_set_event_log(wstubs, true);

    /* Resolve conditional trace symbols */
    if (cond.active)
        cond_trace_resolve(&cond, syms);

    /* Set entry point and initial stack pointer */
    if (has_entry_override)
        cpu[0].pc = entry_override;
    else if (res.entry_point != 0)
        cpu[0].pc = res.entry_point;
    ar_write(&cpu[0], 1, 0x3FFE0000u);  /* SP in SRAM data, above BSS */

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
        if (xtensa_set_breakpoint(&cpu[0], bp_addr) == 0) {
            char sym_buf[128];
            format_addr(syms, bp_addr, sym_buf, sizeof(sym_buf));
            fprintf(stderr, "Breakpoint set at 0x%08X (%s)\n", bp_addr, sym_buf);
        }
    }

    /* ===== Execute ===== */
    uint64_t cycles = 0;
    uint64_t max_cycles_u64 = (uint64_t)max_cycles;
    stop_reason_t stop_reason = STOP_RUNNING;
    char disasm_buf[128];

    /* Exception loop detection */
    uint32_t last_exc_pc = 0;
    uint32_t last_exc_cause = 0xFFFFFFFF;
    int exc_repeat = 0;

    /* Heartbeat state */
    uint64_t last_heartbeat = 0;
    uint32_t hb_stub_count = 0;

    /* Trace state (used in trace windows) */
    uint32_t prev_ar[16];
    uint32_t prev_sar = 0, prev_ps = 0;
    uint32_t prev_pc = cpu[0].pc;
    int call_depth = 0;

    /* Determine if we need per-step execution at all */
    int need_step = trace || call_trace || assert_count;

    /* Unified execution loop */
    int batch = 10000;
    while (cycles < max_cycles_u64 && cpu[0].running && !cpu[0].halted && !cpu[0].breakpoint_hit) {
        /* Are we in a trace window?
         * Use cpu[0].cycle_count (virtual time including FreeRTOS skips)
         * so trace windows match event log timestamps. */
        uint64_t ccnt = cpu[0].cycle_count;
        /* Update window trace gating */
        if (window_trace)
            cpu[0].window_trace_active = (trace_start == 0 || ccnt >= trace_start) &&
                                      (ccnt < trace_end);
        int in_trace_window = need_step &&
            (trace_start == 0 || ccnt >= trace_start) &&
            (ccnt < trace_end);

        if (in_trace_window) {
            /* --- Single-step with full trace output --- */
            int do_trace_output = (verbose_trace || (!call_trace && !assert_count));
            if (do_trace_output && cond.active)
                do_trace_output = cond_trace_active(&cond, cpu[0].pc, ccnt);

            insn_type_t itype = INSN_OTHER;
            if (call_trace)
                itype = classify_insn(&cpu[0]);

            if (do_trace_output) {
                xtensa_disasm(&cpu[0], cpu[0].pc, disasm_buf, sizeof(disasm_buf));
                if (syms) {
                    char sym_buf[128];
                    format_addr(syms, cpu[0].pc, sym_buf, sizeof(sym_buf));
                    trace_emit("[%08X] %s: %s\n", cpu[0].pc, sym_buf, disasm_buf);
                } else {
                    trace_emit("[%08X] %s\n", cpu[0].pc, disasm_buf);
                }
            }

            if (do_trace_output && verbose_trace) {
                for (int r = 0; r < 16; r++) prev_ar[r] = ar_read(&cpu[0], r);
                prev_sar = cpu[0].sar;
                prev_ps = cpu[0].ps;
            }
            prev_pc = cpu[0].pc;

            int rc = xtensa_step(&cpu[0]);
            cycles++;

            if (do_trace_output && verbose_trace) {
                for (int r = 0; r < 16; r++) {
                    uint32_t cur = ar_read(&cpu[0], r);
                    if (cur != prev_ar[r])
                        trace_emit("     a%-2d: 0x%08X -> 0x%08X\n", r, prev_ar[r], cur);
                }
                if (cpu[0].sar != prev_sar)
                    trace_emit("     SAR: %u -> %u\n", prev_sar, cpu[0].sar);
                if (cpu[0].ps != prev_ps)
                    trace_emit("     PS:  0x%08X -> 0x%08X\n", prev_ps, cpu[0].ps);
            }

            if (call_trace) {
                g_call_depth = call_depth;
                if (itype == INSN_CALL || itype == INSN_CALLX) {
                    char sym_buf[128];
                    format_addr(syms, cpu[0].pc, sym_buf, sizeof(sym_buf));
                    int indent = call_depth < 128 ? call_depth : 128;
                    fprintf(stderr, "%*s-> %s (0x%08X)\n", indent * 2, "", sym_buf, cpu[0].pc);
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
                check_assertions(asserts, assert_count, &cpu[0], syms, cycles, mem);

            if (cond.active)
                cond_trace_check_until(&cond, cpu[0].pc);

            if (rc < 0) {
                if (cpu[0].breakpoint_hit) {
                    char sym_buf[128];
                    format_addr(syms, cpu[0].breakpoint_hit_addr, sym_buf, sizeof(sym_buf));
                    trace_emit("[BP] Hit breakpoint at 0x%08X (%s)\n",
                            cpu[0].breakpoint_hit_addr, sym_buf);
                    stop_reason = STOP_BREAKPOINT;
                    break;
                }
                if (cpu[0].halted) {
                    trace_emit("CPU halted (WAITI) at cycle %llu\n",
                            (unsigned long long)cycles);
                    stop_reason = STOP_HALT;
                    break;
                }
                if (cpu[0].exception && do_trace_output && verbose_trace) {
                    const char *vec = detect_vector(cpu[0].pc, cpu[0].vecbase);
                    if (vec)
                        trace_emit("     [INT] %s vector at 0x%08X\n", vec, cpu[0].pc);
                    else
                        trace_emit("     [EXC] cause=%u (%s) at 0x%08X\n",
                                cpu[0].exccause, exc_cause_name(cpu[0].exccause), cpu[0].epc[0]);
                }
                /* Event log: exception events */
                if (cpu[0].exception && event_log && !do_trace_output) {
                    const char *vec = detect_vector(cpu[0].pc, cpu[0].vecbase);
                    if (vec)
                        fprintf(stderr, "[%10llu] EXC   %s at 0x%08X\n",
                                (unsigned long long)cycles, vec, cpu[0].pc);
                    else
                        fprintf(stderr, "[%10llu] EXC   cause=%u (%s) at 0x%08X\n",
                                (unsigned long long)cycles,
                                cpu[0].exccause, exc_cause_name(cpu[0].exccause), cpu[0].epc[0]);
                }
                if (cpu[0].exception) {
                    if (cpu[0].epc[0] == last_exc_pc && cpu[0].exccause == last_exc_cause) {
                        exc_repeat++;
                        if (exc_repeat >= EXC_LOOP_THRESH) {
                            trace_emit("[STOP] Exception loop: cause=%u (%s) at 0x%08X, repeated %dx\n",
                                    cpu[0].exccause, exc_cause_name(cpu[0].exccause), cpu[0].epc[0], exc_repeat);
                            stop_reason = STOP_EXCEPTION_LOOP;
                            break;
                        }
                    } else {
                        last_exc_pc = cpu[0].epc[0];
                        last_exc_cause = cpu[0].exccause;
                        exc_repeat = 1;
                    }
                }
            } else {
                if (do_trace_output && verbose_trace && cpu[0].pc != prev_pc + 2 && cpu[0].pc != prev_pc + 3) {
                    const char *vec = detect_vector(cpu[0].pc, cpu[0].vecbase);
                    if (vec)
                        trace_emit("     [INT] %s vector at 0x%08X\n", vec, cpu[0].pc);
                }
                exc_repeat = 0;
            }

            if (!cpu[0].running) {
                stop_reason = STOP_CPU_STOPPED;
                break;
            }
            if (cpu[0].pc == prev_pc && frt) {
                uint32_t param;
                uint32_t fn = freertos_stubs_consume_deferred_task(frt, &param);
                if (fn) {
                    ar_write(&cpu[0], 1, 0x3FFE0000u);
                    ar_write(&cpu[0], 2, param);
                    cpu[0].pc = fn;
                    cpu[0].ps = 0x00040020u;
                }
            }
        } else {
            /* --- Batch execution --- */
            uint32_t pc_before = cpu[0].pc;
            uint64_t remaining = max_cycles_u64 - cycles;
            int n = remaining < (uint64_t)batch ? (int)remaining : batch;
            /* Cap batch at trace window boundary if approaching.
             * Use cpu[0].cycle_count to match event log timestamps. */
            if (need_step && cpu[0].cycle_count < trace_start) {
                uint64_t to_window = trace_start - cpu[0].cycle_count;
                if (to_window < (uint64_t)n) n = (int)to_window;
            }
            int ran = xtensa_run(&cpu[0], n);
            cycles += ran;

            /* Event log: check exception after batch */
            if (event_log && cpu[0].exception) {
                const char *vec = detect_vector(cpu[0].pc, cpu[0].vecbase);
                if (vec)
                    fprintf(stderr, "[%10llu] EXC   %s at 0x%08X\n",
                            (unsigned long long)cycles, vec, cpu[0].pc);
                else
                    fprintf(stderr, "[%10llu] EXC   cause=%u (%s) at 0x%08X\n",
                            (unsigned long long)cycles,
                            cpu[0].exccause, exc_cause_name(cpu[0].exccause), cpu[0].epc[0]);
            }

            if (ran < n) {
                /* Check if we stopped for a good reason, or exception loop */
                if (cpu[0].breakpoint_hit || cpu[0].halted || !cpu[0].running) break;
            }
            if (cpu[0].pc == pc_before && frt) {
                uint32_t param;
                uint32_t fn = freertos_stubs_consume_deferred_task(frt, &param);
                if (fn) {
                    ar_write(&cpu[0], 1, 0x3FFE0000u);
                    ar_write(&cpu[0], 2, param);
                    cpu[0].pc = fn;
                    cpu[0].ps = 0x00040020u;
                }
            }
        }

        /* Preemptive timeslice check (always) */
        if (frt) freertos_stubs_check_preempt(frt);

        /* Dual-core: check if core 1 should start.
         * Core 1 starts when firmware releases it from reset via DPORT write
         * AND the boot address has been set via ets_set_appcpu_boot_addr. */
        if (!single_core && !cpu[1].running &&
            periph_app_cpu_released(periph) &&
            rom_stubs_app_cpu_boot_addr(rom) != 0) {
            cpu[1].pc = rom_stubs_app_cpu_boot_addr(rom);
            cpu[1].running = true;
            fprintf(stderr, "[%10llu] CORE1 started at 0x%08X\n",
                    (unsigned long long)cpu[0].cycle_count, cpu[1].pc);
        }

        /* Dual-core: run core 1 batch */
        if (!single_core && cpu[1].running) {
            int n1 = (max_cycles_u64 - cycles < (uint64_t)batch) ?
                     (int)(max_cycles_u64 - cycles) : batch;
            xtensa_run(&cpu[1], n1);
            if (frt) freertos_stubs_check_preempt_core(frt, 1);
            /* Sync cycle counts: core 1 tracks core 0's virtual time */
            cpu[1].cycle_count = cpu[0].cycle_count;
            cpu[1].virtual_time_us = cpu[0].virtual_time_us;
        }

        /* Progress heartbeat */
        if (heartbeat_interval > 0 &&
            cpu[0].cycle_count / heartbeat_interval > last_heartbeat) {
            last_heartbeat = cpu[0].cycle_count / heartbeat_interval;
            uint32_t cur_stubs = rom_stubs_total_calls(rom);
            char sym_buf[128];
            format_addr(syms, cpu[0].pc, sym_buf, sizeof(sym_buf));
            const char *task_name = frt ? freertos_stubs_current_task_name(frt, 0) : NULL;
            fprintf(stderr, "[%10llu] ---- %s | stubs:%u | task:%s ----\n",
                    (unsigned long long)cpu[0].cycle_count, sym_buf,
                    cur_stubs - hb_stub_count,
                    task_name ? task_name : "(none)");
            hb_stub_count = cur_stubs;
        }
    }

    /* Determine stop reason */
    if (stop_reason == STOP_RUNNING) {
        if (cpu[0].breakpoint_hit)
            stop_reason = STOP_BREAKPOINT;
        else if (cpu[0].halted)
            stop_reason = STOP_HALT;
        else if (!cpu[0].running)
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
        format_addr(syms, cpu[0].breakpoint_hit_addr, sym_buf, sizeof(sym_buf));
        fprintf(stderr, "Stop reason: %s at 0x%08X (%s)\n",
                stop_reason_str(stop_reason), cpu[0].breakpoint_hit_addr, sym_buf);
    } else {
        fprintf(stderr, "Stop reason: %s\n", stop_reason_str(stop_reason));
    }

    fprintf(stderr, "Cycles:     %llu (virtual: %llu)\n",
            (unsigned long long)cycles, (unsigned long long)cpu[0].cycle_count);

    /* Final PC with symbol */
    char pc_sym[128];
    format_addr(syms, cpu[0].pc, pc_sym, sizeof(pc_sym));
    fprintf(stderr, "Final PC:   0x%08X (%s)\n", cpu[0].pc, pc_sym);
    if (!single_core) {
        char pc1_sym[128];
        format_addr(syms, cpu[1].pc, pc1_sym, sizeof(pc1_sym));
        fprintf(stderr, "Core 1 PC:  0x%08X (%s)\n", cpu[1].pc, pc1_sym);
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
                cpu[0].ps, cpu[0].sar, cpu[0].lbeg, cpu[0].lend, cpu[0].lcount);
        fprintf(stderr, "WINDOWBASE=%u  WINDOWSTART=0x%04X  VECBASE=0x%08X\n",
                cpu[0].windowbase, cpu[0].windowstart, cpu[0].vecbase);
        fprintf(stderr, "EXCCAUSE=%u  EXCVADDR=0x%08X\n", cpu[0].exccause, cpu[0].excvaddr);
        for (int i = 0; i < 16; i += 4)
            fprintf(stderr, "a%d=0x%08X  a%d=0x%08X  a%d=0x%08X  a%d=0x%08X\n",
                    i, ar_read(&cpu[0], i), i+1, ar_read(&cpu[0], i+1),
                    i+2, ar_read(&cpu[0], i+2), i+3, ar_read(&cpu[0], i+3));
    }

    /* Memory dumps */
    for (int i = 0; i < dump_count; i++) {
        fprintf(stderr, "\n--- Memory dump: 0x%08X (%u bytes) ---\n",
                mem_dumps[i].addr, mem_dumps[i].len);
        hexdump(mem, mem_dumps[i].addr, mem_dumps[i].len);
    }

    /* Cleanup */
    ring_destroy(g_ring);
    wifi_stubs_destroy(wstubs);
    sha_stubs_destroy(shstubs);
    aes_stubs_destroy(astubs);
    mpi_stubs_destroy(mstubs);
    sdcard_stubs_destroy(sstubs);
    touch_stubs_destroy(tstubs);
    display_stubs_destroy(dstubs);
    esp_timer_stubs_destroy(etimer);
    freertos_stubs_destroy(frt);
    rom_stubs_destroy(rom);
    periph_destroy(periph);
    mem_destroy(mem);
    elf_symbols_destroy(syms);
    return 0;
}
