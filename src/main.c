#include "xtensa.h"
#include "memory.h"
#include "loader.h"
#include "peripherals.h"
#include "rom_stubs.h"
#include "elf_symbols.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <getopt.h>

/* ===== Configuration ===== */

#define MAX_BP_ARGS     16
#define MAX_MEM_DUMPS   8
#define EXC_LOOP_THRESH 3

typedef struct {
    uint32_t addr;
    uint32_t len;
} mem_dump_t;

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
    fprintf(stderr, "     [ROM] %s @ 0x%08X\n", name, addr);
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
    fprintf(stderr, "  -c <cycles>     Max cycles (default: 10000000)\n");
    fprintf(stderr, "  -t              Instruction trace to stderr\n");
    fprintf(stderr, "  -T              Verbose trace (reg changes, ROM calls, exceptions)\n");
    fprintf(stderr, "  -v              Verbose register dump on exit\n");
    fprintf(stderr, "  -e <addr>       Override entry point (hex)\n");
    fprintf(stderr, "  -s <file.elf>   Load ELF symbols for trace/breakpoints\n");
    fprintf(stderr, "  -b <addr|name>  Set breakpoint (repeatable)\n");
    fprintf(stderr, "  -m <addr[:len]> Dump memory on exit (repeatable, default len=256)\n");
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
    int verbose = 0;
    uint32_t entry_override = 0;
    int has_entry_override = 0;
    const char *elf_path = NULL;
    const char *bp_args[MAX_BP_ARGS];
    int bp_count = 0;
    mem_dump_t mem_dumps[MAX_MEM_DUMPS];
    int dump_count = 0;

    int opt;
    while ((opt = getopt(argc, argv, "c:tTve:s:b:m:")) != -1) {
        switch (opt) {
        case 'c': max_cycles = atoi(optarg); break;
        case 't': trace = 1; break;
        case 'T': verbose_trace = 1; trace = 1; break;
        case 'v': verbose = 1; break;
        case 'e':
            entry_override = (uint32_t)strtoul(optarg, NULL, 16);
            has_entry_override = 1;
            break;
        case 's': elf_path = optarg; break;
        case 'b':
            if (bp_count < MAX_BP_ARGS) bp_args[bp_count++] = optarg;
            break;
        case 'm':
            if (dump_count < MAX_MEM_DUMPS) parse_mem_dump(optarg, &mem_dumps[dump_count++]);
            break;
        default: usage(argv[0]); return 1;
        }
    }

    if (optind >= argc) {
        usage(argv[0]);
        return 1;
    }
    const char *firmware = argv[optind];

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
        return 1;
    }

    esp32_periph_t *periph = periph_create(mem);
    if (!periph) {
        fprintf(stderr, "Failed to create peripherals\n");
        mem_destroy(mem);
        elf_symbols_destroy(syms);
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
        return 1;
    }
    fprintf(stderr, "Loaded %s: %d segments, entry=0x%08X\n",
            firmware, res.segment_count, res.entry_point);

    /* Initialize CPU */
    xtensa_cpu_t cpu;
    xtensa_cpu_init(&cpu);
    xtensa_cpu_reset(&cpu);
    cpu.mem = mem;

    /* Install ROM function stubs */
    esp32_rom_stubs_t *rom = rom_stubs_create(&cpu);
    if (!rom) {
        fprintf(stderr, "Failed to create ROM stubs\n");
        periph_destroy(periph);
        mem_destroy(mem);
        elf_symbols_destroy(syms);
        return 1;
    }

    /* Install ROM log callback for verbose trace */
    if (verbose_trace)
        rom_stubs_set_log_callback(rom, rom_log_cb, NULL);

    /* Set entry point */
    if (has_entry_override)
        cpu.pc = entry_override;
    else if (res.entry_point != 0)
        cpu.pc = res.entry_point;

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
        if (xtensa_set_breakpoint(&cpu, bp_addr) == 0) {
            char sym_buf[128];
            format_addr(syms, bp_addr, sym_buf, sizeof(sym_buf));
            fprintf(stderr, "Breakpoint set at 0x%08X (%s)\n", bp_addr, sym_buf);
        }
    }

    /* ===== Execute ===== */
    int cycles = 0;
    stop_reason_t stop_reason = STOP_RUNNING;
    char disasm_buf[128];

    /* Exception loop detection */
    uint32_t last_exc_pc = 0;
    uint32_t last_exc_cause = 0xFFFFFFFF;
    int exc_repeat = 0;

    if (trace) {
        /* Verbose trace state */
        uint32_t prev_ar[16];
        uint32_t prev_sar = 0, prev_ps = 0;
        uint32_t prev_pc;

        for (int i = 0; i < max_cycles; i++) {
            /* Disassemble before step */
            xtensa_disasm(&cpu, cpu.pc, disasm_buf, sizeof(disasm_buf));
            if (syms) {
                char sym_buf[128];
                format_addr(syms, cpu.pc, sym_buf, sizeof(sym_buf));
                fprintf(stderr, "[%08X] %s: %s\n", cpu.pc, sym_buf, disasm_buf);
            } else {
                fprintf(stderr, "[%08X] %s\n", cpu.pc, disasm_buf);
            }

            /* Snapshot for verbose trace */
            if (verbose_trace) {
                for (int r = 0; r < 16; r++) prev_ar[r] = ar_read(&cpu, r);
                prev_sar = cpu.sar;
                prev_ps = cpu.ps;
            }
            prev_pc = cpu.pc;

            int rc = xtensa_step(&cpu);
            cycles++;

            /* Verbose trace: register changes */
            if (verbose_trace) {
                for (int r = 0; r < 16; r++) {
                    uint32_t cur = ar_read(&cpu, r);
                    if (cur != prev_ar[r])
                        fprintf(stderr, "     a%-2d: 0x%08X -> 0x%08X\n", r, prev_ar[r], cur);
                }
                if (cpu.sar != prev_sar)
                    fprintf(stderr, "     SAR: %u -> %u\n", prev_sar, cpu.sar);
                if (cpu.ps != prev_ps)
                    fprintf(stderr, "     PS:  0x%08X -> 0x%08X\n", prev_ps, cpu.ps);
            }

            if (rc < 0) {
                if (cpu.breakpoint_hit) {
                    char sym_buf[128];
                    format_addr(syms, cpu.breakpoint_hit_addr, sym_buf, sizeof(sym_buf));
                    fprintf(stderr, "[BP] Hit breakpoint at 0x%08X (%s)\n",
                            cpu.breakpoint_hit_addr, sym_buf);
                    stop_reason = STOP_BREAKPOINT;
                    break;
                }
                if (cpu.halted) {
                    fprintf(stderr, "CPU halted (WAITI) at cycle %d\n", cycles);
                    stop_reason = STOP_HALT;
                    break;
                }
                /* Exception/interrupt event logging */
                if (cpu.exception && verbose_trace) {
                    /* Detect vector entry */
                    const char *vec = detect_vector(cpu.pc, cpu.vecbase);
                    if (vec) {
                        fprintf(stderr, "     [INT] %s vector at 0x%08X\n", vec, cpu.pc);
                    } else {
                        fprintf(stderr, "     [EXC] cause=%u (%s) at 0x%08X\n",
                                cpu.exccause, exc_cause_name(cpu.exccause), cpu.epc[0]);
                    }
                }
                /* Exception loop detection */
                if (cpu.exception) {
                    if (cpu.epc[0] == last_exc_pc && cpu.exccause == last_exc_cause) {
                        exc_repeat++;
                        if (exc_repeat >= EXC_LOOP_THRESH) {
                            fprintf(stderr, "[STOP] Exception loop: cause=%u (%s) at 0x%08X, repeated %dx\n",
                                    cpu.exccause, exc_cause_name(cpu.exccause), cpu.epc[0], exc_repeat);
                            stop_reason = STOP_EXCEPTION_LOOP;
                            break;
                        }
                    } else {
                        last_exc_pc = cpu.epc[0];
                        last_exc_cause = cpu.exccause;
                        exc_repeat = 1;
                    }
                }
            } else {
                /* Detect vector entry on normal steps too (interrupt taken) */
                if (verbose_trace && cpu.pc != prev_pc + 2 && cpu.pc != prev_pc + 3) {
                    const char *vec = detect_vector(cpu.pc, cpu.vecbase);
                    if (vec)
                        fprintf(stderr, "     [INT] %s vector at 0x%08X\n", vec, cpu.pc);
                }
                exc_repeat = 0;
            }
            if (!cpu.running) {
                stop_reason = STOP_CPU_STOPPED;
                break;
            }
        }
        if (stop_reason == STOP_RUNNING)
            stop_reason = (cycles >= max_cycles) ? STOP_MAX_CYCLES : STOP_CPU_STOPPED;
    } else {
        /* Batch execution in chunks */
        int batch = 10000;
        while (cycles < max_cycles && cpu.running && !cpu.halted && !cpu.breakpoint_hit) {
            int remaining = max_cycles - cycles;
            int n = remaining < batch ? remaining : batch;
            int ran = xtensa_run(&cpu, n);
            cycles += ran;
            if (ran < n) break;
        }
        /* Determine stop reason */
        if (cpu.breakpoint_hit)
            stop_reason = STOP_BREAKPOINT;
        else if (cpu.halted)
            stop_reason = STOP_HALT;
        else if (!cpu.running)
            stop_reason = STOP_CPU_STOPPED;
        else
            stop_reason = STOP_MAX_CYCLES;
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
        format_addr(syms, cpu.breakpoint_hit_addr, sym_buf, sizeof(sym_buf));
        fprintf(stderr, "Stop reason: %s at 0x%08X (%s)\n",
                stop_reason_str(stop_reason), cpu.breakpoint_hit_addr, sym_buf);
    } else {
        fprintf(stderr, "Stop reason: %s\n", stop_reason_str(stop_reason));
    }

    fprintf(stderr, "Cycles:     %d\n", cycles);

    /* Final PC with symbol */
    char pc_sym[128];
    format_addr(syms, cpu.pc, pc_sym, sizeof(pc_sym));
    fprintf(stderr, "Final PC:   0x%08X (%s)\n", cpu.pc, pc_sym);

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

    fprintf(stderr, "Unhandled:  %d peripheral accesses\n", periph_unhandled_count(periph));

    /* Register dump */
    if (verbose || stop_reason == STOP_BREAKPOINT) {
        fprintf(stderr, "\n--- Registers ---\n");
        fprintf(stderr, "PS=0x%08X  SAR=%u  LBEG=0x%08X  LEND=0x%08X  LCOUNT=%u\n",
                cpu.ps, cpu.sar, cpu.lbeg, cpu.lend, cpu.lcount);
        fprintf(stderr, "WINDOWBASE=%u  WINDOWSTART=0x%04X  VECBASE=0x%08X\n",
                cpu.windowbase, cpu.windowstart, cpu.vecbase);
        fprintf(stderr, "EXCCAUSE=%u  EXCVADDR=0x%08X\n", cpu.exccause, cpu.excvaddr);
        for (int i = 0; i < 16; i += 4)
            fprintf(stderr, "a%d=0x%08X  a%d=0x%08X  a%d=0x%08X  a%d=0x%08X\n",
                    i, ar_read(&cpu, i), i+1, ar_read(&cpu, i+1),
                    i+2, ar_read(&cpu, i+2), i+3, ar_read(&cpu, i+3));
    }

    /* Memory dumps */
    for (int i = 0; i < dump_count; i++) {
        fprintf(stderr, "\n--- Memory dump: 0x%08X (%u bytes) ---\n",
                mem_dumps[i].addr, mem_dumps[i].len);
        hexdump(mem, mem_dumps[i].addr, mem_dumps[i].len);
    }

    /* Cleanup */
    rom_stubs_destroy(rom);
    periph_destroy(periph);
    mem_destroy(mem);
    elf_symbols_destroy(syms);
    return 0;
}
