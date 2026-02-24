/*
 * trace-filter: Post-process verbose trace output from xtensa-emu.
 *
 * Usage: xtensa-emu -q -T ... 2>&1 | trace-filter [options]
 *    or: trace-filter [options] < trace.log
 *
 * Options:
 *   -r          Show ROM calls (registered + unregistered)
 *   -e          Show exceptions and interrupts ([INT], [EXC], DoubleExc)
 *   -u          Show only UNREGISTERED ROM calls
 *   -w          Show window spill/underflow events ([SPILL], [UNDERFLOW])
 *   -p          Show panic/abort/assert path
 *   -s SYMBOL   Show all instructions in the named function
 *   -a ADDR     Show instructions at this hex address (prefix match)
 *   -c N        Show N lines of context around each match
 *   -S          Show summary (last 30 lines, execution summary)
 *   -R REG      Track a specific register (e.g. "a1" to track SP changes)
 *   -A          Show ALL of the above event types (except -s/-a/-R)
 *   -n          Print line numbers
 *
 * Multiple flags combine (OR): -re shows ROM calls AND exceptions.
 * Without flags, acts as a pass-through (prints everything).
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <ctype.h>

#define MAX_CONTEXT 50
#define MAX_SYMBOLS 16
#define MAX_ADDRS   16
#define MAX_REGS    8
#define LINE_MAX    4096

typedef struct {
    bool show_rom;
    bool show_exc;
    bool show_unreg_only;
    bool show_window;
    bool show_panic;
    bool show_summary;
    bool show_line_nums;
    bool has_any_filter;

    char symbols[MAX_SYMBOLS][128];
    int  symbol_count;

    char addrs[MAX_ADDRS][16];
    int  addr_count;

    char regs[MAX_REGS][8];
    int  reg_count;

    int  context;
} filter_opts_t;

/* Circular buffer for context lines */
typedef struct {
    char lines[MAX_CONTEXT][LINE_MAX];
    int  line_nums[MAX_CONTEXT];
    int  head;
    int  count;
    int  capacity;
} context_buf_t;

static void ctx_init(context_buf_t *cb, int capacity) {
    cb->head = 0;
    cb->count = 0;
    cb->capacity = (capacity > MAX_CONTEXT) ? MAX_CONTEXT : capacity;
}

static void ctx_push(context_buf_t *cb, const char *line, int linenum) {
    int idx = (cb->head + cb->count) % cb->capacity;
    if (cb->count == cb->capacity) {
        cb->head = (cb->head + 1) % cb->capacity;
    } else {
        cb->count++;
    }
    strncpy(cb->lines[idx], line, LINE_MAX - 1);
    cb->lines[idx][LINE_MAX - 1] = '\0';
    cb->line_nums[idx] = linenum;
}

static void ctx_flush(context_buf_t *cb, bool show_nums) {
    for (int i = 0; i < cb->count; i++) {
        int idx = (cb->head + i) % cb->capacity;
        if (show_nums)
            printf("%7d: %s", cb->line_nums[idx], cb->lines[idx]);
        else
            printf("%s", cb->lines[idx]);
    }
    cb->head = 0;
    cb->count = 0;
}

static bool match_line(const char *line, const filter_opts_t *opts) {
    /* ROM calls */
    if (opts->show_unreg_only && strstr(line, "UNREGISTERED"))
        return true;
    if (opts->show_rom && strstr(line, "[ROM]"))
        return true;

    /* Exceptions/interrupts */
    if (opts->show_exc) {
        if (strstr(line, "[INT]") || strstr(line, "[EXC]") ||
            strstr(line, "DoubleExc") || strstr(line, "[STOP]"))
            return true;
    }

    /* Window events */
    if (opts->show_window) {
        if (strstr(line, "[SPILL]") || strstr(line, "[UNDERFLOW]") ||
            strstr(line, "WindowOverflow") || strstr(line, "WindowUnderflow"))
            return true;
    }

    /* Panic/abort path */
    if (opts->show_panic) {
        if (strstr(line, "panic") || strstr(line, "abort") ||
            strstr(line, "__assert") || strstr(line, "esp_restart") ||
            strstr(line, "_xt_panic"))
            return true;
    }

    /* Symbol match */
    for (int i = 0; i < opts->symbol_count; i++) {
        if (strstr(line, opts->symbols[i]))
            return true;
    }

    /* Address match - match [ADDR] at start of instruction lines */
    for (int i = 0; i < opts->addr_count; i++) {
        /* Look for [XXXXXXXX] pattern containing our address prefix */
        const char *bracket = strchr(line, '[');
        if (bracket && bracket[9] == ']') {
            if (strncasecmp(bracket + 1, opts->addrs[i],
                           strlen(opts->addrs[i])) == 0)
                return true;
        }
    }

    /* Register tracking */
    for (int i = 0; i < opts->reg_count; i++) {
        /* Match "     a1 : 0x... -> 0x..." or "     PS:  0x..." */
        const char *p = line;
        while (*p == ' ') p++;
        size_t rlen = strlen(opts->regs[i]);
        if (strncmp(p, opts->regs[i], rlen) == 0) {
            char next = p[rlen];
            if (next == ' ' || next == ':')
                return true;
        }
    }

    /* Summary lines */
    if (opts->show_summary) {
        if (strstr(line, "--- Execution") || strstr(line, "Stop reason") ||
            strstr(line, "Cycles:") || strstr(line, "Final PC") ||
            strstr(line, "UART TX") || strstr(line, "ROM calls:") ||
            strstr(line, "Unregistered") || strstr(line, "Unhandled") ||
            strstr(line, "Loaded ") || strstr(line, "Segment "))
            return true;
    }

    return false;
}

static void usage(const char *prog) {
    fprintf(stderr,
        "Usage: %s [options] < trace.log\n"
        "  -r        ROM calls (all)\n"
        "  -u        Unregistered ROM calls only\n"
        "  -e        Exceptions/interrupts\n"
        "  -w        Window spill/underflow\n"
        "  -p        Panic/abort/assert path\n"
        "  -S        Execution summary\n"
        "  -s SYM    Instructions in function SYM\n"
        "  -a ADDR   Instructions at hex address (prefix)\n"
        "  -R REG    Track register changes (e.g. a1, PS)\n"
        "  -c N      Context lines around matches\n"
        "  -A        All event types\n"
        "  -n        Show line numbers\n"
        "\nMultiple flags combine (OR).\n",
        prog);
}

int main(int argc, char *argv[]) {
    filter_opts_t opts = {0};
    opts.context = 0;

    for (int i = 1; i < argc; i++) {
        if (argv[i][0] != '-') {
            /* Try to open as file */
            if (!freopen(argv[i], "r", stdin)) {
                fprintf(stderr, "Cannot open %s\n", argv[i]);
                return 1;
            }
            continue;
        }
        for (const char *f = argv[i] + 1; *f; f++) {
            switch (*f) {
            case 'r': opts.show_rom = true; opts.has_any_filter = true; break;
            case 'e': opts.show_exc = true; opts.has_any_filter = true; break;
            case 'u': opts.show_unreg_only = true; opts.has_any_filter = true; break;
            case 'w': opts.show_window = true; opts.has_any_filter = true; break;
            case 'p': opts.show_panic = true; opts.has_any_filter = true; break;
            case 'S': opts.show_summary = true; opts.has_any_filter = true; break;
            case 'n': opts.show_line_nums = true; break;
            case 'A':
                opts.show_rom = opts.show_exc = opts.show_window = true;
                opts.show_panic = opts.show_summary = true;
                opts.has_any_filter = true;
                break;
            case 's':
                if (++i < argc && opts.symbol_count < MAX_SYMBOLS) {
                    strncpy(opts.symbols[opts.symbol_count], argv[i], 127);
                    opts.symbol_count++;
                    opts.has_any_filter = true;
                }
                goto next_arg;
            case 'a':
                if (++i < argc && opts.addr_count < MAX_ADDRS) {
                    strncpy(opts.addrs[opts.addr_count], argv[i], 15);
                    opts.addr_count++;
                    opts.has_any_filter = true;
                }
                goto next_arg;
            case 'R':
                if (++i < argc && opts.reg_count < MAX_REGS) {
                    strncpy(opts.regs[opts.reg_count], argv[i], 7);
                    opts.reg_count++;
                    opts.has_any_filter = true;
                }
                goto next_arg;
            case 'c':
                if (++i < argc)
                    opts.context = atoi(argv[i]);
                goto next_arg;
            case 'h':
                usage(argv[0]);
                return 0;
            default:
                fprintf(stderr, "Unknown flag: -%c\n", *f);
                usage(argv[0]);
                return 1;
            }
        }
        next_arg:;
    }

    if (!opts.has_any_filter) {
        /* No filters — pass through everything */
        char line[LINE_MAX];
        while (fgets(line, sizeof(line), stdin))
            fputs(line, stdout);
        return 0;
    }

    context_buf_t ctx;
    ctx_init(&ctx, opts.context > 0 ? opts.context : 1);

    char line[LINE_MAX];
    int linenum = 0;
    int after_count = 0;  /* lines remaining to print after a match */
    bool last_was_match = false;
    bool printed_separator = true;

    while (fgets(line, sizeof(line), stdin)) {
        linenum++;

        if (match_line(line, &opts)) {
            /* Print before-context */
            if (opts.context > 0 && !last_was_match && !printed_separator) {
                printf("---\n");
            }
            ctx_flush(&ctx, opts.show_line_nums);

            if (opts.show_line_nums)
                printf("%7d> %s", linenum, line);
            else
                printf("%s", line);

            after_count = opts.context;
            last_was_match = true;
            printed_separator = false;
        } else if (after_count > 0) {
            /* After-context */
            if (opts.show_line_nums)
                printf("%7d: %s", linenum, line);
            else
                printf("%s", line);
            after_count--;
            last_was_match = false;
        } else {
            if (opts.context > 0)
                ctx_push(&ctx, line, linenum);
            last_was_match = false;
        }
    }

    return 0;
}
