#include "hierarchical_trace.h"
#include "xtensa.h"
#include "elf_symbols.h"
#include <stdlib.h>
#include <string.h>

/* Create hierarchical trace system */
hierarchical_trace_t *htrace_create(void) {
    hierarchical_trace_t *ht = calloc(1, sizeof(hierarchical_trace_t));
    if (!ht) return NULL;

    /* Allocate entries for all 16 levels */
    for (int i = 0; i < HTRACE_LEVELS; i++) {
        ht->levels[i].entries = calloc(HTRACE_ENTRIES_PER_LEVEL, sizeof(htrace_entry_t));
        if (!ht->levels[i].entries) {
            /* Cleanup on failure */
            for (int j = 0; j < i; j++)
                free(ht->levels[j].entries);
            free(ht);
            return NULL;
        }
        /* Exponential sampling: 1, 4, 16, 64, 256, 1024, ... */
        ht->levels[i].sample_rate = 1u << (i * 2);
        ht->levels[i].next_sample = 0;
        ht->levels[i].write_index = 0;
        ht->levels[i].total_written = 0;
    }

    ht->enabled = true;
    ht->dump_on_halt = true;
    ht->call_depth = 0;
    ht->total_instructions = 0;
    ht->total_calls = 0;
    ht->total_exceptions = 0;

    return ht;
}

/* Destroy hierarchical trace system */
void htrace_destroy(hierarchical_trace_t *ht) {
    if (!ht) return;

    for (int i = 0; i < HTRACE_LEVELS; i++)
        free(ht->levels[i].entries);

    free(ht);
}

/* ULTRA-FAST PC hook for batch-mode tracing
 * This runs on EVERY instruction, so it must be blazingly fast.
 * Optimizations:
 * - No instruction fetch (we don't need the opcode)
 * - No call classification (reconstruct call stack from dump)
 * - No ar_read for a1 on every instruction (only at dump time)
 * - Inline-friendly small function
 * - Early exit for levels that don't need sampling
 */
int htrace_pc_hook(xtensa_cpu_t *cpu, uint32_t pc, void *ctx) {
    hierarchical_trace_t *ht = (hierarchical_trace_t *)ctx;
    if (!ht || !ht->enabled) return 0;

    uint64_t cycle = cpu->cycle_count;

    /* Fast path: only write to levels that are due for sampling
     * Most instructions will only write to 1-2 levels, not all 16 */

    /* Level 0 (every instruction) - most common case */
    {
        htrace_level_t *lv = &ht->levels[0];
        if (cycle >= lv->next_sample) {
            uint32_t idx = lv->write_index;
            htrace_entry_t *e = &lv->entries[idx];
            e->pc = pc;
            e->cycle = cycle;
            e->instruction = 0;  /* Don't fetch - saves time */
            e->a1 = 0;           /* Don't read - saves time */
            e->exception = cpu->exception ? (uint8_t)cpu->exccause : 0;
            e->call_depth = ht->call_depth;
            e->flags = 0;
            lv->write_index = (idx + 1) & (HTRACE_ENTRIES_PER_LEVEL - 1);
            lv->total_written++;
            lv->next_sample = cycle + 1;
        }
    }

    /* Levels 1-15 (sampled) - loop only if needed */
    for (int i = 1; i < HTRACE_LEVELS; i++) {
        htrace_level_t *lv = &ht->levels[i];
        if (cycle >= lv->next_sample) {
            uint32_t idx = lv->write_index;
            htrace_entry_t *e = &lv->entries[idx];
            e->pc = pc;
            e->cycle = cycle;
            e->instruction = 0;
            e->a1 = 0;
            e->exception = cpu->exception ? (uint8_t)cpu->exccause : 0;
            e->call_depth = ht->call_depth;
            e->flags = 0;
            lv->write_index = (idx + 1) & (HTRACE_ENTRIES_PER_LEVEL - 1);
            lv->total_written++;
            lv->next_sample = cycle + lv->sample_rate;
        } else {
            /* Early exit: if this level isn't ready, higher levels won't be either
             * (since they have larger sample rates) */
            break;
        }
    }

    ht->total_instructions++;
    return 0;  /* Don't skip normal execution */
}

/* Legacy recording function (for single-step mode compatibility) */
void htrace_record(hierarchical_trace_t *ht, const xtensa_cpu_t *cpu,
                   uint32_t instruction, uint16_t flags) {
    if (!ht || !ht->enabled) return;

    uint64_t cycle = cpu->cycle_count;

    /* Write to all levels that are due for sampling */
    for (int i = 0; i < HTRACE_LEVELS; i++) {
        htrace_level_t *level = &ht->levels[i];

        if (cycle >= level->next_sample) {
            uint32_t idx = level->write_index;
            htrace_entry_t *e = &level->entries[idx];

            /* Fast struct copy */
            e->pc = cpu->pc;
            e->instruction = instruction;
            e->cycle = cycle;
            e->a1 = ar_read(cpu, 1);  /* Stack pointer */
            e->exception = cpu->exception ? (uint8_t)cpu->exccause : 0;
            e->call_depth = ht->call_depth;
            e->flags = flags;

            /* Advance circular buffer (power-of-2 optimization) */
            level->write_index = (idx + 1) & (HTRACE_ENTRIES_PER_LEVEL - 1);
            level->total_written++;
            level->next_sample = cycle + level->sample_rate;
        }
    }

    ht->total_instructions++;
    if (flags & (HTRACE_FLAG_CALL | HTRACE_FLAG_CALLX))
        ht->total_calls++;
    if (flags & HTRACE_FLAG_EXCEPTION)
        ht->total_exceptions++;
}

/* Push a call onto the call stack */
void htrace_push_call(hierarchical_trace_t *ht, uint32_t return_pc, uint32_t sp) {
    if (!ht) return;

    if (ht->call_depth < HTRACE_MAX_CALL_DEPTH) {
        ht->call_stack_pc[ht->call_depth] = return_pc;
        ht->call_stack_a1[ht->call_depth] = sp;
        ht->call_depth++;
    }
}

/* Pop a call from the call stack */
void htrace_pop_call(hierarchical_trace_t *ht) {
    if (!ht) return;

    if (ht->call_depth > 0)
        ht->call_depth--;
}

/* Helper: format a PC address with symbol if available */
static void format_pc_with_sym(uint32_t pc, const elf_symbols_t *syms, char *buf, int bufsize) {
    if (!syms) {
        snprintf(buf, (size_t)bufsize, "0x%08X", pc);
        return;
    }

    elf_sym_info_t sym;
    if (elf_symbols_lookup(syms, pc, &sym)) {
        if (sym.offset > 0)
            snprintf(buf, (size_t)bufsize, "%s+0x%X", sym.name, sym.offset);
        else
            snprintf(buf, (size_t)bufsize, "%s", sym.name);
    } else {
        snprintf(buf, (size_t)bufsize, "0x%08X", pc);
    }
}

/* Helper: format coverage duration for a level */
static void format_coverage(uint64_t num_instructions, uint32_t sample_rate,
                           char *buf, int bufsize) {
    uint64_t total_cycles = num_instructions * sample_rate;

    /* Assume 160 MHz CPU (ESP32 default) */
    double seconds = (double)total_cycles / 160000000.0;

    if (seconds < 0.001)
        snprintf(buf, (size_t)bufsize, "%.1f us", seconds * 1000000.0);
    else if (seconds < 1.0)
        snprintf(buf, (size_t)bufsize, "%.1f ms", seconds * 1000.0);
    else if (seconds < 60.0)
        snprintf(buf, (size_t)bufsize, "%.1f sec", seconds);
    else if (seconds < 3600.0)
        snprintf(buf, (size_t)bufsize, "%.1f min", seconds / 60.0);
    else
        snprintf(buf, (size_t)bufsize, "%.1f hr", seconds / 3600.0);
}

/* Dump a single level of trace */
void htrace_dump_level(const hierarchical_trace_t *ht, int level, const elf_symbols_t *syms,
                       FILE *out, int max_entries) {
    if (!ht || level < 0 || level >= HTRACE_LEVELS) return;

    const htrace_level_t *lv = &ht->levels[level];

    /* Calculate how many entries we have */
    uint32_t available = (lv->total_written < HTRACE_ENTRIES_PER_LEVEL)
                        ? (uint32_t)lv->total_written
                        : HTRACE_ENTRIES_PER_LEVEL;

    if (available == 0) {
        fprintf(out, "=== LEVEL %d (empty) ===\n\n", level);
        return;
    }

    /* Limit output if requested */
    uint32_t to_show = available;
    if (max_entries > 0 && to_show > (uint32_t)max_entries)
        to_show = (uint32_t)max_entries;

    /* Calculate coverage */
    char coverage_buf[64];
    format_coverage(available, lv->sample_rate, coverage_buf, sizeof(coverage_buf));

    fprintf(out, "=== LEVEL %d (sample rate: 1/%u, coverage: %s, %u entries) ===\n",
            level, lv->sample_rate, coverage_buf, to_show);

    /* Figure out starting index (show most recent entries) */
    uint32_t start_offset = available - to_show;

    for (uint32_t i = 0; i < to_show; i++) {
        uint32_t idx;
        if (lv->total_written < HTRACE_ENTRIES_PER_LEVEL) {
            /* Haven't wrapped yet */
            idx = start_offset + i;
        } else {
            /* Wrapped - calculate circular index */
            uint32_t offset_from_head = (uint32_t)(lv->total_written - to_show + i);
            idx = offset_from_head & (HTRACE_ENTRIES_PER_LEVEL - 1);
        }

        const htrace_entry_t *e = &lv->entries[idx];

        /* Format PC with symbol */
        char pc_buf[128];
        format_pc_with_sym(e->pc, syms, pc_buf, sizeof(pc_buf));

        fprintf(out, "[%12llu] %s",
                (unsigned long long)e->cycle, pc_buf);

        if (e->call_depth > 0)
            fprintf(out, "  depth=%u", e->call_depth);

        fprintf(out, "\n");
    }
    fprintf(out, "\n");
}

/* Dump comprehensive crash report */
void htrace_dump_on_crash(const hierarchical_trace_t *ht, const elf_symbols_t *syms, FILE *out) {
    if (!ht) return;

    fprintf(out, "\n=== HIERARCHICAL TRACE DUMP ===\n");
    fprintf(out, "Total instructions traced: %llu\n", (unsigned long long)ht->total_instructions);
    fprintf(out, "Total calls: %llu\n", (unsigned long long)ht->total_calls);
    fprintf(out, "Total exceptions: %llu\n", (unsigned long long)ht->total_exceptions);
    fprintf(out, "Call depth at halt: %u\n\n", ht->call_depth);

    /* Dump call stack */
    if (ht->call_depth > 0) {
        fprintf(out, "=== CALL STACK (%u levels deep) ===\n", ht->call_depth);
        for (int i = (int)ht->call_depth - 1; i >= 0; i--) {
            char pc_buf[128];
            format_pc_with_sym(ht->call_stack_pc[i], syms, pc_buf, sizeof(pc_buf));
            fprintf(out, "#%-2d  %s (sp=0x%08X)\n",
                    i, pc_buf, ht->call_stack_a1[i]);
        }
        fprintf(out, "\n");
    }

    /* Dump level 0 completely (last 65K instructions) */
    htrace_dump_level(ht, 0, syms, out, 0);  /* 0 = show all */

    /* Dump levels 1-15 with limited output (last 1K entries each) */
    for (int i = 1; i < HTRACE_LEVELS; i++) {
        htrace_dump_level(ht, i, syms, out, 1000);
    }
}

/* Print statistics */
void htrace_print_stats(const hierarchical_trace_t *ht, FILE *out) {
    if (!ht) return;

    fprintf(out, "\n=== HIERARCHICAL TRACE STATISTICS ===\n");
    fprintf(out, "Total instructions: %llu\n", (unsigned long long)ht->total_instructions);
    fprintf(out, "Total calls: %llu\n", (unsigned long long)ht->total_calls);
    fprintf(out, "Total exceptions: %llu\n\n", (unsigned long long)ht->total_exceptions);

    fprintf(out, "Level  Sample Rate  Entries Written  Buffer Usage\n");
    fprintf(out, "-----  -----------  ---------------  ------------\n");
    for (int i = 0; i < HTRACE_LEVELS; i++) {
        const htrace_level_t *lv = &ht->levels[i];
        uint32_t usage = (lv->total_written < HTRACE_ENTRIES_PER_LEVEL)
                        ? (uint32_t)lv->total_written
                        : HTRACE_ENTRIES_PER_LEVEL;
        double pct = 100.0 * (double)usage / HTRACE_ENTRIES_PER_LEVEL;

        fprintf(out, "%5d  %11u  %15llu  %5u/%u (%.1f%%)\n",
                i, lv->sample_rate, (unsigned long long)lv->total_written,
                usage, HTRACE_ENTRIES_PER_LEVEL, pct);
    }
    fprintf(out, "\n");
}
