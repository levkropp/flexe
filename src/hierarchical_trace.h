#ifndef HIERARCHICAL_TRACE_H
#define HIERARCHICAL_TRACE_H

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

/* Forward declarations */
typedef struct xtensa_cpu xtensa_cpu_t;
typedef struct elf_symbols elf_symbols_t;

/* Binary trace entry - optimized for speed and cache alignment */
#ifdef _MSC_VER
#pragma pack(push, 1)
#endif
typedef struct {
    uint32_t pc;              /* Program counter */
    uint32_t instruction;     /* Raw instruction word */
    uint64_t cycle;           /* Cycle count when executed */
    uint32_t a1;              /* Stack pointer (critical for debugging) */
    uint8_t  exception;       /* Exception cause (0 = none) */
    uint16_t call_depth;      /* Call stack depth */
    uint16_t flags;           /* CALL/RET/EXCEPTION/WINDOW/etc. */
}
#ifndef _MSC_VER
__attribute__((packed))
#endif
htrace_entry_t;  /* 24 bytes - cache-line friendly */
#ifdef _MSC_VER
#pragma pack(pop)
#endif

/* Trace entry flags */
#define HTRACE_FLAG_CALL       (1 << 0)
#define HTRACE_FLAG_CALLX      (1 << 1)
#define HTRACE_FLAG_RET        (1 << 2)
#define HTRACE_FLAG_RETW       (1 << 3)
#define HTRACE_FLAG_ENTRY      (1 << 4)
#define HTRACE_FLAG_EXCEPTION  (1 << 5)
#define HTRACE_FLAG_INTERRUPT  (1 << 6)
#define HTRACE_FLAG_WINDOW     (1 << 7)

/* Per-level ring buffer (64K entries = 1.5 MB per level) */
#define HTRACE_ENTRIES_PER_LEVEL 65536
#define HTRACE_LEVELS 16

typedef struct {
    htrace_entry_t *entries;  /* Pre-allocated array [HTRACE_ENTRIES_PER_LEVEL] */
    uint32_t write_index;     /* Circular write pointer */
    uint64_t total_written;   /* Total instructions written to this level */
    uint32_t sample_rate;     /* 1, 4, 16, 64, ... (exponential) */
    uint64_t next_sample;     /* Next cycle to sample at this level */
} htrace_level_t;

/* Call stack tracking (256 deep for complex firmware) */
#define HTRACE_MAX_CALL_DEPTH 256

typedef struct {
    uint32_t call_stack_pc[HTRACE_MAX_CALL_DEPTH];   /* Return addresses */
    uint32_t call_stack_a1[HTRACE_MAX_CALL_DEPTH];   /* Stack pointer at each level */
    uint16_t call_depth;                              /* Current depth */

    htrace_level_t levels[HTRACE_LEVELS];

    /* Statistics */
    uint64_t total_instructions;
    uint64_t total_calls;
    uint64_t total_exceptions;

    /* Flags */
    bool enabled;
    bool dump_on_halt;
} hierarchical_trace_t;

/* Lifecycle */
hierarchical_trace_t *htrace_create(void);
void htrace_destroy(hierarchical_trace_t *ht);

/* PC Hook for batch-mode tracing (called every instruction) */
int htrace_pc_hook(xtensa_cpu_t *cpu, uint32_t pc, void *ctx);

/* Recording (hot path - must be EXTREMELY fast) */
void htrace_record(hierarchical_trace_t *ht, const xtensa_cpu_t *cpu,
                   uint32_t instruction, uint16_t flags);

/* Call stack management */
void htrace_push_call(hierarchical_trace_t *ht, uint32_t return_pc, uint32_t sp);
void htrace_pop_call(hierarchical_trace_t *ht);

/* Dumping */
void htrace_dump_on_crash(const hierarchical_trace_t *ht, const elf_symbols_t *syms, FILE *out);
void htrace_dump_level(const hierarchical_trace_t *ht, int level, const elf_symbols_t *syms,
                       FILE *out, int max_entries);

/* Statistics */
void htrace_print_stats(const hierarchical_trace_t *ht, FILE *out);

#endif /* HIERARCHICAL_TRACE_H */
