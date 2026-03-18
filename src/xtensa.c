#ifdef _MSC_VER
#include "msvc_compat.h"
#endif

#include "xtensa.h"
#include "memory.h"
#include "rom_stubs.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

/* Set PC from instruction handler (branch/call/ret/exception) and mark it */
#define BRANCH_TO(cpu, addr) do { (cpu)->pc = (addr); (cpu)->_pc_written = true; } while(0)

/* Recompute the nearest ccompare value for timer batching.
 * We pick the ccompare that will fire soonest AFTER current ccount.
 * If none are ahead, we use 0 (which means "check every cycle" is impossible
 * since ccount is 0 only once in 2^32 cycles — effectively a no-op). */
static inline void xtensa_recompute_next_timer(xtensa_cpu_t *cpu) {
    /* Distance from ccount to each ccompare (wrapping arithmetic).
     * 0 distance means "just matched" (already fired) — treat as max. */
    uint32_t d0 = cpu->ccompare[0] - cpu->ccount;
    uint32_t d1 = cpu->ccompare[1] - cpu->ccount;
    uint32_t d2 = cpu->ccompare[2] - cpu->ccount;
    if (d0 == 0) d0 = UINT32_MAX;
    if (d1 == 0) d1 = UINT32_MAX;
    if (d2 == 0) d2 = UINT32_MAX;
    if (d0 <= d1 && d0 <= d2)
        cpu->next_timer_event = cpu->ccompare[0];
    else if (d1 <= d2)
        cpu->next_timer_event = cpu->ccompare[1];
    else
        cpu->next_timer_event = cpu->ccompare[2];
}

/* Fire any matching ccompare timers and recompute next event */
static inline void xtensa_fire_timers(xtensa_cpu_t *cpu) {
    uint32_t old = cpu->interrupt;
    if (cpu->ccount == cpu->ccompare[0]) cpu->interrupt |= (1u << 6);
    if (cpu->ccount == cpu->ccompare[1]) cpu->interrupt |= (1u << 15);
    if (cpu->ccount == cpu->ccompare[2]) cpu->interrupt |= (1u << 16);
    if (cpu->interrupt != old) cpu->irq_check = true;
    xtensa_recompute_next_timer(cpu);
}

void xtensa_cpu_init(xtensa_cpu_t *cpu) {
    memset(cpu, 0, sizeof(*cpu));
    cpu->core_id = 0;
    cpu->next_timer_event = UINT32_MAX;  /* No timer pending until ccompare is written */

    /* ESP32 CPU interrupt level table (matches hardware):
     * Level 1: 0-5, 8-10, 12-13, 17-18
     * Level 2: 19-21
     * Level 3: 11, 15, 22-23, 27, 29
     * Level 4: 24-25, 28, 30
     * Level 5: 16, 26, 31
     * Level 6: 14 (debug)
     * Level 7: 7 (NMI — software, keep at 1 for compat) */
    static const uint8_t esp32_int_level[32] = {
        1, 1, 1, 1, 1, 1, 1, 1,   /* 0-7:   all level 1 (int 7 = software/NMI) */
        1, 1, 1, 3, 1, 1, 6, 3,   /* 8-15:  11=L3, 14=debug(L6), 15=L3(CCOMPARE1) */
        5, 1, 1, 2, 2, 2, 3, 3,   /* 16-23: 16=L5(CCOMPARE2), 19-21=L2, 22-23=L3 */
        4, 4, 5, 3, 4, 3, 4, 5,   /* 24-31: see ESP32 TRM Table 1-4 */
    };
    for (int i = 0; i < 32; i++)
        cpu->int_level[i] = esp32_int_level[i];
}


void xtensa_cpu_reset(xtensa_cpu_t *cpu) {
    xtensa_cpu_init(cpu);

    /* ESP32 reset vector */
    cpu->pc = 0x40000400;

    /* PS: WOE=1, EXCM=1, INTLEVEL=15 */
    cpu->ps = (1 << 18)    /* WOE */
            | (1 << 4)     /* EXCM */
            | 0xF;         /* INTLEVEL = 15 */

    /* Window registers */
    cpu->windowbase = 0;
    cpu->windowstart = 1;   /* Window 0 is valid */

    /* SAR undefined, set to 0 */
    cpu->sar = 0;
    cpu->lcount = 0;
    cpu->ccount = 0;

    /* ESP32 defaults */
    cpu->vecbase = 0x40000000;
    cpu->prid = 0xCDCD;        /* PRO_CPU */
    cpu->cpenable = 0;
    cpu->atomctl = 0x28;
    cpu->configid0 = 0;
    cpu->configid1 = 0;

    cpu->running = true;
}

/* Fast inline fetch for the hot path — avoids function call overhead */
static inline __attribute__((always_inline))
int xtensa_fetch_inline(const xtensa_cpu_t *cpu, uint32_t addr, uint32_t *insn_out) {
    uint8_t *page = cpu->mem->page_table[addr >> 12];
    if (__builtin_expect(!page, 0)) return 0;
    const uint8_t *ptr = page + (addr & 0xFFF);
    if (ptr[0] & 0x8) {
        *insn_out = ptr[0] | ((uint32_t)ptr[1] << 8);
        return 2;
    } else {
        *insn_out = ptr[0] | ((uint32_t)ptr[1] << 8) | ((uint32_t)ptr[2] << 16);
        return 3;
    }
}

/* External version for disasm/trace callers (not performance-critical) */
int xtensa_fetch(const xtensa_cpu_t *cpu, uint32_t addr, uint32_t *insn_out) {
    return xtensa_fetch_inline(cpu, addr, insn_out);
}

/* Pre-decode entire instruction memory at load time.
 * Replaces per-instruction page_table lookup + byte assembly with
 * a single indexed load from a flat array. */
void xtensa_predecode_build(xtensa_cpu_t *cpu) {
#if PREDECODE_SIZE == 0
    (void)cpu;
    return;
#else
    if (!cpu->predecode) {
        size_t sz = (size_t)PREDECODE_SIZE * sizeof(uint32_t);
        cpu->predecode = calloc(PREDECODE_SIZE, sizeof(uint32_t));
        if (!cpu->predecode) {
            fprintf(stderr, "[predecode] Failed to allocate %zu MB table\n",
                    sz / (1024*1024));
            return;
        }
    }

    memset(cpu->predecode, 0, PREDECODE_SIZE * sizeof(uint32_t));
    uint32_t count = 0;
    for (uint32_t addr = PREDECODE_BASE; addr < PREDECODE_END; addr++) {
        uint32_t insn;
        int ilen = xtensa_fetch_inline(cpu, addr, &insn);
        if (ilen > 0) {
            cpu->predecode[addr - PREDECODE_BASE] = PREDECODE_PACK(insn, (uint32_t)ilen);
            count++;
        }
    }
    fprintf(stderr, "[predecode] Built table: %u entries, %u MB (%u MB flash coverage)\n",
            count, (uint32_t)(PREDECODE_SIZE * 4 / (1024*1024)), PREDECODE_FLASH_MB);
#endif
}

/* ===== Special Register Access ===== */

uint32_t sr_read(const xtensa_cpu_t *cpu, int sr) {
    switch (sr) {
    case XT_SR_LBEG:        return cpu->lbeg;
    case XT_SR_LEND:        return cpu->lend;
    case XT_SR_LCOUNT:      return cpu->lcount;
    case XT_SR_SAR:         return cpu->sar;
    case XT_SR_BR:          return cpu->br;
    case XT_SR_LITBASE:     return cpu->litbase;
    case XT_SR_SCOMPARE1:   return cpu->scompare1;
    case XT_SR_ACCLO:       return cpu->acclo;
    case XT_SR_ACCHI:       return cpu->acchi;
    case XT_SR_MR0:         return cpu->mr[0];
    case XT_SR_MR1:         return cpu->mr[1];
    case XT_SR_MR2:         return cpu->mr[2];
    case XT_SR_MR3:         return cpu->mr[3];
    case XT_SR_WINDOWBASE:  return cpu->windowbase;
    case XT_SR_WINDOWSTART: return cpu->windowstart;
    case XT_SR_IBREAKENABLE:return cpu->ibreakenable;
    case XT_SR_MEMCTL:      return cpu->memctl;
    case XT_SR_ATOMCTL:     return cpu->atomctl;
    case XT_SR_IBREAKA0:    return cpu->ibreaka[0];
    case XT_SR_IBREAKA1:    return cpu->ibreaka[1];
    case XT_SR_DBREAKA0:    return cpu->dbreaka[0];
    case XT_SR_DBREAKA1:    return cpu->dbreaka[1];
    case XT_SR_DBREAKC0:    return cpu->dbreakc[0];
    case XT_SR_DBREAKC1:    return cpu->dbreakc[1];
    case XT_SR_CONFIGID0:   return cpu->configid0;
    case XT_SR_EPC1:        return cpu->epc[0];
    case XT_SR_EPC2:        return cpu->epc[1];
    case XT_SR_EPC3:        return cpu->epc[2];
    case XT_SR_EPC4:        return cpu->epc[3];
    case XT_SR_EPC5:        return cpu->epc[4];
    case XT_SR_EPC6:        return cpu->epc[5];
    case XT_SR_EPC7:        return cpu->epc[6];
    case XT_SR_DEPC:        return cpu->depc;
    case XT_SR_EPS2:        return cpu->eps[1];
    case XT_SR_EPS3:        return cpu->eps[2];
    case XT_SR_EPS4:        return cpu->eps[3];
    case XT_SR_EPS5:        return cpu->eps[4];
    case XT_SR_EPS6:        return cpu->eps[5];
    case XT_SR_EPS7:        return cpu->eps[6];
    case XT_SR_CONFIGID1:   return cpu->configid1;
    case XT_SR_EXCSAVE1:    return cpu->excsave[0];
    case XT_SR_EXCSAVE2:    return cpu->excsave[1];
    case XT_SR_EXCSAVE3:    return cpu->excsave[2];
    case XT_SR_EXCSAVE4:    return cpu->excsave[3];
    case XT_SR_EXCSAVE5:    return cpu->excsave[4];
    case XT_SR_EXCSAVE6:    return cpu->excsave[5];
    case XT_SR_EXCSAVE7:    return cpu->excsave[6];
    case XT_SR_CPENABLE:    return cpu->cpenable;
    case XT_SR_INTSET:   return cpu->interrupt;
    case XT_SR_INTENABLE:   return cpu->intenable;
    case XT_SR_PS:          return cpu->ps;
    case XT_SR_VECBASE:     return cpu->vecbase;
    case XT_SR_EXCCAUSE:    return cpu->exccause;
    case XT_SR_DEBUGCAUSE:  return cpu->debugcause;
    case XT_SR_CCOUNT:      return cpu->ccount;
    case XT_SR_PRID:        return cpu->prid;
    case XT_SR_ICOUNT:      return cpu->icount;
    case XT_SR_ICOUNTLEVEL: return cpu->icountlevel;
    case XT_SR_EXCVADDR:    return cpu->excvaddr;
    case XT_SR_CCOMPARE0:   return cpu->ccompare[0];
    case XT_SR_CCOMPARE1:   return cpu->ccompare[1];
    case XT_SR_CCOMPARE2:   return cpu->ccompare[2];
    case XT_SR_MISC0:       return cpu->misc[0];
    case XT_SR_MISC1:       return cpu->misc[1];
    case XT_SR_MISC2:       return cpu->misc[2];
    case XT_SR_MISC3:       return cpu->misc[3];
    default:                return 0;
    }
}

void sr_write(xtensa_cpu_t *cpu, int sr, uint32_t val) {
    switch (sr) {
    case XT_SR_LBEG:        cpu->lbeg = val; break;
    case XT_SR_LEND:        cpu->lend = val; break;
    case XT_SR_LCOUNT:      cpu->lcount = val; break;
    case XT_SR_SAR:         cpu->sar = val & 0x3F; break;
    case XT_SR_BR:          cpu->br = val & 0xFFFF; break;
    case XT_SR_LITBASE:     cpu->litbase = val; break;
    case XT_SR_SCOMPARE1:   cpu->scompare1 = val; break;
    case XT_SR_ACCLO:       cpu->acclo = val; break;
    case XT_SR_ACCHI:       cpu->acchi = val & 0xFF; break;
    case XT_SR_MR0:         cpu->mr[0] = val; break;
    case XT_SR_MR1:         cpu->mr[1] = val; break;
    case XT_SR_MR2:         cpu->mr[2] = val; break;
    case XT_SR_MR3:         cpu->mr[3] = val; break;
    case XT_SR_WINDOWBASE:  cpu->windowbase = val & 0xF; break;
    case XT_SR_WINDOWSTART: cpu->windowstart = val & 0xFFFF; break;
    case XT_SR_IBREAKENABLE:cpu->ibreakenable = val; break;
    case XT_SR_MEMCTL:      cpu->memctl = val; break;
    case XT_SR_ATOMCTL:     cpu->atomctl = val; break;
    case XT_SR_IBREAKA0:    cpu->ibreaka[0] = val; break;
    case XT_SR_IBREAKA1:    cpu->ibreaka[1] = val; break;
    case XT_SR_DBREAKA0:    cpu->dbreaka[0] = val; break;
    case XT_SR_DBREAKA1:    cpu->dbreaka[1] = val; break;
    case XT_SR_DBREAKC0:    cpu->dbreakc[0] = val; break;
    case XT_SR_DBREAKC1:    cpu->dbreakc[1] = val; break;
    case XT_SR_EPC1:        cpu->epc[0] = val; break;
    case XT_SR_EPC2:        cpu->epc[1] = val; break;
    case XT_SR_EPC3:        cpu->epc[2] = val; break;
    case XT_SR_EPC4:        cpu->epc[3] = val; break;
    case XT_SR_EPC5:        cpu->epc[4] = val; break;
    case XT_SR_EPC6:        cpu->epc[5] = val; break;
    case XT_SR_EPC7:        cpu->epc[6] = val; break;
    case XT_SR_DEPC:        cpu->depc = val; break;
    case XT_SR_EPS2:        cpu->eps[1] = val; break;
    case XT_SR_EPS3:        cpu->eps[2] = val; break;
    case XT_SR_EPS4:        cpu->eps[3] = val; break;
    case XT_SR_EPS5:        cpu->eps[4] = val; break;
    case XT_SR_EPS6:        cpu->eps[5] = val; break;
    case XT_SR_EPS7:        cpu->eps[6] = val; break;
    case XT_SR_EXCSAVE1:    cpu->excsave[0] = val; break;
    case XT_SR_EXCSAVE2:    cpu->excsave[1] = val; break;
    case XT_SR_EXCSAVE3:    cpu->excsave[2] = val; break;
    case XT_SR_EXCSAVE4:    cpu->excsave[3] = val; break;
    case XT_SR_EXCSAVE5:    cpu->excsave[4] = val; break;
    case XT_SR_EXCSAVE6:    cpu->excsave[5] = val; break;
    case XT_SR_EXCSAVE7:    cpu->excsave[6] = val; break;
    case XT_SR_CPENABLE:    cpu->cpenable = val; break;
    case XT_SR_INTSET:   cpu->interrupt |= val; cpu->irq_check = true; break;
    case XT_SR_INTCLEAR: cpu->interrupt &= ~val; break;
    case XT_SR_INTENABLE:   cpu->intenable = val; cpu->irq_check = true; break;
    case XT_SR_PS:          cpu->ps = val; break;
    case XT_SR_VECBASE:     cpu->vecbase = val; break;
    case XT_SR_EXCCAUSE:    cpu->exccause = val; break;
    case XT_SR_DEBUGCAUSE:  cpu->debugcause = val; break;
    case XT_SR_CCOUNT:      cpu->ccount = val; break;
    case XT_SR_ICOUNT:      cpu->icount = val; break;
    case XT_SR_ICOUNTLEVEL: cpu->icountlevel = val; break;
    case XT_SR_EXCVADDR:    cpu->excvaddr = val; break;
    case XT_SR_CCOMPARE0:   cpu->ccompare[0] = val; cpu->interrupt &= ~(1u << 6);
                            xtensa_recompute_next_timer(cpu); break;
    case XT_SR_CCOMPARE1:   cpu->ccompare[1] = val; cpu->interrupt &= ~(1u << 15);
                            xtensa_recompute_next_timer(cpu); break;
    case XT_SR_CCOMPARE2:   cpu->ccompare[2] = val; cpu->interrupt &= ~(1u << 16);
                            xtensa_recompute_next_timer(cpu); break;
    case XT_SR_MISC0:       cpu->misc[0] = val; break;
    case XT_SR_MISC1:       cpu->misc[1] = val; break;
    case XT_SR_MISC2:       cpu->misc[2] = val; break;
    case XT_SR_MISC3:       cpu->misc[3] = val; break;
    default: break; /* ignore writes to unknown/read-only SRs */
    }
}

/* ===== Exception/Interrupt Dispatch ===== */

void xtensa_raise_exception(xtensa_cpu_t *cpu, int cause, uint32_t fault_pc, uint32_t vaddr) {
    /* Trap: catch exceptions with out-of-range fault PC */
    if (__builtin_expect(fault_pc < 0x40000000u || fault_pc >= 0x40500000u, 0)) {
        fprintf(stderr, "[EXC-TRAP] cause=%d fault_pc=0x%08X vaddr=0x%08X cycle=%llu core=%d\n",
                cause, fault_pc, vaddr, (unsigned long long)cpu->cycle_count, cpu->prid ? 1 : 0);
        fprintf(stderr, "  PS=0x%08X SAR=%u WB=%u WS=0x%X\n",
                cpu->ps, cpu->sar, cpu->windowbase, cpu->windowstart);
        for (int r = 0; r < 16; r += 4)
            fprintf(stderr, "  a%-2d=0x%08X  a%-2d=0x%08X  a%-2d=0x%08X  a%-2d=0x%08X\n",
                    r, ar_read(cpu, r), r+1, ar_read(cpu, r+1),
                    r+2, ar_read(cpu, r+2), r+3, ar_read(cpu, r+3));
    }
    uint32_t vec;
    if (XT_PS_EXCM(cpu->ps)) {
        /* Double exception */
        cpu->depc = fault_pc;
        cpu->exccause = cause;
        cpu->excvaddr = vaddr;
        vec = cpu->vecbase + VECOFS_DOUBLE_EXC;
    } else {
        cpu->epc[0] = fault_pc;   /* EPC1 */
        cpu->exccause = cause;
        cpu->excvaddr = vaddr;
        /* Save UM state before setting EXCM */
        uint32_t user_mode = XT_PS_UM(cpu->ps);
        XT_PS_SET_EXCM(cpu->ps, 1);
        vec = cpu->vecbase + (user_mode ? VECOFS_USER_EXC : VECOFS_KERNEL_EXC);
    }
    if (!mem_get_ptr(cpu->mem, vec)) {
        cpu->exception = true;
        cpu->running = false;
        return;
    }
    BRANCH_TO(cpu, vec);
}

#define EXCMLEVEL 3  /* ESP32 XCHAL_EXCM_LEVEL=3: when EXCM=1, levels 1-3 masked */

void xtensa_check_interrupts(xtensa_cpu_t *cpu) {
    uint32_t pending = cpu->interrupt & cpu->intenable;
    if (!pending) return;

    int eff_level = XT_PS_INTLEVEL(cpu->ps);
    if (XT_PS_EXCM(cpu->ps) && EXCMLEVEL > eff_level)
        eff_level = EXCMLEVEL;

    /* Find highest-level pending interrupt using bit scan */
    int best_level = 0;
    uint32_t tmp = pending;
    while (tmp) {
        int i = __builtin_ctz(tmp);
        int lvl = cpu->int_level[i];
        if (lvl > eff_level && lvl > best_level)
            best_level = lvl;
        tmp &= tmp - 1;  /* clear lowest set bit */
    }
    if (best_level == 0) return;

    if (best_level == 1) {
        /* Level-1: dispatched as exception */
        xtensa_raise_exception(cpu, EXCCAUSE_LEVEL1_INT, cpu->pc, 0);
    } else {
        /* High-priority (levels 2-7) */
        int idx = best_level - 1;
        cpu->epc[idx] = cpu->pc;
        cpu->eps[idx] = cpu->ps;
        XT_PS_SET_INTLEVEL(cpu->ps, best_level);
        XT_PS_SET_EXCM(cpu->ps, 1);

        static const uint32_t vecofs[] = {
            0, 0, VECOFS_LEVEL2_INT, VECOFS_LEVEL3_INT,
            VECOFS_LEVEL4_INT, VECOFS_LEVEL5_INT,
            VECOFS_DEBUG_EXC, VECOFS_NMI
        };
        uint32_t vec = cpu->vecbase + vecofs[best_level];
        if (!mem_get_ptr(cpu->mem, vec)) {
            cpu->exception = true;
            cpu->running = false;
            return;
        }
        BRANCH_TO(cpu, vec);
    }
}

/* ===== Instruction Execution ===== */

/* ===== Windowed Register Helpers ===== */

/*
 * Read a register from a specific physical window (not the current windowbase).
 */
static inline uint32_t phys_read(const xtensa_cpu_t *cpu, int widx, int reg) {
    return cpu->ar[((widx * 4) + reg) & 63];
}

/*
 * Write a register in a specific physical window.
 */
static inline void phys_write(xtensa_cpu_t *cpu, int widx, int reg, uint32_t val) {
    cpu->ar[((widx * 4) + reg) & 63] = val;
}

/*
 * Find the callee window for widx.
 * Derive directly from widx's a0[31:30] (call size encoded in return addr).
 * The callee is always widx + callsize in the window ring.
 * This avoids the bug where searching via WindowStart skips spilled
 * intermediate windows and lands on a distant window sharing the same SP base.
 */
static int find_callee_window(xtensa_cpu_t *cpu, int widx) {
    uint32_t a0 = phys_read(cpu, widx, 0);
    int callsize = (a0 >> 30) & 3;
    if (callsize == 0) {
        /* callsize=0 not valid for windowed calls; search fallback */
        for (int i = 1; i < 16; i++) {
            int w = (widx + i) & 0xF;
            if (cpu->windowstart & (1u << w))
                return w;
        }
        return widx;
    }
    return (widx + callsize) & 0xF;
}

/*
 * Spill (save) registers of window widx to its callee's stack frame.
 * Uses the callee's SP as base pointer (matching hardware overflow convention)
 * and records the base in spill_base[] for underflow restore.
 */
static void synth_spill_window(xtensa_cpu_t *cpu, int widx) {
    uint32_t a0 = phys_read(cpu, widx, 0);
    int callsize = (a0 >> 30) & 3;

    /* Use callee's SP as base (matches real hardware overflow handler) */
    int callee = find_callee_window(cpu, widx);
    uint32_t base = phys_read(cpu, callee, 1);

    if (cpu->window_trace && cpu->window_trace_active) {
        fprintf(stderr, "     [WIN] SPILL w%d (call%d) callee=w%d base=0x%08X"
                " WS=0x%04X a0=0x%08X a1=0x%08X a2=0x%08X a3=0x%08X",
                widx, callsize * 4, callee, base, cpu->windowstart,
                phys_read(cpu, widx, 0), phys_read(cpu, widx, 1),
                phys_read(cpu, widx, 2), phys_read(cpu, widx, 3));
        if (callsize >= 2)
            fprintf(stderr, " a4=0x%08X a5=0x%08X a6=0x%08X a7=0x%08X",
                    phys_read(cpu, widx, 4), phys_read(cpu, widx, 5),
                    phys_read(cpu, widx, 6), phys_read(cpu, widx, 7));
        fprintf(stderr, "\n");
    }

    /* Record where we saved, so underflow can find the data even if
     * the callee's SP changes (e.g. via MOVSP) before RETW. */
    if (cpu->window_trace && cpu->spill_base[widx & 0xF] != 0 && cpu->spill_base[widx & 0xF] != base)
        fprintf(stderr, "     [WIN] spill_base[%d] OVERWRITE 0x%08X -> 0x%08X\n",
                widx & 0xF, cpu->spill_base[widx & 0xF], base);
    cpu->spill_base[widx & 0xF] = base;

    /* Push onto spill stack for correct underflow restore */
    {
        int si = widx & 0xF;
        int d = cpu->spill_stack[si].depth;
        if (d < SPILL_STACK_DEPTH) {
            cpu->spill_stack[si].base[d] = base;
            cpu->spill_stack[si].depth = d + 1;
        } else {
            fprintf(stderr, "[WARN] spill_stack[%d] depth %d exceeds limit %d at PC=0x%08X cycle=%llu\n",
                    si, d, SPILL_STACK_DEPTH, cpu->pc,
                    (unsigned long long)cpu->cycle_count);
        }
    }

    int nregs = 4;
    /* Always save base 4 regs: a0-a3 at base-16 (on stack for firmware compat)
     * AND to CPU-side buffer (for correct restore immune to stack overwrites) */
    for (int i = 0; i < 4; i++)
        mem_write32(cpu->mem, base - 16 + i * 4, phys_read(cpu, widx, i));
    {
        int si0 = widx & 0xF;
        int d0 = cpu->spill_stack[si0].depth - 1; /* depth was already incremented */
        if (d0 >= 0 && d0 < SPILL_STACK_DEPTH) {
            for (int i = 0; i < 4; i++)
                cpu->spill_stack[si0].core[d0][i] = phys_read(cpu, widx, i);
        }
    }

    if (callsize >= 2) {
        nregs = 8;
        /* CALL8+: save a4-a7 to CPU-side buffer (not stack).
         * On real hardware these go to grandparent_sp - 32, but computing
         * the grandparent requires a chain of stack links.  Storing in a
         * CPU buffer avoids corruption when deeper calls overwrite stack. */
        int si2 = widx & 0xF;
        int d2 = cpu->spill_stack[si2].depth - 1; /* depth was already incremented */
        if (d2 >= 0 && d2 < SPILL_STACK_DEPTH) {
            for (int i = 0; i < 4; i++)
                cpu->spill_stack[si2].extra[d2][i] = phys_read(cpu, widx, 4 + i);
        }
    }
    if (callsize == 3) {
        nregs = 12;
        /* CALL12: also save a8-a11 to CPU-side buffer */
        int si2 = widx & 0xF;
        int d2 = cpu->spill_stack[si2].depth - 1;
        if (d2 >= 0 && d2 < SPILL_STACK_DEPTH) {
            for (int i = 0; i < 4; i++)
                cpu->spill_stack[si2].extra[d2][4 + i] = phys_read(cpu, widx, 8 + i);
        }
    }

    /* Save shadow copies for spill/fill verification */
    if (cpu->spill_verify) {
        int si = widx & 0xF;
        /* Warn if overwriting an unfilled spill record */
        if (cpu->spill_shadow[si].count > 0 &&
            cpu->spill_shadow[si].base != base) {
            fprintf(stderr, "[SPILL_OVERWRITE] w%d: old_base=0x%08X new_base=0x%08X"
                    " PC=0x%08X cycle=%llu WS=0x%04X\n",
                    widx, cpu->spill_shadow[si].base, base,
                    cpu->pc, (unsigned long long)cpu->cycle_count,
                    cpu->windowstart);
        }
        cpu->spill_shadow[si].base = base;
        cpu->spill_shadow[si].count = nregs;
        for (int i = 0; i < nregs; i++)
            cpu->spill_shadow[si].regs[i] = phys_read(cpu, widx, i);
    }

    /* Clear windowstart bit for this window */
    cpu->windowstart &= ~(1u << (widx & 0xF));
}

/*
 * Overflow check: called during ENTRY to spill all endangered windows.
 *
 * Two sources of danger:
 * 1. ISA ENTRY check: wb+1..wb+callinc — windows being rotated over
 * 2. Per-instruction WindowCheck: new_wb+1..new_wb+3 — windows the callee
 *    may access via a4-a15 (the ISA checks these on every instruction,
 *    but we don't, so we do it preemptively here)
 *
 * We spill the union: wb+1 through wb+callinc+3 (= new_wb+3).
 */
static void synth_overflow_check(xtensa_cpu_t *cpu, int callinc) {
    int limit = callinc + 3;  /* check wb+1 through wb+callinc+3 */
    if (cpu->window_trace && cpu->window_trace_active) {
        int any = 0;
        for (int i = 1; i <= limit; i++) {
            int w = (cpu->windowbase + i) & 0xF;
            if (cpu->windowstart & (1u << w)) any = 1;
        }
        if (any)
            fprintf(stderr, "     [WIN] OVERFLOW_CHK wb=%d callinc=%d WS=0x%04X\n",
                    cpu->windowbase, callinc, cpu->windowstart);
    }
    for (int i = 1; i <= limit; i++) {
        int w = (cpu->windowbase + i) & 0xF;
        if (cpu->windowstart & (1u << w))
            synth_spill_window(cpu, w);
    }
}

/*
 * Underflow fill: called during RETW when the caller's windowstart
 * bit is clear (registers were spilled and need restoration).
 * Uses the recorded spill_base (where overflow actually saved the data).
 */
static void synth_underflow_fill(xtensa_cpu_t *cpu, int ret_wb, int owb) {
    /* Pop from the spill stack to get the correct base for this nesting level.
     * Window slots can be spilled multiple times when the ring wraps, and each
     * spill pushes onto the stack. The innermost (most recent) spill is on top
     * and corresponds to the next RETW that needs filling. */
    int si = ret_wb & 0xF;
    uint32_t callee_sp = phys_read(cpu, owb, 1);
    uint32_t base;
    int fill_depth = -1; /* depth index used for CPU buffer restore */
    if (cpu->spill_stack[si].depth > 0) {
        cpu->spill_stack[si].depth--;
        int d = cpu->spill_stack[si].depth;
        fill_depth = d;
        base = (d < SPILL_STACK_DEPTH) ? cpu->spill_stack[si].base[d] : callee_sp;
        /* Sanity check: base must be in valid data memory range.
         * If depth tracking is out of sync, we may pop a stale/wrong entry. */
        if (base < 0x3F800000 || base > 0x3FFFFFFF) {
            base = callee_sp;
            fill_depth = -1; /* don't trust CPU buffer either */
        }
    } else {
        base = callee_sp;
    }

    /* Restore a0-a3 from CPU-side buffer (immune to stack overwrites),
     * falling back to stack memory if buffer unavailable */
    if (fill_depth >= 0 && fill_depth < SPILL_STACK_DEPTH) {
        for (int i = 0; i < 4; i++)
            phys_write(cpu, ret_wb, i, cpu->spill_stack[si].core[fill_depth][i]);
    } else {
        for (int i = 0; i < 4; i++)
            phys_write(cpu, ret_wb, i, mem_read32(cpu->mem, base - 16 + i * 4));
    }

    /* Check restored a0 for call size to determine extra regs */
    uint32_t a0 = phys_read(cpu, ret_wb, 0);
    int callsize = (a0 >> 30) & 3;

    if (callsize >= 2) {
        /* Restore a4-a7 from CPU-side buffer */
        if (fill_depth >= 0 && fill_depth < SPILL_STACK_DEPTH) {
            for (int i = 0; i < 4; i++)
                phys_write(cpu, ret_wb, 4 + i, cpu->spill_stack[si].extra[fill_depth][i]);
        } else {
            for (int i = 0; i < 4; i++)
                phys_write(cpu, ret_wb, 4 + i, mem_read32(cpu->mem, base - 32 + i * 4));
        }
    }
    if (callsize == 3) {
        /* Restore a8-a11 from CPU-side buffer */
        int d = fill_depth;
        if (d >= 0 && d < SPILL_STACK_DEPTH) {
            for (int i = 0; i < 4; i++)
                phys_write(cpu, ret_wb, 8 + i, cpu->spill_stack[si].extra[d][4 + i]);
        } else {
            for (int i = 0; i < 4; i++)
                phys_write(cpu, ret_wb, 8 + i, mem_read32(cpu->mem, base - 48 + i * 4));
        }
    }

    /* Verify restored values match what was originally spilled */
    if (cpu->spill_verify) {
        int si = ret_wb & 0xF;
        int nregs = cpu->spill_shadow[si].count;
        if (nregs > 0) {
            for (int i = 0; i < nregs; i++) {
                uint32_t restored = phys_read(cpu, ret_wb, i);
                uint32_t expected = cpu->spill_shadow[si].regs[i];
                if (restored != expected) {
                    uint32_t spill_base = cpu->spill_shadow[si].base;
                    int slot = (i < 4) ? 0 : (i < 8) ? 1 : 2;
                    uint32_t mem_addr = spill_base - 16 * (slot + 1) + (i % 4) * 4;
                    fprintf(stderr, "[SPILL_CORRUPT] w%d a%d: spilled=0x%08X"
                            " restored=0x%08X at mem=0x%08X"
                            " (orig_base=0x%08X fill_base=0x%08X owb=%d)"
                            " PC=0x%08X cycle=%llu\n",
                            ret_wb, i, expected, restored, mem_addr,
                            spill_base, base, owb, cpu->pc,
                            (unsigned long long)cpu->cycle_count);
                }
            }
        }
    }

    if (cpu->window_trace && cpu->window_trace_active) {
        fprintf(stderr, "     [WIN] FILL  w%d (call%d) base=0x%08X [spill_base={",
                ret_wb, callsize * 4, base);
        for (int _i = 0; _i < 16; _i++)
            fprintf(stderr, "%s0x%X", _i ? "," : "", cpu->spill_base[_i]);
        fprintf(stderr, "}] a0=0x%08X a1=0x%08X a2=0x%08X a3=0x%08X",
                phys_read(cpu, ret_wb, 0), phys_read(cpu, ret_wb, 1),
                phys_read(cpu, ret_wb, 2), phys_read(cpu, ret_wb, 3));
        if (callsize >= 2)
            fprintf(stderr, " a4=0x%08X a5=0x%08X a6=0x%08X a7=0x%08X",
                    phys_read(cpu, ret_wb, 4), phys_read(cpu, ret_wb, 5),
                    phys_read(cpu, ret_wb, 6), phys_read(cpu, ret_wb, 7));
        fprintf(stderr, "\n");
    }

    /* Clear shadow after fill */
    if (cpu->spill_verify)
        cpu->spill_shadow[ret_wb & 0xF].count = 0;

    /* Set windowstart bit */
    cpu->windowstart |= (1u << (ret_wb & 0xF));
}

/*
 * RETW: shared helper for both RETW (24-bit) and RETW.N (16-bit).
 * ISA: n = AR[0][31:30], nextPC = PC[31:30] | AR[0][29:0]
 *   if WS[WB-n] set → normal: clear WS[owb], WB -= n
 *   if WS[WB-n] clear → underflow fill, then WB -= n
 */
static void exec_retw(xtensa_cpu_t *cpu) {
    uint32_t a0 = ar_read(cpu, 0);
    int n = (a0 >> 30) & 3;
    if (n == 0) n = 4;  /* n=0 encoding not used; safety fallback */

    uint32_t next_pc = (cpu->pc & 0xC0000000) | (a0 & 0x3FFFFFFF);

    int owb = cpu->windowbase;
    int ret_wb = (owb - n) & 0xF;

    bool need_fill = !(cpu->windowstart & (1u << ret_wb));
    if (cpu->window_trace && need_fill) {
        fprintf(stderr, "     [WIN] RETW wb=%d->%d (n=%d) UNDERFLOW WS=0x%04X\n",
                owb, ret_wb, n, cpu->windowstart);
    }

    if (need_fill) {
        /* Caller's window was spilled — fill it back */
        synth_underflow_fill(cpu, ret_wb, owb);
    }

    /* Clear current window's WS bit */
    cpu->windowstart &= ~(1u << owb);

    /* Rotate back */
    cpu->windowbase = ret_wb;

    BRANCH_TO(cpu, next_pc);
}

/* ===== Floating-Point Helpers ===== */

static inline uint32_t float_to_bits(float f) {
    uint32_t b; memcpy(&b, &f, 4); return b;
}
static inline float bits_to_float(uint32_t b) {
    float f; memcpy(&f, &b, 4); return f;
}

/* CONST.S lookup table (ISA Table 7-3, reciprocal estimation constants) */
static const uint32_t fp_const_table[16] = {
    0x00000000, /* 0: +0.0 */
    0x3F800000, /* 1: 1.0 */
    0x40000000, /* 2: 2.0 */
    0x3F000000, /* 3: 0.5 */
    0x00000000, /* 4: +0.0 (reserved) */
    0x00000000, /* 5: +0.0 (reserved) */
    0x00000000, /* 6: +0.0 (reserved) */
    0x00000000, /* 7: +0.0 (reserved) */
    0x00000000, /* 8: +0.0 (reserved) */
    0x00000000, /* 9: +0.0 (reserved) */
    0x00000000, /* 10: +0.0 (reserved) */
    0x00000000, /* 11: +0.0 (reserved) */
    0x00000000, /* 12: +0.0 (reserved) */
    0x00000000, /* 13: +0.0 (reserved) */
    0x00000000, /* 14: +0.0 (reserved) */
    0x00000000, /* 15: +0.0 (reserved) */
};

/* Execute FP0 (op0=0, op1=10): arithmetic, conversions, FP1OP */
static void exec_fp0(xtensa_cpu_t *cpu, uint32_t insn) {
    int op2 = XT_OP2(insn);
    int r = XT_R(insn);
    int s = XT_S(insn);
    int t = XT_T(insn);

    switch (op2) {
    case 0: /* ADD.S */
        cpu->fr[r] = cpu->fr[s] + cpu->fr[t];
        break;
    case 1: /* SUB.S */
        cpu->fr[r] = cpu->fr[s] - cpu->fr[t];
        break;
    case 2: /* MUL.S */
        cpu->fr[r] = cpu->fr[s] * cpu->fr[t];
        break;
    case 4: /* MADD.S */
        cpu->fr[r] = cpu->fr[r] + (cpu->fr[s] * cpu->fr[t]);
        break;
    case 5: /* MSUB.S */
        cpu->fr[r] = cpu->fr[r] - (cpu->fr[s] * cpu->fr[t]);
        break;
    case 6: /* MADDN.S (same as MSUB.S, forces round-to-nearest) */
        cpu->fr[r] = cpu->fr[r] - (cpu->fr[s] * cpu->fr[t]);
        break;
    case 7: { /* DIVN.S: fr[r] = fr[s] * fr[t] */
        cpu->fr[r] = cpu->fr[s] * cpu->fr[t];
    } break;
    case 8: { /* ROUND.S: ar[t] = (int32_t)roundf(fr[s] * 2^r) */
        float val = cpu->fr[s];
        if (r) val = val * (float)(1u << r);
        ar_write(cpu, t, (uint32_t)(int32_t)roundf(val));
    } break;
    case 9: { /* TRUNC.S: ar[t] = (int32_t)truncf(fr[s] * 2^r) */
        float val = cpu->fr[s];
        if (r) val = val * (float)(1u << r);
        ar_write(cpu, t, (uint32_t)(int32_t)truncf(val));
    } break;
    case 10: { /* FLOOR.S: ar[t] = (int32_t)floorf(fr[s] * 2^r) */
        float val = cpu->fr[s];
        if (r) val = val * (float)(1u << r);
        ar_write(cpu, t, (uint32_t)(int32_t)floorf(val));
    } break;
    case 11: { /* CEIL.S: ar[t] = (int32_t)ceilf(fr[s] * 2^r) */
        float val = cpu->fr[s];
        if (r) val = val * (float)(1u << r);
        ar_write(cpu, t, (uint32_t)(int32_t)ceilf(val));
    } break;
    case 12: { /* FLOAT.S: fr[r] = (float)(int32_t)ar[s] * 2^(-t) */
        float val = (float)(int32_t)ar_read(cpu, s);
        if (t) val = val / (float)(1u << t);
        cpu->fr[r] = val;
    } break;
    case 13: { /* UFLOAT.S: fr[r] = (float)(uint32_t)ar[s] * 2^(-t) */
        float val = (float)ar_read(cpu, s);
        if (t) val = val / (float)(1u << t);
        cpu->fr[r] = val;
    } break;
    case 14: { /* UTRUNC.S: ar[t] = (uint32_t)(fr[s] * 2^r) */
        float val = cpu->fr[s];
        if (r) val = val * (float)(1u << r);
        ar_write(cpu, t, (uint32_t)val);
    } break;
    case 15: /* FP1OP: sub-dispatch on t */
        switch (t) {
        case 0: /* MOV.S */
            cpu->fr[r] = cpu->fr[s];
            break;
        case 1: /* ABS.S */
            cpu->fr[r] = fabsf(cpu->fr[s]);
            break;
        case 3: /* CONST.S */
            cpu->fr[r] = bits_to_float(fp_const_table[s]);
            break;
        case 4: { /* RFR: ar[r] = fr[s] as bits */
            ar_write(cpu, r, float_to_bits(cpu->fr[s]));
        } break;
        case 5: /* WFR: fr[r] = ar[s] as bits */
            cpu->fr[r] = bits_to_float(ar_read(cpu, s));
            break;
        case 6: /* NEG.S */
            cpu->fr[r] = -cpu->fr[s];
            break;
        case 7: { /* DIV0.S: initial reciprocal approx */
            /* Produce approximate 1/fr[s] using host division */
            float fs = cpu->fr[s];
            if (fs == 0.0f)
                cpu->fr[r] = bits_to_float(0x7F800000); /* +inf */
            else
                cpu->fr[r] = 1.0f / fs;
        } break;
        case 8: { /* RECIP0.S: reciprocal initial approximation */
            float fs = cpu->fr[s];
            if (fs == 0.0f)
                cpu->fr[r] = bits_to_float(0x7F800000);
            else
                cpu->fr[r] = 1.0f / fs;
        } break;
        case 9: { /* SQRT0.S: square root initial approximation */
            float fs = cpu->fr[s];
            cpu->fr[r] = sqrtf(fs);
        } break;
        case 10: { /* RSQRT0.S: reciprocal square root initial */
            float fs = cpu->fr[s];
            if (fs <= 0.0f)
                cpu->fr[r] = bits_to_float(0x7F800000);
            else
                cpu->fr[r] = 1.0f / sqrtf(fs);
        } break;
        case 11: { /* NEXP01.S: force exponent to 127 (range [1.0, 2.0)) */
            uint32_t bits = float_to_bits(cpu->fr[s]);
            bits = (bits & 0x807FFFFFu) | (127u << 23);
            cpu->fr[r] = bits_to_float(bits);
        } break;
        case 12: { /* MKSADJ.S: make sqrt exponent adjustment */
            uint32_t bits = float_to_bits(cpu->fr[s]);
            int exp = (int)((bits >> 23) & 0xFF);
            int adj = 253 - exp; /* for sqrt: (253 - exp) */
            if ((exp & 1) == 0) adj--; /* even exponent */
            uint32_t result = ((uint32_t)(adj & 0xFF) << 23);
            if (bits & 0x80000000u) result |= 0x80000000u;
            cpu->fr[r] = bits_to_float(result);
        } break;
        case 13: { /* MKDADJ.S: make div exponent adjustment */
            uint32_t bits = float_to_bits(cpu->fr[s]);
            int exp = (int)((bits >> 23) & 0xFF);
            int adj = 253 - exp;
            uint32_t result = ((uint32_t)(adj & 0xFF) << 23);
            if (bits & 0x80000000u) result |= 0x80000000u;
            cpu->fr[r] = bits_to_float(result);
        } break;
        case 14: { /* ADDEXP.S: add exponent of fr[s] to fr[r], XOR signs */
            uint32_t rbits = float_to_bits(cpu->fr[r]);
            uint32_t sbits = float_to_bits(cpu->fr[s]);
            int rexp = (int)((rbits >> 23) & 0xFF);
            int sexp = (int)((sbits >> 23) & 0xFF);
            int newexp = rexp + sexp - 127;
            if (newexp < 0) newexp = 0;
            if (newexp > 255) newexp = 255;
            rbits = (rbits & 0x807FFFFFu) | ((uint32_t)(newexp & 0xFF) << 23);
            rbits ^= (sbits & 0x80000000u);
            cpu->fr[r] = bits_to_float(rbits);
        } break;
        case 15: { /* ADDEXPM.S: add exponent from mantissa bits of fr[s] */
            uint32_t rbits = float_to_bits(cpu->fr[r]);
            uint32_t sbits = float_to_bits(cpu->fr[s]);
            int rexp = (int)((rbits >> 23) & 0xFF);
            int mexp = (int)((sbits >> 14) & 0xFF);
            int newexp = rexp + mexp - 127;
            if (newexp < 0) newexp = 0;
            if (newexp > 255) newexp = 255;
            rbits = (rbits & 0x807FFFFFu) | ((uint32_t)(newexp & 0xFF) << 23);
            rbits ^= ((sbits & (1u << 22)) << 9); /* XOR sign with mantissa bit 22 */
            cpu->fr[r] = bits_to_float(rbits);
        } break;
        default: break;
        }
        break;
    default: break;
    }
}

/* Execute FP1 (op0=0, op1=11): comparisons, conditional FP moves */
static void exec_fp1(xtensa_cpu_t *cpu, uint32_t insn) {
    int op2 = XT_OP2(insn);
    int r = XT_R(insn);
    int s = XT_S(insn);
    int t = XT_T(insn);

    switch (op2) {
    case 1: { /* UN.S */
        int result = isnan(cpu->fr[s]) || isnan(cpu->fr[t]);
        cpu->br = (cpu->br & ~(1u << r)) | ((uint32_t)(result != 0) << r);
    } break;
    case 2: { /* OEQ.S */
        int result = !isnan(cpu->fr[s]) && !isnan(cpu->fr[t]) && cpu->fr[s] == cpu->fr[t];
        cpu->br = (cpu->br & ~(1u << r)) | ((uint32_t)(result != 0) << r);
    } break;
    case 3: { /* UEQ.S */
        int result = isnan(cpu->fr[s]) || isnan(cpu->fr[t]) || cpu->fr[s] == cpu->fr[t];
        cpu->br = (cpu->br & ~(1u << r)) | ((uint32_t)(result != 0) << r);
    } break;
    case 4: { /* OLT.S */
        int result = !isnan(cpu->fr[s]) && !isnan(cpu->fr[t]) && cpu->fr[s] < cpu->fr[t];
        cpu->br = (cpu->br & ~(1u << r)) | ((uint32_t)(result != 0) << r);
    } break;
    case 5: { /* ULT.S */
        int result = isnan(cpu->fr[s]) || isnan(cpu->fr[t]) || cpu->fr[s] < cpu->fr[t];
        cpu->br = (cpu->br & ~(1u << r)) | ((uint32_t)(result != 0) << r);
    } break;
    case 6: { /* OLE.S */
        int result = !isnan(cpu->fr[s]) && !isnan(cpu->fr[t]) && cpu->fr[s] <= cpu->fr[t];
        cpu->br = (cpu->br & ~(1u << r)) | ((uint32_t)(result != 0) << r);
    } break;
    case 7: { /* ULE.S */
        int result = isnan(cpu->fr[s]) || isnan(cpu->fr[t]) || cpu->fr[s] <= cpu->fr[t];
        cpu->br = (cpu->br & ~(1u << r)) | ((uint32_t)(result != 0) << r);
    } break;
    case 8: /* MOVEQZ.S */
        if (ar_read(cpu, t) == 0) cpu->fr[r] = cpu->fr[s];
        break;
    case 9: /* MOVNEZ.S */
        if (ar_read(cpu, t) != 0) cpu->fr[r] = cpu->fr[s];
        break;
    case 10: /* MOVLTZ.S */
        if ((int32_t)ar_read(cpu, t) < 0) cpu->fr[r] = cpu->fr[s];
        break;
    case 11: /* MOVGEZ.S */
        if ((int32_t)ar_read(cpu, t) >= 0) cpu->fr[r] = cpu->fr[s];
        break;
    case 12: /* MOVF.S */
        if (!((cpu->br >> t) & 1)) cpu->fr[r] = cpu->fr[s];
        break;
    case 13: /* MOVT.S */
        if ((cpu->br >> t) & 1) cpu->fr[r] = cpu->fr[s];
        break;
    default: break;
    }
}

/* Execute op0=0 (QRST) - the main RRR instruction group */
static void exec_qrst(xtensa_cpu_t *cpu, uint32_t insn) {
    int op1 = XT_OP1(insn);
    int op2 = XT_OP2(insn);
    int r = XT_R(insn);
    int s = XT_S(insn);
    int t = XT_T(insn);

    switch (op1) {
    case 0: /* RST0 */
        switch (op2) {
        case 0: /* ST0: specials */
            switch (r) {
            case 0: /* SNM0 */
                if (s == 0 && t == 0) {
                    /* ILL */
                    xtensa_raise_exception(cpu, EXCCAUSE_ILLEGAL, cpu->pc - 3, 0);
                    return;
                } else {
                    int m = XT_M(insn);
                    int nn = XT_N(insn);
                    if (m == 2 && nn == 0) {
                        /* RET: pc = a0 */
                        BRANCH_TO(cpu, ar_read(cpu, 0));
                        return; /* skip default pc advance */
                    } else if (m == 2 && nn == 1) {
                        /* RETW: windowed return */
                        exec_retw(cpu);
                        return;
                    } else if (m == 2 && nn == 2) {
                        /* JX: pc = ar[s] */
                        BRANCH_TO(cpu, ar_read(cpu, s));
                        return;
                    } else if (m == 3) {
                        /* CALLX0/4/8/12 */
                        uint32_t target = ar_read(cpu, s);
                        if (nn > 0) {
                            XT_PS_SET_CALLINC(cpu->ps, nn);
                            ar_write(cpu, nn * 4, ((uint32_t)nn << 30) | (cpu->pc & 0x3FFFFFFF));
                        } else {
                            ar_write(cpu, 0, cpu->pc);
                        }
                        BRANCH_TO(cpu, target);
                        return;
                    }
                    /* BREAK, etc. */
                    if (m == 0 && nn != 0) {
                        /* Some other SNM0 encoding */
                    }
                }
                break;
            case 1: { /* MOVSP */
                /* Spill any live windows below current */
                for (int i = 1; i <= 3; i++) {
                    int w = (cpu->windowbase - i) & 15;
                    if (cpu->windowstart & (1u << w))
                        synth_spill_window(cpu, w);
                }
                /* Copy base save area from old SP to new SP, and update
                 * spill_base for any windows that were saved at old_sp. */
                uint32_t old_sp = ar_read(cpu, 1);
                uint32_t new_sp = ar_read(cpu, s);
                if (old_sp != new_sp) {
                    for (int i = 0; i < 12; i++) {
                        uint32_t val = mem_read32(cpu->mem, old_sp - 48 + i * 4);
                        mem_write32(cpu->mem, new_sp - 48 + i * 4, val);
                    }
                    /* Update spill_base and spill_stack for any window whose data was at old_sp */
                    for (int i = 0; i < 16; i++) {
                        if (cpu->spill_base[i] == old_sp) {
                            if (cpu->window_trace && cpu->window_trace_active)
                                fprintf(stderr, "     [WIN] MOVSP spill_base[%d] 0x%08X -> 0x%08X (wb=%d)\n",
                                        i, old_sp, new_sp, cpu->windowbase);
                            cpu->spill_base[i] = new_sp;
                        }
                        for (int j = 0; j < cpu->spill_stack[i].depth && j < 8; j++) {
                            if (cpu->spill_stack[i].base[j] == old_sp)
                                cpu->spill_stack[i].base[j] = new_sp;
                        }
                    }
                }
                ar_write(cpu, t, new_sp);
            } break;
            case 2: /* SYNC group */
                /* NOP, ISYNC, RSYNC, ESYNC, DSYNC, EXTW, MEMW, EXCW */
                /* All no-ops for emulation purposes */
                break;
            case 3: /* RFEI group */
                switch (t) {
                case 0: /* RFET: RFE, RFWO, RFWU */
                    switch (s) {
                    case 0: /* RFE */
                        XT_PS_SET_EXCM(cpu->ps, 0);
                        BRANCH_TO(cpu, cpu->epc[0]);
                        return;
                    case 4: /* RFWO */
                        XT_PS_SET_EXCM(cpu->ps, 0);
                        cpu->windowstart &= ~(1u << cpu->windowbase);
                        cpu->windowbase = XT_PS_OWB(cpu->ps);
                        BRANCH_TO(cpu, cpu->epc[0]);
                        return;
                    case 5: /* RFWU */
                        XT_PS_SET_EXCM(cpu->ps, 0);
                        cpu->windowstart |= (1u << cpu->windowbase);
                        cpu->windowbase = XT_PS_OWB(cpu->ps);
                        BRANCH_TO(cpu, cpu->epc[0]);
                        return;
                    default: break;
                    }
                    break;
                case 1: /* RFI */
                    if (s >= 1 && s <= 7) {
                        cpu->ps = cpu->eps[s - 1];
                        BRANCH_TO(cpu, cpu->epc[s - 1]);
                    }
                    return;
                default: break;
                }
                break;
            case 4: /* BREAK */
                cpu->debug_break = true;
                break;
            case 5: /* SYSCALL */
                if (XT_PS_WOE(cpu->ps)) {
                    /* Synthesized window spill-all.  The firmware's SYSCALL
                     * handler uses ROTW to walk all windows and trigger
                     * overflow exceptions.  Our emulator doesn't raise
                     * overflows on ROTW, so intercept here and do the
                     * spill in C. */
                    for (unsigned w = 0; w < 16; w++) {
                        if (w != cpu->windowbase &&
                            (cpu->windowstart & (1u << w)))
                            synth_spill_window(cpu, (int)w);
                    }
                } else {
                    xtensa_raise_exception(cpu, EXCCAUSE_SYSCALL,
                                           cpu->pc - 3, 0);
                    return;
                }
                break;
            case 6: /* RSIL - read/set interrupt level */
                ar_write(cpu, t, cpu->ps);
                cpu->ps = (cpu->ps & ~0xF) | (s & 0xF);
                break;
            case 7: /* WAITI */
                XT_PS_SET_INTLEVEL(cpu->ps, s);
                cpu->halted = true;
                break;
            case 8: /* ANY4: bt = bs|bs+1|bs+2|bs+3 */
                { int val = (cpu->br >> s) & 0xF;
                  cpu->br = (cpu->br & ~(1u << t)) | ((val ? 1u : 0u) << t);
                } break;
            case 9: /* ALL4: bt = bs&bs+1&bs+2&bs+3 */
                { int val = (cpu->br >> s) & 0xF;
                  cpu->br = (cpu->br & ~(1u << t)) | (((val == 0xF) ? 1u : 0u) << t);
                } break;
            case 10: /* ANY8: bt = any of bs..bs+7 */
                { int val = (cpu->br >> s) & 0xFF;
                  cpu->br = (cpu->br & ~(1u << t)) | ((val ? 1u : 0u) << t);
                } break;
            case 11: /* ALL8: bt = all of bs..bs+7 */
                { int val = (cpu->br >> s) & 0xFF;
                  cpu->br = (cpu->br & ~(1u << t)) | (((val == 0xFF) ? 1u : 0u) << t);
                } break;
            default: break;
            }
            break;
        case 1: /* AND */
            ar_write(cpu, r, ar_read(cpu, s) & ar_read(cpu, t));
            break;
        case 2: /* OR */
            ar_write(cpu, r, ar_read(cpu, s) | ar_read(cpu, t));
            break;
        case 3: /* XOR */
            ar_write(cpu, r, ar_read(cpu, s) ^ ar_read(cpu, t));
            break;
        case 4: /* ST1: shift-amount setup */
            switch (r) {
            case 0: /* SSR: SAR = ar[s] & 31 */
                cpu->sar = ar_read(cpu, s) & 0x1F;
                break;
            case 1: /* SSL: SAR = 32 - (ar[s] & 31) */
                cpu->sar = 32 - (ar_read(cpu, s) & 0x1F);
                break;
            case 2: /* SSA8L: SAR = (ar[s] & 3) * 8 */
                cpu->sar = (ar_read(cpu, s) & 3) * 8;
                break;
            case 3: /* SSA8B: SAR = 32 - (ar[s] & 3) * 8 */
                cpu->sar = 32 - (ar_read(cpu, s) & 3) * 8;
                break;
            case 4: /* SSAI: SAR = immediate (s | (t<<4))&31 ... actually just the 5-bit field */
                /* SSAI: SAR = (s | ((t & 1) << 4)) */
                cpu->sar = (s | ((t & 1) << 4));
                break;
            case 8: /* ROTW - rotate window */
                /* Simplified: just adjust windowbase */
                cpu->windowbase = (cpu->windowbase + (int32_t)sign_extend(t, 4)) & 0xF;
                break;
            case 14: /* NSA: normalized shift amount */
                { uint32_t val = ar_read(cpu, s);
                  int n = 0;
                  if ((int32_t)val < 0) val = ~val;
                  if (val == 0) { n = 31; }
                  else { while (!(val & 0x80000000)) { val <<= 1; n++; } }
                  ar_write(cpu, t, (uint32_t)n);
                } break;
            case 15: /* NSAU: normalized shift amount unsigned */
                { uint32_t val = ar_read(cpu, s);
                  int n = 0;
                  if (val == 0) { n = 32; }
                  else { while (!(val & 0x80000000)) { val <<= 1; n++; } }
                  ar_write(cpu, t, (uint32_t)n);
                } break;
            case 6: /* RER: ar[t] = external_reg[ar[s]] */
                ar_write(cpu, t, 0);  /* stub: return 0 */
                break;
            case 7: /* WER: external_reg[ar[s]] = ar[t] */
                break; /* stub: ignore */
            default: break;
            }
            break;
        case 5: /* TLB ops - stub */
            break;
        case 6: /* RT0 */
            switch (s) {
            case 0: /* NEG */
                ar_write(cpu, r, (uint32_t)(-(int32_t)ar_read(cpu, t)));
                break;
            case 1: /* ABS */
                { int32_t val = (int32_t)ar_read(cpu, t);
                  ar_write(cpu, r, (uint32_t)(val < 0 ? -val : val));
                } break;
            default: break;
            }
            break;
        case 7: /* reserved */
            break;
        case 8:  /* ADD */
            ar_write(cpu, r, ar_read(cpu, s) + ar_read(cpu, t));
            break;
        case 9:  /* ADDX2 */
            ar_write(cpu, r, (ar_read(cpu, s) << 1) + ar_read(cpu, t));
            break;
        case 10: /* ADDX4 */
            ar_write(cpu, r, (ar_read(cpu, s) << 2) + ar_read(cpu, t));
            break;
        case 11: /* ADDX8 */
            ar_write(cpu, r, (ar_read(cpu, s) << 3) + ar_read(cpu, t));
            break;
        case 12: /* SUB */
            ar_write(cpu, r, ar_read(cpu, s) - ar_read(cpu, t));
            break;
        case 13: /* SUBX2 */
            ar_write(cpu, r, (ar_read(cpu, s) << 1) - ar_read(cpu, t));
            break;
        case 14: /* SUBX4 */
            ar_write(cpu, r, (ar_read(cpu, s) << 2) - ar_read(cpu, t));
            break;
        case 15: /* SUBX8 */
            ar_write(cpu, r, (ar_read(cpu, s) << 3) - ar_read(cpu, t));
            break;
        }
        break;

    case 1: /* RST1 */
        switch (op2) {
        case 0: case 1: /* SLLI */
            { int sa = 32 - (((op2 & 1) << 4) | t);
              ar_write(cpu, r, (sa >= 32) ? 0 : ar_read(cpu, s) << sa);
            } break;
        case 2: case 3: /* SRAI */
            { int sa = ((op2 & 1) << 4) | s;
              ar_write(cpu, r, (uint32_t)((int32_t)ar_read(cpu, t) >> sa));
            } break;
        case 4: /* SRLI */
            ar_write(cpu, r, ar_read(cpu, t) >> s);
            break;
        case 6: /* XSR */
            { int sr_num = XT_SR_NUM(insn);
              uint32_t tmp = ar_read(cpu, t);
              ar_write(cpu, t, sr_read(cpu, sr_num));
              sr_write(cpu, sr_num, tmp);
            } break;
        case 8: /* SRC - funnel shift: (AR[s]:AR[t]) >> SAR, SAR 0-32 */
            { uint32_t sa = cpu->sar & 0x3F;
              uint64_t concat = ((uint64_t)ar_read(cpu, s) << 32) | (uint64_t)ar_read(cpu, t);
              ar_write(cpu, r, (uint32_t)(concat >> sa));
            } break;
        case 9: /* SRL - funnel shift: (0:AR[t]) >> SAR */
            { uint32_t sa = cpu->sar & 0x3F;
              ar_write(cpu, r, sa >= 32 ? 0 : ar_read(cpu, t) >> sa);
            } break;
        case 10: /* SLL - funnel shift: (AR[s]:0) >> SAR */
            { uint32_t sa = cpu->sar & 0x3F;
              uint64_t concat = (uint64_t)ar_read(cpu, s) << 32;
              ar_write(cpu, r, (uint32_t)(concat >> sa));
            } break;
        case 11: /* SRA - arithmetic right shift by SAR */
            { uint32_t sa = cpu->sar & 0x3F;
              int32_t val = (int32_t)ar_read(cpu, t);
              ar_write(cpu, r, (uint32_t)(sa >= 32 ? (val >> 31) : (val >> sa)));
            } break;
        case 12: /* MUL16U */
            ar_write(cpu, r, (ar_read(cpu, s) & 0xFFFF) * (ar_read(cpu, t) & 0xFFFF));
            break;
        case 13: /* MUL16S */
            { int32_t a = (int32_t)(int16_t)(ar_read(cpu, s) & 0xFFFF);
              int32_t b = (int32_t)(int16_t)(ar_read(cpu, t) & 0xFFFF);
              ar_write(cpu, r, (uint32_t)(a * b));
            } break;
        default: break;
        }
        break;

    case 2: /* RST2 */
        switch (op2) {
        case 0: /* ANDB: br[r] = br[s] AND br[t] */
            { int val = ((cpu->br >> s) & 1) & ((cpu->br >> t) & 1);
              cpu->br = (cpu->br & ~(1u << r)) | ((uint32_t)val << r);
            } break;
        case 1: /* ANDBC: br[r] = br[s] AND NOT br[t] */
            { int val = ((cpu->br >> s) & 1) & (~(cpu->br >> t) & 1);
              cpu->br = (cpu->br & ~(1u << r)) | ((uint32_t)val << r);
            } break;
        case 2: /* ORB: br[r] = br[s] OR br[t] */
            { int val = ((cpu->br >> s) & 1) | ((cpu->br >> t) & 1);
              cpu->br = (cpu->br & ~(1u << r)) | ((uint32_t)val << r);
            } break;
        case 3: /* ORBC: br[r] = br[s] OR NOT br[t] */
            { int val = ((cpu->br >> s) & 1) | (~(cpu->br >> t) & 1);
              cpu->br = (cpu->br & ~(1u << r)) | ((uint32_t)val << r);
            } break;
        case 4: /* XORB: br[r] = br[s] XOR br[t] */
            { int val = ((cpu->br >> s) & 1) ^ ((cpu->br >> t) & 1);
              cpu->br = (cpu->br & ~(1u << r)) | ((uint32_t)val << r);
            } break;
        case 6: /* SALT */
            ar_write(cpu, r, (int32_t)ar_read(cpu, s) < (int32_t)ar_read(cpu, t) ? 1 : 0);
            break;
        case 7: /* SALTU */
            ar_write(cpu, r, ar_read(cpu, s) < ar_read(cpu, t) ? 1 : 0);
            break;
        case 8: /* MULL */
            ar_write(cpu, r, ar_read(cpu, s) * ar_read(cpu, t));
            break;
        case 10: /* MULUH */
            { uint64_t res = (uint64_t)ar_read(cpu, s) * (uint64_t)ar_read(cpu, t);
              ar_write(cpu, r, (uint32_t)(res >> 32));
            } break;
        case 11: /* MULSH */
            { int64_t res = (int64_t)(int32_t)ar_read(cpu, s) * (int64_t)(int32_t)ar_read(cpu, t);
              ar_write(cpu, r, (uint32_t)((uint64_t)res >> 32));
            } break;
        case 12: /* QUOU */
            { uint32_t divisor = ar_read(cpu, t);
              if (divisor == 0) { xtensa_raise_exception(cpu, EXCCAUSE_DIVIDE_BY_ZERO, cpu->pc - 3, 0); return; }
              ar_write(cpu, r, ar_read(cpu, s) / divisor);
            } break;
        case 13: /* QUOS */
            { int32_t divisor = (int32_t)ar_read(cpu, t);
              if (divisor == 0) { xtensa_raise_exception(cpu, EXCCAUSE_DIVIDE_BY_ZERO, cpu->pc - 3, 0); return; }
              int32_t dividend = (int32_t)ar_read(cpu, s);
              /* Handle INT_MIN / -1 overflow */
              if (dividend == (int32_t)0x80000000 && divisor == -1)
                  ar_write(cpu, r, 0x80000000);
              else
                  ar_write(cpu, r, (uint32_t)(dividend / divisor));
            } break;
        case 14: /* REMU */
            { uint32_t divisor = ar_read(cpu, t);
              if (divisor == 0) { xtensa_raise_exception(cpu, EXCCAUSE_DIVIDE_BY_ZERO, cpu->pc - 3, 0); return; }
              ar_write(cpu, r, ar_read(cpu, s) % divisor);
            } break;
        case 15: /* REMS */
            { int32_t divisor = (int32_t)ar_read(cpu, t);
              if (divisor == 0) { xtensa_raise_exception(cpu, EXCCAUSE_DIVIDE_BY_ZERO, cpu->pc - 3, 0); return; }
              int32_t dividend = (int32_t)ar_read(cpu, s);
              if (dividend == (int32_t)0x80000000 && divisor == -1)
                  ar_write(cpu, r, 0);
              else
                  ar_write(cpu, r, (uint32_t)(dividend % divisor));
            } break;
        default: break;
        }
        break;

    case 3: /* RST3 */
        switch (op2) {
        case 0: /* RSR */
            ar_write(cpu, t, sr_read(cpu, XT_SR_NUM(insn)));
            break;
        case 1: /* WSR */
            sr_write(cpu, XT_SR_NUM(insn), ar_read(cpu, t));
            break;
        case 2: /* SEXT - sign extend from bit position (t+7) */
            { int bits = t + 8; /* 8..23 */
              int32_t val = sign_extend(ar_read(cpu, s), bits);
              ar_write(cpu, r, (uint32_t)val);
            } break;
        case 3: /* CLAMPS - clamp to signed range -(2^(t+7)) .. (2^(t+7)-1) */
            { int bits = t + 7; /* 7..22 */
              int32_t val = (int32_t)ar_read(cpu, s);
              int32_t hi = (1 << bits) - 1;
              int32_t lo = -(1 << bits);
              if (val > hi) val = hi;
              else if (val < lo) val = lo;
              ar_write(cpu, r, (uint32_t)val);
            } break;
        case 4: /* MIN */
            { int32_t a = (int32_t)ar_read(cpu, s);
              int32_t b = (int32_t)ar_read(cpu, t);
              ar_write(cpu, r, (uint32_t)(a < b ? a : b));
            } break;
        case 5: /* MAX */
            { int32_t a = (int32_t)ar_read(cpu, s);
              int32_t b = (int32_t)ar_read(cpu, t);
              ar_write(cpu, r, (uint32_t)(a > b ? a : b));
            } break;
        case 6: /* MINU */
            { uint32_t a = ar_read(cpu, s);
              uint32_t b = ar_read(cpu, t);
              ar_write(cpu, r, a < b ? a : b);
            } break;
        case 7: /* MAXU */
            { uint32_t a = ar_read(cpu, s);
              uint32_t b = ar_read(cpu, t);
              ar_write(cpu, r, a > b ? a : b);
            } break;
        case 8: /* MOVEQZ */
            if (ar_read(cpu, t) == 0)
                ar_write(cpu, r, ar_read(cpu, s));
            break;
        case 9: /* MOVNEZ */
            if (ar_read(cpu, t) != 0)
                ar_write(cpu, r, ar_read(cpu, s));
            break;
        case 10: /* MOVLTZ */
            if ((int32_t)ar_read(cpu, t) < 0)
                ar_write(cpu, r, ar_read(cpu, s));
            break;
        case 11: /* MOVGEZ */
            if ((int32_t)ar_read(cpu, t) >= 0)
                ar_write(cpu, r, ar_read(cpu, s));
            break;
        case 12: /* MOVF: if (!bt) ar[r] = ar[s] */
            if (!((cpu->br >> t) & 1))
                ar_write(cpu, r, ar_read(cpu, s));
            break;
        case 13: /* MOVT: if (bt) ar[r] = ar[s] */
            if ((cpu->br >> t) & 1)
                ar_write(cpu, r, ar_read(cpu, s));
            break;
        case 14: /* RUR */
            { int ur = (s << 4) | r;
              switch (ur) {
              case 232: ar_write(cpu, t, cpu->fcr); break;
              case 233: ar_write(cpu, t, cpu->fsr); break;
              default:  ar_write(cpu, t, 0); break;
              }
            } break;
        case 15: /* WUR */
            { int ur = (s << 4) | r;
              switch (ur) {
              case 232: cpu->fcr = ar_read(cpu, t); break;
              case 233: cpu->fsr = ar_read(cpu, t); break;
              default: break;
              }
            } break;
        default: break;
        }
        break;

    case 4: case 5: /* EXTUI */
        { int shift = s | ((op1 & 1) << 4);
          uint32_t mask = (1u << (op2 + 1)) - 1;
          ar_write(cpu, r, (ar_read(cpu, t) >> shift) & mask);
        } break;

    case 8: /* LSCX: indexed FP loads/stores */
        switch (op2) {
        case 0: { /* LSX: fr[r] = mem32[ar[s] + ar[t]] */
            uint32_t addr = ar_read(cpu, s) + ar_read(cpu, t);
            uint32_t tmp = mem_read32(cpu->mem, addr);
            memcpy(&cpu->fr[r], &tmp, 4);
        } break;
        case 1: { /* LSXP: fr[r] = mem32[ar[s]]; ar[s] += ar[t] */
            uint32_t base = ar_read(cpu, s);
            uint32_t tmp = mem_read32(cpu->mem, base);
            memcpy(&cpu->fr[r], &tmp, 4);
            ar_write(cpu, s, base + ar_read(cpu, t));
        } break;
        case 4: { /* SSX: mem32[ar[s] + ar[t]] = fr[r] */
            uint32_t addr = ar_read(cpu, s) + ar_read(cpu, t);
            uint32_t tmp; memcpy(&tmp, &cpu->fr[r], 4);
            mem_write32(cpu->mem, addr, tmp);
        } break;
        case 5: { /* SSXP: mem32[ar[s]] = fr[r]; ar[s] += ar[t] */
            uint32_t base = ar_read(cpu, s);
            uint32_t tmp; memcpy(&tmp, &cpu->fr[r], 4);
            mem_write32(cpu->mem, base, tmp);
            ar_write(cpu, s, base + ar_read(cpu, t));
        } break;
        default: break;
        }
        break;

    case 9: /* LSC4: L32E, S32E */
        switch (op2) {
        case 0: { /* L32E */
            uint32_t addr = ar_read(cpu, s) + (uint32_t)((int32_t)(r << 2) - 64);
            ar_write(cpu, t, mem_read32(cpu->mem, addr));
        } break;
        case 4: { /* S32E */
            uint32_t addr = ar_read(cpu, s) + (uint32_t)((int32_t)(r << 2) - 64);
            mem_write32(cpu->mem, addr, ar_read(cpu, t));
        } break;
        default: break;
        }
        break;

    case 10: /* FP0: FP arithmetic, conversions */
        exec_fp0(cpu, insn);
        break;

    case 11: /* FP1: FP comparisons, conditional moves */
        exec_fp1(cpu, insn);
        break;

    default:
        /* Unimplemented op1 groups */
        break;
    }
}

/* Execute op0=2 (LSAI) - loads, stores, and ALU immediates */
static void exec_lsai(xtensa_cpu_t *cpu, uint32_t insn) {
    int r = XT_R(insn);
    int s = XT_S(insn);
    int t = XT_T(insn);
    int imm8 = XT_IMM8(insn);

    switch (r) {
    case 0x0: /* L8UI */
        { uint32_t addr = ar_read(cpu, s) + (uint32_t)imm8;
          ar_write(cpu, t, mem_read8(cpu->mem, addr));
        } break;
    case 0x1: /* L16UI */
        { uint32_t addr = ar_read(cpu, s) + (uint32_t)(imm8 << 1);
          ar_write(cpu, t, mem_read16(cpu->mem, addr));
        } break;
    case 0x2: /* L32I */
        { uint32_t addr = ar_read(cpu, s) + (uint32_t)(imm8 << 2);
          ar_write(cpu, t, mem_read32(cpu->mem, addr));
        } break;
    case 0x4: /* S8I */
        { uint32_t addr = ar_read(cpu, s) + (uint32_t)imm8;
          mem_write8(cpu->mem, addr, (uint8_t)ar_read(cpu, t));
        } break;
    case 0x5: /* S16I */
        { uint32_t addr = ar_read(cpu, s) + (uint32_t)(imm8 << 1);
          mem_write16(cpu->mem, addr, (uint16_t)ar_read(cpu, t));
        } break;
    case 0x6: /* S32I */
        { uint32_t addr = ar_read(cpu, s) + (uint32_t)(imm8 << 2);
          mem_write32(cpu->mem, addr, ar_read(cpu, t));
        } break;
    case 0x7: /* CACHE ops (DPFR, DPFW, DHWB, etc.) - no-op */
        break;
    case 0x9: /* L16SI */
        { uint32_t addr = ar_read(cpu, s) + (uint32_t)(imm8 << 1);
          ar_write(cpu, t, (uint32_t)sign_extend(mem_read16(cpu->mem, addr), 16));
        } break;
    case 0xB: /* L32AI (acquire semantics = no-op in emulator) */
        { uint32_t addr = ar_read(cpu, s) + (uint32_t)(imm8 << 2);
          ar_write(cpu, t, mem_read32(cpu->mem, addr));
        } break;
    case 0xE: /* S32C1I (conditional store) */
        { uint32_t addr = ar_read(cpu, s) + (uint32_t)(imm8 << 2);
          uint32_t old = mem_read32(cpu->mem, addr);
          if (old == cpu->scompare1)
              mem_write32(cpu->mem, addr, ar_read(cpu, t));
          ar_write(cpu, t, old);
        } break;
    case 0xF: /* S32RI (release semantics = no-op in emulator) */
        { uint32_t addr = ar_read(cpu, s) + (uint32_t)(imm8 << 2);
          mem_write32(cpu->mem, addr, ar_read(cpu, t));
        } break;

    case 0xA: /* MOVI */
        { int32_t imm12 = sign_extend(((uint32_t)s << 8) | (uint32_t)imm8, 12);
          ar_write(cpu, t, (uint32_t)imm12);
        } break;

    case 0xC: /* ADDI */
        { int32_t simm8 = sign_extend(imm8, 8);
          ar_write(cpu, t, ar_read(cpu, s) + (uint32_t)simm8);
        } break;

    case 0xD: /* ADDMI */
        { int32_t simm8 = sign_extend(imm8, 8);
          ar_write(cpu, t, ar_read(cpu, s) + (uint32_t)(simm8 << 8));
        } break;

    default:
        break;
    }
}

/* Execute narrow (16-bit) instructions */
static inline __attribute__((always_inline))
void exec_narrow(xtensa_cpu_t *cpu, uint32_t insn) {
    int op0 = XT_OP0(insn);
    int t = XT_T(insn);
    int s = XT_S(insn);
    int r = XT_R(insn);

    switch (op0) {
    case 0x8: /* L32I.N */
        ar_write(cpu, t, mem_read32(cpu->mem, ar_read(cpu, s) + (uint32_t)(r << 2)));
        break;
    case 0x9: /* S32I.N */
        mem_write32(cpu->mem, ar_read(cpu, s) + (uint32_t)(r << 2), ar_read(cpu, t));
        break;

    case 0xA: /* ADD.N */
        ar_write(cpu, r, ar_read(cpu, s) + ar_read(cpu, t));
        break;

    case 0xB: /* ADDI.N */
        { int imm = (t == 0) ? -1 : t;
          ar_write(cpu, r, ar_read(cpu, s) + (uint32_t)(int32_t)imm);
        } break;

    case 0xC: /* ST2: MOVI.N / BEQZ.N / BNEZ.N */
        { int t_hi = (t >> 2) & 3;
          if (t_hi < 2) {
              /* MOVI.N: range -32..95, NOT standard 7-bit sign extension */
              int imm7 = ((t & 7) << 4) | r;
              int32_t val = (imm7 >= 96) ? (imm7 - 128) : imm7;
              ar_write(cpu, s, (uint32_t)val);
          } else if (t_hi == 2) {
              /* BEQZ.N */
              int imm6 = ((t & 3) << 4) | r;
              if (ar_read(cpu, s) == 0)
                  BRANCH_TO(cpu, cpu->pc + (uint32_t)imm6 + 2);
          } else {
              /* BNEZ.N */
              int imm6 = ((t & 3) << 4) | r;
              if (ar_read(cpu, s) != 0)
                  BRANCH_TO(cpu, cpu->pc + (uint32_t)imm6 + 2);
          }
        } break;

    case 0xD: /* ST3 */
        switch (r) {
        case 0: /* MOV.N */
            ar_write(cpu, t, ar_read(cpu, s));
            break;
        case 15: /* ST3 r=15 subgroup */
            switch (t) {
            case 0: /* RET.N */
                BRANCH_TO(cpu, ar_read(cpu, 0));
                return; /* skip default pc advance */
            case 1: /* RETW.N */
                exec_retw(cpu);
                return;
            case 2: /* BREAK.N */
                cpu->debug_break = true;
                break;
            case 3: /* NOP.N */
                break;
            case 6: /* ILL.N */
                xtensa_raise_exception(cpu, EXCCAUSE_ILLEGAL, cpu->pc - 2, 0);
                return;
            default: break;
            }
            break;
        default: break;
        }
        break;

    default: break;
    }
}

/* B4const / B4constu lookup tables for immediate branches */
static const int32_t b4const[16] = {
    -1, 1, 2, 3, 4, 5, 6, 7, 8, 10, 12, 16, 32, 64, 128, 256
};
static const uint32_t b4constu[16] = {
    32768, 65536, 2, 3, 4, 5, 6, 7, 8, 10, 12, 16, 32, 64, 128, 256
};

/* Execute op0=5 (CALLN) - PC-relative calls */
static void exec_calln(xtensa_cpu_t *cpu, uint32_t insn) {
    int nn = XT_N(insn);
    int32_t offset = sign_extend(XT_OFFSET18(insn), 18);
    /* target[31:2] = (original_pc[31:2] + offset + 1), target[1:0] = 00 */
    uint32_t original_pc = cpu->pc - 3;
    uint32_t target = (((original_pc >> 2) + (uint32_t)offset + 1) << 2);

    if (nn > 0) {
        /* Windowed call: PS.CALLINC = nn, AR[nn*4] = nn || nextPC[29:0] */
        XT_PS_SET_CALLINC(cpu->ps, nn);
        ar_write(cpu, nn * 4, ((uint32_t)nn << 30) | (cpu->pc & 0x3FFFFFFF));
    } else {
        /* CALL0: return address = next instruction */
        ar_write(cpu, 0, cpu->pc);
    }
    BRANCH_TO(cpu, target);
}

/* Execute op0=6 (SI) - J, BRI12, BRI8, LOOP, ENTRY */
static void exec_si(xtensa_cpu_t *cpu, uint32_t insn) {
    int nn = XT_N(insn);
    int m = XT_M(insn);
    int s = XT_S(insn);

    switch (nn) {
    case 0: /* J - unconditional jump */
        { int32_t offset = sign_extend(XT_OFFSET18(insn), 18);
          BRANCH_TO(cpu, cpu->pc + (uint32_t)offset + 1);
        } break;

    case 1: /* BZ - BRI12 zero-compare branches */
        { int32_t imm12 = sign_extend(XT_IMM12(insn), 12);
          uint32_t target = cpu->pc + (uint32_t)imm12 + 1;
          int32_t val = (int32_t)ar_read(cpu, s);
          switch (m) {
          case 0: if (val == 0) BRANCH_TO(cpu, target); break;  /* BEQZ */
          case 1: if (val != 0) BRANCH_TO(cpu, target); break;  /* BNEZ */
          case 2: if (val < 0)  BRANCH_TO(cpu, target); break;  /* BLTZ */
          case 3: if (val >= 0) BRANCH_TO(cpu, target); break;  /* BGEZ */
          }
        } break;

    case 2: /* BI0 - BRI8 immediate-compare branches */
        { int imm8 = XT_IMM8(insn);
          int r = XT_R(insn);
          int32_t offset8 = sign_extend(imm8, 8);
          uint32_t target = cpu->pc + (uint32_t)offset8 + 1;
          int32_t val = (int32_t)ar_read(cpu, s);
          switch (m) {
          case 0: if (val == b4const[r]) BRANCH_TO(cpu, target); break;  /* BEQI */
          case 1: if (val != b4const[r]) BRANCH_TO(cpu, target); break;  /* BNEI */
          case 2: if (val < b4const[r])  BRANCH_TO(cpu, target); break;  /* BLTI */
          case 3: if (val >= b4const[r]) BRANCH_TO(cpu, target); break;  /* BGEI */
          }
        } break;

    case 3: /* BI1 */
        { int imm8 = XT_IMM8(insn);
          int r = XT_R(insn);
          switch (m) {
          case 0: { /* ENTRY */
              int callinc = XT_PS_CALLINC(cpu->ps);
              uint32_t imm12 = XT_IMM12(insn);
              uint32_t frame_size = imm12 << 3;

              /* Set callee's a1 (SP) BEFORE overflow check so that
               * synth_spill_window reads the correct base address.
               * The callee's a1 is in the rotated window at register
               * index (callinc*4 | s&3), which is the same register
               * the ENTRY instruction targets. */
              int new_reg = (callinc << 2) | (s & 3);
              ar_write(cpu, new_reg, ar_read(cpu, s) - frame_size);

              synth_overflow_check(cpu, callinc);

              uint32_t owb = cpu->windowbase;
              cpu->windowbase = (owb + callinc) & 0xF;
              cpu->windowstart |= (1u << cpu->windowbase);
              XT_PS_SET_OWB(cpu->ps, owb);
              XT_PS_SET_CALLINC(cpu->ps, 0);
          } break;
          case 1: /* B1: BF, BT, LOOP, LOOPNEZ, LOOPGTZ */
              switch (r) {
              case 0: /* BF */
              case 1: /* BT */
              {
                  /* Branches use sign-extended offset: target = PC + sext(imm8) + 4 */
                  int32_t offset8 = sign_extend(imm8, 8);
                  uint32_t target = cpu->pc + (uint32_t)offset8 + 1;
                  if (r == 0) {
                      if (!(cpu->br & (1u << s)))
                          BRANCH_TO(cpu, target);
                  } else {
                      if (cpu->br & (1u << s))
                          BRANCH_TO(cpu, target);
                  }
                  break;
              }
              case 8:  /* LOOP */
              case 9:  /* LOOPNEZ */
              case 10: /* LOOPGTZ */
              {
                  /* LOOP uses zero-extended offset: LEND = PC + zext(imm8) + 4
                   * Since cpu->pc is already advanced by 3: LEND = cpu->pc + imm8 + 1 */
                  uint32_t loop_end = cpu->pc + (uint32_t)imm8 + 1;
                  cpu->lend = loop_end;
                  cpu->lbeg = cpu->pc;
                  if (r == 8) {
                      /* LOOP: always enter */
                      cpu->lcount = ar_read(cpu, s) - 1;
                  } else if (r == 9) {
                      /* LOOPNEZ: skip if count == 0 */
                      if (ar_read(cpu, s) == 0) {
                          BRANCH_TO(cpu, loop_end);
                      } else {
                          cpu->lcount = ar_read(cpu, s) - 1;
                      }
                  } else {
                      /* LOOPGTZ: skip if count <= 0 */
                      if ((int32_t)ar_read(cpu, s) <= 0) {
                          BRANCH_TO(cpu, loop_end);
                      } else {
                          cpu->lcount = ar_read(cpu, s) - 1;
                      }
                  }
                  break;
              }
              default: break;
              }
              break;
          case 2: /* BLTUI */
              { int32_t offset8 = sign_extend(imm8, 8);
                uint32_t target = cpu->pc + (uint32_t)offset8 + 1;
                if (ar_read(cpu, s) < b4constu[r])
                    BRANCH_TO(cpu, target);
              } break;
          case 3: /* BGEUI */
              { int32_t offset8 = sign_extend(imm8, 8);
                uint32_t target = cpu->pc + (uint32_t)offset8 + 1;
                if (ar_read(cpu, s) >= b4constu[r])
                    BRANCH_TO(cpu, target);
              } break;
          }
        } break;
    }
}

/* Execute op0=7 (B) - RRI8 conditional branches */
static void exec_b(xtensa_cpu_t *cpu, uint32_t insn) {
    int r = XT_R(insn);
    int s = XT_S(insn);
    int t = XT_T(insn);
    int imm8 = XT_IMM8(insn);
    int32_t offset = sign_extend(imm8, 8);
    uint32_t target = cpu->pc + (uint32_t)offset + 1;
    uint32_t vs = ar_read(cpu, s);
    uint32_t vt = ar_read(cpu, t);

    int taken = 0;
    switch (r) {
    case 0:  taken = (vs & vt) == 0; break;                       /* BNONE */
    case 1:  taken = vs == vt; break;                              /* BEQ */
    case 2:  taken = (int32_t)vs < (int32_t)vt; break;            /* BLT */
    case 3:  taken = vs < vt; break;                               /* BLTU */
    case 4:  taken = (~vs & vt) == 0; break;                       /* BALL */
    case 5:  taken = !(vs & (1u << (vt & 31))); break;             /* BBC */
    case 6: case 7: /* BBCI: r[3]=0 → clear-test; r[0] selects low(0)/high(1) */
        { int bit = t | ((r & 1) << 4);
          taken = !(vs & (1u << bit));
        } break;
    case 14: case 15: /* BBSI: r[3]=1 → set-test; r[0] selects low(0)/high(1) */
        { int bit = t | ((r & 1) << 4);
          taken = (vs & (1u << bit)) != 0;
        } break;
    case 8:  taken = (vs & vt) != 0; break;                       /* BANY */
    case 9:  taken = vs != vt; break;                              /* BNE */
    case 10: taken = (int32_t)vs >= (int32_t)vt; break;           /* BGE */
    case 11: taken = vs >= vt; break;                              /* BGEU */
    case 12: taken = (~vs & vt) != 0; break;                       /* BNALL */
    case 13: taken = (vs & (1u << (vt & 31))) != 0; break;         /* BBS */
    }

    if (taken)
        BRANCH_TO(cpu, target);
}

/* ===== MAC16 Helpers ===== */

static inline int32_t mac16_half(uint32_t val, int hi) {
    return hi ? (int16_t)(val >> 16) : (int16_t)(val & 0xFFFF);
}

static inline int64_t mac16_get_acc(const xtensa_cpu_t *cpu) {
    return ((int64_t)(int8_t)cpu->acchi << 32) | (uint64_t)cpu->acclo;
}

static inline void mac16_set_acc(xtensa_cpu_t *cpu, int64_t val) {
    cpu->acclo = (uint32_t)val;
    cpu->acchi = (uint32_t)((val >> 32) & 0xFF);
}

static void exec_mac16(xtensa_cpu_t *cpu, uint32_t insn) {
    int op1 = XT_OP1(insn);
    int op2 = XT_OP2(insn);
    int r = XT_R(insn);
    int s = XT_S(insn);
    int t = XT_T(insn);

    /* LDDEC / LDINC: op2=4,5 with op1=0 */
    if (op2 == 4 && (op1 & 0xC) == 0) {
        /* LDDEC: mr[r/4] = mem32[as]; as -= 4 */
        uint32_t addr = ar_read(cpu, s);
        cpu->mr[r >> 2] = mem_read32(cpu->mem, addr);
        ar_write(cpu, s, addr - 4);
        return;
    }
    if (op2 == 5 && (op1 & 0xC) == 0) {
        /* LDINC: mr[r/4] = mem32[as]; as += 4 */
        uint32_t addr = ar_read(cpu, s);
        cpu->mr[r >> 2] = mem_read32(cpu->mem, addr);
        ar_write(cpu, s, addr + 4);
        return;
    }

    /* Get source registers based on op2[3:2] */
    uint32_t src1, src2;
    int reg_mode = (op2 >> 2) & 3;
    switch (reg_mode) {
    case 0: /* AA */ src1 = ar_read(cpu, s); src2 = ar_read(cpu, t); break;
    case 1: /* AD */ src1 = ar_read(cpu, s); src2 = cpu->mr[t >> 1]; break;
    case 2: /* DA */ src1 = cpu->mr[s >> 1]; src2 = ar_read(cpu, t); break;
    case 3: /* DD */ src1 = cpu->mr[s >> 1]; src2 = cpu->mr[t >> 1]; break;
    default: return;
    }

    /* Get half-select from op1[1:0] */
    int sel = op1 & 3;
    int32_t h1 = mac16_half(src1, sel >> 1);
    int32_t h2 = mac16_half(src2, sel & 1);

    /* Operation from op1[3:2] */
    int op = (op1 >> 2) & 3;
    int64_t acc = mac16_get_acc(cpu);
    int64_t product;

    if (op == 3) {
        /* UMUL: unsigned */
        product = (int64_t)((uint32_t)(uint16_t)h1 * (uint32_t)(uint16_t)h2);
        acc = product;
    } else {
        product = (int64_t)h1 * (int64_t)h2;
        switch (op) {
        case 0: acc = product; break;    /* MUL */
        case 1: acc += product; break;   /* MULA */
        case 2: acc -= product; break;   /* MULS */
        }
    }
    mac16_set_acc(cpu, acc);

    /* Combined load for op2=8-11: MULA.xx.yy.LDDEC/LDINC */
    if ((op2 & 0xC) == 8) {
        uint32_t addr = ar_read(cpu, s);
        cpu->mr[r >> 2] = mem_read32(cpu->mem, addr);
        if (op2 & 1)
            ar_write(cpu, s, addr + 4); /* LDINC */
        else
            ar_write(cpu, s, addr - 4); /* LDDEC */
    }
}



/* ===== Main step function (always-inlined into xtensa_run hot loop) ===== */
/* Parameters local_cc/bitmap/hook are cached locals from xtensa_run to keep
 * them in registers instead of reloading from the cpu struct each iteration.
 * local_cc accumulates cycle_count in a register; flushed to cpu->cycle_count
 * only before callbacks that may read it (pc_hook stubs). */

static inline __attribute__((always_inline))
int xtensa_step_impl(xtensa_cpu_t *cpu, uint64_t *restrict local_cc) {
    uint32_t insn;
    if (__builtin_expect(cpu->halted, 0)) {
        cpu->ccount++;
        ++*local_cc;
        if (cpu->ccount == cpu->next_timer_event)
            xtensa_fire_timers(cpu);
        uint32_t pending = cpu->interrupt & cpu->intenable;
        if (pending) {
            cpu->halted = false;
            xtensa_check_interrupts(cpu);
        }
        return cpu->exception ? -1 : 0;
    }

    /* Invalid PC trap: catch jumps to unmapped/non-code regions.
     * Valid ESP32 code: 0x40000000-0x400BFFFF (ROM/IRAM), 0x400D0000-0x404FFFFF (flash) */
    if (__builtin_expect(cpu->pc < 0x40000000u || cpu->pc >= 0x40500000u, 0)) {
        cpu->cycle_count = *local_cc;
        fprintf(stderr, "[TRAP] Invalid PC=0x%08X at cycle %llu (core %d, prid=0x%X)\n",
                cpu->pc, (unsigned long long)*local_cc, cpu->core_id, cpu->prid);
        fprintf(stderr, "  PS=0x%08X SAR=%u WindowBase=%u WindowStart=0x%X\n",
                cpu->ps, cpu->sar, cpu->windowbase, cpu->windowstart);
        for (int r = 0; r < 16; r += 4)
            fprintf(stderr, "  a%-2d=0x%08X  a%-2d=0x%08X  a%-2d=0x%08X  a%-2d=0x%08X\n",
                    r, ar_read(cpu, r), r+1, ar_read(cpu, r+1),
                    r+2, ar_read(cpu, r+2), r+3, ar_read(cpu, r+3));
        fprintf(stderr, "  EPC1=0x%08X EPC2=0x%08X EPC3=0x%08X\n",
                cpu->epc[0], cpu->epc[1], cpu->epc[2]);
        cpu->running = false;
        cpu->exception = true;
        return -1;
    }

    /* Breakpoint check */
    if (__builtin_expect(cpu->breakpoint_count > 0, 0)) {
        cpu->breakpoint_hit = false;
        for (int i = 0; i < cpu->breakpoint_count; i++) {
            if (cpu->breakpoints[i] == cpu->pc) {
                cpu->breakpoint_hit = true;
                cpu->breakpoint_hit_addr = cpu->pc;
                return -1;
            }
        }
    }

    /* PC hook: intercept execution at specific addresses (e.g. ROM stubs).
     * Bitmap fast-path: skip the hook call entirely if the bit isn't set.
     * Flush cycle_count before hook callback since stubs may read it. */
    if (cpu->pc_hook && (!cpu->pc_hook_bitmap ||
        rom_stubs_hook_bitmap_test(cpu->pc_hook_bitmap, cpu->pc))) {
        cpu->cycle_count = *local_cc;  /* flush for stub visibility */
        if (cpu->pc_hook(cpu, cpu->pc, cpu->pc_hook_ctx)) {
            cpu->ccount++;
            ++*local_cc;
            cpu->cycle_count = *local_cc;
            /* Stubs may advance time, so check timers unconditionally */
            if (cpu->ccount == cpu->next_timer_event)
                xtensa_fire_timers(cpu);
            if (__builtin_expect(cpu->irq_check, 0)) {
                cpu->irq_check = false;
                if (cpu->interrupt & cpu->intenable)
                    xtensa_check_interrupts(cpu);
            }
            *local_cc = cpu->cycle_count;  /* reload (stub may advance time) */
            return cpu->exception ? -1 : 0;
        }
    }

    int ilen;
#if PREDECODE_SIZE > 0
    /* Pre-decoded instruction table: single indexed load replaces
     * page_table lookup + byte assembly + ilen determination. */
    if (__builtin_expect(cpu->predecode != NULL, 1)) {
        uint32_t pc_off = cpu->pc - PREDECODE_BASE;
        if (__builtin_expect(pc_off < PREDECODE_SIZE, 1)) {
            uint32_t packed = cpu->predecode[pc_off];
            if (__builtin_expect(packed != 0, 1)) {
                insn = PREDECODE_INSN(packed);
                ilen = (int)PREDECODE_ILEN(packed);
                goto have_insn;
            }
        }
    }
#endif
    ilen = xtensa_fetch_inline(cpu, cpu->pc, &insn);
    if (__builtin_expect(ilen == 0, 0)) {
        xtensa_raise_exception(cpu, EXCCAUSE_IFETCH_ERROR, cpu->pc, 0);
        if (cpu->exception) return -1;
        return 0;
    }
#if PREDECODE_SIZE > 0
have_insn:
#endif

    cpu->_pc_written = false;
    cpu->pc += (uint32_t)ilen;

    if (ilen == 2) {
        exec_narrow(cpu, insn);
    } else {
        switch (XT_OP0(insn)) {
        case 0: exec_qrst(cpu, insn); break;
        case 1: /* L32R */
            { int lt = XT_T(insn);
              uint16_t imm16 = (uint16_t)XT_IMM16(insn);
              uint32_t target = (cpu->pc & ~3u) + (0xFFFC0000u | ((uint32_t)imm16 << 2));
              ar_write(cpu, lt, mem_read32(cpu->mem, target));
            } break;
        case 2: exec_lsai(cpu, insn); break;
        case 3: /* LSCI - FP loads/stores */
            { int lr = XT_R(insn);
              int ls = XT_S(insn);
              int lt = XT_T(insn);
              int limm8 = XT_IMM8(insn);
              uint32_t base = ar_read(cpu, ls);
              uint32_t offset = (uint32_t)(limm8 << 2);
              switch (lr) {
              case 0: /* LSI */
                  { uint32_t tmp = mem_read32(cpu->mem, base + offset);
                    memcpy(&cpu->fr[lt], &tmp, 4);
                  } break;
              case 4: /* SSI */
                  { uint32_t tmp; memcpy(&tmp, &cpu->fr[lt], 4);
                    mem_write32(cpu->mem, base + offset, tmp);
                  } break;
              case 8: /* LSIU */
                  { uint32_t tmp = mem_read32(cpu->mem, base + offset);
                    memcpy(&cpu->fr[lt], &tmp, 4);
                    ar_write(cpu, ls, base + offset);
                  } break;
              case 12: /* SSIU */
                  { uint32_t tmp; memcpy(&tmp, &cpu->fr[lt], 4);
                    mem_write32(cpu->mem, base + offset, tmp);
                    ar_write(cpu, ls, base + offset);
                  } break;
              default: break;
              }
            } break;
        case 4: exec_mac16(cpu, insn); break;
        case 5: exec_calln(cpu, insn); break;
        case 6: exec_si(cpu, insn); break;
        case 7: exec_b(cpu, insn); break;
        default: break;
        }
    }

    /* Zero-overhead loop */
    if (__builtin_expect(cpu->lcount > 0, 0) &&
        cpu->pc == cpu->lend && !cpu->_pc_written) {
        cpu->lcount--;
        cpu->pc = cpu->lbeg;
    }

    cpu->ccount++;
    ++*local_cc;

    if (__builtin_expect(cpu->ccount == cpu->next_timer_event, 0))
        xtensa_fire_timers(cpu);

    /* Interrupt check — only when interrupt state has changed
     * (timer fired, INTSET/INTENABLE written, peripheral set a bit). */
    if (__builtin_expect(cpu->irq_check, 0)) {
        cpu->irq_check = false;
        if (cpu->interrupt & cpu->intenable)
            xtensa_check_interrupts(cpu);
    }

    return cpu->exception ? -1 : 0;
}

/* External entry point (for single-step / trace callers).
 * Always checks timers + interrupts unconditionally (no batching). */
int xtensa_step(xtensa_cpu_t *cpu) {
    uint64_t cc = cpu->cycle_count;
    int r = xtensa_step_impl(cpu, &cc);
    cpu->cycle_count = cc;
    if (cpu->ccount == cpu->next_timer_event)
        xtensa_fire_timers(cpu);
    if (cpu->interrupt & cpu->intenable)
        xtensa_check_interrupts(cpu);
    return r;
}

/* Batch execution: step_impl is always_inline → entire decode/execute loop
 * lives in this function body, eliminating per-instruction call overhead.
 * cycle_count is cached in a local to stay in a register across iterations
 * (avoids per-instruction 64-bit memory increment). */
__attribute__((no_stack_protector))
int xtensa_run(xtensa_cpu_t *cpu, int max_cycles) {
    uint64_t cc = cpu->cycle_count;
    int i;
    for (i = 0; i < max_cycles; i++) {
        if (__builtin_expect(xtensa_step_impl(cpu, &cc) != 0, 0))
            break;
        if (__builtin_expect(!cpu->running, 0))
            break;
    }
    cpu->cycle_count = cc;
    return i;
}

/* ===== Breakpoint API ===== */

int xtensa_set_breakpoint(xtensa_cpu_t *cpu, uint32_t addr) {
    if (cpu->breakpoint_count >= MAX_BREAKPOINTS) return -1;
    /* Check for duplicate */
    for (int i = 0; i < cpu->breakpoint_count; i++)
        if (cpu->breakpoints[i] == addr) return 0;
    cpu->breakpoints[cpu->breakpoint_count++] = addr;
    return 0;
}

int xtensa_clear_breakpoint(xtensa_cpu_t *cpu, uint32_t addr) {
    for (int i = 0; i < cpu->breakpoint_count; i++) {
        if (cpu->breakpoints[i] == addr) {
            cpu->breakpoints[i] = cpu->breakpoints[--cpu->breakpoint_count];
            return 0;
        }
    }
    return -1;
}

void xtensa_clear_all_breakpoints(xtensa_cpu_t *cpu) {
    cpu->breakpoint_count = 0;
}

/* xtensa_disasm() is in xtensa_disasm.c */
