#ifdef _MSC_VER
#include "msvc_compat.h"
#endif

#include "xtensa.h"
#include "guest_call.h"
#include "memory.h"
#include "rom_stubs.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

/* Set PC from instruction handler (branch/call/ret/exception) and mark it */
#define BRANCH_TO(cpu, addr) do { (cpu)->pc = (addr); (cpu)->_pc_written = true; } while(0)

/* TEMP DEBUG: window-ops trace + forward decls (definitions near bottom) */
extern int g_dbg_winlog;
extern int g_dbg_c1ilog;



extern uint32_t g_dbg_watch_addr;
extern uint32_t g_dbg_watch_addr2;
extern uint32_t g_dbg_watch_val;
static int g_dbg_wvlog;
#define WINLOG(cpu, fmt, ...) do { if (g_dbg_winlog) \
    fprintf(stderr, "[W%d] %s pc=0x%08X wb=%d ws=%04X " fmt, \
            (cpu)->core_id, __func__, (cpu)->pc, (cpu)->windowbase, \
            (cpu)->windowstart, ##__VA_ARGS__); } while (0)

/* Cache the only WINDOWSTART bits the per-instruction operand guard needs.
 * Public callers may edit the exposed CPU state between runs, so xtensa_run
 * and xtensa_step refresh this too; internal mutations refresh in place. */
static inline __attribute__((always_inline))
void window_hazard_refresh(xtensa_cpu_t *cpu) {
    if (!cpu->real_window_vectors) {
        cpu->window_hazard = 0;
        return;
    }
    uint32_t ws = cpu->windowstart & 0xFFFFu;
    unsigned sh = (cpu->windowbase + 1u) & 0xFu;
    cpu->window_hazard = (uint8_t)(((ws | (ws << 16)) >> sh) & 7u);
}

/* Recompute the nearest ccompare value for timer batching.
 * A ccompare in the past (wrapped distance >= 2^31) is due NOW — give it
 * distance 0 so it is picked and fired at the next opportunity. A ccompare
 * of 0 is treated as disarmed (matches init / no-timer state). */
static inline void xtensa_recompute_next_timer_impl(xtensa_cpu_t *cpu) {
    /* Once a compare interrupt is latched it is no longer a future event.
     * Firmware clears it by writing CCOMPAREn, which recomputes this cache.
     * Keeping an already-latched compare here made a WAITI core revisit the
     * same event on every idle cycle and hid later enabled timers. */
    static const uint8_t int_bit[3] = { 6, 15, 16 };
    bool have_event = false;
    uint32_t best_distance = 0;
    uint32_t best_event = UINT32_MAX;

    for (int i = 0; i < 3; i++) {
        if (!cpu->ccompare[i] || (cpu->interrupt & (1u << int_bit[i])))
            continue;
        uint32_t distance = cpu->ccompare[i] - cpu->ccount;
        if ((int32_t)distance < 0)
            distance = 0;
        if (!have_event || distance < best_distance) {
            have_event = true;
            best_distance = distance;
            best_event = cpu->ccompare[i];
        }
    }
    /* Fold in peripheral timer events (TIMG LACT alarms) */
    if (cpu->periph_next_event) {
        uint32_t e = cpu->periph_next_event(cpu);
        if (e != UINT32_MAX) {
            uint32_t distance = e - cpu->ccount;
            if ((int32_t)distance < 0)
                distance = 0;
            if (!have_event || distance < best_distance) {
                have_event = true;
                best_event = e;
            }
        }
    }
    uint32_t next = have_event ? best_event : UINT32_MAX;
    if (next != cpu->next_timer_event)
        xtensa_native_chain_barrier(cpu);
    cpu->next_timer_event = next;
}

void xtensa_recompute_next_timer(xtensa_cpu_t *cpu) {
    xtensa_recompute_next_timer_impl(cpu);
}

/* Forward declaration: definition follows xtensa_fire_timers() below. */
void xtensa_fire_due_timers(xtensa_cpu_t *cpu);

/* Fire any ccompare timers whose time has arrived, hardware semantics:
 * the interrupt is raised when ccount >= ccompare (not exact equality),
 * so overshooting the target (stubs advancing time, batch accounting)
 * can never lose a tick. ccompare == 0 means disarmed. */
static inline void xtensa_fire_timers(xtensa_cpu_t *cpu) {
    uint32_t old = cpu->interrupt;
    if (cpu->ccompare[0] && (int32_t)(cpu->ccount - cpu->ccompare[0]) >= 0)
        cpu->interrupt |= (1u << 6);
    if (cpu->ccompare[1] && (int32_t)(cpu->ccount - cpu->ccompare[1]) >= 0)
        cpu->interrupt |= (1u << 15);
    if (cpu->ccompare[2] && (int32_t)(cpu->ccount - cpu->ccompare[2]) >= 0)
        cpu->interrupt |= (1u << 16);
    if (cpu->interrupt != old) xtensa_request_irq_check(cpu);
    /* Peripheral timer events (TIMG LACT alarms) */
    if (cpu->periph_event) cpu->periph_event(cpu);
    xtensa_recompute_next_timer_impl(cpu);
}

void xtensa_fire_due_timers(xtensa_cpu_t *cpu) {
    xtensa_fire_timers(cpu);
}

/* Where the ROM keeps ticks-per-microsecond; the same word esp_timer_stubs.c,
 * freertos_stubs.c and peripherals.c each read for the current CPU clock. */
#define ESP32_CPU_TICKS_PER_US_ADDR 0x3FFE01E0u

uint32_t xtensa_cpu_freq_mhz(const xtensa_cpu_t *cpu) {
    uint32_t mhz = cpu->mem ? mem_read32(cpu->mem, ESP32_CPU_TICKS_PER_US_ADDR) : 0;
    return (mhz >= 10u && mhz <= 240u) ? mhz : 160u;
}

/* Move a core's CCOUNT forward across time it did not execute, firing each
 * ccompare and peripheral event inside the interval at its own ccount rather
 * than all of them at the end.
 *
 * Anything that advances a core's clock without running it has to come through
 * here. Assigning cpu->ccount directly steps straight over the guest's own
 * FreeRTOS tick (CCOMPARE0 -> interrupt 6), and because an interrupt latches
 * rather than counts, every tick in the interval collapses into at most one.
 * The guest's tick counter then runs slow and every FreeRTOS timeout measured
 * against it runs long.
 *
 * Stepping stops as soon as an enabled interrupt is latched: the guest cannot
 * observe a second one until it runs again, so visiting further boundaries
 * costs recomputes and buys nothing. */
void xtensa_advance_idle_cycles(xtensa_cpu_t *cpu, uint64_t cycles) {
    while (cycles > 0) {
        uint32_t next = cpu->next_timer_event;
        if (next == UINT32_MAX) break;
        uint32_t distance = ((int32_t)(cpu->ccount - next) >= 0)
                          ? 0u : (uint32_t)(next - cpu->ccount);
        if ((uint64_t)distance > cycles) break;
        cpu->ccount += distance;
        cycles -= distance;
        xtensa_fire_timers(cpu);
        if (cpu->next_timer_event == next) break;   /* no forward progress */
        if (cpu->interrupt & cpu->intenable) break; /* guest must run to see it */
    }
    if (cycles > 0) {
        cpu->ccount += (uint32_t)cycles;
        xtensa_fire_timers(cpu);
    }
}

void xtensa_cpu_init_for_target(xtensa_cpu_t *cpu,
                                const flexe_target_desc_t *target) {
    if (!target)
        target = flexe_target_by_id(FLEXE_TARGET_ESP32);
    memset(cpu, 0, sizeof(*cpu));
    cpu->target = target;
    cpu->core_id = 0;
    {   /* Read once, per CPU, so the hot paths test a field, never getenv(). */
        static int enabled = -1;
        if (enabled < 0) {
            const char *e = getenv("FLEXE_WINDOW_VECTORS");
            /* Guest-owned window vectors are the architectural path. Keep the
             * old synthesized implementation as an explicit diagnostic
             * fallback with FLEXE_WINDOW_VECTORS=0. */
            enabled = e ? atoi(e) : 1;
        }
        cpu->real_window_vectors = enabled != 0;
    }
    cpu->next_timer_event = UINT32_MAX;  /* No timer pending until ccompare is written */
    cpu->jit_chain_limit = UINT32_MAX;   /* Direct JIT-block callers are unbounded */
    /* First step counts as a control-flow transfer so the PC hook /
     * AOT bitmap gates fire on the initial PC (entry vector). */
    cpu->_pc_written = true;

    memcpy(cpu->int_level, target->interrupt_level, sizeof(cpu->int_level));
}

void xtensa_cpu_init(xtensa_cpu_t *cpu) {
    xtensa_cpu_init_for_target(cpu, flexe_target_by_id(FLEXE_TARGET_ESP32));
}

void xtensa_cpu_reset_for_target(xtensa_cpu_t *cpu,
                                 const flexe_target_desc_t *target) {
    if (!target)
        target = flexe_target_by_id(FLEXE_TARGET_ESP32);
    xtensa_cpu_init_for_target(cpu, target);

    cpu->pc = target->reset_vector;

    /* PS: WOE=1, EXCM=1, INTLEVEL=15 */
    cpu->ps = (1 << 18)    /* WOE */
            | (1 << 4)     /* EXCM */
            | 0xF;         /* INTLEVEL = 15 */

    /* Window registers */
    cpu->windowbase = 0;
    cpu->windowstart = 1;   /* Window 0 is valid */
    window_hazard_refresh(cpu);
    /* Default every slot to call8 (the near-universal IDF/GCC call size);
     * ENTRY records the real callsize as windows are created. */
    memset(cpu->window_callsize, 2, sizeof(cpu->window_callsize));

    /* SAR undefined, set to 0 */
    cpu->sar = 0;
    cpu->lcount = 0;
    cpu->ccount = 0;

    /* Core-configuration defaults. */
    cpu->vecbase = target->vecbase_reset;
    cpu->prid = 0xCDCD;        /* PRO_CPU */
    cpu->cpenable = 0;
    cpu->atomctl = 0x28;
    cpu->configid0 = target->configid0;
    cpu->configid1 = target->configid1;

    cpu->running = true;
}

void xtensa_cpu_reset(xtensa_cpu_t *cpu) {
    xtensa_cpu_reset_for_target(cpu, flexe_target_by_id(FLEXE_TARGET_ESP32));
}

/* Fast inline fetch for the hot path — avoids function call overhead */
static inline __attribute__((always_inline))
int xtensa_fetch_inline(const xtensa_cpu_t *cpu, uint32_t addr, uint32_t *insn_out) {
    uint8_t *page = cpu->mem->page_table[addr >> 12];
    if (__builtin_expect(!page, 0)) return 0;
    uint32_t page_off = addr & 0xFFFu;
    const uint8_t *ptr = page + page_off;
    uint32_t b0 = ptr[0];
    uint32_t ilen = (b0 & 0x8u) ? 2u : 3u;

    /* Nearly every instruction is wholly within its current page. Keep that
     * path to one page-table lookup, but resolve boundary bytes separately:
     * adjacent guest pages can be unmapped or backed by unrelated MMU pages. */
    if (__builtin_expect(page_off + ilen <= 0x1000u, 1)) {
        if (ilen == 2u) {
            *insn_out = b0 | ((uint32_t)ptr[1] << 8);
            return 2;
        }
        *insn_out = b0 | ((uint32_t)ptr[1] << 8) | ((uint32_t)ptr[2] << 16);
        return 3;
    }

    if (__builtin_expect(addr > UINT32_MAX - (ilen - 1u), 0)) return 0;
    uint8_t *page1 = cpu->mem->page_table[(addr + 1u) >> 12];
    if (__builtin_expect(!page1, 0)) return 0;
    uint32_t b1 = page1[(addr + 1u) & 0xFFFu];
    if (ilen == 2u) {
        *insn_out = b0 | (b1 << 8);
        return 2;
    }
    uint8_t *page2 = cpu->mem->page_table[(addr + 2u) >> 12];
    if (__builtin_expect(!page2, 0)) return 0;
    uint32_t b2 = page2[(addr + 2u) & 0xFFFu];
    *insn_out = b0 | (b1 << 8) | (b2 << 16);
    return 3;
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
    /* The current direct-index table is deliberately optimized around the
     * classic ESP32's compact 0x4000... instruction geometry. Never build a
     * misleading table for another target; its interpreter fetch path remains
     * correct until predecode storage becomes descriptor-driven. */
    if (!cpu || !cpu->target || cpu->target->id != FLEXE_TARGET_ESP32)
        return;
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

void xtensa_invalidate_code(xtensa_cpu_t *cpu, uint32_t addr, size_t len) {
    if (!cpu || len == 0) return;

#if PREDECODE_SIZE > 0
    if (cpu->predecode) {
        uint64_t start = addr;
        uint64_t end = start + (uint64_t)len;
        uint64_t pd_start = PREDECODE_BASE;
        uint64_t pd_end = PREDECODE_END;
        if (start < pd_end && end > pd_start) {
            uint64_t overlap_start = start > pd_start ? start : pd_start;
            uint64_t overlap_end = end < pd_end ? end : pd_end;
            memset(cpu->predecode + (size_t)(overlap_start - pd_start), 0,
                   (size_t)(overlap_end - overlap_start) * sizeof(uint32_t));
        }
    }
#endif

    if (cpu->code_invalidate)
        cpu->code_invalidate(cpu->code_invalidate_ctx, addr, len);
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
    case XT_SR_WINDOWBASE:
        cpu->windowbase = val & 0xF;
        window_hazard_refresh(cpu);
        break;
    case XT_SR_WINDOWSTART:
        cpu->windowstart = val & 0xFFFF;
        window_hazard_refresh(cpu);
        break;
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
    case XT_SR_INTSET:   cpu->interrupt |= val; xtensa_request_irq_check(cpu); break;
    case XT_SR_INTCLEAR: cpu->interrupt &= ~val; break;
    case XT_SR_INTENABLE:   cpu->intenable = val; xtensa_request_irq_check(cpu); break;
    /* PS carries the interrupt mask (INTLEVEL) and EXCM, so writing it can
     * unmask an interrupt that is already pending. `irq_check` is the hint
     * that says "re-evaluate", and it used to be set only when `interrupt`
     * or `intenable` changed -- so an interrupt asserted while INTLEVEL was
     * high got tested once, declined, and was then never reconsidered when
     * the guest lowered the level. It fired late, whenever some unrelated
     * event happened to set the hint again. Every path that writes PS needs
     * this; the re-evaluation itself re-tests everything, so setting it
     * where nothing was unmasked costs only the store. */
    case XT_SR_PS:          cpu->ps = val;
                            xtensa_request_irq_check(cpu);
                            break;
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
    if (__builtin_expect(!flexe_target_pc_is_executable(cpu->target,
                                                        fault_pc), 0)) {
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

static void synth_spill_window(xtensa_cpu_t *cpu, int widx);
extern int g_flexe_shadow_fill;
static inline uint32_t phys_read(const xtensa_cpu_t *cpu, int widx, int reg);

/* ===== Real window exceptions =====
 *
 * Flexe's default is to synthesize register-window spill and fill in C.
 * That cannot be made right, and the reason is structural: the shadow records
 * are keyed by *physical* window while what they describe is a *logical*
 * frame, and the two stop coinciding the moment the window file wraps or the
 * guest writes WINDOWSTART. Real firmware does write it -- newlib's longjmp
 * executes `wsr.windowstart 1<<wb` to declare every other window already
 * saved, which is sound on hardware because the matching setjmp called
 * xthal_window_spill first.
 *
 * The alternative is to do what the chip does: fault into the guest's own
 * handlers. Every IDF image carries the six of them at VECBASE, and they are
 * short -- WindowOverflow4 is four s32e and an rfwo. Hardware never has to know
 * which logical frame a slot holds, because the handler derives the save
 * address from the callee's own a1.
 *
 * The handler runs with WindowBase rotated onto the frame being moved and
 * PS.OWB holding where to return; RFWO and RFWU already implement that. EPC1
 * is the faulting instruction, which re-executes and this time finds the
 * WindowStart bit it needed.
 */
/* Only fault once the application's vectors are actually in place. VECBASE
 * starts at the ROM's 0x40000000, whose window handlers Flexe does not model,
 * and startup runs for millions of instructions before the app relocates it to
 * IRAM. Faulting into unmapped ROM there turns the first wrap into 50M
 * unregistered ROM calls, which is precisely what it did. Until then the
 * synthesized path carries it, exactly as it does today. */
static inline bool window_vectors_ready(const xtensa_cpu_t *cpu,
                                        uint32_t vecofs) {
    return cpu->vecbase >= 0x40070000u && cpu->vecbase < 0x40400000u &&
           mem_get_ptr(cpu->mem, cpu->vecbase + vecofs) != NULL;
}

static void raise_window_exception(xtensa_cpu_t *cpu, uint32_t fault_pc,
                                   int handler_wb, uint32_t vecofs) {
    if (__builtin_expect(g_dbg_wvlog, 0)) {
        static int n;
        if (n++ < 12)
            fprintf(stderr, "[wv] +0x%03X wb=%d->%d ws=%04X a0=%08X a1=%08X "
                    "pc=%08X vec=%08X core%d\n", vecofs, cpu->windowbase,
                    handler_wb & 0xF, cpu->windowstart,
                    phys_read(cpu, handler_wb & 0xF, 0),
                    phys_read(cpu, handler_wb & 0xF, 1),
                    fault_pc, cpu->vecbase + vecofs, cpu->core_id);
    }
    cpu->epc[0] = fault_pc;
    XT_PS_SET_OWB(cpu->ps, cpu->windowbase);
    XT_PS_SET_EXCM(cpu->ps, 1);
    cpu->windowbase = handler_wb & 0xF;
    window_hazard_refresh(cpu);
    WINLOG(cpu, "WINDOW EXC +0x%03X owb=%d wb=%d epc=%08X ws=%04X\n",
           vecofs, XT_PS_OWB(cpu->ps), cpu->windowbase, fault_pc,
           cpu->windowstart);
    BRANCH_TO(cpu, cpu->vecbase + vecofs);
}

/* Which of the six, from the call size recorded in the moving window's a0.
 * A frame whose a0 carries none is a task bottom; the 4-register form is the
 * smallest the ABI defines and is the right handler for it. */
static uint32_t window_vec_offset(uint32_t a0, bool underflow) {
    unsigned n = (a0 >> 30) & 3u;
    if (n == 0) n = 1;
    return (underflow ? VECOFS_WINDOW_UNDERFLOW4 : VECOFS_WINDOW_OVERFLOW4) +
           (uint32_t)(n - 1) * 0x80u;
}

/* The per-instruction window check: hardware faults on any access to a4-a15
 * whose physical window belongs to another live frame. That is what
 * xthal_window_spill leans on -- it walks the file with `mov.n a12, a0;
 * rotw 3` and the register touch is the whole mechanism.
 *
 * It does not decode which registers the instruction reads: it tests all three
 * windows the current frame can reach, every instruction. More eager than the
 * hardware and harmless, because a WindowStart bit is set only at a live
 * frame's *base* window -- a set bit in WB+1..WB+3 always means a foreign frame
 * is aliased there, and moving it early only does work the next a4-a15 access
 * would have forced. Rotate WindowStart so bit 0 is WB+1, test three bits;
 * they are zero unless the register file has wrapped.
 */
static int find_callee_window(xtensa_cpu_t *cpu, int widx);

/* Which of Overflow4/8/12. The ISA derives this entirely from WINDOWSTART:
 * after rotating to the colliding window, the distance to the next live bit
 * says whether its callee's SP is visible as a5, a9, or a13. Synthetic
 * call-chain metadata is deliberately irrelevant here. It can be stale after
 * longjmp or a context restore, while WINDOWSTART is the architectural source
 * of truth the hardware uses. */
static inline uint32_t window_overflow_vec(xtensa_cpu_t *cpu, int w) {
    int d;
    for (d = 1; d < 16; d++) {
        if (cpu->windowstart & (1u << ((w + d) & 0xF)))
            break;
    }
    if (d > 3) d = 3;
    return VECOFS_WINDOW_OVERFLOW4 + (uint32_t)(d - 1) * 0x80u;
}

bool xtensa_try_window_overflow_exception(xtensa_cpu_t *cpu,
                                          uint32_t fault_pc,
                                          unsigned window_need) {
    if (!cpu || window_need == 0u || !cpu->real_window_vectors ||
        !XT_PS_WOE(cpu->ps) || XT_PS_EXCM(cpu->ps))
        return false;

    if (window_need > 3u) window_need = 3u;
    int hit = -1;
    for (unsigned adjacent = 1; adjacent <= window_need; adjacent++) {
        int w = ((int)cpu->windowbase + (int)adjacent) & 0xF;
        if (cpu->windowstart & (1u << w)) {
            hit = w;
            break;
        }
    }
    if (hit < 0) return false;

    uint32_t vecofs = window_overflow_vec(cpu, hit);
    if (!window_vectors_ready(cpu, vecofs))
        return false;
    raise_window_exception(cpu, fault_pc, hit, vecofs);
    return true;
}

bool xtensa_try_entry_overflow_exception(xtensa_cpu_t *cpu,
                                         uint32_t fault_pc) {
    if (!cpu || !cpu->real_window_vectors || !XT_PS_WOE(cpu->ps) ||
        XT_PS_EXCM(cpu->ps))
        return false;

    unsigned callinc = (unsigned)XT_PS_CALLINC(cpu->ps);
    int hit = -1;
    for (unsigned adjacent = 1; adjacent <= callinc; adjacent++) {
        int w = ((int)cpu->windowbase + (int)adjacent) & 0xF;
        if (cpu->windowstart & (1u << w)) {
            hit = w;
            break;
        }
    }
    if (hit < 0) return false;

    uint32_t vecofs = window_overflow_vec(cpu, hit);
    if (!window_vectors_ready(cpu, vecofs))
        return false;
    raise_window_exception(cpu, fault_pc, hit, vecofs);
    return true;
}

static inline unsigned window_need2(unsigned a, unsigned b) {
    return (a > b ? a : b) >> 2;
}

static inline unsigned window_need3(unsigned a, unsigned b, unsigned c) {
    unsigned m = a > b ? a : b;
    return (m > c ? m : c) >> 2;
}

/* Highest logical AR window an instruction actually touches.  Xtensa reuses
 * the r/s/t bit positions for immediates, FP/boolean/MAC registers and opcode
 * selectors in several formats.  Treating all three nibbles as AR operands
 * makes harmless instructions such as NOP.N (whose fixed r field is 15)
 * raise a WindowOverflow12 exception.  Mirror the implemented decoder here,
 * just as QEMU builds its windowed-register mask from decoded operands. */
static inline unsigned window_operand_need(const xtensa_cpu_t *cpu,
                                           uint32_t insn, int ilen) {
    unsigned t = XT_T(insn);
    unsigned s = XT_S(insn);
    unsigned r = XT_R(insn);

    if (ilen == 2) {
        switch (XT_OP0(insn)) {
        case 0x8: /* L32I.N */
        case 0x9: /* S32I.N */
            return window_need2(s, t);
        case 0xA: /* ADD.N */
            return window_need3(r, s, t);
        case 0xB: /* ADDI.N: t is the immediate */
            return window_need2(r, s);
        case 0xC: /* MOVI.N / BEQZ.N / BNEZ.N */
            return s >> 2;
        case 0xD: /* MOV.N, or fixed encodings with no high AR operand */
            return r == 0 ? window_need2(s, t) : 0;
        default:
            return 0;
        }
    }

    unsigned op0 = XT_OP0(insn);
    unsigned op1 = XT_OP1(insn);
    unsigned op2 = XT_OP2(insn);
    switch (op0) {
    case 0: /* QRST */
        switch (op1) {
        case 0: /* RST0 */
            if (op2 == 0) { /* ST0 specials */
                unsigned m = XT_M(insn);
                unsigned nn = XT_N(insn);
                switch (r) {
                case 0:
                    if (m == 2 && nn == 2) return s >> 2; /* JX */
                    if (m == 3) return window_need2(s, nn * 4u); /* CALLXn */
                    return 0; /* RET/RETW and fixed control encodings */
                case 1: return window_need2(s, t); /* MOVSP */
                case 6: return t >> 2;              /* RSIL; s is a level */
                default: return 0;
                }
            }
            if (op2 >= 1 && op2 <= 3)
                return window_need3(r, s, t); /* AND/OR/XOR */
            if (op2 == 4) { /* shift setup / ROTW / NSA / external regs */
                if (r <= 3) return s >> 2;
                if (r == 6 || r == 7 || r == 14 || r == 15)
                    return window_need2(s, t);
                return 0; /* SSAI and ROTW operands are immediates */
            }
            if (op2 == 6) return window_need2(r, t); /* NEG/ABS */
            if (op2 >= 8) return window_need3(r, s, t); /* integer ALU */
            return 0;

        case 1: /* RST1 */
            switch (op2) {
            case 0: case 1: return window_need2(r, s); /* SLLI */
            case 2: case 3: case 4: return window_need2(r, t); /* SRAI/SRLI */
            case 6: return t >> 2; /* XSR; r/s encode the SR */
            case 8: case 12: case 13: return window_need3(r, s, t);
            case 9: case 11: return window_need2(r, t);
            case 10: return window_need2(r, s);
            default: return 0;
            }

        case 2: /* RST2: boolean ops 0..4 have no AR operands */
            return (op2 == 6 || op2 == 7 || op2 == 8 || op2 >= 10)
                 ? window_need3(r, s, t) : 0;

        case 3: /* RST3 */
            switch (op2) {
            case 0: case 1: return t >> 2; /* RSR/WSR */
            case 2: case 3: return window_need2(r, s); /* SEXT/CLAMPS */
            case 4: case 5: case 6: case 7:
            case 8: case 9: case 10: case 11:
                return window_need3(r, s, t);
            case 12: case 13: return window_need2(r, s); /* t is boolean */
            case 14: return r >> 2; /* RUR: r=dest, s/t=user reg */
            case 15: return t >> 2; /* WUR: t=source, r/s=user reg */
            default: return 0;
            }

        case 4: case 5: /* EXTUI: s/op2 are immediates */
            return window_need2(r, t);

        case 8: /* indexed FP load/store: r is an FP register */
            return (op2 == 0 || op2 == 1 || op2 == 4 || op2 == 5)
                 ? window_need2(s, t) : 0;

        case 9: /* L32E/S32E: r is the displacement */
            return (op2 == 0 || op2 == 4) ? window_need2(s, t) : 0;

        case 10: /* FP0 */
            if (op2 >= 8 && op2 <= 11) return r >> 2;
            if (op2 == 12 || op2 == 13) return s >> 2;
            if (op2 == 14) return r >> 2;
            if (op2 == 15 && t == 4) return r >> 2; /* RFR */
            if (op2 == 15 && t == 5) return s >> 2; /* WFR */
            return 0;

        case 11: /* FP1: only the integer-conditioned forms touch AR[t] */
            return (op2 >= 8 && op2 <= 11) ? t >> 2 : 0;
        default:
            return 0;
        }

    case 1: /* L32R: r/s form the literal displacement */
        return t >> 2;

    case 2: /* LSAI: r selects the operation */
        if (r == 0xA) return t >> 2; /* MOVI */
        if (r == 0x0 || r == 0x1 || r == 0x2 || r == 0x4 ||
            r == 0x5 || r == 0x6 || r == 0xB || r == 0xC ||
            r == 0xD || r == 0xE || r == 0xF)
            return window_need2(s, t);
        if (r == 0x7) return s >> 2; /* cache address */
        return 0;

    case 3: /* LSI/SSI/LSIU/SSIU: only the AR base is integer */
        return s >> 2;

    case 4: { /* MAC16: r names an accumulator/MR, not AR */
        if ((op2 & 0xCu) == 8u) return window_need2(s, t);
        switch ((op2 >> 2) & 3u) {
        case 0: return window_need2(s, t); /* AA */
        case 1: return s >> 2;             /* AD */
        case 2: return t >> 2;             /* DA */
        default: return 0;                 /* DD */
        }
    }

    case 5: /* CALLn writes a0/a4/a8/a12 */
        return XT_N(insn);

    case 6: { /* J / immediate branches / LOOP / ENTRY */
        unsigned nn = XT_N(insn);
        unsigned m = XT_M(insn);
        if (nn == 0) return 0;
        if (nn == 1 || nn == 2) return s >> 2;
        if (m == 0) { /* ENTRY source and CALLINC-selected destination */
            unsigned dst = ((unsigned)XT_PS_CALLINC(cpu->ps) << 2) | (s & 3u);
            return window_need2(s, dst);
        }
        if (m == 1) return (r >= 8 && r <= 10) ? s >> 2 : 0;
        return s >> 2;
    }

    case 7: /* register and immediate-bit branches */
        return (r == 6 || r == 7 || r == 14 || r == 15)
             ? s >> 2 : window_need2(s, t);
    default:
        return 0;
    }
}

unsigned xtensa_window_operand_need(const xtensa_cpu_t *cpu,
                                    uint32_t insn, int ilen) {
    return window_operand_need(cpu, insn, ilen);
}

static inline bool window_access_check(xtensa_cpu_t *cpu, uint32_t insn,
                                       int ilen, unsigned hazard) {
    /* Exception handlers run with EXCM set and cannot take a nested window
     * fault.  Reject that state before decoding the instruction's register
     * operands; window spill/fill handlers otherwise pay the full decoder on
     * every one of their own loads and stores. */
    if (!XT_PS_WOE(cpu->ps) || XT_PS_EXCM(cpu->ps))
        return false;

    unsigned need = window_operand_need(cpu, insn, ilen);
    if (need == 0u || (hazard & ((1u << need) - 1u)) == 0u)
        return false;
    return xtensa_try_window_overflow_exception(cpu, cpu->pc, need);
}



/*
 * SPILL_ALL_WINDOWS emulation.
 *
 * On real hardware, every interrupt entry runs _xt_context_save, which executes
 * SPILL_ALL_WINDOWS: all live register windows of the interrupted context are
 * flushed to its stack via WindowOverflow exceptions, and their WindowStart
 * bits are cleared. flexe does not deliver window exceptions, so the flush is
 * done here, at interrupt delivery. Without it, an outgoing task's caller
 * windows stay marked valid in the register file; after a context switch the
 * incoming task's first RETW can then consume a foreign task's window data
 * (wrong stack, wrong task), corrupting the task context permanently.
 *
 * The spill bases are computed from each window's own a1 chain (intact at
 * delivery), so the flush also establishes a consistent spill_stack state for
 * the ISR and for the eventually-restored task: post-interrupt underflow
 * fills pop the flush's entries in LIFO order, which is correct both when
 * returning to the same task and after a context switch.
 */
void xtensa_flush_windows(xtensa_cpu_t *cpu) {
    /* Spill ALL non-current windows (wb+1..wb+15). The FreeRTOS context
     * switch requires a clean register file: the incoming task restores its
     * own windowbase/windowstart and must never inherit the outgoing task's
     * live frames. Spilling only wb+1..wb+13 (leaving wb-1/wb-2 live) was
     * tried and deadlocks the esp_ipc cross-core handshake at boot, because
     * the ipc task then resumes into a register file still holding the idle
     * task's predecessor frames. */
    for (int w = 1; w < 16; w++) {
        int idx = (cpu->windowbase + w) & 0xF;
        if (cpu->windowstart & (1u << idx))
            synth_spill_window(cpu, idx);
    }
}

uint64_t g_xtensa_irq_dispatched;

void xtensa_check_interrupts(xtensa_cpu_t *cpu) {
    uint32_t pending = cpu->interrupt & cpu->intenable;
    if (!pending) return;

    int eff_level = XT_PS_INTLEVEL(cpu->ps);
    if (XT_PS_EXCM(cpu->ps) && XTENSA_EXCM_LEVEL > eff_level)
        eff_level = XTENSA_EXCM_LEVEL;

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

    /* In architectural-vector mode the guest's interrupt prologue executes
     * SPILL_ALL_WINDOWS itself. Pre-flushing here would make the legacy
     * synthetic spill algorithm race the guest's real overflow handlers and
     * can manufacture save-area links that never existed on hardware. Keep
     * the host-side flush only for the explicit legacy fallback. */
    if (!cpu->real_window_vectors)
        xtensa_flush_windows(cpu);
    /* Counts every vectored interrupt. The JIT verifier samples it across a
     * reference run: a native block defers interrupts to its exit while
     * xtensa_step() checks after every instruction, so a replay that vectors
     * covers different ground and cannot be compared. Watching PS is not
     * enough -- a handler clears EXCM to allow nesting, so by the end of the
     * replay the evidence is gone. */
    g_xtensa_irq_dispatched++;

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
/* Resolve a window slot's callsize: prefer the per-slot value recorded at
 * ENTRY (a window's callsize is a property of its creation call, not of
 * its a0 — tail-called frames can have a0=0); fall back to a0's top bits,
 * then to call8 (the IDF/GCC default). */
static int window_callsize_of(const xtensa_cpu_t *cpu, int widx) {
    int cs = cpu->window_callsize[widx & 0xF];
    if (cs == 0) cs = (phys_read(cpu, widx, 0) >> 30) & 3;
    if (cs == 0) cs = 2;
    return cs;
}

/* Find the callee window for widx: the window widx called. The live call
 * chain is uniquely determined by each window's creation callsize (the
 * distance to its caller), so walk backward from the current windowbase:
 * the first window whose backward step lands on widx is its callee. This
 * is exact for mixed call4/call8/call12 chains — a naive "smallest d with
 * matching callsize" search misfires on intermediate slots and picks the
 * wrong callee, skewing the spill area one frame away from the fill. */
static int find_callee_window(xtensa_cpu_t *cpu, int widx) {
    int w = cpu->windowbase;
    for (int steps = 0; steps < 16; steps++) {
        int caller = (w - window_callsize_of(cpu, w)) & 0xF;
        if (caller == widx) return w;
        w = caller;
        if (w == (int)cpu->windowbase) break;
    }
    /* Fallback: next valid window up the ring */
    for (int i = 1; i < 16; i++) {
        int c = (widx + i) & 0xF;
        if (cpu->windowstart & (1u << c))
            return c;
    }
    return widx;
}

/*
 * Spill (save) registers of window widx to its callee's stack frame.
 * Uses the callee's SP as base pointer (matching hardware overflow convention)
 * and records the base in spill_base[] for underflow restore.
 */
static void synth_spill_window(xtensa_cpu_t *cpu, int widx) {
    /* The callee's SP is the base pointer (hardware overflow convention),
     * and the callee's creation callsize gates how many of the spilled
     * window's registers go to the stack (call4: a0-a3, call8: +a4-a7,
     * call12: +a8-a11) — NOT the spilled window's own callsize. */
    int callee = find_callee_window(cpu, widx);
    int callsize = window_callsize_of(cpu, callee);
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

    /* Push onto spill stack for correct underflow restore.
     *
     * Pointless once fills go to the guest's own vector: nothing consumes
     * these, and they fill the 32-deep stack within 1.5M cycles (openHASP
     * emitted 27,000 "depth exceeds limit" warnings that way). */
    if (!(cpu->real_window_vectors && !g_flexe_shadow_fill &&
          window_vectors_ready(cpu, VECOFS_WINDOW_UNDERFLOW4)))
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
    /* Save to the exact ABI locations used by the ESP32 overflow vectors.
     * a0-a3 live immediately below the callee SP.  For call8/call12, the
     * remaining registers do NOT live below that same SP: the vector follows
     * the caller's base-save-area link at [caller_sp-12], then stores them
     * below that linked top.  Treating them as [callee_sp-32/-48] overlaps a
     * solicited FreeRTOS yield frame and corrupts suspended callers. */
    for (int i = 0; i < 4; i++)
        mem_write32(cpu->mem, base - 16 + i * 4, phys_read(cpu, widx, i));
    if (callsize >= 2) {
        uint32_t caller_sp = phys_read(cpu, widx, 1);
        uint32_t extra_top = mem_read32(cpu->mem, caller_sp - 12);
        for (int i = 0; i < 4; i++)
            mem_write32(cpu->mem, extra_top - (callsize == 3 ? 48 : 32) + i * 4,
                        phys_read(cpu, widx, 4 + i));
        if (callsize == 3)
            for (int i = 0; i < 4; i++)
                mem_write32(cpu->mem, extra_top - 32 + i * 4,
                            phys_read(cpu, widx, 8 + i));
    }
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
    }
    if (callsize == 3) {
        nregs = 12;
    }
    /* Always save a4-a11 to the CPU-side buffer for every spill, regardless
     * of callsize (not stack: on real hardware these go to grandparent/outer
     * frames, but computing those requires a chain of stack links; the CPU
     * buffer avoids corruption when deeper calls overwrite stack).
     * Saving unconditionally matters: a window's callsize as seen at fill
     * time (from the restored a0) can differ from what it was at spill time,
     * and a fill that finds no saved extras would restore stale garbage. */
    {
        int si2 = widx & 0xF;
        int d2 = cpu->spill_stack[si2].depth - 1; /* depth was already incremented */
        if (d2 >= 0 && d2 < SPILL_STACK_DEPTH) {
            for (int i = 0; i < 8; i++)
                cpu->spill_stack[si2].extra[d2][i] = phys_read(cpu, widx, 4 + i);
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
    window_hazard_refresh(cpu);
    WINLOG(cpu, "SPILL w%d callee=w%d base=%08X a0=%08X a1=%08X a2=%08X a3=%08X\n",
           widx, callee, base, phys_read(cpu, widx, 0), phys_read(cpu, widx, 1),
           phys_read(cpu, widx, 2), phys_read(cpu, widx, 3));
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
 *
 * Source selection: the hardware underflow handler fills from the stack at
 * [callee_sp-16] — per-context memory that is always task-correct. flexe's
 * CPU-side spill buffer is only authoritative when its recorded base matches
 * the current callee SP (same execution context, e.g. MOVSP between spill
 * and fill); a LIFO top entry from a *different* context (interrupts, task
 * switches) must never be used, so buffer entries are matched by base rather
 * than popped blindly.
 */
/* An underflow fill that restores a nonzero a0 with bits 31:30 clear has
 * restored something that is not a saved register window: every a0 the
 * hardware writes encodes the call size in those bits, and the only legitimate
 * exception is the zero a FreeRTOS task's bottom frame carries.
 *
 * Worth reporting because the failure is otherwise silent and arbitrarily far
 * from its cause. Tasmota 15.6.0 hangs on exactly this: one fill takes its base
 * from 0x3FFD2D40 -- a *different task's* stack -- restores a0 = 8 into
 * window 14, and the frame it hands back has an uninitialised local that the
 * firmware then dereferences as a pointer, 136 million times.
 *
 * Off unless FLEXE_FILLDBG is set, and the getenv is resolved once: this sits
 * on the underflow path, which legitimate task-entry frames also take. */
/* A fill that restores a0 = 0 is exempted by the check below, because a
 * FreeRTOS task's bottom frame really does hold zero. That exemption hides
 * the failure both remaining corpus images end on: a save area nothing ever
 * wrote reads back as zeroes, the fill accepts it as a bottom frame, and the
 * next RETW computes a return address of (pc & 0xC0000000) | 0 -- 0x40000000,
 * which is unmapped ROM. WLED spends 217 million instructions there.
 *
 * A genuine bottom frame is filled once, at task start, and never returned
 * through. Anything else reported here is a chain that has unwound one frame
 * too far. Separate from FLEXE_FILLDBG so it can be turned on alone; the two
 * ask different questions. */
static __attribute__((noinline, cold))
void report_zero_fill(xtensa_cpu_t *cpu, int ret_wb, uint32_t base, int owb) {
    static int on = -1, shown;
    if (on < 0) on = getenv("FLEXE_ZEROFILL") != NULL;
    if (!on || shown >= 32) return;
    shown++;
    fprintf(stderr, "[ZFILL] a0=0 restored to wb=%d (from wb=%d) a1=%08X "
            "base=%08X retw@%08X core%d\n", ret_wb, owb,
            phys_read(cpu, ret_wb, 1), base, cpu->pc, cpu->core_id);
}

static __attribute__((noinline, cold))
void report_impossible_fill(xtensa_cpu_t *cpu, int ret_wb, uint32_t base,
                            int use_rec, int owb, int callsize) {
    static int enabled = -1, shown;
    if (enabled < 0) enabled = getenv("FLEXE_FILLDBG") != NULL;
    if (!enabled || shown >= 32) return;
    shown++;
    fprintf(stderr,
            "[FILL] impossible a0=%08X restored to wb=%d (from wb=%d, "
            "callsize=%d) a1=%08X base=%08X record=%s retw@%08X core%d\n",
            phys_read(cpu, ret_wb, 0), ret_wb, owb, callsize,
            phys_read(cpu, ret_wb, 1), base, use_rec ? "yes" : "no",
            cpu->pc, cpu->core_id);
    /* The spill records for the same window slot, newest first. When none of
     * them carries the base the fill is looking for, this window was last
     * saved for a different frame -- which is the shape of the failure, not
     * a corrupted copy of the right one. */
    const typeof(cpu->spill_stack[0]) *ss = &cpu->spill_stack[ret_wb & 0xF];
    fprintf(stderr, "       ws=%04X  slot[%d] holds %d spill(s):",
            cpu->windowstart, ret_wb & 0xF, ss->depth);
    for (int d = ss->depth - 1; d >= 0 && d >= ss->depth - 6; d--)
        fprintf(stderr, " base=%08X", ss->base[d]);
    if (ss->depth == 0) fprintf(stderr, " none");
    fprintf(stderr, "\n");
}

/* Does a spill record describe the frame this fill is about to restore?
 * If so the data is in Flexe, not on the guest's stack, and is immune to the
 * stack having been reused since -- which is how the flush's spills get lost. */
static bool have_spill_record(const xtensa_cpu_t *cpu, uint32_t base) {
    for (int s = 0; s < 16; s++)
        for (int d = cpu->spill_stack[s].depth - 1; d >= 0; d--)
            if (cpu->spill_stack[s].base[d] == base) return true;
    return false;
}

static bool try_retw_underflow_exception(xtensa_cpu_t *cpu, uint32_t a0,
                                         int owb, int ret_wb,
                                         uint32_t fault_pc) {
    uint32_t vecofs = window_vec_offset(a0, true);
    if (!cpu->real_window_vectors || !XT_PS_WOE(cpu->ps) ||
        XT_PS_EXCM(cpu->ps) ||
        (g_flexe_shadow_fill &&
         have_spill_record(cpu, phys_read(cpu, owb, 1))) ||
        !window_vectors_ready(cpu, vecofs))
        return false;

    raise_window_exception(cpu, fault_pc, ret_wb, vecofs);
    return true;
}

bool xtensa_try_retw_underflow_exception(xtensa_cpu_t *cpu,
                                         uint32_t fault_pc) {
    if (!cpu) return false;
    uint32_t a0 = ar_read(cpu, 0);
    int n = (int)(a0 >> 30) & 3;
    if (n == 0) {
        n = cpu->window_callsize[cpu->windowbase & 15u];
        if (n == 0) n = 4;
    }
    int owb = (int)cpu->windowbase;
    int ret_wb = (owb - n) & 15;
    if (cpu->windowstart & (1u << ret_wb))
        return false;
    return try_retw_underflow_exception(cpu, a0, owb, ret_wb, fault_pc);
}

static void synth_underflow_fill(xtensa_cpu_t *cpu, int ret_wb, int owb, int callsize) {
    uint32_t base = phys_read(cpu, owb, 1);  /* callee SP (hardware convention) */

    /* Find the innermost spill record whose base matches the current callee
     * SP. Records are pushed per physical window slot, but a logical window's
     * slot shifts when windowbase changes across exception/dispatch paths, so
     * the match must search every slot's stack by base (the base is derived
     * from the window's own a1 chain and is stable across slot moves).
     * Entries above the match (if any) are stale leftovers and are discarded. */
    int m_si = -1, m_d = -1;
    for (int s = 0; s < 16 && m_d < 0; s++) {
        for (int d = cpu->spill_stack[s].depth - 1; d >= 0; d--) {
            if (cpu->spill_stack[s].base[d] == base) { m_si = s; m_d = d; break; }
        }
    }
    if (m_si >= 0) {
        cpu->spill_stack[m_si].depth = m_d;  /* consume match + stale above */
    }

    /* Restore the window. Prefer the matched CPU-side spill record over the
     * stack: the record is written once at spill time and never clobbered,
     * whereas the stack spill areas ALIAS when an intermediate frame is only
     * 16 bytes — a deeper window's a4-a7 at [callee_sp-32] then land on the
     * shallower window's a0-a3 at [callee_sp-16] (two frames 16 bytes apart),
     * so the stack copy of a0-a3 can hold a neighbour's a4-a7 (seen in
     * NerdMiner as a retw jumping to a data pointer). The record is matched
     * by callee-SP base and consumed in LIFO order, so it is always this
     * context's own most recent spill; fall back to the stack when no record
     * matches. The gating callsize is the rotation n from exec_retw, derived
     * from the returning window's a0 — always valid at retw (retw itself
     * needs a0 for the PC) and always local to this context, unlike the
     * per-slot window_callsize[] ENTRY records which a context switch leaves
     * holding a foreign task's values. */
    int use_rec = (m_si >= 0);
    for (int i = 0; i < 4; i++)
        phys_write(cpu, ret_wb, i, use_rec ? cpu->spill_stack[m_si].core[m_d][i]
                                           : mem_read32(cpu->mem, base - 16 + i * 4));

    /* Self-heal the per-slot callsize record from the just-restored a0:
     * after a context switch the ENTRY-era record belongs to a foreign
     * context, but a0's top bits always encode this window's true creation
     * callsize, refreshing the record for later callee-resolution walks. */
    {
        int healed = (int)(phys_read(cpu, ret_wb, 0) >> 30);
        if (healed)
            cpu->window_callsize[ret_wb & 0xF] = (uint8_t)healed;
        else if (phys_read(cpu, ret_wb, 0) != 0)
            report_impossible_fill(cpu, ret_wb, base, use_rec, owb, callsize);
        else
            report_zero_fill(cpu, ret_wb, base, owb);
        /* a0 == 0 is not reported: a FreeRTOS task's bottom frame really does
         * hold zero, so it is the one legitimate way for the top bits to be
         * clear. Any *other* value with bits 31:30 clear cannot be a return
         * address the hardware produced, and means this fill restored
         * something that was never a saved window. */
    }

    if (callsize >= 2) {
        uint32_t caller_sp = phys_read(cpu, ret_wb, 1);
        uint32_t extra_top = mem_read32(cpu->mem, caller_sp - 12);
        for (int i = 0; i < 4; i++)
            phys_write(cpu, ret_wb, 4 + i, use_rec ? cpu->spill_stack[m_si].extra[m_d][i]
                                                   : mem_read32(cpu->mem,
                                                       extra_top - (callsize == 3 ? 48 : 32) + i * 4));
        if (callsize == 3)
            for (int i = 0; i < 4; i++)
                phys_write(cpu, ret_wb, 8 + i,
                           use_rec ? cpu->spill_stack[m_si].extra[m_d][4 + i]
                                   : mem_read32(cpu->mem,
                                                extra_top - 32 + i * 4));
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
        uint32_t a0 = phys_read(cpu, ret_wb, 0);
        int callsize = (a0 >> 30) & 3;
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
    window_hazard_refresh(cpu);
    WINLOG(cpu, "FILL w%d base=%08X match=%d/%d a0=%08X a1=%08X a2=%08X a3=%08X\n",
           ret_wb, base, m_si, m_d, phys_read(cpu, ret_wb, 0),
           phys_read(cpu, ret_wb, 1), phys_read(cpu, ret_wb, 2),
           phys_read(cpu, ret_wb, 3));
}

/*
 * RETW: shared helper for both RETW (24-bit) and RETW.N (16-bit).
 * ISA: n = AR[0][31:30], nextPC = PC[31:30] | AR[0][29:0]
 *   if WS[WB-n] set → normal: clear WS[owb], WB -= n
 *   if WS[WB-n] clear → underflow fill, then WB -= n
 */
static void exec_retw(xtensa_cpu_t *cpu, int retw_len) {
    uint32_t a0 = ar_read(cpu, 0);
    int n = (a0 >> 30) & 3;
    if (n == 0) {
        /* a0's top bits are 0 — tail-called/entry-created window where a0
         * does not encode the creation callsize; use the value tracked at
         * ENTRY. Hardware rotates by a0's bits (0) here, but a live frame
         * that is actually returned through always has a tracked callsize. */
        n = cpu->window_callsize[cpu->windowbase];
        if (n == 0) n = 4;
    }

    uint32_t next_pc = (cpu->pc & 0xC0000000) | (a0 & 0x3FFFFFFF);

    int owb = cpu->windowbase;
    int ret_wb = (owb - n) & 0xF;

    bool need_fill = !(cpu->windowstart & (1u << ret_wb));
    if (cpu->window_trace && need_fill) {
        fprintf(stderr, "     [WIN] RETW wb=%d->%d (n=%d) UNDERFLOW WS=0x%04X\n",
                owb, ret_wb, n, cpu->windowstart);
    }
    WINLOG(cpu, "RETW n=%d owb=%d ret_wb=%d a0=%08X a1=%08X fill=%d\n",
           n, owb, ret_wb, a0, ar_read(cpu, 1), (int)need_fill);

    if (need_fill &&
        __builtin_expect(try_retw_underflow_exception(
                             cpu, a0, owb, ret_wb,
                             cpu->pc - (uint32_t)retw_len), 0)) {
        /* The guest's WindowUnderflow handler reads the frame back from the
         * stack the ABI put it on; RFWU sets the bit and returns to this
         * RETW, which then takes the normal path. cpu->pc is already past the
         * instruction, so back it up by the length the caller decoded. */
        return;
    }

    if (need_fill) {
        /* Caller's window was spilled — fill it back. n (from the returning
         * window's a0) is the fill width, matching the hardware underflow
         * vector selection and immune to cross-context metadata staleness. */
        synth_underflow_fill(cpu, ret_wb, owb, n);
    }

    /* Clear current window's WS bit */
    cpu->windowstart &= ~(1u << owb);

    /* Rotate back */
    cpu->windowbase = ret_wb;
    window_hazard_refresh(cpu);

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
    case 6: /* MADDN.S: fused multiply-add, forced round-to-nearest */
        cpu->fr[r] = fmaf(cpu->fr[s], cpu->fr[t], cpu->fr[r]);
        break;
    case 7: /* DIVN.S: final marker in the collapsed divide sequence */
        break;
    case 8: { /* ROUND.S: ar[r] = (int32_t)roundf(fr[s] * 2^t) */
        float val = cpu->fr[s];
        if (t) val = val * (float)(1u << t);
        ar_write(cpu, r, (uint32_t)(int32_t)roundf(val));
    } break;
    case 9: { /* TRUNC.S: ar[r] = (int32_t)truncf(fr[s] * 2^t) */
        float val = cpu->fr[s];
        if (t) val = val * (float)(1u << t);
        ar_write(cpu, r, (uint32_t)(int32_t)truncf(val));
    } break;
    case 10: { /* FLOOR.S: ar[r] = (int32_t)floorf(fr[s] * 2^t) */
        float val = cpu->fr[s];
        if (t) val = val * (float)(1u << t);
        ar_write(cpu, r, (uint32_t)(int32_t)floorf(val));
    } break;
    case 11: { /* CEIL.S: ar[r] = (int32_t)ceilf(fr[s] * 2^t) */
        float val = cpu->fr[s];
        if (t) val = val * (float)(1u << t);
        ar_write(cpu, r, (uint32_t)(int32_t)ceilf(val));
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
    case 14: { /* UTRUNC.S: ar[r] = (uint32_t)(fr[s] * 2^t) */
        float val = cpu->fr[s];
        if (t) val = val * (float)(1u << t);
        ar_write(cpu, r, (uint32_t)val);
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
        /*
         * Xtensa emits DIV.S and SQRT.S as prescribed helper-instruction
         * sequences.  Preserve their architectural result using the same
         * sequence collapse used by QEMU: MK* computes the exact operation,
         * ADDEXPM forwards it, and the trailing DIVN is a no-op.  This also
         * handles IEEE-754 signs, infinities, NaNs and subnormals through the
         * host single-precision operation instead of exposing the opaque
         * intermediate exponent-adjust encoding.
         */
        case 12: /* MKSADJ.S: materialize the square-root result */
            cpu->fr[r] = sqrtf(cpu->fr[s]);
            break;
        case 13: /* MKDADJ.S: materialize dividend / saved divisor */
            cpu->fr[r] = cpu->fr[s] / cpu->fr[r];
            break;
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
        case 15: /* ADDEXPM.S: forward collapsed DIV.S/SQRT.S result */
            cpu->fr[r] = cpu->fr[s];
            break;
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
static inline __attribute__((always_inline))
void exec_qrst(xtensa_cpu_t *cpu, uint32_t insn) {
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
                        exec_retw(cpu, 3);
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
                /* MOVSP does not move a stack frame.  With the architectural
                 * window handlers enabled it first raises AllocaCause when
                 * all three possible caller windows are spilled.  The
                 * guest's _xt_alloca_exc fills the caller whose save-area
                 * links would otherwise become unreachable, then RFWU
                 * retries this instruction.  Once any caller window is live,
                 * MOVSP is only the register-to-register assignment below.
                 *
                 * The synthesized fallback predates real window exceptions
                 * and retains its private spill bookkeeping.  In particular,
                 * never run its 48-byte stack copy in architectural mode:
                 * restoring an alloca'd SP that way overwrites [sp-12], the
                 * back-chain used by WindowUnderflow8/12. */
                if (__builtin_expect(cpu->real_window_vectors, 0)) {
                    if (XT_PS_WOE(cpu->ps) && !XT_PS_EXCM(cpu->ps)) {
                        uint32_t callers =
                            (1u << ((cpu->windowbase - 1u) & 15u)) |
                            (1u << ((cpu->windowbase - 2u) & 15u)) |
                            (1u << ((cpu->windowbase - 3u) & 15u));
                        if ((cpu->windowstart & callers) == 0u) {
                            xtensa_raise_exception(cpu, EXCCAUSE_ALLOCA,
                                                   cpu->pc - 3u, 0);
                            return;
                        }
                    }
                    ar_write(cpu, t, ar_read(cpu, s));
                    break;
                }

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
                        for (int j = 0; j < cpu->spill_stack[i].depth &&
                                            j < SPILL_STACK_DEPTH; j++) {
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
                        WINLOG(cpu, "RFE epc=%08X ps=%08X a1=%08X\n",
                               cpu->epc[0], cpu->ps, ar_read(cpu, 1));
                        XT_PS_SET_EXCM(cpu->ps, 0);
                        xtensa_request_irq_check(cpu); /* may unmask; see WSR PS */
                        BRANCH_TO(cpu, cpu->epc[0]);
                        return;
                    case 4: /* RFWO */
                        WINLOG(cpu, "RFWO epc=%08X ps=%08X\n", cpu->epc[0], cpu->ps);
                        XT_PS_SET_EXCM(cpu->ps, 0);
                        xtensa_request_irq_check(cpu); /* may unmask; see WSR PS */
                        cpu->windowstart &= ~(1u << cpu->windowbase);
                        cpu->windowbase = XT_PS_OWB(cpu->ps);
                        window_hazard_refresh(cpu);
                        BRANCH_TO(cpu, cpu->epc[0]);
                        return;
                    case 5: /* RFWU */
                        WINLOG(cpu, "RFWU epc=%08X ps=%08X\n", cpu->epc[0], cpu->ps);
                        XT_PS_SET_EXCM(cpu->ps, 0);
                        xtensa_request_irq_check(cpu); /* may unmask; see WSR PS */
                        cpu->windowstart |= (1u << cpu->windowbase);
                        cpu->windowbase = XT_PS_OWB(cpu->ps);
                        window_hazard_refresh(cpu);
                        BRANCH_TO(cpu, cpu->epc[0]);
                        return;
                    default: break;
                    }
                    break;
                case 1: /* RFI */
                    if (s >= 1 && s <= 7) {
                        cpu->ps = cpu->eps[s - 1];
                        /* Deliberately no irq_check here, unlike every
                         * other PS write. Hardware does re-evaluate at an
                         * interrupt return, but our peripherals deassert a
                         * level-triggered source a little later than the
                         * handler acknowledges it, so re-checking on the
                         * spot re-enters the same handler and the guest
                         * makes no progress -- NerdMiner stops finding
                         * shares. The interrupt is not lost: the next
                         * assert, timer tick or PS write picks it up. */
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
                xtensa_request_irq_check(cpu); /* may unmask; see WSR PS */
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
                /* xthal_window_spill walks the window file with
                 * `and a12,a12,a12; rotw 3`. The register touch is the point:
                 * on hardware, reading a12 with WS[WB+1..WB+3] set raises
                 * WindowOverflow, and the handler writes that frame to the
                 * stack. Flexe synthesizes spills at ENTRY and does not do the
                 * per-instruction window check, so the walk used to rotate and
                 * write nothing -- and setjmp, which reads its caller's
                 * spilled a0-a3 straight back from [sp-16], saved stale words.
                 *
                 * Do the check the touch would have done, with the window set
                 * the ISA specifies for an a12-a15 access: WB+1 through WB+3,
                 * and no wider. (The same routine reached through SYSCALL is
                 * handled a few cases above.) */
                /* Architectural mode performs the register-access window
                 * check before execution.  ROTW itself only changes WB.  In
                 * particular, guest overflow/alloca handlers execute ROTW
                 * with PS.EXCM set and must not have their WINDOWSTART bits
                 * silently spilled by the synthesized fallback. */
                if (!cpu->real_window_vectors && XT_PS_WOE(cpu->ps))
                    synth_overflow_check(cpu, 0);
                cpu->windowbase = (cpu->windowbase + (int32_t)sign_extend(t, 4)) & 0xF;
                window_hazard_refresh(cpu);
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
                ar_write(cpu, r, 0u - ar_read(cpu, t));
                break;
            case 1: /* ABS */
                { uint32_t val = ar_read(cpu, t);
                  ar_write(cpu, r, (val & 0x80000000u) ? 0u - val : val);
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
        case 14: /* RUR: r=destination, s/t=user-register number */
            { int ur = (s << 4) | t;
              uint32_t value;
              switch (ur) {
              case XT_UR_EXPSTATE:  value = cpu->expstate; break;
              case XT_UR_THREADPTR: value = cpu->threadptr; break;
              case XT_UR_FCR:       value = cpu->fcr; break;
              case XT_UR_FSR:       value = cpu->fsr; break;
              case XT_UR_F64R_LO:   value = cpu->f64r_lo; break;
              case XT_UR_F64R_HI:   value = cpu->f64r_hi; break;
              case XT_UR_F64S:      value = cpu->f64s; break;
              default:              value = 0; break;
              }
              ar_write(cpu, r, value);
            } break;
        case 15: /* WUR: t=source, r/s=user-register number */
            { int ur = (r << 4) | s;
              uint32_t value = ar_read(cpu, t);
              switch (ur) {
              case XT_UR_EXPSTATE:  cpu->expstate = value; break;
              case XT_UR_THREADPTR: cpu->threadptr = value; break;
              case XT_UR_FCR:       cpu->fcr = value; break;
              case XT_UR_FSR:       cpu->fsr = value; break;
              case XT_UR_F64R_LO:   cpu->f64r_lo = value; break;
              case XT_UR_F64R_HI:   cpu->f64r_hi = value; break;
              case XT_UR_F64S:      cpu->f64s = value; break;
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
static inline __attribute__((always_inline))
void exec_lsai(xtensa_cpu_t *cpu, uint32_t insn) {
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
          if (__builtin_expect(g_dbg_c1ilog && old != XTENSA_SPINLOCK_FREE
                  && old != XTENSA_SPINLOCK_OWNER_CORE0
                  && old != XTENSA_SPINLOCK_OWNER_CORE1
                  && addr < 0x40000000u, 0)) {
              fprintf(stderr, "[C1I] garbage read pc=0x%08X addr=0x%08X old=0x%08X core%d\n",
                      cpu->pc, addr, old, cpu->core_id);
          }
          if (old == cpu->scompare1)
              mem_write32(cpu->mem, addr, ar_read(cpu, t));
          else if (__builtin_expect(xtensa_s32c1i_needs_handoff(cpu, old), 0))
              cpu->core_handoff = true;
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
          ar_write(cpu, t, ar_read(cpu, s) + (uint32_t)(simm8 * 256));
        } break;

    default:
        break;
    }
}

/* Execute narrow (16-bit) instructions */
static inline __attribute__((always_inline))
/* exec_narrow has external linkage (used by tests). The compiler will
 * still inline it into xtensa_step_impl thanks to LTO + the call site
 * being in the same TU. */
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
                exec_retw(cpu, 2);
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
static inline __attribute__((always_inline))
void exec_calln(xtensa_cpu_t *cpu, uint32_t insn) {
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
static inline __attribute__((always_inline))
bool exec_si(xtensa_cpu_t *cpu, uint32_t insn) {
    int nn = XT_N(insn);
    int m = XT_M(insn);
    int s = XT_S(insn);
    bool poll_spin = false;

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
          case 0: /* BEQZ */
              if (val == 0) {
                  BRANCH_TO(cpu, target);
                  if (__builtin_expect(cpu->poll_spin_count != 0u, 0)) {
                      for (unsigned i = 0; i < cpu->poll_spin_count; i++) {
                          if (cpu->poll_spin_pc[i] != target)
                              continue;
                          /* Structural discovery proves this loop only reads
                           * through a9. Mapped memory is stable until the
                           * deterministic scheduler runs the peer core;
                           * MMIO/unmapped reads retain cycle-by-cycle
                           * execution because a device may change itself. */
                          poll_spin = mem_get_ptr(
                                  cpu->mem, ar_read(cpu, 9)) != NULL;
                          break;
                      }
                  }
              }
              break;
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
              int new_reg = (callinc << 2) | (s & 3);
              uint32_t caller_sp = ar_read(cpu, s);
              uint32_t new_sp = caller_sp - frame_size;

              /* The ISA ENTRY check is wb+1 through wb+callinc -- every
               * window the rotation passes over, not just the one it lands on.
               * Getting that wrong is undetectable afterwards: a live frame
               * rotated over is simply gone, and no later access can fault on
               * it. The windows the *callee* will reach through a4-a15 are
               * window_access_check()'s job on the instructions that follow. */
              bool ent_vec = __builtin_expect(cpu->real_window_vectors, 0) &&
                             XT_PS_WOE(cpu->ps) && !XT_PS_EXCM(cpu->ps) &&
                             window_vectors_ready(cpu,
                                                  VECOFS_WINDOW_OVERFLOW4);
              if (ent_vec) {
                  if (xtensa_try_entry_overflow_exception(
                          cpu, cpu->pc - 3u)) {
                      /* ENTRY is restartable.  In particular, its destination
                       * a(4*CALLINC+s) is part of the window that just
                       * collided.  Writing the new SP before taking the
                       * exception destroys that live frame before the guest's
                       * overflow vector can save it. */
                      return false;
                  }
              } else {
                  /* The legacy synthesized spill derives a colliding frame's
                   * save area from the callee SP, so retain its historical
                   * ordering.  Architectural-vector mode above must have no
                   * instruction side effects until all checks pass. */
                  ar_write(cpu, new_reg, new_sp);
                  synth_overflow_check(cpu, callinc);
              }

              if (ent_vec)
                  ar_write(cpu, new_reg, new_sp);

              /* First frame on this core: stand in for the bootloader that
               * would have called us, so this frame can be spilled.  This is
               * an ENTRY side effect too, and therefore belongs after its
               * architectural overflow check. */
              if (__builtin_expect(cpu->seed_entry_link, 0)) {
                  cpu->seed_entry_link = false;
                  if (new_sp >= 0x3FF80000u && new_sp < 0x40000000u &&
                      mem_read32(cpu->mem, new_sp - 12u) == 0u) {
                      mem_write32(cpu->mem, new_sp - 16u, 0u);  /* ends unwind */
                      mem_write32(cpu->mem, new_sp - 12u, caller_sp);
                  }
              }

              uint32_t owb = cpu->windowbase;
              cpu->windowbase = (owb + callinc) & 0xF;
              cpu->windowstart |= (1u << cpu->windowbase);
              window_hazard_refresh(cpu);
              cpu->window_callsize[cpu->windowbase] = (uint8_t)callinc;
              XT_PS_SET_OWB(cpu->ps, owb);
              /* ENTRY must NOT clear PS.CALLINC. Only CALLn/CALLXn write it,
               * and it stays put until the next one -- which is the whole
               * mechanism behind xthal_window_spill: one `call12` followed by
               * a run of bare `entry a1,48; mov.n a12,a0` pairs, each ENTRY
               * rotating another three windows and each a12 touch overflowing
               * the frame it exposes, until the register file has been walked.
               *
               * Clearing it collapsed that walk to its first rotation. On
               * Tasmota the caller's window was left unspilled, so the setjmp
               * that runs immediately afterwards read stale words back from
               * [sp-16] and saved them; the matching longjmp then restored
               * a0 = 8 into a live frame. FLEXE_FILLDBG names that fill. */
              WINLOG(cpu, "ENTRY callinc=%d owb=%d nwb=%d a1=%08X\n",
                     callinc, owb, cpu->windowbase, ar_read(cpu, 1));

              /* ENTRY changes the physical register window underneath the
               * logical a0-a15 names. Give the accelerator a private dispatch
               * boundary at the callee body, which is keyed by the new
               * windowbase. This is deliberately distinct from _pc_written:
               * the fallthrough is not a guest control-flow edge and must not
               * invoke firmware observers registered at the post-ENTRY PC. */
              if (__builtin_expect(cpu->record_branch_targets, 0))
                  cpu->jit_fallthrough_dispatch = true;
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
                  /* LOOP is an interpreter-only context setter. Once it has
                   * entered the body, give an already-compiled LBEG block a
                   * chance to run on the first iteration instead of waiting
                   * for the first architectural back-edge. A skipped
                   * LOOPNEZ/LOOPGTZ wrote PC and already has a real dispatch
                   * boundary, so it must not acquire the private marker. */
                  if (__builtin_expect(cpu->record_branch_targets, 0) &&
                      !cpu->_pc_written)
                      cpu->jit_fallthrough_dispatch = true;
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
    return poll_spin;
}

/* Execute op0=7 (B) - RRI8 conditional branches */
static inline __attribute__((always_inline))
void exec_b(xtensa_cpu_t *cpu, uint32_t insn) {
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

static inline __attribute__((always_inline))
void exec_mac16(xtensa_cpu_t *cpu, uint32_t insn) {
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
/* local_cc accumulates cycle_count in a register and is flushed to the CPU
 * only before callbacks that may read it. prev_pc likewise keeps fatal-trap
 * history out of the per-instruction CPU-state write set. */

static __attribute__((noinline, cold))
void xtensa_invalid_pc_trap(xtensa_cpu_t *cpu) {
    fprintf(stderr, "[TRAP] Invalid PC=0x%08X at cycle %llu (core %d, prid=0x%X) prev_pc=0x%08X\n",
            cpu->pc, (unsigned long long)cpu->cycle_count, cpu->core_id, cpu->prid,
            cpu->dbg_prev_pc);
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
}

static inline bool xtensa_pc_is_valid(const xtensa_cpu_t *cpu, uint32_t pc) {
    return flexe_target_pc_is_executable(cpu->target, pc);
}

static __attribute__((noinline, cold))
int xtensa_invalid_pc_step(xtensa_cpu_t *cpu, uint64_t local_cc,
                           uint32_t last_pc) {
    cpu->cycle_count = local_cc;
    cpu->dbg_prev_pc = last_pc;
    xtensa_invalid_pc_trap(cpu);
    return -1;
}

/* Optional runtime diagnostics. */
uint32_t g_dbg_pc;
int g_dbg_core;
int g_dbg_watch_en = 1;
int g_dbg_pcwatch_en = 1;
uint32_t g_dbg_pcwatch;
uint32_t g_dbg_pcwatch2;
uint32_t g_dbg_watch_addr;
uint32_t g_dbg_watch_addr2;
uint32_t g_dbg_watch_val;
/* PC-armed instruction trace: fires when PC hits arm address, then logs N insns */
#define DBG_RING_MAX 256

uint32_t g_dbg_tarm;
/* Optional register filters on the PC-armed trace, plus the control-transfer
 * ring. See xtensa_dbg_step_trace(). */
uint32_t g_dbg_tarm_a2, g_dbg_tarm_a3;
int      g_dbg_tarm_a2_en, g_dbg_tarm_a3_en;
int      g_dbg_ring_n;
int g_dbg_tn = 200;
int g_dbg_tcore = -1;
int g_dbg_tcount;          /* remaining insns in current fire */
int g_dbg_tfires = 3;      /* max fires */
int g_dbg_winlog;          /* FLEXE_WINLOG: window-ops trace */
int g_dbg_c1ilog;          /* FLEXE_C1ILOG: log garbage s32c1i reads */
/* Single gates for every per-instruction diagnostic below. None of them is
 * armed unless the matching FLEXE_* variable is set, but the checks themselves
 * used to run unconditionally: profiling put the arm-test loads and debug
 * state stores among the hottest instructions in the interpreter, which is
 * pure waste in every normal run. */
int g_dbg_step_trace;
static int g_dbg_step_slow;
int g_flexe_shadow_fill;   /* FLEXE_SHADOWFILL */

__attribute__((constructor))
static void g_dbg_watch_init(void) {
    const char *e = getenv("FLEXE_WATCH");
    if (e) g_dbg_watch_addr = (uint32_t)strtoul(e, NULL, 0);
    e = getenv("FLEXE_WATCH2");
    if (e) g_dbg_watch_addr2 = (uint32_t)strtoul(e, NULL, 0);
    if (!g_dbg_watch_addr && !g_dbg_watch_addr2) g_dbg_watch_en = 0;
    e = getenv("FLEXE_PCWATCH");
    if (e) g_dbg_pcwatch = (uint32_t)strtoul(e, NULL, 0);
    e = getenv("FLEXE_PCWATCH2");
    if (e) g_dbg_pcwatch2 = (uint32_t)strtoul(e, NULL, 0);
    if (!g_dbg_pcwatch && !g_dbg_pcwatch2) g_dbg_pcwatch_en = 0;
    e = getenv("FLEXE_TARM");
    if (e) g_dbg_tarm = (uint32_t)strtoul(e, NULL, 0);
    e = getenv("FLEXE_TN");
    if (e) g_dbg_tn = atoi(e);
    e = getenv("FLEXE_TARM_A2");
    if (e) { g_dbg_tarm_a2 = (uint32_t)strtoul(e, NULL, 0); g_dbg_tarm_a2_en = 1; }
    e = getenv("FLEXE_TARM_A3");
    if (e) { g_dbg_tarm_a3 = (uint32_t)strtoul(e, NULL, 0); g_dbg_tarm_a3_en = 1; }
    e = getenv("FLEXE_RING");
    if (e) {
        g_dbg_ring_n = atoi(e);
        if (g_dbg_ring_n > DBG_RING_MAX) g_dbg_ring_n = DBG_RING_MAX;
        if (g_dbg_ring_n < 0) g_dbg_ring_n = 0;
    }
    e = getenv("FLEXE_TCORE");
    if (e) g_dbg_tcore = atoi(e);
    e = getenv("FLEXE_TFIRES");
    if (e) g_dbg_tfires = atoi(e);
    {   /* Legacy diagnostic fallback only. The default restores through the
         * guest's ABI save areas and underflow vectors, like the hardware. */
        const char *sf = getenv("FLEXE_SHADOWFILL");
        g_flexe_shadow_fill = sf ? atoi(sf) != 0 : 0;
    }
    e = getenv("FLEXE_WINLOG");
    if (e) g_dbg_winlog = atoi(e);
    e = getenv("FLEXE_C1ILOG");
    if (e) g_dbg_c1ilog = atoi(e);
    e = getenv("FLEXE_WVDBG");
    g_dbg_wvlog = e != NULL;
#if FLEXE_PROFILE_BUILD
    xtensa_profile_init();
#endif
    e = getenv("FLEXE_WATCHVAL");
    if (e) g_dbg_watch_val = (uint32_t)strtoul(e, NULL, 0);
    /* g_dbg_core is read by the memory watch prints in memory.h as well as by
     * the instruction trace, so arm the gate for either. */
    g_dbg_mem_watch = (g_dbg_watch_en || g_dbg_pcwatch_en ||
                       g_dbg_watch_val) ? 1 : 0;
    mem_write32_observers_refresh();
    g_dbg_step_trace = (g_dbg_tarm || g_dbg_mem_watch) ? 1 : 0;
    /* g_dbg_pc is consumed only by the step trace, memory watchpoints, and
     * opt-in peripheral/SD logging. Do not publish it on every instruction in
     * normal runs: that cross-translation-unit store is measurable in the
     * interpreter's innermost loop. */
    g_dbg_step_slow = (g_dbg_step_trace || getenv("FLEXE_PERIPHDBG") ||
                       getenv("FLEXE_SDDBG")) ? 1 : 0;
}

/* Per-instruction diagnostics: the PC-armed instruction trace, and publishing
 * the current core for the memory watchpoints in memory.h. Out of line and
 * cold, so the hot path pays a single predictable test of g_dbg_step_slow. */
static __attribute__((noinline, cold))
void xtensa_dbg_step_trace(xtensa_cpu_t *cpu) {
    static uint32_t ring[DBG_RING_MAX];
    static unsigned ring_pos;
    static uint32_t ring_prev;

    g_dbg_core = cpu->core_id;
    /* FLEXE_RING: the last N control transfers, dumped when the trace fires.
     * Keeping only non-sequential PCs turns a few dozen slots into thousands
     * of instructions of context, which is what makes one dump enough to say
     * how the guest arrived somewhere -- through an interrupt vector, a tail
     * call, a scheduler switch. */
    if (g_dbg_ring_n > 0 &&
        (g_dbg_tcore < 0 || cpu->core_id == g_dbg_tcore)) {
        int32_t step = (int32_t)(cpu->pc - ring_prev);
        if (step < 0 || step > 8)
            ring[ring_pos++ % (unsigned)g_dbg_ring_n] = cpu->pc;
        ring_prev = cpu->pc;
    }

    if (g_dbg_tarm && cpu->pc == g_dbg_tarm && g_dbg_tfires > 0
        && (g_dbg_tcore < 0 || cpu->core_id == g_dbg_tcore)
        /* FLEXE_TARM_A2/A3: a hot address like vListInsert is reached by every
         * object in the system. Without a way to say "only this one" the trace
         * is unreadable. */
        && (!g_dbg_tarm_a2_en || ar_read(cpu, 2) == g_dbg_tarm_a2)
        && (!g_dbg_tarm_a3_en || ar_read(cpu, 3) == g_dbg_tarm_a3)) {
        g_dbg_tfires--;
        g_dbg_tcount = g_dbg_tn;
        fprintf(stderr, "[TARM] fire at pc=0x%08X core%d\n", cpu->pc, cpu->core_id);
        for (int i = 0; i < g_dbg_ring_n; i++) {
            uint32_t p = ring[(ring_pos + (unsigned)i) % (unsigned)g_dbg_ring_n];
            if (p) fprintf(stderr, "[RING] %3d pc=%08X\n", i, p);
        }
    }
    if (g_dbg_tcount > 0
        && (g_dbg_tcore < 0 || cpu->core_id == g_dbg_tcore)) {
        g_dbg_tcount--;
        fprintf(stderr,
                "[T%d] pc=0x%08X a0=%08X a1=%08X a2=%08X a3=%08X "
                "a4=%08X a5=%08X a6=%08X a7=%08X a8=%08X a9=%08X "
                "a10=%08X a11=%08X a12=%08X a13=%08X a14=%08X a15=%08X "
                "ps=%08X wb=%u ws=%08X\n",
                cpu->core_id, cpu->pc,
                ar_read(cpu, 0), ar_read(cpu, 1), ar_read(cpu, 2), ar_read(cpu, 3),
                ar_read(cpu, 4), ar_read(cpu, 5), ar_read(cpu, 6), ar_read(cpu, 7),
                ar_read(cpu, 8), ar_read(cpu, 9), ar_read(cpu, 10), ar_read(cpu, 11),
                ar_read(cpu, 12), ar_read(cpu, 13), ar_read(cpu, 14), ar_read(cpu, 15),
                cpu->ps, cpu->windowbase, cpu->windowstart);
    }
}

/* prev_pc carries the previous instruction's PC between iterations. It is a
 * caller-owned local rather than an xtensa_cpu_t field so it stays in a
 * register: the only consumer is the invalid-PC trap, which is fatal and cold,
 * and paying a store to the struct on every instruction to feed it showed up
 * in the profile. */
static inline __attribute__((always_inline))
int xtensa_step_impl(xtensa_cpu_t *cpu, uint64_t *restrict local_cc,
                     uint32_t *restrict prev_pc, uint32_t native_span_room) {
    uint32_t insn;
    const uint32_t last_pc = *prev_pc;
    *prev_pc = cpu->pc;
    if (__builtin_expect(g_dbg_step_slow, 0)) {
        g_dbg_pc = cpu->pc;
        if (g_dbg_step_trace)
            xtensa_dbg_step_trace(cpu);
    }
    if (__builtin_expect(cpu->halted, 0)) {
        cpu->ccount++;
        ++*local_cc;
        if (cpu->ccount >= cpu->next_timer_event)
            xtensa_fire_timers(cpu);
        uint32_t pending = cpu->interrupt & cpu->intenable;
        if (pending) {
            cpu->halted = false;
            xtensa_check_interrupts(cpu);
        }
        return cpu->exception ? -1 : 0;
    }

    /* PC hook: intercept execution at specific addresses (e.g. ROM stubs).
     * Hooks are always registered at function-entry PCs — reachable
     * only via call/ret/branches/exceptions — so we only need to
     * check after a control-flow transfer. `_pc_written` captures
     * that exact condition (set by the previous step's branch/call/
     * ret/exception handlers, cleared below at line 2073 unless the
     * instruction writes PC again). On straight-line execution the
     * bitmap load is skipped entirely.  Runs BEFORE the invalid-PC
     * trap so a hook at 0x00000000 can turn callxN-through-NULL into
     * a benign return-0 (symbol-less firmware driver tables).
     *
     * Some interpreter-only context setters need a private accelerator
     * boundary on their fallthrough. The transient marker lets jit_pc_hook
     * dispatch a compatible block without presenting a second call to
     * ordinary firmware observers at that same address. */
    const bool accelerator_fallthrough =
        __builtin_expect(cpu->jit_fallthrough_dispatch != 0, 0);
    const bool pc_written = cpu->_pc_written != 0;
    const bool dispatch_boundary = pc_written || accelerator_fallthrough;
    if (__builtin_expect(dispatch_boundary, 0)) {
        if (__builtin_expect(cpu->record_branch_targets, 0)) {
            cpu->br_ring[cpu->br_ring_idx & (XT_BR_RING_SIZE - 1)] = cpu->pc;
            cpu->br_ring_idx++;
        }
        /* Return from a guest_call_async() callee. Only reachable through the
         * return address that call planted, so the compare is enough. */
        if (pc_written &&
            __builtin_expect(cpu->pc == GUEST_CALL_ASYNC_SENTINEL, 0))
            guest_call_async_return(cpu);
    }
    if (dispatch_boundary && cpu->pc_hook && (!cpu->pc_hook_bitmap ||
        rom_stubs_hook_bitmap_test(cpu->pc_hook_bitmap, cpu->pc))) {
        cpu->cycle_count = *local_cc;  /* flush for stub visibility */
        /* Only hooks consume the scheduler-room hint. Publishing it around
         * the hook call instead of on every interpreted instruction removes
         * two hot-path stores while preserving the exact same hook contract. */
        cpu->native_span_room = native_span_room;
        int hook_insns = cpu->pc_hook(cpu, cpu->pc, cpu->pc_hook_ctx);
        cpu->native_span_room = 0u;
        if (hook_insns) {
            if (accelerator_fallthrough)
                cpu->jit_fallthrough_dispatch = false;
            /* Native hooks may account a whole block, while time-oriented
             * stubs may fast-forward cycle_count.  Pull that advancement
             * back into the cached counter before charging the dispatching
             * instruction; writing local_cc first used to discard it. */
            *local_cc = cpu->cycle_count;
            cpu->ccount++;
            ++*local_cc;
            cpu->cycle_count = *local_cc;
            /* Stubs may advance time, so check timers unconditionally */
            if (cpu->ccount >= cpu->next_timer_event)
                xtensa_fire_timers(cpu);
            if (__builtin_expect(cpu->irq_check, 0)) {
                cpu->irq_check = false;
                if (cpu->interrupt & cpu->intenable)
                    xtensa_check_interrupts(cpu);
            }
            /* Native hooks may switch task/window context. */
            window_hazard_refresh(cpu);
            *local_cc = cpu->cycle_count;  /* reload (stub may advance time) */
            /* Positive values are internal batch-accounting metadata: the
             * public xtensa_step() API still maps successful execution to 0. */
            return cpu->exception ? -1 : (hook_insns > 1 ? hook_insns : 0);
        }
    }
    /* This transition marker is normally already clear. */
    if (accelerator_fallthrough)
        cpu->jit_fallthrough_dispatch = false;

    /* Breakpoint check */
    if (__builtin_expect(cpu->breakpoint_count > 0, 0)) {
        /* Preserve invalid-PC-before-breakpoint ordering for debugger runs.
         * Normal predecoded execution proves the PC range with its table
         * bounds below and avoids this check entirely. */
        if (__builtin_expect(!xtensa_pc_is_valid(cpu, cpu->pc), 0))
            return xtensa_invalid_pc_step(cpu, *local_cc, last_pc);
        cpu->breakpoint_hit = false;
        for (int i = 0; i < cpu->breakpoint_count; i++) {
            if (cpu->breakpoints[i] == cpu->pc) {
                cpu->breakpoint_hit = true;
                cpu->breakpoint_hit_addr = cpu->pc;
                return -1;
            }
        }
    }

    /* AOT fast path: try the statically-recompiled function for this PC.
     * Skipped if the PC is in the ROM-stub bitmap — those functions
     * (esp_chip_info, memcpy, ets_printf, ...) implement hardware
     * behavior the AOT version can't fake.
     *
     * STATUS: integration plumbing verified, but the translator in
     * tools/aot_recompile.py has runtime correctness bugs that corrupt
     * cpu state on real firmware. Until they're shaken out via
     * differential testing (task #35), --aot is opt-in and prints a
     * warning at startup. Pass --aot only on a verified-good dylib. */
    /* AOT entry-point probing happens only after a control-flow
     * transfer — straight-line execution can't land on a new translated
     * function. `_pc_written` is set by branches/calls/returns/exceptions in
     * the previous step and consumed before executing this one. */
    if (__builtin_expect(pc_written && cpu->aot_bitmap != NULL, 0)) {
        /* An AOT lookup is the only normal-predecode operation that consumes
         * PC before the table proves it is in range. Keep invalid addresses
         * from reaching its bitmap while leaving straight-line interpreter
         * dispatch free of a redundant range check. */
        if (__builtin_expect(!xtensa_pc_is_valid(cpu, cpu->pc), 0))
            return xtensa_invalid_pc_step(cpu, *local_cc, last_pc);
        if (rom_stubs_hook_bitmap_test(cpu->aot_bitmap, cpu->pc) &&
            !(cpu->pc_hook_bitmap &&
              rom_stubs_hook_bitmap_test(cpu->pc_hook_bitmap, cpu->pc))) {
            typedef int (*aot_fn_t)(xtensa_cpu_t *);
            aot_fn_t fn = (aot_fn_t)cpu->aot_lookup_fn(cpu->aot, cpu->pc);
            if (fn) {
                /* Flush local_cc so the AOT function's stub-flush path
                 * (cpu->cycle_count += insn_count) stacks on top of the
                 * current total rather than racing. */
                cpu->cycle_count = *local_cc;
                int n = fn(cpu);
                if (n > 0) {
                    window_hazard_refresh(cpu);
                    cpu->ccount += (uint32_t)n;
                    *local_cc = cpu->cycle_count + (uint64_t)n;
                    if (__builtin_expect(cpu->ccount >= cpu->next_timer_event, 0))
                        xtensa_fire_timers(cpu);
                    return cpu->exception ? -1 : 0;
                }
            }
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
    /* A populated predecode entry is indexed from ESP32_INSN_ADDR_LOW and is
     * therefore also the common-path PC validity proof. Misses still take the
     * architectural invalid-PC trap before the fallback fetch. */
    if (__builtin_expect(!xtensa_pc_is_valid(cpu, cpu->pc), 0))
        return xtensa_invalid_pc_step(cpu, *local_cc, last_pc);
    ilen = xtensa_fetch_inline(cpu, cpu->pc, &insn);
    if (__builtin_expect(ilen == 0, 0)) {
        xtensa_raise_exception(cpu, EXCCAUSE_IFETCH_ERROR, cpu->pc, 0);
        if (cpu->exception) return -1;
        return 0;
    }
#if PREDECODE_SIZE > 0
have_insn:
#endif

#if FLEXE_PROFILE_BUILD
    xtensa_profile_tick_sp(cpu->pc, ar_read(cpu, 1));
#endif
    /* Avoid publishing a redundant zero on straight-line instructions. */
    if (pc_written)
        cpu->_pc_written = false;

    unsigned window_hazard = cpu->window_hazard;
    if (__builtin_expect(window_hazard != 0u, 0) &&
        window_access_check(cpu, insn, ilen, window_hazard)) {
        /* Faulted into a window vector; the instruction has not run and RFWO
         * returns to it. */
        cpu->ccount++;
        ++*local_cc;
        return cpu->exception ? -1 : 0;
    }

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
        case 6:
            if (__builtin_expect(exec_si(cpu, insn), 0)) {
                /* Signal the batch runner without adding a test to every
                 * ordinary instruction. Value 1 is intentionally not a
                 * native-span count; public xtensa_step() already maps it to
                 * one successfully retired instruction. */
                cpu->ccount++;
                ++*local_cc;
                if (__builtin_expect(cpu->ccount >= cpu->next_timer_event, 0))
                    xtensa_fire_timers(cpu);
                if (__builtin_expect(cpu->irq_check, 0)) {
                    cpu->irq_check = false;
                    if (cpu->interrupt & cpu->intenable)
                        xtensa_check_interrupts(cpu);
                }
                return cpu->exception ? -1 : 1;
            }
            break;
        case 7: exec_b(cpu, insn); break;
        default: break;
        }
    }

    /* Zero-overhead loop.  The back-edge is a taken branch, so flag the PC as
     * written: pc_hook dispatch is gated on _pc_written, and without this a
     * JIT block compiled at LBEG could never be entered — leaving every
     * compiler-emitted LOOP body running interpreted. */
    if (__builtin_expect(cpu->lcount > 0, 0) &&
        cpu->pc == cpu->lend && !cpu->_pc_written) {
        cpu->lcount--;
        cpu->pc = cpu->lbeg;
        cpu->_pc_written = true;
    }

    cpu->ccount++;
    ++*local_cc;

    if (__builtin_expect(cpu->ccount >= cpu->next_timer_event, 0))
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
    window_hazard_refresh(cpu);
    uint64_t cc = cpu->cycle_count;
    uint32_t prev_pc = cpu->dbg_prev_pc;
    const bool was_halted = cpu->halted;
    int r = xtensa_step_impl(cpu, &cc, &prev_pc, 0u);
    cpu->dbg_prev_pc = prev_pc;
    cpu->cycle_count = cc;
    /* One dispatch can retire many instructions when a native block runs, and
     * none at all when the core is halted.
     *
     * This used to add the advance in cycle_count, which is a different
     * quantity: a ROM stub charges for the guest function it stands in for,
     * and stub_ets_delay_us() walks the clock forward by an arbitrary amount.
     * That is skipped time, not retired work, and crediting it inflated
     * insn_count -- which every MIPS and JIT-coverage figure divides by.
     * A step_result above 1 is a native block reporting its own length;
     * anything else retired exactly one instruction. */
    if (!was_halted)
        cpu->insn_count += (r > 1) ? (uint64_t)r : 1u;
    if (cpu->ccount >= cpu->next_timer_event)
        xtensa_fire_timers(cpu);
    if (cpu->interrupt & cpu->intenable)
        xtensa_check_interrupts(cpu);
    return r < 0 ? -1 : 0;
}

/* Advance a WAITI core directly to its next observable event. No guest code
 * can run while halted, so visiting every intervening ccount value is both
 * unnecessary and extremely expensive for production firmware. The event
 * boundary is still exact: timers fire at the same ccount and an already
 * pending enabled interrupt wakes the core after one idle cycle, matching
 * xtensa_step_impl(). */
static inline int xtensa_run_halted(xtensa_cpu_t *cpu, uint64_t *local_cc,
                                    int max_cycles) {
    int executed = 0;

    while (cpu->halted && cpu->running && executed < max_cycles) {
        int remaining = max_cycles - executed;
        int advance = remaining;
        bool fire_event = false;

        if (cpu->interrupt & cpu->intenable) {
            advance = 1;
        } else if (cpu->next_timer_event != UINT32_MAX) {
            uint32_t distance;
            if ((int32_t)(cpu->ccount - cpu->next_timer_event) >= 0)
                distance = 1;  /* already due: step once, then fire */
            else
                distance = cpu->next_timer_event - cpu->ccount;

            if (distance <= (uint32_t)remaining) {
                advance = (int)distance;
                fire_event = true;
            }
        }

        cpu->ccount += (uint32_t)advance;
        *local_cc += (uint64_t)advance;
        executed += advance;

        if (fire_event)
            xtensa_fire_timers(cpu);

        if (cpu->interrupt & cpu->intenable) {
            cpu->halted = false;
            xtensa_check_interrupts(cpu);
        }
    }

    return executed;
}

/* Collapse complete repetitions of a structurally verified polling loop.
 * The other core and peripheral callbacks run only at the outer batch
 * boundary, so rereading the same byte inside this timeslice cannot observe a
 * change. Keep the architectural PC/register state at the loop head and
 * charge every skipped guest instruction. Never cross a timer boundary or a
 * debugger breakpoint: either can make an intermediate instruction visible. */
static inline int xtensa_run_poll_spin(xtensa_cpu_t *cpu,
                                       uint64_t *local_cc, int room) {
    unsigned width = cpu->poll_spin_insns;
    if (width == 0u || room < (int)width || cpu->breakpoint_count > 0 ||
        g_dbg_step_slow)
        return 0;

    int skip = room - room % (int)width;
    if (cpu->next_timer_event != UINT32_MAX) {
        uint32_t distance = (int32_t)(cpu->ccount - cpu->next_timer_event) >= 0
                          ? 0u : cpu->next_timer_event - cpu->ccount;
        if (distance <= 1u)
            return 0;
        uint32_t safe = distance - 1u;
        if ((uint32_t)skip > safe)
            skip = (int)(safe - safe % width);
    }
    if (skip <= 0)
        return 0;

    cpu->ccount += (uint32_t)skip;
    *local_cc += (uint64_t)skip;
    return skip;
}

/* Batch execution: step_impl is always_inline → entire decode/execute loop
 * lives in this function body, eliminating per-instruction call overhead.
 * cycle_count is cached in a local to stay in a register across iterations
 * (avoids per-instruction 64-bit memory increment). */

int xtensa_run(xtensa_cpu_t *cpu, int max_cycles) {
    window_hazard_refresh(cpu);
    uint64_t cc = cpu->cycle_count;
    uint32_t prev_pc = cpu->dbg_prev_pc;
    int idle_total = 0;   /* skipped time: advances the clock, retires nothing */
    int exec_total = 0;   /* guest instructions actually retired */

    /* A handoff belongs to one invocation. The caller can inspect the flag
     * after an early return; the next scheduled turn starts fresh. */
    cpu->core_handoff = false;

    /* Alternate between running and idling until the budget is spent.
     *
     * The loop matters. A core that executes WAITI mid-batch has to have its
     * idle skipped -- stepping it a cycle at a time both wasted host time and
     * counted the cycles as retired instructions -- but skipping to the next
     * timer event usually *wakes* it, and the batch must then carry on
     * executing. Returning early instead silently starves the guest: it gets
     * the simulated time but not the work.
     *
     * jit_run() has always had this loop, so only the interpreter was
     * affected, and the stock-ROM gates could not see it because NerdMiner's
     * mining loop -- the one workload that keeps both cores busy for long
     * stretches -- was never actually reached. Once it was, the interpreter
     * submitted zero shares against the JIT's 27.
     */
    /* Not `while (cpu->running && ...)`: the stepping loops check `running`
     * *after* a dispatch, and callers rely on that -- a hook installed at the
     * entry PC must get its one chance to run even when the CPU has not been
     * marked running. Exit conditions are at the bottom instead. */
    for (;;) {
        int remaining = max_cycles - exec_total - idle_total;
        if (remaining <= 0) break;

        if (__builtin_expect(cpu->halted, 0)) {
            int idled = xtensa_run_halted(cpu, &cc, remaining);
            if (idled <= 0) break;          /* nothing to wait for */
            idle_total += idled;
            if (cpu->halted) break;         /* still idle: budget is gone */
            continue;                       /* woke up: go execute */
        }

        int executed = 0;
        /* A native hook can execute hundreds of guest instructions in one
         * xtensa_step_impl() dispatch. Bound and report the batch in guest
         * instructions rather than hook invocations, so embedded frontends
         * keep timer/preemption cadence and throughput accounting honest. */
        if (__builtin_expect(cpu->accelerated_blocks, 0)) {
            for (; executed < remaining; executed++) {
                int step_result = xtensa_step_impl(
                        cpu, &cc, &prev_pc,
                        (uint32_t)(remaining - executed));
                if (__builtin_expect(step_result != 0, 0)) {
                    if (step_result < 0) { executed++; break; }
                    if (step_result == 1)
                        executed += xtensa_run_poll_spin(
                                cpu, &cc, remaining - executed - 1);
                    else
                        executed += step_result - 1;
                }
                if (__builtin_expect(!cpu->running, 0)) {
                    executed++;   /* include the dispatch that stopped the CPU */
                    break;
                }
                if (__builtin_expect(cpu->halted, 0)) {
                    executed++;   /* the WAITI itself did retire */
                    break;
                }
                if (__builtin_expect(cpu->core_handoff, 0)) {
                    executed++;   /* include the contended S32C1I/native run */
                    break;
                }
            }
        } else {
            for (; executed < remaining; executed++) {
                int step_result = xtensa_step_impl(cpu, &cc, &prev_pc, 0u);
                if (__builtin_expect(step_result != 0, 0)) {
                    if (step_result < 0 || step_result > 1) {
                        executed++;
                        break;
                    }
                    executed += xtensa_run_poll_spin(
                            cpu, &cc, remaining - executed - 1);
                }
                if (__builtin_expect(!cpu->running, 0)) { executed++; break; }
                if (__builtin_expect(cpu->halted, 0)) { executed++; break; }
                if (__builtin_expect(cpu->core_handoff, 0)) { executed++; break; }
            }
        }
        cpu->native_span_room = 0u;
        exec_total += executed;
        /* No progress, or the CPU stopped: either way do not spin. */
        if (executed == 0 || !cpu->running || cpu->core_handoff) break;
    }

    cpu->cycle_count = cc;
    cpu->dbg_prev_pc = prev_pc;
    cpu->insn_count += (uint64_t)exec_total;   /* idle retires nothing */
    return idle_total + exec_total;
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
