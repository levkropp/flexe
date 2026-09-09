#include "guest_call.h"
#include "memory.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* A synchronous callback needs a private guest-visible stack so it cannot
 * corrupt whichever FreeRTOS task was interrupted. Map one otherwise-invalid
 * page only for the duration of the call, then restore the page-table entry.
 * This keeps the trampoline out of every architectural ESP32 RAM/MMIO region.
 * The old permanent mapping at 0x60001000 collided with the AHB peripheral
 * mirror and was incorrectly described as RTC slow RAM. */
#define GUEST_CALL_STACK_BASE 0x7FFF0000u
#define GUEST_CALL_STACK_SIZE 0x1000u
#define GUEST_CALL_STACK_TOP  (GUEST_CALL_STACK_BASE + GUEST_CALL_STACK_SIZE)
#define GUEST_CALL_STACK_ROOT (GUEST_CALL_STACK_TOP - 48u)
#define GUEST_CALL_SENTINEL  0x40001FF8u
#define GUEST_CALL_MAX_ARGS  6u

static bool guest_core_is_quiescent(const xtensa_cpu_t *cpu)
{
    return cpu && cpu->running && cpu->halted && !cpu->exception &&
           !cpu->in_guest_call && XT_PS_INTLEVEL(cpu->ps) == 0 &&
           !XT_PS_EXCM(cpu->ps);
}

bool guest_call_injection_is_quiescent(const xtensa_cpu_t *target,
                                       const xtensa_cpu_t *peer)
{
    if (!guest_core_is_quiescent(target))
        return false;
    return !peer || !peer->running || guest_core_is_quiescent(peer);
}

int guest_call8(xtensa_cpu_t *cpu, uint32_t entry,
                const uint32_t *args, size_t arg_count,
                uint32_t instruction_limit, uint32_t *retval_out)
{
    if (!cpu || !cpu->mem || entry == 0 || instruction_limit == 0 ||
        arg_count > GUEST_CALL_MAX_ARGS || (arg_count != 0 && !args))
        return -1;

    uint8_t guest_stack[GUEST_CALL_STACK_SIZE];
    memset(guest_stack, 0, sizeof(guest_stack));
    uint32_t guest_stack_page = GUEST_CALL_STACK_BASE >> 12;
    uint8_t *saved_stack_page = cpu->mem->page_table[guest_stack_page];
    cpu->mem->page_table[guest_stack_page] = guest_stack;

    uint32_t save_ar[64];
    uint32_t save_pc = cpu->pc;
    uint32_t save_ps = cpu->ps;
    uint32_t save_windowbase = cpu->windowbase;
    uint32_t save_windowstart = cpu->windowstart;
    uint32_t save_sar = cpu->sar;
    uint32_t save_lbeg = cpu->lbeg;
    uint32_t save_lend = cpu->lend;
    uint32_t save_lcount = cpu->lcount;
    uint32_t save_br = cpu->br;
    uint32_t save_acclo = cpu->acclo;
    uint32_t save_acchi = cpu->acchi;
    uint32_t save_mr[4];
    uint32_t save_expstate = cpu->expstate;
    uint32_t save_threadptr = cpu->threadptr;
    uint32_t save_fcr = cpu->fcr;
    uint32_t save_fsr = cpu->fsr;
    uint32_t save_f64r_lo = cpu->f64r_lo;
    uint32_t save_f64r_hi = cpu->f64r_hi;
    uint32_t save_f64s = cpu->f64s;
    uint32_t save_cpenable = cpu->cpenable;
    float save_fr[16];
    uint8_t save_window_callsize[sizeof(cpu->window_callsize)];
    uint8_t save_spill_stack[sizeof(cpu->spill_stack)];
    uint8_t save_spill_base[sizeof(cpu->spill_base)];
    uint8_t save_spill_shadow[sizeof(cpu->spill_shadow)];
    bool save_running = cpu->running;
    bool save_halted = cpu->halted;
    bool save_exception = cpu->exception;
    bool save_pc_written = cpu->_pc_written;
    bool save_irq_check = cpu->irq_check;
    bool save_accelerated = cpu->accelerated_blocks;

    memcpy(save_ar, cpu->ar, sizeof(save_ar));
    memcpy(save_mr, cpu->mr, sizeof(save_mr));
    memcpy(save_fr, cpu->fr, sizeof(save_fr));
    memcpy(save_window_callsize, cpu->window_callsize,
           sizeof(save_window_callsize));
    memcpy(save_spill_stack, cpu->spill_stack, sizeof(save_spill_stack));
    memcpy(save_spill_base, cpu->spill_base, sizeof(save_spill_base));
    memcpy(save_spill_shadow, cpu->spill_shadow, sizeof(save_spill_shadow));

    memset(cpu->ar, 0, sizeof(cpu->ar));
    memset(cpu->window_callsize, 0, sizeof(cpu->window_callsize));
    memset(cpu->spill_stack, 0, sizeof(cpu->spill_stack));
    memset(cpu->spill_base, 0, sizeof(cpu->spill_base));
    memset(cpu->spill_shadow, 0, sizeof(cpu->spill_shadow));
    cpu->windowbase = 0;
    cpu->windowstart = 1;
    cpu->ps = 1u << 18; /* WOE, kernel mode */
    XT_PS_SET_INTLEVEL(cpu->ps, 15);
    XT_PS_SET_CALLINC(cpu->ps, 2);
    /* Match ESP-IDF's SET_STACK bootstrap frame. WindowOverflow8/12 save the
     * high registers relative to the caller SP stored at [a1-12], and the
     * matching underflow vectors follow that link to reload them. A bare top
     * of stack leaves a zero link, turning the first deep CALL8 return into
     * reads from 0xffffffe0. */
    mem_write32(cpu->mem, GUEST_CALL_STACK_ROOT - 12u,
                GUEST_CALL_STACK_TOP);
    ar_write(cpu, 1, GUEST_CALL_STACK_ROOT);
    ar_write(cpu, 8, (2u << 30) |
                     (GUEST_CALL_SENTINEL & 0x3FFFFFFFu));
    for (size_t i = 0; i < arg_count; i++)
        ar_write(cpu, 10 + (int)i, args[i]);
    cpu->pc = entry;
    cpu->running = true;
    cpu->halted = false;
    cpu->exception = false;
    cpu->_pc_written = true;
    cpu->irq_check = false;

    bool completed = false;
    uint32_t retval = 0;
    uint32_t executed = 0;
    cpu->in_guest_call++;
    for (; executed < instruction_limit; executed++) {
        if (cpu->pc == GUEST_CALL_SENTINEL) {
            completed = true;
            retval = ar_read(cpu, 10);
            break;
        }
        if (!cpu->running || cpu->exception)
            break;
        xtensa_step(cpu);
    }

    if (!completed && getenv("FLEXE_GUESTCALLDBG"))
        fprintf(stderr,
                "[guest-call] entry=0x%08X stopped pc=0x%08X "
                "instructions=%u/%u running=%d exception=%d ps=0x%08X "
                "wb=%u ws=0x%08X a0=0x%08X a1=0x%08X a2=0x%08X "
                "a3=0x%08X a4=0x%08X a5=0x%08X\n",
                entry, cpu->pc, executed, instruction_limit,
                cpu->running, cpu->exception, cpu->ps, cpu->windowbase,
                cpu->windowstart, ar_read(cpu, 0), ar_read(cpu, 1),
                ar_read(cpu, 2), ar_read(cpu, 3), ar_read(cpu, 4),
                ar_read(cpu, 5));

    memcpy(cpu->ar, save_ar, sizeof(save_ar));
    memcpy(cpu->mr, save_mr, sizeof(save_mr));
    memcpy(cpu->fr, save_fr, sizeof(save_fr));
    memcpy(cpu->window_callsize, save_window_callsize,
           sizeof(save_window_callsize));
    memcpy(cpu->spill_stack, save_spill_stack, sizeof(save_spill_stack));
    memcpy(cpu->spill_base, save_spill_base, sizeof(save_spill_base));
    memcpy(cpu->spill_shadow, save_spill_shadow, sizeof(save_spill_shadow));
    cpu->pc = save_pc;
    cpu->ps = save_ps;
    cpu->windowbase = save_windowbase;
    cpu->windowstart = save_windowstart;
    cpu->sar = save_sar;
    cpu->lbeg = save_lbeg;
    cpu->lend = save_lend;
    cpu->lcount = save_lcount;
    cpu->br = save_br;
    cpu->acclo = save_acclo;
    cpu->acchi = save_acchi;
    cpu->expstate = save_expstate;
    cpu->threadptr = save_threadptr;
    cpu->fcr = save_fcr;
    cpu->fsr = save_fsr;
    cpu->f64r_lo = save_f64r_lo;
    cpu->f64r_hi = save_f64r_hi;
    cpu->f64s = save_f64s;
    cpu->cpenable = save_cpenable;
    cpu->running = save_running;
    /* A completed asynchronous guest callback represents interrupt/event
     * delivery. Interrupt entry wakes WAITI, so do not reinstate a halted
     * bit that the private callback stack intentionally cleared. On a failed
     * synthetic call, restore the original execution state in full. */
    cpu->halted = completed ? false : save_halted;
    cpu->exception = save_exception;
    cpu->_pc_written = save_pc_written;
    cpu->irq_check = save_irq_check;
    cpu->accelerated_blocks = save_accelerated;

    cpu->in_guest_call--;
    cpu->mem->page_table[guest_stack_page] = saved_stack_page;
    if (!completed)
        return -2;
    if (retval_out)
        *retval_out = retval;
    return 0;
}

/* ===== Asynchronous callbacks: see guest_call_async() in guest_call.h ===== */

/* CALL8 hands a0-a7 of the new window to the callee, which is the caller's
 * a8-a15, so those eight are the interrupted registers the ABI does not
 * preserve for us. Keep them outside guest memory while the callback runs.
 *
 * An earlier implementation made room by subtracting 48 from the borrowed
 * task's a1 and putting this state there. That silently changed the ABI link
 * for every live ancestor window: if the callback caused one to overflow, it
 * was saved relative to the temporary a1. After the callback restored the old
 * a1, RETW looked for that ancestor at the old link and reloaded the callback
 * marker as a stack pointer. Leaving a1 alone makes the injected CALL8 a
 * normal nested call and keeps every overflow/underflow save area coherent.
 * Host state remains available across a FreeRTOS context switch just as guest
 * RAM does. */
static uint32_t async_saved_a8_a15[8];
static uint32_t async_resume_pc;
static uint32_t async_resume_sp;
static uint32_t async_resume_wb;

/* One outstanding callback at a time, matching the one-event-per-batch rule
 * the WiFi model already follows. There is one continuation slot, so this is
 * also the backpressure that prevents a second callback from replacing it. */
static bool     async_busy;
static uint64_t async_deadline;
static bool     async_timeout_reported;

bool guest_call_async_busy(const xtensa_cpu_t *cpu) {
    if (!async_busy) return false;
    /* Never overwrite a still-live continuation. A callback that blocks for
     * a long time legitimately owns the borrowed task until it resumes; a
     * second injected call would make either late return restore the other's
     * registers. Report the stall once, but retain correctness and backpressure
     * until this callback actually returns. */
    if (cpu && cpu->cycle_count > async_deadline &&
        !async_timeout_reported) {
        fprintf(stderr, "[flexe] async callback has not returned after 1s of "
                        "guest time; deferring subsequent callbacks\n");
        async_timeout_reported = true;
    }
    return true;
}

int guest_call_async(xtensa_cpu_t *cpu, uint32_t entry,
                     const uint32_t *args, size_t arg_count)
{
    if (!cpu || !cpu->mem || entry == 0 || arg_count > 4 ||
        (arg_count != 0 && !args))
        return -1;
    if (guest_call_async_busy(cpu) || cpu->in_guest_call)
        return -1;
    /* Borrowing the current task is only safe when it is running ordinary
     * code: not halted, not mid-exception, and not inside a critical section
     * or an interrupt handler, where blocking it would deadlock whatever it
     * holds. Windowed calls also need PS.WOE. */
    if (!cpu->running || cpu->halted || cpu->exception ||
        XT_PS_INTLEVEL(cpu->ps) != 0 || XT_PS_EXCM(cpu->ps) ||
        !XT_PS_WOE(cpu->ps)) {
        if (getenv("FLEXE_ASYNCDBG"))
            fprintf(stderr, "[async] refused: run=%d halt=%d exc=%d ps=%08X\n",
                    cpu->running, cpu->halted, cpu->exception, cpu->ps);
        return -1;
    }

    uint32_t sp = ar_read(cpu, 1);
    if ((sp & 3u) != 0 || sp < 0x3FF80000u || sp >= 0x40000000u) {
        if (getenv("FLEXE_ASYNCDBG"))
            fprintf(stderr, "[async] refused: sp=%08X\n", sp);
        return -1;
    }

    for (int i = 0; i < 8; i++)
        async_saved_a8_a15[i] = ar_read(cpu, 8 + i);
    async_resume_pc = cpu->pc;
    async_resume_sp = sp;
    async_resume_wb = cpu->windowbase;

    for (size_t i = 0; i < arg_count; i++)
        ar_write(cpu, 10 + (int)i, args[i]);   /* callee sees a2..a5 */
    /* CALL8 writes its return address into a8 with the window-size encoding
     * in bits 31:30, and sets PS.CALLINC so the callee's ENTRY rotates by 8. */
    ar_write(cpu, 8, (2u << 30) | (GUEST_CALL_ASYNC_SENTINEL & 0x3FFFFFFFu));
    XT_PS_SET_CALLINC(cpu->ps, 2);
    cpu->pc = entry;
    cpu->_pc_written = true;

    async_busy = true;
    async_deadline = cpu->cycle_count + 240000000ull;
    async_timeout_reported = false;
    if (getenv("FLEXE_ASYNCDBG"))
        fprintf(stderr, "[async] start entry=%08X sp=%08X resume=%08X ps=%08X "
                "wb=%u ws=%08X core%d\n", entry, sp,
                async_resume_pc, cpu->ps,
                cpu->windowbase, cpu->windowstart, cpu->core_id);
    return 0;
}

void guest_call_async_return(xtensa_cpu_t *cpu)
{
    uint32_t sp = ar_read(cpu, 1);
    if (!async_busy || sp != async_resume_sp ||
        cpu->windowbase != async_resume_wb) {
        /* Reaching the sentinel in another frame means the callee did not
         * unwind the injected CALL8. Restoring registers into that frame
         * would corrupt it, so stop with an actionable diagnostic. */
        fprintf(stderr, "[flexe] async callback returned in the wrong frame "
                        "(a1=0x%08X wb=%u, expected a1=0x%08X wb=%u)\n",
                        sp, cpu->windowbase, async_resume_sp, async_resume_wb);
        async_busy = false;
        cpu->running = false;
        return;
    }
    for (int i = 0; i < 8; i++)
        ar_write(cpu, 8 + i, async_saved_a8_a15[i]);
    cpu->pc = async_resume_pc;
    if (getenv("FLEXE_ASYNCDBG"))
        fprintf(stderr, "[async] return sp=%08X resume=%08X core%d\n",
                sp, cpu->pc, cpu->core_id);
    cpu->_pc_written = true;
    async_busy = false;
}
