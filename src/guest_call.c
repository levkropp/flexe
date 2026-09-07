#include "guest_call.h"
#include "memory.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* RTC-slow page 1 is intentionally left available by memory.c.  Keeping the
 * synthetic task here avoids corrupting whichever FreeRTOS stack happened to
 * be interrupted when a host device delivered an event. */
#define GUEST_CALL_STACK_TOP 0x60002000u
#define GUEST_CALL_SENTINEL  0x40001FF8u
#define GUEST_CALL_MAX_ARGS  6u

int guest_call8(xtensa_cpu_t *cpu, uint32_t entry,
                const uint32_t *args, size_t arg_count,
                uint32_t instruction_limit, uint32_t *retval_out)
{
    if (!cpu || !cpu->mem || entry == 0 || instruction_limit == 0 ||
        arg_count > GUEST_CALL_MAX_ARGS || (arg_count != 0 && !args))
        return -1;

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
    uint32_t save_fcr = cpu->fcr;
    uint32_t save_fsr = cpu->fsr;
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
    ar_write(cpu, 1, GUEST_CALL_STACK_TOP);
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
    cpu->fcr = save_fcr;
    cpu->fsr = save_fsr;
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
    if (!completed)
        return -2;
    if (retval_out)
        *retval_out = retval;
    return 0;
}

/* ===== Asynchronous callbacks: see guest_call_async() in guest_call.h ===== */

/* Saved on the borrowed task's stack, below its frame, so that a context
 * switch in the middle of the callee cannot lose it. CALL8 hands a0-a7 of the
 * new window to the callee, which is the caller's a8-a15, so those eight are
 * the ones the windowed ABI does not preserve for us. */
#define ASYNC_FRAME_BYTES 48u
#define ASYNC_OFF_RESUME  32u
#define ASYNC_OFF_MAGIC   36u
#define ASYNC_MAGIC       0x41535943u  /* 'ASYC' */

/* One outstanding callback at a time, matching the one-event-per-batch rule
 * the WiFi model already follows. The frame is self-describing, so this is
 * only a rate limit -- correctness does not depend on it. */
static bool     async_busy;
static uint64_t async_deadline;

bool guest_call_async_busy(const xtensa_cpu_t *cpu) {
    if (!async_busy) return false;
    /* A callee that never returns would otherwise stop event delivery for the
     * rest of the run. Give up tracking it after a second of guest time; the
     * frame stays valid, so a late return still restores correctly. */
    if (cpu && cpu->cycle_count > async_deadline) {
        fprintf(stderr, "[flexe] async callback did not return within 1s of "
                        "guest time; allowing the next one\n");
        async_busy = false;
        return false;
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
    if ((sp & 3u) != 0 || sp < 0x3FF80000u || sp >= 0x40000000u ||
        sp - ASYNC_FRAME_BYTES < 0x3FF80000u) {
        if (getenv("FLEXE_ASYNCDBG"))
            fprintf(stderr, "[async] refused: sp=%08X\n", sp);
        return -1;
    }

    uint32_t nsp = sp - ASYNC_FRAME_BYTES;
    for (int i = 0; i < 8; i++)
        mem_write32(cpu->mem, nsp + (uint32_t)i * 4u, ar_read(cpu, 8 + i));
    mem_write32(cpu->mem, nsp + ASYNC_OFF_RESUME, cpu->pc);
    mem_write32(cpu->mem, nsp + ASYNC_OFF_MAGIC, ASYNC_MAGIC);
    ar_write(cpu, 1, nsp);

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
    if (getenv("FLEXE_ASYNCDBG"))
        fprintf(stderr, "[async] start entry=%08X sp=%08X resume=%08X ps=%08X "
                "wb=%u ws=%08X core%d\n", entry, nsp,
                mem_read32(cpu->mem, nsp + ASYNC_OFF_RESUME), cpu->ps,
                cpu->windowbase, cpu->windowstart, cpu->core_id);
    return 0;
}

void guest_call_async_return(xtensa_cpu_t *cpu)
{
    uint32_t sp = ar_read(cpu, 1);
    if (mem_read32(cpu->mem, sp + ASYNC_OFF_MAGIC) != ASYNC_MAGIC) {
        /* Reaching the sentinel without our frame under a1 means the callee
         * returned on a different stack than it was started on. Restoring
         * from whatever is there would be worse than stopping. */
        fprintf(stderr, "[flexe] async callback returned with a1=0x%08X and no "
                        "frame magic; cannot resume\n", sp);
        async_busy = false;
        cpu->running = false;
        return;
    }
    for (int i = 0; i < 8; i++)
        ar_write(cpu, 8 + i, mem_read32(cpu->mem, sp + (uint32_t)i * 4u));
    cpu->pc = mem_read32(cpu->mem, sp + ASYNC_OFF_RESUME);
    ar_write(cpu, 1, sp + ASYNC_FRAME_BYTES);
    if (getenv("FLEXE_ASYNCDBG"))
        fprintf(stderr, "[async] return sp=%08X resume=%08X core%d\n",
                sp, cpu->pc, cpu->core_id);
    cpu->_pc_written = true;
    async_busy = false;
}
