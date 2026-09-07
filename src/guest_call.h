#ifndef GUEST_CALL_H
#define GUEST_CALL_H

#include "xtensa.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Run a firmware function synchronously using the Xtensa windowed CALL8 ABI.
 * The call uses a private guest stack and restores the interrupted task's
 * architectural state on return. Guest memory writes and elapsed emulated
 * time remain visible, and a completed asynchronous call wakes WAITI just
 * like interrupt entry.
 *
 * Returns 0 when the function returns normally, -1 for invalid arguments,
 * and -2 if the instruction limit is reached or the guest faults/stops.
 * CALL8 exposes six register arguments (a10-a15) to the callee. */
int guest_call8(xtensa_cpu_t *cpu, uint32_t entry,
                const uint32_t *args, size_t arg_count,
                uint32_t instruction_limit, uint32_t *retval_out);

/* Run a firmware function that is allowed to block.
 *
 * guest_call8() cannot deliver an IDF event handler. It runs the callee on a
 * fabricated register file at INTLEVEL 15, so the callee can never yield --
 * and the default esp_netif/esp_wifi handlers take locks. What happens
 * instead is worse than a stall: the callee is impersonating whichever task
 * the emulator interrupted, so FreeRTOS blocks *that* task, the yield does
 * not happen, the retry loop comes round and places the same task on the same
 * event list a second time, and the list closes into a ring. WLED dies there,
 * with one core walking a circular list for ever holding the queue's mux.
 *
 * This starts the callee on the interrupted task's own stack, in its own
 * window, with its own PS. It then returns immediately and lets ordinary
 * emulation run the callee. If the callee blocks, the task blocks, the
 * scheduler switches away, and the callee resumes when the semaphore is
 * given -- which is what the hardware does, because there the handler runs in
 * the event task.
 *
 * Everything needed to resume the interrupted code lives on that task's
 * stack, so it survives any number of context switches. Returns 0 if the call
 * was started, -1 if the guest is not in a state where borrowing the current
 * task is safe; the caller should try again later. */
int guest_call_async(xtensa_cpu_t *cpu, uint32_t entry,
                     const uint32_t *args, size_t arg_count);

/* True while a call started by guest_call_async() has not yet returned.
 * Callers use it to deliver one callback at a time. */
bool guest_call_async_busy(const xtensa_cpu_t *cpu);

/* Called by the interpreter when the PC reaches the async return sentinel. */
void guest_call_async_return(xtensa_cpu_t *cpu);

/* The address a guest_call_async() callee returns to. Never a real
 * instruction: the interpreter intercepts it. */
#define GUEST_CALL_ASYNC_SENTINEL 0x40001FF0u

#endif /* GUEST_CALL_H */
