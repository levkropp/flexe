#ifndef GUEST_CALL_H
#define GUEST_CALL_H

#include "xtensa.h"

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

#endif /* GUEST_CALL_H */
