#include "xtensa.h"
#include "memory.h"
#include <string.h>
#include <stdio.h>

void xtensa_cpu_init(xtensa_cpu_t *cpu) {
    memset(cpu, 0, sizeof(*cpu));
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

int xtensa_step(xtensa_cpu_t *cpu) {
    /* Stub: advance PC by 3 (24-bit instruction) */
    cpu->pc += 3;
    cpu->ccount++;
    cpu->cycle_count++;
    return 0;
}

int xtensa_run(xtensa_cpu_t *cpu, int max_cycles) {
    int i;
    for (i = 0; i < max_cycles && cpu->running; i++) {
        if (xtensa_step(cpu) != 0)
            break;
    }
    return i;
}

int xtensa_fetch(const xtensa_cpu_t *cpu, uint32_t addr, uint32_t *insn_out) {
    const uint8_t *ptr = mem_get_ptr(cpu->mem, addr);
    if (!ptr) return 0;
    int op0 = ptr[0] & 0xF;
    if (op0 >= 8) {
        *insn_out = ptr[0] | ((uint32_t)ptr[1] << 8);
        return 2;
    } else {
        *insn_out = ptr[0] | ((uint32_t)ptr[1] << 8) | ((uint32_t)ptr[2] << 16);
        return 3;
    }
}

/* xtensa_disasm() is in xtensa_disasm.c */
