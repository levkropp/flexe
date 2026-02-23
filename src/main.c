#include "xtensa.h"
#include "memory.h"
#include "loader.h"
#include <stdio.h>

int main(int argc, char *argv[]) {
    xtensa_cpu_t cpu;
    xtensa_mem_t *mem = mem_create();
    if (!mem) {
        fprintf(stderr, "Failed to allocate memory\n");
        return 1;
    }

    xtensa_cpu_init(&cpu);
    xtensa_cpu_reset(&cpu);
    cpu.mem = mem;

    if (argc > 1) {
        load_result_t res = loader_load_bin(mem, argv[1]);
        if (res.result != 0) {
            fprintf(stderr, "Load error: %s\n", res.error);
            mem_destroy(mem);
            return 1;
        }
        if (res.entry_point != 0)
            cpu.pc = res.entry_point;
    }

    int cycles = xtensa_run(&cpu, 100);
    printf("Executed %d cycles, PC = 0x%08X\n", cycles, cpu.pc);

    mem_destroy(mem);
    return 0;
}
