#include "xtensa.h"
#include "memory.h"
#include <stdio.h>
#include <stdlib.h>

int main(int argc, char *argv[]) {
    if (argc < 2) {
        fprintf(stderr, "Usage: xt-dis <binary> [base_addr]\n");
        return 1;
    }

    uint32_t base = (argc > 2) ? (uint32_t)strtoul(argv[2], NULL, 0) : 0x40080000;

    FILE *f = fopen(argv[1], "rb");
    if (!f) {
        fprintf(stderr, "Cannot open: %s\n", argv[1]);
        return 1;
    }

    /* Get file size */
    fseek(f, 0, SEEK_END);
    long size = ftell(f);
    fseek(f, 0, SEEK_SET);

    if (size <= 0) {
        fprintf(stderr, "Empty file\n");
        fclose(f);
        return 1;
    }

    uint8_t *data = malloc(size);
    if (!data) {
        fprintf(stderr, "Out of memory\n");
        fclose(f);
        return 1;
    }

    if (fread(data, 1, size, f) != (size_t)size) {
        fprintf(stderr, "Read error\n");
        free(data);
        fclose(f);
        return 1;
    }
    fclose(f);

    /* Create CPU + memory and load the data */
    xtensa_cpu_t cpu;
    xtensa_cpu_init(&cpu);
    cpu.mem = mem_create();
    if (!cpu.mem) {
        fprintf(stderr, "Failed to create memory\n");
        free(data);
        return 1;
    }

    /* Load binary into memory at base address */
    if (mem_load(cpu.mem, base, data, size) != 0) {
        fprintf(stderr, "Failed to load binary at 0x%08X (size %ld)\n", base, size);
        mem_destroy(cpu.mem);
        free(data);
        return 1;
    }

    /* Disassemble */
    uint32_t addr = base;
    uint32_t end = base + (uint32_t)size;
    char buf[128];

    while (addr < end) {
        /* Print address and raw bytes */
        const uint8_t *ptr = mem_get_ptr(cpu.mem, addr);
        if (!ptr) {
            printf("0x%08X: (unmapped)\n", addr);
            break;
        }

        int op0 = ptr[0] & 0xF;
        int len = (op0 >= 8) ? 2 : 3;

        if (addr + (uint32_t)len > end) {
            printf("0x%08X: (truncated)\n", addr);
            break;
        }

        /* Disassemble */
        int ilen = xtensa_disasm(&cpu, addr, buf, sizeof(buf));

        /* Print raw bytes */
        printf("0x%08X: ", addr);
        for (int i = 0; i < ilen; i++)
            printf("%02x ", ptr[i]);
        if (ilen == 2) printf("   ");
        printf("\t%s\n", buf);

        addr += (uint32_t)ilen;
    }

    mem_destroy(cpu.mem);
    free(data);
    return 0;
}
