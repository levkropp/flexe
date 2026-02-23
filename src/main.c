#include "xtensa.h"
#include "memory.h"
#include "loader.h"
#include "peripherals.h"
#include "rom_stubs.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <getopt.h>

static void uart_stdout_cb(void *ctx, uint8_t byte) {
    (void)ctx;
    putchar(byte);
}

static void usage(const char *prog) {
    fprintf(stderr, "Usage: %s [options] <firmware.bin>\n", prog);
    fprintf(stderr, "  -c <cycles>   Max cycles (default: 10000000)\n");
    fprintf(stderr, "  -t            Instruction trace to stderr\n");
    fprintf(stderr, "  -v            Verbose register dump on exit\n");
    fprintf(stderr, "  -e <addr>     Override entry point (hex)\n");
}

int main(int argc, char *argv[]) {
    int max_cycles = 10000000;
    int trace = 0;
    int verbose = 0;
    uint32_t entry_override = 0;
    int has_entry_override = 0;

    int opt;
    while ((opt = getopt(argc, argv, "c:tve:")) != -1) {
        switch (opt) {
        case 'c': max_cycles = atoi(optarg); break;
        case 't': trace = 1; break;
        case 'v': verbose = 1; break;
        case 'e': entry_override = (uint32_t)strtoul(optarg, NULL, 16); has_entry_override = 1; break;
        default: usage(argv[0]); return 1;
        }
    }

    if (optind >= argc) {
        usage(argv[0]);
        return 1;
    }
    const char *firmware = argv[optind];

    xtensa_mem_t *mem = mem_create();
    if (!mem) {
        fprintf(stderr, "Failed to allocate memory\n");
        return 1;
    }

    /* Initialize peripherals before CPU execution */
    esp32_periph_t *periph = periph_create(mem);
    if (!periph) {
        fprintf(stderr, "Failed to create peripherals\n");
        mem_destroy(mem);
        return 1;
    }
    periph_set_uart_callback(periph, uart_stdout_cb, NULL);

    /* Load firmware */
    load_result_t res = loader_load_bin(mem, firmware);
    if (res.result != 0) {
        fprintf(stderr, "Load error: %s\n", res.error);
        periph_destroy(periph);
        mem_destroy(mem);
        return 1;
    }
    fprintf(stderr, "Loaded %s: %d segments, entry=0x%08X\n",
            firmware, res.segment_count, res.entry_point);

    /* Initialize CPU */
    xtensa_cpu_t cpu;
    xtensa_cpu_init(&cpu);
    xtensa_cpu_reset(&cpu);
    cpu.mem = mem;

    /* Install ROM function stubs */
    esp32_rom_stubs_t *rom = rom_stubs_create(&cpu);
    if (!rom) {
        fprintf(stderr, "Failed to create ROM stubs\n");
        periph_destroy(periph);
        mem_destroy(mem);
        return 1;
    }

    if (has_entry_override)
        cpu.pc = entry_override;
    else if (res.entry_point != 0)
        cpu.pc = res.entry_point;

    /* Execute */
    int cycles = 0;
    char disasm_buf[128];
    if (trace) {
        for (int i = 0; i < max_cycles; i++) {
            xtensa_disasm(&cpu, cpu.pc, disasm_buf, sizeof(disasm_buf));
            fprintf(stderr, "[%08X] %s\n", cpu.pc, disasm_buf);
            int rc = xtensa_step(&cpu);
            cycles++;
            if (rc < 0) {
                /* Exception — let vector handler deal with it unless halted */
                if (cpu.halted) {
                    fprintf(stderr, "CPU halted (WAITI) at cycle %d\n", cycles);
                    break;
                }
            }
            if (!cpu.running) break;
        }
    } else {
        /* Batch execution in chunks */
        int batch = 10000;
        while (cycles < max_cycles && cpu.running && !cpu.halted) {
            int remaining = max_cycles - cycles;
            int n = remaining < batch ? remaining : batch;
            int ran = xtensa_run(&cpu, n);
            cycles += ran;
            if (ran < n) break;
        }
    }

    /* Summary */
    fprintf(stderr, "\n--- Execution summary ---\n");
    fprintf(stderr, "Cycles:     %d\n", cycles);
    fprintf(stderr, "Final PC:   0x%08X\n", cpu.pc);
    fprintf(stderr, "UART TX:    %d bytes\n", periph_uart_tx_count(periph));
    fprintf(stderr, "Unhandled:  %d peripheral accesses\n", periph_unhandled_count(periph));
    fprintf(stderr, "ROM calls:  %d output bytes\n", rom_stubs_output_count(rom));

    if (verbose) {
        fprintf(stderr, "\n--- Registers ---\n");
        fprintf(stderr, "PS=0x%08X  SAR=%u  LBEG=0x%08X  LEND=0x%08X  LCOUNT=%u\n",
                cpu.ps, cpu.sar, cpu.lbeg, cpu.lend, cpu.lcount);
        fprintf(stderr, "WINDOWBASE=%u  WINDOWSTART=0x%04X  VECBASE=0x%08X\n",
                cpu.windowbase, cpu.windowstart, cpu.vecbase);
        fprintf(stderr, "EXCCAUSE=%u  EXCVADDR=0x%08X\n", cpu.exccause, cpu.excvaddr);
        for (int i = 0; i < 16; i += 4)
            fprintf(stderr, "a%d=0x%08X  a%d=0x%08X  a%d=0x%08X  a%d=0x%08X\n",
                    i, ar_read(&cpu, i), i+1, ar_read(&cpu, i+1),
                    i+2, ar_read(&cpu, i+2), i+3, ar_read(&cpu, i+3));
    }

    rom_stubs_destroy(rom);
    periph_destroy(periph);
    mem_destroy(mem);
    return 0;
}
