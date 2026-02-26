#include "xtensa.h"
#include "memory.h"
#include "loader.h"
#include "elf_symbols.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static void usage(void) {
    fprintf(stderr,
        "Usage: xt-dis [options] <binary>\n"
        "\n"
        "Disassemble Xtensa binary files.\n"
        "\n"
        "Input formats:\n"
        "  ESP32 .bin image  Segments are loaded at their proper addresses\n"
        "  Raw binary blob   Loaded at base_addr (default 0x40080000)\n"
        "\n"
        "Options:\n"
        "  -s <elf>          Load ELF symbols (enables function-name disassembly)\n"
        "  -f <name>         Disassemble function by name (requires -s)\n"
        "  -a <addr>         Start disassembly at address (hex, e.g. 0x400D798C)\n"
        "  -n <count>        Number of instructions to disassemble (default: 100)\n"
        "  -b <base>         Base address for raw binary (default: 0x40080000)\n"
        "  -r                Raw mode: treat input as flat binary, not ESP32 image\n"
        "\n"
        "Examples:\n"
        "  xt-dis firmware.bin                           # Disassemble from entry point\n"
        "  xt-dis -s firmware.elf firmware.bin -f app_main\n"
        "  xt-dis -s firmware.elf firmware.bin -a 0x400D798C -n 50\n"
        "  xt-dis -r -b 0x40080000 raw_code.bin\n"
    );
}

static void disasm_range(xtensa_cpu_t *cpu, const elf_symbols_t *syms,
                         uint32_t start, uint32_t end, int max_insns) {
    uint32_t addr = start;
    char buf[128];
    int count = 0;

    while (addr < end && count < max_insns) {
        const uint8_t *ptr = mem_get_ptr(cpu->mem, addr);
        if (!ptr) {
            printf("0x%08X: (unmapped)\n", addr);
            break;
        }

        /* Show symbol label at function boundaries */
        if (syms) {
            elf_sym_info_t info;
            if (elf_symbols_lookup(syms, addr, &info) && info.offset == 0) {
                printf("\n%s:\n", info.name);
            }
        }

        int ilen = xtensa_disasm(cpu, addr, buf, sizeof(buf));

        /* Format: addr [+offset]: raw_bytes  mnemonic */
        printf("0x%08X", addr);
        if (syms) {
            elf_sym_info_t info;
            if (elf_symbols_lookup(syms, addr, &info)) {
                printf(" <%s+0x%x>", info.name, info.offset);
            }
        }
        printf(": ");
        for (int i = 0; i < ilen; i++)
            printf("%02x ", ptr[i]);
        if (ilen == 2) printf("   ");
        printf("\t%s\n", buf);

        addr += (uint32_t)ilen;
        count++;
    }
}

int main(int argc, char *argv[]) {
    const char *elf_path = NULL;
    const char *func_name = NULL;
    uint32_t start_addr = 0;
    int has_start_addr = 0;
    int max_insns = 100;
    uint32_t raw_base = 0x40080000;
    int raw_mode = 0;

    int opt_idx = 1;
    while (opt_idx < argc && argv[opt_idx][0] == '-') {
        const char *flag = argv[opt_idx];
        if (strcmp(flag, "-s") == 0 && opt_idx + 1 < argc) {
            elf_path = argv[++opt_idx];
        } else if (strcmp(flag, "-f") == 0 && opt_idx + 1 < argc) {
            func_name = argv[++opt_idx];
        } else if (strcmp(flag, "-a") == 0 && opt_idx + 1 < argc) {
            start_addr = (uint32_t)strtoul(argv[++opt_idx], NULL, 0);
            has_start_addr = 1;
        } else if (strcmp(flag, "-n") == 0 && opt_idx + 1 < argc) {
            max_insns = atoi(argv[++opt_idx]);
        } else if (strcmp(flag, "-b") == 0 && opt_idx + 1 < argc) {
            raw_base = (uint32_t)strtoul(argv[++opt_idx], NULL, 0);
        } else if (strcmp(flag, "-r") == 0) {
            raw_mode = 1;
        } else if (strcmp(flag, "-h") == 0 || strcmp(flag, "--help") == 0) {
            usage();
            return 0;
        } else {
            fprintf(stderr, "Unknown option: %s\n", flag);
            usage();
            return 1;
        }
        opt_idx++;
    }

    if (opt_idx >= argc) {
        usage();
        return 1;
    }
    const char *bin_path = argv[opt_idx];

    /* Create CPU + memory */
    xtensa_cpu_t cpu;
    xtensa_cpu_init(&cpu);
    cpu.mem = mem_create();
    if (!cpu.mem) {
        fprintf(stderr, "Failed to create memory\n");
        return 1;
    }

    uint32_t entry_point = raw_base;

    if (raw_mode) {
        /* Raw binary: load at base address */
        FILE *f = fopen(bin_path, "rb");
        if (!f) {
            fprintf(stderr, "Cannot open: %s\n", bin_path);
            return 1;
        }
        fseek(f, 0, SEEK_END);
        long size = ftell(f);
        fseek(f, 0, SEEK_SET);
        uint8_t *data = malloc(size);
        if (!data || fread(data, 1, size, f) != (size_t)size) {
            fprintf(stderr, "Read error\n");
            fclose(f);
            return 1;
        }
        fclose(f);
        if (mem_load(cpu.mem, raw_base, data, size) != 0) {
            fprintf(stderr, "Failed to load at 0x%08X\n", raw_base);
            free(data);
            return 1;
        }
        free(data);
    } else {
        /* ESP32 .bin image: use segment loader */
        load_result_t res = loader_load_bin(cpu.mem, bin_path);
        if (res.result != 0) {
            fprintf(stderr, "Failed to load %s: %s\n", bin_path, res.error);
            return 1;
        }
        entry_point = res.entry_point;
        fprintf(stderr, "Loaded %s: %d segments, entry=0x%08X\n",
                bin_path, res.segment_count, entry_point);
        for (int i = 0; i < res.segment_count; i++) {
            fprintf(stderr, "  Segment %d: 0x%08X (%u bytes) -> %s\n",
                    i, res.segments[i].addr, res.segments[i].size,
                    loader_region_name(res.segments[i].addr));
        }
    }

    /* Load ELF symbols if provided */
    elf_symbols_t *syms = NULL;
    if (elf_path) {
        syms = elf_symbols_load(elf_path);
        if (!syms) {
            fprintf(stderr, "Warning: failed to load symbols from %s\n", elf_path);
        } else {
            fprintf(stderr, "Loaded %d symbols from %s\n",
                    elf_symbols_count(syms), elf_path);
        }
    }

    /* Determine disassembly range */
    uint32_t dis_start, dis_end;

    if (func_name) {
        if (!syms) {
            fprintf(stderr, "Error: -f requires -s <elf>\n");
            return 1;
        }
        uint32_t faddr;
        if (elf_symbols_find(syms, func_name, &faddr) != 0) {
            fprintf(stderr, "Symbol not found: %s\n", func_name);
            return 1;
        }
        elf_sym_info_t info;
        elf_symbols_lookup(syms, faddr, &info);
        dis_start = faddr;
        dis_end = (info.size > 0) ? faddr + info.size : faddr + 0x10000;
        max_insns = (info.size > 0) ? (int)(info.size / 2) : max_insns;
        fprintf(stderr, "Disassembling %s at 0x%08X (size=%u)\n",
                func_name, faddr, info.size);
    } else if (has_start_addr) {
        dis_start = start_addr;
        dis_end = start_addr + 0x10000; /* large enough, limited by max_insns */
    } else {
        dis_start = entry_point;
        dis_end = entry_point + 0x10000;
    }

    disasm_range(&cpu, syms, dis_start, dis_end, max_insns);

    if (syms) elf_symbols_destroy(syms);
    mem_destroy(cpu.mem);
    return 0;
}
