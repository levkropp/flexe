#include "rom_stubs.h"
#include "memory.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#define MAX_ROM_STUBS 64
#define OUTPUT_BUF_SIZE 8192

/* ROM address range: 0x40000000 - 0x4005FFFF */
#define ROM_BASE 0x40000000u
#define ROM_END  0x40060000u

/* ESP32 UART0 TX FIFO register */
#define UART0_FIFO 0x3FF40000u

typedef struct {
    uint32_t    addr;
    rom_stub_fn fn;
    const char *name;
} rom_stub_entry_t;

struct esp32_rom_stubs {
    xtensa_cpu_t    *cpu;
    rom_stub_entry_t entries[MAX_ROM_STUBS];
    int              count;
    char             output[OUTPUT_BUF_SIZE];
    int              output_len;
    uint32_t         cpu_freq_mhz;
};

/* ===== Calling convention helpers ===== */

static uint32_t rom_arg(xtensa_cpu_t *cpu, int n) {
    int ci = XT_PS_CALLINC(cpu->ps);
    return ar_read(cpu, ci * 4 + 2 + n);
}

static void rom_return(xtensa_cpu_t *cpu, uint32_t retval) {
    int ci = XT_PS_CALLINC(cpu->ps);
    if (ci > 0) {
        ar_write(cpu, ci * 4 + 2, retval);
        uint32_t a0 = ar_read(cpu, ci * 4);
        cpu->pc = (cpu->pc & 0xC0000000u) | (a0 & 0x3FFFFFFFu);
        XT_PS_SET_CALLINC(cpu->ps, 0);
    } else {
        ar_write(cpu, 2, retval);
        cpu->pc = ar_read(cpu, 0);
    }
}

static void rom_return_void(xtensa_cpu_t *cpu) {
    int ci = XT_PS_CALLINC(cpu->ps);
    if (ci > 0) {
        uint32_t a0 = ar_read(cpu, ci * 4);
        cpu->pc = (cpu->pc & 0xC0000000u) | (a0 & 0x3FFFFFFFu);
        XT_PS_SET_CALLINC(cpu->ps, 0);
    } else {
        cpu->pc = ar_read(cpu, 0);
    }
}

/* ===== Output buffer helpers ===== */

static void output_char(esp32_rom_stubs_t *s, char c) {
    if (s->output_len < OUTPUT_BUF_SIZE - 1) {
        s->output[s->output_len++] = c;
        s->output[s->output_len] = '\0';
    }
    /* Also write to UART FIFO so it flows through UART TX callback */
    mem_write32(s->cpu->mem, UART0_FIFO, (uint8_t)c);
}

/* ===== Mini-printf for ets_printf ===== */

static void mini_printf(esp32_rom_stubs_t *s, xtensa_cpu_t *cpu) {
    uint32_t fmt_addr = rom_arg(cpu, 0);
    int argn = 1;  /* next variadic arg index */

    for (;;) {
        uint8_t ch = mem_read8(cpu->mem, fmt_addr++);
        if (ch == 0) break;

        if (ch != '%') {
            output_char(s, (char)ch);
            continue;
        }

        /* Parse format specifier */
        ch = mem_read8(cpu->mem, fmt_addr++);
        if (ch == 0) break;

        if (ch == '%') {
            output_char(s, '%');
            continue;
        }

        /* Parse flags */
        char pad_char = ' ';
        if (ch == '0') {
            pad_char = '0';
            ch = mem_read8(cpu->mem, fmt_addr++);
            if (ch == 0) break;
        }

        /* Parse width */
        int width = 0;
        while (ch >= '0' && ch <= '9') {
            width = width * 10 + (ch - '0');
            ch = mem_read8(cpu->mem, fmt_addr++);
            if (ch == 0) goto done;
        }

        /* Skip 'l' length modifier */
        if (ch == 'l') {
            ch = mem_read8(cpu->mem, fmt_addr++);
            if (ch == 0) break;
        }

        uint32_t val = rom_arg(cpu, argn++);

        char numbuf[12];
        int numlen = 0;

        switch (ch) {
        case 'd':
        case 'i': {
            int32_t sv = (int32_t)val;
            int neg = 0;
            if (sv < 0) { neg = 1; sv = -sv; }
            uint32_t uv = (uint32_t)sv;
            if (uv == 0) numbuf[numlen++] = '0';
            else while (uv > 0) { numbuf[numlen++] = '0' + (uv % 10); uv /= 10; }
            /* Pad */
            int total = numlen + neg;
            while (total < width) { output_char(s, pad_char); total++; }
            if (neg) output_char(s, '-');
            for (int i = numlen - 1; i >= 0; i--) output_char(s, numbuf[i]);
            break;
        }
        case 'u': {
            uint32_t uv = val;
            if (uv == 0) numbuf[numlen++] = '0';
            else while (uv > 0) { numbuf[numlen++] = '0' + (uv % 10); uv /= 10; }
            int total = numlen;
            while (total < width) { output_char(s, pad_char); total++; }
            for (int i = numlen - 1; i >= 0; i--) output_char(s, numbuf[i]);
            break;
        }
        case 'x':
        case 'X':
        case 'p': {
            const char *hexdig = (ch == 'X') ? "0123456789ABCDEF" : "0123456789abcdef";
            uint32_t uv = val;
            if (uv == 0) numbuf[numlen++] = '0';
            else while (uv > 0) { numbuf[numlen++] = hexdig[uv & 0xF]; uv >>= 4; }
            int total = numlen;
            while (total < width) { output_char(s, pad_char); total++; }
            for (int i = numlen - 1; i >= 0; i--) output_char(s, numbuf[i]);
            break;
        }
        case 's': {
            /* Read string from emulator memory */
            uint32_t saddr = val;
            int slen = 0;
            /* Count length first for padding */
            uint32_t tmp = saddr;
            while (mem_read8(cpu->mem, tmp) != 0) { slen++; tmp++; }
            while (slen < width) { output_char(s, ' '); slen++; }
            while (1) {
                uint8_t c = mem_read8(cpu->mem, saddr++);
                if (c == 0) break;
                output_char(s, (char)c);
            }
            break;
        }
        case 'c':
            output_char(s, (char)(val & 0xFF));
            break;
        default:
            output_char(s, '%');
            output_char(s, (char)ch);
            break;
        }
    }
done:;
}

/* ===== ROM function stubs ===== */

static void stub_ets_write_char_uart(xtensa_cpu_t *cpu, void *ctx) {
    esp32_rom_stubs_t *s = ctx;
    uint32_t ch = rom_arg(cpu, 0);
    output_char(s, (char)(ch & 0xFF));
    rom_return(cpu, 0);
}

static void stub_ets_printf(xtensa_cpu_t *cpu, void *ctx) {
    esp32_rom_stubs_t *s = ctx;
    int before = s->output_len;
    mini_printf(s, cpu);
    int written = s->output_len - before;
    rom_return(cpu, (uint32_t)written);
}

static void stub_ets_install_putc1(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    rom_return_void(cpu);
}

static void stub_ets_delay_us(xtensa_cpu_t *cpu, void *ctx) {
    esp32_rom_stubs_t *s = ctx;
    uint32_t us = rom_arg(cpu, 0);
    cpu->ccount += us * s->cpu_freq_mhz;
    rom_return_void(cpu);
}

static void stub_cache_read_enable(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    rom_return_void(cpu);
}

static void stub_cache_read_disable(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    rom_return_void(cpu);
}

static void stub_cache_flush(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    rom_return_void(cpu);
}

static void stub_ets_efuse_get_spiconfig(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    rom_return(cpu, 0);
}

static void stub_ets_get_detected_xtal_freq(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    rom_return(cpu, 40000000);
}

static void stub_software_reset(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    cpu->running = false;
}

static void stub_ets_update_cpu_frequency(xtensa_cpu_t *cpu, void *ctx) {
    esp32_rom_stubs_t *s = ctx;
    s->cpu_freq_mhz = rom_arg(cpu, 0);
    rom_return_void(cpu);
}

static void stub_memcpy(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    uint32_t dst = rom_arg(cpu, 0);
    uint32_t src = rom_arg(cpu, 1);
    uint32_t len = rom_arg(cpu, 2);
    for (uint32_t i = 0; i < len; i++)
        mem_write8(cpu->mem, dst + i, mem_read8(cpu->mem, src + i));
    rom_return(cpu, dst);
}

static void stub_memset(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    uint32_t dst = rom_arg(cpu, 0);
    uint32_t val = rom_arg(cpu, 1);
    uint32_t len = rom_arg(cpu, 2);
    for (uint32_t i = 0; i < len; i++)
        mem_write8(cpu->mem, dst + i, (uint8_t)(val & 0xFF));
    rom_return(cpu, dst);
}

static void stub_strlen(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    uint32_t addr = rom_arg(cpu, 0);
    uint32_t len = 0;
    while (mem_read8(cpu->mem, addr + len) != 0) len++;
    rom_return(cpu, len);
}

/* ===== PC hook ===== */

static int rom_pc_hook(xtensa_cpu_t *cpu, uint32_t pc, void *ctx) {
    if (pc < ROM_BASE || pc >= ROM_END) return 0;
    esp32_rom_stubs_t *s = ctx;
    for (int i = 0; i < s->count; i++) {
        if (s->entries[i].addr == pc) {
            s->entries[i].fn(cpu, s);
            return 1;
        }
    }
    return 0;
}

/* ===== Public API ===== */

esp32_rom_stubs_t *rom_stubs_create(xtensa_cpu_t *cpu) {
    esp32_rom_stubs_t *s = calloc(1, sizeof(*s));
    if (!s) return NULL;
    s->cpu = cpu;
    s->cpu_freq_mhz = 160;

    /* Install PC hook */
    cpu->pc_hook = rom_pc_hook;
    cpu->pc_hook_ctx = s;

    /* Register built-in stubs */
    rom_stubs_register(s, 0x40007cf8, stub_ets_write_char_uart, "ets_write_char_uart");
    rom_stubs_register(s, 0x40007d54, stub_ets_printf,          "ets_printf");
    rom_stubs_register(s, 0x40007d18, stub_ets_install_putc1,   "ets_install_putc1");
    rom_stubs_register(s, 0x40008534, stub_ets_delay_us,        "ets_delay_us");
    rom_stubs_register(s, 0x40009a84, stub_cache_read_enable,   "Cache_Read_Enable");
    rom_stubs_register(s, 0x40009ab8, stub_cache_read_disable,  "Cache_Read_Disable");
    rom_stubs_register(s, 0x40009a14, stub_cache_flush,         "Cache_Flush");
    rom_stubs_register(s, 0x40008658, stub_ets_efuse_get_spiconfig, "ets_efuse_get_spiconfig");
    rom_stubs_register(s, 0x40008588, stub_ets_get_detected_xtal_freq, "ets_get_detected_xtal_freq");
    rom_stubs_register(s, 0x4000824c, stub_software_reset,      "software_reset");
    rom_stubs_register(s, 0x400081d4, stub_ets_update_cpu_frequency, "ets_update_cpu_frequency");
    rom_stubs_register(s, 0x4000c2c8, stub_memcpy,              "memcpy");
    rom_stubs_register(s, 0x4000c3c4, stub_memset,              "memset");
    rom_stubs_register(s, 0x4000c398, stub_strlen,              "strlen");

    return s;
}

void rom_stubs_destroy(esp32_rom_stubs_t *stubs) {
    if (!stubs) return;
    /* Unhook */
    if (stubs->cpu->pc_hook == rom_pc_hook) {
        stubs->cpu->pc_hook = NULL;
        stubs->cpu->pc_hook_ctx = NULL;
    }
    free(stubs);
}

int rom_stubs_register(esp32_rom_stubs_t *stubs, uint32_t addr,
                       rom_stub_fn fn, const char *name) {
    if (stubs->count >= MAX_ROM_STUBS) return -1;
    stubs->entries[stubs->count].addr = addr;
    stubs->entries[stubs->count].fn = fn;
    stubs->entries[stubs->count].name = name;
    stubs->count++;
    return 0;
}

int rom_stubs_output_count(const esp32_rom_stubs_t *stubs) {
    return stubs->output_len;
}

const char *rom_stubs_output_buf(const esp32_rom_stubs_t *stubs) {
    return stubs->output;
}

void rom_stubs_output_clear(esp32_rom_stubs_t *stubs) {
    stubs->output_len = 0;
    stubs->output[0] = '\0';
}
