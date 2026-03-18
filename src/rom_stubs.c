#include "rom_stubs.h"
#include "elf_symbols.h"
#include "memory.h"
#include "peripherals.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <zlib.h>

#define MAX_ROM_STUBS 1024
#define OUTPUT_BUF_SIZE 8192
#define HOOK_HT_SIZE  2048
#define HOOK_HT_MASK  (HOOK_HT_SIZE - 1)

/* ROM address range: 0x40000000 - 0x4005FFFF */
#define ROM_BASE 0x40000000u
#define ROM_END  0x40070000u   /* includes SPI flash ROM at 0x4006xxxx */

/* ESP32 UART0 TX FIFO register */
#define UART0_FIFO 0x3FF40000u

typedef struct {
    uint32_t    addr;
    rom_stub_fn fn;
    const char *name;
    uint32_t    call_count;
    void       *user_ctx;   /* Per-entry context; NULL = use rom_stubs */
    int         spy;        /* If true: call fn, then let original execute */
} rom_stub_entry_t;

struct esp32_rom_stubs {
    xtensa_cpu_t    *cpu;
    rom_stub_entry_t entries[MAX_ROM_STUBS];
    int              count;
    char             output[OUTPUT_BUF_SIZE];
    int              output_len;
    uint32_t         cpu_freq_mhz;
    rom_log_fn       log_fn;
    void            *log_ctx;
    int              unregistered_count;
    uint32_t         total_calls;       /* running counter for heartbeat */
    uint32_t         s_cpu_up_addr;     /* BSS symbol for multicore unblock */
    uint32_t         s_cpu_inited_addr; /* BSS symbol for multicore init wait */
    uint32_t         s_system_inited_addr;      /* system init complete flag */
    uint32_t         s_system_full_inited_addr; /* system full init complete flag */
    uint32_t         app_main_addr;    /* app_main symbol for start_cpu0 hook */
    uint32_t         stack_chk_guard_addr; /* __stack_chk_guard BSS address */
    const elf_symbols_t *syms;         /* ELF symbols (for symbol lookups in stubs) */
    uint32_t         s_other_cpu_startup_done_addr; /* main_task polling flag */
    uint32_t         s_resume_cores_addr;      /* s_resume_cores BSS address */
    uint32_t         app_cpu_boot_addr;        /* Boot address for APP CPU (core 1) */
    bool             app_cpu_start_requested;  /* Core 0 requested core 1 start */
    bool             single_core_mode;         /* -1 flag: fake core 1 init variables */
    bool             native_freertos;         /* -N flag: skip interrupt/lock stubs */
    esp32_periph_t  *periph;                 /* Peripheral state (for intr_matrix_set) */
    struct {
        uint32_t addr;     /* 0 = empty */
        int      idx;      /* index into entries[] */
    } ht[HOOK_HT_SIZE];
    uint64_t hook_bitmap[HOOK_BITMAP_WORDS];
    stub_direct_entry_t *direct;  /* Direct dispatch table (heap-allocated, 64K entries) */
};

/* ===== Calling convention helpers ===== */

static uint32_t rom_arg(xtensa_cpu_t *cpu, int n) {
    int ci = XT_PS_CALLINC(cpu->ps);
    int reg = ci * 4 + 2 + n;
    if (reg < 16) {
        return ar_read(cpu, reg);
    }
    /* Overflow: arg doesn't fit in caller's 16-register window.
     * Compiler stores overflow args on caller's stack at [SP + k*4].
     * Caller's SP is a1 (ar[1] in current window). */
    int overflow_idx = reg - 16;
    uint32_t caller_sp = ar_read(cpu, 1);
    return mem_read32(cpu->mem, caller_sp + overflow_idx * 4);
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

static void rom_return64(xtensa_cpu_t *cpu, uint64_t retval) {
    int ci = XT_PS_CALLINC(cpu->ps);
    if (ci > 0) {
        ar_write(cpu, ci * 4 + 2, (uint32_t)retval);
        ar_write(cpu, ci * 4 + 3, (uint32_t)(retval >> 32));
        uint32_t a0 = ar_read(cpu, ci * 4);
        cpu->pc = (cpu->pc & 0xC0000000u) | (a0 & 0x3FFFFFFFu);
        XT_PS_SET_CALLINC(cpu->ps, 0);
    } else {
        ar_write(cpu, 2, (uint32_t)retval);
        ar_write(cpu, 3, (uint32_t)(retval >> 32));
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
        int left_justify = 0;
        if (ch == '-') {
            left_justify = 1;
            ch = mem_read8(cpu->mem, fmt_addr++);
            if (ch == 0) break;
        }
        if (ch == '0' && !left_justify) {
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

        /* Parse precision (skip) */
        if (ch == '.') {
            ch = mem_read8(cpu->mem, fmt_addr++);
            while (ch >= '0' && ch <= '9') {
                ch = mem_read8(cpu->mem, fmt_addr++);
            }
            if (ch == 0) break;
        }

        /* Parse length modifier: l, ll, h, hh, z */
        int is_long_long = 0;
        if (ch == 'l') {
            ch = mem_read8(cpu->mem, fmt_addr++);
            if (ch == 0) break;
            if (ch == 'l') {
                is_long_long = 1;
                ch = mem_read8(cpu->mem, fmt_addr++);
                if (ch == 0) break;
            }
        } else if (ch == 'h') {
            ch = mem_read8(cpu->mem, fmt_addr++);
            if (ch == 0) break;
            if (ch == 'h') {
                ch = mem_read8(cpu->mem, fmt_addr++);
                if (ch == 0) break;
            }
        } else if (ch == 'z') {
            ch = mem_read8(cpu->mem, fmt_addr++);
            if (ch == 0) break;
        }

        /* Read value — 64-bit for ll, 32-bit otherwise.
         * On Xtensa, 64-bit args are passed in a register pair (even-aligned). */
        uint64_t val64 = 0;
        uint32_t val = 0;
        if (is_long_long) {
            /* 64-bit: align argn to even, then read two 32-bit halves */
            if (argn % 2 != 0) argn++;
            uint32_t lo = rom_arg(cpu, argn++);
            uint32_t hi = rom_arg(cpu, argn++);
            val64 = ((uint64_t)hi << 32) | lo;
            val = (uint32_t)val64;
        } else {
            val = rom_arg(cpu, argn++);
            val64 = val;
        }

        char numbuf[24]; /* enough for 64-bit decimal */
        int numlen = 0;

        switch (ch) {
        case 'd':
        case 'i': {
            int neg = 0;
            uint64_t uv;
            if (is_long_long) {
                int64_t sv = (int64_t)val64;
                if (sv < 0) { neg = 1; sv = -sv; }
                uv = (uint64_t)sv;
            } else {
                int32_t sv = (int32_t)val;
                if (sv < 0) { neg = 1; sv = -sv; }
                uv = (uint32_t)sv;
            }
            if (uv == 0) numbuf[numlen++] = '0';
            else while (uv > 0) { numbuf[numlen++] = '0' + (int)(uv % 10); uv /= 10; }
            int total = numlen + neg;
            if (!left_justify)
                while (total < width) { output_char(s, pad_char); total++; }
            if (neg) output_char(s, '-');
            for (int i = numlen - 1; i >= 0; i--) output_char(s, numbuf[i]);
            if (left_justify)
                while (total < width) { output_char(s, ' '); total++; }
            break;
        }
        case 'u': {
            uint64_t uv = is_long_long ? val64 : val;
            if (uv == 0) numbuf[numlen++] = '0';
            else while (uv > 0) { numbuf[numlen++] = '0' + (int)(uv % 10); uv /= 10; }
            int total = numlen;
            if (!left_justify)
                while (total < width) { output_char(s, pad_char); total++; }
            for (int i = numlen - 1; i >= 0; i--) output_char(s, numbuf[i]);
            if (left_justify)
                while (total < width) { output_char(s, ' '); total++; }
            break;
        }
        case 'x':
        case 'X':
        case 'p': {
            const char *hexdig = (ch == 'X') ? "0123456789ABCDEF" : "0123456789abcdef";
            uint64_t uv = is_long_long ? val64 : val;
            if (uv == 0) numbuf[numlen++] = '0';
            else while (uv > 0) { numbuf[numlen++] = hexdig[uv & 0xF]; uv >>= 4; }
            int total = numlen;
            if (!left_justify)
                while (total < width) { output_char(s, pad_char); total++; }
            for (int i = numlen - 1; i >= 0; i--) output_char(s, numbuf[i]);
            if (left_justify)
                while (total < width) { output_char(s, ' '); total++; }
            break;
        }
        case 's': {
            /* Read string from emulator memory */
            uint32_t saddr = val;
            int slen = 0;
            /* Count length first for padding */
            uint32_t tmp = saddr;
            while (mem_read8(cpu->mem, tmp) != 0) { slen++; tmp++; }
            if (!left_justify)
                while (slen < width) { output_char(s, ' '); slen++; }
            int printed = 0;
            while (1) {
                uint8_t c = mem_read8(cpu->mem, saddr++);
                if (c == 0) break;
                output_char(s, (char)c);
                printed++;
            }
            if (left_justify)
                while (printed < width) { output_char(s, ' '); printed++; }
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

/* ===== Boot-sequence ROM stubs ===== */

static void stub_ets_set_appcpu_boot_addr(xtensa_cpu_t *cpu, void *ctx) {
    esp32_rom_stubs_t *s = ctx;
    uint32_t addr = rom_arg(cpu, 0);
    s->app_cpu_boot_addr = addr;

    /* Always set s_cpu_up[1] = 1 (tells core 0 that core 1 is alive) */
    if (s->s_cpu_up_addr)
        mem_write8(cpu->mem, s->s_cpu_up_addr + 1, 1);

    if (s->single_core_mode) {
        /* Single-core: fake all init variables so core 0 doesn't hang */
        if (s->s_cpu_inited_addr)
            mem_write8(cpu->mem, s->s_cpu_inited_addr + 1, 1);
        if (s->s_system_inited_addr) {
            mem_write8(cpu->mem, s->s_system_inited_addr, 1);
            mem_write8(cpu->mem, s->s_system_inited_addr + 1, 1);
        }
        if (s->s_system_full_inited_addr) {
            mem_write8(cpu->mem, s->s_system_full_inited_addr, 1);
            mem_write8(cpu->mem, s->s_system_full_inited_addr + 1, 1);
        }
    }
    /* Dual-core: core 1 will set its own init variables when it starts */
    rom_return_void(cpu);
}

static void stub_rtc_get_reset_reason(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    rom_return(cpu, 1);  /* POWERON_RESET */
}

static void stub_ets_install_uart_printf(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    rom_return_void(cpu);
}

static void stub_memmove(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    uint32_t dst = rom_arg(cpu, 0);
    uint32_t src = rom_arg(cpu, 1);
    uint32_t len = rom_arg(cpu, 2);
    /* memmove handles overlapping: copy via temp buffer approach */
    if (dst < src || dst >= src + len) {
        /* Forward copy (no overlap or dst before src) */
        for (uint32_t i = 0; i < len; i++)
            mem_write8(cpu->mem, dst + i, mem_read8(cpu->mem, src + i));
    } else {
        /* Backward copy (overlapping, dst within src range) */
        for (uint32_t i = len; i > 0; i--)
            mem_write8(cpu->mem, dst + i - 1, mem_read8(cpu->mem, src + i - 1));
    }
    rom_return(cpu, dst);
}

static void stub_memcmp(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    uint32_t s1 = rom_arg(cpu, 0);
    uint32_t s2 = rom_arg(cpu, 1);
    uint32_t n  = rom_arg(cpu, 2);
    for (uint32_t i = 0; i < n; i++) {
        uint8_t a = mem_read8(cpu->mem, s1 + i);
        uint8_t b = mem_read8(cpu->mem, s2 + i);
        if (a != b) {
            rom_return(cpu, (uint32_t)(int32_t)(a - b));
            return;
        }
    }
    rom_return(cpu, 0);
}

static void stub_bzero(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    uint32_t dst = rom_arg(cpu, 0);
    uint32_t len = rom_arg(cpu, 1);
    for (uint32_t i = 0; i < len; i++)
        mem_write8(cpu->mem, dst + i, 0);
    rom_return_void(cpu);
}

/* ===== Native-accelerated string/memory stubs =====
 * Resolve guest addresses to host pointers and use native libc.
 * Page-boundary-safe: processes in page-sized chunks. */

static void stub_strcmp(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    uint32_t s1 = rom_arg(cpu, 0);
    uint32_t s2 = rom_arg(cpu, 1);
    /* Fast path: compare via host pointers */
    while (1) {
        const uint8_t *p1 = mem_get_ptr(cpu->mem, s1);
        const uint8_t *p2 = mem_get_ptr(cpu->mem, s2);
        if (p1 && p2) {
            uint32_t remain1 = 0x1000 - (s1 & 0xFFF);
            uint32_t remain2 = 0x1000 - (s2 & 0xFFF);
            uint32_t chunk = remain1 < remain2 ? remain1 : remain2;
            for (uint32_t i = 0; i < chunk; i++) {
                if (p1[i] != p2[i] || p1[i] == 0) {
                    rom_return(cpu, (uint32_t)(int32_t)(p1[i] - p2[i]));
                    return;
                }
            }
            s1 += chunk;
            s2 += chunk;
        } else {
            uint8_t a = mem_read8(cpu->mem, s1++);
            uint8_t b = mem_read8(cpu->mem, s2++);
            if (a != b || a == 0) {
                rom_return(cpu, (uint32_t)(int32_t)(a - b));
                return;
            }
        }
    }
}

static void stub_strcpy(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    uint32_t dst = rom_arg(cpu, 0);
    uint32_t src = rom_arg(cpu, 1);
    uint32_t off = 0;
    while (1) {
        const uint8_t *sp = mem_get_ptr(cpu->mem, src + off);
        uint8_t *dp = mem_get_ptr_w(cpu->mem, dst + off);
        if (sp && dp) {
            uint32_t rem_s = 0x1000 - ((src + off) & 0xFFF);
            uint32_t rem_d = 0x1000 - ((dst + off) & 0xFFF);
            uint32_t chunk = rem_s < rem_d ? rem_s : rem_d;
            const uint8_t *nul = memchr(sp, 0, chunk);
            if (nul) {
                uint32_t n = (uint32_t)(nul - sp) + 1; /* include NUL */
                memcpy(dp, sp, n);
                break;
            }
            memcpy(dp, sp, chunk);
            off += chunk;
        } else {
            uint8_t c = mem_read8(cpu->mem, src + off);
            mem_write8(cpu->mem, dst + off, c);
            if (c == 0) break;
            off++;
        }
    }
    rom_return(cpu, dst);
}

static void stub_strncpy(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    uint32_t dst = rom_arg(cpu, 0);
    uint32_t src = rom_arg(cpu, 1);
    uint32_t n   = rom_arg(cpu, 2);
    uint32_t off = 0;
    int pad = 0;
    while (off < n) {
        uint8_t *dp = mem_get_ptr_w(cpu->mem, dst + off);
        uint32_t rem_d = 0x1000 - ((dst + off) & 0xFFF);
        uint32_t chunk = n - off;
        if (chunk > rem_d) chunk = rem_d;
        if (pad) {
            if (dp) { memset(dp, 0, chunk); off += chunk; }
            else { mem_write8(cpu->mem, dst + off, 0); off++; }
        } else {
            const uint8_t *sp = mem_get_ptr(cpu->mem, src + off);
            uint32_t rem_s = 0x1000 - ((src + off) & 0xFFF);
            if (chunk > rem_s) chunk = rem_s;
            if (dp && sp) {
                const uint8_t *nul = memchr(sp, 0, chunk);
                if (nul) {
                    uint32_t pre = (uint32_t)(nul - sp) + 1;
                    memcpy(dp, sp, pre);
                    off += pre;
                    pad = 1;
                } else {
                    memcpy(dp, sp, chunk);
                    off += chunk;
                }
            } else {
                uint8_t c = mem_read8(cpu->mem, src + off);
                mem_write8(cpu->mem, dst + off, c);
                if (c == 0) pad = 1;
                off++;
            }
        }
    }
    rom_return(cpu, dst);
}

static void stub_strlcpy(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    uint32_t dst  = rom_arg(cpu, 0);
    uint32_t src  = rom_arg(cpu, 1);
    uint32_t size = rom_arg(cpu, 2);
    /* Count total source length using native strlen stub */
    uint32_t slen = 0;
    while (1) {
        const uint8_t *p = mem_get_ptr(cpu->mem, src + slen);
        if (p) {
            uint32_t rem = 0x1000 - ((src + slen) & 0xFFF);
            const uint8_t *nul = memchr(p, 0, rem);
            if (nul) { slen += (uint32_t)(nul - p); break; }
            slen += rem;
        } else {
            if (mem_read8(cpu->mem, src + slen) == 0) break;
            slen++;
        }
    }
    /* Copy up to size-1 chars using native memcpy */
    if (size > 0) {
        uint32_t copy = (slen < size - 1) ? slen : size - 1;
        uint32_t off = 0;
        while (off < copy) {
            uint32_t rem_d = 0x1000 - ((dst + off) & 0xFFF);
            uint32_t rem_s = 0x1000 - ((src + off) & 0xFFF);
            uint32_t chunk = copy - off;
            if (chunk > rem_d) chunk = rem_d;
            if (chunk > rem_s) chunk = rem_s;
            uint8_t *dp = mem_get_ptr_w(cpu->mem, dst + off);
            const uint8_t *sp = mem_get_ptr(cpu->mem, src + off);
            if (dp && sp) { memcpy(dp, sp, chunk); off += chunk; }
            else { mem_write8(cpu->mem, dst + off, mem_read8(cpu->mem, src + off)); off++; }
        }
        mem_write8(cpu->mem, dst + copy, 0);
    }
    rom_return(cpu, slen);
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
    (void)ctx;
    uint32_t us = rom_arg(cpu, 0);
    cpu->virtual_time_us += us;
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

/* g_ticks_per_us_pro: address where ESP-IDF reads CPU freq (ticks/us)
 * The real ROM function writes to this address. We derive it from the
 * firmware's literal pool, but it's consistently at 0x3FFE01E0 for ESP32. */
#define G_TICKS_PER_US_PRO 0x3FFE01E0u

static void stub_ets_update_cpu_frequency(xtensa_cpu_t *cpu, void *ctx) {
    esp32_rom_stubs_t *s = ctx;
    uint32_t freq = rom_arg(cpu, 0);
    s->cpu_freq_mhz = freq;
    /* Write to the memory location firmware reads via l32r */
    mem_write32(cpu->mem, G_TICKS_PER_US_PRO, freq);
    rom_return_void(cpu);
}

static void stub_memcpy(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    uint32_t dst = rom_arg(cpu, 0);
    uint32_t src = rom_arg(cpu, 1);
    uint32_t len = rom_arg(cpu, 2);
    uint32_t off = 0;
    while (off < len) {
        uint32_t rem_d = 0x1000 - ((dst + off) & 0xFFF);
        uint32_t rem_s = 0x1000 - ((src + off) & 0xFFF);
        uint32_t chunk = len - off;
        if (chunk > rem_d) chunk = rem_d;
        if (chunk > rem_s) chunk = rem_s;
        uint8_t *dp = mem_get_ptr_w(cpu->mem, dst + off);
        const uint8_t *sp = mem_get_ptr(cpu->mem, src + off);
        if (dp && sp) {
            memcpy(dp, sp, chunk);
            off += chunk;
        } else {
            mem_write8(cpu->mem, dst + off, mem_read8(cpu->mem, src + off));
            off++;
        }
    }
    rom_return(cpu, dst);
}

static void stub_memset(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    uint32_t dst = rom_arg(cpu, 0);
    uint32_t val = rom_arg(cpu, 1) & 0xFF;
    uint32_t len = rom_arg(cpu, 2);
    uint32_t off = 0;
    while (off < len) {
        uint32_t rem = 0x1000 - ((dst + off) & 0xFFF);
        uint32_t chunk = len - off;
        if (chunk > rem) chunk = rem;
        uint8_t *dp = mem_get_ptr_w(cpu->mem, dst + off);
        if (dp) {
            memset(dp, (int)val, chunk);
            off += chunk;
        } else {
            mem_write8(cpu->mem, dst + off, (uint8_t)val);
            off++;
        }
    }
    rom_return(cpu, dst);
}

static void stub_strlen(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    uint32_t addr = rom_arg(cpu, 0);
    uint32_t len = 0;
    while (1) {
        const uint8_t *p = mem_get_ptr(cpu->mem, addr + len);
        if (p) {
            uint32_t rem = 0x1000 - ((addr + len) & 0xFFF);
            const uint8_t *nul = memchr(p, 0, rem);
            if (nul) { len += (uint32_t)(nul - p); break; }
            len += rem;
        } else {
            if (mem_read8(cpu->mem, addr + len) == 0) break;
            len++;
        }
    }
    rom_return(cpu, len);
}

/* ===== Compiler builtins ===== */

static void stub_popcountsi2(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    uint32_t v = rom_arg(cpu, 0);
    v = v - ((v >> 1) & 0x55555555u);
    v = (v & 0x33333333u) + ((v >> 2) & 0x33333333u);
    v = (v + (v >> 4)) & 0x0F0F0F0Fu;
    uint32_t result = (v * 0x01010101u) >> 24;
    rom_return(cpu, result);
}

static void stub_popcountdi2(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    uint32_t lo = rom_arg(cpu, 0);
    uint32_t hi = rom_arg(cpu, 1);
    /* popcount of 64-bit: popcount(lo) + popcount(hi) */
    lo = lo - ((lo >> 1) & 0x55555555u);
    lo = (lo & 0x33333333u) + ((lo >> 2) & 0x33333333u);
    lo = (lo + (lo >> 4)) & 0x0F0F0F0Fu;
    uint32_t pclo = (lo * 0x01010101u) >> 24;
    hi = hi - ((hi >> 1) & 0x55555555u);
    hi = (hi & 0x33333333u) + ((hi >> 2) & 0x33333333u);
    hi = (hi + (hi >> 4)) & 0x0F0F0F0Fu;
    uint32_t pchi = (hi * 0x01010101u) >> 24;
    rom_return(cpu, pclo + pchi);
}

static void stub_clzsi2(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    uint32_t v = rom_arg(cpu, 0);
    if (v == 0) { rom_return(cpu, 32); return; }
    uint32_t n = 0;
    if ((v & 0xFFFF0000u) == 0) { n += 16; v <<= 16; }
    if ((v & 0xFF000000u) == 0) { n += 8;  v <<= 8; }
    if ((v & 0xF0000000u) == 0) { n += 4;  v <<= 4; }
    if ((v & 0xC0000000u) == 0) { n += 2;  v <<= 2; }
    if ((v & 0x80000000u) == 0) { n += 1; }
    rom_return(cpu, n);
}

static void stub_ctzsi2(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    uint32_t v = rom_arg(cpu, 0);
    if (v == 0) { rom_return(cpu, 32); return; }
    uint32_t n = 0;
    if ((v & 0x0000FFFFu) == 0) { n += 16; v >>= 16; }
    if ((v & 0x000000FFu) == 0) { n += 8;  v >>= 8; }
    if ((v & 0x0000000Fu) == 0) { n += 4;  v >>= 4; }
    if ((v & 0x00000003u) == 0) { n += 2;  v >>= 2; }
    if ((v & 0x00000001u) == 0) { n += 1; }
    rom_return(cpu, n);
}

static void stub_ffssi2(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    uint32_t v = rom_arg(cpu, 0);
    if (v == 0) { rom_return(cpu, 0); return; }
    uint32_t n = 1;
    if ((v & 0x0000FFFFu) == 0) { n += 16; v >>= 16; }
    if ((v & 0x000000FFu) == 0) { n += 8;  v >>= 8; }
    if ((v & 0x0000000Fu) == 0) { n += 4;  v >>= 4; }
    if ((v & 0x00000003u) == 0) { n += 2;  v >>= 2; }
    if ((v & 0x00000001u) == 0) { n += 1; }
    rom_return(cpu, n);
}

/* 64-bit division/modulo builtins */
static void stub_udivdi3(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    uint64_t a = (uint64_t)rom_arg(cpu, 0) | ((uint64_t)rom_arg(cpu, 1) << 32);
    uint64_t b = (uint64_t)rom_arg(cpu, 2) | ((uint64_t)rom_arg(cpu, 3) << 32);
    rom_return64(cpu, b ? a / b : 0);
}

static void stub_umoddi3(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    uint64_t a = (uint64_t)rom_arg(cpu, 0) | ((uint64_t)rom_arg(cpu, 1) << 32);
    uint64_t b = (uint64_t)rom_arg(cpu, 2) | ((uint64_t)rom_arg(cpu, 3) << 32);
    rom_return64(cpu, b ? a % b : 0);
}

static void stub_divdi3(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    int64_t a = (int64_t)((uint64_t)rom_arg(cpu, 0) | ((uint64_t)rom_arg(cpu, 1) << 32));
    int64_t b = (int64_t)((uint64_t)rom_arg(cpu, 2) | ((uint64_t)rom_arg(cpu, 3) << 32));
    rom_return64(cpu, (uint64_t)(b ? a / b : 0));
}

static void stub_moddi3(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    int64_t a = (int64_t)((uint64_t)rom_arg(cpu, 0) | ((uint64_t)rom_arg(cpu, 1) << 32));
    int64_t b = (int64_t)((uint64_t)rom_arg(cpu, 2) | ((uint64_t)rom_arg(cpu, 3) << 32));
    rom_return64(cpu, (uint64_t)(b ? a % b : 0));
}

/* itoa — integer to string conversion */
static void stub_itoa(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    int32_t val = (int32_t)rom_arg(cpu, 0);
    uint32_t buf = rom_arg(cpu, 1);
    uint32_t base = rom_arg(cpu, 2);
    char tmp[34];
    int neg = 0;
    uint32_t uval;
    if (base == 10 && val < 0) { neg = 1; uval = (uint32_t)(-val); }
    else uval = (uint32_t)val;
    int i = 0;
    if (uval == 0) tmp[i++] = '0';
    else while (uval > 0) {
        uint32_t d = uval % base;
        tmp[i++] = (char)(d < 10 ? '0' + d : 'a' + d - 10);
        uval /= base;
    }
    if (neg) tmp[i++] = '-';
    /* reverse and write to memory */
    for (int j = 0; j < i; j++)
        mem_write8(cpu->mem, buf + (uint32_t)j, (uint8_t)tmp[i - 1 - j]);
    mem_write8(cpu->mem, buf + (uint32_t)i, 0);
    rom_return(cpu, buf);
}

/* strcat */
static void stub_strcat(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    uint32_t dst = rom_arg(cpu, 0);
    uint32_t src = rom_arg(cpu, 1);
    uint32_t end = dst;
    while (mem_read8(cpu->mem, end)) end++;
    uint8_t c;
    do { c = mem_read8(cpu->mem, src++); mem_write8(cpu->mem, end++, c); } while (c);
    rom_return(cpu, dst);
}

/* qsort — in-memory sort comparing first uint32_t of each element */
static void stub_qsort(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    uint32_t base = rom_arg(cpu, 0);
    uint32_t nmemb = rom_arg(cpu, 1);
    uint32_t size = rom_arg(cpu, 2);
    /* arg3 = compar function pointer (ignored, sort by first 4 bytes) */
    if (nmemb <= 1 || size == 0 || size > 256) {
        rom_return_void(cpu);
        return;
    }
    /* Simple insertion sort — elements are small, counts are low */
    uint8_t tmp[256], cur[256];
    for (uint32_t i = 1; i < nmemb; i++) {
        uint32_t cur_addr = base + i * size;
        uint32_t cur_key = mem_read32(cpu->mem, cur_addr);
        for (uint32_t k = 0; k < size; k++)
            cur[k] = mem_read8(cpu->mem, cur_addr + k);
        int j = (int)i - 1;
        while (j >= 0) {
            uint32_t j_addr = base + (uint32_t)j * size;
            uint32_t j_key = mem_read32(cpu->mem, j_addr);
            if (j_key <= cur_key) break;
            /* shift element j to j+1 */
            for (uint32_t k = 0; k < size; k++)
                tmp[k] = mem_read8(cpu->mem, j_addr + k);
            for (uint32_t k = 0; k < size; k++)
                mem_write8(cpu->mem, j_addr + size + k, tmp[k]);
            j--;
        }
        /* insert cur at j+1 */
        uint32_t ins_addr = base + (uint32_t)(j + 1) * size;
        for (uint32_t k = 0; k < size; k++)
            mem_write8(cpu->mem, ins_addr + k, cur[k]);
    }
    rom_return_void(cpu);
}

/*
 * __sinit: Initialize newlib stdio (stdin/stdout/stderr).
 * Sets stdout->_write to point to our UART write stub so printf works.
 * ESP32 layout: _impure_ptr (0x3FFB054C) -> _REENT, _REENT+8 -> _stdout,
 * __sFILE64 layout (ESP32 newlib, 32-bit):
 *   offset 0:  _p          offset 12: _flags (short)
 *   offset 28: _data       offset 32: _cookie
 *   offset 36: _read       offset 40: _write      ← target
 *   offset 44: _seek       offset 48: _close
 */
#define IMPURE_PTR_ADDR   0x3FFB054Cu
#define REENT_STDOUT_OFS  8
#define FILE_WRITE_OFS    40   /* __sFILE64._write */
#define FILE_FLAGS_OFS    12
#define SINIT_WRITE_STUB  0x40001150u  /* We'll hook __swrite as our UART stub */

static void stub_sinit(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    /* Read _impure_ptr to get _REENT address */
    uint32_t reent = mem_read32(cpu->mem, IMPURE_PTR_ADDR);
    if (reent == 0) {
        rom_return_void(cpu);
        return;
    }
    /* Read _stdout pointer from _REENT */
    uint32_t stdout_fp = mem_read32(cpu->mem, reent + REENT_STDOUT_OFS);
    if (stdout_fp == 0) {
        rom_return_void(cpu);
        return;
    }
    /* Set _stdout->_write to our UART write stub address */
    mem_write32(cpu->mem, stdout_fp + FILE_WRITE_OFS, SINIT_WRITE_STUB);
    /* Ensure _stdout flags have __SWR (writable) set */
    uint16_t flags = (uint16_t)mem_read16(cpu->mem, stdout_fp + FILE_FLAGS_OFS);
    flags |= 0x0008;  /* __SWR */
    mem_write16(cpu->mem, stdout_fp + FILE_FLAGS_OFS, flags);
    rom_return_void(cpu);
}

/*
 * __swrite: newlib's default stdout write function.
 * Signature: int __swrite(struct _reent *ptr, void *cookie, const char *buf, int len)
 * a2=reent, a3=cookie, a4=buf, a5=len
 * We write the buffer to our UART output capture.
 */
static void stub_swrite(xtensa_cpu_t *cpu, void *ctx) {
    esp32_rom_stubs_t *s = ctx;
    uint32_t buf = rom_arg(cpu, 2);   /* a4 in caller = buf */
    int32_t len  = (int32_t)rom_arg(cpu, 3);  /* a5 in caller = len */
    if (len <= 0) { rom_return(cpu, 0); return; }
    for (int i = 0; i < len; i++) {
        uint8_t ch = mem_read8(cpu->mem, buf + (uint32_t)i);
        /* Write to UART FIFO for peripheral capture */
        mem_write8(cpu->mem, UART0_FIFO, ch);
        /* Also buffer in output capture */
        if (s->output_len < OUTPUT_BUF_SIZE - 1)
            s->output[s->output_len++] = (char)ch;
    }
    rom_return(cpu, len);
}

/*
 * _fflush_r: flush a FILE's write buffer.
 * Signature: int _fflush_r(struct _reent *ptr, FILE *fp)
 * a2=reent, a3=fp
 * We flush any buffered data to UART and reset the buffer pointers.
 */
#define FILE_P_OFS      0    /* _p: current buffer position */
#define FILE_W_OFS      8    /* _w: write space left */
#define FILE_BF_BASE_OFS 16  /* _bf._base: buffer base */
#define FILE_BF_SIZE_OFS 20  /* _bf._size: buffer size */

static void stub_fflush_r(xtensa_cpu_t *cpu, void *ctx) {
    esp32_rom_stubs_t *s = ctx;
    uint32_t fp = rom_arg(cpu, 1);  /* a3 in caller = FILE* */
    if (fp == 0) { rom_return(cpu, -1); return; }

    uint32_t p     = mem_read32(cpu->mem, fp + FILE_P_OFS);
    uint32_t base  = mem_read32(cpu->mem, fp + FILE_BF_BASE_OFS);
    int32_t  size  = (int32_t)mem_read32(cpu->mem, fp + FILE_BF_SIZE_OFS);

    int32_t buffered = (int32_t)(p - base);
    if (buffered > 0) {
        /* Write buffered data to UART output */
        for (int32_t i = 0; i < buffered; i++) {
            uint8_t ch = mem_read8(cpu->mem, base + (uint32_t)i);
            mem_write8(cpu->mem, UART0_FIFO, ch);
            if (s->output_len < OUTPUT_BUF_SIZE - 1)
                s->output[s->output_len++] = (char)ch;
        }
        /* Reset buffer pointers */
        mem_write32(cpu->mem, fp + FILE_P_OFS, base);
        mem_write32(cpu->mem, fp + FILE_W_OFS, (uint32_t)size);
    }
    rom_return(cpu, 0);
}

/* Helper: read double from arg pair (a2:a3 or a4:a5) */
static double rom_read_double(xtensa_cpu_t *cpu, int arg_pair) {
    uint32_t lo = rom_arg(cpu, arg_pair * 2);
    uint32_t hi = rom_arg(cpu, arg_pair * 2 + 1);
    uint64_t bits = ((uint64_t)hi << 32) | lo;
    double d;
    memcpy(&d, &bits, 8);
    return d;
}

/* Helper: return double via rom_return64 */
static void rom_return_double(xtensa_cpu_t *cpu, double d) {
    uint64_t bits;
    memcpy(&bits, &d, 8);
    rom_return64(cpu, bits);
}

/* __adddf3: double + double */
static void stub_adddf3(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    rom_return_double(cpu, rom_read_double(cpu, 0) + rom_read_double(cpu, 1));
}

/* __subdf3: double - double */
static void stub_subdf3(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    rom_return_double(cpu, rom_read_double(cpu, 0) - rom_read_double(cpu, 1));
}

/* __muldf3: double * double */
static void stub_muldf3(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    rom_return_double(cpu, rom_read_double(cpu, 0) * rom_read_double(cpu, 1));
}

/* __divdf3: double / double */
static void stub_divdf3(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    rom_return_double(cpu, rom_read_double(cpu, 0) / rom_read_double(cpu, 1));
}

/* __floatunsidf: unsigned int → double */
static void stub_floatunsidf(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    rom_return_double(cpu, (double)rom_arg(cpu, 0));
}

/* __floatsidf: signed int → double */
static void stub_floatsidf(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    rom_return_double(cpu, (double)(int32_t)rom_arg(cpu, 0));
}

/* __fixdfsi: double → signed int (truncate toward zero) */
static void stub_fixdfsi(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    rom_return(cpu, (uint32_t)(int32_t)rom_read_double(cpu, 0));
}

/* __fixunsdfsi: double → unsigned int */
static void stub_fixunsdfsi(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    rom_return(cpu, (uint32_t)rom_read_double(cpu, 0));
}

/*
 * GCC soft-float comparison convention:
 * __ledf2: returns <= 0 if a <= b, > 0 otherwise (for LE test)
 * __ltdf2: returns < 0 if a < b, >= 0 otherwise (for LT test)
 * __gedf2: returns >= 0 if a >= b, < 0 otherwise (for GE test)
 * __gtdf2: returns > 0 if a > b, <= 0 otherwise (for GT test)
 * __eqdf2: returns 0 if a == b
 * All return -1/0/1 like strcmp: a<b → -1, a==b → 0, a>b → 1
 * __unorddf2: returns nonzero if either is NaN
 */
static void stub_cmpdf(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    double a = rom_read_double(cpu, 0);
    double b = rom_read_double(cpu, 1);
    int result;
    if (__builtin_isnan(a) || __builtin_isnan(b))
        result = 1;  /* unordered → positive (like a > b for le/lt) */
    else if (a < b)
        result = -1;
    else if (a > b)
        result = 1;
    else
        result = 0;
    rom_return(cpu, (uint32_t)(int32_t)result);
}

/* __unorddf2: returns nonzero if either arg is NaN */
static void stub_unorddf2(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    double a = rom_read_double(cpu, 0);
    double b = rom_read_double(cpu, 1);
    rom_return(cpu, (__builtin_isnan(a) || __builtin_isnan(b)) ? 1 : 0);
}

/* __truncdfsf2: double → float */
static void stub_truncdfsf2(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    float f = (float)rom_read_double(cpu, 0);
    uint32_t bits;
    memcpy(&bits, &f, 4);
    rom_return(cpu, bits);
}

/* __extendsfdf2: float → double */
static void stub_extendsfdf2(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    uint32_t fbits = rom_arg(cpu, 0);
    float f;
    memcpy(&f, &fbits, 4);
    rom_return_double(cpu, (double)f);
}

/* __bswapsi2: byte-swap a 32-bit word */
static void stub_bswapsi2(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    uint32_t v = rom_arg(cpu, 0);
    v = ((v >> 24) & 0xFF) | ((v >> 8) & 0xFF00) |
        ((v << 8) & 0xFF0000) | ((v << 24) & 0xFF000000u);
    rom_return(cpu, v);
}

/* __ashldi3: 64-bit left shift. (a2,a3) << a4, result in (a2,a3) */
static void stub_ashldi3(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    uint32_t lo = rom_arg(cpu, 0);
    uint32_t hi = rom_arg(cpu, 1);
    uint32_t sh = rom_arg(cpu, 2);
    uint64_t val = ((uint64_t)hi << 32) | lo;
    val <<= (sh & 63);
    rom_return64(cpu, val);
}

/* CRC32 table (standard CRC-32/ISO-HDLC polynomial 0xEDB88320) */
static uint32_t crc32_table[256];
static int crc32_table_ready = 0;

static void crc32_init_table(void) {
    if (crc32_table_ready) return;
    for (int i = 0; i < 256; i++) {
        uint32_t c = (uint32_t)i;
        for (int j = 0; j < 8; j++)
            c = (c >> 1) ^ ((c & 1) ? 0xEDB88320u : 0);
        crc32_table[i] = c;
    }
    crc32_table_ready = 1;
}

/* esp_rom_crc32_le(crc, buf, len) */
static void stub_crc32_le(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    crc32_init_table();
    uint32_t crc = rom_arg(cpu, 0) ^ 0xFFFFFFFFu;
    uint32_t buf = rom_arg(cpu, 1);
    uint32_t len = rom_arg(cpu, 2);
    for (uint32_t i = 0; i < len; i++) {
        uint8_t b = mem_read8(cpu->mem, buf + i);
        crc = crc32_table[(crc ^ b) & 0xFF] ^ (crc >> 8);
    }
    rom_return(cpu, crc ^ 0xFFFFFFFFu);
}

/* esp_crc8(buf, len) — CRC-8 used for eFuse MAC address validation */
static void stub_crc8(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    uint32_t buf = rom_arg(cpu, 0);
    uint32_t len = rom_arg(cpu, 1);
    uint8_t crc = 0;
    for (uint32_t i = 0; i < len; i++) {
        uint8_t b = mem_read8(cpu->mem, buf + i);
        crc ^= b;
        for (int j = 0; j < 8; j++)
            crc = (crc & 0x80) ? (crc << 1) ^ 0x07 : (crc << 1);
    }
    rom_return(cpu, crc);
}

/* ===== ESP-IDF infrastructure stubs ===== */

/* esp_chip_info(info) — fill esp_chip_info_t struct */
static void stub_esp_chip_info(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    uint32_t info = rom_arg(cpu, 0);
    /* esp_chip_info_t: model(u32), features(u32), full_revision(u16), cores(u8), revision(u8) */
    mem_write32(cpu->mem, info + 0, 1);       /* CHIP_ESP32 */
    mem_write32(cpu->mem, info + 4, 0x12);    /* WIFI_BGN(1) | BT(2) | BLE(0x10) */
    mem_write16(cpu->mem, info + 8, 300);     /* full_revision = 3.0 (v3 * 100) */
    mem_write8(cpu->mem, info + 10, 2);       /* cores = 2 */
    mem_write8(cpu->mem, info + 11, 3);       /* revision = 3 */
    rom_return_void(cpu);
}

/* esp_err_to_name(err) — return pointer to static string */
static void stub_esp_err_to_name(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    uint32_t err = rom_arg(cpu, 0);
    /* Write small strings into high SRAM scratch area */
    static const uint32_t str_addr = 0x3FFE3F00u;
    if (err == 0) {
        mem_write8(cpu->mem, str_addr, 'O');
        mem_write8(cpu->mem, str_addr + 1, 'K');
        mem_write8(cpu->mem, str_addr + 2, 0);
    } else {
        const char *s = "FAIL";
        for (int i = 0; s[i]; i++)
            mem_write8(cpu->mem, str_addr + (uint32_t)i, (uint8_t)s[i]);
        mem_write8(cpu->mem, str_addr + 4, 0);
    }
    rom_return(cpu, str_addr);
}

/* esp_get_idf_version() — NO LONGER HOOKED.  The ENTRY-scan aliasing issue
 * that required this hook only occurred when esp_startup_start_app was hooked
 * (adjacent function).  Since esp_startup_start_app is no longer hooked,
 * the real firmware function runs and returns the correct IDF version. */

/* esp_partition_find_first(type, subtype, label) — return fake partition ptr or NULL */
static void stub_esp_partition_find_first(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    uint32_t type    = rom_arg(cpu, 0);
    uint32_t subtype = rom_arg(cpu, 1);
    /* Only return a fake partition for data/coredump (type=1, subtype=0x40) */
    if (type == 1 && subtype == 0x40) {
        /* Return a fake partition struct at a fixed address */
        static const uint32_t fake_part = 0x3FFE3E00u;
        mem_write32(cpu->mem, fake_part + 0, 0x00200000u);  /* address */
        mem_write32(cpu->mem, fake_part + 4, 0x00010000u);  /* size */
        mem_write32(cpu->mem, fake_part + 8, type);
        mem_write32(cpu->mem, fake_part + 12, subtype);
        rom_return(cpu, fake_part);
    } else {
        rom_return(cpu, 0);
    }
}

/* esp_newlib_init — allocate and initialise a minimal _reent struct so that
 * newlib's __sinit / __sfp don't recurse with a NULL pointer.
 * We also write the pointer into _global_impure_ptr (if the symbol exists). */
static void stub_esp_newlib_init(xtensa_cpu_t *cpu, void *ctx) {
    esp32_rom_stubs_t *s = ctx;
    /* Allocate 256 bytes for a minimal _reent struct at a fixed scratch addr */
    static const uint32_t reent_addr = 0x3FFE3C00u;
    /* Zero it out */
    for (uint32_t i = 0; i < 256; i += 4)
        mem_write32(cpu->mem, reent_addr + i, 0);
    /* Set __sdidinit = 1 (offset 24) to prevent __sinit recursion */
    mem_write32(cpu->mem, reent_addr + 24, 1);
    /* newlib struct _reent layout:
     * offset 0: _errno,  offset 4: _stdin,  offset 8: _stdout,  offset 12: _stderr
     * Point stdio at small per-stream structs just past the reent. */
    uint32_t stdin_addr  = reent_addr + 256;
    uint32_t stdout_addr = reent_addr + 256 + 64;
    uint32_t stderr_addr = reent_addr + 256 + 128;
    /* _errno already 0 from zeroing */
    mem_write32(cpu->mem, reent_addr + 4, stdin_addr);
    mem_write32(cpu->mem, reent_addr + 8, stdout_addr);
    mem_write32(cpu->mem, reent_addr + 12, stderr_addr);
    /* Write _global_impure_ptr if we know the address */
    if (s->syms) {
        uint32_t addr;
        if (elf_symbols_find(s->syms, "_global_impure_ptr", &addr) == 0) {
            mem_write32(cpu->mem, addr, reent_addr);
        }
        /* Also set _impure_ptr (per-task reent pointer, same in single-thread) */
        if (elf_symbols_find(s->syms, "_impure_ptr", &addr) == 0) {
            mem_write32(cpu->mem, addr, reent_addr);
        }
    }
    rom_return(cpu, 0);
}

/* __getreent — return global reent pointer (same as set up by esp_newlib_init) */
static void stub_getreent(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    rom_return(cpu, 0x3FFE3C00u);  /* same address as stub_esp_newlib_init */
}

/* esp_ota_get_running_partition() — return fake "factory" partition */
static void stub_esp_ota_get_running_partition(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    static const uint32_t fake_part = 0x3FFE3D00u;
    mem_write32(cpu->mem, fake_part + 0, 0);             /* flash_chip = NULL */
    mem_write32(cpu->mem, fake_part + 4, 0);             /* type = APP (0) */
    mem_write32(cpu->mem, fake_part + 8, 0);             /* subtype = FACTORY (0) */
    mem_write32(cpu->mem, fake_part + 12, 0x00010000u);  /* address = 0x10000 */
    mem_write32(cpu->mem, fake_part + 16, 0x00100000u);  /* size = 1MB */
    /* label at offset 20: "factory\0" */
    const char *label = "factory";
    for (int i = 0; i < 8; i++)
        mem_write8(cpu->mem, fake_part + 20 + (uint32_t)i, (uint8_t)label[i]);
    mem_write8(cpu->mem, fake_part + 37, 0);             /* encrypted = false */
    rom_return(cpu, fake_part);
}

/* esp_partition_mmap(part, offset, size, type, *out_ptr, *out_handle) -> ESP_OK */
static void stub_esp_partition_mmap(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    /* Just return a dummy pointer — firmware won't actually read coredump partition */
    uint32_t out_ptr = rom_arg(cpu, 3);
    uint32_t out_hnd = rom_arg(cpu, 4);
    if (out_ptr) mem_write32(cpu->mem, out_ptr, 0x3FFE3E80u);
    if (out_hnd) mem_write32(cpu->mem, out_hnd, 1);
    rom_return(cpu, 0);
}

/* startup_resume_other_cores — called right before the system init polling loop.
 * In real ESP32, this wakes other cores to run their init functions.
 * In dual-core mode, we signal the main loop to start core 1.
 * In single-core mode, we fake s_system_inited[1] = 1. */
static void stub_startup_resume_other_cores(xtensa_cpu_t *cpu, void *ctx) {
    esp32_rom_stubs_t *s = ctx;
    if (!s->single_core_mode)
        s->app_cpu_start_requested = true;
    /* Always set s_system_inited[1] so core 0's polling loop doesn't hang.
     * In dual-core, core 1 will set the real value later. */
    if (s->s_system_inited_addr)
        mem_write8(cpu->mem, s->s_system_inited_addr + 1, 1);
    /* Signal core 1 to proceed past its call_start_cpu1 wait loop */
    if (s->s_resume_cores_addr)
        mem_write8(cpu->mem, s->s_resume_cores_addr, 1);
    rom_return_void(cpu);
}

/* do_system_init_fn — runs the system_init_fn array, sets s_system_inited.
 * Our stub sets all system_inited flags for both cores and also writes 1
 * to the caller's a2 so start_cpu0 sees "all inited" and skips the polling
 * loop.  The caller stores a2 on the stack right after this call returns
 * and uses it to decide whether to enter the polling loop.
 * (There's also a register window issue where a4 gets corrupted after the
 * call, making the polling accumulator wrong even with flags set.) */
static void stub_do_system_init_fn(xtensa_cpu_t *cpu, void *ctx) {
    esp32_rom_stubs_t *s = ctx;
    if (s->s_system_inited_addr) {
        mem_write8(cpu->mem, s->s_system_inited_addr, 1);      /* core 0 */
        mem_write8(cpu->mem, s->s_system_inited_addr + 1, 1);  /* core 1 */
    }
    if (s->s_system_full_inited_addr)
        mem_write8(cpu->mem, s->s_system_full_inited_addr, 1);
    /* In single-core mode, main_task polls s_other_cpu_startup_done
     * waiting for core 1 to finish.  Set it now so main_task proceeds. */
    if (s->s_other_cpu_startup_done_addr)
        mem_write8(cpu->mem, s->s_other_cpu_startup_done_addr, 1);
    /* rom_return sets ar[ci*4+2] (= caller's a10 for CALL8), but the caller
     * checks a2 after the return.  Hook fires before ENTRY, so ar[2] is
     * still the caller's a2.  Write 1 there so beqz a2 falls through. */
    ar_write(cpu, 2, 1);
    rom_return(cpu, 1);
}

/* esp_startup_start_app — NO LONGER HOOKED.  We let the real function run
 * so that esp_startup_start_app_common() iterates __init_array (C++ global
 * constructors).  After constructors complete, the function calls
 * vTaskStartScheduler() which our FreeRTOS stubs handle. */

/* ===== NVS stubs ===== */

#define ESP_OK              0
#define ESP_ERR_NVS_NOT_FOUND 0x1102

/* nvs_flash_init / nvs_flash_init_partition -> ESP_OK */
void stub_nvs_flash_init(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    rom_return(cpu, ESP_OK);
}

/* nvs_flash_erase -> ESP_OK */
void stub_nvs_flash_erase(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    rom_return(cpu, ESP_OK);
}

/* nvs_open(name, mode, *handle_out) -> ESP_OK, handle=1 */
void stub_nvs_open(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    uint32_t handle_out = rom_arg(cpu, 2);
    if (handle_out)
        mem_write32(cpu->mem, handle_out, 1);
    rom_return(cpu, ESP_OK);
}

/* nvs_open_from_partition(part, name, mode, *handle_out) -> ESP_OK */
void stub_nvs_open_from_partition(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    uint32_t handle_out = rom_arg(cpu, 3);
    if (handle_out)
        mem_write32(cpu->mem, handle_out, 1);
    rom_return(cpu, ESP_OK);
}

/* nvs_close(handle) -> void */
void stub_nvs_close(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    rom_return_void(cpu);
}

/* nvs_get_* -> ESP_ERR_NVS_NOT_FOUND */
void stub_nvs_get_notfound(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    rom_return(cpu, ESP_ERR_NVS_NOT_FOUND);
}

/* nvs_set_* / nvs_commit -> ESP_OK */
void stub_nvs_set_ok(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    rom_return(cpu, ESP_OK);
}

/* ===== GPIO driver stubs ===== */

/* gpio_config(config) -> ESP_OK */
void stub_gpio_config(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    rom_return(cpu, ESP_OK);
}

/* gpio_set_direction(pin, mode) -> ESP_OK */
void stub_gpio_set_direction(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    uint32_t pin  = rom_arg(cpu, 0);
    uint32_t mode = rom_arg(cpu, 1);
    /* Update GPIO ENABLE register if output mode */
    if (mode >= 2 && pin < 32) {  /* GPIO_MODE_OUTPUT = 2 */
        uint32_t enable = mem_read32(cpu->mem, 0x3FF44020u);
        enable |= (1u << pin);
        mem_write32(cpu->mem, 0x3FF44020u, enable);
    }
    rom_return(cpu, ESP_OK);
}

/* gpio_set_level(pin, level) -> ESP_OK */
void stub_gpio_set_level(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    uint32_t pin   = rom_arg(cpu, 0);
    uint32_t level = rom_arg(cpu, 1);
    if (pin < 32) {
        uint32_t out = mem_read32(cpu->mem, 0x3FF44004u);
        if (level)
            out |= (1u << pin);
        else
            out &= ~(1u << pin);
        mem_write32(cpu->mem, 0x3FF44004u, out);
    } else if (pin < 40) {
        uint32_t out1 = mem_read32(cpu->mem, 0x3FF44010u);
        if (level)
            out1 |= (1u << (pin - 32));
        else
            out1 &= ~(1u << (pin - 32));
        mem_write32(cpu->mem, 0x3FF44010u, out1);
    }
    rom_return(cpu, ESP_OK);
}

/* gpio_get_level(pin) -> 0 (no input) */
void stub_gpio_get_level(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    uint32_t pin = rom_arg(cpu, 0);
    uint32_t level = 0;
    if (pin < 32)
        level = (mem_read32(cpu->mem, 0x3FF4403Cu) >> pin) & 1;
    else if (pin < 40)
        level = (mem_read32(cpu->mem, 0x3FF44040u) >> (pin - 32)) & 1;
    rom_return(cpu, level);
}

/* gpio_reset_pin(pin) -> ESP_OK */
void stub_gpio_reset_pin(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    rom_return(cpu, ESP_OK);
}

/* gpio_install_isr_service(flags) -> ESP_OK */
void stub_gpio_isr_service(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    rom_return(cpu, ESP_OK);
}

/* ===== tinfl_decompress stub (ROM function, uses host zlib) ===== */

/*
 * tinfl_decompress is an ESP32 ROM function for DEFLATE decompression.
 * The firmware's PNG decoder (sped.c) calls it via the ROM address.
 * We implement it using the host's zlib inflate() instead of the
 * actual tinfl algorithm, avoiding struct layout mismatches between
 * 32-bit Xtensa and 64-bit host.
 *
 * Firmware calling convention:
 *   tinfl_decompress(r, in_ptr, &in_size, out_start, out_next, &out_size, flags)
 *   7 args — for CALL4 (CALLINC=1), args 0-5 in a6..a11, arg 6 on stack.
 *
 * tinfl_status values: DONE=0, NEEDS_MORE_INPUT=1, HAS_MORE_OUTPUT=2, <0=error
 * tinfl flags: PARSE_ZLIB_HEADER=1, HAS_MORE_INPUT=2
 */

static z_stream tinfl_zstream;
static int tinfl_zstream_inited = 0;

static void stub_tinfl_decompress(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    /* Read arguments: 7 args total */
    uint32_t r_addr       = rom_arg(cpu, 0);  /* tinfl_decompressor* */
    uint32_t in_ptr       = rom_arg(cpu, 1);  /* const uint8_t* compressed data */
    uint32_t in_sz_ptr    = rom_arg(cpu, 2);  /* size_t* input bytes available */
    uint32_t out_start    = rom_arg(cpu, 3);  /* uint8_t* dict start */
    uint32_t out_next     = rom_arg(cpu, 4);  /* uint8_t* current write pos */
    uint32_t out_sz_ptr   = rom_arg(cpu, 5);  /* size_t* output space available */
    /* arg 6 = flags: 7th arg is on the caller's stack.
     * Before ENTRY, window hasn't rotated, so a1 = caller's SP.
     * Xtensa windowed ABI: stack args start at caller_sp + 0. */
    uint32_t caller_sp = ar_read(cpu, 1);
    uint32_t flags = mem_read32(cpu->mem, caller_sp);

    uint32_t in_size  = mem_read32(cpu->mem, in_sz_ptr);
    uint32_t out_size = mem_read32(cpu->mem, out_sz_ptr);

    fprintf(stderr, "[tinfl] r=%08x in=%08x(%u) out_start=%08x out_next=%08x out_sz=%u flags=%u\n",
            r_addr, in_ptr, in_size, out_start, out_next, out_size, flags);

    /* Detect fresh decompressor: firmware calls tinfl_init() which sets m_state=0 */
    uint32_t m_state = mem_read32(cpu->mem, r_addr);
    if (m_state == 0 || !tinfl_zstream_inited) {
        if (tinfl_zstream_inited)
            inflateEnd(&tinfl_zstream);
        memset(&tinfl_zstream, 0, sizeof(tinfl_zstream));
        int wbits = (flags & 1) ? 15 : -15;  /* TINFL_FLAG_PARSE_ZLIB_HEADER */
        fprintf(stderr, "[tinfl] init wbits=%d m_state=%u\n", wbits, m_state);
        inflateInit2(&tinfl_zstream, wbits);
        tinfl_zstream_inited = 1;
        /* Mark as active so we don't re-init on next call */
        mem_write32(cpu->mem, r_addr, 1);
    }

    /* Copy input from emulator memory to host buffer */
    uint8_t *host_in = NULL;
    if (in_size > 0) {
        host_in = malloc(in_size);
        const uint8_t *src = mem_get_ptr(cpu->mem, in_ptr);
        if (src)
            memcpy(host_in, src, in_size);
        else
            for (uint32_t i = 0; i < in_size; i++)
                host_in[i] = mem_read8(cpu->mem, in_ptr + i);
    }

    /* Run inflate */
    tinfl_zstream.next_in = host_in;
    tinfl_zstream.avail_in = in_size;

    uint8_t *host_out = malloc(out_size > 0 ? out_size : 1);
    tinfl_zstream.next_out = host_out;
    tinfl_zstream.avail_out = out_size;

    int zret = inflate(&tinfl_zstream, Z_SYNC_FLUSH);

    uint32_t in_consumed = in_size - (uint32_t)tinfl_zstream.avail_in;
    uint32_t out_produced = out_size - (uint32_t)tinfl_zstream.avail_out;

    fprintf(stderr, "[tinfl] zret=%d consumed=%u produced=%u\n", zret, in_consumed, out_produced);

    /* Copy output to emulator memory */
    if (out_produced > 0) {
        uint8_t *dst = (uint8_t *)mem_get_ptr(cpu->mem, out_next);
        if (dst)
            memcpy(dst, host_out, out_produced);
        else
            for (uint32_t i = 0; i < out_produced; i++)
                mem_write8(cpu->mem, out_next + i, host_out[i]);
    }

    /* Write back consumed/produced sizes */
    mem_write32(cpu->mem, in_sz_ptr, in_consumed);
    mem_write32(cpu->mem, out_sz_ptr, out_produced);

    free(host_in);
    free(host_out);

    /* Map zlib status to tinfl_status */
    int32_t tinfl_status;
    if (zret == Z_STREAM_END) {
        tinfl_status = 0;  /* TINFL_STATUS_DONE */
        inflateEnd(&tinfl_zstream);
        tinfl_zstream_inited = 0;
    } else if (zret == Z_OK || zret == Z_BUF_ERROR) {
        if (tinfl_zstream.avail_out == 0)
            tinfl_status = 2;  /* TINFL_STATUS_HAS_MORE_OUTPUT */
        else
            tinfl_status = 1;  /* TINFL_STATUS_NEEDS_MORE_INPUT */
    } else {
        fprintf(stderr, "[tinfl] ERROR zret=%d msg=%s\n", zret,
                tinfl_zstream.msg ? tinfl_zstream.msg : "(null)");
        tinfl_status = -1;  /* TINFL_STATUS_FAILED */
        inflateEnd(&tinfl_zstream);
        tinfl_zstream_inited = 0;
    }

    fprintf(stderr, "[tinfl] returning status=%d\n", tinfl_status);
    rom_return(cpu, (uint32_t)tinfl_status);
}

/* ===== Heap stubs ===== */

/*
 * Heap allocator for firmware malloc/free/calloc/realloc.
 * Allocates from emulator PSRAM (4MB at 0x3F800000).
 * Each block has an 8-byte header: [size(4)] [magic(4)].
 * Free reclaims top-of-heap blocks and maintains a free list for reuse.
 */
#define HEAP_BASE    0x3F800000u  /* PSRAM base — 4MB available */
#define HEAP_END     0x3FC00000u  /* PSRAM end */
#define HEAP_MAGIC   0x48454150u  /* "HEAP" */
#define HEAP_FREE    0x46524545u  /* "FREE" */
#define HEAP_HDR_SZ  8

static uint32_t heap_ptr = HEAP_BASE;

/* Free list: freed blocks stored as linked list via their header.
 * Header layout for free blocks: [size(4)] [HEAP_FREE(4)] [next_free(4)]
 * next_free is stored in the user data area (first 4 bytes after header). */
static uint32_t free_list = 0;  /* head of free list (emulator address, 0=empty) */

/* Try to find a free-list block >= requested size */
static uint32_t freelist_alloc(xtensa_cpu_t *cpu, uint32_t size) {
    uint32_t prev_addr = 0;
    uint32_t cur = free_list;
    while (cur) {
        uint32_t block_size = mem_read32(cpu->mem, cur);
        uint32_t next = mem_read32(cpu->mem, cur + HEAP_HDR_SZ);  /* next ptr in user area */
        if (block_size >= size) {
            /* Unlink from free list */
            if (prev_addr)
                mem_write32(cpu->mem, prev_addr + HEAP_HDR_SZ, next);
            else
                free_list = next;
            /* Mark as allocated */
            mem_write32(cpu->mem, cur + 4, HEAP_MAGIC);
            return cur + HEAP_HDR_SZ;
        }
        prev_addr = cur;
        cur = next;
    }
    return 0;
}

static uint32_t heap_alloc(xtensa_cpu_t *cpu, uint32_t size) {
    if (size == 0) return 0;
    /* Align size to 4 bytes */
    size = (size + 3) & ~3u;
    /* Try free list first */
    uint32_t ptr = freelist_alloc(cpu, size);
    if (ptr) return ptr;
    /* Bump allocate */
    uint32_t total = HEAP_HDR_SZ + size;
    if (heap_ptr + total > HEAP_END) {
        fprintf(stderr, "[heap] OOM: need %u, have %u free_list=%u\n",
                total, HEAP_END - heap_ptr, free_list ? 1 : 0);
        return 0;
    }
    uint32_t block = heap_ptr;
    mem_write32(cpu->mem, block, size);
    mem_write32(cpu->mem, block + 4, HEAP_MAGIC);
    heap_ptr += total;
    return block + HEAP_HDR_SZ;
}

static void heap_free(xtensa_cpu_t *cpu, uint32_t ptr) {
    if (ptr == 0 || ptr < HEAP_BASE + HEAP_HDR_SZ || ptr >= HEAP_END) return;
    uint32_t block = ptr - HEAP_HDR_SZ;
    uint32_t magic = mem_read32(cpu->mem, block + 4);
    if (magic != HEAP_MAGIC) return;  /* double free or corruption */
    uint32_t size = mem_read32(cpu->mem, block);
    uint32_t total = HEAP_HDR_SZ + ((size + 3) & ~3u);

    /* If this is the topmost block, shrink the heap */
    if (block + total == heap_ptr) {
        mem_write32(cpu->mem, block + 4, 0);  /* clear magic */
        heap_ptr = block;
        /* Single-block reclaim handles the common malloc-then-free pattern.
         * We can't walk backwards because block sizes vary. */
        return;
    }
    /* Otherwise, add to free list */
    mem_write32(cpu->mem, block + 4, HEAP_FREE);
    mem_write32(cpu->mem, ptr, free_list);  /* store next ptr in user data */
    free_list = block;
}

/* malloc(size) -> pointer or NULL */
static void stub_malloc(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    uint32_t size = rom_arg(cpu, 0);
    uint32_t ptr = heap_alloc(cpu, size);
    if (size >= 1024)
        fprintf(stderr, "[heap] malloc(%u) -> 0x%08X (free=%u)\n",
                size, ptr, HEAP_END - heap_ptr);
    rom_return(cpu, ptr);
}

/* calloc(nmemb, size) -> pointer or NULL (zeroed) */
static void stub_calloc(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    uint32_t nmemb = rom_arg(cpu, 0);
    uint32_t size  = rom_arg(cpu, 1);
    uint32_t total = nmemb * size;
    uint32_t ptr = heap_alloc(cpu, total);
    if (ptr) {
        for (uint32_t i = 0; i < total; i++)
            mem_write8(cpu->mem, ptr + i, 0);
    }
    rom_return(cpu, ptr);
}

/* free(ptr) -> void */
static void stub_free(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    uint32_t ptr = rom_arg(cpu, 0);
    if (ptr >= HEAP_BASE && ptr < HEAP_END) {
        uint32_t size = mem_read32(cpu->mem, ptr - HEAP_HDR_SZ);
        if (size >= 1024)
            fprintf(stderr, "[heap] free(0x%08X) size=%u\n", ptr, size);
    }
    heap_free(cpu, ptr);
    rom_return_void(cpu);
}

/* realloc(ptr, size) -> pointer or NULL */
static void stub_realloc(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    uint32_t old_ptr = rom_arg(cpu, 0);
    uint32_t new_size = rom_arg(cpu, 1);
    if (new_size == 0) { heap_free(cpu, old_ptr); rom_return(cpu, 0); return; }
    uint32_t new_ptr = heap_alloc(cpu, new_size);
    if (new_ptr && old_ptr) {
        /* Copy old data — read old size from header */
        uint32_t old_size = mem_read32(cpu->mem, old_ptr - HEAP_HDR_SZ);
        uint32_t copy = old_size < new_size ? old_size : new_size;
        for (uint32_t i = 0; i < copy; i++)
            mem_write8(cpu->mem, new_ptr + i, mem_read8(cpu->mem, old_ptr + i));
        heap_free(cpu, old_ptr);
    }
    rom_return(cpu, new_ptr);
}

/* esp_get_free_heap_size() -> remaining heap bytes */
void stub_esp_get_free_heap_size(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    rom_return(cpu, HEAP_END - heap_ptr);
}

/* esp_get_minimum_free_heap_size() -> remaining heap bytes */
void stub_esp_get_minimum_free_heap_size(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    rom_return(cpu, HEAP_END - heap_ptr);
}

/* esp_log_timestamp() -> ccount / (cpu_freq_mhz * 1000) */
void stub_esp_log_timestamp(xtensa_cpu_t *cpu, void *ctx) {
    esp32_rom_stubs_t *s = ctx;
    uint32_t ms = cpu->ccount / (s->cpu_freq_mhz * 1000);
    rom_return(cpu, ms);
}

/* esp_log_write(level, tag, format, ...) - output via ets_printf mechanism */
static void stub_esp_log_write(xtensa_cpu_t *cpu, void *ctx) {
    esp32_rom_stubs_t *s = ctx;
    uint32_t level = rom_arg(cpu, 0);
    uint32_t tag_addr = rom_arg(cpu, 1);
    uint32_t fmt_addr = rom_arg(cpu, 2);

    /* Read tag string */
    char tag[64] = "";
    for (int i = 0; i < 63; i++) {
        uint8_t c = mem_read8(cpu->mem, tag_addr + i);
        if (c == 0) break;
        tag[i] = (char)c;
        tag[i + 1] = '\0';
    }

    /* Color codes for different log levels */
    const char *color_start = "";
    const char *color_end = "\033[0m";
    const char *level_str = "?";

    switch (level) {
        case 1: /* ESP_LOG_ERROR */
            color_start = "\033[0;31m";  /* Red */
            level_str = "E";
            break;
        case 2: /* ESP_LOG_WARN */
            color_start = "\033[0;33m";  /* Yellow */
            level_str = "W";
            break;
        case 3: /* ESP_LOG_INFO */
            color_start = "\033[0;32m";  /* Green */
            level_str = "I";
            break;
        case 4: /* ESP_LOG_DEBUG */
            color_start = "";
            level_str = "D";
            break;
        case 5: /* ESP_LOG_VERBOSE */
            color_start = "";
            level_str = "V";
            break;
    }

    /* Get timestamp */
    uint32_t ms = cpu->ccount / (s->cpu_freq_mhz * 1000);

    /* Output log prefix: color + level + timestamp + tag */
    int n = snprintf(s->output + s->output_len, OUTPUT_BUF_SIZE - s->output_len,
                     "%s%s (%u) %s: ", color_start, level_str, ms, tag);
    if (n > 0 && s->output_len + n < OUTPUT_BUF_SIZE)
        s->output_len += n;

    /* Process format string and varargs (starting from arg 3),
     * reusing the same format engine as mini_printf */
    {
        /* Temporarily adjust the CPU arg offset so mini_printf reads
         * from arg 3 onwards.  We do this by saving fmt_addr in arg 0
         * position and calling the shared engine. */
        /* Inline format processing — same code as mini_printf
         * but starting from argn=3 */
        int argn = 3;
        for (;;) {
            uint8_t ch = mem_read8(cpu->mem, fmt_addr++);
            if (ch == 0) break;
            if (ch != '%') { output_char(s, (char)ch); continue; }
            ch = mem_read8(cpu->mem, fmt_addr++);
            if (ch == 0) break;
            if (ch == '%') { output_char(s, '%'); continue; }

            /* Parse flags */
            char pad_char = ' ';
            int left_justify = 0;
            if (ch == '-') {
                left_justify = 1;
                ch = mem_read8(cpu->mem, fmt_addr++);
                if (ch == 0) break;
            }
            if (ch == '0' && !left_justify) {
                pad_char = '0';
                ch = mem_read8(cpu->mem, fmt_addr++);
                if (ch == 0) break;
            }

            /* Parse width */
            int width = 0;
            while (ch >= '0' && ch <= '9') {
                width = width * 10 + (ch - '0');
                ch = mem_read8(cpu->mem, fmt_addr++);
                if (ch == 0) goto log_done;
            }

            /* Skip precision */
            if (ch == '.') {
                ch = mem_read8(cpu->mem, fmt_addr++);
                while (ch >= '0' && ch <= '9')
                    ch = mem_read8(cpu->mem, fmt_addr++);
                if (ch == 0) break;
            }

            /* Length modifier */
            int is_long_long = 0;
            if (ch == 'l') {
                ch = mem_read8(cpu->mem, fmt_addr++);
                if (ch == 0) break;
                if (ch == 'l') { is_long_long = 1; ch = mem_read8(cpu->mem, fmt_addr++); if (ch == 0) break; }
            } else if (ch == 'h') {
                ch = mem_read8(cpu->mem, fmt_addr++);
                if (ch == 0) break;
                if (ch == 'h') { ch = mem_read8(cpu->mem, fmt_addr++); if (ch == 0) break; }
            } else if (ch == 'z') {
                ch = mem_read8(cpu->mem, fmt_addr++);
                if (ch == 0) break;
            }

            uint64_t val64 = 0;
            uint32_t val = 0;
            if (is_long_long) {
                if (argn % 2 != 0) argn++;
                uint32_t lo = rom_arg(cpu, argn++);
                uint32_t hi = rom_arg(cpu, argn++);
                val64 = ((uint64_t)hi << 32) | lo;
                val = (uint32_t)val64;
            } else {
                val = rom_arg(cpu, argn++);
                val64 = val;
            }

            char numbuf[24];
            int numlen = 0;
            switch (ch) {
            case 'd': case 'i': {
                int neg = 0;
                uint64_t uv;
                if (is_long_long) { int64_t sv = (int64_t)val64; if (sv < 0) { neg = 1; sv = -sv; } uv = (uint64_t)sv; }
                else { int32_t sv = (int32_t)val; if (sv < 0) { neg = 1; sv = -sv; } uv = (uint32_t)sv; }
                if (uv == 0) numbuf[numlen++] = '0';
                else while (uv > 0) { numbuf[numlen++] = '0' + (int)(uv % 10); uv /= 10; }
                int total = numlen + neg;
                if (!left_justify) while (total < width) { output_char(s, pad_char); total++; }
                if (neg) output_char(s, '-');
                for (int i = numlen - 1; i >= 0; i--) output_char(s, numbuf[i]);
                if (left_justify) while (total < width) { output_char(s, ' '); total++; }
                break;
            }
            case 'u': {
                uint64_t uv = is_long_long ? val64 : val;
                if (uv == 0) numbuf[numlen++] = '0';
                else while (uv > 0) { numbuf[numlen++] = '0' + (int)(uv % 10); uv /= 10; }
                int total = numlen;
                if (!left_justify) while (total < width) { output_char(s, pad_char); total++; }
                for (int i = numlen - 1; i >= 0; i--) output_char(s, numbuf[i]);
                if (left_justify) while (total < width) { output_char(s, ' '); total++; }
                break;
            }
            case 'x': case 'X': case 'p': {
                const char *hexdig = (ch == 'X') ? "0123456789ABCDEF" : "0123456789abcdef";
                uint64_t uv = is_long_long ? val64 : val;
                if (uv == 0) numbuf[numlen++] = '0';
                else while (uv > 0) { numbuf[numlen++] = hexdig[uv & 0xF]; uv >>= 4; }
                int total = numlen;
                if (!left_justify) while (total < width) { output_char(s, pad_char); total++; }
                for (int i = numlen - 1; i >= 0; i--) output_char(s, numbuf[i]);
                if (left_justify) while (total < width) { output_char(s, ' '); total++; }
                break;
            }
            case 's': {
                uint32_t saddr = val;
                int slen = 0;
                uint32_t tmp = saddr;
                while (mem_read8(cpu->mem, tmp) != 0) { slen++; tmp++; }
                if (!left_justify) while (slen < width) { output_char(s, ' '); slen++; }
                int printed = 0;
                while (1) { uint8_t c = mem_read8(cpu->mem, saddr++); if (c == 0) break; output_char(s, (char)c); printed++; }
                if (left_justify) while (printed < width) { output_char(s, ' '); printed++; }
                break;
            }
            case 'c': output_char(s, (char)(val & 0xFF)); break;
            default: output_char(s, '%'); output_char(s, (char)ch); break;
            }
        }
    }
log_done:
    (void)0;

    /* Add color end and newline */
    n = snprintf(s->output + s->output_len, OUTPUT_BUF_SIZE - s->output_len,
                 "%s\n", color_end);
    if (n > 0 && s->output_len + n < OUTPUT_BUF_SIZE)
        s->output_len += n;

    rom_return_void(cpu);
}

/* Generic no-op ROM stub: returns 0 for unregistered ROM calls */
static void stub_unregistered(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    rom_return(cpu, 0);
}

/* Generic void ROM stub: returns without a value */
static void stub_void_unregistered(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    rom_return_void(cpu);
}

/* Real intr_matrix_set(core, source, cpu_int): programs the interrupt matrix.
 * In native FreeRTOS mode, this lets firmware configure interrupt routing. */
static void stub_intr_matrix_set(xtensa_cpu_t *cpu, void *ctx) {
    esp32_rom_stubs_t *s = ctx;
    uint32_t core    = rom_arg(cpu, 0);
    uint32_t source  = rom_arg(cpu, 1);
    uint32_t cpu_int = rom_arg(cpu, 2);
    if (s->periph && core <= 1 && cpu_int < 32)
        periph_intr_matrix_set(s->periph, (int)core, (int)cpu_int, (int)source);
    rom_return_void(cpu);
}

/* Generic stub that returns ESP_FAIL (-1) */
static void stub_ret_esp_fail(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    rom_return(cpu, (uint32_t)-1);  /* ESP_FAIL */
}

/* Return STA_NODISK|STA_NOINIT (3) for ff_sd_initialize — tells FatFS
 * there is no card so f_mount returns FR_NOT_READY immediately. */
static void stub_ret_sd_nodisk(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    rom_return(cpu, 3);  /* STA_NOINIT | STA_NODISK */
}

/* Return WL_CONNECTED (3) for WiFi.status() */
static void stub_ret_wl_connected(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    rom_return(cpu, 3);  /* WL_CONNECTED */
}

/* digitalRead returns HIGH (1) — prevents firmware from thinking
 * buttons are pressed (most buttons are active-low). */
static void stub_digital_read_high(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    rom_return(cpu, 1);
}

/* Return 1 (true) for bool-returning stubs */
static void stub_ret_true(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    rom_return(cpu, 1);
}

/* Return -1 for read-like stubs (no data available) */
static void stub_ret_neg1(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    rom_return(cpu, (uint32_t)-1);
}

/* HardwareSerial::write(buf, len) — return len (arg2) to indicate all bytes written */
static void stub_serial_write_buf(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    uint32_t len = rom_arg(cpu, 2);  /* arg0=this, arg1=buf, arg2=len */
    rom_return(cpu, len);
}

/* uartBegin returns a uart_t* — must be non-null for Serial to work.
 * Return a fake pointer so operator bool() succeeds. */
static void stub_uartBegin(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    rom_return(cpu, 0x3FFB0700u);  /* fake uart_t* in unused DRAM */
}

/* __assert_func / abort — skip assertion failures (missing infra).
 * For noreturn functions, stop the CPU so the main loop can redirect
 * to a deferred task or halt cleanly.  Returning would enter undefined
 * territory (unreachable code after the call). */
static void stub_abort_stop(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    cpu->running = 0;
}

/* __stack_chk_fail — canary mismatch.  In our emulator this is typically
 * caused by window spill/fill register corruption (deep call stacks
 * overwriting spilled data), not real buffer overflows.  Instead of
 * stopping the CPU, scan forward from the return address to find the
 * caller's retw.n and skip to it, effectively ignoring the check.
 *
 * Caller pattern:
 *   beq aX, aY, <retw_target>
 *   nop
 *   call8 __stack_chk_fail
 *   ... unreachable ...
 *   retw.n                    <-- we jump here
 */
static void stub_stack_chk_fail_skip(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    uint32_t a0 = ar_read(cpu, 0);
    uint32_t ret_pc = (a0 & 0x3FFFFFFF) | (cpu->pc & 0xC0000000);

    /* Scan forward for retw.n (0x1D 0xF0) within 128 bytes */
    for (int i = 0; i < 128; i++) {
        uint8_t b0 = mem_read8(cpu->mem, ret_pc + i);
        uint8_t b1 = mem_read8(cpu->mem, ret_pc + i + 1);
        if (b0 == 0x1d && b1 == 0xf0) {
            /* Found retw.n — redirect return address and return */
            uint32_t new_a0 = ((ret_pc + i) & 0x3FFFFFFF) | (a0 & 0xC0000000);
            ar_write(cpu, 0, new_a0);
            rom_return_void(cpu);
            return;
        }
    }
    /* Fallback: stop CPU if retw.n not found */
    cpu->running = 0;
}

/* esp_panic_handler(XtExcFrame *frame) — dump exception info before restart.
 * XtExcFrame layout (esp-idf xtensa_context.h):
 *   exit(0) pc(4) ps(8) a0(12)..a15(72) sar(76) exccause(80) excvaddr(84) */
static void stub_panic_handler(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    uint32_t frame = rom_arg(cpu, 0);
    uint32_t pc_val    = mem_read32(cpu->mem, frame + 4);
    uint32_t ps_val    = mem_read32(cpu->mem, frame + 8);
    uint32_t exccause  = mem_read32(cpu->mem, frame + 80);
    uint32_t excvaddr  = mem_read32(cpu->mem, frame + 84);
    static const char *exc_names[] = {
        [0]  = "IllegalInstruction", [2]  = "InstructionFetchError",
        [3]  = "LoadStoreError",     [6]  = "IntegerDivideByZero",
        [9]  = "LoadStoreAlignment", [28] = "LoadProhibited",
        [29] = "StoreProhibited",
    };
    const char *name = (exccause < 30 && exc_names[exccause])
                       ? exc_names[exccause] : "Unknown";
    fprintf(stderr, "\n[PANIC] %s (cause=%u) at PC=0x%08X  EXCVADDR=0x%08X  PS=0x%08X\n",
            name, exccause, pc_val, excvaddr, ps_val);
    fprintf(stderr, "  Registers from frame at 0x%08X:\n", frame);
    for (int i = 0; i < 16; i++) {
        uint32_t val = mem_read32(cpu->mem, frame + 12 + i * 4);
        fprintf(stderr, "  a%-2d=0x%08X%s", i, val, (i % 4 == 3) ? "\n" : "  ");
    }
    fprintf(stderr, "  SAR=0x%08X\n", mem_read32(cpu->mem, frame + 76));
    /* Dump actual CPU state at panic time */
    fprintf(stderr, "  --- Actual CPU state (not from frame) ---\n");
    fprintf(stderr, "  EPC1=0x%08X EXCCAUSE=%u EXCVADDR=0x%08X\n",
            cpu->epc[0], cpu->exccause, cpu->excvaddr);
    fprintf(stderr, "  PS=0x%08X SAR=%u WB=%u WS=0x%X\n",
            cpu->ps, cpu->sar, cpu->windowbase, cpu->windowstart);
    fprintf(stderr, "  EPC2=0x%08X EPC3=0x%08X DEPC=0x%08X\n",
            cpu->epc[1], cpu->epc[2], cpu->depc);
    fprintf(stderr, "  cycle_count=%llu ccount=%u\n",
            (unsigned long long)cpu->cycle_count, cpu->ccount);
    for (int r = 0; r < 16; r += 4)
        fprintf(stderr, "  AR%-2d=0x%08X  AR%-2d=0x%08X  AR%-2d=0x%08X  AR%-2d=0x%08X\n",
                r, ar_read(cpu, r), r+1, ar_read(cpu, r+1),
                r+2, ar_read(cpu, r+2), r+3, ar_read(cpu, r+3));
    cpu->running = 0;
}

/* ===== RNG stubs ===== */

static uint32_t emu_random32(void) {
    static uint64_t state = 0xDEADBEEFCAFEBABEULL;
    state ^= state << 13;
    state ^= state >> 7;
    state ^= state << 17;
    return (uint32_t)state;
}

/* esp_random() -> uint32_t */
static void stub_esp_random(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    rom_return(cpu, emu_random32());
}

/* esp_fill_random(buf, len) */
static void stub_esp_fill_random(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    uint32_t buf = rom_arg(cpu, 0);
    uint32_t len = rom_arg(cpu, 1);
    for (uint32_t i = 0; i < len; i++)
        mem_write8(cpu->mem, buf + i, (uint8_t)(emu_random32() >> 16));
    rom_return_void(cpu);
}

/* hal_random() -> uint32_t (Arduino wrapper) */
static void stub_hal_random(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    rom_return(cpu, emu_random32());
}

/* bootloader_fill_random(buf, len) */
static void stub_bootloader_fill_random(xtensa_cpu_t *cpu, void *ctx) {
    stub_esp_fill_random(cpu, ctx);
}

/* ===== MD5 ROM stubs ===== */

/* MD5 context layout in emulator memory (matches ESP32 ROM struct) */
#define MD5_CTX_STATE_OFS  0    /* uint32_t buf[4] — state a,b,c,d */
#define MD5_CTX_BITS_OFS   16   /* uint32_t bits[2] — count low, high */
#define MD5_CTX_IN_OFS     24   /* uint8_t in[64] — input buffer */
#define MD5_CTX_SIZE       88

/* MD5 round functions */
#define MD5_F(x,y,z) (((x)&(y))|((~(x))&(z)))
#define MD5_G(x,y,z) (((x)&(z))|((y)&(~(z))))
#define MD5_H(x,y,z) ((x)^(y)^(z))
#define MD5_I(x,y,z) ((y)^((x)|(~(z))))
#define MD5_ROL(x,n) (((x)<<(n))|((x)>>(32-(n))))

#define MD5_STEP(f,a,b,c,d,x,t,s) do { \
    (a) += f((b),(c),(d)) + (x) + (t); \
    (a) = MD5_ROL((a),(s)); \
    (a) += (b); \
} while(0)

static void md5_transform(uint32_t state[4], const uint32_t block[16]) {
    uint32_t a = state[0], b = state[1], c = state[2], d = state[3];

    /* Round 1 */
    MD5_STEP(MD5_F,a,b,c,d, block[ 0], 0xd76aa478,  7);
    MD5_STEP(MD5_F,d,a,b,c, block[ 1], 0xe8c7b756, 12);
    MD5_STEP(MD5_F,c,d,a,b, block[ 2], 0x242070db, 17);
    MD5_STEP(MD5_F,b,c,d,a, block[ 3], 0xc1bdceee, 22);
    MD5_STEP(MD5_F,a,b,c,d, block[ 4], 0xf57c0faf,  7);
    MD5_STEP(MD5_F,d,a,b,c, block[ 5], 0x4787c62a, 12);
    MD5_STEP(MD5_F,c,d,a,b, block[ 6], 0xa8304613, 17);
    MD5_STEP(MD5_F,b,c,d,a, block[ 7], 0xfd469501, 22);
    MD5_STEP(MD5_F,a,b,c,d, block[ 8], 0x698098d8,  7);
    MD5_STEP(MD5_F,d,a,b,c, block[ 9], 0x8b44f7af, 12);
    MD5_STEP(MD5_F,c,d,a,b, block[10], 0xffff5bb1, 17);
    MD5_STEP(MD5_F,b,c,d,a, block[11], 0x895cd7be, 22);
    MD5_STEP(MD5_F,a,b,c,d, block[12], 0x6b901122,  7);
    MD5_STEP(MD5_F,d,a,b,c, block[13], 0xfd987193, 12);
    MD5_STEP(MD5_F,c,d,a,b, block[14], 0xa679438e, 17);
    MD5_STEP(MD5_F,b,c,d,a, block[15], 0x49b40821, 22);

    /* Round 2 */
    MD5_STEP(MD5_G,a,b,c,d, block[ 1], 0xf61e2562,  5);
    MD5_STEP(MD5_G,d,a,b,c, block[ 6], 0xc040b340,  9);
    MD5_STEP(MD5_G,c,d,a,b, block[11], 0x265e5a51, 14);
    MD5_STEP(MD5_G,b,c,d,a, block[ 0], 0xe9b6c7aa, 20);
    MD5_STEP(MD5_G,a,b,c,d, block[ 5], 0xd62f105d,  5);
    MD5_STEP(MD5_G,d,a,b,c, block[10], 0x02441453,  9);
    MD5_STEP(MD5_G,c,d,a,b, block[15], 0xd8a1e681, 14);
    MD5_STEP(MD5_G,b,c,d,a, block[ 4], 0xe7d3fbc8, 20);
    MD5_STEP(MD5_G,a,b,c,d, block[ 9], 0x21e1cde6,  5);
    MD5_STEP(MD5_G,d,a,b,c, block[14], 0xc33707d6,  9);
    MD5_STEP(MD5_G,c,d,a,b, block[ 3], 0xf4d50d87, 14);
    MD5_STEP(MD5_G,b,c,d,a, block[ 8], 0x455a14ed, 20);
    MD5_STEP(MD5_G,a,b,c,d, block[13], 0xa9e3e905,  5);
    MD5_STEP(MD5_G,d,a,b,c, block[ 2], 0xfcefa3f8,  9);
    MD5_STEP(MD5_G,c,d,a,b, block[ 7], 0x676f02d9, 14);
    MD5_STEP(MD5_G,b,c,d,a, block[12], 0x8d2a4c8a, 20);

    /* Round 3 */
    MD5_STEP(MD5_H,a,b,c,d, block[ 5], 0xfffa3942,  4);
    MD5_STEP(MD5_H,d,a,b,c, block[ 8], 0x8771f681, 11);
    MD5_STEP(MD5_H,c,d,a,b, block[11], 0x6d9d6122, 16);
    MD5_STEP(MD5_H,b,c,d,a, block[14], 0xfde5380c, 23);
    MD5_STEP(MD5_H,a,b,c,d, block[ 1], 0xa4beea44,  4);
    MD5_STEP(MD5_H,d,a,b,c, block[ 4], 0x4bdecfa9, 11);
    MD5_STEP(MD5_H,c,d,a,b, block[ 7], 0xf6bb4b60, 16);
    MD5_STEP(MD5_H,b,c,d,a, block[10], 0xbebfbc70, 23);
    MD5_STEP(MD5_H,a,b,c,d, block[13], 0x289b7ec6,  4);
    MD5_STEP(MD5_H,d,a,b,c, block[ 0], 0xeaa127fa, 11);
    MD5_STEP(MD5_H,c,d,a,b, block[ 3], 0xd4ef3085, 16);
    MD5_STEP(MD5_H,b,c,d,a, block[ 6], 0x04881d05, 23);
    MD5_STEP(MD5_H,a,b,c,d, block[ 9], 0xd9d4d039,  4);
    MD5_STEP(MD5_H,d,a,b,c, block[12], 0xe6db99e5, 11);
    MD5_STEP(MD5_H,c,d,a,b, block[15], 0x1fa27cf8, 16);
    MD5_STEP(MD5_H,b,c,d,a, block[ 2], 0xc4ac5665, 23);

    /* Round 4 */
    MD5_STEP(MD5_I,a,b,c,d, block[ 0], 0xf4292244,  6);
    MD5_STEP(MD5_I,d,a,b,c, block[ 7], 0x432aff97, 10);
    MD5_STEP(MD5_I,c,d,a,b, block[14], 0xab9423a7, 15);
    MD5_STEP(MD5_I,b,c,d,a, block[ 5], 0xfc93a039, 21);
    MD5_STEP(MD5_I,a,b,c,d, block[12], 0x655b59c3,  6);
    MD5_STEP(MD5_I,d,a,b,c, block[ 3], 0x8f0ccc92, 10);
    MD5_STEP(MD5_I,c,d,a,b, block[10], 0xffeff47d, 15);
    MD5_STEP(MD5_I,b,c,d,a, block[ 1], 0x85845dd1, 21);
    MD5_STEP(MD5_I,a,b,c,d, block[ 8], 0x6fa87e4f,  6);
    MD5_STEP(MD5_I,d,a,b,c, block[15], 0xfe2ce6e0, 10);
    MD5_STEP(MD5_I,c,d,a,b, block[ 6], 0xa3014314, 15);
    MD5_STEP(MD5_I,b,c,d,a, block[13], 0x4e0811a1, 21);
    MD5_STEP(MD5_I,a,b,c,d, block[ 4], 0xf7537e82,  6);
    MD5_STEP(MD5_I,d,a,b,c, block[11], 0xbd3af235, 10);
    MD5_STEP(MD5_I,c,d,a,b, block[ 2], 0x2ad7d2bb, 15);
    MD5_STEP(MD5_I,b,c,d,a, block[ 9], 0xeb86d391, 21);

    state[0] += a; state[1] += b; state[2] += c; state[3] += d;
}

static void md5_process(xtensa_cpu_t *cpu, uint32_t ctx_addr) {
    uint32_t state[4], block[16];
    for (int i = 0; i < 4; i++)
        state[i] = mem_read32(cpu->mem, ctx_addr + MD5_CTX_STATE_OFS + i * 4);
    for (int i = 0; i < 16; i++)
        block[i] = mem_read32(cpu->mem, ctx_addr + MD5_CTX_IN_OFS + i * 4);
    md5_transform(state, block);
    for (int i = 0; i < 4; i++)
        mem_write32(cpu->mem, ctx_addr + MD5_CTX_STATE_OFS + i * 4, state[i]);
}

/* esp_rom_md5_init(md5_context_t *ctx) */
static void stub_md5_init(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    uint32_t md5ctx = rom_arg(cpu, 0);
    /* Standard MD5 IV */
    mem_write32(cpu->mem, md5ctx + MD5_CTX_STATE_OFS + 0, 0x67452301);
    mem_write32(cpu->mem, md5ctx + MD5_CTX_STATE_OFS + 4, 0xefcdab89);
    mem_write32(cpu->mem, md5ctx + MD5_CTX_STATE_OFS + 8, 0x98badcfe);
    mem_write32(cpu->mem, md5ctx + MD5_CTX_STATE_OFS + 12, 0x10325476);
    mem_write32(cpu->mem, md5ctx + MD5_CTX_BITS_OFS + 0, 0);
    mem_write32(cpu->mem, md5ctx + MD5_CTX_BITS_OFS + 4, 0);
    for (int i = 0; i < 64; i++)
        mem_write8(cpu->mem, md5ctx + MD5_CTX_IN_OFS + (uint32_t)i, 0);
    rom_return_void(cpu);
}

/* esp_rom_md5_update(md5_context_t *ctx, const void *data, uint32_t len) */
static void stub_md5_update(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    uint32_t md5ctx  = rom_arg(cpu, 0);
    uint32_t data    = rom_arg(cpu, 1);
    uint32_t len     = rom_arg(cpu, 2);

    /* Read bit count */
    uint32_t lo = mem_read32(cpu->mem, md5ctx + MD5_CTX_BITS_OFS);
    uint32_t hi = mem_read32(cpu->mem, md5ctx + MD5_CTX_BITS_OFS + 4);
    uint32_t buf_used = (lo >> 3) & 0x3F;  /* bytes in buffer */

    /* Update bit count */
    uint64_t bits = ((uint64_t)hi << 32) | lo;
    bits += (uint64_t)len << 3;
    mem_write32(cpu->mem, md5ctx + MD5_CTX_BITS_OFS, (uint32_t)bits);
    mem_write32(cpu->mem, md5ctx + MD5_CTX_BITS_OFS + 4, (uint32_t)(bits >> 32));

    uint32_t off = 0;

    /* Fill existing buffer */
    if (buf_used > 0) {
        uint32_t fill = 64 - buf_used;
        if (len < fill) fill = len;
        for (uint32_t i = 0; i < fill; i++)
            mem_write8(cpu->mem, md5ctx + MD5_CTX_IN_OFS + buf_used + i,
                       mem_read8(cpu->mem, data + i));
        buf_used += fill;
        off += fill;
        len -= fill;
        if (buf_used == 64) {
            md5_process(cpu, md5ctx);
            buf_used = 0;
        }
    }

    /* Process full blocks */
    while (len >= 64) {
        for (int i = 0; i < 64; i++)
            mem_write8(cpu->mem, md5ctx + MD5_CTX_IN_OFS + (uint32_t)i,
                       mem_read8(cpu->mem, data + off + (uint32_t)i));
        md5_process(cpu, md5ctx);
        off += 64;
        len -= 64;
    }

    /* Store remainder in buffer */
    for (uint32_t i = 0; i < len; i++)
        mem_write8(cpu->mem, md5ctx + MD5_CTX_IN_OFS + i,
                   mem_read8(cpu->mem, data + off + i));

    rom_return_void(cpu);
}

/* esp_rom_md5_final(uint8_t digest[16], md5_context_t *ctx) */
static void stub_md5_final(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    uint32_t digest  = rom_arg(cpu, 0);
    uint32_t md5ctx  = rom_arg(cpu, 1);

    uint32_t lo = mem_read32(cpu->mem, md5ctx + MD5_CTX_BITS_OFS);
    uint32_t buf_used = (lo >> 3) & 0x3F;

    /* Pad: 0x80, then zeros, then 8-byte bit count (LE) */
    mem_write8(cpu->mem, md5ctx + MD5_CTX_IN_OFS + buf_used, 0x80);
    for (uint32_t i = buf_used + 1; i < 64; i++)
        mem_write8(cpu->mem, md5ctx + MD5_CTX_IN_OFS + i, 0);

    if (buf_used >= 56) {
        md5_process(cpu, md5ctx);
        for (int i = 0; i < 56; i++)
            mem_write8(cpu->mem, md5ctx + MD5_CTX_IN_OFS + (uint32_t)i, 0);
    }

    /* Append bit count as little-endian 64-bit */
    uint32_t hi = mem_read32(cpu->mem, md5ctx + MD5_CTX_BITS_OFS + 4);
    mem_write32(cpu->mem, md5ctx + MD5_CTX_IN_OFS + 56, lo);
    mem_write32(cpu->mem, md5ctx + MD5_CTX_IN_OFS + 60, hi);
    md5_process(cpu, md5ctx);

    /* Write digest (state as LE bytes) */
    for (int i = 0; i < 4; i++) {
        uint32_t val = mem_read32(cpu->mem, md5ctx + MD5_CTX_STATE_OFS + i * 4);
        mem_write8(cpu->mem, digest + (uint32_t)(i * 4 + 0), (uint8_t)(val));
        mem_write8(cpu->mem, digest + (uint32_t)(i * 4 + 1), (uint8_t)(val >> 8));
        mem_write8(cpu->mem, digest + (uint32_t)(i * 4 + 2), (uint8_t)(val >> 16));
        mem_write8(cpu->mem, digest + (uint32_t)(i * 4 + 3), (uint8_t)(val >> 24));
    }

    rom_return_void(cpu);
}

/* ===== PC hook ===== */

/* ===== PC hook hash table ===== */

static uint32_t hook_hash(uint32_t addr) {
    return (addr * 2654435761u) >> 21;  /* 11-bit index for 2048 slots */
}

static int hook_ht_lookup(const esp32_rom_stubs_t *s, uint32_t pc) {
    uint32_t h = hook_hash(pc) & HOOK_HT_MASK;
    for (int probe = 0; probe < 16; probe++) {
        uint32_t slot = (h + probe) & HOOK_HT_MASK;
        if (s->ht[slot].addr == pc) return s->ht[slot].idx;
        if (s->ht[slot].addr == 0) return -1;
    }
    return -1;
}

static void hook_bitmap_set(esp32_rom_stubs_t *s, uint32_t addr) {
    uint32_t idx = (addr >> 2) & (HOOK_BITMAP_BITS - 1);
    s->hook_bitmap[idx / 64] |= 1ULL << (idx & 63);
}

static void hook_ht_insert(esp32_rom_stubs_t *s, uint32_t addr, int idx) {
    uint32_t h = hook_hash(addr) & HOOK_HT_MASK;
    for (int probe = 0; probe < HOOK_HT_SIZE; probe++) {
        uint32_t slot = (h + probe) & HOOK_HT_MASK;
        if (s->ht[slot].addr == 0 || s->ht[slot].addr == addr) {
            s->ht[slot].addr = addr;
            s->ht[slot].idx = idx;
            hook_bitmap_set(s, addr);
            break;
        }
    }
    /* Also insert into direct dispatch table (overwrite on collision —
     * collisions fall back to hash via rom_pc_hook). */
    if (s->direct) {
        rom_stub_entry_t *e = &s->entries[idx];
        uint32_t di = (addr >> 2) & STUB_DIRECT_MASK;
        s->direct[di].tag = addr;
        s->direct[di].fn = e->fn;
        s->direct[di].ctx = e->user_ctx ? e->user_ctx : s;
        s->direct[di].call_count = &e->call_count;
    }
}

static int rom_pc_hook(xtensa_cpu_t *cpu, uint32_t pc, void *ctx) {
    esp32_rom_stubs_t *s = ctx;

    /* Fast path: direct dispatch table — single indexed lookup, no hash */
    if (__builtin_expect(s->direct != NULL, 1)) {
        uint32_t di = (pc >> 2) & STUB_DIRECT_MASK;
        stub_direct_entry_t *de = &s->direct[di];
        if (__builtin_expect(de->tag == pc, 1)) {
            s->total_calls++;
            (*de->call_count)++;
            de->fn(cpu, de->ctx);
            return 1;
        }
    }

    /* Slow path: hash table lookup (handles collisions, spy mode, logging) */
    int idx = hook_ht_lookup(s, pc);
    if (idx >= 0) {
        s->entries[idx].call_count++;
        s->total_calls++;
        if (s->log_fn)
            s->log_fn(s->log_ctx, pc, s->entries[idx].name, cpu);
        void *ectx = s->entries[idx].user_ctx ? s->entries[idx].user_ctx : s;
        s->entries[idx].fn(cpu, ectx);
        if (s->entries[idx].spy)
            return 0; /* spy: let original function execute */
        return 1;
    }
    /* Only intercept unregistered calls in ROM range */
    if (pc >= ROM_BASE && pc < ROM_END) {
        if (s->log_fn)
            s->log_fn(s->log_ctx, pc, "UNREGISTERED", cpu);
        s->unregistered_count++;
        stub_unregistered(cpu, s);
        return 1;
    }
    return 0;
}

/* ===== Public API ===== */

esp32_rom_stubs_t *rom_stubs_create(xtensa_cpu_t *cpu) {
    esp32_rom_stubs_t *s = calloc(1, sizeof(*s));
    if (!s) return NULL;
    s->cpu = cpu;
    s->cpu_freq_mhz = 160;

    /* Allocate direct dispatch table (64K entries × 24 bytes ≈ 1.5MB) */
    s->direct = calloc(STUB_DIRECT_SIZE, sizeof(stub_direct_entry_t));

    /* Install PC hook */
    cpu->pc_hook = rom_pc_hook;
    cpu->pc_hook_ctx = s;

    /* Pre-populate bitmap for entire ROM range (unregistered calls also intercepted) */
    for (uint32_t a = ROM_BASE; a < ROM_END; a += 4)
        hook_bitmap_set(s, a);
    cpu->pc_hook_bitmap = s->hook_bitmap;

    /* Pre-initialize g_ticks_per_us_pro so firmware doesn't divide by zero */
    mem_write32(cpu->mem, G_TICKS_PER_US_PRO, 160);

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
    rom_stubs_register(s, 0x4000c2c8, stub_memcpy,              "memcpy");
    rom_stubs_register(s, 0x4000c44c, stub_memset,              "memset");
    rom_stubs_register(s, 0x400014c0, stub_strlen,              "strlen");

    /* Boot-sequence stubs */
    rom_stubs_register(s, 0x4000689c, stub_ets_set_appcpu_boot_addr, "ets_set_appcpu_boot_addr");
    rom_stubs_register(s, 0x400081d4, stub_rtc_get_reset_reason, "rtc_get_reset_reason");
    rom_stubs_register(s, 0x40008550, stub_ets_update_cpu_frequency, "ets_update_cpu_frequency_rom");
    rom_stubs_register(s, 0x40007d28, stub_ets_install_uart_printf, "ets_install_uart_printf");

    /* String/memory functions */
    rom_stubs_register(s, 0x4000c3c0, stub_memmove,             "memmove");
    rom_stubs_register(s, 0x4000c260, stub_memcmp,              "memcmp");
    rom_stubs_register(s, 0x4000c1f4, stub_bzero,               "bzero");
    rom_stubs_register(s, 0x40001274, stub_strcmp,               "strcmp");
    rom_stubs_register(s, 0x400013ac, stub_strcpy,              "strcpy");
    rom_stubs_register(s, 0x400015d4, stub_strncpy,             "strncpy");
    rom_stubs_register(s, 0x4000c584, stub_strlcpy,             "strlcpy");

    /* Compiler builtins */
    rom_stubs_register(s, 0x40002ed0, stub_popcountsi2,        "__popcountsi2");
    rom_stubs_register(s, 0x40002ef8, stub_popcountdi2,        "__popcountdi2");
    rom_stubs_register(s, 0x4000c7e8, stub_clzsi2,             "__clzsi2");
    rom_stubs_register(s, 0x4000c7f0, stub_ctzsi2,             "__ctzsi2");
    rom_stubs_register(s, 0x4000c804, stub_ffssi2,             "__ffssi2");

    /* 64-bit division builtins */
    rom_stubs_register(s, 0x4000cff8, stub_udivdi3,            "__udivdi3");
    rom_stubs_register(s, 0x4000d280, stub_umoddi3,            "__umoddi3");
    rom_stubs_register(s, 0x4000ca84, stub_divdi3,             "__divdi3");
    rom_stubs_register(s, 0x4000cd4c, stub_moddi3,             "__moddi3");

    /* I2C ROM functions (register read/write - return 0) */
    rom_stubs_register(s, 0x40004148, stub_unregistered,        "rom_i2c_readReg");
    rom_stubs_register(s, 0x400041a4, stub_void_unregistered,   "rom_i2c_writeReg");
    rom_stubs_register(s, 0x400041c0, stub_unregistered,        "rom_i2c_readReg_Mask");
    rom_stubs_register(s, 0x400041fc, stub_void_unregistered,   "rom_i2c_writeReg_Mask");

    /* Interrupt matrix — real implementation if native mode, no-op otherwise */
    if (s->native_freertos)
        rom_stubs_register_ctx(s, 0x4000681c, stub_intr_matrix_set, "intr_matrix_set", s);
    else
        rom_stubs_register(s, 0x4000681c, stub_void_unregistered,   "intr_matrix_set");

    /* UART */
    rom_stubs_register(s, 0x40009200, stub_void_unregistered,   "uart_tx_one_char");
    rom_stubs_register(s, 0x40009258, stub_void_unregistered,   "uart_tx_flush");
    rom_stubs_register(s, 0x40009028, stub_void_unregistered,   "uart_tx_switch");

    /* GPIO */
    rom_stubs_register(s, 0x40009edc, stub_void_unregistered,   "gpio_matrix_in");
    rom_stubs_register(s, 0x40009fdc, stub_void_unregistered,   "gpio_pad_select_gpio");
    rom_stubs_register(s, 0x4000a22c, stub_void_unregistered,   "gpio_pad_pullup");

    /* MMU/Cache */
    rom_stubs_register(s, 0x400095a4, stub_void_unregistered,   "mmu_init");
    rom_stubs_register(s, 0x400095e0, stub_unregistered,        "cache_flash_mmu_set");

    /* C library functions */
    rom_stubs_register(s, 0x40056424, stub_qsort,              "qsort");
    rom_stubs_register(s, 0x400566b4, stub_itoa,               "itoa");
    rom_stubs_register(s, 0x40056678, stub_itoa,               "__itoa");
    rom_stubs_register(s, 0x4000c518, stub_strcat,             "strcat");

    /* Newlib stdio initialization */
    rom_stubs_register(s, 0x40001E38, stub_sinit,               "__sinit");
    rom_stubs_register(s, 0x40001150, stub_swrite,              "__swrite");
    rom_stubs_register(s, 0x40001E20, stub_void_unregistered,   "__sinit_lock_acquire");
    rom_stubs_register(s, 0x40001E2C, stub_void_unregistered,   "__sinit_lock_release");

    /* Newlib stdio flush */
    rom_stubs_register(s, 0x40059320, stub_fflush_r,             "_fflush_r");

    /* Soft-float double arithmetic */
    rom_stubs_register(s, 0x40002590, stub_adddf3,              "__adddf3");
    rom_stubs_register(s, 0x400026E4, stub_subdf3,              "__subdf3");
    rom_stubs_register(s, 0x4006358C, stub_muldf3,              "__muldf3");
    rom_stubs_register(s, 0x40002954, stub_divdf3,              "__divdf3");

    /* Soft-float double conversions */
    rom_stubs_register(s, 0x4000C938, stub_floatunsidf,         "__floatunsidf");
    rom_stubs_register(s, 0x4000C944, stub_floatsidf,           "__floatsidf");
    rom_stubs_register(s, 0x40002A78, stub_fixdfsi,             "__fixdfsi");
    rom_stubs_register(s, 0x40002B30, stub_fixunsdfsi,          "__fixunsdfsi");
    rom_stubs_register(s, 0x40002B90, stub_truncdfsf2,          "__truncdfsf2");
    rom_stubs_register(s, 0x40002C34, stub_extendsfdf2,         "__extendsfdf2");

    /* Soft-float double comparisons (all use same -1/0/1 logic except unord) */
    rom_stubs_register(s, 0x400636A8, stub_cmpdf,               "__nedf2");
    rom_stubs_register(s, 0x400636DC, stub_cmpdf,               "__gtdf2");
    rom_stubs_register(s, 0x40063704, stub_cmpdf,               "__ledf2");
    rom_stubs_register(s, 0x40063768, stub_cmpdf,               "__gedf2");
    rom_stubs_register(s, 0x40063790, stub_cmpdf,               "__ltdf2");
    rom_stubs_register(s, 0x400637F4, stub_unorddf2,            "__unorddf2");

    /* Byte-swap and 64-bit shift */
    rom_stubs_register(s, 0x40064AE0, stub_bswapsi2,            "__bswapsi2");
    rom_stubs_register(s, 0x4000C818, stub_ashldi3,             "__ashldi3");

    /* CRC */
    rom_stubs_register(s, 0x4005CFEC, stub_crc32_le,            "esp_rom_crc32_le");
    rom_stubs_register(s, 0x4005D144, stub_crc8,                "esp_crc8");

    /* DEFLATE decompression (used by PNG decoder via sped.c) */
    rom_stubs_register(s, 0x4005ef30, stub_tinfl_decompress,    "tinfl_decompress");

    /* Flash/boot helpers */
    rom_stubs_register(s, 0x40062BC8, stub_unregistered,        "spi_flash_clk_cfg");
    rom_stubs_register(s, 0x40009F0C, stub_void_unregistered,   "spi_flash_attach");
    rom_stubs_register(s, 0x40008264, stub_software_reset,      "software_reset_cpu");

    /* GPIO */
    rom_stubs_register(s, 0x4000C84C, stub_void_unregistered,   "gpio_output_set");

    /* Misc */
    rom_stubs_register(s, 0x40008208, stub_void_unregistered,   "set_rtc_memory_crc");
    /* _xtos_set_intlevel: in native mode, let firmware's RSIL instruction run */
    if (!s->native_freertos)
        rom_stubs_register(s, 0x4000bfdc, stub_unregistered,   "_xtos_set_intlevel");
    rom_stubs_register(s, 0x400092d0, stub_unregistered,        "uart_rx_one_char");
    rom_stubs_register(s, 0x4000c728, stub_void_unregistered,   "__dummy_lock");
    rom_stubs_register(s, 0x4000c730, stub_unregistered,        "__dummy_lock_try");

    /* MD5 ROM functions */
    rom_stubs_register(s, 0x4005DA7C, stub_md5_init,            "esp_rom_md5_init");
    rom_stubs_register(s, 0x4005DA9C, stub_md5_update,          "esp_rom_md5_update");
    rom_stubs_register(s, 0x4005DB1C, stub_md5_final,           "esp_rom_md5_final");

    /* POSIX syscall stubs (used by VFS / mbedTLS for socket I/O) */
    rom_stubs_register(s, 0x40001778, stub_void_unregistered,   "close");
    rom_stubs_register(s, 0x400017DC, stub_unregistered,        "read");
    rom_stubs_register(s, 0x4000181C, stub_unregistered,        "write");

    return s;
}

void rom_stubs_destroy(esp32_rom_stubs_t *stubs) {
    if (!stubs) return;
    /* Unhook */
    if (stubs->cpu->pc_hook == rom_pc_hook) {
        stubs->cpu->pc_hook = NULL;
        stubs->cpu->pc_hook_ctx = NULL;
        stubs->cpu->pc_hook_bitmap = NULL;
    }
    free(stubs->direct);
    free(stubs);
}

/* Hook firmware functions by symbol name from ELF.
 * This allows stubbing functions that live in the loaded firmware
 * (not ROM) — needed for things like newlib lock functions. */
int rom_stubs_hook_symbols(esp32_rom_stubs_t *stubs,
                           const elf_symbols_t *syms) {
    if (!stubs || !syms) return 0;
    stubs->syms = syms;
    int hooked = 0;

    /* Newlib lock functions — no-op in single-threaded emulator */
    static const char *lock_fns[] = {
        "__retarget_lock_init",
        "__retarget_lock_init_recursive",
        "__retarget_lock_close",
        "__retarget_lock_close_recursive",
        "__retarget_lock_acquire",
        "__retarget_lock_acquire_recursive",
        "__retarget_lock_try_acquire",
        "__retarget_lock_try_acquire_recursive",
        "__retarget_lock_release",
        "__retarget_lock_release_recursive",
        /* ESP-IDF internal lock functions (use FreeRTOS semaphores) */
        "_lock_init",
        "_lock_init_recursive",
        "_lock_close",
        "_lock_close_recursive",
        "_lock_acquire",
        "_lock_acquire_recursive",
        "_lock_try_acquire",
        "_lock_try_acquire_recursive",
        "_lock_release",
        "_lock_release_recursive",
        NULL
    };
    for (int i = 0; lock_fns[i]; i++) {
        uint32_t addr;
        if (elf_symbols_find(syms, lock_fns[i], &addr) == 0) {
            rom_stubs_register(stubs, addr, stub_void_unregistered, lock_fns[i]);
            hooked++;
        }
    }

    /* NVS stubs */
    struct { const char *name; rom_stub_fn fn; } nvs_hooks[] = {
        { "nvs_flash_init",             stub_nvs_flash_init },
        { "nvs_flash_init_partition",   stub_nvs_flash_init },
        { "nvs_flash_erase",            stub_nvs_flash_erase },
        { "nvs_open",                   stub_nvs_open },
        { "nvs_open_from_partition",    stub_nvs_open_from_partition },
        { "nvs_close",                  stub_nvs_close },
        { "nvs_get_i8",                 stub_nvs_get_notfound },
        { "nvs_get_u8",                 stub_nvs_get_notfound },
        { "nvs_get_i16",                stub_nvs_get_notfound },
        { "nvs_get_u16",                stub_nvs_get_notfound },
        { "nvs_get_i32",                stub_nvs_get_notfound },
        { "nvs_get_u32",                stub_nvs_get_notfound },
        { "nvs_get_i64",                stub_nvs_get_notfound },
        { "nvs_get_u64",                stub_nvs_get_notfound },
        { "nvs_get_str",                stub_nvs_get_notfound },
        { "nvs_get_blob",               stub_nvs_get_notfound },
        { "nvs_set_i8",                 stub_nvs_set_ok },
        { "nvs_set_u8",                 stub_nvs_set_ok },
        { "nvs_set_i16",                stub_nvs_set_ok },
        { "nvs_set_u16",                stub_nvs_set_ok },
        { "nvs_set_i32",                stub_nvs_set_ok },
        { "nvs_set_u32",                stub_nvs_set_ok },
        { "nvs_set_i64",                stub_nvs_set_ok },
        { "nvs_set_u64",                stub_nvs_set_ok },
        { "nvs_set_str",                stub_nvs_set_ok },
        { "nvs_set_blob",               stub_nvs_set_ok },
        { "nvs_commit",                 stub_nvs_set_ok },
        { NULL, NULL }
    };
    for (int i = 0; nvs_hooks[i].name; i++) {
        uint32_t addr;
        if (elf_symbols_find(syms, nvs_hooks[i].name, &addr) == 0) {
            rom_stubs_register(stubs, addr, nvs_hooks[i].fn, nvs_hooks[i].name);
            hooked++;
        }
    }

    /* GPIO driver stubs */
    struct { const char *name; rom_stub_fn fn; } gpio_hooks[] = {
        { "gpio_config",                stub_gpio_config },
        { "gpio_set_direction",         stub_gpio_set_direction },
        { "gpio_set_level",             stub_gpio_set_level },
        { "gpio_get_level",             stub_gpio_get_level },
        { "gpio_reset_pin",             stub_gpio_reset_pin },
        { "gpio_install_isr_service",   stub_gpio_isr_service },
        { "gpio_isr_handler_add",       stub_gpio_isr_service },
        { "gpio_isr_handler_remove",    stub_gpio_isr_service },
        { "gpio_set_intr_type",         stub_gpio_isr_service },
        { "gpio_intr_enable",           stub_gpio_isr_service },
        { "gpio_intr_disable",          stub_gpio_isr_service },
        { "gpio_pullup_en",             stub_gpio_isr_service },
        { "gpio_pullup_dis",            stub_gpio_isr_service },
        { "gpio_pulldown_en",           stub_gpio_isr_service },
        { "gpio_pulldown_dis",          stub_gpio_isr_service },
        { "gpio_set_pull_mode",         stub_gpio_isr_service },
        { NULL, NULL }
    };
    for (int i = 0; gpio_hooks[i].name; i++) {
        uint32_t addr;
        if (elf_symbols_find(syms, gpio_hooks[i].name, &addr) == 0) {
            rom_stubs_register(stubs, addr, gpio_hooks[i].fn, gpio_hooks[i].name);
            hooked++;
        }
    }

    /* Heap allocator stubs */
    struct { const char *name; rom_stub_fn fn; } alloc_hooks[] = {
        { "malloc",                         stub_malloc },
        { "calloc",                         stub_calloc },
        { "free",                           stub_free },
        { "realloc",                        stub_realloc },
        { "heap_caps_malloc",               stub_malloc },
        { "heap_caps_malloc_default",       stub_malloc },
        { "heap_caps_free",                 stub_free },
        { "heap_caps_realloc",              stub_realloc },
        { "heap_caps_realloc_default",      stub_realloc },
        { "heap_caps_calloc",               stub_calloc },
        { NULL, NULL }
    };
    for (int i = 0; alloc_hooks[i].name; i++) {
        uint32_t addr;
        if (elf_symbols_find(syms, alloc_hooks[i].name, &addr) == 0) {
            rom_stubs_register(stubs, addr, alloc_hooks[i].fn, alloc_hooks[i].name);
            hooked++;
        }
    }

    /* RNG stubs */
    struct { const char *name; rom_stub_fn fn; } rng_hooks[] = {
        { "esp_random",                    stub_esp_random },
        { "esp_fill_random",               stub_esp_fill_random },
        { "hal_random",                    stub_hal_random },
        { "bootloader_fill_random",        stub_bootloader_fill_random },
        { NULL, NULL }
    };
    for (int i = 0; rng_hooks[i].name; i++) {
        uint32_t addr;
        if (elf_symbols_find(syms, rng_hooks[i].name, &addr) == 0) {
            rom_stubs_register(stubs, addr, rng_hooks[i].fn, rng_hooks[i].name);
            hooked++;
        }
    }

    /* Heap size + logging stubs */
    struct { const char *name; rom_stub_fn fn; } misc_hooks[] = {
        { "esp_get_free_heap_size",        stub_esp_get_free_heap_size },
        { "esp_get_minimum_free_heap_size", stub_esp_get_minimum_free_heap_size },
        { "esp_log_timestamp",             stub_esp_log_timestamp },
        { "esp_log_early_timestamp",       stub_esp_log_timestamp },
        { NULL, NULL }
    };
    for (int i = 0; misc_hooks[i].name; i++) {
        uint32_t addr;
        if (elf_symbols_find(syms, misc_hooks[i].name, &addr) == 0) {
            rom_stubs_register(stubs, addr, misc_hooks[i].fn, misc_hooks[i].name);
            hooked++;
        }
    }

    /* Look up multicore BSS symbols for unblocking startup waits */
    {
        uint32_t addr;
        if (elf_symbols_find(syms, "s_cpu_up", &addr) == 0)
            stubs->s_cpu_up_addr = addr;
        if (elf_symbols_find(syms, "s_cpu_inited", &addr) == 0)
            stubs->s_cpu_inited_addr = addr;
        if (elf_symbols_find(syms, "s_system_inited", &addr) == 0)
            stubs->s_system_inited_addr = addr;
        if (elf_symbols_find(syms, "s_system_full_inited", &addr) == 0)
            stubs->s_system_full_inited_addr = addr;
        if (elf_symbols_find(syms, "app_main", &addr) == 0)
            stubs->app_main_addr = addr;
        if (elf_symbols_find(syms, "__stack_chk_guard", &addr) == 0)
            stubs->stack_chk_guard_addr = addr;
        if (elf_symbols_find(syms, "s_resume_cores", &addr) == 0)
            stubs->s_resume_cores_addr = addr;
    }

    /* Don't hook start_cpu0 — let it run naturally so __init_array
     * constructors execute (needed for C++ global objects like Serial).
     * esp_startup_start_app (called at the end of start_cpu0) redirects
     * to app_main instead of starting the FreeRTOS scheduler. */

    /* SPI flash init stubs — skip actual SPI communication, flash data is
     * already memory-mapped.  Return ESP_OK (0). */
    static const char *flash_init_fns[] = {
        "esp_flash_init_main",
        "esp_flash_init_default_chip",
        "esp_flash_read_chip_id",
        "spi_flash_init_chip_state",
        "spi_flash_op_lock",
        "spi_flash_op_unlock",
        "spi_flash_op_block_func",
        NULL
    };
    for (int i = 0; flash_init_fns[i]; i++) {
        uint32_t addr;
        if (elf_symbols_find(syms, flash_init_fns[i], &addr) == 0) {
            rom_stubs_register(stubs, addr, stub_unregistered, flash_init_fns[i]);
            hooked++;
        }
    }

    /* ESP-IDF SPI + LCD panel stubs (no-ops, display handled at higher level) */
    static const char *lcd_noop_fns[] = {
        "spi_bus_initialize",
        "esp_lcd_new_panel_io_spi",
        "esp_lcd_new_panel_st7789",
        "esp_lcd_panel_reset",
        "esp_lcd_panel_init",
        "esp_lcd_panel_swap_xy",
        "esp_lcd_panel_mirror",
        "esp_lcd_panel_disp_on_off",
        "esp_lcd_panel_draw_bitmap",
        "esp_lcd_panel_io_tx_param",
        "esp_lcd_panel_io_tx_color",
        NULL
    };
    for (int i = 0; lcd_noop_fns[i]; i++) {
        uint32_t addr;
        if (elf_symbols_find(syms, lcd_noop_fns[i], &addr) == 0) {
            rom_stubs_register(stubs, addr, stub_unregistered, lcd_noop_fns[i]);
            hooked++;
        }
    }

    /* Interrupt allocation / management — skip in native mode so firmware
     * runs its own esp_intr_alloc, which programs the real interrupt matrix */
    if (!stubs->native_freertos) {
        static const char *intr_fns[] = {
            "esp_intr_alloc",
            "esp_intr_alloc_intrstatus",
            "esp_intr_free",
            "esp_intr_disable",
            "esp_intr_enable",
            NULL
        };
        for (int i = 0; intr_fns[i]; i++) {
            uint32_t addr;
            if (elf_symbols_find(syms, intr_fns[i], &addr) == 0) {
                rom_stubs_register(stubs, addr, stub_unregistered, intr_fns[i]);
                hooked++;
            }
        }
    }

    /* Chip info + error names */
    {
        uint32_t addr;
        if (elf_symbols_find(syms, "esp_chip_info", &addr) == 0) {
            rom_stubs_register(stubs, addr, stub_esp_chip_info, "esp_chip_info");
            hooked++;
        }
        if (elf_symbols_find(syms, "esp_err_to_name", &addr) == 0) {
            rom_stubs_register(stubs, addr, stub_esp_err_to_name, "esp_err_to_name");
            hooked++;
        }
    }

    /* Logging */
    {
        uint32_t addr;
        /* esp_log_write - implement properly to show ESP_LOGI/ESP_LOGW/etc output */
        if (elf_symbols_find(syms, "esp_log_write", &addr) == 0) {
            rom_stubs_register(stubs, addr, stub_esp_log_write, "esp_log_write");
            hooked++;
        }
        /* esp_log_writev - similar to esp_log_write but with va_list, use same stub */
        if (elf_symbols_find(syms, "esp_log_writev", &addr) == 0) {
            rom_stubs_register(stubs, addr, stub_esp_log_write, "esp_log_writev");
            hooked++;
        }
        /* esp_log_level_set - no-op */
        if (elf_symbols_find(syms, "esp_log_level_set", &addr) == 0) {
            rom_stubs_register(stubs, addr, stub_void_unregistered, "esp_log_level_set");
            hooked++;
        }
    }

    /* Partition API */
    {
        uint32_t addr;
        if (elf_symbols_find(syms, "esp_partition_find_first", &addr) == 0) {
            rom_stubs_register(stubs, addr, stub_esp_partition_find_first,
                               "esp_partition_find_first");
            hooked++;
        }
        if (elf_symbols_find(syms, "esp_partition_mmap", &addr) == 0) {
            rom_stubs_register(stubs, addr, stub_esp_partition_mmap,
                               "esp_partition_mmap");
            hooked++;
        }
        if (elf_symbols_find(syms, "esp_ota_get_running_partition", &addr) == 0) {
            rom_stubs_register_ctx(stubs, addr,
                                   stub_esp_ota_get_running_partition,
                                   "esp_ota_get_running_partition", stubs);
            hooked++;
        }
    }

    /* App startup — let esp_startup_start_app run NATURALLY so that
     * esp_startup_start_app_common() iterates __init_array and runs
     * C++ global constructors (sets up vtables for Serial2, etc.).
     * After constructors run, it calls vTaskStartScheduler which
     * our FreeRTOS stubs handle. */
    {
        uint32_t addr;
        if (elf_symbols_find(syms, "startup_resume_other_cores", &addr) == 0) {
            rom_stubs_register(stubs, addr, stub_startup_resume_other_cores,
                               "startup_resume_other_cores");
            hooked++;
        }
        if (elf_symbols_find(syms, "do_system_init_fn", &addr) == 0) {
            rom_stubs_register(stubs, addr, stub_do_system_init_fn,
                               "do_system_init_fn");
            hooked++;
        }
        /* In single-core mode, main_task polls s_other_cpu_startup_done
         * waiting for core 1.  Store address; do_system_init_fn writes it
         * (after BSS is zeroed so the value sticks). */
        if (elf_symbols_find(syms, "s_other_cpu_startup_done", &addr) == 0) {
            stubs->s_other_cpu_startup_done_addr = addr;
        }
        /* esp_register_freertos_idle_hook_for_cpu — no-op (no idle task) */
        if (elf_symbols_find(syms, "esp_register_freertos_idle_hook_for_cpu", &addr) == 0) {
            rom_stubs_register(stubs, addr, stub_unregistered,
                               "esp_register_freertos_idle_hook_for_cpu");
            hooked++;
        }
    }

    /* VFS stubs */
    static const char *vfs_fns[] = {
        "esp_vfs_register_fd_range",
        "esp_vfs_dev_uart_register",
        "esp_vfs_console_register",
        "esp_vfs_null_register",
        "esp_vfs_fat_register",
        "esp_vfs_fat_unregister_path",
        "esp_vfs_register",
        "esp_vfs_register_with_id",
        "esp_vfs_unregister",
        "esp_vfs_unregister_with_id",
        NULL
    };
    for (int i = 0; vfs_fns[i]; i++) {
        uint32_t addr;
        if (elf_symbols_find(syms, vfs_fns[i], &addr) == 0) {
            rom_stubs_register(stubs, addr, stub_unregistered, vfs_fns[i]);
            hooked++;
        }
    }

    /* UART driver stubs — return ESP_OK so Serial.begin() succeeds */
    static const char *uart_fns[] = {
        "uart_driver_install",
        "uart_driver_delete",
        "uart_param_config",
        "uart_set_pin",
        "uart_set_baudrate",
        "uart_get_baudrate",
        "uart_set_word_length",
        "uart_set_stop_bits",
        "uart_set_parity",
        "uart_set_hw_flow_ctrl",
        "uart_set_sw_flow_ctrl",
        "uart_wait_tx_done",
        "uart_tx_chars",
        "uart_write_bytes",
        "uart_read_bytes",
        "uart_flush",
        "uart_flush_input",
        NULL
    };
    for (int i = 0; uart_fns[i]; i++) {
        uint32_t addr;
        if (elf_symbols_find(syms, uart_fns[i], &addr) == 0) {
            rom_stubs_register(stubs, addr, stub_unregistered, uart_fns[i]);
            hooked++;
        }
    }

    /* Arduino UART wrapper — uartBegin returns a uart_t* (non-null = success).
     * It calls uart_driver_install internally; by stubbing the driver fns above
     * AND uartBegin itself, we avoid the ESP_ERROR_CHECK panic. */
    {
        uint32_t addr;
        if (elf_symbols_find(syms, "uartBegin", &addr) == 0) {
            rom_stubs_register(stubs, addr, stub_uartBegin, "uartBegin");
            hooked++;
        }
        if (elf_symbols_find(syms, "uartIsDriverInstalled", &addr) == 0) {
            rom_stubs_register(stubs, addr, stub_ret_true,
                               "uartIsDriverInstalled");
            hooked++;
        }
        /* uartAvailable, uartRead, uartWrite — common HardwareSerial helpers */
        if (elf_symbols_find(syms, "uartAvailable", &addr) == 0) {
            rom_stubs_register(stubs, addr, stub_unregistered,
                               "uartAvailable");
            hooked++;
        }
        if (elf_symbols_find(syms, "uartEnd", &addr) == 0) {
            rom_stubs_register(stubs, addr, stub_void_unregistered,
                               "uartEnd");
            hooked++;
        }
    }

    /* SD card mount (Arduino SD library) — return ESP_OK to skip retry loop */
    {
        uint32_t addr;
        if (elf_symbols_find(syms, "_Z12sdcard_mounthPKchb", &addr) == 0) {
            rom_stubs_register(stubs, addr, stub_unregistered,
                               "_Z12sdcard_mounthPKchb");
            hooked++;
        }
    }

    /* PSRAM / SPIRAM stubs — return ESP_OK so firmware thinks PSRAM is available.
     * The emulator already has PSRAM memory region mapped. */
    static const char *spiram_fns[] = {
        "esp_spiram_init",
        "esp_spiram_init_cache",
        "esp_spiram_add_to_heapalloc",
        "esp_spiram_get_size",
        "psram_enable",
        NULL
    };
    for (int i = 0; spiram_fns[i]; i++) {
        uint32_t addr;
        if (elf_symbols_find(syms, spiram_fns[i], &addr) == 0) {
            rom_stubs_register(stubs, addr, stub_unregistered, spiram_fns[i]);
            hooked++;
        }
    }

    /* Pthread stubs — skip in native mode so firmware runs its own locking.
     * In stub mode: single-threaded, all ops succeed (return 0). */
    if (!stubs->native_freertos) {
        static const char *pthread_fns[] = {
            "pthread_mutex_init",
            "pthread_mutex_destroy",
            "pthread_mutex_lock",
            "pthread_mutex_unlock",
            "pthread_mutex_lock_internal",
            "pthread_mutex_init_if_static$part$3",
            NULL
        };
        for (int i = 0; pthread_fns[i]; i++) {
            uint32_t addr;
            if (elf_symbols_find(syms, pthread_fns[i], &addr) == 0) {
                rom_stubs_register(stubs, addr, stub_unregistered, pthread_fns[i]);
                hooked++;
            }
        }
    }

    /* esp_newlib_init — needs special handling to set up _reent struct */
    {
        uint32_t addr;
        if (elf_symbols_find(syms, "esp_newlib_init", &addr) == 0) {
            rom_stubs_register_ctx(stubs, addr, stub_esp_newlib_init,
                                   "esp_newlib_init", stubs);
            hooked++;
        }
        if (elf_symbols_find(syms, "__getreent", &addr) == 0) {
            rom_stubs_register(stubs, addr, stub_getreent, "__getreent");
            hooked++;
        }
    }

    /* Misc ESP-IDF init functions (all return 0 / void) */
    static const char *init_ret0_fns[] = {
        "esp_newlib_init_global_stdio",
        "esp_newlib_time_init",
        "esp_time_impl_init",
        "esp_timer_impl_early_init",
        "esp_timer_impl_init_system_time",
        "esp_int_wdt_init",
        "esp_int_wdt_cpu_init",
        "esp_task_wdt_init",
        "esp_task_wdt_add",
        "esp_task_wdt_reset",
        "esp_crosscore_int_init",
        "esp_cache_err_int_init",
        "esp_ipc_isr_init",
        "esp_register_shutdown_handler",
        "esp_brownout_init",
        "esp_core_dump_init",
        "esp_core_dump_flash_init",
        "esp_core_dump_to_flash",
        "esp_spiffs_mounted",
        NULL
    };
    for (int i = 0; init_ret0_fns[i]; i++) {
        uint32_t addr;
        if (elf_symbols_find(syms, init_ret0_fns[i], &addr) == 0) {
            rom_stubs_register(stubs, addr, stub_unregistered, init_ret0_fns[i]);
            hooked++;
        }
    }

    /* SPIFFS stubs — hook SPIFFSFS::begin() to return false immediately.
     * This prevents the mount→format→retry error spam AND avoids file-open
     * errors.  Firmware falls back to hardcoded defaults silently.
     * Lower-level stubs kept as safety net for direct callers. */
    {
        uint32_t addr;
        if (elf_symbols_find(syms, "_ZN2fs8SPIFFSFS5beginEbPKchS2_", &addr) == 0) {
            rom_stubs_register(stubs, addr, stub_unregistered,
                               "_ZN2fs8SPIFFSFS5beginEbPKchS2_");
            hooked++;
        }
        if (elf_symbols_find(syms, "_ZN2fs8SPIFFSFS6formatEv", &addr) == 0) {
            rom_stubs_register(stubs, addr, stub_unregistered,
                               "_ZN2fs8SPIFFSFS6formatEv");
            hooked++;
        }
    }
    static const char *spiffs_fail_fns[] = {
        "esp_vfs_spiffs_register",
        "esp_vfs_spiffs_unregister",
        "esp_spiffs_init",
        "esp_spiffs_format",
        "esp_spiffs_info",
        "esp_spiffs_check",
        NULL
    };
    for (int i = 0; spiffs_fail_fns[i]; i++) {
        uint32_t addr;
        if (elf_symbols_find(syms, spiffs_fail_fns[i], &addr) == 0) {
            rom_stubs_register(stubs, addr, stub_ret_esp_fail, spiffs_fail_fns[i]);
            hooked++;
        }
    }

    /* WiFi / network stubs — return ESP_OK (0) so init succeeds.
     * NerdMiner and similar firmwares check the return value and fall into
     * blocking captive-portal loops when esp_netif_init() returns ESP_FAIL. */
    static const char *wifi_ok_fns[] = {
        "esp_wifi_init",
        "esp_wifi_init_internal",
        "esp_wifi_start",
        "esp_wifi_stop",
        "esp_wifi_connect",
        "esp_wifi_disconnect",
        "esp_netif_init",
        "tcpip_adapter_init",
        "tcpip_send_msg_wait_sem",
        NULL
    };
    for (int i = 0; wifi_ok_fns[i]; i++) {
        uint32_t addr;
        if (elf_symbols_find(syms, wifi_ok_fns[i], &addr) == 0) {
            rom_stubs_register(stubs, addr, stub_unregistered, wifi_ok_fns[i]);
            hooked++;
        }
    }

    /* tcpip_init — return 0 */
    {
        uint32_t addr;
        if (elf_symbols_find(syms, "tcpip_init", &addr) == 0) {
            rom_stubs_register(stubs, addr, stub_unregistered, "tcpip_init");
            hooked++;
        }
    }

    /* Set tcpip_mbox to non-zero so tcpip_callback's
     * sys_mbox_valid() assertion passes. tcpip_init may never be called
     * because our stubs skip the real init, but lwip_setsockopt still
     * calls tcpip_callback directly which asserts mbox validity. */
    {
        uint32_t mbox_addr;
        if (elf_symbols_find(syms, "tcpip_mbox", &mbox_addr) == 0) {
            mem_write32(stubs->cpu->mem, mbox_addr, 0xDEAD0001u);
        }
    }

    /* Stub tcpip_callback too — it tries to post to the mbox which
     * doesn't really exist. Just return ERR_OK (0). */
    {
        uint32_t addr;
        if (elf_symbols_find(syms, "tcpip_callback", &addr) == 0) {
            rom_stubs_register(stubs, addr, stub_unregistered, "tcpip_callback");
            hooked++;
        }
    }

    /* __esp_stack_guard_setup — no-op to prevent __stack_chk_guard from
     * being set to a random value during __init_array.  Keeps the guard
     * at zero so canary checks always pass (no real stack protection needed
     * in the emulator). */
    {
        uint32_t addr;
        if (elf_symbols_find(syms, "__esp_stack_guard_setup", &addr) == 0) {
            rom_stubs_register(stubs, addr, stub_unregistered,
                               "__esp_stack_guard_setup");
            hooked++;
        }
        /* __stack_chk_fail — skip back to caller's retw.n.  Window spill/fill
         * corruption can trigger false canary mismatches; ignore them. */
        if (elf_symbols_find(syms, "__stack_chk_fail", &addr) == 0) {
            rom_stubs_register(stubs, addr, stub_stack_chk_fail_skip,
                               "__stack_chk_fail");
            hooked++;
        }
    }

    /* FreeType stubs — return non-zero (error) to skip font loading */
    static const char *ft_fail_fns[] = {
        "FT_Init_FreeType",
        "FT_New_Face",
        "FT_New_Memory_Face",
        "FT_Open_Face",
        NULL
    };
    for (int i = 0; ft_fail_fns[i]; i++) {
        uint32_t addr;
        if (elf_symbols_find(syms, ft_fail_fns[i], &addr) == 0) {
            rom_stubs_register(stubs, addr, stub_ret_esp_fail, ft_fail_fns[i]);
            hooked++;
        }
    }

    /* Arduino framework GPIO wrappers.
     * __pinMode calls gpio_set_direction whose ENTRY instruction sits right
     * before the gpio_config symbol — instruction alignment means our
     * gpio_config hook is never hit.  Hook at the Arduino wrapper level. */
    static const char *arduino_gpio_fns[] = {
        "__pinMode",
        "pinMode",
        "__digitalWrite",
        "digitalWrite",
        "__analogRead",
        "analogRead",
        "analogWrite",
        "ledcSetup",
        "ledcAttachPin",
        "ledcWrite",
        NULL
    };
    for (int i = 0; arduino_gpio_fns[i]; i++) {
        uint32_t addr;
        if (elf_symbols_find(syms, arduino_gpio_fns[i], &addr) == 0) {
            rom_stubs_register(stubs, addr, stub_unregistered, arduino_gpio_fns[i]);
            hooked++;
        }
    }
    /* digitalRead returns HIGH (1) — buttons are active-low, so returning 0
     * makes firmware think buttons are pressed (triggering WiFi reset etc.) */
    static const char *digital_read_fns[] = {
        "__digitalRead", "digitalRead", NULL
    };
    for (int i = 0; digital_read_fns[i]; i++) {
        uint32_t addr;
        if (elf_symbols_find(syms, digital_read_fns[i], &addr) == 0) {
            rom_stubs_register(stubs, addr, stub_digital_read_high, digital_read_fns[i]);
            hooked++;
        }
    }

    /* esp_panic_handler — dump exception info before stopping */
    {
        uint32_t addr;
        if (elf_symbols_find(syms, "esp_panic_handler", &addr) == 0) {
            rom_stubs_register(stubs, addr, stub_panic_handler, "esp_panic_handler");
            hooked++;
        }
    }

    /* esp_restart — halt CPU instead of rebooting (which re-enters firmware
     * with stale ccount, causing infinite restart loops) */
    static const char *restart_fns[] = {
        "esp_restart", "esp_restart_noos",
        "_ZN8EspClass7restartEv",  /* ESP.restart() */
        NULL
    };
    for (int i = 0; restart_fns[i]; i++) {
        uint32_t addr;
        if (elf_symbols_find(syms, restart_fns[i], &addr) == 0) {
            rom_stubs_register(stubs, addr, stub_abort_stop, restart_fns[i]);
            hooked++;
        }
    }

    /* Arduino Serial (HardwareSerial) — no-op to avoid UART register access.
     * begin() signature varies by Arduino core version; hook both variants. */
    static const char *serial_noop_fns[] = {
        "_ZN14HardwareSerial5beginEmh",        /* begin(unsigned long, uint8_t) — older core */
        "_ZN14HardwareSerial5beginEmjaabmh",   /* begin(unsigned long, uint32_t, int8_t, int8_t, bool, unsigned long, uint8_t) — Arduino ESP32 2.x */
        "_ZN14HardwareSerial3endEb",           /* end(bool) */
        "_ZN14HardwareSerial3endEv",           /* end() */
        "_ZN14HardwareSerial9availableEv",     /* available() */
        "_ZN14HardwareSerial17availableForWriteEv", /* availableForWrite() */
        "_ZN14HardwareSerial4peekEv",          /* peek() */
        "_ZN14HardwareSerial5flushEv",         /* flush() */
        "_ZN14HardwareSerial16_createEventTaskEPv", /* _createEventTask(void*) */
        "_ZN14HardwareSerial17_destroyEventTaskEv",  /* _destroyEventTask() */
        "_ZN14HardwareSerial14_uartEventTaskEPv",    /* _uartEventTask(void*) */
        "_ZN14HardwareSerialD1Ev",             /* destructor */
        "_ZN14HardwareSerialD2Ev",             /* destructor */
        "_ZN14HardwareSerialD0Ev",             /* destructor */
        NULL
    };
    for (int i = 0; serial_noop_fns[i]; i++) {
        uint32_t addr;
        if (elf_symbols_find(syms, serial_noop_fns[i], &addr) == 0) {
            rom_stubs_register(stubs, addr, stub_unregistered, serial_noop_fns[i]);
            hooked++;
        }
    }
    /* HardwareSerial::operator bool() — must return true so `while (!Serial)` exits */
    {
        uint32_t addr;
        if (elf_symbols_find(syms, "_ZNK14HardwareSerialcvbEv", &addr) == 0) {
            rom_stubs_register(stubs, addr, stub_ret_true, "HardwareSerial::operator bool");
            hooked++;
        }
    }
    /* HardwareSerial::write — return byte count (arg2) or 1 */
    {
        uint32_t addr;
        if (elf_symbols_find(syms, "_ZN14HardwareSerial5writeEh", &addr) == 0) {
            rom_stubs_register(stubs, addr, stub_ret_true, "HardwareSerial::write(uint8_t)");
            hooked++;
        }
        if (elf_symbols_find(syms, "_ZN14HardwareSerial5writeEPKhj", &addr) == 0) {
            rom_stubs_register(stubs, addr, stub_serial_write_buf, "HardwareSerial::write(buf,len)");
            hooked++;
        }
    }
    /* HardwareSerial::read / readBytes — return 0 / -1 */
    {
        uint32_t addr;
        if (elf_symbols_find(syms, "_ZN14HardwareSerial4readEv", &addr) == 0) {
            rom_stubs_register(stubs, addr, stub_ret_neg1, "HardwareSerial::read()");
            hooked++;
        }
        if (elf_symbols_find(syms, "_ZN14HardwareSerial9readBytesEPhj", &addr) == 0) {
            rom_stubs_register(stubs, addr, stub_unregistered, "HardwareSerial::readBytes(uint8_t*)");
            hooked++;
        }
        if (elf_symbols_find(syms, "_ZN14HardwareSerial9readBytesEPcj", &addr) == 0) {
            rom_stubs_register(stubs, addr, stub_unregistered, "HardwareSerial::readBytes(char*)");
            hooked++;
        }
    }

    /* Arduino UART C wrappers (esp32-hal-uart.c) — intercept remaining functions
     * that access UART peripheral registers */
    static const char *uart_hal_fns[] = {
        "uartSetRxTimeout",
        "uartSetRxFIFOFull",
        "uartWrite",
        "uartWriteBuf",
        "uartReadBytes",
        "uartPeek",
        "uartFlushTxOnly",
        "uartAvailableForWrite",
        "uartDetachPins",
        "uartDetectBaudrate",
        "uartStartDetectBaudrate",
        "uartBaudrateDetect",
        "uartSetDebug",
        "uartGetDebug",
        "uartGetEventQueue",
        "uart_install_putc",
        NULL
    };
    for (int i = 0; uart_hal_fns[i]; i++) {
        uint32_t addr;
        if (elf_symbols_find(syms, uart_hal_fns[i], &addr) == 0) {
            rom_stubs_register(stubs, addr, stub_unregistered, uart_hal_fns[i]);
            hooked++;
        }
    }

    /* GPS interface — no GPS hardware in emulator.  Hook begin() to prevent
     * MicroNMEA::sendSentence from doing a virtual dispatch on Serial2 which
     * may have a corrupted vtable (esp32-hal writes to the object). */
    static const char *gps_fns[] = {
        "_ZN12GpsInterface5beginEv",
        "_ZN12GpsInterface18flush_queue_textinEv",
        "_ZN12GpsInterface16flush_queue_nmeaEv",
        "_ZN12GpsInterface7enqueueER9MicroNMEA",
        NULL
    };
    for (int i = 0; gps_fns[i]; i++) {
        uint32_t addr;
        if (elf_symbols_find(syms, gps_fns[i], &addr) == 0) {
            rom_stubs_register(stubs, addr, stub_unregistered, gps_fns[i]);
            hooked++;
        }
    }

    /* Arduino SPI transfer functions — these poll SPI peripheral registers
     * which we don't emulate.  Hook them to return immediately.
     * The NL (No Lock) variants are the innermost transfer functions.
     * For display drivers like TFT_eSPI, these write commands/data to the
     * ILI9341/ST7789.  We stub them as no-ops (return 0). */
    static const char *spi_transfer_fns[] = {
        "spiTransferByteNL", "spiTransferShortNL", "spiTransferLongNL",
        "spiTransferBytesNL", "spiTransferByte", "spiTransferWord",
        "spiTransferLong", "spiTransferBytes", "spiWriteByteNL",
        "spiWriteShortNL", "spiWriteLongNL", "spiWritePixelsNL",
        "spiWriteNL", "spiWriteByte", "spiWriteWord",
        "spi_transfer",
        "_Z8Sspixfert", /* TFT_eSPI inline SPI pixel transfer */
        "_Z6sdWaithi", "_Z9sdCommandhcjPj",
        "_Z11sdReadByteshPci", "_Z6sdStoph",
        NULL
    };
    for (int i = 0; spi_transfer_fns[i]; i++) {
        uint32_t addr;
        if (elf_symbols_find(syms, spi_transfer_fns[i], &addr) == 0) {
            rom_stubs_register(stubs, addr, stub_unregistered, spi_transfer_fns[i]);
            hooked++;
        }
    }

    /* ff_sd_initialize — return STA_NODISK|STA_NOINIT when no image
     * is configured.  When the sdcard_stubs module has an image, the
     * higher-level SDMMC hooks handle I/O. */
    {
        uint32_t addr;
        if (elf_symbols_find(syms, "_Z16ff_sd_initializeh", &addr) == 0) {
            rom_stubs_register(stubs, addr, stub_ret_sd_nodisk,
                               "_Z16ff_sd_initializeh");
            hooked++;
        }
    }

    /* WiFi manager stubs — skip WiFi setup entirely for NerdMiner etc.
     * These are C++ mangled names; only match firmwares with these symbols. */
    static const struct { const char *name; rom_stub_fn fn; } wifi_mgr_hooks[] = {
        { "_Z16init_WifiManagerv",       stub_void_unregistered },  /* init_WifiManager() */
        { "_Z18wifiManagerProcessv",     stub_void_unregistered },  /* wifiManagerProcess() */
        { "_ZN11WiFiManager7processEv",  stub_void_unregistered },  /* WiFiManager::process() */
        { "_ZN12WiFiSTAClass6statusEv",  stub_ret_wl_connected },   /* WiFi.status() → WL_CONNECTED */
        { NULL, NULL }
    };
    for (int i = 0; wifi_mgr_hooks[i].name; i++) {
        uint32_t addr;
        if (elf_symbols_find(syms, wifi_mgr_hooks[i].name, &addr) == 0) {
            rom_stubs_register(stubs, addr, wifi_mgr_hooks[i].fn,
                               wifi_mgr_hooks[i].name);
            hooked++;
        }
    }

    /* SDMMC / SDSPI host infrastructure stubs.
     * The actual sector I/O (sdmmc_read/write_sectors) and card init are
     * hooked in sdcard_stubs.c.  These cover the SPI bus setup functions
     * that the firmware calls before/after sdmmc_card_init. */
    static const char *sdmmc_noop_fns[] = {
        "sdspi_host_init",
        "sdspi_host_set_card_clk",
        "sdspi_host_remove_device",
        "sdspi_host_get_real_freq",
        "sdspi_host_io_int_enable",
        "sdspi_host_get_dma_info",
        "sdmmc_fix_host_flags",
        "sdmmc_allocate_aligned_buf",
        NULL
    };
    for (int i = 0; sdmmc_noop_fns[i]; i++) {
        uint32_t addr;
        if (elf_symbols_find(syms, sdmmc_noop_fns[i], &addr) == 0) {
            rom_stubs_register(stubs, addr, stub_unregistered, sdmmc_noop_fns[i]);
            hooked++;
        }
    }

    return hooked;
}

int rom_stubs_register(esp32_rom_stubs_t *stubs, uint32_t addr,
                       rom_stub_fn fn, const char *name) {
    return rom_stubs_register_ctx(stubs, addr, fn, name, NULL);
}

int rom_stubs_register_spy(esp32_rom_stubs_t *stubs, uint32_t addr,
                            rom_stub_fn fn, const char *name, void *user_ctx) {
    int rc = rom_stubs_register_ctx(stubs, addr, fn, name, user_ctx);
    if (rc == 0)
        stubs->entries[stubs->count - 1].spy = 1;
    return rc;
}

int rom_stubs_register_ctx(esp32_rom_stubs_t *stubs, uint32_t addr,
                            rom_stub_fn fn, const char *name, void *user_ctx) {
    if (stubs->count >= MAX_ROM_STUBS) return -1;
    stubs->entries[stubs->count].addr = addr;
    stubs->entries[stubs->count].fn = fn;
    stubs->entries[stubs->count].name = name;
    stubs->entries[stubs->count].user_ctx = user_ctx;
    stubs->count++;
    hook_ht_insert(stubs, addr, stubs->count - 1);

    /* Xtensa windowed ABI: ELF symbols sometimes point past the ENTRY
     * instruction (which is 3 bytes, op0 = 0x6, s field = a1).  CALL8/CALL12
     * target the ENTRY itself, so the hook at 'addr' never fires.  Scan the
     * preceding bytes for an ENTRY and register a second hook there.
     * Only do this for firmware addresses (not ROM, which uses ILL placeholders). */
    if (addr >= 0x40080000u && addr < 0x40200000u && stubs->cpu && stubs->cpu->mem) {
        for (int off = 1; off <= 8; off++) {
            uint32_t ea = addr - (uint32_t)off;
            uint8_t b0 = mem_read8(stubs->cpu->mem, ea);
            uint8_t b1 = mem_read8(stubs->cpu->mem, ea + 1);
            /* ENTRY: op0 = 6, s = a1 (bits 3:0 of byte1 = 1) */
            if ((b0 & 0xF) == 0x6 && (b1 & 0xF) == 1) {
                if (stubs->count < MAX_ROM_STUBS) {
                    stubs->entries[stubs->count].addr = ea;
                    stubs->entries[stubs->count].fn = fn;
                    stubs->entries[stubs->count].name = name;
                    stubs->entries[stubs->count].user_ctx = user_ctx;
                    stubs->count++;
                    hook_ht_insert(stubs, ea, stubs->count - 1);
                }
                break;
            }
        }
    }
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

void rom_stubs_set_log_callback(esp32_rom_stubs_t *stubs, rom_log_fn fn, void *ctx) {
    stubs->log_fn = fn;
    stubs->log_ctx = ctx;
}

int rom_stubs_stub_count(const esp32_rom_stubs_t *stubs) {
    return stubs->count;
}

int rom_stubs_get_stats(const esp32_rom_stubs_t *stubs, int index,
                        const char **name_out, uint32_t *addr_out, uint32_t *count_out) {
    if (index < 0 || index >= stubs->count) return -1;
    if (name_out) *name_out = stubs->entries[index].name;
    if (addr_out) *addr_out = stubs->entries[index].addr;
    if (count_out) *count_out = stubs->entries[index].call_count;
    return 0;
}

uint32_t rom_stubs_total_calls(const esp32_rom_stubs_t *stubs) {
    return stubs ? stubs->total_calls : 0;
}

int rom_stubs_unregistered_count(const esp32_rom_stubs_t *stubs) {
    return stubs ? stubs->unregistered_count : 0;
}

const uint64_t *rom_stubs_get_hook_bitmap(const esp32_rom_stubs_t *stubs) {
    return stubs ? stubs->hook_bitmap : NULL;
}

void rom_stubs_set_native_freertos(esp32_rom_stubs_t *stubs, bool native) {
    if (stubs) stubs->native_freertos = native;
}

void rom_stubs_set_periph(esp32_rom_stubs_t *stubs, esp32_periph_t *periph) {
    if (stubs) stubs->periph = periph;
}

void rom_stubs_set_single_core(esp32_rom_stubs_t *stubs, bool single_core) {
    if (stubs) stubs->single_core_mode = single_core;
}

bool rom_stubs_app_cpu_start_requested(const esp32_rom_stubs_t *stubs) {
    return stubs ? stubs->app_cpu_start_requested : false;
}

uint32_t rom_stubs_app_cpu_boot_addr(const esp32_rom_stubs_t *stubs) {
    return stubs ? stubs->app_cpu_boot_addr : 0;
}

void rom_stubs_clear_app_cpu_start(esp32_rom_stubs_t *stubs) {
    if (stubs) stubs->app_cpu_start_requested = false;
}
