#include "rom_stubs.h"
#include "elf_symbols.h"
#include "memory.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#define MAX_ROM_STUBS 512
#define OUTPUT_BUF_SIZE 8192

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
    uint32_t         s_cpu_up_addr;     /* BSS symbol for multicore unblock */
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

/* ===== Boot-sequence ROM stubs ===== */

static void stub_ets_set_appcpu_boot_addr(xtensa_cpu_t *cpu, void *ctx) {
    esp32_rom_stubs_t *s = ctx;
    /* Write s_cpu_up[1] = 1 to unblock multicore startup wait loop */
    if (s->s_cpu_up_addr)
        mem_write8(cpu->mem, s->s_cpu_up_addr + 1, 1);
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

static void stub_strcmp(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    uint32_t s1 = rom_arg(cpu, 0);
    uint32_t s2 = rom_arg(cpu, 1);
    for (;;) {
        uint8_t a = mem_read8(cpu->mem, s1++);
        uint8_t b = mem_read8(cpu->mem, s2++);
        if (a != b || a == 0) {
            rom_return(cpu, (uint32_t)(int32_t)(a - b));
            return;
        }
    }
}

static void stub_strcpy(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    uint32_t dst = rom_arg(cpu, 0);
    uint32_t src = rom_arg(cpu, 1);
    uint32_t i = 0;
    for (;;) {
        uint8_t c = mem_read8(cpu->mem, src + i);
        mem_write8(cpu->mem, dst + i, c);
        if (c == 0) break;
        i++;
    }
    rom_return(cpu, dst);
}

static void stub_strncpy(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    uint32_t dst = rom_arg(cpu, 0);
    uint32_t src = rom_arg(cpu, 1);
    uint32_t n   = rom_arg(cpu, 2);
    uint32_t i = 0;
    int pad = 0;
    for (i = 0; i < n; i++) {
        if (pad) {
            mem_write8(cpu->mem, dst + i, 0);
        } else {
            uint8_t c = mem_read8(cpu->mem, src + i);
            mem_write8(cpu->mem, dst + i, c);
            if (c == 0) pad = 1;
        }
    }
    rom_return(cpu, dst);
}

static void stub_strlcpy(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    uint32_t dst  = rom_arg(cpu, 0);
    uint32_t src  = rom_arg(cpu, 1);
    uint32_t size = rom_arg(cpu, 2);
    /* Count total source length */
    uint32_t slen = 0;
    while (mem_read8(cpu->mem, src + slen) != 0) slen++;
    /* Copy up to size-1 chars */
    if (size > 0) {
        uint32_t copy = (slen < size - 1) ? slen : size - 1;
        for (uint32_t i = 0; i < copy; i++)
            mem_write8(cpu->mem, dst + i, mem_read8(cpu->mem, src + i));
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

/*
 * __nedf2: double not-equal comparison.
 * Returns 0 if a==b, nonzero otherwise. Arguments in a2:a3 and a4:a5.
 */
static void stub_nedf2(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    uint32_t a_lo = rom_arg(cpu, 0);  /* a2 */
    uint32_t a_hi = rom_arg(cpu, 1);  /* a3 */
    uint32_t b_lo = rom_arg(cpu, 2);  /* a4 */
    uint32_t b_hi = rom_arg(cpu, 3);  /* a5 */
    uint64_t a = ((uint64_t)a_hi << 32) | a_lo;
    uint64_t b = ((uint64_t)b_hi << 32) | b_lo;
    double da, db;
    memcpy(&da, &a, 8);
    memcpy(&db, &b, 8);
    rom_return(cpu, (da != db) ? 1 : 0);
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

/* ===== ESP-IDF infrastructure stubs ===== */

/* esp_chip_info(info) — fill esp_chip_info_t struct */
static void stub_esp_chip_info(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    uint32_t info = rom_arg(cpu, 0);
    /* esp_chip_info_t: model(u32), features(u32), cores(u8), revision(u8) */
    mem_write32(cpu->mem, info + 0, 1);       /* CHIP_ESP32 */
    mem_write32(cpu->mem, info + 4, 0x12);    /* WIFI_BGN(1) | BT(2) | BLE(0x10) */
    mem_write8(cpu->mem, info + 8, 2);        /* cores = 2 */
    mem_write8(cpu->mem, info + 9, 3);        /* revision = 3 */
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

/* esp_startup_start_app — skip FreeRTOS scheduler, jump to app_main */
static void stub_esp_startup_start_app(xtensa_cpu_t *cpu, void *ctx) {
    esp32_rom_stubs_t *s = ctx;
    (void)s;
    /* Don't return — we'll let the firmware call app_main naturally
     * through the normal code path. Just return void. */
    rom_return_void(cpu);
}

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

/* ===== Heap stubs ===== */

/* esp_get_free_heap_size() -> 250000 */
void stub_esp_get_free_heap_size(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    rom_return(cpu, 250000);
}

/* esp_get_minimum_free_heap_size() -> 200000 */
void stub_esp_get_minimum_free_heap_size(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    rom_return(cpu, 200000);
}

/* esp_log_timestamp() -> ccount / (cpu_freq_mhz * 1000) */
void stub_esp_log_timestamp(xtensa_cpu_t *cpu, void *ctx) {
    esp32_rom_stubs_t *s = ctx;
    uint32_t ms = cpu->ccount / (s->cpu_freq_mhz * 1000);
    rom_return(cpu, ms);
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

/* ===== PC hook ===== */

static int rom_pc_hook(xtensa_cpu_t *cpu, uint32_t pc, void *ctx) {
    esp32_rom_stubs_t *s = ctx;
    for (int i = 0; i < s->count; i++) {
        if (s->entries[i].addr == pc) {
            s->entries[i].call_count++;
            if (s->log_fn)
                s->log_fn(s->log_ctx, pc, s->entries[i].name, cpu);
            void *ctx = s->entries[i].user_ctx ? s->entries[i].user_ctx : s;
            s->entries[i].fn(cpu, ctx);
            return 1;
        }
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

    /* Install PC hook */
    cpu->pc_hook = rom_pc_hook;
    cpu->pc_hook_ctx = s;

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

    /* Interrupt matrix */
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

    /* Soft-float double comparison */
    rom_stubs_register(s, 0x400636A8, stub_nedf2,               "__nedf2");

    /* Byte-swap and 64-bit shift */
    rom_stubs_register(s, 0x40064AE0, stub_bswapsi2,            "__bswapsi2");
    rom_stubs_register(s, 0x4000C818, stub_ashldi3,             "__ashldi3");

    /* CRC32 */
    rom_stubs_register(s, 0x4005CFEC, stub_crc32_le,            "esp_rom_crc32_le");

    /* Flash/boot helpers */
    rom_stubs_register(s, 0x40062BC8, stub_unregistered,        "spi_flash_clk_cfg");
    rom_stubs_register(s, 0x40008264, stub_software_reset,      "software_reset_cpu");

    /* Misc */
    rom_stubs_register(s, 0x40008208, stub_void_unregistered,   "set_rtc_memory_crc");
    rom_stubs_register(s, 0x4000bfdc, stub_unregistered,        "_xtos_set_intlevel");
    rom_stubs_register(s, 0x400092d0, stub_unregistered,        "uart_rx_one_char");
    rom_stubs_register(s, 0x4000c728, stub_void_unregistered,   "__dummy_lock");
    rom_stubs_register(s, 0x4000c730, stub_unregistered,        "__dummy_lock_try");

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

/* Hook firmware functions by symbol name from ELF.
 * This allows stubbing functions that live in the loaded firmware
 * (not ROM) — needed for things like newlib lock functions. */
int rom_stubs_hook_symbols(esp32_rom_stubs_t *stubs,
                           const elf_symbols_t *syms) {
    if (!stubs || !syms) return 0;
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

    /* Look up s_cpu_up BSS symbol for multicore unblock */
    {
        uint32_t addr;
        if (elf_symbols_find(syms, "s_cpu_up", &addr) == 0)
            stubs->s_cpu_up_addr = addr;
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

    /* Interrupt allocation */
    static const char *intr_fns[] = {
        "esp_intr_alloc",
        "esp_intr_alloc_intrstatus",
        NULL
    };
    for (int i = 0; intr_fns[i]; i++) {
        uint32_t addr;
        if (elf_symbols_find(syms, intr_fns[i], &addr) == 0) {
            rom_stubs_register(stubs, addr, stub_unregistered, intr_fns[i]);
            hooked++;
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

    /* Logging (no-ops — UART logging via ets_printf already works) */
    static const char *log_fns[] = {
        "esp_log_level_set",
        "esp_log_write",
        "esp_log_writev",
        NULL
    };
    for (int i = 0; log_fns[i]; i++) {
        uint32_t addr;
        if (elf_symbols_find(syms, log_fns[i], &addr) == 0) {
            rom_stubs_register(stubs, addr, stub_void_unregistered, log_fns[i]);
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
    }

    /* App startup */
    {
        uint32_t addr;
        if (elf_symbols_find(syms, "esp_startup_start_app", &addr) == 0) {
            rom_stubs_register(stubs, addr, stub_esp_startup_start_app,
                               "esp_startup_start_app");
            hooked++;
        }
    }

    /* VFS stubs */
    static const char *vfs_fns[] = {
        "esp_vfs_register_fd_range",
        "esp_vfs_dev_uart_register",
        "esp_vfs_console_register",
        "esp_vfs_null_register",
        NULL
    };
    for (int i = 0; vfs_fns[i]; i++) {
        uint32_t addr;
        if (elf_symbols_find(syms, vfs_fns[i], &addr) == 0) {
            rom_stubs_register(stubs, addr, stub_unregistered, vfs_fns[i]);
            hooked++;
        }
    }

    /* Misc ESP-IDF init functions (all return 0 / void) */
    static const char *init_ret0_fns[] = {
        "esp_newlib_init",
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
        NULL
    };
    for (int i = 0; init_ret0_fns[i]; i++) {
        uint32_t addr;
        if (elf_symbols_find(syms, init_ret0_fns[i], &addr) == 0) {
            rom_stubs_register(stubs, addr, stub_unregistered, init_ret0_fns[i]);
            hooked++;
        }
    }

    return hooked;
}

int rom_stubs_register(esp32_rom_stubs_t *stubs, uint32_t addr,
                       rom_stub_fn fn, const char *name) {
    return rom_stubs_register_ctx(stubs, addr, fn, name, NULL);
}

int rom_stubs_register_ctx(esp32_rom_stubs_t *stubs, uint32_t addr,
                            rom_stub_fn fn, const char *name, void *user_ctx) {
    if (stubs->count >= MAX_ROM_STUBS) return -1;
    stubs->entries[stubs->count].addr = addr;
    stubs->entries[stubs->count].fn = fn;
    stubs->entries[stubs->count].name = name;
    stubs->entries[stubs->count].user_ctx = user_ctx;
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

int rom_stubs_unregistered_count(const esp32_rom_stubs_t *stubs) {
    return stubs ? stubs->unregistered_count : 0;
}
