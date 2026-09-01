#include "rom_stubs.h"
#include "elf_symbols.h"
#include "memory.h"
#include "peripherals.h"
#include "sandbox_events.h"
#include "guest_call.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <zlib.h>

#define MAX_ROM_STUBS 1024
#define OUTPUT_BUF_SIZE 8192
#define HOOK_HT_SIZE  2048
#define HOOK_HT_MASK  (HOOK_HT_SIZE - 1)

/* Firmware heap hooks use PSRAM for ordinary allocations so stock images have
 * ample room, but ESP32 DMA cannot access external SPI RAM. Reserve the top
 * 48 KiB of data RAM (immediately above the FreeRTOS-stub handle arena) for
 * allocations that explicitly request DMA/internal/exec capabilities. */
#define HEAP_BASE          0x3F800000u
#define HEAP_END           0x3FC00000u
#define INTERNAL_HEAP_BASE 0x3FFF4000u
#define INTERNAL_HEAP_END  0x40000000u
#define HEAP_MAGIC         0x48454150u  /* "HEAP" */
#define HEAP_FREE          0x46524545u  /* "FREE" */
#define HEAP_HDR_SZ        8u

#define MALLOC_CAP_DMA_BIT      (1u << 3)
#define MALLOC_CAP_EXEC_BIT     (1u << 4)
#define MALLOC_CAP_INTERNAL_BIT (1u << 11)

typedef struct {
    uint32_t base;
    uint32_t end;
    uint32_t ptr;
    uint32_t free_list;
} stub_heap_region_t;

/* ROM address range: 0x40000000 - 0x4005FFFF */
#define ROM_BASE 0x40000000u
#define ROM_END  0x40070000u   /* includes SPI flash ROM at 0x4006xxxx */

/* ESP32 UART0 TX FIFO register */
#define UART0_FIFO 0x3FF40000u

typedef struct {
    uint32_t    addr;
    rom_stub_fn fn;
    rom_conditional_stub_fn conditional_fn;
    const char *name;
    uint32_t    call_count;
    void       *user_ctx;   /* Per-entry context; NULL = use rom_stubs */
    int         spy;        /* If true: call fn, then let original execute */
} rom_stub_entry_t;

typedef struct {
    xtensa_cpu_t *cpu;
    uint32_t handler;
    uint32_t arg;
    uint32_t handle;
    bool enabled;
    bool dispatching;
    bool pending;
} stub_irq_t;

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
    rom_firmware_profile_t firmware_profile;  /* exact symbol-less ROM layout */
    esp32_periph_t  *periph;                 /* Peripheral state (for intr_matrix_set) */
    stub_irq_t irq[71];
    /* Per-pin handlers registered through gpio_isr_handler_add(). See
     * gpio_isr_dispatch(). */
    struct { uint32_t fn, arg; } gpio_isr[40];
    bool gpio_isr_installed;
    stub_heap_region_t heap;
    stub_heap_region_t internal_heap;
    struct {
        uint32_t addr;     /* 0 = empty */
        int      idx;      /* index into entries[] */
    } ht[HOOK_HT_SIZE];
    uint64_t hook_bitmap[HOOK_BITMAP_WORDS];
    stub_direct_entry_t *direct;  /* Direct dispatch table (heap-allocated, 64K entries) */

    /* In-memory NVS key/value store (see stub_nvs_* functions) */
    struct nvs_kv_entry {
        int      used;
        char     ns[16];
        char     key[16];
        int      type;       /* 1=i32, 2=u32, 3=blob, 4=str */
        union {
            int32_t  i;
            uint32_t u;
            uint8_t  blob[256];
        } value;
        uint32_t blob_len;
    } nvs_kv[64];
    /* Open-handle table: index 1..N stores namespace string for that handle */
    char     nvs_handle_ns[16][16];   /* up to 16 open handles */
    int      nvs_handle_count;        /* next handle index = count + 1 */
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
        /* 0x40000000 base (not pc & 0xC0000000): every hooked address is in
         * 0x4xxxxxxx, and this also covers the pc==0 null-call hook. */
        cpu->pc = 0x40000000u | (a0 & 0x3FFFFFFFu);
        XT_PS_SET_CALLINC(cpu->ps, 0);
    } else {
        ar_write(cpu, 2, retval);
        /* Mask off the window/call-size bits in a0 exactly like the ci>0 path.
         * A stubbed function reached via a tail `j` (after the caller's `entry`
         * cleared PS.CALLINC) still has a0 = retaddr | (callinc<<30); using it
         * raw yields an invalid PC with the top bits set. All ESP32 code lives
         * in 0x4xxxxxxx, so forcing bit 30 and stripping [31:30] is correct. */
        cpu->pc = 0x40000000u | (ar_read(cpu, 0) & 0x3FFFFFFFu);
    }
}

static void rom_return64(xtensa_cpu_t *cpu, uint64_t retval) {
    int ci = XT_PS_CALLINC(cpu->ps);
    if (ci > 0) {
        ar_write(cpu, ci * 4 + 2, (uint32_t)retval);
        ar_write(cpu, ci * 4 + 3, (uint32_t)(retval >> 32));
        uint32_t a0 = ar_read(cpu, ci * 4);
        cpu->pc = 0x40000000u | (a0 & 0x3FFFFFFFu);
        XT_PS_SET_CALLINC(cpu->ps, 0);
    } else {
        ar_write(cpu, 2, (uint32_t)retval);
        ar_write(cpu, 3, (uint32_t)(retval >> 32));
        cpu->pc = 0x40000000u | (ar_read(cpu, 0) & 0x3FFFFFFFu);
    }
}

static void rom_return_void(xtensa_cpu_t *cpu) {
    int ci = XT_PS_CALLINC(cpu->ps);
    if (ci > 0) {
        uint32_t a0 = ar_read(cpu, ci * 4);
        cpu->pc = 0x40000000u | (a0 & 0x3FFFFFFFu);
        XT_PS_SET_CALLINC(cpu->ps, 0);
    } else {
        cpu->pc = 0x40000000u | (ar_read(cpu, 0) & 0x3FFFFFFFu);
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
    fprintf(stderr, "[APPCPU] boot addr = 0x%08X (from pc=0x%08X)\n", addr, cpu->pc);
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

/* Newlib printf(fmt, ...) — same calling convention as ets_printf.
 * Bypasses newlib's buffered stdio (which never flushes in the emulator
 * because stdout is fully-buffered), writing directly to UART through
 * the mini_printf format engine + UART FIFO hook. */
static void stub_newlib_printf(xtensa_cpu_t *cpu, void *ctx) {
    esp32_rom_stubs_t *s = ctx;
    int before = s->output_len;
    mini_printf(s, cpu);
    int written = s->output_len - before;
    rom_return(cpu, (uint32_t)written);
}

/* puts(const char *s) — write string + newline to UART. */
static void stub_newlib_puts(xtensa_cpu_t *cpu, void *ctx) {
    esp32_rom_stubs_t *s = ctx;
    uint32_t str_addr = rom_arg(cpu, 0);
    int count = 0;
    for (;;) {
        uint8_t c = mem_read8(cpu->mem, str_addr + (uint32_t)count);
        if (c == 0) break;
        output_char(s, (char)c);
        count++;
        if (count > 4096) break;
    }
    output_char(s, '\n');
    rom_return(cpu, (uint32_t)(count + 1));
}

/* putchar(int c) -> c */
static void stub_newlib_putchar(xtensa_cpu_t *cpu, void *ctx) {
    esp32_rom_stubs_t *s = ctx;
    uint32_t c = rom_arg(cpu, 0);
    output_char(s, (char)c);
    rom_return(cpu, c);
}

/* fputs(const char *s, FILE *fp) — ignore fp, route to UART. */
static void stub_newlib_fputs(xtensa_cpu_t *cpu, void *ctx) {
    esp32_rom_stubs_t *s = ctx;
    uint32_t str_addr = rom_arg(cpu, 0);
    int count = 0;
    for (;;) {
        uint8_t c = mem_read8(cpu->mem, str_addr + (uint32_t)count);
        if (c == 0) break;
        output_char(s, (char)c);
        count++;
        if (count > 4096) break;
    }
    rom_return(cpu, (uint32_t)count);
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

static void stub_ets_get_cpu_frequency(xtensa_cpu_t *cpu, void *ctx) {
    esp32_rom_stubs_t *s = ctx;
    rom_return(cpu, s->cpu_freq_mhz);
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

/* __floatundidf: unsigned long long → double */
static void stub_floatundidf(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    uint64_t v = ((uint64_t)rom_arg(cpu, 1) << 32) | rom_arg(cpu, 0);
    rom_return_double(cpu, (double)v);
}

/* __floatdidf: signed long long → double */
static void stub_floatdidf(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    int64_t v = (int64_t)(((uint64_t)rom_arg(cpu, 1) << 32) | rom_arg(cpu, 0));
    rom_return_double(cpu, (double)v);
}

/* __floatundisf: unsigned long long → float */
static void stub_floatundisf(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    uint64_t v = ((uint64_t)rom_arg(cpu, 1) << 32) | rom_arg(cpu, 0);
    float f = (float)v;
    uint32_t bits;
    memcpy(&bits, &f, 4);
    rom_return(cpu, bits);
}

/* __floatdisf: signed long long → float */
static void stub_floatdisf(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    int64_t v = (int64_t)(((uint64_t)rom_arg(cpu, 1) << 32) | rom_arg(cpu, 0));
    float f = (float)v;
    uint32_t bits;
    memcpy(&bits, &f, 4);
    rom_return(cpu, bits);
}

/* __fixdfdi: double → signed long long (truncate toward zero) */
static void stub_fixdfdi(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    rom_return64(cpu, (uint64_t)(int64_t)rom_read_double(cpu, 0));
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
    val = (sh >= 64) ? 0 : (val << sh);
    rom_return64(cpu, val);
}

/* __lshrdi3: 64-bit logical right shift. (a2,a3) >> a4, result in (a2,a3) */
static void stub_lshrdi3(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    uint32_t lo = rom_arg(cpu, 0);
    uint32_t hi = rom_arg(cpu, 1);
    uint32_t sh = rom_arg(cpu, 2);
    uint64_t val = ((uint64_t)hi << 32) | lo;
    val = (sh >= 64) ? 0 : (val >> sh);
    rom_return64(cpu, val);
}

/* __ashrdi3: 64-bit arithmetic right shift. (a2,a3) >> a4, result in (a2,a3) */
static void stub_ashrdi3(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    uint32_t lo = rom_arg(cpu, 0);
    uint32_t hi = rom_arg(cpu, 1);
    uint32_t sh = rom_arg(cpu, 2);
    int64_t val = (int64_t)(((uint64_t)hi << 32) | lo);
    if (sh >= 64) { val = (val < 0) ? -1 : 0; sh = 0; }
    rom_return64(cpu, (uint64_t)(val >> sh));
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

/* ===== esp_lcd panel stubs =====
 * Intercept esp_lcd_new_panel_*() to return fake panel handles tagged
 * with a 4-byte type code, then esp_lcd_panel_draw_bitmap() reads the
 * tag and emits SBX_EV_LCD_PIXELS with the right bit depth + pixel
 * layout for the panel kind.
 *
 * Handle layout: each fake handle is a small struct in DRAM at a
 * fixed address. Byte 0..3 = type tag, bytes 4..7 = width, 8..11 =
 * height. Two flavours so far:
 *   ILI9341  ('L','C','D','1') — 240x320 RGB565 row-major (16bpp)
 *   SSD1306  ('O','L','E','D') — 128x64  monochrome page-col   (1bpp)
 */
#define LCD_FAKE_ILI9341_ADDR 0x3FFFE200u
#define LCD_FAKE_SSD1306_ADDR 0x3FFFE220u

static void lcd_write_panel(xtensa_cpu_t *cpu, uint32_t addr,
                            const char *tag, uint32_t w, uint32_t h) {
    mem_write8(cpu->mem, addr + 0, (uint8_t)tag[0]);
    mem_write8(cpu->mem, addr + 1, (uint8_t)tag[1]);
    mem_write8(cpu->mem, addr + 2, (uint8_t)tag[2]);
    mem_write8(cpu->mem, addr + 3, (uint8_t)tag[3]);
    mem_write32(cpu->mem, addr + 4, w);
    mem_write32(cpu->mem, addr + 8, h);
}

static void stub_esp_lcd_new_panel_ili9341(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    uint32_t ret_handle = rom_arg(cpu, 2);  /* esp_lcd_panel_handle_t *ret_panel */
    lcd_write_panel(cpu, LCD_FAKE_ILI9341_ADDR, "LCD1", 240, 320);
    if (ret_handle)
        mem_write32(cpu->mem, ret_handle, LCD_FAKE_ILI9341_ADDR);
    rom_return(cpu, 0);
}

static void stub_esp_lcd_new_panel_ssd1306(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    /* esp_lcd_new_panel_ssd1306(io, *panel_dev_config, ret_panel)
     * — same shape as ili9341, ret_panel is arg 2. */
    uint32_t ret_handle = rom_arg(cpu, 2);
    lcd_write_panel(cpu, LCD_FAKE_SSD1306_ADDR, "OLED", 128, 64);
    if (ret_handle)
        mem_write32(cpu->mem, ret_handle, LCD_FAKE_SSD1306_ADDR);
    rom_return(cpu, 0);
}

static void stub_esp_lcd_panel_noop_ok(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    rom_return(cpu, 0);  /* ESP_OK */
}

/* esp_lcd_panel_draw_bitmap(panel, x_start, y_start, x_end, y_end, color_data)
 *
 * Reads the panel tag at the handle address to figure out the panel
 * type, then emits a SBX_EV_LCD_PIXELS event with the right bit-depth
 * and pixel layout. The display node on the litegraph side decodes the
 * payload according to its own renderer (RGB565 row-major for ILI9341,
 * column-major-page 1bpp for SSD1306). */
static void stub_esp_lcd_panel_draw_bitmap(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    uint32_t panel  = rom_arg(cpu, 0);
    uint32_t x0     = rom_arg(cpu, 1);
    uint32_t y0     = rom_arg(cpu, 2);
    uint32_t x1     = rom_arg(cpu, 3);
    uint32_t y1     = rom_arg(cpu, 4);
    uint32_t data_p = rom_arg(cpu, 5);

    if (x1 <= x0 || y1 <= y0 || !panel) { rom_return(cpu, 0); return; }

    uint8_t tag[4] = {
        mem_read8(cpu->mem, panel + 0),
        mem_read8(cpu->mem, panel + 1),
        mem_read8(cpu->mem, panel + 2),
        mem_read8(cpu->mem, panel + 3),
    };
    int is_ssd1306 = (tag[0] == 'O' && tag[1] == 'L' && tag[2] == 'E' && tag[3] == 'D');
    int is_ili9341 = (tag[0] == 'L' && tag[1] == 'C' && tag[2] == 'D' && tag[3] == '1');
    if (!is_ssd1306 && !is_ili9341) { rom_return(cpu, 0); return; }

    static uint8_t scratch[256 * 256 * 2];
    uint32_t w = x1 - x0;
    uint32_t h = y1 - y0;
    uint32_t bytes;
    uint32_t bpp;
    if (is_ssd1306) {
        /* SSD1306 page-col format: each byte = 8 vertical pixels at one
         * column. Buffer size = w * (h/8) bytes; round up h to a page. */
        uint32_t pages = (h + 7) / 8;
        bytes = w * pages;
        bpp = 1;
    } else {
        /* RGB565 row-major */
        bytes = w * h * 2;
        bpp = 16;
    }
    if (bytes > sizeof(scratch)) bytes = sizeof(scratch);
    for (uint32_t i = 0; i < bytes; i++)
        scratch[i] = mem_read8(cpu->mem, data_p + i);

    sbx_event_t ev = { .kind = SBX_EV_LCD_PIXELS, .cycle = 0 };
    ev.lcd_pixels.x = x0;
    ev.lcd_pixels.y = y0;
    ev.lcd_pixels.w = w;
    ev.lcd_pixels.h = h;
    ev.lcd_pixels.bpp = (uint16_t)bpp;
    ev.lcd_pixels.pixels = scratch;
    sbx_events_emit(&ev);

    rom_return(cpu, 0);
}

/* esp_flash_get_size(esp_flash_t *chip, uint32_t *out_size) -> ESP_OK, reports 4 MiB.
 * The underlying SPI flash chip isn't modeled, so probing via the real function
 * returns ESP_ERR_NOT_SUPPORTED. Report a conventional 4 MiB image instead. */
static void stub_esp_flash_get_size(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    uint32_t out_ptr = rom_arg(cpu, 1);
    if (out_ptr)
        mem_write32(cpu->mem, out_ptr, 4u * 1024u * 1024u);
    rom_return(cpu, 0);  /* ESP_OK */
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

/* ----- In-memory KV store helpers -----
 *
 * The NVS implementation provides a tiny key/value table that lives entirely
 * in the host. ESP-IDF apps that round-trip through nvs_open / nvs_set_* /
 * nvs_get_* see real values across calls within a single emulator run.
 * Restart wipes the store; we do not persist to disk. */

static void nvs_copy_guest_string(xtensa_cpu_t *cpu, uint32_t guest_addr,
                                  char *dst, size_t cap) {
    if (!guest_addr || cap == 0) {
        if (cap) dst[0] = '\0';
        return;
    }
    size_t i = 0;
    for (; i < cap - 1; i++) {
        uint8_t c = mem_read8(cpu->mem, guest_addr + (uint32_t)i);
        if (!c) break;
        dst[i] = (char)c;
    }
    dst[i] = '\0';
}

static struct nvs_kv_entry *nvs_find(esp32_rom_stubs_t *s, const char *ns,
                                     const char *key) {
    for (int i = 0; i < 64; i++) {
        if (s->nvs_kv[i].used &&
            strcmp(s->nvs_kv[i].ns, ns) == 0 &&
            strcmp(s->nvs_kv[i].key, key) == 0) {
            return &s->nvs_kv[i];
        }
    }
    return NULL;
}

static struct nvs_kv_entry *nvs_alloc(esp32_rom_stubs_t *s, const char *ns,
                                      const char *key) {
    struct nvs_kv_entry *e = nvs_find(s, ns, key);
    if (e) return e;
    for (int i = 0; i < 64; i++) {
        if (!s->nvs_kv[i].used) {
            s->nvs_kv[i].used = 1;
            strncpy(s->nvs_kv[i].ns,  ns,  sizeof(s->nvs_kv[i].ns)  - 1);
            s->nvs_kv[i].ns[sizeof(s->nvs_kv[i].ns) - 1] = '\0';
            strncpy(s->nvs_kv[i].key, key, sizeof(s->nvs_kv[i].key) - 1);
            s->nvs_kv[i].key[sizeof(s->nvs_kv[i].key) - 1] = '\0';
            return &s->nvs_kv[i];
        }
    }
    return NULL;
}

/* Look up the namespace string associated with an open handle (1..N). */
static const char *nvs_handle_ns(esp32_rom_stubs_t *s, uint32_t handle) {
    if (handle == 0 || handle > 16) return "";
    return s->nvs_handle_ns[handle - 1];
}

/* nvs_flash_init / nvs_flash_init_partition -> ESP_OK */
void stub_nvs_flash_init(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    rom_return(cpu, ESP_OK);
}

/* nvs_flash_erase -> ESP_OK (also clears the in-memory KV store) */
void stub_nvs_flash_erase(xtensa_cpu_t *cpu, void *ctx) {
    esp32_rom_stubs_t *s = ctx;
    if (s) {
        memset(s->nvs_kv, 0, sizeof(s->nvs_kv));
    }
    rom_return(cpu, ESP_OK);
}

/* Allocate a new handle that remembers `ns_addr`'s namespace string.
 *
 * For the C path we just return a small integer (1..16) — callers treat the
 * handle as opaque and our nvs_get/set stubs translate it via nvs_handle_ns.
 *
 * The C++ path also needs a "this" pointer that survives a vtable dispatch
 * (NVSHandle::commit is a virtual call: `[this+0][40] -> callx8`). For that
 * we additionally write a fake object into DRAM whose first slot points at
 * the real NVSHandleSimple vtable in flash. The fake-object pointer encodes
 * the handle index in its low bits so per-call stubs can recover the
 * namespace via nvs_object_to_handle below. */
#define NVS_FAKE_OBJ_BASE   0x3FFFE000u   /* DRAM scratch area */
#define NVS_FAKE_OBJ_STRIDE 16u
#define NVS_FAKE_VTABLE     0x3FFFE100u   /* DRAM scratch: fake vtable */
#define NVS_FAKE_VTABLE_SLOTS 32
/* Address of the real NVSHandleSimple::commit symbol in flash; we hook this
 * PC to short-circuit any virtual call that lands on it. By filling our
 * fake vtable with this address in every slot, *any* virtual dispatch from
 * a fake handle ends up returning ESP_OK without touching real C++ code. */
#define NVS_COMMIT_TRAMPOLINE 0x400D8E14u

static uint32_t nvs_handle_to_object(uint32_t handle) {
    return NVS_FAKE_OBJ_BASE + (handle - 1) * NVS_FAKE_OBJ_STRIDE;
}

static uint32_t nvs_object_to_handle(uint32_t obj) {
    if (obj < NVS_FAKE_OBJ_BASE) return obj; /* already a small int */
    uint32_t idx = (obj - NVS_FAKE_OBJ_BASE) / NVS_FAKE_OBJ_STRIDE;
    if (idx >= 16) return 0;
    return idx + 1;
}

static uint32_t nvs_open_alloc_handle(esp32_rom_stubs_t *s, xtensa_cpu_t *cpu,
                                      uint32_t ns_addr) {
    int idx;
    if (s->nvs_handle_count >= 16) {
        /* Reuse slot 0 as a sane fallback. */
        idx = 0;
    } else {
        idx = s->nvs_handle_count++;
    }
    nvs_copy_guest_string(cpu, ns_addr, s->nvs_handle_ns[idx],
                          sizeof(s->nvs_handle_ns[idx]));
    /* Lazily build a fake vtable: every slot points at the hooked
     * NVSHandleSimple::commit trampoline so any virtual dispatch through
     * a fake C++ NVSHandle lands on a stub that returns ESP_OK. */
    if (mem_read32(cpu->mem, NVS_FAKE_VTABLE) != NVS_COMMIT_TRAMPOLINE) {
        for (int i = 0; i < NVS_FAKE_VTABLE_SLOTS; i++) {
            mem_write32(cpu->mem, NVS_FAKE_VTABLE + (uint32_t)i * 4,
                        NVS_COMMIT_TRAMPOLINE);
        }
    }
    /* Stamp the fake C++ object with our fake vtable pointer. */
    uint32_t obj = NVS_FAKE_OBJ_BASE + (uint32_t)idx * NVS_FAKE_OBJ_STRIDE;
    mem_write32(cpu->mem, obj + 0, NVS_FAKE_VTABLE);
    return (uint32_t)(idx + 1);
}

/* nvs_open(name, mode, *handle_out) -> ESP_OK */
void stub_nvs_open(xtensa_cpu_t *cpu, void *ctx) {
    esp32_rom_stubs_t *s = ctx;
    uint32_t name_addr  = rom_arg(cpu, 0);
    uint32_t handle_out = rom_arg(cpu, 2);
    uint32_t handle = s ? nvs_open_alloc_handle(s, cpu, name_addr) : 1;
    if (handle_out)
        mem_write32(cpu->mem, handle_out, handle);
    rom_return(cpu, ESP_OK);
}

/* nvs_open_from_partition(part, name, mode, *handle_out) -> ESP_OK */
void stub_nvs_open_from_partition(xtensa_cpu_t *cpu, void *ctx) {
    esp32_rom_stubs_t *s = ctx;
    uint32_t name_addr  = rom_arg(cpu, 1);
    uint32_t handle_out = rom_arg(cpu, 3);
    uint32_t handle = s ? nvs_open_alloc_handle(s, cpu, name_addr) : 1;
    if (handle_out)
        mem_write32(cpu->mem, handle_out, handle);
    rom_return(cpu, ESP_OK);
}

/* nvs_close(handle) -> void */
void stub_nvs_close(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    rom_return_void(cpu);
}

/* nvs_get_* fallback -> ESP_ERR_NVS_NOT_FOUND (kept for legacy callers/tests) */
void stub_nvs_get_notfound(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    rom_return(cpu, ESP_ERR_NVS_NOT_FOUND);
}

/* nvs_set_* / nvs_commit fallback -> ESP_OK (kept for legacy callers/tests) */
void stub_nvs_set_ok(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    rom_return(cpu, ESP_OK);
}

/* nvs_get_i32(handle, key, *out) */
void stub_nvs_get_i32(xtensa_cpu_t *cpu, void *ctx) {
    esp32_rom_stubs_t *s = ctx;
    uint32_t handle  = rom_arg(cpu, 0);
    uint32_t key_a   = rom_arg(cpu, 1);
    uint32_t out_a   = rom_arg(cpu, 2);
    char key[16];
    nvs_copy_guest_string(cpu, key_a, key, sizeof(key));
    struct nvs_kv_entry *e = s ? nvs_find(s, nvs_handle_ns(s, handle), key) : NULL;
    if (!e) { rom_return(cpu, ESP_ERR_NVS_NOT_FOUND); return; }
    if (out_a) mem_write32(cpu->mem, out_a, (uint32_t)e->value.i);
    rom_return(cpu, ESP_OK);
}

/* nvs_get_u32(handle, key, *out) */
void stub_nvs_get_u32(xtensa_cpu_t *cpu, void *ctx) {
    esp32_rom_stubs_t *s = ctx;
    uint32_t handle  = rom_arg(cpu, 0);
    uint32_t key_a   = rom_arg(cpu, 1);
    uint32_t out_a   = rom_arg(cpu, 2);
    char key[16];
    nvs_copy_guest_string(cpu, key_a, key, sizeof(key));
    struct nvs_kv_entry *e = s ? nvs_find(s, nvs_handle_ns(s, handle), key) : NULL;
    if (!e) { rom_return(cpu, ESP_ERR_NVS_NOT_FOUND); return; }
    if (out_a) mem_write32(cpu->mem, out_a, e->value.u);
    rom_return(cpu, ESP_OK);
}

/* nvs_set_i32(handle, key, value) */
void stub_nvs_set_i32(xtensa_cpu_t *cpu, void *ctx) {
    esp32_rom_stubs_t *s = ctx;
    uint32_t handle  = rom_arg(cpu, 0);
    uint32_t key_a   = rom_arg(cpu, 1);
    uint32_t val     = rom_arg(cpu, 2);
    char key[16];
    nvs_copy_guest_string(cpu, key_a, key, sizeof(key));
    if (!s) { rom_return(cpu, ESP_OK); return; }
    struct nvs_kv_entry *e = nvs_alloc(s, nvs_handle_ns(s, handle), key);
    if (!e) { rom_return(cpu, ESP_OK); return; }
    e->type = 1;
    e->value.i = (int32_t)val;
    rom_return(cpu, ESP_OK);
}

/* nvs_set_u32(handle, key, value) */
void stub_nvs_set_u32(xtensa_cpu_t *cpu, void *ctx) {
    esp32_rom_stubs_t *s = ctx;
    uint32_t handle  = rom_arg(cpu, 0);
    uint32_t key_a   = rom_arg(cpu, 1);
    uint32_t val     = rom_arg(cpu, 2);
    char key[16];
    nvs_copy_guest_string(cpu, key_a, key, sizeof(key));
    if (!s) { rom_return(cpu, ESP_OK); return; }
    struct nvs_kv_entry *e = nvs_alloc(s, nvs_handle_ns(s, handle), key);
    if (!e) { rom_return(cpu, ESP_OK); return; }
    e->type = 2;
    e->value.u = val;
    rom_return(cpu, ESP_OK);
}

/* nvs_set_blob(handle, key, blob, length) */
void stub_nvs_set_blob(xtensa_cpu_t *cpu, void *ctx) {
    esp32_rom_stubs_t *s = ctx;
    uint32_t handle  = rom_arg(cpu, 0);
    uint32_t key_a   = rom_arg(cpu, 1);
    uint32_t blob_a  = rom_arg(cpu, 2);
    uint32_t len     = rom_arg(cpu, 3);
    char key[16];
    nvs_copy_guest_string(cpu, key_a, key, sizeof(key));
    if (!s) { rom_return(cpu, ESP_OK); return; }
    struct nvs_kv_entry *e = nvs_alloc(s, nvs_handle_ns(s, handle), key);
    if (!e) { rom_return(cpu, ESP_OK); return; }
    e->type = 3;
    if (len > sizeof(e->value.blob)) len = sizeof(e->value.blob);
    for (uint32_t i = 0; i < len; i++)
        e->value.blob[i] = mem_read8(cpu->mem, blob_a + i);
    e->blob_len = len;
    rom_return(cpu, ESP_OK);
}

/* nvs_set_str(handle, key, str) */
void stub_nvs_set_str(xtensa_cpu_t *cpu, void *ctx) {
    esp32_rom_stubs_t *s = ctx;
    uint32_t handle  = rom_arg(cpu, 0);
    uint32_t key_a   = rom_arg(cpu, 1);
    uint32_t str_a   = rom_arg(cpu, 2);
    char key[16];
    nvs_copy_guest_string(cpu, key_a, key, sizeof(key));
    if (!s) { rom_return(cpu, ESP_OK); return; }
    struct nvs_kv_entry *e = nvs_alloc(s, nvs_handle_ns(s, handle), key);
    if (!e) { rom_return(cpu, ESP_OK); return; }
    e->type = 4;
    uint32_t i = 0;
    for (; i < sizeof(e->value.blob) - 1; i++) {
        uint8_t c = mem_read8(cpu->mem, str_a + i);
        e->value.blob[i] = c;
        if (!c) break;
    }
    e->value.blob[sizeof(e->value.blob) - 1] = 0;
    e->blob_len = i;
    rom_return(cpu, ESP_OK);
}

/* nvs_get_blob(handle, key, *out_buf, *inout_len) */
void stub_nvs_get_blob(xtensa_cpu_t *cpu, void *ctx) {
    esp32_rom_stubs_t *s = ctx;
    uint32_t handle  = rom_arg(cpu, 0);
    uint32_t key_a   = rom_arg(cpu, 1);
    uint32_t out_a   = rom_arg(cpu, 2);
    uint32_t lenp_a  = rom_arg(cpu, 3);
    char key[16];
    nvs_copy_guest_string(cpu, key_a, key, sizeof(key));
    struct nvs_kv_entry *e = s ? nvs_find(s, nvs_handle_ns(s, handle), key) : NULL;
    if (!e) { rom_return(cpu, ESP_ERR_NVS_NOT_FOUND); return; }
    if (lenp_a) mem_write32(cpu->mem, lenp_a, e->blob_len);
    if (out_a) {
        for (uint32_t i = 0; i < e->blob_len; i++)
            mem_write8(cpu->mem, out_a + i, e->value.blob[i]);
    }
    rom_return(cpu, ESP_OK);
}

/* nvs_get_str(handle, key, *out_buf, *inout_len) */
void stub_nvs_get_str(xtensa_cpu_t *cpu, void *ctx) {
    esp32_rom_stubs_t *s = ctx;
    uint32_t handle  = rom_arg(cpu, 0);
    uint32_t key_a   = rom_arg(cpu, 1);
    uint32_t out_a   = rom_arg(cpu, 2);
    uint32_t lenp_a  = rom_arg(cpu, 3);
    char key[16];
    nvs_copy_guest_string(cpu, key_a, key, sizeof(key));
    struct nvs_kv_entry *e = s ? nvs_find(s, nvs_handle_ns(s, handle), key) : NULL;
    if (!e) { rom_return(cpu, ESP_ERR_NVS_NOT_FOUND); return; }
    if (lenp_a) mem_write32(cpu->mem, lenp_a, e->blob_len + 1);
    if (out_a) {
        for (uint32_t i = 0; i < e->blob_len; i++)
            mem_write8(cpu->mem, out_a + i, e->value.blob[i]);
        mem_write8(cpu->mem, out_a + e->blob_len, 0);
    }
    rom_return(cpu, ESP_OK);
}

/* ----- C++ wrapper stubs (nvs::open_nvs_handle and NVSHandle::{get,set}_item) -----
 *
 * The nvs_rw_value_cxx example never goes through the C nvs_open path; it
 * calls the C++ entry points directly. We mirror them here so the namespace
 * and value tracking still funnels into the same KV store. */

/* nvs::open_nvs_handle(unique_ptr_sret*, name, mode, esp_err_t*) */
void stub_cxx_open_nvs_handle(xtensa_cpu_t *cpu, void *ctx) {
    esp32_rom_stubs_t *s = ctx;
    uint32_t sret_a   = rom_arg(cpu, 0);
    uint32_t name_a   = rom_arg(cpu, 1);
    /* arg2 = mode (ignored) */
    uint32_t err_a    = rom_arg(cpu, 3);
    uint32_t handle = s ? nvs_open_alloc_handle(s, cpu, name_a) : 1;
    uint32_t obj    = nvs_handle_to_object(handle);
    /* unique_ptr layout: { NVSHandle* ptr; } at offset 0 */
    if (sret_a) mem_write32(cpu->mem, sret_a, obj);
    if (err_a)  mem_write32(cpu->mem, err_a, ESP_OK);
    /* C++ ABI: sret functions return the sret pointer in a2 */
    rom_return(cpu, sret_a);
}

/* nvs::open_nvs_handle_from_partition(sret*, part, name, mode, err*) */
void stub_cxx_open_nvs_handle_from_partition(xtensa_cpu_t *cpu, void *ctx) {
    esp32_rom_stubs_t *s = ctx;
    uint32_t sret_a   = rom_arg(cpu, 0);
    uint32_t name_a   = rom_arg(cpu, 2);
    uint32_t err_a    = rom_arg(cpu, 4);
    uint32_t handle = s ? nvs_open_alloc_handle(s, cpu, name_a) : 1;
    uint32_t obj    = nvs_handle_to_object(handle);
    if (sret_a) mem_write32(cpu->mem, sret_a, obj);
    if (err_a)  mem_write32(cpu->mem, err_a, ESP_OK);
    rom_return(cpu, sret_a);
}

/* NVSHandle::get_item<long/int32>(this, key, T& value) */
void stub_cxx_nvshandle_get_item_i32(xtensa_cpu_t *cpu, void *ctx) {
    esp32_rom_stubs_t *s = ctx;
    uint32_t this_a = rom_arg(cpu, 0);
    uint32_t key_a  = rom_arg(cpu, 1);
    uint32_t ref_a  = rom_arg(cpu, 2);
    uint32_t handle = nvs_object_to_handle(this_a);
    char key[16];
    nvs_copy_guest_string(cpu, key_a, key, sizeof(key));
    struct nvs_kv_entry *e = s ? nvs_find(s, nvs_handle_ns(s, handle), key) : NULL;
    if (!e) { rom_return(cpu, ESP_ERR_NVS_NOT_FOUND); return; }
    if (ref_a) mem_write32(cpu->mem, ref_a, (uint32_t)e->value.i);
    rom_return(cpu, ESP_OK);
}

/* NVSHandle::set_item<long/int32>(this, key, T value) */
void stub_cxx_nvshandle_set_item_i32(xtensa_cpu_t *cpu, void *ctx) {
    esp32_rom_stubs_t *s = ctx;
    uint32_t this_a = rom_arg(cpu, 0);
    uint32_t key_a  = rom_arg(cpu, 1);
    uint32_t val    = rom_arg(cpu, 2);
    uint32_t handle = nvs_object_to_handle(this_a);
    char key[16];
    nvs_copy_guest_string(cpu, key_a, key, sizeof(key));
    if (!s) { rom_return(cpu, ESP_OK); return; }
    struct nvs_kv_entry *e = nvs_alloc(s, nvs_handle_ns(s, handle), key);
    if (!e) { rom_return(cpu, ESP_OK); return; }
    e->type = 1;
    e->value.i = (int32_t)val;
    rom_return(cpu, ESP_OK);
}

/* NVSHandleSimple::commit() — reached via vtable from NVSHandle::commit() */
void stub_cxx_nvshandle_commit(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    rom_return(cpu, ESP_OK);
}

/* ===== GPIO driver stubs ===== */

static void gpio_set_direction_registers(xtensa_cpu_t *cpu, uint32_t pin,
                                         bool output) {
    if (pin < 32u) {
        uint32_t bit = 1u << pin;
        mem_write32(cpu->mem, output ? 0x3FF44024u : 0x3FF44028u, bit);
    } else if (pin < 40u) {
        uint32_t bit = 1u << (pin - 32u);
        mem_write32(cpu->mem, output ? 0x3FF44030u : 0x3FF44034u, bit);
    }
}

static void gpio_set_output_level(xtensa_cpu_t *cpu, uint32_t pin,
                                  uint32_t level) {
    if (pin < 32u) {
        uint32_t bit = 1u << pin;
        mem_write32(cpu->mem, level ? 0x3FF44008u : 0x3FF4400Cu, bit);
    } else if (pin < 40u) {
        uint32_t bit = 1u << (pin - 32u);
        mem_write32(cpu->mem, level ? 0x3FF44014u : 0x3FF44018u, bit);
    }
}

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
    /* ESP-IDF GPIO modes are bit flags: bit 1 enables the output driver. */
    gpio_set_direction_registers(cpu, pin, (mode & 2u) != 0);
    rom_return(cpu, ESP_OK);
}

/* gpio_set_level(pin, level) -> ESP_OK */
void stub_gpio_set_level(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    uint32_t pin   = rom_arg(cpu, 0);
    uint32_t level = rom_arg(cpu, 1);
    gpio_set_output_level(cpu, pin, level);
    rom_return(cpu, ESP_OK);
}

/* Arduino-ESP32 wrappers are frequently placed in flash and reached without
 * going through the separately hooked ESP-IDF driver symbols. Preserve their
 * void ABI while driving the same GPIO register model. */
void stub_arduino_pin_mode(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    uint32_t pin = rom_arg(cpu, 0);
    uint32_t mode = rom_arg(cpu, 1);
    /* Arduino OUTPUT is 0x03 and OUTPUT_OPEN_DRAIN is 0x12. Both carry the
     * same output-enable bit used by the ESP-IDF gpio_mode_t values. */
    gpio_set_direction_registers(cpu, pin, (mode & 2u) != 0);
    rom_return_void(cpu);
}

void stub_arduino_digital_write(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    gpio_set_output_level(cpu, rom_arg(cpu, 0), rom_arg(cpu, 1));
    rom_return_void(cpu);
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

/* adc1_get_raw(channel) -> int: read programmable ADC value from periph */
static void stub_adc1_get_raw(xtensa_cpu_t *cpu, void *ctx) {
    esp32_rom_stubs_t *s = ctx;
    uint32_t ch = rom_arg(cpu, 0);
    uint16_t raw = s->periph ? periph_get_adc_value(s->periph, (int)ch) : 0;
    /* adc1_config_width() programs SENS_SAR1_BIT_WIDTH as 0=9 ... 3=12. */
    uint32_t width = mem_read32(cpu->mem, 0x3FF4882Cu) & 3u;
    raw &= (uint16_t)((1u << (9u + width)) - 1u);
    rom_return(cpu, raw);
}

/* adc2_get_raw(channel, width, int *out_raw) -> ESP_OK. ADC2 occupies the
 * second ten-entry bank in the host-injected channel table. */
static void stub_adc2_get_raw(xtensa_cpu_t *cpu, void *ctx) {
    esp32_rom_stubs_t *s = ctx;
    uint32_t ch = rom_arg(cpu, 0);
    uint32_t width = rom_arg(cpu, 1);
    uint32_t out_ptr = rom_arg(cpu, 2);
    uint16_t raw = 0;
    if (s->periph && ch < 10u)
        raw = periph_get_adc_value(s->periph, 10 + (int)ch);
    if (width > 3u) width = 3u;
    raw &= (uint16_t)((1u << (9u + width)) - 1u);
    if (out_ptr)
        mem_write32(cpu->mem, out_ptr, raw);
    rom_return(cpu, ESP_OK);
}

/* gpio_reset_pin(pin) -> ESP_OK */
void stub_gpio_reset_pin(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    rom_return(cpu, ESP_OK);
}

/* esp_ipc_call_blocking(cpu_id, func, arg)
 *
 * IDF runs a per-core IPC task and hands it the callback; drivers use it to
 * allocate an interrupt on a particular core. gpio_isr_register() goes
 * through here, so with the IPC tasks not running the callback was simply
 * dropped and GPIO interrupt allocation never happened -- attachInterrupt()
 * silently did nothing.
 *
 * Which core a handler is installed on does not matter to the emulator, so
 * run the callback synchronously on the caller instead of modelling the task
 * handshake. That is what "blocking" means to the caller anyway. */
static void stub_esp_ipc_call_blocking(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    uint32_t func = rom_arg(cpu, 1);
    uint32_t arg  = rom_arg(cpu, 2);
    if (func)
        (void)guest_call8(cpu, func, &arg, 1, 2000000u, NULL);
    rom_return(cpu, 0); /* ESP_OK */
}


/* ===== GPIO interrupt service =====
 *
 * ESP-IDF installs one shared handler for the GPIO source and dispatches
 * per-pin callbacks from it, reading GPIO_STATUS to find which pins fired and
 * clearing them afterwards. Its own installer routes the allocation through
 * an inter-processor call to a specific core, which is not something this
 * emulator models, so the registration was dropped and attachInterrupt() had
 * no effect at all -- silently, since the driver reports success.
 *
 * Model the service instead of the plumbing: record the per-pin handlers and
 * run the same dispatch loop when the GPIO source asserts. Edge and level
 * detection, GPIO_STATUS and the per-core INT_ENA routing are all already in
 * the peripheral model, so only this dispatch was missing. */
#define GPIO_INTR_SOURCE_NUM 22
#define GPIO_BASE_ADDR       0x3FF44000u
#define GPIO_STATUS_REG_ADDR (GPIO_BASE_ADDR + 0x044u)
#define GPIO_STATUS_W1TC_ADDR (GPIO_BASE_ADDR + 0x04Cu)
#define GPIO_STATUS1_REG_ADDR (GPIO_BASE_ADDR + 0x050u)
#define GPIO_STATUS1_W1TC_ADDR (GPIO_BASE_ADDR + 0x058u)

static void gpio_isr_dispatch(void *ctx, int source) {
    esp32_rom_stubs_t *s = ctx;
    (void)source;
    if (!s || !s->cpu) return;

    uint32_t st0 = mem_read32(s->cpu->mem, GPIO_STATUS_REG_ADDR);
    uint32_t st1 = mem_read32(s->cpu->mem, GPIO_STATUS1_REG_ADDR);
    uint32_t handled0 = 0, handled1 = 0;

    for (int pin = 0; pin < 40; pin++) {
        uint32_t bit = pin < 32 ? (1u << pin) : (1u << (pin - 32));
        uint32_t st = pin < 32 ? st0 : st1;
        if (!(st & bit)) continue;
        if (pin < 32) handled0 |= bit; else handled1 |= bit;
        if (!s->gpio_isr[pin].fn) continue;
        uint32_t args[] = { s->gpio_isr[pin].arg };
        int r = guest_call8(s->cpu, s->gpio_isr[pin].fn, args, 1,
                            2000000u, NULL);
        if (r != 0)
            fprintf(stderr, "[gpio] pin %d handler 0x%08X failed (%d)\n",
                    pin, s->gpio_isr[pin].fn, r);
    }

    /* Acknowledge every pin whose status was latched, handler or not:
     * leaving a bit set holds the source asserted and the guest would be
     * re-entered forever. */
    if (handled0)
        mem_write32(s->cpu->mem, GPIO_STATUS_W1TC_ADDR, handled0);
    if (handled1)
        mem_write32(s->cpu->mem, GPIO_STATUS1_W1TC_ADDR, handled1);
}

/* GPIO_PINn_REG: INT_TYPE is bits 9:7, INT_ENA bits 17:13. The driver
 * normally writes these itself, but gpio_intr_enable() picks the target core
 * from the ISR handle that its own installer would have produced -- which
 * this one does not, since the service is modelled rather than installed. Set
 * the registers here so the peripheral's existing edge detection and per-core
 * routing see what the caller asked for. */
#define GPIO_PIN0_REG_ADDR   (GPIO_BASE_ADDR + 0x088u)
#define GPIO_PIN_INT_TYPE_SHIFT 7
#define GPIO_PIN_INT_ENA_SHIFT  13
#define GPIO_PIN_PRO_CPU_INTR   (1u << 2)

static void gpio_pin_reg_update(xtensa_cpu_t *cpu, uint32_t pin,
                                uint32_t mask, uint32_t value) {
    if (pin >= 40u) return;
    uint32_t addr = GPIO_PIN0_REG_ADDR + pin * 4u;
    uint32_t cfg = mem_read32(cpu->mem, addr);
    mem_write32(cpu->mem, addr, (cfg & ~mask) | (value & mask));
}

/* gpio_install_isr_service(flags) -> ESP_OK */
static void stub_gpio_install_isr_service(xtensa_cpu_t *cpu, void *ctx) {
    esp32_rom_stubs_t *s = ctx;
    if (s && !s->gpio_isr_installed) {
        s->gpio_isr_installed = true;
        if (s->periph)
            periph_set_irq_dispatch(s->periph, GPIO_INTR_SOURCE_NUM,
                                    gpio_isr_dispatch, s);
    }
    rom_return(cpu, ESP_OK);
}

/* gpio_isr_handler_add(gpio_num, isr_handler, args) -> ESP_OK
 *
 * The driver enables the pin's interrupt here rather than making the caller
 * do it -- attachInterrupt() never calls gpio_intr_enable() -- so enable it
 * here too, or the peripheral latches status that is routed to no core. */
static void stub_gpio_isr_handler_add(xtensa_cpu_t *cpu, void *ctx) {
    esp32_rom_stubs_t *s = ctx;
    uint32_t pin = rom_arg(cpu, 0);
    if (s && pin < 40u) {
        s->gpio_isr[pin].fn = rom_arg(cpu, 1);
        s->gpio_isr[pin].arg = rom_arg(cpu, 2);
        gpio_pin_reg_update(cpu, pin, 0x1Fu << GPIO_PIN_INT_ENA_SHIFT,
                            GPIO_PIN_PRO_CPU_INTR << GPIO_PIN_INT_ENA_SHIFT);
    }
    rom_return(cpu, ESP_OK);
}

/* gpio_set_intr_type(gpio_num, intr_type) -> ESP_OK */
static void stub_gpio_set_intr_type(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    uint32_t pin = rom_arg(cpu, 0), type = rom_arg(cpu, 1) & 0x7u;
    gpio_pin_reg_update(cpu, pin, 0x7u << GPIO_PIN_INT_TYPE_SHIFT,
                        type << GPIO_PIN_INT_TYPE_SHIFT);
    rom_return(cpu, ESP_OK);
}

/* gpio_intr_enable(gpio_num) -> ESP_OK */
static void stub_gpio_intr_enable(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    gpio_pin_reg_update(cpu, rom_arg(cpu, 0),
                        0x1Fu << GPIO_PIN_INT_ENA_SHIFT,
                        GPIO_PIN_PRO_CPU_INTR << GPIO_PIN_INT_ENA_SHIFT);
    rom_return(cpu, ESP_OK);
}

/* gpio_intr_disable(gpio_num) -> ESP_OK */
static void stub_gpio_intr_disable(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    gpio_pin_reg_update(cpu, rom_arg(cpu, 0),
                        0x1Fu << GPIO_PIN_INT_ENA_SHIFT, 0u);
    rom_return(cpu, ESP_OK);
}

/* gpio_isr_handler_remove(gpio_num) -> ESP_OK */
static void stub_gpio_isr_handler_remove(xtensa_cpu_t *cpu, void *ctx) {
    esp32_rom_stubs_t *s = ctx;
    uint32_t pin = rom_arg(cpu, 0);
    if (s && pin < 40u) {
        s->gpio_isr[pin].fn = 0;
        s->gpio_isr[pin].arg = 0;
        gpio_pin_reg_update(cpu, pin, 0x1Fu << GPIO_PIN_INT_ENA_SHIFT, 0u);
    }
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
 * Heap allocator for firmware malloc/free/calloc/realloc. Ordinary allocations
 * use emulated PSRAM. Requests that explicitly require DMA/internal/exec memory
 * use a separate internal-DRAM arena because classic ESP32 DMA cannot address
 * external SPI RAM. Each block has an 8-byte [size, magic] header.
 */

static uint32_t freelist_alloc(xtensa_cpu_t *cpu,
                               stub_heap_region_t *region, uint32_t size) {
    uint32_t prev_addr = 0;
    uint32_t cur = region->free_list;
    while (cur) {
        uint32_t block_size = mem_read32(cpu->mem, cur);
        uint32_t next = mem_read32(cpu->mem, cur + HEAP_HDR_SZ);  /* next ptr in user area */
        if (block_size >= size) {
            /* Unlink from free list */
            if (prev_addr)
                mem_write32(cpu->mem, prev_addr + HEAP_HDR_SZ, next);
            else
                region->free_list = next;
            /* Mark as allocated */
            mem_write32(cpu->mem, cur + 4, HEAP_MAGIC);
            return cur + HEAP_HDR_SZ;
        }
        prev_addr = cur;
        cur = next;
    }
    return 0;
}

static uint32_t heap_alloc(xtensa_cpu_t *cpu, stub_heap_region_t *region,
                           uint32_t size) {
    if (size == 0) return 0;
    /* A free block stores its next pointer in the first user word. */
    if (size < 4) size = 4;
    size = (size + 3) & ~3u;
    /* Try free list first */
    uint32_t ptr = freelist_alloc(cpu, region, size);
    if (ptr) return ptr;
    /* Bump allocate */
    uint32_t total = HEAP_HDR_SZ + size;
    if (region->ptr > region->end || total > region->end - region->ptr) {
        fprintf(stderr,
                "[heap] OOM in %s arena: need %u, have %u free_list=%u\n",
                region->base == INTERNAL_HEAP_BASE ? "internal" : "PSRAM",
                total, region->ptr <= region->end ? region->end - region->ptr : 0,
                region->free_list ? 1 : 0);
        return 0;
    }
    uint32_t block = region->ptr;
    mem_write32(cpu->mem, block, size);
    mem_write32(cpu->mem, block + 4, HEAP_MAGIC);
    region->ptr += total;
    return block + HEAP_HDR_SZ;
}

static stub_heap_region_t *heap_region_for_ptr(esp32_rom_stubs_t *s,
                                               uint32_t ptr) {
    if (ptr >= s->heap.base + HEAP_HDR_SZ && ptr < s->heap.end)
        return &s->heap;
    if (ptr >= s->internal_heap.base + HEAP_HDR_SZ &&
        ptr < s->internal_heap.end)
        return &s->internal_heap;
    return NULL;
}

static stub_heap_region_t *heap_region_for_caps(esp32_rom_stubs_t *s,
                                                uint32_t caps) {
    if (caps & (MALLOC_CAP_DMA_BIT | MALLOC_CAP_EXEC_BIT |
                MALLOC_CAP_INTERNAL_BIT))
        return &s->internal_heap;
    return &s->heap;
}

static void heap_free(xtensa_cpu_t *cpu, esp32_rom_stubs_t *s,
                      uint32_t ptr) {
    if (ptr == 0) return;
    stub_heap_region_t *region = heap_region_for_ptr(s, ptr);
    if (!region) return;
    uint32_t block = ptr - HEAP_HDR_SZ;
    uint32_t magic = mem_read32(cpu->mem, block + 4);
    if (magic != HEAP_MAGIC) return;  /* double free or corruption */
    uint32_t size = mem_read32(cpu->mem, block);
    uint32_t total = HEAP_HDR_SZ + ((size + 3) & ~3u);

    /* If this is the topmost block, shrink the heap */
    if (block + total == region->ptr) {
        mem_write32(cpu->mem, block + 4, 0);  /* clear magic */
        region->ptr = block;
        /* Single-block reclaim handles the common malloc-then-free pattern.
         * We can't walk backwards because block sizes vary. */
        return;
    }
    /* Otherwise, add to free list */
    mem_write32(cpu->mem, block + 4, HEAP_FREE);
    mem_write32(cpu->mem, ptr, region->free_list);  /* next in user data */
    region->free_list = block;
}

/* malloc(size) -> pointer or NULL */
static void stub_malloc(xtensa_cpu_t *cpu, void *ctx) {
    esp32_rom_stubs_t *s = ctx;
    uint32_t size = rom_arg(cpu, 0);
    uint32_t ptr = heap_alloc(cpu, &s->heap, size);
    if (size >= 1024)
        fprintf(stderr, "[heap] malloc(%u) -> 0x%08X (free=%u)\n",
                size, ptr, s->heap.end - s->heap.ptr);
    rom_return(cpu, ptr);
}

/* calloc(nmemb, size) -> pointer or NULL (zeroed) */
static void stub_calloc(xtensa_cpu_t *cpu, void *ctx) {
    esp32_rom_stubs_t *s = ctx;
    uint32_t nmemb = rom_arg(cpu, 0);
    uint32_t size  = rom_arg(cpu, 1);
    if (size != 0 && nmemb > UINT32_MAX / size) {
        rom_return(cpu, 0);
        return;
    }
    uint32_t total = nmemb * size;
    uint32_t ptr = heap_alloc(cpu, &s->heap, total);
    if (ptr) {
        for (uint32_t i = 0; i < total; i++)
            mem_write8(cpu->mem, ptr + i, 0);
    }
    rom_return(cpu, ptr);
}

/* free(ptr) -> void */
static void stub_free(xtensa_cpu_t *cpu, void *ctx) {
    esp32_rom_stubs_t *s = ctx;
    uint32_t ptr = rom_arg(cpu, 0);
    if (heap_region_for_ptr(s, ptr)) {
        uint32_t size = mem_read32(cpu->mem, ptr - HEAP_HDR_SZ);
        if (size >= 1024)
            fprintf(stderr, "[heap] free(0x%08X) size=%u\n", ptr, size);
    }
    heap_free(cpu, s, ptr);
    rom_return_void(cpu);
}

/* realloc(ptr, size) -> pointer or NULL */
static uint32_t heap_realloc(xtensa_cpu_t *cpu, esp32_rom_stubs_t *s,
                             stub_heap_region_t *target, uint32_t old_ptr,
                             uint32_t new_size) {
    if (new_size == 0) {
        heap_free(cpu, s, old_ptr);
        return 0;
    }
    uint32_t new_ptr = heap_alloc(cpu, target, new_size);
    if (new_ptr && old_ptr) {
        stub_heap_region_t *old_region = heap_region_for_ptr(s, old_ptr);
        if (old_region) {
            uint32_t old_size = mem_read32(cpu->mem,
                                           old_ptr - HEAP_HDR_SZ);
            uint32_t copy = old_size < new_size ? old_size : new_size;
            for (uint32_t i = 0; i < copy; i++)
                mem_write8(cpu->mem, new_ptr + i,
                           mem_read8(cpu->mem, old_ptr + i));
            heap_free(cpu, s, old_ptr);
        }
    }
    return new_ptr;
}

static void stub_realloc(xtensa_cpu_t *cpu, void *ctx) {
    esp32_rom_stubs_t *s = ctx;
    uint32_t old_ptr = rom_arg(cpu, 0);
    uint32_t new_size = rom_arg(cpu, 1);
    stub_heap_region_t *target = old_ptr ? heap_region_for_ptr(s, old_ptr) : NULL;
    if (!target) target = &s->heap;
    uint32_t new_ptr = heap_realloc(cpu, s, target, old_ptr, new_size);
    rom_return(cpu, new_ptr);
}

/* heap_caps_malloc(size, caps) */
void stub_heap_caps_malloc(xtensa_cpu_t *cpu, void *ctx) {
    esp32_rom_stubs_t *s = ctx;
    uint32_t size = rom_arg(cpu, 0);
    uint32_t caps = rom_arg(cpu, 1);
    stub_heap_region_t *region = heap_region_for_caps(s, caps);
    uint32_t ptr = heap_alloc(cpu, region, size);
    if (getenv("FLEXE_HEAPDBG"))
        fprintf(stderr,
                "[heap] heap_caps_malloc(%u, 0x%08X) -> 0x%08X (%s)\n",
                size, caps, ptr,
                region == &s->internal_heap ? "internal" : "PSRAM");
    rom_return(cpu, ptr);
}

/* heap_caps_calloc(nmemb, size, caps) */
void stub_heap_caps_calloc(xtensa_cpu_t *cpu, void *ctx) {
    esp32_rom_stubs_t *s = ctx;
    uint32_t nmemb = rom_arg(cpu, 0);
    uint32_t size = rom_arg(cpu, 1);
    uint32_t caps = rom_arg(cpu, 2);
    if (size != 0 && nmemb > UINT32_MAX / size) {
        rom_return(cpu, 0);
        return;
    }
    uint32_t total = nmemb * size;
    uint32_t ptr = heap_alloc(cpu, heap_region_for_caps(s, caps), total);
    if (ptr) {
        for (uint32_t i = 0; i < total; i++)
            mem_write8(cpu->mem, ptr + i, 0);
    }
    rom_return(cpu, ptr);
}

/* heap_caps_realloc(ptr, size, caps) */
void stub_heap_caps_realloc(xtensa_cpu_t *cpu, void *ctx) {
    esp32_rom_stubs_t *s = ctx;
    uint32_t old_ptr = rom_arg(cpu, 0);
    uint32_t size = rom_arg(cpu, 1);
    uint32_t caps = rom_arg(cpu, 2);
    rom_return(cpu, heap_realloc(cpu, s, heap_region_for_caps(s, caps),
                                 old_ptr, size));
}

/* esp_get_free_heap_size() -> remaining heap bytes */
void stub_esp_get_free_heap_size(xtensa_cpu_t *cpu, void *ctx) {
    esp32_rom_stubs_t *s = ctx;
    rom_return(cpu, s->heap.end - s->heap.ptr);
}

/* esp_get_minimum_free_heap_size() -> remaining heap bytes */
void stub_esp_get_minimum_free_heap_size(xtensa_cpu_t *cpu, void *ctx) {
    esp32_rom_stubs_t *s = ctx;
    rom_return(cpu, s->heap.end - s->heap.ptr);
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
/* _xtos_set_intlevel(level) — emulate the ROM function: return the current
 * PS, then set PS.INTLEVEL = level (what the ROM's RSIL does). */
static void stub_xtos_set_intlevel(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    uint32_t old_ps = cpu->ps;
    uint32_t level = rom_arg(cpu, 0);
    cpu->ps = (cpu->ps & ~0xFu) | (level & 0xFu);
    cpu->irq_check = true;
    rom_return(cpu, old_ps);
}

/* mmu_init(cpu_no) — invalidate the selected core's flash-MMU table. */
static void stub_mmu_init(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    uint32_t core = rom_arg(cpu, 0);
    if (core <= 1u) {
        uint32_t table = core == 0u ? 0x3FF10000u : 0x3FF12000u;
        for (uint32_t entry = 0; entry < 256u; entry++)
            mem_write32(cpu->mem, table + entry * 4u, 0x100u);
    }
    rom_return_void(cpu);
}

static int cache_flash_mmu_vaddr_entry(uint32_t vaddr, uint32_t num,
                                       uint32_t *entry_out) {
    static const struct {
        uint32_t low;
        uint32_t high;
        uint32_t entry_base;
    } regions[] = {
        { 0x3F400000u, 0x3F800000u,   0u }, /* DROM0 */
        { 0x400D0000u, 0x40400000u,  64u }, /* IRAM0 */
        { 0x40400000u, 0x40800000u, 128u }, /* IRAM1 */
        { 0x40800000u, 0x40C00000u, 192u }, /* IROM0 */
    };
    uint64_t end = (uint64_t)vaddr + (uint64_t)num * 0x10000u;
    for (size_t i = 0; i < sizeof(regions) / sizeof(regions[0]); i++) {
        if (vaddr < regions[i].low || vaddr >= regions[i].high) continue;
        if (end > regions[i].high) return 4;
        *entry_out = regions[i].entry_base +
                     ((vaddr & 0x3FFFFFu) >> 16);
        return 0;
    }
    return 5;
}

/* cache_flash_mmu_set(cpu_no, pid, vaddr, paddr, psize, num) — model the
 * classic ESP32 ROM API. vaddr/paddr are byte addresses (not page numbers),
 * psize is in KiB, and each MMU table entry controls a complete 64 KiB page. */
static void stub_cache_flash_mmu_set(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    uint32_t core  = rom_arg(cpu, 0);
    uint32_t pid   = rom_arg(cpu, 1);
    uint32_t vaddr = rom_arg(cpu, 2);
    uint32_t paddr = rom_arg(cpu, 3);
    uint32_t psize = rom_arg(cpu, 4);
    uint32_t num   = rom_arg(cpu, 5);
    if (getenv("FLEXE_DBG_FLASH"))
        fprintf(stderr,
                "[MMU] core=%u pid=%u vaddr=0x%08X paddr=0x%08X "
                "psize=%u num=%u\n",
                core, pid, vaddr, paddr, psize, num);

    if ((vaddr & 0xFFFFu) || (paddr & 0xFFFFu)) {
        rom_return(cpu, 1);
        return;
    }
    if (psize != 64u) {
        rom_return(cpu, 3);
        return;
    }
    /* PID0/PID1 are the two hardware layouts used by ESP-IDF. The remaining
     * legacy PID windows are not exposed in the ESP32 application map. */
    if (core > 1u || pid > 1u) {
        rom_return(cpu, 2);
        return;
    }

    uint32_t entry;
    int result = cache_flash_mmu_vaddr_entry(vaddr, num, &entry);
    if (result != 0 || (uint64_t)paddr + (uint64_t)num * 0x10000u > 0x1000000ull) {
        rom_return(cpu, result != 0 ? (uint32_t)result : 4u);
        return;
    }

    uint32_t table = core == 0u ? 0x3FF10000u : 0x3FF12000u;
    uint32_t physical_page = paddr >> 16;
    for (uint32_t i = 0; i < num; i++)
        mem_write32(cpu->mem, table + (entry + i) * 4u,
                    physical_page + i);
    rom_return(cpu, 0);
}

static void stub_unregistered(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    rom_return(cpu, 0);
}

/* Compatibility-mode interrupt bridge. FreeRTOS is replaced in this mode,
 * but interrupt-driven IDF drivers still depend on their registered ISR to
 * recycle I2S DMA buffers and complete I2C command queues. Invoke those
 * handlers synchronously on a private guest stack. The pending loop prevents
 * recursive ISR calls when a handler starts more hardware work. */
#define STUB_INTR_HANDLE_BASE 0xF17E0000u

static int stub_intr_source_from_handle(const esp32_rom_stubs_t *s,
                                        uint32_t handle) {
    for (int source = 0; source < 71; source++) {
        if (s->irq[source].handle == handle && handle != 0)
            return source;
    }
    return -1;
}

static void stub_dispatch_guest_irq(void *ctx, int source) {
    esp32_rom_stubs_t *s = ctx;
    if (!s || source < 0 || source >= 71)
        return;
    stub_irq_t *irq = &s->irq[source];
    if (!irq->enabled || !irq->cpu || irq->handler == 0)
        return;
    irq->pending = true;
    if (irq->dispatching)
        return;

    irq->dispatching = true;
    unsigned iterations = 0;
    while (irq->pending && iterations++ < 256) {
        irq->pending = false;
        uint32_t args[] = { irq->arg };
        int result = guest_call8(irq->cpu, irq->handler, args, 1,
                                 2000000u, NULL);
        if (result != 0) {
            fprintf(stderr,
                    "[intr] guest ISR source=%d handler=0x%08X failed (%d)\n",
                    source, irq->handler, result);
            irq->enabled = false;
            break;
        }
    }
    if (iterations > 256) {
        fprintf(stderr, "[intr] guest ISR source=%d did not quiesce\n", source);
        irq->enabled = false;
    }
    irq->dispatching = false;
}

static void stub_esp_intr_alloc_common(xtensa_cpu_t *cpu,
                                       esp32_rom_stubs_t *s,
                                       bool has_status_args) {
    int source = (int)rom_arg(cpu, 0);
    int handler_arg = has_status_args ? 4 : 2;
    int context_arg = has_status_args ? 5 : 3;
    int output_arg = has_status_args ? 6 : 4;
    uint32_t handler = rom_arg(cpu, handler_arg);
    uint32_t arg = rom_arg(cpu, context_arg);
    uint32_t output = rom_arg(cpu, output_arg);

    /* Preserve the compatibility layer's historical no-op behavior for
     * interrupt sources whose peripheral ISR path is not modeled yet.  Some
     * production drivers use a non-NULL handle to select management paths
     * that cannot work while their interrupt is intentionally stubbed.
     *
     * SPI2/SPI3 (30/31) are modelled and must be here: ESP-IDF's spi_master
     * allocates its interrupt with ESP_INTR_FLAG_INTRDISABLED and performs
     * every transaction from the ISR, so a no-op alloc left the driver
     * blocked on its completion semaphore forever. Nothing noticed because
     * the CYD display libraries drive the SPI registers directly and never
     * go near the driver. */
    if (source != 10 && source != 11 &&
        source != 12 && source != 13 && source != 14 && source != 15 &&
        source != 18 && source != 19 &&
        source != 22 &&
        source != 30 && source != 31 &&
        source != 32 && source != 33 && source != 37 && source != 38 &&
        source != 39 && source != 40 &&
        source != 43 && source != 45 && source != 47 && source != 48 && source != 49 &&
        source != 50 && source != 56 && source != 57 && source != 58 &&
        source != 59 && source != 62 && source != 63) {
        rom_return(cpu, 0);
        return;
    }

    if (source >= 0 && source < 71) {
        uint32_t handle = STUB_INTR_HANDLE_BASE | (uint32_t)source;
        s->irq[source].cpu = cpu;
        s->irq[source].handler = handler;
        s->irq[source].arg = arg;
        s->irq[source].handle = handle;
        s->irq[source].enabled = true;
        s->irq[source].pending = false;
        if (output)
            mem_write32(cpu->mem, output, handle);

        if (s->periph)
            periph_set_irq_dispatch(s->periph, source,
                                    stub_dispatch_guest_irq, s);
    } else if (output) {
        mem_write32(cpu->mem, output, 0);
    }
    rom_return(cpu, 0); /* ESP_OK */
}

static void stub_esp_intr_alloc(xtensa_cpu_t *cpu, void *ctx) {
    stub_esp_intr_alloc_common(cpu, ctx, false);
}

static void stub_esp_intr_alloc_intrstatus(xtensa_cpu_t *cpu, void *ctx) {
    stub_esp_intr_alloc_common(cpu, ctx, true);
}

static void stub_esp_intr_free(xtensa_cpu_t *cpu, void *ctx) {
    esp32_rom_stubs_t *s = ctx;
    int source = stub_intr_source_from_handle(s, rom_arg(cpu, 0));
    if (source >= 0) {
        if (s->periph)
            periph_set_irq_dispatch(s->periph, source, NULL, NULL);
        memset(&s->irq[source], 0, sizeof(s->irq[source]));
    }
    rom_return(cpu, 0);
}

static void stub_esp_intr_enable(xtensa_cpu_t *cpu, void *ctx) {
    esp32_rom_stubs_t *s = ctx;
    int source = stub_intr_source_from_handle(s, rom_arg(cpu, 0));
    if (source >= 0) {
        s->irq[source].enabled = true;
        /* Level-triggered hardware may have asserted while the handle was
         * disabled. Re-enabling must service that level even though there is
         * no new low-to-high edge for the peripheral observer to report. */
        if (s->periph && periph_interrupt_pending(s->periph, source))
            stub_dispatch_guest_irq(s, source);
    }
    rom_return(cpu, 0);
}

static void stub_esp_intr_disable(xtensa_cpu_t *cpu, void *ctx) {
    esp32_rom_stubs_t *s = ctx;
    int source = stub_intr_source_from_handle(s, rom_arg(cpu, 0));
    if (source >= 0)
        s->irq[source].enabled = false;
    rom_return(cpu, 0);
}

/* Generic void ROM stub: returns without a value */
static void stub_void_unregistered(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    rom_return_void(cpu);
}

/* ESP32 rev-0 Bluetooth controller ROM exports return pointers to private
 * controller tables. The ROM bytes are not executed by Flexe, but the IDF
 * binary blob still calls these accessors and then writes through the
 * returned pointers. Give it isolated RTC-fast scratch structures instead
 * of the old unregistered-ROM fallback (which returned NULL and made reset
 * loops walk low memory looking for terminators). */
#define ROM_BT_PLF_PTR_SLOT       0x50000DFCu
#define ROM_BT_RF_PHY_FUNCS       0x50000E00u
#define ROM_BT_IP_FUNCS           0x50000E40u
#define ROM_BT_MODULES_FUNCS      0x50001000u /* blob writes through +0x19C */
#define ROM_BT_OPTION_DATA        0x50001200u
#define ROM_BT_LC_DEFAULT_TABLE   0x50001300u
#define ROM_BT_LC_HCI_TABLE       0x50001320u
#define ROM_BT_LLCP_STATE_TABLE   0x50001400u /* 22 indexed handler slots */
#define ROM_BT_LLM_HCI_TABLE      0x50001500u /* 37 entries, eight bytes each */
#define ROM_BT_LLC_DEFAULT_TABLE  0x50000C00u
#define ROM_BT_LM_HCI_TABLE       0x50000C40u
#define ROM_BT_LM_DEFAULT_TABLE   0x50000C80u
#define ROM_BT_LLC_HCI_TABLE      0x50000CC0u
#define ROM_BT_LLM_DEFAULT_TABLE  0x50000D00u
#define ROM_PHY_FUNCS              0x50001900u /* IDF patches through +0x1A4 */
#define VIRTUAL_PHY_NOOP_FN        0x4006FFF0u

/* Initial g_phyFuns_instance from Espressif's ESP32 rev-0 ROM.  Production
 * libphy replaces the chip-specific entries it owns and continues to call
 * the remaining ROM entries indirectly.  Preserve that ABI even though the
 * analog/RF work itself is virtualized below. */
static const uint32_t esp32_rev0_phy_romfuncs[] = {
    0x40002F6Cu, 0x40002F88u, 0x40002FA4u, 0x40002FCCu,
    0x40003000u, 0x4000302Cu, 0x40003044u, 0x40003E3Cu,
    0x40003060u, 0x400030B8u, 0x400030F8u, 0x4000312Cu,
    0x400031A4u, 0x4000348Cu, 0x4000351Cu, 0x40003564u,
    0x40003594u, 0x400035D0u, 0x400036B4u, 0x40003F98u,
    0x4000401Cu, 0x40003710u, 0x40003734u, 0x40003760u,
    0x400037F0u, 0x40003AC8u, 0x40003B70u, 0x4000404Cu,
    0x40003BACu, 0x400040B0u, 0x40003BDCu, 0x40003C2Cu,
    0x40003C78u, 0x40003D48u, 0x40003D90u, 0x40003DB4u,
    0x40003DF4u, 0x40004110u, 0x40004148u, 0x40004168u,
    0x400041A4u, 0x400041C0u, 0x400041FCu, 0x40004270u,
    0x40004334u, 0x40004374u, 0x400043C0u, 0x40004414u,
    0x40004458u, 0x4000446Cu, 0x40004480u, 0x40004508u,
    0x4000453Cu, 0x40004590u, 0x400045E0u, 0x40004638u,
    0x40004680u, 0x400046E0u, 0x40004740u, 0x400047A8u,
    0x400047F8u, 0x40004880u, 0x40004B44u, 0x40004CA8u,
    0x40004CECu, 0x40004D18u, 0x40004D8Cu, 0x40004DC0u,
    0x40004DF8u, 0x40004E10u, 0x40004EA4u, 0x4000506Cu,
    0x00000000u, 0x4000510Cu, 0x40005154u, 0x400051C0u,
    0x40005204u, 0x40005290u, 0x400052DCu, 0x4000538Cu,
    0x400054F0u, 0x40005514u, 0x40005590u, 0x400055C8u,
    0x40005620u, 0x400058E4u, 0x40005A00u, 0x40005A68u,
    0x40005B4Cu, 0x40005BBCu, 0x40005CE0u, 0x40005D50u,
    0x40005DECu, 0x40005F64u, 0x40005FC8u, 0x40006004u,
    0x00000000u, 0x40006058u, 0x400061CCu, 0x40006268u,
    0x40006290u, 0x400062A8u, 0x4000642Cu, 0x40006564u,
    0x400065D4u, 0x4000662Cu,
};

/* phy_get_romfuncs() returns a persistent function table which libphy patches
 * during initialization.  The actual ROM table cannot be executed by the
 * emulator, but returning NULL lets those writes corrupt low memory.  Keep a
 * dedicated, writable table in RTC-fast RAM; higher-level virtual WiFi/PHY
 * hooks provide the operations which would otherwise live in its slots. */
static void stub_phy_get_romfuncs(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    rom_return(cpu, ROM_PHY_FUNCS);
}

static void init_phy_romfuncs(esp32_rom_stubs_t *s) {
    for (size_t i = 0;
         i < sizeof(esp32_rev0_phy_romfuncs) /
                     sizeof(esp32_rev0_phy_romfuncs[0]); i++) {
        uint32_t addr = esp32_rev0_phy_romfuncs[i];
        mem_write32(s->cpu->mem, ROM_PHY_FUNCS + (uint32_t)i * 4u, addr);
        if (addr != 0)
            rom_stubs_register(s, addr, stub_unregistered,
                               "virtual_phy_romfunc");
    }
    rom_stubs_register(s, VIRTUAL_PHY_NOOP_FN, stub_unregistered,
                       "virtual_phy_operation");
}

/* A real ESP32's libphy replaces much of this table and then runs closed,
 * chip-specific RF calibration code.  There is no analog radio behind the
 * virtual CYD, so expose the same writable ABI while making every operation
 * deterministic.  Keep the two NULL slots from the physical ROM intact. */
static void virtualize_phy_romfuncs(esp32_rom_stubs_t *s) {
    for (size_t i = 0;
         i < sizeof(esp32_rev0_phy_romfuncs) /
                     sizeof(esp32_rev0_phy_romfuncs[0]); i++) {
        uint32_t fn = esp32_rev0_phy_romfuncs[i] != 0
                          ? VIRTUAL_PHY_NOOP_FN
                          : 0;
        mem_write32(s->cpu->mem, ROM_PHY_FUNCS + (uint32_t)i * 4u, fn);
    }
}

static void stub_bt_rom_rf_phy_funcs_get(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    rom_return(cpu, ROM_BT_RF_PHY_FUNCS);
}

static void stub_bt_rom_ip_funcs_get(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    rom_return(cpu, ROM_BT_IP_FUNCS);
}

static void stub_bt_rom_modules_funcs_get(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    rom_return(cpu, ROM_BT_MODULES_FUNCS);
}

static void stub_bt_rom_plf_funcs_set(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    mem_write32(cpu->mem, ROM_BT_PLF_PTR_SLOT, rom_arg(cpu, 0));
    rom_return(cpu, 0);
}

static void stub_bt_rom_option_data_get(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    rom_return(cpu, ROM_BT_OPTION_DATA);
}

static void bt_rom_make_terminated_table(xtensa_cpu_t *cpu, uint32_t addr,
                                         uint16_t terminator) {
    mem_write16(cpu->mem, addr, terminator);
    mem_write16(cpu->mem, addr + 2, 0);
    mem_write32(cpu->mem, addr + 4, 0);
    rom_return(cpu, addr);
}

static void stub_bt_rom_lc_default_table_get(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    bt_rom_make_terminated_table(cpu, ROM_BT_LC_DEFAULT_TABLE, 0x0522u);
}

static void stub_bt_rom_lc_hci_table_get(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    bt_rom_make_terminated_table(cpu, ROM_BT_LC_HCI_TABLE, 0x0C7Cu);
}

static void stub_bt_rom_llcp_state_table_get(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    rom_return(cpu, ROM_BT_LLCP_STATE_TABLE);
}

static void stub_bt_rom_llm_hci_table_get(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    rom_return(cpu, ROM_BT_LLM_HCI_TABLE);
}

static void bt_rom_make_tagged_table(xtensa_cpu_t *cpu, uint32_t addr,
                                     const uint16_t *tags, size_t count) {
    for (size_t i = 0; i < count; i++) {
        mem_write16(cpu->mem, addr + (uint32_t)i * 8u, tags[i]);
        mem_write16(cpu->mem, addr + (uint32_t)i * 8u + 2u, 0);
        mem_write32(cpu->mem, addr + (uint32_t)i * 8u + 4u, 0);
    }
    rom_return(cpu, addr);
}

static void stub_bt_rom_llc_default_table_get(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    static const uint16_t tags[] = {0x0104, 0x0807, 0x0101, 0x0005};
    bt_rom_make_tagged_table(cpu, ROM_BT_LLC_DEFAULT_TABLE, tags,
                             sizeof(tags) / sizeof(tags[0]));
}

static void stub_bt_rom_lm_hci_table_get(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    static const uint16_t tags[] = {
        0x0408, 0x0C35, 0x0C1A, 0x0C3F, 0x1407, 0x1804
    };
    bt_rom_make_tagged_table(cpu, ROM_BT_LM_HCI_TABLE, tags,
                             sizeof(tags) / sizeof(tags[0]));
}

static void stub_bt_rom_lm_default_table_get(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    static const uint16_t tags[] = {0x0401, 0x0403, 0x0404, 0x040B};
    bt_rom_make_tagged_table(cpu, ROM_BT_LM_DEFAULT_TABLE, tags,
                             sizeof(tags) / sizeof(tags[0]));
}

static void stub_bt_rom_llc_hci_table_get(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    static const uint16_t tags[] = {
        0x2013, 0x0406, 0x0C2D, 0x201B, 0x2020, 0x2016, 0xFC43
    };
    bt_rom_make_tagged_table(cpu, ROM_BT_LLC_HCI_TABLE, tags,
                             sizeof(tags) / sizeof(tags[0]));
}

static void stub_bt_rom_llm_default_table_get(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    static const uint16_t tags[] = {0x0009, 0x0200, 0x0001, 0x0805};
    bt_rom_make_tagged_table(cpu, ROM_BT_LLM_DEFAULT_TABLE, tags,
                             sizeof(tags) / sizeof(tags[0]));
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

/* Classic ESP32 ROM GPIO-matrix helpers. ESP-IDF's peripheral drivers call
 * these absolute ROM addresses instead of writing GPIO_FUNC_* directly, so
 * dropping the calls leaves SPI/I2C/UART signals disconnected in the model. */
static void stub_gpio_matrix_in(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    uint32_t gpio = rom_arg(cpu, 0);
    uint32_t signal = rom_arg(cpu, 1);
    uint32_t invert = rom_arg(cpu, 2);
    if (signal < 256u) {
        uint32_t value = (gpio & 0x3Fu) | ((invert & 1u) << 6) | (1u << 7);
        mem_write32(cpu->mem, 0x3FF44130u + signal * 4u, value);
    }
    rom_return_void(cpu);
}

static void stub_gpio_matrix_out(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    uint32_t gpio = rom_arg(cpu, 0);
    uint32_t signal = rom_arg(cpu, 1);
    uint32_t out_invert = rom_arg(cpu, 2);
    uint32_t oen_invert = rom_arg(cpu, 3);
    if (getenv("FLEXE_GPIODBG"))
        fprintf(stderr,
                "[GPIO] gpio_matrix_out(gpio=%u signal=%u out_inv=%u "
                "oen_inv=%u)\n",
                gpio, signal, out_invert, oen_invert);
    if (gpio < 40u) {
        uint32_t value = (signal & 0x1FFu) |
                         ((out_invert & 1u) << 9) |
                         ((oen_invert & 1u) << 11);
        mem_write32(cpu->mem, 0x3FF44530u + gpio * 4u, value);
    }
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

/* Return the second argument unchanged (arg1 passthrough).  Used for the
 * xEventGroupWaitBits wrapper (0x4011dab4) so the caller sees the requested
 * event-group bits set. */
static void stub_ret_arg1(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    int ci = XT_PS_CALLINC(cpu->ps);
    rom_return(cpu, ar_read(cpu, ci * 4 + 3));
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
        s->direct[di].conditional_fn = e->conditional_fn;
        s->direct[di].ctx = e->user_ctx ? e->user_ctx : s;
        s->direct[di].call_count = &e->call_count;
        s->direct[di].spy = e->spy;
    }
}

static int rom_pc_hook(xtensa_cpu_t *cpu, uint32_t pc, void *ctx) {
    esp32_rom_stubs_t *s = ctx;

    /* Fast path: direct dispatch table — single indexed lookup, no hash.
     * A calloc'd empty slot has tag==0 and must NOT match pc==0 (its fn and
     * call_count are NULL — dereferencing them crashes the host). Require
     * fn to be set, so only real registrations match. */
    if (__builtin_expect(s->direct != NULL, 1)) {
        uint32_t di = (pc >> 2) & STUB_DIRECT_MASK;
        stub_direct_entry_t *de = &s->direct[di];
        if (__builtin_expect(de->tag == pc &&
            (de->fn != NULL || de->conditional_fn != NULL), 1)) {
            s->total_calls++;
            (*de->call_count)++;
            if (de->conditional_fn)
                return de->conditional_fn(cpu, de->ctx) ? 1 : 0;
            de->fn(cpu, de->ctx);
            if (de->spy)
                return 0; /* spy: let original instruction execute */
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
        if (s->entries[idx].conditional_fn)
            return s->entries[idx].conditional_fn(cpu, ectx) ? 1 : 0;
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
    s->heap = (stub_heap_region_t){
        .base = HEAP_BASE,
        .end = HEAP_END,
        .ptr = HEAP_BASE,
    };
    s->internal_heap = (stub_heap_region_t){
        .base = INTERNAL_HEAP_BASE,
        .end = INTERNAL_HEAP_END,
        .ptr = INTERNAL_HEAP_BASE,
    };

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
    rom_stubs_register(s, 0x4000855c, stub_ets_get_cpu_frequency, "ets_get_cpu_frequency");
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
    rom_stubs_register(s, 0x40004100, stub_phy_get_romfuncs,     "phy_get_romfuncs");
    init_phy_romfuncs(s);
    rom_stubs_register(s, 0x40004148, stub_unregistered,        "rom_i2c_readReg");
    rom_stubs_register(s, 0x400041a4, stub_void_unregistered,   "rom_i2c_writeReg");
    rom_stubs_register(s, 0x400041c0, stub_unregistered,        "rom_i2c_readReg_Mask");
    rom_stubs_register(s, 0x400041fc, stub_void_unregistered,   "rom_i2c_writeReg_Mask");

    /* Private ESP32 Bluetooth-controller ROM table accessors. The IDF binary
     * blob expects writable pointer targets even when the host radio backend
     * is virtualized. */
    rom_stubs_register(s, 0x40054298, stub_bt_rom_rf_phy_funcs_get,
                       "btdm_r_import_rf_phy_func_p_get");
    rom_stubs_register(s, 0x40019AF0, stub_bt_rom_ip_funcs_get,
                       "btdm_r_ip_func_p_get");
    rom_stubs_register(s, 0x4005427C, stub_bt_rom_modules_funcs_get,
                       "btdm_r_modules_func_p_get");
    rom_stubs_register(s, 0x40054288, stub_bt_rom_plf_funcs_set,
                       "btdm_r_plf_func_p_set");
    rom_stubs_register(s, 0x40010004, stub_bt_rom_option_data_get,
                       "btdm_r_btdm_option_data_p_get");
    rom_stubs_register(s, 0x4002F494, stub_bt_rom_lc_default_table_get,
                       "lc_default_state_tab_p_get");
    rom_stubs_register(s, 0x4002F488, stub_bt_rom_lc_hci_table_get,
                       "lc_hci_cmd_handler_tab_p_get");
    rom_stubs_register(s, 0x40043F64, stub_bt_rom_llcp_state_table_get,
                       "llcp_pdu_handler_tab_p_get");
    rom_stubs_register(s, 0x4004C920, stub_bt_rom_llm_hci_table_get,
                       "llm_hci_cmd_handler_tab_p_get");
    rom_stubs_register(s, 0x40046058, stub_bt_rom_llc_default_table_get,
                       "llc_default_state_tab_p_get");
    rom_stubs_register(s, 0x4005425C, stub_bt_rom_lm_hci_table_get,
                       "lm_hci_cmd_handler_tab_p_get");
    rom_stubs_register(s, 0x40054268, stub_bt_rom_lm_default_table_get,
                       "lm_default_state_tab_p_get");
    rom_stubs_register(s, 0x40042358, stub_bt_rom_llc_hci_table_get,
                       "llc_hci_cmd_handler_tab_p_get");
    rom_stubs_register(s, 0x4004E718, stub_bt_rom_llm_default_table_get,
                       "llm_default_state_tab_p_get");
    /* Controller operations reached while the virtual BT host is starting.
     * They have no physical link-layer peer, but are deliberate supported
     * outcomes rather than unknown-ROM fallbacks. */
    rom_stubs_register(s, 0x4003617C, stub_unregistered,
                       "r_ld_acl_sniff");
    rom_stubs_register(s, 0x400185BC, stub_void_unregistered,
                       "r_hci_send_2_host");
    rom_stubs_register(s, 0x4001CD54, stub_void_unregistered,
                       "r_lc_auth_cmp");
    rom_stubs_register(s, 0x4001C948, stub_void_unregistered,
                       "r_lc_init");
    rom_stubs_register(s, 0x400542C4, stub_void_unregistered,
                       "nvds_read");
    rom_stubs_register(s, 0x40054358, stub_void_unregistered,
                       "nvds_init_memory");

    /* Interrupt matrix — always program it. Firmware running real
     * FreeRTOS (symbol-less binaries without ELF hooks) needs working
     * FROM_CPU yield interrupts; without them portYIELD never switches
     * context and SMP kernel lists get corrupted by repeated blocking. */
    rom_stubs_register_ctx(s, 0x4000681c, stub_intr_matrix_set, "intr_matrix_set", s);

    /* UART */
    rom_stubs_register(s, 0x40009200, stub_void_unregistered,   "uart_tx_one_char");
    rom_stubs_register(s, 0x40009258, stub_void_unregistered,   "uart_tx_flush");
    rom_stubs_register(s, 0x40009028, stub_void_unregistered,   "uart_tx_switch");

    /* GPIO */
    rom_stubs_register(s, 0x40009edc, stub_gpio_matrix_in,      "gpio_matrix_in");
    rom_stubs_register(s, 0x40009f0c, stub_gpio_matrix_out,     "gpio_matrix_out");
    rom_stubs_register(s, 0x40009fdc, stub_void_unregistered,   "gpio_pad_select_gpio");
    rom_stubs_register(s, 0x4000a22c, stub_void_unregistered,   "gpio_pad_pullup");

    /* MMU/Cache */
    rom_stubs_register(s, 0x400095a4, stub_mmu_init,            "mmu_init");
    rom_stubs_register(s, 0x400095e0, stub_cache_flash_mmu_set, "cache_flash_mmu_set");

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
    rom_stubs_register(s, 0x4000C978, stub_floatundidf,         "__floatundidf");
    rom_stubs_register(s, 0x4000C988, stub_floatdidf,           "__floatdidf");
    rom_stubs_register(s, 0x4000C8B0, stub_floatundisf,         "__floatundisf");
    rom_stubs_register(s, 0x4000C8C0, stub_floatdisf,           "__floatdisf");
    rom_stubs_register(s, 0x40002AC4, stub_fixdfdi,             "__fixdfdi");
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
    rom_stubs_register(s, 0x4000C830, stub_ashrdi3,             "__ashrdi3");
    rom_stubs_register(s, 0x4000C84C, stub_lshrdi3,             "__lshrdi3");

    /* CRC */
    rom_stubs_register(s, 0x4005CFEC, stub_crc32_le,            "esp_rom_crc32_le");
    rom_stubs_register(s, 0x4005D144, stub_crc8,                "esp_crc8");

    /* DEFLATE decompression (used by PNG decoder via sped.c) */
    rom_stubs_register(s, 0x4005ef30, stub_tinfl_decompress,    "tinfl_decompress");

    /* Flash/boot helpers */
    rom_stubs_register(s, 0x40062BC8, stub_unregistered,        "spi_flash_clk_cfg");
    rom_stubs_register(s, 0x40062A6C, stub_void_unregistered,
                       "esp_rom_spiflash_attach");
    rom_stubs_register(s, 0x40008264, stub_software_reset,      "software_reset_cpu");

    /* GPIO */
    rom_stubs_register(s, 0x40009B24, stub_void_unregistered,   "gpio_output_set");
    rom_stubs_register(s, 0x40009B5C, stub_void_unregistered,   "gpio_output_set_high");

    /* Misc */
    rom_stubs_register(s, 0x40008208, stub_void_unregistered,   "set_rtc_memory_crc");
    /* _xtos_set_intlevel: real implementation in ALL modes — does the RSIL
     * the ROM function would do. Firmware running real FreeRTOS calls this
     * to enable/restore interrupts; a no-op here leaves PS.INTLEVEL stuck
     * high forever, so yields and ticks can never fire. Returns old PS. */
    rom_stubs_register(s, 0x4000bfdc, stub_xtos_set_intlevel,   "_xtos_set_intlevel");
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
    if (stubs->periph) {
        for (int source = 0; source < 71; source++) {
            if (stubs->irq[source].handle != 0)
                periph_set_irq_dispatch(stubs->periph, source, NULL, NULL);
        }
    }
    /* Unhook */
    if (stubs->cpu->pc_hook == rom_pc_hook) {
        stubs->cpu->pc_hook = NULL;
        stubs->cpu->pc_hook_ctx = NULL;
        stubs->cpu->pc_hook_bitmap = NULL;
    }
    free(stubs->direct);
    free(stubs);
}

/* Address-based hooks for symbol-less popular firmwares. These stub the
 * driver-init entry points whose real implementations would spawn driver
 * tasks that crash on null contexts (no radio/hardware attached). Select an
 * exact profile from the entry point plus verified instruction anchors before
 * installing any build-specific addresses. */
typedef struct {
    uint32_t addr;
    rom_stub_fn fn;
    const char *name;
    int spy;                /* nonzero: run fn as side effect, then execute
                             * the real instruction (rom_stubs_register_spy) */
} fw_addr_hook_t;

static bool fw_signature_matches(xtensa_mem_t *mem, uint32_t addr,
                                 const uint8_t *signature, size_t size) {
    for (size_t i = 0; i < size; i++) {
        if (mem_read8(mem, addr + (uint32_t)i) != signature[i])
            return false;
    }
    return true;
}

rom_firmware_profile_t rom_stubs_identify_firmware(
        esp32_rom_stubs_t *stubs, uint32_t entry_point) {
    if (!stubs || !stubs->cpu || !stubs->cpu->mem)
        return ROM_FIRMWARE_UNKNOWN;

    rom_firmware_profile_t profile = ROM_FIRMWARE_UNKNOWN;
    if (entry_point == 0x40089268u) {
        profile = ROM_FIRMWARE_NERDMINER_V183;
    } else if (entry_point == 0x400831D8u) {
        static const uint8_t old_phy[] = {
            0x36, 0x41, 0x00, 0x81, 0xFE, 0xFF, 0xE0, 0x08,
            0x00, 0x81, 0xC5, 0xF0, 0xA9, 0x08, 0x3D, 0xF0,
        };
        static const uint8_t new_phy[] = {
            0x36, 0x41, 0x00, 0x81, 0xFE, 0xFF, 0xE0, 0x08,
            0x00, 0x81, 0xC4, 0xF0, 0xA9, 0x08, 0x3D, 0xF0,
        };
        /* v1.15.x relinked again: the phy wrapper's second L32R offset moved
         * from 0xC4 to 0xBE with the literal pool. */
        static const uint8_t v115_phy[] = {
            0x36, 0x41, 0x00, 0x81, 0xFE, 0xFF, 0xE0, 0x08,
            0x00, 0x81, 0xBE, 0xF0, 0xA9, 0x08, 0x3D, 0xF0,
        };
        static const uint8_t wifi_start[] = {
            0x36, 0x41, 0x00, 0xA5, 0xAE, 0xFF, 0x21, 0x9B,
            0xFE, 0xAC, 0x5A, 0x1C, 0x8A, 0x21, 0x9B, 0xFE,
        };
        xtensa_mem_t *mem = stubs->cpu->mem;
        if (fw_signature_matches(mem, 0x401BDE2Cu,
                                 old_phy, sizeof(old_phy)) &&
            fw_signature_matches(mem, 0x401988A8u,
                                 wifi_start, sizeof(wifi_start)))
            profile = ROM_FIRMWARE_MARAUDER_V1140_1;
        else if (fw_signature_matches(mem, 0x401BE628u,
                                      new_phy, sizeof(new_phy)) &&
                 fw_signature_matches(mem, 0x401990A0u,
                                      wifi_start, sizeof(wifi_start)))
            profile = ROM_FIRMWARE_MARAUDER_V1142_3;
        else if (fw_signature_matches(mem, 0x401C1438u,
                                      v115_phy, sizeof(v115_phy)) &&
                 fw_signature_matches(mem, 0x4019BE94u,
                                      wifi_start, sizeof(wifi_start)))
            profile = ROM_FIRMWARE_MARAUDER_V1151;
    } else if (entry_point == 0x400830D0u) {
        /* v1.14.3 for the other CYD boards. Both carry the v1.14.0/1 phy
         * wrapper bytes verbatim, so only the address the signature sits at
         * separates them. */
        static const uint8_t board_phy[] = {
            0x36, 0x41, 0x00, 0x81, 0xFE, 0xFF, 0xE0, 0x08,
            0x00, 0x81, 0xC5, 0xF0, 0xA9, 0x08, 0x3D, 0xF0,
        };
        static const uint8_t board_wifi_start[] = {
            0x36, 0x41, 0x00, 0xA5, 0xAE, 0xFF, 0x21, 0x9B,
            0xFE, 0xAC, 0x5A, 0x1C, 0x8A, 0x21, 0x9B, 0xFE,
        };
        xtensa_mem_t *mem = stubs->cpu->mem;
        if (fw_signature_matches(mem, 0x401BE0E8u,
                                 board_phy, sizeof(board_phy)) &&
            fw_signature_matches(mem, 0x40198B64u, board_wifi_start,
                                 sizeof(board_wifi_start)))
            profile = ROM_FIRMWARE_MARAUDER_V1143_GUITION;
        else if (fw_signature_matches(mem, 0x401BE274u,
                                      board_phy, sizeof(board_phy)) &&
                 fw_signature_matches(mem, 0x40198CF0u, board_wifi_start,
                                      sizeof(board_wifi_start)))
            profile = ROM_FIRMWARE_MARAUDER_V1143_35INCH;
    }

    stubs->firmware_profile = profile;
    return profile;
}

rom_firmware_profile_t rom_stubs_firmware_profile(
        const esp32_rom_stubs_t *stubs) {
    return stubs ? stubs->firmware_profile : ROM_FIRMWARE_UNKNOWN;
}

/* The symbol-less Marauder image clears three private BT/WiFi dispatch-table
 * globals after its controller setup. A hardware HCI task would repopulate
 * them asynchronously; the virtual controller instead points them at a
 * bounded table of guest return stubs before the first dispatch. */
#define FW_NOOP_FN_V11401 0x401DBFC4u
#define FW_NOOP_FN_V11423 0x401DC890u
#define FW_NOOP_FN_V1151  0x401DF918u
#define FW_NOOP_FN_GUITION 0x401DC394u
#define FW_NOOP_FN_35INCH  0x401DC4F8u
#define FW_FAKE_TBL_BT    0x50000400u
#define FW_FAKE_TBL_WIFI  0x50000800u

typedef struct {
    uint32_t global_addr;
    uint32_t fake_tbl;
} fw_tbl_patch_t;

static void fw_patch_handler_tables(esp32_rom_stubs_t *stubs,
                                    const fw_tbl_patch_t *patches, int n,
                                    uint32_t noop_fn) {
    xtensa_mem_t *mem = stubs->cpu->mem;
    for (int i = 0; i < n; i++) {
        for (int slot = 0; slot < 256; slot++)
            mem_write32(mem, patches[i].fake_tbl + (uint32_t)slot * 4u,
                        noop_fn);
        mem_write32(mem, patches[i].global_addr, patches[i].fake_tbl);
    }
}

static const fw_tbl_patch_t fw_marauder_ble_tbls[] = {
    {0x3FFCD974u, FW_FAKE_TBL_BT},
    {0x3FFD0544u, FW_FAKE_TBL_WIFI},
    {0x3FFD0548u, FW_FAKE_TBL_WIFI},
};

static const fw_tbl_patch_t fw_marauder_v11423_ble_tbls[] = {
    {0x3FFCD984u, FW_FAKE_TBL_BT},
    {0x3FFD0554u, FW_FAKE_TBL_WIFI},
    {0x3FFD0558u, FW_FAKE_TBL_WIFI},
};

/* v1.15.x moved the whole .bss block up by 0x188. */
static const fw_tbl_patch_t fw_marauder_guition_ble_tbls[] = {
    {0x3FFCD774u, FW_FAKE_TBL_BT},
    {0x3FFD0344u, FW_FAKE_TBL_WIFI},
    {0x3FFD0348u, FW_FAKE_TBL_WIFI},
};

static const fw_tbl_patch_t fw_marauder_35inch_ble_tbls[] = {
    {0x3FFCD8B4u, FW_FAKE_TBL_BT},
    {0x3FFD0484u, FW_FAKE_TBL_WIFI},
    {0x3FFD0488u, FW_FAKE_TBL_WIFI},
};

static const fw_tbl_patch_t fw_marauder_v1151_ble_tbls[] = {
    {0x3FFCDB0Cu, FW_FAKE_TBL_BT},
    {0x3FFD06DCu, FW_FAKE_TBL_WIFI},
    {0x3FFD06E0u, FW_FAKE_TBL_WIFI},
};

/* Decode an L32R's literal and return it when it points to firmware DRAM.
 * phy_get_romfunc_addr starts with two L32Rs: the first literal is the ROM
 * accessor (0x40004100), while the second is the writable global into which
 * libphy stores the returned table pointer. */
static bool fw_find_phy_global(xtensa_cpu_t *cpu, uint32_t entry,
                               uint32_t *global_out) {
    uint32_t pc = entry;
    for (int bytes = 0; bytes < 30;) {
        uint32_t insn = 0;
        int len = xtensa_fetch(cpu, pc, &insn);
        if (len != 2 && len != 3)
            break;
        if (len == 3 && XT_OP0(insn) == 1 && XT_T(insn) == 8) {
            uint32_t next_pc = pc + 3u;
            uint32_t literal = (next_pc & ~3u) +
                (0xFFFC0000u | ((uint32_t)XT_IMM16(insn) << 2));
            uint32_t value = mem_read32(cpu->mem, literal);
            if (value >= 0x3FFA0000u && value < 0x40000000u) {
                *global_out = value;
                return true;
            }
        }
        pc += (uint32_t)len;
        bytes += len;
    }

    /* Symbol-less stock-ROM profiles are fixed builds.  Keep their verified
     * globals as a guarded fallback if instruction decoding ever encounters
     * an image whose executable page is not readable through xtensa_fetch. */
    if (entry == 0x40189A2Cu) {
        *global_out = 0x3FFC87ECu;
        return true;
    }
    if (entry == 0x401BDE2Cu) {
        *global_out = 0x3FFCD99Cu;
        return true;
    }
    if (entry == 0x401BE628u) {
        *global_out = 0x3FFCD9ACu;
        return true;
    }
    return false;
}

static bool fw_virtualize_phy_table(xtensa_cpu_t *cpu,
                                    esp32_rom_stubs_t *stubs) {
    uint32_t global = 0;
    if (!fw_find_phy_global(cpu, cpu->pc, &global)) {
        fprintf(stderr,
                "[flexe] virtual PHY: could not locate table global at "
                "0x%08X\n", cpu->pc);
        return false;
    }
    virtualize_phy_romfuncs(stubs);
    mem_write32(cpu->mem, global, ROM_PHY_FUNCS);
    return true;
}

static void stub_fw_virtual_phy_init(xtensa_cpu_t *cpu, void *ctx) {
    fw_virtualize_phy_table(cpu, ctx);
    rom_return_void(cpu);
}

/* Marauder's phy_get_romfunc_addr is also the earliest safe point at which
 * its virtual HCI dispatch tables can be installed. */
static void stub_fw_marauder_phy_init(xtensa_cpu_t *cpu, void *ctx) {
    esp32_rom_stubs_t *stubs = ctx;
    fw_virtualize_phy_table(cpu, stubs);
    if (cpu->pc == 0x401C1438u)
        fw_patch_handler_tables(stubs, fw_marauder_v1151_ble_tbls,
                                (int)(sizeof(fw_marauder_v1151_ble_tbls) /
                                      sizeof(fw_marauder_v1151_ble_tbls[0])),
                                FW_NOOP_FN_V1151);
    else if (cpu->pc == 0x401BE0E8u)
        fw_patch_handler_tables(stubs, fw_marauder_guition_ble_tbls,
                                (int)(sizeof(fw_marauder_guition_ble_tbls) /
                                      sizeof(fw_marauder_guition_ble_tbls[0])),
                                FW_NOOP_FN_GUITION);
    else if (cpu->pc == 0x401BE274u)
        fw_patch_handler_tables(stubs, fw_marauder_35inch_ble_tbls,
                                (int)(sizeof(fw_marauder_35inch_ble_tbls) /
                                      sizeof(fw_marauder_35inch_ble_tbls[0])),
                                FW_NOOP_FN_35INCH);
    else if (cpu->pc == 0x401BE628u)
        fw_patch_handler_tables(stubs, fw_marauder_v11423_ble_tbls,
                                (int)(sizeof(fw_marauder_v11423_ble_tbls) /
                                      sizeof(fw_marauder_v11423_ble_tbls[0])),
                                FW_NOOP_FN_V11423);
    else
        fw_patch_handler_tables(stubs, fw_marauder_ble_tbls,
                                (int)(sizeof(fw_marauder_ble_tbls) /
                                      sizeof(fw_marauder_ble_tbls[0])),
                                FW_NOOP_FN_V11401);
    rom_return_void(cpu);
}

/* The virtual HCI controller has no asynchronous transport thread to emit
 * NimBLE's initial host-sync event. Plant the event's completion flag at the
 * firmware wait-loop head, then let the original load/branch execute. */
static void stub_fw_marauder_ble_synced(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    mem_write8(cpu->mem, ar_read(cpu, 2), 1);
}

/* Marauder suspends NimBLE while switching back to WiFi-only modes. The
 * genuine deinit path first waits for the controller task to stop, but the
 * virtual HCI transport deliberately has no asynchronous controller task.
 * Keep the synchronized host instance alive and report success so a later BLE
 * mode can resume it. In particular, do not write the neighboring C++ object
 * storage: m_ignoreList starts at 0x3FFC9534 in v1.14.0/1 and 0x3FFC9544 in
 * v1.14.2/3. */
static void stub_fw_marauder_nimble_deinit(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    rom_return(cpu, 0);
}

static const fw_addr_hook_t fw_marauder_hooks[] = {
    /* DPORT cache access normally pauses the peer CPU with a level-5 IPC
     * interrupt while MMU/cache state changes.  Flexe's two cores share one
     * coherent memory map, so the pause is unnecessary; executing the real
     * handshake can deadlock when partition discovery nests a cache access
     * while the peer is already parked in esp_ipc_isr_waiting_for_finish_cmd. */
    { 0x400816FC, stub_void_unregistered,
      "esp_dport_access_stall_other_cpu_start", 0 },
    { 0x40081760, stub_void_unregistered,
      "esp_dport_access_stall_other_cpu_end", 0 },
    /* ble_hs_init is NOT stubbed: it creates ble_hs_mutex and zeroes the
     * host state; stubbing it left the mutex handle NULL and the first
     * ble_hs_lock() (NimBLEDevice::setDeviceName during init) hit
     * "assert failed: npl_freertos_mutex_pend (mu->handle)" → core dump →
     * reboot loop.  It now runs for real (the ROM fixup-table stubs below
     * cover what used to hang). */
    { 0x401BDE2C, stub_fw_marauder_phy_init, "phy_get_romfunc_addr", 0 },
    { 0x40104595, stub_fw_marauder_ble_synced, "nimble_synced_flag", 1 },
    { 0x4010463C, stub_fw_marauder_nimble_deinit,
      "NimBLEDevice::deinit", 0 },
    { 0, NULL, NULL, 0 }
};

static const fw_addr_hook_t fw_marauder_v11423_hooks[] = {
    { 0x400816FC, stub_void_unregistered,
      "esp_dport_access_stall_other_cpu_start", 0 },
    { 0x40081760, stub_void_unregistered,
      "esp_dport_access_stall_other_cpu_end", 0 },
    { 0x401BE628, stub_fw_marauder_phy_init, "phy_get_romfunc_addr", 0 },
    { 0x40104C7D, stub_fw_marauder_ble_synced, "nimble_synced_flag", 1 },
    { 0x40104D24, stub_fw_marauder_nimble_deinit,
      "NimBLEDevice::deinit", 0 },
    { 0, NULL, NULL, 0 }
};

/* Marauder v1.15.x reuses entry 0x400831D8 with a third link layout. The
 * IRAM DPORT helpers stayed put; everything in flash moved by a per-library
 * delta (NimBLE C++ +0x2C68, NimBLE host +0x2BE8, phy +0x2E10). */
static const fw_addr_hook_t fw_marauder_guition_hooks[] = {
    { 0x400816E0, stub_void_unregistered,
      "esp_dport_access_stall_other_cpu_start", 0 },
    { 0x40081744, stub_void_unregistered,
      "esp_dport_access_stall_other_cpu_end", 0 },
    { 0x401BE0E8, stub_fw_marauder_phy_init, "phy_get_romfunc_addr", 0 },
    { 0x40104B4D, stub_fw_marauder_ble_synced, "nimble_synced_flag", 1 },
    { 0x40104BF4, stub_fw_marauder_nimble_deinit,
      "NimBLEDevice::deinit", 0 },
    { 0, NULL, NULL, 0 }
};

static const fw_addr_hook_t fw_marauder_35inch_hooks[] = {
    { 0x400816E0, stub_void_unregistered,
      "esp_dport_access_stall_other_cpu_start", 0 },
    { 0x40081744, stub_void_unregistered,
      "esp_dport_access_stall_other_cpu_end", 0 },
    { 0x401BE274, stub_fw_marauder_phy_init, "phy_get_romfunc_addr", 0 },
    { 0x40104D4D, stub_fw_marauder_ble_synced, "nimble_synced_flag", 1 },
    { 0x40104DF4, stub_fw_marauder_nimble_deinit,
      "NimBLEDevice::deinit", 0 },
    { 0, NULL, NULL, 0 }
};

static const fw_addr_hook_t fw_marauder_v1151_hooks[] = {
    { 0x400816FC, stub_void_unregistered,
      "esp_dport_access_stall_other_cpu_start", 0 },
    { 0x40081760, stub_void_unregistered,
      "esp_dport_access_stall_other_cpu_end", 0 },
    { 0x401C1438, stub_fw_marauder_phy_init, "phy_get_romfunc_addr", 0 },
    { 0x401078E5, stub_fw_marauder_ble_synced, "nimble_synced_flag", 1 },
    { 0x4010798C, stub_fw_marauder_nimble_deinit,
      "NimBLEDevice::deinit", 0 },
    { 0, NULL, NULL, 0 }
};

/* NerdMiner v1.8.3 (CYD 2432S028R, entry 0x40089268). */
static const fw_addr_hook_t fw_nerdminer_hooks[] = {
    /* Keep libphy's ABI but bypass physical RF calibration.  Higher-level
     * WiFi/socket hooks model the observable network behavior. */
    { 0x40189A2C, stub_fw_virtual_phy_init, "phy_get_romfunc_addr", 0 },
    /* event_group_wait_bits_wrapper: the Arduino WiFi
     * wait-for-connect path calls this (→ xEventGroupWaitBits) to wait for
     * the WiFi-connected event-group bit, which is never set with no radio.
     * Return the requested bits (arg1) so the caller sees "connected". */
    { 0x4011DAB4, stub_ret_arg1, "wifi_egwait", 0 },
    /* WiFiSTAClass::status() gates the mining transition. */
    { 0x400EECB0, stub_ret_wl_connected, "wifi_status", 0 },
    /* The virtual MAC has no asynchronous connect-complete producer, so the
     * driver's otherwise unbounded command wait completes synchronously. */
    { 0x4010A9F4, stub_unregistered, "esp_wifi_connect_exec", 0 },
    { 0, NULL, NULL, 0 }
};

int rom_stubs_hook_firmware_addrs(esp32_rom_stubs_t *stubs, uint32_t entry_point) {
    const fw_addr_hook_t *tbl = NULL;
    rom_firmware_profile_t profile = rom_stubs_identify_firmware(
            stubs, entry_point);
    if (profile == ROM_FIRMWARE_MARAUDER_V1140_1)
        tbl = fw_marauder_hooks;
    else if (profile == ROM_FIRMWARE_MARAUDER_V1142_3)
        tbl = fw_marauder_v11423_hooks;
    else if (profile == ROM_FIRMWARE_MARAUDER_V1151)
        tbl = fw_marauder_v1151_hooks;
    else if (profile == ROM_FIRMWARE_MARAUDER_V1143_GUITION)
        tbl = fw_marauder_guition_hooks;
    else if (profile == ROM_FIRMWARE_MARAUDER_V1143_35INCH)
        tbl = fw_marauder_35inch_hooks;
    else if (profile == ROM_FIRMWARE_NERDMINER_V183)
        tbl = fw_nerdminer_hooks;
    if (!tbl) {
        if (entry_point == 0x400831D8u)
            fprintf(stderr,
                    "[flexe] unsupported Marauder image signature at "
                    "entry 0x%08X; refusing address-based hooks\n",
                    entry_point);
        return 0;
    }
    int n = 0;
    for (const fw_addr_hook_t *h = tbl; h->fn; h++) {
        if (h->spy)
            rom_stubs_register_spy(stubs, h->addr, h->fn, h->name, NULL);
        else
            rom_stubs_register(stubs, h->addr, h->fn, h->name);
        n++;
    }
    if (n)
        fprintf(stderr, "[flexe] hooked %d firmware driver stub(s) at entry 0x%08X\n",
                n, entry_point);
    return n;
}

/* Hook firmware functions by symbol name from ELF.
 * This allows stubbing functions that live in the loaded firmware
 * (not ROM) — needed for things like newlib lock functions. */
int rom_stubs_hook_symbols(esp32_rom_stubs_t *stubs,
                           const elf_symbols_t *syms) {
    if (!stubs || !syms) return 0;
    stubs->syms = syms;
    int hooked = 0;

    /* The closed-source PHY library's calibration routines require analog
     * hardware.  Preserve its table contract and virtualize that boundary
     * for any ELF which exposes the standard wrapper symbol. */
    uint32_t phy_addr;
    if (elf_symbols_find(syms, "phy_get_romfunc_addr", &phy_addr) == 0) {
        rom_stubs_register_ctx(stubs, phy_addr, stub_fw_virtual_phy_init,
                               "phy_get_romfunc_addr", stubs);
        hooked++;
    }

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
        { "nvs_get_i8",                 stub_nvs_get_i32 },
        { "nvs_get_u8",                 stub_nvs_get_u32 },
        { "nvs_get_i16",                stub_nvs_get_i32 },
        { "nvs_get_u16",                stub_nvs_get_u32 },
        { "nvs_get_i32",                stub_nvs_get_i32 },
        { "nvs_get_u32",                stub_nvs_get_u32 },
        { "nvs_get_i64",                stub_nvs_get_notfound },
        { "nvs_get_u64",                stub_nvs_get_notfound },
        { "nvs_get_str",                stub_nvs_get_str },
        { "nvs_get_blob",               stub_nvs_get_blob },
        { "nvs_set_i8",                 stub_nvs_set_i32 },
        { "nvs_set_u8",                 stub_nvs_set_u32 },
        { "nvs_set_i16",                stub_nvs_set_i32 },
        { "nvs_set_u16",                stub_nvs_set_u32 },
        { "nvs_set_i32",                stub_nvs_set_i32 },
        { "nvs_set_u32",                stub_nvs_set_u32 },
        { "nvs_set_i64",                stub_nvs_set_ok },
        { "nvs_set_u64",                stub_nvs_set_ok },
        { "nvs_set_str",                stub_nvs_set_str },
        { "nvs_set_blob",               stub_nvs_set_blob },
        { "nvs_commit",                 stub_nvs_set_ok },
        /* C++ wrappers used by nvs_handle_cxx.cpp / nvs_rw_value_cxx */
        { "_ZN3nvs15open_nvs_handleEPKc15nvs_open_mode_tPi",
          stub_cxx_open_nvs_handle },
        { "_ZN3nvs30open_nvs_handle_from_partitionEPKcS1_15nvs_open_mode_tPi",
          stub_cxx_open_nvs_handle_from_partition },
        { "_ZN3nvs9NVSHandle8get_itemIlEEiPKcRT_",
          stub_cxx_nvshandle_get_item_i32 },
        { "_ZN3nvs9NVSHandle8set_itemIlEEiPKcT_",
          stub_cxx_nvshandle_set_item_i32 },
        { "_ZN3nvs9NVSHandle8get_itemIiEEiPKcRT_",
          stub_cxx_nvshandle_get_item_i32 },
        { "_ZN3nvs9NVSHandle8set_itemIiEEiPKcT_",
          stub_cxx_nvshandle_set_item_i32 },
        /* Real NVSHandleSimple::commit — reached via vtable dispatch from
         * NVSHandle::commit(). The vtable in flash is unmodified, so
         * `[this+0][40] -> callx8` lands on this real symbol address; we
         * intercept it and short-circuit to ESP_OK. */
        { "_ZN3nvs15NVSHandleSimple6commitEv",  stub_cxx_nvshandle_commit },
        /* unique_ptr<NVSHandle> destruction routes through these destructors;
         * they would otherwise touch un-initialised C++ state in our fake
         * object. Treat them as no-ops. */
        { "_ZN3nvs15NVSHandleSimpleD0Ev",       stub_cxx_nvshandle_commit },
        { "_ZN3nvs15NVSHandleSimpleD1Ev",       stub_cxx_nvshandle_commit },
        { "_ZN3nvs15NVSHandleSimpleD2Ev",       stub_cxx_nvshandle_commit },
        { "_ZN3nvs19NVSPartitionManager12close_handleEPNS_15NVSHandleSimpleE",
          stub_cxx_nvshandle_commit },
        { NULL, NULL }
    };
    for (int i = 0; nvs_hooks[i].name; i++) {
        uint32_t addr;
        if (elf_symbols_find(syms, nvs_hooks[i].name, &addr) == 0) {
            rom_stubs_register_ctx(stubs, addr, nvs_hooks[i].fn,
                                   nvs_hooks[i].name, stubs);
            hooked++;
        }
    }

    /* GPIO driver stubs */
    struct { const char *name; rom_stub_fn fn; } gpio_hooks[] = {
        { "esp_ipc_call_blocking",      stub_esp_ipc_call_blocking },
        { "esp_ipc_call",               stub_esp_ipc_call_blocking },
        { "gpio_install_isr_service",   stub_gpio_install_isr_service },
        { "gpio_set_intr_type",         stub_gpio_set_intr_type },
        { "gpio_intr_enable",           stub_gpio_intr_enable },
        { "gpio_intr_disable",          stub_gpio_intr_disable },
        { "gpio_isr_handler_add",       stub_gpio_isr_handler_add },
        { "gpio_isr_handler_remove",    stub_gpio_isr_handler_remove },
        { "gpio_config",                stub_gpio_config },
        { "gpio_set_direction",         stub_gpio_set_direction },
        { "gpio_set_level",             stub_gpio_set_level },
        { "gpio_get_level",             stub_gpio_get_level },
        { "gpio_reset_pin",             stub_gpio_reset_pin },
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

    /* Legacy ADC stubs need context (periph) to read the sandbox-injected
     * analog values, so use register_ctx with `stubs`. The modern oneshot
     * driver runs through the SENS MMIO model directly. */
    struct { const char *name; rom_stub_fn fn; } adc_hooks[] = {
        { "adc1_get_raw",       stub_adc1_get_raw },
        { "adc2_get_raw",       stub_adc2_get_raw },
        { NULL, NULL }
    };
    for (int i = 0; adc_hooks[i].name; i++) {
        uint32_t addr;
        if (elf_symbols_find(syms, adc_hooks[i].name, &addr) == 0) {
            rom_stubs_register_ctx(stubs, addr, adc_hooks[i].fn,
                                   adc_hooks[i].name, stubs);
            hooked++;
        }
    }

    /* Heap allocator stubs */
    struct { const char *name; rom_stub_fn fn; } alloc_hooks[] = {
        { "malloc",                         stub_malloc },
        { "calloc",                         stub_calloc },
        { "free",                           stub_free },
        { "realloc",                        stub_realloc },
        { "heap_caps_malloc",               stub_heap_caps_malloc },
        { "heap_caps_malloc_default",       stub_malloc },
        { "heap_caps_free",                 stub_free },
        { "heap_caps_realloc",              stub_heap_caps_realloc },
        { "heap_caps_realloc_default",      stub_realloc },
        { "heap_caps_calloc",               stub_heap_caps_calloc },
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

    /* SPI flash op-lock stubs — no-op the operation locking (single-emulated
     * CPU; the real lock/unlock is unnecessary and can wedge on it).  The
     * flash INIT path (esp_flash_init_main, esp_flash_init_default_chip,
     * spi_flash_init_chip_state, esp_flash_read_chip_id) is deliberately NOT
     * stubbed: the emulated SPI flash controller answers RDID with a valid
     * JEDEC ID (EMU_FLASH_JEDEC_ID, GD25Q32/4MB), and the guest's spi_flash
     * size probe needs the real init + RDID to detect the flash size.
     * Stubbing the init path made the probe report "Detected size(0k)" and
     * abort the boot. */
    /* Also no-op the cross-core cache-disable handshake
     * (spi_flash_disable_interrupts_caches_and_other_cpu and its enable
     * counterpart): it raises a high-priority interrupt on the other core and
     * spins waiting for it to pause, which deadlocks when the other core is
     * still in its startup loop.  In the emulator flash ops are memory-mapped,
     * so pausing the other core is unnecessary. */
    static const char *flash_init_fns[] = {
        "spi_flash_op_lock",
        "spi_flash_op_unlock",
        "spi_flash_op_block_func",
        "spi_flash_disable_interrupts_caches_and_other_cpu",
        "spi_flash_enable_interrupts_caches_and_other_cpu",
        NULL
    };
    for (int i = 0; flash_init_fns[i]; i++) {
        uint32_t addr;
        if (elf_symbols_find(syms, flash_init_fns[i], &addr) == 0) {
            rom_stubs_register(stubs, addr, stub_unregistered, flash_init_fns[i]);
            hooked++;
        }
    }

    /* ESP-IDF SPI + LCD panel stubs (no-ops, display handled at higher level).
     * NOTE: spi_bus_initialize is deliberately NOT stubbed — the real driver
     * must run so GP-SPI transactions reach the raw SPI2/SPI3 sniffer
     * (spi_display.c), which is what renders symbol-less firmware. */
    static const char *lcd_noop_fns[] = {
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
        static const struct {
            const char *name;
            rom_stub_fn fn;
        } intr_fns[] = {
            { "esp_intr_alloc", stub_esp_intr_alloc },
            { "esp_intr_alloc_intrstatus", stub_esp_intr_alloc_intrstatus },
            { "esp_intr_free", stub_esp_intr_free },
            { "esp_intr_disable", stub_esp_intr_disable },
            { "esp_intr_enable", stub_esp_intr_enable },
            { NULL, NULL }
        };
        for (int i = 0; intr_fns[i].name; i++) {
            uint32_t addr;
            if (elf_symbols_find(syms, intr_fns[i].name, &addr) == 0) {
                rom_stubs_register(stubs, addr, intr_fns[i].fn,
                                   intr_fns[i].name);
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
        if (elf_symbols_find(syms, "esp_flash_get_size", &addr) == 0) {
            rom_stubs_register(stubs, addr, stub_esp_flash_get_size, "esp_flash_get_size");
            hooked++;
        }
    }

    /* esp_lcd panel driver: fake the panel handle so ILI9341 / i80 / RGB
     * firmware can proceed through init() / reset() / disp_on_off() and
     * reach draw_bitmap(), where we capture pixel data for the sandbox. */
    {
        uint32_t addr;
        const struct { const char *name; rom_stub_fn fn; } lcd_hooks[] = {
            { "esp_lcd_new_panel_ili9341", stub_esp_lcd_new_panel_ili9341 },
            { "esp_lcd_new_panel_ssd1306", stub_esp_lcd_new_panel_ssd1306 },
            { "esp_lcd_panel_reset",       stub_esp_lcd_panel_noop_ok },
            { "esp_lcd_panel_init",        stub_esp_lcd_panel_noop_ok },
            { "esp_lcd_panel_disp_on_off", stub_esp_lcd_panel_noop_ok },
            { "esp_lcd_panel_mirror",      stub_esp_lcd_panel_noop_ok },
            { "esp_lcd_panel_swap_xy",     stub_esp_lcd_panel_noop_ok },
            { "esp_lcd_panel_set_gap",     stub_esp_lcd_panel_noop_ok },
            { "esp_lcd_panel_invert_color",stub_esp_lcd_panel_noop_ok },
            { "esp_lcd_panel_draw_bitmap", stub_esp_lcd_panel_draw_bitmap },
        };
        for (size_t i = 0; i < sizeof(lcd_hooks)/sizeof(lcd_hooks[0]); i++) {
            if (elf_symbols_find(syms, lcd_hooks[i].name, &addr) == 0) {
                rom_stubs_register(stubs, addr, lcd_hooks[i].fn, lcd_hooks[i].name);
                hooked++;
            }
        }
    }

    /* Newlib stdio: printf/puts/etc. In the emulator stdout's FILE* is
     * fully-buffered by newlib and never flushes (the _write path through
     * esp_vfs_write returns -1 for fd=1). Intercept these at symbol level
     * and route the format engine directly to UART, same as ets_printf. */
    {
        uint32_t addr;
        const struct { const char *name; rom_stub_fn fn; } printf_hooks[] = {
            { "printf",    stub_newlib_printf },
            { "_printf_r", stub_newlib_printf },
            { "iprintf",   stub_newlib_printf },
            { "puts",      stub_newlib_puts },
            { "_puts_r",   stub_newlib_puts },
            { "putchar",   stub_newlib_putchar },
            { "putchar_unlocked", stub_newlib_putchar },
            { "fputs",     stub_newlib_fputs },
            { "fputs_unlocked", stub_newlib_fputs },
        };
        for (size_t i = 0; i < sizeof(printf_hooks)/sizeof(printf_hooks[0]); i++) {
            if (elf_symbols_find(syms, printf_hooks[i].name, &addr) == 0) {
                rom_stubs_register(stubs, addr, printf_hooks[i].fn, printf_hooks[i].name);
                hooked++;
            }
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
    static const struct {
        const char *name;
        rom_stub_fn fn;
    } arduino_gpio_fns[] = {
        { "__pinMode",      stub_arduino_pin_mode },
        { "pinMode",        stub_arduino_pin_mode },
        { "__digitalWrite", stub_arduino_digital_write },
        { "digitalWrite",   stub_arduino_digital_write },
        { NULL, NULL },
    };
    for (int i = 0; arduino_gpio_fns[i].name; i++) {
        uint32_t addr;
        if (elf_symbols_find(syms, arduino_gpio_fns[i].name, &addr) == 0) {
            rom_stubs_register(stubs, addr, arduino_gpio_fns[i].fn,
                               arduino_gpio_fns[i].name);
            hooked++;
        }
    }
    static const char *arduino_pwm_fns[] = {
        "analogWrite",
        "ledcSetup",
        "ledcAttachPin",
        "ledcWrite",
        NULL
    };
    for (int i = 0; arduino_pwm_fns[i]; i++) {
        uint32_t addr;
        if (elf_symbols_find(syms, arduino_pwm_fns[i], &addr) == 0) {
            rom_stubs_register(stubs, addr, stub_unregistered,
                               arduino_pwm_fns[i]);
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
    int first = stubs->count;
    int rc = rom_stubs_register_ctx(stubs, addr, fn, name, user_ctx);
    if (rc == 0) {
        /* Mark every entry this call added (register_ctx may append a second
         * hook at a scanned ENTRY address), and refresh any direct-dispatch
         * slot: hook_ht_insert copies e->spy before the flag is set here, so
         * without this the fast path would consume the hooked instruction
         * instead of letting it execute. */
        for (int i = first; i < stubs->count; i++) {
            stubs->entries[i].spy = 1;
            if (stubs->direct) {
                uint32_t di = (stubs->entries[i].addr >> 2) & STUB_DIRECT_MASK;
                if (stubs->direct[di].tag == stubs->entries[i].addr)
                    stubs->direct[di].spy = 1;
            }
        }
    }
    return rc;
}

static int rom_stubs_register_any(esp32_rom_stubs_t *stubs, uint32_t addr,
                                  rom_stub_fn fn,
                                  rom_conditional_stub_fn conditional_fn,
                                  const char *name, void *user_ctx) {
    if (stubs->count >= MAX_ROM_STUBS) return -1;
    stubs->entries[stubs->count].addr = addr;
    stubs->entries[stubs->count].fn = fn;
    stubs->entries[stubs->count].conditional_fn = conditional_fn;
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
                    stubs->entries[stubs->count].conditional_fn =
                            conditional_fn;
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

int rom_stubs_register_ctx(esp32_rom_stubs_t *stubs, uint32_t addr,
                            rom_stub_fn fn, const char *name, void *user_ctx) {
    return rom_stubs_register_any(stubs, addr, fn, NULL, name, user_ctx);
}

int rom_stubs_register_conditional_ctx(
                            esp32_rom_stubs_t *stubs, uint32_t addr,
                            rom_conditional_stub_fn fn, const char *name,
                            void *user_ctx) {
    if (!fn) return -1;
    return rom_stubs_register_any(stubs, addr, NULL, fn, name, user_ctx);
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
