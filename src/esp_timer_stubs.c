#include "esp_timer_stubs.h"
#include "rom_stubs.h"
#include "memory.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#define ESP_OK    0
#define MAX_TIMERS 16

/* Sentinel address for callback return interception */
#define CALLBACK_SENTINEL 0x40001FFCu

typedef struct {
    int      active;
    int      periodic;
    uint32_t callback_addr;
    uint32_t arg;
    uint64_t period_us;
    uint64_t alarm_us;     /* absolute time in microseconds */
    uint32_t handle;       /* address returned as handle */
} emu_timer_t;

struct esp_timer_stubs {
    xtensa_cpu_t      *cpu;
    esp32_rom_stubs_t *rom;
    uint32_t           cpu_freq_mhz;

    emu_timer_t timers[MAX_TIMERS];
    int         timer_count;

    /* Bump allocator for timer handles */
    uint32_t bump_ptr;
};

#define TIMER_BUMP_BASE  0x3FFE8000u
#define TIMER_BUMP_LIMIT 0x3FFE9000u

/* ===== Calling convention helpers ===== */

static uint32_t et_arg(xtensa_cpu_t *cpu, int n) {
    int ci = XT_PS_CALLINC(cpu->ps);
    return ar_read(cpu, ci * 4 + 2 + n);
}

static void et_return(xtensa_cpu_t *cpu, uint32_t retval) {
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

static void et_return_void(xtensa_cpu_t *cpu) {
    int ci = XT_PS_CALLINC(cpu->ps);
    if (ci > 0) {
        uint32_t a0 = ar_read(cpu, ci * 4);
        cpu->pc = (cpu->pc & 0xC0000000u) | (a0 & 0x3FFFFFFFu);
        XT_PS_SET_CALLINC(cpu->ps, 0);
    } else {
        cpu->pc = ar_read(cpu, 0);
    }
}

/* ===== Helper: get current time in microseconds ===== */

static uint64_t current_time_us(esp_timer_stubs_t *et) {
    /* Virtual wall-clock = accumulated sleep time + instruction-driven time.
     * ccount only advances per-instruction (stays small), so inlined
     * 64-bit divisions complete in reasonable iterations. */
    return et->cpu->virtual_time_us + (uint64_t)et->cpu->ccount / et->cpu_freq_mhz;
}

/* ===== Find timer by handle ===== */

static emu_timer_t *find_timer(esp_timer_stubs_t *et, uint32_t handle) {
    for (int i = 0; i < et->timer_count; i++)
        if (et->timers[i].handle == handle)
            return &et->timers[i];
    return NULL;
}

/* ===== Dispatch expired timer callbacks ===== */

static void dispatch_expired_timers(esp_timer_stubs_t *et) {
    uint64_t now = current_time_us(et);

    for (int i = 0; i < et->timer_count; i++) {
        emu_timer_t *t = &et->timers[i];
        if (!t->active) continue;
        if (now < t->alarm_us) continue;

        /* Timer expired — dispatch callback inline.
         * The callback may be a windowed function (ENTRY/RETW/CALL8 etc.)
         * that modifies physical registers across multiple windows. We must
         * save and restore the ENTIRE register file + window state to prevent
         * any side effects on the caller's register context. */
        uint32_t save_pc = et->cpu->pc;
        uint32_t save_ps = et->cpu->ps;
        uint32_t save_wb = et->cpu->windowbase;
        uint32_t save_ws = et->cpu->windowstart;
        uint32_t save_sar = et->cpu->sar;
        uint32_t save_lbeg = et->cpu->lbeg;
        uint32_t save_lend = et->cpu->lend;
        uint32_t save_lcount = et->cpu->lcount;
        uint32_t save_ar[64];
        memcpy(save_ar, et->cpu->ar, sizeof(save_ar));

        /* Set up as CALL4: CALLINC=1, callee's a0 (at ar[4]) = sentinel
         * with bits 31:30 = 01 (CALL4 return encoding), callee's a2 = arg */
        XT_PS_SET_CALLINC(et->cpu->ps, 1);
        ar_write(et->cpu, 4, CALLBACK_SENTINEL); /* bits 31:30 = 01, matches CALL4 */
        ar_write(et->cpu, 6, t->arg);

        et->cpu->pc = t->callback_addr;

        /* Run callback: execute up to 100000 instructions or until sentinel hit */
        int max_cb_cycles = 100000;
        for (int c = 0; c < max_cb_cycles; c++) {
            if (et->cpu->pc == CALLBACK_SENTINEL) break;
            if (!et->cpu->running) break;
            xtensa_step(et->cpu);
        }

        /* Restore entire CPU register state */
        memcpy(et->cpu->ar, save_ar, sizeof(save_ar));
        et->cpu->pc = save_pc;
        et->cpu->ps = save_ps;
        et->cpu->windowbase = save_wb;
        et->cpu->windowstart = save_ws;
        et->cpu->sar = save_sar;
        et->cpu->lbeg = save_lbeg;
        et->cpu->lend = save_lend;
        et->cpu->lcount = save_lcount;

        /* Reschedule periodic or deactivate */
        if (t->periodic) {
            t->alarm_us += t->period_us;
        } else {
            t->active = 0;
        }
    }
}

/* ===== esp_timer stub implementations ===== */

/*
 * esp_timer_create(const esp_timer_create_args_t *args, esp_timer_handle_t *out)
 * args layout: { callback (4 bytes), arg (4 bytes), dispatch_method (4), name (4), ... }
 */
void stub_esp_timer_create(xtensa_cpu_t *cpu, void *ctx) {
    esp_timer_stubs_t *et = ctx;
    uint32_t args_ptr = et_arg(cpu, 0);
    uint32_t out_ptr  = et_arg(cpu, 1);

    if (et->timer_count >= MAX_TIMERS) {
        et_return(cpu, -1);
        return;
    }

    uint32_t callback = mem_read32(cpu->mem, args_ptr);
    uint32_t arg      = mem_read32(cpu->mem, args_ptr + 4);

    emu_timer_t *t = &et->timers[et->timer_count];
    uint32_t handle = et->bump_ptr;
    et->bump_ptr += 4;

    t->handle = handle;
    t->callback_addr = callback;
    t->arg = arg;
    t->active = 0;
    t->periodic = 0;
    t->period_us = 0;
    t->alarm_us = 0;
    et->timer_count++;

    if (out_ptr)
        mem_write32(cpu->mem, out_ptr, handle);

    et_return(cpu, ESP_OK);
}

/* esp_timer_start_periodic(handle, period_us) */
void stub_esp_timer_start_periodic(xtensa_cpu_t *cpu, void *ctx) {
    esp_timer_stubs_t *et = ctx;
    uint32_t handle = et_arg(cpu, 0);
    uint32_t period = et_arg(cpu, 1);

    emu_timer_t *t = find_timer(et, handle);
    if (!t) { et_return(cpu, -1); return; }

    t->active = 1;
    t->periodic = 1;
    t->period_us = period;
    t->alarm_us = current_time_us(et) + period;

    et_return(cpu, ESP_OK);
}

/* esp_timer_start_once(handle, timeout_us) */
void stub_esp_timer_start_once(xtensa_cpu_t *cpu, void *ctx) {
    esp_timer_stubs_t *et = ctx;
    uint32_t handle  = et_arg(cpu, 0);
    uint32_t timeout = et_arg(cpu, 1);

    emu_timer_t *t = find_timer(et, handle);
    if (!t) { et_return(cpu, -1); return; }

    t->active = 1;
    t->periodic = 0;
    t->period_us = 0;
    t->alarm_us = current_time_us(et) + timeout;

    et_return(cpu, ESP_OK);
}

/* esp_timer_stop(handle) */
void stub_esp_timer_stop(xtensa_cpu_t *cpu, void *ctx) {
    esp_timer_stubs_t *et = ctx;
    uint32_t handle = et_arg(cpu, 0);
    emu_timer_t *t = find_timer(et, handle);
    if (t) t->active = 0;
    et_return(cpu, ESP_OK);
}

/* esp_timer_delete(handle) */
void stub_esp_timer_delete(xtensa_cpu_t *cpu, void *ctx) {
    esp_timer_stubs_t *et = ctx;
    uint32_t handle = et_arg(cpu, 0);
    emu_timer_t *t = find_timer(et, handle);
    if (t) {
        t->active = 0;
        t->handle = 0;
    }
    et_return(cpu, ESP_OK);
}

/* esp_timer_get_time() -> int64_t microseconds (returned in a2:a3) */
void stub_esp_timer_get_time(xtensa_cpu_t *cpu, void *ctx) {
    esp_timer_stubs_t *et = ctx;
    uint64_t us = current_time_us(et);
    /* Return 64-bit value: low in a2, high in a3 */
    int ci = XT_PS_CALLINC(cpu->ps);
    if (ci > 0) {
        ar_write(cpu, ci * 4 + 2, (uint32_t)us);
        ar_write(cpu, ci * 4 + 3, (uint32_t)(us >> 32));
        uint32_t a0 = ar_read(cpu, ci * 4);
        cpu->pc = (cpu->pc & 0xC0000000u) | (a0 & 0x3FFFFFFFu);
        XT_PS_SET_CALLINC(cpu->ps, 0);
    } else {
        ar_write(cpu, 2, (uint32_t)us);
        ar_write(cpu, 3, (uint32_t)(us >> 32));
        cpu->pc = ar_read(cpu, 0);
    }
}

/* esp_timer_dump(FILE *stream) — print timer list to stdout */
void stub_esp_timer_dump(xtensa_cpu_t *cpu, void *ctx) {
    esp_timer_stubs_t *et = ctx;
    /* Print timer info to emulator console (not to UART in emulator memory) */
    for (int i = 0; i < et->timer_count; i++) {
        emu_timer_t *t = &et->timers[i];
        if (t->handle) {
            fprintf(stdout, "Timer %d: handle=0x%08X cb=0x%08X %s period=%llu alarm=%llu\n",
                    i, t->handle, t->callback_addr,
                    t->active ? "ACTIVE" : "inactive",
                    (unsigned long long)t->period_us,
                    (unsigned long long)t->alarm_us);
        }
    }
    et_return_void(cpu);
}

/* esp_timer_is_active(handle) -> bool */
void stub_esp_timer_is_active(xtensa_cpu_t *cpu, void *ctx) {
    esp_timer_stubs_t *et = ctx;
    uint32_t handle = et_arg(cpu, 0);
    emu_timer_t *t = find_timer(et, handle);
    et_return(cpu, (t && t->active) ? 1 : 0);
}

/* usleep(us) — advance virtual time, check/dispatch expired timers */
void stub_usleep(xtensa_cpu_t *cpu, void *ctx) {
    esp_timer_stubs_t *et = ctx;
    uint32_t us = et_arg(cpu, 0);
    cpu->virtual_time_us += us;
    dispatch_expired_timers(et);
    et_return(cpu, 0);
}

/* esp_timer_init() — no-op, return ESP_OK */
void stub_esp_timer_init(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    et_return(cpu, ESP_OK);
}

/* millis() — return esp_timer_get_time() / 1000.
 * The firmware's millis() uses an inlined 64-bit software division that
 * takes millions of iterations when ccount is large. Stub it out. */
void stub_millis(xtensa_cpu_t *cpu, void *ctx) {
    esp_timer_stubs_t *et = ctx;
    uint64_t us = current_time_us(et);
    uint32_t ms = (uint32_t)(us / 1000);
    et_return(cpu, ms);
}

/* micros() — return esp_timer_get_time() as uint32_t */
void stub_micros(xtensa_cpu_t *cpu, void *ctx) {
    esp_timer_stubs_t *et = ctx;
    uint32_t us = (uint32_t)current_time_us(et);
    et_return(cpu, us);
}

/* delay(ms) — advance virtual time, dispatch timers */
void stub_delay(xtensa_cpu_t *cpu, void *ctx) {
    esp_timer_stubs_t *et = ctx;
    uint32_t ms = et_arg(cpu, 0);
    cpu->virtual_time_us += (uint64_t)ms * 1000;
    dispatch_expired_timers(et);
    et_return_void(cpu);
}

/* ===== Public API ===== */

esp_timer_stubs_t *esp_timer_stubs_create(xtensa_cpu_t *cpu) {
    esp_timer_stubs_t *et = calloc(1, sizeof(*et));
    if (!et) return NULL;
    et->cpu = cpu;
    et->cpu_freq_mhz = 160;
    et->bump_ptr = TIMER_BUMP_BASE;
    return et;
}

void esp_timer_stubs_destroy(esp_timer_stubs_t *et) {
    free(et);
}

int esp_timer_stubs_hook_symbols(esp_timer_stubs_t *et, const elf_symbols_t *syms) {
    if (!et || !syms) return 0;

    esp32_rom_stubs_t *rom = et->cpu->pc_hook_ctx;
    if (!rom) return 0;
    et->rom = rom;

    int hooked = 0;

    struct {
        const char *name;
        rom_stub_fn fn;
    } hooks[] = {
        { "esp_timer_create",          stub_esp_timer_create },
        { "esp_timer_start_periodic",  stub_esp_timer_start_periodic },
        { "esp_timer_start_once",      stub_esp_timer_start_once },
        { "esp_timer_stop",            stub_esp_timer_stop },
        { "esp_timer_delete",          stub_esp_timer_delete },
        { "esp_timer_get_time",        stub_esp_timer_get_time },
        { "esp_timer_impl_get_time",   stub_esp_timer_get_time },
        { "esp_timer_dump",            stub_esp_timer_dump },
        { "esp_timer_is_active",       stub_esp_timer_is_active },
        { "esp_timer_init",            stub_esp_timer_init },
        { "usleep",                    stub_usleep },
        { "millis",                    stub_millis },
        { "micros",                    stub_micros },
        { "delay",                     stub_delay },
        { NULL, NULL }
    };

    for (int i = 0; hooks[i].name; i++) {
        uint32_t addr;
        if (elf_symbols_find(syms, hooks[i].name, &addr) == 0) {
            rom_stubs_register_ctx(rom, addr, hooks[i].fn, hooks[i].name, et);
            hooked++;
        }
    }

    return hooked;
}

int esp_timer_stubs_timer_count(const esp_timer_stubs_t *et) {
    return et ? et->timer_count : 0;
}
