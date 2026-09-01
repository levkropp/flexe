#ifdef _MSC_VER
#include "msvc_compat.h"
#endif

#include "esp_timer_stubs.h"
#include "rom_stubs.h"
#include "memory.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <time.h>

#define ESP_OK    0
#define MAX_TIMERS 16
#define ESP32_CPU_TICKS_PER_US_ADDR 0x3FFE01E0u

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

    /* Host wall-clock boot time, used only when virtual time is disabled. */
    struct timespec boot_time;

    /* Bump allocator for timer handles */
    uint32_t bump_ptr;

    /* If true (the default), millis/micros/esp_timer_get_time derive from
     * emulated time rather than the host wall clock. Guest-visible clocks
     * must advance with emulated cycles: a firmware that busy-waits on
     * millis() otherwise burns a real wall-clock interval — a different
     * number of guest instructions on every run and on every host — which
     * makes execution non-deterministic and decouples the guest's sense of
     * time from CCOUNT and the peripherals. */
    int use_virtual_time;

    /* Optional blocking-wait delegate (the FreeRTOS scheduler). */
    esp_timer_sleep_fn sleep_fn;
    void              *sleep_ctx;
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
        cpu->pc = (cpu->pc & 0xC0000000u) | (ar_read(cpu, 0) & 0x3FFFFFFFu);
    }
}

static void et_return_void(xtensa_cpu_t *cpu) {
    int ci = XT_PS_CALLINC(cpu->ps);
    if (ci > 0) {
        uint32_t a0 = ar_read(cpu, ci * 4);
        cpu->pc = (cpu->pc & 0xC0000000u) | (a0 & 0x3FFFFFFFu);
        XT_PS_SET_CALLINC(cpu->ps, 0);
    } else {
        cpu->pc = (cpu->pc & 0xC0000000u) | (ar_read(cpu, 0) & 0x3FFFFFFFu);
    }
}

/* ===== Helper: get current time in microseconds ===== */

/* Host wall-clock elapsed time since boot.  Used by millis()/micros()/
 * esp_timer_get_time() so firmware elapsed-time displays track real time
 * instead of fast-forwarded virtual time from vTaskDelay. */
static uint64_t host_elapsed_us(const esp_timer_stubs_t *et) {
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    long sec  = now.tv_sec  - et->boot_time.tv_sec;
    long nsec = now.tv_nsec - et->boot_time.tv_nsec;
    if (nsec < 0) { sec--; nsec += 1000000000L; }
    return (uint64_t)sec * 1000000ULL + (uint64_t)nsec / 1000ULL;
}

/* Monotonic clock used for scheduling esp_timer alarms; see
 * xtensa_guest_time_us() for why executed and skipped time are summed. */
static uint32_t current_cpu_freq_mhz(esp_timer_stubs_t *et) {
    uint32_t mhz = mem_read32(et->cpu->mem, ESP32_CPU_TICKS_PER_US_ADDR);
    if (mhz >= 10u && mhz <= 240u)
        et->cpu_freq_mhz = mhz;
    return et->cpu_freq_mhz;
}

static uint64_t current_time_us(esp_timer_stubs_t *et) {
    return xtensa_guest_time_us(et->cpu, current_cpu_freq_mhz(et));
}

/* Fast-forward past a wait when no scheduler is available to block on.
 * Guest-visible time (ccount, virtual_time_us) advances; cycle_count does
 * not, because it counts *executed* guest instructions and drives the -c
 * budget, and a fast-forward executes none.
 *
 * NOTE: this jumps straight to the end of the wait, so a peripheral or
 * ccompare event scheduled inside the window is collapsed into a single
 * observation instead of firing at its own ccount. Waits that go through the
 * FreeRTOS scheduler (vTaskDelay) do advance event-by-event; Arduino's
 * delay() is intercepted ahead of that call and does not. */
static void advance_wait_time(esp_timer_stubs_t *et, xtensa_cpu_t *cpu,
                              uint64_t us) {
    uint64_t cycles = us * current_cpu_freq_mhz(et);
    cpu->virtual_time_us += us;
    cpu->ccount += (uint32_t)cycles;
}


/* Block for `us` microseconds, preferring the scheduler so peer tasks, ISRs
 * and peripheral events still run during the wait. */
static bool et_block_us(esp_timer_stubs_t *et, xtensa_cpu_t *cpu, uint64_t us) {
    if (et->sleep_fn && et->sleep_fn(et->sleep_ctx, cpu, us))
        return true;
    advance_wait_time(et, cpu, us);
    return false;
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

        /* Run callback: execute up to 100000 instructions or until sentinel hit.
         * Force running=true during callback execution: the dispatcher may be
         * invoked after core 0 has been marked stopped (e.g. main_task exited
         * via vTaskDelete), but periodic timers still need to fire to drive
         * LVGL / tick subsystems running on core 1. */
        int save_running = et->cpu->running;
        int save_halted = et->cpu->halted;
        et->cpu->running = 1;
        /* A halted core executes nothing: xtensa_step() returns immediately
         * and only ticks ccount. Since the idle task parks an otherwise-idle
         * core in WAITI, that is the normal state at the moment a timer comes
         * due -- so leaving it set meant the dispatcher faithfully rescheduled
         * every periodic timer and advanced every alarm while never running a
         * single callback body. We are synthesizing a call, so the core has to
         * be awake for it; the previous state is restored below. */
        et->cpu->halted = false;
        int max_cb_cycles = 100000;
        for (int c = 0; c < max_cb_cycles; c++) {
            if (et->cpu->pc == CALLBACK_SENTINEL) break;
            xtensa_step(et->cpu);
        }
        et->cpu->running = save_running;
        et->cpu->halted = save_halted;

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

/* esp_timer_start_periodic(handle, uint64_t period_us).
 * The 64-bit period_us must occupy an even-aligned register pair under the
 * Xtensa windowed ABI: handle is arg0 (a2/ar[ci*4+2]), and period_us occupies
 * arg2/arg3 (a4/a5) — arg1 (a3) is the padding slot. */
void stub_esp_timer_start_periodic(xtensa_cpu_t *cpu, void *ctx) {
    esp_timer_stubs_t *et = ctx;
    uint32_t handle = et_arg(cpu, 0);
    uint32_t lo = et_arg(cpu, 2);
    uint32_t hi = et_arg(cpu, 3);
    uint64_t period = ((uint64_t)hi << 32) | lo;

    emu_timer_t *t = find_timer(et, handle);
    if (!t) { et_return(cpu, -1); return; }

    t->active = 1;
    t->periodic = 1;
    t->period_us = period;
    t->alarm_us = current_time_us(et) + period;

    et_return(cpu, ESP_OK);
}

/* esp_timer_start_once(handle, uint64_t timeout_us). Same 64-bit alignment
 * rules as esp_timer_start_periodic. */
void stub_esp_timer_start_once(xtensa_cpu_t *cpu, void *ctx) {
    esp_timer_stubs_t *et = ctx;
    uint32_t handle = et_arg(cpu, 0);
    uint32_t lo = et_arg(cpu, 2);
    uint32_t hi = et_arg(cpu, 3);
    uint64_t timeout = ((uint64_t)hi << 32) | lo;

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

/* esp_timer_get_time() -> int64_t microseconds (returned in a2:a3).
 * In stub mode: host wall-clock so firmware gettimeofday() tracks real time.
 * In native mode: virtual time so timing is deterministic. */
void stub_esp_timer_get_time(xtensa_cpu_t *cpu, void *ctx) {
    esp_timer_stubs_t *et = ctx;
    uint64_t us = et->use_virtual_time ? current_time_us(et) : host_elapsed_us(et);
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
        cpu->pc = (cpu->pc & 0xC0000000u) | (ar_read(cpu, 0) & 0x3FFFFFFFu);
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
    bool yielded = et_block_us(et, cpu, us);
    dispatch_expired_timers(et);
    if (!yielded)
        et_return(cpu, 0);
}

/* esp_timer_init() — no-op, return ESP_OK */
void stub_esp_timer_init(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    et_return(cpu, ESP_OK);
}

/* millis() — in stub mode: host wall-clock. In native mode: virtual time. */
void stub_millis(xtensa_cpu_t *cpu, void *ctx) {
    esp_timer_stubs_t *et = ctx;
    uint64_t us = et->use_virtual_time ? current_time_us(et) : host_elapsed_us(et);
    et_return(cpu, (uint32_t)(us / 1000));
}

/* micros() — in stub mode: host wall-clock. In native mode: virtual time. */
void stub_micros(xtensa_cpu_t *cpu, void *ctx) {
    esp_timer_stubs_t *et = ctx;
    uint64_t us = et->use_virtual_time ? current_time_us(et) : host_elapsed_us(et);
    et_return(cpu, (uint32_t)us);
}

/* delay(ms) — Arduino's delay() is vTaskDelay() on real hardware, so block on
 * the scheduler rather than jumping the clock past everything that should
 * have run during the wait. */
void stub_delay(xtensa_cpu_t *cpu, void *ctx) {
    esp_timer_stubs_t *et = ctx;
    uint32_t ms = et_arg(cpu, 0);
    bool yielded = et_block_us(et, cpu, (uint64_t)ms * 1000u);
    dispatch_expired_timers(et);
    if (!yielded)
        et_return_void(cpu);
}

/* ===== Public API ===== */

esp_timer_stubs_t *esp_timer_stubs_create(xtensa_cpu_t *cpu) {
    esp_timer_stubs_t *et = calloc(1, sizeof(*et));
    if (!et) return NULL;
    et->cpu = cpu;
    et->cpu_freq_mhz = 160;
    et->use_virtual_time = 1;
    et->bump_ptr = TIMER_BUMP_BASE;
    clock_gettime(CLOCK_MONOTONIC, &et->boot_time);
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

void esp_timer_stubs_set_virtual_time(esp_timer_stubs_t *et, int enable) {
    if (et) et->use_virtual_time = enable;
}

void esp_timer_stubs_set_sleep_fn(esp_timer_stubs_t *et,
                                  esp_timer_sleep_fn fn, void *ctx) {
    if (!et) return;
    et->sleep_fn = fn;
    et->sleep_ctx = ctx;
}


void esp_timer_stubs_tick(esp_timer_stubs_t *et) {
    if (!et) return;
    /* Fast path: bail out before touching the timer table if nothing is
     * active. This keeps the common-case cost to a few comparisons per
     * batch.  Without a fast-path check here, flexe_session_post_batch
     * would pay O(MAX_TIMERS) on every batch regardless. */
    int any_active = 0;
    for (int i = 0; i < et->timer_count; i++) {
        if (et->timers[i].active) { any_active = 1; break; }
    }
    if (!any_active) return;
    dispatch_expired_timers(et);
}
