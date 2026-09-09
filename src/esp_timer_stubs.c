#ifdef _MSC_VER
#include "msvc_compat.h"
#endif

#include "esp_timer_stubs.h"
#include "rom_stubs.h"
#include "firmware_scan.h"
#include "memory.h"
#include "guest_call.h"
#include "peripherals.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <time.h>

#define ESP_OK    0
#define MAX_TIMERS 16
#define ESP32_CPU_TICKS_PER_US_ADDR 0x3FFE01E0u

#define ESP32_TIMG0_BASE                 0x3FF5F000u
#define ESP32_TIMG1_BASE                 0x3FF60000u
#define ESP32_TIMG_LACT_CONFIG_OFFSET    0x70u
#define IDF_LACT_TIME_DIVIDER            40u
#define IDF_LACT_TIME_MAX_INSNS          160u
#define IDF_LACT_READER_SIZE             74u
#define IDF_LACT_ACCESSOR_SIZE           20u
#define MAX_NATIVE_LACT_ACCESSORS        8u

/* Instruction budget for one esp_timer callback. */
#define ESP_TIMER_CALLBACK_INSNS 100000u

typedef struct {
    int      active;
    int      periodic;
    uint32_t callback_addr;
    uint32_t arg;
    uint64_t period_us;
    uint64_t alarm_us;     /* absolute time in microseconds */
    uint32_t handle;       /* address returned as handle */
} emu_timer_t;

typedef struct {
    uint32_t accessor_entry;
    uint32_t reader_entry;
    uint32_t config_addr;
    int group;
} native_lact_accessor_t;

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

    /* Complete IDF implementations found structurally in stripped images.
     * Each hook keeps the timer group decoded from its own L32R literals. */
    native_lact_accessor_t native_lact[MAX_NATIVE_LACT_ACCESSORS];
    unsigned native_lact_count;
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

/* The clock as the *calling* core sees it.
 *
 * et->cpu is core 0, and each core carries its own cycle_count and
 * virtual_time_us until flexe_session_post_batch() republishes the later of the
 * two to both. Reading core 0 unconditionally meant a task on core 1 could not
 * observe time it had just skipped itself: a run of ets_delay_us() calls
 * returned with the caller's clock unmoved, because the whole sequence fitted
 * inside one batch and the reconciliation had not happened yet. Take the later
 * of the two, which is the value post_batch would publish anyway — just
 * observed now rather than at the next boundary, and still monotonic. */
static uint64_t current_time_us_seen_by(esp_timer_stubs_t *et,
                                        const xtensa_cpu_t *cpu) {
    uint64_t base = current_time_us(et);
    if (!cpu || cpu == et->cpu) return base;
    uint64_t mine = xtensa_guest_time_us(cpu, current_cpu_freq_mhz(et));
    return mine > base ? mine : base;
}

/* Fast-forward past a wait when no scheduler is available to block on.
 * Guest-visible time (ccount, virtual_time_us) advances; cycle_count does
 * not, because it counts *executed* guest instructions and drives the -c
 * budget, and a fast-forward executes none.
 *
 * CCOUNT walks to the end of the wait through xtensa_advance_idle_cycles()
 * rather than being assigned: writing it directly stepped over every ccompare
 * and peripheral deadline inside the window, so a driver that needs a
 * *sequence* of interrupts across a wait saw one. */
static void advance_wait_time(esp_timer_stubs_t *et, xtensa_cpu_t *cpu,
                              uint64_t us) {
    uint64_t cycles = us * current_cpu_freq_mhz(et);
    cpu->virtual_time_us += us;
    xtensa_advance_idle_cycles(cpu, cycles);
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

        /* Timer expired. Deliver it with guest_call8(), the shared
         * synthetic-call path also used for peripheral ISRs and BLE/WiFi
         * callbacks. This used to be hand-rolled here, and the hand-rolled
         * copy had drifted: it ran the callback on whichever task stack
         * happened to be interrupted rather than the private one, saved the
         * general registers but not BR, the MAC16 accumulator, the FP file or
         * the window spill state, did not set in_guest_call (so a callback
         * that touched a FreeRTOS primitive could take a scheduler switch and
         * write the synthetic register file over a real task's TCB), and left
         * cpu->halted set -- which, once the idle task started parking cores
         * in WAITI, meant the callback body never executed at all.
         *
         * A completed call leaves the core awake, as an interrupt waking
         * WAITI would on hardware; the idle task simply re-enters WAITI. */
        uint32_t arg = t->arg;
        (void)guest_call8(et->cpu, t->callback_addr, &arg, 1,
                          ESP_TIMER_CALLBACK_INSNS, NULL);

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
    uint64_t us = et->use_virtual_time ? current_time_us_seen_by(et, cpu)
                                      : host_elapsed_us(et);
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

/* This IDF timer accessor is a pure wrapper around a timer-group LACT
 * counter. The inner routine deliberately polls until an APB/divider tick
 * arrives, so its dynamic length is phase-dependent (33 to 153 instructions
 * at divider 40, plus one possible 7-insn consistency retry). Reproduce
 * those reads and their exact instruction positions in C, but only after the
 * complete outer and inner routines have been discovered and their MMIO
 * literals decoded. */
static int stub_idf_lact_esp_timer_get_time(xtensa_cpu_t *cpu, void *ctx) {
    const native_lact_accessor_t *native = ctx;
    if (!native || XT_PS_CALLINC(cpu->ps) != 2u || cpu->seed_entry_link ||
        cpu->breakpoint_count != 0u || cpu->window_trace ||
        (cpu->irq_check && (cpu->interrupt & cpu->intenable)))
        return 0;

    unsigned wb = cpu->windowbase & 15u;
    uint32_t caller_sp = ar_read(cpu, 1);
    for (unsigned i = 1; i <= 6u; i++) {
        unsigned window = (wb + i) & 0xFu;
        if (cpu->windowstart & (1u << window))
            return 0;
    }

    if (cpu->next_timer_event != UINT32_MAX &&
        ((int32_t)(cpu->ccount - cpu->next_timer_event) >= 0 ||
         cpu->next_timer_event - cpu->ccount <=
             IDF_LACT_TIME_MAX_INSNS))
        return 0;

    uint32_t config = mem_read32(cpu->mem, native->config_addr);
    uint32_t divider = (config >> 13) & 0xFFFFu;
    if (divider != IDF_LACT_TIME_DIVIDER ||
        (config & (3u << 30)) != (3u << 30))
        return 0;

    esp32_periph_t *periph = cpu->periph_event_ctx;
    uint64_t snapshot;
    if (!periph_lact_counter_at_ccount(
            periph, cpu, native->group, 7u, &snapshot))
        return 0;
    uint32_t low_start = (uint32_t)snapshot;

    /* LOW is sampled by every third instruction starting at instruction 18.
     * The predicate is monotone over this tiny span, so locate the first
     * changed sample in at most six pure counter predictions. */
    uint32_t lo_iteration = 0u;
    uint32_t hi_iteration = divider;
    uint32_t poll_delta = 17u + 3u * hi_iteration;
    if (!periph_lact_counter_at_ccount(
            periph, cpu, native->group, poll_delta, &snapshot))
        return 0;
    if ((uint32_t)snapshot != low_start) {
        while (lo_iteration < hi_iteration) {
            uint32_t mid = lo_iteration +
                           (hi_iteration - lo_iteration) / 2u;
            uint32_t delta = 17u + 3u * mid;
            if (!periph_lact_counter_at_ccount(
                    periph, cpu, native->group, delta, &snapshot))
                return 0;
            if ((uint32_t)snapshot == low_start)
                lo_iteration = mid + 1u;
            else
                hi_iteration = mid;
        }
        poll_delta = 17u + 3u * lo_iteration;
        if (!periph_lact_counter_at_ccount(
                periph, cpu, native->group, poll_delta, &snapshot))
            return 0;
    }
    uint32_t low = (uint32_t)snapshot;

    /* Read a consistent HIGH:LOW pair. A rollover between the two LOW reads
     * takes the seven-instruction retry path before checking again. */
    uint32_t high = 0u;
    uint32_t low_delta = 0u;
    for (uint32_t retry = 0u; retry < 2u; retry++) {
        uint32_t high_delta = poll_delta + 5u + 7u * retry;
        low_delta = high_delta + 2u;
        if (!periph_lact_counter_at_ccount(
                periph, cpu, native->group, high_delta, &snapshot))
            return 0;
        high = (uint32_t)(snapshot >> 32);
        if (!periph_lact_counter_at_ccount(
                periph, cpu, native->group, low_delta, &snapshot))
            return 0;
        uint32_t checked_low = (uint32_t)snapshot;
        if (checked_low == low) {
            low = checked_low;
            break;
        }
        low = checked_low;
        if (retry == 1u) return 0;
    }

    /* Nine instructions follow the final LOW read, including the intercepted
     * ENTRY charged by the dispatcher. Decline before changing any caller
     * state when the complete path does not fit in this scheduler batch. */
    uint32_t total = low_delta + 9u;
    if (cpu->native_span_room != 0u && cpu->native_span_room < total)
        return 0;

    uint64_t us = (((uint64_t)high << 32) | low) >> 1;

    int ci = XT_PS_CALLINC(cpu->ps);
    ar_write(cpu, ci * 4 + 2, (uint32_t)us);
    ar_write(cpu, ci * 4 + 3, (uint32_t)(us >> 32));

    /* Reproduce state that survives both RETWs. Inactive physical windows
     * are architectural state: a later spill can expose them, so returning
     * only a2:a3 is not equivalent to executing the function. */
    ar_write(cpu, 9, caller_sp - 32u);           /* outer a1 */
    ar_write(cpu, 16, (2u << 30) |
             ((cpu->pc + 6u) & 0x3FFFFFFFu));    /* inner a0 */
    ar_write(cpu, 17, caller_sp - 64u);          /* inner a1 */
    ar_write(cpu, 18, (uint32_t)us);             /* outer a10 */
    ar_write(cpu, 19, high);                     /* outer a11 / inner a3 */
    ar_write(cpu, 24, low);                      /* inner a8 */
    ar_write(cpu, 25, native->config_addr + 8u); /* inner a9 */
    ar_write(cpu, 26, native->config_addr + 12u);/* inner a10 */

    cpu->lbeg = native->reader_entry + 37u;
    cpu->lend = native->reader_entry + 45u;
    cpu->lcount = divider - lo_iteration;
    cpu->window_callsize[(wb + 2u) & 15u] = 2u;
    cpu->window_callsize[(wb + 4u) & 15u] = 2u;
    XT_PS_SET_OWB(cpu->ps, (wb + 2u) & 15u);
    uint32_t a0 = ar_read(cpu, ci * 4);
    cpu->pc = (cpu->pc & 0xC0000000u) | (a0 & 0x3FFFFFFFu);

    cpu->ccount += total - 1u;
    cpu->cycle_count += total - 1u;
    return (int)total;
}

static bool idf_l32r_value(xtensa_mem_t *mem, uint32_t pc,
                           uint32_t *value_out) {
    uint32_t literal;
    return firmware_xtensa_l32r_target(mem, pc, &literal) &&
           firmware_peek(mem, literal, 4u, value_out);
}

static bool idf_lact_group(uint32_t config_addr, int *group_out) {
    if (!group_out)
        return false;
    if (config_addr == ESP32_TIMG0_BASE + ESP32_TIMG_LACT_CONFIG_OFFSET) {
        *group_out = 0;
        return true;
    }
    if (config_addr == ESP32_TIMG1_BASE + ESP32_TIMG_LACT_CONFIG_OFFSET) {
        *group_out = 1;
        return true;
    }
    return false;
}

/* Match both complete routines while allowing only address-bearing fields to
 * vary: the outer CALL8 displacement and five inner L32R displacements. The
 * decoded literals must describe one real ESP32 timer group's LACT register
 * block, which prevents an instruction lookalike from authorizing MMIO. */
static bool idf_lact_accessor_matches(xtensa_mem_t *mem, uint32_t entry,
                                      native_lact_accessor_t *native_out) {
    static const uint8_t outer_entry[] = {0x36, 0x41, 0x00};
    static const uint8_t outer_suffix[] = {
        0x10, 0x2B, 0x01, 0xA0, 0xA1, 0x41, 0xA0,
        0x22, 0x20, 0xB0, 0x31, 0x41, 0x1D, 0xF0,
    };
    static const uint8_t reader[] = {
        0x36, 0x41, 0x00, 0x21, 0xD3, 0xE8, 0x0C, 0x1A,
        0x91, 0xD1, 0xE8, 0xC0, 0x20, 0x00, 0x38, 0x09,
        0xC0, 0x20, 0x00, 0x88, 0x02, 0x80, 0x8D, 0xF4,
        0xAA, 0x88, 0x21, 0xCE, 0xE8, 0xC0, 0x20, 0x00,
        0xA9, 0x02, 0x76, 0x88, 0x07, 0xC0, 0x20, 0x00,
        0x28, 0x09, 0x27, 0x93, 0xFF, 0xA1, 0xCA, 0xE8,
        0x91, 0xC7, 0xE8, 0xC0, 0x20, 0x00, 0x38, 0x0A,
        0xC0, 0x20, 0x00, 0x88, 0x09, 0x87, 0x92, 0x04,
        0xC0, 0x20, 0x00, 0x1D, 0xF0, 0x2D, 0x08, 0x06,
        0xFA, 0xFF,
    };
    static const size_t reader_relocations[] = {
        4u, 5u, 9u, 10u, 27u, 28u, 46u, 47u, 49u, 50u,
    };
    _Static_assert(sizeof(reader) == IDF_LACT_READER_SIZE,
                   "IDF LACT reader size");
    _Static_assert(sizeof(outer_entry) + 3u + sizeof(outer_suffix) ==
                       IDF_LACT_ACCESSOR_SIZE,
                   "IDF LACT accessor size");

    if (!native_out ||
        !firmware_signature_matches(
            mem, entry, outer_entry, sizeof(outer_entry)) ||
        !firmware_signature_matches(
            mem, entry + 6u, outer_suffix, sizeof(outer_suffix)))
        return false;

    unsigned callinc;
    uint32_t reader_entry;
    if (!firmware_xtensa_call_target(
            mem, entry + 3u, &callinc, &reader_entry) || callinc != 2u ||
        !firmware_signature_matches_except(
            mem, reader_entry, reader, sizeof(reader), reader_relocations,
            sizeof(reader_relocations) / sizeof(reader_relocations[0])))
        return false;

    uint32_t config_addr;
    uint32_t count_low_addr;
    uint32_t update_addr;
    uint32_t count_high_addr;
    uint32_t count_low_again;
    int group;
    if (!idf_l32r_value(mem, reader_entry + 3u, &config_addr) ||
        !idf_l32r_value(mem, reader_entry + 8u, &count_low_addr) ||
        !idf_l32r_value(mem, reader_entry + 26u, &update_addr) ||
        !idf_l32r_value(mem, reader_entry + 45u, &count_high_addr) ||
        !idf_l32r_value(mem, reader_entry + 48u, &count_low_again) ||
        !idf_lact_group(config_addr, &group) ||
        count_low_addr != config_addr + 8u ||
        count_high_addr != config_addr + 12u ||
        update_addr != config_addr + 16u ||
        count_low_again != count_low_addr)
        return false;

    native_out->accessor_entry = entry;
    native_out->reader_entry = reader_entry;
    native_out->config_addr = config_addr;
    native_out->group = group;
    return true;
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
    uint64_t us = et->use_virtual_time ? current_time_us_seen_by(et, cpu)
                                      : host_elapsed_us(et);
    et_return(cpu, (uint32_t)(us / 1000));
}

/* micros() — in stub mode: host wall-clock. In native mode: virtual time. */
void stub_micros(xtensa_cpu_t *cpu, void *ctx) {
    esp_timer_stubs_t *et = ctx;
    uint64_t us = et->use_virtual_time ? current_time_us_seen_by(et, cpu)
                                      : host_elapsed_us(et);
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

int esp_timer_stubs_hook_firmware(esp_timer_stubs_t *et) {
    if (!et || !et->cpu || !et->cpu->mem)
        return 0;

    esp32_rom_stubs_t *rom = et->rom;
    if (!rom)
        rom = et->cpu->pc_hook_ctx;
    if (!rom)
        return 0;
    et->rom = rom;

    int hooked = 0;
    uint32_t last = ESP32_IRAM_INSN_ADDR_HIGH - IDF_LACT_ACCESSOR_SIZE;
    for (uint32_t entry = ESP32_FIRMWARE_INSN_ADDR_LOW;
         entry <= last; entry++) {
        const uint8_t *first = mem_get_ptr(et->cpu->mem, entry);
        if (!first || *first != 0x36u)
            continue;

        native_lact_accessor_t native;
        if (!idf_lact_accessor_matches(et->cpu->mem, entry, &native))
            continue;
        bool already_hooked = false;
        for (unsigned i = 0u; i < et->native_lact_count; i++)
            already_hooked |=
                et->native_lact[i].accessor_entry == native.accessor_entry;
        if (already_hooked)
            continue;
        if (et->native_lact_count >= MAX_NATIVE_LACT_ACCESSORS)
            break;

        native_lact_accessor_t *installed =
            &et->native_lact[et->native_lact_count];
        *installed = native;
        if (rom_stubs_register_conditional_exact_ctx(
                rom, entry, stub_idf_lact_esp_timer_get_time,
                "esp_timer_get_time", installed) != 0)
            continue;
        et->native_lact_count++;
        hooked++;
        entry += IDF_LACT_ACCESSOR_SIZE - 1u;
    }
    if (hooked != 0)
        et->cpu->accelerated_blocks = true;
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
