#include "freertos_stubs.h"
#include "rom_stubs.h"
#include "memory.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

/* Bump allocator region in high SRAM */
#define BUMP_BASE   0x3FFF0000u
#define BUMP_LIMIT  0x3FFF4000u  /* 16KB */

/* FreeRTOS constants */
#define pdTRUE   1
#define pdFALSE  0
#define pdPASS   1

/* Queue storage */
#define MAX_QUEUES       64
#define MAX_QUEUE_ITEMS  32
#define MAX_ITEM_SIZE    64

typedef struct {
    uint32_t handle;       /* address returned as handle */
    int      item_size;
    int      max_items;
    int      count;
    int      head;
    int      tail;
    uint8_t  buf[MAX_QUEUE_ITEMS * MAX_ITEM_SIZE];
} queue_t;

/* ===== Cooperative scheduler types ===== */

typedef enum {
    TASK_UNUSED = 0,
    TASK_READY,
    TASK_RUNNING,
    TASK_SLEEPING,
    TASK_BLOCKED_QUEUE
} task_state_t;

#define MAX_TASKS        8
#define TASK_STACK_SIZE  0x4000u      /* 16KB per task */
#define TASK_STACK_BASE  0x3FB00000u  /* PSRAM, 3MB offset from 0x3F800000 */

typedef struct {
    uint32_t     handle;
    uint32_t     entry_fn;
    uint32_t     param;
    uint8_t      priority;
    task_state_t state;
    char         name[16];
    uint64_t     wake_cycle;     /* cycle_count threshold for SLEEPING */
    uint32_t     blocked_queue;  /* queue handle for BLOCKED_QUEUE */
    /* Saved CPU context */
    uint32_t     ar[64];
    uint32_t     pc, ps;
    uint32_t     windowbase, windowstart;
    uint32_t     spill_base[16];
    struct { uint32_t base[8]; int depth; } spill_stack[16];
    uint32_t     sar, lbeg, lend, lcount;
    uint32_t     stack_top;
} task_tcb_t;

struct freertos_stubs {
    xtensa_cpu_t       *cpu;
    esp32_rom_stubs_t  *rom;   /* for registering hooks */

    /* Bump allocator */
    uint32_t bump_ptr;

    /* Tick configuration: 160 MHz / 100 Hz = 1,600,000 cycles per tick */
    uint32_t cycles_per_tick;
    uint32_t cpu_freq_mhz;

    /* Queue storage */
    queue_t  queues[MAX_QUEUES];
    int      queue_count;

    /* Deferred task: saved by xTaskCreatePinnedToCore for later execution */
    uint32_t deferred_task_fn;
    uint32_t deferred_task_param;

    /* Cooperative scheduler */
    task_tcb_t tasks[MAX_TASKS];
    int        task_count;
    int        current_task;       /* -1 during boot */
    bool       scheduler_started;
    uint64_t   last_switch_cycle;  /* cycle_count at last context switch */

    /* Event callback for task switches */
    freertos_event_fn event_fn;
    void             *event_ctx;
};

static void stub_deferred_task_trampoline(xtensa_cpu_t *cpu, void *ctx);

/* ===== Calling convention helpers (same as rom_stubs.c) ===== */

static uint32_t frt_arg(xtensa_cpu_t *cpu, int n) {
    int ci = XT_PS_CALLINC(cpu->ps);
    return ar_read(cpu, ci * 4 + 2 + n);
}

static void frt_return(xtensa_cpu_t *cpu, uint32_t retval) {
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

static void frt_return_void(xtensa_cpu_t *cpu) {
    int ci = XT_PS_CALLINC(cpu->ps);
    if (ci > 0) {
        uint32_t a0 = ar_read(cpu, ci * 4);
        cpu->pc = (cpu->pc & 0xC0000000u) | (a0 & 0x3FFFFFFFu);
        XT_PS_SET_CALLINC(cpu->ps, 0);
    } else {
        cpu->pc = ar_read(cpu, 0);
    }
}

/* ===== Bump allocator ===== */

static uint32_t bump_alloc(freertos_stubs_t *frt, uint32_t size) {
    /* Align to 4 bytes */
    size = (size + 3) & ~3u;
    if (frt->bump_ptr + size > BUMP_LIMIT)
        return 0;
    uint32_t ptr = frt->bump_ptr;
    frt->bump_ptr += size;
    return ptr;
}

/* ===== Cooperative scheduler core ===== */

static void sched_save_context(freertos_stubs_t *frt) {
    xtensa_cpu_t *cpu = frt->cpu;
    task_tcb_t *t = &frt->tasks[frt->current_task];
    memcpy(t->ar, cpu->ar, sizeof(cpu->ar));
    t->pc = cpu->pc;
    t->ps = cpu->ps;
    t->windowbase = cpu->windowbase;
    t->windowstart = cpu->windowstart;
    memcpy(t->spill_base, cpu->spill_base, sizeof(cpu->spill_base));
    memcpy(t->spill_stack, cpu->spill_stack, sizeof(cpu->spill_stack));
    t->sar = cpu->sar;
    t->lbeg = cpu->lbeg;
    t->lend = cpu->lend;
    t->lcount = cpu->lcount;
}

static void sched_restore_context(freertos_stubs_t *frt) {
    xtensa_cpu_t *cpu = frt->cpu;
    task_tcb_t *t = &frt->tasks[frt->current_task];
    memcpy(cpu->ar, t->ar, sizeof(cpu->ar));
    cpu->pc = t->pc;
    cpu->ps = t->ps;
    cpu->windowbase = t->windowbase;
    cpu->windowstart = t->windowstart;
    memcpy(cpu->spill_base, t->spill_base, sizeof(cpu->spill_base));
    memcpy(cpu->spill_stack, t->spill_stack, sizeof(cpu->spill_stack));
    cpu->sar = t->sar;
    cpu->lbeg = t->lbeg;
    cpu->lend = t->lend;
    cpu->lcount = t->lcount;
}

/* Wake any SLEEPING tasks whose wake_cycle has been reached.
 * Returns the nearest wake time if all tasks are sleeping, or 0. */
static uint64_t sched_wake_sleepers(freertos_stubs_t *frt) {
    uint64_t now = frt->cpu->cycle_count;
    uint64_t nearest = UINT64_MAX;
    for (int i = 0; i < frt->task_count; i++) {
        task_tcb_t *t = &frt->tasks[i];
        if (t->state == TASK_SLEEPING) {
            if (now >= t->wake_cycle) {
                t->state = TASK_READY;
            } else if (t->wake_cycle < nearest) {
                nearest = t->wake_cycle;
            }
        }
    }
    return nearest;
}

/* Find the highest-priority READY task, round-robin among equals.
 * Returns task index or -1 if none runnable. */
static int sched_pick_next(freertos_stubs_t *frt) {
    uint64_t nearest = sched_wake_sleepers(frt);

    /* Find highest priority among READY tasks */
    int best_prio = -1;
    for (int i = 0; i < frt->task_count; i++) {
        if (frt->tasks[i].state == TASK_READY && frt->tasks[i].priority > best_prio)
            best_prio = frt->tasks[i].priority;
    }

    if (best_prio >= 0) {
        /* Round-robin: start searching after current_task */
        int start = (frt->current_task + 1) % frt->task_count;
        for (int j = 0; j < frt->task_count; j++) {
            int i = (start + j) % frt->task_count;
            if (frt->tasks[i].state == TASK_READY && frt->tasks[i].priority == best_prio)
                return i;
        }
    }

    /* No READY tasks — if there are sleepers, fast-forward time */
    if (nearest != UINT64_MAX) {
        uint64_t advance = nearest - frt->cpu->cycle_count;
        frt->cpu->cycle_count = nearest;
        frt->cpu->ccount += (uint32_t)advance;
        /* Wake them and retry */
        sched_wake_sleepers(frt);

        best_prio = -1;
        for (int i = 0; i < frt->task_count; i++) {
            if (frt->tasks[i].state == TASK_READY && frt->tasks[i].priority > best_prio)
                best_prio = frt->tasks[i].priority;
        }
        if (best_prio >= 0) {
            int start = (frt->current_task + 1) % frt->task_count;
            for (int j = 0; j < frt->task_count; j++) {
                int i = (start + j) % frt->task_count;
                if (frt->tasks[i].state == TASK_READY && frt->tasks[i].priority == best_prio)
                    return i;
            }
        }
    }

    return -1;  /* all tasks blocked/unused */
}

/* Context-switch: save current, pick next, restore next.
 * If no runnable task, halt CPU. */
static void sched_switch(freertos_stubs_t *frt) {
    int prev = frt->current_task;
    int next = sched_pick_next(frt);
    if (next < 0) {
        frt->cpu->running = false;
        return;
    }
    frt->current_task = next;
    frt->tasks[next].state = TASK_RUNNING;
    frt->last_switch_cycle = frt->cpu->cycle_count;
    sched_restore_context(frt);

    /* Emit event callback on task change */
    if (frt->event_fn && next != prev) {
        const char *from = (prev >= 0) ? frt->tasks[prev].name : "boot";
        const char *to = frt->tasks[next].name;
        frt->event_fn(from, to, frt->cpu->cycle_count, frt->event_ctx);
    }
}

/* Read a C string from emulator memory into buf */
static void frt_read_string(xtensa_cpu_t *cpu, uint32_t addr, char *buf, int max) {
    for (int i = 0; i < max - 1; i++) {
        uint8_t c = mem_read8(cpu->mem, addr + (uint32_t)i);
        buf[i] = (char)c;
        if (c == 0) return;
    }
    buf[max - 1] = '\0';
}

/* Register a new task in the scheduler TCB table.
 * Returns the task index, or -1 if table is full. */
static int sched_register_task(freertos_stubs_t *frt, uint32_t fn, uint32_t param,
                               uint8_t priority, uint32_t handle) {
    if (frt->task_count >= MAX_TASKS) return -1;
    int idx = frt->task_count++;
    task_tcb_t *t = &frt->tasks[idx];
    memset(t, 0, sizeof(*t));
    t->handle = handle;
    t->entry_fn = fn;
    t->param = param;
    t->priority = priority;
    t->state = TASK_READY;
    /* Assign a stack in PSRAM */
    t->stack_top = TASK_STACK_BASE + (uint32_t)(idx + 1) * TASK_STACK_SIZE;
    /* Pre-initialize saved context for first switch-in */
    t->pc = fn;
    t->ps = 0x00040020u;  /* WOE=1, UM=1 */
    t->windowbase = 0;
    t->windowstart = 1;   /* window 0 valid */
    /* a1 = SP (physical ar[1]), a2 = param (physical ar[2]) */
    t->ar[1] = t->stack_top;
    t->ar[2] = param;
    return idx;
}

/* Start the scheduler: save boot context as implicit task 0 if needed,
 * then pick the highest-priority task to run. */
static void sched_start(freertos_stubs_t *frt) {
    if (frt->scheduler_started) return;
    frt->scheduler_started = true;
    frt->current_task = -1;

    /* Reset ccount so millis()/micros() start near zero when tasks begin.
     * Boot-time ets_delay_us() calls inflate ccount to billions, which
     * causes firmware division-by-subtraction loops on millis() to hang. */
    frt->cpu->ccount = 0;
    frt->cpu->cycle_count = 0;

    /* All registered tasks are READY; pick the first to run */
    int next = sched_pick_next(frt);
    if (next >= 0) {
        frt->current_task = next;
        frt->tasks[next].state = TASK_RUNNING;
        frt->last_switch_cycle = frt->cpu->cycle_count;
        sched_restore_context(frt);
    }
}

/* Promote from legacy single-task mode to scheduler mode.
 * Called when a new task is registered while already running in legacy mode.
 * The currently-executing code becomes task 0 (already in the tasks array). */
static void sched_promote_legacy(freertos_stubs_t *frt) {
    if (frt->scheduler_started || frt->task_count < 2)
        return;
    frt->scheduler_started = true;
    /* The legacy task is tasks[0] — mark it as RUNNING and current */
    frt->current_task = 0;
    frt->tasks[0].state = TASK_RUNNING;
    /* Save current CPU context into tasks[0] so context switches work */
    sched_save_context(frt);
    frt->last_switch_cycle = frt->cpu->cycle_count;
}

/* ===== FreeRTOS stub implementations ===== */

/* vTaskDelay(ticks) — yield to scheduler or advance ccount */
void stub_vTaskDelay(xtensa_cpu_t *cpu, void *ctx) {
    freertos_stubs_t *frt = ctx;
    uint32_t ticks = frt_arg(cpu, 0);
    uint64_t advance = (uint64_t)ticks * frt->cycles_per_tick;
    if (advance > 200000000ULL) advance = 200000000ULL;

    if (frt->scheduler_started && frt->current_task >= 0) {
        /* Yield: unwind call, save, sleep, switch */
        frt_return_void(cpu);
        sched_save_context(frt);
        task_tcb_t *t = &frt->tasks[frt->current_task];
        t->state = TASK_SLEEPING;
        t->wake_cycle = cpu->cycle_count + advance;
        sched_switch(frt);
    } else {
        /* Legacy: single-task mode */
        cpu->ccount += (uint32_t)advance;
        cpu->cycle_count += advance;
        frt_return_void(cpu);
    }
}

/* xTaskCreate(func, name, stack, param, prio, handle_out) -> pdPASS */
void stub_xTaskCreate(xtensa_cpu_t *cpu, void *ctx) {
    freertos_stubs_t *frt = ctx;
    uint32_t task_fn    = frt_arg(cpu, 0);
    uint32_t name_addr  = frt_arg(cpu, 1);
    uint32_t task_par   = frt_arg(cpu, 3);
    uint32_t prio       = frt_arg(cpu, 4);
    uint32_t handle_out = frt_arg(cpu, 5);

    char task_name[16] = "?";
    if (name_addr)
        frt_read_string(cpu, name_addr, task_name, sizeof(task_name));

    uint32_t fake_handle = 0;
    if (handle_out) {
        fake_handle = bump_alloc(frt, 4);
        if (fake_handle)
            mem_write32(cpu->mem, handle_out, fake_handle);
    }

    /* Register in scheduler if app-level task (priority < 20) */
    if (task_fn && prio < 20) {
        if (!fake_handle) fake_handle = bump_alloc(frt, 4);
        int idx = sched_register_task(frt, task_fn, task_par, (uint8_t)prio, fake_handle);
        if (idx >= 0)
            memcpy(frt->tasks[idx].name, task_name, sizeof(task_name));
        /* If we're in legacy single-task mode, promote to scheduler */
        if (!frt->scheduler_started && frt->task_count > 1)
            sched_promote_legacy(frt);
    }

    /* Legacy deferred task (for single-task compat) */
    if (!frt->deferred_task_fn && task_fn) {
        frt->deferred_task_fn = task_fn;
        frt->deferred_task_param = task_par;
    }
    frt_return(cpu, pdPASS);
}

/* xTaskCreatePinnedToCore(func, name, stack, param, prio, handle_out, core) -> pdPASS */
void stub_xTaskCreatePinnedToCore(xtensa_cpu_t *cpu, void *ctx) {
    freertos_stubs_t *frt = ctx;
    uint32_t task_fn    = frt_arg(cpu, 0);
    uint32_t name_addr  = frt_arg(cpu, 1);
    uint32_t task_par   = frt_arg(cpu, 3);
    uint32_t prio       = frt_arg(cpu, 4);
    uint32_t handle_out = frt_arg(cpu, 5);

    char task_name[16] = "?";
    if (name_addr)
        frt_read_string(cpu, name_addr, task_name, sizeof(task_name));

    uint32_t fake_handle = 0;
    if (handle_out) {
        fake_handle = bump_alloc(frt, 4);
        if (fake_handle)
            mem_write32(cpu->mem, handle_out, fake_handle);
    }

    /* Register in scheduler if app-level task (priority < 20) */
    if (task_fn && prio < 20) {
        if (!fake_handle) fake_handle = bump_alloc(frt, 4);
        int idx = sched_register_task(frt, task_fn, task_par, (uint8_t)prio, fake_handle);
        if (idx >= 0)
            memcpy(frt->tasks[idx].name, task_name, sizeof(task_name));
        /* If we're in legacy single-task mode, promote to scheduler */
        if (!frt->scheduler_started && frt->task_count > 1)
            sched_promote_legacy(frt);
    }

    /* Legacy: save most recent task for deferred execution */
    if (task_fn) {
        frt->deferred_task_fn = task_fn;
        frt->deferred_task_param = task_par;
    }
    frt_return(cpu, pdPASS);
}

/* vTaskDelete(handle) — remove task from scheduler or no-op */
void stub_vTaskDelete(xtensa_cpu_t *cpu, void *ctx) {
    freertos_stubs_t *frt = ctx;
    uint32_t handle = frt_arg(cpu, 0);

    if (frt->scheduler_started) {
        frt_return_void(cpu);
        if (handle == 0 && frt->current_task >= 0) {
            /* Delete self: mark UNUSED and switch away */
            frt->tasks[frt->current_task].state = TASK_UNUSED;
            sched_switch(frt);
        } else {
            /* Delete by handle — find and mark UNUSED */
            for (int i = 0; i < frt->task_count; i++) {
                if (frt->tasks[i].handle == handle && frt->tasks[i].state != TASK_UNUSED) {
                    frt->tasks[i].state = TASK_UNUSED;
                    break;
                }
            }
        }
    } else {
        frt_return_void(cpu);
    }
}

/* xTaskGetTickCount() -> ccount / cycles_per_tick */
void stub_xTaskGetTickCount(xtensa_cpu_t *cpu, void *ctx) {
    freertos_stubs_t *frt = ctx;
    uint32_t ticks = cpu->ccount / frt->cycles_per_tick;
    frt_return(cpu, ticks);
}

/* xQueueCreate(length, item_size) -> handle */
void stub_xQueueCreate(xtensa_cpu_t *cpu, void *ctx) {
    freertos_stubs_t *frt = ctx;
    uint32_t length = frt_arg(cpu, 0);
    uint32_t item_size = frt_arg(cpu, 1);

    if (frt->queue_count >= MAX_QUEUES || item_size > MAX_ITEM_SIZE ||
        length > MAX_QUEUE_ITEMS) {
        frt_return(cpu, 0);
        return;
    }

    queue_t *q = &frt->queues[frt->queue_count];
    uint32_t handle = bump_alloc(frt, 4);
    if (!handle) { frt_return(cpu, 0); return; }

    q->handle = handle;
    q->item_size = (int)item_size;
    q->max_items = (int)length;
    q->count = 0;
    q->head = 0;
    q->tail = 0;
    frt->queue_count++;

    frt_return(cpu, handle);
}

/* Find queue by handle */
static queue_t *find_queue(freertos_stubs_t *frt, uint32_t handle) {
    for (int i = 0; i < frt->queue_count; i++)
        if (frt->queues[i].handle == handle)
            return &frt->queues[i];
    return NULL;
}

/* xQueueSend(queue, item, timeout) -> pdTRUE */
void stub_xQueueSend(xtensa_cpu_t *cpu, void *ctx) {
    freertos_stubs_t *frt = ctx;
    uint32_t handle = frt_arg(cpu, 0);
    uint32_t item_ptr = frt_arg(cpu, 1);

    queue_t *q = find_queue(frt, handle);
    if (!q) {
        /* Unknown handle — probably a mutex created via xQueueCreateMutex
         * which only bump-allocates a handle.  Return success. */
        frt_return(cpu, pdTRUE);
        return;
    }
    if (q->count >= q->max_items) {
        frt_return(cpu, pdFALSE);
        return;
    }

    /* Copy item from emulator memory into queue buffer */
    int offset = q->tail * q->item_size;
    for (int i = 0; i < q->item_size; i++)
        q->buf[offset + i] = mem_read8(cpu->mem, item_ptr + (uint32_t)i);
    q->tail = (q->tail + 1) % q->max_items;
    q->count++;

    /* Wake one task blocked on this queue */
    if (frt->scheduler_started) {
        for (int i = 0; i < frt->task_count; i++) {
            if (frt->tasks[i].state == TASK_BLOCKED_QUEUE &&
                frt->tasks[i].blocked_queue == handle) {
                frt->tasks[i].state = TASK_READY;
                break;  /* wake only one */
            }
        }
    }

    frt_return(cpu, pdTRUE);
}

/* xQueueSendFromISR — same as xQueueSend but with extra arg */
void stub_xQueueSendFromISR(xtensa_cpu_t *cpu, void *ctx) {
    stub_xQueueSend(cpu, ctx);
}

/* xQueueReceive(queue, buf, timeout) -> pdTRUE/pdFALSE */
void stub_xQueueReceive(xtensa_cpu_t *cpu, void *ctx) {
    freertos_stubs_t *frt = ctx;
    uint32_t handle = frt_arg(cpu, 0);
    uint32_t buf_ptr = frt_arg(cpu, 1);
    uint32_t timeout = frt_arg(cpu, 2);

    queue_t *q = find_queue(frt, handle);
    if (!q) { frt_return(cpu, pdFALSE); return; }

    if (q->count == 0) {
        if (frt->scheduler_started && frt->current_task >= 0 &&
            timeout > 0) {
            /* Block: unwind call (returning pdFALSE), save, sleep/block, switch */
            frt_return(cpu, pdFALSE);
            sched_save_context(frt);
            task_tcb_t *t = &frt->tasks[frt->current_task];
            if (timeout == 0xFFFFFFFFu) {
                /* portMAX_DELAY: block on queue indefinitely */
                t->state = TASK_BLOCKED_QUEUE;
                t->blocked_queue = handle;
            } else {
                /* Timed wait: sleep until timeout */
                uint64_t advance = (uint64_t)timeout * frt->cycles_per_tick;
                if (advance > 200000000ULL) advance = 200000000ULL;
                t->state = TASK_SLEEPING;
                t->wake_cycle = cpu->cycle_count + advance;
            }
            sched_switch(frt);
            return;
        }
        /* Legacy: advance ccount and return pdFALSE */
        if (timeout > 0 && timeout != 0xFFFFFFFFu) {
            uint64_t advance = (uint64_t)timeout * frt->cycles_per_tick;
            if (advance > 200000000ULL) advance = 200000000ULL;
            cpu->ccount += (uint32_t)advance;
        }
        frt_return(cpu, pdFALSE);
        return;
    }

    /* Dequeue: copy from queue buffer to emulator memory */
    int offset = q->head * q->item_size;
    for (int i = 0; i < q->item_size; i++)
        mem_write8(cpu->mem, buf_ptr + (uint32_t)i, q->buf[offset + i]);
    q->head = (q->head + 1) % q->max_items;
    q->count--;

    frt_return(cpu, pdTRUE);
}

/* xSemaphoreCreateMutex() -> non-null handle */
void stub_xSemaphoreCreateMutex(xtensa_cpu_t *cpu, void *ctx) {
    freertos_stubs_t *frt = ctx;
    uint32_t handle = bump_alloc(frt, 4);
    frt_return(cpu, handle);
}

/* xSemaphoreCreateBinary() -> non-null handle */
void stub_xSemaphoreCreateBinary(xtensa_cpu_t *cpu, void *ctx) {
    freertos_stubs_t *frt = ctx;
    uint32_t handle = bump_alloc(frt, 4);
    frt_return(cpu, handle);
}

/* xSemaphoreTake(sem, timeout) -> pdTRUE */
void stub_xSemaphoreTake(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    frt_return(cpu, pdTRUE);
}

/* xSemaphoreGive(sem) -> pdTRUE */
void stub_xSemaphoreGive(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    frt_return(cpu, pdTRUE);
}

/* pvPortMalloc(size) -> bump-allocated address */
void stub_pvPortMalloc(xtensa_cpu_t *cpu, void *ctx) {
    freertos_stubs_t *frt = ctx;
    uint32_t size = frt_arg(cpu, 0);
    uint32_t ptr = bump_alloc(frt, size);
    frt_return(cpu, ptr);
}

/* vPortFree(ptr) — no-op */
void stub_vPortFree(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    frt_return_void(cpu);
}

/* xTaskGetCurrentTaskHandle() -> current task handle or fixed fake */
void stub_xTaskGetCurrentTaskHandle(xtensa_cpu_t *cpu, void *ctx) {
    freertos_stubs_t *frt = ctx;
    if (frt->scheduler_started && frt->current_task >= 0) {
        frt_return(cpu, frt->tasks[frt->current_task].handle);
    } else {
        frt_return(cpu, 0x3FFF0100u); /* fixed fake handle */
    }
}

/* vTaskSuspend(handle) — mark task UNUSED and switch */
void stub_vTaskSuspend(xtensa_cpu_t *cpu, void *ctx) {
    freertos_stubs_t *frt = ctx;
    if (frt->scheduler_started && frt->current_task >= 0) {
        frt_return_void(cpu);
        sched_save_context(frt);
        frt->tasks[frt->current_task].state = TASK_UNUSED;
        sched_switch(frt);
    } else {
        cpu->running = false;
    }
}

/* uxTaskGetStackHighWaterMark() -> return a comfortable margin */
void stub_uxTaskGetStackHighWaterMark(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    frt_return(cpu, 2048);
}

/* disableCore0WDT / disableCore1WDT — no-op in emulator */
static void stub_disableCoreWDT(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    frt_return_void(cpu);
}

/* xTaskGetIdleTaskHandleForCPU(cpu) -> fake handle */
static void stub_xTaskGetIdleTaskHandleForCPU(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    frt_return(cpu, 0x3FFF0200u);
}

/* xPortEnterCriticalTimeout / vPortExitCritical — no-op in emulator
 * (single-threaded, no contention) */
static void stub_xPortEnterCriticalTimeout(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    frt_return_void(cpu);
}

/* xTaskGetSchedulerState() -> taskSCHEDULER_RUNNING (2) */
static void stub_xTaskGetSchedulerState(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    frt_return(cpu, 2);
}

/* esp_ipc_call / esp_ipc_call_blocking — no-op, IPC not needed */
static void stub_esp_ipc_noop(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    frt_return(cpu, 0);
}

/* vTaskStartScheduler — start cooperative scheduler, never returns */
static void stub_vTaskStartScheduler(xtensa_cpu_t *cpu, void *ctx) {
    freertos_stubs_t *frt = ctx;
    (void)cpu;
    sched_start(frt);
    /* sched_start jumps to first task via sched_restore_context,
     * so we never return to the caller's j-self loop. */
}

/* vPortYield / vTaskSwitchContext — yield to scheduler */
static void stub_vPortYield(xtensa_cpu_t *cpu, void *ctx) {
    freertos_stubs_t *frt = ctx;
    if (frt->scheduler_started && frt->current_task >= 0) {
        frt_return_void(cpu);
        sched_save_context(frt);
        frt->tasks[frt->current_task].state = TASK_READY;
        sched_switch(frt);
    } else {
        frt_return_void(cpu);
    }
}

/* xQueueGenericCreate (underlying implementation xQueueCreate may alias) */
void stub_xQueueGenericCreate(xtensa_cpu_t *cpu, void *ctx) {
    stub_xQueueCreate(cpu, ctx);
}

/* xQueueGenericSend (underlying implementation) */
void stub_xQueueGenericSend(xtensa_cpu_t *cpu, void *ctx) {
    stub_xQueueSend(cpu, ctx);
}

/* xQueueGenericSendFromISR */
void stub_xQueueGenericSendFromISR(xtensa_cpu_t *cpu, void *ctx) {
    stub_xQueueSend(cpu, ctx);
}

/* ===== Preemptive timeslice check ===== */

bool freertos_stubs_check_preempt(freertos_stubs_t *frt) {
    if (!frt || !frt->scheduler_started || frt->current_task < 0)
        return false;
    if (frt->cpu->cycle_count - frt->last_switch_cycle < frt->cycles_per_tick)
        return false;
    int prev = frt->current_task;
    sched_save_context(frt);
    frt->tasks[frt->current_task].state = TASK_READY;

    sched_wake_sleepers(frt);

    int next = -1;
    for (int j = 1; j < frt->task_count; j++) {
        int i = (frt->current_task + j) % frt->task_count;
        if (frt->tasks[i].state == TASK_READY) {
            next = i;
            break;
        }
    }
    if (next < 0) next = frt->current_task;

    frt->current_task = next;
    frt->tasks[next].state = TASK_RUNNING;
    frt->last_switch_cycle = frt->cpu->cycle_count;
    sched_restore_context(frt);

    if (frt->event_fn && next != prev) {
        const char *from = frt->tasks[prev].name;
        const char *to = frt->tasks[next].name;
        frt->event_fn(from, to, frt->cpu->cycle_count, frt->event_ctx);
    }
    return true;
}

/* ===== Public API ===== */

freertos_stubs_t *freertos_stubs_create(xtensa_cpu_t *cpu) {
    freertos_stubs_t *frt = calloc(1, sizeof(*frt));
    if (!frt) return NULL;
    frt->cpu = cpu;
    frt->bump_ptr = BUMP_BASE;
    frt->cpu_freq_mhz = 160;
    frt->cycles_per_tick = 1600000;  /* 160 MHz / 100 Hz */
    frt->current_task = -1;
    return frt;
}

void freertos_stubs_destroy(freertos_stubs_t *frt) {
    free(frt);
}

void freertos_stubs_set_event_fn(freertos_stubs_t *frt, freertos_event_fn fn, void *ctx) {
    if (!frt) return;
    frt->event_fn = fn;
    frt->event_ctx = ctx;
}

const char *freertos_stubs_current_task_name(const freertos_stubs_t *frt) {
    if (!frt || frt->current_task < 0) return NULL;
    return frt->tasks[frt->current_task].name;
}

int freertos_stubs_hook_symbols(freertos_stubs_t *frt, const elf_symbols_t *syms) {
    if (!frt || !syms) return 0;

    /* We need a rom_stubs handle to register hooks. Get it from cpu->pc_hook_ctx */
    esp32_rom_stubs_t *rom = frt->cpu->pc_hook_ctx;
    if (!rom) return 0;
    frt->rom = rom;

    int hooked = 0;

    struct {
        const char *name;
        rom_stub_fn fn;
    } hooks[] = {
        { "vTaskDelay",                    stub_vTaskDelay },
        { "xTaskCreate",                   stub_xTaskCreate },
        { "xTaskCreatePinnedToCore",       stub_xTaskCreatePinnedToCore },
        { "vTaskDelete",                   stub_vTaskDelete },
        { "xTaskGetTickCount",             stub_xTaskGetTickCount },
        { "xQueueCreate",                  stub_xQueueCreate },
        { "xQueueGenericCreate",           stub_xQueueGenericCreate },
        { "xQueueSend",                    stub_xQueueSend },
        { "xQueueSendToBack",             stub_xQueueSend },
        { "xQueueGenericSend",             stub_xQueueGenericSend },
        { "xQueueSendFromISR",             stub_xQueueSendFromISR },
        { "xQueueGenericSendFromISR",      stub_xQueueGenericSendFromISR },
        { "xQueueReceive",                stub_xQueueReceive },
        { "xQueueGenericReceive",          stub_xQueueReceive },
        { "xSemaphoreCreateMutex",         stub_xSemaphoreCreateMutex },
        { "xQueueCreateMutex",             stub_xSemaphoreCreateMutex },
        { "xSemaphoreCreateBinary",        stub_xSemaphoreCreateBinary },
        { "xSemaphoreTake",                stub_xSemaphoreTake },
        { "xQueueSemaphoreTake",           stub_xSemaphoreTake },
        { "xSemaphoreGive",                stub_xSemaphoreGive },
        { "xQueueGenericReset",            stub_xSemaphoreGive },
        { "pvPortMalloc",                  stub_pvPortMalloc },
        { "vPortFree",                     stub_vPortFree },
        { "xTaskGetCurrentTaskHandle",     stub_xTaskGetCurrentTaskHandle },
        { "vTaskSuspend",                  stub_vTaskSuspend },
        { "uxTaskGetStackHighWaterMark",   stub_uxTaskGetStackHighWaterMark },
        { "disableCore0WDT",               stub_disableCoreWDT },
        { "disableCore1WDT",               stub_disableCoreWDT },
        { "disableLoopWDT",                stub_disableCoreWDT },
        { "enableCore0WDT",                stub_disableCoreWDT },
        { "enableCore1WDT",                stub_disableCoreWDT },
        { "enableLoopWDT",                 stub_disableCoreWDT },
        { "xTaskGetIdleTaskHandleForCPU",  stub_xTaskGetIdleTaskHandleForCPU },
        { "xPortEnterCriticalTimeout",     stub_xPortEnterCriticalTimeout },
        { "vPortExitCritical",             stub_xPortEnterCriticalTimeout },
        { "vPortEnterCritical",            stub_xPortEnterCriticalTimeout },
        { "xTaskGetSchedulerState",        stub_xTaskGetSchedulerState },
        { "esp_ipc_call",                  stub_esp_ipc_noop },
        { "esp_ipc_call_blocking",         stub_esp_ipc_noop },
        { "vApplicationStackOverflowHook", stub_disableCoreWDT },
        { "vTaskStartScheduler",           stub_vTaskStartScheduler },
        { "vPortYield",                    stub_vPortYield },
        { "vTaskSwitchContext",            stub_vPortYield },
        { NULL, NULL }
    };

    for (int i = 0; hooks[i].name; i++) {
        uint32_t addr;
        if (elf_symbols_find(syms, hooks[i].name, &addr) == 0) {
            rom_stubs_register_ctx(rom, addr, hooks[i].fn, hooks[i].name, frt);
            hooked++;
        }
    }

    /* Register deferred-task trampoline at ROM base (0x40000000) and _xt_panic.
     * When app_main returns after creating tasks, the call chain unwinds to ROM
     * or panics — this catches both and redirects to the saved task function. */
    rom_stubs_register_ctx(rom, 0x40000000u, stub_deferred_task_trampoline,
                           "deferred_task_trampoline", frt);
    {
        uint32_t addr;
        if (elf_symbols_find(syms, "_xt_panic", &addr) == 0) {
            rom_stubs_register_ctx(rom, addr, stub_deferred_task_trampoline,
                                   "_xt_panic", frt);
            hooked++;
        }
    }

    return hooked;
}

/* Stub that fires when execution reaches a dead end (e.g., app_main returns
 * to ROM after creating a FreeRTOS task).  If multiple tasks were registered,
 * start the cooperative scheduler.  Otherwise, use legacy single-task launch. */
static void stub_deferred_task_trampoline(xtensa_cpu_t *cpu, void *ctx) {
    freertos_stubs_t *frt = ctx;
    if (frt->task_count > 1) {
        /* Multi-task: start cooperative scheduler */
        sched_start(frt);
    } else if (frt->deferred_task_fn) {
        uint32_t fn = frt->deferred_task_fn;
        uint32_t param = frt->deferred_task_param;
        frt->deferred_task_fn = 0;  /* one-shot */
        /* Set up a fresh call context for the task function */
        ar_write(cpu, 1, 0x3FFE0000u);  /* SP */
        ar_write(cpu, 2, param);
        cpu->pc = fn;
        cpu->ps = 0x00040020u;  /* WOE=1, UM=1 */
    } else {
        cpu->running = 0;
    }
}

uint32_t freertos_stubs_bump_ptr(const freertos_stubs_t *frt) {
    return frt ? frt->bump_ptr : 0;
}

uint32_t freertos_stubs_deferred_task(const freertos_stubs_t *frt, uint32_t *param_out) {
    if (!frt) return 0;
    if (param_out) *param_out = frt->deferred_task_param;
    return frt->deferred_task_fn;
}

uint32_t freertos_stubs_consume_deferred_task(freertos_stubs_t *frt, uint32_t *param_out) {
    if (!frt) return 0;
    /* If scheduler has multiple tasks, start it and signal caller to skip manual setup */
    if (frt->task_count > 1) {
        sched_start(frt);
        return 0;  /* tell caller not to do manual PC/SP setup */
    }
    /* Legacy single-task path */
    if (!frt->deferred_task_fn) return 0;
    uint32_t fn = frt->deferred_task_fn;
    if (param_out) *param_out = frt->deferred_task_param;
    frt->deferred_task_fn = 0;  /* one-shot */
    return fn;
}

bool freertos_stubs_scheduler_active(const freertos_stubs_t *frt) {
    return frt && frt->scheduler_started;
}

void freertos_stubs_start_scheduler(freertos_stubs_t *frt) {
    if (!frt) return;
    if (frt->task_count > 1) {
        sched_start(frt);
    } else if (frt->deferred_task_fn) {
        /* Legacy single-task: jump directly to the deferred task */
        uint32_t fn = frt->deferred_task_fn;
        uint32_t param = frt->deferred_task_param;
        frt->deferred_task_fn = 0;
        ar_write(frt->cpu, 1, 0x3FFE0000u);
        ar_write(frt->cpu, 2, param);
        frt->cpu->pc = fn;
        frt->cpu->ps = 0x00040020u;
    } else {
        frt->cpu->running = 0;
    }
}
