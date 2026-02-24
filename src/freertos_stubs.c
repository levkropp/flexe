#include "freertos_stubs.h"
#include "rom_stubs.h"
#include "memory.h"
#include <stdlib.h>
#include <string.h>

/* Bump allocator region in high SRAM */
#define BUMP_BASE   0x3FFF0000u
#define BUMP_LIMIT  0x3FFF4000u  /* 16KB */

/* FreeRTOS constants */
#define pdTRUE   1
#define pdFALSE  0
#define pdPASS   1

/* Queue storage */
#define MAX_QUEUES       16
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
};

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

/* ===== FreeRTOS stub implementations ===== */

/* vTaskDelay(ticks) — advance ccount by ticks * cycles_per_tick */
void stub_vTaskDelay(xtensa_cpu_t *cpu, void *ctx) {
    freertos_stubs_t *frt = ctx;
    uint32_t ticks = frt_arg(cpu, 0);
    /* Cap at 200M cycles per call to avoid overflow */
    uint64_t advance = (uint64_t)ticks * frt->cycles_per_tick;
    if (advance > 200000000ULL) advance = 200000000ULL;
    cpu->ccount += (uint32_t)advance;
    frt_return_void(cpu);
}

/* xTaskCreate(func, name, stack, param, prio, handle_out) -> pdPASS */
void stub_xTaskCreate(xtensa_cpu_t *cpu, void *ctx) {
    freertos_stubs_t *frt = ctx;
    /* arg 4 (index 4) is handle_out pointer */
    uint32_t handle_out = frt_arg(cpu, 4);
    if (handle_out) {
        uint32_t fake_handle = bump_alloc(frt, 4);
        if (fake_handle)
            mem_write32(cpu->mem, handle_out, fake_handle);
    }
    frt_return(cpu, pdPASS);
}

/* xTaskCreatePinnedToCore(func, name, stack, param, prio, handle_out, core) -> pdPASS */
void stub_xTaskCreatePinnedToCore(xtensa_cpu_t *cpu, void *ctx) {
    freertos_stubs_t *frt = ctx;
    uint32_t handle_out = frt_arg(cpu, 4);
    if (handle_out) {
        uint32_t fake_handle = bump_alloc(frt, 4);
        if (fake_handle)
            mem_write32(cpu->mem, handle_out, fake_handle);
    }
    frt_return(cpu, pdPASS);
}

/* vTaskDelete(handle) — no-op */
void stub_vTaskDelete(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    frt_return_void(cpu);
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
    if (!q || q->count >= q->max_items) {
        frt_return(cpu, pdFALSE);
        return;
    }

    /* Copy item from emulator memory into queue buffer */
    int offset = q->tail * q->item_size;
    for (int i = 0; i < q->item_size; i++)
        q->buf[offset + i] = mem_read8(cpu->mem, item_ptr + (uint32_t)i);
    q->tail = (q->tail + 1) % q->max_items;
    q->count++;

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
        /* Advance ccount by timeout ticks */
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

/* xTaskGetCurrentTaskHandle() -> fake handle */
void stub_xTaskGetCurrentTaskHandle(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    frt_return(cpu, 0x3FFF0100u); /* fixed fake handle */
}

/* vTaskDelay wrapper that also stops CPU if ticks == portMAX_DELAY (0xFFFFFFFF) */
void stub_vTaskSuspend(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    /* Suspend = stop CPU (task won't be rescheduled in emulator) */
    cpu->running = false;
}

/* uxTaskGetStackHighWaterMark() -> return a comfortable margin */
void stub_uxTaskGetStackHighWaterMark(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    frt_return(cpu, 2048);
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

/* ===== Public API ===== */

freertos_stubs_t *freertos_stubs_create(xtensa_cpu_t *cpu) {
    freertos_stubs_t *frt = calloc(1, sizeof(*frt));
    if (!frt) return NULL;
    frt->cpu = cpu;
    frt->bump_ptr = BUMP_BASE;
    frt->cpu_freq_mhz = 160;
    frt->cycles_per_tick = 1600000;  /* 160 MHz / 100 Hz */
    return frt;
}

void freertos_stubs_destroy(freertos_stubs_t *frt) {
    free(frt);
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
        { NULL, NULL }
    };

    for (int i = 0; hooks[i].name; i++) {
        uint32_t addr;
        if (elf_symbols_find(syms, hooks[i].name, &addr) == 0) {
            rom_stubs_register_ctx(rom, addr, hooks[i].fn, hooks[i].name, frt);
            hooked++;
        }
    }

    return hooked;
}

uint32_t freertos_stubs_bump_ptr(const freertos_stubs_t *frt) {
    return frt ? frt->bump_ptr : 0;
}
