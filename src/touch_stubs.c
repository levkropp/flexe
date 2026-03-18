#ifdef _MSC_VER
#include "msvc_compat.h"
#endif

#include "touch_stubs.h"
#include "rom_stubs.h"
#include "memory.h"
#include <stdlib.h>
#ifndef _MSC_VER
#include <unistd.h>
#endif

/* CPU frequency for ccount advance during wait_tap polling */
#define DEFAULT_CPU_FREQ_MHZ 160

/* From emu_touch.c */
extern volatile int emu_app_running;

struct touch_stubs {
    xtensa_cpu_t       *cpu;
    esp32_rom_stubs_t  *rom;
    touch_state_fn      get_state;
    void               *state_ctx;
    int                 cpu_freq_mhz;
};

/* ===== Calling convention helpers ===== */

static uint32_t ts_arg(xtensa_cpu_t *cpu, int n) {
    int ci = XT_PS_CALLINC(cpu->ps);
    return ar_read(cpu, ci * 4 + 2 + n);
}

static void ts_return(xtensa_cpu_t *cpu, uint32_t retval) {
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

static void ts_return_void(xtensa_cpu_t *cpu) {
    int ci = XT_PS_CALLINC(cpu->ps);
    if (ci > 0) {
        uint32_t a0 = ar_read(cpu, ci * 4);
        cpu->pc = (cpu->pc & 0xC0000000u) | (a0 & 0x3FFFFFFFu);
        XT_PS_SET_CALLINC(cpu->ps, 0);
    } else {
        cpu->pc = ar_read(cpu, 0);
    }
}

/* ===== Touch stub implementations ===== */

static void stub_touch_init(xtensa_cpu_t *cpu, void *ctx) {
    (void)ctx;
    ts_return_void(cpu);
}

/* touch_read(int *x, int *y) -> bool (1=pressed, 0=not) */
static void stub_touch_read(xtensa_cpu_t *cpu, void *ctx) {
    touch_stubs_t *ts = ctx;
    uint32_t x_ptr = ts_arg(cpu, 0);
    uint32_t y_ptr = ts_arg(cpu, 1);

    int x = 0, y = 0;
    int pressed = 0;
    if (ts->get_state)
        pressed = ts->get_state(&x, &y, ts->state_ctx);

    if (pressed) {
        mem_write32(cpu->mem, x_ptr, (uint32_t)x);
        mem_write32(cpu->mem, y_ptr, (uint32_t)y);
    }
    ts_return(cpu, (uint32_t)pressed);
}

/* touch_wait_tap(int *x, int *y) — blocking: poll until tap detected */
static void stub_touch_wait_tap(xtensa_cpu_t *cpu, void *ctx) {
    touch_stubs_t *ts = ctx;
    uint32_t x_ptr = ts_arg(cpu, 0);
    uint32_t y_ptr = ts_arg(cpu, 1);

    if (!ts->get_state) { ts_return_void(cpu); return; }

    int tx = 0, ty = 0;

    /* Wait for finger down */
    while (emu_app_running && cpu->running) {
        int x, y;
        if (ts->get_state(&x, &y, ts->state_ctx)) {
            tx = x; ty = y;
            break;
        }
        cpu->virtual_time_us += 20000;
        usleep(20000);
    }

    /* Wait for finger up */
    while (emu_app_running && cpu->running) {
        int x, y;
        if (!ts->get_state(&x, &y, ts->state_ctx))
            break;
        tx = x; ty = y;
        cpu->virtual_time_us += 20000;
        usleep(20000);
    }

    mem_write32(cpu->mem, x_ptr, (uint32_t)tx);
    mem_write32(cpu->mem, y_ptr, (uint32_t)ty);
    ts_return_void(cpu);
}

/*
 * LVGL touchpad read callback:
 *   bool my_touchpad_read(lv_indev_drv_t *drv, lv_indev_data_t *data)
 *
 * lv_indev_data_t layout (LVGL 7.x):
 *   +0: lv_point_t point  { int16_t x, int16_t y }  = 4 bytes
 *   +4: uint32_t key
 *   +8: uint32_t btn_id
 *  +12: int16_t enc_diff
 *  +14: uint8_t state   (LV_INDEV_STATE_REL=0, LV_INDEV_STATE_PR=1)
 *
 * Returns false (no more data to read).
 */
static void stub_lv_touchpad_read(xtensa_cpu_t *cpu, void *ctx) {
    touch_stubs_t *ts = ctx;
    uint32_t data_ptr = ts_arg(cpu, 1); /* arg1 = lv_indev_data_t* */

    int x = 0, y = 0;
    int pressed = 0;
    if (ts->get_state)
        pressed = ts->get_state(&x, &y, ts->state_ctx);

    /* Write point.x, point.y as int16_t */
    mem_write16(cpu->mem, data_ptr + 0, (uint16_t)(int16_t)x);
    mem_write16(cpu->mem, data_ptr + 2, (uint16_t)(int16_t)y);
    /* Write state */
    mem_write8(cpu->mem, data_ptr + 14, pressed ? 1 : 0);

    ts_return(cpu, 0); /* false = no more data */
}

/*
 * TFT_eSPI::getTouch(uint16_t *x, uint16_t *y, uint16_t threshold)
 * Returns true if touched (z > threshold).
 * 'this' pointer is arg0 (C++ method).
 */
static void stub_tft_getTouch(xtensa_cpu_t *cpu, void *ctx) {
    touch_stubs_t *ts = ctx;
    uint32_t x_ptr = ts_arg(cpu, 1); /* arg1 = x ptr (arg0 = this) */
    uint32_t y_ptr = ts_arg(cpu, 2); /* arg2 = y ptr */

    int x = 0, y = 0;
    int pressed = 0;
    if (ts->get_state)
        pressed = ts->get_state(&x, &y, ts->state_ctx);

    if (pressed) {
        mem_write16(cpu->mem, x_ptr, (uint16_t)x);
        mem_write16(cpu->mem, y_ptr, (uint16_t)y);
    }
    ts_return(cpu, (uint32_t)pressed);
}

/*
 * TFT_eSPI::getTouchRawZ() — returns pressure value.
 * Return high value if pressed, 0 if not.
 */
static void stub_tft_getTouchRawZ(xtensa_cpu_t *cpu, void *ctx) {
    touch_stubs_t *ts = ctx;
    int x, y;
    int pressed = 0;
    if (ts->get_state)
        pressed = ts->get_state(&x, &y, ts->state_ctx);
    ts_return(cpu, pressed ? 2000 : 0);
}

/* ===== Public API ===== */

touch_stubs_t *touch_stubs_create(xtensa_cpu_t *cpu) {
    touch_stubs_t *ts = calloc(1, sizeof(*ts));
    if (!ts) return NULL;
    ts->cpu = cpu;
    ts->cpu_freq_mhz = DEFAULT_CPU_FREQ_MHZ;
    return ts;
}

void touch_stubs_destroy(touch_stubs_t *ts) {
    free(ts);
}

void touch_stubs_set_state_fn(touch_stubs_t *ts, touch_state_fn fn, void *ctx) {
    if (!ts) return;
    ts->get_state = fn;
    ts->state_ctx = ctx;
}

int touch_stubs_hook_symbols(touch_stubs_t *ts, const elf_symbols_t *syms) {
    if (!ts || !syms) return 0;

    esp32_rom_stubs_t *rom = ts->cpu->pc_hook_ctx;
    if (!rom) return 0;
    ts->rom = rom;

    int hooked = 0;
    struct {
        const char *name;
        rom_stub_fn fn;
    } hooks[] = {
        { "touch_init",     stub_touch_init },
        { "touch_read",     stub_touch_read },
        { "touch_wait_tap", stub_touch_wait_tap },
        /* LVGL touchpad callback (Marauder) */
        { "_Z16my_touchpad_readP15_lv_indev_drv_tP15lv_indev_data_t",
                            stub_lv_touchpad_read },
        /* TFT_eSPI touch methods */
        { "_ZN8TFT_eSPI8getTouchEPtS0_t",  stub_tft_getTouch },
        { "_ZN8TFT_eSPI12getTouchRawZEv",   stub_tft_getTouchRawZ },
        { NULL, NULL }
    };

    for (int i = 0; hooks[i].name; i++) {
        uint32_t addr;
        if (elf_symbols_find(syms, hooks[i].name, &addr) == 0) {
            rom_stubs_register_ctx(rom, addr, hooks[i].fn, hooks[i].name, ts);
            hooked++;
        }
    }

    return hooked;
}
