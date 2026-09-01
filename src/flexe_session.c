/*
 * flexe_session.c — Shared ESP32 emulator session implementation
 *
 * Single source of truth for stub module lifecycle, dual-core
 * management, and cycle synchronization.
 */

#include "flexe_session.h"
#include "memory.h"
#include "loader.h"
#include "peripherals.h"
#include "rom_stubs.h"
#include "elf_symbols.h"
#include "freertos_stubs.h"
#include "esp_timer_stubs.h"
#include "display_stubs.h"
#include "touch_stubs.h"
#include "spi_display.h"
#include "sdcard_stubs.h"
#include "wifi_stubs.h"
#include "vfs_stubs.h"
#include "bt_stubs.h"
#include "sha_stubs.h"
#include "aes_stubs.h"
#include "mpi_stubs.h"
#include "jit.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

struct flexe_session {
    xtensa_cpu_t       cpu[2];
    xtensa_mem_t      *mem;
    esp32_periph_t    *periph;
    elf_symbols_t     *syms;
    esp32_rom_stubs_t *rom;
    freertos_stubs_t  *frt;
    esp_timer_stubs_t *etimer;
    display_stubs_t   *dstubs;
    touch_stubs_t     *tstubs;
    sdcard_stubs_t    *sstubs;
    wifi_stubs_t      *wstubs;
    vfs_stubs_t       *vstubs;
    bt_stubs_t        *bstubs;
    sha_stubs_t       *shstubs;
    aes_stubs_t       *astubs;
    mpi_stubs_t       *mstubs;
    jit_state_t       *jit;
    int                single_core;
    int                native_freertos;
    int              (*touch_fn)(int *x, int *y, void *ctx);
    void              *touch_ctx;
    int                touch_irq_pin;
    int                touch_irq_level;
};

/* Let the esp_timer delay()/usleep() shims block on the FreeRTOS scheduler
 * instead of fast-forwarding the clock past pending work. */
static bool session_sleep_us(void *ctx, xtensa_cpu_t *cpu, uint64_t us)
{
    return freertos_stubs_sleep_us((freertos_stubs_t *)ctx, cpu, us);
}

flexe_session_t *flexe_session_create(const flexe_session_config_t *cfg)
{
    if (!cfg || !cfg->bin_path) return NULL;

    flexe_session_t *s = calloc(1, sizeof(*s));
    if (!s) return NULL;
    s->single_core = cfg->single_core;
    s->native_freertos = cfg->native_freertos;
    s->touch_fn = cfg->touch_fn;
    s->touch_ctx = cfg->touch_ctx;
    s->touch_irq_pin = cfg->spi_touch_irq_pin ? cfg->spi_touch_irq_pin : 36;
    s->touch_irq_level = 1;

    /* Load ELF symbols */
    if (cfg->elf_path) {
        s->syms = elf_symbols_load(cfg->elf_path);
        if (s->syms)
            fprintf(stderr, "Loaded %d symbols from %s\n",
                    elf_symbols_count(s->syms), cfg->elf_path);
        else
            fprintf(stderr, "Warning: failed to load symbols from %s\n",
                    cfg->elf_path);
    }

    /* Create memory */
    s->mem = mem_create();
    if (!s->mem) {
        fprintf(stderr, "flexe: failed to allocate memory\n");
        flexe_session_destroy(s);
        return NULL;
    }

    /* Create peripherals */
    s->periph = periph_create(s->mem);
    if (!s->periph) {
        fprintf(stderr, "flexe: failed to create peripherals\n");
        flexe_session_destroy(s);
        return NULL;
    }
    if (cfg->uart_cb)
        periph_set_uart_callback(s->periph, cfg->uart_cb, cfg->uart_ctx);
    if (s->touch_fn && s->touch_irq_pin >= 0)
        periph_gpio_set_input(s->periph, s->touch_irq_pin, 1);

    /* Load firmware */
    load_result_t res = loader_load_bin(s->mem, cfg->bin_path);
    if (res.result != 0) {
        fprintf(stderr, "flexe: load error: %s\n", res.error);
        flexe_session_destroy(s);
        return NULL;
    }
    fprintf(stderr, "Loaded %s: %d segments, entry=0x%08X\n",
            cfg->bin_path, res.segment_count, res.entry_point);
    for (int i = 0; i < res.segment_count; i++) {
        fprintf(stderr, "  Segment %d: 0x%08X (%u bytes) -> %s\n",
                i, res.segments[i].addr, res.segments[i].size,
                loader_region_name(res.segments[i].addr));
    }

    /* Initialize CPU core 0 */
    xtensa_cpu_init(&s->cpu[0]);
    xtensa_cpu_reset(&s->cpu[0]);
    s->cpu[0].mem = s->mem;
    s->cpu[0].window_trace = cfg->window_trace;
    s->cpu[0].window_trace_active = cfg->window_trace;
    s->cpu[0].spill_verify = cfg->spill_verify;

    /* ROM function stubs */
    s->rom = rom_stubs_create(&s->cpu[0]);
    if (!s->rom) {
        fprintf(stderr, "flexe: failed to create ROM stubs\n");
        flexe_session_destroy(s);
        return NULL;
    }
    rom_stubs_set_single_core(s->rom, cfg->single_core);
    rom_stubs_set_native_freertos(s->rom, cfg->native_freertos);
    rom_stubs_set_periph(s->rom, s->periph);
    rom_stubs_hook_firmware_addrs(s->rom, res.entry_point);
    if (s->syms)
        rom_stubs_hook_symbols(s->rom, s->syms);

    /* FreeRTOS stubs — skip in native mode (firmware runs its own FreeRTOS) */
    if (!cfg->native_freertos) {
        s->frt = freertos_stubs_create(&s->cpu[0]);
        if (s->frt && s->syms)
            freertos_stubs_hook_symbols(s->frt, s->syms);
    }

    /* esp_timer stubs */
    s->etimer = esp_timer_stubs_create(&s->cpu[0]);
    if (s->etimer) {
        if (cfg->native_freertos)
            esp_timer_stubs_set_virtual_time(s->etimer, 1);
        if (s->frt)
            esp_timer_stubs_set_sleep_fn(s->etimer, session_sleep_us, s->frt);
        if (s->syms)
            esp_timer_stubs_hook_symbols(s->etimer, s->syms);
    }

    /* Display stubs */
    s->dstubs = display_stubs_create(&s->cpu[0]);
    if (s->dstubs) {
        if (cfg->framebuf)
            display_stubs_set_framebuf(s->dstubs, cfg->framebuf,
                                        cfg->framebuf_mutex,
                                        cfg->framebuf_w, cfg->framebuf_h);
        if (s->syms) {
            display_stubs_hook_symbols(s->dstubs, s->syms);
            display_stubs_hook_tft_espi(s->dstubs, s->syms);
            display_stubs_hook_tft_esprite(s->dstubs, s->syms);
            display_stubs_hook_ofr(s->dstubs, s->syms);
            display_stubs_hook_lvgl(s->dstubs, s->syms);
        }
    }

    /* Touch stubs */
    s->tstubs = touch_stubs_create(&s->cpu[0]);
    if (s->tstubs) {
        if (cfg->touch_fn)            touch_stubs_set_state_fn(s->tstubs, cfg->touch_fn, cfg->touch_ctx);
        if (s->syms)
            touch_stubs_hook_symbols(s->tstubs, s->syms);
    }

    /* Raw SPI display/touch capture (SPI2/SPI3 sniffing for symbol-less
     * firmware). Pins default to CYD 2432S028R; -1 disables. */
    {
        spi_display_config_t scfg = {
            .dc_pin         = cfg->spi_dc_pin         ? cfg->spi_dc_pin         : 2,
            .display_cs_pin = cfg->spi_display_cs_pin ? cfg->spi_display_cs_pin : 15,
            .display_sck_pin = cfg->spi_display_sck_pin ? cfg->spi_display_sck_pin : 14,
            .touch_cs_pin   = cfg->spi_touch_cs_pin   ? cfg->spi_touch_cs_pin   : 33,
            .touch_sck_pin  = cfg->spi_touch_sck_pin  ? cfg->spi_touch_sck_pin  : 25,
            .sd_cs_pin      = cfg->spi_sd_cs_pin      ? cfg->spi_sd_cs_pin      : 5,
            .sd_sck_pin     = cfg->spi_sd_sck_pin     ? cfg->spi_sd_sck_pin     : 18,
            .sdcard_path    = cfg->sdcard_path,
            .framebuf       = cfg->framebuf,
            .framebuf_mtx   = cfg->framebuf_mutex,
            .fb_w           = cfg->framebuf_w,
            .fb_h           = cfg->framebuf_h,
            .touch_fn       = cfg->touch_fn,
            .touch_ctx      = cfg->touch_ctx,
        };
        if (scfg.dc_pin >= 0)
            periph_enable_spi_display(s->periph, &scfg);
    }

    /* SD card stubs */
    s->sstubs = sdcard_stubs_create(&s->cpu[0]);
    if (s->sstubs) {
        if (cfg->sdcard_path)
            sdcard_stubs_set_image(s->sstubs, cfg->sdcard_path);
        if (cfg->sdcard_size > 0)
            sdcard_stubs_set_size(s->sstubs, cfg->sdcard_size);
        if (cfg->sdcard_path)
            sdcard_stubs_attach_sdmmc(s->sstubs, s->periph);
        if (s->syms)
            sdcard_stubs_hook_symbols(s->sstubs, s->syms);
    }

    /* SHA hardware accelerator stubs */
    s->shstubs = sha_stubs_create(&s->cpu[0]);
    if (s->shstubs && s->syms)
        sha_stubs_hook_symbols(s->shstubs, s->syms);

    /* AES hardware accelerator stubs */
    s->astubs = aes_stubs_create(&s->cpu[0]);
    if (s->astubs && s->syms)
        aes_stubs_hook_symbols(s->astubs, s->syms);

    /* MPI (RSA) hardware accelerator stubs */
    s->mstubs = mpi_stubs_create(&s->cpu[0]);
    if (s->mstubs) {
        mpi_stubs_set_peripheral(s->mstubs, s->periph);
        if (s->syms)
            mpi_stubs_hook_symbols(s->mstubs, s->syms);
    }

    /* WiFi / lwip socket bridge */
    s->wstubs = wifi_stubs_create(&s->cpu[0]);
    if (s->wstubs) {
        wifi_stubs_hook_firmware_addrs(s->wstubs, res.entry_point);
        if (s->syms)
            wifi_stubs_hook_symbols(s->wstubs, s->syms);
    }

    /* VFS / SPIFFS / FATFS stubs (host-backed file I/O).
     * Hook AFTER rom_stubs so we override the rom_stubs ESP_FAIL stubs. */
    s->vstubs = vfs_stubs_create(&s->cpu[0]);
    if (s->vstubs && s->syms)
        vfs_stubs_hook_symbols(s->vstubs, s->syms);

    /* Bluetooth / NimBLE stubs */
    s->bstubs = bt_stubs_create(&s->cpu[0]);
    if (s->bstubs) {
        bt_stubs_hook_firmware_addrs(s->bstubs, res.entry_point);
        if (s->syms)
            bt_stubs_hook_symbols(s->bstubs, s->syms);
    }

    /* Pre-decode instruction memory for fast fetch */
    xtensa_predecode_build(&s->cpu[0]);

    if (getenv("FLEXE_PDCHK")) {
        uint32_t a = (uint32_t)strtoul(getenv("FLEXE_PDCHK"), NULL, 0);
        if (s->cpu[0].predecode && a >= PREDECODE_BASE &&
            a < PREDECODE_END) {
            uint32_t pk = s->cpu[0].predecode[a - PREDECODE_BASE];
            uint32_t insn;
            int il = xtensa_fetch(&s->cpu[0], a, &insn);
            fprintf(stderr, "[PDCHK] 0x%08X: predecode=%06X/%u fetch=%06X/%d page=%p flash_insn=%p off=0x%lX\n",
                    a, PREDECODE_INSN(pk), PREDECODE_ILEN(pk), insn, il,
                    (void *)s->mem->page_table[a >> 12], (void *)s->mem->flash_insn,
                    s->mem->page_table[a >> 12] ? (long)(s->mem->page_table[a >> 12] - s->mem->flash_insn) : -1L);
        } else {
            fprintf(stderr,
                    "[PDCHK] 0x%08X is outside the active predecode table\n",
                    a);
        }
    }

    /* Set entry point */
    if (cfg->entry_override != 0)
        s->cpu[0].pc = cfg->entry_override;
    else if (res.entry_point != 0)
        s->cpu[0].pc = res.entry_point;

    /* Set initial stack pointer */
    uint32_t sp = cfg->initial_sp ? cfg->initial_sp : 0x3FFE0000u;
    ar_write(&s->cpu[0], 1, sp);

    /* Initialize CPU core 1 */
    xtensa_cpu_init(&s->cpu[1]);
    xtensa_cpu_reset(&s->cpu[1]);
    s->cpu[1].mem = s->mem;
    s->cpu[1].predecode = s->cpu[0].predecode;  /* Share predecode table */
    s->cpu[1].core_id = 1;
    s->cpu[1].prid = 0xABAB;
    s->cpu[1].running = false;
    s->cpu[1].window_trace = cfg->window_trace;
    s->cpu[1].window_trace_active = false;
    s->cpu[1].spill_verify = cfg->spill_verify;
    /* Core 1 needs a valid stack before its boot entry runs (on hardware
     * the ROM startup provides one). Give it 16 KB below core 0's stack. */
    ar_write(&s->cpu[1], 1, sp - 0x4000);

    /* Attach CPUs to peripherals for interrupt delivery */
    periph_attach_cpus(s->periph, &s->cpu[0], &s->cpu[1]);

    /* Attach core 1 to FreeRTOS */
    if (!cfg->single_core && s->frt)
        freertos_stubs_attach_cpu(s->frt, 1, &s->cpu[1]);

    /* Share pc_hook infrastructure with core 1 */
    if (!cfg->single_core) {
        s->cpu[1].pc_hook = s->cpu[0].pc_hook;
        s->cpu[1].pc_hook_ctx = s->cpu[0].pc_hook_ctx;
        s->cpu[1].pc_hook_bitmap = s->cpu[0].pc_hook_bitmap;
    }

    /* Keep execution-engine ownership in the shared session so the CLI and
     * embedded frontends run identical code.  JIT is the default wherever a
     * backend exists; jit_init() returns NULL on unsupported hosts. */
    /* FLEXE_DISABLE_JIT is an operational escape hatch for embedded
     * frontends that do not expose the config flag on their own CLI. */
    if (!cfg->disable_jit && getenv("FLEXE_DISABLE_JIT") == NULL) {
        s->jit = jit_init();
        if (s->jit) {
            jit_install_hook(s->jit, &s->cpu[0]);
            if (!cfg->single_core)
                jit_install_hook(s->jit, &s->cpu[1]);
        }
    }

    return s;
}

void flexe_session_destroy(flexe_session_t *s)
{
    if (!s) return;
    jit_destroy(s->jit);
    /* Both cores share this large table; core 0 owns the allocation. */
    free(s->cpu[0].predecode);
    s->cpu[0].predecode = NULL;
    s->cpu[1].predecode = NULL;
    bt_stubs_destroy(s->bstubs);
    vfs_stubs_destroy(s->vstubs);
    wifi_stubs_destroy(s->wstubs);
    mpi_stubs_destroy(s->mstubs);
    aes_stubs_destroy(s->astubs);
    sha_stubs_destroy(s->shstubs);
    sdcard_stubs_destroy(s->sstubs);
    touch_stubs_destroy(s->tstubs);
    display_stubs_destroy(s->dstubs);
    esp_timer_stubs_destroy(s->etimer);
    freertos_stubs_destroy(s->frt);
    rom_stubs_destroy(s->rom);
    periph_destroy(s->periph);
    mem_destroy(s->mem);
    elf_symbols_destroy(s->syms);
    free(s);
}

/* ===== Accessors ===== */

xtensa_cpu_t *flexe_session_cpu(flexe_session_t *s, int core)
{
    if (!s || core < 0 || core > 1) return NULL;
    return &s->cpu[core];
}

xtensa_mem_t *flexe_session_mem(flexe_session_t *s)
{
    return s ? s->mem : NULL;
}

const elf_symbols_t *flexe_session_syms(const flexe_session_t *s)
{
    return s ? s->syms : NULL;
}

esp32_periph_t *flexe_session_periph(flexe_session_t *s)
{
    return s ? s->periph : NULL;
}

esp32_rom_stubs_t *flexe_session_rom(flexe_session_t *s)
{
    return s ? s->rom : NULL;
}

freertos_stubs_t *flexe_session_frt(flexe_session_t *s)
{
    return s ? s->frt : NULL;
}

display_stubs_t *flexe_session_display(flexe_session_t *s)
{
    return s ? s->dstubs : NULL;
}

wifi_stubs_t *flexe_session_wifi(flexe_session_t *s)
{
    return s ? s->wstubs : NULL;
}

bt_stubs_t *flexe_session_bt(flexe_session_t *s)
{
    return s ? s->bstubs : NULL;
}

int flexe_session_is_native_freertos(const flexe_session_t *s)
{
    return s ? s->native_freertos : 0;
}

jit_state_t *flexe_session_jit(flexe_session_t *s)
{
    return s ? s->jit : NULL;
}

int flexe_session_run_core(flexe_session_t *s, int core, int max_cycles)
{
    if (!s || core < 0 || core > 1 || max_cycles <= 0) return 0;
    if (core == 1 && s->single_core) return 0;
    if (s->jit)
        return jit_run(s->jit, &s->cpu[core], max_cycles);
    return xtensa_run(&s->cpu[core], max_cycles);
}

/* ===== Post-batch hook ===== */

void flexe_session_post_batch(flexe_session_t *s, int batch_size)
{
    if (!s) return;

    /* XPT2046 PENIRQ is active-low.  Sampling the frontend's touch callback
     * at batch boundaries turns a newly pressed host touch into the falling
     * GPIO edge expected by interrupt-gated production drivers.  Previously
     * each embedding had to remember to mirror this wire separately, and a
     * driver which sampled "released" once would never ask for SPI data
     * again. */
    if (s->touch_fn && s->touch_irq_pin >= 0) {
        int x = 0, y = 0;
        int level = s->touch_fn(&x, &y, s->touch_ctx) ? 0 : 1;
        if (level != s->touch_irq_level) {
            periph_gpio_set_input(s->periph, s->touch_irq_pin, level);
            s->touch_irq_level = level;
        }
    }

    /* Compatibility for frontends built against the older API which still
     * call xtensa_run(core0) directly.  Sample the batch boundary just like
     * jit_run() so hot core-0 PCs become native after the normal threshold. */
    if (s->jit && s->cpu[0].pc >= ESP32_FIRMWARE_INSN_ADDR_LOW &&
        s->cpu[0].pc < ESP32_INSN_ADDR_HIGH)
        (void)jit_get_block(s->jit, &s->cpu[0], s->cpu[0].pc);

    /* Preemptive timeslice check for core 0 — skip in native mode
     * where firmware's own tick ISR handles scheduling */
    if (s->frt && !s->native_freertos)
        freertos_stubs_check_preempt(s->frt);

    /* Dual-core: check if core 1 should start */
    if (!s->single_core && !s->cpu[1].running &&
        periph_app_cpu_released(s->periph) &&
        rom_stubs_app_cpu_boot_addr(s->rom) != 0) {
        s->cpu[1].pc = rom_stubs_app_cpu_boot_addr(s->rom);
        s->cpu[1].running = true;
        fprintf(stderr, "[%10llu] CORE1 started at 0x%08X\n",
                (unsigned long long)s->cpu[0].cycle_count, s->cpu[1].pc);
    }

    /* Dual-core: run core 1 batch (or poll scheduler if parked) */
    if (!s->single_core && s->cpu[1].pc != 0) {
        if (s->cpu[1].running) {
            flexe_session_run_core(s, 1, batch_size);
            if (s->frt && !s->native_freertos)
                freertos_stubs_check_preempt_core(s->frt, 1);
        } else if (s->frt && !s->native_freertos) {
            /* Core 1 is parked at stub_esp_startup_start_app_other_cores
             * waiting for an eligible task.  Poll the scheduler so it gets
             * resumed once a core-1 task becomes ready. */
            if (freertos_stubs_check_preempt_core(s->frt, 1))
                s->cpu[1].running = true;
        }

        /* Sync cycle counts: both cores share the same clock, so use the
         * maximum of the two.  In native mode, no fast-forward — time
         * advances only via ccount increments. */
        if (s->native_freertos) {
            /* Keep cores loosely in sync without fast-forward */
            if (s->cpu[1].cycle_count > s->cpu[0].cycle_count)
                s->cpu[0].cycle_count = s->cpu[1].cycle_count;
            else
                s->cpu[1].cycle_count = s->cpu[0].cycle_count;
        } else {
            if (s->cpu[1].cycle_count > s->cpu[0].cycle_count)
                s->cpu[0].cycle_count = s->cpu[1].cycle_count;
            else
                s->cpu[1].cycle_count = s->cpu[0].cycle_count;
            /* Publish the later of the two cores' virtual_time_us, and only
             * that. esp_timer's current_time_us() already reconciles it
             * against cycle_count at the *real* CPU frequency; recomputing it
             * here as cycle_count / 160 assumed a 160 MHz part and could move
             * guest-visible time backwards on a 240 MHz one. */
            uint64_t vt = s->cpu[0].virtual_time_us > s->cpu[1].virtual_time_us
                        ? s->cpu[0].virtual_time_us
                        : s->cpu[1].virtual_time_us;
            s->cpu[0].virtual_time_us = vt;
            s->cpu[1].virtual_time_us = vt;
        }
    }

    /* Dispatch any expired esp_timer callbacks (periodic + one-shot).
     * Run after cycle_count / virtual_time_us have been advanced by both
     * cores so the dispatcher sees the latest simulated time.  Without
     * this, periodic timers registered via esp_timer_start_periodic would
     * never fire during normal instruction execution — only when firmware
     * explicitly calls usleep/delay. */
    esp_timer_stubs_tick(s->etimer);
    /* Same for FreeRTOS software timers, which Flexe models directly rather
     * than running the guest's timer daemon. */
    freertos_stubs_tick(s->frt);
}

/* ===== Callback configuration ===== */

void flexe_session_set_rom_log_cb(flexe_session_t *s, rom_log_fn fn, void *ctx)
{
    if (s && s->rom)
        rom_stubs_set_log_callback(s->rom, fn, ctx);
}

void flexe_session_set_event_log(flexe_session_t *s, int enable)
{
    if (!s) return;
    if (s->wstubs)
        wifi_stubs_set_event_log(s->wstubs, enable);
    if (s->bstubs)
        bt_stubs_set_event_log(s->bstubs, enable);
}

void flexe_session_set_freertos_event_fn(flexe_session_t *s,
        freertos_event_fn fn, void *ctx)
{
    if (s && s->frt)
        freertos_stubs_set_event_fn(s->frt, fn, ctx);
}
