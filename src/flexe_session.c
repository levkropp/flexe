/*
 * flexe_session.c — Shared ESP32 emulator session implementation
 *
 * Single source of truth for stub module lifecycle, dual-core
 * management, and cycle synchronization.
 */

#include "flexe_session.h"
#include "memory.h"
#include "loader.h"
#include "rom_elf.h"
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

/* rtc.h: timer wake trigger, and the RESET_REASON for a deep-sleep wake. */
#define RTC_TIMER_WAKE_CAUSE        (1u << 3)
#define RTC_DEEPSLEEP_RESET_CAUSE   5u

struct flexe_session {
    xtensa_cpu_t       cpu[2];
    xtensa_mem_t      *mem;
    const flexe_target_desc_t *target;
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
    /* Everything needed to rebuild the machine on a software reset. The
     * caller owns the strings in here and must outlive the session, which is
     * already true of every frontend. */
    flexe_session_config_t cfg;
    char               bin_path[512];
    char               rom_elf_path[1024];
    loader_image_info_t image;
    uint32_t           entry_point;
    uint32_t           initial_sp;
    unsigned           resets;
    int                preserve_rtc_mem;
};

/* Let the esp_timer delay()/usleep() shims block on the FreeRTOS scheduler
 * instead of fast-forwarding the clock past pending work. */
static bool session_sleep_us(void *ctx, xtensa_cpu_t *cpu, uint64_t us)
{
    return freertos_stubs_sleep_us((freertos_stubs_t *)ctx, cpu, us);
}

static const flexe_target_desc_t *
session_resolve_target(const flexe_session_config_t *cfg,
                       loader_image_info_t *image_out)
{
    loader_image_info_t image;
    char error[256];
    if (loader_probe_bin(cfg->bin_path, &image, error, sizeof(error)) != 0) {
        fprintf(stderr, "flexe: load error: %s\n", error);
        return NULL;
    }

    const flexe_target_desc_t *target = flexe_target_by_id(image.target);
    if (!target) {
        fprintf(stderr, "flexe: no descriptor for detected target %d\n",
                image.target);
        return NULL;
    }
    if (cfg->target != FLEXE_TARGET_AUTO && cfg->target != target->id) {
        const flexe_target_desc_t *requested =
            flexe_target_by_id(cfg->target);
        fprintf(stderr,
                "flexe: image targets %s (chip ID 0x%04X), not requested "
                "target %s\n",
                target->display_name, target->image_chip_id,
                requested ? requested->display_name : "unknown");
        return NULL;
    }
    if (target->support_level == FLEXE_TARGET_UNAVAILABLE) {
        fprintf(stderr,
                "flexe: %s/%s image recognized (chip ID 0x%04X), but "
                "execution support is not implemented yet\n",
                target->display_name,
                target->core_generation == FLEXE_XTENSA_LX7 ? "LX7" :
                                                               "Xtensa",
                target->image_chip_id);
        return NULL;
    }
    if (target->support_level == FLEXE_TARGET_EXPERIMENTAL)
        fprintf(stderr, "flexe: warning: %s support is experimental\n",
                target->display_name);
    if (!(target->capabilities &
          FLEXE_TARGET_CAP_ESP32_CLASSIC_PERIPHERALS) &&
        !cfg->native_freertos) {
        fprintf(stderr,
                "flexe: experimental %s execution currently requires "
                "native FreeRTOS mode (-N)\n",
                target->display_name);
        return NULL;
    }
    if (image.flash_size > target->spi_mem.maximum_flash_size) {
        fprintf(stderr,
                "flexe: %s image requests %u MiB flash, above the target "
                "profile maximum of %u MiB\n",
                target->display_name, image.flash_size >> 20,
                target->spi_mem.maximum_flash_size >> 20);
        return NULL;
    }
    if (target->id == FLEXE_TARGET_ESP32S3 &&
        (!cfg->rom_elf_path || !*cfg->rom_elf_path)) {
        fprintf(stderr,
                "flexe: experimental ESP32-S3 execution requires a matching "
                "official ROM ELF (-R or FLEXE_ROM_ELF)\n");
        return NULL;
    }
    if (image_out) *image_out = image;
    return target;
}

/* Build every emulated subsystem on top of an already-created memory.
 * Shared by session creation and by the software-reset path, which has to
 * produce a machine indistinguishable from a cold boot. */
static int session_build(flexe_session_t *s)
{
    const flexe_session_config_t *cfg = &s->cfg;
    uint32_t rom_flash_data_addr = s->target->rom_flash.live_data_address;
    if (cfg->rom_elf_path && *cfg->rom_elf_path) {
        rom_elf_load_result_t rom_res = rom_elf_load(s->mem,
                                                     cfg->rom_elf_path);
        if (rom_res.result != 0) {
            fprintf(stderr, "flexe: ROM ELF load error: %s\n", rom_res.error);
            return -1;
        }
        rom_flash_data_addr = rom_res.rom_flash_data_addr;
        fprintf(stderr,
                "Loaded %s ROM %s: %u immutable sections (%u bytes), "
                "%u data images (%u bytes), %u interface sections "
                "(%u bytes)\n",
                mem_target(s->mem)->display_name, cfg->rom_elf_path,
                rom_res.sections_loaded,
                rom_res.bytes_loaded, rom_res.data_images_loaded,
                rom_res.data_image_bytes, rom_res.interface_sections_loaded,
                rom_res.interface_bytes_loaded);
    }

    /* Create peripherals */
    s->periph = periph_create(s->mem);
    if (!s->periph) {
        fprintf(stderr, "flexe: failed to create peripherals\n");
        return -1;
    }
    if (cfg->uart_cb)
        periph_set_uart_callback(s->periph, cfg->uart_cb, cfg->uart_ctx);
    if (cfg->usb_serial_jtag_cb)
        periph_set_usb_serial_jtag_callback(
            s->periph, cfg->usb_serial_jtag_cb,
            cfg->usb_serial_jtag_ctx);
    if (s->touch_fn && s->touch_irq_pin >= 0)
        periph_gpio_set_input(s->periph, s->touch_irq_pin, 1);

    /* Load firmware */
    load_result_t res = loader_load_bin_for_target(s->mem, cfg->bin_path,
                                                    cfg->target);
    if (res.result != 0) {
        fprintf(stderr, "flexe: load error: %s\n", res.error);
        return -1;
    }
    const flexe_target_desc_t *target = s->target;
    const bool classic_compat =
        (target->capabilities &
         FLEXE_TARGET_CAP_ESP32_CLASSIC_PERIPHERALS) != 0u;
    fprintf(stderr, "Loaded %s image %s: %d segments, entry=0x%08X\n",
            target ? target->display_name : "unknown", cfg->bin_path,
            res.segment_count, res.entry_point);
    for (int i = 0; i < res.segment_count; i++) {
        fprintf(stderr, "  Segment %d: 0x%08X (%u bytes) -> %s\n",
                i, res.segments[i].addr, res.segments[i].size,
                loader_region_name_for_target(target, res.segments[i].addr));
    }
    if ((target->capabilities & FLEXE_TARGET_CAP_ROM_FLASH_HANDOFF) &&
        mem_prepare_rom_flash(s->mem, rom_flash_data_addr,
                              res.image.flash_size) != 0) {
        fprintf(stderr,
                "flexe: cannot reconstruct the %s ROM flash handoff\n",
                target->display_name);
        return -1;
    }

    /* Initialize CPU core 0 */
    xtensa_cpu_reset_for_target(&s->cpu[0], target);
    s->cpu[0].mem = s->mem;
    s->cpu[0].window_trace = cfg->window_trace;
    s->cpu[0].window_trace_active = cfg->window_trace;
    s->cpu[0].spill_verify = cfg->spill_verify;

    /* ROM function stubs */
    s->rom = rom_stubs_create(&s->cpu[0]);
    if (!s->rom) {
        fprintf(stderr, "flexe: failed to create ROM stubs\n");
        return -1;
    }
    rom_stubs_set_single_core(s->rom, cfg->single_core);
    rom_stubs_set_native_freertos(s->rom, cfg->native_freertos);
    rom_stubs_set_periph(s->rom, s->periph);
    rom_stubs_set_real_rom(s->rom,
                           cfg->rom_elf_path && *cfg->rom_elf_path);

    /* The compatibility providers below implement the classic ESP32 ABI and
     * several classic peripheral shortcuts. A different target may export
     * identical symbol names at unrelated addresses, so composing them based
     * on an ELF alone corrupts otherwise valid guest execution. Target
     * capabilities, rather than firmware identity, own this boundary. */
    if (classic_compat) {
        rom_stubs_hook_firmware_addrs(s->rom, res.entry_point);
        if (s->syms)
            rom_stubs_hook_symbols(s->rom, s->syms);

        /* FreeRTOS stubs — skip in native mode (firmware runs its own FreeRTOS) */
        if (!cfg->native_freertos) {
            s->frt = freertos_stubs_create(&s->cpu[0]);
            if (s->frt && s->syms)
                freertos_stubs_hook_symbols(s->frt, s->syms);
        }
    }

    /* esp_timer stubs */
    s->etimer = classic_compat ? esp_timer_stubs_create(&s->cpu[0]) : NULL;
    if (s->etimer) {
        if (cfg->native_freertos)
            esp_timer_stubs_set_virtual_time(s->etimer, 1);
        if (s->frt)
            esp_timer_stubs_set_sleep_fn(s->etimer, session_sleep_us,
                                         s->frt);
        if (s->syms)
            esp_timer_stubs_hook_symbols(s->etimer, s->syms);
        esp_timer_stubs_hook_firmware(s->etimer);
    }

    /* Display stubs */
    s->dstubs = classic_compat ? display_stubs_create(&s->cpu[0]) : NULL;
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
    s->tstubs = classic_compat ? touch_stubs_create(&s->cpu[0]) : NULL;
    if (s->tstubs) {
        if (cfg->touch_fn)
            touch_stubs_set_state_fn(s->tstubs, cfg->touch_fn,
                                     cfg->touch_ctx);
        if (s->syms)
            touch_stubs_hook_symbols(s->tstubs, s->syms);
    }

    /* GP-SPI controllers are constructed from the target descriptor even in
     * headless sessions. This optional layer only attaches board-side devices.
     * Classic compatibility keeps its historical board-profile defaults;
     * experimental targets require explicit positive pin numbers. */
    if (target->capabilities & FLEXE_TARGET_CAP_GP_SPI) {
        bool openhasp_lanbon = classic_compat &&
            rom_stubs_firmware_profile(s->rom) ==
                ROM_FIRMWARE_OPENHASP_V070RC13_LANBON_L8;
        spi_display_config_t scfg = {
            .dc_pin = classic_compat ?
                (cfg->spi_dc_pin ? cfg->spi_dc_pin :
                 (openhasp_lanbon ? 21 : 2)) :
                (cfg->spi_dc_pin > 0 ? cfg->spi_dc_pin : -1),
            .display_cs_pin = classic_compat ?
                (cfg->spi_display_cs_pin ? cfg->spi_display_cs_pin :
                 (openhasp_lanbon ? 22 : 15)) :
                (cfg->spi_display_cs_pin > 0 ?
                 cfg->spi_display_cs_pin : -1),
            .display_sck_pin = classic_compat ?
                (cfg->spi_display_sck_pin ? cfg->spi_display_sck_pin :
                 (openhasp_lanbon ? 19 : 14)) :
                (cfg->spi_display_sck_pin > 0 ?
                 cfg->spi_display_sck_pin : -1),
            .touch_cs_pin = classic_compat ?
                (cfg->spi_touch_cs_pin ? cfg->spi_touch_cs_pin :
                 (openhasp_lanbon ? -1 : 33)) :
                (cfg->spi_touch_cs_pin > 0 ? cfg->spi_touch_cs_pin : -1),
            .touch_sck_pin = classic_compat ?
                (cfg->spi_touch_sck_pin ? cfg->spi_touch_sck_pin :
                 (openhasp_lanbon ? -1 : 25)) :
                (cfg->spi_touch_sck_pin > 0 ? cfg->spi_touch_sck_pin : -1),
            .touch_mosi_pin = classic_compat ?
                (cfg->spi_touch_mosi_pin ? cfg->spi_touch_mosi_pin :
                 (openhasp_lanbon ? -1 : 32)) :
                (cfg->spi_touch_mosi_pin > 0 ?
                 cfg->spi_touch_mosi_pin : -1),
            .touch_miso_pin = classic_compat ?
                (cfg->spi_touch_miso_pin ? cfg->spi_touch_miso_pin :
                 (openhasp_lanbon ? -1 : 39)) :
                (cfg->spi_touch_miso_pin > 0 ?
                 cfg->spi_touch_miso_pin : -1),
            .sd_cs_pin = classic_compat ?
                (cfg->spi_sd_cs_pin ? cfg->spi_sd_cs_pin :
                 (openhasp_lanbon ? -1 : 5)) :
                (cfg->spi_sd_cs_pin > 0 ? cfg->spi_sd_cs_pin : -1),
            .sd_sck_pin = classic_compat ?
                (cfg->spi_sd_sck_pin ? cfg->spi_sd_sck_pin :
                 (openhasp_lanbon ? -1 : 18)) :
                (cfg->spi_sd_sck_pin > 0 ? cfg->spi_sd_sck_pin : -1),
            .sdcard_path    = cfg->sdcard_path,
            .framebuf       = cfg->framebuf,
            .framebuf_mtx   = cfg->framebuf_mutex,
            .fb_w           = cfg->framebuf_w,
            .fb_h           = cfg->framebuf_h,
            .touch_fn       = cfg->touch_fn,
            .touch_ctx      = cfg->touch_ctx,
        };
        periph_enable_spi_display(s->periph, &scfg);
    }

    /* SD card stubs */
    s->sstubs = classic_compat ? sdcard_stubs_create(&s->cpu[0]) : NULL;
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

    /* Target-described SHA hardware accelerator. Classic compatibility also
     * enables optional, validated software/HAL hooks; newer targets execute
     * their real MMIO and GDMA path so stripped images behave identically. */
    s->shstubs = (target->capabilities & FLEXE_TARGET_CAP_SHA_V1)
        ? sha_stubs_create(&s->cpu[0], periph_gdma(s->periph)) : NULL;
    if (s->shstubs) {
        if (classic_compat) {
            sha_stubs_hook_firmware(s->shstubs);
            if (s->syms)
                sha_stubs_hook_symbols(s->shstubs, s->syms);
        }
    }

    /* AES hardware accelerator stubs */
    s->astubs = classic_compat ? aes_stubs_create(&s->cpu[0]) : NULL;
    if (s->astubs && s->syms)
        aes_stubs_hook_symbols(s->astubs, s->syms);

    /* MPI (RSA) hardware accelerator stubs */
    s->mstubs = classic_compat ? mpi_stubs_create(&s->cpu[0]) : NULL;
    if (s->mstubs) {
        mpi_stubs_set_peripheral(s->mstubs, s->periph);
        if (s->syms)
            mpi_stubs_hook_symbols(s->mstubs, s->syms);
    }

    /* WiFi / lwip socket bridge */
    s->wstubs = classic_compat ? wifi_stubs_create(&s->cpu[0]) : NULL;
    if (s->wstubs) {
        wifi_stubs_hook_firmware(s->wstubs, res.entry_point);
        if (s->syms)
            wifi_stubs_hook_symbols(s->wstubs, s->syms);
    }

    /* VFS / SPIFFS / FATFS stubs (host-backed file I/O).
     * Hook AFTER rom_stubs so we override the rom_stubs ESP_FAIL stubs. */
    s->vstubs = classic_compat ? vfs_stubs_create(&s->cpu[0]) : NULL;
    if (s->vstubs && s->syms)
        vfs_stubs_hook_symbols(s->vstubs, s->syms);

    /* Bluetooth / NimBLE stubs */
    s->bstubs = classic_compat ? bt_stubs_create(&s->cpu[0]) : NULL;
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

    /* PS.EXCM is set out of reset, and on hardware the second-stage
     * bootloader has cleared it long before the app entry runs. Flexe loads
     * the app on its own and jumps straight there, so without this the bit
     * survives into the application -- and everything downstream of it
     * behaves as though an exception were in progress. Concretely, a RETW
     * needing a window fill is diverted away from the guest's own
     * WindowUnderflow vector, because raising there would be a double
     * exception; Tasmota takes that diversion from cycle 165 onward and the
     * synthesized fill reads a link slot nothing ever wrote, at 0xFFFFFFE0.
     *
     * INTLEVEL stays at 15. The app lowers it itself once its vectors are
     * installed, and unmasking before then would deliver an interrupt to
     * whatever the ROM left at VECBASE. */
    XT_PS_SET_EXCM(s->cpu[0].ps, 0);

    /* Set initial stack pointer */
    uint32_t sp = cfg->initial_sp ? cfg->initial_sp :
                  flexe_target_bootstrap_stack(target, 0);
    ar_write(&s->cpu[0], 1, sp);
    s->cpu[0].seed_entry_link = true;

    /* Initialize CPU core 1 */
    xtensa_cpu_reset_for_target(&s->cpu[1], target);
    s->cpu[1].mem = s->mem;
    s->cpu[1].predecode = s->cpu[0].predecode;  /* Share predecode table */
    s->cpu[1].core_id = 1;
    s->cpu[1].prid = 0xABAB;
    XT_PS_SET_EXCM(s->cpu[1].ps, 0);   /* see core 0, above */
    s->cpu[1].running = false;
    s->cpu[1].window_trace = cfg->window_trace;
    s->cpu[1].window_trace_active = false;
    s->cpu[1].spill_verify = cfg->spill_verify;
    memcpy(s->cpu[1].poll_spin_pc, s->cpu[0].poll_spin_pc,
           sizeof(s->cpu[1].poll_spin_pc));
    s->cpu[1].poll_spin_count = s->cpu[0].poll_spin_count;
    s->cpu[1].poll_spin_insns = s->cpu[0].poll_spin_insns;
    /* Core 1 needs a valid stack before its boot entry runs: APP CPU startup
     * begins with ENTRY and does not establish one first. Keep both defaults
     * in the target descriptor because the safe internal-RAM windows differ
     * between LX6 ESP32 and LX7 ESP32-S3. The classic values deliberately put
     * core 1 above core 0, outside the application heap; this prevents the
     * Meshtastic startup corruption that originally exposed the requirement. */
    uint32_t sp1 = cfg->initial_sp ? sp + 0x8000u :
                   flexe_target_bootstrap_stack(target, 1);
    ar_write(&s->cpu[1], 1, sp1);
    s->cpu[1].seed_entry_link = true;

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
        s->cpu[1].pc_hook_contains = s->cpu[0].pc_hook_contains;
        s->cpu[1].pc_hook_contains_ctx = s->cpu[0].pc_hook_contains_ctx;
        /* Address hooks are shared by both cores. Some conditionally consume
         * a complete verified firmware path and return its guest-instruction
         * span, so APP_CPU must use the same exact-work batch accounting as
         * PRO_CPU even before a JIT backend is installed. */
        s->cpu[1].accelerated_blocks = s->cpu[0].accelerated_blocks;
    }

    /* Keep execution-engine ownership in the shared session so the CLI and
     * embedded frontends run identical code.  JIT is the default wherever a
     * backend exists; jit_init() returns NULL on unsupported hosts. */
    /* FLEXE_DISABLE_JIT is an operational escape hatch for embedded
     * frontends that do not expose the config flag on their own CLI. */
    if (classic_compat && !cfg->disable_jit &&
        getenv("FLEXE_DISABLE_JIT") == NULL) {
        s->jit = jit_init();
        if (s->jit) {
            jit_install_hook(s->jit, &s->cpu[0]);
            if (!cfg->single_core)
                jit_install_hook(s->jit, &s->cpu[1]);
        }
    } else if (!classic_compat && !cfg->disable_jit &&
               getenv("FLEXE_DISABLE_JIT") == NULL) {
        fprintf(stderr,
                "flexe: %s JIT is not enabled yet; using the interpreter\n",
                target->display_name);
    }

    return 0;
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

    s->cfg = *cfg;
    snprintf(s->bin_path, sizeof(s->bin_path), "%s", cfg->bin_path);
    s->cfg.bin_path = s->bin_path;
    const char *rom_elf_path = cfg->rom_elf_path;
    if (!rom_elf_path) rom_elf_path = getenv("FLEXE_ROM_ELF");
    if (rom_elf_path && *rom_elf_path) {
        snprintf(s->rom_elf_path, sizeof(s->rom_elf_path), "%s", rom_elf_path);
        s->cfg.rom_elf_path = s->rom_elf_path;
    } else {
        s->cfg.rom_elf_path = NULL;
    }

    s->target = session_resolve_target(&s->cfg, &s->image);
    if (!s->target) {
        flexe_session_destroy(s);
        return NULL;
    }

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
    s->mem = mem_create_for_target_with_flash(s->target,
                                               s->image.flash_size);
    if (!s->mem) {
        fprintf(stderr, "flexe: failed to allocate memory\n");
        flexe_session_destroy(s);
        return NULL;
    }

    if (session_build(s) != 0) {
        flexe_session_destroy(s);
        return NULL;
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


/* Software reset: rebuild the machine as a reboot would.
 *
 * Firmware reboots itself by writing SW_PROCPU_RST to RTC_CNTL_OPTIONS0 and
 * spinning in a `while (1)` until the reset takes. Flexe used to discard that
 * write, so the guest span in that loop forever -- on NerdMiner, after
 * WiFiManager saves credentials, which is why it never got as far as creating
 * its mining tasks.
 *
 * Every subsystem is torn down and rebuilt, because a fresh boot must see a
 * cold machine: half-reset state (peripheral registers mid-transaction, open
 * sockets, a scheduler mid-switch) makes the firmware crash on the way back
 * up. One thing survives, deliberately: the memory object -- so flash, and
 * with it NVS and SPIFFS, persists exactly as across a real reboot, which is
 * the entire reason firmware reboots itself.
 */
/* RTC domains, which a deep-sleep wake must preserve. Everything else is
 * rebuilt: that is the point of the reset. */
#define RTC_SLOW_BASE  0x50000000u
#define RTC_SLOW_SIZE  0x2000u
#define RTC_DRAM_BASE  0x3FF80000u
#define RTC_DRAM_SIZE  0x2000u

void flexe_session_reset(flexe_session_t *s)
{
    if (!s) return;
    uint64_t cycles = s->cpu[0].cycle_count;

    /* A pad held with rtc_gpio_hold_en() keeps its level across the reset;
     * see periph_pad_hold_snapshot(). */
    periph_pad_hold_t pad_hold;
    periph_pad_hold_snapshot(s->periph, &pad_hold);

    /* A deep-sleep wake keeps RTC memory -- RTC_DATA_ATTR variables and the
     * wake stub live there, and firmware counts on them surviving. The reload
     * inside session_build() writes the image's RTC segments back over them,
     * which is right for a cold boot and wrong here, so snapshot and restore. */
    uint8_t *rtc_slow = NULL, *rtc_dram = NULL;
    if (s->preserve_rtc_mem) {
        rtc_slow = malloc(RTC_SLOW_SIZE);
        rtc_dram = malloc(RTC_DRAM_SIZE);
        for (uint32_t i = 0; rtc_slow && i < RTC_SLOW_SIZE; i++)
            rtc_slow[i] = mem_read8(s->mem, RTC_SLOW_BASE + i);
        for (uint32_t i = 0; rtc_dram && i < RTC_DRAM_SIZE; i++)
            rtc_dram[i] = mem_read8(s->mem, RTC_DRAM_BASE + i);
    }
    uint32_t *predecode = s->cpu[0].predecode;
    /* Host-supplied network settings are configuration, not machine state:
     * they describe the world the device is plugged into and must survive a
     * reboot, or the firmware comes back up unable to reach anything. */
    wifi_host_config_t netcfg;
    wifi_stubs_snapshot_host_config(s->wstubs, &netcfg);

    bt_stubs_destroy(s->bstubs);      s->bstubs = NULL;
    vfs_stubs_destroy(s->vstubs);     s->vstubs = NULL;
    wifi_stubs_destroy(s->wstubs);    s->wstubs = NULL;
    mpi_stubs_destroy(s->mstubs);     s->mstubs = NULL;
    aes_stubs_destroy(s->astubs);     s->astubs = NULL;
    sha_stubs_destroy(s->shstubs);    s->shstubs = NULL;
    sdcard_stubs_destroy(s->sstubs);  s->sstubs = NULL;
    touch_stubs_destroy(s->tstubs);   s->tstubs = NULL;
    display_stubs_destroy(s->dstubs); s->dstubs = NULL;
    esp_timer_stubs_destroy(s->etimer); s->etimer = NULL;
    freertos_stubs_destroy(s->frt);   s->frt = NULL;
    rom_stubs_destroy(s->rom);        s->rom = NULL;
    periph_destroy(s->periph);        s->periph = NULL;

    /* session_build() allocates its own predecode table. */
    s->cpu[0].predecode = NULL;
    s->cpu[1].predecode = NULL;
    free(predecode);

    /* session_build() creates the JIT and installs its hook on both cores,
     * so the old one has to go: keeping it leaves the CPUs hooked to a JIT
     * the session no longer points at, which runs but is ruinously slow.
     *
     * Verification is a property of the run, not of the JIT instance, so it
     * has to be carried over -- it was silently switched off by every reboot.
     * NerdMiner reboots partway through its own scenario, so most of that run
     * was unverified while still reporting no mismatches. */
    bool was_verifying = jit_verify_enabled(s->jit);
    jit_destroy(s->jit);
    s->jit = NULL;
    if (session_build(s) != 0) {
        fprintf(stderr, "[reset] rebuild failed; halting\n");
        s->cpu[0].running = false;
        s->cpu[1].running = false;
        return;
    }
    wifi_stubs_apply_host_config(s->wstubs, &netcfg);
    periph_pad_hold_restore(s->periph, &pad_hold);
    if (was_verifying) jit_set_verify(s->jit, true);

    if (s->preserve_rtc_mem) {
        for (uint32_t i = 0; rtc_slow && i < RTC_SLOW_SIZE; i++)
            mem_write8(s->mem, RTC_SLOW_BASE + i, rtc_slow[i]);
        for (uint32_t i = 0; rtc_dram && i < RTC_DRAM_SIZE; i++)
            mem_write8(s->mem, RTC_DRAM_BASE + i, rtc_dram[i]);
        s->preserve_rtc_mem = 0;
    }
    free(rtc_slow);
    free(rtc_dram);

    /* Keep the clock monotonic: emulator budgets and harness deadlines are
     * all measured against it, and a reboot does not rewind wall time here. */
    s->cpu[0].cycle_count = cycles;
    s->cpu[1].cycle_count = cycles;
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

unsigned flexe_session_reset_count(const flexe_session_t *s)
{
    return s ? s->resets : 0u;
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

    /* Dual-core: check if core 1 should start. Newer targets expose the boot
     * address through their hardware controller; classic ESP32 currently
     * obtains the same value from its ROM service shim. */
    uint32_t app_cpu_boot_addr = periph_app_cpu_boot_addr(s->periph);
    if (app_cpu_boot_addr == 0u)
        app_cpu_boot_addr = rom_stubs_app_cpu_boot_addr(s->rom);
    if (!s->single_core && !s->cpu[1].running &&
        periph_app_cpu_released(s->periph) &&
        app_cpu_boot_addr != 0u) {
        s->cpu[1].pc = app_cpu_boot_addr;
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
            /* Publish one timeline to both cores — and hand the core that is
             * behind the CCOUNT to go with it.
             *
             * Taking the maximum of cycle_count and virtual_time_us alone moved
             * the lagging core's guest clock forward while its CCOUNT stood
             * still, and CCOUNT is what that core's own FreeRTOS tick is
             * scheduled against through CCOMPARE0. Every tick inside the
             * injected interval was skipped: Marauder took 2.65 s of time this
             * way across a 36 s run and its guest tick ran at 925 Hz instead of
             * 1000 Hz, so every FreeRTOS timeout measured against it ran ~8%
             * long. Publish the time, then let each core walk its own CCOUNT up
             * to it, firing what falls inside.
             *
             * virtual_time_us is published as-is rather than recomputed from
             * cycle_count: esp_timer's current_time_us() already reconciles the
             * two at the real CPU frequency, and deriving it here as
             * cycle_count / 160 assumed a 160 MHz part and could move
             * guest-visible time backwards on a 240 MHz one. */
            uint32_t mhz = xtensa_cpu_freq_mhz(&s->cpu[0]);
            uint64_t was0 = s->cpu[0].cycle_count +
                            s->cpu[0].virtual_time_us * mhz;
            uint64_t was1 = s->cpu[1].cycle_count +
                            s->cpu[1].virtual_time_us * mhz;
            uint64_t cyc = s->cpu[0].cycle_count > s->cpu[1].cycle_count
                         ? s->cpu[0].cycle_count : s->cpu[1].cycle_count;
            uint64_t vt = s->cpu[0].virtual_time_us > s->cpu[1].virtual_time_us
                        ? s->cpu[0].virtual_time_us
                        : s->cpu[1].virtual_time_us;
            s->cpu[0].cycle_count = s->cpu[1].cycle_count = cyc;
            s->cpu[0].virtual_time_us = s->cpu[1].virtual_time_us = vt;
            uint64_t now = cyc + vt * mhz;
            if (now > was0) xtensa_advance_idle_cycles(&s->cpu[0], now - was0);
            if (now > was1) xtensa_advance_idle_cycles(&s->cpu[1], now - was1);
        }
    }

    /* Dispatch any expired esp_timer callbacks (periodic + one-shot).
     * Run after cycle_count / virtual_time_us have been advanced by both
     * cores so the dispatcher sees the latest simulated time.  Without
     * this, periodic timers registered via esp_timer_start_periodic would
     * never fire during normal instruction execution — only when firmware
     * explicitly calls usleep/delay. */
    /* Sleep. The peripheral decides that the chip is going to sleep and what
     * could wake it; the clock and the reset path live here.
     *
     * Time is stepped in slices rather than jumped, for two reasons: the
     * level-triggered wake sources (EXT0/EXT1/touch) have to be polled, and
     * xtensa_advance_idle_cycles() has to see the interval so the timers
     * inside it still fire. Deep sleep then takes the same reset the software
     * reset path uses -- RTC memory survives it because session_build()
     * reloads image segments into the existing memory, which is exactly
     * deep-sleep semantics. */
    {
        bool deep = false;
        uint64_t timeout_us = 0;
        uint32_t cause = 0;
        if (periph_take_sleep_request(s->periph, &deep, &timeout_us, &cause)) {
            fprintf(stderr, "[sleep] request deep=%d timeout_us=%llu cause=0x%X "
                    "cycles=%llu\n", deep, (unsigned long long)timeout_us, cause,
                    (unsigned long long)s->cpu[0].cycle_count);
            /* A sleep with nothing armed to end it would step forward for
             * ever. That is a firmware bug or a gap in what is modelled here;
             * either way, hanging the emulator is the worst way to report it. */
            if (timeout_us == PERIPH_SLEEP_FOREVER && cause == 0) {
                fprintf(stderr, "[sleep] no wake source armed; refusing to "
                        "sleep (wake_ena=0x%X)\n", cause);
                periph_finish_wake(s->periph, 0);
                return;
            }
            const uint64_t SLICE_US = 1000;
            uint32_t mhz = xtensa_cpu_freq_mhz(&s->cpu[0]);
            uint64_t slept = 0;
            while (cause == 0 && slept < timeout_us) {
                uint64_t step = timeout_us - slept;
                if (step > SLICE_US) step = SLICE_US;
                for (int c = 0; c < 2; c++) {
                    s->cpu[c].virtual_time_us += step;
                    xtensa_advance_idle_cycles(&s->cpu[c], step * mhz);
                }
                slept += step;
                cause = periph_sleep_poll_wake(s->periph);
            }
            if (cause == 0) cause = RTC_TIMER_WAKE_CAUSE;
            fprintf(stderr, "[sleep] %s sleep, woke after %llu us, cause=0x%X\n",
                    deep ? "deep" : "light", (unsigned long long)slept, cause);
            if (deep) {
                s->resets++;
                s->preserve_rtc_mem = 1;
                flexe_session_reset(s);
                periph_set_wake_state(s->periph, cause, RTC_DEEPSLEEP_RESET_CAUSE);
                return;
            }
            periph_finish_wake(s->periph, cause);
        }
    }

    if (periph_take_reset_request(s->periph)) {
        s->resets++;
        fprintf(stderr, "[reset] firmware requested a software reset (#%u)\n",
                s->resets);
        flexe_session_reset(s);
        return;
    }

    esp_timer_stubs_tick(s->etimer);
    /* Same for FreeRTOS software timers, which Flexe models directly rather
     * than running the guest's timer daemon. */
    freertos_stubs_tick(s->frt);
    /* And any queued WiFi/IP event, which likewise re-enters guest code. */
    wifi_stubs_tick(s->wstubs, &s->cpu[0], &s->cpu[1]);
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
