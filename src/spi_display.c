/* spi_display.c — GP-SPI (SPI2/SPI3) emulation with ILI9341 display
 * capture and XPT2046 touch response for symbol-less CYD firmware.
 *
 * Display libraries poke the GP-SPI register file directly (or via the
 * spi_master polling path — same registers). We model the register file,
 * complete every transaction instantly, and sniff the data stream:
 *
 *   - CS and D/C are plain GPIO outputs, tracked through the peripheral
 *     GPIO shadow (periph_gpio_pin_level).
 *   - Bytes clocked while display-CS is low are fed to an ILI9341 command
 *     interpreter that renders RAMWR pixel data into the framebuffer.
 *   - Bytes clocked while touch-CS is low are treated as XPT2046 requests;
 *     MISO is filled with coordinates derived from the session touch_fn.
 *
 * Polling transfers pass through W0..W15. DMA transfers walk the ESP32's
 * native lldesc chains and enter the same device models, so queued SPI-master
 * traffic behaves identically without requiring display-library hooks.
 */

#include "spi_display.h"
#include "memory.h"
#include "sandbox_events.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>

#define SPI2_BASE 0x3FF64000u
#define SPI3_BASE 0x3FF65000u

/* Register offsets (same layout as SPI0/SPI1) */
#define SPI_CMD_REG       0x00
#define SPI_ADDR_REG      0x04
#define SPI_USER_REG      0x1C
#define SPI_USER1_REG     0x20
#define SPI_USER2_REG     0x24
#define SPI_MOSI_DLEN_REG 0x28
#define SPI_MISO_DLEN_REG 0x2C
#define SPI_PIN_REG       0x34
#define SPI_SLAVE_REG     0x38
#define SPI_W0_REG        0x80
#define SPI_DMA_CONF_REG      0x100
#define SPI_DMA_OUT_LINK_REG  0x104
#define SPI_DMA_IN_LINK_REG   0x108
#define SPI_DMA_STATUS_REG    0x10C
#define SPI_DMA_INT_ENA_REG   0x110
#define SPI_DMA_INT_RAW_REG   0x114
#define SPI_DMA_INT_ST_REG    0x118
#define SPI_DMA_INT_CLR_REG   0x11C
#define SPI_IN_ERR_EOF_DES_REG 0x120
#define SPI_IN_SUC_EOF_DES_REG 0x124
#define SPI_INLINK_DSCR_REG    0x128
#define SPI_INLINK_DSCR_BF0_REG 0x12C
#define SPI_INLINK_DSCR_BF1_REG 0x130
#define SPI_OUT_EOF_BFR_DES_REG 0x134
#define SPI_OUT_EOF_DES_REG     0x138
#define SPI_OUTLINK_DSCR_REG     0x13C
#define SPI_OUTLINK_DSCR_BF0_REG 0x140
#define SPI_OUTLINK_DSCR_BF1_REG 0x144
#define SPI_DATE_REG             0x3FC

#define SPI_CMD_USR       (1u << 18)
#define SPI_TRANS_DONE    (1u << 4)
#define SPI_TRANS_INTEN   (1u << 9)
#define SPI_USER_USR_MOSI (1u << 27)
#define SPI_USER_USR_MISO (1u << 28)
#define SPI_USER_USR_ADDR (1u << 30)
#define SPI_USER_USR_CMD  (1u << 31)

#define SPI_DMA_IN_RST          (1u << 2)
#define SPI_DMA_OUT_RST         (1u << 3)
#define SPI_DMA_LINK_ADDR_MASK  0x000FFFFFu
#define SPI_DMA_LINK_STOP       (1u << 28)
#define SPI_DMA_LINK_START      (1u << 29)
#define SPI_DMA_INT_MASK        0x1FFu
#define SPI_DMA_INLINK_EMPTY    (1u << 0)
#define SPI_DMA_OUTLINK_ERROR   (1u << 1)
#define SPI_DMA_INLINK_ERROR    (1u << 2)
#define SPI_DMA_IN_DONE         (1u << 3)
#define SPI_DMA_IN_ERR_EOF      (1u << 4)
#define SPI_DMA_IN_SUC_EOF      (1u << 5)
#define SPI_DMA_OUT_DONE        (1u << 6)
#define SPI_DMA_OUT_EOF         (1u << 7)
#define SPI_DMA_OUT_TOTAL_EOF   (1u << 8)

#define SPI_DMA_DESC_OWNER      (1u << 31)
#define SPI_DMA_DESC_EOF        (1u << 30)
#define SPI_DMA_DESC_LENGTH_MASK 0x00FFF000u
#define SPI_DMA_DESC_LENGTH_SHIFT 12
#define SPI_DMA_DESC_SIZE_MASK  0x00000FFFu
#define SPI_DMA_MAX_DESCRIPTORS 1024
#define SPI_DMA_MAX_TRANSFER    (4u * 1024u * 1024u)

/* ILI9341 commands */
#define ILI_CASET 0x2A
#define ILI_PASET 0x2B
#define ILI_RAMWR 0x2C
#define ILI_MADCTL 0x36

/* Original ESP32 GPIO-matrix signal indices. */
#define HSPICLK_OUT_IDX  8
#define HSPICS0_OUT_IDX 11
#define HSPICS1_OUT_IDX 61
#define HSPICS2_OUT_IDX 62
#define VSPICLK_OUT_IDX 63
#define VSPICS0_OUT_IDX 68
#define VSPICS1_OUT_IDX 69
#define VSPICS2_OUT_IDX 70

#define HSPI_IOMUX_CLK_PIN 14
#define HSPI_IOMUX_CS0_PIN 15
#define VSPI_IOMUX_CLK_PIN 18
#define VSPI_IOMUX_CS0_PIN 5

typedef struct {
    xtensa_mem_t *mem;
    esp32_periph_t *periph;      /* for GPIO CS/D-C sampling */
    spi_display_config_t cfg;

    /* Register shadow */
    uint32_t addr, user, user1, user2, mosi_dlen, miso_dlen;
    uint32_t pin;                /* SPI_PIN_REG: active hardware CS line */
    uint32_t slave;              /* SPI_SLAVE_REG shadow (config bits) */
    int      trans_done;         /* SPI_TRANS_DONE flag (slave reg bit 4) */
    uint32_t w[16];

    /* Classic ESP32 GP-SPI DMA register file and descriptor progress. */
    uint32_t dma_conf;
    uint32_t dma_out_link;
    uint32_t dma_in_link;
    uint32_t dma_int_ena;
    uint32_t dma_int_raw;
    uint32_t dma_in_err_eof_desc;
    uint32_t dma_in_suc_eof_desc;
    uint32_t dma_inlink_dscr;
    uint32_t dma_inlink_dscr_bf0;
    uint32_t dma_inlink_dscr_bf1;
    uint32_t dma_out_eof_bfr_desc;
    uint32_t dma_out_eof_desc;
    uint32_t dma_outlink_dscr;
    uint32_t dma_outlink_dscr_bf0;
    uint32_t dma_outlink_dscr_bf1;

    /* ILI9341 state */
    uint8_t  cur_cmd;
    uint8_t  params[4];
    int      param_cnt;
    uint16_t xs, ys, xe, ye;     /* window */
    uint16_t cx, cy;             /* RAMWR cursor */
    uint8_t  madctl;
    int      pixel_mode;         /* consuming RAMWR data */
    int      pixel_phase;        /* hi/lo byte toggle */
    uint8_t  pixel_hi;
    uint32_t pixels_this_burst;

    /* XPT2046 state */
    uint8_t  touch_cmd;

    int      dc_seen;            /* D/C pin observed high on this host */
    int      host_num;           /* 2 = HSPI, 3 = VSPI */
    uint64_t route_snapshot;
    int      route_reported;

    /* Raw SPI SD card state (SDHC, SPI mode) */
    int      sd_fd;              /* backing image fd, -1 = not open */
    int      sd_fd_tried;
    int      sd_ready;           /* ACMD41 completed */
    uint8_t  sd_cmd[6];
    int      sd_cmd_len;
    uint8_t  sd_resp[520 + 4];   /* MISO queue (max: token + 512 data + crc) */
    int      sd_resp_len;
    int      sd_resp_pos;
    uint32_t sd_write_sector;
    uint8_t  sd_write_buf[512];
    int      sd_write_pos;
    uint32_t sd_multi_sector;    /* CMD18/25 ongoing read/write sector */
    int      sd_multi;           /* 1 = read multi, 2 = write multi */
} spi_display_t;

/* One instance per host controller */
static spi_display_t g_host[2];

static int gpio_level(const spi_display_t *s, int pin) {
    if (pin < 0) return -1;
    return periph_gpio_pin_level(s->periph, pin);
}

/* An inactive high level or an enabled output driver distinguishes a genuine
 * software chip select from an untouched reset pin whose latch also reads 0. */
static uint64_t g_cs_seen_high;   /* bit per GPIO number */

static int cs_seen_high(int pin) {
    return pin >= 0 && pin <= 39 &&
           (g_cs_seen_high & (1ULL << pin)) != 0;
}

static int cs_asserted(spi_display_t *s, int pin) {
    if (pin < 0 || pin > 39) return 0;
    int lvl = gpio_level(s, pin);
    if (lvl == 1) g_cs_seen_high |= (1ULL << pin);
    /* A driver may configure CS as an output and immediately assert it before
     * the first SPI transaction, so the sniffer never observes it high. */
    int asserted = lvl == 0 &&
                   (cs_seen_high(pin) ||
                    periph_gpio_output_enabled(s->periph, pin));
    if (asserted && getenv("FLEXE_CSDBG"))
        fprintf(stderr, "[CS] pin=%d asserted (touch_cs=%d sd_cs=%d)\n", pin, s->cfg.touch_cs_pin, s->cfg.sd_cs_pin);
    return asserted;
}

/* Return 1 when a device's configured clock pin is routed from this host,
 * 0 when it is explicitly routed from the other GP-SPI host, and -1 when
 * the firmware has not exposed enough matrix state to tell.  Treating an
 * unknown route as compatible preserves direct-register/native-IOMUX users;
 * an explicit route always prevents a GPIO asserted on SPI2 from stealing a
 * transaction running on SPI3 (and vice versa). */
static int clock_route(const spi_display_t *s, int pin) {
    int signal = periph_gpio_out_signal(s->periph, pin);
    int expected = s->host_num == 2 ? HSPICLK_OUT_IDX : VSPICLK_OUT_IDX;
    int other = s->host_num == 2 ? VSPICLK_OUT_IDX : HSPICLK_OUT_IDX;
    if (signal == expected) return 1;
    if (signal == other) return 0;
    /* ESP-IDF selects the native pin set through IO_MUX function 1 and never
     * writes GPIO_FUNC_OUT_SEL in that case. Recognize both classic host pin
     * sets so a native route is just as explicit as a matrix route. */
    if (periph_iomux_function(s->periph, pin) == 1) {
        if (pin == HSPI_IOMUX_CLK_PIN) return s->host_num == 2;
        if (pin == VSPI_IOMUX_CLK_PIN) return s->host_num == 3;
    }
    return -1;
}

static int route_allows_host(const spi_display_t *s, int clock_pin) {
    return clock_route(s, clock_pin) != 0;
}

/* If GPIO-matrix routing hands a configured CS pin to this host's CS0/1/2
 * output, the GPIO output latch no longer describes its electrical level.
 * SPI_PIN_REG selects exactly one hardware CS for each spi_master transfer;
 * return that state here, or -1 for a software-controlled/unknown CS. */
static int hardware_cs_state(const spi_display_t *s, int pin) {
    int signal = periph_gpio_out_signal(s->periph, pin);
    int cs = -1;
    if (s->host_num == 2) {
        if (signal == HSPICS0_OUT_IDX) cs = 0;
        else if (signal == HSPICS1_OUT_IDX) cs = 1;
        else if (signal == HSPICS2_OUT_IDX) cs = 2;
        else if (signal == VSPICS0_OUT_IDX || signal == VSPICS1_OUT_IDX ||
                 signal == VSPICS2_OUT_IDX)
            return 0;
    } else {
        if (signal == VSPICS0_OUT_IDX) cs = 0;
        else if (signal == VSPICS1_OUT_IDX) cs = 1;
        else if (signal == VSPICS2_OUT_IDX) cs = 2;
        else if (signal == HSPICS0_OUT_IDX || signal == HSPICS1_OUT_IDX ||
                 signal == HSPICS2_OUT_IDX)
            return 0;
    }
    if (cs < 0 && periph_iomux_function(s->periph, pin) == 1) {
        if (pin == HSPI_IOMUX_CS0_PIN)
            cs = s->host_num == 2 ? 0 : -2;
        else if (pin == VSPI_IOMUX_CS0_PIN)
            cs = s->host_num == 3 ? 0 : -2;
        if (cs == -2) return 0;
    }
    return cs < 0 ? -1 : ((s->pin & (1u << cs)) == 0);
}

static int device_cs_state(spi_display_t *s, int pin) {
    int state = hardware_cs_state(s, pin);
    return state >= 0 ? state : cs_asserted(s, pin);
}

static void report_routes(spi_display_t *s) {
    if (!getenv("FLEXE_SPIROUTEDBG")) return;
    int display_clk = periph_gpio_out_signal(s->periph, s->cfg.display_sck_pin);
    int touch_clk = periph_gpio_out_signal(s->periph, s->cfg.touch_sck_pin);
    int sd_clk = periph_gpio_out_signal(s->periph, s->cfg.sd_sck_pin);
    int display_cs = periph_gpio_out_signal(s->periph, s->cfg.display_cs_pin);
    int touch_cs = periph_gpio_out_signal(s->periph, s->cfg.touch_cs_pin);
    int display_iomux = periph_iomux_function(s->periph,
                                               s->cfg.display_sck_pin);
    int touch_iomux = periph_iomux_function(s->periph,
                                             s->cfg.touch_sck_pin);
    int sd_iomux = periph_iomux_function(s->periph, s->cfg.sd_sck_pin);
    int sd_cs = periph_gpio_out_signal(s->periph, s->cfg.sd_cs_pin);
    uint64_t snapshot = (uint64_t)(display_clk & 0x1FF) |
                        (uint64_t)(touch_clk & 0x1FF) << 9 |
                        (uint64_t)(sd_clk & 0x1FF) << 18 |
                        (uint64_t)(display_cs & 0x1FF) << 27 |
                        (uint64_t)(touch_cs & 0x1FF) << 36 |
                        (uint64_t)(sd_cs & 0x1FF) << 45;
    if (s->route_reported && snapshot == s->route_snapshot) return;
    s->route_reported = 1;
    s->route_snapshot = snapshot;
    fprintf(stderr,
                "[SPIROUTE] SPI%d clk(display=%d touch=%d sd=%d) "
                "iomux(display=%d touch=%d sd=%d) "
                "cs(display=%d touch=%d sd=%d) pin=0x%08X\n",
                s->host_num, display_clk, touch_clk, sd_clk,
                display_iomux, touch_iomux, sd_iomux,
                display_cs, touch_cs, sd_cs, s->pin);
}

/* Display CS is often driven by the SPI hardware (spi_master spics_io_num),
 * never touching the GPIO output shadow, so CS-based detection fails. The
 * D/C line however IS a plain GPIO and toggles on every command/data
 * switch, so once we've seen it move, all traffic on this host is treated
 * as display traffic (the CYD has the panel alone on its SPI host). */
static int display_active(spi_display_t *s) {
    int lvl = gpio_level(s, s->cfg.dc_pin);
    if (lvl == 1) s->dc_seen = 1;
    return s->dc_seen;
}

/* ---- helpers ---- */

static size_t phase_bytes(uint32_t dlen_reg) {
    uint32_t bits = (dlen_reg & 0xFFFFFFu) + 1u;
    return (bits + 7u) / 8u;
}

static void gp_spi_intr_update(spi_display_t *s) {
    bool active = (s->trans_done && (s->slave & SPI_TRANS_INTEN)) ||
                  ((s->dma_int_raw & s->dma_int_ena) != 0);
    int source = s->host_num == 2 ? 30 : 31;
    if (active)
        periph_assert_interrupt(s->periph, source);
    else
        periph_deassert_interrupt(s->periph, source);
}

static int dma_range_mapped(spi_display_t *s, uint32_t addr, size_t len,
                            bool writable) {
    while (len > 0) {
        size_t page_left = 0x1000u - (addr & 0xFFFu);
        size_t chunk = len < page_left ? len : page_left;
        const uint8_t *ptr = writable ? mem_get_ptr_w(s->mem, addr) :
                                        mem_get_ptr(s->mem, addr);
        if (!ptr) return 0;
        addr += (uint32_t)chunk;
        len -= chunk;
    }
    return 1;
}

static uint32_t dma_first_desc(uint32_t link) {
    /* Classic ESP32 stores only descriptor address bits [19:0]; DMA-capable
     * internal DRAM occupies the implicit 0x3FFxxxxx window. */
    return 0x3FF00000u | (link & SPI_DMA_LINK_ADDR_MASK);
}

static uint32_t dma_next_desc(uint32_t next) {
    /* lldesc.next normally contains a complete pointer. Accept the same
     * 20-bit form as the link register as a defensive convenience. */
    return next != 0 && next < 0x00100000u ? 0x3FF00000u | next : next;
}

static size_t gp_spi_dma_read_tx(spi_display_t *s, uint8_t *dst,
                                 size_t wanted) {
    uint32_t desc = dma_first_desc(s->dma_out_link);
    size_t copied = 0;
    bool saw_eof = false;
    bool error = false;

    for (int count = 0; count < SPI_DMA_MAX_DESCRIPTORS && copied < wanted;
         count++) {
        if (!dma_range_mapped(s, desc, 12, false)) {
            if (getenv("FLEXE_SPIDMADBG"))
                fprintf(stderr,
                        "[SPIDMA] SPI%d TX descriptor 0x%08X is unmapped "
                        "(OUT_LINK=0x%08X)\n",
                        s->host_num, desc, s->dma_out_link);
            error = true;
            break;
        }
        uint32_t ctrl = mem_read32(s->mem, desc);
        uint32_t buf = mem_read32(s->mem, desc + 4u);
        uint32_t next = dma_next_desc(mem_read32(s->mem, desc + 8u));
        size_t size = ctrl & SPI_DMA_DESC_SIZE_MASK;
        size_t len = (ctrl & SPI_DMA_DESC_LENGTH_MASK) >>
                     SPI_DMA_DESC_LENGTH_SHIFT;

        s->dma_outlink_dscr = desc;
        s->dma_outlink_dscr_bf0 = next;
        s->dma_outlink_dscr_bf1 = buf;

        if (!(ctrl & SPI_DMA_DESC_OWNER) || len > size ||
            (len != 0 && !dma_range_mapped(s, buf, len, false))) {
            if (getenv("FLEXE_SPIDMADBG"))
                fprintf(stderr,
                        "[SPIDMA] SPI%d invalid TX descriptor 0x%08X: "
                        "ctrl=0x%08X buf=0x%08X next=0x%08X\n",
                        s->host_num, desc, ctrl, buf, next);
            error = true;
            break;
        }

        size_t chunk = len;
        if (chunk > wanted - copied) chunk = wanted - copied;
        for (size_t i = 0; i < chunk; i++)
            dst[copied + i] = mem_read8(s->mem, buf + (uint32_t)i);
        copied += chunk;
        mem_write32(s->mem, desc, ctrl & ~SPI_DMA_DESC_OWNER);
        s->dma_int_raw |= SPI_DMA_OUT_DONE;

        if (ctrl & SPI_DMA_DESC_EOF) {
            saw_eof = true;
            s->dma_out_eof_bfr_desc = buf;
            s->dma_out_eof_desc = desc;
            s->dma_int_raw |= SPI_DMA_OUT_EOF;
        }
        if (copied >= wanted || saw_eof) break;
        if (next == 0 || next == desc) {
            error = true;
            break;
        }
        desc = next;
    }

    if (copied == wanted)
        s->dma_int_raw |= SPI_DMA_OUT_TOTAL_EOF;
    else
        error = true;
    if (error)
        s->dma_int_raw |= SPI_DMA_OUTLINK_ERROR;
    s->dma_out_link &= ~SPI_DMA_LINK_START;
    return copied;
}

static size_t gp_spi_dma_write_rx(spi_display_t *s, const uint8_t *src,
                                  size_t wanted) {
    uint32_t desc = dma_first_desc(s->dma_in_link);
    size_t copied = 0;
    bool error = false;

    for (int count = 0; count < SPI_DMA_MAX_DESCRIPTORS && copied < wanted;
         count++) {
        if (!dma_range_mapped(s, desc, 12, true)) {
            s->dma_in_err_eof_desc = desc;
            error = true;
            break;
        }
        uint32_t ctrl = mem_read32(s->mem, desc);
        uint32_t buf = mem_read32(s->mem, desc + 4u);
        uint32_t next = dma_next_desc(mem_read32(s->mem, desc + 8u));
        size_t size = ctrl & SPI_DMA_DESC_SIZE_MASK;

        s->dma_inlink_dscr = desc;
        s->dma_inlink_dscr_bf0 = next;
        s->dma_inlink_dscr_bf1 = buf;

        if (!(ctrl & SPI_DMA_DESC_OWNER) || size == 0 ||
            !dma_range_mapped(s, buf, size, true)) {
            s->dma_in_err_eof_desc = desc;
            error = true;
            break;
        }

        size_t chunk = size;
        if (chunk > wanted - copied) chunk = wanted - copied;
        for (size_t i = 0; i < chunk; i++)
            mem_write8(s->mem, buf + (uint32_t)i, src[copied + i]);
        copied += chunk;
        ctrl &= ~(SPI_DMA_DESC_OWNER | SPI_DMA_DESC_LENGTH_MASK);
        ctrl |= ((uint32_t)chunk << SPI_DMA_DESC_LENGTH_SHIFT) &
                SPI_DMA_DESC_LENGTH_MASK;
        mem_write32(s->mem, desc, ctrl);
        s->dma_int_raw |= SPI_DMA_IN_DONE;

        if (copied >= wanted) {
            s->dma_in_suc_eof_desc = desc;
            s->dma_int_raw |= SPI_DMA_IN_SUC_EOF;
            break;
        }
        if ((ctrl & SPI_DMA_DESC_EOF) || next == 0 || next == desc) {
            s->dma_in_err_eof_desc = desc;
            if (next == 0) s->dma_int_raw |= SPI_DMA_INLINK_EMPTY;
            error = true;
            break;
        }
        desc = next;
    }

    if (copied != wanted) error = true;
    if (error)
        s->dma_int_raw |= SPI_DMA_INLINK_ERROR | SPI_DMA_IN_ERR_EOF;
    s->dma_in_link &= ~SPI_DMA_LINK_START;
    return copied;
}

/* Map panel-native (x, y) to framebuffer coordinates.
 * Panel RAM is 240x320 portrait; the emulator window is 320x240 landscape.
 * With MADCTL MV clear the app streams 240x320 portrait content, which must
 * be rotated 90° to fill the landscape window. With MV set the app streams
 * 320x240 landscape content directly (the panel scans axes-swapped), so no
 * further rotation is applied — only the MX/MY flips. */
static void fb_write_pixel(spi_display_t *s, uint16_t x, uint16_t y, uint16_t rgb565) {
    if (!s->cfg.framebuf) return;
    uint32_t fx, fy;
    int rot;
    if (s->madctl & 0x20) rot = (s->madctl & 0x40) ? 3 : 1;   /* MV set */
    else                  rot = (s->madctl & 0x40) ? 2 : 0;
    switch (rot) {
    case 0:  fx = 319 - y;  fy = x;        break;  /* portrait, rotated CW */
    case 1:  fx = x;        fy = y;        break;  /* MV: landscape stream */
    case 2:  fx = y;        fy = 239 - x;  break;  /* portrait, rotated CCW */
    default: fx = 319 - x;  fy = y;        break;  /* MV + MX: landscape, col flip */
    }
    if (fx >= (uint32_t)s->cfg.fb_w || fy >= (uint32_t)s->cfg.fb_h) return;
    s->cfg.framebuf[fy * s->cfg.fb_w + fx] = rgb565;
    s->pixels_this_burst++;
}

/* ---- ILI9341 command stream ---- */

static void ili9341_param(spi_display_t *s, uint8_t b) {
    if (s->param_cnt < (int)sizeof(s->params))
        s->params[s->param_cnt] = b;
    s->param_cnt++;

    switch (s->cur_cmd) {
    case ILI_CASET:
        if (s->param_cnt == 4) {
            s->xs = (uint16_t)((s->params[0] << 8) | s->params[1]);
            s->xe = (uint16_t)((s->params[2] << 8) | s->params[3]);
            if (getenv("FLEXE_DISPG"))
                fprintf(stderr, "[DISPG] CASET xs=%u xe=%u (w=%u)\n", s->xs, s->xe, s->xe - s->xs + 1);
        }
        break;
    case ILI_PASET:
        if (s->param_cnt == 4) {
            s->ys = (uint16_t)((s->params[0] << 8) | s->params[1]);
            s->ye = (uint16_t)((s->params[2] << 8) | s->params[3]);
            if (getenv("FLEXE_DISPG"))
                fprintf(stderr, "[DISPG] PASET ys=%u ye=%u (h=%u)\n", s->ys, s->ye, s->ye - s->ys + 1);
        }
        break;
    case ILI_MADCTL:
        if (s->param_cnt == 1) {
            s->madctl = b;
            if (getenv("FLEXE_DISPG"))
                fprintf(stderr, "[DISPG] MADCTL=0x%02X (MY=%d MX=%d MV=%d BGR=%d)\n",
                        b, !!(b & 0x80), !!(b & 0x40), !!(b & 0x20), !!(b & 0x08));
        }
        break;
    default:
        break;   /* everything else: state we don't need */
    }
}

static void ili9341_pixel_byte(spi_display_t *s, uint8_t b) {
    if (s->pixel_phase == 0) {
        s->pixel_hi = b;
        s->pixel_phase = 1;
        return;
    }
    s->pixel_phase = 0;
    uint16_t rgb = (uint16_t)((s->pixel_hi << 8) | b);
    fb_write_pixel(s, s->cx, s->cy, rgb);
    /* Auto-increment with window wrap (panel behavior). The column/page RAM
     * bounds depend on MADCTL MV: with MV clear the panel scans 240 columns x
     * 320 pages (portrait); with MV set the axes swap, so a landscape window
     * legitimately addresses 320 columns x 240 pages. Hardcoding 240/320
     * truncates a 320-wide landscape row at column 240 and shears every
     * following row into vertical stripes. */
    uint16_t cmax = (s->madctl & 0x20) ? 320 : 240;   /* column RAM count */
    uint16_t pmax = (s->madctl & 0x20) ? 240 : 320;   /* page RAM count */
    s->cx++;
    if (s->cx > s->xe || s->cx >= cmax) {
        s->cx = s->xs;
        s->cy++;
        if (s->cy > s->ye || s->cy >= pmax) s->cy = s->ys;
    }
}

static void ili9341_feed(spi_display_t *s, int dc, const uint8_t *data, int len) {
    /* A GP-SPI transaction carries up to 32 RGB565 pixels. Lock the shared
     * framebuffer once for the whole burst instead of once per pixel: the
     * per-pixel lock let a continuously-redrawing firmware starve the SDL
     * render/control thread for minutes. */
    int lock_fb = dc && s->pixel_mode && s->cfg.framebuf && s->cfg.framebuf_mtx;
    if (lock_fb) pthread_mutex_lock(s->cfg.framebuf_mtx);
    for (int i = 0; i < len; i++) {
        if (!dc) {
            s->cur_cmd = data[i];
            s->param_cnt = 0;
            s->pixel_mode = (data[i] == ILI_RAMWR);
            if (s->pixel_mode) {
                s->cx = s->xs;
                s->cy = s->ys;
                s->pixel_phase = 0;
            }
        } else if (s->pixel_mode) {
            ili9341_pixel_byte(s, data[i]);
        } else {
            ili9341_param(s, data[i]);
        }
    }
    if (lock_fb) pthread_mutex_unlock(s->cfg.framebuf_mtx);
}

/* ---- XPT2046 touch ---- */

/* Current touch sample in panel coordinates (0..4095), or -1 if not pressed */
static void touch_sample(spi_display_t *s, int *rx, int *ry, int *pressed) {
    *pressed = 0;
    *rx = *ry = 0;
    if (!s->cfg.touch_fn) return;
    int x = 0, y = 0;
    if (!s->cfg.touch_fn(&x, &y, s->cfg.touch_ctx)) return;
    if (s->cfg.fb_w > 1 && s->cfg.fb_h > 1) {
        if (x < 0) x = 0;
        if (x >= s->cfg.fb_w) x = s->cfg.fb_w - 1;
        if (y < 0) y = 0;
        if (y >= s->cfg.fb_h) y = s->cfg.fb_h - 1;

        /* The 2432S028's LCD/touch glass is 240x320 portrait, while Flexe's
         * framebuffer presents that physical panel rotated clockwise as
         * 320x240.  Convert the host cursor back to panel coordinates, then
         * reproduce the raw ranges used by Marauder's CYD calibration.
         *
         * XPT command 0x91 (chan 1 below) supplies library p.x; 0xD1
         * (chan 5) supplies p.y.  The rx/ry names retain the controller's
         * electrical channel convention, which is opposite those names. */
        int panel_x = y * 239 / (s->cfg.fb_h - 1);
        int panel_y = (s->cfg.fb_w - 1 - x) * 319 / (s->cfg.fb_w - 1);
        *ry = 200 + panel_x * (3700 - 200) / 239; /* library p.x */
        *rx = 240 + panel_y * (3800 - 240) / 319; /* library p.y */
    }
    if (*rx < 0) *rx = 0;
    if (*rx > 4095) *rx = 4095;
    if (*ry < 0) *ry = 0;
    if (*ry > 4095) *ry = 4095;
    *pressed = 1;
}

/* Fill W0.. with the XPT2046 response for the latched command.
 * Channel select bits [6:4]: 1 = Y, 5 = X, 3/4 = Z1/Z2 pressure.
 * The 12-bit result is left-justified in the 16-bit frame (<<3). */
static void xpt2046_respond(spi_display_t *s) {
    int rx, ry, pressed;
    touch_sample(s, &rx, &ry, &pressed);
    int chan = (s->touch_cmd >> 4) & 0x7;
    uint16_t val = 0;
    switch (chan) {
    case 1: val = (uint16_t)ry; break;              /* 0x91: library p.x */
    case 5: val = (uint16_t)rx; break;              /* 0xD1: library p.y */
    /* The driver computes pressure as Z1 + 4095 - Z2.  Open-circuit
     * (released) values are therefore opposite rails, not both zero. */
    case 3: val = pressed ? 600 : 0; break;       /* Z1 */
    case 4: val = pressed ? 3500 : 4095; break;  /* Z2 */
    default: val = 0; break;
    }
    uint16_t frame = (uint16_t)(val << 3);
    /* Firmware reads W0 bytes [0]=hi, [1]=lo of the 16-bit frame */
    s->w[0] = (uint32_t)(((frame >> 8) & 0xFF) | ((frame & 0xFF) << 8));
}

/* ---- Raw SPI SD card (SDHC, SPI mode) ---- */

static int sd_image_open(spi_display_t *s) {
    if (s->sd_fd_tried) return s->sd_fd;
    s->sd_fd_tried = 1;
    s->sd_fd = -1;
    if (s->cfg.sdcard_path)
        s->sd_fd = open(s->cfg.sdcard_path, O_RDWR);
    return s->sd_fd;
}

static void sd_queue(spi_display_t *s, const uint8_t *data, int len) {
    if (s->sd_resp_len + len > (int)sizeof(s->sd_resp)) return;
    memcpy(s->sd_resp + s->sd_resp_len, data, (size_t)len);
    s->sd_resp_len += len;
}

static void sd_queue_byte(spi_display_t *s, uint8_t b) { sd_queue(s, &b, 1); }

static void sd_read_sector(spi_display_t *s, uint32_t lba, uint8_t *out) {
    memset(out, 0, 512);
    if (sd_image_open(s) >= 0) {
        off_t off = (off_t)lba * 512;
        ssize_t n = pread(s->sd_fd, out, 512, off);
        if (n < 0) memset(out, 0, 512);
        else if (n < 512) memset(out + n, 0, 512 - (size_t)n);
    }
}

static void sd_write_sector(spi_display_t *s, uint32_t lba, const uint8_t *in) {
    if (sd_image_open(s) < 0) return;
    /* Loop over short writes; the backing image is a regular file, so a
     * partial write means the sector would silently tear. */
    off_t off = (off_t)lba * 512;
    size_t done = 0;
    while (done < 512) {
        ssize_t n = pwrite(s->sd_fd, in + done, 512 - done, off + (off_t)done);
        if (n <= 0) break;
        done += (size_t)n;
    }
}

/* CSD v2 for a 4 GB SDHC card (C_SIZE = 8191) */
static const uint8_t SD_CSD[16] = {
    0x40, 0x0E, 0x00, 0x32, 0x5B, 0x59, 0x00, 0x00,
    0x1F, 0xFF, 0x80, 0x80, 0x0A, 0xC0, 0x40, 0x00
};

/* CRC16-CCITT (poly 0x1021, init 0) as used for SD data blocks */
static uint16_t sd_crc16(const uint8_t *data, int len) {
    uint16_t crc = 0;
    for (int i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int b = 0; b < 8; b++)
            crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
    }
    return crc;
}

static void sd_queue_block_read(spi_display_t *s, uint32_t lba) {
    uint8_t tok = 0xFE;
    sd_queue(s, &tok, 1);
    uint8_t blk[512];
    sd_read_sector(s, lba, blk);
    if (getenv("FLEXE_SDDBG"))
        fprintf(stderr, "[SD] read lba=%u data=%02X %02X %02X %02X\n",
                lba, blk[0], blk[1], blk[2], blk[3]);
    sd_queue(s, blk, 512);
    uint16_t crc = sd_crc16(blk, 512);
    uint8_t crcb[2] = { (uint8_t)(crc >> 8), (uint8_t)crc };
    sd_queue(s, crcb, 2);
}

static void sd_execute(spi_display_t *s) {
    uint8_t cmd = s->sd_cmd[0] & 0x3F;
    uint32_t arg = ((uint32_t)s->sd_cmd[1] << 24) | ((uint32_t)s->sd_cmd[2] << 16) |
                   ((uint32_t)s->sd_cmd[3] << 8) | s->sd_cmd[4];
    if (getenv("FLEXE_SDDBG"))
        fprintf(stderr, "[SD] cmd%u arg=0x%08X\n", cmd, arg);
    s->sd_resp_pos = 0;
    s->sd_resp_len = 0;
    switch (cmd) {
    case 0:  sd_queue_byte(s, 0x01); break;                       /* GO_IDLE */
    case 1:  s->sd_ready = 1; sd_queue_byte(s, 0x00); break;      /* SEND_OP_COND */
    case 8:  {                                                    /* SEND_IF_COND */
        static const uint8_t r7[] = {0x01, 0x00, 0x00, 0x01, 0xAA};
        sd_queue(s, r7, sizeof(r7));
        break;
    }
    case 9:  sd_queue_byte(s, 0x00); sd_queue_byte(s, 0xFE);      /* SEND_CSD */
             sd_queue(s, SD_CSD, 16); sd_queue_byte(s, 0xFF); sd_queue_byte(s, 0xFF); break;
    case 10: sd_queue_byte(s, 0x00); sd_queue_byte(s, 0xFE);      /* SEND_CID */
             { uint8_t cid[16] = {0}; sd_queue(s, cid, 16); }
             sd_queue_byte(s, 0xFF); sd_queue_byte(s, 0xFF); break;
    case 12: /* STOP_TRANS has one stuff byte before its R1 response. */
             sd_queue_byte(s, 0xFF); sd_queue_byte(s, 0x00);
             s->sd_multi = 0; break;
    case 13: { uint8_t r2[2] = {0x00, 0x00}; sd_queue(s, r2, 2); break; }
    case 16: sd_queue_byte(s, 0x00); break;                       /* SET_BLOCKLEN */
    case 17: sd_queue_byte(s, 0x00);                              /* READ_SINGLE */
             sd_queue_block_read(s, arg); break;
    case 18: sd_queue_byte(s, 0x00);                              /* READ_MULTI */
             s->sd_multi = 1; s->sd_multi_sector = arg;
             sd_queue_block_read(s, arg); break;
    case 23: sd_queue_byte(s, 0x00); break;                       /* ACMD23 */
    case 24: sd_queue_byte(s, 0x00);                              /* WRITE_SINGLE */
             s->sd_write_sector = arg; s->sd_write_pos = -1;
             break;
    case 25: sd_queue_byte(s, 0x00);                              /* WRITE_MULTI */
             s->sd_multi = 2; s->sd_multi_sector = arg;
             s->sd_write_sector = arg; s->sd_write_pos = -1;
             break;
    case 55: sd_queue_byte(s, s->sd_ready ? 0x00 : 0x01); break; /* APP_CMD */
    case 41: s->sd_ready = 1;                                   /* ACMD41: ready */
             sd_queue_byte(s, 0x00); break;
    case 42: sd_queue_byte(s, 0x00); break;                       /* APP_CLR_CD */
    case 58: { /* READ_OCR: idle bit set until ACMD41 completes */
             sd_queue_byte(s, s->sd_ready ? 0x00 : 0x01);
             static const uint8_t ocr[] = {0xC0, 0xFF, 0x80, 0x00};
             sd_queue(s, ocr, sizeof(ocr)); break; }
    case 59: sd_queue_byte(s, 0x05); break;   /* CRC_ON_OFF: illegal → driver
                                                 drops CRC checks (we send
                                                 0xFFFF everywhere) */
    default: sd_queue_byte(s, 0x05); break;                       /* illegal */
    }
}

/* Feed one MOSI byte from an SD-selected transaction; returns the MISO byte. */
static uint8_t sd_byte(spi_display_t *s, uint8_t mosi) {
    /* MISO: next queued response byte, 0xFF when idle */
    uint8_t miso = 0xFF;
    int response_was_pending = s->sd_resp_pos < s->sd_resp_len;
    if (response_was_pending)
        miso = s->sd_resp[s->sd_resp_pos++];

    /* Write-block data capture (after CMD24/25 R1) */
    if (s->sd_write_pos >= -1 && s->sd_write_sector != UINT32_MAX) {
        if (s->sd_write_pos < 0) {
            /* CMD24 uses the single-block 0xFE token.  CMD25 uses 0xFC for
             * every data block and terminates the stream with 0xFD. */
            if (mosi == 0xFE || (s->sd_multi == 2 && mosi == 0xFC)) {
                s->sd_write_pos = 0;
            } else if (s->sd_multi == 2 && mosi == 0xFD) {
                s->sd_multi = 0;
                s->sd_write_sector = UINT32_MAX;
                s->sd_write_pos = 0;
            }
            return miso;
        }
        if (s->sd_write_pos < 512) {
            s->sd_write_buf[s->sd_write_pos++] = mosi;
            return miso;
        }
        if (s->sd_write_pos < 514) {
            s->sd_write_pos++;                        /* 2 CRC bytes, ignore */
            if (s->sd_write_pos == 514) {
                sd_write_sector(s, s->sd_write_sector, s->sd_write_buf);
                sd_queue_byte(s, 0x05);               /* accepted */
                sd_queue_byte(s, 0x00); sd_queue_byte(s, 0x00);
                sd_queue_byte(s, 0x00); sd_queue_byte(s, 0xFF);
                if (s->sd_multi == 2) {               /* expect next block */
                    s->sd_write_sector = ++s->sd_multi_sector;
                    s->sd_write_pos = -1;
                } else {
                    s->sd_write_sector = UINT32_MAX;
                }
            }
            return miso;
        }
    }

    /* Multiblock read continuation: keep serving sectors until CMD12 */
    if (s->sd_multi == 1 && !response_was_pending &&
        s->sd_resp_pos >= s->sd_resp_len &&
        s->sd_cmd_len == 0 && mosi == 0xFF) {
        s->sd_multi_sector++;
        s->sd_resp_pos = 0; s->sd_resp_len = 0;
        sd_queue_block_read(s, s->sd_multi_sector);
        miso = s->sd_resp[s->sd_resp_pos++];
        return miso;
    }

    /* Command frame assembly */
    if (s->sd_cmd_len == 0) {
        if ((mosi & 0xC0) == 0x40) {
            s->sd_cmd[0] = mosi;
            s->sd_cmd_len = 1;
        }
    } else {
        s->sd_cmd[s->sd_cmd_len++] = mosi;
        if (s->sd_cmd_len == 6) {
            sd_execute(s);
            s->sd_cmd_len = 0;
        }
    }
    return miso;
}

/* ---- MMIO handlers ---- */

static void gp_spi_transact(spi_display_t *s) {
    s->trans_done = 0;
    gp_spi_intr_update(s);

    report_routes(s);

    size_t tx_requested = (s->user & SPI_USER_USR_MOSI) ?
                          phase_bytes(s->mosi_dlen) : 0;
    size_t rx_requested = (s->user & SPI_USER_USR_MISO) ?
                          phase_bytes(s->miso_dlen) : 0;
    bool tx_dma = tx_requested != 0 &&
                  (s->dma_out_link & SPI_DMA_LINK_START) != 0;
    bool rx_dma = rx_requested != 0 &&
                  (s->dma_in_link & SPI_DMA_LINK_START) != 0;
    uint8_t *tx_owned = NULL;
    uint8_t *rx_owned = NULL;
    const uint8_t *tx_data = (const uint8_t *)s->w;
    uint8_t *rx_data = (uint8_t *)s->w;
    size_t tx_len = tx_requested > sizeof(s->w) ? sizeof(s->w) : tx_requested;
    size_t rx_len = rx_requested > sizeof(s->w) ? sizeof(s->w) : rx_requested;

    if (tx_dma) {
        tx_len = 0;
        if (tx_requested <= SPI_DMA_MAX_TRANSFER)
            tx_owned = malloc(tx_requested);
        if (tx_owned) {
            memset(tx_owned, 0xFF, tx_requested);
            tx_len = gp_spi_dma_read_tx(s, tx_owned, tx_requested);
            tx_data = tx_owned;
        } else {
            s->dma_out_link &= ~SPI_DMA_LINK_START;
            s->dma_int_raw |= SPI_DMA_OUTLINK_ERROR;
        }
    }
    if (rx_dma) {
        rx_len = 0;
        if (rx_requested <= SPI_DMA_MAX_TRANSFER)
            rx_owned = calloc(rx_requested, 1);
        if (rx_owned) {
            rx_len = rx_requested;
            rx_data = rx_owned;
        } else {
            s->dma_in_link &= ~SPI_DMA_LINK_START;
            s->dma_int_raw |= SPI_DMA_INLINK_ERROR | SPI_DMA_IN_ERR_EOF;
        }
    }

    int cs_touch = route_allows_host(s, s->cfg.touch_sck_pin) &&
                   device_cs_state(s, s->cfg.touch_cs_pin);
    int cs_sd = route_allows_host(s, s->cfg.sd_sck_pin) &&
                device_cs_state(s, s->cfg.sd_cs_pin);
    int display_cs = route_allows_host(s, s->cfg.display_sck_pin) ?
                     device_cs_state(s, s->cfg.display_cs_pin) : 0;
    int cs_disp = display_cs ||
                  (display_cs == 0 &&
                   !cs_seen_high(s->cfg.display_cs_pin) &&
                   route_allows_host(s, s->cfg.display_sck_pin) &&
                   display_active(s));

    if (cs_touch) {
        /* XPT2046 conversions are pipelined.  The MISO bits clocked during
         * this transaction belong to the command accepted previously; a
         * new control byte in MOSI starts the conversion returned by the
         * next transfer.  Paul Stoffregen's driver depends on this when it
         * sends transfer16(next_command) while reading the prior result.
         *
         * ESP32's MSB-first transfer16 stores the on-wire bytes as 00,CMD
         * in W0, so scan the complete MOSI phase rather than only byte 0.
         * Preserve it before xpt2046_respond overwrites W0 with MISO. */
        uint8_t mosi[64];
        const uint8_t *touch_tx = tx_data;
        size_t touch_tx_len = tx_len;
        if (!tx_dma && touch_tx_len > 0) {
            memcpy(mosi, s->w, touch_tx_len);
            touch_tx = mosi;
        }
        uint8_t reply_cmd = s->touch_cmd;
        xpt2046_respond(s);
        if (rx_dma && rx_data) {
            size_t copy = rx_len < sizeof(s->w) ? rx_len : sizeof(s->w);
            memcpy(rx_data, s->w, copy);
        }
        for (size_t i = 0; i < touch_tx_len; i++) {
            if (touch_tx[i] & 0x80)
                s->touch_cmd = touch_tx[i];
        }
        if (getenv("FLEXE_TOUCHDBG")) {
            int rx, ry, pr; touch_sample(s, &rx, &ry, &pr);
            fprintf(stderr, "[TOUCH] reply=0x%02X next=0x%02X W0=0x%08X (rx=%d ry=%d pressed=%d)\n",
                    reply_cmd, s->touch_cmd, s->w[0], rx, ry, pr);
        }
    } else if (cs_disp) {
        /* A software-selected panel takes precedence over a stale hardware-CS
         * configuration left by ESP-IDF's SD driver on the shared host.  The
         * SD path is selected whenever the panel CS is inactive. */
        int dc = (gpio_level(s, s->cfg.dc_pin) == 1);

        /* Command phase (8 bits) if USR_COMMAND set */
        if (s->user & SPI_USER_USR_CMD) {
            uint8_t cmd = (uint8_t)(s->user2 & 0xFF);
            ili9341_feed(s, 0, &cmd, 1);
        }
        /* Address phase — display drivers don't use it; ignore */
        /* Data phase */
        if (tx_len > 0)
            ili9341_feed(s, dc, tx_data, (int)tx_len);
        /* Read commands (RDID 0x04 etc.): leave W as-is (zeros read as 0) */
    } else if (cs_sd) {
        /* Byte-at-a-time SD protocol exchange; write MISO back to W */
        size_t total = tx_len > rx_len ? tx_len : rx_len;
        for (size_t i = 0; i < total; i++) {
            uint8_t miso = sd_byte(s, i < tx_len ? tx_data[i] : 0xFF);
            if (i < rx_len) rx_data[i] = miso;
        }
        if (getenv("FLEXE_SDDBG2")) {
            fprintf(stderr, "[SD2] txn n=%zu m=%zu mosi=", tx_len, rx_len);
            for (size_t i = 0; i < total && i < 8; i++)
                fprintf(stderr, "%02X ", i < tx_len ? tx_data[i] : 0xFF);
            fprintf(stderr, "| miso=");
            for (size_t i = 0; i < rx_len && i < 8; i++)
                fprintf(stderr, "%02X ", rx_data[i]);
            fprintf(stderr, "\n");
        }
    }

    size_t rx_written = 0;
    if (rx_dma && rx_owned)
        rx_written = gp_spi_dma_write_rx(s, rx_owned, rx_requested);
    if ((tx_dma || rx_dma) && getenv("FLEXE_SPIDMADBG"))
        fprintf(stderr,
                "[SPIDMA] SPI%d tx=%zu/%zu rx=%zu/%zu raw=0x%03X\n",
                s->host_num, tx_len, tx_requested, rx_written, rx_requested,
                s->dma_int_raw);
    free(rx_owned);
    free(tx_owned);

    /* Transaction completes instantly: raise TRANS_DONE for polling and
     * interrupt-driven spi_master clients after all DMA writeback is visible. */
    s->trans_done = 1;
    gp_spi_intr_update(s);
}

static uint32_t gp_spi_read(void *ctx, uint32_t addr) {
    spi_display_t *s = ctx;
    uint32_t base = (addr >= SPI3_BASE) ? SPI3_BASE : SPI2_BASE;
    uint32_t off = addr - base;
    switch (off) {
    case SPI_CMD_REG:      return 0;   /* done, not busy */
    case SPI_ADDR_REG:     return s->addr;
    case SPI_USER_REG:     return s->user;
    case SPI_USER1_REG:    return s->user1;
    case SPI_USER2_REG:    return s->user2;
    case SPI_MOSI_DLEN_REG: return s->mosi_dlen;
    case SPI_MISO_DLEN_REG: return s->miso_dlen;
    case SPI_PIN_REG:      return s->pin;
    case SPI_SLAVE_REG:    return s->slave | (s->trans_done ? SPI_TRANS_DONE : 0);
    case SPI_DMA_CONF_REG: return s->dma_conf;
    case SPI_DMA_OUT_LINK_REG: return s->dma_out_link;
    case SPI_DMA_IN_LINK_REG: return s->dma_in_link;
    case SPI_DMA_STATUS_REG:
        return ((s->dma_out_link & SPI_DMA_LINK_START) ? 2u : 0u) |
               ((s->dma_in_link & SPI_DMA_LINK_START) ? 1u : 0u);
    case SPI_DMA_INT_ENA_REG: return s->dma_int_ena;
    case SPI_DMA_INT_RAW_REG: return s->dma_int_raw;
    case SPI_DMA_INT_ST_REG: return s->dma_int_raw & s->dma_int_ena;
    case SPI_DMA_INT_CLR_REG: return 0;
    case SPI_IN_ERR_EOF_DES_REG: return s->dma_in_err_eof_desc;
    case SPI_IN_SUC_EOF_DES_REG: return s->dma_in_suc_eof_desc;
    case SPI_INLINK_DSCR_REG: return s->dma_inlink_dscr;
    case SPI_INLINK_DSCR_BF0_REG: return s->dma_inlink_dscr_bf0;
    case SPI_INLINK_DSCR_BF1_REG: return s->dma_inlink_dscr_bf1;
    case SPI_OUT_EOF_BFR_DES_REG: return s->dma_out_eof_bfr_desc;
    case SPI_OUT_EOF_DES_REG: return s->dma_out_eof_desc;
    case SPI_OUTLINK_DSCR_REG: return s->dma_outlink_dscr;
    case SPI_OUTLINK_DSCR_BF0_REG: return s->dma_outlink_dscr_bf0;
    case SPI_OUTLINK_DSCR_BF1_REG: return s->dma_outlink_dscr_bf1;
    case SPI_DATE_REG: return 0x01604270u;
    default:
        if (off >= SPI_W0_REG && off < SPI_W0_REG + sizeof(s->w))
            return s->w[(off - SPI_W0_REG) / 4];
        return 0;   /* incl. SPI_EXT2_REG: idle */
    }
}

static void gp_spi_write(void *ctx, uint32_t addr, uint32_t val) {
    spi_display_t *s = ctx;
    uint32_t base = (addr >= SPI3_BASE) ? SPI3_BASE : SPI2_BASE;
    uint32_t off = addr - base;
    switch (off) {
    case SPI_CMD_REG:
        if (val & SPI_CMD_USR) gp_spi_transact(s);
        break;
    case SPI_ADDR_REG:     s->addr = val; break;
    case SPI_USER_REG:     s->user = val; break;
    case SPI_USER1_REG:    s->user1 = val; break;
    case SPI_USER2_REG:    s->user2 = val; break;
    case SPI_MOSI_DLEN_REG: s->mosi_dlen = val; break;
    case SPI_MISO_DLEN_REG: s->miso_dlen = val; break;
    case SPI_PIN_REG:      s->pin = val; break;
    case SPI_SLAVE_REG:
        s->slave = val & ~SPI_TRANS_DONE;
        if (!(val & SPI_TRANS_DONE)) s->trans_done = 0;  /* clear_intr */
        gp_spi_intr_update(s);
        break;
    case SPI_DMA_CONF_REG:
        s->dma_conf = val;
        if (val & SPI_DMA_OUT_RST)
            s->dma_out_link &= ~SPI_DMA_LINK_START;
        if (val & SPI_DMA_IN_RST)
            s->dma_in_link &= ~SPI_DMA_LINK_START;
        break;
    case SPI_DMA_OUT_LINK_REG:
        if (getenv("FLEXE_SPIDMADBG"))
            fprintf(stderr, "[SPIDMA] SPI%d OUT_LINK <- 0x%08X\n",
                    s->host_num, val);
        s->dma_out_link = val &
                (SPI_DMA_LINK_ADDR_MASK | SPI_DMA_LINK_START);
        if (val & SPI_DMA_LINK_STOP)
            s->dma_out_link &= ~SPI_DMA_LINK_START;
        break;
    case SPI_DMA_IN_LINK_REG:
        if (getenv("FLEXE_SPIDMADBG"))
            fprintf(stderr, "[SPIDMA] SPI%d IN_LINK <- 0x%08X\n",
                    s->host_num, val);
        s->dma_in_link = val &
                (SPI_DMA_LINK_ADDR_MASK | SPI_DMA_LINK_START);
        if (val & SPI_DMA_LINK_STOP)
            s->dma_in_link &= ~SPI_DMA_LINK_START;
        break;
    case SPI_DMA_INT_ENA_REG:
        s->dma_int_ena = val & SPI_DMA_INT_MASK;
        gp_spi_intr_update(s);
        break;
    case SPI_DMA_INT_CLR_REG:
        s->dma_int_raw &= ~(val & SPI_DMA_INT_MASK);
        gp_spi_intr_update(s);
        break;
    default:
        if (off >= SPI_W0_REG && off < SPI_W0_REG + sizeof(s->w))
            s->w[(off - SPI_W0_REG) / 4] = val;
        break;
    }
}

/* ---- public API ---- */

void periph_disable_spi_display(esp32_periph_t *p) {
    if (!p) return;
    for (int i = 0; i < 2; i++) {
        spi_display_t *s = &g_host[i];
        if (s->periph != p) continue;
        if (s->sd_fd >= 0) close(s->sd_fd);
        memset(s, 0, sizeof(*s));
    }
}

void periph_enable_spi_display(esp32_periph_t *p, const spi_display_config_t *cfg) {
    if (!p || !cfg) return;
    xtensa_mem_t *mem = periph_mem(p);

    periph_disable_spi_display(p);
    g_cs_seen_high = 0;
    for (int i = 0; i < 2; i++) {
        spi_display_t *s = &g_host[i];
        memset(s, 0, sizeof(*s));
        s->mem = mem;
        s->periph = p;
        s->cfg = *cfg;
        s->host_num = i + 2;
        s->pin = 0x7;           /* reset: CS0/1/2 outputs all disabled */
        /* Power-on defaults matching a reset panel */
        s->xe = 239;
        s->ye = 319;
        s->sd_fd = -1;
        s->sd_write_sector = UINT32_MAX;
    }
    mem_register_mmio(mem, (int)((SPI2_BASE - 0x3FF00000u) / 4096),
                      gp_spi_read, gp_spi_write, &g_host[0]);
    mem_register_mmio(mem, (int)((SPI3_BASE - 0x3FF00000u) / 4096),
                      gp_spi_read, gp_spi_write, &g_host[1]);
}
