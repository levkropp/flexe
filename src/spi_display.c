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
 * DMA transfers bypass the SPI FIFO and are not captured (documented
 * limitation); polling transfers — what TFT_eSPI and the Arduino SPI
 * drivers use by default — all pass through W0..W15 here.
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
#define SPI_SLAVE_REG     0x38
#define SPI_W0_REG        0x80

#define SPI_CMD_USR       (1u << 18)
#define SPI_TRANS_DONE    (1u << 4)
#define SPI_USER_USR_MOSI (1u << 27)
#define SPI_USER_USR_MISO (1u << 28)
#define SPI_USER_USR_ADDR (1u << 30)
#define SPI_USER_USR_CMD  (1u << 31)

/* ILI9341 commands */
#define ILI_CASET 0x2A
#define ILI_PASET 0x2B
#define ILI_RAMWR 0x2C
#define ILI_MADCTL 0x36

typedef struct {
    xtensa_mem_t *mem;
    esp32_periph_t *periph;      /* for GPIO CS/D-C sampling */
    spi_display_config_t cfg;

    /* Register shadow */
    uint32_t addr, user, user1, user2, mosi_dlen, miso_dlen;
    uint32_t slave;              /* SPI_SLAVE_REG shadow (config bits) */
    int      trans_done;         /* SPI_TRANS_DONE flag (slave reg bit 4) */
    uint32_t w[16];

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

/* A CS pin counts as "asserted" only if the firmware has driven it high at
 * least once. Undriven pins read 0 in the GPIO shadow, which would look
 * like a permanently-asserted chip select (e.g. touch CS on boards where
 * the app never configures it). */
static uint64_t g_cs_seen_high;   /* bit per GPIO number */

static int cs_asserted(spi_display_t *s, int pin) {
    if (pin < 0 || pin > 39) return 0;
    int lvl = gpio_level(s, pin);
    if (lvl == 1) g_cs_seen_high |= (1ULL << pin);
    return lvl == 0 && (g_cs_seen_high & (1ULL << pin)) != 0;
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

static int data_bytes(uint32_t dlen_reg) {
    int bits = (int)(dlen_reg & 0xFFFFFF) + 1;
    int bytes = (bits + 7) / 8;
    return bytes > 64 ? 64 : bytes;
}

/* Map panel-native (x, y) to framebuffer coordinates.
 * Panel RAM is 240x320 portrait; the emulator window is 320x240
 * landscape, so portrait content is presented rotated 90° CW. */
static void fb_write_pixel(spi_display_t *s, uint16_t x, uint16_t y, uint16_t rgb565) {
    if (!s->cfg.framebuf) return;
    uint32_t fx, fy;
    int rot;
    if (s->madctl & 0x20) rot = (s->madctl & 0x40) ? 3 : 1;   /* MV set */
    else                  rot = (s->madctl & 0x40) ? 2 : 0;
    switch (rot) {
    case 0:  fx = 319 - y;  fy = x;        break;  /* portrait, rotated CW */
    case 1:  fx = y;        fy = 239 - x;  break;  /* landscape */
    case 2:  fx = y;        fy = 239 - x;  break;  /* portrait flipped */
    default: fx = 319 - y;  fy = x;        break;  /* landscape flipped */
    }
    if (fx >= (uint32_t)s->cfg.fb_w || fy >= (uint32_t)s->cfg.fb_h) return;
    if (s->cfg.framebuf_mtx) pthread_mutex_lock(s->cfg.framebuf_mtx);
    s->cfg.framebuf[fy * s->cfg.fb_w + fx] = rgb565;
    if (s->cfg.framebuf_mtx) pthread_mutex_unlock(s->cfg.framebuf_mtx);
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
        }
        break;
    case ILI_PASET:
        if (s->param_cnt == 4) {
            s->ys = (uint16_t)((s->params[0] << 8) | s->params[1]);
            s->ye = (uint16_t)((s->params[2] << 8) | s->params[3]);
        }
        break;
    case ILI_MADCTL:
        if (s->param_cnt == 1) s->madctl = b;
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
    /* Auto-increment with window wrap (panel behavior) */
    s->cx++;
    if (s->cx > s->xe || s->cx >= 240) {
        s->cx = s->xs;
        s->cy++;
        if (s->cy > s->ye || s->cy >= 320) s->cy = s->ys;
    }
}

static void ili9341_feed(spi_display_t *s, int dc, const uint8_t *data, int len) {
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
        *rx = x * 4095 / (s->cfg.fb_w - 1);
        *ry = y * 4095 / (s->cfg.fb_h - 1);
    }
    if (*rx < 0) *rx = 0; if (*rx > 4095) *rx = 4095;
    if (*ry < 0) *ry = 0; if (*ry > 4095) *ry = 4095;
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
    case 1: val = (uint16_t)ry; break;              /* Y */
    case 5: val = (uint16_t)rx; break;              /* X */
    case 3: case 4: val = pressed ? 600 : 0; break; /* Z1/Z2 */
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
    if (sd_image_open(s) >= 0)
        (void)pwrite(s->sd_fd, in, 512, (off_t)lba * 512);
}

/* CSD v2 for a 4 GB SDHC card (C_SIZE = 8191) */
static const uint8_t SD_CSD[16] = {
    0x40, 0x0E, 0x00, 0x32, 0x5B, 0x59, 0x00, 0x00,
    0x1F, 0xFF, 0x80, 0x80, 0x0A, 0xC0, 0x40, 0x00
};

static void sd_queue_block_read(spi_display_t *s, uint32_t lba) {
    uint8_t tok = 0xFE;
    sd_queue(s, &tok, 1);
    uint8_t blk[512];
    sd_read_sector(s, lba, blk);
    sd_queue(s, blk, 512);
    uint8_t crc[2] = {0xFF, 0xFF};
    sd_queue(s, crc, 2);
}

static void sd_execute(spi_display_t *s) {
    uint8_t cmd = s->sd_cmd[0] & 0x3F;
    uint32_t arg = ((uint32_t)s->sd_cmd[1] << 24) | ((uint32_t)s->sd_cmd[2] << 16) |
                   ((uint32_t)s->sd_cmd[3] << 8) | s->sd_cmd[4];
    s->sd_resp_pos = 0;
    s->sd_resp_len = 0;
    switch (cmd) {
    case 0:  sd_queue_byte(s, 0x01); break;                       /* GO_IDLE */
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
    case 12: sd_queue_byte(s, 0x00); s->sd_multi = 0; break;      /* STOP_TRANS */
    case 13: { uint8_t r2[2] = {0x00, 0x00}; sd_queue(s, r2, 2); break; }
    case 16: sd_queue_byte(s, 0x00); break;                       /* SET_BLOCKLEN */
    case 17: sd_queue_byte(s, 0x00);                              /* READ_SINGLE */
             sd_queue_block_read(s, arg); break;
    case 18: sd_queue_byte(s, 0x00);                              /* READ_MULTI */
             s->sd_multi = 1; s->sd_multi_sector = arg;
             sd_queue_block_read(s, arg); break;
    case 24: sd_queue_byte(s, 0x00);                              /* WRITE_SINGLE */
             s->sd_write_sector = arg; s->sd_write_pos = -1;
             break;
    case 25: sd_queue_byte(s, 0x00);                              /* WRITE_MULTI */
             s->sd_multi = 2; s->sd_multi_sector = arg;
             s->sd_write_sector = arg; s->sd_write_pos = -1;
             break;
    case 55: sd_queue_byte(s, 0x01); break;                       /* APP_CMD */
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
    if (s->sd_resp_pos < s->sd_resp_len)
        miso = s->sd_resp[s->sd_resp_pos++];

    /* Write-block data capture (after CMD24/25 R1) */
    if (s->sd_write_pos >= -1 && s->sd_write_sector != UINT32_MAX) {
        if (s->sd_write_pos < 0) {
            if (mosi == 0xFE) s->sd_write_pos = 0;   /* data token */
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
    if (s->sd_multi == 1 && s->sd_resp_pos >= s->sd_resp_len &&
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
    /* Transaction completes instantly: raise TRANS_DONE for drivers that
     * poll SPI_SLAVE_REG (spi_device_polling_end / spi_hal_usr_is_done) */
    s->trans_done = 1;

    int cs_touch = cs_asserted(s, s->cfg.touch_cs_pin);
    int cs_sd = cs_asserted(s, s->cfg.sd_cs_pin);
    int cs_disp = display_active(s) && !cs_touch && !cs_sd;

    if (cs_touch) {
        /* First MOSI byte with bit7 set is the control byte */
        const uint8_t *mosi = (const uint8_t *)s->w;
        if ((s->user & SPI_USER_USR_MOSI) && data_bytes(s->mosi_dlen) >= 1 &&
            (mosi[0] & 0x80))
            s->touch_cmd = mosi[0];
        xpt2046_respond(s);
        return;
    }

    if (cs_sd) {
        /* Byte-at-a-time SD protocol exchange; write MISO back to W */
        int n = data_bytes(s->mosi_dlen);
        int m = data_bytes(s->miso_dlen);
        int total = n > m ? n : m;
        if (total > 64) total = 64;
        const uint8_t *mosi = (const uint8_t *)s->w;
        uint8_t miso[64];
        for (int i = 0; i < total; i++)
            miso[i] = sd_byte(s, i < n ? mosi[i] : 0xFF);
        memcpy(s->w, miso, (size_t)total);
        return;
    }

    if (!cs_disp) return;

    int dc = (gpio_level(s, s->cfg.dc_pin) == 1);

    /* Command phase (8 bits) if USR_COMMAND set */
    if (s->user & SPI_USER_USR_CMD) {
        uint8_t cmd = (uint8_t)(s->user2 & 0xFF);
        ili9341_feed(s, 0, &cmd, 1);
    }
    /* Address phase — display drivers don't use it; ignore */
    /* Data phase */
    if (s->user & SPI_USER_USR_MOSI) {
        int n = data_bytes(s->mosi_dlen);
        if (n > 0) ili9341_feed(s, dc, (const uint8_t *)s->w, n);
    }
    /* Read commands (RDID 0x04 etc.): leave W as-is (zeros read as 0) */
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
    case SPI_SLAVE_REG:    return s->slave | (s->trans_done ? SPI_TRANS_DONE : 0);
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
    case SPI_SLAVE_REG:
        s->slave = val & ~SPI_TRANS_DONE;
        if (!(val & SPI_TRANS_DONE)) s->trans_done = 0;  /* clear_intr */
        break;
    default:
        if (off >= SPI_W0_REG && off < SPI_W0_REG + sizeof(s->w))
            s->w[(off - SPI_W0_REG) / 4] = val;
        break;
    }
}

/* ---- public API ---- */

void periph_enable_spi_display(esp32_periph_t *p, const spi_display_config_t *cfg) {
    if (!p || !cfg) return;
    xtensa_mem_t *mem = periph_mem(p);

    for (int i = 0; i < 2; i++) {
        spi_display_t *s = &g_host[i];
        memset(s, 0, sizeof(*s));
        s->mem = mem;
        s->periph = p;
        s->cfg = *cfg;
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
