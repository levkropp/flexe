/* spi_display.c — target-described GP-SPI (SPI2/SPI3) emulation with
 * optional ILI9341, XPT2046, SD, and host-attached slave endpoints.
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
 * Polling transfers pass through W0..W15. Classic DMA walks native lldesc
 * chains; S2/S3-generation DMA consumes the target's GDMA channels. Both
 * paths enter the same board/device boundary without firmware-specific hooks.
 */

#include "spi_display.h"
#include "gdma.h"
#include "memory.h"
#include "sandbox_events.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>

/* Debug logging switches, resolved once.
 *
 * Several of these sit on per-SPI-byte paths, and a CYD firmware pushes
 * megabytes through them: Marauder writes 5.3 MB to the panel in one scenario.
 * getenv() is a linear scan of environ, so asking it per byte cost 5.2% of the
 * emulator's total host CPU — more than the whole JIT dispatch path. */
static int spi_dbg_cs = -1, spi_dbg_route = -1, spi_dbg_dma = -1,
           spi_dbg_disp = -1, spi_dbg_sd = -1, spi_dbg_sd2 = -1,
           spi_dbg_touch = -1;

static void spi_dbg_resolve(void) {
    spi_dbg_cs    = getenv("FLEXE_CSDBG") != NULL;
    spi_dbg_route = getenv("FLEXE_SPIROUTEDBG") != NULL;
    spi_dbg_dma   = getenv("FLEXE_SPIDMADBG") != NULL;
    spi_dbg_disp  = getenv("FLEXE_DISPG") != NULL;
    spi_dbg_sd    = getenv("FLEXE_SDDBG") != NULL;
    spi_dbg_sd2   = getenv("FLEXE_SDDBG2") != NULL;
    spi_dbg_touch = getenv("FLEXE_TOUCHDBG") != NULL;
}

static inline int spi_dbg(const int *flag) {
    if (__builtin_expect(*flag < 0, 0)) spi_dbg_resolve();
    return *flag;
}

/* Original ESP32 GP-SPI register generation. */
#define ESP32_SPI_CMD_REG          0x000u
#define ESP32_SPI_ADDR_REG         0x004u
#define ESP32_SPI_USER_REG         0x01Cu
#define ESP32_SPI_USER1_REG        0x020u
#define ESP32_SPI_USER2_REG        0x024u
#define ESP32_SPI_MOSI_DLEN_REG    0x028u
#define ESP32_SPI_MISO_DLEN_REG    0x02Cu
#define ESP32_SPI_PIN_REG          0x034u
#define ESP32_SPI_SLAVE_REG        0x038u
#define ESP32_SPI_W0_REG           0x080u
#define ESP32_SPI_EXT2_REG         0x0F8u
#define ESP32_SPI_DMA_CONF_REG     0x100u
#define ESP32_SPI_DMA_OUT_LINK_REG 0x104u
#define ESP32_SPI_DMA_IN_LINK_REG  0x108u
#define ESP32_SPI_DMA_STATUS_REG   0x10Cu
#define ESP32_SPI_DMA_INT_ENA_REG  0x110u
#define ESP32_SPI_DMA_INT_RAW_REG  0x114u
#define ESP32_SPI_DMA_INT_ST_REG   0x118u
#define ESP32_SPI_DMA_INT_CLR_REG  0x11Cu
#define ESP32_SPI_IN_ERR_EOF_DES_REG  0x120u
#define ESP32_SPI_IN_SUC_EOF_DES_REG  0x124u
#define ESP32_SPI_INLINK_DSCR_REG     0x128u
#define ESP32_SPI_INLINK_DSCR_BF0_REG 0x12Cu
#define ESP32_SPI_INLINK_DSCR_BF1_REG 0x130u
#define ESP32_SPI_OUT_EOF_BFR_DES_REG 0x134u
#define ESP32_SPI_OUT_EOF_DES_REG     0x138u
#define ESP32_SPI_OUTLINK_DSCR_REG     0x13Cu
#define ESP32_SPI_OUTLINK_DSCR_BF0_REG 0x140u
#define ESP32_SPI_OUTLINK_DSCR_BF1_REG 0x144u
#define ESP32_SPI_DATE_REG          0x3FCu

/* S2/S3-generation GP-SPI register generation. */
#define S3_SPI_CMD_REG          0x000u
#define S3_SPI_ADDR_REG         0x004u
#define S3_SPI_CTRL_REG         0x008u
#define S3_SPI_CLOCK_REG        0x00Cu
#define S3_SPI_USER_REG         0x010u
#define S3_SPI_USER1_REG        0x014u
#define S3_SPI_USER2_REG        0x018u
#define S3_SPI_MS_DLEN_REG      0x01Cu
#define S3_SPI_MISC_REG         0x020u
#define S3_SPI_DIN_MODE_REG     0x024u
#define S3_SPI_DIN_NUM_REG      0x028u
#define S3_SPI_DOUT_MODE_REG    0x02Cu
#define S3_SPI_DMA_CONF_REG     0x030u
#define S3_SPI_DMA_INT_ENA_REG  0x034u
#define S3_SPI_DMA_INT_CLR_REG  0x038u
#define S3_SPI_DMA_INT_RAW_REG  0x03Cu
#define S3_SPI_DMA_INT_ST_REG   0x040u
#define S3_SPI_DMA_INT_SET_REG  0x044u
#define S3_SPI_W0_REG           0x098u
#define S3_SPI_SLAVE_REG        0x0E0u
#define S3_SPI_SLAVE1_REG       0x0E4u
#define S3_SPI_CLK_GATE_REG     0x0E8u
#define S3_SPI_DATE_REG         0x0F0u

#define ESP32_SPI_CMD_USR       (1u << 18)
#define ESP32_SPI_CMD_DEDICATED_MASK \
    (0xFFFF0000u & ~ESP32_SPI_CMD_USR)
#define S3_SPI_CMD_USR          (1u << 24)
#define S3_SPI_CMD_UPDATE       (1u << 23)
#define S3_SPI_CMD_CONFIG_MASK  0x0003FFFFu
#define SPI_FSM_COMMAND    2u
#define SPI_TRANS_DONE    (1u << 4)
#define SPI_TRANS_INTEN   (1u << 9)
#define SPI_USER_USR_MOSI (1u << 27)
#define SPI_USER_USR_MISO (1u << 28)
#define SPI_USER_USR_ADDR (1u << 30)
#define SPI_USER_USR_CMD  (1u << 31)
#define SPI_USER_MOSI_HIGHPART (1u << 25)
#define SPI_USER_MISO_HIGHPART (1u << 24)

#define S3_SPI_TRANS_DONE       (1u << 12)
#define S3_SPI_DMA_INT_MASK     0x001FFFFFu
#define S3_SPI_DMA_TX_ENABLE    (1u << 28)
#define S3_SPI_DMA_RX_ENABLE    (1u << 27)
#define S3_SPI_INTERNAL_CLK_EN  (1u << 0)
#define S3_SPI_DMA_INFIFO_FULL_ERROR    (1u << 0)
#define S3_SPI_DMA_OUTFIFO_EMPTY_ERROR  (1u << 1)
#define S3_SPI_MST_RX_AFIFO_FULL_ERROR  (1u << 17)
#define S3_SPI_MST_TX_AFIFO_EMPTY_ERROR (1u << 18)
#define S3_SPI_DMA_SEGMENT_ENABLE       (1u << 18)
#define S3_SPI_USER_CONFIG_NEXT         (1u << 15)
#define S3_SPI_SLAVE_MODE               (1u << 26)
#define S3_SPI_SLAVE_SOFT_RESET         (1u << 27)
#define S3_SPI_SLAVE_USER_CONFIG        (1u << 28)

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
#define SPI_DEVICE_MAX          8

/* ILI9341 commands */
#define ILI_CASET 0x2A
#define ILI_PASET 0x2B
#define ILI_RAMWR 0x2C
#define ILI_MADCTL 0x36

typedef struct gp_spi_host {
    flexe_gp_spi_t *owner;
    xtensa_mem_t *mem;
    esp32_periph_t *periph;      /* for GPIO CS/D-C sampling */
    const flexe_gp_spi_instance_desc_t *instance;
    spi_display_config_t cfg;

    /* Register shadow */
    uint32_t cmd_config, addr, user, user1, user2, mosi_dlen, miso_dlen;
    uint32_t ctrl, clock, din_mode, din_num, dout_mode, slave1, clk_gate;
    uint32_t date;
    uint32_t pin;                /* SPI_PIN_REG: active hardware CS line */
    uint32_t slave;              /* SPI_SLAVE_REG shadow (config bits) */
    int      trans_done;         /* SPI_TRANS_DONE flag (slave reg bit 4) */
    uint8_t  fsm_observation;    /* next SPI_EXT2 state observation */
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
    uint8_t  touch_gpio_in;
    uint8_t  touch_gpio_in_bits;
    uint16_t touch_gpio_out;
    uint8_t  touch_gpio_out_bits;
    int      touch_gpio_selected;

    int      dc_seen;            /* D/C pin observed high on this host */
    int      host_num;           /* 2 = HSPI, 3 = VSPI */
    unsigned instance_index;
    bool     system_clock_enabled;
    bool     system_reset_asserted;
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
    /* Harness stand-in for an unmodelled slave (see periph_spi_attach_probe). */
    spi_probe_fn probe_fn;
    void        *probe_ctx;
    struct {
        int cs_pin;
        int sck_pin;
        periph_spi_device_fn fn;
        periph_spi_select_fn select_fn;
        void *ctx;
    } device[SPI_DEVICE_MAX];
} spi_display_t;

struct flexe_gp_spi {
    xtensa_mem_t *mem;
    esp32_periph_t *periph;
    const flexe_target_desc_t *target;
    mmio_read_fn fallback_read;
    mmio_write_fn fallback_write;
    void *fallback_ctx;
    uint64_t cs_seen_high;
    spi_display_t host[FLEXE_TARGET_GP_SPI_HOST_MAX];
};

/* Legacy process-wide observability counters are retained for the stock-ROM
 * harness API. Controller and chip-select state itself is per peripheral. */
static uint64_t g_touch_bitbang_commands;

static int gpio_level(const spi_display_t *s, int pin) {
    if (pin < 0) return -1;
    return periph_gpio_pin_level(s->periph, pin);
}

/* An inactive high level or an enabled output driver distinguishes a genuine
 * software chip select from an untouched reset pin whose latch also reads 0. */
static int cs_seen_high(const spi_display_t *s, int pin) {
    return pin >= 0 && pin < 64 &&
           (s->owner->cs_seen_high & (UINT64_C(1) << pin)) != 0u;
}

static int cs_asserted(spi_display_t *s, int pin) {
    if (pin < 0 || pin >= 64) return 0;
    int lvl = gpio_level(s, pin);
    if (lvl == 1) s->owner->cs_seen_high |= UINT64_C(1) << pin;
    /* A driver may configure CS as an output and immediately assert it before
     * the first SPI transaction, so the sniffer never observes it high. */
    int asserted = lvl == 0 &&
                   (cs_seen_high(s, pin) ||
                    periph_gpio_output_enabled(s->periph, pin));
    if (asserted && spi_dbg(&spi_dbg_cs))
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
    const flexe_gp_spi_desc_t *desc = &s->owner->target->gp_spi;
    if (signal == (int)s->instance->clock_out_signal) return 1;
    for (unsigned i = 0u; i < desc->host_count; i++) {
        if (i != s->instance_index &&
            signal == (int)desc->instance[i].clock_out_signal)
            return 0;
    }
    /* Native routes are target data just like GPIO-matrix signal numbers.
     * SPI3 on S3, for example, deliberately describes no IOMUX route. */
    int iomux = periph_iomux_function(s->periph, pin);
    for (unsigned i = 0u; i < desc->host_count; i++) {
        const flexe_gp_spi_instance_desc_t *instance = &desc->instance[i];
        if (instance->iomux_clock_pin != FLEXE_TARGET_GPIO_NONE &&
            pin == instance->iomux_clock_pin &&
            iomux == instance->iomux_function)
            return i == s->instance_index;
    }
    return -1;
}

static int route_allows_host(const spi_display_t *s, int clock_pin) {
    return clock_route(s, clock_pin) != 0;
}

/* If GPIO-matrix or native IOMUX routing hands a configured pin to one of a
 * host's hardware chip selects, the GPIO latch no longer owns its level.
 * Return whether that CS is enabled for this transaction, zero when it is
 * explicitly routed from another host, or -1 for software/unknown routing. */
static int hardware_cs_state(const spi_display_t *s, int pin) {
    int signal = periph_gpio_out_signal(s->periph, pin);
    int cs = -1;
    const flexe_gp_spi_desc_t *desc = &s->owner->target->gp_spi;
    for (unsigned host = 0u; host < desc->host_count; host++) {
        const flexe_gp_spi_instance_desc_t *instance =
            &desc->instance[host];
        for (unsigned index = 0u; index < instance->chip_select_count;
             index++) {
            if (signal != (int)instance->chip_select_out_signal[index])
                continue;
            if (host != s->instance_index) return 0;
            cs = (int)index;
        }
    }
    int iomux = periph_iomux_function(s->periph, pin);
    if (cs < 0) {
        for (unsigned host = 0u; host < desc->host_count; host++) {
            const flexe_gp_spi_instance_desc_t *instance =
                &desc->instance[host];
            if (instance->iomux_chip_select0_pin == FLEXE_TARGET_GPIO_NONE ||
                pin != instance->iomux_chip_select0_pin ||
                iomux != instance->iomux_function)
                continue;
            if (host != s->instance_index) return 0;
            cs = 0;
        }
    }
    return cs < 0 ? -1 : ((s->pin & (1u << cs)) == 0);
}

static int device_cs_state(spi_display_t *s, int pin) {
    int state = hardware_cs_state(s, pin);
    return state >= 0 ? state : cs_asserted(s, pin);
}

static void report_routes(spi_display_t *s) {
    if (!spi_dbg(&spi_dbg_route)) return;
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

static bool gp_spi_is_s3(const spi_display_t *s) {
    return s->owner->target->gp_spi.layout == FLEXE_GP_SPI_LAYOUT_S2_S3;
}

static size_t phase_bytes(const spi_display_t *s, uint32_t dlen_reg) {
    uint32_t mask = gp_spi_is_s3(s) ? 0x0003FFFFu : 0x00FFFFFFu;
    uint32_t bits = (dlen_reg & mask) + 1u;
    return (bits + 7u) / 8u;
}

static void gp_spi_intr_update(spi_display_t *s) {
    bool active = gp_spi_is_s3(s)
        ? (s->dma_int_raw & s->dma_int_ena) != 0u
        : (s->trans_done && (s->slave & SPI_TRANS_INTEN)) ||
          ((s->dma_int_raw & s->dma_int_ena) != 0u);
    int source = s->instance->interrupt_source;
    if (active)
        periph_assert_interrupt(s->periph, source);
    else
        periph_deassert_interrupt(s->periph, source);
}

static void gp_spi_report_unsupported(spi_display_t *s, uint32_t addr,
                                      uint32_t value) {
    if (s->owner->fallback_write)
        s->owner->fallback_write(s->owner->fallback_ctx, addr, value);
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
            if (spi_dbg(&spi_dbg_dma))
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
            if (spi_dbg(&spi_dbg_dma))
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
            if (spi_dbg(&spi_dbg_disp))
                fprintf(stderr, "[DISPG] CASET xs=%u xe=%u (w=%u)\n", s->xs, s->xe, s->xe - s->xs + 1);
        }
        break;
    case ILI_PASET:
        if (s->param_cnt == 4) {
            s->ys = (uint16_t)((s->params[0] << 8) | s->params[1]);
            s->ye = (uint16_t)((s->params[2] << 8) | s->params[3]);
            if (spi_dbg(&spi_dbg_disp))
                fprintf(stderr, "[DISPG] PASET ys=%u ye=%u (h=%u)\n", s->ys, s->ye, s->ye - s->ys + 1);
        }
        break;
    case ILI_MADCTL:
        if (s->param_cnt == 1) {
            s->madctl = b;
            if (spi_dbg(&spi_dbg_disp))
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

/* Bytes handed to the panel. A firmware whose display task has stopped shows
 * up as this standing still while the CPU stays busy -- which a framebuffer
 * checksum alone cannot distinguish from a UI that is merely redrawing the
 * same picture. */
static uint64_t g_display_bytes;

uint64_t spi_display_bytes_fed(void) { return g_display_bytes; }

static void ili9341_feed(spi_display_t *s, int dc, const uint8_t *data, int len) {
    g_display_bytes += (uint64_t)len;
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
static uint16_t xpt2046_value(spi_display_t *s, uint8_t command) {
    int rx, ry, pressed;
    touch_sample(s, &rx, &ry, &pressed);
    int chan = (command >> 4) & 0x7;
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
    return val;
}

static void xpt2046_respond(spi_display_t *s) {
    uint16_t val = xpt2046_value(s, s->touch_cmd);
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
    if (spi_dbg(&spi_dbg_sd))
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
    if (spi_dbg(&spi_dbg_sd))
        { extern uint32_t g_dbg_pc;
          fprintf(stderr, "[SD] cmd%u arg=0x%08X pc=%08X\n", cmd, arg,
                  g_dbg_pc); }
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
    if (gp_spi_is_s3(s))
        s->dma_int_raw &= ~S3_SPI_TRANS_DONE;
    else
        s->trans_done = 0;
    gp_spi_intr_update(s);

    report_routes(s);

    size_t tx_requested = (s->user & SPI_USER_USR_MOSI) ?
                          phase_bytes(s, s->mosi_dlen) : 0;
    size_t rx_requested = (s->user & SPI_USER_USR_MISO) ?
                          phase_bytes(s, s->miso_dlen) : 0;
    bool tx_dma = tx_requested != 0u &&
        (gp_spi_is_s3(s) ? (s->dma_conf & S3_SPI_DMA_TX_ENABLE) != 0u
                         : (s->dma_out_link & SPI_DMA_LINK_START) != 0u);
    bool rx_dma = rx_requested != 0u &&
        (gp_spi_is_s3(s) ? (s->dma_conf & S3_SPI_DMA_RX_ENABLE) != 0u
                         : (s->dma_in_link & SPI_DMA_LINK_START) != 0u);
    uint8_t *tx_owned = NULL;
    uint8_t *rx_owned = NULL;
    size_t tx_fifo_offset = (s->user & SPI_USER_MOSI_HIGHPART) ? 32u : 0u;
    size_t rx_fifo_offset = (s->user & SPI_USER_MISO_HIGHPART) ? 32u : 0u;
    const uint8_t *tx_data = (const uint8_t *)s->w + tx_fifo_offset;
    uint8_t *rx_data = (uint8_t *)s->w + rx_fifo_offset;
    size_t tx_capacity = sizeof(s->w) - tx_fifo_offset;
    size_t rx_capacity = sizeof(s->w) - rx_fifo_offset;
    size_t tx_len = tx_requested > tx_capacity ? tx_capacity : tx_requested;
    size_t rx_len = rx_requested > rx_capacity ? rx_capacity : rx_requested;

    if (gp_spi_is_s3(s) &&
        ((s->slave & (S3_SPI_SLAVE_MODE | S3_SPI_SLAVE_USER_CONFIG)) ||
         (s->dma_conf & S3_SPI_DMA_SEGMENT_ENABLE) ||
         (s->user & S3_SPI_USER_CONFIG_NEXT) || s->cmd_config != 0u)) {
        /* Slave and segmented/config-buffer transactions have observably
         * different framing. Do not silently turn them into master transfers. */
        gp_spi_report_unsupported(s, s->instance->base + S3_SPI_CMD_REG,
                                  S3_SPI_CMD_USR);
        s->dma_int_raw |= S3_SPI_TRANS_DONE;
        gp_spi_intr_update(s);
        return;
    }

    if (gp_spi_is_s3(s) && !tx_dma && tx_requested > tx_capacity) {
        s->dma_int_raw |= S3_SPI_MST_TX_AFIFO_EMPTY_ERROR;
        gp_spi_report_unsupported(s, s->instance->base, S3_SPI_CMD_USR);
    }
    if (gp_spi_is_s3(s) && !rx_dma && rx_requested > rx_capacity) {
        s->dma_int_raw |= S3_SPI_MST_RX_AFIFO_FULL_ERROR;
        gp_spi_report_unsupported(s, s->instance->base, S3_SPI_CMD_USR);
    }

    if (tx_dma) {
        tx_len = 0;
        if (tx_requested <= SPI_DMA_MAX_TRANSFER)
            tx_owned = malloc(tx_requested);
        if (tx_owned) {
            memset(tx_owned, 0xFF, tx_requested);
            if (gp_spi_is_s3(s)) {
                flexe_gdma_t *gdma = periph_gdma(s->periph);
                if (flexe_gdma_read_tx(
                        gdma, s->instance->gdma_peripheral_id,
                        tx_owned, tx_requested) == 0)
                    tx_len = tx_requested;
                else {
                    s->dma_int_raw |= S3_SPI_DMA_OUTFIFO_EMPTY_ERROR;
                    gp_spi_report_unsupported(
                        s, s->instance->base + S3_SPI_DMA_CONF_REG,
                        s->dma_conf);
                }
            } else {
                tx_len = gp_spi_dma_read_tx(s, tx_owned, tx_requested);
            }
            tx_data = tx_owned;
        } else if (!gp_spi_is_s3(s)) {
            s->dma_out_link &= ~SPI_DMA_LINK_START;
            s->dma_int_raw |= SPI_DMA_OUTLINK_ERROR;
        } else {
            s->dma_int_raw |= S3_SPI_DMA_OUTFIFO_EMPTY_ERROR;
            gp_spi_report_unsupported(
                s, s->instance->base + S3_SPI_DMA_CONF_REG, s->dma_conf);
        }
    }
    if (rx_dma) {
        rx_len = 0;
        if (rx_requested <= SPI_DMA_MAX_TRANSFER)
            rx_owned = calloc(rx_requested, 1);
        if (rx_owned) {
            rx_len = rx_requested;
            rx_data = rx_owned;
        } else if (!gp_spi_is_s3(s)) {
            s->dma_in_link &= ~SPI_DMA_LINK_START;
            s->dma_int_raw |= SPI_DMA_INLINK_ERROR | SPI_DMA_IN_ERR_EOF;
        } else {
            s->dma_int_raw |= S3_SPI_DMA_INFIFO_FULL_ERROR;
            gp_spi_report_unsupported(
                s, s->instance->base + S3_SPI_DMA_CONF_REG, s->dma_conf);
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
                   !cs_seen_high(s, s->cfg.display_cs_pin) &&
                   route_allows_host(s, s->cfg.display_sck_pin) &&
                   display_active(s));

    periph_spi_device_fn device_fn = NULL;
    void *device_ctx = NULL;
    for (unsigned i = 0; i < SPI_DEVICE_MAX; i++) {
        if (!s->device[i].fn ||
            !route_allows_host(s, s->device[i].sck_pin) ||
            !device_cs_state(s, s->device[i].cs_pin))
            continue;
        device_fn = s->device[i].fn;
        device_ctx = s->device[i].ctx;
        break;
    }

    if (device_fn) {
        /* Explicit board wiring is more specific than the built-in CYD
         * defaults. This matters on boards such as the T-Beam, whose LoRa
         * SCK/CS pins overlap the default CYD SD-card pin numbers. */
        device_fn(device_ctx, s->host_num, tx_data, tx_len,
                  rx_data, rx_len);
    } else if (cs_touch) {
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
            memcpy(mosi, tx_data, touch_tx_len);
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
        if (spi_dbg(&spi_dbg_touch)) {
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
    } else if (!cs_touch && !cs_disp && !cs_sd && s->probe_fn) {
        /* No modelled device selected, but a harness is standing in for one. */
        s->probe_fn(tx_data, tx_len, rx_data, rx_len, s->probe_ctx);
    } else if (cs_sd) {
        /* Byte-at-a-time SD protocol exchange; write MISO back to W */
        size_t total = tx_len > rx_len ? tx_len : rx_len;
        for (size_t i = 0; i < total; i++) {
            uint8_t miso = sd_byte(s, i < tx_len ? tx_data[i] : 0xFF);
            if (i < rx_len) rx_data[i] = miso;
        }
        if (spi_dbg(&spi_dbg_sd2)) {
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
    if (rx_dma && rx_owned) {
        if (gp_spi_is_s3(s)) {
            flexe_gdma_t *gdma = periph_gdma(s->periph);
            if (flexe_gdma_write_rx(
                    gdma, s->instance->gdma_peripheral_id,
                    rx_owned, rx_requested) == 0)
                rx_written = rx_requested;
            else {
                s->dma_int_raw |= S3_SPI_DMA_INFIFO_FULL_ERROR;
                gp_spi_report_unsupported(
                    s, s->instance->base + S3_SPI_DMA_CONF_REG,
                        s->dma_conf);
            }
        } else {
            rx_written = gp_spi_dma_write_rx(s, rx_owned, rx_requested);
        }
    }
    if ((tx_dma || rx_dma) && spi_dbg(&spi_dbg_dma))
        fprintf(stderr,
                "[SPIDMA] SPI%d tx=%zu/%zu rx=%zu/%zu raw=0x%03X\n",
                s->host_num, tx_len, tx_requested, rx_written, rx_requested,
                s->dma_int_raw);
    free(rx_owned);
    free(tx_owned);

    /* Transaction completes instantly: raise TRANS_DONE for polling and
     * interrupt-driven spi_master clients after all DMA writeback is visible. */
    if (gp_spi_is_s3(s))
        s->dma_int_raw |= S3_SPI_TRANS_DONE;
    else
        s->trans_done = 1;
    gp_spi_intr_update(s);
}

static uint32_t gp_spi_read(void *ctx, uint32_t addr) {
    spi_display_t *s = ctx;
    uint32_t off = addr - s->instance->base;
    if (gp_spi_is_s3(s)) {
        switch (off) {
        case S3_SPI_CMD_REG:
            return s->cmd_config; /* trigger bits complete synchronously */
        case S3_SPI_ADDR_REG:        return s->addr;
        case S3_SPI_CTRL_REG:        return s->ctrl;
        case S3_SPI_CLOCK_REG:       return s->clock;
        case S3_SPI_USER_REG:        return s->user;
        case S3_SPI_USER1_REG:       return s->user1;
        case S3_SPI_USER2_REG:       return s->user2;
        case S3_SPI_MS_DLEN_REG:     return s->mosi_dlen;
        case S3_SPI_MISC_REG:        return s->pin;
        case S3_SPI_DIN_MODE_REG:    return s->din_mode;
        case S3_SPI_DIN_NUM_REG:     return s->din_num;
        case S3_SPI_DOUT_MODE_REG:   return s->dout_mode;
        case S3_SPI_DMA_CONF_REG:    return s->dma_conf;
        case S3_SPI_DMA_INT_ENA_REG: return s->dma_int_ena;
        case S3_SPI_DMA_INT_CLR_REG:
        case S3_SPI_DMA_INT_SET_REG: return 0u;
        case S3_SPI_DMA_INT_RAW_REG: return s->dma_int_raw;
        case S3_SPI_DMA_INT_ST_REG:
            return s->dma_int_raw & s->dma_int_ena;
        case S3_SPI_SLAVE_REG:       return s->slave;
        case S3_SPI_SLAVE1_REG:      return s->slave1;
        case S3_SPI_CLK_GATE_REG:    return s->clk_gate;
        case S3_SPI_DATE_REG:        return s->date;
        default:
            if (off >= S3_SPI_W0_REG &&
                off < S3_SPI_W0_REG + sizeof(s->w))
                return s->w[(off - S3_SPI_W0_REG) / 4u];
            return s->owner->fallback_read
                ? s->owner->fallback_read(s->owner->fallback_ctx, addr) : 0u;
        }
    }

    switch (off) {
    case ESP32_SPI_CMD_REG:      return 0u;
    case ESP32_SPI_ADDR_REG:     return s->addr;
    case ESP32_SPI_USER_REG:     return s->user;
    case ESP32_SPI_USER1_REG:    return s->user1;
    case ESP32_SPI_USER2_REG:    return s->user2;
    case ESP32_SPI_MOSI_DLEN_REG: return s->mosi_dlen;
    case ESP32_SPI_MISO_DLEN_REG: return s->miso_dlen;
    case ESP32_SPI_PIN_REG:      return s->pin;
    case ESP32_SPI_SLAVE_REG:
        return s->slave | (s->trans_done ? SPI_TRANS_DONE : 0u);
    case ESP32_SPI_EXT2_REG: {
        /* Fast mode completes transfers synchronously, but real software can
         * deliberately sample the read-only FSM while a dedicated command is
         * in flight. Preserve one observable command-phase state before the
         * controller becomes idle. */
        uint32_t state = s->fsm_observation;
        s->fsm_observation = 0u;
        return state;
    }
    case ESP32_SPI_DMA_CONF_REG: return s->dma_conf;
    case ESP32_SPI_DMA_OUT_LINK_REG: return s->dma_out_link;
    case ESP32_SPI_DMA_IN_LINK_REG: return s->dma_in_link;
    case ESP32_SPI_DMA_STATUS_REG:
        return ((s->dma_out_link & SPI_DMA_LINK_START) ? 2u : 0u) |
               ((s->dma_in_link & SPI_DMA_LINK_START) ? 1u : 0u);
    case ESP32_SPI_DMA_INT_ENA_REG: return s->dma_int_ena;
    case ESP32_SPI_DMA_INT_RAW_REG: return s->dma_int_raw;
    case ESP32_SPI_DMA_INT_ST_REG: return s->dma_int_raw & s->dma_int_ena;
    case ESP32_SPI_DMA_INT_CLR_REG: return 0u;
    case ESP32_SPI_IN_ERR_EOF_DES_REG: return s->dma_in_err_eof_desc;
    case ESP32_SPI_IN_SUC_EOF_DES_REG: return s->dma_in_suc_eof_desc;
    case ESP32_SPI_INLINK_DSCR_REG: return s->dma_inlink_dscr;
    case ESP32_SPI_INLINK_DSCR_BF0_REG: return s->dma_inlink_dscr_bf0;
    case ESP32_SPI_INLINK_DSCR_BF1_REG: return s->dma_inlink_dscr_bf1;
    case ESP32_SPI_OUT_EOF_BFR_DES_REG: return s->dma_out_eof_bfr_desc;
    case ESP32_SPI_OUT_EOF_DES_REG: return s->dma_out_eof_desc;
    case ESP32_SPI_OUTLINK_DSCR_REG: return s->dma_outlink_dscr;
    case ESP32_SPI_OUTLINK_DSCR_BF0_REG: return s->dma_outlink_dscr_bf0;
    case ESP32_SPI_OUTLINK_DSCR_BF1_REG: return s->dma_outlink_dscr_bf1;
    case ESP32_SPI_DATE_REG: return s->date;
    default:
        if (off >= ESP32_SPI_W0_REG &&
            off < ESP32_SPI_W0_REG + sizeof(s->w))
            return s->w[(off - ESP32_SPI_W0_REG) / 4u];
        return 0u;
    }
}

static void gp_spi_write(void *ctx, uint32_t addr, uint32_t val) {
    spi_display_t *s = ctx;
    uint32_t off = addr - s->instance->base;
    if (gp_spi_is_s3(s)) {
        switch (off) {
        case S3_SPI_CMD_REG:
            s->cmd_config = val & S3_SPI_CMD_CONFIG_MASK;
            if ((val & (S3_SPI_CMD_USR | S3_SPI_CMD_UPDATE)) != 0u &&
                (!s->system_clock_enabled || s->system_reset_asserted ||
                 (s->clk_gate & S3_SPI_INTERNAL_CLK_EN) == 0u))
                return;
            /* UPDATE synchronizes the retained APB configuration into the
             * functional engine and self-clears immediately in fast mode. */
            if (val & S3_SPI_CMD_USR) gp_spi_transact(s);
            return;
        case S3_SPI_ADDR_REG:      s->addr = val; return;
        case S3_SPI_CTRL_REG:      s->ctrl = val; return;
        case S3_SPI_CLOCK_REG:     s->clock = val; return;
        case S3_SPI_USER_REG:      s->user = val; return;
        case S3_SPI_USER1_REG:     s->user1 = val; return;
        case S3_SPI_USER2_REG:     s->user2 = val; return;
        case S3_SPI_MS_DLEN_REG:
            s->mosi_dlen = s->miso_dlen = val & 0x0003FFFFu;
            return;
        case S3_SPI_MISC_REG:      s->pin = val; return;
        case S3_SPI_DIN_MODE_REG:  s->din_mode = val & 0x0001FFFFu; return;
        case S3_SPI_DIN_NUM_REG:   s->din_num = val & 0x0000FFFFu; return;
        case S3_SPI_DOUT_MODE_REG: s->dout_mode = val & 0x000001FFu; return;
        case S3_SPI_DMA_CONF_REG:
            /* FIFO reset triggers self-clear. Empty/full bits are hardware
             * status and remain at their reset-idle value in fast mode. */
            s->dma_conf = (val & ((3u << 27) | (0xFu << 18))) | 3u;
            return;
        case S3_SPI_DMA_INT_ENA_REG:
            s->dma_int_ena = val & S3_SPI_DMA_INT_MASK;
            gp_spi_intr_update(s);
            return;
        case S3_SPI_DMA_INT_CLR_REG:
            s->dma_int_raw &= ~(val & S3_SPI_DMA_INT_MASK);
            gp_spi_intr_update(s);
            return;
        case S3_SPI_DMA_INT_RAW_REG:
            return; /* read-only */
        case S3_SPI_DMA_INT_SET_REG:
            s->dma_int_raw |= val & S3_SPI_DMA_INT_MASK;
            gp_spi_intr_update(s);
            return;
        case S3_SPI_DMA_INT_ST_REG:
            return; /* read-only */
        case S3_SPI_SLAVE_REG:
            s->slave = val & ~S3_SPI_SLAVE_SOFT_RESET;
            return;
        case S3_SPI_SLAVE1_REG:   s->slave1 = val; return;
        case S3_SPI_CLK_GATE_REG: s->clk_gate = val & 7u; return;
        case S3_SPI_DATE_REG:     s->date = val & 0x0FFFFFFFu; return;
        default:
            if (off >= S3_SPI_W0_REG &&
                off < S3_SPI_W0_REG + sizeof(s->w)) {
                s->w[(off - S3_SPI_W0_REG) / 4u] = val;
                return;
            }
            gp_spi_report_unsupported(s, addr, val);
            return;
        }
    }

    switch (off) {
    case ESP32_SPI_CMD_REG:
        if (val & ESP32_SPI_CMD_USR) gp_spi_transact(s);
        if (val & ESP32_SPI_CMD_DEDICATED_MASK)
            s->fsm_observation = SPI_FSM_COMMAND;
        break;
    case ESP32_SPI_ADDR_REG:     s->addr = val; break;
    case ESP32_SPI_USER_REG:     s->user = val; break;
    case ESP32_SPI_USER1_REG:    s->user1 = val; break;
    case ESP32_SPI_USER2_REG:    s->user2 = val; break;
    case ESP32_SPI_MOSI_DLEN_REG: s->mosi_dlen = val; break;
    case ESP32_SPI_MISO_DLEN_REG: s->miso_dlen = val; break;
    case ESP32_SPI_PIN_REG:      s->pin = val; break;
    case ESP32_SPI_SLAVE_REG:
        /* SPI_TRANS_DONE is writable in both directions, not just
         * write-zero-to-clear. ESP-IDF's spi_ll_set_int_stat() sets it
         * deliberately to raise the interrupt with no transfer behind it:
         * that is how spi_master kicks off the first queued transaction,
         * whose setup happens in the ISR. Ignoring the set left the driver
         * blocked on its completion semaphore forever, so the public
         * spi_master API did not work at all -- only register-level display
         * libraries did, which is all a stock CYD ROM happens to use. */
        s->slave = val & ~SPI_TRANS_DONE;
        s->trans_done = (val & SPI_TRANS_DONE) ? 1 : 0;
        gp_spi_intr_update(s);
        break;
    case ESP32_SPI_DMA_CONF_REG:
        s->dma_conf = val;
        if (val & SPI_DMA_OUT_RST)
            s->dma_out_link &= ~SPI_DMA_LINK_START;
        if (val & SPI_DMA_IN_RST)
            s->dma_in_link &= ~SPI_DMA_LINK_START;
        break;
    case ESP32_SPI_DMA_OUT_LINK_REG:
        if (spi_dbg(&spi_dbg_dma))
            fprintf(stderr, "[SPIDMA] SPI%d OUT_LINK <- 0x%08X\n",
                    s->host_num, val);
        s->dma_out_link = val &
                (SPI_DMA_LINK_ADDR_MASK | SPI_DMA_LINK_START);
        if (val & SPI_DMA_LINK_STOP)
            s->dma_out_link &= ~SPI_DMA_LINK_START;
        break;
    case ESP32_SPI_DMA_IN_LINK_REG:
        if (spi_dbg(&spi_dbg_dma))
            fprintf(stderr, "[SPIDMA] SPI%d IN_LINK <- 0x%08X\n",
                    s->host_num, val);
        s->dma_in_link = val &
                (SPI_DMA_LINK_ADDR_MASK | SPI_DMA_LINK_START);
        if (val & SPI_DMA_LINK_STOP)
            s->dma_in_link &= ~SPI_DMA_LINK_START;
        break;
    case ESP32_SPI_DMA_INT_ENA_REG:
        s->dma_int_ena = val & SPI_DMA_INT_MASK;
        gp_spi_intr_update(s);
        break;
    case ESP32_SPI_DMA_INT_CLR_REG:
        s->dma_int_raw &= ~(val & SPI_DMA_INT_MASK);
        gp_spi_intr_update(s);
        break;
    default:
        if (off >= ESP32_SPI_W0_REG &&
            off < ESP32_SPI_W0_REG + sizeof(s->w))
            s->w[(off - ESP32_SPI_W0_REG) / 4u] = val;
        break;
    }
}

/* ---- public API ---- */

static bool gp_spi_pin_valid(const flexe_gp_spi_t *spi, int pin) {
    if (!spi || pin < 0 || pin >= 64) return false;
    const flexe_target_desc_t *target = spi->target;
    if (target->capabilities & FLEXE_TARGET_CAP_GPIO_V1)
        return (unsigned)pin < target->gpio.gpio_count &&
               (target->gpio.valid_gpio_mask & (UINT64_C(1) << pin)) != 0u;
    return (unsigned)pin < target->io_mux.gpio_count;
}

static bool gp_spi_geometry_valid(const flexe_target_desc_t *target) {
    if (!target || !(target->capabilities & FLEXE_TARGET_CAP_GP_SPI))
        return false;
    const flexe_gp_spi_desc_t *desc = &target->gp_spi;
    uint32_t minimum_size = desc->layout == FLEXE_GP_SPI_LAYOUT_ESP32
        ? ESP32_SPI_DATE_REG + sizeof(uint32_t)
        : S3_SPI_DATE_REG + sizeof(uint32_t);
    if ((desc->layout != FLEXE_GP_SPI_LAYOUT_ESP32 &&
         desc->layout != FLEXE_GP_SPI_LAYOUT_S2_S3) ||
        desc->host_count == 0u ||
        desc->host_count > FLEXE_TARGET_GP_SPI_HOST_MAX ||
        desc->register_size < minimum_size ||
        desc->register_size > 0x1000u)
        return false;

    for (unsigned i = 0u; i < desc->host_count; i++) {
        const flexe_gp_spi_instance_desc_t *instance = &desc->instance[i];
        if ((instance->base & 0xFFFu) != 0u ||
            instance->base < target->peripheral_start ||
            instance->base >= target->peripheral_end ||
            desc->register_size > target->peripheral_end - instance->base ||
            instance->interrupt_source >= FLEXE_TARGET_INTERRUPT_SOURCE_MAX ||
            instance->clock_out_signal == FLEXE_TARGET_MATRIX_SIGNAL_NONE ||
            instance->chip_select_count == 0u ||
            instance->chip_select_count > FLEXE_TARGET_GP_SPI_CS_MAX)
            return false;
        if ((target->capabilities & FLEXE_TARGET_CAP_INTERRUPT_MATRIX_V1) &&
            instance->interrupt_source >=
                target->interrupt_matrix.source_count)
            return false;
        if (desc->layout == FLEXE_GP_SPI_LAYOUT_S2_S3 &&
            (!(target->capabilities & FLEXE_TARGET_CAP_GDMA_V1) ||
             instance->gdma_peripheral_id ==
                 FLEXE_TARGET_GDMA_PERIPHERAL_NONE ||
             instance->gdma_peripheral_id >= 64u))
            return false;
        for (unsigned cs = 0u; cs < instance->chip_select_count; cs++)
            if (instance->chip_select_out_signal[cs] ==
                FLEXE_TARGET_MATRIX_SIGNAL_NONE)
                return false;
        bool has_iomux = instance->iomux_clock_pin != FLEXE_TARGET_GPIO_NONE;
        if (has_iomux !=
                (instance->iomux_chip_select0_pin != FLEXE_TARGET_GPIO_NONE) ||
            has_iomux !=
                (instance->iomux_function != FLEXE_TARGET_GPIO_NONE))
            return false;
        if (has_iomux &&
            (instance->iomux_clock_pin >= target->io_mux.gpio_count ||
             instance->iomux_chip_select0_pin >= target->io_mux.gpio_count ||
             instance->iomux_function >
                (target->io_mux.function_mask >>
                 target->io_mux.function_shift)))
            return false;
        for (unsigned old = 0u; old < i; old++)
            if (desc->instance[old].base == instance->base)
                return false;
    }
    return true;
}

static void gp_spi_host_register_reset(spi_display_t *s) {
    if (!s) return;
    s->cmd_config = 0u;
    s->addr = 0u;
    s->user = 0u;
    s->user1 = 0u;
    s->user2 = 0u;
    s->mosi_dlen = 0u;
    s->miso_dlen = 0u;
    s->ctrl = 0u;
    s->clock = 0u;
    s->din_mode = 0u;
    s->din_num = 0u;
    s->dout_mode = 0u;
    s->slave = 0u;
    s->slave1 = 0u;
    s->clk_gate = 0u;
    s->trans_done = 0;
    s->fsm_observation = 0u;
    memset(s->w, 0, sizeof(s->w));
    s->dma_conf = 0u;
    s->dma_out_link = 0u;
    s->dma_in_link = 0u;
    s->dma_int_ena = 0u;
    s->dma_int_raw = 0u;
    s->dma_in_err_eof_desc = 0u;
    s->dma_in_suc_eof_desc = 0u;
    s->dma_inlink_dscr = 0u;
    s->dma_inlink_dscr_bf0 = 0u;
    s->dma_inlink_dscr_bf1 = 0u;
    s->dma_out_eof_bfr_desc = 0u;
    s->dma_out_eof_desc = 0u;
    s->dma_outlink_dscr = 0u;
    s->dma_outlink_dscr_bf0 = 0u;
    s->dma_outlink_dscr_bf1 = 0u;
    s->date = s->owner->target->gp_spi.date_reset;

    if (gp_spi_is_s3(s)) {
        s->ctrl = 0x003C0000u;
        s->clock = 0x80003043u;
        s->user = 0x800000C0u;
        s->user1 = 0xBC010007u;
        s->user2 = 0x78000000u;
        s->pin = 0x0000003Eu;
        s->dma_conf = 0x00000003u;
        s->slave = 0x02800000u;
    } else {
        s->pin = 0x00000007u;
    }
    gp_spi_intr_update(s);
}

static void gp_spi_host_configure(spi_display_t *s,
                                  const spi_display_config_t *cfg) {
    if (s->sd_fd >= 0) close(s->sd_fd);
    flexe_gp_spi_t *owner = s->owner;
    xtensa_mem_t *mem = s->mem;
    esp32_periph_t *periph = s->periph;
    const flexe_gp_spi_instance_desc_t *instance = s->instance;
    unsigned instance_index = s->instance_index;
    int host_num = s->host_num;
    bool system_clock_enabled = s->system_clock_enabled;
    bool system_reset_asserted = s->system_reset_asserted;
    memset(s, 0, sizeof(*s));
    s->owner = owner;
    s->mem = mem;
    s->periph = periph;
    s->instance = instance;
    s->instance_index = instance_index;
    s->host_num = host_num;
    s->system_clock_enabled = system_clock_enabled;
    s->system_reset_asserted = system_reset_asserted;
    if (cfg) s->cfg = *cfg;
    s->xe = 239u;
    s->ye = 319u;
    s->sd_fd = -1;
    s->sd_write_sector = UINT32_MAX;
    gp_spi_host_register_reset(s);
}

static spi_display_config_t gp_spi_empty_board_config(void) {
    spi_display_config_t cfg = {
        .dc_pin = -1,
        .display_cs_pin = -1,
        .display_sck_pin = -1,
        .touch_cs_pin = -1,
        .touch_sck_pin = -1,
        .touch_mosi_pin = -1,
        .touch_miso_pin = -1,
        .sd_cs_pin = -1,
        .sd_sck_pin = -1,
    };
    return cfg;
}

void periph_spi_attach_probe(esp32_periph_t *p, spi_probe_fn fn, void *ctx) {
    flexe_gp_spi_t *spi = periph_gp_spi(p);
    if (!spi) return;
    for (unsigned i = 0u; i < spi->target->gp_spi.host_count; i++) {
        spi->host[i].probe_fn = fn;
        spi->host[i].probe_ctx = ctx;
    }
}

int periph_spi_attach_device(esp32_periph_t *p, int host, int cs_pin,
                             int sck_pin, periph_spi_device_fn fn, void *ctx) {
    return periph_spi_attach_device_ex(p, host, cs_pin, sck_pin, fn, NULL,
                                       ctx);
}

int periph_spi_attach_device_ex(esp32_periph_t *p, int host, int cs_pin,
                                int sck_pin, periph_spi_device_fn fn,
                                periph_spi_select_fn select_fn, void *ctx) {
    flexe_gp_spi_t *spi = periph_gp_spi(p);
    if (!spi || host < 2 ||
        (unsigned)(host - 2) >= spi->target->gp_spi.host_count ||
        !gp_spi_pin_valid(spi, cs_pin) || !gp_spi_pin_valid(spi, sck_pin))
        return -1;

    spi_display_t *s = &spi->host[host - 2];

    int free_slot = -1;
    for (unsigned i = 0; i < SPI_DEVICE_MAX; i++) {
        if (!s->device[i].fn) {
            if (free_slot < 0) free_slot = (int)i;
            continue;
        }
        if (s->device[i].cs_pin == cs_pin) {
            if (fn) {
                s->device[i].sck_pin = sck_pin;
                s->device[i].fn = fn;
                s->device[i].select_fn = select_fn;
                s->device[i].ctx = ctx;
            } else {
                memset(&s->device[i], 0, sizeof(s->device[i]));
            }
            return 0;
        }
    }
    if (!fn) return 0;
    if (free_slot >= 0) {
        unsigned i = (unsigned)free_slot;
        s->device[i].cs_pin = cs_pin;
        s->device[i].sck_pin = sck_pin;
        s->device[i].fn = fn;
        s->device[i].select_fn = select_fn;
        s->device[i].ctx = ctx;
        return 0;
    }
    return -1;
}

void spi_display_gpio_changed(esp32_periph_t *p, int pin, int level) {
    flexe_gp_spi_t *spi = periph_gp_spi(p);
    if (spi) {
        for (unsigned host = 0u;
             host < spi->target->gp_spi.host_count; host++) {
            spi_display_t *target = &spi->host[host];
            for (unsigned i = 0; i < SPI_DEVICE_MAX; i++) {
                if (target->device[i].fn && target->device[i].select_fn &&
                    target->device[i].cs_pin == pin)
                    target->device[i].select_fn(target->device[i].ctx,
                                                level == 0);
            }
        }
    }

    /* Software SPI is a set of board wires, not a GP-SPI host. Keep one copy
     * of its state even though the register model has an instance per host. */
    if (!spi || spi->target->gp_spi.host_count == 0u) return;
    spi_display_t *s = &spi->host[0];
    if (s->cfg.touch_cs_pin < 0 ||
        s->cfg.touch_sck_pin < 0 || s->cfg.touch_mosi_pin < 0 ||
        s->cfg.touch_miso_pin < 0)
        return;

    if (pin == s->cfg.touch_cs_pin) {
        s->touch_gpio_selected = level == 0;
        s->touch_gpio_in = 0;
        s->touch_gpio_in_bits = 0;
        s->touch_gpio_out = 0;
        s->touch_gpio_out_bits = 0;
        periph_gpio_set_input(p, s->cfg.touch_miso_pin, 0);
        return;
    }

    /* XPT2046 uses SPI mode 0. The bit-banged CYD driver raises SCLK, reads
     * MISO, then lowers it, so present the current response bit on that
     * rising edge and sample MOSI at the same point. */
    if (!s->touch_gpio_selected || pin != s->cfg.touch_sck_pin || level == 0)
        return;

    int miso = 0;
    if (s->touch_gpio_out_bits != 0) {
        miso = (s->touch_gpio_out & 0x8000u) != 0;
        s->touch_gpio_out <<= 1;
        s->touch_gpio_out_bits--;
    }
    periph_gpio_set_input(p, s->cfg.touch_miso_pin, miso);

    s->touch_gpio_in = (uint8_t)((s->touch_gpio_in << 1) |
                                 (gpio_level(s, s->cfg.touch_mosi_pin) == 1));
    if (++s->touch_gpio_in_bits == 8) {
        uint8_t command = s->touch_gpio_in;
        if (command & 0x80u) {
            uint16_t value = xpt2046_value(s, command);
            s->touch_gpio_out = (uint16_t)(value << 3);
            s->touch_gpio_out_bits = 16;
            g_touch_bitbang_commands++;
            if (spi_dbg(&spi_dbg_touch))
                fprintf(stderr,
                        "[TOUCH-GPIO] command=0x%02X value=%u pressed=%d\n",
                        command, value, value != 0 && value != 4095);
        }
        s->touch_gpio_in = 0;
        s->touch_gpio_in_bits = 0;
    }
}

uint64_t spi_touch_bitbang_commands(void) {
    return g_touch_bitbang_commands;
}

void periph_disable_spi_display(esp32_periph_t *p) {
    flexe_gp_spi_t *spi = periph_gp_spi(p);
    if (!spi) return;
    spi_display_config_t cfg = gp_spi_empty_board_config();
    for (unsigned i = 0u; i < spi->target->gp_spi.host_count; i++)
        gp_spi_host_configure(&spi->host[i], &cfg);
    spi->cs_seen_high = 0u;
}

void periph_enable_spi_display(esp32_periph_t *p, const spi_display_config_t *cfg) {
    flexe_gp_spi_t *spi = periph_gp_spi(p);
    if (!spi || !cfg) return;
    spi->cs_seen_high = 0u;
    g_touch_bitbang_commands = 0;
    for (unsigned i = 0u; i < spi->target->gp_spi.host_count; i++)
        gp_spi_host_configure(&spi->host[i], cfg);
}

flexe_gp_spi_t *flexe_gp_spi_create(
    esp32_periph_t *p, mmio_read_fn fallback_read,
    mmio_write_fn fallback_write, void *fallback_ctx) {
    if (!p) return NULL;
    xtensa_mem_t *mem = periph_mem(p);
    const flexe_target_desc_t *target = mem_target(mem);
    if (!mem || !gp_spi_geometry_valid(target)) return NULL;

    flexe_gp_spi_t *spi = calloc(1u, sizeof(*spi));
    if (!spi) return NULL;
    spi->mem = mem;
    spi->periph = p;
    spi->target = target;
    spi->fallback_read = fallback_read;
    spi->fallback_write = fallback_write;
    spi->fallback_ctx = fallback_ctx;
    spi_display_config_t cfg = gp_spi_empty_board_config();
    unsigned registered = 0u;
    for (unsigned i = 0u; i < target->gp_spi.host_count; i++) {
        spi_display_t *s = &spi->host[i];
        s->owner = spi;
        s->mem = mem;
        s->periph = p;
        s->instance = &target->gp_spi.instance[i];
        s->instance_index = i;
        s->host_num = (int)i + 2;
        s->system_clock_enabled = true;
        s->sd_fd = -1;
        gp_spi_host_configure(s, &cfg);
        if (mem_register_mmio_range(
                mem, s->instance->base, target->gp_spi.register_size,
                gp_spi_read, gp_spi_write, s) != 0)
            goto fail;
        registered++;
    }
    return spi;

fail:
    for (unsigned i = 0u; i < registered; i++)
        (void)mem_register_mmio_range(
            mem, target->gp_spi.instance[i].base,
            target->gp_spi.register_size,
            fallback_read, fallback_write, fallback_ctx);
    free(spi);
    return NULL;
}

void flexe_gp_spi_destroy(flexe_gp_spi_t *spi) {
    if (!spi) return;
    for (unsigned i = 0u; i < spi->target->gp_spi.host_count; i++) {
        spi_display_t *s = &spi->host[i];
        if (s->sd_fd >= 0) close(s->sd_fd);
        periph_deassert_interrupt(spi->periph,
                                  s->instance->interrupt_source);
        (void)mem_register_mmio_range(
            spi->mem, s->instance->base, spi->target->gp_spi.register_size,
            spi->fallback_read, spi->fallback_write, spi->fallback_ctx);
    }
    free(spi);
}

void flexe_gp_spi_set_system_state(flexe_gp_spi_t *spi, unsigned instance,
                                   bool clock_enabled,
                                   bool reset_asserted) {
    if (!spi || instance >= spi->target->gp_spi.host_count) return;
    spi_display_t *s = &spi->host[instance];
    bool reset_rising = reset_asserted && !s->system_reset_asserted;
    s->system_clock_enabled = clock_enabled;
    s->system_reset_asserted = reset_asserted;
    if (reset_rising) gp_spi_host_register_reset(s);
}
