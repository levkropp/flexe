#include "peripherals.h"
#include <stdlib.h>
#include <string.h>

/* ESP32 peripheral base addresses */
#define PERIPH_BASE     0x3FF00000u
#define DPORT_BASE      0x3FF00000u
#define UART0_BASE      0x3FF40000u
#define UART1_BASE      0x3FF50000u
#define UART2_BASE      0x3FF6E000u
#define SPI1_BASE       0x3FF42000u
#define SPI0_BASE       0x3FF43000u
#define GPIO_BASE       0x3FF44000u
#define RTC_CNTL_BASE   0x3FF48000u
#define SENS_BASE       0x3FF48800u
#define IO_MUX_BASE     0x3FF49000u
#define EFUSE_BASE      0x3FF5A000u
#define TIMG0_BASE      0x3FF5F000u
#define TIMG1_BASE      0x3FF60000u
#define SYSCON_BASE     0x3FF66000u
#define PAGE_SIZE       4096

/* Page index from absolute address */
#define PAGE_OF(addr) (((addr) - PERIPH_BASE) / PAGE_SIZE)

/* UART TX buffer */
#define UART_TX_BUF_SIZE 4096

/* WDT shadow registers per timer group */
typedef struct {
    uint32_t config0;
    uint32_t config1;
    uint32_t config2;
    uint32_t config3;
    uint32_t config4;
    uint32_t config5;
    uint32_t protect;    /* write protect key */
} wdt_state_t;

/* GPIO shadow state */
typedef struct {
    uint32_t out;
    uint32_t out1;       /* GPIOs 32-39 */
    uint32_t enable;
    uint32_t enable1;
    uint32_t in;         /* GPIO_IN_REG */
    uint32_t in1;        /* GPIO_IN1_REG */
    uint32_t status;     /* GPIO_STATUS_REG (interrupt status) */
    uint32_t status1;    /* GPIO_STATUS1_REG */
    uint32_t pin[40];    /* GPIO_PINn_REG */
    uint32_t func_in_sel[256];   /* GPIO_FUNC_IN_SEL_CFG_REG */
    uint32_t func_out_sel[40];   /* GPIO_FUNC_OUT_SEL_CFG_REG */
} gpio_state_t;

/* RTC calibration state machine per timer group */
typedef struct {
    int      cal_started;    /* write to RTCCALICFG detected */
    int      reads_since;    /* reads since cal_started */
} rtc_cal_state_t;

struct esp32_periph {
    xtensa_mem_t *mem;

    /* UART0 */
    uint8_t  uart_tx[UART_TX_BUF_SIZE];
    int      uart_tx_len;
    uart_tx_cb uart_cb;
    void    *uart_cb_ctx;
    uint32_t uart_shadow[64];   /* shadow config registers */

    /* GPIO */
    gpio_state_t gpio;

    /* Timer groups WDT */
    wdt_state_t timg_wdt[2];

    /* RTC calibration state */
    rtc_cal_state_t rtc_cal[2];

    /* Unhandled access counter */
    int unhandled_count;
};

/* ---- DPORT ---- */

static uint32_t dport_read(void *ctx, uint32_t addr) {
    (void)ctx;
    uint32_t off = addr - DPORT_BASE;
    switch (off) {
    case 0x018: return 1;           /* APPCPU_CTRL_D: core 1 in reset */
    case 0x040: return 0x0A;        /* PRO_CACHE_CTRL: cache enabled */
    case 0x044: return 0x0A;        /* PRO_CACHE_CTRL1 */
    case 0x058: return 0x0A;        /* APP_CACHE_CTRL: cache enabled */
    case 0x3A0: return 0x16042000;  /* DPORT_DATE */
    default:
        /* Interrupt matrix: offsets 0x104-0x2FC, return 16 (disabled) */
        if (off >= 0x104 && off <= 0x2FC)
            return 16;
        return 0;
    }
}

static void dport_write(void *ctx, uint32_t addr, uint32_t val) {
    (void)ctx; (void)addr; (void)val;
    /* Accept and drop DPORT writes */
}

/* ---- UART0 ---- */

static uint32_t uart0_read(void *ctx, uint32_t addr) {
    esp32_periph_t *p = ctx;
    uint32_t off = addr - UART0_BASE;
    switch (off) {
    case 0x00: return 0;            /* FIFO read: no RX data */
    case 0x1C: return 0;            /* STATUS: TX FIFO empty = ready */
    default:
        if (off / 4 < 64) return p->uart_shadow[off / 4];
        return 0;
    }
}

static void uart0_write(void *ctx, uint32_t addr, uint32_t val) {
    esp32_periph_t *p = ctx;
    uint32_t off = addr - UART0_BASE;
    if (off == 0x00) {
        /* FIFO write: TX byte */
        uint8_t byte = (uint8_t)(val & 0xFF);
        if (p->uart_tx_len < UART_TX_BUF_SIZE)
            p->uart_tx[p->uart_tx_len++] = byte;
        if (p->uart_cb)
            p->uart_cb(p->uart_cb_ctx, byte);
    } else {
        if (off / 4 < 64) p->uart_shadow[off / 4] = val;
    }
}

/* ---- GPIO ---- */

static uint32_t gpio_read(void *ctx, uint32_t addr) {
    esp32_periph_t *p = ctx;
    uint32_t off = addr - GPIO_BASE;

    /* Basic registers */
    switch (off) {
    case 0x004: return p->gpio.out;         /* GPIO_OUT_REG */
    case 0x008: return 0;                   /* GPIO_OUT_W1TS (write-only) */
    case 0x00C: return 0;                   /* GPIO_OUT_W1TC (write-only) */
    case 0x010: return p->gpio.out1;        /* GPIO_OUT1_REG */
    case 0x020: return p->gpio.enable;      /* GPIO_ENABLE_REG */
    case 0x02C: return p->gpio.enable1;     /* GPIO_ENABLE1_REG */
    case 0x03C: return p->gpio.in;          /* GPIO_IN_REG */
    case 0x040: return p->gpio.in1;         /* GPIO_IN1_REG */
    case 0x044: return p->gpio.status;      /* GPIO_STATUS_REG */
    case 0x048: return 0;                   /* GPIO_STATUS_W1TS (write-only) */
    case 0x04C: return 0;                   /* GPIO_STATUS_W1TC (write-only) */
    case 0x050: return p->gpio.status1;     /* GPIO_STATUS1_REG */
    default: break;
    }

    /* GPIO_PINn_REG: 0x088 + n*4, n=0..39 */
    if (off >= 0x088 && off < 0x088 + 40 * 4) {
        int n = (int)(off - 0x088) / 4;
        return p->gpio.pin[n];
    }

    /* GPIO_FUNC_IN_SEL_CFG_REG: 0x130 + sig*4, sig=0..255 */
    if (off >= 0x130 && off < 0x130 + 256 * 4) {
        int sig = (int)(off - 0x130) / 4;
        return p->gpio.func_in_sel[sig];
    }

    /* GPIO_FUNC_OUT_SEL_CFG_REG: 0x530 + n*4, n=0..39
     * 0x530 = offset 1328. These extend beyond page boundary (page = 4096).
     * But this handler is also registered for the next page. */
    if (off >= 0x530 && off < 0x530 + 40 * 4) {
        int n = (int)(off - 0x530) / 4;
        return p->gpio.func_out_sel[n];
    }

    return 0;
}

static void gpio_write(void *ctx, uint32_t addr, uint32_t val) {
    esp32_periph_t *p = ctx;
    uint32_t off = addr - GPIO_BASE;

    switch (off) {
    case 0x004: p->gpio.out = val; break;        /* GPIO_OUT_REG */
    case 0x008: p->gpio.out |= val; break;       /* GPIO_OUT_W1TS */
    case 0x00C: p->gpio.out &= ~val; break;      /* GPIO_OUT_W1TC */
    case 0x010: p->gpio.out1 = val; break;       /* GPIO_OUT1_REG */
    case 0x014: p->gpio.out1 |= val; break;      /* GPIO_OUT1_W1TS */
    case 0x018: p->gpio.out1 &= ~val; break;     /* GPIO_OUT1_W1TC */
    case 0x020: p->gpio.enable = val; break;      /* GPIO_ENABLE_REG */
    case 0x024: p->gpio.enable |= val; break;     /* GPIO_ENABLE_W1TS */
    case 0x028: p->gpio.enable &= ~val; break;    /* GPIO_ENABLE_W1TC */
    case 0x02C: p->gpio.enable1 = val; break;     /* GPIO_ENABLE1_REG */
    case 0x030: p->gpio.enable1 |= val; break;    /* GPIO_ENABLE1_W1TS */
    case 0x034: p->gpio.enable1 &= ~val; break;   /* GPIO_ENABLE1_W1TC */
    case 0x044: p->gpio.status = val; break;      /* GPIO_STATUS_REG */
    case 0x048: p->gpio.status |= val; break;     /* GPIO_STATUS_W1TS */
    case 0x04C: p->gpio.status &= ~val; break;    /* GPIO_STATUS_W1TC */
    case 0x050: p->gpio.status1 = val; break;     /* GPIO_STATUS1_REG */
    default: break;
    }

    /* GPIO_PINn_REG */
    if (off >= 0x088 && off < 0x088 + 40 * 4) {
        int n = (int)(off - 0x088) / 4;
        p->gpio.pin[n] = val;
        return;
    }

    /* GPIO_FUNC_IN_SEL_CFG_REG */
    if (off >= 0x130 && off < 0x130 + 256 * 4) {
        int sig = (int)(off - 0x130) / 4;
        p->gpio.func_in_sel[sig] = val;
        return;
    }

    /* GPIO_FUNC_OUT_SEL_CFG_REG */
    if (off >= 0x530 && off < 0x530 + 40 * 4) {
        int n = (int)(off - 0x530) / 4;
        p->gpio.func_out_sel[n] = val;
        return;
    }
}

/* ---- RTC_CNTL ---- */

static uint32_t rtc_cntl_read(void *ctx, uint32_t addr) {
    (void)ctx;
    uint32_t off = addr - RTC_CNTL_BASE;
    /* SENS registers start at offset 0x800 within this page */
    if (off >= 0x800) return 0;  /* SENS: return 0 for all */
    switch (off) {
    case 0x00C: return (1u << 30);   /* TIME_UPDATE: time-valid bit always set */
    case 0x010: return 0;           /* TIME_LOW0: RTC timer low word */
    case 0x014: return 0;           /* TIME_HIGH0: RTC timer high word */
    case 0x034: return 1;           /* RESET_STATE: POWERON */
    case 0x038: return 1;           /* STORE0: wakeup cause = power-on */
    case 0x080: return 0;           /* SLP_TIMER_BASE */
    case 0x070: return 0x00000080;  /* RTC_CLK_CONF: bit7=fast_clk_sel, slow_clk active */
    case 0x0A8: return 0x2210;      /* CLK_CONF: clocks ready */
    case 0x0B0: return 0x00280028;  /* RTC_XTAL_FREQ_REG: 40 MHz crystal (both halves) */
    default: return 0;
    }
}

static void rtc_cntl_write(void *ctx, uint32_t addr, uint32_t val) {
    (void)ctx; (void)addr; (void)val;
}

/* ---- IO_MUX ---- */

static uint32_t io_mux_read(void *ctx, uint32_t addr) {
    (void)ctx; (void)addr;
    return 0x1800;   /* Default pin configuration */
}

static void io_mux_write(void *ctx, uint32_t addr, uint32_t val) {
    (void)ctx; (void)addr; (void)val;
}

/* ---- EFUSE ---- */

static uint32_t efuse_read(void *ctx, uint32_t addr) {
    (void)ctx;
    uint32_t off = addr - EFUSE_BASE;
    switch (off) {
    case 0x044: return 0xAABBCCDD;  /* MAC address low */
    case 0x048: return 0x0000EEFF;  /* MAC address high */
    case 0x058: return 0x00000001;  /* Chip revision 1 */
    default: return 0;
    }
}

static void efuse_write(void *ctx, uint32_t addr, uint32_t val) {
    (void)ctx; (void)addr; (void)val;
}

/* ---- TIMG WDT (shared for TIMG0 and TIMG1) ---- */

static uint32_t timg_read(void *ctx, uint32_t addr) {
    esp32_periph_t *p = ctx;
    int group = (addr >= TIMG1_BASE) ? 1 : 0;
    uint32_t base = group ? TIMG1_BASE : TIMG0_BASE;
    uint32_t off = addr - base;
    wdt_state_t *w = &p->timg_wdt[group];

    switch (off) {
    case 0x048: return w->config0;   /* TIMG_WDTCONFIG0_REG */
    case 0x04C: return w->config1;
    case 0x050: return w->config2;
    case 0x054: return w->config3;
    case 0x058: return w->config4;
    case 0x05C: return w->config5;
    case 0x064: return w->protect;   /* TIMG_WDTWPROTECT_REG */
    case 0x068: {                    /* TIMG_RTCCALICFG_REG */
        rtc_cal_state_t *cal = &p->rtc_cal[group];
        if (cal->cal_started) {
            cal->reads_since++;
            if (cal->reads_since <= 1)
                return 0;            /* RDY=0: calibration in progress */
            cal->cal_started = 0;    /* done, reset */
        }
        return 0x00008000;           /* RDY bit 15 set */
    }
    case 0x06C: return (267 << 7);   /* TIMG_RTCCALICFG1_REG: ~267 XTAL cycles per slow_clk */
    default: return 0;
    }
}

static void timg_write(void *ctx, uint32_t addr, uint32_t val) {
    esp32_periph_t *p = ctx;
    int group = (addr >= TIMG1_BASE) ? 1 : 0;
    uint32_t base = group ? TIMG1_BASE : TIMG0_BASE;
    uint32_t off = addr - base;
    wdt_state_t *w = &p->timg_wdt[group];

    switch (off) {
    case 0x048: w->config0 = val; break;
    case 0x04C: w->config1 = val; break;
    case 0x050: w->config2 = val; break;
    case 0x054: w->config3 = val; break;
    case 0x058: w->config4 = val; break;
    case 0x05C: w->config5 = val; break;
    case 0x060: break;               /* TIMG_WDTFEED: accept feed */
    case 0x064: w->protect = val; break;
    case 0x068: {                    /* TIMG_RTCCALICFG_REG: start calibration */
        rtc_cal_state_t *cal = &p->rtc_cal[group];
        cal->cal_started = 1;
        cal->reads_since = 0;
        break;
    }
    default: break;
    }
}

/* ---- SPI0 (flash controller) ---- */

#define SPI_CMD_REG      0x00
#define SPI_STATUS_REG   0x10
#define SPI_CTRL_REG     0x08
#define SPI0_SHADOW_SIZE 64

static uint32_t spi_read(void *ctx, uint32_t addr) {
    esp32_periph_t *p = ctx;
    (void)p;
    uint32_t base = (addr >= SPI0_BASE) ? SPI0_BASE : SPI1_BASE;
    uint32_t off = addr - base;
    switch (off) {
    case SPI_CMD_REG:    return 0;       /* Command done (not busy) */
    case SPI_STATUS_REG: return 0;       /* Status: ready */
    default:             return 0;
    }
}

static void spi_write(void *ctx, uint32_t addr, uint32_t val) {
    (void)ctx; (void)addr; (void)val;
}

/* ---- SYSCON ---- */

static uint32_t syscon_read(void *ctx, uint32_t addr) {
    (void)ctx;
    uint32_t off = addr - SYSCON_BASE;
    switch (off) {
    case 0x000: return 0;       /* SYSCON_SYSCLK_CONF_REG */
    case 0x07C: return 0x16042000; /* SYSCON_DATE_REG */
    default:    return 0;
    }
}

static void syscon_write(void *ctx, uint32_t addr, uint32_t val) {
    (void)ctx; (void)addr; (void)val;
}

/* ---- UART1/UART2 (minimal shadow) ---- */

static uint32_t uart_other_read(void *ctx, uint32_t addr) {
    (void)ctx; (void)addr;
    return 0;
}

static void uart_other_write(void *ctx, uint32_t addr, uint32_t val) {
    (void)ctx; (void)addr; (void)val;
}

/* ---- Default handler (unhandled peripherals) ---- */

static uint32_t default_read(void *ctx, uint32_t addr) {
    esp32_periph_t *p = ctx;
    (void)addr;
    p->unhandled_count++;
    return 0;
}

static void default_write(void *ctx, uint32_t addr, uint32_t val) {
    esp32_periph_t *p = ctx;
    (void)addr; (void)val;
    p->unhandled_count++;
}

/* ---- Public API ---- */

esp32_periph_t *periph_create(xtensa_mem_t *mem) {
    esp32_periph_t *p = calloc(1, sizeof(esp32_periph_t));
    if (!p) return NULL;
    p->mem = mem;

    /* Register default handler on all 128 peripheral pages */
    for (int i = 0; i < 128; i++)
        mem_register_mmio(mem, i, default_read, default_write, p);

    /* Override specific peripherals */
    /* DPORT: pages 0-4 (0x3FF00000 - 0x3FF04FFF) */
    for (int i = 0; i <= 4; i++)
        mem_register_mmio(mem, (int)PAGE_OF(DPORT_BASE) + i, dport_read, dport_write, p);

    /* UART0 */
    mem_register_mmio(mem, (int)PAGE_OF(UART0_BASE), uart0_read, uart0_write, p);

    /* UART1 */
    mem_register_mmio(mem, (int)PAGE_OF(UART1_BASE), uart_other_read, uart_other_write, p);

    /* UART2 */
    mem_register_mmio(mem, (int)PAGE_OF(UART2_BASE), uart_other_read, uart_other_write, p);

    /* SPI1 (general SPI) */
    mem_register_mmio(mem, (int)PAGE_OF(SPI1_BASE), spi_read, spi_write, p);

    /* SPI0 (flash controller) */
    mem_register_mmio(mem, (int)PAGE_OF(SPI0_BASE), spi_read, spi_write, p);

    /* GPIO: page 68 + page 69 (FUNC_OUT_SEL extends beyond 4096) */
    mem_register_mmio(mem, (int)PAGE_OF(GPIO_BASE), gpio_read, gpio_write, p);
    mem_register_mmio(mem, (int)PAGE_OF(GPIO_BASE) + 1, gpio_read, gpio_write, p);

    /* RTC_CNTL */
    mem_register_mmio(mem, (int)PAGE_OF(RTC_CNTL_BASE), rtc_cntl_read, rtc_cntl_write, p);

    /* SENS (sensor) — shares 4KB page with RTC_CNTL (0x3FF48800 is in page 72) */
    /* 0x3FF48800 falls in page 72 same as RTC_CNTL, but page 73 (0x3FF49000) is IO_MUX */
    /* SENS is at offset 0x800 within the RTC_CNTL page — handled by rtc_cntl read/write */

    /* IO_MUX */
    mem_register_mmio(mem, (int)PAGE_OF(IO_MUX_BASE), io_mux_read, io_mux_write, p);

    /* EFUSE */
    mem_register_mmio(mem, (int)PAGE_OF(EFUSE_BASE), efuse_read, efuse_write, p);

    /* TIMG0 */
    mem_register_mmio(mem, (int)PAGE_OF(TIMG0_BASE), timg_read, timg_write, p);

    /* TIMG1 */
    mem_register_mmio(mem, (int)PAGE_OF(TIMG1_BASE), timg_read, timg_write, p);

    /* SYSCON */
    mem_register_mmio(mem, (int)PAGE_OF(SYSCON_BASE), syscon_read, syscon_write, p);

    return p;
}

void periph_destroy(esp32_periph_t *p) {
    free(p);
}

void periph_set_uart_callback(esp32_periph_t *p, uart_tx_cb cb, void *ctx) {
    if (!p) return;
    p->uart_cb = cb;
    p->uart_cb_ctx = ctx;
}

int periph_uart_tx_count(const esp32_periph_t *p) {
    return p ? p->uart_tx_len : 0;
}

const uint8_t *periph_uart_tx_buf(const esp32_periph_t *p) {
    return p ? p->uart_tx : NULL;
}

int periph_unhandled_count(const esp32_periph_t *p) {
    return p ? p->unhandled_count : 0;
}
