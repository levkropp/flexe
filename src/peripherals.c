#include "peripherals.h"
#include <stdlib.h>
#include <string.h>

/* ESP32 peripheral base addresses */
#define PERIPH_BASE     0x3FF00000u
#define DPORT_BASE      0x3FF00000u
#define UART0_BASE      0x3FF40000u
#define GPIO_BASE       0x3FF44000u
#define RTC_CNTL_BASE   0x3FF48000u
#define IO_MUX_BASE     0x3FF49000u
#define EFUSE_BASE      0x3FF5A000u
#define TIMG0_BASE      0x3FF5F000u
#define TIMG1_BASE      0x3FF60000u
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
} gpio_state_t;

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
    switch (off) {
    case 0x004: return p->gpio.out;         /* GPIO_OUT_REG */
    case 0x010: return p->gpio.out1;        /* GPIO_OUT1_REG */
    case 0x020: return p->gpio.enable;      /* GPIO_ENABLE_REG */
    case 0x02C: return p->gpio.enable1;     /* GPIO_ENABLE1_REG */
    case 0x03C: return 0;                   /* GPIO_IN_REG: no input */
    case 0x040: return 0;                   /* GPIO_IN1_REG */
    default: return 0;
    }
}

static void gpio_write(void *ctx, uint32_t addr, uint32_t val) {
    esp32_periph_t *p = ctx;
    uint32_t off = addr - GPIO_BASE;
    switch (off) {
    case 0x004: p->gpio.out = val; break;        /* GPIO_OUT_REG */
    case 0x008: p->gpio.out |= val; break;       /* GPIO_OUT_W1TS */
    case 0x00C: p->gpio.out &= ~val; break;      /* GPIO_OUT_W1TC */
    case 0x020: p->gpio.enable = val; break;      /* GPIO_ENABLE_REG */
    case 0x024: p->gpio.enable |= val; break;     /* GPIO_ENABLE_W1TS */
    case 0x028: p->gpio.enable &= ~val; break;    /* GPIO_ENABLE_W1TC */
    default: break;
    }
}

/* ---- RTC_CNTL ---- */

static uint32_t rtc_cntl_read(void *ctx, uint32_t addr) {
    (void)ctx;
    uint32_t off = addr - RTC_CNTL_BASE;
    switch (off) {
    case 0x034: return 1;           /* RESET_STATE: POWERON */
    case 0x038: return 1;           /* STORE0: wakeup cause = power-on */
    case 0x080: return 0;           /* SLP_TIMER_BASE */
    case 0x0A8: return 0x2210;      /* CLK_CONF: clocks ready */
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
    default: break;
    }
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

    /* UART0: page 64 */
    mem_register_mmio(mem, (int)PAGE_OF(UART0_BASE), uart0_read, uart0_write, p);

    /* GPIO: page 68 */
    mem_register_mmio(mem, (int)PAGE_OF(GPIO_BASE), gpio_read, gpio_write, p);

    /* RTC_CNTL: page 72 */
    mem_register_mmio(mem, (int)PAGE_OF(RTC_CNTL_BASE), rtc_cntl_read, rtc_cntl_write, p);

    /* IO_MUX: page 73 */
    mem_register_mmio(mem, (int)PAGE_OF(IO_MUX_BASE), io_mux_read, io_mux_write, p);

    /* EFUSE: page 90 */
    mem_register_mmio(mem, (int)PAGE_OF(EFUSE_BASE), efuse_read, efuse_write, p);

    /* TIMG0: page 95 */
    mem_register_mmio(mem, (int)PAGE_OF(TIMG0_BASE), timg_read, timg_write, p);

    /* TIMG1: page 96 */
    mem_register_mmio(mem, (int)PAGE_OF(TIMG1_BASE), timg_read, timg_write, p);

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
