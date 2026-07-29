#include "peripherals.h"
#include "sandbox_events.h"
#include "xtensa.h"
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

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
#define WDEV_BASE       0x3FF75000u  /* WiFi device (contains RNG register) */
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

/* SPI0 (cache flash controller) / SPI1 (memspi host) state.
 * Enough of the register file is modelled for ESP-IDF's esp_flash probe
 * (memspi_host_driver.c) plus NVS/SPIFFS read/write/erase traffic. */
typedef struct {
    uint32_t addr;       /* SPI_ADDR_REG */
    uint32_t user;       /* SPI_USER_REG */
    uint32_t user1;      /* SPI_USER1_REG (addr/dummy bit lengths) */
    uint32_t user2;      /* SPI_USER2_REG (command value/bitlen) */
    uint32_t mosi_dlen;  /* SPI_MOSI_DLEN_REG */
    uint32_t miso_dlen;  /* SPI_MISO_DLEN_REG */
    uint32_t rd_status;  /* SPI_RD_STATUS_REG (ROM-style RDSR result) */
    uint32_t w[16];      /* SPI_W0..W15 data buffer */
    uint8_t  sr[3];      /* emulated flash status registers SR1/SR2/SR3 */
} spi_state_t;

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

    /* APP_CPU reset state for DPORT */
    bool app_cpu_in_reset;   /* true = core 1 held in reset */

    /* Interrupt matrix: maps each CPU interrupt line to a peripheral source.
     * intr_matrix[core][cpu_int] = peripheral source (0-70), 16 = disabled.
     * Mirrors DPORT_PRO_*_MAP_REG / DPORT_APP_*_MAP_REG hardware. */
    uint8_t intr_matrix[2][32];

    /* Pending peripheral interrupt sources (level-triggered) */
    uint32_t pending_sources[3]; /* 71 sources in 3 words (0-31, 32-63, 64-70) */

    /* CPU pointers for interrupt delivery */
    xtensa_cpu_t *cpu[2];

    /* Cross-core interrupt pending state */
    uint32_t from_cpu_intr[4]; /* FROM_CPU_INTR0..3 registers */

    /* Unhandled access counter */
    int unhandled_count;

    /* ADC input shadow values driven from sandbox stdin. Reads by
     * adc_oneshot_read / adc1_get_raw ROM-stubs pull from here. */
    uint16_t adc_value[40];

    /* SPI flash controllers: [0] = SPI0 (cache), [1] = SPI1 (memspi) */
    spi_state_t spi[2];
};

/* ---- DPORT ---- */

/* ESP32 peripheral interrupt source numbers for cross-core interrupts */
#define FROM_CPU_INTR0_SOURCE 24
#define FROM_CPU_INTR1_SOURCE 25
#define FROM_CPU_INTR2_SOURCE 28
#define FROM_CPU_INTR3_SOURCE 29

/* DPORT offsets for cross-core interrupt registers */
#define DPORT_CPU_INTR_FROM_CPU_0_OFF 0x0DC
#define DPORT_CPU_INTR_FROM_CPU_1_OFF 0x0E0
#define DPORT_CPU_INTR_FROM_CPU_2_OFF 0x0E4
#define DPORT_CPU_INTR_FROM_CPU_3_OFF 0x0E8

/* Internal: scan matrix and set/clear CPU interrupt bits for a source */
static void intr_matrix_update_source(esp32_periph_t *p, int source, bool assert) {
    for (int core = 0; core < 2; core++) {
        if (!p->cpu[core]) continue;
        for (int ci = 0; ci < 32; ci++) {
            if (p->intr_matrix[core][ci] == (uint8_t)source) {
                if (assert) {
                    p->cpu[core]->interrupt |= (1u << ci);
                    p->cpu[core]->irq_check = true;
                } else {
                    p->cpu[core]->interrupt &= ~(1u << ci);
                }
            }
        }
    }
}

static uint32_t dport_read(void *ctx, uint32_t addr) {
    esp32_periph_t *p = ctx;
    uint32_t off = addr - DPORT_BASE;
    switch (off) {
    case 0x018: return p->app_cpu_in_reset ? 1 : 0; /* APPCPU_CTRL_D: reset state */
    case 0x02C: return p->app_cpu_in_reset ? 0 : 1; /* APPCPU_CTRL_A: clock gate */
    case 0x030: return p->app_cpu_in_reset ? 0 : 1; /* APPCPU_CTRL_B: clock enable */
    case 0x040: return 0x0A;        /* PRO_CACHE_CTRL: cache enabled */
    case 0x044: return 0x0A;        /* PRO_CACHE_CTRL1 */
    case 0x058: return 0x0A;        /* APP_CACHE_CTRL: cache enabled */
    case 0x3F0: return 0x80;        /* PRO_DCACHE_DBUG0: cache idle (bits[18:7]=1) */
    case 0x3F4: return 0x80;        /* PRO_DCACHE_DBUG1 */
    case 0x3F8: return 0x80;        /* PRO_DCACHE_DBUG2 */
    case 0x3FC: return 0x80;        /* PRO_DCACHE_DBUG3 */
    case 0x418: return 0x80;        /* APP_DCACHE_DBUG0: cache idle (bits[18:7]=1) */
    case 0x3A0: return 0x16042000;  /* DPORT_DATE */
    /* Cross-core interrupt registers */
    case DPORT_CPU_INTR_FROM_CPU_0_OFF: return p->from_cpu_intr[0];
    case DPORT_CPU_INTR_FROM_CPU_1_OFF: return p->from_cpu_intr[1];
    case DPORT_CPU_INTR_FROM_CPU_2_OFF: return p->from_cpu_intr[2];
    case DPORT_CPU_INTR_FROM_CPU_3_OFF: return p->from_cpu_intr[3];
    default:
        /* Interrupt matrix: PRO_CPU offsets 0x104-0x17C (32 regs),
         *                   APP_CPU offsets 0x204-0x27C (32 regs) */
        if (off >= 0x104 && off <= 0x17C && ((off - 0x104) % 4 == 0)) {
            int cpu_int = (int)(off - 0x104) / 4;
            return p->intr_matrix[0][cpu_int];
        }
        if (off >= 0x204 && off <= 0x27C && ((off - 0x204) % 4 == 0)) {
            int cpu_int = (int)(off - 0x204) / 4;
            return p->intr_matrix[1][cpu_int];
        }
        /* Other interrupt matrix range (0x180-0x1FC, 0x280-0x2FC) — status/misc */
        if (off >= 0x104 && off <= 0x2FC)
            return 16;
        return 0;
    }
}

static void dport_write(void *ctx, uint32_t addr, uint32_t val) {
    esp32_periph_t *p = ctx;
    uint32_t off = addr - DPORT_BASE;
    switch (off) {
    case 0x02C: /* APPCPU_CTRL_A: writing 1 releases APP_CPU from reset */
        if (val & 1) p->app_cpu_in_reset = false;
        break;
    case 0x030: /* APPCPU_CTRL_B: clock gate enable */
        break;
    /* Cross-core interrupt registers: writing 1 asserts, writing 0 deasserts */
    case DPORT_CPU_INTR_FROM_CPU_0_OFF:
        p->from_cpu_intr[0] = val;
        if (val & 1) intr_matrix_update_source(p, FROM_CPU_INTR0_SOURCE, true);
        else         intr_matrix_update_source(p, FROM_CPU_INTR0_SOURCE, false);
        break;
    case DPORT_CPU_INTR_FROM_CPU_1_OFF:
        p->from_cpu_intr[1] = val;
        if (val & 1) intr_matrix_update_source(p, FROM_CPU_INTR1_SOURCE, true);
        else         intr_matrix_update_source(p, FROM_CPU_INTR1_SOURCE, false);
        break;
    case DPORT_CPU_INTR_FROM_CPU_2_OFF:
        p->from_cpu_intr[2] = val;
        if (val & 1) intr_matrix_update_source(p, FROM_CPU_INTR2_SOURCE, true);
        else         intr_matrix_update_source(p, FROM_CPU_INTR2_SOURCE, false);
        break;
    case DPORT_CPU_INTR_FROM_CPU_3_OFF:
        p->from_cpu_intr[3] = val;
        if (val & 1) intr_matrix_update_source(p, FROM_CPU_INTR3_SOURCE, true);
        else         intr_matrix_update_source(p, FROM_CPU_INTR3_SOURCE, false);
        break;
    default:
        /* Interrupt matrix writes: PRO_CPU 0x104-0x17C, APP_CPU 0x204-0x27C */
        if (off >= 0x104 && off <= 0x17C && ((off - 0x104) % 4 == 0)) {
            int cpu_int = (int)(off - 0x104) / 4;
            p->intr_matrix[0][cpu_int] = (uint8_t)(val & 0x7F);
        } else if (off >= 0x204 && off <= 0x27C && ((off - 0x204) % 4 == 0)) {
            int cpu_int = (int)(off - 0x204) / 4;
            p->intr_matrix[1][cpu_int] = (uint8_t)(val & 0x7F);
        }
        break;
    }
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
        sbx_event_t ev = { .kind = SBX_EV_UART_TX, .cycle = 0 };
        ev.uart_tx.uart_num = 0;
        ev.uart_tx.byte = byte;
        sbx_events_emit(&ev);
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

static void gpio_emit_changed(uint32_t prev, uint32_t now, int pin_base) {
    uint32_t diff = prev ^ now;
    while (diff) {
        int bit = __builtin_ctz(diff);
        diff &= ~(1u << bit);
        sbx_event_t ev = { .kind = SBX_EV_GPIO_OUT, .cycle = 0 };
        ev.gpio_out.pin = (uint8_t)(pin_base + bit);
        ev.gpio_out.level = (now >> bit) & 1u;
        sbx_events_emit(&ev);
    }
}

static void gpio_write(void *ctx, uint32_t addr, uint32_t val) {
    esp32_periph_t *p = ctx;
    uint32_t off = addr - GPIO_BASE;
    uint32_t prev_out = p->gpio.out;
    uint32_t prev_out1 = p->gpio.out1;

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
        return;    }

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

    /* Fire sandbox events for any output pins that changed level. */
    if (p->gpio.out != prev_out)
        gpio_emit_changed(prev_out, p->gpio.out, 0);
    if (p->gpio.out1 != prev_out1)
        gpio_emit_changed(prev_out1, p->gpio.out1, 32);
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

/* ---- SPI0/SPI1 (flash controllers) ---- */

#define SPI_CMD_REG      0x00
#define SPI_ADDR_REG     0x04
#define SPI_STATUS_REG   0x10   /* SPI_RD_STATUS_REG */
#define SPI_USER_REG     0x1C
#define SPI_USER1_REG    0x20
#define SPI_USER2_REG    0x24
#define SPI_MOSI_DLEN_REG 0x28
#define SPI_MISO_DLEN_REG 0x2C
#define SPI_W0_REG       0x80

/* SPI_CMD_REG bits */
#define SPI_CMD_USR        (1u << 18)
#define SPI_CMD_FLASH_CE   (1u << 22)
#define SPI_CMD_FLASH_BE   (1u << 23)
#define SPI_CMD_FLASH_SE   (1u << 24)
#define SPI_CMD_FLASH_PP   (1u << 25)
#define SPI_CMD_FLASH_WRSR (1u << 26)
#define SPI_CMD_FLASH_RDSR (1u << 27)
#define SPI_CMD_FLASH_RDID (1u << 28)
#define SPI_CMD_FLASH_READ (1u << 31)

/* SPI_USER_REG bits */
#define SPI_USER_USR_MOSI  (1u << 27)
#define SPI_USER_USR_MISO  (1u << 28)
#define SPI_USER_USR_ADDR  (1u << 30)

#define EMU_FLASH_SIZE   (4 * 1024 * 1024)

/* JEDEC ID of a GigaDevice GD25Q32 (4 MB) — matches ESP-IDF's GD chip
 * table (mfg 0xC8, type 0x40, capacity 0x16). First byte on the wire is
 * the manufacturer ID, so it sits in the low byte of W0. */
#define EMU_FLASH_JEDEC_ID 0x001640C8u

static uint32_t spi_flash_offset(const spi_state_t *s) {
    int bitlen = (int)((s->user1 >> 26) & 0x3F) + 1;
    if (bitlen >= 32) return s->addr;
    return s->addr >> (32 - bitlen);
}

static int spi_data_bytes(uint32_t dlen_reg) {
    int bits = (int)(dlen_reg & 0xFFFFFF) + 1;
    int bytes = (bits + 7) / 8;
    return bytes > 64 ? 64 : bytes;
}

static void spi_flash_read_data(esp32_periph_t *p, spi_state_t *s,
                                uint32_t off, int bytes) {
    memset(s->w, 0xFF, sizeof(s->w));   /* erased flash reads as 0xFF */
    if (off < EMU_FLASH_SIZE && p->mem->flash_data) {
        uint32_t avail = EMU_FLASH_SIZE - off;
        if ((uint32_t)bytes > avail) bytes = (int)avail;
        memcpy(s->w, p->mem->flash_data + off, (size_t)bytes);
    }
}

/* Program: real flash can only clear bits, model with AND */
static void spi_flash_program(esp32_periph_t *p, spi_state_t *s,
                              uint32_t off, int bytes) {
    if (off >= EMU_FLASH_SIZE || !p->mem->flash_data) return;
    uint32_t avail = EMU_FLASH_SIZE - off;
    if ((uint32_t)bytes > avail) bytes = (int)avail;
    const uint8_t *src = (const uint8_t *)s->w;
    for (int i = 0; i < bytes; i++)
        p->mem->flash_data[off + i] &= src[i];
}

static void spi_flash_erase(esp32_periph_t *p, uint32_t off, uint32_t len) {
    if (off >= EMU_FLASH_SIZE || !p->mem->flash_data) return;
    if (len > EMU_FLASH_SIZE - off) len = EMU_FLASH_SIZE - off;
    memset(p->mem->flash_data + off, 0xFF, len);
}

/* Execute a flash command started via SPI_CMD_REG. The emulated controller
 * completes instantly: SPI_CMD_REG always reads back 0 (not busy) and
 * SPI_EXT2_REG reads 0 (state machine idle). */
static void spi_flash_execute(esp32_periph_t *p, spi_state_t *s, uint32_t cmd) {
    if (cmd & SPI_CMD_USR) {
        uint8_t fc = (uint8_t)(s->user2 & 0xFF);  /* SPI_USR_COMMAND_VALUE */
        uint32_t off = spi_flash_offset(s);
        int mosi = spi_data_bytes(s->mosi_dlen);
        int miso = spi_data_bytes(s->miso_dlen);
        switch (fc) {
        case 0x9F:                      /* RDID */
        case 0x90:                      /* REMID */
        case 0xAB:                      /* RES / release power-down */
            s->w[0] = EMU_FLASH_JEDEC_ID;
            break;
        case 0x05: s->w[0] = s->sr[0]; break;   /* RDSR1 */
        case 0x35: s->w[0] = s->sr[1]; break;   /* RDSR2 */
        case 0x15: s->w[0] = s->sr[2]; break;   /* RDSR3 */
        case 0x01:                              /* WRSR */
            s->sr[0] = (uint8_t)(s->w[0] & 0xFF);
            if (mosi >= 2) s->sr[1] = (uint8_t)((s->w[0] >> 8) & 0xFF);
            break;
        case 0x31: s->sr[1] = (uint8_t)(s->w[0] & 0xFF); break;  /* WRSR2 */
        case 0x11: s->sr[2] = (uint8_t)(s->w[0] & 0xFF); break;  /* WRSR3 */
        case 0x06: case 0x04: break;            /* WREN / WRDI */
        case 0x03: case 0x0B: case 0x3B:        /* READ / FAST_READ / DUAL */
        case 0x6B: case 0xBB: case 0xEB:        /* QUAD variants */
            spi_flash_read_data(p, s, off, miso);
            break;
        case 0x02: case 0x32:                   /* PP / quad PP */
            spi_flash_program(p, s, off, mosi);
            break;
        case 0x20: spi_flash_erase(p, off & ~0xFFFu, 0x1000);  break; /* SE */
        case 0x52: spi_flash_erase(p, off & ~0x7FFFu, 0x8000); break; /* BE32 */
        case 0xD8: spi_flash_erase(p, off & ~0xFFFFu, 0x10000); break; /* BE64 */
        case 0x60: case 0xC7:                   /* chip erase */
            spi_flash_erase(p, 0, EMU_FLASH_SIZE);
            break;
        default: break;
        }
        return;
    }

    /* ROM-style dedicated command bits (ROM functions are mostly hooked,
     * but handle them anyway for unhooked paths) */
    if (cmd & SPI_CMD_FLASH_RDID) s->w[0] = EMU_FLASH_JEDEC_ID;
    if (cmd & SPI_CMD_FLASH_RDSR) s->rd_status = s->sr[0] | (s->sr[1] << 8) | (s->sr[2] << 16);
    if (cmd & SPI_CMD_FLASH_WRSR) {
        s->sr[0] = (uint8_t)(s->w[0] & 0xFF);
        s->sr[1] = (uint8_t)((s->w[0] >> 8) & 0xFF);
    }
    if (cmd & SPI_CMD_FLASH_READ)
        spi_flash_read_data(p, s, s->addr, spi_data_bytes(s->miso_dlen));
    if (cmd & SPI_CMD_FLASH_PP)
        spi_flash_program(p, s, s->addr, spi_data_bytes(s->mosi_dlen));
    if (cmd & SPI_CMD_FLASH_SE) spi_flash_erase(p, s->addr & ~0xFFFu, 0x1000);
    if (cmd & SPI_CMD_FLASH_BE) spi_flash_erase(p, s->addr & ~0xFFFFu, 0x10000);
    if (cmd & SPI_CMD_FLASH_CE) spi_flash_erase(p, 0, EMU_FLASH_SIZE);
}

static uint32_t spi_read(void *ctx, uint32_t addr) {
    esp32_periph_t *p = ctx;
    spi_state_t *s = &p->spi[(addr >= SPI0_BASE) ? 0 : 1];
    uint32_t base = (addr >= SPI0_BASE) ? SPI0_BASE : SPI1_BASE;
    uint32_t off = addr - base;
    switch (off) {
    case SPI_CMD_REG:      return 0;           /* command done (not busy) */
    case SPI_ADDR_REG:     return s->addr;
    case SPI_STATUS_REG:   return s->rd_status;
    case SPI_USER_REG:     return s->user;
    case SPI_USER1_REG:    return s->user1;
    case SPI_USER2_REG:    return s->user2;
    case SPI_MOSI_DLEN_REG: return s->mosi_dlen;
    case SPI_MISO_DLEN_REG: return s->miso_dlen;
    default:
        if (off >= SPI_W0_REG && off < SPI_W0_REG + sizeof(s->w))
            return s->w[(off - SPI_W0_REG) / 4];
        return 0;   /* incl. SPI_EXT2_REG (0xF8): state machine idle */
    }
}

static void spi_write(void *ctx, uint32_t addr, uint32_t val) {
    esp32_periph_t *p = ctx;
    spi_state_t *s = &p->spi[(addr >= SPI0_BASE) ? 0 : 1];
    uint32_t base = (addr >= SPI0_BASE) ? SPI0_BASE : SPI1_BASE;
    uint32_t off = addr - base;
    switch (off) {
    case SPI_CMD_REG:      spi_flash_execute(p, s, val); break;
    case SPI_ADDR_REG:     s->addr = val; break;
    case SPI_USER_REG:     s->user = val; break;
    case SPI_USER1_REG:    s->user1 = val; break;
    case SPI_USER2_REG:    s->user2 = val; break;
    case SPI_MOSI_DLEN_REG: s->mosi_dlen = val; break;
    case SPI_MISO_DLEN_REG: s->miso_dlen = val; break;
    default:
        if (off >= SPI_W0_REG && off < SPI_W0_REG + sizeof(s->w))
            s->w[(off - SPI_W0_REG) / 4] = val;
        break;
    }
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

/* ---- WDEV (WiFi device — RNG register) ---- */

static uint32_t wdev_read(void *ctx, uint32_t addr) {
    (void)ctx;
    if (addr == 0x3FF75144u) {
        /* WDEV_RND_REG — return random data */
        static uint64_t rng_state = 0x12345678ABCDEF01ULL;
        /* xorshift64 PRNG — fast, good enough for TLS nonces */
        rng_state ^= rng_state << 13;
        rng_state ^= rng_state >> 7;
        rng_state ^= rng_state << 17;
        return (uint32_t)rng_state;
    }
    return 0;
}

static void wdev_write(void *ctx, uint32_t addr, uint32_t val) {
    (void)ctx; (void)addr; (void)val;
}

/* ---- Default handler (unhandled peripherals) ---- */

static uint32_t default_read(void *ctx, uint32_t addr) {
    esp32_periph_t *p = ctx;
    p->unhandled_count++;
    return 0;
}

static void default_write(void *ctx, uint32_t addr, uint32_t val) {
    esp32_periph_t *p = ctx;
    p->unhandled_count++;
}

/* ---- Public API ---- */

esp32_periph_t *periph_create(xtensa_mem_t *mem) {
    esp32_periph_t *p = calloc(1, sizeof(esp32_periph_t));
    if (!p) return NULL;
    p->mem = mem;
    p->app_cpu_in_reset = true;

    /* Initialize interrupt matrix: all lines disabled (source 16 = none) */
    memset(p->intr_matrix, 16, sizeof(p->intr_matrix));

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

    /* WDEV (WiFi device — RNG register at 0x3FF75144) */
    mem_register_mmio(mem, (int)PAGE_OF(WDEV_BASE), wdev_read, wdev_write, p);

    return p;
}

xtensa_mem_t *periph_mem(esp32_periph_t *p) { return p ? p->mem : NULL; }

int periph_gpio_pin_level(const esp32_periph_t *p, int pin) {
    if (!p || pin < 0 || pin > 39) return -1;
    if (pin < 32) return (int)((p->gpio.out >> pin) & 1u);
    return (int)((p->gpio.out1 >> (pin - 32)) & 1u);
}

void periph_destroy(esp32_periph_t *p) {    free(p);
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

bool periph_app_cpu_released(const esp32_periph_t *p) {
    return p ? !p->app_cpu_in_reset : false;
}

void periph_attach_cpus(esp32_periph_t *p, xtensa_cpu_t *cpu0, xtensa_cpu_t *cpu1) {
    if (!p) return;
    p->cpu[0] = cpu0;
    p->cpu[1] = cpu1;
}

void periph_assert_interrupt(esp32_periph_t *p, int source) {
    if (!p || source < 0 || source > 70) return;
    p->pending_sources[source / 32] |= (1u << (source % 32));
    intr_matrix_update_source(p, source, true);
}

void periph_deassert_interrupt(esp32_periph_t *p, int source) {
    if (!p || source < 0 || source > 70) return;
    p->pending_sources[source / 32] &= ~(1u << (source % 32));
    intr_matrix_update_source(p, source, false);
}

void periph_intr_matrix_set(esp32_periph_t *p, int core, int cpu_int, int source) {
    if (!p || core < 0 || core > 1 || cpu_int < 0 || cpu_int > 31) return;
    p->intr_matrix[core][cpu_int] = (uint8_t)(source & 0x7F);
}

int periph_intr_matrix_get(const esp32_periph_t *p, int core, int cpu_int) {
    if (!p || core < 0 || core > 1 || cpu_int < 0 || cpu_int > 31) return 16;
    return p->intr_matrix[core][cpu_int];
}

void periph_set_adc_value(esp32_periph_t *p, int channel, uint16_t raw) {
    if (!p || channel < 0 || channel >= 40) return;
    p->adc_value[channel] = raw;
}

uint16_t periph_get_adc_value(const esp32_periph_t *p, int channel) {
    if (!p || channel < 0 || channel >= 40) return 0;
    return p->adc_value[channel];
}

void periph_gpio_set_input(esp32_periph_t *p, int pin, int level) {
    if (!p || pin < 0 || pin > 39) return;
    if (pin < 32) {
        uint32_t mask = 1u << pin;
        if (level) p->gpio.in  |=  mask; else p->gpio.in  &= ~mask;
    } else {
        uint32_t mask = 1u << (pin - 32);
        if (level) p->gpio.in1 |=  mask; else p->gpio.in1 &= ~mask;
    }
}
