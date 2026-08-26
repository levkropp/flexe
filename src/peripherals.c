#include "peripherals.h"
#include "spi_display.h"
#include "sandbox_events.h"
#include "xtensa.h"
#include <errno.h>
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
#define FE2_BASE        0x3FF45000u
#define FE_BASE         0x3FF46000u
#define PHY_BASE        0x3FF4E000u  /* undocumented WiFi PHY calibration window */
#define RTC_CNTL_BASE   0x3FF48000u
#define SENS_BASE       0x3FF48800u
#define IO_MUX_BASE     0x3FF49000u
#define BT_BASE         0x3FF51000u
#define EFUSE_BASE      0x3FF5A000u
#define NRX_PRIVATE_BASE 0x3FF5C000u /* includes documented NRX at +0xC00 */
#define BB_BASE         0x3FF5D000u
#define I2S0_BASE       0x3FF4F000u
#define LEDC_BASE       0x3FF59000u
#define TIMG0_BASE      0x3FF5F000u
#define TIMG1_BASE      0x3FF60000u
#define I2S1_BASE       0x3FF6D000u
#define SYSCON_BASE     0x3FF66000u
#define BT_PRIVATE_BASE 0x3FF71000u
#define WIFI_MAC_BASE   0x3FF73000u  /* WiFi MAC/BB control registers */
#define WIFI_MAC_SIZE   0x2000u
#define WDEV_BASE       0x3FF75000u  /* WiFi device (contains RNG register) */
#define PAGE_SIZE       4096
#define PAGE_WORDS      (PAGE_SIZE / sizeof(uint32_t))

/* Page index from absolute address */
#define PAGE_OF(addr) (((addr) - PERIPH_BASE) / PAGE_SIZE)

/* UART FIFOs / host capture. ESP32 UART hardware has 128-byte FIFOs. */
#define UART_TX_BUF_SIZE 4096
#define UART_RX_FIFO_SIZE 128
#define UART_COUNT 3

/* UART interrupt sources/register bits used by the ESP-IDF driver. */
#define UART_RXFIFO_FULL_INT     (1u << 0)
#define UART_TXFIFO_EMPTY_INT    (1u << 1)
#define UART_RXFIFO_OVF_INT      (1u << 4)
#define UART_RXFIFO_TOUT_INT     (1u << 8)
#define UART_TX_DONE_INT         (1u << 14)
#define UART_INT_VALID_MASK      0x7FFFFu

/* LEDC register offsets and interrupt source (ESP32, not S2/S3). */
#define LEDC_INT_RAW_OFF         0x180u
#define LEDC_INT_ST_OFF          0x184u
#define LEDC_INT_ENA_OFF         0x188u
#define LEDC_INT_CLR_OFF         0x18Cu
#define LEDC_DATE_OFF            0x1FCu
#define LEDC_INTR_SOURCE         43

static uint32_t default_read(void *ctx, uint32_t addr);
static void default_write(void *ctx, uint32_t addr, uint32_t val);

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

/* TG LACT (low-alarm-counter) state — the esp_timer hardware timebase.
 * The 64-bit counter ticks at ccount/DIVIDER (DIVIDER from LACTCONFIG);
 * when the counter reaches the 64-bit alarm with ALARM_EN set, the
 * TGn_LACT_LEVEL interrupt source asserts (level). */
typedef struct {
    uint32_t config;        /* LACTCONFIG */
    uint32_t rtc;           /* LACTRTC */
    uint64_t alarm;         /* LACTALARMHI:LO */
    uint64_t load;          /* LACTLOADHI:LO pending value */
    uint64_t load_ccount;   /* cpu0 ccount when LACTLOAD fired */
    uint32_t int_ena;       /* TIMG_INT_ENA_TIMERS */
    uint32_t int_raw;       /* TIMG_INT_RAW_TIMERS */
    bool     loaded;        /* a LACTLOAD has occurred */
    bool     level;         /* interrupt level currently asserted */
} lact_state_t;

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

/* The ESP32 WiFi/BT binary blobs directly program several undocumented RF,
 * PHY, baseband, and controller windows while calibrating the radio. Most of
 * this traffic is ordinary read/modify/write configuration. Retain an
 * independent register file for each page so updates are visible to the HAL;
 * command/status registers with active behavior stay explicit in the handler
 * (currently the WiFi MAC reset/ready handshake and WDEV RNG). */
typedef struct {
    uint32_t fe2[PAGE_WORDS];
    uint32_t fe[PAGE_WORDS];
    uint32_t phy[PAGE_WORDS];
    uint32_t bt[PAGE_WORDS];
    uint32_t nrx[PAGE_WORDS];
    uint32_t bb[PAGE_WORDS];
    uint32_t bt_private[PAGE_WORDS];
    uint32_t wifi_mac[WIFI_MAC_SIZE / sizeof(uint32_t)];
    uint32_t wdev[PAGE_WORDS];
    uint64_t rng_state;
} radio_state_t;

typedef struct {
    uint8_t  tx[UART_TX_BUF_SIZE];
    int      tx_len;
    uint8_t  rx[UART_RX_FIFO_SIZE];
    uint16_t rx_head;
    uint16_t rx_tail;
    uint16_t rx_count;
    uart_tx_cb cb;
    void    *cb_ctx;
    uint32_t shadow[64];
    uint32_t int_raw;
    uint32_t int_ena;
} uart_state_t;

struct esp32_periph {
    xtensa_mem_t *mem;

    /* Three independent ESP32 UART controllers. */
    uart_state_t uart[UART_COUNT];

    /* GPIO */
    gpio_state_t gpio;

    /* IO_MUX pin configuration registers. Keep a written bitmap so reset
     * defaults are not mistaken for an explicitly selected native function. */
    uint32_t io_mux[64];
    uint64_t io_mux_written;

    /* Timer groups WDT */
    wdt_state_t timg_wdt[2];

    /* Timer groups LACT (low-alarm-counter, esp_timer hardware timebase) */
    lact_state_t lact[2];

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

    /* BT low-power clock registers: DPORT_BT_LPCK_DIV_INT (0xD4) and
     * DPORT_BT_LPCK_DIV_FRAC (0xD8) — the BT lpclk select/div code writes
     * these and reads them back to verify, so they must persist. */
    uint32_t bt_lpck[2];

    /* Unhandled access counter */
    int unhandled_count;

    /* ADC input shadow values driven from sandbox stdin. Reads by
     * adc_oneshot_read / adc1_get_raw ROM-stubs pull from here. */
    uint16_t adc_value[40];

    /* SPI flash controllers: [0] = SPI0 (cache), [1] = SPI1 (memspi) */
    spi_state_t spi[2];

    /* Radio/PHY register state used by the closed-source WiFi/BT HAL. */
    radio_state_t radio;

    /* LEDC register file. Channel duty updates complete synchronously, while
     * interrupt raw/status/enable/clear retain their hardware distinctions. */
    uint32_t ledc_regs[0x200 / sizeof(uint32_t)];

    /* The IDF clock initializer reads these values back before any I2S DMA is
     * active. Keep only that explicitly modelled register visible so future
     * audio traffic still reports as an unimplemented hardware gap. */
    uint32_t i2s_clkm_conf[2];

    /* Flash cache MMU table shadows (DPORT_PRO/APP_FLASH_MMU_TABLE).
     * Entry i (0-63 = DROM 0x3F400000+, 64-127 = IROM 0x400C2000+) holds
     * the 64 KB flash page mapped into that vaddr slot. */
    uint32_t flash_mmu_pro[256];
    uint32_t flash_mmu_app[256];
};

/* Bootloader-style initial flash MMU contents for an app at flash 0x10000:
 * DROM slot S -> page S+1 (vaddr 0x3F400000 maps flash 0x10000), IROM
 * slots for app text at 0x400D0000+ -> pages 5+ (flash 0x50000+, matching
 * the contiguous app image layout). Without this, esp_mm's init computes
 * app paddrs from zeroed tables and believes the app sits at paddr 0. */
/* Reset state of the DPORT flash MMU tables: all entries invalid
 * (SOC_MMU_INVALID = 0x100, i.e. free). The loader marks the app's own
 * DROM pages used once it knows the image layout (loader_seed_flash_mmu);
 * everything else must stay free or esp_mmu_map / ROM spi_flash_mmap find
 * no free vaddr slot and fail with ESP_ERR_NO_MEM (seen as
 * "load_partitions returned 0x101"). */
static void flash_mmu_init_bootloader(esp32_periph_t *p) {
    for (uint32_t s = 0; s < 256; s++) {
        p->flash_mmu_pro[s] = 0x100;
        p->flash_mmu_app[s] = 0x100;
    }
}

/* ---- DPORT ---- */

/* ESP32 peripheral interrupt source numbers for cross-core interrupts */
#define FROM_CPU_INTR0_SOURCE 24
#define FROM_CPU_INTR1_SOURCE 25
#define FROM_CPU_INTR2_SOURCE 26
#define FROM_CPU_INTR3_SOURCE 27

/* ETS_GPIO_INTR_SOURCE */
#define GPIO_INTR_SOURCE 22
#define GPIO_NMI_SOURCE  23

/* GPIO_PINn.INT_ENA field values (register bits 17:13). */
#define GPIO_APP_CPU_INTR_ENA      (1u << 0)
#define GPIO_APP_CPU_NMI_INTR_ENA  (1u << 1)
#define GPIO_PRO_CPU_INTR_ENA      (1u << 2)
#define GPIO_PRO_CPU_NMI_INTR_ENA  (1u << 3)
#define GPIO_SDIO_EXT_INTR_ENA     (1u << 4)

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

/* GPIO has per-pin CPU routing, so its normal/NMI sources may be asserted on
 * one core without being asserted on the other. */
static void intr_matrix_update_source_core(esp32_periph_t *p, int core,
                                           int source, bool assert) {
    if (core < 0 || core > 1 || !p->cpu[core]) return;
    for (int ci = 0; ci < 32; ci++) {
        if (p->intr_matrix[core][ci] != (uint8_t)source) continue;
        if (assert) {
            p->cpu[core]->interrupt |= (1u << ci);
            p->cpu[core]->irq_check = true;
        } else {
            p->cpu[core]->interrupt &= ~(1u << ci);
        }
    }
}

/* Map one flash MMU table entry into the CPU page table (shared for both
 * cores; firmware writes identical mappings per core in practice).
 * Entry layout (real hardware / IDF mmu_ll): 0-63 = DROM0 window
 * (0x3F400000+), 64-127 = IRAM0 cache window (0x40080000+). */
static void flash_mmu_map_entry(esp32_periph_t *p, uint32_t entry, uint32_t val) {
    if (getenv("FLEXE_DBG_FLASH"))
        fprintf(stderr, "[MMUTBL] entry=%u val=0x%X (pp=0x%X)\n", entry, val, val << 16);
    if (entry >= 128) return;
    uint32_t pp = val << 16;                 /* 64 KB units -> bytes */
    if (pp + 0x10000 > (4u * 1024 * 1024)) return;
    xtensa_mem_t *mem = p->mem;
    if (entry < 64) {                        /* DROM window */
        for (uint32_t off = 0; off < 0x10000; off += 4096)
            mem->page_table[(0x3F400000u >> 12) + (entry << 4) + (off >> 12)] =
                mem->flash_data + pp + off;
    } else {                                 /* IROM window */
        /* Real entry layout (IDF mmu_ll): entry 64 ↔ vaddr 0x40000000,
         * so the flash text window 0x400D0000-0x40400000 maps to entries
         * 77-127. Only remap pages in the emulator's flash instruction
         * window — lower vaddrs alias ROM/internal IRAM. */
        uint32_t vbase = 0x40000000u + (entry - 64) * 0x10000u;
        if (vbase >= 0x400C2000u) {
            for (uint32_t off = 0; off < 0x10000; off += 4096)
                mem->page_table[(vbase >> 12) + (off >> 12)] =
                    mem->flash_insn + pp + off;
        }
    }
}

static uint32_t dport_read(void *ctx, uint32_t addr) {
    esp32_periph_t *p = ctx;
    uint32_t off = addr - DPORT_BASE;
    /* Flash MMU tables: PRO 0x3FF10000-0x3FF103FF, APP 0x3FF12000-0x3FF123FF */
    if (off >= 0x10000 && off < 0x10400)
        return p->flash_mmu_pro[(off - 0x10000) >> 2];
    if (off >= 0x12000 && off < 0x12400)
        return p->flash_mmu_app[(off - 0x12000) >> 2];
    switch (off) {
    case 0x018: return p->app_cpu_in_reset ? 1 : 0; /* APPCPU_CTRL_D: reset state */
    case 0x02C: return p->app_cpu_in_reset ? 0 : 1; /* APPCPU_CTRL_A: clock gate */
    case 0x030: return p->app_cpu_in_reset ? 0 : 1; /* APPCPU_CTRL_B: clock enable */
    case 0x0D4: return p->bt_lpck[0];   /* DPORT_BT_LPCK_DIV_INT */
    case 0x0D8: return p->bt_lpck[1];   /* DPORT_BT_LPCK_DIV_FRAC */
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
    /* Flash MMU tables: PRO 0x3FF10000-0x3FF103FF, APP 0x3FF12000-0x3FF123FF.
     * Entry value = 64 KB flash page mapped into that vaddr slot; entries
     * 0-63 = DROM (0x3F400000+), 64-127 = IROM (0x400C2000+) — the same
     * model as stub_cache_flash_mmu_set. */
    if (off >= 0x10000 && off < 0x10400) {
        uint32_t entry = (off - 0x10000) >> 2;
        p->flash_mmu_pro[entry] = val;
        flash_mmu_map_entry(p, entry, val);
        return;
    }
    if (off >= 0x12000 && off < 0x12400) {
        uint32_t entry = (off - 0x12000) >> 2;
        p->flash_mmu_app[entry] = val;
        flash_mmu_map_entry(p, entry, val);
        return;
    }
    switch (off) {
    case 0x0D4: p->bt_lpck[0] = val; break;  /* DPORT_BT_LPCK_DIV_INT */
    case 0x0D8: p->bt_lpck[1] = val; break;  /* DPORT_BT_LPCK_DIV_FRAC */
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

/* ---- UART0/UART1/UART2 ---- */

static const uint32_t uart_bases[UART_COUNT] = {
    UART0_BASE, UART1_BASE, UART2_BASE
};

static const int uart_intr_sources[UART_COUNT] = {34, 35, 36};

static int uart_num_from_addr(uint32_t addr) {
    for (int i = 0; i < UART_COUNT; i++) {
        if (addr >= uart_bases[i] && addr < uart_bases[i] + PAGE_SIZE)
            return i;
    }
    return -1;
}

/* Flexe drains each TX FIFO write immediately into the host callback.  Keep
 * the hardware-visible FIFO empty while still presenting the level/edge
 * interrupts that ESP-IDF's buffered UART driver relies on to dequeue its
 * transmit ring buffer. */
static void uart_intr_update(esp32_periph_t *p, int uart_num) {
    uart_state_t *uart = &p->uart[uart_num];
    int source = uart_intr_sources[uart_num];
    uint32_t mask = 1u << (source % 32);
    bool active = (uart->int_raw & uart->int_ena) != 0;
    if (active)
        p->pending_sources[source / 32] |= mask;
    else
        p->pending_sources[source / 32] &= ~mask;
    intr_matrix_update_source(p, source, active);
}

static void uart_refresh_level_conditions(uart_state_t *uart) {
    uint32_t conf1 = uart->shadow[0x24 / 4];
    uint32_t full_threshold = conf1 & 0x7Fu;
    if (full_threshold > 0 && uart->rx_count >= full_threshold)
        uart->int_raw |= UART_RXFIFO_FULL_INT;
    if (uart->int_ena & UART_TXFIFO_EMPTY_INT)
        uart->int_raw |= UART_TXFIFO_EMPTY_INT;
}

static uint32_t uart_read(void *ctx, uint32_t addr) {
    esp32_periph_t *p = ctx;
    int uart_num = uart_num_from_addr(addr);
    if (uart_num < 0) return 0;
    uart_state_t *uart = &p->uart[uart_num];
    uint32_t off = addr - uart_bases[uart_num];
    switch (off) {
    case 0x00: {                    /* FIFO read */
        if (uart->rx_count == 0) return 0;
        uint8_t byte = uart->rx[uart->rx_tail];
        uart->rx_tail = (uint16_t)((uart->rx_tail + 1) % UART_RX_FIFO_SIZE);
        uart->rx_count--;
        return byte;
    }
    case 0x04: return uart->int_raw;                    /* INT_RAW */
    case 0x08: return uart->int_raw & uart->int_ena;    /* INT_ST */
    case 0x0C: return uart->int_ena;                    /* INT_ENA */
    case 0x10: return 0;            /* INT_CLR is write-only */
    case 0x1C: return uart->rx_count; /* STATUS: RX count; TX count is zero */
    case 0x60:                      /* MEM_RX_STATUS */
        return ((uint32_t)(uart->rx_tail & 0x7FFu) << 2) |
               ((uint32_t)(uart->rx_head & 0x7FFu) << 13);
    default:
        if (off / 4 < 64) return uart->shadow[off / 4];
        return 0;
    }
}

static void uart_write(void *ctx, uint32_t addr, uint32_t val) {
    esp32_periph_t *p = ctx;
    int uart_num = uart_num_from_addr(addr);
    if (uart_num < 0) return;
    uart_state_t *uart = &p->uart[uart_num];
    uint32_t off = addr - uart_bases[uart_num];
    if (off == 0x00) {
        /* FIFO write: TX byte */
        uint8_t byte = (uint8_t)(val & 0xFF);
        if (uart->tx_len < UART_TX_BUF_SIZE)
            uart->tx[uart->tx_len++] = byte;
        if (uart->cb)
            uart->cb(uart->cb_ctx, byte);
        sbx_event_t ev = { .kind = SBX_EV_UART_TX, .cycle = 0 };
        ev.uart_tx.uart_num = (uint8_t)uart_num;
        ev.uart_tx.byte = byte;
        sbx_events_emit(&ev);
        /* The byte has already left our zero-depth FIFO.  TXFIFO_EMPTY is a
         * level condition; TX_DONE records completion of this byte. */
        uart->int_raw |= UART_TXFIFO_EMPTY_INT | UART_TX_DONE_INT;
        uart_intr_update(p, uart_num);
    } else if (off == 0x0C) {       /* INT_ENA */
        uart->int_ena = val & UART_INT_VALID_MASK;
        /* Enabling TXFIFO_EMPTY while the FIFO is empty raises it at once. */
        uart_refresh_level_conditions(uart);
        uart_intr_update(p, uart_num);
    } else if (off == 0x10) {       /* INT_CLR (W1TC) */
        uart->int_raw &= ~(val & UART_INT_VALID_MASK);
        /* FIFO threshold and TX empty are level-triggered. */
        uart_refresh_level_conditions(uart);
        uart_intr_update(p, uart_num);
    } else {
        if (off / 4 < 64) uart->shadow[off / 4] = val;
        if (off == 0x24) {          /* CONF1 threshold/timeout controls */
            uart_refresh_level_conditions(uart);
            if (uart->rx_count > 0 && (val & (1u << 31)))
                uart->int_raw |= UART_RXFIFO_TOUT_INT;
            uart_intr_update(p, uart_num);
        }
    }
}

/* ---- GPIO ---- */

/* Return the latched status bits routed to one INT_ENA destination. */
static uint32_t gpio_routed_status(const esp32_periph_t *p, bool high,
                                   uint32_t route) {
    uint32_t status = high ? p->gpio.status1 : p->gpio.status;
    uint32_t routed = 0;
    int base = high ? 32 : 0;
    int count = high ? 8 : 32;
    for (int bit = 0; bit < count; bit++) {
        uint32_t mask = 1u << bit;
        if (!(status & mask)) continue;
        uint32_t int_ena = (p->gpio.pin[base + bit] >> 13) & 0x1Fu;
        if (int_ena & route) routed |= mask;
    }
    return routed;
}

/* An active level trigger immediately re-latches after a W1TC acknowledge.
 * Edge-triggered bits remain latched until the guest clears them. */
static void gpio_latch_active_levels(esp32_periph_t *p) {
    for (int pin = 0; pin < 40; pin++) {
        uint32_t cfg = p->gpio.pin[pin];
        if (((cfg >> 13) & 0x1Fu) == 0) continue;
        uint32_t int_type = (cfg >> 7) & 0x7u;
        if (int_type != 4 && int_type != 5) continue;

        uint32_t mask = pin < 32 ? (1u << pin) : (1u << (pin - 32));
        uint32_t input = pin < 32 ? p->gpio.in : p->gpio.in1;
        bool high = (input & mask) != 0;
        if ((int_type == 4 && !high) || (int_type == 5 && high)) {
            if (pin < 32) p->gpio.status |= mask;
            else          p->gpio.status1 |= mask;
        }
    }
}

/* Raise/lower the GPIO normal and NMI sources for each CPU according to the
 * latched status and each pin's INT_ENA routing field. */
static void gpio_intr_update(esp32_periph_t *p) {
    gpio_latch_active_levels(p);

    bool app_intr = (gpio_routed_status(p, false, GPIO_APP_CPU_INTR_ENA) |
                     gpio_routed_status(p, true, GPIO_APP_CPU_INTR_ENA)) != 0;
    bool pro_intr = (gpio_routed_status(p, false, GPIO_PRO_CPU_INTR_ENA) |
                     gpio_routed_status(p, true, GPIO_PRO_CPU_INTR_ENA)) != 0;
    bool app_nmi = (gpio_routed_status(p, false, GPIO_APP_CPU_NMI_INTR_ENA) |
                    gpio_routed_status(p, true, GPIO_APP_CPU_NMI_INTR_ENA)) != 0;
    bool pro_nmi = (gpio_routed_status(p, false, GPIO_PRO_CPU_NMI_INTR_ENA) |
                    gpio_routed_status(p, true, GPIO_PRO_CPU_NMI_INTR_ENA)) != 0;

    uint32_t intr_mask = 1u << (GPIO_INTR_SOURCE % 32);
    uint32_t nmi_mask = 1u << (GPIO_NMI_SOURCE % 32);
    if (app_intr || pro_intr) p->pending_sources[0] |= intr_mask;
    else                      p->pending_sources[0] &= ~intr_mask;
    if (app_nmi || pro_nmi) p->pending_sources[0] |= nmi_mask;
    else                    p->pending_sources[0] &= ~nmi_mask;

    intr_matrix_update_source_core(p, 0, GPIO_INTR_SOURCE, pro_intr);
    intr_matrix_update_source_core(p, 1, GPIO_INTR_SOURCE, app_intr);
    intr_matrix_update_source_core(p, 0, GPIO_NMI_SOURCE, pro_nmi);
    intr_matrix_update_source_core(p, 1, GPIO_NMI_SOURCE, app_nmi);
}

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
    case 0x050: return p->gpio.status1;     /* GPIO_STATUS1_REG */
    /* W1T registers read back the current status on real hardware; ISR
     * dispatch code does read-modify-write acks on them. */
    case 0x048: return p->gpio.status;      /* GPIO_STATUS_W1TS */
    case 0x04C: return p->gpio.status;      /* GPIO_STATUS_W1TC */
    case 0x054: return p->gpio.status1;     /* GPIO_STATUS1_W1TS */
    case 0x058: return p->gpio.status1;     /* GPIO_STATUS1_W1TC */
    /* ESP32 per-destination interrupt status mirrors.  Unlike the S2/S3,
     * the original ESP32 groups all low-pin destinations first, followed
     * by the GPIO32-39 mirrors at 0x74-0x84. */
    case 0x060: return gpio_routed_status(p, false, GPIO_APP_CPU_INTR_ENA);
    case 0x064: return gpio_routed_status(p, false, GPIO_APP_CPU_NMI_INTR_ENA);
    case 0x068: return gpio_routed_status(p, false, GPIO_PRO_CPU_INTR_ENA);
    case 0x06C: return gpio_routed_status(p, false, GPIO_PRO_CPU_NMI_INTR_ENA);
    case 0x070: return gpio_routed_status(p, false, GPIO_SDIO_EXT_INTR_ENA);
    case 0x074: return gpio_routed_status(p, true, GPIO_APP_CPU_INTR_ENA);
    case 0x078: return gpio_routed_status(p, true, GPIO_APP_CPU_NMI_INTR_ENA);
    case 0x07C: return gpio_routed_status(p, true, GPIO_PRO_CPU_INTR_ENA);
    case 0x080: return gpio_routed_status(p, true, GPIO_PRO_CPU_NMI_INTR_ENA);
    case 0x084: return gpio_routed_status(p, true, GPIO_SDIO_EXT_INTR_ENA);
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
        if (getenv("FLEXE_GPIODBG"))
            fprintf(stderr, "[GPIO] pin%d -> %d\n", pin_base + bit, (now >> bit) & 1u);
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
    case 0x050: p->gpio.status1 = val & 0xFFu; break;  /* GPIO_STATUS1_REG */
    case 0x054: p->gpio.status1 |= val & 0xFFu; break; /* GPIO_STATUS1_W1TS */
    case 0x058: p->gpio.status1 &= ~(val & 0xFFu); break; /* GPIO_STATUS1_W1TC */
    default: break;
    }
    /* Interrupt source line follows the latched status registers */
    if (off >= 0x044 && off <= 0x058)
        gpio_intr_update(p);

    /* GPIO_PINn_REG */
    if (off >= 0x088 && off < 0x088 + 40 * 4) {
        int n = (int)(off - 0x088) / 4;
        p->gpio.pin[n] = val;
        gpio_intr_update(p);
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
    esp32_periph_t *p = ctx;
    uint32_t off = addr - IO_MUX_BASE;
    uint32_t index = off / 4u;
    if ((off & 3u) == 0 && index < 64u &&
        (p->io_mux_written & (1ULL << index)))
        return p->io_mux[index];
    return 0x1800;   /* Reset/default pin configuration */
}

static void io_mux_write(void *ctx, uint32_t addr, uint32_t val) {
    esp32_periph_t *p = ctx;
    uint32_t off = addr - IO_MUX_BASE;
    uint32_t index = off / 4u;
    if ((off & 3u) == 0 && index < 64u) {
        p->io_mux[index] = val;
        p->io_mux_written |= 1ULL << index;
    }
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

/* ---- TIMG LACT (esp_timer hardware timebase) ---- */

#define LACT_CFG_EN        (1u << 31)   /* TIMG_LACT_EN */
#define LACT_CFG_ALARM_EN  (1u << 10)   /* TIMG_LACT_ALARM_EN */
#define LACT_INT_BIT       (1u << 3)    /* TIMG_LACT_INT_ENA/RAW/ST/CLR */
#define TG_LACT_LEVEL_SRC(g) ((g) == 0 ? 17 : 21)  /* ETS_TGn_LACT_LEVEL */

static uint32_t lact_divider(const lact_state_t *l) {
    uint32_t d = (l->config >> 13) & 0xFFFF;
    return d ? d : 1;
}

/* Live 64-bit LACT counter in timer ticks (cpu0 ccount / DIVIDER). */
static uint64_t lact_counter(const esp32_periph_t *p, int group) {
    const lact_state_t *l = &p->lact[group];
    uint64_t cc = p->cpu[0] ? p->cpu[0]->ccount : 0;
    uint32_t div = lact_divider(l);
    uint64_t ticks = cc / div;
    if (l->loaded) {
        uint64_t base = l->load_ccount / div;
        ticks = ticks - base + l->load;
    }
    return ticks;
}

/* Re-evaluate the alarm condition and drive the level interrupt source.
 * Hardware: INT_RAW sets when counter >= alarm with ALARM_EN; the source
 * line asserts while (INT_RAW & INT_ENA). */
static void lact_eval_irq(esp32_periph_t *p, int group) {
    lact_state_t *l = &p->lact[group];
    if ((l->config & LACT_CFG_ALARM_EN) && lact_counter(p, group) >= l->alarm)
        l->int_raw |= LACT_INT_BIT;
    bool level = (l->int_raw & l->int_ena & LACT_INT_BIT) != 0;
    if (level != l->level) {
        l->level = level;
        intr_matrix_update_source(p, TG_LACT_LEVEL_SRC(group), level);
    }
}

/* Next cpu ccount at which a LACT alarm will fire (for next_timer_event). */
static uint32_t lact_next_fire(esp32_periph_t *p, xtensa_cpu_t *cpu) {
    uint64_t best = UINT32_MAX;
    for (int group = 0; group < 2; group++) {
        lact_state_t *l = &p->lact[group];
        if (!(l->config & LACT_CFG_ALARM_EN)) continue;
        if (l->int_raw & LACT_INT_BIT) continue; /* event already latched */
        uint64_t now = lact_counter(p, group);
        if (now >= l->alarm) return cpu->ccount;   /* fire now */
        uint64_t cycles = (l->alarm - now) * lact_divider(l);
        uint64_t event = (uint64_t)cpu->ccount + cycles;
        if (event > UINT32_MAX) event = UINT32_MAX;
        if (event < best) best = event;
    }
    return (uint32_t)best;
}

/* CPU hooks (registered on both cores, wired into next_timer_event). */
static uint32_t periph_next_event_hook(xtensa_cpu_t *cpu) {
    return lact_next_fire((esp32_periph_t *)cpu->periph_event_ctx, cpu);
}
static void periph_event_hook(xtensa_cpu_t *cpu) {
    esp32_periph_t *p = (esp32_periph_t *)cpu->periph_event_ctx;
    for (int group = 0; group < 2; group++)
        lact_eval_irq(p, group);
}

/* Recompute both cores' next_timer_event after LACT state changes. */
static void lact_kick(esp32_periph_t *p) {
    for (int core = 0; core < 2; core++)
        if (p->cpu[core]) xtensa_recompute_next_timer(p->cpu[core]);
}

/* ---- TIMG WDT (shared for TIMG0 and TIMG1) ---- */

static uint32_t timg_read(void *ctx, uint32_t addr) {
    esp32_periph_t *p = ctx;
    int group = (addr >= TIMG1_BASE) ? 1 : 0;
    uint32_t base = group ? TIMG1_BASE : TIMG0_BASE;
    uint32_t off = addr - base;
    wdt_state_t *w = &p->timg_wdt[group];
    lact_state_t *l = &p->lact[group];

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
    /* LACT (low-alarm-counter) — esp_timer hardware timebase */
    case 0x070: return l->config;
    case 0x074: return l->rtc;
    case 0x078: return (uint32_t)lact_counter(p, group);           /* LACTLO */
    case 0x07C: return (uint32_t)(lact_counter(p, group) >> 32);   /* LACTHI */
    case 0x080: return 0;                                          /* LACTUPDATE */
    case 0x084: return (uint32_t)l->alarm;                         /* LACTALARMLO */
    case 0x088: return (uint32_t)(l->alarm >> 32);                 /* LACTALARMHI */
    case 0x08C: return (uint32_t)l->load;                          /* LACTLOADLO */
    case 0x090: return (uint32_t)(l->load >> 32);                  /* LACTLOADHI */
    case 0x098: return l->int_ena;                                 /* INT_ENA_TIMERS */
    case 0x09C: lact_eval_irq(p, group); return l->int_raw;        /* INT_RAW_TIMERS */
    case 0x0A0: lact_eval_irq(p, group); return l->int_raw & l->int_ena; /* INT_ST_TIMERS */
    default: return 0;
    }
}

static void timg_write(void *ctx, uint32_t addr, uint32_t val) {
    esp32_periph_t *p = ctx;
    int group = (addr >= TIMG1_BASE) ? 1 : 0;
    uint32_t base = group ? TIMG1_BASE : TIMG0_BASE;
    uint32_t off = addr - base;
    wdt_state_t *w = &p->timg_wdt[group];
    lact_state_t *l = &p->lact[group];

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
    /* LACT (low-alarm-counter) — esp_timer hardware timebase */
    case 0x070: l->config = val; lact_eval_irq(p, group); lact_kick(p); break;
    case 0x074: l->rtc = val; break;
    case 0x080: break;               /* LACTUPDATE: reads are live, no latch needed */
    case 0x084: l->alarm = (l->alarm & 0xFFFFFFFF00000000ull) | val;
               lact_eval_irq(p, group); lact_kick(p); break;
    case 0x088: l->alarm = (l->alarm & 0xFFFFFFFFull) | ((uint64_t)val << 32);
               lact_eval_irq(p, group); lact_kick(p); break;
    case 0x08C: l->load = (l->load & 0xFFFFFFFF00000000ull) | val; break;
    case 0x090: l->load = (l->load & 0xFFFFFFFFull) | ((uint64_t)val << 32); break;
    case 0x094:                    /* LACTLOAD: counter := load value */
        l->loaded = true;
        l->load_ccount = p->cpu[0] ? p->cpu[0]->ccount : 0;
        lact_eval_irq(p, group); lact_kick(p);
        break;
    case 0x098: l->int_ena = val; lact_eval_irq(p, group); break;  /* INT_ENA_TIMERS */
    case 0x0A4: l->int_raw &= ~val; lact_eval_irq(p, group); break; /* INT_CLR_TIMERS */
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
#define SPI_CMD_FLASH_WRDI (1u << 29)
#define SPI_CMD_FLASH_WREN (1u << 30)
#define SPI_CMD_FLASH_READ (1u << 31)

/* SPI_USER_REG bits */
#define SPI_USER_USR_MOSI  (1u << 27)
#define SPI_USER_USR_MISO  (1u << 28)
#define SPI_USER_USR_ADDR  (1u << 30)

#define EMU_FLASH_SIZE   (4 * 1024 * 1024)

/* Status-register bits managed by the flash itself.  In particular, ESP-IDF
 * verifies every WREN/WRDI by reading SR1.WEL back before it attempts a
 * program or erase.  Treating those commands as no-ops makes the generic
 * flash driver return ESP_ERR_NOT_FOUND even though the chip was detected. */
#define FLASH_SR_WIP       (1u << 0)
#define FLASH_SR_WEL       (1u << 1)

/* JEDEC ID of a GigaDevice GD25Q32 (4 MB) — matches ESP-IDF's GD chip
 * table (mfg 0xC8, type 0x40, capacity 0x16). First byte on the wire is
 * the manufacturer ID, so it sits in the low byte of W0. */
#define EMU_FLASH_JEDEC_ID 0x001640C8u

/* FLEXE_SPIDBG is deliberately range-filtered: filesystem probes can issue
 * thousands of reads, so an unconditional transaction dump is rarely useful.
 * Accepted values are "all", a single offset, or START-END (base 0).  The
 * historical bare setting keeps tracing the boot/partition area below 128 KB.
 */
static bool spi_debug_offset(uint32_t off) {
    const char *spec = getenv("FLEXE_SPIDBG");
    if (!spec) return false;
    if (*spec == '\0') return off < 0x20000u;
    if (strcmp(spec, "all") == 0) return true;

    errno = 0;
    char *end = NULL;
    unsigned long first = strtoul(spec, &end, 0);
    if (errno || end == spec) return off < 0x20000u;
    unsigned long last = first;
    if (*end == '-') {
        const char *tail = end + 1;
        errno = 0;
        last = strtoul(tail, &end, 0);
        if (errno || end == tail) return off < 0x20000u;
    }
    if (*end != '\0') return off < 0x20000u;
    return (unsigned long)off >= first && (unsigned long)off <= last;
}

static void spi_debug_command(const esp32_periph_t *p, const spi_state_t *s,
                              uint8_t opcode, uint32_t off, int mosi, int miso) {
    if (!spi_debug_offset(off)) return;
    int host = s == &p->spi[0] ? 0 : 1;
    fprintf(stderr,
            "[SPI%d] op=%02X off=0x%06X mosi=%d miso=%d pc=%08X/%08X "
            "user=%08X user1=%08X addr=%08X\n",
            host, opcode, off, mosi, miso,
            p->cpu[0] ? p->cpu[0]->pc : 0,
            p->cpu[1] ? p->cpu[1]->pc : 0,
            s->user, s->user1, s->addr);
}

static uint32_t spi_flash_offset(const spi_state_t *s) {
    int bitlen = (int)((s->user1 >> 26) & 0x3F) + 1;
    /* ESP32 appends dual/quad-I/O mode bits to USER1.USR_ADDR_BITLEN.  The
     * HAL still left-aligns only the whole-byte (24- or 32-bit) flash address
     * in SPI_ADDR, then leaves the low padding bits high.  Round the wire
     * phase down to its address bytes so those mode bits do not become a
     * spurious low address nibble (for example 0x3100FC -> 0x3100FCF). */
    int address_bits = bitlen & ~7;
    if (address_bits <= 0) return 0;
    if (address_bits >= 32) return s->addr;
    return s->addr >> (32 - address_bits);
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
    const uint8_t *dst = (const uint8_t *)s->w;
    if (spi_debug_offset(off)) {
        fprintf(stderr, "[SPIRD] off=0x%X bytes=%d w0=%02X %02X %02X %02X %02X %02X %02X %02X\n",
                off, bytes, dst[0], dst[1], dst[2], dst[3],
                dst[4], dst[5], dst[6], dst[7]);
    }
}

/* Program: real flash can only clear bits, model with AND */
static void spi_flash_program(esp32_periph_t *p, spi_state_t *s,
                              uint32_t off, int bytes) {
    if (off >= EMU_FLASH_SIZE || !p->mem->flash_data) return;
    uint32_t avail = EMU_FLASH_SIZE - off;
    if ((uint32_t)bytes > avail) bytes = (int)avail;
    const uint8_t *src = (const uint8_t *)s->w;
    if (spi_debug_offset(off)) {
        fprintf(stderr,
                "[SPIWR] off=0x%X bytes=%d w0=%02X %02X %02X %02X %02X %02X %02X %02X\n",
                off, bytes, src[0], src[1], src[2], src[3],
                src[4], src[5], src[6], src[7]);
    }
    for (int i = 0; i < bytes; i++)
        p->mem->flash_data[off + i] &= src[i];
}

static void spi_flash_erase(esp32_periph_t *p, uint32_t off, uint32_t len) {
    if (off >= EMU_FLASH_SIZE || !p->mem->flash_data) return;
    if (len > EMU_FLASH_SIZE - off) len = EMU_FLASH_SIZE - off;
    if (spi_debug_offset(off))
        fprintf(stderr, "[SPIERASE] off=0x%X bytes=%u\n", off, len);
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
        spi_debug_command(p, s, fc, off, mosi, miso);
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
            if (!(s->sr[0] & FLASH_SR_WEL)) break;
            s->sr[0] = (uint8_t)((s->sr[0] & (FLASH_SR_WIP | FLASH_SR_WEL)) |
                                 (s->w[0] & ~(FLASH_SR_WIP | FLASH_SR_WEL)));
            if (mosi >= 2) s->sr[1] = (uint8_t)((s->w[0] >> 8) & 0xFF);
            s->sr[0] &= (uint8_t)~FLASH_SR_WEL;
            break;
        case 0x31:                              /* WRSR2 */
            if (s->sr[0] & FLASH_SR_WEL) {
                s->sr[1] = (uint8_t)(s->w[0] & 0xFF);
                s->sr[0] &= (uint8_t)~FLASH_SR_WEL;
            }
            break;
        case 0x11:                              /* WRSR3 */
            if (s->sr[0] & FLASH_SR_WEL) {
                s->sr[2] = (uint8_t)(s->w[0] & 0xFF);
                s->sr[0] &= (uint8_t)~FLASH_SR_WEL;
            }
            break;
        case 0x06: s->sr[0] |= FLASH_SR_WEL; break;   /* WREN */
        case 0x04: s->sr[0] &= (uint8_t)~FLASH_SR_WEL; break; /* WRDI */
        case 0x03: case 0x0B: case 0x3B:        /* READ / FAST_READ / DUAL */
        case 0x6B: case 0xBB: case 0xEB:        /* QUAD variants */
            spi_flash_read_data(p, s, off, miso);
            break;
        case 0x02: case 0x32:                   /* PP / quad PP */
            if (s->sr[0] & FLASH_SR_WEL) {
                spi_flash_program(p, s, off, mosi);
                s->sr[0] &= (uint8_t)~FLASH_SR_WEL;
            }
            break;
        case 0x20:                              /* SE */
            if (s->sr[0] & FLASH_SR_WEL) {
                spi_flash_erase(p, off & ~0xFFFu, 0x1000);
                s->sr[0] &= (uint8_t)~FLASH_SR_WEL;
            }
            break;
        case 0x52:                              /* BE32 */
            if (s->sr[0] & FLASH_SR_WEL) {
                spi_flash_erase(p, off & ~0x7FFFu, 0x8000);
                s->sr[0] &= (uint8_t)~FLASH_SR_WEL;
            }
            break;
        case 0xD8:                              /* BE64 */
            if (s->sr[0] & FLASH_SR_WEL) {
                spi_flash_erase(p, off & ~0xFFFFu, 0x10000);
                s->sr[0] &= (uint8_t)~FLASH_SR_WEL;
            }
            break;
        case 0x60: case 0xC7:                   /* chip erase */
            if (s->sr[0] & FLASH_SR_WEL) {
                spi_flash_erase(p, 0, EMU_FLASH_SIZE);
                s->sr[0] &= (uint8_t)~FLASH_SR_WEL;
            }
            break;
        default: break;
        }
        return;
    }

    /* ROM-style dedicated command bits (ROM functions are mostly hooked,
     * but handle them anyway for unhooked paths) */
    if (cmd & SPI_CMD_FLASH_RDID) s->w[0] = EMU_FLASH_JEDEC_ID;
    if (cmd & SPI_CMD_FLASH_RDSR) s->rd_status = s->sr[0] | (s->sr[1] << 8) | (s->sr[2] << 16);
    if (cmd & SPI_CMD_FLASH_WRDI) s->sr[0] &= (uint8_t)~FLASH_SR_WEL;
    if (cmd & SPI_CMD_FLASH_WREN) s->sr[0] |= FLASH_SR_WEL;
    if ((cmd & SPI_CMD_FLASH_WRSR) && (s->sr[0] & FLASH_SR_WEL)) {
        s->sr[0] = (uint8_t)((s->sr[0] & (FLASH_SR_WIP | FLASH_SR_WEL)) |
                             (s->w[0] & ~(FLASH_SR_WIP | FLASH_SR_WEL)));
        s->sr[1] = (uint8_t)((s->w[0] >> 8) & 0xFF);
        s->sr[0] &= (uint8_t)~FLASH_SR_WEL;
    }
    if (cmd & SPI_CMD_FLASH_READ)
        spi_flash_read_data(p, s, s->addr, spi_data_bytes(s->miso_dlen));
    if ((cmd & SPI_CMD_FLASH_PP) && (s->sr[0] & FLASH_SR_WEL)) {
        spi_flash_program(p, s, s->addr, spi_data_bytes(s->mosi_dlen));
        s->sr[0] &= (uint8_t)~FLASH_SR_WEL;
    }
    if ((cmd & SPI_CMD_FLASH_SE) && (s->sr[0] & FLASH_SR_WEL)) {
        spi_flash_erase(p, s->addr & ~0xFFFu, 0x1000);
        s->sr[0] &= (uint8_t)~FLASH_SR_WEL;
    }
    if ((cmd & SPI_CMD_FLASH_BE) && (s->sr[0] & FLASH_SR_WEL)) {
        spi_flash_erase(p, s->addr & ~0xFFFFu, 0x10000);
        s->sr[0] &= (uint8_t)~FLASH_SR_WEL;
    }
    if ((cmd & SPI_CMD_FLASH_CE) && (s->sr[0] & FLASH_SR_WEL)) {
        spi_flash_erase(p, 0, EMU_FLASH_SIZE);
        s->sr[0] &= (uint8_t)~FLASH_SR_WEL;
    }
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

/* ---- WiFi/BT RF, PHY, baseband, and controller register files ---- */

#define WIFI_MAC_INIT_CTRL 0x3FF73D24u
#define WDEV_RND_REG       0x3FF75144u
#define PHY_CAL_COMMAND    0x3FF4E0C4u

static uint32_t *radio_reg_ptr(esp32_periph_t *p, uint32_t addr) {
    uint32_t page = addr & ~(PAGE_SIZE - 1u);
    uint32_t word = (addr & (PAGE_SIZE - 1u)) / sizeof(uint32_t);

    switch (page) {
    case FE2_BASE:         return &p->radio.fe2[word];
    case FE_BASE:          return &p->radio.fe[word];
    case PHY_BASE:         return &p->radio.phy[word];
    case BT_BASE:          return &p->radio.bt[word];
    case NRX_PRIVATE_BASE: return &p->radio.nrx[word];
    case BB_BASE:          return &p->radio.bb[word];
    case BT_PRIVATE_BASE:  return &p->radio.bt_private[word];
    case WIFI_MAC_BASE:
    case WIFI_MAC_BASE + PAGE_SIZE:
        return &p->radio.wifi_mac[(addr - WIFI_MAC_BASE) /
                                  sizeof(uint32_t)];
    default:
        return NULL;
    }
}

static uint32_t radio_read(void *ctx, uint32_t addr) {
    esp32_periph_t *p = ctx;
    if (addr == PHY_CAL_COMMAND)
        return 0; /* indexed calibration command completes synchronously */
    uint32_t *reg = radio_reg_ptr(p, addr);
    return reg ? *reg : default_read(ctx, addr);
}

static void radio_write(void *ctx, uint32_t addr, uint32_t val) {
    esp32_periph_t *p = ctx;
    uint32_t *reg = radio_reg_ptr(p, addr);
    if (!reg) {
        default_write(ctx, addr, val);
        return;
    }

    if (addr == WIFI_MAC_INIT_CTRL) {
        /* hal_init sets bit 1, then spins until the MAC reports ready in
         * bit 0. Hardware completes this short reset synchronously from the
         * guest's perspective, so expose ready immediately. */
        *reg = val | ((val & (1u << 1)) ? 1u : 0u);
        return;
    }
    *reg = val;
}

/* WDEV is distinct from the MAC register window. Its timestamp/control words
 * are ordinary RMW registers; the random source is active on every read. */
static uint32_t wdev_read(void *ctx, uint32_t addr) {
    esp32_periph_t *p = ctx;
    if (addr == WDEV_RND_REG) {
        /* Per-session xorshift64 stream. It is deterministic for reproducible
         * firmware tests while still changing on every hardware read. */
        p->radio.rng_state ^= p->radio.rng_state << 13;
        p->radio.rng_state ^= p->radio.rng_state >> 7;
        p->radio.rng_state ^= p->radio.rng_state << 17;
        return (uint32_t)p->radio.rng_state;
    }
    return p->radio.wdev[(addr - WDEV_BASE) / sizeof(uint32_t)];
}

static void wdev_write(void *ctx, uint32_t addr, uint32_t val) {
    esp32_periph_t *p = ctx;
    p->radio.wdev[(addr - WDEV_BASE) / sizeof(uint32_t)] = val;
}

/* ---- LEDC PWM controller ---- */

static void ledc_update_irq(esp32_periph_t *p) {
    uint32_t raw = p->ledc_regs[LEDC_INT_RAW_OFF / 4];
    uint32_t ena = p->ledc_regs[LEDC_INT_ENA_OFF / 4];
    uint32_t mask = 1u << (LEDC_INTR_SOURCE % 32);
    if (raw & ena)
        p->pending_sources[LEDC_INTR_SOURCE / 32] |= mask;
    else
        p->pending_sources[LEDC_INTR_SOURCE / 32] &= ~mask;
    intr_matrix_update_source(p, LEDC_INTR_SOURCE, (raw & ena) != 0);
}

static int ledc_channel_from_offset(uint32_t off, uint32_t *channel_base,
                                    int *low_speed) {
    if (off < 0x0A0u) {
        int channel = (int)(off / 0x14u);
        if (channel < 8) {
            *channel_base = (uint32_t)channel * 0x14u;
            *low_speed = 0;
            return channel;
        }
    } else if (off < 0x140u) {
        int channel = (int)((off - 0x0A0u) / 0x14u);
        if (channel < 8) {
            *channel_base = 0x0A0u + (uint32_t)channel * 0x14u;
            *low_speed = 1;
            return channel;
        }
    }
    return -1;
}

static uint32_t ledc_read(void *ctx, uint32_t addr) {
    esp32_periph_t *p = ctx;
    uint32_t off = addr - LEDC_BASE;
    if ((off & 3u) || off > LEDC_DATE_OFF)
        return default_read(ctx, addr);

    if (off == LEDC_INT_ST_OFF)
        return p->ledc_regs[LEDC_INT_RAW_OFF / 4] &
               p->ledc_regs[LEDC_INT_ENA_OFF / 4];
    if (off == LEDC_INT_CLR_OFF)
        return 0; /* write-only */

    uint32_t channel_base = 0;
    int low_speed = 0;
    if (ledc_channel_from_offset(off, &channel_base, &low_speed) >= 0 &&
        off - channel_base == 0x10u) {
        (void)low_speed;
        return p->ledc_regs[(channel_base + 0x08u) / 4] & 0x01FFFFFFu;
    }

    return p->ledc_regs[off / 4];
}

static void ledc_write(void *ctx, uint32_t addr, uint32_t val) {
    esp32_periph_t *p = ctx;
    uint32_t off = addr - LEDC_BASE;
    if ((off & 3u) || off > LEDC_DATE_OFF) {
        default_write(ctx, addr, val);
        return;
    }

    if (off == LEDC_INT_CLR_OFF) {
        p->ledc_regs[LEDC_INT_RAW_OFF / 4] &= ~val;
        ledc_update_irq(p);
        return;
    }
    if (off == LEDC_INT_RAW_OFF || off == LEDC_INT_ST_OFF)
        return; /* read-only */

    uint32_t channel_base = 0;
    int low_speed = 0;
    int channel = ledc_channel_from_offset(off, &channel_base, &low_speed);
    if (channel >= 0 && off - channel_base == 0x0Cu) {
        /* Duty ramps are not wall-clock asynchronous in the emulator. Latch
         * the programmed duty immediately, self-clear DUTY_START, and expose
         * the corresponding change-complete interrupt bit. */
        p->ledc_regs[off / 4] = val & ~(1u << 31);
        if (val & (1u << 31)) {
            unsigned bit = (unsigned)channel + (low_speed ? 16u : 8u);
            p->ledc_regs[LEDC_INT_RAW_OFF / 4] |= 1u << bit;
        }
        ledc_update_irq(p);
        return;
    }

    p->ledc_regs[off / 4] = val;
    if (off == LEDC_INT_ENA_OFF)
        ledc_update_irq(p);
}

/* ---- I2S clock configuration ---- */

static uint32_t i2s_clock_read(void *ctx, uint32_t addr) {
    esp32_periph_t *p = ctx;
    uint32_t base = addr >= I2S1_BASE ? I2S1_BASE : I2S0_BASE;
    if (addr - base == 0x0ACu)
        return p->i2s_clkm_conf[base == I2S1_BASE ? 1 : 0];
    return default_read(ctx, addr);
}

static void i2s_clock_write(void *ctx, uint32_t addr, uint32_t val) {
    esp32_periph_t *p = ctx;
    uint32_t base = addr >= I2S1_BASE ? I2S1_BASE : I2S0_BASE;
    if (addr - base == 0x0ACu) {
        p->i2s_clkm_conf[base == I2S1_BASE ? 1 : 0] = val;
        return;
    }
    default_write(ctx, addr, val);
}

/* ---- Default handler (unhandled peripherals) ---- */

static uint32_t default_read(void *ctx, uint32_t addr) {
    esp32_periph_t *p = ctx;
    p->unhandled_count++;
    if (getenv("FLEXE_PERIPHDBG"))
        fprintf(stderr, "[PERIPH] unhandled read  0x%08X pc=0x%08X\n", addr, g_dbg_pc);
    return 0;
}

static void default_write(void *ctx, uint32_t addr, uint32_t val) {
    esp32_periph_t *p = ctx;
    p->unhandled_count++;
    if (getenv("FLEXE_PERIPHDBG"))
        fprintf(stderr, "[PERIPH] unhandled write 0x%08X <- 0x%08X pc=0x%08X\n", addr, val, g_dbg_pc);
}

/* ---- Public API ---- */

esp32_periph_t *periph_create(xtensa_mem_t *mem) {
    esp32_periph_t *p = calloc(1, sizeof(esp32_periph_t));
    if (!p) return NULL;
    p->mem = mem;
    p->app_cpu_in_reset = true;
    p->radio.rng_state = 0x12345678ABCDEF01ULL;

    /* Strapping-pin idle levels: GPIO0/5/15 have pull-ups enabled at reset
     * (GPIO2/12 pull-downs read low). Firmware reads GPIO_IN for buttons
     * tied to these pins (e.g. Marauder's BOOT-button on GPIO0) — leaving
     * them low looks like a permanently held button. */
    p->gpio.in = (1u << 0) | (1u << 5) | (1u << 15);

    /* Initialize interrupt matrix: all lines disabled (source 16 = none) */
    memset(p->intr_matrix, 16, sizeof(p->intr_matrix));

    /* LEDC reset state: all eight timers begin held in reset, and DATE is
     * the ESP32 peripheral version value from the vendor register map. */
    for (uint32_t off = 0x140u; off <= 0x178u; off += 8u)
        p->ledc_regs[off / 4] = 1u << 24;
    p->ledc_regs[LEDC_DATE_OFF / 4] = 0x16031700u;

    /* Bootloader-style initial flash MMU contents (app at flash 0x10000) */
    flash_mmu_init_bootloader(p);

    /* Register default handler on all 128 peripheral pages */
    for (int i = 0; i < 128; i++)
        mem_register_mmio(mem, i, default_read, default_write, p);

    /* Override specific peripherals */
    /* DPORT control registers occupy page 0. Pages 1/2/3 are the independent
     * AES/RSA/SHA accelerators and must not be silently swallowed by this
     * handler (SHA installs its own MMIO model during session creation). */
    mem_register_mmio(mem, (int)PAGE_OF(DPORT_BASE),
                      dport_read, dport_write, p);
    /* DPORT flash MMU tables: PRO 0x3FF10000, APP 0x3FF12000 */
    mem_register_mmio(mem, (int)PAGE_OF(0x3FF10000u), dport_read, dport_write, p);
    mem_register_mmio(mem, (int)PAGE_OF(0x3FF12000u), dport_read, dport_write, p);

    /* Three independent UART controllers (interrupt sources 34/35/36). */
    mem_register_mmio(mem, (int)PAGE_OF(UART0_BASE), uart_read, uart_write, p);
    mem_register_mmio(mem, (int)PAGE_OF(UART1_BASE), uart_read, uart_write, p);
    mem_register_mmio(mem, (int)PAGE_OF(UART2_BASE), uart_read, uart_write, p);

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

    /* LEDC PWM controller */
    mem_register_mmio(mem, (int)PAGE_OF(LEDC_BASE), ledc_read, ledc_write, p);

    /* I2S clock-control readback used during peripheral clock init. Other I2S
     * registers deliberately remain visible as unhandled until DMA/audio is
     * modelled. */
    mem_register_mmio(mem, (int)PAGE_OF(I2S0_BASE), i2s_clock_read, i2s_clock_write, p);
    mem_register_mmio(mem, (int)PAGE_OF(I2S1_BASE), i2s_clock_read, i2s_clock_write, p);

    /* TIMG0 */
    mem_register_mmio(mem, (int)PAGE_OF(TIMG0_BASE), timg_read, timg_write, p);

    /* TIMG1 */
    mem_register_mmio(mem, (int)PAGE_OF(TIMG1_BASE), timg_read, timg_write, p);

    /* SYSCON */
    mem_register_mmio(mem, (int)PAGE_OF(SYSCON_BASE), syscon_read, syscon_write, p);

    /* WiFi/BT RF calibration and controller register files. The WiFi MAC
     * spans two pages; WDEV is a separate page containing the RNG source. */
    mem_register_mmio(mem, (int)PAGE_OF(FE2_BASE), radio_read, radio_write, p);
    mem_register_mmio(mem, (int)PAGE_OF(FE_BASE), radio_read, radio_write, p);
    mem_register_mmio(mem, (int)PAGE_OF(PHY_BASE), radio_read, radio_write, p);
    mem_register_mmio(mem, (int)PAGE_OF(BT_BASE), radio_read, radio_write, p);
    mem_register_mmio(mem, (int)PAGE_OF(NRX_PRIVATE_BASE),
                      radio_read, radio_write, p);
    mem_register_mmio(mem, (int)PAGE_OF(BB_BASE), radio_read, radio_write, p);
    mem_register_mmio_range(mem, WIFI_MAC_BASE, WIFI_MAC_SIZE,
                            radio_read, radio_write, p);
    mem_register_mmio(mem, (int)PAGE_OF(WDEV_BASE), wdev_read, wdev_write, p);

    /* Bluetooth controller private register page. */
    mem_register_mmio(mem, (int)PAGE_OF(BT_PRIVATE_BASE),
                      radio_read, radio_write, p);

    return p;
}

xtensa_mem_t *periph_mem(esp32_periph_t *p) { return p ? p->mem : NULL; }

int periph_gpio_pin_level(const esp32_periph_t *p, int pin) {
    if (!p || pin < 0 || pin > 39) return -1;
    if (pin < 32) return (int)((p->gpio.out >> pin) & 1u);
    return (int)((p->gpio.out1 >> (pin - 32)) & 1u);
}

int periph_gpio_output_enabled(const esp32_periph_t *p, int pin) {
    if (!p || pin < 0 || pin > 39) return 0;
    if (pin < 32) return (int)((p->gpio.enable >> pin) & 1u);
    return (int)((p->gpio.enable1 >> (pin - 32)) & 1u);
}

int periph_gpio_out_signal(const esp32_periph_t *p, int pin) {
    if (!p || pin < 0 || pin > 39) return -1;
    return (int)(p->gpio.func_out_sel[pin] & 0x1FFu);
}

int periph_iomux_function(const esp32_periph_t *p, int pin) {
    /* Offsets from the classic ESP32 GPIO_PIN_MUX_REG table. GPIO28..31 are
     * not bonded as general-purpose pins and therefore have no entries. */
    static const uint8_t offsets[40] = {
        0x44, 0x88, 0x40, 0x84, 0x48, 0x6C, 0x60, 0x64,
        0x68, 0x54, 0x58, 0x5C, 0x34, 0x38, 0x30, 0x3C,
        0x4C, 0x50, 0x70, 0x74, 0x78, 0x7C, 0x80, 0x8C,
        0x90, 0x24, 0x28, 0x2C, 0xFF, 0xFF, 0xFF, 0xFF,
        0x1C, 0x20, 0x14, 0x18, 0x04, 0x08, 0x0C, 0x10,
    };
    if (!p || pin < 0 || pin > 39 || offsets[pin] == 0xFF) return -1;
    uint32_t index = offsets[pin] / 4u;
    if (!(p->io_mux_written & (1ULL << index))) return -1;
    return (int)((p->io_mux[index] >> 12) & 7u);
}

void periph_destroy(esp32_periph_t *p) {
    periph_disable_spi_display(p);
    free(p);
}

void periph_set_uart_callback(esp32_periph_t *p, uart_tx_cb cb, void *ctx) {
    periph_set_uart_callback_num(p, 0, cb, ctx);
}

int periph_uart_tx_count(const esp32_periph_t *p) {
    return periph_uart_tx_count_num(p, 0);
}

const uint8_t *periph_uart_tx_buf(const esp32_periph_t *p) {
    return periph_uart_tx_buf_num(p, 0);
}

void periph_set_uart_callback_num(esp32_periph_t *p, int uart_num,
                                  uart_tx_cb cb, void *ctx) {
    if (!p || uart_num < 0 || uart_num >= UART_COUNT) return;
    p->uart[uart_num].cb = cb;
    p->uart[uart_num].cb_ctx = ctx;
}

int periph_uart_tx_count_num(const esp32_periph_t *p, int uart_num) {
    if (!p || uart_num < 0 || uart_num >= UART_COUNT) return 0;
    return p->uart[uart_num].tx_len;
}

const uint8_t *periph_uart_tx_buf_num(const esp32_periph_t *p,
                                      int uart_num) {
    if (!p || uart_num < 0 || uart_num >= UART_COUNT) return NULL;
    return p->uart[uart_num].tx;
}

size_t periph_uart_rx_inject_num(esp32_periph_t *p, int uart_num,
                                 const uint8_t *data, size_t len) {
    if (!p || uart_num < 0 || uart_num >= UART_COUNT ||
        (!data && len != 0)) return 0;
    uart_state_t *uart = &p->uart[uart_num];
    size_t accepted = 0;
    while (accepted < len && uart->rx_count < UART_RX_FIFO_SIZE) {
        uart->rx[uart->rx_head] = data[accepted++];
        uart->rx_head = (uint16_t)((uart->rx_head + 1) % UART_RX_FIFO_SIZE);
        uart->rx_count++;
    }

    /* CONF1: RXFIFO_FULL_THRHD[6:0], RX_TOUT_EN[31]. The host injection
     * represents already-arrived bytes, so publish the timeout condition at
     * once for short packets; larger bursts also assert the FIFO threshold. */
    uint32_t conf1 = uart->shadow[0x24 / 4];
    uart_refresh_level_conditions(uart);
    if (accepted > 0 && (conf1 & (1u << 31)))
        uart->int_raw |= UART_RXFIFO_TOUT_INT;
    if (accepted < len)
        uart->int_raw |= UART_RXFIFO_OVF_INT;
    uart_intr_update(p, uart_num);
    return accepted;
}

size_t periph_uart_rx_inject(esp32_periph_t *p, const uint8_t *data,
                             size_t len) {
    return periph_uart_rx_inject_num(p, 0, data, len);
}

size_t periph_uart_rx_pending_num(const esp32_periph_t *p, int uart_num) {
    if (!p || uart_num < 0 || uart_num >= UART_COUNT) return 0;
    return p->uart[uart_num].rx_count;
}

size_t periph_uart_rx_pending(const esp32_periph_t *p) {
    return periph_uart_rx_pending_num(p, 0);
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
    /* Wire the TIMG LACT timer-event hooks into both cores so esp_timer
     * alarms fire on time (and can wake the cores from WAITI). */
    for (int i = 0; i < 2; i++) {
        xtensa_cpu_t *c = i == 0 ? cpu0 : cpu1;
        if (!c) continue;
        c->periph_event_ctx = p;
        c->periph_next_event = periph_next_event_hook;
        c->periph_event = periph_event_hook;
    }
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
    uint32_t mask = (pin < 32) ? (1u << pin) : (1u << (pin - 32));
    uint32_t *in = (pin < 32) ? &p->gpio.in : &p->gpio.in1;
    int old = (*in & mask) ? 1 : 0;
    int now = level ? 1 : 0;
    if (now) *in |= mask; else *in &= ~mask;
    if (now == old) return;

    /* Edge/level-triggered pin interrupt, per GPIO_PINn_REG config:
     * INT_TYPE [9:7]: 1=rise 2=fall 3=any 4=low 5=high; INT_ENA [17:13]. */
    uint32_t cfg = p->gpio.pin[pin];
    uint32_t int_type = (cfg >> 7) & 0x7;
    bool fire = false;
    if (cfg & (0x1Fu << 13)) {
        switch (int_type) {
        case 1: fire = (now == 1); break;
        case 2: fire = (now == 0); break;
        case 3: fire = true; break;
        case 4: fire = (now == 0); break;
        case 5: fire = (now == 1); break;
        default: break;
        }
    }
    if (fire) {
        if (pin < 32) p->gpio.status  |= mask;
        else          p->gpio.status1 |= mask;
        if (getenv("FLEXE_GPIODBG"))
            fprintf(stderr, "[GPIO] pin%d intr (type=%u ena=0x%X level=%d)\n",
                    pin, int_type, (cfg >> 13) & 0x1Fu, now);
    }
    gpio_intr_update(p);
    if (fire && getenv("FLEXE_GPIODBG")) {
        fprintf(stderr,
                "[GPIO] delivery cpu0=int:%08X ena:%08X ps:%08X "
                "cpu1=int:%08X ena:%08X ps:%08X\n",
                p->cpu[0] ? p->cpu[0]->interrupt : 0,
                p->cpu[0] ? p->cpu[0]->intenable : 0,
                p->cpu[0] ? p->cpu[0]->ps : 0,
                p->cpu[1] ? p->cpu[1]->interrupt : 0,
                p->cpu[1] ? p->cpu[1]->intenable : 0,
                p->cpu[1] ? p->cpu[1]->ps : 0);
    }
}
