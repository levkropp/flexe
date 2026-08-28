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
#define I2C0_BASE       0x3FF53000u
#define I2C1_BASE       0x3FF67000u
#define RMT_BASE        0x3FF56000u
#define GPIO_BASE       0x3FF44000u
#define FE2_BASE        0x3FF45000u
#define FE_BASE         0x3FF46000u
#define PHY_BASE        0x3FF4E000u  /* undocumented WiFi PHY calibration window */
#define RTC_CNTL_BASE   0x3FF48000u
#define RTCIO_BASE      0x3FF48400u
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
#define EMU_FLASH_SIZE  (4u * 1024u * 1024u)

/* Classic ESP32 flash-cache MMU geometry. Each core owns a 256-entry table:
 * DROM0, IRAM0, IRAM1, and IROM0 each consume 64 entries. */
#define FLASH_MMU_ENTRY_COUNT 256u
#define FLASH_MMU_PAGE_SIZE   0x10000u
#define FLASH_MMU_INVALID     0x100u
#define FLASH_MMU_VALUE_MASK  0x1FFu

/* RTC-domain analog register geometry (classic ESP32). RTCIO and SENS share
 * RTC_CNTL's 4 KiB MMIO page, at offsets 0x400 and 0x800 respectively. */
#define RTCIO_REG_FILE_SIZE         0x100u
#define RTCIO_DAC1_OFF              0x084u
#define RTCIO_DAC2_OFF              0x088u
#define RTCIO_DAC_VALUE_MASK        (0xFFu << 19)
#define RTCIO_DAC_XPD               (1u << 18)
#define RTCIO_DAC_XPD_FORCE         (1u << 10)

#define SENS_REG_FILE_SIZE          0x100u
#define SENS_SAR_START_FORCE_OFF    0x02Cu
#define SENS_SAR_MEAS_START1_OFF    0x054u
#define SENS_SAR_MEAS_START2_OFF    0x094u
#define SENS_SAR_EN_PAD_MASK        (0xFFFu << 19)
#define SENS_SAR_START              (1u << 17)
#define SENS_SAR_DONE               (1u << 16)
#define SENS_SAR_CONFIG_MASK        0xFFFE0000u

/* Page index from absolute address */
#define PAGE_OF(addr) (((addr) - PERIPH_BASE) / PAGE_SIZE)

/* UART FIFOs / host capture. ESP32 UART hardware has 128-byte FIFOs. */
#define UART_TX_BUF_SIZE 4096
#define UART_RX_FIFO_SIZE 128
#define UART_COUNT 3

/* Classic ESP32 I2C controller register/FIFO geometry. */
#define I2C_PORT_COUNT       2
#define I2C_DEVICE_COUNT     128
#define I2C_FIFO_SIZE        32
#define I2C_COMMAND_COUNT    16
#define I2C_REG_FILE_SIZE    0x104u
#define I2C_MAX_PENDING_WRITE (1024u * 1024u)

#define I2C_CTR_OFF          0x004u
#define I2C_SR_OFF           0x008u
#define I2C_RXFIFO_ST_OFF    0x014u
#define I2C_FIFO_CONF_OFF    0x018u
#define I2C_DATA_OFF         0x01Cu
#define I2C_INT_RAW_OFF      0x020u
#define I2C_INT_CLR_OFF      0x024u
#define I2C_INT_ENA_OFF      0x028u
#define I2C_INT_ST_OFF       0x02Cu
#define I2C_COMMAND0_OFF     0x058u
#define I2C_DATE_OFF         0x0F8u

#define I2C_CTR_TRANS_START  (1u << 5)
#define I2C_FIFO_RX_RST      (1u << 12)
#define I2C_FIFO_TX_RST      (1u << 13)

#define I2C_INT_RXFIFO_FULL  (1u << 0)
#define I2C_INT_TXFIFO_EMPTY (1u << 1)
#define I2C_INT_RXFIFO_OVF   (1u << 2)
#define I2C_INT_END_DETECT   (1u << 3)
#define I2C_INT_MASTER_DONE  (1u << 6)
#define I2C_INT_TRANS_DONE   (1u << 7)
#define I2C_INT_TRANS_START  (1u << 9)
#define I2C_INT_ACK_ERR      (1u << 10)
#define I2C_INT_VALID_MASK   0x1FFFu

#define I2C_CMD_RESTART      0u
#define I2C_CMD_WRITE        1u
#define I2C_CMD_READ         2u
#define I2C_CMD_STOP         3u
#define I2C_CMD_END          4u
#define I2C_CMD_DONE         (1u << 31)

/* Classic ESP32 dual I2S controller and circular lldesc DMA geometry. */
#define I2S_PORT_COUNT            2
#define I2S_REG_FILE_SIZE         0x100u
#define I2S_RX_FIFO_SIZE          (64u * 1024u)
#define I2S_DMA_MAX_DESCRIPTORS   128
#define I2S_DMA_MAX_BUFFER        4095u

#define I2S_CONF_OFF              0x008u
#define I2S_INT_RAW_OFF           0x00Cu
#define I2S_INT_ST_OFF            0x010u
#define I2S_INT_ENA_OFF           0x014u
#define I2S_INT_CLR_OFF           0x018u
#define I2S_FIFO_CONF_OFF         0x020u
#define I2S_RXEOF_NUM_OFF         0x024u
#define I2S_OUT_LINK_OFF          0x030u
#define I2S_IN_LINK_OFF           0x034u
#define I2S_OUT_EOF_DESC_OFF      0x038u
#define I2S_IN_EOF_DESC_OFF       0x03Cu
#define I2S_OUT_EOF_BUF_OFF       0x040u
#define I2S_INLINK_DSCR_OFF       0x048u
#define I2S_INLINK_DSCR_BF0_OFF   0x04Cu
#define I2S_INLINK_DSCR_BF1_OFF   0x050u
#define I2S_OUTLINK_DSCR_OFF      0x054u
#define I2S_OUTLINK_DSCR_BF0_OFF  0x058u
#define I2S_OUTLINK_DSCR_BF1_OFF  0x05Cu
#define I2S_LC_CONF_OFF           0x060u
#define I2S_CLKM_CONF_OFF         0x0ACu
#define I2S_SAMPLE_RATE_OFF       0x0B0u
#define I2S_STATE_OFF             0x0BCu
#define I2S_DATE_OFF              0x0FCu

#define I2S_CONF_TX_START         (1u << 4)
#define I2S_CONF_RX_START         (1u << 5)
#define I2S_CONF_TX_MONO          (1u << 14)
#define I2S_CONF_RX_MONO          (1u << 15)
#define I2S_FIFO_DSCR_EN          (1u << 12)
#define I2S_LINK_ADDR_MASK        0x000FFFFFu
#define I2S_LINK_STOP             (1u << 28)
#define I2S_LINK_START            (1u << 29)
#define I2S_LINK_RESTART          (1u << 30)
#define I2S_LINK_PARK             (1u << 31)

#define I2S_INT_IN_DONE           (1u << 8)
#define I2S_INT_IN_SUC_EOF        (1u << 9)
#define I2S_INT_OUT_DONE          (1u << 11)
#define I2S_INT_OUT_EOF           (1u << 12)
#define I2S_INT_IN_DSCR_ERR       (1u << 13)
#define I2S_INT_OUT_DSCR_ERR      (1u << 14)
#define I2S_INT_IN_DSCR_EMPTY     (1u << 15)
#define I2S_INT_OUT_TOTAL_EOF     (1u << 16)
#define I2S_INT_VALID_MASK        0x1FFFFu

#define I2S_DESC_SIZE_MASK        0x00000FFFu
#define I2S_DESC_LENGTH_MASK      0x00FFF000u
#define I2S_DESC_LENGTH_SHIFT     12
#define I2S_DESC_EOF              (1u << 30)
#define I2S_DESC_OWNER            (1u << 31)

/* Classic ESP32 RMT controller: eight channels share 512 32-bit pulse words.
 * Register and RAM apertures occupy one 4 KiB peripheral page. */
#define RMT_CHANNEL_COUNT          8u
#define RMT_MEM_WORDS_PER_CHANNEL  64u
#define RMT_MEM_WORD_COUNT         512u
#define RMT_MEM_OFF                0x800u
#define RMT_CONF0_OFF              0x020u
#define RMT_CONF1_OFF              0x024u
#define RMT_STATUS_OFF             0x060u
#define RMT_ADDR_OFF               0x080u
#define RMT_INT_RAW_OFF            0x0A0u
#define RMT_INT_ST_OFF             0x0A4u
#define RMT_INT_ENA_OFF            0x0A8u
#define RMT_INT_CLR_OFF            0x0ACu
#define RMT_CARRIER_DUTY_OFF       0x0B0u
#define RMT_TX_LIMIT_OFF           0x0D0u
#define RMT_APB_CONF_OFF           0x0F0u
#define RMT_DATE_OFF               0x0FCu

#define RMT_CONF1_TX_START         (1u << 0)
#define RMT_CONF1_RX_EN            (1u << 1)
#define RMT_CONF1_MEM_WR_RST       (1u << 2)
#define RMT_CONF1_MEM_RD_RST       (1u << 3)
#define RMT_CONF1_APB_MEM_RST      (1u << 4)
#define RMT_CONF1_MEM_OWNER_RX     (1u << 5)
#define RMT_CONF1_TX_CONTINUOUS    (1u << 6)
#define RMT_CONF1_REF_APB          (1u << 17)
#define RMT_APB_FIFO_MASK          (1u << 0)
#define RMT_APB_TX_WRAP            (1u << 1)
#define RMT_STATUS_MEM_EMPTY       (1u << 29)
#define RMT_STATUS_MEM_FULL        (1u << 28)
#define RMT_STATUS_MEM_OWNER_ERR   (1u << 27)
#define RMT_STATUS_STATE_TX        (1u << 24)
#define RMT_STATUS_STATE_RX        (3u << 24)
#define RMT_INTR_SOURCE            47

#define RMT_TX_END_INT(ch)         (1u << ((ch) * 3u))
#define RMT_RX_END_INT(ch)         (1u << ((ch) * 3u + 1u))
#define RMT_ERROR_INT(ch)          (1u << ((ch) * 3u + 2u))
#define RMT_TX_THRESHOLD_INT(ch)   (1u << ((ch) + 24u))
#define ESP32_CPU_TICKS_PER_US_ADDR 0x3FFE01E0u

/* UART interrupt sources/register bits used by the ESP-IDF driver. */
#define UART_RXFIFO_FULL_INT     (1u << 0)
#define UART_TXFIFO_EMPTY_INT    (1u << 1)
#define UART_RXFIFO_OVF_INT      (1u << 4)
#define UART_RXFIFO_TOUT_INT     (1u << 8)
#define UART_TX_DONE_INT         (1u << 14)
#define UART_INT_VALID_MASK      0x7FFFFu

/* LEDC register offsets and interrupt source (ESP32, not S2/S3). */
#define LEDC_SPEED_MODE_COUNT    2u
#define LEDC_CHANNEL_COUNT       8u
#define LEDC_TIMER_COUNT         4u
#define LEDC_CHANNEL_STRIDE      0x14u
#define LEDC_LS_CHANNEL_OFF      0x0A0u
#define LEDC_TIMER_OFF           0x140u
#define LEDC_LS_TIMER_OFF        0x160u
#define LEDC_TIMER_STRIDE        0x008u
#define LEDC_INT_RAW_OFF         0x180u
#define LEDC_INT_ST_OFF          0x184u
#define LEDC_INT_ENA_OFF         0x188u
#define LEDC_INT_CLR_OFF         0x18Cu
#define LEDC_CONF_OFF            0x190u
#define LEDC_DATE_OFF            0x1FCu
#define LEDC_INTR_SOURCE         43
#define LEDC_INT_VALID_MASK      0x00FFFFFFu

#define LEDC_CH_TIMER_SEL_MASK   0x00000003u
#define LEDC_CH_SIG_OUT_EN       (1u << 2)
#define LEDC_CH_IDLE_LEVEL       (1u << 3)
#define LEDC_CH_DUTY_MASK        0x01FFFFFFu
#define LEDC_CH_DUTY_START       (1u << 31)
#define LEDC_CH_DUTY_INC         (1u << 30)
#define LEDC_CH_DUTY_NUM_MASK    0x3FFu
#define LEDC_CH_DUTY_NUM_SHIFT   20u
#define LEDC_CH_DUTY_CYCLE_SHIFT 10u
#define LEDC_CH_DUTY_SCALE_MASK  0x3FFu

#define LEDC_TIMER_RES_MASK      0x1Fu
#define LEDC_TIMER_DIV_SHIFT     5u
#define LEDC_TIMER_DIV_MASK      0x3FFFFu
#define LEDC_TIMER_PAUSE         (1u << 23)
#define LEDC_TIMER_RESET         (1u << 24)
#define LEDC_TIMER_TICK_SEL      (1u << 25)
#define LEDC_TIMER_PARA_UP       (1u << 26)
#define LEDC_HS_SIGNAL_BASE      71u
#define LEDC_LS_SIGNAL_BASE      79u

static uint32_t default_read(void *ctx, uint32_t addr);
static void default_write(void *ctx, uint32_t addr, uint32_t val);
static uint32_t i2s_next_fire(esp32_periph_t *p, xtensa_cpu_t *cpu);
static void i2s_eval_events(esp32_periph_t *p, xtensa_cpu_t *cpu);
static uint32_t rmt_next_fire(esp32_periph_t *p, xtensa_cpu_t *cpu);
static void rmt_eval_events(esp32_periph_t *p, xtensa_cpu_t *cpu);
static void rmt_reset_state(esp32_periph_t *p);
static uint32_t ledc_next_fire(esp32_periph_t *p, xtensa_cpu_t *cpu);
static void ledc_eval_events(esp32_periph_t *p, xtensa_cpu_t *cpu);
static void ledc_reset_state(esp32_periph_t *p);
static void ledc_gpio_route_changed(esp32_periph_t *p, int gpio,
                                    uint32_t before, uint32_t after);

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

typedef struct {
    periph_i2c_device_fn fn;
    void *ctx;
} i2c_device_t;

typedef struct {
    uint32_t regs[I2C_REG_FILE_SIZE / sizeof(uint32_t)];
    uint8_t tx_fifo[I2C_FIFO_SIZE];
    uint8_t rx_fifo[I2C_FIFO_SIZE];
    uint8_t tx_head;
    uint8_t tx_tail;
    uint8_t tx_count;
    uint8_t rx_head;
    uint8_t rx_tail;
    uint8_t rx_count;
    uint32_t int_raw;
    uint32_t int_ena;
    bool ack_nack;

    /* Bus transaction state survives END commands: ESP-IDF streams long
     * command links through the 32-byte FIFO over several interrupts. */
    bool active;
    bool expect_address;
    bool have_address;
    bool read_direction;
    bool target_present;
    uint8_t address;
    uint8_t *pending_write;
    size_t pending_write_len;
    size_t pending_write_cap;

    i2c_device_t device[I2C_DEVICE_COUNT];
} i2c_state_t;

typedef struct {
    uint32_t regs[I2S_REG_FILE_SIZE / sizeof(uint32_t)];
    uint32_t int_raw;
    uint32_t int_ena;
    uint32_t tx_desc;
    uint32_t rx_desc;
    bool tx_link_running;
    bool rx_link_running;
    bool tx_active;
    bool rx_active;
    bool tx_event_armed;
    bool rx_event_armed;
    uint32_t next_tx_ccount;
    uint32_t next_rx_ccount;

    uint8_t rx_fifo[I2S_RX_FIFO_SIZE];
    size_t rx_head;
    size_t rx_len;

    periph_i2s_tx_fn tx_cb;
    void *tx_cb_ctx;
} i2s_state_t;

typedef enum {
    RMT_TX_EVENT_NONE = 0,
    RMT_TX_EVENT_THRESHOLD,
    RMT_TX_EVENT_END,
    RMT_TX_EVENT_LOOP,
    RMT_TX_EVENT_ERROR,
} rmt_tx_event_kind_t;

typedef struct {
    uint32_t conf0;
    uint32_t conf1;
    uint32_t status_flags;
    uint16_t apb_index;
    uint16_t tx_index;
    uint16_t rx_index;
    uint16_t tx_since_threshold;

    bool tx_active;
    bool rx_active;
    bool tx_event_armed;
    uint32_t next_tx_ccount;

    rmt_tx_event_kind_t pending_kind;
    uint16_t pending_next_index;
    uint16_t pending_next_threshold;
    size_t pending_count;
    uint32_t pending_items[RMT_MEM_WORD_COUNT];

    periph_rmt_tx_fn tx_cb;
    void *tx_cb_ctx;
} rmt_channel_state_t;

typedef struct {
    rmt_channel_state_t channel[RMT_CHANNEL_COUNT];
    uint32_t memory[RMT_MEM_WORD_COUNT];
    uint32_t int_raw;
    uint32_t int_ena;
    uint32_t carrier_duty[RMT_CHANNEL_COUNT];
    uint32_t tx_limit[RMT_CHANNEL_COUNT];
    uint32_t apb_conf;
    uint32_t date;
} rmt_state_t;

typedef struct {
    uint32_t active_conf;
    uint32_t anchor_count;
    uint64_t anchor_cycles;
    uint64_t reported_wraps;
} ledc_timer_state_t;

typedef struct {
    uint32_t active_duty;       /* hardware Q21.4 duty register value */
    uint32_t update_old_duty;
    uint32_t update_start_duty;
    uint32_t update_target_duty;
    uint32_t update_scale;
    uint32_t update_steps;
    uint64_t update_start_cycle;
    uint64_t update_end_cycle;
    uint64_t update_step_cycles;
    bool update_active;
    bool update_started;

    periph_ledc_output_fn output_cb;
    void *output_cb_ctx;
    int last_gpio;
    uint32_t last_frequency_hz;
    uint32_t last_duty;
    uint32_t last_duty_max;
    bool last_enabled;
    bool last_inverted;
    bool output_reported;
} ledc_channel_state_t;

typedef struct {
    uint32_t regs[0x200 / sizeof(uint32_t)];
    ledc_timer_state_t timer[LEDC_SPEED_MODE_COUNT][LEDC_TIMER_COUNT];
    ledc_channel_state_t channel[LEDC_SPEED_MODE_COUNT][LEDC_CHANNEL_COUNT];
} ledc_state_t;

struct esp32_periph {
    xtensa_mem_t *mem;

    /* Three independent ESP32 UART controllers. */
    uart_state_t uart[UART_COUNT];

    /* Two independent classic ESP32 I2C controllers and their virtual bus
     * targets. */
    i2c_state_t i2c[I2C_PORT_COUNT];

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

    /* DPORT peripheral clock/reset register shadows.  Individual modeled
     * peripherals apply their reset semantics when their bit is asserted. */
    uint32_t dport_perip_clk_en;
    uint32_t dport_perip_rst_en;

    /* Interrupt matrix: maps each CPU interrupt line to a peripheral source.
     * intr_matrix[core][cpu_int] = peripheral source (0-70), 16 = disabled.
     * Mirrors DPORT_PRO_*_MAP_REG / DPORT_APP_*_MAP_REG hardware. */
    uint8_t intr_matrix[2][32];

    /* Pending peripheral interrupt sources (level-triggered) */
    uint32_t pending_sources[3]; /* 71 sources in 3 words (0-31, 32-63, 64-70) */

    /* Compatibility-mode guest ISR dispatch. source_level supplies edge
     * detection independently of interrupt-matrix routing. */
    bool source_level[71];
    periph_irq_dispatch_fn irq_dispatch[71];
    void *irq_dispatch_ctx[71];

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

    /* RTC-domain pin and sensor register files. The two SAR measurement
     * registers additionally expose synchronous single-conversion state. */
    uint32_t rtcio_regs[RTCIO_REG_FILE_SIZE / sizeof(uint32_t)];
    uint32_t sens_regs[SENS_REG_FILE_SIZE / sizeof(uint32_t)];
    uint16_t sens_adc_result[2];
    bool sens_adc_done[2];

    /* SPI flash controllers: [0] = SPI0 (cache), [1] = SPI1 (memspi) */
    spi_state_t spi[2];

    /* Radio/PHY register state used by the closed-source WiFi/BT HAL. */
    radio_state_t radio;

    /* Classic ESP32 high/low-speed LEDC timers, channels, and timed fades. */
    ledc_state_t ledc;
    uint64_t ledc_time_cycles;
    uint32_t ledc_last_ccount;
    bool ledc_time_valid;

    /* Two independent classic ESP32 I2S controllers with circular DMA. */
    i2s_state_t i2s[I2S_PORT_COUNT];

    /* Eight-channel classic ESP32 remote-control/pulse engine. */
    rmt_state_t rmt;

    /* Flash cache MMU table shadows (DPORT_PRO/APP_FLASH_MMU_TABLE).
     * Entries 0-63 are DROM0; 64-127, 128-191, and 192-255 are the
     * IRAM0, IRAM1, and IROM0 cache regions respectively. */
    uint32_t flash_mmu_pro[FLASH_MMU_ENTRY_COUNT];
    uint32_t flash_mmu_app[FLASH_MMU_ENTRY_COUNT];
    uint16_t flash_mmu_effective[FLASH_MMU_ENTRY_COUNT];
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
    for (uint32_t s = 0; s < FLASH_MMU_ENTRY_COUNT; s++) {
        p->flash_mmu_pro[s] = FLASH_MMU_INVALID;
        p->flash_mmu_app[s] = FLASH_MMU_INVALID;
        /* Force the first explicit invalid-table write to remove mem_create's
         * temporary linear loader mappings from the guest page table. */
        p->flash_mmu_effective[s] = UINT16_MAX;
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
#define DPORT_PERIP_CLK_EN_OFF         0x0C0
#define DPORT_PERIP_RST_EN_OFF         0x0C4
#define DPORT_RMT_MODULE_BIT           (1u << 9)
#define DPORT_LEDC_MODULE_BIT          (1u << 11)

/* Internal: scan matrix and set/clear CPU interrupt bits for a source */
static void intr_matrix_update_source(esp32_periph_t *p, int source, bool assert) {
    if (source < 0 || source >= 71)
        return;
    bool rising = assert && !p->source_level[source];
    p->source_level[source] = assert;
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
    if (rising && p->irq_dispatch[source])
        p->irq_dispatch[source](p->irq_dispatch_ctx[source], source);
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

static void flash_mmu_invalidate_code(esp32_periph_t *p, uint32_t addr,
                                      size_t len) {
    xtensa_cpu_t *cpu0 = p->cpu[0];
    xtensa_cpu_t *cpu1 = p->cpu[1];
    if (cpu0) xtensa_invalidate_code(cpu0, addr, len);
    if (!cpu1) return;

    /* Flexe sessions share both predecode and JIT state between cores. Avoid
     * clearing/flushing the same objects twice, while retaining correctness
     * for embedders which attach independent execution engines. */
    if (cpu0 && cpu1->predecode == cpu0->predecode &&
        cpu1->code_invalidate == cpu0->code_invalidate &&
        cpu1->code_invalidate_ctx == cpu0->code_invalidate_ctx)
        return;
    xtensa_invalidate_code(cpu1, addr, len);
}

static int flash_mmu_entry_vaddr(uint32_t entry, uint32_t *vbase,
                                 bool *instruction) {
    if (entry < 64u) {
        *vbase = 0x3F400000u + entry * FLASH_MMU_PAGE_SIZE;
        *instruction = false;
        return 1;
    }
    if (entry < 77u || entry >= FLASH_MMU_ENTRY_COUNT)
        return 0;  /* entries below IRAM0's 0x400D0000 low address are unused */
    *vbase = 0x40000000u + (entry - 64u) * FLASH_MMU_PAGE_SIZE;
    *instruction = true;
    return 1;
}

/* Apply one effective table entry to Flexe's shared guest page table. Real
 * hardware has one cache/MMU per core; ESP-IDF mirrors mappings between cores,
 * which lets the emulator use the most recently written valid mapping. */
static void flash_mmu_apply_entry(esp32_periph_t *p, uint32_t entry,
                                  uint32_t val) {
    uint32_t vbase;
    bool instruction;
    if (!flash_mmu_entry_vaddr(entry, &vbase, &instruction)) return;

    uint32_t physical = (val & 0xFFu) * FLASH_MMU_PAGE_SIZE;
    bool mapped = (val & FLASH_MMU_INVALID) == 0 &&
                  physical <= EMU_FLASH_SIZE - FLASH_MMU_PAGE_SIZE;
    uint8_t *backing = instruction ? p->mem->flash_insn : p->mem->flash_data;
    for (uint32_t off = 0; off < FLASH_MMU_PAGE_SIZE; off += PAGE_SIZE) {
        p->mem->page_table[(vbase + off) >> 12] =
            mapped ? backing + physical + off : NULL;
    }

    if (getenv("FLEXE_DBG_FLASH"))
        fprintf(stderr,
                "[MMUTBL] entry=%u vaddr=0x%08X val=0x%03X paddr=0x%06X %s\n",
                entry, vbase, val, physical, mapped ? "mapped" : "invalid");
    if (instruction)
        flash_mmu_invalidate_code(p, vbase, FLASH_MMU_PAGE_SIZE);
}

static void flash_mmu_write_entry(esp32_periph_t *p, int core,
                                  uint32_t entry, uint32_t val) {
    if (entry >= FLASH_MMU_ENTRY_COUNT || core < 0 || core > 1) return;
    val &= FLASH_MMU_VALUE_MASK;
    uint32_t *own = core == 0 ? p->flash_mmu_pro : p->flash_mmu_app;
    uint32_t *other = core == 0 ? p->flash_mmu_app : p->flash_mmu_pro;
    own[entry] = val;

    /* An invalidation on one core must not remove a mapping still active on
     * the other core. Once both are invalid, remove all 4 KiB host pages. */
    uint32_t effective = val;
    if ((effective & FLASH_MMU_INVALID) &&
        !(other[entry] & FLASH_MMU_INVALID))
        effective = other[entry];
    if (p->flash_mmu_effective[entry] == effective) return;
    p->flash_mmu_effective[entry] = (uint16_t)effective;
    flash_mmu_apply_entry(p, entry, effective);
}

/* Invalidate translated code for every instruction mapping which aliases a
 * programmed/erased physical flash range. flash_insn itself is synchronized
 * by the SPI controller before this helper is called. */
static void flash_mmu_invalidate_physical(esp32_periph_t *p, uint32_t offset,
                                          uint32_t len) {
    if (len == 0) return;
    uint32_t first = offset / FLASH_MMU_PAGE_SIZE;
    uint32_t last = (offset + len - 1u) / FLASH_MMU_PAGE_SIZE;
    for (uint32_t entry = 77u; entry < FLASH_MMU_ENTRY_COUNT; entry++) {
        uint32_t val = p->flash_mmu_effective[entry];
        if ((val & FLASH_MMU_INVALID) || (val & 0xFFu) < first ||
            (val & 0xFFu) > last)
            continue;
        uint32_t vbase = 0x40000000u +
                         (entry - 64u) * FLASH_MMU_PAGE_SIZE;
        flash_mmu_invalidate_code(p, vbase, FLASH_MMU_PAGE_SIZE);
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
    case DPORT_PERIP_CLK_EN_OFF: return p->dport_perip_clk_en;
    case DPORT_PERIP_RST_EN_OFF: return p->dport_perip_rst_en;
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
    /* Flash MMU tables: PRO 0x3FF10000-0x3FF103FF, APP 0x3FF12000-0x3FF123FF. */
    if (off >= 0x10000 && off < 0x10400) {
        uint32_t entry = (off - 0x10000) >> 2;
        flash_mmu_write_entry(p, 0, entry, val);
        return;
    }
    if (off >= 0x12000 && off < 0x12400) {
        uint32_t entry = (off - 0x12000) >> 2;
        flash_mmu_write_entry(p, 1, entry, val);
        return;
    }
    switch (off) {
    case DPORT_PERIP_CLK_EN_OFF:
        p->dport_perip_clk_en = val;
        break;
    case DPORT_PERIP_RST_EN_OFF:
        p->dport_perip_rst_en = val;
        if (val & DPORT_RMT_MODULE_BIT)
            rmt_reset_state(p);
        if (val & DPORT_LEDC_MODULE_BIT)
            ledc_reset_state(p);
        break;
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
        uint32_t before = p->gpio.func_out_sel[n];
        p->gpio.func_out_sel[n] = val;
        ledc_gpio_route_changed(p, n, before, val);
        return;
    }

    /* Fire sandbox events for any output pins that changed level. */
    if (p->gpio.out != prev_out)
        gpio_emit_changed(prev_out, p->gpio.out, 0);
    if (p->gpio.out1 != prev_out1)
        gpio_emit_changed(prev_out1, p->gpio.out1, 32);
}

/* ---- RTC_CNTL ---- */

static bool rtcio_dac_is_enabled(uint32_t reg) {
    return (reg & (RTCIO_DAC_XPD | RTCIO_DAC_XPD_FORCE)) ==
           (RTCIO_DAC_XPD | RTCIO_DAC_XPD_FORCE);
}

static uint8_t rtcio_dac_value(uint32_t reg) {
    return (uint8_t)((reg & RTCIO_DAC_VALUE_MASK) >> 19);
}

static void rtcio_emit_dac_change(int channel, uint32_t before,
                                  uint32_t after) {
    bool old_enabled = rtcio_dac_is_enabled(before);
    bool new_enabled = rtcio_dac_is_enabled(after);
    uint8_t old_value = rtcio_dac_value(before);
    uint8_t new_value = rtcio_dac_value(after);
    if (old_enabled == new_enabled && old_value == new_value)
        return;

    sbx_event_t ev = { .kind = SBX_EV_DAC_OUT, .cycle = 0 };
    ev.dac_out.channel = (uint8_t)channel;
    ev.dac_out.enabled = new_enabled ? 1u : 0u;
    ev.dac_out.value = new_value;
    sbx_events_emit(&ev);
}

static uint32_t rtcio_read(esp32_periph_t *p, uint32_t off) {
    if ((off & 3u) != 0 || off >= RTCIO_REG_FILE_SIZE)
        return 0;
    /* W1TS/W1TC registers are write-only. */
    switch (off) {
    case 0x004u:
    case 0x008u:
    case 0x010u:
    case 0x014u:
    case 0x01Cu:
    case 0x020u:
        return 0;
    default:
        return p->rtcio_regs[off / 4u];
    }
}

static void rtcio_write(esp32_periph_t *p, uint32_t off, uint32_t val) {
    if ((off & 3u) != 0 || off >= RTCIO_REG_FILE_SIZE)
        return;

    switch (off) {
    case 0x004u: p->rtcio_regs[0x000u / 4u] |= val; return;
    case 0x008u: p->rtcio_regs[0x000u / 4u] &= ~val; return;
    case 0x010u: p->rtcio_regs[0x00Cu / 4u] |= val; return;
    case 0x014u: p->rtcio_regs[0x00Cu / 4u] &= ~val; return;
    case 0x01Cu: p->rtcio_regs[0x018u / 4u] |= val; return;
    case 0x020u: p->rtcio_regs[0x018u / 4u] &= ~val; return;
    case 0x024u: return; /* RTC_GPIO_IN is read-only. */
    default: break;
    }

    uint32_t before = p->rtcio_regs[off / 4u];
    p->rtcio_regs[off / 4u] = val;
    if (off == RTCIO_DAC1_OFF)
        rtcio_emit_dac_change(0, before, val);
    else if (off == RTCIO_DAC2_OFF)
        rtcio_emit_dac_change(1, before, val);
}

static int sens_measure_unit(uint32_t off) {
    if (off == SENS_SAR_MEAS_START1_OFF) return 0;
    if (off == SENS_SAR_MEAS_START2_OFF) return 1;
    return -1;
}

static uint32_t sens_read(esp32_periph_t *p, uint32_t off) {
    if ((off & 3u) != 0 || off >= SENS_REG_FILE_SIZE)
        return 0;
    int unit = sens_measure_unit(off);
    uint32_t val = p->sens_regs[off / 4u];
    if (unit >= 0) {
        val &= SENS_SAR_CONFIG_MASK;
        if (p->sens_adc_done[unit])
            val |= SENS_SAR_DONE;
        val |= p->sens_adc_result[unit];
    }
    return val;
}

static void sens_write(esp32_periph_t *p, uint32_t off, uint32_t val) {
    if ((off & 3u) != 0 || off >= SENS_REG_FILE_SIZE)
        return;
    int unit = sens_measure_unit(off);
    if (unit < 0) {
        p->sens_regs[off / 4u] = val;
        return;
    }

    /* DONE and DATA are read-only. A low START phase clears DONE; the rising
     * phase completes immediately and latches the host-injected channel. */
    p->sens_regs[off / 4u] = val & SENS_SAR_CONFIG_MASK;
    if ((val & SENS_SAR_START) == 0) {
        p->sens_adc_done[unit] = false;
        return;
    }

    uint32_t enabled = (val & SENS_SAR_EN_PAD_MASK) >> 19;
    unsigned channel = 0;
    while (channel < 12u && (enabled & (1u << channel)) == 0)
        channel++;

    uint16_t raw = 0;
    if (channel < 10u)
        raw = p->adc_value[unit * 10 + channel];
    unsigned width_code =
        (p->sens_regs[SENS_SAR_START_FORCE_OFF / 4u] >> (unit * 2)) & 3u;
    unsigned bits = 9u + width_code;
    raw &= (uint16_t)((1u << bits) - 1u);
    p->sens_adc_result[unit] = raw;
    p->sens_adc_done[unit] = true;
}

static uint32_t rtc_cntl_read(void *ctx, uint32_t addr) {
    esp32_periph_t *p = ctx;
    uint32_t off = addr - RTC_CNTL_BASE;
    if (addr >= SENS_BASE && addr < SENS_BASE + SENS_REG_FILE_SIZE)
        return sens_read(p, addr - SENS_BASE);
    if (addr >= RTCIO_BASE && addr < RTCIO_BASE + RTCIO_REG_FILE_SIZE)
        return rtcio_read(p, addr - RTCIO_BASE);
    /* Reserved RTCIO/SENS portions of the shared page read as zero. */
    if (off >= 0x400u) return 0;
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
    esp32_periph_t *p = ctx;
    if (addr >= SENS_BASE && addr < SENS_BASE + SENS_REG_FILE_SIZE) {
        sens_write(p, addr - SENS_BASE, val);
        return;
    }
    if (addr >= RTCIO_BASE && addr < RTCIO_BASE + RTCIO_REG_FILE_SIZE) {
        rtcio_write(p, addr - RTCIO_BASE, val);
        return;
    }
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
    esp32_periph_t *p = (esp32_periph_t *)cpu->periph_event_ctx;
    uint32_t events[] = {
        lact_next_fire(p, cpu),
        i2s_next_fire(p, cpu),
        rmt_next_fire(p, cpu),
        ledc_next_fire(p, cpu),
    };
    bool have = false;
    uint32_t best = UINT32_MAX;
    uint32_t best_distance = 0;
    for (size_t i = 0; i < sizeof(events) / sizeof(events[0]); i++) {
        if (events[i] == UINT32_MAX) continue;
        uint32_t distance = events[i] - cpu->ccount;
        if ((int32_t)distance < 0) distance = 0;
        if (!have || distance < best_distance) {
            have = true;
            best = events[i];
            best_distance = distance;
        }
    }
    return have ? best : UINT32_MAX;
}
static void periph_event_hook(xtensa_cpu_t *cpu) {
    esp32_periph_t *p = (esp32_periph_t *)cpu->periph_event_ctx;
    for (int group = 0; group < 2; group++)
        lact_eval_irq(p, group);
    i2s_eval_events(p, cpu);
    rmt_eval_events(p, cpu);
    ledc_eval_events(p, cpu);
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
    for (int i = 0; i < bytes; i++) {
        p->mem->flash_data[off + i] &= src[i];
        p->mem->flash_insn[off + i] &= src[i];
    }
    flash_mmu_invalidate_physical(p, off, (uint32_t)bytes);
}

static void spi_flash_erase(esp32_periph_t *p, uint32_t off, uint32_t len) {
    if (off >= EMU_FLASH_SIZE || !p->mem->flash_data) return;
    if (len > EMU_FLASH_SIZE - off) len = EMU_FLASH_SIZE - off;
    if (spi_debug_offset(off))
        fprintf(stderr, "[SPIERASE] off=0x%X bytes=%u\n", off, len);
    memset(p->mem->flash_data + off, 0xFF, len);
    memset(p->mem->flash_insn + off, 0xFF, len);
    flash_mmu_invalidate_physical(p, off, len);
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

/* ---- I2C0/I2C1 master controllers ---- */

static const uint32_t i2c_bases[I2C_PORT_COUNT] = {
    I2C0_BASE, I2C1_BASE
};

static const int i2c_intr_sources[I2C_PORT_COUNT] = {49, 50};

static int i2c_port_from_addr(uint32_t addr) {
    for (int port = 0; port < I2C_PORT_COUNT; port++) {
        if (addr >= i2c_bases[port] &&
            addr < i2c_bases[port] + PAGE_SIZE)
            return port;
    }
    return -1;
}

static void i2c_intr_update(esp32_periph_t *p, int port) {
    i2c_state_t *i2c = &p->i2c[port];
    int source = i2c_intr_sources[port];
    uint32_t mask = 1u << (source % 32);
    bool active = (i2c->int_raw & i2c->int_ena) != 0;
    if (active)
        p->pending_sources[source / 32] |= mask;
    else
        p->pending_sources[source / 32] &= ~mask;
    intr_matrix_update_source(p, source, active);
}

static void i2c_tx_reset(i2c_state_t *i2c) {
    i2c->tx_head = 0;
    i2c->tx_tail = 0;
    i2c->tx_count = 0;
}

static void i2c_rx_reset(i2c_state_t *i2c) {
    i2c->rx_head = 0;
    i2c->rx_tail = 0;
    i2c->rx_count = 0;
}

static void i2c_tx_push(i2c_state_t *i2c, uint8_t byte) {
    if (i2c->tx_count >= I2C_FIFO_SIZE)
        return;
    i2c->tx_fifo[i2c->tx_head] = byte;
    i2c->tx_head = (uint8_t)((i2c->tx_head + 1u) % I2C_FIFO_SIZE);
    i2c->tx_count++;
}

static uint8_t i2c_tx_pop(i2c_state_t *i2c) {
    if (i2c->tx_count == 0)
        return 0xFFu;
    uint8_t byte = i2c->tx_fifo[i2c->tx_tail];
    i2c->tx_tail = (uint8_t)((i2c->tx_tail + 1u) % I2C_FIFO_SIZE);
    i2c->tx_count--;
    return byte;
}

static bool i2c_rx_push(i2c_state_t *i2c, uint8_t byte) {
    if (i2c->rx_count >= I2C_FIFO_SIZE)
        return false;
    i2c->rx_fifo[i2c->rx_head] = byte;
    i2c->rx_head = (uint8_t)((i2c->rx_head + 1u) % I2C_FIFO_SIZE);
    i2c->rx_count++;
    return true;
}

static uint8_t i2c_rx_pop(i2c_state_t *i2c) {
    if (i2c->rx_count == 0)
        return 0;
    uint8_t byte = i2c->rx_fifo[i2c->rx_tail];
    i2c->rx_tail = (uint8_t)((i2c->rx_tail + 1u) % I2C_FIFO_SIZE);
    i2c->rx_count--;
    return byte;
}

static void i2c_emit_transfer(int port, uint8_t address, bool read,
                              const uint8_t *data, size_t len) {
    size_t offset = 0;
    do {
        size_t chunk = len - offset;
        if (chunk > UINT16_MAX)
            chunk = UINT16_MAX;
        sbx_event_t ev = { .kind = SBX_EV_I2C_XFER, .cycle = 0 };
        ev.i2c_xfer.port = (uint8_t)port;
        ev.i2c_xfer.addr = address;
        ev.i2c_xfer.read = read ? 1u : 0u;
        ev.i2c_xfer.len = (uint16_t)chunk;
        ev.i2c_xfer.data = data ? data + offset : NULL;
        sbx_events_emit(&ev);
        offset += chunk;
    } while (offset < len);
}

static bool i2c_pending_append(i2c_state_t *i2c, uint8_t byte) {
    if (i2c->pending_write_len >= I2C_MAX_PENDING_WRITE)
        return false;
    if (i2c->pending_write_len == i2c->pending_write_cap) {
        size_t cap = i2c->pending_write_cap ? i2c->pending_write_cap * 2u : 64u;
        if (cap > I2C_MAX_PENDING_WRITE)
            cap = I2C_MAX_PENDING_WRITE;
        uint8_t *new_data = realloc(i2c->pending_write, cap);
        if (!new_data)
            return false;
        i2c->pending_write = new_data;
        i2c->pending_write_cap = cap;
    }
    i2c->pending_write[i2c->pending_write_len++] = byte;
    return true;
}

static void i2c_end_transaction(i2c_state_t *i2c) {
    i2c->active = false;
    i2c->expect_address = false;
    i2c->have_address = false;
    i2c->read_direction = false;
    i2c->target_present = false;
    i2c->pending_write_len = 0;
}

static int i2c_commit_write(esp32_periph_t *p, int port) {
    i2c_state_t *i2c = &p->i2c[port];
    if (!i2c->have_address || i2c->read_direction)
        return 0;
    i2c_device_t *device = &i2c->device[i2c->address];
    if (!i2c->target_present || !device->fn)
        return -1;
    int result = device->fn(device->ctx, port, i2c->address,
                            i2c->pending_write, i2c->pending_write_len,
                            NULL, 0);
    i2c_emit_transfer(port, i2c->address, false, i2c->pending_write,
                      i2c->pending_write_len);
    i2c->pending_write_len = 0;
    return result;
}

static int i2c_fill_read(esp32_periph_t *p, int port, size_t wanted) {
    i2c_state_t *i2c = &p->i2c[port];
    size_t available = I2C_FIFO_SIZE - i2c->rx_count;
    size_t count = wanted < available ? wanted : available;
    uint8_t data[I2C_FIFO_SIZE];
    memset(data, 0xFF, sizeof(data));

    int result = -1;
    if (i2c->have_address && i2c->read_direction &&
        i2c->target_present) {
        i2c_device_t *device = &i2c->device[i2c->address];
        if (device->fn)
            result = device->fn(device->ctx, port, i2c->address,
                                i2c->pending_write,
                                i2c->pending_write_len, data, count);
    }
    if (i2c->pending_write_len != 0)
        i2c_emit_transfer(port, i2c->address, false,
                          i2c->pending_write, i2c->pending_write_len);
    i2c->pending_write_len = 0;

    for (size_t index = 0; index < count; index++)
        (void)i2c_rx_push(i2c, data[index]);
    if (wanted > count)
        i2c->int_raw |= I2C_INT_RXFIFO_OVF;
    i2c_emit_transfer(port, i2c->address, true, data, count);
    return result;
}

static bool i2c_select_address(esp32_periph_t *p, int port,
                               uint8_t address_byte) {
    i2c_state_t *i2c = &p->i2c[port];
    uint8_t address = address_byte >> 1;
    bool read = (address_byte & 1u) != 0;
    bool combined_read = i2c->have_address && !i2c->read_direction && read &&
                         i2c->address == address;

    if (i2c->have_address && !i2c->read_direction && !combined_read) {
        if (i2c_commit_write(p, port) != 0)
            i2c->int_raw |= I2C_INT_ACK_ERR;
    }
    if (!combined_read)
        i2c->pending_write_len = 0;

    i2c->address = address;
    i2c->read_direction = read;
    i2c->have_address = true;
    i2c->expect_address = false;
    i2c->target_present = i2c->device[address].fn != NULL;
    i2c->ack_nack = !i2c->target_present;
    return i2c->target_present;
}

static bool i2c_execute_write(esp32_periph_t *p, int port, uint32_t cmd) {
    i2c_state_t *i2c = &p->i2c[port];
    unsigned count = cmd & 0xFFu;
    bool nack = false;

    for (unsigned index = 0; index < count; index++) {
        uint8_t byte = i2c_tx_pop(i2c);
        if (i2c->expect_address || !i2c->have_address) {
            if (!i2c_select_address(p, port, byte))
                nack = true;
            continue;
        }
        if (i2c->read_direction || !i2c->target_present) {
            nack = true;
            continue;
        }
        if (!i2c_pending_append(i2c, byte))
            nack = true;
    }

    i2c->ack_nack = nack;
    if ((cmd & (1u << 8)) != 0) {
        bool expected_nack = (cmd & (1u << 9)) != 0;
        if (nack != expected_nack)
            i2c->int_raw |= I2C_INT_ACK_ERR;
    }
    return !nack;
}

static void i2c_execute(esp32_periph_t *p, int port) {
    i2c_state_t *i2c = &p->i2c[port];
    i2c->int_raw |= I2C_INT_TRANS_START;

    for (unsigned index = 0; index < I2C_COMMAND_COUNT; index++) {
        uint32_t *reg = &i2c->regs[(I2C_COMMAND0_OFF / 4u) + index];
        uint32_t cmd = *reg & 0x3FFFu;
        unsigned opcode = (cmd >> 11) & 7u;
        bool stop = false;

        switch (opcode) {
        case I2C_CMD_RESTART:
            i2c->active = true;
            i2c->expect_address = true;
            break;
        case I2C_CMD_WRITE:
            (void)i2c_execute_write(p, port, cmd);
            break;
        case I2C_CMD_READ:
            if (i2c_fill_read(p, port, cmd & 0xFFu) != 0)
                i2c->int_raw |= I2C_INT_ACK_ERR;
            break;
        case I2C_CMD_STOP:
            if (i2c_commit_write(p, port) != 0)
                i2c->int_raw |= I2C_INT_ACK_ERR;
            i2c_end_transaction(i2c);
            i2c->int_raw |= I2C_INT_TRANS_DONE;
            stop = true;
            break;
        case I2C_CMD_END:
            i2c->int_raw |= I2C_INT_END_DETECT;
            stop = true;
            break;
        default:
            i2c->int_raw |= I2C_INT_ACK_ERR;
            stop = true;
            break;
        }

        *reg = cmd | I2C_CMD_DONE;
        i2c->int_raw |= I2C_INT_MASTER_DONE;
        if ((i2c->int_raw & I2C_INT_ACK_ERR) != 0 &&
            opcode == I2C_CMD_WRITE) {
            i2c_end_transaction(i2c);
            stop = true;
        }
        if (stop)
            break;
    }

    if (i2c->tx_count == 0)
        i2c->int_raw |= I2C_INT_TXFIFO_EMPTY;
    uint32_t rx_threshold = i2c->regs[I2C_FIFO_CONF_OFF / 4u] & 0x1Fu;
    if (i2c->rx_count > rx_threshold)
        i2c->int_raw |= I2C_INT_RXFIFO_FULL;
    i2c->regs[I2C_CTR_OFF / 4u] &= ~I2C_CTR_TRANS_START;
    i2c_intr_update(p, port);
}

static uint32_t i2c_read(void *ctx, uint32_t addr) {
    esp32_periph_t *p = ctx;
    int port = i2c_port_from_addr(addr);
    if (port < 0)
        return default_read(ctx, addr);
    i2c_state_t *i2c = &p->i2c[port];
    uint32_t off = addr - i2c_bases[port];
    if ((off & 3u) != 0 || off >= I2C_REG_FILE_SIZE)
        return default_read(ctx, addr);

    switch (off) {
    case I2C_SR_OFF:
        return ((uint32_t)i2c->tx_count << 18) |
               ((uint32_t)i2c->rx_count << 8) |
               (i2c->active ? 1u << 4 : 0) |
               (i2c->ack_nack ? 1u : 0);
    case I2C_RXFIFO_ST_OFF:
        return ((uint32_t)(i2c->tx_head & 0x1Fu) << 15) |
               ((uint32_t)(i2c->tx_tail & 0x1Fu) << 10) |
               ((uint32_t)(i2c->rx_head & 0x1Fu) << 5) |
               (uint32_t)(i2c->rx_tail & 0x1Fu);
    case I2C_DATA_OFF:
        return i2c_rx_pop(i2c);
    case I2C_INT_RAW_OFF:
        return i2c->int_raw;
    case I2C_INT_CLR_OFF:
        return 0;
    case I2C_INT_ENA_OFF:
        return i2c->int_ena;
    case I2C_INT_ST_OFF:
        return i2c->int_raw & i2c->int_ena;
    default:
        return i2c->regs[off / 4u];
    }
}

static void i2c_write(void *ctx, uint32_t addr, uint32_t val) {
    esp32_periph_t *p = ctx;
    int port = i2c_port_from_addr(addr);
    if (port < 0) {
        default_write(ctx, addr, val);
        return;
    }
    i2c_state_t *i2c = &p->i2c[port];
    uint32_t off = addr - i2c_bases[port];
    if ((off & 3u) != 0 || off >= I2C_REG_FILE_SIZE) {
        default_write(ctx, addr, val);
        return;
    }

    switch (off) {
    case I2C_CTR_OFF:
        i2c->regs[off / 4u] = val;
        if (val & I2C_CTR_TRANS_START)
            i2c_execute(p, port);
        break;
    case I2C_FIFO_CONF_OFF:
        i2c->regs[off / 4u] = val;
        if (val & I2C_FIFO_TX_RST)
            i2c_tx_reset(i2c);
        if (val & I2C_FIFO_RX_RST)
            i2c_rx_reset(i2c);
        break;
    case I2C_DATA_OFF:
        i2c_tx_push(i2c, (uint8_t)val);
        break;
    case I2C_INT_RAW_OFF:
    case I2C_INT_ST_OFF:
        break; /* read-only */
    case I2C_INT_CLR_OFF:
        i2c->int_raw &= ~(val & I2C_INT_VALID_MASK);
        i2c_intr_update(p, port);
        break;
    case I2C_INT_ENA_OFF:
        i2c->int_ena = val & I2C_INT_VALID_MASK;
        i2c_intr_update(p, port);
        break;
    default:
        i2c->regs[off / 4u] = val;
        break;
    }
}

/* ---- RMT remote-control / pulse engine ---- */

static size_t rmt_channel_capacity(const rmt_state_t *rmt, unsigned channel) {
    if (channel >= RMT_CHANNEL_COUNT) return 0;
    uint32_t blocks = (rmt->channel[channel].conf0 >> 24) & 0xFu;
    uint32_t available = RMT_CHANNEL_COUNT - channel;
    if (blocks > available) blocks = available;
    return (size_t)blocks * RMT_MEM_WORDS_PER_CHANNEL;
}

static uint32_t rmt_channel_tick_hz(const rmt_channel_state_t *channel) {
    uint32_t divider = channel->conf0 & 0xFFu;
    if (divider == 0) divider = 256u;
    uint32_t source_hz =
        channel->conf1 & RMT_CONF1_REF_APB ? 80000000u : 1000000u;
    uint32_t tick_hz = source_hz / divider;
    return tick_hz ? tick_hz : 1u;
}

static uint32_t rmt_channel_carrier_hz(const rmt_state_t *rmt,
                                       unsigned channel) {
    const rmt_channel_state_t *state = &rmt->channel[channel];
    if (!(state->conf0 & (1u << 28))) return 0;
    uint32_t duty = rmt->carrier_duty[channel];
    uint32_t low = duty & 0xFFFFu;
    uint32_t high = duty >> 16;
    if (low == 0) low = 65536u;
    if (high == 0) high = 65536u;
    return 80000000u / (low + high);
}

static uint32_t rmt_item_cycles(const esp32_periph_t *p,
                                const rmt_channel_state_t *channel,
                                uint32_t item) {
    uint32_t duration0 = item & 0x7FFFu;
    uint32_t duration1 = (item >> 16) & 0x7FFFu;
    uint32_t divider = channel->conf0 & 0xFFu;
    if (divider == 0) divider = 256u;
    uint32_t cpu_mhz = mem_read32(p->mem, ESP32_CPU_TICKS_PER_US_ADDR);
    if (cpu_mhz < 10u || cpu_mhz > 240u) cpu_mhz = 240u;
    uint32_t source_mhz =
        channel->conf1 & RMT_CONF1_REF_APB ? 80u : 1u;
    uint64_t numerator = (uint64_t)(duration0 + duration1) * divider *
                         cpu_mhz;
    uint64_t cycles = (numerator + source_mhz - 1u) / source_mhz;
    return cycles > UINT32_MAX ? UINT32_MAX : (uint32_t)cycles;
}

static void rmt_irq_update(esp32_periph_t *p) {
    if (p->rmt.int_raw & p->rmt.int_ena)
        periph_assert_interrupt(p, RMT_INTR_SOURCE);
    else
        periph_deassert_interrupt(p, RMT_INTR_SOURCE);
}

static void rmt_kick(esp32_periph_t *p) {
    for (int core = 0; core < 2; core++)
        if (p->cpu[core]) xtensa_recompute_next_timer(p->cpu[core]);
}

static void rmt_tx_cancel(rmt_channel_state_t *channel) {
    channel->tx_active = false;
    channel->tx_event_armed = false;
    channel->pending_kind = RMT_TX_EVENT_NONE;
    channel->pending_count = 0;
    channel->conf1 &= ~RMT_CONF1_TX_START;
}

/* Snapshot the next immutable portion of the active memory ring. Threshold
 * delivery happens before the following portion is sampled, so the genuine
 * driver can safely refill the half which hardware has just consumed. */
static void rmt_plan_tx_segment_at(esp32_periph_t *p, unsigned channel_index,
                                   uint32_t start_ccount) {
    rmt_state_t *rmt = &p->rmt;
    rmt_channel_state_t *channel = &rmt->channel[channel_index];
    if (!channel->tx_active || channel->tx_event_armed) return;

    size_t capacity = rmt_channel_capacity(rmt, channel_index);
    if (capacity == 0) {
        channel->pending_kind = RMT_TX_EVENT_ERROR;
        channel->pending_count = 0;
        channel->pending_next_index = 0;
        channel->pending_next_threshold = 0;
        channel->next_tx_ccount = start_ccount + 1u;
        channel->tx_event_armed = true;
        return;
    }

    size_t base = channel_index * RMT_MEM_WORDS_PER_CHANNEL;
    size_t index = channel->tx_index % capacity;
    uint32_t since_threshold = channel->tx_since_threshold;
    uint32_t threshold = rmt->tx_limit[channel_index] & 0x1FFu;
    bool wrap = (rmt->apb_conf & RMT_APB_TX_WRAP) != 0;
    bool continuous = (channel->conf1 & RMT_CONF1_TX_CONTINUOUS) != 0;
    size_t scan_limit = threshold ?
        (since_threshold < threshold ? threshold - since_threshold : 1u) :
        capacity;
    if (scan_limit == 0 || scan_limit > RMT_MEM_WORD_COUNT)
        scan_limit = RMT_MEM_WORD_COUNT;

    channel->pending_count = 0;
    channel->pending_kind = RMT_TX_EVENT_NONE;
    uint64_t cycles = 0;

    for (size_t scanned = 0; scanned < scan_limit; scanned++) {
        uint32_t item = rmt->memory[(base + index) % RMT_MEM_WORD_COUNT];
        uint32_t duration0 = item & 0x7FFFu;
        uint32_t duration1 = (item >> 16) & 0x7FFFu;

        if (duration0 == 0) {
            channel->pending_kind = continuous ? RMT_TX_EVENT_LOOP :
                                                  RMT_TX_EVENT_END;
            break;
        }

        channel->pending_items[channel->pending_count++] = item;
        cycles += rmt_item_cycles(p, channel, item);
        index++;
        since_threshold++;

        if (index >= capacity) {
            if (wrap || continuous)
                index = 0;
            else if (duration1 != 0) {
                channel->pending_kind = RMT_TX_EVENT_ERROR;
                break;
            }
        }

        if (duration1 == 0) {
            channel->pending_kind = continuous ? RMT_TX_EVENT_LOOP :
                                                  RMT_TX_EVENT_END;
            break;
        }
        if (threshold && since_threshold >= threshold) {
            since_threshold = 0;
            channel->pending_kind = RMT_TX_EVENT_THRESHOLD;
            break;
        }
    }

    if (channel->pending_kind == RMT_TX_EVENT_NONE)
        channel->pending_kind = RMT_TX_EVENT_ERROR;
    if (channel->pending_kind == RMT_TX_EVENT_LOOP)
        index = 0;

    channel->pending_next_index = (uint16_t)index;
    channel->pending_next_threshold = (uint16_t)since_threshold;
    if (cycles == 0) cycles = 1;
    if (cycles > INT32_MAX) cycles = INT32_MAX;
    channel->next_tx_ccount = start_ccount + (uint32_t)cycles;
    channel->tx_event_armed = true;
}

static void rmt_plan_tx_segment(esp32_periph_t *p, unsigned channel_index) {
    uint32_t now = p->cpu[0] ? p->cpu[0]->ccount : 0;
    rmt_plan_tx_segment_at(p, channel_index, now);
}

static void rmt_reset_state(esp32_periph_t *p) {
    periph_rmt_tx_fn callbacks[RMT_CHANNEL_COUNT];
    void *callback_contexts[RMT_CHANNEL_COUNT];
    for (unsigned channel = 0; channel < RMT_CHANNEL_COUNT; channel++) {
        callbacks[channel] = p->rmt.channel[channel].tx_cb;
        callback_contexts[channel] = p->rmt.channel[channel].tx_cb_ctx;
    }

    memset(&p->rmt, 0, sizeof(p->rmt));
    for (unsigned channel = 0; channel < RMT_CHANNEL_COUNT; channel++) {
        /* Production ESP-IDF configures TX without clearing MEM_OWNER, so
         * transmitter ownership is the observable silicon reset state. */
        p->rmt.channel[channel].conf0 = 0x31100002u;
        p->rmt.channel[channel].conf1 = 0x00000F00u;
        p->rmt.channel[channel].tx_cb = callbacks[channel];
        p->rmt.channel[channel].tx_cb_ctx = callback_contexts[channel];
    }
    p->rmt.date = 0x16022600u;
    rmt_irq_update(p);
    rmt_kick(p);
}

static uint32_t rmt_status_word(const esp32_periph_t *p, unsigned index) {
    const rmt_channel_state_t *channel = &p->rmt.channel[index];
    uint32_t base = index * RMT_MEM_WORDS_PER_CHANNEL;
    uint32_t status = channel->status_flags;
    if (channel->tx_active)
        status |= RMT_STATUS_STATE_TX;
    else if (channel->rx_active)
        status |= RMT_STATUS_STATE_RX;
    status |= ((base + channel->tx_index) & 0x3FFu) << 12;
    status |= (base + channel->rx_index) & 0x3FFu;
    return status;
}

static uint32_t rmt_next_fire(esp32_periph_t *p, xtensa_cpu_t *cpu) {
    if (!p || !cpu || cpu != p->cpu[0]) return UINT32_MAX;
    bool have = false;
    uint32_t best = UINT32_MAX;
    uint32_t best_distance = 0;
    for (unsigned index = 0; index < RMT_CHANNEL_COUNT; index++) {
        rmt_channel_state_t *channel = &p->rmt.channel[index];
        if (!channel->tx_event_armed) continue;
        uint32_t distance = channel->next_tx_ccount - cpu->ccount;
        if ((int32_t)distance < 0) distance = 0;
        if (!have || distance < best_distance) {
            have = true;
            best = channel->next_tx_ccount;
            best_distance = distance;
        }
    }
    return have ? best : UINT32_MAX;
}

static void rmt_eval_events(esp32_periph_t *p, xtensa_cpu_t *cpu) {
    if (!p || !cpu || cpu != p->cpu[0]) return;
    for (unsigned index = 0; index < RMT_CHANNEL_COUNT; index++) {
        rmt_channel_state_t *channel = &p->rmt.channel[index];
        /* A wait stub may advance CCOUNT across several RMT deadlines in one
         * dispatch. Drain finite transfers completely, while bounding a
         * continuously looping channel so it cannot monopolize the CPU. */
        unsigned drained = 0;
        while (channel->tx_event_armed &&
               (int32_t)(cpu->ccount - channel->next_tx_ccount) >= 0 &&
               drained++ < 4096u) {
            uint32_t event_ccount = channel->next_tx_ccount;

            channel->tx_event_armed = false;
            channel->tx_index = channel->pending_next_index;
            channel->tx_since_threshold = channel->pending_next_threshold;
            rmt_tx_event_kind_t kind = channel->pending_kind;
            channel->pending_kind = RMT_TX_EVENT_NONE;

            if (channel->tx_cb &&
                (channel->pending_count != 0 || kind == RMT_TX_EVENT_END ||
                 kind == RMT_TX_EVENT_ERROR)) {
                channel->tx_cb(channel->tx_cb_ctx, (int)index,
                               channel->pending_items,
                               channel->pending_count,
                               rmt_channel_tick_hz(channel),
                               rmt_channel_carrier_hz(&p->rmt, index),
                               kind == RMT_TX_EVENT_END ||
                                   kind == RMT_TX_EVENT_ERROR);
            }
            channel->pending_count = 0;

            switch (kind) {
            case RMT_TX_EVENT_THRESHOLD:
                p->rmt.int_raw |= RMT_TX_THRESHOLD_INT(index);
                /* The compatibility dispatcher runs the refill ISR inline. */
                rmt_irq_update(p);
                if (channel->tx_active)
                    rmt_plan_tx_segment_at(p, index, event_ccount);
                break;
            case RMT_TX_EVENT_END:
                rmt_tx_cancel(channel);
                p->rmt.int_raw |= RMT_TX_END_INT(index);
                rmt_irq_update(p);
                break;
            case RMT_TX_EVENT_LOOP:
                channel->tx_index = 0;
                if (channel->tx_active)
                    rmt_plan_tx_segment_at(p, index, event_ccount);
                break;
            case RMT_TX_EVENT_ERROR:
                channel->status_flags |= RMT_STATUS_MEM_EMPTY;
                rmt_tx_cancel(channel);
                p->rmt.int_raw |= RMT_ERROR_INT(index);
                rmt_irq_update(p);
                break;
            case RMT_TX_EVENT_NONE:
            default:
                break;
            }
        }
    }
    rmt_kick(p);
}

static uint32_t rmt_fifo_read(rmt_state_t *rmt, unsigned index) {
    rmt_channel_state_t *channel = &rmt->channel[index];
    size_t capacity = rmt_channel_capacity(rmt, index);
    if (capacity == 0 || channel->apb_index >= capacity) {
        channel->status_flags |= 1u << 31; /* APB_MEM_RD_ERR */
        return 0;
    }
    size_t word = index * RMT_MEM_WORDS_PER_CHANNEL + channel->apb_index++;
    return rmt->memory[word];
}

static void rmt_fifo_write(rmt_state_t *rmt, unsigned index, uint32_t value) {
    rmt_channel_state_t *channel = &rmt->channel[index];
    size_t capacity = rmt_channel_capacity(rmt, index);
    if (capacity == 0 || channel->apb_index >= capacity) {
        channel->status_flags |= 1u << 30; /* APB_MEM_WR_ERR */
        return;
    }
    size_t word = index * RMT_MEM_WORDS_PER_CHANNEL + channel->apb_index++;
    rmt->memory[word] = value;
}

static uint32_t rmt_read(void *ctx, uint32_t addr) {
    esp32_periph_t *p = ctx;
    rmt_state_t *rmt = &p->rmt;
    uint32_t off = addr - RMT_BASE;
    if ((off & 3u) != 0 || off >= PAGE_SIZE)
        return default_read(ctx, addr);

    if (off >= RMT_MEM_OFF)
        return rmt->memory[(off - RMT_MEM_OFF) / 4u];
    if (off < RMT_CONF0_OFF)
        return rmt_fifo_read(rmt, off / 4u);
    if (off >= RMT_CONF0_OFF && off < RMT_STATUS_OFF) {
        unsigned index = (off - RMT_CONF0_OFF) / 8u;
        return ((off - RMT_CONF0_OFF) & 4u) ?
            rmt->channel[index].conf1 : rmt->channel[index].conf0;
    }
    if (off >= RMT_STATUS_OFF && off < RMT_ADDR_OFF)
        return rmt_status_word(p, (off - RMT_STATUS_OFF) / 4u);
    if (off >= RMT_ADDR_OFF && off < RMT_INT_RAW_OFF)
        return rmt->channel[(off - RMT_ADDR_OFF) / 4u].apb_index;
    if (off == RMT_INT_RAW_OFF) return rmt->int_raw;
    if (off == RMT_INT_ST_OFF) return rmt->int_raw & rmt->int_ena;
    if (off == RMT_INT_ENA_OFF) return rmt->int_ena;
    if (off == RMT_INT_CLR_OFF) return 0;
    if (off >= RMT_CARRIER_DUTY_OFF && off < RMT_TX_LIMIT_OFF)
        return rmt->carrier_duty[(off - RMT_CARRIER_DUTY_OFF) / 4u];
    if (off >= RMT_TX_LIMIT_OFF && off < RMT_APB_CONF_OFF)
        return rmt->tx_limit[(off - RMT_TX_LIMIT_OFF) / 4u];
    if (off == RMT_APB_CONF_OFF) return rmt->apb_conf;
    if (off == RMT_DATE_OFF) return rmt->date;
    return 0;
}

static void rmt_write_conf1(esp32_periph_t *p, unsigned index,
                            uint32_t value) {
    rmt_channel_state_t *channel = &p->rmt.channel[index];
    uint32_t old = channel->conf1;
    channel->conf1 = value;

    if (value & RMT_CONF1_MEM_WR_RST) {
        channel->rx_index = 0;
        channel->status_flags &= ~RMT_STATUS_MEM_FULL;
    }
    if (value & RMT_CONF1_MEM_RD_RST) {
        channel->tx_index = 0;
        channel->tx_since_threshold = 0;
        channel->status_flags &= ~RMT_STATUS_MEM_EMPTY;
    }
    if (value & RMT_CONF1_APB_MEM_RST) {
        channel->apb_index = 0;
        channel->status_flags &= ~((1u << 31) | (1u << 30));
    }

    channel->rx_active = (value & RMT_CONF1_RX_EN) != 0;
    if (!(value & RMT_CONF1_TX_START) && (old & RMT_CONF1_TX_START))
        rmt_tx_cancel(channel);
    if ((value & RMT_CONF1_TX_START) && !(old & RMT_CONF1_TX_START)) {
        if (value & RMT_CONF1_MEM_OWNER_RX) {
            channel->status_flags |= RMT_STATUS_MEM_OWNER_ERR;
            channel->conf1 &= ~RMT_CONF1_TX_START;
            p->rmt.int_raw |= RMT_ERROR_INT(index);
            rmt_irq_update(p);
        } else {
            channel->status_flags &= ~(RMT_STATUS_MEM_EMPTY |
                                       RMT_STATUS_MEM_OWNER_ERR);
            channel->tx_active = true;
            rmt_plan_tx_segment(p, index);
        }
    }
    rmt_kick(p);
}

static void rmt_write(void *ctx, uint32_t addr, uint32_t value) {
    esp32_periph_t *p = ctx;
    rmt_state_t *rmt = &p->rmt;
    uint32_t off = addr - RMT_BASE;
    if ((off & 3u) != 0 || off >= PAGE_SIZE) {
        default_write(ctx, addr, value);
        return;
    }

    if (off >= RMT_MEM_OFF) {
        rmt->memory[(off - RMT_MEM_OFF) / 4u] = value;
        return;
    }
    if (off < RMT_CONF0_OFF) {
        rmt_fifo_write(rmt, off / 4u, value);
        return;
    }
    if (off >= RMT_CONF0_OFF && off < RMT_STATUS_OFF) {
        unsigned index = (off - RMT_CONF0_OFF) / 8u;
        if ((off - RMT_CONF0_OFF) & 4u)
            rmt_write_conf1(p, index, value);
        else
            rmt->channel[index].conf0 = value;
        return;
    }
    if (off >= RMT_STATUS_OFF && off < RMT_INT_RAW_OFF)
        return; /* STATUS/ADDR are read-only. */
    if (off == RMT_INT_RAW_OFF || off == RMT_INT_ST_OFF)
        return;
    if (off == RMT_INT_ENA_OFF) {
        rmt->int_ena = value;
        rmt_irq_update(p);
        return;
    }
    if (off == RMT_INT_CLR_OFF) {
        rmt->int_raw &= ~value;
        rmt_irq_update(p);
        return;
    }
    if (off >= RMT_CARRIER_DUTY_OFF && off < RMT_TX_LIMIT_OFF) {
        rmt->carrier_duty[(off - RMT_CARRIER_DUTY_OFF) / 4u] = value;
        return;
    }
    if (off >= RMT_TX_LIMIT_OFF && off < RMT_APB_CONF_OFF) {
        rmt->tx_limit[(off - RMT_TX_LIMIT_OFF) / 4u] = value & 0x1FFu;
        return;
    }
    if (off == RMT_APB_CONF_OFF) {
        rmt->apb_conf = value & (RMT_APB_FIFO_MASK | RMT_APB_TX_WRAP);
        return;
    }
    if (off == RMT_DATE_OFF)
        rmt->date = value;
}

/* ---- LEDC PWM controller ---- */

static uint32_t ledc_cpu_mhz(const esp32_periph_t *p) {
    uint32_t mhz = mem_read32(p->mem, ESP32_CPU_TICKS_PER_US_ADDR);
    return mhz >= 10u && mhz <= 240u ? mhz : 240u;
}

static uint64_t ledc_now_cycles(esp32_periph_t *p) {
    if (!p->cpu[0]) return p->ledc_time_cycles;

    uint32_t now = p->cpu[0]->ccount;
    if (!p->ledc_time_valid) {
        p->ledc_last_ccount = now;
        p->ledc_time_valid = true;
        return p->ledc_time_cycles;
    }

    /* Peripherals advance with the hardware clock, including time skipped by
     * delay()/WAITI. cycle_count deliberately measures executed guest work
     * and therefore cannot be used as a peripheral timebase. Sampling at
     * every peripheral event extends the 32-bit CCOUNT wrap into 64 bits. */
    uint32_t elapsed = now - p->ledc_last_ccount;
    if (elapsed < (uint32_t)INT32_MAX) {
        if (p->ledc_time_cycles > UINT64_MAX - elapsed)
            p->ledc_time_cycles = UINT64_MAX;
        else
            p->ledc_time_cycles += elapsed;
    }
    p->ledc_last_ccount = now;
    return p->ledc_time_cycles;
}

/* floor(value * multiplier / divisor) without overflowing solely because of
 * the intermediate product. LEDC's multiplier is at most 80 * 256. */
static uint64_t ledc_mul_div_floor(uint64_t value, uint64_t multiplier,
                                   uint64_t divisor) {
    if (divisor == 0) return 0;
    uint64_t quotient = value / divisor;
    uint64_t remainder = value % divisor;
    if (multiplier != 0 && quotient > UINT64_MAX / multiplier)
        return UINT64_MAX;
    uint64_t result = quotient * multiplier;
    uint64_t tail = remainder * multiplier / divisor;
    return result > UINT64_MAX - tail ? UINT64_MAX : result + tail;
}

static uint32_t ledc_channel_offset(unsigned speed_mode, unsigned channel) {
    return (speed_mode ? LEDC_LS_CHANNEL_OFF : 0u) +
           channel * LEDC_CHANNEL_STRIDE;
}

static uint32_t ledc_timer_offset(unsigned speed_mode, unsigned timer) {
    return (speed_mode ? LEDC_LS_TIMER_OFF : LEDC_TIMER_OFF) +
           timer * LEDC_TIMER_STRIDE;
}

static uint32_t ledc_timer_source_mhz(const esp32_periph_t *p,
                                      unsigned speed_mode,
                                      const ledc_timer_state_t *timer) {
    if (!(timer->active_conf & LEDC_TIMER_TICK_SEL)) return 1u;
    if (!speed_mode) return 80u;
    return p->ledc.regs[LEDC_CONF_OFF / 4u] & 1u ? 80u : 8u;
}

static uint32_t ledc_timer_resolution(const ledc_timer_state_t *timer) {
    uint32_t resolution = timer->active_conf & LEDC_TIMER_RES_MASK;
    return resolution <= 20u ? resolution : 20u;
}

static uint64_t ledc_timer_period_counts(const ledc_timer_state_t *timer) {
    return 1ull << ledc_timer_resolution(timer);
}

static bool ledc_timer_running(const ledc_timer_state_t *timer) {
    uint32_t divider = (timer->active_conf >> LEDC_TIMER_DIV_SHIFT) &
                       LEDC_TIMER_DIV_MASK;
    return divider != 0 &&
           !(timer->active_conf & (LEDC_TIMER_RESET | LEDC_TIMER_PAUSE));
}

static uint64_t ledc_timer_total_counts(esp32_periph_t *p,
                                        unsigned speed_mode,
                                        const ledc_timer_state_t *timer) {
    if (!ledc_timer_running(timer)) return timer->anchor_count;
    uint64_t now = ledc_now_cycles(p);
    uint64_t elapsed = now >= timer->anchor_cycles ?
                       now - timer->anchor_cycles : 0;
    uint64_t multiplier =
        (uint64_t)ledc_timer_source_mhz(p, speed_mode, timer) * 256u;
    uint64_t divisor =
        (uint64_t)ledc_cpu_mhz(p) *
        ((timer->active_conf >> LEDC_TIMER_DIV_SHIFT) &
         LEDC_TIMER_DIV_MASK);
    uint64_t advanced = ledc_mul_div_floor(elapsed, multiplier, divisor);
    return advanced > UINT64_MAX - timer->anchor_count ?
           UINT64_MAX : advanced + timer->anchor_count;
}

static uint32_t ledc_timer_count(esp32_periph_t *p,
                                 unsigned speed_mode, unsigned timer_index) {
    const ledc_timer_state_t *timer =
        &p->ledc.timer[speed_mode][timer_index];
    return (uint32_t)(ledc_timer_total_counts(p, speed_mode, timer) %
                      ledc_timer_period_counts(timer));
}

static uint64_t ledc_timer_counts_cpu_cycles(const esp32_periph_t *p,
                                             unsigned speed_mode,
                                             unsigned timer_index,
                                             uint64_t counts) {
    const ledc_timer_state_t *timer =
        &p->ledc.timer[speed_mode][timer_index];
    if (!ledc_timer_running(timer)) return 0;
    uint64_t numerator = counts *
        ((timer->active_conf >> LEDC_TIMER_DIV_SHIFT) &
         LEDC_TIMER_DIV_MASK) * ledc_cpu_mhz(p);
    uint64_t denominator =
        (uint64_t)ledc_timer_source_mhz(p, speed_mode, timer) * 256u;
    return (numerator + denominator - 1u) / denominator;
}

static uint64_t ledc_timer_period_cpu_cycles(const esp32_periph_t *p,
                                             unsigned speed_mode,
                                             unsigned timer_index) {
    return ledc_timer_counts_cpu_cycles(
        p, speed_mode, timer_index,
        ledc_timer_period_counts(&p->ledc.timer[speed_mode][timer_index]));
}

static uint64_t ledc_timer_remaining_cpu_cycles(esp32_periph_t *p,
                                                unsigned speed_mode,
                                                unsigned timer_index) {
    const ledc_timer_state_t *timer =
        &p->ledc.timer[speed_mode][timer_index];
    uint64_t period = ledc_timer_period_counts(timer);
    uint32_t count = ledc_timer_count(p, speed_mode, timer_index);
    return ledc_timer_counts_cpu_cycles(p, speed_mode, timer_index,
                                        period - count);
}

static uint32_t ledc_timer_frequency_hz(const esp32_periph_t *p,
                                        unsigned speed_mode,
                                        unsigned timer_index) {
    const ledc_timer_state_t *timer =
        &p->ledc.timer[speed_mode][timer_index];
    if (!ledc_timer_running(timer)) return 0;
    uint64_t divider =
        (timer->active_conf >> LEDC_TIMER_DIV_SHIFT) & LEDC_TIMER_DIV_MASK;
    uint64_t denominator = divider * ledc_timer_period_counts(timer);
    uint64_t numerator =
        (uint64_t)ledc_timer_source_mhz(p, speed_mode, timer) *
        1000000u * 256u;
    return denominator ? (uint32_t)(numerator / denominator) : 0;
}

static uint32_t ledc_current_duty_raw(esp32_periph_t *p,
                                      unsigned speed_mode,
                                      unsigned channel_index) {
    const ledc_channel_state_t *channel =
        &p->ledc.channel[speed_mode][channel_index];
    if (!channel->update_active) return channel->active_duty;

    uint64_t now = ledc_now_cycles(p);
    if (now < channel->update_start_cycle)
        return channel->update_old_duty;
    if (channel->update_scale == 0 || channel->update_steps == 0 ||
        now >= channel->update_end_cycle)
        return now >= channel->update_end_cycle ?
               channel->update_target_duty : channel->update_start_duty;

    uint64_t elapsed = now - channel->update_start_cycle;
    uint64_t steps = channel->update_step_cycles ?
        elapsed / channel->update_step_cycles : channel->update_steps;
    if (steps > channel->update_steps) steps = channel->update_steps;
    uint64_t delta = steps * channel->update_scale * 16u;
    if (channel->update_target_duty >= channel->update_start_duty) {
        uint64_t duty = (uint64_t)channel->update_start_duty + delta;
        return duty > channel->update_target_duty ?
               channel->update_target_duty : (uint32_t)duty;
    }
    return delta >= channel->update_start_duty -
                    channel->update_target_duty ?
           channel->update_target_duty :
           channel->update_start_duty - (uint32_t)delta;
}

static int ledc_channel_gpio(const esp32_periph_t *p, unsigned speed_mode,
                             unsigned channel, bool *inverted) {
    uint32_t signal = (speed_mode ? LEDC_LS_SIGNAL_BASE :
                                    LEDC_HS_SIGNAL_BASE) + channel;
    for (int gpio = 0; gpio < 40; gpio++) {
        uint32_t route = p->gpio.func_out_sel[gpio];
        if ((route & 0x1FFu) == signal) {
            if (inverted) *inverted = (route & (1u << 9)) != 0;
            return gpio;
        }
    }
    if (inverted) *inverted = false;
    return -1;
}

static void ledc_emit_channel(esp32_periph_t *p, unsigned speed_mode,
                              unsigned channel_index, bool force) {
    ledc_channel_state_t *channel =
        &p->ledc.channel[speed_mode][channel_index];
    uint32_t base = ledc_channel_offset(speed_mode, channel_index);
    uint32_t conf0 = p->ledc.regs[base / 4u];
    unsigned timer_index = conf0 & LEDC_CH_TIMER_SEL_MASK;
    const ledc_timer_state_t *timer =
        &p->ledc.timer[speed_mode][timer_index];
    bool inverted = false;
    int gpio = ledc_channel_gpio(p, speed_mode, channel_index, &inverted);
    uint32_t frequency = ledc_timer_frequency_hz(p, speed_mode, timer_index);
    uint32_t duty = ledc_current_duty_raw(p, speed_mode, channel_index) >> 4;
    uint32_t resolution = ledc_timer_resolution(timer);
    uint32_t duty_max = resolution == 0 ? 0u :
                        (uint32_t)((1ull << resolution) - 1u);
    bool enabled = gpio >= 0 && (conf0 & LEDC_CH_SIG_OUT_EN) != 0 &&
                   ledc_timer_running(timer);

    bool changed = !channel->output_reported || gpio != channel->last_gpio ||
        frequency != channel->last_frequency_hz ||
        duty != channel->last_duty || duty_max != channel->last_duty_max ||
        enabled != channel->last_enabled ||
        inverted != channel->last_inverted;
    if (!force && !changed) return;

    channel->last_gpio = gpio;
    channel->last_frequency_hz = frequency;
    channel->last_duty = duty;
    channel->last_duty_max = duty_max;
    channel->last_enabled = enabled;
    channel->last_inverted = inverted;
    channel->output_reported = true;

    if (channel->output_cb)
        channel->output_cb(channel->output_cb_ctx, (int)speed_mode,
                           (int)channel_index, gpio, frequency, duty,
                           duty_max, enabled, inverted);

    sbx_event_t event = { .kind = SBX_EV_PWM_OUT, .cycle = ledc_now_cycles(p) };
    event.pwm_out.gpio = (int8_t)gpio;
    event.pwm_out.speed_mode = (uint8_t)speed_mode;
    event.pwm_out.channel = (uint8_t)channel_index;
    event.pwm_out.enabled = enabled ? 1u : 0u;
    event.pwm_out.inverted = inverted ? 1u : 0u;
    event.pwm_out.frequency_hz = frequency;
    event.pwm_out.duty = duty;
    event.pwm_out.duty_max = duty_max;
    sbx_events_emit(&event);
}

static void ledc_emit_timer_channels(esp32_periph_t *p, unsigned speed_mode,
                                     unsigned timer_index) {
    for (unsigned channel = 0; channel < LEDC_CHANNEL_COUNT; channel++) {
        uint32_t base = ledc_channel_offset(speed_mode, channel);
        if ((p->ledc.regs[base / 4u] & LEDC_CH_TIMER_SEL_MASK) == timer_index)
            ledc_emit_channel(p, speed_mode, channel, false);
    }
}

static void ledc_update_irq(esp32_periph_t *p) {
    uint32_t raw = p->ledc.regs[LEDC_INT_RAW_OFF / 4u];
    uint32_t ena = p->ledc.regs[LEDC_INT_ENA_OFF / 4u];
    if (raw & ena)
        periph_assert_interrupt(p, LEDC_INTR_SOURCE);
    else
        periph_deassert_interrupt(p, LEDC_INTR_SOURCE);
}

static int ledc_channel_from_offset(uint32_t off, uint32_t *channel_base,
                                    int *low_speed) {
    if (off < LEDC_LS_CHANNEL_OFF) {
        int channel = (int)(off / LEDC_CHANNEL_STRIDE);
        if (channel < (int)LEDC_CHANNEL_COUNT) {
            *channel_base = (uint32_t)channel * LEDC_CHANNEL_STRIDE;
            *low_speed = 0;
            return channel;
        }
    } else if (off < LEDC_TIMER_OFF) {
        int channel = (int)((off - LEDC_LS_CHANNEL_OFF) /
                            LEDC_CHANNEL_STRIDE);
        if (channel < (int)LEDC_CHANNEL_COUNT) {
            *channel_base = LEDC_LS_CHANNEL_OFF +
                            (uint32_t)channel * LEDC_CHANNEL_STRIDE;
            *low_speed = 1;
            return channel;
        }
    }
    return -1;
}

static int ledc_timer_from_offset(uint32_t off, unsigned *speed_mode,
                                  unsigned *timer_index,
                                  uint32_t *timer_base) {
    if (off >= LEDC_TIMER_OFF && off < LEDC_LS_TIMER_OFF) {
        *speed_mode = 0;
        *timer_index = (off - LEDC_TIMER_OFF) / LEDC_TIMER_STRIDE;
    } else if (off >= LEDC_LS_TIMER_OFF && off < LEDC_INT_RAW_OFF) {
        *speed_mode = 1;
        *timer_index = (off - LEDC_LS_TIMER_OFF) / LEDC_TIMER_STRIDE;
    } else {
        return -1;
    }
    if (*timer_index >= LEDC_TIMER_COUNT) return -1;
    *timer_base = ledc_timer_offset(*speed_mode, *timer_index);
    return 0;
}

static void ledc_reanchor_timer(esp32_periph_t *p, unsigned speed_mode,
                                unsigned timer_index, uint32_t conf) {
    ledc_timer_state_t *timer =
        &p->ledc.timer[speed_mode][timer_index];
    uint32_t count = ledc_timer_count(p, speed_mode, timer_index);
    timer->active_conf = conf;
    timer->anchor_count = conf & LEDC_TIMER_RESET ? 0u : count;
    timer->anchor_cycles = ledc_now_cycles(p);
    timer->reported_wraps = 0;
}

static void ledc_sync_timer_overflows(esp32_periph_t *p) {
    for (unsigned speed = 0; speed < LEDC_SPEED_MODE_COUNT; speed++) {
        for (unsigned index = 0; index < LEDC_TIMER_COUNT; index++) {
            ledc_timer_state_t *timer = &p->ledc.timer[speed][index];
            if (!ledc_timer_running(timer)) continue;
            uint64_t wraps = ledc_timer_total_counts(p, speed, timer) /
                             ledc_timer_period_counts(timer);
            if (wraps > timer->reported_wraps) {
                timer->reported_wraps = wraps;
                p->ledc.regs[LEDC_INT_RAW_OFF / 4u] |=
                    1u << (index + speed * LEDC_TIMER_COUNT);
            }
        }
    }
}

static void ledc_kick(esp32_periph_t *p) {
    for (int core = 0; core < 2; core++)
        if (p->cpu[core]) xtensa_recompute_next_timer(p->cpu[core]);
}

static uint64_t ledc_add_saturating(uint64_t value, uint64_t addend) {
    return value > UINT64_MAX - addend ? UINT64_MAX : value + addend;
}

static void ledc_start_channel_update(esp32_periph_t *p,
                                      unsigned speed_mode,
                                      unsigned channel_index,
                                      uint32_t conf1) {
    ledc_channel_state_t *channel =
        &p->ledc.channel[speed_mode][channel_index];
    uint32_t base = ledc_channel_offset(speed_mode, channel_index);
    unsigned timer_index =
        p->ledc.regs[base / 4u] & LEDC_CH_TIMER_SEL_MASK;
    uint64_t period_cycles =
        ledc_timer_period_cpu_cycles(p, speed_mode, timer_index);
    if (period_cycles == 0) {
        channel->update_active = false;
        channel->update_started = false;
        return;
    }

    channel->update_old_duty =
        ledc_current_duty_raw(p, speed_mode, channel_index);
    channel->active_duty = channel->update_old_duty;
    channel->update_start_duty =
        p->ledc.regs[(base + 0x08u) / 4u] & LEDC_CH_DUTY_MASK;
    channel->update_scale = conf1 & LEDC_CH_DUTY_SCALE_MASK;
    uint32_t cycle_count =
        (conf1 >> LEDC_CH_DUTY_CYCLE_SHIFT) & 0x3FFu;
    channel->update_steps =
        (conf1 >> LEDC_CH_DUTY_NUM_SHIFT) & LEDC_CH_DUTY_NUM_MASK;
    if (cycle_count == 0) cycle_count = 1;

    uint64_t step_cycles = period_cycles;
    if (cycle_count > UINT64_MAX / step_cycles)
        step_cycles = UINT64_MAX;
    else
        step_cycles *= cycle_count;
    channel->update_step_cycles = step_cycles;

    uint64_t delta =
        (uint64_t)channel->update_scale * channel->update_steps * 16u;
    uint64_t target;
    if (conf1 & LEDC_CH_DUTY_INC) {
        target = (uint64_t)channel->update_start_duty + delta;
        if (target > LEDC_CH_DUTY_MASK) target = LEDC_CH_DUTY_MASK;
    } else {
        target = delta >= channel->update_start_duty ? 0u :
                 channel->update_start_duty - delta;
    }
    channel->update_target_duty = (uint32_t)target;

    uint64_t now = ledc_now_cycles(p);
    uint64_t boundary_cycles =
        ledc_timer_remaining_cpu_cycles(p, speed_mode, timer_index);
    channel->update_start_cycle = ledc_add_saturating(now, boundary_cycles);
    uint64_t fade_cycles = 0;
    if (channel->update_scale != 0 && channel->update_steps != 0) {
        fade_cycles = channel->update_steps > UINT64_MAX / step_cycles ?
                      UINT64_MAX : channel->update_steps * step_cycles;
    }
    channel->update_end_cycle =
        ledc_add_saturating(channel->update_start_cycle, fade_cycles);
    channel->update_active = true;
    channel->update_started = false;
    ledc_kick(p);
}

static void ledc_finish_channel_update(esp32_periph_t *p,
                                       unsigned speed_mode,
                                       unsigned channel_index) {
    ledc_channel_state_t *channel =
        &p->ledc.channel[speed_mode][channel_index];
    uint32_t base = ledc_channel_offset(speed_mode, channel_index);
    channel->active_duty = channel->update_target_duty;
    channel->update_active = false;
    channel->update_started = false;
    p->ledc.regs[(base + 0x0Cu) / 4u] &= ~LEDC_CH_DUTY_START;
    p->ledc.regs[LEDC_INT_RAW_OFF / 4u] |=
        1u << (8u + channel_index + speed_mode * LEDC_CHANNEL_COUNT);
    ledc_emit_channel(p, speed_mode, channel_index, false);
    ledc_update_irq(p);
}

static uint32_t ledc_deadline_ccount(esp32_periph_t *p,
                                     const xtensa_cpu_t *cpu,
                                     uint64_t deadline) {
    uint64_t now = ledc_now_cycles(p);
    if (deadline <= now) return cpu->ccount;
    uint64_t distance = deadline - now;
    if (distance >= (uint64_t)INT32_MAX)
        distance = (uint64_t)INT32_MAX - 1u;
    return cpu->ccount + (uint32_t)distance;
}

static uint32_t ledc_next_fire(esp32_periph_t *p, xtensa_cpu_t *cpu) {
    if (!p || !cpu || cpu != p->cpu[0]) return UINT32_MAX;
    bool have = false;
    uint32_t best = UINT32_MAX;
    uint32_t best_distance = 0;

    for (unsigned speed = 0; speed < LEDC_SPEED_MODE_COUNT; speed++) {
        for (unsigned channel_index = 0;
             channel_index < LEDC_CHANNEL_COUNT; channel_index++) {
            ledc_channel_state_t *channel =
                &p->ledc.channel[speed][channel_index];
            if (!channel->update_active) continue;
            uint64_t deadline = channel->update_started ?
                channel->update_end_cycle : channel->update_start_cycle;
            uint32_t event = ledc_deadline_ccount(p, cpu, deadline);
            uint32_t distance = event - cpu->ccount;
            if (!have || distance < best_distance) {
                have = true;
                best = event;
                best_distance = distance;
            }
        }
    }

    uint32_t raw = p->ledc.regs[LEDC_INT_RAW_OFF / 4u];
    uint32_t ena = p->ledc.regs[LEDC_INT_ENA_OFF / 4u];
    for (unsigned speed = 0; speed < LEDC_SPEED_MODE_COUNT; speed++) {
        for (unsigned timer_index = 0;
             timer_index < LEDC_TIMER_COUNT; timer_index++) {
            unsigned bit = timer_index + speed * LEDC_TIMER_COUNT;
            if (!(ena & (1u << bit)) || (raw & (1u << bit))) continue;
            const ledc_timer_state_t *timer =
                &p->ledc.timer[speed][timer_index];
            if (!ledc_timer_running(timer)) continue;
            uint64_t remaining_cycles =
                ledc_timer_remaining_cpu_cycles(p, speed, timer_index);
            uint32_t event = ledc_deadline_ccount(
                p, cpu, ledc_add_saturating(ledc_now_cycles(p),
                                             remaining_cycles));
            uint32_t distance = event - cpu->ccount;
            if (!have || distance < best_distance) {
                have = true;
                best = event;
                best_distance = distance;
            }
        }
    }
    return have ? best : UINT32_MAX;
}

static void ledc_eval_events(esp32_periph_t *p, xtensa_cpu_t *cpu) {
    if (!p || !cpu || cpu != p->cpu[0]) return;
    ledc_sync_timer_overflows(p);

    for (unsigned speed = 0; speed < LEDC_SPEED_MODE_COUNT; speed++) {
        for (unsigned channel_index = 0;
             channel_index < LEDC_CHANNEL_COUNT; channel_index++) {
            unsigned drained = 0;
            ledc_channel_state_t *channel =
                &p->ledc.channel[speed][channel_index];
            while (channel->update_active && drained++ < 1024u) {
                uint64_t now = ledc_now_cycles(p);
                if (!channel->update_started) {
                    if (now < channel->update_start_cycle) break;
                    channel->active_duty = channel->update_start_duty;
                    channel->update_started = true;
                    ledc_emit_channel(p, speed, channel_index, false);
                    if (channel->update_end_cycle <= now)
                        ledc_finish_channel_update(p, speed, channel_index);
                } else {
                    if (now < channel->update_end_cycle) break;
                    ledc_finish_channel_update(p, speed, channel_index);
                }
            }
        }
    }
    ledc_update_irq(p);
    ledc_kick(p);
}

static void ledc_reset_state(esp32_periph_t *p) {
    periph_ledc_output_fn callbacks[LEDC_SPEED_MODE_COUNT][LEDC_CHANNEL_COUNT];
    void *contexts[LEDC_SPEED_MODE_COUNT][LEDC_CHANNEL_COUNT];
    bool reported[LEDC_SPEED_MODE_COUNT][LEDC_CHANNEL_COUNT];
    for (unsigned speed = 0; speed < LEDC_SPEED_MODE_COUNT; speed++) {
        for (unsigned channel = 0; channel < LEDC_CHANNEL_COUNT; channel++) {
            callbacks[speed][channel] =
                p->ledc.channel[speed][channel].output_cb;
            contexts[speed][channel] =
                p->ledc.channel[speed][channel].output_cb_ctx;
            reported[speed][channel] =
                p->ledc.channel[speed][channel].output_reported;
        }
    }

    memset(&p->ledc, 0, sizeof(p->ledc));
    for (unsigned speed = 0; speed < LEDC_SPEED_MODE_COUNT; speed++) {
        for (unsigned timer = 0; timer < LEDC_TIMER_COUNT; timer++) {
            uint32_t off = ledc_timer_offset(speed, timer);
            p->ledc.regs[off / 4u] = LEDC_TIMER_RESET;
            p->ledc.timer[speed][timer].active_conf = LEDC_TIMER_RESET;
            p->ledc.timer[speed][timer].anchor_cycles = ledc_now_cycles(p);
        }
        for (unsigned channel = 0; channel < LEDC_CHANNEL_COUNT; channel++) {
            p->ledc.channel[speed][channel].output_cb =
                callbacks[speed][channel];
            p->ledc.channel[speed][channel].output_cb_ctx =
                contexts[speed][channel];
        }
    }
    p->ledc.regs[LEDC_DATE_OFF / 4u] = 0x16031700u;
    ledc_update_irq(p);
    ledc_kick(p);

    for (unsigned speed = 0; speed < LEDC_SPEED_MODE_COUNT; speed++)
        for (unsigned channel = 0; channel < LEDC_CHANNEL_COUNT; channel++)
            if (reported[speed][channel] || callbacks[speed][channel])
                ledc_emit_channel(p, speed, channel, true);
}

static void ledc_gpio_route_changed(esp32_periph_t *p, int gpio,
                                    uint32_t before, uint32_t after) {
    (void)gpio;
    uint32_t routes[2] = {before & 0x1FFu, after & 0x1FFu};
    for (unsigned i = 0; i < 2; i++) {
        uint32_t route = routes[i];
        if (route >= LEDC_HS_SIGNAL_BASE &&
            route < LEDC_HS_SIGNAL_BASE + LEDC_CHANNEL_COUNT)
            ledc_emit_channel(p, 0, route - LEDC_HS_SIGNAL_BASE, true);
        else if (route >= LEDC_LS_SIGNAL_BASE &&
                 route < LEDC_LS_SIGNAL_BASE + LEDC_CHANNEL_COUNT)
            ledc_emit_channel(p, 1, route - LEDC_LS_SIGNAL_BASE, true);
    }
}

static uint32_t ledc_read(void *ctx, uint32_t addr) {
    esp32_periph_t *p = ctx;
    uint32_t off = addr - LEDC_BASE;
    if ((off & 3u) || off > LEDC_DATE_OFF)
        return default_read(ctx, addr);

    if (p->cpu[0]) ledc_eval_events(p, p->cpu[0]);

    if (off == LEDC_INT_ST_OFF)
        return p->ledc.regs[LEDC_INT_RAW_OFF / 4u] &
               p->ledc.regs[LEDC_INT_ENA_OFF / 4u];
    if (off == LEDC_INT_CLR_OFF)
        return 0; /* write-only */

    uint32_t channel_base = 0;
    int low_speed = 0;
    int channel = ledc_channel_from_offset(off, &channel_base, &low_speed);
    if (channel >= 0 && off - channel_base == 0x10u) {
        return ledc_current_duty_raw(p, (unsigned)low_speed,
                                     (unsigned)channel) & LEDC_CH_DUTY_MASK;
    }

    unsigned speed_mode = 0;
    unsigned timer_index = 0;
    uint32_t timer_base = 0;
    if (ledc_timer_from_offset(off, &speed_mode, &timer_index,
                               &timer_base) == 0 &&
        off - timer_base == 0x04u) {
        return ledc_timer_count(p, speed_mode, timer_index);
    }

    return p->ledc.regs[off / 4u];
}

static void ledc_write(void *ctx, uint32_t addr, uint32_t val) {
    esp32_periph_t *p = ctx;
    uint32_t off = addr - LEDC_BASE;
    if ((off & 3u) || off > LEDC_DATE_OFF) {
        default_write(ctx, addr, val);
        return;
    }

    if (p->cpu[0]) ledc_eval_events(p, p->cpu[0]);

    if (off == LEDC_INT_CLR_OFF) {
        p->ledc.regs[LEDC_INT_RAW_OFF / 4u] &=
            ~(val & LEDC_INT_VALID_MASK);
        ledc_update_irq(p);
        ledc_kick(p);
        return;
    }
    if (off == LEDC_INT_RAW_OFF || off == LEDC_INT_ST_OFF)
        return; /* read-only */
    if (off == LEDC_INT_ENA_OFF) {
        p->ledc.regs[off / 4u] = val & LEDC_INT_VALID_MASK;
        ledc_update_irq(p);
        ledc_kick(p);
        return;
    }

    unsigned speed_mode = 0;
    unsigned timer_index = 0;
    uint32_t timer_base = 0;
    if (ledc_timer_from_offset(off, &speed_mode, &timer_index,
                               &timer_base) == 0) {
        if (off - timer_base == 0x04u)
            return; /* live timer counter is read-only */
        p->ledc.regs[off / 4u] = val;
        ledc_reanchor_timer(p, speed_mode, timer_index, val);
        if (ledc_timer_running(&p->ledc.timer[speed_mode][timer_index])) {
            for (unsigned channel = 0; channel < LEDC_CHANNEL_COUNT;
                 channel++) {
                uint32_t base = ledc_channel_offset(speed_mode, channel);
                uint32_t conf0 = p->ledc.regs[base / 4u];
                uint32_t conf1 = p->ledc.regs[(base + 0x0Cu) / 4u];
                if ((conf0 & LEDC_CH_TIMER_SEL_MASK) == timer_index &&
                    (conf1 & LEDC_CH_DUTY_START) &&
                    !p->ledc.channel[speed_mode][channel].update_active)
                    ledc_start_channel_update(p, speed_mode, channel, conf1);
            }
        }
        ledc_emit_timer_channels(p, speed_mode, timer_index);
        ledc_kick(p);
        return;
    }

    uint32_t channel_base = 0;
    int low_speed = 0;
    int channel = ledc_channel_from_offset(off, &channel_base, &low_speed);
    if (channel >= 0) {
        uint32_t relative = off - channel_base;
        if (relative == 0x10u)
            return; /* live duty is read-only */
        p->ledc.regs[off / 4u] = val;
        if (relative == 0x0Cu) {
            ledc_channel_state_t *state =
                &p->ledc.channel[(unsigned)low_speed][(unsigned)channel];
            if (val & LEDC_CH_DUTY_START) {
                ledc_start_channel_update(p, (unsigned)low_speed,
                                          (unsigned)channel, val);
            } else {
                state->active_duty = ledc_current_duty_raw(
                    p, (unsigned)low_speed, (unsigned)channel);
                state->update_active = false;
                state->update_started = false;
                ledc_emit_channel(p, (unsigned)low_speed,
                                  (unsigned)channel, false);
                ledc_kick(p);
            }
        } else if (relative == 0u) {
            ledc_emit_channel(p, (unsigned)low_speed,
                              (unsigned)channel, false);
        }
        return;
    }

    if (off == LEDC_CONF_OFF) {
        uint32_t counts[LEDC_TIMER_COUNT];
        for (unsigned timer = 0; timer < LEDC_TIMER_COUNT; timer++)
            counts[timer] = ledc_timer_count(p, 1, timer);
        p->ledc.regs[off / 4u] = val;
        uint64_t now = ledc_now_cycles(p);
        for (unsigned timer = 0; timer < LEDC_TIMER_COUNT; timer++) {
            p->ledc.timer[1][timer].anchor_count = counts[timer];
            p->ledc.timer[1][timer].anchor_cycles = now;
            p->ledc.timer[1][timer].reported_wraps = 0;
        }
        for (unsigned channel = 0; channel < LEDC_CHANNEL_COUNT; channel++) {
            ledc_channel_state_t *state = &p->ledc.channel[1][channel];
            if (state->update_active && !state->update_started) {
                uint32_t base = ledc_channel_offset(1, channel);
                ledc_start_channel_update(
                    p, 1, channel, p->ledc.regs[(base + 0x0Cu) / 4u]);
            }
        }
        for (unsigned timer = 0; timer < LEDC_TIMER_COUNT; timer++)
            ledc_emit_timer_channels(p, 1, timer);
        ledc_kick(p);
        return;
    }
    p->ledc.regs[off / 4u] = val;
}

/* ---- I2S0/I2S1 + circular lldesc DMA ---- */

static const int i2s_intr_sources[I2S_PORT_COUNT] = {32, 33};

static int i2s_port_from_addr(uint32_t addr) {
    if (addr >= I2S0_BASE && addr < I2S0_BASE + PAGE_SIZE) return 0;
    if (addr >= I2S1_BASE && addr < I2S1_BASE + PAGE_SIZE) return 1;
    return -1;
}

static uint32_t i2s_base(int port) {
    return port == 0 ? I2S0_BASE : I2S1_BASE;
}

static bool i2s_dma_range_mapped(esp32_periph_t *p, uint32_t addr,
                                 size_t len, bool writable) {
    while (len > 0) {
        size_t page_left = 0x1000u - (addr & 0xFFFu);
        size_t chunk = len < page_left ? len : page_left;
        const uint8_t *ptr = writable ? mem_get_ptr_w(p->mem, addr) :
                                        mem_get_ptr(p->mem, addr);
        if (!ptr) return false;
        addr += (uint32_t)chunk;
        len -= chunk;
    }
    return true;
}

static uint32_t i2s_first_desc(uint32_t link) {
    return 0x3FF00000u | (link & I2S_LINK_ADDR_MASK);
}

static uint32_t i2s_next_desc(uint32_t next) {
    return next != 0 && next < 0x00100000u ? 0x3FF00000u | next : next;
}

static uint8_t i2s_bits_per_sample(const i2s_state_t *s, bool tx) {
    uint32_t rate = s->regs[I2S_SAMPLE_RATE_OFF / 4u];
    uint32_t bits = tx ? ((rate >> 12) & 0x3Fu) : ((rate >> 18) & 0x3Fu);
    if (bits == 0 || bits > 32) bits = 16;
    return (uint8_t)bits;
}

static uint8_t i2s_channel_count(const i2s_state_t *s, bool tx) {
    uint32_t conf = s->regs[I2S_CONF_OFF / 4u];
    return conf & (tx ? I2S_CONF_TX_MONO : I2S_CONF_RX_MONO) ? 1u : 2u;
}

static uint32_t i2s_sample_rate(const i2s_state_t *s, bool tx) {
    uint32_t clkm = s->regs[I2S_CLKM_CONF_OFF / 4u];
    uint32_t num = clkm & 0xFFu;
    uint32_t b = (clkm >> 8) & 0x3Fu;
    uint32_t a = (clkm >> 14) & 0x3Fu;
    if (num == 0) num = 4;
    uint64_t divider_64 = (uint64_t)num * 64u;
    if (a != 0) divider_64 += (uint64_t)b * 64u / a;
    if (divider_64 == 0) divider_64 = 256u;
    uint64_t module_hz = 160000000ull * 64u / divider_64;

    uint32_t rate = s->regs[I2S_SAMPLE_RATE_OFF / 4u];
    uint32_t bck_div = tx ? (rate & 0x3Fu) : ((rate >> 6) & 0x3Fu);
    if (bck_div == 0) bck_div = 1;
    uint32_t frame_bits = (uint32_t)i2s_bits_per_sample(s, tx) *
                          i2s_channel_count(s, tx);
    uint64_t sample_hz = module_hz / bck_div / frame_bits;
    if (sample_hz == 0) sample_hz = 1;
    if (sample_hz > UINT32_MAX) sample_hz = UINT32_MAX;
    return (uint32_t)sample_hz;
}

static uint32_t i2s_descriptor_cycles(const i2s_state_t *s, bool tx,
                                      size_t len) {
    uint32_t bytes_per_frame =
        ((uint32_t)i2s_bits_per_sample(s, tx) + 7u) / 8u;
    bytes_per_frame *= i2s_channel_count(s, tx);
    uint64_t bytes_per_second =
        (uint64_t)i2s_sample_rate(s, tx) * bytes_per_frame;
    uint64_t cycles = bytes_per_second ?
        ((uint64_t)len * 240000000ull + bytes_per_second - 1u) /
            bytes_per_second : 100000u;
    if (cycles == 0) cycles = 1;
    /* Event comparisons use signed modular ccount distances, so keep one
     * descriptor deadline within the unambiguous half of the 32-bit range. */
    if (cycles > INT32_MAX) cycles = INT32_MAX;
    return (uint32_t)cycles;
}

static void i2s_irq_update(esp32_periph_t *p, int port) {
    i2s_state_t *s = &p->i2s[port];
    if (s->int_raw & s->int_ena & I2S_INT_VALID_MASK)
        periph_assert_interrupt(p, i2s_intr_sources[port]);
    else
        periph_deassert_interrupt(p, i2s_intr_sources[port]);
}

static size_t i2s_rx_fifo_push(i2s_state_t *s, const uint8_t *data,
                               size_t len) {
    size_t accepted = len;
    if (accepted > I2S_RX_FIFO_SIZE - s->rx_len)
        accepted = I2S_RX_FIFO_SIZE - s->rx_len;
    for (size_t i = 0; i < accepted; i++) {
        size_t tail = (s->rx_head + s->rx_len) % I2S_RX_FIFO_SIZE;
        s->rx_fifo[tail] = data[i];
        s->rx_len++;
    }
    return accepted;
}

static uint8_t i2s_rx_fifo_pop(i2s_state_t *s) {
    if (s->rx_len == 0) return 0;
    uint8_t value = s->rx_fifo[s->rx_head];
    s->rx_head = (s->rx_head + 1u) % I2S_RX_FIFO_SIZE;
    s->rx_len--;
    return value;
}

static void i2s_emit_tx(esp32_periph_t *p, int port, const uint8_t *data,
                        size_t len) {
    i2s_state_t *s = &p->i2s[port];
    uint32_t sample_rate = i2s_sample_rate(s, true);
    uint8_t bits = i2s_bits_per_sample(s, true);
    uint8_t channels = i2s_channel_count(s, true);
    if (s->tx_cb)
        s->tx_cb(s->tx_cb_ctx, port, data, len, sample_rate, bits, channels);

    sbx_event_t ev = { .kind = SBX_EV_I2S_TX, .cycle = 0 };
    ev.i2s_tx.port = (uint8_t)port;
    ev.i2s_tx.bits_per_sample = bits;
    ev.i2s_tx.channels = channels;
    ev.i2s_tx.len = (uint16_t)len;
    ev.i2s_tx.sample_rate = sample_rate;
    ev.i2s_tx.data = data;
    sbx_events_emit(&ev);
}

static bool i2s_process_tx_descriptor(esp32_periph_t *p, int port) {
    i2s_state_t *s = &p->i2s[port];
    uint32_t desc = s->tx_desc;
    if (!desc || !i2s_dma_range_mapped(p, desc, 12, false)) {
        s->int_raw |= I2S_INT_OUT_DSCR_ERR;
        s->tx_link_running = false;
        s->tx_active = false;
        i2s_irq_update(p, port);
        return false;
    }

    uint32_t ctrl = mem_read32(p->mem, desc);
    uint32_t buf = mem_read32(p->mem, desc + 4u);
    uint32_t next = i2s_next_desc(mem_read32(p->mem, desc + 8u));
    size_t size = ctrl & I2S_DESC_SIZE_MASK;
    size_t len = (ctrl & I2S_DESC_LENGTH_MASK) >> I2S_DESC_LENGTH_SHIFT;
    s->regs[I2S_OUTLINK_DSCR_OFF / 4u] = desc;
    s->regs[I2S_OUTLINK_DSCR_BF0_OFF / 4u] = next;
    s->regs[I2S_OUTLINK_DSCR_BF1_OFF / 4u] = buf;

    if (!(ctrl & I2S_DESC_OWNER) || len > size || len > I2S_DMA_MAX_BUFFER ||
        (len != 0 && !i2s_dma_range_mapped(p, buf, len, false))) {
        s->int_raw |= I2S_INT_OUT_DSCR_ERR;
        s->tx_link_running = false;
        s->tx_active = false;
        i2s_irq_update(p, port);
        return false;
    }

    uint8_t audio[I2S_DMA_MAX_BUFFER];
    for (size_t i = 0; i < len; i++)
        audio[i] = mem_read8(p->mem, buf + (uint32_t)i);
    if (len != 0)
        i2s_emit_tx(p, port, audio, len);

    s->regs[I2S_OUT_EOF_DESC_OFF / 4u] = desc;
    s->regs[I2S_OUT_EOF_BUF_OFF / 4u] = buf;
    s->int_raw |= I2S_INT_OUT_DONE;
    if (ctrl & I2S_DESC_EOF)
        s->int_raw |= I2S_INT_OUT_EOF;

    if (next == 0) {
        s->int_raw |= I2S_INT_OUT_TOTAL_EOF;
        s->tx_link_running = false;
        s->tx_active = false;
        s->tx_desc = 0;
    } else {
        s->tx_desc = next;
    }
    i2s_irq_update(p, port);
    return true;
}

static bool i2s_process_rx_descriptor(esp32_periph_t *p, int port) {
    i2s_state_t *s = &p->i2s[port];
    uint32_t desc = s->rx_desc;
    if (!desc || !i2s_dma_range_mapped(p, desc, 12, true)) {
        s->int_raw |= I2S_INT_IN_DSCR_ERR;
        s->rx_link_running = false;
        s->rx_active = false;
        i2s_irq_update(p, port);
        return false;
    }

    uint32_t ctrl = mem_read32(p->mem, desc);
    uint32_t buf = mem_read32(p->mem, desc + 4u);
    uint32_t next = i2s_next_desc(mem_read32(p->mem, desc + 8u));
    size_t size = ctrl & I2S_DESC_SIZE_MASK;
    size_t len = size;
    uint32_t eof_num = s->regs[I2S_RXEOF_NUM_OFF / 4u];
    /* Classic ESP32 RX_EOF_NUM counts 32-bit FIFO words, while lldesc size
     * and length fields count bytes. ESP-IDF therefore programs 32 here for
     * a 128-byte stereo/16-bit DMA buffer. */
    uint64_t eof_bytes = (uint64_t)eof_num * sizeof(uint32_t);
    if (eof_num != 0 && len > eof_bytes) len = (size_t)eof_bytes;
    s->regs[I2S_INLINK_DSCR_OFF / 4u] = desc;
    s->regs[I2S_INLINK_DSCR_BF0_OFF / 4u] = next;
    s->regs[I2S_INLINK_DSCR_BF1_OFF / 4u] = buf;

    if (!(ctrl & I2S_DESC_OWNER) || len == 0 || len > I2S_DMA_MAX_BUFFER ||
        !i2s_dma_range_mapped(p, buf, len, true)) {
        s->int_raw |= I2S_INT_IN_DSCR_ERR;
        s->rx_link_running = false;
        s->rx_active = false;
        i2s_irq_update(p, port);
        return false;
    }

    for (size_t i = 0; i < len; i++)
        mem_write8(p->mem, buf + (uint32_t)i, i2s_rx_fifo_pop(s));
    ctrl &= ~I2S_DESC_LENGTH_MASK;
    ctrl |= ((uint32_t)len << I2S_DESC_LENGTH_SHIFT) &
            I2S_DESC_LENGTH_MASK;
    mem_write32(p->mem, desc, ctrl);

    s->regs[I2S_IN_EOF_DESC_OFF / 4u] = desc;
    s->int_raw |= I2S_INT_IN_DONE | I2S_INT_IN_SUC_EOF;
    if (next == 0) {
        s->int_raw |= I2S_INT_IN_DSCR_EMPTY;
        s->rx_link_running = false;
        s->rx_active = false;
        s->rx_desc = 0;
    } else {
        s->rx_desc = next;
    }
    i2s_irq_update(p, port);
    return true;
}

static int i2s_ring_descriptor_count(esp32_periph_t *p, uint32_t first) {
    uint32_t desc = first;
    for (int count = 1; count <= I2S_DMA_MAX_DESCRIPTORS; count++) {
        if (!i2s_dma_range_mapped(p, desc, 12, false)) return 0;
        uint32_t next = i2s_next_desc(mem_read32(p->mem, desc + 8u));
        if (next == first) return count;
        if (next == 0 || next == desc) return count;
        desc = next;
    }
    return I2S_DMA_MAX_DESCRIPTORS;
}

static void i2s_kick(esp32_periph_t *p) {
    if (p->cpu[0]) xtensa_recompute_next_timer(p->cpu[0]);
}

static void i2s_arm_event(esp32_periph_t *p, int port, bool tx) {
    i2s_state_t *s = &p->i2s[port];
    uint32_t desc = tx ? s->tx_desc : s->rx_desc;
    bool active = tx ? s->tx_active : s->rx_active;
    if (!active || !desc || !i2s_dma_range_mapped(p, desc, 4, false)) {
        if (tx) s->tx_event_armed = false;
        else s->rx_event_armed = false;
        return;
    }
    uint32_t ctrl = mem_read32(p->mem, desc);
    size_t len = tx ?
        ((ctrl & I2S_DESC_LENGTH_MASK) >> I2S_DESC_LENGTH_SHIFT) :
        (ctrl & I2S_DESC_SIZE_MASK);
    uint32_t now = p->cpu[0] ? p->cpu[0]->ccount : 0;
    uint32_t next = now + i2s_descriptor_cycles(s, tx, len ? len : 4u);
    if (tx) {
        s->next_tx_ccount = next;
        s->tx_event_armed = true;
    } else {
        s->next_rx_ccount = next;
        s->rx_event_armed = true;
    }
    i2s_kick(p);
}

static void i2s_seed_ring(esp32_periph_t *p, int port, bool tx) {
    i2s_state_t *s = &p->i2s[port];
    uint32_t first = tx ? s->tx_desc : s->rx_desc;
    int count = i2s_ring_descriptor_count(p, first);
    int completions = 1;
    int source = i2s_intr_sources[port];
    if (p->irq_dispatch[source] && count > 1)
        completions = count - 1;
    uint32_t eof_bit = tx ? I2S_INT_OUT_EOF : I2S_INT_IN_SUC_EOF;
    for (int i = 0; i < completions; i++) {
        bool ok = tx ? i2s_process_tx_descriptor(p, port) :
                       i2s_process_rx_descriptor(p, port);
        if (!ok) break;
        /* Native IRQ delivery cannot consume a second completion until the
         * first status bit has been acknowledged by guest code. */
        if (s->int_raw & eof_bit) break;
    }
}

static void i2s_refresh_active(esp32_periph_t *p, int port) {
    i2s_state_t *s = &p->i2s[port];
    uint32_t conf = s->regs[I2S_CONF_OFF / 4u];
    bool dma = (s->regs[I2S_FIFO_CONF_OFF / 4u] & I2S_FIFO_DSCR_EN) != 0;
    bool tx_active = dma && s->tx_link_running &&
                     (conf & I2S_CONF_TX_START) != 0;
    bool rx_active = dma && s->rx_link_running &&
                     (conf & I2S_CONF_RX_START) != 0;

    if (tx_active && !s->tx_active) {
        s->tx_active = true;
        i2s_seed_ring(p, port, true);
        if (s->tx_active) i2s_arm_event(p, port, true);
    } else if (!tx_active) {
        s->tx_active = false;
        s->tx_event_armed = false;
    }
    if (rx_active && !s->rx_active) {
        s->rx_active = true;
        i2s_seed_ring(p, port, false);
        if (s->rx_active) i2s_arm_event(p, port, false);
    } else if (!rx_active) {
        s->rx_active = false;
        s->rx_event_armed = false;
    }
    i2s_kick(p);
}

static uint32_t i2s_next_fire(esp32_periph_t *p, xtensa_cpu_t *cpu) {
    if (!p || !cpu || cpu != p->cpu[0]) return UINT32_MAX;
    bool have = false;
    uint32_t best = UINT32_MAX;
    uint32_t best_distance = 0;
    for (int port = 0; port < I2S_PORT_COUNT; port++) {
        i2s_state_t *s = &p->i2s[port];
        uint32_t events[2] = {s->next_tx_ccount, s->next_rx_ccount};
        bool armed[2] = {s->tx_event_armed, s->rx_event_armed};
        for (int direction = 0; direction < 2; direction++) {
            if (!armed[direction]) continue;
            uint32_t distance = events[direction] - cpu->ccount;
            if ((int32_t)distance < 0) distance = 0;
            if (!have || distance < best_distance) {
                have = true;
                best = events[direction];
                best_distance = distance;
            }
        }
    }
    return have ? best : UINT32_MAX;
}

static void i2s_eval_events(esp32_periph_t *p, xtensa_cpu_t *cpu) {
    if (!p || !cpu || cpu != p->cpu[0]) return;
    for (int port = 0; port < I2S_PORT_COUNT; port++) {
        i2s_state_t *s = &p->i2s[port];
        if (s->tx_event_armed &&
            (int32_t)(cpu->ccount - s->next_tx_ccount) >= 0) {
            s->tx_event_armed = false;
            (void)i2s_process_tx_descriptor(p, port);
            if (s->tx_active) i2s_arm_event(p, port, true);
        }
        if (s->rx_event_armed &&
            (int32_t)(cpu->ccount - s->next_rx_ccount) >= 0) {
            s->rx_event_armed = false;
            (void)i2s_process_rx_descriptor(p, port);
            if (s->rx_active) i2s_arm_event(p, port, false);
        }
    }
}

static uint32_t i2s_read(void *ctx, uint32_t addr) {
    esp32_periph_t *p = ctx;
    int port = i2s_port_from_addr(addr);
    if (port < 0) return default_read(ctx, addr);
    uint32_t off = addr - i2s_base(port);
    if ((off & 3u) != 0 || off >= I2S_REG_FILE_SIZE)
        return default_read(ctx, addr);
    i2s_state_t *s = &p->i2s[port];
    switch (off) {
    case I2S_INT_RAW_OFF: return s->int_raw;
    case I2S_INT_ST_OFF: return s->int_raw & s->int_ena;
    case I2S_INT_ENA_OFF: return s->int_ena;
    case I2S_INT_CLR_OFF: return 0;
    case I2S_OUT_LINK_OFF:
        return (s->regs[off / 4u] & I2S_LINK_ADDR_MASK) |
               (s->tx_link_running ? 0u : I2S_LINK_PARK);
    case I2S_IN_LINK_OFF:
        return (s->regs[off / 4u] & I2S_LINK_ADDR_MASK) |
               (s->rx_link_running ? 0u : I2S_LINK_PARK);
    case I2S_STATE_OFF:
        return (1u << 2) | (1u << 1) | (s->tx_active ? 0u : 1u);
    default:
        return s->regs[off / 4u];
    }
}

static void i2s_write(void *ctx, uint32_t addr, uint32_t val) {
    esp32_periph_t *p = ctx;
    int port = i2s_port_from_addr(addr);
    if (port < 0) { default_write(ctx, addr, val); return; }
    uint32_t off = addr - i2s_base(port);
    if ((off & 3u) != 0 || off >= I2S_REG_FILE_SIZE) {
        default_write(ctx, addr, val);
        return;
    }
    i2s_state_t *s = &p->i2s[port];
    switch (off) {
    case I2S_INT_RAW_OFF:
    case I2S_INT_ST_OFF:
    case I2S_OUT_EOF_DESC_OFF:
    case I2S_IN_EOF_DESC_OFF:
    case I2S_OUT_EOF_BUF_OFF:
    case I2S_INLINK_DSCR_OFF:
    case I2S_INLINK_DSCR_BF0_OFF:
    case I2S_INLINK_DSCR_BF1_OFF:
    case I2S_OUTLINK_DSCR_OFF:
    case I2S_OUTLINK_DSCR_BF0_OFF:
    case I2S_OUTLINK_DSCR_BF1_OFF:
    case I2S_STATE_OFF:
        return; /* read-only */
    case I2S_INT_ENA_OFF:
        s->int_ena = val & I2S_INT_VALID_MASK;
        i2s_irq_update(p, port);
        return;
    case I2S_INT_CLR_OFF:
        s->int_raw &= ~(val & I2S_INT_VALID_MASK);
        i2s_irq_update(p, port);
        return;
    case I2S_OUT_LINK_OFF:
        s->regs[off / 4u] = val & I2S_LINK_ADDR_MASK;
        if (val & I2S_LINK_STOP) {
            s->tx_link_running = false;
        } else if (val & (I2S_LINK_START | I2S_LINK_RESTART)) {
            s->tx_desc = i2s_first_desc(val);
            s->tx_link_running = true;
        }
        i2s_refresh_active(p, port);
        return;
    case I2S_IN_LINK_OFF:
        s->regs[off / 4u] = val & I2S_LINK_ADDR_MASK;
        if (val & I2S_LINK_STOP) {
            s->rx_link_running = false;
        } else if (val & (I2S_LINK_START | I2S_LINK_RESTART)) {
            s->rx_desc = i2s_first_desc(val);
            s->rx_link_running = true;
        }
        i2s_refresh_active(p, port);
        return;
    case I2S_CONF_OFF:
        s->regs[off / 4u] = val;
        if (val & ((1u << 0) | (1u << 2)))
            s->tx_desc = i2s_first_desc(s->regs[I2S_OUT_LINK_OFF / 4u]);
        if (val & ((1u << 1) | (1u << 3)))
            s->rx_desc = i2s_first_desc(s->regs[I2S_IN_LINK_OFF / 4u]);
        i2s_refresh_active(p, port);
        return;
    case I2S_LC_CONF_OFF:
        s->regs[off / 4u] = val;
        if (val & (1u << 1))
            s->tx_desc = i2s_first_desc(s->regs[I2S_OUT_LINK_OFF / 4u]);
        if (val & 1u)
            s->rx_desc = i2s_first_desc(s->regs[I2S_IN_LINK_OFF / 4u]);
        return;
    case I2S_FIFO_CONF_OFF:
        s->regs[off / 4u] = val;
        i2s_refresh_active(p, port);
        return;
    default:
        s->regs[off / 4u] = val;
        return;
    }
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
    for (int port = 0; port < I2C_PORT_COUNT; port++)
        p->i2c[port].regs[I2C_DATE_OFF / 4u] = 0x16042000u;

    /* Both SAR units reset to 12-bit conversion width. RTC DAC pads reset
     * disabled at code zero with their documented drive-strength value. */
    p->sens_regs[SENS_SAR_START_FORCE_OFF / 4u] = 0xFu;
    p->rtcio_regs[RTCIO_DAC1_OFF / 4u] = 2u << 30;
    p->rtcio_regs[RTCIO_DAC2_OFF / 4u] = 2u << 30;

    for (int port = 0; port < I2S_PORT_COUNT; port++) {
        i2s_state_t *i2s = &p->i2s[port];
        i2s->regs[I2S_CONF_OFF / 4u] =
            (1u << 17) | (1u << 16) | (1u << 9) | (1u << 8);
        i2s->regs[I2S_FIFO_CONF_OFF / 4u] =
            I2S_FIFO_DSCR_EN | (32u << 6) | 32u;
        i2s->regs[I2S_RXEOF_NUM_OFF / 4u] = 64u;
        i2s->regs[I2S_LC_CONF_OFF / 4u] = 1u << 8;
        i2s->regs[0x074u / 4u] = (1u << 11) | 0x10u;
        i2s->regs[I2S_CLKM_CONF_OFF / 4u] = 4u;
        i2s->regs[I2S_SAMPLE_RATE_OFF / 4u] =
            (16u << 18) | (16u << 12) | (6u << 6) | 6u;
        i2s->regs[I2S_DATE_OFF / 4u] = 0x01604201u;
    }

    rmt_reset_state(p);

    /* Strapping-pin idle levels: GPIO0/5/15 have pull-ups enabled at reset
     * (GPIO2/12 pull-downs read low). Firmware reads GPIO_IN for buttons
     * tied to these pins (e.g. Marauder's BOOT-button on GPIO0) — leaving
     * them low looks like a permanently held button. */
    p->gpio.in = (1u << 0) | (1u << 5) | (1u << 15);

    /* Initialize interrupt matrix: all lines disabled (source 16 = none) */
    memset(p->intr_matrix, 16, sizeof(p->intr_matrix));

    /* LEDC reset state: all eight timers begin held in reset, and DATE is
     * the ESP32 peripheral version value from the vendor register map. */
    ledc_reset_state(p);

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

    /* Two classic ESP32 I2C controllers (interrupt sources 49/50). */
    mem_register_mmio(mem, (int)PAGE_OF(I2C0_BASE), i2c_read, i2c_write, p);
    mem_register_mmio(mem, (int)PAGE_OF(I2C1_BASE), i2c_read, i2c_write, p);

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

    /* Two classic ESP32 I2S controllers, including circular lldesc DMA. */
    mem_register_mmio(mem, (int)PAGE_OF(I2S0_BASE), i2s_read, i2s_write, p);
    mem_register_mmio(mem, (int)PAGE_OF(I2S1_BASE), i2s_read, i2s_write, p);

    /* Eight-channel RMT register file plus its shared 512-word pulse RAM. */
    mem_register_mmio(mem, (int)PAGE_OF(RMT_BASE), rmt_read, rmt_write, p);

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
    if (!p) return;
    periph_disable_spi_display(p);
    for (int port = 0; port < I2C_PORT_COUNT; port++)
        free(p->i2c[port].pending_write);
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

int periph_i2c_attach_device(esp32_periph_t *p, int port, uint8_t address,
                             periph_i2c_device_fn fn, void *ctx) {
    if (!p || port < 0 || port >= I2C_PORT_COUNT ||
        address >= I2C_DEVICE_COUNT)
        return -1;
    p->i2c[port].device[address].fn = fn;
    p->i2c[port].device[address].ctx = fn ? ctx : NULL;
    return 0;
}

int periph_set_i2s_tx_callback(esp32_periph_t *p, int port,
                               periph_i2s_tx_fn fn, void *ctx) {
    if (!p || port < 0 || port >= I2S_PORT_COUNT) return -1;
    p->i2s[port].tx_cb = fn;
    p->i2s[port].tx_cb_ctx = fn ? ctx : NULL;
    return 0;
}

size_t periph_i2s_rx_inject(esp32_periph_t *p, int port,
                            const uint8_t *data, size_t len) {
    if (!p || port < 0 || port >= I2S_PORT_COUNT || (!data && len != 0))
        return 0;
    return i2s_rx_fifo_push(&p->i2s[port], data, len);
}

size_t periph_i2s_rx_pending(const esp32_periph_t *p, int port) {
    if (!p || port < 0 || port >= I2S_PORT_COUNT) return 0;
    return p->i2s[port].rx_len;
}

int periph_set_rmt_tx_callback(esp32_periph_t *p, int channel,
                               periph_rmt_tx_fn fn, void *ctx) {
    if (!p || channel < 0 || channel >= (int)RMT_CHANNEL_COUNT) return -1;
    p->rmt.channel[channel].tx_cb = fn;
    p->rmt.channel[channel].tx_cb_ctx = fn ? ctx : NULL;
    return 0;
}

int periph_set_ledc_output_callback(esp32_periph_t *p, int speed_mode,
                                    int channel, periph_ledc_output_fn fn,
                                    void *ctx) {
    if (!p || speed_mode < 0 ||
        speed_mode >= (int)LEDC_SPEED_MODE_COUNT || channel < 0 ||
        channel >= (int)LEDC_CHANNEL_COUNT)
        return -1;
    ledc_channel_state_t *state = &p->ledc.channel[speed_mode][channel];
    state->output_cb = fn;
    state->output_cb_ctx = fn ? ctx : NULL;
    state->output_reported = false;
    if (fn)
        ledc_emit_channel(p, (unsigned)speed_mode, (unsigned)channel, true);
    return 0;
}

size_t periph_rmt_rx_inject(esp32_periph_t *p, int channel_index,
                            const uint32_t *items, size_t count) {
    if (!p || channel_index < 0 ||
        channel_index >= (int)RMT_CHANNEL_COUNT || (!items && count != 0))
        return 0;

    unsigned index = (unsigned)channel_index;
    rmt_channel_state_t *channel = &p->rmt.channel[index];
    if (!channel->rx_active || !(channel->conf1 & RMT_CONF1_RX_EN) ||
        !(channel->conf1 & RMT_CONF1_MEM_OWNER_RX))
        return 0;

    size_t capacity = rmt_channel_capacity(&p->rmt, index);
    size_t available = channel->rx_index < capacity ?
        capacity - channel->rx_index : 0;
    size_t accepted = count < available ? count : available;
    size_t base = index * RMT_MEM_WORDS_PER_CHANNEL;
    for (size_t i = 0; i < accepted; i++)
        p->rmt.memory[base + channel->rx_index + i] = items[i];
    channel->rx_index += (uint16_t)accepted;

    if (accepted < count) {
        channel->status_flags |= RMT_STATUS_MEM_FULL;
        p->rmt.int_raw |= RMT_ERROR_INT(index);
    }
    /* Host injection represents a complete pulse train followed by the
     * configured idle gap, so RX_END becomes visible even for an empty train. */
    p->rmt.int_raw |= RMT_RX_END_INT(index);
    rmt_irq_update(p);
    return accepted;
}

int periph_set_irq_dispatch(esp32_periph_t *p, int source,
                            periph_irq_dispatch_fn fn, void *ctx) {
    if (!p || source < 0 || source >= 71)
        return -1;
    p->irq_dispatch[source] = fn;
    p->irq_dispatch_ctx[source] = fn ? ctx : NULL;
    return 0;
}

bool periph_interrupt_pending(const esp32_periph_t *p, int source) {
    if (!p || source < 0 || source >= 71) return false;
    return (p->pending_sources[source / 32] &
            (1u << (source % 32))) != 0;
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

int periph_dac_enabled(const esp32_periph_t *p, int channel) {
    if (!p || channel < 0 || channel > 1) return -1;
    uint32_t off = channel == 0 ? RTCIO_DAC1_OFF : RTCIO_DAC2_OFF;
    return rtcio_dac_is_enabled(p->rtcio_regs[off / 4u]) ? 1 : 0;
}

uint8_t periph_dac_value(const esp32_periph_t *p, int channel) {
    if (!p || channel < 0 || channel > 1) return 0;
    uint32_t off = channel == 0 ? RTCIO_DAC1_OFF : RTCIO_DAC2_OFF;
    return rtcio_dac_value(p->rtcio_regs[off / 4u]);
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
