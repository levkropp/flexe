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
#define PCNT_BASE       0x3FF57000u
#define MCPWM0_BASE     0x3FF5E000u
#define MCPWM1_BASE     0x3FF6C000u
#define GPIO_BASE       0x3FF44000u
#define FE2_BASE        0x3FF45000u
#define FE_BASE         0x3FF46000u
#define FRC_TIMER_BASE  0x3FF47000u
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

/* Classic ESP32 pulse counter register file and shared interrupt source. */
#define PCNT_UNIT_COUNT          8u
#define PCNT_CHANNEL_COUNT       2u
#define PCNT_UNIT_CONF_STRIDE    0x0Cu
#define PCNT_CNT_OFF             0x060u
#define PCNT_INT_RAW_OFF         0x080u
#define PCNT_INT_ST_OFF          0x084u
#define PCNT_INT_ENA_OFF         0x088u
#define PCNT_INT_CLR_OFF         0x08Cu
#define PCNT_STATUS_OFF          0x090u
#define PCNT_CTRL_OFF            0x0B0u
#define PCNT_DATE_OFF            0x0FCu
#define PCNT_INTR_SOURCE         48
#define PCNT_INT_VALID_MASK      0xFFu
#define PCNT_CONF_FILTER_MASK    0x3FFu
#define PCNT_CONF_FILTER_EN      (1u << 10)
#define PCNT_EVT_ZERO            (1u << 6)
#define PCNT_EVT_H_LIM           (1u << 5)
#define PCNT_EVT_L_LIM           (1u << 4)
#define PCNT_EVT_THRES0          (1u << 3)
#define PCNT_EVT_THRES1          (1u << 2)

/* Classic ESP32 motor-control PWM register file. Both units have three
 * timers, three operators, two generators per operator, and a shared
 * capture/fault/sync/interrupt block. */
#define MCPWM_UNIT_COUNT             2u
#define MCPWM_TIMER_COUNT            3u
#define MCPWM_OPERATOR_COUNT         3u
#define MCPWM_GENERATOR_COUNT        2u
#define MCPWM_REG_FILE_SIZE          0x128u
#define MCPWM_TIMER_BASE_OFF         0x004u
#define MCPWM_TIMER_STRIDE           0x010u
#define MCPWM_TIMER_CFG0_REL         0x000u
#define MCPWM_TIMER_CFG1_REL         0x004u
#define MCPWM_TIMER_SYNC_REL         0x008u
#define MCPWM_TIMER_STATUS_REL       0x00Cu
#define MCPWM_TIMER_SYNCI_CFG_OFF    0x034u
#define MCPWM_OPERATOR_TIMERSEL_OFF  0x038u
#define MCPWM_OPERATOR_BASE_OFF      0x03Cu
#define MCPWM_OPERATOR_STRIDE        0x038u
#define MCPWM_GEN_STMP_CFG_REL       0x000u
#define MCPWM_GEN_TSTMP_A_REL        0x004u
#define MCPWM_GEN_TSTMP_B_REL        0x008u
#define MCPWM_GEN_CFG0_REL           0x00Cu
#define MCPWM_GEN_FORCE_REL          0x010u
#define MCPWM_GEN_A_REL              0x014u
#define MCPWM_GEN_B_REL              0x018u
#define MCPWM_DT_CFG_REL             0x01Cu
#define MCPWM_DT_FED_REL             0x020u
#define MCPWM_DT_RED_REL             0x024u
#define MCPWM_CARRIER_REL            0x028u
#define MCPWM_FH_CFG0_REL            0x02Cu
#define MCPWM_FH_CFG1_REL            0x030u
#define MCPWM_FH_STATUS_REL          0x034u
#define MCPWM_FAULT_DETECT_OFF       0x0E4u
#define MCPWM_CAP_TIMER_CFG_OFF      0x0E8u
#define MCPWM_CAP_TIMER_PHASE_OFF    0x0ECu
#define MCPWM_CAP_CH_CFG_OFF         0x0F0u
#define MCPWM_CAP_CH_VALUE_OFF       0x0FCu
#define MCPWM_CAP_STATUS_OFF         0x108u
#define MCPWM_UPDATE_CFG_OFF         0x10Cu
#define MCPWM_INT_ENA_OFF            0x110u
#define MCPWM_INT_RAW_OFF            0x114u
#define MCPWM_INT_ST_OFF             0x118u
#define MCPWM_INT_CLR_OFF            0x11Cu
#define MCPWM_CLK_OFF                0x120u
#define MCPWM_VERSION_OFF            0x124u
#define MCPWM_INT_VALID_MASK         0x3FFFFFFFu
#define MCPWM_VERSION_RESET          0x02107230u
#define MCPWM_TIMER_PERIOD_RESET     0x0000FF00u
#define MCPWM_GEN_FORCE_RESET        0x00000020u
#define MCPWM_DT_CFG_RESET           0x00018000u
#define MCPWM_UPDATE_CFG_RESET       0x00000055u
#define MCPWM_SOURCE_CLOCK_MHZ       160u
#define MCPWM_CAPTURE_CLOCK_MHZ      80u
#define MCPWM_UPDATE_EVENT_TEZ       (1u << 0)
#define MCPWM_UPDATE_EVENT_TEP       (1u << 1)
#define MCPWM_UPDATE_EVENT_SYNC      (1u << 2)
#define MCPWM_UPDATE_EVENT_TEA       (1u << 3)
#define MCPWM_UPDATE_EVENT_TEB       (1u << 4)

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
static uint32_t pcnt_next_fire(esp32_periph_t *p, xtensa_cpu_t *cpu);
static void pcnt_eval_events(esp32_periph_t *p, xtensa_cpu_t *cpu);
static void pcnt_reset_state(esp32_periph_t *p);
static void pcnt_gpio_route_changed(esp32_periph_t *p, unsigned signal);
static void pcnt_gpio_input_changed(esp32_periph_t *p, int gpio);
static uint32_t mcpwm_next_fire(esp32_periph_t *p, xtensa_cpu_t *cpu);
static void mcpwm_eval_events(esp32_periph_t *p, xtensa_cpu_t *cpu);
static void mcpwm_reset_unit(esp32_periph_t *p, unsigned unit);
static uint64_t timg_now_cycles(esp32_periph_t *p);
static void timg_sync_all_to(esp32_periph_t *p, uint64_t now);
static uint32_t timg_next_fire(esp32_periph_t *p, xtensa_cpu_t *cpu);
static void timg_eval_events(esp32_periph_t *p, xtensa_cpu_t *cpu);
static void timg_reset_group(esp32_periph_t *p, unsigned group);
static void timg_kick(esp32_periph_t *p);
static void mcpwm_gpio_output_route_changed(esp32_periph_t *p, int gpio,
                                             uint32_t before,
                                             uint32_t after);
static void mcpwm_gpio_input_route_changed(esp32_periph_t *p,
                                            unsigned signal);
static void mcpwm_gpio_input_changed(esp32_periph_t *p, int gpio);

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

/* Each classic ESP32 timer group contains two independent 64-bit general
 * purpose timers clocked from the 80 MHz APB domain.  The exposed LO/HI
 * registers are software-captured snapshots; counter is the live value. */
typedef struct {
    uint32_t config;
    uint64_t counter;
    uint64_t latched;
    uint64_t alarm;
    uint64_t load;
    uint64_t last_cycles;
    uint64_t tick_remainder;
} timg_timer_state_t;

typedef struct {
    timg_timer_state_t timer[2];
    uint32_t int_ena;
    uint32_t int_raw;
    uint32_t date;
    uint32_t regclk;
} timg_group_state_t;

/* A shared monotonic CPU-cycle timeline prevents sequential execution of the
 * two emulated cores from advancing APB peripherals twice. */
typedef struct {
    uint64_t cycles;
    uint64_t core_cycles[2];
    uint32_t last_ccount[2];
    bool valid[2];
} timg_clock_state_t;

/* The legacy FRC block predates the Timer Groups. FRC1 is a 23-bit
 * down-counter; FRC2 is a 32-bit up-counter with a programmable compare. */
typedef struct {
    uint32_t load;
    uint32_t counter;
    uint32_t config;
    uint32_t alarm;
    uint64_t last_cycles;
    uint64_t tick_remainder;
    bool int_status;
} frc_timer_state_t;

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

typedef struct {
    bool pending;
    bool level;
    uint32_t deadline;
} pcnt_filter_state_t;

typedef struct {
    int16_t count;
    uint32_t status;
    bool pulse_level[PCNT_CHANNEL_COUNT];
    bool control_level[PCNT_CHANNEL_COUNT];
    pcnt_filter_state_t pulse_filter[PCNT_CHANNEL_COUNT];
    pcnt_filter_state_t control_filter[PCNT_CHANNEL_COUNT];
} pcnt_unit_state_t;

typedef struct {
    uint32_t regs[0x100 / sizeof(uint32_t)];
    pcnt_unit_state_t unit[PCNT_UNIT_COUNT];
    uint32_t pending_filters;
} pcnt_state_t;

typedef struct {
    /* Phase advances monotonically around the timer's mode-dependent cycle:
     * up/down use period+1 phases, while symmetric mode uses 2*period. */
    uint32_t phase;
    uint32_t active_period;
    uint64_t last_cycles;
    uint64_t tick_remainder;
    uint8_t active_prescale;
    uint8_t mode;
    bool running;
    bool stop_at_tez;
    bool stop_at_tep;
    bool period_pending;
} mcpwm_timer_state_t;

typedef struct {
    uint16_t active_compare[MCPWM_GENERATOR_COUNT];
    bool compare_pending[MCPWM_GENERATOR_COUNT];
    uint32_t active_generator[MCPWM_GENERATOR_COUNT];
    bool generator_pending[MCPWM_GENERATOR_COUNT];
    uint16_t active_fed;
    uint16_t active_red;
    bool fed_pending;
    bool red_pending;
    uint8_t active_force[MCPWM_GENERATOR_COUNT];
    bool force_pending;
    bool generator_level[MCPWM_GENERATOR_COUNT];
    bool cbc_on;
    bool ost_on;
    bool cbc_override_valid[MCPWM_GENERATOR_COUNT];
    bool cbc_override_level[MCPWM_GENERATOR_COUNT];
    bool ost_override_valid[MCPWM_GENERATOR_COUNT];
    bool ost_override_level[MCPWM_GENERATOR_COUNT];

    periph_mcpwm_output_fn output_cb[MCPWM_GENERATOR_COUNT];
    void *output_cb_ctx[MCPWM_GENERATOR_COUNT];
    periph_mcpwm_output_info_t last_info[MCPWM_GENERATOR_COUNT];
    bool output_reported[MCPWM_GENERATOR_COUNT];
} mcpwm_operator_state_t;

typedef struct {
    uint32_t regs[MCPWM_REG_FILE_SIZE / sizeof(uint32_t)];
    mcpwm_timer_state_t timer[MCPWM_TIMER_COUNT];
    mcpwm_operator_state_t operators[MCPWM_OPERATOR_COUNT];
    uint32_t capture_counter;
    uint64_t capture_last_cycles;
    uint64_t capture_remainder;
    uint16_t capture_prescale_count[MCPWM_TIMER_COUNT];
    bool sync_level[MCPWM_TIMER_COUNT];
    bool fault_level[MCPWM_TIMER_COUNT];
    bool capture_level[MCPWM_TIMER_COUNT];
} mcpwm_unit_state_t;

typedef struct {
    mcpwm_unit_state_t unit[MCPWM_UNIT_COUNT];
    uint64_t time_cycles;
    uint64_t core_cycles[2];
    uint32_t last_ccount[2];
    bool time_valid[2];
} mcpwm_state_t;

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

    /* Timer groups: two APB general-purpose timers plus WDT/LACT functions. */
    timg_group_state_t timg[2];
    timg_clock_state_t timg_clock;
    bool timg_alarm_active;

    /* Legacy APB timers: FRC1 countdown and FRC2 count-up/compare. */
    frc_timer_state_t frc_timer[2];
    bool frc_event_active;

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

    /* Eight two-channel classic ESP32 pulse-counter units. */
    pcnt_state_t pcnt;

    /* Two classic ESP32 motor-control PWM units. */
    mcpwm_state_t mcpwm;

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
#define DPORT_PCNT_MODULE_BIT          (1u << 10)
#define DPORT_LEDC_MODULE_BIT          (1u << 11)
#define DPORT_TIMG0_MODULE_BIT         (1u << 13)
#define DPORT_TIMG1_MODULE_BIT         (1u << 15)
#define DPORT_PWM0_MODULE_BIT          (1u << 17)
#define DPORT_PWM1_MODULE_BIT          (1u << 20)

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
        timg_sync_all_to(p, timg_now_cycles(p));
        p->dport_perip_clk_en = val;
        timg_kick(p);
        break;
    case DPORT_PERIP_RST_EN_OFF:
        timg_sync_all_to(p, timg_now_cycles(p));
        p->dport_perip_rst_en = val;
        if (val & DPORT_RMT_MODULE_BIT)
            rmt_reset_state(p);
        if (val & DPORT_PCNT_MODULE_BIT)
            pcnt_reset_state(p);
        if (val & DPORT_LEDC_MODULE_BIT)
            ledc_reset_state(p);
        if (val & DPORT_TIMG0_MODULE_BIT)
            timg_reset_group(p, 0);
        if (val & DPORT_TIMG1_MODULE_BIT)
            timg_reset_group(p, 1);
        if (val & DPORT_PWM0_MODULE_BIT)
            mcpwm_reset_unit(p, 0);
        if (val & DPORT_PWM1_MODULE_BIT)
            mcpwm_reset_unit(p, 1);
        timg_kick(p);
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
        pcnt_gpio_route_changed(p, (unsigned)sig);
        mcpwm_gpio_input_route_changed(p, (unsigned)sig);
        return;
    }

    /* GPIO_FUNC_OUT_SEL_CFG_REG */
    if (off >= 0x530 && off < 0x530 + 40 * 4) {
        int n = (int)(off - 0x530) / 4;
        uint32_t before = p->gpio.func_out_sel[n];
        p->gpio.func_out_sel[n] = val;
        ledc_gpio_route_changed(p, n, before, val);
        mcpwm_gpio_output_route_changed(p, n, before, val);
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

/* ---- TIMG general-purpose timers and LACT timebase ---- */

#define TIMG_TIMER_COUNT          2u
#define TIMG_APB_CLOCK_MHZ        80u
#define TIMG_TIMER_STRIDE         0x24u
#define TIMG_TIMER_CONFIG_RESET   0x60002000u
#define TIMG_TIMER_CONFIG_MASK    0xFFFFFC00u
#define TIMG_TIMER_EN             (1u << 31)
#define TIMG_TIMER_INCREASE       (1u << 30)
#define TIMG_TIMER_AUTORELOAD     (1u << 29)
#define TIMG_TIMER_EDGE_INT_EN    (1u << 12)
#define TIMG_TIMER_LEVEL_INT_EN   (1u << 11)
#define TIMG_TIMER_ALARM_EN       (1u << 10)
#define TIMG_INT_VALID_MASK       0xFu
#define TIMG_WDT_INT_BIT          (1u << 2)
#define LACT_CFG_EN               (1u << 31) /* TIMG_LACT_EN */
#define LACT_CFG_EDGE_INT_EN      (1u << 12)
#define LACT_CFG_LEVEL_INT_EN     (1u << 11)
#define LACT_CFG_ALARM_EN         (1u << 10) /* TIMG_LACT_ALARM_EN */
#define LACT_INT_BIT              (1u << 3)  /* INT_ENA/RAW/ST/CLR */

static int timg_timer_level_source(unsigned group, unsigned timer) {
    return group == 0u ? 14 + (int)timer : 18 + (int)timer;
}

static int timg_timer_edge_source(unsigned group, unsigned timer) {
    return group == 0u ? 58 + (int)timer : 62 + (int)timer;
}

static int timg_wdt_level_source(unsigned group) {
    return group == 0u ? 16 : 20;
}

static int timg_wdt_edge_source(unsigned group) {
    return group == 0u ? 60 : 64;
}

static int timg_lact_level_source(unsigned group) {
    return group == 0u ? 17 : 21;
}

static int timg_lact_edge_source(unsigned group) {
    return group == 0u ? 61 : 65;
}

static uint32_t timg_cpu_mhz(const esp32_periph_t *p) {
    uint32_t mhz = mem_read32(p->mem, ESP32_CPU_TICKS_PER_US_ADDR);
    return mhz >= 10u && mhz <= 240u ? mhz : 240u;
}

static uint64_t timg_now_cycles(esp32_periph_t *p) {
    for (unsigned core = 0; core < 2u; core++) {
        if (!p->cpu[core]) continue;
        uint32_t now = p->cpu[core]->ccount;
        if (!p->timg_clock.valid[core]) {
            p->timg_clock.last_ccount[core] = now;
            p->timg_clock.core_cycles[core] = p->timg_clock.cycles;
            p->timg_clock.valid[core] = true;
            continue;
        }
        uint32_t elapsed = now - p->timg_clock.last_ccount[core];
        if (elapsed < (uint32_t)INT32_MAX) {
            if (p->timg_clock.core_cycles[core] > UINT64_MAX - elapsed)
                p->timg_clock.core_cycles[core] = UINT64_MAX;
            else
                p->timg_clock.core_cycles[core] += elapsed;
        }
        p->timg_clock.last_ccount[core] = now;
        if (p->timg_clock.core_cycles[core] > p->timg_clock.cycles)
            p->timg_clock.cycles = p->timg_clock.core_cycles[core];
    }
    return p->timg_clock.cycles;
}

static bool timg_group_clocked(const esp32_periph_t *p, unsigned group) {
    uint32_t bit = group == 0u ? DPORT_TIMG0_MODULE_BIT :
                                 DPORT_TIMG1_MODULE_BIT;
    return (p->dport_perip_clk_en & bit) != 0u &&
           (p->dport_perip_rst_en & bit) == 0u;
}

static uint32_t timg_timer_divider(const timg_timer_state_t *timer) {
    uint32_t divider = (timer->config >> 13) & 0xFFFFu;
    return divider != 0u ? divider : 65536u;
}

/* Interrupt-matrix updates scan both cores and can synchronously dispatch a
 * compatibility ISR.  Only touch a source when its electrical level changes;
 * timed peripherals call these helpers on every shared event-hook pass. */
static void timg_drive_source(esp32_periph_t *p, int source, bool active) {
    if (p->source_level[source] == active) return;
    if (active)
        periph_assert_interrupt(p, source);
    else
        periph_deassert_interrupt(p, source);
}

static void timg_update_timer_irq(esp32_periph_t *p, unsigned group,
                                  unsigned timer) {
    timg_group_state_t *state = &p->timg[group];
    uint32_t bit = 1u << timer;
    uint32_t active = state->int_raw & state->int_ena;
    uint32_t config = state->timer[timer].config;
    timg_drive_source(p, timg_timer_level_source(group, timer),
                      (active & bit) != 0u &&
                      (config & TIMG_TIMER_LEVEL_INT_EN) != 0u);

    /* A compatibility ISR may synchronously clear RAW or alter CONFIG while
     * the first source is asserted, so refresh before driving the other. */
    active = state->int_raw & state->int_ena;
    config = state->timer[timer].config;
    timg_drive_source(p, timg_timer_edge_source(group, timer),
                      (active & bit) != 0u &&
                      (config & TIMG_TIMER_EDGE_INT_EN) != 0u);
}

static void timg_update_wdt_irq(esp32_periph_t *p, unsigned group) {
    timg_group_state_t *state = &p->timg[group];
    uint32_t active = state->int_raw & state->int_ena;
    uint32_t wdt_config = p->timg_wdt[group].config0;
    timg_drive_source(p, timg_wdt_level_source(group),
                      (active & TIMG_WDT_INT_BIT) != 0u &&
                      (wdt_config & (1u << 21)) != 0u);
    active = state->int_raw & state->int_ena;
    wdt_config = p->timg_wdt[group].config0;
    timg_drive_source(p, timg_wdt_edge_source(group),
                      (active & TIMG_WDT_INT_BIT) != 0u &&
                      (wdt_config & (1u << 22)) != 0u);
}

static void timg_update_lact_irq(esp32_periph_t *p, unsigned group) {
    timg_group_state_t *state = &p->timg[group];
    lact_state_t *lact = &p->lact[group];
    uint32_t active = state->int_raw & state->int_ena;
    bool lact_level = (active & LACT_INT_BIT) != 0u &&
                      (lact->config & LACT_CFG_LEVEL_INT_EN) != 0u;
    lact->level = lact_level;
    timg_drive_source(p, timg_lact_level_source(group), lact_level);
    active = state->int_raw & state->int_ena;
    timg_drive_source(p, timg_lact_edge_source(group),
                      (active & LACT_INT_BIT) != 0u &&
                      (lact->config & LACT_CFG_EDGE_INT_EN) != 0u);
}

static void timg_update_group_irqs(esp32_periph_t *p, unsigned group) {
    for (unsigned timer = 0; timer < TIMG_TIMER_COUNT; timer++)
        timg_update_timer_irq(p, group, timer);
    timg_update_wdt_irq(p, group);
    timg_update_lact_irq(p, group);
}

static void timg_refresh_alarm_active(esp32_periph_t *p) {
    p->timg_alarm_active = false;
    for (unsigned group = 0; group < 2u; group++) {
        if (!timg_group_clocked(p, group)) continue;
        timg_group_state_t *state = &p->timg[group];
        for (unsigned timer = 0; timer < TIMG_TIMER_COUNT; timer++) {
            uint32_t config = state->timer[timer].config;
            uint32_t bit = 1u << timer;
            if ((config & TIMG_TIMER_EN) != 0u &&
                (config & TIMG_TIMER_ALARM_EN) != 0u &&
                (config & (TIMG_TIMER_LEVEL_INT_EN |
                           TIMG_TIMER_EDGE_INT_EN)) != 0u &&
                (state->int_ena & bit) != 0u) {
                p->timg_alarm_active = true;
                return;
            }
        }
    }
}

static void timg_fire_alarm(esp32_periph_t *p, unsigned group,
                            unsigned timer_index) {
    timg_group_state_t *state = &p->timg[group];
    timg_timer_state_t *timer = &state->timer[timer_index];
    bool autoreload = (timer->config & TIMG_TIMER_AUTORELOAD) != 0u;
    timer->config &= ~TIMG_TIMER_ALARM_EN;
    if (autoreload) {
        timer->counter = timer->load;
    }
    state->int_raw |= 1u << timer_index;
    /* Compatibility-mode interrupt dispatch is synchronous.  The genuine
     * ESP-IDF ISR can clear RAW and re-arm the alarm before this returns. */
    timg_update_timer_irq(p, group, timer_index);
    timg_refresh_alarm_active(p);
}

static void timg_advance_timer_ticks(esp32_periph_t *p, unsigned group,
                                     unsigned timer_index, uint64_t ticks) {
    timg_timer_state_t *timer = &p->timg[group].timer[timer_index];
    unsigned events = 0u;
    while (ticks != 0u) {
        bool increase = (timer->config & TIMG_TIMER_INCREASE) != 0u;
        bool alarm_enabled =
            (timer->config & TIMG_TIMER_ALARM_EN) != 0u;
        uint64_t distance = increase ? timer->alarm - timer->counter :
                                       timer->counter - timer->alarm;
        /* Equality at the start is not a new edge; the alarm fires when a
         * subsequent timer tick reaches the compare value.  A full 2^64
         * wrap cannot fit in the finite tick count represented here. */
        if (!alarm_enabled || distance == 0u || ticks < distance) {
            timer->counter = increase ? timer->counter + ticks :
                                        timer->counter - ticks;
            break;
        }

        timer->counter = timer->alarm;
        ticks -= distance;
        timg_fire_alarm(p, group, timer_index);
        if (++events > 1000000u) {
            bool now_increase =
                (timer->config & TIMG_TIMER_INCREASE) != 0u;
            timer->counter = now_increase ? timer->counter + ticks :
                                            timer->counter - ticks;
            break;
        }
        if (!(timer->config & TIMG_TIMER_EN) ||
            !timg_group_clocked(p, group))
            break;
    }
}

static void timg_sync_timer_to(esp32_periph_t *p, unsigned group,
                               unsigned timer_index, uint64_t now) {
    timg_timer_state_t *timer = &p->timg[group].timer[timer_index];
    uint64_t elapsed = now >= timer->last_cycles ?
                       now - timer->last_cycles : 0u;
    timer->last_cycles = now;
    if (!(timer->config & TIMG_TIMER_EN) ||
        !timg_group_clocked(p, group))
        return;

    uint64_t denominator =
        (uint64_t)timg_cpu_mhz(p) * timg_timer_divider(timer);
    if (denominator == 0u) return;
    uint64_t product = elapsed >
        (UINT64_MAX - timer->tick_remainder) / TIMG_APB_CLOCK_MHZ ?
        UINT64_MAX : timer->tick_remainder + elapsed * TIMG_APB_CLOCK_MHZ;
    uint64_t ticks = product / denominator;
    timer->tick_remainder = product % denominator;
    if (ticks != 0u)
        timg_advance_timer_ticks(p, group, timer_index, ticks);
}

static void timg_sync_group_to(esp32_periph_t *p, unsigned group,
                               uint64_t now) {
    for (unsigned timer = 0; timer < TIMG_TIMER_COUNT; timer++)
        timg_sync_timer_to(p, group, timer, now);
}

static void timg_sync_all_to(esp32_periph_t *p, uint64_t now) {
    for (unsigned group = 0; group < 2u; group++)
        timg_sync_group_to(p, group, now);
}

static uint64_t timg_cycles_until_ticks(const esp32_periph_t *p,
                                        const timg_timer_state_t *timer,
                                        uint64_t ticks) {
    uint64_t denominator =
        (uint64_t)timg_cpu_mhz(p) * timg_timer_divider(timer);
    uint64_t needed = ticks > UINT64_MAX / denominator ? UINT64_MAX :
                      ticks * denominator;
    if (needed <= timer->tick_remainder) return 1u;
    needed -= timer->tick_remainder;
    return needed / TIMG_APB_CLOCK_MHZ +
           (needed % TIMG_APB_CLOCK_MHZ != 0u);
}

static uint32_t timg_next_fire(esp32_periph_t *p, xtensa_cpu_t *cpu) {
    if (!p || !cpu || !p->timg_alarm_active) return UINT32_MAX;
    uint64_t now = timg_now_cycles(p);
    timg_sync_all_to(p, now);
    bool have = false;
    uint64_t best_distance = 0u;
    for (unsigned group = 0; group < 2u; group++) {
        if (!timg_group_clocked(p, group)) continue;
        timg_group_state_t *state = &p->timg[group];
        for (unsigned timer_index = 0; timer_index < TIMG_TIMER_COUNT;
             timer_index++) {
            timg_timer_state_t *timer = &state->timer[timer_index];
            uint32_t bit = 1u << timer_index;
            if (!(timer->config & TIMG_TIMER_EN) ||
                !(timer->config & TIMG_TIMER_ALARM_EN) ||
                !(timer->config & (TIMG_TIMER_LEVEL_INT_EN |
                                    TIMG_TIMER_EDGE_INT_EN)) ||
                !(state->int_ena & bit))
                continue;
            bool increase =
                (timer->config & TIMG_TIMER_INCREASE) != 0u;
            uint64_t ticks = increase ? timer->alarm - timer->counter :
                                        timer->counter - timer->alarm;
            if (ticks == 0u) continue;
            uint64_t distance = timg_cycles_until_ticks(p, timer, ticks);
            if (!have || distance < best_distance) {
                have = true;
                best_distance = distance;
            }
        }
    }
    if (!have) return UINT32_MAX;
    if (best_distance >= (uint64_t)INT32_MAX)
        best_distance = (uint64_t)INT32_MAX - 1u;
    return cpu->ccount + (uint32_t)best_distance;
}

static void timg_eval_events(esp32_periph_t *p, xtensa_cpu_t *cpu) {
    (void)cpu;
    if (!p->timg_alarm_active) return;
    uint64_t now = timg_now_cycles(p);
    timg_sync_all_to(p, now);
}

static void timg_kick(esp32_periph_t *p) {
    timg_refresh_alarm_active(p);
    for (unsigned core = 0; core < 2u; core++)
        if (p->cpu[core]) xtensa_recompute_next_timer(p->cpu[core]);
}

static void timg_reset_group(esp32_periph_t *p, unsigned group) {
    if (!p || group >= 2u) return;
    for (unsigned timer = 0; timer < TIMG_TIMER_COUNT; timer++) {
        periph_deassert_interrupt(p, timg_timer_level_source(group, timer));
        periph_deassert_interrupt(p, timg_timer_edge_source(group, timer));
    }
    periph_deassert_interrupt(p, timg_wdt_level_source(group));
    periph_deassert_interrupt(p, timg_wdt_edge_source(group));
    periph_deassert_interrupt(p, timg_lact_level_source(group));
    periph_deassert_interrupt(p, timg_lact_edge_source(group));

    memset(&p->timg[group], 0, sizeof(p->timg[group]));
    memset(&p->timg_wdt[group], 0, sizeof(p->timg_wdt[group]));
    memset(&p->lact[group], 0, sizeof(p->lact[group]));
    memset(&p->rtc_cal[group], 0, sizeof(p->rtc_cal[group]));
    p->timg[group].date = 0x01604290u;
    p->lact[group].config = 0x60002300u;
    for (unsigned timer = 0; timer < TIMG_TIMER_COUNT; timer++) {
        p->timg[group].timer[timer].config = TIMG_TIMER_CONFIG_RESET;
        p->timg[group].timer[timer].last_cycles = p->timg_clock.cycles;
    }
}

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
static void lact_latch_alarm(esp32_periph_t *p, int group) {
    lact_state_t *l = &p->lact[group];
    if ((l->config & LACT_CFG_ALARM_EN) && lact_counter(p, group) >= l->alarm)
        p->timg[group].int_raw |= LACT_INT_BIT;
}

static void lact_eval_irq(esp32_periph_t *p, int group) {
    lact_latch_alarm(p, group);
    timg_update_lact_irq(p, (unsigned)group);
}

/* Next cpu ccount at which a LACT alarm will fire (for next_timer_event). */
static uint32_t lact_next_fire(esp32_periph_t *p, xtensa_cpu_t *cpu) {
    uint64_t best = UINT32_MAX;
    for (int group = 0; group < 2; group++) {
        lact_state_t *l = &p->lact[group];
        if (!(l->config & LACT_CFG_ALARM_EN)) continue;
        if (p->timg[group].int_raw & LACT_INT_BIT) {
            continue; /* event already latched */
        }
        uint64_t now = lact_counter(p, group);
        if (now >= l->alarm) {
            return cpu->ccount;   /* fire now */
        }
        uint64_t cycles = (l->alarm - now) * lact_divider(l);
        uint64_t event = (uint64_t)cpu->ccount + cycles;
        if (event > UINT32_MAX) event = UINT32_MAX;
        if (event < best) best = event;
    }
    return (uint32_t)best;
}

/* ---- FRC1/FRC2 legacy APB timers ---- */

#define FRC_TIMER_COUNT          2u
#define FRC_TIMER_STRIDE         0x20u
#define FRC_CTRL_LEVEL_INT       (1u << 0)
#define FRC_CTRL_PRESCALER_MASK  (7u << 1)
#define FRC_CTRL_AUTOLOAD        (1u << 6)
#define FRC_CTRL_ENABLE          (1u << 7)
#define FRC_CTRL_INT_STATUS      (1u << 8)
#define FRC_CTRL_CONFIG_MASK     (FRC_CTRL_LEVEL_INT | \
                                  FRC_CTRL_PRESCALER_MASK | \
                                  FRC_CTRL_AUTOLOAD | FRC_CTRL_ENABLE)
#define FRC1_COUNT_MASK          0x007FFFFFu

static uint32_t frc_count_mask(unsigned timer_index) {
    return timer_index == 0u ? FRC1_COUNT_MASK : UINT32_MAX;
}

static uint32_t frc_prescaler(const frc_timer_state_t *timer) {
    switch ((timer->config & FRC_CTRL_PRESCALER_MASK) >> 1) {
    case 2u: return 16u;
    case 4u: return 256u;
    default: return 1u;
    }
}

static void frc_update_irq(esp32_periph_t *p, unsigned timer_index) {
    timg_drive_source(p, 56 + (int)timer_index,
                      p->frc_timer[timer_index].int_status);
}

static void frc_refresh_event_active(esp32_periph_t *p) {
    p->frc_event_active = false;
    for (unsigned timer = 0; timer < FRC_TIMER_COUNT; timer++) {
        if ((p->frc_timer[timer].config & FRC_CTRL_ENABLE) != 0u &&
            !p->frc_timer[timer].int_status) {
            p->frc_event_active = true;
            return;
        }
    }
}

static void frc_kick(esp32_periph_t *p) {
    frc_refresh_event_active(p);
    for (unsigned core = 0; core < 2u; core++)
        if (p->cpu[core]) xtensa_recompute_next_timer(p->cpu[core]);
}

static void frc_fire(esp32_periph_t *p, unsigned timer_index) {
    frc_timer_state_t *timer = &p->frc_timer[timer_index];
    if (timer_index == 0u &&
        (timer->config & FRC_CTRL_AUTOLOAD) != 0u)
        timer->counter = timer->load & FRC1_COUNT_MASK;
    if ((timer->config & FRC_CTRL_LEVEL_INT) != 0u) {
        timer->int_status = true;
        /* Compatibility-mode dispatch is synchronous; the ISR may clear
         * status, rewrite LOAD/ALARM, or disable the timer before return. */
        frc_update_irq(p, timer_index);
    } else {
        /* Edge mode is an electrical pulse, not a latched STATUS condition.
         * Assert first so compatibility ISRs run at the real event boundary. */
        timer->int_status = false;
        periph_assert_interrupt(p, 56 + (int)timer_index);
        periph_deassert_interrupt(p, 56 + (int)timer_index);
    }
    frc_refresh_event_active(p);
}

static void frc_advance_frc1(esp32_periph_t *p, uint64_t ticks) {
    frc_timer_state_t *timer = &p->frc_timer[0];
    const uint64_t modulus = (uint64_t)FRC1_COUNT_MASK + 1u;
    unsigned events = 0u;
    while (ticks != 0u) {
        if (timer->int_status) {
            if ((timer->config & FRC_CTRL_AUTOLOAD) == 0u) {
                timer->counter =
                    (timer->counter - (uint32_t)(ticks % modulus)) &
                    FRC1_COUNT_MASK;
                break;
            }

            uint64_t first = timer->counter;
            if (ticks < first) {
                timer->counter -= (uint32_t)ticks;
                break;
            }
            ticks -= first;
            uint64_t period = timer->load & FRC1_COUNT_MASK;
            if (period == 0u) period = modulus;
            uint64_t remainder = ticks % period;
            timer->counter = remainder == 0u ?
                (timer->load & FRC1_COUNT_MASK) :
                (uint32_t)(period - remainder) & FRC1_COUNT_MASK;
            break;
        }

        uint64_t distance = timer->counter != 0u ?
                            timer->counter : modulus;
        if (ticks < distance) {
            timer->counter =
                (timer->counter - (uint32_t)ticks) & FRC1_COUNT_MASK;
            break;
        }

        timer->counter = 0u;
        ticks -= distance;
        frc_fire(p, 0u);
        if (++events > 1000000u ||
            (timer->config & FRC_CTRL_ENABLE) == 0u)
            break;
    }
}

static void frc_advance_frc2(esp32_periph_t *p, uint64_t ticks) {
    frc_timer_state_t *timer = &p->frc_timer[1];
    const uint64_t modulus = (uint64_t)UINT32_MAX + 1u;
    unsigned events = 0u;
    while (ticks != 0u) {
        if (timer->int_status) {
            timer->counter += (uint32_t)(ticks % modulus);
            break;
        }

        uint64_t distance = (uint32_t)(timer->alarm - timer->counter);
        if (distance == 0u) distance = modulus;
        if (ticks < distance) {
            timer->counter += (uint32_t)ticks;
            break;
        }

        timer->counter = timer->alarm;
        ticks -= distance;
        frc_fire(p, 1u);
        if (++events > 1000000u ||
            (timer->config & FRC_CTRL_ENABLE) == 0u)
            break;
    }
}

static void frc_sync_timer_to(esp32_periph_t *p, unsigned timer_index,
                              uint64_t now) {
    frc_timer_state_t *timer = &p->frc_timer[timer_index];
    uint64_t elapsed = now >= timer->last_cycles ?
                       now - timer->last_cycles : 0u;
    timer->last_cycles = now;
    if ((timer->config & FRC_CTRL_ENABLE) == 0u) return;

    uint64_t denominator =
        (uint64_t)timg_cpu_mhz(p) * frc_prescaler(timer);
    uint64_t product = elapsed >
        (UINT64_MAX - timer->tick_remainder) / TIMG_APB_CLOCK_MHZ ?
        UINT64_MAX : timer->tick_remainder + elapsed * TIMG_APB_CLOCK_MHZ;
    uint64_t ticks = product / denominator;
    timer->tick_remainder = product % denominator;
    if (ticks == 0u) return;
    if (timer_index == 0u)
        frc_advance_frc1(p, ticks);
    else
        frc_advance_frc2(p, ticks);
}

static void frc_sync_all_to(esp32_periph_t *p, uint64_t now) {
    for (unsigned timer = 0; timer < FRC_TIMER_COUNT; timer++)
        frc_sync_timer_to(p, timer, now);
}

static uint64_t frc_cycles_until_ticks(const esp32_periph_t *p,
                                       const frc_timer_state_t *timer,
                                       uint64_t ticks) {
    uint64_t denominator =
        (uint64_t)timg_cpu_mhz(p) * frc_prescaler(timer);
    uint64_t needed = ticks > UINT64_MAX / denominator ? UINT64_MAX :
                      ticks * denominator;
    if (needed <= timer->tick_remainder) return 1u;
    needed -= timer->tick_remainder;
    return needed / TIMG_APB_CLOCK_MHZ +
           (needed % TIMG_APB_CLOCK_MHZ != 0u);
}

static uint32_t frc_next_fire(esp32_periph_t *p, xtensa_cpu_t *cpu) {
    if (!p || !cpu || !p->frc_event_active) return UINT32_MAX;
    uint64_t now = timg_now_cycles(p);
    frc_sync_all_to(p, now);
    bool have = false;
    uint64_t best_distance = 0u;
    for (unsigned timer_index = 0; timer_index < FRC_TIMER_COUNT;
         timer_index++) {
        frc_timer_state_t *timer = &p->frc_timer[timer_index];
        if ((timer->config & FRC_CTRL_ENABLE) == 0u || timer->int_status)
            continue;
        uint64_t ticks;
        if (timer_index == 0u) {
            ticks = timer->counter != 0u ? timer->counter :
                    (uint64_t)FRC1_COUNT_MASK + 1u;
        } else {
            ticks = (uint32_t)(timer->alarm - timer->counter);
            if (ticks == 0u) ticks = (uint64_t)UINT32_MAX + 1u;
        }
        uint64_t distance = frc_cycles_until_ticks(p, timer, ticks);
        if (!have || distance < best_distance) {
            have = true;
            best_distance = distance;
        }
    }
    if (!have) return UINT32_MAX;
    if (best_distance >= (uint64_t)INT32_MAX)
        best_distance = (uint64_t)INT32_MAX - 1u;
    return cpu->ccount + (uint32_t)best_distance;
}

static void frc_eval_events(esp32_periph_t *p, xtensa_cpu_t *cpu) {
    (void)cpu;
    if (!p->frc_event_active) return;
    frc_sync_all_to(p, timg_now_cycles(p));
}

static uint32_t frc_read(void *ctx, uint32_t addr) {
    esp32_periph_t *p = ctx;
    uint32_t off = addr - FRC_TIMER_BASE;
    unsigned timer_index = off / FRC_TIMER_STRIDE;
    uint32_t relative = off % FRC_TIMER_STRIDE;
    if (timer_index >= FRC_TIMER_COUNT || relative > 0x10u) return 0u;

    frc_sync_timer_to(p, timer_index, timg_now_cycles(p));
    frc_timer_state_t *timer = &p->frc_timer[timer_index];
    switch (relative) {
    case 0x00: return timer->load;
    case 0x04: return timer->counter;
    case 0x08: return timer->config |
                      (timer->int_status ? FRC_CTRL_INT_STATUS : 0u);
    case 0x0C: return 0u;
    case 0x10: return timer_index == 1u ? timer->alarm : 0u;
    default: return 0u;
    }
}

static void frc_write(void *ctx, uint32_t addr, uint32_t value) {
    esp32_periph_t *p = ctx;
    uint32_t off = addr - FRC_TIMER_BASE;
    unsigned timer_index = off / FRC_TIMER_STRIDE;
    uint32_t relative = off % FRC_TIMER_STRIDE;
    if (timer_index >= FRC_TIMER_COUNT || relative > 0x10u) return;

    frc_sync_timer_to(p, timer_index, timg_now_cycles(p));
    frc_timer_state_t *timer = &p->frc_timer[timer_index];
    uint32_t mask = frc_count_mask(timer_index);
    switch (relative) {
    case 0x00:
        timer->load = value & mask;
        timer->counter = timer->load;
        timer->tick_remainder = 0u;
        frc_kick(p);
        break;
    case 0x08: {
        uint32_t old_config = timer->config;
        timer->config = value & FRC_CTRL_CONFIG_MASK;
        uint32_t prescaler = timer->config & FRC_CTRL_PRESCALER_MASK;
        if (prescaler != 0u && prescaler != (2u << 1) &&
            prescaler != (4u << 1))
            timer->config &= ~FRC_CTRL_PRESCALER_MASK;
        if ((old_config & FRC_CTRL_PRESCALER_MASK) !=
            (timer->config & FRC_CTRL_PRESCALER_MASK))
            timer->tick_remainder = 0u;
        frc_update_irq(p, timer_index);
        frc_kick(p);
        break;
    }
    case 0x0C:
        if ((value & 1u) != 0u) {
            timer->int_status = false;
            frc_update_irq(p, timer_index);
            frc_kick(p);
        }
        break;
    case 0x10:
        if (timer_index == 1u) {
            timer->alarm = value;
            frc_kick(p);
        }
        break;
    default:
        break;
    }
}

static void frc_reset(esp32_periph_t *p) {
    if (!p) return;
    for (unsigned timer = 0; timer < FRC_TIMER_COUNT; timer++)
        timg_drive_source(p, 56 + (int)timer, false);
    memset(p->frc_timer, 0, sizeof(p->frc_timer));
    for (unsigned timer = 0; timer < FRC_TIMER_COUNT; timer++)
        p->frc_timer[timer].last_cycles = p->timg_clock.cycles;
    p->frc_event_active = false;
}

/* CPU hooks (registered on both cores, wired into next_timer_event). */
static uint32_t periph_next_event_hook(xtensa_cpu_t *cpu) {
    esp32_periph_t *p = (esp32_periph_t *)cpu->periph_event_ctx;
    uint32_t events[] = {
        timg_next_fire(p, cpu),
        lact_next_fire(p, cpu),
        frc_next_fire(p, cpu),
        i2s_next_fire(p, cpu),
        rmt_next_fire(p, cpu),
        ledc_next_fire(p, cpu),
        pcnt_next_fire(p, cpu),
        mcpwm_next_fire(p, cpu),
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
    timg_eval_events(p, cpu);
    for (int group = 0; group < 2; group++)
        lact_eval_irq(p, group);
    frc_eval_events(p, cpu);
    i2s_eval_events(p, cpu);
    rmt_eval_events(p, cpu);
    ledc_eval_events(p, cpu);
    pcnt_eval_events(p, cpu);
    mcpwm_eval_events(p, cpu);
}

/* Recompute both cores' next_timer_event after LACT state changes. */
static void lact_kick(esp32_periph_t *p) {
    for (int core = 0; core < 2; core++)
        if (p->cpu[core]) xtensa_recompute_next_timer(p->cpu[core]);
}

/* ---- TIMG register file (GPTimer, WDT, calibration, and LACT) ---- */

static uint32_t timg_read(void *ctx, uint32_t addr) {
    esp32_periph_t *p = ctx;
    int group = (addr >= TIMG1_BASE) ? 1 : 0;
    uint32_t base = group ? TIMG1_BASE : TIMG0_BASE;
    uint32_t off = addr - base;
    timg_group_state_t *state = &p->timg[group];
    wdt_state_t *w = &p->timg_wdt[group];
    lact_state_t *l = &p->lact[group];

    timg_sync_group_to(p, (unsigned)group, timg_now_cycles(p));
    if (off < 0x048u) {
        unsigned timer_index = off / TIMG_TIMER_STRIDE;
        uint32_t relative = off % TIMG_TIMER_STRIDE;
        if (timer_index < TIMG_TIMER_COUNT) {
            timg_timer_state_t *timer = &state->timer[timer_index];
            switch (relative) {
            case 0x00: return timer->config;
            case 0x04: return (uint32_t)timer->latched;
            case 0x08: return (uint32_t)(timer->latched >> 32);
            case 0x0C: return 0u;
            case 0x10: return (uint32_t)timer->alarm;
            case 0x14: return (uint32_t)(timer->alarm >> 32);
            case 0x18: return (uint32_t)timer->load;
            case 0x1C: return (uint32_t)(timer->load >> 32);
            case 0x20: return 0u;
            default: return 0u;
            }
        }
    }

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
    case 0x098: return state->int_ena;                     /* INT_ENA_TIMERS */
    case 0x09C: lact_eval_irq(p, group); return state->int_raw; /* INT_RAW_TIMERS */
    case 0x0A0: lact_eval_irq(p, group);                   /* INT_ST_TIMERS */
                return state->int_raw & state->int_ena;
    case 0x0A4: return 0u;                                 /* INT_CLR_TIMERS */
    case 0x0F8: return state->date;
    case 0x0FC: return state->regclk;
    default: return 0;
    }
}

static void timg_write(void *ctx, uint32_t addr, uint32_t val) {
    esp32_periph_t *p = ctx;
    int group = (addr >= TIMG1_BASE) ? 1 : 0;
    uint32_t base = group ? TIMG1_BASE : TIMG0_BASE;
    uint32_t off = addr - base;
    timg_group_state_t *state = &p->timg[group];
    wdt_state_t *w = &p->timg_wdt[group];
    lact_state_t *l = &p->lact[group];

    uint64_t now = timg_now_cycles(p);
    timg_sync_group_to(p, (unsigned)group, now);
    if (off < 0x048u) {
        unsigned timer_index = off / TIMG_TIMER_STRIDE;
        uint32_t relative = off % TIMG_TIMER_STRIDE;
        if (timer_index < TIMG_TIMER_COUNT) {
            timg_timer_state_t *timer = &state->timer[timer_index];
            switch (relative) {
            case 0x00: {
                uint32_t old_config = timer->config;
                timer->config = val & TIMG_TIMER_CONFIG_MASK;
                if (((old_config ^ timer->config) &
                     (0xFFFFu << 13)) != 0u)
                    timer->tick_remainder = 0u;
                timg_update_timer_irq(p, (unsigned)group, timer_index);
                timg_kick(p);
                return;
            }
            case 0x0C:
                timer->latched = timer->counter;
                return;
            case 0x10:
                timer->alarm = (timer->alarm & 0xFFFFFFFF00000000ull) |
                               val;
                timg_kick(p);
                return;
            case 0x14:
                timer->alarm = (timer->alarm & 0xFFFFFFFFull) |
                               ((uint64_t)val << 32);
                timg_kick(p);
                return;
            case 0x18:
                timer->load = (timer->load & 0xFFFFFFFF00000000ull) | val;
                return;
            case 0x1C:
                timer->load = (timer->load & 0xFFFFFFFFull) |
                              ((uint64_t)val << 32);
                return;
            case 0x20:
                timer->counter = timer->load;
                timer->tick_remainder = 0u;
                timg_kick(p);
                return;
            default:
                return;
            }
        }
    }

    switch (off) {
    case 0x048: w->config0 = val;
                timg_update_wdt_irq(p, (unsigned)group); break;
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
    case 0x098:
        state->int_ena = val & TIMG_INT_VALID_MASK;
        lact_latch_alarm(p, group);
        timg_update_group_irqs(p, (unsigned)group);
        timg_kick(p);
        break;
    case 0x0A4:
        state->int_raw &= ~(val & TIMG_INT_VALID_MASK);
        lact_latch_alarm(p, group);
        timg_update_group_irqs(p, (unsigned)group);
        timg_kick(p);
        break;
    case 0x0F8: state->date = val & 0x0FFFFFFFu; break;
    case 0x0FC: state->regclk = val & (1u << 31); break;
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

/* ---- PCNT pulse counter ---- */

static uint32_t pcnt_unit_conf_offset(unsigned unit) {
    return unit * PCNT_UNIT_CONF_STRIDE;
}

static uint32_t pcnt_signal_base(unsigned unit) {
    return unit < 5u ? 39u + unit * 4u : 71u + (unit - 5u) * 4u;
}

static uint32_t pcnt_filter_bit(unsigned unit, unsigned channel,
                                bool control) {
    return 1u << (unit * 4u + channel * 2u + (control ? 1u : 0u));
}

static bool pcnt_decode_signal(unsigned signal, unsigned *unit,
                               unsigned *channel, bool *control) {
    for (unsigned candidate = 0; candidate < PCNT_UNIT_COUNT; candidate++) {
        unsigned base = pcnt_signal_base(candidate);
        if (signal < base || signal >= base + 4u) continue;
        unsigned relative = signal - base;
        if (unit) *unit = candidate;
        if (channel) *channel = relative & 1u;
        if (control) *control = relative >= 2u;
        return true;
    }
    return false;
}

static int pcnt_matrix_gpio(const esp32_periph_t *p, unsigned signal) {
    uint32_t route = p->gpio.func_in_sel[signal];
    if (!(route & (1u << 7))) return -1;
    unsigned gpio = route & 0x3Fu;
    return gpio < 40u ? (int)gpio : -1;
}

static bool pcnt_matrix_level(const esp32_periph_t *p, unsigned signal) {
    uint32_t route = p->gpio.func_in_sel[signal];
    bool level = false;
    if (route & (1u << 7)) {
        unsigned gpio = route & 0x3Fu;
        if (gpio < 32u)
            level = (p->gpio.in & (1u << gpio)) != 0;
        else if (gpio < 40u)
            level = (p->gpio.in1 & (1u << (gpio - 32u))) != 0;
        else if (gpio == 0x38u)
            level = true;  /* GPIO_MATRIX_CONST_ONE_INPUT */
        /* GPIO_MATRIX_CONST_ZERO_INPUT (0x30) and other internal selectors
         * default low in the host-facing GPIO model. */
    }
    if (route & (1u << 6)) level = !level;
    return level;
}

static uint32_t pcnt_cpu_mhz(const esp32_periph_t *p) {
    uint32_t mhz = mem_read32(p->mem, ESP32_CPU_TICKS_PER_US_ADDR);
    return mhz >= 10u && mhz <= 240u ? mhz : 240u;
}

static void pcnt_kick(esp32_periph_t *p) {
    for (int core = 0; core < 2; core++)
        if (p->cpu[core]) xtensa_recompute_next_timer(p->cpu[core]);
}

static void pcnt_update_irq(esp32_periph_t *p) {
    uint32_t raw = p->pcnt.regs[PCNT_INT_RAW_OFF / 4u];
    uint32_t ena = p->pcnt.regs[PCNT_INT_ENA_OFF / 4u];
    if (raw & ena)
        periph_assert_interrupt(p, PCNT_INTR_SOURCE);
    else
        periph_deassert_interrupt(p, PCNT_INTR_SOURCE);
}

static bool pcnt_unit_running(const esp32_periph_t *p, unsigned unit) {
    uint32_t ctrl = p->pcnt.regs[PCNT_CTRL_OFF / 4u];
    return (p->dport_perip_clk_en & DPORT_PCNT_MODULE_BIT) != 0 &&
           (p->dport_perip_rst_en & DPORT_PCNT_MODULE_BIT) == 0 &&
           (ctrl & (1u << (unit * 2u))) == 0 &&
           (ctrl & (1u << (unit * 2u + 1u))) == 0;
}

static void pcnt_set_count_mode(pcnt_unit_state_t *state, int16_t before,
                                int16_t after) {
    uint32_t mode;
    if (after > 0)
        mode = 3u;
    else if (after < 0)
        mode = 2u;
    else
        mode = before < 0 ? 1u : 0u;
    state->status = (state->status & ~3u) | mode;
}

static uint32_t pcnt_enabled_events(uint32_t conf0,
                                    uint32_t event_flags) {
    uint32_t enabled = 0;
    for (unsigned bit = 2u; bit <= 6u; bit++) {
        uint32_t event = 1u << bit;
        uint32_t enable = 1u << (17u - bit);
        if ((event_flags & event) && (conf0 & enable)) enabled |= event;
    }
    return enabled;
}

static void pcnt_latch_events(esp32_periph_t *p, unsigned unit,
                              uint32_t event_flags) {
    uint32_t conf0 = p->pcnt.regs[pcnt_unit_conf_offset(unit) / 4u];
    uint32_t enabled = pcnt_enabled_events(conf0, event_flags);
    if (!enabled) return;

    pcnt_unit_state_t *state = &p->pcnt.unit[unit];
    state->status = (state->status & 3u) | enabled;
    p->pcnt.regs[PCNT_INT_RAW_OFF / 4u] |= 1u << unit;
    pcnt_update_irq(p);
}

static unsigned pcnt_edge_action(uint32_t conf0, unsigned channel,
                                 bool rising) {
    unsigned shift = channel == 0u ? (rising ? 18u : 16u) :
                                     (rising ? 26u : 24u);
    return (conf0 >> shift) & 3u;
}

static unsigned pcnt_control_action(uint32_t conf0, unsigned channel,
                                    bool high) {
    unsigned shift = channel == 0u ? (high ? 20u : 22u) :
                                     (high ? 28u : 30u);
    return (conf0 >> shift) & 3u;
}

static void pcnt_count_edge(esp32_periph_t *p, unsigned unit,
                            unsigned channel, bool rising) {
    if (!pcnt_unit_running(p, unit)) return;

    uint32_t base = pcnt_unit_conf_offset(unit);
    uint32_t conf0 = p->pcnt.regs[base / 4u];
    unsigned edge_action = pcnt_edge_action(conf0, channel, rising);
    if (edge_action != 1u && edge_action != 2u) return;

    unsigned control_action = pcnt_control_action(
        conf0, channel, p->pcnt.unit[unit].control_level[channel]);
    if (control_action >= 2u) return; /* hold/forbidden */

    int delta = edge_action == 1u ? 1 : -1;
    if (control_action == 1u) delta = -delta;

    pcnt_unit_state_t *state = &p->pcnt.unit[unit];
    int16_t before = state->count;
    int32_t candidate = (int32_t)before + delta;
    int16_t threshold0 = (int16_t)
        (p->pcnt.regs[(base + 0x04u) / 4u] & 0xFFFFu);
    int16_t threshold1 = (int16_t)
        (p->pcnt.regs[(base + 0x04u) / 4u] >> 16);
    int16_t high_limit = (int16_t)
        (p->pcnt.regs[(base + 0x08u) / 4u] & 0xFFFFu);
    int16_t low_limit = (int16_t)
        (p->pcnt.regs[(base + 0x08u) / 4u] >> 16);
    uint32_t events = 0;

    if (candidate == threshold0) events |= PCNT_EVT_THRES0;
    if (candidate == threshold1) events |= PCNT_EVT_THRES1;
    if (candidate == 0) events |= PCNT_EVT_ZERO;

    bool hit_high = delta > 0 && high_limit > 0 &&
                    candidate >= high_limit;
    bool hit_low = delta < 0 && low_limit < 0 && candidate <= low_limit;
    if (hit_high) events |= PCNT_EVT_H_LIM;
    if (hit_low) events |= PCNT_EVT_L_LIM;

    int16_t after = hit_high || hit_low ? 0 : (int16_t)candidate;
    state->count = after;
    pcnt_set_count_mode(state, before, after);
    pcnt_latch_events(p, unit, events);
}

static void pcnt_accept_level(esp32_periph_t *p, unsigned unit,
                              unsigned channel, bool control, bool level) {
    pcnt_unit_state_t *state = &p->pcnt.unit[unit];
    bool *stable = control ? &state->control_level[channel] :
                             &state->pulse_level[channel];
    bool before = *stable;
    *stable = level;
    if (!control && before != level)
        pcnt_count_edge(p, unit, channel, level);
}

static uint32_t pcnt_filter_cpu_cycles(const esp32_periph_t *p,
                                       uint32_t threshold) {
    uint64_t numerator = (uint64_t)threshold * pcnt_cpu_mhz(p);
    uint32_t cycles = (uint32_t)((numerator + 79u) / 80u);
    return cycles ? cycles : 1u;
}

static void pcnt_handle_signal_level(esp32_periph_t *p, unsigned unit,
                                     unsigned channel, bool control) {
    unsigned signal = pcnt_signal_base(unit) + channel +
                      (control ? 2u : 0u);
    bool level = pcnt_matrix_level(p, signal);
    pcnt_unit_state_t *state = &p->pcnt.unit[unit];
    bool stable = control ? state->control_level[channel] :
                            state->pulse_level[channel];
    pcnt_filter_state_t *filter = control ?
        &state->control_filter[channel] : &state->pulse_filter[channel];
    uint32_t conf0 =
        p->pcnt.regs[pcnt_unit_conf_offset(unit) / 4u];
    uint32_t threshold = conf0 & PCNT_CONF_FILTER_MASK;
    uint32_t pending_bit = pcnt_filter_bit(unit, channel, control);

    if (!(conf0 & PCNT_CONF_FILTER_EN) || threshold == 0u || !p->cpu[0]) {
        filter->pending = false;
        p->pcnt.pending_filters &= ~pending_bit;
        if (level != stable)
            pcnt_accept_level(p, unit, channel, control, level);
        return;
    }

    if (level == stable) {
        filter->pending = false; /* pulse returned before qualification */
        p->pcnt.pending_filters &= ~pending_bit;
    } else {
        filter->pending = true;
        p->pcnt.pending_filters |= pending_bit;
        filter->level = level;
        filter->deadline = p->cpu[0]->ccount +
            pcnt_filter_cpu_cycles(p, threshold);
    }
    pcnt_kick(p);
}

static void pcnt_rebind_signal(esp32_periph_t *p, unsigned unit,
                               unsigned channel, bool control) {
    unsigned signal = pcnt_signal_base(unit) + channel +
                      (control ? 2u : 0u);
    pcnt_unit_state_t *state = &p->pcnt.unit[unit];
    pcnt_filter_state_t *filter = control ?
        &state->control_filter[channel] : &state->pulse_filter[channel];
    filter->pending = false;
    p->pcnt.pending_filters &= ~pcnt_filter_bit(unit, channel, control);
    if (control)
        state->control_level[channel] = pcnt_matrix_level(p, signal);
    else
        state->pulse_level[channel] = pcnt_matrix_level(p, signal);
}

static void pcnt_gpio_route_changed(esp32_periph_t *p, unsigned signal) {
    unsigned unit;
    unsigned channel;
    bool control;
    if (!pcnt_decode_signal(signal, &unit, &channel, &control)) return;
    pcnt_rebind_signal(p, unit, channel, control);
    pcnt_kick(p);
}

static void pcnt_gpio_input_changed(esp32_periph_t *p, int gpio) {
    for (unsigned unit = 0; unit < PCNT_UNIT_COUNT; unit++) {
        for (unsigned channel = 0; channel < PCNT_CHANNEL_COUNT; channel++) {
            for (unsigned control = 0; control < 2u; control++) {
                unsigned signal = pcnt_signal_base(unit) + channel +
                                  (control ? 2u : 0u);
                if (pcnt_matrix_gpio(p, signal) == gpio)
                    pcnt_handle_signal_level(p, unit, channel,
                                             control != 0u);
            }
        }
    }
}

static uint32_t pcnt_next_fire(esp32_periph_t *p, xtensa_cpu_t *cpu) {
    if (!p || !cpu || cpu != p->cpu[0]) return UINT32_MAX;
    if (p->pcnt.pending_filters == 0) return UINT32_MAX;
    bool have = false;
    uint32_t best = UINT32_MAX;
    uint32_t best_distance = 0;
    for (unsigned unit = 0; unit < PCNT_UNIT_COUNT; unit++) {
        pcnt_unit_state_t *state = &p->pcnt.unit[unit];
        for (unsigned channel = 0; channel < PCNT_CHANNEL_COUNT; channel++) {
            pcnt_filter_state_t *filters[2] = {
                &state->pulse_filter[channel],
                &state->control_filter[channel],
            };
            for (unsigned index = 0; index < 2u; index++) {
                pcnt_filter_state_t *filter = filters[index];
                if (!filter->pending) continue;
                uint32_t distance = filter->deadline - cpu->ccount;
                if ((int32_t)distance < 0) distance = 0;
                if (!have || distance < best_distance) {
                    have = true;
                    best = cpu->ccount + distance;
                    best_distance = distance;
                }
            }
        }
    }
    return have ? best : UINT32_MAX;
}

static void pcnt_eval_filter(esp32_periph_t *p, unsigned unit,
                             unsigned channel, bool control) {
    pcnt_unit_state_t *state = &p->pcnt.unit[unit];
    pcnt_filter_state_t *filter = control ?
        &state->control_filter[channel] : &state->pulse_filter[channel];
    if (!filter->pending ||
        (int32_t)(p->cpu[0]->ccount - filter->deadline) < 0)
        return;

    unsigned signal = pcnt_signal_base(unit) + channel +
                      (control ? 2u : 0u);
    bool level = pcnt_matrix_level(p, signal);
    bool accepted = level == filter->level;
    filter->pending = false;
    p->pcnt.pending_filters &= ~pcnt_filter_bit(unit, channel, control);
    if (accepted)
        pcnt_accept_level(p, unit, channel, control, level);
}

static void pcnt_eval_events(esp32_periph_t *p, xtensa_cpu_t *cpu) {
    if (!p || !cpu || cpu != p->cpu[0]) return;
    if (p->pcnt.pending_filters == 0) return;
    for (unsigned unit = 0; unit < PCNT_UNIT_COUNT; unit++) {
        for (unsigned channel = 0; channel < PCNT_CHANNEL_COUNT; channel++) {
            /* Qualify control first so a pulse edge expiring at the same APB
             * cycle observes the newly stable control level. */
            pcnt_eval_filter(p, unit, channel, true);
            pcnt_eval_filter(p, unit, channel, false);
        }
    }
    pcnt_kick(p);
}

static void pcnt_reset_state(esp32_periph_t *p) {
    memset(&p->pcnt, 0, sizeof(p->pcnt));
    for (unsigned unit = 0; unit < PCNT_UNIT_COUNT; unit++) {
        uint32_t base = pcnt_unit_conf_offset(unit);
        p->pcnt.regs[base / 4u] = 0x00003C10u;
    }
    p->pcnt.regs[PCNT_CTRL_OFF / 4u] = 0x00005555u;
    p->pcnt.regs[PCNT_DATE_OFF / 4u] = 0x14122600u;
    for (unsigned unit = 0; unit < PCNT_UNIT_COUNT; unit++)
        for (unsigned channel = 0; channel < PCNT_CHANNEL_COUNT; channel++) {
            pcnt_rebind_signal(p, unit, channel, false);
            pcnt_rebind_signal(p, unit, channel, true);
        }
    pcnt_update_irq(p);
    pcnt_kick(p);
}

static uint32_t pcnt_read(void *ctx, uint32_t addr) {
    esp32_periph_t *p = ctx;
    uint32_t off = addr - PCNT_BASE;
    if ((off & 3u) || off > PCNT_DATE_OFF)
        return default_read(ctx, addr);
    if (p->cpu[0]) pcnt_eval_events(p, p->cpu[0]);

    if (off >= PCNT_CNT_OFF && off < PCNT_CNT_OFF + PCNT_UNIT_COUNT * 4u) {
        unsigned unit = (off - PCNT_CNT_OFF) / 4u;
        return (uint16_t)p->pcnt.unit[unit].count;
    }
    if (off == PCNT_INT_ST_OFF)
        return p->pcnt.regs[PCNT_INT_RAW_OFF / 4u] &
               p->pcnt.regs[PCNT_INT_ENA_OFF / 4u];
    if (off == PCNT_INT_CLR_OFF) return 0;
    if (off >= PCNT_STATUS_OFF &&
        off < PCNT_STATUS_OFF + PCNT_UNIT_COUNT * 4u) {
        unsigned unit = (off - PCNT_STATUS_OFF) / 4u;
        return p->pcnt.unit[unit].status;
    }
    return p->pcnt.regs[off / 4u];
}

static void pcnt_write(void *ctx, uint32_t addr, uint32_t val) {
    esp32_periph_t *p = ctx;
    uint32_t off = addr - PCNT_BASE;
    if ((off & 3u) || off > PCNT_DATE_OFF) {
        default_write(ctx, addr, val);
        return;
    }
    if (p->cpu[0]) pcnt_eval_events(p, p->cpu[0]);

    if (off < PCNT_CNT_OFF) {
        p->pcnt.regs[off / 4u] = val;
        if (off % PCNT_UNIT_CONF_STRIDE == 0u) {
            unsigned unit = off / PCNT_UNIT_CONF_STRIDE;
            for (unsigned channel = 0; channel < PCNT_CHANNEL_COUNT;
                 channel++) {
                pcnt_handle_signal_level(p, unit, channel, false);
                pcnt_handle_signal_level(p, unit, channel, true);
            }
        }
        return;
    }
    if (off >= PCNT_CNT_OFF && off < PCNT_CNT_OFF + PCNT_UNIT_COUNT * 4u)
        return; /* live counters are read-only */
    if (off == PCNT_INT_RAW_OFF || off == PCNT_INT_ST_OFF ||
        (off >= PCNT_STATUS_OFF &&
         off < PCNT_STATUS_OFF + PCNT_UNIT_COUNT * 4u))
        return;
    if (off == PCNT_INT_ENA_OFF) {
        p->pcnt.regs[off / 4u] = val & PCNT_INT_VALID_MASK;
        pcnt_update_irq(p);
        return;
    }
    if (off == PCNT_INT_CLR_OFF) {
        p->pcnt.regs[PCNT_INT_RAW_OFF / 4u] &=
            ~(val & PCNT_INT_VALID_MASK);
        pcnt_update_irq(p);
        return;
    }
    if (off == PCNT_CTRL_OFF) {
        p->pcnt.regs[off / 4u] = val & 0x1FFFFu;
        for (unsigned unit = 0; unit < PCNT_UNIT_COUNT; unit++) {
            if (!(val & (1u << (unit * 2u)))) continue;
            p->pcnt.unit[unit].count = 0;
            p->pcnt.unit[unit].status = 0;
        }
        return;
    }
    p->pcnt.regs[off / 4u] = val;
}

/* ---- MCPWM motor-control PWM ---- */

static bool mcpwm_operator_has_active_cbc_fault(
    const mcpwm_unit_state_t *state, unsigned operator_index);
static void mcpwm_clear_operator_cbc(esp32_periph_t *p, unsigned unit,
                                      unsigned operator_index);

static uint32_t mcpwm_timer_offset(unsigned timer) {
    return MCPWM_TIMER_BASE_OFF + timer * MCPWM_TIMER_STRIDE;
}

static uint32_t mcpwm_operator_offset(unsigned operator_index) {
    return MCPWM_OPERATOR_BASE_OFF +
           operator_index * MCPWM_OPERATOR_STRIDE;
}

static unsigned mcpwm_addr_unit(uint32_t addr) {
    return addr >= MCPWM1_BASE ? 1u : 0u;
}

static uint32_t mcpwm_unit_base(unsigned unit) {
    return unit ? MCPWM1_BASE : MCPWM0_BASE;
}

static uint32_t mcpwm_unit_dport_bit(unsigned unit) {
    return unit ? DPORT_PWM1_MODULE_BIT : DPORT_PWM0_MODULE_BIT;
}

static int mcpwm_unit_interrupt_source(unsigned unit) {
    return unit ? 40 : 39;
}

static uint32_t mcpwm_output_signal(unsigned unit,
                                    unsigned operator_index,
                                    unsigned generator) {
    return (unit ? 108u : 32u) + operator_index * 2u + generator;
}

static uint32_t mcpwm_sync_signal(unsigned unit, unsigned channel) {
    return (unit ? 103u : 31u) + channel;
}

static uint32_t mcpwm_fault_signal(unsigned unit, unsigned channel) {
    return (unit ? 106u : 34u) + channel;
}

static uint32_t mcpwm_capture_signal(unsigned unit, unsigned channel) {
    return (unit ? 112u : 109u) + channel;
}

static bool mcpwm_decode_input_signal(unsigned signal, unsigned *unit,
                                      unsigned *kind, unsigned *channel) {
    for (unsigned candidate = 0; candidate < MCPWM_UNIT_COUNT; candidate++) {
        for (unsigned index = 0; index < MCPWM_TIMER_COUNT; index++) {
            if (signal == mcpwm_sync_signal(candidate, index)) {
                if (unit) *unit = candidate;
                if (kind) *kind = 0;
                if (channel) *channel = index;
                return true;
            }
            if (signal == mcpwm_fault_signal(candidate, index)) {
                if (unit) *unit = candidate;
                if (kind) *kind = 1;
                if (channel) *channel = index;
                return true;
            }
            if (signal == mcpwm_capture_signal(candidate, index)) {
                if (unit) *unit = candidate;
                if (kind) *kind = 2;
                if (channel) *channel = index;
                return true;
            }
        }
    }
    return false;
}

static int mcpwm_matrix_input_gpio(const esp32_periph_t *p,
                                   unsigned signal) {
    uint32_t route = p->gpio.func_in_sel[signal];
    if (!(route & (1u << 7))) return -1;
    unsigned gpio = route & 0x3Fu;
    return gpio < 40u ? (int)gpio : -1;
}

static bool mcpwm_matrix_input_level(const esp32_periph_t *p,
                                     unsigned signal) {
    uint32_t route = p->gpio.func_in_sel[signal];
    bool level = false;
    if (route & (1u << 7)) {
        unsigned gpio = route & 0x3Fu;
        if (gpio < 32u)
            level = (p->gpio.in & (1u << gpio)) != 0;
        else if (gpio < 40u)
            level = (p->gpio.in1 & (1u << (gpio - 32u))) != 0;
        else if (gpio == 0x38u)
            level = true;
    }
    if (route & (1u << 6)) level = !level;
    return level;
}

static int mcpwm_output_gpio(const esp32_periph_t *p, unsigned unit,
                             unsigned operator_index, unsigned generator,
                             bool *inverted) {
    uint32_t signal = mcpwm_output_signal(unit, operator_index, generator);
    for (int gpio = 0; gpio < 40; gpio++) {
        uint32_t route = p->gpio.func_out_sel[gpio];
        if ((route & 0x1FFu) != signal) continue;
        if (inverted) *inverted = (route & (1u << 9)) != 0;
        return gpio;
    }
    if (inverted) *inverted = false;
    return -1;
}

static uint32_t mcpwm_cpu_mhz(const esp32_periph_t *p) {
    uint32_t mhz = mem_read32(p->mem, ESP32_CPU_TICKS_PER_US_ADDR);
    return mhz >= 10u && mhz <= 240u ? mhz : 240u;
}

static uint64_t mcpwm_now_cycles(esp32_periph_t *p) {
    for (unsigned core = 0; core < 2u; core++) {
        if (!p->cpu[core]) continue;
        uint32_t now = p->cpu[core]->ccount;
        if (!p->mcpwm.time_valid[core]) {
            p->mcpwm.last_ccount[core] = now;
            p->mcpwm.core_cycles[core] = p->mcpwm.time_cycles;
            p->mcpwm.time_valid[core] = true;
            continue;
        }
        uint32_t elapsed = now - p->mcpwm.last_ccount[core];
        if (elapsed < (uint32_t)INT32_MAX) {
            if (p->mcpwm.core_cycles[core] > UINT64_MAX - elapsed)
                p->mcpwm.core_cycles[core] = UINT64_MAX;
            else
                p->mcpwm.core_cycles[core] += elapsed;
        }
        p->mcpwm.last_ccount[core] = now;
        if (p->mcpwm.core_cycles[core] > p->mcpwm.time_cycles)
            p->mcpwm.time_cycles = p->mcpwm.core_cycles[core];
    }
    return p->mcpwm.time_cycles;
}

static bool mcpwm_unit_clocked(const esp32_periph_t *p, unsigned unit) {
    uint32_t bit = mcpwm_unit_dport_bit(unit);
    return (p->dport_perip_clk_en & bit) != 0 &&
           (p->dport_perip_rst_en & bit) == 0;
}

static uint32_t mcpwm_group_divider(const mcpwm_unit_state_t *state) {
    return (state->regs[0] & 0xFFu) + 1u;
}

static uint32_t mcpwm_group_frequency_hz(const esp32_periph_t *p,
                                         unsigned unit) {
    if (!mcpwm_unit_clocked(p, unit)) return 0;
    return MCPWM_SOURCE_CLOCK_MHZ * 1000000u /
           mcpwm_group_divider(&p->mcpwm.unit[unit]);
}

static uint32_t mcpwm_timer_cycle(const mcpwm_timer_state_t *timer) {
    if (timer->mode == 3u) {
        return timer->active_period ? timer->active_period * 2u : 1u;
    }
    return timer->active_period + 1u;
}

static uint32_t mcpwm_timer_count(const mcpwm_timer_state_t *timer) {
    uint32_t period = timer->active_period;
    uint32_t cycle = mcpwm_timer_cycle(timer);
    uint32_t phase = cycle ? timer->phase % cycle : 0u;
    if (timer->mode == 2u)
        return phase <= period ? period - phase : 0u;
    if (timer->mode == 3u && phase > period)
        return period * 2u - phase;
    return phase <= period ? phase : 0u;
}

static bool mcpwm_timer_down(const mcpwm_timer_state_t *timer) {
    if (timer->mode == 2u) return true;
    return timer->mode == 3u && timer->phase > timer->active_period;
}

static uint64_t mcpwm_timer_denominator(const esp32_periph_t *p,
                                        unsigned unit,
                                        const mcpwm_timer_state_t *timer) {
    return (uint64_t)mcpwm_cpu_mhz(p) *
           mcpwm_group_divider(&p->mcpwm.unit[unit]) *
           ((uint32_t)timer->active_prescale + 1u);
}

static uint32_t mcpwm_timer_frequency_hz(const esp32_periph_t *p,
                                         unsigned unit, unsigned timer_index) {
    const mcpwm_timer_state_t *timer =
        &p->mcpwm.unit[unit].timer[timer_index];
    if (!timer->running || timer->mode == 0u ||
        !mcpwm_unit_clocked(p, unit))
        return 0;
    uint64_t denominator =
        (uint64_t)mcpwm_group_divider(&p->mcpwm.unit[unit]) *
        ((uint32_t)timer->active_prescale + 1u) *
        mcpwm_timer_cycle(timer);
    return denominator ?
        (uint32_t)((uint64_t)MCPWM_SOURCE_CLOCK_MHZ * 1000000u /
                   denominator) : 0u;
}

static unsigned mcpwm_operator_timer(const mcpwm_unit_state_t *state,
                                      unsigned operator_index) {
    unsigned timer =
        (state->regs[MCPWM_OPERATOR_TIMERSEL_OFF / 4u] >>
         (operator_index * 2u)) & 3u;
    return timer < MCPWM_TIMER_COUNT ? timer : 0u;
}

static int mcpwm_forced_level(const mcpwm_operator_state_t *op,
                              unsigned generator) {
    if (op->ost_on && op->ost_override_valid[generator])
        return op->ost_override_level[generator] ? 1 : 0;
    if (op->cbc_on && op->cbc_override_valid[generator])
        return op->cbc_override_level[generator] ? 1 : 0;
    if (op->active_force[generator] == 1u) return 0;
    if (op->active_force[generator] == 2u) return 1;
    return -1;
}

static bool mcpwm_info_equal(const periph_mcpwm_output_info_t *left,
                             const periph_mcpwm_output_info_t *right) {
    return left->gpio == right->gpio &&
           left->frequency_hz == right->frequency_hz &&
           left->period_ticks == right->period_ticks &&
           left->compare_ticks == right->compare_ticks &&
           left->rising_delay_ticks == right->rising_delay_ticks &&
           left->falling_delay_ticks == right->falling_delay_ticks &&
           left->deadtime_clock_hz == right->deadtime_clock_hz &&
           left->carrier_hz == right->carrier_hz &&
           left->carrier_duty_eighths == right->carrier_duty_eighths &&
           left->count_mode == right->count_mode &&
           left->enabled == right->enabled &&
           left->inverted == right->inverted &&
           left->fault_active == right->fault_active &&
           left->forced_level == right->forced_level;
}

static void mcpwm_emit_output(esp32_periph_t *p, unsigned unit,
                              unsigned operator_index, unsigned generator,
                              bool force) {
    mcpwm_unit_state_t *state = &p->mcpwm.unit[unit];
    mcpwm_operator_state_t *op = &state->operators[operator_index];
    unsigned timer_index = mcpwm_operator_timer(state, operator_index);
    mcpwm_timer_state_t *timer = &state->timer[timer_index];
    uint32_t base = mcpwm_operator_offset(operator_index);
    uint32_t dt_cfg = state->regs[(base + MCPWM_DT_CFG_REL) / 4u];
    uint32_t carrier = state->regs[(base + MCPWM_CARRIER_REL) / 4u];
    bool inverted = false;

    periph_mcpwm_output_info_t info;
    memset(&info, 0, sizeof(info));
    info.gpio = mcpwm_output_gpio(p, unit, operator_index, generator,
                                  &inverted);
    info.frequency_hz = mcpwm_timer_frequency_hz(p, unit, timer_index);
    info.period_ticks = mcpwm_timer_cycle(timer);
    info.compare_ticks = op->active_compare[generator];
    info.rising_delay_ticks = op->active_red;
    info.falling_delay_ticks = op->active_fed;
    info.deadtime_clock_hz = (dt_cfg & (1u << 17)) ?
        (mcpwm_group_frequency_hz(p, unit) /
         ((uint32_t)timer->active_prescale + 1u)) :
        mcpwm_group_frequency_hz(p, unit);
    if (carrier & 1u) {
        uint32_t carrier_div = ((carrier >> 1) & 0xFu) + 1u;
        info.carrier_hz = mcpwm_group_frequency_hz(p, unit) /
                          (carrier_div * 8u);
        info.carrier_duty_eighths = (carrier >> 5) & 7u;
    }
    info.count_mode = timer->mode;
    info.enabled = info.gpio >= 0 && mcpwm_unit_clocked(p, unit) &&
                   timer->running && timer->mode != 0u;
    info.inverted = inverted;
    info.fault_active = op->cbc_on || op->ost_on;
    info.forced_level = (int8_t)mcpwm_forced_level(op, generator);

    bool changed = !op->output_reported[generator] ||
                   !mcpwm_info_equal(&info, &op->last_info[generator]);
    if (!force && !changed) return;
    op->last_info[generator] = info;
    op->output_reported[generator] = true;
    if (op->output_cb[generator])
        op->output_cb[generator](op->output_cb_ctx[generator], (int)unit,
                                 (int)operator_index, (int)generator,
                                 &info);
}

static void mcpwm_emit_operator(esp32_periph_t *p, unsigned unit,
                                unsigned operator_index) {
    for (unsigned generator = 0; generator < MCPWM_GENERATOR_COUNT;
         generator++)
        mcpwm_emit_output(p, unit, operator_index, generator, false);
}

static void mcpwm_emit_timer_operators(esp32_periph_t *p, unsigned unit,
                                       unsigned timer_index) {
    mcpwm_unit_state_t *state = &p->mcpwm.unit[unit];
    for (unsigned operator_index = 0;
         operator_index < MCPWM_OPERATOR_COUNT; operator_index++) {
        if (mcpwm_operator_timer(state, operator_index) == timer_index)
            mcpwm_emit_operator(p, unit, operator_index);
    }
}

static void mcpwm_update_irq(esp32_periph_t *p, unsigned unit) {
    mcpwm_unit_state_t *state = &p->mcpwm.unit[unit];
    uint32_t raw = state->regs[MCPWM_INT_RAW_OFF / 4u];
    uint32_t ena = state->regs[MCPWM_INT_ENA_OFF / 4u];
    if (raw & ena)
        periph_assert_interrupt(p, mcpwm_unit_interrupt_source(unit));
    else
        periph_deassert_interrupt(p, mcpwm_unit_interrupt_source(unit));
}

static void mcpwm_kick(esp32_periph_t *p) {
    for (int core = 0; core < 2; core++)
        if (p->cpu[core]) xtensa_recompute_next_timer(p->cpu[core]);
}

static void mcpwm_apply_level_action(bool *level, unsigned action) {
    switch (action & 3u) {
    case 1u: *level = false; break;
    case 2u: *level = true; break;
    case 3u: *level = !*level; break;
    default: break;
    }
}

static bool mcpwm_operator_updates_enabled(const mcpwm_unit_state_t *state,
                                           unsigned operator_index) {
    uint32_t update = state->regs[MCPWM_UPDATE_CFG_OFF / 4u];
    return (update & 1u) != 0 &&
           (update & (1u << (2u + operator_index * 2u))) != 0;
}

static bool mcpwm_method_matches(uint32_t method, uint32_t event_bit,
                                 bool forced) {
    if (forced) return true;
    if (method & 8u) return false;
    return method == 0u || (method & event_bit) != 0;
}

static void mcpwm_transfer_operator(esp32_periph_t *p, unsigned unit,
                                     unsigned operator_index,
                                     uint32_t event_bit, bool forced) {
    mcpwm_unit_state_t *state = &p->mcpwm.unit[unit];
    mcpwm_operator_state_t *op = &state->operators[operator_index];
    if (!mcpwm_operator_updates_enabled(state, operator_index) && !forced)
        return;

    uint32_t base = mcpwm_operator_offset(operator_index);
    uint32_t stamp_cfg = state->regs[(base + MCPWM_GEN_STMP_CFG_REL) / 4u];
    for (unsigned compare = 0; compare < MCPWM_GENERATOR_COUNT; compare++) {
        uint32_t method = (stamp_cfg >> (compare * 4u)) & 0xFu;
        if (!op->compare_pending[compare] ||
            !mcpwm_method_matches(method, event_bit, forced))
            continue;
        op->active_compare[compare] = (uint16_t)
            state->regs[(base + MCPWM_GEN_TSTMP_A_REL + compare * 4u) / 4u];
        op->compare_pending[compare] = false;
        state->regs[(base + MCPWM_GEN_STMP_CFG_REL) / 4u] &=
            ~(1u << (8u + compare));
    }

    uint32_t generator_method =
        state->regs[(base + MCPWM_GEN_CFG0_REL) / 4u] & 0xFu;
    for (unsigned generator = 0; generator < MCPWM_GENERATOR_COUNT;
         generator++) {
        if (!op->generator_pending[generator] ||
            !mcpwm_method_matches(generator_method, event_bit, forced))
            continue;
        op->active_generator[generator] =
            state->regs[(base + MCPWM_GEN_A_REL + generator * 4u) / 4u] &
            0x00FFFFFFu;
        op->generator_pending[generator] = false;
    }

    uint32_t dt_cfg = state->regs[(base + MCPWM_DT_CFG_REL) / 4u];
    if (op->fed_pending &&
        mcpwm_method_matches(dt_cfg & 0xFu, event_bit, forced)) {
        op->active_fed = (uint16_t)
            state->regs[(base + MCPWM_DT_FED_REL) / 4u];
        op->fed_pending = false;
    }
    if (op->red_pending &&
        mcpwm_method_matches((dt_cfg >> 4) & 0xFu, event_bit, forced)) {
        op->active_red = (uint16_t)
            state->regs[(base + MCPWM_DT_RED_REL) / 4u];
        op->red_pending = false;
    }

    uint32_t force_reg = state->regs[(base + MCPWM_GEN_FORCE_REL) / 4u];
    uint32_t force_method = force_reg & 0x3Fu;
    bool force_match = forced ||
        (!(force_method & (1u << 5)) &&
         (force_method == 0u || (force_method & event_bit) != 0));
    if (op->force_pending && force_match) {
        op->active_force[0] = (force_reg >> 6) & 3u;
        op->active_force[1] = (force_reg >> 8) & 3u;
        op->force_pending = false;
    }
}

static void mcpwm_set_timer_count(mcpwm_timer_state_t *timer,
                                  uint32_t count, bool down) {
    uint32_t period = timer->active_period;
    if (count > period) count = period;
    if (timer->mode == 2u) {
        timer->phase = period - count;
    } else if (timer->mode == 3u && down && count != period && count != 0u) {
        timer->phase = period * 2u - count;
    } else {
        timer->phase = count;
    }
    uint32_t cycle = mcpwm_timer_cycle(timer);
    if (cycle) timer->phase %= cycle;
}

static void mcpwm_transfer_period(esp32_periph_t *p, unsigned unit,
                                  unsigned timer_index,
                                  uint32_t event_bit, bool forced) {
    mcpwm_unit_state_t *state = &p->mcpwm.unit[unit];
    mcpwm_timer_state_t *timer = &state->timer[timer_index];
    if (!timer->period_pending) return;
    uint32_t base = mcpwm_timer_offset(timer_index);
    uint32_t cfg0 = state->regs[(base + MCPWM_TIMER_CFG0_REL) / 4u];
    uint32_t method = (cfg0 >> 24) & 3u;
    if (!forced && method != 0u && !(method & event_bit)) return;

    uint32_t count = mcpwm_timer_count(timer);
    bool down = mcpwm_timer_down(timer);
    timer->active_period = (cfg0 >> 8) & 0xFFFFu;
    timer->period_pending = false;
    mcpwm_set_timer_count(timer, count, down);
}

static void mcpwm_flush_operator(esp32_periph_t *p, unsigned unit,
                                  unsigned operator_index) {
    mcpwm_transfer_operator(p, unit, operator_index, 0u, true);
    mcpwm_emit_operator(p, unit, operator_index);
}

static void mcpwm_flush_all(esp32_periph_t *p, unsigned unit) {
    for (unsigned timer = 0; timer < MCPWM_TIMER_COUNT; timer++)
        mcpwm_transfer_period(p, unit, timer, 0u, true);
    for (unsigned operator_index = 0;
         operator_index < MCPWM_OPERATOR_COUNT; operator_index++)
        mcpwm_flush_operator(p, unit, operator_index);
}

static unsigned mcpwm_generator_action(const mcpwm_operator_state_t *op,
                                        unsigned generator, bool down,
                                        unsigned event_index) {
    unsigned shift = event_index * 2u + (down ? 12u : 0u);
    return (op->active_generator[generator] >> shift) & 3u;
}

static void mcpwm_apply_generator_event(mcpwm_operator_state_t *op,
                                         unsigned event_index, bool down) {
    for (unsigned generator = 0; generator < MCPWM_GENERATOR_COUNT;
         generator++) {
        unsigned action = mcpwm_generator_action(op, generator, down,
                                                  event_index);
        mcpwm_apply_level_action(&op->generator_level[generator], action);
    }
}

static void mcpwm_apply_trigger_event(mcpwm_unit_state_t *state,
                                       unsigned operator_index,
                                       unsigned trigger, bool down) {
    mcpwm_operator_state_t *op = &state->operators[operator_index];
    mcpwm_apply_generator_event(op, 4u + trigger, down);
}

static void mcpwm_sync_timer(esp32_periph_t *p, unsigned unit,
                              unsigned timer_index) {
    mcpwm_unit_state_t *state = &p->mcpwm.unit[unit];
    mcpwm_timer_state_t *timer = &state->timer[timer_index];
    uint32_t base = mcpwm_timer_offset(timer_index);
    uint32_t sync = state->regs[(base + MCPWM_TIMER_SYNC_REL) / 4u];
    if (!(sync & 1u)) return;

    mcpwm_transfer_period(p, unit, timer_index, 2u, false);
    uint32_t phase = (sync >> 4) & 0xFFFFu;
    bool down = (sync & (1u << 20)) != 0;
    mcpwm_set_timer_count(timer, phase, down);
    timer->tick_remainder = 0;

    for (unsigned operator_index = 0;
         operator_index < MCPWM_OPERATOR_COUNT; operator_index++) {
        if (mcpwm_operator_timer(state, operator_index) != timer_index)
            continue;
        mcpwm_transfer_operator(p, unit, operator_index,
                                 MCPWM_UPDATE_EVENT_SYNC, false);
        uint32_t cfg0 = state->regs[(mcpwm_operator_offset(operator_index) +
                                     MCPWM_GEN_CFG0_REL) / 4u];
        bool direction = mcpwm_timer_down(timer);
        if (((cfg0 >> 4) & 7u) == 3u)
            mcpwm_apply_trigger_event(state, operator_index, 0u, direction);
        if (((cfg0 >> 7) & 7u) == 3u)
            mcpwm_apply_trigger_event(state, operator_index, 1u, direction);
        mcpwm_emit_operator(p, unit, operator_index);
    }
    mcpwm_emit_timer_operators(p, unit, timer_index);
}

static void mcpwm_propagate_timer_sync(esp32_periph_t *p, unsigned unit,
                                        unsigned source_timer) {
    mcpwm_unit_state_t *state = &p->mcpwm.unit[unit];
    uint32_t selectors = state->regs[MCPWM_TIMER_SYNCI_CFG_OFF / 4u];
    for (unsigned target = 0; target < MCPWM_TIMER_COUNT; target++) {
        unsigned selected = (selectors >> (target * 3u)) & 7u;
        if (selected == source_timer + 1u)
            mcpwm_sync_timer(p, unit, target);
    }
}

static bool mcpwm_timer_at_tez(const mcpwm_timer_state_t *timer) {
    return mcpwm_timer_count(timer) == 0u;
}

static bool mcpwm_timer_at_tep(const mcpwm_timer_state_t *timer) {
    return mcpwm_timer_count(timer) == timer->active_period;
}

static void mcpwm_timer_event(esp32_periph_t *p, unsigned unit,
                               unsigned timer_index) {
    mcpwm_unit_state_t *state = &p->mcpwm.unit[unit];
    mcpwm_timer_state_t *timer = &state->timer[timer_index];
    uint32_t raw = state->regs[MCPWM_INT_RAW_OFF / 4u];
    bool tez = mcpwm_timer_at_tez(timer);
    bool tep = mcpwm_timer_at_tep(timer);
    bool down = mcpwm_timer_down(timer);

    if (tez) {
        mcpwm_transfer_period(p, unit, timer_index, 1u, false);
        raw |= 1u << (3u + timer_index);
    }
    if (tep) raw |= 1u << (6u + timer_index);

    uint32_t count = mcpwm_timer_count(timer);
    for (unsigned operator_index = 0;
         operator_index < MCPWM_OPERATOR_COUNT; operator_index++) {
        if (mcpwm_operator_timer(state, operator_index) != timer_index)
            continue;
        mcpwm_operator_state_t *op = &state->operators[operator_index];
        if (tez)
            mcpwm_transfer_operator(p, unit, operator_index, 1u, false);
        if (tep)
            mcpwm_transfer_operator(p, unit, operator_index, 2u, false);
        if (tez) mcpwm_apply_generator_event(op, 0u, down);
        if (tep) mcpwm_apply_generator_event(op, 1u, down);
        for (unsigned compare = 0; compare < MCPWM_GENERATOR_COUNT;
             compare++) {
            if (op->active_compare[compare] > timer->active_period ||
                op->active_compare[compare] != count)
                continue;
            raw |= 1u << (15u + compare * 3u + operator_index);
            mcpwm_apply_generator_event(op, 2u + compare, down);
            mcpwm_transfer_operator(
                p, unit, operator_index,
                compare == 0u ? MCPWM_UPDATE_EVENT_TEA :
                                MCPWM_UPDATE_EVENT_TEB,
                false);
        }
        uint32_t fh_cfg1 = state->regs[
            (mcpwm_operator_offset(operator_index) + MCPWM_FH_CFG1_REL) / 4u];
        if (op->cbc_on && !mcpwm_operator_has_active_cbc_fault(
                state, operator_index) &&
            ((tez && (fh_cfg1 & (1u << 1))) ||
             (tep && (fh_cfg1 & (1u << 2)))))
            mcpwm_clear_operator_cbc(p, unit, operator_index);
        mcpwm_emit_operator(p, unit, operator_index);
    }

    bool stop = (tez && timer->stop_at_tez) ||
                (tep && timer->stop_at_tep);
    if (stop) {
        timer->running = false;
        timer->stop_at_tez = false;
        timer->stop_at_tep = false;
        raw |= 1u << timer_index;
    }
    state->regs[MCPWM_INT_RAW_OFF / 4u] = raw & MCPWM_INT_VALID_MASK;

    uint32_t sync = state->regs[(mcpwm_timer_offset(timer_index) +
                                 MCPWM_TIMER_SYNC_REL) / 4u];
    unsigned sync_out = (sync >> 2) & 3u;
    if ((tez && sync_out == 1u) || (tep && sync_out == 2u))
        mcpwm_propagate_timer_sync(p, unit, timer_index);

    mcpwm_update_irq(p, unit);
    mcpwm_emit_timer_operators(p, unit, timer_index);
}

static void mcpwm_consider_phase(uint32_t current, uint32_t cycle,
                                  uint32_t candidate, uint32_t *best) {
    if (!cycle || candidate >= cycle) return;
    uint32_t distance = candidate >= current ? candidate - current :
                        cycle - current + candidate;
    if (distance == 0u) distance = cycle;
    if (distance < *best) *best = distance;
}

static uint32_t mcpwm_timer_next_event_ticks(
    const mcpwm_unit_state_t *state, unsigned timer_index) {
    const mcpwm_timer_state_t *timer = &state->timer[timer_index];
    uint32_t cycle = mcpwm_timer_cycle(timer);
    uint32_t current = cycle ? timer->phase % cycle : 0u;
    uint32_t best = UINT32_MAX;
    uint32_t period = timer->active_period;

    /* TEZ and TEP phase positions. */
    if (timer->mode == 2u) {
        mcpwm_consider_phase(current, cycle, period, &best); /* TEZ */
        mcpwm_consider_phase(current, cycle, 0u, &best);    /* TEP */
    } else {
        mcpwm_consider_phase(current, cycle, 0u, &best);    /* TEZ */
        mcpwm_consider_phase(current, cycle, period, &best);/* TEP */
    }

    for (unsigned operator_index = 0;
         operator_index < MCPWM_OPERATOR_COUNT; operator_index++) {
        if (mcpwm_operator_timer(state, operator_index) != timer_index)
            continue;
        const mcpwm_operator_state_t *op =
            &state->operators[operator_index];
        for (unsigned compare = 0; compare < MCPWM_GENERATOR_COUNT;
             compare++) {
            uint32_t value = op->active_compare[compare];
            if (value > period) continue;
            if (timer->mode == 2u) {
                mcpwm_consider_phase(current, cycle, period - value, &best);
            } else if (timer->mode == 3u && period != 0u) {
                mcpwm_consider_phase(current, cycle, value, &best);
                uint32_t down_phase = period * 2u - value;
                if (down_phase < cycle)
                    mcpwm_consider_phase(current, cycle, down_phase, &best);
            } else {
                mcpwm_consider_phase(current, cycle, value, &best);
            }
        }
    }
    return best == UINT32_MAX ? 1u : best;
}

static bool mcpwm_timer_can_skip_cycles(const mcpwm_unit_state_t *state,
                                         unsigned timer_index) {
    const mcpwm_timer_state_t *timer = &state->timer[timer_index];
    if (timer->period_pending || timer->stop_at_tez || timer->stop_at_tep)
        return false;
    uint32_t selectors = state->regs[MCPWM_TIMER_SYNCI_CFG_OFF / 4u];
    uint32_t sync = state->regs[(mcpwm_timer_offset(timer_index) +
                                 MCPWM_TIMER_SYNC_REL) / 4u];
    unsigned sync_out = (sync >> 2) & 3u;
    if (sync_out == 1u || sync_out == 2u) {
        for (unsigned target = 0; target < MCPWM_TIMER_COUNT; target++) {
            if (((selectors >> (target * 3u)) & 7u) == timer_index + 1u)
                return false;
        }
    }
    for (unsigned operator_index = 0;
         operator_index < MCPWM_OPERATOR_COUNT; operator_index++) {
        if (mcpwm_operator_timer(state, operator_index) != timer_index)
            continue;
        const mcpwm_operator_state_t *op =
            &state->operators[operator_index];
        if (op->compare_pending[0] || op->compare_pending[1] ||
            op->generator_pending[0] || op->generator_pending[1] ||
            op->fed_pending || op->red_pending || op->force_pending)
            return false;
        for (unsigned generator = 0; generator < MCPWM_GENERATOR_COUNT;
             generator++) {
            uint32_t actions = op->active_generator[generator];
            for (unsigned field = 0; field < 12u; field++)
                if (((actions >> (field * 2u)) & 3u) == 3u)
                    return false;
        }
    }
    return true;
}

static void mcpwm_advance_timer_ticks(esp32_periph_t *p, unsigned unit,
                                       unsigned timer_index,
                                       uint64_t ticks) {
    mcpwm_unit_state_t *state = &p->mcpwm.unit[unit];
    mcpwm_timer_state_t *timer = &state->timer[timer_index];
    uint32_t cycle = mcpwm_timer_cycle(timer);
    if (cycle && ticks > (uint64_t)cycle * 2u &&
        mcpwm_timer_can_skip_cycles(state, timer_index)) {
        uint64_t complete = ticks / cycle;
        if (complete > 1u) ticks -= (complete - 1u) * cycle;
    }

    unsigned events = 0;
    while (ticks != 0u && timer->running && timer->mode != 0u) {
        uint32_t distance = mcpwm_timer_next_event_ticks(state, timer_index);
        if ((uint64_t)distance > ticks) {
            uint32_t active_cycle = mcpwm_timer_cycle(timer);
            timer->phase = active_cycle ?
                (uint32_t)((timer->phase + ticks) % active_cycle) : 0u;
            break;
        }
        uint32_t active_cycle = mcpwm_timer_cycle(timer);
        timer->phase = active_cycle ?
            (timer->phase + distance) % active_cycle : 0u;
        ticks -= distance;
        mcpwm_timer_event(p, unit, timer_index);
        if (++events > 1000000u) {
            /* Pathological multi-second jumps with toggle-heavy waveforms
             * should not monopolize the emulator. Preserve phase and latch
             * all event classes by draining one final cycle. */
            active_cycle = mcpwm_timer_cycle(timer);
            if (active_cycle && ticks > active_cycle)
                ticks %= active_cycle;
            events = 0;
        }
    }
}

static void mcpwm_sync_timer_to(esp32_periph_t *p, unsigned unit,
                                 unsigned timer_index, uint64_t now) {
    mcpwm_timer_state_t *timer =
        &p->mcpwm.unit[unit].timer[timer_index];
    uint64_t elapsed = now >= timer->last_cycles ?
                       now - timer->last_cycles : 0u;
    timer->last_cycles = now;
    if (!timer->running || timer->mode == 0u ||
        !mcpwm_unit_clocked(p, unit)) {
        timer->tick_remainder = 0;
        return;
    }
    uint64_t denominator = mcpwm_timer_denominator(p, unit, timer);
    if (!denominator) return;
    uint64_t product = elapsed > (UINT64_MAX - timer->tick_remainder) /
                                 MCPWM_SOURCE_CLOCK_MHZ ? UINT64_MAX :
        timer->tick_remainder + elapsed * MCPWM_SOURCE_CLOCK_MHZ;
    uint64_t ticks = product / denominator;
    timer->tick_remainder = product % denominator;
    if (ticks) mcpwm_advance_timer_ticks(p, unit, timer_index, ticks);
}

static uint64_t mcpwm_capture_denominator(const esp32_periph_t *p) {
    return mcpwm_cpu_mhz(p);
}

static void mcpwm_sync_capture_to(esp32_periph_t *p, unsigned unit,
                                   uint64_t now) {
    mcpwm_unit_state_t *state = &p->mcpwm.unit[unit];
    uint64_t elapsed = now >= state->capture_last_cycles ?
                       now - state->capture_last_cycles : 0u;
    state->capture_last_cycles = now;
    if (!(state->regs[MCPWM_CAP_TIMER_CFG_OFF / 4u] & 1u) ||
        !mcpwm_unit_clocked(p, unit)) {
        state->capture_remainder = 0;
        return;
    }
    uint64_t denominator = mcpwm_capture_denominator(p);
    uint64_t product = elapsed > (UINT64_MAX - state->capture_remainder) /
                                 MCPWM_CAPTURE_CLOCK_MHZ ? UINT64_MAX :
        state->capture_remainder + elapsed * MCPWM_CAPTURE_CLOCK_MHZ;
    uint64_t ticks = denominator ? product / denominator : 0u;
    state->capture_remainder = denominator ? product % denominator : 0u;
    state->capture_counter += (uint32_t)ticks;
}

static bool mcpwm_fault_is_active(const mcpwm_unit_state_t *state,
                                   unsigned fault) {
    return (state->regs[MCPWM_FAULT_DETECT_OFF / 4u] &
            (1u << (6u + fault))) != 0;
}

static bool mcpwm_operator_has_active_cbc_fault(
    const mcpwm_unit_state_t *state, unsigned operator_index) {
    uint32_t cfg0 = state->regs[(mcpwm_operator_offset(operator_index) +
                                 MCPWM_FH_CFG0_REL) / 4u];
    for (unsigned fault = 0; fault < MCPWM_TIMER_COUNT; fault++) {
        if (mcpwm_fault_is_active(state, fault) &&
            (cfg0 & (1u << (3u - fault))))
            return true;
    }
    return false;
}

static void mcpwm_set_fault_override(mcpwm_operator_state_t *op,
                                      unsigned generator, bool oneshot,
                                      unsigned action) {
    bool *valid = oneshot ? &op->ost_override_valid[generator] :
                            &op->cbc_override_valid[generator];
    bool *level = oneshot ? &op->ost_override_level[generator] :
                            &op->cbc_override_level[generator];
    if (action == 0u) {
        *valid = false;
        return;
    }
    bool current = op->generator_level[generator];
    int forced = mcpwm_forced_level(op, generator);
    if (forced >= 0) current = forced != 0;
    if (action == 1u)
        current = false;
    else if (action == 2u)
        current = true;
    else
        current = !current;
    *valid = true;
    *level = current;
}

static unsigned mcpwm_fault_action(uint32_t cfg0, unsigned generator,
                                    bool oneshot, bool down) {
    unsigned shift = 8u + generator * 8u + (oneshot ? 4u : 0u) +
                     (down ? 0u : 2u);
    return (cfg0 >> shift) & 3u;
}

static void mcpwm_activate_operator_fault(esp32_periph_t *p, unsigned unit,
                                           unsigned operator_index,
                                           bool oneshot) {
    mcpwm_unit_state_t *state = &p->mcpwm.unit[unit];
    mcpwm_operator_state_t *op = &state->operators[operator_index];
    unsigned timer_index = mcpwm_operator_timer(state, operator_index);
    bool down = mcpwm_timer_down(&state->timer[timer_index]);
    uint32_t cfg0 = state->regs[(mcpwm_operator_offset(operator_index) +
                                 MCPWM_FH_CFG0_REL) / 4u];
    if (oneshot)
        op->ost_on = true;
    else
        op->cbc_on = true;
    for (unsigned generator = 0; generator < MCPWM_GENERATOR_COUNT;
         generator++) {
        unsigned action = mcpwm_fault_action(cfg0, generator, oneshot, down);
        mcpwm_set_fault_override(op, generator, oneshot, action);
    }
    state->regs[MCPWM_INT_RAW_OFF / 4u] |=
        1u << ((oneshot ? 24u : 21u) + operator_index);
    mcpwm_emit_operator(p, unit, operator_index);
}

static void mcpwm_clear_operator_cbc(esp32_periph_t *p, unsigned unit,
                                      unsigned operator_index) {
    mcpwm_operator_state_t *op =
        &p->mcpwm.unit[unit].operators[operator_index];
    op->cbc_on = false;
    for (unsigned generator = 0; generator < MCPWM_GENERATOR_COUNT;
         generator++)
        op->cbc_override_valid[generator] = false;
    mcpwm_emit_operator(p, unit, operator_index);
}

static void mcpwm_clear_operator_ost(esp32_periph_t *p, unsigned unit,
                                      unsigned operator_index) {
    mcpwm_operator_state_t *op =
        &p->mcpwm.unit[unit].operators[operator_index];
    op->ost_on = false;
    for (unsigned generator = 0; generator < MCPWM_GENERATOR_COUNT;
         generator++)
        op->ost_override_valid[generator] = false;
    mcpwm_emit_operator(p, unit, operator_index);
}

static void mcpwm_fault_transition(esp32_periph_t *p, unsigned unit,
                                    unsigned fault, bool active) {
    mcpwm_unit_state_t *state = &p->mcpwm.unit[unit];
    uint32_t status_bit = 1u << (6u + fault);
    bool before = (state->regs[MCPWM_FAULT_DETECT_OFF / 4u] &
                   status_bit) != 0;
    if (before == active) return;

    if (active) {
        state->regs[MCPWM_FAULT_DETECT_OFF / 4u] |= status_bit;
        state->regs[MCPWM_INT_RAW_OFF / 4u] |= 1u << (9u + fault);
    } else {
        state->regs[MCPWM_FAULT_DETECT_OFF / 4u] &= ~status_bit;
        state->regs[MCPWM_INT_RAW_OFF / 4u] |= 1u << (12u + fault);
    }

    for (unsigned operator_index = 0;
         operator_index < MCPWM_OPERATOR_COUNT; operator_index++) {
        uint32_t base = mcpwm_operator_offset(operator_index);
        uint32_t cfg0 = state->regs[(base + MCPWM_FH_CFG0_REL) / 4u];
        if (active) {
            if (cfg0 & (1u << (3u - fault)))
                mcpwm_activate_operator_fault(p, unit, operator_index, false);
            if (cfg0 & (1u << (7u - fault)))
                mcpwm_activate_operator_fault(p, unit, operator_index, true);

            uint32_t gen_cfg =
                state->regs[(base + MCPWM_GEN_CFG0_REL) / 4u];
            bool down = mcpwm_timer_down(
                &state->timer[mcpwm_operator_timer(state, operator_index)]);
            if (((gen_cfg >> 4) & 7u) == fault)
                mcpwm_apply_trigger_event(state, operator_index, 0u, down);
            if (((gen_cfg >> 7) & 7u) == fault)
                mcpwm_apply_trigger_event(state, operator_index, 1u, down);
        } else if (!mcpwm_operator_has_active_cbc_fault(state,
                                                         operator_index)) {
            mcpwm_clear_operator_cbc(p, unit, operator_index);
        }
        mcpwm_emit_operator(p, unit, operator_index);
    }
    mcpwm_update_irq(p, unit);
}

static void mcpwm_refresh_fault(esp32_periph_t *p, unsigned unit,
                                 unsigned fault) {
    mcpwm_unit_state_t *state = &p->mcpwm.unit[unit];
    uint32_t detect = state->regs[MCPWM_FAULT_DETECT_OFF / 4u];
    bool enabled = (detect & (1u << fault)) != 0;
    bool active_level = (detect & (1u << (3u + fault))) != 0;
    bool active = enabled && state->fault_level[fault] == active_level;
    mcpwm_fault_transition(p, unit, fault, active);
}

static void mcpwm_capture_event(esp32_periph_t *p, unsigned unit,
                                 unsigned channel, bool negative) {
    mcpwm_unit_state_t *state = &p->mcpwm.unit[unit];
    uint32_t cfg = state->regs[(MCPWM_CAP_CH_CFG_OFF + channel * 4u) / 4u];
    if (!(cfg & 1u)) return;
    bool allowed = negative ? (cfg & (1u << 1)) != 0 :
                              (cfg & (1u << 2)) != 0;
    if (!allowed) return;
    uint32_t divisor = ((cfg >> 3) & 0xFFu) + 1u;
    uint16_t count = ++state->capture_prescale_count[channel];
    if (count < divisor) return;
    state->capture_prescale_count[channel] = 0;
    state->regs[(MCPWM_CAP_CH_VALUE_OFF + channel * 4u) / 4u] =
        state->capture_counter;
    if (negative)
        state->regs[MCPWM_CAP_STATUS_OFF / 4u] |= 1u << channel;
    else
        state->regs[MCPWM_CAP_STATUS_OFF / 4u] &= ~(1u << channel);
    state->regs[MCPWM_INT_RAW_OFF / 4u] |= 1u << (27u + channel);
    mcpwm_update_irq(p, unit);
}

static void mcpwm_external_sync_event(esp32_periph_t *p, unsigned unit,
                                       unsigned channel) {
    mcpwm_unit_state_t *state = &p->mcpwm.unit[unit];
    uint32_t selectors = state->regs[MCPWM_TIMER_SYNCI_CFG_OFF / 4u];
    for (unsigned timer = 0; timer < MCPWM_TIMER_COUNT; timer++) {
        if (((selectors >> (timer * 3u)) & 7u) == channel + 4u)
            mcpwm_sync_timer(p, unit, timer);
    }
    uint32_t capture_cfg = state->regs[MCPWM_CAP_TIMER_CFG_OFF / 4u];
    if ((capture_cfg & (1u << 1)) &&
        ((capture_cfg >> 2) & 7u) == channel + 4u) {
        state->capture_counter =
            state->regs[MCPWM_CAP_TIMER_PHASE_OFF / 4u];
        state->capture_remainder = 0;
    }
}

static bool mcpwm_effective_sync_level(const esp32_periph_t *p,
                                        unsigned unit, unsigned channel) {
    bool level = mcpwm_matrix_input_level(p,
        mcpwm_sync_signal(unit, channel));
    uint32_t cfg =
        p->mcpwm.unit[unit].regs[MCPWM_TIMER_SYNCI_CFG_OFF / 4u];
    if (cfg & (1u << (9u + channel))) level = !level;
    return level;
}

static bool mcpwm_effective_capture_level(const esp32_periph_t *p,
                                           unsigned unit,
                                           unsigned channel) {
    bool level = mcpwm_matrix_input_level(p,
        mcpwm_capture_signal(unit, channel));
    uint32_t cfg = p->mcpwm.unit[unit].regs[
        (MCPWM_CAP_CH_CFG_OFF + channel * 4u) / 4u];
    if (cfg & (1u << 11)) level = !level;
    return level;
}

static void mcpwm_handle_input_signal(esp32_periph_t *p, unsigned unit,
                                       unsigned kind, unsigned channel) {
    mcpwm_unit_state_t *state = &p->mcpwm.unit[unit];
    uint64_t now = mcpwm_now_cycles(p);
    mcpwm_sync_capture_to(p, unit, now);
    for (unsigned timer = 0; timer < MCPWM_TIMER_COUNT; timer++)
        mcpwm_sync_timer_to(p, unit, timer, now);
    if (kind == 0u) {
        bool level = mcpwm_effective_sync_level(p, unit, channel);
        bool before = state->sync_level[channel];
        state->sync_level[channel] = level;
        if (!before && level) mcpwm_external_sync_event(p, unit, channel);
    } else if (kind == 1u) {
        bool level = mcpwm_matrix_input_level(p,
            mcpwm_fault_signal(unit, channel));
        state->fault_level[channel] = level;
        mcpwm_refresh_fault(p, unit, channel);
    } else {
        bool level = mcpwm_effective_capture_level(p, unit, channel);
        bool before = state->capture_level[channel];
        state->capture_level[channel] = level;
        if (before != level) mcpwm_capture_event(p, unit, channel, !level);
    }
    mcpwm_kick(p);
}

static void mcpwm_rebind_input_signal(esp32_periph_t *p, unsigned unit,
                                       unsigned kind, unsigned channel) {
    mcpwm_unit_state_t *state = &p->mcpwm.unit[unit];
    if (kind == 0u)
        state->sync_level[channel] =
            mcpwm_effective_sync_level(p, unit, channel);
    else if (kind == 1u)
        state->fault_level[channel] = mcpwm_matrix_input_level(
            p, mcpwm_fault_signal(unit, channel));
    else
        state->capture_level[channel] =
            mcpwm_effective_capture_level(p, unit, channel);
}

static void mcpwm_gpio_input_route_changed(esp32_periph_t *p,
                                            unsigned signal) {
    unsigned unit;
    unsigned kind;
    unsigned channel;
    if (!mcpwm_decode_input_signal(signal, &unit, &kind, &channel)) return;
    mcpwm_rebind_input_signal(p, unit, kind, channel);
    if (kind == 1u) mcpwm_refresh_fault(p, unit, channel);
    mcpwm_kick(p);
}

static void mcpwm_gpio_input_changed(esp32_periph_t *p, int gpio) {
    for (unsigned unit = 0; unit < MCPWM_UNIT_COUNT; unit++) {
        for (unsigned channel = 0; channel < MCPWM_TIMER_COUNT; channel++) {
            uint32_t signals[3] = {
                mcpwm_sync_signal(unit, channel),
                mcpwm_fault_signal(unit, channel),
                mcpwm_capture_signal(unit, channel),
            };
            for (unsigned kind = 0; kind < 3u; kind++) {
                if (mcpwm_matrix_input_gpio(p, signals[kind]) == gpio)
                    mcpwm_handle_input_signal(p, unit, kind, channel);
            }
        }
    }
}

static void mcpwm_gpio_output_route_changed(esp32_periph_t *p, int gpio,
                                             uint32_t before,
                                             uint32_t after) {
    (void)gpio;
    uint32_t signals[2] = {before & 0x1FFu, after & 0x1FFu};
    for (unsigned which = 0; which < 2u; which++) {
        for (unsigned unit = 0; unit < MCPWM_UNIT_COUNT; unit++) {
            uint32_t base = mcpwm_output_signal(unit, 0, 0);
            if (signals[which] < base || signals[which] >= base + 6u)
                continue;
            unsigned relative = signals[which] - base;
            mcpwm_emit_output(p, unit, relative / 2u, relative & 1u,
                               false);
        }
    }
}

static bool mcpwm_generator_pin_level(const esp32_periph_t *p,
                                       unsigned unit,
                                       unsigned operator_index,
                                       unsigned generator) {
    const mcpwm_unit_state_t *state = &p->mcpwm.unit[unit];
    const mcpwm_operator_state_t *op = &state->operators[operator_index];
    bool levels[2] = {op->generator_level[0], op->generator_level[1]};
    for (unsigned index = 0; index < 2u; index++) {
        int forced = mcpwm_forced_level(op, index);
        if (forced >= 0) levels[index] = forced != 0;
    }
    uint32_t dt = state->regs[(mcpwm_operator_offset(operator_index) +
                               MCPWM_DT_CFG_REL) / 4u];
    bool paths[2];
    paths[0] = (dt & (1u << 15)) ? levels[0] :
        (levels[(dt >> 11) & 1u] ^ ((dt & (1u << 13)) != 0));
    paths[1] = (dt & (1u << 16)) ? levels[1] :
        (levels[(dt >> 12) & 1u] ^ ((dt & (1u << 14)) != 0));
    bool outputs[2] = {
        (dt & (1u << 9)) ? paths[1] : paths[0],
        (dt & (1u << 10)) ? paths[0] : paths[1],
    };
    return outputs[generator];
}

static void mcpwm_sync_unit_to(esp32_periph_t *p, unsigned unit,
                                uint64_t now) {
    mcpwm_sync_capture_to(p, unit, now);
    for (unsigned timer = 0; timer < MCPWM_TIMER_COUNT; timer++)
        mcpwm_sync_timer_to(p, unit, timer, now);
}

static void mcpwm_sync_all_to(esp32_periph_t *p, uint64_t now) {
    for (unsigned unit = 0; unit < MCPWM_UNIT_COUNT; unit++)
        mcpwm_sync_unit_to(p, unit, now);
}

static void mcpwm_start_timer(esp32_periph_t *p, unsigned unit,
                               unsigned timer_index, uint32_t command,
                               uint64_t now) {
    mcpwm_unit_state_t *state = &p->mcpwm.unit[unit];
    mcpwm_timer_state_t *timer = &state->timer[timer_index];
    uint32_t base = mcpwm_timer_offset(timer_index);
    uint32_t cfg0 = state->regs[(base + MCPWM_TIMER_CFG0_REL) / 4u];
    bool was_running = timer->running;

    timer->stop_at_tez = false;
    timer->stop_at_tep = false;
    if (command == 0u) {
        if (timer->running) timer->stop_at_tez = true;
    } else if (command == 1u) {
        if (timer->running) timer->stop_at_tep = true;
    } else if (command >= 2u && command <= 4u && timer->mode != 0u) {
        timer->running = true;
        timer->stop_at_tez = command == 3u;
        timer->stop_at_tep = command == 4u;
    }

    if (!was_running && timer->running) {
        timer->active_prescale = cfg0 & 0xFFu;
        timer->last_cycles = now;
        timer->tick_remainder = 0;
        if (timer->mode == 2u)
            mcpwm_set_timer_count(timer, timer->active_period, true);
        else
            mcpwm_set_timer_count(timer, 0u, false);
        /* Starting on a boundary makes generator TEZ/TEP actions visible
         * immediately, matching the hardware's first PWM cycle. */
        mcpwm_timer_event(p, unit, timer_index);
    }
    if (timer->mode == 0u) timer->running = false;
    mcpwm_emit_timer_operators(p, unit, timer_index);
}

static void mcpwm_write_timer(esp32_periph_t *p, unsigned unit,
                               unsigned timer_index, uint32_t relative,
                               uint32_t val, uint64_t now) {
    mcpwm_unit_state_t *state = &p->mcpwm.unit[unit];
    mcpwm_timer_state_t *timer = &state->timer[timer_index];
    uint32_t base = mcpwm_timer_offset(timer_index);
    if (relative == MCPWM_TIMER_CFG0_REL) {
        state->regs[base / 4u] = val & 0x03FFFFFFu;
        timer->period_pending = true;
        if (!timer->running) timer->active_prescale = val & 0xFFu;
        mcpwm_transfer_period(p, unit, timer_index, 0u, false);
        mcpwm_emit_timer_operators(p, unit, timer_index);
        return;
    }
    if (relative == MCPWM_TIMER_CFG1_REL) {
        uint32_t count = mcpwm_timer_count(timer);
        bool down = mcpwm_timer_down(timer);
        state->regs[(base + relative) / 4u] = val & 0x1Fu;
        timer->mode = (val >> 3) & 3u;
        mcpwm_set_timer_count(timer, count, down);
        mcpwm_start_timer(p, unit, timer_index, val & 7u, now);
        return;
    }
    if (relative == MCPWM_TIMER_SYNC_REL) {
        uint32_t old = state->regs[(base + relative) / 4u];
        state->regs[(base + relative) / 4u] = val & 0x001FFFFFu;
        if (((old ^ val) & (1u << 1)) != 0)
            mcpwm_sync_timer(p, unit, timer_index);
        mcpwm_kick(p);
    }
}

static void mcpwm_trigger_noncontinuous_force(
    esp32_periph_t *p, unsigned unit, unsigned operator_index,
    unsigned generator, unsigned mode) {
    mcpwm_operator_state_t *op =
        &p->mcpwm.unit[unit].operators[operator_index];
    if (mode == 1u)
        op->generator_level[generator] = false;
    else if (mode == 2u)
        op->generator_level[generator] = true;
    mcpwm_emit_output(p, unit, operator_index, generator, false);
}

static void mcpwm_write_operator(esp32_periph_t *p, unsigned unit,
                                  unsigned operator_index,
                                  uint32_t relative, uint32_t val) {
    mcpwm_unit_state_t *state = &p->mcpwm.unit[unit];
    mcpwm_operator_state_t *op = &state->operators[operator_index];
    uint32_t base = mcpwm_operator_offset(operator_index);
    uint32_t index = (base + relative) / 4u;

    switch (relative) {
    case MCPWM_GEN_STMP_CFG_REL:
        state->regs[index] = (val & 0xFFu) |
            (state->regs[index] & 0x300u);
        mcpwm_transfer_operator(p, unit, operator_index, 0u, false);
        break;
    case MCPWM_GEN_TSTMP_A_REL:
    case MCPWM_GEN_TSTMP_B_REL: {
        unsigned compare = (relative - MCPWM_GEN_TSTMP_A_REL) / 4u;
        state->regs[index] = val & 0xFFFFu;
        op->compare_pending[compare] = true;
        state->regs[(base + MCPWM_GEN_STMP_CFG_REL) / 4u] |=
            1u << (8u + compare);
        mcpwm_transfer_operator(p, unit, operator_index, 0u, false);
        break;
    }
    case MCPWM_GEN_CFG0_REL:
        state->regs[index] = val & 0x3FFu;
        break;
    case MCPWM_GEN_FORCE_REL: {
        uint32_t old = state->regs[index];
        state->regs[index] = val & 0xFFFFu;
        op->force_pending = true;
        mcpwm_transfer_operator(p, unit, operator_index, 0u, false);
        if ((old ^ val) & (1u << 10))
            mcpwm_trigger_noncontinuous_force(
                p, unit, operator_index, 0u, (val >> 11) & 3u);
        if ((old ^ val) & (1u << 13))
            mcpwm_trigger_noncontinuous_force(
                p, unit, operator_index, 1u, (val >> 14) & 3u);
        break;
    }
    case MCPWM_GEN_A_REL:
    case MCPWM_GEN_B_REL: {
        unsigned generator = (relative - MCPWM_GEN_A_REL) / 4u;
        state->regs[index] = val & 0x00FFFFFFu;
        op->generator_pending[generator] = true;
        mcpwm_transfer_operator(p, unit, operator_index, 0u, false);
        break;
    }
    case MCPWM_DT_CFG_REL:
        state->regs[index] = val & 0x3FFFFu;
        mcpwm_transfer_operator(p, unit, operator_index, 0u, false);
        break;
    case MCPWM_DT_FED_REL:
        state->regs[index] = val & 0xFFFFu;
        op->fed_pending = true;
        mcpwm_transfer_operator(p, unit, operator_index, 0u, false);
        break;
    case MCPWM_DT_RED_REL:
        state->regs[index] = val & 0xFFFFu;
        op->red_pending = true;
        mcpwm_transfer_operator(p, unit, operator_index, 0u, false);
        break;
    case MCPWM_CARRIER_REL:
        state->regs[index] = val & 0x3FFFu;
        break;
    case MCPWM_FH_CFG0_REL:
        state->regs[index] = val & 0x00FFFFFFu;
        break;
    case MCPWM_FH_CFG1_REL: {
        uint32_t old = state->regs[index];
        state->regs[index] = val & 0x1Fu;
        if (!(old & 1u) && (val & 1u))
            mcpwm_clear_operator_ost(p, unit, operator_index);
        uint32_t cfg0 = state->regs[(base + MCPWM_FH_CFG0_REL) / 4u];
        if (((old ^ val) & (1u << 3)) && (cfg0 & 1u))
            mcpwm_activate_operator_fault(p, unit, operator_index, false);
        if (((old ^ val) & (1u << 4)) && (cfg0 & (1u << 4)))
            mcpwm_activate_operator_fault(p, unit, operator_index, true);
        break;
    }
    default:
        return;
    }
    mcpwm_emit_operator(p, unit, operator_index);
    mcpwm_update_irq(p, unit);
    mcpwm_kick(p);
}

static void mcpwm_software_capture(esp32_periph_t *p, unsigned unit,
                                    unsigned channel) {
    mcpwm_unit_state_t *state = &p->mcpwm.unit[unit];
    if (!(state->regs[(MCPWM_CAP_CH_CFG_OFF + channel * 4u) / 4u] & 1u))
        return;
    state->capture_prescale_count[channel] = 0;
    state->regs[(MCPWM_CAP_CH_VALUE_OFF + channel * 4u) / 4u] =
        state->capture_counter;
    state->regs[MCPWM_CAP_STATUS_OFF / 4u] &= ~(1u << channel);
    state->regs[MCPWM_INT_RAW_OFF / 4u] |= 1u << (27u + channel);
    mcpwm_update_irq(p, unit);
}

static uint32_t mcpwm_read(void *ctx, uint32_t addr) {
    esp32_periph_t *p = ctx;
    unsigned unit = mcpwm_addr_unit(addr);
    uint32_t off = addr - mcpwm_unit_base(unit);
    if ((off & 3u) || off > MCPWM_VERSION_OFF)
        return default_read(ctx, addr);
    uint64_t now = mcpwm_now_cycles(p);
    mcpwm_sync_unit_to(p, unit, now);
    mcpwm_unit_state_t *state = &p->mcpwm.unit[unit];

    if (off >= MCPWM_TIMER_BASE_OFF && off < MCPWM_TIMER_SYNCI_CFG_OFF) {
        unsigned timer_index = (off - MCPWM_TIMER_BASE_OFF) /
                               MCPWM_TIMER_STRIDE;
        uint32_t relative = (off - MCPWM_TIMER_BASE_OFF) %
                            MCPWM_TIMER_STRIDE;
        if (relative == MCPWM_TIMER_STATUS_REL) {
            mcpwm_timer_state_t *timer = &state->timer[timer_index];
            return mcpwm_timer_count(timer) |
                   (mcpwm_timer_down(timer) ? 1u << 16 : 0u);
        }
    }
    if (off >= MCPWM_OPERATOR_BASE_OFF && off < MCPWM_FAULT_DETECT_OFF) {
        unsigned operator_index = (off - MCPWM_OPERATOR_BASE_OFF) /
                                  MCPWM_OPERATOR_STRIDE;
        uint32_t relative = (off - MCPWM_OPERATOR_BASE_OFF) %
                            MCPWM_OPERATOR_STRIDE;
        if (relative == MCPWM_FH_STATUS_REL) {
            mcpwm_operator_state_t *op =
                &state->operators[operator_index];
            return (op->cbc_on ? 1u : 0u) | (op->ost_on ? 2u : 0u);
        }
    }
    if (off == MCPWM_INT_ST_OFF)
        return state->regs[MCPWM_INT_RAW_OFF / 4u] &
               state->regs[MCPWM_INT_ENA_OFF / 4u];
    if (off == MCPWM_INT_CLR_OFF) return 0;
    return state->regs[off / 4u];
}

static void mcpwm_write(void *ctx, uint32_t addr, uint32_t val) {
    esp32_periph_t *p = ctx;
    unsigned unit = mcpwm_addr_unit(addr);
    uint32_t off = addr - mcpwm_unit_base(unit);
    if ((off & 3u) || off > MCPWM_VERSION_OFF) {
        default_write(ctx, addr, val);
        return;
    }
    uint64_t now = mcpwm_now_cycles(p);
    mcpwm_sync_unit_to(p, unit, now);
    mcpwm_unit_state_t *state = &p->mcpwm.unit[unit];

    if (off == 0u) {
        state->regs[0] = val & 0xFFu;
        for (unsigned timer = 0; timer < MCPWM_TIMER_COUNT; timer++) {
            state->timer[timer].tick_remainder = 0;
            state->timer[timer].last_cycles = now;
            mcpwm_emit_timer_operators(p, unit, timer);
        }
        mcpwm_kick(p);
        return;
    }
    if (off >= MCPWM_TIMER_BASE_OFF && off < MCPWM_TIMER_SYNCI_CFG_OFF) {
        unsigned timer_index = (off - MCPWM_TIMER_BASE_OFF) /
                               MCPWM_TIMER_STRIDE;
        uint32_t relative = (off - MCPWM_TIMER_BASE_OFF) %
                            MCPWM_TIMER_STRIDE;
        if (relative == MCPWM_TIMER_STATUS_REL) return;
        mcpwm_write_timer(p, unit, timer_index, relative, val, now);
        mcpwm_kick(p);
        return;
    }
    if (off == MCPWM_TIMER_SYNCI_CFG_OFF) {
        state->regs[off / 4u] = val & 0xFFFu;
        for (unsigned channel = 0; channel < MCPWM_TIMER_COUNT; channel++)
            mcpwm_rebind_input_signal(p, unit, 0u, channel);
        mcpwm_kick(p);
        return;
    }
    if (off == MCPWM_OPERATOR_TIMERSEL_OFF) {
        state->regs[off / 4u] = val & 0x3Fu;
        for (unsigned operator_index = 0;
             operator_index < MCPWM_OPERATOR_COUNT; operator_index++)
            mcpwm_emit_operator(p, unit, operator_index);
        mcpwm_kick(p);
        return;
    }
    if (off >= MCPWM_OPERATOR_BASE_OFF && off < MCPWM_FAULT_DETECT_OFF) {
        unsigned operator_index = (off - MCPWM_OPERATOR_BASE_OFF) /
                                  MCPWM_OPERATOR_STRIDE;
        uint32_t relative = (off - MCPWM_OPERATOR_BASE_OFF) %
                            MCPWM_OPERATOR_STRIDE;
        if (relative == MCPWM_FH_STATUS_REL) return;
        mcpwm_write_operator(p, unit, operator_index, relative, val);
        return;
    }
    if (off == MCPWM_FAULT_DETECT_OFF) {
        uint32_t status = state->regs[off / 4u] & 0x1C0u;
        state->regs[off / 4u] = (val & 0x3Fu) | status;
        for (unsigned fault = 0; fault < MCPWM_TIMER_COUNT; fault++)
            mcpwm_refresh_fault(p, unit, fault);
        mcpwm_kick(p);
        return;
    }
    if (off == MCPWM_CAP_TIMER_CFG_OFF) {
        state->regs[off / 4u] = val & 0x1Fu;
        if ((val & (1u << 5)) && (val & (1u << 1))) {
            state->capture_counter =
                state->regs[MCPWM_CAP_TIMER_PHASE_OFF / 4u];
            state->capture_remainder = 0;
        }
        state->capture_last_cycles = now;
        return;
    }
    if (off == MCPWM_CAP_TIMER_PHASE_OFF) {
        state->regs[off / 4u] = val;
        return;
    }
    if (off >= MCPWM_CAP_CH_CFG_OFF &&
        off < MCPWM_CAP_CH_CFG_OFF + MCPWM_TIMER_COUNT * 4u) {
        unsigned channel = (off - MCPWM_CAP_CH_CFG_OFF) / 4u;
        state->regs[off / 4u] = val & 0xFFFu;
        state->capture_prescale_count[channel] = 0;
        mcpwm_rebind_input_signal(p, unit, 2u, channel);
        if (val & (1u << 12)) mcpwm_software_capture(p, unit, channel);
        return;
    }
    if ((off >= MCPWM_CAP_CH_VALUE_OFF && off <= MCPWM_CAP_STATUS_OFF) ||
        off == MCPWM_INT_ST_OFF)
        return;
    if (off == MCPWM_UPDATE_CFG_OFF) {
        uint32_t old = state->regs[off / 4u];
        state->regs[off / 4u] = val & 0xFFu;
        if ((old ^ val) & (1u << 1))
            mcpwm_flush_all(p, unit);
        for (unsigned operator_index = 0;
             operator_index < MCPWM_OPERATOR_COUNT; operator_index++) {
            if ((old ^ val) & (1u << (3u + operator_index * 2u)))
                mcpwm_flush_operator(p, unit, operator_index);
        }
        mcpwm_kick(p);
        return;
    }
    if (off == MCPWM_INT_ENA_OFF) {
        state->regs[off / 4u] = val & MCPWM_INT_VALID_MASK;
        mcpwm_update_irq(p, unit);
        mcpwm_kick(p);
        return;
    }
    if (off == MCPWM_INT_RAW_OFF) return;
    if (off == MCPWM_INT_CLR_OFF) {
        state->regs[MCPWM_INT_RAW_OFF / 4u] &=
            ~(val & MCPWM_INT_VALID_MASK);
        mcpwm_update_irq(p, unit);
        mcpwm_kick(p);
        return;
    }
    if (off == MCPWM_CLK_OFF) {
        state->regs[off / 4u] = val & 1u;
        return;
    }
    if (off == MCPWM_VERSION_OFF) {
        state->regs[off / 4u] = val & 0x0FFFFFFFu;
        return;
    }
    state->regs[off / 4u] = val;
}

static bool mcpwm_unit_has_internal_sync(const mcpwm_unit_state_t *state) {
    uint32_t selectors = state->regs[MCPWM_TIMER_SYNCI_CFG_OFF / 4u];
    for (unsigned target = 0; target < MCPWM_TIMER_COUNT; target++) {
        unsigned selected = (selectors >> (target * 3u)) & 7u;
        if (selected >= 1u && selected <= 3u) return true;
    }
    return false;
}

static bool mcpwm_timer_has_deferred_boundary_work(
    const mcpwm_unit_state_t *state, unsigned timer_index) {
    const mcpwm_timer_state_t *timer = &state->timer[timer_index];
    if (timer->stop_at_tez || timer->stop_at_tep || timer->period_pending)
        return true;
    for (unsigned operator_index = 0;
         operator_index < MCPWM_OPERATOR_COUNT; operator_index++) {
        if (mcpwm_operator_timer(state, operator_index) != timer_index)
            continue;
        const mcpwm_operator_state_t *op =
            &state->operators[operator_index];
        if (op->compare_pending[0] || op->compare_pending[1] ||
            op->generator_pending[0] || op->generator_pending[1] ||
            op->fed_pending || op->red_pending || op->force_pending)
            return true;
    }
    return false;
}

static uint64_t mcpwm_cycles_until_ticks(const esp32_periph_t *p,
                                          unsigned unit,
                                          const mcpwm_timer_state_t *timer,
                                          uint32_t ticks) {
    uint64_t denominator = mcpwm_timer_denominator(p, unit, timer);
    uint64_t needed = (uint64_t)ticks * denominator;
    if (needed <= timer->tick_remainder) return 0;
    needed -= timer->tick_remainder;
    return (needed + MCPWM_SOURCE_CLOCK_MHZ - 1u) /
           MCPWM_SOURCE_CLOCK_MHZ;
}

static uint32_t mcpwm_deadline_ccount(const xtensa_cpu_t *cpu,
                                      uint64_t distance) {
    if (distance == 0u) distance = 1u;
    if (distance >= (uint64_t)INT32_MAX)
        distance = (uint64_t)INT32_MAX - 1u;
    return cpu->ccount + (uint32_t)distance;
}

static uint32_t mcpwm_next_fire(esp32_periph_t *p, xtensa_cpu_t *cpu) {
    if (!p || !cpu || cpu != p->cpu[0]) return UINT32_MAX;
    uint64_t now = mcpwm_now_cycles(p);
    mcpwm_sync_all_to(p, now);
    bool have = false;
    uint32_t best = UINT32_MAX;
    uint32_t best_distance = 0;
    for (unsigned unit = 0; unit < MCPWM_UNIT_COUNT; unit++) {
        mcpwm_unit_state_t *state = &p->mcpwm.unit[unit];
        uint32_t raw = state->regs[MCPWM_INT_RAW_OFF / 4u];
        uint32_t ena = state->regs[MCPWM_INT_ENA_OFF / 4u];
        bool timed_irq_pending = ((ena & ~raw) & 0x001FFFFFu) != 0;
        bool internal_sync = mcpwm_unit_has_internal_sync(state);
        for (unsigned timer_index = 0;
             timer_index < MCPWM_TIMER_COUNT; timer_index++) {
            mcpwm_timer_state_t *timer = &state->timer[timer_index];
            if (!timer->running || timer->mode == 0u ||
                !mcpwm_unit_clocked(p, unit))
                continue;
            if (!timed_irq_pending && !internal_sync &&
                !mcpwm_timer_has_deferred_boundary_work(state,
                                                        timer_index))
                continue;
            uint32_t ticks =
                mcpwm_timer_next_event_ticks(state, timer_index);
            uint64_t cycles = mcpwm_cycles_until_ticks(
                p, unit, timer, ticks);
            uint32_t event = mcpwm_deadline_ccount(cpu, cycles);
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

static void mcpwm_eval_events(esp32_periph_t *p, xtensa_cpu_t *cpu) {
    if (!p || !cpu || cpu != p->cpu[0]) return;
    uint64_t now = mcpwm_now_cycles(p);
    mcpwm_sync_all_to(p, now);
    for (unsigned unit = 0; unit < MCPWM_UNIT_COUNT; unit++)
        mcpwm_update_irq(p, unit);
    mcpwm_kick(p);
}

static void mcpwm_reset_unit(esp32_periph_t *p, unsigned unit) {
    if (!p || unit >= MCPWM_UNIT_COUNT) return;
    mcpwm_unit_state_t *state = &p->mcpwm.unit[unit];
    periph_mcpwm_output_fn callbacks[MCPWM_OPERATOR_COUNT]
                                      [MCPWM_GENERATOR_COUNT];
    void *contexts[MCPWM_OPERATOR_COUNT][MCPWM_GENERATOR_COUNT];
    bool reported[MCPWM_OPERATOR_COUNT][MCPWM_GENERATOR_COUNT];
    for (unsigned operator_index = 0;
         operator_index < MCPWM_OPERATOR_COUNT; operator_index++) {
        for (unsigned generator = 0; generator < MCPWM_GENERATOR_COUNT;
             generator++) {
            callbacks[operator_index][generator] =
                state->operators[operator_index].output_cb[generator];
            contexts[operator_index][generator] =
                state->operators[operator_index].output_cb_ctx[generator];
            reported[operator_index][generator] =
                state->operators[operator_index].output_reported[generator];
        }
    }

    memset(state, 0, sizeof(*state));
    uint64_t now = mcpwm_now_cycles(p);
    for (unsigned timer = 0; timer < MCPWM_TIMER_COUNT; timer++) {
        uint32_t base = mcpwm_timer_offset(timer);
        state->regs[(base + MCPWM_TIMER_CFG0_REL) / 4u] =
            MCPWM_TIMER_PERIOD_RESET;
        state->timer[timer].active_period = 255u;
        state->timer[timer].active_prescale = 0u;
        state->timer[timer].last_cycles = now;
    }
    for (unsigned operator_index = 0;
         operator_index < MCPWM_OPERATOR_COUNT; operator_index++) {
        uint32_t base = mcpwm_operator_offset(operator_index);
        state->regs[(base + MCPWM_GEN_FORCE_REL) / 4u] =
            MCPWM_GEN_FORCE_RESET;
        state->regs[(base + MCPWM_DT_CFG_REL) / 4u] =
            MCPWM_DT_CFG_RESET;
        for (unsigned generator = 0; generator < MCPWM_GENERATOR_COUNT;
             generator++) {
            state->operators[operator_index].output_cb[generator] =
                callbacks[operator_index][generator];
            state->operators[operator_index].output_cb_ctx[generator] =
                contexts[operator_index][generator];
            state->operators[operator_index].output_reported[generator] =
                reported[operator_index][generator];
        }
    }
    state->regs[MCPWM_UPDATE_CFG_OFF / 4u] = MCPWM_UPDATE_CFG_RESET;
    state->regs[MCPWM_VERSION_OFF / 4u] = MCPWM_VERSION_RESET;
    state->capture_last_cycles = now;
    for (unsigned channel = 0; channel < MCPWM_TIMER_COUNT; channel++) {
        mcpwm_rebind_input_signal(p, unit, 0u, channel);
        mcpwm_rebind_input_signal(p, unit, 1u, channel);
        mcpwm_rebind_input_signal(p, unit, 2u, channel);
    }
    mcpwm_update_irq(p, unit);
    for (unsigned operator_index = 0;
         operator_index < MCPWM_OPERATOR_COUNT; operator_index++)
        mcpwm_emit_operator(p, unit, operator_index);
    mcpwm_kick(p);
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
    pcnt_reset_state(p);

    /* Strapping-pin idle levels: GPIO0/5/15 have pull-ups enabled at reset
     * (GPIO2/12 pull-downs read low). Firmware reads GPIO_IN for buttons
     * tied to these pins (e.g. Marauder's BOOT-button on GPIO0) — leaving
     * them low looks like a permanently held button. */
    p->gpio.in = (1u << 0) | (1u << 5) | (1u << 15);

    /* Initialize interrupt matrix: all lines disabled (source 16 = none) */
    memset(p->intr_matrix, 16, sizeof(p->intr_matrix));

    /* General-purpose timers reset with count-up and auto-reload selected,
     * but remain stopped until firmware enables their group clock and Tx_EN. */
    timg_reset_group(p, 0);
    timg_reset_group(p, 1);

    /* Both legacy FRC timers reset disabled with count/load/alarm at zero. */
    frc_reset(p);

    /* LEDC reset state: all eight timers begin held in reset, and DATE is
     * the ESP32 peripheral version value from the vendor register map. */
    ledc_reset_state(p);

    /* Both motor-control PWM units have independent reset domains. */
    mcpwm_reset_unit(p, 0);
    mcpwm_reset_unit(p, 1);

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

    /* Legacy FRC1/FRC2 timers (interrupt sources 56/57). */
    mem_register_mmio(mem, (int)PAGE_OF(FRC_TIMER_BASE),
                      frc_read, frc_write, p);

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

    /* Eight two-channel pulse-counter units (interrupt source 48). */
    mem_register_mmio(mem, (int)PAGE_OF(PCNT_BASE), pcnt_read, pcnt_write, p);

    /* Two motor-control PWM units (interrupt sources 39/40). */
    mem_register_mmio(mem, (int)PAGE_OF(MCPWM0_BASE),
                      mcpwm_read, mcpwm_write, p);
    mem_register_mmio(mem, (int)PAGE_OF(MCPWM1_BASE),
                      mcpwm_read, mcpwm_write, p);

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

    uint32_t route = p->gpio.func_out_sel[pin];
    uint32_t signal = route & 0x1FFu;
    for (unsigned unit = 0; unit < MCPWM_UNIT_COUNT; unit++) {
        uint32_t base = mcpwm_output_signal(unit, 0, 0);
        if (signal < base || signal >= base + 6u) continue;
        unsigned relative = signal - base;
        bool level = mcpwm_generator_pin_level(
            p, unit, relative / 2u, relative & 1u);
        if (route & (1u << 9)) level = !level;
        return level ? 1 : 0;
    }

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

int periph_set_mcpwm_output_callback(esp32_periph_t *p, int unit,
                                     int operator_index, int generator,
                                     periph_mcpwm_output_fn fn, void *ctx) {
    if (!p || unit < 0 || unit >= (int)MCPWM_UNIT_COUNT ||
        operator_index < 0 ||
        operator_index >= (int)MCPWM_OPERATOR_COUNT || generator < 0 ||
        generator >= (int)MCPWM_GENERATOR_COUNT)
        return -1;

    mcpwm_operator_state_t *op =
        &p->mcpwm.unit[unit].operators[operator_index];
    op->output_cb[generator] = fn;
    op->output_cb_ctx[generator] = fn ? ctx : NULL;
    op->output_reported[generator] = false;
    if (fn)
        mcpwm_emit_output(p, (unsigned)unit, (unsigned)operator_index,
                          (unsigned)generator, true);
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
    for (unsigned core = 0; core < 2u; core++) {
        xtensa_cpu_t *cpu = core == 0u ? cpu0 : cpu1;
        p->timg_clock.core_cycles[core] = p->timg_clock.cycles;
        p->timg_clock.last_ccount[core] = cpu ? cpu->ccount : 0u;
        p->timg_clock.valid[core] = cpu != NULL;
        p->mcpwm.core_cycles[core] = p->mcpwm.time_cycles;
        p->mcpwm.last_ccount[core] = cpu ? cpu->ccount : 0u;
        p->mcpwm.time_valid[core] = cpu != NULL;
    }
    for (unsigned timer = 0; timer < FRC_TIMER_COUNT; timer++)
        p->frc_timer[timer].last_cycles = p->timg_clock.cycles;
    /* Wire Timer Group, LACT, and other timed-peripheral event hooks into
     * both cores so alarms fire on time and can wake a core from WAITI. */
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

    /* GPIO-matrix consumers see the pad transition independently of the
     * GPIO block's own edge/level interrupt configuration. */
    pcnt_gpio_input_changed(p, pin);
    mcpwm_gpio_input_changed(p, pin);

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
