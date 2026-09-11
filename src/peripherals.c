#include "peripherals.h"
#include "esp32s3_extmem.h"
#include "efuse.h"
#include "flash_mmu.h"
#include "io_mux.h"
#include "rtc_cntl.h"
#include "regi2c.h"
#include "sensitive_memprot.h"
#include "spi_mem.h"
#include "system_clock.h"
#include "systimer.h"
#include "timer_group.h"
#include "usb_serial_jtag.h"
#include "spi_display.h"
#include "sandbox_events.h"
#include "xtensa.h"
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

/* Resolved once. gpio_dbg() is asked on every edge evaluation, which a CYD
 * firmware does constantly: 4,122,787 lookups in one Marauder scenario, which
 * put getenv() -- a linear scan of environ -- at 3.9% of the emulator's host
 * CPU. Same fix and same reason as spi_dbg() in spi_display.c.
 *
 * The memcmp time next to it in the profile is not this: it stayed at 3.8%
 * after these lookups went away, because it is the guest's own memcmp running
 * through stub_memcmp(). That one is real work, not overhead. */
static int gpio_dbg_flag = -1;
static inline int gpio_dbg(void) {
    if (__builtin_expect(gpio_dbg_flag < 0, 0))
        gpio_dbg_flag = getenv("FLEXE_GPIODBG") != NULL;
    return gpio_dbg_flag;
}

/* ESP32 peripheral base addresses */
#define PERIPH_BASE     0x3FF00000u
#define DPORT_BASE      0x3FF00000u
#define UHCI0_BASE      0x3FF54000u
#define UHCI1_BASE      0x3FF4C000u
#define HINF_BASE       0x3FF4B000u
#define SLCHOST_BASE    0x3FF55000u
#define SLC_BASE        0x3FF58000u
#define I2C0_BASE       0x3FF53000u
#define I2C1_BASE       0x3FF67000u
#define SDMMC_BASE      0x3FF68000u
#define EMAC_DMA_BASE   0x3FF69000u
#define EMAC_EXT_BASE   0x3FF69800u
#define EMAC_MAC_BASE   0x3FF6A000u
#define TWAI_BASE       0x3FF6B000u
#define RMT_BASE        0x3FF56000u
#define PCNT_BASE       0x3FF57000u
#define MCPWM0_BASE     0x3FF5E000u
#define MCPWM1_BASE     0x3FF6C000u
#define GPIO_BASE       0x3FF44000u
#define GPIO_SD_BASE    0x3FF44F00u
#define FE2_BASE        0x3FF45000u
#define FE_BASE         0x3FF46000u
#define FRC_TIMER_BASE  0x3FF47000u
#define PHY_BASE        0x3FF4E000u  /* undocumented WiFi PHY calibration window */
#define RTC_CNTL_BASE   0x3FF48000u
#define RTCIO_BASE      0x3FF48400u
#define SENS_BASE       0x3FF48800u
#define RTC_I2C_BASE    0x3FF48C00u
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
#define BT_MAC_BASE     0x3FF72000u
#define WIFI_MAC_BASE   0x3FF73000u  /* WiFi MAC/BB control registers */
#define WIFI_MAC_SIZE   0x2000u
#define WDEV_BASE       0x3FF75000u  /* WiFi device (contains RNG register) */
#define PAGE_SIZE       4096
#define PAGE_WORDS      (PAGE_SIZE / sizeof(uint32_t))
#define EMU_FLASH_SIZE  (4u * 1024u * 1024u)
#define PERIPH_DEFERRED_MAX 16u

/* Classic ESP32 flash-cache MMU geometry. Each core exposes an 8 KiB register
 * window. The first 256 entries drive the four 64-entry DROM0/IRAM0/IRAM1/
 * IROM0 XIP regions; later entries include reserved and PSRAM/PID banks. */
#define FLASH_MMU_REGISTER_COUNT 2048u
#define FLASH_MMU_ENTRY_COUNT    256u
#define FLASH_MMU_PAGE_SIZE      0x10000u
#define FLASH_MMU_INVALID        0x100u
#define FLASH_MMU_VALUE_MASK     0x1FFu

/* RTC-domain analog register geometry (classic ESP32). RTCIO and SENS share
 * RTC_CNTL's 4 KiB MMIO page, at offsets 0x400 and 0x800 respectively. */
#define RTCIO_REG_FILE_SIZE         0x100u
#define RTCIO_DAC1_OFF              0x084u
#define RTCIO_DAC2_OFF              0x088u
#define RTCIO_DAC_VALUE_MASK        (0xFFu << 19)
#define RTCIO_DAC_XPD               (1u << 18)
#define RTCIO_DAC_XPD_FORCE         (1u << 10)

/* RTC GPIO. Eighteen RTC-capable pads, addressed by channel rather than by
 * GPIO number, all packed into bits 31:14 of their registers (rtc_io_reg.h). */
#define RTC_GPIO_OUT_OFF            0x000u
#define RTC_GPIO_OUT_W1TS_OFF       0x004u
#define RTC_GPIO_OUT_W1TC_OFF       0x008u
#define RTC_GPIO_ENABLE_OFF         0x00Cu
#define RTC_GPIO_ENABLE_W1TS_OFF    0x010u
#define RTC_GPIO_ENABLE_W1TC_OFF    0x014u
#define RTC_GPIO_IN_OFF             0x024u
#define RTC_GPIO_DATA_S             14
#define RTC_GPIO_CHANNELS           18

/* Channel -> GPIO, from rtc_io_channel.h. */
static const int8_t RTCIO_CHANNEL_GPIO[RTC_GPIO_CHANNELS] = {
    36, 37, 38, 39, 34, 35, 25, 26, 33, 32, 4, 0, 2, 15, 13, 12, 14, 27
};


#define SENS_REG_FILE_SIZE          0x100u
#define SENS_SAR_START_FORCE_OFF    0x02Cu
#define SENS_SAR_SLAVE_ADDR1_OFF    0x03Cu
#define SENS_SAR_SLAVE_ADDR2_OFF    0x040u
#define SENS_SAR_SLAVE_ADDR3_OFF    0x044u
#define SENS_SAR_SLAVE_ADDR4_OFF    0x048u
#define SENS_SAR_I2C_CTRL_OFF       0x050u
#define SENS_SAR_MEAS_START1_OFF    0x054u
#define SENS_SAR_MEAS_START2_OFF    0x094u
#define SENS_I2C_ADDR_FIELDS_MASK   0x003FFFFFu
#define SENS_I2C_RDATA_MASK         (0xFFu << 22)
#define SENS_I2C_DONE               (1u << 30)
#define SENS_I2C_CTRL_MASK          0x3FFFFFFFu
#define SENS_I2C_START              (1u << 28)
#define SENS_I2C_START_FORCE        (1u << 29)
#define SENS_SAR_EN_PAD_MASK        (0xFFFu << 19)
#define SENS_SAR_START              (1u << 17)
#define SENS_SAR_DONE               (1u << 16)
#define SENS_SAR_CONFIG_MASK        0xFFFE0000u

/* Capacitive touch controller. All ten pads live inside the SENS window this
 * file already decodes. Offsets and field shifts are from sens_reg.h; the
 * THRES and OUT registers each carry two pads, the even one in bits 31:16 and
 * the odd one in bits 15:0. */
#define SENS_SAR_TOUCH_CTRL1_OFF    0x058u
#define SENS_SAR_TOUCH_THRES1_OFF   0x05Cu
#define SENS_SAR_TOUCH_OUT1_OFF     0x070u
#define SENS_SAR_TOUCH_CTRL2_OFF    0x084u
#define SENS_SAR_TOUCH_ENABLE_OFF   0x08Cu
#define SENS_TOUCH_PAD_COUNT        10
#define SENS_TOUCH_STATUS_MASK      0x000003FFu   /* CTRL2 bits 9:0  */
#define SENS_TOUCH_MEAS_DONE_BIT    (1u << 10)
#define SENS_TOUCH_START_FSM_EN_BIT (1u << 11)
#define SENS_TOUCH_START_EN_BIT     (1u << 12)
#define SENS_TOUCH_START_FORCE_BIT  (1u << 13)
#define SENS_TOUCH_MEAS_EN_CLR_BIT  (1u << 30)
#define SENS_TOUCH_WORKEN_MASK      0x000003FFu   /* ENABLE bits 9:0 */

/* RTC_CNTL interrupt block (rtc_cntl_reg.h). The touch controller reports
 * through bit 6, delivered on ETS_RTC_CORE_INTR_SOURCE. */
#define RTC_CNTL_INT_ENA_OFF        0x03Cu
#define RTC_CNTL_INT_RAW_OFF        0x040u
#define RTC_CNTL_INT_ST_OFF         0x044u
#define RTC_CNTL_INT_CLR_OFF        0x048u
#define RTC_CNTL_TOUCH_INT_BIT      (1u << 6)
#define RTC_CNTL_SLP_WAKEUP_INT_BIT (1u << 0)
#define RTC_CORE_INTR_SOURCE        46

/* Sleep and wake (rtc_cntl_reg.h). */
#define RTC_CNTL_SLP_TIMER0_OFF     0x004u
#define RTC_CNTL_SLP_TIMER1_OFF     0x008u
#define RTC_CNTL_TIME_UPDATE_OFF    0x00Cu
#define RTC_CNTL_TIME0_OFF          0x010u
#define RTC_CNTL_TIME1_OFF          0x014u
#define RTC_CNTL_STATE0_OFF         0x018u
#define RTC_CNTL_RESET_STATE_OFF    0x034u
#define RTC_CNTL_WAKEUP_STATE_OFF   0x038u
#define RTC_CNTL_STORE0_OFF         0x04Cu
#define RTC_CNTL_EXT_WAKEUP_CONF_OFF 0x060u
#define RTC_CNTL_DIG_PWC_OFF        0x084u
#define RTC_CNTL_EXT_WAKEUP1_OFF    0x0CCu
#define RTC_CNTL_SLEEP_EN_BIT       (1u << 31)
#define RTC_CNTL_SLP_WAKEUP_BIT     (1u << 29)
#define RTC_CNTL_TOUCH_SLP_TIMER_EN_BIT (1u << 23)
#define RTC_CNTL_WAKEUP_ENA_S       11
#define RTC_CNTL_WAKEUP_ENA_MASK    0x7FFu
#define RTC_CNTL_DG_WRAP_PD_EN_BIT  (1u << 31)
#define RTC_CNTL_EXT_WAKEUP0_LV_BIT (1u << 30)
#define RTC_CNTL_EXT_WAKEUP1_LV_BIT (1u << 31)
#define RTC_IO_EXT_WAKEUP0_OFF      0x0BCu
#define RTC_IO_EXT_WAKEUP0_SEL_S    27

/* Wake trigger bits, from rtc.h. These are what esp_sleep_get_wakeup_cause()
 * decodes out of RTC_CNTL_WAKEUP_STATE. */
#define RTC_EXT0_TRIG_EN            (1u << 0)
#define RTC_EXT1_TRIG_EN            (1u << 1)
#define RTC_TIMER_TRIG_EN           (1u << 3)
#define RTC_TOUCH_TRIG_EN           (1u << 8)

/* RTC slow clock, nominal 150 kHz. */
#define RTC_SLOW_CLK_HZ             150000u
/* rtc.h RESET_REASON. */
#define RTC_POWERON_RESET           1u
#define RTC_DEEPSLEEP_RESET         5u

#define RTC_CPU_PERIOD_CONF_MASK    0xE0000000u
#define RTC_CLK_CONF_MASK           0xFFFFFFF0u

/* Classic RTC-domain I2C controller. Its raw interrupt event bitmap is one
 * bit lower than the enable/status/clear bitmap, as documented by the TRM. */
#define RTC_I2C_REG_FILE_SIZE       0x088u
#define RTC_I2C_SCL_LOW_OFF         0x000u
#define RTC_I2C_CTRL_OFF            0x004u
#define RTC_I2C_DEBUG_STATUS_OFF    0x008u
#define RTC_I2C_TIMEOUT_OFF         0x00Cu
#define RTC_I2C_SLAVE_ADDR_OFF      0x010u
#define RTC_I2C_DATA_OFF            0x01Cu
#define RTC_I2C_INT_RAW_OFF         0x020u
#define RTC_I2C_INT_CLR_OFF         0x024u
#define RTC_I2C_INT_ENA_OFF         0x028u
#define RTC_I2C_INT_ST_OFF          0x02Cu
#define RTC_I2C_SDA_DUTY_OFF        0x030u
#define RTC_I2C_SCL_HIGH_OFF        0x038u
#define RTC_I2C_SCL_START_OFF       0x040u
#define RTC_I2C_SCL_STOP_OFF        0x044u
#define RTC_I2C_COMMAND0_OFF        0x048u
#define RTC_I2C_COMMAND_COUNT       16u
#define RTC_I2C_CTRL_MASK           0x000000F3u
#define RTC_I2C_CTRL_TRANS_START    (1u << 5)
#define RTC_I2C_CTRL_MASTER         (1u << 4)
#define RTC_I2C_DEBUG_MASK          0x7E00007Fu
#define RTC_I2C_DEBUG_BYTE_TRANS    (1u << 6)
#define RTC_I2C_DEBUG_BUS_BUSY      (1u << 4)
#define RTC_I2C_DEBUG_ARB_LOST      (1u << 3)
#define RTC_I2C_DEBUG_TIMED_OUT     (1u << 2)
#define RTC_I2C_DEBUG_ACK_VAL       (1u << 0)
#define RTC_I2C_PERIOD19_MASK       0x0007FFFFu
#define RTC_I2C_PERIOD20_MASK       0x000FFFFFu
#define RTC_I2C_SLAVE_ADDR_MASK     0x80007FFFu
#define RTC_I2C_INT_RAW_TIMEOUT     (1u << 7)
#define RTC_I2C_INT_RAW_TRANS_DONE  (1u << 6)
#define RTC_I2C_INT_RAW_MASTER_DONE (1u << 5)
#define RTC_I2C_INT_RAW_ARB_LOST    (1u << 4)
#define RTC_I2C_INT_RAW_SLAVE_DONE  (1u << 3)
#define RTC_I2C_INT_RAW_MASK        0x000000F8u
#define RTC_I2C_INT_SHIFTED_MASK    0x000001F0u
#define RTC_I2C_INT_ENA_MASK        0x000001E0u
#define RTC_I2C_COMMAND_MASK        0x80003FFFu

/* The original ESP32 embeds its eight-channel sigma-delta/PDM block in the
 * final 256 bytes of the GPIO page. Each channel adds a signed duty offset to
 * the 50% midpoint, producing duty+128 high samples per 256-sample period. */
#define SIGMADELTA_CHANNEL_COUNT     8u
#define SIGMADELTA_SIGNAL_BASE       100u
#define SIGMADELTA_CHANNEL_MASK      0x0000FFFFu
#define SIGMADELTA_CG_OFF            0x020u
#define SIGMADELTA_MISC_OFF          0x024u
#define SIGMADELTA_VERSION_OFF       0x028u
#define SIGMADELTA_CG_MASK           (1u << 31)
#define SIGMADELTA_MISC_MASK         (1u << 31)
#define SIGMADELTA_VERSION_MASK      0x0FFFFFFFu
#define SIGMADELTA_VERSION_RESET     0x01506190u
#define SIGMADELTA_APB_MHZ           80u

/* Page index from absolute address */
#define PAGE_OF(addr) (((addr) - PERIPH_BASE) / PAGE_SIZE)

/* UART FIFOs / host capture. ESP32 UART hardware has 128-byte FIFOs. */
#define UART_TX_BUF_SIZE 4096
#define UART_RX_FIFO_SIZE 128
#define UART_COUNT FLEXE_TARGET_UART_MAX

/* Classic ESP32 has two UART DMA (UHCI) controllers shared by UART0/1/2.
 * Descriptor words use the same lldesc layout as the SPI/I2S DMA engines. */
#define UHCI_PORT_COUNT             2
#define UHCI_REG_FILE_SIZE          0x100u
#define UHCI_DMA_MAX_DESCRIPTORS    256u
#define UHCI_DMA_MAX_BUFFER         4095u
#define UHCI_FRAME_MAX              (4u + UHCI_DMA_MAX_BUFFER + 2u)
#define UHCI_DEBUG_FIFO_SIZE        128u

#define UHCI_CONF0_OFF              0x000u
#define UHCI_INT_RAW_OFF            0x004u
#define UHCI_INT_ST_OFF             0x008u
#define UHCI_INT_ENA_OFF            0x00Cu
#define UHCI_INT_CLR_OFF            0x010u
#define UHCI_DMA_OUT_STATUS_OFF     0x014u
#define UHCI_DMA_OUT_PUSH_OFF       0x018u
#define UHCI_DMA_IN_STATUS_OFF      0x01Cu
#define UHCI_DMA_IN_POP_OFF         0x020u
#define UHCI_DMA_OUT_LINK_OFF       0x024u
#define UHCI_DMA_IN_LINK_OFF        0x028u
#define UHCI_CONF1_OFF              0x02Cu
#define UHCI_STATE0_OFF             0x030u
#define UHCI_STATE1_OFF             0x034u
#define UHCI_OUT_EOF_DESC_OFF       0x038u
#define UHCI_IN_SUC_EOF_DESC_OFF    0x03Cu
#define UHCI_IN_ERR_EOF_DESC_OFF    0x040u
#define UHCI_OUT_EOF_BFR_DESC_OFF   0x044u
#define UHCI_AHB_TEST_OFF           0x048u
#define UHCI_IN_DSCR_OFF            0x04Cu
#define UHCI_IN_DSCR_BF0_OFF        0x050u
#define UHCI_IN_DSCR_BF1_OFF        0x054u
#define UHCI_OUT_DSCR_OFF           0x058u
#define UHCI_OUT_DSCR_BF0_OFF       0x05Cu
#define UHCI_OUT_DSCR_BF1_OFF       0x060u
#define UHCI_ESCAPE_CONF_OFF        0x064u
#define UHCI_HUNG_CONF_OFF          0x068u
#define UHCI_ACK_NUM_OFF            0x06Cu
#define UHCI_RX_HEAD_OFF            0x070u
#define UHCI_QUICK_SENT_OFF         0x074u
#define UHCI_Q_DATA_FIRST_OFF       0x078u
#define UHCI_Q_DATA_LAST_OFF        0x0ACu
#define UHCI_ESC_CONF0_OFF          0x0B0u
#define UHCI_ESC_CONF1_OFF          0x0B4u
#define UHCI_ESC_CONF2_OFF          0x0B8u
#define UHCI_ESC_CONF3_OFF          0x0BCu
#define UHCI_PKT_THRES_OFF          0x0C0u
#define UHCI_DATE_OFF               0x0FCu

#define UHCI_CONF0_IN_RST           (1u << 0)
#define UHCI_CONF0_OUT_RST          (1u << 1)
#define UHCI_CONF0_AHB_FIFO_RST     (1u << 2)
#define UHCI_CONF0_AHB_RST          (1u << 3)
#define UHCI_CONF0_IN_LOOP_TEST     (1u << 4)
#define UHCI_CONF0_OUT_LOOP_TEST    (1u << 5)
#define UHCI_CONF0_OUT_AUTO_WRBACK  (1u << 6)
#define UHCI_CONF0_OUT_EOF_MODE     (1u << 8)
#define UHCI_CONF0_UART_CE_MASK     (7u << 9)
#define UHCI_CONF0_SEPER_EN         (1u << 16)
#define UHCI_CONF0_HEAD_EN          (1u << 17)
#define UHCI_CONF0_CRC_REC_EN       (1u << 18)
#define UHCI_CONF0_UART_IDLE_EOF_EN (1u << 19)
#define UHCI_CONF0_LEN_EOF_EN       (1u << 20)
#define UHCI_CONF0_ENCODE_CRC_EN    (1u << 21)
#define UHCI_CONF0_UART_BRK_EOF_EN  (1u << 23)
#define UHCI_CONF0_VALID_MASK       0x00FFFFFFu

#define UHCI_CONF1_CHECK_SUM_EN     (1u << 0)
#define UHCI_CONF1_CHECK_SEQ_EN     (1u << 1)
#define UHCI_CONF1_CRC_DISABLE      (1u << 2)
#define UHCI_CONF1_SAVE_HEAD        (1u << 3)
#define UHCI_CONF1_TX_CHECK_SUM_RE  (1u << 4)
#define UHCI_CONF1_TX_ACK_NUM_RE    (1u << 5)
#define UHCI_CONF1_CHECK_OWNER      (1u << 6)
#define UHCI_CONF1_WAIT_SW_START    (1u << 7)
#define UHCI_CONF1_SW_START         (1u << 8)
#define UHCI_CONF1_VALID_MASK       0x001FFFFFu

#define UHCI_LINK_ADDR_MASK         0x000FFFFFu
#define UHCI_INLINK_AUTO_RET        (1u << 20)
#define UHCI_LINK_STOP              (1u << 28)
#define UHCI_LINK_START             (1u << 29)
#define UHCI_LINK_RESTART           (1u << 30)
#define UHCI_LINK_PARK              (1u << 31)

#define UHCI_INT_RX_START           (1u << 0)
#define UHCI_INT_TX_START           (1u << 1)
#define UHCI_INT_RX_HUNG            (1u << 2)
#define UHCI_INT_TX_HUNG            (1u << 3)
#define UHCI_INT_IN_DONE            (1u << 4)
#define UHCI_INT_IN_SUC_EOF         (1u << 5)
#define UHCI_INT_IN_ERR_EOF         (1u << 6)
#define UHCI_INT_OUT_DONE           (1u << 7)
#define UHCI_INT_OUT_EOF            (1u << 8)
#define UHCI_INT_IN_DSCR_ERR        (1u << 9)
#define UHCI_INT_OUT_DSCR_ERR       (1u << 10)
#define UHCI_INT_IN_DSCR_EMPTY      (1u << 11)
#define UHCI_INT_OUTLINK_EOF_ERR    (1u << 12)
#define UHCI_INT_OUT_TOTAL_EOF      (1u << 13)
#define UHCI_INT_SEND_S_Q           (1u << 14)
#define UHCI_INT_SEND_A_Q           (1u << 15)
#define UHCI_INT_IN_FIFO_FULL_WM    (1u << 16)
#define UHCI_INT_VALID_MASK         0x0001FFFFu

#define UHCI_DESC_SIZE_MASK         0x00000FFFu
#define UHCI_DESC_LENGTH_MASK       0x00FFF000u
#define UHCI_DESC_LENGTH_SHIFT      12u
#define UHCI_DESC_EOF               (1u << 30)
#define UHCI_DESC_OWNER             (1u << 31)

#define UHCI_CONF0_RESET            0x00370100u
#define UHCI_CONF1_RESET            0x00000033u
#define UHCI_ESCAPE_CONF_RESET      0x00000033u
#define UHCI_HUNG_CONF_RESET        0x00810810u
#define UHCI_ESC_CONF0_RESET        0x00DCDBC0u
#define UHCI_ESC_CONF1_RESET        0x00DDDBDBu
#define UHCI_ESC_CONF2_RESET        0x00DEDB11u
#define UHCI_ESC_CONF3_RESET        0x00DFDB13u
#define UHCI_PKT_THRES_RESET        0x00000080u
#define UHCI_DATE_RESET             0x16041001u

/* Classic ESP32 SDIO-slave interface. HINF exposes card identity/CCCR state,
 * SLCHOST contains the shared-register and host-visible counter/interrupt
 * windows, and SLC owns the two lldesc DMA channels. Espressif's naming is
 * from the DMA's viewpoint: SLC RX is slave-to-host, while SLC TX is
 * host-to-slave. */
#define HINF_REG_FILE_SIZE           0x100u
#define SLCHOST_REG_FILE_SIZE        0x200u
#define SLC_REG_FILE_SIZE            0x200u
#define SLC_CHANNEL_COUNT            2u
#define SLC_DMA_MAX_DESCRIPTORS      256u
#define SLC_INTR_SOURCE0             10
#define SLC_INTR_SOURCE1             11

#define SLC_CONF0_OFF                0x000u
#define SLC_INT0_RAW_OFF             0x004u
#define SLC_INT0_ST_OFF              0x008u
#define SLC_INT0_ENA_OFF             0x00Cu
#define SLC_INT0_CLR_OFF             0x010u
#define SLC_INT1_RAW_OFF             0x014u
#define SLC_INT1_ST_OFF              0x018u
#define SLC_INT1_ENA_OFF             0x01Cu
#define SLC_INT1_CLR_OFF             0x020u
#define SLC_RX_LINK0_OFF             0x03Cu
#define SLC_TX_LINK0_OFF             0x040u
#define SLC_RX_LINK1_OFF             0x044u
#define SLC_TX_LINK1_OFF             0x048u
#define SLC_INTVEC_TOHOST_OFF        0x04Cu
#define SLC_TOKEN0_0_OFF             0x050u
#define SLC_TOKEN1_0_OFF             0x054u
#define SLC_TOKEN0_1_OFF             0x058u
#define SLC_TOKEN1_1_OFF             0x05Cu
#define SLC_CONF1_OFF                0x060u
#define SLC_TO_EOF_DESC0_OFF         0x078u
#define SLC_TX_EOF_DESC0_OFF         0x07Cu
#define SLC_TO_EOF_BFR_DESC0_OFF     0x080u
#define SLC_TO_EOF_DESC1_OFF         0x084u
#define SLC_TX_EOF_DESC1_OFF         0x088u
#define SLC_TO_EOF_BFR_DESC1_OFF     0x08Cu
#define SLC_RX_DSCR_CONF_OFF         0x098u
#define SLC_TX_DSCR0_OFF             0x09Cu
#define SLC_TX_DSCR0_BF0_OFF         0x0A0u
#define SLC_TX_DSCR0_BF1_OFF         0x0A4u
#define SLC_RX_DSCR0_OFF             0x0A8u
#define SLC_RX_DSCR0_BF0_OFF         0x0ACu
#define SLC_RX_DSCR0_BF1_OFF         0x0B0u
#define SLC_TX_DSCR1_OFF             0x0B4u
#define SLC_TX_DSCR1_BF0_OFF         0x0B8u
#define SLC_TX_DSCR1_BF1_OFF         0x0BCu
#define SLC_RX_DSCR1_OFF             0x0C0u
#define SLC_RX_DSCR1_BF0_OFF         0x0C4u
#define SLC_RX_DSCR1_BF1_OFF         0x0C8u
#define SLC_TX_ERR_EOF_DESC0_OFF     0x0CCu
#define SLC_TX_ERR_EOF_DESC1_OFF     0x0D0u
#define SLC_LEN_CONF0_OFF            0x0E4u
#define SLC_LENGTH0_OFF              0x0E8u
#define SLC_DATE_OFF                 0x1F8u
#define SLC_ID_OFF                   0x1FCu

#define SLC_LINK_ADDR_MASK           0x000FFFFFu
#define SLC_LINK_STOP                (1u << 28)
#define SLC_LINK_START               (1u << 29)
#define SLC_LINK_RESTART             (1u << 30)
#define SLC_LINK_PARK                (1u << 31)
#define SLC_DESC_SIZE_MASK           0x00000FFFu
#define SLC_DESC_LENGTH_MASK         0x00FFF000u
#define SLC_DESC_LENGTH_SHIFT        12u
#define SLC_DESC_EOF                 (1u << 30)
#define SLC_DESC_OWNER               (1u << 31)
#define SLC_INT_FRHOST_MASK          0x000000FFu
#define SLC_INT_RX_START             (1u << 8)
#define SLC_INT_TX_START             (1u << 9)
#define SLC_INT_RX_UDF               (1u << 10)
#define SLC_INT_TX_OVF               (1u << 11)
#define SLC_INT_TOKEN0_EMPTY         (1u << 12)
#define SLC_INT_TOKEN1_EMPTY         (1u << 13)
#define SLC_INT_TX_DONE              (1u << 14)
#define SLC_INT_TX_SUC_EOF           (1u << 15)
#define SLC_INT_RX_DONE              (1u << 16)
#define SLC_INT_RX_EOF               (1u << 17)
#define SLC_INT_TOHOST               (1u << 18)
#define SLC_INT_TX_DSCR_ERR          (1u << 19)
#define SLC_INT_RX_DSCR_ERR          (1u << 20)
#define SLC_INT_TX_DSCR_EMPTY        (1u << 21)
#define SLC_INT_VALID0_MASK          0x07FFFFFFu
#define SLC_INT_VALID1_MASK          0x01FFFFFFu
#define SLC_TOKEN_VALUE_MASK         0x00000FFFu
#define SLC_TOKEN_WR                 (1u << 12)
#define SLC_TOKEN_INC                (1u << 13)
#define SLC_TOKEN_INC_MORE           (1u << 14)
#define SLC_TOKEN_READ_SHIFT         16u
#define SLC_LEN_VALUE_MASK           0x000FFFFFu
#define SLC_LEN_WR                   (1u << 20)
#define SLC_LEN_INC                  (1u << 21)
#define SLC_LEN_INC_MORE             (1u << 22)

#define SLCHOST_TOKEN_RDATA0_OFF     0x044u
#define SLCHOST_INT0_RAW_OFF         0x050u
#define SLCHOST_INT1_RAW_OFF         0x054u
#define SLCHOST_INT0_ST_OFF          0x058u
#define SLCHOST_INT1_ST_OFF          0x05Cu
#define SLCHOST_PKT_LEN_OFF          0x060u
#define SLCHOST_SHARED0_OFF          0x06Cu
#define SLCHOST_TOKEN_RDATA1_OFF     0x0C4u
#define SLCHOST_TOKEN_WDATA0_OFF     0x0C8u
#define SLCHOST_TOKEN_WDATA1_OFF     0x0CCu
#define SLCHOST_TOKEN_CON_OFF        0x0D0u
#define SLCHOST_INT0_CLR_OFF         0x0D4u
#define SLCHOST_INT1_CLR_OFF         0x0D8u
#define SLCHOST_FUNC1_INT0_ENA_OFF   0x0DCu
#define SLCHOST_FUNC1_INT1_ENA_OFF   0x0E0u
#define SLCHOST_FUNC2_INT0_ENA_OFF   0x0E4u
#define SLCHOST_FUNC2_INT1_ENA_OFF   0x0E8u
#define SLCHOST_INT0_ENA_OFF         0x0ECu
#define SLCHOST_INT1_ENA_OFF         0x0F0u
#define SLCHOST_LEN_WDATA0_OFF       0x0FCu
#define SLCHOST_DATE_OFF             0x178u
#define SLCHOST_ID_OFF               0x17Cu
#define SLCHOST_CONF_OFF             0x1F0u
#define SLCHOST_INF_ST_OFF           0x1F4u
#define SLCHOST_INT_VALID_MASK       0x03FFFFFFu
#define SLCHOST_INT_TOHOST_MASK      0x000000FFu
#define SLCHOST_INT_TOKEN0_EMPTY     (1u << 8)
#define SLCHOST_INT_TOKEN1_EMPTY     (1u << 9)
#define SLCHOST_INT_TOKEN0_READY     (1u << 10)
#define SLCHOST_INT_TOKEN1_READY     (1u << 11)
#define SLCHOST_INT_RX_SOF           (1u << 12)
#define SLCHOST_INT_RX_EOF           (1u << 13)
#define SLCHOST_INT_RX_START         (1u << 14)
#define SLCHOST_INT_TX_START         (1u << 15)
#define SLCHOST_INT_RX_UDF           (1u << 16)
#define SLCHOST_INT_TX_OVF           (1u << 17)
#define SLCHOST_INT_RX_NEW_PACKET    (1u << 23)

#define HINF_CFG_DATA0_OFF           0x000u
#define HINF_CFG_DATA1_OFF           0x004u
#define HINF_CFG_DATA7_OFF           0x01Cu
#define HINF_CIS_CONF0_OFF           0x020u
#define HINF_CFG_DATA16_OFF          0x040u
#define HINF_DATE_OFF                0x0FCu
#define HINF_SDIO_ENABLE             (1u << 0)
#define HINF_SDIO_IOREADY1           (1u << 1)
#define HINF_SDIO_INT_MASK           (1u << 6)
#define HINF_SDIO_RESET              (1u << 16)

#define SLC_CONF0_RESET              0xFF3CFF30u
#define SLC_CONF1_RESET              0x00300078u
#define SLC_RX_DSCR_CONF_RESET       0x101B101Au
#define SLC_DATE_RESET               0x16022500u
#define SLC_ID_RESET                 0x00000100u
#define SLCHOST_DATE_RESET           0x16022500u
#define SLCHOST_ID_RESET             0x00000600u
#define HINF_CFG_DATA0_RESET         0x22226666u
#define HINF_CFG_DATA1_RESET         0x01110011u
#define HINF_CFG_DATA7_RESET         0x00020000u
#define HINF_CFG_DATA16_RESET        0x33336666u
#define HINF_DATE_RESET              0x15030200u

/* Classic ESP32 DesignWare SD/MMC host. Two logical slots share one command,
 * FIFO, and internal-DMA engine. The register block extends through the clock
 * register at +0x800 but remains inside one 4 KiB APB page. */
#define SDMMC_SLOT_COUNT             2u
#define SDMMC_REG_FILE_SIZE          0x804u
#define SDMMC_FIFO_WORDS             32u
#define SDMMC_INTR_SOURCE            37
#define SDMMC_DMA_MAX_DESCRIPTORS    256u
#define SDMMC_TRANSFER_MAX           (16u * 1024u * 1024u)

#define SDMMC_CTRL_OFF               0x000u
#define SDMMC_PWREN_OFF              0x004u
#define SDMMC_CLKDIV_OFF             0x008u
#define SDMMC_CLKSRC_OFF             0x00Cu
#define SDMMC_CLKENA_OFF             0x010u
#define SDMMC_TMOUT_OFF              0x014u
#define SDMMC_CTYPE_OFF              0x018u
#define SDMMC_BLKSIZ_OFF             0x01Cu
#define SDMMC_BYTCNT_OFF             0x020u
#define SDMMC_INTMASK_OFF            0x024u
#define SDMMC_CMDARG_OFF             0x028u
#define SDMMC_CMD_OFF                0x02Cu
#define SDMMC_RESP0_OFF              0x030u
#define SDMMC_RESP1_OFF              0x034u
#define SDMMC_RESP2_OFF              0x038u
#define SDMMC_RESP3_OFF              0x03Cu
#define SDMMC_MINTSTS_OFF            0x040u
#define SDMMC_RINTSTS_OFF            0x044u
#define SDMMC_STATUS_OFF             0x048u
#define SDMMC_FIFOTH_OFF             0x04Cu
#define SDMMC_CDETECT_OFF            0x050u
#define SDMMC_WRTPRT_OFF             0x054u
#define SDMMC_GPIO_OFF               0x058u
#define SDMMC_TCBCNT_OFF             0x05Cu
#define SDMMC_TBBCNT_OFF             0x060u
#define SDMMC_DEBNCE_OFF             0x064u
#define SDMMC_USRID_OFF              0x068u
#define SDMMC_VERID_OFF              0x06Cu
#define SDMMC_HCON_OFF               0x070u
#define SDMMC_UHS_OFF                0x074u
#define SDMMC_RST_N_OFF              0x078u
#define SDMMC_BMOD_OFF               0x080u
#define SDMMC_PLDMND_OFF             0x084u
#define SDMMC_DBADDR_OFF             0x088u
#define SDMMC_IDSTS_OFF              0x08Cu
#define SDMMC_IDINTEN_OFF            0x090u
#define SDMMC_DSCADDR_OFF            0x094u
#define SDMMC_BUFADDRL_OFF           0x0A0u
#define SDMMC_CARDTHRCTL_OFF         0x100u
#define SDMMC_BACK_END_POWER_OFF     0x104u
#define SDMMC_UHS_EXT_OFF            0x108u
#define SDMMC_EMMC_DDR_OFF           0x10Cu
#define SDMMC_ENABLE_SHIFT_OFF       0x110u
#define SDMMC_FIFO_OFF               0x200u
#define SDMMC_CLOCK_OFF              0x800u

#define SDMMC_CTRL_CONTROLLER_RESET  (1u << 0)
#define SDMMC_CTRL_FIFO_RESET        (1u << 1)
#define SDMMC_CTRL_DMA_RESET         (1u << 2)
#define SDMMC_CTRL_INT_ENABLE        (1u << 4)
#define SDMMC_CTRL_DMA_ENABLE        (1u << 5)
#define SDMMC_CTRL_USE_INTERNAL_DMA  (1u << 25)
#define SDMMC_CTRL_VALID_MASK        0x03FF0FFFu

#define SDMMC_CMD_INDEX_MASK         0x3Fu
#define SDMMC_CMD_RESPONSE_EXPECT    (1u << 6)
#define SDMMC_CMD_RESPONSE_LONG      (1u << 7)
#define SDMMC_CMD_DATA_EXPECTED      (1u << 9)
#define SDMMC_CMD_WRITE              (1u << 10)
#define SDMMC_CMD_SEND_AUTO_STOP     (1u << 12)
#define SDMMC_CMD_CARD_SHIFT         16u
#define SDMMC_CMD_CARD_MASK          (0x1Fu << SDMMC_CMD_CARD_SHIFT)
#define SDMMC_CMD_UPDATE_CLOCK       (1u << 21)
#define SDMMC_CMD_START              (1u << 31)

#define SDMMC_INT_CARD_DETECT        (1u << 0)
#define SDMMC_INT_RESP_ERROR         (1u << 1)
#define SDMMC_INT_CMD_DONE           (1u << 2)
#define SDMMC_INT_DATA_OVER          (1u << 3)
#define SDMMC_INT_TXDR               (1u << 4)
#define SDMMC_INT_RXDR               (1u << 5)
#define SDMMC_INT_RESP_CRC           (1u << 6)
#define SDMMC_INT_DATA_CRC           (1u << 7)
#define SDMMC_INT_RESP_TIMEOUT       (1u << 8)
#define SDMMC_INT_DATA_TIMEOUT       (1u << 9)
#define SDMMC_INT_FIFO_RUN           (1u << 11)
#define SDMMC_INT_HLE                (1u << 12)
#define SDMMC_INT_AUTO_CMD_DONE      (1u << 14)
#define SDMMC_INT_VALID_MASK         0xFFFFFFFFu

#define SDMMC_IDSTS_TX               (1u << 0)
#define SDMMC_IDSTS_RX               (1u << 1)
#define SDMMC_IDSTS_FATAL_BUS        (1u << 2)
#define SDMMC_IDSTS_DESC_UNAVAIL     (1u << 4)
#define SDMMC_IDSTS_CARD_ERROR       (1u << 5)
#define SDMMC_IDSTS_NORMAL_SUMMARY   (1u << 8)
#define SDMMC_IDSTS_ABNORMAL_SUMMARY (1u << 9)
#define SDMMC_IDSTS_VALID_MASK       0x00000337u

#define SDMMC_BMOD_SW_RESET          (1u << 0)
#define SDMMC_BMOD_ENABLE            (1u << 7)

#define SDMMC_DESC_DIC               (1u << 1)
#define SDMMC_DESC_LAST              (1u << 2)
#define SDMMC_DESC_FIRST             (1u << 3)
#define SDMMC_DESC_CHAINED           (1u << 4)
#define SDMMC_DESC_END_RING          (1u << 5)
#define SDMMC_DESC_CARD_ERROR        (1u << 30)
#define SDMMC_DESC_OWNER             (1u << 31)
#define SDMMC_DESC_SIZE_MASK         0x1FFFu
#define SDMMC_DESC_SIZE2_SHIFT       13u

#define SDMMC_STATUS_FIFO_EMPTY      (1u << 2)
#define SDMMC_STATUS_FIFO_FULL       (1u << 3)
#define SDMMC_STATUS_CARD_PRESENT    (1u << 8)
#define SDMMC_STATUS_DATA_BUSY       (1u << 9)
#define SDMMC_STATUS_DATA_FSM_BUSY   (1u << 10)
#define SDMMC_STATUS_RESP_SHIFT      11u
#define SDMMC_STATUS_FIFO_SHIFT      17u

#define SDMMC_VERID_RESET            0x5342240Au

/* Classic ESP32 TWAI controller. The peripheral is an SJA1000-compatible
 * PeliCAN core whose 8-bit registers occupy the low byte of 32-bit APB words. */
#define TWAI_REG_FILE_SIZE            0x080u
#define TWAI_INTR_SOURCE              45
#define TWAI_RX_FIFO_BYTES            64u
#define TWAI_RX_FIFO_FRAMES           32u

#define TWAI_MODE_OFF                 0x000u
#define TWAI_COMMAND_OFF              0x004u
#define TWAI_STATUS_OFF               0x008u
#define TWAI_INTERRUPT_OFF            0x00Cu
#define TWAI_INTERRUPT_ENABLE_OFF     0x010u
#define TWAI_BUS_TIMING_0_OFF         0x018u
#define TWAI_BUS_TIMING_1_OFF         0x01Cu
#define TWAI_ARB_LOST_CAPTURE_OFF     0x02Cu
#define TWAI_ERROR_CODE_CAPTURE_OFF   0x030u
#define TWAI_ERROR_WARNING_LIMIT_OFF  0x034u
#define TWAI_RX_ERROR_COUNT_OFF       0x038u
#define TWAI_TX_ERROR_COUNT_OFF       0x03Cu
#define TWAI_BUFFER_OFF               0x040u
#define TWAI_RX_MESSAGE_COUNT_OFF     0x074u
#define TWAI_CLOCK_DIVIDER_OFF        0x07Cu

#define TWAI_MODE_RESET               (1u << 0)
#define TWAI_MODE_LISTEN_ONLY         (1u << 1)
#define TWAI_MODE_SELF_TEST           (1u << 2)
#define TWAI_MODE_SINGLE_FILTER       (1u << 3)
#define TWAI_MODE_VALID_MASK          0x0Fu

#define TWAI_COMMAND_TX               (1u << 0)
#define TWAI_COMMAND_ABORT            (1u << 1)
#define TWAI_COMMAND_RELEASE_RX       (1u << 2)
#define TWAI_COMMAND_CLEAR_OVERRUN    (1u << 3)
#define TWAI_COMMAND_SELF_RX          (1u << 4)

#define TWAI_STATUS_RX_BUFFER         (1u << 0)
#define TWAI_STATUS_DATA_OVERRUN      (1u << 1)
#define TWAI_STATUS_TX_BUFFER         (1u << 2)
#define TWAI_STATUS_TX_COMPLETE       (1u << 3)
#define TWAI_STATUS_RECEIVING         (1u << 4)
#define TWAI_STATUS_TRANSMITTING      (1u << 5)
#define TWAI_STATUS_ERROR             (1u << 6)
#define TWAI_STATUS_BUS_OFF           (1u << 7)

#define TWAI_INT_RX                   (1u << 0)
#define TWAI_INT_TX                   (1u << 1)
#define TWAI_INT_ERROR                (1u << 2)
#define TWAI_INT_DATA_OVERRUN         (1u << 3)
#define TWAI_INT_ERROR_PASSIVE        (1u << 5)
#define TWAI_INT_ARB_LOST             (1u << 6)
#define TWAI_INT_BUS_ERROR            (1u << 7)
#define TWAI_INT_VALID_MASK           0xEFu

#define TWAI_FRAME_DLC_MASK           0x0Fu
#define TWAI_FRAME_SELF_RX            (1u << 4)
#define TWAI_FRAME_SINGLE_SHOT        (1u << 5)
#define TWAI_FRAME_REMOTE             (1u << 6)
#define TWAI_FRAME_EXTENDED           (1u << 7)

#define TWAI_CLOCK_EXTENDED_LAYOUT    (1u << 7)
#define TWAI_DEFAULT_EWL              96u

/* Classic ESP32 Ethernet MAC: Synopsys DesignWare GMAC, enhanced chained
 * descriptors, and Espressif's RMII/MII clock-extension block. */
#define EMAC_INTR_SOURCE               38
#define EMAC_DMA_REG_FILE_SIZE         0x058u
#define EMAC_EXT_REG_FILE_SIZE         0x100u
#define EMAC_MAC_REG_FILE_SIZE         0x0E0u
#define EMAC_MAX_FRAME_SIZE            16379u /* 14-bit length includes FCS */
#define EMAC_DMA_MAX_DESCRIPTORS       256u

#define EMAC_DMA_BUS_MODE_OFF          0x000u
#define EMAC_DMA_TX_POLL_OFF           0x004u
#define EMAC_DMA_RX_POLL_OFF           0x008u
#define EMAC_DMA_RX_BASE_OFF           0x00Cu
#define EMAC_DMA_TX_BASE_OFF           0x010u
#define EMAC_DMA_STATUS_OFF            0x014u
#define EMAC_DMA_OPMODE_OFF            0x018u
#define EMAC_DMA_INT_ENA_OFF           0x01Cu
#define EMAC_DMA_MISSED_OFF            0x020u
#define EMAC_DMA_RX_WDT_OFF            0x024u
#define EMAC_DMA_TX_CUR_DESC_OFF       0x048u
#define EMAC_DMA_RX_CUR_DESC_OFF       0x04Cu
#define EMAC_DMA_TX_CUR_BUF_OFF        0x050u
#define EMAC_DMA_RX_CUR_BUF_OFF        0x054u

#define EMAC_DMA_BUS_SW_RESET          (1u << 0)
#define EMAC_DMA_BUS_DESC_SKIP_MASK    (0x1Fu << 2)
#define EMAC_DMA_BUS_ENHANCED_DESC     (1u << 7)
#define EMAC_DMA_OP_RX_START           (1u << 1)
#define EMAC_DMA_OP_TX_START           (1u << 13)
#define EMAC_DMA_OP_FLUSH_TX           (1u << 20)

#define EMAC_DMA_ST_TX                 (1u << 0)
#define EMAC_DMA_ST_TX_STOP            (1u << 1)
#define EMAC_DMA_ST_TX_UNAVAILABLE     (1u << 2)
#define EMAC_DMA_ST_TX_JABBER          (1u << 3)
#define EMAC_DMA_ST_RX_OVERFLOW        (1u << 4)
#define EMAC_DMA_ST_TX_UNDERFLOW       (1u << 5)
#define EMAC_DMA_ST_RX                 (1u << 6)
#define EMAC_DMA_ST_RX_UNAVAILABLE     (1u << 7)
#define EMAC_DMA_ST_RX_STOP            (1u << 8)
#define EMAC_DMA_ST_RX_WATCHDOG        (1u << 9)
#define EMAC_DMA_ST_EARLY_TX           (1u << 10)
#define EMAC_DMA_ST_FATAL_BUS          (1u << 13)
#define EMAC_DMA_ST_EARLY_RX           (1u << 14)
#define EMAC_DMA_ST_ABNORMAL_SUMMARY   (1u << 15)
#define EMAC_DMA_ST_NORMAL_SUMMARY     (1u << 16)
#define EMAC_DMA_ST_NORMAL_EVENTS      (EMAC_DMA_ST_TX | \
                                        EMAC_DMA_ST_TX_UNAVAILABLE | \
                                        EMAC_DMA_ST_RX | \
                                        EMAC_DMA_ST_EARLY_RX)
#define EMAC_DMA_ST_ABNORMAL_EVENTS    (EMAC_DMA_ST_TX_STOP | \
                                        EMAC_DMA_ST_TX_JABBER | \
                                        EMAC_DMA_ST_RX_OVERFLOW | \
                                        EMAC_DMA_ST_TX_UNDERFLOW | \
                                        EMAC_DMA_ST_RX_UNAVAILABLE | \
                                        EMAC_DMA_ST_RX_STOP | \
                                        EMAC_DMA_ST_RX_WATCHDOG | \
                                        EMAC_DMA_ST_EARLY_TX | \
                                        EMAC_DMA_ST_FATAL_BUS)
#define EMAC_DMA_ST_EVENT_MASK         (EMAC_DMA_ST_NORMAL_EVENTS | \
                                        EMAC_DMA_ST_ABNORMAL_EVENTS | \
                                        EMAC_DMA_ST_NORMAL_SUMMARY | \
                                        EMAC_DMA_ST_ABNORMAL_SUMMARY)
#define EMAC_DMA_INT_ABNORMAL_SUMMARY  (1u << 15)
#define EMAC_DMA_INT_NORMAL_SUMMARY    (1u << 16)

#define EMAC_DESC_OWN                  (1u << 31)
#define EMAC_TX_DESC_IOC               (1u << 30)
#define EMAC_TX_DESC_LAST              (1u << 29)
#define EMAC_TX_DESC_FIRST             (1u << 28)
#define EMAC_TX_DESC_DISABLE_CRC       (1u << 27)
#define EMAC_TX_DESC_DISABLE_PAD       (1u << 26)
#define EMAC_TX_DESC_CHAINED           (1u << 20)
#define EMAC_TX_DESC_END_RING          (1u << 21)
#define EMAC_TX_DESC_ERROR             (1u << 15)
#define EMAC_TX_DESC_NO_CARRIER        (1u << 10)
#define EMAC_RX_DESC_DA_FILTER_FAIL    (1u << 30)
#define EMAC_RX_DESC_FRAME_LEN_SHIFT   16u
#define EMAC_RX_DESC_FRAME_LEN_MASK    (0x3FFFu << 16)
#define EMAC_RX_DESC_ERROR             (1u << 15)
#define EMAC_RX_DESC_VLAN              (1u << 10)
#define EMAC_RX_DESC_FIRST             (1u << 9)
#define EMAC_RX_DESC_LAST              (1u << 8)
#define EMAC_RX_DESC_CHAINED           (1u << 14)
#define EMAC_RX_DESC_END_RING          (1u << 15)
#define EMAC_RX_DESC_DISABLE_IRQ       (1u << 31)
#define EMAC_DESC_BUF1_SIZE_MASK       0x1FFFu
#define EMAC_DESC_BUF2_SIZE_SHIFT      16u
#define EMAC_DESC_BUF2_SIZE_MASK       (0x1FFFu << 16)

#define EMAC_MAC_CONFIG_OFF            0x000u
#define EMAC_MAC_FRAME_FILTER_OFF      0x004u
#define EMAC_MAC_HASH_HIGH_OFF         0x008u
#define EMAC_MAC_HASH_LOW_OFF          0x00Cu
#define EMAC_MAC_MII_ADDR_OFF          0x010u
#define EMAC_MAC_MII_DATA_OFF          0x014u
#define EMAC_MAC_FLOW_CTRL_OFF         0x018u
#define EMAC_MAC_DEBUG_OFF             0x024u
#define EMAC_MAC_INT_STATUS_OFF        0x038u
#define EMAC_MAC_INT_MASK_OFF          0x03Cu
#define EMAC_MAC_ADDR0_HIGH_OFF        0x040u
#define EMAC_MAC_ADDR0_LOW_OFF         0x044u
#define EMAC_MAC_ADDR_LAST_OFF         0x07Cu
#define EMAC_MAC_STATUS_OFF            0x0D8u
#define EMAC_MAC_WATCHDOG_OFF          0x0DCu

#define EMAC_MAC_CONFIG_RX             (1u << 2)
#define EMAC_MAC_CONFIG_TX             (1u << 3)
#define EMAC_MAC_CONFIG_LOOPBACK       (1u << 12)
#define EMAC_MAC_CONFIG_DUPLEX         (1u << 11)
#define EMAC_MAC_CONFIG_FAST_SPEED     (1u << 14)
#define EMAC_MAC_CONFIG_MII            (1u << 15)
#define EMAC_MAC_FILTER_PROMISCUOUS    (1u << 0)
#define EMAC_MAC_FILTER_HASH_UNICAST   (1u << 1)
#define EMAC_MAC_FILTER_HASH_MULTICAST (1u << 2)
#define EMAC_MAC_FILTER_DA_INVERSE     (1u << 3)
#define EMAC_MAC_FILTER_ALL_MULTICAST  (1u << 4)
#define EMAC_MAC_FILTER_BLOCK_BCAST    (1u << 5)
#define EMAC_MAC_FILTER_CTRL_MASK      (3u << 6)
#define EMAC_MAC_FILTER_SA_INVERSE     (1u << 8)
#define EMAC_MAC_FILTER_SA_ENABLE      (1u << 9)
#define EMAC_MAC_FILTER_RECEIVE_ALL    (1u << 31)

#define EMAC_MII_BUSY                  (1u << 0)
#define EMAC_MII_WRITE                 (1u << 1)
#define EMAC_MII_REG_SHIFT             6u
#define EMAC_MII_PHY_SHIFT             11u

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

#define I2C_SLAVE_ADDR_OFF   0x010u
#define I2C_CTR_MS_MODE      (1u << 4)   /* 1 = master, 0 = slave */
#define I2C_CTR_TRANS_START  (1u << 5)
#define I2C_SR_SLAVE_ADDRESSED (1u << 5)
#define I2C_SLAVE_ADDR_MASK  0x7FFFu
#define I2C_SLAVE_ADDR_10BIT (1u << 31)
#define I2C_FIFO_RX_RST      (1u << 12)
#define I2C_FIFO_TX_RST      (1u << 13)

#define I2C_INT_RXFIFO_FULL  (1u << 0)
#define I2C_INT_TXFIFO_EMPTY (1u << 1)
#define I2C_INT_RXFIFO_OVF   (1u << 2)
#define I2C_INT_END_DETECT   (1u << 3)
#define I2C_INT_SLAVE_TRAN_COMP (1u << 4)
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

/* Deadline producers share one scheduler. This single registry generates the
 * source IDs and the two direct-call hot paths below. A producer stays a
 * candidate until its next-fire query reports no armed event, then sleeps
 * until its own state transition marks it dirty again. The final field says
 * that a free-running model still needs evaluation without a wake deadline. */
#define PERIPH_EVENT_SOURCE_LIST(X) \
    X(RTC_CNTL, target_rtc_cntl_next_fire,                         \
                target_rtc_cntl_eval_events, false)                \
    X(SYSTIMER, systimer_next_fire, systimer_eval_events, false) \
    X(TIMER_GROUP, target_timer_group_next_fire,                    \
                   target_timer_group_eval_events, false)          \
    X(TIMG,     timg_next_fire,     timg_eval_events,     false) \
    X(LACT,     lact_next_fire,     lact_eval_events,     true)  \
    X(FRC,      frc_next_fire,      frc_eval_events,      false) \
    X(UHCI,     uhci_next_fire,     uhci_eval_events,     false) \
    X(SDMMC,    sdmmc_next_fire,    sdmmc_eval_events,    false) \
    X(TWAI,     twai_next_fire,     twai_eval_events,     false) \
    X(I2S,      i2s_next_fire,      i2s_eval_events,      false) \
    X(RMT,      rmt_next_fire,      rmt_eval_events,      false) \
    X(LEDC,     ledc_next_fire,     ledc_eval_events,     true)  \
    X(PCNT,     pcnt_next_fire,     pcnt_eval_events,     false) \
    X(MCPWM,    mcpwm_next_fire,    mcpwm_eval_events,    true)  \
    X(DEFERRED, deferred_next_fire, deferred_eval_events, false)

typedef enum {
#define PERIPH_EVENT_ENUM(name, next, eval, continuous) PERIPH_EVENT_##name,
    PERIPH_EVENT_SOURCE_LIST(PERIPH_EVENT_ENUM)
#undef PERIPH_EVENT_ENUM
    PERIPH_EVENT_SOURCE_COUNT,
} periph_event_source_t;

#define PERIPH_EVENT_ALL_MASK \
    ((1u << PERIPH_EVENT_SOURCE_COUNT) - 1u)

static uint32_t default_read(void *ctx, uint32_t addr);
static void default_write(void *ctx, uint32_t addr, uint32_t val);
static void system_clock_gate_changed(
    void *ctx, flexe_system_device_t device, unsigned instance,
    bool clock_enabled, bool reset_asserted);
static uint32_t target_rtc_cntl_next_fire(esp32_periph_t *p,
                                          xtensa_cpu_t *cpu);
static void target_rtc_cntl_eval_events(esp32_periph_t *p,
                                        xtensa_cpu_t *cpu);
static void target_rtc_cntl_state_changed(void *ctx);
static void target_rtc_cntl_irq_changed(void *ctx, bool level);
static void target_rtc_cntl_reset_requested(
    void *ctx, flexe_rtc_cntl_wdt_action_t action);
static uint32_t systimer_next_fire(esp32_periph_t *p, xtensa_cpu_t *cpu);
static void systimer_eval_events(esp32_periph_t *p, xtensa_cpu_t *cpu);
static void systimer_state_changed(void *ctx);
static void systimer_irq_changed(void *ctx, unsigned alarm, bool level);
static uint32_t target_timer_group_next_fire(esp32_periph_t *p,
                                             xtensa_cpu_t *cpu);
static void target_timer_group_eval_events(esp32_periph_t *p,
                                           xtensa_cpu_t *cpu);
static void target_timer_group_state_changed(void *ctx);
static void target_timer_group_irq_changed(void *ctx, unsigned group,
                                           unsigned event, bool level);
static void target_timer_group_reset_requested(
    void *ctx, unsigned group, flexe_timer_group_wdt_action_t action);
static void usb_serial_jtag_irq_changed(void *ctx, bool level);
static uint32_t uhci_next_fire(esp32_periph_t *p, xtensa_cpu_t *cpu);
static void uhci_eval_events(esp32_periph_t *p, xtensa_cpu_t *cpu);
static void uhci_reset_state(esp32_periph_t *p, unsigned port);
static void uhci_dport_update(esp32_periph_t *p);
static void sdio_slave_reset_state(esp32_periph_t *p);
static void sdio_slave_dport_update(esp32_periph_t *p);
static size_t uhci_uart_rx_feed(esp32_periph_t *p, int uart_num,
                                const uint8_t *data, size_t len,
                                bool idle_after);
static bool uhci_uart_rx_break(esp32_periph_t *p, int uart_num);
static uint32_t sdmmc_next_fire(esp32_periph_t *p, xtensa_cpu_t *cpu);
static void sdmmc_eval_events(esp32_periph_t *p, xtensa_cpu_t *cpu);
static void sdmmc_reset_state(esp32_periph_t *p);
static void sdmmc_dport_update(esp32_periph_t *p);
static uint32_t twai_next_fire(esp32_periph_t *p, xtensa_cpu_t *cpu);
static void twai_eval_events(esp32_periph_t *p, xtensa_cpu_t *cpu);
static void twai_reset_state(esp32_periph_t *p);
static void twai_dport_update(esp32_periph_t *p);
static void emac_reset_state(esp32_periph_t *p);
static void emac_dport_update(esp32_periph_t *p);
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
static void sigmadelta_reset_state(esp32_periph_t *p);
static void sigmadelta_emit_channel(esp32_periph_t *p, unsigned channel,
                                    bool force);
static void sigmadelta_gpio_route_changed(esp32_periph_t *p,
                                          uint32_t before, uint32_t after);
static void sigmadelta_gpio_enable_changed(esp32_periph_t *p);
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
static uint32_t deferred_next_fire(esp32_periph_t *p, xtensa_cpu_t *cpu);
static void deferred_eval_events(esp32_periph_t *p, xtensa_cpu_t *cpu);
static void deferred_kick(esp32_periph_t *p);
static void periph_event_source_changed(esp32_periph_t *p,
                                        periph_event_source_t source);

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
} periph_clock_t;

typedef periph_clock_t timg_clock_state_t;

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
 * Like the general-purpose timers, LACT is clocked from the fixed 80 MHz APB
 * domain.  last_cycles/tick_remainder convert the shared CPU CCOUNT timeline
 * without losing fractional APB/divider ticks. */
typedef struct {
    uint32_t config;        /* LACTCONFIG */
    uint32_t rtc;           /* LACTRTC */
    uint64_t counter;       /* live 64-bit timer value */
    uint64_t alarm;         /* LACTALARMHI:LO */
    uint64_t load;          /* LACTLOADHI:LO pending value */
    uint64_t last_cycles;   /* shared CPU-cycle timeline at last sync */
    uint64_t tick_remainder;
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

typedef struct {
    uint32_t config;              /* duty[7:0], prescaler[15:8] */
    uint64_t anchor_cycles;
    uint8_t anchor_accumulator;
    bool anchor_level;

    periph_sigmadelta_output_fn output_cb;
    void *output_cb_ctx;
    int last_gpio;
    uint32_t last_frequency_hz;
    int8_t last_duty;
    bool last_enabled;
    bool last_inverted;
    bool output_reported;
} sigmadelta_channel_state_t;

typedef struct {
    sigmadelta_channel_state_t channel[SIGMADELTA_CHANNEL_COUNT];
    uint32_t cg;
    uint32_t misc;
    uint32_t version;

    /* Shared monotonic CPU-cycle timeline. Sampling both cores and taking the
     * maximum avoids advancing an always-on APB peripheral twice. */
    periph_clock_t clock;
} sigmadelta_state_t;

/* RTC calibration state machine per timer group */
typedef struct {
    int      cal_started;    /* write to RTCCALICFG detected */
    int      reads_since;    /* reads since cal_started */
} rtc_cal_state_t;

typedef struct {
    uint32_t config;
    uint32_t timeout;
    uint32_t result;
    unsigned reads_since;
    bool active;
    bool ready;
} target_rtc_cal_state_t;

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
    uint32_t bt_mac[PAGE_WORDS];
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
    uint32_t regs[UHCI_REG_FILE_SIZE / sizeof(uint32_t)];
    uint32_t int_raw;
    uint32_t int_ena;

    uint32_t tx_desc;
    uint32_t rx_desc;
    size_t rx_offset;
    uint32_t rx_ctrl;
    uint32_t rx_buf;
    uint32_t rx_next;
    size_t rx_size;
    bool rx_desc_loaded;
    unsigned tx_descriptors_seen;
    unsigned rx_descriptors_seen;
    bool tx_link_running;
    bool rx_link_running;

    /* TX descriptors complete at the selected UART's configured wire rate.
     * Quick-send packets share the same ccount event source. */
    bool tx_event_armed;
    bool quick_event_armed;
    uint8_t quick_event_kind; /* 1 = single, 2 = continuous */
    uint64_t next_tx_cycle;
    uint64_t next_quick_cycle;

    /* The reset protocol is Bluetooth H:5: a four-byte header and optional
     * CRC inside configurable SLIP framing. State spans descriptors. */
    bool tx_frame_started;
    uint8_t tx_header[4];
    uint8_t tx_header_len;
    uint32_t tx_payload_count;
    uint16_t tx_crc;

    bool rx_frame_active;
    bool rx_escape_pending;
    uint8_t rx_escape_prefix;
    uint8_t rx_frame[UHCI_FRAME_MAX];
    size_t rx_frame_len;
    uint32_t rx_payload_count;
    uint8_t expected_rx_sequence;
    uint8_t rx_error_cause;
    uint8_t rx_frame_error_cause;
    uint8_t decode_state;
    uint8_t encode_state;

    /* APB debug FIFO access. DMA normally drains/fills these immediately,
     * but retaining the receive side makes IN_POP and watermark behavior
     * observable to low-level diagnostics. */
    uint16_t debug_in_fifo[UHCI_DEBUG_FIFO_SIZE];
    uint16_t debug_in_head;
    uint16_t debug_in_count;
    uint16_t debug_in_last;
} uhci_state_t;

typedef struct {
    uint32_t slc[SLC_REG_FILE_SIZE / sizeof(uint32_t)];
    uint32_t host[SLCHOST_REG_FILE_SIZE / sizeof(uint32_t)];
    uint32_t hinf[HINF_REG_FILE_SIZE / sizeof(uint32_t)];
    uint32_t int_raw[SLC_CHANNEL_COUNT];
    uint32_t int_ena[SLC_CHANNEL_COUNT];
    uint32_t host_int_raw[SLC_CHANNEL_COUNT];
    uint16_t token[SLC_CHANNEL_COUNT][2];
    uint32_t packet_len[SLC_CHANNEL_COUNT];
    uint32_t host_consumed_len[SLC_CHANNEL_COUNT];

    /* SLC RX pulls slave memory toward the host; SLC TX fills slave memory
     * from host traffic. Addresses are full reconstructed DRAM addresses. */
    uint32_t rx_desc[SLC_CHANNEL_COUNT];
    uint32_t tx_desc[SLC_CHANNEL_COUNT];
    uint32_t rx_last_desc[SLC_CHANNEL_COUNT];
    uint32_t tx_last_desc[SLC_CHANNEL_COUNT];
    bool rx_link_running[SLC_CHANNEL_COUNT];
    bool tx_link_running[SLC_CHANNEL_COUNT];
} sdio_slave_state_t;

typedef struct {
    bool attached;
    bool write_protected;
    bool ready;
    bool selected;
    bool app_cmd;
    uint16_t rca;
    uint32_t sector_count;
    uint32_t block_len;
    periph_sdmmc_read_blocks_fn read_fn;
    periph_sdmmc_write_blocks_fn write_fn;
    void *ctx;
} sdmmc_card_state_t;

typedef struct {
    uint32_t regs[(SDMMC_REG_FILE_SIZE + 3u) / sizeof(uint32_t)];
    uint32_t rintsts;
    uint32_t idsts;
    uint32_t fifo[SDMMC_FIFO_WORDS];
    uint8_t fifo_head;
    uint8_t fifo_count;

    sdmmc_card_state_t card[SDMMC_SLOT_COUNT];

    uint8_t *transfer;
    size_t transfer_capacity;
    size_t transfer_total;
    size_t transfer_pos;
    uint32_t transfer_lba;
    uint32_t transfer_desc;
    unsigned transfer_descriptors_seen;
    uint8_t transfer_slot;
    uint8_t transfer_cmd;
    bool transfer_write;
    bool transfer_block_io;
    bool transfer_auto_stop;
    bool transfer_dma;
    bool transfer_active;
    bool transfer_wait_command_ack;
    bool transfer_event_armed;
    uint32_t next_transfer_ccount;
} sdmmc_state_t;

typedef struct {
    uint8_t bytes[13];
    uint8_t storage_bytes;
} twai_rx_entry_t;

typedef struct {
    uint8_t mode;
    uint8_t int_raw;
    uint8_t int_ena;
    uint8_t bus_timing_0;
    uint8_t bus_timing_1;
    uint8_t arbitration_lost_capture;
    uint8_t error_code_capture;
    uint8_t error_warning_limit;
    uint16_t rx_error_count;
    uint16_t tx_error_count;
    uint8_t acceptance_code[4];
    uint8_t acceptance_mask[4];
    uint8_t clock_divider;

    uint8_t tx_buffer[13];
    periph_twai_frame_t tx_frame;
    bool tx_busy;
    bool tx_complete;
    bool tx_event_armed;
    bool data_overrun;
    bool bus_off;
    uint64_t next_tx_cycle;

    twai_rx_entry_t rx_fifo[TWAI_RX_FIFO_FRAMES];
    uint8_t rx_head;
    uint8_t rx_count;
    uint8_t rx_bytes;

    bool recovery_event_armed;
    uint8_t recovery_phase;
    uint64_t next_recovery_cycle;

    periph_twai_tx_fn tx_cb;
    void *tx_cb_ctx;
} twai_state_t;

typedef struct {
    uint32_t dma[EMAC_DMA_REG_FILE_SIZE / sizeof(uint32_t)];
    uint32_t ext[EMAC_EXT_REG_FILE_SIZE / sizeof(uint32_t)];
    uint32_t mac[EMAC_MAC_REG_FILE_SIZE / sizeof(uint32_t)];
    uint32_t dma_status;
    uint32_t tx_current_desc;
    uint32_t rx_current_desc;

    uint8_t tx_frame[EMAC_MAX_FRAME_SIZE];
    uint16_t phy_regs[32][32];
    uint32_t phy_present;

    periph_emac_tx_fn tx_cb;
    void *tx_cb_ctx;
    periph_emac_mdio_fn mdio_cb;
    void *mdio_cb_ctx;
} emac_state_t;

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
    bool slave_addressed;

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
    uint32_t regs[RTC_I2C_REG_FILE_SIZE / sizeof(uint32_t)];
    uint32_t int_raw;
    uint32_t int_ena;
    uint8_t data;
    bool done;
    i2c_device_t device[I2C_DEVICE_COUNT];
} rtc_i2c_state_t;

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
    uint64_t next_tx_cycle;
    uint64_t next_rx_cycle;

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
    /* Native ping-pong drivers refill RAM before acknowledging THRESHOLD. */
    bool tx_waiting_for_threshold_clear;
    uint64_t next_tx_cycle;

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
    uint64_t deadline;
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
    periph_clock_t clock;
} mcpwm_state_t;

typedef struct {
    periph_deferred_fn fn;
    void *ctx;
    uint64_t deadline;
    bool armed;
} periph_deferred_event_t;

struct esp32_periph {
    /* Set when firmware or a modeled watchdog requests a system reset. */
    bool reset_requested;
    xtensa_mem_t *mem;
    const flexe_target_desc_t *target;

    /* Target-specific external-memory devices. Classic ESP32 retains the
     * mature DPORT cache/MMU model below; shared-MMU targets compose these
     * independent devices instead of inheriting classic register aliases. */
    flexe_flash_mmu_t *shared_flash_mmu;
    flexe_esp32s3_extmem_t *s3_extmem;
    flexe_efuse_t *target_efuse;
    flexe_io_mux_t *io_mux;
    flexe_rtc_cntl_t *target_rtc_cntl;
    flexe_regi2c_t *regi2c;
    flexe_sensitive_memprot_t *sensitive_memprot;
    flexe_system_clock_t *system_clock;
    flexe_systimer_t *systimer;
    flexe_timer_group_t *target_timer_group;
    flexe_spi_mem_t *spi_mem;
    flexe_usb_serial_jtag_t *usb_serial_jtag;

    /* Three independent ESP32 UART controllers. */
    uart_state_t uart[UART_COUNT];

    /* Two classic UART DMA engines, each attachable to UART0, UART1, or
     * UART2 through UHCI_CONF0.UARTx_CE. */
    uhci_state_t uhci[UHCI_PORT_COUNT];

    /* External SDIO card/slave endpoint: HINF + SLCHOST + dual-channel SLC. */
    sdio_slave_state_t sdio_slave;

    /* Two independent classic ESP32 I2C controllers and their virtual bus
     * targets. */
    i2c_state_t i2c[I2C_PORT_COUNT];

    /* Always-on RTC-domain I2C is the ULP's independent third bus. */
    rtc_i2c_state_t rtc_i2c;

    /* GPIO */
    gpio_state_t gpio;

    /* Eight always-on classic GPIO sigma-delta/PDM channels. */
    sigmadelta_state_t sigmadelta;

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

    /* Target-described RTC calibration blocks used when the surrounding
     * timer-group register layout is not the classic ESP32 layout. */
    target_rtc_cal_state_t target_rtc_cal[FLEXE_TARGET_RTC_CAL_GROUP_MAX];

    /* APP_CPU reset/stall/clock state from classic DPORT or the target's
     * descriptor-driven secondary-core controller. */
    bool app_cpu_in_reset;
    uint32_t secondary_core_control;
    uint32_t secondary_core_boot_addr;

    /* DPORT peripheral clock/reset register shadows.  Individual modeled
     * peripherals apply their reset semantics when their bit is asserted. */
    uint32_t dport_perip_clk_en;
    uint32_t dport_perip_rst_en;
    uint32_t dport_wifi_clk_en;
    uint32_t dport_core_rst_en;
    uint32_t dport_cpu_per_conf;

    /* Interrupt matrix: each peripheral source owns one five-bit CPU interrupt
     * selection register per core.  Multiple sources may feed the same CPU
     * line, so store the hardware's source -> CPU-interrupt direction. */
    uint8_t intr_matrix[FLEXE_TARGET_INTERRUPT_CORE_MAX]
                       [FLEXE_TARGET_INTERRUPT_SOURCE_MAX];

    /* Pending peripheral interrupt sources (level-triggered) */
    uint32_t pending_sources[(FLEXE_TARGET_INTERRUPT_SOURCE_MAX + 31u) / 32u];

    /* Compatibility-mode guest ISR dispatch. source_level supplies edge
     * detection independently of interrupt-matrix routing.  Per-core levels
     * retain GPIO's asymmetric routing and make fan-in/remapping coherent. */
    bool source_level[FLEXE_TARGET_INTERRUPT_SOURCE_MAX];
    bool source_level_core[FLEXE_TARGET_INTERRUPT_CORE_MAX]
                          [FLEXE_TARGET_INTERRUPT_SOURCE_MAX];
    /* Last enabled-status mask seen for each source, so a *new* condition
     * arriving while the line is already high can be re-dispatched. See
     * periph_assert_interrupt_status(). */
    uint32_t source_status[FLEXE_TARGET_INTERRUPT_SOURCE_MAX];
    periph_irq_dispatch_fn irq_dispatch[FLEXE_TARGET_INTERRUPT_SOURCE_MAX];
    void *irq_dispatch_ctx[FLEXE_TARGET_INTERRUPT_SOURCE_MAX];

    /* Target-described matrix register state. Classic ESP32 keeps these
     * controls inside DPORT and therefore does not use these shadows. */
    uint32_t intr_matrix_clock_gate[FLEXE_TARGET_INTERRUPT_CORE_MAX];
    uint32_t intr_matrix_date[FLEXE_TARGET_INTERRUPT_CORE_MAX];

    /* CPU pointers for interrupt delivery */
    xtensa_cpu_t *cpu[2];

    /* Target-selected producers and the per-core subset which may currently
     * be armed.  Keep registration separate from candidacy: continuously
     * sampled models still need to run after their deadline query sleeps,
     * but must never run for a target which did not register the device. */
    uint32_t event_source_registered_mask;
    uint32_t event_source_candidates[2];

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
    uint32_t rtc_cpu_period_conf;
    uint32_t rtc_clk_conf;
    uint16_t sens_adc_result[2];
    bool sens_adc_done[2];
    /* Touch: per-pad count injected by the host, and the latched
     * below-threshold status the guest reads back out of CTRL2. */
    uint16_t touch_value[SENS_TOUCH_PAD_COUNT];
    uint32_t touch_status;
    uint32_t rtc_int_raw;
    uint32_t rtc_int_ena;
    /* Sleep. rtc_regs holds the RTC_CNTL words that are plain storage
     * (STORE0-3, the wake config, DIG_PWC); the rest are computed. */
    uint32_t rtc_state0;
    uint32_t rtc_store[4];
    uint32_t rtc_ext_wakeup_conf;
    uint32_t rtc_ext_wakeup1;
    uint32_t rtc_ext_wakeup0;      /* RTCIO 0xBC */
    uint32_t rtc_dig_pwc;
    uint64_t rtc_slp_target;       /* SLP_TIMER0/1, in slow-clock ticks */
    uint64_t rtc_time_latched;     /* snapshot taken on TIME_UPDATE */
    uint32_t rtc_wakeup_cause;
    uint32_t rtc_wakeup_ena;
    uint32_t rtc_reset_cause;
    bool     sleep_requested;
    bool     sleep_deep;

    /* Radio/PHY register state used by the closed-source WiFi/BT HAL. */
    radio_state_t radio;

    /* Shared cycle timeline for the deadline-driven peripheral models. */
    periph_clock_t event_clock;

    /* Timed completions owned by board-level devices attached to this SoC. */
    periph_deferred_event_t deferred[PERIPH_DEFERRED_MAX];

    /* Classic ESP32 high/low-speed LEDC timers, channels, and timed fades. */
    ledc_state_t ledc;
    periph_clock_t ledc_clock;

    /* Eight two-channel classic ESP32 pulse-counter units. */
    pcnt_state_t pcnt;

    /* Two classic ESP32 motor-control PWM units. */
    mcpwm_state_t mcpwm;

    /* Two independent classic ESP32 I2S controllers with circular DMA. */
    i2s_state_t i2s[I2S_PORT_COUNT];

    /* Dual-slot native SD/MMC host, FIFO, and DesignWare internal DMA. */
    sdmmc_state_t sdmmc;

    /* SJA1000-compatible classic ESP32 TWAI/CAN controller and RX FIFO. */
    twai_state_t twai;

    /* Synopsys GMAC DMA/MAC plus Espressif's MII/RMII extension registers. */
    emac_state_t emac;

    /* Eight-channel classic ESP32 remote-control/pulse engine. */
    rmt_state_t rmt;

    /* Flash cache MMU table shadows (DPORT_PRO/APP_FLASH_MMU_TABLE).
     * Entries 0-63 are DROM0; 64-127, 128-191, and 192-255 are the
     * IRAM0, IRAM1, and IROM0 cache regions respectively. The full 2048-word
     * register files remain readable and writable for startup-time mirroring. */
    uint32_t flash_mmu_pro[FLASH_MMU_REGISTER_COUNT];
    uint32_t flash_mmu_app[FLASH_MMU_REGISTER_COUNT];
    uint16_t flash_mmu_effective[FLASH_MMU_ENTRY_COUNT];
};

static void periph_event_source_changed(esp32_periph_t *p,
                                        periph_event_source_t source) {
    if (!p || (unsigned)source >= PERIPH_EVENT_SOURCE_COUNT) return;
    uint32_t bit = 1u << (unsigned)source;
    p->event_source_candidates[0] |= bit;
    p->event_source_candidates[1] |= bit;
}

/* ===== Shared monotonic peripheral cycle clock =====
 *
 * The two emulated cores advance their own CCOUNT independently:
 * flexe_session_post_batch() syncs cycle_count between them but deliberately
 * not ccount, and esp_timer's advance_wait_time() bumps only the calling
 * core. A model that samples cpu[0] alone therefore freezes whenever the task
 * driving it runs on core 1 -- which is where Arduino's loopTask lives.
 *
 * Accumulate elapsed ccount per core and publish the maximum. Sampling is
 * idempotent and monotone, so both cores may sample freely without ever
 * advancing the peripheral twice.
 *
 * NOTE: periph_clock_now() mutates state, so a *_next_fire() built on it is
 * not a pure query; sampling from either core moves the shared timeline. */
static uint64_t periph_clock_now(esp32_periph_t *p, periph_clock_t *c) {
    for (unsigned core = 0; core < 2u; core++) {
        if (!p->cpu[core]) continue;
        uint32_t now = p->cpu[core]->ccount;
        if (!c->valid[core]) {
            c->last_ccount[core] = now;
            c->core_cycles[core] = c->cycles;
            c->valid[core] = true;
            continue;
        }
        uint32_t elapsed = now - c->last_ccount[core];
        if (elapsed < (uint32_t)INT32_MAX) {
            if (c->core_cycles[core] > UINT64_MAX - elapsed)
                c->core_cycles[core] = UINT64_MAX;
            else
                c->core_cycles[core] += elapsed;
        }
        c->last_ccount[core] = now;
        if (c->core_cycles[core] > c->cycles)
            c->cycles = c->core_cycles[core];
    }
    return c->cycles;
}

/* Read the shared timeline without advancing it, for callers that must stay
 * pure queries. Never lower than the last periph_clock_now(). */
static uint64_t periph_clock_peek(const esp32_periph_t *p,
                                  const periph_clock_t *c) {
    uint64_t best = c->cycles;
    for (unsigned core = 0; core < 2u; core++) {
        if (!p->cpu[core] || !c->valid[core]) continue;
        uint32_t elapsed = p->cpu[core]->ccount - c->last_ccount[core];
        if (elapsed >= (uint32_t)INT32_MAX) continue;
        uint64_t candidate = c->core_cycles[core] > UINT64_MAX - elapsed ?
            UINT64_MAX : c->core_cycles[core] + elapsed;
        if (candidate > best) best = candidate;
    }
    return best;
}

/* Translate a deadline on the shared timeline into `cpu`'s 32-bit CCOUNT
 * frame. next_timer_event is compared against that core's ccount, so a
 * deadline must never be handed back in shared-timeline units. */
static uint32_t periph_deadline_ccount(esp32_periph_t *p, periph_clock_t *c,
                                       const xtensa_cpu_t *cpu,
                                       uint64_t deadline) {
    uint64_t now = periph_clock_now(p, c);
    uint64_t delta = deadline > now ? deadline - now : 0;
    if (delta > (uint64_t)INT32_MAX) delta = (uint64_t)INT32_MAX;
    return cpu->ccount + (uint32_t)delta;
}

static void deferred_kick(esp32_periph_t *p) {
    periph_event_source_changed(p, PERIPH_EVENT_DEFERRED);
    for (int core = 0; core < 2; core++)
        if (p->cpu[core]) xtensa_recompute_next_timer(p->cpu[core]);
}

static uint32_t deferred_next_fire(esp32_periph_t *p, xtensa_cpu_t *cpu) {
    if (!p || !cpu) return UINT32_MAX;
    bool have = false;
    uint64_t best = 0;
    for (size_t i = 0; i < PERIPH_DEFERRED_MAX; i++) {
        if (!p->deferred[i].armed) continue;
        if (!have || p->deferred[i].deadline < best) {
            have = true;
            best = p->deferred[i].deadline;
        }
    }
    return have ? periph_deadline_ccount(p, &p->event_clock, cpu, best) :
                  UINT32_MAX;
}

static void deferred_eval_events(esp32_periph_t *p, xtensa_cpu_t *cpu) {
    if (!p || !cpu) return;
    uint64_t now = periph_clock_now(p, &p->event_clock);
    for (size_t i = 0; i < PERIPH_DEFERRED_MAX; i++) {
        periph_deferred_event_t *event = &p->deferred[i];
        if (!event->armed || event->deadline > now) continue;
        periph_deferred_fn fn = event->fn;
        void *ctx = event->ctx;
        event->armed = false;
        if (fn) fn(ctx);
    }
    deferred_kick(p);
}

int periph_schedule_deferred_us(esp32_periph_t *p, uint64_t delay_us,
                                periph_deferred_fn fn, void *ctx) {
    if (!p || !fn) return -1;
    int slot = -1;
    for (size_t i = 0; i < PERIPH_DEFERRED_MAX; i++) {
        if (p->deferred[i].fn == fn && p->deferred[i].ctx == ctx) {
            slot = (int)i;
            break;
        }
        if (slot < 0 && !p->deferred[i].armed)
            slot = (int)i;
    }
    if (slot < 0) return -1;

    xtensa_cpu_t *clock_cpu = p->cpu[0] ? p->cpu[0] : p->cpu[1];
    uint64_t mhz = clock_cpu ? xtensa_cpu_freq_mhz(clock_cpu) : 160u;
    uint64_t delay_cycles = delay_us > UINT64_MAX / mhz ?
                            UINT64_MAX : delay_us * mhz;
    uint64_t now = periph_clock_now(p, &p->event_clock);
    periph_deferred_event_t *event = &p->deferred[slot];
    event->fn = fn;
    event->ctx = ctx;
    event->deadline = now > UINT64_MAX - delay_cycles ?
                      UINT64_MAX : now + delay_cycles;
    event->armed = true;
    deferred_kick(p);
    return 0;
}

void periph_cancel_deferred(esp32_periph_t *p, periph_deferred_fn fn,
                            void *ctx) {
    if (!p || !fn) return;
    for (size_t i = 0; i < PERIPH_DEFERRED_MAX; i++) {
        periph_deferred_event_t *event = &p->deferred[i];
        if (event->fn == fn && event->ctx == ctx) {
            event->armed = false;
            event->fn = NULL;
            event->ctx = NULL;
        }
    }
    deferred_kick(p);
}


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
    for (uint32_t s = 0; s < FLASH_MMU_REGISTER_COUNT; s++) {
        p->flash_mmu_pro[s] = FLASH_MMU_INVALID;
        p->flash_mmu_app[s] = FLASH_MMU_INVALID;
    }
    for (uint32_t s = 0; s < FLASH_MMU_ENTRY_COUNT; s++) {
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
#define DPORT_WIFI_CLK_EN_OFF          0x0CC
#define DPORT_CORE_RST_EN_OFF          0x0D0
#define DPORT_SDIO_HOST_CLK_BIT        (1u << 13)
#define DPORT_SDIO_HOST_RST_BIT        (1u << 6)
#define DPORT_SDIO_SLAVE_CLK_BIT       (1u << 4)
#define DPORT_SDIO_SLAVE_RST_BIT       (1u << 5)
#define DPORT_EMAC_CLK_BIT             (1u << 14)
#define DPORT_EMAC_RST_BIT             (1u << 7)
#define DPORT_UHCI0_MODULE_BIT         (1u << 8)
#define DPORT_RMT_MODULE_BIT           (1u << 9)
#define DPORT_PCNT_MODULE_BIT          (1u << 10)
#define DPORT_LEDC_MODULE_BIT          (1u << 11)
#define DPORT_UHCI1_MODULE_BIT         (1u << 12)
#define DPORT_TIMG0_MODULE_BIT         (1u << 13)
#define DPORT_TIMG1_MODULE_BIT         (1u << 15)
#define DPORT_PWM0_MODULE_BIT          (1u << 17)
#define DPORT_TWAI_MODULE_BIT          (1u << 19)
#define DPORT_PWM1_MODULE_BIT          (1u << 20)

#define DPORT_PRO_INTR_MAP_BASE_OFF    0x104u
#define DPORT_APP_INTR_MAP_BASE_OFF    0x218u
#define DPORT_INTR_MAP_SOURCE_COUNT    69u
#define ESP32_INTERRUPT_SOURCE_COUNT   71u
#define DPORT_INTR_MAP_RESET           16u

static unsigned intr_matrix_source_count(const esp32_periph_t *p)
{
    if (!p || !p->target) return 0u;
    if (p->target->capabilities & FLEXE_TARGET_CAP_INTERRUPT_MATRIX_V1)
        return p->target->interrupt_matrix.source_count;
    if (p->target->capabilities & FLEXE_TARGET_CAP_ESP32_CLASSIC_PERIPHERALS)
        return ESP32_INTERRUPT_SOURCE_COUNT;
    return 0u;
}

static unsigned intr_matrix_core_count(const esp32_periph_t *p)
{
    if (!p || !p->target) return 0u;
    unsigned count = p->target->core_count;
    return count < FLEXE_TARGET_INTERRUPT_CORE_MAX ?
        count : FLEXE_TARGET_INTERRUPT_CORE_MAX;
}

static uint32_t intr_matrix_map_reset(const esp32_periph_t *p)
{
    if (p && p->target &&
        (p->target->capabilities & FLEXE_TARGET_CAP_INTERRUPT_MATRIX_V1))
        return p->target->interrupt_matrix.map_reset;
    return DPORT_INTR_MAP_RESET;
}

static bool intr_matrix_core_enabled(const esp32_periph_t *p, int core)
{
    if (!p || core < 0 || (unsigned)core >= intr_matrix_core_count(p))
        return false;
    if (!(p->target->capabilities & FLEXE_TARGET_CAP_INTERRUPT_MATRIX_V1))
        return true;
    const flexe_interrupt_matrix_desc_t *desc =
        &p->target->interrupt_matrix;
    return desc->clock_gate_writable_mask == 0u ||
           (p->intr_matrix_clock_gate[core] &
            desc->clock_gate_writable_mask) != 0u;
}

/* Xtensa CPU interrupts 6/15/16 are internal CCOMPARE timers, 7/29 are
 * software interrupts, and 11 is the profiling interrupt.  The DPORT map
 * registers retain writes selecting those numbers, but peripheral sources
 * cannot drive or clear the corresponding core-local lines.  ESP-IDF relies
 * on this property and bulk-writes ETS_INVALID_INUM (6) to disconnect every
 * source during interrupt allocator initialization. */
static bool intr_matrix_cpu_line_is_routeable(int cpu_int) {
    const uint32_t internal = (1u << 6) | (1u << 7) | (1u << 11) |
                              (1u << 15) | (1u << 16) | (1u << 29);
    return cpu_int >= 0 && cpu_int <= 31 &&
           (internal & (1u << cpu_int)) == 0u;
}

static void intr_matrix_refresh_cpu_line(esp32_periph_t *p, int core,
                                         int cpu_int) {
    if (core < 0 || (unsigned)core >= intr_matrix_core_count(p) ||
        !intr_matrix_cpu_line_is_routeable(cpu_int) || !p->cpu[core])
        return;

    bool asserted = false;
    unsigned source_count = intr_matrix_source_count(p);
    for (unsigned source = 0;
         intr_matrix_core_enabled(p, core) && source < source_count;
         source++) {
        if (p->intr_matrix[core][source] == (uint8_t)cpu_int &&
            p->source_level_core[core][source]) {
            asserted = true;
            break;
        }
    }

    uint32_t mask = 1u << cpu_int;
    if (asserted) {
        p->cpu[core]->interrupt |= mask;
        xtensa_request_irq_check(p->cpu[core]);
    } else {
        p->cpu[core]->interrupt &= ~mask;
    }
}

static void intr_matrix_refresh_core(esp32_periph_t *p, int core)
{
    if (!p || core < 0 || (unsigned)core >= intr_matrix_core_count(p))
        return;
    for (int cpu_int = 0; cpu_int < 32; cpu_int++)
        intr_matrix_refresh_cpu_line(p, core, cpu_int);
}

static void intr_matrix_map_source(esp32_periph_t *p, int core, int source,
                                   int cpu_int) {
    if (!p || core < 0 || (unsigned)core >= intr_matrix_core_count(p) ||
        source < 0 || (unsigned)source >= intr_matrix_source_count(p) ||
        cpu_int < 0 || cpu_int > 31)
        return;
    int old_cpu_int = p->intr_matrix[core][source];
    p->intr_matrix[core][source] = (uint8_t)cpu_int;
    intr_matrix_refresh_cpu_line(p, core, old_cpu_int);
    intr_matrix_refresh_cpu_line(p, core, cpu_int);
}

/* Internal: set/clear CPU interrupt bits for one peripheral source. */
static void intr_matrix_update_source(esp32_periph_t *p, int source, bool assert) {
    if (!p || source < 0 ||
        (unsigned)source >= intr_matrix_source_count(p))
        return;
    bool rising = assert && !p->source_level[source];
    p->source_level[source] = assert;
    for (unsigned core = 0; core < intr_matrix_core_count(p); core++) {
        p->source_level_core[core][source] = assert;
        intr_matrix_refresh_cpu_line(p, (int)core,
                                     p->intr_matrix[core][source]);
    }
    if (rising && p->irq_dispatch[source])
        p->irq_dispatch[source](p->irq_dispatch_ctx[source], source);
}

/* GPIO has per-pin CPU routing, so its normal/NMI sources may be asserted on
 * one core without being asserted on the other. */
static void intr_matrix_update_source_core(esp32_periph_t *p, int core,
                                           int source, bool assert) {
    if (!p || core < 0 || (unsigned)core >= intr_matrix_core_count(p) ||
        source < 0 || (unsigned)source >= intr_matrix_source_count(p))
        return;
    p->source_level_core[core][source] = assert;
    intr_matrix_refresh_cpu_line(p, core, p->intr_matrix[core][source]);

    /* Dispatch on the combined rising edge, exactly as the whole-source path
     * does. Without this a handler registered for a per-core source is never
     * called, which is why no GPIO interrupt ever reached the guest however
     * it was registered: the pin latched status and the CPU line was raised,
     * and nothing ran. */
    bool now = false;
    for (unsigned i = 0; i < intr_matrix_core_count(p); i++)
        now = now || p->source_level_core[i][source];
    bool rising = now && !p->source_level[source];
    p->source_level[source] = now;
    if (rising && p->irq_dispatch[source])
        p->irq_dispatch[source](p->irq_dispatch_ctx[source], source);
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
    if (entry >= FLASH_MMU_REGISTER_COUNT || core < 0 || core > 1) return;
    val &= FLASH_MMU_VALUE_MASK;
    uint32_t *own = core == 0 ? p->flash_mmu_pro : p->flash_mmu_app;
    uint32_t *other = core == 0 ? p->flash_mmu_app : p->flash_mmu_pro;
    own[entry] = val;

    /* Only the first four 64-entry banks describe flash XIP buses. Preserve
     * the remainder of the hardware register window without treating its
     * reserved/PSRAM/PID banks as aliases of those virtual addresses. */
    if (entry >= FLASH_MMU_ENTRY_COUNT) return;

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

/* NOR mutations originate in the target-described SPI-memory device. Route
 * translated-code invalidation through the MMU implementation belonging to
 * this target rather than teaching the flash controller either address map. */
static void periph_flash_changed(void *ctx, uint32_t offset, uint32_t len) {
    esp32_periph_t *p = ctx;
    if (!p || len == 0u) return;
    if (p->target->capabilities &
        FLEXE_TARGET_CAP_ESP32_CLASSIC_PERIPHERALS)
        flash_mmu_invalidate_physical(p, offset, len);
    else
        flexe_flash_mmu_flash_changed(p->shared_flash_mmu, offset, len);
}

static uint32_t dport_read(void *ctx, uint32_t addr) {
    esp32_periph_t *p = ctx;
    uint32_t off = addr - DPORT_BASE;
    /* Full 8 KiB MMU register files. ESP-IDF 5.5 mirrors all 2048 words from
     * PRO to APP during multicore startup, including non-XIP banks. */
    if (off >= 0x10000 && off < 0x12000)
        return p->flash_mmu_pro[(off - 0x10000) >> 2];
    if (off >= 0x12000 && off < 0x14000)
        return p->flash_mmu_app[(off - 0x12000) >> 2];
    switch (off) {
    case 0x018: return p->app_cpu_in_reset ? 1 : 0; /* APPCPU_CTRL_D: reset state */
    case 0x02C: return p->app_cpu_in_reset ? 0 : 1; /* APPCPU_CTRL_A: clock gate */
    case 0x030: return p->app_cpu_in_reset ? 0 : 1; /* APPCPU_CTRL_B: clock enable */
    case DPORT_PERIP_CLK_EN_OFF: return p->dport_perip_clk_en;
    case DPORT_PERIP_RST_EN_OFF: return p->dport_perip_rst_en;
    case DPORT_WIFI_CLK_EN_OFF: return p->dport_wifi_clk_en;
    case DPORT_CORE_RST_EN_OFF: return p->dport_core_rst_en;
    case 0x03C: return p->dport_cpu_per_conf; /* CPU_PER_CONF */
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
        /* DPORT exposes one contiguous map register per source: 69 for PRO,
         * followed immediately by 69 for APP.  Each register contains the
         * selected CPU interrupt number, not a peripheral source number. */
        if (off >= DPORT_PRO_INTR_MAP_BASE_OFF &&
            off < DPORT_PRO_INTR_MAP_BASE_OFF +
                  DPORT_INTR_MAP_SOURCE_COUNT * 4u &&
            ((off - DPORT_PRO_INTR_MAP_BASE_OFF) & 3u) == 0u) {
            uint32_t source = (off - DPORT_PRO_INTR_MAP_BASE_OFF) / 4u;
            return p->intr_matrix[0][source];
        }
        if (off >= DPORT_APP_INTR_MAP_BASE_OFF &&
            off < DPORT_APP_INTR_MAP_BASE_OFF +
                  DPORT_INTR_MAP_SOURCE_COUNT * 4u &&
            ((off - DPORT_APP_INTR_MAP_BASE_OFF) & 3u) == 0u) {
            uint32_t source = (off - DPORT_APP_INTR_MAP_BASE_OFF) / 4u;
            return p->intr_matrix[1][source];
        }
        return 0;
    }
}

static void dport_write(void *ctx, uint32_t addr, uint32_t val) {
    esp32_periph_t *p = ctx;
    uint32_t off = addr - DPORT_BASE;
    if (off >= 0x10000 && off < 0x12000) {
        uint32_t entry = (off - 0x10000) >> 2;
        flash_mmu_write_entry(p, 0, entry, val);
        return;
    }
    if (off >= 0x12000 && off < 0x14000) {
        uint32_t entry = (off - 0x12000) >> 2;
        flash_mmu_write_entry(p, 1, entry, val);
        return;
    }
    switch (off) {
    case 0x03C:
        p->dport_cpu_per_conf = val & 0xFu;
        break;
    case DPORT_PERIP_CLK_EN_OFF:
        timg_sync_all_to(p, timg_now_cycles(p));
        p->dport_perip_clk_en = val;
        uhci_dport_update(p);
        twai_dport_update(p);
        timg_kick(p);
        break;
    case DPORT_WIFI_CLK_EN_OFF:
        p->dport_wifi_clk_en = val;
        sdio_slave_dport_update(p);
        sdmmc_dport_update(p);
        emac_dport_update(p);
        break;
    case DPORT_CORE_RST_EN_OFF:
        p->dport_core_rst_en = val;
        if (val & DPORT_SDIO_SLAVE_RST_BIT)
            sdio_slave_reset_state(p);
        if (val & DPORT_SDIO_HOST_RST_BIT)
            sdmmc_reset_state(p);
        if (val & DPORT_EMAC_RST_BIT)
            emac_reset_state(p);
        sdio_slave_dport_update(p);
        sdmmc_dport_update(p);
        emac_dport_update(p);
        break;
    case DPORT_PERIP_RST_EN_OFF:
        timg_sync_all_to(p, timg_now_cycles(p));
        p->dport_perip_rst_en = val;
        if (val & DPORT_UHCI0_MODULE_BIT)
            uhci_reset_state(p, 0);
        if (val & DPORT_UHCI1_MODULE_BIT)
            uhci_reset_state(p, 1);
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
        if (val & DPORT_TWAI_MODULE_BIT)
            twai_reset_state(p);
        if (val & DPORT_PWM1_MODULE_BIT)
            mcpwm_reset_unit(p, 1);
        uhci_dport_update(p);
        twai_dport_update(p);
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
        if (off >= DPORT_PRO_INTR_MAP_BASE_OFF &&
            off < DPORT_PRO_INTR_MAP_BASE_OFF +
                  DPORT_INTR_MAP_SOURCE_COUNT * 4u &&
            ((off - DPORT_PRO_INTR_MAP_BASE_OFF) & 3u) == 0u) {
            int source = (int)((off - DPORT_PRO_INTR_MAP_BASE_OFF) / 4u);
            intr_matrix_map_source(p, 0, source, (int)(val & 0x1Fu));
        } else if (off >= DPORT_APP_INTR_MAP_BASE_OFF &&
                   off < DPORT_APP_INTR_MAP_BASE_OFF +
                         DPORT_INTR_MAP_SOURCE_COUNT * 4u &&
                   ((off - DPORT_APP_INTR_MAP_BASE_OFF) & 3u) == 0u) {
            int source = (int)((off - DPORT_APP_INTR_MAP_BASE_OFF) / 4u);
            intr_matrix_map_source(p, 1, source, (int)(val & 0x1Fu));
        }
        break;
    }
}

/* ---- UART0/UART1/UART2 ---- */

static bool uart_num_valid(const esp32_periph_t *p, int uart_num) {
    return p && uart_num >= 0 &&
           uart_num < (int)p->target->uart_count && uart_num < UART_COUNT;
}

static const flexe_uart_instance_desc_t *uart_desc(const esp32_periph_t *p,
                                                    int uart_num) {
    return uart_num_valid(p, uart_num) ? &p->target->uart[uart_num] : NULL;
}

static int uart_num_from_addr(const esp32_periph_t *p, uint32_t addr) {
    if (!p || p->target->uart_count > UART_COUNT) return -1;
    for (int i = 0; i < (int)p->target->uart_count; i++) {
        const flexe_uart_instance_desc_t *desc = &p->target->uart[i];
        if (addr >= desc->base &&
            addr - desc->base < p->target->uart_ip.register_size)
            return i;
    }
    return -1;
}

/* Flexe drains each TX FIFO write immediately into the host callback.  Keep
 * the hardware-visible FIFO empty while still presenting the level/edge
 * interrupts that ESP-IDF's buffered UART driver relies on to dequeue its
 * transmit ring buffer. */
static void uart_intr_update(esp32_periph_t *p, int uart_num) {
    const flexe_uart_instance_desc_t *desc = uart_desc(p, uart_num);
    if (!desc) return;
    uart_state_t *uart = &p->uart[uart_num];
    int source = (int)desc->interrupt_source;

    /* Targets without an interrupt fabric retain UART raw/masked state but
     * cannot deliver it. A target-described matrix uses the same source-level
     * path as classic DPORT rather than a UART-specific shortcut. */
    if (source < 0 || (unsigned)source >= intr_matrix_source_count(p))
        return;

    uint32_t mask = 1u << (source % 32);
    bool active = (uart->int_raw & uart->int_ena) != 0;
    if (active) {
        periph_assert_interrupt_status(p, source,
                                       uart->int_raw & uart->int_ena);
    } else {
        p->pending_sources[source / 32] &= ~mask;
        p->source_status[source] = 0;
        intr_matrix_update_source(p, source, false);
    }
    (void)mask;
}

static void uart_refresh_level_conditions(esp32_periph_t *p, int uart_num) {
    const flexe_uart_ip_desc_t *ip = p ? &p->target->uart_ip : NULL;
    if (!uart_num_valid(p, uart_num)) return;
    uart_state_t *uart = &p->uart[uart_num];
    uint32_t conf1 = uart->shadow[0x24 / 4];
    uint32_t full_threshold = conf1 & ip->rx_full_threshold_mask;
    if (full_threshold > 0 && uart->rx_count >= full_threshold)
        uart->int_raw |= UART_RXFIFO_FULL_INT;
    if (uart->int_ena & UART_TXFIFO_EMPTY_INT)
        uart->int_raw |= UART_TXFIFO_EMPTY_INT;
}

static void uart_emit_tx_byte(esp32_periph_t *p, int uart_num, uint8_t byte,
                              bool apb_fifo_write) {
    if (!uart_num_valid(p, uart_num)) return;
    uart_state_t *uart = &p->uart[uart_num];
    if (uart->tx_len < UART_TX_BUF_SIZE)
        uart->tx[uart->tx_len++] = byte;
    if (uart->cb)
        uart->cb(uart->cb_ctx, byte);
    sbx_event_t ev = { .kind = SBX_EV_UART_TX, .cycle = 0 };
    ev.uart_tx.uart_num = (uint8_t)uart_num;
    ev.uart_tx.byte = byte;
    sbx_events_emit(&ev);

    if (apb_fifo_write) {
        /* The byte has already left Flexe's zero-depth FIFO. */
        uart->int_raw |= UART_TXFIFO_EMPTY_INT | UART_TX_DONE_INT;
        uart_intr_update(p, uart_num);
    }
}

static void uart_dma_tx_done(esp32_periph_t *p, int uart_num) {
    if (!uart_num_valid(p, uart_num)) return;
    p->uart[uart_num].int_raw |= UART_TXFIFO_EMPTY_INT | UART_TX_DONE_INT;
    uart_intr_update(p, uart_num);
}

static uint32_t uart_read(void *ctx, uint32_t addr) {
    esp32_periph_t *p = ctx;
    int uart_num = uart_num_from_addr(p, addr);
    if (uart_num < 0) return 0;
    const flexe_uart_instance_desc_t *desc = uart_desc(p, uart_num);
    const flexe_uart_ip_desc_t *ip = &p->target->uart_ip;
    uart_state_t *uart = &p->uart[uart_num];
    uint32_t off = addr - desc->base;
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
    case 0x1C:                       /* STATUS: TX FIFO drains immediately */
        return ip->status_idle_value | uart->rx_count;
    default:
        if (off == ip->mem_rx_status_offset)
            return ((uint32_t)(uart->rx_tail & ip->fifo_address_mask)
                    << ip->mem_rx_read_shift) |
                   ((uint32_t)(uart->rx_head & ip->fifo_address_mask)
                    << ip->mem_rx_write_shift);
        if (off / 4 < 64) return uart->shadow[off / 4];
        return 0;
    }
}

static void uart_write(void *ctx, uint32_t addr, uint32_t val) {
    esp32_periph_t *p = ctx;
    int uart_num = uart_num_from_addr(p, addr);
    if (uart_num < 0) return;
    const flexe_uart_instance_desc_t *desc = uart_desc(p, uart_num);
    const flexe_uart_ip_desc_t *ip = &p->target->uart_ip;
    uart_state_t *uart = &p->uart[uart_num];
    uint32_t off = addr - desc->base;
    if (off == 0x00) {
        /* FIFO write: TX byte */
        uint8_t byte = (uint8_t)(val & 0xFF);
        uart_emit_tx_byte(p, uart_num, byte, true);
    } else if (off == 0x0C) {       /* INT_ENA */
        uart->int_ena = val & ip->interrupt_valid_mask;
        /* Enabling TXFIFO_EMPTY while the FIFO is empty raises it at once. */
        uart_refresh_level_conditions(p, uart_num);
        uart_intr_update(p, uart_num);
    } else if (off == 0x10) {       /* INT_CLR (W1TC) */
        uart->int_raw &= ~(val & ip->interrupt_valid_mask);
        /* FIFO threshold and TX empty are level-triggered. */
        uart_refresh_level_conditions(p, uart_num);
        uart_intr_update(p, uart_num);
    } else {
        if (off / 4 < 64) uart->shadow[off / 4] = val;
        if (off == 0x24) {          /* CONF1 threshold/timeout controls */
            uart_refresh_level_conditions(p, uart_num);
            if (uart->rx_count > 0 &&
                (val & ip->rx_timeout_enable_mask))
                uart->int_raw |= UART_RXFIFO_TOUT_INT;
            uart_intr_update(p, uart_num);
        }
    }
}

static int uart_register_target(esp32_periph_t *p) {
    if (!p || p->target->uart_count > UART_COUNT) return -1;
    const flexe_uart_ip_desc_t *ip = &p->target->uart_ip;
    for (int uart_num = 0; uart_num < (int)p->target->uart_count;
         uart_num++) {
        const flexe_uart_instance_desc_t *desc = uart_desc(p, uart_num);
        if (!desc || ip->register_size == 0u ||
            ip->date_offset >= sizeof(p->uart[uart_num].shadow) ||
            mem_register_mmio_range(p->mem, desc->base,
                                    ip->register_size,
                                    uart_read, uart_write, p) != 0)
            return -1;
        p->uart[uart_num].int_raw = ip->interrupt_raw_reset;
        p->uart[uart_num].shadow[ip->date_offset / 4u] = ip->date_reset;
    }
    return 0;
}

/* ---- GPIO sigma-delta / pulse-density modulator ---- */

static uint32_t sigmadelta_cpu_mhz(const esp32_periph_t *p) {
    uint32_t mhz = mem_read32(p->mem, ESP32_CPU_TICKS_PER_US_ADDR);
    return mhz >= 10u && mhz <= 240u ? mhz : 240u;
}

static uint64_t sigmadelta_add_saturating(uint64_t value,
                                          uint64_t addend) {
    return value > UINT64_MAX - addend ? UINT64_MAX : value + addend;
}

static uint64_t sigmadelta_peek_cycles(const esp32_periph_t *p) {
    return periph_clock_peek(p, &p->sigmadelta.clock);
}

static uint64_t sigmadelta_now_cycles(esp32_periph_t *p) {
    return periph_clock_now(p, &p->sigmadelta.clock);
}

static uint64_t sigmadelta_mul_div_floor(uint64_t value,
                                         uint64_t multiplier,
                                         uint64_t divisor) {
    if (divisor == 0u) return 0u;
    uint64_t quotient = value / divisor;
    uint64_t remainder = value % divisor;
    if (multiplier != 0u && quotient > UINT64_MAX / multiplier)
        return UINT64_MAX;
    uint64_t result = quotient * multiplier;
    uint64_t tail = remainder * multiplier / divisor;
    return sigmadelta_add_saturating(result, tail);
}

static unsigned sigmadelta_increment(
    const sigmadelta_channel_state_t *channel) {
    return (unsigned)((int)(int8_t)(channel->config & 0xFFu) + 128);
}

static void sigmadelta_phase_at(const esp32_periph_t *p, unsigned index,
                                uint64_t now, uint8_t *accumulator,
                                bool *level) {
    const sigmadelta_channel_state_t *channel =
        &p->sigmadelta.channel[index];
    uint64_t elapsed = now >= channel->anchor_cycles ?
                       now - channel->anchor_cycles : 0u;
    uint32_t prescaler = (channel->config >> 8u) & 0xFFu;
    uint64_t divisor = (uint64_t)sigmadelta_cpu_mhz(p) *
                       (prescaler + 1u);
    uint64_t ticks = sigmadelta_mul_div_floor(
        elapsed, SIGMADELTA_APB_MHZ, divisor);
    unsigned increment = sigmadelta_increment(channel);

    if (accumulator) {
        uint64_t advance = (ticks & 0xFFu) * increment;
        *accumulator = (uint8_t)(channel->anchor_accumulator + advance);
    }
    if (!level) return;
    if (ticks == 0u) {
        *level = channel->anchor_level;
        return;
    }
    unsigned previous = (unsigned)channel->anchor_accumulator +
        (unsigned)(((ticks - 1u) & 0xFFu) * increment);
    previous &= 0xFFu;
    *level = previous + increment >= 0x100u;
}

static bool sigmadelta_channel_level(const esp32_periph_t *p,
                                     unsigned channel) {
    bool level = false;
    sigmadelta_phase_at(p, channel, sigmadelta_peek_cycles(p), NULL,
                        &level);
    return level;
}

static void sigmadelta_sync_channel(esp32_periph_t *p, unsigned index) {
    sigmadelta_channel_state_t *channel =
        &p->sigmadelta.channel[index];
    uint64_t now = sigmadelta_now_cycles(p);
    uint8_t accumulator = 0u;
    bool level = false;
    sigmadelta_phase_at(p, index, now, &accumulator, &level);
    channel->anchor_cycles = now;
    channel->anchor_accumulator = accumulator;
    channel->anchor_level = level;
}

static int sigmadelta_channel_gpio(const esp32_periph_t *p,
                                   unsigned channel, bool *inverted) {
    uint32_t signal = SIGMADELTA_SIGNAL_BASE + channel;
    for (int gpio = 0; gpio < 40; gpio++) {
        uint32_t route = p->gpio.func_out_sel[gpio];
        if ((route & 0x1FFu) != signal) continue;
        if (inverted) *inverted = (route & (1u << 9u)) != 0u;
        return gpio;
    }
    if (inverted) *inverted = false;
    return -1;
}

static bool sigmadelta_gpio_enabled(const esp32_periph_t *p, int gpio) {
    if (gpio < 0 || gpio >= 40) return false;
    if (gpio < 32) return (p->gpio.enable & (1u << gpio)) != 0u;
    return (p->gpio.enable1 & (1u << (gpio - 32))) != 0u;
}

static uint32_t sigmadelta_frequency_hz(
    const sigmadelta_channel_state_t *channel) {
    uint32_t prescaler = (channel->config >> 8u) & 0xFFu;
    return SIGMADELTA_APB_MHZ * 1000000u /
           ((prescaler + 1u) * 256u);
}

static void sigmadelta_emit_channel(esp32_periph_t *p, unsigned index,
                                    bool force) {
    if (index >= SIGMADELTA_CHANNEL_COUNT) return;
    sigmadelta_channel_state_t *channel =
        &p->sigmadelta.channel[index];
    bool inverted = false;
    int gpio = sigmadelta_channel_gpio(p, index, &inverted);
    uint32_t frequency = sigmadelta_frequency_hz(channel);
    int8_t duty = (int8_t)(channel->config & 0xFFu);
    bool enabled = sigmadelta_gpio_enabled(p, gpio);

    bool changed = !channel->output_reported ||
        channel->last_gpio != gpio ||
        channel->last_frequency_hz != frequency ||
        channel->last_duty != duty ||
        channel->last_enabled != enabled ||
        channel->last_inverted != inverted;
    if (!force && !changed) return;

    channel->last_gpio = gpio;
    channel->last_frequency_hz = frequency;
    channel->last_duty = duty;
    channel->last_enabled = enabled;
    channel->last_inverted = inverted;
    channel->output_reported = true;

    if (channel->output_cb) {
        channel->output_cb(channel->output_cb_ctx, (int)index, gpio,
                           frequency, duty, enabled, inverted);
    }

    sbx_event_t event = {
        .kind = SBX_EV_SIGMADELTA_OUT,
        .cycle = sigmadelta_now_cycles(p),
    };
    event.sigmadelta_out.gpio = (int8_t)gpio;
    event.sigmadelta_out.duty = duty;
    event.sigmadelta_out.channel = (uint8_t)index;
    event.sigmadelta_out.enabled = enabled ? 1u : 0u;
    event.sigmadelta_out.inverted = inverted ? 1u : 0u;
    event.sigmadelta_out.frequency_hz = frequency;
    sbx_events_emit(&event);
}

static void sigmadelta_reset_state(esp32_periph_t *p) {
    periph_sigmadelta_output_fn callbacks[SIGMADELTA_CHANNEL_COUNT];
    void *contexts[SIGMADELTA_CHANNEL_COUNT];
    for (unsigned index = 0; index < SIGMADELTA_CHANNEL_COUNT; index++) {
        callbacks[index] = p->sigmadelta.channel[index].output_cb;
        contexts[index] = p->sigmadelta.channel[index].output_cb_ctx;
    }

    uint64_t now = sigmadelta_peek_cycles(p);
    memset(&p->sigmadelta, 0, sizeof(p->sigmadelta));
    p->sigmadelta.clock.cycles = now;
    p->sigmadelta.version = SIGMADELTA_VERSION_RESET;
    for (unsigned core = 0; core < 2u; core++) {
        p->sigmadelta.clock.core_cycles[core] = now;
        if (p->cpu[core]) {
            p->sigmadelta.clock.last_ccount[core] = p->cpu[core]->ccount;
            p->sigmadelta.clock.valid[core] = true;
        }
    }
    for (unsigned index = 0; index < SIGMADELTA_CHANNEL_COUNT; index++) {
        sigmadelta_channel_state_t *channel =
            &p->sigmadelta.channel[index];
        channel->config = 0x0000FF00u;
        channel->anchor_cycles = now;
        channel->output_cb = callbacks[index];
        channel->output_cb_ctx = contexts[index];
        if (callbacks[index]) sigmadelta_emit_channel(p, index, true);
    }
}

static uint32_t sigmadelta_read(esp32_periph_t *p, uint32_t addr) {
    uint32_t off = addr - GPIO_SD_BASE;
    if ((off & 3u) != 0u || off >= 0x100u) return 0u;
    if (off < SIGMADELTA_CHANNEL_COUNT * 4u)
        return p->sigmadelta.channel[off / 4u].config;
    switch (off) {
    case SIGMADELTA_CG_OFF: return p->sigmadelta.cg;
    case SIGMADELTA_MISC_OFF: return p->sigmadelta.misc;
    case SIGMADELTA_VERSION_OFF: return p->sigmadelta.version;
    default: return 0u;
    }
}

static void sigmadelta_write(esp32_periph_t *p, uint32_t addr,
                             uint32_t value) {
    uint32_t off = addr - GPIO_SD_BASE;
    if ((off & 3u) != 0u || off >= 0x100u) return;
    if (off < SIGMADELTA_CHANNEL_COUNT * 4u) {
        unsigned index = off / 4u;
        sigmadelta_sync_channel(p, index);
        p->sigmadelta.channel[index].config =
            value & SIGMADELTA_CHANNEL_MASK;
        sigmadelta_emit_channel(p, index, false);
        return;
    }
    switch (off) {
    case SIGMADELTA_CG_OFF:
        /* ESP32's LL explicitly treats this generated register as absent;
         * retain its documented bit for diagnostics without gating output. */
        p->sigmadelta.cg = value & SIGMADELTA_CG_MASK;
        return;
    case SIGMADELTA_MISC_OFF:
        p->sigmadelta.misc = value & SIGMADELTA_MISC_MASK;
        return;
    case SIGMADELTA_VERSION_OFF:
        p->sigmadelta.version = value & SIGMADELTA_VERSION_MASK;
        return;
    default:
        return;
    }
}

static void sigmadelta_gpio_route_changed(esp32_periph_t *p,
                                          uint32_t before,
                                          uint32_t after) {
    uint32_t routes[2] = {before & 0x1FFu, after & 0x1FFu};
    for (unsigned which = 0; which < 2u; which++) {
        uint32_t route = routes[which];
        if (route >= SIGMADELTA_SIGNAL_BASE &&
            route < SIGMADELTA_SIGNAL_BASE + SIGMADELTA_CHANNEL_COUNT) {
            sigmadelta_emit_channel(p, route - SIGMADELTA_SIGNAL_BASE,
                                    true);
        }
    }
}

static void sigmadelta_gpio_enable_changed(esp32_periph_t *p) {
    for (unsigned index = 0; index < SIGMADELTA_CHANNEL_COUNT; index++)
        sigmadelta_emit_channel(p, index, false);
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
    if (addr >= GPIO_SD_BASE && addr < GPIO_SD_BASE + 0x100u)
        return sigmadelta_read(p, addr);
    uint32_t off = addr - GPIO_BASE;

    /* Basic registers */
    switch (off) {
    case 0x004: return p->gpio.out;         /* GPIO_OUT_REG */
    case 0x008: return 0;                   /* GPIO_OUT_W1TS (write-only) */
    case 0x00C: return 0;                   /* GPIO_OUT_W1TC (write-only) */
    case 0x010: return p->gpio.out1;        /* GPIO_OUT1_REG */
    case 0x020: return p->gpio.enable;      /* GPIO_ENABLE_REG */
    case 0x02C: return p->gpio.enable1;     /* GPIO_ENABLE1_REG */
    /* Deliberately the externally-applied level only. Feeding a driven output
     * back into GPIO_IN looks right and is not: the pad's input buffer is
     * gated by IO_MUX FUN_IE, which GPIO_MODE_OUTPUT leaves off, so firmware
     * that configures an output-only pin is entitled to read zero here. Doing
     * the read-back unconditionally would need FUN_IE modelled first. */
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

static void gpio_emit_changed(esp32_periph_t *p, uint32_t prev, uint32_t now,
                              int pin_base) {
    uint32_t diff = prev ^ now;
    while (diff) {
        int bit = __builtin_ctz(diff);
        diff &= ~(1u << bit);
        if (gpio_dbg())
            fprintf(stderr, "[GPIO] pin%d -> %d\n", pin_base + bit, (now >> bit) & 1u);
        spi_display_gpio_changed(p, pin_base + bit, (now >> bit) & 1u);
        sbx_event_t ev = { .kind = SBX_EV_GPIO_OUT, .cycle = 0 };
        ev.gpio_out.pin = (uint8_t)(pin_base + bit);
        ev.gpio_out.level = (now >> bit) & 1u;
        sbx_events_emit(&ev);
    }
}

static void gpio_write(void *ctx, uint32_t addr, uint32_t val) {
    esp32_periph_t *p = ctx;
    if (addr >= GPIO_SD_BASE && addr < GPIO_SD_BASE + 0x100u) {
        sigmadelta_write(p, addr, val);
        return;
    }
    uint32_t off = addr - GPIO_BASE;
    uint32_t prev_out = p->gpio.out;
    uint32_t prev_out1 = p->gpio.out1;
    uint32_t prev_enable = p->gpio.enable;
    uint32_t prev_enable1 = p->gpio.enable1;

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
        sigmadelta_gpio_route_changed(p, before, val);
        return;
    }

    if (p->gpio.enable != prev_enable || p->gpio.enable1 != prev_enable1)
        sigmadelta_gpio_enable_changed(p);

    /* Fire sandbox events for any output pins that changed level. */
    if (p->gpio.out != prev_out)
        gpio_emit_changed(p, prev_out, p->gpio.out, 0);
    if (p->gpio.out1 != prev_out1)
        gpio_emit_changed(p, prev_out1, p->gpio.out1, 32);
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

/* RTC GPIO and the ordinary GPIO block are two views of the same eighteen
 * pads, and firmware mixes them freely -- rtc_gpio_init() a pin, then read it
 * with gpio_get_level(), or drive it from the RTC domain and watch it on the
 * matrix. Keeping a second copy of the pad state here would give those two
 * views different answers, so RTC_GPIO_OUT/ENABLE push into the shared shadow
 * and RTC_GPIO_IN is built from it. */
/* Pad hold. Each RTC pad carries its own HOLD bit, in its own register at its
 * own bit position -- IDF's rtc_gpio_hold_en() is a read-modify-write of that
 * word, so there is no single "hold" register to watch. Offsets and bits are
 * from rtc_io_reg.h; the channel order matches RTCIO_CHANNEL_GPIO above.
 *
 * A held pad keeps driving whatever it was driving, and ignores further
 * writes, until it is unheld. That is what makes it useful: the hold survives
 * the digital domain being reset, so an output can be parked at a known level
 * across deep sleep. */
static const struct { uint16_t off; uint8_t bit; } RTCIO_HOLD[RTC_GPIO_CHANNELS] = {
    { 0x7Cu, 31 }, { 0x7Cu, 30 }, { 0x7Cu, 29 }, { 0x7Cu, 28 },  /* 36 37 38 39 */
    { 0x80u, 31 }, { 0x80u, 30 },                                /* 34 35 */
    { 0x84u, 29 }, { 0x88u, 29 },                                /* 25 26 (DAC) */
    { 0x8Cu, 29 }, { 0x8Cu, 24 },                                /* 33 32 (32k) */
    { 0x94u, 31 }, { 0x98u, 31 }, { 0x9Cu, 31 }, { 0xA0u, 31 },  /* 4 0 2 15 */
    { 0xA4u, 31 }, { 0xA8u, 31 }, { 0xACu, 31 }, { 0xB0u, 31 },  /* 13 12 14 27 */
};

/* Only RTC pads are modelled here. RTC_IO_DIG_PAD_HOLD_REG (0x74) is stored
 * but not acted on: it holds *digital* pads, whose bit positions are a pad
 * map rather than GPIO numbers, and it only survives light sleep -- deep
 * sleep powers the digital domain down, which is exactly why firmware that
 * needs a level across deep sleep uses rtc_gpio_hold_en() instead. */

static bool rtcio_channel_held(const esp32_periph_t *p, int ch) {
    if (ch < 0 || ch >= RTC_GPIO_CHANNELS) return false;
    uint32_t word = p->rtcio_regs[RTCIO_HOLD[ch].off / 4u];
    return (word >> RTCIO_HOLD[ch].bit) & 1u;
}

static void rtcio_publish_pads(esp32_periph_t *p, uint32_t out_before,
                               uint32_t en_before) {
    uint32_t out = p->rtcio_regs[RTC_GPIO_OUT_OFF / 4u] >> RTC_GPIO_DATA_S;
    uint32_t en  = p->rtcio_regs[RTC_GPIO_ENABLE_OFF / 4u] >> RTC_GPIO_DATA_S;
    uint32_t was_en = en_before >> RTC_GPIO_DATA_S;
    (void)out_before;

    for (int ch = 0; ch < RTC_GPIO_CHANNELS; ch++) {
        int gpio = RTCIO_CHANNEL_GPIO[ch];
        uint32_t bit = 1u << ch;
        uint32_t mask = (gpio < 32) ? (1u << gpio) : (1u << (gpio - 32));
        if (rtcio_channel_held(p, ch)) continue;   /* held: pad ignores writes */
        uint32_t *g_out = (gpio < 32) ? &p->gpio.out : &p->gpio.out1;
        uint32_t *g_en  = (gpio < 32) ? &p->gpio.enable : &p->gpio.enable1;

        if (en & bit) {
            *g_en |= mask;
            if (out & bit) *g_out |= mask; else *g_out &= ~mask;
        } else if (was_en & bit) {
            /* Handed back: stop driving, but leave the level alone so the
             * matrix keeps whatever it had. */
            *g_en &= ~mask;
        }
    }
}

/* RTC_GPIO_IN reflects the pad. For an input that is whatever the host or
 * another peripheral put on it; for a pad the RTC is *driving*, it is the
 * level being driven -- a pin reads back what it outputs, and firmware that
 * holds a level across deep sleep checks it exactly that way. */
static uint32_t rtcio_input_word(const esp32_periph_t *p) {
    uint32_t out = p->rtcio_regs[RTC_GPIO_OUT_OFF / 4u] >> RTC_GPIO_DATA_S;
    uint32_t en  = p->rtcio_regs[RTC_GPIO_ENABLE_OFF / 4u] >> RTC_GPIO_DATA_S;
    uint32_t in = 0;
    for (int ch = 0; ch < RTC_GPIO_CHANNELS; ch++) {
        int gpio = RTCIO_CHANNEL_GPIO[ch];
        uint32_t mask = (gpio < 32) ? (1u << gpio) : (1u << (gpio - 32));
        uint32_t level = (gpio < 32) ? p->gpio.in : p->gpio.in1;
        bool driven = (en >> ch) & 1u;
        bool bit = driven ? ((out >> ch) & 1u) != 0u : (level & mask) != 0u;
        if (bit) in |= (1u << ch);
    }
    return in << RTC_GPIO_DATA_S;
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
    case RTC_GPIO_IN_OFF:
        return rtcio_input_word(p);
    default:
        return p->rtcio_regs[off / 4u];
    }
}

static void rtcio_write(esp32_periph_t *p, uint32_t off, uint32_t val) {
    if ((off & 3u) != 0 || off >= RTCIO_REG_FILE_SIZE)
        return;

    uint32_t out_before = p->rtcio_regs[RTC_GPIO_OUT_OFF / 4u];
    uint32_t en_before   = p->rtcio_regs[RTC_GPIO_ENABLE_OFF / 4u];

    switch (off) {
    case RTC_GPIO_OUT_W1TS_OFF:
        p->rtcio_regs[RTC_GPIO_OUT_OFF / 4u] |= val;
        rtcio_publish_pads(p, out_before, en_before); return;
    case RTC_GPIO_OUT_W1TC_OFF:
        p->rtcio_regs[RTC_GPIO_OUT_OFF / 4u] &= ~val;
        rtcio_publish_pads(p, out_before, en_before); return;
    case RTC_GPIO_ENABLE_W1TS_OFF:
        p->rtcio_regs[RTC_GPIO_ENABLE_OFF / 4u] |= val;
        rtcio_publish_pads(p, out_before, en_before); return;
    case RTC_GPIO_ENABLE_W1TC_OFF:
        p->rtcio_regs[RTC_GPIO_ENABLE_OFF / 4u] &= ~val;
        rtcio_publish_pads(p, out_before, en_before); return;
    case 0x01Cu: p->rtcio_regs[0x018u / 4u] |= val; return;
    case 0x020u: p->rtcio_regs[0x018u / 4u] &= ~val; return;
    case RTC_GPIO_IN_OFF: return; /* read-only */
    case RTC_IO_EXT_WAKEUP0_OFF:
        p->rtc_ext_wakeup0 = val;
        p->rtcio_regs[off / 4u] = val;
        return;
    default: break;
    }

    uint32_t before = p->rtcio_regs[off / 4u];
    p->rtcio_regs[off / 4u] = val;
    if (off == RTC_GPIO_OUT_OFF || off == RTC_GPIO_ENABLE_OFF) {
        rtcio_publish_pads(p, out_before, en_before);
        return;
    }
    if (off == RTCIO_DAC1_OFF)
        rtcio_emit_dac_change(0, before, val);
    else if (off == RTCIO_DAC2_OFF)
        rtcio_emit_dac_change(1, before, val);
}

static uint32_t rtc_i2c_read(esp32_periph_t *p, uint32_t off);
static void rtc_i2c_write(esp32_periph_t *p, uint32_t off, uint32_t val);
static void rtc_i2c_execute_sens(esp32_periph_t *p, uint32_t control);

static int sens_measure_unit(uint32_t off) {
    if (off == SENS_SAR_MEAS_START1_OFF) return 0;
    if (off == SENS_SAR_MEAS_START2_OFF) return 1;
    return -1;
}

/* Threshold for one pad. THRES1..5 pack two pads each, the even one in the
 * high half. */
static uint16_t touch_threshold(const esp32_periph_t *p, int pad) {
    uint32_t reg = p->sens_regs[(SENS_SAR_TOUCH_THRES1_OFF / 4u) + (uint32_t)(pad / 2)];
    return (uint16_t)((pad & 1) ? (reg & 0xFFFFu) : (reg >> 16));
}

static void rtc_irq_update(esp32_periph_t *p) {
    if (p->rtc_int_raw & p->rtc_int_ena)
        periph_assert_interrupt(p, RTC_CORE_INTR_SOURCE);
    else
        periph_deassert_interrupt(p, RTC_CORE_INTR_SOURCE);
}

static void touch_raise_int(esp32_periph_t *p) {
    p->rtc_int_raw |= RTC_CNTL_TOUCH_INT_BIT;
    rtc_irq_update(p);
}

/* One scan of the enabled pads.
 *
 * A pad reads *lower* as capacitance rises, so "touched" is count below
 * threshold -- the opposite of the intuitive direction, and the thing to get
 * right: inverting it makes an untouched panel look permanently pressed. The
 * count lands in the OUT pair registers, the below-threshold set latches into
 * CTRL2's low ten bits where touch_pad_get_status() reads it, and a newly
 * touched pad raises the RTC core interrupt. */
static void touch_run_measurement(esp32_periph_t *p) {
    uint32_t worken = p->sens_regs[SENS_SAR_TOUCH_ENABLE_OFF / 4u] &
                      SENS_TOUCH_WORKEN_MASK;
    uint32_t before = p->touch_status;

    for (int pad = 0; pad < SENS_TOUCH_PAD_COUNT; pad++) {
        uint32_t *out = &p->sens_regs[(SENS_SAR_TOUCH_OUT1_OFF / 4u) +
                                      (uint32_t)(pad / 2)];
        if (!(worken & (1u << pad))) {
            /* A disabled pad measures nothing and cannot be touched. */
            p->touch_status &= ~(1u << pad);
            continue;
        }
        uint16_t value = p->touch_value[pad];
        if (pad & 1) *out = (*out & 0xFFFF0000u) | value;
        else         *out = (*out & 0x0000FFFFu) | ((uint32_t)value << 16);

        uint16_t thres = touch_threshold(p, pad);
        if (thres != 0 && value < thres) p->touch_status |= (1u << pad);
        else                             p->touch_status &= ~(1u << pad);
    }

    p->sens_regs[SENS_SAR_TOUCH_CTRL2_OFF / 4u] |= SENS_TOUCH_MEAS_DONE_BIT;
    if (p->touch_status & ~before)
        touch_raise_int(p);
}

static bool touch_fsm_running(const esp32_periph_t *p) {
    uint32_t ctrl2 = p->sens_regs[SENS_SAR_TOUCH_CTRL2_OFF / 4u];
    /* IDF 5.5's new touch-sensor driver starts continuous scanning through
     * RTC_CNTL_STATE0.touch_slp_timer_en.  Older drivers use the SENS FSM bit.
     * Both controls feed the same touch measurement engine. */
    return (ctrl2 & SENS_TOUCH_START_FSM_EN_BIT) != 0 ||
           (p->rtc_state0 & RTC_CNTL_TOUCH_SLP_TIMER_EN_BIT) != 0;
}

static uint32_t sens_read(esp32_periph_t *p, uint32_t off) {
    if ((off & 3u) != 0 || off >= SENS_REG_FILE_SIZE)
        return 0;
    int unit = sens_measure_unit(off);
    uint32_t val = p->sens_regs[off / 4u];
    if (off == SENS_SAR_SLAVE_ADDR4_OFF) {
        val &= SENS_I2C_ADDR_FIELDS_MASK;
        val |= (uint32_t)p->rtc_i2c.data << 22u;
        if (p->rtc_i2c.done)
            val |= SENS_I2C_DONE;
    }
    if (unit >= 0) {
        val &= SENS_SAR_CONFIG_MASK;
        if (p->sens_adc_done[unit])
            val |= SENS_SAR_DONE;
        val |= p->sens_adc_result[unit];
    }
    if (off == SENS_SAR_TOUCH_CTRL2_OFF) {
        /* The low ten bits are status, not a stored field. */
        val = (val & ~SENS_TOUCH_STATUS_MASK) |
              (p->touch_status & SENS_TOUCH_STATUS_MASK);
    }
    return val;
}

static void sens_write(esp32_periph_t *p, uint32_t off, uint32_t val) {
    if ((off & 3u) != 0 || off >= SENS_REG_FILE_SIZE)
        return;
    int unit = sens_measure_unit(off);
    if (unit < 0) {
        if (off >= SENS_SAR_SLAVE_ADDR1_OFF &&
            off <= SENS_SAR_SLAVE_ADDR4_OFF) {
            p->sens_regs[off / 4u] = val & SENS_I2C_ADDR_FIELDS_MASK;
            return;
        }
        if (off == SENS_SAR_TOUCH_CTRL2_OFF) {
            uint32_t hardware_done =
                p->sens_regs[off / 4u] & SENS_TOUCH_MEAS_DONE_BIT;
            if (val & SENS_TOUCH_MEAS_EN_CLR_BIT)
                p->touch_status = 0;
            /* MEAS_DONE and the status bits are hardware-owned.  In
             * particular, preserve DONE across the driver's read/modify/write
             * that lowers START_EN after a one-shot scan; treating the RO bit
             * as writable made the following DONE poll wait forever. */
            p->sens_regs[off / 4u] =
                val & ~(SENS_TOUCH_STATUS_MASK | SENS_TOUCH_MEAS_DONE_BIT |
                        SENS_TOUCH_MEAS_EN_CLR_BIT);
            p->sens_regs[off / 4u] |= hardware_done;
            /* Either a software-forced start or the FSM being switched on
             * produces a scan; IDF uses both, depending on the mode. */
            if ((val & (SENS_TOUCH_START_FORCE_BIT | SENS_TOUCH_START_EN_BIT)) ==
                    (SENS_TOUCH_START_FORCE_BIT | SENS_TOUCH_START_EN_BIT) ||
                (val & SENS_TOUCH_START_FSM_EN_BIT))
                touch_run_measurement(p);
            return;
        }
        if (off == SENS_SAR_TOUCH_ENABLE_OFF) {
            p->sens_regs[off / 4u] = val;
            touch_run_measurement(p);
            return;
        }
        if (off == SENS_SAR_I2C_CTRL_OFF) {
            uint32_t before = p->sens_regs[off / 4u];
            val &= SENS_I2C_CTRL_MASK;
            p->sens_regs[off / 4u] = val;
            bool rising_start = (before & SENS_I2C_START) == 0 &&
                                (val & (SENS_I2C_START_FORCE |
                                        SENS_I2C_START)) ==
                                    (SENS_I2C_START_FORCE |
                                     SENS_I2C_START);
            if (rising_start)
                rtc_i2c_execute_sens(p, val & 0x0FFFFFFFu);
            return;
        }
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

/* The RTC slow clock, derived from elapsed emulated time. It has to be a real
 * counter: the sleep timer is armed as an absolute value on this clock, so a
 * register that always reads zero makes every timed sleep either instant or
 * infinite. */
static uint64_t rtc_slow_ticks(esp32_periph_t *p) {
    uint32_t mhz = mem_read32(p->mem, ESP32_CPU_TICKS_PER_US_ADDR);
    if (mhz < 10u || mhz > 240u) mhz = 160u;
    uint64_t us = timg_now_cycles(p) / mhz;
    return us * RTC_SLOW_CLK_HZ / 1000000u;
}

static int rtc_pad_level(const esp32_periph_t *p, int channel) {
    if (channel < 0 || channel >= RTC_GPIO_CHANNELS) return 0;
    int gpio = RTCIO_CHANNEL_GPIO[channel];
    uint32_t mask = (gpio < 32) ? (1u << gpio) : (1u << (gpio - 32));
    uint32_t in = (gpio < 32) ? p->gpio.in : p->gpio.in1;
    return (in & mask) ? 1 : 0;
}

/* Which armed wake source, if any, is satisfied right now. Returns the trigger
 * bits esp_sleep_get_wakeup_cause() expects, or 0. The timer is handled by the
 * caller, which knows how far time has moved. */
static uint32_t rtc_wake_condition(esp32_periph_t *p) {
    uint32_t ena = p->rtc_wakeup_ena;
    uint32_t cause = 0;

    if (ena & RTC_EXT0_TRIG_EN) {
        int ch = (int)(p->rtc_ext_wakeup0 >> RTC_IO_EXT_WAKEUP0_SEL_S) & 0x1F;
        int want = (p->rtc_ext_wakeup_conf & RTC_CNTL_EXT_WAKEUP0_LV_BIT) ? 1 : 0;
        if (rtc_pad_level(p, ch) == want) cause |= RTC_EXT0_TRIG_EN;
    }
    if (ena & RTC_EXT1_TRIG_EN) {
        uint32_t sel = p->rtc_ext_wakeup1 & 0x3FFFFu;
        bool all_low = (p->rtc_ext_wakeup_conf & RTC_CNTL_EXT_WAKEUP1_LV_BIT) == 0;
        bool any_high = false, every_low = sel != 0;
        for (int ch = 0; ch < RTC_GPIO_CHANNELS; ch++) {
            if (!(sel & (1u << ch))) continue;
            if (rtc_pad_level(p, ch)) any_high = true; else every_low = false;
        }
        /* ESP_EXT1_WAKEUP_ANY_HIGH vs ALL_LOW. */
        if (sel && (all_low ? every_low : any_high)) cause |= RTC_EXT1_TRIG_EN;
    }
    if ((ena & RTC_TOUCH_TRIG_EN) && p->touch_status != 0)
        cause |= RTC_TOUCH_TRIG_EN;
    return cause;
}

static uint32_t rtc_cntl_read(void *ctx, uint32_t addr) {
    esp32_periph_t *p = ctx;
    uint32_t off = addr - RTC_CNTL_BASE;
    if (addr >= RTC_I2C_BASE &&
        addr < RTC_I2C_BASE + RTC_I2C_REG_FILE_SIZE)
        return rtc_i2c_read(p, addr - RTC_I2C_BASE);
    if (addr >= SENS_BASE && addr < SENS_BASE + SENS_REG_FILE_SIZE)
        return sens_read(p, addr - SENS_BASE);
    if (addr >= RTCIO_BASE && addr < RTCIO_BASE + RTCIO_REG_FILE_SIZE)
        return rtcio_read(p, addr - RTCIO_BASE);
    /* Reserved RTCIO/SENS portions of the shared page read as zero. */
    if (off >= 0x400u) return 0;
    switch (off) {
    case RTC_CNTL_TIME_UPDATE_OFF: return (1u << 30); /* time always valid */
    case RTC_CNTL_TIME0_OFF: return (uint32_t)p->rtc_time_latched;
    case RTC_CNTL_TIME1_OFF: return (uint32_t)(p->rtc_time_latched >> 32);
    case RTC_CNTL_SLP_TIMER0_OFF: return (uint32_t)p->rtc_slp_target;
    case RTC_CNTL_SLP_TIMER1_OFF: return (uint32_t)(p->rtc_slp_target >> 32);
    case RTC_CNTL_STATE0_OFF: return p->rtc_state0;
    /* Reset cause, duplicated into both the PRO and APP CPU fields. */
    case RTC_CNTL_RESET_STATE_OFF:
        return p->rtc_reset_cause | (p->rtc_reset_cause << 6);
    /* One register, two fields: WAKEUP_ENA at bit 11 is what firmware writes
     * to arm the sources, WAKEUP_CAUSE at bit 0 is the read-only report that
     * esp_sleep_get_wakeup_cause() decodes. This used to answer a hardcoded 1
     * -- reading as "woke from EXT0" on a board that had never slept. */
    case RTC_CNTL_WAKEUP_STATE_OFF:
        return (p->rtc_wakeup_ena << RTC_CNTL_WAKEUP_ENA_S) |
               p->rtc_wakeup_cause;
    case RTC_CNTL_STORE0_OFF + 0u:  return p->rtc_store[0];
    case RTC_CNTL_STORE0_OFF + 4u:  return p->rtc_store[1];
    case RTC_CNTL_STORE0_OFF + 8u:  return p->rtc_store[2];
    case RTC_CNTL_STORE0_OFF + 12u: return p->rtc_store[3];
    case RTC_CNTL_EXT_WAKEUP_CONF_OFF: return p->rtc_ext_wakeup_conf;
    case RTC_CNTL_EXT_WAKEUP1_OFF: return p->rtc_ext_wakeup1;
    case RTC_CNTL_DIG_PWC_OFF: return p->rtc_dig_pwc;
    case RTC_CNTL_INT_ENA_OFF: return p->rtc_int_ena;
    case RTC_CNTL_INT_RAW_OFF: return p->rtc_int_raw;
    case RTC_CNTL_INT_ST_OFF:  return p->rtc_int_raw & p->rtc_int_ena;
    case 0x080: return 0;           /* SLP_TIMER_BASE */
    case 0x068: return p->rtc_cpu_period_conf;
    case 0x070: return p->rtc_clk_conf;
    case 0x0A8: return 0x2210;      /* CLK_CONF: clocks ready */
    case 0x0B0: return 0x00280028;  /* RTC_XTAL_FREQ_REG: 40 MHz crystal (both halves) */
    default: return 0;
    }
}

static void rtc_cntl_write(void *ctx, uint32_t addr, uint32_t val) {
    esp32_periph_t *p = ctx;
    if (addr >= RTC_I2C_BASE &&
        addr < RTC_I2C_BASE + RTC_I2C_REG_FILE_SIZE) {
        rtc_i2c_write(p, addr - RTC_I2C_BASE, val);
        return;
    }
    if (addr >= SENS_BASE && addr < SENS_BASE + SENS_REG_FILE_SIZE) {
        sens_write(p, addr - SENS_BASE, val);
        return;
    }
    if (addr >= RTCIO_BASE && addr < RTCIO_BASE + RTCIO_REG_FILE_SIZE) {
        rtcio_write(p, addr - RTCIO_BASE, val);
        return;
    }
    uint32_t off = addr - RTC_CNTL_BASE;
    if (off == 0x000u) {
        /* RTC_CNTL_OPTIONS0. SW_SYS_RST (bit 31) and SW_PROCPU_RST (bit 5)
         * are how firmware reboots itself: esp_restart_noos() sets one and
         * then spins in a `while (1)` waiting for the reset to take. This
         * register had no case here at all, so the request was discarded and
         * the guest span in that loop for ever. SW_APPCPU_RST (bit 4) is
         * routine -- the PRO CPU uses it to start the APP CPU -- so it is not
         * a reboot on its own. */
        if (val & ((1u << 31) | (1u << 5)))
            p->reset_requested = true;
        return;
    }
    switch (off) {
    case RTC_CNTL_TIME_UPDATE_OFF:
        /* Firmware sets TIME_UPDATE and then reads TIME0/TIME1; the counter is
         * latched at that moment so the two halves are consistent. */
        p->rtc_time_latched = rtc_slow_ticks(p);
        return;
    case RTC_CNTL_SLP_TIMER0_OFF:
        p->rtc_slp_target = (p->rtc_slp_target & 0xFFFFFFFF00000000ULL) | val;
        return;
    case RTC_CNTL_SLP_TIMER1_OFF:
        p->rtc_slp_target = (p->rtc_slp_target & 0xFFFFFFFFULL) |
                            ((uint64_t)(val & 0xFFFFu) << 32);
        return;
    case RTC_CNTL_STATE0_OFF:
        p->rtc_state0 = val;
        if (val & RTC_CNTL_SLEEP_EN_BIT) {
            /* Deep sleep is distinguished by the digital domain being powered
             * down; light sleep leaves it up and simply resumes. */
            p->sleep_deep = (p->rtc_dig_pwc & RTC_CNTL_DG_WRAP_PD_EN_BIT) != 0;
            p->sleep_requested = true;
        }
        return;
    case RTC_CNTL_RESET_STATE_OFF: return;   /* read-only */
    case RTC_CNTL_WAKEUP_STATE_OFF:
        /* rtc_sleep_start() arms the wake sources here, not in STATE0. */
        p->rtc_wakeup_ena = (val >> RTC_CNTL_WAKEUP_ENA_S) &
                            RTC_CNTL_WAKEUP_ENA_MASK;
        return;
    case RTC_CNTL_STORE0_OFF + 0u:  p->rtc_store[0] = val; return;
    case RTC_CNTL_STORE0_OFF + 4u:  p->rtc_store[1] = val; return;
    case RTC_CNTL_STORE0_OFF + 8u:  p->rtc_store[2] = val; return;
    case RTC_CNTL_STORE0_OFF + 12u: p->rtc_store[3] = val; return;
    case RTC_CNTL_EXT_WAKEUP_CONF_OFF: p->rtc_ext_wakeup_conf = val; return;
    case RTC_CNTL_EXT_WAKEUP1_OFF: p->rtc_ext_wakeup1 = val; return;
    case RTC_CNTL_DIG_PWC_OFF: p->rtc_dig_pwc = val; return;
    case RTC_CNTL_INT_ENA_OFF:
        p->rtc_int_ena = val;
        /* Enabling a source whose condition is already latched has to deliver
         * it, or firmware that arms the interrupt after configuring the
         * peripheral never hears about a pad that is already down. */
        rtc_irq_update(p);
        return;
    case RTC_CNTL_INT_CLR_OFF:
        p->rtc_int_raw &= ~val;
        /* RTC_CORE is level-sensitive: clearing the last enabled raw bit must
         * lower source 46 so a later touch/wake event has a new rising edge. */
        rtc_irq_update(p);
        return;
    case 0x068u:
        p->rtc_cpu_period_conf = val & RTC_CPU_PERIOD_CONF_MASK;
        return;
    case 0x070u:
        p->rtc_clk_conf = val & RTC_CLK_CONF_MASK;
        return;
    default:
        return;
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
#define LACT_CFG_INCREASE         (1u << 30)
#define LACT_CFG_AUTORELOAD       (1u << 29)
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
    return periph_clock_now(p, &p->timg_clock);
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
    periph_event_source_changed(p, PERIPH_EVENT_TIMG);
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
    p->lact[group].last_cycles = p->timg_clock.cycles;
    for (unsigned timer = 0; timer < TIMG_TIMER_COUNT; timer++) {
        p->timg[group].timer[timer].config = TIMG_TIMER_CONFIG_RESET;
        p->timg[group].timer[timer].last_cycles = p->timg_clock.cycles;
    }
}

static uint32_t lact_divider(const lact_state_t *l) {
    uint32_t d = (l->config >> 13) & 0xFFFF;
    if (d == 0u) return 65536u;
    return d < 2u ? 2u : d;
}

bool periph_lact_counter_at_ccount(const esp32_periph_t *p,
                                   const xtensa_cpu_t *cpu, int group,
                                   uint32_t ccount_ahead, uint64_t *counter) {
    if (!p || !cpu || !counter || group < 0 || group >= 2)
        return false;

    const periph_clock_t *clock = &p->timg_clock;
    uint64_t now = periph_clock_peek(p, clock);
    bool attached = false;
    for (unsigned core = 0; core < 2u; core++) {
        if (p->cpu[core] != cpu) continue;
        attached = true;
        if (!clock->valid[core]) break;
        uint32_t future = cpu->ccount + ccount_ahead;
        uint32_t elapsed = future - clock->last_ccount[core];
        if (elapsed >= (uint32_t)INT32_MAX) return false;
        uint64_t candidate = clock->core_cycles[core] >
            UINT64_MAX - elapsed ? UINT64_MAX :
            clock->core_cycles[core] + elapsed;
        if (candidate > now) now = candidate;
        break;
    }
    if (!attached) return false;

    const lact_state_t *l = &p->lact[group];
    uint64_t value = l->counter;
    if ((l->config & LACT_CFG_EN) != 0u &&
        timg_group_clocked(p, (unsigned)group)) {
        uint64_t elapsed = now >= l->last_cycles ?
                           now - l->last_cycles : 0u;
        uint64_t denominator =
            (uint64_t)timg_cpu_mhz(p) * lact_divider(l);
        uint64_t product = elapsed >
            (UINT64_MAX - l->tick_remainder) / TIMG_APB_CLOCK_MHZ ?
            UINT64_MAX : l->tick_remainder +
                         elapsed * TIMG_APB_CLOCK_MHZ;
        uint64_t ticks = product / denominator;
        value = l->config & LACT_CFG_INCREASE ? value + ticks :
                                                 value - ticks;
    }
    *counter = value;
    return true;
}

static void lact_sync_to(esp32_periph_t *p, int group, uint64_t now) {
    lact_state_t *l = &p->lact[group];
    uint64_t elapsed = now >= l->last_cycles ? now - l->last_cycles : 0u;
    l->last_cycles = now;
    if (!(l->config & LACT_CFG_EN) ||
        !timg_group_clocked(p, (unsigned)group))
        return;

    uint64_t denominator = (uint64_t)timg_cpu_mhz(p) * lact_divider(l);
    uint64_t product = elapsed >
        (UINT64_MAX - l->tick_remainder) / TIMG_APB_CLOCK_MHZ ?
        UINT64_MAX : l->tick_remainder + elapsed * TIMG_APB_CLOCK_MHZ;
    uint64_t ticks = product / denominator;
    l->tick_remainder = product % denominator;
    if (ticks == 0u) return;

    bool increase = (l->config & LACT_CFG_INCREASE) != 0u;
    uint64_t distance = increase ? l->alarm - l->counter :
                                   l->counter - l->alarm;
    bool fire = (l->config & LACT_CFG_ALARM_EN) != 0u &&
                distance != 0u && ticks >= distance;
    if (!fire) {
        l->counter = increase ? l->counter + ticks : l->counter - ticks;
        return;
    }

    l->counter = l->alarm;
    ticks -= distance;
    bool autoreload = (l->config & LACT_CFG_AUTORELOAD) != 0u;
    l->config &= ~LACT_CFG_ALARM_EN;
    if (autoreload) l->counter = l->load;
    p->timg[group].int_raw |= LACT_INT_BIT;
    timg_update_lact_irq(p, (unsigned)group);

    /* ALARM_EN is one-shot. Any cycles left after the compare still advance
     * the live counter (from LOAD when autoreload is selected). */
    increase = (l->config & LACT_CFG_INCREASE) != 0u;
    l->counter = increase ? l->counter + ticks : l->counter - ticks;
}

/* Live 64-bit LACT counter in APB/DIVIDER ticks. Sampling from either core
 * advances the same shared timeline without double-counting dual-core work. */
static uint64_t lact_counter(esp32_periph_t *p, int group) {
    uint64_t now = timg_now_cycles(p);
    lact_sync_to(p, group, now);
    return p->lact[group].counter;
}

/* Re-evaluate the alarm condition and drive the level interrupt source.
 * Hardware: INT_RAW sets when counter >= alarm with ALARM_EN; the source
 * line asserts while (INT_RAW & INT_ENA). */
static void lact_latch_alarm(esp32_periph_t *p, int group) {
    (void)lact_counter(p, group);
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
        if (!(l->config & LACT_CFG_ALARM_EN) ||
            !(l->config & LACT_CFG_EN) ||
            !timg_group_clocked(p, (unsigned)group))
            continue;
        if (p->timg[group].int_raw & LACT_INT_BIT) {
            continue; /* event already latched */
        }
        uint64_t now = lact_counter(p, group);
        bool increase = (l->config & LACT_CFG_INCREASE) != 0u;
        if ((increase && l->alarm <= now) ||
            (!increase && l->alarm >= now))
            continue; /* hardware does not retrigger an already-past compare */
        uint64_t ticks = increase ? l->alarm - now : now - l->alarm;
        uint64_t denominator =
            (uint64_t)timg_cpu_mhz(p) * lact_divider(l);
        uint64_t needed = ticks > UINT64_MAX / denominator ? UINT64_MAX :
                          ticks * denominator;
        uint64_t cycles;
        if (needed <= l->tick_remainder) {
            cycles = 1u;
        } else {
            needed -= l->tick_remainder;
            cycles = needed / TIMG_APB_CLOCK_MHZ +
                     (needed % TIMG_APB_CLOCK_MHZ != 0u);
        }
        uint64_t event = (uint64_t)cpu->ccount + cycles;
        if (event > UINT32_MAX) event = UINT32_MAX;
        if (event < best) best = event;
    }
    return (uint32_t)best;
}

static void lact_eval_events(esp32_periph_t *p, xtensa_cpu_t *cpu) {
    (void)cpu;
    for (int group = 0; group < 2; group++)
        lact_eval_irq(p, group);
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
    periph_event_source_changed(p, PERIPH_EVENT_FRC);
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

static int periph_event_core(const esp32_periph_t *p,
                             const xtensa_cpu_t *cpu) {
    if (p && cpu == p->cpu[0]) return 0;
    if (p && cpu == p->cpu[1]) return 1;
    return -1;
}

/* CPU hooks (registered on both cores, wired into next_timer_event). */
static uint32_t periph_next_event_hook(xtensa_cpu_t *cpu) {
    esp32_periph_t *p = (esp32_periph_t *)cpu->periph_event_ctx;
    int core = periph_event_core(p, cpu);
    if (core < 0) return UINT32_MAX;

    uint32_t candidates = p->event_source_candidates[core] &
                          p->event_source_registered_mask;
    bool have = false;
    uint32_t best = UINT32_MAX;
    uint32_t best_distance = 0;
#define PERIPH_CONSIDER_EVENT(name, next, eval, continuous) do {             \
        const uint32_t bit = 1u << PERIPH_EVENT_##name;                      \
        if (candidates & bit) {                                               \
            uint32_t event = next(p, cpu);                                    \
            if (event == UINT32_MAX) {                                        \
                p->event_source_candidates[core] &= ~bit;                     \
            } else {                                                          \
                uint32_t distance = event - cpu->ccount;                      \
                if ((int32_t)distance < 0) distance = 0;                      \
                if (!have || distance < best_distance) {                     \
                    have = true;                                              \
                    best = event;                                             \
                    best_distance = distance;                                 \
                }                                                             \
            }                                                                 \
        }                                                                     \
    } while (0);
    PERIPH_EVENT_SOURCE_LIST(PERIPH_CONSIDER_EVENT)
#undef PERIPH_CONSIDER_EVENT
    return have ? best : UINT32_MAX;
}
static void periph_event_hook(xtensa_cpu_t *cpu) {
    esp32_periph_t *p = (esp32_periph_t *)cpu->periph_event_ctx;
    int core = periph_event_core(p, cpu);
    if (core < 0) return;

    uint32_t registered = p->event_source_registered_mask;
    uint32_t candidates = p->event_source_candidates[core] & registered;
#define PERIPH_EVAL_EVENT(name, next, eval, continuous) do {                 \
        const uint32_t bit = 1u << PERIPH_EVENT_##name;                      \
        if ((candidates & bit) || ((continuous) && (registered & bit)))       \
            eval(p, cpu);                                                     \
    } while (0);
    PERIPH_EVENT_SOURCE_LIST(PERIPH_EVAL_EVENT)
#undef PERIPH_EVAL_EVENT
}

/* Recompute both cores' next_timer_event after LACT state changes. */
static void lact_kick(esp32_periph_t *p) {
    periph_event_source_changed(p, PERIPH_EVENT_LACT);
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
    case 0x070: {
        lact_sync_to(p, group, now);
        uint32_t old_config = l->config;
        l->config = val;
        if (((old_config ^ l->config) & (0xFFFFu << 13)) != 0u)
            l->tick_remainder = 0u;
        lact_eval_irq(p, group);
        lact_kick(p);
        break;
    }
    case 0x074: l->rtc = val; break;
    case 0x080: break;               /* LACTUPDATE: reads are live, no latch needed */
    case 0x084: l->alarm = (l->alarm & 0xFFFFFFFF00000000ull) | val;
               lact_eval_irq(p, group); lact_kick(p); break;
    case 0x088: l->alarm = (l->alarm & 0xFFFFFFFFull) | ((uint64_t)val << 32);
               lact_eval_irq(p, group); lact_kick(p); break;
    case 0x08C: l->load = (l->load & 0xFFFFFFFF00000000ull) | val; break;
    case 0x090: l->load = (l->load & 0xFFFFFFFFull) | ((uint64_t)val << 32); break;
    case 0x094:                    /* LACTLOAD: counter := load value */
        l->counter = l->load;
        l->last_cycles = now;
        l->tick_remainder = 0u;
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
#define WDEV_RND_OFF       0x144u
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
    case BT_MAC_BASE:      return &p->radio.bt_mac[word];
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
    uint32_t off = addr - WDEV_BASE;
    if (off == WDEV_RND_OFF) {
        /* Per-session xorshift64 stream. It is deterministic for reproducible
         * firmware tests while still changing on every hardware read. */
        p->radio.rng_state ^= p->radio.rng_state << 13;
        p->radio.rng_state ^= p->radio.rng_state >> 7;
        p->radio.rng_state ^= p->radio.rng_state << 17;
        return (uint32_t)p->radio.rng_state;
    }
    return p->radio.wdev[off / sizeof(uint32_t)];
}

static void wdev_write(void *ctx, uint32_t addr, uint32_t val) {
    esp32_periph_t *p = ctx;
    uint32_t off = addr - WDEV_BASE;
    p->radio.wdev[off / sizeof(uint32_t)] = val;
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

/* Deliver the ISR again when a *new* enabled condition appears, even though
 * the source line was already high.
 *
 * intr_matrix_update_source() dispatches a registered handler on the rising
 * edge of the aggregate line only. That is fine for a source that goes quiet
 * between events, and wrong for a level-triggered one that accumulates: the
 * I2C slave holds TXFIFO_EMPTY high for as long as its transmit FIFO is empty,
 * so the line never falls, and every later condition -- including the
 * TRANS_COMPLETE that tells the driver to drain a received transfer -- arrived
 * with no edge to carry it. The ISR ran exactly once in the whole run.
 *
 * Scoped to I2C rather than changed in the matrix, because re-entering every
 * level-triggered handler for as long as its line is high is what hardware
 * does and is also how an interrupt storm starts; the general gap is worth
 * fixing separately and with its own measurements. */
static void i2c_intr_update(esp32_periph_t *p, int port) {
    i2c_state_t *i2c = &p->i2c[port];
    int source = i2c_intr_sources[port];

    /* The two FIFO interrupts are *level* conditions, not events: the hardware
     * asserts TXFIFO_EMPTY for as long as the TX FIFO is below its threshold
     * and RXFIFO_FULL for as long as the RX FIFO is above its. Latching them
     * only where the FIFOs happen to be touched leaves the slave driver
     * deadlocked -- it stages a reply into its ringbuffer and waits for
     * TXFIFO_EMPTY to tell its ISR to move it into the FIFO, and that
     * interrupt never comes because the FIFO was already empty when it was
     * enabled. Recompute both here instead. */
    uint32_t conf = i2c->regs[I2C_FIFO_CONF_OFF / 4u];
    uint32_t rx_threshold = conf & 0x1Fu;
    uint32_t tx_threshold = (conf >> 5) & 0x1Fu;
    if (i2c->tx_count <= tx_threshold) i2c->int_raw |= I2C_INT_TXFIFO_EMPTY;
    else                               i2c->int_raw &= ~I2C_INT_TXFIFO_EMPTY;
    if (i2c->rx_count > rx_threshold)  i2c->int_raw |= I2C_INT_RXFIFO_FULL;
    else                               i2c->int_raw &= ~I2C_INT_RXFIFO_FULL;

    /* Level conditions need the status-aware form: a fresh condition arriving
     * while the line is already high produces no edge, and the plain assert
     * would never call the handler again. This used to be a bespoke
     * i2c_dispatch_new_conditions() helper; it is the general mechanism now,
     * so every peripheral with a raw/ena pair gets the same behaviour. */
    if (i2c->int_raw & i2c->int_ena)
        periph_assert_interrupt_status(p, source, i2c->int_raw & i2c->int_ena);
    else
        periph_deassert_interrupt(p, source);
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

static bool rtc_i2c_selected_address(const esp32_periph_t *p,
                                     unsigned selector,
                                     uint8_t *address) {
    if (selector >= 8u || !address)
        return false;
    uint32_t off = SENS_SAR_SLAVE_ADDR1_OFF + (selector / 2u) * 4u;
    uint32_t value = p->sens_regs[off / 4u];
    if ((selector & 1u) == 0)
        value >>= 11u;
    *address = (uint8_t)(value & 0x7Fu);
    return true;
}

/* Execute the lower 28 bits of the classic ULP I2C instruction. The SENS
 * software-start path exposes this same control word to either Xtensa core,
 * which provides a useful main-CPU path even before Flexe grows a ULP FSM
 * execution engine. Transactions are synchronous at the virtual bus edge. */
static void rtc_i2c_execute_sens(esp32_periph_t *p, uint32_t control) {
    rtc_i2c_state_t *rtc = &p->rtc_i2c;
    uint32_t *debug = &rtc->regs[RTC_I2C_DEBUG_STATUS_OFF / 4u];
    uint8_t subaddress = (uint8_t)control;
    uint8_t value = (uint8_t)(control >> 8u);
    unsigned low_bit = (control >> 16u) & 7u;
    unsigned high_bit = (control >> 19u) & 7u;
    unsigned selector = (control >> 22u) & 0xFu;
    bool write = (control & (1u << 27u)) != 0;
    uint8_t address = 0u;

    rtc->done = false;
    *debug &= ~(RTC_I2C_DEBUG_BYTE_TRANS | RTC_I2C_DEBUG_BUS_BUSY |
                RTC_I2C_DEBUG_ARB_LOST | RTC_I2C_DEBUG_TIMED_OUT |
                RTC_I2C_DEBUG_ACK_VAL);
    *debug |= RTC_I2C_DEBUG_BUS_BUSY;

    if ((rtc->regs[RTC_I2C_CTRL_OFF / 4u] & RTC_I2C_CTRL_MASTER) == 0) {
        *debug &= ~RTC_I2C_DEBUG_BUS_BUSY;
        *debug |= RTC_I2C_DEBUG_TIMED_OUT;
        rtc->int_raw |= RTC_I2C_INT_RAW_TIMEOUT;
        rtc->done = true;
        return;
    }

    bool selected = rtc_i2c_selected_address(p, selector, &address);
    i2c_device_t *device = selected ? &rtc->device[address] : NULL;
    bool target_present = device && device->fn;
    int result = -1;

    if (write) {
        unsigned width = high_bit >= low_bit ? high_bit - low_bit + 1u : 0u;
        uint8_t field_mask = width >= 8u ? 0xFFu :
                             width == 0u ? 0u :
                             (uint8_t)((1u << width) - 1u);
        uint8_t wire_value = (uint8_t)((value & field_mask) << low_bit);
        uint8_t data[2] = {subaddress, wire_value};
        if (target_present)
            result = device->fn(device->ctx, PERIPH_I2C_PORT_RTC, address,
                                data, sizeof(data), NULL, 0u);
        i2c_emit_transfer(PERIPH_I2C_PORT_RTC, address, false,
                          data, sizeof(data));
    } else {
        uint8_t read_value = 0xFFu;
        if (target_present)
            result = device->fn(device->ctx, PERIPH_I2C_PORT_RTC, address,
                                &subaddress, 1u, &read_value, 1u);
        i2c_emit_transfer(PERIPH_I2C_PORT_RTC, address, false,
                          &subaddress, 1u);
        i2c_emit_transfer(PERIPH_I2C_PORT_RTC, address, true,
                          &read_value, 1u);
        rtc->data = read_value;
    }

    *debug &= ~RTC_I2C_DEBUG_BUS_BUSY;
    *debug |= RTC_I2C_DEBUG_BYTE_TRANS;
    if (result != 0)
        *debug |= RTC_I2C_DEBUG_ACK_VAL;
    rtc->int_raw |= RTC_I2C_INT_RAW_TRANS_DONE |
                    RTC_I2C_INT_RAW_MASTER_DONE;
    rtc->done = true;
}

static uint32_t rtc_i2c_read(esp32_periph_t *p, uint32_t off) {
    rtc_i2c_state_t *rtc = &p->rtc_i2c;
    if ((off & 3u) != 0 || off >= RTC_I2C_REG_FILE_SIZE)
        return 0u;

    switch (off) {
    case RTC_I2C_DATA_OFF:
        return rtc->data;
    case RTC_I2C_INT_RAW_OFF:
        return rtc->int_raw & RTC_I2C_INT_RAW_MASK;
    case RTC_I2C_INT_CLR_OFF:
        return 0u;
    case RTC_I2C_INT_ENA_OFF:
        return rtc->int_ena;
    case RTC_I2C_INT_ST_OFF:
        return ((rtc->int_raw << 1u) & rtc->int_ena) &
               RTC_I2C_INT_ENA_MASK;
    default:
        return rtc->regs[off / 4u];
    }
}

static void rtc_i2c_write(esp32_periph_t *p, uint32_t off, uint32_t val) {
    rtc_i2c_state_t *rtc = &p->rtc_i2c;
    if ((off & 3u) != 0 || off >= RTC_I2C_REG_FILE_SIZE)
        return;

    if (off >= RTC_I2C_COMMAND0_OFF &&
        off < RTC_I2C_COMMAND0_OFF + RTC_I2C_COMMAND_COUNT * 4u) {
        rtc->regs[off / 4u] = val & RTC_I2C_COMMAND_MASK;
        return;
    }

    switch (off) {
    case RTC_I2C_SCL_LOW_OFF:
        rtc->regs[off / 4u] = val & RTC_I2C_PERIOD19_MASK;
        break;
    case RTC_I2C_CTRL_OFF:
        rtc->regs[off / 4u] = val & RTC_I2C_CTRL_MASK;
        break;
    case RTC_I2C_DEBUG_STATUS_OFF:
        rtc->regs[off / 4u] = val & RTC_I2C_DEBUG_MASK;
        break;
    case RTC_I2C_TIMEOUT_OFF:
    case RTC_I2C_SDA_DUTY_OFF:
    case RTC_I2C_SCL_HIGH_OFF:
    case RTC_I2C_SCL_START_OFF:
    case RTC_I2C_SCL_STOP_OFF:
        rtc->regs[off / 4u] = val & RTC_I2C_PERIOD20_MASK;
        break;
    case RTC_I2C_SLAVE_ADDR_OFF:
        rtc->regs[off / 4u] = val & RTC_I2C_SLAVE_ADDR_MASK;
        break;
    case RTC_I2C_DATA_OFF:
    case RTC_I2C_INT_RAW_OFF:
    case RTC_I2C_INT_ST_OFF:
        break;
    case RTC_I2C_INT_CLR_OFF:
        rtc->int_raw &= ~((val & RTC_I2C_INT_SHIFTED_MASK) >> 1u);
        break;
    case RTC_I2C_INT_ENA_OFF:
        rtc->int_ena = val & RTC_I2C_INT_ENA_MASK;
        break;
    default:
        break;
    }
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

/* The mirror of periph_i2c_attach_device(): there the host supplies a device
 * for the guest's master to talk to, here the host *is* the master and the
 * guest is the slave. The guest never originates a slave transfer, so nothing
 * on its side can start one -- it has to arrive from outside.
 *
 * Returns the number of bytes the slave accepted, or -1 if it is not in slave
 * mode or the address does not match. */
int periph_i2c_master_xfer(esp32_periph_t *p, int port, uint8_t address,
                           const uint8_t *wr, size_t wrlen,
                           uint8_t *rd, size_t rdlen)
{
    if (!p || port < 0 || port >= I2C_PORT_COUNT) return -1;
    i2c_state_t *i2c = &p->i2c[port];

    if (i2c->regs[I2C_CTR_OFF / 4u] & I2C_CTR_MS_MODE) return -1;
    uint32_t own = i2c->regs[I2C_SLAVE_ADDR_OFF / 4u] & I2C_SLAVE_ADDR_MASK;
    if ((own & 0x7Fu) != (uint32_t)(address & 0x7Fu)) return -1;

    i2c->slave_addressed = true;
    int accepted = 0;

    /* A write from the master lands in the RX FIFO, which is where
     * i2c_slave_read_buffer() drains it. Overflow is reported rather than
     * silently dropped: a slave that is not being serviced fast enough is a
     * real condition firmware handles. */
    for (size_t i = 0; i < wrlen; i++) {
        if (!i2c_rx_push(i2c, wr[i])) {
            i2c->int_raw |= I2C_INT_RXFIFO_OVF;
            break;
        }
        accepted++;
    }
    if (wrlen) {
        uint32_t threshold = i2c->regs[I2C_FIFO_CONF_OFF / 4u] & 0x1Fu;
        if (i2c->rx_count > threshold)
            i2c->int_raw |= I2C_INT_RXFIFO_FULL;
    }

    /* A read is answered from whatever the guest left in the TX FIFO. Real
     * hardware clocks out 0xFF when the slave has nothing staged. */
    for (size_t i = 0; i < rdlen; i++)
        rd[i] = i2c->tx_count ? i2c_tx_pop(i2c) : 0xFFu;
    if (rdlen && i2c->tx_count == 0)
        i2c->int_raw |= I2C_INT_TXFIFO_EMPTY;

    i2c->int_raw |= I2C_INT_SLAVE_TRAN_COMP | I2C_INT_TRANS_DONE;
    i2c_intr_update(p, port);
    return accepted;
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
               (i2c->slave_addressed ? I2C_SR_SLAVE_ADDRESSED : 0) |
               (i2c->ack_nack ? 1u : 0);
    case I2C_RXFIFO_ST_OFF:
        return ((uint32_t)(i2c->tx_head & 0x1Fu) << 15) |
               ((uint32_t)(i2c->tx_tail & 0x1Fu) << 10) |
               ((uint32_t)(i2c->rx_head & 0x1Fu) << 5) |
               (uint32_t)(i2c->rx_tail & 0x1Fu);
    case I2C_DATA_OFF: {
        uint8_t byte = i2c_rx_pop(i2c);
        i2c_intr_update(p, i2c_port_from_addr(addr));
        return byte;
    }
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
        /* The command list belongs to master mode. A slave has no say in when
         * a transfer happens -- it answers one -- so running the list here
         * would have the port originate traffic it should be receiving. */
        if ((val & I2C_CTR_TRANS_START) && (val & I2C_CTR_MS_MODE))
            i2c_execute(p, port);
        break;
    case I2C_FIFO_CONF_OFF:
        i2c->regs[off / 4u] = val;
        if (val & I2C_FIFO_TX_RST)
            i2c_tx_reset(i2c);
        if (val & I2C_FIFO_RX_RST)
            i2c_rx_reset(i2c);
        i2c_intr_update(p, port);
        break;
    case I2C_DATA_OFF:
        i2c_tx_push(i2c, (uint8_t)val);
        i2c_intr_update(p, port);
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
        periph_assert_interrupt_status(p, RMT_INTR_SOURCE,
                                       p->rmt.int_raw & p->rmt.int_ena);
    else
        periph_deassert_interrupt(p, RMT_INTR_SOURCE);
}

static void rmt_kick(esp32_periph_t *p) {
    periph_event_source_changed(p, PERIPH_EVENT_RMT);
    for (int core = 0; core < 2; core++)
        if (p->cpu[core]) xtensa_recompute_next_timer(p->cpu[core]);
}

static void rmt_tx_cancel(rmt_channel_state_t *channel) {
    channel->tx_active = false;
    channel->tx_event_armed = false;
    channel->tx_waiting_for_threshold_clear = false;
    channel->pending_kind = RMT_TX_EVENT_NONE;
    channel->pending_count = 0;
    channel->conf1 &= ~RMT_CONF1_TX_START;
}

/* Snapshot the next immutable portion of the active memory ring. Threshold
 * delivery happens before the following portion is sampled, so the genuine
 * driver can safely refill the half which hardware has just consumed. */
static void rmt_plan_tx_segment_at(esp32_periph_t *p, unsigned channel_index,
                                   uint64_t start_cycle) {
    rmt_state_t *rmt = &p->rmt;
    rmt_channel_state_t *channel = &rmt->channel[channel_index];
    if (!channel->tx_active || channel->tx_event_armed) return;

    size_t capacity = rmt_channel_capacity(rmt, channel_index);
    if (capacity == 0) {
        channel->pending_kind = RMT_TX_EVENT_ERROR;
        channel->pending_count = 0;
        channel->pending_next_index = 0;
        channel->pending_next_threshold = 0;
        channel->next_tx_cycle = start_cycle + 1u;
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
    channel->next_tx_cycle = start_cycle + cycles;
    channel->tx_event_armed = true;
}

static void rmt_plan_tx_segment(esp32_periph_t *p, unsigned channel_index) {
    rmt_plan_tx_segment_at(p, channel_index,
                           periph_clock_now(p, &p->event_clock));
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
    if (!p || !cpu) return UINT32_MAX;
    bool have = false;
    uint64_t best = 0;
    for (unsigned index = 0; index < RMT_CHANNEL_COUNT; index++) {
        rmt_channel_state_t *channel = &p->rmt.channel[index];
        if (!channel->tx_event_armed) continue;
        if (!have || channel->next_tx_cycle < best) {
            have = true;
            best = channel->next_tx_cycle;
        }
    }
    if (!have) return UINT32_MAX;
    return periph_deadline_ccount(p, &p->event_clock, cpu, best);
}

static void rmt_eval_events(esp32_periph_t *p, xtensa_cpu_t *cpu) {
    if (!p || !cpu) return;
    uint64_t now_cycle = periph_clock_now(p, &p->event_clock);
    for (unsigned index = 0; index < RMT_CHANNEL_COUNT; index++) {
        rmt_channel_state_t *channel = &p->rmt.channel[index];
        /* A wait stub may advance CCOUNT across several RMT deadlines in one
         * dispatch. Drain finite transfers completely, while bounding a
         * continuously looping channel so it cannot monopolize the CPU. */
        unsigned drained = 0;
        while (channel->tx_event_armed &&
               now_cycle >= channel->next_tx_cycle &&
               drained++ < 4096u) {
            uint64_t event_cycle = channel->next_tx_cycle;

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
            {
                uint32_t threshold_bit = RMT_TX_THRESHOLD_INT(index);
                bool threshold_irq_enabled =
                        (p->rmt.int_ena & threshold_bit) != 0;
                p->rmt.int_raw |= threshold_bit;
                /* A compatibility dispatcher may run the refill ISR inline.
                 * A native guest ISR runs after this event hook returns. Do
                 * not let an overshot host time slice consume another half
                 * first: two thresholds would collapse into one level IRQ,
                 * leaving alternating ping-pong refills out of phase and
                 * repeating stale LED data. INT_CLR resumes transmission
                 * after the guest has written the consumed half. */
                rmt_irq_update(p);
                if (channel->tx_active &&
                    (!threshold_irq_enabled ||
                     !(p->rmt.int_raw & threshold_bit)))
                    rmt_plan_tx_segment_at(p, index, event_cycle);
                else if (channel->tx_active)
                    channel->tx_waiting_for_threshold_clear = true;
                break;
            }
            case RMT_TX_EVENT_END:
                rmt_tx_cancel(channel);
                p->rmt.int_raw |= RMT_TX_END_INT(index);
                rmt_irq_update(p);
                break;
            case RMT_TX_EVENT_LOOP:
                channel->tx_index = 0;
                if (channel->tx_active)
                    rmt_plan_tx_segment_at(p, index, event_cycle);
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
        for (unsigned index = 0; index < RMT_CHANNEL_COUNT; index++) {
            rmt_channel_state_t *channel = &rmt->channel[index];
            if (!(value & RMT_TX_THRESHOLD_INT(index)) ||
                !channel->tx_waiting_for_threshold_clear)
                continue;
            channel->tx_waiting_for_threshold_clear = false;
            if (channel->tx_active && !channel->tx_event_armed)
                rmt_plan_tx_segment(p, index);
        }
        rmt_irq_update(p);
        rmt_kick(p);
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
    periph_event_source_changed(p, PERIPH_EVENT_PCNT);
    for (int core = 0; core < 2; core++)
        if (p->cpu[core]) xtensa_recompute_next_timer(p->cpu[core]);
}

static void pcnt_update_irq(esp32_periph_t *p) {
    uint32_t raw = p->pcnt.regs[PCNT_INT_RAW_OFF / 4u];
    uint32_t ena = p->pcnt.regs[PCNT_INT_ENA_OFF / 4u];
    if (raw & ena)
        periph_assert_interrupt_status(p, PCNT_INTR_SOURCE, raw & ena);
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

    if (!(conf0 & PCNT_CONF_FILTER_EN) || threshold == 0u ||
        (!p->cpu[0] && !p->cpu[1])) {
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
        filter->deadline = periph_clock_now(p, &p->event_clock) +
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
    if (!p || !cpu) return UINT32_MAX;
    if (p->pcnt.pending_filters == 0) return UINT32_MAX;
    bool have = false;
    uint64_t best = 0;
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
                if (!have || filter->deadline < best) {
                    have = true;
                    best = filter->deadline;
                }
            }
        }
    }
    if (!have) return UINT32_MAX;
    return periph_deadline_ccount(p, &p->event_clock, cpu, best);
}

static void pcnt_eval_filter(esp32_periph_t *p, unsigned unit,
                             unsigned channel, bool control, uint64_t now) {
    pcnt_unit_state_t *state = &p->pcnt.unit[unit];
    pcnt_filter_state_t *filter = control ?
        &state->control_filter[channel] : &state->pulse_filter[channel];
    if (!filter->pending || now < filter->deadline)
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
    if (!p || !cpu) return;
    if (p->pcnt.pending_filters == 0) return;
    uint64_t now_cycle = periph_clock_now(p, &p->event_clock);
    for (unsigned unit = 0; unit < PCNT_UNIT_COUNT; unit++) {
        for (unsigned channel = 0; channel < PCNT_CHANNEL_COUNT; channel++) {
            /* Qualify control first so a pulse edge expiring at the same APB
             * cycle observes the newly stable control level. */
            pcnt_eval_filter(p, unit, channel, true, now_cycle);
            pcnt_eval_filter(p, unit, channel, false, now_cycle);
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
    return periph_clock_now(p, &p->mcpwm.clock);
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
        periph_assert_interrupt_status(p, mcpwm_unit_interrupt_source(unit),
                                       raw & ena);
    else
        periph_deassert_interrupt(p, mcpwm_unit_interrupt_source(unit));
}

static void mcpwm_kick(esp32_periph_t *p) {
    periph_event_source_changed(p, PERIPH_EVENT_MCPWM);
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
    return periph_clock_now(p, &p->ledc_clock);
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
        periph_assert_interrupt_status(p, LEDC_INTR_SOURCE, raw & ena);
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
    periph_event_source_changed(p, PERIPH_EVENT_LEDC);
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
    return periph_deadline_ccount(p, &p->ledc_clock, cpu, deadline);
}

static uint32_t ledc_next_fire(esp32_periph_t *p, xtensa_cpu_t *cpu) {
    if (!p || !cpu) return UINT32_MAX;
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
    if (!p || !cpu) return;
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

/* ---- UHCI0/UHCI1 UART DMA ---- */

static const uint32_t uhci_bases[UHCI_PORT_COUNT] = {
    UHCI0_BASE, UHCI1_BASE,
};

static const int uhci_intr_sources[UHCI_PORT_COUNT] = {12, 13};

static int uhci_port_from_addr(uint32_t addr) {
    for (int port = 0; port < UHCI_PORT_COUNT; port++) {
        if (addr >= uhci_bases[port] && addr < uhci_bases[port] + PAGE_SIZE)
            return port;
    }
    return -1;
}

static uint32_t uhci_dport_bit(unsigned port) {
    return port == 0u ? DPORT_UHCI0_MODULE_BIT : DPORT_UHCI1_MODULE_BIT;
}

static bool uhci_clocked(const esp32_periph_t *p, unsigned port) {
    uint32_t bit = uhci_dport_bit(port);
    return (p->dport_perip_clk_en & bit) != 0u &&
           (p->dport_perip_rst_en & bit) == 0u;
}

static int uhci_uart_num(const uhci_state_t *s) {
    uint32_t selected =
        (s->regs[UHCI_CONF0_OFF / 4u] & UHCI_CONF0_UART_CE_MASK) >> 9;
    for (int uart = 0; uart < UART_COUNT; uart++)
        if (selected & (1u << uart)) return uart;
    return -1;
}

static bool uhci_dma_range_mapped(esp32_periph_t *p, uint32_t addr,
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

static uint32_t uhci_first_desc(uint32_t link) {
    return 0x3FF00000u | (link & UHCI_LINK_ADDR_MASK);
}

static uint32_t uhci_next_desc(uint32_t next) {
    return next != 0u && next < 0x00100000u ? 0x3FF00000u | next : next;
}

static void uhci_irq_update(esp32_periph_t *p, unsigned port) {
    uhci_state_t *s = &p->uhci[port];
    bool active = uhci_clocked(p, port) &&
                  (s->int_raw & s->int_ena & UHCI_INT_VALID_MASK) != 0u;
    if (active)
        periph_assert_interrupt_status(p, uhci_intr_sources[port],
                                       s->int_raw & s->int_ena &
                                       UHCI_INT_VALID_MASK);
    else
        periph_deassert_interrupt(p, uhci_intr_sources[port]);
}

static xtensa_cpu_t *uhci_event_cpu(esp32_periph_t *p) {
    return p->cpu[0] ? p->cpu[0] : p->cpu[1];
}

static void uhci_kick(esp32_periph_t *p) {
    periph_event_source_changed(p, PERIPH_EVENT_UHCI);
    for (int core = 0; core < 2; core++)
        if (p->cpu[core]) xtensa_recompute_next_timer(p->cpu[core]);
}

static uint16_t uhci_crc16_update(uint16_t crc, uint8_t byte) {
    crc ^= byte;
    for (unsigned bit = 0; bit < 8u; bit++)
        crc = (crc & 1u) ? (uint16_t)((crc >> 1) ^ 0x8408u) :
                           (uint16_t)(crc >> 1);
    return crc;
}

static uint16_t uhci_bit_reverse16(uint16_t value) {
    value = (uint16_t)(((value & 0x5555u) << 1) |
                       ((value >> 1) & 0x5555u));
    value = (uint16_t)(((value & 0x3333u) << 2) |
                       ((value >> 2) & 0x3333u));
    value = (uint16_t)(((value & 0x0F0Fu) << 4) |
                       ((value >> 4) & 0x0F0Fu));
    return (uint16_t)((value << 8) | (value >> 8));
}

static uint32_t uhci_uart_cycles(const esp32_periph_t *p, int uart_num,
                                 size_t wire_bytes) {
    if (uart_num < 0 || uart_num >= UART_COUNT || wire_bytes == 0u) return 1u;
    const uart_state_t *uart = &p->uart[uart_num];
    uint32_t conf0 = uart->shadow[0x20u / 4u];
    uint32_t clkdiv = uart->shadow[0x14u / 4u];
    uint32_t divider16 = (clkdiv & 0xFFFFFu) * 16u +
                         ((clkdiv >> 20) & 0xFu);
    uint32_t source_hz = conf0 & (1u << 27) ? 80000000u : 1000000u;
    uint32_t baud = divider16 ?
        (uint32_t)(((uint64_t)source_hz * 16u) / divider16) : 115200u;
    if (baud == 0u) baud = 1u;

    uint32_t data_bits = ((conf0 >> 2) & 3u) + 5u;
    uint32_t parity_bits = conf0 & (1u << 1) ? 1u : 0u;
    uint32_t stop_mode = (conf0 >> 4) & 3u;
    uint32_t stop_half_bits = stop_mode == 2u ? 3u :
                              stop_mode == 3u ? 4u : 2u;
    uint32_t frame_half_bits = 2u * (1u + data_bits + parity_bits) +
                               stop_half_bits;
    uint64_t numerator = (uint64_t)wire_bytes * frame_half_bits *
                         timg_cpu_mhz(p) * 1000000ull;
    uint64_t denominator = (uint64_t)baud * 2u;
    uint64_t cycles = (numerator + denominator - 1u) / denominator;
    if (cycles == 0u) cycles = 1u;
    if (cycles > INT32_MAX) cycles = INT32_MAX;
    return (uint32_t)cycles;
}

static void uhci_update_desc_history(uhci_state_t *s, bool tx,
                                     uint32_t desc) {
    uint32_t current_off = tx ? UHCI_OUT_DSCR_OFF : UHCI_IN_DSCR_OFF;
    uint32_t previous_off = tx ? UHCI_OUT_DSCR_BF0_OFF : UHCI_IN_DSCR_BF0_OFF;
    uint32_t older_off = tx ? UHCI_OUT_DSCR_BF1_OFF : UHCI_IN_DSCR_BF1_OFF;
    s->regs[older_off / 4u] = s->regs[previous_off / 4u];
    s->regs[previous_off / 4u] = s->regs[current_off / 4u];
    s->regs[current_off / 4u] = desc;
}

static void uhci_reset_tx_frame(uhci_state_t *s) {
    s->tx_frame_started = false;
    s->tx_header_len = 0u;
    s->tx_payload_count = 0u;
    s->tx_crc = 0xFFFFu;
    s->encode_state = 0u;
}

static void uhci_reset_rx_frame(uhci_state_t *s) {
    s->rx_frame_active = false;
    s->rx_escape_pending = false;
    s->rx_escape_prefix = 0u;
    s->rx_frame_len = 0u;
    s->rx_payload_count = 0u;
    s->rx_frame_error_cause = 0u;
    s->decode_state = 0u;
}

static void uhci_reset_state(esp32_periph_t *p, unsigned port) {
    if (!p || port >= UHCI_PORT_COUNT) return;
    periph_deassert_interrupt(p, uhci_intr_sources[port]);
    uhci_state_t *s = &p->uhci[port];
    memset(s, 0, sizeof(*s));
    s->regs[UHCI_CONF0_OFF / 4u] = UHCI_CONF0_RESET;
    s->regs[UHCI_DMA_IN_LINK_OFF / 4u] = UHCI_INLINK_AUTO_RET;
    s->regs[UHCI_CONF1_OFF / 4u] = UHCI_CONF1_RESET;
    s->regs[UHCI_ESCAPE_CONF_OFF / 4u] = UHCI_ESCAPE_CONF_RESET;
    s->regs[UHCI_HUNG_CONF_OFF / 4u] = UHCI_HUNG_CONF_RESET;
    s->regs[UHCI_ESC_CONF0_OFF / 4u] = UHCI_ESC_CONF0_RESET;
    s->regs[UHCI_ESC_CONF1_OFF / 4u] = UHCI_ESC_CONF1_RESET;
    s->regs[UHCI_ESC_CONF2_OFF / 4u] = UHCI_ESC_CONF2_RESET;
    s->regs[UHCI_ESC_CONF3_OFF / 4u] = UHCI_ESC_CONF3_RESET;
    s->regs[UHCI_PKT_THRES_OFF / 4u] = UHCI_PKT_THRES_RESET;
    s->regs[UHCI_DATE_OFF / 4u] = UHCI_DATE_RESET;
    uhci_reset_tx_frame(s);
    uhci_reset_rx_frame(s);
}

static bool uhci_tx_escape_pair(const uhci_state_t *s, uint8_t byte,
                                uint8_t *first, uint8_t *second) {
    if (!(s->regs[UHCI_CONF0_OFF / 4u] & UHCI_CONF0_SEPER_EN)) return false;
    uint32_t enable = s->regs[UHCI_ESCAPE_CONF_OFF / 4u];
    static const uint32_t config_offsets[4] = {
        UHCI_ESC_CONF0_OFF, UHCI_ESC_CONF1_OFF,
        UHCI_ESC_CONF2_OFF, UHCI_ESC_CONF3_OFF,
    };
    for (unsigned index = 0; index < 4u; index++) {
        if (!(enable & (1u << (index + 4u)))) continue;
        uint32_t config = s->regs[config_offsets[index] / 4u];
        if (byte != (uint8_t)config) continue;
        *first = (uint8_t)(config >> 8);
        *second = (uint8_t)(config >> 16);
        return true;
    }
    return false;
}

static size_t uhci_emit_tx_octet(esp32_periph_t *p, unsigned port,
                                 int uart_num, uint8_t byte) {
    uhci_state_t *s = &p->uhci[port];
    uint8_t first = 0u;
    uint8_t second = 0u;
    if (uhci_tx_escape_pair(s, byte, &first, &second)) {
        uart_emit_tx_byte(p, uart_num, first, false);
        uart_emit_tx_byte(p, uart_num, second, false);
        return 2u;
    }
    uart_emit_tx_byte(p, uart_num, byte, false);
    return 1u;
}

static void uhci_emit_separator(esp32_periph_t *p, unsigned port,
                                int uart_num) {
    uhci_state_t *s = &p->uhci[port];
    uint8_t separator =
        (uint8_t)s->regs[UHCI_ESC_CONF0_OFF / 4u];
    uart_emit_tx_byte(p, uart_num, separator, false);
}

static bool uhci_tx_begin_frame(esp32_periph_t *p, unsigned port,
                                int uart_num) {
    uhci_state_t *s = &p->uhci[port];
    if (s->tx_frame_started) return true;
    if (s->regs[UHCI_CONF0_OFF / 4u] & UHCI_CONF0_SEPER_EN) {
        uhci_emit_separator(p, port, uart_num);
        s->int_raw |= UHCI_INT_RX_START;
    }
    s->tx_frame_started = true;
    s->tx_header_len = 0u;
    s->tx_payload_count = 0u;
    s->tx_crc = 0xFFFFu;
    s->encode_state = 1u;
    return true;
}

static void uhci_tx_emit_header(esp32_periph_t *p, unsigned port,
                                int uart_num) {
    uhci_state_t *s = &p->uhci[port];
    uint32_t conf1 = s->regs[UHCI_CONF1_OFF / 4u];
    if (conf1 & UHCI_CONF1_TX_ACK_NUM_RE) {
        uint8_t ack = (uint8_t)(s->regs[UHCI_ACK_NUM_OFF / 4u] & 7u);
        s->tx_header[0] = (uint8_t)((s->tx_header[0] & ~0x38u) |
                                    (ack << 3));
    }
    if (conf1 & UHCI_CONF1_TX_CHECK_SUM_RE)
        s->tx_header[3] = (uint8_t)~(s->tx_header[0] + s->tx_header[1] +
                                     s->tx_header[2]);
    for (unsigned index = 0; index < 4u; index++) {
        s->tx_crc = uhci_crc16_update(s->tx_crc, s->tx_header[index]);
        (void)uhci_emit_tx_octet(p, port, uart_num, s->tx_header[index]);
    }
}

static void uhci_tx_emit_data(esp32_periph_t *p, unsigned port, int uart_num,
                              const uint8_t *data, size_t len) {
    uhci_state_t *s = &p->uhci[port];
    bool header_mode =
        (s->regs[UHCI_CONF0_OFF / 4u] & UHCI_CONF0_HEAD_EN) != 0u;
    for (size_t index = 0; index < len; index++) {
        uint8_t byte = data[index];
        if (header_mode && s->tx_header_len < 4u) {
            s->tx_header[s->tx_header_len++] = byte;
            if (s->tx_header_len == 4u)
                uhci_tx_emit_header(p, port, uart_num);
            continue;
        }
        if (header_mode) {
            s->tx_crc = uhci_crc16_update(s->tx_crc, byte);
            s->tx_payload_count++;
        }
        (void)uhci_emit_tx_octet(p, port, uart_num, byte);
    }
}

static bool uhci_tx_end_frame(esp32_periph_t *p, unsigned port,
                              int uart_num) {
    uhci_state_t *s = &p->uhci[port];
    uint32_t conf0 = s->regs[UHCI_CONF0_OFF / 4u];
    uint32_t conf1 = s->regs[UHCI_CONF1_OFF / 4u];
    bool valid = true;
    if (conf0 & UHCI_CONF0_HEAD_EN) {
        if (s->tx_header_len != 4u) {
            valid = false;
        } else {
            uint32_t expected = ((uint32_t)s->tx_header[1] >> 4) |
                                ((uint32_t)s->tx_header[2] << 4);
            if (expected != s->tx_payload_count) valid = false;
            if ((s->tx_header[0] & 0x40u) &&
                (conf0 & UHCI_CONF0_ENCODE_CRC_EN) &&
                !(conf1 & UHCI_CONF1_CRC_DISABLE)) {
                uint16_t crc = uhci_bit_reverse16(s->tx_crc);
                (void)uhci_emit_tx_octet(p, port, uart_num,
                                         (uint8_t)(crc >> 8));
                (void)uhci_emit_tx_octet(p, port, uart_num, (uint8_t)crc);
            }
        }
    }
    if (conf0 & UHCI_CONF0_SEPER_EN)
        uhci_emit_separator(p, port, uart_num);
    uhci_reset_tx_frame(s);
    return valid;
}

static size_t uhci_tx_wire_estimate(esp32_periph_t *p, unsigned port,
                                    uint32_t desc) {
    uhci_state_t *s = &p->uhci[port];
    if (!uhci_dma_range_mapped(p, desc, 12u, false)) return 1u;
    uint32_t ctrl = mem_read32(p->mem, desc);
    uint32_t buf = mem_read32(p->mem, desc + 4u);
    size_t len = (ctrl & UHCI_DESC_LENGTH_MASK) >> UHCI_DESC_LENGTH_SHIFT;
    if (len > UHCI_DMA_MAX_BUFFER ||
        (len != 0u && !uhci_dma_range_mapped(p, buf, len, false)))
        return 1u;
    size_t wire = len;
    uint32_t conf0 = s->regs[UHCI_CONF0_OFF / 4u];
    if ((conf0 & UHCI_CONF0_SEPER_EN) && !s->tx_frame_started) wire++;
    if ((ctrl & UHCI_DESC_EOF) && (conf0 & UHCI_CONF0_SEPER_EN)) wire++;
    /* Escaping can at most double the descriptor body. Counting exact bytes
     * keeps transparent-mode timing exact and framed timing conservative. */
    if (conf0 & UHCI_CONF0_SEPER_EN) {
        size_t escaped = 0u;
        for (size_t index = 0; index < len; index++) {
            uint8_t first = 0u, second = 0u;
            escaped += uhci_tx_escape_pair(
                s, mem_read8(p->mem, buf + (uint32_t)index),
                &first, &second) ? 2u : 1u;
        }
        wire = escaped + (wire - len);
    }
    if ((ctrl & UHCI_DESC_EOF) && (conf0 & UHCI_CONF0_HEAD_EN) &&
        s->tx_header_len == 4u && (s->tx_header[0] & 0x40u) &&
        (conf0 & UHCI_CONF0_ENCODE_CRC_EN))
        wire += 2u;
    return wire ? wire : 1u;
}

static void uhci_arm_tx_event(esp32_periph_t *p, unsigned port) {
    uhci_state_t *s = &p->uhci[port];
    s->tx_event_armed = false;
    if (!s->tx_link_running || !uhci_clocked(p, port)) return;
    if ((s->regs[UHCI_CONF1_OFF / 4u] & UHCI_CONF1_WAIT_SW_START) &&
        !(s->regs[UHCI_CONF1_OFF / 4u] & UHCI_CONF1_SW_START)) {
        s->encode_state = 5u;
        return;
    }
    int uart_num = uhci_uart_num(s);
    if (uart_num < 0) {
        s->int_raw |= UHCI_INT_TX_HUNG;
        s->tx_link_running = false;
        uhci_irq_update(p, port);
        return;
    }
    xtensa_cpu_t *cpu = uhci_event_cpu(p);
    if (!cpu) return;
    size_t wire = uhci_tx_wire_estimate(p, port, s->tx_desc);
    s->next_tx_cycle = periph_clock_now(p, &p->event_clock) +
                       uhci_uart_cycles(p, uart_num, wire);
    s->tx_event_armed = true;
    uhci_kick(p);
}

static void uhci_tx_descriptor_error(esp32_periph_t *p, unsigned port,
                                     uint32_t desc, uint32_t interrupt) {
    uhci_state_t *s = &p->uhci[port];
    s->regs[UHCI_OUT_EOF_BFR_DESC_OFF / 4u] = desc;
    s->int_raw |= interrupt;
    s->tx_link_running = false;
    s->tx_event_armed = false;
    s->encode_state = 0u;
    uhci_irq_update(p, port);
    uhci_kick(p);
}

static bool uhci_complete_tx_descriptor(esp32_periph_t *p, unsigned port) {
    uhci_state_t *s = &p->uhci[port];
    uint32_t desc = s->tx_desc;
    if (!s->tx_link_running) return false;
    if (++s->tx_descriptors_seen > UHCI_DMA_MAX_DESCRIPTORS ||
        (desc & 3u) != 0u ||
        !uhci_dma_range_mapped(p, desc, 12u, true)) {
        uhci_tx_descriptor_error(p, port, desc, UHCI_INT_OUT_DSCR_ERR);
        return false;
    }

    uint32_t ctrl = mem_read32(p->mem, desc);
    uint32_t buf = mem_read32(p->mem, desc + 4u);
    uint32_t next = uhci_next_desc(mem_read32(p->mem, desc + 8u));
    size_t size = ctrl & UHCI_DESC_SIZE_MASK;
    size_t len = (ctrl & UHCI_DESC_LENGTH_MASK) >> UHCI_DESC_LENGTH_SHIFT;
    uhci_update_desc_history(s, true, desc);

    bool check_owner =
        (s->regs[UHCI_CONF1_OFF / 4u] & UHCI_CONF1_CHECK_OWNER) != 0u;
    if ((check_owner && !(ctrl & UHCI_DESC_OWNER)) || len > size ||
        size > UHCI_DMA_MAX_BUFFER || (buf & 3u) != 0u ||
        (size != 0u && (size & 3u) != 0u) ||
        (len != 0u && !uhci_dma_range_mapped(p, buf, len, false))) {
        uhci_tx_descriptor_error(p, port, desc, UHCI_INT_OUT_DSCR_ERR);
        return false;
    }
    if (len == 0u && !(s->regs[UHCI_CONF0_OFF / 4u] &
                       UHCI_CONF0_OUT_AUTO_WRBACK)) {
        uhci_tx_descriptor_error(p, port, desc, UHCI_INT_OUT_DSCR_ERR);
        return false;
    }

    int uart_num = uhci_uart_num(s);
    if (uart_num < 0) {
        uhci_tx_descriptor_error(p, port, desc, UHCI_INT_TX_HUNG);
        return false;
    }
    (void)uhci_tx_begin_frame(p, port, uart_num);
    uint8_t bytes[UHCI_DMA_MAX_BUFFER];
    for (size_t index = 0; index < len; index++)
        bytes[index] = mem_read8(p->mem, buf + (uint32_t)index);
    uhci_tx_emit_data(p, port, uart_num, bytes, len);

    mem_write32(p->mem, desc, ctrl & ~UHCI_DESC_OWNER);
    s->int_raw |= UHCI_INT_OUT_DONE;
    if (ctrl & UHCI_DESC_EOF) {
        s->regs[UHCI_OUT_EOF_DESC_OFF / 4u] = desc;
        s->regs[UHCI_OUT_EOF_BFR_DESC_OFF / 4u] =
            s->regs[UHCI_OUT_DSCR_BF0_OFF / 4u];
        s->int_raw |= UHCI_INT_OUT_EOF | UHCI_INT_OUT_TOTAL_EOF;
        if (next != 0u) s->int_raw |= UHCI_INT_OUTLINK_EOF_ERR;
        if (!uhci_tx_end_frame(p, port, uart_num))
            s->int_raw |= UHCI_INT_OUTLINK_EOF_ERR;
        s->tx_link_running = false;
        s->tx_desc = 0u;
        uart_dma_tx_done(p, uart_num);
    } else if (next == 0u || next == desc) {
        s->int_raw |= UHCI_INT_OUTLINK_EOF_ERR;
        s->regs[UHCI_OUT_EOF_BFR_DESC_OFF / 4u] = desc;
        s->tx_link_running = false;
        s->tx_desc = 0u;
        uhci_reset_tx_frame(s);
    } else {
        s->tx_desc = next;
    }
    uhci_irq_update(p, port);
    if (s->tx_link_running) uhci_arm_tx_event(p, port);
    else uhci_kick(p);
    return true;
}

static void uhci_emit_quick_packet(esp32_periph_t *p, unsigned port,
                                   unsigned packet) {
    uhci_state_t *s = &p->uhci[port];
    int uart_num = uhci_uart_num(s);
    if (uart_num < 0 || packet >= 7u) {
        s->int_raw |= UHCI_INT_TX_HUNG;
        return;
    }
    uint32_t first = s->regs[(UHCI_Q_DATA_FIRST_OFF + packet * 8u) / 4u];
    uint32_t second =
        s->regs[(UHCI_Q_DATA_FIRST_OFF + packet * 8u + 4u) / 4u];
    uint8_t data[8];
    for (unsigned index = 0; index < 4u; index++) {
        data[index] = (uint8_t)(first >> (index * 8u));
        data[index + 4u] = (uint8_t)(second >> (index * 8u));
    }
    if (s->regs[UHCI_CONF0_OFF / 4u] & UHCI_CONF0_SEPER_EN) {
        uhci_emit_separator(p, port, uart_num);
        s->int_raw |= UHCI_INT_RX_START;
    }
    for (unsigned index = 0; index < 8u; index++)
        (void)uhci_emit_tx_octet(p, port, uart_num, data[index]);
    if (s->regs[UHCI_CONF0_OFF / 4u] & UHCI_CONF0_SEPER_EN)
        uhci_emit_separator(p, port, uart_num);
    uart_dma_tx_done(p, uart_num);
}

static void uhci_arm_quick_event(esp32_periph_t *p, unsigned port) {
    uhci_state_t *s = &p->uhci[port];
    s->quick_event_armed = false;
    uint32_t quick = s->regs[UHCI_QUICK_SENT_OFF / 4u];
    if (!uhci_clocked(p, port)) return;
    if (quick & (1u << 3))
        s->quick_event_kind = 1u;
    else if (quick & (1u << 7))
        s->quick_event_kind = 2u;
    else
        return;
    int uart_num = uhci_uart_num(s);
    if (uart_num < 0) {
        s->int_raw |= UHCI_INT_TX_HUNG;
        uhci_irq_update(p, port);
        return;
    }
    xtensa_cpu_t *cpu = uhci_event_cpu(p);
    if (!cpu) return;
    size_t wire = 8u +
        ((s->regs[UHCI_CONF0_OFF / 4u] & UHCI_CONF0_SEPER_EN) ? 2u : 0u);
    s->next_quick_cycle = periph_clock_now(p, &p->event_clock) +
                          uhci_uart_cycles(p, uart_num, wire);
    s->quick_event_armed = true;
    uhci_kick(p);
}

static void uhci_complete_quick_event(esp32_periph_t *p, unsigned port) {
    uhci_state_t *s = &p->uhci[port];
    uint32_t quick = s->regs[UHCI_QUICK_SENT_OFF / 4u];
    unsigned packet = s->quick_event_kind == 1u ? (quick & 7u) :
                                                   ((quick >> 4) & 7u);
    uhci_emit_quick_packet(p, port, packet);
    if (s->quick_event_kind == 1u) {
        s->int_raw |= UHCI_INT_SEND_S_Q;
        s->regs[UHCI_QUICK_SENT_OFF / 4u] &= ~(1u << 3);
    } else {
        s->int_raw |= UHCI_INT_SEND_A_Q;
    }
    s->quick_event_armed = false;
    uhci_irq_update(p, port);
    if (s->regs[UHCI_QUICK_SENT_OFF / 4u] & (1u << 7))
        uhci_arm_quick_event(p, port);
}

static void uhci_debug_rx_push(uhci_state_t *s, uint16_t value) {
    if (s->debug_in_count < UHCI_DEBUG_FIFO_SIZE) {
        unsigned tail = (s->debug_in_head + s->debug_in_count) %
                        UHCI_DEBUG_FIFO_SIZE;
        s->debug_in_fifo[tail] = value & 0x0FFFu;
        s->debug_in_count++;
    }
    uint32_t threshold =
        (s->regs[UHCI_CONF1_OFF / 4u] >> 9) & 0x0FFFu;
    if (threshold != 0u && s->debug_in_count > threshold)
        s->int_raw |= UHCI_INT_IN_FIFO_FULL_WM;
}

static void uhci_reset_tx_path(uhci_state_t *s) {
    s->tx_desc = 0u;
    s->tx_descriptors_seen = 0u;
    s->tx_link_running = false;
    s->tx_event_armed = false;
    s->quick_event_armed = false;
    s->quick_event_kind = 0u;
    uhci_reset_tx_frame(s);
}

static void uhci_reset_rx_path(uhci_state_t *s) {
    s->rx_desc = 0u;
    s->rx_offset = 0u;
    s->rx_ctrl = 0u;
    s->rx_buf = 0u;
    s->rx_next = 0u;
    s->rx_size = 0u;
    s->rx_desc_loaded = false;
    s->rx_descriptors_seen = 0u;
    s->rx_link_running = false;
    s->rx_error_cause = 0u;
    s->debug_in_head = 0u;
    s->debug_in_count = 0u;
    s->debug_in_last = 0u;
    uhci_reset_rx_frame(s);
}

static void uhci_rx_descriptor_error(esp32_periph_t *p, unsigned port,
                                     uint32_t desc, uint32_t interrupt) {
    uhci_state_t *s = &p->uhci[port];
    s->regs[UHCI_IN_ERR_EOF_DESC_OFF / 4u] = desc;
    s->int_raw |= interrupt;
    s->rx_link_running = false;
    s->rx_desc_loaded = false;
    s->decode_state = 0u;
    uhci_irq_update(p, port);
}

static bool uhci_load_rx_descriptor(esp32_periph_t *p, unsigned port) {
    uhci_state_t *s = &p->uhci[port];
    if (s->rx_desc_loaded) return true;
    uint32_t desc = s->rx_desc;
    if (!s->rx_link_running || desc == 0u) return false;
    if (++s->rx_descriptors_seen > UHCI_DMA_MAX_DESCRIPTORS ||
        (desc & 3u) != 0u ||
        !uhci_dma_range_mapped(p, desc, 12u, true)) {
        uhci_rx_descriptor_error(p, port, desc, UHCI_INT_IN_DSCR_ERR);
        return false;
    }

    uint32_t ctrl = mem_read32(p->mem, desc);
    uint32_t buf = mem_read32(p->mem, desc + 4u);
    uint32_t next = uhci_next_desc(mem_read32(p->mem, desc + 8u));
    size_t size = ctrl & UHCI_DESC_SIZE_MASK;
    bool check_owner =
        (s->regs[UHCI_CONF1_OFF / 4u] & UHCI_CONF1_CHECK_OWNER) != 0u;
    uhci_update_desc_history(s, false, desc);
    if ((check_owner && !(ctrl & UHCI_DESC_OWNER)) || size == 0u ||
        size > UHCI_DMA_MAX_BUFFER || (size & 3u) != 0u ||
        (buf & 3u) != 0u ||
        !uhci_dma_range_mapped(p, buf, size, true)) {
        uhci_rx_descriptor_error(p, port, desc, UHCI_INT_IN_DSCR_ERR);
        return false;
    }

    s->rx_ctrl = ctrl;
    s->rx_buf = buf;
    s->rx_next = next;
    s->rx_size = size;
    s->rx_offset = 0u;
    s->rx_desc_loaded = true;
    s->decode_state = 2u;
    return true;
}

static bool uhci_complete_rx_descriptor(esp32_periph_t *p, unsigned port,
                                        bool eof, bool success) {
    uhci_state_t *s = &p->uhci[port];
    if (!s->rx_desc_loaded) return false;
    uint32_t desc = s->rx_desc;
    uint32_t ctrl = s->rx_ctrl &
        ~(UHCI_DESC_LENGTH_MASK | UHCI_DESC_EOF | UHCI_DESC_OWNER);
    ctrl |= ((uint32_t)s->rx_offset << UHCI_DESC_LENGTH_SHIFT) &
            UHCI_DESC_LENGTH_MASK;
    if (eof) ctrl |= UHCI_DESC_EOF;
    mem_write32(p->mem, desc, ctrl);
    s->int_raw |= UHCI_INT_IN_DONE;
    if (eof) {
        if (success) {
            s->regs[UHCI_IN_SUC_EOF_DESC_OFF / 4u] = desc;
            s->int_raw |= UHCI_INT_IN_SUC_EOF;
        } else {
            s->regs[UHCI_IN_ERR_EOF_DESC_OFF / 4u] = desc;
            s->int_raw |= UHCI_INT_IN_ERR_EOF;
        }
    }

    uint32_t next = s->rx_next;
    s->rx_desc_loaded = false;
    s->rx_offset = 0u;
    s->rx_ctrl = 0u;
    s->rx_buf = 0u;
    s->rx_next = 0u;
    s->rx_size = 0u;
    if (next != 0u && next != desc) {
        s->rx_desc = next;
    } else {
        s->rx_desc = 0u;
        s->rx_link_running = false;
        if (!eof) s->int_raw |= UHCI_INT_IN_DSCR_EMPTY;
    }
    s->decode_state = s->rx_link_running ? 1u : 0u;
    uhci_irq_update(p, port);
    return true;
}

static size_t uhci_write_rx_payload(esp32_periph_t *p, unsigned port,
                                    const uint8_t *data, size_t len,
                                    bool eof, bool success) {
    uhci_state_t *s = &p->uhci[port];
    size_t accepted = 0u;
    while (accepted < len) {
        if (!uhci_load_rx_descriptor(p, port)) break;
        size_t available = s->rx_size - s->rx_offset;
        size_t chunk = len - accepted;
        if (chunk > available) chunk = available;
        for (size_t index = 0; index < chunk; index++) {
            uint8_t byte = data[accepted + index];
            mem_write8(p->mem, s->rx_buf + (uint32_t)s->rx_offset +
                       (uint32_t)index, byte);
            uhci_debug_rx_push(s, byte);
        }
        s->rx_offset += chunk;
        accepted += chunk;
        bool final = eof && accepted == len;
        if (s->rx_offset == s->rx_size) {
            if (!uhci_complete_rx_descriptor(p, port, final,
                                             final ? success : true))
                break;
        }
    }

    if (eof && accepted == len) {
        if (!s->rx_desc_loaded && len == 0u)
            (void)uhci_load_rx_descriptor(p, port);
        if (s->rx_desc_loaded)
            (void)uhci_complete_rx_descriptor(p, port, true, success);
    }
    if (accepted < len && s->rx_link_running) {
        s->int_raw |= UHCI_INT_IN_DSCR_EMPTY;
        s->rx_link_running = false;
    }
    uhci_irq_update(p, port);
    return accepted;
}

static bool uhci_rx_escape_mapping(const uhci_state_t *s, uint8_t first,
                                   uint8_t second, uint8_t *decoded) {
    uint32_t enable = s->regs[UHCI_ESCAPE_CONF_OFF / 4u];
    static const uint32_t offsets[4] = {
        UHCI_ESC_CONF0_OFF, UHCI_ESC_CONF1_OFF,
        UHCI_ESC_CONF2_OFF, UHCI_ESC_CONF3_OFF,
    };
    for (unsigned index = 0; index < 4u; index++) {
        if (!(enable & (1u << index))) continue;
        uint32_t config = s->regs[offsets[index] / 4u];
        if (first == (uint8_t)(config >> 8) &&
            second == (uint8_t)(config >> 16)) {
            *decoded = (uint8_t)config;
            return true;
        }
    }
    return false;
}

static bool uhci_rx_escape_prefix(const uhci_state_t *s, uint8_t byte) {
    uint32_t enable = s->regs[UHCI_ESCAPE_CONF_OFF / 4u];
    static const uint32_t offsets[4] = {
        UHCI_ESC_CONF0_OFF, UHCI_ESC_CONF1_OFF,
        UHCI_ESC_CONF2_OFF, UHCI_ESC_CONF3_OFF,
    };
    for (unsigned index = 0; index < 4u; index++) {
        if (!(enable & (1u << index))) continue;
        if (byte == (uint8_t)(s->regs[offsets[index] / 4u] >> 8))
            return true;
    }
    return false;
}

static size_t uhci_rx_expected_frame(const uhci_state_t *s) {
    uint32_t conf0 = s->regs[UHCI_CONF0_OFF / 4u];
    uint32_t conf1 = s->regs[UHCI_CONF1_OFF / 4u];
    if (!(conf0 & UHCI_CONF0_HEAD_EN) || s->rx_frame_len < 4u)
        return 0u;
    size_t payload = ((size_t)s->rx_frame[1] >> 4) |
                     ((size_t)s->rx_frame[2] << 4);
    bool crc = (s->rx_frame[0] & 0x40u) != 0u &&
               (conf0 & UHCI_CONF0_CRC_REC_EN) != 0u &&
               (conf1 & UHCI_CONF1_CRC_DISABLE) == 0u;
    return 4u + payload + (crc ? 2u : 0u);
}

static bool uhci_finish_rx_frame(esp32_periph_t *p, unsigned port) {
    uhci_state_t *s = &p->uhci[port];
    uint32_t conf0 = s->regs[UHCI_CONF0_OFF / 4u];
    uint32_t conf1 = s->regs[UHCI_CONF1_OFF / 4u];
    bool head_mode = (conf0 & UHCI_CONF0_HEAD_EN) != 0u;
    bool success = true;
    uint8_t cause = s->rx_frame_error_cause;
    const uint8_t *payload = s->rx_frame;
    size_t payload_len = s->rx_frame_len;

    if (head_mode) {
        if (s->rx_frame_len < 4u) {
            success = false;
            if (cause == 0u) cause = 4u;
            payload_len = 0u;
        } else {
            uint8_t *header = s->rx_frame;
            uint32_t packed = (uint32_t)header[0] |
                              ((uint32_t)header[1] << 8) |
                              ((uint32_t)header[2] << 16) |
                              ((uint32_t)header[3] << 24);
            s->regs[UHCI_RX_HEAD_OFF / 4u] = packed;
            size_t declared = ((size_t)header[1] >> 4) |
                              ((size_t)header[2] << 4);
            bool crc_present = (header[0] & 0x40u) != 0u &&
                               (conf0 & UHCI_CONF0_CRC_REC_EN) != 0u &&
                               (conf1 & UHCI_CONF1_CRC_DISABLE) == 0u;
            size_t expected = 4u + declared + (crc_present ? 2u : 0u);
            if (s->rx_frame_len != expected) {
                success = false;
                if (cause == 0u)
                    cause = s->rx_frame_len < expected ? 4u : 5u;
            }
            if ((conf1 & UHCI_CONF1_CHECK_SUM_EN) &&
                (uint8_t)(header[0] + header[1] + header[2] + header[3]) !=
                    0xFFu) {
                success = false;
                if (cause == 0u) cause = 1u;
            }
            if ((conf1 & UHCI_CONF1_CHECK_SEQ_EN) &&
                (header[0] & 0x80u) &&
                (header[0] & 7u) != s->expected_rx_sequence) {
                success = false;
                if (cause == 0u) cause = 2u;
            }
            if (crc_present && s->rx_frame_len >= expected) {
                uint16_t crc = 0xFFFFu;
                for (size_t index = 0; index < 4u + declared; index++)
                    crc = uhci_crc16_update(crc, s->rx_frame[index]);
                crc = uhci_bit_reverse16(crc);
                uint16_t received =
                    ((uint16_t)s->rx_frame[4u + declared] << 8) |
                    s->rx_frame[5u + declared];
                if (crc != received) {
                    success = false;
                    if (cause == 0u) cause = 6u;
                }
            }
            if (success && (header[0] & 0x80u)) {
                s->expected_rx_sequence =
                    (uint8_t)((s->expected_rx_sequence + 1u) & 7u);
                s->regs[UHCI_ACK_NUM_OFF / 4u] = s->expected_rx_sequence;
            }
            if (conf1 & UHCI_CONF1_SAVE_HEAD) {
                payload = s->rx_frame;
                payload_len = 4u +
                    (declared < s->rx_frame_len - 4u ? declared :
                     s->rx_frame_len - 4u);
            } else {
                payload = s->rx_frame + 4u;
                payload_len = declared < s->rx_frame_len - 4u ? declared :
                              s->rx_frame_len - 4u;
            }
        }
    }

    s->rx_error_cause = cause;
    s->decode_state = success ? 3u : 4u;
    size_t written = uhci_write_rx_payload(p, port, payload, payload_len,
                                           true, success);
    bool accepted = written == payload_len;
    uhci_reset_rx_frame(s);
    uhci_irq_update(p, port);
    return accepted;
}

static bool uhci_append_rx_frame(esp32_periph_t *p, unsigned port,
                                 uint8_t byte) {
    uhci_state_t *s = &p->uhci[port];
    if (!s->rx_frame_active) {
        s->rx_frame_active = true;
        s->decode_state = 1u;
    }
    if (s->rx_frame_len >= sizeof(s->rx_frame)) {
        s->rx_frame_error_cause = 5u;
        (void)uhci_finish_rx_frame(p, port);
        return false;
    }
    s->rx_frame[s->rx_frame_len++] = byte;
    size_t expected = uhci_rx_expected_frame(s);
    uint32_t conf0 = s->regs[UHCI_CONF0_OFF / 4u];
    if ((conf0 & UHCI_CONF0_LEN_EOF_EN) && expected != 0u &&
        s->rx_frame_len == expected)
        return uhci_finish_rx_frame(p, port);
    if (!(conf0 & UHCI_CONF0_HEAD_EN) &&
        (conf0 & UHCI_CONF0_LEN_EOF_EN)) {
        size_t threshold =
            s->regs[UHCI_PKT_THRES_OFF / 4u] & 0x1FFFu;
        if (threshold != 0u && s->rx_frame_len >= threshold)
            return uhci_finish_rx_frame(p, port);
    }
    return true;
}

static size_t uhci_feed_framed_rx(esp32_periph_t *p, unsigned port,
                                  const uint8_t *data, size_t len) {
    uhci_state_t *s = &p->uhci[port];
    uint32_t conf0 = s->regs[UHCI_CONF0_OFF / 4u];
    uint8_t separator = (uint8_t)s->regs[UHCI_ESC_CONF0_OFF / 4u];
    size_t accepted = 0u;
    while (accepted < len && s->rx_link_running) {
        uint8_t byte = data[accepted++];
        if ((conf0 & UHCI_CONF0_SEPER_EN) && byte == separator) {
            s->int_raw |= UHCI_INT_TX_START;
            if (s->rx_escape_pending) {
                s->rx_frame_error_cause = 4u;
                s->rx_escape_pending = false;
            }
            if (s->rx_frame_active && s->rx_frame_len != 0u)
                (void)uhci_finish_rx_frame(p, port);
            s->rx_frame_active = true;
            s->decode_state = 1u;
            continue;
        }
        if ((conf0 & UHCI_CONF0_SEPER_EN) && s->rx_escape_pending) {
            uint8_t decoded = 0u;
            if (uhci_rx_escape_mapping(s, s->rx_escape_prefix, byte,
                                       &decoded)) {
                s->rx_escape_pending = false;
                if (!uhci_append_rx_frame(p, port, decoded)) break;
                continue;
            }
            uint8_t prefix = s->rx_escape_prefix;
            s->rx_escape_pending = false;
            if (!uhci_append_rx_frame(p, port, prefix) ||
                !uhci_append_rx_frame(p, port, byte))
                break;
            continue;
        }
        if ((conf0 & UHCI_CONF0_SEPER_EN) &&
            uhci_rx_escape_prefix(s, byte)) {
            s->rx_escape_pending = true;
            s->rx_escape_prefix = byte;
            continue;
        }
        if (!uhci_append_rx_frame(p, port, byte)) break;
    }
    uhci_irq_update(p, port);
    return accepted;
}

static size_t uhci_feed_transparent_rx(esp32_periph_t *p, unsigned port,
                                       const uint8_t *data, size_t len,
                                       bool idle_eof) {
    uhci_state_t *s = &p->uhci[port];
    size_t accepted = 0u;
    uint32_t conf0 = s->regs[UHCI_CONF0_OFF / 4u];
    size_t threshold =
        s->regs[UHCI_PKT_THRES_OFF / 4u] & 0x1FFFu;
    while (accepted < len && s->rx_link_running) {
        size_t chunk = len - accepted;
        if ((conf0 & UHCI_CONF0_LEN_EOF_EN) && threshold != 0u) {
            /* A threshold lowered below an in-flight packet must not wrap the
             * unsigned subtraction and effectively disable the next EOF.  The
             * next received byte closes that packet, matching the peripheral's
             * forward-only byte stream semantics. */
            size_t remaining = s->rx_payload_count < threshold
                ? threshold - s->rx_payload_count : 1u;
            if (chunk > remaining) chunk = remaining;
        }
        bool eof = ((conf0 & UHCI_CONF0_LEN_EOF_EN) && threshold != 0u &&
                    s->rx_payload_count + chunk >= threshold) ||
                   (idle_eof && accepted + chunk == len);
        size_t written = uhci_write_rx_payload(
            p, port, data + accepted, chunk, eof, true);
        accepted += written;
        s->rx_payload_count += (uint32_t)written;
        if (eof && written == chunk) s->rx_payload_count = 0u;
        if (written < chunk || chunk == 0u) break;
    }
    return accepted;
}

static size_t uhci_uart_rx_feed(esp32_periph_t *p, int uart_num,
                                const uint8_t *data, size_t len,
                                bool idle_after) {
    size_t consumed = 0u;
    for (unsigned port = 0; port < UHCI_PORT_COUNT; port++) {
        uhci_state_t *s = &p->uhci[port];
        if (!uhci_clocked(p, port) || !s->rx_link_running ||
            uhci_uart_num(s) != uart_num)
            continue;
        uint32_t conf0 = s->regs[UHCI_CONF0_OFF / 4u];
        bool framed = (conf0 & (UHCI_CONF0_SEPER_EN |
                                UHCI_CONF0_HEAD_EN)) != 0u;
        size_t port_consumed = framed ?
            uhci_feed_framed_rx(p, port, data, len) :
            uhci_feed_transparent_rx(
                p, port, data, len,
                idle_after && (conf0 & UHCI_CONF0_UART_IDLE_EOF_EN));
        if (port_consumed > consumed) consumed = port_consumed;

        if (idle_after && port_consumed != 0u &&
            (conf0 & UHCI_CONF0_UART_IDLE_EOF_EN)) {
            if (framed) {
                if (s->rx_frame_active && s->rx_frame_len != 0u)
                    (void)uhci_finish_rx_frame(p, port);
            }
            if (!framed) s->rx_payload_count = 0u;
        }
        uhci_irq_update(p, port);
    }
    return consumed;
}

static bool uhci_uart_rx_break(esp32_periph_t *p, int uart_num) {
    bool handled = false;
    for (unsigned port = 0; port < UHCI_PORT_COUNT; port++) {
        uhci_state_t *s = &p->uhci[port];
        uint32_t conf0 = s->regs[UHCI_CONF0_OFF / 4u];
        if (!uhci_clocked(p, port) || !s->rx_link_running ||
            uhci_uart_num(s) != uart_num ||
            !(conf0 & UHCI_CONF0_UART_BRK_EOF_EN))
            continue;
        bool framed = (conf0 & (UHCI_CONF0_SEPER_EN |
                                UHCI_CONF0_HEAD_EN)) != 0u;
        if (framed) {
            if (s->rx_frame_active && s->rx_frame_len != 0u)
                (void)uhci_finish_rx_frame(p, port);
        } else if (s->rx_desc_loaded && s->rx_offset != 0u) {
            (void)uhci_complete_rx_descriptor(p, port, true, true);
            s->rx_payload_count = 0u;
        }
        handled = true;
    }
    return handled;
}

static uint32_t uhci_next_fire(esp32_periph_t *p, xtensa_cpu_t *cpu) {
    if (!p || !cpu) return UINT32_MAX;
    bool have = false;
    uint64_t best = 0;
    for (unsigned port = 0; port < UHCI_PORT_COUNT; port++) {
        uhci_state_t *s = &p->uhci[port];
        uint64_t events[2] = {s->next_tx_cycle, s->next_quick_cycle};
        bool armed[2] = {s->tx_event_armed, s->quick_event_armed};
        for (unsigned index = 0; index < 2u; index++) {
            if (!armed[index] || !uhci_clocked(p, port)) continue;
            if (!have || events[index] < best) {
                have = true;
                best = events[index];
            }
        }
    }
    if (!have) return UINT32_MAX;
    return periph_deadline_ccount(p, &p->event_clock, cpu, best);
}

static void uhci_eval_events(esp32_periph_t *p, xtensa_cpu_t *cpu) {
    if (!p || !cpu) return;
    uint64_t now_cycle = periph_clock_now(p, &p->event_clock);
    for (unsigned port = 0; port < UHCI_PORT_COUNT; port++) {
        uhci_state_t *s = &p->uhci[port];
        if (!uhci_clocked(p, port)) continue;
        if (s->tx_event_armed && now_cycle >= s->next_tx_cycle) {
            s->tx_event_armed = false;
            (void)uhci_complete_tx_descriptor(p, port);
        }
        if (s->quick_event_armed && now_cycle >= s->next_quick_cycle) {
            s->quick_event_armed = false;
            uhci_complete_quick_event(p, port);
        }
    }
}

static void uhci_dport_update(esp32_periph_t *p) {
    if (!p) return;
    for (unsigned port = 0; port < UHCI_PORT_COUNT; port++) {
        uhci_state_t *s = &p->uhci[port];
        if (!uhci_clocked(p, port)) {
            s->tx_event_armed = false;
            s->quick_event_armed = false;
            periph_deassert_interrupt(p, uhci_intr_sources[port]);
            continue;
        }
        uhci_irq_update(p, port);
        if (s->tx_link_running && !s->tx_event_armed)
            uhci_arm_tx_event(p, port);
        if (!s->quick_event_armed &&
            (s->regs[UHCI_QUICK_SENT_OFF / 4u] & 0x88u))
            uhci_arm_quick_event(p, port);
    }
    uhci_kick(p);
}

static uint32_t uhci_read(void *ctx, uint32_t addr) {
    esp32_periph_t *p = ctx;
    int port = uhci_port_from_addr(addr);
    if (port < 0) return default_read(ctx, addr);
    uint32_t off = addr - uhci_bases[port];
    if ((off & 3u) != 0u || off >= UHCI_REG_FILE_SIZE)
        return default_read(ctx, addr);
    uhci_state_t *s = &p->uhci[port];
    xtensa_cpu_t *cpu = uhci_event_cpu(p);
    if (cpu) uhci_eval_events(p, cpu);
    switch (off) {
    case UHCI_INT_RAW_OFF: return s->int_raw & UHCI_INT_VALID_MASK;
    case UHCI_INT_ST_OFF: return s->int_raw & s->int_ena &
                                 UHCI_INT_VALID_MASK;
    case UHCI_INT_ENA_OFF: return s->int_ena;
    case UHCI_INT_CLR_OFF: return 0u;
    case UHCI_DMA_OUT_STATUS_OFF: return 1u << 1;
    case UHCI_DMA_IN_STATUS_OFF:
        return ((uint32_t)(s->rx_error_cause & 7u) << 4) |
               (s->debug_in_count == 0u ? (1u << 1) : 0u) |
               (s->debug_in_count >= UHCI_DEBUG_FIFO_SIZE ? 1u : 0u);
    case UHCI_DMA_IN_POP_OFF: return s->debug_in_last & 0x0FFFu;
    case UHCI_DMA_OUT_LINK_OFF:
        return (s->regs[off / 4u] & UHCI_LINK_ADDR_MASK) |
               (s->tx_link_running ? 0u : UHCI_LINK_PARK);
    case UHCI_DMA_IN_LINK_OFF:
        return (s->regs[off / 4u] &
                (UHCI_LINK_ADDR_MASK | UHCI_INLINK_AUTO_RET)) |
               (s->rx_link_running ? 0u : UHCI_LINK_PARK);
    case UHCI_STATE0_OFF:
        return (uint32_t)s->decode_state |
               ((uint32_t)(s->rx_error_cause & 7u) << 8);
    case UHCI_STATE1_OFF: return s->encode_state;
    default: return s->regs[off / 4u];
    }
}

static void uhci_write_conf0(esp32_periph_t *p, unsigned port,
                             uint32_t value) {
    uhci_state_t *s = &p->uhci[port];
    s->regs[UHCI_CONF0_OFF / 4u] = value & UHCI_CONF0_VALID_MASK;
    if (value & UHCI_CONF0_AHB_RST) {
        uhci_reset_tx_path(s);
        uhci_reset_rx_path(s);
    } else {
        if (value & UHCI_CONF0_OUT_RST) uhci_reset_tx_path(s);
        if (value & UHCI_CONF0_IN_RST) uhci_reset_rx_path(s);
    }
    if (value & UHCI_CONF0_AHB_FIFO_RST) {
        s->debug_in_head = 0u;
        s->debug_in_count = 0u;
        s->debug_in_last = 0u;
    }
    uhci_irq_update(p, port);
    if (s->tx_link_running) uhci_arm_tx_event(p, port);
    uhci_kick(p);
}

static void uhci_write(void *ctx, uint32_t addr, uint32_t value) {
    esp32_periph_t *p = ctx;
    int port_index = uhci_port_from_addr(addr);
    if (port_index < 0) { default_write(ctx, addr, value); return; }
    unsigned port = (unsigned)port_index;
    uint32_t off = addr - uhci_bases[port];
    if ((off & 3u) != 0u || off >= UHCI_REG_FILE_SIZE) {
        default_write(ctx, addr, value);
        return;
    }
    uhci_state_t *s = &p->uhci[port];
    switch (off) {
    case UHCI_INT_RAW_OFF:
    case UHCI_INT_ST_OFF:
    case UHCI_DMA_OUT_STATUS_OFF:
    case UHCI_DMA_IN_STATUS_OFF:
    case UHCI_STATE0_OFF:
    case UHCI_STATE1_OFF:
    case UHCI_OUT_EOF_DESC_OFF:
    case UHCI_IN_SUC_EOF_DESC_OFF:
    case UHCI_IN_ERR_EOF_DESC_OFF:
    case UHCI_OUT_EOF_BFR_DESC_OFF:
    case UHCI_IN_DSCR_OFF:
    case UHCI_IN_DSCR_BF0_OFF:
    case UHCI_IN_DSCR_BF1_OFF:
    case UHCI_OUT_DSCR_OFF:
    case UHCI_OUT_DSCR_BF0_OFF:
    case UHCI_OUT_DSCR_BF1_OFF:
    case UHCI_RX_HEAD_OFF:
        return;
    case UHCI_CONF0_OFF:
        uhci_write_conf0(p, port, value);
        return;
    case UHCI_INT_ENA_OFF:
        s->int_ena = value & UHCI_INT_VALID_MASK;
        uhci_irq_update(p, port);
        return;
    case UHCI_INT_CLR_OFF:
        s->int_raw &= ~(value & UHCI_INT_VALID_MASK);
        uhci_irq_update(p, port);
        return;
    case UHCI_DMA_OUT_PUSH_OFF:
        s->regs[off / 4u] = value & 0x000101FFu;
        if (value & (1u << 16)) {
            int uart_num = uhci_uart_num(s);
            if (uart_num >= 0)
                uart_emit_tx_byte(p, uart_num, (uint8_t)value, false);
            else
                s->int_raw |= UHCI_INT_TX_HUNG;
            uhci_irq_update(p, port);
        }
        return;
    case UHCI_DMA_IN_POP_OFF:
        if ((value & (1u << 16)) && s->debug_in_count != 0u) {
            s->debug_in_last = s->debug_in_fifo[s->debug_in_head];
            s->debug_in_head =
                (uint16_t)((s->debug_in_head + 1u) % UHCI_DEBUG_FIFO_SIZE);
            s->debug_in_count--;
        }
        return;
    case UHCI_DMA_OUT_LINK_OFF:
        s->regs[off / 4u] = value & UHCI_LINK_ADDR_MASK;
        if (value & UHCI_LINK_STOP) {
            s->tx_link_running = false;
            s->tx_event_armed = false;
            uhci_reset_tx_frame(s);
        } else if (value & (UHCI_LINK_START | UHCI_LINK_RESTART)) {
            s->tx_desc = uhci_first_desc(value);
            s->tx_descriptors_seen = 0u;
            s->tx_link_running = true;
            uhci_reset_tx_frame(s);
            uhci_arm_tx_event(p, port);
        }
        uhci_kick(p);
        return;
    case UHCI_DMA_IN_LINK_OFF:
        s->regs[off / 4u] = value &
            (UHCI_LINK_ADDR_MASK | UHCI_INLINK_AUTO_RET);
        if (value & UHCI_LINK_STOP) {
            s->rx_link_running = false;
            s->rx_desc_loaded = false;
            uhci_reset_rx_frame(s);
        } else if (value & (UHCI_LINK_START | UHCI_LINK_RESTART)) {
            s->rx_desc = uhci_first_desc(value);
            s->rx_descriptors_seen = 0u;
            s->rx_desc_loaded = false;
            s->rx_link_running = true;
            s->rx_error_cause = 0u;
            uhci_reset_rx_frame(s);
        }
        uhci_kick(p);
        return;
    case UHCI_CONF1_OFF:
        s->regs[off / 4u] = value & UHCI_CONF1_VALID_MASK;
        if (s->tx_link_running) uhci_arm_tx_event(p, port);
        return;
    case UHCI_AHB_TEST_OFF:
        s->regs[off / 4u] = value & 0x37u;
        return;
    case UHCI_ESCAPE_CONF_OFF:
        s->regs[off / 4u] = value & 0xFFu;
        return;
    case UHCI_HUNG_CONF_OFF:
        s->regs[off / 4u] = value & 0x00FFFFFFu;
        return;
    case UHCI_ACK_NUM_OFF:
        s->regs[off / 4u] = value & 7u;
        return;
    case UHCI_QUICK_SENT_OFF:
        s->regs[off / 4u] = value & 0xFFu;
        uhci_arm_quick_event(p, port);
        return;
    case UHCI_ESC_CONF0_OFF:
    case UHCI_ESC_CONF1_OFF:
    case UHCI_ESC_CONF2_OFF:
    case UHCI_ESC_CONF3_OFF:
        s->regs[off / 4u] = value & 0x00FFFFFFu;
        return;
    case UHCI_PKT_THRES_OFF:
        s->regs[off / 4u] = value & 0x1FFFu;
        return;
    default:
        s->regs[off / 4u] = value;
        return;
    }
}

/* ---- SDIO slave (HINF + SLCHOST + SLC DMA) ---- */

static bool sdio_slave_clocked(const esp32_periph_t *p) {
    return p &&
           (p->dport_wifi_clk_en & DPORT_SDIO_SLAVE_CLK_BIT) != 0u &&
           (p->dport_core_rst_en & DPORT_SDIO_SLAVE_RST_BIT) == 0u;
}

static uint32_t slc_first_desc(uint32_t link) {
    return 0x3FF00000u | (link & SLC_LINK_ADDR_MASK);
}

static uint32_t slc_next_desc(uint32_t next) {
    return next != 0u && next < 0x00100000u ?
           0x3FF00000u | next : next;
}

static uint32_t slc_valid_interrupts(unsigned channel) {
    return channel == 0u ? SLC_INT_VALID0_MASK : SLC_INT_VALID1_MASK;
}

static void slc_irq_update(esp32_periph_t *p, unsigned channel) {
    if (!p || channel >= SLC_CHANNEL_COUNT) return;
    sdio_slave_state_t *s = &p->sdio_slave;
    bool active = sdio_slave_clocked(p) &&
                  (s->int_raw[channel] & s->int_ena[channel] &
                   slc_valid_interrupts(channel)) != 0u;
    int source = channel == 0u ? SLC_INTR_SOURCE0 : SLC_INTR_SOURCE1;
    if (active)
        periph_assert_interrupt_status(p, source,
                                       s->int_raw[channel] &
                                       s->int_ena[channel] &
                                       slc_valid_interrupts(channel));
    else
        periph_deassert_interrupt(p, source);
}

static void slc_set_interrupts(esp32_periph_t *p, unsigned channel,
                               uint32_t bits) {
    if (!p || channel >= SLC_CHANNEL_COUNT) return;
    sdio_slave_state_t *s = &p->sdio_slave;
    s->int_raw[channel] |= bits & slc_valid_interrupts(channel);
    slc_irq_update(p, channel);
}

static void slchost_set_interrupts(sdio_slave_state_t *s, unsigned channel,
                                   uint32_t bits) {
    if (!s || channel >= SLC_CHANNEL_COUNT) return;
    s->host_int_raw[channel] |= bits & SLCHOST_INT_VALID_MASK;
}

static void slc_update_desc_history(sdio_slave_state_t *s,
                                    unsigned channel, bool rx,
                                    uint32_t desc) {
    uint32_t current;
    if (channel == 0u)
        current = rx ? SLC_RX_DSCR0_OFF : SLC_TX_DSCR0_OFF;
    else
        current = rx ? SLC_RX_DSCR1_OFF : SLC_TX_DSCR1_OFF;
    uint32_t bf0 = current + 4u;
    uint32_t bf1 = current + 8u;
    s->slc[bf1 / 4u] = s->slc[bf0 / 4u];
    s->slc[bf0 / 4u] = s->slc[current / 4u];
    s->slc[current / 4u] = desc;
}

static void slc_reset_dma_path(esp32_periph_t *p, unsigned channel,
                               bool rx) {
    if (!p || channel >= SLC_CHANNEL_COUNT) return;
    sdio_slave_state_t *s = &p->sdio_slave;
    if (rx) {
        s->rx_desc[channel] = 0u;
        s->rx_last_desc[channel] = 0u;
        s->rx_link_running[channel] = false;
        s->int_raw[channel] &= ~(SLC_INT_RX_START | SLC_INT_RX_UDF |
                                 SLC_INT_RX_DONE | SLC_INT_RX_EOF |
                                 SLC_INT_RX_DSCR_ERR);
    } else {
        s->tx_desc[channel] = 0u;
        s->tx_last_desc[channel] = 0u;
        s->tx_link_running[channel] = false;
        s->int_raw[channel] &= ~(SLC_INT_TX_START | SLC_INT_TX_OVF |
                                 SLC_INT_TX_DONE | SLC_INT_TX_SUC_EOF |
                                 SLC_INT_TX_DSCR_ERR |
                                 SLC_INT_TX_DSCR_EMPTY);
    }
    slc_irq_update(p, channel);
}

static void sdio_slave_reset_state(esp32_periph_t *p) {
    if (!p) return;
    periph_deassert_interrupt(p, SLC_INTR_SOURCE0);
    periph_deassert_interrupt(p, SLC_INTR_SOURCE1);
    sdio_slave_state_t *s = &p->sdio_slave;
    memset(s, 0, sizeof(*s));

    s->slc[SLC_CONF0_OFF / 4u] = SLC_CONF0_RESET;
    s->slc[SLC_CONF1_OFF / 4u] = SLC_CONF1_RESET;
    s->slc[SLC_RX_DSCR_CONF_OFF / 4u] = SLC_RX_DSCR_CONF_RESET;
    s->slc[SLC_DATE_OFF / 4u] = SLC_DATE_RESET;
    s->slc[SLC_ID_OFF / 4u] = SLC_ID_RESET;

    s->host[SLCHOST_DATE_OFF / 4u] = SLCHOST_DATE_RESET;
    s->host[SLCHOST_ID_OFF / 4u] = SLCHOST_ID_RESET;

    s->hinf[HINF_CFG_DATA0_OFF / 4u] = HINF_CFG_DATA0_RESET;
    s->hinf[HINF_CFG_DATA1_OFF / 4u] = HINF_CFG_DATA1_RESET;
    s->hinf[HINF_CFG_DATA7_OFF / 4u] = HINF_CFG_DATA7_RESET;
    for (uint32_t off = HINF_CIS_CONF0_OFF; off < HINF_CFG_DATA16_OFF;
         off += 4u)
        s->hinf[off / 4u] = UINT32_MAX;
    s->hinf[HINF_CFG_DATA16_OFF / 4u] = HINF_CFG_DATA16_RESET;
    s->hinf[HINF_DATE_OFF / 4u] = HINF_DATE_RESET;
}

static void sdio_slave_dport_update(esp32_periph_t *p) {
    if (!p) return;
    slc_irq_update(p, 0u);
    slc_irq_update(p, 1u);
}

static void slc_token_set(esp32_periph_t *p, unsigned channel,
                          unsigned token_index, uint16_t value) {
    if (!p || channel >= SLC_CHANNEL_COUNT || token_index > 1u) return;
    sdio_slave_state_t *s = &p->sdio_slave;
    uint16_t old = s->token[channel][token_index];
    value &= SLC_TOKEN_VALUE_MASK;
    s->token[channel][token_index] = value;
    if (old == 0u && value != 0u) {
        slchost_set_interrupts(
            s, channel, token_index == 0u ? SLCHOST_INT_TOKEN0_READY :
                                           SLCHOST_INT_TOKEN1_READY);
    } else if (old != 0u && value == 0u) {
        slchost_set_interrupts(
            s, channel, token_index == 0u ? SLCHOST_INT_TOKEN0_EMPTY :
                                           SLCHOST_INT_TOKEN1_EMPTY);
        slc_set_interrupts(
            p, channel, token_index == 0u ? SLC_INT_TOKEN0_EMPTY :
                                           SLC_INT_TOKEN1_EMPTY);
    }
}

static void slc_token_write(esp32_periph_t *p, unsigned channel,
                            unsigned token_index, uint32_t value) {
    sdio_slave_state_t *s = &p->sdio_slave;
    uint16_t next = s->token[channel][token_index];
    if (value & SLC_TOKEN_WR) {
        next = (uint16_t)(value & SLC_TOKEN_VALUE_MASK);
    } else if (value & SLC_TOKEN_INC_MORE) {
        next = (uint16_t)((next + (value & SLC_TOKEN_VALUE_MASK)) &
                          SLC_TOKEN_VALUE_MASK);
    } else if (value & SLC_TOKEN_INC) {
        next = (uint16_t)((next + 1u) & SLC_TOKEN_VALUE_MASK);
    }
    slc_token_set(p, channel, token_index, next);
}

static void slc_packet_length_set(sdio_slave_state_t *s, unsigned channel,
                                  uint32_t value) {
    if (!s || channel >= SLC_CHANNEL_COUNT) return;
    value &= SLC_LEN_VALUE_MASK;
    uint32_t old = s->packet_len[channel];
    s->packet_len[channel] = value;
    if (channel == 0u)
        s->slc[SLC_LENGTH0_OFF / 4u] = value;
    if (old != value)
        slchost_set_interrupts(s, channel, SLCHOST_INT_RX_NEW_PACKET);
}

static void slc_packet_length_write(sdio_slave_state_t *s,
                                    unsigned channel, uint32_t value) {
    uint32_t next = s->packet_len[channel];
    if (value & SLC_LEN_WR) {
        next = value & SLC_LEN_VALUE_MASK;
    } else if (value & SLC_LEN_INC_MORE) {
        next = (next + (value & SLC_LEN_VALUE_MASK)) & SLC_LEN_VALUE_MASK;
    } else if (value & SLC_LEN_INC) {
        next = (next + 1u) & SLC_LEN_VALUE_MASK;
    }
    slc_packet_length_set(s, channel, next);
}

static bool slc_descriptor_valid(esp32_periph_t *p, uint32_t desc,
                                 uint32_t *ctrl, uint32_t *buffer,
                                 uint32_t *next) {
    if ((desc & 3u) != 0u ||
        !uhci_dma_range_mapped(p, desc, 12u, false)) return false;
    *ctrl = mem_read32(p->mem, desc);
    *buffer = mem_read32(p->mem, desc + 4u);
    *next = slc_next_desc(mem_read32(p->mem, desc + 8u));
    return true;
}

static void slc_link_start(esp32_periph_t *p, unsigned channel, bool rx,
                           uint32_t value) {
    sdio_slave_state_t *s = &p->sdio_slave;
    uint32_t *current = rx ? &s->rx_desc[channel] : &s->tx_desc[channel];
    uint32_t *last = rx ? &s->rx_last_desc[channel] :
                          &s->tx_last_desc[channel];
    bool *running = rx ? &s->rx_link_running[channel] :
                         &s->tx_link_running[channel];

    if (value & SLC_LINK_STOP) {
        *running = false;
        return;
    }
    if (value & SLC_LINK_START) {
        *current = slc_first_desc(value);
        *last = 0u;
        *running = true;
    } else if (value & SLC_LINK_RESTART) {
        if (*current == 0u && *last != 0u &&
            uhci_dma_range_mapped(p, *last + 8u, 4u, false))
            *current = slc_next_desc(mem_read32(p->mem, *last + 8u));
        *running = *current != 0u;
    } else {
        return;
    }

    if (*current == 0u || (*current & 3u) != 0u ||
        !uhci_dma_range_mapped(p, *current, 12u, false)) {
        *running = false;
        slc_set_interrupts(p, channel,
                           rx ? SLC_INT_RX_DSCR_ERR : SLC_INT_TX_DSCR_ERR);
        return;
    }
    slc_update_desc_history(s, channel, rx, *current);
    if (rx) {
        /* Loading an RX descriptor into the SLC FIFO raises RX_DONE before
         * the SDIO host drains it. The stock HAL intentionally keeps this
         * raw latch high and toggles its enable bit as a software ISR doorbell. */
        slc_set_interrupts(p, channel, SLC_INT_RX_START | SLC_INT_RX_DONE);
    } else {
        slc_set_interrupts(p, channel, SLC_INT_TX_START);
    }
}

static uint32_t slc_read(void *ctx, uint32_t addr) {
    esp32_periph_t *p = ctx;
    uint32_t off = addr - SLC_BASE;
    if ((off & 3u) != 0u || off >= SLC_REG_FILE_SIZE)
        return default_read(ctx, addr);
    sdio_slave_state_t *s = &p->sdio_slave;
    switch (off) {
    case SLC_INT0_RAW_OFF: return s->int_raw[0];
    case SLC_INT0_ST_OFF: return s->int_raw[0] & s->int_ena[0];
    case SLC_INT0_ENA_OFF: return s->int_ena[0];
    case SLC_INT0_CLR_OFF: return 0u;
    case SLC_INT1_RAW_OFF: return s->int_raw[1];
    case SLC_INT1_ST_OFF: return s->int_raw[1] & s->int_ena[1];
    case SLC_INT1_ENA_OFF: return s->int_ena[1];
    case SLC_INT1_CLR_OFF: return 0u;
    case SLC_RX_LINK0_OFF:
        return (s->slc[off / 4u] & SLC_LINK_ADDR_MASK) |
               (s->rx_link_running[0] ? 0u : SLC_LINK_PARK);
    case SLC_TX_LINK0_OFF:
        return (s->slc[off / 4u] & SLC_LINK_ADDR_MASK) |
               (s->tx_link_running[0] ? 0u : SLC_LINK_PARK);
    case SLC_RX_LINK1_OFF:
        return (s->slc[off / 4u] & SLC_LINK_ADDR_MASK) |
               (s->rx_link_running[1] ? 0u : SLC_LINK_PARK);
    case SLC_TX_LINK1_OFF:
        return (s->slc[off / 4u] & SLC_LINK_ADDR_MASK) |
               (s->tx_link_running[1] ? 0u : SLC_LINK_PARK);
    case SLC_TOKEN0_0_OFF:
        return (uint32_t)s->token[0][0] << SLC_TOKEN_READ_SHIFT;
    case SLC_TOKEN1_0_OFF:
        return (uint32_t)s->token[0][1] << SLC_TOKEN_READ_SHIFT;
    case SLC_TOKEN0_1_OFF:
        return (uint32_t)s->token[1][0] << SLC_TOKEN_READ_SHIFT;
    case SLC_TOKEN1_1_OFF:
        return (uint32_t)s->token[1][1] << SLC_TOKEN_READ_SHIFT;
    case SLC_LENGTH0_OFF: return s->packet_len[0];
    default: return s->slc[off / 4u];
    }
}

static void slc_write(void *ctx, uint32_t addr, uint32_t value) {
    esp32_periph_t *p = ctx;
    uint32_t off = addr - SLC_BASE;
    if ((off & 3u) != 0u || off >= SLC_REG_FILE_SIZE) {
        default_write(ctx, addr, value);
        return;
    }
    sdio_slave_state_t *s = &p->sdio_slave;
    switch (off) {
    case SLC_CONF0_OFF:
        s->slc[off / 4u] = value;
        if (value & ((1u << 2) | (1u << 3))) {
            for (unsigned channel = 0; channel < SLC_CHANNEL_COUNT;
                 channel++) {
                slc_reset_dma_path(p, channel, false);
                slc_reset_dma_path(p, channel, true);
            }
        }
        if (value & (1u << 0)) slc_reset_dma_path(p, 0u, false);
        if (value & (1u << 1)) slc_reset_dma_path(p, 0u, true);
        if (value & (1u << 16)) slc_reset_dma_path(p, 1u, false);
        if (value & (1u << 17)) slc_reset_dma_path(p, 1u, true);
        return;
    case SLC_INT0_RAW_OFF:
    case SLC_INT0_ST_OFF:
    case SLC_INT1_RAW_OFF:
    case SLC_INT1_ST_OFF:
        return;
    case SLC_INT0_ENA_OFF:
        s->int_ena[0] = value & SLC_INT_VALID0_MASK;
        slc_irq_update(p, 0u);
        return;
    case SLC_INT1_ENA_OFF:
        s->int_ena[1] = value & SLC_INT_VALID1_MASK;
        slc_irq_update(p, 1u);
        return;
    case SLC_INT0_CLR_OFF:
        s->int_raw[0] &= ~(value & SLC_INT_VALID0_MASK);
        slc_irq_update(p, 0u);
        return;
    case SLC_INT1_CLR_OFF:
        s->int_raw[1] &= ~(value & SLC_INT_VALID1_MASK);
        slc_irq_update(p, 1u);
        return;
    case SLC_RX_LINK0_OFF:
    case SLC_TX_LINK0_OFF:
    case SLC_RX_LINK1_OFF:
    case SLC_TX_LINK1_OFF: {
        unsigned channel = off >= SLC_RX_LINK1_OFF ? 1u : 0u;
        bool rx = off == SLC_RX_LINK0_OFF || off == SLC_RX_LINK1_OFF;
        s->slc[off / 4u] = value & SLC_LINK_ADDR_MASK;
        slc_link_start(p, channel, rx, value);
        return;
    }
    case SLC_INTVEC_TOHOST_OFF:
        s->slc[off / 4u] = value & 0x00FF00FFu;
        if (value & 0xFFu) {
            slchost_set_interrupts(s, 0u, value & 0xFFu);
            slc_set_interrupts(p, 0u, SLC_INT_TOHOST);
        }
        if (value & 0x00FF0000u) {
            slchost_set_interrupts(s, 1u, (value >> 16u) & 0xFFu);
            slc_set_interrupts(p, 1u, SLC_INT_TOHOST);
        }
        return;
    case SLC_TOKEN0_0_OFF: slc_token_write(p, 0u, 0u, value); return;
    case SLC_TOKEN1_0_OFF: slc_token_write(p, 0u, 1u, value); return;
    case SLC_TOKEN0_1_OFF: slc_token_write(p, 1u, 0u, value); return;
    case SLC_TOKEN1_1_OFF: slc_token_write(p, 1u, 1u, value); return;
    case SLC_LEN_CONF0_OFF:
        s->slc[off / 4u] = value;
        slc_packet_length_write(s, 0u, value);
        return;
    case SLC_LENGTH0_OFF:
    case SLC_TO_EOF_DESC0_OFF:
    case SLC_TX_EOF_DESC0_OFF:
    case SLC_TO_EOF_BFR_DESC0_OFF:
    case SLC_TO_EOF_DESC1_OFF:
    case SLC_TX_EOF_DESC1_OFF:
    case SLC_TO_EOF_BFR_DESC1_OFF:
    case SLC_TX_DSCR0_OFF:
    case SLC_TX_DSCR0_BF0_OFF:
    case SLC_TX_DSCR0_BF1_OFF:
    case SLC_RX_DSCR0_OFF:
    case SLC_RX_DSCR0_BF0_OFF:
    case SLC_RX_DSCR0_BF1_OFF:
    case SLC_TX_DSCR1_OFF:
    case SLC_TX_DSCR1_BF0_OFF:
    case SLC_TX_DSCR1_BF1_OFF:
    case SLC_RX_DSCR1_OFF:
    case SLC_RX_DSCR1_BF0_OFF:
    case SLC_RX_DSCR1_BF1_OFF:
    case SLC_TX_ERR_EOF_DESC0_OFF:
    case SLC_TX_ERR_EOF_DESC1_OFF:
        return;
    default:
        s->slc[off / 4u] = value;
        return;
    }
}

static uint32_t slchost_token_value(const sdio_slave_state_t *s,
                                    unsigned channel) {
    return (uint32_t)s->token[channel][0] |
           ((uint32_t)s->token[channel][1] << 16u);
}

static uint32_t slchost_read(void *ctx, uint32_t addr) {
    esp32_periph_t *p = ctx;
    uint32_t off = addr - SLCHOST_BASE;
    if (off >= SLCHOST_REG_FILE_SIZE)
        return default_read(ctx, addr);
    /* The public SDIO API intentionally reads shared registers through an
     * 8-bit volatile pointer. MMIO callbacks receive that exact byte address,
     * so select it from the containing hardware register here. */
    if ((off & 3u) != 0u) {
        uint32_t aligned = off & ~3u;
        uint32_t word = slchost_read(ctx, SLCHOST_BASE + aligned);
        return (word >> ((off & 3u) * 8u)) & 0xFFu;
    }
    sdio_slave_state_t *s = &p->sdio_slave;
    switch (off) {
    case SLCHOST_TOKEN_RDATA0_OFF: return slchost_token_value(s, 0u);
    case SLCHOST_TOKEN_RDATA1_OFF: return slchost_token_value(s, 1u);
    case SLCHOST_INT0_RAW_OFF: return s->host_int_raw[0];
    case SLCHOST_INT1_RAW_OFF: return s->host_int_raw[1];
    case SLCHOST_INT0_ST_OFF:
        return s->host_int_raw[0] & s->host[SLCHOST_INT0_ENA_OFF / 4u];
    case SLCHOST_INT1_ST_OFF:
        return s->host_int_raw[1] & s->host[SLCHOST_INT1_ENA_OFF / 4u];
    case SLCHOST_PKT_LEN_OFF: return s->packet_len[0];
    case SLCHOST_INT0_CLR_OFF:
    case SLCHOST_INT1_CLR_OFF:
    case SLCHOST_TOKEN_CON_OFF:
        return 0u;
    default: return s->host[off / 4u];
    }
}

static void slchost_apply_token_control(esp32_periph_t *p, uint32_t value) {
    sdio_slave_state_t *s = &p->sdio_slave;
    for (unsigned channel = 0; channel < SLC_CHANNEL_COUNT; channel++) {
        uint32_t wdata = s->host[(channel == 0u ?
                                 SLCHOST_TOKEN_WDATA0_OFF :
                                 SLCHOST_TOKEN_WDATA1_OFF) / 4u];
        unsigned shift = channel * 4u;
        if (value & (1u << shift))
            slc_token_set(p, channel, 0u,
                          (uint16_t)((s->token[channel][0] - 1u) &
                                     SLC_TOKEN_VALUE_MASK));
        if (value & (2u << shift))
            slc_token_set(p, channel, 1u,
                          (uint16_t)((s->token[channel][1] - 1u) &
                                     SLC_TOKEN_VALUE_MASK));
        if (value & (4u << shift))
            slc_token_set(p, channel, 0u,
                          (uint16_t)(wdata & SLC_TOKEN_VALUE_MASK));
        if (value & (8u << shift))
            slc_token_set(p, channel, 1u,
                          (uint16_t)((wdata >> 16u) & SLC_TOKEN_VALUE_MASK));
    }
    if (value & (1u << 8u))
        slc_packet_length_set(
            s, 0u, s->host[SLCHOST_LEN_WDATA0_OFF / 4u]);
}

static void slchost_write(void *ctx, uint32_t addr, uint32_t value) {
    esp32_periph_t *p = ctx;
    uint32_t off = addr - SLCHOST_BASE;
    if ((off & 3u) != 0u || off >= SLCHOST_REG_FILE_SIZE) {
        default_write(ctx, addr, value);
        return;
    }
    sdio_slave_state_t *s = &p->sdio_slave;
    switch (off) {
    case SLCHOST_TOKEN_RDATA0_OFF:
    case SLCHOST_TOKEN_RDATA1_OFF:
    case SLCHOST_INT0_RAW_OFF:
    case SLCHOST_INT1_RAW_OFF:
    case SLCHOST_INT0_ST_OFF:
    case SLCHOST_INT1_ST_OFF:
    case SLCHOST_PKT_LEN_OFF:
        return;
    case SLCHOST_INT0_CLR_OFF:
        s->host_int_raw[0] &= ~(value & SLCHOST_INT_VALID_MASK);
        return;
    case SLCHOST_INT1_CLR_OFF:
        s->host_int_raw[1] &= ~(value & SLCHOST_INT_VALID_MASK);
        return;
    case SLCHOST_TOKEN_CON_OFF:
        slchost_apply_token_control(p, value);
        return;
    case SLCHOST_FUNC1_INT0_ENA_OFF:
    case SLCHOST_FUNC1_INT1_ENA_OFF:
    case SLCHOST_FUNC2_INT0_ENA_OFF:
    case SLCHOST_FUNC2_INT1_ENA_OFF:
    case SLCHOST_INT0_ENA_OFF:
    case SLCHOST_INT1_ENA_OFF:
        s->host[off / 4u] = value & SLCHOST_INT_VALID_MASK;
        return;
    default:
        s->host[off / 4u] = value;
        return;
    }
}

static uint32_t hinf_read(void *ctx, uint32_t addr) {
    esp32_periph_t *p = ctx;
    uint32_t off = addr - HINF_BASE;
    if ((off & 3u) != 0u || off >= HINF_REG_FILE_SIZE)
        return default_read(ctx, addr);
    return p->sdio_slave.hinf[off / 4u];
}

static void hinf_write(void *ctx, uint32_t addr, uint32_t value) {
    esp32_periph_t *p = ctx;
    uint32_t off = addr - HINF_BASE;
    if ((off & 3u) != 0u || off >= HINF_REG_FILE_SIZE) {
        default_write(ctx, addr, value);
        return;
    }
    if (off == HINF_CFG_DATA7_OFF && (value & HINF_SDIO_RESET)) {
        sdio_slave_reset_state(p);
        return;
    }
    p->sdio_slave.hinf[off / 4u] = value;
}

static int slchost_shared_offset(unsigned position, uint32_t *off,
                                 unsigned *shift) {
    if (position >= 64u || !off || !shift) return -1;
    uint32_t byte_off = SLCHOST_SHARED0_OFF + position;
    if (position > 23u) byte_off += 4u;
    if (position > 31u) byte_off += 12u;
    *off = byte_off & ~3u;
    *shift = (byte_off & 3u) * 8u;
    return 0;
}

bool periph_sdio_slave_host_ready(const esp32_periph_t *p) {
    if (!sdio_slave_clocked(p)) return false;
    uint32_t cfg = p->sdio_slave.hinf[HINF_CFG_DATA1_OFF / 4u];
    return (cfg & (HINF_SDIO_ENABLE | HINF_SDIO_IOREADY1)) ==
           (HINF_SDIO_ENABLE | HINF_SDIO_IOREADY1);
}

uint16_t periph_sdio_slave_host_send_buffers(const esp32_periph_t *p) {
    return p ? p->sdio_slave.token[0][1] : 0u;
}

uint32_t periph_sdio_slave_host_receive_bytes(const esp32_periph_t *p) {
    if (!p) return 0u;
    const sdio_slave_state_t *s = &p->sdio_slave;
    return (s->packet_len[0] - s->host_consumed_len[0]) &
           SLC_LEN_VALUE_MASK;
}

int periph_sdio_slave_host_read_packet(esp32_periph_t *p, uint8_t *data,
                                       size_t capacity, size_t *out_len) {
    if (out_len) *out_len = 0u;
    if (!p || !out_len) return -1;
    sdio_slave_state_t *s = &p->sdio_slave;
    uint32_t available = periph_sdio_slave_host_receive_bytes(p);
    if (available == 0u) return 0;
    *out_len = available;
    if (!data || capacity < available || !periph_sdio_slave_host_ready(p) ||
        !s->rx_link_running[0] || s->rx_desc[0] == 0u)
        return -1;

    uint32_t descriptors[SLC_DMA_MAX_DESCRIPTORS];
    uint16_t lengths[SLC_DMA_MAX_DESCRIPTORS];
    unsigned count = 0u;
    size_t remaining = available;
    uint32_t desc = s->rx_desc[0];
    uint32_t next = 0u;
    while (remaining != 0u && count < SLC_DMA_MAX_DESCRIPTORS) {
        uint32_t ctrl, buffer;
        if (!slc_descriptor_valid(p, desc, &ctrl, &buffer, &next) ||
            !(ctrl & SLC_DESC_OWNER))
            break;
        size_t size = ctrl & SLC_DESC_SIZE_MASK;
        size_t length = (ctrl & SLC_DESC_LENGTH_MASK) >>
                        SLC_DESC_LENGTH_SHIFT;
        if (length == 0u || length > size || length > remaining ||
            !uhci_dma_range_mapped(p, buffer, length, false))
            break;
        descriptors[count] = desc;
        lengths[count] = (uint16_t)length;
        count++;
        remaining -= length;
        desc = next;
    }
    if (remaining != 0u) {
        slc_set_interrupts(p, 0u, SLC_INT_RX_DSCR_ERR);
        return -1;
    }

    size_t copied = 0u;
    for (unsigned index = 0; index < count; index++) {
        uint32_t item = descriptors[index];
        uint32_t ctrl = mem_read32(p->mem, item);
        uint32_t buffer = mem_read32(p->mem, item + 4u);
        for (size_t byte = 0; byte < lengths[index]; byte++)
            data[copied++] = mem_read8(p->mem, buffer + (uint32_t)byte);
        if (s->slc[SLC_CONF0_OFF / 4u] & (1u << 6u))
            mem_write32(p->mem, item, ctrl & ~SLC_DESC_OWNER);
        slc_update_desc_history(s, 0u, true, item);
        s->rx_last_desc[0] = item;
    }
    s->rx_desc[0] = next;
    if (next == 0u) s->rx_link_running[0] = false;
    s->slc[SLC_TO_EOF_BFR_DESC0_OFF / 4u] =
        count > 1u ? descriptors[count - 2u] : 0u;
    s->slc[SLC_TO_EOF_DESC0_OFF / 4u] = descriptors[count - 1u];
    s->host_consumed_len[0] = s->packet_len[0];
    slchost_set_interrupts(s, 0u,
                           SLCHOST_INT_RX_START | SLCHOST_INT_RX_EOF);
    slc_set_interrupts(p, 0u, SLC_INT_RX_EOF);
    *out_len = copied;
    return 1;
}

int periph_sdio_slave_host_write_packet(esp32_periph_t *p,
                                        const uint8_t *data, size_t len) {
    if (!p || !data || len == 0u || !periph_sdio_slave_host_ready(p))
        return -1;
    sdio_slave_state_t *s = &p->sdio_slave;
    if (!s->tx_link_running[0] || s->tx_desc[0] == 0u ||
        s->token[0][1] == 0u)
        return 0;

    uint32_t descriptors[SLC_DMA_MAX_DESCRIPTORS];
    uint32_t buffers[SLC_DMA_MAX_DESCRIPTORS];
    uint16_t lengths[SLC_DMA_MAX_DESCRIPTORS];
    unsigned count = 0u;
    size_t remaining = len;
    uint32_t desc = s->tx_desc[0];
    uint32_t next = 0u;
    while (remaining != 0u && count < SLC_DMA_MAX_DESCRIPTORS &&
           count < s->token[0][1]) {
        uint32_t ctrl, buffer;
        if (!slc_descriptor_valid(p, desc, &ctrl, &buffer, &next) ||
            !(ctrl & SLC_DESC_OWNER))
            break;
        size_t size = ctrl & SLC_DESC_SIZE_MASK;
        size_t chunk = remaining < size ? remaining : size;
        if (size == 0u || !uhci_dma_range_mapped(p, buffer, chunk, true))
            break;
        descriptors[count] = desc;
        buffers[count] = buffer;
        lengths[count] = (uint16_t)chunk;
        count++;
        remaining -= chunk;
        desc = next;
    }
    if (remaining != 0u) {
        slchost_set_interrupts(s, 0u, SLCHOST_INT_TX_OVF);
        slc_set_interrupts(p, 0u, SLC_INT_TX_OVF);
        return 0;
    }

    size_t consumed = 0u;
    for (unsigned index = 0; index < count; index++) {
        uint32_t item = descriptors[index];
        uint32_t ctrl = mem_read32(p->mem, item);
        for (size_t byte = 0; byte < lengths[index]; byte++)
            mem_write8(p->mem, buffers[index] + (uint32_t)byte,
                       data[consumed++]);
        ctrl &= ~(SLC_DESC_LENGTH_MASK | SLC_DESC_EOF | SLC_DESC_OWNER);
        ctrl |= (uint32_t)lengths[index] << SLC_DESC_LENGTH_SHIFT;
        if (index + 1u == count) ctrl |= SLC_DESC_EOF;
        mem_write32(p->mem, item, ctrl);
        slc_update_desc_history(s, 0u, false, item);
        s->tx_last_desc[0] = item;
        slc_token_set(p, 0u, 1u,
                      (uint16_t)((s->token[0][1] - 1u) &
                                 SLC_TOKEN_VALUE_MASK));
    }
    s->tx_desc[0] = next;
    if (next == 0u) s->tx_link_running[0] = false;
    s->slc[SLC_TO_EOF_BFR_DESC0_OFF / 4u] =
        count > 1u ? descriptors[count - 2u] : 0u;
    s->slc[SLC_TX_EOF_DESC0_OFF / 4u] = descriptors[count - 1u];
    slchost_set_interrupts(s, 0u, SLCHOST_INT_TX_START);
    slc_set_interrupts(p, 0u, SLC_INT_TX_DONE | SLC_INT_TX_SUC_EOF);
    return 1;
}

int periph_sdio_slave_host_read_reg(const esp32_periph_t *p,
                                    unsigned position, uint8_t *value) {
    uint32_t off;
    unsigned shift;
    if (!p || !value || slchost_shared_offset(position, &off, &shift) != 0)
        return -1;
    *value = (uint8_t)(p->sdio_slave.host[off / 4u] >> shift);
    return 0;
}

int periph_sdio_slave_host_write_reg(esp32_periph_t *p, unsigned position,
                                     uint8_t value) {
    uint32_t off;
    unsigned shift;
    if (!p || slchost_shared_offset(position, &off, &shift) != 0)
        return -1;
    uint32_t mask = 0xFFu << shift;
    p->sdio_slave.host[off / 4u] =
        (p->sdio_slave.host[off / 4u] & ~mask) |
        ((uint32_t)value << shift);
    return 0;
}

int periph_sdio_slave_host_interrupt(esp32_periph_t *p, uint8_t mask) {
    if (!p || mask == 0u || !sdio_slave_clocked(p)) return -1;
    slc_set_interrupts(p, 0u, mask);
    return 0;
}

uint32_t periph_sdio_slave_host_interrupt_raw(const esp32_periph_t *p) {
    return p ? p->sdio_slave.host_int_raw[0] : 0u;
}

uint32_t periph_sdio_slave_host_interrupt_pending(const esp32_periph_t *p) {
    if (!p) return 0u;
    const sdio_slave_state_t *s = &p->sdio_slave;
    if (s->hinf[HINF_CFG_DATA1_OFF / 4u] & HINF_SDIO_INT_MASK) return 0u;
    return s->host_int_raw[0] &
           s->host[SLCHOST_FUNC1_INT0_ENA_OFF / 4u];
}

void periph_sdio_slave_host_interrupt_clear(esp32_periph_t *p,
                                            uint32_t mask) {
    if (!p) return;
    p->sdio_slave.host_int_raw[0] &= ~(mask & SLCHOST_INT_VALID_MASK);
}

/* ---- Native SDMMC host + FIFO/IDMAC ---- */

static bool sdmmc_clocked(const esp32_periph_t *p) {
    return (p->dport_wifi_clk_en & DPORT_SDIO_HOST_CLK_BIT) != 0u &&
           (p->dport_core_rst_en & DPORT_SDIO_HOST_RST_BIT) == 0u;
}

static xtensa_cpu_t *sdmmc_event_cpu(esp32_periph_t *p) {
    return p->cpu[0] ? p->cpu[0] : p->cpu[1];
}

static void sdmmc_kick(esp32_periph_t *p) {
    periph_event_source_changed(p, PERIPH_EVENT_SDMMC);
    for (unsigned core = 0; core < 2u; core++)
        if (p->cpu[core]) xtensa_recompute_next_timer(p->cpu[core]);
}

static void sdmmc_irq_update(esp32_periph_t *p) {
    sdmmc_state_t *s = &p->sdmmc;
    uint32_t ctrl = s->regs[SDMMC_CTRL_OFF / 4u];
    uint32_t normal = s->rintsts & s->regs[SDMMC_INTMASK_OFF / 4u];
    uint32_t dma = s->idsts & s->regs[SDMMC_IDINTEN_OFF / 4u];
    bool active = sdmmc_clocked(p) && (ctrl & SDMMC_CTRL_INT_ENABLE) &&
                  (normal != 0u || dma != 0u);
    if (active)
        periph_assert_interrupt_status(p, SDMMC_INTR_SOURCE, normal | dma);
    else
        periph_deassert_interrupt(p, SDMMC_INTR_SOURCE);
}

static void sdmmc_cancel_transfer(sdmmc_state_t *s) {
    s->transfer_total = 0u;
    s->transfer_pos = 0u;
    s->transfer_lba = 0u;
    s->transfer_desc = 0u;
    s->transfer_descriptors_seen = 0u;
    s->transfer_slot = 0u;
    s->transfer_cmd = 0u;
    s->transfer_write = false;
    s->transfer_block_io = false;
    s->transfer_auto_stop = false;
    s->transfer_dma = false;
    s->transfer_active = false;
    s->transfer_wait_command_ack = false;
    s->transfer_event_armed = false;
    s->next_transfer_ccount = 0u;
}

static void sdmmc_reset_state(esp32_periph_t *p) {
    if (!p) return;
    periph_deassert_interrupt(p, SDMMC_INTR_SOURCE);
    sdmmc_state_t *s = &p->sdmmc;
    sdmmc_card_state_t cards[SDMMC_SLOT_COUNT];
    memcpy(cards, s->card, sizeof(cards));
    uint8_t *transfer = s->transfer;
    size_t transfer_capacity = s->transfer_capacity;
    memset(s, 0, sizeof(*s));
    memcpy(s->card, cards, sizeof(cards));
    s->transfer = transfer;
    s->transfer_capacity = transfer_capacity;
    s->regs[SDMMC_TMOUT_OFF / 4u] = 0xFFFFFFFFu;
    s->regs[SDMMC_BLKSIZ_OFF / 4u] = 0x200u;
    s->regs[SDMMC_BYTCNT_OFF / 4u] = 0x200u;
    s->regs[SDMMC_FIFOTH_OFF / 4u] = 0x00070008u;
    s->regs[SDMMC_DEBNCE_OFF / 4u] = 0x00FFFFFFu;
    s->regs[SDMMC_VERID_OFF / 4u] = SDMMC_VERID_RESET;
    for (unsigned slot = 0; slot < SDMMC_SLOT_COUNT; slot++) {
        s->card[slot].ready = false;
        s->card[slot].selected = false;
        s->card[slot].app_cmd = false;
        s->card[slot].rca = (uint16_t)(slot + 1u);
        s->card[slot].block_len = 512u;
    }
}

static void sdmmc_dport_update(esp32_periph_t *p) {
    if (!p) return;
    if (!sdmmc_clocked(p)) {
        p->sdmmc.transfer_event_armed = false;
        periph_deassert_interrupt(p, SDMMC_INTR_SOURCE);
    } else {
        sdmmc_irq_update(p);
    }
    sdmmc_kick(p);
}

static uint32_t sdmmc_card_detect(const sdmmc_state_t *s) {
    uint32_t value = 0u;
    for (unsigned slot = 0; slot < SDMMC_SLOT_COUNT; slot++)
        if (!s->card[slot].attached) value |= 1u << slot;
    return value;
}

static uint32_t sdmmc_write_protect(const sdmmc_state_t *s) {
    uint32_t value = 0u;
    for (unsigned slot = 0; slot < SDMMC_SLOT_COUNT; slot++)
        if (s->card[slot].write_protected ||
            (s->card[slot].attached && !s->card[slot].write_fn))
            value |= 1u << slot;
    return value;
}

static uint32_t sdmmc_status(const sdmmc_state_t *s) {
    uint32_t value = 0u;
    if (s->fifo_count == 0u) value |= SDMMC_STATUS_FIFO_EMPTY;
    if (s->fifo_count >= SDMMC_FIFO_WORDS) value |= SDMMC_STATUS_FIFO_FULL;
    if (s->card[0].attached || s->card[1].attached)
        value |= SDMMC_STATUS_CARD_PRESENT;
    if (s->transfer_active)
        value |= SDMMC_STATUS_DATA_BUSY | SDMMC_STATUS_DATA_FSM_BUSY;
    value |= ((uint32_t)s->transfer_cmd & 0x3Fu) <<
             SDMMC_STATUS_RESP_SHIFT;
    value |= ((uint32_t)s->fifo_count & 0x1FFFu) <<
             SDMMC_STATUS_FIFO_SHIFT;
    uint32_t fifoth = s->regs[SDMMC_FIFOTH_OFF / 4u];
    uint32_t tx_watermark = fifoth & 0xFFFu;
    uint32_t rx_watermark = (fifoth >> 16) & 0xFFFu;
    if (s->fifo_count <= tx_watermark) value |= 1u << 1;
    if (s->fifo_count > rx_watermark) value |= 1u;
    return value;
}

static void sdmmc_resp_set_bits(uint32_t response[4], unsigned start,
                                unsigned len, uint32_t value) {
    if (len == 0u || len > 32u || start >= 128u || start + len > 128u)
        return;
    uint32_t mask = len == 32u ? UINT32_MAX : (1u << len) - 1u;
    value &= mask;
    unsigned word = start / 32u;
    unsigned shift = start % 32u;
    response[word] &= ~(mask << shift);
    response[word] |= value << shift;
    if (shift + len > 32u) {
        unsigned high_len = shift + len - 32u;
        uint32_t high_mask = high_len == 32u ? UINT32_MAX :
                             (1u << high_len) - 1u;
        response[word + 1u] &= ~high_mask;
        response[word + 1u] |= value >> (32u - shift);
    }
}

static void sdmmc_set_response(sdmmc_state_t *s,
                               const uint32_t response[4]) {
    for (unsigned index = 0; index < 4u; index++)
        s->regs[(SDMMC_RESP0_OFF / 4u) + index] = response[index];
}

static uint32_t sdmmc_r1_status(const sdmmc_card_state_t *card,
                                bool app_cmd) {
    uint32_t state = card->selected ? 4u : card->ready ? 3u : 0u;
    return (state << 9) | (card->ready ? 1u << 8 : 0u) |
           (app_cmd ? 1u << 5 : 0u);
}

static void sdmmc_csd_response(const sdmmc_card_state_t *card,
                               uint32_t response[4]) {
    memset(response, 0, 4u * sizeof(uint32_t));
    uint32_t sectors = card->sector_count < 1024u ? 1024u :
                       card->sector_count;
    uint32_t c_size = sectors / 1024u - 1u;
    if (c_size > 0x3FFFFFu) c_size = 0x3FFFFFu;
    sdmmc_resp_set_bits(response, 126u, 2u, 1u);     /* CSD v2 */
    sdmmc_resp_set_bits(response, 112u, 8u, 0x0Eu); /* TAAC */
    sdmmc_resp_set_bits(response, 96u, 8u, 0x32u);  /* 25 MHz */
    sdmmc_resp_set_bits(response, 84u, 12u, 0x5B5u);
    sdmmc_resp_set_bits(response, 80u, 4u, 9u);     /* 512-byte block */
    sdmmc_resp_set_bits(response, 48u, 22u, c_size);
    sdmmc_resp_set_bits(response, 46u, 1u, 1u);     /* sector erase */
    sdmmc_resp_set_bits(response, 39u, 7u, 0x7Fu);
    sdmmc_resp_set_bits(response, 22u, 4u, 9u);
}

static void sdmmc_cid_response(unsigned slot, uint32_t response[4]) {
    static const char name[5] = {'F', 'L', 'E', 'X', 'E'};
    memset(response, 0, 4u * sizeof(uint32_t));
    sdmmc_resp_set_bits(response, 120u, 8u, 0xF1u);
    sdmmc_resp_set_bits(response, 104u, 16u, 0x4658u); /* FX */
    for (unsigned index = 0; index < 5u; index++)
        sdmmc_resp_set_bits(response, 96u - index * 8u, 8u,
                            (uint8_t)name[index]);
    sdmmc_resp_set_bits(response, 56u, 8u, 0x10u);
    sdmmc_resp_set_bits(response, 24u, 32u, 0xF1E00000u | slot);
    sdmmc_resp_set_bits(response, 8u, 12u, 0x1A8u);
}

static bool sdmmc_ensure_transfer(sdmmc_state_t *s, size_t size) {
    if (size > SDMMC_TRANSFER_MAX) return false;
    if (size <= s->transfer_capacity) return true;
    uint8_t *next = realloc(s->transfer, size);
    if (!next) return false;
    s->transfer = next;
    s->transfer_capacity = size;
    return true;
}

static bool sdmmc_card_range_valid(const sdmmc_card_state_t *card,
                                    uint32_t lba, size_t count) {
    return card->attached && count <= card->sector_count &&
           lba <= card->sector_count - count;
}

static void sdmmc_fill_scr(uint8_t *data, size_t len) {
    uint64_t scr = (uint64_t)2u << 56 | (uint64_t)5u << 48 |
                   (uint64_t)1u << 47 | (uint64_t)1u << 33;
    memset(data, 0, len);
    for (unsigned index = 0; index < 8u && index < len; index++)
        data[index] = (uint8_t)(scr >> (56u - index * 8u));
}

static bool sdmmc_prepare_transfer(esp32_periph_t *p, unsigned slot,
                                    unsigned command, uint32_t argument,
                                    bool write, bool block_io) {
    sdmmc_state_t *s = &p->sdmmc;
    sdmmc_card_state_t *card = &s->card[slot];
    size_t total = s->regs[SDMMC_BYTCNT_OFF / 4u];
    if (total == 0u || !sdmmc_ensure_transfer(s, total)) return false;
    memset(s->transfer, 0, total);

    uint32_t lba = argument;
    if (block_io) {
        if ((total % 512u) != 0u) return false;
        size_t blocks = total / 512u;
        if (!sdmmc_card_range_valid(card, lba, blocks)) return false;
        if (!write && (!card->read_fn ||
            card->read_fn(card->ctx, lba, s->transfer, blocks) != 0))
            return false;
    } else if (!write) {
        if (command == 51u) {
            sdmmc_fill_scr(s->transfer, total);
        } else if (command == 6u && total >= 64u) {
            /* A zeroed SWITCH_FUNC status reports no optional high-speed
             * functions while preserving mandatory default-speed operation. */
            memset(s->transfer, 0, total);
        } else if (command == 13u) {
            memset(s->transfer, 0, total); /* SD status */
        }
    }

    s->transfer_total = total;
    s->transfer_pos = 0u;
    s->transfer_lba = lba;
    s->transfer_desc = s->regs[SDMMC_DBADDR_OFF / 4u];
    s->transfer_descriptors_seen = 0u;
    s->transfer_slot = (uint8_t)slot;
    s->transfer_cmd = (uint8_t)command;
    s->transfer_write = write;
    s->transfer_block_io = block_io;
    s->transfer_auto_stop =
        (s->regs[SDMMC_CMD_OFF / 4u] & SDMMC_CMD_SEND_AUTO_STOP) != 0u;
    s->transfer_dma =
        (s->regs[SDMMC_CTRL_OFF / 4u] &
         (SDMMC_CTRL_DMA_ENABLE | SDMMC_CTRL_USE_INTERNAL_DMA)) ==
            (SDMMC_CTRL_DMA_ENABLE | SDMMC_CTRL_USE_INTERNAL_DMA) &&
        (s->regs[SDMMC_BMOD_OFF / 4u] & SDMMC_BMOD_ENABLE) != 0u;
    s->transfer_active = true;
    s->transfer_wait_command_ack = s->transfer_dma;
    s->transfer_event_armed = false;
    s->fifo_head = 0u;
    s->fifo_count = 0u;
    return true;
}

static bool sdmmc_commit_write(sdmmc_state_t *s) {
    if (!s->transfer_write || !s->transfer_block_io) return true;
    sdmmc_card_state_t *card = &s->card[s->transfer_slot];
    size_t blocks = s->transfer_total / 512u;
    return card->write_fn && !card->write_protected &&
           card->write_fn(card->ctx, s->transfer_lba,
                          s->transfer, blocks) == 0;
}

static void sdmmc_finish_transfer(esp32_periph_t *p, bool success) {
    sdmmc_state_t *s = &p->sdmmc;
    if (success && !sdmmc_commit_write(s)) success = false;
    if (success) {
        s->rintsts |= SDMMC_INT_DATA_OVER;
        if (s->transfer_auto_stop)
            s->rintsts |= SDMMC_INT_AUTO_CMD_DONE;
    } else {
        s->rintsts |= SDMMC_INT_DATA_CRC;
        s->idsts |= SDMMC_IDSTS_CARD_ERROR |
                    SDMMC_IDSTS_ABNORMAL_SUMMARY;
    }
    s->transfer_active = false;
    s->transfer_wait_command_ack = false;
    s->transfer_event_armed = false;
    s->regs[SDMMC_TCBCNT_OFF / 4u] = (uint32_t)s->transfer_pos;
    s->regs[SDMMC_TBBCNT_OFF / 4u] = (uint32_t)s->transfer_pos;
    sdmmc_irq_update(p);
    sdmmc_kick(p);
}

static bool sdmmc_dma_range_mapped(esp32_periph_t *p, uint32_t addr,
                                    size_t len, bool writable) {
    while (len != 0u) {
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

static bool sdmmc_dma_copy_buffer(esp32_periph_t *p, uint32_t addr,
                                   size_t size) {
    sdmmc_state_t *s = &p->sdmmc;
    size_t remaining = s->transfer_total - s->transfer_pos;
    if (size > remaining) size = remaining;
    if (size == 0u) return true;
    if ((addr & 3u) != 0u ||
        !sdmmc_dma_range_mapped(p, addr, size, !s->transfer_write))
        return false;
    for (size_t index = 0; index < size; index++) {
        if (s->transfer_write)
            s->transfer[s->transfer_pos + index] =
                mem_read8(p->mem, addr + (uint32_t)index);
        else
            mem_write8(p->mem, addr + (uint32_t)index,
                       s->transfer[s->transfer_pos + index]);
    }
    s->transfer_pos += size;
    s->regs[SDMMC_BUFADDRL_OFF / 4u] = addr + (uint32_t)size;
    s->regs[SDMMC_TCBCNT_OFF / 4u] = (uint32_t)s->transfer_pos;
    s->regs[SDMMC_TBBCNT_OFF / 4u] = (uint32_t)s->transfer_pos;
    return true;
}

static void sdmmc_dma_error(esp32_periph_t *p, uint32_t desc,
                            uint32_t idsts) {
    sdmmc_state_t *s = &p->sdmmc;
    if ((desc & 3u) == 0u &&
        sdmmc_dma_range_mapped(p, desc, 4u, true)) {
        uint32_t ctrl = mem_read32(p->mem, desc);
        mem_write32(p->mem, desc,
                    (ctrl & ~SDMMC_DESC_OWNER) | SDMMC_DESC_CARD_ERROR);
    }
    s->regs[SDMMC_DSCADDR_OFF / 4u] = desc;
    s->idsts |= idsts | SDMMC_IDSTS_ABNORMAL_SUMMARY;
    s->rintsts |= SDMMC_INT_FIFO_RUN;
    s->transfer_active = false;
    s->transfer_event_armed = false;
    sdmmc_irq_update(p);
    sdmmc_kick(p);
}

static void sdmmc_arm_transfer(esp32_periph_t *p) {
    sdmmc_state_t *s = &p->sdmmc;
    xtensa_cpu_t *cpu = sdmmc_event_cpu(p);
    if (!cpu || !sdmmc_clocked(p) || !s->transfer_active ||
        !s->transfer_dma || s->transfer_wait_command_ack ||
        s->transfer_event_armed)
        return;
    /* A 4 KiB descriptor needs about 65k CPU cycles on a 20 MHz, four-bit
     * default-speed bus.  Keeping that wire-time lower bound matters for the
     * stock ESP-IDF four-entry ring: its task refills a completed descriptor
     * after the ISR reports RI/TI, before IDMAC can wrap back to it. */
    uint32_t bytes = 0u;
    uint32_t desc = s->transfer_desc;
    if ((desc & 3u) == 0u &&
        sdmmc_dma_range_mapped(p, desc, 16u, false)) {
        uint32_t ctrl = mem_read32(p->mem, desc);
        uint32_t sizes = mem_read32(p->mem, desc + 4u);
        bytes = sizes & SDMMC_DESC_SIZE_MASK;
        if (!(ctrl & SDMMC_DESC_CHAINED))
            bytes += (sizes >> SDMMC_DESC_SIZE2_SHIFT) &
                     SDMMC_DESC_SIZE_MASK;
    }
    uint32_t delay = bytes > UINT32_MAX / 16u ? UINT32_MAX : bytes * 16u;
    if (delay < 2048u) delay = 2048u;
    s->next_transfer_ccount = cpu->ccount + delay;
    s->transfer_event_armed = true;
    sdmmc_kick(p);
}

static void sdmmc_process_descriptor(esp32_periph_t *p) {
    sdmmc_state_t *s = &p->sdmmc;
    uint32_t desc = s->transfer_desc;
    if (!s->transfer_active || !s->transfer_dma) return;
    if (++s->transfer_descriptors_seen > SDMMC_DMA_MAX_DESCRIPTORS ||
        (desc & 3u) != 0u ||
        !sdmmc_dma_range_mapped(p, desc, 16u, true)) {
        sdmmc_dma_error(p, desc, SDMMC_IDSTS_FATAL_BUS);
        return;
    }
    uint32_t ctrl = mem_read32(p->mem, desc);
    uint32_t sizes = mem_read32(p->mem, desc + 4u);
    uint32_t buffer1 = mem_read32(p->mem, desc + 8u);
    uint32_t fourth = mem_read32(p->mem, desc + 12u);
    size_t size1 = sizes & SDMMC_DESC_SIZE_MASK;
    size_t size2 = (sizes >> SDMMC_DESC_SIZE2_SHIFT) &
                   SDMMC_DESC_SIZE_MASK;
    s->regs[SDMMC_DSCADDR_OFF / 4u] = desc;
    if (!(ctrl & SDMMC_DESC_OWNER) || (size1 == 0u && size2 == 0u)) {
        sdmmc_dma_error(p, desc, SDMMC_IDSTS_DESC_UNAVAIL);
        return;
    }

    bool ok = sdmmc_dma_copy_buffer(p, buffer1, size1);
    if (ok && !(ctrl & SDMMC_DESC_CHAINED) && size2 != 0u)
        ok = sdmmc_dma_copy_buffer(p, fourth, size2);
    bool final = (ctrl & SDMMC_DESC_LAST) != 0u ||
                 s->transfer_pos >= s->transfer_total;
    uint32_t writeback = ctrl & ~SDMMC_DESC_OWNER;
    if (!ok) writeback |= SDMMC_DESC_CARD_ERROR;
    mem_write32(p->mem, desc, writeback);
    if (!ok) {
        sdmmc_dma_error(p, desc, SDMMC_IDSTS_FATAL_BUS);
        return;
    }

    uint32_t completion = s->transfer_write ? SDMMC_IDSTS_TX :
                                                  SDMMC_IDSTS_RX;
    bool interrupt = (ctrl & SDMMC_DESC_DIC) == 0u || final;
    if (interrupt)
        s->idsts |= completion | SDMMC_IDSTS_NORMAL_SUMMARY;

    if (final) {
        if (s->transfer_pos != s->transfer_total) {
            sdmmc_dma_error(p, desc, SDMMC_IDSTS_DESC_UNAVAIL);
            return;
        }
        sdmmc_finish_transfer(p, true);
        return;
    }

    uint32_t next;
    if (ctrl & SDMMC_DESC_CHAINED) {
        next = fourth;
    } else if (ctrl & SDMMC_DESC_END_RING) {
        next = s->regs[SDMMC_DBADDR_OFF / 4u];
    } else {
        uint32_t skip = (s->regs[SDMMC_BMOD_OFF / 4u] >> 2) & 0x1Fu;
        next = desc + 16u + skip * 4u;
    }
    if (next == 0u || next == desc) {
        sdmmc_dma_error(p, desc, SDMMC_IDSTS_DESC_UNAVAIL);
        return;
    }
    s->transfer_desc = next;
    if (!interrupt) sdmmc_arm_transfer(p);
    sdmmc_irq_update(p);
}

static void sdmmc_fifo_fill_read(esp32_periph_t *p) {
    sdmmc_state_t *s = &p->sdmmc;
    while (s->fifo_count < SDMMC_FIFO_WORDS &&
           s->transfer_pos < s->transfer_total) {
        uint32_t word = 0u;
        for (unsigned byte = 0; byte < 4u &&
             s->transfer_pos < s->transfer_total; byte++)
            word |= (uint32_t)s->transfer[s->transfer_pos++] << (byte * 8u);
        unsigned tail = (s->fifo_head + s->fifo_count) % SDMMC_FIFO_WORDS;
        s->fifo[tail] = word;
        s->fifo_count++;
    }
    if (s->fifo_count != 0u) s->rintsts |= SDMMC_INT_RXDR;
    sdmmc_irq_update(p);
}

static uint32_t sdmmc_fifo_read(esp32_periph_t *p) {
    sdmmc_state_t *s = &p->sdmmc;
    if (s->fifo_count == 0u) {
        s->rintsts |= SDMMC_INT_FIFO_RUN;
        sdmmc_irq_update(p);
        return 0u;
    }
    uint32_t value = s->fifo[s->fifo_head];
    s->fifo_head = (uint8_t)((s->fifo_head + 1u) % SDMMC_FIFO_WORDS);
    s->fifo_count--;
    if (s->fifo_count == 0u) {
        s->rintsts &= ~SDMMC_INT_RXDR;
        if (s->transfer_active && !s->transfer_dma &&
            !s->transfer_write) {
            if (s->transfer_pos < s->transfer_total)
                sdmmc_fifo_fill_read(p);
            else
                sdmmc_finish_transfer(p, true);
        }
    }
    return value;
}

static void sdmmc_fifo_write(esp32_periph_t *p, uint32_t value) {
    sdmmc_state_t *s = &p->sdmmc;
    if (!s->transfer_active || s->transfer_dma || !s->transfer_write) {
        if (s->fifo_count >= SDMMC_FIFO_WORDS) {
            s->rintsts |= SDMMC_INT_FIFO_RUN;
            sdmmc_irq_update(p);
            return;
        }
        unsigned tail = (s->fifo_head + s->fifo_count) % SDMMC_FIFO_WORDS;
        s->fifo[tail] = value;
        s->fifo_count++;
        return;
    }
    for (unsigned byte = 0; byte < 4u &&
         s->transfer_pos < s->transfer_total; byte++)
        s->transfer[s->transfer_pos++] = (uint8_t)(value >> (byte * 8u));
    if (s->transfer_pos >= s->transfer_total) {
        s->rintsts &= ~SDMMC_INT_TXDR;
        sdmmc_finish_transfer(p, true);
    } else {
        s->rintsts |= SDMMC_INT_TXDR;
        sdmmc_irq_update(p);
    }
}

static void sdmmc_execute_command(esp32_periph_t *p, uint32_t value) {
    sdmmc_state_t *s = &p->sdmmc;
    s->regs[SDMMC_CMD_OFF / 4u] = value & ~SDMMC_CMD_START;
    if (!(value & SDMMC_CMD_START)) return;
    if (value & SDMMC_CMD_UPDATE_CLOCK) {
        sdmmc_irq_update(p);
        return;
    }

    unsigned command = value & SDMMC_CMD_INDEX_MASK;
    unsigned slot = (value & SDMMC_CMD_CARD_MASK) >> SDMMC_CMD_CARD_SHIFT;
    uint32_t argument = s->regs[SDMMC_CMDARG_OFF / 4u];
    s->transfer_cmd = (uint8_t)command;
    uint32_t response[4] = {0u, 0u, 0u, 0u};
    uint32_t status = SDMMC_INT_CMD_DONE;
    bool data = (value & SDMMC_CMD_DATA_EXPECTED) != 0u;

    if (slot >= SDMMC_SLOT_COUNT || !s->card[slot].attached) {
        if (value & SDMMC_CMD_RESPONSE_EXPECT)
            status |= SDMMC_INT_RESP_TIMEOUT;
        s->rintsts |= status;
        sdmmc_irq_update(p);
        return;
    }

    sdmmc_card_state_t *card = &s->card[slot];
    bool was_app = card->app_cmd;
    if (command != 55u) card->app_cmd = false;

    switch (command) {
    case 0: /* GO_IDLE_STATE */
        card->ready = false;
        card->selected = false;
        card->app_cmd = false;
        card->block_len = 512u;
        break;
    case 1: /* MMC SEND_OP_COND compatibility */
        card->ready = true;
        response[0] = 0xC0FF8000u;
        break;
    case 2: /* ALL_SEND_CID */
    case 10: /* SEND_CID */
        sdmmc_cid_response(slot, response);
        break;
    case 3: /* SEND_RELATIVE_ADDR */
        if ((argument >> 16) != 0u) card->rca = (uint16_t)(argument >> 16);
        card->ready = true;
        response[0] = (uint32_t)card->rca << 16 |
                      sdmmc_r1_status(card, false);
        break;
    case 5: /* no SDIO functions on the attached memory card */
        status |= SDMMC_INT_RESP_TIMEOUT;
        break;
    case 6:
        response[0] = sdmmc_r1_status(card, false);
        if (!was_app && data &&
            !sdmmc_prepare_transfer(p, slot, command, argument, false, false))
            status |= SDMMC_INT_DATA_TIMEOUT;
        break;
    case 7: /* SELECT/DESELECT_CARD */
        card->selected = argument != 0u;
        response[0] = sdmmc_r1_status(card, false);
        break;
    case 8: /* SEND_IF_COND (SD); EXT_CSD is not exposed by an SD card */
        response[0] = argument & 0xFFFu;
        break;
    case 9: /* SEND_CSD */
        sdmmc_csd_response(card, response);
        break;
    case 11: /* VOLTAGE_SWITCH */
    case 12: /* STOP_TRANSMISSION */
        response[0] = sdmmc_r1_status(card, false);
        break;
    case 13:
        response[0] = sdmmc_r1_status(card, false);
        if (was_app && data &&
            !sdmmc_prepare_transfer(p, slot, command, argument, false, false))
            status |= SDMMC_INT_DATA_TIMEOUT;
        break;
    case 16: /* SET_BLOCKLEN */
        if (argument != 0u && argument <= 4096u) card->block_len = argument;
        response[0] = sdmmc_r1_status(card, false);
        break;
    case 17: /* READ_SINGLE_BLOCK */
    case 18: /* READ_MULTIPLE_BLOCK */
        response[0] = sdmmc_r1_status(card, false);
        if (!data || !sdmmc_prepare_transfer(p, slot, command, argument,
                                              false, true))
            status |= SDMMC_INT_DATA_TIMEOUT;
        break;
    case 23: /* SET_BLOCK_COUNT / ACMD23 */
        response[0] = sdmmc_r1_status(card, false);
        break;
    case 24: /* WRITE_BLOCK */
    case 25: /* WRITE_MULTIPLE_BLOCK */
        response[0] = sdmmc_r1_status(card, false);
        if (!data || card->write_protected || !card->write_fn ||
            !sdmmc_prepare_transfer(p, slot, command, argument, true, true))
            status |= SDMMC_INT_DATA_TIMEOUT;
        break;
    case 41: /* ACMD41 SD_SEND_OP_COND */
        if (!was_app) {
            status |= SDMMC_INT_RESP_ERROR;
        } else {
            card->ready = true;
            response[0] = 0xC0FF8000u;
        }
        break;
    case 42: /* ACMD42 SET_CLR_CARD_DETECT */
        response[0] = sdmmc_r1_status(card, false);
        break;
    case 51: /* ACMD51 SEND_SCR */
        response[0] = sdmmc_r1_status(card, false);
        if (!was_app || !data ||
            !sdmmc_prepare_transfer(p, slot, command, argument, false, false))
            status |= SDMMC_INT_DATA_TIMEOUT;
        break;
    case 55: /* APP_CMD */
        card->app_cmd = true;
        response[0] = sdmmc_r1_status(card, true);
        break;
    default:
        response[0] = sdmmc_r1_status(card, false) | (1u << 22);
        status |= SDMMC_INT_RESP_ERROR;
        break;
    }

    sdmmc_set_response(s, response);
    if (status & SDMMC_INT_DATA_TIMEOUT) {
        sdmmc_cancel_transfer(s);
        status |= SDMMC_INT_DATA_OVER;
    }
    s->rintsts |= status;
    if (s->transfer_active && !s->transfer_dma) {
        s->transfer_wait_command_ack = false;
        if (s->transfer_write)
            s->rintsts |= SDMMC_INT_TXDR;
        else
            sdmmc_fifo_fill_read(p);
    }
    sdmmc_irq_update(p);
    sdmmc_kick(p);
}

static uint32_t sdmmc_next_fire(esp32_periph_t *p, xtensa_cpu_t *cpu) {
    if (!p || !cpu || cpu != sdmmc_event_cpu(p) || !sdmmc_clocked(p))
        return UINT32_MAX;

    sdmmc_state_t *s = &p->sdmmc;
    if (!s->transfer_event_armed) return UINT32_MAX;
    return s->next_transfer_ccount;
}

static void sdmmc_eval_events(esp32_periph_t *p, xtensa_cpu_t *cpu) {
    if (!p || !cpu || cpu != sdmmc_event_cpu(p) || !sdmmc_clocked(p))
        return;

    sdmmc_state_t *s = &p->sdmmc;
    if (!s->transfer_event_armed ||
        (int32_t)(cpu->ccount - s->next_transfer_ccount) < 0)
        return;
    s->transfer_event_armed = false;
    sdmmc_process_descriptor(p);
}

static uint32_t sdmmc_read(void *ctx, uint32_t addr) {
    esp32_periph_t *p = ctx;
    sdmmc_state_t *s = &p->sdmmc;
    uint32_t off = addr - SDMMC_BASE;
    xtensa_cpu_t *cpu = sdmmc_event_cpu(p);
    if (cpu) sdmmc_eval_events(p, cpu);
    if ((off & 3u) != 0u || off >= SDMMC_REG_FILE_SIZE)
        return default_read(ctx, addr);
    if (off >= SDMMC_FIFO_OFF && off < SDMMC_FIFO_OFF + 0x80u)
        return sdmmc_fifo_read(p);
    switch (off) {
    case SDMMC_MINTSTS_OFF:
        return s->rintsts & s->regs[SDMMC_INTMASK_OFF / 4u];
    case SDMMC_RINTSTS_OFF: return s->rintsts;
    case SDMMC_STATUS_OFF: return sdmmc_status(s);
    case SDMMC_CDETECT_OFF: return sdmmc_card_detect(s);
    case SDMMC_WRTPRT_OFF: return sdmmc_write_protect(s);
    case SDMMC_IDSTS_OFF: return s->idsts;
    default: return s->regs[off / 4u];
    }
}

static void sdmmc_write(void *ctx, uint32_t addr, uint32_t value) {
    esp32_periph_t *p = ctx;
    sdmmc_state_t *s = &p->sdmmc;
    uint32_t off = addr - SDMMC_BASE;
    xtensa_cpu_t *cpu = sdmmc_event_cpu(p);
    if (cpu) sdmmc_eval_events(p, cpu);
    if ((off & 3u) != 0u || off >= SDMMC_REG_FILE_SIZE) {
        default_write(ctx, addr, value);
        return;
    }
    if (off >= SDMMC_FIFO_OFF && off < SDMMC_FIFO_OFF + 0x80u) {
        sdmmc_fifo_write(p, value);
        return;
    }
    switch (off) {
    case SDMMC_CTRL_OFF: {
        uint32_t reset = value & (SDMMC_CTRL_CONTROLLER_RESET |
                                  SDMMC_CTRL_FIFO_RESET |
                                  SDMMC_CTRL_DMA_RESET);
        if (reset & SDMMC_CTRL_CONTROLLER_RESET) {
            sdmmc_reset_state(p);
            s = &p->sdmmc;
        }
        if (reset & SDMMC_CTRL_FIFO_RESET) {
            s->fifo_head = 0u;
            s->fifo_count = 0u;
        }
        if (reset & SDMMC_CTRL_DMA_RESET)
            sdmmc_cancel_transfer(s);
        s->regs[off / 4u] = value & SDMMC_CTRL_VALID_MASK & ~reset;
        sdmmc_irq_update(p);
        sdmmc_kick(p);
        return;
    }
    case SDMMC_CMD_OFF:
        s->regs[off / 4u] = value;
        if (sdmmc_clocked(p)) sdmmc_execute_command(p, value);
        return;
    case SDMMC_RINTSTS_OFF: {
        uint32_t cleared = value & SDMMC_INT_VALID_MASK;
        s->rintsts &= ~cleared;
        if (s->transfer_active && s->transfer_dma &&
            s->transfer_wait_command_ack &&
            (cleared & SDMMC_INT_CMD_DONE)) {
            s->transfer_wait_command_ack = false;
            sdmmc_arm_transfer(p);
        }
        sdmmc_irq_update(p);
        return;
    }
    case SDMMC_IDSTS_OFF:
        s->idsts &= ~(value & SDMMC_IDSTS_VALID_MASK);
        if (s->transfer_active && s->transfer_dma &&
            s->idsts == 0u)
            sdmmc_arm_transfer(p);
        sdmmc_irq_update(p);
        return;
    case SDMMC_INTMASK_OFF:
        s->regs[off / 4u] = value;
        sdmmc_irq_update(p);
        return;
    case SDMMC_IDINTEN_OFF:
        s->regs[off / 4u] = value & SDMMC_IDSTS_VALID_MASK;
        sdmmc_irq_update(p);
        return;
    case SDMMC_BMOD_OFF:
        s->regs[off / 4u] = value & ~SDMMC_BMOD_SW_RESET;
        if (value & SDMMC_BMOD_SW_RESET) s->idsts = 0u;
        sdmmc_irq_update(p);
        return;
    case SDMMC_PLDMND_OFF:
        s->regs[off / 4u] = value;
        if (value != 0u) sdmmc_arm_transfer(p);
        return;
    case SDMMC_MINTSTS_OFF:
    case SDMMC_STATUS_OFF:
    case SDMMC_CDETECT_OFF:
    case SDMMC_WRTPRT_OFF:
    case SDMMC_RESP0_OFF:
    case SDMMC_RESP1_OFF:
    case SDMMC_RESP2_OFF:
    case SDMMC_RESP3_OFF:
    case SDMMC_TCBCNT_OFF:
    case SDMMC_TBBCNT_OFF:
    case SDMMC_VERID_OFF:
    case SDMMC_HCON_OFF:
    case SDMMC_DSCADDR_OFF:
    case SDMMC_BUFADDRL_OFF:
        return;
    default:
        s->regs[off / 4u] = value;
        return;
    }
}

/* ---- Classic ESP32 TWAI/CAN (SJA1000-compatible PeliCAN core) ---- */

static bool twai_clocked(const esp32_periph_t *p) {
    return (p->dport_perip_clk_en & DPORT_TWAI_MODULE_BIT) != 0u &&
           (p->dport_perip_rst_en & DPORT_TWAI_MODULE_BIT) == 0u;
}

static xtensa_cpu_t *twai_event_cpu(esp32_periph_t *p) {
    return p->cpu[0] ? p->cpu[0] : p->cpu[1];
}

static void twai_kick(esp32_periph_t *p) {
    periph_event_source_changed(p, PERIPH_EVENT_TWAI);
    for (unsigned core = 0; core < 2u; core++)
        if (p->cpu[core]) xtensa_recompute_next_timer(p->cpu[core]);
}

static uint8_t twai_status(const twai_state_t *s) {
    uint8_t status = 0u;
    if (s->rx_count != 0u) status |= TWAI_STATUS_RX_BUFFER;
    if (s->data_overrun) status |= TWAI_STATUS_DATA_OVERRUN;
    if (!s->tx_busy) status |= TWAI_STATUS_TX_BUFFER;
    if (s->tx_complete) status |= TWAI_STATUS_TX_COMPLETE;
    if (s->tx_busy) status |= TWAI_STATUS_TRANSMITTING;
    if (s->tx_error_count >= s->error_warning_limit ||
        s->rx_error_count >= s->error_warning_limit)
        status |= TWAI_STATUS_ERROR;
    if (s->bus_off) status |= TWAI_STATUS_BUS_OFF;
    return status;
}

static void twai_irq_update(esp32_periph_t *p) {
    twai_state_t *s = &p->twai;
    bool active = twai_clocked(p) &&
                  (s->int_raw & s->int_ena & TWAI_INT_VALID_MASK) != 0u;
    if (active)
        periph_assert_interrupt_status(p, TWAI_INTR_SOURCE,
                                       s->int_raw & s->int_ena &
                                       TWAI_INT_VALID_MASK);
    else
        periph_deassert_interrupt(p, TWAI_INTR_SOURCE);
}

static void twai_clear_rx_fifo(twai_state_t *s) {
    memset(s->rx_fifo, 0, sizeof(s->rx_fifo));
    s->rx_head = 0u;
    s->rx_count = 0u;
    s->rx_bytes = 0u;
    s->int_raw &= (uint8_t)~TWAI_INT_RX;
}

static void twai_reset_state(esp32_periph_t *p) {
    if (!p) return;
    periph_deassert_interrupt(p, TWAI_INTR_SOURCE);
    twai_state_t *s = &p->twai;
    periph_twai_tx_fn tx_cb = s->tx_cb;
    void *tx_cb_ctx = s->tx_cb_ctx;
    memset(s, 0, sizeof(*s));
    s->mode = TWAI_MODE_RESET;
    s->tx_complete = true;
    s->error_warning_limit = TWAI_DEFAULT_EWL;
    memset(s->acceptance_mask, 0xFF, sizeof(s->acceptance_mask));
    s->tx_cb = tx_cb;
    s->tx_cb_ctx = tx_cb_ctx;
    twai_kick(p);
}

static void twai_dport_update(esp32_periph_t *p) {
    if (!p) return;
    twai_state_t *s = &p->twai;
    if (!twai_clocked(p)) {
        s->tx_event_armed = false;
        s->recovery_event_armed = false;
        s->tx_busy = false;
        s->tx_complete = true;
        periph_deassert_interrupt(p, TWAI_INTR_SOURCE);
    } else {
        twai_irq_update(p);
    }
    twai_kick(p);
}

static bool twai_frame_valid(const periph_twai_frame_t *frame) {
    if (!frame || frame->data_length_code > 15u) return false;
    uint32_t mask = frame->extended ? 0x1FFFFFFFu : 0x7FFu;
    return (frame->identifier & ~mask) == 0u;
}

static void twai_encode_frame(const periph_twai_frame_t *frame,
                              uint8_t bytes[13]) {
    memset(bytes, 0, 13u);
    bytes[0] = frame->data_length_code & TWAI_FRAME_DLC_MASK;
    if (frame->extended) bytes[0] |= TWAI_FRAME_EXTENDED;
    if (frame->remote) bytes[0] |= TWAI_FRAME_REMOTE;
    if (frame->single_shot) bytes[0] |= TWAI_FRAME_SINGLE_SHOT;
    if (frame->self_reception) bytes[0] |= TWAI_FRAME_SELF_RX;

    uint8_t data_off;
    if (frame->extended) {
        bytes[1] = (uint8_t)(frame->identifier >> 21);
        bytes[2] = (uint8_t)(frame->identifier >> 13);
        bytes[3] = (uint8_t)(frame->identifier >> 5);
        bytes[4] = (uint8_t)(frame->identifier << 3);
        data_off = 5u;
    } else {
        bytes[1] = (uint8_t)(frame->identifier >> 3);
        bytes[2] = (uint8_t)(frame->identifier << 5);
        data_off = 3u;
    }
    if (!frame->remote) {
        uint8_t len = frame->data_length_code < 8u ?
                      frame->data_length_code : 8u;
        memcpy(&bytes[data_off], frame->data, len);
    }
}

static void twai_decode_frame(const uint8_t bytes[13],
                              periph_twai_frame_t *frame) {
    memset(frame, 0, sizeof(*frame));
    frame->data_length_code = bytes[0] & TWAI_FRAME_DLC_MASK;
    frame->extended = (bytes[0] & TWAI_FRAME_EXTENDED) != 0u;
    frame->remote = (bytes[0] & TWAI_FRAME_REMOTE) != 0u;
    frame->single_shot = (bytes[0] & TWAI_FRAME_SINGLE_SHOT) != 0u;
    frame->self_reception = (bytes[0] & TWAI_FRAME_SELF_RX) != 0u;

    uint8_t data_off;
    if (frame->extended) {
        frame->identifier = ((uint32_t)bytes[1] << 21) |
                            ((uint32_t)bytes[2] << 13) |
                            ((uint32_t)bytes[3] << 5) |
                            ((uint32_t)bytes[4] >> 3);
        data_off = 5u;
    } else {
        frame->identifier = ((uint32_t)bytes[1] << 3) |
                            ((uint32_t)bytes[2] >> 5);
        data_off = 3u;
    }
    if (!frame->remote) {
        uint8_t len = frame->data_length_code < 8u ?
                      frame->data_length_code : 8u;
        memcpy(frame->data, &bytes[data_off], len);
    }
}

static uint8_t twai_frame_storage_bytes(const periph_twai_frame_t *frame) {
    uint8_t data_bytes = frame->remote ? 0u :
        (frame->data_length_code < 8u ? frame->data_length_code : 8u);
    return (uint8_t)((frame->extended ? 5u : 3u) + data_bytes);
}

static bool twai_filter_match(const twai_state_t *s,
                              const uint8_t frame[13]) {
    uint8_t key[4] = {0};
    bool extended = (frame[0] & TWAI_FRAME_EXTENDED) != 0u;
    bool remote = (frame[0] & TWAI_FRAME_REMOTE) != 0u;
    if (extended) {
        memcpy(key, &frame[1], sizeof(key));
        if (remote) key[3] |= 1u << 2;
    } else {
        key[0] = frame[1];
        key[1] = (uint8_t)(frame[2] | (remote ? 1u << 4 : 0u));
        if (!remote) {
            key[2] = frame[3];
            key[3] = frame[4];
        }
    }

    if (s->mode & TWAI_MODE_SINGLE_FILTER) {
        for (unsigned i = 0; i < 4u; i++)
            if (((key[i] ^ s->acceptance_code[i]) &
                 (uint8_t)~s->acceptance_mask[i]) != 0u)
                return false;
        return true;
    }

    /* Dual-filter mode compares two independent 16-bit prefixes. This is the
     * SJA1000 layout used for standard-ID filters and the upper 16 ID bits of
     * extended frames. */
    bool first = true;
    bool second = true;
    for (unsigned i = 0; i < 2u; i++) {
        if (((key[i] ^ s->acceptance_code[i]) &
             (uint8_t)~s->acceptance_mask[i]) != 0u)
            first = false;
        if (((key[i] ^ s->acceptance_code[i + 2u]) &
             (uint8_t)~s->acceptance_mask[i + 2u]) != 0u)
            second = false;
    }
    return first || second;
}

static int twai_enqueue_rx(esp32_periph_t *p,
                           const periph_twai_frame_t *frame,
                           bool apply_filter) {
    twai_state_t *s = &p->twai;
    if (!twai_frame_valid(frame) || !twai_clocked(p) || s->bus_off ||
        (s->mode & TWAI_MODE_RESET) != 0u)
        return 0;

    uint8_t encoded[13];
    twai_encode_frame(frame, encoded);
    encoded[0] &= (uint8_t)~(TWAI_FRAME_SELF_RX | TWAI_FRAME_SINGLE_SHOT);
    if (apply_filter && !twai_filter_match(s, encoded)) return 0;

    uint8_t storage = twai_frame_storage_bytes(frame);
    if (s->rx_count >= TWAI_RX_FIFO_FRAMES ||
        storage > TWAI_RX_FIFO_BYTES - s->rx_bytes) {
        s->data_overrun = true;
        s->int_raw |= TWAI_INT_DATA_OVERRUN;
        twai_irq_update(p);
        return 0;
    }

    unsigned tail = (s->rx_head + s->rx_count) % TWAI_RX_FIFO_FRAMES;
    memcpy(s->rx_fifo[tail].bytes, encoded, sizeof(encoded));
    s->rx_fifo[tail].storage_bytes = storage;
    s->rx_count++;
    s->rx_bytes = (uint8_t)(s->rx_bytes + storage);
    s->int_raw |= TWAI_INT_RX;
    twai_irq_update(p);
    return 1;
}

static void twai_release_rx(esp32_periph_t *p) {
    twai_state_t *s = &p->twai;
    if (s->rx_count != 0u) {
        s->rx_bytes = (uint8_t)(s->rx_bytes -
            s->rx_fifo[s->rx_head].storage_bytes);
        memset(&s->rx_fifo[s->rx_head], 0,
               sizeof(s->rx_fifo[s->rx_head]));
        s->rx_head = (uint8_t)((s->rx_head + 1u) % TWAI_RX_FIFO_FRAMES);
        s->rx_count--;
    }
    if (s->rx_count == 0u) s->int_raw &= (uint8_t)~TWAI_INT_RX;
    else s->int_raw |= TWAI_INT_RX;
    twai_irq_update(p);
}

typedef struct {
    uint32_t wire_bits;
    uint16_t crc;
    uint8_t last;
    uint8_t run;
    bool have_last;
} twai_bit_stream_t;

static void twai_stream_bit(twai_bit_stream_t *stream, unsigned bit,
                            bool update_crc) {
    bit &= 1u;
    if (update_crc) {
        unsigned feedback = ((stream->crc >> 14) & 1u) ^ bit;
        stream->crc = (uint16_t)((stream->crc << 1) & 0x7FFFu);
        if (feedback) stream->crc ^= 0x4599u;
    }

    stream->wire_bits++;
    if (!stream->have_last || stream->last != bit) {
        stream->last = (uint8_t)bit;
        stream->run = 1u;
        stream->have_last = true;
    } else {
        stream->run++;
    }
    if (stream->run == 5u) {
        stream->wire_bits++;
        stream->last ^= 1u;
        stream->run = 1u;
    }
}

static void twai_stream_field(twai_bit_stream_t *stream, uint32_t value,
                              unsigned width) {
    while (width-- != 0u)
        twai_stream_bit(stream, value >> width, true);
}

static uint32_t twai_frame_wire_bits(const periph_twai_frame_t *frame) {
    twai_bit_stream_t stream = {0};
    twai_stream_bit(&stream, 0u, true); /* SOF */
    if (frame->extended) {
        twai_stream_field(&stream, frame->identifier >> 18, 11u);
        twai_stream_bit(&stream, 1u, true); /* SRR */
        twai_stream_bit(&stream, 1u, true); /* IDE */
        twai_stream_field(&stream, frame->identifier, 18u);
        twai_stream_bit(&stream, frame->remote, true);
        twai_stream_bit(&stream, 0u, true); /* r1 */
        twai_stream_bit(&stream, 0u, true); /* r0 */
    } else {
        twai_stream_field(&stream, frame->identifier, 11u);
        twai_stream_bit(&stream, frame->remote, true);
        twai_stream_bit(&stream, 0u, true); /* IDE */
        twai_stream_bit(&stream, 0u, true); /* r0 */
    }
    twai_stream_field(&stream, frame->data_length_code, 4u);
    if (!frame->remote) {
        unsigned len = frame->data_length_code < 8u ?
                       frame->data_length_code : 8u;
        for (unsigned i = 0; i < len; i++)
            twai_stream_field(&stream, frame->data[i], 8u);
    }

    uint16_t crc = stream.crc;
    for (unsigned bit = 15u; bit-- != 0u;)
        twai_stream_bit(&stream, crc >> bit, false);

    /* CRC delimiter, ACK slot/delimiter, EOF, and intermission are outside
     * the bit-stuffed region. */
    return stream.wire_bits + 13u;
}

static uint32_t twai_bit_cycles(const esp32_periph_t *p) {
    const twai_state_t *s = &p->twai;
    uint32_t brp = 2u * ((s->bus_timing_0 & 0x3Fu) + 1u);
    if (s->int_ena & (1u << 4)) brp *= 2u;
    uint32_t tseg1 = (s->bus_timing_1 & 0x0Fu) + 1u;
    uint32_t tseg2 = ((s->bus_timing_1 >> 4) & 0x07u) + 1u;
    uint64_t numerator = (uint64_t)brp * (1u + tseg1 + tseg2) *
                         timg_cpu_mhz((esp32_periph_t *)p);
    uint32_t cycles = (uint32_t)((numerator + 79u) / 80u);
    return cycles != 0u ? cycles : 1u;
}

static uint32_t twai_wire_cycles(const esp32_periph_t *p,
                                 const periph_twai_frame_t *frame) {
    uint64_t cycles = (uint64_t)twai_frame_wire_bits(frame) *
                      twai_bit_cycles(p);
    if (cycles == 0u) cycles = 1u;
    if (cycles > INT32_MAX) cycles = INT32_MAX;
    return (uint32_t)cycles;
}

static void twai_arm_tx(esp32_periph_t *p) {
    twai_state_t *s = &p->twai;
    xtensa_cpu_t *cpu = twai_event_cpu(p);
    if (!cpu) return;
    s->tx_event_armed = true;
    s->next_tx_cycle = periph_clock_now(p, &p->event_clock) +
                       twai_wire_cycles(p, &s->tx_frame);
    twai_kick(p);
}

static void twai_update_error_thresholds(twai_state_t *s,
                                         uint16_t previous_tec,
                                         uint16_t previous_rec) {
    bool was_warning = previous_tec >= s->error_warning_limit ||
                       previous_rec >= s->error_warning_limit;
    bool now_warning = s->tx_error_count >= s->error_warning_limit ||
                       s->rx_error_count >= s->error_warning_limit;
    if (was_warning != now_warning) s->int_raw |= TWAI_INT_ERROR;

    bool was_passive = previous_tec >= 128u || previous_rec >= 128u;
    bool now_passive = s->tx_error_count >= 128u ||
                       s->rx_error_count >= 128u;
    if (was_passive != now_passive) s->int_raw |= TWAI_INT_ERROR_PASSIVE;
}

static void twai_finish_tx_attempt(esp32_periph_t *p) {
    twai_state_t *s = &p->twai;
    if (!s->tx_busy) return;

    periph_twai_tx_result_t result = s->tx_cb ?
        s->tx_cb(s->tx_cb_ctx, &s->tx_frame) : PERIPH_TWAI_TX_ACK;
    if (result < PERIPH_TWAI_TX_ACK || result > PERIPH_TWAI_TX_BUS_ERROR)
        result = PERIPH_TWAI_TX_BUS_ERROR;
    if (result == PERIPH_TWAI_TX_NO_ACK &&
        (s->mode & TWAI_MODE_SELF_TEST))
        result = PERIPH_TWAI_TX_ACK;

    if (result == PERIPH_TWAI_TX_ACK) {
        uint16_t previous_tec = s->tx_error_count;
        uint16_t previous_rec = s->rx_error_count;
        if (s->tx_error_count != 0u) s->tx_error_count--;
        twai_update_error_thresholds(s, previous_tec, previous_rec);
        s->tx_busy = false;
        s->tx_complete = true;
        /* Latch TX completion before self-reception can assert RX. ESP-IDF's
         * classic-ESP32 lost-TX-interrupt workaround treats TBS plus an
         * occupied HAL buffer as completion even when TI is absent. Exposing
         * RI in that transient state would therefore complete the same frame
         * once for RI and again for the subsequently raised TI. */
        s->int_raw |= TWAI_INT_TX;
        if (s->tx_frame.self_reception)
            (void)twai_enqueue_rx(p, &s->tx_frame, true);
        twai_irq_update(p);
        twai_kick(p);
        return;
    }

    if (result == PERIPH_TWAI_TX_ARBITRATION_LOST) {
        s->arbitration_lost_capture = 0u;
        s->int_raw |= TWAI_INT_ARB_LOST;
    } else {
        uint16_t previous_tec = s->tx_error_count;
        uint16_t previous_rec = s->rx_error_count;
        if (s->tx_error_count <= 247u) s->tx_error_count += 8u;
        else s->tx_error_count = 256u;
        s->error_code_capture = result == PERIPH_TWAI_TX_NO_ACK ?
            (uint8_t)((3u << 6) | 25u) : 10u;
        s->int_raw |= TWAI_INT_BUS_ERROR;
        twai_update_error_thresholds(s, previous_tec, previous_rec);
        if (s->tx_error_count >= 256u) {
            s->bus_off = true;
            s->mode |= TWAI_MODE_RESET;
            s->tx_error_count = 128u;
            s->tx_busy = false;
            s->tx_complete = false;
            s->int_raw |= TWAI_INT_ERROR;
            twai_irq_update(p);
            twai_kick(p);
            return;
        }
    }

    if (s->tx_frame.single_shot) {
        s->tx_busy = false;
        s->tx_complete = false;
        s->int_raw |= TWAI_INT_TX;
    } else {
        twai_arm_tx(p);
    }
    twai_irq_update(p);
}

static void twai_start_tx(esp32_periph_t *p, uint8_t command) {
    twai_state_t *s = &p->twai;
    bool self_rx = (command & TWAI_COMMAND_SELF_RX) != 0u;
    if (!twai_clocked(p) || s->tx_busy || s->bus_off ||
        (s->mode & (TWAI_MODE_RESET | TWAI_MODE_LISTEN_ONLY)) != 0u)
        return;

    twai_decode_frame(s->tx_buffer, &s->tx_frame);
    s->tx_frame.single_shot = (command & TWAI_COMMAND_ABORT) != 0u;
    s->tx_frame.self_reception = self_rx;
    s->tx_busy = true;
    s->tx_complete = false;
    if (twai_event_cpu(p)) twai_arm_tx(p);
    else twai_finish_tx_attempt(p);
}

static void twai_abort_tx(esp32_periph_t *p) {
    twai_state_t *s = &p->twai;
    if (!s->tx_busy) return;
    s->tx_busy = false;
    s->tx_complete = true;
    s->tx_event_armed = false;
    s->int_raw |= TWAI_INT_TX;
    twai_irq_update(p);
    twai_kick(p);
}

static void twai_start_recovery(esp32_periph_t *p) {
    twai_state_t *s = &p->twai;
    xtensa_cpu_t *cpu = twai_event_cpu(p);
    if (!cpu || !s->bus_off || !twai_clocked(p)) return;
    uint64_t delay = (uint64_t)twai_bit_cycles(p) * 64u * 11u;
    if (delay == 0u) delay = 1u;
    if (delay > INT32_MAX) delay = INT32_MAX;
    s->recovery_phase = 1u;
    s->recovery_event_armed = true;
    s->next_recovery_cycle = periph_clock_now(p, &p->event_clock) + delay;
    twai_kick(p);
}

static void twai_recovery_step(esp32_periph_t *p) {
    twai_state_t *s = &p->twai;
    xtensa_cpu_t *cpu = twai_event_cpu(p);
    if (!s->bus_off || !cpu) return;
    if (s->recovery_phase == 1u) {
        s->tx_error_count = 0u;
        s->rx_error_count = 0u;
        s->int_raw |= TWAI_INT_ERROR;
        uint64_t delay = (uint64_t)twai_bit_cycles(p) * 64u * 11u;
        if (delay == 0u) delay = 1u;
        if (delay > INT32_MAX) delay = INT32_MAX;
        s->recovery_phase = 2u;
        s->recovery_event_armed = true;
        s->next_recovery_cycle = periph_clock_now(p, &p->event_clock) + delay;
    } else {
        s->bus_off = false;
        s->recovery_phase = 0u;
        s->tx_complete = true;
        s->int_raw |= TWAI_INT_ERROR;
    }
    twai_irq_update(p);
    twai_kick(p);
}

static uint32_t twai_next_fire(esp32_periph_t *p, xtensa_cpu_t *cpu) {
    if (!p || !cpu || cpu != twai_event_cpu(p) || !twai_clocked(p))
        return UINT32_MAX;
    const twai_state_t *s = &p->twai;
    bool have = false;
    uint64_t best = 0;
    if (s->tx_event_armed) {
        have = true;
        best = s->next_tx_cycle;
    }
    if (s->recovery_event_armed &&
        (!have || s->next_recovery_cycle < best)) {
        have = true;
        best = s->next_recovery_cycle;
    }
    if (!have) return UINT32_MAX;
    return periph_deadline_ccount(p, &p->event_clock, cpu, best);
}

static void twai_eval_events(esp32_periph_t *p, xtensa_cpu_t *cpu) {
    if (!p || !cpu || cpu != twai_event_cpu(p) || !twai_clocked(p))
        return;
    twai_state_t *s = &p->twai;
    uint64_t now_cycle = periph_clock_now(p, &p->event_clock);
    if (s->tx_event_armed && now_cycle >= s->next_tx_cycle) {
        s->tx_event_armed = false;
        twai_finish_tx_attempt(p);
    }
    if (s->recovery_event_armed && now_cycle >= s->next_recovery_cycle) {
        s->recovery_event_armed = false;
        twai_recovery_step(p);
    }
}

static uint32_t twai_read(void *ctx, uint32_t addr) {
    esp32_periph_t *p = ctx;
    twai_state_t *s = &p->twai;
    uint32_t off = addr - TWAI_BASE;
    xtensa_cpu_t *cpu = twai_event_cpu(p);
    if (cpu) twai_eval_events(p, cpu);
    if ((off & 3u) != 0u || off >= TWAI_REG_FILE_SIZE)
        return default_read(ctx, addr);

    switch (off) {
    case TWAI_MODE_OFF: return s->mode;
    case TWAI_COMMAND_OFF: return 0u;
    case TWAI_STATUS_OFF: return twai_status(s);
    case TWAI_INTERRUPT_OFF: {
        uint8_t value = s->int_raw;
        s->int_raw &= TWAI_INT_RX;
        if (s->rx_count == 0u) s->int_raw = 0u;
        twai_irq_update(p);
        return value;
    }
    case TWAI_INTERRUPT_ENABLE_OFF: return s->int_ena;
    case TWAI_BUS_TIMING_0_OFF: return s->bus_timing_0;
    case TWAI_BUS_TIMING_1_OFF: return s->bus_timing_1;
    case TWAI_ARB_LOST_CAPTURE_OFF: return s->arbitration_lost_capture;
    case TWAI_ERROR_CODE_CAPTURE_OFF: return s->error_code_capture;
    case TWAI_ERROR_WARNING_LIMIT_OFF: return s->error_warning_limit;
    case TWAI_RX_ERROR_COUNT_OFF: return (uint8_t)s->rx_error_count;
    case TWAI_TX_ERROR_COUNT_OFF: return (uint8_t)s->tx_error_count;
    case TWAI_RX_MESSAGE_COUNT_OFF: return s->rx_count;
    case TWAI_CLOCK_DIVIDER_OFF: return s->clock_divider;
    default:
        if (off >= TWAI_BUFFER_OFF && off <= TWAI_BUFFER_OFF + 12u * 4u) {
            unsigned index = (off - TWAI_BUFFER_OFF) / 4u;
            if (s->mode & TWAI_MODE_RESET) {
                if (index < 4u) return s->acceptance_code[index];
                if (index < 8u) return s->acceptance_mask[index - 4u];
                return 0u;
            }
            if (s->rx_count != 0u)
                return s->rx_fifo[s->rx_head].bytes[index];
            return s->tx_buffer[index];
        }
        return 0u;
    }
}

static void twai_write_mode(esp32_periph_t *p, uint8_t value) {
    twai_state_t *s = &p->twai;
    bool was_reset = (s->mode & TWAI_MODE_RESET) != 0u;
    bool now_reset = (value & TWAI_MODE_RESET) != 0u;
    s->mode = value & TWAI_MODE_VALID_MASK;
    if (!was_reset && now_reset) {
        s->tx_busy = false;
        s->tx_complete = true;
        s->tx_event_armed = false;
        s->recovery_event_armed = false;
        twai_clear_rx_fifo(s);
    } else if (was_reset && !now_reset && s->bus_off) {
        twai_start_recovery(p);
    }
    twai_irq_update(p);
    twai_kick(p);
}

static void twai_write(void *ctx, uint32_t addr, uint32_t value) {
    esp32_periph_t *p = ctx;
    twai_state_t *s = &p->twai;
    uint32_t off = addr - TWAI_BASE;
    xtensa_cpu_t *cpu = twai_event_cpu(p);
    if (cpu) twai_eval_events(p, cpu);
    if ((off & 3u) != 0u || off >= TWAI_REG_FILE_SIZE) {
        default_write(ctx, addr, value);
        return;
    }
    uint8_t byte = (uint8_t)value;

    switch (off) {
    case TWAI_MODE_OFF:
        twai_write_mode(p, byte);
        return;
    case TWAI_COMMAND_OFF: {
        if (byte & TWAI_COMMAND_RELEASE_RX) twai_release_rx(p);
        if (byte & TWAI_COMMAND_CLEAR_OVERRUN) {
            s->data_overrun = false;
            s->int_raw &= (uint8_t)~TWAI_INT_DATA_OVERRUN;
        }
        if (byte & (TWAI_COMMAND_TX | TWAI_COMMAND_SELF_RX))
            twai_start_tx(p, byte);
        else if (byte & TWAI_COMMAND_ABORT)
            twai_abort_tx(p);
        twai_irq_update(p);
        return;
    }
    case TWAI_INTERRUPT_ENABLE_OFF:
        if (s->mode & TWAI_MODE_RESET) s->int_ena = byte;
        twai_irq_update(p);
        return;
    case TWAI_BUS_TIMING_0_OFF:
        if (s->mode & TWAI_MODE_RESET) s->bus_timing_0 = byte;
        return;
    case TWAI_BUS_TIMING_1_OFF:
        if (s->mode & TWAI_MODE_RESET) s->bus_timing_1 = byte;
        return;
    case TWAI_ERROR_WARNING_LIMIT_OFF:
        if (s->mode & TWAI_MODE_RESET) s->error_warning_limit = byte;
        return;
    case TWAI_RX_ERROR_COUNT_OFF:
        if (s->mode & TWAI_MODE_RESET) s->rx_error_count = byte;
        return;
    case TWAI_TX_ERROR_COUNT_OFF:
        if (s->mode & TWAI_MODE_RESET) s->tx_error_count = byte;
        return;
    case TWAI_CLOCK_DIVIDER_OFF:
        if (s->mode & TWAI_MODE_RESET)
            s->clock_divider = byte & 0x8Fu;
        return;
    case TWAI_STATUS_OFF:
    case TWAI_INTERRUPT_OFF:
    case TWAI_ARB_LOST_CAPTURE_OFF:
    case TWAI_ERROR_CODE_CAPTURE_OFF:
    case TWAI_RX_MESSAGE_COUNT_OFF:
        return;
    default:
        if (off >= TWAI_BUFFER_OFF && off <= TWAI_BUFFER_OFF + 12u * 4u) {
            unsigned index = (off - TWAI_BUFFER_OFF) / 4u;
            if (s->mode & TWAI_MODE_RESET) {
                if (index < 4u) s->acceptance_code[index] = byte;
                else if (index < 8u)
                    s->acceptance_mask[index - 4u] = byte;
            } else if (!s->tx_busy) {
                s->tx_buffer[index] = byte;
            }
        }
        return;
    }
}

/* ---- Classic ESP32 Ethernet MAC + enhanced descriptor DMA ---- */

typedef struct {
    uint32_t address;
    uint32_t status;
    uint32_t control;
    uint32_t buffer1;
    uint32_t buffer2_or_next;
    uint32_t next;
    uint16_t size1;
    uint16_t size2;
} emac_descriptor_t;

static bool emac_clocked(const esp32_periph_t *p) {
    return p && (p->dport_wifi_clk_en & DPORT_EMAC_CLK_BIT) != 0u &&
           (p->dport_core_rst_en & DPORT_EMAC_RST_BIT) == 0u;
}

static bool emac_dma_range_mapped(esp32_periph_t *p, uint32_t addr,
                                  size_t len, bool writable) {
    while (len != 0u) {
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

static uint32_t emac_descriptor_stride(const emac_state_t *s) {
    uint32_t bus_mode = s->dma[EMAC_DMA_BUS_MODE_OFF / 4u];
    uint32_t base = (bus_mode & EMAC_DMA_BUS_ENHANCED_DESC) ? 32u : 16u;
    return base + ((bus_mode & EMAC_DMA_BUS_DESC_SKIP_MASK) >> 2u) * 4u;
}

static uint32_t emac_next_descriptor(const emac_state_t *s,
                                     uint32_t address, uint32_t status,
                                     uint32_t control,
                                     uint32_t buffer2_or_next, bool tx) {
    uint32_t chained = tx ? EMAC_TX_DESC_CHAINED : EMAC_RX_DESC_CHAINED;
    uint32_t end_ring = tx ? EMAC_TX_DESC_END_RING : EMAC_RX_DESC_END_RING;
    uint32_t descriptor_control = tx ? status : control;
    if (descriptor_control & chained) return buffer2_or_next & ~3u;
    if (descriptor_control & end_ring) {
        uint32_t off = tx ? EMAC_DMA_TX_BASE_OFF : EMAC_DMA_RX_BASE_OFF;
        return s->dma[off / 4u] & ~3u;
    }
    return address + emac_descriptor_stride(s);
}

static bool emac_read_descriptor(esp32_periph_t *p, uint32_t address,
                                 bool tx, emac_descriptor_t *desc) {
    if (!desc || (address & 3u) != 0u ||
        !emac_dma_range_mapped(p, address, 16u, true))
        return false;
    desc->address = address;
    desc->status = mem_read32(p->mem, address);
    desc->control = mem_read32(p->mem, address + 4u);
    desc->buffer1 = mem_read32(p->mem, address + 8u);
    desc->buffer2_or_next = mem_read32(p->mem, address + 12u);
    desc->size1 = (uint16_t)(desc->control & EMAC_DESC_BUF1_SIZE_MASK);
    desc->size2 = (uint16_t)((desc->control & EMAC_DESC_BUF2_SIZE_MASK) >>
                             EMAC_DESC_BUF2_SIZE_SHIFT);
    desc->next = emac_next_descriptor(&p->emac, address, desc->status,
                                      desc->control,
                                      desc->buffer2_or_next, tx);
    return true;
}

static void emac_refresh_summaries(emac_state_t *s) {
    uint32_t enabled = s->dma[EMAC_DMA_INT_ENA_OFF / 4u];
    if ((enabled & EMAC_DMA_INT_NORMAL_SUMMARY) != 0u &&
        (s->dma_status & enabled & EMAC_DMA_ST_NORMAL_EVENTS) != 0u)
        s->dma_status |= EMAC_DMA_ST_NORMAL_SUMMARY;
    if ((enabled & EMAC_DMA_INT_ABNORMAL_SUMMARY) != 0u &&
        (s->dma_status & enabled & EMAC_DMA_ST_ABNORMAL_EVENTS) != 0u)
        s->dma_status |= EMAC_DMA_ST_ABNORMAL_SUMMARY;
}

static void emac_irq_update(esp32_periph_t *p) {
    emac_state_t *s = &p->emac;
    uint32_t enabled = s->dma[EMAC_DMA_INT_ENA_OFF / 4u];
    bool normal = (s->dma_status & EMAC_DMA_ST_NORMAL_SUMMARY) != 0u &&
                  (enabled & EMAC_DMA_INT_NORMAL_SUMMARY) != 0u &&
                  (s->dma_status & enabled &
                   EMAC_DMA_ST_NORMAL_EVENTS) != 0u;
    bool abnormal = (s->dma_status & EMAC_DMA_ST_ABNORMAL_SUMMARY) != 0u &&
                    (enabled & EMAC_DMA_INT_ABNORMAL_SUMMARY) != 0u &&
                    (s->dma_status & enabled &
                     EMAC_DMA_ST_ABNORMAL_EVENTS) != 0u;
    if (emac_clocked(p) && (normal || abnormal))
        periph_assert_interrupt_status(p, EMAC_INTR_SOURCE,
                                       s->dma_status & enabled);
    else
        periph_deassert_interrupt(p, EMAC_INTR_SOURCE);
}

static void emac_raise_status(esp32_periph_t *p, uint32_t status) {
    p->emac.dma_status |= status & (EMAC_DMA_ST_NORMAL_EVENTS |
                                    EMAC_DMA_ST_ABNORMAL_EVENTS);
    emac_refresh_summaries(&p->emac);
    emac_irq_update(p);
}

static void emac_reset_dma(esp32_periph_t *p) {
    emac_state_t *s = &p->emac;
    memset(s->dma, 0, sizeof(s->dma));
    s->dma_status = 0u;
    s->tx_current_desc = 0u;
    s->rx_current_desc = 0u;
    periph_deassert_interrupt(p, EMAC_INTR_SOURCE);
}

static void emac_reset_state(esp32_periph_t *p) {
    if (!p) return;
    emac_state_t *s = &p->emac;
    periph_emac_tx_fn tx_cb = s->tx_cb;
    void *tx_ctx = s->tx_cb_ctx;
    periph_emac_mdio_fn mdio_cb = s->mdio_cb;
    void *mdio_ctx = s->mdio_cb_ctx;
    uint16_t phy_regs[32][32];
    memcpy(phy_regs, s->phy_regs, sizeof(phy_regs));
    uint32_t phy_present = s->phy_present;
    memset(s, 0, sizeof(*s));
    s->tx_cb = tx_cb;
    s->tx_cb_ctx = tx_ctx;
    s->mdio_cb = mdio_cb;
    s->mdio_cb_ctx = mdio_ctx;
    memcpy(s->phy_regs, phy_regs, sizeof(phy_regs));
    s->phy_present = phy_present;
    s->mac[EMAC_MAC_CONFIG_OFF / 4u] = EMAC_MAC_CONFIG_MII;
    s->mac[EMAC_MAC_ADDR0_HIGH_OFF / 4u] = 1u << 31;
    s->ext[(EMAC_EXT_REG_FILE_SIZE - 4u) / 4u] = 0x15040200u;
    periph_deassert_interrupt(p, EMAC_INTR_SOURCE);
}

static void emac_dport_update(esp32_periph_t *p) {
    if (!p) return;
    if (!emac_clocked(p))
        periph_deassert_interrupt(p, EMAC_INTR_SOURCE);
    else
        emac_irq_update(p);
}

static void emac_fatal_bus_error(esp32_periph_t *p) {
    emac_raise_status(p, EMAC_DMA_ST_FATAL_BUS);
}

static uint32_t emac_dma_status_value(const emac_state_t *s) {
    uint32_t value = s->dma_status;
    uint32_t opmode = s->dma[EMAC_DMA_OPMODE_OFF / 4u];
    uint32_t rx_state = 0u;
    uint32_t tx_state = 0u;
    if (opmode & EMAC_DMA_OP_RX_START)
        rx_state = (s->dma_status & EMAC_DMA_ST_RX_UNAVAILABLE) ? 4u : 3u;
    if (opmode & EMAC_DMA_OP_TX_START)
        tx_state = (s->dma_status & EMAC_DMA_ST_TX_UNAVAILABLE) ? 6u : 3u;
    value |= rx_state << 17u;
    value |= tx_state << 20u;
    return value;
}

static uint32_t emac_crc32(const uint8_t *data, size_t len) {
    uint32_t crc = UINT32_MAX;
    for (size_t index = 0; index < len; index++) {
        uint8_t byte = data[index];
        for (unsigned bit = 0; bit < 8u; bit++) {
            uint32_t mix = (crc ^ byte) & 1u;
            crc >>= 1u;
            if (mix) crc ^= 0xEDB88320u;
            byte >>= 1u;
        }
    }
    return ~crc;
}

static bool emac_address_is_broadcast(const uint8_t *address) {
    for (unsigned index = 0; index < 6u; index++)
        if (address[index] != 0xFFu) return false;
    return true;
}

static bool emac_address_register_match(const emac_state_t *s,
                                        unsigned slot,
                                        const uint8_t *address,
                                        bool source) {
    uint32_t high_off = EMAC_MAC_ADDR0_HIGH_OFF + slot * 8u;
    uint32_t low_off = high_off + 4u;
    uint32_t high = s->mac[high_off / 4u];
    uint32_t low = s->mac[low_off / 4u];
    if (slot != 0u) {
        if ((high & (1u << 31)) == 0u) return false;
        if (((high & (1u << 30)) != 0u) != source) return false;
    } else if (source) {
        return false;
    }
    uint8_t expected[6] = {
        (uint8_t)low, (uint8_t)(low >> 8u),
        (uint8_t)(low >> 16u), (uint8_t)(low >> 24u),
        (uint8_t)high, (uint8_t)(high >> 8u),
    };
    uint32_t mask = slot == 0u ? 0u : (high >> 24u) & 0x3Fu;
    for (unsigned index = 0; index < 6u; index++)
        if ((mask & (1u << index)) == 0u &&
            expected[index] != address[index])
            return false;
    return true;
}

static bool emac_hash_match(const emac_state_t *s,
                            const uint8_t *address) {
    unsigned index = (unsigned)(emac_crc32(address, 6u) >> 26u);
    uint32_t word = index < 32u ?
        s->mac[EMAC_MAC_HASH_LOW_OFF / 4u] :
        s->mac[EMAC_MAC_HASH_HIGH_OFF / 4u];
    return (word & (1u << (index & 31u))) != 0u;
}

static bool emac_filter_frame(const emac_state_t *s, const uint8_t *frame,
                              size_t len, bool *destination_failed) {
    uint32_t filter = s->mac[EMAC_MAC_FRAME_FILTER_OFF / 4u];
    bool receive_all = (filter & EMAC_MAC_FILTER_RECEIVE_ALL) != 0u;
    if (destination_failed) *destination_failed = false;
    if (filter & EMAC_MAC_FILTER_PROMISCUOUS) return true;
    if (len < 12u) {
        if (destination_failed) *destination_failed = true;
        return receive_all;
    }

    const uint8_t *destination = frame;
    bool broadcast = emac_address_is_broadcast(destination);
    bool multicast = (destination[0] & 1u) != 0u && !broadcast;
    bool match = false;
    if (broadcast) {
        match = (filter & EMAC_MAC_FILTER_BLOCK_BCAST) == 0u;
    } else {
        for (unsigned slot = 0; slot < 8u && !match; slot++)
            match = emac_address_register_match(s, slot, destination,
                                                false);
        if (!match && multicast &&
            (filter & EMAC_MAC_FILTER_ALL_MULTICAST) != 0u)
            match = true;
        uint32_t hash_enable = multicast ?
            EMAC_MAC_FILTER_HASH_MULTICAST : EMAC_MAC_FILTER_HASH_UNICAST;
        if (!match && (filter & hash_enable) != 0u)
            match = emac_hash_match(s, destination);
        if (filter & EMAC_MAC_FILTER_DA_INVERSE) match = !match;
    }

    bool source_match = true;
    if (filter & EMAC_MAC_FILTER_SA_ENABLE) {
        source_match = false;
        for (unsigned slot = 1u; slot < 8u && !source_match; slot++)
            source_match = emac_address_register_match(s, slot, frame + 6u,
                                                       true);
        if (filter & EMAC_MAC_FILTER_SA_INVERSE)
            source_match = !source_match;
    }

    bool control_ok = true;
    if (len >= 14u && frame[12] == 0x88u && frame[13] == 0x08u)
        control_ok = (filter & EMAC_MAC_FILTER_CTRL_MASK) != 0u;
    bool passed = match && source_match && control_ok;
    if (destination_failed) *destination_failed = !match;
    return passed || receive_all;
}

static void emac_record_missed_frame(esp32_periph_t *p) {
    emac_state_t *s = &p->emac;
    uint32_t missed = s->dma[EMAC_DMA_MISSED_OFF / 4u];
    uint32_t count = missed & 0xFFFFu;
    if (count == 0xFFFFu) {
        missed |= 1u << 16;
        count = 0u;
    } else {
        count++;
    }
    s->dma[EMAC_DMA_MISSED_OFF / 4u] = (missed & ~0xFFFFu) | count;
}

static int emac_receive_frame(esp32_periph_t *p, const uint8_t *frame,
                              size_t len) {
    if (!p || !frame || len == 0u || len > EMAC_MAX_FRAME_SIZE ||
        !emac_clocked(p))
        return 0;
    emac_state_t *s = &p->emac;
    uint32_t opmode = s->dma[EMAC_DMA_OPMODE_OFF / 4u];
    uint32_t config = s->mac[EMAC_MAC_CONFIG_OFF / 4u];
    if ((opmode & EMAC_DMA_OP_RX_START) == 0u ||
        (config & EMAC_MAC_CONFIG_RX) == 0u ||
        (s->dma_status & EMAC_DMA_ST_FATAL_BUS) != 0u)
        return 0;

    bool destination_failed = false;
    if (!emac_filter_frame(s, frame, len, &destination_failed))
        return 0;

    emac_descriptor_t descriptors[EMAC_DMA_MAX_DESCRIPTORS];
    size_t descriptor_count = 0u;
    size_t remaining = len + 4u;
    uint32_t address = s->rx_current_desc;
    if (address == 0u)
        address = s->dma[EMAC_DMA_RX_BASE_OFF / 4u] & ~3u;

    while (remaining != 0u &&
           descriptor_count < EMAC_DMA_MAX_DESCRIPTORS) {
        emac_descriptor_t *desc = &descriptors[descriptor_count];
        if (address == 0u || !emac_read_descriptor(p, address, false, desc)) {
            emac_fatal_bus_error(p);
            return 0;
        }
        if ((desc->status & EMAC_DESC_OWN) == 0u) {
            emac_record_missed_frame(p);
            emac_raise_status(p, EMAC_DMA_ST_RX_UNAVAILABLE);
            return 0;
        }

        size_t capacity = desc->size1;
        bool chained = (desc->control & EMAC_RX_DESC_CHAINED) != 0u;
        if (!chained) capacity += desc->size2;
        if (capacity == 0u ||
            (desc->size1 != 0u &&
             !emac_dma_range_mapped(p, desc->buffer1, desc->size1, true)) ||
            (!chained && desc->size2 != 0u &&
             !emac_dma_range_mapped(p, desc->buffer2_or_next,
                                    desc->size2, true))) {
            emac_fatal_bus_error(p);
            return 0;
        }
        descriptor_count++;
        if (capacity >= remaining) remaining = 0u;
        else remaining -= capacity;
        if (remaining != 0u) {
            if (desc->next == 0u || desc->next == address) {
                emac_fatal_bus_error(p);
                return 0;
            }
            address = desc->next;
        }
    }
    if (remaining != 0u || descriptor_count == 0u) {
        emac_fatal_bus_error(p);
        return 0;
    }

    uint32_t fcs = emac_crc32(frame, len);
    size_t stream_pos = 0u;
    size_t total = len + 4u;
    for (size_t index = 0; index < descriptor_count; index++) {
        emac_descriptor_t *desc = &descriptors[index];
        uint32_t buffers[2] = {desc->buffer1, desc->buffer2_or_next};
        uint16_t sizes[2] = {desc->size1, desc->size2};
        unsigned buffer_count =
            (desc->control & EMAC_RX_DESC_CHAINED) ? 1u : 2u;
        for (unsigned buffer = 0; buffer < buffer_count; buffer++) {
            for (uint16_t byte = 0;
                 byte < sizes[buffer] && stream_pos < total;
                 byte++, stream_pos++) {
                uint8_t value;
                if (stream_pos < len) value = frame[stream_pos];
                else value = (uint8_t)(fcs >> ((stream_pos - len) * 8u));
                mem_write8(p->mem, buffers[buffer] + byte, value);
            }
        }
        uint32_t status = 0u;
        if (index == 0u) status |= EMAC_RX_DESC_FIRST;
        if (index + 1u == descriptor_count) {
            status |= EMAC_RX_DESC_LAST;
            status |= (uint32_t)(len + 4u) <<
                      EMAC_RX_DESC_FRAME_LEN_SHIFT;
            if (destination_failed) status |= EMAC_RX_DESC_DA_FILTER_FAIL;
            if (len >= 14u && frame[12] == 0x81u && frame[13] == 0x00u)
                status |= EMAC_RX_DESC_VLAN;
        }
        mem_write32(p->mem, desc->address, status);
    }

    emac_descriptor_t *last = &descriptors[descriptor_count - 1u];
    s->rx_current_desc = last->next;
    s->dma[EMAC_DMA_RX_CUR_DESC_OFF / 4u] = s->rx_current_desc;
    s->dma[EMAC_DMA_RX_CUR_BUF_OFF / 4u] = last->buffer1;
    if ((last->control & EMAC_RX_DESC_DISABLE_IRQ) == 0u)
        emac_raise_status(p, EMAC_DMA_ST_RX);
    return 1;
}

static bool emac_copy_tx_buffer(esp32_periph_t *p, uint32_t address,
                                size_t len, size_t *frame_len,
                                bool *too_large) {
    if (len == 0u) return true;
    if (!emac_dma_range_mapped(p, address, len, false)) return false;
    size_t available = *frame_len < EMAC_MAX_FRAME_SIZE ?
        EMAC_MAX_FRAME_SIZE - *frame_len : 0u;
    size_t copy = len < available ? len : available;
    for (size_t index = 0; index < copy; index++)
        p->emac.tx_frame[*frame_len + index] =
            mem_read8(p->mem, address + (uint32_t)index);
    *frame_len += copy;
    if (copy != len) *too_large = true;
    return true;
}

static void emac_process_tx(esp32_periph_t *p) {
    emac_state_t *s = &p->emac;
    uint32_t opmode = s->dma[EMAC_DMA_OPMODE_OFF / 4u];
    uint32_t config = s->mac[EMAC_MAC_CONFIG_OFF / 4u];
    if (!emac_clocked(p) || (opmode & EMAC_DMA_OP_TX_START) == 0u ||
        (config & EMAC_MAC_CONFIG_TX) == 0u ||
        (s->dma_status & EMAC_DMA_ST_FATAL_BUS) != 0u)
        return;

    uint32_t current = s->tx_current_desc;
    if (current == 0u)
        current = s->dma[EMAC_DMA_TX_BASE_OFF / 4u] & ~3u;
    unsigned total_seen = 0u;
    while (current != 0u && total_seen < EMAC_DMA_MAX_DESCRIPTORS) {
        emac_descriptor_t frame_desc[EMAC_DMA_MAX_DESCRIPTORS];
        size_t frame_desc_count = 0u;
        size_t frame_len = 0u;
        bool too_large = false;
        bool have_first = false;
        bool complete = false;
        bool interrupt_on_completion = false;
        uint32_t address = current;

        while (frame_desc_count < EMAC_DMA_MAX_DESCRIPTORS &&
               total_seen + frame_desc_count < EMAC_DMA_MAX_DESCRIPTORS) {
            emac_descriptor_t *desc = &frame_desc[frame_desc_count];
            if (!emac_read_descriptor(p, address, true, desc)) {
                emac_fatal_bus_error(p);
                return;
            }
            if ((desc->status & EMAC_DESC_OWN) == 0u) {
                if (frame_desc_count == 0u)
                    emac_raise_status(p, EMAC_DMA_ST_TX_UNAVAILABLE);
                return;
            }
            if (frame_desc_count == 0u)
                have_first = (desc->status & EMAC_TX_DESC_FIRST) != 0u;
            if (desc->status & EMAC_TX_DESC_IOC)
                interrupt_on_completion = true;
            if (!emac_copy_tx_buffer(p, desc->buffer1, desc->size1,
                                     &frame_len, &too_large)) {
                emac_fatal_bus_error(p);
                return;
            }
            if ((desc->status & EMAC_TX_DESC_CHAINED) == 0u &&
                !emac_copy_tx_buffer(p, desc->buffer2_or_next, desc->size2,
                                     &frame_len, &too_large)) {
                emac_fatal_bus_error(p);
                return;
            }
            frame_desc_count++;
            if (desc->status & EMAC_TX_DESC_LAST) {
                complete = true;
                break;
            }
            if (desc->next == 0u || desc->next == address) {
                emac_fatal_bus_error(p);
                return;
            }
            address = desc->next;
        }

        if (!complete) {
            emac_raise_status(p, EMAC_DMA_ST_TX_UNDERFLOW);
            return;
        }

        emac_descriptor_t *last = &frame_desc[frame_desc_count - 1u];
        bool valid = have_first && !too_large && frame_len != 0u;
        int wire_result = valid && s->tx_cb ?
            s->tx_cb(s->tx_cb_ctx, s->tx_frame, frame_len) : 0;
        for (size_t index = 0; index < frame_desc_count; index++) {
            uint32_t status = frame_desc[index].status & ~EMAC_DESC_OWN;
            if (index + 1u == frame_desc_count &&
                (!valid || wire_result != 0)) {
                status |= EMAC_TX_DESC_ERROR;
                if (wire_result != 0) status |= EMAC_TX_DESC_NO_CARRIER;
            }
            mem_write32(p->mem, frame_desc[index].address, status);
        }

        s->tx_current_desc = last->next;
        s->dma[EMAC_DMA_TX_CUR_DESC_OFF / 4u] = s->tx_current_desc;
        s->dma[EMAC_DMA_TX_CUR_BUF_OFF / 4u] = last->buffer1;
        if (interrupt_on_completion)
            emac_raise_status(p, EMAC_DMA_ST_TX);
        if (valid && wire_result == 0 &&
            (config & EMAC_MAC_CONFIG_LOOPBACK) != 0u)
            (void)emac_receive_frame(p, s->tx_frame, frame_len);

        total_seen += (unsigned)frame_desc_count;
        current = s->tx_current_desc;
        if (current == 0u) return;
        emac_descriptor_t next;
        if (!emac_read_descriptor(p, current, true, &next)) {
            emac_fatal_bus_error(p);
            return;
        }
        if ((next.status & EMAC_DESC_OWN) == 0u) {
            emac_raise_status(p, EMAC_DMA_ST_TX_UNAVAILABLE);
            return;
        }
    }
}

static void emac_mdio_transaction(esp32_periph_t *p, uint32_t command) {
    emac_state_t *s = &p->emac;
    uint8_t reg = (uint8_t)((command >> EMAC_MII_REG_SHIFT) & 0x1Fu);
    uint8_t phy = (uint8_t)((command >> EMAC_MII_PHY_SHIFT) & 0x1Fu);
    bool write = (command & EMAC_MII_WRITE) != 0u;
    uint16_t value = (uint16_t)s->mac[EMAC_MAC_MII_DATA_OFF / 4u];
    int result = 0;
    if (s->mdio_cb) {
        result = s->mdio_cb(s->mdio_cb_ctx, phy, reg, write, &value);
    } else if ((s->phy_present & (1u << phy)) != 0u) {
        if (write) {
            if (reg == 0u && (value & 0x8000u) != 0u)
                value &= (uint16_t)~0x8000u;
            s->phy_regs[phy][reg] = value;
        } else {
            value = s->phy_regs[phy][reg];
        }
    } else {
        result = -1;
    }
    if (!write)
        s->mac[EMAC_MAC_MII_DATA_OFF / 4u] =
            result == 0 ? value : 0xFFFFu;
    s->mac[EMAC_MAC_MII_ADDR_OFF / 4u] = command & ~EMAC_MII_BUSY;
}

static uint32_t emac_read(void *ctx, uint32_t addr) {
    esp32_periph_t *p = ctx;
    emac_state_t *s = &p->emac;
    if ((addr & 3u) != 0u) return default_read(ctx, addr);

    if (addr >= EMAC_DMA_BASE && addr < EMAC_EXT_BASE) {
        uint32_t off = addr - EMAC_DMA_BASE;
        if (off >= EMAC_DMA_REG_FILE_SIZE) return 0u;
        switch (off) {
        case EMAC_DMA_STATUS_OFF: return emac_dma_status_value(s);
        case EMAC_DMA_MISSED_OFF: {
            uint32_t value = s->dma[off / 4u];
            s->dma[off / 4u] = 0u;
            return value;
        }
        case EMAC_DMA_TX_CUR_DESC_OFF: return s->tx_current_desc;
        case EMAC_DMA_RX_CUR_DESC_OFF: return s->rx_current_desc;
        default: return s->dma[off / 4u];
        }
    }
    if (addr >= EMAC_EXT_BASE && addr < EMAC_DMA_BASE + PAGE_SIZE) {
        uint32_t off = addr - EMAC_EXT_BASE;
        if (off >= EMAC_EXT_REG_FILE_SIZE) return 0u;
        return s->ext[off / 4u];
    }
    if (addr >= EMAC_MAC_BASE && addr < EMAC_MAC_BASE + PAGE_SIZE) {
        uint32_t off = addr - EMAC_MAC_BASE;
        if (off >= EMAC_MAC_REG_FILE_SIZE) return 0u;
        switch (off) {
        case EMAC_MAC_DEBUG_OFF: return 0u;
        case EMAC_MAC_ADDR0_HIGH_OFF:
            return s->mac[off / 4u] | (1u << 31);
        case EMAC_MAC_STATUS_OFF: {
            uint32_t config = s->mac[EMAC_MAC_CONFIG_OFF / 4u];
            uint32_t speed = (config & EMAC_MAC_CONFIG_FAST_SPEED) ? 1u : 0u;
            return ((config & EMAC_MAC_CONFIG_DUPLEX) ? 1u : 0u) |
                   (speed << 1u);
        }
        default: return s->mac[off / 4u];
        }
    }
    return default_read(ctx, addr);
}

static void emac_dma_write(esp32_periph_t *p, uint32_t off,
                           uint32_t value) {
    emac_state_t *s = &p->emac;
    switch (off) {
    case EMAC_DMA_BUS_MODE_OFF:
        if (value & EMAC_DMA_BUS_SW_RESET) {
            emac_reset_dma(p);
            return;
        }
        s->dma[off / 4u] = value & ~EMAC_DMA_BUS_SW_RESET;
        return;
    case EMAC_DMA_TX_POLL_OFF:
        s->dma[off / 4u] = value;
        emac_process_tx(p);
        return;
    case EMAC_DMA_RX_POLL_OFF:
        s->dma[off / 4u] = value;
        return;
    case EMAC_DMA_RX_BASE_OFF:
        s->dma[off / 4u] = value & ~3u;
        s->rx_current_desc = value & ~3u;
        s->dma[EMAC_DMA_RX_CUR_DESC_OFF / 4u] = s->rx_current_desc;
        return;
    case EMAC_DMA_TX_BASE_OFF:
        s->dma[off / 4u] = value & ~3u;
        s->tx_current_desc = value & ~3u;
        s->dma[EMAC_DMA_TX_CUR_DESC_OFF / 4u] = s->tx_current_desc;
        return;
    case EMAC_DMA_STATUS_OFF:
        s->dma_status &= ~(value & EMAC_DMA_ST_EVENT_MASK);
        emac_refresh_summaries(s);
        emac_irq_update(p);
        return;
    case EMAC_DMA_OPMODE_OFF: {
        bool tx_was_stopped =
            (s->dma[off / 4u] & EMAC_DMA_OP_TX_START) == 0u;
        bool rx_was_stopped =
            (s->dma[off / 4u] & EMAC_DMA_OP_RX_START) == 0u;
        s->dma[off / 4u] = value & ~EMAC_DMA_OP_FLUSH_TX;
        if (rx_was_stopped && (value & EMAC_DMA_OP_RX_START)) {
            s->rx_current_desc = s->dma[EMAC_DMA_RX_BASE_OFF / 4u];
            s->dma[EMAC_DMA_RX_CUR_DESC_OFF / 4u] = s->rx_current_desc;
        }
        if (tx_was_stopped && (value & EMAC_DMA_OP_TX_START)) {
            s->tx_current_desc = s->dma[EMAC_DMA_TX_BASE_OFF / 4u];
            s->dma[EMAC_DMA_TX_CUR_DESC_OFF / 4u] = s->tx_current_desc;
            emac_process_tx(p);
        }
        return;
    }
    case EMAC_DMA_INT_ENA_OFF:
        s->dma[off / 4u] = value & 0x0001E7FFu;
        emac_refresh_summaries(s);
        emac_irq_update(p);
        return;
    case EMAC_DMA_MISSED_OFF:
    case EMAC_DMA_TX_CUR_DESC_OFF:
    case EMAC_DMA_RX_CUR_DESC_OFF:
    case EMAC_DMA_TX_CUR_BUF_OFF:
    case EMAC_DMA_RX_CUR_BUF_OFF:
        return;
    default:
        s->dma[off / 4u] = value;
        return;
    }
}

static void emac_write(void *ctx, uint32_t addr, uint32_t value) {
    esp32_periph_t *p = ctx;
    emac_state_t *s = &p->emac;
    if ((addr & 3u) != 0u) {
        default_write(ctx, addr, value);
        return;
    }
    if (addr >= EMAC_DMA_BASE && addr < EMAC_EXT_BASE) {
        uint32_t off = addr - EMAC_DMA_BASE;
        if (off < EMAC_DMA_REG_FILE_SIZE) emac_dma_write(p, off, value);
        return;
    }
    if (addr >= EMAC_EXT_BASE && addr < EMAC_DMA_BASE + PAGE_SIZE) {
        uint32_t off = addr - EMAC_EXT_BASE;
        if (off < EMAC_EXT_REG_FILE_SIZE) s->ext[off / 4u] = value;
        return;
    }
    if (addr >= EMAC_MAC_BASE && addr < EMAC_MAC_BASE + PAGE_SIZE) {
        uint32_t off = addr - EMAC_MAC_BASE;
        if (off >= EMAC_MAC_REG_FILE_SIZE) return;
        switch (off) {
        case EMAC_MAC_MII_ADDR_OFF:
            s->mac[off / 4u] = value;
            if (value & EMAC_MII_BUSY) emac_mdio_transaction(p, value);
            return;
        case EMAC_MAC_FLOW_CTRL_OFF:
            s->mac[off / 4u] = value & ~1u;
            return;
        case EMAC_MAC_DEBUG_OFF:
        case EMAC_MAC_INT_STATUS_OFF:
        case EMAC_MAC_STATUS_OFF:
            return;
        case EMAC_MAC_ADDR0_HIGH_OFF:
            s->mac[off / 4u] = value | (1u << 31);
            return;
        default:
            s->mac[off / 4u] = value;
            return;
        }
    }
    default_write(ctx, addr, value);
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
        periph_assert_interrupt_status(p, i2s_intr_sources[port],
                                       s->int_raw & s->int_ena &
                                       I2S_INT_VALID_MASK);
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
    periph_event_source_changed(p, PERIPH_EVENT_I2S);
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
    uint64_t now = periph_clock_now(p, &p->event_clock);
    uint64_t next = now + i2s_descriptor_cycles(s, tx, len ? len : 4u);
    if (tx) {
        s->next_tx_cycle = next;
        s->tx_event_armed = true;
    } else {
        s->next_rx_cycle = next;
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
    if (!p || !cpu) return UINT32_MAX;
    bool have = false;
    uint64_t best = 0;
    for (int port = 0; port < I2S_PORT_COUNT; port++) {
        i2s_state_t *s = &p->i2s[port];
        uint64_t events[2] = {s->next_tx_cycle, s->next_rx_cycle};
        bool armed[2] = {s->tx_event_armed, s->rx_event_armed};
        for (int direction = 0; direction < 2; direction++) {
            if (!armed[direction]) continue;
            if (!have || events[direction] < best) {
                have = true;
                best = events[direction];
            }
        }
    }
    if (!have) return UINT32_MAX;
    return periph_deadline_ccount(p, &p->event_clock, cpu, best);
}

static void i2s_eval_events(esp32_periph_t *p, xtensa_cpu_t *cpu) {
    if (!p || !cpu) return;
    uint64_t now_cycle = periph_clock_now(p, &p->event_clock);
    for (int port = 0; port < I2S_PORT_COUNT; port++) {
        i2s_state_t *s = &p->i2s[port];
        if (s->tx_event_armed && now_cycle >= s->next_tx_cycle) {
            s->tx_event_armed = false;
            (void)i2s_process_tx_descriptor(p, port);
            if (s->tx_active) i2s_arm_event(p, port, true);
        }
        if (s->rx_event_armed && now_cycle >= s->next_rx_cycle) {
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

/* ---- Target-described peripheral interrupt matrix ---- */

static bool intr_matrix_span_valid(uint32_t offset, uint32_t count,
                                   uint32_t register_size)
{
    if ((offset & 3u) != 0u || count == 0u ||
        count > UINT32_MAX / sizeof(uint32_t))
        return false;
    uint32_t bytes = count * (uint32_t)sizeof(uint32_t);
    return offset <= register_size && bytes <= register_size - offset;
}

static bool intr_matrix_geometry_valid(const flexe_target_desc_t *target)
{
    if (!target || !(target->capabilities &
                     FLEXE_TARGET_CAP_INTERRUPT_MATRIX_V1))
        return false;
    const flexe_interrupt_matrix_desc_t *desc =
        &target->interrupt_matrix;
    if (target->core_count == 0u ||
        target->core_count > FLEXE_TARGET_INTERRUPT_CORE_MAX ||
        desc->source_count == 0u ||
        desc->source_count > FLEXE_TARGET_INTERRUPT_SOURCE_MAX ||
        desc->register_size == 0u ||
        ((desc->base | desc->register_size) & 0xFFFu) != 0u ||
        desc->base < target->peripheral_start ||
        desc->base >= target->peripheral_end ||
        desc->register_size > target->peripheral_end - desc->base ||
        desc->map_writable_mask == 0u ||
        (desc->map_writable_mask & ~0x1Fu) != 0u ||
        (desc->map_reset & ~desc->map_writable_mask) != 0u ||
        desc->clock_gate_writable_mask == 0u ||
        (desc->clock_gate_reset &
         ~desc->clock_gate_writable_mask) != 0u ||
        (desc->date_reset & ~desc->date_writable_mask) != 0u)
        return false;

    uint32_t status_words = (desc->source_count + 31u) / 32u;
    for (unsigned core = 0; core < target->core_count; core++) {
        if (!intr_matrix_span_valid(desc->map_offset[core],
                                    desc->source_count,
                                    desc->register_size) ||
            !intr_matrix_span_valid(desc->status_offset[core],
                                    status_words,
                                    desc->register_size) ||
            !intr_matrix_span_valid(desc->clock_gate_offset[core], 1u,
                                    desc->register_size) ||
            !intr_matrix_span_valid(desc->date_offset[core], 1u,
                                    desc->register_size))
            return false;
    }

    if (desc->software_interrupt_count == 0u) return true;
    if (desc->software_interrupt_count >
            FLEXE_TARGET_SOFTWARE_INTERRUPT_MAX ||
        desc->software_interrupt_stride < sizeof(uint32_t) ||
        (desc->software_interrupt_stride & 3u) != 0u ||
        desc->software_interrupt_writable_mask == 0u ||
        desc->software_interrupt_source_base >= desc->source_count ||
        desc->software_interrupt_count >
            desc->source_count - desc->software_interrupt_source_base ||
        !(target->capabilities & FLEXE_TARGET_CAP_SECONDARY_CORE_CONTROL))
        return false;

    const flexe_secondary_core_desc_t *system = &target->secondary_core;
    if (desc->software_interrupt_base != system->base ||
        system->register_size < sizeof(uint32_t) ||
        desc->software_interrupt_offset >
            system->register_size - sizeof(uint32_t))
        return false;
    uint32_t last = (uint32_t)(desc->software_interrupt_count - 1u) *
                    desc->software_interrupt_stride;
    return last <= system->register_size - sizeof(uint32_t) -
                   desc->software_interrupt_offset;
}

static bool intr_matrix_decode_register(uint32_t off, uint32_t first,
                                        uint32_t count, unsigned *index)
{
    if (off < first || ((off - first) & 3u) != 0u) return false;
    uint32_t candidate = (off - first) / sizeof(uint32_t);
    if (candidate >= count) return false;
    *index = candidate;
    return true;
}

static uint32_t intr_matrix_status_word(const esp32_periph_t *p,
                                        unsigned core, unsigned word)
{
    uint32_t value = 0u;
    unsigned first = word * 32u;
    unsigned count = intr_matrix_source_count(p);
    for (unsigned bit = 0; bit < 32u && first + bit < count; bit++)
        if (p->source_level_core[core][first + bit]) value |= 1u << bit;
    return value;
}

static uint32_t target_intr_matrix_read(void *ctx, uint32_t addr)
{
    esp32_periph_t *p = ctx;
    const flexe_interrupt_matrix_desc_t *desc =
        &p->target->interrupt_matrix;
    uint32_t off = addr - desc->base;
    uint32_t status_words = (desc->source_count + 31u) / 32u;

    for (unsigned core = 0; core < intr_matrix_core_count(p); core++) {
        unsigned index;
        if (intr_matrix_decode_register(off, desc->map_offset[core],
                                        desc->source_count, &index))
            return p->intr_matrix[core][index];
        if (intr_matrix_decode_register(off, desc->status_offset[core],
                                        status_words, &index))
            return intr_matrix_status_word(p, core, index);
        if (off == desc->clock_gate_offset[core])
            return p->intr_matrix_clock_gate[core];
        if (off == desc->date_offset[core])
            return p->intr_matrix_date[core];
    }
    return default_read(ctx, addr);
}

static void target_intr_matrix_write(void *ctx, uint32_t addr,
                                     uint32_t value)
{
    esp32_periph_t *p = ctx;
    const flexe_interrupt_matrix_desc_t *desc =
        &p->target->interrupt_matrix;
    uint32_t off = addr - desc->base;
    uint32_t status_words = (desc->source_count + 31u) / 32u;

    for (unsigned core = 0; core < intr_matrix_core_count(p); core++) {
        unsigned index;
        if (intr_matrix_decode_register(off, desc->map_offset[core],
                                        desc->source_count, &index)) {
            intr_matrix_map_source(p, (int)core, (int)index,
                                   (int)(value & desc->map_writable_mask));
            return;
        }
        if (intr_matrix_decode_register(off, desc->status_offset[core],
                                        status_words, &index))
            return; /* Raw source status is read-only. */
        if (off == desc->clock_gate_offset[core]) {
            uint32_t next = value & desc->clock_gate_writable_mask;
            if (next != p->intr_matrix_clock_gate[core]) {
                p->intr_matrix_clock_gate[core] = next;
                intr_matrix_refresh_core(p, (int)core);
            }
            return;
        }
        if (off == desc->date_offset[core]) {
            p->intr_matrix_date[core] = value & desc->date_writable_mask;
            return;
        }
    }
    default_write(ctx, addr, value);
}

static int intr_matrix_software_interrupt_index(const esp32_periph_t *p,
                                                uint32_t addr)
{
    if (!p || !(p->target->capabilities &
                FLEXE_TARGET_CAP_INTERRUPT_MATRIX_V1))
        return -1;
    const flexe_interrupt_matrix_desc_t *desc =
        &p->target->interrupt_matrix;
    if (desc->software_interrupt_count == 0u ||
        addr < desc->software_interrupt_base)
        return -1;
    uint32_t off = addr - desc->software_interrupt_base;
    if (off < desc->software_interrupt_offset) return -1;
    off -= desc->software_interrupt_offset;
    if (off % desc->software_interrupt_stride != 0u) return -1;
    uint32_t index = off / desc->software_interrupt_stride;
    return index < desc->software_interrupt_count ? (int)index : -1;
}

static void intr_matrix_write_software_interrupt(esp32_periph_t *p,
                                                 unsigned index,
                                                 uint32_t value)
{
    const flexe_interrupt_matrix_desc_t *desc =
        &p->target->interrupt_matrix;
    value &= desc->software_interrupt_writable_mask;
    p->from_cpu_intr[index] = value;
    int source = (int)desc->software_interrupt_source_base + (int)index;
    if (value != 0u) periph_assert_interrupt(p, source);
    else             periph_deassert_interrupt(p, source);
}

/* ---- Target-described RTC controller and watchdog ---- */

static uint32_t target_rtc_cntl_next_fire(esp32_periph_t *p,
                                          xtensa_cpu_t *cpu)
{
    return p && p->target_rtc_cntl ?
        flexe_rtc_cntl_next_event(p->target_rtc_cntl, cpu) :
        UINT32_MAX;
}

static void target_rtc_cntl_eval_events(esp32_periph_t *p,
                                        xtensa_cpu_t *cpu)
{
    (void)cpu;
    if (p && p->target_rtc_cntl)
        flexe_rtc_cntl_eval(p->target_rtc_cntl);
}

static void target_rtc_cntl_state_changed(void *ctx)
{
    esp32_periph_t *p = ctx;
    if (!p) return;
    periph_event_source_changed(p, PERIPH_EVENT_RTC_CNTL);
    for (unsigned core = 0u; core < 2u; core++)
        if (p->cpu[core]) xtensa_recompute_next_timer(p->cpu[core]);
}

static void target_rtc_cntl_irq_changed(void *ctx, bool level)
{
    esp32_periph_t *p = ctx;
    if (!p || !(p->target->capabilities & FLEXE_TARGET_CAP_RTC_CNTL_V1))
        return;
    int source = (int)p->target->rtc_cntl.interrupt_source;
    if (level) periph_assert_interrupt(p, source);
    else periph_deassert_interrupt(p, source);
}

static void target_rtc_cntl_reset_requested(
    void *ctx, flexe_rtc_cntl_wdt_action_t action)
{
    esp32_periph_t *p = ctx;
    (void)action;
    if (p) p->reset_requested = true;
}

/* ---- Target-described system timer ---- */

static void system_clock_gate_changed(
    void *ctx, flexe_system_device_t device, unsigned instance,
    bool clock_enabled, bool reset_asserted)
{
    esp32_periph_t *p = ctx;
    if (!p) return;
    switch (device) {
    case FLEXE_SYSTEM_DEVICE_SYSTIMER:
        if (instance == 0u)
            flexe_systimer_set_system_state(
                p->systimer, clock_enabled, reset_asserted);
        return;
    case FLEXE_SYSTEM_DEVICE_TIMER_GROUP:
        flexe_timer_group_set_system_state(
            p->target_timer_group, instance,
            clock_enabled, reset_asserted);
        return;
    case FLEXE_SYSTEM_DEVICE_NONE:
        return;
    }
}

static uint32_t systimer_next_fire(esp32_periph_t *p, xtensa_cpu_t *cpu)
{
    return p && p->systimer ?
        flexe_systimer_next_event(p->systimer, cpu) : UINT32_MAX;
}

static void systimer_eval_events(esp32_periph_t *p, xtensa_cpu_t *cpu)
{
    (void)cpu;
    if (p && p->systimer) flexe_systimer_eval(p->systimer);
}

static void systimer_state_changed(void *ctx)
{
    esp32_periph_t *p = ctx;
    if (!p) return;
    periph_event_source_changed(p, PERIPH_EVENT_SYSTIMER);
    for (unsigned core = 0; core < 2u; core++)
        if (p->cpu[core]) xtensa_recompute_next_timer(p->cpu[core]);
}

static void systimer_irq_changed(void *ctx, unsigned alarm, bool level)
{
    esp32_periph_t *p = ctx;
    if (!p || alarm >= p->target->systimer.alarm_count) return;
    int source = p->target->systimer.interrupt_source[alarm];
    if (level) periph_assert_interrupt(p, source);
    else periph_deassert_interrupt(p, source);
}

/* ---- Target-described timer groups and main watchdogs ---- */

static uint32_t target_timer_group_next_fire(esp32_periph_t *p,
                                             xtensa_cpu_t *cpu)
{
    return p && p->target_timer_group ?
        flexe_timer_group_next_event(p->target_timer_group, cpu) :
        UINT32_MAX;
}

static void target_timer_group_eval_events(esp32_periph_t *p,
                                           xtensa_cpu_t *cpu)
{
    (void)cpu;
    if (p && p->target_timer_group)
        flexe_timer_group_eval(p->target_timer_group);
}

static void target_timer_group_state_changed(void *ctx)
{
    esp32_periph_t *p = ctx;
    if (!p) return;
    periph_event_source_changed(p, PERIPH_EVENT_TIMER_GROUP);
    for (unsigned core = 0u; core < 2u; core++)
        if (p->cpu[core]) xtensa_recompute_next_timer(p->cpu[core]);
}

static void target_timer_group_irq_changed(void *ctx, unsigned group,
                                           unsigned event, bool level)
{
    esp32_periph_t *p = ctx;
    if (!p || group >= p->target->timer_group.group_count ||
        event >= FLEXE_TARGET_TIMER_GROUP_EVENT_MAX)
        return;
    int source = p->target->timer_group.interrupt_source[group][event];
    if (level) periph_assert_interrupt(p, source);
    else periph_deassert_interrupt(p, source);
}

static void target_timer_group_reset_requested(
    void *ctx, unsigned group, flexe_timer_group_wdt_action_t action)
{
    esp32_periph_t *p = ctx;
    (void)group;
    (void)action;
    if (p) p->reset_requested = true;
}

static void usb_serial_jtag_irq_changed(void *ctx, bool level)
{
    esp32_periph_t *p = ctx;
    if (!p || !(p->target->capabilities &
                FLEXE_TARGET_CAP_USB_SERIAL_JTAG_V1))
        return;
    int source = (int)p->target->usb_serial_jtag.interrupt_source;
    if (level) periph_assert_interrupt(p, source);
    else periph_deassert_interrupt(p, source);
}

/* ---- Target-described secondary-core control ---- */

static bool secondary_core_geometry_valid(const flexe_target_desc_t *target)
{
    if (!target || !(target->capabilities &
                     FLEXE_TARGET_CAP_SECONDARY_CORE_CONTROL))
        return false;
    const flexe_secondary_core_desc_t *desc = &target->secondary_core;
    return target->core_count > 1u && desc->register_size >= 4u &&
           (desc->base & 0xFFFu) == 0u &&
           (desc->control_offset & 3u) == 0u &&
           (desc->boot_address_offset & 3u) == 0u &&
           desc->control_offset <= desc->register_size - 4u &&
           desc->boot_address_offset <= desc->register_size - 4u &&
           desc->base >= target->peripheral_start &&
           desc->base < target->peripheral_end &&
           desc->register_size <= target->peripheral_end - desc->base;
}

static void secondary_core_update_release(esp32_periph_t *p)
{
    const flexe_secondary_core_desc_t *desc = &p->target->secondary_core;
    uint32_t control = p->secondary_core_control;
    bool clocked = desc->clock_gate_mask == 0u ||
                   (control & desc->clock_gate_mask) != 0u;
    bool held = (control & (desc->reset_mask | desc->runstall_mask)) != 0u;
    p->app_cpu_in_reset = !clocked || held;
}

static uint32_t secondary_core_read(void *ctx, uint32_t addr)
{
    esp32_periph_t *p = ctx;
    const flexe_secondary_core_desc_t *desc = &p->target->secondary_core;
    uint32_t off = addr - desc->base;
    if (off == desc->control_offset) return p->secondary_core_control;
    if (off == desc->boot_address_offset)
        return p->secondary_core_boot_addr;
    int software = intr_matrix_software_interrupt_index(p, addr);
    if (software >= 0) return p->from_cpu_intr[software];
    return default_read(ctx, addr);
}

static void secondary_core_write(void *ctx, uint32_t addr, uint32_t value)
{
    esp32_periph_t *p = ctx;
    const flexe_secondary_core_desc_t *desc = &p->target->secondary_core;
    uint32_t off = addr - desc->base;
    if (off == desc->control_offset) {
        p->secondary_core_control = value &
            (desc->reset_mask | desc->clock_gate_mask | desc->runstall_mask);
        secondary_core_update_release(p);
        return;
    }
    if (off == desc->boot_address_offset) {
        p->secondary_core_boot_addr = value;
        return;
    }
    int software = intr_matrix_software_interrupt_index(p, addr);
    if (software >= 0) {
        intr_matrix_write_software_interrupt(p, (unsigned)software, value);
        return;
    }
    default_write(ctx, addr, value);
}

/* ---- Target-described RTC slow-clock calibration ---- */

static bool rtc_calibration_geometry_valid(const flexe_target_desc_t *target)
{
    if (!target || !(target->capabilities &
                     FLEXE_TARGET_CAP_RTC_CALIBRATION))
        return false;
    const flexe_rtc_calibration_desc_t *desc = &target->rtc_calibration;
    if (desc->group_count == 0u ||
        desc->group_count > FLEXE_TARGET_RTC_CAL_GROUP_MAX ||
        desc->register_size < 4u || desc->reference_clock_hz == 0u ||
        desc->cycles_shift >= 32u || desc->clock_select_shift >= 32u ||
        desc->result_shift >= 32u || desc->cycles_mask == 0u ||
        desc->clock_select_mask == 0u || desc->result_mask == 0u ||
        desc->start_mask == 0u || desc->ready_mask == 0u ||
        desc->timeout_mask == 0u ||
        (desc->config_offset & 3u) != 0u ||
        (desc->value_offset & 3u) != 0u ||
        (desc->timeout_offset & 3u) != 0u ||
        desc->config_offset > desc->register_size - 4u ||
        desc->value_offset > desc->register_size - 4u ||
        desc->timeout_offset > desc->register_size - 4u ||
        (desc->config_writable_mask & desc->ready_mask) != 0u ||
        (desc->timeout_writable_mask & desc->timeout_mask) != 0u ||
        (desc->config_reset & ~desc->config_writable_mask) != 0u ||
        (desc->timeout_reset &
         ~(desc->timeout_writable_mask | desc->timeout_mask)) != 0u)
        return false;

    uint32_t max_selector =
        desc->clock_select_mask >> desc->clock_select_shift;
    if (max_selector >= FLEXE_TARGET_RTC_CAL_CLOCK_MAX)
        return false;
    for (uint32_t i = 0; i <= max_selector; i++)
        if (desc->source_clock_hz[i] == 0u) return false;

    for (unsigned group = 0; group < desc->group_count; group++) {
        uint32_t base = desc->base[group];
        if ((base & 0xFFFu) != 0u ||
            base < target->peripheral_start ||
            base >= target->peripheral_end ||
            desc->register_size > target->peripheral_end - base)
            return false;
    }
    return true;
}

static uint32_t rtc_calibration_result(
    const flexe_rtc_calibration_desc_t *desc, uint32_t config)
{
    uint32_t cycles = (config & desc->cycles_mask) >> desc->cycles_shift;
    uint32_t selector = (config & desc->clock_select_mask) >>
                        desc->clock_select_shift;
    uint32_t source_hz = desc->source_clock_hz[selector];
    uint64_t numerator = (uint64_t)desc->reference_clock_hz * cycles;
    uint64_t count = (numerator + source_hz / 2u) / source_hz;
    uint32_t max_count = desc->result_mask >> desc->result_shift;
    if (count > max_count) count = max_count;
    return ((uint32_t)count << desc->result_shift) & desc->result_mask;
}

static void rtc_calibration_start(
    esp32_periph_t *p, unsigned group)
{
    const flexe_rtc_calibration_desc_t *desc =
        &p->target->rtc_calibration;
    target_rtc_cal_state_t *state = &p->target_rtc_cal[group];
    state->result = rtc_calibration_result(desc, state->config);
    state->reads_since = 0u;
    state->active = true;
    state->ready = false;
}

static int rtc_calibration_group(
    const flexe_rtc_calibration_desc_t *desc, uint32_t addr,
    uint32_t *offset)
{
    for (unsigned group = 0; group < desc->group_count; group++) {
        uint32_t base = desc->base[group];
        if (addr >= base && addr - base < desc->register_size) {
            *offset = addr - base;
            return (int)group;
        }
    }
    return -1;
}

static uint32_t rtc_calibration_read(void *ctx, uint32_t addr)
{
    esp32_periph_t *p = ctx;
    const flexe_rtc_calibration_desc_t *desc =
        &p->target->rtc_calibration;
    uint32_t off = 0u;
    int group = rtc_calibration_group(desc, addr, &off);
    if (group < 0) return default_read(ctx, addr);
    target_rtc_cal_state_t *state = &p->target_rtc_cal[group];

    if (off == desc->config_offset) {
        if (state->active && ++state->reads_since >= 2u) {
            state->active = false;
            state->ready = true;
        }
        return state->config | (state->ready ? desc->ready_mask : 0u);
    }
    if (off == desc->value_offset) return state->result;
    if (off == desc->timeout_offset)
        return state->timeout; /* Supported sources complete successfully. */
    return default_read(ctx, addr);
}

static void rtc_calibration_write(void *ctx, uint32_t addr, uint32_t value)
{
    esp32_periph_t *p = ctx;
    const flexe_rtc_calibration_desc_t *desc =
        &p->target->rtc_calibration;
    uint32_t off = 0u;
    int group = rtc_calibration_group(desc, addr, &off);
    if (group < 0) {
        default_write(ctx, addr, value);
        return;
    }
    target_rtc_cal_state_t *state = &p->target_rtc_cal[group];

    if (off == desc->config_offset) {
        uint32_t old = state->config;
        state->config = value & desc->config_writable_mask;
        uint32_t trigger_mask = desc->start_mask | desc->cycling_mask;
        bool rising = (state->config & trigger_mask & ~old) != 0u;
        if (rising) {
            rtc_calibration_start(p, (unsigned)group);
        } else if ((state->config & trigger_mask) == 0u) {
            state->active = false;
            state->ready = false;
            state->reads_since = 0u;
        }
        return;
    }
    if (off == desc->timeout_offset) {
        state->timeout = value & desc->timeout_writable_mask;
        return;
    }
    if (off == desc->value_offset) return; /* Read-only result. */
    default_write(ctx, addr, value);
}

/* ---- Public API ---- */

esp32_periph_t *periph_create(xtensa_mem_t *mem) {
    const flexe_target_desc_t *target = mem_target(mem);
    if (!mem || !target) return NULL;
    esp32_periph_t *p = calloc(1, sizeof(esp32_periph_t));
    if (!p) return NULL;
    p->mem = mem;
    p->target = target;
    p->app_cpu_in_reset = true;
    /* Every target starts with peripheral sources disconnected from CPU
     * lines. Target-specific interrupt-matrix models replace these routes as
     * firmware programs them. */
    memset(p->intr_matrix, DPORT_INTR_MAP_RESET, sizeof(p->intr_matrix));

    /* First install an explicit catch-all for this target's native MMIO
     * aperture. Individual device models replace only the pages they own. */
    for (uint32_t i = 0; i < mem->mmio_page_count; i++)
        mem_register_mmio(mem, (int)i, default_read, default_write, p);

    if (uart_register_target(p) != 0) {
        periph_destroy(p);
        return NULL;
    }

    if (target->capabilities & FLEXE_TARGET_CAP_SECONDARY_CORE_CONTROL) {
        if (!secondary_core_geometry_valid(target)) {
            periph_destroy(p);
            return NULL;
        }
        p->secondary_core_control = target->secondary_core.control_reset;
        secondary_core_update_release(p);
        if (mem_register_mmio_range(mem, target->secondary_core.base,
                                    target->secondary_core.register_size,
                                    secondary_core_read,
                                    secondary_core_write, p) != 0) {
            periph_destroy(p);
            return NULL;
        }
    }

    if (target->capabilities & FLEXE_TARGET_CAP_SYSTEM_CLOCK_V1) {
        mmio_read_fn fallback_read = default_read;
        mmio_write_fn fallback_write = default_write;
        if (target->capabilities &
            FLEXE_TARGET_CAP_SECONDARY_CORE_CONTROL) {
            const flexe_system_clock_desc_t *clock =
                &target->system_clock;
            const flexe_secondary_core_desc_t *core =
                &target->secondary_core;
            bool overlap = clock->base <= core->base ?
                clock->register_size > core->base - clock->base :
                core->register_size > clock->base - core->base;
            /* MMIO dispatch is page-granular. Shared owners must describe
             * the same aperture so fallback restoration cannot leave a
             * partially overwritten or dangling handler. */
            if (overlap && (clock->base != core->base ||
                            clock->register_size != core->register_size)) {
                periph_destroy(p);
                return NULL;
            }
            if (overlap) {
                fallback_read = secondary_core_read;
                fallback_write = secondary_core_write;
            }
        }
        p->system_clock = flexe_system_clock_create(
            mem, fallback_read, fallback_write, p,
            system_clock_gate_changed, p);
        if (!p->system_clock) {
            periph_destroy(p);
            return NULL;
        }
    }

    if (target->capabilities & FLEXE_TARGET_CAP_IO_MUX_V1) {
        p->io_mux = flexe_io_mux_create(
            mem, default_read, default_write, p);
        if (!p->io_mux) {
            periph_destroy(p);
            return NULL;
        }
    }

    if (target->capabilities & FLEXE_TARGET_CAP_INTERRUPT_MATRIX_V1) {
        if (!intr_matrix_geometry_valid(target)) {
            periph_destroy(p);
            return NULL;
        }
        const flexe_interrupt_matrix_desc_t *desc =
            &target->interrupt_matrix;
        memset(p->intr_matrix, (int)desc->map_reset,
               sizeof(p->intr_matrix));
        for (unsigned core = 0; core < target->core_count; core++) {
            p->intr_matrix_clock_gate[core] = desc->clock_gate_reset;
            p->intr_matrix_date[core] = desc->date_reset;
        }
        if (mem_register_mmio_range(mem, desc->base, desc->register_size,
                                    target_intr_matrix_read,
                                    target_intr_matrix_write, p) != 0) {
            periph_destroy(p);
            return NULL;
        }
    }

    if (target->capabilities & FLEXE_TARGET_CAP_RTC_CALIBRATION) {
        if (!rtc_calibration_geometry_valid(target)) {
            periph_destroy(p);
            return NULL;
        }
        const flexe_rtc_calibration_desc_t *desc =
            &target->rtc_calibration;
        for (unsigned group = 0; group < desc->group_count; group++) {
            target_rtc_cal_state_t *state = &p->target_rtc_cal[group];
            state->config = desc->config_reset;
            state->timeout = desc->timeout_reset &
                             desc->timeout_writable_mask;
            state->result = rtc_calibration_result(desc, state->config);
            if (state->config & desc->cycling_mask)
                rtc_calibration_start(p, group);
            if (mem_register_mmio_range(mem, desc->base[group],
                                        desc->register_size,
                                        rtc_calibration_read,
                                        rtc_calibration_write, p) != 0) {
                periph_destroy(p);
                return NULL;
            }
        }
    }

    if (target->capabilities & FLEXE_TARGET_CAP_TIMER_GROUP_V1) {
        mmio_read_fn fallback_read = default_read;
        mmio_write_fn fallback_write = default_write;
        if (target->capabilities & FLEXE_TARGET_CAP_RTC_CALIBRATION) {
            fallback_read = rtc_calibration_read;
            fallback_write = rtc_calibration_write;
        }
        p->target_timer_group = flexe_timer_group_create(
            mem, fallback_read, fallback_write, p,
            target_timer_group_state_changed, p,
            target_timer_group_irq_changed, p,
            target_timer_group_reset_requested, p);
        if (!p->target_timer_group) {
            periph_destroy(p);
            return NULL;
        }
    }

    if (target->capabilities & FLEXE_TARGET_CAP_REGI2C) {
        p->regi2c = flexe_regi2c_create(mem, default_read, default_write, p);
        if (!p->regi2c) {
            periph_destroy(p);
            return NULL;
        }
    }

    if (target->capabilities & FLEXE_TARGET_CAP_SENSITIVE_MEMPROT_V1) {
        p->sensitive_memprot = flexe_sensitive_memprot_create(
            mem, default_read, default_write, p);
        if (!p->sensitive_memprot) {
            periph_destroy(p);
            return NULL;
        }
    }

    if (target->capabilities & FLEXE_TARGET_CAP_SYSTIMER_V1) {
        p->systimer = flexe_systimer_create(
            mem, default_read, default_write, p,
            systimer_state_changed, p, systimer_irq_changed, p);
        if (!p->systimer) {
            periph_destroy(p);
            return NULL;
        }
    }

    if (target->capabilities & FLEXE_TARGET_CAP_USB_SERIAL_JTAG_V1) {
        p->usb_serial_jtag = flexe_usb_serial_jtag_create(
            mem, default_read, default_write, p,
            usb_serial_jtag_irq_changed, p);
        if (!p->usb_serial_jtag) {
            periph_destroy(p);
            return NULL;
        }
    }

    bool classic = (target->capabilities &
                    FLEXE_TARGET_CAP_ESP32_CLASSIC_PERIPHERALS) != 0u;
    if (!classic) {
        if (target->capabilities & FLEXE_TARGET_CAP_EFUSE_READ_V1)
            p->target_efuse = flexe_efuse_create(
                mem, default_read, default_write, p);
        if (target->capabilities & FLEXE_TARGET_CAP_RTC_CNTL_V1) {
            p->target_rtc_cntl = flexe_rtc_cntl_create(
                mem, default_read, default_write, p,
                target_rtc_cntl_state_changed, p,
                target_rtc_cntl_irq_changed, p,
                target_rtc_cntl_reset_requested, p);
            if (p->target_rtc_cntl)
                flexe_rtc_cntl_application_handoff(p->target_rtc_cntl);
        }
        if (target->flash_mmu.shared_instruction_data)
            p->shared_flash_mmu = flexe_flash_mmu_create(mem);
        if (target->capabilities & FLEXE_TARGET_CAP_ESP32S3_EXTMEM) {
            p->s3_extmem = flexe_esp32s3_extmem_create(mem);
            flexe_esp32s3_extmem_application_handoff(p->s3_extmem);
        }
        if (((target->capabilities & FLEXE_TARGET_CAP_EFUSE_READ_V1) &&
             !p->target_efuse) ||
            ((target->capabilities & FLEXE_TARGET_CAP_RTC_CNTL_V1) &&
             !p->target_rtc_cntl) ||
            (target->flash_mmu.shared_instruction_data &&
             !p->shared_flash_mmu) ||
            ((target->capabilities & FLEXE_TARGET_CAP_ESP32S3_EXTMEM) &&
             !p->s3_extmem)) {
            periph_destroy(p);
            return NULL;
        }
    }

    if (target->capabilities & FLEXE_TARGET_CAP_SPI_MEM) {
        p->spi_mem = flexe_spi_mem_create(
            mem, default_read, default_write, p,
            periph_flash_changed, p);
        if (!p->spi_mem) {
            periph_destroy(p);
            return NULL;
        }
    }

    flexe_system_clock_publish_gates(p->system_clock);

    if (!classic) return p;

    p->dport_wifi_clk_en = 0xFFFCE030u;
    /* Flexe loads an application image directly, after the second-stage
     * bootloader would have selected the SDK's default 160 MHz PLL clock.
     * Exposing XTAL reset state here made genuine Arduino APIs derive a
     * false 40 MHz APB clock even though g_ticks_per_us already held 160. */
    p->dport_cpu_per_conf = 1u;       /* CPUPERIOD_SEL_160 */
    p->rtc_cpu_period_conf = 1u << 30u;
    p->rtc_clk_conf = (1u << 27u) | 0x00002210u; /* PLL + reset dividers */
    p->radio.rng_state = 0x12345678ABCDEF01ULL;
    p->rtc_reset_cause = RTC_POWERON_RESET;
    /* RTC_SLOW_CLK_CAL_REG is STORE1, and the second-stage bootloader would
     * have calibrated it. Flexe loads an application image directly, so it
     * starts at zero -- and rtc_time_us_to_slowclk() divides by it, making
     * every timed sleep ask to wake at the time it started. The format is the
     * slow-clock period in microseconds, Q13.19. */
    p->rtc_store[1] = (uint32_t)((1000000ull << 19) / RTC_SLOW_CLK_HZ);
    /* An untouched pad reads *high* -- the count falls as capacitance rises.
     * Leaving these zeroed made every enabled pad look permanently pressed to
     * firmware the host never drives, which is the wrong default to hand a
     * ROM that merely happens to call touch_pad_init(). */
    for (int pad = 0; pad < SENS_TOUCH_PAD_COUNT; pad++)
        p->touch_value[pad] = 0xFFFFu;
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

    sigmadelta_reset_state(p);
    rmt_reset_state(p);
    pcnt_reset_state(p);

    /* Strapping-pin idle levels: GPIO0/5/15 have pull-ups enabled at reset
     * (GPIO2/12 pull-downs read low). Firmware reads GPIO_IN for buttons
     * tied to these pins (e.g. Marauder's BOOT-button on GPIO0) — leaving
     * them low looks like a permanently held button. */
    p->gpio.in = (1u << 0) | (1u << 5) | (1u << 15);

    /* General-purpose timers reset with count-up and auto-reload selected,
     * but remain stopped until firmware enables their group clock and Tx_EN. */
    timg_reset_group(p, 0);
    timg_reset_group(p, 1);

    /* Both legacy FRC timers reset disabled with count/load/alarm at zero. */
    frc_reset(p);

    /* Both UART DMA controllers reset to the documented H:5 defaults. */
    for (unsigned port = 0; port < UHCI_PORT_COUNT; port++)
        uhci_reset_state(p, port);

    /* External SDIO-slave endpoint and its two SLC descriptor engines. */
    sdio_slave_reset_state(p);

    /* Native SDMMC host reset state; cards can be attached after creation. */
    sdmmc_reset_state(p);

    /* Classic SJA1000-compatible TWAI controller starts in reset mode. */
    twai_reset_state(p);

    /* Classic DesignWare Ethernet MAC/DMA and RMII extension reset state. */
    emac_reset_state(p);

    /* LEDC reset state: all eight timers begin held in reset, and DATE is
     * the ESP32 peripheral version value from the vendor register map. */
    ledc_reset_state(p);

    /* Both motor-control PWM units have independent reset domains. */
    mcpwm_reset_unit(p, 0);
    mcpwm_reset_unit(p, 1);

    /* Bootloader-style initial flash MMU contents (app at flash 0x10000) */
    flash_mmu_init_bootloader(p);

    /* Override specific peripherals */
    /* DPORT control registers occupy page 0. Pages 1/2/3 are the independent
     * AES/RSA/SHA accelerators and must not be silently swallowed by this
     * handler (SHA installs its own MMIO model during session creation). */
    mem_register_mmio(mem, (int)PAGE_OF(DPORT_BASE),
                      dport_read, dport_write, p);
    /* Two adjacent 8 KiB DPORT flash-MMU register windows. */
    mem_register_mmio_range(mem, 0x3FF10000u, 0x4000u,
                            dport_read, dport_write, p);

    /* Dual UART DMA controllers (UHCI0/UHCI1, interrupt sources 12/13). */
    mem_register_mmio(mem, (int)PAGE_OF(UHCI0_BASE), uhci_read, uhci_write, p);
    mem_register_mmio(mem, (int)PAGE_OF(UHCI1_BASE), uhci_read, uhci_write, p);

    /* SDIO slave identity, shared host window, and SLC DMA (sources 10/11). */
    mem_register_mmio(mem, (int)PAGE_OF(HINF_BASE), hinf_read, hinf_write, p);
    mem_register_mmio(mem, (int)PAGE_OF(SLCHOST_BASE),
                      slchost_read, slchost_write, p);
    mem_register_mmio(mem, (int)PAGE_OF(SLC_BASE), slc_read, slc_write, p);

    /* Two classic ESP32 I2C controllers (interrupt sources 49/50). */
    mem_register_mmio(mem, (int)PAGE_OF(I2C0_BASE), i2c_read, i2c_write, p);
    mem_register_mmio(mem, (int)PAGE_OF(I2C1_BASE), i2c_read, i2c_write, p);

    /* Dual-slot native SD/MMC host (interrupt source 37). */
    mem_register_mmio(mem, (int)PAGE_OF(SDMMC_BASE),
                      sdmmc_read, sdmmc_write, p);

    /* Classic TWAI/CAN controller (interrupt source 45). */
    mem_register_mmio(mem, (int)PAGE_OF(TWAI_BASE),
                      twai_read, twai_write, p);

    /* Ethernet DMA/EXT share one page; MAC registers occupy the next page
     * (interrupt source 38). */
    mem_register_mmio(mem, (int)PAGE_OF(EMAC_DMA_BASE),
                      emac_read, emac_write, p);
    mem_register_mmio(mem, (int)PAGE_OF(EMAC_MAC_BASE),
                      emac_read, emac_write, p);

    /* GPIO: page 68 + page 69 (FUNC_OUT_SEL extends beyond 4096) */
    mem_register_mmio(mem, (int)PAGE_OF(GPIO_BASE), gpio_read, gpio_write, p);
    mem_register_mmio(mem, (int)PAGE_OF(GPIO_BASE) + 1, gpio_read, gpio_write, p);

    /* Legacy FRC1/FRC2 timers (interrupt sources 56/57). */
    mem_register_mmio(mem, (int)PAGE_OF(FRC_TIMER_BASE),
                      frc_read, frc_write, p);

    /* RTC_CNTL */
    mem_register_mmio(mem, (int)PAGE_OF(RTC_CNTL_BASE), rtc_cntl_read, rtc_cntl_write, p);

    /* SENS is at offset 0x800 within the RTC_CNTL page and is dispatched by
     * rtc_cntl_read/write. IO_MUX is installed from the target descriptor. */

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
    mem_register_mmio(mem, (int)PAGE_OF(BT_MAC_BASE),
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

    if (signal >= SIGMADELTA_SIGNAL_BASE &&
        signal < SIGMADELTA_SIGNAL_BASE + SIGMADELTA_CHANNEL_COUNT) {
        bool level = sigmadelta_channel_level(
            p, signal - SIGMADELTA_SIGNAL_BASE);
        if (route & (1u << 9u)) level = !level;
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
    if (!p || pin < 0) return -1;
    return flexe_io_mux_function(p->io_mux, (unsigned)pin);
}

void periph_destroy(esp32_periph_t *p) {
    if (!p) return;
    flexe_usb_serial_jtag_destroy(p->usb_serial_jtag);
    flexe_spi_mem_destroy(p->spi_mem);
    flexe_timer_group_destroy(p->target_timer_group);
    flexe_systimer_destroy(p->systimer);
    flexe_sensitive_memprot_destroy(p->sensitive_memprot);
    flexe_regi2c_destroy(p->regi2c);
    flexe_efuse_destroy(p->target_efuse);
    if (p->target->capabilities & FLEXE_TARGET_CAP_EFUSE_READ_V1)
        (void)mem_register_mmio_range(
            p->mem, p->target->efuse.base,
            p->target->efuse.register_size, NULL, NULL, NULL);
    flexe_rtc_cntl_destroy(p->target_rtc_cntl);
    if (p->target->capabilities & FLEXE_TARGET_CAP_RTC_CNTL_V1)
        (void)mem_register_mmio_range(
            p->mem, p->target->rtc_cntl.base,
            p->target->rtc_cntl.register_size, NULL, NULL, NULL);
    flexe_io_mux_destroy(p->io_mux);
    if (p->target->capabilities & FLEXE_TARGET_CAP_IO_MUX_V1)
        (void)mem_register_mmio_range(
            p->mem, p->target->io_mux.base,
            p->target->io_mux.register_size, NULL, NULL, NULL);
    flexe_system_clock_destroy(p->system_clock);
    if (p->target->capabilities & FLEXE_TARGET_CAP_SYSTEM_CLOCK_V1)
        (void)mem_register_mmio_range(
            p->mem, p->target->system_clock.base,
            p->target->system_clock.register_size, NULL, NULL, NULL);
    if (p->target->capabilities & FLEXE_TARGET_CAP_RTC_CALIBRATION) {
        const flexe_rtc_calibration_desc_t *desc =
            &p->target->rtc_calibration;
        for (unsigned group = 0; group < desc->group_count; group++)
            (void)mem_register_mmio_range(
                p->mem, desc->base[group], desc->register_size,
                NULL, NULL, NULL);
    }
    if (p->target->capabilities & FLEXE_TARGET_CAP_SECONDARY_CORE_CONTROL)
        (void)mem_register_mmio_range(
            p->mem, p->target->secondary_core.base,
            p->target->secondary_core.register_size, NULL, NULL, NULL);
    if (p->target->capabilities & FLEXE_TARGET_CAP_INTERRUPT_MATRIX_V1)
        (void)mem_register_mmio_range(
            p->mem, p->target->interrupt_matrix.base,
            p->target->interrupt_matrix.register_size, NULL, NULL, NULL);
    flexe_esp32s3_extmem_destroy(p->s3_extmem);
    flexe_flash_mmu_destroy(p->shared_flash_mmu);
    periph_disable_spi_display(p);
    for (int port = 0; port < I2C_PORT_COUNT; port++)
        free(p->i2c[port].pending_write);
    free(p->sdmmc.transfer);
    free(p);
}

int periph_sdmmc_attach_card(esp32_periph_t *p, int slot,
                             uint32_t sector_count,
                             periph_sdmmc_read_blocks_fn read_fn,
                             periph_sdmmc_write_blocks_fn write_fn,
                             void *ctx) {
    if (!p || slot < 0 || slot >= (int)SDMMC_SLOT_COUNT ||
        (read_fn && sector_count < 1024u))
        return -1;
    sdmmc_card_state_t *card = &p->sdmmc.card[slot];
    bool was_attached = card->attached;
    memset(card, 0, sizeof(*card));
    if (read_fn) {
        card->attached = true;
        card->sector_count = sector_count;
        card->read_fn = read_fn;
        card->write_fn = write_fn;
        card->ctx = ctx;
        card->rca = (uint16_t)(slot + 1);
        card->block_len = 512u;
    }
    if (p->sdmmc.transfer_active &&
        p->sdmmc.transfer_slot == (uint8_t)slot)
        sdmmc_cancel_transfer(&p->sdmmc);
    if (was_attached != card->attached)
        p->sdmmc.rintsts |= SDMMC_INT_CARD_DETECT;
    sdmmc_irq_update(p);
    return 0;
}

int periph_sdmmc_set_write_protected(esp32_periph_t *p, int slot,
                                     bool write_protected) {
    if (!p || slot < 0 || slot >= (int)SDMMC_SLOT_COUNT ||
        !p->sdmmc.card[slot].attached)
        return -1;
    p->sdmmc.card[slot].write_protected = write_protected;
    return 0;
}

int periph_set_twai_tx_callback(esp32_periph_t *p, periph_twai_tx_fn fn,
                                void *ctx) {
    if (!p) return -1;
    p->twai.tx_cb = fn;
    p->twai.tx_cb_ctx = fn ? ctx : NULL;
    return 0;
}

int periph_twai_rx_inject(esp32_periph_t *p,
                          const periph_twai_frame_t *frame) {
    if (!p) return 0;
    return twai_enqueue_rx(p, frame, true);
}

size_t periph_twai_rx_pending(const esp32_periph_t *p) {
    return p ? p->twai.rx_count : 0u;
}

int periph_set_emac_tx_callback(esp32_periph_t *p, periph_emac_tx_fn fn,
                                void *ctx) {
    if (!p) return -1;
    p->emac.tx_cb = fn;
    p->emac.tx_cb_ctx = fn ? ctx : NULL;
    return 0;
}

int periph_set_emac_mdio_callback(esp32_periph_t *p,
                                  periph_emac_mdio_fn fn, void *ctx) {
    if (!p) return -1;
    p->emac.mdio_cb = fn;
    p->emac.mdio_cb_ctx = fn ? ctx : NULL;
    return 0;
}

int periph_emac_phy_set_reg(esp32_periph_t *p, uint8_t phy_address,
                            uint8_t reg, uint16_t value) {
    if (!p || phy_address >= 32u || reg >= 32u) return -1;
    p->emac.phy_regs[phy_address][reg] = value;
    p->emac.phy_present |= 1u << phy_address;
    return 0;
}

int periph_emac_phy_get_reg(const esp32_periph_t *p, uint8_t phy_address,
                            uint8_t reg, uint16_t *value) {
    if (!p || !value || phy_address >= 32u || reg >= 32u ||
        (p->emac.phy_present & (1u << phy_address)) == 0u)
        return -1;
    *value = p->emac.phy_regs[phy_address][reg];
    return 0;
}

int periph_emac_rx_inject(esp32_periph_t *p, const uint8_t *frame,
                          size_t len) {
    return emac_receive_frame(p, frame, len);
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

void periph_set_usb_serial_jtag_callback(esp32_periph_t *p,
                                         uart_tx_cb cb, void *ctx) {
    if (!p) return;
    flexe_usb_serial_jtag_set_tx_callback(p->usb_serial_jtag, cb, ctx);
}

size_t periph_usb_serial_jtag_tx_count(const esp32_periph_t *p) {
    return p ? flexe_usb_serial_jtag_tx_count(p->usb_serial_jtag) : 0u;
}

const uint8_t *periph_usb_serial_jtag_tx_buf(const esp32_periph_t *p) {
    return p ? flexe_usb_serial_jtag_tx_buf(p->usb_serial_jtag) : NULL;
}

size_t periph_usb_serial_jtag_rx_inject(esp32_periph_t *p,
                                        const uint8_t *data, size_t len) {
    return p ? flexe_usb_serial_jtag_rx_inject(
        p->usb_serial_jtag, data, len) : 0u;
}

size_t periph_usb_serial_jtag_rx_pending(const esp32_periph_t *p) {
    return p ? flexe_usb_serial_jtag_rx_pending(p->usb_serial_jtag) : 0u;
}

void periph_usb_serial_jtag_set_connected(esp32_periph_t *p,
                                          bool connected) {
    if (p) flexe_usb_serial_jtag_set_connected(
        p->usb_serial_jtag, connected);
}

bool periph_usb_serial_jtag_connected(const esp32_periph_t *p) {
    return p ? flexe_usb_serial_jtag_connected(p->usb_serial_jtag) : false;
}

void periph_usb_serial_jtag_host_sof(esp32_periph_t *p) {
    if (p) flexe_usb_serial_jtag_host_sof(p->usb_serial_jtag);
}

void periph_set_uart_callback_num(esp32_periph_t *p, int uart_num,
                                  uart_tx_cb cb, void *ctx) {
    if (!uart_num_valid(p, uart_num)) return;
    p->uart[uart_num].cb = cb;
    p->uart[uart_num].cb_ctx = ctx;
}

int periph_uart_tx_count_num(const esp32_periph_t *p, int uart_num) {
    if (!uart_num_valid(p, uart_num)) return 0;
    return p->uart[uart_num].tx_len;
}

const uint8_t *periph_uart_tx_buf_num(const esp32_periph_t *p,
                                      int uart_num) {
    if (!uart_num_valid(p, uart_num)) return NULL;
    return p->uart[uart_num].tx;
}

size_t periph_uart_rx_inject_num(esp32_periph_t *p, int uart_num,
                                 const uint8_t *data, size_t len) {
    if (!uart_num_valid(p, uart_num) ||
        (!data && len != 0)) return 0;
    const flexe_uart_ip_desc_t *ip = &p->target->uart_ip;
    uart_state_t *uart = &p->uart[uart_num];
    size_t dma_accepted = uhci_uart_rx_feed(p, uart_num, data, len, true);
    size_t accepted = dma_accepted;
    while (accepted < len && uart->rx_count < UART_RX_FIFO_SIZE) {
        uart->rx[uart->rx_head] = data[accepted++];
        uart->rx_head = (uint16_t)((uart->rx_head + 1) % UART_RX_FIFO_SIZE);
        uart->rx_count++;
    }

    /* Host injection represents already-arrived bytes, so publish the
     * target-described timeout condition at once for short packets; larger
     * bursts also assert the FIFO threshold. */
    uint32_t conf1 = uart->shadow[0x24 / 4];
    uart_refresh_level_conditions(p, uart_num);
    if (accepted > dma_accepted &&
        (conf1 & ip->rx_timeout_enable_mask))
        uart->int_raw |= UART_RXFIFO_TOUT_INT;
    if (accepted < len)
        uart->int_raw |= UART_RXFIFO_OVF_INT;
    uart_intr_update(p, uart_num);
    return accepted;
}

bool periph_uart_rx_break_num(esp32_periph_t *p, int uart_num) {
    if (!uart_num_valid(p, uart_num)) return false;
    return uhci_uart_rx_break(p, uart_num);
}

size_t periph_uart_rx_inject(esp32_periph_t *p, const uint8_t *data,
                             size_t len) {
    return periph_uart_rx_inject_num(p, 0, data, len);
}

size_t periph_uart_rx_pending_num(const esp32_periph_t *p, int uart_num) {
    if (!uart_num_valid(p, uart_num)) return 0;
    return p->uart[uart_num].rx_count;
}

size_t periph_uart_rx_pending(const esp32_periph_t *p) {
    return periph_uart_rx_pending_num(p, 0);
}

int periph_i2c_attach_device(esp32_periph_t *p, int port, uint8_t address,
                             periph_i2c_device_fn fn, void *ctx) {
    if (!p || port < 0 || port > PERIPH_I2C_PORT_RTC ||
        address >= I2C_DEVICE_COUNT)
        return -1;
    i2c_device_t *device = port == PERIPH_I2C_PORT_RTC ?
                           &p->rtc_i2c.device[address] :
                           &p->i2c[port].device[address];
    device->fn = fn;
    device->ctx = fn ? ctx : NULL;
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

int periph_set_sigmadelta_output_callback(
    esp32_periph_t *p, int channel, periph_sigmadelta_output_fn fn,
    void *ctx) {
    if (!p || channel < 0 ||
        channel >= (int)SIGMADELTA_CHANNEL_COUNT)
        return -1;
    sigmadelta_channel_state_t *state =
        &p->sigmadelta.channel[channel];
    state->output_cb = fn;
    state->output_cb_ctx = fn ? ctx : NULL;
    state->output_reported = false;
    if (fn) sigmadelta_emit_channel(p, (unsigned)channel, true);
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
    if (!p || source < 0 ||
        (unsigned)source >= intr_matrix_source_count(p))
        return -1;
    p->irq_dispatch[source] = fn;
    p->irq_dispatch_ctx[source] = fn ? ctx : NULL;
    return 0;
}

bool periph_interrupt_pending(const esp32_periph_t *p, int source) {
    if (!p || source < 0 ||
        (unsigned)source >= intr_matrix_source_count(p))
        return false;
    return (p->pending_sources[source / 32] &
            (1u << (source % 32))) != 0;
}

/* Hand a pending sleep to the session, which owns the clock and the reset
 * path. `timeout_us` is how long the armed RTC timer has left, or
 * PERIPH_SLEEP_FOREVER when no timer is armed; `cause` is any wake source
 * already satisfied at entry, in which case the sleep is over immediately. */
bool periph_take_sleep_request(esp32_periph_t *p, bool *deep,
                               uint64_t *timeout_us, uint32_t *cause)
{
    if (!p || !p->sleep_requested) return false;
    p->sleep_requested = false;
    if (deep) *deep = p->sleep_deep;

    uint32_t ena = p->rtc_wakeup_ena;
    uint64_t us = PERIPH_SLEEP_FOREVER;
    if (ena & RTC_TIMER_TRIG_EN) {
        /* Measure the interval in the guest's own units, not ours.
         *
         * timer_wakeup_prepare() computes its target as "the value I just read
         * out of TIME0/TIME1, plus the requested delay converted through the
         * calibration in RTC_SLOW_CLK_CAL_REG". Both of those are the guest's
         * numbers: it re-calibrates the slow clock at startup and writes its
         * own answer into STORE1. Comparing the target against a tick count
         * derived independently here compounds every disagreement between the
         * two -- it produced a 50 ms sleep that asked to end before it began.
         * Taking the difference from the same reading the guest used, and
         * converting with the same calibration, makes the sleep exactly as
         * long as the firmware intended however the clock is modelled. */
        uint64_t ticks = (p->rtc_slp_target > p->rtc_time_latched)
                       ? p->rtc_slp_target - p->rtc_time_latched : 0;
        uint32_t cal = p->rtc_store[1];   /* slow-clock period, Q13.19 us */
        if (cal == 0) cal = (uint32_t)((1000000ull << 19) / RTC_SLOW_CLK_HZ);
        us = (ticks * cal) >> 19;
    }
    if (timeout_us) *timeout_us = us;
    if (cause) *cause = rtc_wake_condition(p);
    return true;
}

/* Poll the level-triggered wake sources while time is being stepped forward. */
uint32_t periph_sleep_poll_wake(esp32_periph_t *p)
{
    return p ? rtc_wake_condition(p) : 0;
}

/* Record why the chip woke and release the guest's wait. Light sleep resumes
 * inside rtc_sleep_start()'s poll on SLP_WAKEUP; deep sleep resets instead,
 * and the session carries the cause across through periph_set_wake_state(). */
void periph_finish_wake(esp32_periph_t *p, uint32_t cause)
{
    if (!p) return;
    p->rtc_wakeup_cause = cause & RTC_CNTL_WAKEUP_ENA_MASK;
    p->rtc_state0 &= ~RTC_CNTL_SLEEP_EN_BIT;
    p->rtc_state0 |= RTC_CNTL_SLP_WAKEUP_BIT;
    p->rtc_int_raw |= RTC_CNTL_SLP_WAKEUP_INT_BIT;
    rtc_irq_update(p);
}

void periph_set_wake_state(esp32_periph_t *p, uint32_t wake_cause,
                           uint32_t reset_cause)
{
    if (!p) return;
    p->rtc_wakeup_cause = wake_cause & RTC_CNTL_WAKEUP_ENA_MASK;
    p->rtc_reset_cause = reset_cause & 0x3Fu;
}

uint32_t periph_reset_cause(const esp32_periph_t *p)
{
    return p ? p->rtc_reset_cause : RTC_POWERON_RESET;
}

bool periph_take_reset_request(esp32_periph_t *p)
{
    if (!p || !p->reset_requested) return false;
    p->reset_requested = false;
    return true;
}

int periph_unhandled_count(const esp32_periph_t *p) {
    return p ? p->unhandled_count : 0;
}

uint32_t periph_app_cpu_boot_addr(const esp32_periph_t *p) {
    return p ? p->secondary_core_boot_addr : 0u;
}

bool periph_app_cpu_released(const esp32_periph_t *p) {
    return p ? !p->app_cpu_in_reset : false;
}

void periph_attach_cpus(esp32_periph_t *p, xtensa_cpu_t *cpu0, xtensa_cpu_t *cpu1) {
    if (!p) return;
    p->cpu[0] = cpu0;
    p->cpu[1] = cpu1;
    flexe_flash_mmu_attach_cpus(p->shared_flash_mmu, cpu0, cpu1);
    flexe_esp32s3_extmem_attach_cpus(p->s3_extmem, cpu0, cpu1);
    flexe_systimer_attach_cpus(p->systimer, cpu0, cpu1);
    flexe_timer_group_attach_cpus(p->target_timer_group, cpu0, cpu1);
    flexe_rtc_cntl_attach_cpus(p->target_rtc_cntl, cpu0, cpu1);

    bool classic = (p->target->capabilities &
                    FLEXE_TARGET_CAP_ESP32_CLASSIC_PERIPHERALS) != 0u;
    uint32_t candidates = classic ?
        (PERIPH_EVENT_ALL_MASK &
         ~((1u << PERIPH_EVENT_RTC_CNTL) |
           (1u << PERIPH_EVENT_SYSTIMER) |
           (1u << PERIPH_EVENT_TIMER_GROUP))) : 0u;
    if (p->target_rtc_cntl)
        candidates |= 1u << PERIPH_EVENT_RTC_CNTL;
    if (p->systimer) candidates |= 1u << PERIPH_EVENT_SYSTIMER;
    if (p->target_timer_group)
        candidates |= 1u << PERIPH_EVENT_TIMER_GROUP;
    p->event_source_registered_mask = candidates;
    p->event_source_candidates[0] = candidates;
    p->event_source_candidates[1] = candidates;

    if (classic) {
        /* Re-anchor every shared clock against the newly attached cores.
         * Listing them here rather than open-coding each one keeps a clock
         * added later from being silently left un-anchored. */
        periph_clock_t *clocks[] = {
            &p->timg_clock, &p->mcpwm.clock, &p->sigmadelta.clock,
            &p->ledc_clock, &p->event_clock,
        };
        for (unsigned core = 0; core < 2u; core++) {
            xtensa_cpu_t *cpu = core == 0u ? cpu0 : cpu1;
            for (size_t i = 0; i < sizeof(clocks) / sizeof(clocks[0]); i++) {
                clocks[i]->core_cycles[core] = clocks[i]->cycles;
                clocks[i]->last_ccount[core] = cpu ? cpu->ccount : 0u;
                clocks[i]->valid[core] = cpu != NULL;
            }
        }
        for (unsigned group = 0; group < 2u; group++) {
            p->lact[group].last_cycles = p->timg_clock.cycles;
            for (unsigned timer = 0; timer < TIMG_TIMER_COUNT; timer++)
                p->timg[group].timer[timer].last_cycles =
                    p->timg_clock.cycles;
        }
        for (unsigned timer = 0; timer < FRC_TIMER_COUNT; timer++)
            p->frc_timer[timer].last_cycles = p->timg_clock.cycles;
        uhci_dport_update(p);
    }

    for (unsigned core = 0; core < intr_matrix_core_count(p); core++) {
        if (!p->cpu[core]) continue;
        for (unsigned source = 0;
             source < intr_matrix_source_count(p); source++) {
            if (p->source_level_core[core][source])
                intr_matrix_refresh_cpu_line(
                    p, (int)core, p->intr_matrix[core][source]);
        }
    }

    /* Wire every available timed device into each execution engine so its
     * alarms participate in WAITI fast-forwarding and interrupt delivery. */
    for (int i = 0; i < 2; i++) {
        xtensa_cpu_t *c = i == 0 ? cpu0 : cpu1;
        if (!c || candidates == 0u) continue;
        c->periph_event_ctx = p;
        c->periph_next_event = periph_next_event_hook;
        c->periph_event = periph_event_hook;
    }
}

void periph_assert_interrupt(esp32_periph_t *p, int source) {
    if (!p || source < 0 ||
        (unsigned)source >= intr_matrix_source_count(p))
        return;
    p->pending_sources[source / 32] |= (1u << (source % 32));
    intr_matrix_update_source(p, source, true);
}

void periph_assert_interrupt_status(esp32_periph_t *p, int source,
                                    uint32_t status) {
    if (!p || source < 0 ||
        (unsigned)source >= intr_matrix_source_count(p))
        return;
    p->pending_sources[source / 32] |= (1u << (source % 32));

    bool was = p->source_level[source];
    uint32_t fresh = status & ~p->source_status[source];
    p->source_status[source] = status;

    intr_matrix_update_source(p, source, true);

    /* Dispatch on a *new* condition, independently of the electrical edge
     * that intr_matrix_update_source() already reported. The two are
     * different signals -- "the line went high" and "there is work the
     * handler has not seen" -- so the first assert after a deassert calls the
     * handler twice. That is deliberate and load-bearing: ESP-IDF's I2C slave
     * driver does not drain a staged transfer on a single invocation, and
     * gating this on the line having already been high (the obvious
     * simplification) reproduces the original bug, with guest_got=0 in
     * the i2c_slave fixture gate. An unchanged mask still dispatches nothing,
     * which keeps this safe on the every-register-write path. */
    (void)was;
    if (fresh && p->irq_dispatch[source])
        p->irq_dispatch[source](p->irq_dispatch_ctx[source], source);
}

void periph_deassert_interrupt(esp32_periph_t *p, int source) {
    if (!p || source < 0 ||
        (unsigned)source >= intr_matrix_source_count(p))
        return;
    p->pending_sources[source / 32] &= ~(1u << (source % 32));
    p->source_status[source] = 0;
    intr_matrix_update_source(p, source, false);
}

void periph_pad_hold_snapshot(const esp32_periph_t *p, periph_pad_hold_t *out)
{
    if (!out) return;
    memset(out, 0, sizeof(*out));
    if (!p) return;

    for (int ch = 0; ch < RTC_GPIO_CHANNELS; ch++) {
        if (!rtcio_channel_held(p, ch)) continue;
        out->hold_mask |= 1u << ch;
        /* Carry the pad word itself, so the hold bit is still set after the
         * restore and the pad stays held until firmware clears it. */
        uint16_t off = RTCIO_HOLD[ch].off;
        unsigned k;
        for (k = 0; k < out->reg_count; k++)
            if (out->reg_off[k] == off) break;
        if (k == out->reg_count && out->reg_count < 8u) {
            out->reg_off[k] = off;
            out->regs[k] = p->rtcio_regs[off / 4u];
            out->reg_count++;
        }
    }
    if (!out->hold_mask) return;
    out->rtcio_out    = p->rtcio_regs[RTC_GPIO_OUT_OFF / 4u];
    out->rtcio_enable = p->rtcio_regs[RTC_GPIO_ENABLE_OFF / 4u];
}

void periph_pad_hold_restore(esp32_periph_t *p, const periph_pad_hold_t *in)
{
    if (!p || !in || !in->hold_mask) return;

    /* Put the hold bits back first: rtcio_publish_pads() consults them, and
     * restoring the levels while the pads read as unheld would let a later
     * write clobber exactly what was being preserved. */
    for (unsigned k = 0; k < in->reg_count; k++)
        p->rtcio_regs[in->reg_off[k] / 4u] = in->regs[k];

    uint32_t keep = in->hold_mask << RTC_GPIO_DATA_S;
    p->rtcio_regs[RTC_GPIO_OUT_OFF / 4u] =
        (p->rtcio_regs[RTC_GPIO_OUT_OFF / 4u] & ~keep) | (in->rtcio_out & keep);
    p->rtcio_regs[RTC_GPIO_ENABLE_OFF / 4u] =
        (p->rtcio_regs[RTC_GPIO_ENABLE_OFF / 4u] & ~keep) |
        (in->rtcio_enable & keep);

    /* Drive the pins directly. rtcio_publish_pads() would skip these, since
     * they are held -- which is correct for a guest write and wrong here. */
    uint32_t out = in->rtcio_out >> RTC_GPIO_DATA_S;
    uint32_t en  = in->rtcio_enable >> RTC_GPIO_DATA_S;
    for (int ch = 0; ch < RTC_GPIO_CHANNELS; ch++) {
        if (!(in->hold_mask & (1u << ch))) continue;
        int gpio = RTCIO_CHANNEL_GPIO[ch];
        uint32_t mask = (gpio < 32) ? (1u << gpio) : (1u << (gpio - 32));
        uint32_t *g_out = (gpio < 32) ? &p->gpio.out : &p->gpio.out1;
        uint32_t *g_en  = (gpio < 32) ? &p->gpio.enable : &p->gpio.enable1;
        if (en & (1u << ch)) {
            *g_en |= mask;
            if (out & (1u << ch)) *g_out |= mask; else *g_out &= ~mask;
        }
    }
}

void periph_intr_matrix_set(esp32_periph_t *p, int core, int cpu_int, int source) {
    if (!p || source < 0 ||
        (unsigned)source >= intr_matrix_source_count(p))
        return;
    intr_matrix_map_source(p, core, source, cpu_int);
}

int periph_intr_matrix_get(const esp32_periph_t *p, int core, int cpu_int) {
    if (!p || core < 0 || (unsigned)core >= intr_matrix_core_count(p) ||
        cpu_int < 0 || cpu_int > 31)
        return (int)intr_matrix_map_reset(p);
    if (!intr_matrix_cpu_line_is_routeable(cpu_int))
        return (int)intr_matrix_map_reset(p);
    for (unsigned source = 0;
         source < intr_matrix_source_count(p); source++) {
        if (p->intr_matrix[core][source] == (uint8_t)cpu_int)
            return (int)source;
    }
    return (int)intr_matrix_map_reset(p);
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

/* Inject a touch-pad reading. Counts fall as capacitance rises, so a "touched"
 * pad is one whose value is below its threshold; the gate drives it that way.
 * A scan runs immediately when the FSM is on, which is what lets a host-side
 * touch raise the interrupt without the guest polling for it. */
void periph_touch_set_value(esp32_periph_t *p, int pad, uint32_t value) {
    if (!p || pad < 0 || pad >= SENS_TOUCH_PAD_COUNT) return;
    p->touch_value[pad] = (uint16_t)value;
    if (touch_fsm_running(p))
        touch_run_measurement(p);
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
        if (gpio_dbg())
            fprintf(stderr, "[GPIO] pin%d intr (type=%u ena=0x%X level=%d)\n",
                    pin, int_type, (cfg >> 13) & 0x1Fu, now);
    }
    gpio_intr_update(p);
    if (fire && gpio_dbg()) {
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
