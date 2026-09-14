/*
 * target.h -- Versioned descriptions of ESP32-family execution targets.
 *
 * Keep image identification and architectural geometry out of firmware-
 * specific loaders and hooks. A descriptor being present means Flexe can
 * identify the target; support_level says whether it can execute it yet.
 */

#ifndef FLEXE_TARGET_H
#define FLEXE_TARGET_H

#include <stdbool.h>
#include <stdint.h>

#define FLEXE_TARGET_EXEC_RANGE_MAX 5u
#define FLEXE_TARGET_MEM_REGION_MAX 10u
#define FLEXE_TARGET_UART_MAX 3
#define FLEXE_TARGET_I2C_MAX 2u
#define FLEXE_TARGET_RTC_CAL_GROUP_MAX 2u
#define FLEXE_TARGET_RTC_CAL_CLOCK_MAX 4u
#define FLEXE_TARGET_REGI2C_HOST_MAX 2u
#define FLEXE_TARGET_REGI2C_AUX_REGISTER_MAX 8u
#define FLEXE_TARGET_SYSTIMER_COUNTER_MAX 2u
#define FLEXE_TARGET_SYSTIMER_ALARM_MAX 3u
#define FLEXE_TARGET_TIMER_GROUP_MAX 2u
#define FLEXE_TARGET_TIMER_GROUP_TIMER_MAX 2u
#define FLEXE_TARGET_TIMER_GROUP_EVENT_MAX 3u
#define FLEXE_TARGET_SPI_MEM_HOST_MAX 2u
#define FLEXE_TARGET_GP_SPI_HOST_MAX 2u
#define FLEXE_TARGET_GP_SPI_CS_MAX 6u
#define FLEXE_TARGET_INTERRUPT_CORE_MAX 2u
#define FLEXE_TARGET_INTERRUPT_SOURCE_MAX 128u
#define FLEXE_TARGET_SOFTWARE_INTERRUPT_MAX 4u
#define FLEXE_TARGET_GPIO_MAX 54u
#define FLEXE_TARGET_IO_MUX_REGISTER_MAX 64u
#define FLEXE_TARGET_IO_MUX_OFFSET_NONE UINT16_MAX
#define FLEXE_TARGET_RTC_STORE_MAX 8u
#define FLEXE_TARGET_RTC_SEQUENCE_REGISTER_MAX 6u
#define FLEXE_TARGET_RTC_IO_PIN_MAX 22u
#define FLEXE_TARGET_RTC_WDT_STAGE_MAX 4u
#define FLEXE_TARGET_RTC_WDT_CONFIG_MAX \
    (FLEXE_TARGET_RTC_WDT_STAGE_MAX + 1u)
#define FLEXE_TARGET_EFUSE_READ_WORD_MAX 96u
#define FLEXE_TARGET_SENS_ADC_UNIT_MAX 2u
#define FLEXE_TARGET_SYSTEM_REGISTER_MAX 7u
#define FLEXE_TARGET_SYSTEM_GATE_MAX 13u
#define FLEXE_TARGET_RADIO_WINDOW_MAX 10u
#define FLEXE_TARGET_RADIO_COMPLETION_MAX 4u
#define FLEXE_TARGET_GDMA_CHANNEL_MAX 5u
#define FLEXE_TARGET_SHA_MODE_MAX 8u
#define FLEXE_SPI_MEM_CS_NONE UINT8_MAX
#define FLEXE_TARGET_GPIO_NONE UINT8_MAX
#define FLEXE_TARGET_GDMA_PERIPHERAL_NONE UINT8_MAX
#define FLEXE_TARGET_MATRIX_SIGNAL_NONE UINT16_MAX
#define FLEXE_TARGET_DESCRIPTOR_VERSION 43u

/* Device-model capabilities are architectural properties of a target, not
 * guesses derived from a firmware image. Keep each bit tied to a reusable IP
 * model so machine construction remains data-driven as the family grows. */
typedef enum {
    FLEXE_TARGET_CAP_ESP32_CLASSIC_PERIPHERALS = 1ull << 0,
    FLEXE_TARGET_CAP_ESP32S3_EXTMEM             = 1ull << 1,
    FLEXE_TARGET_CAP_DIRECT_ROM_DATA_INIT       = 1ull << 2,
    FLEXE_TARGET_CAP_SECONDARY_CORE_CONTROL     = 1ull << 3,
    FLEXE_TARGET_CAP_RTC_CALIBRATION            = 1ull << 4,
    FLEXE_TARGET_CAP_REGI2C                     = 1ull << 5,
    FLEXE_TARGET_CAP_SENSITIVE_MEMPROT_V1       = 1ull << 6,
    FLEXE_TARGET_CAP_SYSTIMER_V1                = 1ull << 7,
    FLEXE_TARGET_CAP_SPI_MEM                    = 1ull << 8,
    FLEXE_TARGET_CAP_INTERRUPT_MATRIX_V1        = 1ull << 9,
    FLEXE_TARGET_CAP_USB_SERIAL_JTAG_V1          = 1ull << 10,
    FLEXE_TARGET_CAP_TIMER_GROUP_V1              = 1ull << 11,
    FLEXE_TARGET_CAP_SYSTEM_CLOCK_V1             = 1ull << 12,
    FLEXE_TARGET_CAP_IO_MUX_V1                   = 1ull << 13,
    FLEXE_TARGET_CAP_RTC_CNTL_V1                  = 1ull << 14,
    FLEXE_TARGET_CAP_EFUSE_READ_V1                = 1ull << 15,
    FLEXE_TARGET_CAP_GPIO_V1                      = 1ull << 16,
    FLEXE_TARGET_CAP_I2C_V1                       = 1ull << 17,
    FLEXE_TARGET_CAP_SENS_V1                      = 1ull << 18,
    FLEXE_TARGET_CAP_RADIO_REGS_V1                = 1ull << 19,
    FLEXE_TARGET_CAP_GDMA_V1                      = 1ull << 20,
    FLEXE_TARGET_CAP_SHA_V1                       = 1ull << 21,
    FLEXE_TARGET_CAP_ROM_FLASH_HANDOFF            = 1ull << 22,
    FLEXE_TARGET_CAP_GP_SPI                       = 1ull << 23,
    FLEXE_TARGET_CAP_RMT_V1                        = 1ull << 24,
    FLEXE_TARGET_CAP_APB_SARADC_V1                = 1ull << 25,
    FLEXE_TARGET_CAP_RTC_IO_V1                    = 1ull << 26,
    FLEXE_TARGET_CAP_LEDC_V1                      = 1ull << 27,
} flexe_target_capability_t;

typedef enum {
    FLEXE_TARGET_AUTO = 0,
    FLEXE_TARGET_ESP32,
    FLEXE_TARGET_ESP32S3,
} flexe_target_id_t;

typedef enum {
    FLEXE_XTENSA_LX6 = 6,
    FLEXE_XTENSA_LX7 = 7,
} flexe_xtensa_generation_t;

typedef enum {
    FLEXE_TARGET_UNAVAILABLE = 0,
    FLEXE_TARGET_EXPERIMENTAL,
    FLEXE_TARGET_STABLE,
} flexe_target_support_t;

typedef struct {
    uint32_t start;
    uint32_t end;   /* exclusive */
} flexe_addr_range_t;

/* Host allocations used by the target's memory map. Multiple guest regions
 * may reference the same backing (for example S3 D/IRAM aliases). */
typedef enum {
    FLEXE_MEM_SRAM = 0,
    FLEXE_MEM_ROM,
    FLEXE_MEM_FLASH_DATA,
    FLEXE_MEM_FLASH_INSN,
    FLEXE_MEM_RTC_FAST,
    FLEXE_MEM_RTC_SLOW,
    FLEXE_MEM_PSRAM,
    FLEXE_MEM_BACKING_COUNT,
} flexe_mem_backing_t;

typedef struct {
    uint32_t              start;
    uint32_t              end;       /* exclusive */
    flexe_mem_backing_t   backing;
    uint32_t              backing_offset;
    const char           *name;
} flexe_target_mem_region_t;

typedef struct {
    uint32_t page_size;
    uint16_t entry_count;
    uint32_t linear_addr_mask;
    uint32_t table_base[2];
    uint32_t invalid_entry;
    uint32_t invalid_mask;
    uint32_t physical_page_mask;
    uint32_t target_mask;
    bool     shared_instruction_data;
} flexe_flash_mmu_desc_t;

/* UART register geometry shared by the functional UART model. The ESP32 and
 * ESP32-S3 use closely related UART IP, but details after the common FIFO and
 * interrupt registers differ. Keeping those differences here lets another
 * target reuse the device without inheriting either chip's address map. */
typedef struct {
    uint32_t register_size;
    uint32_t interrupt_valid_mask;
    uint32_t interrupt_raw_reset;
    uint32_t status_idle_value;
    uint32_t rx_full_threshold_mask;
    uint32_t rx_timeout_enable_mask;
    uint32_t mem_rx_status_offset;
    uint32_t fifo_address_mask;
    uint8_t  mem_rx_read_shift;
    uint8_t  mem_rx_write_shift;
    uint32_t date_offset;
    uint32_t date_reset;
} flexe_uart_ip_desc_t;

typedef struct {
    uint32_t base;
    uint32_t interrupt_source;
} flexe_uart_instance_desc_t;

/* External I2C controller front end. ESP32 and ESP32-S3 retain the same
 * FIFO/register positions, but the S3 moves each instance, shortens the
 * command list, changes command opcodes, and replaces several interrupt
 * meanings. Keep those architectural differences out of the bus model. */
typedef struct {
    uint32_t base;
    uint8_t  interrupt_source;
} flexe_i2c_instance_desc_t;

typedef struct {
    uint32_t register_size;
    uint32_t date_reset;
    uint32_t interrupt_valid_mask;
    uint32_t interrupt_rxfifo_full_mask;
    uint32_t interrupt_txfifo_empty_mask;
    uint32_t interrupt_rxfifo_overflow_mask;
    uint32_t interrupt_end_detect_mask;
    uint32_t interrupt_slave_complete_mask;
    uint32_t interrupt_command_done_mask;
    uint32_t interrupt_transaction_complete_mask;
    uint32_t interrupt_transaction_start_mask;
    uint32_t interrupt_nack_mask;
    uint8_t  instance_count;
    uint8_t  command_count;
    uint8_t  opcode_restart;
    uint8_t  opcode_write;
    uint8_t  opcode_read;
    uint8_t  opcode_stop;
    uint8_t  opcode_end;
    flexe_i2c_instance_desc_t instance[FLEXE_TARGET_I2C_MAX];
} flexe_i2c_desc_t;

/* Register interface used to start a target's secondary CPU. Bit positions
 * and register locations vary by SoC even when the Xtensa boot contract does
 * not, so machine construction consumes this descriptor rather than addresses
 * from firmware or target-name checks. */
typedef struct {
    uint32_t base;
    uint32_t register_size;
    uint32_t control_offset;
    uint32_t boot_address_offset;
    uint32_t control_reset;
    uint32_t reset_mask;
    uint32_t clock_gate_mask;
    uint32_t runstall_mask;
} flexe_secondary_core_desc_t;

typedef enum {
    FLEXE_SYSTEM_DEVICE_NONE = 0,
    FLEXE_SYSTEM_DEVICE_SYSTIMER,
    FLEXE_SYSTEM_DEVICE_TIMER_GROUP,
    FLEXE_SYSTEM_DEVICE_I2C,
    FLEXE_SYSTEM_DEVICE_GP_SPI,
    FLEXE_SYSTEM_DEVICE_SHA,
    FLEXE_SYSTEM_DEVICE_LEDC,
    FLEXE_SYSTEM_DEVICE_UART,
    FLEXE_SYSTEM_DEVICE_USB_SERIAL_JTAG,
} flexe_system_device_t;

typedef struct {
    uint16_t offset;
    uint32_t reset;
    uint32_t writable_mask;
} flexe_system_register_desc_t;

/* A semantic device mapping turns clock/reset bits into actual model state.
 * Unmapped writable fields retain their architectural readback but report a
 * diagnostic when changed, so extending the register bank cannot silently
 * imply that an unimplemented peripheral or power effect works. */
typedef struct {
    flexe_system_device_t device;
    uint8_t  instance;
    uint16_t clock_offset;
    uint16_t reset_offset;
    uint32_t clock_mask;
    uint32_t reset_mask;
} flexe_system_gate_desc_t;

/* CPU/system-clock and peripheral clock/reset registers used by S2/S3-style
 * clock trees. Register geometry and device mappings are target data; the
 * reusable owner preserves register state and publishes exact gate edges.
 * CPU-clock propagation into target-cycle timing remains a separate,
 * calibrated-mode concern. */
typedef struct {
    uint32_t base;
    uint32_t register_size;
    uint32_t cpu_per_conf_offset;
    uint32_t cpu_per_conf_reset;
    uint32_t cpu_per_conf_writable_mask;
    uint32_t sysclk_conf_offset;
    uint32_t sysclk_conf_reset;
    uint32_t sysclk_conf_writable_mask;
    uint8_t register_count;
    uint8_t gate_count;
    flexe_system_register_desc_t
        reg[FLEXE_TARGET_SYSTEM_REGISTER_MAX];
    flexe_system_gate_desc_t gate[FLEXE_TARGET_SYSTEM_GATE_MAX];
} flexe_system_clock_desc_t;

/* Digital pad configuration register file. GPIO-to-register routing belongs
 * to the target because package pin maps differ even when the IO_MUX fields
 * are shared by an IP generation. */
typedef struct {
    uint32_t base;
    uint32_t register_size;
    uint8_t  register_count;
    uint8_t  gpio_count;
    uint8_t  function_shift;
    uint32_t control_offset;
    uint32_t control_reset;
    uint32_t control_writable_mask;
    uint32_t register_reset;
    uint32_t register_writable_mask;
    uint32_t function_mask;
    uint32_t date_offset;
    uint32_t date_reset;
    uint16_t gpio_register_offset[FLEXE_TARGET_GPIO_MAX];
} flexe_io_mux_desc_t;

/* S2/S3-generation digital GPIO matrix. The capability selects the reusable
 * register layout while target data supplies package-valid pads, virtual
 * strap state, constant matrix inputs, and interrupt-source wiring. GPIO
 * registers exist for more indices than every package bonds out, hence the
 * separate valid mask and exclusive gpio_count upper bound. */
typedef struct {
    uint32_t base;
    uint32_t register_size;
    uint64_t valid_gpio_mask;
    uint32_t strap_reset;
    uint32_t date_reset;
    uint8_t  gpio_count;
    uint8_t  matrix_const_one_input;
    uint8_t  matrix_const_zero_input;
    uint8_t  interrupt_source;
    uint8_t  nmi_interrupt_source;
} flexe_gpio_desc_t;

/* RTC power-sequencer register, with explicit reset and writable fields. */
typedef struct {
    uint16_t offset;
    uint32_t reset;
    uint32_t writable_mask;
} flexe_rtc_sequence_register_desc_t;

/* Always-on RTC controller state shared by the ROM, bootloader and
 * application. Offsets are explicit because the register layout, timer
 * width, interrupt bank, watchdog, and routed source vary across the ESP32
 * family, while older chips also split STORE0..3 and STORE4..7 into separate
 * parts of RTC_CNTL. */
typedef struct {
    uint32_t base;
    uint32_t register_size;
    uint8_t  store_count;
    uint8_t  slow_clock_cal_store;
    uint8_t  xtal_frequency_store;
    uint32_t slow_clock_hz;
    uint32_t xtal_frequency_mhz;
    uint16_t store_offset[FLEXE_TARGET_RTC_STORE_MAX];
    uint32_t store_reset[FLEXE_TARGET_RTC_STORE_MAX];
    uint16_t time_update_offset;
    uint16_t time_low_offset;
    uint16_t time_high_offset;
    uint16_t reset_state_offset;
    /* CPU software stall uses a two-bit low field in OPTIONS0 and a
     * six-bit high field in another register. The combined value 0x86
     * stalls the corresponding core. Zero high offset disables this block. */
    uint16_t cpu_stall_options_offset;
    uint16_t cpu_stall_high_offset;
    uint32_t cpu_stall_options_reset;
    uint8_t cpu_stall_low_shift[2];
    uint8_t cpu_stall_high_shift[2];
    uint32_t software_reset_cpu0_mask;
    uint32_t software_reset_cpu1_mask;
    uint32_t software_reset_system_mask;
    uint16_t clock_conf_offset;
    uint8_t  slow_clock_select_shift;
    uint32_t time_update_mask;
    uint32_t time_high_mask;
    uint32_t reset_state_reset;
    uint32_t clock_conf_reset;
    uint32_t clock_conf_writable_mask;
    uint32_t slow_clock_select_mask;
    uint32_t slow_clock_source_hz[4];
    /* RTC analog power controls. The SAR-I2C bit gates access to the
     * internal SAR analog-register slave; other writable bits retain their
     * register value but remain diagnostic until their consumers exist. */
    uint16_t analog_conf_offset;
    uint32_t analog_conf_reset;
    uint32_t analog_conf_writable_mask;
    uint32_t sar_i2c_power_mask;
    /* Optional RTC USB PHY mux. Only the software override and selection
     * bits are functional; analog USB controls remain diagnostic. */
    uint16_t usb_conf_offset;
    uint32_t usb_conf_reset;
    uint32_t usb_conf_writable_mask;
    uint32_t usb_phy_override_mask;
    uint32_t usb_phy_select_mask;
    uint16_t interrupt_enable_offset;
    uint16_t interrupt_raw_offset;
    uint16_t interrupt_status_offset;
    uint16_t interrupt_clear_offset;
    uint32_t interrupt_enable_reset;
    uint32_t interrupt_raw_reset;
    uint32_t interrupt_valid_mask;
    uint32_t interrupt_raw_writable_mask;
    uint8_t  interrupt_source;
    uint16_t wdt_config_offset[FLEXE_TARGET_RTC_WDT_CONFIG_MAX];
    uint16_t wdt_feed_offset;
    uint16_t wdt_write_protect_offset;
    uint32_t wdt_config_reset[FLEXE_TARGET_RTC_WDT_CONFIG_MAX];
    uint32_t wdt_config_writable_mask[FLEXE_TARGET_RTC_WDT_CONFIG_MAX];
    uint32_t wdt_enable_mask;
    uint32_t wdt_flashboot_enable_mask;
    uint32_t wdt_feed_mask;
    uint32_t wdt_write_protect_key;
    uint32_t wdt_interrupt_mask;
    uint8_t  wdt_stage_action_shift[FLEXE_TARGET_RTC_WDT_STAGE_MAX];
    uint8_t  wdt_stage_action_mask;
    uint8_t  wdt_stage0_multiplier;
    /* RTC and digital pad-hold registers. Each field maps consecutive GPIO
     * pins to bits; the two sources may overlap at a physical pad. */
    uint16_t rtc_pad_hold_offset;
    uint8_t  rtc_pad_hold_first_gpio;
    uint8_t  rtc_pad_hold_first_bit;
    uint8_t  rtc_pad_hold_count;
    uint16_t digital_pad_hold_offset;
    uint8_t  digital_pad_hold_first_gpio;
    uint8_t  digital_pad_hold_first_bit;
    uint8_t  digital_pad_hold_count;
    /* Optional RTC sleep/wake register block. A zero timer-low offset means
     * that this target has no sleep model in the target RTC controller. */
    uint16_t sleep_timer_low_offset;
    uint16_t sleep_timer_high_offset;
    uint16_t sleep_state_offset;
    /* Power-domain sequencing timers are software-visible RTC state. Fast
     * execution does not assign their analog settle delays to guest time. */
    uint8_t sequence_register_count;
    flexe_rtc_sequence_register_desc_t
        sequence_register[FLEXE_TARGET_RTC_SEQUENCE_REGISTER_MAX];
    uint16_t cpu_stall_enable_offset;
    uint32_t cpu_stall_enable_mask;
    uint16_t wakeup_state_offset;
    uint16_t digital_power_offset;
    uint16_t wakeup_cause_offset;
    uint16_t rtc_power_offset;
    uint32_t rtc_power_reset;
    uint32_t rtc_pad_force_hold_mask;
    /* Digital-pad isolation/force-hold register. Other isolation controls
     * remain visible but are not electrically modeled. */
    uint16_t digital_iso_offset;
    uint32_t digital_iso_reset;
    uint32_t digital_pad_force_hold_mask;
    uint32_t digital_pad_force_unhold_mask;
    uint32_t digital_power_reset;
    uint32_t sleep_enable_mask;
    uint32_t sleep_wakeup_mask;
    uint32_t sleep_alarm_enable_mask;
    uint32_t sleep_alarm_interrupt_mask;
    uint32_t digital_wrap_power_down_mask;
    uint32_t timer_wakeup_mask;
    uint32_t ext0_wakeup_mask;
    uint32_t ext1_wakeup_mask;
    uint16_t ext_wakeup_config_offset;
    uint16_t ext1_select_offset;
    uint16_t ext1_status_offset;
    uint16_t brownout_offset;
    uint32_t brownout_reset;
    uint32_t brownout_writable_mask;
    uint32_t brownout_detect_mask;
    uint32_t brownout_count_clear_mask;
    uint32_t ext0_wakeup_level_mask;
    uint32_t ext1_wakeup_level_mask;
    uint32_t ext1_select_mask;
    uint32_t ext1_status_clear_mask;
    uint32_t sleep_wakeup_interrupt_mask;
    uint32_t wakeup_valid_mask;
    uint32_t wakeup_enable_reset;
    uint8_t wakeup_enable_shift;
} flexe_rtc_cntl_desc_t;

/* S3-generation RTC GPIO output bank and contiguous per-pad RTC mux. GPIO
 * numbering matches RTC GPIO numbering on S3, but the register values and
 * page address remain target data. */
typedef struct {
    uint32_t base;
    uint32_t register_size;
    uint8_t gpio_count;
    uint8_t data_shift;
    uint16_t pad_base_offset;
    uint32_t pad_mux_mask;
    uint16_t ext0_select_offset;
    uint8_t ext0_select_shift;
    uint8_t ext0_select_width;
    uint32_t pad_reset[FLEXE_TARGET_RTC_IO_PIN_MAX];
} flexe_rtc_io_desc_t;

/* Read views of a virtual chip's one-time-programmable fuse blocks. Burning
 * fuses is intentionally a separate capability: a read-only profile must not
 * silently accept irreversible programming commands. */
typedef struct {
    uint32_t base;
    uint32_t register_size;
    uint32_t read_data_offset;
    uint16_t read_data_word_count;
    uint32_t date_offset;
    uint32_t date_reset;
    uint32_t date_writable_mask;
    uint32_t read_data[FLEXE_TARGET_EFUSE_READ_WORD_MAX];
} flexe_efuse_desc_t;

/* Peripheral interrupt fabric used by newer ESP32-family targets. Each
 * source has one CPU-interrupt selector per core. Raw source status remains
 * visible independently of routing, while the per-core clock gate controls
 * delivery. Software-generated sources can live in a separate system-control
 * block, so their address is described independently from the matrix page. */
typedef struct {
    uint32_t base;
    uint32_t register_size;
    uint16_t source_count;
    uint32_t map_offset[FLEXE_TARGET_INTERRUPT_CORE_MAX];
    uint32_t status_offset[FLEXE_TARGET_INTERRUPT_CORE_MAX];
    uint32_t clock_gate_offset[FLEXE_TARGET_INTERRUPT_CORE_MAX];
    uint32_t date_offset[FLEXE_TARGET_INTERRUPT_CORE_MAX];
    uint32_t map_reset;
    uint32_t map_writable_mask;
    uint32_t clock_gate_reset;
    uint32_t clock_gate_writable_mask;
    uint32_t date_reset;
    uint32_t date_writable_mask;

    uint32_t software_interrupt_base;
    uint32_t software_interrupt_offset;
    uint32_t software_interrupt_stride;
    uint16_t software_interrupt_source_base;
    uint8_t  software_interrupt_count;
    uint32_t software_interrupt_writable_mask;
} flexe_interrupt_matrix_desc_t;

/* RTC slow-clock calibration is embedded in each timer-group register block.
 * The surrounding timer IP changes across ESP32-family targets, so exposing
 * this small common contract avoids registering a classic timer-group model
 * at an incompatible address. Clock frequencies are nominal functional-mode
 * values; calibrated timing profiles can replace them in accurate modes. */
typedef struct {
    uint8_t  group_count;
    uint32_t base[FLEXE_TARGET_RTC_CAL_GROUP_MAX];
    uint32_t register_size;
    uint32_t config_offset;
    uint32_t value_offset;
    uint32_t timeout_offset;
    uint32_t config_reset;
    uint32_t timeout_reset;
    uint32_t config_writable_mask;
    uint32_t timeout_writable_mask;
    uint32_t start_mask;
    uint32_t cycling_mask;
    uint32_t ready_mask;
    uint32_t timeout_mask;
    uint32_t cycles_mask;
    uint32_t clock_select_mask;
    uint32_t result_mask;
    uint8_t  cycles_shift;
    uint8_t  clock_select_shift;
    uint8_t  result_shift;
    uint32_t reference_clock_hz;
    uint32_t source_clock_hz[FLEXE_TARGET_RTC_CAL_CLOCK_MAX];
} flexe_rtc_calibration_desc_t;

typedef struct {
    uint32_t offset;
    uint32_t reset;
    uint32_t writable_mask;
} flexe_regi2c_aux_register_desc_t;

/* RTC-domain sensor controller. Temperature and polled SAR ADC register
 * geometry live here rather than in the machine frontend.
 * In fast mode a powered, clocked conversion completes synchronously; timed
 * modes can derive latency from the retained divider and wait fields. */
typedef struct {
    uint16_t reader_offset;
    uint16_t measure_offset;
    uint16_t mux_offset;
    uint16_t atten_offset;
    uint32_t reader_reset;
    uint32_t reader_writable_mask;
    uint32_t reader_invert_mask;
    uint32_t mux_writable_mask;
    uint32_t mux_rtc_block_mask;
    uint32_t mux_rtc_bypass_mask;
    uint32_t mux_unsupported_mask;
    uint8_t calibration_ground_mask;
    bool arbiter_controlled;
} flexe_sens_adc_unit_desc_t;

/* APB SAR ADC2 arbitration. Only the RTC requester is currently modeled;
 * digital DMA and Wi-Fi/PWDET requests remain explicit unsupported paths. */
typedef struct {
    uint32_t base;
    uint32_t register_size;
    uint16_t arbiter_offset;
    uint32_t arbiter_reset;
    uint32_t arbiter_writable_mask;
    uint32_t grant_force_mask;
    uint32_t rtc_force_mask;
    uint32_t force_selection_mask;
} flexe_apb_saradc_desc_t;

typedef struct {
    uint32_t base;
    uint32_t register_size;
    uint16_t control_offset;
    uint16_t control2_offset;
    uint16_t clock_gate_offset;
    uint16_t reset_offset;
    uint32_t control_reset;
    uint32_t control2_reset;
    uint32_t clock_gate_reset;
    uint32_t reset_reset;
    uint32_t control_writable_mask;
    uint32_t control2_writable_mask;
    uint32_t clock_gate_writable_mask;
    uint32_t reset_writable_mask;
    uint32_t dump_out_mask;
    uint32_t power_up_force_mask;
    uint32_t power_up_mask;
    uint32_t input_invert_mask;
    uint32_t interrupt_enable_mask;
    uint32_t ready_mask;
    uint32_t output_mask;
    uint32_t xpd_force_mask;
    uint32_t clock_enable_mask;
    uint32_t reset_mask;
    uint32_t rtc_interrupt_mask;
    uint16_t default_output;
    uint8_t adc_unit_count;
    uint8_t adc_channels_per_unit;
    uint16_t adc_power_offset;
    uint16_t adc_status_offset;
    uint32_t adc_power_writable_mask;
    uint32_t adc_status_writable_mask;
    uint32_t adc_clock_enable_mask;
    uint32_t adc_reset_mask;
    uint16_t adc_output_mask;
    uint8_t adc_calibration_slave;
    uint8_t adc_calibration_address;
    flexe_sens_adc_unit_desc_t adc_unit[FLEXE_TARGET_SENS_ADC_UNIT_MAX];
} flexe_sens_desc_t;

/* Undocumented Wi-Fi/Bluetooth controller apertures contain a mixture of
 * ordinary configuration words and short hardware operations. Retaining
 * each target-described register window is sufficient for read/modify/write
 * setup; completion descriptors give the few firmware-visible state
 * machines explicit semantics instead of matching a firmware or ROM PC.
 *
 * A completion becomes ready while every active_mask bit is set and clears
 * when the operation is disabled. self_clear_mask describes command bits
 * which hardware consumes on write. status_mask is read-only even when the
 * status and control addresses are the same register. */
typedef struct {
    uint32_t base;
    uint32_t register_size;
} flexe_radio_window_desc_t;

typedef struct {
    uint32_t control_address;
    uint32_t active_mask;
    uint32_t self_clear_mask;
    uint32_t status_address;
    uint32_t status_mask;
} flexe_radio_completion_desc_t;

/* The private BT controller can snapshot its baseband half-slot clock into
 * two registers. The ROM's time_get path requests a latch then reads the
 * half-slot count and its down-counting subslot phase. */
typedef struct {
    uint32_t count_address;
    uint32_t phase_address;
    uint32_t capture_mask;
    uint32_t count_mask;
    uint32_t tick_hz;
    uint16_t ticks_per_half_slot;
} flexe_radio_time_latch_desc_t;

typedef struct {
    uint8_t window_count;
    uint8_t completion_count;
    flexe_radio_window_desc_t
        window[FLEXE_TARGET_RADIO_WINDOW_MAX];
    flexe_radio_completion_desc_t
        completion[FLEXE_TARGET_RADIO_COMPLETION_MAX];
    uint32_t random_address;
    uint64_t random_seed;
    flexe_radio_time_latch_desc_t time_latch;
} flexe_radio_desc_t;

/* General-purpose DMA v1 is the five-channel AHB DMA shared by several
 * ESP32-family peripherals. The register layout and 12-byte linked-list
 * descriptor format are properties of the v1 IP; target data supplies its
 * placement, instantiated channel count, and the address prefix omitted by
 * the hardware's 20-bit link register. */
typedef struct {
    uint32_t base;
    uint32_t register_size;
    uint32_t channel_stride;
    uint32_t descriptor_address_prefix;
    uint8_t  channel_count;
    uint8_t  rx_interrupt_source[FLEXE_TARGET_GDMA_CHANNEL_MAX];
    uint8_t  tx_interrupt_source[FLEXE_TARGET_GDMA_CHANNEL_MAX];
} flexe_gdma_desc_t;

typedef enum {
    FLEXE_SHA_LAYOUT_NONE = 0,
    /* Classic ESP32: one START/CONTINUE/LOAD/BUSY quartet per algorithm and
     * a shared message/digest register file. */
    FLEXE_SHA_LAYOUT_ESP32,
    /* S2/S3 generation: one MODE register selects the algorithm, with
     * separate message and digest register files and optional GDMA input. */
    FLEXE_SHA_LAYOUT_UNIFIED,
} flexe_sha_layout_t;

typedef enum {
    FLEXE_SHA_ALGORITHM_NONE = 0,
    FLEXE_SHA_ALGORITHM_SHA1,
    FLEXE_SHA_ALGORITHM_SHA224,
    FLEXE_SHA_ALGORITHM_SHA256,
    FLEXE_SHA_ALGORITHM_SHA384,
    FLEXE_SHA_ALGORITHM_SHA512,
    FLEXE_SHA_ALGORITHM_SHA512_224,
    FLEXE_SHA_ALGORITHM_SHA512_256,
    FLEXE_SHA_ALGORITHM_SHA512_T,
} flexe_sha_algorithm_t;

/* SHA mode values are part of the ROM/HAL ABI and differ between classic
 * ESP32 and newer unified accelerators. Mapping them in the target keeps the
 * crypto engine independent of firmware versions and absolute addresses.
 * dma_peripheral_id is the GDMA v1 OUT_PERI_SEL value, or UINT8_MAX when the
 * target has no DMA path. */
typedef struct {
    uint32_t             base;
    uint32_t             register_size;
    flexe_sha_layout_t   layout;
    uint8_t              mode_count;
    uint8_t              dma_peripheral_id;
    flexe_sha_algorithm_t mode[FLEXE_TARGET_SHA_MODE_MAX];
} flexe_sha_desc_t;

/* Internal analog-register I2C fabric used by ROM clock, bias, PHY, and ADC
 * code. This is distinct from the externally routed I2C controllers. The ROM
 * command ABI is described here so the same device model can serve targets
 * whose host count, register locations, or bit fields differ. Some revisions
 * expose adjacent analog-controller state in the same aperture; auxiliary
 * register descriptors give those surfaces explicit reset and access masks
 * without baking target addresses into the device implementation. */
typedef struct {
    uint32_t base;
    uint32_t register_size;
    uint8_t  host_count;
    uint32_t command_offset;
    uint32_t command_stride;
    uint32_t analog_control_offset;
    uint32_t config_offset;
    uint32_t config2_offset;
    uint32_t analog_control_reset;
    uint32_t config_reset;
    uint32_t config2_reset;
    uint32_t analog_control_writable_mask;
    uint32_t config_writable_mask;
    uint32_t config2_writable_mask;
    uint32_t command_start_mask;
    uint32_t command_busy_mask;
    uint32_t command_write_mask;
    uint32_t slave_mask;
    uint32_t address_mask;
    uint32_t data_mask;
    uint8_t  slave_shift;
    uint8_t  address_shift;
    uint8_t  data_shift;
    uint32_t bbpll_stop_high_mask;
    uint32_t bbpll_stop_low_mask;
    uint32_t bbpll_done_mask;
    uint8_t  aux_register_count;
    flexe_regi2c_aux_register_desc_t
        aux_register[FLEXE_TARGET_REGI2C_AUX_REGISTER_MAX];
} flexe_regi2c_desc_t;

/* Security/memory-protection register IP shared by compatible targets. The
 * capability selects the register layout; the descriptor supplies only its
 * target address aperture. */
typedef struct {
    uint32_t base;
    uint32_t register_size;
} flexe_sensitive_memprot_desc_t;

/* System-timer IP used by newer ESP32-family SoCs. V1 has two 52-bit
 * counters and three comparators; counts and field widths remain described
 * so machine construction rejects an incompatible target rather than
 * quietly applying S3 semantics to it. */
typedef struct {
    uint32_t base;
    uint32_t register_size;
    uint32_t counter_frequency_hz;
    uint32_t config_reset;
    uint32_t date_reset;
    uint8_t  counter_count;
    uint8_t  alarm_count;
    uint8_t  counter_width;
    uint8_t  interrupt_source[FLEXE_TARGET_SYSTIMER_ALARM_MAX];
} flexe_systimer_desc_t;

/* Newer ESP32-family timer-group IP. V1 describes the S3-style register
 * layout: two general-purpose timers, a four-stage main watchdog, and three
 * independent interrupt sources per group. RTC calibration occupies the
 * same pages but remains a separate capability so either block can be tested
 * and reused independently. */
typedef struct {
    uint32_t base[FLEXE_TARGET_TIMER_GROUP_MAX];
    uint32_t register_size;
    uint32_t apb_clock_hz;
    uint32_t xtal_clock_hz;
    uint32_t timer_config_reset;
    uint32_t timer_config_writable_mask;
    uint32_t wdt_config_reset[6];
    uint32_t wdt_config_writable_mask[6];
    uint32_t wdt_write_protect_key;
    uint32_t date_reset;
    uint32_t date_writable_mask;
    uint32_t regclk_reset;
    uint32_t regclk_writable_mask;
    uint8_t  group_count;
    uint8_t  timer_count;
    uint8_t  counter_width;
    uint8_t  interrupt_source[FLEXE_TARGET_TIMER_GROUP_MAX]
                                     [FLEXE_TARGET_TIMER_GROUP_EVENT_MAX];
} flexe_timer_group_desc_t;

/* ESP32-S3-generation RMT pulse engine. Register offsets and command bits
 * belong to the V1 IP block; geometry, clocks and IRQ routing belong to the
 * target instance. TX and RX channels share the same pulse RAM. */
typedef struct {
    uint32_t base;
    uint32_t register_size;
    uint16_t memory_offset;
    uint8_t  tx_channel_count;
    uint8_t  channel_count;
    uint8_t  words_per_channel;
    uint8_t  interrupt_source;
    uint16_t input_signal_base; /* RX0 GPIO-matrix signal */
    uint32_t apb_clock_hz;
    uint32_t ref_clock_hz;
    uint32_t xtal_clock_hz;
} flexe_rmt_v1_desc_t;

/* Low-speed-only LEDC generation used by ESP32-S3. The register layout is
 * intrinsic to this IP version; clocks, GPIO-matrix routes and IRQ are SoC
 * wiring and therefore remain target data. */
typedef struct {
    uint32_t base;
    uint32_t register_size;
    uint32_t source_clock_hz[4]; /* CONF.APB_CLK_SEL: 0 unused, 1/2/3 */
    uint32_t date_reset;
    uint16_t output_signal_base;
    uint8_t interrupt_source;
} flexe_ledc_v1_desc_t;

/* General-purpose SPI2/SPI3 controller generations keep the same transaction
 * phases while moving the FIFO, completion interrupt, and length registers.
 * The layout selects those IP semantics; every address, interrupt source,
 * GPIO-matrix route, native IOMUX route, and GDMA trigger remains target data.
 * A UINT*_MAX route value means that the corresponding hardware connection
 * does not exist on this target/host. */
typedef enum {
    FLEXE_GP_SPI_LAYOUT_NONE = 0,
    FLEXE_GP_SPI_LAYOUT_ESP32,
    FLEXE_GP_SPI_LAYOUT_S2_S3,
} flexe_gp_spi_layout_t;

typedef struct {
    uint32_t base;
    uint16_t clock_out_signal;
    uint16_t chip_select_out_signal[FLEXE_TARGET_GP_SPI_CS_MAX];
    uint8_t  interrupt_source;
    uint8_t  chip_select_count;
    uint8_t  iomux_clock_pin;
    uint8_t  iomux_chip_select0_pin;
    uint8_t  iomux_function;
    uint8_t  gdma_peripheral_id;
} flexe_gp_spi_instance_desc_t;

typedef struct {
    uint32_t register_size;
    uint32_t date_reset;
    uint8_t  host_count;
    flexe_gp_spi_layout_t layout;
    flexe_gp_spi_instance_desc_t instance[FLEXE_TARGET_GP_SPI_HOST_MAX];
} flexe_gp_spi_desc_t;

/* SPI0/SPI1 memory-controller generations share command semantics but move
 * the transaction and data-buffer registers. The selected architectural
 * layout supplies those register definitions; target data identifies the
 * instantiated hosts and devices attached to their hardware chip selects,
 * keeping board defaults out of the controller implementation. */
typedef enum {
    FLEXE_SPI_MEM_LAYOUT_NONE = 0,
    FLEXE_SPI_MEM_LAYOUT_ESP32,
    FLEXE_SPI_MEM_LAYOUT_S2_S3,
} flexe_spi_mem_layout_t;

typedef enum {
    FLEXE_SPI_MEM_PSRAM_NONE = 0,
    FLEXE_SPI_MEM_PSRAM_QUAD,
    FLEXE_SPI_MEM_PSRAM_AP_8M_OPI,
} flexe_spi_mem_psram_kind_t;

/* Optional board population; zero retains the target's standard board.
 * The AP profile represents a physical 64-Mbit APS6408L-3OBMx on S3 CS1. */
typedef enum {
    FLEXE_BOARD_PSRAM_DEFAULT = 0,
    FLEXE_BOARD_PSRAM_AP_8M_OPI,
} flexe_board_psram_t;

typedef struct {
    uint32_t                base[FLEXE_TARGET_SPI_MEM_HOST_MAX];
    uint32_t                register_size;
    uint32_t                default_jedec_id;
    uint32_t                maximum_flash_size;
    uint32_t                date_reset;
    uint64_t                default_psram_id;
    uint8_t                 host_count;
    uint8_t                 flash_chip_select;
    uint8_t                 psram_chip_select;
    flexe_spi_mem_layout_t  layout;
    flexe_spi_mem_psram_kind_t psram_kind;
} flexe_spi_mem_desc_t;

/* Live flash geometry shared by the ROM, second-stage bootloader, and
 * application. Older ROMs export the structure itself at a stable address;
 * newer ROMs expose an ABI pointer whose symbol is available in the official
 * ROM ELF. Field offsets remain target data so direct application handoff
 * does not depend on a C SDK structure or firmware-specific address. */
typedef struct {
    uint32_t    live_data_address;
    const char *pointer_symbol;
    uint16_t    struct_size;
    uint16_t    device_id_offset;
    uint16_t    chip_size_offset;
    uint16_t    block_size_offset;
    uint16_t    sector_size_offset;
    uint16_t    page_size_offset;
    uint16_t    status_mask_offset;
    uint32_t    block_size;
    uint32_t    sector_size;
    uint32_t    page_size;
    uint32_t    status_mask;
} flexe_rom_flash_desc_t;

/* Native USB Serial/JTAG device shared by newer ESP32-family targets. V1
 * fixes the register layout while the descriptor supplies target placement,
 * reset values, masks, interrupt routing, and endpoint geometry. */
typedef struct {
    uint32_t base;
    uint32_t register_size;
    uint32_t interrupt_source;
    uint32_t interrupt_valid_mask;
    uint32_t interrupt_raw_reset;
    uint32_t conf0_reset;
    uint32_t conf0_writable_mask;
    uint32_t test_reset;
    uint32_t test_writable_mask;
    uint32_t misc_conf_reset;
    uint32_t misc_conf_writable_mask;
    uint32_t mem_conf_reset;
    uint32_t mem_conf_writable_mask;
    uint32_t date_reset;
    uint32_t date_writable_mask;
    uint8_t  endpoint_size;
} flexe_usb_serial_jtag_desc_t;

typedef struct {
    /* Increment when the descriptor ABI or the meaning of a field changes. */
    uint32_t                    descriptor_version;
    flexe_target_id_t           id;
    const char                 *name;
    const char                 *display_name;
    uint16_t                    image_chip_id;
    flexe_xtensa_generation_t   core_generation;
    uint8_t                     core_count;
    flexe_target_support_t      support_level;
    uint64_t                    capabilities;

    /* Xtensa core identity and reset state from the official core config. */
    uint32_t                    reset_vector;
    uint32_t                    vecbase_reset;
    uint32_t                    configid0;
    uint32_t                    configid1;
    uint8_t                     interrupt_level[32];

    /* CPU clock state at direct application handoff. The ROM-maintained
     * ticks-per-microsecond word lets frequency changes remain visible to
     * target-independent timers and idle-time accounting. */
    uint32_t                    default_cpu_frequency_mhz;
    uint32_t                    cpu_frequency_word;

    /* Safe scratch stacks for direct-to-application startup. Frontends may
     * override core 0; these defaults must lie in writable internal RAM. */
    uint32_t                    bootstrap_stack_top[2];

    /* SoC address geometry. Ranges are inclusive/exclusive. */
    uint32_t                    iram_start;
    uint32_t                    iram_end;
    uint32_t                    dram_start;
    uint32_t                    dram_end;

    /* Flash-mapped address windows from the target's image format. */
    uint32_t                    drom_start;
    uint32_t                    drom_end;
    uint32_t                    irom_start;
    uint32_t                    irom_end;
    uint32_t                    default_app_offset;
    uint32_t                    partition_table_offset;
    flexe_flash_mmu_desc_t      flash_mmu;

    /* Optional cache/external-memory control register block. Its register
     * semantics are selected by a capability above; addresses stay in the
     * target descriptor rather than leaking into generic machine setup. */
    uint32_t                    cache_control_base;
    uint32_t                    cache_control_size;

    /* Reusable on-chip UART instances. A zero count means the target has no
     * registered UART model, independently of its overall support level. */
    uint8_t                     uart_count;
    flexe_uart_ip_desc_t        uart_ip;
    flexe_uart_instance_desc_t  uart[FLEXE_TARGET_UART_MAX];

    /* Optional external I2C controllers and pluggable bus endpoints. */
    flexe_i2c_desc_t            i2c;

    /* Optional SoC register block controlling the secondary CPU. */
    flexe_secondary_core_desc_t secondary_core;

    /* Optional CPU/system-clock selection register block. */
    flexe_system_clock_desc_t    system_clock;

    /* Optional digital pad configuration register file. */
    flexe_io_mux_desc_t          io_mux;

    /* Optional S2/S3-generation digital GPIO matrix. */
    flexe_gpio_desc_t             gpio;

    /* Optional always-on RTC controller and boot-handoff state. */
    flexe_rtc_cntl_desc_t        rtc_cntl;
    flexe_rtc_io_desc_t          rtc_io;

    /* Optional read-only virtual-silicon eFuse profile. */
    flexe_efuse_desc_t           efuse;

    /* Optional V1 peripheral interrupt matrix and software generators. */
    flexe_interrupt_matrix_desc_t interrupt_matrix;

    /* Optional timer-group RTC slow-clock calibration interface. */
    flexe_rtc_calibration_desc_t rtc_calibration;

    /* Optional internal analog-register I2C fabric. */
    flexe_regi2c_desc_t           regi2c;

    /* Optional RTC-domain ADC/touch/temperature sensor controller. */
    flexe_sens_desc_t             sens;
    flexe_apb_saradc_desc_t       apb_saradc;

    /* Optional RF/baseband/controller register and calibration surfaces. */
    flexe_radio_desc_t            radio;

    /* Optional general-purpose DMA fabric and SHA accelerator. */
    flexe_gdma_desc_t             gdma;
    flexe_sha_desc_t              sha;

    /* Optional SENSITIVE v1 memory-protection configuration block. */
    flexe_sensitive_memprot_desc_t sensitive_memprot;

    /* Optional V1 system-timer register block. */
    flexe_systimer_desc_t         systimer;

    /* Optional V1 timer-group and main-watchdog register blocks. */
    flexe_timer_group_desc_t      timer_group;

    /* Optional V1 remote-control pulse engine. */
    flexe_rmt_v1_desc_t           rmt_v1;
    flexe_ledc_v1_desc_t          ledc_v1;

    /* Optional general-purpose SPI2/SPI3 controllers and board routes. */
    flexe_gp_spi_desc_t           gp_spi;

    /* Optional SPI memory controllers and their default attached devices. */
    flexe_spi_mem_desc_t          spi_mem;

    /* Optional ROM/bootloader-to-application flash handoff structure. */
    flexe_rom_flash_desc_t        rom_flash;

    /* Optional native USB Serial/JTAG endpoint controller. */
    flexe_usb_serial_jtag_desc_t  usb_serial_jtag;

    /* Initial address map. Flash cache windows are initially linear so an
     * image can be loaded; the target MMU replaces those mappings at boot. */
    uint32_t                    backing_size[FLEXE_MEM_BACKING_COUNT];
    uint8_t                     memory_region_count;
    flexe_target_mem_region_t   memory_region[FLEXE_TARGET_MEM_REGION_MAX];

    /* Canonical MMIO window and an optional address alias. alias_delta is
     * added to an address in the alias window before handler dispatch. */
    uint32_t                    peripheral_start;
    uint32_t                    peripheral_end;
    uint32_t                    peripheral_alias_start;
    uint32_t                    peripheral_alias_end;
    int32_t                     peripheral_alias_delta;

    /* All architecturally executable windows, including reset/RTC memory. */
    uint8_t                     executable_range_count;
    flexe_addr_range_t          executable[FLEXE_TARGET_EXEC_RANGE_MAX];
} flexe_target_desc_t;

const flexe_target_desc_t *flexe_target_by_id(flexe_target_id_t id);
const flexe_target_desc_t *flexe_target_by_image_chip_id(uint16_t chip_id);
const flexe_target_desc_t *flexe_target_by_name(const char *name);
/* Copy a canonical target and populate an optional board-side PSRAM device.
 * The caller owns `out` for as long as memory, CPUs and peripherals use it. */
bool flexe_target_with_board_psram(const flexe_target_desc_t *base,
                                   flexe_board_psram_t board_psram,
                                   flexe_target_desc_t *out);

/* Parse "auto" or a recognized descriptor name/alias. */
int flexe_target_parse(const char *name, flexe_target_id_t *id_out);

/* True when pc is inside one of the descriptor's executable windows. */
bool flexe_target_pc_is_executable(const flexe_target_desc_t *target,
                                   uint32_t pc);

/* True when the complete guest range belongs to one initial-map region backed
 * by `backing`. Empty linker-described ranges are valid. This describes target
 * geometry; callers after MMU changes must separately validate live mappings. */
bool flexe_target_range_uses_backing(const flexe_target_desc_t *target,
                                     uint32_t addr, uint32_t size,
                                     flexe_mem_backing_t backing);

/* Return a descriptor-provided direct-start stack, or zero for bad input. */
uint32_t flexe_target_bootstrap_stack(const flexe_target_desc_t *target,
                                      unsigned core);

#endif /* FLEXE_TARGET_H */
