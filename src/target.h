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
#define FLEXE_TARGET_RTC_CAL_GROUP_MAX 2u
#define FLEXE_TARGET_RTC_CAL_CLOCK_MAX 4u
#define FLEXE_TARGET_REGI2C_HOST_MAX 2u
#define FLEXE_TARGET_SYSTIMER_COUNTER_MAX 2u
#define FLEXE_TARGET_SYSTIMER_ALARM_MAX 3u
#define FLEXE_TARGET_TIMER_GROUP_MAX 2u
#define FLEXE_TARGET_TIMER_GROUP_TIMER_MAX 2u
#define FLEXE_TARGET_TIMER_GROUP_EVENT_MAX 3u
#define FLEXE_TARGET_SPI_MEM_HOST_MAX 2u
#define FLEXE_TARGET_INTERRUPT_CORE_MAX 2u
#define FLEXE_TARGET_INTERRUPT_SOURCE_MAX 128u
#define FLEXE_TARGET_SOFTWARE_INTERRUPT_MAX 4u
#define FLEXE_TARGET_GPIO_MAX 54u
#define FLEXE_TARGET_IO_MUX_REGISTER_MAX 64u
#define FLEXE_TARGET_IO_MUX_OFFSET_NONE UINT16_MAX
#define FLEXE_TARGET_RTC_STORE_MAX 8u
#define FLEXE_TARGET_EFUSE_READ_WORD_MAX 96u
#define FLEXE_SPI_MEM_CS_NONE UINT8_MAX
#define FLEXE_TARGET_DESCRIPTOR_VERSION 20u

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
    FLEXE_TARGET_CAP_RTC_STORAGE_V1               = 1ull << 14,
    FLEXE_TARGET_CAP_EFUSE_READ_V1                = 1ull << 15,
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

/* CPU/system-clock selection registers used by S2/S3-style clock trees.
 * The first version preserves the architectural register state firmware uses
 * to derive CPU and APB frequencies. Clock propagation into target-cycle
 * timing is deliberately a separate, calibrated-mode concern. */
typedef struct {
    uint32_t base;
    uint32_t register_size;
    uint32_t cpu_per_conf_offset;
    uint32_t cpu_per_conf_reset;
    uint32_t cpu_per_conf_writable_mask;
    uint32_t sysclk_conf_offset;
    uint32_t sysclk_conf_reset;
    uint32_t sysclk_conf_writable_mask;
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

/* Always-on scratch registers shared by the ROM, bootloader and application.
 * The offsets are explicit because older chips split STORE0..3 and STORE4..7
 * into separate parts of RTC_CNTL. */
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
} flexe_rtc_storage_desc_t;

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

/* Internal analog-register I2C fabric used by ROM clock, bias, PHY, and ADC
 * code. This is distinct from the externally routed I2C controllers. The ROM
 * command ABI is described here so the same device model can serve targets
 * whose host count, register locations, or bit fields differ. */
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

typedef struct {
    uint32_t                base[FLEXE_TARGET_SPI_MEM_HOST_MAX];
    uint32_t                register_size;
    uint32_t                default_jedec_id;
    uint32_t                date_reset;
    uint64_t                default_psram_id;
    uint8_t                 host_count;
    uint8_t                 flash_chip_select;
    uint8_t                 psram_chip_select;
    flexe_spi_mem_layout_t  layout;
} flexe_spi_mem_desc_t;

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

    /* Optional SoC register block controlling the secondary CPU. */
    flexe_secondary_core_desc_t secondary_core;

    /* Optional CPU/system-clock selection register block. */
    flexe_system_clock_desc_t    system_clock;

    /* Optional digital pad configuration register file. */
    flexe_io_mux_desc_t          io_mux;

    /* Optional RTC-domain scratch registers used across boot stages. */
    flexe_rtc_storage_desc_t     rtc_storage;

    /* Optional read-only virtual-silicon eFuse profile. */
    flexe_efuse_desc_t           efuse;

    /* Optional V1 peripheral interrupt matrix and software generators. */
    flexe_interrupt_matrix_desc_t interrupt_matrix;

    /* Optional timer-group RTC slow-clock calibration interface. */
    flexe_rtc_calibration_desc_t rtc_calibration;

    /* Optional internal analog-register I2C fabric. */
    flexe_regi2c_desc_t           regi2c;

    /* Optional SENSITIVE v1 memory-protection configuration block. */
    flexe_sensitive_memprot_desc_t sensitive_memprot;

    /* Optional V1 system-timer register block. */
    flexe_systimer_desc_t         systimer;

    /* Optional V1 timer-group and main-watchdog register blocks. */
    flexe_timer_group_desc_t      timer_group;

    /* Optional SPI memory controllers and their default attached devices. */
    flexe_spi_mem_desc_t          spi_mem;

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
