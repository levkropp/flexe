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
#define FLEXE_TARGET_DESCRIPTOR_VERSION 7u

/* Device-model capabilities are architectural properties of a target, not
 * guesses derived from a firmware image. Keep each bit tied to a reusable IP
 * model so machine construction remains data-driven as the family grows. */
typedef enum {
    FLEXE_TARGET_CAP_ESP32_CLASSIC_PERIPHERALS = 1ull << 0,
    FLEXE_TARGET_CAP_ESP32S3_EXTMEM             = 1ull << 1,
    FLEXE_TARGET_CAP_DIRECT_ROM_DATA_INIT       = 1ull << 2,
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

/* Return a descriptor-provided direct-start stack, or zero for bad input. */
uint32_t flexe_target_bootstrap_stack(const flexe_target_desc_t *target,
                                      unsigned core);

#endif /* FLEXE_TARGET_H */
