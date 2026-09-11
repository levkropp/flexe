#include "target.h"

#include <stddef.h>
#include <string.h>

/* Image chip IDs follow Espressif's public esp_chip_id_t values. The mapped
 * flash windows match esptool's target definitions. Keeping recognized but
 * unavailable targets here lets the loader fail explicitly instead of
 * feeding a valid image to the wrong SoC model. */
static const flexe_target_desc_t TARGETS[] = {
    {
        .descriptor_version = FLEXE_TARGET_DESCRIPTOR_VERSION,
        .id = FLEXE_TARGET_ESP32,
        .name = "esp32",
        .display_name = "ESP32",
        .image_chip_id = 0x0000,
        .core_generation = FLEXE_XTENSA_LX6,
        .core_count = 2,
        .support_level = FLEXE_TARGET_STABLE,
        .capabilities = FLEXE_TARGET_CAP_ESP32_CLASSIC_PERIPHERALS |
                        FLEXE_TARGET_CAP_SPI_MEM,
        .reset_vector = 0x40000400u,
        .vecbase_reset = 0x40000000u,
        .configid0 = 0xC2BCFFFEu,
        .configid1 = 0x1CC5FE96u,
        .default_cpu_frequency_mhz = 160u,
        .cpu_frequency_word = 0x3FFE01E0u,
        .bootstrap_stack_top = { 0x3FFE0000u, 0x3FFE8000u },
        .interrupt_level = {
            1, 1, 1, 1, 1, 1, 1, 1,
            1, 1, 1, 3, 1, 1, 7, 3,
            5, 1, 1, 2, 2, 2, 3, 3,
            4, 4, 5, 3, 4, 3, 4, 5,
        },
        .iram_start = 0x40080000u,
        .iram_end = 0x400AA000u,
        .dram_start = 0x3FFAE000u,
        .dram_end = 0x40000000u,
        .drom_start = 0x3F400000u,
        .drom_end = 0x3F800000u,
        .irom_start = 0x400D0000u,
        .irom_end = 0x40400000u,
        .default_app_offset = 0x00010000u,
        .partition_table_offset = 0x00008000u,
        .flash_mmu = {
            .page_size = 0x00010000u,
            .entry_count = 256,
            .table_base = { 0x3FF10000u, 0x3FF12000u },
            .invalid_entry = 0x00000100u,
            .invalid_mask = 0x00000100u,
            .physical_page_mask = 0x000000FFu,
        },
        .cache_control_base = 0x3FF00000u,
        .cache_control_size = 0x00014000u,
        .uart_count = 3,
        .uart_ip = {
            .register_size = 0x100u,
            .interrupt_valid_mask = 0x0007FFFFu,
            .status_idle_value = 0u,
            .rx_full_threshold_mask = 0x7Fu,
            .rx_timeout_enable_mask = 1u << 31,
            .mem_rx_status_offset = 0x60u,
            .fifo_address_mask = 0x7FFu,
            .mem_rx_read_shift = 2u,
            .mem_rx_write_shift = 13u,
            .date_offset = 0x78u,
            .date_reset = 0x15122500u,
        },
        .uart = {
            { .base = 0x3FF40000u, .interrupt_source = 34u },
            { .base = 0x3FF50000u, .interrupt_source = 35u },
            { .base = 0x3FF6E000u, .interrupt_source = 36u },
        },
        .spi_mem = {
            .base = { 0x3FF43000u, 0x3FF42000u },
            .register_size = 0x1000u,
            .default_jedec_id = 0x001640C8u,
            .date_reset = 0x01604270u,
            /* AP Memory's original 32-Mbit device: MFID 0x0D, KGD 0x5D,
             * EID 0x20. This matches the existing 4-MiB PSRAM backing used
             * by the default classic-ESP32 board profile. */
            .default_psram_id = UINT64_C(0x0000000000205D0D),
            .host_count = 2u,
            .flash_chip_select = 0u,
            .psram_chip_select = 1u,
            .layout = FLEXE_SPI_MEM_LAYOUT_ESP32,
        },
        .backing_size = {
            [FLEXE_MEM_SRAM] = 0x000B0000u,
            [FLEXE_MEM_ROM] = 0x00080000u,
            [FLEXE_MEM_FLASH_DATA] = 0x00400000u,
            [FLEXE_MEM_FLASH_INSN] = 0x00400000u,
            [FLEXE_MEM_RTC_FAST] = 0x00002000u,
            [FLEXE_MEM_RTC_SLOW] = 0x00002000u,
            [FLEXE_MEM_PSRAM] = 0x00400000u,
        },
        .memory_region_count = 10,
        .memory_region = {
            { 0x3F400000u, 0x3F800000u, FLEXE_MEM_FLASH_DATA, 0, "flash_data" },
            { 0x3F800000u, 0x3FC00000u, FLEXE_MEM_PSRAM, 0, "psram" },
            { 0x3FF80000u, 0x3FF82000u, FLEXE_MEM_RTC_FAST, 0, "rtc_dram" },
            { 0x3FF90000u, 0x3FFA0000u, FLEXE_MEM_ROM, 0x00070000u, "rom_data" },
            { 0x3FFA0000u, 0x40000000u, FLEXE_MEM_SRAM, 0, "sram_data" },
            { 0x40000000u, 0x40070000u, FLEXE_MEM_ROM, 0, "rom" },
            { 0x40070000u, 0x400C0000u, FLEXE_MEM_SRAM, 0x00060000u, "sram_insn" },
            { 0x400C0000u, 0x400C2000u, FLEXE_MEM_RTC_FAST, 0, "rtc_iram" },
            { 0x400D0000u, 0x40400000u, FLEXE_MEM_FLASH_INSN, 0, "flash_insn" },
            { 0x50000000u, 0x50002000u, FLEXE_MEM_RTC_SLOW, 0, "rtc_slow" },
        },
        .peripheral_start = 0x3FF00000u,
        .peripheral_end = 0x3FF80000u,
        .peripheral_alias_start = 0x60000000u,
        .peripheral_alias_end = 0x60040000u,
        .peripheral_alias_delta = -0x200C0000,
        .executable_range_count = 3,
        .executable = {
            { 0x40000000u, 0x400C2000u }, /* ROM, cache, IRAM, RTC fast */
            { 0x400D0000u, 0x40C00000u }, /* flash MMU instruction buses */
            { 0x50000000u, 0x50002000u }, /* RTC slow/reset vector 0 */
        },
    },
    {
        .descriptor_version = FLEXE_TARGET_DESCRIPTOR_VERSION,
        .id = FLEXE_TARGET_ESP32S3,
        .name = "esp32s3",
        .display_name = "ESP32-S3",
        .image_chip_id = 0x0009,
        .core_generation = FLEXE_XTENSA_LX7,
        .core_count = 2,
        .support_level = FLEXE_TARGET_EXPERIMENTAL,
        .capabilities = FLEXE_TARGET_CAP_ESP32S3_EXTMEM |
                        FLEXE_TARGET_CAP_DIRECT_ROM_DATA_INIT |
                        FLEXE_TARGET_CAP_SECONDARY_CORE_CONTROL |
                        FLEXE_TARGET_CAP_RTC_CALIBRATION |
                        FLEXE_TARGET_CAP_REGI2C |
                        FLEXE_TARGET_CAP_SENSITIVE_MEMPROT_V1 |
                        FLEXE_TARGET_CAP_SYSTIMER_V1 |
                        FLEXE_TARGET_CAP_SPI_MEM |
                        FLEXE_TARGET_CAP_INTERRUPT_MATRIX_V1 |
                        FLEXE_TARGET_CAP_USB_SERIAL_JTAG_V1 |
                        FLEXE_TARGET_CAP_TIMER_GROUP_V1,
        .reset_vector = 0x40000400u,
        .vecbase_reset = 0x40000000u,
        .configid0 = 0xC2F0FFFEu,
        .configid1 = 0x23090F1Fu,
        .default_cpu_frequency_mhz = 160u,
        .cpu_frequency_word = 0x3FCEF758u,
        .bootstrap_stack_top = { 0x3FCE0000u, 0x3FCF0000u },
        .interrupt_level = {
            1, 1, 1, 1, 1, 1, 1, 1,
            1, 1, 1, 3, 1, 1, 7, 3,
            5, 1, 1, 2, 2, 2, 3, 3,
            4, 4, 5, 3, 4, 3, 4, 5,
        },
        .iram_start = 0x40370000u,
        .iram_end = 0x403E0000u,
        .dram_start = 0x3FC88000u,
        .dram_end = 0x3FD00000u,
        .drom_start = 0x3C000000u,
        .drom_end = 0x3E000000u,
        .irom_start = 0x42000000u,
        .irom_end = 0x44000000u,
        .default_app_offset = 0x00010000u,
        .partition_table_offset = 0x00008000u,
        .flash_mmu = {
            .page_size = 0x00010000u,
            .entry_count = 512,
            .linear_addr_mask = 0x01FFFFFFu,
            .table_base = { 0x600C5000u, 0 },
            .invalid_entry = 0x00004000u,
            .invalid_mask = 0x00004000u,
            .physical_page_mask = 0x00003FFFu,
            .target_mask = 0x00008000u,
            .shared_instruction_data = true,
        },
        .cache_control_base = 0x600C4000u,
        .cache_control_size = 0x00001000u,
        .uart_count = 3,
        .uart_ip = {
            .register_size = 0x100u,
            .interrupt_valid_mask = 0x000FFFFFu,
            .interrupt_raw_reset = 1u << 1,
            .status_idle_value = (1u << 14) | (1u << 15) |
                                 (1u << 29) | (1u << 30) | (1u << 31),
            .rx_full_threshold_mask = 0x3FFu,
            .rx_timeout_enable_mask = 1u << 23,
            .mem_rx_status_offset = 0x68u,
            .fifo_address_mask = 0x3FFu,
            .mem_rx_read_shift = 0u,
            .mem_rx_write_shift = 11u,
            .date_offset = 0x7Cu,
            .date_reset = 0x02008270u,
        },
        .uart = {
            { .base = 0x60000000u, .interrupt_source = 27u },
            { .base = 0x60010000u, .interrupt_source = 28u },
            { .base = 0x6002E000u, .interrupt_source = 29u },
        },
        .secondary_core = {
            .base = 0x600C0000u,
            .register_size = 0x1000u,
            .control_offset = 0x000u,
            .boot_address_offset = 0x004u,
            .control_reset = 1u << 2,
            .reset_mask = 1u << 2,
            .clock_gate_mask = 1u << 1,
            .runstall_mask = 1u << 0,
        },
        .interrupt_matrix = {
            .base = 0x600C2000u,
            .register_size = 0x1000u,
            .source_count = 99u,
            .map_offset = { 0x000u, 0x800u },
            .status_offset = { 0x18Cu, 0x98Cu },
            .clock_gate_offset = { 0x19Cu, 0x99Cu },
            .date_offset = { 0x7FCu, 0xFFCu },
            .map_reset = 16u,
            .map_writable_mask = 0x1Fu,
            .clock_gate_reset = 1u,
            .clock_gate_writable_mask = 1u,
            .date_reset = 0x02012300u,
            .date_writable_mask = 0x0FFFFFFFu,
            .software_interrupt_base = 0x600C0000u,
            .software_interrupt_offset = 0x030u,
            .software_interrupt_stride = 4u,
            .software_interrupt_source_base = 79u,
            .software_interrupt_count = 4u,
            .software_interrupt_writable_mask = 1u,
        },
        .rtc_calibration = {
            .group_count = 2,
            .base = { 0x6001F000u, 0x60020000u },
            .register_size = 0x1000u,
            .config_offset = 0x068u,
            .value_offset = 0x06Cu,
            .timeout_offset = 0x080u,
            .config_reset = 0x00013000u,
            .timeout_reset = 0xFFFFFF98u,
            .config_writable_mask = 0xFFFF7000u,
            .timeout_writable_mask = 0xFFFFFFF8u,
            .start_mask = 1u << 31,
            .cycling_mask = 1u << 12,
            .ready_mask = 1u << 15,
            .timeout_mask = 1u << 0,
            .cycles_mask = 0x7FFFu << 16,
            .clock_select_mask = 3u << 13,
            .result_mask = 0x01FFFFFFu << 7,
            .cycles_shift = 16,
            .clock_select_shift = 13,
            .result_shift = 7,
            .reference_clock_hz = 40000000u,
            /* ESP-IDF's nominal RC_SLOW, RC_FAST/256, XTAL32K, and
             * INTERNAL_OSC selections for ESP32-S3 functional mode. */
            .source_clock_hz = { 136000u, 68359u, 32768u, 136000u },
        },
        .regi2c = {
            .base = 0x6000E000u,
            .register_size = 0x1000u,
            .host_count = 2,
            .command_offset = 0x000u,
            .command_stride = 0x004u,
            .analog_control_offset = 0x040u,
            .config_offset = 0x044u,
            .config2_offset = 0x048u,
            .analog_control_writable_mask = (1u << 2) | (1u << 3),
            .config_writable_mask = UINT32_MAX,
            .config2_writable_mask = UINT32_MAX,
            .command_start_mask = 1u << 26,
            .command_busy_mask = 1u << 25,
            .command_write_mask = 1u << 24,
            .slave_mask = 0xFFu,
            .address_mask = 0xFFu << 8,
            .data_mask = 0xFFu << 16,
            .slave_shift = 0,
            .address_shift = 8,
            .data_shift = 16,
            .bbpll_stop_high_mask = 1u << 2,
            .bbpll_stop_low_mask = 1u << 3,
            .bbpll_done_mask = 1u << 24,
        },
        .sensitive_memprot = {
            .base = 0x600C1000u,
            .register_size = 0x1000u,
        },
        .systimer = {
            .base = 0x60023000u,
            .register_size = 0x1000u,
            .counter_frequency_hz = 16000000u,
            .config_reset = 0x46000000u,
            .date_reset = 0x02012251u,
            .counter_count = 2u,
            .alarm_count = 3u,
            .counter_width = 52u,
            .interrupt_source = { 57u, 58u, 59u },
        },
        .timer_group = {
            .base = { 0x6001F000u, 0x60020000u },
            .register_size = 0x1000u,
            .apb_clock_hz = 80000000u,
            .xtal_clock_hz = 40000000u,
            .timer_config_reset = 0x60002000u,
            .timer_config_writable_mask = 0xFFFFE600u,
            .wdt_config_reset = {
                0x0004C000u,
                0x00010000u,
                0x018CBA80u,
                0x07FFFFFFu,
                0x000FFFFFu,
                0x000FFFFFu,
            },
            .wdt_config_writable_mask = {
                0xFF9FF000u,
                0xFFFF0000u,
                UINT32_MAX,
                UINT32_MAX,
                UINT32_MAX,
                UINT32_MAX,
            },
            .wdt_write_protect_key = 0x50D83AA1u,
            .date_reset = 0x02003071u,
            .date_writable_mask = 0x0FFFFFFFu,
            .regclk_reset = 0u,
            .regclk_writable_mask = 1u << 31,
            .group_count = 2u,
            .timer_count = 2u,
            .counter_width = 54u,
            .interrupt_source = {
                { 50u, 51u, 52u },
                { 53u, 54u, 55u },
            },
        },
        .spi_mem = {
            .base = { 0x60003000u, 0x60002000u },
            .register_size = 0x1000u,
            .default_jedec_id = 0x001640C8u,
            .date_reset = 0x02101040u,
            .host_count = 2u,
            .flash_chip_select = 0u,
            .psram_chip_select = FLEXE_SPI_MEM_CS_NONE,
            .layout = FLEXE_SPI_MEM_LAYOUT_S2_S3,
        },
        .usb_serial_jtag = {
            .base = 0x60038000u,
            .register_size = 0x1000u,
            .interrupt_source = 96u,
            .interrupt_valid_mask = 0x00000FFFu,
            .interrupt_raw_reset = 1u << 3,
            .conf0_reset = (1u << 14) | (1u << 9),
            .conf0_writable_mask = 0x0001FFFFu,
            .test_writable_mask = 0x0000000Fu,
            .misc_conf_writable_mask = 1u,
            .mem_conf_reset = 1u << 1,
            .mem_conf_writable_mask = 3u,
            .date_reset = 0x02101200u,
            .date_writable_mask = UINT32_MAX,
            .endpoint_size = 64u,
        },
        .backing_size = {
            [FLEXE_MEM_SRAM] = 0x00080000u,
            [FLEXE_MEM_ROM] = 0x00060000u,
            [FLEXE_MEM_FLASH_DATA] = 0x00400000u,
            [FLEXE_MEM_FLASH_INSN] = 0x00400000u,
            [FLEXE_MEM_RTC_FAST] = 0x00002000u,
            [FLEXE_MEM_RTC_SLOW] = 0x00002000u,
        },
        .memory_region_count = 9,
        .memory_region = {
            { 0x3C000000u, 0x3C400000u, FLEXE_MEM_FLASH_DATA, 0, "flash_data" },
            { 0x3FC88000u, 0x3FD00000u, FLEXE_MEM_SRAM, 0, "sram_data" },
            /* The S3 mask ROM exposes its final 128 KiB through the D-bus.
             * Espressif's ROM ELF gives .rodata a VMA in this window and an
             * LMA in the matching 0x4004_0000 instruction-ROM aperture. */
            { 0x3FF00000u, 0x3FF20000u, FLEXE_MEM_ROM, 0x00040000u, "rom_data" },
            { 0x40000000u, 0x40060000u, FLEXE_MEM_ROM, 0, "rom" },
            /* 0x40370000..0x40377fff is cache SRAM. The remaining IRAM
             * aliases 0x3fc88000..0x3fceffff byte-for-byte. */
            { 0x40370000u, 0x40378000u, FLEXE_MEM_SRAM, 0x00078000u, "sram_insn" },
            { 0x40378000u, 0x403E0000u, FLEXE_MEM_SRAM, 0, "sram_insn" },
            { 0x42000000u, 0x42400000u, FLEXE_MEM_FLASH_INSN, 0, "flash_insn" },
            { 0x50000000u, 0x50002000u, FLEXE_MEM_RTC_SLOW, 0, "rtc_slow" },
            { 0x600FE000u, 0x60100000u, FLEXE_MEM_RTC_FAST, 0, "rtc_fast" },
        },
        .peripheral_start = 0x60000000u,
        .peripheral_end = 0x600D1000u,
        .executable_range_count = 5,
        .executable = {
            { 0x40000000u, 0x40060000u }, /* mask ROM */
            { 0x40370000u, 0x403E0000u }, /* internal IRAM */
            { 0x42000000u, 0x44000000u }, /* flash/PSRAM MMU window */
            { 0x50000000u, 0x50002000u }, /* RTC slow/reset vector 0 */
            { 0x600FE000u, 0x60100000u }, /* RTC fast memory */
        },
    },
};

const flexe_target_desc_t *flexe_target_by_id(flexe_target_id_t id)
{
    for (unsigned i = 0; i < sizeof(TARGETS) / sizeof(TARGETS[0]); i++)
        if (TARGETS[i].id == id) return &TARGETS[i];
    return NULL;
}

const flexe_target_desc_t *flexe_target_by_image_chip_id(uint16_t chip_id)
{
    for (unsigned i = 0; i < sizeof(TARGETS) / sizeof(TARGETS[0]); i++)
        if (TARGETS[i].image_chip_id == chip_id) return &TARGETS[i];
    return NULL;
}

const flexe_target_desc_t *flexe_target_by_name(const char *name)
{
    if (!name) return NULL;
    for (unsigned i = 0; i < sizeof(TARGETS) / sizeof(TARGETS[0]); i++)
        if (strcmp(TARGETS[i].name, name) == 0) return &TARGETS[i];
    if (strcmp(name, "esp32-s3") == 0)
        return flexe_target_by_id(FLEXE_TARGET_ESP32S3);
    return NULL;
}

int flexe_target_parse(const char *name, flexe_target_id_t *id_out)
{
    if (!name || !id_out) return -1;
    if (strcmp(name, "auto") == 0) {
        *id_out = FLEXE_TARGET_AUTO;
        return 0;
    }
    const flexe_target_desc_t *target = flexe_target_by_name(name);
    if (!target) return -1;
    *id_out = target->id;
    return 0;
}

bool flexe_target_pc_is_executable(const flexe_target_desc_t *target,
                                   uint32_t pc)
{
    if (!target ||
        target->executable_range_count > FLEXE_TARGET_EXEC_RANGE_MAX)
        return false;
    for (unsigned i = 0; i < target->executable_range_count; i++) {
        const flexe_addr_range_t *range = &target->executable[i];
        if (range->end > range->start &&
            (uint32_t)(pc - range->start) < range->end - range->start)
            return true;
    }
    return false;
}

bool flexe_target_range_uses_backing(const flexe_target_desc_t *target,
                                     uint32_t addr, uint32_t size,
                                     flexe_mem_backing_t backing)
{
    uint64_t end = (uint64_t)addr + size;
    if (!target || backing >= FLEXE_MEM_BACKING_COUNT ||
        target->memory_region_count > FLEXE_TARGET_MEM_REGION_MAX ||
        end > (UINT64_C(1) << 32))
        return false;
    for (unsigned i = 0; i < target->memory_region_count; i++) {
        const flexe_target_mem_region_t *region = &target->memory_region[i];
        if (region->backing == backing && addr >= region->start &&
            end <= region->end)
            return true;
    }
    return false;
}

uint32_t flexe_target_bootstrap_stack(const flexe_target_desc_t *target,
                                      unsigned core)
{
    if (!target || core >= target->core_count || core >= 2u) return 0;
    return target->bootstrap_stack_top[core];
}
