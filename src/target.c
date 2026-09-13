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
                        FLEXE_TARGET_CAP_SPI_MEM |
                        FLEXE_TARGET_CAP_IO_MUX_V1 |
                        FLEXE_TARGET_CAP_I2C_V1 |
                        FLEXE_TARGET_CAP_RADIO_REGS_V1 |
                        FLEXE_TARGET_CAP_SHA_V1 |
                        FLEXE_TARGET_CAP_GP_SPI |
                        FLEXE_TARGET_CAP_ROM_FLASH_HANDOFF,
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
        .i2c = {
            .register_size = 0x104u,
            .date_reset = 0x16042000u,
            .interrupt_valid_mask = 0x00001FFFu,
            .interrupt_rxfifo_full_mask = 1u << 0,
            .interrupt_txfifo_empty_mask = 1u << 1,
            .interrupt_rxfifo_overflow_mask = 1u << 2,
            .interrupt_end_detect_mask = 1u << 3,
            .interrupt_slave_complete_mask = 1u << 4,
            .interrupt_command_done_mask = 1u << 6,
            .interrupt_transaction_complete_mask = 1u << 7,
            .interrupt_transaction_start_mask = 1u << 9,
            .interrupt_nack_mask = 1u << 10,
            .instance_count = 2u,
            .command_count = 16u,
            .opcode_restart = 0u,
            .opcode_write = 1u,
            .opcode_read = 2u,
            .opcode_stop = 3u,
            .opcode_end = 4u,
            .instance = {
                { .base = 0x3FF53000u, .interrupt_source = 49u },
                { .base = 0x3FF67000u, .interrupt_source = 50u },
            },
        },
        .io_mux = {
            .base = 0x3FF49000u,
            .register_size = 0x1000u,
            .register_count = 64u,
            .gpio_count = 40u,
            .function_shift = 12u,
            .control_offset = 0u,
            .control_reset = 0u,
            .control_writable_mask = 0x0000FFFFu,
            .register_reset = 0x00001800u,
            .register_writable_mask = 0x0000FFFFu,
            .function_mask = 0x00007000u,
            .date_offset = UINT32_MAX,
            .gpio_register_offset = {
                0x044u, 0x088u, 0x040u, 0x084u, 0x048u,
                0x06Cu, 0x060u, 0x064u, 0x068u, 0x054u,
                0x058u, 0x05Cu, 0x034u, 0x038u, 0x030u,
                0x03Cu, 0x04Cu, 0x050u, 0x070u, 0x074u,
                0x078u, 0x07Cu, 0x080u, 0x08Cu, 0x090u,
                0x024u, 0x028u, 0x02Cu,
                FLEXE_TARGET_IO_MUX_OFFSET_NONE,
                FLEXE_TARGET_IO_MUX_OFFSET_NONE,
                FLEXE_TARGET_IO_MUX_OFFSET_NONE,
                FLEXE_TARGET_IO_MUX_OFFSET_NONE,
                0x01Cu, 0x020u, 0x014u, 0x018u, 0x004u,
                0x008u, 0x00Cu, 0x010u,
            },
        },
        .radio = {
            .window_count = 10u,
            .completion_count = 2u,
            .window = {
                { 0x3FF45000u, 0x1000u }, /* FE2 */
                { 0x3FF46000u, 0x1000u }, /* FE */
                { 0x3FF4E000u, 0x1000u }, /* private PHY */
                { 0x3FF51000u, 0x1000u }, /* BT */
                { 0x3FF5C000u, 0x1000u }, /* private NRX + NRX */
                { 0x3FF5D000u, 0x1000u }, /* BB */
                { 0x3FF71000u, 0x1000u }, /* private BT */
                { 0x3FF72000u, 0x1000u }, /* BT MAC */
                { 0x3FF73000u, 0x2000u }, /* Wi-Fi MAC */
                { 0x3FF75000u, 0x1000u }, /* WDEV */
            },
            .completion = {
                /* Wi-Fi MAC reset request bit 1 reports ready in bit 0. */
                {
                    .control_address = 0x3FF73D24u,
                    .active_mask = 1u << 1,
                    .status_address = 0x3FF73D24u,
                    .status_mask = 1u << 0,
                },
                /* Indexed PHY calibration command is consumed on write. */
                {
                    .control_address = 0x3FF4E0C4u,
                    .self_clear_mask = UINT32_MAX,
                },
            },
            .random_address = 0x3FF75144u,
            .random_seed = UINT64_C(0x12345678ABCDEF01),
        },
        .sha = {
            .base = 0x3FF03000u,
            .register_size = 0x1000u,
            .layout = FLEXE_SHA_LAYOUT_ESP32,
            .mode_count = 4u,
            .dma_peripheral_id = UINT8_MAX,
            .mode = {
                FLEXE_SHA_ALGORITHM_SHA1,
                FLEXE_SHA_ALGORITHM_SHA256,
                FLEXE_SHA_ALGORITHM_SHA384,
                FLEXE_SHA_ALGORITHM_SHA512,
            },
        },
        .gp_spi = {
            .register_size = 0x1000u,
            .date_reset = 0x01604270u,
            .host_count = 2u,
            .layout = FLEXE_GP_SPI_LAYOUT_ESP32,
            .instance = {
                {
                    .base = 0x3FF64000u,
                    .clock_out_signal = 8u,
                    .chip_select_out_signal = {
                        11u, 61u, 62u,
                        FLEXE_TARGET_MATRIX_SIGNAL_NONE,
                        FLEXE_TARGET_MATRIX_SIGNAL_NONE,
                        FLEXE_TARGET_MATRIX_SIGNAL_NONE,
                    },
                    .interrupt_source = 30u,
                    .chip_select_count = 3u,
                    .iomux_clock_pin = 14u,
                    .iomux_chip_select0_pin = 15u,
                    .iomux_function = 1u,
                    .gdma_peripheral_id = FLEXE_TARGET_GDMA_PERIPHERAL_NONE,
                },
                {
                    .base = 0x3FF65000u,
                    .clock_out_signal = 63u,
                    .chip_select_out_signal = {
                        68u, 69u, 70u,
                        FLEXE_TARGET_MATRIX_SIGNAL_NONE,
                        FLEXE_TARGET_MATRIX_SIGNAL_NONE,
                        FLEXE_TARGET_MATRIX_SIGNAL_NONE,
                    },
                    .interrupt_source = 31u,
                    .chip_select_count = 3u,
                    .iomux_clock_pin = 18u,
                    .iomux_chip_select0_pin = 5u,
                    .iomux_function = 1u,
                    .gdma_peripheral_id = FLEXE_TARGET_GDMA_PERIPHERAL_NONE,
                },
            },
        },
        .spi_mem = {
            .base = { 0x3FF43000u, 0x3FF42000u },
            .register_size = 0x1000u,
            .default_jedec_id = 0x001640C8u,
            .maximum_flash_size = 0x01000000u,
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
        .rom_flash = {
            .live_data_address = 0x3FFAE270u,
            .struct_size = 24u,
            .device_id_offset = 0u,
            .chip_size_offset = 4u,
            .block_size_offset = 8u,
            .sector_size_offset = 12u,
            .page_size_offset = 16u,
            .status_mask_offset = 20u,
            .block_size = 0x00010000u,
            .sector_size = 0x00001000u,
            .page_size = 0x00000100u,
            .status_mask = 0x0000FFFFu,
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
                        FLEXE_TARGET_CAP_TIMER_GROUP_V1 |
                        FLEXE_TARGET_CAP_SYSTEM_CLOCK_V1 |
                        FLEXE_TARGET_CAP_IO_MUX_V1 |
                        FLEXE_TARGET_CAP_RTC_CNTL_V1 |
                        FLEXE_TARGET_CAP_EFUSE_READ_V1 |
                        FLEXE_TARGET_CAP_GPIO_V1 |
                        FLEXE_TARGET_CAP_I2C_V1 |
                        FLEXE_TARGET_CAP_SENS_V1 |
                        FLEXE_TARGET_CAP_RADIO_REGS_V1 |
                        FLEXE_TARGET_CAP_GDMA_V1 |
                        FLEXE_TARGET_CAP_SHA_V1 |
                        FLEXE_TARGET_CAP_GP_SPI |
                        FLEXE_TARGET_CAP_RMT_V1 |
                        FLEXE_TARGET_CAP_ROM_FLASH_HANDOFF,
        .reset_vector = 0x40000400u,
        .vecbase_reset = 0x40000000u,
        .configid0 = 0xC2F0FFFEu,
        .configid1 = 0x23090F1Fu,
        .default_cpu_frequency_mhz = 160u,
        .cpu_frequency_word = 0x3FCEF758u,
        /* ESP32-S3 rev-0 ROM linker symbols __stack and __stack_app.
         * Application handoff still runs the PRO/APP startup paths before
         * FreeRTOS installs task stacks, so both cores need the ROM-owned
         * 8-KiB startup ranges. 0x3FCED710 and above is ROM static state;
         * placing APP_CPU at the old 0x3FCF0000 boundary let its first LX7
         * window spill overwrite the ROM ABI interface immediately below. */
        .bootstrap_stack_top = { 0x3FCEB710u, 0x3FCED710u },
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
        .i2c = {
            .register_size = 0x184u,
            .date_reset = 0x20070201u,
            .interrupt_valid_mask = 0x0003FFFFu,
            .interrupt_rxfifo_full_mask = 1u << 0,
            .interrupt_txfifo_empty_mask = 1u << 1,
            .interrupt_rxfifo_overflow_mask = 1u << 2,
            .interrupt_end_detect_mask = 1u << 3,
            .interrupt_command_done_mask = 1u << 4,
            .interrupt_transaction_complete_mask = 1u << 7,
            .interrupt_transaction_start_mask = 1u << 9,
            .interrupt_nack_mask = 1u << 10,
            .instance_count = 2u,
            .command_count = 8u,
            /* ESP32-S3's HAL command encoding is 6/1/3/2/4, unlike the
             * classic ESP32's otherwise similar FIFO command front end. */
            .opcode_restart = 6u,
            .opcode_write = 1u,
            .opcode_read = 3u,
            .opcode_stop = 2u,
            .opcode_end = 4u,
            .instance = {
                { .base = 0x60013000u, .interrupt_source = 42u },
                { .base = 0x60027000u, .interrupt_source = 43u },
            },
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
        .system_clock = {
            .base = 0x600C0000u,
            .register_size = 0x1000u,
            .cpu_per_conf_offset = 0x010u,
            .cpu_per_conf_reset = 0x0000000Cu,
            .cpu_per_conf_writable_mask = 0x000000FFu,
            .sysclk_conf_offset = 0x060u,
            .sysclk_conf_reset = 0x00000001u,
            .sysclk_conf_writable_mask = 0x00000FFFu,
            .register_count = 7u,
            .gate_count = 8u,
            .reg = {
                { .offset = 0x014u, .reset = 0x00000001u,
                  .writable_mask = 0x00000001u },
                { .offset = 0x018u, .reset = 0xF9C1E06Fu,
                  .writable_mask = UINT32_MAX },
                { .offset = 0x01Cu, .reset = 0x00000600u,
                  .writable_mask = 0x000007FFu },
                { .offset = 0x020u, .reset = 0x00000000u,
                  .writable_mask = UINT32_MAX },
                { .offset = 0x024u, .reset = 0x000001FEu,
                  .writable_mask = 0x000007FFu },
                { .offset = 0x028u, .reset = 0x000000FFu,
                  .writable_mask = 0x00000FFFu },
                { .offset = 0x02Cu, .reset = 0x02001001u,
                  .writable_mask = 0x1FFFFFFFu },
            },
            .gate = {
                {
                    .device = FLEXE_SYSTEM_DEVICE_SYSTIMER,
                    .clock_offset = 0x018u,
                    .reset_offset = 0x020u,
                    .clock_mask = 1u << 29,
                    .reset_mask = 1u << 29,
                },
                {
                    .device = FLEXE_SYSTEM_DEVICE_TIMER_GROUP,
                    .instance = 0u,
                    .clock_offset = 0x018u,
                    .reset_offset = 0x020u,
                    .clock_mask = 1u << 13,
                    .reset_mask = 1u << 13,
                },
                {
                    .device = FLEXE_SYSTEM_DEVICE_TIMER_GROUP,
                    .instance = 1u,
                    .clock_offset = 0x018u,
                    .reset_offset = 0x020u,
                    .clock_mask = 1u << 15,
                    .reset_mask = 1u << 15,
                },
                {
                    .device = FLEXE_SYSTEM_DEVICE_I2C,
                    .instance = 0u,
                    .clock_offset = 0x018u,
                    .reset_offset = 0x020u,
                    .clock_mask = 1u << 7,
                    .reset_mask = 1u << 7,
                },
                {
                    .device = FLEXE_SYSTEM_DEVICE_I2C,
                    .instance = 1u,
                    .clock_offset = 0x018u,
                    .reset_offset = 0x020u,
                    .clock_mask = 1u << 18,
                    .reset_mask = 1u << 18,
                },
                {
                    .device = FLEXE_SYSTEM_DEVICE_GP_SPI,
                    .instance = 0u,
                    .clock_offset = 0x018u,
                    .reset_offset = 0x020u,
                    .clock_mask = 1u << 6,
                    .reset_mask = 1u << 6,
                },
                {
                    .device = FLEXE_SYSTEM_DEVICE_GP_SPI,
                    .instance = 1u,
                    .clock_offset = 0x018u,
                    .reset_offset = 0x020u,
                    .clock_mask = 1u << 16,
                    .reset_mask = 1u << 16,
                },
                {
                    .device = FLEXE_SYSTEM_DEVICE_SHA,
                    .instance = 0u,
                    .clock_offset = 0x01Cu,
                    .reset_offset = 0x024u,
                    .clock_mask = 1u << 2,
                    .reset_mask = 1u << 2,
                },
            },
        },
        .io_mux = {
            .base = 0x60009000u,
            .register_size = 0x1000u,
            .register_count = 64u,
            .gpio_count = 49u,
            .function_shift = 12u,
            .control_offset = 0u,
            .control_reset = 0u,
            .control_writable_mask = 0x0000FFFFu,
            .register_reset = 0u,
            .register_writable_mask = 0x0000FFFFu,
            .function_mask = 0x00007000u,
            .date_offset = 0x0FCu,
            .date_reset = 0x01907160u,
            .gpio_register_offset = {
                0x004u, 0x008u, 0x00Cu, 0x010u, 0x014u,
                0x018u, 0x01Cu, 0x020u, 0x024u, 0x028u,
                0x02Cu, 0x030u, 0x034u, 0x038u, 0x03Cu,
                0x040u, 0x044u, 0x048u, 0x04Cu, 0x050u,
                0x054u, 0x058u,
                FLEXE_TARGET_IO_MUX_OFFSET_NONE,
                FLEXE_TARGET_IO_MUX_OFFSET_NONE,
                FLEXE_TARGET_IO_MUX_OFFSET_NONE,
                FLEXE_TARGET_IO_MUX_OFFSET_NONE,
                0x06Cu, 0x070u, 0x074u, 0x078u, 0x07Cu,
                0x080u, 0x084u, 0x088u, 0x08Cu, 0x090u,
                0x094u, 0x098u, 0x09Cu, 0x0A0u, 0x0A4u,
                0x0A8u, 0x0ACu, 0x0B0u, 0x0B4u, 0x0B8u,
                0x0BCu, 0x0C0u, 0x0C4u,
            },
        },
        .gpio = {
            .base = 0x60004000u,
            .register_size = 0x1000u,
            /* ESP32-S3 has GPIO0..48 with package holes at GPIO22..25. */
            .valid_gpio_mask = UINT64_C(0x0001FFFFFC3FFFFF),
            .strap_reset = 0u,
            .date_reset = 0x01907040u,
            .gpio_count = 49u,
            .matrix_const_one_input = 0x38u,
            .matrix_const_zero_input = 0x3Cu,
            .interrupt_source = 16u,
            .nmi_interrupt_source = 17u,
        },
        .rtc_cntl = {
            .base = 0x60008000u,
            .register_size = 0x1000u,
            .store_count = 8u,
            .slow_clock_cal_store = 1u,
            .xtal_frequency_store = 4u,
            .slow_clock_hz = 136000u,
            .xtal_frequency_mhz = 40u,
            .store_offset = {
                0x050u, 0x054u, 0x058u, 0x05Cu,
                0x0C0u, 0x0C4u, 0x0C8u, 0x0CCu,
            },
            .time_update_offset = 0x00Cu,
            .time_low_offset = 0x010u,
            .time_high_offset = 0x014u,
            .reset_state_offset = 0x038u,
            .clock_conf_offset = 0x074u,
            .slow_clock_select_shift = 30u,
            .time_update_mask = 0x80000000u,
            .time_high_mask = 0x0000FFFFu,
            /* PRO/APPCPU state-vector selection reset bits plus a power-on
             * reset cause in each six-bit core field. */
            .reset_state_reset = 0x00003041u,
            .clock_conf_reset = 0x1158321Cu,
            .clock_conf_writable_mask = 0xFFFFF7FEu,
            .slow_clock_select_mask = 0xC0000000u,
            /* RC_SLOW, XTAL32K, and RC_FAST/256 nominal frequencies. The
             * fourth mux encoding is reserved by the S3 clock-tree HAL. */
            .slow_clock_source_hz = {
                136000u, 32768u, 68359u, 0u,
            },
            .interrupt_enable_offset = 0x040u,
            .interrupt_raw_offset = 0x044u,
            .interrupt_status_offset = 0x048u,
            .interrupt_clear_offset = 0x04Cu,
            .interrupt_enable_reset = 0u,
            .interrupt_raw_reset = 0u,
            .interrupt_valid_mask = 0x001FFFFFu,
            /* TOUCH_APPROACH_LOOP_DONE is the sole software-writable raw
             * source on S3; device producers update all valid raw bits
             * through the controller API. */
            .interrupt_raw_writable_mask = 1u << 20,
            .interrupt_source = 39u,
            .wdt_config_offset = {
                0x098u, 0x09Cu, 0x0A0u, 0x0A4u, 0x0A8u,
            },
            .wdt_feed_offset = 0x0ACu,
            .wdt_write_protect_offset = 0x0B0u,
            .wdt_config_reset = {
                0x00013214u, 200000u, 80000u, 0xFFFu, 0xFFFu,
            },
            .wdt_config_writable_mask = {
                UINT32_MAX, UINT32_MAX, UINT32_MAX,
                UINT32_MAX, UINT32_MAX,
            },
            .wdt_enable_mask = 1u << 31,
            .wdt_flashboot_enable_mask = 1u << 12,
            .wdt_feed_mask = 1u << 31,
            .wdt_write_protect_key = 0x50D83AA1u,
            .wdt_interrupt_mask = 1u << 3,
            .wdt_stage_action_shift = { 28u, 25u, 22u, 19u },
            .wdt_stage_action_mask = 0x7u,
            /* Revision-0 WDT_DELAY_SEL is zero, making stage 0 count two
             * slow-clock ticks for each stored hold unit. */
            .wdt_stage0_multiplier = 2u,
            /* ESP32-S3 gpio_periph.c maps GPIO21..47 to bits 1..27 of
             * RTC_CNTL_DIG_PAD_HOLD_REG; GPIO22..25 are unbonded. */
            .digital_pad_hold_offset = 0x0DCu,
            .digital_pad_hold_first_gpio = 21u,
            .digital_pad_hold_first_bit = 1u,
            .digital_pad_hold_count = 27u,
        },
        .efuse = {
            .base = 0x60007000u,
            .register_size = 0x1000u,
            .read_data_offset = 0x02Cu,
            .read_data_word_count = 84u,
            .date_offset = 0x1FCu,
            .date_reset = 0x02101290u,
            .date_writable_mask = 0x0FFFFFFFu,
            /* ESP32-S3 revision 0.0, no embedded flash/PSRAM, and the
             * locally administered unicast MAC 02:00:00:00:00:01. */
            .read_data = {
                [6] = 0x00000001u, /* RD_MAC_SPI_SYS_0: MAC bits 31:0 */
                [7] = 0x00000200u, /* RD_MAC_SPI_SYS_1: MAC bits 47:32 */
            },
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
            .aux_register_count = 2u,
            .aux_register = {
                /* PHY TX-DC calibration command/result. Bit 24 reports
                 * completion; fast mode resolves the conversion immediately
                 * while retaining the 24-bit software command payload. */
                { 0x04Cu, 0x01000000u, 0x00FFFFFFu },
                /* S3 ROM SAR2 power/peak-detector controller. The three-bit
                 * FSM status is idle/ready at reset; functional fast mode
                 * completes its internal analog conversion synchronously. */
                { 0x050u, 0x07000000u, 0xF8FFFFFFu },
            },
        },
        .sens = {
            .base = 0x60008800u,
            .register_size = 0x200u,
            .control_offset = 0x050u,
            .control2_offset = 0x054u,
            .clock_gate_offset = 0x104u,
            .reset_offset = 0x108u,
            .control_reset = 0x00019000u,
            .control2_reset = 0x00004002u,
            .control_writable_mask = 0x01FFF000u,
            .control2_writable_mask = 0x00007FFFu,
            .clock_gate_writable_mask = 0xE8000000u,
            .reset_writable_mask = 0x6A000000u,
            .dump_out_mask = 1u << 24,
            .power_up_force_mask = 1u << 23,
            .power_up_mask = 1u << 22,
            .input_invert_mask = 1u << 13,
            .interrupt_enable_mask = 1u << 12,
            .ready_mask = 1u << 8,
            .output_mask = 0xFFu,
            .xpd_force_mask = 3u << 12,
            .clock_enable_mask = 1u << 29,
            .reset_mask = 1u << 29,
            .rtc_interrupt_mask = 1u << 12,
            /* Raw code 104 is approximately 25 C in range 2 (offset 0). */
            .default_output = 104u,
        },
        .radio = {
            .window_count = 9u,
            .completion_count = 2u,
            .window = {
                { 0x60005000u, 0x1000u }, /* FE2 */
                { 0x60006000u, 0x1000u }, /* FE */
                { 0x60011000u, 0x1000u }, /* BT */
                { 0x6001C000u, 0x1000u }, /* private NRX + NRX */
                { 0x6001D000u, 0x1000u }, /* BB */
                { 0x60031000u, 0x1000u }, /* private BT */
                { 0x60032000u, 0x1000u }, /* BT MAC */
                { 0x60033000u, 0x2000u }, /* Wi-Fi MAC */
                { 0x60035000u, 0x1000u }, /* WDEV */
            },
            .completion = {
                /* ESP32-S3 rev-0 ROM rom_iq_est_enable writes enable bits
                 * 0 then 1 at FE+0x144 and polls FE+0x174 bit 16. A quiet
                 * virtual RF input leaves the three result accumulators at
                 * zero while making the digital completion protocol exact. */
                {
                    .control_address = 0x60006144u,
                    .active_mask = (1u << 1) | (1u << 0),
                    .status_address = 0x60006174u,
                    .status_mask = 1u << 16,
                },
                /* The S3 Wi-Fi HAL asserts MAC reset bit 1 and waits for
                 * the controller's ready response in bit 0. */
                {
                    .control_address = 0x60033D14u,
                    .active_mask = 1u << 1,
                    .status_address = 0x60033D14u,
                    .status_mask = 1u << 0,
                },
            },
            /* WDEV_RND_REG from the public ESP32-S3 register header. */
            .random_address = 0x6003507Cu,
            .random_seed = UINT64_C(0x12345678ABCDEF01),
            .time_latch = {
                .count_address = 0x6003101Cu,
                .phase_address = 0x60031020u,
                .capture_mask = 1u << 31,
                .count_mask = 0x0FFFFFFFu,
                .tick_hz = 2000000u,
                .ticks_per_half_slot = 625u,
            },
        },
        .gdma = {
            .base = 0x6003F000u,
            .register_size = 0x1000u,
            .channel_stride = 0x0C0u,
            .descriptor_address_prefix = 0x3FC00000u,
            .channel_count = 5u,
        },
        .sha = {
            .base = 0x6003B000u,
            .register_size = 0x1000u,
            .layout = FLEXE_SHA_LAYOUT_UNIFIED,
            .mode_count = 8u,
            .dma_peripheral_id = 7u,
            .mode = {
                FLEXE_SHA_ALGORITHM_SHA1,
                FLEXE_SHA_ALGORITHM_SHA224,
                FLEXE_SHA_ALGORITHM_SHA256,
                FLEXE_SHA_ALGORITHM_SHA384,
                FLEXE_SHA_ALGORITHM_SHA512,
                FLEXE_SHA_ALGORITHM_SHA512_224,
                FLEXE_SHA_ALGORITHM_SHA512_256,
                FLEXE_SHA_ALGORITHM_SHA512_T,
            },
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
        .rmt_v1 = {
            .base = 0x60016000u,
            .register_size = 0x1000u,
            .memory_offset = 0x800u,
            .tx_channel_count = 4u,
            .channel_count = 8u,
            .words_per_channel = 48u,
            .interrupt_source = 40u,
            .apb_clock_hz = 80000000u,
            .ref_clock_hz = 8000000u,
            .xtal_clock_hz = 40000000u,
        },
        .gp_spi = {
            .register_size = 0x100u,
            .date_reset = 0x02101190u,
            .host_count = 2u,
            .layout = FLEXE_GP_SPI_LAYOUT_S2_S3,
            .instance = {
                {
                    .base = 0x60024000u,
                    .clock_out_signal = 101u,
                    .chip_select_out_signal = {
                        110u, 111u, 112u, 113u, 114u, 115u,
                    },
                    .interrupt_source = 21u,
                    .chip_select_count = 6u,
                    .iomux_clock_pin = 12u,
                    .iomux_chip_select0_pin = 10u,
                    .iomux_function = 4u,
                    .gdma_peripheral_id = 0u,
                },
                {
                    .base = 0x60025000u,
                    .clock_out_signal = 66u,
                    .chip_select_out_signal = {
                        71u, 72u, 127u,
                        FLEXE_TARGET_MATRIX_SIGNAL_NONE,
                        FLEXE_TARGET_MATRIX_SIGNAL_NONE,
                        FLEXE_TARGET_MATRIX_SIGNAL_NONE,
                    },
                    .interrupt_source = 22u,
                    .chip_select_count = 3u,
                    .iomux_clock_pin = FLEXE_TARGET_GPIO_NONE,
                    .iomux_chip_select0_pin = FLEXE_TARGET_GPIO_NONE,
                    .iomux_function = FLEXE_TARGET_GPIO_NONE,
                    .gdma_peripheral_id = 1u,
                },
            },
        },
        .spi_mem = {
            .base = { 0x60003000u, 0x60002000u },
            .register_size = 0x1000u,
            .default_jedec_id = 0x001640C8u,
            .maximum_flash_size = 0x08000000u,
            .date_reset = 0x02101040u,
            .host_count = 2u,
            .flash_chip_select = 0u,
            .psram_chip_select = FLEXE_SPI_MEM_CS_NONE,
            .layout = FLEXE_SPI_MEM_LAYOUT_S2_S3,
        },
        .rom_flash = {
            .pointer_symbol = "rom_spiflash_legacy_data",
            .struct_size = 24u,
            .device_id_offset = 0u,
            .chip_size_offset = 4u,
            .block_size_offset = 8u,
            .sector_size_offset = 12u,
            .page_size_offset = 16u,
            .status_mask_offset = 20u,
            .block_size = 0x00010000u,
            .sector_size = 0x00001000u,
            .page_size = 0x00000100u,
            .status_mask = 0x0000FFFFu,
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
