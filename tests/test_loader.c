/*
 * Tests for the ESP32 .bin loader.
 */
#include "test_helpers.h"
#include "loader.h"
#include "peripherals.h"
#include "flash_mmu.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

/* Helper: write a little-endian uint32_t to a buffer */
static void put_le32(uint8_t *buf, uint32_t val) {
    buf[0] = (uint8_t)(val);
    buf[1] = (uint8_t)(val >> 8);
    buf[2] = (uint8_t)(val >> 16);
    buf[3] = (uint8_t)(val >> 24);
}

static void put_le16(uint8_t *buf, uint16_t val) {
    buf[0] = (uint8_t)val;
    buf[1] = (uint8_t)(val >> 8);
}

static uint32_t get_le32(const uint8_t *buf) {
    return (uint32_t)buf[0] |
           ((uint32_t)buf[1] << 8) |
           ((uint32_t)buf[2] << 16) |
           ((uint32_t)buf[3] << 24);
}

/* Helper: create a temp file with given contents, return path */
static const char *write_temp(const uint8_t *data, size_t len) {
    static char path[256];
    snprintf(path, sizeof(path), "/tmp/xt_test_loader.bin");
    FILE *f = fopen(path, "wb");
    if (!f) return NULL;
    fwrite(data, 1, len, f);
    fclose(f);
    return path;
}

TEST(loader_single_segment) {
    /* Build a minimal .bin: 24-byte header + 1 segment */
    uint8_t bin[64];
    memset(bin, 0, sizeof(bin));
    bin[0] = 0xE9;         /* magic */
    bin[1] = 1;            /* 1 segment */
    put_le32(&bin[4], 0x40080000); /* entry point */
    /* Segment at offset 24: load_addr, data_len, data */
    put_le32(&bin[24], 0x3FFB0000);  /* load to SRAM data */
    put_le32(&bin[28], 4);           /* 4 bytes of data */
    put_le32(&bin[32], 0xDEADBEEF);  /* the data */

    const char *path = write_temp(bin, 36);
    ASSERT_TRUE(path != NULL);

    xtensa_mem_t *mem = mem_create();
    load_result_t res = loader_load_bin(mem, path);
    ASSERT_EQ(res.result, 0);
    ASSERT_EQ(res.entry_point, 0x40080000);
    ASSERT_EQ(res.segment_count, 1);
    ASSERT_EQ(res.image.target, FLEXE_TARGET_ESP32);
    ASSERT_EQ(res.image.chip_id, 0);
    ASSERT_EQ(mem_read32(mem, 0x3FFB0000), 0xDEADBEEF);
    mem_destroy(mem);
}

TEST(loader_reports_image_revision_metadata) {
    uint8_t bin[64] = {0};
    bin[0] = 0xE9;
    bin[1] = 1;
    bin[3] = 0x32; /* 8 MiB capacity, 40 MHz speed */
    put_le32(&bin[4], 0x40080000u);
    put_le16(&bin[12], 0x0000u);
    bin[14] = 3;
    put_le16(&bin[15], 301u);
    put_le16(&bin[17], 399u);
    put_le32(&bin[24], 0x3FFB0000u);
    put_le32(&bin[28], 4u);
    put_le32(&bin[32], 0x12345678u);

    const char *path = write_temp(bin, 36);
    ASSERT_TRUE(path != NULL);
    loader_image_info_t info;
    char error[256];
    ASSERT_EQ(loader_probe_bin(path, &info, error, sizeof(error)), 0);
    ASSERT_EQ(info.target, FLEXE_TARGET_ESP32);
    ASSERT_EQ(info.min_chip_rev, 3u);
    ASSERT_EQ(info.min_chip_rev_full, 301u);
    ASSERT_EQ(info.max_chip_rev_full, 399u);
    ASSERT_EQ(info.image_offset, 0u);
    ASSERT_EQ(info.flash_size, 8u * 1024u * 1024u);
}

TEST(loader_rejects_reserved_flash_capacity) {
    uint8_t bin[24] = {0};
    bin[0] = 0xE9;
    bin[1] = 0;
    bin[3] = 0x80;

    const char *path = write_temp(bin, sizeof(bin));
    ASSERT_TRUE(path != NULL);
    loader_image_info_t info;
    char error[256];
    ASSERT_EQ(loader_probe_bin(path, &info, error, sizeof(error)), -1);
    ASSERT_TRUE(strstr(error, "flash-size code 0x8") != NULL);
}

TEST(loader_rejects_app_larger_than_declared_flash) {
    const size_t bin_size = 0xF0001u;
    uint8_t *bin = calloc(1, bin_size);
    ASSERT_TRUE(bin != NULL);
    if (!bin) return;
    bin[0] = 0xE9;
    bin[1] = 0;
    bin[3] = 0x00; /* 1 MiB, including the 0x10000 app offset */

    const char *path = write_temp(bin, bin_size);
    free(bin);
    ASSERT_TRUE(path != NULL);
    if (!path) return;

    xtensa_mem_t *mem = mem_create();
    load_result_t res = loader_load_bin(mem, path);
    ASSERT_EQ(res.result, -1);
    ASSERT_TRUE(strstr(res.error, "declared flash capacity 1048576") != NULL);
    mem_destroy(mem);
}

TEST(loader_recognizes_s3_before_rejecting_classic_memory) {
    uint8_t bin[64] = {0};
    bin[0] = 0xE9;
    bin[1] = 1;
    put_le32(&bin[4], 0x40370000u);
    put_le16(&bin[12], 0x0009u);
    put_le32(&bin[24], 0x3FC88000u);
    put_le32(&bin[28], 4u);
    put_le32(&bin[32], 0xA5A55A5Au);

    const char *path = write_temp(bin, 36);
    ASSERT_TRUE(path != NULL);
    loader_image_info_t info;
    char error[256];
    ASSERT_EQ(loader_probe_bin(path, &info, error, sizeof(error)), 0);
    ASSERT_EQ(info.target, FLEXE_TARGET_ESP32S3);
    ASSERT_EQ(info.chip_id, 9u);

    xtensa_mem_t *mem = mem_create();
    load_result_t res = loader_load_bin(mem, path);
    ASSERT_EQ(res.result, -1);
    ASSERT_TRUE(strstr(res.error,
                       "memory is configured for ESP32") != NULL);
    ASSERT_EQ(mem_read32(mem, 0x3FFB0000u), 0u);
    mem_destroy(mem);
}

TEST(loader_loads_s3_segments_through_shared_flash_mmu) {
    uint8_t bin[96] = {0};
    bin[0] = 0xE9;
    bin[1] = 4;
    put_le32(&bin[4], 0x40374000u);
    put_le16(&bin[12], 0x0009u);

    /* DROM data starts at image offset 0x20, matching vaddr low bits. */
    put_le32(&bin[24], 0x3C020020u);
    put_le32(&bin[28], 4u);
    put_le32(&bin[32], 0x11111111u);
    /* Internal DRAM and cache SRAM are copied directly. */
    put_le32(&bin[36], 0x3FC92300u);
    put_le32(&bin[40], 4u);
    put_le32(&bin[44], 0x22222222u);
    put_le32(&bin[48], 0x40374000u);
    put_le32(&bin[52], 4u);
    put_le32(&bin[56], 0x33333333u);
    /* This segment's data is at image offset 0x44, so preserve the S3 MMU's
     * required physical/virtual low-bit congruence. */
    put_le32(&bin[60], 0x42000044u);
    put_le32(&bin[64], 4u);
    put_le32(&bin[68], 0x44444444u);

    const char *path = write_temp(bin, 72);
    ASSERT_TRUE(path != NULL);
    const flexe_target_desc_t *s3 =
        flexe_target_by_id(FLEXE_TARGET_ESP32S3);
    xtensa_mem_t *mem = mem_create_for_target(s3);
    ASSERT_TRUE(mem != NULL);
    if (!mem) return;
    flexe_flash_mmu_t *mmu = flexe_flash_mmu_create(mem);
    ASSERT_TRUE(mmu != NULL);
    if (!mmu) {
        mem_destroy(mem);
        return;
    }

    load_result_t res = loader_load_bin(mem, path);
    ASSERT_EQ(res.result, 0);
    ASSERT_EQ(res.image.target, FLEXE_TARGET_ESP32S3);
    ASSERT_EQ(res.entry_point, 0x40374000u);
    ASSERT_EQ(res.segment_count, 4);
    ASSERT_EQ(mem_read32(mem, 0x3C020020u), 0x11111111u);
    ASSERT_EQ(mem_read32(mem, 0x3FC92300u), 0x22222222u);
    ASSERT_EQ(mem_read32(mem, 0x40374000u), 0x33333333u);
    ASSERT_EQ(mem_read32(mem, 0x42000044u), 0x44444444u);

    /* One S3 MMU entry is visible through both cache buses. */
    ASSERT_TRUE(mem_get_ptr(mem, 0x3C000000u) ==
                mem->flash_data + 0x10000u);
    ASSERT_TRUE(mem_get_ptr(mem, 0x42020000u) ==
                mem->flash_insn + 0x10000u);
    ASSERT_TRUE(mem_get_ptr(mem, 0x3C010000u) == NULL);
    ASSERT_TRUE(mem_get_ptr(mem, 0x42010000u) == NULL);
    ASSERT_EQ(mem_read32(mem, 0x600C5000u), 1u);
    ASSERT_EQ(mem_read32(mem, 0x600C5004u), 0x4000u);
    ASSERT_EQ(mem_read32(mem, 0x600C5008u), 1u);
    flexe_flash_mmu_destroy(mmu);
    mem_destroy(mem);
}

TEST(loader_maps_classic_flash_without_peripheral_model) {
    uint8_t bin[64] = {0};
    bin[0] = 0xE9;
    bin[1] = 2;
    put_le32(&bin[4], 0x400D002Cu);

    /* Segment data begins at image offset 0x20. */
    put_le32(&bin[24], 0x3F400020u);
    put_le32(&bin[28], 4u);
    put_le32(&bin[32], 0x11223344u);
    /* The second payload begins at image offset 0x2c. */
    put_le32(&bin[36], 0x400D002Cu);
    put_le32(&bin[40], 4u);
    put_le32(&bin[44], 0x55667788u);

    const char *path = write_temp(bin, 48);
    ASSERT_TRUE(path != NULL);
    xtensa_mem_t *mem = mem_create();
    ASSERT_TRUE(mem != NULL);
    if (!mem) return;

    /* No esp32_periph_t is attached: the loader must still produce the same
     * boot mappings, without manufacturing unmapped guest accesses. */
    load_result_t res = loader_load_bin(mem, path);
    ASSERT_EQ(res.result, 0);
    ASSERT_EQ(mem_read32(mem, 0x3F400020u), 0x11223344u);
    ASSERT_EQ(mem_read32(mem, 0x400D002Cu), 0x55667788u);
    ASSERT_TRUE(mem_get_ptr(mem, 0x3F400000u) ==
                mem->flash_data + 0x10000u);
    ASSERT_TRUE(mem_get_ptr(mem, 0x400D0000u) ==
                mem->flash_insn + 0x10000u);
    ASSERT_TRUE(mem_get_ptr(mem, 0x3F410000u) == NULL);
    ASSERT_EQ(mem_unmapped_count(mem), 0u);
    mem_destroy(mem);
}

TEST(loader_rejects_target_mismatch_before_loading) {
    uint8_t bin[64] = {0};
    bin[0] = 0xE9;
    bin[1] = 1;
    put_le32(&bin[4], 0x40080000u);
    put_le32(&bin[24], 0x3FFB0000u);
    put_le32(&bin[28], 4u);
    put_le32(&bin[32], 0xDEADBEEFu);

    const char *path = write_temp(bin, 36);
    ASSERT_TRUE(path != NULL);
    xtensa_mem_t *mem = mem_create();
    load_result_t res = loader_load_bin_for_target(
        mem, path, FLEXE_TARGET_ESP32S3);
    ASSERT_EQ(res.result, -1);
    ASSERT_TRUE(strstr(res.error, "not requested target ESP32-S3") != NULL);
    ASSERT_EQ(mem_read32(mem, 0x3FFB0000u), 0u);
    mem_destroy(mem);
}

TEST(loader_rejects_unknown_image_chip_id) {
    uint8_t bin[64] = {0};
    bin[0] = 0xE9;
    bin[1] = 1;
    put_le32(&bin[4], 0x40080000u);
    put_le16(&bin[12], 0x1234u);

    const char *path = write_temp(bin, 24);
    ASSERT_TRUE(path != NULL);
    loader_image_info_t info;
    char error[256];
    ASSERT_EQ(loader_probe_bin(path, &info, error, sizeof(error)), -1);
    ASSERT_TRUE(strstr(error, "Unknown ESP image chip ID 0x1234") != NULL);
}

TEST(target_descriptors_are_stable_and_parse_aliases) {
    const flexe_target_desc_t *esp32 =
        flexe_target_by_id(FLEXE_TARGET_ESP32);
    const flexe_target_desc_t *s3 = flexe_target_by_name("esp32-s3");
    ASSERT_TRUE(esp32 != NULL);
    ASSERT_TRUE(s3 != NULL);
    ASSERT_EQ(esp32->descriptor_version, FLEXE_TARGET_DESCRIPTOR_VERSION);
    ASSERT_EQ(esp32->core_generation, FLEXE_XTENSA_LX6);
    ASSERT_EQ(esp32->support_level, FLEXE_TARGET_STABLE);
    ASSERT_TRUE(esp32->capabilities &
                FLEXE_TARGET_CAP_ESP32_CLASSIC_PERIPHERALS);
    ASSERT_TRUE(esp32->capabilities & FLEXE_TARGET_CAP_SPI_MEM);
    ASSERT_TRUE(esp32->capabilities & FLEXE_TARGET_CAP_IO_MUX_V1);
    ASSERT_TRUE(esp32->capabilities & FLEXE_TARGET_CAP_I2C_V1);
    ASSERT_TRUE(esp32->capabilities & FLEXE_TARGET_CAP_RADIO_REGS_V1);
    ASSERT_TRUE(esp32->capabilities & FLEXE_TARGET_CAP_SHA_V1);
    ASSERT_TRUE(esp32->capabilities & FLEXE_TARGET_CAP_GP_SPI);
    ASSERT_TRUE(esp32->capabilities & FLEXE_TARGET_CAP_ROM_FLASH_HANDOFF);
    ASSERT_EQ(esp32->i2c.instance_count, 2u);
    ASSERT_EQ(esp32->i2c.instance[0].base, 0x3FF53000u);
    ASSERT_EQ(esp32->i2c.opcode_restart, 0u);
    ASSERT_EQ(esp32->i2c.command_count, 16u);
    ASSERT_EQ(esp32->sha.base, 0x3FF03000u);
    ASSERT_EQ(esp32->sha.layout, FLEXE_SHA_LAYOUT_ESP32);
    ASSERT_EQ(esp32->sha.mode[1], FLEXE_SHA_ALGORITHM_SHA256);
    ASSERT_EQ(esp32->sha.dma_peripheral_id, UINT8_MAX);
    ASSERT_EQ(s3->image_chip_id, 9u);
    ASSERT_EQ(s3->core_generation, FLEXE_XTENSA_LX7);
    ASSERT_EQ(s3->support_level, FLEXE_TARGET_EXPERIMENTAL);
    ASSERT_TRUE(s3->capabilities & FLEXE_TARGET_CAP_ESP32S3_EXTMEM);
    ASSERT_TRUE(s3->capabilities & FLEXE_TARGET_CAP_DIRECT_ROM_DATA_INIT);
    ASSERT_TRUE(s3->capabilities &
                FLEXE_TARGET_CAP_SECONDARY_CORE_CONTROL);
    ASSERT_TRUE(s3->capabilities & FLEXE_TARGET_CAP_RTC_CALIBRATION);
    ASSERT_TRUE(s3->capabilities & FLEXE_TARGET_CAP_REGI2C);
    ASSERT_TRUE(s3->capabilities & FLEXE_TARGET_CAP_SENSITIVE_MEMPROT_V1);
    ASSERT_TRUE(s3->capabilities & FLEXE_TARGET_CAP_SYSTIMER_V1);
    ASSERT_TRUE(s3->capabilities & FLEXE_TARGET_CAP_TIMER_GROUP_V1);
    ASSERT_TRUE(s3->capabilities & FLEXE_TARGET_CAP_SPI_MEM);
    ASSERT_TRUE(s3->capabilities & FLEXE_TARGET_CAP_INTERRUPT_MATRIX_V1);
    ASSERT_TRUE(s3->capabilities & FLEXE_TARGET_CAP_USB_SERIAL_JTAG_V1);
    ASSERT_TRUE(s3->capabilities & FLEXE_TARGET_CAP_SYSTEM_CLOCK_V1);
    ASSERT_TRUE(s3->capabilities & FLEXE_TARGET_CAP_IO_MUX_V1);
    ASSERT_TRUE(s3->capabilities & FLEXE_TARGET_CAP_RTC_CNTL_V1);
    ASSERT_TRUE(s3->capabilities & FLEXE_TARGET_CAP_EFUSE_READ_V1);
    ASSERT_TRUE(s3->capabilities & FLEXE_TARGET_CAP_GPIO_V1);
    ASSERT_TRUE(s3->capabilities & FLEXE_TARGET_CAP_I2C_V1);
    ASSERT_TRUE(s3->capabilities & FLEXE_TARGET_CAP_SENS_V1);
    ASSERT_TRUE(s3->capabilities & FLEXE_TARGET_CAP_RADIO_REGS_V1);
    ASSERT_TRUE(s3->capabilities & FLEXE_TARGET_CAP_GDMA_V1);
    ASSERT_TRUE(s3->capabilities & FLEXE_TARGET_CAP_SHA_V1);
    ASSERT_TRUE(s3->capabilities & FLEXE_TARGET_CAP_GP_SPI);
    ASSERT_TRUE(s3->capabilities & FLEXE_TARGET_CAP_ROM_FLASH_HANDOFF);
    ASSERT_EQ(s3->i2c.instance_count, 2u);
    ASSERT_EQ(s3->i2c.instance[0].base, 0x60013000u);
    ASSERT_EQ(s3->i2c.instance[1].base, 0x60027000u);
    ASSERT_EQ(s3->i2c.instance[0].interrupt_source, 42u);
    ASSERT_EQ(s3->i2c.instance[1].interrupt_source, 43u);
    ASSERT_EQ(s3->i2c.opcode_restart, 6u);
    ASSERT_EQ(s3->i2c.opcode_read, 3u);
    ASSERT_EQ(s3->i2c.opcode_stop, 2u);
    ASSERT_EQ(s3->i2c.command_count, 8u);
    ASSERT_EQ(s3->gdma.base, 0x6003F000u);
    ASSERT_EQ(s3->gdma.channel_stride, 0xC0u);
    ASSERT_EQ(s3->gdma.channel_count, 5u);
    ASSERT_EQ(s3->gdma.descriptor_address_prefix, 0x3FC00000u);
    ASSERT_EQ(s3->sha.base, 0x6003B000u);
    ASSERT_EQ(s3->sha.layout, FLEXE_SHA_LAYOUT_UNIFIED);
    ASSERT_EQ(s3->sha.mode_count, 8u);
    ASSERT_EQ(s3->sha.mode[2], FLEXE_SHA_ALGORITHM_SHA256);
    ASSERT_EQ(s3->sha.dma_peripheral_id, 7u);
    ASSERT_EQ(s3->default_cpu_frequency_mhz, 160u);
    ASSERT_EQ(s3->cpu_frequency_word, 0x3FCEF758u);
    ASSERT_EQ(s3->secondary_core.base, 0x600C0000u);
    ASSERT_EQ(s3->secondary_core.boot_address_offset, 4u);
    ASSERT_EQ(s3->system_clock.base, 0x600C0000u);
    ASSERT_EQ(s3->system_clock.cpu_per_conf_offset, 0x10u);
    ASSERT_EQ(s3->system_clock.cpu_per_conf_reset, 0x0Cu);
    ASSERT_EQ(s3->system_clock.sysclk_conf_offset, 0x60u);
    ASSERT_EQ(s3->system_clock.sysclk_conf_reset, 1u);
    ASSERT_EQ(s3->system_clock.register_count, 7u);
    ASSERT_EQ(s3->system_clock.gate_count, 8u);
    ASSERT_EQ(s3->system_clock.reg[1].offset, 0x18u);
    ASSERT_EQ(s3->system_clock.reg[1].reset, 0xF9C1E06Fu);
    ASSERT_EQ(s3->system_clock.gate[0].device,
              FLEXE_SYSTEM_DEVICE_SYSTIMER);
    ASSERT_EQ(s3->system_clock.gate[2].instance, 1u);
    ASSERT_EQ(s3->system_clock.gate[3].device,
              FLEXE_SYSTEM_DEVICE_I2C);
    ASSERT_EQ(s3->system_clock.gate[3].clock_mask, 1u << 7);
    ASSERT_EQ(s3->system_clock.gate[4].instance, 1u);
    ASSERT_EQ(s3->system_clock.gate[4].clock_mask, 1u << 18);
    ASSERT_EQ(s3->system_clock.gate[5].device,
              FLEXE_SYSTEM_DEVICE_GP_SPI);
    ASSERT_EQ(s3->system_clock.gate[5].instance, 0u);
    ASSERT_EQ(s3->system_clock.gate[5].clock_mask, 1u << 6);
    ASSERT_EQ(s3->system_clock.gate[6].instance, 1u);
    ASSERT_EQ(s3->system_clock.gate[6].clock_mask, 1u << 16);
    ASSERT_EQ(s3->system_clock.gate[7].device,
              FLEXE_SYSTEM_DEVICE_SHA);
    ASSERT_EQ(s3->system_clock.gate[7].clock_mask, 1u << 2);
    ASSERT_EQ(s3->system_clock.gate[7].reset_mask, 1u << 2);
    ASSERT_EQ(s3->io_mux.base, 0x60009000u);
    ASSERT_EQ(s3->io_mux.gpio_count, 49u);
    ASSERT_EQ(s3->io_mux.gpio_register_offset[26], 0x6Cu);
    ASSERT_EQ(s3->gpio.base, 0x60004000u);
    ASSERT_EQ(s3->gpio.gpio_count, 49u);
    ASSERT_EQ64(s3->gpio.valid_gpio_mask,
                UINT64_C(0x0001FFFFFC3FFFFF));
    ASSERT_EQ(s3->gpio.interrupt_source, 16u);
    ASSERT_EQ(s3->gpio.nmi_interrupt_source, 17u);
    ASSERT_EQ(s3->rtc_cntl.base, 0x60008000u);
    ASSERT_EQ(s3->rtc_cntl.store_offset[4], 0xC0u);
    ASSERT_EQ(s3->rtc_cntl.xtal_frequency_mhz, 40u);
    ASSERT_EQ(s3->rtc_cntl.time_update_offset, 0x0Cu);
    ASSERT_EQ(s3->rtc_cntl.time_high_mask, 0xFFFFu);
    ASSERT_EQ(s3->rtc_cntl.reset_state_reset, 0x3041u);
    ASSERT_EQ(s3->rtc_cntl.clock_conf_offset, 0x74u);
    ASSERT_EQ(s3->rtc_cntl.clock_conf_reset, 0x1158321Cu);
    ASSERT_EQ(s3->rtc_cntl.slow_clock_source_hz[1], 32768u);
    ASSERT_EQ(s3->rtc_cntl.interrupt_enable_offset, 0x40u);
    ASSERT_EQ(s3->rtc_cntl.interrupt_clear_offset, 0x4Cu);
    ASSERT_EQ(s3->rtc_cntl.interrupt_valid_mask, 0x001FFFFFu);
    ASSERT_EQ(s3->rtc_cntl.interrupt_raw_writable_mask, 1u << 20);
    ASSERT_EQ(s3->rtc_cntl.interrupt_source, 39u);
    ASSERT_EQ(s3->rtc_cntl.wdt_config_offset[0], 0x98u);
    ASSERT_EQ(s3->rtc_cntl.wdt_config_offset[4], 0xA8u);
    ASSERT_EQ(s3->rtc_cntl.wdt_config_reset[0], 0x00013214u);
    ASSERT_EQ(s3->rtc_cntl.wdt_config_reset[1], 200000u);
    ASSERT_EQ(s3->rtc_cntl.wdt_write_protect_key, 0x50D83AA1u);
    ASSERT_EQ(s3->rtc_cntl.wdt_interrupt_mask, 1u << 3);
    ASSERT_EQ(s3->rtc_cntl.wdt_stage0_multiplier, 2u);
    ASSERT_EQ(s3->efuse.base, 0x60007000u);
    ASSERT_EQ(s3->efuse.read_data_word_count, 84u);
    ASSERT_EQ(s3->efuse.read_data[6], 0x00000001u);
    ASSERT_EQ(s3->efuse.read_data[7], 0x00000200u);
    ASSERT_EQ(s3->interrupt_matrix.base, 0x600C2000u);
    ASSERT_EQ(s3->interrupt_matrix.source_count, 99u);
    ASSERT_EQ(s3->interrupt_matrix.map_offset[1], 0x800u);
    ASSERT_EQ(s3->interrupt_matrix.status_offset[1], 0x98Cu);
    ASSERT_EQ(s3->interrupt_matrix.software_interrupt_source_base, 79u);
    ASSERT_EQ(s3->rtc_calibration.group_count, 2u);
    ASSERT_EQ(s3->rtc_calibration.base[0], 0x6001F000u);
    ASSERT_EQ(s3->rtc_calibration.base[1], 0x60020000u);
    ASSERT_EQ(s3->rtc_calibration.reference_clock_hz, 40000000u);
    ASSERT_EQ(s3->timer_group.group_count, 2u);
    ASSERT_EQ(s3->timer_group.timer_count, 2u);
    ASSERT_EQ(s3->timer_group.counter_width, 54u);
    ASSERT_EQ(s3->timer_group.base[0], 0x6001F000u);
    ASSERT_EQ(s3->timer_group.base[1], 0x60020000u);
    ASSERT_EQ(s3->timer_group.interrupt_source[0][0], 50u);
    ASSERT_EQ(s3->timer_group.interrupt_source[1][2], 55u);
    ASSERT_EQ(s3->regi2c.base, 0x6000E000u);
    ASSERT_EQ(s3->regi2c.host_count, 2u);
    ASSERT_EQ(s3->regi2c.bbpll_done_mask, 1u << 24);
    ASSERT_EQ(s3->regi2c.aux_register_count, 2u);
    ASSERT_EQ(s3->regi2c.aux_register[0].offset, 0x04Cu);
    ASSERT_EQ(s3->regi2c.aux_register[0].reset, 0x01000000u);
    ASSERT_EQ(s3->regi2c.aux_register[1].offset, 0x050u);
    ASSERT_EQ(s3->regi2c.aux_register[1].reset, 0x07000000u);
    ASSERT_EQ(s3->sensitive_memprot.base, 0x600C1000u);
    ASSERT_EQ(s3->sensitive_memprot.register_size, 0x1000u);
    ASSERT_EQ(s3->systimer.base, 0x60023000u);
    ASSERT_EQ(s3->systimer.counter_frequency_hz, 16000000u);
    ASSERT_EQ(s3->systimer.counter_count, 2u);
    ASSERT_EQ(s3->systimer.alarm_count, 3u);
    ASSERT_EQ(esp32->gp_spi.layout, FLEXE_GP_SPI_LAYOUT_ESP32);
    ASSERT_EQ(esp32->gp_spi.host_count, 2u);
    ASSERT_EQ(esp32->gp_spi.instance[0].base, 0x3FF64000u);
    ASSERT_EQ(esp32->gp_spi.instance[0].clock_out_signal, 8u);
    ASSERT_EQ(esp32->gp_spi.instance[0].chip_select_out_signal[0], 11u);
    ASSERT_EQ(esp32->gp_spi.instance[0].interrupt_source, 30u);
    ASSERT_EQ(esp32->gp_spi.instance[1].base, 0x3FF65000u);
    ASSERT_EQ(esp32->gp_spi.instance[1].clock_out_signal, 63u);
    ASSERT_EQ(esp32->gp_spi.instance[1].interrupt_source, 31u);
    ASSERT_EQ(s3->gp_spi.layout, FLEXE_GP_SPI_LAYOUT_S2_S3);
    ASSERT_EQ(s3->gp_spi.register_size, 0x100u);
    ASSERT_EQ(s3->gp_spi.host_count, 2u);
    ASSERT_EQ(s3->gp_spi.date_reset, 0x02101190u);
    ASSERT_EQ(s3->gp_spi.instance[0].base, 0x60024000u);
    ASSERT_EQ(s3->gp_spi.instance[0].clock_out_signal, 101u);
    ASSERT_EQ(s3->gp_spi.instance[0].chip_select_count, 6u);
    ASSERT_EQ(s3->gp_spi.instance[0].chip_select_out_signal[5], 115u);
    ASSERT_EQ(s3->gp_spi.instance[0].interrupt_source, 21u);
    ASSERT_EQ(s3->gp_spi.instance[0].gdma_peripheral_id, 0u);
    ASSERT_EQ(s3->gp_spi.instance[1].base, 0x60025000u);
    ASSERT_EQ(s3->gp_spi.instance[1].clock_out_signal, 66u);
    ASSERT_EQ(s3->gp_spi.instance[1].chip_select_out_signal[2], 127u);
    ASSERT_EQ(s3->gp_spi.instance[1].interrupt_source, 22u);
    ASSERT_EQ(s3->gp_spi.instance[1].gdma_peripheral_id, 1u);
    ASSERT_EQ(esp32->spi_mem.base[0], 0x3FF43000u);
    ASSERT_EQ(esp32->spi_mem.layout, FLEXE_SPI_MEM_LAYOUT_ESP32);
    ASSERT_EQ(esp32->spi_mem.maximum_flash_size, 0x01000000u);
    ASSERT_EQ(esp32->spi_mem.default_psram_id,
              UINT64_C(0x0000000000205D0D));
    ASSERT_EQ(esp32->spi_mem.flash_chip_select, 0u);
    ASSERT_EQ(esp32->spi_mem.psram_chip_select, 1u);
    ASSERT_EQ(s3->spi_mem.base[0], 0x60003000u);
    ASSERT_EQ(s3->spi_mem.base[1], 0x60002000u);
    ASSERT_EQ(s3->spi_mem.host_count, 2u);
    ASSERT_EQ(s3->spi_mem.layout, FLEXE_SPI_MEM_LAYOUT_S2_S3);
    ASSERT_EQ(s3->spi_mem.default_jedec_id, 0x001640C8u);
    ASSERT_EQ(s3->spi_mem.maximum_flash_size, 0x08000000u);
    ASSERT_EQ(s3->spi_mem.date_reset, 0x02101040u);
    ASSERT_EQ(s3->spi_mem.flash_chip_select, 0u);
    ASSERT_EQ(s3->spi_mem.psram_chip_select, FLEXE_SPI_MEM_CS_NONE);
    ASSERT_EQ(esp32->rom_flash.live_data_address, 0x3FFAE270u);
    ASSERT_TRUE(esp32->rom_flash.pointer_symbol == NULL);
    ASSERT_EQ(s3->rom_flash.live_data_address, 0u);
    ASSERT_TRUE(strcmp(s3->rom_flash.pointer_symbol,
                       "rom_spiflash_legacy_data") == 0);
    ASSERT_EQ(s3->rom_flash.struct_size, 24u);
    ASSERT_EQ(s3->rom_flash.chip_size_offset, 4u);
    ASSERT_EQ(s3->usb_serial_jtag.base, 0x60038000u);
    ASSERT_EQ(s3->usb_serial_jtag.interrupt_source, 96u);
    ASSERT_EQ(s3->usb_serial_jtag.endpoint_size, 64u);
    ASSERT_EQ(s3->usb_serial_jtag.interrupt_raw_reset, 1u << 3);
    ASSERT_EQ(s3->reset_vector, 0x40000400u);
    ASSERT_EQ(s3->vecbase_reset, 0x40000000u);
    ASSERT_EQ(s3->configid0, 0xC2F0FFFEu);
    ASSERT_EQ(s3->configid1, 0x23090F1Fu);
    ASSERT_EQ(s3->interrupt_level[14], 7u);
    ASSERT_EQ(s3->flash_mmu.entry_count, 512u);
    ASSERT_EQ(s3->flash_mmu.table_base[0], 0x600C5000u);
    ASSERT_TRUE(s3->flash_mmu.shared_instruction_data);
    ASSERT_EQ(s3->cache_control_base, 0x600C4000u);
    ASSERT_EQ(s3->cache_control_size, 0x1000u);
    ASSERT_EQ(esp32->uart_count, 3u);
    ASSERT_EQ(esp32->uart[0].base, 0x3FF40000u);
    ASSERT_EQ(esp32->uart[2].interrupt_source, 36u);
    ASSERT_EQ(s3->uart_count, 3u);
    ASSERT_EQ(s3->uart[0].base, 0x60000000u);
    ASSERT_EQ(s3->uart[1].base, 0x60010000u);
    ASSERT_EQ(s3->uart[2].base, 0x6002E000u);
    ASSERT_EQ(s3->uart[0].interrupt_source, 27u);
    ASSERT_EQ(s3->uart[2].interrupt_source, 29u);
    ASSERT_EQ(s3->uart_ip.mem_rx_status_offset, 0x68u);
    ASSERT_EQ(s3->uart_ip.rx_timeout_enable_mask, 1u << 23);
    ASSERT_TRUE(flexe_target_pc_is_executable(s3, 0x40370000u));
    ASSERT_TRUE(flexe_target_pc_is_executable(s3, 0x42000000u));
    ASSERT_TRUE(flexe_target_pc_is_executable(s3, 0x600FE000u));
    ASSERT_FALSE(flexe_target_pc_is_executable(s3, 0x400D0000u));
    flexe_target_id_t parsed = FLEXE_TARGET_AUTO;
    ASSERT_EQ(flexe_target_parse("esp32s3", &parsed), 0);
    ASSERT_EQ(parsed, FLEXE_TARGET_ESP32S3);
}

TEST(loader_multi_segment) {
    uint8_t bin[128];
    memset(bin, 0, sizeof(bin));
    bin[0] = 0xE9;
    bin[1] = 2;            /* 2 segments */
    put_le32(&bin[4], 0x40080000);

    /* Segment 0 */
    put_le32(&bin[24], 0x3FFB0000);
    put_le32(&bin[28], 4);
    put_le32(&bin[32], 0x11111111);

    /* Segment 1 */
    put_le32(&bin[36], 0x3FFB0100);
    put_le32(&bin[40], 4);
    put_le32(&bin[44], 0x22222222);

    const char *path = write_temp(bin, 48);
    ASSERT_TRUE(path != NULL);

    xtensa_mem_t *mem = mem_create();
    load_result_t res = loader_load_bin(mem, path);
    ASSERT_EQ(res.result, 0);
    ASSERT_EQ(res.segment_count, 2);
    ASSERT_EQ(mem_read32(mem, 0x3FFB0000), 0x11111111);
    ASSERT_EQ(mem_read32(mem, 0x3FFB0100), 0x22222222);
    mem_destroy(mem);
}

TEST(loader_nerdminer_reconstructs_huge_app_partitions) {
    uint8_t bin[64] = {0};
    bin[0] = 0xE9;
    bin[1] = 1;
    put_le32(&bin[4], 0x40089268u);
    put_le32(&bin[24], 0x3FFB0000u);
    put_le32(&bin[28], 4);
    put_le32(&bin[32], 0xA5A55A5Au);

    const char *path = write_temp(bin, 36);
    ASSERT_TRUE(path != NULL);
    xtensa_mem_t *mem = mem_create();
    load_result_t res = loader_load_bin(mem, path);
    ASSERT_EQ(res.result, 0);

    const uint32_t table = 0x8000u;
    ASSERT_EQ(mem->flash_data[table + 2u], 1);
    ASSERT_EQ(mem->flash_data[table + 3u], 2);
    ASSERT_EQ(get_le32(mem->flash_data + table + 4u), 0x9000u);
    ASSERT_EQ(get_le32(mem->flash_data + table + 8u), 0x5000u);
    const uint32_t spiffs = table + 3u * 32u;
    ASSERT_EQ(mem->flash_data[spiffs + 2u], 1);
    ASSERT_EQ(mem->flash_data[spiffs + 3u], 0x82);
    ASSERT_EQ(get_le32(mem->flash_data + spiffs + 4u), 0x310000u);
    ASSERT_EQ(get_le32(mem->flash_data + spiffs + 8u), 0x0E0000u);
    ASSERT_EQ(memcmp(mem->flash_data + spiffs + 12u, "spiffs", 6), 0);
    ASSERT_EQ(mem->flash_data[0x310000u], 0xFF);
    mem_destroy(mem);
}

TEST(loader_replaces_temporary_flash_maps_with_boot_mmu) {
    uint8_t bin[64] = {0};
    bin[0] = 0xE9;
    bin[1] = 1;
    put_le32(&bin[4], 0x40080000u);
    put_le32(&bin[24], 0x3FFB0000u);
    put_le32(&bin[28], 4u);
    put_le32(&bin[32], 0x12345678u);

    const char *path = write_temp(bin, 36);
    ASSERT_TRUE(path != NULL);
    xtensa_mem_t *mem = mem_create();
    esp32_periph_t *periph = periph_create(mem);
    load_result_t res = loader_load_bin(mem, path);
    ASSERT_EQ(res.result, 0);

    /* Only the header page remains live for this image. All linear mappings
     * used while parsing must be gone before application startup. */
    ASSERT_TRUE(mem_get_ptr(mem, 0x3F400000u) == mem->flash_data + 0x10000u);
    ASSERT_TRUE(mem_get_ptr(mem, 0x3F410000u) == NULL);
    ASSERT_TRUE(mem_get_ptr(mem, 0x400C2000u) == NULL);
    ASSERT_TRUE(mem_get_ptr(mem, 0x400D0000u) == NULL);
    ASSERT_EQ(mem_read32(mem, 0x3FF10000u), 1u);
    ASSERT_EQ(mem_read32(mem, 0x3FF12000u), 1u);
    ASSERT_EQ(mem_read32(mem, 0x3FF10004u), 0x100u);
    ASSERT_EQ(mem_read32(mem, 0x3FF10000u + 77u * 4u), 0x100u);

    periph_destroy(periph);
    mem_destroy(mem);
}

TEST(loader_bad_magic) {
    uint8_t bin[32];
    memset(bin, 0, sizeof(bin));
    bin[0] = 0x00; /* wrong magic */
    bin[1] = 1;
    const char *path = write_temp(bin, 32);
    ASSERT_TRUE(path != NULL);

    xtensa_mem_t *mem = mem_create();
    load_result_t res = loader_load_bin(mem, path);
    ASSERT_EQ(res.result, -1);
    mem_destroy(mem);
}

TEST(loader_null_path) {
    xtensa_mem_t *mem = mem_create();
    load_result_t res = loader_load_bin(mem, NULL);
    ASSERT_EQ(res.result, -1);
    mem_destroy(mem);
}

TEST(loader_null_mem) {
    load_result_t res = loader_load_bin(NULL, "/tmp/whatever");
    ASSERT_EQ(res.result, -1);
}

void run_loader_tests(void) {
    TEST_SUITE("ESP32 .bin Loader");

    RUN_TEST(loader_single_segment);
    RUN_TEST(loader_reports_image_revision_metadata);
    RUN_TEST(loader_rejects_reserved_flash_capacity);
    RUN_TEST(loader_rejects_app_larger_than_declared_flash);
    RUN_TEST(loader_recognizes_s3_before_rejecting_classic_memory);
    RUN_TEST(loader_loads_s3_segments_through_shared_flash_mmu);
    RUN_TEST(loader_maps_classic_flash_without_peripheral_model);
    RUN_TEST(loader_rejects_target_mismatch_before_loading);
    RUN_TEST(loader_rejects_unknown_image_chip_id);
    RUN_TEST(target_descriptors_are_stable_and_parse_aliases);
    RUN_TEST(loader_multi_segment);
    RUN_TEST(loader_nerdminer_reconstructs_huge_app_partitions);
    RUN_TEST(loader_replaces_temporary_flash_maps_with_boot_mmu);
    RUN_TEST(loader_bad_magic);
    RUN_TEST(loader_null_path);
    RUN_TEST(loader_null_mem);
}
