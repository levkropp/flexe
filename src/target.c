#include "target.h"

#include <stddef.h>
#include <string.h>

/* Image chip IDs follow Espressif's public esp_chip_id_t values. The mapped
 * flash windows match esptool's target definitions. Keeping recognized but
 * unavailable targets here lets the loader fail explicitly instead of
 * feeding a valid image to the wrong SoC model. */
static const flexe_target_desc_t TARGETS[] = {
    {
        .descriptor_version = 2,
        .id = FLEXE_TARGET_ESP32,
        .name = "esp32",
        .display_name = "ESP32",
        .image_chip_id = 0x0000,
        .core_generation = FLEXE_XTENSA_LX6,
        .core_count = 2,
        .support_level = FLEXE_TARGET_STABLE,
        .reset_vector = 0x40000400u,
        .vecbase_reset = 0x40000000u,
        .configid0 = 0xC2BCFFFEu,
        .configid1 = 0x1CC5FE96u,
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
        .executable_range_count = 3,
        .executable = {
            { 0x40000000u, 0x400C2000u }, /* ROM, cache, IRAM, RTC fast */
            { 0x400D0000u, 0x40C00000u }, /* flash MMU instruction buses */
            { 0x50000000u, 0x50002000u }, /* RTC slow/reset vector 0 */
        },
    },
    {
        .descriptor_version = 2,
        .id = FLEXE_TARGET_ESP32S3,
        .name = "esp32s3",
        .display_name = "ESP32-S3",
        .image_chip_id = 0x0009,
        .core_generation = FLEXE_XTENSA_LX7,
        .core_count = 2,
        .support_level = FLEXE_TARGET_UNAVAILABLE,
        .reset_vector = 0x40000400u,
        .vecbase_reset = 0x40000000u,
        .configid0 = 0xC2F0FFFEu,
        .configid1 = 0x23090F1Fu,
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
