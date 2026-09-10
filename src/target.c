#include "target.h"

#include <stddef.h>
#include <string.h>

/* Image chip IDs follow Espressif's public esp_chip_id_t values. The mapped
 * flash windows match esptool's target definitions. Keeping recognized but
 * unavailable targets here lets the loader fail explicitly instead of
 * feeding a valid image to the wrong SoC model. */
static const flexe_target_desc_t TARGETS[] = {
    {
        .descriptor_version = 1,
        .id = FLEXE_TARGET_ESP32,
        .name = "esp32",
        .display_name = "ESP32",
        .image_chip_id = 0x0000,
        .core_generation = FLEXE_XTENSA_LX6,
        .core_count = 2,
        .support_level = FLEXE_TARGET_STABLE,
        .drom_start = 0x3F400000u,
        .drom_end = 0x3F800000u,
        .irom_start = 0x400D0000u,
        .irom_end = 0x40400000u,
    },
    {
        .descriptor_version = 1,
        .id = FLEXE_TARGET_ESP32S3,
        .name = "esp32s3",
        .display_name = "ESP32-S3",
        .image_chip_id = 0x0009,
        .core_generation = FLEXE_XTENSA_LX7,
        .core_count = 2,
        .support_level = FLEXE_TARGET_UNAVAILABLE,
        .drom_start = 0x3C000000u,
        .drom_end = 0x3E000000u,
        .irom_start = 0x42000000u,
        .irom_end = 0x44000000u,
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
