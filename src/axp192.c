#include "axp192.h"

#include <stdlib.h>
#include <string.h>

#define AXP192_ADDRESS             0x34u
#define AXP192_REG_STATUS          0x00u
#define AXP192_REG_CHARGE_STATUS   0x01u
#define AXP192_REG_CHIP_ID         0x03u
#define AXP192_REG_OUTPUT_CONTROL  0x12u
#define AXP192_REG_BATTERY_H       0x78u
#define AXP192_REG_BATTERY_L       0x79u
#define AXP192_REG_VBUS_H          0x5Au
#define AXP192_REG_VBUS_L          0x5Bu
#define AXP192_REG_SYSTEM_H        0x7Eu
#define AXP192_REG_SYSTEM_L        0x7Fu
#define AXP192_REG_TEMPERATURE_H   0x5Eu
#define AXP192_REG_TEMPERATURE_L   0x5Fu

#define AXP192_CHIP_ID             0x03u
#define AXP192_STATUS_VBUS_GOOD    (1u << 5)
#define AXP192_STATUS_VBUS_ABOVE   (1u << 2)
#define AXP192_STATUS_BATTERY      (1u << 5)

struct axp192 {
    uint8_t reg[256];
    uint8_t cursor;
    bool battery_present;
    bool vbus_present;
    uint16_t battery_mv;
    uint16_t vbus_mv;
    axp192_stats_t stats;
};

static bool axp192_irq_status_register(uint8_t address)
{
    return (address >= 0x44u && address <= 0x47u) || address == 0x4Du;
}

static bool axp192_read_only_register(uint8_t address)
{
    if (address <= AXP192_REG_CHIP_ID) return true;
    if (address >= 0x56u && address <= 0x7Fu) return true;
    return false;
}

static void axp192_encode_adc12(uint8_t reg[256], uint8_t high_address,
                                unsigned raw)
{
    if (raw > 0xFFFu) raw = 0xFFFu;
    reg[high_address] = (uint8_t)(raw >> 4);
    reg[(uint8_t)(high_address + 1u)] = (uint8_t)(raw & 0x0Fu);
}

static void axp192_update_inputs(axp192_t *pmu)
{
    pmu->reg[AXP192_REG_STATUS] &=
        (uint8_t)~(AXP192_STATUS_VBUS_GOOD | AXP192_STATUS_VBUS_ABOVE);
    if (pmu->vbus_present)
        pmu->reg[AXP192_REG_STATUS] |=
            AXP192_STATUS_VBUS_GOOD | AXP192_STATUS_VBUS_ABOVE;

    pmu->reg[AXP192_REG_CHARGE_STATUS] &=
        (uint8_t)~AXP192_STATUS_BATTERY;
    if (pmu->battery_present)
        pmu->reg[AXP192_REG_CHARGE_STATUS] |= AXP192_STATUS_BATTERY;

    /* AXP192 voltage ADCs use 12-bit high-eight/low-four encodings. */
    axp192_encode_adc12(pmu->reg, AXP192_REG_BATTERY_H,
                        (unsigned)pmu->battery_mv * 10u / 11u);
    axp192_encode_adc12(pmu->reg, AXP192_REG_VBUS_H,
                        (unsigned)pmu->vbus_mv * 10u / 17u);
    axp192_encode_adc12(pmu->reg, AXP192_REG_SYSTEM_H, 3300u * 10u / 14u);
    /* 25 C, with the documented 0.1 C/LSB and -144.7 C offset. */
    axp192_encode_adc12(pmu->reg, AXP192_REG_TEMPERATURE_H, 1697u);
}

void axp192_reset(axp192_t *pmu)
{
    if (!pmu) return;
    memset(pmu->reg, 0, sizeof(pmu->reg));
    pmu->cursor = 0;

    pmu->reg[AXP192_REG_CHIP_ID] = AXP192_CHIP_ID;
    pmu->reg[AXP192_REG_OUTPUT_CONTROL] = 0x03u; /* DCDC1/3 on */
    pmu->reg[0x23u] = 0x16u;
    pmu->reg[0x26u] = 0x68u; /* DCDC1 = 3.3 V */
    pmu->reg[0x27u] = 0x68u; /* DCDC3 = 3.3 V */
    pmu->reg[0x28u] = 0xFFu; /* LDO2/3 = 3.3 V */
    pmu->reg[0x31u] = 0x03u;
    pmu->reg[0x33u] = 0xC0u;
    pmu->reg[0x36u] = 0x0Cu;
    pmu->reg[0x82u] = 0xFFu;
    pmu->reg[0x83u] = 0x80u;
    pmu->reg[0x84u] = 0x32u;
    /* Meshtastic uses this nonzero register to distinguish AXP192/2101 from
     * a TCA8418 keyboard at the same I2C address. */
    pmu->reg[0x90u] = 0x07u;
    axp192_update_inputs(pmu);
}

axp192_t *axp192_create(const axp192_config_t *config)
{
    axp192_t *pmu = calloc(1, sizeof(*pmu));
    if (!pmu) return NULL;
    pmu->battery_present = config ? config->battery_present : true;
    pmu->vbus_present = config ? config->vbus_present : true;
    pmu->battery_mv = config ? config->battery_mv : 3970u;
    pmu->vbus_mv = config ? config->vbus_mv : 5000u;
    pmu->stats.last_port = -1;
    axp192_reset(pmu);
    return pmu;
}

void axp192_destroy(axp192_t *pmu)
{
    free(pmu);
}

static uint8_t axp192_read_register(axp192_t *pmu, uint8_t address)
{
    pmu->stats.register_reads++;
    if (address == AXP192_REG_CHIP_ID) pmu->stats.chip_id_reads++;
    return pmu->reg[address];
}

static void axp192_write_register(axp192_t *pmu, uint8_t address,
                                  uint8_t value)
{
    pmu->stats.register_writes++;
    if (axp192_irq_status_register(address)) {
        pmu->reg[address] &= (uint8_t)~value;
        pmu->stats.irq_clears++;
    } else if (!axp192_read_only_register(address)) {
        pmu->reg[address] = value;
    }
}

int axp192_i2c_transfer(void *ctx, int port, uint8_t address,
                        const uint8_t *write_data, size_t write_len,
                        uint8_t *read_data, size_t read_len)
{
    axp192_t *pmu = ctx;
    if (!pmu || address != AXP192_ADDRESS ||
        (!write_data && write_len != 0) || (!read_data && read_len != 0))
        return -1;

    pmu->stats.transfers++;
    pmu->stats.last_port = port;
    if (write_len != 0) {
        pmu->cursor = write_data[0];
        for (size_t i = 1; i < write_len; i++)
            axp192_write_register(pmu, pmu->cursor++, write_data[i]);
    }
    for (size_t i = 0; i < read_len; i++)
        read_data[i] = axp192_read_register(pmu, pmu->cursor++);
    return 0;
}

void axp192_set_battery(axp192_t *pmu, bool present, uint16_t millivolts)
{
    if (!pmu) return;
    pmu->battery_present = present;
    pmu->battery_mv = millivolts;
    axp192_update_inputs(pmu);
}

void axp192_set_vbus(axp192_t *pmu, bool present, uint16_t millivolts)
{
    if (!pmu) return;
    pmu->vbus_present = present;
    pmu->vbus_mv = millivolts;
    axp192_update_inputs(pmu);
}

void axp192_raise_irq(axp192_t *pmu, uint64_t flags)
{
    if (!pmu) return;
    for (unsigned byte = 0; byte < 4; byte++)
        pmu->reg[0x44u + byte] |= (uint8_t)(flags >> (byte * 8u));
    pmu->reg[0x4Du] |= (uint8_t)(flags >> 32u);
}

void axp192_get_stats(const axp192_t *pmu, axp192_stats_t *out)
{
    if (!out) return;
    if (pmu) *out = pmu->stats;
    else memset(out, 0, sizeof(*out));
}

uint8_t axp192_register(const axp192_t *pmu, uint8_t address)
{
    return pmu ? pmu->reg[address] : 0;
}
