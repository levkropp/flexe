#ifndef AXP192_H
#define AXP192_H

#include "peripherals.h"

#include <stdbool.h>
#include <stdint.h>

typedef struct axp192 axp192_t;

typedef struct {
    bool battery_present;
    bool vbus_present;
    uint16_t battery_mv;
    uint16_t vbus_mv;
} axp192_config_t;

typedef struct {
    uint64_t transfers;
    uint64_t register_reads;
    uint64_t register_writes;
    uint64_t chip_id_reads;
    uint64_t irq_clears;
    int last_port;
} axp192_stats_t;

axp192_t *axp192_create(const axp192_config_t *config);
void axp192_destroy(axp192_t *pmu);
void axp192_reset(axp192_t *pmu);

/* Signature-compatible with periph_i2c_device_fn. */
int axp192_i2c_transfer(void *ctx, int port, uint8_t address,
                        const uint8_t *write_data, size_t write_len,
                        uint8_t *read_data, size_t read_len);

void axp192_set_battery(axp192_t *pmu, bool present, uint16_t millivolts);
void axp192_set_vbus(axp192_t *pmu, bool present, uint16_t millivolts);
void axp192_raise_irq(axp192_t *pmu, uint64_t flags);
void axp192_get_stats(const axp192_t *pmu, axp192_stats_t *out);
uint8_t axp192_register(const axp192_t *pmu, uint8_t address);

#endif /* AXP192_H */
