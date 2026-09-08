#ifndef SX127X_H
#define SX127X_H

#include "peripherals.h"

#include <stddef.h>
#include <stdint.h>

typedef struct sx127x sx127x_t;

typedef void (*sx127x_tx_fn)(void *ctx, const uint8_t *data, size_t len);

typedef struct {
    esp32_periph_t *periph;
    int dio0_pin;
} sx127x_config_t;

typedef struct {
    uint64_t spi_transfers;
    uint64_t spi_frames;
    uint64_t register_reads;
    uint64_t register_writes;
    uint64_t version_reads;
    uint64_t tx_packets;
    uint64_t rx_packets;
    size_t last_tx_len;
} sx127x_stats_t;

sx127x_t *sx127x_create(const sx127x_config_t *config);
void sx127x_destroy(sx127x_t *radio);
void sx127x_reset(sx127x_t *radio);

/* Signature-compatible with periph_spi_device_fn. */
void sx127x_spi_transfer(void *ctx, int host,
                         const uint8_t *mosi, size_t mosi_len,
                         uint8_t *miso, size_t miso_len);
/* Signature-compatible with periph_spi_select_fn. */
void sx127x_spi_select(void *ctx, int selected);

void sx127x_set_tx_callback(sx127x_t *radio, sx127x_tx_fn fn, void *ctx);

/* Deliver a valid LoRa packet to a radio in RX single/continuous mode.
 * snr_quarter_db uses the SX127x register's signed quarter-dB units. */
int sx127x_inject_packet(sx127x_t *radio, const uint8_t *data, size_t len,
                         int8_t rssi_dbm, int8_t snr_quarter_db);

void sx127x_get_stats(const sx127x_t *radio, sx127x_stats_t *out);
uint8_t sx127x_register(const sx127x_t *radio, uint8_t address);

#endif /* SX127X_H */
