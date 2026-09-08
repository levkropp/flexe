#ifndef UBLOX_GPS_H
#define UBLOX_GPS_H

#include "peripherals.h"

#include <stddef.h>
#include <stdint.h>

typedef struct ublox_gps ublox_gps_t;

typedef struct {
    esp32_periph_t *periph;
    int uart_num;
} ublox_gps_config_t;

typedef struct {
    uint64_t tx_bytes;
    uint64_t rx_bytes;
    uint64_t ubx_commands;
    uint64_t ack_responses;
    uint64_t mon_ver_requests;
    uint64_t nmea_sentences;
    uint64_t queue_overflows;
    size_t pending_bytes;
} ublox_gps_stats_t;

ublox_gps_t *ublox_gps_create(const ublox_gps_config_t *config);
void ublox_gps_destroy(ublox_gps_t *gps);

/* Signature-compatible with uart_tx_cb. Parses commands written by firmware
 * and queues the corresponding receiver response on the configured UART. */
void ublox_gps_uart_tx(void *ctx, uint8_t byte);
void ublox_gps_poll(ublox_gps_t *gps);

/* Queue a deterministic valid fix from a virtual NEO-M8 receiver. */
int ublox_gps_inject_fix(ublox_gps_t *gps);

void ublox_gps_get_stats(const ublox_gps_t *gps, ublox_gps_stats_t *out);

#endif /* UBLOX_GPS_H */
