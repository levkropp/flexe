#include "ublox_gps.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define UBLOX_QUEUE_CAPACITY 1024u
#define UBLOX_PACKET_CAPACITY 768u

struct ublox_gps {
    esp32_periph_t *periph;
    int uart_num;
    uint8_t ubx[UBLOX_PACKET_CAPACITY];
    size_t ubx_len;
    size_t ubx_expected;
    uint8_t queue[UBLOX_QUEUE_CAPACITY];
    size_t queue_head;
    size_t queue_count;
    ublox_gps_stats_t stats;
};

static void ublox_checksum(const uint8_t *data, size_t len,
                           uint8_t *a_out, uint8_t *b_out)
{
    uint8_t a = 0;
    uint8_t b = 0;
    for (size_t i = 0; i < len; i++) {
        a = (uint8_t)(a + data[i]);
        b = (uint8_t)(b + a);
    }
    *a_out = a;
    *b_out = b;
}

static int ublox_queue_bytes(ublox_gps_t *gps, const uint8_t *data,
                             size_t len)
{
    if (len > UBLOX_QUEUE_CAPACITY - gps->queue_count) {
        gps->stats.queue_overflows++;
        return -1;
    }
    for (size_t i = 0; i < len; i++) {
        size_t tail = (gps->queue_head + gps->queue_count) %
                      UBLOX_QUEUE_CAPACITY;
        gps->queue[tail] = data[i];
        gps->queue_count++;
    }
    return 0;
}

void ublox_gps_poll(ublox_gps_t *gps)
{
    if (!gps || gps->queue_count == 0) return;
    size_t contiguous = UBLOX_QUEUE_CAPACITY - gps->queue_head;
    if (contiguous > gps->queue_count) contiguous = gps->queue_count;
    size_t accepted = periph_uart_rx_inject_num(
            gps->periph, gps->uart_num, gps->queue + gps->queue_head,
            contiguous);
    gps->queue_head = (gps->queue_head + accepted) % UBLOX_QUEUE_CAPACITY;
    gps->queue_count -= accepted;
    gps->stats.rx_bytes += accepted;
}

static int ublox_queue_packet(ublox_gps_t *gps, uint8_t msg_class,
                              uint8_t msg_id, const uint8_t *payload,
                              size_t payload_len)
{
    if (payload_len + 8u > UBLOX_PACKET_CAPACITY) return -1;
    uint8_t packet[UBLOX_PACKET_CAPACITY];
    packet[0] = 0xB5u;
    packet[1] = 0x62u;
    packet[2] = msg_class;
    packet[3] = msg_id;
    packet[4] = (uint8_t)payload_len;
    packet[5] = (uint8_t)(payload_len >> 8);
    if (payload_len != 0) memcpy(packet + 6, payload, payload_len);
    ublox_checksum(packet + 2, payload_len + 4u,
                   &packet[payload_len + 6u], &packet[payload_len + 7u]);
    int result = ublox_queue_bytes(gps, packet, payload_len + 8u);
    ublox_gps_poll(gps);
    return result;
}

static int ublox_queue_ack(ublox_gps_t *gps, uint8_t msg_class,
                            uint8_t msg_id)
{
    uint8_t payload[2] = {msg_class, msg_id};
    int result = ublox_queue_packet(gps, 0x05u, 0x01u,
                                    payload, sizeof(payload));
    if (result == 0) gps->stats.ack_responses++;
    return result;
}

static int ublox_queue_mon_ver(ublox_gps_t *gps)
{
    uint8_t payload[100];
    memset(payload, 0, sizeof(payload));
    memcpy(payload, "ROM CORE 3.01", 13);
    memcpy(payload + 30, "00080000", 8);
    memcpy(payload + 40, "PROTVER=18.00", 13);
    memcpy(payload + 70, "MOD=NEO-M8N-0", 13);
    return ublox_queue_packet(gps, 0x0Au, 0x04u,
                              payload, sizeof(payload));
}

static void ublox_handle_packet(ublox_gps_t *gps)
{
    size_t payload_len = (size_t)gps->ubx[4] |
                         ((size_t)gps->ubx[5] << 8);
    uint8_t a;
    uint8_t b;
    ublox_checksum(gps->ubx + 2, payload_len + 4u, &a, &b);
    if (a != gps->ubx[payload_len + 6u] ||
        b != gps->ubx[payload_len + 7u])
        return;

    uint8_t msg_class = gps->ubx[2];
    uint8_t msg_id = gps->ubx[3];
    gps->stats.ubx_commands++;
    if (msg_class == 0x0Au && msg_id == 0x04u && payload_len == 0) {
        gps->stats.mon_ver_requests++;
        (void)ublox_queue_mon_ver(gps);
    } else {
        (void)ublox_queue_ack(gps, msg_class, msg_id);
    }
}

static void ublox_consume_tx_byte(ublox_gps_t *gps, uint8_t byte)
{
    if (gps->ubx_len == 0) {
        if (byte == 0xB5u) {
            gps->ubx[0] = byte;
            gps->ubx_len = 1;
            gps->ubx_expected = 0;
        }
        return;
    }

    if (gps->ubx_len == 1 && byte != 0x62u) {
        gps->ubx_len = 0;
        if (byte == 0xB5u) {
            gps->ubx[0] = byte;
            gps->ubx_len = 1;
        }
        return;
    }
    if (gps->ubx_len >= sizeof(gps->ubx)) {
        gps->ubx_len = 0;
        gps->ubx_expected = 0;
        return;
    }

    gps->ubx[gps->ubx_len++] = byte;
    if (gps->ubx_len == 6) {
        size_t payload_len = (size_t)gps->ubx[4] |
                             ((size_t)gps->ubx[5] << 8);
        gps->ubx_expected = payload_len + 8u;
        if (gps->ubx_expected > sizeof(gps->ubx)) {
            gps->ubx_len = 0;
            gps->ubx_expected = 0;
            return;
        }
    }
    if (gps->ubx_expected != 0 && gps->ubx_len == gps->ubx_expected) {
        ublox_handle_packet(gps);
        gps->ubx_len = 0;
        gps->ubx_expected = 0;
    }
}

void ublox_gps_uart_tx(void *ctx, uint8_t byte)
{
    ublox_gps_t *gps = ctx;
    if (!gps) return;
    gps->stats.tx_bytes++;
    ublox_consume_tx_byte(gps, byte);
}

ublox_gps_t *ublox_gps_create(const ublox_gps_config_t *config)
{
    if (!config || !config->periph || config->uart_num < 0 ||
        config->uart_num > 2)
        return NULL;
    ublox_gps_t *gps = calloc(1, sizeof(*gps));
    if (!gps) return NULL;
    gps->periph = config->periph;
    gps->uart_num = config->uart_num;
    return gps;
}

void ublox_gps_destroy(ublox_gps_t *gps)
{
    free(gps);
}

static int ublox_queue_nmea(ublox_gps_t *gps, const char *body)
{
    uint8_t checksum = 0;
    for (const char *p = body; *p; p++) checksum ^= (uint8_t)*p;
    char sentence[192];
    int len = snprintf(sentence, sizeof(sentence), "$%s*%02X\r\n",
                       body, checksum);
    if (len < 0 || (size_t)len >= sizeof(sentence) ||
        ublox_queue_bytes(gps, (const uint8_t *)sentence, (size_t)len) != 0)
        return -1;
    gps->stats.nmea_sentences++;
    return 0;
}

int ublox_gps_inject_fix(ublox_gps_t *gps)
{
    if (!gps) return -1;
    if (ublox_queue_nmea(
                gps,
                "GPRMC,123519.00,A,4807.5000,N,01131.0000,E,0.0,0.0,"
                "010137,,,A") != 0 ||
        ublox_queue_nmea(
                gps,
                "GPGGA,123519.00,4807.5000,N,01131.0000,E,1,08,0.9,"
                "545.4,M,46.9,M,,") != 0 ||
        ublox_queue_nmea(
                gps,
                "GNGSA,A,3,04,05,09,12,24,25,29,31,,,,,1.8,0.9,1.5,1")
                != 0)
        return -1;
    ublox_gps_poll(gps);
    return 0;
}

void ublox_gps_get_stats(const ublox_gps_t *gps, ublox_gps_stats_t *out)
{
    if (!out) return;
    if (gps) {
        *out = gps->stats;
        out->pending_bytes = gps->queue_count;
    } else {
        memset(out, 0, sizeof(*out));
    }
}
