#include "ublox_gps.h"

#include <string.h>

static size_t ublox_test_packet(uint8_t *out, uint8_t msg_class,
                                uint8_t msg_id)
{
    out[0] = 0xB5u;
    out[1] = 0x62u;
    out[2] = msg_class;
    out[3] = msg_id;
    out[4] = 0;
    out[5] = 0;
    uint8_t a = 0;
    uint8_t b = 0;
    for (size_t i = 2; i < 6; i++) {
        a = (uint8_t)(a + out[i]);
        b = (uint8_t)(b + a);
    }
    out[6] = a;
    out[7] = b;
    return 8;
}

static size_t ublox_test_drain(xtensa_mem_t *mem, esp32_periph_t *periph,
                               ublox_gps_t *gps, uint8_t *out, size_t cap)
{
    size_t len = 0;
    while (len < cap) {
        while (periph_uart_rx_pending_num(periph, 1) != 0 && len < cap)
            out[len++] = (uint8_t)mem_read32(mem, 0x3FF50000u);
        ublox_gps_poll(gps);
        ublox_gps_stats_t stats;
        ublox_gps_get_stats(gps, &stats);
        if (periph_uart_rx_pending_num(periph, 1) == 0 &&
            stats.pending_bytes == 0)
            break;
    }
    return len;
}

TEST(ublox_gps_answers_probe_and_mon_ver)
{
    xtensa_mem_t *mem = mem_create();
    esp32_periph_t *periph = periph_create(mem);
    ublox_gps_config_t config = {.periph = periph, .uart_num = 1};
    ublox_gps_t *gps = ublox_gps_create(&config);
    ASSERT_TRUE(gps != NULL);

    uint8_t command[8];
    size_t command_len = ublox_test_packet(command, 0x06u, 0x08u);
    for (size_t i = 0; i < command_len; i++)
        ublox_gps_uart_tx(gps, command[i]);
    uint8_t response[256] = {0};
    size_t response_len = ublox_test_drain(mem, periph, gps, response,
                                            sizeof(response));
    ASSERT_EQ(response_len, 10u);
    static const uint8_t ack_prefix[] = {
        0xB5u, 0x62u, 0x05u, 0x01u, 0x02u, 0x00u, 0x06u, 0x08u,
    };
    ASSERT_TRUE(memcmp(response, ack_prefix, sizeof(ack_prefix)) == 0);

    command_len = ublox_test_packet(command, 0x0Au, 0x04u);
    for (size_t i = 0; i < command_len; i++)
        ublox_gps_uart_tx(gps, command[i]);
    response_len = ublox_test_drain(mem, periph, gps, response,
                                     sizeof(response));
    ASSERT_EQ(response_len, 108u);
    ASSERT_EQ(response[2], 0x0Au);
    ASSERT_EQ(response[3], 0x04u);
    ASSERT_TRUE(memcmp(response + 36u, "00080000", 8) == 0);
    ASSERT_TRUE(memcmp(response + 46u, "PROTVER=18.00", 13) == 0);

    ublox_gps_stats_t stats;
    ublox_gps_get_stats(gps, &stats);
    ASSERT_EQ64(stats.ubx_commands, 2u);
    ASSERT_EQ64(stats.ack_responses, 1u);
    ASSERT_EQ64(stats.mon_ver_requests, 1u);
    ASSERT_EQ64(stats.queue_overflows, 0u);

    ublox_gps_destroy(gps);
    periph_destroy(periph);
    mem_destroy(mem);
}

TEST(ublox_gps_streams_valid_nmea_fix)
{
    xtensa_mem_t *mem = mem_create();
    esp32_periph_t *periph = periph_create(mem);
    ublox_gps_config_t config = {.periph = periph, .uart_num = 1};
    ublox_gps_t *gps = ublox_gps_create(&config);
    ASSERT_TRUE(gps != NULL);
    ASSERT_EQ(ublox_gps_inject_fix(gps), 0);

    uint8_t response[512] = {0};
    size_t response_len = ublox_test_drain(mem, periph, gps, response,
                                            sizeof(response) - 1u);
    response[response_len] = '\0';
    ASSERT_TRUE(strstr((char *)response, "$GPRMC,") != NULL);
    ASSERT_TRUE(strstr((char *)response, ",010137,,,A*") != NULL);
    ASSERT_TRUE(strstr((char *)response, "$GPGGA,") != NULL);
    ASSERT_TRUE(strstr((char *)response, "$GNGSA,") != NULL);
    ublox_gps_stats_t stats;
    ublox_gps_get_stats(gps, &stats);
    ASSERT_EQ64(stats.nmea_sentences, 3u);
    ASSERT_EQ(stats.pending_bytes, 0u);
    ASSERT_EQ64(stats.rx_bytes, response_len);

    ublox_gps_destroy(gps);
    periph_destroy(periph);
    mem_destroy(mem);
}

static void run_ublox_gps_tests(void)
{
    TEST_SUITE("u-blox GPS");
    RUN_TEST(ublox_gps_answers_probe_and_mon_ver);
    RUN_TEST(ublox_gps_streams_valid_nmea_fix);
}
