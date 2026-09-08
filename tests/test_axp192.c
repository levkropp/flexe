#include "axp192.h"

#include <string.h>

static uint8_t axp192_test_read(axp192_t *pmu, uint8_t address)
{
    uint8_t value = 0;
    ASSERT_EQ(axp192_i2c_transfer(pmu, 0, 0x34u, &address, 1,
                                  &value, 1), 0);
    return value;
}

static void axp192_test_write(axp192_t *pmu, uint8_t address, uint8_t value)
{
    uint8_t bytes[2] = {address, value};
    ASSERT_EQ(axp192_i2c_transfer(pmu, 0, 0x34u, bytes, sizeof(bytes),
                                  NULL, 0), 0);
}

TEST(axp192_register_protocol_and_power_outputs)
{
    axp192_config_t config = {
        .battery_present = true,
        .vbus_present = true,
        .battery_mv = 3960,
        .vbus_mv = 5000,
    };
    axp192_t *pmu = axp192_create(&config);
    ASSERT_TRUE(pmu != NULL);

    ASSERT_EQ(axp192_test_read(pmu, 0x03u), 0x03u);
    ASSERT_TRUE(axp192_test_read(pmu, 0x00u) & (1u << 5));
    ASSERT_TRUE(axp192_test_read(pmu, 0x01u) & (1u << 5));
    ASSERT_TRUE(axp192_test_read(pmu, 0x90u) != 0);

    axp192_test_write(pmu, 0x28u, 0xF5u);
    axp192_test_write(pmu, 0x12u, 0x07u);
    ASSERT_EQ(axp192_test_read(pmu, 0x28u), 0xF5u);
    ASSERT_EQ(axp192_test_read(pmu, 0x12u), 0x07u);

    uint8_t start = 0x78u;
    uint8_t battery[2] = {0};
    ASSERT_EQ(axp192_i2c_transfer(pmu, 0, 0x34u, &start, 1,
                                  battery, sizeof(battery)), 0);
    unsigned raw = ((unsigned)battery[0] << 4) | (battery[1] & 0x0Fu);
    ASSERT_EQ(raw * 11u / 10u, 3960u);

    axp192_stats_t stats;
    axp192_get_stats(pmu, &stats);
    ASSERT_EQ64(stats.chip_id_reads, 1u);
    ASSERT_TRUE(stats.register_reads >= 8u);
    ASSERT_EQ64(stats.register_writes, 2u);
    ASSERT_EQ(stats.last_port, 0);

    axp192_destroy(pmu);
}

TEST(axp192_input_changes_and_irq_write_one_to_clear)
{
    axp192_t *pmu = axp192_create(NULL);
    ASSERT_TRUE(pmu != NULL);

    axp192_set_battery(pmu, false, 0);
    axp192_set_vbus(pmu, false, 0);
    ASSERT_EQ(axp192_test_read(pmu, 0x00u) & (1u << 5), 0u);
    ASSERT_EQ(axp192_test_read(pmu, 0x01u) & (1u << 5), 0u);

    /* Status and identity are read-only. */
    axp192_test_write(pmu, 0x03u, 0x4Au);
    ASSERT_EQ(axp192_test_read(pmu, 0x03u), 0x03u);

    axp192_raise_irq(pmu, 0x00000000000000A5ull);
    ASSERT_EQ(axp192_test_read(pmu, 0x44u), 0xA5u);
    axp192_test_write(pmu, 0x44u, 0x24u);
    ASSERT_EQ(axp192_test_read(pmu, 0x44u), 0x81u);
    axp192_stats_t stats;
    axp192_get_stats(pmu, &stats);
    ASSERT_EQ64(stats.irq_clears, 1u);

    uint8_t dummy = 0;
    ASSERT_EQ(axp192_i2c_transfer(pmu, 0, 0x35u, &dummy, 1,
                                  NULL, 0), -1);
    axp192_destroy(pmu);
}

static void run_axp192_tests(void)
{
    TEST_SUITE("AXP192 PMU");
    RUN_TEST(axp192_register_protocol_and_power_outputs);
    RUN_TEST(axp192_input_changes_and_irq_write_one_to_clear);
}
