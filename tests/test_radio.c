/* Target-described Wi-Fi/Bluetooth controller register tests. */
#include "test_helpers.h"
#include "peripherals.h"
#include "radio.h"

TEST(esp32s3_radio_windows_retain_independent_configuration)
{
    const flexe_target_desc_t *s3 =
        flexe_target_by_id(FLEXE_TARGET_ESP32S3);
    xtensa_mem_t *mem = mem_create_for_target(s3);
    esp32_periph_t *periph = periph_create(mem);
    ASSERT_TRUE(mem != NULL);
    ASSERT_TRUE(periph != NULL);
    if (!mem || !periph) {
        periph_destroy(periph);
        mem_destroy(mem);
        return;
    }

    static const struct {
        uint32_t address;
        uint32_t value;
    } registers[] = {
        { 0x600050F0u, 0x00000600u }, /* FE2 */
        { 0x60006090u, 0x00000030u }, /* FE */
        { 0x60011020u, 0x10203040u }, /* BT */
        { 0x6001C018u, 0x50607080u }, /* private NRX */
        { 0x6001CC48u, 0x88776655u }, /* documented NRX */
        { 0x6001D040u, 0x90A0B0C0u }, /* BB */
        { 0x6003120Cu, 0xA5A5A5A4u }, /* private BT */
        { 0x60032124u, 0x10200200u }, /* BT MAC */
        { 0x600340B8u, 0xCAFEBABEu }, /* second Wi-Fi MAC page */
        { 0x60035020u, 0x98000000u }, /* WDEV */
    };

    for (unsigned i = 0u;
         i < sizeof(registers) / sizeof(registers[0]); i++)
        mem_write32(mem, registers[i].address, registers[i].value);
    for (unsigned i = 0u;
         i < sizeof(registers) / sizeof(registers[0]); i++)
        ASSERT_EQ(mem_read32(mem, registers[i].address), registers[i].value);

    ASSERT_EQ(periph_unhandled_count(periph), 0u);
    periph_destroy(periph);
    mem_destroy(mem);
}

TEST(esp32s3_rom_iq_estimation_completes_from_control_protocol)
{
    const flexe_target_desc_t *s3 =
        flexe_target_by_id(FLEXE_TARGET_ESP32S3);
    xtensa_mem_t *mem = mem_create_for_target(s3);
    esp32_periph_t *periph = periph_create(mem);
    ASSERT_TRUE(mem != NULL);
    ASSERT_TRUE(periph != NULL);
    if (!mem || !periph) {
        periph_destroy(periph);
        mem_destroy(mem);
        return;
    }

    const uint32_t control = 0x60006144u;
    const uint32_t status = 0x60006174u;
    const uint32_t done = 1u << 16;

    ASSERT_EQ(mem_read32(mem, status) & done, 0u);
    /* This is the exact ordering in ESP32-S3 rev-0 ROM
     * rom_iq_est_enable: enable, delay, then start. */
    mem_write32(mem, control, 1u << 0);
    ASSERT_EQ(mem_read32(mem, status) & done, 0u);
    mem_write32(mem, control, (1u << 1) | (1u << 0) | (7u << 2));
    ASSERT_EQ(mem_read32(mem, control), 0x1Fu);
    ASSERT_EQ(mem_read32(mem, status) & done, done);

    /* Completion status is produced by hardware, not guest-writable. */
    mem_write32(mem, status, 0u);
    ASSERT_EQ(mem_read32(mem, status) & done, done);

    /* rom_iq_est_disable clears start before clearing enable. */
    mem_write32(mem, control, 1u << 0);
    ASSERT_EQ(mem_read32(mem, status) & done, 0u);
    ASSERT_EQ(mem_read32(mem, 0x6000615Cu), 0u);
    ASSERT_EQ(mem_read32(mem, 0x60006160u), 0u);
    ASSERT_EQ(mem_read32(mem, 0x60006164u), 0u);
    ASSERT_EQ(periph_unhandled_count(periph), 0u);

    periph_destroy(periph);
    mem_destroy(mem);
}

TEST(esp32s3_wifi_mac_reset_reports_ready)
{
    const flexe_target_desc_t *s3 =
        flexe_target_by_id(FLEXE_TARGET_ESP32S3);
    xtensa_mem_t *mem = mem_create_for_target(s3);
    esp32_periph_t *periph = periph_create(mem);
    ASSERT_TRUE(mem != NULL);
    ASSERT_TRUE(periph != NULL);
    if (!mem || !periph) {
        periph_destroy(periph);
        mem_destroy(mem);
        return;
    }

    const uint32_t mac_init_control = 0x60033D14u;
    ASSERT_EQ(mem_read32(mem, mac_init_control), 0u);
    mem_write32(mem, mac_init_control, 1u << 1);
    ASSERT_EQ(mem_read32(mem, mac_init_control) & 3u, 3u);
    mem_write32(mem, mac_init_control, 0u);
    ASSERT_EQ(mem_read32(mem, mac_init_control) & 3u, 0u);
    ASSERT_EQ(periph_unhandled_count(periph), 0u);

    periph_destroy(periph);
    mem_destroy(mem);
}

TEST(esp32s3_wdev_random_source_is_deterministic_and_live)
{
    const flexe_target_desc_t *s3 =
        flexe_target_by_id(FLEXE_TARGET_ESP32S3);
    xtensa_mem_t *mem_a = mem_create_for_target(s3);
    xtensa_mem_t *mem_b = mem_create_for_target(s3);
    esp32_periph_t *periph_a = periph_create(mem_a);
    esp32_periph_t *periph_b = periph_create(mem_b);
    ASSERT_TRUE(periph_a != NULL);
    ASSERT_TRUE(periph_b != NULL);
    if (!periph_a || !periph_b) {
        periph_destroy(periph_a);
        periph_destroy(periph_b);
        mem_destroy(mem_a);
        mem_destroy(mem_b);
        return;
    }

    const uint32_t random_address = 0x6003507Cu;
    uint32_t first_a = mem_read32(mem_a, random_address);
    uint32_t second_a = mem_read32(mem_a, random_address);
    uint32_t first_b = mem_read32(mem_b, random_address);
    ASSERT_TRUE(first_a != second_a);
    ASSERT_EQ(first_a, first_b);
    mem_write32(mem_a, random_address, 0u);
    ASSERT_TRUE(mem_read32(mem_a, random_address) != 0u);
    ASSERT_EQ(periph_unhandled_count(periph_a), 0u);

    periph_destroy(periph_a);
    periph_destroy(periph_b);
    mem_destroy(mem_a);
    mem_destroy(mem_b);
}

TEST(radio_rejects_absent_capability_and_overlapping_windows)
{
    const flexe_target_desc_t *s3 =
        flexe_target_by_id(FLEXE_TARGET_ESP32S3);
    flexe_target_desc_t invalid = *s3;
    invalid.capabilities &= ~FLEXE_TARGET_CAP_RADIO_REGS_V1;
    xtensa_mem_t *mem = mem_create_for_target(&invalid);
    ASSERT_TRUE(mem != NULL);
    flexe_radio_t *radio = flexe_radio_create(mem, NULL, NULL, NULL);
    ASSERT_TRUE(radio == NULL);
    mem_destroy(mem);

    invalid = *s3;
    invalid.radio.window[1].base = invalid.radio.window[0].base;
    mem = mem_create_for_target(&invalid);
    ASSERT_TRUE(mem != NULL);
    radio = flexe_radio_create(mem, NULL, NULL, NULL);
    ASSERT_TRUE(radio == NULL);
    mem_destroy(mem);
}

static void run_radio_tests(void)
{
    TEST_SUITE("Target-described radio registers");
    RUN_TEST(esp32s3_radio_windows_retain_independent_configuration);
    RUN_TEST(esp32s3_rom_iq_estimation_completes_from_control_protocol);
    RUN_TEST(esp32s3_wifi_mac_reset_reports_ready);
    RUN_TEST(esp32s3_wdev_random_source_is_deterministic_and_live);
    RUN_TEST(radio_rejects_absent_capability_and_overlapping_windows);
}
