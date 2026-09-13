/* Target-described always-on RTC controller tests. */
#include "test_helpers.h"
#include "peripherals.h"
#include "regi2c.h"
#include "rtc_cntl.h"

typedef struct {
    unsigned reads;
    unsigned writes;
    uint32_t last_write_addr;
    uint32_t last_write_value;
} rtc_cntl_fallback_t;

typedef struct {
    unsigned changes;
    bool level;
} rtc_cntl_irq_probe_t;

static uint32_t rtc_cntl_test_fallback_read(void *ctx, uint32_t addr)
{
    rtc_cntl_fallback_t *fallback = ctx;
    fallback->reads++;
    return addr ^ 0x5A5A5A5Au;
}

static void rtc_cntl_test_fallback_write(void *ctx, uint32_t addr,
                                         uint32_t value)
{
    rtc_cntl_fallback_t *fallback = ctx;
    fallback->writes++;
    fallback->last_write_addr = addr;
    fallback->last_write_value = value;
}

static void rtc_cntl_test_irq_changed(void *ctx, bool level)
{
    rtc_cntl_irq_probe_t *probe = ctx;
    probe->changes++;
    probe->level = level;
}

TEST(rtc_cntl_storage_resets_persists_and_delegates)
{
    const flexe_target_desc_t *s3 =
        flexe_target_by_id(FLEXE_TARGET_ESP32S3);
    const flexe_rtc_cntl_desc_t *desc = &s3->rtc_cntl;
    xtensa_mem_t *mem = mem_create_for_target(s3);
    rtc_cntl_fallback_t fallback = {0};
    flexe_rtc_cntl_t *rtc = flexe_rtc_cntl_create(
        mem, rtc_cntl_test_fallback_read,
        rtc_cntl_test_fallback_write, &fallback,
        NULL, NULL, NULL, NULL, NULL, NULL);
    ASSERT_TRUE(mem != NULL);
    ASSERT_TRUE(rtc != NULL);
    if (!mem || !rtc) {
        flexe_rtc_cntl_destroy(rtc);
        mem_destroy(mem);
        return;
    }

    uint32_t store0 = desc->base + desc->store_offset[0];
    uint32_t store4 = desc->base + desc->store_offset[4];
    ASSERT_EQ(mem_read32(mem, store0), 0u);
    ASSERT_EQ(mem_read32(mem, store4), 0u);
    mem_write32(mem, store0, 0x01234567u);
    mem_write32(mem, store4, 0x89ABCDEFu);
    ASSERT_EQ(mem_read32(mem, store0), 0x01234567u);
    ASSERT_EQ(mem_read32(mem, store4), 0x89ABCDEFu);

    uint32_t reserved = desc->base + 0x1F8u;
    ASSERT_EQ(mem_read32(mem, reserved), reserved ^ 0x5A5A5A5Au);
    mem_write32(mem, reserved, 1u);
    ASSERT_EQ(fallback.reads, 1u);
    ASSERT_EQ(fallback.writes, 1u);

    flexe_rtc_cntl_destroy(rtc);
    mem_destroy(mem);
}

TEST(rtc_cntl_sar_i2c_power_gates_analog_slave)
{
    const flexe_target_desc_t *s3 =
        flexe_target_by_id(FLEXE_TARGET_ESP32S3);
    const flexe_rtc_cntl_desc_t *rtc_desc = &s3->rtc_cntl;
    const flexe_regi2c_desc_t *bus = &s3->regi2c;
    xtensa_mem_t *mem = mem_create_for_target(s3);
    rtc_cntl_fallback_t fallback = {0};
    flexe_rtc_cntl_t *rtc = flexe_rtc_cntl_create(
        mem, rtc_cntl_test_fallback_read,
        rtc_cntl_test_fallback_write, &fallback,
        NULL, NULL, NULL, NULL, NULL, NULL);
    flexe_regi2c_t *regi2c = flexe_regi2c_create(
        mem, rtc_cntl_test_fallback_read,
        rtc_cntl_test_fallback_write, &fallback);
    ASSERT_TRUE(mem != NULL);
    ASSERT_TRUE(rtc != NULL);
    ASSERT_TRUE(regi2c != NULL);
    if (!mem || !rtc || !regi2c) {
        flexe_regi2c_destroy(regi2c);
        flexe_rtc_cntl_destroy(rtc);
        mem_destroy(mem);
        return;
    }
    flexe_regi2c_attach_rtc_cntl(regi2c, rtc);

    uint32_t analog = rtc_desc->base + rtc_desc->analog_conf_offset;
    uint32_t host = bus->base + bus->command_offset + bus->command_stride;
    uint32_t write = bus->command_start_mask | bus->command_write_mask |
        s3->sens.adc_calibration_slave |
        ((uint32_t)s3->sens.adc_calibration_address <<
         bus->address_shift) |
        (0x35u << bus->data_shift);
    uint8_t data = 0u;

    ASSERT_EQ(mem_read32(mem, analog), rtc_desc->analog_conf_reset);
    ASSERT_TRUE(flexe_rtc_cntl_sar_i2c_powered(rtc));
    mem_write32(mem, host, write);
    ASSERT_TRUE(flexe_regi2c_register_read(
        regi2c, s3->sens.adc_calibration_slave,
        s3->sens.adc_calibration_address, &data));
    ASSERT_EQ(data, 0x35u);
    ASSERT_EQ(fallback.writes, 0u);

    /* An RTC power-down makes the analog register inaccessible and prevents
     * new SAR commands from completing, without erasing other slaves. */
    uint32_t off = rtc_desc->analog_conf_reset &
                   ~rtc_desc->sar_i2c_power_mask;
    mem_write32(mem, analog, off);
    ASSERT_TRUE(!flexe_rtc_cntl_sar_i2c_powered(rtc));
    ASSERT_EQ(mem_read32(mem, analog), off);
    ASSERT_TRUE(!flexe_regi2c_register_read(
        regi2c, s3->sens.adc_calibration_slave,
        s3->sens.adc_calibration_address, &data));
    mem_write32(mem, host,
                (write & ~bus->data_mask) | (0xA6u << bus->data_shift));
    ASSERT_EQ(mem_read32(mem, host) & bus->command_busy_mask,
              bus->command_busy_mask);
    ASSERT_EQ(fallback.writes, 1u);
    ASSERT_EQ(fallback.last_write_addr, host);
    uint32_t other = bus->command_start_mask | bus->command_write_mask |
                     0x66u | (0x5Au << bus->data_shift);
    mem_write32(mem, host, other);
    ASSERT_EQ(mem_read32(mem, host) & bus->command_busy_mask, 0u);
    ASSERT_EQ(fallback.writes, 1u);

    mem_write32(mem, analog,
                off | rtc_desc->sar_i2c_power_mask);
    ASSERT_TRUE(flexe_rtc_cntl_sar_i2c_powered(rtc));
    ASSERT_TRUE(flexe_regi2c_register_read(
        regi2c, s3->sens.adc_calibration_slave,
        s3->sens.adc_calibration_address, &data));
    ASSERT_EQ(data, 0x35u);
    mem_write32(mem, host,
                (write & ~bus->data_mask) | (0xA6u << bus->data_shift));
    ASSERT_TRUE(flexe_regi2c_register_read(
        regi2c, s3->sens.adc_calibration_slave,
        s3->sens.adc_calibration_address, &data));
    ASSERT_EQ(data, 0xA6u);

    /* Register retention is not a claim that RF/PLL controls are modeled. */
    mem_write32(mem, analog, rtc_desc->analog_conf_reset | (1u << 31));
    ASSERT_EQ(mem_read32(mem, analog),
              rtc_desc->analog_conf_reset | (1u << 31));
    ASSERT_EQ(fallback.writes, 2u);
    ASSERT_EQ(fallback.last_write_addr, analog);
    mem_write32(mem, analog, rtc_desc->analog_conf_reset | (1u << 31) | 1u);
    ASSERT_EQ(mem_read32(mem, analog),
              rtc_desc->analog_conf_reset | (1u << 31));
    ASSERT_EQ(fallback.writes, 3u);

    flexe_regi2c_destroy(regi2c);
    flexe_rtc_cntl_destroy(rtc);
    mem_destroy(mem);
}

TEST(rtc_cntl_application_handoff_uses_target_clocks)
{
    const flexe_target_desc_t *s3 =
        flexe_target_by_id(FLEXE_TARGET_ESP32S3);
    const flexe_rtc_cntl_desc_t *desc = &s3->rtc_cntl;
    xtensa_mem_t *mem = mem_create_for_target(s3);
    flexe_rtc_cntl_t *rtc =
        flexe_rtc_cntl_create(mem, NULL, NULL, NULL,
                              NULL, NULL, NULL, NULL, NULL, NULL);
    ASSERT_TRUE(mem != NULL);
    ASSERT_TRUE(rtc != NULL);
    if (!mem || !rtc) {
        flexe_rtc_cntl_destroy(rtc);
        mem_destroy(mem);
        return;
    }

    flexe_rtc_cntl_application_handoff(rtc);
    uint32_t cal_addr = desc->base +
                        desc->store_offset[desc->slow_clock_cal_store];
    uint32_t xtal_addr = desc->base +
                         desc->store_offset[desc->xtal_frequency_store];
    ASSERT_EQ(mem_read32(mem, cal_addr),
              (uint32_t)((UINT64_C(1000000) << 19) /
                         desc->slow_clock_hz));
    ASSERT_EQ(mem_read32(mem, xtal_addr), 0x00280028u);
    uint32_t wdt_config0 = desc->base + desc->wdt_config_offset[0];
    ASSERT_EQ(mem_read32(mem, wdt_config0) &
              desc->wdt_flashboot_enable_mask, 0u);
    ASSERT_EQ(mem_read32(mem, wdt_config0) &
              desc->wdt_enable_mask,
              desc->wdt_config_reset[0] & desc->wdt_enable_mask);

    flexe_rtc_cntl_destroy(rtc);
    mem_destroy(mem);
}

TEST(rtc_cntl_watchdog_reports_unmodeled_configuration)
{
    const flexe_target_desc_t *s3 =
        flexe_target_by_id(FLEXE_TARGET_ESP32S3);
    const flexe_rtc_cntl_desc_t *desc = &s3->rtc_cntl;
    xtensa_mem_t *mem = mem_create_for_target(s3);
    rtc_cntl_fallback_t fallback = {0};
    flexe_rtc_cntl_t *rtc = flexe_rtc_cntl_create(
        mem, rtc_cntl_test_fallback_read,
        rtc_cntl_test_fallback_write, &fallback,
        NULL, NULL, NULL, NULL, NULL, NULL);
    ASSERT_TRUE(mem != NULL);
    ASSERT_TRUE(rtc != NULL);
    if (!mem || !rtc) {
        flexe_rtc_cntl_destroy(rtc);
        mem_destroy(mem);
        return;
    }

    uint32_t config0 = desc->base + desc->wdt_config_offset[0];
    uint32_t protect = desc->base + desc->wdt_write_protect_offset;
    uint32_t value = desc->wdt_config_reset[0] |
        (5u << desc->wdt_stage_action_shift[0]);
    mem_write32(mem, config0, value);
    ASSERT_EQ(mem_read32(mem, config0), value);
    ASSERT_EQ(fallback.writes, 1u);
    ASSERT_EQ(fallback.last_write_addr, config0);
    ASSERT_EQ(fallback.last_write_value, value);

    /* Locked writes are physically inert and therefore must neither mutate
     * the modeled register nor report a behavior that never took effect. */
    mem_write32(mem, protect, 0u);
    mem_write32(mem, config0, desc->wdt_config_reset[0]);
    ASSERT_EQ(mem_read32(mem, config0), value);
    ASSERT_EQ(fallback.writes, 1u);

    /* Changing a defined but behaviorally unmodeled field is also explicit,
     * while moving back to supported stage actions is handled normally. */
    mem_write32(mem, protect, desc->wdt_write_protect_key);
    mem_write32(mem, config0, desc->wdt_config_reset[0]);
    ASSERT_EQ(fallback.writes, 1u);
    mem_write32(mem, config0, desc->wdt_config_reset[0] ^ 1u);
    ASSERT_EQ(fallback.writes, 2u);
    ASSERT_EQ(fallback.last_write_addr, config0);

    /* Feed is a one-bit write-only command; unexpected command bits remain
     * visible to the common unsupported-access path. */
    uint32_t feed = desc->base + desc->wdt_feed_offset;
    mem_write32(mem, feed, desc->wdt_feed_mask | 1u);
    ASSERT_EQ(fallback.writes, 3u);
    ASSERT_EQ(fallback.last_write_addr, feed);

    flexe_rtc_cntl_destroy(rtc);
    mem_destroy(mem);
}

static uint64_t rtc_capture(xtensa_mem_t *mem,
                            const flexe_rtc_cntl_desc_t *desc)
{
    mem_write32(mem, desc->base + desc->time_update_offset,
                desc->time_update_mask);
    uint64_t low = mem_read32(mem, desc->base + desc->time_low_offset);
    uint64_t high = mem_read32(mem, desc->base + desc->time_high_offset);
    return low | (high << 32u);
}

TEST(rtc_cntl_counter_tracks_shared_time_and_frequency)
{
    const flexe_target_desc_t *s3 =
        flexe_target_by_id(FLEXE_TARGET_ESP32S3);
    const flexe_rtc_cntl_desc_t *desc = &s3->rtc_cntl;
    xtensa_mem_t *mem = mem_create_for_target(s3);
    esp32_periph_t *periph = periph_create(mem);
    xtensa_cpu_t cpu[2];
    ASSERT_TRUE(mem != NULL);
    ASSERT_TRUE(periph != NULL);
    if (!mem || !periph) {
        periph_destroy(periph);
        mem_destroy(mem);
        return;
    }

    xtensa_cpu_init_for_target(&cpu[0], s3);
    xtensa_cpu_init_for_target(&cpu[1], s3);
    cpu[0].mem = mem;
    cpu[1].mem = mem;
    cpu[1].core_id = 1;
    periph_attach_cpus(periph, &cpu[0], &cpu[1]);

    ASSERT_EQ(mem_read32(mem, desc->base + desc->reset_state_offset),
              0x00003041u);
    ASSERT_EQ(mem_read32(mem, desc->base + desc->time_update_offset), 0u);
    ASSERT_EQ64(rtc_capture(mem, desc), 0u);

    /* One millisecond at 160 MHz is 136 ticks of the target-described
     * nominal S3 RC slow clock. The second core cannot double-count it. */
    cpu[0].ccount = 160000u;
    ASSERT_EQ64(rtc_capture(mem, desc), 136u);
    cpu[1].ccount = 80000u;
    ASSERT_EQ64(rtc_capture(mem, desc), 136u);
    cpu[1].ccount = 320000u;
    ASSERT_EQ64(rtc_capture(mem, desc), 272u);

    /* Scaling changes at an observed frequency boundary without disturbing
     * the accumulated always-on count. */
    mem_write32(mem, s3->cpu_frequency_word, 80u);
    cpu[0].ccount = 400000u;
    ASSERT_EQ64(rtc_capture(mem, desc), 408u);
    ASSERT_EQ(periph_unhandled_count(periph), 0);

    periph_destroy(periph);
    mem_destroy(mem);
}

TEST(rtc_cntl_switches_slow_clock_at_an_exact_boundary)
{
    const flexe_target_desc_t *s3 =
        flexe_target_by_id(FLEXE_TARGET_ESP32S3);
    const flexe_rtc_cntl_desc_t *desc = &s3->rtc_cntl;
    xtensa_mem_t *mem = mem_create_for_target(s3);
    esp32_periph_t *periph = periph_create(mem);
    xtensa_cpu_t cpu;
    ASSERT_TRUE(mem != NULL);
    ASSERT_TRUE(periph != NULL);
    if (!mem || !periph) {
        periph_destroy(periph);
        mem_destroy(mem);
        return;
    }

    xtensa_cpu_init_for_target(&cpu, s3);
    cpu.mem = mem;
    periph_attach_cpus(periph, &cpu, NULL);

    uint32_t clock_addr = desc->base + desc->clock_conf_offset;
    ASSERT_EQ(mem_read32(mem, clock_addr), 0x1158321Cu);
    cpu.ccount = 160000u;
    ASSERT_EQ64(rtc_capture(mem, desc), 136u);

    uint32_t xtal32k = (mem_read32(mem, clock_addr) &
                        ~desc->slow_clock_select_mask) |
                       (1u << desc->slow_clock_select_shift);
    mem_write32(mem, clock_addr, xtal32k);
    cpu.ccount = 320000u;
    ASSERT_EQ64(rtc_capture(mem, desc), 168u);
    cpu.ccount = 480000u;
    ASSERT_EQ64(rtc_capture(mem, desc), 201u);
    ASSERT_EQ(periph_unhandled_count(periph), 0);

    /* Fast-clock selection is retained for correct register readback, but
     * its unmodeled electrical effect remains an explicit diagnostic. */
    mem_write32(mem, clock_addr, xtal32k | (1u << 29u));
    ASSERT_TRUE(mem_read32(mem, clock_addr) & (1u << 29u));
    ASSERT_EQ(periph_unhandled_count(periph), 1);

    /* The fourth mux value is reserved. Preserve what firmware wrote, use
     * the deterministic target fallback rate, and never claim support. */
    uint32_t reserved =
        (mem_read32(mem, clock_addr) & ~desc->slow_clock_select_mask) |
        desc->slow_clock_select_mask;
    mem_write32(mem, clock_addr, reserved);
    ASSERT_EQ(mem_read32(mem, clock_addr), reserved);
    ASSERT_EQ(periph_unhandled_count(periph), 2);

    periph_destroy(periph);
    mem_destroy(mem);
}

TEST(rtc_cntl_unmodeled_power_registers_remain_unsupported)
{
    const flexe_target_desc_t *s3 =
        flexe_target_by_id(FLEXE_TARGET_ESP32S3);
    const flexe_rtc_cntl_desc_t *desc = &s3->rtc_cntl;
    xtensa_mem_t *mem = mem_create_for_target(s3);
    esp32_periph_t *periph = periph_create(mem);
    ASSERT_TRUE(mem != NULL);
    ASSERT_TRUE(periph != NULL);
    if (!mem || !periph) {
        periph_destroy(periph);
        mem_destroy(mem);
        return;
    }

    int before = periph_unhandled_count(periph);
    ASSERT_EQ(mem_read32(mem, desc->base + 0x7Cu), 0u);
    mem_write32(mem, desc->base + 0x7Cu, 1u);
    ASSERT_EQ(periph_unhandled_count(periph), before + 2);

    /* Unsupported timestamp-control bits share TIME_UPDATE with the modeled
     * latch command and must still produce an explicit diagnostic. */
    mem_write32(mem, desc->base + desc->time_update_offset,
                desc->time_update_mask | (1u << 29u));
    ASSERT_EQ(periph_unhandled_count(periph), before + 3);

    periph_destroy(periph);
    mem_destroy(mem);
}

TEST(rtc_cntl_digital_pad_hold_freezes_physical_gpio_not_latches)
{
    const flexe_target_desc_t *s3 =
        flexe_target_by_id(FLEXE_TARGET_ESP32S3);
    const flexe_rtc_cntl_desc_t *rtc = &s3->rtc_cntl;
    const flexe_gpio_desc_t *gpio = &s3->gpio;
    xtensa_mem_t *mem = mem_create_for_target(s3);
    esp32_periph_t *periph = mem ? periph_create(mem) : NULL;
    ASSERT_TRUE(periph != NULL);
    if (!periph) {
        mem_destroy(mem);
        return;
    }

    uint32_t hold = rtc->base + rtc->digital_pad_hold_offset;
    ASSERT_EQ(mem_read32(mem, hold), 0u);
    ASSERT_EQ(rtc->digital_pad_hold_first_gpio, 21u);
    ASSERT_EQ(rtc->digital_pad_hold_first_bit, 1u);
    /* GPIO21 is the first held pad, GPIO31 the last in the low bank. */
    mem_write32(mem, gpio->base + 0x008u, (1u << 21u) | (1u << 31u));
    mem_write32(mem, gpio->base + 0x024u, (1u << 21u) | (1u << 31u));
    mem_write32(mem, hold, (1u << 1u) | (1u << 11u));
    ASSERT_EQ(mem_read32(mem, hold), (1u << 1u) | (1u << 11u));

    mem_write32(mem, gpio->base + 0x00Cu, (1u << 21u) | (1u << 31u));
    mem_write32(mem, gpio->base + 0x028u, (1u << 21u) | (1u << 31u));
    ASSERT_EQ(mem_read32(mem, gpio->base + 0x004u), 0u);
    ASSERT_EQ(mem_read32(mem, gpio->base + 0x020u), 0u);
    ASSERT_EQ(periph_gpio_pin_level(periph, 21), 1);
    ASSERT_EQ(periph_gpio_output_enabled(periph, 31), 1);

    mem_write32(mem, hold, 1u << 11u);
    ASSERT_EQ(periph_gpio_pin_level(periph, 21), 0);
    ASSERT_EQ(periph_gpio_output_enabled(periph, 21), 0);
    ASSERT_EQ(periph_gpio_pin_level(periph, 31), 1);
    mem_write32(mem, hold, 0u);
    ASSERT_EQ(periph_gpio_pin_level(periph, 31), 0);
    ASSERT_EQ(periph_gpio_output_enabled(periph, 31), 0);

    /* GPIO47 uses the upper bank; GPIO48 has no RTC digital hold bit. */
    mem_write32(mem, gpio->base + 0x014u, 1u << 15u);
    mem_write32(mem, gpio->base + 0x030u, 1u << 15u);
    mem_write32(mem, hold, 1u << 27u);
    mem_write32(mem, gpio->base + 0x018u, 1u << 15u);
    ASSERT_EQ(periph_gpio_pin_level(periph, 47), 1);
    mem_write32(mem, hold, 0u);
    ASSERT_EQ(periph_gpio_pin_level(periph, 47), 0);

    int before = periph_unhandled_count(periph);
    /* Bit 2 maps to unbonded GPIO22; bit 28 is not an S3 hold bit. */
    mem_write32(mem, hold, (1u << 2u) | (1u << 28u));
    ASSERT_EQ(mem_read32(mem, hold), 0u);
    ASSERT_EQ(periph_unhandled_count(periph), before + 1);
    ASSERT_EQ(mem_unmapped_count(mem), 0u);
    periph_destroy(periph);
    mem_destroy(mem);
}

TEST(rtc_cntl_digital_pad_hold_survives_rebuild_without_unheld_gpio)
{
    const flexe_target_desc_t *s3 =
        flexe_target_by_id(FLEXE_TARGET_ESP32S3);
    const flexe_rtc_cntl_desc_t *rtc = &s3->rtc_cntl;
    const flexe_gpio_desc_t *gpio = &s3->gpio;
    xtensa_mem_t *mem = mem_create_for_target(s3);
    esp32_periph_t *periph = mem ? periph_create(mem) : NULL;
    ASSERT_TRUE(periph != NULL);
    if (!periph) {
        mem_destroy(mem);
        return;
    }

    uint32_t hold = rtc->base + rtc->digital_pad_hold_offset;
    mem_write32(mem, gpio->base + 0x008u,
                (1u << 21u) | (1u << 26u));
    mem_write32(mem, gpio->base + 0x024u,
                (1u << 21u) | (1u << 26u));
    mem_write32(mem, hold, 1u << 1u);
    periph_pad_hold_t snapshot;
    periph_pad_hold_snapshot(periph, &snapshot);
    ASSERT_EQ64(snapshot.target_gpio.mask, UINT64_C(1) << 21u);
    ASSERT_EQ(snapshot.target_rtc_hold, 1u << 1u);

    periph_destroy(periph);
    periph = periph_create(mem);
    ASSERT_TRUE(periph != NULL);
    if (periph) {
        ASSERT_EQ(mem_read32(mem, hold), 0u);
        periph_pad_hold_restore(periph, &snapshot);
        ASSERT_EQ(mem_read32(mem, hold), 1u << 1u);
        ASSERT_EQ(mem_read32(mem, gpio->base + 0x004u), 0u);
        ASSERT_EQ(periph_gpio_pin_level(periph, 21), 1);
        ASSERT_EQ(periph_gpio_output_enabled(periph, 21), 1);
        ASSERT_EQ(periph_gpio_pin_level(periph, 26), 0);
        ASSERT_EQ(periph_gpio_output_enabled(periph, 26), 0);
        mem_write32(mem, hold, 0u);
        ASSERT_EQ(periph_gpio_pin_level(periph, 21), 0);
        ASSERT_EQ(periph_gpio_output_enabled(periph, 21), 0);
        ASSERT_EQ(periph_unhandled_count(periph), 0);
    }
    periph_destroy(periph);
    mem_destroy(mem);
}

TEST(rtc_cntl_rtc_pad_hold_freezes_mux_input_and_output)
{
    const flexe_target_desc_t *s3 =
        flexe_target_by_id(FLEXE_TARGET_ESP32S3);
    const flexe_rtc_cntl_desc_t *rtc = &s3->rtc_cntl;
    const flexe_rtc_io_desc_t *io = &s3->rtc_io;
    xtensa_mem_t *mem = mem_create_for_target(s3);
    esp32_periph_t *periph = mem ? periph_create(mem) : NULL;
    ASSERT_TRUE(periph != NULL);
    if (!periph) {
        mem_destroy(mem);
        return;
    }

    uint32_t hold = rtc->base + rtc->rtc_pad_hold_offset;
    uint32_t digital_hold = rtc->base + rtc->digital_pad_hold_offset;
    uint32_t pad4 = io->base + io->pad_base_offset + 4u * 4u;
    uint32_t pad11 = io->base + io->pad_base_offset + 4u * 11u;
    uint32_t rtc4 = 1u << (io->data_shift + 4u);
    uint32_t rtc11 = 1u << (io->data_shift + 11u);
    uint32_t gpio = s3->gpio.base;
    ASSERT_EQ(rtc->rtc_pad_hold_first_gpio, 0u);
    ASSERT_EQ(rtc->rtc_pad_hold_first_bit, 0u);
    ASSERT_EQ(rtc->rtc_pad_hold_count, 22u);
    ASSERT_EQ(mem_read32(mem, hold), 0u);

    /* A digitally owned held pad ignores subsequent changes to both GPIO
     * output latches and the RTC owner mux, but those writes still read back. */
    mem_write32(mem, gpio + 0x008u, 1u << 4u);
    mem_write32(mem, gpio + 0x024u, 1u << 4u);
    periph_gpio_set_input(periph, 4, 1);
    mem_write32(mem, hold, 1u << 4u);
    mem_write32(mem, gpio + 0x00Cu, 1u << 4u);
    mem_write32(mem, gpio + 0x028u, 1u << 4u);
    mem_write32(mem, io->base + 0x010u, rtc4);
    mem_write32(mem, pad4, io->pad_reset[4] | io->pad_mux_mask |
                           (1u << 13u));
    ASSERT_EQ(mem_read32(mem, pad4) & io->pad_mux_mask,
              io->pad_mux_mask);
    ASSERT_EQ(periph_gpio_pin_level(periph, 4), 1);
    ASSERT_EQ(periph_gpio_output_enabled(periph, 4), 1);
    ASSERT_EQ(mem_read32(mem, gpio + 0x03Cu) & (1u << 4u), 1u << 4u);
    ASSERT_EQ(mem_read32(mem, io->base + 0x024u) & rtc4, 0u);
    mem_write32(mem, hold, 0u);
    ASSERT_EQ(periph_gpio_pin_level(periph, 4), 0);
    ASSERT_EQ(periph_gpio_output_enabled(periph, 4), 1);
    ASSERT_EQ(mem_read32(mem, gpio + 0x03Cu) & (1u << 4u), 0u);
    ASSERT_EQ(mem_read32(mem, io->base + 0x024u) & rtc4, rtc4);

    /* An RTC-owned pad keeps FUN_IE and its owner while held. */
    periph_gpio_set_input(periph, 11, 1);
    mem_write32(mem, io->base + 0x004u, rtc11);
    mem_write32(mem, io->base + 0x010u, rtc11);
    mem_write32(mem, pad11, io->pad_reset[11] | io->pad_mux_mask |
                            (1u << 13u));
    mem_write32(mem, hold, 1u << 11u);
    mem_write32(mem, io->base + 0x008u, rtc11);
    mem_write32(mem, io->base + 0x014u, rtc11);
    mem_write32(mem, pad11, io->pad_reset[11]);
    ASSERT_EQ(periph_gpio_pin_level(periph, 11), 1);
    ASSERT_EQ(periph_gpio_output_enabled(periph, 11), 1);
    ASSERT_EQ(mem_read32(mem, gpio + 0x03Cu) & (1u << 11u), 0u);
    ASSERT_EQ(mem_read32(mem, io->base + 0x024u) & rtc11, rtc11);
    mem_write32(mem, hold, 0u);
    ASSERT_EQ(periph_gpio_pin_level(periph, 11), 0);
    ASSERT_EQ(periph_gpio_output_enabled(periph, 11), 0);
    ASSERT_EQ(mem_read32(mem, gpio + 0x03Cu) & (1u << 11u), 1u << 11u);
    ASSERT_EQ(mem_read32(mem, io->base + 0x024u) & rtc11, 0u);

    /* GPIO21 has two independent hold sources: clearing only one does not
     * release the physical pad. */
    mem_write32(mem, gpio + 0x008u, 1u << 21u);
    mem_write32(mem, gpio + 0x024u, 1u << 21u);
    mem_write32(mem, hold, 1u << 21u);
    mem_write32(mem, digital_hold, 1u << 1u);
    mem_write32(mem, gpio + 0x00Cu, 1u << 21u);
    mem_write32(mem, hold, 0u);
    ASSERT_EQ(periph_gpio_pin_level(periph, 21), 1);
    mem_write32(mem, digital_hold, 0u);
    ASSERT_EQ(periph_gpio_pin_level(periph, 21), 0);

    int before = periph_unhandled_count(periph);
    mem_write32(mem, hold, 1u << 22u);
    ASSERT_EQ(mem_read32(mem, hold), 0u);
    ASSERT_EQ(periph_unhandled_count(periph), before + 1);
    ASSERT_EQ(mem_unmapped_count(mem), 0u);
    periph_destroy(periph);
    mem_destroy(mem);
}

TEST(rtc_cntl_rtc_pad_hold_survives_rebuild_with_frozen_mux)
{
    const flexe_target_desc_t *s3 =
        flexe_target_by_id(FLEXE_TARGET_ESP32S3);
    const flexe_rtc_cntl_desc_t *rtc = &s3->rtc_cntl;
    const flexe_rtc_io_desc_t *io = &s3->rtc_io;
    xtensa_mem_t *mem = mem_create_for_target(s3);
    esp32_periph_t *periph = mem ? periph_create(mem) : NULL;
    ASSERT_TRUE(periph != NULL);
    if (!periph) {
        mem_destroy(mem);
        return;
    }

    uint32_t hold = rtc->base + rtc->rtc_pad_hold_offset;
    uint32_t pad4 = io->base + io->pad_base_offset + 4u * 4u;
    uint32_t bit4 = 1u << (io->data_shift + 4u);
    uint32_t gpio = s3->gpio.base;
    periph_gpio_set_input(periph, 4, 1);
    mem_write32(mem, io->base + 0x004u, bit4);
    mem_write32(mem, io->base + 0x010u, bit4);
    mem_write32(mem, pad4, io->pad_reset[4] | io->pad_mux_mask |
                           (1u << 13u));
    mem_write32(mem, hold, 1u << 4u);
    mem_write32(mem, io->base + 0x008u, bit4);
    mem_write32(mem, pad4, io->pad_reset[4]);

    periph_pad_hold_t snapshot;
    periph_pad_hold_snapshot(periph, &snapshot);
    ASSERT_EQ(snapshot.target_rtc_io_hold, 1u << 4u);
    ASSERT_EQ64(snapshot.target_gpio.mask, UINT64_C(1) << 4u);
    ASSERT_EQ(snapshot.target_rtc_io.mask, 1u << 4u);
    ASSERT_EQ(snapshot.target_rtc_io.pad[4],
              io->pad_reset[4] | io->pad_mux_mask | (1u << 13u));
    periph_destroy(periph);
    periph = periph_create(mem);
    ASSERT_TRUE(periph != NULL);
    if (periph) {
        ASSERT_EQ(mem_read32(mem, hold), 0u);
        periph_gpio_set_input(periph, 4, 1);
        periph_pad_hold_restore(periph, &snapshot);
        ASSERT_EQ(mem_read32(mem, hold), 1u << 4u);
        ASSERT_EQ(periph_gpio_pin_level(periph, 4), 1);
        ASSERT_EQ(periph_gpio_output_enabled(periph, 4), 1);
        ASSERT_EQ(mem_read32(mem, io->base + 0x024u) & bit4, bit4);
        ASSERT_EQ(mem_read32(mem, gpio + 0x03Cu) & (1u << 4u), 0u);
        mem_write32(mem, hold, 0u);
        ASSERT_EQ(periph_gpio_pin_level(periph, 4), 0);
        ASSERT_EQ(mem_read32(mem, io->base + 0x024u) & bit4, 0u);
        ASSERT_EQ(mem_read32(mem, gpio + 0x03Cu) & (1u << 4u), 1u << 4u);
        ASSERT_EQ(periph_unhandled_count(periph), 0);
    }
    periph_destroy(periph);
    mem_destroy(mem);
}

TEST(rtc_cntl_interrupt_bank_latches_masks_clears_and_publishes_level)
{
    const flexe_target_desc_t *s3 =
        flexe_target_by_id(FLEXE_TARGET_ESP32S3);
    const flexe_rtc_cntl_desc_t *desc = &s3->rtc_cntl;
    xtensa_mem_t *mem = mem_create_for_target(s3);
    rtc_cntl_irq_probe_t probe = {0};
    flexe_rtc_cntl_t *rtc = flexe_rtc_cntl_create(
        mem, NULL, NULL, NULL, NULL, NULL,
        rtc_cntl_test_irq_changed, &probe, NULL, NULL);
    ASSERT_TRUE(mem != NULL);
    ASSERT_TRUE(rtc != NULL);
    if (!mem || !rtc) {
        flexe_rtc_cntl_destroy(rtc);
        mem_destroy(mem);
        return;
    }

    uint32_t ena = desc->base + desc->interrupt_enable_offset;
    uint32_t raw = desc->base + desc->interrupt_raw_offset;
    uint32_t status = desc->base + desc->interrupt_status_offset;
    uint32_t clear = desc->base + desc->interrupt_clear_offset;
    uint32_t brownout = 1u << 9;
    uint32_t software_raw = 1u << 20;
    ASSERT_EQ(mem_read32(mem, ena), 0u);
    ASSERT_EQ(mem_read32(mem, raw), 0u);
    ASSERT_EQ(mem_read32(mem, status), 0u);
    ASSERT_EQ(mem_read32(mem, clear), 0u);

    /* A disabled event still latches RAW. Enabling it later raises the one
     * aggregate level, and W1C removes both the condition and the level. */
    flexe_rtc_cntl_set_interrupts(rtc, brownout, true);
    ASSERT_EQ(mem_read32(mem, raw), brownout);
    ASSERT_EQ(mem_read32(mem, status), 0u);
    ASSERT_EQ(probe.changes, 0u);
    mem_write32(mem, ena, brownout);
    ASSERT_EQ(mem_read32(mem, status), brownout);
    ASSERT_EQ(probe.changes, 1u);
    ASSERT_TRUE(probe.level);
    mem_write32(mem, ena, 0u);
    ASSERT_EQ(mem_read32(mem, raw), brownout);
    ASSERT_EQ(probe.changes, 2u);
    ASSERT_FALSE(probe.level);
    mem_write32(mem, ena, brownout);
    ASSERT_EQ(probe.changes, 3u);
    ASSERT_TRUE(probe.level);
    mem_write32(mem, clear, brownout);
    ASSERT_EQ(mem_read32(mem, raw), 0u);
    ASSERT_EQ(probe.changes, 4u);
    ASSERT_FALSE(probe.level);

    /* S3 exposes only bit 20 as software-writable RAW. Writes to physical
     * producer bits and injections outside the descriptor mask are ignored. */
    mem_write32(mem, ena, brownout | software_raw);
    mem_write32(mem, raw, brownout);
    ASSERT_EQ(mem_read32(mem, raw), 0u);
    mem_write32(mem, raw, software_raw);
    ASSERT_EQ(mem_read32(mem, status), software_raw);
    ASSERT_EQ(probe.changes, 5u);
    ASSERT_TRUE(probe.level);
    mem_write32(mem, raw, 0u);
    ASSERT_EQ(probe.changes, 6u);
    ASSERT_FALSE(probe.level);
    flexe_rtc_cntl_set_interrupts(rtc, 1u << 31, true);
    ASSERT_EQ(mem_read32(mem, raw), 0u);
    ASSERT_EQ(probe.changes, 6u);

    flexe_rtc_cntl_destroy(rtc);
    mem_destroy(mem);
}

TEST(rtc_cntl_interrupt_routes_through_target_matrix)
{
    const flexe_target_desc_t *s3 =
        flexe_target_by_id(FLEXE_TARGET_ESP32S3);
    const flexe_rtc_cntl_desc_t *desc = &s3->rtc_cntl;
    xtensa_mem_t *mem = mem_create_for_target(s3);
    esp32_periph_t *periph = periph_create(mem);
    xtensa_cpu_t cpu;
    ASSERT_TRUE(mem != NULL);
    ASSERT_TRUE(periph != NULL);
    if (!mem || !periph) {
        periph_destroy(periph);
        mem_destroy(mem);
        return;
    }

    xtensa_cpu_init_for_target(&cpu, s3);
    cpu.mem = mem;
    periph_attach_cpus(periph, &cpu, NULL);
    uint32_t line = 5u;
    uint32_t raw_bit = desc->interrupt_raw_writable_mask;
    uint32_t map = s3->interrupt_matrix.base +
                   s3->interrupt_matrix.map_offset[0] +
                   desc->interrupt_source * 4u;
    mem_write32(mem, map, line);
    mem_write32(mem, desc->base + desc->interrupt_enable_offset, raw_bit);
    mem_write32(mem, desc->base + desc->interrupt_raw_offset, raw_bit);
    ASSERT_TRUE(periph_interrupt_pending(periph,
                                         desc->interrupt_source));
    ASSERT_TRUE(cpu.interrupt & (1u << line));
    mem_write32(mem, desc->base + desc->interrupt_clear_offset, raw_bit);
    ASSERT_FALSE(periph_interrupt_pending(periph,
                                          desc->interrupt_source));
    ASSERT_EQ(cpu.interrupt & (1u << line), 0u);
    ASSERT_EQ(periph_unhandled_count(periph), 0);

    periph_destroy(periph);
    mem_destroy(mem);
}

TEST(rtc_cntl_watchdog_schedules_feed_interrupt_and_reset)
{
    const flexe_target_desc_t *s3 =
        flexe_target_by_id(FLEXE_TARGET_ESP32S3);
    const flexe_rtc_cntl_desc_t *desc = &s3->rtc_cntl;
    xtensa_mem_t *mem = mem_create_for_target(s3);
    esp32_periph_t *periph = periph_create(mem);
    xtensa_cpu_t cpu;
    ASSERT_TRUE(mem != NULL);
    ASSERT_TRUE(periph != NULL);
    if (!mem || !periph) {
        periph_destroy(periph);
        mem_destroy(mem);
        return;
    }

    xtensa_cpu_init_for_target(&cpu, s3);
    cpu.mem = mem;
    periph_attach_cpus(periph, &cpu, NULL);

    uint32_t config0 = desc->base + desc->wdt_config_offset[0];
    uint32_t stage0_hold = desc->base + desc->wdt_config_offset[1];
    uint32_t stage1_hold = desc->base + desc->wdt_config_offset[2];
    uint32_t feed = desc->base + desc->wdt_feed_offset;
    uint32_t protect = desc->base + desc->wdt_write_protect_offset;
    uint32_t interrupt_enable =
        desc->base + desc->interrupt_enable_offset;
    uint32_t interrupt_clear =
        desc->base + desc->interrupt_clear_offset;
    ASSERT_EQ(mem_read32(mem, config0),
              desc->wdt_config_reset[0] &
              ~desc->wdt_flashboot_enable_mask);
    ASSERT_EQ(mem_read32(mem, stage0_hold), 200000u);
    ASSERT_EQ(mem_read32(mem, protect), desc->wdt_write_protect_key);

    /* The application handoff already cleared flashboot mode. Configure four
     * effective slow ticks for stage 0 (the revision-0 multiplier is two),
     * followed by a three-tick system-reset stage. */
    uint32_t base_config = desc->wdt_config_reset[0] &
        ~(desc->wdt_enable_mask | desc->wdt_flashboot_enable_mask);
    mem_write32(mem, config0, base_config);
    mem_write32(mem, stage0_hold, 2u);
    mem_write32(mem, stage1_hold, 3u);
    mem_write32(mem, interrupt_enable, desc->wdt_interrupt_mask);

    uint32_t line = 5u;
    uint32_t map = s3->interrupt_matrix.base +
                   s3->interrupt_matrix.map_offset[0] +
                   desc->interrupt_source * 4u;
    mem_write32(mem, map, line);
    uint32_t armed_config = base_config | desc->wdt_enable_mask |
        (1u << desc->wdt_stage_action_shift[0]) |
        ((uint32_t)FLEXE_RTC_CNTL_WDT_RESET_SYSTEM <<
         desc->wdt_stage_action_shift[1]);
    mem_write32(mem, config0, armed_config);
    ASSERT_TRUE(cpu.next_timer_event != UINT32_MAX);
    ASSERT_TRUE((int32_t)(cpu.next_timer_event - cpu.ccount) > 1);

    /* A feed one CPU cycle before expiry must restart stage 0. */
    cpu.ccount = cpu.next_timer_event - 1u;
    cpu.periph_event(&cpu);
    ASSERT_EQ(mem_read32(mem, desc->base + desc->interrupt_raw_offset) &
              desc->wdt_interrupt_mask, 0u);
    mem_write32(mem, feed, desc->wdt_feed_mask);
    uint32_t fed_deadline = cpu.next_timer_event;
    ASSERT_TRUE((int32_t)(fed_deadline - cpu.ccount) > 1);
    cpu.ccount = fed_deadline - 1u;
    cpu.periph_event(&cpu);
    ASSERT_FALSE(periph_interrupt_pending(periph,
                                           desc->interrupt_source));
    cpu.ccount++;
    cpu.periph_event(&cpu);
    ASSERT_TRUE(periph_interrupt_pending(periph,
                                          desc->interrupt_source));
    ASSERT_TRUE(cpu.interrupt & (1u << line));
    mem_write32(mem, interrupt_clear, desc->wdt_interrupt_mask);
    ASSERT_FALSE(periph_interrupt_pending(periph,
                                           desc->interrupt_source));

    /* Locking makes both configuration writes and feed pulses inert. The
     * already-running stage 1 therefore reaches its reset boundary. */
    mem_write32(mem, protect, 0u);
    mem_write32(mem, stage1_hold, 1000u);
    ASSERT_EQ(mem_read32(mem, stage1_hold), 3u);
    uint32_t reset_deadline = cpu.next_timer_event;
    mem_write32(mem, feed, desc->wdt_feed_mask);
    ASSERT_EQ(cpu.next_timer_event, reset_deadline);
    cpu.ccount = reset_deadline - 1u;
    cpu.periph_event(&cpu);
    ASSERT_FALSE(periph_take_reset_request(periph));
    cpu.ccount++;
    cpu.periph_event(&cpu);
    ASSERT_TRUE(periph_take_reset_request(periph));
    ASSERT_FALSE(periph_take_reset_request(periph));
    ASSERT_EQ(periph_unhandled_count(periph), 0);

    periph_destroy(periph);
    mem_destroy(mem);
}

void run_rtc_cntl_tests(void)
{
    TEST_SUITE("Target RTC controller");
    RUN_TEST(rtc_cntl_storage_resets_persists_and_delegates);
    RUN_TEST(rtc_cntl_sar_i2c_power_gates_analog_slave);
    RUN_TEST(rtc_cntl_application_handoff_uses_target_clocks);
    RUN_TEST(rtc_cntl_watchdog_reports_unmodeled_configuration);
    RUN_TEST(rtc_cntl_counter_tracks_shared_time_and_frequency);
    RUN_TEST(rtc_cntl_switches_slow_clock_at_an_exact_boundary);
    RUN_TEST(rtc_cntl_unmodeled_power_registers_remain_unsupported);
    RUN_TEST(rtc_cntl_digital_pad_hold_freezes_physical_gpio_not_latches);
    RUN_TEST(rtc_cntl_digital_pad_hold_survives_rebuild_without_unheld_gpio);
    RUN_TEST(rtc_cntl_rtc_pad_hold_freezes_mux_input_and_output);
    RUN_TEST(rtc_cntl_rtc_pad_hold_survives_rebuild_with_frozen_mux);
    RUN_TEST(rtc_cntl_interrupt_bank_latches_masks_clears_and_publishes_level);
    RUN_TEST(rtc_cntl_interrupt_routes_through_target_matrix);
    RUN_TEST(rtc_cntl_watchdog_schedules_feed_interrupt_and_reset);
}
