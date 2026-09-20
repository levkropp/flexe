/* Target-described always-on RTC controller tests. */
#include "test_helpers.h"
#include "peripherals.h"
#include "regi2c.h"
#include "rtc_cntl.h"
#include "target.h"

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

typedef struct {
    unsigned changes;
    uint32_t powered;
    uint32_t isolated;
} rtc_cntl_domain_probe_t;

typedef struct {
    unsigned changes;
    uint32_t powered;
} rtc_cntl_supply_probe_t;

typedef struct {
    unsigned changes;
    flexe_rtc_cntl_control_state_t state;
} rtc_cntl_control_probe_t;

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

static void rtc_cntl_test_domain_changed(void *ctx, uint32_t powered,
                                         uint32_t isolated)
{
    rtc_cntl_domain_probe_t *probe = ctx;
    probe->changes++;
    probe->powered = powered;
    probe->isolated = isolated;
}

static void rtc_cntl_test_supply_changed(void *ctx, uint32_t powered)
{
    rtc_cntl_supply_probe_t *probe = ctx;
    probe->changes++;
    probe->powered = powered;
}

static void rtc_cntl_test_control_changed(
    void *ctx, const flexe_rtc_cntl_control_state_t *state)
{
    rtc_cntl_control_probe_t *probe = ctx;
    probe->changes++;
    probe->state = *state;
}

static void rtc_cntl_test_reset_requested(
    void *ctx, flexe_rtc_cntl_reset_action_t action)
{
    *(flexe_rtc_cntl_reset_action_t *)ctx = action;
}

TEST(rtc_cntl_software_stall_uses_both_fields_and_preserves_reset_commands)
{
    const flexe_target_desc_t *s3 =
        flexe_target_by_id(FLEXE_TARGET_ESP32S3);
    const flexe_rtc_cntl_desc_t *desc = &s3->rtc_cntl;
    xtensa_mem_t *mem = mem_create_for_target(s3);
    rtc_cntl_fallback_t fallback = {0};
    flexe_rtc_cntl_reset_action_t reset = 0;
    flexe_rtc_cntl_t *rtc = flexe_rtc_cntl_create(
        mem, rtc_cntl_test_fallback_read,
        rtc_cntl_test_fallback_write, &fallback,
        NULL, NULL, NULL, NULL, rtc_cntl_test_reset_requested, &reset);
    ASSERT_TRUE(mem != NULL);
    ASSERT_TRUE(rtc != NULL);
    if (!mem || !rtc) {
        flexe_rtc_cntl_destroy(rtc);
        mem_destroy(mem);
        return;
    }

    uint32_t options = desc->base + desc->cpu_stall_options_offset;
    uint32_t high = desc->base + desc->cpu_stall_high_offset;
    ASSERT_EQ(mem_read32(mem, options), 0x1C00A000u);
    ASSERT_EQ(mem_read32(mem, high), 0u);
    ASSERT_FALSE(flexe_rtc_cntl_cpu_stalled(rtc, 0u));
    ASSERT_FALSE(flexe_rtc_cntl_cpu_stalled(rtc, 1u));

    /* Neither half alone is the 0x86 stall key. */
    mem_write32(mem, options, desc->cpu_stall_options_reset | 2u);
    ASSERT_FALSE(flexe_rtc_cntl_cpu_stalled(rtc, 1u));
    mem_write32(mem, high, 0x21u << 20);
    ASSERT_TRUE(flexe_rtc_cntl_cpu_stalled(rtc, 1u));
    ASSERT_FALSE(flexe_rtc_cntl_cpu_stalled(rtc, 0u));
    ASSERT_EQ(fallback.writes, 0u);

    mem_write32(mem, options, desc->cpu_stall_options_reset | 0xAu);
    mem_write32(mem, high, (0x21u << 26) | (0x21u << 20));
    ASSERT_TRUE(flexe_rtc_cntl_cpu_stalled(rtc, 0u));
    ASSERT_TRUE(flexe_rtc_cntl_cpu_stalled(rtc, 1u));
    mem_write32(mem, options, desc->cpu_stall_options_reset | 0x8u);
    ASSERT_TRUE(flexe_rtc_cntl_cpu_stalled(rtc, 0u));
    ASSERT_FALSE(flexe_rtc_cntl_cpu_stalled(rtc, 1u));
    mem_write32(mem, high, 0u);
    ASSERT_FALSE(flexe_rtc_cntl_cpu_stalled(rtc, 0u));
    ASSERT_EQ(fallback.writes, 0u);

    /* OPTIONS0 power controls retain exact readback and are exposed through
     * normalized state rather than being treated as unsupported MMIO. */
    uint32_t other = desc->cpu_stall_options_reset ^ (1u << 13);
    mem_write32(mem, options, other);
    ASSERT_EQ(mem_read32(mem, options), other);
    ASSERT_EQ(fallback.writes, 0u);
    mem_write32(mem, high, 1u);
    ASSERT_EQ(mem_read32(mem, high), 1u);
    ASSERT_EQ(fallback.writes, 1u);

    /* Write-only reset commands are not sticky register bits. */
    mem_write32(mem, options, other | desc->software_reset_cpu1_mask);
    ASSERT_EQ(reset, FLEXE_RTC_CNTL_SW_RESET_CPU1);
    ASSERT_EQ(mem_read32(mem, options), other);
    ASSERT_EQ(fallback.writes, 1u);
    mem_write32(mem, options, other | desc->software_reset_cpu0_mask);
    ASSERT_EQ(reset, FLEXE_RTC_CNTL_SW_RESET_CPU);
    ASSERT_EQ(mem_read32(mem, options), other);
    mem_write32(mem, options, other | desc->software_reset_system_mask);
    ASSERT_EQ(reset, FLEXE_RTC_CNTL_SW_RESET_SYSTEM);
    ASSERT_EQ(mem_read32(mem, options), other);

    flexe_rtc_cntl_destroy(rtc);
    mem_destroy(mem);
}

TEST(rtc_cntl_s3_routes_app_cpu_reset_without_system_reset)
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

    uint32_t options = desc->base + desc->cpu_stall_options_offset;
    int before = periph_unhandled_count(periph);
    mem_write32(mem, options,
                desc->cpu_stall_options_reset |
                desc->software_reset_cpu1_mask);
    ASSERT_TRUE(periph_take_cpu_reset_request(periph, 1u));
    ASSERT_FALSE(periph_take_cpu_reset_request(periph, 1u));
    ASSERT_FALSE(periph_take_cpu_reset_request(periph, 0u));
    ASSERT_FALSE(periph_take_reset_request(periph));
    ASSERT_EQ(mem_read32(mem, options),
              desc->cpu_stall_options_reset);
    ASSERT_EQ(periph_unhandled_count(periph), before);

    mem_write32(mem, options,
                desc->cpu_stall_options_reset |
                desc->software_reset_cpu0_mask);
    ASSERT_TRUE(periph_take_reset_request(periph));
    ASSERT_FALSE(periph_take_reset_request(periph));
    ASSERT_FALSE(periph_take_cpu_reset_request(periph, 1u));

    periph_destroy(periph);
    mem_destroy(mem);
}

TEST(rtc_cntl_s3_control_fabric_resolves_force_pairs_and_notifies)
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

    ASSERT_EQ(desc->options_supply_count, 4u);
    ASSERT_EQ(desc->options_isolation_count, 3u);
    ASSERT_EQ(desc->options_reset_count, 1u);
    ASSERT_EQ(desc->analog_control_count, 11u);
    flexe_rtc_cntl_control_state_t state;
    flexe_rtc_cntl_control_state(rtc, &state);
    ASSERT_EQ(state.powered_options_supplies, 0xFu);
    ASSERT_EQ(state.isolated_options_domains, 0u);
    ASSERT_EQ(state.reset_options_domains, 0u);
    ASSERT_EQ(state.enabled_analog_controls, 1u << 9u);
    ASSERT_EQ(state.xtal_enable_wait, 2u);
    ASSERT_FALSE(state.analog_reset_por_powered);
    ASSERT_FALSE(state.digital_pad_isolated);
    ASSERT_FALSE(state.digital_pad_autohold_enabled);
    ASSERT_FALSE(state.digital_isolation_enabled);

    rtc_cntl_control_probe_t probe = {0};
    flexe_rtc_cntl_set_control_listener(
        rtc, rtc_cntl_test_control_changed, &probe);
    ASSERT_EQ(probe.changes, 1u);

    uint32_t options = 7u << desc->options_xtal_wait_shift;
    options |= desc->options_supply[0].force_power_down_mask;
    options |= desc->options_supply[1].force_power_up_mask;
    options |= desc->options_supply[2].force_power_down_mask;
    options |= desc->options_supply[3].force_power_up_mask |
               desc->options_supply[3].force_power_down_mask;
    options |= desc->options_isolation[0].force_set_mask;
    options |= desc->options_isolation[1].force_clear_mask |
               desc->options_isolation[1].force_set_mask;
    options |= desc->options_isolation[2].force_clear_mask;
    options |= desc->options_reset[0].force_clear_mask |
               desc->options_reset[0].force_set_mask;
    uint32_t options_addr = desc->base + desc->cpu_stall_options_offset;
    mem_write32(mem, options_addr, options);
    ASSERT_EQ(mem_read32(mem, options_addr), options);
    ASSERT_EQ(probe.changes, 2u);
    ASSERT_EQ(probe.state.powered_options_supplies, 1u << 1u);
    ASSERT_EQ(probe.state.isolated_options_domains, 3u);
    ASSERT_EQ(probe.state.reset_options_domains, 1u);
    ASSERT_EQ(probe.state.xtal_enable_wait, 7u);

    /* Set wins conflicting isolation/reset pairs; power-down wins supplies.
     * Repeating normalized state does not emit a spurious transition. */
    mem_write32(mem, options_addr, options);
    ASSERT_EQ(probe.changes, 2u);
    mem_write32(mem, options_addr, options | (1u << 22u));
    ASSERT_EQ(mem_read32(mem, options_addr), options);
    ASSERT_EQ(fallback.writes, 1u);
    ASSERT_EQ(probe.changes, 2u);

    uint32_t analog = desc->analog_control_mask[0] |
                      desc->analog_control_mask[2] |
                      desc->analog_control_mask[9] |
                      desc->analog_control_mask[10] |
                      desc->analog_reset_por_supply.force_power_up_mask |
                      desc->analog_reset_por_supply.force_power_down_mask;
    uint32_t analog_addr = desc->base + desc->analog_conf_offset;
    mem_write32(mem, analog_addr, analog);
    ASSERT_EQ(mem_read32(mem, analog_addr), analog);
    ASSERT_EQ(probe.changes, 3u);
    ASSERT_EQ(probe.state.enabled_analog_controls,
              (1u << 0u) | (1u << 2u) | (1u << 9u) | (1u << 10u));
    ASSERT_FALSE(probe.state.analog_reset_por_powered);
    analog &= ~desc->analog_reset_por_supply.force_power_down_mask;
    mem_write32(mem, analog_addr, analog);
    ASSERT_TRUE(probe.state.analog_reset_por_powered);
    ASSERT_EQ(probe.changes, 4u);
    ASSERT_TRUE(mem_read32(mem, analog_addr) &
                desc->analog_control_mask[2]);
    mem_write32(mem, analog_addr, analog | (1u << 21u));
    ASSERT_EQ(mem_read32(mem, analog_addr), analog);
    ASSERT_EQ(fallback.writes, 2u);
    ASSERT_EQ(probe.changes, 4u);

    uint32_t iso = desc->digital_iso_reset;
    iso &= ~desc->digital_pad_isolation.force_clear_mask;
    iso |= desc->digital_pad_isolation.force_set_mask |
           desc->digital_pad_autohold_enable_mask;
    iso &= ~desc->digital_isolation.force_clear_mask;
    iso |= desc->digital_isolation.force_set_mask;
    uint32_t iso_addr = desc->base + desc->digital_iso_offset;
    mem_write32(mem, iso_addr, iso);
    ASSERT_EQ(mem_read32(mem, iso_addr), iso);
    ASSERT_EQ(probe.changes, 5u);
    ASSERT_TRUE(probe.state.digital_pad_isolated);
    ASSERT_TRUE(probe.state.digital_pad_autohold_enabled);
    ASSERT_TRUE(probe.state.digital_isolation_enabled);

    mem_write32(mem, iso_addr,
                iso | desc->digital_pad_isolation.force_clear_mask);
    ASSERT_EQ(probe.changes, 5u);
    mem_write32(mem, iso_addr, iso | desc->digital_iso_strobe_mask);
    ASSERT_EQ(mem_read32(mem, iso_addr), iso);
    ASSERT_EQ(probe.changes, 5u);
    mem_write32(mem, iso_addr, iso | desc->digital_iso_read_only_mask);
    ASSERT_EQ(mem_read32(mem, iso_addr), iso);
    ASSERT_EQ(fallback.writes, 2u);
    mem_write32(mem, iso_addr, iso | 1u);
    ASSERT_EQ(mem_read32(mem, iso_addr), iso);
    ASSERT_EQ(fallback.writes, 3u);
    ASSERT_EQ(fallback.last_write_addr, iso_addr);

    flexe_rtc_cntl_destroy(rtc);
    mem_destroy(mem);
}

TEST(rtc_cntl_s3_sequence_timers_read_back_and_enable_cpu_stall)
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

    ASSERT_EQ(desc->sequence_register_count, 6u);
    const uint16_t expected_offset[] = {
        0x01Cu, 0x020u, 0x024u, 0x028u, 0x02Cu, 0x030u,
    };
    const uint32_t expected_reset[] = {
        0x28140403u, 0x01080000u, 0x14160A08u,
        0x10200A08u, 0x00008000u, 0x10200A08u,
    };
    const uint32_t expected_mask[] = {
        UINT32_MAX, 0xFFFF8000u, UINT32_MAX,
        UINT32_MAX, 0x0000FF00u, UINT32_MAX,
    };
    for (unsigned i = 0u; i < desc->sequence_register_count; i++) {
        const flexe_rtc_sequence_register_desc_t *reg =
            &desc->sequence_register[i];
        ASSERT_EQ(reg->offset, expected_offset[i]);
        ASSERT_EQ(reg->reset, expected_reset[i]);
        ASSERT_EQ(reg->writable_mask, expected_mask[i]);
        uint32_t addr = desc->base + reg->offset;
        ASSERT_EQ(mem_read32(mem, addr), reg->reset);
        uint32_t expected = reg->reset ^
                            (reg->writable_mask & 0x00AA55AAu);
        mem_write32(mem, addr, expected);
        ASSERT_EQ(mem_read32(mem, addr), expected);
    }
    ASSERT_EQ(fallback.reads, 0u);
    ASSERT_EQ(fallback.writes, 0u);

    /* TIMER2/TIMER5 have reserved bits: an unsupported write remains
     * diagnostic and cannot contaminate their documented fields. */
    const flexe_rtc_sequence_register_desc_t *timer2 =
        &desc->sequence_register[1];
    uint32_t timer2_addr = desc->base + timer2->offset;
    uint32_t timer2_value = mem_read32(mem, timer2_addr);
    mem_write32(mem, timer2_addr, timer2_value | 1u);
    ASSERT_EQ(mem_read32(mem, timer2_addr), timer2_value);
    ASSERT_EQ(fallback.writes, 1u);
    ASSERT_EQ(fallback.last_write_addr, timer2_addr);
    const flexe_rtc_sequence_register_desc_t *timer5 =
        &desc->sequence_register[4];
    uint32_t timer5_addr = desc->base + timer5->offset;
    uint32_t timer5_value = mem_read32(mem, timer5_addr);
    mem_write32(mem, timer5_addr, timer5_value | 1u);
    ASSERT_EQ(mem_read32(mem, timer5_addr), timer5_value);
    ASSERT_EQ(fallback.writes, 2u);

    uint32_t options = desc->base + desc->cpu_stall_options_offset;
    uint32_t high = desc->base + desc->cpu_stall_high_offset;
    mem_write32(mem, options, desc->cpu_stall_options_reset | 2u);
    mem_write32(mem, high, 0x21u << 20);
    ASSERT_TRUE(flexe_rtc_cntl_cpu_stalled(rtc, 1u));
    uint32_t timer1_addr = desc->base +
                           desc->cpu_stall_enable_offset;
    uint32_t timer1 = mem_read32(mem, timer1_addr);
    ASSERT_TRUE((timer1 & desc->cpu_stall_enable_mask) != 0u);
    mem_write32(mem, timer1_addr,
                timer1 & ~desc->cpu_stall_enable_mask);
    ASSERT_FALSE(flexe_rtc_cntl_cpu_stalled(rtc, 1u));
    mem_write32(mem, timer1_addr, timer1);
    ASSERT_TRUE(flexe_rtc_cntl_cpu_stalled(rtc, 1u));
    ASSERT_EQ(fallback.writes, 2u);

    flexe_rtc_cntl_destroy(rtc);
    mem_destroy(mem);
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

    /* Analog controls publish normalized state even when no electrical/RF
     * consumer is attached. Reserved bits remain diagnostic. */
    mem_write32(mem, analog, rtc_desc->analog_conf_reset | (1u << 31));
    ASSERT_EQ(mem_read32(mem, analog),
              rtc_desc->analog_conf_reset | (1u << 31));
    ASSERT_EQ(fallback.writes, 1u);
    mem_write32(mem, analog, rtc_desc->analog_conf_reset | (1u << 31) | 1u);
    ASSERT_EQ(mem_read32(mem, analog),
              rtc_desc->analog_conf_reset | (1u << 31));
    ASSERT_EQ(fallback.writes, 2u);
    ASSERT_EQ(fallback.last_write_addr, analog);

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

    /* Moving back to supported stage actions is handled normally. Reset
     * target enables and pulse-width fields have exact architectural
     * readback; functional mode intentionally collapses their pulse timing
     * into the atomic reset boundary. */
    mem_write32(mem, protect, desc->wdt_write_protect_key);
    mem_write32(mem, config0, desc->wdt_config_reset[0]);
    ASSERT_EQ(fallback.writes, 1u);
    mem_write32(mem, config0, desc->wdt_config_reset[0] ^ 1u);
    ASSERT_EQ(mem_read32(mem, config0), desc->wdt_config_reset[0] ^ 1u);
    ASSERT_EQ(fallback.writes, 1u);
    mem_write32(mem, config0, 0x0007EE00u);
    ASSERT_EQ(mem_read32(mem, config0), 0x0007EE00u);
    ASSERT_EQ(fallback.writes, 1u);

    /* Feed is a one-bit write-only command; unexpected command bits remain
     * visible to the common unsupported-access path. */
    uint32_t feed = desc->base + desc->wdt_feed_offset;
    mem_write32(mem, feed, desc->wdt_feed_mask | 1u);
    ASSERT_EQ(fallback.writes, 2u);
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

    /* RTC_FAST_CLK is a separate one-bit mux: XTAL/2 and RC_FAST are both
     * modeled target sources and do not disturb the slow-counter phase. */
    mem_write32(mem, clock_addr, xtal32k | (1u << 29u));
    ASSERT_TRUE(mem_read32(mem, clock_addr) & (1u << 29u));
    ASSERT_EQ(periph_unhandled_count(periph), 0);

    /* The fourth mux value is reserved. Preserve what firmware wrote, use
     * the deterministic target fallback rate, and never claim support. */
    uint32_t reserved =
        (mem_read32(mem, clock_addr) & ~desc->slow_clock_select_mask) |
        desc->slow_clock_select_mask;
    mem_write32(mem, clock_addr, reserved);
    ASSERT_EQ(mem_read32(mem, clock_addr), reserved);
    ASSERT_EQ(periph_unhandled_count(periph), 1);

    periph_destroy(periph);
    mem_destroy(mem);
}

TEST(rtc_cntl_s3_fast_clock_and_date_register_follow_descriptor)
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

    uint32_t clock = desc->base + desc->clock_conf_offset;
    ASSERT_EQ(desc->fast_clock_source_hz[0], 20000000u);
    ASSERT_EQ(desc->fast_clock_source_hz[1], 17500000u);
    ASSERT_EQ(flexe_rtc_cntl_fast_clock_hz(rtc), 20000000u);
    mem_write32(mem, clock,
                desc->clock_conf_reset | desc->fast_clock_select_mask);
    ASSERT_EQ(flexe_rtc_cntl_fast_clock_hz(rtc), 17500000u);
    ASSERT_EQ(fallback.writes, 0u);
    /* Oscillator gating is exact retained configuration in functional mode;
     * analog start/stop latency is deliberately collapsed. */
    uint32_t gated = mem_read32(mem, clock) & ~(1u << 28u);
    mem_write32(mem, clock, gated);
    ASSERT_EQ(mem_read32(mem, clock), gated);
    ASSERT_EQ(fallback.writes, 0u);

    uint32_t date = desc->base + desc->date_offset;
    ASSERT_EQ(mem_read32(mem, date), 0x02101271u);
    /* DATE[18:13] is also the documented six-bit LDO trim payload. Its
     * electrical voltage is outside functional mode, but its register state
     * must be exact for ROM and IDF read-modify-write sequences. */
    uint32_t trim_mask = 0x3Fu << 13u;
    uint32_t trimmed = (desc->date_reset & ~trim_mask) | (7u << 13u);
    mem_write32(mem, date, trimmed);
    ASSERT_EQ(mem_read32(mem, date), trimmed);
    ASSERT_EQ(fallback.reads, 0u);
    ASSERT_EQ(fallback.writes, 0u);

    mem_write32(mem, date, UINT32_MAX);
    ASSERT_EQ(mem_read32(mem, date), desc->date_writable_mask);
    ASSERT_EQ(fallback.writes, 1u);
    ASSERT_EQ(fallback.last_write_addr, date);

    flexe_rtc_cntl_destroy(rtc);
    mem_destroy(mem);
}

TEST(rtc_cntl_s3_regulator_force_pairs_are_functional_and_trim_is_diagnostic)
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

    ASSERT_EQ(desc->regulator_offset, 0x84u);
    ASSERT_EQ(desc->regulator_reset, 0xA0000000u);
    ASSERT_EQ(desc->regulator_writable_mask, 0xF03FC080u);
    ASSERT_EQ(desc->regulator_supply_count, 2u);
    uint32_t addr = desc->base + desc->regulator_offset;
    uint32_t supplies = (1u << desc->regulator_supply_count) - 1u;
    ASSERT_EQ(mem_read32(mem, addr), desc->regulator_reset);
    ASSERT_EQ(flexe_rtc_cntl_powered_supplies(rtc), supplies);

    rtc_cntl_supply_probe_t probe = {0};
    flexe_rtc_cntl_set_supply_listener(
        rtc, rtc_cntl_test_supply_changed, &probe);
    ASSERT_EQ(probe.changes, 1u);
    ASSERT_EQ(probe.powered, supplies);

    /* Removing force-up leaves both supplies powered in functional mode and
     * therefore does not manufacture a logical transition. */
    mem_write32(mem, addr, 0u);
    ASSERT_EQ(mem_read32(mem, addr), 0u);
    ASSERT_EQ(flexe_rtc_cntl_powered_supplies(rtc), supplies);
    ASSERT_EQ(probe.changes, 1u);

    const flexe_rtc_supply_desc_t *regulator =
        &desc->regulator_supply[0];
    mem_write32(mem, addr, regulator->force_power_down_mask);
    ASSERT_EQ(flexe_rtc_cntl_powered_supplies(rtc), 2u);
    ASSERT_EQ(probe.changes, 2u);
    ASSERT_EQ(probe.powered, 2u);
    mem_write32(mem, addr, regulator->force_power_down_mask |
                           regulator->force_power_up_mask);
    ASSERT_EQ(flexe_rtc_cntl_powered_supplies(rtc), 2u);
    ASSERT_EQ(probe.changes, 2u); /* Force-down wins the conflict. */

    mem_write32(mem, addr, desc->regulator_reset);
    ASSERT_EQ(flexe_rtc_cntl_powered_supplies(rtc), supplies);
    ASSERT_EQ(probe.changes, 3u);

    /* SCK_DCAP and DIG_CAL retain their documented values, but changing
     * analog calibration remains visible to the unsupported-access audit. */
    uint32_t diagnostic = desc->regulator_reset |
                          (0x5Au << 14u) | (1u << 7u);
    mem_write32(mem, addr, diagnostic);
    ASSERT_EQ(mem_read32(mem, addr), diagnostic);
    ASSERT_EQ(fallback.writes, 1u);
    ASSERT_EQ(probe.changes, 3u);
    mem_write32(mem, addr, diagnostic | (1u << 13u));
    ASSERT_EQ(mem_read32(mem, addr), diagnostic);
    ASSERT_EQ(fallback.writes, 2u);
    ASSERT_EQ(fallback.last_write_addr, addr);

    flexe_rtc_cntl_destroy(rtc);
    mem_destroy(mem);
}

TEST(rtc_cntl_rejects_invalid_clock_and_control_geometry)
{
    const flexe_target_desc_t *s3 =
        flexe_target_by_id(FLEXE_TARGET_ESP32S3);
    flexe_target_desc_t invalid = *s3;
    invalid.rtc_cntl.date_offset = invalid.rtc_cntl.analog_conf_offset;
    xtensa_mem_t *mem = mem_create_for_target(&invalid);
    ASSERT_TRUE(mem != NULL);
    flexe_rtc_cntl_t *rtc = flexe_rtc_cntl_create(
        mem, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL);
    ASSERT_TRUE(rtc == NULL);
    mem_destroy(mem);

    invalid = *s3;
    invalid.rtc_cntl.fast_clock_select_mask =
        invalid.rtc_cntl.slow_clock_select_mask;
    mem = mem_create_for_target(&invalid);
    ASSERT_TRUE(mem != NULL);
    rtc = flexe_rtc_cntl_create(
        mem, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL);
    ASSERT_TRUE(rtc == NULL);
    mem_destroy(mem);

    invalid = *s3;
    invalid.rtc_cntl.options_supply[1].force_power_up_mask =
        invalid.rtc_cntl.options_supply[0].force_power_up_mask;
    mem = mem_create_for_target(&invalid);
    ASSERT_TRUE(mem != NULL);
    rtc = flexe_rtc_cntl_create(
        mem, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL);
    ASSERT_TRUE(rtc == NULL);
    mem_destroy(mem);

    invalid = *s3;
    invalid.rtc_cntl.analog_control_mask[0] =
        invalid.rtc_cntl.analog_reset_por_supply.force_power_up_mask;
    mem = mem_create_for_target(&invalid);
    ASSERT_TRUE(mem != NULL);
    rtc = flexe_rtc_cntl_create(
        mem, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL);
    ASSERT_TRUE(rtc == NULL);
    mem_destroy(mem);

    invalid = *s3;
    invalid.rtc_cntl.digital_pad_isolation.force_clear_mask =
        invalid.rtc_cntl.digital_domain[0].force_noiso_mask;
    mem = mem_create_for_target(&invalid);
    ASSERT_TRUE(mem != NULL);
    rtc = flexe_rtc_cntl_create(
        mem, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL);
    ASSERT_TRUE(rtc == NULL);
    mem_destroy(mem);
}

TEST(rtc_cntl_rejects_invalid_regulator_and_rtc_power_geometry)
{
    const flexe_target_desc_t *s3 =
        flexe_target_by_id(FLEXE_TARGET_ESP32S3);
    flexe_target_desc_t invalid = *s3;
    invalid.rtc_cntl.regulator_offset = invalid.rtc_cntl.analog_conf_offset;
    xtensa_mem_t *mem = mem_create_for_target(&invalid);
    ASSERT_TRUE(mem != NULL);
    flexe_rtc_cntl_t *rtc = flexe_rtc_cntl_create(
        mem, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL);
    ASSERT_TRUE(rtc == NULL);
    mem_destroy(mem);

    invalid = *s3;
    invalid.rtc_cntl.regulator_supply[1].force_power_up_mask =
        invalid.rtc_cntl.regulator_supply[0].force_power_up_mask;
    mem = mem_create_for_target(&invalid);
    ASSERT_TRUE(mem != NULL);
    rtc = flexe_rtc_cntl_create(
        mem, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL);
    ASSERT_TRUE(rtc == NULL);
    mem_destroy(mem);

    invalid = *s3;
    invalid.rtc_cntl.rtc_power_domain[1].follow_cpu_mask =
        invalid.rtc_cntl.rtc_power_domain[2].follow_cpu_mask;
    mem = mem_create_for_target(&invalid);
    ASSERT_TRUE(mem != NULL);
    rtc = flexe_rtc_cntl_create(
        mem, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL);
    ASSERT_TRUE(rtc == NULL);
    mem_destroy(mem);

    invalid = *s3;
    invalid.rtc_cntl.rtc_power_domain[1].follow_cpu_mask =
        invalid.rtc_cntl.rtc_power_domain[1].force_power_up_mask;
    mem = mem_create_for_target(&invalid);
    ASSERT_TRUE(mem != NULL);
    rtc = flexe_rtc_cntl_create(
        mem, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL);
    ASSERT_TRUE(rtc == NULL);
    mem_destroy(mem);

    invalid = *s3;
    invalid.rtc_cntl.digital_domain[0].sleep_power_down_mask =
        invalid.rtc_cntl.digital_domain[0].force_power_up_mask;
    mem = mem_create_for_target(&invalid);
    ASSERT_TRUE(mem != NULL);
    rtc = flexe_rtc_cntl_create(
        mem, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL);
    ASSERT_TRUE(rtc == NULL);
    mem_destroy(mem);

    invalid = *s3;
    invalid.rtc_cntl.rtc_follow_cpu_domain =
        invalid.rtc_cntl.digital_domain_count + 1u;
    mem = mem_create_for_target(&invalid);
    ASSERT_TRUE(mem != NULL);
    rtc = flexe_rtc_cntl_create(
        mem, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL);
    ASSERT_TRUE(rtc == NULL);
    mem_destroy(mem);
}

TEST(rtc_cntl_s3_configuration_bank_masks_retains_and_audits)
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

    const uint16_t expected_offset[] = {
        0x068u, 0x07Cu, 0x080u, 0x08Cu, 0x10Cu, 0x148u,
    };
    const uint32_t expected_reset[] = {
        0x00000000u, 0x0AB0BE0Au, 0x00010800u,
        0x00000000u, 0x000840CCu, 0x00000007u,
    };
    const uint32_t expected_writable[] = {
        0xFFFFF000u, 0xFEFFFEFFu, 0x3FFFFC00u,
        0x0FFFFFFFu, 0xFFFFFFFCu, 0x00000007u,
    };
    const uint32_t expected_read_only[] = {
        0u, 0x01000000u, 0u, 0u, 0u, 0u,
    };
    ASSERT_EQ(desc->config_register_count, 6u);
    for (unsigned i = 0u; i < desc->config_register_count; i++) {
        const flexe_rtc_config_register_desc_t *reg =
            &desc->config_register[i];
        ASSERT_EQ(reg->offset, expected_offset[i]);
        ASSERT_EQ(reg->reset, expected_reset[i]);
        ASSERT_EQ(reg->writable_mask, expected_writable[i]);
        ASSERT_EQ(reg->read_only_mask, expected_read_only[i]);
        ASSERT_EQ(mem_read32(mem, desc->base + reg->offset), reg->reset);
        if (reg->supported_mask != 0u) {
            uint32_t bit = reg->supported_mask &
                           (0u - reg->supported_mask);
            uint32_t changed = reg->reset ^ bit;
            mem_write32(mem, desc->base + reg->offset, changed);
            ASSERT_EQ(mem_read32(mem, desc->base + reg->offset), changed);
        }
    }
    ASSERT_EQ(fallback.reads, 0u);
    ASSERT_EQ(fallback.writes, 0u);

    /* SDIO's readiness bit is read-only and ignored by a normal RMW. */
    const flexe_rtc_config_register_desc_t *sdio =
        &desc->config_register[1];
    uint32_t sdio_addr = desc->base + sdio->offset;
    uint32_t sdio_before = mem_read32(mem, sdio_addr);
    mem_write32(mem, sdio_addr, sdio_before | sdio->read_only_mask);
    ASSERT_EQ(mem_read32(mem, sdio_addr), sdio_before);
    ASSERT_EQ(fallback.writes, 0u);

    /* Touch configuration reads its architectural reset, while activating
     * its unimplemented FSM remains visible to the access audit. */
    const flexe_rtc_config_register_desc_t *touch =
        &desc->config_register[4];
    uint32_t touch_addr = desc->base + touch->offset;
    mem_write32(mem, touch_addr, touch->reset | (1u << 31u));
    ASSERT_EQ(mem_read32(mem, touch_addr), touch->reset | (1u << 31u));
    ASSERT_EQ(fallback.writes, 1u);

    /* Clearing the FIB brownout selector gives software ownership to the
     * modeled brownout configuration without fabricating a voltage event. */
    const flexe_rtc_config_register_desc_t *fib =
        &desc->config_register[5];
    uint32_t fib_addr = desc->base + fib->offset;
    mem_write32(mem, fib_addr, fib->reset & ~(1u << 1u));
    ASSERT_EQ(mem_read32(mem, fib_addr), 0x5u);
    ASSERT_EQ(fallback.writes, 1u);

    /* Reserved bits neither latch nor disappear from the audit. */
    const flexe_rtc_config_register_desc_t *reject =
        &desc->config_register[0];
    uint32_t reject_addr = desc->base + reject->offset;
    uint32_t reject_before = mem_read32(mem, reject_addr);
    mem_write32(mem, reject_addr, reject_before | 1u);
    ASSERT_EQ(mem_read32(mem, reject_addr), reject_before);
    ASSERT_EQ(fallback.writes, 2u);

    /* Unsupported timestamp-control bits share TIME_UPDATE with the modeled
     * latch command and must still produce an explicit diagnostic. */
    mem_write32(mem, desc->base + desc->time_update_offset,
                desc->time_update_mask | (1u << 29u));
    ASSERT_EQ(fallback.writes, 3u);

    flexe_rtc_cntl_destroy(rtc);
    mem_destroy(mem);
}

TEST(rtc_cntl_rejects_invalid_configuration_bank_geometry)
{
    const flexe_target_desc_t *s3 =
        flexe_target_by_id(FLEXE_TARGET_ESP32S3);
    flexe_target_desc_t invalid = *s3;
    invalid.rtc_cntl.config_register_count =
        FLEXE_TARGET_RTC_CONFIG_REGISTER_MAX + 1u;
    xtensa_mem_t *mem = mem_create_for_target(&invalid);
    flexe_rtc_cntl_t *rtc = flexe_rtc_cntl_create(
        mem, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL);
    ASSERT_TRUE(mem != NULL);
    ASSERT_TRUE(rtc == NULL);
    flexe_rtc_cntl_destroy(rtc);
    mem_destroy(mem);

    invalid = *s3;
    invalid.rtc_cntl.config_register[1].offset =
        invalid.rtc_cntl.config_register[0].offset;
    mem = mem_create_for_target(&invalid);
    rtc = flexe_rtc_cntl_create(
        mem, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL);
    ASSERT_TRUE(mem != NULL);
    ASSERT_TRUE(rtc == NULL);
    flexe_rtc_cntl_destroy(rtc);
    mem_destroy(mem);

    invalid = *s3;
    invalid.rtc_cntl.config_register[0].offset =
        invalid.rtc_cntl.clock_conf_offset;
    mem = mem_create_for_target(&invalid);
    rtc = flexe_rtc_cntl_create(
        mem, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL);
    ASSERT_TRUE(mem != NULL);
    ASSERT_TRUE(rtc == NULL);
    flexe_rtc_cntl_destroy(rtc);
    mem_destroy(mem);

    invalid = *s3;
    invalid.rtc_cntl.config_register[0].supported_mask = 1u;
    mem = mem_create_for_target(&invalid);
    rtc = flexe_rtc_cntl_create(
        mem, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL);
    ASSERT_TRUE(mem != NULL);
    ASSERT_TRUE(rtc == NULL);
    flexe_rtc_cntl_destroy(rtc);
    mem_destroy(mem);
}

TEST(rtc_cntl_digital_domains_resolve_force_and_sleep_policy)
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

    ASSERT_EQ(desc->digital_domain_count, 6u);
    uint32_t domains = (1u << desc->digital_domain_count) - 1u;
    ASSERT_EQ(flexe_rtc_cntl_powered_digital_domains(rtc), domains);
    ASSERT_EQ(flexe_rtc_cntl_isolated_digital_domains(rtc), 0u);

    const unsigned wifi_index = 1u;
    const flexe_rtc_digital_domain_desc_t *wifi =
        &desc->digital_domain[wifi_index];
    uint32_t power_addr = desc->base + desc->digital_power_offset;
    uint32_t iso_addr = desc->base + desc->digital_iso_offset;
    uint32_t power = desc->digital_power_reset &
                     ~wifi->force_power_up_mask;
    power |= wifi->force_power_down_mask;
    mem_write32(mem, power_addr, power);
    ASSERT_EQ(flexe_rtc_cntl_powered_digital_domains(rtc) &
              (1u << wifi_index), 0u);
    ASSERT_EQ(fallback.writes, 0u);

    /* Force-down and force-isolation win deterministic conflicts, just as
     * the consuming device needs while firmware changes a pair by RMW. */
    mem_write32(mem, power_addr, power | wifi->force_power_up_mask);
    ASSERT_EQ(flexe_rtc_cntl_powered_digital_domains(rtc) &
              (1u << wifi_index), 0u);
    uint32_t iso = (desc->digital_iso_reset &
                    ~wifi->force_noiso_mask) |
                   wifi->force_iso_mask;
    mem_write32(mem, iso_addr, iso);
    ASSERT_EQ(flexe_rtc_cntl_isolated_digital_domains(rtc) &
              (1u << wifi_index), 1u << wifi_index);
    mem_write32(mem, iso_addr, iso | wifi->force_noiso_mask);
    ASSERT_EQ(flexe_rtc_cntl_isolated_digital_domains(rtc) &
              (1u << wifi_index), 1u << wifi_index);
    ASSERT_EQ(fallback.writes, 0u);

    /* With neither force asserted, sleep PD policy takes effect only while
     * the RTC sleep state is active and is undone at wake. */
    const unsigned wrap_index = 0u;
    const flexe_rtc_digital_domain_desc_t *wrap =
        &desc->digital_domain[wrap_index];
    power = desc->digital_power_reset & ~wrap->force_power_up_mask;
    power |= wrap->sleep_power_down_mask;
    mem_write32(mem, power_addr, power);
    iso = desc->digital_iso_reset & ~wrap->force_noiso_mask;
    mem_write32(mem, iso_addr, iso);
    ASSERT_TRUE(flexe_rtc_cntl_powered_digital_domains(rtc) &
                (1u << wrap_index));
    mem_write32(mem, desc->base + desc->sleep_timer_low_offset, 100u);
    mem_write32(mem, desc->base + desc->sleep_timer_high_offset,
                desc->sleep_alarm_enable_mask);
    mem_write32(mem, desc->base + desc->wakeup_state_offset,
                desc->timer_wakeup_mask << desc->wakeup_enable_shift);
    mem_write32(mem, desc->base + desc->sleep_state_offset,
                desc->sleep_enable_mask);
    ASSERT_EQ(flexe_rtc_cntl_powered_digital_domains(rtc) &
              (1u << wrap_index), 0u);
    ASSERT_TRUE(flexe_rtc_cntl_isolated_digital_domains(rtc) &
                (1u << wrap_index));
    flexe_rtc_cntl_finish_wake(rtc, desc->timer_wakeup_mask);
    ASSERT_TRUE(flexe_rtc_cntl_powered_digital_domains(rtc) &
                (1u << wrap_index));
    ASSERT_EQ(flexe_rtc_cntl_isolated_digital_domains(rtc) &
              (1u << wrap_index), 0u);
    ASSERT_EQ(fallback.writes, 0u);

    flexe_rtc_cntl_destroy(rtc);
    mem_destroy(mem);
}

TEST(rtc_cntl_rtc_domains_resolve_force_follow_cpu_and_sleep_policy)
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

    ASSERT_EQ(desc->rtc_power_domain_count, 3u);
    ASSERT_EQ(desc->rtc_follow_cpu_domain, 3u);
    ASSERT_EQ(desc->rtc_power_writable_mask, 0x003C0FFFu);
    uint32_t domains = (1u << desc->rtc_power_domain_count) - 1u;
    uint32_t power_addr = desc->base + desc->rtc_power_offset;
    uint32_t digital_addr = desc->base + desc->digital_power_offset;
    ASSERT_EQ(mem_read32(mem, power_addr), desc->rtc_power_reset);
    ASSERT_EQ(flexe_rtc_cntl_powered_rtc_domains(rtc), domains);
    ASSERT_EQ(flexe_rtc_cntl_isolated_rtc_domains(rtc), 0u);

    rtc_cntl_domain_probe_t probe = {0};
    flexe_rtc_cntl_set_rtc_domain_listener(
        rtc, rtc_cntl_test_domain_changed, &probe);
    ASSERT_EQ(probe.changes, 1u);
    ASSERT_EQ(probe.powered, domains);
    ASSERT_EQ(probe.isolated, 0u);

    /* Active-mode automatic sequencing keeps domains available when neither
     * force is asserted, so clearing the register is not a state change. */
    mem_write32(mem, power_addr, 0u);
    ASSERT_EQ(flexe_rtc_cntl_powered_rtc_domains(rtc), domains);
    ASSERT_EQ(flexe_rtc_cntl_isolated_rtc_domains(rtc), 0u);
    ASSERT_EQ(probe.changes, 1u);

    const flexe_rtc_power_domain_desc_t *peri =
        &desc->rtc_power_domain[0];
    uint32_t power = peri->force_power_down_mask |
                     peri->force_iso_mask;
    mem_write32(mem, power_addr, power);
    ASSERT_EQ(flexe_rtc_cntl_powered_rtc_domains(rtc), domains & ~1u);
    ASSERT_EQ(flexe_rtc_cntl_isolated_rtc_domains(rtc), 1u);
    ASSERT_EQ(probe.changes, 2u);
    power |= peri->force_power_up_mask | peri->force_noiso_mask;
    mem_write32(mem, power_addr, power);
    ASSERT_EQ(flexe_rtc_cntl_powered_rtc_domains(rtc), domains & ~1u);
    ASSERT_EQ(flexe_rtc_cntl_isolated_rtc_domains(rtc), 1u);
    ASSERT_EQ(probe.changes, 2u); /* Down and isolation win conflicts. */

    mem_write32(mem, power_addr, desc->rtc_power_reset);
    ASSERT_EQ(flexe_rtc_cntl_powered_rtc_domains(rtc), domains);
    ASSERT_EQ(flexe_rtc_cntl_isolated_rtc_domains(rtc), 0u);
    ASSERT_EQ(probe.changes, 3u);

    /* Slow memory follows the target-described CPU-top domain only after its
     * follow bit replaces force-up. No S3-specific domain index lives in the
     * RTC device model. */
    const flexe_rtc_power_domain_desc_t *slow =
        &desc->rtc_power_domain[1];
    unsigned cpu_index = desc->rtc_follow_cpu_domain - 1u;
    const flexe_rtc_digital_domain_desc_t *cpu =
        &desc->digital_domain[cpu_index];
    power = (desc->rtc_power_reset & ~slow->force_power_up_mask) |
            slow->follow_cpu_mask;
    mem_write32(mem, power_addr, power);
    ASSERT_EQ(probe.changes, 3u);
    uint32_t digital =
        (desc->digital_power_reset & ~cpu->force_power_up_mask) |
        cpu->force_power_down_mask;
    mem_write32(mem, digital_addr, digital);
    ASSERT_EQ(flexe_rtc_cntl_powered_rtc_domains(rtc), domains & ~(1u << 1));
    ASSERT_EQ(flexe_rtc_cntl_isolated_rtc_domains(rtc), 0u);
    ASSERT_EQ(probe.changes, 4u);

    power = (power & ~slow->force_noiso_mask) | slow->force_iso_mask;
    mem_write32(mem, power_addr, power);
    ASSERT_EQ(flexe_rtc_cntl_isolated_rtc_domains(rtc), 1u << 1);
    ASSERT_EQ(probe.changes, 5u);
    mem_write32(mem, digital_addr, desc->digital_power_reset);
    ASSERT_EQ(flexe_rtc_cntl_powered_rtc_domains(rtc), domains);
    ASSERT_EQ(flexe_rtc_cntl_isolated_rtc_domains(rtc), 1u << 1);
    ASSERT_EQ(probe.changes, 6u);

    /* RTC-peripheral sleep PD applies only while the sleep state is active
     * and reverses at wake, just like the digital-domain policy. */
    mem_write32(mem, power_addr,
                desc->rtc_power_reset | peri->sleep_power_down_mask);
    ASSERT_EQ(probe.changes, 7u); /* Removes slow-memory force isolation. */
    mem_write32(mem, desc->base + desc->sleep_timer_low_offset, 100u);
    mem_write32(mem, desc->base + desc->sleep_timer_high_offset,
                desc->sleep_alarm_enable_mask);
    mem_write32(mem, desc->base + desc->wakeup_state_offset,
                desc->timer_wakeup_mask << desc->wakeup_enable_shift);
    mem_write32(mem, desc->base + desc->sleep_state_offset,
                desc->sleep_enable_mask);
    ASSERT_EQ(flexe_rtc_cntl_powered_rtc_domains(rtc), domains & ~1u);
    ASSERT_EQ(probe.changes, 8u);
    flexe_rtc_cntl_finish_wake(rtc, desc->timer_wakeup_mask);
    ASSERT_EQ(flexe_rtc_cntl_powered_rtc_domains(rtc), domains);
    ASSERT_EQ(probe.changes, 9u);

    /* Reserved PWC bits do not contaminate the register and remain visible
     * to the unsupported-access audit. */
    mem_write32(mem, power_addr, desc->rtc_power_reset | (1u << 12u));
    ASSERT_EQ(mem_read32(mem, power_addr), desc->rtc_power_reset);
    ASSERT_EQ(fallback.writes, 1u);
    ASSERT_EQ(fallback.last_write_addr, power_addr);
    ASSERT_EQ(probe.changes, 9u);

    flexe_rtc_cntl_destroy(rtc);
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
    ASSERT_EQ(rtc->digital_pad_hold_first_gpio, 22u);
    ASSERT_EQ(rtc->digital_pad_hold_first_bit, 1u);
    /* GPIO26 is the first bonded digital pad, GPIO31 the last in bank 0. */
    mem_write32(mem, gpio->base + 0x008u, (1u << 26u) | (1u << 31u));
    mem_write32(mem, gpio->base + 0x024u, (1u << 26u) | (1u << 31u));
    mem_write32(mem, hold, (1u << 5u) | (1u << 10u));
    ASSERT_EQ(mem_read32(mem, hold), (1u << 5u) | (1u << 10u));

    mem_write32(mem, gpio->base + 0x00Cu, (1u << 26u) | (1u << 31u));
    mem_write32(mem, gpio->base + 0x028u, (1u << 26u) | (1u << 31u));
    ASSERT_EQ(mem_read32(mem, gpio->base + 0x004u), 0u);
    ASSERT_EQ(mem_read32(mem, gpio->base + 0x020u), 0u);
    ASSERT_EQ(periph_gpio_pin_level(periph, 26), 1);
    ASSERT_EQ(periph_gpio_output_enabled(periph, 31), 1);

    mem_write32(mem, hold, 1u << 10u);
    ASSERT_EQ(periph_gpio_pin_level(periph, 26), 0);
    ASSERT_EQ(periph_gpio_output_enabled(periph, 26), 0);
    ASSERT_EQ(periph_gpio_pin_level(periph, 31), 1);
    mem_write32(mem, hold, 0u);
    ASSERT_EQ(periph_gpio_pin_level(periph, 31), 0);
    ASSERT_EQ(periph_gpio_output_enabled(periph, 31), 0);

    /* GPIO48 uses the final bit and the upper GPIO bank. */
    mem_write32(mem, gpio->base + 0x014u, 1u << 16u);
    mem_write32(mem, gpio->base + 0x030u, 1u << 16u);
    mem_write32(mem, hold, 1u << 27u);
    mem_write32(mem, gpio->base + 0x018u, 1u << 16u);
    ASSERT_EQ(periph_gpio_pin_level(periph, 48), 1);
    mem_write32(mem, hold, 0u);
    ASSERT_EQ(periph_gpio_pin_level(periph, 48), 0);

    int before = periph_unhandled_count(periph);
    /* Bit 1 maps to unbonded GPIO22; bit 28 is not an S3 hold bit. */
    mem_write32(mem, hold, (1u << 1u) | (1u << 28u));
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
                (1u << 26u) | (1u << 27u));
    mem_write32(mem, gpio->base + 0x024u,
                (1u << 26u) | (1u << 27u));
    mem_write32(mem, hold, 1u << 5u);
    periph_pad_hold_t snapshot;
    periph_pad_hold_snapshot(periph, &snapshot);
    ASSERT_EQ64(snapshot.target_gpio.mask, UINT64_C(1) << 26u);
    ASSERT_EQ(snapshot.target_rtc_hold, 1u << 5u);

    periph_destroy(periph);
    periph = periph_create(mem);
    ASSERT_TRUE(periph != NULL);
    if (periph) {
        ASSERT_EQ(mem_read32(mem, hold), 0u);
        periph_pad_hold_restore(periph, &snapshot);
        ASSERT_EQ(mem_read32(mem, hold), 1u << 5u);
        ASSERT_EQ(mem_read32(mem, gpio->base + 0x004u), 0u);
        ASSERT_EQ(periph_gpio_pin_level(periph, 26), 1);
        ASSERT_EQ(periph_gpio_output_enabled(periph, 26), 1);
        ASSERT_EQ(periph_gpio_pin_level(periph, 27), 0);
        ASSERT_EQ(periph_gpio_output_enabled(periph, 27), 0);
        mem_write32(mem, hold, 0u);
        ASSERT_EQ(periph_gpio_pin_level(periph, 26), 0);
        ASSERT_EQ(periph_gpio_output_enabled(periph, 26), 0);
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

    mem_write32(mem, s3->io_mux.base +
                     s3->io_mux.gpio_register_offset[4],
                s3->io_mux.input_enable_mask);
    mem_write32(mem, s3->io_mux.base +
                     s3->io_mux.gpio_register_offset[11],
                s3->io_mux.input_enable_mask);

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

    /* GPIO21 has an RTC hold bit but no digital hold bit. */
    mem_write32(mem, gpio + 0x008u, 1u << 21u);
    mem_write32(mem, gpio + 0x024u, 1u << 21u);
    mem_write32(mem, hold, 1u << 21u);
    int before_digital = periph_unhandled_count(periph);
    mem_write32(mem, digital_hold, 1u << 1u);
    ASSERT_EQ(mem_read32(mem, digital_hold), 0u);
    ASSERT_EQ(periph_unhandled_count(periph), before_digital + 1);
    mem_write32(mem, gpio + 0x00Cu, 1u << 21u);
    ASSERT_EQ(periph_gpio_pin_level(periph, 21), 1);
    mem_write32(mem, hold, 0u);
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
        mem_write32(mem, s3->io_mux.base +
                         s3->io_mux.gpio_register_offset[4],
                    s3->io_mux.input_enable_mask);
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

TEST(rtc_cntl_s3_force_hold_freezes_all_rtc_pads_and_preserves_sources)
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

    uint32_t power = rtc->base + rtc->rtc_power_offset;
    uint32_t hold = rtc->base + rtc->rtc_pad_hold_offset;
    uint32_t rtc11 = 1u << (io->data_shift + 11u);
    uint32_t pad11 = io->base + io->pad_base_offset + 4u * 11u;
    uint32_t gpio = s3->gpio.base;
    ASSERT_EQ(mem_read32(mem, power), rtc->rtc_power_reset);
    mem_write32(mem, gpio + 0x008u, 1u << 4u);
    mem_write32(mem, gpio + 0x024u, 1u << 4u);
    mem_write32(mem, io->base + 0x004u, rtc11);
    mem_write32(mem, io->base + 0x010u, rtc11);
    mem_write32(mem, pad11, io->pad_reset[11] | io->pad_mux_mask |
                            (1u << 13u));
    ASSERT_EQ(periph_gpio_pin_level(periph, 4), 1);
    ASSERT_EQ(periph_gpio_pin_level(periph, 11), 1);

    mem_write32(mem, power,
                rtc->rtc_power_reset | rtc->rtc_pad_force_hold_mask);
    periph_pad_hold_t snapshot;
    periph_pad_hold_snapshot(periph, &snapshot);
    /* The active global source holds every RTC pad, but a rebuild snapshot
     * contains only independently held pads that survive ROM boot. */
    ASSERT_EQ64(snapshot.target_gpio.mask, 0u);
    ASSERT_EQ(snapshot.target_rtc_io.mask, 0u);
    mem_write32(mem, gpio + 0x00Cu, 1u << 4u);
    mem_write32(mem, io->base + 0x008u, rtc11);
    ASSERT_EQ(periph_gpio_pin_level(periph, 4), 1);
    ASSERT_EQ(periph_gpio_pin_level(periph, 11), 1);

    /* A per-pad hold remains after releasing the global source. */
    mem_write32(mem, hold, 1u << 4u);
    mem_write32(mem, power, rtc->rtc_power_reset);
    ASSERT_EQ(periph_gpio_pin_level(periph, 4), 1);
    ASSERT_EQ(periph_gpio_pin_level(periph, 11), 0);
    mem_write32(mem, hold, 0u);
    ASSERT_EQ(periph_gpio_pin_level(periph, 4), 0);
    ASSERT_EQ(periph_unhandled_count(periph), 0);
    /* RTC-peripheral sleep policy shares PWC with pad hold and is modeled
     * independently; touching it must not become an unsupported access. */
    mem_write32(mem, power, rtc->rtc_power_reset | (1u << 20u));
    ASSERT_EQ(mem_read32(mem, power),
              rtc->rtc_power_reset | (1u << 20u));
    ASSERT_EQ(periph_unhandled_count(periph), 0);

    periph_destroy(periph);
    mem_destroy(mem);
}

TEST(rtc_cntl_s3_force_hold_clears_at_boot_without_inventing_pad_holds)
{
    const flexe_target_desc_t *s3 =
        flexe_target_by_id(FLEXE_TARGET_ESP32S3);
    const flexe_rtc_cntl_desc_t *rtc = &s3->rtc_cntl;
    xtensa_mem_t *mem = mem_create_for_target(s3);
    esp32_periph_t *periph = mem ? periph_create(mem) : NULL;
    ASSERT_TRUE(periph != NULL);
    if (!periph) {
        mem_destroy(mem);
        return;
    }
    uint32_t power = rtc->base + rtc->rtc_power_offset;
    uint32_t hold = rtc->base + rtc->rtc_pad_hold_offset;
    uint32_t gpio = s3->gpio.base;
    mem_write32(mem, gpio + 0x008u, 1u << 4u);
    mem_write32(mem, gpio + 0x024u, 1u << 4u);
    mem_write32(mem, power,
                rtc->rtc_power_reset | rtc->rtc_pad_force_hold_mask);
    mem_write32(mem, gpio + 0x00Cu, 1u << 4u);
    periph_pad_hold_t snapshot;
    periph_pad_hold_snapshot(periph, &snapshot);
    ASSERT_EQ(snapshot.target_rtc_io_hold, 0u);
    ASSERT_EQ64(snapshot.target_gpio.mask, 0u);
    ASSERT_EQ(snapshot.target_rtc_io.mask, 0u);
    periph_destroy(periph);

    periph = periph_create(mem);
    ASSERT_TRUE(periph != NULL);
    if (periph) {
        ASSERT_EQ(mem_read32(mem, power), rtc->rtc_power_reset);
        periph_pad_hold_restore(periph, &snapshot);
        ASSERT_EQ(mem_read32(mem, power), rtc->rtc_power_reset);
        ASSERT_EQ(mem_read32(mem, hold), 0u);
        ASSERT_EQ(periph_gpio_pin_level(periph, 4), 0);
        periph_pad_hold_snapshot(periph, &snapshot);
        ASSERT_EQ64(snapshot.target_gpio.mask, 0u);
        ASSERT_EQ(periph_unhandled_count(periph), 0);
        periph_destroy(periph);
    }
    mem_destroy(mem);
}

TEST(rtc_cntl_s3_force_hold_rebuild_retains_only_individual_source)
{
    const flexe_target_desc_t *s3 =
        flexe_target_by_id(FLEXE_TARGET_ESP32S3);
    const flexe_rtc_cntl_desc_t *rtc = &s3->rtc_cntl;
    xtensa_mem_t *mem = mem_create_for_target(s3);
    esp32_periph_t *periph = mem ? periph_create(mem) : NULL;
    ASSERT_TRUE(periph != NULL);
    if (!periph) {
        mem_destroy(mem);
        return;
    }
    uint32_t power = rtc->base + rtc->rtc_power_offset;
    uint32_t hold = rtc->base + rtc->rtc_pad_hold_offset;
    uint32_t gpio = s3->gpio.base;
    uint32_t pin4 = 1u << 4u;
    mem_write32(mem, gpio + 0x008u, pin4);
    mem_write32(mem, gpio + 0x024u, pin4);
    mem_write32(mem, power,
                rtc->rtc_power_reset | rtc->rtc_pad_force_hold_mask);
    mem_write32(mem, hold, pin4);
    mem_write32(mem, gpio + 0x00Cu, pin4);
    ASSERT_EQ(periph_gpio_pin_level(periph, 4), 1);

    periph_pad_hold_t snapshot;
    periph_pad_hold_snapshot(periph, &snapshot);
    ASSERT_EQ(snapshot.target_rtc_io_hold, pin4);
    ASSERT_EQ64(snapshot.target_gpio.mask, pin4);
    ASSERT_EQ(snapshot.target_rtc_io.mask, pin4);
    periph_destroy(periph);

    periph = periph_create(mem);
    ASSERT_TRUE(periph != NULL);
    if (periph) {
        periph_pad_hold_restore(periph, &snapshot);
        ASSERT_EQ(mem_read32(mem, power), rtc->rtc_power_reset);
        ASSERT_EQ(mem_read32(mem, hold), pin4);
        ASSERT_EQ(periph_gpio_pin_level(periph, 4), 1);
        periph_pad_hold_snapshot(periph, &snapshot);
        ASSERT_EQ64(snapshot.target_gpio.mask, pin4);
        mem_write32(mem, hold, 0u);
        ASSERT_EQ(periph_gpio_pin_level(periph, 4), 0);
        ASSERT_EQ(periph_unhandled_count(periph), 0);
        periph_destroy(periph);
    }
    mem_destroy(mem);
}

TEST(rtc_cntl_s3_digital_force_hold_freezes_digital_pads_only)
{
    const flexe_target_desc_t *s3 =
        flexe_target_by_id(FLEXE_TARGET_ESP32S3);
    const flexe_rtc_cntl_desc_t *rtc = &s3->rtc_cntl;
    xtensa_mem_t *mem = mem_create_for_target(s3);
    esp32_periph_t *periph = mem ? periph_create(mem) : NULL;
    ASSERT_TRUE(periph != NULL);
    if (!periph) {
        mem_destroy(mem);
        return;
    }
    uint32_t iso = rtc->base + rtc->digital_iso_offset;
    uint32_t digital_hold = rtc->base + rtc->digital_pad_hold_offset;
    uint32_t gpio = s3->gpio.base;
    uint32_t low = (1u << 21u) | (1u << 26u);
    uint32_t high = 1u << (48u - 32u);
    ASSERT_EQ(mem_read32(mem, iso), rtc->digital_iso_reset);
    mem_write32(mem, gpio + 0x008u, low);
    mem_write32(mem, gpio + 0x024u, low);
    mem_write32(mem, gpio + 0x014u, high);
    mem_write32(mem, gpio + 0x030u, high);
    mem_write32(mem, iso,
                (rtc->digital_iso_reset &
                 ~rtc->digital_pad_force_unhold_mask) |
                rtc->digital_pad_force_hold_mask);
    mem_write32(mem, gpio + 0x00Cu, low);
    mem_write32(mem, gpio + 0x018u, high);
    ASSERT_EQ(periph_gpio_pin_level(periph, 21), 0);
    ASSERT_EQ(periph_gpio_pin_level(periph, 26), 1);
    ASSERT_EQ(periph_gpio_pin_level(periph, 48), 1);
    ASSERT_EQ(periph_unhandled_count(periph), 0);

    /* Individual hold remains after releasing the global digital source. */
    mem_write32(mem, digital_hold, 1u << 5u);
    mem_write32(mem, iso, rtc->digital_iso_reset);
    ASSERT_EQ(periph_gpio_pin_level(periph, 26), 1);
    ASSERT_EQ(periph_gpio_pin_level(periph, 48), 0);
    mem_write32(mem, digital_hold, 0u);
    ASSERT_EQ(periph_gpio_pin_level(periph, 26), 0);

    /* Pad isolation and the autohold-clear command are real modeled control
     * state. Their downstream electrical effects remain independently
     * attachable through the normalized listener. */
    mem_write32(mem, iso, rtc->digital_iso_reset | (1u << 13u));
    ASSERT_EQ(mem_read32(mem, iso), rtc->digital_iso_reset | (1u << 13u));
    ASSERT_EQ(periph_unhandled_count(periph), 0);
    mem_write32(mem, iso, rtc->digital_iso_reset | (1u << 10u));
    ASSERT_EQ(mem_read32(mem, iso), rtc->digital_iso_reset);
    ASSERT_EQ(periph_unhandled_count(periph), 0);
    periph_destroy(periph);
    mem_destroy(mem);
}

TEST(rtc_cntl_s3_digital_force_hold_clears_at_boot_but_per_pad_survives)
{
    const flexe_target_desc_t *s3 =
        flexe_target_by_id(FLEXE_TARGET_ESP32S3);
    const flexe_rtc_cntl_desc_t *rtc = &s3->rtc_cntl;
    xtensa_mem_t *mem = mem_create_for_target(s3);
    esp32_periph_t *periph = mem ? periph_create(mem) : NULL;
    ASSERT_TRUE(periph != NULL);
    if (!periph) {
        mem_destroy(mem);
        return;
    }
    uint32_t iso = rtc->base + rtc->digital_iso_offset;
    uint32_t digital_hold = rtc->base + rtc->digital_pad_hold_offset;
    uint32_t gpio = s3->gpio.base;
    uint32_t high = 1u << (48u - 32u);
    mem_write32(mem, gpio + 0x008u, 1u << 26u);
    mem_write32(mem, gpio + 0x024u, 1u << 26u);
    mem_write32(mem, gpio + 0x014u, high);
    mem_write32(mem, gpio + 0x030u, high);
    mem_write32(mem, iso,
                (rtc->digital_iso_reset &
                 ~rtc->digital_pad_force_unhold_mask) |
                rtc->digital_pad_force_hold_mask);
    periph_pad_hold_t snapshot;
    periph_pad_hold_snapshot(periph, &snapshot);
    ASSERT_EQ64(snapshot.target_gpio.mask, 0u);
    mem_write32(mem, digital_hold, 1u << 5u);
    mem_write32(mem, gpio + 0x00Cu, 1u << 26u);
    mem_write32(mem, gpio + 0x018u, high);
    periph_pad_hold_snapshot(periph, &snapshot);
    ASSERT_EQ64(snapshot.target_gpio.mask, UINT64_C(1) << 26u);
    ASSERT_EQ(snapshot.target_rtc_hold, 1u << 5u);
    periph_destroy(periph);

    periph = periph_create(mem);
    ASSERT_TRUE(periph != NULL);
    if (periph) {
        periph_pad_hold_restore(periph, &snapshot);
        ASSERT_EQ(mem_read32(mem, iso), rtc->digital_iso_reset);
        ASSERT_EQ(mem_read32(mem, digital_hold), 1u << 5u);
        ASSERT_EQ(periph_gpio_pin_level(periph, 26), 1);
        ASSERT_EQ(periph_gpio_pin_level(periph, 48), 0);
        mem_write32(mem, digital_hold, 0u);
        ASSERT_EQ(periph_gpio_pin_level(periph, 26), 0);
        ASSERT_EQ(periph_unhandled_count(periph), 0);
        periph_destroy(periph);
    }
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

TEST(rtc_cntl_watchdog_pause_in_sleep_preserves_stage)
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
    uint32_t low = desc->base + desc->sleep_timer_low_offset;
    uint32_t high = desc->base + desc->sleep_timer_high_offset;
    uint32_t wakeup = desc->base + desc->wakeup_state_offset;
    uint32_t state = desc->base + desc->sleep_state_offset;
    uint32_t action_fields = 0u;
    for (unsigned stage = 0u; stage < FLEXE_TARGET_RTC_WDT_STAGE_MAX;
         stage++)
        action_fields |= (uint32_t)desc->wdt_stage_action_mask <<
                         desc->wdt_stage_action_shift[stage];
    uint32_t base_config = desc->wdt_config_reset[0] &
        ~(desc->wdt_enable_mask | desc->wdt_flashboot_enable_mask |
          action_fields);
    mem_write32(mem, config0, base_config);
    mem_write32(mem, stage0_hold, 2u);
    mem_write32(mem, config0,
                base_config | desc->wdt_enable_mask |
                desc->wdt_pause_in_sleep_mask |
                ((uint32_t)FLEXE_RTC_CNTL_WDT_RESET_SYSTEM <<
                 desc->wdt_stage_action_shift[0]));
    uint32_t running_deadline = cpu.next_timer_event;
    ASSERT_TRUE(running_deadline != UINT32_MAX);

    uint64_t alarm = rtc_capture(mem, desc) + 1360u;
    mem_write32(mem, low, (uint32_t)alarm);
    mem_write32(mem, high, (uint32_t)(alarm >> 32u));
    mem_write32(mem, high,
                (uint32_t)(alarm >> 32u) |
                desc->sleep_alarm_enable_mask);
    mem_write32(mem, wakeup,
                desc->timer_wakeup_mask << desc->wakeup_enable_shift);
    mem_write32(mem, state, desc->sleep_enable_mask);
    bool deep = true;
    uint64_t timeout_us = 0u;
    uint32_t cause = UINT32_MAX;
    ASSERT_TRUE(periph_take_sleep_request(periph, &deep, &timeout_us,
                                           &cause));
    ASSERT_FALSE(deep);

    /* Cross the watchdog's original deadline while sleep is active. */
    cpu.ccount = running_deadline + 1u;
    cpu.periph_event(&cpu);
    ASSERT_FALSE(periph_take_reset_request(periph));

    /* Waking resumes the unconsumed stage interval. */
    periph_finish_wake(periph, desc->timer_wakeup_mask);
    uint32_t resumed_deadline = cpu.next_timer_event;
    ASSERT_TRUE(resumed_deadline != UINT32_MAX);
    ASSERT_TRUE((int32_t)(resumed_deadline - cpu.ccount) > 0);
    cpu.ccount = resumed_deadline;
    cpu.periph_event(&cpu);
    ASSERT_TRUE(periph_take_reset_request(periph));
    ASSERT_EQ(periph_unhandled_count(periph), 0);

    periph_destroy(periph);
    mem_destroy(mem);
}

TEST(rtc_cntl_s3_timer_sleep_wakes_and_reports_cause)
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

    uint32_t low = desc->base + desc->sleep_timer_low_offset;
    uint32_t high = desc->base + desc->sleep_timer_high_offset;
    uint32_t state = desc->base + desc->sleep_state_offset;
    uint32_t wakeup = desc->base + desc->wakeup_state_offset;
    uint32_t cause_reg = desc->base + desc->wakeup_cause_offset;
    uint32_t digital = desc->base + desc->digital_power_offset;
    uint32_t raw = desc->base + desc->interrupt_raw_offset;
    uint32_t clear = desc->base + desc->interrupt_clear_offset;
    ASSERT_EQ(mem_read32(mem, digital), desc->digital_power_reset);
    ASSERT_EQ(mem_read32(mem, cause_reg), 0u);
    ASSERT_EQ(mem_read32(mem, wakeup),
              desc->wakeup_enable_reset << desc->wakeup_enable_shift);

    /* The official S3 HAL writes an absolute 48-bit target, then a
     * write-only alarm-enable command in the high word. */
    mem_write32(mem, low, 13600u); /* 100 ms at 136 kHz */
    mem_write32(mem, high, 0u);
    mem_write32(mem, high, desc->sleep_alarm_enable_mask);
    ASSERT_EQ(mem_read32(mem, low), 13600u);
    ASSERT_EQ(mem_read32(mem, high), 0u);
    mem_write32(mem, wakeup,
                desc->timer_wakeup_mask << desc->wakeup_enable_shift);
    mem_write32(mem, state, desc->sleep_enable_mask);

    bool deep = true;
    uint64_t timeout_us = 0u;
    uint32_t cause = UINT32_MAX;
    ASSERT_TRUE(periph_take_sleep_request(periph, &deep, &timeout_us,
                                           &cause));
    ASSERT_FALSE(deep);
    ASSERT_EQ64(timeout_us, 100000u);
    ASSERT_EQ(cause, 0u);
    ASSERT_FALSE(periph_take_sleep_request(periph, &deep, &timeout_us,
                                            &cause));

    cpu.ccount = 16000000u;
    cpu.periph_event(&cpu);
    ASSERT_EQ(mem_read32(mem, raw) & desc->sleep_alarm_interrupt_mask,
              desc->sleep_alarm_interrupt_mask);
    periph_finish_wake(periph, desc->timer_wakeup_mask);
    ASSERT_EQ(mem_read32(mem, state) & desc->sleep_enable_mask, 0u);
    ASSERT_EQ(mem_read32(mem, state) & desc->sleep_wakeup_mask,
              desc->sleep_wakeup_mask);
    ASSERT_EQ(mem_read32(mem, raw) & desc->sleep_wakeup_interrupt_mask,
              desc->sleep_wakeup_interrupt_mask);
    ASSERT_EQ(mem_read32(mem, cause_reg), desc->timer_wakeup_mask);
    mem_write32(mem, clear, desc->sleep_alarm_interrupt_mask |
                            desc->sleep_wakeup_interrupt_mask);

    /* A second alarm can enter deep sleep; the reset cause and wake source
     * use separate S3 registers, unlike the classic ESP32 layout. */
    uint64_t now = rtc_capture(mem, desc);
    mem_write32(mem, low, (uint32_t)(now + 68u));
    mem_write32(mem, high,
                (uint32_t)((now + 68u) >> 32u) |
                desc->sleep_alarm_enable_mask);
    mem_write32(mem, digital, desc->digital_power_reset |
                               desc->digital_wrap_power_down_mask);
    mem_write32(mem, state, desc->sleep_enable_mask);
    ASSERT_TRUE(periph_take_sleep_request(periph, &deep, &timeout_us,
                                           &cause));
    ASSERT_TRUE(deep);
    ASSERT_EQ64(timeout_us, 500u);
    periph_set_wake_state(periph, desc->timer_wakeup_mask, 5u);
    ASSERT_EQ(mem_read32(mem, cause_reg), desc->timer_wakeup_mask);
    ASSERT_EQ(mem_read32(mem, desc->base + desc->reset_state_offset) &
              0xFFFu, 5u | (5u << 6u));
    periph_set_wake_state(periph, 0u, 12u);
    ASSERT_EQ(mem_read32(mem, cause_reg), 0u);
    ASSERT_EQ(mem_read32(mem, desc->base + desc->reset_state_offset) &
              0xFFFu, 12u | (12u << 6u));
    ASSERT_EQ(periph_unhandled_count(periph), 0);

    periph_destroy(periph);
    mem_destroy(mem);
}

TEST(rtc_cntl_s3_sleep_rejects_unmodeled_wake_sources)
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
    uint32_t wakeup = desc->base + desc->wakeup_state_offset;
    uint32_t state = desc->base + desc->sleep_state_offset;
    uint32_t raw = desc->base + desc->interrupt_raw_offset;

    mem_write32(mem, wakeup,
                desc->timer_wakeup_mask << desc->wakeup_enable_shift);
    mem_write32(mem, state, desc->sleep_enable_mask);
    bool deep = false;
    uint64_t timeout_us = 0u;
    uint32_t cause = 0u;
    ASSERT_FALSE(periph_take_sleep_request(periph, &deep, &timeout_us,
                                            &cause));
    ASSERT_EQ(periph_unhandled_count(periph), 1);

    mem_write32(mem, wakeup, (1u << 5u) << desc->wakeup_enable_shift);
    mem_write32(mem, state, desc->sleep_enable_mask);
    ASSERT_FALSE(periph_take_sleep_request(periph, &deep, &timeout_us,
                                            &cause));
    ASSERT_EQ(mem_read32(mem, raw) &
              desc->sleep_wakeup_interrupt_mask, 0u);
    ASSERT_EQ(periph_unhandled_count(periph), 3);
    /* One transaction touching both an unmodeled wake source and another
     * unmodeled STATE0 field contributes one audit event, not two. */
    mem_write32(mem, state, desc->sleep_enable_mask | (1u << 22u));
    ASSERT_EQ(mem_read32(mem, state) & (1u << 22u), 1u << 22u);
    ASSERT_EQ(periph_unhandled_count(periph), 4);
    periph_destroy(periph);
    mem_destroy(mem);
}

TEST(rtc_cntl_s3_ext0_ext1_wake_samples_gpio_and_latches_status)
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

    uint32_t state = rtc->base + rtc->sleep_state_offset;
    uint32_t wakeup = rtc->base + rtc->wakeup_state_offset;
    uint32_t cause_reg = rtc->base + rtc->wakeup_cause_offset;
    uint32_t extconf = rtc->base + rtc->ext_wakeup_config_offset;
    uint32_t ext1sel = rtc->base + rtc->ext1_select_offset;
    uint32_t ext1status = rtc->base + rtc->ext1_status_offset;
    uint32_t ext0sel = io->base + io->ext0_select_offset;
    uint32_t pad4 = io->base + io->pad_base_offset + 4u * 4u;
    bool deep = true;
    uint64_t timeout_us = 0u;
    uint32_t cause = UINT32_MAX;

    /* EXT0 requires an RTC-owned input. A raw host level on a digitally
     * owned pad cannot wake it, even if that level matches the trigger. */
    mem_write32(mem, ext0sel, 4u << io->ext0_select_shift);
    mem_write32(mem, extconf, rtc->ext0_wakeup_level_mask);
    mem_write32(mem, wakeup,
                rtc->ext0_wakeup_mask << rtc->wakeup_enable_shift);
    periph_gpio_set_input(periph, 4, 1);
    mem_write32(mem, state, rtc->sleep_enable_mask);
    ASSERT_TRUE(periph_take_sleep_request(periph, &deep, &timeout_us,
                                          &cause));
    ASSERT_FALSE(deep);
    ASSERT_EQ64(timeout_us, PERIPH_SLEEP_FOREVER);
    ASSERT_EQ(cause, 0u);
    ASSERT_TRUE(periph_sleep_has_gpio_wake(periph));
    ASSERT_EQ(periph_sleep_poll_wake(periph), 0u);
    mem_write32(mem, pad4, io->pad_reset[4] | io->pad_mux_mask |
                            (1u << 13u));
    ASSERT_EQ(periph_sleep_poll_wake(periph), rtc->ext0_wakeup_mask);
    periph_finish_wake(periph, rtc->ext0_wakeup_mask);
    ASSERT_EQ(mem_read32(mem, cause_reg), rtc->ext0_wakeup_mask);
    ASSERT_EQ(periph_sleep_poll_wake(periph), 0u);

    /* EXT1 ANY_HIGH reports the triggering RTC pad(s), not every selected
     * pad. The status-clear command is write-only and selection remains. */
    periph_gpio_set_input(periph, 4, 0);
    periph_gpio_set_input(periph, 11, 0);
    mem_write32(mem, ext1sel, (1u << 4u) | (1u << 11u) |
                              rtc->ext1_status_clear_mask);
    mem_write32(mem, extconf, rtc->ext1_wakeup_level_mask);
    mem_write32(mem, wakeup,
                rtc->ext1_wakeup_mask << rtc->wakeup_enable_shift);
    mem_write32(mem, state, rtc->sleep_enable_mask);
    ASSERT_TRUE(periph_take_sleep_request(periph, &deep, &timeout_us,
                                          &cause));
    ASSERT_EQ(cause, 0u);
    periph_gpio_set_input(periph, 11, 1);
    ASSERT_EQ(periph_sleep_poll_wake(periph), rtc->ext1_wakeup_mask);
    ASSERT_EQ(mem_read32(mem, ext1status), 1u << 11u);
    periph_finish_wake(periph, rtc->ext1_wakeup_mask);
    ASSERT_EQ(mem_read32(mem, cause_reg), rtc->ext1_wakeup_mask);
    ASSERT_EQ(periph_sleep_poll_wake(periph), 0u);
    mem_write32(mem, ext1sel, (1u << 4u) | (1u << 11u) |
                              rtc->ext1_status_clear_mask);
    ASSERT_EQ(mem_read32(mem, ext1status), 0u);
    ASSERT_EQ(mem_read32(mem, ext1sel), (1u << 4u) | (1u << 11u));

    /* EXT1 ALL_LOW requires every selected pad low and reports both. */
    mem_write32(mem, extconf, 0u);
    mem_write32(mem, state, rtc->sleep_enable_mask);
    ASSERT_TRUE(periph_take_sleep_request(periph, &deep, &timeout_us,
                                          &cause));
    ASSERT_EQ(cause, 0u);
    periph_gpio_set_input(periph, 11, 0);
    ASSERT_EQ(periph_sleep_poll_wake(periph), rtc->ext1_wakeup_mask);
    ASSERT_EQ(mem_read32(mem, ext1status), (1u << 4u) | (1u << 11u));
    ASSERT_EQ(periph_unhandled_count(periph), 0);

    periph_destroy(periph);
    mem_destroy(mem);
}

TEST(rtc_cntl_s3_gpio_wake_rejects_invalid_selection_and_retains_status)
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
    uint32_t ext0sel = io->base + io->ext0_select_offset;
    uint32_t ext1sel = rtc->base + rtc->ext1_select_offset;
    uint32_t ext1status = rtc->base + rtc->ext1_status_offset;
    uint32_t wakeup = rtc->base + rtc->wakeup_state_offset;
    uint32_t state = rtc->base + rtc->sleep_state_offset;
    bool deep = false;
    uint64_t timeout_us = 0u;
    uint32_t cause = 0u;

    mem_write32(mem, ext0sel, 22u << io->ext0_select_shift);
    ASSERT_EQ(mem_read32(mem, ext0sel), 22u << io->ext0_select_shift);
    ASSERT_EQ(periph_unhandled_count(periph), 1);
    mem_write32(mem, wakeup,
                rtc->ext0_wakeup_mask << rtc->wakeup_enable_shift);
    mem_write32(mem, state, rtc->sleep_enable_mask);
    ASSERT_TRUE(periph_take_sleep_request(periph, &deep, &timeout_us,
                                          &cause));
    periph_gpio_set_input(periph, 21, 1);
    ASSERT_EQ(periph_sleep_poll_wake(periph), 0u);
    periph_finish_wake(periph, 0u);

    mem_write32(mem, ext1sel, 0u);
    mem_write32(mem, wakeup,
                rtc->ext1_wakeup_mask << rtc->wakeup_enable_shift);
    mem_write32(mem, state, rtc->sleep_enable_mask);
    ASSERT_FALSE(periph_take_sleep_request(periph, &deep, &timeout_us,
                                           &cause));
    ASSERT_EQ(periph_unhandled_count(periph), 2);

    mem_write32(mem, ext1sel, 1u << 4u);
    mem_write32(mem, wakeup,
                (rtc->ext1_wakeup_mask | rtc->timer_wakeup_mask) <<
                rtc->wakeup_enable_shift);
    mem_write32(mem, state, rtc->sleep_enable_mask);
    ASSERT_FALSE(periph_take_sleep_request(periph, &deep, &timeout_us,
                                           &cause));
    ASSERT_EQ(periph_unhandled_count(periph), 3); /* timer unarmed */
    mem_write32(mem, ext1status, UINT32_MAX);
    ASSERT_EQ(mem_read32(mem, ext1status), 0u);
    ASSERT_EQ(periph_unhandled_count(periph), 3);

    periph_destroy(periph);
    mem_destroy(mem);
}

TEST(rtc_cntl_s3_gpio_wake_preempts_armed_timer)
{
    const flexe_target_desc_t *s3 =
        flexe_target_by_id(FLEXE_TARGET_ESP32S3);
    const flexe_rtc_cntl_desc_t *rtc = &s3->rtc_cntl;
    xtensa_mem_t *mem = mem_create_for_target(s3);
    esp32_periph_t *periph = mem ? periph_create(mem) : NULL;
    ASSERT_TRUE(periph != NULL);
    if (!periph) {
        mem_destroy(mem);
        return;
    }
    xtensa_cpu_t cpu;
    xtensa_cpu_init_for_target(&cpu, s3);
    cpu.mem = mem;
    periph_attach_cpus(periph, &cpu, NULL);
    mem_write32(mem, rtc->base + rtc->ext1_select_offset, 1u << 4u);
    mem_write32(mem, rtc->base + rtc->ext_wakeup_config_offset,
                rtc->ext1_wakeup_level_mask);
    mem_write32(mem, rtc->base + rtc->sleep_timer_low_offset, 13600u);
    mem_write32(mem, rtc->base + rtc->sleep_timer_high_offset,
                rtc->sleep_alarm_enable_mask);
    mem_write32(mem, rtc->base + rtc->wakeup_state_offset,
                (rtc->ext1_wakeup_mask | rtc->timer_wakeup_mask) <<
                rtc->wakeup_enable_shift);
    mem_write32(mem, rtc->base + rtc->sleep_state_offset,
                rtc->sleep_enable_mask);
    bool deep = false;
    uint64_t timeout_us = 0u;
    uint32_t cause = 0u;
    ASSERT_TRUE(periph_take_sleep_request(periph, &deep, &timeout_us,
                                          &cause));
    ASSERT_EQ64(timeout_us, 100000u);
    ASSERT_EQ(cause, 0u);
    periph_gpio_set_input(periph, 4, 1);
    ASSERT_EQ(periph_sleep_poll_wake(periph), rtc->ext1_wakeup_mask);
    periph_finish_wake(periph, rtc->ext1_wakeup_mask);
    ASSERT_EQ(mem_read32(mem, rtc->base + rtc->wakeup_cause_offset),
              rtc->ext1_wakeup_mask);
    cpu.ccount = 16000000u;
    cpu.periph_event(&cpu);
    ASSERT_EQ(mem_read32(mem, rtc->base + rtc->interrupt_raw_offset) &
              rtc->sleep_alarm_interrupt_mask, 0u);
    ASSERT_EQ(periph_unhandled_count(periph), 0);
    periph_destroy(periph);
    mem_destroy(mem);
}

TEST(rtc_cntl_s3_brownout_config_reads_back_without_analog_detection)
{
    const flexe_target_desc_t *s3 =
        flexe_target_by_id(FLEXE_TARGET_ESP32S3);
    const flexe_rtc_cntl_desc_t *desc = &s3->rtc_cntl;
    xtensa_mem_t *mem = mem_create_for_target(s3);
    esp32_periph_t *periph = mem ? periph_create(mem) : NULL;
    ASSERT_TRUE(periph != NULL);
    if (!periph) {
        mem_destroy(mem);
        return;
    }
    uint32_t addr = desc->base + desc->brownout_offset;
    ASSERT_EQ(mem_read32(mem, addr), desc->brownout_reset);
    ASSERT_EQ(mem_read32(mem, addr) & desc->brownout_detect_mask, 0u);

    uint32_t config = (1u << 28u) | (1u << 27u) |
                      (17u << 16u) | (1u << 14u) | (3u << 4u);
    mem_write32(mem, addr, config | desc->brownout_count_clear_mask);
    ASSERT_EQ(mem_read32(mem, addr), config);
    ASSERT_EQ(periph_unhandled_count(periph), 0);
    mem_write32(mem, addr, config | desc->brownout_detect_mask);
    ASSERT_EQ(mem_read32(mem, addr), config);
    ASSERT_EQ(periph_unhandled_count(periph), 1);

    periph_destroy(periph);
    periph = periph_create(mem);
    ASSERT_TRUE(periph != NULL);
    if (periph) {
        ASSERT_EQ(mem_read32(mem, addr), desc->brownout_reset);
        periph_destroy(periph);
    }
    mem_destroy(mem);
}

TEST(rtc_cntl_s3_counter_and_store_survive_controller_rebuild)
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
    xtensa_cpu_t cpu;
    xtensa_cpu_init_for_target(&cpu, s3);
    cpu.mem = mem;
    periph_attach_cpus(periph, &cpu, NULL);
    cpu.ccount = 160001u;
    ASSERT_EQ64(rtc_capture(mem, desc), 136u);
    mem_write32(mem, desc->base + desc->store_offset[0], 0xA5A55A5Au);
    mem_write32(mem, desc->base + desc->store_offset[1], 0x12345678u);
    mem_write32(mem, desc->base + desc->sleep_timer_low_offset, 99u);
    mem_write32(mem, desc->base + desc->ext1_select_offset, 1u << 4u);
    mem_write32(mem, desc->base + desc->ext_wakeup_config_offset,
                desc->ext1_wakeup_level_mask);
    mem_write32(mem, desc->base + desc->wakeup_state_offset,
                desc->ext1_wakeup_mask << desc->wakeup_enable_shift);
    mem_write32(mem, desc->base + desc->sleep_state_offset,
                desc->sleep_enable_mask);
    periph_gpio_set_input(periph, 4, 1);
    ASSERT_EQ(periph_sleep_poll_wake(periph), desc->ext1_wakeup_mask);

    flexe_rtc_cntl_retained_t retained;
    periph_rtc_retained_snapshot(periph, &retained);
    periph_destroy(periph);
    periph = periph_create(mem);
    ASSERT_TRUE(periph != NULL);
    if (periph) {
        xtensa_cpu_t restarted;
        xtensa_cpu_init_for_target(&restarted, s3);
        restarted.mem = mem;
        periph_attach_cpus(periph, &restarted, NULL);
        periph_rtc_retained_restore(periph, &retained);
        ASSERT_EQ64(rtc_capture(mem, desc), 136u);
        ASSERT_EQ(mem_read32(mem, desc->base + desc->store_offset[0]),
                  0xA5A55A5Au);
        ASSERT_EQ(mem_read32(mem, desc->base + desc->store_offset[1]),
                  0x12345678u);
        ASSERT_EQ(mem_read32(mem, desc->base + desc->sleep_timer_low_offset),
                  0u);
        ASSERT_EQ(mem_read32(mem, desc->base + desc->ext1_status_offset),
                  1u << 4u);
        /* One extra pre-reset CPU cycle contributes a fractional RTC tick.
         * The first 1,176 cycles after reboot cross the tick boundary only
         * if that phase was retained with the integer counter. */
        restarted.ccount = 1176u;
        ASSERT_EQ64(rtc_capture(mem, desc), 137u);
        restarted.ccount = 161176u;
        ASSERT_EQ64(rtc_capture(mem, desc), 273u);
    }
    periph_destroy(periph);
    mem_destroy(mem);
}

void run_rtc_cntl_tests(void)
{
    TEST_SUITE("Target RTC controller");
    RUN_TEST(rtc_cntl_software_stall_uses_both_fields_and_preserves_reset_commands);
    RUN_TEST(rtc_cntl_s3_routes_app_cpu_reset_without_system_reset);
    RUN_TEST(rtc_cntl_s3_control_fabric_resolves_force_pairs_and_notifies);
    RUN_TEST(rtc_cntl_s3_sequence_timers_read_back_and_enable_cpu_stall);
    RUN_TEST(rtc_cntl_storage_resets_persists_and_delegates);
    RUN_TEST(rtc_cntl_sar_i2c_power_gates_analog_slave);
    RUN_TEST(rtc_cntl_application_handoff_uses_target_clocks);
    RUN_TEST(rtc_cntl_watchdog_reports_unmodeled_configuration);
    RUN_TEST(rtc_cntl_counter_tracks_shared_time_and_frequency);
    RUN_TEST(rtc_cntl_switches_slow_clock_at_an_exact_boundary);
    RUN_TEST(rtc_cntl_s3_fast_clock_and_date_register_follow_descriptor);
    RUN_TEST(rtc_cntl_s3_regulator_force_pairs_are_functional_and_trim_is_diagnostic);
    RUN_TEST(rtc_cntl_rejects_invalid_clock_and_control_geometry);
    RUN_TEST(rtc_cntl_rejects_invalid_regulator_and_rtc_power_geometry);
    RUN_TEST(rtc_cntl_s3_configuration_bank_masks_retains_and_audits);
    RUN_TEST(rtc_cntl_rejects_invalid_configuration_bank_geometry);
    RUN_TEST(rtc_cntl_digital_domains_resolve_force_and_sleep_policy);
    RUN_TEST(rtc_cntl_rtc_domains_resolve_force_follow_cpu_and_sleep_policy);
    RUN_TEST(rtc_cntl_digital_pad_hold_freezes_physical_gpio_not_latches);
    RUN_TEST(rtc_cntl_digital_pad_hold_survives_rebuild_without_unheld_gpio);
    RUN_TEST(rtc_cntl_rtc_pad_hold_freezes_mux_input_and_output);
    RUN_TEST(rtc_cntl_rtc_pad_hold_survives_rebuild_with_frozen_mux);
    RUN_TEST(rtc_cntl_s3_force_hold_freezes_all_rtc_pads_and_preserves_sources);
    RUN_TEST(rtc_cntl_s3_force_hold_clears_at_boot_without_inventing_pad_holds);
    RUN_TEST(rtc_cntl_s3_force_hold_rebuild_retains_only_individual_source);
    RUN_TEST(rtc_cntl_s3_digital_force_hold_freezes_digital_pads_only);
    RUN_TEST(rtc_cntl_s3_digital_force_hold_clears_at_boot_but_per_pad_survives);
    RUN_TEST(rtc_cntl_interrupt_bank_latches_masks_clears_and_publishes_level);
    RUN_TEST(rtc_cntl_interrupt_routes_through_target_matrix);
    RUN_TEST(rtc_cntl_watchdog_schedules_feed_interrupt_and_reset);
    RUN_TEST(rtc_cntl_watchdog_pause_in_sleep_preserves_stage);
    RUN_TEST(rtc_cntl_s3_timer_sleep_wakes_and_reports_cause);
    RUN_TEST(rtc_cntl_s3_sleep_rejects_unmodeled_wake_sources);
    RUN_TEST(rtc_cntl_s3_ext0_ext1_wake_samples_gpio_and_latches_status);
    RUN_TEST(rtc_cntl_s3_gpio_wake_rejects_invalid_selection_and_retains_status);
    RUN_TEST(rtc_cntl_s3_gpio_wake_preempts_armed_timer);
    RUN_TEST(rtc_cntl_s3_brownout_config_reads_back_without_analog_detection);
    RUN_TEST(rtc_cntl_s3_counter_and_store_survive_controller_rebuild);
}
