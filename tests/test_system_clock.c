/* Target-described CPU/system-clock register tests. */
#include "test_helpers.h"
#include "peripherals.h"
#include "system_clock.h"
#include "target.h"

#define SC_SYSTEM_BASE          0x600C0000u
#define SC_CPU_PER_CONF_OFF     0x010u
#define SC_MEM_PD_MASK_OFF      0x014u
#define SC_PERIP_CLK_EN0_OFF    0x018u
#define SC_PERIP_CLK_EN1_OFF    0x01Cu
#define SC_PERIP_RST_EN0_OFF    0x020u
#define SC_PERIP_RST_EN1_OFF    0x024u
#define SC_BT_LPCK_DIV_INT_OFF  0x028u
#define SC_BT_LPCK_DIV_FRAC_OFF 0x02Cu
#define SC_SYSCLK_CONF_OFF      0x060u
#define SC_CPU_WAIT_FORCE_ON    (1u << 3)
#define SC_PLL_480M_SELECT      (1u << 2)
#define SC_CPU_PERIOD_160M      1u
#define SC_SOC_CLOCK_PLL        (1u << 10)
#define SC_SYSTIMER_GATE        (1u << 29)
#define SC_TIMG0_GATE           (1u << 13)
#define SC_TIMG1_GATE           (1u << 15)

#define SC_SYSTIMER_BASE        0x60023000u
#define SC_SYSTIMER_CONF        0x000u
#define SC_SYSTIMER_UNIT0_OP    0x004u
#define SC_SYSTIMER_TARGET0_HI  0x01Cu
#define SC_SYSTIMER_TARGET0_LO  0x020u
#define SC_SYSTIMER_VALUE_LO    0x044u
#define SC_SYSTIMER_COMP0_LOAD  0x050u
#define SC_SYSTIMER_INT_ENA     0x064u
#define SC_SYSTIMER_INT_RAW     0x068u
#define SC_SYSTIMER_UPDATE      (1u << 30)
#define SC_SYSTIMER_TARGET0_EN  (1u << 24)

#define SC_TIMG0_BASE           0x6001F000u
#define SC_TIMG1_BASE           0x60020000u
#define SC_TIMG_CONFIG          0x000u
#define SC_TIMG_VALUE_LO        0x004u
#define SC_TIMG_UPDATE          0x00Cu
#define SC_TIMG_LOAD_LO         0x018u
#define SC_TIMG_LOAD            0x020u
#define SC_TIMG_ENABLE          (1u << 31)
#define SC_TIMG_INCREASE        (1u << 30)

#define SC_UART0_BASE           0x60000000u
#define SC_UART1_BASE           0x60010000u
#define SC_UART2_BASE           0x6002E000u
#define SC_UART0_GATE           (1u << 2)
#define SC_UART1_GATE           (1u << 5)
#define SC_UART2_GATE           (1u << 9)
#define SC_UART_INT_ENA         0x00Cu
#define SC_UART_CONF1           0x024u
#define SC_UART_DATE            0x07Cu

typedef struct {
    unsigned reads;
    unsigned writes;
    uint32_t last_addr;
    uint32_t last_value;
} sc_fallback_probe_t;

typedef struct {
    unsigned changes;
    flexe_system_low_power_state_t state;
} sc_low_power_probe_t;

typedef struct {
    unsigned changes;
    flexe_system_peripheral_state_t state;
} sc_peripheral_probe_t;

static uint32_t sc_fallback_read(void *ctx, uint32_t addr)
{
    sc_fallback_probe_t *probe = ctx;
    probe->reads++;
    probe->last_addr = addr;
    return 0u;
}

static void sc_fallback_write(void *ctx, uint32_t addr, uint32_t value)
{
    sc_fallback_probe_t *probe = ctx;
    probe->writes++;
    probe->last_addr = addr;
    probe->last_value = value;
}

static void sc_low_power_changed(
    void *ctx, const flexe_system_low_power_state_t *state)
{
    sc_low_power_probe_t *probe = ctx;
    probe->changes++;
    probe->state = *state;
}

static void sc_peripheral_changed(
    void *ctx, const flexe_system_peripheral_state_t *state)
{
    sc_peripheral_probe_t *probe = ctx;
    probe->changes++;
    probe->state = *state;
}

static uint32_t sc_systimer_value(xtensa_mem_t *mem)
{
    mem_write32(mem, SC_SYSTIMER_BASE + SC_SYSTIMER_UNIT0_OP,
                SC_SYSTIMER_UPDATE);
    return mem_read32(mem, SC_SYSTIMER_BASE + SC_SYSTIMER_VALUE_LO);
}

static void sc_timg_start(xtensa_mem_t *mem, uint32_t base)
{
    mem_write32(mem, base + SC_TIMG_LOAD_LO, 0u);
    mem_write32(mem, base + SC_TIMG_LOAD, 1u);
    mem_write32(mem, base + SC_TIMG_CONFIG,
                SC_TIMG_ENABLE | SC_TIMG_INCREASE | (80u << 13));
}

static uint32_t sc_timg_value(xtensa_mem_t *mem, uint32_t base)
{
    mem_write32(mem, base + SC_TIMG_UPDATE, 0u);
    return mem_read32(mem, base + SC_TIMG_VALUE_LO);
}

TEST(system_clock_reset_masks_and_shared_page_composition)
{
    const flexe_target_desc_t *s3 =
        flexe_target_by_id(FLEXE_TARGET_ESP32S3);
    const flexe_system_clock_desc_t *desc = &s3->system_clock;
    xtensa_mem_t *mem = mem_create_for_target(s3);
    esp32_periph_t *periph = periph_create(mem);
    ASSERT_TRUE(mem != NULL);
    ASSERT_TRUE(periph != NULL);
    if (!mem || !periph) {
        periph_destroy(periph);
        mem_destroy(mem);
        return;
    }

    ASSERT_TRUE(s3->capabilities & FLEXE_TARGET_CAP_SYSTEM_CLOCK_V1);
    ASSERT_EQ(desc->base, SC_SYSTEM_BASE);
    ASSERT_EQ(mem_read32(mem, SC_SYSTEM_BASE + SC_CPU_PER_CONF_OFF),
              0x0000000Cu);
    ASSERT_EQ(mem_read32(mem, SC_SYSTEM_BASE + SC_SYSCLK_CONF_OFF),
              0x00000001u);
    ASSERT_EQ(mem_read32(mem, SC_SYSTEM_BASE + SC_MEM_PD_MASK_OFF), 1u);
    ASSERT_EQ(mem_read32(mem, SC_SYSTEM_BASE + SC_PERIP_CLK_EN0_OFF),
              0xF9C1E06Fu);
    ASSERT_EQ(mem_read32(mem, SC_SYSTEM_BASE + SC_PERIP_CLK_EN1_OFF),
              0x00000600u);
    ASSERT_EQ(mem_read32(mem, SC_SYSTEM_BASE + SC_PERIP_RST_EN0_OFF), 0u);
    ASSERT_EQ(mem_read32(mem, SC_SYSTEM_BASE + SC_PERIP_RST_EN1_OFF),
              0x000001FEu);
    ASSERT_EQ(mem_read32(mem, SC_SYSTEM_BASE + SC_BT_LPCK_DIV_INT_OFF),
              0x000000FFu);
    ASSERT_EQ(mem_read32(mem, SC_SYSTEM_BASE + SC_BT_LPCK_DIV_FRAC_OFF),
              0x02001001u);

    int before = periph_unhandled_count(periph);
    mem_write32(mem, SC_SYSTEM_BASE + SC_CPU_PER_CONF_OFF, UINT32_MAX);
    mem_write32(mem, SC_SYSTEM_BASE + SC_SYSCLK_CONF_OFF, UINT32_MAX);
    ASSERT_EQ(mem_read32(mem, SC_SYSTEM_BASE + SC_CPU_PER_CONF_OFF),
              0x000000FFu);
    ASSERT_EQ(mem_read32(mem, SC_SYSTEM_BASE + SC_SYSCLK_CONF_OFF),
              0x00000FFFu);

    /* This is the clock state used by a normal 160 MHz ESP-IDF boot: the
     * 480 MHz PLL feeds a 160 MHz CPU while AHB/APB remain at 80 MHz. */
    mem_write32(mem, SC_SYSTEM_BASE + SC_CPU_PER_CONF_OFF,
                SC_CPU_WAIT_FORCE_ON | SC_PLL_480M_SELECT |
                SC_CPU_PERIOD_160M);
    mem_write32(mem, SC_SYSTEM_BASE + SC_SYSCLK_CONF_OFF,
                SC_SOC_CLOCK_PLL);
    ASSERT_EQ(mem_read32(mem, SC_SYSTEM_BASE + SC_CPU_PER_CONF_OFF),
              0x0000000Du);
    ASSERT_EQ(mem_read32(mem, SC_SYSTEM_BASE + SC_SYSCLK_CONF_OFF),
              SC_SOC_CLOCK_PLL);

    /* Clock registers, APP CPU control, and FROM_CPU interrupts are separate
     * owners layered over the same 4 KiB SYSTEM aperture. */
    const uint32_t boot = s3->secondary_core.base +
                          s3->secondary_core.boot_address_offset;
    const uint32_t control = s3->secondary_core.base +
                             s3->secondary_core.control_offset;
    mem_write32(mem, boot, 0x403751E4u);
    mem_write32(mem, control, s3->secondary_core.clock_gate_mask);
    ASSERT_EQ(periph_app_cpu_boot_addr(periph), 0x403751E4u);
    ASSERT_TRUE(periph_app_cpu_released(periph));

    const uint32_t from_cpu =
        s3->interrupt_matrix.software_interrupt_base +
        s3->interrupt_matrix.software_interrupt_offset;
    mem_write32(mem, from_cpu, 1u);
    ASSERT_EQ(mem_read32(mem, from_cpu), 1u);
    mem_write32(mem, from_cpu, 0u);
    ASSERT_EQ(mem_read32(mem, from_cpu), 0u);
    ASSERT_EQ(periph_unhandled_count(periph), before);

    /* These fields publish semantic low-power policy rather than being
     * treated as unexplained register storage. */
    mem_write32(mem, SC_SYSTEM_BASE + SC_MEM_PD_MASK_OFF, 0u);
    mem_write32(mem, SC_SYSTEM_BASE + SC_BT_LPCK_DIV_INT_OFF, 0u);
    ASSERT_EQ(mem_read32(mem, SC_SYSTEM_BASE + SC_MEM_PD_MASK_OFF), 0u);
    ASSERT_EQ(mem_read32(mem, SC_SYSTEM_BASE + SC_BT_LPCK_DIV_INT_OFF), 0u);
    ASSERT_EQ(periph_unhandled_count(periph), before);

    /* Changes outside a semantic mapping remain explicit diagnostics. */
    ASSERT_EQ(mem_read32(mem, SC_SYSTEM_BASE + 0x008u), 0u);
    mem_write32(mem, SC_SYSTEM_BASE + 0x008u, 1u);
    ASSERT_EQ(periph_unhandled_count(periph), before + 2);
    ASSERT_EQ(mem_unmapped_count(mem), 0u);

    periph_destroy(periph);
    mem_destroy(mem);
}

TEST(system_clock_low_power_policy_is_target_described_and_observable)
{
    const flexe_target_desc_t *s3 =
        flexe_target_by_id(FLEXE_TARGET_ESP32S3);
    const flexe_system_low_power_desc_t *desc =
        &s3->system_clock.low_power;
    xtensa_mem_t *mem = mem_create_for_target(s3);
    sc_fallback_probe_t fallback = {0};
    flexe_system_clock_t *clock = flexe_system_clock_create(
        mem, sc_fallback_read, sc_fallback_write, &fallback,
        NULL, NULL);
    ASSERT_TRUE(mem != NULL);
    ASSERT_TRUE(clock != NULL);
    if (!mem || !clock) {
        flexe_system_clock_destroy(clock);
        mem_destroy(mem);
        return;
    }

    ASSERT_EQ(desc->memory_power_down_offset, SC_MEM_PD_MASK_OFF);
    ASSERT_EQ(desc->divider_integer_offset, SC_BT_LPCK_DIV_INT_OFF);
    ASSERT_EQ(desc->divider_fraction_offset, SC_BT_LPCK_DIV_FRAC_OFF);
    flexe_system_low_power_state_t state = {0};
    ASSERT_TRUE(flexe_system_clock_low_power_state(clock, &state));
    ASSERT_FALSE(state.memory_power_down_allowed);
    ASSERT_FALSE(state.rtc_clock_enabled);
    ASSERT_EQ(state.selected_sources,
              FLEXE_SYSTEM_LOW_POWER_SOURCE_INTERNAL);
    ASSERT_EQ(state.divider_integer, 255u);
    ASSERT_EQ(state.divider_a, 1u);
    ASSERT_EQ(state.divider_b, 1u);

    sc_low_power_probe_t probe = {0};
    flexe_system_clock_set_low_power_listener(
        clock, sc_low_power_changed, &probe);
    ASSERT_EQ(probe.changes, 1u);
    ASSERT_FALSE(probe.state.memory_power_down_allowed);

    mem_write32(mem, SC_SYSTEM_BASE + SC_MEM_PD_MASK_OFF, 0u);
    ASSERT_EQ(probe.changes, 2u);
    ASSERT_TRUE(probe.state.memory_power_down_allowed);
    mem_write32(mem, SC_SYSTEM_BASE + SC_BT_LPCK_DIV_INT_OFF, 7u);
    ASSERT_EQ(probe.changes, 3u);
    ASSERT_EQ(probe.state.divider_integer, 7u);

    uint32_t rtc_fraction = desc->rtc_clock_enable_mask |
                            desc->source_rtc_slow_mask |
                            (3u << 12u) | 2u;
    mem_write32(mem, SC_SYSTEM_BASE + SC_BT_LPCK_DIV_FRAC_OFF,
                rtc_fraction);
    ASSERT_EQ(probe.changes, 4u);
    ASSERT_TRUE(probe.state.rtc_clock_enabled);
    ASSERT_EQ(probe.state.selected_sources,
              FLEXE_SYSTEM_LOW_POWER_SOURCE_RTC_SLOW);
    ASSERT_EQ(probe.state.divider_a, 3u);
    ASSERT_EQ(probe.state.divider_b, 2u);
    mem_write32(mem, SC_SYSTEM_BASE + SC_BT_LPCK_DIV_FRAC_OFF,
                rtc_fraction);
    ASSERT_EQ(probe.changes, 4u);
    ASSERT_EQ(fallback.reads, 0u);
    ASSERT_EQ(fallback.writes, 0u);

    /* The hardware exposes independent source bits. Preserve conflicts for
     * a consumer to diagnose rather than selecting an arbitrary winner. */
    mem_write32(mem, SC_SYSTEM_BASE + SC_BT_LPCK_DIV_FRAC_OFF,
                rtc_fraction | desc->source_xtal_mask);
    ASSERT_EQ(probe.changes, 5u);
    ASSERT_EQ(probe.state.selected_sources,
              FLEXE_SYSTEM_LOW_POWER_SOURCE_RTC_SLOW |
              FLEXE_SYSTEM_LOW_POWER_SOURCE_XTAL);

    flexe_system_clock_set_low_power_listener(clock, NULL, NULL);
    flexe_system_clock_destroy(clock);
    mem_destroy(mem);

    flexe_target_desc_t invalid = *s3;
    invalid.system_clock.low_power.source_xtal_mask =
        invalid.system_clock.low_power.divider_a_mask;
    mem = mem_create_for_target(&invalid);
    clock = flexe_system_clock_create(
        mem, NULL, NULL, NULL, NULL, NULL);
    ASSERT_TRUE(mem != NULL);
    ASSERT_TRUE(clock == NULL);
    flexe_system_clock_destroy(clock);
    mem_destroy(mem);
}

TEST(system_clock_gates_and_resets_target_devices_at_exact_boundaries)
{
    const flexe_target_desc_t *s3 =
        flexe_target_by_id(FLEXE_TARGET_ESP32S3);
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

    uint32_t clock_addr = SC_SYSTEM_BASE + SC_PERIP_CLK_EN0_OFF;
    uint32_t reset_addr = SC_SYSTEM_BASE + SC_PERIP_RST_EN0_OFF;
    uint32_t clocks = mem_read32(mem, clock_addr);
    ASSERT_TRUE(clocks & SC_SYSTIMER_GATE);
    ASSERT_TRUE(clocks & SC_TIMG0_GATE);
    ASSERT_TRUE(clocks & SC_TIMG1_GATE);

    /* The 16 MHz SYSTIMER advances for exactly the clocked intervals. */
    cpu.ccount = 160u;
    ASSERT_EQ(sc_systimer_value(mem), 16u);
    mem_write32(mem, clock_addr, clocks & ~SC_SYSTIMER_GATE);
    cpu.ccount = 320u;
    ASSERT_EQ(sc_systimer_value(mem), 16u);
    mem_write32(mem, clock_addr, clocks);
    cpu.ccount = 480u;
    ASSERT_EQ(sc_systimer_value(mem), 32u);

    /* Raise a routed comparator interrupt, then prove a reset edge restores
     * registers, withdraws the line, and holds the counter at zero until
     * reset is released without disturbing other target devices. */
    mem_write32(mem, SC_SYSTIMER_BASE + SC_SYSTIMER_TARGET0_HI, 0u);
    mem_write32(mem, SC_SYSTIMER_BASE + SC_SYSTIMER_TARGET0_LO, 33u);
    mem_write32(mem, SC_SYSTIMER_BASE + SC_SYSTIMER_COMP0_LOAD, 1u);
    mem_write32(mem, SC_SYSTIMER_BASE + SC_SYSTIMER_INT_ENA, 1u);
    mem_write32(mem, SC_SYSTIMER_BASE + SC_SYSTIMER_CONF,
                mem_read32(mem, SC_SYSTIMER_BASE + SC_SYSTIMER_CONF) |
                SC_SYSTIMER_TARGET0_EN);
    cpu.ccount = 490u;
    cpu.periph_event(&cpu);
    ASSERT_EQ(mem_read32(mem, SC_SYSTIMER_BASE + SC_SYSTIMER_INT_RAW), 1u);
    ASSERT_TRUE(periph_interrupt_pending(periph, 57));
    mem_write32(mem, reset_addr, SC_SYSTIMER_GATE);
    ASSERT_EQ(mem_read32(mem, SC_SYSTIMER_BASE + SC_SYSTIMER_CONF),
              0x46000000u);
    ASSERT_EQ(mem_read32(mem, SC_SYSTIMER_BASE + SC_SYSTIMER_INT_RAW), 0u);
    ASSERT_FALSE(periph_interrupt_pending(periph, 57));
    cpu.ccount = 640u;
    ASSERT_EQ(sc_systimer_value(mem), 0u);
    mem_write32(mem, reset_addr, 0u);
    cpu.ccount = 800u;
    ASSERT_EQ(sc_systimer_value(mem), 16u);

    /* Timer groups have independent gates on the same SYSTEM register. */
    sc_timg_start(mem, SC_TIMG0_BASE);
    sc_timg_start(mem, SC_TIMG1_BASE);
    cpu.ccount = 2400u;
    ASSERT_EQ(sc_timg_value(mem, SC_TIMG0_BASE), 10u);
    ASSERT_EQ(sc_timg_value(mem, SC_TIMG1_BASE), 10u);
    mem_write32(mem, clock_addr, clocks & ~SC_TIMG0_GATE);
    cpu.ccount = 4000u;
    ASSERT_EQ(sc_timg_value(mem, SC_TIMG0_BASE), 10u);
    ASSERT_EQ(sc_timg_value(mem, SC_TIMG1_BASE), 20u);
    mem_write32(mem, clock_addr, clocks);
    cpu.ccount = 5600u;
    ASSERT_EQ(sc_timg_value(mem, SC_TIMG0_BASE), 20u);
    ASSERT_EQ(sc_timg_value(mem, SC_TIMG1_BASE), 30u);

    mem_write32(mem, reset_addr, SC_TIMG0_GATE);
    ASSERT_EQ(mem_read32(mem, SC_TIMG0_BASE + SC_TIMG_CONFIG),
              0x60002000u);
    ASSERT_EQ(sc_timg_value(mem, SC_TIMG0_BASE), 0u);
    ASSERT_TRUE(mem_read32(mem, SC_TIMG1_BASE + SC_TIMG_CONFIG) &
                SC_TIMG_ENABLE);
    mem_write32(mem, reset_addr, 0u);
    cpu.ccount = 7200u;
    ASSERT_EQ(sc_timg_value(mem, SC_TIMG0_BASE), 0u);
    ASSERT_EQ(sc_timg_value(mem, SC_TIMG1_BASE), 40u);
    ASSERT_EQ(periph_unhandled_count(periph), 0);

    periph_destroy(periph);
    mem_destroy(mem);
}

TEST(system_clock_publishes_complete_peripheral_banks)
{
    const flexe_target_desc_t *s3 =
        flexe_target_by_id(FLEXE_TARGET_ESP32S3);
    const flexe_system_peripheral_banks_desc_t *banks =
        &s3->system_clock.peripheral_banks;
    xtensa_mem_t *mem = mem_create_for_target(s3);
    sc_fallback_probe_t fallback = {0};
    flexe_system_clock_t *clock = flexe_system_clock_create(
        mem, sc_fallback_read, sc_fallback_write, &fallback,
        NULL, NULL);
    ASSERT_TRUE(mem != NULL);
    ASSERT_TRUE(clock != NULL);
    if (!mem || !clock) {
        flexe_system_clock_destroy(clock);
        mem_destroy(mem);
        return;
    }

    ASSERT_EQ(banks->bank_count, 2u);
    ASSERT_EQ(banks->clock_offset[0], SC_PERIP_CLK_EN0_OFF);
    ASSERT_EQ(banks->clock_offset[1], SC_PERIP_CLK_EN1_OFF);
    ASSERT_EQ(banks->reset_offset[0], SC_PERIP_RST_EN0_OFF);
    ASSERT_EQ(banks->reset_offset[1], SC_PERIP_RST_EN1_OFF);
    ASSERT_EQ(banks->valid_mask[0], UINT32_MAX);
    ASSERT_EQ(banks->valid_mask[1], 0x7FFu);

    flexe_system_peripheral_state_t state = {0};
    ASSERT_TRUE(flexe_system_clock_peripheral_state(clock, &state));
    ASSERT_EQ(state.valid_mask,
              (UINT64_C(0x7FF) << 32u) | UINT32_MAX);
    ASSERT_EQ(state.clock_enabled,
              (UINT64_C(0x600) << 32u) | UINT64_C(0xF9C1E06F));
    ASSERT_EQ(state.reset_asserted, UINT64_C(0x1FE) << 32u);

    sc_peripheral_probe_t probe = {0};
    flexe_system_clock_set_peripheral_listener(
        clock, sc_peripheral_changed, &probe);
    ASSERT_EQ(probe.changes, 1u);

    /* Full-register writes used during multicore ESP-IDF startup publish
     * every real clock/reset domain without turning reserved bits into state. */
    mem_write32(mem, SC_SYSTEM_BASE + SC_PERIP_CLK_EN0_OFF,
                0x7100E207u);
    mem_write32(mem, SC_SYSTEM_BASE + SC_PERIP_RST_EN0_OFF,
                0x8EFF1DF8u);
    ASSERT_EQ(probe.changes, 3u);
    ASSERT_EQ((uint32_t)probe.state.clock_enabled, 0x7100E207u);
    ASSERT_EQ((uint32_t)probe.state.reset_asserted, 0x8EFF1DF8u);
    ASSERT_EQ(fallback.writes, 0u);

    mem_write32(mem, SC_SYSTEM_BASE + SC_PERIP_CLK_EN1_OFF,
                UINT32_MAX);
    mem_write32(mem, SC_SYSTEM_BASE + SC_PERIP_RST_EN1_OFF,
                UINT32_MAX);
    ASSERT_EQ(probe.changes, 5u);
    ASSERT_EQ(probe.state.clock_enabled >> 32u, 0x7FFu);
    ASSERT_EQ(probe.state.reset_asserted >> 32u, 0x7FFu);
    ASSERT_EQ(mem_read32(mem, SC_SYSTEM_BASE + SC_PERIP_CLK_EN1_OFF),
              0x7FFu);
    ASSERT_EQ(mem_read32(mem, SC_SYSTEM_BASE + SC_PERIP_RST_EN1_OFF),
              0x7FFu);
    ASSERT_EQ(fallback.writes, 0u);

    flexe_system_clock_set_peripheral_listener(clock, NULL, NULL);
    mem_write32(mem, SC_SYSTEM_BASE + SC_PERIP_CLK_EN0_OFF, 0u);
    ASSERT_EQ(probe.changes, 5u);
    ASSERT_FALSE(flexe_system_clock_peripheral_state(NULL, &state));
    ASSERT_FALSE(flexe_system_clock_peripheral_state(clock, NULL));

    flexe_system_clock_destroy(clock);
    mem_destroy(mem);

    flexe_target_desc_t invalid = *s3;
    invalid.system_clock.peripheral_banks.valid_mask[1] = 1u << 31;
    mem = mem_create_for_target(&invalid);
    clock = flexe_system_clock_create(
        mem, NULL, NULL, NULL, NULL, NULL);
    ASSERT_TRUE(mem != NULL);
    ASSERT_TRUE(clock == NULL);
    flexe_system_clock_destroy(clock);
    mem_destroy(mem);
}

TEST(system_clock_uart_gates_and_resets_are_per_port)
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

    uint32_t clocks0 = mem_read32(mem, SC_SYSTEM_BASE +
                                       SC_PERIP_CLK_EN0_OFF);
    uint32_t clocks1 = mem_read32(mem, SC_SYSTEM_BASE +
                                       SC_PERIP_CLK_EN1_OFF);
    ASSERT_TRUE(clocks0 & SC_UART0_GATE);
    ASSERT_TRUE(clocks0 & SC_UART1_GATE);
    ASSERT_TRUE(clocks1 & SC_UART2_GATE);

    mem_write32(mem, SC_UART0_BASE, 'A');
    ASSERT_EQ(periph_uart_tx_count_num(periph, 0), 1);
    mem_write32(mem, SC_UART0_BASE + SC_UART_INT_ENA, 1u << 1);
    ASSERT_TRUE(periph_interrupt_pending(periph, 27));

    mem_write32(mem, SC_SYSTEM_BASE + SC_PERIP_CLK_EN0_OFF,
                clocks0 & ~SC_UART0_GATE);
    ASSERT_FALSE(periph_interrupt_pending(periph, 27));
    mem_write32(mem, SC_UART0_BASE, 'B');
    ASSERT_EQ(periph_uart_tx_count_num(periph, 0), 1);
    const uint8_t input[] = { 'x', 'y' };
    ASSERT_EQ(periph_uart_rx_inject_num(periph, 0, input, sizeof(input)), 0u);
    mem_write32(mem, SC_UART1_BASE, '1');
    ASSERT_EQ(periph_uart_tx_count_num(periph, 1), 1);

    mem_write32(mem, SC_SYSTEM_BASE + SC_PERIP_CLK_EN0_OFF, clocks0);
    ASSERT_TRUE(periph_interrupt_pending(periph, 27));
    ASSERT_EQ(periph_uart_rx_inject_num(periph, 0, input, sizeof(input)),
              sizeof(input));
    ASSERT_EQ(mem_read32(mem, SC_UART0_BASE), 'x');
    ASSERT_EQ(periph_uart_rx_pending_num(periph, 0), 1u);
    mem_write32(mem, SC_UART0_BASE + SC_UART_CONF1, 0x1234u);
    mem_write32(mem, SC_SYSTEM_BASE + SC_PERIP_RST_EN0_OFF,
                SC_UART0_GATE);
    ASSERT_FALSE(periph_interrupt_pending(periph, 27));
    ASSERT_EQ(periph_uart_rx_pending_num(periph, 0), 0u);
    ASSERT_EQ(mem_read32(mem, SC_UART0_BASE + SC_UART_INT_ENA), 0u);
    ASSERT_EQ(mem_read32(mem, SC_UART0_BASE + SC_UART_CONF1), 0u);
    ASSERT_EQ(mem_read32(mem, SC_UART0_BASE + SC_UART_DATE),
              s3->uart_ip.date_reset);
    mem_write32(mem, SC_UART0_BASE, 'C');
    ASSERT_EQ(periph_uart_tx_count_num(periph, 0), 1);
    ASSERT_EQ(periph_uart_rx_inject_num(periph, 0, input, sizeof(input)), 0u);
    mem_write32(mem, SC_UART1_BASE, '2');
    ASSERT_EQ(periph_uart_tx_count_num(periph, 1), 2);
    mem_write32(mem, SC_SYSTEM_BASE + SC_PERIP_RST_EN0_OFF, 0u);
    mem_write32(mem, SC_UART0_BASE, 'D');
    ASSERT_EQ(periph_uart_tx_count_num(periph, 0), 2);

    mem_write32(mem, SC_SYSTEM_BASE + SC_PERIP_CLK_EN1_OFF,
                clocks1 & ~SC_UART2_GATE);
    mem_write32(mem, SC_UART2_BASE, 'E');
    ASSERT_EQ(periph_uart_tx_count_num(periph, 2), 0);
    mem_write32(mem, SC_SYSTEM_BASE + SC_PERIP_CLK_EN1_OFF, clocks1);
    mem_write32(mem, SC_UART2_BASE, 'F');
    ASSERT_EQ(periph_uart_tx_count_num(periph, 2), 1);
    mem_write32(mem, SC_SYSTEM_BASE + SC_PERIP_RST_EN1_OFF,
                mem_read32(mem, SC_SYSTEM_BASE + SC_PERIP_RST_EN1_OFF) |
                SC_UART2_GATE);
    ASSERT_EQ(mem_read32(mem, SC_UART2_BASE + SC_UART_DATE),
              s3->uart_ip.date_reset);
    ASSERT_EQ(periph_unhandled_count(periph), 0);

    periph_destroy(periph);
    mem_destroy(mem);
}

void run_system_clock_tests(void)
{
    TEST_SUITE("Target system clock");
    RUN_TEST(system_clock_reset_masks_and_shared_page_composition);
    RUN_TEST(system_clock_low_power_policy_is_target_described_and_observable);
    RUN_TEST(system_clock_gates_and_resets_target_devices_at_exact_boundaries);
    RUN_TEST(system_clock_publishes_complete_peripheral_banks);
    RUN_TEST(system_clock_uart_gates_and_resets_are_per_port);
}
