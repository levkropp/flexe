/* Target-described CPU/system-clock register tests. */
#include "test_helpers.h"
#include "peripherals.h"

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

    /* Register readback is architectural even before every downstream
     * electrical effect exists. Changes outside semantic gate mappings stay
     * visible as unsupported instead of becoming silent fake behavior. */
    mem_write32(mem, SC_SYSTEM_BASE + SC_MEM_PD_MASK_OFF, 0u);
    mem_write32(mem, SC_SYSTEM_BASE + SC_BT_LPCK_DIV_INT_OFF, 0u);
    ASSERT_EQ(mem_read32(mem, SC_SYSTEM_BASE + SC_MEM_PD_MASK_OFF), 0u);
    ASSERT_EQ(mem_read32(mem, SC_SYSTEM_BASE + SC_BT_LPCK_DIV_INT_OFF), 0u);
    ASSERT_EQ(periph_unhandled_count(periph), before + 2);

    ASSERT_EQ(mem_read32(mem, SC_SYSTEM_BASE + 0x008u), 0u);
    mem_write32(mem, SC_SYSTEM_BASE + 0x008u, 1u);
    ASSERT_EQ(periph_unhandled_count(periph), before + 4);
    ASSERT_EQ(mem_unmapped_count(mem), 0u);

    periph_destroy(periph);
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

void run_system_clock_tests(void)
{
    TEST_SUITE("Target system clock");
    RUN_TEST(system_clock_reset_masks_and_shared_page_composition);
    RUN_TEST(system_clock_gates_and_resets_target_devices_at_exact_boundaries);
}
