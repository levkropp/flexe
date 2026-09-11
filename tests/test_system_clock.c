/* Target-described CPU/system-clock register tests. */
#include "test_helpers.h"
#include "peripherals.h"

#define SC_SYSTEM_BASE          0x600C0000u
#define SC_CPU_PER_CONF_OFF     0x010u
#define SC_SYSCLK_CONF_OFF      0x060u
#define SC_CPU_WAIT_FORCE_ON    (1u << 3)
#define SC_PLL_480M_SELECT      (1u << 2)
#define SC_CPU_PERIOD_160M      1u
#define SC_SOC_CLOCK_PLL        (1u << 10)

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

    ASSERT_EQ(mem_read32(mem, SC_SYSTEM_BASE + 0x008u), 0u);
    mem_write32(mem, SC_SYSTEM_BASE + 0x008u, 1u);
    ASSERT_EQ(periph_unhandled_count(periph), before + 2);
    ASSERT_EQ(mem_unmapped_count(mem), 0u);

    periph_destroy(periph);
    mem_destroy(mem);
}

void run_system_clock_tests(void)
{
    TEST_SUITE("Target system clock");
    RUN_TEST(system_clock_reset_masks_and_shared_page_composition);
}
