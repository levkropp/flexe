/* Target-described S2/S3-generation digital GPIO tests. */
#include "test_helpers.h"
#include "gpio.h"
#include "peripherals.h"

TEST(target_gpio_models_s3_banks_masks_and_software_output)
{
    const flexe_target_desc_t *s3 =
        flexe_target_by_id(FLEXE_TARGET_ESP32S3);
    const flexe_gpio_desc_t *desc = &s3->gpio;
    xtensa_mem_t *mem = mem_create_for_target(s3);
    esp32_periph_t *periph = periph_create(mem);
    ASSERT_TRUE(mem != NULL);
    ASSERT_TRUE(periph != NULL);
    if (!mem || !periph) {
        periph_destroy(periph);
        mem_destroy(mem);
        return;
    }

    ASSERT_TRUE(s3->capabilities & FLEXE_TARGET_CAP_GPIO_V1);
    ASSERT_EQ(mem_read32(mem, desc->base + 0x004u), 0u);
    ASSERT_EQ(mem_read32(mem, desc->base + 0x010u), 0u);
    ASSERT_EQ(mem_read32(mem, desc->base + 0x020u), 0u);
    ASSERT_EQ(mem_read32(mem, desc->base + 0x02Cu), 0u);
    ASSERT_EQ(mem_read32(mem, desc->base + 0x554u), 0x100u);
    ASSERT_EQ(mem_read32(mem, desc->base + 0x614u), 0x100u);
    ASSERT_EQ(mem_read32(mem, desc->base + 0x62Cu), 1u);
    ASSERT_EQ(mem_read32(mem, desc->base + 0x6FCu), 0x01907040u);

    mem_write32(mem, desc->base + 0x008u, 1u << 31u);
    mem_write32(mem, desc->base + 0x024u, 1u << 31u);
    ASSERT_EQ(mem_read32(mem, desc->base + 0x004u), 1u << 31u);
    ASSERT_EQ(mem_read32(mem, desc->base + 0x020u), 1u << 31u);
    ASSERT_EQ(periph_gpio_pin_level(periph, 31), 1);
    ASSERT_EQ(periph_gpio_output_enabled(periph, 31), 1);

    mem_write32(mem, desc->base + 0x014u, 1u << 16u);
    mem_write32(mem, desc->base + 0x030u, 1u << 16u);
    ASSERT_EQ(periph_gpio_pin_level(periph, 48), 1);
    ASSERT_EQ(periph_gpio_output_enabled(periph, 48), 1);
    ASSERT_EQ(periph_gpio_out_signal(periph, 48), 256);
    ASSERT_EQ(periph_gpio_pin_level(periph, 22), -1);
    ASSERT_EQ(periph_gpio_pin_level(periph, 49), -1);

    mem_write32(mem, desc->base + 0x614u, 0x100u | (1u << 9u));
    ASSERT_EQ(periph_gpio_pin_level(periph, 48), 0);
    mem_write32(mem, desc->base + 0x018u, 1u << 16u);
    ASSERT_EQ(periph_gpio_pin_level(periph, 48), 1);
    mem_write32(mem, desc->base + 0x010u, UINT32_MAX);
    mem_write32(mem, desc->base + 0x02Cu, UINT32_MAX);
    ASSERT_EQ(mem_read32(mem, desc->base + 0x010u), 0x003FFFFFu);
    ASSERT_EQ(mem_read32(mem, desc->base + 0x02Cu), 0x003FFFFFu);
    ASSERT_EQ(periph_unhandled_count(periph), 0);
    ASSERT_EQ(mem_unmapped_count(mem), 0u);

    periph_destroy(periph);
    mem_destroy(mem);
}

TEST(target_gpio_routes_s3_edge_and_level_interrupts_to_both_cores)
{
    const flexe_target_desc_t *s3 =
        flexe_target_by_id(FLEXE_TARGET_ESP32S3);
    const flexe_gpio_desc_t *desc = &s3->gpio;
    xtensa_mem_t *mem = mem_create_for_target(s3);
    esp32_periph_t *periph = periph_create(mem);
    xtensa_cpu_t cpu0;
    xtensa_cpu_t cpu1;
    ASSERT_TRUE(mem != NULL);
    ASSERT_TRUE(periph != NULL);
    if (!mem || !periph) {
        periph_destroy(periph);
        mem_destroy(mem);
        return;
    }

    xtensa_cpu_init_for_target(&cpu0, s3);
    xtensa_cpu_init_for_target(&cpu1, s3);
    cpu0.mem = mem;
    cpu1.mem = mem;
    periph_attach_cpus(periph, &cpu0, &cpu1);
    periph_intr_matrix_set(periph, 0, 4, desc->interrupt_source);
    periph_intr_matrix_set(periph, 1, 4, desc->interrupt_source);
    periph_intr_matrix_set(periph, 0, 14, desc->nmi_interrupt_source);
    periph_intr_matrix_set(periph, 1, 14, desc->nmi_interrupt_source);

    uint32_t pin4 = desc->base + 0x074u + 4u * 4u;
    mem_write32(mem, pin4, (1u << 13u) | (1u << 7u));
    periph_gpio_set_input(periph, 4, 1);
    ASSERT_EQ(mem_read32(mem, desc->base + 0x044u), 1u << 4u);
    ASSERT_EQ(mem_read32(mem, desc->base + 0x05Cu), 1u << 4u);
    ASSERT_TRUE(periph_interrupt_pending(periph, desc->interrupt_source));
    ASSERT_TRUE(cpu0.interrupt & (1u << 4u));
    ASSERT_TRUE(cpu1.interrupt & (1u << 4u));
    mem_write32(mem, desc->base + 0x04Cu, 1u << 4u);
    ASSERT_FALSE(periph_interrupt_pending(periph, desc->interrupt_source));
    ASSERT_FALSE(cpu0.interrupt & (1u << 4u));
    ASSERT_FALSE(cpu1.interrupt & (1u << 4u));

    uint32_t pin48 = desc->base + 0x074u + 48u * 4u;
    mem_write32(mem, pin48, (1u << 14u) | (5u << 7u));
    periph_gpio_set_input(periph, 48, 1);
    ASSERT_EQ(mem_read32(mem, desc->base + 0x050u), 1u << 16u);
    ASSERT_EQ(mem_read32(mem, desc->base + 0x06Cu), 1u << 16u);
    ASSERT_TRUE(periph_interrupt_pending(periph,
                                         desc->nmi_interrupt_source));
    ASSERT_TRUE(cpu0.interrupt & (1u << 14u));
    ASSERT_TRUE(cpu1.interrupt & (1u << 14u));

    /* A high-level source re-latches immediately after W1TC while the input
     * remains high, then drops only after the level becomes inactive and the
     * guest acknowledges the latched bit. */
    mem_write32(mem, desc->base + 0x058u, 1u << 16u);
    ASSERT_EQ(mem_read32(mem, desc->base + 0x050u), 1u << 16u);
    periph_gpio_set_input(periph, 48, 0);
    mem_write32(mem, desc->base + 0x058u, 1u << 16u);
    ASSERT_EQ(mem_read32(mem, desc->base + 0x050u), 0u);
    ASSERT_FALSE(periph_interrupt_pending(periph,
                                          desc->nmi_interrupt_source));
    ASSERT_FALSE(cpu0.interrupt & (1u << 14u));
    ASSERT_FALSE(cpu1.interrupt & (1u << 14u));
    ASSERT_EQ(periph_unhandled_count(periph), 0);

    periph_destroy(periph);
    mem_destroy(mem);
}

TEST(target_gpio_resolves_matrix_inputs_and_rejects_unbonded_pads)
{
    const flexe_target_desc_t *s3 =
        flexe_target_by_id(FLEXE_TARGET_ESP32S3);
    const flexe_gpio_desc_t *desc = &s3->gpio;
    xtensa_mem_t *mem = mem_create_for_target(s3);
    flexe_gpio_t *gpio = flexe_gpio_create(
        mem, NULL, NULL, NULL, NULL, NULL, NULL, NULL);
    ASSERT_TRUE(mem != NULL);
    ASSERT_TRUE(gpio != NULL);
    if (!mem || !gpio) {
        flexe_gpio_destroy(gpio);
        mem_destroy(mem);
        return;
    }

    uint32_t signal42 = desc->base + 0x154u + 42u * 4u;
    flexe_gpio_set_input(gpio, 48u, true);
    mem_write32(mem, signal42, (1u << 7u) | 48u);
    ASSERT_EQ(flexe_gpio_input_signal_level(gpio, 42u), 1);
    mem_write32(mem, signal42, (1u << 7u) | (1u << 6u) | 48u);
    ASSERT_EQ(flexe_gpio_input_signal_level(gpio, 42u), 0);
    mem_write32(mem, signal42,
                (1u << 7u) | desc->matrix_const_one_input);
    ASSERT_EQ(flexe_gpio_input_signal_level(gpio, 42u), 1);
    mem_write32(mem, signal42,
                (1u << 7u) | desc->matrix_const_zero_input);
    ASSERT_EQ(flexe_gpio_input_signal_level(gpio, 42u), 0);
    mem_write32(mem, signal42, (1u << 7u) | 22u);
    ASSERT_EQ(flexe_gpio_input_signal_level(gpio, 42u), -1);
    mem_write32(mem, signal42, 48u);
    ASSERT_EQ(flexe_gpio_input_signal_level(gpio, 42u), -1);

    flexe_gpio_destroy(gpio);
    mem_destroy(mem);
}

TEST(target_gpio_diagnoses_behavior_outside_functional_envelope)
{
    const flexe_target_desc_t *s3 =
        flexe_target_by_id(FLEXE_TARGET_ESP32S3);
    const flexe_gpio_desc_t *desc = &s3->gpio;
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
    mem_write32(mem, desc->base + 0x074u, 1u << 2u); /* open drain */
    ASSERT_EQ(periph_unhandled_count(periph), before + 1);
    ASSERT_EQ(mem_read32(mem, desc->base + 0x074u), 1u << 2u);
    mem_write32(mem, desc->base + 0x554u, 43u); /* unattached producer */
    ASSERT_EQ(periph_unhandled_count(periph), before + 2);
    ASSERT_EQ(periph_gpio_out_signal(periph, 0), 43);
    ASSERT_EQ(periph_gpio_pin_level(periph, 0), -1);
    mem_write32(mem, desc->base + 0x62Cu, 0u); /* gate effect */
    ASSERT_EQ(periph_unhandled_count(periph), before + 3);
    ASSERT_EQ(mem_read32(mem, desc->base + 0x62Cu), 0u);
    ASSERT_EQ(mem_read32(mem, desc->base + 0x630u), 0u);
    mem_write32(mem, desc->base + 0x630u, 1u);
    ASSERT_EQ(periph_unhandled_count(periph), before + 5);
    ASSERT_EQ(mem_unmapped_count(mem), 0u);

    periph_destroy(periph);
    mem_destroy(mem);
}

void run_target_gpio_tests(void)
{
    TEST_SUITE("Target GPIO");
    RUN_TEST(target_gpio_models_s3_banks_masks_and_software_output);
    RUN_TEST(target_gpio_routes_s3_edge_and_level_interrupts_to_both_cores);
    RUN_TEST(target_gpio_resolves_matrix_inputs_and_rejects_unbonded_pads);
    RUN_TEST(target_gpio_diagnoses_behavior_outside_functional_envelope);
}
