/* ESP32-S3 EXTMEM cache-controller tests. */
#include "test_helpers.h"
#include "esp32s3_extmem.h"
#include "peripherals.h"

#define S3_EXTMEM_BASE 0x600C4000u

typedef struct {
    unsigned calls;
    uint32_t addr;
    size_t len;
} s3_invalidate_probe_t;

static void s3_invalidate(void *ctx, uint32_t addr, size_t len)
{
    s3_invalidate_probe_t *probe = ctx;
    probe->calls++;
    probe->addr = addr;
    probe->len = len;
}

TEST(esp32s3_extmem_exposes_documented_reset_state) {
    const flexe_target_desc_t *s3 =
        flexe_target_by_id(FLEXE_TARGET_ESP32S3);
    xtensa_mem_t *mem = mem_create_for_target(s3);
    flexe_esp32s3_extmem_t *extmem =
        flexe_esp32s3_extmem_create(mem);
    ASSERT_TRUE(mem != NULL);
    ASSERT_TRUE(extmem != NULL);
    if (!mem || !extmem) {
        flexe_esp32s3_extmem_destroy(extmem);
        mem_destroy(mem);
        return;
    }

    ASSERT_EQ(mem_read32(mem, S3_EXTMEM_BASE + 0x000u), 0u);
    ASSERT_EQ(mem_read32(mem, S3_EXTMEM_BASE + 0x004u), 3u);
    ASSERT_EQ(mem_read32(mem, S3_EXTMEM_BASE + 0x060u), 0u);
    ASSERT_EQ(mem_read32(mem, S3_EXTMEM_BASE + 0x064u), 3u);
    ASSERT_EQ(mem_read32(mem, S3_EXTMEM_BASE + 0x130u), 0x1001u);
    ASSERT_EQ(mem_read32(mem, S3_EXTMEM_BASE + 0x3FCu), 0x02012310u);
    ASSERT_EQ(mem_unmapped_count(mem), 0u);

    flexe_esp32s3_extmem_destroy(extmem);
    mem_destroy(mem);
}

TEST(esp32s3_extmem_application_handoff_enables_cache_buses) {
    const flexe_target_desc_t *s3 =
        flexe_target_by_id(FLEXE_TARGET_ESP32S3);
    xtensa_mem_t *mem = mem_create_for_target(s3);
    flexe_esp32s3_extmem_t *extmem =
        flexe_esp32s3_extmem_create(mem);
    ASSERT_TRUE(mem != NULL);
    ASSERT_TRUE(extmem != NULL);
    if (!mem || !extmem) {
        flexe_esp32s3_extmem_destroy(extmem);
        mem_destroy(mem);
        return;
    }

    flexe_esp32s3_extmem_application_handoff(extmem);
    ASSERT_EQ(mem_read32(mem, S3_EXTMEM_BASE + 0x000u) & 1u, 1u);
    ASSERT_EQ(mem_read32(mem, S3_EXTMEM_BASE + 0x004u), 0u);
    ASSERT_EQ(mem_read32(mem, S3_EXTMEM_BASE + 0x060u) & 1u, 1u);
    ASSERT_EQ(mem_read32(mem, S3_EXTMEM_BASE + 0x064u), 0u);
    ASSERT_EQ(mem_read32(mem, S3_EXTMEM_BASE + 0x0B4u), 0x42000000u);
    ASSERT_EQ(mem_read32(mem, S3_EXTMEM_BASE + 0x0B8u), 0x43FFFFFFu);
    ASSERT_EQ(mem_read32(mem, S3_EXTMEM_BASE + 0x0BCu), 0x3C000000u);
    ASSERT_EQ(mem_read32(mem, S3_EXTMEM_BASE + 0x0C0u), 0x3DFFFFFFu);

    flexe_esp32s3_extmem_destroy(extmem);
    mem_destroy(mem);
}

TEST(esp32s3_extmem_operations_complete_and_invalidate_code) {
    const flexe_target_desc_t *s3 =
        flexe_target_by_id(FLEXE_TARGET_ESP32S3);
    xtensa_mem_t *mem = mem_create_for_target(s3);
    flexe_esp32s3_extmem_t *extmem =
        flexe_esp32s3_extmem_create(mem);
    xtensa_cpu_t cpu[2];
    s3_invalidate_probe_t probe[2] = {0};
    ASSERT_TRUE(mem != NULL);
    ASSERT_TRUE(extmem != NULL);
    if (!mem || !extmem) {
        flexe_esp32s3_extmem_destroy(extmem);
        mem_destroy(mem);
        return;
    }

    xtensa_cpu_init_for_target(&cpu[0], s3);
    xtensa_cpu_init_for_target(&cpu[1], s3);
    cpu[0].code_invalidate = s3_invalidate;
    cpu[0].code_invalidate_ctx = &probe[0];
    cpu[1].code_invalidate = s3_invalidate;
    cpu[1].code_invalidate_ctx = &probe[1];
    flexe_esp32s3_extmem_attach_cpus(extmem, &cpu[0], &cpu[1]);

    mem_write32(mem, S3_EXTMEM_BASE + 0x08Cu, 0x42012340u);
    mem_write32(mem, S3_EXTMEM_BASE + 0x090u, 4u);
    mem_write32(mem, S3_EXTMEM_BASE + 0x088u, 1u);
    ASSERT_EQ(mem_read32(mem, S3_EXTMEM_BASE + 0x088u), 2u);
    ASSERT_EQ(probe[0].calls, 1u);
    ASSERT_EQ(probe[1].calls, 1u);
    ASSERT_EQ(probe[0].addr, 0x42012340u);
    ASSERT_EQ(probe[0].len, 64u);

    mem_write32(mem, S3_EXTMEM_BASE + 0x028u, 7u);
    ASSERT_EQ(mem_read32(mem, S3_EXTMEM_BASE + 0x028u), 8u);
    mem_write32(mem, S3_EXTMEM_BASE + 0x01Cu, 3u);
    ASSERT_EQ(mem_read32(mem, S3_EXTMEM_BASE + 0x01Cu), 4u);
    mem_write32(mem, S3_EXTMEM_BASE + 0x040u, 5u);
    ASSERT_EQ(mem_read32(mem, S3_EXTMEM_BASE + 0x040u), 6u);

    mem_write32(mem, S3_EXTMEM_BASE + 0x150u, 1u);
    ASSERT_EQ(mem_read32(mem, S3_EXTMEM_BASE + 0x150u), 5u);
    mem_write32(mem, S3_EXTMEM_BASE + 0x150u, 4u);
    ASSERT_EQ(mem_read32(mem, S3_EXTMEM_BASE + 0x150u), 0u);
    mem_write32(mem, S3_EXTMEM_BASE + 0x154u, 3u);
    ASSERT_EQ(mem_read32(mem, S3_EXTMEM_BASE + 0x154u), 7u);
    mem_write32(mem, S3_EXTMEM_BASE + 0x154u, 6u);
    ASSERT_EQ(mem_read32(mem, S3_EXTMEM_BASE + 0x154u), 2u);

    flexe_esp32s3_extmem_destroy(extmem);
    mem_destroy(mem);
}

TEST(peripherals_compose_s3_devices_without_classic_aliases) {
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

    /* periph_create models direct application entry and owns the shared MMU. */
    ASSERT_EQ(mem_read32(mem, S3_EXTMEM_BASE) & 1u, 1u);
    ASSERT_EQ(mem_read32(mem, S3_EXTMEM_BASE + 0x60u) & 1u, 1u);
    ASSERT_EQ(mem_read32(mem, 0x600C5000u), 0x4000u);
    ASSERT_TRUE(mem_get_ptr(mem, 0x42000000u) == NULL);

    /* A classic DPORT address is S3 D-ROM, not an alias into native MMIO. */
    mem_write32(mem, 0x40040000u, 0xA55A1234u);
    uint64_t before = mem_unmapped_count(mem);
    ASSERT_EQ(mem_read32(mem, 0x3FF00000u), 0xA55A1234u);
    ASSERT_EQ(mem_unmapped_count(mem), before);

    periph_destroy(periph);
    mem_destroy(mem);
}

TEST(peripherals_compose_target_described_s3_uarts) {
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

    int before = periph_unhandled_count(periph);
    mem_write32(mem, 0x60000000u, 'S');
    mem_write32(mem, 0x60010000u, '3');
    mem_write32(mem, 0x6002E000u, '!');
    ASSERT_EQ(periph_uart_tx_count_num(periph, 0), 1);
    ASSERT_EQ(periph_uart_tx_count_num(periph, 1), 1);
    ASSERT_EQ(periph_uart_tx_count_num(periph, 2), 1);
    ASSERT_EQ(periph_uart_tx_buf_num(periph, 0)[0], 'S');
    ASSERT_EQ(periph_uart_tx_buf_num(periph, 1)[0], '3');
    ASSERT_EQ(periph_uart_tx_buf_num(periph, 2)[0], '!');

    static const uint8_t rx[] = { 'O', 'K' };
    ASSERT_EQ(periph_uart_rx_inject_num(periph, 0, rx, sizeof(rx)),
              sizeof(rx));
    ASSERT_EQ(mem_read32(mem, 0x6000001Cu) & 0x3FFu, 2u);
    ASSERT_EQ(mem_read32(mem, 0x60000068u), 2u << 11);
    ASSERT_EQ(mem_read32(mem, 0x60000000u), 'O');
    ASSERT_EQ(mem_read32(mem, 0x60000068u), 1u | (2u << 11));
    ASSERT_EQ(mem_read32(mem, 0x6000007Cu), 0x02008270u);
    ASSERT_EQ(periph_unhandled_count(periph), before);

    periph_destroy(periph);
    mem_destroy(mem);
}

TEST(peripherals_model_target_described_secondary_core_control) {
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

    const uint32_t control = s3->secondary_core.base +
                             s3->secondary_core.control_offset;
    const uint32_t boot = s3->secondary_core.base +
                          s3->secondary_core.boot_address_offset;
    ASSERT_EQ(mem_read32(mem, control), 1u << 2);
    ASSERT_EQ(mem_read32(mem, boot), 0u);
    ASSERT_FALSE(periph_app_cpu_released(periph));

    mem_write32(mem, boot, 0x403751E4u);
    ASSERT_EQ(periph_app_cpu_boot_addr(periph), 0x403751E4u);
    mem_write32(mem, control, (1u << 2) | (1u << 1));
    ASSERT_FALSE(periph_app_cpu_released(periph));
    mem_write32(mem, control, 1u << 1);
    ASSERT_TRUE(periph_app_cpu_released(periph));
    mem_write32(mem, control, (1u << 1) | (1u << 0));
    ASSERT_FALSE(periph_app_cpu_released(periph));

    int before = periph_unhandled_count(periph);
    ASSERT_EQ(mem_read32(mem, s3->secondary_core.base + 8u), 0u);
    ASSERT_EQ(periph_unhandled_count(periph), before + 1);

    periph_destroy(periph);
    mem_destroy(mem);
}

TEST(peripherals_model_target_described_rtc_calibration) {
    const flexe_target_desc_t *s3 =
        flexe_target_by_id(FLEXE_TARGET_ESP32S3);
    const flexe_rtc_calibration_desc_t *desc = &s3->rtc_calibration;
    xtensa_mem_t *mem = mem_create_for_target(s3);
    esp32_periph_t *periph = periph_create(mem);
    ASSERT_TRUE(mem != NULL);
    ASSERT_TRUE(periph != NULL);
    if (!mem || !periph) {
        periph_destroy(periph);
        mem_destroy(mem);
        return;
    }

    uint32_t cfg0 = desc->base[0] + desc->config_offset;
    uint32_t value0 = desc->base[0] + desc->value_offset;
    uint32_t timeout0 = desc->base[0] + desc->timeout_offset;
    ASSERT_EQ(mem_read32(mem, cfg0), desc->config_reset);
    ASSERT_EQ(mem_read32(mem, cfg0),
              desc->config_reset | desc->ready_mask);
    ASSERT_EQ(mem_read32(mem, value0), 585u << desc->result_shift);
    ASSERT_EQ(mem_read32(mem, timeout0),
              desc->timeout_reset & desc->timeout_writable_mask);

    uint32_t one_shot = (1024u << desc->cycles_shift) |
                        desc->start_mask;
    mem_write32(mem, cfg0, one_shot);
    ASSERT_EQ(mem_read32(mem, cfg0), one_shot);
    ASSERT_EQ(mem_read32(mem, cfg0), one_shot | desc->ready_mask);
    ASSERT_EQ(mem_read32(mem, value0), 301176u << desc->result_shift);
    mem_write32(mem, cfg0, one_shot & ~desc->start_mask);
    ASSERT_EQ(mem_read32(mem, cfg0), one_shot & ~desc->start_mask);

    mem_write32(mem, timeout0, 0x12345679u);
    ASSERT_EQ(mem_read32(mem, timeout0),
              0x12345679u & desc->timeout_writable_mask);

    /* Each timer group owns independent calibration state. */
    uint32_t cfg1 = desc->base[1] + desc->config_offset;
    uint32_t value1 = desc->base[1] + desc->value_offset;
    uint32_t xtal32k = (10u << desc->cycles_shift) |
                       (2u << desc->clock_select_shift) |
                       desc->start_mask;
    mem_write32(mem, cfg1, 0u);
    mem_write32(mem, cfg1, xtal32k);
    ASSERT_EQ(mem_read32(mem, cfg1), xtal32k);
    ASSERT_EQ(mem_read32(mem, cfg1), xtal32k | desc->ready_mask);
    ASSERT_EQ(mem_read32(mem, value1), 12207u << desc->result_shift);
    ASSERT_EQ(mem_read32(mem, value0), 301176u << desc->result_shift);

    int before = periph_unhandled_count(periph);
    ASSERT_EQ(mem_read32(mem, desc->base[0] + 0x084u), 0u);
    mem_write32(mem, desc->base[1] + 0x084u, 1u);
    ASSERT_EQ(periph_unhandled_count(periph), before + 2);
    ASSERT_EQ(mem_unmapped_count(mem), 0u);

    periph_destroy(periph);
    mem_destroy(mem);
}

void run_esp32s3_extmem_tests(void) {
    TEST_SUITE("ESP32-S3 EXTMEM");
    RUN_TEST(esp32s3_extmem_exposes_documented_reset_state);
    RUN_TEST(esp32s3_extmem_application_handoff_enables_cache_buses);
    RUN_TEST(esp32s3_extmem_operations_complete_and_invalidate_code);
    RUN_TEST(peripherals_compose_s3_devices_without_classic_aliases);
    RUN_TEST(peripherals_compose_target_described_s3_uarts);
    RUN_TEST(peripherals_model_target_described_secondary_core_control);
    RUN_TEST(peripherals_model_target_described_rtc_calibration);
}
