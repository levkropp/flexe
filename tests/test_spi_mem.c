/* Target-described SPI-memory controller and NOR flash tests. */
#include "test_helpers.h"
#include "peripherals.h"

#define SPI_CMD_USR             (1u << 18)
#define SPI_USER_MOSI           (1u << 27)
#define SPI_USER_MISO           (1u << 28)
#define SPI_USER_ADDR           (1u << 30)
#define SPI_USER_COMMAND        (1u << 31)

#define CLASSIC_SPI0_BASE       0x3FF43000u
#define CLASSIC_SPI1_BASE       0x3FF42000u
#define CLASSIC_SPI_USER        0x01Cu
#define CLASSIC_SPI_USER1       0x020u
#define CLASSIC_SPI_USER2       0x024u
#define CLASSIC_SPI_MOSI_DLEN   0x028u
#define CLASSIC_SPI_MISO_DLEN   0x02Cu
#define CLASSIC_SPI_PIN         0x034u
#define CLASSIC_SPI_W0          0x080u

#define S3_SPI0_BASE            0x60003000u
#define S3_SPI1_BASE            0x60002000u
#define S3_SPI_CMD              0x000u
#define S3_SPI_ADDR             0x004u
#define S3_SPI_USER             0x018u
#define S3_SPI_USER1            0x01Cu
#define S3_SPI_USER2            0x020u
#define S3_SPI_MOSI_DLEN        0x024u
#define S3_SPI_MISO_DLEN        0x028u
#define S3_SPI_MISC             0x034u
#define S3_SPI_W0               0x058u
#define S3_SPI_DATE             0x3FCu

typedef struct {
    unsigned calls;
    uint32_t addr;
    size_t len;
} spi_mem_invalidate_probe_t;

static void spi_mem_test_invalidate(void *ctx, uint32_t addr, size_t len) {
    spi_mem_invalidate_probe_t *probe = ctx;
    probe->calls++;
    probe->addr = addr;
    probe->len = len;
}

static void spi_user_command(xtensa_mem_t *mem, uint32_t base,
                             uint32_t user_offset, uint32_t user2_offset,
                             uint32_t user, uint8_t opcode) {
    mem_write32(mem, base + user_offset, user);
    mem_write32(mem, base + user2_offset, (7u << 28) | opcode);
    mem_write32(mem, base, SPI_CMD_USR);
}

static void s3_spi_user_command(xtensa_mem_t *mem, uint32_t base,
                                uint32_t user, uint8_t opcode) {
    spi_user_command(mem, base, S3_SPI_USER, S3_SPI_USER2, user, opcode);
}

TEST(spi_mem_uses_target_layouts_and_reports_jedec_id) {
    xtensa_mem_t *classic_mem = mem_create();
    esp32_periph_t *classic = periph_create(classic_mem);
    ASSERT_TRUE(classic_mem != NULL);
    ASSERT_TRUE(classic != NULL);
    if (!classic_mem || !classic) {
        periph_destroy(classic);
        mem_destroy(classic_mem);
        return;
    }

    ASSERT_EQ(mem_read32(classic_mem,
                         CLASSIC_SPI1_BASE + CLASSIC_SPI_USER),
              0x80000040u);
    ASSERT_EQ(mem_read32(classic_mem,
                         CLASSIC_SPI1_BASE + CLASSIC_SPI_USER1),
              0x5C000007u);
    ASSERT_EQ(mem_read32(classic_mem,
                         CLASSIC_SPI1_BASE + CLASSIC_SPI_USER2),
              0x70000000u);
    ASSERT_EQ(mem_read32(classic_mem,
                         CLASSIC_SPI1_BASE + CLASSIC_SPI_PIN),
              0x00000006u);

    mem_write32(classic_mem, CLASSIC_SPI1_BASE + CLASSIC_SPI_MISO_DLEN, 23u);
    spi_user_command(classic_mem, CLASSIC_SPI1_BASE, CLASSIC_SPI_USER,
                     CLASSIC_SPI_USER2,
                     SPI_USER_COMMAND | SPI_USER_MISO, 0x9Fu);
    ASSERT_EQ(mem_read32(classic_mem,
                         CLASSIC_SPI1_BASE + CLASSIC_SPI_W0),
              0x001640C8u);
    ASSERT_EQ(mem_read8(classic_mem, CLASSIC_SPI1_BASE + CLASSIC_SPI_W0),
              0xC8u);
    ASSERT_EQ(mem_read8(classic_mem, CLASSIC_SPI1_BASE + CLASSIC_SPI_W0 + 1u),
              0x40u);
    ASSERT_EQ(mem_read8(classic_mem, CLASSIC_SPI1_BASE + CLASSIC_SPI_W0 + 2u),
              0x16u);
    ASSERT_EQ(periph_unhandled_count(classic), 0);
    periph_destroy(classic);
    mem_destroy(classic_mem);

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


    ASSERT_EQ(mem_read32(mem, S3_SPI1_BASE + 0x018u), 0x80000000u);
    ASSERT_EQ(mem_read32(mem, S3_SPI1_BASE + S3_SPI_USER1), 0x5C000007u);
    ASSERT_EQ(mem_read32(mem, S3_SPI1_BASE + S3_SPI_USER2), 0x70000000u);
    ASSERT_EQ(mem_read32(mem, S3_SPI1_BASE + S3_SPI_MISC), 0x00000002u);

    mem_write32(mem, S3_SPI1_BASE + S3_SPI_MISO_DLEN, 23u);
    s3_spi_user_command(mem, S3_SPI1_BASE,
                        SPI_USER_COMMAND | SPI_USER_MISO, 0x9Fu);
    ASSERT_EQ(mem_read32(mem, S3_SPI1_BASE + S3_SPI_CMD), 0u);
    ASSERT_EQ(mem_read32(mem, S3_SPI1_BASE + S3_SPI_W0), 0x001640C8u);
    ASSERT_EQ(mem_read32(mem, S3_SPI0_BASE + S3_SPI_W0), 0u);
    ASSERT_EQ(mem_read32(mem, S3_SPI1_BASE + S3_SPI_DATE), 0x02101040u);

    const uint32_t offset = 0x12340u;
    mem->flash_data[offset] = 0xC9u;
    mem->flash_data[offset + 1u] = 0x04u;
    mem_write32(mem, S3_SPI1_BASE + S3_SPI_USER1, 23u << 26);
    mem_write32(mem, S3_SPI1_BASE + S3_SPI_ADDR, offset << 8);
    mem_write32(mem, S3_SPI1_BASE + S3_SPI_MISO_DLEN, 15u);
    s3_spi_user_command(mem, S3_SPI1_BASE,
                        SPI_USER_COMMAND | SPI_USER_ADDR | SPI_USER_MISO,
                        0x03u);
    ASSERT_EQ(mem_read32(mem, S3_SPI1_BASE + S3_SPI_W0) & 0xFFFFu,
              0x04C9u);
    ASSERT_EQ(periph_unhandled_count(periph), 0);

    /* Reserved offsets stay explicit instead of becoming a silent register
     * file merely because the controller owns the surrounding page. */
    ASSERT_EQ(mem_read32(mem, S3_SPI1_BASE + 0x300u), 0u);
    ASSERT_EQ(periph_unhandled_count(periph), 1);

    periph_destroy(periph);
    mem_destroy(mem);
}

TEST(spi_mem_classic_routes_psram_by_chip_select_and_wire_phases) {
    xtensa_mem_t *mem = mem_create();
    esp32_periph_t *periph = periph_create(mem);
    ASSERT_TRUE(mem != NULL);
    ASSERT_TRUE(periph != NULL);
    if (!mem || !periph) {
        periph_destroy(periph);
        mem_destroy(mem);
        return;
    }

    /* Match ESP-IDF's PSRAM command helper: disable CS0/CS2 and enable CS1. */
    mem_write32(mem, CLASSIC_SPI1_BASE + CLASSIC_SPI_PIN, 0x5u);

    /* Old 32-Mbit devices receive EXIT_QMODE as MOSI data. At 40 MHz the
     * DCLK workaround inserts one complete zero byte ahead of 0xF5. */
    mem_write32(mem, CLASSIC_SPI1_BASE + CLASSIC_SPI_USER, SPI_USER_MOSI);
    mem_write32(mem, CLASSIC_SPI1_BASE + CLASSIC_SPI_MOSI_DLEN, 15u);
    mem_write32(mem, CLASSIC_SPI1_BASE + CLASSIC_SPI_W0, 0x0000F500u);
    mem_write32(mem, CLASSIC_SPI1_BASE, SPI_CMD_USR);
    ASSERT_EQ(periph_unhandled_count(periph), 0);

    /* The same workaround expresses two idle clocks as a short zero command
     * phase, then carries the real 0x9F opcode in a 32-bit address phase. */
    mem_write32(mem, CLASSIC_SPI1_BASE + CLASSIC_SPI_USER,
                SPI_USER_COMMAND | SPI_USER_ADDR | SPI_USER_MISO);
    mem_write32(mem, CLASSIC_SPI1_BASE + CLASSIC_SPI_USER2, 1u << 28);
    mem_write32(mem, CLASSIC_SPI1_BASE + CLASSIC_SPI_USER1, 31u << 26);
    mem_write32(mem, CLASSIC_SPI1_BASE + 0x004u, 0x9F000000u);
    mem_write32(mem, CLASSIC_SPI1_BASE + CLASSIC_SPI_MISO_DLEN, 63u);
    mem_write32(mem, CLASSIC_SPI1_BASE, SPI_CMD_USR);
    ASSERT_EQ(mem_read32(mem, CLASSIC_SPI1_BASE + CLASSIC_SPI_W0),
              0x00205D0Du);
    ASSERT_EQ(mem_read32(mem, CLASSIC_SPI1_BASE + CLASSIC_SPI_W0 + 4u),
              0u);

    /* ENTER_QMODE is likewise packed into the address phase. */
    mem_write32(mem, CLASSIC_SPI1_BASE + CLASSIC_SPI_USER,
                SPI_USER_COMMAND | SPI_USER_ADDR);
    mem_write32(mem, CLASSIC_SPI1_BASE + CLASSIC_SPI_USER2, 1u << 28);
    mem_write32(mem, CLASSIC_SPI1_BASE + CLASSIC_SPI_USER1, 7u << 26);
    mem_write32(mem, CLASSIC_SPI1_BASE + 0x004u, 0x35000000u);
    mem_write32(mem, CLASSIC_SPI1_BASE, SPI_CMD_USR);

    /* Address-packed QPI reads and writes reach the existing PSRAM backing
     * and wrap at its physical 4-MiB boundary. */
    const uint32_t psram_size =
        mem_backing_size(mem, FLEXE_MEM_PSRAM);
    const uint32_t offset = psram_size - 2u;
    mem_write32(mem, CLASSIC_SPI1_BASE + CLASSIC_SPI_USER,
                SPI_USER_ADDR | SPI_USER_MOSI);
    mem_write32(mem, CLASSIC_SPI1_BASE + CLASSIC_SPI_USER1, 31u << 26);
    mem_write32(mem, CLASSIC_SPI1_BASE + 0x004u,
                (0x38u << 24) | offset);
    mem_write32(mem, CLASSIC_SPI1_BASE + CLASSIC_SPI_MOSI_DLEN, 31u);
    mem_write32(mem, CLASSIC_SPI1_BASE + CLASSIC_SPI_W0, 0x44332211u);
    mem_write32(mem, CLASSIC_SPI1_BASE, SPI_CMD_USR);
    ASSERT_EQ(mem->psram[offset], 0x11u);
    ASSERT_EQ(mem->psram[offset + 1u], 0x22u);
    ASSERT_EQ(mem->psram[0], 0x33u);
    ASSERT_EQ(mem->psram[1], 0x44u);

    mem_write32(mem, CLASSIC_SPI1_BASE + CLASSIC_SPI_USER,
                SPI_USER_ADDR | SPI_USER_MISO);
    mem_write32(mem, CLASSIC_SPI1_BASE + 0x004u,
                (0xEBu << 24) | offset);
    mem_write32(mem, CLASSIC_SPI1_BASE + CLASSIC_SPI_MISO_DLEN, 31u);
    mem_write32(mem, CLASSIC_SPI1_BASE, SPI_CMD_USR);
    ASSERT_EQ(mem_read32(mem, CLASSIC_SPI1_BASE + CLASSIC_SPI_W0),
              0x44332211u);

    /* Restoring CS0 routes the same opcode back to NOR flash. */
    mem_write32(mem, CLASSIC_SPI1_BASE + CLASSIC_SPI_PIN, 0x6u);
    mem_write32(mem, CLASSIC_SPI1_BASE + CLASSIC_SPI_MISO_DLEN, 23u);
    spi_user_command(mem, CLASSIC_SPI1_BASE, CLASSIC_SPI_USER,
                     CLASSIC_SPI_USER2,
                     SPI_USER_COMMAND | SPI_USER_MISO, 0x9Fu);
    ASSERT_EQ(mem_read32(mem, CLASSIC_SPI1_BASE + CLASSIC_SPI_W0),
              0x001640C8u);
    ASSERT_EQ(periph_unhandled_count(periph), 0);

    periph_destroy(periph);
    mem_destroy(mem);
}

TEST(spi_mem_s3_program_erase_and_shared_mmu_invalidation) {
    const flexe_target_desc_t *s3 =
        flexe_target_by_id(FLEXE_TARGET_ESP32S3);
    xtensa_mem_t *mem = mem_create_for_target(s3);
    esp32_periph_t *periph = periph_create(mem);
    xtensa_cpu_t cpu;
    spi_mem_invalidate_probe_t probe = {0};
    ASSERT_TRUE(mem != NULL);
    ASSERT_TRUE(periph != NULL);
    if (!mem || !periph) {
        periph_destroy(periph);
        mem_destroy(mem);
        return;
    }

    xtensa_cpu_init_for_target(&cpu, s3);
    cpu.mem = mem;
    cpu.code_invalidate = spi_mem_test_invalidate;
    cpu.code_invalidate_ctx = &probe;
    periph_attach_cpus(periph, &cpu, NULL);

    /* Virtual page 3 aliases physical flash page 2. */
    mem_write32(mem, s3->flash_mmu.table_base[0] + 3u * 4u, 2u);
    probe.calls = 0u;
    const uint32_t offset = 0x20020u;
    mem->flash_data[offset] = 0xF0u;
    mem->flash_insn[offset] = 0xF0u;
    mem_write32(mem, S3_SPI1_BASE + S3_SPI_USER1, 23u << 26);
    mem_write32(mem, S3_SPI1_BASE + S3_SPI_ADDR, offset << 8);
    mem_write32(mem, S3_SPI1_BASE + S3_SPI_MOSI_DLEN, 7u);
    mem_write32(mem, S3_SPI1_BASE + S3_SPI_W0, 0xAAu);

    /* NOR program/erase operations require the device's write-enable latch. */
    s3_spi_user_command(mem, S3_SPI1_BASE,
                        SPI_USER_COMMAND | SPI_USER_ADDR | SPI_USER_MOSI,
                        0x02u);
    ASSERT_EQ(mem->flash_data[offset], 0xF0u);
    /* Both controller hosts reach one physical flash device: WREN on SPI0
     * must arm a page program launched through SPI1. */
    s3_spi_user_command(mem, S3_SPI0_BASE, SPI_USER_COMMAND, 0x06u);
    s3_spi_user_command(mem, S3_SPI1_BASE,
                        SPI_USER_COMMAND | SPI_USER_ADDR | SPI_USER_MOSI,
                        0x02u);
    ASSERT_EQ(mem->flash_data[offset], 0xA0u);
    ASSERT_EQ(mem->flash_insn[offset], 0xA0u);
    ASSERT_EQ(probe.calls, 1u);
    ASSERT_EQ(probe.addr, 0x42030000u);
    ASSERT_EQ(probe.len, 0x10000u);

    s3_spi_user_command(mem, S3_SPI1_BASE,
                        SPI_USER_COMMAND | SPI_USER_MISO, 0x05u);
    ASSERT_EQ(mem_read32(mem, S3_SPI1_BASE + S3_SPI_W0) & (1u << 1), 0u);
    s3_spi_user_command(mem, S3_SPI0_BASE, SPI_USER_COMMAND, 0x06u);
    s3_spi_user_command(mem, S3_SPI1_BASE,
                        SPI_USER_COMMAND | SPI_USER_ADDR, 0x20u);
    ASSERT_EQ(mem->flash_data[offset], 0xFFu);
    ASSERT_EQ(mem->flash_insn[offset], 0xFFu);
    ASSERT_EQ(probe.calls, 2u);
    ASSERT_EQ(periph_unhandled_count(periph), 0);

    periph_destroy(periph);
    mem_destroy(mem);
}

TEST(spi_mem_reports_allocated_flash_capacity) {
    const flexe_target_desc_t *s3 =
        flexe_target_by_id(FLEXE_TARGET_ESP32S3);
    xtensa_mem_t *mem = mem_create_for_target_with_flash(s3, 0x00800000u);
    esp32_periph_t *periph = periph_create(mem);
    ASSERT_TRUE(mem != NULL);
    ASSERT_TRUE(periph != NULL);
    if (!mem || !periph) {
        periph_destroy(periph);
        mem_destroy(mem);
        return;
    }

    mem_write32(mem, S3_SPI1_BASE + S3_SPI_MISO_DLEN, 23u);
    s3_spi_user_command(mem, S3_SPI1_BASE,
                        SPI_USER_COMMAND | SPI_USER_MISO, 0x9Fu);
    ASSERT_EQ(mem_read32(mem, S3_SPI1_BASE + S3_SPI_W0), 0x001740C8u);
    ASSERT_EQ(periph_unhandled_count(periph), 0u);
    periph_destroy(periph);
    mem_destroy(mem);
}

void run_spi_mem_tests(void) {
    TEST_SUITE("Target SPI memory controller");
    RUN_TEST(spi_mem_uses_target_layouts_and_reports_jedec_id);
    RUN_TEST(spi_mem_classic_routes_psram_by_chip_select_and_wire_phases);
    RUN_TEST(spi_mem_s3_program_erase_and_shared_mmu_invalidation);
    RUN_TEST(spi_mem_reports_allocated_flash_capacity);
}
