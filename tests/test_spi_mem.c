/* Target-described SPI-memory controller and NOR flash tests. */
#include "test_helpers.h"
#include "peripherals.h"

#define SPI_CMD_USR             (1u << 18)
#define SPI_CMD_FLASH_CE        (1u << 22)
#define SPI_CMD_FLASH_SE        (1u << 24)
#define SPI_CMD_FLASH_PP        (1u << 25)
#define SPI_CMD_FLASH_WREN      (1u << 30)
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

static void s3_opi_command(xtensa_mem_t *mem, uint32_t base,
                           uint32_t user, uint16_t opcode,
                           uint32_t address) {
    mem_write32(mem, base + S3_SPI_USER, user | SPI_USER_COMMAND |
                                         SPI_USER_ADDR);
    mem_write32(mem, base + S3_SPI_USER1, 31u << 26);
    mem_write32(mem, base + S3_SPI_USER2, (15u << 28) | opcode);
    mem_write32(mem, base + S3_SPI_ADDR, address);
    mem_write32(mem, base + S3_SPI_CMD, SPI_CMD_USR);
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
    mem_write32(mem, S3_SPI1_BASE + S3_SPI_ADDR, offset);
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

TEST(spi_mem_unpopulated_chip_select_clocks_any_command_width) {
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

    /* Octal PSRAM mode-register traffic is 16-bit command + 32-bit address.
     * With no PSRAM attached to S3 CS1, the wires clock but nobody drives
     * MISO. The same command on CS0 must still be rejected by NOR flash. */
    mem_write32(mem, S3_SPI1_BASE + S3_SPI_MISC, 0x1u);
    mem_write32(mem, S3_SPI1_BASE + S3_SPI_USER,
                SPI_USER_COMMAND | SPI_USER_ADDR | SPI_USER_MISO);
    mem_write32(mem, S3_SPI1_BASE + S3_SPI_USER1, 31u << 26);
    mem_write32(mem, S3_SPI1_BASE + S3_SPI_USER2,
                (15u << 28) | 0x4040u);
    mem_write32(mem, S3_SPI1_BASE + S3_SPI_MISO_DLEN, 15u);
    mem_write32(mem, S3_SPI1_BASE + S3_SPI_W0, 0u);
    mem_write32(mem, S3_SPI1_BASE + S3_SPI_CMD, SPI_CMD_USR);
    ASSERT_EQ(mem_read32(mem, S3_SPI1_BASE + S3_SPI_W0), 0xFFFFFFFFu);
    ASSERT_EQ(periph_unhandled_count(periph), 0u);

    mem_write32(mem, S3_SPI1_BASE + S3_SPI_USER,
                SPI_USER_COMMAND | SPI_USER_ADDR | SPI_USER_MOSI);
    mem_write32(mem, S3_SPI1_BASE + S3_SPI_USER2,
                (15u << 28) | 0xC0C0u);
    mem_write32(mem, S3_SPI1_BASE + S3_SPI_MOSI_DLEN, 15u);
    mem_write32(mem, S3_SPI1_BASE + S3_SPI_W0, 0x28u);
    mem_write32(mem, S3_SPI1_BASE + S3_SPI_CMD, SPI_CMD_USR);
    ASSERT_EQ(mem_read32(mem, S3_SPI1_BASE + S3_SPI_W0), 0x28u);
    ASSERT_EQ(periph_unhandled_count(periph), 0u);

    mem_write32(mem, S3_SPI1_BASE + S3_SPI_MISC, 0x2u);
    mem_write32(mem, S3_SPI1_BASE + S3_SPI_CMD, SPI_CMD_USR);
    ASSERT_EQ(periph_unhandled_count(periph), 1u);
    periph_destroy(periph);
    mem_destroy(mem);

    /* The classic controller has an unpopulated CS2 as well. */
    mem = mem_create();
    periph = periph_create(mem);
    ASSERT_TRUE(mem != NULL);
    ASSERT_TRUE(periph != NULL);
    if (!mem || !periph) {
        periph_destroy(periph);
        mem_destroy(mem);
        return;
    }
    mem_write32(mem, CLASSIC_SPI1_BASE + CLASSIC_SPI_PIN, 0x3u);
    mem_write32(mem, CLASSIC_SPI1_BASE + CLASSIC_SPI_USER,
                SPI_USER_COMMAND | SPI_USER_MISO);
    mem_write32(mem, CLASSIC_SPI1_BASE + CLASSIC_SPI_USER2,
                (15u << 28) | 0x4040u);
    mem_write32(mem, CLASSIC_SPI1_BASE + CLASSIC_SPI_MISO_DLEN, 15u);
    mem_write32(mem, CLASSIC_SPI1_BASE, SPI_CMD_USR);
    ASSERT_EQ(mem_read32(mem, CLASSIC_SPI1_BASE + CLASSIC_SPI_W0),
              0xFFFFFFFFu);
    ASSERT_EQ(periph_unhandled_count(periph), 0u);
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
    mem_write32(mem, S3_SPI1_BASE + S3_SPI_ADDR, offset);
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

TEST(spi_mem_sfdp_matches_advertised_gd25q_c_profiles) {
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

    /* Use the stock S3 bootloader's 8-bit command and 24-bit address phases;
     * four-byte reads also cover parameter-word and arbitrary byte offsets.
     * Both SPI hosts share one NOR but keep independent data buffers. */
    mem_write32(mem, S3_SPI1_BASE + S3_SPI_USER1, 23u << 26);
    mem_write32(mem, S3_SPI1_BASE + S3_SPI_MISO_DLEN, 31u);
    const uint32_t offsets[] = { 0u, 4u, 0x10u, 0x2Eu,
                                 0x34u, 0x4Cu, 0x60u, 0x6Cu };
    const uint32_t expected[] = { 0x50444653u, 0xFF010100u,
                                  0x030100C8u, 0x20E5FFFFu,
                                  0x01FFFFFFu, 0x520F200Cu,
                                  0x27003600u, 0xFFFFFFFFu };
    for (size_t i = 0; i < sizeof(offsets) / sizeof(offsets[0]); i++) {
        mem_write32(mem, S3_SPI1_BASE + S3_SPI_ADDR, offsets[i]);
        s3_spi_user_command(mem, S3_SPI1_BASE,
                            SPI_USER_COMMAND | SPI_USER_ADDR | SPI_USER_MISO,
                            0x5Au);
        ASSERT_EQ(mem_read32(mem, S3_SPI1_BASE + S3_SPI_W0), expected[i]);
    }
    ASSERT_EQ(mem_read32(mem, S3_SPI0_BASE + S3_SPI_W0), 0u);
    ASSERT_EQ(periph_unhandled_count(periph), 0u);
    periph_destroy(periph);
    mem_destroy(mem);

    /* The documented GD25Q64C has the same parameter bytes except for its
     * 64-Mbit density DWORD. Check the same reads through SPI0 as well as
     * the chip ID that selects this profile. */
    mem = mem_create_for_target_with_flash(s3, 0x00800000u);
    periph = periph_create(mem);
    ASSERT_TRUE(mem != NULL);
    ASSERT_TRUE(periph != NULL);
    if (!mem || !periph) {
        periph_destroy(periph);
        mem_destroy(mem);
        return;
    }
    ASSERT_EQ(mem_flash_jedec_id(mem), 0x001740C8u);
    mem_write32(mem, S3_SPI0_BASE + S3_SPI_USER1, 23u << 26);
    mem_write32(mem, S3_SPI0_BASE + S3_SPI_MISO_DLEN, 31u);
    for (size_t i = 0; i < sizeof(offsets) / sizeof(offsets[0]); i++) {
        mem_write32(mem, S3_SPI0_BASE + S3_SPI_ADDR, offsets[i]);
        s3_spi_user_command(mem, S3_SPI0_BASE,
                            SPI_USER_COMMAND | SPI_USER_ADDR | SPI_USER_MISO,
                            0x5Au);
        ASSERT_EQ(mem_read32(mem, S3_SPI0_BASE + S3_SPI_W0),
                  offsets[i] == 0x34u ? 0x03FFFFFFu : expected[i]);
    }
    ASSERT_EQ(periph_unhandled_count(periph), 0u);
    periph_destroy(periph);
    mem_destroy(mem);

    /* An unprofiled 16 MiB device must not inherit either density, even
     * though its generated JEDEC manufacturer/family bytes still match. */
    mem = mem_create_for_target_with_flash(s3, 0x01000000u);
    periph = periph_create(mem);
    ASSERT_TRUE(mem != NULL);
    ASSERT_TRUE(periph != NULL);
    if (!mem || !periph) {
        periph_destroy(periph);
        mem_destroy(mem);
        return;
    }
    ASSERT_EQ(mem_flash_jedec_id(mem), 0x001840C8u);
    mem_write32(mem, S3_SPI1_BASE + S3_SPI_USER1, 23u << 26);
    mem_write32(mem, S3_SPI1_BASE + S3_SPI_MISO_DLEN, 31u);
    mem_write32(mem, S3_SPI1_BASE + S3_SPI_ADDR, 0u);
    mem_write32(mem, S3_SPI1_BASE + S3_SPI_W0, 0xDEADBEEFu);
    s3_spi_user_command(mem, S3_SPI1_BASE,
                        SPI_USER_COMMAND | SPI_USER_ADDR | SPI_USER_MISO,
                        0x5Au);
    ASSERT_EQ(mem_read32(mem, S3_SPI1_BASE + S3_SPI_W0), 0xDEADBEEFu);
    ASSERT_EQ(periph_unhandled_count(periph), 1u);
    periph_destroy(periph);
    mem_destroy(mem);
}

TEST(spi_mem_gd25q32c_status_survives_flash_reset) {
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

    mem_write32(mem, S3_SPI1_BASE + S3_SPI_MISO_DLEN, 7u);
    s3_spi_user_command(mem, S3_SPI1_BASE,
                        SPI_USER_COMMAND | SPI_USER_MISO, 0x15u);
    ASSERT_EQ(mem_read32(mem, S3_SPI1_BASE + S3_SPI_W0) & 0xFFu, 0x20u);

    mem_write32(mem, S3_SPI1_BASE + S3_SPI_MOSI_DLEN, 7u);
    mem_write32(mem, S3_SPI1_BASE + S3_SPI_W0, 0x02u);
    s3_spi_user_command(mem, S3_SPI1_BASE,
                        SPI_USER_COMMAND | SPI_USER_MOSI, 0x31u);
    s3_spi_user_command(mem, S3_SPI1_BASE,
                        SPI_USER_COMMAND | SPI_USER_MISO, 0x35u);
    ASSERT_EQ(mem_read32(mem, S3_SPI1_BASE + S3_SPI_W0) & 0xFFu, 0u);

    s3_spi_user_command(mem, S3_SPI0_BASE, SPI_USER_COMMAND, 0x06u);
    mem_write32(mem, S3_SPI1_BASE + S3_SPI_W0, 0x02u);
    s3_spi_user_command(mem, S3_SPI1_BASE,
                        SPI_USER_COMMAND | SPI_USER_MOSI, 0x31u);
    s3_spi_user_command(mem, S3_SPI0_BASE,
                        SPI_USER_COMMAND | SPI_USER_MISO, 0x35u);
    ASSERT_EQ(mem_read32(mem, S3_SPI0_BASE + S3_SPI_W0) & 0xFFu, 0x02u);

    s3_spi_user_command(mem, S3_SPI0_BASE, SPI_USER_COMMAND, 0x06u);
    s3_spi_user_command(mem, S3_SPI1_BASE, SPI_USER_COMMAND, 0x66u);
    s3_spi_user_command(mem, S3_SPI1_BASE, SPI_USER_COMMAND, 0x99u);
    s3_spi_user_command(mem, S3_SPI1_BASE,
                        SPI_USER_COMMAND | SPI_USER_MISO, 0x05u);
    ASSERT_EQ(mem_read32(mem, S3_SPI1_BASE + S3_SPI_W0) & (1u << 1), 0u);
    s3_spi_user_command(mem, S3_SPI1_BASE,
                        SPI_USER_COMMAND | SPI_USER_MISO, 0x35u);
    ASSERT_EQ(mem_read32(mem, S3_SPI1_BASE + S3_SPI_W0) & 0xFFu, 0x02u);
    s3_spi_user_command(mem, S3_SPI1_BASE,
                        SPI_USER_COMMAND | SPI_USER_MISO, 0x15u);
    ASSERT_EQ(mem_read32(mem, S3_SPI1_BASE + S3_SPI_W0) & 0xFFu, 0x20u);
    ASSERT_EQ(periph_unhandled_count(periph), 0u);
    periph_destroy(periph);
    mem_destroy(mem);
}

TEST(spi_mem_gd25q32c_protection_blocks_whole_operations) {
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

    /* BP4/BP0 protect the top 4 KiB. WRSR on SPI1 takes WREN from SPI0. */
    mem_write32(mem, S3_SPI1_BASE + S3_SPI_MOSI_DLEN, 7u);
    mem_write32(mem, S3_SPI1_BASE + S3_SPI_W0, 0x44u);
    s3_spi_user_command(mem, S3_SPI0_BASE, SPI_USER_COMMAND, 0x06u);
    s3_spi_user_command(mem, S3_SPI1_BASE,
                        SPI_USER_COMMAND | SPI_USER_MOSI, 0x01u);
    s3_spi_user_command(mem, S3_SPI0_BASE,
                        SPI_USER_COMMAND | SPI_USER_MISO, 0x05u);
    ASSERT_EQ(mem_read32(mem, S3_SPI0_BASE + S3_SPI_W0) & 0xFFu, 0x44u);

    const uint32_t top = 0x3FF010u;
    const uint32_t below = 0x3FE010u;
    mem->flash_data[top] = mem->flash_insn[top] = 0xAAu;
    mem->flash_data[below] = mem->flash_insn[below] = 0xAAu;
    mem_write32(mem, S3_SPI1_BASE + S3_SPI_USER1, 23u << 26);
    mem_write32(mem, S3_SPI1_BASE + S3_SPI_MOSI_DLEN, 7u);
    mem_write32(mem, S3_SPI1_BASE + S3_SPI_W0, 0u);
    mem_write32(mem, S3_SPI1_BASE + S3_SPI_ADDR, top);
    s3_spi_user_command(mem, S3_SPI0_BASE, SPI_USER_COMMAND, 0x06u);
    s3_spi_user_command(mem, S3_SPI1_BASE,
                        SPI_USER_COMMAND | SPI_USER_ADDR | SPI_USER_MOSI,
                        0x02u);
    ASSERT_EQ(mem->flash_data[top], 0xAAu);
    ASSERT_EQ(mem->flash_insn[top], 0xAAu);
    s3_spi_user_command(mem, S3_SPI0_BASE,
                        SPI_USER_COMMAND | SPI_USER_MISO, 0x05u);
    ASSERT_EQ(mem_read32(mem, S3_SPI0_BASE + S3_SPI_W0) & 0x02u, 0u);

    /* A 32 KiB erase containing a protected 4 KiB sector is rejected in
     * full, including its unprotected portion. Chip erase is rejected too. */
    mem_write32(mem, S3_SPI1_BASE + S3_SPI_ADDR, below);
    s3_spi_user_command(mem, S3_SPI0_BASE, SPI_USER_COMMAND, 0x06u);
    s3_spi_user_command(mem, S3_SPI1_BASE,
                        SPI_USER_COMMAND | SPI_USER_ADDR, 0x52u);
    ASSERT_EQ(mem->flash_data[below], 0xAAu);
    s3_spi_user_command(mem, S3_SPI0_BASE, SPI_USER_COMMAND, 0x06u);
    s3_spi_user_command(mem, S3_SPI1_BASE,
                        SPI_USER_COMMAND | SPI_USER_ADDR, 0x20u);
    ASSERT_EQ(mem->flash_data[below], 0xFFu);
    ASSERT_EQ(mem->flash_data[top], 0xAAu);
    mem->flash_data[below] = mem->flash_insn[below] = 0xAAu;
    s3_spi_user_command(mem, S3_SPI0_BASE, SPI_USER_COMMAND, 0x06u);
    s3_spi_user_command(mem, S3_SPI1_BASE, SPI_USER_COMMAND, 0xC7u);
    ASSERT_EQ(mem->flash_data[top], 0xAAu);
    ASSERT_EQ(mem->flash_data[below], 0xAAu);

    /* Dedicated controller commands must use the same NOR protection gate. */
    mem_write32(mem, S3_SPI1_BASE + S3_SPI_ADDR, top);
    mem_write32(mem, S3_SPI1_BASE + S3_SPI_W0, 0u);
    mem_write32(mem, S3_SPI1_BASE + S3_SPI_CMD, SPI_CMD_FLASH_WREN);
    mem_write32(mem, S3_SPI1_BASE + S3_SPI_CMD, SPI_CMD_FLASH_PP);
    ASSERT_EQ(mem->flash_data[top], 0xAAu);
    mem_write32(mem, S3_SPI1_BASE + S3_SPI_CMD, SPI_CMD_FLASH_WREN);
    mem_write32(mem, S3_SPI1_BASE + S3_SPI_CMD, SPI_CMD_FLASH_SE);
    ASSERT_EQ(mem->flash_data[top], 0xAAu);
    mem_write32(mem, S3_SPI1_BASE + S3_SPI_CMD, SPI_CMD_FLASH_WREN);
    mem_write32(mem, S3_SPI1_BASE + S3_SPI_CMD, SPI_CMD_FLASH_CE);
    ASSERT_EQ(mem->flash_data[below], 0xAAu);

    /* CMP complements the range: only the top 4 KiB can now change. */
    mem_write32(mem, S3_SPI1_BASE + S3_SPI_MOSI_DLEN, 15u);
    mem_write32(mem, S3_SPI1_BASE + S3_SPI_W0, 0x4044u);
    s3_spi_user_command(mem, S3_SPI0_BASE, SPI_USER_COMMAND, 0x06u);
    s3_spi_user_command(mem, S3_SPI1_BASE,
                        SPI_USER_COMMAND | SPI_USER_MOSI, 0x01u);
    s3_spi_user_command(mem, S3_SPI1_BASE,
                        SPI_USER_COMMAND | SPI_USER_MISO, 0x35u);
    ASSERT_EQ(mem_read32(mem, S3_SPI1_BASE + S3_SPI_W0) & 0x40u, 0x40u);
    s3_spi_user_command(mem, S3_SPI1_BASE, SPI_USER_COMMAND, 0x66u);
    s3_spi_user_command(mem, S3_SPI1_BASE, SPI_USER_COMMAND, 0x99u);
    flexe_spi_mem_nor_state_t chip;
    periph_flash_chip_snapshot(periph, &chip);
    periph_flash_chip_restore(periph, &chip, true);
    periph_flash_chip_snapshot(periph, &chip);
    ASSERT_EQ(chip.status[0] & 0x7Cu, 0x44u);
    ASSERT_EQ(chip.status[1] & 0x40u, 0x40u);
    mem_write32(mem, S3_SPI1_BASE + S3_SPI_MOSI_DLEN, 7u);
    mem_write32(mem, S3_SPI1_BASE + S3_SPI_W0, 0u);
    mem_write32(mem, S3_SPI1_BASE + S3_SPI_ADDR, below);
    s3_spi_user_command(mem, S3_SPI0_BASE, SPI_USER_COMMAND, 0x06u);
    s3_spi_user_command(mem, S3_SPI1_BASE,
                        SPI_USER_COMMAND | SPI_USER_ADDR | SPI_USER_MOSI,
                        0x02u);
    ASSERT_EQ(mem->flash_data[below], 0xAAu);
    mem_write32(mem, S3_SPI1_BASE + S3_SPI_ADDR, top);
    s3_spi_user_command(mem, S3_SPI0_BASE, SPI_USER_COMMAND, 0x06u);
    s3_spi_user_command(mem, S3_SPI1_BASE,
                        SPI_USER_COMMAND | SPI_USER_ADDR | SPI_USER_MOSI,
                        0x02u);
    ASSERT_EQ(mem->flash_data[top], 0u);
    ASSERT_EQ(mem->flash_insn[top], 0u);
    ASSERT_EQ(periph_unhandled_count(periph), 0u);
    periph_destroy(periph);
    mem_destroy(mem);
}

TEST(spi_mem_gd25q32c_protection_table_boundaries) {
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

    /* Address pairs straddle the GigaDevice Tables 1.0/1.1 boundaries. */
    static const struct {
        uint8_t bp;
        bool cmp;
        uint32_t blocked;
        uint32_t writable;
    } cases[] = {
        {0x00u, false, UINT32_MAX, 0x000000u},
        {0x00u, true, 0x000000u, UINT32_MAX},
        {0x07u, false, 0x3FFFFFu, UINT32_MAX},
        {0x07u, true, UINT32_MAX, 0x3FFFFFu},
        {0x01u, false, 0x3F0000u, 0x3EFFFFu},
        {0x09u, false, 0x00FFFFu, 0x010000u},
        {0x11u, false, 0x3FF000u, 0x3FEFFFu},
        {0x19u, false, 0x000FFFu, 0x001000u},
        {0x16u, false, 0x3F8000u, 0x3F7FFFu},
        {0x1Eu, false, 0x007FFFu, 0x008000u},
        {0x06u, false, 0x200000u, 0x1FFFFFu},
        {0x0Eu, false, 0x1FFFFFu, 0x200000u},
        {0x01u, true, 0x3EFFFFu, 0x3F0000u},
        {0x09u, true, 0x010000u, 0x00FFFFu},
        {0x11u, true, 0x3FEFFFu, 0x3FF000u},
        {0x19u, true, 0x001000u, 0x000FFFu},
    };
    mem_write32(mem, S3_SPI1_BASE + S3_SPI_USER1, 23u << 26);
    mem_write32(mem, S3_SPI1_BASE + S3_SPI_MOSI_DLEN, 7u);
    for (size_t i = 0; i < sizeof(cases) / sizeof(cases[0]); i++) {
        flexe_spi_mem_nor_state_t chip = {0};
        chip.status[0] = cases[i].bp << 2;
        chip.status[1] = cases[i].cmp ? 0x40u : 0u;
        chip.status[2] = 0x20u;
        periph_flash_chip_restore(periph, &chip, false);
        const uint32_t addresses[] = {cases[i].blocked, cases[i].writable};
        for (unsigned kind = 0; kind < 2u; kind++) {
            uint32_t address = addresses[kind];
            if (address == UINT32_MAX) continue;
            mem->flash_data[address] = mem->flash_insn[address] = 0xFFu;
            mem_write32(mem, S3_SPI1_BASE + S3_SPI_ADDR, address);
            mem_write32(mem, S3_SPI1_BASE + S3_SPI_W0, 0u);
            s3_spi_user_command(mem, S3_SPI0_BASE, SPI_USER_COMMAND, 0x06u);
            s3_spi_user_command(mem, S3_SPI1_BASE,
                                SPI_USER_COMMAND | SPI_USER_ADDR |
                                    SPI_USER_MOSI, 0x02u);
            uint8_t expected = kind == 0u ? 0xFFu : 0u;
            ASSERT_EQ(mem->flash_data[address], expected);
            ASSERT_EQ(mem->flash_insn[address], expected);
        }
    }
    ASSERT_EQ(periph_unhandled_count(periph), 0u);
    periph_destroy(periph);
    mem_destroy(mem);
}

TEST(spi_mem_page_program_wraps_within_physical_page) {
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
    const uint32_t offset = 0x12FFu;
    mem->flash_data[0x1200u] = mem->flash_insn[0x1200u] = 0xFFu;
    mem->flash_data[0x1300u] = mem->flash_insn[0x1300u] = 0x5Au;
    mem_write32(mem, S3_SPI1_BASE + S3_SPI_USER1, 23u << 26);
    mem_write32(mem, S3_SPI1_BASE + S3_SPI_ADDR, offset);
    mem_write32(mem, S3_SPI1_BASE + S3_SPI_MOSI_DLEN, 15u);
    mem_write32(mem, S3_SPI1_BASE + S3_SPI_W0, 0x0000AABBu);
    s3_spi_user_command(mem, S3_SPI1_BASE, SPI_USER_COMMAND, 0x06u);
    s3_spi_user_command(mem, S3_SPI1_BASE,
                        SPI_USER_COMMAND | SPI_USER_ADDR | SPI_USER_MOSI,
                        0x02u);
    ASSERT_EQ(mem->flash_data[offset], 0xBBu);
    ASSERT_EQ(mem->flash_data[0x1200u], 0xAAu);
    ASSERT_EQ(mem->flash_data[0x1300u], 0x5Au);
    ASSERT_EQ(mem->flash_insn[0x1200u], 0xAAu);
    ASSERT_EQ(mem->flash_insn[0x1300u], 0x5Au);
    ASSERT_EQ(periph_unhandled_count(periph), 0u);
    periph_destroy(periph);
    mem_destroy(mem);
}

TEST(spi_mem_other_flash_profiles_reject_unknown_protection_layout) {
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

    /* C8 40 17 is not the profiled GD25Q32C. A nonzero BP bit must not
     * silently write through its unknown protection region. */
    flexe_spi_mem_nor_state_t chip = {0};
    chip.status[0] = 0x04u;
    periph_flash_chip_restore(periph, &chip, false);
    const uint32_t offset = 0x1234u;
    mem->flash_data[offset] = mem->flash_insn[offset] = 0xAAu;
    mem_write32(mem, S3_SPI1_BASE + S3_SPI_USER1, 23u << 26);
    mem_write32(mem, S3_SPI1_BASE + S3_SPI_ADDR, offset);
    mem_write32(mem, S3_SPI1_BASE + S3_SPI_MOSI_DLEN, 7u);
    mem_write32(mem, S3_SPI1_BASE + S3_SPI_W0, 0u);
    s3_spi_user_command(mem, S3_SPI0_BASE, SPI_USER_COMMAND, 0x06u);
    s3_spi_user_command(mem, S3_SPI1_BASE,
                        SPI_USER_COMMAND | SPI_USER_ADDR | SPI_USER_MOSI,
                        0x02u);
    ASSERT_EQ(mem->flash_data[offset], 0xAAu);
    ASSERT_EQ(periph_unhandled_count(periph), 1u);
    mem_write32(mem, S3_SPI1_BASE + S3_SPI_CMD, SPI_CMD_FLASH_WREN);
    mem_write32(mem, S3_SPI1_BASE + S3_SPI_CMD, SPI_CMD_FLASH_SE);
    ASSERT_EQ(mem->flash_data[offset], 0xAAu);
    ASSERT_EQ(periph_unhandled_count(periph), 2u);

    /* The same larger backing still supports unprotected NOR writes. */
    chip.status[0] = 0u;
    periph_flash_chip_restore(periph, &chip, false);
    s3_spi_user_command(mem, S3_SPI0_BASE, SPI_USER_COMMAND, 0x06u);
    s3_spi_user_command(mem, S3_SPI1_BASE,
                        SPI_USER_COMMAND | SPI_USER_ADDR | SPI_USER_MOSI,
                        0x02u);
    ASSERT_EQ(mem->flash_data[offset], 0u);
    ASSERT_EQ(periph_unhandled_count(periph), 2u);
    periph_destroy(periph);
    mem_destroy(mem);
}

TEST(spi_mem_s3_external_nor_state_survives_controller_rebuild) {
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

    /* Program QE through SPI1 after WREN on SPI0, then put the external
     * flash to sleep. A new SoC controller must not power-cycle the chip. */
    s3_spi_user_command(mem, S3_SPI0_BASE, SPI_USER_COMMAND, 0x06u);
    mem_write32(mem, S3_SPI1_BASE + S3_SPI_MOSI_DLEN, 7u);
    mem_write32(mem, S3_SPI1_BASE + S3_SPI_W0, 0x02u);
    s3_spi_user_command(mem, S3_SPI1_BASE,
                        SPI_USER_COMMAND | SPI_USER_MOSI, 0x31u);
    s3_spi_user_command(mem, S3_SPI1_BASE, SPI_USER_COMMAND, 0xB9u);
    mem_write32(mem, S3_SPI1_BASE + S3_SPI_USER1, 0u);
    flexe_spi_mem_nor_state_t chip;
    periph_flash_chip_snapshot(periph, &chip);
    ASSERT_EQ(chip.status[1], 0x02u);
    ASSERT_TRUE(chip.powered_down);
    periph_destroy(periph);

    periph = periph_create(mem);
    ASSERT_TRUE(periph != NULL);
    if (!periph) {
        mem_destroy(mem);
        return;
    }
    ASSERT_EQ(mem_read32(mem, S3_SPI1_BASE + S3_SPI_USER1), 0x5C000007u);
    flexe_spi_mem_nor_state_t cold_chip;
    periph_flash_chip_snapshot(periph, &cold_chip);
    ASSERT_EQ(cold_chip.status[1], 0u);
    ASSERT_TRUE(!cold_chip.powered_down);

    periph_flash_chip_restore(periph, &chip, false);
    s3_spi_user_command(mem, S3_SPI1_BASE,
                        SPI_USER_COMMAND | SPI_USER_MISO, 0x35u);
    ASSERT_EQ(mem_read32(mem, S3_SPI1_BASE + S3_SPI_W0) & 0xFFu, 0xFFu);
    s3_spi_user_command(mem, S3_SPI1_BASE, SPI_USER_COMMAND, 0xABu);
    s3_spi_user_command(mem, S3_SPI1_BASE,
                        SPI_USER_COMMAND | SPI_USER_MISO, 0x35u);
    ASSERT_EQ(mem_read32(mem, S3_SPI1_BASE + S3_SPI_W0) & 0xFFu, 0x02u);

    /* Deep sleep removes flash power. GD25Q32C's QE and drive-strength
     * settings survive, while WEL, suspend/HPF and transient modes do not. */
    flexe_spi_mem_nor_state_t deep = chip;
    deep.status[0] |= 0x02u;
    deep.status[1] |= 0x84u;
    deep.status[2] |= 0x10u;
    deep.address_4byte = true;
    deep.reset_armed = true;
    periph_flash_chip_restore(periph, &deep, true);
    flexe_spi_mem_nor_state_t wake;
    periph_flash_chip_snapshot(periph, &wake);
    ASSERT_EQ(wake.status[0] & 0x03u, 0u);
    ASSERT_EQ(wake.status[1], 0x02u);
    ASSERT_EQ(wake.status[2], 0x20u);
    ASSERT_TRUE(!wake.powered_down);
    ASSERT_TRUE(!wake.address_4byte);
    ASSERT_TRUE(!wake.reset_armed);
    ASSERT_EQ(periph_unhandled_count(periph), 0u);
    periph_destroy(periph);
    mem_destroy(mem);

    /* A different capacity has no claimed nonvolatile status layout. Its
     * deep-sleep path stays cold and emits an explicit diagnostic. */
    mem = mem_create_for_target_with_flash(s3, 0x00800000u);
    periph = periph_create(mem);
    ASSERT_TRUE(mem != NULL);
    ASSERT_TRUE(periph != NULL);
    if (!mem || !periph) {
        periph_destroy(periph);
        mem_destroy(mem);
        return;
    }
    periph_flash_chip_restore(periph, &deep, true);
    periph_flash_chip_snapshot(periph, &wake);
    ASSERT_EQ(wake.status[1], 0u);
    periph_destroy(periph);
    mem_destroy(mem);
}

TEST(spi_mem_s3_user_address_matches_flash_partition_offset) {
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

    /* The S3 SPI_MEM_ADDR register uses bits 23:0 for a 24-bit address.
     * ESP-IDF writes 0x003100FE, not 0x3100FE00, for an operation in a
     * 0x310000 filesystem partition. A shifted decode corrupts flash at
     * 0x003100 while reporting successful erase/program operations. */
    const uint32_t offset = 0x3100FEu;
    mem->flash_data[offset] = 0x34u;
    mem->flash_data[offset + 1u] = 0x12u;
    mem->flash_data[0x3100u] = 0x5Au;
    mem_write32(mem, S3_SPI1_BASE + S3_SPI_USER1, 23u << 26);
    mem_write32(mem, S3_SPI1_BASE + S3_SPI_ADDR, offset);
    mem_write32(mem, S3_SPI1_BASE + S3_SPI_MISO_DLEN, 15u);
    s3_spi_user_command(mem, S3_SPI1_BASE,
                        SPI_USER_COMMAND | SPI_USER_ADDR | SPI_USER_MISO,
                        0xEBu);
    ASSERT_EQ(mem_read32(mem, S3_SPI1_BASE + S3_SPI_W0) & 0xFFFFu,
              0x1234u);

    s3_spi_user_command(mem, S3_SPI1_BASE, SPI_USER_COMMAND, 0x06u);
    s3_spi_user_command(mem, S3_SPI1_BASE,
                        SPI_USER_COMMAND | SPI_USER_ADDR, 0x20u);
    ASSERT_EQ(mem->flash_data[offset], 0xFFu);
    ASSERT_EQ(mem->flash_data[0x3100u], 0x5Au);

    mem_write32(mem, S3_SPI1_BASE + S3_SPI_MOSI_DLEN, 15u);
    mem_write32(mem, S3_SPI1_BASE + S3_SPI_W0, 0x0000BBAAu);
    s3_spi_user_command(mem, S3_SPI1_BASE, SPI_USER_COMMAND, 0x06u);
    s3_spi_user_command(mem, S3_SPI1_BASE,
                        SPI_USER_COMMAND | SPI_USER_ADDR | SPI_USER_MOSI,
                        0x02u);
    ASSERT_EQ(mem->flash_data[offset], 0xAAu);
    ASSERT_EQ(mem->flash_data[offset + 1u], 0xBBu);
    ASSERT_EQ(mem->flash_data[0x3100u], 0x5Au);
    ASSERT_EQ(periph_unhandled_count(periph), 0);

    periph_destroy(periph);
    mem_destroy(mem);
}

TEST(spi_mem_s3_optional_ap_opi_psram_registers_array_and_cache) {
    const flexe_target_desc_t *s3 =
        flexe_target_by_id(FLEXE_TARGET_ESP32S3);
    flexe_target_desc_t board;
    ASSERT_TRUE(!flexe_target_with_board_psram(
        flexe_target_by_id(FLEXE_TARGET_ESP32),
        FLEXE_BOARD_PSRAM_AP_8M_OPI, &board));
    ASSERT_TRUE(flexe_target_with_board_psram(
        s3, FLEXE_BOARD_PSRAM_AP_8M_OPI, &board));
    ASSERT_EQ(board.backing_size[FLEXE_MEM_PSRAM], 0x800000u);

    xtensa_mem_t *mem = mem_create_for_target(&board);
    esp32_periph_t *periph = periph_create(mem);
    ASSERT_TRUE(mem != NULL);
    ASSERT_TRUE(periph != NULL);
    if (!mem || !periph) {
        periph_destroy(periph);
        mem_destroy(mem);
        return;
    }
    ASSERT_EQ(mem_backing_size(mem, FLEXE_MEM_PSRAM), 0x800000u);
    mem_write32(mem, S3_SPI1_BASE + S3_SPI_MISC, 1u);
    mem_write32(mem, S3_SPI0_BASE + S3_SPI_MISC, 1u);
    mem_write32(mem, S3_SPI1_BASE + S3_SPI_MISO_DLEN, 15u);
    s3_opi_command(mem, S3_SPI1_BASE, SPI_USER_MISO, 0x4040u, 0u);
    ASSERT_EQ(mem_read32(mem, S3_SPI1_BASE + S3_SPI_W0) & 0xFFFFu,
              0x0D09u);
    s3_opi_command(mem, S3_SPI1_BASE, SPI_USER_MISO, 0x4040u, 2u);
    ASSERT_EQ(mem_read32(mem, S3_SPI1_BASE + S3_SPI_W0) & 0xFFFFu,
              0xE093u);

    /* Controller hosts share the same external chip's mode registers. */
    mem_write32(mem, S3_SPI1_BASE + S3_SPI_MOSI_DLEN, 15u);
    mem_write32(mem, S3_SPI1_BASE + S3_SPI_W0, 0x0028u);
    s3_opi_command(mem, S3_SPI1_BASE, SPI_USER_MOSI, 0xC0C0u, 0u);
    mem_write32(mem, S3_SPI0_BASE + S3_SPI_MISO_DLEN, 15u);
    s3_opi_command(mem, S3_SPI0_BASE, SPI_USER_MISO, 0x4040u, 0u);
    ASSERT_EQ(mem_read32(mem, S3_SPI0_BASE + S3_SPI_W0) & 0xFFFFu,
              0x0D28u);

    /* OPI array writes are visible through both CPU cache buses after the
     * S3 MMU selects a PSRAM physical page, and vice versa. */
    mem_write32(mem, S3_SPI1_BASE + S3_SPI_MOSI_DLEN, 31u);
    mem_write32(mem, S3_SPI1_BASE + S3_SPI_W0, 0x44332211u);
    s3_opi_command(mem, S3_SPI1_BASE, SPI_USER_MOSI, 0x8080u, 0x20122u);
    ASSERT_EQ(mem->psram[0x20122u], 0x11u);
    mem_write32(mem, board.flash_mmu.table_base[0] + 13u * 4u,
                0x8002u);
    ASSERT_EQ(mem_read32(mem, 0x3C0D0120u) >> 16, 0x2211u);
    ASSERT_EQ(mem_read8(mem, 0x420D0122u), 0x11u);
    mem_write8(mem, 0x3C0D0123u, 0xABu);
    mem_write32(mem, S3_SPI1_BASE + S3_SPI_MISO_DLEN, 31u);
    s3_opi_command(mem, S3_SPI1_BASE, SPI_USER_MISO, 0x2020u, 0x20122u);
    ASSERT_EQ(mem_read32(mem, S3_SPI1_BASE + S3_SPI_W0),
              0x4433AB11u);

    /* Default MR8 selects 32-byte hybrid wrap: finish the current burst,
     * wrap to its start, then continue with the next burst. */
    mem->psram[30u] = 0x1Eu;
    mem->psram[31u] = 0x1Fu;
    mem->psram[0u] = 0xA0u;
    mem->psram[1u] = 0xA1u;
    mem->psram[29u] = 0xADu;
    mem->psram[32u] = 0xB0u;
    mem->psram[33u] = 0xB1u;
    mem_write32(mem, S3_SPI1_BASE + S3_SPI_MISO_DLEN, 34u * 8u - 1u);
    s3_opi_command(mem, S3_SPI1_BASE, SPI_USER_MISO, 0x0000u, 30u);
    ASSERT_EQ(mem_read32(mem, S3_SPI1_BASE + S3_SPI_W0),
              0xA1A01F1Eu);
    ASSERT_EQ(mem_read8(mem, S3_SPI1_BASE + S3_SPI_W0 + 31u), 0xADu);
    ASSERT_EQ(mem_read8(mem, S3_SPI1_BASE + S3_SPI_W0 + 32u), 0xB0u);
    ASSERT_EQ(mem_read8(mem, S3_SPI1_BASE + S3_SPI_W0 + 33u), 0xB1u);

    /* Linear reads stay inside a 1 KiB row until MR8[3] enables RBX. */
    mem->psram[1022u] = 0xE2u;
    mem->psram[1023u] = 0xE3u;
    mem->psram[1024u] = 0xC0u;
    mem->psram[1025u] = 0xC1u;
    mem_write32(mem, S3_SPI1_BASE + S3_SPI_MISO_DLEN, 31u);
    s3_opi_command(mem, S3_SPI1_BASE, SPI_USER_MISO, 0x2020u, 1022u);
    ASSERT_EQ(mem_read32(mem, S3_SPI1_BASE + S3_SPI_W0),
              0xA1A0E3E2u);
    mem_write32(mem, S3_SPI1_BASE + S3_SPI_W0, 0x0Du);
    s3_opi_command(mem, S3_SPI1_BASE, SPI_USER_MOSI, 0xC0C0u, 8u);
    s3_opi_command(mem, S3_SPI1_BASE, SPI_USER_MISO, 0x2020u, 1022u);
    ASSERT_EQ(mem_read32(mem, S3_SPI1_BASE + S3_SPI_W0),
              0xC1C0E3E2u);

    /* Reset the external chip without erasing the PSRAM array. Unsupported
     * 8-bit commands do not alias this 16-bit octal device. */
    s3_opi_command(mem, S3_SPI1_BASE, 0u, 0xFFFFu, 0u);
    s3_opi_command(mem, S3_SPI1_BASE, SPI_USER_MISO, 0x4040u, 0u);
    ASSERT_EQ(mem_read32(mem, S3_SPI1_BASE + S3_SPI_W0) & 0xFFFFu,
              0x0D09u);
    ASSERT_EQ(mem->psram[0x20123u], 0xABu);
    ASSERT_EQ(periph_unhandled_count(periph), 0u);

    mem_write32(mem, S3_SPI1_BASE + S3_SPI_W0, 0x28u);
    s3_opi_command(mem, S3_SPI1_BASE, SPI_USER_MOSI, 0xC0C0u, 0u);
    flexe_spi_mem_psram_state_t chip;
    periph_psram_chip_snapshot(periph, &chip);
    periph_destroy(periph);
    periph = periph_create(mem);
    ASSERT_TRUE(periph != NULL);
    if (!periph) { mem_destroy(mem); return; }
    periph_psram_chip_restore(periph, &chip, false);
    mem_write32(mem, S3_SPI1_BASE + S3_SPI_MISC, 1u);
    mem_write32(mem, S3_SPI1_BASE + S3_SPI_MISO_DLEN, 15u);
    s3_opi_command(mem, S3_SPI1_BASE, SPI_USER_MISO, 0x4040u, 0u);
    ASSERT_EQ(mem_read32(mem, S3_SPI1_BASE + S3_SPI_W0) & 0xFFFFu,
              0x0D28u);
    ASSERT_EQ(mem->psram[0x20123u], 0xABu);

    /* A chip power cycle returns the mode registers to their documented
     * defaults; byte-sized NOR commands cannot be reinterpreted as OPI. */
    periph_destroy(periph);
    periph = periph_create(mem);
    ASSERT_TRUE(periph != NULL);
    if (!periph) { mem_destroy(mem); return; }
    periph_psram_chip_restore(periph, &chip, true);
    mem_write32(mem, S3_SPI1_BASE + S3_SPI_MISC, 1u);
    mem_write32(mem, S3_SPI1_BASE + S3_SPI_MISO_DLEN, 15u);
    s3_opi_command(mem, S3_SPI1_BASE, SPI_USER_MISO, 0x4040u, 0u);
    ASSERT_EQ(mem_read32(mem, S3_SPI1_BASE + S3_SPI_W0) & 0xFFFFu,
              0x0D09u);
    s3_spi_user_command(mem, S3_SPI1_BASE,
                        SPI_USER_COMMAND | SPI_USER_MISO, 0x9Fu);
    ASSERT_EQ(periph_unhandled_count(periph), 1u);
    s3_opi_command(mem, S3_SPI1_BASE, SPI_USER_MISO, 0x4040u, 9u);
    ASSERT_EQ(periph_unhandled_count(periph), 2u);

    periph_destroy(periph);
    mem_destroy(mem);
}

void run_spi_mem_tests(void) {
    TEST_SUITE("Target SPI memory controller");
    RUN_TEST(spi_mem_uses_target_layouts_and_reports_jedec_id);
    RUN_TEST(spi_mem_classic_routes_psram_by_chip_select_and_wire_phases);
    RUN_TEST(spi_mem_unpopulated_chip_select_clocks_any_command_width);
    RUN_TEST(spi_mem_s3_program_erase_and_shared_mmu_invalidation);
    RUN_TEST(spi_mem_reports_allocated_flash_capacity);
    RUN_TEST(spi_mem_sfdp_matches_advertised_gd25q_c_profiles);
    RUN_TEST(spi_mem_gd25q32c_status_survives_flash_reset);
    RUN_TEST(spi_mem_gd25q32c_protection_blocks_whole_operations);
    RUN_TEST(spi_mem_gd25q32c_protection_table_boundaries);
    RUN_TEST(spi_mem_page_program_wraps_within_physical_page);
    RUN_TEST(spi_mem_other_flash_profiles_reject_unknown_protection_layout);
    RUN_TEST(spi_mem_s3_external_nor_state_survives_controller_rebuild);
    RUN_TEST(spi_mem_s3_user_address_matches_flash_partition_offset);
    RUN_TEST(spi_mem_s3_optional_ap_opi_psram_registers_array_and_cache);
}
