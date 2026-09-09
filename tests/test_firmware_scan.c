#include "firmware_scan.h"
#include "test_helpers.h"

TEST(xtensa_firmware_fingerprint_normalizes_relocations) {
    xtensa_cpu_t cpu;
    setup(&cpu);
    const uint32_t first = BASE + 0x100u;
    const uint32_t second = BASE + 0x180u;
    static const uint8_t prefix[] = {0x36, 0x41, 0x00};
    static const uint8_t body[] = {
        0x36, 0x41, 0x00,             /* ENTRY */
        0x65, 0xB3, 0x00,             /* CALL8 */
        0x81, 0x12, 0xFE,             /* L32R a8 */
        0x3D, 0xF0,                   /* RETW.N */
    };
    put_test_bytes(&cpu, first, body, sizeof(body));
    put_test_bytes(&cpu, second, body, sizeof(body));

    /* Move only CALLn's 18-bit target. Its opcode and CALLINC bits survive
     * normalization, including the two high displacement bits in byte 3. */
    mem_write8(cpu.mem, second + 3u, 0xA5u);
    mem_write8(cpu.mem, second + 4u, 0x12u);
    mem_write8(cpu.mem, second + 5u, 0xFEu);
    mem_write8(cpu.mem, second + 7u, 0xA5u);
    mem_write8(cpu.mem, second + 8u, 0x01u);
    uint32_t found = 0u;
    ASSERT_TRUE(firmware_find_xtensa_crc32_body(
            cpu.mem, second, second + sizeof(body), prefix, sizeof(prefix),
            sizeof(body), 0x3065139Fu, &found));
    ASSERT_EQ(found, second);

    /* Changing CALLINC is an ABI change, not relocation, and changing any
     * ordinary body byte must invalidate the complete fingerprint. */
    mem_write8(cpu.mem, second + 3u, 0xB5u);
    ASSERT_FALSE(firmware_xtensa_crc32_matches(
            cpu.mem, second, sizeof(body), 0x3065139Fu));
    mem_write8(cpu.mem, second + 3u, 0xA5u);
    mem_write8(cpu.mem, second + 9u, 0x2Du);
    ASSERT_FALSE(firmware_xtensa_crc32_matches(
            cpu.mem, second, sizeof(body), 0x3065139Fu));

    /* A fingerprint cannot silently stop inside an instruction. */
    ASSERT_FALSE(firmware_xtensa_crc32_matches(
            cpu.mem, first, sizeof(body) - 1u, 0x3065139Fu));

    teardown(&cpu);
}

static void run_firmware_scan_tests(void) {
    TEST_SUITE("Firmware scanning");
    RUN_TEST(xtensa_firmware_fingerprint_normalizes_relocations);
}
