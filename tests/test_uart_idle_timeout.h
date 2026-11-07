/* Classic ESP32 UART idle timeout regression; virtual time, no host sleeps.
 * Included by test_peripherals.c. Uses the ordinary MMIO/injection/event paths.
 */
static void uart_idle_advance_us(xtensa_cpu_t *cpu, uint32_t us) {
    cpu->ccount += xtensa_cpu_freq_mhz(cpu) * us;
    cpu->periph_event(cpu);
}

TEST(uart_idle_timeout_rearm) {
    const uint32_t bases[] = {0x3ff40000u, 0x3ff50000u, 0x3ff6e000u};
    for (unsigned port = 0; port < 3; ++port) {
        xtensa_mem_t *mem = mem_create();
        esp32_periph_t *p = periph_create(mem);
        xtensa_cpu_t cpu;
        xtensa_cpu_init(&cpu); cpu.mem = mem;
        periph_attach_cpus(p, &cpu, NULL);
        periph_intr_matrix_set(p, 0, 5, 34u + port);
        uint32_t base = bases[port];
        mem_write32(mem, base + 0x14, 80u); /* APB/80 = 1 Mbaud. */
        mem_write32(mem, base + 0x20, 1u << 27);
        mem_write32(mem, base + 0x24, (1u << 31) | (4u << 24) | 100u);
        mem_write32(mem, base + 0x0c, 1u << 8);
        const uint8_t input[] = {'A', 'B', 'C'};
        ASSERT_EQ(periph_uart_rx_inject_num(p, port, input, sizeof input), 3u);
        ASSERT_EQ(mem_read32(mem, base + 4) & (1u << 8), 0u);
        ASSERT_EQ(cpu.interrupt & (1u << 5), 0u);
        /* Four baud/8 ticks = 32 us. */
        uart_idle_advance_us(&cpu, 31u);
        ASSERT_EQ(mem_read32(mem, base + 8), 0u);
        uart_idle_advance_us(&cpu, 1u);
        ASSERT_EQ(mem_read32(mem, base + 8), 1u << 8);
        ASSERT_EQ(cpu.interrupt & (1u << 5), 1u << 5);
        ASSERT_EQ(mem_read32(mem, base), 'A');
        mem_write32(mem, base + 0x10, 1u << 8);
        ASSERT_EQ(mem_read32(mem, base + 8), 0u);
        ASSERT_EQ(cpu.interrupt & (1u << 5), 0u);
        uart_idle_advance_us(&cpu, 31u);
        ASSERT_EQ(mem_read32(mem, base + 8), 0u);
        uart_idle_advance_us(&cpu, 1u);
        /* No more input: unread B,C must still cause a new idle interrupt. */
        ASSERT_EQ(periph_uart_rx_pending_num(p, port), 2u);
        ASSERT_EQ(mem_read32(mem, base + 8), 1u << 8);
        ASSERT_EQ(cpu.interrupt & (1u << 5), 1u << 5);
        ASSERT_EQ(mem_read32(mem, base), 'B');
        ASSERT_EQ(mem_read32(mem, base), 'C');
        mem_write32(mem, base + 0x10, 1u << 8);
        uart_idle_advance_us(&cpu, 64u);
        ASSERT_EQ(mem_read32(mem, base + 8), 0u);
        ASSERT_EQ(cpu.interrupt & (1u << 5), 0u);
        periph_destroy(p); mem_destroy(mem);
    }
}

TEST(uart_idle_timeout_restart_and_disable) {
    xtensa_mem_t *mem = mem_create();
    esp32_periph_t *p = periph_create(mem);
    xtensa_cpu_t cpu;
    xtensa_cpu_init(&cpu); cpu.mem = mem;
    periph_attach_cpus(p, &cpu, NULL);
    const uint32_t base = 0x3ff40000u;
    const uint32_t conf1 = (1u << 31) | (4u << 24) | 100u;
    const uint8_t byte = 'X';
    mem_write32(mem, base + 0x14, 80u);
    mem_write32(mem, base + 0x20, 1u << 27);
    mem_write32(mem, base + 0x24, conf1);
    mem_write32(mem, base + 0x0c, 1u << 8);
    ASSERT_EQ(periph_uart_rx_inject(p, &byte, 1), 1u);
    uart_idle_advance_us(&cpu, 16u);
    ASSERT_EQ(periph_uart_rx_inject(p, &byte, 1), 1u);
    uart_idle_advance_us(&cpu, 16u);
    ASSERT_EQ(mem_read32(mem, base + 8), 0u); /* New arrival restarts idle time. */
    uart_idle_advance_us(&cpu, 16u);
    ASSERT_EQ(mem_read32(mem, base + 8), 1u << 8);
    mem_write32(mem, base + 0x10, 1u << 8);
    mem_write32(mem, base + 0x24, conf1 & ~(1u << 31));
    uart_idle_advance_us(&cpu, 64u);
    ASSERT_EQ(mem_read32(mem, base + 8), 0u);
    mem_write32(mem, base + 0x24, conf1);
    uart_idle_advance_us(&cpu, 32u);
    ASSERT_EQ(mem_read32(mem, base + 8), 1u << 8);
    mem_write32(mem, base + 0x10, 1u << 8);
    while (periph_uart_rx_pending(p)) (void)mem_read32(mem, base);
    uart_idle_advance_us(&cpu, 64u);
    ASSERT_EQ(mem_read32(mem, base + 8), 0u); /* No stale callback after drain. */
    periph_destroy(p); mem_destroy(mem);
}
