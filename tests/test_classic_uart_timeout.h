/* Classic ESP32 timeout regression, using MMIO and virtual peripheral time. */
typedef struct {
    xtensa_mem_t *mem;
    esp32_periph_t *periph;
    xtensa_cpu_t cpu;
    uint32_t base;
    unsigned port;
} classic_uart_fixture_t;

static bool classic_uart_init(classic_uart_fixture_t *f, unsigned port,
                               bool apb, uint32_t divider, uint32_t threshold) {
    static const uint32_t bases[] = {0x3ff40000u, 0x3ff50000u, 0x3ff6e000u};
    memset(f, 0, sizeof *f);
    f->mem = mem_create();
    f->periph = f->mem ? periph_create(f->mem) : NULL;
    ASSERT_TRUE(f->mem != NULL && f->periph != NULL);
    if (!f->mem || !f->periph) {
        periph_destroy(f->periph); mem_destroy(f->mem);
        return false;
    }
    f->base = bases[port]; f->port = port;
    xtensa_cpu_init(&f->cpu); f->cpu.mem = f->mem;
    periph_attach_cpus(f->periph, &f->cpu, NULL);
    periph_intr_matrix_set(f->periph, 0, 5, 34u + port);
    mem_write32(f->mem, f->base + 0x14u, divider);
    mem_write32(f->mem, f->base + 0x20u, apb ? 1u << 27 : 0u);
    mem_write32(f->mem, f->base + 0x24u,
                (1u << 31) | ((threshold & 127u) << 24) | 100u);
    mem_write32(f->mem, f->base + 0x0cu, 1u << 8);
    return true;
}

static void classic_uart_destroy(classic_uart_fixture_t *f) {
    periph_destroy(f->periph); mem_destroy(f->mem);
}

static void classic_uart_advance(classic_uart_fixture_t *f, uint32_t us) {
    f->cpu.ccount += us * xtensa_cpu_freq_mhz(&f->cpu);
    f->cpu.periph_event(&f->cpu);
}

static void classic_uart_check_irq(classic_uart_fixture_t *f, bool pending) {
    uint32_t bit = pending ? 1u << 8 : 0u;
    ASSERT_EQ(mem_read32(f->mem, f->base + 4u) & (1u << 8), bit);
    ASSERT_EQ(mem_read32(f->mem, f->base + 8u) & (1u << 8), bit);
    ASSERT_EQ(f->cpu.interrupt & (1u << 5), pending ? 1u << 5 : 0u);
}

TEST(classic_uart_timeout_holds_until_drained) {
    for (unsigned port = 0; port < 3; ++port) {
        for (unsigned read_before_clear = 0; read_before_clear < 3; ++read_before_clear) {
            classic_uart_fixture_t f;
            if (!classic_uart_init(&f, port, true, 80u, 4u)) return;
            static const uint8_t input[] = {'A', 'B', 'C'};
            ASSERT_EQ(periph_uart_rx_inject_num(f.periph, port, input, sizeof input), 3u);
            classic_uart_check_irq(&f, false);
            classic_uart_advance(&f, 31u);
            classic_uart_check_irq(&f, false);
            classic_uart_advance(&f, 1u);
            classic_uart_check_irq(&f, true);
            for (unsigned i = 0; i < read_before_clear; ++i)
                ASSERT_EQ(mem_read32(f.mem, f.base), input[i]);
            mem_write32(f.mem, f.base + 0x10u, 1u << 8);
            /* No waiting and no new arrivals: a nonempty FIFO prevents clear. */
            ASSERT_EQ(periph_uart_rx_pending_num(f.periph, port), 3u - read_before_clear);
            classic_uart_check_irq(&f, true);
            classic_uart_advance(&f, 1u);
            classic_uart_check_irq(&f, true);
            for (unsigned i = read_before_clear; i < 3; ++i)
                ASSERT_EQ(mem_read32(f.mem, f.base), input[i]);
            /* Draining does not itself acknowledge the interrupt. */
            classic_uart_check_irq(&f, true);
            mem_write32(f.mem, f.base + 0x10u, 1u << 8);
            classic_uart_check_irq(&f, false);
            classic_uart_advance(&f, 64u);
            classic_uart_check_irq(&f, false);
            classic_uart_destroy(&f);
        }
    }
}

TEST(classic_uart_timeout_clock_units) {
    /* CLKDIV's fraction is in bits20..23. Retain fractions until rounding up.
     * APB threshold units are eight bits; REF_TICK units are one eighth bit.
     * The two zero-divider cases retain the shared model's 115200 fallback.
     */
    static const struct {
        bool apb; uint32_t divider, threshold, deadline_us;
    } cases[] = {
        {true, 80u, 4u, 32u}, {false, 8u, 32u, 32u},
        {true, 80u | (8u << 20), 1u, 9u},
        {false, 8u | (8u << 20), 8u, 9u},
        {true, 0u, 4u, 278u}, {false, 0u, 8u, 9u},
        {true, 80u, 0u, 1u},
    };
    for (unsigned port = 0; port < 3; ++port) {
        for (unsigned i = 0; i < sizeof cases / sizeof cases[0]; ++i) {
            classic_uart_fixture_t f;
            if (!classic_uart_init(&f, port, cases[i].apb, cases[i].divider,
                                    cases[i].threshold)) return;
            const uint8_t byte = 'X';
            ASSERT_EQ(periph_uart_rx_inject_num(f.periph, port, &byte, 1), 1u);
            classic_uart_check_irq(&f, false);
            classic_uart_advance(&f, cases[i].deadline_us - 1u);
            classic_uart_check_irq(&f, false);
            classic_uart_advance(&f, 1u);
            classic_uart_check_irq(&f, true);
            ASSERT_EQ(mem_read32(f.mem, f.base), 'X');
            mem_write32(f.mem, f.base + 0x10u, 1u << 8);
            classic_uart_check_irq(&f, false);
            classic_uart_destroy(&f);
        }
    }
}

TEST(classic_uart_timeout_restart_enable_and_drain) {
    for (unsigned port = 0; port < 3; ++port) {
        classic_uart_fixture_t f;
        if (!classic_uart_init(&f, port, true, 80u, 4u)) return;
        uint32_t conf1 = (1u << 31) | (4u << 24) | 100u;
        const uint8_t byte = 'X';
        ASSERT_EQ(periph_uart_rx_inject_num(f.periph, port, &byte, 1), 1u);
        classic_uart_advance(&f, 16u);
        ASSERT_EQ(periph_uart_rx_inject_num(f.periph, port, &byte, 1), 1u);
        classic_uart_advance(&f, 16u);
        classic_uart_check_irq(&f, false);
        classic_uart_advance(&f, 16u);
        classic_uart_check_irq(&f, true);
        while (periph_uart_rx_pending_num(f.periph, port)) (void)mem_read32(f.mem, f.base);
        mem_write32(f.mem, f.base + 0x10u, 1u << 8);
        ASSERT_EQ(periph_uart_rx_inject_num(f.periph, port, &byte, 1), 1u);
        classic_uart_advance(&f, 16u);
        mem_write32(f.mem, f.base + 0x24u, conf1 & ~(1u << 31));
        classic_uart_advance(&f, 64u);
        classic_uart_check_irq(&f, false);
        mem_write32(f.mem, f.base + 0x24u, conf1);
        classic_uart_check_irq(&f, false);
        classic_uart_advance(&f, 31u);
        classic_uart_check_irq(&f, false);
        classic_uart_advance(&f, 1u);
        classic_uart_check_irq(&f, true);
        ASSERT_EQ(mem_read32(f.mem, f.base), 'X');
        mem_write32(f.mem, f.base + 0x10u, 1u << 8);
        ASSERT_EQ(periph_uart_rx_inject_num(f.periph, port, &byte, 1), 1u);
        classic_uart_advance(&f, 16u);
        ASSERT_EQ(mem_read32(f.mem, f.base), 'X');
        classic_uart_advance(&f, 64u);
        classic_uart_check_irq(&f, false);
        classic_uart_destroy(&f);
    }
}

TEST(shared_uart_timeout_reset_cancels_pending) {
    /* S3 exposes the system reset controls for this shared UART lifecycle.
     * Classic DPORT UART reset wiring is not modeled by this change.
     */
    const flexe_target_desc_t *target = flexe_target_by_id(FLEXE_TARGET_ESP32S3);
    xtensa_mem_t *mem = mem_create_for_target(target);
    esp32_periph_t *p = mem ? periph_create(mem) : NULL;
    ASSERT_TRUE(mem != NULL && p != NULL);
    if (!mem || !p) { periph_destroy(p); mem_destroy(mem); return; }
    xtensa_cpu_t cpu; xtensa_cpu_init_for_target(&cpu, target); cpu.mem = mem;
    periph_attach_cpus(p, &cpu, NULL);
    const uint32_t base = 0x60010000u;
    mem_write32(mem, base + 0x78u, 1u << 20);
    mem_write32(mem, base + 0x14u, 80u);
    mem_write32(mem, base + 0x60u, 32u << 17);
    mem_write32(mem, base + 0x24u, (1u << 23) | 100u);
    const uint8_t byte = 'X';
    ASSERT_EQ(periph_uart_rx_inject_num(p, 1, &byte, 1), 1u);
    cpu.ccount += 16u * xtensa_cpu_freq_mhz(&cpu); cpu.periph_event(&cpu);
    uint32_t reset = mem_read32(mem, 0x600c0020u);
    mem_write32(mem, 0x600c0020u, reset | (1u << 5));
    mem_write32(mem, 0x600c0020u, reset & ~(1u << 5));
    ASSERT_EQ(periph_uart_rx_pending_num(p, 1), 0u);
    cpu.ccount += 64u * xtensa_cpu_freq_mhz(&cpu); cpu.periph_event(&cpu);
    ASSERT_EQ(mem_read32(mem, base + 4u) & (1u << 8), 0u);
    periph_destroy(p); mem_destroy(mem);
}
