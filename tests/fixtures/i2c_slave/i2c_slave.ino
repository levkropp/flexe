/* I2C slave mode, through driver/i2c.h.
 *
 * Both ports were master-only. I2C_CTR.MS_MODE was stored and never consulted,
 * so a port configured as a slave still ran the master command list whenever
 * TRANS_START was written -- originating traffic it should have been answering
 * -- and nothing could deliver a transfer *to* the guest at all. Firmware
 * calling i2c_driver_install(I2C_MODE_SLAVE) installed cleanly and then never
 * saw a byte.
 *
 * The host is the bus master here. The guest stages a reply, waits for a write
 * to arrive, and checks it got the bytes that were sent; the host checks it
 * read back the bytes the guest staged. Neither side alone is enough: a model
 * that loops the guest's own TX FIFO into its RX FIFO satisfies the guest.
 */
#include <Arduino.h>
#include <driver/i2c.h>

#define SUCCESS_MARKER 0x12C51AEEu
#define FAIL_BASE      0xBAD00000u

#define PORT        I2C_NUM_0
#define SLAVE_ADDR  0x42
#define SDA_PIN     21
#define SCL_PIN     22
#define SPIN_LIMIT  200000

/* What the host sends, and what it should read back. */
static const uint8_t EXPECT_RX[4] = { 0xDE, 0xAD, 0xBE, 0xEF };
static const uint8_t STAGE_TX[3]  = { 0x11, 0x22, 0x33 };

volatile uint32_t flexe_i2cslave_stage = 0;
volatile uint32_t flexe_i2cslave_result[8];

static void fail(uint32_t code) { flexe_i2cslave_stage = FAIL_BASE | code; }

void setup() {
    i2c_config_t cfg = {};
    cfg.mode = I2C_MODE_SLAVE;
    cfg.sda_io_num = SDA_PIN;
    cfg.scl_io_num = SCL_PIN;
    cfg.sda_pullup_en = GPIO_PULLUP_ENABLE;
    cfg.scl_pullup_en = GPIO_PULLUP_ENABLE;
    cfg.slave.addr_10bit_en = 0;
    cfg.slave.slave_addr = SLAVE_ADDR;

    if (i2c_param_config(PORT, &cfg) != ESP_OK) { fail(1); return; }
    if (i2c_driver_install(PORT, I2C_MODE_SLAVE, 256, 256, 0) != ESP_OK) {
        fail(2); return;
    }

    /* Stage a reply for the host to read. */
    int staged = i2c_slave_write_buffer(PORT, STAGE_TX, sizeof STAGE_TX,
                                        100 / portTICK_PERIOD_MS);
    flexe_i2cslave_result[0] = (uint32_t)staged;
    if (staged != (int)sizeof STAGE_TX) { fail(3); return; }

    /* Tell the host to run a transfer, then wait for the bytes to arrive. */
    flexe_i2cslave_stage = 1;

    uint8_t buf[8] = {0};
    int got = 0;
    for (uint32_t i = 0; i < SPIN_LIMIT && got < (int)sizeof EXPECT_RX; i++) {
        int n = i2c_slave_read_buffer(PORT, buf + got,
                                      sizeof EXPECT_RX - (size_t)got,
                                      10 / portTICK_PERIOD_MS);
        if (n > 0) got += n;
    }
    flexe_i2cslave_result[1] = (uint32_t)got;
    if (got != (int)sizeof EXPECT_RX) { fail(4); return; }

    /* Pack what arrived so the host can check the bytes, not just the count. */
    flexe_i2cslave_result[2] = (uint32_t)buf[0] | ((uint32_t)buf[1] << 8) |
                               ((uint32_t)buf[2] << 16) |
                               ((uint32_t)buf[3] << 24);
    for (size_t i = 0; i < sizeof EXPECT_RX; i++)
        if (buf[i] != EXPECT_RX[i]) { fail(5); return; }

    flexe_i2cslave_stage = SUCCESS_MARKER;
}

void loop() { delay(10); }
