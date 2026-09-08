#include "sx127x.h"

#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#define SX127X_REG_FIFO                 0x00u
#define SX127X_REG_OP_MODE              0x01u
#define SX127X_REG_FIFO_ADDR_PTR        0x0Du
#define SX127X_REG_FIFO_TX_BASE_ADDR    0x0Eu
#define SX127X_REG_FIFO_RX_BASE_ADDR    0x0Fu
#define SX127X_REG_FIFO_RX_CURRENT_ADDR 0x10u
#define SX127X_REG_IRQ_FLAGS_MASK       0x11u
#define SX127X_REG_IRQ_FLAGS            0x12u
#define SX127X_REG_RX_NB_BYTES          0x13u
#define SX127X_REG_PKT_SNR_VALUE        0x19u
#define SX127X_REG_PKT_RSSI_VALUE       0x1Au
#define SX127X_REG_HOP_CHANNEL          0x1Cu
#define SX127X_REG_PAYLOAD_LENGTH       0x22u
#define SX127X_REG_DIO_MAPPING_1        0x40u
#define SX127X_REG_VERSION              0x42u

#define SX127X_LONG_RANGE_MODE          0x80u
#define SX127X_MODE_MASK                0x07u
#define SX127X_MODE_TX                  0x03u
#define SX127X_MODE_RX_CONTINUOUS       0x05u
#define SX127X_MODE_RX_SINGLE           0x06u

#define SX127X_IRQ_RX_DONE              0x40u
#define SX127X_IRQ_VALID_HEADER         0x10u
#define SX127X_IRQ_TX_DONE              0x08u
#define SX127X_IRQ_CAD_DONE             0x04u

struct sx127x {
    esp32_periph_t *periph;
    int dio0_pin;
    uint8_t reg[128];
    uint8_t fifo[256];
    sx127x_tx_fn tx_fn;
    void *tx_ctx;
    sx127x_stats_t stats;
    bool selected;
    bool have_command;
    bool write;
    uint8_t address;
};

static bool sx127x_lora_mode(const sx127x_t *radio) {
    return (radio->reg[SX127X_REG_OP_MODE] & SX127X_LONG_RANGE_MODE) != 0;
}

static void sx127x_update_dio0(sx127x_t *radio) {
    if (!radio || !radio->periph || radio->dio0_pin < 0) return;

    bool high = false;
    if (sx127x_lora_mode(radio)) {
        uint8_t mapping = radio->reg[SX127X_REG_DIO_MAPPING_1] >> 6;
        uint8_t flags = radio->reg[SX127X_REG_IRQ_FLAGS];
        if (mapping == 0u) high = (flags & SX127X_IRQ_RX_DONE) != 0;
        else if (mapping == 1u) high = (flags & SX127X_IRQ_TX_DONE) != 0;
        else if (mapping == 2u) high = (flags & SX127X_IRQ_CAD_DONE) != 0;
    }
    periph_gpio_set_input(radio->periph, radio->dio0_pin, high ? 1 : 0);
}

void sx127x_reset(sx127x_t *radio) {
    if (!radio) return;
    memset(radio->reg, 0, sizeof(radio->reg));
    memset(radio->fifo, 0, sizeof(radio->fifo));
    radio->selected = false;
    radio->have_command = false;

    /* SX1276/RFM95 reset values used by RadioLib during discovery and
     * read-modify-write configuration. */
    radio->reg[SX127X_REG_OP_MODE] = 0x01u;
    radio->reg[0x06u] = 0x6Cu;
    radio->reg[0x07u] = 0x80u;
    radio->reg[0x08u] = 0x00u;
    radio->reg[0x0Bu] = 0x2Bu;
    radio->reg[0x0Cu] = 0x20u;
    radio->reg[0x1Du] = 0x72u;
    radio->reg[0x1Eu] = 0x74u;
    radio->reg[0x26u] = 0x04u;
    radio->reg[0x4Du] = 0x84u;
    radio->reg[SX127X_REG_VERSION] = 0x12u;
    sx127x_update_dio0(radio);
}

sx127x_t *sx127x_create(const sx127x_config_t *config) {
    if (!config || !config->periph || config->dio0_pin < -1 ||
        config->dio0_pin > 39)
        return NULL;
    sx127x_t *radio = calloc(1, sizeof(*radio));
    if (!radio) return NULL;
    radio->periph = config->periph;
    radio->dio0_pin = config->dio0_pin;
    sx127x_reset(radio);
    return radio;
}

void sx127x_destroy(sx127x_t *radio) {
    if (!radio) return;
    if (radio->periph && radio->dio0_pin >= 0)
        periph_gpio_set_input(radio->periph, radio->dio0_pin, 0);
    free(radio);
}

static uint8_t sx127x_read_register(sx127x_t *radio, uint8_t address) {
    radio->stats.register_reads++;
    if (address == SX127X_REG_VERSION) radio->stats.version_reads++;
    if (address == SX127X_REG_FIFO) {
        uint8_t ptr = radio->reg[SX127X_REG_FIFO_ADDR_PTR]++;
        return radio->fifo[ptr];
    }
    return radio->reg[address & 0x7Fu];
}

static void sx127x_complete_tx(sx127x_t *radio) {
    uint8_t base = radio->reg[SX127X_REG_FIFO_TX_BASE_ADDR];
    size_t len = radio->reg[SX127X_REG_PAYLOAD_LENGTH];
    uint8_t packet[256];
    for (size_t i = 0; i < len; i++)
        packet[i] = radio->fifo[(uint8_t)(base + i)];

    radio->stats.tx_packets++;
    radio->stats.last_tx_len = len;
    if (radio->tx_fn) radio->tx_fn(radio->tx_ctx, packet, len);
    radio->reg[SX127X_REG_IRQ_FLAGS] |= SX127X_IRQ_TX_DONE;
}

static void sx127x_write_register(sx127x_t *radio, uint8_t address,
                                  uint8_t value) {
    address &= 0x7Fu;
    radio->stats.register_writes++;
    if (address == SX127X_REG_FIFO) {
        uint8_t ptr = radio->reg[SX127X_REG_FIFO_ADDR_PTR]++;
        radio->fifo[ptr] = value;
        return;
    }
    if (address == SX127X_REG_VERSION) return;
    if (address == SX127X_REG_IRQ_FLAGS) {
        radio->reg[address] &= (uint8_t)~value;
        return;
    }

    uint8_t old = radio->reg[address];
    radio->reg[address] = value;
    if (address == SX127X_REG_OP_MODE &&
        (old & (SX127X_LONG_RANGE_MODE | SX127X_MODE_MASK)) !=
            (value & (SX127X_LONG_RANGE_MODE | SX127X_MODE_MASK)) &&
        (value & SX127X_LONG_RANGE_MODE) != 0 &&
        (value & SX127X_MODE_MASK) == SX127X_MODE_TX)
        sx127x_complete_tx(radio);
}

void sx127x_spi_transfer(void *ctx, int host,
                         const uint8_t *mosi, size_t mosi_len,
                         uint8_t *miso, size_t miso_len) {
    sx127x_t *radio = ctx;
    if (!radio) return;
    radio->stats.spi_transfers++;

    if (!mosi || mosi_len == 0) {
        if (miso) memset(miso, 0, miso_len);
        return;
    }

    size_t clocks = mosi_len > miso_len ? mosi_len : miso_len;
    bool standalone = !radio->selected;
    if (standalone) {
        radio->have_command = false;
        radio->stats.spi_frames++;
    }
    for (size_t i = 0; i < clocks; i++) {
        uint8_t value = i < mosi_len ? mosi[i] : 0xFFu;
        if (!radio->have_command) {
            radio->write = (value & 0x80u) != 0;
            radio->address = value & 0x7Fu;
            radio->have_command = true;
            if (miso && i < miso_len) miso[i] = 0;
            continue;
        }
        if (radio->write) {
            if (i < mosi_len)
                sx127x_write_register(radio, radio->address, value);
            if (miso && i < miso_len) miso[i] = 0;
        } else if (miso && i < miso_len) {
            miso[i] = sx127x_read_register(radio, radio->address);
        }
        if (radio->address != SX127X_REG_FIFO)
            radio->address = (radio->address + 1u) & 0x7Fu;
    }
    if (standalone) radio->have_command = false;
    sx127x_update_dio0(radio);
}

void sx127x_spi_select(void *ctx, int selected) {
    sx127x_t *radio = ctx;
    if (!radio) return;
    radio->selected = selected != 0;
    radio->have_command = false;
    if (radio->selected) radio->stats.spi_frames++;
}

void sx127x_set_tx_callback(sx127x_t *radio, sx127x_tx_fn fn, void *ctx) {
    if (!radio) return;
    radio->tx_fn = fn;
    radio->tx_ctx = ctx;
}

int sx127x_inject_packet(sx127x_t *radio, const uint8_t *data, size_t len,
                         int8_t rssi_dbm, int8_t snr_quarter_db) {
    if (!radio || (!data && len != 0) || len > 255u ||
        !sx127x_lora_mode(radio))
        return -1;
    uint8_t mode = radio->reg[SX127X_REG_OP_MODE] & SX127X_MODE_MASK;
    if (mode != SX127X_MODE_RX_CONTINUOUS && mode != SX127X_MODE_RX_SINGLE)
        return -1;

    uint8_t base = radio->reg[SX127X_REG_FIFO_RX_BASE_ADDR];
    for (size_t i = 0; i < len; i++)
        radio->fifo[(uint8_t)(base + i)] = data[i];
    radio->reg[SX127X_REG_FIFO_RX_CURRENT_ADDR] = base;
    radio->reg[SX127X_REG_RX_NB_BYTES] = (uint8_t)len;
    radio->reg[SX127X_REG_PKT_SNR_VALUE] = (uint8_t)snr_quarter_db;
    int raw_rssi = (int)rssi_dbm + 157;
    if (raw_rssi < 0) raw_rssi = 0;
    if (raw_rssi > 255) raw_rssi = 255;
    radio->reg[SX127X_REG_PKT_RSSI_VALUE] = (uint8_t)raw_rssi;
    radio->reg[SX127X_REG_HOP_CHANNEL] |= 1u << 6; /* valid payload CRC */
    radio->reg[SX127X_REG_IRQ_FLAGS] |=
        SX127X_IRQ_RX_DONE | SX127X_IRQ_VALID_HEADER;
    radio->stats.rx_packets++;
    sx127x_update_dio0(radio);
    return 0;
}

void sx127x_get_stats(const sx127x_t *radio, sx127x_stats_t *out) {
    if (!out) return;
    if (radio) *out = radio->stats;
    else memset(out, 0, sizeof(*out));
}

uint8_t sx127x_register(const sx127x_t *radio, uint8_t address) {
    return radio ? radio->reg[address & 0x7Fu] : 0;
}
