#include <Arduino.h>
#include <esp_eth_mac.h>
#include <esp_eth_com.h>

volatile uint32_t flexe_emac_stage = 0;
volatile uint32_t flexe_emac_command = 0;
volatile uint32_t flexe_emac_result[32] = {};

static constexpr uint32_t kSuccessMarker = 0x454D4143u;
static constexpr size_t kTxLength = 700;
static esp_eth_mac_t *emac;

typedef struct {
  esp_eth_mediator_t parent;
} test_mediator_t;

static test_mediator_t mediator;

static void fail(uint32_t stage, uint32_t detail) {
  flexe_emac_result[31] = detail;
  flexe_emac_stage = 0xBAD00000u | stage;
}

static uint32_t checksum(const uint8_t *data, size_t length) {
  uint32_t value = 0;
  for (size_t index = 0; index < length; ++index)
    value = (value * 33u) ^ data[index];
  return value;
}

static esp_err_t phyRead(esp_eth_mediator_t *, uint32_t address,
                         uint32_t reg, uint32_t *value) {
  return emac->read_phy_reg(emac, address, reg, value);
}

static esp_err_t phyWrite(esp_eth_mediator_t *, uint32_t address,
                          uint32_t reg, uint32_t value) {
  return emac->write_phy_reg(emac, address, reg, value);
}

static esp_err_t stackInput(esp_eth_mediator_t *, uint8_t *buffer,
                            uint32_t length) {
  uint32_t index = flexe_emac_result[9]++;
  if (index == 0u) {
    flexe_emac_result[3] = length;
    flexe_emac_result[5] = checksum(buffer, length);
    flexe_emac_result[14] = buffer[0] | ((uint32_t)buffer[length - 1u] << 8u);
  } else if (index == 1u) {
    flexe_emac_result[6] = length;
    flexe_emac_result[8] = checksum(buffer, length);
    flexe_emac_result[15] = buffer[0] | ((uint32_t)buffer[length - 1u] << 8u);
  }
  free(buffer);
  return ESP_OK;
}

static esp_err_t stateChanged(esp_eth_mediator_t *, esp_eth_state_t state,
                              void *) {
  flexe_emac_result[10] |= 1u << (unsigned)state;
  return ESP_OK;
}

static bool waitForHostAndFrames() {
  for (unsigned waited = 0; waited < 10000; ++waited) {
    if (flexe_emac_command == 1u && flexe_emac_result[9] >= 2u)
      return true;
    delay(1);
  }
  return false;
}

void setup() {
  flexe_emac_stage = 1;
  mediator.parent.phy_reg_read = phyRead;
  mediator.parent.phy_reg_write = phyWrite;
  mediator.parent.stack_input = stackInput;
  mediator.parent.on_state_changed = stateChanged;

  eth_mac_config_t config = ETH_MAC_DEFAULT_CONFIG();
  config.smi_mdc_gpio_num = -1;
  config.smi_mdio_gpio_num = -1;
  config.clock_config.rmii.clock_mode = EMAC_CLK_EXT_IN;
  config.clock_config.rmii.clock_gpio = EMAC_CLK_IN_GPIO;
  emac = esp_eth_mac_new_esp32(&config);
  if (!emac) {
    fail(1, 0);
    return;
  }
  flexe_emac_result[0] = emac->set_mediator(emac, &mediator.parent);
  if (flexe_emac_result[0] != ESP_OK) {
    fail(2, flexe_emac_result[0]);
    return;
  }
  flexe_emac_result[13] = emac->init(emac);
  if (flexe_emac_result[13] != ESP_OK) {
    fail(3, flexe_emac_result[13]);
    return;
  }

  uint8_t address[6] = {0x02, 0x11, 0x22, 0x33, 0x44, 0x55};
  if (emac->set_addr(emac, address) != ESP_OK) {
    fail(4, 0);
    return;
  }
  uint32_t phy_value = 0;
  if (emac->read_phy_reg(emac, 1, 2, &phy_value) != ESP_OK) {
    fail(5, 0);
    return;
  }
  flexe_emac_result[2] = phy_value;
  if (emac->write_phy_reg(emac, 1, 4, 0x01E1u) != ESP_OK) {
    fail(6, 0);
    return;
  }
  flexe_emac_result[12] = emac->set_link(emac, ETH_LINK_UP);
  if (flexe_emac_result[12] != ESP_OK) {
    fail(7, flexe_emac_result[12]);
    return;
  }

  uint8_t tx_frame[kTxLength];
  for (size_t index = 0; index < sizeof(tx_frame); ++index)
    tx_frame[index] = (uint8_t)(index * 7u + 3u);
  flexe_emac_result[1] = emac->transmit(emac, tx_frame, sizeof(tx_frame));
  flexe_emac_result[11] = checksum(tx_frame, sizeof(tx_frame));
  if (flexe_emac_result[1] != ESP_OK) {
    fail(8, flexe_emac_result[1]);
    return;
  }

  flexe_emac_stage = 2;
  if (!waitForHostAndFrames()) {
    fail(9, flexe_emac_result[9]);
    return;
  }
  if (flexe_emac_result[3] != 700u || flexe_emac_result[6] != 100u ||
      flexe_emac_result[2] != 0x2000u ||
      flexe_emac_result[14] != (0xA8u << 8u | 0x02u) ||
      flexe_emac_result[15] != (0x82u << 8u | 0x01u)) {
    fail(10, flexe_emac_result[14]);
    return;
  }

  flexe_emac_result[16] = emac->set_link(emac, ETH_LINK_DOWN);
  flexe_emac_result[17] = emac->deinit(emac);
  flexe_emac_result[18] = emac->del(emac);
  emac = nullptr;
  if (flexe_emac_result[16] != ESP_OK ||
      flexe_emac_result[17] != ESP_OK ||
      flexe_emac_result[18] != ESP_OK) {
    fail(11, flexe_emac_result[16]);
    return;
  }

  flexe_emac_result[31] = 0;
  flexe_emac_stage = kSuccessMarker;
}

void loop() {
  delay(1000);
}
