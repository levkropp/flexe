#include <Arduino.h>
#include <esp_intr_alloc.h>
#include <driver/periph_ctrl.h>
#include <hal/emac_hal.h>
#include <hal/emac_ll.h>
#include <soc/soc.h>

volatile uint32_t flexe_emac_stage = 0;
volatile uint32_t flexe_emac_command = 0;
volatile uint32_t flexe_emac_result[32] = {};

static constexpr uint32_t kSuccessMarker = 0x454D4143u;  // "EMAC"
static constexpr size_t kTxLength = 700;
static constexpr size_t kRx1Length = 700;
static constexpr size_t kRx2Length = 100;

static emac_hal_context_t hal_context;
static uint8_t descriptors[
    (CONFIG_ETH_DMA_RX_BUFFER_NUM + CONFIG_ETH_DMA_TX_BUFFER_NUM) *
    sizeof(eth_dma_rx_descriptor_t)] __attribute__((aligned(4)));
static uint8_t rx_storage[CONFIG_ETH_DMA_RX_BUFFER_NUM]
                         [CONFIG_ETH_DMA_BUFFER_SIZE]
    __attribute__((aligned(4)));
static uint8_t tx_storage[CONFIG_ETH_DMA_TX_BUFFER_NUM]
                         [CONFIG_ETH_DMA_BUFFER_SIZE]
    __attribute__((aligned(4)));
static uint8_t *rx_buffers[CONFIG_ETH_DMA_RX_BUFFER_NUM];
static uint8_t *tx_buffers[CONFIG_ETH_DMA_TX_BUFFER_NUM];
static uint8_t tx_frame[kTxLength];
static uint8_t rx_frame[kRx1Length + 32];
static intr_handle_t interrupt_handle;

static void fail(uint32_t stage, uint32_t detail) {
  flexe_emac_result[31] = detail;
  flexe_emac_stage = 0xBAD00000u | stage;
}

static bool waitForCommand(uint32_t command) {
  for (unsigned waited = 0;
       waited < 10000 && flexe_emac_command != command; ++waited) {
    delay(1);
  }
  return flexe_emac_command == command;
}

static void IRAM_ATTR emacInterrupt(void *) {
  uint32_t status = emac_ll_get_intr_status(&EMAC_DMA);
  flexe_emac_result[9]++;
  flexe_emac_result[10] |= status;
  emac_ll_clear_corresponding_intr(&EMAC_DMA, status);
}

static uint32_t checksum(const uint8_t *data, size_t length) {
  uint32_t value = 0;
  for (size_t index = 0; index < length; ++index)
    value = (value * 33u) ^ data[index];
  return value;
}

void setup() {
  flexe_emac_stage = 1;
  for (unsigned index = 0; index < CONFIG_ETH_DMA_RX_BUFFER_NUM; ++index)
    rx_buffers[index] = rx_storage[index];
  for (unsigned index = 0; index < CONFIG_ETH_DMA_TX_BUFFER_NUM; ++index)
    tx_buffers[index] = tx_storage[index];
  memset(descriptors, 0, sizeof(descriptors));

  /* Drive the genuine ESP-IDF HAL against Flexe's three register windows. */
  periph_module_enable(PERIPH_EMAC_MODULE);
  EMAC_DMA.dmabusmode.sw_rst = 1;
  for (unsigned waited = 0;
       EMAC_DMA.dmabusmode.sw_rst && waited < 10000; ++waited) {
  }
  if (EMAC_DMA.dmabusmode.sw_rst) {
    fail(1, EMAC_DMA.dmabusmode.val);
    return;
  }

  emac_hal_init(&hal_context, descriptors, rx_buffers, tx_buffers);
  emac_hal_reset_desc_chain(&hal_context);
  emac_hal_init_mac_default(&hal_context);
  emac_hal_init_dma_default(&hal_context);
  emac_ll_clock_enable_rmii_input(&EMAC_EXT);
  uint8_t mac_address[6] = {0x02, 0x11, 0x22, 0x33, 0x44, 0x55};
  emac_hal_set_address(&hal_context, mac_address);

  esp_err_t err = esp_intr_alloc(ETS_ETH_MAC_INTR_SOURCE,
                                 ESP_INTR_FLAG_IRAM,
                                 emacInterrupt, nullptr,
                                 &interrupt_handle);
  flexe_emac_result[0] = (uint32_t)err;
  if (err != ESP_OK) {
    fail(2, (uint32_t)err);
    return;
  }
  emac_hal_start(&hal_context);

  /* Exercise Clause-22 reads and writes through the real HAL command path. */
  emac_hal_set_phy_cmd(&hal_context, 1, 2, false);
  for (unsigned waited = 0;
       emac_ll_is_mii_busy(&EMAC_MAC) && waited < 10000; ++waited) {
  }
  flexe_emac_result[2] = emac_ll_get_phy_data(&EMAC_MAC);
  emac_ll_set_phy_data(&EMAC_MAC, 0x01E1u);
  emac_hal_set_phy_cmd(&hal_context, 1, 4, true);
  for (unsigned waited = 0;
       emac_ll_is_mii_busy(&EMAC_MAC) && waited < 10000; ++waited) {
  }

  for (size_t index = 0; index < sizeof(tx_frame); ++index)
    tx_frame[index] = (uint8_t)(index * 7u + 3u);
  flexe_emac_result[1] =
      emac_hal_transmit_frame(&hal_context, tx_frame, sizeof(tx_frame));
  flexe_emac_result[11] = checksum(tx_frame, sizeof(tx_frame));
  if (flexe_emac_result[1] != sizeof(tx_frame)) {
    fail(3, flexe_emac_result[1]);
    return;
  }

  flexe_emac_stage = 2;
  if (!waitForCommand(1)) {
    fail(4, flexe_emac_command);
    return;
  }

  uint32_t frames_remain = 0;
  uint32_t free_descriptors = 0;
  memset(rx_frame, 0, sizeof(rx_frame));
  flexe_emac_result[3] = emac_hal_receive_frame(
      &hal_context, rx_frame, sizeof(rx_frame),
      &frames_remain, &free_descriptors);
  flexe_emac_result[4] =
      (frames_remain & 0xFFFFu) | (free_descriptors << 16);
  flexe_emac_result[5] = checksum(rx_frame, flexe_emac_result[3]);
  if (flexe_emac_result[3] != kRx1Length ||
      rx_frame[0] != 0x02 || rx_frame[699] != (uint8_t)(699u * 5u + 1u)) {
    fail(5, flexe_emac_result[3]);
    return;
  }

  memset(rx_frame, 0, sizeof(rx_frame));
  frames_remain = 0;
  free_descriptors = 0;
  flexe_emac_result[6] = emac_hal_receive_frame(
      &hal_context, rx_frame, sizeof(rx_frame),
      &frames_remain, &free_descriptors);
  flexe_emac_result[7] =
      (frames_remain & 0xFFFFu) | (free_descriptors << 16);
  flexe_emac_result[8] = checksum(rx_frame, flexe_emac_result[6]);
  if (flexe_emac_result[6] != kRx2Length ||
      rx_frame[0] != 0x01 || rx_frame[99] != (uint8_t)(99u * 9u + 7u)) {
    fail(6, flexe_emac_result[6]);
    return;
  }

  if (flexe_emac_result[2] != 0x2000u ||
      flexe_emac_result[9] < 2u ||
      !(flexe_emac_result[10] & (1u << 6))) {
    fail(7, flexe_emac_result[10]);
    return;
  }
  flexe_emac_result[12] = (uint32_t)emac_hal_stop(&hal_context);
  esp_intr_free(interrupt_handle);
  if (flexe_emac_result[12] != ESP_OK) {
    fail(8, flexe_emac_result[12]);
    return;
  }

  flexe_emac_result[31] = 0;
  flexe_emac_stage = kSuccessMarker;
}

void loop() {
  delay(1000);
}
