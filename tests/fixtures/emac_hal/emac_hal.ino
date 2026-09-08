#include <Arduino.h>
#include <esp_idf_version.h>
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

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 4, 0)
static unsigned tx_descriptor_index;
static unsigned rx_descriptor_index;

static eth_dma_rx_descriptor_t *rxDescriptors() {
  return reinterpret_cast<eth_dma_rx_descriptor_t *>(descriptors);
}

static eth_dma_tx_descriptor_t *txDescriptors() {
  return reinterpret_cast<eth_dma_tx_descriptor_t *>(
      descriptors + CONFIG_ETH_DMA_RX_BUFFER_NUM *
                        sizeof(eth_dma_rx_descriptor_t));
}

static void initDescriptorChains() {
  eth_dma_rx_descriptor_t *rx = rxDescriptors();
  eth_dma_tx_descriptor_t *tx = txDescriptors();
  for (unsigned i = 0; i < CONFIG_ETH_DMA_RX_BUFFER_NUM; ++i) {
    memset(&rx[i], 0, sizeof(rx[i]));
    rx[i].RDES0.Own = 1;
    rx[i].RDES1.ReceiveBuffer1Size = CONFIG_ETH_DMA_BUFFER_SIZE;
    rx[i].RDES1.SecondAddressChained = 1;
    rx[i].Buffer1Addr = (uint32_t)(uintptr_t)rx_buffers[i];
    rx[i].Buffer2NextDescAddr =
        (uint32_t)(uintptr_t)&rx[(i + 1u) % CONFIG_ETH_DMA_RX_BUFFER_NUM];
  }
  for (unsigned i = 0; i < CONFIG_ETH_DMA_TX_BUFFER_NUM; ++i) {
    memset(&tx[i], 0, sizeof(tx[i]));
    tx[i].TDES0.SecondAddressChained = 1;
    tx[i].Buffer1Addr = (uint32_t)(uintptr_t)tx_buffers[i];
    tx[i].Buffer2NextDescAddr =
        (uint32_t)(uintptr_t)&tx[(i + 1u) % CONFIG_ETH_DMA_TX_BUFFER_NUM];
  }
  tx_descriptor_index = 0;
  rx_descriptor_index = 0;
}

static uint32_t transmitFrame(const uint8_t *frame, size_t length) {
  eth_dma_tx_descriptor_t *tx = txDescriptors();
  size_t remaining = length;
  unsigned count = (unsigned)((length + CONFIG_ETH_DMA_BUFFER_SIZE - 1u) /
                              CONFIG_ETH_DMA_BUFFER_SIZE);
  if (count == 0u || count > CONFIG_ETH_DMA_TX_BUFFER_NUM) return 0;

  for (unsigned part = 0; part < count; ++part) {
    unsigned index = (tx_descriptor_index + part) % CONFIG_ETH_DMA_TX_BUFFER_NUM;
    if (tx[index].TDES0.Own) return 0;
    size_t chunk = remaining < CONFIG_ETH_DMA_BUFFER_SIZE
                       ? remaining
                       : CONFIG_ETH_DMA_BUFFER_SIZE;
    memcpy(tx_buffers[index], frame + (length - remaining), chunk);
    tx[index].TDES1.Value = 0;
    tx[index].TDES1.TransmitBuffer1Size = (uint32_t)chunk;
    tx[index].TDES0.Value = 0;
    tx[index].TDES0.SecondAddressChained = 1;
    tx[index].TDES0.FirstSegment = part == 0u;
    tx[index].TDES0.LastSegment = part + 1u == count;
    tx[index].TDES0.InterruptOnComplete = part + 1u == count;
    tx[index].TDES0.Own = 1;
    remaining -= chunk;
  }
  tx_descriptor_index = (tx_descriptor_index + count) %
                        CONFIG_ETH_DMA_TX_BUFFER_NUM;
  emac_hal_transmit_poll_demand(&hal_context);
  return (uint32_t)length;
}

static uint32_t receiveFrame(uint8_t *frame, size_t capacity,
                             uint32_t *frames_remain,
                             uint32_t *free_descriptors) {
  eth_dma_rx_descriptor_t *rx = rxDescriptors();
  unsigned indices[CONFIG_ETH_DMA_RX_BUFFER_NUM];
  unsigned count = 0;
  uint32_t wire_length = 0;
  size_t copied = 0;

  while (count < CONFIG_ETH_DMA_RX_BUFFER_NUM) {
    unsigned index = (rx_descriptor_index + count) %
                     CONFIG_ETH_DMA_RX_BUFFER_NUM;
    if (rx[index].RDES0.Own) return 0;
    indices[count++] = index;
    size_t chunk = CONFIG_ETH_DMA_BUFFER_SIZE;
    if (copied + chunk > capacity) chunk = capacity - copied;
    if (chunk != 0u) memcpy(frame + copied, rx_buffers[index], chunk);
    copied += chunk;
    if (rx[index].RDES0.LastDescriptor) {
      wire_length = rx[index].RDES0.FrameLength;
      break;
    }
  }
  if (wire_length < 4u || count == 0u) return 0;
  uint32_t frame_length = wire_length - 4u;
  if (frame_length > capacity) frame_length = (uint32_t)capacity;

  for (unsigned part = 0; part < count; ++part) {
    unsigned index = indices[part];
    rx[index].RDES0.Value = 0;
    rx[index].RDES0.Own = 1;
  }
  rx_descriptor_index = (rx_descriptor_index + count) %
                        CONFIG_ETH_DMA_RX_BUFFER_NUM;
  if (frames_remain) *frames_remain = 0;
  if (free_descriptors) *free_descriptors = CONFIG_ETH_DMA_RX_BUFFER_NUM;
  emac_hal_receive_poll_demand(&hal_context);
  return frame_length;
}
#endif

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

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 4, 0)
  emac_hal_init(&hal_context);
  initDescriptorChains();
  emac_hal_set_rx_tx_desc_addr(&hal_context, rxDescriptors(), txDescriptors());
#else
  emac_hal_init(&hal_context, descriptors, rx_buffers, tx_buffers);
  emac_hal_reset_desc_chain(&hal_context);
#endif
  emac_hal_init_mac_default(&hal_context);
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 4, 0)
  emac_hal_set_promiscuous(&hal_context, true);
  emac_hal_dma_config_t dma_config = {
      .dma_burst_len = ETH_DMA_BURST_LEN_32,
  };
  emac_hal_init_dma_default(&hal_context, &dma_config);
  emac_hal_clock_enable_rmii_input(&hal_context);
#else
  emac_hal_init_dma_default(&hal_context);
  emac_ll_clock_enable_rmii_input(&EMAC_EXT);
#endif
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
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 4, 0)
  flexe_emac_result[1] = transmitFrame(tx_frame, sizeof(tx_frame));
#else
  flexe_emac_result[1] =
      emac_hal_transmit_frame(&hal_context, tx_frame, sizeof(tx_frame));
#endif
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
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 4, 0)
  flexe_emac_result[3] = receiveFrame(
      rx_frame, sizeof(rx_frame), &frames_remain, &free_descriptors);
#else
  flexe_emac_result[3] = emac_hal_receive_frame(
      &hal_context, rx_frame, sizeof(rx_frame),
      &frames_remain, &free_descriptors);
#endif
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
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 4, 0)
  flexe_emac_result[6] = receiveFrame(
      rx_frame, sizeof(rx_frame), &frames_remain, &free_descriptors);
#else
  flexe_emac_result[6] = emac_hal_receive_frame(
      &hal_context, rx_frame, sizeof(rx_frame),
      &frames_remain, &free_descriptors);
#endif
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
