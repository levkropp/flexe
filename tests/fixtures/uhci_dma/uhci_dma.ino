#include <Arduino.h>
#include <esp_intr_alloc.h>
#include <rom/lldesc.h>
#include <soc/dport_reg.h>
#include <soc/soc.h>
#include <soc/uhci_reg.h>

volatile uint32_t flexe_uhci_stage = 0;
volatile uint32_t flexe_uhci_result[16] = {};
volatile uint32_t flexe_uhci_tx_calls = 0;
volatile uint32_t flexe_uhci_rx_calls = 0;

static intr_handle_t uhciHandle = nullptr;
static DRAM_ATTR lldesc_t txDesc __attribute__((aligned(16)));
static DRAM_ATTR lldesc_t rxDesc __attribute__((aligned(16)));
static DRAM_ATTR uint8_t txBuffer[16] __attribute__((aligned(16)));
static DRAM_ATTR uint8_t rxBuffer[16] __attribute__((aligned(16)));

static void fail(uint32_t stage, uint32_t detail) {
  flexe_uhci_result[15] = detail;
  flexe_uhci_stage = 0xBAD00000u | stage;
}

static void IRAM_ATTR onUhci(void *) {
  uint32_t status = REG_READ(UHCI_INT_ST_REG(0));
  if (status & (UHCI_OUT_DONE_INT_ST | UHCI_OUT_EOF_INT_ST |
                UHCI_OUT_TOTAL_EOF_INT_ST)) {
    ++flexe_uhci_tx_calls;
  }
  if (status & (UHCI_IN_DONE_INT_ST | UHCI_IN_SUC_EOF_INT_ST)) {
    ++flexe_uhci_rx_calls;
  }
  flexe_uhci_result[3] |= status;
  REG_WRITE(UHCI_INT_CLR_REG(0), status);
}

static void prepareDescriptors() {
  memset((void *)&txDesc, 0, sizeof(txDesc));
  memset((void *)&rxDesc, 0, sizeof(rxDesc));
  memset(rxBuffer, 0, sizeof(rxBuffer));
  for (size_t index = 0; index < sizeof(txBuffer); ++index) {
    txBuffer[index] = (uint8_t)(0x5Au ^ index);
  }

  txDesc.size = sizeof(txBuffer);
  txDesc.length = sizeof(txBuffer);
  txDesc.eof = 1;
  txDesc.owner = 1;
  txDesc.buf = txBuffer;
  txDesc.empty = 0;

  rxDesc.size = sizeof(rxBuffer);
  rxDesc.length = 0;
  rxDesc.eof = 0;
  rxDesc.owner = 1;
  rxDesc.buf = rxBuffer;
  rxDesc.empty = 0;
}

void setup() {
  flexe_uhci_stage = 1;
  DPORT_SET_PERI_REG_MASK(DPORT_PERIP_CLK_EN_REG,
                          DPORT_UHCI0_CLK_EN | DPORT_UHCI1_CLK_EN);
  DPORT_CLEAR_PERI_REG_MASK(DPORT_PERIP_RST_EN_REG,
                            DPORT_UHCI0_RST | DPORT_UHCI1_RST);

  /* Exercise the genuine register ABI in the transparent mode used by the
     production UHCI driver: no separator, packet header, or CRC transform. */
  REG_WRITE(UHCI_CONF0_REG(0),
            UHCI_CLK_EN | UHCI_UART0_CE | UHCI_UART_IDLE_EOF_EN);
  REG_WRITE(UHCI_CONF1_REG(0), UHCI_CHECK_OWNER);
  REG_WRITE(UHCI_INT_CLR_REG(0), 0x1FFFFu);
  flexe_uhci_result[0] = (uint32_t)esp_intr_alloc(
      ETS_UHCI0_INTR_SOURCE, ESP_INTR_FLAG_LEVEL1 | ESP_INTR_FLAG_IRAM,
      onUhci, nullptr, &uhciHandle);
  if (flexe_uhci_result[0] != ESP_OK || uhciHandle == nullptr) {
    fail(1, flexe_uhci_result[0]);
    return;
  }

  prepareDescriptors();
  REG_WRITE(UHCI_INT_ENA_REG(0),
            UHCI_OUT_DONE_INT_ENA | UHCI_OUT_EOF_INT_ENA |
                UHCI_OUT_TOTAL_EOF_INT_ENA | UHCI_IN_DONE_INT_ENA |
                UHCI_IN_SUC_EOF_INT_ENA | UHCI_IN_DSCR_ERR_INT_ENA |
                UHCI_OUT_DSCR_ERR_INT_ENA);
  REG_WRITE(UHCI_DMA_IN_LINK_REG(0),
            ((uint32_t)&rxDesc & UHCI_INLINK_ADDR_M) |
                UHCI_INLINK_AUTO_RET | UHCI_INLINK_START);
  REG_WRITE(UHCI_DMA_OUT_LINK_REG(0),
            ((uint32_t)&txDesc & UHCI_OUTLINK_ADDR_M) |
                UHCI_OUTLINK_START);

  /* The host runner injects UART0 bytes after observing this stage. */
  flexe_uhci_stage = 2;
  for (unsigned wait = 0;
       wait < 100u && (flexe_uhci_tx_calls == 0u ||
                       flexe_uhci_rx_calls == 0u);
       ++wait) {
    delay(1);
  }
  if (flexe_uhci_tx_calls == 0u || flexe_uhci_rx_calls == 0u) {
    fail(2, (flexe_uhci_tx_calls << 16) | flexe_uhci_rx_calls);
    return;
  }

  flexe_uhci_result[1] = flexe_uhci_tx_calls;
  flexe_uhci_result[2] = flexe_uhci_rx_calls;
  flexe_uhci_result[4] = (rxDesc.length & 0xFFFu) |
                          (rxDesc.eof ? 1u << 16 : 0u) |
                          (rxDesc.owner ? 1u << 17 : 0u);
  flexe_uhci_result[5] = txDesc.owner;
  flexe_uhci_result[6] = REG_READ(UHCI_DMA_OUT_EOF_DES_ADDR_REG(0));
  flexe_uhci_result[7] = REG_READ(UHCI_DMA_IN_SUC_EOF_DES_ADDR_REG(0));
  flexe_uhci_result[8] = REG_READ(UHCI_DMA_OUT_DSCR_REG(0));
  flexe_uhci_result[9] = REG_READ(UHCI_DMA_IN_DSCR_REG(0));
  flexe_uhci_result[10] = REG_READ(UHCI_CONF0_REG(0));
  flexe_uhci_result[11] = REG_READ(UHCI_CONF1_REG(0));
  flexe_uhci_result[12] = REG_READ(UHCI_DMA_OUT_LINK_REG(0));
  flexe_uhci_result[13] = REG_READ(UHCI_DMA_IN_LINK_REG(0));

  for (size_t index = 0; index < sizeof(rxBuffer); ++index) {
    uint8_t expected = (uint8_t)(0xA0u ^ index);
    if (rxBuffer[index] != expected) {
      fail(3, ((uint32_t)index << 16) | rxBuffer[index]);
      return;
    }
  }
  if (rxDesc.length != sizeof(rxBuffer) || !rxDesc.eof || rxDesc.owner ||
      txDesc.owner || flexe_uhci_result[6] != (uint32_t)&txDesc ||
      flexe_uhci_result[7] != (uint32_t)&rxDesc ||
      flexe_uhci_result[8] != (uint32_t)&txDesc ||
      flexe_uhci_result[9] != (uint32_t)&rxDesc) {
    fail(4, flexe_uhci_result[4]);
    return;
  }

  /* UHCI1 owns an independent register file and reset domain. */
  REG_WRITE(UHCI_ESCAPE_CONF_REG(1), 0xA5u);
  flexe_uhci_result[14] = REG_READ(UHCI_ESCAPE_CONF_REG(1));
  if (flexe_uhci_result[14] != 0xA5u ||
      REG_READ(UHCI_ESCAPE_CONF_REG(0)) != 0x33u) {
    fail(5, flexe_uhci_result[14]);
    return;
  }

  REG_WRITE(UHCI_INT_ENA_REG(0), 0);
  REG_WRITE(UHCI_INT_CLR_REG(0), 0x1FFFFu);
  esp_intr_free(uhciHandle);
  flexe_uhci_result[15] = 0;
  flexe_uhci_stage = 0x55484349u;  // "UHCI"
}

void loop() {
  delay(1000);
}
