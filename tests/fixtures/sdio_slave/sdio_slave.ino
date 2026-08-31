#include <Arduino.h>
#include <driver/sdio_slave.h>

volatile uint32_t flexe_sdio_stage = 0;
volatile uint32_t flexe_sdio_command = 0;
volatile uint32_t flexe_sdio_result[24] = {};

static constexpr uint32_t kSuccessMarker = 0x5344494Fu;  // "SDIO"
static constexpr size_t kRecvBufferSize = 16;
static constexpr size_t kSendLength = 21;
static constexpr uintptr_t kSendArgument = 0x13579BDFu;

static DRAM_ATTR uint8_t recvBuffer0[kRecvBufferSize]
    __attribute__((aligned(4)));
static DRAM_ATTR uint8_t recvBuffer1[kRecvBufferSize]
    __attribute__((aligned(4)));
static DRAM_ATTR uint8_t sendBuffer[24] __attribute__((aligned(4)));
static sdio_slave_buf_handle_t recvHandle0;
static sdio_slave_buf_handle_t recvHandle1;
static volatile uint32_t eventMask;
static volatile uint32_t eventCount;

static void fail(uint32_t stage, uint32_t detail) {
  flexe_sdio_result[23] = detail;
  flexe_sdio_stage = 0xBAD00000u | stage;
}

static uint32_t checksum(const uint8_t *data, size_t length) {
  uint32_t value = 0;
  for (size_t index = 0; index < length; ++index)
    value = (value * 33u) ^ data[index];
  return value;
}

static void onHostInterrupt(uint8_t event) {
  eventMask |= 1u << event;
  ++eventCount;
}

void setup() {
  flexe_sdio_stage = 1;
  memset(recvBuffer0, 0, sizeof(recvBuffer0));
  memset(recvBuffer1, 0, sizeof(recvBuffer1));
  for (size_t index = 0; index < sizeof(sendBuffer); ++index)
    sendBuffer[index] = (uint8_t)(0x40u + index * 3u);

  sdio_slave_config_t config = {};
  config.timing = SDIO_SLAVE_TIMING_PSEND_PSAMPLE;
  config.sending_mode = SDIO_SLAVE_SEND_PACKET;
  config.send_queue_size = 4;
  config.recv_buffer_size = kRecvBufferSize;
  config.event_cb = onHostInterrupt;
  config.flags = 0;

  flexe_sdio_result[0] = sdio_slave_initialize(&config);
  if (flexe_sdio_result[0] != ESP_OK) {
    fail(1, flexe_sdio_result[0]);
    return;
  }

  recvHandle0 = sdio_slave_recv_register_buf(recvBuffer0);
  recvHandle1 = sdio_slave_recv_register_buf(recvBuffer1);
  flexe_sdio_result[1] = (recvHandle0 ? 1u : 0u) |
                          (recvHandle1 ? 2u : 0u);
  if (!recvHandle0 || !recvHandle1) {
    fail(2, flexe_sdio_result[1]);
    return;
  }
  flexe_sdio_result[2] = sdio_slave_recv_load_buf(recvHandle0);
  flexe_sdio_result[3] = sdio_slave_recv_load_buf(recvHandle1);
  if (flexe_sdio_result[2] != ESP_OK ||
      flexe_sdio_result[3] != ESP_OK) {
    fail(3, (flexe_sdio_result[2] << 16) | flexe_sdio_result[3]);
    return;
  }

  flexe_sdio_result[4] = sdio_slave_start();
  flexe_sdio_result[5] = sdio_slave_write_reg(5, 0xA5u);
  sdio_slave_set_host_intena((sdio_slave_hostint_t)(
      SDIO_SLAVE_HOSTINT_BIT2 | SDIO_SLAVE_HOSTINT_SEND_NEW_PACKET));
  flexe_sdio_result[6] = (uint32_t)sdio_slave_get_host_intena();
  flexe_sdio_result[7] = sdio_slave_send_host_int(2);
  flexe_sdio_result[8] = sdio_slave_send_queue(
      sendBuffer, kSendLength, (void *)kSendArgument, 0);
  if (flexe_sdio_result[4] != ESP_OK ||
      flexe_sdio_result[5] != ESP_OK ||
      flexe_sdio_result[7] != ESP_OK ||
      flexe_sdio_result[8] != ESP_OK) {
    fail(4, flexe_sdio_result[8]);
    return;
  }

  /* The host runner completes both directions, updates shared register 5,
     injects interrupt 3, then sets flexe_sdio_command. */
  flexe_sdio_stage = 2;
}

void loop() {
  if (flexe_sdio_stage != 2u || flexe_sdio_command != 1u) {
    delay(1);
    return;
  }
  flexe_sdio_command = 2u;

  flexe_sdio_result[9] = sdio_slave_wait_int(3, 0);
  flexe_sdio_result[10] = eventMask;
  flexe_sdio_result[11] = eventCount;

  sdio_slave_buf_handle_t returned0 = nullptr;
  sdio_slave_buf_handle_t returned1 = nullptr;
  flexe_sdio_result[12] = sdio_slave_recv_packet(&returned0, 0);
  size_t length0 = 0;
  uint8_t *data0 = sdio_slave_recv_get_buf(returned0, &length0);
  flexe_sdio_result[13] = length0;
  flexe_sdio_result[14] = data0 ? checksum(data0, length0) : 0;

  flexe_sdio_result[15] = sdio_slave_recv_packet(&returned1, 0);
  size_t length1 = 0;
  uint8_t *data1 = sdio_slave_recv_get_buf(returned1, &length1);
  flexe_sdio_result[16] = length1;
  flexe_sdio_result[17] = data1 ? checksum(data1, length1) : 0;

  void *finishedArgument = nullptr;
  flexe_sdio_result[18] =
      sdio_slave_send_get_finished(&finishedArgument, 0);
  flexe_sdio_result[19] = (uint32_t)finishedArgument;
  flexe_sdio_result[20] = sdio_slave_read_reg(5);

  bool dataOk = data0 != nullptr && data1 != nullptr;
  for (size_t index = 0; dataOk && index < length0; ++index)
    if (data0[index] != (uint8_t)(0xC0u ^ index)) dataOk = false;
  for (size_t index = 0; dataOk && index < length1; ++index)
    if (data1[index] != (uint8_t)(0xC0u ^ (index + length0)))
      dataOk = false;

  if (flexe_sdio_result[9] != pdTRUE ||
      (eventMask & (1u << 3)) == 0u || eventCount == 0u ||
      flexe_sdio_result[12] != ESP_ERR_NOT_FINISHED ||
      length0 != kRecvBufferSize ||
      flexe_sdio_result[15] != ESP_OK || length1 != 8u || !dataOk ||
      flexe_sdio_result[18] != ESP_OK ||
      (uintptr_t)finishedArgument != kSendArgument ||
      flexe_sdio_result[20] != 0x5Au) {
    fail(5, (flexe_sdio_result[12] << 16) |
                (flexe_sdio_result[15] & 0xFFFFu));
    return;
  }

  sdio_slave_stop();
  sdio_slave_deinit();
  flexe_sdio_result[21] = 1u;
  flexe_sdio_result[22] = checksum(sendBuffer, kSendLength);
  flexe_sdio_result[23] = 0u;
  flexe_sdio_stage = kSuccessMarker;
}
