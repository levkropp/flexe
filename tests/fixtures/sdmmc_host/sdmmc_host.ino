#include <Arduino.h>
#include <driver/sdmmc_defs.h>
#include <driver/sdmmc_host.h>
#include <driver/sdmmc_types.h>

volatile uint32_t flexe_sdmmc_stage = 0;
volatile uint32_t flexe_sdmmc_result[24] = {};

static constexpr size_t kSectorBytes = 512u;
static constexpr size_t kMultiBytes = 20u * 1024u;
static uint8_t dmaBuffer[kMultiBytes] __attribute__((aligned(4)));

static void fail(uint32_t stage, uint32_t detail) {
  flexe_sdmmc_result[23] = detail;
  flexe_sdmmc_stage = 0xBAD00000u | stage;
}

static esp_err_t command(uint32_t opcode, uint32_t argument, int flags,
                         void *data = nullptr, size_t length = 0,
                         size_t blockLength = 0,
                         uint32_t *response = nullptr) {
  sdmmc_command_t cmd = {};
  cmd.opcode = opcode;
  cmd.arg = argument;
  cmd.data = data;
  cmd.datalen = length;
  cmd.blklen = blockLength;
  cmd.flags = flags;
  cmd.timeout_ms = 1000;
  esp_err_t err = sdmmc_host_do_transaction(SDMMC_HOST_SLOT_1, &cmd);
  if (response != nullptr) {
    for (unsigned index = 0; index < 4; ++index)
      response[index] = cmd.response[index];
  }
  return err;
}

static uint8_t cardPattern(uint32_t sector, size_t offset) {
  return (uint8_t)(0x61u ^ (sector * 7u + offset));
}

void setup() {
  uint32_t response[4] = {};
  flexe_sdmmc_stage = 1;
  esp_err_t err = sdmmc_host_init();
  flexe_sdmmc_result[0] = (uint32_t)err;
  if (err != ESP_OK) {
    fail(1, (uint32_t)err);
    return;
  }

  sdmmc_slot_config_t slot = SDMMC_SLOT_CONFIG_DEFAULT();
  slot.width = 1;
  slot.flags = SDMMC_SLOT_FLAG_INTERNAL_PULLUP;
  err = sdmmc_host_init_slot(SDMMC_HOST_SLOT_1, &slot);
  flexe_sdmmc_result[1] = (uint32_t)err;
  if (err != ESP_OK) {
    fail(2, (uint32_t)err);
    return;
  }
  err = sdmmc_host_set_card_clk(SDMMC_HOST_SLOT_1, SDMMC_FREQ_PROBING);
  flexe_sdmmc_result[2] = (uint32_t)err;
  if (err != ESP_OK) {
    fail(3, (uint32_t)err);
    return;
  }

  flexe_sdmmc_stage = 2;
  err = command(MMC_GO_IDLE_STATE, 0, SCF_CMD_BC | SCF_RSP_R0);
  flexe_sdmmc_result[3] = (uint32_t)err;
  if (err != ESP_OK) {
    fail(4, (uint32_t)err);
    return;
  }
  err = command(SD_SEND_IF_COND, 0x1AAu,
                SCF_CMD_BCR | SCF_RSP_R7, nullptr, 0, 0, response);
  flexe_sdmmc_result[4] = (uint32_t)err;
  flexe_sdmmc_result[5] = response[0];
  if (err != ESP_OK || response[0] != 0x1AAu) {
    fail(5, response[0]);
    return;
  }
  err = command(MMC_APP_CMD, 0, SCF_CMD_AC | SCF_RSP_R1);
  flexe_sdmmc_result[6] = (uint32_t)err;
  if (err != ESP_OK) {
    fail(6, (uint32_t)err);
    return;
  }
  err = command(SD_APP_OP_COND, SD_OCR_SDHC_CAP | SD_OCR_VOL_MASK,
                SCF_CMD_BCR | SCF_RSP_R3, nullptr, 0, 0, response);
  flexe_sdmmc_result[7] = (uint32_t)err;
  flexe_sdmmc_result[8] = response[0];
  if (err != ESP_OK ||
      (response[0] & (MMC_OCR_MEM_READY | SD_OCR_SDHC_CAP)) !=
          (MMC_OCR_MEM_READY | SD_OCR_SDHC_CAP)) {
    fail(7, response[0]);
    return;
  }

  flexe_sdmmc_stage = 3;
  err = command(MMC_ALL_SEND_CID, 0,
                SCF_CMD_BCR | SCF_RSP_R2, nullptr, 0, 0, response);
  flexe_sdmmc_result[9] = (uint32_t)err;
  flexe_sdmmc_result[10] = response[3];
  if (err != ESP_OK || (response[0] | response[1] |
                        response[2] | response[3]) == 0) {
    fail(8, response[3]);
    return;
  }
  err = command(SD_SEND_RELATIVE_ADDR, 0,
                SCF_CMD_BCR | SCF_RSP_R6, nullptr, 0, 0, response);
  flexe_sdmmc_result[11] = (uint32_t)err;
  uint32_t rca = response[0] & 0xFFFF0000u;
  flexe_sdmmc_result[12] = rca;
  if (err != ESP_OK || rca == 0) {
    fail(9, response[0]);
    return;
  }
  err = command(MMC_SEND_CSD, rca,
                SCF_CMD_AC | SCF_RSP_R2, nullptr, 0, 0, response);
  flexe_sdmmc_result[13] = (uint32_t)err;
  flexe_sdmmc_result[14] = SD_CSD_CSDVER(response);
  if (err != ESP_OK || flexe_sdmmc_result[14] != 1u) {
    fail(10, flexe_sdmmc_result[14]);
    return;
  }
  err = command(MMC_SELECT_CARD, rca, SCF_CMD_AC | SCF_RSP_R1);
  flexe_sdmmc_result[15] = (uint32_t)err;
  if (err != ESP_OK) {
    fail(11, (uint32_t)err);
    return;
  }

  flexe_sdmmc_stage = 4;
  memset(dmaBuffer, 0, kSectorBytes);
  err = command(MMC_READ_BLOCK_SINGLE, 4,
                SCF_CMD_ADTC | SCF_CMD_READ | SCF_RSP_R1,
                dmaBuffer, kSectorBytes, kSectorBytes);
  flexe_sdmmc_result[16] = (uint32_t)err;
  if (err != ESP_OK) {
    fail(12, (uint32_t)err);
    return;
  }
  for (size_t index = 0; index < kSectorBytes; ++index) {
    if (dmaBuffer[index] != cardPattern(4, index)) {
      fail(13, ((uint32_t)index << 8) | dmaBuffer[index]);
      return;
    }
  }
  flexe_sdmmc_result[17] = 1;

  for (size_t index = 0; index < kSectorBytes; ++index)
    dmaBuffer[index] = (uint8_t)(0xC3u ^ index);
  err = command(MMC_WRITE_BLOCK_SINGLE, 5,
                SCF_CMD_ADTC | SCF_RSP_R1,
                dmaBuffer, kSectorBytes, kSectorBytes);
  flexe_sdmmc_result[18] = (uint32_t)err;
  if (err != ESP_OK) {
    fail(14, (uint32_t)err);
    return;
  }

  flexe_sdmmc_stage = 5;
  memset(dmaBuffer, 0, sizeof(dmaBuffer));
  err = command(MMC_READ_BLOCK_MULTIPLE, 8,
                SCF_CMD_ADTC | SCF_CMD_READ | SCF_RSP_R1,
                dmaBuffer, sizeof(dmaBuffer), kSectorBytes);
  flexe_sdmmc_result[19] = (uint32_t)err;
  if (err != ESP_OK) {
    fail(15, (uint32_t)err);
    return;
  }
  for (size_t index = 0; index < sizeof(dmaBuffer); ++index) {
    uint32_t sector = 8u + (uint32_t)(index / kSectorBytes);
    size_t offset = index % kSectorBytes;
    if (dmaBuffer[index] != cardPattern(sector, offset)) {
      fail(16, ((uint32_t)index << 8) | dmaBuffer[index]);
      return;
    }
  }
  flexe_sdmmc_result[20] = 1;

  for (size_t index = 0; index < sizeof(dmaBuffer); ++index)
    dmaBuffer[index] = (uint8_t)(0x96u ^ (index * 3u));
  err = command(MMC_WRITE_BLOCK_MULTIPLE, 64,
                SCF_CMD_ADTC | SCF_RSP_R1,
                dmaBuffer, sizeof(dmaBuffer), kSectorBytes);
  flexe_sdmmc_result[21] = (uint32_t)err;
  if (err != ESP_OK) {
    fail(17, (uint32_t)err);
    return;
  }

  flexe_sdmmc_stage = 6;
  err = sdmmc_host_deinit();
  flexe_sdmmc_result[22] = (uint32_t)err;
  if (err != ESP_OK) {
    fail(18, (uint32_t)err);
    return;
  }
  flexe_sdmmc_result[23] = 0;
  flexe_sdmmc_stage = 0x53444D4Du;  // "SDMM"
}

void loop() {
  delay(1000);
}
