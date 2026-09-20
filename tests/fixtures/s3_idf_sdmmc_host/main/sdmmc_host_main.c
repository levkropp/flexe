/* Exercise ESP-IDF 5.3's native SDMMC host driver, ISR queue, FIFO, and
 * DesignWare IDMAC path without replacing any guest driver function. */
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "driver/sdmmc_defs.h"
#include "driver/sdmmc_host.h"
#include "driver/sdmmc_types.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define SDMMC_DONE UINT32_C(0x53444D4D)
#define SECTOR_BYTES 512u
#define MULTI_BYTES (20u * 1024u)

volatile uint32_t flexe_sdmmc_stage;
volatile uint32_t flexe_sdmmc_result[24];

static uint8_t dma_buffer[MULTI_BYTES] __attribute__((aligned(4)));

static void fail(uint32_t stage, uint32_t detail)
{
    flexe_sdmmc_result[23] = detail;
    flexe_sdmmc_stage = UINT32_C(0xBAD00000) | stage;
    printf("SDMMC_FAIL stage=%u detail=0x%08X\n",
           (unsigned)stage, (unsigned)detail);
    fflush(stdout);
    for (;;) vTaskDelay(pdMS_TO_TICKS(100));
}

static esp_err_t command(uint32_t opcode, uint32_t argument, int flags,
                         void *data, size_t length, size_t block_length,
                         uint32_t *response)
{
    sdmmc_command_t cmd = {
        .opcode = opcode,
        .arg = argument,
        .data = data,
        .datalen = length,
        .buflen = length,
        .blklen = block_length,
        .flags = flags,
        .timeout_ms = 1000,
    };
    esp_err_t error = sdmmc_host_do_transaction(SDMMC_HOST_SLOT_1, &cmd);
    if (response)
        memcpy(response, cmd.response, sizeof(cmd.response));
    return error;
}

static uint8_t card_pattern(uint32_t sector, size_t offset)
{
    return (uint8_t)(0x61u ^ (sector * 7u + offset));
}

void app_main(void)
{
    uint32_t response[4] = {0};
    flexe_sdmmc_stage = 1u;
    esp_err_t error = sdmmc_host_init();
    flexe_sdmmc_result[0] = (uint32_t)error;
    if (error != ESP_OK) fail(1u, (uint32_t)error);

    sdmmc_slot_config_t slot = SDMMC_SLOT_CONFIG_DEFAULT();
    slot.width = 1u;
    slot.flags = SDMMC_SLOT_FLAG_INTERNAL_PULLUP;
    error = sdmmc_host_init_slot(SDMMC_HOST_SLOT_1, &slot);
    flexe_sdmmc_result[1] = (uint32_t)error;
    if (error != ESP_OK) fail(2u, (uint32_t)error);
    error = sdmmc_host_set_card_clk(
        SDMMC_HOST_SLOT_1, SDMMC_FREQ_PROBING);
    flexe_sdmmc_result[2] = (uint32_t)error;
    if (error != ESP_OK) fail(3u, (uint32_t)error);

    flexe_sdmmc_stage = 2u;
    error = command(MMC_GO_IDLE_STATE, 0u, SCF_CMD_BC | SCF_RSP_R0,
                    NULL, 0u, 0u, NULL);
    flexe_sdmmc_result[3] = (uint32_t)error;
    if (error != ESP_OK) fail(4u, (uint32_t)error);
    error = command(SD_SEND_IF_COND, 0x1AAu,
                    SCF_CMD_BCR | SCF_RSP_R7,
                    NULL, 0u, 0u, response);
    flexe_sdmmc_result[4] = (uint32_t)error;
    flexe_sdmmc_result[5] = response[0];
    if (error != ESP_OK || response[0] != 0x1AAu)
        fail(5u, response[0]);
    error = command(MMC_APP_CMD, 0u, SCF_CMD_AC | SCF_RSP_R1,
                    NULL, 0u, 0u, NULL);
    flexe_sdmmc_result[6] = (uint32_t)error;
    if (error != ESP_OK) fail(6u, (uint32_t)error);
    error = command(SD_APP_OP_COND, SD_OCR_SDHC_CAP | SD_OCR_VOL_MASK,
                    SCF_CMD_BCR | SCF_RSP_R3,
                    NULL, 0u, 0u, response);
    flexe_sdmmc_result[7] = (uint32_t)error;
    flexe_sdmmc_result[8] = response[0];
    if (error != ESP_OK ||
        (response[0] & (MMC_OCR_MEM_READY | SD_OCR_SDHC_CAP)) !=
            (MMC_OCR_MEM_READY | SD_OCR_SDHC_CAP))
        fail(7u, response[0]);

    flexe_sdmmc_stage = 3u;
    error = command(MMC_ALL_SEND_CID, 0u, SCF_CMD_BCR | SCF_RSP_R2,
                    NULL, 0u, 0u, response);
    flexe_sdmmc_result[9] = (uint32_t)error;
    flexe_sdmmc_result[10] = response[3];
    if (error != ESP_OK ||
        (response[0] | response[1] | response[2] | response[3]) == 0u)
        fail(8u, response[3]);
    error = command(SD_SEND_RELATIVE_ADDR, 0u,
                    SCF_CMD_BCR | SCF_RSP_R6,
                    NULL, 0u, 0u, response);
    flexe_sdmmc_result[11] = (uint32_t)error;
    uint32_t rca = response[0] & UINT32_C(0xFFFF0000);
    flexe_sdmmc_result[12] = rca;
    if (error != ESP_OK || rca == 0u) fail(9u, response[0]);
    error = command(MMC_SEND_CSD, rca, SCF_CMD_AC | SCF_RSP_R2,
                    NULL, 0u, 0u, response);
    flexe_sdmmc_result[13] = (uint32_t)error;
    flexe_sdmmc_result[14] = SD_CSD_CSDVER(response);
    if (error != ESP_OK || flexe_sdmmc_result[14] != 1u)
        fail(10u, flexe_sdmmc_result[14]);
    error = command(MMC_SELECT_CARD, rca, SCF_CMD_AC | SCF_RSP_R1,
                    NULL, 0u, 0u, NULL);
    flexe_sdmmc_result[15] = (uint32_t)error;
    if (error != ESP_OK) fail(11u, (uint32_t)error);

    flexe_sdmmc_stage = 4u;
    memset(dma_buffer, 0, SECTOR_BYTES);
    error = command(MMC_READ_BLOCK_SINGLE, 4u,
                    SCF_CMD_ADTC | SCF_CMD_READ | SCF_RSP_R1,
                    dma_buffer, SECTOR_BYTES, SECTOR_BYTES, NULL);
    flexe_sdmmc_result[16] = (uint32_t)error;
    if (error != ESP_OK) fail(12u, (uint32_t)error);
    for (size_t index = 0u; index < SECTOR_BYTES; index++)
        if (dma_buffer[index] != card_pattern(4u, index))
            fail(13u, ((uint32_t)index << 8u) | dma_buffer[index]);
    flexe_sdmmc_result[17] = 1u;

    for (size_t index = 0u; index < SECTOR_BYTES; index++)
        dma_buffer[index] = (uint8_t)(0xC3u ^ index);
    error = command(MMC_WRITE_BLOCK_SINGLE, 5u,
                    SCF_CMD_ADTC | SCF_RSP_R1,
                    dma_buffer, SECTOR_BYTES, SECTOR_BYTES, NULL);
    flexe_sdmmc_result[18] = (uint32_t)error;
    if (error != ESP_OK) fail(14u, (uint32_t)error);

    flexe_sdmmc_stage = 5u;
    memset(dma_buffer, 0, sizeof(dma_buffer));
    error = command(MMC_READ_BLOCK_MULTIPLE, 8u,
                    SCF_CMD_ADTC | SCF_CMD_READ | SCF_RSP_R1,
                    dma_buffer, sizeof(dma_buffer), SECTOR_BYTES, NULL);
    flexe_sdmmc_result[19] = (uint32_t)error;
    if (error != ESP_OK) fail(15u, (uint32_t)error);
    for (size_t index = 0u; index < sizeof(dma_buffer); index++) {
        uint32_t sector = 8u + (uint32_t)(index / SECTOR_BYTES);
        size_t offset = index % SECTOR_BYTES;
        if (dma_buffer[index] != card_pattern(sector, offset))
            fail(16u, ((uint32_t)index << 8u) | dma_buffer[index]);
    }
    flexe_sdmmc_result[20] = 1u;

    for (size_t index = 0u; index < sizeof(dma_buffer); index++)
        dma_buffer[index] = (uint8_t)(0x96u ^ (index * 3u));
    error = command(MMC_WRITE_BLOCK_MULTIPLE, 64u,
                    SCF_CMD_ADTC | SCF_RSP_R1,
                    dma_buffer, sizeof(dma_buffer), SECTOR_BYTES, NULL);
    flexe_sdmmc_result[21] = (uint32_t)error;
    if (error != ESP_OK) fail(17u, (uint32_t)error);

    flexe_sdmmc_stage = 6u;
    error = sdmmc_host_deinit();
    flexe_sdmmc_result[22] = (uint32_t)error;
    if (error != ESP_OK) fail(18u, (uint32_t)error);
    flexe_sdmmc_result[23] = 0u;
    flexe_sdmmc_stage = SDMMC_DONE;
    printf("SDMMC_DONE\n");
    fflush(stdout);
    for (;;) vTaskDelay(pdMS_TO_TICKS(100));
}
