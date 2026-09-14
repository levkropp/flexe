/* Exercise ESP-IDF 5.3's new I2C master driver and interrupt/FIFO path,
 * without replacing any guest driver function. */
#include <stdint.h>
#include <stdio.h>

#include "driver/i2c_master.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define DATA_LENGTH 40u
#define I2C_DONE UINT32_C(0x1C2C5343)

volatile uint32_t flexe_i2c_stage;
volatile uint32_t flexe_i2c_result[4];

static void fail(const char *stage, esp_err_t error)
{
    flexe_i2c_result[3] = (uint32_t)error;
    flexe_i2c_stage = UINT32_C(0xBAD00000);
    printf("I2C_FAIL %s %d\n", stage, (int)error);
    fflush(stdout);
    for (;;) vTaskDelay(pdMS_TO_TICKS(100));
}

void app_main(void)
{
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = GPIO_NUM_8,
        .scl_io_num = GPIO_NUM_9,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = 1,
    };
    i2c_master_bus_handle_t bus;
    esp_err_t error = i2c_new_master_bus(&bus_config, &bus);
    if (error != ESP_OK) fail("bus", error);

    i2c_device_config_t device_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = 0x34,
        .scl_speed_hz = 400000,
    };
    i2c_master_dev_handle_t device;
    error = i2c_master_bus_add_device(bus, &device_config, &device);
    if (error != ESP_OK) fail("device", error);

    uint8_t write_data[DATA_LENGTH + 1u];
    write_data[0] = 0x20;
    for (uint8_t i = 0; i < DATA_LENGTH; i++)
        write_data[i + 1u] = (uint8_t)(0x5Au ^ i);
    flexe_i2c_stage = 1u;
    error = i2c_master_transmit(device, write_data,
                                sizeof(write_data), 1000);
    if (error != ESP_OK) fail("transmit", error);
    flexe_i2c_stage = 2u;

    uint8_t address = 0x20;
    uint8_t read_data[DATA_LENGTH] = {0};
    error = i2c_master_transmit_receive(device, &address, 1u,
                                        read_data, sizeof(read_data), 1000);
    if (error != ESP_OK) fail("transmit_receive", error);
    uint32_t checksum = 0u;
    for (uint8_t i = 0; i < DATA_LENGTH; i++) {
        uint8_t expected = (uint8_t)(0x5Au ^ i);
        if (read_data[i] != expected)
            fail("readback", (esp_err_t)(((uint32_t)i << 8u) |
                                         read_data[i]));
        checksum = checksum * 33u + read_data[i];
    }
    flexe_i2c_result[0] = checksum;
    flexe_i2c_stage = 3u;

    error = i2c_master_probe(bus, 0x35, 1000);
    flexe_i2c_result[1] = (uint32_t)error;
    if (error != ESP_ERR_NOT_FOUND) fail("probe-nack", error);
    error = i2c_master_bus_rm_device(device);
    if (error != ESP_OK) fail("remove-device", error);
    error = i2c_del_master_bus(bus);
    if (error != ESP_OK) fail("remove-bus", error);

    flexe_i2c_stage = I2C_DONE;
    printf("I2C_MASTER_DONE checksum=%08X nack=%d\n",
           (unsigned)checksum, (int)flexe_i2c_result[1]);
    fflush(stdout);
    for (;;) vTaskDelay(pdMS_TO_TICKS(100));
}
