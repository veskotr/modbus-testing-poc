/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */
#include <inttypes.h>
#include "driver/gpio.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_modbus_master.h" // ESP-IDF Modbus component

#define HEATER_GPIO 22
#define FAN_GPIO 21
#define AUX_GPIO 19
#define AUX2_GPIO 18
#define NEXTION_RX_GPIO 33
#define NEXTION_TX_GPIO 32
#define BUZZER_GPIO 23

#define NUM_TEST_PINS 10

// static const uint8_t test_pins[NUM_TEST_PINS] = {HEATER_GPIO, FAN_GPIO, AUX_GPIO, AUX2_GPIO, RS485_TX_GPIO, RS485_RX_GPIO, NEXTION_RX_GPIO, NEXTION_TX_GPIO, RS485_DE_GPIO, BUZZER_GPIO};

// RS485/UART pins
#define RS485_TX_GPIO 27
#define RS485_RX_GPIO 26
#define RS485_DE_GPIO 25
#define RS485_UART_PORT UART_NUM_2

#define SLAVE_ADDR 1 // MS9024 Modbus slave ID
#define BAUD_RATE 9600

// Modbus function codes
#define MB_FUNC_READ_COILS 0x01
#define MB_FUNC_READ_DISCRETE_INPUTS 0x02
#define MB_FUNC_READ_HOLDING_REGISTER 0x03
#define MB_FUNC_READ_INPUT_REGISTER 0x04
#define MB_FUNC_WRITE_SINGLE_COIL 0x05
#define MB_FUNC_WRITE_SINGLE_REGISTER 0x06
#define MB_FUNC_WRITE_MULTIPLE_REGISTERS 0x10

#define SENSOR_TYPE_REGISTER_ADDRESS 0x1C
#define SENSOR_TYPE_PT100 0x14

#define SENSOR_WIRING_MODE_REGISTER 0x20
#define SENSOR_WIRING_MODE_4_WIRE 0x04

#define PV_MIN 26
#define PV_MAX 27

static const char *TAG = "RTD_TEST";

static void *master_handle = NULL;

esp_err_t readFloatRegister(uint8_t slave_addr, uint16_t reg, float *result);
esp_err_t writeFloatRegister(uint8_t slave_addr, uint16_t reg, float value);

esp_err_t readRegister(uint8_t slave_addr, uint16_t reg, uint16_t *value);
esp_err_t writeRegister(uint8_t slave_addr, uint16_t reg, uint16_t value);

static float registers_to_float(uint16_t *regs)
{
    union
    {
        uint16_t w[2];
        float f;
    } u;

    // Original ordering: regs[0] = MSB, regs[1] = LSB
    u.w[0] = regs[0];
    u.w[1] = regs[1];
    float v1 = u.f;

    // Swap words
    u.w[0] = regs[1];
    u.w[1] = regs[0];
    float v2 = u.f;

    // Swap bytes inside words
    u.w[0] = (regs[0] >> 8) | (regs[0] << 8);
    u.w[1] = (regs[1] >> 8) | (regs[1] << 8);
    float v3 = u.f;

    // Swap words + swap bytes
    u.w[0] = (regs[1] >> 8) | (regs[1] << 8);
    u.w[1] = (regs[0] >> 8) | (regs[0] << 8);
    float v4 = u.f;

    ESP_LOGI(TAG, "Float variants: v1=%.5f, v2=%.5f, v3=%.5f, v4=%.5f", v1, v2, v3, v4);

    return v1;
}

void app_main(void)
{
    esp_err_t err;

    ESP_LOGI(TAG, "Initializing RS485 UART for Modbus Master...");

    // --- Initialize Modbus Master Controller ---
    mb_communication_info_t comm_info = {
        .ser_opts.port = RS485_UART_PORT,
        .ser_opts.mode = MB_RTU,
        .ser_opts.baudrate = BAUD_RATE,
        .ser_opts.parity = MB_PARITY_NONE,
        .ser_opts.data_bits = UART_DATA_8_BITS,
        .ser_opts.stop_bits = UART_STOP_BITS_1,
        .ser_opts.uid = 0,
        .ser_opts.response_tout_ms = 1000,
    };

    err = mbc_master_create_serial(&comm_info, &master_handle);
    if (err != ESP_OK || master_handle == NULL)
    {
        ESP_LOGE(TAG, "Failed to create Modbus master: %d", err);
        return;
    }

    // Set UART pin numbers and RS485 mode AFTER creation but BEFORE start
    err = uart_set_pin(RS485_UART_PORT, RS485_TX_GPIO, RS485_RX_GPIO, RS485_DE_GPIO, UART_PIN_NO_CHANGE);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to set UART pins: %d", err);
        mbc_master_delete(master_handle);
        return;
    }

    // Set driver mode to Half Duplex - this enables automatic DE pin control
    err = uart_set_mode(RS485_UART_PORT, UART_MODE_RS485_HALF_DUPLEX);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to set UART mode: %d", err);
        mbc_master_delete(master_handle);
        return;
    }

    vTaskDelay(pdMS_TO_TICKS(5)); // Small delay for hardware to settle

    err = mbc_master_start(master_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to start Modbus master controller: %d", err);
        return;
    }

    ESP_LOGI(TAG, "Modbus master started.");

    // --- Allocate buffer for 1 float (2 registers) ---
    uint16_t regs[2];
    const uint16_t rtd_pv_reg = 728 + 512; // PV in IEEE-754 float mode

    writeRegister(SLAVE_ADDR, SENSOR_TYPE_REGISTER_ADDRESS, SENSOR_TYPE_PT100);
    vTaskDelay(pdMS_TO_TICKS(100)); // 1 second delay
    /* writeRegister(SLAVE_ADDR, PV_MAX, 200);
    vTaskDelay(pdMS_TO_TICKS(100)); // 1 second delay
 */
    while (1)
    {

        float value;
        err = readFloatRegister(SLAVE_ADDR, 728 + 512 -1, &value);

        if (err == ESP_OK)
        {
            ESP_LOGI(TAG, "RTD PV = %.2f °C", value);
        }
        else
        {
            ESP_LOGW(TAG, "Failed to read RTD PV: 0x%x", err);
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

esp_err_t writeRegister(uint8_t slave_addr, uint16_t reg, uint16_t value)
{
    mb_param_request_t request = {
        .slave_addr = slave_addr,
        .command = MB_FUNC_WRITE_SINGLE_REGISTER,
        .reg_start = reg,
        .reg_size = 1};

    return mbc_master_send_request(master_handle, &request, &value);
}

esp_err_t readRegister(uint8_t slave_addr, uint16_t reg, uint16_t *value)
{
    mb_param_request_t request = {
        .slave_addr = slave_addr,
        .command = MB_FUNC_READ_HOLDING_REGISTER,
        .reg_start = reg,
        .reg_size = 1};

    return mbc_master_send_request(master_handle, &request, value);
}

esp_err_t writeFloatRegister(uint8_t slave_addr, uint16_t reg, float value)
{
    union
    {
        uint16_t w[2];
        float f;
    } u;

    u.f = value;

    uint16_t regs[2];

    // MS9024 word order
    regs[0] = u.w[1]; // LSB
    regs[1] = u.w[0]; // MSB

    mb_param_request_t request = {
        .slave_addr = slave_addr,
        .command = MB_FUNC_WRITE_MULTIPLE_REGISTERS,
        .reg_start = reg,
        .reg_size = 2};

    return mbc_master_send_request(master_handle, &request, regs);
}

esp_err_t readFloatRegister(uint8_t slave_addr, uint16_t reg, float *result)
{
    uint16_t regs[2];

    mb_param_request_t request = {
        .slave_addr = slave_addr,
        .command = MB_FUNC_READ_HOLDING_REGISTER,
        .reg_start = reg,
        .reg_size = 2};

    esp_err_t err = mbc_master_send_request(master_handle, &request, regs);
    if (err != ESP_OK)
        return err;

    ESP_LOGI(TAG, "Raw regs: 0x%04X 0x%04X", regs[0], regs[1]);

    *result = registers_to_float(regs);
    return ESP_OK;
}
