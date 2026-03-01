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

//static const uint8_t test_pins[NUM_TEST_PINS] = {HEATER_GPIO, FAN_GPIO, AUX_GPIO, AUX2_GPIO, RS485_TX_GPIO, RS485_RX_GPIO, NEXTION_RX_GPIO, NEXTION_TX_GPIO, RS485_DE_GPIO, BUZZER_GPIO};

// RS485/UART pins
#define RS485_TX_GPIO 27
#define RS485_RX_GPIO 26
#define RS485_DE_GPIO 25
#define RS485_UART_PORT UART_NUM_2

#define SLAVE_ADDR 1 // MS9024 Modbus slave ID
#define BAUD_RATE 9600

static const char *TAG = "RTD_TEST";

static float registers_to_float(uint16_t *regs)
{
    union {
        uint16_t w[2];
        float f;
    } u;

    u.w[0] = regs[1]; // MSB
    u.w[1] = regs[0]; // LSB
    return u.f;
}

void app_main(void)
{
    esp_err_t err;
    void *master_handle = NULL;

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
    if (err != ESP_OK || master_handle == NULL) {
        ESP_LOGE(TAG, "Failed to create Modbus master: %d", err);
        return;
    }

    // Set UART pin numbers and RS485 mode AFTER creation but BEFORE start
    err = uart_set_pin(RS485_UART_PORT, RS485_TX_GPIO, RS485_RX_GPIO, RS485_DE_GPIO, UART_PIN_NO_CHANGE);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set UART pins: %d", err);
        mbc_master_delete(master_handle);
        return;
    }

    // Set driver mode to Half Duplex - this enables automatic DE pin control
    err = uart_set_mode(RS485_UART_PORT, UART_MODE_RS485_HALF_DUPLEX);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set UART mode: %d", err);
        mbc_master_delete(master_handle);
        return;
    }

    vTaskDelay(pdMS_TO_TICKS(5)); // Small delay for hardware to settle

    err = mbc_master_start(master_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start Modbus master controller: %d", err);
        return;
    }

    ESP_LOGI(TAG, "Modbus master started.");

    // --- Allocate buffer for 1 float (2 registers) ---
    uint16_t regs[2];
    const uint16_t rtd_pv_reg = 728 + 512; // PV in IEEE-754 float mode

    while (1) {
        // --- Read Holding Registers ---
        mb_param_request_t request = {
            .slave_addr = SLAVE_ADDR,
            .command = 3,  // MB_FUNC_READ_HOLDING_REGISTER
            .reg_start = rtd_pv_reg,
            .reg_size = 2
        };
        
        err = mbc_master_send_request(master_handle, &request, regs);
        if (err == ESP_OK) {
            float temperature = registers_to_float(regs);
            ESP_LOGI(TAG, "RTD PV = %.2f °C", temperature);
        } else {
            ESP_LOGW(TAG, "Failed to read RTD PV: 0x%x", err);
        }

        vTaskDelay(pdMS_TO_TICKS(1000)); // 1 second delay
    }
}