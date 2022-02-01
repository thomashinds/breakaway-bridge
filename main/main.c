/*
 * Licensed to the Apache Software Foundation (ASF) under one
 * or more contributor license agreements.  See the NOTICE file
 * distributed with this work for additional information
 * regarding copyright ownership.  The ASF licenses this file
 * to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance
 * with the License.  You may obtain a copy of the License at
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
 * KIND, either express or implied.  See the License for the
 * specific language governing permissions and limitations
 * under the License.
 */

#include "driver/uart.h"
#include "esp_log.h"

#include "ble-trainer.h"


void uart_stuff() {
    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = {
        .baud_rate = 19200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    int intr_alloc_flags = 0;


    ESP_ERROR_CHECK(uart_driver_install(2 /* port */, 1024 * 2, 256, 0, NULL, intr_alloc_flags));
    ESP_ERROR_CHECK(uart_param_config(2 /* port */, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(2 /* port */, 14, 13, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    // Configure a temporary buffer for the incoming data
    uint8_t *data = (uint8_t *) malloc(1024);

    uint8_t init[4] = {0xFE, 0x00, 0xFE, 0xF6};
    uint8_t get_power[4] = {0xF5, 0x44, 0x39, 0xF6};

    while (1) {
        int wlen = uart_write_bytes(2, get_power, 4);

        // Read data from the UART
        int len = uart_read_bytes(2, data, 1024, 20 / portTICK_RATE_MS);
        if (len > 0) {
            uint8_t dir = data[0];
            uint8_t metric = data[1];
            uint8_t payload_len = data[2];

            // ESP_LOGI("", "dir: %x, metric: %x, len: %x\n", dir, metric, payload_len);

            if (dir == 0xF1 && metric == 0x44) {
                assert(payload_len == 5);
                assert(len == 10);

                int power_deciwatts = 0;
                int power_10 = 1;
                for (int place = 0; place < 5; place++) {

                    power_deciwatts += power_10 * (data[3 + place] - (int) '0');

                    power_10 *= 10;
                }

                ESP_LOGI("", "power: %d deciwatts\n", power_deciwatts);
                notify_power(power_deciwatts / 10);
            }
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        // Write data back to the UART
        // ESP_LOGI(tag, "uart: %d\n", len);
    }
}

void app_main(void)
{
    ble_init();

    uart_stuff();
}
