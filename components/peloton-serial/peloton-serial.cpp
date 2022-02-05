#include "peloton-serial.h"

#include "driver/uart.h"
#include "esp_log.h"

PelotonSerial::PelotonSerial(EventHandler &event_handler) : event_handler{event_handler} {
    // Todo defend against these constructors being called more than once
    xTaskCreateStatic(this->TxTask, "PelotonSerialTX", this->TX_STACK_SIZE, this, tskIDLE_PRIORITY + 1,
                          this->tx_task_stack, &this->tx_task_storage);

    xTaskCreateStatic(this->RxTask, "PelotonSerialRX", this->RX_STACK_SIZE, this, tskIDLE_PRIORITY + 4,
                      this->rx_task_stack, &this->rx_task_storage);
}

void PelotonSerial::Init() {
    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = {
        .baud_rate = 19200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
        .source_clk = UART_SCLK_APB,
    };
    int intr_alloc_flags = 0;

    // ESP_ERROR_CHECK(uart_enable_pattern_det_baud_intr(2, 0xF1, 1, 1, 0, 80));
    ESP_ERROR_CHECK(uart_driver_install(2 /* port */, 1024 * 2, 256, 0, NULL, intr_alloc_flags));
    ESP_ERROR_CHECK(uart_param_config(2 /* port */, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(2 /* port */, 14, 13, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
}

void PelotonSerial::TxTask(void *context) {
    PelotonSerial *peloton_serial = static_cast<PelotonSerial *>(context);

    uint8_t get_power[4] = {0xF5, 0x44, 0x39, 0xF6};

    vTaskDelay(pdMS_TO_TICKS(1000));

    while (1) {
        int wlen = uart_write_bytes(2, get_power, 4); // todo make sure this actually writes

        vTaskDelay(pdMS_TO_TICKS(100)); // todo rate monotonic analysis RMA
    }
}


void PelotonSerial::RxTask(void *context) {
    PelotonSerial *peloton_serial = static_cast<PelotonSerial *>(context);

    peloton_serial->Init();  // todo initialization? is there a better way to do this? (init
                             // everything then start the tasks?)

    // Configure a temporary buffer for the incoming data
    uint8_t *data = (uint8_t *)malloc(10);  // todo make static and smaller
    // also todo make state machine / pattern matching

    // uint8_t init[4] = {0xFE, 0x00, 0xFE, 0xF6};
    uint8_t get_power[4] = {0xF5, 0x44, 0x39, 0xF6};

    while (1) {

        // Read data from the UART
        int len = uart_read_bytes(2, data, 10, pdMS_TO_TICKS(5));
        if (len > 0) {
            uint8_t dir = data[0];
            uint8_t metric = data[1];
            uint8_t payload_len = data[2];

            // ESP_LOGI("", "dir: %x, metric: %x, len: %x\n", dir, metric, payload_len);

            if (dir == 0xF1 && metric == 0x44) {
                assert(payload_len == 5);
                assert(len == 10);

                uint16_t power_deciwatts = 0;
                uint16_t power_10 = 1;
                for (int place = 0; place < 5; place++) {
                    power_deciwatts += power_10 * (data[3 + place] - (int)'0'); // todo extract into function
                    power_10 *= 10;
                }

                // ESP_LOGI("", "power: %d deciwatts\n", power_deciwatts); // todo add tag
                uint16_t power_watts = power_deciwatts / 10;
                peloton_serial->event_handler.SubmitEvent(PowerReading{power_watts});
            }
        }
        // Write data back to the UART
        // ESP_LOGI(tag, "uart: %d\n", len);
    }
}