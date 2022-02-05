#include "peloton-serial.h"

#include <numeric>

#include "driver/uart.h"
#include "esp_log.h"

PelotonSerial::PelotonSerial(EventHandler &event_handler) : event_handler{event_handler} {
    // Todo defend against these constructors being called more than once
    xTaskCreateStatic(this->TxTaskWrapper, "PelotonSerialTX", this->TX_STACK_SIZE, this,
                      tskIDLE_PRIORITY + 1, this->tx_task_stack, &this->tx_task_storage);

    xTaskCreateStatic(this->RxTaskWrapper, "PelotonSerialRX", this->RX_STACK_SIZE, this,
                      tskIDLE_PRIORITY + 4, this->rx_task_stack, &this->rx_task_storage);
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

std::optional<Event> PelotonSerial::ExtractData(
    std::array<uint8_t, PelotonSerial::PACKET_LENGTH> raw_packet) {
    constexpr uint8_t FROM_BIKE{0xF1};
    constexpr uint8_t POWER{0x44};
    constexpr uint8_t CADENCE{0x41};
    constexpr uint8_t STOP{0xF6};

    uint8_t direction = raw_packet[0];
    uint8_t metric = raw_packet[1];
    uint8_t payload_len = raw_packet[2];
    uint8_t checksum = raw_packet[3 + payload_len];
    uint8_t stop = raw_packet[3 + payload_len + 1];

    // ESP_LOG_BUFFER_HEX("RX", raw_packet.data(), raw_packet.size());
    // ESP_LOGI("", "dir: %x, metric: %x, len: %x\n", direction, metric, payload_len);

    uint8_t packet_sum =
        std::accumulate(raw_packet.cbegin(), raw_packet.cbegin() + 3 + payload_len, 0);

    if (direction == FROM_BIKE && stop == STOP && packet_sum == checksum) {
        uint16_t payload = 0;
        uint16_t power_10 = 1;

        for (int place = 0; place < payload_len; place++) {
            payload += power_10 * (raw_packet[3 + place] - (int)'0');  // todo use fancy algorithms
            power_10 *= 10;
        }

        // todo narrowing cleanup and check payload len?
        if (metric == POWER) {
            return std::make_optional(PowerReading{static_cast<uint16_t>(payload / 10)});
        } else if (metric == CADENCE) {
            return std::make_optional(CadenceReading{payload});
        } else {
            return std::nullopt;  // todo cyclomatic
        }

    } else {
        return std::nullopt;
    }
}
void PelotonSerial::TxTask() {
    uint8_t get_power[4] = {0xF5, 0x44, 0x39, 0xF6};
    uint8_t get_cadence[4] = {0xF5, 0x41, 0x36, 0xF6};

    vTaskDelay(pdMS_TO_TICKS(1000));

    while (1) {
        int wlen = uart_write_bytes(2, get_power, 4);  // todo make sure this actually writes

        vTaskDelay(pdMS_TO_TICKS(100));  // todo rate monotonic analysis RMA

        wlen = uart_write_bytes(2, get_cadence, 4);

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void PelotonSerial::RxTask() {
    this->Init();  // todo initialization? is there a better way to do this? (init
                   // everything then start the tasks?)

    while (1) {
        // Read data from the UART
        std::array<uint8_t, PACKET_LENGTH> rx_buffer{};

        int len = uart_read_bytes(2, rx_buffer.data(), 10, pdMS_TO_TICKS(5));
        if (len > 0) {
            std::optional<Event> event = this->ExtractData(rx_buffer);
            if (event) {
                this->event_handler.SubmitEvent(*event);
            }
        }
    }
}

void PelotonSerial::TxTaskWrapper(void *context) {
    static_cast<PelotonSerial *>(context)->TxTask();
}

void PelotonSerial::RxTaskWrapper(void *context) {
    static_cast<PelotonSerial *>(context)->RxTask();
}
