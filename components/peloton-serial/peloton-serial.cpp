#include "peloton-serial.h"

#include <bits/ranges_algo.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <numeric>
#include <optional>
#include <span>

#include "driver/uart.h"
#include "esp_log.h"
#include "event-handler.h"

PelotonSerial::PelotonSerial(EventHandler const &event_handler)
    : is_calibrated(false), event_handler{event_handler} {
  // Set up the UART peripheral
  PelotonSerial::Init();

  // Todo defend against these constructors being called more than once
  xTaskCreateStatic(
      [](void *context) { static_cast<PelotonSerial *>(context)->TxTask(); },
      "PelotonSerialTX",
      PelotonSerial::TX_STACK_SIZE,
      this,
      tskIDLE_PRIORITY + 1,
      this->tx_task_stack.data(),
      &this->tx_task_storage);

  xTaskCreateStatic(
      [](void *context) { static_cast<PelotonSerial *>(context)->RxTask(); },
      "PelotonSerialRX",
      PelotonSerial::RX_STACK_SIZE,
      this,
      tskIDLE_PRIORITY + 4,
      this->rx_task_stack.data(),
      &this->rx_task_storage);

  this->resistance_cal_table = {
      174, 185, 206, 232, 276, 340, 404,  469,  520,  576, 629,
      667, 708, 743, 780, 804, 830, 854,  874,  894,  911, 927,
      941, 956, 964, 976, 982, 994, 1002, 1006, 1014,
  };
}

void PelotonSerial::Init() {
  /* Configure parameters of an UART driver,
   * communication pins and install the driver */
  const uart_config_t uart_config = {
      .baud_rate = 19200,
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
      .rx_flow_ctrl_thresh = 0,
      .source_clk = UART_SCLK_APB,
  };
  const int intr_alloc_flags = 0;

  // ESP_ERROR_CHECK(uart_enable_pattern_det_baud_intr(2, 0xF1, 1, 1, 0, 80));
  assert(ESP_OK ==
         uart_driver_install(2, 1024 * 2, 256, 0, nullptr, intr_alloc_flags));
  assert(ESP_OK == uart_param_config(2, &uart_config));
  assert(ESP_OK ==
         uart_set_pin(2, 14, 13, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
}

uint16_t parse_number(std::span<const uint8_t> span) {
  uint16_t number = 0;
  auto pow10 = 1;

  // todo use fancy algorithms
  for (auto dig : span) {
    number += pow10 * (dig - (int)'0');
    pow10 *= 10;
  }

  return number;
}

std::optional<Event> PelotonSerial::ExtractData(
    std::array<uint8_t, PelotonSerial::PACKET_LENGTH> const &raw_packet) {
  constexpr uint8_t FROM_BIKE{0xF1};
  constexpr uint8_t POWER{0x44};
  constexpr uint8_t CADENCE{0x41};
  constexpr uint8_t RAW_RESISTANCE{0x4A};
  constexpr uint8_t SCALED_RESISTANCE{0x49};  // todo wat?
  constexpr uint8_t STOP{0xF6};

  const uint8_t direction = raw_packet[0];
  const uint8_t metric = raw_packet[1];
  const uint8_t payload_len = raw_packet[2];
  const uint8_t checksum = raw_packet[3 + payload_len];
  const uint8_t stop = raw_packet[3 + payload_len + 1];

  // ESP_LOG_BUFFER_HEX_LEVEL(
  //     "Serial", raw_packet.data(), raw_packet.size(), ESP_LOG_INFO);

  const uint8_t packet_sum = std::accumulate(
      raw_packet.cbegin(), raw_packet.cbegin() + 3 + payload_len, 0);

  ESP_LOGD("Serial", "Packet sum %u\t\tChecksum%u\n", packet_sum, checksum);

  if (direction == FROM_BIKE && stop == STOP && packet_sum == checksum) {
    auto payload =
        parse_number(std::span(&raw_packet[3], &raw_packet[3 + payload_len]));

    // todo narrowing cleanup and check payload len?
    switch (metric) {
      case POWER: {
        return std::make_optional(
            PowerReading{static_cast<uint16_t>(payload / 10)});
      }

      case CADENCE: {
        return std::make_optional(CadenceReading{payload});
      }
      case SCALED_RESISTANCE:
      case RAW_RESISTANCE: {
        ESP_LOGI("Serial", "Raw resistance: %d", payload);
        // const float scaled_resistance =
        //     this->GetScaledResistance(ResistanceReading{payload});
        return std::make_optional(ResistanceReading{payload});
      }
      default: {
        return std::nullopt;
      }
    }
  }
  // The packet was not formed as we expect
  return std::nullopt;
}

float PelotonSerial::GetScaledResistance(
    ResistanceReading const &raw_resistance) {
  // Get the bounding values from the calibration table
  uint16_t *upper_bound = std::ranges::find_if(
      this->resistance_cal_table,
      [&raw_resistance](auto const &item) { return item >= raw_resistance.r; });
  uint16_t *lower_bound = upper_bound - 1;

  const size_t lower_offset =
      std::distance(this->resistance_cal_table.data(), lower_bound);
  const size_t upper_offset =
      std::distance(this->resistance_cal_table.data(), upper_bound);

  static constexpr float cal_step = 100.0 / 30.0;

  const auto lower_resistance = static_cast<float>(lower_offset) * cal_step;
  const auto upper_resistance = static_cast<float>(upper_offset) * cal_step;

  const auto fraction = static_cast<float>(raw_resistance.r - *lower_bound) /
                        static_cast<float>(*upper_bound - *lower_bound);

  return std::lerp(lower_resistance, upper_resistance, fraction);
}

void PelotonSerial::ResistanceCal() {
  for (uint8_t i = 0; i < NUM_CAL_ENTRIES; i++) {
    static constexpr uint8_t GET_CAL = 0xF7;

    const auto checksum = static_cast<uint8_t>((GET_CAL + i) % 256);
    const Query resistance_cal_query = {GET_CAL, i, checksum, 0xF6};

    // Send bytes over the serial port
    const int bytes_written = uart_write_bytes(
        2, resistance_cal_query.data(), resistance_cal_query.size());

    // Check that the bytes were written successfully
    if (bytes_written != resistance_cal_query.size()) {
      ESP_LOGE("Serial",
               "TX Error - Tried to write %u bytes but wrote %d instead",
               resistance_cal_query.size(),
               bytes_written);
    }
    std::array<uint8_t, 9> read_cal_data{};
    const int len =
        uart_read_bytes(2, read_cal_data.data(), 9, pdMS_TO_TICKS(500));
    assert(len == 9);

    // ESP_LOGD("Serial", "Read %d bytes\n", len);

    auto number = parse_number(std::span(&read_cal_data[3], 4));
    this->resistance_cal_table[i] = number;

    vTaskDelay(pdMS_TO_TICKS(10));
  }

  for (auto entry : this->resistance_cal_table) {
    printf("%u\n", entry);
  }

  this->is_calibrated = true;
}

void PelotonSerial::TxTask() {
  // Data type for query commands to the Peloton Bike
  using Query = std::array<uint8_t, 4>;

  const Query get_power{0xF5, 0x44, 0x39, 0xF6};
  const Query get_cadence = {0xF5, 0x41, 0x36, 0xF6};
  const Query get_resistance = {0xF5, 0x49, 0x3e, 0xF6};

  // What we plan to ask the bike for, in order
  const std::array<Query, 3> queries = {get_power, get_resistance, get_cadence};

  vTaskDelay(pdMS_TO_TICKS(1000));

  // This should unblock the Rx task once it's done
  // this->ResistanceCal();
  this->is_calibrated = true;
  // Loop forever
  while (true) {
    for (auto const &query : queries) {
      // Send bytes over the serial port
      const int bytes_written = uart_write_bytes(2, query.data(), query.size());
      // ESP_LOGD("Serial", "sending %x", query[1]);
      // Check that the bytes were written successfully
      if (bytes_written != query.size()) {
        ESP_LOGE("Serial",
                 "TX Error - Tried to write %u bytes but wrote %d instead",
                 query.size(),
                 bytes_written);
      }
      // Wait before running again
      vTaskDelay(pdMS_TO_TICKS(100));
    }
  }
}

void PelotonSerial::RxTask() {
  // this->Init();  // todo initialization? is there a better way to do this?
  // (init
  //                // everything then start the tasks?)

  while (true) {
    if (this->is_calibrated) {
      // Read data from the UART
      std::array<uint8_t, PACKET_LENGTH> rx_buffer{};

      std::memset(rx_buffer.data(), 0, rx_buffer.size());

      const int len =
          uart_read_bytes(2, rx_buffer.data(), 10, pdMS_TO_TICKS(5));
      // ESP_LOGI("Serial", "Read %d bytes\n", len);
      if (len > 0) {
        std::optional<Event> event = this->ExtractData(rx_buffer);
        if (event) {
          this->event_handler.SubmitEvent(*event);
        }
        else {
          ESP_LOG_BUFFER_HEX("Serial", rx_buffer.data(), rx_buffer.size());
        }
      }
    }
    vTaskDelay(1);
  }
}