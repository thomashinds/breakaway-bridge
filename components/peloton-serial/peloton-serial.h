#ifndef PELOTON_SERIAL_H
#define PELOTON_SERIAL_H

#include <optional>

#include "event-handler.h"
#include "freertos/FreeRTOS.h"

class PelotonSerial {
 public:
  PelotonSerial(EventHandler const& event_handler);

 private:
  using Query = std::array<uint8_t, 4>;

  bool is_calibrated;

  StaticTask_t tx_task_storage;
  static constexpr size_t TX_STACK_SIZE{4096};  // todo shrink once ble moves
  std::array<StackType_t, TX_STACK_SIZE> tx_task_stack{};

  StaticTask_t rx_task_storage;
  static constexpr size_t RX_STACK_SIZE{4096};  // todo shrink once ble moves
  std::array<StackType_t, RX_STACK_SIZE> rx_task_stack{};

  static constexpr size_t PACKET_LENGTH{10};

  static constexpr uint8_t NUM_CAL_ENTRIES = 31;
  std::array<uint16_t, NUM_CAL_ENTRIES> resistance_cal_table{};

  EventHandler const& event_handler;

  std::optional<Event> ExtractData(
      std::array<uint8_t, PACKET_LENGTH> const& raw_packet);

  static void Init();

  float GetScaledResistance(ResistanceReading const& raw_resistance);
  void ResistanceCal();
  void TxTask();
  void RxTask();
};

#endif