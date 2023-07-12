#ifndef EVENT_HANDLER_H
#define EVENT_HANDLER_H

#include <cstdint>
#include <variant>

#include "ble-trainer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

struct PowerReading {
  uint16_t p;
};

struct CadenceReading {
  uint16_t c;
};

struct ResistanceReading {
  uint16_t r;
};

using Event = std::variant<PowerReading, CadenceReading, ResistanceReading>;

// Visitor for the Event std::variant type - calls the appropriate BLETrainer
// method for the corresponding event.
class EventDispatch {
 public:
  EventDispatch(BLETrainer &ble_trainer) : ble_trainer{ble_trainer} {}
  void operator()(PowerReading power_reading);
  void operator()(CadenceReading cadence_reading);
  void operator()(ResistanceReading resistance_reading);

 private:
  BLETrainer &ble_trainer;
};

class EventHandler {
 public:
  EventHandler(BLETrainer &ble_trainer);

  void HandleEvents();

  bool SubmitEvent(Event event) const;

 private:
  // FreeRTOS trask
  StaticTask_t task_storage;
  static constexpr size_t STACK_SIZE{8192};
  StackType_t task_stack[STACK_SIZE];

  // FreeRTOS queue
  std::array<uint8_t, sizeof(Event) * 10> queue_buffer;
  QueueHandle_t event_queue_handle;
  StaticQueue_t queue_storage;

  BLETrainer &ble_trainer;
  EventDispatch event_dispatch;

  static void Task(void *context);
};

#endif