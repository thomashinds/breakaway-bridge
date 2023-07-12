#include "event-handler.h"

#include "ble-trainer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

EventHandler::EventHandler(BLETrainer &ble_trainer)
    : event_queue_handle{xQueueCreateStatic(
          10, sizeof(Event), queue_buffer.data(), &queue_storage)},
      ble_trainer{ble_trainer},
      event_dispatch{EventDispatch(ble_trainer)} {
  xTaskCreateStatic(this->Task,
                    "EventHandler",
                    this->STACK_SIZE,
                    this,
                    tskIDLE_PRIORITY + 3,
                    this->task_stack,
                    &this->task_storage);
}

void EventDispatch::operator()(PowerReading power_reading) {
  ESP_LOGI("Event", "Power: %u", power_reading.p);
  ble_trainer.notify_power(power_reading.p);
}

void EventDispatch::operator()(CadenceReading cadence_reading) {
  ESP_LOGI("Event", "Cadence: %u", cadence_reading.c);
  ble_trainer.notify_cadence(cadence_reading.c);
}

void EventDispatch::operator()(ResistanceReading resistance_reading) {
  ESP_LOGI("Event", "Resistance: %u", resistance_reading.r);
  // Not doing anything with this right now.
  // Ideas:
  // - Output to a separate display
  // - Use a different BLE characteristic to show the resistance (temperature /
  //   tire pressure / etc)
  // - Use the BLE cycling trainer characteristic to show the resistance
}

void EventHandler::HandleEvents() {
  // Storage for the event from the queue
  Event event;
  // Pop an event from the queue if one is present
  if (xQueueReceive(this->event_queue_handle, &event, 0) == pdTRUE) {
    // Use the dispatcher to call the appropriate method on the BLETrainer
    std::visit(this->event_dispatch, event);
  }
}

// Add an event to the event queue - a return value of false means the queue was full
bool EventHandler::SubmitEvent(Event event) const {
  return pdTRUE == xQueueSend(this->event_queue_handle, &event, 0);
}

void EventHandler::Task(void *context) {
  if (context != nullptr) {
    // Task setup
    auto *event_handler = reinterpret_cast<EventHandler *>(context);

    event_handler->ble_trainer.Init();

    // Task loop
    while (true) {
      event_handler->HandleEvents();
      vTaskDelay(pdMS_TO_TICKS(20));
    }
  }
}