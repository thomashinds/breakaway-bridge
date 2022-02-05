#include "event-handler.h"

#include "ble-trainer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

void EventDispatch::operator()(PowerReading power_reading) {
    ESP_LOGI("Event", "Power: %u", power_reading.p);
    ble_trainer.notify_power(power_reading.p);
}

void EventDispatch::operator()(CadenceReading cadence_reading) {
    ESP_LOGI("Event", "Cadence: %u", cadence_reading.c);
}

void EventDispatch::operator()(ResistanceReading resistance_reading) {
    ESP_LOGI("Event", "Resistance: %u", resistance_reading.r);
}

void EventHandler::HandleEvents() {
    Event event;
    if (xQueueReceive(this->event_queue, &event, 0) == pdTRUE) {
        std::visit(this->event_dispatch, event);  // Can throw an exception?
    }
}

// false means full
bool EventHandler::SubmitEvent(Event event) {
    return pdTRUE == xQueueSend(this->event_queue, &event, 0);  // block forever?
}

void EventHandler::Task(void* context) {
    if (context != nullptr) {
        EventHandler* event_handler = reinterpret_cast<EventHandler*>(context);

        event_handler->ble_trainer
            .Init();  // todo Should be done in the same task context as everything
                      // else BLE. Possibly worth putting a check here for this.

        while (true) {
            event_handler->HandleEvents();
            vTaskDelay(pdMS_TO_TICKS(20));
        }
    }
    // Shouldn't get here
}