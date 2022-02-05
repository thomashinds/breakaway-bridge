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

class EventDispatch {
   public:
    EventDispatch(BLETrainer& ble_trainer) : ble_trainer{ble_trainer} {}
    void operator()(PowerReading power_reading);
    void operator()(CadenceReading cadence_reading);
    void operator()(ResistanceReading resistance_reading);

   private:
    BLETrainer& ble_trainer;
};

class EventHandler {
   public:
    EventHandler(BLETrainer& ble_trainer2) // todo move to cpp
        : ble_trainer{ble_trainer2},
          event_queue{nullptr},
          event_dispatch{EventDispatch(ble_trainer2)} {
        event_queue = xQueueCreate(10, sizeof(Event)); // todo make static
        xTaskCreateStatic(this->Task, "EventHandler", this->STACK_SIZE, this, tskIDLE_PRIORITY + 3,
                          this->task_stack, &this->task_storage);
    };

    void HandleEvents();

    bool SubmitEvent(Event event);

   private:
    StaticTask_t task_storage;
    static constexpr size_t STACK_SIZE{8192}; // todo shrink once ble moves
    StackType_t task_stack[STACK_SIZE];

    BLETrainer& ble_trainer;
    QueueHandle_t event_queue;
    EventDispatch event_dispatch;

    static void Task(void* context);
};

#endif