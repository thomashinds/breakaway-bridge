#ifndef PELOTON_SERIAL_H
#define PELOTON_SERIAL_H

#include "event-handler.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

class PelotonSerial {
   public:
    PelotonSerial(EventHandler& event_handler);
    void Init();

   private:
    StaticTask_t task_storage;
    static constexpr size_t STACK_SIZE{4096};  // todo shrink once ble moves
    StackType_t task_stack[STACK_SIZE];

    EventHandler& event_handler;

    static void Task(void* context);
};

#endif