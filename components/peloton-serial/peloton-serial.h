#ifndef PELOTON_SERIAL_H
#define PELOTON_SERIAL_H

#include "event-handler.h"
#include "freertos/FreeRTOS.h"

class PelotonSerial {
   public:
    PelotonSerial(EventHandler& event_handler);
    void Init();

   private:
    StaticTask_t tx_task_storage;
    static constexpr size_t TX_STACK_SIZE{4096};  // todo shrink once ble moves
    StackType_t tx_task_stack[TX_STACK_SIZE];

    StaticTask_t rx_task_storage;
    static constexpr size_t RX_STACK_SIZE{4096};  // todo shrink once ble moves
    StackType_t rx_task_stack[RX_STACK_SIZE];

    EventHandler& event_handler;

    static void RxTask(void* context);
    static void TxTask(void* context);
};

#endif