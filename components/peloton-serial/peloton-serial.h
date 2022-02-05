#ifndef PELOTON_SERIAL_H
#define PELOTON_SERIAL_H

#include <optional>

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

    static constexpr size_t PACKET_LENGTH{10};

    EventHandler& event_handler;

    std::optional<Event> ExtractData(std::array<uint8_t, PACKET_LENGTH> raw_packet);

    void TxTask();
    void RxTask();

    static void TxTaskWrapper(void* context);
    static void RxTaskWrapper(void* context);
};

#endif