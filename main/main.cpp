#include <cstdbool>
#include <cstdint>

#include "ble-trainer.h"
#include "esp_log.h"
#include "event-handler.h"
#include "peloton-serial.h"

extern "C" void app_main(void) {
  // Initialize applications
  static BLETrainer ble_trainer;
  static const EventHandler event_handler(ble_trainer);
  static const PelotonSerial peloton_serial(event_handler);

  esp_log_level_set("*", ESP_LOG_WARN);

  vTaskDelay(pdMS_TO_TICKS(1000));

  while (true) {
    // Uncomment the following lines to test BLE functionality without a physical
    // connection to the Peloton

    // event_handler.SubmitEvent(PowerReading{200});
    // vTaskDelay(pdMS_TO_TICKS(60));
    // event_handler.SubmitEvent(CadenceReading{80});
    // vTaskDelay(pdMS_TO_TICKS(60));
    // event_handler.SubmitEvent(ResistanceReading{44});
    vTaskDelay(pdMS_TO_TICKS(600));
  }
}
