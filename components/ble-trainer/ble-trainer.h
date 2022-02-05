#ifndef BLE_TRAINER_H
#define BLE_TRAINER_H

#include "NimBLEDevice.h"

// Forward declaration
class BLETrainer;

class CPSMeasurementCallbacks : public NimBLECharacteristicCallbacks {
   public:
    CPSMeasurementCallbacks(BLETrainer& ble_trainer) : ble_trainer{ble_trainer} {}

    void onSubscribe(NimBLECharacteristic* pCharacteristic, ble_gap_conn_desc* desc,
                     uint16_t subValue) override {
        if (subValue == 1 || subValue == 3) {
            // Subscribed todo make this actually change something
        } else if (subValue == 0) {
            // Unsubscribed
        }
    }

   private:
   BLETrainer& ble_trainer;
};

class BLETrainer {
   public:
    BLETrainer();
    void Init();
    void notify_power(uint16_t power_watts);

   private:
    StaticTask_t task_storage;
    static constexpr size_t STACK_SIZE{1024};
    StackType_t task_stack[STACK_SIZE];

    bool notify_enable;

    NimBLEServer* pServer;

    NimBLECharacteristic* cps_feature_characteristic;
    NimBLECharacteristic* cps_sensor_location_characteristic;
    NimBLECharacteristic* cps_measurement_characteristic;

    CPSMeasurementCallbacks cps_measurement_callbacks;

    static void Task(void*);

};

#endif