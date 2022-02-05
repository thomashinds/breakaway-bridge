#include "ble-trainer.h"

#include <cstdio>

#include "NimBLEDevice.h"

// Device Info
constexpr uint16_t GATT_DEVICE_INFO_UUID = 0x180A;
constexpr uint16_t GATT_MANUFACTURER_NAME_UUID = 0x2A29;
constexpr uint16_t GATT_MODEL_NUMBER_UUID = 0x2A24;

// Cycling Power Service
constexpr uint16_t GATT_CPS_UUID = 0x1818;
constexpr uint16_t GATT_CPS_MEASUREMENT_UUID = 0x2A63;
constexpr uint16_t GATT_CPS_FEATURE_UUID = 0x2A65;
constexpr uint16_t GATT_CPS_SENSOR_LOC_UUID = 0x2A5D;

BLETrainer::BLETrainer()
    : notify_enable{false},
      pServer{nullptr},
      cps_feature_characteristic{nullptr},
      cps_sensor_location_characteristic{nullptr},

      cps_measurement_characteristic{nullptr},
      cps_measurement_callbacks{CPSMeasurementCallbacks(*this)} {
    xTaskCreateStatic(this->Task, "BLETrainer", this->STACK_SIZE, this, tskIDLE_PRIORITY + 5,
                      this->task_stack, &this->task_storage);
}

void BLETrainer::Init() {
    // Create the BLE Device
    NimBLEDevice::init("ESP32");

    // Create the BLE Server
    pServer = NimBLEDevice::createServer();

    // Create the Cycling Power BLE Service
    NimBLEService* cps = pServer->createService(GATT_CPS_UUID);

    // Create a BLE Characteristic
    cps_feature_characteristic =
        cps->createCharacteristic(NimBLEUUID(GATT_CPS_FEATURE_UUID), NIMBLE_PROPERTY::READ);
    cps_feature_characteristic->setValue(static_cast<uint16_t>(0x0000));

    cps_sensor_location_characteristic =
        cps->createCharacteristic(NimBLEUUID(GATT_CPS_SENSOR_LOC_UUID), NIMBLE_PROPERTY::READ);
    cps_sensor_location_characteristic->setValue(static_cast<uint8_t>(0x04));

    cps_measurement_characteristic =
        cps->createCharacteristic(NimBLEUUID(GATT_CPS_MEASUREMENT_UUID), NIMBLE_PROPERTY::NOTIFY);

    cps_measurement_characteristic->setCallbacks(&cps_measurement_callbacks);

    cps->start();

    // Start advertising
    NimBLEAdvertising* pAdvertising = NimBLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(GATT_CPS_UUID);
    pAdvertising->setScanResponse(false);

    NimBLEDevice::startAdvertising();
    printf("Waiting a client connection to notify...\n");
}

// todo put this in a different context?
void BLETrainer::notify_power(const uint16_t power_watts) {
    static constexpr uint16_t flags = 0x0000;
    const std::array<uint16_t, 2> payload = {flags, power_watts};
    this->cps_measurement_characteristic->setValue(payload);
    this->cps_measurement_characteristic->notify();
}

void BLETrainer::Task(void* context) {
    if (context != nullptr) {
        BLETrainer* ble_trainer = reinterpret_cast<BLETrainer*>(context);

        while (true) {
            // event_handler->HandleEvents();
            vTaskDelay(pdMS_TO_TICKS(20));
        }

    }
    // Shouldn't get here
}