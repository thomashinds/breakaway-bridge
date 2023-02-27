#ifndef BLE_TRAINER_H
#define BLE_TRAINER_H

#include "NimBLECharacteristic.h"
#include "NimBLEServer.h"
#include "NimBLEService.h"

class BLETrainer {
 public:
  BLETrainer();
  void Init();
  void notify_power(int16_t power_watts);
  void notify_cadence(uint16_t cadence_rpm);

 private:
  StaticTask_t task_storage;
  static constexpr size_t STACK_SIZE{1024};
  StackType_t task_stack[STACK_SIZE];

  NimBLEServer *gatt_server;

  // Device info
  NimBLEService *device_info_service;
  NimBLECharacteristic *di_manufacturer_name_characteristic;
  NimBLECharacteristic *di_model_number_characteristic;
  NimBLECharacteristic *di_serial_number_characteristic;
  NimBLECharacteristic *di_hardware_revision_characteristic;
  NimBLECharacteristic *di_firmware_revision_characteristic;

  // Cycling Power
  NimBLEService *cps_service;
  NimBLECharacteristic *cps_feature_characteristic;
  NimBLECharacteristic *cps_sensor_location_characteristic;
  NimBLECharacteristic *cps_measurement_characteristic;

  // Cycling Speed and Cadence
  // NimBLEService *csc_service;
  // NimBLECharacteristic *csc_characteristic_measurement;
  // NimBLECharacteristic *csc_characteristic_feature;

  // Adapter state data
  uint16_t stored_crank_revolutions;
  uint16_t stored_crank_event_time;

  // u32 max mm = 2668 miles
  uint32_t total_mm_travelled;
  uint32_t last_ms_stored;

  uint32_t stored_wheel_revolutions;
  uint16_t stored_wheel_event_time_2048ths;

  // CPSMeasurementCallbacks cps_measurement_callbacks;

  static void Task(void *);
};

#endif