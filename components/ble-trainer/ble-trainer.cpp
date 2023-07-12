#include "ble-trainer.h"

#include <sys/_stdint.h>
#include <sys/types.h>

#include <cmath>
#include <cstdio>
#include <optional>

#include "NimBLECharacteristic.h"
#include "NimBLEDevice.h"
#include "NimBLEUUID.h"
#include "esp_log.h"
#include "esp_timer.h"

// Device Info
constexpr uint16_t DEVICE_INFO_UUID = 0x180A;
constexpr uint16_t MANUFACTURER_NAME_UUID = 0x2A29;
constexpr uint16_t MODEL_NUMBER_UUID = 0x2A24;
constexpr uint16_t SERIAL_NUMBER_UUID = 0x2A25;
constexpr uint16_t FIRMWARE_REVISION_UUID = 0x2A26;
constexpr uint16_t HARDWARE_REVISION_UUID = 0x2A27;

// Cycling Power Service
constexpr uint16_t CPS_UUID = 0x1818;
constexpr uint16_t CPS_MEASUREMENT_UUID = 0x2A63;
constexpr uint16_t CPS_FEATURE_UUID = 0x2A65;
constexpr uint16_t CPS_SENSOR_LOC_UUID = 0x2A5D;

// Cycling Speed and Cadence Service
// Currently unimplemented in favor of using the cycling power service
// for speed and cadence as well.
// constexpr uint16_t CSC_SERVICE_UUID{0x1816};
// constexpr uint16_t CSC_CHARACTERISTIC_MEASUREMENT{0x2A5B};
// constexpr uint16_t CSC_CHARACTERISTIC_FEATURE{0x2A5C};

BLETrainer::BLETrainer()
    : gatt_server{nullptr},
      cps_feature_characteristic{nullptr},
      cps_sensor_location_characteristic{nullptr},
      cps_measurement_characteristic{nullptr},
      stored_crank_revolutions{0},
      stored_crank_event_time{0},
      total_mm_travelled{0},
      last_ms_stored{0} {}

// Diagnostic logging
class MyCallbacks : public NimBLECharacteristicCallbacks {
 public:
  explicit MyCallbacks(std::string_view name) : name(name){};

  void onRead(NimBLECharacteristic* /*pCharacteristic*/,
              NimBLEConnInfo& /*connInfo*/) final {
    ESP_LOGI("BLE", "Read %s", name.c_str());
  }
  void onWrite(NimBLECharacteristic* /* pCharacteristic */,
               NimBLEConnInfo& /* connInfo */) final {
    ESP_LOGI("BLE", "Write %s", name.c_str());
  }
  void onNotify(NimBLECharacteristic* /* pCharacteristic */) final {
    ESP_LOGI("BLE", "Notify %s", name.c_str());
  }
  void onStatus(NimBLECharacteristic* /* pCharacteristic */, int code) final {
    ESP_LOGI("BLE", "Status %s: [%d]", name.c_str(), code);
  }
  void onSubscribe(NimBLECharacteristic* /*pCharacteristic*/,
                   NimBLEConnInfo& /*connInfo*/,
                   uint16_t /*subValue*/) final {
    ESP_LOGI("BLE", "Subscribe %s", name.c_str());
  }

 private:
  std::string name;
};

void BLETrainer::Init() {
  // Create the BLE Device
  NimBLEDevice::init("Peloton Bridge");

  // Use maximum transmit power
  NimBLEDevice::setPower(ESP_PWR_LVL_P9, ESP_BLE_PWR_TYPE_DEFAULT);

  // Create the BLE Server
  this->gatt_server = NimBLEDevice::createServer();

  //
  // Create the Device Info BLE service
  //
  this->device_info_service =
      this->gatt_server->createService(DEVICE_INFO_UUID);

  // Create Device Info BLE Characteristics
  this->di_manufacturer_name_characteristic =
      this->device_info_service->createCharacteristic(MANUFACTURER_NAME_UUID,
                                                      NIMBLE_PROPERTY::READ);
  this->di_manufacturer_name_characteristic->setValue("thomas co");

  // Model Number
  this->di_model_number_characteristic =
      this->device_info_service->createCharacteristic(MODEL_NUMBER_UUID,
                                                      NIMBLE_PROPERTY::READ);
  this->di_model_number_characteristic->setValue("PB1");

  // Serial Number
  this->di_serial_number_characteristic =
      this->device_info_service->createCharacteristic(SERIAL_NUMBER_UUID,
                                                      NIMBLE_PROPERTY::READ);
  this->di_serial_number_characteristic->setValue("8675309");

  // Firmware Revision
  this->di_firmware_revision_characteristic =
      this->device_info_service->createCharacteristic(FIRMWARE_REVISION_UUID,
                                                      NIMBLE_PROPERTY::READ);
  this->di_firmware_revision_characteristic->setValue("FW0.1");

  // Hardware Revision
  this->di_hardware_revision_characteristic =
      this->device_info_service->createCharacteristic(HARDWARE_REVISION_UUID,
                                                      NIMBLE_PROPERTY::READ);
  this->di_hardware_revision_characteristic->setValue("HW0.1");

  //
  // Create the Cycling Power BLE Service
  //
  this->cps_service = gatt_server->createService(CPS_UUID);

  // Create Cycling Power Service BLE Characteristics
  this->cps_feature_characteristic = this->cps_service->createCharacteristic(
      NimBLEUUID(CPS_FEATURE_UUID), NIMBLE_PROPERTY::READ);
  this->cps_feature_characteristic->setValue(
      static_cast<uint16_t>(1 << 2 | 1 << 3));  // Speed and Cadence supported

  // Sensor Location
  this->cps_sensor_location_characteristic =
      this->cps_service->createCharacteristic(NimBLEUUID(CPS_SENSOR_LOC_UUID),
                                              NIMBLE_PROPERTY::READ);
  this->cps_sensor_location_characteristic->setValue(
      static_cast<uint8_t>(0x04));  // Front Wheel

  // Cycling Power Measurement
  this->cps_measurement_characteristic =
      this->cps_service->createCharacteristic(NimBLEUUID(CPS_MEASUREMENT_UUID),
                                              NIMBLE_PROPERTY::NOTIFY);

  // Uncomment for diagnostic logging of BLE events for the cycling power
  // service static MyCallbacks cps_measurement_callbacks("CPS Measurement");
  // this->cps_measurement_characteristic->setCallbacks(
  //     &cps_measurement_callbacks);

  //
  // Create Cycling Speed and Cadence BLE Service
  // Currently unused in favor of using the cycling power service for speed and
  // cadence as well.
  //
  // this->csc_service = gatt_server->createService(CSC_SERVICE_UUID);

  // this->csc_characteristic_measurement =
  //     this->csc_service->createCharacteristic(
  //         NimBLEUUID(CSC_CHARACTERISTIC_MEASUREMENT),
  //         NIMBLE_PROPERTY::NOTIFY);

  // static MyCallbacks csc_measurement_callbacks("CSC Measurement");
  // this->csc_characteristic_measurement->setCallbacks(
  //     &csc_measurement_callbacks);

  // this->csc_characteristic_feature = this->csc_service->createCharacteristic(
  //     NimBLEUUID(CSC_CHARACTERISTIC_FEATURE), NIMBLE_PROPERTY::READ);

  // // Crank revolution data supported (cadence) and Wheel revolution data
  // // supported (speed)
  // this->csc_characteristic_feature->setValue(static_cast<uint16_t>(0b011));

  // Start the services
  this->device_info_service->start();
  this->cps_service->start();
  // this->csc_service->start();

  // Start advertising
  NimBLEAdvertising* advertising = NimBLEDevice::getAdvertising();
  static_assert(1157 == 0x485);
  advertising->setAppearance(1157);
  advertising->addServiceUUID(DEVICE_INFO_UUID);
  advertising->addServiceUUID(CPS_UUID);
  // advertising->addServiceUUID(CSC_SERVICE_UUID);
  advertising->setScanResponse(true);

  NimBLEDevice::startAdvertising();
}

// Payload for the Cycling Power Measurement characteristic, given the data
// fields indicated by the cycling power feature characteristic
struct __attribute((packed)) CPSPayload {
  uint16_t flags;
  int16_t power_watts;
  uint32_t cumulative_wheel_revolutions;
  uint16_t last_wheel_event_time_x2;  // this is in 2048th seconds intead of
                                      // 1024ths seconds (used by the speed and
                                      // cadence service)
  uint16_t cumulative_crank_revolutions;
  uint16_t last_crank_event_time;
};

// Approximation of the calculation done by the Peloton head unit for speed in
// mph given power in watts.
float get_speed(const int16_t power_watts) {
  // Calculate this once
  const float sqrt_power = std::sqrt(static_cast<float>(power_watts));

  // Using the relationship as characterized by Imran Haque
  // https://ihaque.org/posts/2020/12/25/pelomon-part-ib-computing-speed/
  // S = 0.057 - 0.172 r + 0.759 r^2 - 0.079 r^3
  // where r = sqrt(P)

  if (power_watts < 26) {
    return (0.057F) - (0.172F * sqrt_power) +
           (0.759F * sqrt_power * sqrt_power) -
           (0.079F * sqrt_power * sqrt_power * sqrt_power);
  }

  // Otherwise
  return (-1.635F) + (2.325F * sqrt_power) -
         (0.064F * sqrt_power * sqrt_power) +
         (0.001F * sqrt_power * sqrt_power * sqrt_power);
}

constexpr uint32_t ms_to_2048th_sec(uint32_t time_ms) {
  return static_cast<uint32_t>(static_cast<float>(time_ms) *
                               static_cast<float>(2048.0 / 1000.0));
}

// Sanity check "Unit test"
static_assert(ms_to_2048th_sec(1000) == 2048);

// Send a cycling power measurement notification with the power in watts.
// Also calculate the speed, and cadence as defined by the gatt characteristic
// (number of rotations and last event time).
// We have to do this since we have the actual measurements, but the spec is
// defined for dumb sensors that just count when a magnet passes by.
void BLETrainer::notify_power(const int16_t power_watts) {
  // Set up constants for unit conversion
  // Assume a 700x32c tire (circumfrence 2155mm)
  const uint32_t circumference = 2155;
  const float mm_per_mile = 1609344;
  const float ms_per_hour = 60 * 60 * 1000;
  const float speed_mph = get_speed(power_watts);

  // Can't just do the trivial calculation like with cadence because it messes
  // up total distance and average speed metrics. Instantaneous speed AND total
  // distance must be correct.

  // Calculate the distance travelled and elapsed time since the last
  // measurement event
  const uint32_t current_ms = esp_timer_get_time() / 1000;
  const uint32_t time_delta = current_ms - this->last_ms_stored;
  const float speed_mm_per_ms = (speed_mph * mm_per_mile) / ms_per_hour;
  const uint32_t mm_travelled =
      static_cast<uint32_t>(speed_mm_per_ms * static_cast<float>(time_delta));

  // Store the total distance travelled and current time, for the next
  // measurement
  this->total_mm_travelled += mm_travelled;
  this->last_ms_stored = current_ms;

  // Flags for the Cycling Power Measurement characteristic
  static constexpr uint16_t flags =
      1 << 4 | 1 << 5;  // Speed and Cadence present

  // How many total wheel revolutions have there been?
  const uint32_t wheel_revolutions = total_mm_travelled / circumference;

  // And how much extra distance have we travelled since the last complete wheel
  // rotation
  const uint32_t extra_mm =
      total_mm_travelled - (wheel_revolutions * circumference);

  // And how long did that extra distance take?
  const uint32_t extra_time_ms =
      static_cast<uint32_t>(static_cast<float>(extra_mm) / speed_mm_per_ms);

  // Given what we just calculated, what is the last event time?
  const uint16_t wheel_event_time_2048ths = static_cast<uint16_t>(
      ms_to_2048th_sec(this->last_ms_stored - extra_time_ms));

  // Store the current wheel revolutions and event time
  // These were previously used by the speed and cadence service handler
  // (currently disabled)
  this->stored_wheel_revolutions = wheel_revolutions;
  this->stored_wheel_event_time_2048ths = wheel_event_time_2048ths;

  // Set up the payload for the Cycling Power Measurement characteristic
  const CPSPayload payload{flags,
                           power_watts,
                           wheel_revolutions,
                           wheel_event_time_2048ths,
                           this->stored_crank_revolutions,
                           this->stored_crank_event_time};

  // Send the notification
  this->cps_measurement_characteristic->setValue(payload);
  this->cps_measurement_characteristic->notify();
}

// Payload for the Cycling Speed and Cadence Measurement characteristic
struct __attribute__((packed)) CSCPayload {
  uint8_t flags;
  uint32_t cumulative_wheel_revolutions;
  uint16_t last_wheel_event_time;
  uint16_t cumulative_crank_revolutions;
  uint16_t last_crank_event_time;
};

// Update the cadence data, but don't send a notification
// (notifications are sent by the cycling power service)
void BLETrainer::notify_cadence(const uint16_t cadence_rpm) {
  // Units of time are 1/1024th seconds
  static constexpr uint8_t flags{
      0b11};  // crank and wheel revolution data present

  // If the cadence is zero, don't update the stored values (since the crank
  // hasn't moved)
  if (cadence_rpm != 0) {
    // Otherwise, increment the crank revolutions and calculate the time it
    // would have taken, given the cadence This means that our total crank
    // revolutions isn't going to be accurate, but it doesn't seem to affect the
    // data collected by Zwift or Wahoo.
    // If that doesn't work, it could be calculated like speed instead
    this->stored_crank_revolutions++;
    const uint16_t time_diff = (60 * 1024) / cadence_rpm;
    this->stored_crank_event_time += time_diff;
  }

  // Payload for the Cycling Speed and Cadence Measurement characteristic,
  // unused const CSCPayload payload{
  //     flags,
  //     this->stored_wheel_revolutions,
  //     static_cast<uint16_t>(this->stored_wheel_event_time_2048ths / 2),
  //     this->stored_crank_revolutions,
  //     this->stored_crank_event_time};

  // this->csc_characteristic_measurement->setValue(payload);
  // this->csc_characteristic_measurement->notify();
}