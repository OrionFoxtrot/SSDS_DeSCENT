#pragma once
#include "includes/communication/LoRaRadio.hpp"
#include "includes/sensors/IMUSensor.hpp"
#include "includes/sensors/BMESensor.hpp"
#include "includes/sensors/GPSModule.hpp"
#include "includes/communication/DataPacket.hpp"
#include "Constants.hpp"
#include <SoftwareSerial.h>

// enum class SensorState
// {
//   FALLING,
//   GROUND_OPS
// };

class SystemManager
{
public:
  SystemManager(
      SoftwareSerial &debugSerial,
      std::array<uint32_t, 5> rfswitch_pins,
      std::array<Module::RfSwitchMode_t, 5> rfswitch_table,
      uint8_t gpsRxPin,
      uint8_t gpsTxPin);

  void begin(); // initialize all sensors and radio
  void loop();  // todo in a loop iteration

private:
  SoftwareSerial &debug;

  // SensorState state_ = SensorState::FALLING;
  // const uint32_t DUTY_CYCLE_PERIOD_MS = 20000; // 20s total → 10s on, 10s off
  static constexpr uint32_t IMU_WARMUP_MS = 2000;

  bool imuBmeAwake_ = true;
  uint32_t lastToggle_ = 0;
  uint32_t lastWakeTime_ = 0;
  // uint32_t onGroundSince_ = 0;
  // float lastAlt_ = 0.0f;
  // bool altInitialized_ = false;

  // everything else
  LoRaRadio radio;
  IMUSensor imu;
  BMESensor bme;
  GPSModule gps;

  // Transmission
  uint32_t lastTx = 0;

  // Payload storage
  DataPacket payload_;
  uint8_t txBuffer[ChipSatPacket::PACKET_SIZE]; // persistent, not stack

  void transmitData();

  void applyDutyCycle();
  // void updateState();
  void setImuBmeSleep(bool sleep);
  // DataPacket collectData(); // building the packet
  // void printPacket(DataPacket &data);
};