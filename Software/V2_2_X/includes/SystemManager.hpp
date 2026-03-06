#pragma once
#include "includes/communication/LoRaRadio.hpp"
#include "includes/sensors/IMUSensor.hpp"
#include "includes/sensors/BMESensor.hpp"
#include "includes/sensors/GPSModule.hpp"
#include "includes/communication/DataPacket.hpp"
#include "Constants.hpp"
#include <SoftwareSerial.h>

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
  // from constructor
  // const uint32_t *_rfswitch_pins;
  // const Module::RfSwitchMode_t *_rfswitch_table;
  // uint8_t _gpsRxPin;
  // uint8_t _gpsTxPin;

  // everything else
  LoRaRadio radio;
  IMUSensor imu;
  BMESensor bme;
  GPSModule gps;

  // Timing / state for IMU/BME duty cycle
  uint32_t lastIMUToggle = 0;
  bool imuOn = true;

  uint32_t lastIMURead = 0;
  uint32_t lastBMERead = 0;

  const uint32_t imuOnTime = 10000;  // 10s ON
  const uint32_t imuOffTime = 10000; // 10s OFF
  const uint32_t imuSampleInterval = 1000;
  const uint32_t bmeSampleInterval = 1000;

  // Transmission
  uint32_t lastTx = 0;

  // Payload storage
  DataPacket payload_;
  uint8_t txBuffer[ChipSatPacket::PACKET_SIZE]; // persistent, not stack

  // DataPacket collectData(); // building the packet
  void transmitData();
  // void printPacket(DataPacket &data);
};