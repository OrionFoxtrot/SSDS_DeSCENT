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

  void begin();  // initialize all sensors and radio
  void inloop(); // todo in a loop iteration

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

  DataPacket collectData(); // building the packet
  void transmitData(const DataPacket &payload);
  void printPacket(DataPacket &data);
};