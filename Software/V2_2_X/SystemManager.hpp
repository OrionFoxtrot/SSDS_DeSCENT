#pragma once
#include "LoRaRadio.hpp"
#include "IMUSensor.hpp"
#include "BMESensor.hpp"
#include "GPSModule.hpp"
#include <SoftwareSerial.h>


class SystemManager{
public:
  SystemManager(
      SoftwareSerial &debugSerial,
      const uint32_t *rfswitch_pins,
      const Module::RfSwitchMode_t *rfswitch_table,
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

  String collectData(); // building the packet
  void transmitData(const String &payload);
};