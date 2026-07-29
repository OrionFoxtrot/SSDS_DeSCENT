// GPSModule.h
#pragma once
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#include "includes/communication/DataPacket.hpp"
#include "includes/Constants.hpp"

class GPSModule
{
public:
  GPSModule(uint8_t rxPin, uint8_t txPin);
  bool begin(long baud);
  GPSData readData();

  bool setAirborneMode();     // configure dynamic model to Airborne <2g
  bool confirmAirborneMode(); // poll and verify the setting took effect

private:
  TinyGPSPlus gps;
  HardwareSerial gpsSerial;

  bool readUBX(uint8_t *buf, uint8_t len, uint16_t timeout_ms);
  void calcChecksum(uint8_t *buf, uint8_t len, uint8_t &ckA, uint8_t &ckB);
};