// LoRaRadio.h
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

private:
  TinyGPSPlus gps;
  SoftwareSerial gpsSerial;
};