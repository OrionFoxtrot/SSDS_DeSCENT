// LoRaRadio.h
#pragma once
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>

class GPSModule
{
public:
  GPSModule(uint8_t rxPin, uint8_t txPin);
  bool begin(long baud);
  String readData();

private:
  TinyGPSPlus gps;
  SoftwareSerial gpsSerial;
  String formatInfo();
};