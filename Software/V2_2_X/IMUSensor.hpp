// LoRaRadio.h
#pragma once
#include <Wire.h>
#include "SparkFun_BNO080_Arduino_Library.h"
#include <SoftwareSerial.h>

class IMUSensor
{
public:
  IMUSensor();
  bool begin(uint8_t address = 0x4A);
  bool available();  // check if data available
  String readData(); // returns x,y,z as string

private:
  BNO080 imu;
  float gyroX, gyroY, gyroZ;
  float linX, linY, linZ;
  byte linAccuracy;
};