// IMUSensor.h
#pragma once
#include <Wire.h>
#include "SparkFun_BNO080_Arduino_Library.h"
#include <SoftwareSerial.h>
#include "includes/Constants.hpp"
#include "includes/communication/DataPacket.hpp"

class IMUSensor
{
public:
  IMUSensor();
  bool begin(uint8_t address = 0x4A);
  bool available();   // check if data available
  IMUData readData(); // returns x,y,z as string
  void sleep();
  void wake();

private:
  BNO080 imu;
  float gyroX, gyroY, gyroZ;
  float linX, linY, linZ;
  // include magnometer value?
  byte linAccuracy;
  int consecutiveFailures = 0;
};