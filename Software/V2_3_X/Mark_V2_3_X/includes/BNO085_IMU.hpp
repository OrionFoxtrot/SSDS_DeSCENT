// IMUSensor.h
#pragma once
#include <Wire.h>
#include "SparkFun_BNO08x_Arduino_Library.h"

struct IMUData
{
  int16_t gyroX;
  int16_t gyroY;
  int16_t gyroZ;

  int16_t linX;
  int16_t linY;
  int16_t linZ;

  int16_t magX;
  int16_t magY;
  int16_t magZ;
};
class BNO085_IMU
{
public:
  BNO085_IMU();
  bool begin(uint8_t address = 0x4A, uint8_t BNO_INT = PB3, uint8_t BNO_RST = PB4);
  bool available();   // check if data available
  void readData(); // returns x,y,z as string
  void sleep();
  void wake();

private:
  BNO08x imu; // Actual IMU object
  float gyroX, gyroY, gyroZ;
  float linX, linY, linZ;

  byte linAccuracy;
  int consecutiveFailures = 0;
};