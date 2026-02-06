#include "IMUSensor.hpp"
#include "Constants.hpp"

IMUSensor::IMUSensor() {}

bool IMUSensor::begin(uint8_t address)
{
  Wire.begin();
  if (!imu.begin(0x4A))
  {
    Print_tx_rx.print("IMU init failed!");
    return false;
  }
  imu.begin(address);
  Wire.setClock(400000);
  imu.enableLinearAccelerometer(50);
  return true;
}

bool IMUSensor::available()
{
  return imu.dataAvailable();
}

String IMUSensor::readData()
{
  String str = "";

  gyroX = imu.getGyroX();
  gyroY = imu.getGyroY();
  gyroZ = imu.getGyroZ();
  linX = imu.getLinAccelX();
  linY = imu.getLinAccelY();
  linZ = imu.getLinAccelZ();
  // linAccuracy = imu.getLinAccelAccuracy();

  str = str + String(gyroX) + ',' + String(gyroY) + ',' + String(gyroZ) + ',' + String(linX) + ',' + String(linY) + ',' + String(linZ);
  return (str);
}
