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

  x = imu.getLinAccelX();
  y = imu.getLinAccelY();
  z = imu.getLinAccelZ();
  linAccuracy = imu.getLinAccelAccuracy();

  str = str + String(x) + ',' + String(y) + ',' + String(z);
  return (str);
}
