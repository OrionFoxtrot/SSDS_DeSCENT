#include "includes/sensors/IMUSensor.hpp"
#include "includes/Constants.hpp"

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

IMUData IMUSensor::readData()
{
  IMUData data;

  float gyroX, gyroY, gyroZ;
  float linX, linY, linZ;
  float magX, magY, magZ;
  uint8_t gyroAccuracy, accelAccuracy, magAccuracy;

  imu.getGyro(gyroX, gyroY, gyroZ, gyroAccuracy);
  imu.getLinAccel(linX, linY, linZ, accelAccuracy);
  imu.getMag(magX, magY, magZ, magAccuracy);

  // Convert to int16_t (scale as needed) - TODO: change
  data.gyroX = int16_t(gyroX * 100);
  data.gyroY = int16_t(gyroY * 100);
  data.gyroZ = int16_t(gyroZ * 100);

  data.linX = int16_t(linX * 100);
  data.linY = int16_t(linY * 100);
  data.linZ = int16_t(linZ * 100);

  data.magX = int16_t(magX * 100);
  data.magY = int16_t(magY * 100);
  data.magZ = int16_t(magZ * 100);

  return data;
}
