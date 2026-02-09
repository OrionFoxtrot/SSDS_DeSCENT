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
  // imu.begin(address);
  Wire.setClock(400000);

  imu.enableLinearAccelerometer(50);
  imu.enableGyro(10);          // 10ms = 100Hz
  imu.enableMagnetometer(100); // TODO: check if these are right later, maybe 20?

  return true;
}

bool IMUSensor::available()
{
  return imu.dataAvailable();
}

IMUData IMUSensor::readData()
{
  IMUData data;

  if (imu.dataAvailable())
  {
    Print_tx_rx.printf("data available");
    float gyroX, gyroY, gyroZ;
    float linX, linY, linZ;
    float magX, magY, magZ;
    uint8_t gyroAccuracy, accelAccuracy, magAccuracy;

    imu.getGyro(gyroX, gyroY, gyroZ, gyroAccuracy);
    imu.getLinAccel(linX, linY, linZ, accelAccuracy);
    imu.getMag(magX, magY, magZ, magAccuracy);

    // DEBUG PRINT: See raw floats before they get converted to int16_t
    // Using Print_tx_rx or Serial
    Print_tx_rx.printf("RAW_IMU: %.2f, %.2f, %.2f\n", linX, linY, linZ);

    // Helper lambda to scale and clamp safely
    auto packFloat = [](float value, float scale) -> int16_t
    {
      float scaled = value * scale;
      // Clamp to prevent integer overflow/wrap-around
      return (int16_t)std::clamp(lroundf(scaled), -32768L, 32767L);
    };

    // Convert to int16_t (scale as needed) - TODO: change
    data.gyroX = packFloat(gyroX, 100.0f);
    data.gyroY = packFloat(gyroY, 100.0f);
    data.gyroZ = packFloat(gyroZ, 100.0f);

    data.linX = packFloat(linX, 100.0f);
    data.linY = packFloat(linY, 100.0f);
    data.linZ = packFloat(linZ, 100.0f);

    data.magX = packFloat(magX, 100.0f);
    data.magY = packFloat(magY, 100.0f);
    data.magZ = packFloat(magZ, 100.0f);
  }
  return data;
}
