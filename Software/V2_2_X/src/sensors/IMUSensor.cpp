#include "includes/sensors/IMUSensor.hpp"

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
  imu.enableGyro(50); // 10ms = 100Hz
  imu.enableMagnetometer(50);

  return true;
}

// bool IMUSensor::available()
// {
//   return imu.dataAvailable();
// }

static int consecutiveFailures = 0;
IMUData IMUSensor::readData()
{
  if (consecutiveFailures >= 5)
  {
    Print_tx_rx.println(F("!!! IMU STALL/RESET - RE-INITIALIZING HARDWARE !!!"));

    // Actually re-run the begin sequence
    if (imu.begin(0x4A))
    {
      // TODO: just call begin() instead?
      imu.enableLinearAccelerometer(50);
      imu.enableGyro(50);
      imu.enableMagnetometer(50);
      consecutiveFailures = 0;
    }
    else
    {
      Print_tx_rx.println(F("IMU Re-init Failed. Bus still busy?"));
    }
  }

  IMUData data = {0};
  Print_tx_rx.print("Checking IMU... ");

  unsigned long timeout = millis() + 100; // 100ms timeout
  while (!imu.dataAvailable())
  {
    if (millis() > timeout)
      break; // don't hang forever
  }

  if (imu.dataAvailable())
  {
    consecutiveFailures = 0;
    Print_tx_rx.println("DATA FOUND!");

    float gyroX, gyroY, gyroZ;
    float linX, linY, linZ;
    float magX, magY, magZ;
    byte gyroAccuracy, accelAccuracy, magAccuracy;

    imu.getGyro(gyroX, gyroY, gyroZ, gyroAccuracy);
    imu.getLinAccel(linX, linY, linZ, accelAccuracy);
    imu.getMag(magX, magY, magZ, magAccuracy);
    if (magAccuracy == 0)
    {
      Print_tx_rx.println("Mag Status: Unreliable (Calibrate me!)");
    }

    // Debug print for raw floats
    Print_tx_rx.print("Gyro X: ");
    Print_tx_rx.print(gyroX, 3);
    Print_tx_rx.print(" | Y: ");
    Print_tx_rx.print(gyroY, 3);
    Print_tx_rx.print(" | Z: ");
    Print_tx_rx.println(gyroZ, 3);
    Print_tx_rx.print("LinAccel X: ");
    Print_tx_rx.print(linX, 3);
    Print_tx_rx.print(" | Y: ");
    Print_tx_rx.print(linY, 3);
    Print_tx_rx.print(" | Z: ");
    Print_tx_rx.println(linZ, 3);
    Print_tx_rx.print("Mag X: ");
    Print_tx_rx.print(magX, 3);
    Print_tx_rx.print(" | Y: ");
    Print_tx_rx.print(magY, 3);
    Print_tx_rx.print(" | Z: ");
    Print_tx_rx.println(magZ, 3);

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
  else
  {
    consecutiveFailures++; // Increment failure count
    Print_tx_rx.print(F("NO DATA - skipping! (Fail count: "));
    Print_tx_rx.print(consecutiveFailures);
    Print_tx_rx.println(F(")"));
  }
  Print_tx_rx.println();

  return data;
}
