#include "includes/BNO085_IMU.hpp"
#include "includes/constants.hpp"

BNO085_IMU::BNO085_IMU() {}

bool BNO085_IMU::begin(uint8_t address, uint8_t BNO_INT , uint8_t BNO_RST)
{
  Wire.begin();
  Wire.flush();
  Print_tx_rx.println("I2C Wiped, Ready to Boot");
//   delay(500);
  while (imu.begin(address, Wire, -1, -1) == false) {
    Print_tx_rx.println("BNO08x not detected at default I2C address. Check your jumpers and the hookup guide. Freezing...");
    delay(2500); // Try to initialize IMU
  }
  Print_tx_rx.println("--BNO08x FOUND!--");


  // imu.enableLinearAccelerometer(50);
  if (imu.enableLinearAccelerometer(50) == true) {
    Print_tx_rx.println(F("Accelerometer enabled"));
    Print_tx_rx.println(F("Output in form x, y, z, in m/s^2"));
  } else {
    Print_tx_rx.println("Could not enable accelerometer");
  }
  imu.enableGyro(50); // 10ms = 100Hz
  imu.enableMagnetometer(50);

  return true;
}


void BNO085_IMU::readData()
{


//   return data;
}

void BNO085_IMU::sleep()
{
  imu.modeSleep();
}

void BNO085_IMU::wake()
{
  imu.modeOn();
  consecutiveFailures = 0;
}
