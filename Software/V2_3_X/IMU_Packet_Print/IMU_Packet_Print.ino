/*
I AM BAD
*/

#include <Wire.h>

//#define Print_rxPin PC1
//#define Print_txPin PC0
#define Print_rxPin PB7
#define Print_txPin PB6
#include <SoftwareSerial.h>

HardwareSerial Print_tx_rx =  HardwareSerial(Print_rxPin, Print_txPin);

#include "SparkFun_BNO080_Arduino_Library.h" // Click here to get the library: http://librarymanager/All#SparkFun_BNO080
BNO080 myIMU;

void setup()
{
  Print_tx_rx.begin(115200);

  Print_tx_rx.println();
  Print_tx_rx.println("BNO080 Read Example");


  Wire.setSDA(PA15);
  Wire.setSCL(PB15);
  Wire.begin();
  delay(1000);

  myIMU.begin(0x4A);

  Wire.setClock(400000); //Increase I2C data rate to 400kHz

  myIMU.enableDebugging(Print_tx_rx); //Output debug messages to the Serial port. Serial1, SerialUSB, etc is also allowed.

  myIMU.enableMagnetometer(50);
  myIMU.enableAccelerometer(50);
  // myIMU.enableLinearAccelerometer(50);
}

void loop()
{
  //Look for reports from the IMU
  if (myIMU.receivePacket() == true)
  {
    myIMU.printPacket();
    // float x = myIMU.getLinAccelX();
    // float y = myIMU.getLinAccelY();
    // float z = myIMU.getLinAccelZ();
    // byte linAccuracy = myIMU.getLinAccelAccuracy();

    // Print_tx_rx.print(x, 2);
    // Print_tx_rx.print(F(","));
    // Print_tx_rx.print(y, 2);
    // Print_tx_rx.print(F(","));
    // Print_tx_rx.print(z, 2);
    // Print_tx_rx.print(F(","));
    // Print_tx_rx.print(linAccuracy);

    // Print_tx_rx.println();
  }
}
