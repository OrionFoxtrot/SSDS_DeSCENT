/*
  Using the BNO080 IMU
  By: Nathan Seidle
  SparkFun Electronics
  Date: December 21st, 2017
  SparkFun code, firmware, and software is released under the MIT License.
	Please see LICENSE.md for further details.

  Feel like supporting our work? Buy a board from SparkFun!
  https://www.sparkfun.com/products/14586

  This example shows how to output accelerometer values

  Hardware Connections:
  Attach the Qwiic Shield to your Arduino/Photon/ESP32 or other
  Plug the sensor onto the shield
  Serial.print it out at 115200 baud to serial monitor.
*/

#include <Wire.h>
#include <Wire.h>

//#define Print_rxPin PC1
//#define Print_txPin PC0
#define Print_rxPin PB7
#define Print_txPin PB6
#include <SoftwareSerial.h>

SoftwareSerial Print_tx_rx =  SoftwareSerial(Print_rxPin, Print_txPin);

#include "SparkFun_BNO080_Arduino_Library.h" // Click here to get the library: http://librarymanager/All#SparkFun_BNO080
BNO080 myIMU;

void setup()
{
  Print_tx_rx.begin(9600);
  Print_tx_rx.println();
  Print_tx_rx.println("BNO080 Read Example");

  Wire.begin();

  myIMU.begin(0x4A);

  Wire.setClock(400000); //Increase I2C data rate to 400kHz

  myIMU.enableLinearAccelerometer(50); //Send data update every 50ms

  Print_tx_rx.println(F("Linear Accelerometer enabled"));
  Print_tx_rx.println(F("Output in form x, y, z, in m/s^2"));
  pinMode(PA9, OUTPUT);
}

void loop()
{
  //Look for reports from the IMU
  if (myIMU.dataAvailable() == true)
  {
    digitalWrite(PA9,HIGH);
    float x = myIMU.getLinAccelX();
    float y = myIMU.getLinAccelY();
    float z = myIMU.getLinAccelZ();
    byte linAccuracy = myIMU.getLinAccelAccuracy();

    Print_tx_rx.print(x, 2);
    Print_tx_rx.print(F(","));
    Print_tx_rx.print(y, 2);
    Print_tx_rx.print(F(","));
    Print_tx_rx.print(z, 2);
    Print_tx_rx.print(F(","));
    Print_tx_rx.print(linAccuracy);

    Print_tx_rx.println();
    
  }
  digitalWrite(PA9,LOW);
}
