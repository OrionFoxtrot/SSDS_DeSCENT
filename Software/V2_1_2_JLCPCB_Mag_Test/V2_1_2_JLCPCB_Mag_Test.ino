/*
  Using the BNO080 IMU
  By: Nathan Seidle
  SparkFun Electronics
  Date: December 21st, 2017
  SparkFun code, firmware, and software is released under the MIT License.
	Please see LICENSE.md for further details.

  Feel like supporting our work? Buy a board from SparkFun!
  https://www.sparkfun.com/products/14586

  This example shows how to output the parts of the magnetometer.

  It takes about 1ms at 400kHz I2C to read a record from the sensor, but we are polling the sensor continually
  between updates from the sensor. Use the interrupt pin on the BNO080 breakout to avoid polling.

  Hardware Connections:
  Attach the Qwiic Shield to your Arduino/Photon/ESP32 or other
  Plug the sensor onto the shield
  Serial.print it out at 115200 baud to serial monitor.
*/

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

  myIMU.enableMagnetometer(50); //Send data update every 50ms

  Print_tx_rx.println(F("Magnetometer enabled"));
  Print_tx_rx.println(F("Output in form x, y, z, in uTesla"));
}
void loop()
{
  //Look for reports from the IMU
  if (myIMU.dataAvailable() == true)
  {
    float x = myIMU.getMagX();
    float y = myIMU.getMagY();
    float z = myIMU.getMagZ();
    byte accuracy = myIMU.getMagAccuracy();
    float bearing = 90-atan2(y/x)*180/PI;

    Print_tx_rx.print(x, 2);
    Print_tx_rx.print(F(","));
    Print_tx_rx.print(y, 2);
    Print_tx_rx.print(F(","));
    Print_tx_rx.print(z, 2);
    Print_tx_rx.print(F(","));
    Print_tx_rx.print(bearing,2);


    Print_tx_rx.println();
  }
}

//Given a accuracy number, print what it means
void printAccuracyLevel(byte accuracyNumber)
{
  if(accuracyNumber == 0) Print_tx_rx.print(F("Unreliable"));
  else if(accuracyNumber == 1) Print_tx_rx.print(F("Low"));
  else if(accuracyNumber == 2) Print_tx_rx.print(F("Medium"));
  else if(accuracyNumber == 3) Print_tx_rx.print(F("High"));
}