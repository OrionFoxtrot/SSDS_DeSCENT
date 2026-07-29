// SPDX-FileCopyrightText: 2023 Carter Nelson for Adafruit Industries
//
// SPDX-License-Identifier: MIT
// --------------------------------------
// i2c_scanner
//
// Modified from https://playground.arduino.cc/Main/I2cScanner/
// --------------------------------------

#include <Wire.h>
#define rxPin PB7
#define txPin PB6
#include <SoftwareSerial.h>
#define blinkypin PA9

HardwareSerial soft_tx_rx = HardwareSerial(rxPin, txPin);

// Set I2C bus to use: Wire, Wire1, etc.
#define WIRE Wire

void setup() {
  // Wio-E5 module I2C2 pins
  Wire.setSDA(PA15);
  Wire.setSCL(PB15);

  WIRE.begin();
  pinMode(blinkypin, OUTPUT);
  digitalWrite(blinkypin, HIGH);

  soft_tx_rx.begin(115200);
  while (!soft_tx_rx)
    delay(10);
  soft_tx_rx.println("\nI2C Scanner");
}


void loop() {
  byte error, address;
  int nDevices;

  soft_tx_rx.println("Scanning...");

  nDevices = 0;
  for (address = 1; address < 127; address++) {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    WIRE.beginTransmission(address);
    error = WIRE.endTransmission();

    if (error == 0) {
      soft_tx_rx.print("I2C device found at address 0x");
      if (address < 16)
        soft_tx_rx.print("0");
      soft_tx_rx.print(address, HEX);
      soft_tx_rx.println("  !");

      nDevices++;
    } else if (error == 4) {
      soft_tx_rx.print("Unknown error at address 0x");
      if (address < 16)
        soft_tx_rx.print("0");
      soft_tx_rx.println(address, HEX);
    }
  }
  if (nDevices == 0)
    soft_tx_rx.println("No I2C devices found\n");
  else
    soft_tx_rx.println("done\n");
  digitalWrite(blinkypin, HIGH);
  delay(2000);  // wait 5 seconds for next scan
  digitalWrite(blinkypin, LOW);
}
