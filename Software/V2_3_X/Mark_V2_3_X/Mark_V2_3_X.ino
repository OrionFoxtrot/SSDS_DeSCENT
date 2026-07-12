#include "includes/BNO085_IMU.hpp"
#include "includes/constants.hpp"
#define BNO08X_INT PB3 // not used
#define BNO08X_RST PB4 // not used
#define BNO08X_ADDR 0x4A  // Alternate address if ADR jumper is closed
#define Print_rxPin PB7
#define Print_txPin PB6

// HardwareSerial Print_tx_rx = HardwareSerial(Print_rxPin, Print_txPin);
BNO085_IMU myIMU;
void setup() {

  Print_tx_rx.begin(115200);
  delay(500);

  Print_tx_rx.println();
  Print_tx_rx.println("--Boot--");
  Print_tx_rx.flush();

  myIMU.begin();

  Print_tx_rx.println("Hello World");
}

void loop() {
  // put your main code here, to run repeatedly:
  // Print_tx_rx.println("Hello World");
  // delay(5000);
}
