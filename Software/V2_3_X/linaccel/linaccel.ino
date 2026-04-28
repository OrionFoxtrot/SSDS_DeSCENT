
#include <Wire.h>

#include "SparkFun_BNO08x_Arduino_Library.h"  // CTRL+Click here to get the library: http://librarymanager/All#SparkFun_BNO08x
BNO08x myIMU;


#define BNO08X_INT -1 // not used
#define BNO08X_RST -1 // not used
#define BNO08X_ADDR 0x4A  // Alternate address if ADR jumper is closed

#define Print_rxPin PB7
#define Print_txPin PB6
HardwareSerial Print_tx_rx = HardwareSerial(Print_rxPin, Print_txPin);

void setup() {
  Print_tx_rx.begin(115200);

  while (!Print_tx_rx) delay(10);  
  Print_tx_rx.println();
  Print_tx_rx.println("BNO08x Read Example");

  Wire.begin();
 if (myIMU.begin(BNO08X_ADDR, Wire, BNO08X_INT, BNO08X_RST) == false) {
    Print_tx_rx.println("BNO08x not detected at default I2C address. Check your jumpers and the hookup guide. Freezing...");
    while (1)
      ;
  }
  Print_tx_rx.println("BNO08x found!");

  setReports();

  Print_tx_rx.println("Reading events");
  delay(100);
}

// Here is where you define the sensor outputs you want to receive
void setReports(void) {
  Print_tx_rx.println("Setting desired reports");
  
  if (myIMU.enableLinearAccelerometer() == true) {
    Print_tx_rx.println(F("Accelerometer enabled"));
    Print_tx_rx.println(F("Output in form x, y, z, in m/s^2"));
  } else {
    Print_tx_rx.println("Could not enable accelerometer");
  }
}

void loop() {
  delay(10);

  if (myIMU.wasReset()) {
    Print_tx_rx.print("sensor was reset ");
    setReports();
  }

  // Has a new event come in on the Sensor Hub Bus?
  if (myIMU.getSensorEvent() == true) {

    float x = myIMU.getLinAccelX();
    float y = myIMU.getLinAccelY();
    float z = myIMU.getLinAccelZ();

    Print_tx_rx.print(x, 2);
    Print_tx_rx.print(F(","));
    Print_tx_rx.print(y, 2);
    Print_tx_rx.print(F(","));
    Print_tx_rx.print(z, 2);

    Print_tx_rx.println();

  }
}
