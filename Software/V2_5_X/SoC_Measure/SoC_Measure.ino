#include "Adafruit_MAX1704X.h"

Adafruit_MAX17048 maxlipo;

#define Print_rxPin PB7
#define Print_txPin PB6
HardwareSerial Print_tx_rx = HardwareSerial(Print_rxPin, Print_txPin);


void setup() {
  Print_tx_rx.begin(115200);

  while (!Print_tx_rx) delay(10);    // wait until serial monitor opens

  Print_tx_rx.println(F("\nAdafruit MAX17048 simple demo"));

  while (!maxlipo.begin()) {
    Print_tx_rx.println(F("Couldnt find Adafruit MAX17048?\nMake sure a battery is plugged in!"));
    delay(2000);
  }
  Print_tx_rx.print(F("Found MAX17048"));
  Print_tx_rx.print(F(" with Chip ID: 0x")); 
  Print_tx_rx.println(maxlipo.getChipID(), HEX);
}

void loop() {
  float cellVoltage = maxlipo.cellVoltage();
  if (isnan(cellVoltage)) {
    Print_tx_rx.println("Failed to read cell voltage, check battery is connected!");
    delay(2000);
    return;
  }
  Print_tx_rx.print(F("Batt Voltage: ")); Print_tx_rx.print(cellVoltage, 3); Print_tx_rx.println(" V");
  Print_tx_rx.print(F("Batt Percent: ")); Print_tx_rx.print(maxlipo.cellPercent(), 1); Print_tx_rx.println(" %");
  Print_tx_rx.println();

  delay(2000);  // dont query too often!
}
