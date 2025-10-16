#include "BMESensor.hpp"
#include "Constants.hpp"
// #include <Arduino.h>

#define SEALEVELPRESSURE_HPA (1013.25)

BMESensor::BMESensor() {}

unsigned BMESensor::begin()
{
  unsigned status;
  status = bme.begin();
  if (!status)
  {
    Print_tx_rx.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
    Print_tx_rx.print("SensorID was: 0x");
    Print_tx_rx.println(bme.sensorID(), 16);
    Print_tx_rx.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
    Print_tx_rx.print("   ID of 0x56-0x58 represents a BMP 280,\n");
    Print_tx_rx.print("        ID of 0x60 represents a BME 280.\n");
    Print_tx_rx.print("        ID of 0x61 represents a BME 680.\n");
    while (1)
      delay(10);
  }

  return status;
}

String BMESensor::readData()
{
  String str = "";
  // match your `getBMEInfo()` order: temp, pressure, altitude, humidity
  str += String(int(bme.readTemperature()));
  str += "," + String(int(bme.readPressure() / 100.0F));
  str += "," + String(int(bme.readAltitude(SEALEVELPRESSURE_HPA)));
  str += "," + String(int(bme.readHumidity()));
  return str;
}
