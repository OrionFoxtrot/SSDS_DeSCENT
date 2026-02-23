#include "includes/sensors/BMESensor.hpp"
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

BMEData BMESensor::readData()
{
  BMEData data;

  data.temp = int16_t(bme.readTemperature());
  data.pressure = int16_t(bme.readPressure() / 100.0F);
  data.humidity = int16_t(bme.readHumidity());

  return data;
}
