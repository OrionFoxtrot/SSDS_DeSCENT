// BMESensor.h
#pragma once
#include <Adafruit_BME280.h>
#include <SoftwareSerial.h>


class BMESensor
{
public:
  BMESensor();
  unsigned begin();  // your sketch didn’t set an addr, default is 0x76
  String readData(); // returns temp,pressure,altitude,humidity

private:
  Adafruit_BME280 bme;
  
};
