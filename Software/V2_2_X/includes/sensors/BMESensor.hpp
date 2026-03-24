// BMESensor.h
#pragma once
#include <Adafruit_BME280.h>
#include <SoftwareSerial.h>
#include "includes/communication/DataPacket.hpp"
#include "includes/Constants.hpp"

class BMESensor
{
public:
  BMESensor();
  unsigned begin();   // your sketch didn’t set an addr, default is 0x76
  BMEData readData(); // returns temp,pressure,altitude,humidity
  void sleep();
  void wake();

private:
  Adafruit_BME280 bme;
};