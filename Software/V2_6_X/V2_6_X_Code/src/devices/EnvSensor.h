#pragma once

#include <stdint.h>
#include "../platform/I2cBus.h"
#include "../platform/Status.h"
#include "../platform/System.h"
#include "SensorData.h"

namespace ChipSatDevices
{

// BME280 with Adafruit's defaults. Altitude uses a fixed 1013.25 hPa sea level
class EnvSensor
{
public:
  EnvSensor(ChipSatPlatform::I2cBus &bus, ChipSatPlatform::System &system);

  ChipSatPlatform::Status begin();
  ChipSatPlatform::Status read();    // all four values have to be finite, or the old ones stay

  bool ready() const { return ready_; }
  const ChipSatSensors::EnvironmentalData &data() const { return data_; }

private:
  ChipSatPlatform::I2cBus &bus_;
  ChipSatPlatform::System &system_;
  ChipSatSensors::EnvironmentalData data_;
  bool ready_ = false;
};

} // namespace ChipSatDevices
