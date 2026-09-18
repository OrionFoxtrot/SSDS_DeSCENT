#pragma once

#include <stdint.h>
#include "../platform/I2cBus.h"
#include "../platform/Status.h"
#include "../platform/System.h"
#include "SensorData.h"

namespace ChipSatDevices
{

// MAX17048. Adafruit's begin() resets the gauge every boot
class FuelGauge
{
public:
  FuelGauge(ChipSatPlatform::I2cBus &bus, ChipSatPlatform::System &system);

  ChipSatPlatform::Status begin();
  ChipSatPlatform::Status read();    // NaN keeps the old value, otherwise clamped to 0-100

  bool ready() const { return ready_; }
  const ChipSatSensors::StateOfChargeData &data() const { return data_; }

  float rawPercent() const { return rawPercent_; }   // before the clamp, for logging

private:
  ChipSatPlatform::I2cBus &bus_;
  ChipSatPlatform::System &system_;
  ChipSatSensors::StateOfChargeData data_;
  bool ready_ = false;
  float rawPercent_ = 0.0f;
};

} // namespace ChipSatDevices
