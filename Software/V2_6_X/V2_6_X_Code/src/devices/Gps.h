#pragma once

#include <stdint.h>
#include "../platform/Status.h"
#include "../platform/System.h"
#include "../platform/Uart.h"
#include "SensorData.h"

namespace ChipSatDevices
{

// MAX-M10S on LPUART1. Configured once at boot, never put to sleep
class Gps
{
public:
  Gps(ChipSatPlatform::Uart &port, ChipSatPlatform::System &system);

  // The port has to be open already
  ChipSatPlatform::Status begin();

  // Ok only with a 3D fix. Coordinates update even without a fix
  ChipSatPlatform::Status read(uint16_t maxWaitMs);

  bool ready() const { return ready_; }
  const ChipSatSensors::GPSData &data() const { return data_; }

  // For logging, from the last read. Calling the library getters again would poll the receiver again
  uint8_t fixType() const { return fixType_; }
  bool llhChecked() const { return llhChecked_; }
  bool invalidLlh() const { return invalidLlh_; }

private:
  bool configureAirborne();
  bool configureNormalContinuous();

  ChipSatPlatform::Uart &port_;
  ChipSatPlatform::System &system_;
  ChipSatSensors::GPSData data_;
  bool ready_ = false;

  bool rateOkay_ = false;
  bool navOkay_ = false;
  bool modeOkay_ = false;
  uint8_t fixType_ = 0;
  bool llhChecked_ = false;
  bool invalidLlh_ = false;
};

} // namespace ChipSatDevices
