#pragma once

#include <stddef.h>
#include <stdint.h>
#include "../platform/Status.h"

namespace ChipSatDevices
{

// The STM32WL's built in LoRa radio. Setup is split into steps so the caller can log and halt between them
class Radio
{
public:
  ChipSatPlatform::Status begin();
  ChipSatPlatform::Status setCurrentLimit();
  float currentLimitMa();   // reads it back over SPI
  ChipSatPlatform::Status setOutputPower();
  ChipSatPlatform::Status applyPaConfig();   // also resets the current limit, to 60 mA LP / 140 mA HP (DS_SX1261-2 Rev 2.2 Table 5-2)

  ChipSatPlatform::Status transmit(const uint8_t *data, size_t length);   // blocking

  // Raw RadioLib codes, for logging
  int16_t lastCode() const { return lastCode_; }
  int16_t ldroCode() const { return ldroCode_; }

private:
  int16_t lastCode_ = 0;
  int16_t ldroCode_ = 0;
};

} // namespace ChipSatDevices
