#include <Arduino.h>
#include "../config.h"
#include "Led.h"

namespace ChipSatDevices
{

void Led::begin()
{
  pinMode(ChipSatConfig::kLedPin, OUTPUT);
  digitalWrite(ChipSatConfig::kLedPin, ChipSatConfig::kLedLevelAtBoot);
}

void Led::transmitStarted()
{
  digitalWrite(ChipSatConfig::kLedPin, ChipSatConfig::kLedLevelDuringTx);
}

void Led::transmitEnded()
{
  digitalWrite(ChipSatConfig::kLedPin, ChipSatConfig::kLedLevelAfterTx);
}

} // namespace ChipSatDevices
