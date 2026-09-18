#pragma once

#include <stdint.h>

namespace ChipSatPlatform
{

// RCC->CSR as read at boot
struct ResetCause
{
  uint32_t csr = 0;
  bool lowPower = false;
  bool windowWatchdog = false;
  bool independentWatchdog = false;
  bool software = false;
  bool brownOut = false;
  bool pin = false;
  bool optionByteLoad = false;
  bool radioIllegalAccess = false;
  bool radio = false;
};

class System
{
public:
  uint32_t nowMs() const;   // wraps after ~49.7 days, compare as now - start
  void waitMs(uint32_t ms) const;

  // Clears the flags so the next boot only shows its own reset cause
  ResetCause readAndClearResetCause();

  [[noreturn]] void haltForever(uint32_t pollMs);
};

} // namespace ChipSatPlatform
