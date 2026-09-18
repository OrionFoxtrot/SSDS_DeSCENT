#include <Arduino.h>
#include "../../config.h"

#if CHIPSAT_SYSTEM_DRIVER == CHIPSAT_DRIVER_LIBRARY

#include "../System.h"

namespace ChipSatPlatform
{

uint32_t System::nowMs() const
{
  return millis();
}

void System::waitMs(uint32_t ms) const
{
  delay(ms);
}

ResetCause System::readAndClearResetCause()
{
  ResetCause cause;
  cause.csr = RCC->CSR;
  cause.lowPower = (cause.csr & RCC_CSR_LPWRRSTF) != 0;
  cause.windowWatchdog = (cause.csr & RCC_CSR_WWDGRSTF) != 0;
  cause.independentWatchdog = (cause.csr & RCC_CSR_IWDGRSTF) != 0;
  cause.software = (cause.csr & RCC_CSR_SFTRSTF) != 0;
  cause.brownOut = (cause.csr & RCC_CSR_BORRSTF) != 0;
  cause.pin = (cause.csr & RCC_CSR_PINRSTF) != 0;
  cause.optionByteLoad = (cause.csr & RCC_CSR_OBLRSTF) != 0;
  cause.radioIllegalAccess = (cause.csr & RCC_CSR_RFILARSTF) != 0;
  cause.radio = (cause.csr & RCC_CSR_RFRSTF) != 0;

  __HAL_RCC_CLEAR_RESET_FLAGS();
  return cause;
}

void System::haltForever(uint32_t pollMs)
{
  while (true) {
    delay(pollMs);
  }
}

} // namespace ChipSatPlatform

#endif
