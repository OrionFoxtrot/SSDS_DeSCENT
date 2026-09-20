#include <Arduino.h>
#include "../../config.h"
#include "../../constants.h"

#if CHIPSAT_GAUGE_DRIVER == CHIPSAT_DRIVER_OWN

#include "../FuelGauge.h"
#include "../../log/Log.h"

namespace ChipSatDevices
{

using ChipSatPlatform::Status;
using namespace ChipSatConfig;
using namespace ChipSatConstants;

FuelGauge::FuelGauge(ChipSatPlatform::I2cBus &bus, ChipSatPlatform::System &system)
  : bus_(bus), system_(system)
{
}

Status FuelGauge::read16(uint8_t reg, uint16_t &value)
{
  uint8_t rx[2] = {0, 0};
  const Status status = bus_.writeThenRead(kGaugeAddress, &reg, 1, rx, sizeof rx);
  if (status != Status::Ok) {
    return status;
  }
  value = static_cast<uint16_t>(rx[0]) << 8 | rx[1];
  return Status::Ok;
}

Status FuelGauge::write16(uint8_t reg, uint16_t value)
{
  const uint8_t tx[3] = {reg, static_cast<uint8_t>(value >> 8), static_cast<uint8_t>(value)};
  return bus_.write(kGaugeAddress, tx, sizeof tx);
}

Status FuelGauge::begin()
{
  LOG_I(Soc, "start");
  const uint32_t startMs = system_.nowMs();

  for (uint8_t attempt = 0; attempt < kInitAttempts; ++attempt) {
    uint16_t version = 0;
    if (bus_.probe(kGaugeAddress) == Status::Ok && read16(kGaugeRegVersion, version) == Status::Ok &&
        (version & kGaugeVersionMask) == kGaugeVersionValue) {
      ready_ = true;

      // The gauge tracks the cell over time. A power-up reading is taken under whatever load we are
      // drawing, so only clear the flag on a real power-up and leave a warm reboot's estimate alone
      uint16_t status = 0;
      const bool statusOk = read16(kGaugeRegStatus, status) == Status::Ok;
      const bool poweredUp = statusOk && ((status >> 8) & kGaugeStatusResetIndicator) != 0;
      if (poweredUp) {
        write16(kGaugeRegStatus, status & ~(static_cast<uint16_t>(kGaugeStatusResetIndicator) << 8));
      }

      LOG_I(Soc, "init") {
        line.field("ok", true);
        line.field("attempts", attempt + 1);
        line.fieldHex("ver", version, 4);
        line.field("powerup", poweredUp);
        line.field("ms", system_.nowMs() - startMs);
      }
      return Status::Ok;
    }

    system_.waitMs(kInitRetryDelayMs);
  }

  ready_ = false;
  LOG_E(Soc, "init") {
    line.field("ok", false);
    line.field("attempts", kInitAttempts);
    line.field("retry", "never");
    line.field("nexts", kFirstTxIntervalMs / 1000);   // no battery reading ever, so the interval stays at 5 s
  }
  return Status::NoAck;
}

Status FuelGauge::read()
{
  if (!ready_) {
    data_.valid = false;
    LOG_W(Soc, "refused") {
      line.field("call", "read");
      line.field("ready", false);
    }
    return Status::NotReady;
  }

  uint16_t raw = 0;
  const Status status = read16(kGaugeRegSoc, raw);
  if (status != Status::Ok) {
    // A failed transfer stays a failure. The old value keeps its place in the packet, marked invalid
    data_.valid = false;
    rawPercent_ = 0.0f;
    LOG_W(Soc, "read") {
      line.field("ok", false);
      line.field("reason", "bus");
    }
    return status;
  }

  const float percentage = static_cast<float>(raw) / 256.0f;
  rawPercent_ = percentage;

  // constrain is a macro that evaluates its args twice, keep it on a local
  data_.cellPercentage = constrain(percentage, 0.0f, 100.0f);
  data_.updatedMs = system_.nowMs();
  data_.valid = true;
  return Status::Ok;
}

} // namespace ChipSatDevices

#endif
