#pragma once

#include <stdint.h>
#include "../platform/I2cBus.h"
#include "../platform/Status.h"
#include "../platform/System.h"
#include "SensorData.h"

namespace ChipSatDevices
{

// One bit per report
enum ImuReport : uint8_t
{
  kImuLinearAcceleration = 1U << 0,
  kImuGyroscope = 1U << 1,
  kImuMagnetometer = 1U << 2,
  kImuOrientation = 1U << 3,
  kImuAllReports = 0x0F
};

// BNO085 through the SparkFun library. All four reports stream at 50 ms, it only sleeps around TX
class Imu
{
public:
  Imu(ChipSatPlatform::I2cBus &bus, ChipSatPlatform::System &system);

  ChipSatPlatform::Status begin(uint16_t reportIntervalMs);
  void service();   // call every loop pass
  ChipSatPlatform::Status waitForFresh(uint16_t timeoutMs);
  ChipSatPlatform::Status sleep();   // Ok if already asleep
  ChipSatPlatform::Status wake();    // Ok if it wasn't asleep

  // for when reports stop without any error. Only sends the soft reset, update() turns the
  // reports back on once the IMU reports it
  ChipSatPlatform::Status resetHub();

  bool ready() const { return ready_; }
  bool sleeping() const { return sleeping_; }
  const ChipSatSensors::IMUData &data() const { return data_; }

  // For logging
  uint8_t missingReports() const { return missing_; }
  const uint32_t *eventCounts() const { return events_; }   // reports received since boot

private:
  bool configureReports(uint16_t startupDelayMs);
  bool update();
  void invalidate();
  void updateOverallValidity();

  ChipSatPlatform::I2cBus &bus_;
  ChipSatPlatform::System &system_;
  ChipSatSensors::IMUData data_;
  uint16_t reportIntervalMs_ = 50;
  bool ready_ = false;
  bool sleeping_ = false;

  uint8_t missing_ = 0;
  uint8_t tries_[4] = {0, 0, 0, 0};
  uint32_t events_[4] = {0, 0, 0, 0};
  bool firstResetSeen_ = false;
};

} // namespace ChipSatDevices
