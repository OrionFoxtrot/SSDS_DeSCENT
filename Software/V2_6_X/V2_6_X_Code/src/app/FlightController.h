#pragma once

#include <stdint.h>
#include "../devices/EnvSensor.h"
#include "../devices/FuelGauge.h"
#include "../devices/Gps.h"
#include "../devices/Imu.h"
#include "../devices/Led.h"
#include "../devices/Radio.h"
#include "../devices/SensorData.h"
#include "../platform/I2cBus.h"
#include "../platform/System.h"
#include "../platform/Uart.h"
#include "Telemetry.h"

namespace ChipSatApp
{

// Boot sequence and the telemetry cycle
class FlightController
{
public:
  FlightController(ChipSatPlatform::System &system,
                   ChipSatPlatform::I2cBus &i2c,
                   ChipSatPlatform::Uart &console,
                   ChipSatPlatform::Uart &gpsPort,
                   ChipSatDevices::Imu &imu,
                   ChipSatDevices::Gps &gps,
                   ChipSatDevices::EnvSensor &env,
                   ChipSatDevices::FuelGauge &gauge,
                   ChipSatDevices::Radio &radio,
                   ChipSatDevices::Led &led);

  void setup();
  void loop();

private:
  struct CycleReads
  {
    uint32_t gateMs = 0;
    bool imuOkay = false;
    bool gpsOkay = false;
    bool socOkay = false;
    bool envOkay = false;
    uint32_t imuMs = 0;
    uint32_t gpsMs = 0;
    uint32_t socMs = 0;
    uint32_t envMs = 0;
  };

  void logResetCause();
  void beginSensors();
  void sendStatusPacket();
  void keepAlive();
  bool configRadio();   // false if any step failed
  void runCycle();
  void transmit();
  void logReads(const CycleReads &reads) const;

  ChipSatPlatform::System &system_;
  ChipSatPlatform::I2cBus &i2c_;
  ChipSatPlatform::Uart &console_;
  ChipSatPlatform::Uart &gpsPort_;
  ChipSatDevices::Imu &imu_;
  ChipSatDevices::Gps &gps_;
  ChipSatDevices::EnvSensor &env_;
  ChipSatDevices::FuelGauge &gauge_;
  ChipSatDevices::Radio &radio_;
  ChipSatDevices::Led &led_;

  ChipSatSensors::SensorData data_;
  ChipSatTelemetry::TelemetryPacket packet_{};
  uint16_t packetCounter_ = 0;
  uint32_t previousReadMs_ = 0;
  uint32_t txIntervalMs_;
  uint32_t cycle_ = 0;   // for logging
  bool radioReady_ = false;
  uint32_t lastTxMs_ = 0;

  // IMU stuck check
  uint32_t imuCountsLastCycle_[4] = {0, 0, 0, 0};
  uint8_t imuStuckCycles_ = 0;
  uint8_t imuSoftResets_ = 0;
};

} // namespace ChipSatApp
