#include <Arduino.h>
#include "FlightController.h"
#include "ImuValidity.h"
#include "TxInterval.h"
#include "../config.h"
#include "../constants.h"
#include "../log/Log.h"

namespace ChipSatApp
{

using ChipSatPlatform::Status;
using namespace ChipSatConfig;
using namespace ChipSatConstants;

[[maybe_unused]] static uint8_t imuValidBits(const ChipSatSensors::IMUData &imu)
{
  return (imu.linearAccelerationValid ? 1U : 0U) | (imu.gyroscopeValid ? 2U : 0U) |
         (imu.magnetometerValid ? 4U : 0U) | (imu.orientationValid ? 8U : 0U);
}

FlightController::FlightController(ChipSatPlatform::System &system,
                                   ChipSatPlatform::I2cBus &i2c,
                                   ChipSatPlatform::Uart &console,
                                   ChipSatPlatform::Uart &gpsPort,
                                   ChipSatDevices::Imu &imu,
                                   ChipSatDevices::Gps &gps,
                                   ChipSatDevices::EnvSensor &env,
                                   ChipSatDevices::FuelGauge &gauge,
                                   ChipSatDevices::Radio &radio,
                                   ChipSatDevices::Led &led)
  : system_(system), i2c_(i2c), console_(console), gpsPort_(gpsPort), imu_(imu), gps_(gps), env_(env),
    gauge_(gauge), radio_(radio), led_(led), txIntervalMs_(kTxIntervalMs)
{
}

void FlightController::setup()
{
  led_.begin();

  console_.begin(kConsoleBaud);
  ChipSatLog::begin(console_, system_);

  LOG_BOOT(Boot, "start") { line.field("log", CHIPSAT_LOG_LEVEL); }
  logResetCause();
  system_.startWatchdog(kWatchdogTimeoutMs);

  // Radio before the sensors, so a sensor that hangs at boot still leaves one packet on air.
  // Never halts, a missing sensor just goes out invalid
  radioReady_ = configRadio();
  sendStatusPacket();

  beginSensors();

  LOG_BOOT(Boot, "setup") {
    line.field("done", true);
    line.field("radiostate", radio_.lastCode());
  }
}

void FlightController::logResetCause()
{
  const ChipSatPlatform::ResetCause cause = system_.readAndClearResetCause();

  LOG_BOOT(Rst, "cause") {
    line.fieldHex("csr", cause.csr, 8);
    line.field("lpwr", cause.lowPower);
    line.field("wwdg", cause.windowWatchdog);
    line.field("iwdg", cause.independentWatchdog);
    line.field("sft", cause.software);
    line.field("bor", cause.brownOut);
    line.field("pin", cause.pin);
    line.field("obl", cause.optionByteLoad);
    line.field("rfilar", cause.radioIllegalAccess);
  }
}

void FlightController::beginSensors()
{
  i2c_.begin(kI2cSdaPin, kI2cSclPin);
  gpsPort_.begin(kGpsBaud);   // before the IMU

  imu_.begin(kImuReportIntervalMs);
  keepAlive();
  gps_.begin();
  keepAlive();
  gauge_.begin();
  keepAlive();
  env_.begin();
}

// Whatever is known so far, all fresh bits clear
void FlightController::sendStatusPacket()
{
  ChipSatTelemetry::encodePacket(data_, packetCounter_, kChipSatId, false, packet_);
  packetCounter_++;
  LOG_PACKET(reinterpret_cast<const uint8_t *>(&packet_), sizeof(packet_));
  transmit();
}

// A slow sensor start after a reboot mid-flight shouldn't leave the air quiet
void FlightController::keepAlive()
{
  if (system_.nowMs() - lastTxMs_ >= kBootKeepAliveMs) {
    sendStatusPacket();
  }
}

bool FlightController::configRadio()
{
  LOG_I(Radio, "start");
  Status status = radio_.begin();
  LOG_D(Radio, "ldro") { line.field("code", radio_.ldroCode()); }
  if (status != Status::Ok) {
    LOG_E(Radio, "failed") {
      line.field("reason", "radiobegin");
      line.field("code", radio_.lastCode());
    }
    return false;
  }
  LOG_I(Radio, "init") {
    line.field("ok", true);
    line.field("code", radio_.lastCode());
  }

  status = radio_.setCurrentLimit();
  if (status != Status::Ok) {
    LOG_E(Radio, "failed") {
      line.field("reason", "radioocp");
      line.field("code", radio_.lastCode());
    }
    return false;
  }
  status = radio_.setOutputPower();
  if (status != Status::Ok) {
    LOG_E(Radio, "failed") {
      line.field("reason", "radiopower");
      line.field("code", radio_.lastCode());
    }
    return false;
  }
  LOG_I(Radio, "power") { line.field("ok", true); }

  status = radio_.applyPaConfig();
  if (status != Status::Ok) {
    LOG_E(Radio, "failed") {
      line.field("reason", "radiopa");
      line.field("code", radio_.lastCode());
    }
    return false;
  }
  LOG_I(Radio, "pa") {
    line.field("ok", true);
    line.field("module", kRadioModuleName);
    line.field("dbm", kPaPowerDbm);   // what SetTxParams gets, the PA row decides the real output
    line.field("code", radio_.lastCode());
  }
  // SPI read, only compiled in with this log line. After the PA step, which resets it
  LOG_I(Radio, "ocp") { line.field("ma", radio_.currentLimitMa(), 2); }
  return true;
}

void FlightController::loop()
{
  system_.feedWatchdog();
  imu_.service();
  gps_.service();

  if (system_.nowMs() - previousReadMs_ >= txIntervalMs_) {
    runCycle();
  }
}

void FlightController::runCycle()
{
  ++cycle_;
  ChipSatLog::setCycle(cycle_);

  CycleReads reads;
  reads.gateMs = system_.nowMs();
  env_.startMeasurement();   // converts while the other reads run
  const Status imuRead = imu_.waitForFresh(kImuFreshWaitCycleMs);
  reads.imuOkay = imuRead == Status::Ok;
  const uint32_t imuDoneMs = system_.nowMs();
  reads.gpsOkay = gps_.read() == Status::Ok;
  const uint32_t gpsDoneMs = system_.nowMs();
  reads.socOkay = gauge_.read() == Status::Ok;
  const uint32_t socDoneMs = system_.nowMs();
  reads.envOkay = env_.read() == Status::Ok;
  const uint32_t envDoneMs = system_.nowMs();

  reads.imuMs = imuDoneMs - reads.gateMs;
  reads.gpsMs = gpsDoneMs - imuDoneMs;
  reads.socMs = socDoneMs - gpsDoneMs;
  reads.envMs = envDoneMs - socDoneMs;

  data_.imu = imu_.data();
  data_.gps = gps_.data();
  data_.stateOfCharge = gauge_.data();
  data_.environmental = env_.data();

  // A GPS frame taken in during the env wait can clear valid after read() said Ok
  reads.gpsOkay = reads.gpsOkay && data_.gps.valid;
  const bool allDataFresh = reads.imuOkay && reads.gpsOkay && reads.socOkay && reads.envOkay;

  // Don't send frozen IMU values as valid. Only the packet copy changes
  const uint8_t staleReports = clearStaleImuReports(data_.imu, system_.nowMs(), kImuStaleMs);
  if (staleReports != 0) {
    LOG_W(Imu, "stale") { line.fieldBits("cleared", staleReports, 4); }
  }

  logReads(reads);
  system_.waitMs(0);   // the console blocks while its buffer drains, read the GPS port

  const uint16_t counterUsed = packetCounter_;
  ChipSatTelemetry::encodePacket(data_, packetCounter_, kChipSatId, allDataFresh, packet_);
  packetCounter_++;

  LOG_PACKET(reinterpret_cast<const uint8_t *>(&packet_), sizeof(packet_));
  system_.waitMs(0);
  LOG_I(Cyc, "packet") {
    line.field("ctr", counterUsed);
    line.fieldHex("valid", packet_.sensorValidity, 2);
    line.field("fresh", allDataFresh);
    line.field("gate", reads.gateMs);
    line.field("imuready", imu_.ready());
    line.field("imusleep", imu_.sleeping());
  }

  // HP module only, the IMU sleeps for the transmission. The packet goes out either way
  const bool imuSleeps = kImuSleepsDuringTx && imu_.ready();
  if (imuSleeps) {
    imu_.sleep();
  }
  transmit();
  if (imuSleeps) {
    imu_.wake();
  }

  // Soft reset the IMU if a report stays silent for a few cycles
  const uint8_t silent = silentImuReports(imu_.eventCounts(), imuCountsLastCycle_);
  for (uint8_t i = 0; i < 4; ++i) {
    imuCountsLastCycle_[i] = imu_.eventCounts()[i];
  }
  if (imu_.ready() && !imu_.sleeping() && silent != 0) {
    ++imuStuckCycles_;
  } else {
    imuStuckCycles_ = 0;
  }
  if (imuStuckCycles_ >= kImuStuckCycles) {
    LOG_W(Imu, "stuck") {
      line.fieldBits("silent", silent, 4);
      line.field("cycles", imuStuckCycles_);
      line.field("resets", imuSoftResets_);
    }
    if (imuSoftResets_ < kImuMaxSoftResets) {
      imu_.resetHub();
      ++imuSoftResets_;
    }
    imuStuckCycles_ = 0;
  }

  const bool fromBattery = kTxFromBattery && reads.socOkay;
  txIntervalMs_ = fromBattery ? txIntervalFromSoc(data_.stateOfCharge.cellPercentage) : kTxIntervalMs;

  previousReadMs_ = system_.nowMs();

  LOG_I(Cyc, "end") {
    line.field("nextms", txIntervalMs_);
    line.field("battery", fromBattery);
    line.field("durms", previousReadMs_ - reads.gateMs);
  }
}

void FlightController::transmit()
{
  bool freshSetup = false;
  if (!radioReady_) {
    freshSetup = true;
    radioReady_ = configRadio();
    if (!radioReady_) {
      LOG_E(Tx, "skipped") { line.field("reason", "radio"); }
      return;
    }
  }

  LOG_D(Tx, "send") { line.field("step", "start"); }

  const uint32_t startMs = system_.nowMs();
  led_.transmitStarted();
  Status status = radio_.startTransmit(reinterpret_cast<const uint8_t *>(&packet_), sizeof(packet_));
  if (status != Status::Ok && !freshSetup) {
    // a radio that reset itself fails here. Set it up again and try once more, so it costs no packet
    radioReady_ = configRadio();
    if (radioReady_) {
      status = radio_.startTransmit(reinterpret_cast<const uint8_t *>(&packet_), sizeof(packet_));
    }
  }
  if (status == Status::Ok) {
    // time on air is exact, 100 ms covers TCXO start and ramp. waitMs keeps the GPS port read meanwhile
    const uint32_t timeoutMs = radio_.timeOnAirMs(sizeof(packet_)) + 100;
    while (!radio_.transmitDone() && system_.nowMs() - startMs <= timeoutMs) {
      system_.waitMs(1);
    }
    const bool done = radio_.transmitDone();
    status = radio_.finishTransmit();
    if (!done) {
      status = Status::Timeout;
    }
  }
  led_.transmitEnded();
  lastTxMs_ = system_.nowMs();
  const uint32_t txMs = lastTxMs_ - startMs;

  if (status == Status::Ok) {
    LOG_I(Tx, "done") {
      line.field("ok", true);
      line.field("code", radio_.lastCode());
      line.field("ms", txMs);
    }
  } else {
    radioReady_ = false;   // set the radio up again before the next packet
    LOG_E(Tx, "done") {
      line.field("ok", false);
      line.field("code", radio_.lastCode());
      line.field("ms", txMs);
    }
  }
}

void FlightController::logReads(const CycleReads &reads) const
{
  LOG_I(Imu, "read") {
    line.field("ok", reads.imuOkay);
    line.field("ms", reads.imuMs);
    line.fieldBits("valid", imuValidBits(data_.imu), 4);
    const uint32_t age[4] = {
      line.ms() - data_.imu.linearAccelerationUpdatedMs,
      line.ms() - data_.imu.gyroscopeUpdatedMs,
      line.ms() - data_.imu.magnetometerUpdatedMs,
      line.ms() - data_.imu.orientationUpdatedMs,
    };
    line.fieldList("age", age, 4);
  }
  LOG_I(Gps, "read") {
    line.field("ok", reads.gpsOkay);
    line.field("fix", gps_.fixType());
    if (gps_.llhChecked()) {
      line.field("llhbad", gps_.invalidLlh());
    }
    line.field("valid", data_.gps.valid);
    line.field("ms", reads.gpsMs);
  }
  LOG_I(Soc, "read") {
    line.field("ok", reads.socOkay);
    line.field("valid", data_.stateOfCharge.valid);
    line.field("ms", reads.socMs);
  }
  LOG_I(Env, "read") {
    line.field("ok", reads.envOkay);
    line.field("valid", data_.environmental.valid);
    line.field("ms", reads.envMs);
  }

  LOG_D(Imu, "values") {
    line.field("ax", data_.imu.linearAccelerationMps2.x, 3);
    line.field("ay", data_.imu.linearAccelerationMps2.y, 3);
    line.field("az", data_.imu.linearAccelerationMps2.z, 3);
    line.field("gx", data_.imu.gyroscopeRadPerSec.x, 4);
    line.field("gy", data_.imu.gyroscopeRadPerSec.y, 4);
    line.field("gz", data_.imu.gyroscopeRadPerSec.z, 4);
    line.field("mx", data_.imu.magnetometerMicroTesla.x, 2);
    line.field("my", data_.imu.magnetometerMicroTesla.y, 2);
    line.field("mz", data_.imu.magnetometerMicroTesla.z, 2);
  }
  LOG_D(Imu, "quat") {
    line.field("qi", data_.imu.orientation.i, 5);
    line.field("qj", data_.imu.orientation.j, 5);
    line.field("qk", data_.imu.orientation.k, 5);
    line.field("qr", data_.imu.orientation.real, 5);
    const uint32_t updated[4] = {
      data_.imu.linearAccelerationUpdatedMs,
      data_.imu.gyroscopeUpdatedMs,
      data_.imu.magnetometerUpdatedMs,
      data_.imu.orientationUpdatedMs,
    };
    line.fieldList("upd", updated, 4);
    line.fieldList("events", imu_.eventCounts(), 4);
  }
  LOG_D(Gps, "values") {
    line.fieldFixed("lat", data_.gps.latitudeE7, 7);
    line.fieldFixed("lon", data_.gps.longitudeE7, 7);
    line.field("altmm", data_.gps.altitudeMSLmm);
    line.field("upd", data_.gps.updatedMs);
  }
  LOG_D(Soc, "values") {
    line.field("pct", data_.stateOfCharge.cellPercentage, 2);
    line.field("raw", gauge_.rawPercent(), 2);
    line.field("upd", data_.stateOfCharge.updatedMs);
  }
  LOG_D(Env, "values") {
    line.field("temp", data_.environmental.temperatureC, 2);
    line.field("phpa", data_.environmental.pressurePa / 100.0f, 2);
    line.field("rh", data_.environmental.humidityPercent, 2);
    line.field("altm", data_.environmental.altitudeM, 2);
    line.field("upd", data_.environmental.updatedMs);
  }
}

} // namespace ChipSatApp
