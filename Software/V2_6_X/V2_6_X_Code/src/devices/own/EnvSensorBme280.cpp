#include <Arduino.h>
#include <math.h>
#include "../../config.h"
#include "../../constants.h"

#if CHIPSAT_ENV_DRIVER == CHIPSAT_DRIVER_OWN

#include "../EnvSensor.h"
#include "../../log/Log.h"

namespace ChipSatDevices
{

using ChipSatPlatform::Status;
using namespace ChipSatConfig;
using namespace ChipSatConstants;

// Bosch's compensation maths, BME280 datasheet section 4.2.3. Raw readings mean nothing without it:
// every chip carries its own calibration constants and these formulas turn counts into real units.
// Copied from the datasheet, same as the Adafruit driver had them, minus its temperature offset

// The 18 factory constants, read out of the chip at boot
struct Bme280Calibration
{
  uint16_t t1 = 0;
  int16_t t2 = 0, t3 = 0;
  uint16_t p1 = 0;
  int16_t p2 = 0, p3 = 0, p4 = 0, p5 = 0, p6 = 0, p7 = 0, p8 = 0, p9 = 0;
  uint8_t h1 = 0;
  int16_t h2 = 0;
  uint8_t h3 = 0;
  int16_t h4 = 0, h5 = 0;
  int8_t h6 = 0;
};

static int16_t signed16(const uint8_t *from) { return static_cast<int16_t>(from[0] | from[1] << 8); }
static uint16_t unsigned16(const uint8_t *from) { return static_cast<uint16_t>(from[0] | from[1] << 8); }

static void parseCalibration(const uint8_t *calib1, const uint8_t *calib2, Bme280Calibration &out)
{
  out.t1 = unsigned16(&calib1[0]);
  out.t2 = signed16(&calib1[2]);
  out.t3 = signed16(&calib1[4]);

  out.p1 = unsigned16(&calib1[6]);
  out.p2 = signed16(&calib1[8]);
  out.p3 = signed16(&calib1[10]);
  out.p4 = signed16(&calib1[12]);
  out.p5 = signed16(&calib1[14]);
  out.p6 = signed16(&calib1[16]);
  out.p7 = signed16(&calib1[18]);
  out.p8 = signed16(&calib1[20]);
  out.p9 = signed16(&calib1[22]);

  out.h1 = calib1[25];   // 0xA1
  out.h2 = signed16(&calib2[0]);
  out.h3 = calib2[2];

  // h4 and h5 share the byte at 0xE5, high nibble to one and low nibble to the other
  out.h4 = static_cast<int16_t>(static_cast<int8_t>(calib2[3]) << 4 | (calib2[4] & 0x0F));
  out.h5 = static_cast<int16_t>(static_cast<int8_t>(calib2[5]) << 4 | (calib2[4] >> 4));
  out.h6 = static_cast<int8_t>(calib2[6]);
}

static int32_t temperatureFine(const Bme280Calibration &cal, int32_t adcT)
{
  int32_t var1 = static_cast<int32_t>((adcT / 8) - (static_cast<int32_t>(cal.t1) * 2));
  var1 = (var1 * static_cast<int32_t>(cal.t2)) / 2048;
  int32_t var2 = static_cast<int32_t>((adcT / 16) - static_cast<int32_t>(cal.t1));
  var2 = (((var2 * var2) / 4096) * static_cast<int32_t>(cal.t3)) / 16384;
  return var1 + var2;
}

static float temperature(int32_t tFine)
{
  const int32_t t = (tFine * 5 + 128) / 256;
  return static_cast<float>(t) / 100.0f;
}

static float pressure(const Bme280Calibration &cal, int32_t adcP, int32_t tFine)
{
  int64_t var1 = static_cast<int64_t>(tFine) - 128000;
  int64_t var2 = var1 * var1 * static_cast<int64_t>(cal.p6);
  var2 = var2 + ((var1 * static_cast<int64_t>(cal.p5)) * 131072);
  var2 = var2 + (static_cast<int64_t>(cal.p4) * 34359738368);
  var1 = ((var1 * var1 * static_cast<int64_t>(cal.p3)) / 256) +
         ((var1 * static_cast<int64_t>(cal.p2) * 4096));
  const int64_t var3 = static_cast<int64_t>(1) * 140737488355328;
  var1 = (var3 + var1) * static_cast<int64_t>(cal.p1) / 8589934592;

  if (var1 == 0) {
    return 0.0f;   // the division below would trap. Bosch returns zero here, so do we
  }

  int64_t var4 = 1048576 - adcP;
  var4 = (((var4 * 2147483648) - var2) * 3125) / var1;
  var1 = (static_cast<int64_t>(cal.p9) * (var4 / 8192) * (var4 / 8192)) / 33554432;
  var2 = (static_cast<int64_t>(cal.p8) * var4) / 524288;
  var4 = ((var4 + var1 + var2) / 256) + (static_cast<int64_t>(cal.p7) * 16);

  return static_cast<float>(var4) / 256.0f;
}

static float humidity(const Bme280Calibration &cal, int32_t adcH, int32_t tFine)
{
  int32_t var1 = tFine - static_cast<int32_t>(76800);
  int32_t var2 = static_cast<int32_t>(adcH * 16384);
  int32_t var3 = static_cast<int32_t>(static_cast<int32_t>(cal.h4) * 1048576);
  int32_t var4 = static_cast<int32_t>(cal.h5) * var1;
  int32_t var5 = (((var2 - var3) - var4) + static_cast<int32_t>(16384)) / 32768;
  var2 = (var1 * static_cast<int32_t>(cal.h6)) / 1024;
  var3 = (var1 * static_cast<int32_t>(cal.h3)) / 2048;
  var4 = ((var2 * (var3 + static_cast<int32_t>(32768))) / 1024) + static_cast<int32_t>(2097152);
  var2 = ((var4 * static_cast<int32_t>(cal.h2)) + 8192) / 16384;
  var3 = var5 * var2;
  var4 = ((var3 / 32768) * (var3 / 32768)) / 128;
  var5 = var3 - ((var4 * static_cast<int32_t>(cal.h1)) / 16);
  var5 = (var5 < 0 ? 0 : var5);
  var5 = (var5 > 419430400 ? 419430400 : var5);
  const uint32_t h = static_cast<uint32_t>(var5 / 4096);

  return static_cast<float>(h) / 1024.0f;
}

static float altitude(float pressurePa, float seaLevelHpa)
{
  // Same approximation Adafruit used, from the BMP180 datasheet, in float. The double version cost
  // 4.3 kB of flash for a difference well below a millimetre
  const float atmospheric = pressurePa / 100.0f;
  return 44330.0f * (1.0f - powf(atmospheric / seaLevelHpa, 0.1903f));
}

static Bme280Calibration calibration;

EnvSensor::EnvSensor(ChipSatPlatform::I2cBus &bus, ChipSatPlatform::System &system)
  : bus_(bus), system_(system)
{
}

Status EnvSensor::readRegisters(uint8_t reg, uint8_t *buffer, size_t length)
{
  return bus_.writeThenRead(kEnvAddress, &reg, 1, buffer, length);
}

Status EnvSensor::writeRegister(uint8_t reg, uint8_t value)
{
  const uint8_t tx[2] = {reg, value};
  return bus_.write(kEnvAddress, tx, sizeof tx);
}

// Waits for one status bit to clear. A failed read counts as a failure, not as "still busy", which
// is how the Adafruit driver can spin here forever
Status EnvSensor::waitForStatus(uint8_t mask, uint16_t timeoutMs)
{
  const uint32_t deadlineMs = system_.nowMs() + timeoutMs;
  while (true) {
    uint8_t status = 0;
    const Status result = readRegisters(kEnvRegStatus, &status, 1);
    if (result != Status::Ok) {
      return result;
    }
    if ((status & mask) == 0) {
      return Status::Ok;
    }
    if (system_.nowMs() >= deadlineMs) {
      return Status::Timeout;
    }
    system_.waitMs(kEnvMeasurePollMs);
  }
}

Status EnvSensor::begin()
{
  LOG_I(Env, "start");
  const uint32_t startMs = system_.nowMs();

  for (uint8_t attempt = 0; attempt < kInitAttempts; ++attempt) {
    uint8_t chipId = 0;
    if (readRegisters(kEnvRegChipId, &chipId, 1) == Status::Ok && chipId == kEnvChipIdValue &&
        writeRegister(kEnvRegReset, kEnvResetCommand) == Status::Ok) {
      system_.waitMs(kEnvResetSettleMs);

      // The chip copies its calibration out of its own memory after a reset
      uint8_t calib1[kEnvCalib1Length] = {0};
      uint8_t calib2[kEnvCalib2Length] = {0};
      const bool ready = waitForStatus(kEnvStatusImUpdate, kEnvCalibrationWaitMs) == Status::Ok &&
                         readRegisters(kEnvRegCalib1, calib1, sizeof calib1) == Status::Ok &&
                         readRegisters(kEnvRegCalib2, calib2, sizeof calib2) == Status::Ok;

      // Humidity oversampling only takes effect on the next write to ctrl_meas, so it goes first
      const bool configured = ready &&
                              writeRegister(kEnvRegCtrlHum, kEnvOversampling) == Status::Ok &&
                              writeRegister(kEnvRegConfig, kEnvFilterOff) == Status::Ok &&
                              writeRegister(kEnvRegCtrlMeas, measControl(kEnvModeSleep)) == Status::Ok;

      if (configured) {
        parseCalibration(calib1, calib2, calibration);
        ready_ = true;
        LOG_I(Env, "init") {
          line.field("ok", true);
          line.field("attempts", attempt + 1);
          line.field("ms", system_.nowMs() - startMs);
        }
        return Status::Ok;
      }
    }

    system_.waitMs(kInitRetryDelayMs);
  }

  ready_ = false;
  LOG_E(Env, "init") {
    line.field("ok", false);
    line.field("attempts", kInitAttempts);
    line.field("retry", "never");
  }
  return Status::NoAck;
}

uint8_t EnvSensor::measControl(uint8_t mode)
{
  return static_cast<uint8_t>(kEnvOversampling << 5 | kEnvOversampling << 2 | mode);
}

Status EnvSensor::read()
{
  if (!ready_) {
    data_.valid = false;
    LOG_W(Env, "refused") {
      line.field("call", "read");
      line.field("ready", false);
    }
    return Status::NotReady;
  }

  // Forced mode: one measurement on demand, then the sensor goes back to sleep by itself
  Status status = writeRegister(kEnvRegCtrlMeas, measControl(kEnvModeForced));
  if (status == Status::Ok) {
    status = waitForStatus(kEnvStatusMeasuring, kEnvMeasureTimeoutMs);
  }

  uint8_t raw[kEnvDataLength] = {0};
  if (status == Status::Ok) {
    // One burst, 0xF7 to 0xFE, so all three come from the same conversion
    status = readRegisters(kEnvRegData, raw, sizeof raw);
  }

  if (status != Status::Ok) {
    data_.valid = false;   // old values stay, marked invalid
    LOG_W(Env, "read") {
      line.field("ok", false);
      line.field("reason", status == Status::Timeout ? "slow" : "bus");
    }
    return status;
  }

  const int32_t adcP = static_cast<int32_t>(raw[0]) << 12 | static_cast<int32_t>(raw[1]) << 4 | raw[2] >> 4;
  const int32_t adcT = static_cast<int32_t>(raw[3]) << 12 | static_cast<int32_t>(raw[4]) << 4 | raw[5] >> 4;
  const int32_t adcH = static_cast<int32_t>(raw[6]) << 8 | raw[7];

  const int32_t tFine = temperatureFine(calibration, adcT);
  data_.temperatureC = temperature(tFine);
  data_.pressurePa = pressure(calibration, adcP, tFine);
  data_.humidityPercent = humidity(calibration, adcH, tFine);
  data_.altitudeM = altitude(data_.pressurePa, kSeaLevelPressureHpa);
  data_.updatedMs = system_.nowMs();
  data_.valid = true;
  return Status::Ok;
}

} // namespace ChipSatDevices

#endif
