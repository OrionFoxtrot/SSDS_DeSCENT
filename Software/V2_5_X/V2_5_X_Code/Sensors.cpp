#include "Sensors.h"

#include <math.h>

namespace ChipSatSensors {

namespace {
// DEBUG_PRINT verbosity levels:
//   0 = no terminal output
//   1 = status, state changes, and errors
//   2 = level 1 plus detailed IMU diagnostics

constexpr uint8_t kInitializationAttempts = 3;          // Maximum number of times to retry initializing each sensor.
constexpr uint8_t kReportEnableAttempts = 3;           // Maximum number of attempts to enable each BNO08x report.
constexpr uint16_t kInitializationRetryDelayMs = 500;  // Delay between failed sensor initialization attempts.
constexpr uint16_t kReportEnableRetryDelayMs = 75;     // Delay before retrying a failed BNO08x report-enable command.
constexpr uint16_t kReportEnableSpacingMs = 50;        // Delay between successful report-enable commands to avoid sending them too quickly.
constexpr uint16_t kIMUInitialStartupDelayMs = 1000;   // Time to let the BNO08x finish starting before enabling reports during initial setup.
constexpr uint16_t kIMUWakeStartupDelayMs = 150;       // Time to let the BNO08x stabilize after waking before restoring reports.
constexpr uint16_t kIMUFreshDataTimeoutMs = 3000;      // Maximum time to wait for one fresh sample from every enabled IMU report.


// Each IMU report type gets its own bit in an 8-bit status value.
// A bit is set when a new report of that type has been received.
constexpr uint8_t kFreshLinearAccelerationBit = 1U << 0;  // 0001
constexpr uint8_t kFreshGyroscopeBit          = 1U << 1;  // 0010
constexpr uint8_t kFreshMagnetometerBit       = 1U << 2;  // 0100
constexpr uint8_t kFreshOrientationBit        = 1U << 3;  // 1000

// All four bits set means that fresh acceleration, gyroscope,
// magnetometer, and orientation reports have all been received.
constexpr uint8_t kAllFreshIMUReportsMask =
  kFreshLinearAccelerationBit |
  kFreshGyroscopeBit |
  kFreshMagnetometerBit |
  kFreshOrientationBit;  // 1111

}  // namespace

Sensors::Sensors(TwoWire &wirePort)
  : _wire(wirePort),
    _gpsSerial(CHIPSAT_GPS_RX_PIN, CHIPSAT_GPS_TX_PIN) {
}

bool Sensors::begin(Stream *debugPort,
                    uint32_t gpsBaud,
                    uint16_t imuReportIntervalMs) {
  _debugPort = debugPort;
  _gpsBaud = gpsBaud;
  _imuReportIntervalMs = imuReportIntervalMs;

  _wire.begin();
  _gpsSerial.begin(_gpsBaud);

  _imuReady = beginIMU();
  _imuSleeping = false;
  _gpsReady = beginGPS();
  _gpsSleeping = false;
  _stateOfChargeReady = beginStateOfCharge();
  _environmentalReady = beginEnvironmental();

  return allReady();
}

void Sensors::service() {
  // Do not touch the BNO08x while it is asleep. I2C traffic is unnecessary and
  // can interfere with a clean low power interval.
  if (!_imuSleeping) {
    updateIMU();
  }
}

bool Sensors::beginIMU() {
  if ((DEBUG_PRINT >= 1) && (_debugPort != nullptr)) {
    _debugPort->println(F("Initializing BNO08x..."));
  }

  for (uint8_t attempt = 0; attempt < kInitializationAttempts; ++attempt) {
    //BNO08X_INT, BNO08X_RST
    if (_imu.begin(CHIPSAT_BNO08X_ADDRESS,
                   _wire,
                   CHIPSAT_BNO08X_INT_PIN,
                   CHIPSAT_BNO08X_RST_PIN)) {
      if (configureIMUReports(kIMUInitialStartupDelayMs)) {
        if ((DEBUG_PRINT >= 1) && (_debugPort != nullptr)) {
          _debugPort->println(F("BNO08x initialized"));
        }
        return true;
      }

      if ((DEBUG_PRINT >= 1) && (_debugPort != nullptr)) {
        _debugPort->println(F("BNO08x found, but one or more reports failed to enable"));
      }
      return false;
    }

    delay(kInitializationRetryDelayMs);
  }

  if ((DEBUG_PRINT >= 1) && (_debugPort != nullptr)) {
    _debugPort->println(F("BNO08x initialization failed"));
  }
  return false;
}

bool Sensors::configureIMUReports(uint16_t startupDelayMs) {
  // The BNO08x can acknowledge modeOn before its report interface is fully
  // ready. Give it a short settling period, then retry every report command a
  // bounded number of times. Linear acceleration is configured explicitly;
  // it is not implied by enabling the normal accelerometer or rotation vector.
  if (startupDelayMs > 0) {
    delay(startupDelayMs);
  }

  bool accelerationOkay = false;
  bool gyroOkay = false;
  bool magnetometerOkay = false;
  bool quaternionOkay = false;

  for (uint8_t attempt = 0;
       attempt < kReportEnableAttempts && !accelerationOkay;
       ++attempt) {
    accelerationOkay =
      _imu.enableLinearAccelerometer(_imuReportIntervalMs);
    delay(accelerationOkay ? kReportEnableSpacingMs // if ok: delay spaceing, if not ok, retry delay
                           : kReportEnableRetryDelayMs);
  }

  for (uint8_t attempt = 0;
       attempt < kReportEnableAttempts && !gyroOkay;
       ++attempt) {
    gyroOkay = _imu.enableGyro(_imuReportIntervalMs);
    delay(gyroOkay ? kReportEnableSpacingMs
                   : kReportEnableRetryDelayMs);
  }

  for (uint8_t attempt = 0;
       attempt < kReportEnableAttempts && !magnetometerOkay;
       ++attempt) {
    magnetometerOkay = _imu.enableMagnetometer(_imuReportIntervalMs);
    delay(magnetometerOkay ? kReportEnableSpacingMs
                           : kReportEnableRetryDelayMs);
  }

  for (uint8_t attempt = 0;
       attempt < kReportEnableAttempts && !quaternionOkay;
       ++attempt) {
    quaternionOkay = _imu.enableRotationVector(_imuReportIntervalMs);
    delay(quaternionOkay ? kReportEnableSpacingMs
                         : kReportEnableRetryDelayMs);
  }

  if ((DEBUG_PRINT >= 2) && (_debugPort != nullptr)) {
    _debugPort->print(F("Accel OK: "));
    _debugPort->println(accelerationOkay);
    _debugPort->print(F("Gyro OK: "));
    _debugPort->println(gyroOkay);
    _debugPort->print(F("Mag OK: "));
    _debugPort->println(magnetometerOkay);
    _debugPort->print(F("Quat OK: "));
    _debugPort->println(quaternionOkay);
  }

  return accelerationOkay && gyroOkay && magnetometerOkay && quaternionOkay;
}

bool Sensors::waitForFreshIMUData(uint16_t timeoutMs) {
  if (!_imuReady || _imuSleeping) {
    if ((DEBUG_PRINT >= 1) && (_debugPort != nullptr)) {
      _debugPort->println(F("Cannot collect fresh IMU data: BNO08x is unavailable"));
    }
    return false;
  }

  // Drain anything which was already queued before this freshness check.
  // Samples counted below must therefore be processed after this call begins.
  updateIMU();

  // Valid flags alone are not enough: they may describe samples captured
  // before the previous transmission. Save every report timestamp so this
  // call can prove that each report has advanced at least once.
  const uint32_t previousAccelerationMs =
    data.imu.linearAccelerationUpdatedMs;
  const uint32_t previousGyroscopeMs =
    data.imu.gyroscopeUpdatedMs;
  const uint32_t previousMagnetometerMs =
    data.imu.magnetometerUpdatedMs;
  const uint32_t previousOrientationMs =
    data.imu.orientationUpdatedMs;

  uint8_t freshReports = 0;
  const uint32_t startMs = millis();

  do {
    // One call drains every currently queued BNO08x event. Reports are
    // independent, so several iterations may be needed before all four arrive.
    updateIMU();

    if (data.imu.linearAccelerationValid && data.imu.linearAccelerationUpdatedMs != previousAccelerationMs) {
      freshReports |= kFreshLinearAccelerationBit;
    }

    if (data.imu.gyroscopeValid && data.imu.gyroscopeUpdatedMs != previousGyroscopeMs) {
      freshReports |= kFreshGyroscopeBit;
    }

    if (data.imu.magnetometerValid && data.imu.magnetometerUpdatedMs != previousMagnetometerMs) {
      freshReports |= kFreshMagnetometerBit;
    }

    if (data.imu.orientationValid && data.imu.orientationUpdatedMs != previousOrientationMs) {
      freshReports |= kFreshOrientationBit;
    }

    if (freshReports == kAllFreshIMUReportsMask) {
      return true;
    }

    delay(2);
  } while (static_cast<uint32_t>(millis() - startMs) < timeoutMs);

  if ((DEBUG_PRINT >= 2) && (_debugPort != nullptr)) {
    _debugPort->print(F("Fresh IMU data timed out; missing:"));

    if ((freshReports & kFreshLinearAccelerationBit) == 0) {
      _debugPort->print(F(" linaccel"));
    }
    if ((freshReports & kFreshGyroscopeBit) == 0) {
      _debugPort->print(F(" gyro"));
    }
    if ((freshReports & kFreshMagnetometerBit) == 0) {
      _debugPort->print(F(" mag"));
    }
    if ((freshReports & kFreshOrientationBit) == 0) {
      _debugPort->print(F(" quat"));
    }

    _debugPort->println();
  }

  return false;
}

bool Sensors::beginGPS() {
  if ((DEBUG_PRINT >= 1) && (_debugPort != nullptr)) {
    _debugPort->println(F("Initializing MAX-M10S GNSS over UART..."));
  }

  for (uint8_t attempt = 0; attempt < kInitializationAttempts; attempt++) {
    if (_gps.begin(_gpsSerial)) {
      // Keep only UBX output on UART to reduce unsolicited NMEA traffic. This
      // changes the active configuration but does not write it to flash.
      _gps.setUART1Output(COM_TYPE_UBX);

      if ((DEBUG_PRINT >= 1) && (_debugPort != nullptr)) {
        _debugPort->println(F("MAX-M10S GNSS initialized"));
      }
      return true;
    }

    delay(kInitializationRetryDelayMs);
  }

  if ((DEBUG_PRINT >= 1) && (_debugPort != nullptr)) {
    _debugPort->println(F("MAX-M10S GNSS initialization failed"));
  }
  return false;
}

bool Sensors::beginStateOfCharge() {
  if ((DEBUG_PRINT >= 1) && (_debugPort != nullptr)) {
    _debugPort->println(F("Initializing MAX17048 fuel gauge..."));
  }

  for (uint8_t attempt = 0; attempt < kInitializationAttempts; ++attempt) {
    if (_fuelGauge.begin(&_wire)) {
      if ((DEBUG_PRINT >= 1) && (_debugPort != nullptr)) {
        _debugPort->println(F("MAX17048 initialized"));
      }
      return true;
    }

    delay(kInitializationRetryDelayMs);
  }

  if ((DEBUG_PRINT >= 1) && (_debugPort != nullptr)) {
    _debugPort->println(F("MAX17048 initialization failed"));
  }
  return false;
}

bool Sensors::beginEnvironmental() {
  if ((DEBUG_PRINT >= 1) && (_debugPort != nullptr)) {
    _debugPort->println(F("Initializing BME280..."));
  }

  for (uint8_t attempt = 0; attempt < kInitializationAttempts; ++attempt) {
    if (_bme280.begin(CHIPSAT_BME280_ADDRESS, &_wire)) {
      if ((DEBUG_PRINT >= 1) && (_debugPort != nullptr)) {
        _debugPort->println(F("BME280 initialized"));
      }
      return true;
    }

    delay(kInitializationRetryDelayMs);
  }

  if ((DEBUG_PRINT >= 1) && (_debugPort != nullptr)) {
    _debugPort->println(F("BME280 initialization failed"));
  }
  return false;
}

bool Sensors::updateIMU() {
  if (!_imuReady || _imuSleeping) {
    return false;
  }

  if (_imu.wasReset()) {
    if ((DEBUG_PRINT >= 1) && (_debugPort != nullptr)) {
      _debugPort->println(F("BNO08x reset detected; reenabling reports"));
    }
    invalidateIMUData();
    _imuReady = configureIMUReports(kIMUWakeStartupDelayMs);

    if (!_imuReady) {
      if ((DEBUG_PRINT >= 1) && (_debugPort != nullptr)) {
        _debugPort->println(F("Failed to reenable BNO08x reports"));
      }
      return false;
    }
  }

  bool updated = false;

  // Each event contains one enabled report. Drain all pending reports so the
  // public data object always contains the latest value from each IMU output.
  while (_imu.getSensorEvent()) {
    const uint32_t now = millis();

    switch (_imu.getSensorEventID()) {
      case SENSOR_REPORTID_LINEAR_ACCELERATION:
        data.imu.linearAccelerationMps2.x = _imu.getLinAccelX();
        data.imu.linearAccelerationMps2.y = _imu.getLinAccelY();
        data.imu.linearAccelerationMps2.z = _imu.getLinAccelZ();
        data.imu.linearAccelerationValid = true;
        data.imu.linearAccelerationUpdatedMs = now;
        updated = true;
        break;

      case SENSOR_REPORTID_GYROSCOPE_CALIBRATED:
        data.imu.gyroscopeRadPerSec.x = _imu.getGyroX();
        data.imu.gyroscopeRadPerSec.y = _imu.getGyroY();
        data.imu.gyroscopeRadPerSec.z = _imu.getGyroZ();
        data.imu.gyroscopeValid = true;
        data.imu.gyroscopeUpdatedMs = now;
        updated = true;
        break;

      case SENSOR_REPORTID_MAGNETIC_FIELD:
        data.imu.magnetometerMicroTesla.x = _imu.getMagX();
        data.imu.magnetometerMicroTesla.y = _imu.getMagY();
        data.imu.magnetometerMicroTesla.z = _imu.getMagZ();
        data.imu.magnetometerValid = true;
        data.imu.magnetometerUpdatedMs = now;
        updated = true;
        break;

      case SENSOR_REPORTID_ROTATION_VECTOR:
        data.imu.orientation.i = _imu.getQuatI();
        data.imu.orientation.j = _imu.getQuatJ();
        data.imu.orientation.k = _imu.getQuatK();
        data.imu.orientation.real = _imu.getQuatReal();
        data.imu.orientationValid = true;
        data.imu.orientationUpdatedMs = now;
        updated = true;
        break;

      default:
        break;
    }
  }

  updateOverallIMUValidity();
  return updated;
}

bool Sensors::readGPS(uint16_t maxWaitMs) {
  // Do not poll the receiver while it is in software backup mode. UART traffic
  // is the wake source, so a normal query would wake it unintentionally.
  if (!_gpsReady || _gpsSleeping) {
    return false;
  }

  // getPVT updates the library's cached NAV-PVT data. The requested values are
  // then copied in their native compact integer formats.
  if (!_gps.getPVT(maxWaitMs)) {
    data.gps.valid = false;
    return false;
  }

  data.gps.latitudeE7 = _gps.getLatitude();
  data.gps.longitudeE7 = _gps.getLongitude();
  data.gps.altitudeMSLmm = _gps.getAltitudeMSL();
  data.gps.updatedMs = millis();

  // A 2D or 3D fix is required before the position is marked valid.
  const uint8_t fixType = _gps.getFixType();
  data.gps.valid = (fixType >= 2) && !_gps.getInvalidLlh();

  return data.gps.valid;
}

bool Sensors::readStateOfCharge() {
  if (!_stateOfChargeReady) {
    data.stateOfCharge.valid = false;
    return false;
  }

  const float percentage = _fuelGauge.cellPercent();

  if (!isfinite(percentage)) {
    data.stateOfCharge.valid = false;
    return false;
  }

  data.stateOfCharge.cellPercentage =
    constrain(percentage, 0.0f, 100.0f);
  data.stateOfCharge.updatedMs = millis();
  data.stateOfCharge.valid = true;

  return true;
}

bool Sensors::readEnvironmental() {
  if (!_environmentalReady) {
    data.environmental.valid = false;
    return false;
  }

  const float temperatureC = _bme280.readTemperature();
  const float pressurePa = _bme280.readPressure();
  const float humidityPercent = _bme280.readHumidity();
  const float altitudeM = _bme280.readAltitude(_seaLevelPressureHpa);

  const bool readingsValid =
    isfinite(temperatureC) && isfinite(pressurePa) && isfinite(humidityPercent) && isfinite(altitudeM);

  if (!readingsValid) {
    data.environmental.valid = false;
    return false;
  }

  data.environmental.temperatureC = temperatureC;
  data.environmental.pressurePa = pressurePa;
  data.environmental.humidityPercent = humidityPercent;
  data.environmental.altitudeM = altitudeM;
  data.environmental.updatedMs = millis();
  data.environmental.valid = true;

  return true;
}

bool Sensors::readSlowSensors(uint16_t gpsMaxWaitMs) {
  const bool gpsOkay = readGPS(gpsMaxWaitMs);
  const bool stateOfChargeOkay = readStateOfCharge();
  const bool environmentalOkay = readEnvironmental();

  return gpsOkay && stateOfChargeOkay && environmentalOkay;
}


bool Sensors::sleepIMU() {
  if (!_imuReady) {
    if ((DEBUG_PRINT >= 2) && (_debugPort != nullptr)) {
      _debugPort->println(F("Cannot sleep BNO08x: IMU is not initialized"));
    }
    return false;
  }

  if (_imuSleeping) {
    return true;
  }

  if (!_imu.modeSleep()) {
    if ((DEBUG_PRINT >= 2) && (_debugPort != nullptr)) {
      _debugPort->println(F("Failed to put BNO08x into sleep mode"));
    }
    return false;
  }

  _imuSleeping = true;
  if ((DEBUG_PRINT >= 2) && (_debugPort != nullptr)) {
    _debugPort->println(F("BNO08x sleeping"));
  }
  return true;
}

bool Sensors::wakeIMU() {
  if (!_imuReady) {
    if ((DEBUG_PRINT >= 2) && (_debugPort != nullptr)) {
      _debugPort->println(F("Cannot wake BNO08x: IMU is not initialized"));
    }
    return false;
  }

  if (!_imuSleeping) {
    return true;
  }

  if (!_imu.modeOn()) {
    if ((DEBUG_PRINT >= 2) && (_debugPort != nullptr)) {
      _debugPort->println(F("Failed to wake BNO08x"));
    }
    return false;
  }

  // Clear the software sleep guard before configuring reports because the
  // fresh-data wait below calls updateIMU().
  _imuSleeping = false;

  // Do not assume feature reports survive BNO08x sleep. Reset all report

  if (!configureIMUReports(kIMUWakeStartupDelayMs)) {
    if ((DEBUG_PRINT >= 2) && (_debugPort != nullptr)) {
      _debugPort->println(F("BNO08x woke, but reports could not be restored"));
    }
    return false;
  }

  // modeOn and successful report commands only prove that commands were
  // accepted. Require one actual NEW event from every enabled report before
  // declaring the IMU ready.
  if (!waitForFreshIMUData(kIMUFreshDataTimeoutMs)) {
    if ((DEBUG_PRINT >= 2) && (_debugPort != nullptr)) {
      _debugPort->println(F("BNO08x woke, but not every fresh report arrived"));
    }
    return false;
  }

  if ((DEBUG_PRINT >= 2) && (_debugPort != nullptr)) {
    _debugPort->println(F("BNO08x awake; all fresh reports received"));
  }
  return true;
}

bool Sensors::sleepGPS(uint32_t durationMs) {
  if (!_gpsReady) {
    if ((DEBUG_PRINT >= 2) && (_debugPort != nullptr)) {
      _debugPort->println(F("Cannot sleep MAX-M10S: GPS is not initialized"));
    }
    return false;
  }

  if (_gpsSleeping) {
    return true;
  }

  // durationMs == 0 means remain in software backup mode until a wake source
  // occurs. UART RX is selected so no extra MAX-M10S wake pin is required.
  const bool commandAccepted = _gps.powerOffWithInterrupt(
    durationMs,
    VAL_RXM_PMREQ_WAKEUPSOURCE_UARTRX);

  if (!commandAccepted) {
    if ((DEBUG_PRINT >= 1) && (_debugPort != nullptr)) {
      _debugPort->println(F("Failed to put MAX-M10S into software backup mode"));
    }
    return false;
  }

  _gpsSleeping = true;
  if ((DEBUG_PRINT >= 2) && (_debugPort != nullptr)) {
    _debugPort->println(F("MAX-M10S sleeping"));
  }
  return true;
}

bool Sensors::wakeGPS(uint16_t wakeDelayMs) {
  if (!_gpsReady) {
    if ((DEBUG_PRINT >= 1) && (_debugPort != nullptr)) {
      _debugPort->println(F("Cannot wake MAX-M10S: GPS is not initialized"));
    }
    return false;
  }

  if (!_gpsSleeping) {
    return true;
  }

  // A start bit on UART RX creates the edge required to wake the receiver.
  // 0xFF is intentionally not a valid standalone UBX message, so the receiver
  // discards it after waking.
  _gpsSerial.write(static_cast<uint8_t>(0xFF));
  _gpsSerial.flush();
  delay(wakeDelayMs);

  _gpsSleeping = false;

  // MAX-M10S software backup can clear RAM-only configuration. Restore the
  // quiet UBX-only UART output setting used during beginGPS().
  if (!_gps.setUART1Output(COM_TYPE_UBX)) {
    if ((DEBUG_PRINT >= 2) && (_debugPort != nullptr)) {
      _debugPort->println(F("MAX-M10S woke, but UART output reconfiguration failed"));
    }
    return false;
  }

  if ((DEBUG_PRINT >= 2) && (_debugPort != nullptr)) {
    _debugPort->println(F("MAX-M10S awake"));
  }
  return true;
}

bool Sensors::sleepForTransmit(uint32_t gpsSleepDurationMs) {
  // Attempt both even if one fails so a single sensor fault does not prevent
  // the other sensor from reducing its load during radio transmission.
  const bool imuOkay = sleepIMU();
  const bool gpsOkay = sleepGPS(gpsSleepDurationMs);
  return imuOkay && gpsOkay;
}

bool Sensors::wakeAfterTransmit(uint16_t gpsWakeDelayMs) {
  // Wake GPS first because it needs a short startup delay. The IMU wake command
  // is then issued after that delay.
  const bool gpsOkay = wakeGPS(gpsWakeDelayMs);
  const bool imuOkay = wakeIMU();
  return gpsOkay && imuOkay;
}

bool Sensors::imuSleeping() const {
  return _imuSleeping;
}

bool Sensors::gpsSleeping() const {
  return _gpsSleeping;
}

void Sensors::setSeaLevelPressureHpa(float pressureHpa) {
  if (isfinite(pressureHpa) && pressureHpa > 0.0f) {
    _seaLevelPressureHpa = pressureHpa;
  }
}

float Sensors::seaLevelPressureHpa() const {
  return _seaLevelPressureHpa;
}

bool Sensors::imuReady() const {
  return _imuReady;
}

bool Sensors::gpsReady() const {
  return _gpsReady;
}

bool Sensors::stateOfChargeReady() const {
  return _stateOfChargeReady;
}

bool Sensors::environmentalReady() const {
  return _environmentalReady;
}

bool Sensors::allReady() const {
  return _imuReady && _gpsReady && _stateOfChargeReady && _environmentalReady;
}

BNO08x &Sensors::imu() {
  return _imu;
}

SFE_UBLOX_GNSS &Sensors::gps() {
  return _gps;
}

Adafruit_MAX17048 &Sensors::fuelGauge() {
  return _fuelGauge;
}

Adafruit_BME280 &Sensors::bme280() {
  return _bme280;
}

HardwareSerial &Sensors::gpsSerial() {
  return _gpsSerial;
}

void Sensors::invalidateIMUData() {
  data.imu.linearAccelerationValid = false;
  data.imu.gyroscopeValid = false;
  data.imu.magnetometerValid = false;
  data.imu.orientationValid = false;
  data.imu.valid = false;
}

void Sensors::updateOverallIMUValidity() {
  data.imu.valid =
    data.imu.linearAccelerationValid && data.imu.gyroscopeValid && data.imu.magnetometerValid && data.imu.orientationValid;
}


}  // namespace ChipSatSensors
