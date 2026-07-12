#pragma once

#include <Arduino.h>
#include <Wire.h>

#include <Adafruit_BME280.h>
#include <Adafruit_MAX1704X.h>
#include <SparkFun_BNO08x_Arduino_Library.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>

// -----------------------------------------------------------------------------
// ChipSat sensor pin and address configuration
//
// These defaults come from the supplied sensor test sketches and the existing
// MAX-M10S UART wiring. Define any of these before including Sensors.h to
// override them for a different board revision.
// -----------------------------------------------------------------------------

#ifndef CHIPSAT_BNO08X_INT_PIN // Norm PB3
#define CHIPSAT_BNO08X_INT_PIN -1
#endif

#ifndef CHIPSAT_BNO08X_RST_PIN // Norm PB4
#define CHIPSAT_BNO08X_RST_PIN -1
#endif

#ifndef CHIPSAT_BNO08X_ADDRESS
#define CHIPSAT_BNO08X_ADDRESS 0x4A
#endif

#ifndef CHIPSAT_BME280_ADDRESS
#define CHIPSAT_BME280_ADDRESS 0x77
#endif

#ifndef CHIPSAT_GPS_RX_PIN
#define CHIPSAT_GPS_RX_PIN PC1
#endif

#ifndef CHIPSAT_GPS_TX_PIN
#define CHIPSAT_GPS_TX_PIN PC0
#endif


// debug Level:
// 0: None
// 1: Boot Only
// 2: All
#ifndef DEBUG_PRINT
#define DEBUG_PRINT 2
#endif

namespace ChipSatSensors
{

struct Vector3f
{
  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;
};

struct Quaternionf
{
  float i = 0.0f;
  float j = 0.0f;
  float k = 0.0f;
  float real = 1.0f;
};

struct IMUData
{
  // Matches the supplied IMU test: gravity is removed by the BNO08x.
  Vector3f linearAccelerationMps2;
  Vector3f gyroscopeRadPerSec;
  Vector3f magnetometerMicroTesla;
  Quaternionf orientation;

  bool linearAccelerationValid = false;
  bool gyroscopeValid = false;
  bool magnetometerValid = false;
  bool orientationValid = false;
  bool valid = false;

  uint32_t linearAccelerationUpdatedMs = 0;
  uint32_t gyroscopeUpdatedMs = 0;
  uint32_t magnetometerUpdatedMs = 0;
  uint32_t orientationUpdatedMs = 0;
};

struct GPSData
{
  // Native SparkFun u-blox units:
  // latitude / longitude: degrees multiplied by 10^7
  // altitude MSL: millimeters
  int32_t latitudeE7 = 0;
  int32_t longitudeE7 = 0;
  int32_t altitudeMSLmm = 0;

  bool valid = false;
  uint32_t updatedMs = 0;
};

struct StateOfChargeData
{
  float cellPercentage = 0.0f;

  bool valid = false;
  uint32_t updatedMs = 0;
};

struct EnvironmentalData
{
  float temperatureC = 0.0f;
  float pressurePa = 0.0f;
  float humidityPercent = 0.0f;
  float altitudeM = 0.0f;

  bool valid = false;
  uint32_t updatedMs = 0;
};

struct SensorData
{
  IMUData imu;
  GPSData gps;
  StateOfChargeData stateOfCharge;
  EnvironmentalData environmental;
};

class Sensors
{
public:
  explicit Sensors(TwoWire &wirePort = Wire);

  // Initializes all four sensors. debugPort can point to any working Serial,
  // HardwareSerial, or SoftwareSerial object. Pass nullptr for no messages.
  // Returns true only if every sensor initializes successfully.
  bool begin(Stream *debugPort = nullptr,
             uint32_t gpsBaud = 9600,
             uint16_t imuReportIntervalMs = 50);

  // Call service() continuously from loop(). It drains BNO08x events and keeps
  // the latest acceleration, gyro, magnetometer, and quaternion values.
  void service();

  // Individual read functions. These update the public data object.
  bool updateIMU();

  // Blocks until at least one NEW event has arrived for every enabled BNO08x
  // report: linear acceleration, gyro, magnetometer, and rotation vector.
  // Returns false if any report is still missing when timeoutMs expires.
  bool waitForFreshIMUData(uint16_t timeoutMs = 500);

  bool readGPS(uint16_t maxWaitMs = 250);
  bool readStateOfCharge();
  bool readEnvironmental();

  // Convenience function for sensors that are normally sampled only when a
  // telemetry packet is being prepared.
  bool readSlowSensors(uint16_t gpsMaxWaitMs = 250);

  // Low power controls. sleepGPS(0) sleeps indefinitely until UART activity
  // wakes the MAX-M10S. The latest values in data are preserved while sleeping
  // so they can be packetized and transmitted after the sensors are shut down.
  bool sleepIMU();
  bool wakeIMU();
  bool sleepGPS(uint32_t durationMs = 0);
  bool wakeGPS(uint16_t wakeDelayMs = 250);

  // Simple pair intended to wrap a radio transmission:
  //   sensors.sleepForTransmit();
  //   radio.transmit(...);
  //   sensors.wakeAfterTransmit();
  bool sleepForTransmit(uint32_t gpsSleepDurationMs = 0);
  bool wakeAfterTransmit(uint16_t gpsWakeDelayMs = 250);

  bool imuSleeping() const;
  bool gpsSleeping() const;

  // Change the reference used by BME280 pressure-derived altitude.
  void setSeaLevelPressureHpa(float pressureHpa);
  float seaLevelPressureHpa() const;

  // Per-sensor initialization state.
  bool imuReady() const;
  bool gpsReady() const;
  bool stateOfChargeReady() const;
  bool environmentalReady() const;
  bool allReady() const;

  // Direct access is available for later sensor-specific configuration, such
  // as GNSS sleep commands or BNO08x calibration.
  BNO08x &imu();
  SFE_UBLOX_GNSS &gps();
  Adafruit_MAX17048 &fuelGauge();
  Adafruit_BME280 &bme280();
  HardwareSerial &gpsSerial();

  // Latest readable, uncompressed measurements.
  SensorData data;

private:
  bool beginIMU();
  bool configureIMUReports(uint16_t startupDelayMs = 0);
  bool beginGPS();
  bool beginStateOfCharge();
  bool beginEnvironmental();

  void invalidateIMUData();
  void updateOverallIMUValidity();


  TwoWire &_wire;
  HardwareSerial _gpsSerial;

  BNO08x _imu;
  SFE_UBLOX_GNSS _gps;
  Adafruit_MAX17048 _fuelGauge;
  Adafruit_BME280 _bme280;

  Stream *_debugPort = nullptr;

  uint32_t _gpsBaud = 9600;
  uint16_t _imuReportIntervalMs = 50;
  float _seaLevelPressureHpa = 1013.25f;

  bool _imuReady = false;
  bool _gpsReady = false;
  bool _stateOfChargeReady = false;
  bool _environmentalReady = false;

  bool _imuSleeping = false;
  bool _gpsSleeping = false;
};

} // namespace ChipSatSensors
