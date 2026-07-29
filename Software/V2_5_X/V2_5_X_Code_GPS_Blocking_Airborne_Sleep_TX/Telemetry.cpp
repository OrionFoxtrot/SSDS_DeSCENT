#include "Telemetry.h"

#include <math.h>
#include <limits.h>

namespace ChipSatTelemetry {

// ---------------------------------------------------------------------------
// Safe scaling helpers
// ---------------------------------------------------------------------------

static int16_t scaleToInt16(
  float value,
  float scaleFactor) {
  if (!isfinite(value)) {
    return 0;
  }

  float scaled = roundf(value * scaleFactor);

  if (scaled > INT16_MAX) {
    return INT16_MAX;
  }

  if (scaled < INT16_MIN) {
    return INT16_MIN;
  }

  return static_cast<int16_t>(scaled);
}

static uint16_t scaleToUint16(
  float value,
  float scaleFactor) {
  if (!isfinite(value) || value <= 0.0f) {
    return 0;
  }

  float scaled = roundf(value * scaleFactor);

  if (scaled > UINT16_MAX) {
    return UINT16_MAX;
  }

  return static_cast<uint16_t>(scaled);
}

static uint8_t scaleToUint8(
  float value,
  float scaleFactor) {
  if (!isfinite(value) || value <= 0.0f) {
    return 0;
  }

  float scaled = roundf(value * scaleFactor);

  if (scaled > UINT8_MAX) {
    return UINT8_MAX;
  }

  return static_cast<uint8_t>(scaled);
}

static int32_t scaleToInt32(
  float value,
  float scaleFactor) {
  if (!isfinite(value)) {
    return 0;
  }

  double scaled = round(
    static_cast<double>(value) * static_cast<double>(scaleFactor));

  if (scaled > INT32_MAX) {
    return INT32_MAX;
  }

  if (scaled < INT32_MIN) {
    return INT32_MIN;
  }

  return static_cast<int32_t>(scaled);
}

// ---------------------------------------------------------------------------
// Encode readable sensor data into the compact packet
// ---------------------------------------------------------------------------

void encodePacket(
  const ChipSatSensors::SensorData &source,
  uint16_t packetCounter,
  bool allDataFresh,
  TelemetryPacket &packet) {
  // Clear the entire packet first.
  packet = {};

  // GPS values are already supplied by the u-blox library in compact units.
  packet.latitudeE7 =
    source.gps.latitudeE7;

  packet.longitudeE7 =
    source.gps.longitudeE7;

  packet.gpsAltitudeMSLmm =
    source.gps.altitudeMSLmm;

  // Environmental altitude:
  // meters -> centimeters
  packet.environmentalAltitudeCm =
    scaleToInt32(
      source.environmental.altitudeM,
      100.0f);

  // Linear acceleration:
  // m/s^2 -> 0.01 m/s^2
  packet.accelerationCentiMps2[0] =
    scaleToInt16(
      source.imu.linearAccelerationMps2.x,
      100.0f);

  packet.accelerationCentiMps2[1] =
    scaleToInt16(
      source.imu.linearAccelerationMps2.y,
      100.0f);

  packet.accelerationCentiMps2[2] =
    scaleToInt16(
      source.imu.linearAccelerationMps2.z,
      100.0f);

  // Gyroscope:
  // rad/s -> degrees/s -> 0.1 degrees/s
  constexpr float GYRO_SCALE =
    RAD_TO_DEG * 10.0f;

  packet.gyroDeciDegPerSec[0] =
    scaleToInt16(
      source.imu.gyroscopeRadPerSec.x,
      GYRO_SCALE);

  packet.gyroDeciDegPerSec[1] =
    scaleToInt16(
      source.imu.gyroscopeRadPerSec.y,
      GYRO_SCALE);

  packet.gyroDeciDegPerSec[2] =
    scaleToInt16(
      source.imu.gyroscopeRadPerSec.z,
      GYRO_SCALE);

  // Magnetometer:
  // microtesla -> 0.1 microtesla
  packet.magneticFieldDeciMicroTesla[0] =
    scaleToInt16(
      source.imu.magnetometerMicroTesla.x,
      10.0f);

  packet.magneticFieldDeciMicroTesla[1] =
    scaleToInt16(
      source.imu.magnetometerMicroTesla.y,
      10.0f);

  packet.magneticFieldDeciMicroTesla[2] =
    scaleToInt16(
      source.imu.magnetometerMicroTesla.z,
      10.0f);

  // Quaternion:
  // -1.0 through +1.0 -> approximately -32767 through +32767
  packet.orientationQ15[0] =
    scaleToInt16(
      constrain(source.imu.orientation.i,
                -1.0f,
                1.0f),
      32767.0f);

  packet.orientationQ15[1] =
    scaleToInt16(
      constrain(source.imu.orientation.j,
                -1.0f,
                1.0f),
      32767.0f);

  packet.orientationQ15[2] =
    scaleToInt16(
      constrain(source.imu.orientation.k,
                -1.0f,
                1.0f),
      32767.0f);

  packet.orientationQ15[3] =
    scaleToInt16(
      constrain(source.imu.orientation.real,
                -1.0f,
                1.0f),
      32767.0f);

  // Temperature:
  // degrees C -> 0.01 degrees C
  packet.temperatureCentiC =
    scaleToInt16(
      source.environmental.temperatureC,
      100.0f);

  // Pressure:
  // SensorData is in Pa.
  // Divide by 10 to produce units of 0.1 hPa.
  packet.pressureDeciHpa =
    scaleToUint16(
      source.environmental.pressurePa,
      0.1f);

  // Humidity:
  // percent -> 0.01 percent
  packet.humidityCentiPercent =
    scaleToUint16(
      constrain(
        source.environmental.humidityPercent,
        0.0f,
        100.0f),
      100.0f);

  // State of charge:
  // percent -> 0.5 percent increments
  packet.cellPercentageX2 =
    scaleToUint8(
      constrain(
        source.stateOfCharge.cellPercentage,
        0.0f,
        100.0f),
      2.0f);

  packet.packetCounter = packetCounter;
  packet.CSID = CHIPSAT_ID;

  packet.sensorValidity = 0;

  if (source.imu.linearAccelerationValid) { packet.sensorValidity |= kLinearAccelerationValidBit; }
  if (source.imu.gyroscopeValid) { packet.sensorValidity |= kGyroscopeValidBit; }
  if (source.imu.magnetometerValid) { packet.sensorValidity |= kMagnetometerValidBit; }
  if (source.imu.orientationValid) { packet.sensorValidity |= kQuaternionValidBit; }
  if (source.gps.valid) { packet.sensorValidity |= kGPSValidBit; }
  if (source.stateOfCharge.valid) { packet.sensorValidity |= kStateOfChargeValidBit; }
  if (source.environmental.valid) { packet.sensorValidity |= kEnvironmentalValidBit; }

  if (allDataFresh &&
      (packet.sensorValidity & kAllSensorDataValidMask) == kAllSensorDataValidMask) {
    packet.sensorValidity |= kAllDataFreshBit;
  }

  // Calculate this last. It covers bytes 0 through 52.
  packet.crc16 = calculateCRC16(
    reinterpret_cast<const uint8_t *>(&packet),
    offsetof(TelemetryPacket, crc16));
}

// ---------------------------------------------------------------------------
// CRC-16-CCITT-FALSE
//
// Polynomial: 0x1021
// Initial value: 0xFFFF
// ---------------------------------------------------------------------------

uint16_t calculateCRC16(
  const uint8_t *data,
  size_t length) {
  uint16_t crc = 0xFFFF;

  for (size_t byteIndex = 0;
       byteIndex < length;
       byteIndex++) {
    crc ^= static_cast<uint16_t>(data[byteIndex]) << 8;

    for (uint8_t bit = 0; bit < 8; bit++) {
      if (crc & 0x8000) {
        crc = static_cast<uint16_t>(
          (crc << 1) ^ 0x1021);
      } else {
        crc = static_cast<uint16_t>(crc << 1);
      }
    }
  }

  return crc;
}

// ---------------------------------------------------------------------------
// Verify a received packet
// ---------------------------------------------------------------------------

bool packetCRCIsValid(
  const TelemetryPacket &packet) {
  uint16_t calculatedCRC = calculateCRC16(
    reinterpret_cast<const uint8_t *>(&packet),
    offsetof(TelemetryPacket, crc16));

  return calculatedCRC == packet.crc16;
}

// ---------------------------------------------------------------------------
// Print raw packet bytes
// ---------------------------------------------------------------------------

void printPacketHex(
  const TelemetryPacket &packet,
  Stream &output) {
  const uint8_t *bytes =
    reinterpret_cast<const uint8_t *>(&packet);

  output.print(F("Packet size: "));
  output.print(sizeof(packet));
  output.println(F(" bytes"));

  for (size_t i = 0; i < sizeof(packet); i++) {
    if (bytes[i] < 0x10) {
      output.print('0');
    }

    output.print(bytes[i], HEX);

    if (i < sizeof(packet) - 1) {
      output.print(' ');
    }
  }

  output.println();
}

// ---------------------------------------------------------------------------
// Print sensor validity bits
// ---------------------------------------------------------------------------

void printSensorValidity(
  uint8_t sensorValidity,
  Stream &output) {
  output.print(F("Sensor validity: 0x"));
  if (sensorValidity < 0x10) { output.print('0'); }
  output.println(sensorValidity, HEX);
  output.print(F("  Linear acceleration valid: ")); output.println((sensorValidity & kLinearAccelerationValidBit) != 0);
  output.print(F("  Gyroscope valid: ")); output.println((sensorValidity & kGyroscopeValidBit) != 0);
  output.print(F("  Magnetometer valid: ")); output.println((sensorValidity & kMagnetometerValidBit) != 0);
  output.print(F("  Quaternion valid: ")); output.println((sensorValidity & kQuaternionValidBit) != 0);
  output.print(F("  GPS valid: ")); output.println((sensorValidity & kGPSValidBit) != 0);
  output.print(F("  State of charge valid: ")); output.println((sensorValidity & kStateOfChargeValidBit) != 0);
  output.print(F("  Environmental valid: ")); output.println((sensorValidity & kEnvironmentalValidBit) != 0);
  output.print(F("  All data fresh this packet: ")); output.println((sensorValidity & kAllDataFreshBit) != 0);
}

}  // namespace ChipSatTelemetry