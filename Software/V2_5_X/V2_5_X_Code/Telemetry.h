#pragma once

#include <Arduino.h>
#include <stddef.h>
#include "Sensors.h"

namespace ChipSatTelemetry
{

constexpr uint8_t CHIPSAT_ID = 2;

// Sensor validity bit assignments.
// Bits 0 through 6 show whether each stored measurement is valid.
// Bit 7 shows whether every required measurement was freshly collected for
// the current packet cycle.
constexpr uint8_t kLinearAccelerationValidBit = 1U << 0;
constexpr uint8_t kGyroscopeValidBit          = 1U << 1;
constexpr uint8_t kMagnetometerValidBit       = 1U << 2;
constexpr uint8_t kQuaternionValidBit         = 1U << 3;
constexpr uint8_t kGPSValidBit                = 1U << 4;
constexpr uint8_t kStateOfChargeValidBit      = 1U << 5;
constexpr uint8_t kEnvironmentalValidBit      = 1U << 6;
constexpr uint8_t kAllDataFreshBit            = 1U << 7;

constexpr uint8_t kAllSensorDataValidMask =
    kLinearAccelerationValidBit |
    kGyroscopeValidBit |
    kMagnetometerValidBit |
    kQuaternionValidBit |
    kGPSValidBit |
    kStateOfChargeValidBit |
    kEnvironmentalValidBit;

struct __attribute__((packed)) TelemetryPacket
{
  // GPS and environmental altitude: 16 bytes
  int32_t latitudeE7;
  int32_t longitudeE7;
  int32_t gpsAltitudeMSLmm;
  int32_t environmentalAltitudeCm;

  // IMU: 26 bytes
  int16_t accelerationCentiMps2[3];
  int16_t gyroDeciDegPerSec[3];
  int16_t magneticFieldDeciMicroTesla[3];
  int16_t orientationQ15[4];

  // Environmental measurements: 6 bytes
  int16_t temperatureCentiC;
  uint16_t pressureDeciHpa;
  uint16_t humidityCentiPercent;

  // Packet information: 5 bytes
  uint16_t packetCounter;
  uint8_t CSID;
  uint8_t cellPercentageX2;
  uint8_t sensorValidity;

  // CRC over the preceding 53 bytes
  uint16_t crc16;
};

static_assert(
    sizeof(TelemetryPacket) == 55,
    "TelemetryPacket must be exactly 55 bytes");

static_assert(
    offsetof(TelemetryPacket, sensorValidity) == 52,
    "Sensor validity byte must begin at byte 52");

static_assert(
    offsetof(TelemetryPacket, crc16) == 53,
    "CRC must begin at byte 53");

// Convert readable sensor data into the compact packet.
// allDataFresh should be true only when all required sensor measurements were
// successfully refreshed during the current packet cycle.
void encodePacket(
    const ChipSatSensors::SensorData &source,
    uint16_t packetCounter,
    bool allDataFresh,
    TelemetryPacket &destination);

// Calculate the packet CRC.
uint16_t calculateCRC16(
    const uint8_t *data,
    size_t length);

// Verify a received packet.
bool packetCRCIsValid(
    const TelemetryPacket &packet);

// Debug utility: print the 55 transmitted bytes in hexadecimal.
void printPacketHex(
    const TelemetryPacket &packet,
    Stream &output);

// Debug utility: print each sensor-validity bit in readable form.
void printSensorValidity(
    uint8_t sensorValidity,
    Stream &output);

} // namespace ChipSatTelemetry
