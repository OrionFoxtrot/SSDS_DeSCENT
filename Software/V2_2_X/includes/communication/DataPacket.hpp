#pragma once

struct __attribute__((packed)) GPSData
{
  int32_t latitude, longitude, altitude;
  uint16_t time;
};
struct __attribute__((packed)) IMUData
{
  int16_t gyroX, gyroY, gyroZ;
  int16_t linX, linY, linZ;
  int16_t magX, magY, magZ;
};
struct __attribute__((packed)) BMEData
{
  int16_t temp, pressure, humidity;
};

struct __attribute__((packed)) DataPacket
{
  uint8_t chip_id;

  // Sensor Data
  GPSData gpsData;
  IMUData imuData;
  BMEData bmeData;
};

// Verify all sizes at compile time
static_assert(sizeof(GPSData) == 14, "GPSData should be 14 bytes");
static_assert(sizeof(IMUData) == 18, "IMUData should be 18 bytes");
static_assert(sizeof(BMEData) == 6, "BMEData should be 6 bytes");
static_assert(sizeof(DataPacket) == 39, "DataPacket should be 39 bytes");