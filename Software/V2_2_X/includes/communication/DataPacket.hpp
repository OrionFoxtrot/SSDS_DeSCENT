#pragma once
#include <cstdint>
#include <cstring>
#include "includes/Constants.hpp"

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

class ChipSatPacket
{
public:
  static constexpr size_t PACKET_SIZE = 39;

  static void serialize(const DataPacket &data, uint8_t *buffer)
  {
    Print_tx_rx.println("Serialize start.");

    size_t pos = 0;
    buffer[pos++] = data.chip_id;

    // GPS (14 bytes)
    write_i32(data.gpsData.latitude, buffer, pos);
    pos += 4;
    write_i32(data.gpsData.longitude, buffer, pos);
    pos += 4;
    write_i32(data.gpsData.altitude, buffer, pos);
    pos += 4;
    write_u16(data.gpsData.time, buffer, pos);
    pos += 2;

    // IMU (18 bytes)
    write_i16(data.imuData.gyroX, buffer, pos);
    pos += 2;
    write_i16(data.imuData.gyroY, buffer, pos);
    pos += 2;
    write_i16(data.imuData.gyroZ, buffer, pos);
    pos += 2;
    write_i16(data.imuData.linX, buffer, pos);
    pos += 2;
    write_i16(data.imuData.linY, buffer, pos);
    pos += 2;
    write_i16(data.imuData.linZ, buffer, pos);
    pos += 2;
    write_i16(data.imuData.magX, buffer, pos);
    pos += 2;
    write_i16(data.imuData.magY, buffer, pos);
    pos += 2;
    write_i16(data.imuData.magZ, buffer, pos);
    pos += 2;

    // BME (6 bytes)
    write_i16(data.bmeData.temp, buffer, pos);
    pos += 2;
    write_i16(data.bmeData.pressure, buffer, pos);
    pos += 2;
    write_i16(data.bmeData.humidity, buffer, pos);
    pos += 2;

    Print_tx_rx.println("Serialize fin.");
    Print_tx_rx.println();
  }

  static void deserializeAndPrint(const uint8_t *buffer)
  {
    Print_tx_rx.println("Deserialize start.");

    size_t pos = 0;
    uint8_t chip_id = buffer[pos++];

    // GPS (14 bytes)
    int32_t lat = read_i32(buffer, pos);
    pos += 4;
    int32_t lon = read_i32(buffer, pos);
    pos += 4;
    int32_t alt = read_i32(buffer, pos);
    pos += 4;
    uint16_t gpsTime = read_u16(buffer, pos);
    pos += 2;

    // IMU (18 bytes)
    int16_t gx = read_i16(buffer, pos);
    pos += 2;
    int16_t gy = read_i16(buffer, pos);
    pos += 2;
    int16_t gz = read_i16(buffer, pos);
    pos += 2;
    int16_t lx = read_i16(buffer, pos);
    pos += 2;
    int16_t ly = read_i16(buffer, pos);
    pos += 2;
    int16_t lz = read_i16(buffer, pos);
    pos += 2;
    int16_t mx = read_i16(buffer, pos);
    pos += 2;
    int16_t my = read_i16(buffer, pos);
    pos += 2;
    int16_t mz = read_i16(buffer, pos);
    pos += 2;

    // BME (6 bytes)
    int16_t temp = read_i16(buffer, pos);
    pos += 2;
    int16_t press = read_i16(buffer, pos);
    pos += 2;
    int16_t hum = read_i16(buffer, pos);
    pos += 2;

    // --- Printing Results ---
    Print_tx_rx.println("=== Sensor Data ===");
    Print_tx_rx.printf("Chip ID: %u", chip_id);
    Print_tx_rx.printf("GPS: %ld, %ld, %ld @ %u\n",
                       lat, lon, alt, gpsTime);

    Print_tx_rx.printf("Gyro: %d, %d, %d\n",
                       gx, gy, gz);

    Print_tx_rx.printf("LinAcc: %d, %d, %d\n",
                       lx, ly, lz);

    Print_tx_rx.printf("Mag: %d, %d, %d\n",
                       mx, my, mz);

    Print_tx_rx.printf("BME: T=%d P=%d H=%d\n",
                       temp, press, hum);

    // 2. Raw Hex Dump (The "Is it actually packed?" test)
    Print_tx_rx.print("Raw Bytes (Size: ");
    Print_tx_rx.print(sizeof(DataPacket));
    Print_tx_rx.print("): ");

    for (size_t i = 0; i < sizeof(DataPacket); i++)
    {
      if (buffer[i] < 0x10)
        Print_tx_rx.print("0");
      Print_tx_rx.print(buffer[i], HEX);
      Print_tx_rx.print(" ");
    }
    Print_tx_rx.println("\n==================");

    Print_tx_rx.println("Deserialize fin.");
    Print_tx_rx.println();
  }

private:
  static void write_u16(uint16_t val, uint8_t *buf, size_t pos)
  {
    buf[pos] = val & 0xFF;
    buf[pos + 1] = (val >> 8) & 0xFF;
  }

  static void write_i16(int16_t val, uint8_t *buf, size_t pos)
  {
    write_u16(static_cast<uint16_t>(val), buf, pos);
  }

  static void write_i32(int32_t val, uint8_t *buf, size_t pos)
  {
    buf[pos] = val & 0xFF;
    buf[pos + 1] = (val >> 8) & 0xFF;
    buf[pos + 2] = (val >> 16) & 0xFF;
    buf[pos + 3] = (val >> 24) & 0xFF;
  }

  static uint16_t read_u16(const uint8_t *buf, size_t pos)
  {
    return (uint16_t)buf[pos] | ((uint16_t)buf[pos + 1] << 8);
  }

  static int16_t read_i16(const uint8_t *buf, size_t pos)
  {
    return (int16_t)read_u16(buf, pos);
  }

  static int32_t read_i32(const uint8_t *buf, size_t pos)
  {
    return (int32_t)buf[pos] |
           ((int32_t)buf[pos + 1] << 8) |
           ((int32_t)buf[pos + 2] << 16) |
           ((int32_t)buf[pos + 3] << 24);
  }
};