#include <stdint.h>
uint8_t packet[32]; // Example size
#define CHIP_ID 0x01

typedef struct GPS
{
  float lat;
  float GPSlong;
  float alt;
} GPSdata; // From BNO08x
typedef struct Gyro
{
  float x;
  float y;
  float z;
} GyroData; // From BME280
typedef struct HumidityData
{
  float temp;
  float approx_alt;
  float pressure;
  float humidity;
} BmeData;

void toSendBuffer(uint8_t *buffer, int send, int start, int end)
{
  int i;
  for (i = start; i <= end; i++)
  {
    buffer[i] = (send >> 8 * (i - start)) & 0xFF;
  }
}

void assemblePacket(GyroData gyro, BmeData bmeD)
{
  packet[0] = (uint8_t)CHIP_ID;
  toSendBuffer(packet, *((int *)&(gyro.x)), 1, 4);
  toSendBuffer(packet, *(int *)&(gyro.y), 5, 8);
  toSendBuffer(packet, *(int *)&(gyro.z), 9, 12);
  toSendBuffer(packet, *(int *)&(bmeD.temp), 13, 16);
  toSendBuffer(packet, *(int *)&(bmeD.approx_alt), 17, 20);
  toSendBuffer(packet, *(int *)&(bmeD.humidity), 21, 24);
  toSendBuffer(packet, *(int *)&(bmeD.pressure), 25, 28);
}

#pragma once
#include <cstdint>
#include <cstring>

struct GPSData {
  int32_t latitude, longitude, altitude;
  uint16_t time;
};

struct IMUData {
  int16_t gyroX, gyroY, gyroZ;
  int16_t linX, linY, linZ;
  int16_t magX, magY, magZ;
};

struct BMEData {
  int16_t temp, pressure, humidity;
};

struct DataPacket {
  uint8_t chip_id;
  GPSData gpsData;
  IMUData imuData;
  BMEData bmeData;
};

// Production-quality serialization
class PacketCodec {
public:
    static constexpr size_t PACKET_SIZE = 43;  // 1+14+18+6+2+2 (header+data+crc)
    
    static void serialize(const DataPacket& data, uint8_t* buffer) {
        size_t pos = 0;
        
        // Header
        buffer[pos++] = 0xA5;  // Sync byte
        buffer[pos++] = 0x01;  // Version
        buffer[pos++] = data.chip_id;
        
        // GPS (14 bytes)
        write_i32(buffer, pos, data.gpsData.latitude);  pos += 4;
        write_i32(buffer, pos, data.gpsData.longitude); pos += 4;
        write_i32(buffer, pos, data.gpsData.altitude);  pos += 4;
        write_u16(buffer, pos, data.gpsData.time);      pos += 2;
        
        // IMU (18 bytes)
        write_i16(buffer, pos, data.imuData.gyroX); pos += 2;
        write_i16(buffer, pos, data.imuData.gyroY); pos += 2;
        write_i16(buffer, pos, data.imuData.gyroZ); pos += 2;
        write_i16(buffer, pos, data.imuData.linX);  pos += 2;
        write_i16(buffer, pos, data.imuData.linY);  pos += 2;
        write_i16(buffer, pos, data.imuData.linZ);  pos += 2;
        write_i16(buffer, pos, data.imuData.magX);  pos += 2;
        write_i16(buffer, pos, data.imuData.magY);  pos += 2;
        write_i16(buffer, pos, data.imuData.magZ);  pos += 2;
        
        // BME (6 bytes)
        write_i16(buffer, pos, data.bmeData.temp);     pos += 2;
        write_i16(buffer, pos, data.bmeData.pressure); pos += 2;
        write_i16(buffer, pos, data.bmeData.humidity); pos += 2;
        
        // CRC
        uint16_t crc = calculate_crc16(buffer, pos);
        write_u16(buffer, pos, crc);
    }
    
    static bool deserialize(const uint8_t* buffer, size_t len, DataPacket& data) {
        if (len < PACKET_SIZE) return false;
        
        size_t pos = 0;
        
        // Validate header
        if (buffer[pos++] != 0xA5) return false;  // Bad sync
        if (buffer[pos++] != 0x01) return false;  // Wrong version
        
        // Verify CRC
        uint16_t recv_crc = read_u16(buffer, PACKET_SIZE - 2);
        uint16_t calc_crc = calculate_crc16(buffer, PACKET_SIZE - 2);
        if (recv_crc != calc_crc) return false;  // Corrupted!
        
        // Parse data
        data.chip_id = buffer[pos++];
        
        // GPS
        data.gpsData.latitude = read_i32(buffer, pos);  pos += 4;
        data.gpsData.longitude = read_i32(buffer, pos); pos += 4;
        data.gpsData.altitude = read_i32(buffer, pos);  pos += 4;
        data.gpsData.time = read_u16(buffer, pos);      pos += 2;
        
        // IMU
        data.imuData.gyroX = read_i16(buffer, pos); pos += 2;
        data.imuData.gyroY = read_i16(buffer, pos); pos += 2;
        data.imuData.gyroZ = read_i16(buffer, pos); pos += 2;
        data.imuData.linX = read_i16(buffer, pos);  pos += 2;
        data.imuData.linY = read_i16(buffer, pos);  pos += 2;
        data.imuData.linZ = read_i16(buffer, pos);  pos += 2;
        data.imuData.magX = read_i16(buffer, pos);  pos += 2;
        data.imuData.magY = read_i16(buffer, pos);  pos += 2;
        data.imuData.magZ = read_i16(buffer, pos);  pos += 2;
        
        // BME
        data.bmeData.temp = read_i16(buffer, pos);     pos += 2;
        data.bmeData.pressure = read_i16(buffer, pos); pos += 2;
        data.bmeData.humidity = read_i16(buffer, pos); pos += 2;
        
        return true;
    }

private:
    static void write_u16(uint8_t* buf, size_t pos, uint16_t val) {
        buf[pos] = val & 0xFF;
        buf[pos+1] = (val >> 8) & 0xFF;
    }
    
    static void write_i16(uint8_t* buf, size_t pos, int16_t val) {
        write_u16(buf, pos, static_cast<uint16_t>(val));
    }
    
    static void write_i32(uint8_t* buf, size_t pos, int32_t val) {
        buf[pos] = val & 0xFF;
        buf[pos+1] = (val >> 8) & 0xFF;
        buf[pos+2] = (val >> 16) & 0xFF;
        buf[pos+3] = (val >> 24) & 0xFF;
    }
    
    static uint16_t read_u16(const uint8_t* buf, size_t pos) {
        return buf[pos] | (buf[pos+1] << 8);
    }
    
    static int16_t read_i16(const uint8_t* buf, size_t pos) {
        return static_cast<int16_t>(read_u16(buf, pos));
    }
    
    static int32_t read_i32(const uint8_t* buf, size_t pos) {
        return buf[pos] | (buf[pos+1] << 8) | (buf[pos+2] << 16) | (buf[pos+3] << 24);
    }
    
    static uint16_t calculate_crc16(const uint8_t* data, size_t len) {
        uint16_t crc = 0xFFFF;
        for (size_t i = 0; i < len; i++) {
            crc ^= static_cast<uint16_t>(data[i]) << 8;
            for (uint8_t bit = 0; bit < 8; bit++) {
                crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : crc << 1;
            }
        }
        return crc;
    }
};