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