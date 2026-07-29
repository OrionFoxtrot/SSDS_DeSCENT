#include "includes/sensors/GPSModule.hpp"

GPSModule::GPSModule(uint8_t rxPin, uint8_t txPin)
    : gpsSerial(rxPin, txPin) // initialize SoftwareSerial with RX/TX pins
{
}

bool GPSModule::begin(long baud)
{
    gpsSerial.begin(baud);
    return true;
}

GPSData GPSModule::readData()
{
    while (gpsSerial.available() > 0)
    {
        gps.encode(gpsSerial.read());
    }

    GPSData data;

    if (gps.location.isValid())
    {
        // Convert to int32_t (microdegrees: degrees * 1,000,000)
        data.latitude = int32_t(gps.location.lat() * 1000000);
        data.longitude = int32_t(gps.location.lng() * 1000000);
        data.altitude = int32_t(gps.altitude.meters() * 100);
    }
    else
    {
        // TODO: replace with actual "dead" signal number later
        data.latitude = INT32_MAX;
        data.longitude = INT32_MAX;
        data.altitude = INT32_MAX;
    }

    if (gps.time.isValid())
    {
        data.time = gps.time.hour() * 100 + gps.time.minute(); // HHMM
    }
    else
    {
        data.time = 0xFFFF;
    }

    return data;
}

// ─── UBX helpers ────────────────────────────────────────────────────────────

bool GPSModule::readUBX(uint8_t *buf, uint8_t len, uint16_t timeout_ms)
{
    unsigned long start = millis();
    uint8_t idx = 0;

    while (millis() - start < timeout_ms)
    {
        if (gpsSerial.available())
        {
            uint8_t b = gpsSerial.read();
            if (idx == 0 && b != 0xB5)
                continue;
            if (idx == 1 && b != 0x62)
            {
                idx = 0;
                continue;
            }
            buf[idx++] = b;
            if (idx >= len)
                return true;
        }
    }
    return false;
}

void GPSModule::calcChecksum(uint8_t *buf, uint8_t len, uint8_t &ckA, uint8_t &ckB)
{
    ckA = 0;
    ckB = 0;
    for (int i = 2; i < len - 2; i++)
    {
        ckA += buf[i];
        ckB += ckA;
    }
}

// ─── Airborne mode ──────────────────────────────────────────────────────────

bool GPSModule::setAirborneMode()
{
    while (gpsSerial.available())
        gpsSerial.read();

    uint8_t poll[] = {0xB5, 0x62, 0x06, 0x24, 0x00, 0x00, 0x2A, 0x84};
    gpsSerial.write(poll, sizeof(poll));

    uint8_t buf[44];
    if (!readUBX(buf, 44, 3000))
    {
        return false;
    }

    buf[8] = 0x06; // dynamic model = Airborne <2g

    uint8_t ckA, ckB;
    calcChecksum(buf, 44, ckA, ckB);
    buf[42] = ckA;
    buf[43] = ckB;

    while (gpsSerial.available())
        gpsSerial.read();
    gpsSerial.write(buf, 44);
    return true;
}

bool GPSModule::confirmAirborneMode()
{
    while (gpsSerial.available())
        gpsSerial.read();

    uint8_t poll[] = {0xB5, 0x62, 0x06, 0x24, 0x00, 0x00, 0x2A, 0x84};
    gpsSerial.write(poll, sizeof(poll));

    uint8_t buf[44];
    if (!readUBX(buf, 44, 3000))
    {
        return false;
    }

    return buf[8] == 0x06;
}