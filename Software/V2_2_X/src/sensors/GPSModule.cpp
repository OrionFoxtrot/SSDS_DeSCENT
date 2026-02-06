#include "includes/sensors/GPSModule.hpp"
#include "includes/Constants.hpp"

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