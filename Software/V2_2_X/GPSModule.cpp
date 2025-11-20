#include "GPSModule.hpp"
#include "Constants.hpp"

GPSModule::GPSModule(uint8_t rxPin, uint8_t txPin)
    : gpsSerial(rxPin, txPin) // initialize SoftwareSerial with RX/TX pins
{
}

bool GPSModule::begin(long baud)
{
    gpsSerial.begin(baud);
    return true;
}

String GPSModule::readData()
{
    while (gpsSerial.available() > 0)
    {
        gps.encode(gpsSerial.read());
    }

    return formatInfo();
}

String GPSModule::formatInfo()
{
    String result = " ";

    if (gps.location.isValid())
    {
        result += String(gps.location.lat(), 6) + ", " + String(gps.location.lng(), 6);
    }
    else
    {
        result += "INVALID";
    }

    result += ",";
    if (gps.date.isValid())
    {
        result += String(gps.date.month()) + "/" + String(gps.date.day()) + "/" + String(gps.date.year());
    }
    else
    {
        result += "INVALID";
    }

    result += ",";
    if (gps.time.isValid())
    {
        char buf[16];
        snprintf(buf, sizeof(buf), "%02d:%02d:%02d%02d",
                 gps.time.hour(),
                 gps.time.minute(),
                 gps.time.second(),
                 gps.time.centisecond());
        result += buf;
    }
    else
    {
        result += "INVALID";
    }

    return result;
}
